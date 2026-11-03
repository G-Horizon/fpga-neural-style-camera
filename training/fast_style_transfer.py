# -*- coding: utf-8 -*-
"""Fast neural style transfer inference (Johnson et al., 2016).

Applies a pre-trained transformer network -- the PyTorch
``examples/fast_neural_style`` architecture released in 2017 -- to every image
in a folder.

Tiled inference
---------------
In eval mode the network's InstanceNorm2d layers use their stored running
statistics, i.e. a fixed per-channel affine transform, so every layer is a
local convolution and the output at a pixel depends only on a bounded
neighbourhood of the input.  That lets the image be processed in tiles with a
``halo`` margin: each tile's trustworthy centre is written out and the halo
(whose values are polluted by reflection padding at the tile edge) is
discarded.  The result matches whole-image inference while keeping GPU memory
flat, which matters because the source photos are 12 MP.

Colour
------
The network expects sRGB.  Photos tagged with another profile (these are
Display P3) are converted first, and the output is written with an sRGB
profile, so the stylised image has the same colour appearance as the original.
Pass --keep-profile to feed the raw pixel values instead.

Usage
-----
    python fast_style_transfer.py --input picture/original --output picture/candy
"""

import argparse
import io
import os
import sys
import time

import torch
import torch.nn as nn
import torch.nn.functional as F
import torchvision.transforms as transforms
from PIL import Image, ImageCms


# --------------------------------------------------------------------------
# Network definition (matches the 2017 fast_neural_style release)
# --------------------------------------------------------------------------

class ConvLayer(nn.Module):
    def __init__(self, in_channels, out_channels, kernel_size, stride):
        super().__init__()
        self.reflection_pad = nn.ReflectionPad2d(kernel_size // 2)
        self.conv2d = nn.Conv2d(in_channels, out_channels, kernel_size, stride)

    def forward(self, x):
        return self.conv2d(self.reflection_pad(x))


class ResidualBlock(nn.Module):
    def __init__(self, channels):
        super().__init__()
        self.conv1 = ConvLayer(channels, channels, kernel_size=3, stride=1)
        self.in1 = nn.InstanceNorm2d(channels, affine=True, track_running_stats=True)
        self.conv2 = ConvLayer(channels, channels, kernel_size=3, stride=1)
        self.in2 = nn.InstanceNorm2d(channels, affine=True, track_running_stats=True)
        self.relu = nn.ReLU()

    def forward(self, x):
        out = self.relu(self.in1(self.conv1(x)))
        out = self.in2(self.conv2(out))
        return out + x


class UpsampleConvLayer(nn.Module):
    def __init__(self, in_channels, out_channels, kernel_size, stride, upsample=None):
        super().__init__()
        self.upsample = upsample
        self.reflection_pad = nn.ReflectionPad2d(kernel_size // 2)
        self.conv2d = nn.Conv2d(in_channels, out_channels, kernel_size, stride)

    def forward(self, x):
        if self.upsample:
            x = F.interpolate(x, mode='nearest', scale_factor=self.upsample)
        return self.conv2d(self.reflection_pad(x))


class TransformerNet(nn.Module):
    def __init__(self):
        super().__init__()
        self.conv1 = ConvLayer(3, 32, kernel_size=9, stride=1)
        self.in1 = nn.InstanceNorm2d(32, affine=True, track_running_stats=True)
        self.conv2 = ConvLayer(32, 64, kernel_size=3, stride=2)
        self.in2 = nn.InstanceNorm2d(64, affine=True, track_running_stats=True)
        self.conv3 = ConvLayer(64, 128, kernel_size=3, stride=2)
        self.in3 = nn.InstanceNorm2d(128, affine=True, track_running_stats=True)

        self.res1 = ResidualBlock(128)
        self.res2 = ResidualBlock(128)
        self.res3 = ResidualBlock(128)
        self.res4 = ResidualBlock(128)
        self.res5 = ResidualBlock(128)

        self.deconv1 = UpsampleConvLayer(128, 64, kernel_size=3, stride=1, upsample=2)
        self.in4 = nn.InstanceNorm2d(64, affine=True, track_running_stats=True)
        self.deconv2 = UpsampleConvLayer(64, 32, kernel_size=3, stride=1, upsample=2)
        self.in5 = nn.InstanceNorm2d(32, affine=True, track_running_stats=True)
        self.deconv3 = ConvLayer(32, 3, kernel_size=9, stride=1)

        self.relu = nn.ReLU()

    def forward(self, x):
        y = self.relu(self.in1(self.conv1(x)))
        y = self.relu(self.in2(self.conv2(y)))
        y = self.relu(self.in3(self.conv3(y)))
        y = self.res1(y)
        y = self.res2(y)
        y = self.res3(y)
        y = self.res4(y)
        y = self.res5(y)
        y = self.relu(self.in4(self.deconv1(y)))
        y = self.relu(self.in5(self.deconv2(y)))
        y = self.deconv3(y)
        return y


def load_model(checkpoint, device):
    model = TransformerNet()
    state = torch.load(checkpoint, map_location='cpu')
    if isinstance(state, dict) and 'state_dict' in state:
        state = state['state_dict']
    model.load_state_dict(state)          # strict: the checkpoint must match
    return model.to(device).eval()


# --------------------------------------------------------------------------
# Tiling
# --------------------------------------------------------------------------

def tile_starts(length, tile, halo):
    """Offsets of tiles covering [0, length).

    Every tile is ``tile`` wide; its trustworthy region starts ``halo`` inside
    it (except at an image border).  The trustworthy regions of consecutive
    tiles abut exactly, so they partition [0, length).
    """
    step = tile - 2 * halo
    starts = list(range(0, max(length - tile, 0) + 1, step)) or [0]
    if starts[-1] + tile < length:        # final sliver: shift a full tile back
        starts.append(length - tile)
    return starts


def trustworthy(start, tile, halo, length):
    """Half-open sub-range of a tile that is free of tile-edge artefacts."""
    inner_lo = start + halo if start > 0 else 0
    inner_hi = start + tile - halo if start + tile < length else length
    return inner_lo, inner_hi


def pad_to_min(tensor, minimum):
    """Reflect-pad so both spatial dims are >= minimum and a multiple of 4."""
    h, w = tensor.shape[-2:]
    target_h = -(-max(h, minimum) // 4) * 4
    target_w = -(-max(w, minimum) // 4) * 4
    if (target_h, target_w) != (h, w):
        tensor = F.pad(tensor, (0, target_w - w, 0, target_h - h), mode='reflect')
    return tensor


def stylize(model, tensor, tile, halo):
    """Style-transfer a 1x3xHxW tensor in 0..255, tiled. Returns same shape."""
    tensor = pad_to_min(tensor, tile)
    _, _, height, width = tensor.shape
    out = torch.empty_like(tensor)
    with torch.inference_mode():
        for y0 in tile_starts(height, tile, halo):
            vy0, vy1 = trustworthy(y0, tile, halo, height)
            for x0 in tile_starts(width, tile, halo):
                vx0, vx1 = trustworthy(x0, tile, halo, width)
                patch = tensor[:, :, y0:y0 + tile, x0:x0 + tile]
                result = model(patch)
                out[:, :, vy0:vy1, vx0:vx1] = result[
                    :, :, vy0 - y0:vy1 - y0, vx0 - x0:vx1 - x0]
    return out


# --------------------------------------------------------------------------
# Image I/O
# --------------------------------------------------------------------------

def load_rgb(path, keep_profile):
    """Open and convert to sRGB unless the image already is sRGB."""
    with Image.open(path) as im:
        im = im.convert('RGB')
        icc = im.info.get('icc_profile')
        if icc and not keep_profile:
            src = ImageCms.ImageCmsProfile(io.BytesIO(icc))
            if 'srgb' not in ImageCms.getProfileDescription(src).lower():
                im = ImageCms.profileToProfile(
                    im, src, ImageCms.createProfile('sRGB'),
                    renderingIntent=ImageCms.Intent.RELATIVE_COLORIMETRIC,
                    outputMode='RGB')
    return im


def main(argv=None):
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--input', required=True, help='folder of source images')
    parser.add_argument('--output', required=True, help='folder for stylised images')
    parser.add_argument('--checkpoint', required=True, help='style weights')
    parser.add_argument('--tile', type=int, default=1024,
                        help='tile size, multiple of 4 (default 1024)')
    parser.add_argument('--halo', type=int, default=128,
                        help='discarded margin per tile side (default 128)')
    parser.add_argument('--quality', type=int, default=95, help='JPEG quality')
    parser.add_argument('--keep-profile', action='store_true',
                        help='do not convert non-sRGB input to sRGB')
    parser.add_argument('--device', default=None, help='cuda / cpu (default: auto)')
    args = parser.parse_args(argv)

    if args.tile % 4 or args.halo % 4 or args.halo * 2 >= args.tile:
        parser.error('--tile and --halo must be multiples of 4, with 2*halo < tile')

    device = torch.device(
        args.device or ('cuda' if torch.cuda.is_available() else 'cpu'))
    model = load_model(args.checkpoint, device)
    print('loaded %s on %s' % (args.checkpoint, device), flush=True)

    names = sorted(f for f in os.listdir(args.input)
                   if f.lower().endswith(('.jpg', '.jpeg', '.png', '.bmp', '.webp')))
    if not names:
        parser.error('no images in %s' % args.input)
    os.makedirs(args.output, exist_ok=True)

    srgb_icc = ImageCms.ImageCmsProfile(ImageCms.createProfile('sRGB')).tobytes()
    to_tensor = transforms.ToTensor()

    for index, name in enumerate(names, 1):
        src = os.path.join(args.input, name)
        dst = os.path.join(args.output, name)
        started = time.time()

        image = load_rgb(src, args.keep_profile)
        width, height = image.size
        tensor = to_tensor(image).mul(255).unsqueeze(0).to(device)

        result = stylize(model, tensor, args.tile, args.halo)
        result = result[:, :, :height, :width].clamp(0, 255)
        array = result[0].permute(1, 2, 0).cpu().numpy().astype('uint8')

        Image.fromarray(array).save(
            dst, quality=args.quality, subsampling=0,
            icc_profile=None if args.keep_profile else srgb_icc)
        print('[%d/%d] %s  %dx%d  %.1fs  -> %s'
              % (index, len(names), name, width, height,
                 time.time() - started, dst), flush=True)

    print('done')


if __name__ == '__main__':
    sys.exit(main())
