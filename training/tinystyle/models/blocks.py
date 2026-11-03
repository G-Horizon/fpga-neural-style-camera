# -*- coding: utf-8 -*-
"""Building blocks shared by the Candy teacher and the compressed students.

Tensor convention
-----------------
Images are float32 ``NCHW`` RGB tensors in **0..255**.  That is the convention
the ``candy.pth`` checkpoint was trained with: feeding 0..1 produces garbage
(verified -- MSE against the reference stylisation is 6884 for 0..1 versus 157
for 0..255).

Padding
-------
Every convolution is preceded by ``ReflectionPad2d(k // 2)``, exactly as in the
2017 ``fast_neural_style`` release, so teacher and students share one border
behaviour.

Normalisation
-------------
``norm`` picks how a channel is normalised:

``instance``
    ``InstanceNorm2d(affine=True, track_running_stats=True)``: statistics are
    taken per image while training and the accumulated *running* statistics are
    used at inference.  This is precisely what the candy teacher does, and it
    is what makes tiled / streaming inference exact -- at deployment the layer
    is a fixed per-channel affine map, not a data-dependent one.
``fixed``
    ``y = gamma * (x - mu) / sqrt(var + eps) + beta`` with ``mu``/``var`` frozen
    buffers.  Only ``gamma``/``beta`` are learned.
``affine``
    ``y = a * x + b`` -- the fully folded limit, no normalisation left.
``none``
    Identity (used for the output convolution).

``fixed`` and ``affine`` are the same map at inference; they differ only in
parameterisation, which is the point of the stage-3 ablation.
"""

from __future__ import annotations

import torch
import torch.nn as nn
import torch.nn.functional as F

NORM_KINDS = ("instance", "fixed", "affine", "none")


def _chshape(x: torch.Tensor, c: int):
    return (1, c) + (1,) * (x.dim() - 2)


class FixedStatNorm2d(nn.Module):
    """Per-channel normalisation with statistics that are never taken from the
    current input.

    The affine map always uses the ``mean``/``var`` buffers.  While training and
    ``momentum > 0`` those buffers follow an exponential moving average of the
    batch statistics, which is how they reach sensible values without ever
    becoming data-dependent at inference.  Set ``momentum=0`` (or call
    :meth:`freeze_stats`) to hold them completely still, which is what the
    stage-3 ablation and every export do.
    """

    def __init__(self, num_features: int, eps: float = 1e-5, momentum: float = 0.05):
        super().__init__()
        self.num_features = num_features
        self.eps = eps
        self.momentum = momentum
        self.weight = nn.Parameter(torch.ones(num_features))
        self.bias = nn.Parameter(torch.zeros(num_features))
        self.register_buffer("mean", torch.zeros(num_features))
        self.register_buffer("var", torch.ones(num_features))

    @torch.no_grad()
    def load_stats(self, mean, var):
        self.mean.copy_(torch.as_tensor(mean).detach().reshape(-1).float())
        self.var.copy_(torch.as_tensor(var).detach().reshape(-1).float().clamp_min(0.0))
        return self

    @torch.no_grad()
    def freeze_stats(self):
        self.momentum = 0.0
        return self

    def forward(self, x):
        if self.training and self.momentum > 0:
            with torch.no_grad():
                m = x.mean(dim=(0, 2, 3))
                v = x.var(dim=(0, 2, 3), unbiased=False)
                self.mean.mul_(1.0 - self.momentum).add_(m * self.momentum)
                self.var.mul_(1.0 - self.momentum).add_(v * self.momentum)
        scale = self.weight * torch.rsqrt(self.var + self.eps)
        shift = self.bias - self.mean * scale
        return x * scale.reshape(_chshape(x, -1)) + shift.reshape(_chshape(x, -1))

    @torch.no_grad()
    def folded_affine(self):
        """Return ``(scale, shift)`` such that ``y = scale * x + shift``."""
        scale = self.weight * torch.rsqrt(self.var + self.eps)
        shift = self.bias - self.mean * scale
        return scale.detach(), shift.detach()


class AffineNorm2d(nn.Module):
    """``y = weight * x + bias``; a normalisation that is already folded."""

    def __init__(self, num_features: int):
        super().__init__()
        self.num_features = num_features
        self.weight = nn.Parameter(torch.ones(num_features))
        self.bias = nn.Parameter(torch.zeros(num_features))

    def forward(self, x):
        return x * self.weight.reshape(_chshape(x, -1)) + self.bias.reshape(_chshape(x, -1))

    @torch.no_grad()
    def folded_affine(self):
        return self.weight.detach(), self.bias.detach()


def make_norm(kind: str, channels: int, eps: float = 1e-5,
              momentum: float = 0.05) -> nn.Module:
    if kind == "instance":
        return nn.InstanceNorm2d(channels, affine=True, eps=eps, track_running_stats=True)
    if kind == "fixed":
        return FixedStatNorm2d(channels, eps=eps, momentum=momentum)
    if kind == "affine":
        return AffineNorm2d(channels)
    if kind == "none":
        return nn.Identity()
    raise ValueError("unknown norm kind %r (expected one of %s)" % (kind, NORM_KINDS))


def folded_affine_of(norm: nn.Module):
    """``(scale, shift)`` for any foldable normalisation this project emits.

    Returns ``None`` for a data-dependent InstanceNorm (``track_running_stats``
    off), which by construction cannot be folded into a convolution.
    """
    if isinstance(norm, (FixedStatNorm2d, AffineNorm2d)):
        return norm.folded_affine()
    if isinstance(norm, nn.InstanceNorm2d):
        if not norm.track_running_stats:
            return None
        rm, rv = norm.running_mean, norm.running_var
        if rm is None or rv is None:
            return None
        scale = norm.weight * torch.rsqrt(rv + norm.eps)
        shift = norm.bias - rm * scale
        return scale.detach(), shift.detach()
    return None


class ConvLayer(nn.Module):
    """Reflection pad -> conv -> norm -> optional ReLU.

    Attribute names (``reflection_pad``, ``conv2d``) match the 2017 release so
    the candy checkpoint loads verbatim.
    """

    def __init__(self, in_channels, out_channels, kernel_size, stride=1,
                 norm="none", act=False, bias=True, norm_momentum=0.05):
        super().__init__()
        self.in_channels = in_channels
        self.out_channels = out_channels
        self.kernel_size = kernel_size
        self.stride = stride
        self.reflection_pad = nn.ReflectionPad2d(kernel_size // 2)
        self.conv2d = nn.Conv2d(in_channels, out_channels, kernel_size, stride, bias=bias)
        self.norm = make_norm(norm, out_channels, momentum=norm_momentum)
        self.act = nn.ReLU(inplace=False) if act else nn.Identity()

    def forward(self, x):
        return self.act(self.norm(self.conv2d(self.reflection_pad(x))))


class ResidualBlock(nn.Module):
    """``x + conv2(conv1(x))``; no activation after the add (as in the teacher)."""

    def __init__(self, channels, norm="instance", kernel_size=3, norm_momentum=0.05):
        super().__init__()
        self.conv1 = ConvLayer(channels, channels, kernel_size, 1, norm=norm, act=True,
                               norm_momentum=norm_momentum)
        self.conv2 = ConvLayer(channels, channels, kernel_size, 1, norm=norm, act=False,
                               norm_momentum=norm_momentum)

    def forward(self, x):
        return x + self.conv2(self.conv1(x))


class BottleneckBlock(nn.Module):
    """``x + conv3(conv2(conv1(x)))`` with 1x1 -> kxk -> 1x1 dense convolutions.

    All three are dense, which is what the Ti60 PE array is built for.  With
    ``inner = r * channels`` the pair of 3x3 maps costs ``(2r + 9 r^2) / 18`` of
    a plain two-3x3 block -- 5.5x less at ``r = 1/2``.
    """

    def __init__(self, channels, inner, norm="instance", kernel_size=3,
                 norm_momentum=0.05):
        super().__init__()
        self.conv1 = ConvLayer(channels, inner, 1, 1, norm=norm, act=True,
                               norm_momentum=norm_momentum)
        self.conv2 = ConvLayer(inner, inner, kernel_size, 1, norm=norm, act=True,
                               norm_momentum=norm_momentum)
        self.conv3 = ConvLayer(inner, channels, 1, 1, norm=norm, act=False,
                               norm_momentum=norm_momentum)

    def forward(self, x):
        return x + self.conv3(self.conv2(self.conv1(x)))


class UpsampleConv(nn.Module):
    """Nearest upsample -> reflection pad -> conv (the teacher's decoder stage).

    Flagged as a MAC unit so the accounting can charge it either way: the
    canonical convention credits it as the transposed convolution it is
    *equivalent in cost to*, while ``convention="native"`` charges the
    convolution that actually runs on the upsampled grid -- four times as much.

    This is also the form the FPGA reference implements, so a student that keeps
    it needs no new datapath.  See ``docs/DECONV_CONTRACT.md``.
    """

    MAC_UNIT = True

    def __init__(self, in_channels, out_channels, kernel_size, upsample=2):
        super().__init__()
        self.in_channels = in_channels
        self.out_channels = out_channels
        self.kernel_size = kernel_size
        self.upsample = upsample
        self.reflection_pad = nn.ReflectionPad2d(kernel_size // 2)
        self.conv2d = nn.Conv2d(in_channels, out_channels, kernel_size, 1)

    def forward(self, x):
        if self.upsample:
            x = F.interpolate(x, mode="nearest", scale_factor=self.upsample)
        return self.conv2d(self.reflection_pad(x))


class UpConvTranspose(nn.Module):
    """Stride-2 transposed convolution -- the cheap learned upsample.

    A ``nearest + kxk conv`` upsample spends ``k*k*Cin*Cout`` per *output*
    pixel; the transposed form spends it per *input* pixel, i.e. 4x less at
    stride 2.  The canonical MAC accounting credits the transposed form, so this
    is the only upsample whose executed MAC equals its credited MAC.

    It is *not* the operator the teacher uses, and not the operator the FPGA
    reference implements today.  Whether the hardware adopts it is a separate
    decision with a real cost -- see ``docs/DECONV_CONTRACT.md``.
    """

    def __init__(self, in_channels, out_channels, kernel_size, stride=2):
        super().__init__()
        if kernel_size % 2 != 1:
            raise ValueError("transposed-conv kernel must be odd")
        self.in_channels = in_channels
        self.out_channels = out_channels
        self.kernel_size = kernel_size
        self.stride = stride
        self.conv2d = nn.ConvTranspose2d(in_channels, out_channels, kernel_size,
                                         stride=stride, padding=kernel_size // 2,
                                         output_padding=1)

    def forward(self, x):
        return self.conv2d(x)


class UpBlock(nn.Module):
    """Decoder stage: learned x2 upsample -> norm -> optional ReLU."""

    def __init__(self, in_channels, out_channels, kernel_size=3, stride=2,
                 norm="instance", act=True, method="transpose", norm_momentum=0.05):
        super().__init__()
        self.method = method
        self.in_channels = in_channels
        self.out_channels = out_channels
        self.kernel_size = kernel_size
        self.stride = stride
        if method == "transpose":
            self.conv = UpConvTranspose(in_channels, out_channels, kernel_size, stride)
        elif method == "resize":
            self.conv = UpsampleConv(in_channels, out_channels, kernel_size, stride)
        else:
            raise ValueError("unknown upsample method %r" % (method,))
        self.norm = make_norm(norm, out_channels, momentum=norm_momentum)
        self.act = nn.ReLU(inplace=False) if act else nn.Identity()

    def forward(self, x):
        return self.act(self.norm(self.conv(x)))
