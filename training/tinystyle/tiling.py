# -*- coding: utf-8 -*-
"""Tiled whole-image inference.

Once every normalisation is a fixed per-channel affine map -- which is the case
for the teacher's ``InstanceNorm2d(track_running_stats=True)`` at eval, and for
every exported student -- each output pixel depends only on a bounded
neighbourhood of the input.  The image can then be processed in tiles with a
``halo`` margin: the trustworthy centre of each tile is kept and the halo, whose
values are polluted by reflection padding at the tile edge, is discarded.  The
result matches whole-image inference while keeping memory flat.

The teacher's receptive field at 256x256 scale is 115 pixels, so the default
halo of 128 is sufficient.
"""

from __future__ import annotations

import torch
import torch.nn.functional as F


def tile_starts(length: int, tile: int, halo: int):
    """Offsets of tiles covering ``[0, length)``."""
    step = tile - 2 * halo
    if step <= 0:
        raise ValueError("tile (%d) must exceed 2*halo (%d)" % (tile, 2 * halo))
    starts = list(range(0, max(length - tile, 0) + 1, step)) or [0]
    if starts[-1] + tile < length:
        starts.append(length - tile)
    return starts


def trustworthy(start: int, tile: int, halo: int, length: int):
    """Half-open sub-range of a tile that is free of tile-edge artefacts."""
    inner_lo = start + halo if start > 0 else 0
    inner_hi = start + tile - halo if start + tile < length else length
    return inner_lo, inner_hi


def pad_to_min(tensor: torch.Tensor, minimum: int):
    """Reflect-pad so both spatial dims are >= ``minimum`` and a multiple of 4."""
    h, w = tensor.shape[-2:]
    target_h = -(-max(h, minimum) // 4) * 4
    target_w = -(-max(w, minimum) // 4) * 4
    if (target_h, target_w) != (h, w):
        tensor = F.pad(tensor, (0, target_w - w, 0, target_h - h), mode="reflect")
    return tensor


def stylize_tiled(model, tensor: torch.Tensor, tile: int = 1024, halo: int = 128):
    """Style-transfer a ``1x3xHxW`` tensor in 0..255.  Returns the same shape."""
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
                out[:, :, vy0:vy1, vx0:vx1] = result[:, :, vy0 - y0:vy1 - y0,
                                                      vx0 - x0:vx1 - x0]
    return out
