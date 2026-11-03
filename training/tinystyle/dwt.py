# -*- coding: utf-8 -*-
"""Haar wavelet decomposition, used only as a *training* loss.

Compressed generators lose exactly the sub-band energy that carries brush
strokes, edges and fine texture, so the high-frequency distillation term
compares LH/HL/HH between the teacher and the student output.  The transform is
never part of a deployed model.
"""

from __future__ import annotations

import torch
import torch.nn.functional as F

_SQRT1_2 = 0.7071067811865476

#: Analysis filters, orthonormal Haar, 2x2.
_FILTERS = {
    "LL": ((1.0, 1.0), (1.0, 1.0)),
    "LH": ((1.0, 1.0), (-1.0, -1.0)),
    "HL": ((1.0, -1.0), (1.0, -1.0)),
    "HH": ((1.0, -1.0), (-1.0, 1.0)),
}


def _kernels(channels: int, device, dtype):
    out = {}
    for name, k in _FILTERS.items():
        w = torch.tensor(k, device=device, dtype=dtype) * 0.5
        # (C_out=channels, C_in=1, 2, 2) -> depthwise
        out[name] = w.reshape(1, 1, 2, 2).expand(channels, 1, 2, 2).contiguous()
    return out


def haar2d(x: torch.Tensor, levels: int = 1):
    """Return ``[(LL, LH, HL, HH), ...]``, one tuple per decomposition level."""
    if x.dim() != 4:
        raise ValueError("expected NCHW")
    out = []
    cur = x
    for _ in range(levels):
        n, c, h, w = cur.shape
        pad_h = h % 2
        pad_w = w % 2
        if pad_h or pad_w:
            cur = F.pad(cur, (0, pad_w, 0, pad_h), mode="replicate")
        k = _kernels(c, cur.device, cur.dtype)
        bands = {}
        for name, weight in k.items():
            bands[name] = F.conv2d(cur, weight, stride=2, groups=c)
        out.append((bands["LL"], bands["LH"], bands["HL"], bands["HH"]))
        cur = bands["LL"]
    return out


def high_frequency_bands(x: torch.Tensor, levels: int = 1):
    """Just the detail bands, flattened across levels."""
    result = []
    for _ll, lh, hl, hh in haar2d(x, levels):
        result.extend([lh, hl, hh])
    return result
