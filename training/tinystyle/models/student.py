# -*- coding: utf-8 -*-
"""Configurable student network.

The whole family described in the project brief is one parameterised model::

    head (cK s1) -> downsample xN (3x3 s2) -> trunk xM -> upsample xN -> tail (cK s1)

with the trunk either plain (``3x3 -> 3x3 + skip``) or bottleneck
(``1x1 -> 3x3 -> 1x1 + skip``), dense throughout.

Every student implements :meth:`tap_modules` with the same logical names as the
teacher, so the feature-distillation loss needs no per-architecture plumbing.
"""

from __future__ import annotations

from collections import OrderedDict
from dataclasses import asdict, dataclass

import torch.nn as nn

from .blocks import BottleneckBlock, ConvLayer, ResidualBlock, UpBlock, make_norm
from .teacher import TAP_NAMES  # noqa: F401  (re-exported for callers)

TRUNK_KINDS = ("plain", "bottleneck")
UPSAMPLE_METHODS = ("transpose", "resize")


@dataclass
class StudentSpec:
    """Architecture description; serialised verbatim into ``model_spec.json``."""

    name: str = "student"
    head_kernel: int = 5
    head_channels: int = 16
    downs: tuple = (32, 64)
    trunk_kind: str = "plain"
    trunk_channels: int = 64
    trunk_blocks: int = 5
    trunk_kernel: int = 3
    bottleneck_ratio: float = 0.5
    ups: tuple = (32, 16)
    tail_kernel: int = 5
    norm: str = "instance"
    upsample: str = "transpose"
    fixed_momentum: float = 0.05

    def __post_init__(self):
        self.downs = tuple(int(c) for c in self.downs)
        self.ups = tuple(int(c) for c in self.ups)
        if self.trunk_kind not in TRUNK_KINDS:
            raise ValueError("trunk_kind must be one of %s" % (TRUNK_KINDS,))
        if self.upsample not in UPSAMPLE_METHODS:
            raise ValueError("upsample must be one of %s" % (UPSAMPLE_METHODS,))
        if not self.downs:
            raise ValueError("at least one downsample stage is required")
        if len(self.ups) != len(self.downs):
            raise ValueError("downs and ups must have the same length (%d vs %d)"
                             % (len(self.downs), len(self.ups)))
        if self.downs[-1] != self.trunk_channels:
            raise ValueError("last downsample channel %d must equal trunk_channels %d"
                             % (self.downs[-1], self.trunk_channels))
        if self.trunk_blocks < 1:
            raise ValueError("trunk_blocks must be >= 1")

    @property
    def bottleneck_inner(self) -> int:
        return max(1, int(round(self.trunk_channels * self.bottleneck_ratio)))

    @property
    def scale(self) -> int:
        """Total downsampling factor of the trunk."""
        return 2 ** len(self.downs)

    def arch_string(self) -> str:
        """Compact form, e.g. ``c5s1-16 d32 d64 R64x5 u32 u16 c5s1-3``."""
        parts = ["c%ds1-%d" % (self.head_kernel, self.head_channels)]
        parts += ["d%d" % c for c in self.downs]
        if self.trunk_kind == "plain":
            parts.append("R%d x%d k%d" % (self.trunk_channels, self.trunk_blocks,
                                          self.trunk_kernel))
        else:
            parts.append("B%d>%d>%d x%d" % (self.trunk_channels, self.bottleneck_inner,
                                            self.trunk_channels, self.trunk_blocks))
        parts += ["u%d" % c for c in self.ups]
        parts.append("c%ds1-3" % self.tail_kernel)
        return " ".join(parts)

    def to_dict(self) -> dict:
        d = asdict(self)
        d["downs"] = list(self.downs)
        d["ups"] = list(self.ups)
        d["bottleneck_inner"] = self.bottleneck_inner
        d["arch_string"] = self.arch_string()
        d["trunk_scale"] = self.scale
        return d


def spec_from_dict(d: dict) -> StudentSpec:
    """Build a spec from a config mapping; unknown keys are an error."""
    known = set(StudentSpec.__dataclass_fields__)
    unknown = set(d) - known
    if unknown:
        raise ValueError("unknown student spec keys: %s" % sorted(unknown))
    return StudentSpec(**d)


class StudentNet(nn.Module):
    def __init__(self, spec: StudentSpec):
        super().__init__()
        self.spec = spec
        norm = spec.norm
        momentum = spec.fixed_momentum

        self.head = ConvLayer(3, spec.head_channels, spec.head_kernel, 1,
                              norm=norm, act=True, norm_momentum=momentum)

        downs = []
        cin = spec.head_channels
        for c in spec.downs:
            downs.append(ConvLayer(cin, c, 3, 2, norm=norm, act=True,
                                   norm_momentum=momentum))
            cin = c
        self.downs = nn.ModuleList(downs)

        trunk = []
        for _ in range(spec.trunk_blocks):
            if spec.trunk_kind == "plain":
                trunk.append(ResidualBlock(spec.trunk_channels, norm=norm,
                                           kernel_size=spec.trunk_kernel,
                                           norm_momentum=momentum))
            else:
                trunk.append(BottleneckBlock(spec.trunk_channels, spec.bottleneck_inner,
                                             norm=norm, kernel_size=spec.trunk_kernel,
                                             norm_momentum=momentum))
        self.trunk = nn.ModuleList(trunk)

        ups = []
        cin = spec.trunk_channels
        for c in spec.ups:
            ups.append(UpBlock(cin, c, 3, 2, norm=norm, act=True,
                               method=spec.upsample, norm_momentum=momentum))
            cin = c
        self.ups = nn.ModuleList(ups)

        self.tail = ConvLayer(cin, 3, spec.tail_kernel, 1, norm="none", act=False)


    @property
    def _mid_index(self) -> int:
        return (len(self.trunk) - 1) // 2

    def tap_modules(self):
        """Logical name -> module whose *output* is the distillation tap.

        Pre-activation for the down/up stages, post-residual-add for the trunk,
        so teacher and student taps line up point for point.
        """
        return OrderedDict([
            ("down1", self.downs[0].norm),
            ("trunk_in", self.downs[-1].norm),
            ("trunk_mid", self.trunk[self._mid_index]),
            ("trunk_out", self.trunk[-1]),
            ("pre_tail", self.ups[-1].norm),
        ])

    def tap_channels(self):
        """Channel count of each tap, so a loss can size its adapters up front."""
        return OrderedDict([
            ("down1", self.downs[0].out_channels),
            ("trunk_in", self.downs[-1].out_channels),
            ("trunk_mid", self.spec.trunk_channels),
            ("trunk_out", self.spec.trunk_channels),
            ("pre_tail", self.ups[-1].out_channels),
        ])

    def forward(self, x):
        y = self.head(x)
        for d in self.downs:
            y = d(y)
        for b in self.trunk:
            y = b(y)
        for u in self.ups:
            y = u(y)
        return self.tail(y)
