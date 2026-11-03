# -*- coding: utf-8 -*-
"""The frozen Candy teacher.

A verbatim re-implementation of the 2017 ``fast_neural_style`` transformer,
built on :class:`tinystyle.models.blocks.ConvLayer` so that the parameter names
(``conv1.conv2d.weight``, ``res1.in1.running_mean``, ...) match ``candy.pth``
exactly and the checkpoint loads with ``strict=True``.

Macro structure::

    c9s1-32 -> d64 -> d128 -> R128 x5 -> u64 -> u32 -> c9s1-3

The teacher is only ever used for inference / feature extraction.
"""

from __future__ import annotations

from collections import OrderedDict

import torch
import torch.nn as nn

from .blocks import ConvLayer, UpsampleConv

#: Logical tap points shared by the teacher and every student.  The feature
#: distillation loss pairs teacher/student features through these names.
TAP_NAMES = ("down1", "trunk_in", "trunk_mid", "trunk_out", "pre_tail")


class TeacherResidualBlock(nn.Module):
    """Teacher block: conv-IN-ReLU, conv-IN, plus the skip.  No ReLU on the add."""

    def __init__(self, channels):
        super().__init__()
        self.conv1 = ConvLayer(channels, channels, 3, 1)
        self.in1 = nn.InstanceNorm2d(channels, affine=True, track_running_stats=True)
        self.conv2 = ConvLayer(channels, channels, 3, 1)
        self.in2 = nn.InstanceNorm2d(channels, affine=True, track_running_stats=True)
        self.relu = nn.ReLU(inplace=False)

    def forward(self, x):
        out = self.relu(self.in1(self.conv1(x)))
        out = self.in2(self.conv2(out))
        return out + x


class TransformerNet(nn.Module):
    def __init__(self):
        super().__init__()
        self.conv1 = ConvLayer(3, 32, 9, 1)
        self.in1 = nn.InstanceNorm2d(32, affine=True, track_running_stats=True)
        self.conv2 = ConvLayer(32, 64, 3, 2)
        self.in2 = nn.InstanceNorm2d(64, affine=True, track_running_stats=True)
        self.conv3 = ConvLayer(64, 128, 3, 2)
        self.in3 = nn.InstanceNorm2d(128, affine=True, track_running_stats=True)

        self.res1 = TeacherResidualBlock(128)
        self.res2 = TeacherResidualBlock(128)
        self.res3 = TeacherResidualBlock(128)
        self.res4 = TeacherResidualBlock(128)
        self.res5 = TeacherResidualBlock(128)

        self.deconv1 = UpsampleConv(128, 64, 3, upsample=2)
        self.in4 = nn.InstanceNorm2d(64, affine=True, track_running_stats=True)
        self.deconv2 = UpsampleConv(64, 32, 3, upsample=2)
        self.in5 = nn.InstanceNorm2d(32, affine=True, track_running_stats=True)
        self.deconv3 = ConvLayer(32, 3, 9, 1)

        self.relu = nn.ReLU(inplace=False)

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
        return self.deconv3(y)

    def tap_modules(self):
        """Logical name -> module whose *output* is the distillation tap.

        Every tap is taken pre-activation (or post-residual-add) so teacher and
        student taps line up for the same logical point.
        """
        return OrderedDict([
            ("down1", self.in2),
            ("trunk_in", self.in3),
            ("trunk_mid", self.res3),
            ("trunk_out", self.res5),
            ("pre_tail", self.in5),
        ])

    def tap_channels(self):
        return OrderedDict([
            ("down1", 64),
            ("trunk_in", 128),
            ("trunk_mid", 128),
            ("trunk_out", 128),
            ("pre_tail", 32),
        ])


def load_teacher(checkpoint, device="cpu", eval_mode=True):
    """Load ``candy.pth``.  Raises immediately if the checkpoint does not match."""
    model = TransformerNet()
    state = torch.load(checkpoint, map_location="cpu", weights_only=True)
    if isinstance(state, dict) and "state_dict" in state:
        state = state["state_dict"]
    model.load_state_dict(state, strict=True)
    model.to(device)
    if eval_mode:
        model.eval()
        for p in model.parameters():
            p.requires_grad_(False)
    return model
