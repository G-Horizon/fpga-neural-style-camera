#!/usr/bin/env python3
"""Derive the d64 INT8 integer contract from the float graph.

What this produces
------------------

Per convolution: a per-output-channel INT8 weight, an INT32 bias in the
accumulator's scale, and a per-output-channel requantisation multiplier and
shift.  Per tensor: a format -- scale, zero point and representable integer
range -- that every producer and consumer of that tensor agrees on.

The format chain is not a free choice, and the measurement is what says so
-----------------------------------------------------------------------------

The trunk stream is **signed**.  ``x + conv3(...)`` has no activation after it,
so the block output is not a ReLU output and its minimum goes below zero, and it
drifts further with depth::

    trunk.0.out     -34.60 .. 134.11
    trunk.4.out    -127.21 .. 205.80

Those five blocks also have to share **one** format, because the skip is the
block input carried unchanged to the add and then onwards: block N's output is
block N+1's input *and* block N+1's skip, and the skip is added to conv3
without a requant in between, so conv3's output has to be representable in the
same format as the block input.  Five formats would need four conversions on
the skip path; one format needs none.

Every other tensor is a ReLU output, so its minimum is exactly 0, its zero point
is 0, and it is a plain uint8.  The input is uint8 with scale 1.0 and zero point
0, which represents the 0..255 input exactly -- no first-layer bias correction
is needed, because the correction term ``-z * sum(w)`` is zero when z is zero.

The zero point, and where it is not free
----------------------------------------

For a normal convolution a non-zero input zero point costs nothing::

    sum((x - z) * w) = sum(x * w) - z * sum(w)

and ``-z * sum(w)`` folds into the bias once per output channel at load time.

That trick does **not** survive the transposed convolution.  Its output pixel at
``(oh, ow)`` only receives the kernel taps whose parity matches, and at the
borders some of those fall outside the input, so the set of contributing taps
varies per output pixel and the correction stops being one constant per channel.

So this tool does not rely on it.  The activation operand presented to the MAC
is ``a = q - zero``, a small signed integer, and the zero point disappears from
the datapath entirely.  ``operand_bits`` reports the width that needs: 8 bits
for every ReLU tensor, whose zero point sits at the bottom of its range, and one
more than the trunk stream's code width for the trunk itself, which is the only
tensor with an interior zero point.

Why the trunk stream gets more codes than everything else
---------------------------------------------------------

The residual chain accumulates.  Measured over the 24-image calibration set the
trunk stream spans -277..387, while the tensor entering it spans only 0..209 --
and because all five blocks share one format, that wide range sets the step for
every value written into the stream, including the narrow ones.  The error is
then amplified about 1.5x per block, roughly 7x end to end, so a half-step of
error at the trunk entry arrives at the transposed convolution an order of
magnitude larger.

Measured on held-out images, going from 8 to 10 codes-worth of bits for the
trunk stream is worth several dB at the output, and past 10 it stops mattering,
because the stream is no longer what limits the result.  ``--trunk-bits`` is
that knob and 10 is the default.

Requantisation
--------------

    acc  = sum(a * w_q) + b_q                      signed accumulator
    y_q  = clamp( ((acc * M + (1 << (shift-1))) >> shift) + z_out )
    y    = (y_q - z_out) * s_out

``M`` and ``shift`` are per output channel, because the weight scale is::

    R = (s_in * s_w[oc]) / s_out        M = round(R * 2**shift)

with ``shift`` chosen so ``M`` lands in ``[2**(m_bits-1), 2**m_bits)``.  The
shift is arithmetic, so the rounding is round-half-up on ties and floor
otherwise; that is a contract the RTL has to match exactly, not an
implementation detail.  ``--m-bits`` narrows the multiplier and reports what it
costs in error.

Usage
-----

    python tools/d64_quant.py                             # derive and print
    python tools/d64_quant.py --write                     # also write golden/
    python tools/d64_quant.py --trunk-format int8_sym     # the format knob
    python tools/d64_quant.py --percentile 0.01           # clip the tails
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
from collections import OrderedDict
from dataclasses import dataclass, asdict

import numpy as np
import torch

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from d64_net import D64Net  # noqa: E402

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
GOLDEN_DIR = os.path.join(HERE, "golden")

#: The trunk stream's format.  'uint8_asym' uses the full 0..255 code range with
#: an interior zero point; 'int8_sym' forces the zero point to 0 and pays for it
#: with a coarser step.  Both are legal; which one is worth it is a measurement,
#: not an opinion, so both are reported.
TRUNK_FORMATS = ("uint8_asym", "int8_sym")


# ---------------------------------------------------------------------------
# formats
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class Format:
    """``x = (q - zero) * scale``, with ``q`` confined to ``[qmin, qmax]``."""

    name: str
    qmin: int
    qmax: int
    scale: float
    zero: int

    @property
    def levels(self) -> int:
        return self.qmax - self.qmin

    @property
    def operand_bits(self) -> int:
        """Width of ``q - zero``, which is what the MAC actually sees.

        When the zero point sits at the bottom of the range -- which is every
        ReLU output, since their minimum is exactly 0 -- the operand is
        non-negative and fits unsigned.  Otherwise it is signed, and the signed
        bound is ``-2**(W-1) <= lo``, not ``-2**(W-1) < lo``: -128 fits in eight
        signed bits, and ``abs(lo).bit_length() + 1`` says nine.  That off-by-one
        is not cosmetic here, because eight signed bits is exactly the boundary
        where the mapper reaches for EFX_DSP24 instead of EFX_DSP48.
        """
        if self.zero == self.qmin:
            return max(int((self.qmax - self.qmin).bit_length()), 1)
        lo, hi = self.qmin - self.zero, self.qmax - self.zero
        w_lo = (abs(lo) - 1).bit_length() + 1 if lo < 0 else 0
        w_hi = hi.bit_length() + 1 if hi > 0 else 0
        return max(w_lo, w_hi, 1)

    @property
    def operand_signed(self) -> bool:
        return self.zero != self.qmin

    def quantize(self, x: np.ndarray) -> np.ndarray:
        q = np.rint(np.asarray(x, dtype=np.float64) / self.scale).astype(np.int64) + self.zero
        return np.clip(q, self.qmin, self.qmax).astype(np.int32)

    def dequantize(self, q: np.ndarray) -> np.ndarray:
        return (np.asarray(q, dtype=np.float64) - self.zero) * self.scale

    def to_dict(self) -> dict:
        d = asdict(self)
        d["levels"] = self.levels
        d["operand_bits"] = self.operand_bits
        d["operand_signed"] = self.operand_signed
        return d


def derive_format(name: str, lo: float, hi: float, scheme: str, bits: int = 8) -> Format:
    """Fit a format to an observed range under the named scheme.

    ``bits`` sets how many codes the format has.  It is 8 everywhere except the
    trunk stream, where the residual chain's accumulated range makes 8 bits the
    tightest place in the network -- see the note in derive().
    """
    top = (1 << bits) - 1
    if scheme == "uint8_exact":
        # The 0..255 input and output: scale 1, no rounding at all.
        return Format(name, 0, top, 1.0, 0)
    if hi < lo:
        raise ValueError(f"{name}: empty range {lo}..{hi}")
    if scheme == "uint8_z0":
        if lo < 0:
            raise ValueError(f"{name}: uint8_z0 needs a non-negative range, "
                             f"got {lo}..{hi}")
        if hi <= 0:
            return Format(name, 0, top, 1.0, 0)
        return Format(name, 0, top, hi / top, 0)
    if scheme == "int8_sym":
        m = max(abs(lo), abs(hi))
        lim = (1 << (bits - 1)) - 1
        if m <= 0:
            return Format(name, -lim, lim, 1.0, 0)
        return Format(name, -lim, lim, m / lim, 0)
    if scheme == "operand_s8":
        # Unsigned storage, zero point in the middle, so the MAC operand
        # (q - zero) is SIGNED and fits the mapper's DSP24 rule.  This is the
        # only reason the scheme exists: measured on the Ti60, a multiplier
        # uses EFX_DSP24 -- two per physical block -- when both operands are
        # 8-bit signed, and EFX_DSP48 -- one per block -- the moment either
        # operand is 9 bits.  An unsigned 8-bit activation becomes a 9-bit
        # signed operand when it is zero-extended, which halves the MAC
        # density.  Moving the zero point to the midpoint keeps it at 8.
        #
        # The cost is range: the operand spans -128..127, so a tensor that
        # only ever goes positive gets 127 codes instead of 255 and its step
        # doubles.  Whether that is worth 2x the MAC density is a measurement.
        mid = 1 << (bits - 1)
        lim = mid - 1
        m = max(abs(lo), abs(hi))
        if m <= 0:
            return Format(name, 0, top, 1.0, mid)
        return Format(name, 0, top, m / lim, mid)
    if scheme == "uint8_asym":
        span = hi - lo
        if span <= 0:
            return Format(name, 0, top, 1.0, 0)
        scale = span / top
        zero = int(round(-lo / scale))
        zero = min(max(zero, 0), top)
        return Format(name, 0, top, scale, zero)
    raise ValueError(f"unknown format scheme {scheme!r}")


# ---------------------------------------------------------------------------
# the graph, as quantisable layers and tensors
# ---------------------------------------------------------------------------

@dataclass
class Layer:
    name: str
    kind: str            # conv | tconv
    cin: int
    cout: int
    k: int
    stride: int
    relu: bool
    in_tensor: str
    out_tensor: str
    weight: np.ndarray   # float32, [cout, cin, k, k] for conv, [cin, cout, k, k] for tconv
    bias: np.ndarray     # float32, [cout]

    @property
    def taps_per_output(self) -> int:
        """MAC terms per output element, for the accumulator bound.

        A normal convolution reads ``cin * k * k``.  A stride-2 transposed one
        reads fewer: the output at ``(oh, ow)`` only takes the kernel positions
        whose parity matches, which is ``ceil(k/2)`` per dimension, squared.
        """
        if self.kind == "conv":
            return self.cin * self.k * self.k
        per_dim = (self.k + 1) // 2
        return self.cin * per_dim * per_dim


def build_layers(net: D64Net) -> "list[Layer]":
    """The deployable layers, in execution order, with their tensor names."""
    L: list[Layer] = []

    def conv(name, block, in_t, out_t, relu, stride=1):
        w = block.conv2d.weight.detach().numpy().astype(np.float64)
        b = block.conv2d.bias.detach().numpy().astype(np.float64)
        L.append(Layer(name, "conv", w.shape[1], w.shape[0], w.shape[2],
                       stride, relu, in_t, out_t, w, b))

    conv("head", net.head, "in", "head", True)
    # the two downsamplers are stride 2, not 1 -- and the stride is not
    # recoverable from the weight shape, which is why it is written down here
    # rather than inferred
    conv("downs.0", net.downs[0], "head", "downs.0", True, stride=2)
    conv("downs.1", net.downs[1], "downs.0", "trunk", True, stride=2)
    for i, b in enumerate(net.trunk):
        conv(f"trunk.{i}.conv1", b.conv1, "trunk", f"trunk.{i}.conv1", True)
        conv(f"trunk.{i}.conv2", b.conv2, f"trunk.{i}.conv1", f"trunk.{i}.conv2", True)
        conv(f"trunk.{i}.conv3", b.conv3, f"trunk.{i}.conv2", f"trunk.{i}.conv3", False)
    for i, u in enumerate(net.ups):
        w = u.conv.conv2d.weight.detach().numpy().astype(np.float64)
        b_ = u.conv.conv2d.bias.detach().numpy().astype(np.float64)
        in_t = "trunk" if i == 0 else f"ups.{i - 1}"
        L.append(Layer(f"ups.{i}", "tconv", w.shape[0], w.shape[1], w.shape[2],
                       2, True, in_t, f"ups.{i}", w, b_))
    conv("tail", net.tail, "ups.1", "out", False)
    return L


# ---------------------------------------------------------------------------
# calibration
# ---------------------------------------------------------------------------

def load_images(paths, size=256):
    """uint8 RGB files -> float32 1x3xHxW in 0..255, via the training recipe."""
    from PIL import Image
    out = []
    for p in paths:
        with Image.open(p) as im:
            a = np.array(im.convert("RGB"), dtype=np.uint8, copy=True)
        x = torch.from_numpy(a).permute(2, 0, 1).contiguous().float()
        # resize the shorter side to `size`, then centre crop -- the same view
        # ValImageDataset produces, so calibration and validation see one recipe
        _, h, w = x.shape
        if min(h, w) != size:
            scale = size / min(h, w)
            nh, nw = max(size, int(round(h * scale))), max(size, int(round(w * scale)))
            x = torch.nn.functional.interpolate(
                x.unsqueeze(0), size=(nh, nw), mode="bilinear",
                align_corners=False, antialias=True)[0]
        _, h, w = x.shape
        top, left = max((h - size) // 2, 0), max((w - size) // 2, 0)
        out.append(x[:, top:top + size, left:left + size])
    return out


def calibrate(net: D64Net, images, percentile: float | None = None) -> dict:
    """Observed value range per tensor name, over the calibration set.

    With ``percentile`` set, the range is clipped at the ``p`` and ``100 - p``
    percentiles of the observed values rather than at the extremes.  That
    matters here more than it usually does: the trunk stream is a residual chain
    five blocks deep and its tails are long, so the single worst pixel in
    24 images sets the step for every value in the stream.  Clipping trades a
    handful of saturated pixels for a finer step everywhere else, and the
    integer reference in the next step measures which way that trade goes.
    """
    samples: dict[str, list[np.ndarray]] = {}
    with torch.no_grad():
        for img in images:
            for k, v in net.forward_taps(img.unsqueeze(0)).items():
                samples.setdefault(k, []).append(v.numpy().reshape(-1))

    lo: dict[str, float] = {}
    hi: dict[str, float] = {}
    for k, chunks in samples.items():
        a = np.concatenate(chunks) if percentile is not None else None
        if a is not None:
            lo[k] = float(np.percentile(a, percentile))
            hi[k] = float(np.percentile(a, 100.0 - percentile))
        else:
            lo[k] = float(min(c.min() for c in chunks))
            hi[k] = float(max(c.max() for c in chunks))
    return {"lo": lo, "hi": hi}


# ---------------------------------------------------------------------------
# the integer contract
# ---------------------------------------------------------------------------

def requant_params(s_acc: float, s_out: float, m_bits: int) -> "tuple[int, int]":
    """``(M, shift)`` with ``M`` in ``[2**(m_bits-1), 2**m_bits)``."""
    if s_acc <= 0 or s_out <= 0:
        raise ValueError(f"non-positive scale: s_acc={s_acc} s_out={s_out}")
    r = s_acc / s_out
    shift = m_bits - 1 - math.floor(math.log2(r))
    shift = max(shift, 1)
    m = int(round(r * (1 << shift)))
    while m >= (1 << m_bits) and shift > 1:
        shift -= 1
        m = int(round(r * (1 << shift)))
    while m < (1 << (m_bits - 1)) and shift < 62:
        shift += 1
        m = int(round(r * (1 << shift)))
    return m, shift


def derive(net: D64Net, ranges: dict, trunk_format: str, m_bits: int,
           weight_bits: int = 8, residual: str = "shared",
           trunk_bits: int = 8, narrow_format: str = "uint8_z0") -> "tuple[dict, dict]":
    """Everything the RTL needs, derived from the calibrated ranges.

    Returns ``(doc, arrays)``: ``doc`` is the JSON-serialisable contract, and
    ``arrays`` holds the integer tensors themselves, which are far too big to
    want in a report.

    ``residual`` picks how conv3 reaches the trunk format:

    ``shared``
        conv3's requant targets the trunk format directly, so the add is an
        integer add and nothing else.  This is the default because it costs no
        hardware: every convolution already has a requant, and the only
        decision is what output scale to aim it at.  The price is that conv3's
        own output range is narrower than the trunk stream's, so it uses only
        part of the code range.
    ``dual``
        conv3 gets a format fitted to its own range, and a second requant maps
        it onto the trunk format before the add.  Finer for conv3, one extra
        multiplier per block.
    """
    lo, hi = ranges["lo"], ranges["hi"]
    layers = build_layers(net)
    if residual not in ("shared", "dual"):
        raise ValueError(f"residual must be 'shared' or 'dual', got {residual!r}")

    # --- one format per tensor -------------------------------------------
    # Post-ReLU tensors are uint8 with zero point 0.  The trunk stream is the
    # exception and gets whatever the caller asked for, fitted to the union of
    # every tensor that shares it.
    # Everything that is *carried in* the trunk stream: the tensor entering
    # block 0, each block's output (which is the next block's input and skip),
    # and each conv3's output, which is written straight into the same format.
    # conv1 and conv2 are not in this list -- they are ReLU outputs.
    trunk_tensors = (["downs.1"]
                     + [f"trunk.{i}.out" for i in range(len(net.trunk))]
                     + [f"trunk.{i}.conv3" for i in range(len(net.trunk))])
    t_lo = min(lo[t] for t in trunk_tensors)
    t_hi = max(hi[t] for t in trunk_tensors)

    formats: "OrderedDict[str, Format]" = OrderedDict()
    formats["in"] = derive_format("in", 0.0, 255.0, "uint8_exact")
    formats["out"] = derive_format("out", 0.0, 255.0, "uint8_exact")
    formats["trunk"] = derive_format("trunk", t_lo, t_hi, trunk_format, trunk_bits)
    for name in ("head", "downs.0", "ups.0", "ups.1"):
        formats[name] = derive_format(name, lo[name], hi[name], narrow_format)
    for i in range(len(net.trunk)):
        for c in ("conv1", "conv2"):
            t = f"trunk.{i}.{c}"
            formats[t] = derive_format(t, lo[t], hi[t], narrow_format)
    # conv3 writes the trunk stream.  'shared' means it writes it outright;
    # 'dual' gives it its own format and leaves the conversion to the add.
    for i in range(len(net.trunk)):
        t = f"trunk.{i}.conv3"
        formats[t] = formats["trunk"] if residual == "shared" else \
            derive_format(t, lo[t], hi[t], "uint8_asym", trunk_bits)

    # --- per-layer weights, bias and requant ------------------------------
    records = []
    arrays = OrderedDict()
    for L in layers:
        f_in = formats[L.in_tensor]
        f_out = formats[L.out_tensor]

        # per-output-channel symmetric INT8.  The oc axis is dim 0 for a normal
        # convolution and dim 1 for a transposed one -- the two layouts differ,
        # and using the wrong one scales the tensor along the wrong axis without
        # raising anything.
        oc_axis = 0 if L.kind == "conv" else 1
        reduce_axes = tuple(i for i in range(L.weight.ndim) if i != oc_axis)
        absmax = np.abs(L.weight).max(axis=reduce_axes)
        lim = (1 << (weight_bits - 1)) - 1
        w_scale = np.where(absmax > 0, absmax / lim, 1.0)
        w_q = np.rint(L.weight / w_scale.reshape(
            [-1 if i == oc_axis else 1 for i in range(L.weight.ndim)])).astype(np.int64)
        w_q = np.clip(w_q, -lim, lim).astype(np.int32)

        s_acc = f_in.scale * w_scale                 # accumulator scale, per oc
        b_q = np.rint(L.bias / s_acc).astype(np.int64)
        # the accumulator also has to hold z_in * sum(w), which the operand
        # pre-subtraction turns into an extra input magnitude of |z_in| * |w|;
        # the bound below accounts for it rather than assuming it away
        max_acc = int(math.ceil(
            max(abs(f_in.qmin - f_in.zero), abs(f_in.qmax - f_in.zero))
            * lim * L.taps_per_output))
        acc_bits = int(max_acc.bit_length()) + 1
        b_bits = int(max(int(np.abs(b_q).max()), 1).bit_length()) + 1

        M, shift = [], []
        for oc in range(L.cout):
            m, s = requant_params(float(s_acc[oc]), f_out.scale, m_bits)
            M.append(m)
            shift.append(s)

        records.append(OrderedDict([
            ("name", L.name), ("kind", L.kind),
            ("cin", L.cin), ("cout", L.cout), ("k", L.k), ("stride", L.stride),
            ("relu", L.relu),
            ("in_tensor", L.in_tensor), ("out_tensor", L.out_tensor),
            ("taps_per_output", L.taps_per_output),
            ("weight_layout", "cout,cin,k,k" if L.kind == "conv" else "cin,cout,k,k"),
            ("weight_bits", weight_bits),
            ("weight_scale", [float(v) for v in w_scale]),
            ("weight_scale_range", [float(w_scale.min()), float(w_scale.max())]),
            ("bias_bits", b_bits),
            ("bias_int32", [int(v) for v in b_q]),
            ("acc_bits_required", acc_bits),
            ("acc_worst_case", max_acc),
            ("requant_M", M), ("requant_shift", shift),
            ("requant_M_bits", int(max(M).bit_length())),
            ("product_bits", acc_bits + int(max(M).bit_length())),
        ]))
        arrays[L.name] = OrderedDict([
            ("w_q", w_q), ("w_scale", w_scale), ("b_q", b_q.astype(np.int64)),
            ("M", np.asarray(M, dtype=np.int64)),
            ("shift", np.asarray(shift, dtype=np.int64)),
        ])

    doc = OrderedDict([
        ("trunk_format", trunk_format),
        ("residual", residual),
        ("weight_bits", weight_bits),
        ("trunk_bits", trunk_bits),
        ("narrow_format", narrow_format),
        ("m_bits", m_bits),
        ("formats", OrderedDict((k, v.to_dict()) for k, v in formats.items())),
        ("tap_formats", tap_formats(formats, net)),
        ("calibration", {
            "observed_range_trunk_union": [t_lo, t_hi],
            "trunk_tensors_sharing_one_format": trunk_tensors,
        }),
        ("layers", records),
    ])
    return doc, arrays


def tap_formats(formats: dict, net: D64Net) -> dict:
    """Tap name -> the format its quantized values are expressed in.

    ``downs.1`` is the odd one: it is the tap that *enters* the trunk, and it is
    already in the trunk stream's format, so it is produced by downs.1's requant
    aiming at that format rather than at a format of its own.
    """
    m = {"in": "in", "head": "head", "downs.0": "downs.0", "downs.1": "trunk"}
    for i in range(len(net.trunk)):
        m[f"trunk.{i}.conv1"] = f"trunk.{i}.conv1"
        m[f"trunk.{i}.conv2"] = f"trunk.{i}.conv2"
        m[f"trunk.{i}.conv3"] = f"trunk.{i}.conv3"
        m[f"trunk.{i}.out"] = "trunk"
    m["ups.0"], m["ups.1"], m["tail"] = "ups.0", "ups.1", "out"
    missing = [k for k, v in m.items() if v not in formats]
    if missing:
        raise ValueError(f"tap_formats references unknown formats: {missing}")
    return m


def report(doc: dict) -> None:
    tl, th = doc["calibration"]["observed_range_trunk_union"]
    print("The trunk stream, which is the only signed tensor in the network")
    print()
    print(f"  observed union   {tl:9.2f} .. {th:9.2f}   span {th - tl:.2f}")
    f = doc["formats"]["trunk"]
    used = (th - tl) / f["scale"]
    print(f"  format           zero {f['zero']}, step {f['scale']:.4f}, "
          f"covers {(0 - f['zero']) * f['scale']:.1f} .. "
          f"{(255 - f['zero']) * f['scale']:.1f}")
    print(f"  the five blocks that share it: {len(doc['calibration']['trunk_tensors_sharing_one_format'])} tensors")
    print(f"  {used:.0f} of 255 codes used, operand {f['operand_bits']} bits "
          f"({'signed' if f['operand_signed'] else 'unsigned'})")
    print()

    print("Formats -- x = (q - zero) * scale")
    print()
    print(f"  {'tensor':16s} {'q range':>12s} {'zero':>5s} {'scale':>10s} "
          f"{'bits':>5s}  operand")
    for name, f in doc["formats"].items():
        rng = f"[{f['qmin']},{f['qmax']}]"
        print(f"  {name:16s} {rng:>12s} {f['zero']:5d} {f['scale']:10.6f} "
              f"{f['operand_bits']:5d}  "
              f"{'signed' if f['operand_signed'] else 'unsigned'}")
    print()

    print("Layers")
    print()
    print(f"  {'layer':16s} {'in':>9s} {'out':>9s} {'acc':>5s} {'worst acc':>12s} "
          f"{'M bits':>6s} {'prod':>5s} {'relu':>5s}")
    for r in doc["layers"]:
        print(f"  {r['name']:16s} {r['in_tensor']:>9s} {r['out_tensor']:>9s} "
              f"{r['acc_bits_required']:5d} {r['acc_worst_case']:12,d} "
              f"{r['requant_M_bits']:6d} {r['product_bits']:5d} "
              f"{'yes' if r['relu'] else 'no':>5s}")
    print()

    worst = max(r["acc_bits_required"] for r in doc["layers"])
    prods = max(r["product_bits"] for r in doc["layers"])
    print(f"  widest accumulator   {worst} bits signed")
    print(f"  widest requant product {prods} bits")
    print(f"  trunk format         {doc['trunk_format']}, "
          f"zero {doc['formats']['trunk']['zero']}, "
          f"scale {doc['formats']['trunk']['scale']:.4f}")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--checkpoint", default=os.path.join(HERE, "models", "weights", "vangogh", "model_fp32.pth"))
    ap.add_argument("--data-root", default=os.path.join(HERE, "training",
                                                        "data", "coco_val2017"))
    ap.add_argument("--split", default=None,
                    help="list of image paths relative to --data-root; default "
                         "is the training tree's val_fixed.txt")
    ap.add_argument("--limit", type=int, default=0, help="cap the calibration set")
    ap.add_argument("--trunk-format", default="uint8_asym", choices=TRUNK_FORMATS)
    ap.add_argument("--m-bits", type=int, default=16,
                    help="requant multiplier width.  16 is the frozen D2.1 " 
                         "contract: held-out quality is indistinguishable from " 
                         "31, and 25x16 costs two DSP48 where 25x31 costs four")
    ap.add_argument("--weight-bits", type=int, default=8)
    ap.add_argument("--percentile", type=float, default=0.01,
                    help="clip calibration to this percentile of the observed "
                         "values instead of using the raw extremes")
    ap.add_argument("--trunk-bits", type=int, default=10,
                    help="codes for the trunk stream format; the residual chain "
                         "is where 8 bits is tightest")
    ap.add_argument("--narrow-format", default="uint8_z0",
                    choices=("uint8_z0", "operand_s8"),
                    help="operand_s8 moves the ReLU layers' zero point to the "
                         "midpoint so their MAC operand is signed 8-bit, which "
                         "is what the mapper's DSP24 rule requires")
    ap.add_argument("--residual", default="shared", choices=("shared", "dual"),
                    help="how conv3 reaches the trunk format -- see derive()")
    ap.add_argument("--write", action="store_true")
    ap.add_argument("--json", default=os.path.join(GOLDEN_DIR, "d64_int8.json"))
    ap.add_argument("--npz", default=os.path.join(GOLDEN_DIR, "d64_int8_weights.npz"))
    args = ap.parse_args()

    split = args.split
    if split is None:
        cand = os.path.join(args.data_root, "..", "splits", "val_fixed.txt")
        split = cand if os.path.isfile(cand) else None
    if split and os.path.isfile(split):
        with open(split, encoding="utf-8") as fh:
            names = [ln.strip() for ln in fh if ln.strip() and not ln.startswith("#")]
    else:
        names = sorted(f for f in os.listdir(args.data_root)
                       if f.lower().endswith((".jpg", ".png")))
    if args.limit:
        names = names[:args.limit]
    paths = [os.path.join(args.data_root, n) for n in names]
    missing = [p for p in paths if not os.path.isfile(p)]
    if missing:
        sys.exit(f"missing calibration images, e.g. {missing[0]}")

    net = D64Net.from_checkpoint(args.checkpoint)
    images = load_images(paths)
    print(f"calibrating on {len(images)} images from {args.data_root}")
    ranges = calibrate(net, images, args.percentile)
    doc, arrays = derive(net, ranges, args.trunk_format, args.m_bits,
                         args.weight_bits, args.residual, args.trunk_bits,
                         args.narrow_format)
    doc["calibration"]["images"] = len(images)
    doc["calibration"]["source"] = os.path.relpath(args.data_root, HERE).replace("\\", "/")
    doc["calibration"]["percentile"] = args.percentile
    print()

    if args.trunk_format == "int8_sym":
        alt, _ = derive(net, ranges, "uint8_asym", args.m_bits, args.weight_bits,
                        args.residual, args.trunk_bits)
        f1, f2 = doc["formats"]["trunk"], alt["formats"]["trunk"]
        print("trunk format comparison")
        print(f"  uint8_asym  zero {f2['zero']:3d}  step {f2['scale']:.6f}  "
              f"operand {f2['operand_bits']} bits")
        print(f"  int8_sym    zero {f1['zero']:3d}  step {f1['scale']:.6f}  "
              f"operand {f1['operand_bits']} bits")
        print(f"  int8_sym's step is {f1['scale'] / f2['scale']:.3f}x coarser")
        print()

    report(doc)

    if args.write:
        os.makedirs(os.path.dirname(args.json), exist_ok=True)
        with open(args.json, "w", encoding="utf-8", newline="\n") as fh:
            json.dump(doc, fh, indent=1)
            fh.write("\n")
        print(f"\nwrote {args.json}")

        flat = {}
        for name, a in arrays.items():
            for k, v in a.items():
                flat[f"{name}/{k}"] = v
        np.savez_compressed(args.npz, **flat)
        print(f"wrote {args.npz}  ({len(flat)} arrays, "
              f"{os.path.getsize(args.npz) / 1024:.1f} KiB)")


if __name__ == "__main__":
    main()
