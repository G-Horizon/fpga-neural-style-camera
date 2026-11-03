# -*- coding: utf-8 -*-
"""Canonical MAC / parameter accounting.

The rule
--------
This is the accounting the Ti60 project uses, and it is the one that reproduces
the teacher exactly:

* ``Conv2d``            ``MAC = Cin * Cout * kh * kw * H_out * W_out``
* ``ConvTranspose2d``   ``MAC = Cin * Cout * kh * kw * H_in  * W_in``
* ``Linear``            ``MAC = in_features * out_features``

For a stride-``s`` transposed convolution the second rule is the count of
*non-zero* multiplications -- the zero-insertion step is not counted.  At
``s = 2`` that is 4x cheaper than "nearest upsample then convolve on the
upsampled grid", which is what :class:`tinystyle.models.blocks.UpsampleConv`
actually executes.

``UpsampleConv`` (the teacher's decoder stage, and the ``resize`` upsample
option for students) is therefore credited with the *transposed* count under the
default ``fpga`` convention -- that is the budget convention, defined by the
8.266973184 figure.  Pass ``convention="native"`` to instead charge it for the
work the PyTorch graph really performs, which is four times as much.  Students
built with ``upsample='transpose'`` report the same number under both
conventions.

Note that the convention is a *budget*, not a description of the hardware: the
teacher's own decoder is nearest+conv, so the teacher executes 10.08 GMAC while
being credited 8.27, and whether the FPGA will implement a transposed
convolution is an open decision -- see ``docs/DECONV_CONTRACT.md``.

What is *not* counted
---------------------
Normalisation, activation, residual adds, upsampling itself and memory traffic
contribute no MACs here, matching the project's existing budget.  They are real
FPGA costs, but they are not convolution arithmetic and are not allowed to
inflate or deflate this comparison.

Verification
------------
``profile(teacher, (3, 256, 256))`` returns exactly ``8266973184`` MAC and
``1674432`` convolution weights.  ``tests/test_mac.py`` asserts both.
"""

from __future__ import annotations

from collections import OrderedDict
from dataclasses import dataclass, field

import torch
import torch.nn as nn

from .models.blocks import UpsampleConv

CONVENTIONS = ("fpga", "native")

#: MAC and convolution-weight count of the candy teacher at 256x256.
TEACHER_MAC_256 = 8266973184
TEACHER_CONV_PARAMS = 1674432


@dataclass
class OpRecord:
    name: str
    kind: str
    cin: int
    cout: int
    kernel: tuple
    stride: tuple
    in_hw: tuple
    out_hw: tuple
    mac: int
    weight_params: int
    bias_params: int

    def to_dict(self) -> dict:
        return OrderedDict([
            ("name", self.name),
            ("kind", self.kind),
            ("cin", self.cin),
            ("cout", self.cout),
            ("kernel", list(self.kernel)),
            ("stride", list(self.stride)),
            ("in_hw", list(self.in_hw)),
            ("out_hw", list(self.out_hw)),
            ("mac", self.mac),
            ("weight_params", self.weight_params),
            ("bias_params", self.bias_params),
        ])


@dataclass
class MacReport:
    model: str
    convention: str
    input_resolution: tuple
    ops: list = field(default_factory=list)

    # -- aggregates -------------------------------------------------------
    @property
    def mac_total(self) -> int:
        return sum(o.mac for o in self.ops)

    @property
    def weight_params(self) -> int:
        return sum(o.weight_params for o in self.ops)

    @property
    def bias_params(self) -> int:
        return sum(o.bias_params for o in self.ops)

    @property
    def gmac_per_frame(self) -> float:
        return self.mac_total / 1e9

    def mac_by_output_resolution(self) -> "OrderedDict[str, int]":
        out = OrderedDict()
        for o in self.ops:
            key = "%dx%d" % tuple(o.out_hw)
            out[key] = out.get(key, 0) + o.mac
        return OrderedDict(sorted(out.items(), key=lambda kv: -kv[1]))

    def mac_by_kind(self) -> "OrderedDict[str, int]":
        out = OrderedDict()
        for o in self.ops:
            out[o.kind] = out.get(o.kind, 0) + o.mac
        return out

    def format_table(self) -> str:
        rows = [("op", "kind", "in", "out", "res", "MAC", "%")]
        total = max(self.mac_total, 1)
        for o in self.ops:
            rows.append((
                o.name,
                o.kind,
                "%dx%d" % (o.cin, o.cout),
                "%dx%d/%d" % (o.kernel[0], o.kernel[1], o.stride[0]),
                "%dx%d" % tuple(o.out_hw),
                "%.6f G" % (o.mac / 1e9),
                "%.2f" % (100.0 * o.mac / total),
            ))
        widths = [max(len(r[i]) for r in rows) for i in range(len(rows[0]))]
        lines = []
        for i, r in enumerate(rows):
            lines.append("  ".join(c.ljust(widths[j]) for j, c in enumerate(r)).rstrip())
            if i == 0:
                lines.append("  ".join("-" * w for w in widths))
        return "\n".join(lines)

    def to_dict(self) -> dict:
        return OrderedDict([
            ("model", self.model),
            ("convention", self.convention),
            ("mac_convention_note",
             "Conv2d: Cin*Cout*kh*kw*Ho*Wo. ConvTranspose2d: Cin*Cout*kh*kw*Hi*Wi "
             "(non-zero MACs only). UpsampleConv is credited as the equivalent "
             "stride-2 transposed convolution under convention='fpga'."),
            ("input_resolution", list(self.input_resolution)),
            ("mac_per_frame", self.mac_total),
            ("gmac_per_frame", float("%.9f" % self.gmac_per_frame)),
            ("conv_weight_params", self.weight_params),
            ("conv_bias_params", self.bias_params),
            ("conv_params_total", self.weight_params + self.bias_params),
            ("mac_by_output_resolution", self.mac_by_output_resolution()),
            ("mac_by_kind", self.mac_by_kind()),
            ("ops", [o.to_dict() for o in self.ops]),
        ])


# ---------------------------------------------------------------------------
# unit discovery
# ---------------------------------------------------------------------------

def is_mac_unit(module: nn.Module) -> bool:
    if getattr(module, "MAC_UNIT", False):
        return True
    return isinstance(module, (nn.Conv2d, nn.ConvTranspose2d, nn.Linear))


def iter_mac_units(model: nn.Module):
    """Yield ``(qualified_name, module)`` for the outermost MAC units.

    Descending stops at a unit, so ``UpsampleConv`` is reported as one operator
    instead of leaking its inner ``Conv2d``.
    """
    found = []

    def walk(module, prefix, inside_unit):
        for child_name, child in module.named_children():
            full = "%s.%s" % (prefix, child_name) if prefix else child_name
            if inside_unit:
                walk(child, full, True)
            elif is_mac_unit(child):
                found.append((full, child))
                walk(child, full, True)
            else:
                walk(child, full, False)

    if is_mac_unit(model):
        found.append(("", model))
    else:
        walk(model, "", False)
    return found


# ---------------------------------------------------------------------------
# per-unit MAC
# ---------------------------------------------------------------------------

def _conv2d_record(name, mod, in_shape, out_shape, convention):
    cin, cout = mod.in_channels, mod.out_channels
    kh, kw = mod.kernel_size
    ho, wo = out_shape[-2:]
    mac = cin * cout * kh * kw * ho * wo
    return OpRecord(name, "conv2d", cin, cout, mod.kernel_size, mod.stride,
                    tuple(in_shape[-2:]), (ho, wo), mac,
                    mod.weight.numel(), 0 if mod.bias is None else mod.bias.numel())


def _conv_transpose_record(name, mod, in_shape, out_shape, convention):
    cin, cout = mod.in_channels, mod.out_channels
    kh, kw = mod.kernel_size
    hi, wi = in_shape[-2:]
    mac = cin * cout * kh * kw * hi * wi
    return OpRecord(name, "conv_transpose2d", cin, cout, mod.kernel_size, mod.stride,
                    (hi, wi), tuple(out_shape[-2:]), mac,
                    mod.weight.numel(), 0 if mod.bias is None else mod.bias.numel())


def _linear_record(name, mod, in_shape, out_shape, convention):
    mac = mod.in_features * mod.out_features
    return OpRecord(name, "linear", mod.in_features, mod.out_features, (1, 1), (1, 1),
                    (1, 1), (1, 1), mac, mod.weight.numel(),
                    0 if mod.bias is None else mod.bias.numel())


def _upsample_conv_record(name, mod, in_shape, out_shape, convention):
    """Teacher-style decoder stage: nearest upsample + conv.

    Credited as the transposed convolution it is equivalent to.  The spatial
    reference is the *input* map, i.e. the size the transposed convolution would
    run at, not the upsampled grid the PyTorch graph materialises.
    """
    conv = mod.conv2d
    cin, cout = conv.in_channels, conv.out_channels
    kh, kw = conv.kernel_size
    hi, wi = in_shape[-2:]
    ho, wo = out_shape[-2:]
    if convention == "native":
        mac = cin * cout * kh * kw * ho * wo
    else:
        mac = cin * cout * kh * kw * hi * wi
    return OpRecord(name, "upsample_conv(%s)" % convention, cin, cout,
                    conv.kernel_size, (mod.upsample, mod.upsample),
                    (hi, wi), (ho, wo), mac,
                    conv.weight.numel(), 0 if conv.bias is None else conv.bias.numel())


_DISPATCH = [
    (UpsampleConv, _upsample_conv_record),
    (nn.ConvTranspose2d, _conv_transpose_record),
    (nn.Conv2d, _conv2d_record),
    (nn.Linear, _linear_record),
]


def _record_for(name, mod, in_shape, out_shape, convention):
    for cls, fn in _DISPATCH:
        if isinstance(mod, cls):
            return fn(name, mod, in_shape, out_shape, convention)
    raise TypeError("no MAC rule for %s (%s)" % (name, type(mod).__name__))


# ---------------------------------------------------------------------------
# entry point
# ---------------------------------------------------------------------------

def profile(model: nn.Module, input_size=(3, 256, 256), convention: str = "fpga",
            name: str = None, device=None, strict_repeats: bool = True) -> MacReport:
    """Run one forward pass on a zero input and tabulate every MAC unit.

    ``input_size`` may be ``(C, H, W)`` or ``(N, C, H, W)``.  The model is *not*
    moved: the probe input is built on whatever device the model already lives
    on, so profiling never disturbs the caller's placement.
    """
    if convention not in CONVENTIONS:
        raise ValueError("convention must be one of %s" % (CONVENTIONS,))
    if len(input_size) == 3:
        input_size = (1,) + tuple(input_size)
    input_size = tuple(int(v) for v in input_size)

    if device is None:
        try:
            device = next(model.parameters()).device
        except StopIteration:
            device = torch.device("cpu")

    was_training = model.training
    model.eval()

    units = iter_mac_units(model)
    seen = OrderedDict()
    order = []

    def make_hook(qname):
        def hook(module, inputs, output):
            if qname in seen:
                if strict_repeats:
                    raise RuntimeError(
                        "module %r ran more than once in one forward pass; MAC "
                        "accounting assumes a single execution per module" % qname)
                return
            seen[qname] = (tuple(inputs[0].shape), tuple(output.shape))
            order.append(qname)
        return hook

    handles = [m.register_forward_hook(make_hook(n)) for n, m in units]
    try:
        with torch.no_grad():
            model(torch.zeros(*input_size, device=device))
    finally:
        for h in handles:
            h.remove()
        if was_training:
            model.train()

    by_name = dict(units)
    ops = []
    for qname in order:
        in_shape, out_shape = seen[qname]
        ops.append(_record_for(qname, by_name[qname], in_shape, out_shape, convention))

    return MacReport(model=name or type(model).__name__, convention=convention,
                     input_resolution=input_size[-2:], ops=ops)


def trainable_parameter_count(model: nn.Module) -> int:
    return sum(p.numel() for p in model.parameters())


def parameter_breakdown(model: nn.Module) -> dict:
    """Parameter counts split into convolution weights, biases and the rest."""
    conv_w = conv_b = other = 0
    for qname, mod in iter_mac_units(model):
        for pname, p in mod.named_parameters():
            if pname.endswith("bias"):
                conv_b += p.numel()
            else:
                conv_w += p.numel()
    counted = set()
    for _, mod in iter_mac_units(model):
        for p in mod.parameters():
            counted.add(id(p))
    for p in model.parameters():
        if id(p) not in counted:
            other += p.numel()
    return OrderedDict([
        ("conv_weight", conv_w),
        ("conv_bias", conv_b),
        ("norm_and_other", other),
        ("total", conv_w + conv_b + other),
    ])
