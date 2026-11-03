# -*- coding: utf-8 -*-
"""Normalisation conversion: the stage-3 path.

``instance -> fixed -> affine`` is a sequence of *exact* rewrites of the same
affine map, not retraining steps::

    y = gamma * (x - mu) / sqrt(var + eps) + beta
      = (gamma / sqrt(var + eps)) * x + (beta - mu * gamma / sqrt(var + eps))
      = a * x + b

Every normalisation this project emits -- ``InstanceNorm2d`` with
``track_running_stats=True``, ``FixedStatNorm2d`` and ``AffineNorm2d`` -- stores
or can derive that ``(a, b)``.  Converting therefore preserves the function
exactly, which is what makes the ablation meaningful: any change in quality
after a conversion comes from subsequent finetuning, never from the rewrite.

The rewrite matters for deployment because ``y = a*x + b`` folds into the
preceding convolution's weights and bias, leaving pure convolution.
"""

from __future__ import annotations

from dataclasses import replace

import torch
import torch.nn as nn

from .models.blocks import AffineNorm2d, FixedStatNorm2d, folded_affine_of
from .models.student import StudentNet

_NORM_TYPES = (nn.InstanceNorm2d, FixedStatNorm2d, AffineNorm2d)

#: How far a rewrite may move the output, in uint8 levels.  Outputs are in
#: 0..255, so an absolute difference of 1.0 is one level.  A quarter of a level
#: is far enough below the rounding step that the delivered image cannot change
#: by more than a rounding decision, and far enough above the float32 noise
#: floor (~0.13 levels on real content) that the check is meaningful rather than
#: impossible.  In float64 the same rewrites are exact.
FOLD_TOLERANCE_LEVELS = 0.25


def is_norm_module(module: nn.Module) -> bool:
    return isinstance(module, _NORM_TYPES)


def norm_kind(module: nn.Module) -> str:
    if isinstance(module, nn.InstanceNorm2d):
        return "instance" if module.track_running_stats else "instance_dynamic"
    if isinstance(module, FixedStatNorm2d):
        return "fixed"
    if isinstance(module, AffineNorm2d):
        return "affine"
    if isinstance(module, nn.Identity):
        return "none"
    return type(module).__name__


@torch.no_grad()
def _write_norm(dst: nn.Module, scale: torch.Tensor, shift: torch.Tensor, eps: float):
    """Realise ``y = scale * x + shift`` in the parameterisation ``dst`` uses."""
    if isinstance(dst, AffineNorm2d):
        dst.weight.copy_(scale)
        dst.bias.copy_(shift)
    elif isinstance(dst, FixedStatNorm2d):
        # mean 0 and var 1-eps make the layer's own scale/sqrt(var+eps) equal 1,
        # so (weight, bias) carry the map directly.
        dst.weight.copy_(scale)
        dst.bias.copy_(shift)
        dst.mean.zero_()
        dst.var.fill_(1.0 - eps)
    elif isinstance(dst, nn.InstanceNorm2d):
        dst.weight.copy_(scale)
        dst.bias.copy_(shift)
        dst.running_mean.zero_()
        dst.running_var.fill_(1.0 - eps)
    else:
        raise TypeError("cannot write an affine map into %s" % type(dst).__name__)


@torch.no_grad()
def convert_norm(model: StudentNet, target_norm: str, eps: float = 1e-5) -> StudentNet:
    """Return a copy of ``model`` whose normalisation is ``target_norm``.

    Function-preserving: the returned model computes the same thing as the input
    for every foldable normalisation.
    """
    if target_norm not in ("instance", "fixed", "affine"):
        raise ValueError("target_norm must be instance/fixed/affine, got %r" % (target_norm,))
    spec = replace(model.spec, norm=target_norm)
    out = StudentNet(spec)

    old_modules = dict(model.named_modules())
    translated = 0
    for name, new_mod in out.named_modules():
        if not name:
            continue
        old_mod = old_modules[name]
        if is_norm_module(new_mod):
            folded = folded_affine_of(old_mod)
            if folded is None:
                raise ValueError(
                    "cannot fold %s at %r: it has no fixed statistics "
                    "(dynamic InstanceNorm). Train it, or convert from an "
                    "instance model whose running statistics were accumulated."
                    % (type(old_mod).__name__, name))
            _write_norm(new_mod, folded[0], folded[1], eps)
            translated += 1
        elif is_norm_module(old_mod):
            raise TypeError("structure mismatch at %r" % name)
        elif not list(new_mod.children()):
            new_mod.load_state_dict(old_mod.state_dict())
    if translated == 0:
        raise RuntimeError("no normalisation layers were converted")
    return out


@torch.no_grad()
def check_conversion(model: StudentNet, converted: StudentNet, x: torch.Tensor):
    """``(max_abs_drift, relative_drift)`` between a model and its rewrite.

    Both are reported because the absolute figure alone is not interpretable:
    this network amplifies a point perturbation by ~90x, so any float32 rewrite
    shows a few hundredths of a level of drift on real content, and a *zero*
    input -- which sits far outside the data distribution and pins every
    normalisation at its floor -- is neither representative nor stable.  The
    relative figure answers the question that matters: does the rewrite move the
    delivered uint8 image?
    """
    model.eval()
    converted.eval()
    a = model(x)
    b = converted(x)
    drift = float((a - b).abs().max())
    span = float(a.max() - a.min())
    return drift, drift / max(span, 1e-6)


def freeze_fixed_stats(model: nn.Module) -> int:
    """Stop every ``FixedStatNorm2d`` from following batch statistics."""
    n = 0
    for m in model.modules():
        if isinstance(m, FixedStatNorm2d):
            m.freeze_stats()
            n += 1
    return n


def fold_all(model: nn.Module):
    """Return ``{module_name: (scale, shift)}`` for every foldable normalisation.

    ``None`` values mark layers that cannot be folded (dynamic statistics or
    ``Identity``), which is what a deployment check should be looking for.
    """
    out = {}
    for name, m in model.named_modules():
        if is_norm_module(m):
            out[name] = folded_affine_of(m)
    return out


def norm_summary(model: nn.Module) -> dict:
    """Per-kind counts of the normalisation layers plus their statistics range."""
    kinds, means, vars_ = {}, [], []
    for m in model.modules():
        if not is_norm_module(m):
            continue
        k = norm_kind(m)
        kinds[k] = kinds.get(k, 0) + 1
        if isinstance(m, nn.InstanceNorm2d):
            mean, var = m.running_mean, m.running_var
        else:
            mean, var = getattr(m, "mean", None), getattr(m, "var", None)
        if mean is not None and var is not None:
            means.append(float(mean.mean()))
            vars_.append(float(var.mean()))
    return {
        "layers": kinds,
        "mean_of_means": sum(means) / len(means) if means else None,
        "mean_of_vars": sum(vars_) / len(vars_) if vars_ else None,
    }


# ---------------------------------------------------------------------------
# folding: normalisation -> convolution
# ---------------------------------------------------------------------------

def _fold_units(model: nn.Module):
    """Outermost modules that own both a convolution and its normalisation."""
    from .models.blocks import ConvLayer, UpBlock, UpsampleConv

    found = []

    def walk(module, prefix, inside):
        for child_name, child in module.named_children():
            full = "%s.%s" % (prefix, child_name) if prefix else child_name
            if inside:
                walk(child, full, True)
            elif isinstance(child, (ConvLayer, UpBlock, UpsampleConv)):
                found.append((full, child))
                walk(child, full, True)
            else:
                walk(child, full, False)

    walk(model, "", False)
    return found


@torch.no_grad()
def fold_model(model: "StudentNet", eps: float = 1e-5):
    """Return an equivalent model with every normalisation folded into the
    preceding convolution.

    ``y = conv(x)`` then ``y = a*y + b`` is the same function as a convolution
    with ``W' = W * a`` and ``b' = b_conv * a + b``.  The result contains no
    normalisation at all -- only convolution, ReLU and residual adds, which is
    what the FPGA implements and what makes the ``a*x + b`` endpoint real rather
    than nominal.

    The rewrite is exact in exact arithmetic; :func:`check_fold` measures the
    float32 drift.
    """
    from .models.blocks import ConvLayer, UpBlock, UpsampleConv

    spec = replace(model.spec, norm="none")
    out = StudentNet(spec)
    out.load_state_dict(model.state_dict(), strict=False)

    old_units = dict(_fold_units(model))
    new_units = dict(_fold_units(out))
    folded = 0

    for name, old in old_units.items():
        new = new_units[name]
        if isinstance(old, ConvLayer):
            conv_old, conv_new = old.conv2d, new.conv2d
            aff = folded_affine_of(old.norm)
        elif isinstance(old, UpBlock):
            conv_old, conv_new = old.conv.conv2d, new.conv.conv2d
            aff = folded_affine_of(old.norm)
        else:  # UpsampleConv carries no normalisation of its own
            conv_old, conv_new = old.conv2d, new.conv2d
            aff = None

        if aff is None:
            conv_new.weight.copy_(conv_old.weight)
            if conv_old.bias is not None:
                conv_new.bias.copy_(conv_old.bias)
            continue

        scale, shift = aff
        # Conv2d stores (Cout, Cin, kh, kw); ConvTranspose2d stores (Cin, Cout, kh, kw).
        # The per-channel affine applies to the *output* channels.
        dim = 1 if isinstance(conv_old, nn.ConvTranspose2d) else 0
        shape = [1] * conv_old.weight.dim()
        shape[dim] = -1
        conv_new.weight.copy_(conv_old.weight * scale.reshape(shape))
        base = conv_old.bias if conv_old.bias is not None else torch.zeros_like(scale)
        conv_new.bias.copy_(base * scale + shift)
        folded += 1

    if folded == 0:
        raise RuntimeError("nothing was folded; is the model already affine-only?")
    return out


@torch.no_grad()
def check_fold(model, folded, x: torch.Tensor) -> float:
    """Max absolute output difference between a model and its folded twin."""
    model.eval()
    folded.eval()
    return float((model(x) - folded(x)).abs().max())


def assert_no_data_dependent_norm(model) -> list:
    """Names of layers that still need per-image statistics, i.e. cannot fold."""
    bad = []
    for name, m in model.named_modules():
        if isinstance(m, nn.InstanceNorm2d) and not m.track_running_stats:
            bad.append(name)
        elif isinstance(m, nn.InstanceNorm2d) and (m.running_mean is None):
            bad.append(name)
    return bad
