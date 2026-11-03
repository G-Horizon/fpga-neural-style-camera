# -*- coding: utf-8 -*-
"""FPGA delivery bundle.

``export`` writes, for one checkpoint:

    model_spec.json     layer-by-layer description, training-only modules removed
    model_fp32.pth      weights of the folded (normalisation-free) graph
    folded_norm.json    the per-layer (scale, shift) that was folded in
    quant_params.json   first-cut INT8 scales; superseded by ``qat``
    mac_report.json     MAC / parameters under the canonical convention
    evaluation.json     the unified evaluation of this exact checkpoint
    sample_outputs/     content | teacher | student | |diff| strips

The exported graph is verified two ways: folding must reproduce the original
output, and the spec must contain no module that exists only for training.
"""

from __future__ import annotations

import os
from collections import OrderedDict

import torch
import torch.nn as nn

from .convert import FOLD_TOLERANCE_LEVELS, fold_all, fold_model, check_fold
from .data import ValImageDataset, build_loader, read_split, val_collate
from .evaluator import evaluate_model
from .losses import DistillLoss
from .mac import parameter_breakdown, profile
from .models.blocks import AffineNorm2d, BottleneckBlock, ConvLayer, FixedStatNorm2d, \
    ResidualBlock, UpBlock, UpConvTranspose, UpsampleConv
from .models.teacher import load_teacher
from .predict import load_model_from_checkpoint
from .utils import ROOT, load_config, setup_logger, write_json


# ---------------------------------------------------------------------------
# structural description
# ---------------------------------------------------------------------------

def _norm_record(module: nn.Module) -> dict:
    if isinstance(module, nn.Identity):
        return {"type": "none", "foldable": True}
    if isinstance(module, AffineNorm2d):
        return {"type": "affine", "foldable": True, "form": "y = a*x + b"}
    if isinstance(module, FixedStatNorm2d):
        return {"type": "fixed", "foldable": True,
                "form": "y = gamma*(x-mu)/sqrt(var+eps) + beta",
                "eps": module.eps, "statistics": "frozen buffers"}
    if isinstance(module, nn.InstanceNorm2d):
        return {"type": "instance", "foldable": bool(module.track_running_stats),
                "statistics": "running" if module.track_running_stats else "per-image",
                "eps": module.eps}
    return {"type": type(module).__name__, "foldable": False}


def _conv_record(name, conv: nn.Conv2d, norm: nn.Module, activation: str) -> dict:
    return OrderedDict([
        ("name", name),
        ("type", "conv2d"),
        ("cin", conv.in_channels),
        ("cout", conv.out_channels),
        ("kernel", list(conv.kernel_size)),
        ("stride", list(conv.stride)),
        ("padding", {"mode": "reflect", "amount": [conv.kernel_size[0] // 2,
                                                   conv.kernel_size[1] // 2]}),
        ("dilation", list(conv.dilation)),
        ("groups", conv.groups),
        ("bias", conv.bias is not None),
        ("activation", activation),
        ("normalization", _norm_record(norm)),
    ])


def _conv_transpose_record(name, conv: nn.ConvTranspose2d, norm: nn.Module,
                           activation: str) -> dict:
    return OrderedDict([
        ("name", name),
        ("type", "conv_transpose2d"),
        ("cin", conv.in_channels),
        ("cout", conv.out_channels),
        ("kernel", list(conv.kernel_size)),
        ("stride", list(conv.stride)),
        ("padding", list(conv.padding)),
        ("output_padding", list(conv.output_padding)),
        ("groups", conv.groups),
        ("bias", conv.bias is not None),
        ("activation", activation),
        ("normalization", _norm_record(norm)),
        ("upsample_method", "transpose"),
    ])


def describe_model(model) -> dict:
    """Flatten the student into an ordered, FPGA-implementable layer list."""
    spec = model.spec
    layers = []

    def add_conv_layer(path, block: ConvLayer):
        layers.append(_conv_record(path, block.conv2d, block.norm,
                                   "relu" if _is_relu(block.act) else "none"))

    add_conv_layer("head", model.head)
    for i, d in enumerate(model.downs):
        add_conv_layer("downs.%d" % i, d)

    for i, block in enumerate(model.trunk):
        if isinstance(block, BottleneckBlock):
            layers.append(OrderedDict([("name", "trunk.%d" % i), ("type", "residual_block"),
                                       ("bottleneck", True), ("skip", True)]))
            add_conv_layer("trunk.%d.conv1" % i, block.conv1)
            add_conv_layer("trunk.%d.conv2" % i, block.conv2)
            add_conv_layer("trunk.%d.conv3" % i, block.conv3)
        else:
            layers.append(OrderedDict([("name", "trunk.%d" % i), ("type", "residual_block"),
                                       ("bottleneck", False), ("skip", True)]))
            add_conv_layer("trunk.%d.conv1" % i, block.conv1)
            add_conv_layer("trunk.%d.conv2" % i, block.conv2)

    for i, up in enumerate(model.ups):
        layers.append(_conv_transpose_record("ups.%d.conv" % i, up.conv.conv2d, up.norm,
                                             "relu" if _is_relu(up.act) else "none"))

    add_conv_layer("tail", model.tail)

    return OrderedDict([
        ("name", spec.name),
        ("arch_string", spec.arch_string()),
        ("input", OrderedDict([("layout", "NCHW"), ("channels", 3),
                               ("range", [0, 255]), ("dtype", "uint8 -> float32"),
                               ("canonical_resolution", [256, 256])])),
        ("output", OrderedDict([("channels", 3), ("range", [0, 255]),
                                ("activation", "none")])),
        ("trunk_scale", spec.scale),
        ("normalization", spec.norm),
        ("upsample_method", spec.upsample),
        ("residual_add_activation", "none"),
        ("layers", layers),
        ("training_only_modules_removed",
         ["feature adapters (1x1 convs)", "VGG16 loss network", "Haar DWT loss"]),
    ])


def _is_relu(module) -> bool:
    return isinstance(module, nn.ReLU)


def _iter_convs(model):
    """Every convolution the deployed graph executes, in order."""
    for module in model.modules():
        if isinstance(module, (nn.Conv2d, nn.ConvTranspose2d)):
            yield module


# ---------------------------------------------------------------------------
# INT8 scales (first cut)
# ---------------------------------------------------------------------------

@torch.no_grad()
def calibration_scales(model, loader, device, percentile: float = 99.99):
    """Per-tensor activation scales and per-channel weight scales.

    This is a plain max-observation calibration, enough to describe the export.
    It is *not* a substitute for the QAT pass, which is what fixes the final
    scales; ``quant_params.json`` records which of the two produced it.
    """
    from .mac import iter_mac_units
    from .models.blocks import UpsampleConv

    units = dict(iter_mac_units(model))
    act_max = {name: 0.0 for name in units}
    handles = []

    def hook(name):
        def fn(module, inputs, output):
            act_max[name] = max(act_max[name], float(output.detach().abs().max()))
        return fn

    for name, module in units.items():
        handles.append(module.register_forward_hook(hook(name)))
    try:
        for content, _names in loader:
            model(content.to(device))
    finally:
        for h in handles:
            h.remove()

    weights = OrderedDict()
    for name, module in units.items():
        conv = module.conv2d if isinstance(module, UpsampleConv) else module
        if not hasattr(conv, "weight"):
            continue
        # per-output-channel symmetric scales
        dims = tuple(i for i in range(conv.weight.dim()) if i != _out_dim(conv))
        per_channel = conv.weight.detach().abs().amax(dim=dims)
        weights[name] = OrderedDict([
            ("scheme", "symmetric per-output-channel"),
            ("bits", 8),
            ("scale", [float(v) / 127.0 for v in per_channel]),
        ])
    activations = OrderedDict(
        (name, OrderedDict([("scheme", "symmetric per-tensor"),
                            ("bits", 8),
                            ("observed_abs_max", act_max[name]),
                            ("percentile", percentile),
                            ("scale", act_max[name] / 127.0 if act_max[name] > 0 else 0.0)]))
        for name in units)
    return weights, activations


def _out_dim(conv) -> int:
    return 1 if isinstance(conv, nn.ConvTranspose2d) else 0


# ---------------------------------------------------------------------------
# entry point
# ---------------------------------------------------------------------------

def export_model(args):
    cfg = load_config(args.config)
    device = torch.device(args.device) if args.device else torch.device(
        "cuda" if torch.cuda.is_available() else "cpu")
    out_dir = args.out_dir
    os.makedirs(os.path.join(out_dir, "sample_outputs"), exist_ok=True)
    logger = setup_logger(os.path.join(out_dir, "export.log"))

    model, ckpt = load_model_from_checkpoint(args.checkpoint, device=device)
    logger.info("exporting %s from %s", model.spec.arch_string(), args.checkpoint)

    # --- structural spec -------------------------------------------------
    spec = describe_model(model)
    convs = [l for l in spec["layers"] if "normalization" in l]
    if len(convs) != sum(1 for _ in _iter_convs(model)):
        raise RuntimeError("spec lists %d convolutions, model has %d"
                           % (len(convs), sum(1 for _ in _iter_convs(model))))
    unfoldable = [l["name"] for l in convs if not l["normalization"]["foldable"]]
    if unfoldable:
        raise RuntimeError(
            "these layers keep data-dependent statistics and cannot be folded: %s"
            % unfoldable)
    write_json(os.path.join(out_dir, "model_spec.json"), spec)

    # --- folding ---------------------------------------------------------
    #
    # The probe is real validation content, not zeros.  This network amplifies a
    # point perturbation by roughly two orders of magnitude, so a zero input --
    # which sits far outside the data distribution and drives every
    # normalisation to its floor -- is not a representative or stable place to
    # measure rounding.  The acceptance test is deliberately *relative*: the
    # fold must not move the output by a fraction of one uint8 level, because
    # that is the thing the FPGA actually ships.
    val_ds = ValImageDataset(os.path.join(ROOT, cfg["data"]["root"]),
                             read_split(os.path.join(ROOT, cfg["data"]["val_list"])),
                             crop=int(cfg["data"].get("val_crop", 256)))
    probe = torch.stack([val_ds[i][0] for i in range(min(4, len(val_ds)))], 0).to(device)
    with torch.no_grad():
        reference = model(probe)
    span = float(reference.max() - reference.min())
    folded = fold_model(model).to(device)
    drift = check_fold(model, folded, probe)
    relative = drift / max(span, 1e-6)
    logger.info("fold drift max %.3e over an output span of %.1f "
                "(%.2e relative, %.4f of one uint8 level)",
                drift, span, relative, drift)
    if drift > FOLD_TOLERANCE_LEVELS:
        raise RuntimeError(
            "folding moved the output by %.4f uint8 levels, above the %.2f-level "
            "bar, so it is a real change and not rounding"
            % (drift, FOLD_TOLERANCE_LEVELS))
    drift_note = OrderedDict([
        ("probe", "4 real validation images, 256x256"),
        ("max_abs_output_difference", drift),
        ("output_span", span),
        ("relative", relative),
        # The output is already in 0..255 units, so an absolute difference of
        # 1.0 IS one uint8 level.
        ("max_abs_difference_in_uint8_levels", drift),
    ])
    torch.save({"model": folded.state_dict(), "spec": folded.spec.to_dict()},
               os.path.join(out_dir, "model_fp32.pth"))
    write_json(os.path.join(out_dir, "folded_norm.json"),
               OrderedDict((k, None if v is None else
                            {"scale": v[0].tolist(), "shift": v[1].tolist()})
                           for k, v in fold_all(model).items()))
    write_json(os.path.join(out_dir, "fold_report.json"),
               OrderedDict(list(drift_note.items()) +
                           [("note", "the folded graph is what the FPGA runs; "
                                     "only convolution, ReLU and residual adds")]))

    # --- MAC -------------------------------------------------------------
    rep = profile(folded, (3, 256, 256), convention="fpga", name=model.spec.name)
    payload = rep.to_dict()
    payload["parameter_breakdown"] = parameter_breakdown(folded)
    payload["graph"] = "folded (normalisation removed)"
    write_json(os.path.join(out_dir, "mac_report.json"), payload)
    logger.info("%.6f GMAC/frame, %d params after folding",
                rep.gmac_per_frame, payload["parameter_breakdown"]["total"])

    # --- INT8 scales -----------------------------------------------------
    cal_loader = build_loader(val_ds, 4, False, 0, collate_fn=val_collate)
    weights, activations = calibration_scales(folded, cal_loader, device)
    write_json(os.path.join(out_dir, "quant_params.json"), OrderedDict([
        ("source", "post-training calibration, not QAT"),
        ("calibration_set", cfg["data"]["val_list"]),
        ("n_images", len(val_ds)),
        ("weight", weights),
        ("activation", activations),
    ]))

    # --- evaluation and samples ------------------------------------------
    teacher = load_teacher(os.path.join(ROOT, cfg["teacher"]["checkpoint"]), device=device)
    loss_fn = DistillLoss(cfg["loss"], model, teacher, device=device).to(device)
    result = evaluate_model(model, teacher, cal_loader, loss_fn=loss_fn, device=device,
                            out_dir=os.path.join(out_dir, "sample_outputs"),
                            export_images=True, contact_sheet=True, logger=logger)
    evaluation = result.to_dict()
    evaluation["arch_string"] = model.spec.arch_string()
    evaluation["mac_report"] = payload
    evaluation["checkpoint"] = args.checkpoint
    evaluation["fold_drift_absolute"] = drift
    evaluation["fold_drift_relative"] = relative
    write_json(os.path.join(out_dir, "evaluation.json"), evaluation)

    logger.info("exported to %s", out_dir)
    return 0
