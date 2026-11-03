# -*- coding: utf-8 -*-
"""INT8 quantisation-aware training, run on the *folded* graph.

Order matters and is the project's stated one::

    architecture fixed -> normalisation fixed -> FP32 converged
      -> distillation finetune -> topology frozen -> INT8 QAT
      -> calibration -> bit-exact export

So this module never invents a topology.  It takes a checkpoint, folds every
normalisation into the preceding convolution (leaving pure convolution, ReLU,
padding and residual adds), and only then attaches quantisation observers.  That
is what makes the exported integer weights mean something: there is no
normalisation left whose scale would have to be re-derived on the FPGA.

QAT finetunes against the same frozen teacher with the output, perceptual, style
and high-frequency terms.  Feature distillation is disabled because the FX graph
module exposes no intermediate taps; the term is a small part of the objective
and the teachers' own output is still the dominant signal.

The two numbers that matter are reported side by side: the fp32 student's
validation error and the same error after ``convert_fx``.
"""

from __future__ import annotations

import copy
import os
from collections import OrderedDict

import torch
import torch.nn as nn
from torch.ao.quantization import get_default_qat_qconfig_mapping
from torch.ao.quantization.quantize_fx import convert_fx, prepare_qat_fx

from .convert import check_fold, fold_model
from .data import ContentDataset, ValImageDataset, build_loader, read_split, val_collate
from .evaluator import evaluate_model
from .losses import DistillLoss
from .mac import parameter_breakdown, profile
from .models.teacher import load_teacher
from .predict import load_model_from_checkpoint
from .utils import ROOT, load_config, setup_logger, write_json


def fold_for_quantisation(model):
    """Fold, then assert the result carries no normalisation at all."""
    folded = fold_model(model).eval()
    leftovers = [n for n, m in folded.named_modules()
                 if isinstance(m, (nn.InstanceNorm2d,)) or
                 type(m).__name__ in ("FixedStatNorm2d", "AffineNorm2d")]
    if leftovers:
        raise RuntimeError("normalisation survived folding: %s" % leftovers)
    return folded


def extract_int8(converted, name_prefix=""):
    """Per-layer integer weights, scales and zero points from a converted model."""
    from torch.ao.nn.quantized import Conv2d as QConv2d
    from torch.ao.nn.quantized import ConvTranspose2d as QConvT2d

    out = OrderedDict()
    for name, module in converted.named_modules():
        if isinstance(module, (QConv2d, QConvT2d)):
            weight = module.weight().int_repr().to(torch.int8)
            out[name] = OrderedDict([
                ("type", type(module).__name__),
                ("weight_int8", weight.tolist()),
                ("weight_shape", list(weight.shape)),
                ("weight_scale", float(module.weight().q_scale())),
                ("weight_zero_point", int(module.weight().q_zero_point())),
                ("output_scale", float(module.scale)),
                ("output_zero_point", int(module.zero_point)),
                ("input_scale", float(module.input_scale) if hasattr(module, "input_scale") else None),
                ("bias_scale", float(module.bias().q_scale()) if module.bias() is not None else None),
            ])
    return out


def run_qat(args):
    cfg = load_config(args.config)
    device = torch.device(args.device) if args.device else torch.device("cpu")
    out_dir = args.out_dir
    os.makedirs(out_dir, exist_ok=True)
    logger = setup_logger(os.path.join(out_dir, "qat.log"))

    model, ckpt = load_model_from_checkpoint(args.checkpoint, device="cpu")
    folded = fold_for_quantisation(model)
    example = torch.zeros(1, 3, 256, 256)
    drift = check_fold(model, folded, example)
    logger.info("folded %s (drift %.2e)", folded.spec.arch_string(), drift)

    mapping = get_default_qat_qconfig_mapping("qnnpack")
    prepared = prepare_qat_fx(copy.deepcopy(folded).train(), mapping, (example,))
    n_fake = sum(1 for m in prepared.modules()
                 if type(m).__name__ == "FusedMovingAvgObsFakeQuantize")
    logger.info("QAT prepared: %d fake-quant nodes", n_fake)

    teacher = load_teacher(os.path.join(ROOT, cfg["teacher"]["checkpoint"]), device=device)
    loss_fn = DistillLoss(cfg["loss"], prepared, teacher, device=device).to(device)

    train_ds = ContentDataset(
        os.path.join(ROOT, cfg["data"]["root"]),
        read_split(os.path.join(ROOT, cfg["data"]["train_list"])),
        crop=int(cfg["data"].get("crop", 256)),
        scales=tuple(cfg["data"].get("view_scales", [1.0])),
        augment_d4=True, seed=int(cfg["experiment"].get("seed", 0)))
    train_loader = build_loader(train_ds, int(cfg["optim"]["batch_size"]), True,
                                int(cfg["data"].get("num_workers", 4)),
                                seed=int(cfg["experiment"].get("seed", 0)), drop_last=True)

    params = list(prepared.parameters())
    if loss_fn.adapters is not None:
        params += list(loss_fn.adapters.parameters())
    optimizer = torch.optim.Adam(params, lr=float(args.lr))
    loader = iter(train_loader)
    prepared.train()
    for step in range(int(args.steps)):
        try:
            content = next(loader)
        except StopIteration:
            loader = iter(train_loader)
            content = next(loader)
        content = content.to(device)
        with torch.no_grad():
            teacher_out = teacher(content)
        optimizer.zero_grad(set_to_none=True)
        student_out = prepared(content)
        total, parts = loss_fn(student_out.float(), teacher_out, {}, {})
        total.backward()
        torch.nn.utils.clip_grad_norm_(params, 1.0)
        optimizer.step()
        if step % 50 == 0:
            logger.info("qat step %-5d loss %.4f  %s", step, float(total),
                        loss_fn.describe(parts))

    # Calibrate observers on real data, then freeze the graph.
    prepared.eval()
    with torch.no_grad():
        for i, content in enumerate(build_loader(train_ds, 4, False, 0)):
            prepared(content)
            if i >= 15:
                break
    converted = convert_fx(prepared)
    logger.info("converted to INT8: %d modules", sum(1 for _ in converted.modules()))

    # --- measure the quantisation gap on the frozen validation set ---------
    val_ds = ValImageDataset(os.path.join(ROOT, cfg["data"]["root"]),
                             read_split(os.path.join(ROOT, cfg["data"]["val_list"])),
                             crop=int(cfg["data"].get("val_crop", 256)))
    val_loader = build_loader(val_ds, 4, False, 0, collate_fn=val_collate)

    qat_loss = DistillLoss(cfg["loss"], prepared, teacher, device=device).to(device)
    fp32_result = evaluate_model(folded, teacher, val_loader, loss_fn=None, device=device,
                                 export_images=False, logger=logger)
    int8_result = evaluate_model(converted, teacher, val_loader, loss_fn=None, device=device,
                                 export_images=True,
                                 out_dir=os.path.join(out_dir, "sample_outputs"),
                                 contact_sheet=True, logger=logger)

    logger.info("fp32  val L1 %.4f  PSNR %.2f", fp32_result.metrics["output_l1"],
                fp32_result.metrics["psnr"])
    logger.info("int8  val L1 %.4f  PSNR %.2f", int8_result.metrics["output_l1"],
                int8_result.metrics["psnr"])

    int8_weights = extract_int8(converted)
    torch.save({"int8": int8_weights,
                "fp32_state_dict": folded.state_dict(),
                "spec": folded.spec.to_dict()},
               os.path.join(out_dir, "model_qat_int8.pth"))

    rep = profile(folded, (3, 256, 256), convention="fpga", name=folded.spec.name)
    write_json(os.path.join(out_dir, "quant_params.json"), OrderedDict([
        ("source", "INT8 QAT via torch.ao.quantization (qnnpack)"),
        ("steps", int(args.steps)),
        ("scheme", "symmetric per-output-channel weights, per-tensor activations"),
        ("layers", int8_weights),
    ]))
    write_json(os.path.join(out_dir, "mac_report.json"),
               OrderedDict(list(rep.to_dict().items()) +
                           [("parameter_breakdown", parameter_breakdown(folded))]))
    write_json(os.path.join(out_dir, "evaluation.json"), OrderedDict([
        ("arch_string", folded.spec.arch_string()),
        ("checkpoint_fp32", args.checkpoint),
        ("fold_drift", drift),
        ("fp32", fp32_result.to_dict()),
        ("int8", int8_result.to_dict()),
        ("degradation", OrderedDict([
            ("output_l1_delta", int8_result.metrics["output_l1"] - fp32_result.metrics["output_l1"]),
            ("psnr_delta", int8_result.metrics["psnr"] - fp32_result.metrics["psnr"]),
        ])),
    ]))
    logger.info("wrote %s", out_dir)
    return 0
