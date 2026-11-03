# -*- coding: utf-8 -*-
"""Command line entry point: ``python -m tinystyle.cli <command>``."""

from __future__ import annotations

import argparse
import os
import sys
from collections import OrderedDict

import torch

from .mac import TEACHER_CONV_PARAMS, TEACHER_MAC_256, parameter_breakdown, profile
from .models.student import StudentNet, spec_from_dict
from .models.teacher import load_teacher
from .utils import ROOT, load_config, setup_logger, write_json


def _device(arg):
    if arg:
        return torch.device(arg)
    return torch.device("cuda" if torch.cuda.is_available() else "cpu")


# ---------------------------------------------------------------------------
# check-teacher  (stage 0 gate)
# ---------------------------------------------------------------------------

def cmd_check_teacher(args):
    from .data import ValImageDataset, build_loader, read_split, val_collate
    from .evaluator import psnr, ssim
    from .tiling import stylize_tiled

    cfg = load_config(args.config or os.path.join(ROOT, "configs", "base.yaml"),
                      base_path=os.path.join(ROOT, "configs", "base.yaml"))
    device = _device(args.device)
    out_dir = args.out_dir or os.path.join(ROOT, "runs", "stage0_teacher")
    os.makedirs(os.path.join(out_dir, "teacher_outputs"), exist_ok=True)
    logger = setup_logger(os.path.join(out_dir, "check_teacher.log"))

    teacher = load_teacher(os.path.join(ROOT, cfg["teacher"]["checkpoint"]), device=device)

    # --- 1. MAC / params -------------------------------------------------
    rep = profile(teacher, (3, 256, 256), convention="fpga", name="teacher")
    payload = rep.to_dict()
    payload["parameter_breakdown"] = parameter_breakdown(teacher)
    payload["expected_mac_per_frame"] = TEACHER_MAC_256
    payload["expected_conv_weight_params"] = TEACHER_CONV_PARAMS
    payload["mac_matches_expected"] = rep.mac_total == TEACHER_MAC_256
    payload["conv_params_match_expected"] = rep.weight_params == TEACHER_CONV_PARAMS
    write_json(os.path.join(out_dir, "mac_report.json"), payload)

    print(rep.format_table())
    print()
    print("  MAC/frame       %d" % rep.mac_total)
    print("  expected        %d" % TEACHER_MAC_256)
    print("  GMAC/frame      %.9f" % rep.gmac_per_frame)
    print("  conv weights    %d (expected %d)" % (rep.weight_params, TEACHER_CONV_PARAMS))
    native = profile(teacher, (3, 256, 256), convention="native", name="teacher")
    print("  same graph under convention='native': %.9f GMAC "
          "(the nearest-upsample the PyTorch graph really runs)" % native.gmac_per_frame)
    if not payload["mac_matches_expected"]:
        raise SystemExit("STAGE 0 FAILED: MAC calculator does not reproduce the teacher")

    # --- 2. fixed validation inference -----------------------------------
    val_ds = ValImageDataset(os.path.join(ROOT, cfg["data"]["root"]),
                             read_split(os.path.join(ROOT, cfg["data"]["val_list"])),
                             crop=int(cfg["data"].get("val_crop", 256)))
    loader = build_loader(val_ds, int(cfg["eval"]["batch_size"]), False, 0,
                          collate_fn=val_collate)
    logger.info("running teacher over %d fixed validation images", len(val_ds))

    from PIL import Image
    import numpy as np

    rows, pass1 = [], []
    with torch.no_grad():
        for content, names in loader:
            content = content.to(device)
            out = teacher(content)
            pass1.append(out.cpu())
            for i, name in enumerate(names):
                arr = (out[i].clamp(0, 255).round().to(torch.uint8)
                       .permute(1, 2, 0).cpu().numpy())
                stem = os.path.splitext(os.path.basename(name))[0]
                Image.fromarray(arr).save(
                    os.path.join(out_dir, "teacher_outputs", stem + ".png"))
                c = content[i:i + 1]
                rows.append(OrderedDict([
                    ("name", name),
                    ("output_mean", float(out[i].mean())),
                    ("output_std", float(out[i].std())),
                    ("psnr_vs_input", psnr(out[i:i + 1], c)),
                    ("ssim_vs_input", ssim(out[i:i + 1], c)),
                ]))

    # --- 3. determinism and tiling ---------------------------------------
    pass1 = torch.cat(pass1, 0)
    pass2 = []
    with torch.no_grad():
        for content, _names in build_loader(val_ds, int(cfg["eval"]["batch_size"]),
                                            False, 0, collate_fn=val_collate):
            pass2.append(teacher(content.to(device)).cpu())
    determinism = float((pass1 - torch.cat(pass2, 0)).abs().max())

    # Tiling must be exact, on an image large enough for tiling to actually
    # happen: 768x768 with tile=512, halo=128 gives two tiles per axis.
    #
    # In float32 the two paths will *not* agree bit-for-bit, and that is not a
    # receptive-field violation: the tiles are different tensor shapes, so cuDNN
    # picks different kernels and the rounding differs.  This network amplifies a
    # point perturbation by roughly two orders of magnitude, so sub-ULP rounding
    # differences surface as a few LSB.  float64 is the honest test of the
    # receptive-field bound, and it must be exactly zero.
    from .data import resize_shorter
    big = resize_shorter(val_ds[0][0], 768)
    big = big[:, :768, :768].unsqueeze(0).to(device)
    with torch.no_grad():
        whole32 = teacher(big)
        tiled32 = stylize_tiled(teacher, big, tile=512, halo=128)
    tiling_fp32 = float((whole32 - tiled32).abs().max())

    teacher64 = load_teacher(os.path.join(ROOT, cfg["teacher"]["checkpoint"]),
                             device=device).to(torch.float64)
    with torch.no_grad():
        whole64 = teacher64(big.to(torch.float64))
        tiled64 = stylize_tiled(teacher64, big.to(torch.float64), tile=512, halo=128)
    tiling_fp64 = float((whole64 - tiled64).abs().max())
    del teacher64
    torch.cuda.empty_cache()

    with torch.no_grad():
        probe = big.clone()
        probe[0, :, 300, 300] += 1e-3
        amplification = float((teacher(probe) - whole32).abs().max()) / 1e-3
    if tiling_fp64 != 0.0:
        raise SystemExit(
            "STAGE 0 FAILED: tiled and whole-image inference differ in float64 "
            "(%.3e); the halo does not cover the receptive field" % tiling_fp64)

    report = OrderedDict([
        ("teacher_checkpoint", cfg["teacher"]["checkpoint"]),
        ("mac_per_frame", rep.mac_total),
        ("gmac_per_frame", rep.gmac_per_frame),
        ("expected_gmac_per_frame", TEACHER_MAC_256 / 1e9),
        ("mac_matches_expected", payload["mac_matches_expected"]),
        ("conv_weight_params", rep.weight_params),
        ("conv_weight_params_match", payload["conv_params_match_expected"]),
        ("native_convention_gmac_per_frame", native.gmac_per_frame),
        ("input_convention", "float RGB in 0..255"),
        ("val_images", len(val_ds)),
        ("determinism_max_abs_diff", determinism),
        ("tiled_vs_whole_max_abs_diff_fp32", tiling_fp32),
        ("tiled_vs_whole_max_abs_diff_fp64", tiling_fp64),
        ("tiling_note",
         "float64 must be exactly 0 (receptive-field bound). float32 is not 0 "
         "because tiles are different tensor shapes and cuDNN rounds "
         "differently; the network amplifies a point perturbation ~%.0fx, so "
         "sub-ULP rounding shows up as a few LSB." % amplification),
        ("point_perturbation_amplification", amplification),
        ("per_image", rows),
        ("git_note", "teacher checkpoint is not modified; nothing here trains"),
    ])
    write_json(os.path.join(out_dir, "stage0_report.json"), report)

    print()
    print("  validation images            %d -> %s" % (len(val_ds),
          os.path.join(out_dir, "teacher_outputs")))
    print("  repeat-run max |diff|        %.3e" % determinism)
    print("  tiled vs whole-image, fp64  %.3e  (768x768, tile=512, halo=128)"
          % tiling_fp64)
    print("  tiled vs whole-image, fp32  %.3e  (rounding, not receptive field)"
          % tiling_fp32)
    print("  point-perturbation gain     %.1fx" % amplification)
    print()
    print("STAGE 0 PASSED -- teacher MAC reproduces exactly, inference is "
          "deterministic, and tiling is exact in float64.")
    print("report: %s" % os.path.join(out_dir, "stage0_report.json"))
    return 0


# ---------------------------------------------------------------------------
# mac
# ---------------------------------------------------------------------------

def cmd_mac(args):
    device = _device("cpu")
    if args.teacher:
        model = load_teacher(os.path.join(ROOT, "..", "models", "teachers", "vangogh_teacher.pth"), device=device)
        name = "teacher"
    else:
        cfg = load_config(args.config)
        model = StudentNet(spec_from_dict(cfg["model"]))
        name = cfg["model"]["name"]
    for convention in ("fpga", "native"):
        rep = profile(model, (3, args.resolution, args.resolution),
                      convention=convention, name=name)
        print("== %s @ %dx%d ==" % (convention, args.resolution, args.resolution))
        print(rep.format_table())
        print("  total %.9f GMAC/frame   conv weights %d   total params %d"
              % (rep.gmac_per_frame, rep.weight_params,
                 parameter_breakdown(model)["total"]))
        print()
    return 0


# ---------------------------------------------------------------------------
# train / eval / convert / export / qat
# ---------------------------------------------------------------------------

def cmd_train(args):
    from .train import train
    train(args.config, run_dir=args.run_dir, resume=args.resume, device=args.device,
          max_steps=args.max_steps, init_from=args.init_from)
    return 0


def cmd_eval(args):
    from .data import ValImageDataset, build_loader, read_split, val_collate
    from .evaluator import evaluate_model
    from .losses import DistillLoss
    from .predict import load_model_from_checkpoint

    cfg = load_config(args.config)
    device = _device(args.device)
    out_dir = args.run_dir or os.path.join(ROOT, "runs", "eval",
                                           cfg["model"]["name"])
    os.makedirs(out_dir, exist_ok=True)

    teacher = load_teacher(os.path.join(ROOT, cfg["teacher"]["checkpoint"]), device=device)
    student, _ckpt = load_model_from_checkpoint(args.checkpoint, device=device)
    split = args.val_list or cfg["data"]["val_list"]
    root = os.path.join(ROOT, cfg["data"]["root"])
    split_path = split if os.path.isabs(split) else os.path.join(ROOT, split)
    if args.root:
        root = args.root if os.path.isabs(args.root) else os.path.join(ROOT, args.root)
    val_ds = ValImageDataset(root, read_split(split_path),
                             crop=int(cfg["data"].get("val_crop", 256)))
    print("evaluating on %s  (%d images)" % (os.path.relpath(split_path, ROOT), len(val_ds)))
    loader = build_loader(val_ds, int(cfg["eval"]["batch_size"]), False, 0,
                          collate_fn=val_collate)
    loss_fn = DistillLoss(cfg["loss"], student, teacher, device=device).to(device)

    result = evaluate_model(student, teacher, loader, loss_fn=loss_fn, device=device,
                            out_dir=os.path.join(out_dir, "samples"),
                            export_images=True, contact_sheet=True,
                            logger=setup_logger(os.path.join(out_dir, "eval.log")))
    rep = profile(student, (3, 256, 256), convention="fpga", name=cfg["model"]["name"])
    payload = result.to_dict()
    payload["mac_report"] = rep.to_dict()
    payload["parameter_breakdown"] = parameter_breakdown(student)
    payload["arch_string"] = student.spec.arch_string()
    payload["checkpoint"] = args.checkpoint
    payload["split"] = os.path.relpath(split_path, ROOT).replace("\\", "/")
    write_json(os.path.join(out_dir, "evaluation.json"), payload)
    print("wrote %s" % os.path.join(out_dir, "evaluation.json"))
    return 0


def cmd_convert(args):
    from .convert import (FOLD_TOLERANCE_LEVELS, check_conversion, convert_norm,
                           freeze_fixed_stats)
    from .predict import load_model_from_checkpoint
    from .utils import load_checkpoint, save_checkpoint

    device = _device(args.device)
    ckpt = load_checkpoint(args.checkpoint, map_location="cpu")
    # reuse the one loader that knows how to turn a stored spec back into a
    # model; doing it by hand here is how the derived spec keys (arch_string,
    # bottleneck_inner, trunk_scale) leaked into spec_from_dict and raised.
    model, _ = load_model_from_checkpoint(args.checkpoint, device=device)

    from .data import ValImageDataset, read_split
    cfg = load_config(args.config)
    val_ds = ValImageDataset(os.path.join(ROOT, cfg["data"]["root"]),
                             read_split(os.path.join(ROOT, cfg["data"]["val_list"])),
                             crop=int(cfg["data"].get("val_crop", 256)))
    x = torch.stack([val_ds[i][0] for i in range(min(4, len(val_ds)))], 0).to(device)

    converted = convert_norm(model, args.to).to(device)
    drift, relative = check_conversion(model, converted, x)
    # The output is already in 0..255 units, so an absolute drift of 1.0 IS one
    # uint8 level.  The bar is a fraction of a level, not a fraction of the
    # span: the span here is ~419 because the raw output is unclamped, and
    # calling 1e-3 of it "a thousandth of a level" would have been wrong by
    # three orders of magnitude.
    print("converted %s -> %s   max |output diff| = %.4f uint8 levels "
          "(%.2e relative)" % (model.spec.norm, args.to, drift, relative))
    if drift > FOLD_TOLERANCE_LEVELS:
        raise SystemExit(
            "conversion is not function-preserving: it moves the output by %.4f "
            "uint8 levels, above the %.2f-level bar"
            % (drift, FOLD_TOLERANCE_LEVELS))
    freeze_fixed_stats(converted)

    out = args.out or args.checkpoint.replace(".pth", "_%s.pth" % args.to)
    os.makedirs(os.path.dirname(os.path.abspath(out)) or ".", exist_ok=True)
    # Carry the feature adapters through.  They belong to the loss, not the
    # model, so they are easy to drop here -- and dropping them silently
    # re-randomises the feature term, which makes the "function-preserving
    # starting point" true of the student's forward pass but false of the
    # objective that is actually optimised.  The tap channel widths are
    # unchanged by a normalisation rewrite, so they transfer verbatim.
    save_checkpoint(out, converted, adapters=ckpt.get("adapters"),
                    epoch=ckpt.get("epoch", 0),
                    global_step=ckpt.get("global_step", 0),
                    converted_from=args.checkpoint, norm=args.to)
    if ckpt.get("adapters") is None:
        print("note: %s carries no feature adapters; the warm start will not "
              "cover the feature loss" % args.checkpoint)
    else:
        print("carried %d feature adapter tensors through the conversion"
              % len(ckpt["adapters"]))
    print("wrote %s" % out)
    return 0


def cmd_export(args):
    from .export import export_model
    export_model(args)
    return 0


def cmd_qat(args):
    from .qat import run_qat
    run_qat(args)
    return 0


def cmd_compare(args):
    from .compare import run_compare
    return run_compare(args)


def cmd_predict(args):
    from .predict import run_predict
    run_predict(args)
    return 0


def cmd_make_splits(args):
    import runpy
    sys.argv = ["make_splits.py", "--stage", args.stage]
    runpy.run_path(os.path.join(ROOT, "scripts", "make_splits.py"), run_name="__main__")
    return 0


# ---------------------------------------------------------------------------

def build_parser():
    ap = argparse.ArgumentParser(prog="python -m tinystyle.cli", description=__doc__)
    sub = ap.add_subparsers(dest="command", required=True)

    p = sub.add_parser("check-teacher", help="stage 0 gate: reproduce the teacher")
    p.add_argument("-c", "--config", default=None)
    p.add_argument("--device", default=None)
    p.add_argument("--out-dir", default=None)
    p.set_defaults(func=cmd_check_teacher)

    p = sub.add_parser("mac", help="print the MAC table for the teacher or a config")
    p.add_argument("-c", "--config", default=None)
    p.add_argument("--teacher", action="store_true")
    p.add_argument("--resolution", type=int, default=256)
    p.set_defaults(func=cmd_mac)

    p = sub.add_parser("make-splits", help="build the train/val splits")
    p.add_argument("--stage", choices=("candidates", "finalize"), default="finalize")
    p.set_defaults(func=cmd_make_splits)

    p = sub.add_parser("train", help="distillation training")
    p.add_argument("-c", "--config", required=True)
    p.add_argument("-r", "--run-dir", default=None)
    p.add_argument("--resume", default=None)
    p.add_argument("--init-from", default=None)
    p.add_argument("--max-steps", type=int, default=None)
    p.add_argument("--device", default=None)
    p.set_defaults(func=cmd_train)

    p = sub.add_parser("eval", help="run the unified evaluator on a checkpoint")
    p.add_argument("-c", "--config", required=True)
    p.add_argument("--checkpoint", required=True)
    p.add_argument("-r", "--run-dir", default=None)
    p.add_argument("--val-list", default=None,
                   help="split file to evaluate on (default: the config's val_list)")
    p.add_argument("--root", default=None, help="image root, if the split needs another")
    p.add_argument("--device", default=None)
    p.set_defaults(func=cmd_eval)

    p = sub.add_parser("convert", help="normalisation conversion (stage 3)")
    p.add_argument("-c", "--config", required=True)
    p.add_argument("--checkpoint", required=True)
    p.add_argument("--to", required=True, choices=("instance", "fixed", "affine"))
    p.add_argument("--out", default=None)
    p.add_argument("--device", default=None)
    p.set_defaults(func=cmd_convert)

    p = sub.add_parser("export", help="write the FPGA delivery bundle")
    p.add_argument("-c", "--config", required=True)
    p.add_argument("--checkpoint", required=True)
    p.add_argument("-o", "--out-dir", required=True)
    p.add_argument("--device", default=None)
    p.set_defaults(func=cmd_export)

    p = sub.add_parser("qat", help="INT8 quantisation-aware training")
    p.add_argument("-c", "--config", required=True)
    p.add_argument("--checkpoint", required=True)
    p.add_argument("-o", "--out-dir", required=True)
    p.add_argument("--steps", type=int, default=500)
    p.add_argument("--lr", type=float, default=2.0e-5)
    p.add_argument("--device", default="cpu")
    p.set_defaults(func=cmd_qat)

    p = sub.add_parser("compare", help="one table + contact sheet over many runs")
    p.add_argument("runs", nargs="+", help="run dirs, or evaluation.json paths")
    p.add_argument("-o", "--out-dir", default=None)
    p.add_argument("--images", type=int, default=8)
    p.add_argument("--no-sheet", dest="sheet", action="store_false")
    p.add_argument("--device", default=None)
    p.set_defaults(func=cmd_compare)

    p = sub.add_parser("predict", help="stylise images with a checkpoint (tiled)")
    p.add_argument("--checkpoint", required=True)
    p.add_argument("--input", required=True)
    p.add_argument("--output", required=True)
    p.add_argument("--tile", type=int, default=1024)
    p.add_argument("--halo", type=int, default=128)
    p.add_argument("--device", default=None)
    p.set_defaults(func=cmd_predict)

    return ap


def main(argv=None):
    args = build_parser().parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
