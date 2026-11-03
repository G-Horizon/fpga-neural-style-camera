# -*- coding: utf-8 -*-
"""Cross-experiment comparison.

The project's whole point is picking a work point, and that means looking at
several models side by side on the *same* frozen validation set.  This module
collects the ``evaluation.json`` of any number of runs into one table and one
contact sheet, so the decision is made from a single picture rather than from
scrollback.
"""

from __future__ import annotations

import glob
import json
import os
from collections import OrderedDict

import numpy as np
import torch
from PIL import Image, ImageDraw, ImageFont

from .data import ValImageDataset, build_loader, load_rgb_tensor, read_split, val_collate
from .evaluator import DIFF_GAIN, _to_uint8
from .models.teacher import load_teacher
from .predict import load_model_from_checkpoint
from .utils import ROOT, write_json

COLUMNS = [
    ("model_name", "model", "%-22s"),
    ("flag", "", "%s"),
    ("gmac", "GMAC", "%9.4f"),
    ("params", "params", "%9.0f"),
    ("fp32_kb", "FP32 KB", "%8.1f"),
    ("int8_kb", "INT8 KB", "%8.1f"),
    ("norm", "norm", "%-9s"),
    ("output_l1", "val L1", "%8.3f"),
    ("psnr", "PSNR", "%7.2f"),
    ("ssim", "SSIM", "%7.4f"),
    ("lpips", "LPIPS", "%7.4f"),
    ("perceptual", "perc", "%8.4f"),
    ("style", "style", "%8.4f"),
    ("hf", "hf", "%8.4f"),
]


def _row_from_evaluation(path: str) -> dict:
    with open(path, encoding="utf-8") as fh:
        ev = json.load(fh)
    mac = ev.get("mac_report", {})
    if not mac:
        # runs written before evaluation.json carried the budget: fall back to
        # the mac_report.json sitting next to it
        sibling = os.path.join(os.path.dirname(path), "mac_report.json")
        if os.path.isfile(sibling):
            with open(sibling, encoding="utf-8") as fh:
                mac = json.load(fh)
    m = ev.get("metrics", {})
    run_dir = os.path.dirname(path)
    name, norm_hint = None, None
    if not ev.get("arch_string"):
        # Older runs predate evaluation.json carrying the architecture; the
        # fully merged config next to it is the authoritative record.
        cfg_path = os.path.join(run_dir, "config.json")
        if os.path.isfile(cfg_path):
            with open(cfg_path, encoding="utf-8") as fh:
                model_cfg = json.load(fh).get("model", {})
            name = model_cfg.get("name")
            norm_hint = model_cfg.get("norm")
    params = (mac.get("parameter_breakdown") or {}).get(
        "total", mac.get("conv_params_total", 0)) or 0
    return OrderedDict([
        ("model_name", ev.get("arch_string") or name
         or ev.get("model_name") or os.path.basename(run_dir)),
        ("path", path),
        ("gmac", mac.get("gmac_per_frame", float("nan"))),
        ("params", params),
        ("norm", ev.get("norm_type") or norm_hint or "?"),
        ("fp32_kb", params * 4 / 1024.0),
        ("int8_kb", params / 1024.0),
        ("output_l1", m.get("output_l1", float("nan"))),
        ("psnr", m.get("psnr", float("nan"))),
        ("ssim", m.get("ssim", float("nan"))),
        ("lpips", m.get("lpips", float("nan"))),
        ("perceptual", m.get("perceptual", float("nan"))),
        ("style", m.get("style", float("nan"))),
        ("hf", m.get("hf", float("nan"))),
    ])


def find_evaluations(run_roots):
    """Accept run directories, timestamped run directories, or evaluation.json paths."""
    found = []
    for root in run_roots:
        if os.path.isfile(root) and root.endswith(".json"):
            found.append(root)
            continue
        direct = os.path.join(root, "evaluation.json")
        if os.path.isfile(direct):
            found.append(direct)
        else:
            found.extend(sorted(glob.glob(os.path.join(root, "*", "evaluation.json"))))
    return found


#: normalisations whose training function differs from the deployed one.
CONTAMINATED_NORMS = {"instance"}


def mark_pareto(rows, keys=("gmac", "params", "output_l1")):
    """Flag rows not dominated on (MAC, params, L1) -- all three minimised.

    Only *clean* rows may hold a Pareto point or dominate one.  A run whose
    training function differs from its deployed one (``norm=instance``) has a
    quality figure that is not its capacity limit, so letting it dominate would
    mark a genuinely better model as dominated for no reason -- and as the
    sweeps grow this table becomes the main decision tool, so that error would
    compound.
    """
    def val(r, k):
        v = r.get(k)
        return float("inf") if v is None or v != v else float(v)

    clean = [r for r in rows if r.get("norm") not in CONTAMINATED_NORMS]
    contaminated = [r for r in rows if r.get("norm") in CONTAMINATED_NORMS]

    for r in rows:
        r["pareto"] = False
    for a in clean:
        a["pareto"] = True
    for a in clean:
        for b in clean:
            if a is b:
                continue
            if all(val(b, k) <= val(a, k) for k in keys) and                any(val(b, k) < val(a, k) for k in keys):
                a["pareto"] = False
                a["dominated_by"] = b["model_name"]
                break

    for r in contaminated:
        r["pareto"] = None          # not eligible, not a point
        r["excluded"] = ("trained on a different function than it is scored and "
                         "deployed with")
    return rows


def format_table(rows) -> str:
    header = [h for _, h, _ in COLUMNS]
    body = [[fmt % row[key] for key, _, fmt in COLUMNS] for row in rows]
    widths = [max(len(header[i]), *(len(r[i]) for r in body)) if body else len(header[i])
              for i in range(len(header))]
    lines = ["  ".join(h.ljust(widths[i]) for i, h in enumerate(header)),
             "  ".join("-" * w for w in widths)]
    lines += ["  ".join(c.ljust(widths[i]) for i, c in enumerate(r)) for r in body]
    return "\n".join(lines)


@torch.no_grad()
def comparison_sheet(rows, out_path, teacher_ckpt, n_images=8, panel=160,
                     crop=256, device="cuda"):
    """One row per model: teacher on top, student below, for the same content."""
    ds = ValImageDataset(os.path.join(ROOT, "data", "coco_val2017"),
                         read_split(os.path.join(ROOT, "data", "splits", "val_fixed.txt")),
                         crop=crop)
    loader = build_loader(ds, 1, False, 0, collate_fn=val_collate)
    teacher = load_teacher(teacher_ckpt, device=device)
    models = []
    for row in rows:
        ckpt = os.path.join(os.path.dirname(row["path"]), "checkpoints", "best.pth")
        if not os.path.isfile(ckpt):
            ckpt = None
        if ckpt:
            model, _ = load_model_from_checkpoint(ckpt, device=device)
            models.append((row["model_name"], model))
        else:
            models.append((row["model_name"], None))

    picks = list(range(min(n_images, len(ds))))
    contents, teacher_outs = [], []
    for idx in picks:
        x, _ = ds[idx]
        x = x.unsqueeze(0).to(device)
        contents.append(x[0].cpu())
        teacher_outs.append(teacher(x)[0].cpu())

    n_models = len(models)
    bar = 20
    label_w = 150
    W = label_w + len(picks) * (panel + 4)
    H = (n_models * 2 + 1) * (panel + 4) + bar
    sheet = Image.new("RGB", (W, H), (20, 20, 24))
    draw = ImageDraw.Draw(sheet)
    try:
        font = ImageFont.load_default(size=12)
    except TypeError:  # pragma: no cover
        font = ImageFont.load_default()

    def paste(img, cx, cy, label=None):
        im = Image.fromarray(img).resize((panel, panel), Image.LANCZOS)
        sheet.paste(im, (cx, cy))
        if label:
            draw.text((cx + 2, cy + panel - 14), label[:26], (255, 235, 180), font=font)

    cy = bar
    draw.text((4, 3), "teacher output (all rows share this content)", (235, 235, 235),
              font=font)
    for j, t in enumerate(teacher_outs):
        paste(_to_uint8(t), label_w + j * (panel + 4), cy)
    cy += panel + 4

    for name, model in models:
        draw.text((4, cy + panel // 2), name[:24], (235, 235, 235), font=font)
        for j in range(len(picks)):
            cx = label_w + j * (panel + 4)
            paste(_to_uint8(contents[j]), cx, cy, label="content")
            if model is None:
                continue
            out = model(contents[j].unsqueeze(0).to(device))[0].cpu()
            paste(_to_uint8(out), cx, cy + panel + 4, label="student")
        cy += 2 * (panel + 4)

    sheet.save(out_path)
    return out_path


def run_compare(args):
    paths = find_evaluations(args.runs)
    if not paths:
        raise SystemExit("no evaluation.json found under: %s" % args.runs)
    rows = [_row_from_evaluation(p) for p in paths]
    rows.sort(key=lambda r: r["gmac"])
    mark_pareto(rows)
    for r in rows:
        if r.get("pareto") is None:
            r["flag"] = "!"            # contaminated: shown, never ranked
        else:
            r["flag"] = "P" if r["pareto"] else "x"

    print(format_table(rows))
    print()
    print("flag: P = on the (GMAC, params, L1) Pareto front, among clean rows only")
    print("      x = clean but dominated by another clean row")
    print("      ! = norm=%s -- trained on a different function than it is"
          % "/".join(sorted(CONTAMINATED_NORMS)))
    print("          scored and deployed with (train/eval norm mismatch). Shown")
    print("          for reference; excluded from ranking, may not dominate and")
    print("          may not hold a Pareto point.")
    out_dir = args.out_dir or os.path.join(ROOT, "runs", "_compare")
    os.makedirs(out_dir, exist_ok=True)
    write_json(os.path.join(out_dir, "comparison.json"),
               OrderedDict([("rows", rows),
                            ("columns", [c[0] for c in COLUMNS])]))
    with open(os.path.join(out_dir, "comparison.txt"), "w", encoding="utf-8") as fh:
        fh.write(format_table(rows) + "\n")
    print("\nwrote %s" % os.path.join(out_dir, "comparison.txt"))

    if args.sheet:
        device = torch.device(args.device) if args.device else torch.device(
            "cuda" if torch.cuda.is_available() else "cpu")
        from .utils import load_config
        cfg = load_config(os.path.join(ROOT, "configs", "base.yaml"),
                          base_path=os.path.join(ROOT, "configs", "base.yaml"))
        path = comparison_sheet(rows, os.path.join(out_dir, "comparison_sheet.png"),
                                os.path.join(ROOT, cfg["teacher"]["checkpoint"]),
                                n_images=args.images, device=device)
        print("wrote %s" % path)
    return 0
