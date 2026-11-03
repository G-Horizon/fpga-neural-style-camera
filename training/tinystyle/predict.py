# -*- coding: utf-8 -*-
"""Whole-image stylisation with a trained checkpoint, in tiles."""

from __future__ import annotations

import os
import time

import torch
from PIL import Image

from .data import load_rgb_tensor
from .models.student import StudentNet, spec_from_dict
from .tiling import stylize_tiled
from .utils import load_checkpoint


def load_model_from_checkpoint(path, device="cuda", eval_mode=True):
    ckpt = load_checkpoint(path, map_location="cpu")
    spec = ckpt.get("spec")
    if spec is None:
        raise ValueError("%s has no architecture spec; it may be a raw state dict" % path)
    spec = dict(spec)
    for key in ("bottleneck_inner", "arch_string", "trunk_scale"):
        spec.pop(key, None)
    model = StudentNet(spec_from_dict(spec))
    model.load_state_dict(ckpt["model"])
    model.to(device)
    if eval_mode:
        model.eval()
    return model, ckpt


def run_predict(args):
    device = torch.device(args.device) if args.device else torch.device(
        "cuda" if torch.cuda.is_available() else "cpu")
    model, _ckpt = load_model_from_checkpoint(args.checkpoint, device=device)
    os.makedirs(args.output, exist_ok=True)

    names = sorted(f for f in os.listdir(args.input)
                   if f.lower().endswith((".jpg", ".jpeg", ".png", ".bmp", ".webp")))
    if not names:
        raise SystemExit("no images in %s" % args.input)

    for i, name in enumerate(names, 1):
        started = time.time()
        x = load_rgb_tensor(os.path.join(args.input, name)).unsqueeze(0).to(device)
        _, _, h, w = x.shape
        with torch.no_grad():
            y = stylize_tiled(model, x, tile=args.tile, halo=args.halo)
        arr = (y[0, :, :h, :w].clamp(0, 255).round().to(torch.uint8)
               .permute(1, 2, 0).cpu().numpy())
        dst = os.path.join(args.output, os.path.splitext(name)[0] + ".png")
        Image.fromarray(arr).save(dst)
        print("[%d/%d] %s  %dx%d  %.1fs -> %s"
              % (i, len(names), name, w, h, time.time() - started, dst))
    return 0
