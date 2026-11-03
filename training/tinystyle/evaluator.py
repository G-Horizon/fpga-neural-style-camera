# -*- coding: utf-8 -*-
"""The single evaluator every model goes through.

Training-time validation and standalone evaluation call the same function with
the same fixed content set, so a number in ``evaluation.json`` means the same
thing regardless of which stage produced it.  Metrics are computed at 256x256
against the live fp32 teacher, never against the uint8 training cache.
"""

from __future__ import annotations

import os
from collections import OrderedDict
from dataclasses import dataclass, field

import numpy as np
import torch
import torch.nn.functional as F
from PIL import Image, ImageDraw, ImageFont

from .losses import TapCollector

try:  # optional
    from skimage.metrics import structural_similarity as _ssim
except Exception:  # pragma: no cover
    _ssim = None

try:  # optional
    import lpips as _lpips
except Exception:  # pragma: no cover
    _lpips = None


DIFF_GAIN = 4.0


# ---------------------------------------------------------------------------
# metrics
# ---------------------------------------------------------------------------

def psnr(a: torch.Tensor, b: torch.Tensor, max_val: float = 255.0) -> float:
    mse = F.mse_loss(a, b).item()
    if mse <= 0:
        return float("inf")
    return float(20.0 * np.log10(max_val) - 10.0 * np.log10(mse))


def ssim(a: torch.Tensor, b: torch.Tensor) -> float:
    """Mean SSIM over the batch; quantises to uint8 as the display path does."""
    if _ssim is None:
        return float("nan")
    total, n = 0.0, 0
    an = a.detach().clamp(0, 255).round().to(torch.uint8).cpu().numpy()
    bn = b.detach().clamp(0, 255).round().to(torch.uint8).cpu().numpy()
    for i in range(an.shape[0]):
        total += _ssim(np.transpose(an[i], (1, 2, 0)), np.transpose(bn[i], (1, 2, 0)),
                       channel_axis=2, data_range=255)
        n += 1
    return total / max(n, 1)


class LpipsMetric:
    """LPIPS if the optional package is installed, otherwise a no-op."""

    def __init__(self, device="cpu"):
        self.net = None
        if _lpips is not None:
            try:
                self.net = _lpips.LPIPS(net="alex", verbose=False).to(device).eval()
                for p in self.net.parameters():
                    p.requires_grad_(False)
            except Exception:
                self.net = None

    @property
    def available(self) -> bool:
        return self.net is not None

    @torch.no_grad()
    def __call__(self, a: torch.Tensor, b: torch.Tensor) -> float:
        if self.net is None:
            return float("nan")
        a = (a.clamp(0, 255) / 127.5 - 1.0)
        b = (b.clamp(0, 255) / 127.5 - 1.0)
        return float(self.net(a, b).mean())


# ---------------------------------------------------------------------------
# result container
# ---------------------------------------------------------------------------

@dataclass
class EvalResult:
    n: int = 0
    metrics: "OrderedDict[str, float]" = field(default_factory=OrderedDict)
    per_image: list = field(default_factory=list)

    def to_dict(self):
        return OrderedDict([
            ("n_images", self.n),
            ("metrics", self.metrics),
            ("per_image", self.per_image),
        ])


# ---------------------------------------------------------------------------
# evaluation
# ---------------------------------------------------------------------------

@torch.no_grad()
def evaluate_model(model, teacher, loader, loss_fn=None, device="cuda",
                   out_dir: str = None, export_images: bool = True,
                   contact_sheet: bool = True, max_export: int = 32,
                   lpips_metric=None, logger=None) -> EvalResult:
    """Run the model and the teacher over the fixed validation loader."""
    model.eval()
    teacher.eval()
    if lpips_metric is None:
        lpips_metric = LpipsMetric(device)

    sums = OrderedDict((k, 0.0) for k in
                       ("output_l1", "output_charbonnier", "psnr", "ssim", "lpips",
                        "perceptual", "style", "hf", "feature"))
    loss_parts_sum = OrderedDict()
    per_image = []
    exported = []
    n_total = 0

    if export_images and out_dir:
        os.makedirs(out_dir, exist_ok=True)

    for content, names in loader:
        content = content.to(device, non_blocking=True)
        with TapCollector(teacher) as tc:
            teacher_out = teacher(content)
        with TapCollector(model) as sc:
            student_out = model(content)

        # Metrics describe the delivered image, so both sides are clamped to
        # the uint8 range the product outputs.  The teacher's raw output reaches
        # 1239 and -527; scoring against those would understate every model.
        student_out = student_out.clamp(0, 255)
        teacher_out = teacher_out.clamp(0, 255)
        l1 = F.l1_loss(student_out, teacher_out).item()
        char = torch.sqrt((student_out - teacher_out) ** 2 + 1e-6).mean().item()
        batch = content.shape[0]
        sums["output_l1"] += l1 * batch
        sums["output_charbonnier"] += char * batch
        sums["psnr"] += psnr(student_out, teacher_out) * batch
        sums["ssim"] += ssim(student_out, teacher_out) * batch
        if lpips_metric.available:
            sums["lpips"] += lpips_metric(student_out, teacher_out) * batch

        if loss_fn is not None:
            _total, parts = loss_fn(student_out, teacher_out, sc.captured, tc.captured)
            for k, (raw, _w) in parts.items():
                if k in sums:
                    sums[k] += float(raw) * batch
                loss_parts_sum[k] = loss_parts_sum.get(k, 0.0) + float(raw) * batch

        for i in range(batch):
            # Per-image, not just batch-averaged: deciding between two models
            # that differ by under a point of L1 needs a paired test over images,
            # and a paired test needs the per-image values.
            row = OrderedDict([
                ("name", names[i]),
                ("l1", float(F.l1_loss(student_out[i], teacher_out[i]).item())),
                ("psnr", psnr(student_out[i:i + 1], teacher_out[i:i + 1])),
                ("ssim", ssim(student_out[i:i + 1], teacher_out[i:i + 1])),
            ])
            if lpips_metric.available:
                row["lpips"] = lpips_metric(student_out[i:i + 1], teacher_out[i:i + 1])
            per_image.append(row)
            if export_images and out_dir and len(exported) < max_export:
                exported.append((names[i], content[i].cpu(), teacher_out[i].cpu(),
                                 student_out[i].cpu()))

        n_total += batch

    metrics = OrderedDict()
    for k, v in sums.items():
        if k == "lpips" and not lpips_metric.available:
            metrics["lpips"] = float("nan")
            continue
        metrics[k] = float(v / max(n_total, 1))
    for k, v in loss_parts_sum.items():
        if k not in metrics:
            metrics["loss_" + k] = float(v / max(n_total, 1))

    if logger:
        logger.info("val %s", "  ".join("%s=%.4f" % (k, v) for k, v in metrics.items()
                                        if not np.isnan(v)))

    result = EvalResult(n=n_total, metrics=metrics, per_image=per_image)

    if export_images and out_dir and exported:
        _export_comparisons(exported, out_dir, contact_sheet=contact_sheet)
    model.train()
    return result


# ---------------------------------------------------------------------------
# image export
# ---------------------------------------------------------------------------

def _to_uint8(t: torch.Tensor) -> np.ndarray:
    return t.clamp(0, 255).round().to(torch.uint8).permute(1, 2, 0).numpy()


def _panel_strip(content, teacher_out, student_out, diff_gain=DIFF_GAIN):
    c = _to_uint8(content)
    t = _to_uint8(teacher_out)
    s = _to_uint8(student_out)
    d = np.clip(np.abs(s.astype(np.float32) - t.astype(np.float32)) * diff_gain,
                0, 255).astype(np.uint8)
    return np.concatenate([c, t, s, d], axis=1)


def _export_comparisons(exported, out_dir, contact_sheet=True):
    """Write ``content | teacher | student | 4x|diff|`` strips and a grid."""
    panel_h = panel_w = None
    cell = 128
    labels = ["content", "teacher", "student", "|diff| x%d" % int(DIFF_GAIN)]
    rows = []

    for name, content, teacher_out, student_out in exported:
        strip = _panel_strip(content, teacher_out, student_out)
        panel_h, panel_w = strip.shape[0], strip.shape[1] // 4
        stem = os.path.splitext(os.path.basename(name))[0]
        Image.fromarray(strip).save(os.path.join(out_dir, "%s_compare.png" % stem))
        rows.append((stem, strip))

    if not contact_sheet or not rows:
        return

    scale = cell / panel_h
    small_h = cell
    small_w = int(round(panel_w * scale))
    bar = 18
    pad = 4
    W = pad + 4 * (small_w + pad)
    H = bar + pad + len(rows) * (small_h + pad)
    sheet = Image.new("RGB", (W, pad + H), (22, 22, 26))
    draw = ImageDraw.Draw(sheet)
    try:
        font = ImageFont.load_default(size=13)
    except TypeError:  # pragma: no cover
        font = ImageFont.load_default()
    for j, label in enumerate(labels):
        draw.text((pad + j * (small_w + pad) + 3, 3), label, (235, 235, 235), font=font)

    for i, (stem, strip) in enumerate(rows):
        im = Image.fromarray(strip).resize((4 * small_w, small_h), Image.LANCZOS)
        sheet.paste(im, (pad, bar + pad + i * (small_h + pad)))
    sheet.save(os.path.join(out_dir, "contact_sheet.png"))


def stylize_tiled(model, tensor: torch.Tensor, tile: int = 1024, halo: int = 128):
    """Whole-image stylisation in tiles; exact for fixed-statistic models."""
    from .tiling import stylize_tiled as _impl
    return _impl(model, tensor, tile=tile, halo=halo)
