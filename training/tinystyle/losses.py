# -*- coding: utf-8 -*-
"""Distillation losses.

The student is never trained with the plain style-transfer objective from
scratch.  The frozen teacher turns every content image into a deterministic
target, so the dominant term is a reconstruction loss against ``T(x)``; the
perceptual, Gram, feature, high-frequency and TV terms exist to stop a small
student from collapsing to the mean colour field that pixel loss alone rewards.

Every term is reported raw *and* weighted so the weighting can be tuned from
measurements instead of guesswork -- see :meth:`DistillLoss.describe`.
"""

from __future__ import annotations


from collections import OrderedDict

import torch
import torch.nn as nn
import torch.nn.functional as F

from .dwt import high_frequency_bands

IMAGENET_MEAN = (0.485, 0.456, 0.406)
IMAGENET_STD = (0.229, 0.224, 0.225)

#: name -> number of ``vgg16.features`` blocks to run for that activation
VGG_LAYER_SLICES = OrderedDict([
    ("relu1_2", 4),
    ("relu2_2", 9),
    ("relu3_3", 16),
    ("relu4_3", 23),
])


# ---------------------------------------------------------------------------
# feature network
# ---------------------------------------------------------------------------

class VGGFeatures(nn.Module):
    """Frozen VGG16 trunk returning named intermediate activations.

    Input is the project's 0..255 tensor convention; scaling and ImageNet
    normalisation happen inside so callers never have to think about it.
    """

    def __init__(self, layers=("relu2_2", "relu3_3", "relu4_3"), device="cpu"):
        super().__init__()
        import torchvision
        weights = torchvision.models.VGG16_Weights.IMAGENET1K_V1
        features = torchvision.models.vgg16(weights=weights).features
        self.layers = tuple(layers)
        max_idx = max(VGG_LAYER_SLICES[l] for l in self.layers)
        self.trunk = features[:max_idx]
        self.trunk.eval()
        for p in self.trunk.parameters():
            p.requires_grad_(False)
        self.register_buffer("mean", torch.tensor(IMAGENET_MEAN).reshape(1, 3, 1, 1))
        self.register_buffer("std", torch.tensor(IMAGENET_STD).reshape(1, 3, 1, 1))
        self.to(device)

    def forward(self, x: torch.Tensor) -> "OrderedDict[str, torch.Tensor]":
        x = (x / 255.0 - self.mean) / self.std
        out = OrderedDict()
        # VGG_LAYER_SLICES holds the exclusive slice end, so the activation sits
        # at end-1 within the trunk.
        wanted = {VGG_LAYER_SLICES[l] - 1: l for l in self.layers}
        for i, block in enumerate(self.trunk):
            x = block(x)
            if i in wanted:
                out[wanted[i]] = x
        missing = set(self.layers) - set(out)
        if missing:
            raise RuntimeError("VGG taps not produced: %s" % sorted(missing))
        return out


# ---------------------------------------------------------------------------
# individual terms
# ---------------------------------------------------------------------------

DISPLAY_MIN, DISPLAY_MAX = 0.0, 255.0


def clamp_display(x: torch.Tensor) -> torch.Tensor:
    """The range the product actually delivers (uint8 sRGB)."""
    return x.clamp(DISPLAY_MIN, DISPLAY_MAX)


def clamp_ste(x: torch.Tensor) -> torch.Tensor:
    """Clamp for the forward value, identity for the gradient.

    The candy teacher's raw output runs from -527 to 1239, and 9.3% of its
    pixels fall outside 0..255.  Those values are unreachable by the deployed
    model, which clamps to uint8, so training against them is asking for
    something undeliverable -- and under L1 that pushes the student toward the
    conditional *median*, which shows up as a systematic darkening of about 17
    levels.  Clamping both sides into the display range removes the incentive.

    The straight-through form is used for the student so that gradients still
    flow when its own output is momentarily out of range; a hard clamp would
    zero them and could stall recovery.
    """
    return x + (clamp_display(x) - x).detach()


def charbonnier(pred, target, eps: float = 1e-3):
    diff = pred - target
    return torch.sqrt(diff * diff + eps * eps).mean()


def gram(f: torch.Tensor) -> torch.Tensor:
    n, c, h, w = f.shape
    flat = f.reshape(n, c, h * w)
    return torch.bmm(flat, flat.transpose(1, 2)) / (c * h * w)


def total_variation(x: torch.Tensor) -> torch.Tensor:
    dh = (x[:, :, 1:, :] - x[:, :, :-1, :]).abs().mean()
    dw = (x[:, :, :, 1:] - x[:, :, :, :-1]).abs().mean()
    return dh + dw


class FeatureAdapters(nn.Module):
    """Training-only 1x1 projections from student channels to teacher channels.

    These exist purely to make the feature loss dimensionally legal; they are
    deleted before export and never reach the FPGA.
    """

    def __init__(self, pairs, mode: str = "cosine", hidden: int = None):
        super().__init__()
        self.mode = mode
        self.adapters = nn.ModuleDict()
        for tap, (s_ch, t_ch) in pairs.items():
            self.adapters[tap] = nn.Conv2d(s_ch, t_ch, 1, bias=True)
        self.adapters.to(torch.device("cuda" if torch.cuda.is_available() else "cpu"))

    def forward(self, tap: str, s: torch.Tensor, t: torch.Tensor) -> torch.Tensor:
        s = self.adapters[tap](s.float())
        t = t.float()
        if s.shape[-2:] != t.shape[-2:]:
            # Only reachable for the 32x32-trunk student: low-pass the teacher
            # rather than asking a downsampled map to reproduce detail that is
            # not representable.
            t = F.adaptive_avg_pool2d(t, s.shape[-2:])
        if self.mode == "cosine":
            return (1.0 - F.cosine_similarity(s, t, dim=1, eps=1e-6)).mean()
        if self.mode == "l1":
            return F.l1_loss(s, t)
        if self.mode == "mse":
            return F.mse_loss(s, t)
        raise ValueError("unknown feature loss mode %r" % (self.mode,))


# ---------------------------------------------------------------------------
# tap collection
# ---------------------------------------------------------------------------

class TapCollector:
    """Context manager capturing the outputs of ``model.tap_modules()``.

    A model with no taps -- an FX graph module after quantisation, say -- simply
    captures nothing, which is correct as long as the loss that consumes them is
    disabled.
    """

    def __init__(self, model):
        self.model = model
        self.modules = model.tap_modules() if hasattr(model, "tap_modules") else OrderedDict()
        self.captured = OrderedDict()
        self._handles = []

    def __enter__(self):
        for name, module in self.modules.items():
            self._handles.append(
                module.register_forward_hook(self._make(name)))
        return self

    def _make(self, name):
        def hook(module, inputs, output):
            self.captured[name] = output
        return hook

    def __exit__(self, *exc):
        for h in self._handles:
            h.remove()
        self._handles = []
        return False


# ---------------------------------------------------------------------------
# combined loss
# ---------------------------------------------------------------------------

DEFAULT_LOSS = {
    "output": {"kind": "charbonnier", "weight": 1.0, "eps": 0.001},
    "perceptual": {"weight": 0.5, "layers": ["relu1_2", "relu2_2", "relu3_3"],
                   "weights": [1.0, 1.0, 1.0]},
    "style": {"weight": 2.0, "layers": ["relu1_2", "relu2_2", "relu3_3"],
              "weights": [1.0, 1.0, 1.0], "normalize": "relative"},
    "feature": {"weight": 2.0, "taps": ["down1", "trunk_in", "trunk_mid",
                                        "trunk_out", "pre_tail"],
                "tap_weights": [1.0, 1.0, 1.0, 1.0, 1.0], "mode": "cosine"},
    "hf": {"weight": 0.1, "levels": 1},
    "tv": {"weight": 0.0},
}


class DistillLoss(nn.Module):
    """Weighted sum of every distillation term.

    ``forward(student_out, teacher_out, student_feats, teacher_feats)`` returns
    ``(total, parts)`` where ``parts`` maps each enabled term to
    ``(raw, weighted)``.
    """

    def __init__(self, cfg: dict, student: nn.Module, teacher: nn.Module, device="cpu"):
        super().__init__()
        cfg = _disable_feature_if_no_taps(cfg, student)
        self.cfg = cfg
        self.device = device
        self.terms = OrderedDict()
        self.raw = OrderedDict()

        self.output_cfg = cfg.get("output", DEFAULT_LOSS["output"])
        self.perc_cfg = cfg.get("perceptual", DEFAULT_LOSS["perceptual"])
        self.style_cfg = cfg.get("style", DEFAULT_LOSS["style"])
        self.feat_cfg = cfg.get("feature", DEFAULT_LOSS["feature"])
        self.hf_cfg = cfg.get("hf", DEFAULT_LOSS["hf"])
        self.tv_cfg = cfg.get("tv", DEFAULT_LOSS["tv"])

        vgg_layers = sorted(set(self.perc_cfg.get("layers", [])) |
                            set(self.style_cfg.get("layers", [])),
                            key=lambda n: VGG_LAYER_SLICES[n])
        self.vgg = VGGFeatures(vgg_layers, device=device) if (
            self.perc_cfg.get("weight", 0) or self.style_cfg.get("weight", 0)) else None

        self.adapters = None
        if self.feat_cfg.get("weight", 0):
            s_ch = student.tap_channels()
            t_ch = teacher.tap_channels()
            pairs = {tap: (s_ch[tap], t_ch[tap]) for tap in self.feat_cfg["taps"]}
            self.adapters = FeatureAdapters(pairs, mode=self.feat_cfg.get("mode", "cosine"))

    # -- helpers ----------------------------------------------------------
    @staticmethod
    def _w(section, key="weight"):
        return float(section.get(key, 0.0))

    def forward(self, student_out, teacher_out, student_feats, teacher_feats):
        parts = OrderedDict()

        # Everything is measured in the display range: the delivered image is
        # clamp(x, 0, 255), so that is the space the loss and the metrics live in.
        student_out = clamp_ste(student_out)
        teacher_out = clamp_display(teacher_out)

        w = self._w(self.output_cfg)
        if w:
            kind = self.output_cfg.get("kind", "charbonnier")
            if kind == "charbonnier":
                raw = charbonnier(student_out, teacher_out, self.output_cfg.get("eps", 1e-3))
            elif kind == "l1":
                raw = F.l1_loss(student_out, teacher_out)
            else:
                raise ValueError("unknown output loss %r" % (kind,))
            parts["output"] = (raw, w * raw)

        if self.vgg is not None:
            with torch.no_grad():
                t_feats = self.vgg(teacher_out.detach())
            s_feats = self.vgg(student_out)

            w = self._w(self.perc_cfg)
            if w:
                layers = self.perc_cfg["layers"]
                lw = self.perc_cfg.get("weights", [1.0] * len(layers))
                raw = sum(float(a) * F.l1_loss(s_feats[l], t_feats[l])
                          for a, l in zip(lw, layers)) / max(sum(lw), 1e-9)
                parts["perceptual"] = (raw, w * raw)

            w = self._w(self.style_cfg)
            if w:
                layers = self.style_cfg["layers"]
                lw = self.style_cfg.get("weights", [1.0] * len(layers))
                rel = self.style_cfg.get("normalize", "relative") == "relative"
                terms = []
                for a, l in zip(lw, layers):
                    gs, gt = gram(s_feats[l]), gram(t_feats[l])
                    d = F.l1_loss(gs, gt)
                    if rel:
                        # Gram entries scale with the layer's activation
                        # magnitude, so an absolute L1 is not comparable across
                        # layers or models.  Dividing by the target's own mean
                        # magnitude makes the term "fractional style error" and
                        # its weight interpretable.
                        d = d / (gt.detach().abs().mean() + 1e-8)
                    terms.append(float(a) * d)
                raw = sum(terms) / max(sum(lw), 1e-9)
                parts["style"] = (raw, w * raw)

        w = self._w(self.feat_cfg)
        if w and self.adapters is not None:
            taps = self.feat_cfg["taps"]
            tw = self.feat_cfg.get("tap_weights", [1.0] * len(taps))
            raw = sum(
                float(a) * self.adapters(t, student_feats[t], teacher_feats[t])
                for a, t in zip(tw, taps)
            ) / max(sum(tw), 1e-9)
            parts["feature"] = (raw, w * raw)

        w = self._w(self.hf_cfg)
        if w:
            levels = int(self.hf_cfg.get("levels", 1))
            s_bands = high_frequency_bands(student_out, levels)
            with torch.no_grad():
                t_bands = high_frequency_bands(teacher_out.detach(), levels)
            raw = sum(F.l1_loss(a, b) for a, b in zip(s_bands, t_bands)) / len(s_bands)
            parts["hf"] = (raw, w * raw)

        w = self._w(self.tv_cfg)
        if w:
            raw = total_variation(student_out)
            parts["tv"] = (raw, w * raw)

        total = sum(v[1] for v in parts.values())
        self.raw = parts
        return total, parts

    # -- diagnostics ------------------------------------------------------
    def describe(self, parts) -> str:
        cells = []
        for name, (raw, weighted) in parts.items():
            share = 100.0 * float(weighted) / max(
                float(sum(v[1] for v in parts.values())), 1e-12)
            cells.append("%s raw=%.4f w=%.4f (%.1f%%)"
                         % (name, float(raw), float(weighted), share))
        return "  ".join(cells)


def _disable_feature_if_no_taps(cfg: dict, student: nn.Module) -> dict:
    """Drop the feature term for models that do not expose taps.

    The post-QAT graph is an FX module with no ``tap_modules``; the feature
    adapters cannot attach to it, and the frozen teacher's taps have nothing to
    pair with.  Every other term is unaffected.
    """
    if hasattr(student, "tap_channels"):
        return cfg
    feat = cfg.get("feature", {})
    if feat.get("weight", 0):
        import warnings
        warnings.warn("student exposes no taps; disabling the feature loss term "
                      "(was weight=%s)" % feat["weight"])
        cfg = dict(cfg)
        cfg["feature"] = dict(feat, weight=0.0)
    return cfg
