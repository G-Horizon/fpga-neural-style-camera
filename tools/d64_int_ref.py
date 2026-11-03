#!/usr/bin/env python3
"""The d64 integer reference: the arithmetic the RTL has to reproduce exactly.

Why an integer model and not just quantised weights
---------------------------------------------------

Weights alone do not pin a fixed-point datapath.  Two implementations can share
every weight and every scale and still disagree on every output, because the
things that decide the result are the ones nobody writes down: what the zero
point is subtracted from, whether the requantisation rounds before or after the
shift, whether the shift is arithmetic, where saturation happens relative to the
ReLU, and whether the residual add is one integer add or two requants.

So this module is written the way the hardware will be, in int64 numpy, and it
emits a tap at every stage the RTL will have a register at.  When the RTL
disagrees, the first tap that differs says which stage is wrong:

    pre-requant accumulator   -- the MAC array, the weight layout, the bias
    post-requant activation   -- the multiplier, the shift, the rounding, the clamp

The contracts this fixes, each of which the RTL must match exactly
------------------------------------------------------------------

**The MAC operand is ``q - zero``, not ``q``.**  Subtracting the zero point from
the stored value, rather than folding ``-zero * sum(w)`` into the bias, is a
deliberate choice: the fold is free for a normal convolution but not for a
transposed one, whose output pixel only receives the kernel taps whose parity
matches and, at the borders, not even all of those.  One uniform rule beats one
clever rule and one exception.

**Requantisation is** ``(acc * M + (1 << (shift-1))) >> shift``  with an
arithmetic shift.  That is round-half-up on ties and floor otherwise; it is not
the same as round-half-away-from-zero, and the difference is visible on exactly
the kind of value that makes a golden diff look like a bug.

**Saturation happens at the output format, and the ReLU is a clamp at the zero
point.**  For a uint8 ReLU tensor the two coincide, since the zero point is 0 --
which is why those layers need one clamp and not two.

**The residual add is** ``q_out = q_skip + q_conv3 - zero``.  Both inputs are
already in the trunk stream's format, so there is no requant between them.  In
``--residual dual`` there is, and that is the whole difference between the two
strategies.

Usage
-----

    python tools/d64_int_ref.py --quality              # int8 vs float, on the golden input
    python tools/d64_int_ref.py --build                # write golden/d64_int_taps.json
    python tools/d64_int_ref.py --check                # recompute and compare
    python tools/d64_int_ref.py --sweep                # compare the design knobs
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import sys
from collections import OrderedDict

import numpy as np
import torch

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from d64_net import D64Net  # noqa: E402
from d64_quant import (GOLDEN_DIR, build_layers, calibrate, derive,  # noqa: E402
                       load_images, requant_params)


# ---------------------------------------------------------------------------
# integer primitives -- these are the contract
# ---------------------------------------------------------------------------

def requant(acc: np.ndarray, M: np.ndarray, shift: np.ndarray,
            attrs: dict) -> np.ndarray:
    """``clamp(((acc * M + half) >> shift) + zero)``, arithmetic shift.

    ``half = 1 << (shift - 1)``, or 0 when shift is 0.  The shift is applied to
    a signed value, so it floors; combined with ``half`` that is round-half-up.
    """
    half = np.where(shift > 0, np.left_shift(np.int64(1), np.maximum(shift - 1, 0)), 0)
    y = (acc * M + half) >> shift
    y = y + attrs["zero"]
    return np.clip(y, attrs["qmin"], attrs["qmax"])


def conv_int(a_q: np.ndarray, L: dict, P: dict, f_in: dict, f_out: dict,
             want_acc: bool = False):
    """Reflection-padded correlation over int operands.  ``a_q`` is HWC int."""
    k, s, pad = L["k"], L["stride"], L["k"] // 2
    a_op = a_q.astype(np.int64) - f_in["zero"]
    ap = np.pad(a_op, ((pad, pad), (pad, pad), (0, 0)), mode="reflect")
    h = (ap.shape[0] - k) // s + 1
    w = (ap.shape[1] - k) // s + 1
    acc = np.zeros((h, w, L["cout"]), dtype=np.int64)
    W = P["w_q"]                                     # [cout, cin, k, k]
    for kh in range(k):
        for kw in range(k):
            patch = ap[kh:kh + s * (h - 1) + 1:s, kw:kw + s * (w - 1) + 1:s, :]
            acc += patch @ W[:, :, kh, kw].T
    acc += P["b_q"]
    y = requant(acc, P["M"], P["shift"], f_out)
    if L["relu"]:
        y = np.maximum(y, f_out["zero"])
    return (y.astype(np.int32), acc) if want_acc else y.astype(np.int32)


def tconv_int(a_q: np.ndarray, L: dict, P: dict, f_in: dict, f_out: dict,
              want_acc: bool = False):
    """Scatter form: ``oh = ih*stride - padding + kh``, no spatial flip.

    ``W`` is ``[cin, cout, k, k]`` -- the transposed layout, not the normal one.
    """
    k, s, pad = L["k"], L["stride"], L["k"] // 2
    a_op = a_q.astype(np.int64) - f_in["zero"]
    h, w = a_op.shape[0], a_op.shape[1]
    ho, wo = h * s, w * s
    acc = np.zeros((ho, wo, L["cout"]), dtype=np.int64)
    W = P["w_q"]                                     # [cin, cout, k, k]
    for kh in range(k):
        ohs = np.arange(h) * s - pad + kh
        mh = (ohs >= 0) & (ohs < ho)
        if not mh.any():
            continue
        for kw in range(k):
            ows = np.arange(w) * s - pad + kw
            mw = (ows >= 0) & (ows < wo)
            if not mw.any():
                continue
            patch = a_op[mh][:, mw]                  # [nh, nw, cin]
            acc[np.ix_(ohs[mh], ows[mw])] += patch @ W[:, :, kh, kw]
    acc += P["b_q"]
    y = requant(acc, P["M"], P["shift"], f_out)
    if L["relu"]:
        y = np.maximum(y, f_out["zero"])
    return (y.astype(np.int32), acc) if want_acc else y.astype(np.int32)


def add_int(q_skip: np.ndarray, q_branch: np.ndarray, f_trunk: dict) -> np.ndarray:
    """``q_out = q_skip + q_branch - zero``, then saturate.  No requant.

    Two's complement makes this exact: ``(a - z) + (b - z) = (a + b - z) - z``,
    so the sum of two values in one format is that format's sum minus one zero
    point.  That single constant is the entire cost of the residual add.
    """
    q = q_skip.astype(np.int64) + q_branch.astype(np.int64) - f_trunk["zero"]
    return np.clip(q, f_trunk["qmin"], f_trunk["qmax"]).astype(np.int32)


def requant_between(q: np.ndarray, f_from: dict, f_to: dict, m_bits: int) -> np.ndarray:
    """Move a quantized tensor from one format to another.  One multiplier."""
    m, sh = requant_params(f_from["scale"], f_to["scale"], m_bits)
    M = np.asarray([m], dtype=np.int64)
    S = np.asarray([sh], dtype=np.int64)
    x = q.astype(np.int64) - f_from["zero"]
    return requant(x, M, S, f_to).astype(np.int32)


# ---------------------------------------------------------------------------
# the model
# ---------------------------------------------------------------------------

class IntModel:
    """Executes the integer contract, tap by tap, in the float graph's order."""

    def __init__(self, doc: dict, arrays: dict, net: D64Net):
        self.doc = doc
        self.arr = arrays
        self.net = net
        self.fmt = doc["formats"]
        self.tap_fmt = doc["tap_formats"]
        self.m_bits = doc["m_bits"]
        self.residual = doc["residual"]
        self.layer = {r["name"]: r for r in doc["layers"]}

    def _call(self, name, q, want_acc=False):
        L = self.layer[name]
        f_in = self.fmt[L["in_tensor"]]
        f_out = self.fmt[L["out_tensor"]]
        fn = conv_int if L["kind"] == "conv" else tconv_int
        return fn(q, L, self.arr[name], f_in, f_out, want_acc)

    def run(self, x_uint8: np.ndarray, with_acc: bool = True) -> dict:
        """``x_uint8`` is HWC uint8.  Returns tap name -> HWC int32 (and accs)."""
        t: dict[str, np.ndarray] = {}
        accs: dict[str, np.ndarray] = {}
        f_trunk = self.fmt["trunk"]

        def step(name, q):
            y, acc = self._call(name, q, want_acc=True)
            t[name] = y
            accs[name] = acc
            return y

        q = x_uint8.astype(np.int32)
        t["in"] = q
        q = step("head", q)
        q = step("downs.0", q)
        q = step("downs.1", q)

        for i in range(5):
            skip = q
            q = step(f"trunk.{i}.conv1", q)
            q = step(f"trunk.{i}.conv2", q)
            c = step(f"trunk.{i}.conv3", q)
            if self.residual == "dual":
                # conv3 has its own format; bring it onto the trunk stream.
                c = requant_between(c, self.fmt[f"trunk.{i}.conv3"], f_trunk,
                                    self.m_bits)
            q = add_int(skip, c, f_trunk)
            t[f"trunk.{i}.out"] = q

        q = step("ups.0", q)
        q = step("ups.1", q)
        q = step("tail", q)
        return {"taps": t, "acc": accs}


# ---------------------------------------------------------------------------
# comparison and quality
# ---------------------------------------------------------------------------

def dequantize(q: np.ndarray, f: dict) -> np.ndarray:
    return (q.astype(np.float64) - f["zero"]) * f["scale"]


def float_taps_hwc(net: D64Net, x_uint8: np.ndarray) -> dict:
    x = torch.from_numpy(x_uint8).permute(2, 0, 1).contiguous().float().unsqueeze(0)
    with torch.no_grad():
        taps = net.forward_taps(x)
    return {k: v[0].permute(1, 2, 0).numpy().astype(np.float64) for k, v in taps.items()}


def compare(model: IntModel, int_taps: dict, ref: dict) -> "list[dict]":
    """Per-tap difference, dequantized, against the float graph.

    ``clipped`` is measured on the *float* values against the format's
    representable range, not on the quantized ones against qmin/qmax.  Counting
    ``q == qmin`` would report every zero of every ReLU tensor as saturation,
    which is most of them: a ReLU output sits at exactly 0 for a quarter of the
    head and half of trunk.0.conv1, and none of that is clipping.
    """
    rows = []
    for name, r in ref.items():
        if name not in int_taps:
            continue
        f = model.fmt[model.tap_fmt[name]]
        d = dequantize(int_taps[name], f) - r
        span = float(r.max() - r.min())
        lo = (f["qmin"] - f["zero"]) * f["scale"]
        hi = (f["qmax"] - f["zero"]) * f["scale"]
        rows.append({
            "tap": name,
            "format": model.tap_fmt[name],
            "span": span,
            "max_abs": float(np.abs(d).max()),
            "mean_abs": float(np.abs(d).mean()),
            "levels": float(np.abs(d).max()) / (span / 255.0) if span > 0 else 0.0,
            "clipped": float(np.mean((r < lo) | (r > hi))),
        })
    return rows


def report_compare(rows: "list[dict]") -> None:
    print(f"  {'tap':18s} {'span':>9s} {'max|err|':>10s} {'mean|err|':>10s} "
          f"{'levels':>8s} {'clipped':>9s}")
    for r in rows:
        print(f"  {r['tap']:18s} {r['span']:9.2f} {r['max_abs']:10.4f} "
              f"{r['mean_abs']:10.4f} {r['levels']:8.3f} {r['clipped']:8.3%}")


def quality(int_out: np.ndarray, ref_out: np.ndarray) -> dict:
    """The delivered image compared with the float one.

    Both sides go through the same delivery path the training evaluator uses --
    clamp to 0..255 -- because that is what ships.  The int8 side is already a
    set of integers in the 0..255 format, so clamping is a no-op for it and the
    comparison is honest about which side carries the rounding.
    """
    a = np.clip(int_out.astype(np.float64), 0, 255)
    b = np.clip(ref_out, 0, 255)
    d = a - b
    mse = float((d ** 2).mean())
    return {
        "max_abs": float(np.abs(d).max()),
        "mean_abs": float(np.abs(d).mean()),
        "rmse": float(math.sqrt(mse)),
        "psnr": float(20 * math.log10(255.0) - 10 * math.log10(mse)) if mse > 0 else None,
        "exact_fraction": float(np.mean(np.round(a) == np.round(b))),
    }


# ---------------------------------------------------------------------------
# golden
# ---------------------------------------------------------------------------

def digest(a: np.ndarray) -> str:
    return hashlib.sha256(np.ascontiguousarray(a, dtype="<i4").tobytes()).hexdigest()


def summarise(q: np.ndarray) -> dict:
    flat = q.reshape(-1)
    n = flat.size
    idx = np.linspace(0, n - 1, 8).astype(np.int64) if n >= 8 else np.arange(n)
    return {
        "shape": list(q.shape),
        "sha256": digest(q),
        "min": int(q.min()), "max": int(q.max()),
        "mean": float(q.mean()),
        "distinct": int(np.unique(q).size),
        "samples_idx": [int(i) for i in idx],
        "samples_val": [int(flat[i]) for i in idx],
    }


# ---------------------------------------------------------------------------

def build(checkpoint: str, data_root: str, split, limit: int, args) -> tuple:
    net = D64Net.from_checkpoint(checkpoint)
    names = read_split(split) if split else sorted(
        f for f in os.listdir(data_root) if f.lower().endswith((".jpg", ".png")))
    if limit:
        names = names[:limit]
    images = load_images([os.path.join(data_root, n) for n in names])
    ranges = calibrate(net, images, args.percentile)
    doc, arrays = derive(net, ranges, args.trunk_format, args.m_bits,
                         args.weight_bits, args.residual, args.trunk_bits)
    doc["calibration"].update({"images": len(images), "percentile": args.percentile,
                               "source": data_root})
    return net, doc, arrays


def read_split(path: str):
    with open(path, encoding="utf-8") as fh:
        return [ln.strip() for ln in fh if ln.strip() and not ln.startswith("#")]


def load_golden_input() -> np.ndarray:
    cands = sorted(f for f in os.listdir(GOLDEN_DIR)
                   if f.startswith("d64_input_") and f.endswith(".npy"))
    if not cands:
        sys.exit("no pinned input in golden/; see tools/d64_golden.py --build-input")
    return np.load(os.path.join(GOLDEN_DIR, cands[0]))


def held_out_images(data_root: str, split, n: int):
    """Images from ``data_root`` that are *not* in the calibration split.

    Quality has to be measured on content the calibration never saw, or the
    scales are being scored on the data that chose them.  The pinned golden
    input is itself one of the validation images, so it is a determinism
    fixture, not a quality measurement.
    """
    used = set(read_split(split)) if split and os.path.isfile(split) else set()
    pool = sorted(f for f in os.listdir(data_root)
                  if f.lower().endswith((".jpg", ".png")) and f not in used)
    if not pool:
        sys.exit(f"no images in {data_root} outside the calibration split")
    return [os.path.join(data_root, f) for f in pool[:n]]


def eval_quality(args, net, model: IntModel, paths) -> dict:
    """int8 and float against the teacher, on images the calibration never saw.

    Comparing int8 with the float student says how much the quantiser moved;
    comparing both with the *teacher* says whether that movement matters.  It is
    the second question the product cares about, and the answer here is that the
    quantiser's error is an order of magnitude below the distillation error it
    sits on top of, so it does not survive into the delivered metric at all.

    Both sides go through the evaluator's own delivery path -- clamp to 0..255 --
    so this is the image that ships being compared, not an intermediate.
    """
    root = os.path.abspath(args.teacher_root)
    if not os.path.isdir(os.path.join(root, "tinystyle")):
        sys.exit(f"--teacher-root {root} is not the training tree")
    if root not in sys.path:
        sys.path.insert(0, root)
    import yaml
    from tinystyle.models.teacher import load_teacher
    cfg = yaml.safe_load(open(os.path.join(root, "configs", "base.yaml"), encoding="utf-8"))
    ckpt = os.path.join(root, cfg["teacher"]["checkpoint"])
    if not os.path.isfile(ckpt):
        sys.exit(f"no teacher checkpoint at {ckpt}")
    teacher = load_teacher(ckpt, device=torch.device("cpu")).eval()

    rows = []
    for p in paths:
        e = np.array(load_images([p])[0].permute(1, 2, 0).numpy(), dtype=np.uint8)
        x = torch.from_numpy(e).permute(2, 0, 1).contiguous().float().unsqueeze(0)
        with torch.no_grad():
            t = teacher(x).clamp(0, 255)
            f = net(x).clamp(0, 255)
        out = model.run(e, with_acc=False)
        i = torch.from_numpy(dequantize(out["taps"]["tail"], model.fmt["out"])) \
            .permute(2, 0, 1).unsqueeze(0).float().clamp(0, 255)
        rows.append({
            "image": os.path.basename(p),
            "float_vs_teacher": float((f - t).abs().mean()),
            "int8_vs_teacher": float((i - t).abs().mean()),
            "int8_vs_float": float((i - f).abs().mean()),
            "psnr_int8_vs_float": _psnr(i, f),
        })

    agg = {k: float(np.mean([r[k] for r in rows]))
           for k in ("float_vs_teacher", "int8_vs_teacher", "int8_vs_float")}
    agg["psnr_int8_vs_float"] = float(np.mean([r["psnr_int8_vs_float"] for r in rows]))
    delta = agg["int8_vs_teacher"] - agg["float_vs_teacher"]
    agg["int8_cost_l1"] = delta
    agg["int8_cost_fraction"] = delta / agg["float_vs_teacher"]
    return {"n_images": len(rows), "aggregate": agg, "per_image": rows}


def _psnr(a: torch.Tensor, b: torch.Tensor) -> float:
    mse = float(((a - b) ** 2).mean())
    return float(20 * math.log10(255.0) - 10 * math.log10(mse)) if mse > 0 else float("inf")


def report_quality(q: dict) -> None:
    a = q["aggregate"]
    print(f"held-out images: {q['n_images']}")
    print()
    print(f"  float student vs teacher   L1 {a['float_vs_teacher']:.4f}")
    print(f"  int8  student vs teacher   L1 {a['int8_vs_teacher']:.4f}")
    print(f"  int8  vs float             L1 {a['int8_vs_float']:.4f}  "
          f"PSNR {a['psnr_int8_vs_float']:.3f} dB")
    print()
    print(f"  INT8 costs {a['int8_cost_l1']:+.4f} L1 against the teacher "
          f"({a['int8_cost_fraction']:+.2%} of the distillation error)")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--checkpoint", default=os.path.join(
        os.path.dirname(GOLDEN_DIR), "models", "weights", "vangogh", "model_fp32.pth"))
    ap.add_argument("--data-root", default=os.path.join(
        os.path.dirname(GOLDEN_DIR), "training", "data", "coco_val2017"))
    ap.add_argument("--split", default=os.path.join(
        os.path.dirname(GOLDEN_DIR), "training", "data", "splits",
        "val_fixed.txt"))
    ap.add_argument("--limit", type=int, default=0)
    ap.add_argument("--percentile", type=float, default=0.01)
    ap.add_argument("--trunk-format", default="uint8_asym")
    ap.add_argument("--residual", default="shared", choices=("shared", "dual"))
    ap.add_argument("--m-bits", type=int, default=16,
                    help="requant multiplier width.  16 is the frozen D2.1 " 
                         "contract: held-out quality is indistinguishable from " 
                         "31, and 25x16 costs two DSP48 where 25x31 costs four")
    ap.add_argument("--trunk-bits", type=int, default=10)
    ap.add_argument("--eval-n", type=int, default=8,
                    help="held-out images for quality; never the calibration set")
    ap.add_argument("--weight-bits", type=int, default=8)
    ap.add_argument("--input", default=None)
    ap.add_argument("--quality", action="store_true")
    ap.add_argument("--build", action="store_true")
    ap.add_argument("--check", action="store_true")
    ap.add_argument("--sweep", action="store_true",
                    help="compare the calibration and format knobs")
    ap.add_argument("--eval-quality", action="store_true",
                    help="int8 and float against the teacher, on held-out images")
    ap.add_argument("--teacher-root", default=os.path.join(
        os.path.dirname(GOLDEN_DIR), "training"))
    ap.add_argument("--quality-out", default=os.path.join(
        GOLDEN_DIR, "d64_int8_quality.json"))
    ap.add_argument("--golden", default=os.path.join(GOLDEN_DIR, "d64_int_taps.json"))
    args = ap.parse_args()

    if args.sweep:
        return sweep(args)

    if args.eval_quality:
        net = D64Net.from_checkpoint(args.checkpoint)
        names = read_split(args.split) if os.path.isfile(args.split) else []
        images = load_images([os.path.join(args.data_root, n) for n in names])
        ranges = calibrate(net, images, args.percentile)
        doc, arrays = derive(net, ranges, args.trunk_format, args.m_bits,
                             args.weight_bits, args.residual, args.trunk_bits)
        model = IntModel(doc, arrays, net)
        paths = held_out_images(args.data_root, args.split, args.eval_n)
        q = eval_quality(args, net, model, paths)
        q["contract"] = {"trunk_bits": args.trunk_bits, "trunk_format": args.trunk_format,
                         "residual": args.residual, "percentile": args.percentile,
                         "calibration_images": len(images)}
        report_quality(q)
        with open(args.quality_out, "w", encoding="utf-8", newline="\n") as fh:
            json.dump(q, fh, indent=1)
            fh.write("\n")
        print(f"\nwrote {args.quality_out}")
        return

    net, doc, arrays = build(args.checkpoint, args.data_root, args.split,
                             args.limit, args)
    model = IntModel(doc, arrays, net)
    x = np.load(args.input) if args.input else load_golden_input()
    out = model.run(x)
    ref = float_taps_hwc(net, x)

    if args.quality or args.build or args.check:
        print(f"input {x.shape} uint8, calibration {doc['calibration']['images']} images"
              + (f", percentile {args.percentile}" if args.percentile else ", max"))
        print(f"trunk format {doc['trunk_format']}, residual {doc['residual']}")
        print()
        rows = compare(model, out["taps"], ref)
        report_compare(rows)
        print()
        q = quality(dequantize(out["taps"]["tail"], model.fmt["out"]), ref["tail"])
        print("delivered image, int8 vs float")
        for k, v in q.items():
            print(f"  {k:16s} {v if v is None else round(v, 6)}")
        print()

    if args.build:
        payload = OrderedDict([
            ("note", "integer golden; every tap the RTL has a register at"),
            ("contract", {"trunk_format": doc["trunk_format"],
                          "residual": doc["residual"],
                          "weight_bits": doc["weight_bits"],
                          "m_bits": doc["m_bits"],
                          "requant": "(acc * M + (1 << (shift-1))) >> shift, "
                                     "arithmetic shift, then +zero, then clamp",
                          "mac_operand": "q - zero",
                          "residual_add": "q_skip + q_conv3 - zero",
                          "calibration": doc["calibration"]}),
            ("formats", doc["formats"]),
            ("tap_formats", doc["tap_formats"]),
            ("taps", OrderedDict((k, summarise(v)) for k, v in out["taps"].items())),
            ("accumulators", OrderedDict(
                (k, {"min": int(v.min()), "max": int(v.max()),
                     "absmax": int(np.abs(v).max()),
                     "sha256": hashlib.sha256(
                         np.ascontiguousarray(v, dtype="<i8").tobytes()).hexdigest()})
                for k, v in out["acc"].items())),
        ])
        os.makedirs(os.path.dirname(args.golden), exist_ok=True)
        with open(args.golden, "w", encoding="utf-8", newline="\n") as fh:
            json.dump(payload, fh, indent=1)
            fh.write("\n")
        print(f"wrote {args.golden} ({len(payload['taps'])} activation taps, "
              f"{len(payload['accumulators'])} accumulators)")
    elif args.check:
        with open(args.golden, encoding="utf-8") as fh:
            golden = json.load(fh)
        bad = 0
        for k, info in golden["taps"].items():
            got = summarise(out["taps"][k])
            if got["sha256"] != info["sha256"]:
                bad += 1
                print(f"  FAIL  {k:18s} {info['sha256'][:12]} -> {got['sha256'][:12]}")
        for k, info in golden["accumulators"].items():
            a = out["acc"][k]
            d = max(abs(int(a.min()) - info["min"]), abs(int(a.max()) - info["max"]))
            if d:
                bad += 1
                print(f"  FAIL  acc {k:14s} range moved by {d}")
        print("PASS" if not bad else f"{bad} FAILED")
        sys.exit(1 if bad else 0)


def sweep(args) -> None:
    """Compare the knobs that the derivation leaves open, on held-out images.

    Every number here is measured on images the calibration never saw.  Scoring
    on the calibration set would reward whichever setting overfits it hardest,
    and the trunk percentile is exactly the kind of knob that does.
    """
    net = D64Net.from_checkpoint(args.checkpoint)
    names = read_split(args.split) if os.path.isfile(args.split) else sorted(
        f for f in os.listdir(args.data_root) if f.lower().endswith((".jpg", ".png")))
    if args.limit:
        names = names[:args.limit]
    images = load_images([os.path.join(args.data_root, n) for n in names])
    eval_paths = held_out_images(args.data_root, args.split, args.eval_n)
    eval_imgs = [np.array(t.permute(1, 2, 0).numpy(), dtype=np.uint8)
                 for t in load_images(eval_paths)]
    refs = [float_taps_hwc(net, e) for e in eval_imgs]

    variants = []
    for pct in (None, 0.1, 0.01):
        for tb in (8, 9, 10, 12):
            variants.append((pct, "uint8_asym", "shared", tb, "uint8_z0"))
    # The narrow operand width is a hardware decision, not a quality one: the
    # mapper only reaches for EFX_DSP24 -- two multipliers per physical block --
    # when both operands are 8-bit signed, and an unsigned 8-bit activation
    # becomes a 9-bit signed operand when it is zero-extended.  Moving the ReLU
    # layers' zero point to the midpoint keeps it at 8.  This is what that costs.
    for nf in ("operand_s8",):
        variants.append((0.01, "uint8_asym", "shared", 10, nf))

    print(f"calibrated on {len(images)} images; measured on {len(eval_imgs)} "
          f"held-out images")
    print()
    print(f"  {'calib':>7s} {'trunk':>6s} {'narrow':>11s} {'trunk step':>11s} "
          f"{'PSNR':>8s} {'mean|err|':>10s} {'max|err|':>9s} {'exact':>7s} "
          f"{'clip':>7s}")
    rows_out = []
    for pct, tf, res, tb, nf in variants:
        ranges = calibrate(net, images, pct)
        doc, arrays = derive(net, ranges, tf, args.m_bits, args.weight_bits, res, tb, nf)
        model = IntModel(doc, arrays, net)
        agg = {"psnr": [], "mean_abs": [], "max_abs": [], "exact": [], "clip": []}
        for e, ref in zip(eval_imgs, refs):
            out = model.run(e, with_acc=False)
            q = quality(dequantize(out["taps"]["tail"], model.fmt["out"]), ref["tail"])
            agg["psnr"].append(q["psnr"])
            agg["mean_abs"].append(q["mean_abs"])
            agg["max_abs"].append(q["max_abs"])
            agg["exact"].append(q["exact_fraction"])
            rows = compare(model, out["taps"], ref)
            agg["clip"].append(max(r["clipped"] for r in rows if r["tap"] != "tail"))
        mean = {k: float(np.mean(v)) for k, v in agg.items()}
        label = "max" if pct is None else f"p{pct}"
        print(f"  {label:>7s} {tb:6d} {nf:>11s} {doc['formats']['trunk']['scale']:11.4f} "
              f"{mean['psnr']:8.3f} {mean['mean_abs']:10.4f} {mean['max_abs']:9.3f} "
              f"{mean['exact']:6.2%} {mean['clip']:6.2%}")
        rows_out.append((mean["psnr"], label, tb, nf, mean))
    print()
    best = max(rows_out)
    print(f"  best mean PSNR: calib={best[1]} trunk_bits={best[2]} "
          f"narrow={best[3]}  {best[0]:.3f} dB, mean|err| {best[4]['mean_abs']:.4f}")


if __name__ == "__main__":
    main()
