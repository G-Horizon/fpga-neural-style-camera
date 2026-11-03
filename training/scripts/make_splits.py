# -*- coding: utf-8 -*-
"""Build the frozen data splits.

The validation set has to be *fixed*: every model in stages 1-5 is compared on
exactly the same pixels, so it is curated once, written to
``data/splits/val_fixed.txt`` and committed.  The brief asks for people,
architecture, nature, complex texture, large flat areas, high-frequency detail,
low light and high contrast, so candidates are chosen by measurement on those
axes and then confirmed by eye on a contact sheet.

    python scripts/make_splits.py --stage candidates     # measure + contact sheet
    python scripts/make_splits.py --stage finalize       # write the final split

``--stage finalize`` uses the ``VAL_FIXED`` list below, which is the human
selection made from the contact sheet.
"""

from __future__ import annotations

import argparse
import os
import sys

import numpy as np
import torch
import torch.nn.functional as F
from PIL import Image, ImageDraw, ImageFont

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)

from tinystyle.data import center_crop, list_images, load_rgb_tensor, write_split  # noqa: E402

DATA = os.path.join(ROOT, "data")
COCO = os.path.join(DATA, "coco_val2017")
SPLITS = os.path.join(DATA, "splits")
CANDIDATES_TXT = os.path.join(SPLITS, "candidates.txt")
CONTACT_SHEET = os.path.join(SPLITS, "candidates_contact_sheet.png")

#: Human selection off ``data/splits/candidates_contact_sheet.png`` (indices in
#: brackets are the contact-sheet cells).  Coverage was chosen against the
#: brief's list; changing this invalidates every number produced before it.
#:
#:   人物        000000289229, 000000343453, 000000579655, 000000232563,
#:               000000051610, 000000406417, 000000222118, 000000162092
#:   建筑        000000419882, 000000460967, 000000545958, 000000117719,
#:               000000173044
#:   自然景物    000000496409, 000000459809, 000000282046, 000000542423
#:   复杂纹理    000000068933, 000000256518, 000000565607
#:   大面积纯色  000000496409, 000000006614, 000000459809, 000000406417
#:   高频细节    000000565607, 000000542423, 000000068933
#:   低光        000000477805, 000000579655, 000000117719, 000000162092
#:   高对比度    000000122745, 000000419882, 000000433774
VAL_FIXED = [
    "000000477805.jpg",   # [ 0] low-light silhouette, person
    "000000122745.jpg",   # [ 2] STOP sign, saturated red, hard edges
    "000000289229.jpg",   # [ 7] person against a clean white ground
    "000000496409.jpg",   # [ 9] snow field, large flat low-contrast area
    "000000343453.jpg",   # [12] person on snow, flat bright background
    "000000419882.jpg",   # [20] red architecture, high contrast
    "000000565607.jpg",   # [23] flower field, dense high-frequency detail
    "000000579655.jpg",   # [24] portrait, low key
    "000000006614.jpg",   # [26] grayscale produce, smooth flat subject
    "000000433774.jpg",   # [27] grayscale street, high contrast, texture
    "000000460967.jpg",   # [36] bus and building, urban architecture
    "000000068933.jpg",   # [38] zebras, repeating high-frequency pattern
    "000000459809.jpg",   # [43] sea haze, almost uniform region
    "000000232563.jpg",   # [44] people at a table, busy scene
    "000000282046.jpg",   # [51] wetland, natural low-contrast wash
    "000000545958.jpg",   # [59] blue doorway, saturated flat colour
    "000000051610.jpg",   # [64] person indoors, soft light
    "000000406417.jpg",   # [67] three people on a plain white background
    "000000117719.jpg",   # [70] night-time colourful architecture
    "000000222118.jpg",   # [75] person on the phone, structural background
    "000000256518.jpg",   # [76] dining table, dense object texture
    "000000542423.jpg",   # [79] dry grass field, fine high-frequency texture
    "000000173044.jpg",   # [82] clock tower, architectural detail
    "000000162092.jpg",   # [84] candle-lit dinner, very low light
]


# ---------------------------------------------------------------------------
# measurement
# ---------------------------------------------------------------------------

def measure(path: str) -> dict:
    x = load_rgb_tensor(path)
    x = F.interpolate(x.unsqueeze(0), size=(160, 160), mode="bilinear",
                      align_corners=False)[0]
    r, g, b = x[0], x[1], x[2]
    luma = 0.299 * r + 0.587 * g + 0.114 * b
    mx = x.max(dim=0).values
    mn = x.min(dim=0).values
    sat = (mx - mn) / (mx + 1e-6)
    # Hasler-Suesstrunk colourfulness
    rg = r - g
    yb = 0.5 * (r + g) - b
    colorfulness = float(torch.sqrt(rg.std() ** 2 + yb.std() ** 2)
                         + 0.3 * torch.sqrt(rg.mean() ** 2 + yb.mean() ** 2))
    gx = luma[:, 1:] - luma[:, :-1]          # (H, W-1)
    gy = luma[1:, :] - luma[:-1, :]          # (H-1, W)
    edge = float(torch.sqrt(gx[:-1, :] ** 2 + gy[:, :-1] ** 2).mean())
    # high-frequency energy fraction: detail bands vs total, one Haar level
    ll = F.avg_pool2d(luma.reshape(1, 1, *luma.shape), 2)
    hf = luma.reshape(1, 1, *luma.shape) - F.interpolate(ll, scale_factor=2, mode="nearest")
    hf_ratio = float(hf.pow(2).mean() / (luma.pow(2).mean() + 1e-6))
    return {
        "luma": float(luma.mean()),
        "contrast": float(luma.std()),
        "colorfulness": colorfulness,
        "saturation": float(sat.mean()),
        "edge": edge,
        "hf": hf_ratio,
        "dark": float((luma < 40).float().mean()),
        "bright": float((luma > 215).float().mean()),
    }


AXES = ["luma", "contrast", "colorfulness", "saturation", "edge", "hf", "dark", "bright"]


def pick_candidates(feats: dict, per_axis: int = 8, seed: int = 0):
    names = sorted(feats)
    scores = {a: np.array([feats[n][a] for n in names]) for a in AXES}
    chosen = []

    def take(order, count, tag):
        added = 0
        for i in order:
            n = names[int(i)]
            if n not in chosen:
                chosen.append(n)
                added += 1
                if added >= count:
                    break
        return added

    # extremes on every axis, both directions
    spans = {}
    for a in AXES:
        v = scores[a]
        lo, hi = float(np.percentile(v, 2)), float(np.percentile(v, 98))
        spans[a] = max(hi - lo, 1e-9)
    for a in AXES:
        v = scores[a]
        take(np.argsort(v), per_axis // 2, a + "_low")
        take(np.argsort(-v), per_axis // 2, a + "_high")

    # a spread of "typical" images so the set is not all outliers
    typical = np.argsort(-np.abs(
        sum(((scores[a] - np.median(scores[a])) / spans[a]) ** 2 for a in AXES)))
    take(typical, 12, "typical")

    # random filler for breadth
    rng = np.random.default_rng(seed)
    take(rng.permutation(len(names)), 12, "random")
    return chosen


def contact_sheet(names, path, cols: int = 8, cell: int = 190):
    rows = (len(names) + cols - 1) // cols
    pad, bar = 6, 22
    W = cols * (cell + pad) + pad
    H = rows * (cell + pad + bar) + pad
    sheet = Image.new("RGB", (W, H), (24, 24, 28))
    draw = ImageDraw.Draw(sheet)
    try:
        font = ImageFont.load_default(size=15)
    except TypeError:
        font = ImageFont.load_default()
    for i, rel in enumerate(names):
        r, c = divmod(i, cols)
        x0 = pad + c * (cell + pad)
        y0 = pad + r * (cell + pad + bar)
        with Image.open(os.path.join(COCO, rel)) as im:
            im = im.convert("RGB")
            w, h = im.size
            s = min(w, h)
            im = im.crop(((w - s) // 2, (h - s) // 2, (w - s) // 2 + s, (h - s) // 2 + s))
            im = im.resize((cell, cell), Image.LANCZOS)
        sheet.paste(im, (x0, y0))
        draw.text((x0 + 3, y0 + cell + 2), "%d  %s" % (i, rel[:12]), (235, 235, 235),
                  font=font)
    sheet.save(path)
    return path


# ---------------------------------------------------------------------------
# stages
# ---------------------------------------------------------------------------

def stage_candidates():
    names = list_images(COCO)
    print("measuring %d images ..." % len(names))
    feats = {}
    for i, rel in enumerate(names):
        feats[rel] = measure(os.path.join(COCO, rel))
        if (i + 1) % 1000 == 0:
            print("  %d/%d" % (i + 1, len(names)))
    chosen = pick_candidates(feats)
    os.makedirs(SPLITS, exist_ok=True)
    write_split(CANDIDATES_TXT, chosen)
    contact_sheet(chosen, CONTACT_SHEET)
    np.save(os.path.join(SPLITS, "candidate_features.npy"),
            np.array([[feats[n][a] for a in AXES] for n in chosen]))
    print("candidates : %d -> %s" % (len(chosen), CANDIDATES_TXT))
    print("contact    : %s" % CONTACT_SHEET)


def stage_finalize():
    names = set(list_images(COCO))
    missing = [n for n in VAL_FIXED if n not in names]
    if missing:
        raise SystemExit("validation images missing from %s: %s" % (COCO, missing))
    if len(set(VAL_FIXED)) != len(VAL_FIXED):
        raise SystemExit("VAL_FIXED contains duplicates")
    val = list(VAL_FIXED)
    train = sorted(names - set(val))
    write_split(os.path.join(SPLITS, "val_fixed.txt"), val)
    write_split(os.path.join(SPLITS, "train.txt"), train)
    print("val_fixed : %d -> data/splits/val_fixed.txt" % len(val))
    print("train     : %d -> data/splits/train.txt" % len(train))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--stage", choices=("candidates", "finalize"), required=True)
    args = ap.parse_args()
    if args.stage == "candidates":
        stage_candidates()
    else:
        stage_finalize()


if __name__ == "__main__":
    main()
