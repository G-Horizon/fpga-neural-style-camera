# -*- coding: utf-8 -*-
"""Datasets.

Training runs the frozen teacher live on every step.  Feature distillation needs
its intermediate activations, which means a teacher forward is paid anyway, so
pre-computing its output would buy nothing and would restrict augmentation to
transforms the teacher commutes with.  Live content means arbitrary random
scale, crop and D4 flips on every step instead.

Validation uses the same live fp32 teacher, through the same evaluator, on a
frozen 24-image set.
"""

from __future__ import annotations

import os
from collections import OrderedDict

import numpy as np
import torch
import torchvision.transforms as T
from PIL import Image
from torch.utils.data import Dataset

IMG_EXT = (".jpg", ".jpeg", ".png", ".bmp", ".webp")


# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------

def list_images(root: str):
    names = []
    for dirpath, _dirnames, filenames in os.walk(root):
        for f in filenames:
            if f.lower().endswith(IMG_EXT):
                names.append(os.path.relpath(os.path.join(dirpath, f), root))
    return sorted(names)


def read_split(path: str):
    with open(path, encoding="utf-8") as fh:
        return [ln.strip() for ln in fh if ln.strip() and not ln.startswith("#")]


def write_split(path: str, entries):
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w", encoding="utf-8") as fh:
        for e in entries:
            fh.write(e + "\n")


def load_rgb_tensor(path: str) -> torch.Tensor:
    """Load an image as a float32 ``3xHxW`` tensor in 0..255."""
    with Image.open(path) as im:
        im = im.convert("RGB")
        arr = np.array(im, dtype=np.uint8, copy=True)
    return torch.from_numpy(arr).permute(2, 0, 1).contiguous().float()


def resize_shorter(x: torch.Tensor, size: int) -> torch.Tensor:
    _, h, w = x.shape
    if min(h, w) == size:
        return x
    scale = size / min(h, w)
    nh, nw = max(size, int(round(h * scale))), max(size, int(round(w * scale)))
    return T.functional.resize(x, [nh, nw], antialias=True)


def center_crop(x: torch.Tensor, size) -> torch.Tensor:
    _, h, w = x.shape
    if isinstance(size, int):
        size = (size, size)
    ch, cw = size
    top = max((h - ch) // 2, 0)
    left = max((w - cw) // 2, 0)
    return x[:, top:top + ch, left:left + cw]


#: The 8 dihedral transforms, as (flip_h, flip_v, n_rot90) applied in that order.
D4_GROUP = [(fh, fv, k) for k in range(4) for fv in (False, True) for fh in (False, True)]


def apply_d4(x: torch.Tensor, spec) -> torch.Tensor:
    flip_h, flip_v, k = spec
    if flip_h:
        x = torch.flip(x, dims=[-1])
    if flip_v:
        x = torch.flip(x, dims=[-2])
    if k:
        x = torch.rot90(x, k, dims=[-2, -1])
    return x


# ---------------------------------------------------------------------------
# training
# ---------------------------------------------------------------------------

class ContentDataset(Dataset):
    """Random 256x256 views of the training content images.

    Each item is one independent random scale + random crop, so "epoch" here
    means ``steps_per_epoch`` optimiser steps rather than a pass over the list.

    D4 flips are applied to the *content only*.  They are ordinary input
    augmentation, not a free reuse of a cached target: ``nn.Conv2d`` computes
    cross-correlation, so a layer commutes with flips only if its kernel is
    symmetric, and nothing here constrains that.  ``T(flip(x)) != flip(T(x))``
    by a wide margin (``tests/test_invariants.py`` measures it), which is why
    the teacher is evaluated live on every augmented view.
    """

    def __init__(self, root: str, entries, crop: int = 256, scales=(1.0, 1.15, 1.35),
                 augment_d4: bool = True, seed: int = 0):
        self.root = root
        self.entries = list(entries)
        self.crop = crop
        self.scales = tuple(scales) or (1.0,)
        self.augment_d4 = augment_d4
        self.seed = seed
        self._rng = None

    def __len__(self):
        return len(self.entries)

    def _rng_for_worker(self):
        info = torch.utils.data.get_worker_info()
        if info is None:
            if self._rng is None:
                self._rng = np.random.default_rng(self.seed)
            return self._rng
        if self._rng is None:
            self._rng = np.random.default_rng(self.seed + 1000 * info.id)
        return self._rng

    def __getitem__(self, index):
        rng = self._rng_for_worker()
        rel = self.entries[index]
        x = load_rgb_tensor(os.path.join(self.root, rel))
        scale = float(self.scales[int(rng.integers(len(self.scales)))])
        x = make_view(x, self.crop, scale, rng)
        if self.augment_d4:
            spec = D4_GROUP[int(rng.integers(len(D4_GROUP)))]
            x = apply_d4(x, spec).contiguous()
        return x


# ---------------------------------------------------------------------------
# validation
# ---------------------------------------------------------------------------

class ValImageDataset(Dataset):
    """Deterministic 256x256 views of the frozen validation content set.

    ``Resize(256)`` then ``CenterCrop(256)`` -- the same recipe for every model,
    so nothing about the comparison moves between runs.
    """

    def __init__(self, root: str, entries, crop: int = 256):
        self.root = root
        self.entries = list(entries)
        self.crop = crop

    def __len__(self):
        return len(self.entries)

    def __getitem__(self, index):
        rel = self.entries[index]
        x = load_rgb_tensor(os.path.join(self.root, rel))
        x = resize_shorter(x, self.crop)
        x = center_crop(x, self.crop)
        return x, rel


def val_collate(batch):
    tensors = torch.stack([b[0] for b in batch], dim=0)
    return tensors, [b[1] for b in batch]


def build_loader(dataset, batch_size, shuffle, num_workers, seed=0, collate_fn=None,
                 drop_last=False):
    generator = torch.Generator()
    generator.manual_seed(seed)
    kwargs = dict(
        batch_size=batch_size, shuffle=shuffle, num_workers=num_workers,
        pin_memory=torch.cuda.is_available(), drop_last=drop_last,
        generator=generator, collate_fn=collate_fn,
    )
    if num_workers > 0:
        from .utils import seed_worker
        kwargs["worker_init_fn"] = seed_worker
        kwargs["persistent_workers"] = True
        kwargs["prefetch_factor"] = 2
    return torch.utils.data.DataLoader(dataset, **kwargs)


# ---------------------------------------------------------------------------
# view generation
# ---------------------------------------------------------------------------

def make_view(x: torch.Tensor, crop: int, scale: float, rng: np.random.Generator):
    """Random scale + random crop of a loaded image, clipped to the image box."""
    _, h, w = x.shape
    target = int(round(crop * scale))
    if min(h, w) < target:
        scale_down = target / min(h, w)
        nh, nw = int(round(h * scale_down)), int(round(w * scale_down))
        x = T.functional.resize(x, [nh, nw], antialias=True)
        h, w = nh, nw
    top = int(rng.integers(0, max(h - crop, 0) + 1))
    left = int(rng.integers(0, max(w - crop, 0) + 1))
    return x[:, top:top + crop, left:left + crop].contiguous()


__all__ = [
    "ContentDataset", "ValImageDataset", "apply_d4", "build_loader",
    "center_crop", "list_images", "load_rgb_tensor", "make_view", "read_split",
    "resize_shorter", "val_collate", "write_split", "D4_GROUP",
]
