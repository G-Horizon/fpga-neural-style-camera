# -*- coding: utf-8 -*-
"""Config loading, seeding, run directories and small shared helpers."""

from __future__ import annotations

import hashlib
import json
import logging
import os
import random
import subprocess
import sys
import time
from collections import OrderedDict

import numpy as np
import torch
import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BASE_CONFIG = os.path.join(ROOT, "configs", "base.yaml")


# ---------------------------------------------------------------------------
# config
# ---------------------------------------------------------------------------

def deep_merge(base: dict, override: dict) -> dict:
    out = dict(base)
    for k, v in override.items():
        if k in out and isinstance(out[k], dict) and isinstance(v, dict):
            out[k] = deep_merge(out[k], v)
        else:
            out[k] = v
    return out


def load_config(path, base_path: str = None) -> dict:
    """Load an experiment YAML deep-merged on top of ``configs/base.yaml``."""
    with open(base_path or BASE_CONFIG, encoding="utf-8") as fh:
        cfg = yaml.safe_load(fh)
    with open(path, encoding="utf-8") as fh:
        exp = yaml.safe_load(fh) or {}
    cfg = deep_merge(cfg, exp)
    cfg["_config_path"] = os.path.abspath(path)
    cfg["_config_hash"] = config_hash(cfg)
    return cfg


def config_hash(cfg: dict) -> str:
    payload = json.dumps({k: v for k, v in cfg.items() if not k.startswith("_")},
                         sort_keys=True, default=str)
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()[:16]


def resolve(cfg: dict, key: str) -> str:
    """Resolve a config path relative to the project root."""
    value = cfg[key]
    return value if os.path.isabs(value) else os.path.join(ROOT, value)


# ---------------------------------------------------------------------------
# reproducibility
# ---------------------------------------------------------------------------

def set_seed(seed: int, deterministic: bool = False):
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    torch.cuda.manual_seed_all(seed)
    if deterministic:
        torch.backends.cudnn.deterministic = True
        torch.backends.cudnn.benchmark = False
    else:
        torch.backends.cudnn.benchmark = True


def seed_worker(worker_id):
    worker_seed = torch.initial_seed() % (2 ** 32)
    np.random.seed(worker_seed)
    random.seed(worker_seed)


def git_state() -> dict:
    def run(*args):
        try:
            out = subprocess.run(args, cwd=ROOT, capture_output=True, text=True, timeout=10)
            return out.stdout.strip() if out.returncode == 0 else None
        except Exception:
            return None

    commit = run("git", "rev-parse", "HEAD")
    status = run("git", "status", "--porcelain")
    return OrderedDict([
        ("commit", commit),
        ("dirty", bool(status)),
        ("describe", run("git", "describe", "--always", "--dirty")),
    ])


def environment_state() -> dict:
    return OrderedDict([
        ("python", sys.version.split()[0]),
        ("torch", torch.__version__),
        ("cuda_available", torch.cuda.is_available()),
        ("device_name", torch.cuda.get_device_name(0) if torch.cuda.is_available() else None),
        ("cudnn", torch.backends.cudnn.version()),
    ])


# ---------------------------------------------------------------------------
# run directory
# ---------------------------------------------------------------------------

def make_run_dir(cfg: dict, run_dir: str = None) -> str:
    """Create ``runs/<name>/<timestamp>`` (or use an explicit directory)."""
    if run_dir:
        path = run_dir if os.path.isabs(run_dir) else os.path.join(ROOT, run_dir)
    else:
        root = cfg["experiment"].get("output_root", "runs")
        root = root if os.path.isabs(root) else os.path.join(ROOT, root)
        stamp = time.strftime("%Y%m%d-%H%M%S")
        path = os.path.join(root, cfg["experiment"]["name"], stamp)
    os.makedirs(path, exist_ok=True)
    os.makedirs(os.path.join(path, "checkpoints"), exist_ok=True)
    os.makedirs(os.path.join(path, "samples"), exist_ok=True)
    return path


def write_json(path: str, payload, indent: int = 2):
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(payload, fh, indent=indent, ensure_ascii=False, default=str)
        fh.write("\n")


def read_json(path: str):
    with open(path, encoding="utf-8") as fh:
        return json.load(fh)


class JsonlLogger:
    """Append-only metric stream; one JSON object per event."""

    def __init__(self, path: str):
        self.path = path
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)

    def log(self, **fields):
        fields.setdefault("wall_time", time.time())
        with open(self.path, "a", encoding="utf-8") as fh:
            fh.write(json.dumps(fields, ensure_ascii=False, default=str) + "\n")


def setup_logger(logfile: str = None, level=logging.INFO) -> logging.Logger:
    logger = logging.getLogger("tinystyle")
    logger.setLevel(level)
    logger.handlers.clear()
    fmt = logging.Formatter("[%(asctime)s] %(levelname)s %(message)s", "%H:%M:%S")
    sh = logging.StreamHandler(sys.stdout)
    sh.setFormatter(fmt)
    logger.addHandler(sh)
    if logfile:
        fh = logging.FileHandler(logfile, encoding="utf-8")
        fh.setFormatter(fmt)
        logger.addHandler(fh)
    logger.propagate = False
    return logger


# ---------------------------------------------------------------------------
# checkpoints
# ---------------------------------------------------------------------------

def save_checkpoint(path: str, model, optimizer=None, scaler=None, adapters=None,
                    **extra):
    """Write a resumable checkpoint.

    ``adapters`` are the training-only 1x1 feature projections.  They are
    parameters of the *loss*, not the model, so they need saving explicitly --
    omitting them silently re-initialises them on resume and makes the feature
    term jump back to its random value (measured: 0.21 -> 0.87).
    """
    payload = OrderedDict([
        ("model", model.state_dict()),
        ("model_class", type(model).__name__),
    ])
    spec = getattr(model, "spec", None)
    if spec is not None:
        payload["spec"] = spec.to_dict()
    if adapters is not None:
        # Accept either the module or an already-serialised state dict, so a
        # conversion can pass the adapters straight through from a checkpoint it
        # loaded rather than having to reconstruct the loss to obtain them.
        payload["adapters"] = (adapters.state_dict()
                               if hasattr(adapters, "state_dict") else dict(adapters))
    if optimizer is not None:
        payload["optimizer"] = optimizer.state_dict()
    if scaler is not None:
        payload["scaler"] = scaler.state_dict()
    payload.update(extra)
    tmp = path + ".tmp"
    torch.save(payload, tmp)
    os.replace(tmp, path)


def load_checkpoint(path: str, map_location="cpu"):
    return torch.load(path, map_location=map_location, weights_only=False)


def count_parameters(model) -> int:
    return sum(p.numel() for p in model.parameters())


def human(n: float) -> str:
    for unit in ("", "K", "M", "G", "T"):
        if abs(n) < 1000.0:
            return "%.3f%s" % (n, unit)
        n /= 1000.0
    return "%.3fP" % n
