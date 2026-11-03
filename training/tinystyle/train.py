# -*- coding: utf-8 -*-
"""Distillation training loop.

One entry point drives every stage.  What changes between stages is the student
spec and the normalisation, never the plumbing: same frozen validation set, same
evaluator, same metric definitions, so a number from stage 1 and a number from
stage 5 mean the same thing.

Each step pays one frozen-teacher forward, which yields both the distillation
target and the five feature taps the feature loss needs.  The teacher is 8.27
GMAC/frame against students from 1.97 down to 0.17, so for the smallest models
it dominates step time -- still only minutes over a whole run, and it buys
unrestricted content augmentation.

Reproducibility: the run directory holds the merged config, the git state, the
environment, the MAC report and a JSON-lines metric stream; ``--resume``
restores model, optimiser, scaler and step counter.
"""

from __future__ import annotations

import math
import os
import time
from collections import OrderedDict

import torch

from .convert import freeze_fixed_stats, norm_summary
from .data import ContentDataset, ValImageDataset, build_loader, read_split, val_collate
from .evaluator import evaluate_model
from .losses import DistillLoss, TapCollector
from .mac import parameter_breakdown, profile
from .models.student import StudentNet, spec_from_dict
from .models.teacher import load_teacher
from .utils import (JsonlLogger, ROOT, environment_state, git_state, human, load_config,
                    load_checkpoint, make_run_dir, save_checkpoint, set_seed,
                    setup_logger, write_json)


# ---------------------------------------------------------------------------
# setup helpers
# ---------------------------------------------------------------------------

def path_from(cfg, key, *sub):
    value = cfg[key]
    for part in sub:
        value = value[part]
    return value if os.path.isabs(value) else os.path.join(ROOT, value)


def build_student(cfg):
    return StudentNet(spec_from_dict(cfg["model"]))


def mac_report_for(model, name):
    rep = profile(model, (3, 256, 256), convention="fpga", name=name)
    payload = rep.to_dict()
    payload["parameter_breakdown"] = parameter_breakdown(model)
    return rep, payload


def lr_at(step: int, total: int, base_lr: float, warmup: int, schedule: str) -> float:
    if warmup and step < warmup:
        return base_lr * float(step + 1) / float(warmup)
    if schedule == "none":
        return base_lr
    progress = min(max(float(step - warmup) / float(max(total - warmup, 1)), 0.0), 1.0)
    if schedule == "cosine":
        return base_lr * (0.01 + 0.99 * 0.5 * (1.0 + math.cos(math.pi * progress)))
    if schedule == "step":
        return base_lr * (0.1 ** int(progress * 3))
    raise ValueError("unknown lr_schedule %r" % (schedule,))


class InfiniteLoader:
    """Cycle a loader forever; the loop is step-based, not epoch-based."""

    def __init__(self, loader):
        self.loader = loader
        self._it = None

    def __iter__(self):
        return self

    def __next__(self):
        if self._it is None:
            self._it = iter(self.loader)
        try:
            return next(self._it)
        except StopIteration:
            self._it = iter(self.loader)
            return next(self._it)


def build_loaders(cfg, seed):
    data_cfg = cfg["data"]
    train_ds = ContentDataset(
        path_from(cfg, "data", "root"),
        read_split(path_from(cfg, "data", "train_list")),
        crop=int(data_cfg.get("crop", 256)),
        scales=tuple(data_cfg.get("view_scales", [1.0])),
        augment_d4=bool(data_cfg.get("augment_d4", True)),
        seed=seed)
    train_loader = build_loader(train_ds, int(cfg["optim"]["batch_size"]), True,
                                int(data_cfg.get("num_workers", 4)), seed=seed,
                                drop_last=True)
    val_ds = ValImageDataset(path_from(cfg, "data", "root"),
                             read_split(path_from(cfg, "data", "val_list")),
                             crop=int(data_cfg.get("val_crop", 256)))
    val_loader = build_loader(val_ds, int(cfg["eval"]["batch_size"]), False,
                              min(2, int(data_cfg.get("num_workers", 4))), seed=seed,
                              collate_fn=val_collate)
    return train_loader, val_loader, train_ds, val_ds


# ---------------------------------------------------------------------------
# training
# ---------------------------------------------------------------------------

def train(cfg_path: str, run_dir: str = None, resume: str = None, device: str = None,
          max_steps: int = None, init_from: str = None, quiet: bool = False) -> str:
    cfg = load_config(cfg_path)
    device = torch.device(device or ("cuda" if torch.cuda.is_available() else "cpu"))
    seed = int(cfg["experiment"].get("seed", 0))
    set_seed(seed)

    run_dir = make_run_dir(cfg, run_dir)
    logger = setup_logger(os.path.join(run_dir, "train.log"))
    write_json(os.path.join(run_dir, "config.json"), cfg)
    write_json(os.path.join(run_dir, "git.json"), git_state())
    write_json(os.path.join(run_dir, "environment.json"), environment_state())
    logger.info("run dir  %s", run_dir)
    logger.info("device   %s", device)
    logger.info("config   %s  hash=%s", cfg_path, cfg["_config_hash"])
    logger.info("git      %s", git_state().get("describe"))

    teacher = load_teacher(path_from(cfg, "teacher", "checkpoint"), device=device)
    trep, tpayload = mac_report_for(teacher, "teacher")
    write_json(os.path.join(run_dir, "teacher_mac_report.json"), tpayload)
    logger.info("teacher  %.9f GMAC/frame  %d conv weights",
                trep.gmac_per_frame, trep.weight_params)

    student = build_student(cfg).to(device)
    init_ckpt = None
    if init_from:
        init_ckpt = load_checkpoint(init_from, map_location="cpu")
        student.load_state_dict(init_ckpt["model"])
        logger.info("initialised weights from %s", init_from)
    rep, payload = mac_report_for(student, cfg["model"]["name"])
    write_json(os.path.join(run_dir, "mac_report.json"), payload)
    logger.info("student  %s", student.spec.arch_string())
    logger.info("         %.6f GMAC/frame  %d params  norm=%s upsample=%s",
                rep.gmac_per_frame, payload["parameter_breakdown"]["total"],
                student.spec.norm, student.spec.upsample)
    logger.info("         norm %s", norm_summary(student))

    train_loader, val_loader, train_ds, val_ds = build_loaders(cfg, seed)
    logger.info("data     %d train images, %d val images, crop %d",
                len(train_ds), len(val_ds), cfg["data"].get("crop", 256))

    loss_fn = DistillLoss(cfg["loss"], student, teacher, device=device).to(device)
    if init_ckpt is not None and loss_fn.adapters is not None:
        # The adapters are parameters of the loss, not of the model, so a warm
        # start that only loads the student leaves them random -- which makes
        # "warm-started" true of the student's forward pass but false of the
        # objective actually being optimised.
        if "adapters" in init_ckpt:
            loss_fn.adapters.load_state_dict(init_ckpt["adapters"])
            logger.info("warm-started %d feature adapters from %s",
                        len(init_ckpt["adapters"]), init_from)
        else:
            logger.warning(
                "init checkpoint %s carries no feature adapters; they restart "
                "from random initialisation, so the warm start does not cover "
                "the feature loss term", init_from)
    params = list(student.parameters())
    n_adapters = 0
    if loss_fn.adapters is not None:
        params += list(loss_fn.adapters.parameters())
        n_adapters = len(loss_fn.adapters.adapters)
    n_trainable = sum(p.numel() for p in params)
    logger.info("optim    %s trainable params (%d adapters, %s of them)",
                human(n_trainable), n_adapters,
                human(sum(p.numel() for p in loss_fn.adapters.parameters()))
                if loss_fn.adapters is not None else "0")

    optim_cfg = cfg["optim"]
    optimizer = torch.optim.Adam(params, lr=float(optim_cfg["lr"]),
                                 weight_decay=float(optim_cfg.get("weight_decay", 0.0)),
                                 betas=(0.9, 0.999), eps=1e-8)
    use_amp = bool(optim_cfg.get("amp", False)) and device.type == "cuda"
    scaler = torch.amp.GradScaler("cuda", enabled=use_amp)

    epochs = int(optim_cfg["epochs"])
    spe = int(optim_cfg["steps_per_epoch"])
    total_steps = epochs * spe if max_steps is None else max_steps
    warmup = int(optim_cfg.get("warmup_steps", 0))
    grad_clip = float(optim_cfg.get("grad_clip", 0.0))
    log_every = int(optim_cfg.get("log_every", 50))

    start_epoch, global_step, best = 0, 0, float("inf")
    if resume:
        ckpt = load_checkpoint(resume, map_location=device)
        student.load_state_dict(ckpt["model"])
        if "optimizer" in ckpt:
            optimizer.load_state_dict(ckpt["optimizer"])
        if "scaler" in ckpt and use_amp:
            scaler.load_state_dict(ckpt["scaler"])
        if "adapters" in ckpt and loss_fn.adapters is not None:
            loss_fn.adapters.load_state_dict(ckpt["adapters"])
            logger.info("restored feature adapters from the checkpoint")
        elif loss_fn.adapters is not None:
            logger.warning("checkpoint has no feature adapters; they restart from "
                           "random initialisation")
        start_epoch = int(ckpt.get("epoch", 0))
        global_step = int(ckpt.get("global_step", 0))
        best = float(ckpt.get("best_val_l1", float("inf")))
        logger.info("resumed from %s at epoch %d step %d (best %.4f)",
                    resume, start_epoch, global_step, best)

    metrics = JsonlLogger(os.path.join(run_dir, "metrics.jsonl"))
    metrics.log(event="start", run_dir=run_dir, config_hash=cfg["_config_hash"],
                gmac=rep.gmac_per_frame, params=payload["parameter_breakdown"]["total"])

    loader = InfiniteLoader(train_loader)
    t0 = time.time()
    epoch = start_epoch
    while True:
        student.train()
        for _ in range(spe):
            lr = lr_at(global_step, total_steps, float(optim_cfg["lr"]), warmup,
                       optim_cfg.get("lr_schedule", "cosine"))
            for group in optimizer.param_groups:
                group["lr"] = lr

            content = next(loader).to(device, non_blocking=True)

            with torch.no_grad(), TapCollector(teacher) as tc:
                teacher_out = teacher(content)

            optimizer.zero_grad(set_to_none=True)
            with TapCollector(student) as sc:
                with torch.autocast("cuda", dtype=torch.float16, enabled=use_amp):
                    student_out = student(content)
            student_out = student_out.float()

            total, parts = loss_fn(student_out, teacher_out, sc.captured, tc.captured)
            scaler.scale(total).backward()
            if grad_clip:
                scaler.unscale_(optimizer)
                torch.nn.utils.clip_grad_norm_(params, grad_clip)
            scaler.step(optimizer)
            scaler.update()

            if global_step % log_every == 0:
                metrics.log(event="step", epoch=epoch, step=global_step, lr=lr,
                            total=float(total),
                            **{k: float(v[0]) for k, v in parts.items()},
                            **{"w_" + k: float(v[1]) for k, v in parts.items()})
                if global_step % (log_every * 8) == 0 and not quiet:
                    logger.info("e%d s%-6d lr=%.2e total=%.4f  %s", epoch, global_step,
                                lr, float(total), loss_fn.describe(parts))

            global_step += 1
            if global_step >= total_steps:
                break

        last_epoch = global_step >= total_steps
        every = max(int(cfg["eval"].get("every_epochs", 1)), 1)
        if (epoch + 1) % every == 0 or last_epoch:
            export = bool(cfg["eval"].get("export_images", True))
            result = evaluate_model(
                student, teacher, val_loader, loss_fn=loss_fn, device=device,
                out_dir=os.path.join(run_dir, "samples", "epoch_%03d" % (epoch + 1))
                if export else None,
                export_images=export,
                contact_sheet=bool(cfg["eval"].get("contact_sheet", True)),
                logger=None if quiet else logger)
            val_l1 = result.metrics["output_l1"]
            metrics.log(event="val", epoch=epoch, step=global_step,
                        **{k: float(v) for k, v in result.metrics.items()})
            write_json(os.path.join(run_dir, "evaluation_latest.json"), result.to_dict())
            save_checkpoint(os.path.join(run_dir, "checkpoints", "last.pth"), student,
                            optimizer, scaler, adapters=loss_fn.adapters,
                            epoch=epoch + 1, global_step=global_step,
                            best_val_l1=best, config_hash=cfg["_config_hash"])
            if val_l1 < best:
                best = val_l1
                save_checkpoint(os.path.join(run_dir, "checkpoints", "best.pth"), student,
                                optimizer, scaler, adapters=loss_fn.adapters,
                                epoch=epoch + 1, global_step=global_step,
                                best_val_l1=best, config_hash=cfg["_config_hash"])
                metrics.log(event="best", epoch=epoch, val_output_l1=val_l1)
                logger.info("e%d new best val output L1 = %.4f", epoch, val_l1)
        if last_epoch:
            break
        epoch += 1

    elapsed = time.time() - t0
    frozen = freeze_fixed_stats(student)
    if frozen:
        logger.info("froze %d fixed-stat norm layers", frozen)

    save_checkpoint(os.path.join(run_dir, "checkpoints", "final.pth"), student,
                    optimizer, scaler, adapters=loss_fn.adapters,
                    epoch=epoch + 1, global_step=global_step,
                    best_val_l1=best, config_hash=cfg["_config_hash"])

    final = evaluate_model(student, teacher, val_loader, loss_fn=loss_fn, device=device,
                           out_dir=os.path.join(run_dir, "samples", "final"),
                           export_images=True, contact_sheet=True)
    summary = OrderedDict([
        ("model_name", cfg["model"]["name"]),
        ("arch_string", student.spec.arch_string()),
        ("norm_type", student.spec.norm),
        ("upsample", student.spec.upsample),
        ("input_resolution", [256, 256]),
        ("gmac_per_frame", rep.gmac_per_frame),
        ("parameter_breakdown", payload["parameter_breakdown"]),
        ("weight_bytes_fp32", payload["parameter_breakdown"]["total"] * 4),
        ("weight_bytes_int8", payload["parameter_breakdown"]["total"]),
        ("teacher_gmac_per_frame", trep.gmac_per_frame),
        ("best_val_output_l1", best),
        ("final_metrics", final.metrics),
        ("epochs", epoch + 1),
        ("global_step", global_step),
        ("wall_seconds", elapsed),
        ("checkpoint", os.path.join(run_dir, "checkpoints", "best.pth")),
        ("config", cfg),
        ("config_hash", cfg["_config_hash"]),
        ("git", git_state()),
        ("environment", environment_state()),
        ("norm_summary", norm_summary(student)),
    ])
    write_json(os.path.join(run_dir, "experiment_summary.json"), summary)
    # evaluation.json has to stand on its own: a consumer reading just this file
    # needs the architecture, its MAC budget and its parameters, not a second
    # lookup.  (Without this, `compare` had to guess and printed NaN.)
    evaluation = final.to_dict()
    evaluation["arch_string"] = student.spec.arch_string()
    evaluation["norm_type"] = student.spec.norm
    evaluation["upsample_method"] = student.spec.upsample
    evaluation["gmac_per_frame"] = rep.gmac_per_frame
    evaluation["parameter_breakdown"] = payload["parameter_breakdown"]
    evaluation["mac_report"] = payload
    evaluation["checkpoint"] = os.path.join(run_dir, "checkpoints", "best.pth")
    evaluation["config_hash"] = cfg["_config_hash"]
    write_json(os.path.join(run_dir, "evaluation.json"), evaluation)
    metrics.log(event="done", best_val_output_l1=best, wall_seconds=elapsed)
    logger.info("done in %.1f min  best val output L1 = %.4f", elapsed / 60.0, best)
    logger.info("summary  %s", os.path.join(run_dir, "experiment_summary.json"))
    return run_dir
