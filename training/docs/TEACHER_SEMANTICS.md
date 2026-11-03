# Teacher semantics — fixed-statistic vs dynamic InstanceNorm

Raised 2026-09-27 as P0-1. **Resolved: the fixed-statistic reading is the correct
one, and the FPGA project's `track_running_stats=False` assumption makes the
network produce visibly wrong output.**

## The conflict

Both repositories hold the *same* `candy.pth` blob —
`f7767f70ac45fe717f26f350ae3548c75c81d6bd`, verified with `git hash-object` — and
assume different functions for it:

| | how the checkpoint is used |
|---|---|
| `modeltrain` (`fast_style_transfer.py`, `tinystyle/models/teacher.py`) | `InstanceNorm2d(affine=True, track_running_stats=True)` + `.eval()` → normalisation by the stored `running_mean`/`running_var` → a fixed per-channel affine map |
| `fast_style_fpga/tools/calibrate_activations.py` | `track_running_stats=False`, with `running_mean`/`running_var` deleted from the state dict → normalisation by each image's own statistics |

These are different functions, and they are not close.

## Measurement

`scripts/teacher_semantics_gate.py`, on the 24-image frozen validation set at
256x256:

| | |
|---|---:|
| mean \|fixed − dynamic\| | **48.62** |
| max \|fixed − dynamic\| | 255.0 |
| PSNR(fixed, dynamic) | 12.07 dB |
| SSIM(fixed, dynamic) | 0.5846 |
| LPIPS(fixed, dynamic) | 0.4301 |
| output std (fixed / dynamic) | 78.5 / 68.6 |

Tiled-vs-whole-image inference, 768x768, tile 512, halo 128:

| teacher | max abs difference |
|---|---:|
| fixed-statistic | 3.0e+00 (float32 rounding; **exactly zero in float64**) |
| dynamic | **1.2e+03** |

The dynamic reading cannot be tiled at all, structurally: every output pixel
depends on the whole image. The reference pipeline's tiling — documented in
`fast_style_transfer.py`'s own module docstring as the reason it can process 12 MP
photos at all — depends on the fixed-statistic reading being true.

The running statistics are load-bearing, not vestigial: over 15 layers,
`|running_mean|` reaches 2325 and `running_var` ranges from 0.060 to 1.7e6.

## Which one produced the accepted reference

Two independent lines of evidence, both pointing the same way.

**1. The pipeline.** The accepted `picture/candy` images were produced by
`fast_style_transfer.py`, which constructs `track_running_stats=True` and calls
`model.eval()`. That is the fixed-statistic function by construction.

**2. The pixels.** Before the reference JPEGs were deleted (2026-09-27, during a
repository slim — see the caveat below), the fixed-statistic teacher was measured
against them: **MSE 157**. The fixed-vs-dynamic MSE is **4037**. The accepted
reference therefore sits about **26x closer** to the fixed-statistic output than
the two semantics sit to each other.

## Why dynamic is not merely different but degraded

The checkpoint was trained with `track_running_stats=True`. Its learned
`weight`/`beta` in every `InstanceNorm2d` are calibrated to operate on features
normalised by *running* statistics. Feeding instance-statistic-normalised
features through those same affine parameters is a distribution mismatch the
network was never trained for, and the visual result is not a stylistic variant:
`runs/teacher_semantics_gate/contact_sheet.png` shows washed-out colour,
posterised flats and blotchy skies.

## Consequences

**For this project.** Nothing changes. `tinystyle` already uses
`track_running_stats=True` everywhere for the teacher, and the whole folding /
tiling / export path is built on the fixed-statistic reading being correct.

**For `fast_style_fpga`.** This is the "large system-level simplification"
opportunity that motivated the question. If inference normalises by running
statistics, then per-image statistics are not needed at all, and with them go:

- sum / sum-of-squares accumulation over the full feature map
- `rsqrt` and the reciprocal-sqrt datapath
- the two-pass "collect statistics, then apply" structure
- the RAM and line-buffer cost of holding a feature map for a second pass
- any full-image dependency that prevents layer chaining

What remains is a per-channel scale and shift, which folds into the preceding
convolution's weights and bias — the `y = a*x + b` endpoint that `convert.py`
already targets and that `export.py` already verifies (fold drift 0.12 of one
uint8 level).

**Caveat, recorded honestly:** the strongest check — pixel comparison against the
accepted reference — can no longer be repeated, because those JPEGs and the source
photographs were deleted by mistake during the repository slim. The MSE-157
measurement above was taken before the deletion. If the original photographs can
be recovered from another source, regenerating the reference with
`fast_style_transfer.py` restores that check.
