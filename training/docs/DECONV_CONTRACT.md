# Decoder contract — what the upsample stage is, and who owes what

Status: **open decision** · raised 2026-09-27 · blocks Student-C/D

## The mismatch

Two different operators are in play for the same logical stage.

| | operator | where |
|---|---|---|
| teacher | `nearest x2 upsample -> reflection pad -> Conv2d(3x3, stride 1)` | `tinystyle/models/blocks.py::UpsampleConv`, and `fast_style_fpga/tools/calibrate_activations.py` |
| student | `ConvTranspose2d(3x3, stride 2, padding 1, output_padding 1)` | `tinystyle/models/blocks.py::UpConvTranspose` (`upsample: transpose`, the default for every student config) |

The teacher's decoder form and the FPGA project's reference are the *same*
operator. The students are not. `mac.py` and `blocks.py` previously described the
transposed form as "the operator the FPGA implements" — that was an overclaim and
has been corrected; this document replaces it.

## Why the students use the transposed form

The canonical MAC convention credits an upsample stage with

```
Cin * Cout * K * K * H_in * W_in
```

i.e. the cost of a transposed convolution. A `nearest + conv` stage really costs
four times that, because it runs a full `K x K` convolution on the upsampled
grid — one multiplication per kernel tap per *output* pixel, where the transposed
form needs one per tap per *input* pixel:

```
nearest + conv : 9 MAC per output pixel   -> 36 * Cin*Cout*H*W for a x2 stage
ConvTranspose2d: 9 MAC per input pixel    ->  9 * Cin*Cout*H*W
```

So a student with a resize decoder would be *credited* 0.368 GMAC (student_c_balanced)
and *execute* 0.764. The convention is only honest about a model that actually
runs the transposed form.

This is also why the teacher's own as-executed cost is 10.08 GMAC while its
credited cost is 8.27: the convention was defined by the 8.266973184 figure and
under-credits the teacher's decoder by 4x on 7.3% of the budget. Both numbers are
reported in `runs/stage0_teacher/mac_report.json`.

## The two forms are different functions, not two spellings of one

`scripts/decoder_equivalence.py` measures this rather than arguing it. Each
stride-2 decoder has four output phases; the input pixels each phase reads:

| phase | transposed conv | nearest + conv |
|---|---|---|
| (0,0) | 1 tap `[(0,0)]` | 4 taps `[(-1,-1),(-1,0),(0,-1),(0,0)]` |
| (0,1) | 2 taps | 4 taps |
| (1,0) | 2 taps | 4 taps |
| (1,1) | 4 taps | 4 taps |

9 incidences per input pixel versus 16. Solving for the resize weights that best
reproduce a random transposed convolution leaves a **relative residual of
0.62–0.71** — nearest+conv captures roughly half the energy of a transposed
convolution and cannot represent the rest. They are overlapping but distinct
9-parameter families.

**Consequence:** `upsample: transpose` puts the students in a different model
class from the teacher. The existing Student-A/B quality numbers describe that
class. They do not automatically transfer to a resize-decoder student, and a
resize-decoder student would need retraining to be compared fairly.

## The decision

**Route A — accept `ConvTranspose2d`.**
The training project keeps the current students and their budgets stand at
0.368 / 0.297 / 0.172 / 0.182 GMAC. The FPGA side commits to a stride-2
transposed-convolution datapath (or the equivalent sub-pixel/phase formulation).
Not free: it is a new engine or a new mode on the existing one, and it should be
sized by a small `deconv` feasibility DSE before it is assumed to divide the same
way as the dense 3x3 engines.

**Route B — keep `nearest + conv` to match the teacher and the current FPGA
reference.**
Then every student with more than one decoder stage must be re-derived against
honest MAC, and every model has to be retrained:

| model | credited today | as resize | ratio | verdict |
|---|---:|---:|---:|---|
| `student_a` | 1.969 | 2.422 | 1.23x | 2.4x over the 1.0 GMAC quality-first budget |
| `student_b_deep` | 0.732 | 1.185 | 1.62x | loses its ≤1 GMAC property |
| `student_b_wide` | 0.903 | 1.469 | 1.63x | loses its ≤1 GMAC property |
| `student_c_balanced` | 0.368 | 0.764 | 2.08x | 2.1x over the 30 FPS budget |
| `student_c_fast` | 0.297 | 0.523 | 1.76x | 1.8x over |
| `student_d64` | 0.172 | 0.285 | 1.66x | 1.7x over the 60 FPS target |
| `student_d32` | 0.182 | 0.352 | 1.93x | 1.9x over |

(Measured by building each spec with `upsample="resize"` and profiling it under
`convention="native"`, which charges the executed cost. The overall ratio is
below the per-stage 4x because only the decoder stages change.)

Route B therefore re-opens the entire architecture search, and the C/D work
points as specified do not fit at all — they would have to shrink by about 2x,
which puts them well below anything the brief contemplated.

**Recommendation: Route A**, for these reasons:

1. Route B invalidates every trained result and every budget in the brief.
2. A stride-2 transposed convolution is a standard, well-understood operator; the
   sub-pixel/phase view means it decomposes into four small dense convolutions
   over the input grid (supports 1/2/2/4), which maps onto a dense MAC array more
   naturally than a resize+conv would.
3. The teacher's own decoder being resize+conv is an artefact of the 2017
   reference implementation, not a property of the style. Nothing about Candy
   requires the decoder to replicate pixels.

**What Route A owes the hardware side:** a micro-DSE on the transposed-conv
engine — PE shape, the 4/2/2/1 phase supports, line-buffer cost for the
`output_padding=1` offset — before the 0.368 GMAC figure is treated as
achievable. That DSE is not in this repository.

## What has been changed here

- `tinystyle/mac.py` and `tinystyle/models/blocks.py` no longer claim the FPGA
  implements the transposed form; they state that it is what the canonical
  convention credits, and point here.
- `scripts/decoder_equivalence.py` reproduces the measurement above.
- A `upsample: resize` student can be built and profiled today
  (`tests/test_mac.py::test_transpose_and_resize_upsample_agree_under_fpga_convention`
  asserts both the credited equality and the 4x executed gap), so Route B can be
  costed without writing new model code.
