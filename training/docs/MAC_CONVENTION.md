# MAC accounting convention

Every number this project reports for every model comes from one function,
`tinystyle.mac.profile`. Nothing else is allowed to produce a MAC figure, because
the whole point of the exercise is comparing students against a teacher on the
same ruler.

## The rule

| operator | MAC |
|---|---|
| `nn.Conv2d` | `Cin * Cout * kh * kw * H_out * W_out` |
| `nn.ConvTranspose2d` | `Cin * Cout * kh * kw * H_in * W_in` |
| `nn.Linear` | `in_features * out_features` |

For a stride-`s` transposed convolution the second row is the count of **non-zero**
multiplications — the zero-insertion step is not counted. At `s = 2` that is 4x
cheaper than "nearest upsample, then convolve on the upsampled grid".

## Why `UpsampleConv` is credited as a transposed convolution

The 2017 `fast_neural_style` decoder stage (and this project's `UpsampleConv`)
literally runs `interpolate(nearest, x2)` followed by a 3x3 convolution, so at
face value it costs `Cin*Cout*9*(2H)*(2W)`. The canonical accounting instead
credits it with the equivalent transposed convolution, `Cin*Cout*9*H*W`.

This is the convention that reproduces the teacher's stated budget, and it is the
operator the FPGA implements. `profile(..., convention="fpga")` is the default;
`convention="native"` charges the graph as written and exists only as a
diagnostic.

**The convention is only honest if the model actually runs the cheap form.** A
student whose decoder uses `nearest + conv` would be credited 0.368 GMAC while
executing 0.764, which is why `StudentNet` builds `ConvTranspose2d` for every
decoder stage (`upsample="transpose"`, the default). The `resize` option exists
to demonstrate the gap and is asserted against in `tests/test_mac.py`.

## What is not counted

Normalisation, activation, residual adds, the upsampling itself and all memory
traffic contribute zero MACs. These are real FPGA costs, but they are not
convolution arithmetic, and letting them inflate or deflate the comparison would
make the work points incomparable.

## Verification

`tinystyle.mac` reproduces the teacher exactly, to the last MAC:

```
256x256 teacher = 8,266,973,184 MAC = 8.266973184 GMAC/frame
convolution weights = 1,674,432
```

and every work point in the brief:

| model | architecture | GMAC/frame | budget |
|---|---|---:|---:|
| `student_a` | `c5s1-16 d32 d64 R64x5 u32 u16 c5s1-3` | 1.969226 | ≈1.969 |
| `student_b_deep` | `c5s1-16 d32 d64 B64>32>64 x5 u32 u16 c5s1-3` | 0.731906 | ≈0.732 |
| `student_b_wide` | `c5s1-16 d32 d96 B96>48>96 x3 u32 u16 c5s1-3` | 0.902824 | ≈0.903 |
| `student_c_balanced` | `c3s1-16 d32 d48 B48>16>48 x3 u32 u16 c3s1-3` | 0.368050 | ≈0.368 |
| `student_c_fast` | `c3s1-16 d16 d64 B64>16>64 x5 u16 u16 c3s1-3` | 0.296747 | ≈0.297 |
| `student_d64` | `c3s1-8 d16 d32 B32>16>32 x5 u16 u8 c3s1-3` | 0.171966 | ≈0.172 |
| `student_d32` | `c3s1-8 d16 d32 d64 B64>32>64 x3 u32 u16 u8 c3s1-3` | 0.182452 | ≈0.182 |

`tests/test_mac.py` asserts all of these, plus an independent hand-arithmetic
re-derivation of the teacher so a bug in the module walker cannot hide behind a
shared constant.

```
python tests/test_mac.py
python -m tinystyle.cli mac --teacher
```

## Teacher, layer by layer

| layer | MAC/frame | share |
|---|---:|---:|
| `9x9, 3->32 @256x256` | 0.509608 G | 6.16% |
| `3x3, 32->64 @128x128` | 0.301990 G | 3.65% |
| `3x3, 64->128 @64x64` | 0.301990 G | 3.65% |
| 5 residual blocks, 10x `3x3, 128->128 @64x64` | 6.039798 G | **73.06%** |
| `u: 128->64` | 0.301990 G | 3.65% |
| `u: 64->32` | 0.301990 G | 3.65% |
| `9x9, 32->3 @256x256` | 0.509608 G | 6.16% |
| **total** | **8.266973 G** | 100% |

The two 9x9 endpoints alone are 1.019 GMAC — more than the entire `student_b_deep`
budget. That is why `head_kernel` and `tail_kernel` drop to 5 and then 3 in the
later stages.

## The teacher's *native* cost

Run as written, the same graph costs **10.078912512 GMAC/frame**: the two decoder
stages execute 1.207960 G each instead of 0.301990 G, a 3x premium on 7.3% of the
budget. The teacher is only ever used for inference and distillation, so this does
not affect any student's budget, but it is reported in
`runs/stage0_teacher/mac_report.json` under `native_convention` so the gap is
never invisible.
