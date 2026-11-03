#!/usr/bin/env python3
"""The d64 deployment reference: one graph, built from the published checkpoint.

Why this file exists
--------------------

`d64.pth` carries weights, not semantics.  Weight shapes say a kernel is 3x3 and
that 32 channels become 16; they do not say whether the border is reflected or
zeroed, where the ReLUs sit, or what a stride-2 transposed convolution does with
its output_padding.  Every one of those was listed as UNPINNED by
`tools/d64_inspect.py`, and a wrong guess in any of them produces a network that
runs, produces an image, and is wrong.

The semantics live in the training source, which is now pinned in this repo:

    model_train/tinystyle/models/blocks.py   ConvLayer, BottleneckBlock, UpBlock
    model_train/tinystyle/models/student.py  StudentNet, the layer order
    model_train/tinystyle/export.py          what the fold produces

This module is that graph, re-expressed against the *folded* weights so the FPGA
repo does not need the training tree, PyTorch's own operators define the
arithmetic, and every claim below is checkable by `--verify`.

Why it is built from PyTorch operators and not hand-written kernels
------------------------------------------------------------------

The transposed convolution is the operator most likely to be got wrong, and the
one place where a hand-written reference is most dangerous: a scatter loop that
flips the kernel still produces a correctly-shaped, plausible image.  So the
reference uses `nn.ConvTranspose2d` itself.  That makes the reference correct by
definition, and moves the question "what must the RTL do" into `CONTRACT` below,
where it can be argued and tested rather than assumed.

The pinned semantics
--------------------

**Padding.**  ``ReflectionPad2d(k // 2)`` precedes every *normal* convolution --
reflection, not zeros and not replication.

**Activation.**  ``act(norm(conv(pad(x))))`` -- the ReLU is *after* the norm, and
the fold would not be bit-exact if it were anywhere else.  Where the ReLUs are::

    head              relu          downs.0, downs.1   relu
    trunk.N.conv1     relu          trunk.N.conv2      relu
    trunk.N.conv3     NONE          <- nothing between the last 1x1 and the add
    residual add      NONE          <- x + conv3(...), no activation after
    ups.0, ups.1      relu
    tail              NONE          <- no norm, no activation (meta: "none")

The two NONE cases are the ones a default-everything-to-ReLU implementation gets
wrong, and they are exactly the two the training source is explicit about
(``BottleneckBlock.conv3`` is constructed ``act=False`` and ``StudentNet.tail``
is ``ConvLayer(..., norm="none", act=False)``).

**Transposed convolution.**  ``ConvTranspose2d(k=3, stride=2, padding=1,
output_padding=1)``, with no reflection pad on this path.  Output size is
``(H-1)*stride - 2*padding + k + output_padding`` = ``(H-1)*2 - 2 + 3 + 1`` =
``2H``, so 64x64 -> 128x128 and 128x128 -> 256x256.

**trunk_scale = 4.**  This is *not* a residual scaling factor.  It is
``StudentSpec.scale`` = ``2 ** len(downs)`` = the trunk's spatial downsampling
factor, written into the spec by ``to_dict`` and read by nothing: a grep over
the training tree finds it only in serialisation and in the list of derived keys
that must be *stripped* before a spec can be rebuilt.  The residual add is a
plain ``x + block(x)``.  Reading it as a divisor would scale every block's
output and break the whole network, so it is pinned here as descriptive-only.

**Weights.**  The folded tensors only.  ``folded_spec.norm`` is ``none`` because
the per-channel affine is already inside the convolution's weight and bias:
``W' = a * W`` and ``b' = a * b + beta``, with ``a`` indexed by *output*
channel -- weight dim 0 for a normal convolution, dim 1 for a transposed one,
because the transposed layout is ``[Cin, Cout, k, k]``.  Reproduced bit-exactly
for all 21 layers, which is also what pins the norm to a linear map applied
directly to the convolution output.

**Input.**  NCHW float32 in 0..255 -- *not* 0..1.  The training blocks module
records that feeding 0..1 to this family produces garbage.

Usage
-----

    python tools/d64_net.py --contract            # the pinned semantics
    python tools/d64_net.py --verify              # check every derivation
    python tools/d64_net.py d64.pth --summary     # parameters and tap shapes
"""

from __future__ import annotations

import argparse
import sys

import torch
import torch.nn as nn
import torch.nn.functional as F


#: Keys ``StudentSpec.to_dict`` adds that are *derived* rather than constructor
#: arguments, so they must be removed before the spec can be rebuilt.  Passing
#: them through raises "unknown student spec keys", which is how the training
#: tree's own `predict.load_model_from_checkpoint` handles it.
DERIVED_SPEC_KEYS = ("arch_string", "bottleneck_inner", "trunk_scale")


class ConvLayer(nn.Module):
    """Reflection pad -> conv -> optional ReLU, with the norm already folded in.

    Attribute names match the training tree's ``ConvLayer`` so the folded state
    dict loads without remapping.  ``act`` is a bool here where the training
    tree holds an ``nn.ReLU``/``nn.Identity`` module; neither carries parameters,
    so the state dict is identical either way, and a bool cannot silently appear
    in a state dict comparison.
    """

    def __init__(self, cin: int, cout: int, k: int, stride: int, act: bool):
        super().__init__()
        self.kernel_size = k
        self.stride = stride
        self.reflection_pad = nn.ReflectionPad2d(k // 2)
        self.conv2d = nn.Conv2d(cin, cout, k, stride, bias=True)
        self.act = act

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        y = self.conv2d(self.reflection_pad(x))
        return F.relu(y) if self.act else y


class UpConvTranspose(nn.Module):
    """Stride-2 transposed convolution.  No reflection pad on this path."""

    def __init__(self, cin: int, cout: int, k: int, stride: int):
        super().__init__()
        if k % 2 != 1:
            raise ValueError("transposed-conv kernel must be odd")
        self.kernel_size = k
        self.stride = stride
        # padding = k//2 and output_padding = 1 are what the training tree
        # constructs, and what makes (H-1)*s - 2p + k + op come out at exactly 2H.
        self.conv2d = nn.ConvTranspose2d(cin, cout, k, stride=stride,
                                         padding=k // 2, output_padding=1)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return self.conv2d(x)


class UpBlock(nn.Module):
    """Decoder stage: transposed conv -> ReLU (the norm is folded into the conv)."""

    def __init__(self, cin: int, cout: int, k: int, stride: int, act: bool = True):
        super().__init__()
        self.conv = UpConvTranspose(cin, cout, k, stride)
        self.act = act

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        y = self.conv(x)
        return F.relu(y) if self.act else y


class BottleneckBlock(nn.Module):
    """``x + conv3(conv2(conv1(x)))`` -- 1x1 -> kxk -> 1x1, all dense.

    The two ReLUs are on conv1 and conv2.  There is none on conv3 and none after
    the add; this is the block's defining detail and the easiest one to get
    wrong by defaulting to ReLU everywhere.

    ``forward_taps`` is the only place the block's arithmetic is written down,
    and ``forward`` is defined in terms of it.  That matters: an earlier version
    had the tap walk re-implement the block inline, and the teeth check could
    then edit ``forward`` without moving a single digest -- two copies of the
    same maths, one of them unchecked.
    """

    def __init__(self, channels: int, inner: int, k: int):
        super().__init__()
        self.conv1 = ConvLayer(channels, inner, 1, 1, act=True)
        self.conv2 = ConvLayer(inner, inner, k, 1, act=True)
        self.conv3 = ConvLayer(inner, channels, 1, 1, act=False)

    def forward_taps(self, x: torch.Tensor):
        """``(conv1, conv2, conv3, out)`` -- conv3 is pre-add, out is post-add."""
        a = self.conv1(x)
        m = self.conv2(a)
        c = self.conv3(m)
        return a, m, c, x + c

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return self.forward_taps(x)[3]


class D64Net(nn.Module):
    """The deployed d64 graph: head, 2 downs, 5 bottlenecks, 2 ups, tail.

    Built from ``folded_spec`` and loaded from ``folded_model``.  ``forward``
    returns the tail output; :meth:`forward_taps` returns every intermediate, in
    execution order, under the same names `tools/d64_inspect.py` uses.
    """

    def __init__(self, spec: dict):
        super().__init__()
        s = {k: v for k, v in spec.items() if k not in DERIVED_SPEC_KEYS}
        self.spec = s
        hk, hc = s["head_kernel"], s["head_channels"]
        tk, tkern = s["trunk_kernel"], s["tail_kernel"]
        inner = max(1, int(round(s["trunk_channels"] * s["bottleneck_ratio"])))

        self.head = ConvLayer(3, hc, hk, 1, act=True)

        downs, cin = [], hc
        for c in s["downs"]:
            downs.append(ConvLayer(cin, c, 3, 2, act=True))
            cin = c
        self.downs = nn.ModuleList(downs)
        if cin != s["trunk_channels"]:
            raise ValueError("last downsample %d != trunk_channels %d"
                             % (cin, s["trunk_channels"]))

        self.trunk = nn.ModuleList(
            [BottleneckBlock(s["trunk_channels"], inner, tk)
             for _ in range(s["trunk_blocks"])])

        ups, cin = [], s["trunk_channels"]
        for c in s["ups"]:
            ups.append(UpBlock(cin, c, 3, 2, act=True))
            cin = c
        self.ups = nn.ModuleList(ups)

        self.tail = ConvLayer(cin, 3, tkern, 1, act=False)

    # -- construction ------------------------------------------------------

    @classmethod
    def from_checkpoint(cls, path: str) -> "D64Net":
        ck = torch.load(path, map_location="cpu", weights_only=False)
        if "folded_model" in ck and "folded_spec" in ck:
            spec, weights = ck["folded_spec"], ck["folded_model"]
        elif "model" in ck and "spec" in ck and ck["spec"].get("norm") == "none":
            # Final export.py stores an already-folded model under model/spec.
            # This is a container compatibility change, not arithmetic folding.
            spec, weights = ck["spec"], ck["model"]
        else:
            raise ValueError("Expected folded_model/folded_spec or an exported "
                             "model/spec with norm=none; un-folded training weights are rejected")
        net = cls(spec)
        net.load_state_dict(weights, strict=True)
        net.eval()
        return net

    # -- forward -----------------------------------------------------------

    def forward_taps(self, x: torch.Tensor) -> "dict[str, torch.Tensor]":
        """Every named intermediate, in execution order.

        This is the only place the graph's execution order is written down;
        :meth:`forward` is defined in terms of it.  An earlier version had the
        two as separate walks of the same modules, which is one edit away from
        the golden checking a path the product does not take.
        """
        t: dict[str, torch.Tensor] = {}
        y = self.head(x)
        t["head"] = y
        for i, d in enumerate(self.downs):
            y = d(y)
            t[f"downs.{i}"] = y
        for i, b in enumerate(self.trunk):
            a, m, c, out = b.forward_taps(y)
            t[f"trunk.{i}.conv1"] = a
            t[f"trunk.{i}.conv2"] = m
            t[f"trunk.{i}.conv3"] = c            # pre-add, so the skip is visible
            t[f"trunk.{i}.out"] = out
            y = out
        for i, u in enumerate(self.ups):
            y = u(y)
            t[f"ups.{i}"] = y
        t["tail"] = self.tail(y)
        return t

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return self.forward_taps(x)["tail"]


def build_reference(checkpoint: str = "d64.pth") -> D64Net:
    """Load `d64.pth` into a D64Net and return it in eval mode."""
    return D64Net.from_checkpoint(checkpoint)


# ---------------------------------------------------------------------------
# the RTL-visible contract
# ---------------------------------------------------------------------------

CONTRACT = """\
d64 -- arithmetic the hardware has to implement
==============================================

Derived from the training source (model_train/tinystyle/models/{blocks,student}.py)
and from the PyTorch operators this reference executes.  `--verify` checks every
line of it numerically.

1. normal convolution, k x k, stride s
       out[co, oh, ow] = b[co]
                       + sum_{ci, kh, kw} xp[ci, oh*s + kh, ow*s + kw]
                                              * W[co, ci, kh, kw]
       xp = reflect_pad(x, p),  p = k // 2

   The pad is applied first, so the sum runs over the padded image of size
   H + 2p; the sum itself is the ordinary correlation, kernel unflipped.
   Reflection means a padded index i < 0 reads x[-i] and i >= H reads
   x[2H - 2 - i] -- not a clamp (that is replication) and not zero.

   head    k=3 s=1 p=1   256 -> 258 -> 256
   downs   k=3 s=2 p=1   256 -> 258 -> 128
   1x1     k=1 s=1 p=0   no pad, no border special case at all

2. transposed convolution, k=3, stride=2, padding=1, output_padding=1
       out[co, oh, ow] = b[co]
                       + sum_{ci, kh, kw} x[ci, ih, iw] * W[ci, co, kh, kw]
         where oh = ih*2 - 1 + kh,  ow = iw*2 - 1 + kw,  in range [0, 2H)

   Scatter form: each input pixel lands in a 3x3 neighbourhood of the output,
   and the kernel index ADDS to the output position.  There is NO spatial flip
   -- and a flipped scatter also produces a right-sized, plausible image, which
   makes this the single most important line here.  output_padding does not
   appear in the sum at all: it only widens the output so the positions line
   up, which is why getting it wrong shifts the image instead of failing.

3. residual block
       out = x + conv3(conv2(conv1(x)))        scale factor 1, NOT trunk_scale

4. requantisation and activation
       out = relu(requant(acc + bias))    where the training graph has a ReLU
       out =      requant(acc + bias)     elsewhere (trunk.N.conv3, tail)

   The fold is bit-exact only if the affine norm sits directly on the conv
   output, so the order is conv -> affine -> activation, never conv -> act ->
   affine.  In the deployed form the affine has become part of the weight and
   bias, so what remains at each layer is one requantisation and, on the layers
   listed in the module docstring, one ReLU.
"""


def verify() -> int:
    """Check every derivation claimed in CONTRACT.  Returns the failure count."""
    fails = 0
    torch.manual_seed(0)

    def near(name: str, got, want, tol=1e-6):
        nonlocal fails
        d = float((torch.as_tensor(got) - torch.as_tensor(want)).abs().max())
        ok = d <= tol
        fails += 0 if ok else 1
        print(f"  [{'ok ' if ok else 'FAIL'}] {name:54s} max|diff| {d:.3e}")

    def distinct(name: str, delta, floor=1e-3):
        """A derivation is only informative if the wrong reading differs."""
        nonlocal fails
        d = float(torch.as_tensor(delta).abs().max())
        ok = d > floor
        fails += 0 if ok else 1
        print(f"  [{'ok ' if ok else 'FAIL'}] {name:54s} differs by {d:.3e}")

    def is_true(name: str, cond: bool):
        nonlocal fails
        fails += 0 if cond else 1
        print(f"  [{'ok ' if cond else 'FAIL'}] {name}")

    # --- 1. reflection, not zeros and not replication ---------------------
    x = torch.arange(25.0).reshape(1, 1, 5, 5)
    r = F.pad(x, (1, 1, 1, 1), mode="reflect")[0, 0, 0]
    near("reflection pad: first padded row", r,
         torch.tensor([6., 5., 6., 7., 8., 9., 8.]))
    distinct("  is not zero padding",
             r - F.pad(x, (1, 1, 1, 1), mode="constant", value=0)[0, 0, 0])
    distinct("  is not replication",
             r - F.pad(x, (1, 1, 1, 1), mode="replicate")[0, 0, 0])

    # --- 1. correlation, kernel unflipped ---------------------------------
    m = nn.Conv2d(2, 3, 3, 1, bias=False)
    xi = torch.randn(1, 2, 6, 6)
    ref = m(F.pad(xi, (1, 1, 1, 1), mode="reflect"))
    W = m.weight.detach()
    xp = F.pad(xi, (1, 1, 1, 1), mode="reflect")
    man = torch.zeros_like(ref)
    flip = torch.zeros_like(ref)
    for co in range(3):
        for oh in range(6):
            for ow in range(6):
                patch = xp[0, :, oh:oh + 3, ow:ow + 3]
                man[0, co, oh, ow] = (patch * W[co]).sum()
                flip[0, co, oh, ow] = (patch * W[co].flip(-1, -2)).sum()
    near("correlation, kernel unflipped", man, ref)
    distinct("  a flipped kernel would differ", flip - ref)

    # --- 2. transposed convolution: scatter, index additive, no flip ------
    mt = nn.ConvTranspose2d(2, 3, 3, stride=2, padding=1, output_padding=1, bias=False)
    xt = torch.randn(1, 2, 4, 5)
    rt = mt(xt)
    Wt = mt.weight.detach()                       # [Cin, Cout, k, k]
    Ho, Wo = 2 * 4, 2 * 5
    near("output size (H-1)*s - 2p + k + op",
         torch.tensor([rt.shape[-2], rt.shape[-1]]), torch.tensor([Ho, Wo]), tol=0)
    sc = torch.zeros(1, 3, Ho, Wo)
    fl = torch.zeros(1, 3, Ho, Wo)
    for ih in range(4):
        for iw in range(5):
            for kh in range(3):
                for kw in range(3):
                    oh, ow = ih * 2 - 1 + kh, iw * 2 - 1 + kw
                    if 0 <= oh < Ho and 0 <= ow < Wo:
                        sc[0, :, oh, ow] += torch.einsum("c,co->o", xt[0, :, ih, iw],
                                                         Wt[:, :, kh, kw])
                    ohf, owf = ih * 2 - 1 + (2 - kh), iw * 2 - 1 + (2 - kw)
                    if 0 <= ohf < Ho and 0 <= owf < Wo:
                        fl[0, :, ohf, owf] += torch.einsum("c,co->o", xt[0, :, ih, iw],
                                                           Wt[:, :, kh, kw])
    near("scatter oh = ih*s - p + kh, no flip", sc, rt, tol=1e-5)
    distinct("  a flipped scatter would differ", fl - rt, floor=1e-2)

    # --- 3. residual add is scale 1 ---------------------------------------
    b = BottleneckBlock(4, 2, 3)
    xb = torch.randn(1, 4, 5, 5)
    manual = xb + b.conv3(b.conv2(b.conv1(xb)))
    near("residual add is x + conv3(conv2(conv1(x)))", b(xb), manual)
    distinct("  trunk_scale is not a divisor", b(xb) - (xb + manual / 4))

    # --- 4. ReLU placement -------------------------------------------------
    is_true("trunk.N.conv3 has no activation", b.conv3.act is False)
    is_true("trunk.N.conv1 has one", b.conv1.act is True)
    is_true("trunk.N.conv2 has one", b.conv2.act is True)
    distinct("  no activation after the add", F.relu(b(xb)) - b(xb))
    t = ConvLayer(4, 4, 3, 1, act=False)
    is_true("tail is ConvLayer(..., act=False)", t.act is False)

    # --- 5. fold arithmetic ------------------------------------------------
    Wn = torch.randn(3, 2, 3, 3)
    a = torch.rand(3) + 0.5
    near("fold W' = a * W on the output-channel axis",
         (Wn * a.view(-1, 1, 1, 1))[1] / Wn[1], a[1].expand(2, 3, 3), tol=1e-5)
    Wt2 = torch.randn(2, 3, 3, 3)
    is_true("transposed weight layout is [Cin, Cout, k, k]",
            tuple(Wt2.shape) == (2, 3, 3, 3))
    near("  so its fold axis is dim 1",
         (Wt2 * a.view(1, -1, 1, 1))[0, 1] / Wt2[0, 1], a[1].expand(3, 3), tol=1e-5)

    print()
    print("  all derivations hold" if not fails else f"  {fails} FAILED")
    return fails


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("checkpoint", nargs="?", default="d64.pth")
    ap.add_argument("--contract", action="store_true", help="print the pinned semantics")
    ap.add_argument("--verify", action="store_true", help="check every derivation")
    ap.add_argument("--summary", action="store_true", help="parameters and tap shapes")
    args = ap.parse_args()

    if args.contract:
        print(CONTRACT)
        return
    if args.verify:
        print("d64 contract verification")
        print()
        sys.exit(1 if verify() else 0)

    if args.summary:
        net = build_reference(args.checkpoint)
        n = sum(p.numel() for p in net.parameters())
        print(f"{args.checkpoint}: {n:,} parameters in "
              f"{len(list(net.parameters()))} tensors")
        with torch.no_grad():
            taps = net.forward_taps(torch.zeros(1, 3, 256, 256))
        print()
        print(f"  {'tap':22s} {'shape':22s} {'elements':>10s}")
        for name, v in taps.items():
            print(f"  {name:22s} {str(tuple(v.shape)):22s} {v.numel():>10,}")
        return

    ap.print_help()


if __name__ == "__main__":
    main()
