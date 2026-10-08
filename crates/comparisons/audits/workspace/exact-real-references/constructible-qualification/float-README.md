# Cancellation-stable floating views — 2026-09-06

Unchanged Constructible source/dependencies and the build caveats are recorded
in boundary-README.md. FloatProbe emits the exact rational value of each native
Double as well as its decimal rendering, so the independent oracle does not
confuse decimal parsing with the binary result being tested.

194 signed cases per optimization level: the documented 14-radical close sum
and its negative, plus both signs of q*sqrt(2)-p for 96 Pell convergents. Starting
from (p,q)=(1,1), iterate (p+2q,p+q). The independent oracle checks p²-2q²=±1.
Values reach approximately 1.79e-37 without converting a tiny residual through
uncompensated primitive subtraction. Native O0/O2 output is byte-identical,
SHA2562533c29c98e9faa916f3b9353d6f7c943d583d9666372afdda3efc363e3467f8.

`float_oracle.rs` independently computes 4096-bit directed-MPFR intervals for
each square root and combines endpoints as exact Rug rationals. For each case:

1. Check the native Double lies within the reference interval enlarged by
   relative 2^-48 (not a correctly-rounded or 0.5-ulp claim).
2. Check public Hyper Real::to_f64_lossy is finite and meets the same bound.
3. Independently build the unsimplified expression with public Computable
   operations and check its 512-bit approximation, with radius 2^-512,
   encloses the full directed reference interval.

All 582 checks pass for each native-output/Hyper-build combination: O0/debug,
O0/release, O2/debug and O2/release. Every process exits0 under its60s cap; no
timeouts. This demonstrates the donor's conjugation benefit on these inputs
and confirms no corresponding Hyper gap in this corpus. It does not rescind
the separately reproduced extreme-scale overflow limitation. No production
change or general correct-rounding promise follows.

Two Rust harness build failures were corrected before execution and frozen
successful binaries: Option<f64> needed explicit unwrapping, and one Rug shift
needed an Integer type annotation. Their logs remain separate. No library code
was changed to make the harness compile or the tests pass.

Frozen sources:

- FloatProbe.hs:2432a1263ead3b7065643a86ac1a336e713b9131629fc39c70dfea2a4519783b
- float_oracle.rs:4231672af009549533e07234082f11600dd22a501ea626c614a91c987f39badb

Frozen binaries under `.audit-constructible-build.l4UDoe/`:

- float-before-O0:8f4b8c6644e578e70a649c98313d55646d66693cfd9aff70cbe533357649d5e7
- float-before-O2:dc810f9b6d29a659c44d242a7056017850052f4b2cb576520e5ce3e4f5aaf550
- float-oracle-debug:7cc704c5624f4ae6060c33149bbd8101f686587a520cb475e6d5d5ac044aaf23
- float-oracle-release:d0996a5eb954196ea378b99abcf0535c6dc37055def559e7fbfce76197c48504

Reproduction: build FloatProbe.hs using the direct GHC recipe, capture its
output, then run each frozen float-oracle binary with that file as its only
argument. Logs are `float-O{0,2}.log` and `float-oracle-{debug,release}-O{0,2}.log`.
