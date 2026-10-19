# Negative exponential composition admission

Base revision: `4b7b0ed4c`. Hyperreal owns the scalar approximation contract
(P1–P7, P35): an approximation at binary precision `p` differs from the
represented value by strictly less than `2^p`. Exact exponential values remain
positive; a zero approximation is not a zero value or a sign proof.

Hyperspatial's inverse transverse chart exposed a stack overflow while refining
a longitude with normalized easting `10^320`. The independent public scalar
consumer `atan(exp(-10^320)).certified_rational_interval(-128)` reproduces the
abort on the default Rust thread stack, without Hyperspatial in its dependency
graph. The baseline is retained in
[`atan-negative-exp-baseline.log`](../../hyperspatial/verification/2026-10-18-e2-transverse/atan-negative-exp-baseline.log).
Standalone exponential refinement succeeds; composition reaches the deferred
Computable exponential kernel. Its failed logarithmic reduction falls back to
a nested square graph whose depth grows with the input's magnitude.

The extension is a precision-dependent tail bound at that owning kernel, not
a change to the represented expression. With `r` the precision `-4`
approximation, `|16*x-r| < 1`. If `r < 16*min(p,0)`, then
`x < min(p,0)` and `0 < exp(x) < 2^p` (using `e > 2` for negative `p`).
Returning integer zero therefore satisfies this approximation request. All
other inputs keep the existing range reduction and Taylor path. The threshold
uses arbitrary-size integer arithmetic; neither `p*16` overflow nor exponent
conversion is permitted. Cancellation still invalidates the request through
the existing signal contract.

Required evidence: the minimized scalar composition and original transverse
workflow; direct kernel boundary requests; independent MPFR enclosures on both
sides of the threshold; cold/warm and fine/coarse/fine orders; retained nonzero
sign and finer refinement; rational and symbolic negative arguments; serde
replay, cache configurations, cancellation and concurrent consumers where
applicable. This does not establish a general time or memory bound for all
scalar expressions, or repair huge positive exponential materialization.

The 22-certificate sweep also reproduced `Problem::Exhausted` for
`exp(-10^320 * ln(3))`: the integer-log identity called `Rational::powi`
directly, bypassing `Real::powi`'s existing exact symbolic fallback when an
eager rational exceeds its output budget. The same identity should delegate
to the native Real power operation. Rational output budgets remain unchanged;
small powers must still collapse to exact rationals. The representation sweep
and replay tests retain this case with no allowance for exhaustion.

The repairs are retained. The independent consumer now completes on the default
thread stack, and the original ±10^320 transverse fixtures complete inverse,
reprojection and ECEF consumption. The six public regression tests cover all
22 certificate families, sign preservation, 512-bit requests, ordinary exact
integer-log collapse, JSON/CBOR replay, cancellation and concurrent readers.
Two kernel tests cover precision endpoints and 315 directed-MPFR comparisons
around the proof threshold (nine precisions, 35 inputs each).

Executed evidence is retained with the
[transverse validation bundle](../../hyperspatial/verification/2026-10-18-e2-transverse/README.md):
the full Hyperreal test run has 1,115 passing executions; the final focused
cache matrix has 28 executions including the two kernel tests. The later
ordinary-integer-collapse regression is included in that focused matrix.
Hyperlattice/Hyperlimit library tests pass (271 executions), and all-target,
all-feature Hyperreal Clippy passes with warnings denied. The native regression
suite also passes on Rust 1.95.0. Final downstream source, coverage, sanitizer
and compiler evidence is recorded in the linked bundle.

The fresh-process release probe completes five trials at precision -128; its
time and peak RSS include startup and the fork/exec harness, with other
validation work running on the host. These establish completion under that
measurement setup, not a speed ratio against the aborting baseline. No
primitive output demotion, larger thread stack, rational-budget increase or
weakened acceptance threshold is retained.
