# Changelog

## 0.1.0 (unreleased)

- Keep exact zero on the other dyadic operand's grid for arithmetic, set
  operations and comparisons. Zero regridding and extreme-scale compositions
  no longer reserve storage for an irrelevant exponent gap; work and size
  exhaustion remain explicit.

- Add tight fixed-point division by an exact signed integer, including divisors
  outside the fixed-point value range and signed extremes, with allocation-free
  arithmetic and checked replay into further operations.

- Add explicit tight midpoint/radius enclosures for fixed, rational, dyadic and
  strict IEEE intervals, including half-grid and minimum-exponent cases.

- Add scale-tagged unsigned fixed widths and retained exact IEEE widths with
  bounded integer value ordering, checked replay and explicit projection.
- Expose budgeted dyadic singleton comparison using existing exact point order.

- Add rational, dyadic and extended dyadic tangent/cotangent images with
  certified interior and endpoint poles, one-sided limits, split components
  and explicit domain coverage. Reuse the existing trigonometric proof helpers.
- Exercise near-pole refinement, subnormal-scale inputs, canonical replay and
  subsequent pole/atan/exponential operations with shared budget accounting.

- Add budgeted rational, dyadic and extended dyadic rational powers with
  arbitrary-size exponent components, reduced real-branch semantics, exact
  integer root witnesses, compact perfect roots and split pole images.
- Exercise rational-power output through canonical replay and further powers
  and exponentials; preserve local and inherited domain qualifications.

- Add normalized IntervalSet2 carriers and extended dyadic division/reciprocal
  with split pole images, explicit domain coverage and canonical replay.
- Preserve exact dyadic quotient bounds before grid expansion, with budgeted
  fraction reduction and continued set/exponential operations.

- Add budgeted rational/dyadic sine and cosine images with certified quadrant
  selection, interior extrema, explicit whole-period results and extended inputs.
- Restore the no-feature bounds-module gate and enforce strict no-feature
  Clippy in CI after reproducing the arctangent insertion regression.

- Add budgeted rational, dyadic and extended-dyadic arctangent images using
  exact reductions, alternating-series tails and certified unbounded limits.
  Preserve exact zero, extreme-scale inputs, local domain receipts and retry.

- Add sealed extended real enclosures with validated singleton ray bounds;
  dyadic set operations, replay and exponentiation consume unbounded results.
- Add budgeted rational/dyadic logarithms with tight bounds and explicit
  partial-domain/lower-unbounded outcomes. Preserve known source-side signs
  during range reconstruction, including near-one rational inputs.

- Add tight rational/dyadic exponential images using budgeted interval Taylor
  tails, range reconstruction and endpoint refinement. Preserve explicit
  domain coverage, negative-tail shortcuts and retry after exhaustion.

- Add allocation-free fixed-point and qualified IEEE square roots with tight
  endpoints and explicit domain coverage. Validate maximal-scale overflow,
  subnormal inputs and independence from native rounding/FTZ/DAZ modes.

- Add budgeted rational/dyadic square-root enclosures with tight grid bounds,
  explicit partial-domain coverage and typed empty-domain failures. Exact
  integer arithmetic establishes direction; generated bounds remain dyadic
  arithmetic and validated replay operands.

- Add exact signed-integer powers for rational intervals, preserving empty,
  zero-domain, parity and monotonicity semantics. Retain scalar-kernel errors
  and their source, including Hyperreal's existing eager-power growth refusal.

- Add exact absolute value and binary min/max images across integer, fixed,
  rational, dyadic and qualified IEEE carriers. Preserve empty-image semantics,
  checked signed overflow, dyadic accounting and IEEE subnormal endpoints.

- Add independent `float-enclosure` finite f32/f64 carriers, initially qualified
  on x86_64, with integer directed arithmetic, fused affine evaluation, bounded
  nonnegative powers, validated bit replay and optional exact endpoint bridges.
- Keep finite-bit validation out of line: optimized inlining could replace
  integer zero checks with native float comparisons and erase subnormal
  proposal inputs under DAZ. Release mode regressions now cover that case.

- Add explicit scalar-set states, sign sets, and typed domain/failure carriers.
- Add checked integer/fixed endpoint carriers and named set relations.
- Add optional checked serialization with no default allocation dependency.
- Add checked integer arithmetic and widened fixed arithmetic, explicit midpoint
  rounding, exact storage conversion, outward rescaling and discrete/real bridges.
- Add independent exhaustive oracles, actual Alumina Q30 differential client,
  bounded arithmetic fuzzing and a paired cost probe.
- Add optional rational intervals, common-scale dyadic arithmetic and directed
  conversion with explicit size/work reservations, and exact-center Real balls.
- Validate Real bridges against directed MPFR bounds and rational arithmetic
  against GMP; compare rational product dispatch in isolated fresh processes.
  Existing solver and domain consumers have not yet migrated.
- Add budgeted dyadic width, exact midpoint/radius, point containment, square
  and directed reciprocal; normalize representable temporary exponent extremes.
- Add exact nonnegative dyadic powers with parity-aware bounds and shared
  size/work accounting, including constant-cost zero and unit endpoints.
- Add separate no_std primitive proposal wrappers with checked finite ingress,
  native estimate arithmetic, checked bit-pattern replay and no proof APIs.
