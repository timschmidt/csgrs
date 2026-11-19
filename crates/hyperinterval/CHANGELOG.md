# Changelog

- Add `F32ProposalInterval::radius_estimate` for finite native half-widths,
  including subnormal intervals and the full binary32 range.

- Add budgeted, tight outward dyadic-to-F32/F64 enclosure conversion, preserving
  finite overflow and unsupported-environment diagnostics. Qualify exact output
  bits, retry, native rounding-mode independence and actual WASM execution.

## 0.1.0 (unreleased)

- Rational square roots now charge the stored width of positive generated
  endpoints before reduction. Dyadic and rational square roots use one
  unsigned division to determine both tight output bounds.

- Rational sine, cosine, tangent and cotangent check both stored fractions
  against the caller's budget before reducing either endpoint. Generated
  coordinates with large common factors can now exhaust the budget at ingress.

- Add `RationalInterval::scale` for exact scalar multiplication by a borrowed
  rational, including negative and zero factors.

- Add opt-in certified binary32 and binary64 enclosures on `wasm32-wasip1`
  through `float-enclosure-wasip1`. The threaded ABI retains typed refusal.

- Add exact binary32 scalar ingress to `DyadicInterval` with binary32 significand
  accounting under either float feature.

- Add exact dyadic scalar scaling and affine images with bounded endpoint products and
  shared work accounting for translation.

- Add exact, budgeted overlap and disjointness queries for finite, unbounded
  and split dyadic sets. Queries preserve contact at closed endpoints and gaps
  between components.

- Add exact, budgeted midpoint bisection for closed dyadic intervals.

- Add fused integer affine hulls for all primitive endpoint types. Signed
  cancellation can recover a representable result from an out-of-range product.

- Add exact fixed-point interval conversion to the optional dyadic and rational
  backends. The dyadic conversion charges the caller before allocating.

- Qualify the allocation-free integer IEEE enclosure backend on
  `wasm32-unknown-unknown`, with exact-bit oracle execution and generated-code
  checks. Other previously unsupported targets still fail closed. Proposal
  intervals remain non-authoritative; native arithmetic and rounding are unchanged.

- Normalize temporary dyadic product exponents when the exact hull endpoints fit
  the storage format, using the same budgeted normalization as squares/powers.
  Previously rejected exponent sums can now succeed or reach a declared size/work
  limit; genuinely unrepresentable lower endpoints still return `Overflow`.

- Share exact primitive lattice midpoint arithmetic between integer and fixed
  intervals, retaining all rounding modes and full unsigned widths. Fixed
  accessors avoid widened arithmetic and remain allocation-free.

- Preserve proved endpoint order during fixed arithmetic reconstruction and
  exact storage conversion. Use native checked additive operations and explicit
  checked narrowing without changing rounded bounds or failure outcomes.

- Add allocation-free signed integer quotient bounds and exact scalar images of
  primitive integer ranges, including signed extremes and typed overflow. Share
  directed quotient arithmetic with fixed-point operations.

- Add budgeted `DyadicInterval::from_rational` and use it for structurally
  rational `from_real` inputs. Stored coefficient sizes are admitted before
  copying, normalization or division, including lazy unreduced ratios. Exact
  outward grid bounds are preserved; symbolic refinement keeps its separate
  resource contract.

- Add dyadic principal `asin`, `acos`, `acosh` and `atanh` enclosures using
  certified native arithmetic, square root, arctangent and logarithm. Preserve
  complete/partial/empty domains, exact zero special values and unbounded atanh
  endpoint limits. Bounds are outward but are not promised tight.

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
