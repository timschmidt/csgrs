# Changelog

## 0.1.0 (unreleased)

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
