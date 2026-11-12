# Changelog

## 0.1.0 (unreleased)

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
