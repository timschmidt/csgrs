# Rational

`Rational` is the exact arithmetic base for `hyperreal`.

## Representation

`Rational` stores:

- a `num::bigint::Sign`
- a non-negative `BigUint` numerator
- a non-zero `BigUint` denominator

Zero is canonicalized as `NoSign`, numerator `0`, denominator `1`. Non-zero
values are reduced when constructors or operations require canonical form.

## Module map

- `mod.rs`: public module export.
- `arithmetic.rs`: representation, constructors, arithmetic, reduction,
  structural predicates, exact product sums, and tests.
- `convert.rs`: primitive integer and floating-point conversions.
- `parse.rs`: exact text parsing for integers, decimals, and fractions.

## API expectations

- `Rational::new` builds exact integers.
- `Rational::fraction` validates the denominator and reduces exactly.
- finite `f32`/`f64` imports decode the IEEE-754 value exactly, including values
  like `0.3` that are not decimal `3/10`.
- `NaN` and infinities are rejected.
- text decimals and fractions parse as exact rationals; scientific notation is
  not the exact text format.
- `-0.0` imports as canonical rational zero, so IEEE signed zero is not
  preserved.

## Performance expectations

The hot path avoids generic `BigInt` work where the representation already has
the needed facts:

- signs are stored separately from magnitudes
- dyadic denominators reduce by shifts instead of full GCDs
- exact dot products and signed product sums build shared denominators and
  reduce once at the end
- product-sum signs are computed once and reused across reducer stages
- all-zero and single-term sums exit before denominator construction

These optimizations support the higher-level `Real` and `hyperlattice`
matrix/vector kernels, where repeated rational reduction can dominate runtime.

## Numerical explosion controls

`Rational` is the first line of defense against exact-value growth:

- canonical zero and separate sign storage keep common identities small
- finite float imports become exact dyadics, preserving shift-only denominator
  reduction where possible
- shared-denominator dot products and signed product sums accumulate related
  terms before the final reduction
- all-zero and single-term exits avoid building denominators that will be
  discarded immediately
- reducers should use already-known signs, zero checks, and denominator facts
  instead of re-querying scalar properties inside hot accumulation loops

## Error expectations

`Rational` reports divide-by-zero construction or inversion through `Problem`.
Ordinary arithmetic on valid rationals is exact and total except for operations
that explicitly require a non-zero denominator or divisor.
