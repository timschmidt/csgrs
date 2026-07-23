# Rational

`Rational` is the exact arithmetic base for `hyperreal`.

## Representation

`Rational` stores:

- a `num::bigint::Sign`
- a non-negative `BigUint` numerator
- a non-zero `BigUint` denominator

Zero is canonicalized as `NoSign`, numerator `0`, denominator `1`. Non-zero
values are reduced when constructors or operations require canonical form.
One internal exception serves fused exact-geometry kernels: a result may retain
an exact numerator/denominator pair whose remaining common factor has not yet
been divided out. The stored ratio is still the exact value. Public numerator
and denominator access, exact extraction, hashing, formatting, debugging,
serialization, integer/dyadic classification, roots, and lossy IO views all
observe a canonical reduction. Exact sign and cross-product comparisons can use
the unreduced ratio directly. The first canonical observation is computed once
and shared safely by every clone and thread.

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
- small reduced dyadics share canonical storage; immutable operand pairs retain one
  exact product, while repeated borrowed pairs adaptively retain a sum or directed
  difference through weak keys
- wide dyadic products keep word-sized numerators in `u128` even when the combined
  power-of-two denominator requires `BigUint`
- exact dot products and signed product sums build shared denominators and
  reduce once at the end; dyadic dots first align checked `u128` products and
  fall through to `BigUint` unchanged on overflow
- fused exact-dyadic line intersections reduce their two ordering parameters
  eagerly but retain the two point coordinates as exact internal quotients;
  output assembly can compare and clone those coordinates without paying two
  otherwise-unused odd GCDs per proper crossing
- product-sum signs are computed once and reused across reducer stages
- all-zero and single-term sums exit before denominator construction

These optimizations support the higher-level `Real` and `hyperlattice`
matrix/vector kernels, where repeated rational reduction can dominate runtime.

## Numerical explosion controls

`Rational` is the first line of defense against exact-value growth:

- canonical zero and separate sign storage keep common identities small
- lazy internal intersection quotients reuse the existing primary cache slot
  for their canonical value; this adds no field to the 88-byte rational node,
  and public or dyadic-specialized consumers cross that boundary explicitly
- finite float imports become exact dyadics, preserving shift-only denominator
  reduction where possible
- bounded product retention reuses repeated coefficients without changing canonical
  numerator/denominator values or retaining either operand strongly
- repeated powers two through five first use the direct integer kernel, then reuse
  that same bounded product graph when the immutable base is observed again
- adaptive linear-result retention reuses repeated exact translations and differences;
  unshared first-use operands only record a one-byte hint, the second observation
  admits a bounded result, and later calls reuse it; the lazy arithmetic box has room
  for up to three weak-keyed sums, differences, or secondary products while `RationalData`
  remains 88 bytes
- dense identity-equal three- and four-term self-dots reuse those bounded product
  and linear entries after observation; one conflict-attempt bit admits a square when
  the primary product belongs to a shared inverse norm, while cold, sparse,
  distinct-operand, and full-cache rows keep the aggregate zero-pruned reducer
- repeated square-root reductions use the same adaptive schedule, retaining the exact
  square factor and residual only after reuse is observed; the dedicated
  lazy slot remains cycle-free and does not displace either linear result or unary pair
- shared rationals retain one exact reciprocal in that same bounded lazy box; the
  reciprocal points back weakly, avoiding ownership cycles while stabilizing the
  multiplier identity used by repeated scalar division
- repeated sign flips use a second cycle-free unary slot in the lazy box; the source
  owns its opposite sign and the reverse edge is weak, while reciprocal and both
  linear-result entries remain independently available
- shared-denominator dot products and signed product sums accumulate related
  terms before the final reduction; word-sized dyadic accumulators avoid one
  arbitrary-precision allocation per dense vector lane
- all-zero and single-term exits avoid building denominators that will be
  discarded immediately
- reducers should use already-known signs, zero checks, and denominator facts
  instead of re-querying scalar properties inside hot accumulation loops

## Error expectations

`Rational` reports divide-by-zero construction or inversion through `Problem`.
Ordinary arithmetic on valid rationals is exact and total except for operations
that explicitly require a non-zero denominator or divisor.
