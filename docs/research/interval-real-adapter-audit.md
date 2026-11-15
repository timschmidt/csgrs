# Hypersolve symbolic interval migration boundary

This is a Phase 0/7 ownership audit and acceptance witness, not an accepted
arithmetic migration. It covers all 490 lines and 19 declarations of
[`real_interval.rs`](../../crates/hypersolve/src/real_interval.rs) at
`1daa5f87581d1ce88ccf74dd5e64d18567c387e8`, SHA-256
`3800abd4d0bc0941e756933758699443922d11965d80c3be793def3d3c4eb8de`.
No production body changes. The inventory records body review separately from
selected caller review; the broader call graph and inventory remain unfinished.
The governing properties are P1, P4, P6, P8–P10, P17–P20, P27, P36 and P37.

## Observable boundary

`RealInterval` has two public exact `Real` endpoints. It is a conservative
closed enclosure adapter, with neither an empty variant nor a checked public
constructor. Ordered valid inputs are the arithmetic precondition; public field
access does not prove it. It derives Clone, Debug and PartialEq, not serde traits.
The module is unconditional and reexports the carrier from the crate root;
Hypersolve requires std and its declared Rust 1.95 compiler. Exact symbolic
comparisons belong to Hyperlimit, which cannot become a Hyperinterval dependency.

The [migration acceptance suite](../../crates/hypersolve/tests/real_interval_migration_contract.rs)
exhibits a bound difference on the simplest strict-straddling inputs:

| Operation chain | Existing symbolic adapter | Tight rational carrier |
| --- | --- | --- |
| `[-1,1] × [-1,1]` | `[-2,2]` | `[-1,1]` |
| Square that generated enclosure | `[0,4]` | `[0,1]` |

Both product enclosures contain every exact point product. They are different
represented sets and different subsequent operands. The symbolic implementation
sums the two nonpositive and two nonnegative corner products to avoid comparing
deep expressions. This is deliberate, even when its endpoints happen to be
rational. A tight rational replacement cannot be called an identical-output
refactor. A separately admitted tightening algorithm is possible, but must
qualify changed bounds, statuses, branch selection, dispatch and consumer costs.
The governing preference order and existing migration gates are unchanged.

## Declaration dispositions

All rows below have body review. Caller coverage is only the scope specified
below; it is not inferred from a symbol search. Category numbers refer to
Section 7.1 of the implementation plan.

| Declaration | Contract and order to retain | Disposition |
| --- | --- | --- |
| `RealInterval` | Two exact endpoints; no empty/validation/receipt carrier | 3: retain symbolic proof adapter |
| `strict_real_sign` | Structural zero first, then STRICT predicate; undecided becomes `None` | 3: retain policy adapter |
| `strict_compare_reals` | Pointer identity, rational comparison, STRICT comparison, then sign of difference | 3: retain; pointer equality is only a sufficient shortcut |
| `compare_to_zero` | Immediate sign before general strict comparison | 3: retain distinct cheaper decision sequence |
| `strict_nonzero_sign` | Lower positive first, otherwise upper negative; zero-containing does not certify zero | 3: retain proof query |
| `add` | Lower sum before upper sum; exact expression endpoints | 3: retain until a qualified exact-symbolic replacement exists |
| `subtract` | Lower minus other upper, then upper minus other lower | 3: same boundary as addition |
| `multiply` | Four endpoint sign queries precede case dispatch; strict-straddling sums, otherwise extrema/fallback hull | 3: retain observed enclosure and predicate order |
| `square` | Sign-specialized dependent square; spanning-zero magnitude comparison may fall back to sum of squares | 3: retain symbolic comparison/uncertainty behavior |
| `divide` | Positive and negative denominator tests, then decreasing reciprocal endpoints and multiply | 3: retain zero-exclusion evidence and evaluation order |
| `evaluate_power_basis` | Reverse coefficient Horner iteration using this adapter's product and endpoint additions | 4: retain polynomial enclosure algorithm; not tight polynomial range |
| `evaluate_bivariate_power_basis` | Reverse outer rows, multiply by first interval, then add inner Horner image over second | 4: retain polynomial operation order and dependency inflation |
| `nonnegative_square_root` | Upper/lower sign admission, nonnegative-domain clipping, optional outward coefficient rounding, exact radical fallback | 3: retain symbolic/domain adapter; generic endpoint rounding remains a migration candidate |
| `from_values` | Empty array gives `None`; compare each next value to lower, then upper | 3: retain strict hull/uncertainty behavior |
| `rational_interval_bernstein_strict_sign` | Closed-domain polynomial sign certificate from rational control enclosures, optional bounded subdivision | 4: retain polynomial proof ownership in Hypersolve |
| `strict_bernstein_bound` | Strict first/last controls, weak interior signs; zero interior controls are allowed | 4: retain polynomial certificate helper |
| `strict_signs_are_opposite` | Only decided positive/negative pairs match; zero or `None` never count | 3: retain sign-proof convenience; no scalar interval arithmetic to relocate |
| `rational_dyadic_square_root_bound` | One directed rational endpoint, nonpositive grid exponent, exact integer root and square check | 1: generic arithmetic candidate for Hyperinterval, migration not yet qualified |
| `rational_square_root_bounds_are_outward_dyadic_and_exact_on_squares` | Existing private root-rounding regression | Preserve test; not production arithmetic or a migration completion |

The scalar root helper cannot simply be replaced by the current budgeted
`RationalInterval::sqrt`: that API admits empty/partial domains, returns both grid
endpoints and coverage, and charges a `DyadicBudget`. The legacy helper returns
one rational endpoint or `None`, rejects positive exponents to trigger its
caller's existing fallback, and has no comparable total-work contract. Reuse the
existing owner arithmetic when attempting this migration, while preserving the
consumer's decisions and proving its costs; do not add an unchecked duplicate.

## Reviewed consumer scope

The following specific bodies were read, not merely located by search:

- `represented_coordinate_interval` and `represented_univariate_coordinate` in
  `represented_root.rs`: retain exact-point handling, derivative exclusion,
  endpoint root existence checks, source metadata and final root validation.
- `dense_two_positive_square_root_interval_with_coefficient_precision`, its
  no-precision wrapper, `dense_positive_square_root_interval_with_coefficient_precision`
  and its wrapper in the same module: retain source tuples, omit stored-zero
  radical terms before root admission, reuse roots within the expression, and
  propagate failed optional bounds.
- `RecursiveQuadraticValue::interval_over_source_box` and
  `interval_over_source_box_with_witnesses`: retained witnesses can supply a point
  interval before recursive box evaluation. Replacing that path by independent
  coordinate bounds would lose correlation. Stored-zero radical coefficients
  skip otherwise unnecessary square-root decisions.
- `bivariate_fiber_strict_sign_on_parameter_range` in Hypercurve
  `bezier_offset/parallel_components.rs`: the one named production reference to
  the rational Bernstein helper found in the workspace. Its entire body was
  reviewed, including finite-enclosure admission, polynomial restriction,
  selected-root refinement, strict counterpart, the rational control stage, and
  later selected-control/subdivision fallbacks. The helper's `None` leaves those
  fallbacks available; it is not a negative geometric classification.
- `coefficients_value_interval_on_parameter_interval` and
  `coefficients_dyadic_convex_hull` in Hypercurve `bezier_parameter.rs`: certified
  scalar bounds form the ordered rational control pairs consumed by that stage.

The separator-strip call context in `bivariate_topology.rs` was also inspected:
Horner zero exclusion determines whether further refinement/root counts are
needed. Its entire enclosing algorithm was **not** audited here. Other uses of
`RealInterval`, strict sign/comparison helpers, external consumers and serialization
through enclosing reports are not exhaustively reviewed. Search results are
navigation evidence only. No caller is newly migrated by this audit.

## Bounds, resource and qualification limits

`None` is the existing optional-enclosure result. It does not carry a reason
that distinguishes undecided order, unavailable bounds, an excluded quotient
or an invalid square-root domain. This audit does not establish the complete
typed-outcome target for the stack. Successful enclosures use strict decisions;
there is no approximate terminal in the adapter itself.

Scalar arithmetic and expression allocation remain governed by their existing
owners. The optional square-root precision is a grid choice, not a total-work or
heap budget. The helper's integer shifts can grow with precision/coefficient
size; no extreme-resource completion is established here. No new allocation or
latency bounds follow from the finite regression corpus.

The Bernstein routine first tries control-sign certificates, checks endpoint
compatibility, clears lower/upper denominators separately, then uses the existing
integer de Casteljau helper. That helper gives each child's controls one common
positive scale, preserving signs. The depth-10 / visited-at-least-256 stop test
runs on an unresolved segment; it is not a hard cap of 256 total popped nodes,
because already certified pending nodes can still be consumed. These controls
do not bound degree, coefficient payload, heap or total scalar work.

The five new tests cover 784 ordered integer interval pairs (81 produce a wider
symbolic product), generated-result composition, exact sqrt(2) endpoints,
quotient exclusions, and independent quadratic Bernstein expectations. Existing
Hypercurve tests supply the strict-only uncertainty and polynomial fallback
controls. The progress ledger and archived command records report their actual
execution outcomes. No direct named RealInterval benchmark was found in the
Hypersolve/Hypercurve benchmark source search; that search is not a cost study.
A future migration still needs complete owner/downstream dispatch, allocation,
latency, code/binary size and supported-composition qualification.
