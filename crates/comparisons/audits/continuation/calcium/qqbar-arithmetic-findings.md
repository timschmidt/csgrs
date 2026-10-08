# Algebraic arithmetic and relation boundaries — checkpoint 51

This checkpoint completes 54 selected full-file reads / 7,223 new lines across
the two pinned implementations: 27 archived Calcium files / 3,654 lines and
27 current FLINT files / 3,569 lines. It includes the 21 files provisionally
recorded before the report handoff, not an additional credit for those reads.
Thirty-two implementation files and twenty-two associated test files are covered.
Both copies were read independently; no credit comes from similarity, navigation
output or the metadata hash check. This is not all of qqbar or the ecosystem.

## Arithmetic architecture

Rational operands, zero, one and minus one take short exact paths. An affine
image `(a*x+b)/c` uses one inverse polynomial substitution, primitive-content
normalization and a separately validated root enclosure. A nonconstant affine
map preserves irreducibility, so it need not factor a fresh general resultant.
Zero slope and rational input use direct rational construction. The nonzero
denominator is a precondition, not something interval overlap establishes.

General binary operations construct a degree-product annihilator, factor it,
then select the factor and root using an arithmetic image enclosure and a
uniqueness certificate. A zero-containing polynomial residual only filters
candidates. Same-degree inputs of degree at least four first try a rational
guess; any guessed identity must survive inverse arithmetic and exact equality.
The read standalone guess interface is heuristic; its overlap acceptance is
not a proof that an arbitrary input approximation denotes that algebraic value.

The composed-polynomial routine obtains root power sums from reciprocal
polynomial logarithmic derivatives. Products use componentwise multiplication
of power sums. Sums use factorial-scaled convolution; subtraction negates one
root family. Division reverses the second polynomial and requires its constant
coefficient to be nonzero. Integration/exponentiation and reversal reconstruct
the annihilator. All root-pair multiplicities are retained; it is not generally
the minimal polynomial of the selected arithmetic result.

Inverse arithmetic reverses the minimal polynomial and certifies the reciprocal
enclosure. Dyadic scaling updates exact coefficient exponents and removes common
powers of two. Powers exploit rational values, roots of unity, positive surds,
polynomial deflation and a special square construction. Generic powers delegate
to polynomial evaluation. Those local shortcuts are representation-dependent;
they are not a reason to change Hyper's principal/real-root or partiality contracts.

Polynomial evaluation first handles constants/rationals/affine cases, then
reduces the polynomial modulo the defining polynomial. The general field path
constructs the multiplication matrix of the field element, obtains its minimal
polynomial and validates the selected enclosure. Current FLINT explicitly
canonicalizes a rational evaluation result; the archived path lacks that line.
No archived runtime claim is made. Storage-lifetime concerns in the remainder
path remain source-only; no malformed, promoted-coefficient or failure-path
probe was constructed to investigate them.

Polynomial-value equality uses exact modular composition to prove that the
candidate is a root, then certifies the correct embedding by enclosure
refinement/uniqueness. Field-expression discovery uses a numerical integer
relation as a candidate and calls that exact equality check. The `max_bits`
and `flags` parameters are unused in the read field-expression implementation;
that is a source observation, not qualification of a resource limit. Direct
LLL and guess experiments remain separate and were not run here.

The upstream tests exercise algebraic identities, aliasing and mixed degrees,
but mostly compare operations sharing qqbar's backend. The general-power test
only checks an identity when every requested power succeeds. The field-
expression test accepts a reported success without independently comparing
the returned coefficients. All 22 tests were read, not newly executed.

## Independent exact qualification

The collector constructs sixteen explicitly specified values in
`Q(sqrt(2),sqrt(3))`, spanning zero, signed rational and quadratic values,
biquadratic sums, cancellations, and fractional mixed values. The basis is
`[1,sqrt(2),sqrt(3),sqrt(6)]`. Only small valid values and nonzero divisors are
used. Each value is checked in its initial state and after explicit 256-bit
enclosure caching.

The corpus contains:

- 7,344 value results: inputs, all valid ordered binary operations with distinct
  output/left alias/right alias, inverse, signed powers -3 through 4, dyadic
  scaling, three affine maps and five rational polynomials, including remainder
  and number-field evaluation paths. Unary/evaluation results use distinct and
  in-place outputs. These are repeated operations and states, not 7,344 distinct
  algebraic numbers.
- 1,008 full composed annihilators before caching, preserving the complete
  Cartesian product of input conjugates, including repeated image values.
- 2,560 polynomial-value decisions, with 44 exact equalities and 2,516 unequal
  controls derived independently from the field coefficients.
- 32 readonly polynomial/enclosure preservation records and one terminal record.

The independent JavaScript oracle implements rational BigInt arithmetic in the
four-dimensional field. Basis multiplication follows the two square relations;
inversion uses the other three Galois images and their rational norm. Distinct
sign-flip images give all conjugates. Multiplying their linear factors, then
clearing denominators and content, gives the expected full minimal polynomial.
This uses the rational independence of the four basis elements; it is not a
general-purpose minimal-polynomial algorithm for arbitrary algebraic inputs.

Every exported root enclosure is checked against exact directed rational bounds
computed by integer square roots, refining independently through a bounded
schedule. Rational endpoints are compared by integer cross-products. No qqbar
operation, binary64 root or interval-overlap assertion supplies the mathematical
oracle. Every result also has exact zero imaginary part and absolute enclosure
width at most `2^-96` for this small-value, 128-bit-output corpus. That width
check is not a universal relative-error or correctly-rounded-output claim.

All 10,945 expected records are present without duplicates. All **40,352
assertions pass**: 7,344 each for minimal polynomials, ordered endpoints,
containment, width and real-axis exactness; 1,008 full annihilators; 2,560
relations; and 64 input-preservation checks. The checker independently verifies
the complete record membership and terminal counts. This does not establish
branch coverage, arbitrary complex arithmetic, direct LLL correctness, arbitrary
field membership completeness, archived execution, or Hyper qualification.

Native and focused Memcheck stdout are byte-identical, 2,032,198 bytes each.
Memcheck reports zero errors/suppressed errors and zero live blocks, with
3,180,130 allocations/frees and 150,670,769 cumulative requested bytes. Setup,
collection and output are included; this is not an operation-only allocation
benchmark or a peak/RSS measurement. Earlier failing checkpoints remain intact.

One small executable in `/tmp/calcium-qqbar-arithmetic.oUgkCO/controls` reuses
the existing pinned FLINT and system libraries. No dependency/Rust rebuild,
source-tree copy, cleanup, deletion, production/donor edit, external report,
commit or push was performed. Captured collection time is not a performance
comparison. The manifest records exact executable/library/source identities.

## Hyper comparison and open transfer

The complete live `hypersolve/src/algebraic_binary.rs` was read. Its binary
constructor checks represented-root evidence, exact-rational coefficients and
the existing product-degree cap of nine. Oversized repeated carriers can first
be square-freed, with reuse for shared carriers. Divisor intervals must already
be certified away from zero. The arithmetic image is accepted only after Sturm
root counting/refinement and representation validation; a new polynomial
construction must leave those checks in place.

Its private polynomial kernel samples exact resultants at consecutive integers
and interpolates. The supporting exact-value path already avoids the full public
report and uses a flat BigInt Sylvester matrix with fraction-free Bareiss
elimination. Integer interpolation already uses factorial-scaled forward
differences and removes common content once. The complete interpolation module
and the relevant resultant ranges were read. No claim is made that the entire
resultant module or its recursive dependencies became source-complete here.

Consequently a power-sum construction is a genuine alternative to compare, not
an already-covered optimization. A bounded Newton-recurrence implementation can
construct the same polynomial without repeated determinant samples. For monic
`P = X^m + p1*X^(m-1) + ...`, Newton's identities give its root power sums.
Addition/subtraction combine them with binomial convolution; multiplication
uses pointwise products; division uses the reciprocal root family. A second
Newton recurrence reconstructs the monic output coefficients, followed by exact
primitive-integer normalization. These are mathematical construction ideas,
not an implemented or measured Hyper candidate.

The next candidate must address the **full current Hyper contract**:

1. Preserve rational, nonmonic, signed, reducible and repeated-root carriers,
   including output multiplicities and equivalent primitive scaling. A selected
   root's minimal polynomial is not a replacement for the entire current
   resultant carrier unless that API change is separately justified.
2. Handle a divisor carrier with an unused zero root. The selected divisor
   interval may be nonzero even when its polynomial constant is zero. Direct
   reciprocal power sums are then unavailable; retaining the existing exact
   fallback or proving/removing only the irrelevant zero factors is necessary.
3. Keep the degree cap, square-free preprocessing, policy/Unknown outcomes,
   nonisolating-image rejection and all root-evidence validation unchanged.
4. Compare full output polynomials independently, not only a residual at one
   root. Extend the current biquadratic corpus to repeated/reducible carriers,
   cubic and other currently supported degrees and signed rational scales.
5. Measure isolated polynomial construction and complete public queries against
   a frozen baseline using paired CPU, allocation and representative size
   campaigns. Rational coefficient growth could erase savings from removing
   determinants. Include scalar/consumer regressions and slower controls before
   retention; reuse the existing build cache.

No Hyper prototype or matched benchmark has been run in checkpoint 51. The
candidate remains open for implementation and qualification, not rejected or
retained. All five earlier retained transfers remain unchanged. Further qqbar,
recursive polynomial/root/field support, every remaining ecosystem reference
and full inventory reconciliation are still required for the original goal.
