# Performance and Reference Audit

This document records how every source in the README reference list maps to
`hypersolve`, which changes were supported by measurements, and which ideas are
deliberately bounded or architecture-inapplicable. Solver proposals may cross a named lossy
boundary, but no optimization may turn such a proposal into proof.

## Reference-by-reference findings

| Reference | Applied finding and disposition |
| --- | --- |
| Bareiss, integer-preserving Gaussian elimination | The exact dense and sparse direct solvers use the fraction-free recurrence and certified pivots. The dense solver formerly evaluated one Bareiss determinant for the matrix and one per Cramer numerator. It now eliminates the augmented system once, retains the same determinant/numerator/replay report, and falls back to the former Cramer construction if an augmented exact division is not representable. This is the largest retained optimization below. |
| Bouma et al., geometric constraint solving | Graph reduction and directed algebraic decomposition correspond to prepared dependency facts, equality-substitution classes, affine row elimination, solver blocks, and sketch lowering. These exact/direct passes remain ahead of the nonlinear proposal loop; solution choice and interactive dragging are reported rather than hidden. |
| Collins, subresultants and reduced polynomial remainder sequences | `resultant` exposes exact Sylvester resultants and a fraction-free pseudo-remainder chain for common-factor evidence. It does not claim the complete multivariate reduced-PRS system from the paper. The audit retained binary powering for the constant-polynomial resultant case. |
| Collins and Loos, real zeros of polynomials | `root_isolation` extracts exact-rational univariate rows, performs square-free reduction, constructs Sturm evidence, isolates distinct roots, refines bounded intervals, and replays rational witnesses. Unsupported coefficients and incomplete refinement remain explicit statuses. |
| Descartes, *La Geometrie* | Power-basis sign variation provides exact positive-root count bounds after certified degree trimming. The bound is used as a filter/report and is not treated as an exact root count when the parity gap remains. |
| Farin, CAGD | Bezier/Bernstein evaluation, subdivision, power-basis substitution, and variation behavior support curve-residual preparation and root intervals. Hypersolve keeps these as algebraic solver objects; curve topology remains owned by `hypercurve`. |
| Farouki and Rajan, Bernstein-form algorithms | Exact Bernstein conversion, interval sign variation, and midpoint subdivision drive Bernstein count and subdivision reports. The implementation preserves the form where its convex-hull/sign information is useful instead of converting every stage back to power basis. |
| Krawczyk, interval Newton operators | Affine, univariate quadratic, and multivariate quadratic Krawczyk reports construct exact center steps and contraction/remainder bounds. Inclusion or exclusion is decided by exact comparisons; singular or unsupported inverses are reported rather than guessed. |
| Levenberg, damped least squares | The dense proposal adapter adds a diagonal damping term to normal equations. Its `f64` precision boundary, pivots, and damping are retained in diagnostics, and exact candidate replay remains separate. |
| Marquardt, nonlinear least-squares estimation | The named Levenberg--Marquardt proposal uses the damped least-squares route. The current prototype does not claim a complete modern gain-ratio/damping policy; this limitation remains visible rather than being labeled as a certified nonlinear solve. |
| Moore, interval analysis | Variable balls, residual interval bounds, exact endpoint/magnitude comparisons, and unknown statuses follow interval-analysis discipline. Intervals are proof surfaces for supported affine/quadratic rows, not universal replacements for symbolic structure. |
| Nocedal and Wright, numerical optimization | Dense BFGS, dogleg/trust-region, and an equality least-squares SQP relaxation are implemented as named lossy proposal engines. They do not provide convexity, KKT, line-search, or constrained-optimality certificates, so exact feasibility and domain semantics remain separate. |
| Powell, hybrid method | Powell-hybrid and dogleg names route to the dense dogleg trust-region proposal. The step is bounded and diagnosed, while acceptance still depends on exact residual/candidate certification. |
| Sederberg and Nishita, Bezier clipping | Bernstein convex-hull sign exclusion and recursive interval contraction inform the root-subdivision surfaces. Hypersolve returns algebraic parameter intervals; it does not duplicate `hypercurve`'s curve/curve topology machinery. |
| Shoemake, quaternion curves | Unit quaternions and quaternion-derived frame axes are used for exact sketch workplanes, projection, and symmetry constraints. Spherical interpolation was not added because this crate constrains static frames rather than generating animation curves. |
| Smale, one-point Newton estimates | Prepared univariate quadratics expose an exact conservative alpha certificate, using the rational threshold `1/8` to avoid adding a radical solely for the bound. Exact roots, multiple roots, zero derivatives, failed bounds, and undecided comparisons are distinct statuses. |
| Sturm, numerical equations | Exact Sturm sequences count distinct roots over rational intervals and guide isolation/refinement. Endpoint roots and multiplicities are handled through square-free and replay evidence instead of floating tolerances. |
| Sylvester, syzygetic relations and resultants | The coefficient matrix of the two-polynomial elimination map is constructed explicitly and its determinant is evaluated by Bareiss. Constant-polynomial conventions bypass artificial zero-dimensional determinants. |
| Tinney and Walker, optimally ordered sparse factorization | Symbolic fill is audited separately from numeric fraction-free updates, with certified-zero cancellation and conservative unknown entries. A retained opt-in symmetric minimum-degree solver now records both permutations, performs exact sparse Bareiss elimination in the reordered system, restores source variable order, and exactly replays the source system. The authored-order API remains unchanged because already-good orderings are faster without the extra analysis. |
| Yap, exact geometric computation | Exact expressions, prepared object facts, certified signs, algebraic intervals, and named lossy adapters enforce the construction/proof boundary throughout the crate. Every retained optimization reuses exact evidence or changes only the construction schedule; none changes a branch criterion. |

## Retained measurements

The timings below are paired Criterion release runs on the same machine. They
are regression evidence for these workloads, not portable absolute claims.

| Workload | Before | After | Change |
| --- | ---: | ---: | ---: |
| Dense exact Bareiss solve, 2 x 2 | 2.051 us | 1.546 us | 24.6% faster |
| Two dense exact right-hand sides, 2 x 2 | 2.562 us sequential | 2.228 us shared | 13.0% faster |
| Dense exact Bareiss solve, tridiagonal 8 x 8 | 245.34 us | 35.29 us | 85.6% faster |
| Pattern-preserving sparse Bareiss solve, 3 x 3 | 4.325 us | 3.768 us | 12.9% faster |
| Symbolic cyclic sparse row swaps, 64 x 64 | 139.99 us | 73.31 us | 47.6% faster |
| Degree-64 polynomial versus constant resultant | 6.397 us | 1.710 us | 73.3% faster |
| Sparse arrowhead solve, 32 x 32 | 4.901 ms authored | 0.790 ms minimum degree | 83.9% faster |
| Sparse tridiagonal solve, 32 x 32 | 315.46 us authored | 388.12 us minimum degree | 23.0% slower |
| Exact roots of `x^2 - 2`, Hypersolve versus CGAL 6.0.3 | 428.90 ns CGAL median | 270.97 ns Hypersolve estimate | 36.8% faster |
| Hypercurve all-family exact Boolean instructions | 320,660,631 | 189,533,986 | 40.9% fewer |

The exact-quadratic competitor row compares Hypersolve's public prepared-row
solver with CGAL's exact `Gmpq`
[`compute_roots_of_2`](https://doc.cgal.org/latest/Number_types/group__nt__ralgebraic.html)
API. Both construct both irrational roots of `x^2 - 2`; the Hypersolve result
additionally retains the source constraint index and symbol. Run the Criterion
`competitor_exact_quadratic_roots/hypersolve` filter and
`bash benches/competitors/run_cgal_quadratic.sh` to reproduce the paired release
measurements. The retained zero-linear fast path constructs
`sqrt(-c/a)` directly, preserves the generic quadratic formula's root order for
both signs of `a`, and falls back to the unchanged discriminant path otherwise.
Open CASCADE's corresponding
[`math_DirectPolynomialRoots`](https://dev.opencascade.org/doc/refman/html/classmath___direct_polynomial_roots.html)
constructor consumes `double`, so it is a lossy proposal baseline rather than a
correctness-equivalent competitor for this exact row.

The sparse-solve result has two independently measured increments. Reusing the
certified symbolic rows for numeric elimination improved 4.325 us to 4.115 us
(4.9%); reusing their exactly accumulated form for final proof replay improved
4.115 us to 3.768 us (8.0% more). A unit test with duplicate and cancelling
terms proves that the reused replay report equals a fresh replay of the source
terms.

The multi-right-hand-side solve performs one certified fraction-free matrix
elimination while carrying every augmented column through the same row
operations. Each result still retains its own Cramer numerators and exact
residual replay. The paired Criterion comparison includes all returned evidence
on both paths; the shared API reduces the 2-by-2 two-coordinate solve from
2.562 us to 2.228 us (13.0%). Hypercurve's complete three-point NURBS
interpolation benefits more because it replaces a determinant-per-coordinate
Cramer construction: its five-run median falls from 19.276 us to 13.098 us
(32.1%).

Rational algebraic images now clear the source polynomial to primitive integer
coefficients and clear the numerator/denominator pair with one shared scale
before resultant construction. Scaling either Sylvester input only scales its
determinant by a nonzero constant, so the represented image roots and their
isolating interval are unchanged. The original rational coefficients still
own domain evaluation and endpoint image bounds. Bareiss elimination then
uses a checked exactly-divisible integer quotient in every dense, multi-RHS,
and sparse recurrence, falling back to general `Real` division whenever the
integer preconditions do not hold.

On Hypercurve's one-cell all-family exact Boolean sentinel, denominator
clearing alone reduced instructions from 320,660,631 to 229,530,874; direct
integer Bareiss division reduced them to 192,745,230, and primitive content
removal reached 189,533,986. Five ordinary runs had an 18.154 ms complete
median and 9.016 ms pair-preparation median, versus 26.628 and 16.830 ms before
the cross-crate change. All runs retained 9 candidate pairs, 48 fragments,
2 point classifications, 4 decided operations, and checksum 6.

Sampled polynomial, rational, and binary algebraic resultants now normalize
every defining polynomial to primitive integers before elimination. Polynomial
images include the symbolic `y` coefficient in their one shared scale, so
sampling still represents exactly `q(x) - y`; rational maps continue to scale
their numerator and denominator together. The resulting integer samples use
forward-difference Newton interpolation multiplied by the common `n!` scale,
avoiding per-basis rational division before one final primitive-content
normalization. This changes only a defining polynomial's nonzero global scale.

On the same Hypercurve sentinel, primitive polynomial-image relations reduced
instructions from 189,533,986 to 176,631,590. Integer-scaled interpolation
shared by polynomial and rational images reached 165,927,095, a combined 12.5%
reduction. Binary independent-root arithmetic uses the same path and has
fractionally scaled source-polynomial regression coverage. The downstream
workload retained all 9 candidate pairs, 48 fragments, 2 point
classifications, 4 decided operations, and checksum 6. The complete
all-feature test suite, strict all-target Clippy, and warning-denied rustdoc
passed.

Bareiss updates on primitive integer matrices now fuse the full
`(pivot * value - eliminand * pivot_value) / previous_pivot` recurrence through
Hyperreal's checked integer cross-difference quotient. This avoids allocating
and caching two intermediate rational products and their difference before
the already guaranteed fraction-free division. Noninteger inputs or a failed
divisibility check take the unchanged general `Real` path; a fractional-matrix
regression exercises that fallback.

On the Hypercurve sentinel, the fused recurrence reduced ten-run instructions
from 151,620,313 to 131,393,603 (13.3%). Eleven ordinary runs had a 12.985 ms
complete median and a 7.844 ms pair-preparation median, versus 14.994 and
8.682 ms. Exact topology remained 9 candidate pairs, 48 fragments, 2 point
classifications, 4 decided operations, and checksum 6.
The complete all-feature unit, property, smoke, and doctest suites, strict
all-target Clippy, and warning-denied rustdoc passed.

Algebraic polynomial and rational images now reuse the direct Mobius
substitution for affine and linear-fractional maps. General maps build the
multiplication matrices of their numerator and denominator once in
`Q[x] / (P)`, clear both matrices with one shared exact scale, and sample the
relation norm with `deg(P)`-dimensional Bareiss determinants. The norm differs
from the corresponding Sylvester resultant only by a nonzero source-leading
coefficient power shared by every image sample, so primitive interpolation
retains the same defining roots. Unsupported exact inputs keep the former
Sylvester construction as a fallback.

The generated regression compares quotient-ring and Sylvester samples across
small cubic sources and quadratic-over-linear maps, including nonmonic source
evidence. A separate rational-image regression cancels the relation's leading
coefficient at an interpolation sample and verifies that the shared norm scale
still produces the expected primitive polynomial. On Hypercurve's
quartic-source sentinel, the hot determinant dimension falls from six or seven
to four. The ten-run instruction median
fell from 85,201,993 to 79,151,572 (7.1%), 75.3% below the original
320,660,631 baseline. Eleven ordinary runs had an 8.625 ms complete median,
a 6.272 ms preparation median, and a 0.393 ms exact-polyline projection
median. Every run retained 9 candidate pairs, 48 fragments, 2 point
classifications, 4 decided operations, no blockers, and checksum 6. Heaptrack
fell from 133,767 to 119,861 allocations and from 9,677 to 7,949 temporary
allocations; measured peak heap moved from 1.92 to 1.97 MiB.

The complete Hypersolve and downstream Hypercurve all-feature and
no-default-feature suites, formatting, warning-denied Clippy and rustdoc, and
release WASM library builds passed. The requested downstream AddressSanitizer
region-Boolean fuzz replay completed at 2,512 executions with 5,897 coverage
points and 19,158 feature edges; LeakSanitizer alone remained disabled under
ptrace.

The quotient basis now uses a fraction-free pseudo-reduction when the source
and map coefficients are already primitive integers. Every column is scaled
by the same fixed power of the source leading coefficient, so numerator and
denominator multiplication matrices retain one common nonzero scale.
Noninteger callers remain unsupported by this private fast path and continue
through the retained Sylvester fallback. The nonmonic and interpolation-degree
cancellation regressions exercise the scale invariant.

On the same Hypercurve sentinel, the ten-run instruction median fell from
79,151,572 to 78,335,067 (1.0%), 75.6% below the original baseline. Eleven
ordinary runs had an 8.458 ms complete median, a 6.073 ms preparation median,
and a 0.430 ms exact-polyline projection median. Heaptrack fell from 119,861
to 116,469 allocations and from 7,949 to 6,834 temporary allocations while
peak heap remained 1.97 MiB. Exact topology and checksum were unchanged.

The complete Hypersolve and downstream Hypercurve feature matrices,
warning-denied Clippy and rustdoc, formatting, and release WASM library builds
passed. The requested AddressSanitizer region-Boolean fuzz replay completed
all 2,509 executions at 5,895 coverage points and 19,157 feature edges;
LeakSanitizer alone remained disabled under ptrace.

Exact-rational Mobius polynomial images now evaluate the homogeneous
substitution by Horner recurrence. For inverse linear forms
`A(y) = d*y - b` and `B(y) = a - c*y`, the recurrence starts with the leading
source coefficient and repeatedly forms `A*H + p_k*B^j`. It tracks `B^j`
alongside `H`, so it constructs exactly the same
`B(y)^n P(A(y) / B(y))` as the retained sum of independently expanded powers.
Both multiplications are by a linear polynomial and use a dedicated
two-diagonal convolution. The fast path is guarded by exact-rational source
and transform coefficients; every other `Real` representation keeps the
former power-sum construction.

A fixed degree-five regression and generated exact-rational polynomials of
degrees zero through five compare the Horner result directly with the retained
power-sum result. On Hypercurve's same all-family exact Boolean sentinel, the
ten-run instruction median fell from 78,335,067 to 77,532,932 (1.0%), 75.8%
below the original 320,660,631 baseline. Specializing the linear convolution
accounted for a further 0.10% reduction from the generic Horner implementation.
Eleven ordinary runs had an 8.593 ms complete median, a 6.437 ms preparation
median, and a 0.409 ms exact-polyline projection median. Heaptrack recorded
115,778 allocations, 6,834 temporary allocations, 1.96 MiB peak heap, and
13.01 MiB peak RSS. Exact topology remained 9 candidate pairs, 48 fragments,
2 classifications, 4 decided operations, no blockers, and checksum 6.

The complete Hypersolve and downstream Hypercurve all-feature and
no-default-feature suites, formatting, warning-denied all-target Clippy and
rustdoc, and release WASM library builds passed. The requested downstream
AddressSanitizer region-Boolean fuzz replay completed all 2,509 executions at
5,895 coverage points and 19,165 feature edges; LeakSanitizer alone remained
disabled under ptrace.

Hyperreal's binary linear cache now requires arithmetic observation rather
than treating an ownership clone as proof that an exact sum or difference will
recur. Hypersolve's elimination and image coefficients are frequently cloned
into immutable matrices and polynomials but paired only once, so this avoids
speculative cache boxes without changing any exact operation or branch.
Existing product/linear evidence still admits immediately, and the known-repeat
self-dot path primes its sum intermediates explicitly.

On the same Hypercurve sentinel, the ten-run instruction median fell from
77,532,932 to 76,301,712 (1.6%), 76.2% below the original baseline.
Heaptrack allocations fell from 115,778 to 114,193, `retain_linear`-descended
allocations from 7,971 to 3,718, and peak heap from 1.96 to 1.53 MiB.
Temporary allocations measured 6,833. Eleven ordinary runs had an 8.840 ms
complete median, a 6.630 ms preparation median, and a 0.376 ms exact-polyline
projection median, with identical topology and checksum.

The complete Hyperreal, Hypersolve, and downstream Hypercurve feature
matrices, formatting, warning-denied all-target Clippy and rustdoc, and release
WASM library builds passed. The requested AddressSanitizer region-Boolean fuzz
replay completed all 2,509 executions at 5,903 coverage points and 19,183
feature edges; LeakSanitizer alone remained disabled under ptrace.

The quotient-ring image path now evaluates its already-flat exact-integer
multiplication matrices with a private flat Bareiss kernel. This avoids
rebuilding nested `Real` rows and constructing the public determinant report
for each interpolation sample. The recurrence still uses Hyperreal's checked
exact integer cross-difference quotient; a failed shape check or exact division
returns `None` and preserves the established Sylvester-resultant fallback.
Fixed zero-, one-, two-, and three-dimensional cases plus generated four-by-four
integer matrices compare the private result with the public report-bearing
determinant, while the existing generated quotient-ring/Sylvester comparison
continues to cover the caller.

On the same Hypercurve sentinel, the ten-run instruction median fell from
76,301,712 to 74,732,427 (2.1%), 76.7% below the original baseline. Heaptrack
allocations fell from 114,193 to 112,178; temporary allocations remained
6,833, peak heap remained 1.53 MiB, and peak RSS was 12.51 MiB. Eleven ordinary
runs had an 8.004 ms complete median, a 5.749 ms preparation median, and a
0.379 ms exact-polyline projection median. Exact topology remained 9 candidate
pairs, 48 fragments, 2 classifications, 4 decided operations, no blockers,
and checksum 6.

The complete Hypersolve and downstream Hypercurve all-feature and
no-default-feature suites, formatting, warning-denied all-target Clippy and
rustdoc, and release WASM library builds passed. The requested AddressSanitizer
region-Boolean fuzz replay completed all 2,509 executions at 5,890 coverage
points and 19,156 feature edges; LeakSanitizer alone remained disabled under
ptrace.

Flat quotient-ring interpolation now forms every sampled matrix entry with
Hyperreal's checked exact integer scaled difference. The new primitive computes
`N - y*D` directly on integer magnitudes instead of materializing a general
rational product and subtraction. Any unexpected noninteger input returns
`None`, preserving this caller's established Sylvester-resultant fallback.
Exhaustive small signed, fractional-rejection, and wide-integer scalar tests
supplement the fixed and generated determinant/resultant comparisons.

Matched fresh 192-bit Hyperreal Criterion sentinels measured the composed
operation at 310.13 ns and the fused operation at 102.50 ns, a 67.0% reduction.
On the same Hypercurve sentinel, the ten-run instruction median fell from
74,732,427 to 72,782,675 (2.6%), 77.3% below the original baseline. Heaptrack
allocations fell from 112,178 to 108,842 and temporary allocations from 6,833
to 6,236; peak heap remained 1.53 MiB and peak RSS was 12.55 MiB. Eleven
ordinary runs had a 7.672 ms complete median, a 5.501 ms preparation median,
and a 0.339 ms exact-polyline projection median, with identical topology and
checksum.

The complete Hyperreal, Hypersolve, and downstream Hypercurve all-feature and
no-default-feature suites, formatting, warning-denied all-target Clippy and
rustdoc, and release WASM library builds passed. The requested AddressSanitizer
region-Boolean fuzz replay completed all 2,509 executions at 5,895 coverage
points and 19,144 feature edges; LeakSanitizer alone remained disabled under
ptrace.

Hyperreal's checked integer cross-difference quotient now recognizes a divisor
with magnitude one after all exact-integer and nonzero guards. Applying only
the divisor sign avoids big-integer division in the first Bareiss stage.
A matched fresh 192-bit Criterion sentinel measured 625.27 ns for the composed
multiply/subtract/divide and 190.15 ns for the fused unit-divisor path, a 69.6%
reduction.

On the same downstream sentinel, the ten-run instruction median fell from
72,782,675 to 72,479,577 (0.4%), 77.4% below the original baseline. Heaptrack
allocations fell from 108,842 to 107,461; temporary allocations remained
6,236, peak heap remained 1.53 MiB, and peak RSS was 12.65 MiB. Eleven ordinary
runs had a 7.552 ms complete median, a 5.446 ms preparation median, and a
0.353 ms exact-polyline projection median, with unchanged topology and
checksum.

The complete Hyperreal, Hypersolve, and downstream Hypercurve all-feature and
no-default-feature suites, formatting, warning-denied all-target Clippy and
rustdoc, and release WASM library builds passed. The requested AddressSanitizer
region-Boolean replay completed its 2,509-run budget after 2,513 executions at
5,900 coverage points and 19,165 feature edges; LeakSanitizer alone remained
disabled under ptrace.

Quotient-basis pseudo-reduction now evaluates each affected coefficient as one
checked exact integer cross difference, `leading*value - source*eliminand`.
Only the untouched low-degree prefix is scaled separately, and a unit source
leading coefficient leaves that prefix unchanged. Initial shifted relation
coefficients are cloned directly instead of being added to zero. Any failed
integer check still returns `None` from the quotient-ring path and preserves
the established Sylvester-resultant fallback; the generated
quotient-ring/Sylvester property covers the complete schedule.

On the same downstream sentinel, the ten-run instruction median fell from
72,479,577 to 71,182,596 (1.8%), 77.8% below the original baseline. Heaptrack
allocations fell from 107,461 to 105,140; temporary allocations rose from
6,236 to 6,409, peak heap remained 1.53 MiB, and peak RSS fell from 12.65 to
12.37 MiB. Eleven ordinary runs had a 7.446 ms complete median, a 5.430 ms
preparation median, and a 0.343 ms exact-polyline projection median, with
unchanged topology and checksum.

The complete Hypersolve and downstream Hypercurve all-feature and
no-default-feature suites, formatting, warning-denied all-target Clippy and
rustdoc, and release WASM library builds passed. The requested AddressSanitizer
region-Boolean replay completed all 2,509 executions at 5,893 coverage points
and 19,125 feature edges; LeakSanitizer alone remained disabled under ptrace.

Quotient-ring multiplication now omits the highest coefficient update in each
pseudo-reduction step. The chosen quotient coefficient makes that slot exactly
`leading*eliminand - leading*eliminand`, so assigning exact zero avoids a
redundant big-integer multiply, subtraction, and checked division. All lower
coefficients still use the checked integer cross-difference primitive, and any
failure there still selects the retained Sylvester fallback. Fixed and
generated quotient-ring/Sylvester comparisons cover the complete schedule.

Together with downstream Hypercurve's borrowed polynomial-remainder divisor,
the one-cell all-family exact Boolean sentinel's ten-run instruction median
fell from 64,966,544 to 64,678,125 (0.44%), 79.8% below the original
320,660,631 baseline. Heaptrack allocations fell from 97,195 to 96,817 and
temporary allocations from 6,331 to 6,165; peak heap remained 1.49 MiB and
peak RSS remained 12.39 MiB. Eleven ordinary runs had a 6.967 ms complete
median, a 4.841 ms preparation median, and a 0.337 ms exact-polyline projection
median, with unchanged topology and checksum.

The complete Hypersolve and downstream Hypercurve all-feature and
no-default-feature suites, formatting, warning-denied all-target Clippy,
all-feature and no-default-feature rustdoc, and default and no-default release
WASM library builds passed. The AddressSanitizer region-Boolean replay
completed all 2,509 executions at 5,892 coverage points and 19,170 feature
edges with no finding; LeakSanitizer alone remained disabled under ptrace.

Quotient-ring resultant construction now crosses into `BigInt` once, before
pseudo-reduction, sampling, and Bareiss elimination. The two multiplication
matrices and one sampled determinant buffer are reused directly; each quotient
column also moves its reduced coefficients out of one reusable product buffer.
This removes per-sample rational wrappers and vectors while retaining exact
Bareiss divisibility checks and the Sylvester-resultant fallback boundary.

Integer Newton interpolation likewise keeps forward differences, falling
factorials, factorial scaling, coefficient accumulation, and final content
removal in `BigInt`. Differences are updated in place, the falling-factorial
basis grows in place, and callers no longer repeat primitive rational
normalization. Arbitrary-size factorial scaling also removes the former `i64`
degree ceiling; a degree-22 constant interpolation regression exercises that
expanded exact range. Fixed and generated Bareiss, quotient/Sylvester,
polynomial-image, rational-image, and binary algebraic-image tests cover the
complete path.

On the downstream one-cell all-family exact Boolean sentinel, the ten-run
instruction median fell from 61,647,633 to 57,499,110 (6.73%), 82.07% below
the original 320,660,631 baseline. Heaptrack allocation events fell from
98,024 to 87,084; temporary events rose from 6,461 to 7,847 because direct
integer products are released promptly, peak heap remained 1.41 MiB, and peak
RSS measured 12.51 MiB.

Eleven ordinary runs had a 5.736 ms complete median, a 3.794 ms preparation
median, and a 0.331 ms exact-polyline projection median, with unchanged
topology and checksum. The complete Hypersolve and downstream Hypercurve
all-feature and no-default-feature suites, formatting, warning-denied
all-target Clippy and rustdoc, and supported default and no-default release
WASM library builds passed. The AddressSanitizer region-Boolean replay
completed all 2,509 executions at 5,899 coverage points and 19,166 feature
edges with no finding; LeakSanitizer alone remained disabled under ptrace.

## Dispatch-path coverage

Run `cargo bench --bench dispatch_trace --features dispatch-trace` to regenerate
`dispatch_trace.md`. The diagnostic harness pairs major families from the timed
`certification` benchmark with the shared `hyperreal` trace recorder. It covers
sketch lowering and preflight, prepared candidate/batch/active-set work,
direct and fraction-free linear algebra, resultants and curve substitution,
root isolation and interval proof, affine Krawczyk proof, domain proof, and the
named lossy-proposal/exact-replay boundary. Every row contains a Hypersolve
workload marker, and the run fails unless that row also records dependency
dispatch or rational reducer work.

Trace builds are diagnostic and must not be used for timing comparisons. The
feature forwards trace instrumentation into `hyperreal`, `hyperlattice`, and
`hyperlimit`; the ordinary default build retains no trace hooks.

## Why the retained changes preserve proof

The augmented dense solve applies the same Bareiss recurrence to `A` and `b`
in one pass, then reconstructs the Cramer numerators as `det(A) * x`. Exact
residual replay is unchanged, pivot swaps are tested, generated nonsingular
systems recover their authored solutions, and the former Cramer path remains a
fallback.

The multi-right-hand-side sibling applies those identical certified row
operations to every `b` column. Pivot-row swaps are replayed across all columns,
each solution is independently checked against its original `A*x-b`, and the
single-right-hand-side reports are used as exact test oracles for solutions,
numerators, and replay rows. The same oracle is exercised by a dedicated
generated triangular-system fuzz target (1,000 retained executions in this
batch), and the shared solve is part of the exact-linear dispatch trace family.

The sparse solver's symbolic report already contains every duplicate-
accumulated exact entry and its certified zero/nonzero status. Feeding those
entries into numeric elimination and replay removes redundant maps and sign
queries without omitting any source coefficient. Moving a symbolic row via the
ordered set's active range is exactly the same permutation as probing every
possible column.

Binary powering changes only the multiplication schedule for the exact scalar
identity `c^n`; a degree-64 regression compares it with a linear exact product.

The minimum-degree path symmetrizes only the certified-nonzero sparsity graph,
repeatedly eliminates the active minimum-degree source vertex with stable index
ties, and models symbolic fill by connecting its active neighbors. It applies
the resulting permutation to rows, columns, and the right-hand side together.
The returned report carries both permutation directions, the complete permuted
solve report, the source-order solution, and a fresh exact residual replay of
the original terms. The arrowhead sentinel exposes the intended high-fill
crossover; the tridiagonal sentinel documents why this remains opt-in.

## Architecture boundaries

Tinney--Walker style fill reduction is now implemented with explicit row and
column permutation evidence, solution unpermutation, pivot provenance, source
replay, and contrasting sparse benchmark families. It is not automatically
inserted into the authored-order API because the measured tridiagonal case
shows that analysis and replay overhead can dominate when the input order is
already good.

The numerical-optimization references offer richer damping updates, line
searches, trust-region acceptance policies, full SQP, and sparse numerical
factorizations. Those are intentionally not smuggled into the exact layer.
They should arrive as named proposal adapters with their own diagnostics and
benchmarks, while Krawczyk, alpha, Sturm, and exact residual replay remain the
proof-producing surfaces.
