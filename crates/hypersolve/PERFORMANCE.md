# Performance and Reference Audit

This document records how every source in the README reference list maps to
`hypersolve`, which changes were supported by measurements, and which ideas are
deliberately bounded or architecture-inapplicable. Solver proposals may cross a named lossy
boundary, but no optimization may turn such a proposal into proof.

## Reference-by-reference findings

| Reference | Applied finding and disposition |
| --- | --- |
| Bareiss, integer-preserving Gaussian elimination | The exact dense and sparse direct solvers use the fraction-free recurrence and certified pivots. The dense solver formerly evaluated one Bareiss determinant for the matrix and one per Cramer numerator. It now eliminates the augmented system once, retains the same determinant/numerator/replay report, and falls back to the former Cramer construction if an augmented exact division is not representable. This is the largest retained optimization below. |
| Bouma et al., geometric constraint solving | Graph reduction and directed algebraic decomposition correspond to analyzed dependency facts, equality-substitution classes, affine row elimination, solver blocks, and sketch lowering. These exact/direct passes remain ahead of the nonlinear proposal loop; solution choice and interactive dragging are reported rather than hidden. |
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
| Smale, one-point Newton estimates | Univariate quadratic forms expose an exact conservative alpha certificate, using the rational threshold `1/8` to avoid adding a radical solely for the bound. Exact roots, multiple roots, zero derivatives, failed bounds, and undecided comparisons are distinct statuses. |
| Sturm, numerical equations | Exact Sturm sequences count distinct roots over rational intervals and guide isolation/refinement. Endpoint roots and multiplicities are handled through square-free and replay evidence instead of floating tolerances. |
| Sylvester, syzygetic relations and resultants | The coefficient matrix of the two-polynomial elimination map is constructed explicitly and its determinant is evaluated by Bareiss. Constant-polynomial conventions bypass artificial zero-dimensional determinants. |
| Tinney and Walker, optimally ordered sparse factorization | Symbolic fill is audited separately from numeric fraction-free updates, with certified-zero cancellation and conservative unknown entries. A retained opt-in symmetric minimum-degree solver now records both permutations, performs exact sparse Bareiss elimination in the reordered system, restores source variable order, and exactly replays the source system. The authored-order API remains unchanged because already-good orderings are faster without the extra analysis. |
| Yap, exact geometric computation | Exact expressions, analyzed object facts, certified signs, algebraic intervals, and named lossy adapters enforce the construction/proof boundary throughout the crate. Every retained optimization reuses exact evidence or changes only the construction schedule; none changes a branch criterion. |

## Exact Bernstein subdivision gate

Recursive Bernstein subdivision now carries each node's exact Bernstein
coefficients and constructs its two midpoint children with de Casteljau's
recurrence. Previously every child re-extracted the source expression,
reevaluated both endpoints, and reconverted the original power polynomial;
the initial interval was also converted twice. The retained schedule shares the
exact de Casteljau terminal coefficient as the midpoint witness and recomputes
only the two child sign-variation counts.

This also closes an exactness hole in the former terminal rule. Variation one
describes the open interval after zero endpoint coefficients are skipped, so an
interval with both an endpoint root and variation one can still contain one
interior root. The old path emitted the endpoint and returned. The new path
continues exact subdivision until the evidence is separated or reports the
configured depth limit. The `x(x-2)` sentinel on `[0,4]` now retains both exact
roots, and the cubic `(x-1)(x-2)(x-3)` sentinel retains all three rather than
only its first midpoint witness. A generated cubic-coefficient property checks
both de Casteljau children against independent exact power-to-Bernstein
conversion.

The existing 50-sample Criterion benchmark over 16 repeated-quadratic rows
improved from 237.99--240.53 us to 56.728--57.347 us, a 76.0% midpoint
reduction. The proof surface is stronger: no tolerance or approximate branch
was introduced, and unresolved endpoint separation remains an explicit
`DepthLimit` report.

## Exact dyadic root-bound gate

Whole-line Sturm isolation now starts from a power-of-two Fujiwara bound
instead of `1 + max |a_i/a_n|`. For each lower coefficient, exact rational
numerator/denominator product bit lengths round
`|a_i/a_n|^(1/(n-i))` upward to a power of two. Doubling the maximum is a
strict root bound by the usual geometric-series argument, while its dyadic
endpoint stays on Hyperreal's inexpensive exact path. No algebraic radical or
floating approximation enters the proof.

This tighter bound exposed and closed an older endpoint-counting defect. When
several rational roots landed on successive bisection points, a previously
emitted upper-endpoint root could be counted again and an unrelated left root
discarded. Recursive nodes now count only roots in their open interval,
separate an exact endpoint before accepting a neighboring isolator, and emit
intervals in root order. The `(x-1)(x-2)(x-3)(x-4)` repeated-factor sentinel
now returns the four exact points rather than losing `x=1` to a duplicate
endpoint interval.

Five serialized release runs used independently regenerated exact degree-80
Chebyshev, Hermite, and Wilkinson polynomials plus the exact degree-81
Mignotte specimen from ExactCore's `hermiteLevel` corpus. Medians include
square-free reduction, Sturm construction, and isolation:

| Polynomial | Cauchy bound | Dyadic Fujiwara bound | Reduction |
| --- | ---: | ---: | ---: |
| Chebyshev 80, 80 roots | 260.10 ms | 112.63 ms | 56.7% |
| Hermite 80, 80 roots | 1.5599 s | 101.80 ms | 93.5% |
| Wilkinson 80, 80 exact roots | 3.1479 s | 143.94 ms | 95.4% |
| Mignotte 81, 3 roots | 279.85 ms | 223.22 ms | 20.2% |

On Wilkinson 80, peak RSS fell from 9,144 KiB to 7,436 KiB (18.7%). The
release `rlib` grew by 37,792 bytes (0.30%). Generated rational-root bound
properties, exact midpoint ordering, all nine ExactCore root-family
regressions, default/no-default/all-feature suites, warning-denied Clippy and
rustdoc, and formatting all passed.

## Immediate substitution and workplane APIs

The remaining public `build_` lifecycle names returned completed values with
no deferred execution phase. They are now `equality_substitution_classes`,
`sketch_workplane_frame`, and `sketch_workplane_frame_with_policy`. Algorithms,
reports, and exact acceptance criteria are unchanged.

Serialized 100-sample Criterion measurements found no regression:

| Benchmark | Before | After | Midpoint change |
| --- | ---: | ---: | ---: |
| `build_equality_substitution_classes_exact` | 4.8974--4.9367 us | 4.8147--4.8278 us | -1.92% |
| `sketch_workplane_frame` | 1.2273--1.2406 us | 1.2109--1.2158 us | -1.65% |

The first post-change substitution-class run measured 4.9733--5.0438 us; the
required isolated confirmation returned below baseline and is the accepted
serialized gate above. The historical `build_equality_substitution_classes`
benchmark identifier remains stable for comparison continuity and is not a
public API.

## Immediate residual-form API gate

Affine and quadratic rows are now named as immutable mathematical forms:
`AffineResidual`, `UnivariateQuadraticResidual`, `QuadraticResidual`,
`QuadraticLinearTerm`, and `QuadraticTerm`. Their immediate `from_expr`
constructors replace preparation functions, and a shape error now reports
`ResidualShapeMismatch`.

Serialized 100-sample comparisons found no regression. Constructing a
16-row univariate quadratic problem moved from 46.313 us to 46.586 us
(+0.27%, `p = 0.31`, 95% interval -0.27% to +0.76%). Constructing the
multivariate equivalent improved from 35.860 us to 34.955 us (-2.88%,
`p < 0.05`, 95% interval -3.84% to -2.08%). The parsing and evaluation
kernels are unchanged; the improvement is consistent with code layout. The
renamed retained sentinels subsequently measured 48.200 us and 35.011 us.

## Problem-analysis API gate

The central reusable solver state is now `ProblemAnalysis`, available directly
through `Problem::analyze`. Its public summaries are `ProblemFacts`,
`ConstraintFacts`, `SolverBlock`, and `SolverBlockFacts`; the module is
`analysis`. This retains the one-time dependency, row-form, and Jacobian work
without exposing a preparation lifecycle.

Serialized 100-sample comparisons found no regression. Analysis of the 16-row
univariate quadratic problem improved from 46.159 us to 44.460 us (-3.68%,
`p < 0.05`, 95% interval -4.17% to -3.24%). Exact affine candidate
certification moved from 2.5113 us to 2.5168 us (+0.63%, within Criterion's
noise threshold, 95% interval +0.03% to +1.19%). The report now reserves its
known active-row capacity, and the cross-crate `analyze` accessor is inline;
arithmetic and certification criteria are unchanged.

## Sparse replay API gate

The retained exact matrix is now named for what it is:
`SparseLinearSystem::from_terms` assembles validated row terms, while
`replay_sparse_linear_residuals` and `replay_sparse_linear_residual_batch`
remain immediate one-shot operations. The former preparation constructor and
prepared-batch free function are removed; repeated callers use the system's
`replay_candidate` and `replay_batch` methods.

Serialized 100-sample comparisons found no regression. Replaying a 16-candidate
batch through one assembled system improved from 6.3428 us to 6.0008 us
(-5.76%, `p < 0.05`, 95% interval -6.47% to -5.15%). Immediate one-candidate
assembly and replay moved from 658.98 ns to 653.20 ns (-0.99%, within
Criterion's noise threshold, 95% interval -1.40% to -0.60%). The retained
benchmark is now `sparse_linear_batch_replay`.

## Rational-image API gate

`transform_algebraic_root_rational_images` immediately transforms several
numerators over one denominator while sharing its exact evaluation and source
polynomial internally. `AlgebraicRootRationalMap` remains a retained
mathematical map because its exact elimination is reused across independently
discovered roots of the same source polynomial. The singular
`transform_algebraic_root_rational_image` functions remain the ordinary
one-expression surface.

Three serialized downstream runs gated removal of the former public
shared-denominator context. HyperCurve's 20,000-iteration rational Bezier
algebraic point-and-tangent image sentinel moved from a 6.132 us median to
6.123 us (-0.15%), with all 40,000 expected coordinate images transformed.

Serialized 100-sample comparisons found no regression. The small immediate
rational image moved from 5.0899 us to 5.0823 us (Criterion change -0.32%,
`p = 0.25`, 95% interval -0.85% to +0.22%). The degree-12 cubic-map sentinel
moved from 1.7491 ms to 1.7548 ms (Criterion change +0.19%, `p = 0.55`, 95%
interval -0.43% to +0.77%). Retaining the private direct-map builder as a free
function preserved the former code layout; an associated-constructor
experiment regressed the small transform and was rejected.

## Rational-map cache-state API gate

`AlgebraicRootRationalMap` now exposes only the immediate `transform` and
`transform_in_interval` operations, not whether its private resultant
`OnceLock` has been populated. Module tests still inspect that private state to
prove reuse and mismatched-source fallback. A dedicated retained-map Criterion
case now keeps the completed operation performance-gated: removing
`is_resultant_cached` improved a warm nonlinear transform from
3.3783–3.4054 us to 3.1845–3.2242 us.

## Policy-certified local image content

Primitive local polynomial-image normalization now keeps its structural-zero
fast path, then asks the owning predicate policy about an unresolved remainder
coefficient before rejecting exact divisibility. This closes a completeness
gap where polynomial GCD and division had already certified a common factor,
but the final structural-only check returned `Undecided` for the same bounded
algebraic cancellation. A nested-radical regression exercises both the private
content step and the public image projection under strict and approximate
policies; an unsupported trigonometric identity still returns `Undecided`.

The permanent `algebraic_image_policy_zero_content` wall-clock row completes
the public projection in 8.99 us per operation over 5,000 iterations. The old
structurally-zero remainder path remains short-circuited before policy
refinement. Alternating release runs of the eight pre-existing algebraic-fiber
rows stayed within ordinary run-to-run variation, with no consistent slowdown.
A warmed 32-iteration counting-allocator probe measured 133 allocations and
9,152 allocated bytes per completed projection, with 3,584 peak live bytes and
zero end-of-epoch live-byte delta. The production edit is a one-line net source
increase; the default release rlib grew from 12,813,840 to 12,823,298 bytes
(9,458 bytes, 0.074%).

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

The exact-quadratic competitor row compares Hypersolve's public row-form
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

### Shared-denominator rational-image reuse

Projective point and derivative coordinates transform several rational
expressions at the same represented root with one homogeneous denominator.
Independent calls formerly repeated the denominator polynomial evaluation and
the source-polynomial rational-to-primitive-integer conversion before building
each coordinate's distinct exact resultant.

The immediate `transform_algebraic_root_rational_images` operation evaluates
and retains the denominator certificate only for the duration of one completed
batch. It preserves complete per-coordinate reports while lazily converting
and sharing the common source polynomial only when the direct resultant route
reaches that stage. Numerator evaluation, rational-map monotonicity proof,
coordinate-specific resultant, isolating interval, fallback images,
validation, and uncertainty statuses remain unchanged. A batch-versus-
independent report equality regression covers two nonlinear numerators over
one denominator.

On the downstream one-cell all-family exact Boolean sentinel, reuse by
Hypercurve's rational point and derivative images reduced the rounded ten-run
instruction median from 32,588,978 to 32,508,278 (0.25%), 89.86% below the
original 320,660,631 baseline. Heaptrack allocation events fell from 47,057 to
46,869 and temporary events from 2,960 to 2,930; peak heap remained 1.13 MiB
and peak RSS measured 11.17 MiB. Every measured run retained 9 candidate
pairs, 48 fragments, 2 classifications, 4 decided operations, no blockers,
and checksum 6.

The complete Hypersolve and downstream Hypercurve all-feature and
no-default-feature suites, formatting, warning-denied all-target Clippy and
rustdoc, and supported default/no-default release WASM library builds passed.
The requested AddressSanitizer region-Boolean replay completed with libFuzzer
reporting 2,515 executions at 5,893 coverage points and 19,142 feature edges
with no finding; LeakSanitizer remained disabled under ptrace.

#### Retention gate

A July 2026 experiment replaced the shared-denominator context with an
immediate two-numerator transform
that preserved shared denominator evaluation, lazy source-polynomial reuse,
complete coordinate reports, and the first-coordinate short circuit. The best
return-by-value form increased the complete algebraic-parameter benchmark from
9,570,229,121 to 9,586,812,181 Callgrind instructions (0.17%); report-pair
assembly alone executed 7,880,394 `memcpy` instructions. An output-slot form
instead reached 9,598,735,012 instructions and increased the loadable artifact
by 4,092 bytes. Both forms were reverted, so the prepared carrier remains part
of the measured performance contract.

Prepared-surface removal in Hyperreal, Hyperlattice, Hyperlimit, or Hypersolve
uses a two-tier acceptance gate. The candidate must first beat or equal every
immediately affected public API on exact-result regressions, executed release
benchmarks, deterministic instructions, allocations, binary size, and
callgraph depth. Only a candidate that clears that tier proceeds to correctness
and representative executed performance tests for every `hyper*` crate in the
workspace, followed by relevant sanitizer-backed fuzz targets. Compile-only
bench checks do not satisfy the performance tier.

### Rational maps across source roots

Several represented roots of one polynomial can be transported through the
same rational map. The defining polynomial and rational-map coefficients
determine the direct resultant; the selected isolating interval determines
which real root of that resultant is retained. Independent transformations
formerly rebuilt the identical elimination for every source root.

`AlgebraicRootRationalMap` retains the primitive source polynomial
and direct resultant lazily across those transformations. Each call still
performs its own numerator and denominator evaluations, denominator-domain
proof, local monotonicity proof, rational image interval, target rejection,
root selection, and exact validation. A mismatched source polynomial takes the
ordinary independent path. A two-root regression compares complete prepared
and independent reports while proving that the nonlinear map's elimination is
cached after the first root.

On the downstream one-cell all-family exact Boolean sentinel, reuse by
Hypercurve's implicit-conic contact transport reduced the rounded ten-run
instruction median from 32,508,278 to 32,120,772 (1.19%), 89.98% below the
original 320,660,631 baseline. Heaptrack allocation events fell from 46,869 to
46,054; recorder-level temporary allocation events fell from 2,930 to 2,714,
while the postprocessor's broader temporary count was 2,962. Peak heap
remained 1.13 MiB and peak RSS measured 11.25 MiB. Every measured run retained
9 candidate pairs, 48 fragments, 2 classifications, 4 decided operations, no
blockers, and checksum 6.

The complete Hypersolve and downstream Hypercurve all-feature and
no-default-feature suites, formatting, warning-denied all-target Clippy and
rustdoc, and supported default/no-default release WASM library builds passed.
The requested AddressSanitizer region-Boolean replay completed with libFuzzer
reporting 2,510 executions at 5,900 coverage points and 19,164 feature edges
with no finding; LeakSanitizer remained disabled under ptrace.

### Retained rational-map algebra

Prepared cross-root maps initially retained only the final source resultant.
Every represented root still trimmed and classified the same numerator and
denominator, reconstructed the quotient-rule derivative polynomial, and
cleared the same coefficient denominators before consulting that resultant.
Those values depend only on the prepared map and its fixed predicate policy.

The prepared carrier now retains normalized coefficients and constant-map
classification when first needed. Derivative construction and common integer
scaling each remain separately lazy, so disjoint target rejection, invalid
source evidence, constant maps, and nonmonotone maps preserve their former
short-circuit boundaries. Source-root interval evaluation, derivative interval
evaluation, image bounds, root selection, and validation remain per call.
Regressions compare complete reports across two roots, inspect each retained
stage, and prove that a different but valid source polynomial takes the
independent path without populating the caches.

On the downstream one-cell all-family exact Boolean sentinel, the rounded
ten-run instruction median fell from 32,120,772 to 32,005,223 (0.36%), 90.02%
below the original 320,660,631 baseline. Heaptrack allocation events fell from
46,054 to 45,821; recorder-level temporary events fell from 2,714 to 2,702 and
the postprocessor count fell from 2,962 to 2,950. Peak heap remained 1.13 MiB
and peak RSS fell from 11.25 to 11.15 MiB. Every measured run retained 9
candidate pairs, 48 fragments, 2 classifications, 4 decided operations, no
blockers, and checksum 6.

The complete Hypersolve and downstream Hypercurve all-feature and
no-default-feature suites, formatting, warning-denied all-target Clippy and
rustdoc, and supported default/no-default release WASM library builds passed.
The AddressSanitizer region-Boolean replay completed all 2,509 requested
executions at 5,900 coverage points and 19,225 feature edges with no finding;
LeakSanitizer remained disabled under ptrace.

### Consumed subset-determinant states

The quotient-ring rational-image resultant expands the determinant of a matrix
whose entries are linear polynomials in the image variable. Its subset dynamic
program visits masks in numeric order, so every predecessor of a mask has
already contributed before that mask is processed. The implementation
previously cloned the complete exact coefficient vector at every visited mask
and then never read the stored vector again. It now takes ownership of that
completed state. The determinant recurrence, signs, coefficient products, and
degree bound are unchanged.

Generated quotient-ring resultants still agree with independently evaluated
Sylvester resultants, and the downstream 67-cell all-family Hypercurve workload
still returns all 268 exact Boolean results with 603 candidate pairs, 3,248
fragments, 134 classifications, no blockers, and checksum 6.

On the identical three-cell downstream Callgrind workload, instruction
references fell from 135,526,781 to 134,833,076 (0.51%). Heaptrack allocation
events fell from 197,456 to 195,380 (1.05%) and its postprocessed temporary
count from 24,392 to 24,352; peak heap remained 2.19 MiB. The seven-run
67-cell wall-time median was effectively unchanged at 521.43 milliseconds
versus 521.68 milliseconds before the ownership change. The same-compiler
pathological executable grew by 60 text bytes, shed 64 BSS bytes and four
total loadable bytes, and was 400 bytes smaller on disk.

The certification benchmark now includes the maximum supported degree-12
public immediate rational-image transform with a cubic numerator and quadratic
denominator. The retained subset representation measured 1.741--1.763
milliseconds in the confirming run. Two lower-allocation representations were
rejected at the immediate performance gate. A single flat arena improved the
large case by about 9%, but raised the one-cell downstream exact Boolean
workload by 94,593 Callgrind instructions (0.33%) and added 7,064 text bytes.
A cardinality-layered arena retained a large-case improvement and removed 96
downstream allocations, but added 11,415 instructions to the affected
resultant kernel, 4,761 instructions end to end, and 7,276 text bytes. Both
production changes were reverted; the benchmark remains to stop future
large-input gains from hiding an immediate-API or downstream regression.

### Solver-analysis preparation boundary

`ProblemAnalysis::new` is no longer a second public construction path;
`Problem::analyze()` is the sole source-associated entry. The retained
analysis view itself cannot yet be replaced by per-operation immediate
reconstruction without violating the performance gate: analysis of 16
univariate quadratic rows measured 47.655--47.833 us, while certification
against one retained affine analysis measured 2.636--2.663 us.

Rebuilding structural forms inside every immediate certification would
therefore add roughly 18 times the measured certification cost. Automatic
retention would first require making `Problem` mutation cache-safe; its
variables and constraints remain publicly mutable today. The retained view is
kept as the measured exception rather than hiding that regression behind a
shorter call. A same-tree visibility A/B also confirmed no regression from
hiding the duplicate constructor: restored public construction measured
51.901--52.418 us and the final crate-private build measured
49.478--49.631 us.

### Reuse certified nonzero evidence at exact construction boundaries

Algebraic images, Krawczyk rows, exact linear pivots, tensor/resultant
normalization, and bivariate long division previously made a policy decision
that a denominator was nonzero and then asked ordinary `Real` division to prove
the same fact again. The retained path now separates the common structural
inverse from a cold policy fallback. Ordinary inputs keep their former owned
division route; only `UnknownZero`-style construction failures reuse the
already certified decision through `inverse_ref_assuming_nonzero`.

The adversarial fixture is positive and exactly equal to `2^-3000`, but its
structural zero status and ordinary inverse remain undecided at Hyperreal's
bounded inverse limit. Strict exact-normal policy proves it positive. It now
passes rational evaluation and interval enclosure, binary and Mobius images,
tensor linear-root compaction, quadratic Krawczyk certification, exact pivot
normalization, resultant normalization, and bivariate division. Independent
unsupported transcendental zero controls remain `Undecided` or fail closed.

Pinned Criterion A/B runs shaped the implementation rather than merely
checking it afterward:

| Workload | Retained result versus legacy/control |
| --- | ---: |
| Affine Krawczyk box | -3.31% |
| Univariate quadratic Krawczyk rows | no detectable change (+0.64%) |
| Multivariate quadratic Krawczyk rows | -1.97% |
| Rational-expression evaluation, ordinary algebraic root | -6.01% versus the eager-policy form |
| Mobius transform | -5.87% versus the eager-policy form; within about 1.1% of direct legacy division |
| Rational image | -3.47% versus the eager-policy form and about -1.8% versus legacy |
| Binary divide image | -1.00% versus the eager-policy form, within the noise threshold |

An eager policy reciprocal made the new permanent
`divide_bivariate_polynomial_exact` rational microbenchmark 15.28% slower, so
that form was rejected. The retained implementation executes the original long
division first and enters a cold strict retry only if leading-coefficient
division fails. Its ordinary profile remains 6 allocations and 768 bytes per
operation, exactly matching the direct control; the deep policy retry also uses
6 allocations (528 bytes). The retry adds 8,972 bytes (0.07%) to the default
release rlib, while the complete default rlib after this slice is 12,804,092
bytes, 19,206 bytes smaller than the preceding retained build. Shared test
fixtures remove six duplicate Hypersolve adversarial constructors.

### Complete strict sparse proof boundaries

Sparse symbolic classification, fill pruning, pivot selection, back
substitution, and residual replay formerly stopped after
`Real::certified_sign_until(min_precision)`, even though the materialized dense
solver already continued through Hyperlimit's strict exact predicate cascade.
The retained sparse path now shares a strict-only sign helper after the caller's
fast refinement bound. A value still unknown after that cascade remains an
explicit `UnknownSign`, `UndecidedPivot`, or `UnknownResidual`; no approximate
terminal decision was added.

The exact-normal adversary is mathematically `2^-3000`, exhausts the ordinary
inverse and `-64` sign-refinement bound, and is nevertheless proved positive by
strict algebraic normalization. It now passes the symbolic audit and the 1 by 1
pattern-preserving Bareiss solve. A separately normalized zero residual now
passes sparse replay, while the unsupported trigonometric identity control
still fails closed. Back substitution first preserves ordinary `Real` division
and enters a cold assumed-reciprocal branch only after that division fails; the
first unconditional-reciprocal prototype was removed after regressing ordinary
sparse controls by 3--6%.

Pinned before/after Criterion runs found the final ordinary path within the
noise threshold: 3 by 3 pattern-preserving solve +1.65%, 32 by 32 arrowhead
authored/minimum-degree -0.83%/+0.48%, tridiagonal authored/minimum-degree
-1.12%/+1.09%, and symbolic pattern audit +1.57%. The permanent cold-input
exact-normal benchmark reports a 456.30 us mean (453.03--459.89 us 95%
interval) for strict proof, construction, and replay. A 32-iteration
steady-state allocation probe measured
18 allocations and 3,192 bytes per solve for both the exact-normal case and a
rational 1 by 1 control, with zero live-byte delta. The default release rlib is
12,811,586 bytes, 7,494 bytes (0.06%) above the preceding retained build.

### Complete exact resultant coefficient boundaries

Univariate degree trimming, pseudo-remainder zero detection, and scheduled
resultant signs, plus the typed curve and rank-independent tensor degree,
trimming, zero-polynomial, normalization, and independent-axis checks,
formerly stopped at the caller's scalar-refinement bound. These exact
constructors now preserve that bound as the fast path and enter Hyperlimit's
strict exact predicate cascade only for an unresolved coefficient. A value
still unsupported by that cascade retains the existing
`UndecidedCoefficient` or `UndecidedResultantSign` result; approximation is
never used to choose a degree, discard a coefficient, or declare a resultant
nonzero.

Regressions cover an exact-normal positive value equal to `2^-3000`, a
separately normalized exact zero that is still unknown at precision `-64`, and
an unsupported trigonometric zero identity. The first two now complete
univariate resultants and subresultants, scheduled coprimality, every typed
degree helper, monic trivariate and tensor constraint normalization, tensor
canonicalization, and certified independent-axis removal. The trigonometric
control remains undecided at every corresponding boundary.

Pinned ordinary-path comparisons retained the original direct sign branches.
The univariate resultant, subresultant, and scheduler rows were within the
noise threshold at +1.58%, -1.37%, and +1.70%. The typed trivariate row was
unchanged (+0.65%), and an isolated confirmation put the tensor row at +0.37%
with overlapping confidence intervals. Permanent cold-branch rows measured
92.705 us (92.209--93.260 us 95% interval) for univariate trimming, 460.90 us
(455.85--466.75 us) for constrained trivariate construction, and 202.58 us
(200.54--204.75 us) for constrained tensor construction. Fixture creation is
excluded from those timings.

A warmed 32-iteration allocation probe found zero live-byte deltas throughout.
Rational versus exact-normal paths used 2 versus 358 allocations (144 versus
20,992 bytes) for the univariate fixture, 39 versus 240 (3,048 versus 15,040
bytes) for trivariate construction, and 59 versus 261 (2,352 versus 14,096
bytes) for tensor construction. The complete default release rlib is
12,869,532 bytes, 57,946 bytes (0.45%) above the preceding retained build.

### Canonical exact resultant and projective-image storage

Algebraic-fiber polynomial-image projection formerly removed degree-bearing
input rows and projective output coefficients only when `Real` carried a
structural zero fact. Consequently, a separately normalized exact zero that
was still structurally unknown could leave a spurious image degree and turn a
constructible `z^2 - 2` image into `DegreeLimitExceeded`. The exact-`Real`
quotient-ring norm had the same structural-only final trim, and the direct
tensor-resultant path published that storage without a later canonicalizer.
These storage boundaries now retain the structural check as their fast path
and use the strict exact sign cascade only for an unknown candidate
coefficient. Unsupported identities remain stored.

The regression adds an exact-zero `z^2` coefficient to the relation `z - u`,
sets the image-degree budget to one, and observes construction after exact
trimming. Exact-normal positive and unsupported trigonometric trailing
coefficients remain degree two and fail the same budget, while direct local
and projective output checks prove that the exact zero is removed and the
unsupported identity is retained. A broader prototype also applied this
predicate to every quotient-field reduction. It had no distinct public
correctness trigger and made the cold projection row 4.90% slower, so it was
removed.

The quotient-ring regression constrains `x` to `sqrt(2) + sqrt(3)` and
eliminates `1 + y * (x^2 - (5 + 2*sqrt(6)))`. Its trailing determinant
coefficient is exactly zero but has no rational normal form. The old public
tensor result reported degree one; the retained path proves and returns the
constant one. A positive perturbation and an unsupported trigonometric zero
remain degree one.

Pinned ordinary comparisons are neutral. Algebraic-fiber projection measured
6.452 us mean (6.437--6.468 us 95% interval), with a -1.23% fitted change. The
direct exact-`Real` tensor norm measured 1.509 us (1.505--1.515 us); a fresh
back-to-back structural baseline measured 1.525 us and the fitted change was
-1.05%. Fixture-excluded cold rows measured 85.132 us (84.722--85.574 us) for
the exact-normal image-degree proof and 19.509 us (19.427--19.602 us) for the
radical determinant-output proof. The latter replaces different output
semantics and is 459% slower than the former unproved trim.

A warmed 32-iteration allocation probe found zero live-byte delta throughout.
Rational versus strict exact-zero algebraic-fiber projection used 165 versus
279 allocations and 8,880 versus 16,120 bytes per operation. Structurally
nonzero versus strict radical-zero direct tensor norms used 46 versus 266
allocations and 2,448 versus 11,032 bytes. Centralizing the predicate also
reduced the matched default release rlib from 12,870,158 to 12,846,628 bytes,
a 23,530-byte (0.18%) reduction; the final artifact is 22,904 bytes smaller
than the preceding retained slice. Default/all-feature library tests pass
292/293, every target passes, and strict Clippy, warning-denied rustdoc, both
release configurations, formatting, and diff gates pass.

### Reuse certified tensor degree and structural padding evidence

After exact degree certification, the direct one-retained-axis quotient norm
still materialized every coefficient in the authored dense dimension. The
sampled multi-axis path likewise expanded represented rational zeros at every
interpolation point. The retained paths now reuse the certified retained
degree and skip arithmetic only for borrowed exact-rational zero storage.
Unknown exact coefficients still participate, so this changes neither policy
nor proof authority.

The quadratic-source/linear-fiber closed form intentionally runs before
semantic degree certification: it remains valid across a degree drop and must
not reject an unsupported trailing identity. Its extracted coefficient tensors
now use a nonallocating mixed-radix scan to compact only represented rational
zero boundaries. A terminal trigonometric zero therefore retains the original
dimension, while a 64-slot rationally padded input returns the compact
three-coefficient polynomial `1 - 2*y^2`; its conservative published degree
bound remains unchanged.

Pinned Criterion rows show the following retained means and fitted changes:

| Tensor resultant case | Before | Retained | Change |
| --- | ---: | ---: | ---: |
| Compact quadratic closed form | 1.511 us | 1.215 us | 19.60% faster |
| 1,024-slot padded direct norm | 195.74 us | 9.845 us | 94.97% faster |
| 1,024-slot padded quadratic closed form | 501.16 us | 8.499 us | 98.30% faster |
| Compact 2 by 2 sampled grid | 7.191 us | 6.657 us | 7.43% faster |
| Padded 64 by 64 sampled grid | 1.1301 ms | 69.346 us | 93.86% faster |

Warmed 32-iteration allocation probes all ended with zero live-byte delta.
The padded direct norm falls from 6,220 allocations and 574,872 requested
bytes per call to 71 and 2,680; the padded sampled grid falls from 32,894 and
819,320 to 138 and 33,176; and the padded closed form falls from 12,323 and
1,032,528 to 51 and 1,824. The default release rlib grows 16,912 bytes (0.13%)
from 12,846,628 to 12,863,540. Default/all-feature library tests pass 295/296,
every target passes, and strict Clippy, warning-denied rustdoc, both release
configurations, formatting, and diff gates pass.

### Prepare quotient-ring tensor reduction once

`DenseTensorPolynomial::reduce_axis_modulo` formerly rebuilt retained exponent
tuples, cloned and reclassified the same modulus, reconstructed its leading
reciprocal, and allocated a new coefficient fiber for every retained tensor
coordinate. This work repeated even for an all-zero fiber or a tensor whose
selected axis was already below the certified divisor degree.

The retained implementation prepares one owned certified divisor, uses checked
row-major stride arithmetic and one reusable fiber buffer, and skips a fiber
only when every stored coefficient is a represented exact-rational zero. Any
opaque coefficient follows the unchanged quotient-ring algebra. A validated
already-reduced tensor returns a clone, while a modulus with certified trailing
zeros still produces the old authored output dimension. An independent
coordinate-based reference test compares all three possible axis positions;
separate tests preserve nominal padded shape and a terminal trigonometric-zero
fiber.

Pinned Criterion rows show the following retained means and fitted changes:

| Tensor quotient-ring reduction | Before | Retained | Change |
| --- | ---: | ---: | ---: |
| Dense 8 by 2 by 2 input | 5.551 us | 4.452 us | 19.67% faster |
| Sparse padded 8 by 64 by 64 input | 1.3773 ms | 129.62 us | 90.62% faster |
| Already-reduced 2 by 64 by 64 input | 342.78 us | 99.988 us | 70.80% faster |

The already-reduced baseline is deliberately conservative: it is the retained
prepared-divisor implementation with only the early return disabled, not the
older per-fiber-certification path. Warmed 32-iteration allocation probes end
with zero live-byte delta. Compact reduction falls from 23 allocations and
2,856 requested bytes per call to 8 and 1,080; padded reduction falls from
16,391 and 2,719,944 to 8 and 393,912, most of the retained bytes being the
required dense output itself. A borrowed-divisor prototype saved one further
allocation and 144 bytes, but repeated padded timing regressed 1.67% after an
initial 5.64% regression, so it was removed. The default release rlib shrinks
3,132 bytes (0.02%) from 12,863,540 to 12,860,408. Default/all-feature library
tests pass 297/298, every target passes, and strict Clippy, warning-denied
rustdoc, both release configurations, formatting, and diff gates pass.

### Move algebraic-image tensor storage between consumers

Algebraic tensor-image construction owns each intermediate tensor, but its
rational canonicalizer formerly cloned both dense vectors before consuming the
copy. The final univariate square-free handoff cloned the coefficient vector
again, and tagged projection cloned every scalar while converting a validated
rank-two tensor into bivariate rows.

An internal consuming `into_parts` handoff now moves the validated dimensions
and coefficient storage through all three boundaries. It changes no
normalization or proof: `exact_rational_normal_form` still alone decides which
coefficients collapse, opaque exact-`Real` fields remain opaque, and final
square-free and tagged projection routines receive the same coefficient order.
The existing four-root, nonrational-field, extraneous-cardinal, and tagged
foreign-component tests cover those semantic controls.

The public four-root image padded to 256 output slots falls from 781.42 to
717.96 us, 8.39% faster. The compact four-root row is noise-limited: retained
samples ranged from 398.97 to 419.74 us against a 410.23 us baseline, and the
final comparison found no change (`p=.09`). Tagged projection likewise remains
unchanged at 152.51 versus 152.28 us (`p=.46`). Warmed 32-iteration allocation
probes all end with zero live-byte delta:

| Algebraic tensor-image path | Before | Retained |
| --- | ---: | ---: |
| Compact four-root image | 7,635 allocs / 502,832 bytes | 7,616 / 488,328 |
| 256-slot padded four-root image | 9,667 allocs / 1,754,544 bytes | 9,648 / 1,544,968 |
| Tagged opposite-conjugate projection | 3,278 allocs / 278,208 bytes | 3,268 / 247,232 |

Moving rather than cloning the rank-two tagged storage reduces the default
rlib 9,048 bytes relative to the otherwise retained clone-based variant. The
complete slice grows the preceding artifact only 2,602 bytes (0.02%), from
12,860,408 to 12,863,010. The permanent catalog contains 264 Criterion rows.
Default/all-feature library tests pass 297/298, every target passes, and strict
Clippy, warning-denied rustdoc, both release configurations, formatting, and
diff gates pass.

### Reuse rational square-free evidence at root intake

`square_free_part` formerly cloned its already-trimmed source polynomial before
discovering that a borrowed rational modular or primitive-integer GCD path was
sufficient. It now inspects `exact_rational_ref` first. A modular coprimality
proof returns the original owned polynomial directly; a repeated-factor input
tries the primitive-integer GCD before paying for a generic source clone.
Nonrational exact coefficient fields deliberately retain the original generic
GCD path.

The derivative remains trimmed before the integer kernels. Removing that trim
looked redundant but repeatedly slowed the degree-64 repeated-factor control;
the trim also materializes/canonicalizes the lazy rational leading coefficient
that those kernels immediately consume. A broader helper that changed generic
GCD preparation regressed the repeated row 16.25% in its first paired run and
39.1% in isolation, so it was removed. Extending the integer probe to exact
nonrational coefficients made the exact-`Real` repeated quadratic 2.09% slower;
the retained borrowed-rational gate restores that path exactly. Separately,
Sturm construction now lets `sign_preserving_primitive_polynomial` perform its
own mandatory trim instead of cloning and trimming each initial member first.

Pinned Criterion slope estimates are:

| Root-intake path | Before | Retained | Result |
| --- | ---: | ---: | ---: |
| Degree-64 rational square-free polynomial | 6.707 us | 5.856 us | 12.26% faster |
| Degree-64 rational repeated polynomial | 29.299 us | 28.132 us | neutral in final run (`p=.06`); isolated retained run 5.04% faster |
| Exact-`Real` repeated quadratic | 3.445 us | 3.444 us | unchanged (`p=.31`) |
| Public Sturm isolation | 125.26 us | 122.42 us | 2.79% faster |

Warmed allocation probes end with zero live-byte delta. The square-free
rational case falls from 51 allocations / 19,808 requested bytes to 49 /
16,640, and the repeated rational case falls from 259 / 50,640 to 258 /
47,520. The exact-`Real` control remains exactly 69 / 3,872, and the public
Sturm row remains 3,526 / 124,288. The default release rlib grows 6,478 bytes
(0.05%), from 12,863,010 to 12,869,488. The permanent catalog contains 268
Criterion rows. Default/all-feature library tests pass 298/299, every target
passes, and strict all-target Clippy, warning-denied rustdoc, both release
configurations, formatting, and diff gates pass.

### Reuse canonical polynomial-division evidence

Generic polynomial long division formerly cloned and policy-trimmed a divisor
even when GCD or Sturm construction had just produced canonical storage. Each
step then cloned both arithmetic operands, added its quotient scale to a
preallocated zero, materialized the selected leading cancellation as another
zero, and asked the scalar predicate to prove it. The completed quotient was
trimmed once more even though its highest scale is the product of two values
already certified nonzero.

The retained divider keeps the validating wrapper for arbitrary crate callers,
but GCD, square-free, and Sturm internals use a private canonical-input path.
Borrowed `Real` arithmetic updates each lower remainder coefficient, the
selected leading slot is popped as the exact field identity it represents, and
each quotient degree replaces its preallocated zero exactly once. A canonical
polynomial with more than one stored coefficient is already known nonzero, so
only its scalar-constant case needs another predicate. Sturm also borrows its
last chain member instead of cloning it twice, and monic normalization trusts
the canonical GCD storage supplied by all three of its producers.

Generated exact-integer products through degree seven, with independently
padded dividend and divisor storage, recover their complete quotient and zero
remainder. A direct exact-`Real` product checks the nonrational field path, and
all prior unresolved-cancellation controls keep their fail-closed behavior. No
approximate decision, coefficient discard, or new public proof authority is
introduced.

Pinned release measurements are:

| Division/root path | Before | Retained | Change |
| --- | ---: | ---: | ---: |
| Dense degree-48 by degree-16 rational division | 55.427 us | 32.870 us | 39.94% faster |
| Exact-`Real` quadratic by linear division | 20.467 us | 19.119 us | 6.74% faster |
| Degree-64 rational square-free polynomial | 5.856 us | 5.825 us | unchanged control |
| Degree-64 repeated rational polynomial | 28.132 us | 25.120 us | 10.71% faster |
| Exact-`Real` repeated quadratic | 3.444 us | 1.572 us | 54.36% faster |
| Public Sturm isolation | 122.42 us | 117.28 us | 4.19% faster |

Warmed square-free allocations/requested bytes are unchanged at 49/16,640 for
the early-return rational control, fall from 258/47,520 to 257/45,936 for the
repeated rational case, and fall from 69/3,872 to 25/1,552 for the exact-`Real`
case. Public Sturm falls from 3,526/124,288 to 3,494/121,984. After the dense
rational fixture's lazy caches are fully warm, direct dense and exact-`Real`
division use 147/12,048 and 485/20,368 respectively; every final probe has zero
live-byte delta.

A broader borrowed prepared-divisor carrier was tested on public rational
images with monic and nonmonic shared factors. It changed the two end-to-end
rows by only +0.09% (`p=.70`) and +0.25% (`p=.49`), so the carrier was removed;
the rows remain as regression controls. The default release rlib shrinks 4,356
bytes (0.03%), from 12,869,488 to 12,865,132. The permanent catalog contains
272 Criterion rows. Default/all-feature library tests pass 299/300, every
target passes, and strict all-target Clippy, warning-denied rustdoc, both
release configurations, formatting, and diff gates pass.

### Remove duplicate local-field modulus storage

`LocalAlgebraicField` formerly owned both its mutable algebraic-root
representation and a second `Vec<Real>` clone of the same defining
coefficients. The root must remain owned because interval refinement updates
its interval, kind, and validation evidence, but the duplicate modulus never
mutated. The retained field exposes a private borrowed `modulus()` view into
the root instead, removing one `Vec` from the field layout and one coefficient
clone at construction.

The public exact univariate-division implementation also has a narrow
crate-private entry point that policy-trims a divisor as a borrowed slice. It
is used only by the public curve-resultant division surface, where the caller
does not retain a canonical prepared divisor. The ordinary generic divider,
local-field reductions, and GCD/Sturm schedules retain their established owned
or canonical-input behavior. Generated exact products with independently
padded storage exercise both owned and borrowed validating paths, including an
exact-`Real` coefficient field.

A global borrowed-divisor replacement was rejected after it regressed local
root-count rows. Explicit owned wrappers and an extra clone inside local-field
reduction did not recover that schedule consistently, so they were removed as
well. The final selective boundary leaves local reduction on the original
divider while retaining the independent storage win. Pinned release slope
estimates are:

| Local-field/division path | Before | Retained | Change |
| --- | ---: | ---: | ---: |
| Algebraic polynomial-fiber projection | 6.7562 us | 6.4575 us | 4.86% faster |
| Strict exact-zero image projection | 89.599 us | 87.517 us | 2.60% faster |
| Even-multiplicity local root count | 20.106 us | 20.112 us | unchanged (`p=.06`) |
| Common-fiber degree-drop root count | 13.187 us | 12.829 us | 2.75% faster |
| Dense degree-48 by degree-16 rational division | 33.428 us | 32.287 us | 4.77% faster |
| Exact-`Real` quadratic by linear division | 19.679 us | 14.160 us | 26.82% faster |

The direct-division comparisons isolate the borrowed-divisor addition from the
preceding canonical-divider work. Warmed polynomial-fiber projection falls
from 165 allocations / 8,880 requested bytes to 164 / 8,736, exactly the
quadratic modulus clone removed by the 24-byte-smaller field layout. Direct
dense division falls from 147/12,048 to 146/11,232; exact-`Real` division falls
from 485/20,368 to 334/14,360 because borrowing avoids cloning nonrational
coefficient handles and their cached graph state. Every final probe has zero
live-byte delta.

The default release rlib grows 6,370 bytes (0.05%), from 12,865,132 to
12,871,502; this small code-size cost is retained for the structural memory
reduction and the exact-`Real` public-division gain. The permanent catalog
contains 274 Criterion rows. Default/all-feature library tests pass 299/300,
every target passes, and strict all-target Clippy, warning-denied rustdoc, both
release configurations, formatting, and diff gates pass.

### Move local quotient arithmetic values

Local quotient-ring multiplication formerly cloned its destination `Real`
before every coefficient update, even though Hyperreal's assignment operator
can update rational scales in place and uses the same symbolic fallback for
nonrational values. Local exact division also cloned each computed quotient
scale, replaced an algebraically canceled final coefficient with a freshly
allocated local zero, and immediately trimmed it. Horner evaluation began from
another allocated zero rather than from the known leading coefficient.

The retained path preserves the original multiplication term order while
using `+=`, moves a quotient scale after its final divisor product, pops the
certified leading cancellation, and seeds both local-element and defining-`Real`
Horner evaluation from the actual leading term. Empty input still evaluates to
the exact additive identity. None of these operations adds proof authority:
the popped coefficient is the same field identity chosen by exact long
division, and all lower coefficients continue through the original reduction
and sign paths.

Denominator clearing no longer clones every optional denominator into a
parallel vector or materializes `[1]` for absent denominators. Prefix products
clone their already-canonical identity only when the coefficient has no
denominator; reverse traversal takes each owned denominator after its numerator
has been scaled and consumes it into the suffix product. A permanent
non-unit-scaled conjugate-saturation row forces this path and proves that the
same exact relation is constructed.

Pinned release measurements are:

| Local arithmetic path | Before slope | Retained slope | Criterion mean change |
| --- | ---: | ---: | ---: |
| Algebraic polynomial-fiber projection | 6.4935 us | 6.3703 us | unchanged (`-0.53%`, `p=.19`) |
| Strict exact-zero image projection | 87.762 us | 87.784 us | unchanged (`+0.28%`, `p=.49`) |
| Even-multiplicity local root count | 19.927 us | 18.620 us | 7.26% faster |
| Common-fiber degree-drop root count | 12.936 us | 11.908 us | 7.59% faster |
| Non-unit saturated conjugate factor | 113.64 us | 114.47 us | unchanged (`+0.27%`, `p=.66`) |

In a matched warmed allocation probe, the two projection controls are
unchanged at 162 allocations / 8,736 requested bytes and 276 / 15,976. The
even-multiplicity row falls from 250/19,259 to 238/18,683, common degree-drop
falls from 154/12,347 to 144/11,867, and non-unit saturation falls from
1,901/117,135 to 1,893/116,727. Every epoch ends with zero live bytes.

Seeding multiplication directly from its first product row was rejected after
it made the strict exact-zero projection 1.05% slower. Rewriting addition and
multiplication together produced 1.11--2.12% strict-control regressions across
two runs, even though each isolated edit was neutral or faster, so addition
keeps its original construction schedule. The default release rlib shrinks
16,154 bytes (0.13%), from 12,871,502 to 12,855,348. The permanent catalog
contains 275 Criterion rows. Default/all-feature library tests pass 299/300,
every target passes, and strict all-target Clippy, warning-denied rustdoc, both
release configurations, formatting, and diff gates pass.

### Reuse reduced local-field numerator evidence

Every local sign and zero query is private and is reached exclusively through
`LocalFieldElement`. Its zero/from-polynomial constructors and every add,
subtract, multiply, divide, and scale operation already reduce the numerator
modulo the selected root's defining polynomial. Nevertheless, each query
formerly cloned that numerator and ran the generic local reduction again
before checking structural zero, consulting the sign cache, or evaluating at
the selected root.

The retained boundary consumes the borrowed canonical numerator directly.
Debug builds assert that it is nonempty, has degree strictly below the modulus,
and has no structurally zero trailing coefficient; release builds pay none of
those checks. Cache misses still clone exactly once when a durable sign is
inserted. Evaluation, exact common-root identity, interval refinement,
certainty aggregation, and the behavior of undecided coefficients are
unchanged.

Pinned final release measurements are:

| Local sign/zero consumer | Before slope | Retained slope | Criterion mean change |
| --- | ---: | ---: | ---: |
| Algebraic polynomial-fiber projection | 6.2912 us | 6.0834 us | 3.62% faster |
| Strict exact-zero image projection | 89.232 us | 87.053 us | 1.25% faster |
| Non-unit saturated conjugate factor | 115.28 us | 110.45 us | 3.35% faster |
| Even-multiplicity local root count | 18.323 us | 18.114 us | 1.38% faster |
| Common-fiber degree-drop root count | 11.625 us | 11.404 us | 1.81% faster |

A separate focused repeat of the strict exact-`Real` row measured 83.309 us
and a 4.94% mean improvement; the full matched pass above is retained as the
more conservative result. Under the same warmed allocation convention as the
preceding slice, ordinary projection falls from 162 allocations / 8,736
requested bytes to 154 / 8,304, strict projection from 276/15,976 to
268/15,544, saturation from 1,893/116,727 to 1,775/109,143, even multiplicity
from 238/18,683 to 235/18,539, and degree-drop from 144/11,867 to 135/11,435.
Every epoch ends with zero live bytes.

Searching the cache newest-first was tested and removed: it made the even row
neutral and the degree-drop row 0.61% slower, showing that insertion order
matches these Sturm reuse patterns better than temporal locality. The default
release rlib grows 9,094 bytes (0.07%), from 12,855,348 to 12,864,442; this is
accepted for the consistent public time and allocation reductions. The
permanent catalog remains at 275 Criterion rows. Default/all-feature library
tests pass 299/300, every target passes, and strict all-target Clippy,
warning-denied rustdoc, both release configurations, formatting, and diff
gates pass.

### Build local quotients and small determinants directly

`LocalFieldElement::zero` owns a one-coefficient numerator vector. Both
synthetic dividers formerly filled a quotient with those owning zeros and then
overwrote every slot. Exact local division did the same even though long
division discovers nonzero quotient terms in descending degree order. The
retained schedules build that order directly and reverse it once. Exact
division inserts a zero only when cancellation trims across a genuinely absent
quotient degree; a dedicated generated product preserves such a middle gap and
a non-unit local-field denominator.

The polynomial-image norm formerly sent even 1-by-1 and 2-by-2 matrices through
the recursive Berkowitz characteristic construction. Those two dimensions now
use the entry itself and `a*d - b*c`, respectively, after the same checked
matrix-shape validation and final field-aware trim. Dimensions three and above
retain the division-free Berkowitz path unchanged. A direct private regression
fixes row-major determinant orientation, denominator canonicalization, and
exact certainty, while the existing public degree-one, saturated degree-two,
degree-three, reducible-modulus, and exact-`Real` image tests remain the output
oracles.

Pinned final release measurements are:

| Local quotient/determinant consumer | Original mean | Retained mean | Criterion mean change |
| --- | ---: | ---: | ---: |
| Triple diagonal-root deflation | 6.7536 us | 6.4747 us | 3.70% faster |
| Seven repeated closed-endpoint deflations | 10.241 us | 9.7751 us | 4.27% faster |
| Ordinary degree-one polynomial image | 6.0947 us | 5.6276 us | 7.62% faster |
| Strict exact-zero degree-one image | 88.306 us | 88.433 us | unchanged (`p=.54`) |
| Non-unit saturated degree-two image | 113.95 us | 110.08 us | 2.75% faster |

The exact-divider change in isolation is time-neutral on saturation (`-0.10%`,
`p=.66`) but removes 8 allocations and 384 requested bytes there. In matched
warmed epochs, diagonal deflation falls from 87 allocations / 7,880 requested
bytes to 77 / 7,400, repeated endpoint deflation from 197/11,168 to 169/9,824,
ordinary projection from 156/8,304 to 147/7,680, and saturation from
1,786/109,143 to 1,757/107,151. The determinant accounts for 21 allocations
and 1,608 bytes of the saturation reduction; exact quotient construction
accounts for the remaining 8/384. Every epoch ends with zero live bytes.

The default release rlib grows 1,566 bytes (0.01%), from 12,864,442 to
12,866,008, accepted for the public time and allocation gains. Two permanent
rows bring the Criterion catalog to 277 entries. Default/all-feature library
tests pass 301/302, every target passes, and strict all-target Clippy,
warning-denied rustdoc, both release configurations, formatting, and diff
gates pass.

### Materialize only populated local-image storage

Local image-polynomial convolution formerly created an owning
`LocalFieldElement::zero` at every output degree before replacing most of those
values with products. It now accumulates into optional slots and materializes
canonical zero only for degrees that receive no term. Product order, each
field multiplication/addition, structural-zero skips, denominator handling,
and final field-aware trimming are unchanged. A private sparse convolution
regression fixes the retained zero gaps, coefficients, denominator form, and
exact certainty.

The quotient-multiplication matrix similarly uses checked optional row-major
slots, moves each completed entry out of its column workspace, verifies that
every slot was filled, and only then returns the dense matrix. An eliminated
high coefficient is moved because that workspace slot is never read again.
The pseudo-reduction loop still applies exactly the same leading-coefficient
powers in the same order. Finally, each Berkowitz dot product and
characteristic convolution moves its first term into an optional accumulator;
all subsequent ordered additions are the original operations.

Pinned original-to-final release measurements are:

| Local image consumer | Original mean | Retained mean | Criterion mean change |
| --- | ---: | ---: | ---: |
| Ordinary degree-one image | 5.7036 us | 5.4633 us | 3.72% faster |
| Strict exact-zero degree-one image | 87.268 us | 85.733 us | within noise threshold |
| Non-unit saturated degree-two image | 108.92 us | 104.20 us | 5.45% faster |
| Cubic-source/quadratic-image Berkowitz norm | 25.705 us | 21.876 us | 14.63% faster |

The new cubic row permanently protects the general determinant path. In
matched warmed allocation epochs, saturation falls from 1,757 allocations /
107,151 requested bytes to 1,556 / 94,719, while the cubic norm falls from
610/39,184 to 463/31,192. Every epoch ends with zero live bytes. In isolation,
restoring clone-over-zero matrix assembly made saturation 3.92% slower and the
cubic row 5.23% slower. Moving the eliminated coefficient removes another
34 allocations/1,824 bytes and 24/1,152 bytes respectively; its isolated time
was order-sensitive, but the complete retained bundle improves both rows.

Two broader variants were removed. Hoisting a structural-zero check around
pseudo-scaling duplicated the nonzero scan and slowed both measured rows about
1.4%. Moving structural-zero subtraction survivors before the existing add
helper made saturation 4.07% slower. The original scaling and addition
schedules therefore remain. The default release rlib grows 50,638 bytes
(0.39%), from 12,866,008 to 12,916,646, accepted for the consistent public
time and allocation reductions. One permanent row brings the Criterion
catalog to 278 entries. Default/all-feature library tests pass 302/303, every
target passes, and strict all-target Clippy, warning-denied rustdoc, both
release configurations, formatting, and diff gates pass.

### Move completed local fibers into bivariate output

The two callers of `local_fiber_to_bivariate` formerly kept a completed local
fiber alive while the exporter cloned every numerator. First-parameter output
then cloned every scalar a second time while transposing the ragged coefficient
grid. Both callers discard the local fiber immediately after export. The
exporter now consumes it after the same all-coefficient denominator check.
Second-parameter output moves each numerator vector directly; first-parameter
output moves its scalars into retained-power rows and creates canonical zero
only for a missing ragged entry. Coefficient order, zero padding, and the rule
that any noncanonical local denominator makes export unsupported are unchanged.
A private regression fixes both orientations and the denominator boundary.

Pinned release measurements are:

| Local-fiber export consumer | Borrowed/cloning mean | Consuming mean | Result |
| --- | ---: | ---: | ---: |
| Triple diagonal deflation, retain first parameter | 6.5186 us | 6.4113 us | 1.63% faster, within Criterion's noise threshold |
| Triple diagonal deflation, retain second parameter | 6.4501 us | 6.3081 us | 2.20% faster (paired change 1.90%) |
| Saturated polynomial image | 102.99 us | 103.83 us | unchanged (`p=.87`) |

In matched warmed allocation epochs, first-parameter deflation falls from 76
allocations / 7,400 requested bytes to 73 / 7,208, and saturation falls from
1,545/94,719 to 1,541/94,479. Both epochs end with zero live bytes.

Three broader storage shapes were removed. An iterator transpose grew the rlib
18,658 bytes beyond the retained loop. Dense zero-prefill grew it 7,478 bytes.
A shared transpose helper saved one fewer allocation and moved saturated time
1.29% in the wrong direction. Fusing `fiber_coefficient_polynomials` into local
specialization removed only one outer allocation, left all three public timing
controls within the noise threshold, and grew the rlib another 36,702 bytes, so
the shared specialization helper remains. The retained exporter grows the
default release rlib 7,498 bytes (0.058%), from 12,916,646 to 12,924,144. A
second-parameter row brings the Criterion catalog to 279 entries.
Default/all-feature library tests pass 303/304, every all-feature target passes,
and strict all-target Clippy, warning-denied rustdoc, both release
configurations, formatting, and diff gates pass.

### Reuse image content across source-component saturation

An identically-zero polynomial-image determinant triggers saturation of source
components on which every image-parameter coefficient vanishes. The former
loop recomputed `gcd(residual, coefficient_0, ..., coefficient_n)` after each
removed source multiplicity. The coefficient-only GCD is independent of that
residual. It is now computed once in the same coefficient order, after every
coefficient has passed the same local trim and zero query. Each successive
residual is intersected with that one content factor, so a source primary
component is still removed to full multiplicity even when the image relation
contains its factor only once. The freshly owned coefficient polynomials move
into this one-time fold with `mem::take`; denominator clearing and final factor
export are unchanged.

A new exact regression uses `F=(u-alpha)(u-1)^4` and a four-coefficient image
relation whose coefficients each contain `(u-1)` once. Under strict and
approximate policies it removes all four multiplicities and checks the exact
factor and retained relation. The same case also runs over the reducible local
carrier `(alpha^2-2)(alpha-3)`, fixing the unreduced retained-power storage and
exact certainty rather than assuming a globally irreducible modulus.

Pinned original-to-final release measurements are:

| Saturation consumer | Repeated coefficient GCD | Reused image content | Result |
| --- | ---: | ---: | ---: |
| Repeated conjugate factor | 103.97 us | 95.109 us | 8.39% faster |
| Fourfold, four-coefficient component | 2.3469 ms | 2.1867 ms | 7.17% paired improvement |

In matched warmed allocation epochs, the repeated-conjugate row falls from
1,423 allocations / 85,679 requested bytes to 1,326 / 80,236. The fourfold row
falls from 34,313/2,482,554 to 32,410/2,377,292. Every epoch ends with zero live
bytes.

Consuming the complete coefficient vector through an outer iterator regressed
the fourfold row 2.05% and grew the rlib 22,902 bytes relative to the retained
indexed move. Omitting structurally absent ragged tails before the established
trim schedule regressed it 2.38% and grew the rlib 19,006 bytes. Both variants
were removed. The retained slice shrinks the default release rlib 6,126 bytes
(0.047%), from 12,924,144 to 12,918,018. One permanent fourfold row brings the
Criterion catalog to 280 entries.
Default/all-feature library tests pass 304/305, every all-feature target passes,
and strict all-target Clippy, warning-denied rustdoc, both release
configurations, README, formatting, and diff gates pass.

### Build ordered-field Bernstein work from populated coefficients

Selected algebraic fibers first attempt division-free root isolation over the
already selected exact coefficient field. The former Bernstein path repeatedly
constructed the field zero, multiplied the first value by one, and added zero
while evaluating endpoints and controls. Affine interval composition also
created an owning zero for the new highest coefficient before immediately
overwriting it. Those identities are now omitted: Horner evaluation starts at
the leading coefficient, each control starts at the shifted constant, and the
highest composed coefficient is appended directly. All remaining scale/add
operations retain their original order and exact ratios.

When a represented dyadic root is found, synthetic division now appends the
descending quotient coefficients and reverses once instead of allocating an
owning zero for every quotient slot. Direct generic regressions fix both closed
endpoints plus a repeated midpoint of `x(x-1)(x-1/2)^2`, and require
`(x^2-2)^2` on `(1,2)` to request the complete fallback. The public selected-
fiber rows separately require successful rational deflation and successful
Sturm fallback for a repeated irrational root.

Pinned original-to-final release measurements are:

| Selected-fiber consumer | Zero-seeded storage | Populated storage | Result |
| --- | ---: | ---: | ---: |
| Rational Bernstein deflation | 36.863 us | 31.448 us | 14.54% faster |
| Repeated-irrational Sturm fallback | 65.331 us | 59.522 us | 9.51% paired improvement |

Matched warmed allocation epochs fall from 389 allocations / 29,998 requested
bytes to 301/25,198 for rational deflation, and from 758/52,676 to 646/46,292
for the fallback row. Every epoch ends with zero live bytes.

The adjacent retained-field rational normalizer now borrows the denominator's
leading scalar while computing its projective inverse. A permanent public
`1/(4+2*alpha)` row proves the exact normalized representation
`(1/2)/(2+alpha)`. Borrowing is timing-neutral at 2.4044 versus 2.3946 us
(`+0.075%`, `p=.92`) and leaves its warmed allocation epoch unchanged at 26
allocations / 2,384 requested bytes with zero live bytes, but shrinks the
default rlib 2,320 bytes relative to the cloning form.

The complete retained slice grows the default release rlib 16,718 bytes
(0.129%), from 12,918,018 to 12,934,736, accepted for the two substantial
public time/memory improvements. Three permanent rows bring the Criterion
catalog to 283 entries. Default/all-feature library tests pass 306/307, every
all-feature target passes, and strict all-target Clippy, warning-denied rustdoc,
both release configurations, README, formatting, and diff gates pass.

### Reuse rational-fiber pivot and residual storage

`reduce_bivariate_rational_function_at_algebraic_parameter` chooses the first
nonzero denominator coefficient, constructs one retained-field ratio, and
proves every remaining coefficient has that same ratio. The former loop cloned
the numerator pivot and both coefficients of every residual even though local
division, multiplication, subtraction, and zero testing only borrow them. It
now borrows populated coefficients and uses one lazy canonical zero for a
genuinely missing ragged power.

The pivot scan itself is authoritative nonzero evidence. Ratio construction
now enters a private exact symbolic-quotient helper with that proof instead of
repeating the same zero query. The residual at the pivot is the defining
cross-multiplied identity `ratio*denominator[pivot]-numerator[pivot]`; it is
therefore skipped. Every other residual retains the same multiplication,
subtraction, reduction, and policy-authoritative zero query in the same order.

A new exact regression exercises both retained-axis orientations under strict
and approximate policies and over both `alpha^2-2` and the reducible carrier
`(alpha^2-2)(alpha-3)`. A zero numerator is ragged against three non-monic
denominator powers and must still export the visible projective value
`0/(2+alpha)`. Separate cases require `FiberDependent` for a numerator-only
power and `ZeroDenominator` for a vanishing denominator. The pre-existing
degree-15 correlation test remains the higher-degree output oracle.

Pinned original-to-final release measurements are:

| Rational-fiber consumer | Cloned residuals | Borrowed residuals | Result |
| --- | ---: | ---: | ---: |
| Non-monic retained denominator | 2.3946 us | 1.6947 us | 29.23% faster |
| Dense degree-15 proportional identity | 9.2402 us | 7.5038 us | 18.79% faster |

Matched warmed allocation epochs fall from 26 allocations / 2,384 requested
bytes to 18/1,712 for the non-monic row, and from 138/13,736 to 103/12,056 for
the dense row. Every epoch ends with zero live bytes. Making the shared zero
lazy was timing-neutral on both rows while avoiding its unused allocation and
shrinking code. Reusing the pivot proof was neutral on the one-coefficient row
and improved the dense row 2.35%; omitting the tautological pivot residual
improved those final intermediate baselines 24.01% and 1.64%, respectively.

The complete slice shrinks the default release rlib 2,650 bytes (0.020%), from
12,934,736 to 12,932,086. One permanent dense row brings the Criterion catalog
to 284 entries. Default/all-feature library tests pass 307/308, every
all-feature target passes, and strict all-target Clippy, warning-denied rustdoc,
both release configurations, README, formatting, and diff gates pass.

### Construct scalar image inputs directly

After polynomial-image degree validation, each authored scalar coefficient is
wrapped as a constant in the selected retained field. The former conversion
sent every one-element polynomial through generic quotient reduction and then
collected an error that could not occur: `LocalAlgebraicField::new` accepts only
a defining polynomial of positive degree, so a scalar is already strictly
below its modulus. The projection now constructs the same denominator-free
`LocalFieldElement` directly. Coefficient order, stored unsupported values,
degree bounds, primitive-content proof, local determinant, saturation, and
retained/global report construction are unchanged.

The common linear projection moved from 5.7290 to 5.5493 us in the paired
release comparison (2.48% faster). The cubic Berkowitz control remained
unchanged at 22.471 versus 22.150 us (`p=.20`); a final restored-schedule repeat
measured 21.982 us. Matched warmed allocation epochs are identical at 139
allocations / 7,176 requested bytes for the linear row and 463/31,192 for the
cubic row, with zero live bytes.

A broader attempt also bypassed generic construction for the determinant's
constant-one polynomial. It removed no allocation, its causal cubic change was
within the noise threshold (2.05%), and it grew the rlib 12,260 bytes; it was
removed. The retained scalar-input change instead shrinks the default release
rlib 34,982 bytes (0.271%), from 12,932,086 to 12,897,104. The Criterion catalog
remains at 284 entries. Default/all-feature library tests pass 307/308, every
all-feature target passes, and strict all-target Clippy, warning-denied rustdoc,
both release configurations, README, formatting, and diff gates pass.

### Short-circuit decided lower Sturm boundaries

The batched open-interval fiber counter builds one local-field Sturm sequence
and caches boundary variations across all requested intervals. It formerly
evaluated both boundaries eagerly, even when the lower boundary was already an
exact root or its evaluation had failed. The single-interval API stops at that
lower result. The batch path now follows the same ordering, so an irrelevant
upper evaluation cannot consume arithmetic, allocations, or retained-root
refinement before the report is constructed. The ordinary counted path and its
insertion-order cache are unchanged.

An exact regression compares complete batch reports with independent
single-interval reports for lower and upper endpoint roots, both retained-axis
orientations, strict and approximate policies, and both irreducible
`alpha^2-2` and reducible `(alpha^2-2)(alpha-3)` carriers. Three permanent
64-interval rows separately cover adjacent boundaries, one shared nonroot
lower boundary, and one shared lower endpoint.

Pinned eager-to-short-circuit release measurements are:

| Batched root-count consumer | Eager boundaries | Lower-first | Result |
| --- | ---: | ---: | ---: |
| 64 adjacent intervals | 438.00 us | 438.62 us | unchanged (`p=.38`) |
| 64 intervals sharing a nonroot lower bound | 421.41 us | 420.49 us | within noise threshold |
| 64 intervals sharing a lower endpoint root | 224.58 us | 4.9101 us | 97.81% faster |

For the endpoint workload, a warmed 64-call allocation epoch falls from 994
allocations / 59,744 requested bytes to 45 / 7,768, with zero live bytes. A
cache-search prototype that checked the first and last stored boundaries before
the interior was neutral for adjacent intervals (`-1.14%`, `p=.15`) but
regressed the shared-lower row 2.89%, so insertion-order lookup remains. The
retained short circuit grows the default release rlib by only 318 bytes
(0.0025%), from 12,897,104 to 12,897,422. Three rows bring the Criterion catalog
to 287 entries. Default/all-feature library tests pass 308/309, every
all-feature target passes, and strict all-target Clippy, warning-denied rustdoc,
both release configurations, README, formatting, and diff gates pass.

### Preserve selected-fiber fallback work and ownership

Selected-fiber isolation first attempts division-free Bernstein subdivision,
then uses a local-field Sturm sequence for repeated or inseparable roots. A
terminal error after that fallback formerly discarded all visible work:
`sturm_sequence_length`, `subdivision_steps`, and
`retained_refinement_steps` were reset to zero. Progress-aware cold report
construction now retains each counter and the accumulated certainty at every
post-construction failure. A failed midpoint deflation also keeps its actual
error class instead of unconditionally becoming `Undecided`.

The exact incomplete fixture combines the repeated selected root
`alpha=sqrt(1/2)` with rational fiber roots at `1/4`, `1/2`, and `3/4`. It
finishes `Undecided` only after nonzero Sturm, subdivision, and retained-root
refinement work. The regression requires all three evidence fields under both
retained-axis orientations, strict and approximate policies, and irreducible
`2*alpha^2-1` plus reducible `(2*alpha^2-1)(alpha-2)` carriers.

The fallback also cloned its complete source fiber before every Sturm build,
although the returned sequence already owns that unchanged polynomial as row
zero. It now moves the carrier into the sequence and evaluates midpoint signs
through that row. Only a discovered rational root clones row zero back for the
destructive deflation/restart; terminal fallback paths never clone it.

Pinned cloned-carrier-to-final release measurements are:

| Selected-fiber consumer | Cloned carrier | Deferred restart clone | Result |
| --- | ---: | ---: | ---: |
| Bernstein-only rational deflation control | 31.564 us | 30.849 us | 1.83% faster control |
| Repeated-irrational Sturm fallback | 58.856 us | 57.347 us | 3.11% faster |
| Two-rational-root partitioned fallback | 570.72 us | 567.47 us | 0.94% faster, within noise threshold |

For the repeated fallback, a warmed allocation epoch falls from 634
allocations / 46,292 requested bytes to 630 / 45,956, with an unchanged
9,000-byte peak and zero live bytes. A streaming boundary/value-vector
prototype regressed the partitioned row 2.52% and was removed. Moving row zero
back out of the sequence saved more restart clones but inflated the release
rlib about 36 KB; deferred cloning retains the terminal-path gain without that
drop-state cost. The complete retained slice grows the default rlib only 5,234
bytes (0.041%), from 12,897,422 to 12,902,656. One permanent partitioned row
brings the Criterion catalog to 288 entries. Default/all-feature library tests
pass 309/310, every all-feature target passes, and strict all-target Clippy,
warning-denied rustdoc, both release configurations, README, formatting, and
diff gates pass.

### Borrow plain algebraic-fiber projection evidence and storage

The plain quotient-ring fiber projector formerly constructed a complete
`LocalAlgebraicField` even though it used only the retained root's immutable
defining polynomial. That cloned the refinable root, allocated its polynomial
storage, and created an empty sign cache whose policy, certainty, and
refinement state were never consulted. A shared admission predicate now keeps
the exact same valid-root, singleton-isolator, and positive-degree checks while
the projector borrows the defining coefficients directly. Invalid evidence
still returns the same empty `InvalidEvidence` report.

The second retained-axis orientation was also passed through
`fiber_coefficient_polynomials`, which cloned the complete authored coefficient
grid and immediately lent it to the quotient norm. It now lends the original
grid. The first orientation still builds its required transpose. A regression
requires identical constructed results across both orientations under strict
and approximate policies and under irreducible `2*alpha^2-1` and reducible
`(2*alpha^2-1)(alpha-2)` carriers. It also compares the shared private
admission boundary and public report for invalid validation, a non-singleton
isolator, and a constant defining polynomial. The adjacent API documentation
now correctly states that determinant growth follows the retained defining
degree; `max_source_degree` bounds that dimension rather than causing it.

Pinned constructor-clone-to-direct-admission runs measured 1.7201 to 1.6455 us
for the first orientation (Criterion fitted change -5.71%) and 1.7537 to
1.6730 us for the second (-4.59%). In the final matched grid-clone-to-borrow
confirmation, the first-orientation control was unchanged at 1.6857 versus
1.6771 us (`p=.13`), while the second orientation improved from 1.6356 to
1.5648 us (fitted change -4.07%). An earlier pair measured the latter gain at
10.42%, so only the conservative final confirmation is used for the retained
claim.

Warmed allocation epochs end with zero live bytes. Relative to complete field
construction, the first orientation falls from 75 allocations / 3,584
requested bytes / 2,512 peak live bytes to 74 / 3,440 / 2,368. The second
orientation falls from 73 / 3,368 / 2,400 to 69 / 3,032 / 2,064 after both
changes; the grid borrow alone removes three allocations and 192 requested and
peak-live bytes. A compact shared-call formulation regressed the two rows
3.93% and 5.76% and grew the rlib slightly, so the explicit optimized arms
remain. Their matched grid-clone-to-final rlib cost is 12,854 bytes; the whole
slice grows the default rlib 13,086 bytes (0.101%), from 12,902,656 to
12,915,742. Two permanent rows bring the Criterion catalog to 290 entries.
Default/all-feature library tests pass 310/311, every all-feature target passes,
and strict all-target Clippy, warning-denied rustdoc, both release
configurations, README, formatting, and diff gates pass.

### Reuse tagged-tensor storage and square-free carriers

`project_selected_tensor_fiber_via_tagged_norm` formerly constructed separate
dense tensors for `z` and `source[0]`, subtracted them, and invoked generic
multidimensional convolution to obtain the fixed polynomial
`(z-source[0])^2`. It now constructs the exact three populated coefficients
`z^2`, `-2*z*source[0]`, and `source[0]^2` directly in the same validated
row-major shape. An independent oracle compares this storage with the generic
tensor arithmetic for ranks three through six and checks invalid tag axes.

After exact source elimination, the completed flat `[tag,target]` tensor was
first nested as bivariate rows, then cloned and transposed into the primitive
component's fiber-major input, after which the first grid was discarded. A
fallible move-based handoff now constructs only the required fiber vectors.
Its shape and orientation are checked against the public primitive-component
path, including malformed dimensions. The primitive GCD, division,
normalization, and final bivariate reconstruction are unchanged.

The function already square-freed every source constraint for quotient
reduction and elimination, but its final tag projection reverted to the
original first carrier. For `(x^2-2)^3`, that needlessly changed a degree-two
root enumerator into a degree-six subset determinant with repeated output
multiplicity. The final norm now reuses the strictly validated square-free
carrier only when its degree is lower; otherwise the original root remains on
the hot path. Structurally identical conjugate carriers also share the first
square-free computation, while differently represented carriers take the full
independent path. Exact regressions require repeated interval carriers and the
exact-witness carrier `(x-1)^3` to return the same report as their square-free
representations. Empty sources, misshaped tensors, and invalid stored evidence
retain `InvalidEvidence`.

Pinned release measurements are:

| Tagged selected-tensor projection | Before | Final | Result |
| --- | ---: | ---: | ---: |
| Opposite-conjugate cubic, square-free carriers | 147.95 us | 142.44 us | 3.72% lower observed mean |
| Distinct quadratic carriers | 166.83 us | 166.55 us | unchanged (`p=.35`) |
| Opposite-conjugate cubic, degree-six repeated carriers | 292.61 us | 157.12 us | 46.30% lower observed mean |

The direct tag construction's paired fitted change was -1.71%, and the flat
handoff's was -0.74%; both stayed inside Criterion's noise threshold but had
material allocation reductions. Reusing the square-free projection carrier
produced a -42.96% paired fitted change on the repeated case, and sharing its
duplicate constraint added another -3.28%.

Warmed epochs all end with zero live bytes. The ordinary case falls from 3,143
allocations / 242,472 requested bytes / 32,288 peak live bytes at slice start
to 3,074 / 234,688 / 31,248. Direct tag construction removes 28 allocations
and 1,184 bytes; the move handoff removes another 10 and 5,832 bytes; shared
constraint preparation removes the remainder. Against a conservative
direct-storage baseline without constraint or projection reuse, the repeated
case falls from 6,023 allocations / 365 reallocations / 379,072 requested
bytes / 60,696 peak live bytes to 3,344 / 83 / 245,760 / 34,728.

The complete default rlib shrinks 45,370 bytes (0.351%), from 12,915,742 to
12,870,372. Two permanent adversarial/control rows bring the Criterion catalog
to 292 entries. Default/all-feature library tests pass 313/314, every
all-feature target passes, and strict all-target Clippy, warning-denied
rustdoc, both release configurations, README, formatting, and diff gates pass.

### Preserve correlated-image progress and avoid proved no-op work

`represent_algebraic_tensor_image` collapses duplicate or exactly certified
affine-related source axes before constrained resultants. Those substitutions
are successful source eliminations, but a later failure formerly reported only
the compacted resultant-loop index. The report now advances one shared counter
for every successful affine substitution, certified independent-axis removal,
and resultant. A regression collapses two identical cubic source axes and then
forces the remaining resultant to retain `UndecidedCoefficient`; the complete
failure evidence remains present and `elimination_count` is exactly one.

Every retained source constraint is square-freed before quotient-ring work.
Distinct non-affine conjugates can still carry the same polynomial, so exact
structural carrier equality now reuses the earlier proved square-free
constraint. Different coefficient storage follows the original independent
path. Two roots of `x^3-3x+1` prove that affine collapse does not trigger, and
their repeated degree-six carriers return the exact same final report as the
square-free cubic.

The orchestration also used the borrowed `reduce_axis_modulo` API for every
source before elimination and for every remaining source afterward. When an
axis stores `d` coefficients and `d < constraint.len()`, its largest possible
power is already strictly below the divisor degree; the polynomial is its own
exact remainder. The caller now skips that operation instead of cloning the
complete dense tensor or padding a low-degree axis. Quotient reductions whose
stored power bound reaches the divisor degree are unchanged. Finally,
rational-class `Real` coefficients bypass the bounded symbolic normal-form
walk: they carry no computable DAG, and their `Rational` payload already owns
the lazy canonical-coordinate cache used by numeric operations. Nonrational
coefficients still take the same exact normal-form proof path.

Pinned paired release measurements are:

| Correlated tensor image | Before | After | Paired fitted result |
| --- | ---: | ---: | ---: |
| Four distinct square roots | 389.24 us | 385.91 us | -1.00%, within noise threshold |
| Same relation, output dimension padded to 256 | 705.15 us | 521.67 us | -26.23% |
| Rational-class short circuit, compact control | 385.45 us | 382.16 us | -1.16% |
| Rational-class short circuit, padded control | 518.42 us | 485.19 us | -7.05% |
| Two non-affine conjugates, square-free cubic | 166.74 us | 165.31 us | unchanged (`p=.16`) |
| Same conjugates, repeated degree-six carrier | 202.05 us | 186.97 us | -8.06% |

The no-op reduction and rational short circuit compound to about 31.2% on the
padded public row. The shared canonicalization helper also improved the three
tagged projection controls by about 3.1--3.2% without changing their allocation
counts. Stable four-root allocation epochs fall from 7,524 allocations and
472,824 requested bytes to 7,512 and 465,944 for the compact relation. The
padded relation falls from 9,556 allocations / 1,529,464 requested bytes /
395,616 peak live bytes to 9,544 / 742,296 / 208,584. Shared preparation of the
repeated conjugate constraint falls from 3,883 allocations / 64 reallocations /
196,560 requested bytes / 27,200 peak bytes to 3,485 / 52 / 187,032 / 26,800.
Every epoch ends with zero live bytes; the rational-class short circuit changes
none of those stable allocation counts.

Three broader forms were removed. Borrowing roots through a vector of
references saved four allocations and 1,472 bytes but regressed reversed
paired timings 3.72%/2.75% through poorer locality. Skipping only the final
duplicate canonicalization pass was allocation-neutral and timing-neutral or
slightly worse. Extending the quotient no-op guards into tagged projection was
also allocation-neutral, with two controls drifting 0.45%/0.41% slower.

The complete default rlib shrinks 3,684 bytes (0.0286%), from 12,870,372 to
12,866,688. Two permanent conjugate-carrier rows bring the Criterion catalog
to 294 entries. Default/all-feature library tests pass 315/316, every
all-feature target passes, and strict all-target Clippy, warning-denied
rustdoc, both release configurations, README, formatting, and diff gates pass.

### Complete and streamline independent-root binary images

`transform_algebraic_roots_binary` formerly trusted each carrier's cached
`is_valid` flag without strictly replaying its stored isolating evidence. A
carrier whose interval was later inverted could therefore enter elimination.
Both operands now pass strict validation at this public construction boundary;
invalid or stale evidence returns `InvalidEvidence` before any relation is
built.

The degree budget was also enforced against authored carrier degrees before
the exact square-free reduction that resultants actually need. Repeated
degree-six carriers such as `(x^2-2)^3` and `(x^2-3)^3` were consequently
reported unsupported even though their independent binary image has degree at
most four. The hot path still borrows ordinary in-budget polynomials. Only an
over-budget product triggers strict square-free preparation, structurally
equal carriers share that proof, and the checked reduced-degree product is
then tested against the same limit. A square-free quartic-by-cubic control
continues to return `UnsupportedDegree`; shared opposite conjugates of the
same repeated carrier complete to exact zero.

For exact-integer source polynomials, binary interpolation now asks a private
scalar resultant helper for just the determinant value. It trims borrowed
exact-rational storage, crosses once into a flat `BigInt` Sylvester matrix, and
uses the existing checked fraction-free Bareiss kernel. The public
report-bearing resultant API is unchanged. Fractional rational coefficients
and any failed integer precondition retain the general exact-`Real` Sylvester
fallback; generated integer, fractional, and maximum-dimension comparisons
require equality with the public report.

Relation construction preserves operand orientation while removing proved
work. Addition and subtraction use exact Horner composition; their slopes are
signed units, so the linear factor is formed directly without general scalar
multiplication. Multiplication and division build ascending powers
incrementally and reverse them once for interpolation. Exact rational output
trimming inspects the rational payload directly and refuses to discard an
unexpected nonrational leading coefficient. Division retains the same
policy-certified denominator nonzero proof and interval-domain rejection.

Pinned release measurements from the slice boundary to the retained form are:

| Independent binary image | Before | Retained | Change |
| --- | ---: | ---: | ---: |
| Addition | 36.544 us | 24.751 us | -32.50% |
| Division | 28.388 us | 21.403 us | -24.33% |
| Multiplication | 27.02 us | 20.23 us | -25.03% |
| Distinct repeated degree-six carriers | 49.294 us | 41.129 us | -16.13% |
| Shared repeated degree-six carrier | 45.519 us | 29.368 us | -35.49% |
| Subtraction control | - | 28.630 us | new permanent row |

Warmed epochs end with zero live bytes. Addition falls from 545 allocations /
34,504 requested bytes / 5,584 peak live bytes to 527 / 21,200 / 5,584;
multiplication from 562 / 31,752 / 4,632 to 535 / 19,288 / 3,392; division
from 532 / 30,504 / 4,568 to 521 / 18,936 / 3,112; and the shared repeated
case from 828 / 41,920 / 5,232 to 814 / 29,168 / 5,184.

Four broader forms were measured and removed: direct-seeded affine
composition regressed division 1.45%; moving sorted interval endpoints
regressed both measured paths about 1.6%; borrowing both equal square-free
operands from one storage vector was not repeatably faster; and propagating
endpoint-order proof through division provided no benefit. The retained form
shrinks the default rlib 7,100 bytes (0.0552%), from 12,866,688 to 12,859,588.
Five permanent binary rows bring the Criterion catalog to 299 entries. Ten new
exact regressions bring default/all-feature library totals to 325/326; every
all-feature target, the no-default library, strict all-target Clippy,
warning-denied rustdoc, both release configurations, README, formatting, and
diff gates pass.

### Complete signed square-root images and canonicalize exact witnesses

`square_root_algebraic_root_representation` formerly trusted the source's
cached `Valid` status. As with the binary-image boundary, a subsequently
inverted stored isolator could therefore enter construction. The entry point
now strictly replays the complete source representation before classifying the
radicand or branch, and stale evidence returns `InvalidEvidence`.

The former image enclosure always rounded square-root endpoints at precision
`-128`. That is normally ample, but it is not a completeness proof: for source
roots `2 +/- 2^-200`, the isolator `[2,5/2]` selects only the upper source root,
while the downward-rounded lower image bound can still include both nearby
square-root images. A `NonUnitIsolation` result now remaps the original source
at successively doubled dyadic precision through `-4096`. The bound is explicit
and fail-closed; a separation of `2^-200` now transforms on both signed sheets,
while a `2^-4200` control still returns `NonIsolatingImageInterval`. A negative
outward lower rounding is clamped to exact zero because the mapped radical was
already proved nonnegative.

An exact-rational source witness supplies stronger evidence than its authored
carrier. Rational square roots now return the canonical linear exact-point
representation. Irrational square roots use the minimal `y^2-r` relation and
the signed half-axis enclosure directly: for `r>0`, that relation has exactly
one root on each selected half-axis, so a redundant general Sturm replay is not
needed. Generated numerator/denominator tests independently replay that unit
isolator, exact polynomial evaluation, interval containment, metadata, and
strict validation on both sheets. The general path still composes `P(y^2)`;
it builds the interleaved coefficient vector directly with checked length
arithmetic and moves the two dyadic bounds into final storage.

Pinned 100-sample release measurements are:

| Signed square-root image | Slice start | Retained | Change |
| --- | ---: | ---: | ---: |
| Ordinary `sqrt(sqrt(2))` | 10.973 us | 10.844 us | unchanged/noise-limited |
| Exact witness `sqrt(2)` from `(x-2)^6` | 25.146 us | 2.2477 us | -91.05% |
| Exact rational square from `(x-4)^6` | - | 1.1508 us | new canonical control |
| Neighboring images separated by `2^-200` | formerly failed | 55.159 us | new complete result |

Over 256 warmed operations, the ordinary path is allocation-identical at 219
allocations, 219 frees, 7 reallocations, 12,184 requested bytes, 6,216 peak
live bytes, and zero terminal live bytes. The repeated irrational-witness path
falls from 624 allocations / 32 reallocations / 25,296 requested bytes / 6,208
peak bytes to 32 / 3 / 1,616 / 568. The rational-square point remains one
allocation but falls from 624 requested/peak bytes to 96, and every epoch ends
with zero live bytes.

Three broader forms were removed. Testing for a rational payload before the
normal-form square check regressed the reversed exact-square measurement
3.75%. Retaining prepared radical endpoints across precision retries improved
the adversarial row about 1.63% but raised the ordinary peak by 232 bytes.
Zero-filling the composed polynomial instead of directly interleaving its
known slots offered no repeatable ordinary gain and performed unnecessary
overwrites. Reusing the already proved source-sign evidence remains because a
reversed comparison improved the ordinary path about 1.15% without changing
allocation behavior.

The default release rlib grows 38,038 bytes (0.296%), from 12,859,588 to
12,897,626; this is accepted for the strict-admission and bounded-completeness
repairs. Four permanent rows bring the Criterion catalog to 303 entries. Five
new exact tests bring default/all-feature library totals to 330/331. Every
all-feature target, the no-default library, fuzz-target compilation, strict
all-target Clippy, warning-denied rustdoc, both release configurations, README,
formatting, and diff gates pass.

### Complete unary polynomial images and streamline rational kernels

`transform_algebraic_root_polynomial_image` formerly trusted a cached `Valid`
source flag and admitted some storage that its exact-rational elimination
kernel could not faithfully consume. It now strictly replays the complete
source representation, then requires exact-rational source coefficients,
interval bounds, optional witness, and image coefficients. An empty authored
image is invalid rather than silently becoming zero, and an unsupported
nonrational trailing coefficient is rejected before exact-zero trimming can
hide it. Successful exact-witness transforms retain the caller's trimmed
authored image polynomial in the report.

Constant results now use one canonical linear exact-point constructor. In
addition to literal constants and rational source witnesses, a same-degree
relation `q(x) = scale * P(x) + constant` is detected directly from rational
payloads, so `P(x)=x^2-2, q(x)=x^2` returns the exact witness `2` instead of
the repeated eliminant `(y-2)^2`. The represented-root arithmetic caller also
exports that result through `ComputedExactRationalWitness`, matching the
report contract rather than wrapping a rational point as a general algebraic
representation. Generated relations independently perturb every supported
coefficient to guard against a false shortcut.

The raw checked Sylvester dimension still has the public bound eight. Only an
authored source/image pair beyond that bound triggers strict square-free
preparation and a second checked admission. Thus `(x^2-2)^4` with `q=x+x^2`
now completes through its quadratic carrier, while a square-free degree-seven
control remains `UnsupportedDegree`. The same prepared carrier is reused by
resultant construction and interval certification. The resultant fallback
asks the existing scalar exact-rational helper for determinant values instead
of constructing public reports that are immediately discarded.

Endpoint and exact-witness Horner evaluation, derivative construction,
interval Horner evaluation, product sorting, and sign checks now operate on
borrowed `Rational` payloads. Interval Horner starts at the leading
coefficient and handles its first multiplication with two endpoint products;
later uncertain-sign products retain all four combinations. Only final
interval endpoints cross back into `Real`. Generated endpoint/midpoint tests
compare those enclosures with independent exact scalar evaluation.

Pinned 100-sample release measurements are:

| Unary polynomial image | Slice start | Retained | Change |
| --- | ---: | ---: | ---: |
| Ordinary quadratic image | 3.7727 us | 3.1591 us | -16.26% |
| Constant image | 253.02 ns | 119.61 ns | -52.73% |
| Rational source witness | 664.04 ns | 534.52 ns | -19.51% |
| Exact modulus relation | 9.3888 us | 181.02 ns | -98.07% |
| Repeated degree-eight carrier | unsupported | 14.165 us | new complete result |
| Stationary image | 4.7175 us | 3.5402 us | -24.96% |
| Foreign-resultant-root refinement | 49.923 us | 47.867 us | -4.12% |

Over 256 warmed operations, ordinary allocation/free counts remain 102 with
zero reallocations; requested bytes move from 4,200 to 4,296 and peak live
storage remains 1,984 bytes. Constant images remain 2 allocations and 144
requested/peak bytes. Exact witnesses fall from 11 allocations and 880 bytes
to 10 and 832. The exact modulus relation falls from 257 allocations / 8,352
requested bytes / 2,344 peak bytes to 2 / 240 / 240. Stationary images remain
93 allocations and one reallocation, with requested bytes moving 4,112 to
4,160 and peak unchanged at 1,944. Foreign-root refinement falls from 965 to
959 allocations and 42,792 to 42,552 requested bytes; reallocations remain 3
and peak moves 10,352 to 10,448. The newly complete degree-eight case uses 401
allocations, 18 reallocations, 14,296 requested bytes, and 2,528 peak bytes.
Every epoch has paired frees and zero terminal live bytes.

An eager square-free variant was measured and removed. Preparing every source
at Sylvester dimension eight improved a repeated degree-six control from
47.949 to 12.080 us (-74.78%), but regressed the corresponding already
square-free control from 31.556 to 34.497 us (+9.58%). The retained threshold
therefore prepares only formerly unsupported inputs beyond the cap and keeps
the ordinary admitted schedule.

The complete slice grows the default release rlib 1,512 bytes (0.0117%), from
12,897,626 to 12,899,138. Eight permanent controls plus the newly measured
ordinary row bring the Criterion catalog to 312 entries. Seven new exact tests
bring default/all-feature library totals to 337/338. Every all-feature target,
the no-default library, fuzz-target compilation, strict all-target Clippy,
warning-denied rustdoc, both release configurations, README, formatting, and
diff gates pass.

### Complete Möbius images and keep rational kernels in payload form

`transform_algebraic_root_mobius` formerly trusted a cached `Valid` flag and
then rejected every defining polynomial containing a nonrational exact `Real`,
despite the public report and retained homogeneous construction using exact
`Real` coefficients. It now strictly replays the source carrier and admits the
general exact representation. A linear `x-sqrt(2)` carrier under
`(x+1)/(x+3)` is retained exactly, and fixed plus generated comparisons check
the in-place exact-`Real` Horner construction against the former independent
power-sum expansion. If exact trimming is undecided, that power-sum schedule
remains the authoritative fallback.

Exact rational witnesses no longer traverse homogeneous substitution and
return a scaled relation such as `4*y-7`. After determinant proof and a
denominator proof at the selected exact point, the image is evaluated directly
from borrowed `Rational` payloads and emitted as the canonical point relation
`y-7/4`.
Determinant sign, affine denominator endpoint signs, Möbius endpoint values,
and rational homogeneous Horner work likewise stay in `Rational` until the
final public values are constructed. Constant forward denominators retain one
scalar inverse-denominator power rather than growing a polynomial of structural
zeros.

The proved determinant sign also gives endpoint orientation because the
derivative has sign `sign((a*d-b*c)/(c*x+d)^2)`. The interval path therefore
moves the mapped endpoints directly instead of sorting them with another exact
comparison. Nonrational endpoint division first keeps the ordinary exact
division path, then uses the already certified nonzero denominator to construct
an inverse only when structural zero recognition was insufficient. Generated
intervals cover both denominator signs away from a pole, and a strict exact-
normal regression exercises the policy-certified fallback.

Pinned 100-sample release measurements are:

| Möbius image | Initial complete path | Retained | Change |
| --- | ---: | ---: | ---: |
| Ordinary shifted reciprocal | 2.0447 us | 1.1943 us | -41.59% |
| Exact rational witness | 2.1317 us | 643.07 ns | -69.83% |
| Constant denominator | 745.38 ns | 608.85 ns | -18.32% |
| Reciprocal | 527.94 ns | 516.70 ns | -2.13%, within noise |
| Exact-`Real` source | unsupported | 1.8336 us | new complete result; -44.22% from the first complete 3.2872 us path |

A faithful pre-change implementation and the retained public path were also
run for 256 warmed operations. Ordinary transforms fall from 23 allocations /
1,368 requested bytes / 840 peak bytes to 18/792/384. Constant-denominator
images fall from 8/768/576 to 2/168/168, reciprocal images from 8/768/576 to
3/192/192, and exact witnesses from 36/1,680/864 to 13/576/240. The newly
supported exact-`Real` row uses 45 allocations, 2,136 requested bytes, and 992
peak bytes. No row reallocates or ends with live bytes.

Two tempting specializations were measured and removed. Replacing the direct
rational determinant with aggregate product-sum ordering regressed the ordinary
row about 2.1%. Branching inside every linear Horner multiply improved one
reciprocal sample but reproducibly regressed the ordinary path; an immediate
rebuilt A/B also favored the compact unspecialized schedule for the reciprocal.

The complete slice grows the default release rlib 48,970 bytes (0.380%), from
12,899,138 to 12,948,108, and the all-feature rlib 65,216 bytes (0.500%), from
13,051,062 to 13,116,278. Four permanent rows bring the Criterion catalog to
316 entries. Four new exact tests bring default/all-feature library totals to
341/342. Default, all-feature, no-default, and all-target matrices pass, as do
fuzz-target compilation, strict all-target Clippy, warning-denied rustdoc, both
release configurations, README, formatting, and diff gates.

### Replace affine binomial expansion with homogeneous Horner

`transform_algebraic_root_affine` formerly trusted the cached validation flag
and expanded every source monomial independently. Each term recomputed scalar
and offset powers inside the binomial loop, and its binomial coefficient passed
through `u64` and then `i64`. High-degree transforms could therefore panic in
debug builds or silently wrap coefficients in release builds. The public entry
now strictly replays its source, and the polynomial kernel evaluates
`scale^n P((y-offset)/scale)` by the recurrence
`H = (y-offset)*H + p_k*scale^(n-k)`. Its complexity is quadratic in degree,
it has no machine-binomial bound, and its leading coefficient remains the
source leading coefficient.

Rational source/scalar storage uses one in-place `Vec<Rational>` recurrence and
converts only the finished coefficients to public `Real` values. General exact
`Real` coefficients and scalars use the same recurrence in `Real`. The former
independent expansion is compiled only as a test oracle. Fixed rational and
exact-`Real` comparisons plus generated rational transforms with both scale
signs and generated `r+s*sqrt(2)` coefficients require exact equality with that
oracle. A degree-128 `x^128-2` transform checks every coefficient of
`(y-1)^128-2` against independently generated `BigInt` binomials.

Exact point sources bypass degree-preserving substitution and return the
canonical linear point relation. That relation is replayed under `STRICT` in
debug builds. Rational scale signs are read directly from their payloads, and
the proved sign moves mapped interval endpoints into increasing order without a
second exact sort. A separate exact-`Real` scale/offset regression transforms
`x^2-2` by `sqrt(2)*x+pi` and checks the complete polynomial and interval.

Pinned 100-sample release measurements are:

| Affine image | Original expansion | Retained Horner | Change |
| --- | ---: | ---: | ---: |
| Ordinary `2*x+3` | 1.0997 us | 468.18 ns | -57.43% |
| Exact point witness | 841.05 ns | 290.95 ns | -65.41% |
| Exact-`Real` source coefficient | 1.3566 us | 774.50 ns | -42.91% |
| Sparse degree 16 | 77.414 us | 17.352 us | -77.58% |
| Unit-scale translation | not previously stored | 396.50 ns | new control |
| Zero-offset scaling | not previously stored | 313.74 ns | new control |
| Dense degree 16 | not previously stored | 17.619 us | new control |
| Mixed-scalar arithmetic handoff | not previously stored | 436.55 ns | new stored control |

In warmed allocation epochs, ordinary and scaling transforms move from one
allocation / 144 requested and peak bytes to 2/168/168. Translation moves from
4 allocations / 264 requested and peak bytes to 5/288/264. Exact witnesses
remain 1/96/96. The exact-`Real` row falls from 27/1,424/816 to 15/784/664;
sparse degree 16 falls from 253/13,224/3,144 to 2/952/952, and dense degree 16
from 814/44,640/3,168 to 2/952/952. No row reallocates or ends with live bytes.
The extra 24-byte vector allocation on small rational paths is retained for
the much larger timing win and the bounded high-degree allocation schedule.

A unit-scale loop split did not improve translation and regressed the ordinary
path about 6%, while a zero-offset shift specialization was timing-neutral; both
were removed. Conversely, asking whether every source coefficient was zero
before adding it was slower than letting the exact reducer consume the zero
product: removing that branch improved sparse and dense degree-16 rows 3.41%
and 7.86% in the immediate A/B.

The complete slice shrinks the default release rlib 5,438 bytes (0.0420%), from
12,948,108 to 12,942,670, and the all-feature rlib 17,430 bytes (0.133%), from
13,116,278 to 13,098,848. Six permanent controls plus first stored measurements
for the existing ordinary and mixed-scalar rows bring the Criterion catalog to
324 entries. Seven new exact tests bring default/all-feature library totals to
348/349. Default, all-feature, no-default, and all-target matrices pass, as do
fuzz-target compilation, strict all-target Clippy, warning-denied rustdoc, both
release configurations, README, formatting, and diff gates.

### Complete exact-point and same-root algebraic arithmetic

`arithmetic_algebraic_root_representations` formerly treated every stored
point as an exact *rational* witness, even when its `Real` payload was an exact
radical, constant, or computable expression. It also routed a missing binary
operand according to the left carrier, trusted cached validation on several
direct identities, and returned `Undecided` for zero divided by a certified
nonzero represented root. The report now has a distinct
`ComputedExactRealWitness` outcome. Exact point/point operations return that
outcome without manufacturing an algebraic representation when their result
lacks a stored rational payload; this matters for exact values such as `pi`
that need not be algebraic. Rational payloads retain the narrower rational-
witness status.

Direct point and identity routes replay their local input payloads under
`STRICT`, including a no-allocation canonical-linear check and an exact-
rational Horner fallback for noncanonical point storage. The upstream Sturm
report remains the uniqueness proof for interval-only carriers, matching the
documented representation contract. Missing right operands now report
`InvalidEvidence` before arithmetic routing. A general exact point without a
rational payload can serve as the scalar for supported affine and linear-
fractional lowering, and a policy-certified reciprocal completes exact-normal point division when
ordinary structural division reports an unknown-zero denominator.

Two identities no longer depend on a whole interval excluding zero.
`0/alpha` first proves the selected represented root nonzero and then returns
exact zero. `alpha/alpha` uses the same selected-root sign proof, so an
isolating interval may touch zero when its defining polynomial proves that
zero is not the selected root. Same-root addition lowers directly to the
affine image `2*alpha`. For a rational quadratic
`p0 + p1*alpha + p2*alpha^2 = 0`, squaring uses
`alpha^2 = -(p1/p2)*alpha - p0/p2`; the constant case returns a rational
witness and the general case reuses the affine image package instead of
constructing a resultant.

Affine and Mobius point images now preserve truthful metadata when an exact
`Real` scale, offset, or mapped source value is non-rational: their linear
relation is retained as a zero-width isolating representation, not labeled an
exact rational witness. Dedicated direct constructors remain on rational
point paths. After forcing the small Mobius constructor inline, its exact-
rational control measured 618.42 ns versus the previously closed 643.07 ns;
the ordinary rational-image control measured 5.7360 us versus 7.1102 us.

Pinned 100-sample release measurements, plus one immediate same-process A/B,
are:

| Arithmetic route | Previous behavior/time | Retained behavior/time | Change |
| --- | ---: | ---: | ---: |
| Rational point multiplication | 68.02 ns | 119.68 ns | +75.96%; strict point replay |
| Mixed rational scalar addition | 485.54 ns | 517.76 ns | +6.64% |
| Same `sqrt(2)` square | 269.57 ns | 103.45 ns | -61.63% |
| Same-root addition | 883.41 ns | 381.34 ns | -56.83% |
| Nonzero-linear quadratic square, paired A/B | 2.535--2.551 us | 344.5--352.0 ns | about -86% |
| Interval-root negation | 142.47 ns | 122.30 ns | -14.16% |
| Zero dividend | 90.67 ns, `Undecided` | 91.04 ns, exact zero | +0.41%, within noise |
| Self-division with a zero-touching interval | 76.37 ns, `Undecided` | 101.39 ns, exact one | +32.76% for a complete result |
| General exact-point addition | 224.24 ns, mislabeled rational | 393.80 ns, exact-`Real` status | +75.61% for truthful replay |
| Mixed exact-`Real` scalar addition | 3.6717 us | 3.7774 us | +2.88% |
| Exact-normal point division | `Undecided`, not stored | 10.763 us, exact-`Real` status | new complete result |

Allocation accounting explains the fast quadratic routes. Rational point
multiplication remains allocation-free. The `sqrt(2)` square core falls from
two allocations / 240 requested bytes to one / 63, and the nonzero-linear
quadratic square falls from 94/3,776 with 1,984 peak bytes to 3/232 with 208
peak bytes. Zero-dividend construction falls from one 57-byte allocation to
zero. Truthful exact-`Real` point addition uses 9 allocations / 520 requested
bytes / 296 peak bytes; the discarded representation-producing form used
11/686/462. Mixed exact-`Real` scalar addition adds one allocation and 72
requested bytes (100/5,799 versus 99/5,727) without increasing its 1,920-byte
peak. Completing the exact-normal division necessarily raises the former
failed-return profile from 49/2,176/1,208 to 260/13,776/3,184. No measured row
reallocates or ends with live bytes.

A full Sturm recount on every interval-only identity was measured and removed.
It changed the layer's documented trust boundary, rejected valid endpoint-root
carriers used by the structural representation API, and moved the small direct
rows from roughly 0.1 us to 0.3--0.5 us. A single shared point constructor was
also removed from rational affine, Mobius, and rational-image hot paths after
same-process controls exposed extra payload classification; split rational and
exact-`Real` constructors preserve both truthful metadata and the closed
rational timings.

The complete slice grows the default release rlib 62,884 bytes (0.486%), from
12,942,670 to 13,005,554, and the all-feature rlib 34,136 bytes (0.261%), from
13,098,848 to 13,132,984. Ten newly stored arithmetic measurements bring the
Criterion catalog to 334 entries. Ten exact regressions bring default/all-
feature library totals to 358/359. Default, all-feature, no-default, and all-
target matrices pass, as do fuzz-target compilation, strict all-target Clippy,
warning-denied rustdoc, both release configurations, README, formatting, and
diff gates.

### Exact algebraic polynomial and rational-expression evaluation

The evaluation entry points formerly trusted a cached-valid flag, classified
every exact point result as rational, cloned the complete coefficient vector
before trimming, and ran general `Real` interval products even when every
endpoint and coefficient already had a rational payload. Rational-expression
evaluation also replayed the root once per polynomial, reproved denominator
signs around inversion, sorted the two images of a monotone reciprocal, and
constructed avoidable reciprocals for several exact identities.

The retained entry path strictly replays the local root payload once, then
shares that admission between numerator and denominator. Exact-point reports
now distinguish a stored rational payload from a general exact `Real` value;
the latter status describes representation, not a proof of mathematical
irrationality. `exact_point_witness` is the sole point-carrier query; callers
that genuinely need a rational payload inspect `Real::exact_rational_ref`.
Rational-image and local-field consumers handle both evaluation statuses
without manufacturing rational metadata.

Coefficient trimming borrows the surviving prefix. Point evaluation delegates
to Hyperreal's rational/symbolic polynomial reducer, while interval evaluation
starts from the real leading coefficient instead of a synthetic zero. A
rational interval Horner path keeps coefficients and endpoints in `Rational`;
its sign-partitioned product uses two endpoint products except when both
intervals straddle zero. Quotient intervals reuse the denominator report's
certified side, reverse reciprocal endpoints by monotonicity, use the same
rational product kernel, and derive the quotient sign from existing numerator
and denominator evidence when possible. Exact quotients directly handle zero
numerators, structurally identical nonzero values, unit denominators, and two
rational payloads before the complete policy-certified reciprocal fallback.

Pinned 100-sample release slopes are:

| Evaluation route | Slice start | Retained | Change |
| --- | ---: | ---: | ---: |
| Polynomial at interval root | 955.56 ns | 365.78 ns | -61.72% |
| Polynomial at rational point | 389.35 ns | 261.86 ns | -32.74% |
| Polynomial at exact-`Real` point | 545.15 ns | 470.49 ns | -13.70% |
| Degree-16 interval polynomial | 6.5403 us | 2.7280 us | -58.29% |
| Exact-normal constant polynomial | new control | 6.8637 us | complete strict sign |
| Rational expression at interval root | 1.8663 us | 707.75 ns | -62.08% |
| Rational expression at rational point | 715.18 ns | 457.31 ns | -36.06% |
| Rational expression at exact-`Real` point | 1.0002 us | 708.63 ns | -29.15% |
| Structurally identical exact values | new control | 710.20 ns | exact one |
| Exact-normal denominator | 15.886 us reconstructed old path | 9.0858 us | -42.89% |
| Zero over exact-normal denominator | new control | 7.1016 us | 21.84% below nonzero quotient |

The initially stored 602.41 ns exact-normal denominator sample was rejected as
a contaminated baseline: it did not reproduce with independent fixtures. A
temporary external, non-inlined reconstruction measured the old public
polynomial stage at 6.934 us versus 7.016 us retained, and the complete old
rational-expression path at 15.886 us versus 9.073 us retained in the same
process. Every diagnostic hook and file was then removed; each permanent
exact-normal benchmark now owns an independent fixture.

A temporary 128-iteration allocation census measured 126 allocations / 5,568
requested bytes / 408 peak live bytes for the degree-16 interval polynomial,
11/712/712 for the interval rational expression, 6/240/240 for the rational-
point quadratic, and 3/120/120 for its rational expression. No row reallocated
or ended with live bytes. The permanent dispatch trace's current
`algebraic_evaluation` workload records 90 dependency events, including 63
exact reducers, 66 rational temporaries, and seven predicate events, with no
approximation or refinement event. The reduction from the slice-closing trace
comes from the later allocation-free structural-negation replay described
below.

Exact regressions cover stale point evidence, general exact `Real` status,
defining-polynomial cancellation to rational zero, same-value quotient
cancellation to rational one, exact-normal policy division and zero numerator,
negative denominator orientation, and a generated all-sign rational interval-
product oracle. Eleven stored timing rows bring the Criterion catalog to 344;
default/all-feature library totals are 361/362. Clean isolated release builds
produce 13,019,768-byte default and 13,160,342-byte all-feature rlibs, increases
of 14,214 bytes (0.109%) and 27,358 bytes (0.208%) from the preceding arithmetic
slice. The closing matrix passes all-target/all-feature tests, default and
no-default-feature library tests, clippy with warnings denied, rustdoc with
warnings denied, every fuzz binary, formatting, and diff hygiene.

### Strict algebraic-root comparison, refinement, and sign

The comparison entry points formerly trusted a cached-valid flag. Direct
orders then used stored interval bounds even when an exact witness identified
a narrower selected point; refinement revalidated both inputs on every round
and invoked the Sturm refiner even for an already exact operand. Difference
comparison recomputed known-disjoint common-root evidence, constructed a
difference after direct orders, lost `SameRepresentation` status, and fed the
original rather than narrowed intervals to its final construction. Public sign
queries likewise trusted cached validation and created generic zero/comparison
work for rational payloads.

The retained boundary replays both inputs under `STRICT` once. Exact witnesses
act as both effective bounds, exact operands are skipped by the refinement
loop, and any interval-refinement failure still terminates undecided. Direct
orders and self identity return without GCD or difference construction. An
exact point is matched to an interval by exact polynomial evaluation at that
point; two interval carriers retain the polynomial-GCD plus Sturm-count proof.
An already proved `false` common-root result is monotone under interval
narrowing and is not recomputed. If narrowed intervals still overlap, those
narrowed carriers feed the bounded difference constructor. Rational endpoint,
point, and sign decisions stay in `Rational`; Hyperreal's new structural
opposite-scale certificate replays canonical exact-`Real` linear points without
constructing a temporary negation.

This is deliberately not ExactCore's complete `AlgebraicNum` package. The
audited implementation in `AlgebraicNum.h` recomputes interval root bounds in
`valid()`, repeatedly refines zero-touching intervals in `checkZero`, proves
equality by polynomial GCD plus a common interval root bound, and orders with a
constructed difference. Hypersolve adopts the transferable selected-root and
GCD model, but keeps its documented upstream uniqueness authority and bounded
reports. A full Sturm recount was already measured at roughly 3--5x on small
rows and rejected because it also invalidated supported endpoint carriers.
Continuing after either requested interval refinement fails was rejected: a
failed Sturm recount can contradict the stored isolation proof. ExactCore's
unbounded refinement/difference behavior and a prepared Sturm carrier were not
introduced into this bounded orchestration slice.

Pinned Criterion means expose both the safety cost and the retained wins:

| Comparison/sign route | Slice start | Retained | Change |
| --- | ---: | ---: | ---: |
| Direct overlapping intervals | 23.932 ns | 42.01 ns | +75.5% |
| Direct disjoint intervals | 7.7365 ns | 25.26 ns | +226.5% |
| Direct rational points | 10.569 ns | 51.19 ns | +384.3% |
| Direct exact-`Real` points | 848.64 ns | 945.03 ns | +11.4% |
| Refinement comparison | 28.450 us | 27.60 us | -3.0% |
| Exact-`Real` coefficient refinement | 12.369 us | 12.58 us | +1.7% |
| Difference-backed overlap | 23.999 us | 21.68 us | -9.7% |
| Interval sign | 17.035 ns | 12.34 ns | -27.6% |
| Rational-point sign | 9.2525 ns | 22.54 ns | +143.6% |
| Exact-`Real` point sign | 29.460 ns | 58.48 ns | +98.5% |

The increased direct and point-sign costs are the measured price of rejecting
stale public evidence instead of trusting a mutable cached status. The first
strict exact-`Real` comparison/sign implementations measured 1.091 us and
139.8 ns; structural negation recovered 13.4% and 58.2% respectively. New
sentinels measure 57.25 ns for a witness inside wider stored bounds, 8.48 us
for one exact point against one refined interval, 296.98 ns for a direct
difference-API disjoint order, 317.83 ns for its rational-point order,
313.51 ns for self identity, and 473.43 ns for exact-point/common-isolator
equality.

A warmed 128-iteration allocation census records one 51-byte message
allocation for the expected-overlap direct report and zero allocations for
disjoint and rational-point comparison. Exact-`Real` point comparison uses 21
allocations/1,176 bytes per call; refinement uses 532/26,065 plus six
reallocations, and difference-backed overlap uses 622/24,843 plus eight.
Interval, rational-point, and exact-`Real` sign are all allocation-free, and no
epoch ends with live bytes. The expanded `algebraic_comparison` trace records
2,000 dependency events, 238 predicates, 1,351 exact reducers, 706 rational
temporaries, one rational GCD, and 58 cache events. It contains no approximation
or precision-refinement event; its six fallback-classified events are exact
`Real` difference-sign decisions.

Six regressions bring default/all-feature library totals to 367/368, and 16
stored comparison/sign rows bring the Criterion catalog to 360 entries. Clean
isolated default/all-feature rlibs are 13,025,626 and 13,168,228 bytes, increases
of 5,858 (0.045%) and 7,886 (0.060%) from the preceding evaluation slice.
Default, no-default, all-feature, every all-feature target and benchmark smoke,
strict Clippy, warning-denied rustdoc, both release configurations, every fuzz
binary, formatting, and diff hygiene pass. Hyperreal independently passes
610/690 library tests and its complete corresponding matrix. Hypercurve remains
excluded at the user's direction during concurrent editing.

### Validate once and refine isolated roots by defining-polynomial sign

The public isolated-root refiner formerly trusted any stored exact witness,
cloned coefficients before trimming, rebuilt square-free and Sturm evidence
for every non-witness call, reevaluated the defining polynomial alongside the
whole chain, and recounted that chain at every midpoint. It also described its
endpoints as rational even though the implementation and represented-root
callers support general exact `Real` coefficient fields and endpoints.

The retained boundary first proves a nonconstant polynomial, ordered endpoints,
the claimed distinct-root count, and—when present—closed witness containment,
polynomial vanishing, and interval uniqueness. A witnessed singleton and a
valid linear witness are then complete proofs. An unwitnessed linear carrier is
solved exactly with one certified reciprocal and accepted only in its owned
`(lower, upper]` interval. For degree at most two, nonzero opposite endpoint
signs already prove exactly one interior root. Only the remaining cases build
the square-free Sturm chain. Initial chain evaluation now returns both the
variation count and defining-polynomial sign, and after that one-root proof the
bisection loop needs only the defining polynomial against the stable nonroot
upper sign. Exact midpoints use `Real::average_pair`, coefficient trimming and
width subtraction borrow their inputs, and every unresolved predicate retains
an explicit `Undecided` report.

Endpoint ownership is intentionally narrower than ExactCore's closed count.
Hypersolve's positive-width partitioned isolators own `(lower, upper]`; changing
that convention selected an extraneous cardinal conjugate in the existing
tensor-image sentinel. Explicit witnesses instead own their point, including a
lower endpoint, and therefore request a closed recount when the fast degree
proof is insufficient. This rejects a wide nonlinear witness if the opposite
endpoint contributes another root. ExactCore's transferable design is the
single Sturm uniqueness proof followed by defining-polynomial-sign bisection;
its unbounded refinement and globally closed ownership were not imported. A
cross-call prepared carrier would extend represented-root state and memory for
evidence already cheap on the common degree-two path, so it remains outside
this bounded slice.

Pinned Criterion means show the scheduling gain:

| Refinement route | Slice start | Retained | Change |
| --- | ---: | ---: | ---: |
| Quadratic, four requested steps | 8.3271 us | 957.29 ns | -88.52% |
| Repeated quadratic, four requested steps | 13.526 us | 9.47 us | -29.84% |
| Exact-`Real` linear carrier | 17.042 us | 2.07 us | -87.83% |
| Width already satisfied | 3.6468 us | 461.70 ns | -87.32% |
| Exact linear witness | 144.53 ns | 230.50 ns | +59.28% |
| Exact quadratic witness | 145.27 ns | 290.36 ns | +100.78% |

The sub-0.30-us witness increases are the cost of replacing blind trust with
strict replay. New controls measure 496.64 ns for a wide exact quadratic
witness, 3.59 us for rejection of a lower-only unwitnessed endpoint root, and
3.97 us for exact upper-endpoint canonicalization. The downstream represented-
root comparison benchmark falls from 27.60 to 3.34 us; its one-sided
exact-`Real` carrier falls from 12.58 to 4.36 us, and exact-point-versus-interval
comparison falls from 8.48 to 3.89 us.

A warmed allocation census records 3 allocations/120 requested bytes for the
ordinary quadratic, 233 allocations plus 7 reallocations/8,408 requested bytes
for its repeated-factor fallback, zero allocations for linear and quadratic
exact-witness paths, 3/120 for a wide exact witness, 57/3,272 for the
exact-`Real` linear carrier, and 6/240 when the width is already satisfied. No
probe ends with live bytes. The same permanent comparison dispatch workload
falls from 2,000 to 771 dependency events, 238 to 100 predicates, 1,351 to 542
exact reducers, and 706 to 296 rational temporaries; its rational GCD remains
one, exact fallback events remain six, cache events fall from 58 to 44, and no
approximation or precision-refinement event appears.

Nine focused regressions cover reversed/collapsed intervals, false/outside and
wide witnesses, both endpoint conventions, exact midpoint discovery, trimmed
linear carriers, exact step/width budgets including an unreachable negative
width, general exact-`Real` coefficients, generated repeated-root refinement,
and generated half-open quadratic counts. The tensor-image ownership sentinel
also passes under all features. Nine stored refinement rows bring the Criterion
catalog to 369 entries. Default/no-default library tests pass 373 and the
all-feature library passes 374. Isolated default/all-feature release rlibs are
13,059,632 and 13,204,880 bytes, increases of 34,006 (0.261%) and 36,652
(0.278%) from the preceding comparison slice. The complete all-feature target
matrix and benchmark smoke, every fuzz binary, strict Clippy, warning-denied
rustdoc, both release configurations, formatting, and diff hygiene pass.

### Certify interval root presence without losing endpoint ownership

The interval-presence helper now distinguishes its public open interval from
the partition-owned half-open intervals used by algebraic images. The public
query counts roots in `(lower, upper)` and excludes either endpoint even when
another root is interior. Image enclosures count `(lower, upper]`, matching the
root isolator's positive-width partition convention. This repairs two
independent defects: an endpoint root formerly caused the public helper to
discard every interior root, while image admission used open ownership and
could accept or reject the wrong root count at the included upper boundary.

Bernstein variation is again used only as a one-sided Descartes bound. The
cubic `-2 + 9*x - 15*x^2 + 10*x^3` has Bernstein controls `[-2, 1, -1, 2]` on
`[0,1]`, hence variation three, but its strictly positive derivative and exact
Sturm count prove that `x=1/2` is its sole root. The old helper returned false
as soon as the square-free variation remained greater than one; the retained
path falls through to Sturm. This agrees with Exact Core's
`shiftAndSigncount`, which reports its greater-than-one result as inconclusive,
not as a root-count disproof. Exact Core's closed `Sturm::numberOfRoots` also
special-cases endpoint roots; Hypersolve instead keeps its native half-open
Sturm count and performs the requested open/half-open endpoint adjustment
explicitly.

The exact schedule is now endpoint evaluation, a degree-two sign-crossing
proof, a streamed derivative interval for nonquadratic monotonicity, exact
Bernstein variation, square-free reduction, and Sturm only when every cheaper
certificate remains inconclusive. A variation-two quadratic is complete in
constant space: for `a(x-r)^2`, the three interval Bernstein controls have
signs proportional to `(lower-r)^2`, `(lower-r)(upper-r)`, and
`(upper-r)^2`. A zero discriminant therefore makes variation two equivalent to
one repeated interior root; a nonzero discriminant cannot leave exactly one
distinct root. The specialized path reuses the endpoint signs already proved
by its caller and uses borrowed arithmetic for the middle control.

Exact-rational Horner interval evaluation stays in `Rational` payloads. Its
streamed derivative does not allocate a derivative vector, and the shared
sign-partitioned interval product forms only the extremal products except when
both inputs straddle zero. The same product kernel now serves algebraic value
intervals and root presence. An isolated degree-16 algebraic-evaluation A/B was
noise-neutral at 2.9407 versus 2.8957 us; centralizing the identical proof and
removing duplicate source, rather than claiming that noise as a speedup, is the
retained benefit.

Pinned Criterion comparisons cover every proof depth:

| Root-presence route | Correctness-fixed baseline | Retained | Change |
| --- | ---: | ---: | ---: |
| Quadratic endpoint signs | 273.15 ns | 101.43 ns | -62.86% |
| Monotone degree 16 | 3.3035 us | 904.97 ns | -72.77% |
| Bernstein degree 16 | 21.319 us | 18.467 us | -13.12% |
| Repeated rational quadratic | 5.9037 us | 497.90 ns | -91.53% |
| Repeated exact-`Real` quadratic | 12.920 us | 5.3709 us | -58.46% |
| Inconclusive-Bernstein cubic | 11.265 us | 8.3547 us | -26.07% |
| Endpoint plus interior root | 842.49 ns | 227.74 ns | -72.90% |

Algebraic-image admission now policy-trims and rejects constant image
polynomials before accepting an interval. A supplied exact image witness must
be inside the closed enclosure and exactly annihilate that polynomial; valid
witnesses are moved from the owned interval and canonicalized to a collapsed
point, while stale outside or nonvanishing evidence fails closed. Ordinary
enclosures retain the half-open count and can request at most eight batches of
eight source-refinement steps. The first enclosure borrows the caller's source
interval rather than cloning it eagerly.

Against the pre-admission baseline, an exact polynomial-image witness measures
442.00 versus 471.21 ns (-6.25%), and the ordinary rational image is
noise-neutral at 4.6801 versus 4.7260 us (-1.12%). The ordinary polynomial
image is 2.9596 versus 2.8817 us (+2.68%); this small aggregate cost remains
for strict nonconstant image admission and is accepted for the repaired
failure boundary. A same-layout ownership A/B puts the retained witness move
at 448.56 ns and the redundant clone at 453.29 ns (+1.36%, below the practical
noise threshold). A proposed cross-round square-free/Sturm cache increased the
foreign-root refinement case from 30.485 to 31.627 us (+2.73%, `p<0.01`), so it
was removed; the restored one-shot path is statistically unchanged at 30.917
us.

A warmed 32-call allocator probe records zero allocations for the quadratic
sign and endpoint/interior paths; 3 allocations/120 requested bytes for both
the monotone degree-16 and repeated-rational paths; 828 allocations, 20
reallocations, 17,352 requested bytes, and 2,312 peak live bytes for the full
degree-16 Bernstein conversion; 112/5,880/2,696 for the repeated exact-`Real`
path; 225 allocations, 2 reallocations, 8,264 requested bytes, and 4,088 peak
bytes for the cubic Sturm fallback; and 10/832/832 for an exact polynomial
image witness. Every epoch frees all measured bytes. The roots dispatch
workload now includes monotone and repeated-quadratic presence proofs: it
records 918 dependency events, 80 predicates, 554 exact reducers, 420 rational
temporaries, one rational reduction, and no approximation or precision-
refinement event.

Fixed regressions cover the cubic false negative, all endpoint combinations,
half-open image ownership, constant image rejection, canonical valid witnesses,
stale outside/nonvanishing witnesses, exact-`Real` coefficients, unresolved
identities, and all variation-two quadratic outcomes. A generated oracle uses
repeated roots and rational bounds with denominators one through five and
matches exact open, half-open, and closed-empty distinct-root sets. Seven
permanent timings are stored in the aggregate Criterion catalogue. The current
default/all-feature release rlibs are 13,073,832 and 13,208,192 bytes,
increases of 14,200 (0.109%) and 3,312 (0.025%) from the refinement slice.
Default/no-default/all-feature library tests pass 379/379/380, as do the
complete all-feature target matrix and benchmark smoke, fuzz compilation and
lint, strict all-target Clippy, warning-denied rustdoc, dispatch regeneration,
both release configurations, formatting, and diff hygiene.

### Complete rational algebraic-root images and share domain evidence

Rational-image construction now treats the authored denominator as the domain
authority before either common-factor cancellation or source-relation
reduction. An exact point evaluates it directly; an interval carrier computes
an exact polynomial GCD and counts common roots with the isolator's
`(lower, upper]` ownership. A selected common root reports
`CertifiedZeroDenominator`, including when numerator and denominator would
cancel algebraically. If interval Horner merely broadens a nonzero denominator
through zero, at most eight rounds of eight exact source-refinement steps try
to separate its sign before preserving the original
`DenominatorMayContainZero` failure. This completes both an excluded lower-
endpoint pole beside `sqrt(2)` and the dependency-broadened positive
denominator `x^2 - 2*x + 2` without sampling or changing the bounded failure
contract.

After that authored-domain proof, exact-rational numerator and denominator
polynomials are reduced modulo the primitive defining relation. The smaller
map is normalized again before the Sylvester dimension check, so `x^2` at
`sqrt(2)` becomes the exact constant two and a degree-20 authored map whose
remainder is linear no longer fails the pre-reduction budget. Retained
`AlgebraicRootRationalMap` objects cache this reduced map only for their exact
configured source polynomial; immediate transforms keep it on the stack. A
borrowed cross-product proportionality test recognizes high-degree constant
rational maps before polynomial GCD or division.

Constant and degree-one maps no longer inherit the nonlinear exact-rational
coefficient restriction. They reuse the established general exact-`Real`
Möbius construction, and canonical point construction preserves rational
versus nonrational metadata. For a decreasing Möbius map, an exact source
endpoint root triggers bounded source refinement before endpoint reversal, so
the source isolator's half-open ownership is not silently inverted. Linear
image relations are solved directly; quadratic relations use their exact
discriminant to promote only a repeated root, leaving distinct quadratics
unchanged without a square-free GCD.

Fixed-size batches lazily cache one denominator-domain result. Invalid
numerators retain their own report precedence, a zero-sized batch performs no
evaluation, and only the first numerator that actually encounters an interval
zero requests the exact GCD/refinement path. The refined source carrier and
denominator report are then reused in input order by every remaining image.
The rational evaluator likewise reuses the denominator's strict source
admission instead of validating the same root for the numerator. Direct
target-interval rejection still precedes elimination and leaves every retained
construction cache cold, but it cannot hide a true pole.

Exact-rational interval products use the shared sign-partitioned payload
kernel, quotient enclosures reverse reciprocal endpoints explicitly, and
polynomial interval Horner starts from the leading coefficient. Exact source
witnesses collapse after one rational evaluation. Derivative construction is
deferred until neither constant nor Möbius reduction applies, and a constant
denominator omits the identically zero quotient-rule product.

Pinned means use the relevant pre-change baseline for each retained layer:

| Rational-image route | Baseline | Retained | Change |
| --- | ---: | ---: | ---: |
| Ordinary linear fractional | 4.8132 us | 2.9337 us | -39.05% |
| Four-map linear batch | 16.398 us | 12.519 us | -23.65% |
| Four-map dependency-denominator batch | 55.360 us | 36.223 us | -34.57% |
| Proportional degree-six constant | 8.7628 us | 3.6117 us | -58.78% |
| Quadratic reduced by source relation | 10.351 us | 2.0545 us | -80.15% |
| Stationary cubic reduced by source relation | 6.5982 us | 3.7333 us | -43.42% |
| Dependency-broadened denominator | 14.238 us | 10.154 us | -28.68% |
| Certified algebraic pole | 1.2031 us | 1.2111 us | +0.66% |
| Shared monic linear factor | 20.025 us | 6.9169 us | -65.46% |
| Shared nonmonic linear factor | 20.559 us | 7.5076 us | -63.48% |
| Retained rational map | 4.0083 us | 2.2472 us | -43.94% |
| Degree-12 source/cubic map | 2.0196 ms | 1.9638 ms | -2.76% |

The pole row's sub-percent drift is accepted for the new exact status; every
ordinary construction control improves. A warmed allocator census records
allocation/requested/peak-live bytes of 43/2,264/1,680 for the ordinary image,
137/7,056/3,648 for its four-map batch, 43/3,328/1,408 for the degree-six
constant, 36/2,488/1,856 for the reduced quadratic, 61/3,792/2,264 for the
stationary cubic, 174/6,400/2,384 for the shared-factor map,
186/10,273/6,448 for the dependency denominator, 691/28,079/13,491 for its
four-map batch, and 13/790/497 for the pole. No row reallocates or ends with
live bytes. Relative to four singleton calls, the two batches save 35
allocations/2,000 requested bytes and 53 allocations/13,013 requested bytes.

Three broader schedules were removed. Reusing derivative-sign endpoint work
was noise-neutral (+0.38%, `p=.18`); an explicit unit-denominator derivative
branch regressed its stationary control 1.19%; and carrying a runtime
`domain_checked` flag through every batch image regressed the ordinary batch
3.24%, so domain establishment and admitted transformation remain separate
functions. A fresh boxed-versus-stack A/B favors stack ownership of immediate
source-reduced maps by 1.74%, 3.48%, and 1.24% on the reduced quadratic,
stationary, and shared-factor controls while also removing one allocation and
200 requested bytes.

Fourteen new fixed/generated regressions bring default/no-default/all-feature
library totals to 393/393/394. They cover exact-`Real` sources and map
coefficients, truthful point metadata, decreasing endpoint ownership, true and
excluded-endpoint poles, dependency refinement, cancellation-before-domain
authority, batch order/equivalence/no-op behavior, source-relation and
over-budget reduction, retained-cache/target laziness, status propagation, and
generated witnesses/shared factors/batches. The dedicated dispatch workload
records 975 dependency events, 112 predicates, 819 exact reducers, 311
rational temporaries, 79 cache events, and no approximation, precision
refinement, or fallback event. The generated Criterion catalogue contains 385
rows. Isolated default/all-feature rlibs are 13,132,898 and 13,288,296 bytes,
increases of 59,066 (0.452%) and 80,104 (0.606%) from the interval-admission
slice. The complete all-feature target/benchmark matrix, fuzz compilation and
strict lint, all-target Clippy, warning-denied rustdoc, dispatch regeneration,
both release configurations, formatting, and diff hygiene pass. Three
semantics-identical contiguous integer match arms in pre-existing fuzz targets
were expressed as ranges to satisfy rustc 1.97's strict lint. Hypercurve
remained untouched.

### Preserve reversed interval ownership and separate foreign Mobius poles

Positive-width represented-root intervals own `(lower, upper]`. The standalone
affine and Mobius transforms previously reversed only the numeric endpoints of
a decreasing image. If the defining polynomial also vanished at a stored
endpoint, this changed ownership: the image could include a foreign source
lower root or exclude the selected upper root. Both transforms now share a
bounded ownership-refinement path. It performs at most eight rounds of eight
exact root-refinement steps, revalidates every progressed carrier, and either
removes both endpoint roots or collapses the selected root to an exact point
before reversing the interval.

The common small-integer affine path first evaluates the defining polynomial
at both integer endpoints modulo the 32-bit prime `4_294_967_291`. Two nonzero
residues are an exact nonvanishing certificate, so the ordinary reflected path
adds no allocations. An inapplicable or zero modular image is only
inconclusive and falls through to authoritative exact `Real` polynomial
evaluation; a generated integer-Horner oracle and a deliberate prime-collision
case guard that boundary. The shared generic endpoint test and all refinement
ownership live out of line, and only the rare path clones a boxed carrier.

Mobius domain admission now distinguishes a pole at the selected algebraic
root from a foreign pole elsewhere in the stored enclosure. When interval
evaluation includes zero, exact polynomial GCD/root-selection evidence first
decides whether the linear denominator vanishes at the owned root. A selected
pole retains `DenominatorMayVanish`; a proved foreign pole receives the same
bounded exact source refinement until the enclosure excludes zero. Exact point
witnesses prove the denominator only at that point, so an irrelevant pole in
their wider retained bounds no longer rejects a valid image. The rational-image
wrapper now delegates its linear-fractional case directly to this standalone
owner; its duplicate orientation and ownership refiners were removed.

Pinned Criterion means are:

| Standalone transform route | Slice start | Retained | Change |
| --- | ---: | ---: | ---: |
| Negative affine scaling | 309.62 ns | 326.33 ns | +5.40%; exact ownership proof |
| Reciprocal Mobius image | 512.04 ns | 566.06 ns | +10.55%; exact ownership proof |
| Exact Mobius witness | 618.42 ns | 527.66 ns | -14.68% |
| Negative affine endpoint refinement | unavailable/incorrect | 12.35 us | new correct result |
| Reversed Mobius endpoint refinement | unavailable/incorrect | 12.97 us | new correct result |
| Excluded-endpoint pole refinement | rejected | 4.65 us | new complete result |

Increasing affine scaling remains 320.63 ns, the ordinary Mobius row remains
1.20 us, and its constant-denominator row is 598.52 ns. A warmed allocation
census records 2 calls / 168 requested and peak bytes for ordinary negative
affine scaling and 3/192/192 for the reciprocal, exactly matching their
established controls. The affine and Mobius endpoint-refinement rows each use
241 calls, one reallocation, 12,776 requested bytes, and 5,904 peak bytes; the
foreign-pole row uses 84 calls, no reallocations, 3,984 requested bytes, and
2,168 peak bytes. Every row ends with zero live bytes.

Two broader endpoint schedules were removed. Replaying both endpoints through
generic exact `Real` evaluation raised negative affine scaling to 542.66 ns
and reciprocal Mobius to 746.30 ns, about 76% and 46%. A low-degree aggregate
rational prototype reduced those figures to about 403.5 and 640.8 ns, but
generated large per-shape monomorphizations and inflated the default rlib to
13,235,230 bytes. The retained modular certificate is both faster and smaller.

Nine fixed/generated regressions bring default/no-default/all-feature library
totals to 402/402/403. They cover foreign lower roots, selected upper roots,
modular collisions, general exact-`Real` endpoints, excluded and interior
foreign poles, selected poles, and exact points with irrelevant outer poles;
zero-step independent root refinement validates each transformed interval.
Four permanent rows bring the generated Criterion catalogue to 389. The
rational-image dispatch harness falls from 975 to 926 dependency events, 112
to 107 predicates, 819 to 779 exact reducers, 311 to 301 rational temporaries,
and 79 to 76 cache events, with no approximation, precision-refinement, or
fallback event.

The default/all-feature release rlibs are 13,164,358 and 13,298,428 bytes,
increases of 31,460 (0.240%) and 10,132 (0.076%) from the rational-image slice.
Exact Core's refine-before-interval-arithmetic discipline transfers; its
globally closed interval model and unbounded refinement loops do not. Default,
no-default, all-feature, all-target, benchmark, fuzz compile/lint, strict
Clippy, warning-denied rustdoc, dispatch regeneration, both release,
formatting, and diff gates pass.

## Representation, coverage, fuzz, and memory gates

The dedicated representation matrix inventories all 22 optimized finite
Hyperreal certificate classes currently constructible through the public API:
`One`, `Pi`, `PiPow`, `PiInv`, `PiExp`, `PiInvExp`, `PiSqrt`,
`ConstProduct`, `ConstOffset`, `ConstProductSqrt`, `Sqrt`, `Exp`, `Ln`,
`LnAffine`, `LnProduct`, `Log10`, `Log2`, `Pow10`, `Pow2`, `SinPi`,
`TanPi`, and `Irrational`. The recipes are checked against their certificate
names and cover all eight public `StructuralKind` values. Every class crosses
problem analysis, tree and retained evaluation, symbolic Jacobians, candidate
and domain certification, predicates, direct affine solving, dense Bareiss
solving and proof replay, and both materialized and pattern-preserving sparse
surfaces. A separate 22 by 22 ordered-pair matrix exercises arithmetic,
expression evaluation, and dense and sparse replay. Deterministic tests and the
fuzz target also build shared opaque computable DAGs with variable depth and
topology, covering the open-ended representation beyond the finite optimized
certificate set.

`scripts/representation_coverage.sh` runs that matrix with neither primitive
cache, the f32 cache, the f64 cache, and both caches plus all Hypersolve
features. `scripts/coverage.sh` merges the no-feature, dispatch-trace, and
all-target/benchmark-fixture configurations with LLVM source coverage. The
2026-08-30 run covered 29,629 of 34,790 production executable lines (85.17%)
after excluding trailing inline test modules. The complete instrumented source
report, which includes inline test code, covered 87.45% of lines and executed
94.52% of functions. CI enforces an 85% production-line floor and emits both
annotated text and HTML reports.

The address-sanitized representation fuzzer completed 1,000 retained
executions with no finding, reaching 7,906 coverage points, 17,402 features,
and 290 corpus entries with 529 MiB peak process RSS. Leak detection alone was
disabled because LeakSanitizer cannot run under this host's ptrace policy; the
AddressSanitizer instrumentation remained active. Independent allocation
accounting warms lazy exact-real refinement caches and then measures all 66
certificate/operation rows for analysis plus certification, dense Bareiss plus
proof, and sparse exact replay. In the confirming 32-iteration run, every row
ended with a zero live-byte delta. Per-operation allocation volume ranged from
7 allocations and 1,536 bytes for rational or opaque sparse replay to 7,806
allocations and 398,392 bytes for the `TanPi` dense exact solve.

On the same Ryzen 7 5800X3D host with rustc 1.97.0, the end-to-end
representation benchmark ranged from 6.130--6.373 us for `One` to
582.89--607.36 us for `TanPi`. The competitive controls make their unequal
semantics explicit: exact Hypersolve quadratic candidates measured
300.93--313.40 ns versus 3.640--3.699 ns for the `roots` f64 proposal, and the
4 by 4 exact Bareiss solve with residual proof measured 5.462--5.538 us versus
67.933--68.306 ns for `nalgebra` f64 LU. Those rows measure the cost of
retaining exact candidates and replay evidence; they are not
correctness-equivalent speed claims. The correctness-equivalent CGAL/Gmpq 6.0.3
companion run produced a 432.815 ns median (426.731 ns p05, 573.478 ns p95) for
the same two exact roots.

`benches/competitors/run_cgal_quadratic.sh` now preserves each companion run in
`cgal_quadratic_benchmarks.md` as well as printing it, so this non-Criterion
comparison remains represented by the generated benchmark catalogue.

## Dispatch-path coverage

Every Criterion timing suite refreshes the complete unfiltered catalogue in
`benchmarks.md`, while the custom algebraic-fiber wall-clock harness writes
`algebraic_fiber_benchmarks.md`. Run `cargo run --example
write_benchmarks_md` to rebuild the aggregate report from stored Criterion
output without rerunning measurements.

Run `cargo bench --bench dispatch_trace --features dispatch-trace` to regenerate
`dispatch_trace.md`. The diagnostic harness pairs major families from the timed
`certification` benchmark with the shared `hyperreal` trace recorder. It covers
sketch lowering and preflight, analyzed candidate/batch/active-set work,
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
