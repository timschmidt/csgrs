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
