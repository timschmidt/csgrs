<h1>
  Hypersolve
  <img src="./doc/hypersolve.png" alt="Hypersolve logo" width="144" align="right">
</h1>

Exact-aware symbolic constraints, direct subproblem solvers, candidate
certification, and explicit numerical proposal adapters for the Hyper stack.

Hypersolve represents variables and residuals with `hyperreal::Real`, analyzes
their algebraic structure, solves supported exact subproblems, and records
whether a proposed candidate was certified, violated, or left unresolved. It
is not a claim that every nonlinear problem can be solved exactly, nor is its
dense floating-point proposal loop a source of topology truth.

Domain geometry remains owned by Hyperlimit, Hypercurve, Hypertri, and their
consumers. Hypersolve owns reusable equation structure and certification.

This README describes crate version `0.3.1`.

## Primary types

| Type | Role |
| --- | --- |
| `Expr`, `SymbolId`, `ExprFacts`, `ExprDegree` | Symbolic residual and structural facts |
| `Problem`, `Variable`, `Constraint` | Solver model |
| `ProblemAnalysis`, `ConstraintFacts`, `SolverBlock` | Dependency, degree, affine, polynomial, and sparsity analysis |
| `EvaluationContext`, `ResidualEvaluation` | Exact candidate bindings and residual values |
| `AffineResidual`, `QuadraticResidual`, `UnivariateQuadraticResidual` | Recognized exact subproblems |
| `CandidateCertificationReport`, `CandidateDomainReport` | Candidate proof, violation, uncertainty, and domain evidence |
| `AlgebraicRootRepresentation` | Isolated exact algebraic root carrier |
| `SolverConfig`, `SolverState`, `SolveReport` | Explicit proposal-engine state and diagnostics |
| `SketchSolveProblem`, `SketchConstraintBuilder` | Sketch-specific equation construction and reports |

## Install

```toml
[dependencies]
hypersolve = "0.3.1"
```

Hypersolve has no default features. `dispatch-trace` is for exact-dispatch
instrumentation and does not change the mathematical API.

## Quick start

This checked example evaluates and certifies the candidate \(x=2\) for
\(x^2-4=0\).

<!-- quickstart:start -->
```rust
use hyperreal::Real;
use hypersolve::{Constraint, Expr, Problem, certify_candidate, context_from_problem};

fn main() {
    let mut problem = Problem::default();
    let x = problem.add_variable("x", Real::from(2));
    let x_expr = Expr::symbol(problem.variables[x.0 as usize].symbol, "x");
    problem.add_constraint(Constraint::equality(
        "x squared is four",
        x_expr.clone() * x_expr - Expr::real(Real::from(4)),
    ));

    let analysis = problem.analyze();
    let candidate = context_from_problem(&problem);
    let certification = certify_candidate(&analysis, &candidate);

    assert_eq!(certification.certified_satisfied_rows, 1);
    assert!(!certification.has_certified_violation());
}
```
<!-- quickstart:end -->

Run it with:

```sh
cargo run --example basic
```

## Solver model

```text
Expr + Variable + Constraint
             │
          Problem
             │ analyze
       ProblemAnalysis
        ┌────┴───────────────┐
 exact/direct routes    numerical proposal
        │                        │
        └──────── candidate ─────┘
                     │
       domain preflight + exact replay
                     │
       certified / violated / unknown
```

Structural analysis is intentionally reusable. It records dependencies,
degrees, constant/affine/polynomial rows, sparse Jacobian structure, and solver
blocks before an algorithm is selected.

## API guide

### Expressions, models, and evaluation

- `Expr::{real, symbol, structural_facts, dependencies, simplify, derivative,
  eval_real}` builds and inspects symbolic residuals. Standard arithmetic and
  supported elementary functions compose expressions.
- `Variable::new` and `Variable::with_bounds` create variables directly.
  `Problem::{add_variable, add_constraint, analyze}` is the usual model path.
- `Constraint::equality` creates a zero-residual equation;
  `ConstraintKind` identifies other row semantics.
- `context_from_problem`, `EvaluationContext::bind`, and
  `evaluate_residuals` create and evaluate exact candidates.
- `symbolic_jacobian`, `symbolic_jacobian_with_analysis`, and
  `finite_difference_jacobian` expose exact-symbolic and explicitly finite
  Jacobian routes.

### Structural and direct exact solving

- `ProblemAnalysis::{facts, constraints, jacobian_sparsity,
  affine_residuals, univariate_quadratic_residuals, quadratic_residuals,
  evaluate_residuals}` exposes retained row structure.
- `AffineResidual::from_expr`, `QuadraticResidual::from_expr`, and
  `UnivariateQuadraticResidual::from_expr` recognize useful exact forms.
- `solve_direct_affine_equalities` and `solve_direct_affine_system` solve
  supported one-row or square affine systems.
- `determinant_bareiss`, `solve_dense_linear_system_bareiss`, and the sparse,
  multi-right-hand-side, pattern-preserving, and minimum-degree Bareiss
  variants provide exact fraction-free linear algebra with reports. Dense
  solves take an explicit `PredicatePolicy`, extend unresolved fast pivots
  through that policy, fall back to a pivot-free exact determinant construction
  when no pivot can be selected, and retain the weakest pivot,
  determinant, and residual certainty in their reports. Sparse pattern,
  pivot, and replay signs keep the requested refinement bound as their fast
  path, then use the strict exact predicate cascade; unsupported identities
  remain explicit rather than becoming approximate sparse decisions.
- `analyze_exact_affine_rank` reports exact rank evidence.
- `find_equality_substitutions`, `validate_equality_substitutions`,
  `equality_substitution_classes`, `apply_equality_substitutions`,
  `apply_equality_substitution_classes`, and
  `eliminate_affine_rows_with_substitution_classes` expose exact rewriting
  without mutating the source problem.
- `solve_direct_univariate_quadratic_equalities` and
  `certify_direct_univariate_quadratic_roots` enumerate and replay supported
  quadratic candidates.

### Polynomial and algebraic roots

- Root-isolation APIs provide Descartes/Bernstein bounds, recursive Bernstein
  subdivision, square-free reduction, exact dyadic Fujiwara whole-line bounds,
  Sturm intervals, bounded refinement, and exact rational-witness replay.
  Positive-width Sturm partitions own `(lower, upper]`; explicit exact
  witnesses own their point and are strictly checked for closed containment,
  polynomial vanishing, and unique-root evidence. Linear carriers are solved
  exactly, while validated one-root intervals bisect by exact midpoint and
  defining-polynomial sign under hard step and optional exact-width bounds.
  The public interval-presence query excludes both endpoints and returns an
  explicit unknown when its configured exact predicates do not decide. It uses
  endpoint/monotonicity proofs first, then exact Bernstein variation, a
  repeated-quadratic discriminant rule, square-free reduction, and Sturm as the
  complete fallback; a variation greater than one is never treated as a
  disproof. Algebraic image admission uses `(lower, upper]` for positive-width
  enclosures, strictly replays and collapses exact image witnesses, and bounds
  source refinement to eight rounds.
- `represent_univariate_algebraic_roots` and
  `validate_algebraic_root_representation` create and check retained isolated
  roots.
- `compare_algebraic_root_representations`, affine/Möbius/binary transforms,
  polynomial images, rational images, and arithmetic reports operate without
  first lowering roots to primitive floats.
- Represented-root comparison and sign replay cached-valid payloads under the
  strict policy before making a decision. Exact witnesses supersede wider
  stored bounds, exact operands are skipped during Sturm refinement, and
  difference comparison preserves direct/self orders before trying point
  polynomial replay, interval GCD evidence, narrowed-operand construction, or
  a represented difference sign. Rational sign decisions stay in rational
  payloads; unsupported identities remain explicit `Undecided` reports.
- Möbius transforms strictly replay their source carrier, accept general exact
  `Real` defining coefficients, and retain the power-sum construction as a
  fallback when homogeneous Horner trimming is undecided. Exact rational
  witnesses become canonical linear point representations, while mapped
  exact-`Real` points use zero-width isolating representations rather than
  being mislabeled as rational witnesses. Foreign poles are separated by
  bounded source refinement, selected-root poles remain explicit, and
  decreasing maps preserve `(lower, upper]` ownership. Rational determinant,
  denominator, endpoint, and homogeneous-polynomial work stays in rational
  payloads until final report values are constructed.
- Affine root transforms also strictly replay source evidence. Homogeneous
  Horner replaces repeated machine-binomial expansion, so degree is no longer
  limited by `u64` binomial storage; rational coefficients stay in rational
  payloads, exact point sources become canonical linear representations with
  truthful rational-versus-exact-`Real` metadata, and the certified scale sign
  orients reflected interval endpoints directly after bounded refinement has
  removed any ownership-changing endpoint roots.
- Represented-root arithmetic distinguishes stored exact rational results from
  general exact `Real` results without a rational payload, accepts either as a
  scalar for supported affine or Möbius lowering, and uses policy-certified
  inversion when ordinary
  exact division cannot decide a symbolic denominator. Direct point and
  identity routes replay their source payloads under `STRICT`; missing binary
  operands and unsupported coefficient fields retain distinct reports. Zero
  divided by a certified nonzero represented root is exactly zero, a selected
  nonzero root divides by itself exactly even when its interval touches zero,
  same-root addition uses an affine image, and rational quadratic squares use
  their defining linear relation instead of a general resultant.
- Algebraic-root polynomial and rational-expression evaluation strictly
  replays cached-valid point evidence and reports rational payloads separately
  from general exact `Real` values. Borrowed trimming and leading-seeded Horner
  avoid transient coefficient storage; rational intervals stay in `Rational`
  through endpoint products and quotients. Rational expressions reuse one
  source admission and the denominator/numerator sign proofs, with direct
  exact-zero, same-value, unit-denominator, and rational-payload quotient paths.
- Independent-root binary transforms strictly replay both source carriers,
  square-free repeated defining polynomials before enforcing the bounded
  resultant degree, and retain explicit unsupported-degree and denominator-
  domain reports. Exact-integer resultants use a scalar fraction-free path;
  general exact-rational coefficients keep the report-bearing fallback.
- Signed square-root transforms strictly replay their source representation
  and adaptively tighten exact dyadic image bounds when a nearby conjugate
  defeats the initial enclosure. Exact rational sources produce the canonical
  linear witness for rational square roots or the minimal quadratic relation
  for irrational ones; every bounded failure remains an explicit report.
- Unary polynomial-image transforms strictly replay their source evidence and
  admit only nonempty exact-rational authored image storage. Exact relations
  `q = scale * P + constant` collapse directly to a canonical rational point,
  while an oversized repeated carrier is square-freed before the same bounded
  Sylvester limit is enforced. Rational Horner and interval kernels stay in
  rational payloads until their final report values are constructed.
- Rational-image transforms keep the authored denominator authoritative:
  exact GCD selection reports a true pole at the chosen algebraic root, while
  a dependency-broadened interval triggers bounded exact source refinement.
  After domain admission, common factors and exact remainders modulo the
  source polynomial reduce the map before its fixed resultant budget. Exact
  constants and linear-fractional maps reuse general exact-`Real` Möbius
  support, decreasing maps preserve `(lower, upper]` endpoint ownership, and
  fixed-size batches share denominator admission, refinement, and source
  conversion without changing output order or per-image reports.
- `resultant_parametric_curve_intersection` and
  `resultant_rational_parametric_curve_intersection` expose solver-level curve
  elimination reports. Equal-degree rational systems through degree 12 use a
  half-size Bezout matrix, while other modest systems use a sparse Sylvester
  matrix. Both lanes evaluate one division-free polynomial determinant instead
  of repeated scalar determinants plus interpolation; the generic
  exact-coefficient path remains authoritative outside those bounded lanes.
  Curve topology still belongs to Hypercurve.
- Univariate, typed trivariate/quadrivariate, and rank-independent tensor
  resultants use the requested coefficient-refinement bound as their fast
  path, then continue unresolved exact degree, trimming, zero, normalization,
  and scheduled-resultant signs through the strict predicate cascade.
  Unsupported identities remain explicit `UndecidedCoefficient` or
  `UndecidedResultantSign` outcomes; no approximate sign enters elimination.
- Algebraic-fiber polynomial images and exact-`Real` quotient-ring norms use
  strict exact zero proofs when canonicalizing degree-bearing input,
  determinant output, and projective output storage. An unsupported trailing
  coefficient remains retained and participates in the configured degree
  rather than being discarded as an approximate zero.
- Direct tensor norms reuse already-certified retained degrees instead of
  rebuilding nominal zero padding. Sampled norms skip only represented
  rational zero arithmetic, and the quadratic closed form compacts only that
  same structural padding; opaque or unsupported coefficients still
  participate without an added sign decision.
- Dense tensor quotient-ring reduction certifies and inverts its shared
  modulus once, reuses one fiber buffer, and bypasses only entirely represented
  rational-zero fibers. Already-reduced tensors take a validated clone path;
  authored modulus shape and every opaque coefficient remain unchanged.
- Algebraic tensor images move validated dense storage through rational
  canonicalization, final square-free intake, and tagged bivariate conversion
  without cloning the complete tensor. The existing exact-rational normal form
  and all opaque coefficient decisions remain authoritative.
- Square-free intake inspects borrowed rational coefficients before deciding
  whether it needs an owned generic GCD. Modular coprimality can return the
  original polynomial, and the primitive-integer GCD path avoids its source
  clone; nonrational exact coefficients retain the generic path. Sturm setup
  likewise relies on its primitive normalizer's single trim rather than
  trimming each input twice.
- Polynomial long division reuses its certified canonical degrees: it borrows
  already-trimmed GCD/Sturm divisors, removes each algebraically canceled
  leading slot directly, and writes each quotient degree once. Arbitrary public
  divisors still pass through policy trimming, and exact nonrational fields use
  the same field operations and predicate authority as before.
- Local algebraic fields reuse the defining coefficients inside their owned,
  refinable root representation instead of cloning a second modulus. Public
  exact univariate division can trim borrowed divisor storage; internal
  local-field and GCD schedules retain owned divisors where matched benchmarks
  require them.
- Local quotient arithmetic updates rational product accumulators in place,
  moves certified leading cancellations, and seeds Horner evaluation from the
  actual leading coefficient. Denominator clearing consumes its optional
  denominators instead of cloning a parallel copy.
- Local sign and zero queries reuse the reduced numerator invariant established
  by every `LocalFieldElement` constructor and operation. Debug builds assert
  the canonical degree/trailing-coefficient boundary; release builds avoid a
  repeated clone, trim, and quotient reduction before consulting the sign
  cache.
- Local synthetic and exact division build quotient coefficients in descending
  discovery order, allocating an owning zero only for a genuinely skipped
  degree. Polynomial-image norms use direct 1-by-1 and 2-by-2 determinants;
  larger matrices retain the division-free Berkowitz construction.
- Local image convolution materializes only output degrees that receive a
  product. Quotient-multiplication matrices move completed row-major entries
  and eliminated high coefficients, while Berkowitz dot products move their
  first term before retaining the original ordered accumulation.
- Local-fiber exports consume their completed quotient-ring coefficients.
  Second-parameter output moves numerator vectors directly; first-parameter
  output moves each scalar into retained-power rows and pads only genuinely
  ragged entries. Any local denominator still rejects the export unchanged.
- Identically-zero image saturation computes the common image-coefficient
  factor once, then intersects every residual source multiplicity with that
  same factor. Fresh coefficient polynomials move into the GCD fold; local
  reduction, denominator clearing, and explicit removed-component evidence
  remain unchanged.
- Selected-fiber ordered-field Bernstein isolation seeds Horner values and
  controls from actual coefficients, creates the highest affine-composition
  term directly, and builds represented-root synthetic quotients in descending
  order. Endpoint ownership, variation counts, depth limits, and the complete
  repeated-root fallback are unchanged. Retained-field rational export also
  borrows its projective leading scalar while making a denominator monic.
- Rational-fiber reduction borrows its already reduced pivot and residual
  coefficients, materializes a shared zero only for a genuinely ragged power,
  and reuses the pivot's nonzero proof. The pivot residual is the exact
  cross-multiplied identity used to define the ratio; all other residuals keep
  their full quotient-ring zero proofs and public status boundaries.
- Polynomial-image projection constructs authored scalar image coefficients
  directly as denominator-free local-field constants. A validated retained
  modulus always has positive degree, so this removes an unreachable quotient
  reduction/error layer without changing degree budgets, unsupported storage,
  determinant arithmetic, or final retained/global relations.
- Batched open-interval fiber counts evaluate the lower Sturm boundary first
  and stop immediately when it is a root or an error. This matches the
  single-interval report order and avoids irrelevant upper-boundary work and
  retained-root refinement; ordinary boundary caching remains unchanged.
- Selected-fiber Sturm fallback moves its source polynomial into the sequence
  that already retains it, cloning that row back only for a rational-root
  restart. Incomplete reports preserve the Sturm length, subdivision count,
  retained-root refinements, certainty, and underlying error accumulated
  before termination.
- Plain algebraic-fiber projection shares the local field's exact evidence
  admission without cloning the complete refinable root or creating an unused
  sign cache. Retaining the second parameter also borrows the bivariate
  polynomial's authored coefficient grid directly; retaining the first still
  constructs the required transpose. The explicit degree budget bounds the
  retained defining-polynomial degree that determines matrix size.
- Selected-tensor tagged projection constructs `(z-source[0])^2` directly,
  moves its final flat tensor into primitive bivariate fibers, and reuses an
  exactly shared conjugate constraint. When exact square-free reduction lowers
  the first carrier's degree, that already-proved carrier also bounds the final
  quotient norm; repeated multiplicity cannot inflate the enumerator or its
  determinant dimension.
- Correlated tensor-image orchestration counts exact affine/duplicate-axis
  collapses as completed eliminations, reuses square-free constraints shared
  by non-affine conjugates, and skips quotient reductions whose stored power
  bounds are already below the divisor degree. Rational-class coefficients
  bypass symbolic normal-form traversal while opaque exact fields retain the
  unchanged proof path.
- Bézier, rational Bézier, B-spline span, and NURBS span substitution functions
  convert retained curve data into polynomial systems with explicit status.

### Certification

- `certify_candidate`, `certify_candidate_with_config`, and
  `certify_candidate_with_residual_balls` replay candidate rows.
- `certify_candidate_domains` preflights division, powers, square root,
  logarithm, inverse circular functions, and inverse hyperbolic functions.
- Interval APIs certify exact residual ranges over candidate boxes.
- Affine and quadratic Krawczyk reports, plus
  `certify_univariate_quadratic_alpha`, expose supported local existence and
  uniqueness evidence.
- `certify_candidate_batch` and `schedule_candidate_batch_predicates` reuse
  predicate schedules across candidates.
- `replay_dense_linear_residuals` is the shared policy-explicit exact replay
  helper for domain crates that generate small dense systems. Its residuals
  are retained once, while row evidence records sign and certainty without
  duplicating each `Real`.
- `project_origin_onto_simplex` returns exact closest-point, squared-distance,
  barycentric, and ordering evidence for small convex hulls.

Unknown certification is a result, not a request to fall through to a float
comparison.

### Proposal engines and active sets

- `solve_damped_least_squares` runs the configured dense proposal engine and
  returns `SolveReport`.
- `SolverConfig` selects limits, tolerances, Jacobian policy, and
  `ProposalEngineKind`; `SolverState` carries candidate values.
- Dense linear, Levenberg–Marquardt, Powell hybrid/dogleg, BFGS, and SQP routes
  are proposal generators. Their precision and preprocessing are retained in
  `ProposalEngineReport`.
- `audit_active_set`, `propose_active_set_update`,
  `regenerate_active_set_affine_candidate`,
  `regenerate_active_set_quadratic_candidates`, and
  `run_active_set_update_loop` make active-mask changes auditable.
- Failed-constraint search reports identify single, pair, and bounded set
  removals without silently weakening the source model.

### Sketch and domain helpers

- `SketchSolveProblem` and `SketchConstraintBuilder` build exact-aware 2D and
  projected-workplane constraints.
- Builder methods cover coincidence, incidence, horizontal/vertical,
  parallel/perpendicular/same-direction, distances and ranges, radii,
  concentricity, length equality/ratio/difference, angles, tangency,
  midpoint/symmetry, parameter bounds, chamfer/fillet-related continuity, and
  projected 3D equivalents.
- Sketch preflight functions audit entity domains, workplanes, unit/tolerance
  declarations, degeneracy, compatibility fixtures, construction
  certificates, and failed constraints.
- `SolverPoint2`, `squared_distance_expr`, `point_coincidence_equations`, and
  tangent helpers are small reusable equation builders.

Routing, PCB, and toolpath residual builders live in Hyperpath, where their
domain semantics can be documented and validated.

## Guarantees and boundaries

- Residual definitions, direct algebra, and certification use
  `hyperreal::Real`.
- A numerical proposal is never automatically a certified solution.
- Domain-invalid expressions are reported before residual acceptance.
- Geometry predicates are delegated to the crate that owns the geometry.
- Dense finite linear algebra, finite differences, convergence tolerances, and
  numerical stopping conditions are named adapter policy.
- Exact reports retain dependencies, row structure, pivots, permutations,
  intervals, witnesses, or replay values needed to audit the conclusion.

The nonlinear proposal layer is experimental. Consumers should accept a
candidate only when the required domain and residual reports are certified for
their use case.

## Feature flags

| Feature | Default | Purpose |
| --- | --- | --- |
| `dispatch-trace` | no | Hyperreal/Hyperlattice/Hyperlimit exact-dispatch instrumentation |

## Validation and performance

```sh
cargo fmt --all -- --check
cargo test --locked --all-targets --all-features
cargo clippy --locked --all-targets --all-features -- -D warnings
RUSTDOCFLAGS="-D warnings" cargo doc --locked --no-deps --all-features
cargo check --locked --manifest-path fuzz/Cargo.toml --bins
scripts/representation_coverage.sh
scripts/coverage.sh
scripts/allocation_profile.sh
cargo bench --locked --bench representations
cargo bench --locked --bench competitive
```

The representation matrix explicitly inventories Hyperreal's 22 optimized
finite certificate classes and all eight public structural kinds. It also
exercises variable-depth opaque computable DAGs. The coverage script generates
an HTML report and enforces an 85% production-line floor; the allocation
profiler reports steady-state counts and bytes for analysis/certification,
exact dense solving, and sparse replay for every class, and fails on a nonzero
signed live-byte delta. Competitive Criterion rows compare exact Hypersolve
roots and dense solves with the proposal-only `roots` and `nalgebra` float
APIs; the CGAL/Gmpq harness remains the exact quadratic comparison.

The benchmark protocol and retained/rejected optimization record are in
[PERFORMANCE.md](PERFORMANCE.md). The automatically refreshed
[benchmarks.md](benchmarks.md) catalogues every timing and diagnostic suite,
all stored Criterion rows, and comparative ratios; the custom algebraic-fiber
timings are retained in
[algebraic_fiber_benchmarks.md](algebraic_fiber_benchmarks.md), and the external
exact CGAL/Gmpq comparison writes
[cgal_quadratic_benchmarks.md](cgal_quadratic_benchmarks.md). Fuzz replay
instructions are in [fuzz/README.md](fuzz/README.md); dispatch instrumentation
is documented in [dispatch_trace.md](dispatch_trace.md).

## References

These sources describe the exact algebra, interval certification, nonlinear
proposal, and geometric-constraint techniques relevant to Hypersolve:

- Bareiss, E. H. “Sylvester’s Identity and Multistep Integer-Preserving
  Gaussian Elimination.” *Mathematics of Computation* 22, 1968.
  [DOI: 10.1090/S0025-5718-1968-0226829-0](https://doi.org/10.1090/S0025-5718-1968-0226829-0).
- Bouma, W., Fudos, I., Hoffmann, C. M., Cai, J., and Paige, R. “A Geometric
  Constraint Solver.” *Computer-Aided Design* 27(6), 1995.
  [DOI: 10.1016/0010-4485(95)00007-4](https://doi.org/10.1016/0010-4485(95)00007-4).
- Collins, G. E. “Subresultants and Reduced Polynomial Remainder Sequences.”
  *JACM* 14(1), 1967.
  [DOI: 10.1145/321371.321381](https://doi.org/10.1145/321371.321381).
- Farouki, R. T., and Rajan, V. T. “Algorithms for Polynomials in Bernstein
  Form.” *Computer Aided Geometric Design* 5(1), 1988.
  [DOI: 10.1016/0167-8396(88)90016-7](https://doi.org/10.1016/0167-8396(88)90016-7).
- Krawczyk, R. “Newton-Algorithmen zur Bestimmung von Nullstellen mit
  Fehlerschranken.” *Computing* 4, 1969.
  [DOI: 10.1007/BF02234767](https://doi.org/10.1007/BF02234767).
- Levenberg, K. “A Method for the Solution of Certain Non-Linear Problems in
  Least Squares.” *Quarterly of Applied Mathematics* 2, 1944.
  [DOI: 10.1090/qam/10666](https://doi.org/10.1090/qam/10666).
- Marquardt, D. W. “An Algorithm for Least-Squares Estimation of Nonlinear
  Parameters.” *SIAM Journal on Applied Mathematics* 11(2), 1963.
  [DOI: 10.1137/0111030](https://doi.org/10.1137/0111030).
- Moore, R. E. *Interval Analysis*. Prentice-Hall, 1966.
- Nocedal, J., and Wright, S. J. *Numerical Optimization*, 2nd ed. Springer,
  2006. [DOI: 10.1007/978-0-387-40065-5](https://doi.org/10.1007/978-0-387-40065-5).
- Powell, M. J. D. “A Hybrid Method for Nonlinear Equations.” In *Numerical
  Methods for Nonlinear Algebraic Equations*, 1970.
  [DOI: 10.1007/BFb0067700](https://doi.org/10.1007/BFb0067700).
- Sederberg, T. W., and Nishita, T. “Curve Intersection Using Bézier
  Clipping.” *Computer-Aided Design* 22(9), 1990.
  [DOI: 10.1016/0010-4485(90)90039-F](https://doi.org/10.1016/0010-4485(90)90039-F).
- Smale, S. “Newton’s Method Estimates from Data at One Point.” In *The
  Merging of Disciplines*, 1986.
  [DOI: 10.1007/978-1-4612-4984-9_1](https://doi.org/10.1007/978-1-4612-4984-9_1).
- Sturm, C. “Mémoire sur la résolution des équations numériques.” 1835.
  [EuDML](https://eudml.org/doc/183280).
- Tinney, W. F., and Walker, J. W. “Direct Solutions of Sparse Network
  Equations by Optimally Ordered Triangular Factorization.” *IEEE Transactions
  on Power Apparatus and Systems*, 1967.
  [DOI: 10.1109/TPAS.1967.291823](https://doi.org/10.1109/TPAS.1967.291823).
- Yap, C. K. “Towards Exact Geometric Computation.” *Computational Geometry*
  7(1–2), 1997.
  [DOI: 10.1016/0925-7721(95)00040-2](https://doi.org/10.1016/0925-7721(95)00040-2).

## Acknowledgements

Hypersolve builds on
[Hyperreal](https://github.com/timschmidt/hyperreal),
[Hyperlattice](https://github.com/timschmidt/hyperlattice), and
[Hyperlimit](https://github.com/timschmidt/hyperlimit). The research cited
above informs its algorithms and evidence boundaries without implying
source-code derivation.

## License and contributing

Licensed under the [Apache License 2.0](LICENSE).

Bug reports should include the smallest problem, initial candidate, selected
proposal engine, enabled features, and complete analysis/certification report.
Before proposing a change, run formatting, the focused regression, all-feature
tests, and strict Clippy.
