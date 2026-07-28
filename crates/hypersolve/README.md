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
  variants provide exact fraction-free linear algebra with reports.
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
  subdivision, square-free reduction, Sturm intervals, bounded refinement, and
  exact rational-witness replay.
- `represent_univariate_algebraic_roots` and
  `validate_algebraic_root_representation` create and check retained isolated
  roots.
- `compare_algebraic_root_representations`, affine/Möbius/binary transforms,
  polynomial images, rational images, and arithmetic reports operate without
  first lowering roots to primitive floats.
- `resultant_parametric_curve_intersection` and
  `resultant_rational_parametric_curve_intersection` expose solver-level curve
  elimination reports. Curve topology still belongs to Hypercurve.
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
- `replay_dense_linear_residuals` is the shared exact replay helper for domain
  crates that generate small dense systems.
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
cargo test --all-features
cargo clippy --all-targets --all-features -- -D warnings
RUSTDOCFLAGS="-D warnings" cargo doc --no-deps --all-features
cargo check --benches --all-features
```

The benchmark protocol, measured results, and retained/rejected optimization
record are in [PERFORMANCE.md](PERFORMANCE.md). Fuzz replay instructions are in
[fuzz/README.md](fuzz/README.md); dispatch instrumentation is documented in
[dispatch_trace.md](dispatch_trace.md).

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
