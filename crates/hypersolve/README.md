<h1>
  hypersolve
  <img src="./doc/hypersolve.png" alt="hypersolve logo" width="144" align="right">
</h1>

`hypersolve` is the experimental exact-aware solver layer for the Hyper stack. It models
variables, constraints, symbolic residuals, preparation facts, interval and candidate
certification, direct equality helpers, and lossy dense-solver adapters while keeping
`hyperreal::Real` values visible.

The crate is not a production nonlinear optimizer yet. It is the place where solver
intent, residual structure, active-set facts, and adapter precision boundaries are made
auditable before future high-performance backends are introduced.

## Typical Solver Problems

Solvers often mix symbolic intent, numeric residual evaluation, linear algebra,
predicate branches, and convergence policy inside one loop. When a solve fails, barely
passes, or changes branch, it can be unclear whether the issue was conditioning, stale
Jacobian structure, a lossy adapter, a wrong active set, or a genuinely discrete
topology rule.

`hypersolve` separates those layers. Expressions carry dependency and degree facts;
prepared problems retain row and Jacobian structure; direct and interval helpers expose
certificates; dense primitive linear solving is a named adapter with diagnostics rather
than internal truth.

## Main Types

- `Expr`, `SymbolId`, `ExprFacts`, and `ExprDegree` describe symbolic residual
  expressions over `Real`.
- `Problem`, `Variable`, `VariableId`, `Constraint`, and `ConstraintKind` describe the
  solver model.
- `EvaluationContext`, `ResidualEvaluation`, Jacobian helpers, and prepared problem
  types evaluate residuals and preserve structure.
- `PreparedAffineResidual`, polynomial residual types, solver-block facts, equality
  substitution classes, class-application reports, affine-row elimination reports,
  direct equality helpers, and univariate root-isolation reports expose reusable
  exact subproblems.
- `CandidateCertificationReport`, lossy-adapter-only reports, candidate-domain
  reports, residual balls, interval certification helpers, affine Krawczyk reports,
  univariate quadratic alpha reports, and predicate reports describe proof or
  uncertainty.
- `ProposalEngineKind`, `ProposalEngineReport`, `DenseLinearBackend`,
  `LinearSolveReport`, `SolverConfig`, `SolverState`, and `SolveReport` make
  lossy candidate generation explicit.
- `DenseResidualReplayReport`, `DenseResidualReplayError`, and
  `replay_dense_linear_residuals` provide the shared exact dense residual replay helper
  used by crates such as `hypercircuit`.

## Precision Model

Residual definitions and structural facts use `Real`. Dense primitive solves are
explicit adapters and report their precision boundary, pivot diagnostics, and
convergence outcome. Geometry or topology predicates should be delegated to
`hyperlimit`, `hypercurve`, `hypertri`, or the domain crate that owns the object.

Unknown certification is a first-class result. The crate should not turn unresolved
interval, predicate, or residual evidence into a float decision just to make a solve
look complete.

## Numerical Explosion

`hypersolve` combats numerical explosion by separating symbolic residual structure from
candidate generation. Degree facts, dependency masks, affine rows, substitution classes,
Sturm intervals, and replay reports keep exact cases small and push generic nonlinear
work into named proposal adapters whose outputs must be certified before acceptance.

## Performance Model

`hypersolve` works to keep expensive solving out of cases where structure already gives
an answer. Prepared facts record constant rows, affine rows, polynomial rows,
dependency masks, sparse Jacobian structure, and affine residual reuse. Direct equality
substitution, substitution-class candidate propagation, substitution-class affine row
elimination, univariate quadratic helpers, and Sturm root isolation handle small exact
cases before a generic nonlinear loop is needed.

Future backend work should exploit the same records: skip structural zeros, reuse
prepared Jacobians, route affine blocks to exact/direct solvers, and report when a dense
or sparse numeric adapter has crossed a lossy boundary. Levenberg-Marquardt is a named
lossy route through the current dense damped normal-equation proposal step. Powell
hybrid, dogleg, BFGS, and SQP remain named proposal choices that are reported as
unsupported rather than silently mapped to a different implementation.

## Current Status

Implemented today:

- symbolic expressions, simplification, structural facts, and differentiation;
- exact residual evaluation contexts and finite-difference/symbolic Jacobian builders;
- prepared problem, affine, polynomial, and solver-block fact records;
- direct one-row and square-system affine helpers, univariate-quadratic equality
  helpers with full-candidate replay reports, equality-substitution analysis,
  exact substitution-class construction, class-based candidate propagation, and
  non-mutating affine row elimination reports;
- exact-rational univariate polynomial root isolation by Descartes and Bernstein
  count bounds, recursive Bernstein subdivision reports, represented algebraic
  root wrappers, square-free reduction, Sturm intervals, bounded interval
  refinement, and exact rational witness replay;
- candidate, lossy-adapter-only, candidate-domain, interval, affine Krawczyk,
  univariate and multivariate quadratic Krawczyk, and univariate quadratic alpha
  certification surfaces;
- exact domain preflight for division, negative powers, square root, logarithm,
  inverse circular, and inverse hyperbolic residual nodes;
- reusable exact dense linear residual replay for domain crates that build their own
  small linear systems;
- a dense damped least-squares prototype with proposal-engine and adapter diagnostics;
- geometry residual helper modules. PCB, routing, and toolpath residual builders now
  live in `hyperpath::solve`, where their domain semantics are owned.

Known limits: nonlinear solve policy is still experimental, dense linear solving is
intentionally approximate, and domain topology remains delegated to sibling crates.

## Installation

```toml
[dependencies]
hypersolve = "0.3.0"
```

For sibling checkouts:

```toml
[dependencies]
hypersolve = { path = "../hypersolve" }
```

## Usage

Define residuals symbolically, evaluate candidates, and certify the result instead of
trusting a proposal loop by itself:

```rust
use hypersolve::{
    Constraint, Expr, PreparedProblem, Problem, certify_candidate, context_from_problem,
};
use hyperreal::Real;

let mut problem = Problem::default();
let x = problem.add_variable("x", Real::from(2));
let x_expr = Expr::symbol(problem.variables[x.0 as usize].symbol, "x");
problem.add_constraint(Constraint::equality(
    "x squared is four",
    x_expr.clone() * x_expr - Expr::real(Real::from(4)),
));

let prepared = PreparedProblem::new(&problem);
let candidate = context_from_problem(&problem);
let certification = certify_candidate(&prepared, &candidate);

assert_eq!(certification.certified_satisfied_rows, 1);
assert!(!certification.has_certified_violation());
```

Prepared affine/quadratic residuals, direct equality substitution, domain preflight,
interval, Krawczyk, and alpha reports, dense linear adapter diagnostics, predicate
reports, and domain helper modules keep solver structure visible across geometry, PCB,
and toolpath problems.

## Development

Useful local checks:

```sh
cargo fmt --all -- --check
cargo test --all-features
cargo clippy --all-targets --all-features -- -D warnings
RUSTDOCFLAGS="-D warnings" cargo doc --no-deps --all-features
cargo check --benches --all-features
```

## References

- [Bareiss, "Sylvester's Identity and Multistep Integer-Preserving Gaussian Elimination" (1968)](https://doi.org/10.1090/S0025-5718-1968-0226829-0)
- Bouma et al., ["A Geometric Constraint Solver" (1995)](https://doi.org/10.1016/0010-4485(95)00007-4)
- Collins, ["Subresultants and Reduced Polynomial Remainder Sequences" (1967)](https://doi.org/10.1145/321371.321381)
- Collins and Loos, "Real Zeros of Polynomials," in *Computer Algebra* (1982)
- Descartes, [*La Géométrie* (1637)](https://gallica.bnf.fr/ark:/12148/btv1b86069594)
- Farin, [*Curves and Surfaces for CAGD*, fifth edition (2002)](https://doi.org/10.1016/B978-1-55860-737-8.X5000-5)
- Farouki and Rajan, ["Algorithms for Polynomials in Bernstein Form" (1988)](https://doi.org/10.1016/0167-8396(88)90016-7)
- Krawczyk, ["Newton-Algorithmen zur Bestimmung von Nullstellen mit Fehlerschranken" (1969)](https://doi.org/10.1007/BF02234767)
- Levenberg, ["A Method for the Solution of Certain Non-Linear Problems in Least Squares" (1944)](https://doi.org/10.1090/qam/10666)
- Marquardt, ["An Algorithm for Least-Squares Estimation of Nonlinear Parameters" (1963)](https://doi.org/10.1137/0111030)
- Moore, *Interval Analysis* (1966)
- Nocedal and Wright, [*Numerical Optimization*, second edition (2006)](https://doi.org/10.1007/978-0-387-40065-5)
- Powell, ["A Hybrid Method for Nonlinear Equations" (1970)](https://doi.org/10.1007/BFb0067700)
- Sederberg and Nishita, ["Curve Intersection Using Bézier Clipping" (1990)](https://doi.org/10.1145/97879.97913)
- Shoemake, ["Animating Rotation with Quaternion Curves" (1985)](https://doi.org/10.1145/325334.325242)
- Smale, ["Newton's Method Estimates from Data at One Point" (1986)](https://doi.org/10.1007/978-1-4612-4984-9_1)
- Sturm, ["Mémoire sur la résolution des équations numériques" (1835)](https://eudml.org/doc/183280)
- Sylvester, ["On a Theory of the Syzygetic Relations of Two Rational Integral Functions" (1853)](https://doi.org/10.1098/rsta.1853.0005)
- Tinney and Walker, ["Direct Solutions of Sparse Network Equations by Optimally Ordered Triangular Factorization" (1967)](https://doi.org/10.1109/TPAS.1967.291823)
- Yap, ["Towards Exact Geometric Computation" (1997)](https://doi.org/10.1016/S0925-7721(96)00040-2)

## Hyper Ecosystem

`hypersolve` uses [hyperreal](https://github.com/timschmidt/hyperreal),
[hyperlattice](https://github.com/timschmidt/hyperlattice), and
[hyperlimit](https://github.com/timschmidt/hyperlimit). It provides solver and
certification services to the other [Hyper geometry and engineering
crates](https://github.com/timschmidt?tab=repositories&q=hyper&type=source).
