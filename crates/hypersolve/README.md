# hypersolve

`hypersolve` is the experimental exact-aware solver layer for the Hyper stack. It models
variables, constraints, symbolic residuals, preparation facts, interval and candidate
certification, direct equality helpers, and lossy dense-solver adapters while keeping
`hyperreal::Real` values visible.

The crate is not a production nonlinear optimizer yet. It is the place where solver
intent, residual structure, active-set facts, and adapter precision boundaries are made
auditable before future high-performance backends are introduced.

## Hyper Ecosystem

`hypersolve` owns solver-domain concepts and delegates the rest of the stack.

- [hyperreal](https://github.com/timschmidt/hyperreal): exact scalar residual and
  coefficient values.
- [hyperlattice](https://github.com/timschmidt/hyperlattice): small exact vector and
  matrix algebra used by solver helpers.
- [hyperlimit](https://github.com/timschmidt/hyperlimit): exact predicate decisions for
  geometry-facing constraints.
- [hypercurve](https://github.com/timschmidt/hypercurve),
  [hypertri](https://github.com/timschmidt/hypertri), and
  [hypermesh](https://github.com/timschmidt/hypermesh): geometry/topology owners for
  constraints that should not be reduced to local float tests.
- [hyperpath](https://github.com/timschmidt/hyperpath),
  [hyperdrc](https://github.com/timschmidt/hyperdrc),
  [hypercircuit](https://github.com/timschmidt/hypercircuit), and
  [hyperphysics](https://github.com/timschmidt/hyperphysics): domain crates that can
  generate residuals and replay accepted candidates.

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
- `PreparedAffineResidual`, polynomial residual types, solver-block facts, and direct
  equality helpers expose reusable exact subproblems.
- `CandidateCertificationReport`, residual balls, interval certification helpers, and
  predicate reports describe proof or uncertainty.
- `DenseLinearBackend`, `LinearSolveReport`, `SolverConfig`, `SolverState`, and
  `SolveReport` make lossy dense solving explicit.

## Precision Model

Residual definitions and structural facts use `Real`. Dense primitive solves are
explicit adapters and report their precision boundary, pivot diagnostics, and
convergence outcome. Geometry or topology predicates should be delegated to
`hyperlimit`, `hypercurve`, `hypertri`, or the domain crate that owns the object.

Unknown certification is a first-class result. The crate should not turn unresolved
interval, predicate, or residual evidence into a float decision just to make a solve
look complete.

## Performance Model

`hypersolve` works to keep expensive solving out of cases where structure already gives
an answer. Prepared facts record constant rows, affine rows, polynomial rows,
dependency masks, sparse Jacobian structure, and affine residual reuse. Direct equality
substitution and univariate quadratic helpers handle small exact cases before a generic
nonlinear loop is needed.

Future backend work should exploit the same records: skip structural zeros, reuse
prepared Jacobians, route affine blocks to exact/direct solvers, and report when a dense
or sparse numeric adapter has crossed a lossy boundary.

## Current Status

Implemented today:

- symbolic expressions, simplification, structural facts, and differentiation;
- exact residual evaluation contexts and finite-difference/symbolic Jacobian builders;
- prepared problem, affine, polynomial, and solver-block fact records;
- direct affine and univariate-quadratic equality helpers plus equality-substitution
  analysis;
- candidate and interval certification surfaces;
- a dense damped least-squares prototype with adapter diagnostics;
- geometry, PCB, and toolpath constraint helper modules.

Known limits: nonlinear solve policy is still experimental, dense linear solving is
intentionally approximate, and domain topology remains delegated to sibling crates.

## Installation

```toml
[dependencies]
hypersolve = "0.1.0"
```

For sibling checkouts:

```toml
[dependencies]
hypersolve = { path = "../hypersolve" }
```

## Development

Useful local checks:

```sh
cargo test
cargo bench --bench certification
```
