# hypersolve

`hypersolve` is the experimental solver layer for the hyper stack. It currently
depends on `hyperreal` for exact scalar values and `hyperlattice` for exact-aware
linear algebra, with the goal of keeping residuals, active sets, and repeated
constraint kernels explicit instead of lowering silently to primitive floats.

It is not yet a production optimizer. The current crate is a model, expression,
preparation, and diagnostics layer for testing how exact scalar facts and
prepared residual structure should flow into future nonlinear, geometric, PCB,
toolpath, and physics solvers.

## Hyper Stack Links

- [hyperreal](../hyperreal/README.md): exact rational, symbolic, and computable
  real arithmetic.
- [hyperlimit](../hyperlimit/README.md): exact predicate policy and certified
  geometric decisions.
- [hyperlattice](../hyperlattice/README.md): small exact vector, matrix, and
  transform algebra.
- [hypercurve](../hypercurve/README.md): planar curve, contour, region, and
  boolean geometry.
- [hypertri](../hypertri/README.md): exact polygon triangulation and constrained
  Delaunay topology.
- [hypermesh](../hypermesh/README.md): 3D mesh boolean experiments and the
  future exact-aware mesh-topology layer.
- [hypersolve](../hypersolve/README.md): experimental exact-aware solver layer.
- [hyperdrc](../hyperdrc/README.md): PCB design-readiness checks over exact-aware
  geometry adapters.
- [hyperphysics](../hyperphysics/README.md): placeholder physics-domain crate
  for the exact geometry stack.
- [csgrs](../csgrs/readme.md): constructive solid geometry and polygon boolean
  engine used by HyperDRC and available as an interop target.

## Semantic Boundary

`hypersolve` should own solver-domain concepts: variables, constraints,
prepared residual blocks, active-set state, diagnostics, convergence policy, and
lossy adapters to dense numerical solvers when an approximation is explicitly
requested.

It should not own scalar arithmetic, matrix storage, predicate policy, curve or
triangulation topology, PCB/CAM metadata, or CSG boolean semantics. Those remain
in the sibling crates linked above.

## Traditional Solver Problems

Solvers often mix symbolic intent, numeric residual evaluation, linear algebra,
predicate branches, and convergence policy into one opaque loop. That makes it
hard to tell whether a failure came from bad conditioning, a lossy adapter, an
incorrect active set, stale Jacobian structure, or a domain constraint that was
really discrete/topological.

`hypersolve` keeps those boundaries separate. Expressions carry dependency and
degree facts; prepared problems retain residual-row summaries, Jacobian
sparsity, affine residuals, and constant-row certificates; dense primitive
linear solving is exposed as an explicit lossy adapter with diagnostics rather
than as the internal truth. The goal is to control numerical explosion by
reusing symbolic structure, skipping structurally zero work, and reporting
precision boundaries before a later high-performance backend is introduced.

## Current Status

Implemented pieces include:

- symbolic expressions over `hyperreal::Real`, named symbols, arithmetic,
  powers, roots, trig/log nodes, simplification, structural facts, and symbolic
  differentiation;
- `Problem`, `Variable`, and `Constraint` models with equality constraints and
  optional variable bounds;
- evaluation contexts and residual evaluation over exact `Real` bindings;
- finite-difference and symbolic Jacobian builders, plus prepared symbolic
  Jacobian evaluation;
- prepared problem facts for active rows, constant rows, affine rows,
  polynomial rows, dependency masks, structural Jacobian nonzeros, and affine
  residual reuse;
- a dense damped least-squares prototype whose linear solve report records the
  adapter precision boundary and pivot diagnostics;
- domain placeholder modules for geometry, PCB, and toolpath constraints;
- smoke and proptest coverage for residual and preparation invariants.

Known limits: nonlinear solve policy is still experimental, dense linear
solving is intentionally approximate, and geometry/topology constraints should
delegate exact predicates to `hyperlimit`, `hypercurve`, and `hypertri`.
