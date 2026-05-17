# hypersolve

`hypersolve` is the experimental solver layer for the hyper stack. It currently
depends on `hyperreal` for exact scalar values and `hyperlattice` for exact-aware
linear algebra, with the goal of keeping residuals, active sets, and repeated
constraint kernels explicit instead of lowering silently to primitive floats.

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
