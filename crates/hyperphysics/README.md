# hyperphysics

`hyperphysics` is currently a placeholder crate. Its future numerical policy
should match the rest of the hyper stack: hyperreal-backed values are the
primary representation, while primitive `f32`/`f64` appear only at rendering,
simulation-engine interop, file IO, diagnostics, or external-library adapters.

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

`hyperphysics` should own physics-domain concepts such as bodies, fixtures,
materials, contacts, constraints, time integration policy, collision-response
diagnostics, and adapters to external physics engines. It should not own scalar
arithmetic, small linear algebra kernels, exact predicate policy, curve/region
topology, or triangulation data structures.

Expected dependencies by responsibility:

- `hyperreal`: scalar values, exact rationals, symbolic constants, and scalar
  structural facts.
- `hyperlattice`: vectors, matrices, transforms, and object-level structural
  facts such as zero masks, affine transform kind, and cached determinant state.
- `hyperlimit`: exact/refined geometric predicates for contact, sidedness, and
  incidence decisions.
- `hypercurve` and `hypertri`: boundary/region and triangulation preparation
  when physics shapes originate from exact geometry.

## Structural-Information Opportunities

Physics code should carry inexpensive metadata discovered at import or shape
construction time: static/dynamic body class, convexity, axis alignment,
circle/sphere/capsule/box kind, transform kind, mass/inertia diagonal shape,
known-zero velocities, broad-phase bounds, grid scale, and material/contact
category. These facts can select faster exact setup or narrower external-engine
adapters while keeping lossy numeric simulation decisions explicitly named.

## Plan

- Keep the crate minimal until the exact geometry stack surfaces needed by
  physics shapes are stable.
- Add exact shape constructors that lift finite external coordinates into
  hyperreal-backed `hyperlattice` values.
- Isolate any `f64` physics-engine bridge behind an adapter that documents
  lossy export, solver tolerance, and exact-result validation where possible.
- Reuse `hyperlimit` for contact/topology predicates instead of implementing
  primitive-float predicates locally.
