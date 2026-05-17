# hyperphysics

`hyperphysics` is currently a placeholder crate for future exact-aware physics
and simulation adapters in the Hyper ecosystem. Its numerical policy should
match the rest of the stack: hyperreal-backed values are the primary
representation for authored geometry and configuration, while primitive
`f32`/`f64` appear only at rendering, simulation-engine interop, file IO,
diagnostics, or external-library adapters.

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
- `hypermesh`: future manifold mesh preparation and 3D collision/boolean
  topology when solid inputs are involved.

## Traditional Physics Problems

Physics engines usually optimize for throughput and stability under finite
time steps, not for exact authored geometry. Contact manifolds, resting
contacts, thin features, collision margins, mass properties, and constraint
solvers all mix approximation policy with topology. That is appropriate for a
runtime engine, but it makes it hard to audit whether an exported simulation
lost a constraint because of tolerance, geometry cleanup, or a real modeling
issue.

`hyperphysics` should become an adapter layer rather than a replacement physics
engine. Exact geometry and material facts should be retained at construction
time, lossy runtime export should be explicit, and any post-simulation checks
should use Hyper predicates where possible. Performance work should focus on
shape classification, broad-phase bounds, transform-kind facts, mass/inertia
structure, and prepared contact geometry before crossing into an approximate
engine.

## Current Status

`hyperphysics` intentionally has no production physics API yet. The crate is a
planning and boundary marker while the exact scalar, predicate, curve,
triangulation, mesh, and solver layers settle. Its README records the intended
ownership split so future implementation work does not bury lossy simulation
policy inside geometry constructors or exact predicates.

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
