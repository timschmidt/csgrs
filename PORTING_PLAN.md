# Porting Plan: hyperlimit, hyperlattice, and hyperreal

This plan describes how `csgrs` should be ported onto the `hyperlimit`,
`hyperlattice`, and `hyperreal` stack without blurring ownership boundaries.
The goal is to make `csgrs` more robust while keeping it focused on CSG and CAD
modeling rather than turning it into a numeric kernel.

The immediate transition should not jump straight from `csgrs`'s current
f64/epsilon behavior to full `hyperreal` semantics. First, audit the existing
f64 predicates and epsilon guards, teach `hyperlimit` to match or exceed those
needs through an approximate backend, and then remove the duplicated
`csgrs`-local epsilon system. Once `csgrs` depends on `hyperlimit` for
approximate predicate behavior, porting the same predicate surface to
`hyperreal` becomes a backend upgrade instead of a simultaneous semantic and API
rewrite.

## Target ownership model

The stack should have clear vertical responsibilities:

```text
hyperreal     = scalar semantics and exact/structural real arithmetic
hyperlattice  = vectors, points, matrices, transforms, and scalar-backed linear algebra
hyperlimit    = robust geometric predicates and predicate carrier types
csgrs         = CSG objects, modeling operations, topology, metadata, IO, and user API
```

### `hyperreal`

`hyperreal` should own scalar behavior:

- exact rationals
- symbolic and computable reals
- lazy approximation
- sign, zero, magnitude, and structural facts
- refinement and conservative comparison support

It should not own CSG, polygon, mesh, or CAD-specific geometry types.

### `hyperlattice`

`hyperlattice` should own the general linear algebra layer:

- `Scalar`
- `Vector2`, `Vector3`, `Vector4`
- `Point2`, `Point3` if the point types participate in affine APIs
- `Matrix3`, `Matrix4`
- transforms, dot products, cross products, normalization, and affine operations

It should not know about:

- polygon winding
- BSP splitting
- mesh manifoldness
- CSG Boolean operations
- triangulation policy
- file formats
- metadata propagation

### `hyperlimit`

`hyperlimit` should own robust predicate-facing geometry:

- minimal `Point2<S>` and `Point3<S>` carrier types
- `Plane3<S>` in implicit form
- predicate case structs for batching, where useful
- orientation and classification results

It should provide:

- `orient2d`
- `orient3d`
- `incircle2d`
- `insphere3d`
- point-line classification
- point-plane classification
- oriented point-plane classification

These types should remain small and policy-free. `hyperlimit` should not own
rich polygon, mesh, sketch, vertex, toolpath, or CSG types.

### `csgrs`

`csgrs` should continue to own product-level geometry:

- `Sketch<S>`
- `Mesh<S>`
- `Polygon<S>`
- `Vertex`
- BSP nodes and splitting logic
- optional `BMesh` / manifold-backed representations
- shape constructors
- extrusion, revolve, sweep, loft, offset, and modeling operations
- triangulation orchestration
- smoothing, quality, manifold, and connectivity utilities
- CNC/toolpath-facing types
- import/export glue for STL, OBJ, PLY, DXF, Gerber, SVG, glTF, AMF, and WASM
- metadata propagation rules

`csgrs` may re-export or alias lower-level point/vector/matrix types for user
convenience, but it should avoid owning a new fundamental numeric geometry
kernel once `hyperlattice` is available.

## Current `csgrs` type disposition

### Keep in `csgrs`

Keep these as `csgrs` types because they encode CSG or product semantics:

- `mesh::Mesh<S>`
- `sketch::Sketch<S>`
- `polygon::Polygon<S>`
- `vertex::Vertex`
- `bmesh::BMesh<S>`
- `voxels` module types
- `toolpath` module types
- IO module types
- shape-constructor APIs

`Vertex` should remain in `csgrs` while it means "mesh/render/CSG vertex" with a
position, normal, interpolation behavior, and export implications. A lower-level
point type should not absorb normals or shading semantics.

### Convert or wrap

The current `mesh::plane::Plane` should eventually stop being the authoritative
robust predicate type. It can remain as a CSG face-plane cache or wrapper, but
classification and splitting should delegate to `hyperlimit`.

Recommended direction:

- store face-plane data in whichever form is most useful for `Polygon<S>`
- convert to `hyperlimit::Plane3` or oriented `hyperlimit::Point3` triples at
  predicate boundaries
- use `hyperlimit` outcomes to classify vertices and polygons
- keep CSG policy decisions, such as `COPLANAR`, `FRONT`, `BACK`, and
  `SPANNING`, in `csgrs`

### Stop owning long term

Long term, `csgrs` should not own independent definitions of fundamental:

- scalar semantics
- generic vectors
- generic points
- generic matrices
- generic robust predicates

Those should come from `hyperreal`, `hyperlattice`, and `hyperlimit`.

## Migration phases

### Phase 1: Audit the existing f64/epsilon predicate surface

Inventory every `csgrs` path that makes geometric decisions using f64,
epsilon-based guards, or approximate equality.

Initial targets:

- `mesh::plane::Plane::eq`
- point-plane classification
- polygon splitting
- BSP front/back/coplanar/spanning classification
- coplanarity checks
- point-line and point-plane tests used by triangulation or projection logic
- any local robust predicate wrappers
- guards that turn small magnitudes into zero
- equality checks that affect topology or metadata propagation

For each path, record:

- the input type and coordinate source
- the epsilon or tolerance rule
- the output classification
- the downstream policy decision in `csgrs`
- whether the behavior is required compatibility or just an implementation
  accident

The audit output should become tests before implementation changes are made.

### Phase 2: Add `hyperlimit` parity and stress tests

Add `hyperlimit` tests that match the current `csgrs` predicate needs before
replacing code in `csgrs`.

The test set should include:

- parity cases for current f64/epsilon behavior
- near-coplanar polygons
- near-degenerate triangles
- spanning polygons with vertices close to the plane
- point equality and near-equality cases that affect topology
- oriented point-plane classification
- point-line classification used by projection or triangulation
- cases where current `csgrs` behavior is unstable or under-specified

The goal is not to freeze every old epsilon decision forever. The goal is to
make the intended compatibility surface explicit, then let `hyperlimit` exceed
it with better uncertainty handling and clearer classifications.

### Phase 3: Add the `hyperlimit` approximate backend needed by `csgrs`

Implement the `hyperlimit` approximate backend as the first replacement target
for `csgrs`'s current f64 system.

This backend should:

- accept current `csgrs` f64-like coordinate data through adapters
- expose the predicate results `csgrs` needs without exposing CSG policy
- represent uncertainty explicitly
- support the current classification cases, including `COPLANAR`, `FRONT`,
  `BACK`, and `SPANNING` translation at the `csgrs` boundary
- centralize epsilon or approximate comparison behavior in `hyperlimit`
- leave room for a later `hyperreal` backend behind the same predicate surface

After this phase, `hyperlimit` should be able to replace the f64/epsilon
predicate machinery in `csgrs` without requiring a full scalar or public API
port.

### Phase 4: Introduce explicit adapter boundaries

Add internal conversion helpers between current `csgrs` numeric types and the
new stack:

- `nalgebra::Point2<Real>` to `hyperlimit::Point2<RealLike>`
- `nalgebra::Point3<Real>` to `hyperlimit::Point3<RealLike>`
- `mesh::plane::Plane` to oriented point triples or `hyperlimit::Plane3`
- `nalgebra::Matrix4<Real>` to the chosen `hyperlattice` transform type once
  that API is stable enough

Keep these adapters private at first. The first milestone is internal
correctness, not public API churn.

### Phase 5: Replace predicate calls and rip out local epsilon guards

Replace direct robust predicate usage and local f64/epsilon guards in `csgrs`
with `hyperlimit` approximate-backend calls.

Initial targets:

- `mesh::plane::Plane::eq`
- vertex classification in polygon splitting
- coplanarity checks
- point-line and point-plane tests used by triangulation or projection logic

The CSG layer should translate `hyperlimit` results into existing `csgrs`
classification constants or enums.

Once coverage proves the replacement, remove the duplicated local predicate
machinery from `csgrs`. `csgrs` should keep modeling policy such as
`COPLANAR`, `FRONT`, `BACK`, and `SPANNING`, but it should not keep a second
epsilon-based predicate implementation beside `hyperlimit`.

### Phase 6: Move linear algebra usage behind aliases

Introduce crate-local aliases for the public numeric types used throughout
`csgrs`.

For example, instead of importing concrete third-party point/vector/matrix types
throughout the codebase, route them through `float_types` or a successor module:

- `Point2`
- `Point3`
- `Vector2`
- `Vector3`
- `Matrix4`
- `Real`

This makes the eventual switch from `nalgebra`-centered internals to
`hyperlattice` incremental instead of global.

### Phase 7: Port transforms and constructors

Move transform-heavy APIs onto the chosen `hyperlattice` types:

- `translate`
- `rotate`
- `scale`
- `mirror`
- `transform`
- shape constructors that build many points or normals
- sweep, revolve, loft, and extrusion routines

Preserve existing user-facing ergonomics where possible. If public type changes
are unavoidable, introduce them in a single documented release boundary.

### Phase 8: Revisit scalar configurability

Today `csgrs` selects `Real` through `f32` and `f64` features. After the
predicate and linear algebra ports are stable, decide whether `csgrs` should:

- keep `f32` / `f64` as the primary public modes and use the new stack
  internally for robustness, or
- become generic over a `hyperlattice` backend, or
- offer a small set of backend feature modes.

The conservative default should be to preserve the current `Real`-based public
API until the benefits of exposing backend generics are clear.

### Phase 9: Port the predicate backend toward `hyperreal`

After the f64/epsilon system has been replaced by `hyperlimit`'s approximate
backend, add or enable a `hyperreal`-backed implementation behind the same
predicate-facing API.

At this point, the semantic boundary should already be correct:

- `csgrs` owns CSG policy
- `hyperlimit` owns predicate classification
- `hyperreal` owns scalar refinement and exact/structural arithmetic

The hyperreal port should therefore focus on backend behavior, performance, and
additional certificates rather than rewriting `csgrs` modeling logic again.

### Phase 10: Public API cleanup

After internals are stable:

- decide which lower-stack types to re-export
- document the ownership model in the crate docs
- deprecate redundant public helpers gradually
- keep compatibility shims for common `nalgebra` interop if downstream users
  rely on it

## Design rules during the port

- Lower crates provide facts; `csgrs` makes modeling decisions.
- Predicate uncertainty should be represented explicitly until `csgrs` decides
  how to handle it.
- Metadata behavior remains entirely owned by `csgrs`.
- IO formats should not leak lower-stack internals into public file APIs.
- Avoid moving a type downward just because it contains coordinates.
- Avoid making `hyperlimit` responsible for CSG concepts such as polygon
  splitting policy or manifold repair.
- Prefer private adapters first, then public API changes after behavior is
  proven.

## Suggested first implementation target

Start with `mesh::plane`.

It is the highest-leverage boundary because polygon splitting and BSP Booleans
depend on point-plane classification. A focused first change would:

1. Audit the existing `mesh::plane` f64/epsilon behavior.
2. Add `hyperlimit` parity and stress tests for that behavior.
3. Implement any approximate-backend features needed by those tests.
4. Add conversion helpers from `csgrs::vertex::Vertex` and
   `csgrs::mesh::plane::Plane` into `hyperlimit` predicate inputs.
5. Replace direct `robust::orient3d` comparisons with `hyperlimit::orient3d` or
   `hyperlimit::classify_point_oriented_plane`.
6. Keep the existing `COPLANAR`, `FRONT`, `BACK`, and `SPANNING` constants.
7. Remove the local epsilon predicates after `hyperlimit` coverage is in place.
8. Add regression tests for near-coplanar and spanning polygons.
9. Only then evaluate whether the internal `Plane` representation should change.

This gives an early robustness win without forcing a broad public API rewrite.

## Non-goals

- Do not move `Mesh<S>` or `Sketch<S>` into the lower stack.
- Do not make `hyperlimit` a mesh-processing crate.
- Do not make `hyperlattice` aware of file formats or CSG operations.
- Do not expose backend generics everywhere until the ergonomics are proven.
- Do not remove `nalgebra` interop until replacement ergonomics are clearly
  better for existing users.
