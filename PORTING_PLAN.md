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

The approximate f64 phase should evaluate
[`decorum`](https://github.com/olson-sean-k/decorum) as a bridge for making
floating-point behavior explicit before the full `hyperreal` port. `decorum`
provides constrained IEEE-754 proxy types with total ordering, equivalence,
hashing, configurable handling of non-real values, and `approx` integration.
Those properties map directly to several current `csgrs` pain points: finite
coordinate enforcement, deterministic map/set keys, explicit NaN/infinity
policy, and centralized approximate comparisons.

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

## Boolean, triangulation, contour, and kernel policy

Mesh booleans should prefer `boolmesh` as the primary boolean layer. The current
`BMesh`/boolmesh integration should be treated as the direction of travel, not
as a side experiment:

- use the existing BSP layer as the early porting and verification target before
  completing the boolmesh port
- port BSP predicates, splitting, and classification boundaries first so the
  hyperlimit/hyperreal behavior can be verified against the current mesh boolean
  semantics
- keep BSP available as the differential test oracle while boolmesh-backed
  booleans are being brought up
- finish the experimental boolean feature so boolmesh-backed booleans are
  usable as the robust mesh boolean path
- port boolmesh-facing adapters and, where necessary, boolmesh-derived logic to
  hyperreals instead of treating f64 conversion as the final numeric boundary
- keep conversions between ordinary `Mesh`, `BMesh`, and boolmesh explicit,
  tested, and easy to remove or simplify after the hyperreal backend is in place

Triangulation and 2D contour dependencies should be brought under the same
numeric plan:

- port `spade` usage to hyperreals for Delaunay and constrained triangulation
- port `earcut` / `earcutr` usage to hyperreals for earcut triangulation
- port `cavalier_contours` usage to hyperreals for offsets, contour cleanup, and
  any 2D boolean-style contour operations
- keep f64 paths as transitional compatibility shims, not as the long-term
  robustness model

External geometry kernels should be handled in two stages. `curvo` and other
useful kernels can be adapted to `csgrs` as f64 immediately when they provide
practical functionality, but those adapters should not become permanent
high-maintenance patches against moving upstream targets. The long-term plan is
to harvest the useful algorithms, port them to hyperreals, and integrate them
behind small internal traits or direct implementations so `csgrs` carries less
patch burden over time.

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
- evaluate `decorum` constrained f64 proxy types for the approximate backend's
  scalar carrier, especially where finite-real invariants, total ordering,
  equality, hashing, or `approx` traits are required
- use `decorum` at adapter and predicate boundaries first, not as a broad public
  API replacement for `Real`
- make the NaN/infinity policy explicit: reject, sanitize, or propagate
  uncertainty deliberately instead of relying on primitive f64 behavior
- expose the predicate results `csgrs` needs without exposing CSG policy
- represent uncertainty explicitly
- support the current classification cases, including `COPLANAR`, `FRONT`,
  `BACK`, and `SPANNING` translation at the `csgrs` boundary
- centralize epsilon or approximate comparison behavior in `hyperlimit`
- leave room for a later `hyperreal` backend behind the same predicate surface

After this phase, `hyperlimit` should be able to replace the f64/epsilon
predicate machinery in `csgrs` without requiring a full scalar or public API
port.

`decorum` should remain a transition and evaluation tool unless it proves to be
the right implementation detail for the approximate backend. The long-term
numeric ownership model still routes scalar semantics through `hyperreal`; any
`decorum` use should either disappear behind `hyperlimit`/`hyperlattice`
backend traits or become an internal f64 compatibility backend.

### Phase 4: Introduce explicit adapter boundaries

Add internal conversion helpers between current `csgrs` numeric types and the
new stack:

- `nalgebra::Point2<Real>` to `hyperlimit::Point2<RealLike>`
- `nalgebra::Point3<Real>` to `hyperlimit::Point3<RealLike>`
- primitive `Real` values to finite, ordered approximate scalar wrappers
  considered in Phase 3, including any `decorum`-backed candidate
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

During the approximate f64 phase, the successor module may expose internal
aliases for `decorum`-backed finite/ordered f64 wrappers where deterministic
ordering or hashing is needed. These aliases should remain private or clearly
experimental until the public scalar story is settled.

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

## Porting test plan

The port should be treated as a behavior-preserving migration with deliberately
expanded adversarial coverage. Every phase should add tests before replacing
production code, and every lower-stack adapter should be tested as a semantic
boundary rather than as a simple conversion helper.

Test categories used throughout:

- **golden regression tests**: fixed examples that lock current expected
  behavior or intentionally document a behavior change
- **differential tests**: compare old f64/epsilon behavior, BSP behavior,
  boolmesh behavior, `hyperlimit` approximate behavior, and later `hyperreal`
  behavior
- **property tests**: broad `proptest` coverage over finite coordinates,
  near-degenerate coordinates, transforms, and topology-preserving operations
- **fuzz tests**: byte/input fuzzers for constructors, imports, booleans,
  triangulation, splitting, transforms, and export round trips
- **metamorphic tests**: verify that translated, rotated, scaled, reversed,
  reordered, or triangulated equivalent inputs produce equivalent decisions
- **oracle tests**: use simple analytic cases, exact rational cases, known
  convex solids, and independent kernels as cross-checks
- **panic-safety tests**: invalid or non-finite input must either be rejected,
  converted to an explicit uncertainty/error, or produce an empty result by
  policy; it must not accidentally panic in public APIs unless documented
- **performance guard tests**: keep pathological migrations from creating
  quadratic or unbounded blowups where a cached AABB, indexed mesh, or TriMesh
  acceleration structure is expected

### Global test invariants

These invariants should be reused across phases and backends:

- all public constructors either reject non-finite coordinates or normalize them
  through one explicit policy
- all `Mesh`, `Sketch`, `BMesh`, `Polygon`, `Vertex`, and `Nurbs` outputs have
  finite coordinates unless the API explicitly reports failure
- boolean output metadata follows the documented `csgrs` policy, independent of
  predicate backend
- transformations invalidate cached bounding boxes and query meshes
- cached bounding boxes match recomputed bounding boxes after mutation,
  conversion, splitting, boolean ops, triangulation, and import
- query acceleration caches never change answers relative to uncached paths
- equivalent geometry under translation, uniform scale, rotation, mirror, vertex
  reordering, and polygon ordering produces equivalent classifications and
  topology where the operation is invariant
- topology-affecting tolerance decisions are centralized and observable in tests
- all feature combinations that select a numeric backend compile, and invalid
  feature combinations fail at compile time

### Phase 1 audit tests: current f64 and epsilon behavior

Add tests that record the current behavior before changing code:

- `float_types::tolerance`
  - default value for `f32` and `f64`
  - `CSGRS_TOLERANCE` compile-time override
  - `set_tolerance` one-shot behavior
  - values below `Real::EPSILON`
  - negative, zero, NaN, infinity, and subnormal override attempts
  - consistency across test threads and repeated calls
- `mesh::plane::Plane`
  - construction from exactly coplanar triangles
  - construction from nearly collinear vertices
  - construction from duplicate vertices
  - `normal`, `offset`, `flip`, `from_normal`, `from_points`, and
    `from_vertices`
  - point classification at exactly zero distance
  - point classification at `0.5 * tolerance`, `tolerance`,
    `2 * tolerance`, and powers of ten around tolerance
  - polygon classification for all-coplanar, all-front, all-back, spanning,
    nearly spanning, and vertex-on-plane polygons
  - `Plane::eq` for parallel planes, antiparallel planes, offset planes, and
    planes with normals that differ only inside tolerance
- `polygon::Polygon`
  - `new`, `flip`, `edges`, `bounding_box`, `triangulate`,
    `subdivide_triangles`, `set_new_normal`, and metadata access
  - clockwise and counterclockwise winding
  - repeated first/last vertex and repeated internal vertices
  - zero-area, needle, bow-tie, and nearly bow-tie rings
  - tiny triangles around subnormal and tolerance boundaries
  - huge coordinates close to `Real::MAX.sqrt()`
  - polygons whose projected 2D basis is near singular
- `vertex::Vertex`
  - `flip`, `interpolate`, `slerp_interpolate`, distance helpers,
    barycentric interpolation, cotangent weights, quality metrics, and cluster
    helpers
  - interpolation at `t < 0`, `0`, `0.5`, `1`, `> 1`, NaN, and infinity
  - normal interpolation for zero, opposite, and nearly parallel normals
  - position equality and hashing assumptions used by connectivity code
- BSP behavior
  - `Node::new`, `from_polygons`, `invert`, `clip_polygons`, `clip_to`,
    `all_polygons`, `build`, and `slice`
  - empty BSP
  - one polygon
  - coplanar polygon stacks
  - sorted, reverse-sorted, and randomized polygon order
  - sliver triangles
  - many coincident planes
  - parallel feature parity with sequential BSP
- CSG behavior
  - `union`, `difference`, `intersection`, `xor`, `inverse`, transforms,
    `center`, `float`, and bounding box behavior
  - cube/cube, cube/sphere, sphere/sphere, disjoint solids, touching solids,
    nested solids, coincident solids, inverted solids, and empty operands
  - metadata propagation for every boolean operation
  - bounding-box optimized and non-optimized mesh boolean outputs

### Phase 2 hyperlimit parity and stress tests

For every predicate added to `hyperlimit`, create parity fixtures and property
tests that encode `csgrs` needs:

- `orient2d`
  - exact clockwise, counterclockwise, and collinear triples
  - nearly collinear triples around tolerance
  - duplicated points
  - huge and tiny coordinates mixed in one input
  - translated, rotated, uniformly scaled, and point-order-permuted inputs
  - proptest-generated triangles with analytic sign computed from high
    precision or exact rational inputs where possible
- `orient3d`
  - tetrahedra with known positive, negative, and zero volume
  - nearly coplanar fourth point
  - degenerate base triangle
  - all point permutations and expected sign flips
  - transformed tetrahedra under affine transforms with positive and negative
    determinant
- `incircle2d` and `insphere3d`
  - points exactly on circle/sphere
  - inside/outside perturbations around tolerance
  - duplicated and cocircular/cospherical input points
  - orientation-dependent sign conventions
  - differential checks against robust predicates, exact arithmetic, or
    high-precision fixtures
- point-line and point-plane classification
  - exactly on boundary, near boundary, and far from boundary
  - normalized and unnormalized lines/planes
  - reversed line direction and flipped plane normal
  - explicit uncertainty band tests
- oriented point-plane classification
  - `FRONT`, `BACK`, `COPLANAR`, and uncertain cases
  - translation/scale invariance
  - plane represented as implicit coefficients and as oriented point triples
- batch predicate cases
  - stable ordering of batched results
  - no hidden allocation growth across large batches
  - identical answers for scalar and batched evaluation

Required fuzz targets:

- fuzz 2D predicate triples and compare classification invariants
- fuzz 3D predicate quadruples and compare classification invariants
- fuzz finite/non-finite scalar payloads into approximate backend adapters
- fuzz serialized parity fixtures so corpus entries survive backend changes

### Phase 3 approximate backend and decorum tests

The approximate backend must prove that it centralizes f64 policy:

- finite scalar wrapper tests
  - accepts ordinary finite values
  - rejects, sanitizes, or explicitly represents NaN and infinities
  - preserves signed zero policy intentionally
  - handles subnormals, `MIN_POSITIVE`, and very large finite values
  - total ordering is deterministic and documented
  - hashing/equality are compatible with map/set use
- `decorum` evaluation tests
  - candidate wrapper equality vs `approx` equality
  - candidate wrapper ordering for `-0.0`, `0.0`, subnormal, finite extremes,
    NaN, and infinities according to the chosen policy
  - serde or debug formatting if wrappers may cross diagnostics
  - conversion round trips through `Real`
  - no accidental public API leakage
- uncertainty representation tests
  - all predicate result variants are constructible and matchable
  - uncertain classification can be translated to current `csgrs` behavior at
    the adapter boundary
  - uncertain results can be logged or inspected in diagnostics
  - uncertainty is not silently coerced in lower crates
- epsilon centralization tests
  - there is one source of tolerance policy for f64 approximate predicates
  - per-call override behavior, if any, is explicit
  - old local tolerance helpers are not used after replacement

### Phase 4 adapter boundary tests

Every adapter should have round-trip, invalid-input, and invariance tests:

- `nalgebra::Point2<Real>` to `hyperlimit::Point2`
  - finite round trip
  - NaN/infinity rejection policy
  - signed zero policy
  - huge/tiny coordinate preservation
- `nalgebra::Point3<Real>` to `hyperlimit::Point3`
  - same as `Point2`
  - points produced by mesh constructors and importers
- primitive `Real` to finite/ordered scalar wrappers
  - conversion from all relevant scalar constants
  - proptest over finite values
  - fuzz over raw bit patterns
- `mesh::plane::Plane` to `hyperlimit::Plane3`
  - implicit coefficients preserve classification
  - oriented triples preserve classification
  - flipped planes invert classification
  - degenerate planes produce explicit failure/uncertainty
- `Vertex` to predicate point
  - normals and metadata are ignored by predicate adapters
  - position conversion is exact or explicitly approximate
  - invalid vertices are rejected before use in topology decisions
- `Matrix4<Real>` to `hyperlattice` transform
  - identity, translation, rotation, scale, mirror, shear, perspective-like
    invalid matrices, singular matrices, and near-singular matrices
  - transform composition order parity
  - inverse transform behavior where supported
- collection adapters
  - empty slices
  - one item
  - duplicate items
  - large vectors
  - borrowed slice lifetime and allocation behavior

### Phase 5 predicate replacement tests in csgrs

Replacement should be guarded by differential tests that run old and new paths
side by side while both exist:

- `Plane::eq` differential tests for every audit fixture
- polygon splitting differential tests
  - split by axis planes
  - split by arbitrary oblique planes
  - split with one vertex on plane
  - split with one edge on plane
  - split with entire polygon coplanar
  - split with nearly coincident edges
  - split n-gons, triangles, quads, holes where applicable, and invalid input
  - verify side polygons preserve metadata and winding policy
- BSP differential tests
  - old BSP vs hyperlimit-backed BSP polygon counts and bounding boxes
  - old BSP vs new BSP boolean outputs on fixture catalog
  - old BSP vs new BSP slice outputs
  - parallel vs sequential outputs under the new predicate layer
- classification translation tests
  - `hyperlimit` result to `COPLANAR`, `FRONT`, `BACK`, `SPANNING`
  - explicit tests for uncertain classifications
  - policy tests for choosing split vs snap vs reject
- removal tests
  - static search tests or lint checks that no direct robust predicate or local
    epsilon classification path remains in replaced modules
  - compile tests that lower crates do not depend on `csgrs`

### Phase 6 alias and type routing tests

When point/vector/matrix imports move behind aliases:

- compile tests for every module using crate-local aliases
- no accidental mixed `nalgebra::Point3` and alias imports in migrated modules
- doc examples compile with existing public types
- downstream compatibility tests using `nalgebra` inputs still compile
- feature tests for `f32`, `f64`, `parallel`, `mesh`, `sketch`, `bmesh`,
  `wasm`, `nurbs`, IO features, and triangulation choices
- serialization/export tests still emit identical or intentionally updated
  output for representative geometry
- proptest transform round trips through alias types
- fuzz transform matrices through alias-backed transform APIs

### Phase 7 transform and constructor tests

Every transform-heavy API should be tested under exact, approximate, and
invalid inputs:

- `translate`
  - zero, positive, negative, huge, tiny, NaN, infinity
  - vector and component overloads
  - metadata preservation
  - bounding box and cached TriMesh invalidation
- `rotate`
  - axes at 0, 90, 180, 270, 360 degrees
  - arbitrary angles
  - repeated rotations that should compose to identity
  - gimbal-like cases and quaternion parity where available
  - normals remain unit or follow documented normalization policy
- `scale`
  - uniform, non-uniform, zero, negative, mirror-equivalent, huge, tiny
  - orientation and normal behavior
  - volume/mass scaling for simple solids
- `mirror`
  - XY, YZ, XZ, arbitrary planes
  - normal flipping
  - boolean parity after mirror
- `transform`
  - identity
  - composition parity with translate/rotate/scale
  - singular and near-singular matrices
  - affine vs non-affine inputs
- shape constructors in `Sketch`
  - square, rectangle, circle, polygon, rounded rectangle, ellipse,
    regular_ngon, arrow, right_triangle, trapezoid, star, teardrop, egg,
    squircle, keyhole, reuleaux, ring, pie_slice, supershape,
    circle_with_keyway, circle_with_flat, circle_with_two_flats, from_image,
    text, metaballs, airfoil_naca4, bezier, bspline, heart, crescent,
    hilbert_curve, involute_gear, cycloidal gear/rack paths
  - minimum segments, zero segments, one/two segments, large segments
  - zero/negative dimensions
  - very large and very small dimensions
  - finite output and bounding box sanity
  - triangulation and extrusion sanity where closed
- shape constructors in `Mesh`
  - cube, cuboid, sphere, cylinder, frustum, frustum_ptp, polyhedron,
    octahedron, icosahedron, torus, egg, teardrop, teardrop_cylinder,
    ellipsoid, metaballs, sdf, arrow, gyroid, schwarz_p, schwarz_d,
    spur_gear_involute, helical/cycloidal gear paths
  - minimum segments/stacks
  - degenerate radii/heights
  - start equals end for point-to-point shapes
  - invalid face indices in polyhedron
  - non-manifold and self-intersecting polyhedron inputs
  - finite output, bounding box, manifold expectation where applicable
- extrusions and sweeps
  - `extrude`, `extrude_vector`, `revolve`, `loft`, and `sweep`
  - open vs closed sketches
  - holes and nested holes
  - zero, negative, huge, and tiny height/vector/path segments
  - self-intersecting paths
  - profile with duplicate points
  - cap winding and side normal consistency
  - path frames across sharp turns and nearly parallel tangents

### Phase 8 scalar configurability tests

Before changing public scalar modes:

- full test matrix for `--no-default-features --features f32,...`
- full test matrix for `--no-default-features --features f64,...`
- compile-fail tests for selecting both `f32` and `f64`
- compile-fail tests for selecting no scalar backend
- result comparison between `f32` and `f64` for low-resolution simple fixtures
- tolerance default and override behavior for both scalar modes
- serialization round trips do not accidentally depend on scalar mode except for
  precision
- public docs state which scalar mode is active in examples
- backend-generic experiments, if added, have type inference and error-message
  tests

### Phase 9 hyperreal backend tests

The hyperreal backend should be tested as a semantic backend, not just a type
replacement:

- exact rational coordinate fixtures for every predicate
- structural equality and zero tests
- lazy approximation refinement tests
- conservative comparison tests that intentionally remain uncertain until
  refined
- certified sign tests for orientation and in-sphere predicates
- fallback-to-approx tests where exact resolution is too expensive
- performance limits for pathological refinement
- no accidental conversion to f64 in hyperreal predicate paths
- differential tests against approximate f64 backend for ordinary inputs
- intentional divergence tests where f64 epsilon was wrong or unstable
- serialization/debug output for failure diagnostics
- fuzz tests that generate small exact expressions and transformed variants

### Phase 10 public API cleanup tests

Public cleanup needs compatibility tests:

- crate docs compile
- README examples compile
- examples compile under default features
- examples compile under minimal feature sets where intended
- downstream-style tests using `nalgebra` inputs still compile
- deprecation warnings appear only where intended
- removed helpers have migration notes and compile-fail tests if practical
- re-exported lower-stack types are stable and documented
- no lower-stack implementation detail leaks into IO or metadata APIs

### Boolean layer tests: BSP, BMesh, boolmesh, and Mesh

Because boolmesh is the desired mesh boolean layer and BSP is the early
verification layer, keep a standing differential suite:

- `Mesh` BSP vs `BMesh` boolmesh on:
  - cube/cube overlap
  - cube/sphere difference
  - sphere/sphere union/intersection
  - touching faces
  - touching edges
  - touching vertices
  - nested solids
  - disjoint solids
  - coincident solids
  - inverted solids
  - open/non-manifold inputs
  - many tiny translated copies
- conversion tests:
  - `Mesh -> BMesh -> Mesh`
  - `BMesh -> Mesh -> BMesh`
  - metadata expectations across conversions
  - triangle soup inputs accepted by boolmesh adapter
  - invalid boolmesh construction errors are explicit
  - coordinate precision loss is measured and bounded
- boolean algebra tests:
  - commutativity of union and intersection
  - non-commutativity of difference
  - associativity checks for simple fixtures where stable
  - idempotence: `a union a`, `a intersection a`, `a difference empty`
  - complement/inverse identities where supported
- topology tests:
  - manifoldness
  - edge use count
  - closed volume expectation
  - outward normals
  - no NaN normals
  - no zero-area output polygons unless explicitly tolerated
- performance tests:
  - cached AABB partitioning avoids splitting obviously disjoint polygons
  - polygon count growth is bounded on disjoint solids
  - repeated booleans do not reuse stale bounding boxes or query meshes

### Triangulation tests: Spade, Earcut, Delaunay, and hyperreal ports

Triangulation is a central risk area because it bridges topology and numeric
predicates:

- feature matrix:
  - `delaunay`
  - `earcut`
  - `delaunay-rs`
  - compile-fail for multiple selected triangulators
  - compile-fail for none selected
- polygon cases:
  - triangle, quad, concave polygon, polygon with one hole, polygon with
    multiple holes, nested invalid holes, self-intersecting ring, duplicate
    vertices, repeated closing vertex, collinear runs, sliver ears
  - clockwise and counterclockwise winding
  - holes with reversed winding
  - disconnected multipolygons
- invariants:
  - total triangle area equals polygon area within backend policy
  - no triangle centroid outside polygon for constrained triangulation
  - triangle normals match source polygon normal after 3D projection
  - no duplicate or inverted triangles after round trip
  - triangulation is stable under translation and uniform scale
- projection tests:
  - polygons in XY, YZ, XZ, and arbitrary oblique planes
  - near-axis-aligned normals
  - near-singular projection bases
  - huge/tiny coordinate projection
- fuzz targets:
  - random simple polygons
  - polygons with holes
  - invalid rings
  - byte-derived point clouds filtered into rings
  - triangulate then export/import round trips

### Contour, offset, and cavalier_contours tests

When moving 2D contour logic toward a hyperreal-aware path:

- offset tests:
  - positive, negative, zero, huge, tiny offsets
  - square, circle approximation, concave polygon, star, crescent, keyhole,
    ring, and text outlines
  - offsets that collapse geometry
  - offsets that create self-intersections
  - miter, bevel, and rounded join behavior where available
- straight skeleton tests:
  - convex, concave, holes, degenerate, near-parallel edges
  - inside vs outside orientation
- contour cleanup tests:
  - duplicate adjacent points
  - near-duplicate points
  - collinear simplification
  - tiny loops
  - nested loops
  - winding normalization
- differential tests:
  - current `geo-buffer`/offset behavior vs new contour backend
  - f64 transitional path vs hyperreal path
  - known simple analytic offsets
- fuzz tests:
  - random closed polylines
  - SVG-imported paths
  - image-derived contours
  - repeated offset/inset cycles

### Curvo, NURBS, and harvested-kernel tests

Curvo can remain useful as an f64 adapter while algorithms are harvested:

- `Nurbs::empty`, `from_regions`, `from_compound`, `from_curve`, `polyline`,
  `rectangle`, `circle`, `tessellate_regions`, `to_multipolygon`,
  `to_sketch`, `extrude_vector`, `try_union`, `try_difference`, and
  `try_intersection`
- curve cases:
  - line segment
  - polyline with duplicate points
  - closed polyline
  - circle and near-circle
  - tiny and huge radii
  - self-intersecting curve
  - multiple regions
- boolean cases:
  - overlapping rectangles
  - tangent circles
  - nested regions
  - disjoint regions
  - invalid region handling
- tessellation cases:
  - tolerance sweep
  - monotonic improvement with smaller tolerance
  - finite output
  - no inverted rings after conversion
- adapter tests:
  - Curvo f64 input to `csgrs` `Real`
  - later harvested algorithm to hyperreal backend
  - differential Curvo result vs harvested result on fixtures

### IO and serialization tests during numeric migration

IO should remain a `csgrs` responsibility and should not expose backend
internals:

- STL ASCII/binary:
  - export/import round trip
  - NaN/infinity rejection
  - normal regeneration
  - metadata loss policy
- OBJ:
  - invalid face indices
  - n-gons
  - duplicate vertices
  - negative indices if supported
  - comments and object names
- PLY/AMF/glTF:
  - coordinate precision
  - color/metadata where supported
  - empty mesh behavior
- DXF/SVG/Gerber:
  - arcs, lines, polylines, closed paths, open paths
  - malformed input fuzzing
  - coordinate scaling and unit assumptions
- image import:
  - empty images
  - all-black/all-white images
  - one-pixel features
  - threshold sweeps
  - large images and memory bounds
- cross-backend:
  - exported geometry from f64 approximate backend imports under hyperreal mode
  - hyperreal-generated output can be approximated for ordinary file formats
  - IO paths never require `hyperreal` public types unless explicitly enabled

### Toolpath and CAM tests

Toolpath code depends heavily on topology and offsets:

- `fdm_layer_from_sketch`
  - simple polygon, holes, open lines, multiple islands
  - extrusion/retraction markers if present
  - layer height and nozzle width sweeps
- `cut2d_contours`, `contour_only`, and `pocket2d`
  - inside/outside kerf side
  - lead-in and lead-out behavior
  - pocket step-over sweeps
  - collapsed offsets
  - invalid tool diameter and feed settings
- `lathe_rough_from_profile`
  - monotone and non-monotone profiles
  - profile touching axis
  - negative radius rejection
- G-code output
  - finite coordinates
  - no unintended rapid/cut mode changes
  - stable formatting across backends
  - no NaN/infinity strings
- fuzz tests:
  - random sketches into toolpath generators
  - byte-derived feed/tool configs
  - malformed but finite profile paths

### WASM tests

WASM bindings must preserve the Rust API contract:

- constructor parity between Rust and JS-facing APIs
- boolean parity for small fixtures
- transform parity
- bounding box and invalidation behavior
- typed-array layout for positions, normals, indices, and matrices
- metadata serialization/deserialization policy
- exception/error behavior for invalid inputs
- feature-gated functions unavailable when their features are off
- wasm-pack build smoke tests
- browser-side snapshot tests for representative APIs where practical

### Caches and acceleration structure tests

The port should expand cache-specific coverage:

- `Polygon::bounding_box`
  - computed once and reused
  - correct after clone
  - invalidated or rebuilt after operations that mutate vertices
  - exact for empty-disallowed and degenerate polygons
- `Mesh::bounding_box`
  - empty mesh behavior
  - stale-cache prevention after transforms, booleans, triangulation,
    smoothing, subdivision, conversion, import, and inverse
- `Mesh::query_trimesh`
  - `contains_vertex` parity with uncached ray path
  - `ray_intersections` parity with direct triangle loop
  - `intersect_polyline` parity
  - stale-cache prevention after transform and mutation
- `BMesh::bounding_box`
  - boolmesh-derived bounds parity with converted `Mesh`
- performance guards:
  - repeated query does not rebuild TriMesh each time
  - repeated bounding-box access is effectively constant time
  - disjoint boolean inputs use AABB pruning under `mesh-bbopt`

### Property-test strategy

Add shared `proptest` strategies for:

- finite scalar values with configurable magnitude bands
- raw scalar bit patterns for adapter rejection/sanitization
- points in 2D and 3D
- near-duplicate points
- nearly collinear triples
- nearly coplanar quadruples
- simple polygons
- invalid polygons
- convex polyhedra from small point sets
- transform matrices
- planes from point triples and normal/offset pairs
- mesh primitive parameter sets
- sketch primitive parameter sets
- boolean operation trees
- IO names and metadata payloads

Each strategy should have small, medium, and stress modes so ordinary CI can run
quickly while long adversarial jobs can sweep much larger domains.

### Fuzz-target strategy

Keep and extend fuzzing around:

- plane split polygon
- mesh boolean pairs
- sketch boolean pairs
- mesh primitive catalog
- sketch shape catalog
- sketch polygon triangulation
- extrusion/revolve/sweep
- transform matrices
- mesh polyhedron constructor
- vertex arithmetic and quality
- SDF and TPMS generation
- OBJ, DXF, SVG, Gerber import
- export names and format writers
- toolpaths from sketches
- adapter boundaries for `hyperlimit`, `hyperlattice`, and `hyperreal`
- differential old/new predicate paths while both exist

Fuzz corpora should include minimized regressions from `adversarial.txt`, known
crash fixtures, fixtures from real CAD files, and generated cases around every
tolerance threshold.

### Assumption tests and release gates

Before each migration phase is considered complete:

- every assumption in that phase has at least one unit or property test
- every changed predicate has parity fixtures and adversarial fixtures
- every changed public function has ordinary, boundary, invalid, and fuzz
  coverage
- old and new backends run in differential mode for at least one release window
  or one internal milestone
- test failures identify whether the failure is numeric, topological, metadata,
  IO, or performance related
- README examples, crate docs, examples, adversarial scripts, fuzz targets, and
  feature-matrix checks all compile against the selected backend
- intentional behavior changes are documented in migration notes and backed by
  before/after tests

## External robustness research and implementation plans

This section converts lessons from professional CAD kernels, computational
geometry libraries, mesh boolean systems, numerical analysis, and public CAD
datasets into concrete `csgrs` implementation work. The intent is not to copy
another kernel. The intent is to harvest their failure modes, test discipline,
and robustness boundaries.

### CGAL-style exact predicates and inexact constructions

CGAL's kernel design is the clearest model for separating predicates from
constructions. Exact predicates drive control flow; inexact constructions are
allowed only when downstream correctness can tolerate approximate constructed
coordinates.

Implementation plan:

- define a `PredicateKernel` trait in the successor predicate boundary with
  orientation, in-circle, in-sphere, point-line, point-plane, and distance
  comparison predicates
- define a separate `ConstructionKernel` or construction adapter layer for
  midpoint, split point, projected point, transformed point, curve sample, and
  intersection-point construction
- forbid construction helpers from returning plain booleans or topology
  decisions
- add lint/search gates that prevent direct `Real` determinant signs in CSG
  control flow once the predicate layer is active
- model the first backend after CGAL's exact-predicate/inexact-construction
  split: f64 storage with robust predicate classification
- add a second backend that uses exact or hyperreal constructions for selected
  operations, starting with polygon splitting and surface intersection points
- include "construction poisoning" tests where an inexact split point is reused
  by later predicates; the test should prove whether the backend refines,
  reports uncertainty, or preserves the current f64 behavior intentionally

Failure modes to encode:

- exact predicate says a point is on one side, but inexact constructed split
  point drifts to the other side
- distance comparison implemented by subtracting approximate distances instead
  of using a robust comparison predicate
- mixed exact and inexact paths make contradictory decisions on the same input
- a construction silently introduces NaN, infinity, or signed-zero dependence

Show-off examples:

- four nearly coplanar tetrahedron points where f64 orientation flips under
  translation but the predicate backend remains stable
- two large-coordinate planes whose closest-distance ordering is stable only
  with a dedicated distance-comparison predicate
- a polygon split where every new split vertex certifies as coplanar with the
  splitting plane under the predicate backend

### OCCT DRAW, BOP, and shape-healing discipline

Open CASCADE's DRAW Test Harness and Boolean Operation argument checks are a
good model for making bad input diagnosable. `csgrs` should have equivalent
scriptable checks for mesh/sketch validity, boolean readiness, and repair
status.

Implementation plan:

- add a `diagnostics` module that can produce structured reports for `Mesh`,
  `Sketch`, `Polygon`, `BMesh`, and imported shapes
- create `MeshCheckStatus` and `SketchCheckStatus` bitflags or enums inspired
  by BRepCheck/BOP argument checks
- report self-intersections, too-small edges, too-close vertices, bad faces,
  non-manifold edges, open boundaries, inconsistent orientation, non-finite
  coordinates, stale caches, and unsupported topology
- add CLI/example commands that load files, run diagnostics, and print stable
  machine-readable output
- add repair passes only after diagnostics: deduplicate vertices, drop
  zero-area triangles, orient components, close tiny cracks by policy, and
  re-normalize faces
- keep repair operations non-magical: every repair returns a report that lists
  edits, tolerances, and whether topology changed
- build "argument check before boolean" helpers for BSP and boolmesh paths
- add fixture tests where bad input is diagnosed but not repaired, diagnosed and
  repaired, and rejected because repair would change semantics too much

Failure modes to encode:

- boolean operation starts on unsupported input and fails deep in splitting
- automatic repair deletes important small features without reporting it
- tolerance growth turns separate parts into touching or overlapping parts
- "fixed" geometry is no longer watertight or has changed component count
- warnings are emitted but not available to tests or downstream tooling

Show-off examples:

- a mesh with high vertex tolerance equivalents that triggers too-close vertex
  and too-small edge diagnostics before boolean operations
- a broken STL from a fixture corpus that can be repaired into a manifold mesh
  with a complete edit report
- a failed boolean that produces a diagnostics bundle identifying the first
  unsupported subshape pair

### Real-world bad mesh corpora: Thingi10K and repair datasets

Thingi10K is valuable because it reflects actual printable models rather than
clean academic inputs. It explicitly includes non-solid, self-intersecting,
coplanar self-intersecting, multi-component, non-manifold, degenerate,
non-oriented, and open meshes.

Implementation plan:

- create a small checked-in "Thingi10K-style" fixture subset with licensing
  notes, or scripts that download a configured external corpus outside ordinary
  CI
- classify fixtures by failure tag: non-solid, self-intersection, coplanar
  self-intersection, multiple components, non-manifold, degenerate faces,
  non-PWN, open boundary, and non-oriented
- add import-only tests that guarantee no panic and deterministic diagnostics
- add repair tests for each class with explicit expected status transitions
- add boolean smoke tests for repaired fixtures against cube/sphere cutters
- add export/import round-trip tests after repair
- add long-running benchmark scripts that run diagnostics over thousands of
  real-world STL meshes and summarize status counts

Failure modes to encode:

- slicer-style "repair" hides non-manifoldness by guessing wrong caps
- zero-area triangles break normal calculation or triangulation
- self-intersections create contradictory inside/outside classification
- multiple components confuse boolean algebra and mass properties
- open surfaces are treated as solids without an explicit policy

Show-off examples:

- batch diagnostics over a real-world mesh corpus with a status histogram
- a repair pipeline that turns a selected open/non-oriented mesh into a
  boolean-ready manifold or reports why it cannot
- side-by-side BSP/boolmesh behavior on repaired vs unrepaired fixtures

### CAD B-rep benchmark datasets: ABC, Fusion 360 Gallery, MCB, and CADBench

CAD datasets are useful even when `csgrs` is not a B-rep kernel. They provide
real modeling sequences, analytic surfaces, mechanical part categories,
assembly/joint patterns, and STEP-derived topology that can be converted into
stress fixtures.

Implementation plan:

- add optional external-data scripts for ABC and Fusion 360 Gallery-derived
  STEP/mesh fixtures
- extract primitive patterns relevant to `csgrs`: extrude, cut, revolve,
  fillet-like geometry, chamfer-like geometry, holes, slots, bosses, ribs,
  thin walls, gears, brackets, hinges, and fastener-like shapes
- build a "professional CAD shape catalog" in examples/tests from simplified
  parametric replicas rather than storing large proprietary-style assets
- create benchmark tiers by face count, component count, feature count, and
  operation count
- add geometry fidelity checks: bounding box, volume, surface area, component
  count, manifoldness, ray hits, and boolean stability
- add reconstruction-style tests where a generated `csgrs` program exports a
  mesh and compares against a reference mesh envelope
- use MCB-style mechanical categories to ensure coverage of bearings, gears,
  bolts, nuts, washers, brackets, housings, impellers, springs, clips, and
  enclosures

Failure modes to encode:

- simple visual similarity hides wrong topology
- reconstruction succeeds for primitives but fails under feature interactions
- holes and fillets create tiny sliver faces after triangulation
- CAD-derived meshes contain analytic surfaces sampled at inconsistent density
- imported STEP/mesh boundaries lose semantic feature identity

Show-off examples:

- a README/example gallery of mechanical fixtures generated entirely from
  `csgrs` operations
- a "CADBench-lite" suite that runs generated parts through export/import,
  booleans, diagnostics, and render generation
- operation-sequence stress tests modeled after human-authored sketch/extrude
  timelines

### Exact and robust mesh boolean systems

Modern mesh boolean work emphasizes arrangements, exact or indirect predicates,
regularized output, provenance, and robust handling of degeneracies. `csgrs`
should use BSP as the verification layer but move production mesh booleans
toward boolmesh or a similarly indexed arrangement-backed layer.

Implementation plan:

- define a `BooleanBackend` test harness with BSP, boolmesh/BMesh, and any
  future arrangement backend behind the same fixture runner
- for every fixture, record operation, backend, output polygon count, bounding
  box, manifoldness, component count, volume estimate, and diagnostics
- add provenance tests inspired by arrangement systems: output faces should be
  traceable to input A, input B, generated intersection faces, or repair passes
- add regularization tests: dangling sheets, internal faces, zero-thickness
  overlaps, coincident faces, and non-manifold intersections should have
  explicit output policy
- add iterated-boolean tests that apply hundreds of operations and verify no
  progressive topological decay
- add exact/approx backend differential tests around line-plane intersection,
  triangle-triangle intersection, and cell classification
- add performance thresholds for large triangle counts, many disjoint parts,
  many coplanar faces, and repeated identical operations

Failure modes to encode:

- cracks along intersection curves
- duplicate coincident faces after union
- missing caps after difference
- inverted components after intersection
- non-manifold output after touching-edge or touching-vertex booleans
- iterative CSG accumulates tiny cracks or sliver faces
- backend claims success but returns empty or non-solid output

Show-off examples:

- "boolean torture cube": repeated holes, slots, coincident cuts, and tangent
  cutters with stable output
- iterated CSG stress scene with hundreds of operations and bounded topology
  growth
- differential report showing where BSP and boolmesh intentionally diverge and
  which backend policy wins

### Manifold test-suite harvesting

Manifold is especially relevant because its stated goal is guaranteed manifold
output for triangle-mesh operations, and its repository includes a substantial
GoogleTest-based suite plus fuzz targets. Upstream is Apache-2.0, and the test
sources carry Apache-2.0 notices, so using the suite is license-compatible with
proper attribution, retained notices for copied material, and clear separation
between directly ported fixtures and independently reimplemented scenarios.

Implementation plan:

- audit `test/polygon_test.cpp`, `test/properties_test.cpp`,
  `test/manifold_test.cpp`, `test/boolean_test.cpp`,
  `test/boolean_complex_test.cpp`, `test/sdf_test.cpp`, `test/smooth_test.cpp`,
  `test/hull_test.cpp`, `test/samples_test.cpp`, `polygon_fuzz.cpp`, and
  `manifold_fuzz.cpp`
- create a `tests/manifold_inspired/` module whose file headers identify the
  cases as "derived from or inspired by Manifold" and point to the upstream
  Apache-2.0 license
- prefer reexpressing tests as `csgrs` constructors and operations rather than
  copying C++ test code line-for-line
- if a fixture, expected numeric value, or test topology is copied directly,
  retain the Apache-2.0 notice and add it to a third-party notices file
- categorize imported cases by feature: constructors, mesh properties,
  booleans, complex booleans, cross-section/polygon behavior, hulls, SDFs,
  smoothing/refinement, language-binding parity concepts, and fuzz corpora
- map Manifold's status checks to `csgrs` diagnostics: manifoldness, vertex and
  triangle count, genus/component expectations, bounding boxes, volume,
  surface area, face properties, winding, and open-boundary rejection
- add direct differential tests where possible: generate the same primitive or
  boolean sequence in `csgrs`, run BSP and boolmesh paths, and compare
  topological status plus coarse numeric invariants
- add long-running ignored tests for Manifold-style stress scenes such as
  Menger sponge differences, many repeated booleans, near-coplanar cutters, and
  deeply nested smooth/refine operations
- add fuzz target equivalents for `cargo fuzz`: polygon construction, boolean
  operation sequences, mesh import/repair, arbitrary triangle soup, and
  cross-section-to-extrusion paths
- document a future optional differential runner that builds upstream Manifold
  separately and compares `csgrs` outputs against Manifold outputs without
  making Manifold a normal dependency

Failure modes to encode:

- `csgrs` reports success but returns a non-manifold boolean result where
  Manifold's suite expects a closed solid
- output topology depends on operand order, thread count, or triangle ordering
- copied test fixtures accidentally lose license notices or provenance
- a test only checks visual/triangle-count similarity and misses open boundary,
  inverted normal, or wrong component count
- fuzz-discovered bad meshes shrink to a case that is hard to replay because no
  stable manifest was written

Show-off examples:

- a Manifold-inspired Menger sponge boolean stress test implemented entirely in
  Rust and checked by volume, component count, manifoldness, and render output
- a primitive/property parity table that compares `csgrs` BSP, `csgrs`
  boolmesh, and optional upstream Manifold runs
- a fuzz corpus seeded from Manifold-inspired polygon and boolean cases, with
  minimized regressions promoted into fixed Rust tests

### Watertight B-rep and tolerance-controlled boolean lessons

CAD B-rep boolean literature repeatedly identifies surface-surface
intersection, trimming, gaps, and tolerance management as central failure
points. `csgrs` mesh booleans are not B-rep booleans, but the same lesson
applies: intersection data must have a single authoritative representation and
all topology must refer to it consistently.

Implementation plan:

- introduce an internal `IntersectionGraph` or equivalent temporary structure
  for mesh boolean/splitting experiments
- store intersection vertices once and refer to them by stable IDs during
  splitting/classification
- add tolerance-control reports that record whether an intersection point was
  exact, approximate, snapped, merged, or uncertain
- make "same intersection curve represented twice" a test failure
- add gap-detection tests on boolean outputs using ray casts, boundary edge
  checks, and connected component checks
- add smoothness/normal continuity checks for cases that should preserve
  tangent continuity or at least avoid inverted normals

Failure modes to encode:

- two faces compute slightly different versions of the same intersection edge
- trimming leaves a small gap that is smaller than tolerance but topologically
  open
- tolerance merging changes surface identity or deletes narrow features
- downstream export/import reveals cracks not visible in the original mesh

Show-off examples:

- a near-tangent cutter that produces a watertight result or a precise
  uncertainty report
- a gap scanner that highlights boundary loops created by failed booleans
- a split operation where all adjacent faces share identical intersection
  vertex IDs

### Floating-point bad practices and numerical dark patterns

The port should explicitly prevent known bad practices instead of relying on
review discipline.

Implementation plan:

- add a numeric anti-pattern checklist to code review and CI
- add static searches or clippy-style checks for direct float equality in
  topology paths, `Real::EPSILON` as geometry tolerance, determinant sign tests
  outside predicates, sorting floats without explicit NaN policy, and hashing
  raw floats as topology keys
- replace naked `abs(a - b) < eps` topology decisions with named predicates
  that document absolute/relative/scaled tolerance behavior
- add magnitude-sweep tests showing why one fixed epsilon fails for tiny and
  huge coordinates
- add signed-zero tests around plane offsets, transforms, sorting, hashing, and
  serialization
- add cancellation demonstrations for distance, area, volume, and determinant
  calculations
- add tests that turn compiler/platform variation into explicit assumptions:
  fused multiply-add, fast-math-disabled behavior, deterministic sorting, and
  stable serialization precision

Failure modes to encode:

- fixed epsilon is too large near the origin and too small at large coordinates
- `float.EPSILON`/machine epsilon is mistaken for model tolerance
- subtracting nearly equal values destroys distance or area precision
- sorting floats with NaN creates nondeterministic topology
- approximate equality is not transitive but is used as an equivalence relation
- scale changes the boolean result because tolerance is not scale-aware

Show-off examples:

- a one-page executable example showing fixed epsilon failure at `1e-12`,
  `1.0`, and `1e12`
- a determinant cancellation case where f64 gives the wrong sign but
  `hyperlimit` reports the right sign or uncertainty
- a hash-map vertex merge test that fails under raw floats but passes with the
  finite ordered wrapper policy

### Classroom and adversarial robustness demonstrations

Classic robustness examples are useful as permanent tests because they are
small, explainable, and hostile to naive implementations.

Implementation plan:

- add a `tests/robustness_demos.rs` file for small, named demonstrations
- include orientation, incircle, insphere, segment intersection, point-in-polygon,
  plane classification, and convex hull-style examples
- keep each demo minimal enough to explain in a failing assertion message
- add README or developer-doc snippets showing why each demo matters
- run the same demos against f64 approximate, `hyperlimit`, and hyperreal
  backends
- preserve any old f64 failures as ignored tests or documented divergence tests
  until replacement is complete

Failure modes to encode:

- contradictory orientation decisions create impossible polygon winding
- point-in-polygon changes answer when the polygon is translated
- segment intersection misses endpoint or overlapping segment cases
- convex hull includes an interior point or drops an extreme point
- Delaunay incircle test flips for nearly cocircular points

Show-off examples:

- "translate-to-break" predicate demo
- "scale-to-break" epsilon demo
- "near-cocircular" triangulation demo
- "endpoint-on-segment" contour cleanup demo

### Professional regression harness design

Professional CAD systems invest in reproducible command harnesses, not only unit
tests. `csgrs` should have a lightweight equivalent using Rust examples,
fixtures, and scripts.

Implementation plan:

- create a `scripts/geometry-check.sh` or cargo xtask equivalent for structured
  diagnostics over files and generated fixtures
- support stable JSON output for diagnostics, boolean summaries, timing, and
  backend comparison
- define fixture manifests with input paths, feature flags, operation, expected
  diagnostics, expected output class, and expected failure policy
- add minimized failing cases to the repo and large corpora to external scripts
- add a "stop on first faulty" mode and a "full report" mode, mirroring OCCT
  style test ergonomics
- add render generation for selected failing cases so regressions are easy to
  inspect visually
- make every future bug report reducible to a fixture manifest entry

Failure modes to encode:

- test passes only because it checks polygon count rather than geometry
- benchmark compares different semantics across backends
- failing inputs cannot be minimized or replayed
- diagnostics are human-only and cannot gate CI
- visual render looks correct but topology is broken

Show-off examples:

- a single command that runs predicate demos, boolean torture tests, repair
  diagnostics, render generation, and backend comparison
- a generated Markdown/JSON report with links to rendered failure thumbnails
- a CI artifact bundle for every failed geometry regression

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
depend on point-plane classification. BSP should be the first full boolean
verification layer for the port; boolmesh remains the preferred end-state mesh
boolean layer, but should be completed after the BSP predicate and splitting
behavior is under test. A focused first change would:

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

## References

> Attene, Marco. "As-Exact-As-Possible Repair of Unprintable STL Files."
> *Rapid Prototyping Journal*, vol. 24, no. 5, 2018, pp. 855-864.

> Attene, Marco. "Indirect Predicates for Geometric Constructions." *Computer
> Aided Geometric Design*, vol. 79, 2020.

> Barki, Hichem, et al. "Exact, Robust, and Efficient Regularized Booleans on
> General 3D Meshes." *Computers & Mathematics with Applications*, vol. 70,
> no. 6, 2015, pp. 1235-1254,
> https://doi.org/10.1016/j.camwa.2015.06.016.

> Campen, Marcel, and Leif Kobbelt. "Exact and Robust (Self-)Intersections for
> Polygonal Meshes." *Computer Graphics Forum*, vol. 29, no. 2, 2010, pp.
> 397-406.

> Chernyaev, Evgeni. "Marching Cubes 33: Construction of Topologically Correct
> Isosurfaces." *CERN Document Server*, 1995,
> https://cds.cern.ch/record/292771.

> "CGAL 6.1.1 - Manual: Robustness Issues." *CGAL*, The CGAL Project,
> https://doc.cgal.org/latest/Manual/devman_robustness.html.

> "CGAL Manual: Predicates and Constructions." *CGAL*, The CGAL Project,
> https://doc.cgal.org/Manual/3.1/doc_html/cgal_manual/Kernel_d/Chapter_predicates_constructions_d.html.

> "DRAW Test Harness." *Open CASCADE Technology Documentation*, Open Cascade,
> https://dev.opencascade.org/doc/overview/html/occt_user_guides__test_harness.html.

> "Boolean Operations." *Open CASCADE Technology Documentation*, Open Cascade,
> https://dev.opencascade.org/doc/overview/html/specification__boolean_operations.html.

> "Shape Healing." *Open CASCADE Technology Documentation*, Open Cascade,
> https://old.opencascade.com/doc/occt-6.9.1/overview/html/occt_user_guides__shape_healing.html.

> Diazzi, Lorenzo, et al. "Interactive and Robust Mesh Booleans." *ACM
> Transactions on Graphics*, vol. 41, no. 6, 2022,
> https://arxiv.org/abs/2205.14151.

> Goldberg, David. "What Every Computer Scientist Should Know about
> Floating-Point Arithmetic." *ACM Computing Surveys*, vol. 23, no. 1, 1991,
> pp. 5-48, https://doi.org/10.1145/103162.103163.

> Held, Martin. "FIST: Fast Industrial-Strength Triangulation of Polygons."
> *Algorithmica*, vol. 30, no. 4, 2001, pp. 563-596.

> Jacobson, Alec, et al. "libigl: A Simple C++ Geometry Processing Library."
> *ACM SIGGRAPH Courses*, 2018, https://libigl.github.io/.

> Koch, Sebastian, et al. "ABC: A Big CAD Model Dataset for Geometric Deep
> Learning." *Proceedings of the IEEE/CVF Conference on Computer Vision and
> Pattern Recognition*, 2019, pp. 9601-9611,
> https://arxiv.org/abs/1812.06216.

> Lafage, Vincent. "Revisiting 'What Every Computer Scientist Should Know about
> Floating-Point Arithmetic.'" *arXiv*, 2020,
> https://arxiv.org/abs/2012.02492.

> Livesu, Marco, et al. "Fast and Robust Mesh Arrangements Using Floating-Point
> Arithmetic." *ACM Transactions on Graphics*, vol. 39, no. 6, 2020,
> https://pers.ge.imati.cnr.it/livesu/papers/CLSA20/CLSA20.pdf.

> Lorensen, William E., and Harvey E. Cline. "Marching Cubes: A High Resolution
> 3D Surface Construction Algorithm." *Computer Graphics*, vol. 21, no. 4,
> 1987, pp. 163-169.

> "Manifold." *GitHub*, Elalish, https://github.com/elalish/manifold.

> "Manifold License." *GitHub*, Elalish,
> https://github.com/elalish/manifold/blob/master/LICENSE.

> "Manifold Test CMake Configuration." *GitHub*, Elalish,
> https://github.com/elalish/manifold/blob/master/test/CMakeLists.txt.

> "Mesh Boolean." *libigl Documentation*, libigl,
> https://libigl.github.io/dox/mesh__boolean_8h.html.

> Patrikalakis, Nicholas M., Takashi Maekawa, and Wen-Chyung Cho. *Shape
> Interrogation for Computer Aided Design and Manufacturing*. MIT,
> https://web.mit.edu/hyperbook/Patrikalakis-Maekawa-Cho/.

> Salesin, David H. *Epsilon Geometry: Building Robust Algorithms from
> Imprecise Computations*. Stanford University, 1991,
> https://books.google.com/books/about/Epsilon_Geometry.html?id=UM5EAQAAIAAJ.

> Shewchuk, Jonathan Richard. "Adaptive Precision Floating-Point Arithmetic and
> Fast Robust Geometric Predicates." *Discrete & Computational Geometry*, vol.
> 18, no. 3, 1997, pp. 305-363,
> https://www.cs.cmu.edu/~quake/robust.html.

> Shewchuk, Jonathan Richard. "Robust Adaptive Floating-Point Geometric
> Predicates." *Proceedings of the Twelfth Annual Symposium on Computational
> Geometry*, ACM, 1996, pp. 141-150,
> https://people.eecs.berkeley.edu/~jrs/papers/robust-predicates.pdf.

> Willis, Karl D. D., et al. "Fusion 360 Gallery: A Dataset and Environment for
> Programmatic CAD Construction from Human Design Sequences." *ACM Transactions
> on Graphics*, vol. 40, no. 4, 2021,
> https://arxiv.org/abs/2010.02392.

> Zhou, Qingnan, and Alec Jacobson. "Thingi10K: A Dataset of 10,000 3D-Printing
> Models." *arXiv*, 2016, https://arxiv.org/abs/1605.04797.

> Zhou, Qingnan, et al. "Mesh Arrangements for Solid Geometry." *ACM
> Transactions on Graphics*, vol. 35, no. 4, 2016,
> https://www.cs.columbia.edu/cg/mesh-arrangements/mesh-arrangements-for-solid-geometry-siggraph-2016-compressed-zhou-et-al.pdf.
