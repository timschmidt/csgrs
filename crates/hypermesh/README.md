<h1>
  hypermesh
  <img src="./doc/ChatGPT%20Image%20May%2012,%202026,%2005_22_15%20AM.png" alt="hypermesh logo" width="144" align="right">
</h1>

`hypermesh` is the experimental 3D mesh-topology crate in the Hyper workspace. It
contains a legacy float-oriented closed-mesh boolean engine and a newer exact-stack
boundary for mesh validation, provenance, face-pair classification, coplanar
arrangements, split plans, and exact-aware boolean preflight.

The crate is in transition: the legacy path is useful for current mesh boolean
experiments, while the exact path is where Hyper-native topology decisions are being
made auditable.

## Hyper Ecosystem

`hypermesh` is the 3D topology layer.

- [hyperreal](https://github.com/timschmidt/hyperreal): exact scalar import and retained
  coordinate values.
- [hyperlattice](https://github.com/timschmidt/hyperlattice): exact vector and transform
  algebra.
- [hyperlimit](https://github.com/timschmidt/hyperlimit): exact predicate policy for
  orientation, incidence, and sidedness decisions.
- [hypertri](https://github.com/timschmidt/hypertri): planar triangulation support for
  exact face-region assembly.
- [hyperphysics](https://github.com/timschmidt/hyperphysics) and
  [hypervoxel](https://github.com/timschmidt/hypervoxel): downstream consumers of mesh
  validation, mass, collision, and voxelization facts.

## Typical Mesh Boolean Problems

Mesh booleans combine geometry, topology, and cleanup in one fragile pipeline. Nearly
coplanar triangles, vertices on edges, duplicate faces, slivers, non-manifold input, and
tolerance-based repair can all change output topology. Engines also need broad-phase
pruning and locality for speed, while exact incidence decisions are needed at branch
points.

`hypermesh` splits those concerns. The legacy boolean path remains available for closed
manifold float buffers. The exact path records imported-coordinate provenance, mesh
facts, validation diagnostics, face-pair relations, split plans, coplanar arrangements,
and boolean readiness reports so topology decisions can move toward exact predicates
without globally canonicalizing every coordinate.

## Main Types

- `Manifold`, `OpType`, and `compute_boolean` are the legacy closed-mesh boolean API.
- `exact::ExactMesh`, `ExactPoint3`, `Triangle`, `MeshFacts`, and `ValidationReport`
  describe exact-aware mesh inputs and diagnostics.
- `SourceProvenance`, `ApproximationPolicy`, `PredicateUse`, and construction
  provenance records preserve import and decision history.
- `MeshFacePairClassification`, triangle-plane/triangle-triangle reports, coplanar
  reports, and intersection graphs describe local topology evidence.
- `ExactEdgeSplitPlan`, `ExactFaceSplitPlan`, `ExactBooleanPreflight`, and
  `ExactBooleanResult` describe readiness and assembly state.
- Surface, region, convex-solid, and boundary-touching reports capture certified fast
  paths before the general boolean path is complete.

## Precision Model

The legacy boolean engine remains float operationally. The exact path imports finite
`f64` coordinates by dyadic lifting into `hyperreal::Real` and records lossy import
policy explicitly. Exact predicates and validation reports should be the source of
topology decisions as kernels are ported.

Unresolved coplanar, boundary, or winding readiness is reported as a blocker rather than
patched with a tolerance.

## Performance Model

The performance direction is to combine broad-phase pruning with exact local decisions.
Morton broad-phase, retained bounds, face-pair classification, split plans, and
coplanar arrangement reports are intended to narrow work before expensive predicates or
topology rebuilds. Feature flags keep legacy boolean, exact validation, triangulation,
Rayon, and Bevy/demo surfaces separable.

Future benchmarks should separate broad phase, narrow classification, split planning,
region assembly, and simplification so exactness work can be optimized without hiding
where time is spent.

## Current Status

Implemented today:

- feature-gated `exact`, `exact-triangulation`, and `legacy-boolean` paths;
- legacy `Manifold::new` and `compute_boolean` for union, subtraction, and intersection
  over closed manifold triangle meshes;
- Morton broad phase, triangle intersection kernels, topology simplification, and
  ear-clipping support in the legacy engine;
- exact mesh, bounds, facts, provenance, validation, face-pair, coplanar, construction,
  split-plan, surface, convex-solid, and preflight APIs;
- tests, proptests, fuzz targets, examples, and exact-validation benchmarks.

Known limits: inputs must already be closed and manifold for the legacy path, and the
exact path is not yet a full replacement for every boolean/intersection/simplification
kernel.

## Installation

```toml
[dependencies]
hypermesh = "0.2.0"
```

For exact validation without legacy boolean kernels:

```toml
[dependencies]
hypermesh = { version = "0.2.0", default-features = false, features = ["exact"] }
```

## Usage

The exact-facing path is the default feature set and is the preferred boundary for new
code:

```rust,ignore
use hypermesh::prelude::*;

let mesh = ExactMesh::from_triangles(vec![
    Triangle::new(0, 1, 2),
], vec![
    ExactPoint3::from_i64(0, 0, 0),
    ExactPoint3::from_i64(1, 0, 0),
    ExactPoint3::from_i64(0, 1, 0),
])?;

let facts = mesh.facts();
let validation = mesh.validate();
assert!(validation.is_manifold_boundary_reported());
```

The legacy boolean adapter is opt-in and should be treated as an approximate runtime
surface:

```rust,ignore
use hypermesh::prelude::*;

let left = Manifold::new(&positions_a, &indices_a)?;
let right = Manifold::new(&positions_b, &indices_b)?;
let result = compute_boolean(&left, &right, OpType::Subtract)?;
```

Use exact validation, face-pair classification, split-plan, and preflight reports to
audit topology before relying on boolean output.

## Development

Useful local checks:

```sh
cargo test
cargo test --no-default-features --features exact
cargo bench --bench exact_validation --features exact
```
