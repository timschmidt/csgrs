<h1>
  hypermesh
  <img src="./doc/ChatGPT%20Image%20May%2012,%202026,%2005_22_15%20AM.png" alt="hypermesh logo" width="144" align="right">
</h1>

`hypermesh` is the experimental 3D mesh-topology crate in the Hyper workspace. It
contains a legacy float-oriented closed-mesh boolean engine and a newer exact-stack
boundary for mesh validation, provenance, face-pair classification, coplanar
arrangements, split plans, exact-aware boolean preflight, and feature-gated exact
boolean assembly.

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
- [hypercurve](https://github.com/timschmidt/hypercurve): curve and Bezier evidence for
  future curved-surface and intersection boundaries.
- [hyperpath](https://github.com/timschmidt/hyperpath): routing and swept-path consumers
  of exact obstacle and fixture mesh evidence.
- [hypersolve](https://github.com/timschmidt/hypersolve): residual replay and constraint
  certification for future reconstruction and fitting passes.
- [hyperphysics](https://github.com/timschmidt/hyperphysics) and
  [hypervoxel](https://github.com/timschmidt/hypervoxel): downstream consumers of mesh
  validation, mass, collision, and voxelization facts.
- [hyperdrc](https://github.com/timschmidt/hyperdrc): manufacturability checks that can
  consume exact board, package, and mesh keepout evidence.
- [hypercircuit](https://github.com/timschmidt/hypercircuit): electrical context for
  electromechanical mesh and package consumers.
- [hyperpack](https://github.com/timschmidt/hyperpack): package and enclosure metadata
  that can anchor mesh handoffs.
- [hyperparts](https://github.com/timschmidt/hyperparts): part records and footprints
  that can reference mesh, package, and placement geometry.
- [hyperevolution](https://github.com/timschmidt/hyperevolution): optimization layer for
  exact topology and geometry candidates.
- [hyperbrep](https://github.com/timschmidt/hyperbrep): boundary-representation source
  geometry for future mesh conversion and validation.
- [hypersdf](https://github.com/timschmidt/hypersdf): signed-distance evidence and
  implicit previews for mesh and voxel workflows.

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

- `Manifold`, `OpType`, `LegacyBooleanReport`, `LegacyBooleanResult`, and
  `compute_boolean_with_report` are the legacy closed-mesh boolean API.
- `exact::ExactMesh`, `ExactPoint3`, `Triangle`, `MeshFacts`, and `ValidationReport`
  describe exact-aware mesh inputs and diagnostics.
- `ValidationPolicy`, `BoundaryPolicy`, `MeshValidationFacts`, `VertexFacts`,
  `EdgeFacts`, `FaceFacts`, and `FacePlaneFacts` retain topology and determinant-form
  face-plane evidence.
- `SourceProvenance`, `ApproximationPolicy`, `PredicateUse`, and construction
  provenance records preserve import and decision history.
- `MeshFacePairClassification`, triangle-plane/triangle-triangle reports, coplanar
  reports, and intersection graphs describe local topology evidence.
- `ExactEdgeSplitPlan`, `ExactFaceSplitPlan`, `ExactBooleanPreflight`, and
  `ExactBooleanResult` describe readiness and assembly state.
- Surface, region, convex-solid, boundary-touching, winding, handoff-package, and
  consumer-readiness reports capture certified fast paths and downstream contracts.

## Precision Model

The legacy boolean engine remains float operationally. The exact path imports finite
`f64` coordinates by dyadic lifting into `hyperreal::Real` and records lossy import
policy explicitly. Integer-grid input is lifted directly into exact `Real` values, and
retained face planes keep unnormalized determinant coefficients instead of unit normals.
Exact predicates and validation reports should be the source of topology decisions as
kernels are ported.

Unresolved coplanar, boundary, or winding readiness is reported as a blocker rather than
patched with a tolerance.

Numerical explosion is controlled by preserving source rows, bounds, face planes,
predicate uses, split graphs, and readiness reports as structured artifacts. The crate
does not globally canonicalize every coordinate or expand every possible intersection
unless a downstream topology stage needs that evidence.

## Performance Model

The performance direction is to combine broad-phase pruning with exact local decisions.
Morton broad-phase, retained bounds, face-pair classification, split plans, support
DOPs, coplanar arrangement reports, and handoff packages are intended to narrow work
before expensive predicates or topology rebuilds. Feature flags keep legacy boolean,
exact validation, exact triangulation, Rayon, and Bevy/demo surfaces separable.

Future benchmarks should separate broad phase, narrow classification, split planning,
region assembly, and simplification so exactness work can be optimized without hiding
where time is spent.

## Current Status

Implemented today:

- feature-gated `exact`, `exact-triangulation`, and `legacy-boolean` paths;
- legacy `Manifold::new` and `compute_boolean_with_report` for union, subtraction, and
  intersection over closed manifold triangle meshes;
- Morton broad phase, triangle intersection kernels, topology simplification, and
  ear-clipping support in the legacy engine;
- exact mesh, bounds, facts, provenance, validation, audit, face-pair, coplanar,
  construction, split-plan, support, surface, winding, convex-solid, consumer-readiness,
  handoff-package, preflight, and exact-boolean APIs;
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
use hypermesh::exact::{ExactMesh, ValidationPolicy};

let input = ExactMesh::inspect_i64_triangles(&[
    0, 0, 0,
    1, 0, 0,
    0, 1, 0,
], &[0, 1, 2]);
assert!(input.edge_ready());

let mesh = ExactMesh::from_i64_triangles_with_policy(
    &[
        0, 0, 0,
        1, 0, 0,
        0, 1, 0,
    ],
    &[0, 1, 2],
    ValidationPolicy::ALLOW_BOUNDARY,
)?;

let facts = mesh.facts();
assert_eq!(facts.mesh.face_count, 1);
assert_eq!(facts.mesh.boundary_edges, 3);
mesh.validate_retained_state()?;
```

The legacy boolean adapter is opt-in and should be treated as an approximate runtime
surface:

```rust,ignore
use hypermesh::prelude::*;

let left = Manifold::new(&positions_a, &indices_a)?;
let right = Manifold::new(&positions_b, &indices_b)?;
let result = compute_boolean_with_report(&left, &right, OpType::Subtract)?;
assert!(result.report.used_primitive_float_adapter);
```

Use exact validation, audit, face-pair classification, split-plan, preflight,
consumer-readiness, and handoff-package reports to audit topology before relying on
boolean output.

## References

- Yap, Chee K. "Towards Exact Geometric Computation." *Computational Geometry* 7.1-2
  (1997): 3-23.
- Shewchuk, Jonathan Richard. "Adaptive Precision Floating-Point Arithmetic and Fast
  Robust Geometric Predicates." *Discrete & Computational Geometry* 18.3 (1997):
  305-363.
- Moller, Tomas. "A Fast Triangle-Triangle Intersection Test." *Journal of Graphics
  Tools* 2.2 (1997): 25-30.
- Guigue, Philippe, and Olivier Devillers. "Fast and Robust Triangle-Triangle Overlap
  Test Using Orientation Predicates." *Journal of Graphics Tools* 8.1 (2003): 25-42.
- Boissonnat, Jean-Daniel, Olivier Devillers, Sylvain Pion, Monique Teillaud, and
  Mariette Yvinec. "Triangulations in CGAL." *Computational Geometry* 22.1-3 (2002):
  5-19.
- de Berg, Mark, Otfried Cheong, Marc van Kreveld, and Mark Overmars. *Computational
  Geometry: Algorithms and Applications*. Springer.
- Preparata, Franco P., and Michael Ian Shamos. *Computational Geometry: An
  Introduction*. Springer, 1985.
- Sutherland, Ivan E., and Gary W. Hodgman. "Reentrant Polygon Clipping."
  *Communications of the ACM* 17.1 (1974): 32-42.
- Weiler, Kevin, and Peter Atherton. "Hidden Surface Removal Using Polygon Area
  Sorting." *SIGGRAPH Computer Graphics* 11.2 (1977): 214-222.
- Requicha, Aristides A. G. "Representations for Rigid Solids: Theory, Methods, and
  Systems." *ACM Computing Surveys* 12.4 (1980): 437-464.
- Lee, D. T., and Arthur K. Lin. "Generalized Delaunay Triangulation for Planar
  Graphs." *Discrete & Computational Geometry*.

## Development

Useful local checks:

```sh
cargo test
cargo test --no-default-features --features exact
cargo bench --bench exact_validation --features exact
```
