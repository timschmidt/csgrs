<h1>
  hypermesh
  <img src="./doc/ChatGPT%20Image%20May%2012,%202026,%2005_22_15%20AM.png" alt="hypermesh logo" width="144" align="right">
</h1>

`hypermesh` is the experimental 3D mesh-topology crate in the Hyper workspace. It
uses Hyper-native exact scalar evidence for mesh validation, provenance, face-pair
classification, coplanar arrangements, split plans, exact-aware boolean preflight, and
exact boolean assembly.

`hyperreal` is the canonical geometry scalar. Primitive floats are only accepted at
explicit import or preview boundaries where approximation policy and provenance are
recorded.

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

`hypermesh` splits those concerns. The exact path records imported-coordinate
provenance, mesh facts, validation diagnostics, face-pair relations, split plans,
coplanar arrangements, and boolean readiness reports so topology decisions are made
from exact predicates and retained evidence.

## Main Types

- `exact::ExactMesh`, `hyperlimit::Point3`, `Triangle`, `MeshFacts`, and
  `ValidationReport` describe exact-aware mesh inputs and diagnostics.
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

Geometry is stored as `hyperreal::Real`. Finite `f64` coordinates can be imported by
dyadic lifting with lossy import policy recorded explicitly; integer-grid input is
lifted directly into exact `Real` values. Retained face planes keep unnormalized
determinant coefficients instead of unit normals. Exact predicates and validation
reports are the source of topology decisions.

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
before expensive predicates or topology rebuilds. Feature flags are reserved for
diagnostic/probing hooks and Bevy/demo surfaces.

Future benchmarks should separate broad phase, narrow classification, split planning,
region assembly, and simplification so exactness work can be optimized without hiding
where time is spent.

## Current Status

Implemented today:

- exact mesh topology path;
- exact mesh, bounds, facts, provenance, validation, audit, face-pair, coplanar,
  construction, split-plan, support, surface, winding, convex-solid, exact arrangement,
  cell-complex simplification, consumer-readiness, handoff-package, preflight, and
  exact-boolean APIs;
- tests, proptests, fuzz targets, examples, and exact-validation benchmarks.

Known limits: unsupported boolean/intersection/simplification topology is reported as a
diagnostic instead of falling back to tolerance-based geometry.

## Installation

```toml
[dependencies]
hypermesh = "0.3.0"
```

## Usage

The exact-facing path is always available and is the preferred boundary for new code:

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

Use exact validation, audit, face-pair classification, split-plan, preflight,
consumer-readiness, and handoff-package reports to audit topology before relying on
boolean output.

Named booleans first try the exact arrangement/cell-complex path before falling back to
older certified shortcuts:

```rust,ignore
use hypermesh::exact::{
    ExactArrangement, ExactBooleanOperation, ExactRegularizationPolicy,
    ValidationPolicy, boolean_exact,
};

let arrangement = ExactArrangement::from_meshes(&left, &right)?;
let mesh = arrangement
    .label_regions(ExactRegularizationPolicy::REGULARIZED_SOLID)?
    .select(ExactBooleanOperation::Union)?
    .simplify_exact()?
    .triangulate()?;

let result = boolean_exact(
    &left,
    &right,
    ExactBooleanOperation::Union,
    ValidationPolicy::ALLOW_BOUNDARY,
)?;
```

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
- Held, Martin. "FIST: Fast Industrial-Strength Triangulation of Polygons."
  *Algorithmica* 30 (2001).
- de Berg, Mark, Otfried Cheong, Marc van Kreveld, and Mark Overmars. *Computational
  Geometry: Algorithms and Applications*. Springer.
- Preparata, Franco P., and Michael Ian Shamos. *Computational Geometry: An
  Introduction*. Springer, 1985.
- Andrew, A. M. "Another Efficient Algorithm for Convex Hulls in Two Dimensions."
  *Information Processing Letters* 9.5 (1979).
- Hormann, Kai, and Alexander Agathos. "The Point in Polygon Problem for Arbitrary
  Polygons." *Computational Geometry* 20.3 (2001).
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
cargo test --no-default-features
cargo check --features internal-fuzzing
cargo fuzz run exact_arrangement
cargo bench --bench exact_validation
```
