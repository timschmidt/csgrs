# hyperbrep

`hyperbrep` provides exact-aware boundary-representation evidence carriers for
the Hyper ecosystem. It stores retained BREP topology, analytic surface evidence,
validation reports, import/export adapter reports, construction provenance, and
derived-mesh handoff certificates without silently healing topology or hiding
tolerance decisions.

The crate is deliberately not a private CAD kernel. It is the report-bearing
boundary between exact geometry crates and downstream consumers that need to know
whether topology, surfaces, tessellation, and provenance are exact-ready.

## Hyper Ecosystem

`hyperbrep` is the retained topology layer for exact mechanical and manufacturing
evidence.

- [hyperreal](https://github.com/timschmidt/hyperreal): exact scalar arithmetic,
  source-grid values, rational facts, and dyadic lifts used by BREP reports.
- [hyperlattice](https://github.com/timschmidt/hyperlattice): exact point, vector,
  transform, and projective carriers used by model-space BREP geometry.
- [hyperlimit](https://github.com/timschmidt/hyperlimit): exact predicates,
  point-plane classification, AABB relations, and predicate policy.
- [hypersolve](https://github.com/timschmidt/hypersolve): exact residual replay for
  future solver-backed BREP construction and fitting checks.
- [hypercurve](https://github.com/timschmidt/hypercurve): exact 2D curve and region
  geometry consumed by pcurves and planar trim domains; spatial curves remain owned
  by `hyperbrep`.
- [hypertri](https://github.com/timschmidt/hypertri): constrained triangulation and
  simplex handoff target for exact planar face tessellation.
- [hyperpath](https://github.com/timschmidt/hyperpath): exact routing, offsetting,
  board-outline, and toolpath carriers that can consume BREP-derived boundaries.
- [hypermesh](https://github.com/timschmidt/hypermesh): exact mesh validation and
  derived triangle handoff consumer.
- [hypervoxel](https://github.com/timschmidt/hypervoxel): sparse voxel and exact
  half-space evidence consumer for BREP-derived solids.
- [hyperparts](https://github.com/timschmidt/hyperparts): part, package, compliance,
  and procurement evidence that can attach BREP geometry provenance.
- [hyperdrc](https://github.com/timschmidt/hyperdrc): PCB/package readiness review
  that can carry BREP mechanical handoff evidence.
- [hypercircuit](https://github.com/timschmidt/hypercircuit): circuit certification
  surfaces for future electro-mechanical handoff checks.
- [hyperphysics](https://github.com/timschmidt/hyperphysics): material, mass,
  thermal, electromagnetic, and contact evidence for BREP bodies.
- [hyperpack](https://github.com/timschmidt/hyperpack): exact packing and placement
  verification that can consume BREP bounds and mesh handoffs.
- [hyperevolution](https://github.com/timschmidt/hyperevolution): optimization and
  replay loops for generated geometry and placement candidates.
- [hypersdf](https://github.com/timschmidt/hypersdf): exact signed-distance evidence
  and implicit-shape handoffs complementary to retained BREP topology.

## Current Status

`hyperbrep` is version `0.2.0` and is currently an exact-evidence carrier, not a
general-purpose modeler. Implemented today:

- retained topology records for vertices, edges, oriented coedges, loops, faces,
  surfaces, and shells;
- top-level exact model-space curves with stable source/version provenance for
  line segments, arbitrary-degree rational Beziers, and finite-domain NURBS;
  homogeneous de Casteljau/de Boor evaluation never round-trips through planar or
  primitive-float coordinates, and clones share weighted control nets;
- retained planar pcurves over Hypercurve's top-level paths, including rational
  Bezier and NURBS carriers, with exact structural image equality, clone-shared
  reversal/native-segment facts, face-region trim domains, point classification,
  edge-use reports, and typed failures for malformed trim topology;
- exact planar surface support backed by `hyperlimit::Plane3`;
- explicit unsupported and lossy surface-family records for adapter boundaries;
- shell closure, topology validation, surface inventory, face bounds, trim-loop,
  geometry, face validation, and solid-readiness reports;
- exact face/shell AABB reports and broad-phase AABB/plane/segment/point preflight
  queries;
- construction provenance manifests with source versions, selected references,
  replay status, and stale topology-snapshot detection;
- lossy primitive-float import audits that count finite coordinates, exact dyadic
  lifts, surface-family support, topology evidence, and tolerance declarations;
- tessellation and mesh handoff reports that keep generated triangles derived from,
  not replacements for, retained BREP topology;
- export manifests for OBJ, glTF, polyline, STEP-style, and external BREP routes.

Non-planar analytic surfaces, geometric equality across differently partitioned
pcurves, non-native curved face-trim edge-use, boolean topology edits, sewing, exact
spatial-curve derivatives/intersections/editing and periodic NURBS, exact
volume/orientation proof, and full STEP/IGES import are intentionally not hidden behind
tolerance heuristics yet. They must arrive as new exact reports or explicit adapter
evidence.

## Main Types

- `BrepVertex`, `BrepEdge`, `BrepCoedge`, `BrepLoop`, `BrepFace`, and `BrepShell`
  are the retained topology carriers. Edges have stable ids and canonical
  directions; coedges record oriented uses; loops bound faces; shells aggregate
  vertices, edges, surfaces, and faces.
- `BrepCurve3` is the top-level spatial curve carrier. `BrepLineSegment3`,
  `BrepRationalBezier3`, and `BrepNurbsCurve3` own exact 3D geometry;
  `BrepCurveSource3` and contextual `BrepCurveError3` preserve source version,
  operation, family, and failure reason.
- `BrepSurface`, `BrepSurfaceId`, `BrepSurfaceKind`, and `BrepSurfaceSource` retain
  support-surface evidence. `Plane` is the current exact-core family; unsupported
  and lossy imports stay named instead of being promoted.
- `BrepSurfaceFacts`, `PreparedBrepSurface`, and `BrepSurfacePointReport` cache
  exact plane facts and replay point/surface predicates.
- `BrepPcurve`, `BrepPlanarTrimLoop`, and `BrepPlanarFaceRegion` bind exact
  `hypercurve` UV geometry to `BrepSurfaceId`. `BrepPcurve` owns a top-level
  `CurvePath2`, so every planar curve family is retained without demotion. Their image,
  point, and edge-use reports preserve support-surface identity before classifying trim
  topology; `PreparedBrepPlanarFaceRegion` reuses prepared planar-region facts.
- `BrepPlanarError` distinguishes missing material, support mismatch, duplicate,
  self-contacting or intersecting trims, unowned holes, unresolved predicates, and
  contradictory report evidence.
- `BrepTopologyValidationReport`, `BrepShellClosureReport`, and
  `BrepSurfaceInventoryReport` summarize identity, incidence, closure, manifold
  edge use, and supported-surface readiness.
- `BrepFaceBoundsReport`, `BrepShellBoundsReport`,
  `PreparedBrepFaceBounds`, `PreparedBrepShellBounds`, and
  `BrepFaceAabbPreflightReport` provide exact AABB evidence for broad-phase
  scheduling.
- `BrepTrimLoopReport` and `BrepFaceTrimSetReport` validate topology-only trim
  evidence: loop cardinality, edge references, degenerate edges, vertex-chain
  closure, and surface replay readiness.
- `BrepGeometryValidationReport` and `BrepFaceValidationReport` aggregate surface,
  trim, bounds, vertex-on-surface, and optional tessellation evidence for one face.
- `BrepFacePlanePreflightReport`, `BrepSegmentFacePlaneReport`, and
  `BrepPointFacePlaneReport` expose exact rejection/candidate queries without
  claiming hidden boolean logic.
- `BrepConstructionManifest`, `BrepConstructionProvenanceReport`,
  `BrepFeatureId`, `BrepSourceVersion`, `BrepSelectedReference`, and
  `BrepTopologySnapshot` preserve source freshness and replay status.
- `BrepFaceTessellationManifest`, `BrepShellTessellationReport`, and
  `BrepMeshHandoffReport` certify derived mesh output for `hypermesh`, display,
  voxelization, and physics handoffs.
- `BrepLossyFloatImportReport`, `BrepImportedSurfaceFamily`, and
  `BrepLossyImportBlocker` make primitive-float import risk explicit.
- `BrepExportManifest`, `BrepExportReport`, `BrepExportFormat`, and
  `BrepExportScalarPolicy` keep export artifacts separate from authoritative
  retained topology.

## Precision

`hyperbrep` follows Yap's exact-geometric-computation boundary: topology-changing
decisions require exact or certified predicates, while uncertain adapter evidence stays
explicit. Exact coordinates are held as `hyperreal::Real` through `hyperlimit::Point3`
and `Plane3`. Plane preparation, point-plane classification, AABB relations, and
segment-plane classification are delegated to `hyperlimit` instead of local epsilon
tests.

Spatial rational curves remain homogeneous through evaluation. Affine `Point3`
coordinates are formed only once at the requested parameter, so projective poles are
typed failures rather than silently sampled or demoted values.

The crate improves precision by separating representation from certification. A finite
`f64` import can be lifted exactly as a dyadic rational, but the import remains a lossy
adapter report until topology evidence, tolerance metadata, and surface-family support
are replayed. Unsupported surfaces, zero normals, non-finite coordinates, unknown
relations, and stale construction snapshots are blockers, not silent repairs.

## Performance

Reports are designed as reusable scheduling evidence. Surface facts, prepared planes,
face bounds, shell bounds, trim summaries, and tessellation manifests let downstream
systems reject impossible work before expensive narrow-phase checks. A certified
disjoint AABB can skip face/face work; an intersecting AABB remains only a candidate
signal until exact trim and surface predicates replay.

The crate avoids doing all work eagerly. Most APIs produce per-face or per-route
reports, and shell-level reports aggregate those facts only when requested. Criterion
coverage in `benches/shell_audit.rs` tracks closure, pcurve/face-region queries, trim,
bounds, preflight, validation, tessellation, import, export, and prepared-surface paths.

## Numerical Explosion

`hyperbrep` combats numerical explosion by keeping derived artifacts derived. Meshes,
exports, preview adapters, and lossy imports do not replace retained topology. Exact
plane and AABB predicates are introduced at report boundaries, and broad-phase filters
are allowed to reject work only when the exact relation is decided.

Unsupported analytic families remain compact `Unsupported` records until their frames,
parameter domains, and intersection predicates exist. This avoids expanding every
surface or imported mesh into speculative exact algebra. Construction manifests carry
source versions and topology snapshots so old exact reports are rejected when topology
changes, rather than recomputed or trusted blindly.

## Usage

Construct a small exact planar shell and replay topology, trim, bounds, surface, and
solid-readiness reports:

```rust,ignore
use hyperbrep::{
    BrepCoedge, BrepEdge, BrepEdgeId, BrepEdgeOrientation, BrepFace, BrepFaceId,
    BrepLoop, BrepLoopId, BrepShell, BrepSurface, BrepSurfaceId, BrepSurfaceSource,
    BrepVertex, BrepVertexId,
};
use hyperlimit::{Plane3, Point3};
use hyperreal::Real;

fn r(value: i32) -> Real {
    Real::from(value)
}

fn p(x: i32, y: i32, z: i32) -> Point3 {
    Point3::new(r(x), r(y), r(z))
}

let shell = BrepShell {
    vertices: vec![
        BrepVertex::new(BrepVertexId(0), p(0, 0, 0)),
        BrepVertex::new(BrepVertexId(1), p(1, 0, 0)),
        BrepVertex::new(BrepVertexId(2), p(0, 1, 0)),
    ],
    edges: vec![
        BrepEdge::new(BrepEdgeId(0), BrepVertexId(0), BrepVertexId(1)),
        BrepEdge::new(BrepEdgeId(1), BrepVertexId(1), BrepVertexId(2)),
        BrepEdge::new(BrepEdgeId(2), BrepVertexId(2), BrepVertexId(0)),
    ],
    surfaces: vec![BrepSurface::plane(
        BrepSurfaceId(0),
        Plane3::new(p(0, 0, 1), r(0)),
        BrepSurfaceSource::ExactConstruction,
    )],
    faces: vec![
        BrepFace::new(
            BrepFaceId(0),
            BrepSurfaceId(0),
            BrepLoop::new(
                BrepLoopId(0),
                vec![
                    BrepCoedge::new(BrepEdgeId(0), BrepEdgeOrientation::Forward),
                    BrepCoedge::new(BrepEdgeId(1), BrepEdgeOrientation::Forward),
                    BrepCoedge::new(BrepEdgeId(2), BrepEdgeOrientation::Forward),
                ],
            ),
        ),
        BrepFace::new(
            BrepFaceId(1),
            BrepSurfaceId(0),
            BrepLoop::new(
                BrepLoopId(1),
                vec![
                    BrepCoedge::new(BrepEdgeId(2), BrepEdgeOrientation::Reversed),
                    BrepCoedge::new(BrepEdgeId(1), BrepEdgeOrientation::Reversed),
                    BrepCoedge::new(BrepEdgeId(0), BrepEdgeOrientation::Reversed),
                ],
            ),
        ),
    ],
};

let closure = shell.audit_closure();
assert!(closure.closed);
assert!(closure.exact_shell_ready);

let topology = shell.validate_topology();
assert!(topology.topology_ready);
assert_eq!(topology.euler_characteristic, 2);

let trim = shell.trim_set_report(BrepFaceId(0));
assert!(trim.trim_set_ready);

let bounds = shell.face_bounds_report(BrepFaceId(0));
assert!(bounds.exact_bounds_ready);

let prepared = shell.surfaces[0].prepare();
assert!(prepared.exact_replay_ready());
assert!(prepared.classify_point(&p(0, 0, 0)).on_surface);

let point = shell.point_face_plane_preflight(BrepFaceId(0), &p(0, 0, 0));
assert!(point.on_support_plane);
assert!(point.requires_trim_replay);

let face = shell.face_validation_report(BrepFaceId(0), None);
assert!(face.exact_face_ready);

let solid = shell.solid_readiness_report(None);
assert!(solid.exact_solid_boundary_ready);
```

Add construction provenance and derived mesh handoff evidence:

```rust,ignore
use hyperbrep::{
    BrepConstructionKind, BrepConstructionManifest, BrepFaceId,
    BrepFaceTessellationManifest, BrepFeatureId, BrepMeshHandoffReport, BrepSourceVersion,
};

let construction = BrepConstructionManifest::exact(
    BrepFeatureId::new("feature:tri-shell").unwrap(),
    BrepConstructionKind::PlanarFace,
    vec![BrepSourceVersion::new("source:example", 1).unwrap()],
    &shell,
);
assert!(construction.report(&shell).construction_fresh);

let manifests = vec![
    BrepFaceTessellationManifest::exact_planar(BrepFaceId(0), 1, 3, 3, 0),
    BrepFaceTessellationManifest::exact_planar(BrepFaceId(1), 1, 3, 3, 0),
];
let mesh = BrepMeshHandoffReport::from_shell_manifests_with_construction(
    &shell,
    &manifests,
    Some(&construction),
);
assert!(mesh.exact_surface_handoff_ready);
assert!(mesh.exact_solid_handoff_ready);
assert!(mesh.derived_mesh_only);
```

Audit lossy imports and export routes without promoting either to authoritative BREP
topology:

```rust,ignore
use hyperbrep::{
    BrepExportFormat, BrepExportManifest, BrepExportScalarPolicy, BrepImportedSurfaceFamily,
    BrepLossyFloatImportReport,
};

let import = BrepLossyFloatImportReport::inspect_f64(
    "viewer-adapter",
    &[0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
    &[BrepImportedSurfaceFamily::Plane],
    true,
    true,
);
assert!(import.adapter_replay_ready);
assert!(import.lossy_adapter_only);

let export = BrepExportManifest {
    format: BrepExportFormat::Obj,
    scalar_policy: BrepExportScalarPolicy::F64,
    source_object_ids: vec!["shell:tri".into()],
    exported_primitives: mesh.triangle_count,
    exported_coordinates: mesh.lifted_vertex_count * 3,
    finite_exported_coordinates: mesh.lifted_vertex_count * 3,
    labels_preserved: true,
    exact_replay_declared: true,
}
.report(Some(&mesh));

assert!(export.export_ready);
assert!(export.export_adapter_only);
```

## Development

Useful local checks:

```sh
cargo test
cargo bench --bench shell_audit
```

## References

- Farin, Gerald. *Curves and Surfaces for CAGD: A Practical Guide*. 5th ed., Academic Press, 2002.
- Mantyla, Martti. *An Introduction to Solid Modeling*. Computer Science Press, 1988.
- Piegl, Les, and Wayne Tiller. *The NURBS Book*. 2nd ed., Springer, 1997.
- Requicha, Aristides A. G., and Herbert B. Voelcker. "Solid Modeling: A Historical Summary and Contemporary Assessment." *IEEE Computer Graphics and Applications*, vol. 2, no. 2, 1982, pp. 9-24, https://doi.org/10.1109/MCG.1982.1674149.
- Weiler, Kevin. "Topological Structures for Geometric Modeling." PhD dissertation, Rensselaer Polytechnic Institute, 1986.
- Yap, Chee K. "Towards Exact Geometric Computation." *Computational Geometry*, vol. 7, nos. 1-2, 1997, pp. 3-23, https://doi.org/10.1016/0925-7721(95)00040-2.
