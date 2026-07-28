# hyperbrep

`hyperbrep` is the retained boundary-representation layer of the Hyper geometry
stack. It stores vertices, oriented edge uses, loops, faces, analytic surfaces,
spatial curves, construction provenance, and explicit validation or handoff
reports.

The crate does not silently sew gaps, merge nearby vertices, infer missing
topology, or promote a derived mesh into source BREP truth. A consumer receives
the evidence that was replayed, the readiness decision, and typed blockers when
that decision could not be certified.

## Quick start

During workspace development, use the sibling checkout:

```toml
[dependencies]
hyperbrep = { path = "../hyperbrep" }
hyperlimit = { path = "../hyperlimit" }
hyperreal = { path = "../hyperreal" }
```

This example builds one exact triangular planar face and validates its retained
boundary. It is intentionally an open surface, not a closed solid:

```rust,ignore
use hyperbrep::{
    BrepCoedge, BrepEdge, BrepEdgeId, BrepEdgeOrientation, BrepFace,
    BrepFaceId, BrepLoop, BrepLoopId, BrepShell, BrepSurface,
    BrepSurfaceId, BrepSurfaceSource, BrepVertex, BrepVertexId,
};
use hyperlimit::{Plane3, Point3};
use hyperreal::Real;

fn point(x: i32, y: i32, z: i32) -> Point3 {
    Point3::new(Real::from(x), Real::from(y), Real::from(z))
}

let shell = BrepShell {
    vertices: vec![
        BrepVertex::new(BrepVertexId(0), point(0, 0, 0)),
        BrepVertex::new(BrepVertexId(1), point(1, 0, 0)),
        BrepVertex::new(BrepVertexId(2), point(0, 1, 0)),
    ],
    edges: vec![
        BrepEdge::new(BrepEdgeId(0), BrepVertexId(0), BrepVertexId(1)),
        BrepEdge::new(BrepEdgeId(1), BrepVertexId(1), BrepVertexId(2)),
        BrepEdge::new(BrepEdgeId(2), BrepVertexId(2), BrepVertexId(0)),
    ],
    surfaces: vec![BrepSurface::plane(
        BrepSurfaceId(0),
        Plane3::new(point(0, 0, 1), Real::from(0)),
        BrepSurfaceSource::ExactConstruction,
    )],
    faces: vec![BrepFace::new(
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
    )],
};

let topology = shell.validate_topology();
assert!(topology.topology_ready);

let face = shell.face_validation_report(BrepFaceId(0), None);
assert!(face.exact_face_ready);

let closure = shell.audit_closure();
assert!(!closure.closed);
```

## Core model

- `BrepVertex`, `BrepEdge`, `BrepCoedge`, `BrepLoop`, `BrepFace`, and
  `BrepShell` are the retained topological records. Stable typed identifiers
  connect them without implicit geometric matching.
- `BrepSurface` retains source provenance and analytic geometry. Exact planes
  are the supported surface family; their graph frames support arbitrary
  certifiably nonzero normals. Unsupported or lossy imports remain named.
- `BrepSurfaceDifferentialReport` interrogates an exact plane at UV, retaining
  its model point, parameter tangents, oriented unit normal, first and second
  fundamental forms, orientation alignment, and exact zero curvature.
- staged curve/surface and surface/surface reports exactly classify finite
  line/plane and plane/plane relations, retain constructed point or Pluecker
  line evidence, and provide exact stationary/minimum-distance witnesses for
  separated line segments and parallel planes.
- `BrepCurve3` owns exact model-space line, rational Bezier, and finite-domain
  NURBS curves. Rational evaluation stays homogeneous until final projection.
- `BrepPcurve`, `BrepPlanarTrimLoop`, and `BrepPlanarFaceRegion` bind exact
  `hypercurve` UV geometry to a retained surface identity.
- `BrepConstructionManifest` records feature identity, source versions,
  selected references, a topology snapshot, and a deterministic freshness
  fingerprint.

Most operations return report types rather than a bare boolean. The principal
entry points on `BrepShell` are:

- `validate_topology`, `audit_closure`, `trim_set_report`,
  `face_validation_report`, and `shell_validation_report`;
- `face_bounds_report`, `shell_bounds_report`, `face_area_report`, and
  `shell_volume_report`;
- `face_query_evidence`, `face_aabb_preflight`, `face_plane_preflight`,
  `segment_face_plane_preflight`, and `point_face_plane_preflight`;
- `solid_readiness_report`, `exact_surface_handoff`, and
  `exact_solid_handoff`;
- `exact_planar_tessellation_report`, `exact_triangle_mesh_handoff_report`,
  physics handoffs, voxel handoffs, and `handoff_package_report`.

Inspect a report's `blockers` and component reports when a readiness field is
false; the blocker is part of the API contract, not ancillary logging.

## Construction and derived output

`planar_region_shell` immediately constructs a retained planar face from a
line-only exact `hypercurve::CurveRegion2` and a derived surface frame.
`vertical_prism_shell` immediately constructs a closed vertical prism with
analytic side planes. Both return the finished shell after their validation
gates pass, or a typed error containing every observed blocker.

Tessellation projects exact-ready planar loops into UV, calls `hypertri`, and
lifts the result back to exact 3D points. Triangle, physics, voxel, mesh, and
export reports remain derived handoffs; none replace the retained BREP.

Primitive-float imports are audited by `BrepLossyFloatImportReport`. A finite
IEEE-754 coordinate can be lifted exactly as a dyadic rational, but that does
not recover the source system's topology, surface semantics, or tolerance
policy.

## Precision and performance

Coordinates remain `hyperreal::Real` values through `hyperlimit` points,
planes, AABBs, and predicates. Unknown orderings or signs become blockers
instead of epsilon comparisons. Broad-phase reports may reject work only after
certifying disjointness; overlap or contact remains a narrow-phase candidate.

Surface and face-query evidence, retained bounds, planar regions, and cached
homogeneous control nets amortize repeated work. The curve and pcurve cache
carriers use `Rc` and `OnceCell`, so they are currently intended for
thread-local ownership, not shared cross-thread mutation.

The measured reference audit, retained evidence-reuse changes, and benchmark
protocol are recorded in [PERFORMANCE.md](PERFORMANCE.md).

Current limitations are explicit: nonplanar analytic surfaces and their frames,
nonlinear curve/surface root isolation,
geometric equality across differently partitioned pcurves, curved trim-edge
reconstruction, BREP booleans and sewing, periodic NURBS, and full STEP/IGES
exchange are not yet implemented as exact report-bearing operations.

## Development

```sh
cargo fmt --all --check
cargo test --all-targets
cargo clippy --all-targets -- -D warnings
cargo bench --bench shell_audit
```

Fuzz targets for planar pcurves and voxel handoffs live under `fuzz/` and run
with `cargo fuzz`.

## References

- Chee K. Yap, [“Towards Exact Geometric Computation”](https://doi.org/10.1016/0925-7721(95)00040-2), *Computational Geometry* 7(1–2), 1997.
- Martti Mäntylä, *An Introduction to Solid Modeling*, Computer Science Press, 1988.
- Les Piegl and Wayne Tiller, [*The NURBS Book*](https://doi.org/10.1007/978-3-642-97385-7), 2nd ed., Springer, 1997.
- Gerald Farin, [*Curves and Surfaces for CAGD*](https://www.sciencedirect.com/book/9781558607378/curves-and-surfaces-for-cagd), 5th ed., Morgan Kaufmann, 2002.
- G. H. Meisters, [“Polygons Have Ears”](https://digitalcommons.unl.edu/mathfacpub/54/), *American Mathematical Monthly* 82(6), 1975.
- Brian Mirtich, [“Fast and Accurate Computation of Polyhedral Mass Properties”](https://doi.org/10.1080/10867651.1996.10487458), *Journal of Graphics Tools* 1(2), 1996.
- Aristides Requicha and Herbert Voelcker, [“Solid Modeling: A Historical Summary and Contemporary Assessment”](https://doi.org/10.1109/MCG.1982.1674149), *IEEE Computer Graphics and Applications* 2(2), 1982.
- Kevin Weiler, *Topological Structures for Geometric Modeling*, PhD dissertation, Rensselaer Polytechnic Institute, 1986.

Direct dependencies: [hyperreal](https://github.com/timschmidt/hyperreal) ·
[hyperlimit](https://github.com/timschmidt/hyperlimit) ·
[hypercurve](https://github.com/timschmidt/hypercurve) ·
[hypertri](https://github.com/timschmidt/hypertri) ·
[hyperphysics](https://github.com/timschmidt/hyperphysics) ·
[hypervoxel](https://github.com/timschmidt/hypervoxel). Related consumers:
[hypermesh](https://github.com/timschmidt/hypermesh) ·
[csgrs](https://github.com/timschmidt/csgrs).

## License

Apache-2.0.
