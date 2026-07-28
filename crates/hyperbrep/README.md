# Hyperbrep

Exact-aware retained boundary representation for the Hyper geometry stack.

Hyperbrep stores the topology and geometry of faces and shells: vertices,
edges, oriented edge uses, loops, analytic surfaces, spatial curves, planar
trim curves, and the evidence used to decide whether a model is ready for a
particular downstream operation. It is a BREP data and validation layer, not a
CSG language, implicit-solid kernel, or triangle-mesh Boolean engine.

The crate never silently sews gaps, merges nearby vertices, infers missing
topology, or promotes a derived mesh into source BREP truth. Unsupported
geometry and undecidable predicates remain typed blockers in public reports.

This README describes crate version `0.2.0`.

## Primary types

| Type | Role |
| --- | --- |
| `BrepShell` | Retained vertices, edges, surfaces, and faces |
| `BrepVertex`, `BrepEdge`, `BrepCoedge`, `BrepLoop`, `BrepFace` | Topological records connected by stable typed IDs |
| `BrepSurface` | Analytic surface plus exact-replay evidence |
| `BrepCurve3` | Exact spatial line, rational Bézier, or finite-domain NURBS carrier |
| `BrepPcurve`, `BrepPlanarTrimLoop`, `BrepPlanarFaceRegion` | Hypercurve geometry bound to a surface’s UV domain |
| `BrepTopologyValidationReport`, `BrepShellClosureReport` | Graph validity and closed-manifold evidence |
| `BrepFaceValidationReport`, `BrepShellValidationReport` | Combined topology/geometry readiness |
| `BrepExactSurfaceHandoffReport`, `BrepExactSolidHandoffReport` | Explicit downstream handoff gates |

Most operations return a report rather than a bare Boolean. When a readiness
field is false, inspect its `blockers` and component reports; that evidence is
part of the API contract.

## Install

```toml
[dependencies]
hyperbrep = "0.2.0"
```

Hyperbrep currently has no Cargo feature flags.

## Quick start

The following checked example converts an exact Hypercurve rectangle into a
closed retained prism and inspects its solid-readiness report.

<!-- quickstart:start -->
```rust
use hyperbrep::vertical_prism_shell;
use hypercurve::{Contour2, CurvePolicy, CurveRegion2, LineSeg2, Point2, Segment2};
use hyperreal::Real;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let p = |x, y| Point2::new(Real::from(x), Real::from(y));
    let boundary = [(0, 0), (2, 0), (2, 3), (0, 3), (0, 0)]
        .windows(2)
        .map(|pair| {
            LineSeg2::try_new(p(pair[0].0, pair[0].1), p(pair[1].0, pair[1].1)).map(Segment2::Line)
        })
        .collect::<hypercurve::CurveResult<Vec<_>>>()?;
    let contour = Contour2::try_new(boundary)?;
    let region =
        CurveRegion2::try_from_native_material_contours(vec![contour], &CurvePolicy::certified())?;

    let shell = vertical_prism_shell(&region, Real::from(0), Real::from(4))?;
    let solid = shell.solid_readiness_report();
    assert!(solid.exact_solid_boundary_ready);
    assert!(solid.exact_volume_ready);
    Ok(())
}
```
<!-- quickstart:end -->

Run it with:

```sh
cargo run --example basic
```

## Model and ownership

```text
BrepVertex ── BrepEdge
                  │ referenced with orientation
              BrepCoedge ── BrepLoop ── BrepFace ── BrepSurface
                                                │
                                           BrepShell
                                                │
                    validate / measure / tessellate / hand off
```

Stable identifiers connect retained records without geometric matching.
`BrepFace` owns one outer loop and optional inner loops. `BrepSurface` owns the
surface identity and evidence; planar trim geometry is represented separately
in the surface’s parameter space.

Exact planes are the supported analytic surface family. Spatial lines,
rational Béziers, and non-periodic finite-domain NURBS are supported curve
families. An unsupported surface can still be retained with
`BrepSurface::unsupported`, but exact operations will report the corresponding
blocker.

Higher-level modeling grammar belongs above this crate. In particular, CSGRS
may convert modeled results into a BREP, but Hyperbrep does not own CSG parsing,
feature history, or mesh Boolean policy.

## API guide

### Topology and validation

- `BrepVertex::new`, `BrepEdge::new`, `BrepCoedge::new`,
  `BrepLoop::new`, `BrepFace::new`, and `BrepFace::with_inner` construct the
  retained records.
- `BrepShell::validate_topology` checks identifiers, loop connectivity,
  components, Euler summaries, boundary edges, non-manifold uses, and
  orientation agreement.
- `BrepShell::closure_report` audits whether the shell is a closed,
  consistently oriented two-manifold over supported surfaces.
- `trim_set_report`, `geometry_validation_report`,
  `face_validation_report`, and `shell_validation_report` progressively add
  trim, geometry, and shell evidence.
- `edge_agreement_report` exposes per-edge use and orientation evidence.

### Surfaces, frames, curves, and trims

- `BrepSurface::{plane, unsupported, facts, evidence,
  is_supported_exact_plane, frame_report, interrogate_uv,
  intersect_surface}` constructs and interrogates retained surfaces.
- `evaluate_frame_uv` and `project_frame_point` move between a supported
  plane’s model and UV frames. `face_uv_bounds_report` measures a face in that
  frame.
- `classify_surface_point_with_evidence` and
  `classify_plane_surface_point_with_evidence` retain point/surface
  classification evidence.
- `BrepCurve3::{new, family, geometry, parameter_domain, point_at,
  intersect_surface}` is the unified spatial-curve entry point.
  `BrepLineSegment3::new`, `BrepRationalBezier3::try_new`, and
  `BrepNurbsCurve3::try_new` construct its native families.
- `stationary_distance_to_surface` reports exact stationary/minimum-distance
  witnesses for supported separated curve/surface cases.
- `BrepPcurve::new`, `BrepPlanarTrimLoop::new`, and
  `BrepPlanarFaceRegion::try_new` bind Hypercurve paths and contours to a
  retained surface. Face regions expose UV point classification and
  edge-use/image-equality reports.

### Construction and measurement

- `planar_region_shell` builds a retained planar face from a line-only exact
  `hypercurve::CurveRegion2`.
- `vertical_prism_shell` builds a closed vertical prism with analytic side
  planes. Both constructors validate before returning and preserve every
  observed blocker on failure.
- `face_bounds_report`, `shell_bounds_report`, and `face_aabb_preflight`
  provide exact bounds and certified broad-phase rejection.
- `face_area_report` and `shell_volume_report` calculate supported exact area
  and signed volume with readiness evidence.
- `face_plane_preflight`, `face_plane_preflight_batch`,
  `segment_face_plane_preflight`, `point_face_plane_preflight`, and
  `face_query_batch_report` group certified face-query preflights.

### Downstream handoffs

- `solid_readiness_report` combines closure, topology, surface, and volume
  requirements for an exact solid.
- `exact_surface_handoff` and `exact_solid_handoff` gate exact downstream use.
- `exact_triangle_mesh_handoff_report` triangulates exact-ready planar faces
  through Hypertri and returns exact 3D triangles or blockers.
- `physics_shape_handoff_report`, `physics_fixture_handoff_report`, and
  `physics_mass_handoff_report` prepare explicit Hyperphysics adapters.
- `voxel_geometry` prepares the Hypervoxel handoff without changing retained
  BREP ownership.

Derived triangles, physics shapes, and voxel geometry never replace the source
shell.

## Guarantees and current boundaries

- Coordinates remain `hyperreal::Real` values through Hyperlimit points,
  planes, bounds, and predicates.
- Unknown ordering or sign becomes a blocker; it is not replaced by an
  undocumented epsilon comparison.
- Broad-phase reports reject work only after certifying disjointness. Contact
  or overlap remains a narrow-phase candidate.
- Primitive `f64` values can be lifted exactly as dyadic rationals, but doing
  so cannot recover the source system’s topology, analytic intent, or
  tolerance policy.
- Cached curve, pcurve, and evidence carriers use `Rc` and `OnceCell`; they are
  intended for thread-local ownership.

Not yet implemented as exact report-bearing operations are nonplanar analytic
surface frames, general nonlinear curve/surface root isolation, curved
trim-edge reconstruction, BREP Boolean and sewing operations, periodic NURBS,
and complete STEP/IGES exchange.

## Validation and performance

```sh
cargo fmt --all -- --check
cargo test --all-targets
cargo clippy --all-targets -- -D warnings
RUSTDOCFLAGS="-D warnings" cargo doc --no-deps
cargo check --benches
```

The benchmark protocol and measured reference audit are in
[PERFORMANCE.md](PERFORMANCE.md). Fuzz targets for planar pcurves and voxel
handoffs live in `fuzz/`.

## References

These works define the solid-modeling, spline, triangulation, mass-property,
and exact-computation foundations relevant to Hyperbrep:

- Requicha, A. A. G., and Voelcker, H. B. “Solid Modeling: A Historical
  Summary and Contemporary Assessment.” *IEEE Computer Graphics and
  Applications* 2(2), 1982.
  [DOI: 10.1109/MCG.1982.1674149](https://doi.org/10.1109/MCG.1982.1674149).
- Mäntylä, M. *An Introduction to Solid Modeling*. Computer Science Press,
  1988.
- Weiler, K. *Topological Structures for Geometric Modeling*. PhD
  dissertation, Rensselaer Polytechnic Institute, 1986.
- Piegl, L., and Tiller, W. *The NURBS Book*, 2nd ed. Springer, 1997.
  [DOI: 10.1007/978-3-642-97385-7](https://doi.org/10.1007/978-3-642-97385-7).
- Farin, G. *Curves and Surfaces for CAGD*, 5th ed. Morgan Kaufmann, 2002.
  [Publisher](https://www.sciencedirect.com/book/9781558607378/curves-and-surfaces-for-cagd).
- Meisters, G. H. “Polygons Have Ears.” *American Mathematical Monthly*
  82(6), 1975, 648–651.
  [DOI: 10.2307/2319703](https://doi.org/10.2307/2319703).
- Mirtich, B. “Fast and Accurate Computation of Polyhedral Mass Properties.”
  *Journal of Graphics Tools* 1(2), 1996, 31–50.
  [DOI: 10.1080/10867651.1996.10487458](https://doi.org/10.1080/10867651.1996.10487458).
- Yap, C. K. “Towards Exact Geometric Computation.” *Computational Geometry*
  7(1–2), 1997, 3–23.
  [DOI: 10.1016/0925-7721(95)00040-2](https://doi.org/10.1016/0925-7721(95)00040-2).

## Acknowledgements

Hyperbrep builds directly on
[Hyperreal](https://github.com/timschmidt/hyperreal),
[Hyperlimit](https://github.com/timschmidt/hyperlimit),
[Hypercurve](https://github.com/timschmidt/hypercurve),
[Hypertri](https://github.com/timschmidt/hypertri),
[Hyperphysics](https://github.com/timschmidt/hyperphysics), and
[Hypervoxel](https://github.com/timschmidt/hypervoxel). Hypermesh and CSGRS are
related consumers and conversion peers.

The research cited above informs the topology, exact-computation, curve,
tessellation, and mass-property models. It does not imply source-code
derivation.

## License and contributing

Licensed under Apache-2.0, as declared in [`Cargo.toml`](Cargo.toml).

When reporting a failure, include the smallest retained shell, the operation,
and the returned blockers. Before proposing a change, run formatting, the
focused regression, the complete test suite, and strict Clippy.
