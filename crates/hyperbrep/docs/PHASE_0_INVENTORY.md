# Phase 0 Replacement Inventory

Status: Baseline recorded
Date: 2026-07-29
Plan: [`../../HYPERBREP_REBUILD_PLAN.md`](../../HYPERBREP_REBUILD_PLAN.md)

This inventory defines what the clean-break rebuild must replace or delete. It
is deliberately not a migration map: no legacy public item survives solely to
support an old caller.

## Repository Baseline

- Package: `hyperbrep 0.2.0`
- Baseline commit: `1d5a644649ca` (`Prepare hyperbrep dependency release`)
- Branch: `main`
- Worktree before Phase 0 changes: clean
- Public declarations found under `src`: 124
- External workspace Cargo dependencies on `hyperbrep`: none
- External workspace Rust imports of `hyperbrep` or `Brep*`: none
- Internal examples, tests, benches, and fuzz targets are the complete caller
  set.
- Baseline `cargo test --all-targets`: passed on 2026-07-29
  (53 unit tests, 38 integration tests, two README tests, all benchmark smoke
  runs, and the example target).

The caller scan searched every workspace `Cargo.toml` for `hyperbrep` and every
workspace Rust source for `hyperbrep::` or a `Brep*` identifier. Only files
inside the `hyperbrep` repository matched.

## Structural Defects Requiring Replacement

### Topology

The current topology records are public mutable vectors and structs:

- `BrepShell` exposes `vertices`, `edges`, `surfaces`, and `faces`.
- `BrepVertex`, `BrepEdge`, `BrepCoedge`, `BrepLoop`, and `BrepFace` expose
  fields that can be changed without rebuilding indexes or revalidation.
- `BrepEdge` stores only endpoint IDs. It does not own a `BrepCurve3` or its
  exact parameter domain.
- `BrepCoedge` stores only an edge ID and orientation. It does not own its
  face-local pcurve or curve/pcurve parameter correspondence.
- Loops are nested values inside faces instead of arena records with their own
  stable identity and adjacency.
- There is no canonical `Solid` topology record.
- Most queries reconstruct identifier maps from vectors.

These records will be deleted and replaced by private arena records in an
immutable `Model`.

### Geometry

- `BrepSurface` stores a plane equation, not a complete parameterized plane
  with model-space origin and `u`/`v` basis.
- Unsupported surfaces can be retained in the normal model instead of being
  confined to unvalidated import state.
- `BrepCurve3`, rational Bézier curves, and NURBS curves use `Rc` and
  `std::cell::OnceCell`, preventing the canonical geometry model from being
  shared safely across threads.
- Geometry `PartialEq` implementations compare representations structurally and
  are used in paths whose names imply image or geometric equality.
- Parameter validation and measurement code call `Real::partial_cmp` directly
  instead of consuming the common Hyperlimit predicate outcome.
- Point deduplication, loop closure, bounds degeneracy, and curve-image checks
  contain direct structural point or coordinate equality.

The useful algorithms may be ported, but the current carriers and semantic
surface are not retained.

### Reports

The current API exposes many readiness reports because invalid data is a normal
public state. The final validated model will guarantee core invariants by type.
Diagnostic evidence remains available for raw imports, failed construction,
failed edits, unresolved predicates, and unsupported operations, but successful
ordinary calls will return conventional values.

### Dependencies

Current unconditional dependencies:

- `hyperlimit`
- `hypercurve`
- `hyperphysics`
- `hyperreal`
- `hypertri` with `earcut`
- `hypervoxel`

Target core dependencies:

- `hyperreal`
- `hyperlattice`
- `hyperlimit`
- `hypersolve`
- `hypercurve`

`hypertri`, `hyperphysics`, and `hypervoxel` are removed from the core graph.
They may return as optional feature integrations or downstream adapter crates
after the source model is complete.

## Public Deletion Matrix

### Replace with canonical geometry

| Delete | Replacement |
| --- | --- |
| `BrepCurve3` | `Curve3` |
| `BrepCurveGeometry3` | private representation behind `Curve3` |
| `BrepCurveFamily3` | `Curve3Kind` if public dispatch is needed |
| `BrepCurveParameterDomain3` | `ParameterDomain` |
| `BrepLineSegment3` | `Line3`/`LineSegment3` under `Curve3` |
| `BrepRationalBezier3` | `RationalBezier3` under `Curve3` |
| `BrepNurbsCurve3` | `NurbsCurve3` under `Curve3` |
| `BrepPcurve` | `Pcurve` |
| `BrepSurface` | `Surface` |
| `BrepSurfaceKind` | private representation behind `Surface` |
| implicit-only plane storage | parameterized `Plane` surface |

All `BrepCurve*Error*` and `BrepPlanar*Error*` types are replaced by errors
organized around the final operations rather than the old carrier names.

### Replace with canonical topology

| Delete | Replacement |
| --- | --- |
| `BrepVertexId` | `VertexId` |
| `BrepEdgeId` | `EdgeId` |
| no edge-use ID | `EdgeUseId` |
| `BrepLoopId` | `WireId` |
| `BrepFaceId` | `FaceId` |
| `BrepSurfaceId` | `SurfaceId` |
| no curve/pcurve IDs | `Curve3Id` and `PcurveId` |
| no shell/solid IDs | `ShellId` and `SolidId` |
| `BrepVertex` | private `Vertex` record exposed by accessor |
| `BrepEdge` | private `Edge` record with curve and domain |
| `BrepCoedge` | private `EdgeUse` record with pcurve |
| `BrepLoop` | private `Wire` record |
| `BrepFace` | private `Face` record |
| `BrepShell` | private `Shell` record |
| no solid record | private `Solid` record |
| public record vectors | immutable indexed `Model` |

`BrepEdgeOrientation` becomes the conventional `Direction` or `Orientation`
enum used by `EdgeUse`; the final name is fixed with the arena API.

### Delete as core integrations

- Every `BrepPhysics*` item and `physics.rs`
- Every `BrepVoxel*` item and `voxel.rs`
- Every `BrepTriangle*` item and `triangle.rs`
- `BrepExact*Handoff*` reports that merely compensate for a possibly invalid
  source carrier

Tessellation, physics, and voxel conversions will consume a validated model
through optional integrations. Their results remain derived data.

### Replace report families by validation or operation evidence

The following prefixes are removed:

- `BrepTopologyValidation*`
- `BrepShellClosure*`
- `BrepEdgeAgreement*`
- `BrepFaceValidation*`
- `BrepShellValidation*`
- `BrepTrimLoop*`
- `BrepFaceTrimSet*`
- `BrepFaceBounds*`
- `BrepShellBounds*`
- `BrepFaceArea*`
- `BrepShellVolume*`
- `BrepSolidReadiness*`
- `BrepSurfaceFrame*`
- `BrepSurfaceIntersection*`
- `BrepCurveSurface*`
- `BrepFacePlanePreflight*`
- `BrepSegmentFacePlane*`
- `BrepPointFacePlane*`
- `BrepFaceQueryBatch*`

Useful evidence fields move into:

- `ValidationReport` for `RawModel` or builder commit failure;
- operation-specific errors for unsupported or unresolved work;
- certified intersection result types;
- optional tracing/diagnostic objects when callers request provenance.

## File Disposition

| Current file | Disposition |
| --- | --- |
| `topology.rs` | Replace with typed arenas, private records, builder, raw model, and edit transaction |
| `curve.rs` | Port exact algorithms into final `Curve3` API; replace carriers and caches |
| `pcurve.rs` | Port useful planar bindings into per-edge-use `Pcurve` ownership |
| `surface.rs` | Replace with final parameterized `Surface` API |
| `construction.rs` | Replace with conventional `builder` module |
| `adjacency.rs` | Fold retained O(1) adjacency into `Model` |
| `report.rs` | Replace by validation diagnostics and model-guaranteed invariants |
| `validation.rs` | Rebuild around `RawModel`/`ModelBuilder::finish`/`Edit::commit` |
| `trim.rs` | Fold certified wire and face invariants into construction |
| `frame.rs` | Port into parameterized surface evaluation |
| `interrogation.rs` | Port into the conventional surface API |
| `bounds.rs` | Port certified bounds into retained model indexes/caches |
| `area.rs` | Port as successful measurement or typed operation error |
| `volume.rs` | Port as successful measurement or typed operation error |
| `surface_intersection.rs` | Replace by retained certified intersection objects |
| `query.rs` | Replace reports with conventional certified queries |
| `solid.rs` | Replace readiness report with canonical `Solid` |
| `handoff.rs` | Delete |
| `triangle.rs` | Remove from core; recreate as optional integration |
| `physics.rs` | Remove from core; recreate as optional integration |
| `voxel.rs` | Remove from core; recreate as optional integration |

## Equality and Ordering Audit

Authoritative direct structural comparisons currently appear in:

- construction: point chaining and vertex deduplication;
- bounds/frame: zero extents;
- trim/area/volume/triangle: loop closure;
- curve/pcurve: parameter order and geometric/image equality;
- HyperTRI triangle classification: direct query-point/vertex equality.

The audit policy is:

1. ID and finite enum equality remains ordinary `Eq`.
2. Representation comparison, if needed, receives an explicit
   `same_representation` name.
3. Mathematical scalar order uses `hyperlimit::compare_reals`.
4. Point equality uses `hyperlimit::point2_equal` or `point3_equal`.
5. Higher geometric equality uses a geometry-specific certified predicate.
6. `PredicateOutcome::Unknown` becomes a typed unresolved error or is retained;
   it is never mapped to `false`, equality, or zero extent.

## Internal Caller Inventory

The clean cutover must rewrite or delete:

- `src/lib.rs` unit fixtures and tests;
- `tests/antagonistic.rs`;
- `tests/pcurve.rs`;
- `tests/spatial_curve.rs`;
- `tests/readme.rs`;
- `examples/basic.rs`;
- `benches/shell_audit.rs`;
- `fuzz/fuzz_targets/hyperreal_representations.rs`;
- `fuzz/fuzz_targets/planar_pcurve.rs`;
- `fuzz/fuzz_targets/voxel_handoff.rs`;
- `README.md`;
- `PERFORMANCE.md`.

The voxel fuzz target is deleted with the core voxel integration. New fuzz
targets will exercise `RawModel` validation, builder commit, editing,
intersections, and Booleans.

## First Vertical Slice

The first implementation milestone is one final-form exact planar solid:

- private typed-ID arena;
- line `Curve3`;
- exact `ParameterDomain`;
- parameterized plane `Surface`;
- line `Pcurve` per `EdgeUse`;
- `ModelBuilder` with local checks;
- whole-model commit certification;
- exact box construction through `builder`;
- exact transform, area, volume, bounds, classification, and deterministic
  serialization;
- no old carrier or compatibility wrapper in the path.

Additional geometry families begin only after this slice uses the final
ownership, decision, and error contracts.
