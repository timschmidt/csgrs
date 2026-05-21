# hypersdf

`hypersdf` provides exact-aware signed-distance and implicit-field carriers for
the Hyper stack. It retains expression structure, exact parameters, primitive
object packages, classification reports, preview adapters, solver replay reports,
and voxel handoff envelopes without turning primitive-float samples into topology
truth.

The crate is best understood as a continuous-field evidence layer. It answers
inside/boundary/outside questions through exact or certified predicates where
available, and it keeps preview sampling, meshing, shader export, and external
solver proposals explicitly separate from certified geometry.

## Hyper Ecosystem

`hypersdf` is the implicit-field counterpart to retained BREP, mesh, and voxel
geometry in the Hyper ecosystem.

- [hyperreal](https://github.com/timschmidt/hyperreal): exact scalar arithmetic,
  rational facts, dyadic schedules, exact trigonometric shortcuts, and lossy
  preview lowering.
- [hyperlattice](https://github.com/timschmidt/hyperlattice): exact vectors,
  matrices, affine transforms, and point carriers used by linear fields,
  gradients, normals, and transforms.
- [hyperlimit](https://github.com/timschmidt/hyperlimit): point, AABB, sphere,
  plane, segment, sign, and ordering predicates used for exact classification.
- [hypersolve](https://github.com/timschmidt/hypersolve): residual replay and
  iterative candidate generation that can feed `hypersdf` projection replay.
- [hypercurve](https://github.com/timschmidt/hypercurve): exact planar curve and
  region evidence for future 2D implicit/curve handoffs.
- [hypertri](https://github.com/timschmidt/hypertri): exact triangulation target
  for future certified level-set meshing.
- [hyperpath](https://github.com/timschmidt/hyperpath): routing, offset, and
  toolpath carriers that can consume implicit boundaries after replay.
- [hypermesh](https://github.com/timschmidt/hypermesh): exact mesh validation
  consumer for replayed level-set or preview mesh handoffs.
- [hypervoxel](https://github.com/timschmidt/hypervoxel): sparse-grid storage and
  continuous-field intake target for exact SDF cell classifications.
- [hyperparts](https://github.com/timschmidt/hyperparts): part and package evidence
  that can reference implicit geometry and source-version freshness.
- [hyperdrc](https://github.com/timschmidt/hyperdrc): design-readiness review that
  can carry SDF-based mechanical or manufacturing evidence.
- [hypercircuit](https://github.com/timschmidt/hypercircuit): circuit evidence for
  future electro-mechanical field coupling.
- [hyperphysics](https://github.com/timschmidt/hyperphysics): material, contact,
  thermal, and electromagnetic consumers for implicit shapes.
- [hyperpack](https://github.com/timschmidt/hyperpack): exact packing and placement
  verification that can use SDF bounds and voxel handoffs.
- [hyperevolution](https://github.com/timschmidt/hyperevolution): search and
  optimization loops for generated implicit fields and replayed candidates.
- [hyperbrep](https://github.com/timschmidt/hyperbrep): retained BREP topology and
  analytic-surface evidence complementary to implicit SDF fields.

## Current Status

`hypersdf` is version `0.1.0`. Implemented today:

- retained `SdfExpr` expression trees for constants, coordinates, linear fields,
  primitives, CSG union/intersection/complement, arithmetic, absolute value,
  square root, trigonometric nodes, offsets, translations, and affine transforms;
- exact-friendly primitives for planes, spheres, AABBs, rounded AABBs, finite
  cylinders, capsules, tori, and slabs;
- `PreparedSdf` handles with structural facts, source-version freshness, point
  classification, conservative cell classification, intervals, gradients,
  normals, Lipschitz reports, batch reports, previews, projection replay, and
  handoff packaging;
- `SdfFacts` summaries for node counts, primitive counts, transform counts,
  parameter exactness, dyadic/common-denominator schedules, domain status,
  metric status, gradient status, and Lipschitz status;
- preview-only point/grid sampling, GLSL export, and Surface Nets mesh diagnostics;
- exact conservative voxel-cell and frame-aware `hypervoxel` handoff reports;
- optional `hypervoxel-adapter` feature for materializing continuous-field intake
  manifests into `hypervoxel` storage-facing records.

The crate does not claim that every expression is a true Euclidean signed distance.
Many routes are sign-equivalent implicit fields, and that distinction is carried by
`SdfMetricStatus`. Unsupported trig cell ranges, nonsmooth gradients, invalid
domains, stale source versions, non-octree voxel frames, and preview-only adapters
are explicit report states.

## Main Types

- `SdfExpr` is the retained expression tree. It keeps high-level object structure
  instead of immediately flattening everything into scalar samples.
- `SdfPrimitive` owns analytic shape packages: `Plane`, `Sphere`, `Aabb`,
  `RoundedAabb`, `Cylinder`, `Capsule`, `Torus`, and `Slab`.
- `SdfCoordinate` identifies coordinate fields and primitive axes.
- `PreparedSdf` caches `SdfFacts` and exposes all classification, preview, solver,
  and handoff APIs.
- `SdfFacts` records structural scheduling data and exact-parameter facts.
- `SdfPointClassificationReport` and `SdfCellClassificationReport` carry
  certified or unknown point/cell location evidence.
- `SdfMetricStatus`, `SdfDomainStatus`, `SdfEvidenceStatus`, `SdfFreshness`,
  `SdfGradientStatus`, `SdfNormalStatus`, and `SdfLipschitzStatus` separate metric
  claims, domain validity, predicate evidence, freshness, and differential support.
- `SdfIntervalReport`, `SdfGradientReport`, `SdfNormalReport`, and
  `SdfLipschitzReport` provide exact scalar ranges and differential facts where
  certified.
- `SdfBatchDispatch`, `SdfCachePayoffReport`,
  `SdfPointBatchClassificationReport`, and `SdfCellBatchClassificationReport`
  expose prepared-handle reuse without changing scalar semantics.
- `SdfPreviewGrid`, `SdfSamplingReport`, `SdfGridSamplingReport`,
  `SdfMeshPreviewReport`, and `SdfShaderExportReport` are preview-only adapter
  reports.
- `SdfProjectionProposal` and `SdfProjectionReplayReport` accept or reject external
  solver candidates by replaying exact boundary classification.
- `SdfVoxelHandoffReport`, `SdfVoxelCellGrid`, `SdfHypervoxelHandoffReport`,
  `SdfHypervoxelInterchangeManifest`, and `SdfHypervoxelInterchangeReport` bridge
  continuous fields into grid and voxel consumers.
- `SdfHandoffPackage` and `SdfHandoffRequirementReport` force downstream consumers
  to request a named domain before trusting any optional payload.

## Precision

`hypersdf` follows Yap's exact-geometric-computation discipline: topology decisions
are report facts produced from retained objects and exact predicates, not accidental
consequences of preview samples. Exact scalar values use `hyperreal::Real`, points and
planes come from `hyperlimit`, and vector/matrix structure comes from `hyperlattice`.

Several primitives are deliberately square-root-free for classification. Spheres,
rounded boxes, cylinders, capsules, and tori retain squared-radius or polynomial
forms so point and cell predicates can compare exact signs without constructing
unnecessary radicals. Domain checks reject negative squared radii and invalid widths
as `Unknown` or invalid-domain evidence rather than classifying them as outside.

Metric precision is also explicit. `SdfMetricStatus::SignEquivalent` means the zero
set and sign are useful, but the scalar is not certified as Euclidean distance.
Preview lowering, shader output, and Surface Nets meshes remain adapter data until a
consumer replays exact predicates.

## Performance

`PreparedSdf` caches structural facts once and reuses them across point, cell, batch,
gradient, interval, preview, and handoff APIs. Batch reports currently use scalar
replay but include dispatch and cache-payoff metadata so future vectorized or parallel
evaluators can preserve the same report contract.

Cell classification uses stronger primitive routes where available, including exact
AABB, plane, sphere, and interval predicates. Exact grid preview points are generated
from origin, step, and integer indices before lossy lowering. Mesh extraction uses
`fast-surface-nets` only as a preview proposal engine, while the report keeps crossing
counts, non-finite output counts, normal provenance, and preview-only topology status.

Criterion coverage in `benches/classification.rs` tracks primitives, CSG, transforms,
arithmetic, gradients, normals, intervals, Lipschitz bounds, previews, projection
replay, handoff packages, and hypervoxel grid/interchange paths.

## Numerical Explosion

`hypersdf` combats numerical explosion by keeping object packages intact until a
specific report needs a decision. A torus stays a torus polynomial, a sphere keeps
squared radius, an affine transform keeps an exact inverse matrix, and a CSG node
keeps min/max semantics. The crate does not expand every operation into a giant
scalar expression just to sample it.

Intervals and Lipschitz bounds are local, report-scoped evidence. Unsupported trig
cell ranges, nonsmooth CSG ties, ambiguous gradients, invalid domains, and failed
float lowerings become explicit unknowns. Handoff packages require named domains, so
a preview mesh or shader cannot accidentally stand in for continuous-field evidence.
Source-version freshness lets downstream crates reject stale prepared facts instead of
trusting cached classifications after a generator changes.

## Usage

Build primitives and CSG with exact parameters:

```rust,ignore
use hyperlimit::{Plane3, Point3};
use hyperreal::Real;
use hypersdf::{prepare, SdfCoordinate, SdfExpr, SdfPointLocation};

fn r(value: i32) -> Real {
    Real::from(value)
}

fn p(x: i32, y: i32, z: i32) -> Point3 {
    Point3::new(r(x), r(y), r(z))
}

let sphere = SdfExpr::sphere(p(0, 0, 0), r(25));
let slab = SdfExpr::slab(Plane3::new(p(0, 0, 1), r(0)), r(3));
let field = prepare(sphere.intersection(slab).offset(r(1)));

assert_eq!(field.classify_point(&p(0, 0, 0)).location, SdfPointLocation::Inside);
assert_eq!(field.classify_point(&p(0, 0, 4)).location, SdfPointLocation::Boundary);
assert_eq!(field.classify_point(&p(8, 0, 0)).location, SdfPointLocation::Outside);

let cylinder = prepare(SdfExpr::cylinder(SdfCoordinate::Z, p(0, 0, 0), r(25), r(3)));
assert_eq!(cylinder.classify_point(&p(3, 4, 0)).location, SdfPointLocation::Boundary);
```

Use linear fields, transforms, batches, intervals, gradients, normals, and Lipschitz
reports:

```rust,ignore
use hyperlattice::{Matrix4, Vector3};
use hypersdf::{prepare_versioned, SdfBatchDispatch, SdfFreshness};

let linear = prepare_versioned(SdfExpr::linear(Vector3([r(2), r(-3), r(5)]), r(-7)), 1);
let stale = linear.clone().with_current_source_version(2);
assert_eq!(stale.freshness(), SdfFreshness::Stale);

let points = [p(1, 0, 1), p(1, 1, 1)];
let batch = linear.classify_points_report(points.iter());
assert_eq!(batch.dispatch, SdfBatchDispatch::ScalarReplay);
assert!(batch.is_self_consistent());

let interval = linear
    .interval_cell(&p(0, 0, 0), &p(1, 1, 1))
    .interval
    .expect("linear interval");
assert_eq!(interval.upper, r(0));

let gradient = linear.gradient_point(&p(1, 0, 1));
assert!(gradient.is_certified());

let normal = linear.normal_point(&p(1, 0, 1));
assert!(normal.is_certified_direction());

let lipschitz = linear.lipschitz_cell(&p(0, 0, 0), &p(1, 1, 1));
assert!(lipschitz.is_certified());

let swap_xy = Matrix4([
    [r(0), r(1), r(0), r(0)],
    [r(1), r(0), r(0), r(0)],
    [r(0), r(0), r(1), r(0)],
    [r(0), r(0), r(0), r(1)],
]);
let transformed = prepare(SdfExpr::x().affine_transform(swap_xy)?);
assert_eq!(
    transformed.classify_point(&p(10, 0, 0)).location,
    SdfPointLocation::Boundary
);
```

Preview samples, meshes, and shader source without promoting them to topology:

```rust,ignore
use hypersdf::{SdfPreviewGrid, SdfSampleTopologyStatus, SdfSamplingPrecision};

let sdf = prepare(SdfExpr::sphere(p(0, 0, 0), r(25)));
let points = [p(0, 0, 0), p(3, 4, 0), p(8, 0, 0)];
let samples = sdf.sample_points_preview(points.iter(), SdfSamplingPrecision::F32);
assert_eq!(samples.topology_status, SdfSampleTopologyStatus::PreviewOnly);
assert!(samples.is_self_consistent());

let grid = SdfPreviewGrid::new(p(-6, -6, -6), p(3, 3, 3), [5, 5, 5]);
let mesh = sdf
    .mesh_preview_from_grid(grid.clone(), SdfSamplingPrecision::F32)
    .expect("valid preview grid");
assert_eq!(mesh.topology_status, SdfSampleTopologyStatus::PreviewOnly);
assert!(mesh.is_self_consistent());

let shader = sdf.export_glsl_preview("field", SdfSamplingPrecision::F32);
assert!(shader.is_complete());
```

Replay solver proposals and package named handoff domains:

```rust,ignore
use hypersdf::{
    SdfHandoffDomain, SdfProjectionProposal, SdfProjectionProposalKind,
    SdfProjectionReplayStatus, SdfVoxelCellGrid, SdfVoxelGridSource, SdfVoxelLengthUnit,
};

let sdf = prepare(SdfExpr::sphere(p(0, 0, 0), r(25)));
let projection = sdf.replay_projection_proposal(SdfProjectionProposal::new(
    "closest-point-fixture",
    SdfProjectionProposalKind::ClosestPoint,
    p(10, 0, 0),
    p(5, 0, 0),
));
assert_eq!(projection.status, SdfProjectionReplayStatus::BoundaryCertified);

let voxel_grid = SdfVoxelCellGrid::new(p(-4, -4, -4), p(4, 4, 4), [2, 2, 2])
    .with_units(SdfVoxelLengthUnit::Millimeter)
    .with_source(SdfVoxelGridSource::new("sdf:sphere", 1));
let voxel_report = sdf
    .classify_voxel_grid_for_handoff(voxel_grid)
    .expect("valid exact voxel grid");
assert!(voxel_report.hypervoxel_ready());

let package = sdf
    .handoff_package()
    .with_hypervoxel_grid(voxel_report)
    .with_projection_replay(projection);

assert!(package.require_domain(SdfHandoffDomain::ContinuousField).is_ready());
assert!(package.require_domain(SdfHandoffDomain::HypervoxelGrid).is_ready());
assert!(package.require_domain(SdfHandoffDomain::ProjectionReplay).is_ready());
```

With the optional `hypervoxel-adapter` feature, `continuous_field_manifest_from_sdf`
can lower a ready `SdfHypervoxelHandoffReport` into a `hypervoxel`
`ContinuousFieldVoxelManifest` using the caller's `GridFrame` and material id.

## Development

Useful local checks:

```sh
cargo test
cargo bench --bench classification
cargo test --features hypervoxel-adapter
```

## References

- Gibson, Sarah F. F. "Constrained Elastic Surface Nets: Generating Smooth Surfaces from Binary Segmented Data." *Medical Image Computing and Computer-Assisted Intervention*, 1998, pp. 888-898, https://doi.org/10.1007/BFb0056308.
- Hart, John C. "Sphere Tracing: A Geometric Method for the Antialiased Ray Tracing of Implicit Surfaces." *The Visual Computer*, vol. 12, no. 10, 1996, pp. 527-545, https://doi.org/10.1007/s003710050084.
- Lorensen, William E., and Harvey E. Cline. "Marching Cubes: A High Resolution 3D Surface Construction Algorithm." *Computer Graphics*, vol. 21, no. 4, 1987, pp. 163-169, https://doi.org/10.1145/37402.37422.
- Moore, Ramon E. *Interval Analysis*. Prentice-Hall, 1966.
- Frisken, Sarah F., et al. "Adaptively Sampled Distance Fields: A General Representation of Shape for Computer Graphics." *Proceedings of SIGGRAPH 2000*, 2000, pp. 249-254, https://doi.org/10.1145/344779.344899.
- Yap, Chee K. "Towards Exact Geometric Computation." *Computational Geometry*, vol. 7, nos. 1-2, 1997, pp. 3-23, https://doi.org/10.1016/0925-7721(95)00040-2.
