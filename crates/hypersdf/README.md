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

## Current Status

`hypersdf` is version `0.2.0`. Implemented today:

- retained `SdfExpr` expression trees for constants, coordinates, linear fields,
  primitives, CSG union/intersection/complement, arithmetic, absolute value,
  square root, trigonometric nodes, offsets, translations, and affine transforms;
- exact-friendly primitives for planes, spheres, AABBs, rounded AABBs, finite
  cylinders, capsules, tori, and slabs;
- `Sdf` fields with cached structural facts, immediate point and conservative
  cell classification, intervals, gradients, normals, Lipschitz reports,
  previews, projection replay, and voxel classification;
- `SdfFacts` summaries for node counts, primitive counts, transform counts,
  parameter exactness, dyadic/common-denominator schedules, domain status,
  metric status, gradient status, and Lipschitz status;
- preview-only point/grid sampling, GLSL export, and Surface Nets mesh diagnostics;
- exact conservative voxel-cell batches and frame-aware `hypervoxel` lowering;
- optional `hypervoxel-adapter` feature for materializing continuous-field intake
  batches into `hypervoxel` storage-facing records.

The crate does not claim that every expression is a true Euclidean signed distance.
Many routes are sign-equivalent implicit fields, and that distinction is carried by
`SdfMetricStatus`. Unsupported trig cell ranges, nonsmooth gradients, invalid
domains, unsupported voxel frames, and preview-only adapters are explicit report
states.

## Main Types

- `SdfExpr` is the retained expression tree. It keeps high-level object structure
  instead of immediately flattening everything into scalar samples.
- `SdfPrimitive` owns analytic shape packages: `Plane`, `Sphere`, `Aabb`,
  `RoundedAabb`, `Cylinder`, `Capsule`, `Torus`, and `Slab`.
- `SdfCoordinate` identifies coordinate fields and primitive axes.
- `Sdf` retains an expression, caches `SdfFacts`, and exposes the classification,
  preview, solver-replay, and voxel APIs directly.
- `SdfFacts` records structural scheduling data and exact-parameter facts.
- `SdfPointClassificationReport` and `SdfCellClassificationReport` carry
  certified or unknown point/cell location evidence.
- `SdfMetricStatus`, `SdfDomainStatus`, `SdfEvidenceStatus`,
  `SdfGradientStatus`, `SdfNormalStatus`, and `SdfLipschitzStatus` separate metric
  claims, domain validity, predicate evidence, and differential support.
- `SdfIntervalReport`, `SdfGradientReport`, `SdfNormalReport`, and
  `SdfLipschitzReport` provide exact scalar ranges and differential facts where
  certified.
- `Sdf::classify_points` and `Sdf::classify_cells` provide immediate batch
  queries without changing scalar semantics.
- `SdfPreviewGrid`, `SdfSamplingReport`, `SdfGridSamplingReport`,
  `SdfMeshPreviewReport`, and `SdfShaderExportReport` are preview-only adapter
  reports.
- `SdfProjectionProposal` and `SdfProjectionReplayReport` accept or reject external
  solver candidates by replaying exact boundary classification.
- `SdfVoxelCellGrid`, `SdfVoxelBatch`, and the optional
  `continuous_field_batch_from_sdf` adapter bridge continuous fields into voxel
  consumers.

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

`Sdf::new` caches structural facts once and the field reuses them across point,
cell, batch, gradient, interval, preview, and voxel APIs. Batch queries currently
use scalar replay, leaving room for later vectorized or parallel evaluators without
changing the report contract.

Point reports carry the scalar produced during classification instead of evaluating the
expression a second time. Exact affine intervals use per-axis interval accumulation,
sphere/AABB bounds use separable squared-distance extrema, and regular preview grids
reuse each exact axis coordinate across the other two dimensions. These are arithmetic
schedule changes only; classification evidence and preview/topology boundaries are
unchanged. Reproducible measurements and the reference-by-reference audit are recorded
in [`PERFORMANCE.md`](PERFORMANCE.md).

Cell classification uses stronger primitive routes where available, including exact
AABB, plane, sphere, and interval predicates. Exact grid preview points are generated
from origin, step, and integer indices before lossy lowering. Mesh extraction uses
`fast-surface-nets` only as a preview proposal engine, while the report keeps crossing
counts, non-finite output counts, normal provenance, and preview-only topology status.

Criterion coverage in `benches/classification.rs` tracks primitives, CSG, transforms,
arithmetic, gradients, normals, intervals, Lipschitz bounds, previews, projection
replay, and voxel-grid classification.

## Numerical Explosion

`hypersdf` combats numerical explosion by keeping object packages intact until a
specific report needs a decision. A torus stays a torus polynomial, a sphere keeps
squared radius, an affine transform keeps an exact inverse matrix, and a CSG node
keeps min/max semantics. The crate does not expand every operation into a giant
scalar expression just to sample it.

Intervals and Lipschitz bounds are local, report-scoped evidence. Unsupported trig
cell ranges, nonsmooth CSG ties, ambiguous gradients, invalid domains, and failed
float lowerings become explicit unknowns. Preview meshes and shaders remain named
lossy outputs rather than continuous-field evidence. Callers construct a new `Sdf`
when an expression changes, so cached facts and the retained expression cannot
diverge.

## Usage

Build primitives and CSG with exact parameters:

```rust,no_run
use hyperlimit::{Plane3, Point3};
use hyperreal::Real;
use hypersdf::{Sdf, SdfCoordinate, SdfExpr, SdfPointLocation};

fn r(value: i32) -> Real {
    Real::from(value)
}

fn p(x: i32, y: i32, z: i32) -> Point3 {
    Point3::new(r(x), r(y), r(z))
}

let sphere = SdfExpr::sphere(p(0, 0, 0), r(25));
let slab = SdfExpr::slab(Plane3::new(p(0, 0, 1), r(0)), r(3));
let field = Sdf::new(sphere.intersection(slab).offset(r(1)));

assert_eq!(field.classify_point(&p(0, 0, 0)).location, SdfPointLocation::Inside);
assert_eq!(field.classify_point(&p(0, 0, 4)).location, SdfPointLocation::Boundary);
assert_eq!(field.classify_point(&p(8, 0, 0)).location, SdfPointLocation::Outside);

let cylinder = Sdf::new(SdfExpr::cylinder(
    SdfCoordinate::Z,
    p(0, 0, 0),
    r(25),
    r(3),
));
assert_eq!(cylinder.classify_point(&p(3, 4, 0)).location, SdfPointLocation::Boundary);
```

Use linear fields, transforms, batches, intervals, gradients, normals, and Lipschitz
reports:

```rust,ignore
use hyperlattice::{Matrix4, Vector3};
use hypersdf::{Sdf, SdfExpr};

let linear = Sdf::new(SdfExpr::linear(Vector3([r(2), r(-3), r(5)]), r(-7)));

let points = [p(1, 0, 1), p(1, 1, 1)];
let batch = linear.classify_points(points.iter());
assert_eq!(batch.len(), points.len());

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
let transformed = Sdf::new(SdfExpr::x().affine_transform(swap_xy)?);
assert_eq!(
    transformed.classify_point(&p(10, 0, 0)).location,
    SdfPointLocation::Boundary
);
```

Preview samples, meshes, and shader source without promoting them to topology:

```rust,ignore
use hypersdf::{Sdf, SdfPreviewGrid, SdfSampleTopologyStatus, SdfSamplingPrecision};

let sdf = Sdf::new(SdfExpr::sphere(p(0, 0, 0), r(25)));
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

Replay solver proposals and classify an exact voxel grid:

```rust,ignore
use hypersdf::{
    Sdf, SdfProjectionProposal, SdfProjectionProposalKind,
    SdfProjectionReplayStatus, SdfVoxelCellGrid, SdfVoxelLengthUnit,
};

let sdf = Sdf::new(SdfExpr::sphere(p(0, 0, 0), r(25)));
let projection = sdf.replay_projection_proposal(SdfProjectionProposal::new(
    "closest-point-fixture",
    SdfProjectionProposalKind::ClosestPoint,
    p(10, 0, 0),
    p(5, 0, 0),
));
assert_eq!(projection.status, SdfProjectionReplayStatus::BoundaryCertified);

let voxel_grid = SdfVoxelCellGrid::new(p(-4, -4, -4), p(4, 4, 4), [2, 2, 2])
    .with_units(SdfVoxelLengthUnit::Millimeter);
let voxel_batch = sdf
    .classify_voxel_grid(voxel_grid)
    .expect("valid exact voxel grid");
assert!(voxel_batch.is_complete());
```

With the optional `hypervoxel-adapter` feature,
`continuous_field_batch_from_sdf` lowers an `SdfVoxelBatch` into
`hypervoxel` continuous-field intake records.

## Development

```sh
cargo fmt --all -- --check
cargo test --locked
cargo check --benches --locked
cargo clippy --all-targets --locked -- -D warnings
RUSTDOCFLAGS="-D warnings" cargo doc --no-deps --locked
cargo bench --bench classification
cargo test --locked --features hypervoxel-adapter
cargo test --locked --all-features
```

## References

Implementation comments describe local invariants and evidence boundaries; the
algorithmic and numerical background is consolidated here.

- Gibson, Sarah F. F. "Constrained Elastic Surface Nets: Generating Smooth Surfaces from Binary Segmented Data." *Medical Image Computing and Computer-Assisted Intervention*, 1998, pp. 888-898, https://doi.org/10.1007/BFb0056277.
- Arvo, James. "Transforming Axis-Aligned Bounding Boxes." *Graphics Gems*, Academic Press, 1990, pp. 548-550.
- Hart, John C. "Sphere Tracing: A Geometric Method for the Antialiased Ray Tracing of Implicit Surfaces." *The Visual Computer*, vol. 12, no. 10, 1996, pp. 527-545, https://doi.org/10.1007/s003710050084.
- Ju, Tao, et al. "Dual Contouring of Hermite Data." *Proceedings of SIGGRAPH 2002*, 2002, pp. 339-346, https://doi.org/10.1145/566570.566586.
- Lorensen, William E., and Harvey E. Cline. "Marching Cubes: A High Resolution 3D Surface Construction Algorithm." *Computer Graphics*, vol. 21, no. 4, 1987, pp. 163-169, https://doi.org/10.1145/37402.37422.
- Moore, Ramon E. *Interval Analysis*. Prentice-Hall, 1966.
- Frisken, Sarah F., et al. "Adaptively Sampled Distance Fields: A General Representation of Shape for Computer Graphics." *Proceedings of SIGGRAPH 2000*, 2000, pp. 249-254, https://doi.org/10.1145/344779.344899.
- Yap, Chee K. "Towards Exact Geometric Computation." *Computational Geometry*, vol. 7, nos. 1-2, 1997, pp. 3-23, https://doi.org/10.1016/0925-7721(95)00040-2.

## Hyper Ecosystem

`hypersdf` builds exact fields over [hyperreal](https://github.com/timschmidt/hyperreal),
[hyperlattice](https://github.com/timschmidt/hyperlattice), and
[hyperlimit](https://github.com/timschmidt/hyperlimit). It exchanges solver,
mesh, and grid evidence with [hypersolve](https://github.com/timschmidt/hypersolve),
[hypermesh](https://github.com/timschmidt/hypermesh), and
[hypervoxel](https://github.com/timschmidt/hypervoxel); complementary analytic
topology lives in [hyperbrep](https://github.com/timschmidt/hyperbrep).
