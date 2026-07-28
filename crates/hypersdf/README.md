# Hypersdf

Exact-aware signed-distance and implicit-field carriers for the Hyper geometry
stack.

Hypersdf retains continuous-field expression structure, exact parameters,
classification evidence, interval and differential reports, preview adapters,
solver replay, and voxel handoffs. It can answer inside/boundary/outside
questions without treating primitive-float samples or a preview mesh as
topological truth.

The crate owns implicit fields. Solid-modeling grammar belongs in CSGRS,
analytic boundary topology belongs in Hyperbrep, exact triangle topology
belongs in Hypermesh, and sampled hierarchical storage belongs in Hypervoxel.

This README describes crate version `0.2.0`.

## Primary types

| Type | Role |
| --- | --- |
| `SdfExpr` | Retained implicit expression tree |
| `SdfPrimitive` | Plane, sphere, box, rounded box, cylinder, capsule, torus, or slab |
| `Sdf` | Expression plus cached structural facts and query API |
| `SdfFacts` | Node, primitive, transform, exactness, domain, metric, and schedule summary |
| `SdfPointClassificationReport`, `SdfCellClassificationReport` | Exact/certified location evidence |
| `SdfIntervalReport`, `SdfGradientReport`, `SdfNormalReport`, `SdfLipschitzReport` | Scalar-range and differential evidence |
| `SdfPreviewGrid`, `SdfSamplingReport`, `SdfMeshPreviewReport` | Explicitly lossy preview data |
| `SdfProjectionReplayReport` | Exact replay of an external solver proposal |
| `SdfVoxelCellGrid`, `SdfVoxelBatch` | Conservative voxel classification |

## Install

```toml
[dependencies]
hypersdf = "0.2.0"
```

There are no default features. Enable `hypervoxel-adapter` only when
materializing a classified field batch into Hypervoxel records.

## Quick start

This example intersects a sphere with a slab, offsets the result, and performs
three exact point classifications.

<!-- quickstart:start -->
```rust
use hyperlimit::{Plane3, Point3};
use hyperreal::Real;
use hypersdf::{Sdf, SdfExpr, SdfPointLocation};

fn r(value: i32) -> Real {
    Real::from(value)
}

fn p(x: i32, y: i32, z: i32) -> Point3 {
    Point3::new(r(x), r(y), r(z))
}

fn main() {
    let sphere = SdfExpr::sphere(p(0, 0, 0), r(25));
    let slab = SdfExpr::slab(Plane3::new(p(0, 0, 1), r(0)), r(3));
    let field = Sdf::new(sphere.intersection(slab).offset(r(1)));

    assert_eq!(
        field.classify_point(&p(0, 0, 0)).location,
        SdfPointLocation::Inside
    );
    assert_eq!(
        field.classify_point(&p(0, 0, 4)).location,
        SdfPointLocation::Boundary
    );
    assert_eq!(
        field.classify_point(&p(8, 0, 0)).location,
        SdfPointLocation::Outside
    );
}
```
<!-- quickstart:end -->

Run the checked copy:

```sh
cargo run --example basic
```

## Field and evidence model

```text
SdfPrimitive / coordinate / constant
                 │
              SdfExpr
      arithmetic / CSG / transform
                 │
                Sdf  ── cached SdfFacts
                 │
      ┌──────────┼──────────────┐
 exact reports  preview data  solver/voxel handoffs
```

An `SdfExpr` retains object packages until a query requires a decision. A torus
stays a torus polynomial, a sphere keeps its squared radius, an affine
transform keeps its exact inverse matrix, and a CSG node keeps its min/max
semantics. Construct a new `Sdf` when the expression changes so its cached
facts cannot diverge.

“SDF” is the ecosystem term, but not every supported expression is certified
as a Euclidean signed distance. `SdfMetricStatus::SignEquivalent` means the
zero set and sign are usable while the scalar distance itself is not a metric
guarantee.

## API guide

### Building expressions

- `SdfExpr::{constant, x, y, z, linear}` creates scalar coordinate fields.
- `SdfExpr::{plane, sphere, aabb, rounded_aabb, cylinder, capsule, torus,
  slab}` creates retained analytic primitives. Radius arguments named
  `radius_squared` are squared radii.
- `union`, `intersection`, and `complement` build regularized sign-based CSG.
- `add_expr`, `sub_expr`, `mul_expr`, `abs`, `sqrt`, `sin`, `cos`, and `tan`
  build scalar expressions.
- `offset`, `translate`, and `affine_transform` transform a field.
- `SdfTransform::{translation, affine, inverse_point, inverse_aabb}` exposes
  the checked transform package directly.
- `SdfExpr::metric_status`, `Sdf::metric_status`, and `Sdf::facts` describe
  what the resulting scalar can certify.

### Exact and conservative queries

- `Sdf::classify_point` returns scalar, location, domain, metric, and evidence
  status for one exact point; `classify_points` batches the same semantics.
- `classify_cell` conservatively classifies an exact axis-aligned cell;
  `classify_cells` is the batch form.
- `interval_cell` returns the supported exact scalar interval over a cell.
- `gradient_point` and `normal_point` return differential status and evidence;
  `gradient_points` and `normal_points` batch them.
- `lipschitz_cell` reports a supported local Lipschitz bound.
- `SdfDomainStatus`, `SdfEvidenceStatus`, `SdfMetricStatus`,
  `SdfGradientStatus`, `SdfNormalStatus`, and `SdfLipschitzStatus` keep
  independent claims independent.

### Preview and proposal APIs

- `sample_points_preview` and `sample_grid_preview` lower exact query points to
  the selected `SdfSamplingPrecision`.
- `mesh_preview_from_grid` runs Surface Nets diagnostics over a preview grid.
- `dual_contouring_report_from_grid` and
  `gradient_contouring_report_from_grid` retain sampled crossings, proposed
  vertices, connectivity, validation readiness, and blockers.
- `export_glsl_preview` emits a named GLSL field and an explicit completeness
  report.
- `replay_projection_proposal` accepts a proposed surface point from another
  solver and replays exact boundary classification before certifying it.

These APIs return proposals or display data. None promotes sampled triangles,
finite gradients, or shader evaluation into exact topology.

### Voxel handoff

- `SdfVoxelCellGrid::{new, with_units, cell_count, validate_positive_step}`
  defines an exact classification grid.
- `Sdf::classify_voxel_grid` returns one conservative `SdfVoxelCell` per cell
  in an `SdfVoxelBatch`; `is_complete` and `has_unknown` summarize it.
- With `hypervoxel-adapter`, `continuous_field_batch_from_sdf` lowers that
  batch into Hypervoxel’s continuous-field intake records.

## Precision and guarantees

- Scalars use `hyperreal::Real`; points and planes use Hyperlimit; vectors and
  matrices use Hyperlattice.
- Point and cell topology comes from retained expressions and exact or
  certified predicates, not from preview samples.
- Sphere, rounded-box, cylinder, capsule, and torus classification retains
  squared-radius or polynomial forms to avoid unnecessary radicals.
- Invalid domains, unsupported trigonometric cell ranges, nonsmooth CSG ties,
  ambiguous gradients, and failed finite lowering are explicit statuses or
  errors.
- Exact regular preview-grid points are formed from origin, step, and integer
  indices before any requested finite conversion.
- `Sdf::new` computes structural facts once; subsequent queries reuse those
  facts without changing classification semantics.

A supported expression node does not imply every interval, derivative, metric,
or topology query is certifiable for every `Real` value. Check the report’s
status rather than interpreting unknown evidence as outside geometry.

## Feature flags

| Feature | Default | Purpose |
| --- | --- | --- |
| `dispatch-trace` | no | Hyperreal/Hyperlimit predicate-dispatch instrumentation |
| `hypervoxel-adapter` | no | Materialize SDF voxel batches into Hypervoxel intake records |

## Validation and performance

```sh
cargo fmt --all -- --check
cargo test --locked
cargo test --locked --all-features
cargo clippy --all-targets --all-features -- -D warnings
RUSTDOCFLAGS="-D warnings" cargo doc --no-deps --all-features
cargo check --benches --all-features
```

Reproducible benchmark definitions and the reference-guided performance audit
live in [PERFORMANCE.md](PERFORMANCE.md). The benchmark suite covers primitive,
CSG, transform, differential, interval, preview, replay, and voxel queries.

## References

These sources describe the exact-computation, interval, implicit-surface, and
preview-meshing ideas relevant to the crate:

- Yap, C. K. “Towards Exact Geometric Computation.” *Computational Geometry*
  7(1–2), 1997, 3–23.
  [DOI: 10.1016/0925-7721(95)00040-2](https://doi.org/10.1016/0925-7721(95)00040-2).
- Moore, R. E. *Interval Analysis*. Prentice-Hall, 1966.
- Hart, J. C. “Sphere Tracing: A Geometric Method for the Antialiased Ray
  Tracing of Implicit Surfaces.” *The Visual Computer* 12(10), 1996, 527–545.
  [DOI: 10.1007/s003710050084](https://doi.org/10.1007/s003710050084).
- Frisken, S. F., Perry, R. N., Rockwood, A. P., and Jones, T. R.
  “Adaptively Sampled Distance Fields: A General Representation of Shape for
  Computer Graphics.” *Proceedings of SIGGRAPH 2000*, 249–254.
  [DOI: 10.1145/344779.344899](https://doi.org/10.1145/344779.344899).
- Gibson, S. F. F. “Constrained Elastic Surface Nets: Generating Smooth
  Surfaces from Binary Segmented Data.” *MICCAI 1998*, 888–898.
  [DOI: 10.1007/BFb0056277](https://doi.org/10.1007/BFb0056277).
- Ju, T., Losasso, F., Schaefer, S., and Warren, J. “Dual Contouring of
  Hermite Data.” *Proceedings of SIGGRAPH 2002*, 339–346.
  [DOI: 10.1145/566570.566586](https://doi.org/10.1145/566570.566586).
- Lorensen, W. E., and Cline, H. E. “Marching Cubes: A High Resolution 3D
  Surface Construction Algorithm.” *Computer Graphics* 21(4), 1987, 163–169.
  [DOI: 10.1145/37402.37422](https://doi.org/10.1145/37402.37422).
- Arvo, J. “Transforming Axis-Aligned Bounding Boxes.” In *Graphics Gems*,
  Academic Press, 1990, 548–550.

## Acknowledgements

Hypersdf builds on
[Hyperreal](https://github.com/timschmidt/hyperreal),
[Hyperlattice](https://github.com/timschmidt/hyperlattice), and
[Hyperlimit](https://github.com/timschmidt/hyperlimit), with optional
[Hypervoxel](https://github.com/timschmidt/hypervoxel) integration.

Preview meshing uses the
[`fast-surface-nets`](https://crates.io/crates/fast-surface-nets) crate as a
proposal engine. The research cited above informs the evidence and adapter
boundaries; it does not imply source-code derivation.

## License and contributing

Licensed under the [Apache License 2.0](LICENSE).

Bug reports should include the smallest expression, exact query point or cell,
enabled features, and all returned statuses. Before proposing a change, run
formatting, the focused regression, the complete feature suite, and strict
Clippy.
