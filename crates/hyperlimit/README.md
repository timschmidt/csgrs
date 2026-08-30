<h1>
  hyperlimit
  <img src="./doc/hyperlimit.png" alt="Hyper, a clever mathematician" width="144" align="right">
</h1>

`hyperlimit` provides exact-aware geometric predicates over
`hyperreal::Real`. Each predicate returns either a classified value with the
stage and certainty that decided it, or explicit uncertainty.

The crate owns reusable predicate semantics, exact constructions that support
those predicates, and strict escalation. It does not own curves,
triangulations, meshes, BSP trees, CSG grammar, or application topology.

## Why a predicate layer?

Geometry algorithms change combinatorial structure at branch points: a point
is left or right of a line, inside or outside a ring, above or below a plane,
or within a circumsphere. An epsilon comparison can make those answers depend
on scale and can give different parts of an algorithm inconsistent results.

Hyperlimit makes the decision path visible:

```text
Real coordinates + retained object facts
                  │
                  ▼
 structural facts / exact reducers / certified filters
                  │
                  ▼
          bounded Real refinement
             ┌────┴────┐
             ▼         ▼
          Decided    Unknown
       value + stage  needed + stage
```

An `Unknown` outcome is not a false or zero result. Higher topology code must
propagate it, request more capability, or choose an explicitly documented
policy outside this crate.

## Primary types

| Type | Purpose |
| --- | --- |
| `PredicateOutcome<T>` | A decided value with certainty/provenance, or an unresolved result. |
| `Sign`, `SignKnowledge` | Exact sign and partial sign knowledge. |
| `Certainty`, `Escalation`, `RefinementNeed` | Describe why a result is trustworthy or what remains unresolved. |
| `PredicatePolicy` | The crate's strict bounded-refinement policy. |
| `Point2`, `Point3` | Predicate-facing re-exports of Hyperlattice points. |
| `Plane3` | Plane represented by `normal · point + offset = 0`. |
| `LineSide`, `PlaneSide`, location/relation enums | Typed classifications for geometric queries. |
| Evidence and `*Facts` types | Reusable exact structure for repeated queries. |
| Report and validation types | Replayable classifications with retained intermediate evidence. |
| `SupportDop3`, `WitnessedSupportDop3` | Exact support-slab bounding volumes, optionally retaining source witnesses. |
| `error::PredicateError`, `error::Result<T>` | Construction and validation errors. |

## Quick start

Create a project and add the crate:

```sh
cargo new exact-predicates
cd exact-predicates
cargo add hyperlimit
```

Equivalent manifest entry:

```toml
[dependencies]
hyperlimit = "0.4.1"
```

Replace `src/main.rs` with:

<!-- quickstart:start -->
```rust
use hyperlimit::{Point2, Real, Sign, orient2};

fn main() {
    let a = Point2::new(Real::from(0), Real::from(0));
    let b = Point2::new(Real::from(1), Real::from(0));
    let c = Point2::new(Real::from(0), Real::from(1));

    let orientation = orient2(&a, &b, &c, hyperlimit::PredicatePolicy::APPROXIMATE_512);
    assert_eq!(orientation.value(), Some(Sign::Positive));
    println!("{orientation:?}");
}
```
<!-- quickstart:end -->

Run it with `cargo run`. The same source is checked in as
[`examples/readme_quickstart.rs`](examples/readme_quickstart.rs), compiled by
the test suite, and compared with this README block.

## Reading predicate outcomes

`PredicateOutcome::Decided` contains `value`, `certainty`, and `stage`.
`PredicateOutcome::Unknown` contains the `RefinementNeed` and the stage where
evaluation stopped. Use `value()` only when discarding that diagnostic context
is acceptable:

```rust
use hyperlimit::{PredicateOutcome, RefinementNeed};

fn require_decided<T>(outcome: PredicateOutcome<T>) -> Result<T, RefinementNeed> {
    match outcome {
        PredicateOutcome::Decided { value, .. } => Ok(value),
        PredicateOutcome::Unknown { needed, .. } => Err(needed),
    }
}
```

## API guide

### Signs, order, and intervals

| Task | API |
| --- | --- |
| Classify one scalar | `classify_real_sign`, `RealPredicateExt` |
| Compare scalars | `compare_reals`, `compare_reals_with_policy`, `real_le`, `real_ge`, `real_min`, `real_max`, `real_clamp` |
| Compare points | `compare_point2_lexicographic`, `compare_point3_lexicographic`, `point2_equal`, `point3_equal` |
| Classify a closed interval | `classify_real_closed_interval`, `real_in_closed_interval` |
| Intersect intervals | `classify_closed_interval_intersection`, `closed_intervals_intersect` |
| Use certified scalar filters | `certified_ball_sign`, `certified_interval_sign`, `classify_ball_sign_with_policy` |

### Orientation and lifted predicates

| Task | API |
| --- | --- |
| 2D/3D orientation | `orient2`, `orient2_with_policy`, `orient3` |
| Point against a directed line | `classify_point_line` |
| Retain a line orientation | `line2_orientation`, `line2_orientation_with_facts`, `classify_point_line_with_orientation` |
| In-circle | `incircle2`, `incircle2_evidence`, `incircle2_with_evidence` |
| In-sphere | `insphere3`, `insphere3_evidence`, `insphere3_with_evidence` |
| D-dimensional predicates | `orient_d`, `insphere_d`, `affine_independent_d` |

Evidence-based classifiers also expose `*_with_policy` variants for callers
that need an explicit `PredicatePolicy`.

### Segments, rings, and planar regions

| Task | API |
| --- | --- |
| Point on segment | `classify_point_segment`, `classify_point_segment3`, `point_on_segment`, `point_on_segment3` |
| Segment intersection | `classify_segment_intersection`, `classify_segment3_intersection`, `construct_line_intersection_point` |
| Reuse 2D facts | `point2_displacement_facts`, `segment2_facts`, `triangle2_facts`, `classify_point_segment_with_facts`, `classify_segment_intersection_with_facts` |
| Ring structure | `ring2_facts`, `indexed_ring2_facts`, `ring_area_sign`, `indexed_ring_area_sign`, `ring_convexity`, `indexed_ring_convexity` |
| Point in ring | `classify_point_ring_even_odd`, `classify_point_indexed_ring_even_odd`, `point_in_ring_even_odd`, `point_in_indexed_ring_even_odd` |
| Replay ring decisions | `classify_point_ring_even_odd_report`, `classify_point_indexed_ring_even_odd_report` |
| Convex containment | `classify_point_convex_polygon2`, `classify_point_convex_planes3` |

### AABBs, distances, circles, and spheres

| Task | API |
| --- | --- |
| Point/AABB | `classify_point_aabb2`, `classify_point_aabb3`, `point_in_aabb2`, `point_in_aabb3` |
| AABB/AABB | `classify_aabb2_intersection`, `classify_aabb3_intersection`, `aabb2s_intersect`, `aabb3s_intersect` |
| Ordered AABBs | `ordered_aabb2s_intersect_coordinates`, `point_in_ordered_aabb2_coordinates`, `ordered_aabb3_contains`, `ordered_aabb3s_intersect`, `point_in_ordered_aabb3_relative_interior` |
| Reuse box facts | `aabb2_facts`, `classify_aabb2_intersection_with_facts`, `point_in_triangle2_aabb` |
| Compare squared distances | `compare_point2_distance_squared`, `compare_point3_distance_squared`, `compare_point_line3_distance_squared`, `compare_point_segment3_distance_squared`, `compare_point_plane_distance_squared` |
| Circle relations | `classify_circle_line2`, `classify_circle_segment2` |
| Sphere relations | `classify_point_sphere3`, `classify_sphere3_intersection`, `classify_aabb3_sphere_intersection` |

### Planes and exact constructions

Construct `Plane3` with its public `normal: Point3` and `offset: Real` fields.

| Task | API |
| --- | --- |
| Point/plane | `classify_point_plane`, `classify_point_oriented_plane` |
| Plane/segment or triangle | `classify_plane_segment`, `classify_plane_triangle`, `classify_triangle_against_oriented_plane` |
| Plane/AABB | `classify_plane_aabb3`, `classify_plane_aabb3_report` |
| Retain evidence | `plane3_evidence`, `oriented_plane3_evidence`, corresponding `classify_*_with_evidence` methods |
| Homogeneous intersections | `intersect_two_planes`, `intersect_three_planes`, `intersect_homogeneous_line_plane` |
| Homogeneous incidence | `classify_homogeneous_point_plane` |
| Segment/plane construction | `intersect_segment_with_plane`, `intersect_segment_with_oriented_plane`, `intersect_segment_with_plane_values` |
| Validate/reconstruct crossings | `construct_segment_plane_crossing_from_values`, `interpolate_point3`, `point_plane_value`, `segment_parameter_from_axis` |

Reports expose `validate` and `validate_against_sources`/`*_triangles` methods
so retained decisions can be checked against their source geometry.

### Triangles and tetrahedra

| Task | API |
| --- | --- |
| Point/triangle | `classify_point_triangle`, `classify_point_triangle3`, facts/orientation reuse variants |
| Triangle orientation and degeneracy | `triangle3_orientation`, `triangle3_winding_normal_sign`, `classify_triangle3_degeneracy` |
| Segment or ray/triangle | `classify_segment_triangle3_intersection`, `classify_ray_triangle3_intersection` and report variants |
| Triangle/triangle | `classify_triangle_triangle3`, `classify_triangle_triangle3_with_policy`, `classify_triangle_triangle3_points_with_policy` |
| Point/tetrahedron | `classify_point_tetrahedron` |
| Coplanar triangles | `classify_coplanar_triangles`, `classify_coplanar_triangle_points`, `derive_coplanar_triangle_relation` |
| Coplanar projection | `choose_coplanar_projection`, `project_point3`, `project_triangle3`, projected area, turn, line, and segment helpers |

### Support DOPs and halfspaces

| Task | API |
| --- | --- |
| Build a support DOP | `SupportDop3::from_points`, `support_dop3_from_points` |
| Retain witnesses | `WitnessedSupportDop3::from_points`, `witnessed_support_dop3_from_points` |
| Inspect or update | `slabs`, `validate`, `validate_against_points`, `refresh_for_changed_vertices`, `to_support_dop3` |
| Classify | `classify_point`, `classify_aabb3`, `classify_plane3` and report variants |
| Work with slabs/axes | `SupportDopAxis3`, `SupportSlab3::new`, `project_point` |
| Test convex feasibility | `classify_halfspace_feasibility3`, `HalfspaceFeasibilityReport`, `HalfspaceInfeasibilityCertificate` |

Witness and report types make broad-phase decisions replayable without turning
their cached data into an unchecked certificate.

### Batch evaluation

Sequential batch front doors are `orient2_batch`, `orient3_batch`,
`incircle2_batch`, `insphere3_batch`, `classify_point_line_batch`,
`classify_point_plane_batch`, `classify_point_oriented_plane_batch`,
`classify_segment3_intersection_batch`,
`classify_segment_triangle3_intersection_batch`,
`classify_ray_triangle3_intersection_batch`,
`classify_circle_line2_batch`, and `classify_circle_segment2_batch`.

With `parallel`, the same names gain a `_parallel` suffix and use Rayon.
Associated `*Case` aliases document each batch tuple shape.

## Features

| Feature | Default | Effect |
| --- | --- | --- |
| `std` | yes | Standard-library support used by the current crate build. |
| `parallel` | no | Enables Rayon-backed parallel batch variants; implies `std`. |
| `dispatch-trace` | no | Enables lower-stack predicate/scalar dispatch instrumentation. |
| `serde` | no | Serializes the shared exact `Point2` carrier. |

## Validation and profiling

The repository keeps correctness, coverage, fuzz, timing, and allocation
checks independently reproducible:

```sh
cargo test --all-targets --all-features
cargo test --all-targets --no-default-features
cargo bench --all-features --bench predicates
cargo run --release --all-features --example allocation_profile
scripts/representation_coverage.sh

rustup component add llvm-tools-preview
scripts/coverage.sh                 # also requires jq

cargo check --manifest-path fuzz/Cargo.toml --bins
cargo +nightly fuzz run predicate_invariants --fuzz-dir fuzz -- -max_total_time=30
cargo +nightly fuzz run hyperreal_representations --fuzz-dir fuzz -- -max_total_time=30
```

The coverage command instruments all feature-enabled unit and integration
tests and reports only `src/`. It prints a raw LLVM view (which necessarily
includes trailing inline `#[cfg(test)]` modules sharing those files) and a
production-only physical-line view, then writes browsable raw annotations to
`target/coverage/html/index.html`. The allocation example warms every query
before counting so fixture construction is excluded. Competitive benchmark
rows are intentionally labeled by representation: Hyperlimit accepts exact
`Real` inputs and returns policy/provenance-bearing outcomes, whereas the
binary64 comparison crates return determinant values or signs.

Coverage is measured on two axes. LLVM reports executable source coverage;
`tests/real_representations.rs` independently enforces scalar representation
coverage across all eight public Hyperreal structural kinds, all twenty
optimized class certificates, every rational storage class and primitive range
status, primitive-float imports, every optional primitive-cache feature
combination, cache and abort state, JSON/CBOR forms, and unresolved opaque
policy behavior. Serde drift probes additionally enumerate all 57 private
computable node tags and all 18 shared constants; every representative is
evaluated, round-tripped, embedded in an opaque `Real`, and crossed through
predicate families. A new finite representation cannot silently escape the
matrix. Opaque computable expression topology is unbounded, so variable-depth
shared graphs and metamorphic fuzzing cover composition rather than claiming a
finite list of every possible tree.

## Guarantees and boundaries

- A decided result comes from structural, filtered, exact, or bounded-refined
  evidence recorded in the outcome.
- Primitive floats are never an undocumented fallback for predicate decisions.
- An unresolved predicate remains `Unknown`.
- Retained facts and evidence can reduce repeated-query cost but do not change
  predicate semantics.
- Approximate metadata, including intentionally lossy DOP expansion adapters,
  is labeled and is not proof-producing.
- Hyperlimit owns predicates and small predicate-supporting constructions.
  Curves, rings as topology, triangulations, meshes, and CSG remain higher-layer
  responsibilities.

## Ecosystem and further documentation

- [Hyperreal](https://github.com/timschmidt/hyperreal) supplies exact-aware
  scalars, structural facts, and bounded refinement.
- [Hyperlattice](https://github.com/timschmidt/hyperlattice) owns points,
  homogeneous carriers, and linear algebra.
- [Hypercurve](https://github.com/timschmidt/hypercurve),
  [Hypertri](https://github.com/timschmidt/hypertri), and
  [Hypermesh](https://github.com/timschmidt/hypermesh) consume these predicates.

[`PERFORMANCE.md`](PERFORMANCE.md) records benchmark methodology and retained
optimization evidence. [`benchmarks.md`](benchmarks.md) contains generated
results. Generate complete type fields and signatures with `cargo doc --open`.

## References

- Guigue, Philippe, and Olivier Devillers. “Fast and Robust
  Triangle-Triangle Overlap Test Using Orientation Predicates.” *Journal of
  Graphics Tools*, vol. 8, no. 1, 2003, pp. 39–52.
  [doi:10.1080/10867651.2003.10487580](https://doi.org/10.1080/10867651.2003.10487580).
- Hormann, Kai, and Alexander Agathos. “The Point in Polygon Problem for
  Arbitrary Polygons.” *Computational Geometry*, vol. 20, no. 3, 2001,
  pp. 131–144.
  [doi:10.1016/S0925-7721(01)00012-8](https://doi.org/10.1016/S0925-7721(01)00012-8).
- Klosowski, James T., et al. “Efficient Collision Detection Using Bounding
  Volume Hierarchies of k-DOPs.” *IEEE Transactions on Visualization and
  Computer Graphics*, vol. 4, no. 1, 1998, pp. 21–36.
  [doi:10.1109/2945.675649](https://doi.org/10.1109/2945.675649).
- Moore, Ramon E. *Interval Analysis*. Prentice-Hall, 1966.
- Seidel, Raimund. “Small-Dimensional Linear Programming and Convex Hulls Made
  Easy.” *Discrete & Computational Geometry*, vol. 6, 1991, pp. 423–434.
  [doi:10.1007/BF02574699](https://doi.org/10.1007/BF02574699).
- Shewchuk, Jonathan Richard. “Adaptive Precision Floating-Point Arithmetic
  and Fast Robust Geometric Predicates.” *Discrete & Computational Geometry*,
  vol. 18, 1997, pp. 305–363.
  [doi:10.1007/PL00009321](https://doi.org/10.1007/PL00009321).
- Yap, Chee K. “Towards Exact Geometric Computation.” *Computational
  Geometry*, vol. 7, 1997, pp. 3–23.
  [doi:10.1016/0925-7721(95)00040-2](https://doi.org/10.1016/0925-7721(95)00040-2).

Shewchuk and Yap motivate exact escalation; Guigue and Devillers cover the
orientation-based triangle overlap route; Hormann and Agathos cover even-odd
point/ring classification; Klosowski et al. cover k-DOP bounds; Moore and
Seidel underpin certified bounds and small-dimensional feasibility.

## Acknowledgements

Hyperlimit is developed by Timothy Schmidt as the predicate layer of the Hyper
ecosystem. It builds on the exact-real work and contributors acknowledged by
Hyperreal and on Hyperlattice's object carriers.

## License and contributing

Hyperlimit is available under either the MIT License or the Apache License 2.0,
as declared in [`Cargo.toml`](Cargo.toml). The repository's [`LICENSE`](LICENSE)
contains the MIT terms.

Changes should preserve explicit uncertainty and keep topology ownership out of
the predicate layer. Before submitting a change, run:

```sh
cargo fmt --all -- --check
cargo test --all-targets --all-features
cargo test --all-targets --no-default-features
cargo clippy --all-targets --all-features -- -D warnings
RUSTDOCFLAGS="-D warnings" cargo doc --no-deps --all-features
```
