//! One-pass dispatch reports for four public Hyperlattice API families.
//!
//! Cargo requires the hyperreal-dispatch-trace feature for this harness-free
//! target. It executes box construction, planar algebra, Cartesian point helpers
//! and projective incidence algebra once each, including fixture construction.
//! It uses neither Criterion nor a timer. The single per-workload assertion only
//! requires non-marker dispatch or rational temporaries; even construction work
//! can satisfy it. It does not assert exact counts, expected coordinates, every
//! requested path, certification receipts or complete public-API coverage.
//!
//! Recording is current-thread-only. Each case resets and drains that thread's
//! counters, but does not reset scalar caches or establish a cold process. The
//! fixed fixtures provide no independent work/memory limit or cancellation input.
//! Lossy point exports are explicitly exercised alongside exact construction;
//! label-based correlation categories do not certify numeric error or fidelity.
//!
//! On normal completion, main overwrites api_dispatch_trace.md relative to the
//! process working directory. Presence of HYPERLATTICE_SKIP_BENCHMARK_REPORTS,
//! even with an empty value, suppresses the write only. Arguments are not parsed:
//! --help, --list, --test and benchmark filters do not suppress the workloads or
//! report. This differs from the shared Criterion report callback used elsewhere.

use std::collections::BTreeMap;
use std::fmt::Write as _;
use std::fs;
use std::hint::black_box;

use hyperlattice::{
    Aabb, HomogeneousLine3, HomogeneousPoint3, Point2, Point3, ProjectivePlane3, Rational, Real,
};

/// Synthetic workload marker excluded from the nonempty-dispatch assertion and
/// Dependency Dispatch column, but retained in the report's detailed path table.
const MARKER_LAYER: &str = "hyperlattice-api-benchmark";

/// Construct an exact signed-integer Real through Rational, including its trace work.
fn r(value: i64) -> Real {
    Real::new(Rational::new(value))
}

/// Construct an exact rational Real for aggregation and retained-point fixtures.
///
/// Panics for a zero denominator; all callers here supply positive denominators.
/// Numerator/denominator reduction and scalar conversion are inside recording when
/// this helper is called by one of the workloads.
fn frac(numerator: i64, denominator: u64) -> Real {
    Rational::fraction(numerator, denominator)
        .expect("trace fraction denominator is nonzero")
        .into()
}

/// Run one closure in a fresh counter window and require evidence beyond its marker.
///
/// Clears existing current-thread dispatch/rational counters, records one named
/// marker, and invokes workload once under with_recording. The prior recording
/// flag is restored on leaving that scope. The returned value passes through
/// black_box and is dropped afterward, before take_trace drains both counter sets.
/// Local fixture work and destruction inside workload are within the scope; a
/// returned T's destruction is outside it. If recording was already active on
/// entry, however, the restored outer scope can also record that later destruction.
///
/// Panics unless the sum of counts outside MARKER_LAYER is positive or at least
/// one rational temporary was recorded. This weak check can pass on constructors
/// alone and neither validates T nor requires each intended operation to dispatch.
/// Panics from workload propagate without producing a snapshot; the recording
/// guard restores the flag, but this helper does not drain counters on that path.
/// Reset/drain makes this unsuitable for preserving a caller's enclosing trace.
/// Child-thread work is not merged, scalar caches are not cleared, and no budget
/// is applied to the workload or the ordinary u64 count sum.
fn trace_case<T>(
    name: &'static str,
    workload: impl FnOnce() -> T,
) -> hyperreal::dispatch_trace::TraceSnapshot {
    hyperreal::dispatch_trace::reset();
    let result = hyperreal::dispatch_trace::with_recording(|| {
        hyperreal::dispatch_trace::record(MARKER_LAYER, name, "recorded-workload");
        workload()
    });
    black_box(result);

    let snapshot = hyperreal::dispatch_trace::take_trace();
    let dependency_dispatch = snapshot
        .dispatch
        .iter()
        .filter(|entry| entry.layer != MARKER_LAYER)
        .map(|entry| entry.count)
        .sum::<u64>();
    assert!(
        dependency_dispatch > 0 || snapshot.rational.temporary_rationals > 0,
        "{name} recorded no lattice/dependency dispatch or rational work"
    );
    snapshot
}

/// Trace planar expression construction and fact helpers on a 3-4 right triangle.
///
/// Uses a=(0,0), b=(3,0), c=(0,4). Displacement a-to-b is (3,0); the signed product
/// sum, wedge and orientation expressions are 12; positive product sum and dot are
/// zero; b's squared norm is 9 and squared distance b-to-c is 25. These mathematical
/// expectations explain the fixture and are not asserted by this workload.
///
/// Also requests displacement, product-term, product-sum and orientation facts.
/// All results are discarded through black_box, with no sign decision or policy
/// receipt. Sparse zero coordinates and integer inputs cover a narrow path set.
fn trace_algebra2() -> hyperreal::dispatch_trace::TraceSnapshot {
    trace_case("algebra2_exact_helpers_and_facts", || {
        let a = [r(0), r(0)];
        let b = [r(3), r(0)];
        let c = [r(0), r(4)];
        let ab = [&a[0], &a[1]];
        let bb = [&b[0], &b[1]];
        let cb = [&c[0], &c[1]];

        let displacement = hyperlattice::displacement2(ab, bb);
        black_box(hyperlattice::displacement2_facts(ab, bb));
        black_box(hyperlattice::product_term2_facts([
            &displacement[0],
            &displacement[1],
        ]));
        black_box(hyperlattice::product_sum2_facts([
            [&b[0], &c[1]],
            [&b[1], &c[0]],
        ]));
        black_box(hyperlattice::orient2_expr_facts(ab, bb, cb));
        black_box(hyperlattice::signed_product_sum2(
            [true, false],
            [[&b[0], &c[1]], [&b[1], &c[0]]],
        ));
        black_box(hyperlattice::positive_product_sum2([
            [&b[0], &c[0]],
            [&b[1], &c[1]],
        ]));
        black_box(hyperlattice::wedge2(bb, cb));
        black_box(hyperlattice::dot2(bb, cb));
        black_box(hyperlattice::squared_norm2(bb));
        black_box(hyperlattice::squared_distance2(bb, cb));
        black_box(hyperlattice::orient2_expr(ab, bb, cb));
    })
}

/// Trace exact point ingress, explicit lossy exports, aggregation and subtraction.
///
/// Builds 2D points (1,2)/(3,4) and 3D points (1,2,3)/(4,5,6) from finite f64/f32
/// literals, unwrapping ingress. These integers are exactly representable; the
/// general conversion contract retains each finite float's exact binary value.
/// Exercises borrowed/owned vector conversion, half-way interpolation, centroid,
/// weighted_sum with weights (1,2), shared-scale views and structural facts.
/// Weighted sums are unnormalized: expected coordinates are (7,10) and (9,12,15),
/// not division by total weight three. Options and exported coordinates are not
/// checked, so the trace assertion does not establish successful aggregation.
///
/// Finally constructs two rational 3D points with differing denominators and
/// repeats their borrowed subtraction three times on the same retained operands.
/// Counts aggregate all three calls; there are no separate cold/warm snapshots or
/// equality assertions. Construction, cloning and local destruction are recorded.
fn trace_points() -> hyperreal::dispatch_trace::TraceSnapshot {
    trace_case("point_construction_facts_and_aggregation", || {
        let p2 = Point2::try_from_f64_array([1.0, 2.0]).expect("finite point2 input");
        let q2 = Point2::try_from_f32_array([3.0, 4.0]).expect("finite point2 input");
        black_box(Point2::to_f64_array_lossy(&p2));
        black_box(Point2::to_f32_array_lossy(&q2));
        black_box(Point2::to_vector(&p2));
        black_box(Point2::into_vector(q2.clone()));
        black_box(Point2::lerp(&p2, &q2, &frac(1, 2)));
        black_box(Point2::centroid(&[p2.clone(), q2.clone()]));
        black_box(Point2::weighted_sum(
            &[p2.clone(), q2.clone()],
            &[r(1), r(2)],
        ));
        black_box(Point2::shared_scale_view(&p2));
        black_box(Point2::structural_facts(&p2));

        let p3 = Point3::try_from_f64_array([1.0, 2.0, 3.0]).expect("finite point3 input");
        let q3 = Point3::try_from_f32_array([4.0, 5.0, 6.0]).expect("finite point3 input");
        black_box(Point3::to_f64_array_lossy(&p3));
        black_box(Point3::to_f32_array_lossy(&q3));
        black_box(Point3::to_vector(&p3));
        black_box(Point3::into_vector(q3.clone()));
        black_box(Point3::lerp(&p3, &q3, &frac(1, 2)));
        black_box(Point3::centroid(&[p3.clone(), q3.clone()]));
        black_box(Point3::weighted_sum(
            &[p3.clone(), q3.clone()],
            &[r(1), r(2)],
        ));
        black_box(Point3::shared_scale_view(&p3));
        black_box(Point3::structural_facts(&p3));

        let retained_left = Point3::new(frac(5, 7), frac(-11, 13), frac(17, 19));
        let retained_right = Point3::new(frac(-23, 29), frac(31, 37), frac(-41, 43));
        for _ in 0..3 {
            black_box(&retained_left - &retained_right);
        }
    })
}

/// Trace homogeneous intersection algebra for x=2, y=3 and z=4 planes.
///
/// Three-plane cofactors give (2,3,4,1); affine conversion is explicitly expected
/// to succeed. Requests coordinates/facts and incidence with x=2. The x/y plane
/// intersection has direction (0,0,1) and moment (3,-2,0); its z=4 intersection is
/// constructed both through the method and free-function entry points. Incidence
/// is likewise exercised through both entry points. No equality, zero-expression,
/// rank or path-count assertions compare these results; all are black-boxed.
/// This orthogonal finite fixture does not cover dependent planes or infinity.
fn trace_projective() -> hyperreal::dispatch_trace::TraceSnapshot {
    trace_case("projective_plane_line_and_point_algebra", || {
        let px = ProjectivePlane3::new(Point3::new(r(1), r(0), r(0)), r(-2));
        let py = ProjectivePlane3::new(Point3::new(r(0), r(1), r(0)), r(-3));
        let pz = ProjectivePlane3::new(Point3::new(r(0), r(0), r(1)), r(-4));

        let point = hyperlattice::intersect_three_planes(&px, &py, &pz);
        black_box(HomogeneousPoint3::coordinates(&point));
        black_box(HomogeneousPoint3::coordinate_facts(&point));
        black_box(
            HomogeneousPoint3::to_affine_point(&point)
                .expect("orthogonal trace planes meet at a finite point"),
        );
        black_box(HomogeneousPoint3::plane_expression(&point, &px));

        let line = hyperlattice::intersect_two_planes(&px, &py);
        black_box(HomogeneousLine3::coordinate_facts(&line));
        black_box(HomogeneousLine3::intersect_plane(&line, &pz));
        black_box(hyperlattice::intersect_homogeneous_line_plane(&line, &pz));
        black_box(hyperlattice::homogeneous_point_plane_expression(
            &point, &px,
        ));
    })
}

/// Trace degenerate origin bounds and a box from (-1,-2,-3) to (1,2,3).
///
/// Both constructors merely retain corners; origin is not a canonical empty-set
/// sentinel. No ordering, containment or set-validity predicate runs. Scalar/point
/// construction can satisfy the trace check even without a distinct Aabb event.
fn trace_aabb() -> hyperreal::dispatch_trace::TraceSnapshot {
    trace_case("aabb_construction", || {
        black_box(Aabb::origin());
        black_box(Aabb::new(
            Point3::new(r(-1), r(-2), r(-3)),
            Point3::new(r(1), r(2), r(3)),
        ));
    })
}

/// Render supplied snapshots and overwrite a working-directory Markdown report.
///
/// Returns immediately if HYPERLATTICE_SKIP_BENCHMARK_REPORTS exists. Otherwise
/// emits rows in BTreeMap key order. Dependency Dispatch excludes the marker layer;
/// the selected correlation columns use Hyperreal's label heuristics, while rational
/// columns use recorded counters. These categories can overlap and are neither a
/// cost partition nor correctness certificates. Unlike Dependency Dispatch, the
/// correlation classifier includes the marker: its layer counts as linear algebra,
/// and its workload name can also match other categories. The detailed table retains
/// every supplied dispatch entry, including markers, in snapshot order. Snapshots taken
/// by trace_case are sorted by layer/operation/path by the recorder.
///
/// The output contains no timestamp, revision, actual build feature configuration, numerical
/// outcomes or timing samples. Labels are inserted without Markdown escaping; the
/// current fixed workload/dispatch labels are controlled by the implementation.
/// No directory creation, append/merge, atomic replacement or locking is performed.
/// File errors panic; writing to the String is expected to be infallible apart
/// from allocation failure. The skip gate does not suppress prior workload execution.
fn write_report(rows: &BTreeMap<&'static str, hyperreal::dispatch_trace::TraceSnapshot>) {
    if std::env::var_os("HYPERLATTICE_SKIP_BENCHMARK_REPORTS").is_some() {
        return;
    }

    let mut out = String::from(
        "# Hyperlattice Public-Family Dispatch Trace\n\n\
Generated by `cargo bench --bench api_dispatch_trace --features hyperreal-dispatch-trace`. Each workload runs once outside Criterion and must record lattice/dependency dispatch or rational work. Use `mathbench` for timings.\n\n\
## Correlation Summary\n\n\
| Workload | Dependency Dispatch | Linear Algebra | Exact Reducers | Approximation | Rational Temporaries | Rational Reductions | Rational GCDs |\n\
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |\n",
    );
    for (name, trace) in rows {
        let summary = trace.correlation_summary();
        let dependency_dispatch = trace
            .dispatch
            .iter()
            .filter(|entry| entry.layer != MARKER_LAYER)
            .map(|entry| entry.count)
            .sum::<u64>();
        writeln!(
            out,
            "| `{name}` | {dependency_dispatch} | {} | {} | {} | {} | {} | {} |",
            summary.linear_algebra_events,
            summary.exact_reducer_events,
            summary.approximation_events,
            trace.rational.temporary_rationals,
            trace.rational.reductions,
            trace.rational.gcds,
        )
        .expect("writing to String cannot fail");
    }

    out.push_str(
        "\n## Dispatch Paths\n\n\
| Workload | Layer | Operation | Path | Count |\n\
| --- | --- | --- | --- | ---: |\n",
    );
    for (name, trace) in rows {
        for entry in &trace.dispatch {
            writeln!(
                out,
                "| `{name}` | `{}` | `{}` | `{}` | {} |",
                entry.layer, entry.operation, entry.path, entry.count,
            )
            .expect("writing to String cannot fail");
        }
    }
    fs::write("api_dispatch_trace.md", out).expect("write API dispatch trace report");
}

/// Execute all four trace families in array order, then write their sorted report.
///
/// Runs Aabb, planar algebra, points and projective algebra exactly once each on
/// the calling thread. Any workload/assertion panic prevents report creation;
/// skipping reports still runs every workload. Does not inspect command-line flags,
/// print timing results, or start Criterion. BTreeMap keys determine output order.
fn main() {
    let rows = BTreeMap::from([
        ("aabb_construction", trace_aabb()),
        ("algebra2_exact_helpers_and_facts", trace_algebra2()),
        ("point_construction_facts_and_aggregation", trace_points()),
        (
            "projective_plane_line_and_point_algebra",
            trace_projective(),
        ),
    ]);
    write_report(&rows);
}
