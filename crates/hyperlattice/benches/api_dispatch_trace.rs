use std::collections::BTreeMap;
use std::fmt::Write as _;
use std::fs;
use std::hint::black_box;

use hyperlattice::{
    Aabb, HomogeneousLine3, HomogeneousPoint3, Point2, Point3, ProjectivePlane3, Rational, Real,
};

const MARKER_LAYER: &str = "hyperlattice-api-benchmark";

fn r(value: i64) -> Real {
    Real::new(Rational::new(value))
}

fn frac(numerator: i64, denominator: u64) -> Real {
    Rational::fraction(numerator, denominator)
        .expect("trace fraction denominator is nonzero")
        .into()
}

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
        black_box(Point3::weighted_sum(&[p3.clone(), q3], &[r(1), r(2)]));
        black_box(Point3::shared_scale_view(&p3));
        black_box(Point3::structural_facts(&p3));
    })
}

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

fn trace_aabb() -> hyperreal::dispatch_trace::TraceSnapshot {
    trace_case("aabb_construction", || {
        black_box(Aabb::origin());
        black_box(Aabb::new(
            Point3::new(r(-1), r(-2), r(-3)),
            Point3::new(r(1), r(2), r(3)),
        ));
    })
}

fn write_report(rows: &BTreeMap<&'static str, hyperreal::dispatch_trace::TraceSnapshot>) {
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
