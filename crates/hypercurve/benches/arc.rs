use std::hint::black_box;
use std::time::Instant;

use hypercurve::{
    CircularArc2, Classification, Contour2, Curve2, CurveGeometry2, CurvePolicy, LineSeg2, Point2,
    Real, Segment2,
};

fn r(value: i32) -> Real {
    value.into()
}

fn p(x: i32, y: i32) -> Point2 {
    Point2::new(r(x), r(y))
}

fn q(numerator: i32, denominator: i32) -> Real {
    (r(numerator) / r(denominator)).expect("benchmark denominator is nonzero")
}

fn main() {
    let arc = CircularArc2::try_from_center(p(5, 0), p(0, 5), p(0, 0), true)
        .expect("benchmark arc is valid");
    let iterations = 20_000_u32;

    let started = Instant::now();
    let mut raw_checksum = 0_usize;
    for _ in 0..iterations {
        raw_checksum ^= black_box(
            arc.rational_bezier_decomposition()
                .expect("arc decomposition remains exact")
                .spans()
                .len(),
        );
    }
    let elapsed = started.elapsed();
    println!(
        "arc_cached_rational_decomposition: {iterations} iterations in {elapsed:?} ({:?}/iter), checksum={raw_checksum}",
        elapsed / iterations
    );

    let major_start = p(4, 0);
    let major_end = p(0, 4);
    let major = Contour2::try_new(vec![
        Segment2::Arc(
            CircularArc2::try_from_center(major_start.clone(), major_end.clone(), p(0, 0), true)
                .expect("major benchmark arc is valid"),
        ),
        Segment2::Line(
            LineSeg2::try_new(major_end, major_start).expect("major benchmark chord is valid"),
        ),
    ])
    .expect("major benchmark contour is valid");
    let policy = CurvePolicy::certified();
    let major_query = p(-1, 0);
    major.classify_point(&major_query, &policy);
    let started = Instant::now();
    for _ in 0..iterations {
        black_box(major.classify_point(black_box(&major_query), &policy));
    }
    let elapsed = started.elapsed();
    println!(
        "major_arc_cached_containment: {iterations} iterations in {elapsed:?} ({:?}/iter)",
        elapsed / iterations
    );

    let retained = Curve2::new(CurveGeometry2::CircularArc(arc));
    retained
        .native_bezier_fragments()
        .expect("initial arc promotion remains exact");
    let started = Instant::now();
    let mut retained_checksum = 0_usize;
    for _ in 0..iterations {
        retained_checksum ^= black_box(
            retained
                .native_bezier_fragments()
                .expect("retained arc promotion remains exact")
                .len(),
        );
    }
    let elapsed = started.elapsed();
    println!(
        "arc_cached_native_promotion: {iterations} iterations in {elapsed:?} ({:?}/iter), checksum={retained_checksum}",
        elapsed / iterations
    );

    let parameter = (r(1) / r(3)).expect("three is nonzero");
    let started = Instant::now();
    let mut evaluation_count = 0_u32;
    for _ in 0..iterations {
        let point = retained
            .point_at(&parameter)
            .expect("retained arc evaluation remains exact");
        black_box(point);
        evaluation_count += 1;
    }
    let elapsed = started.elapsed();
    println!(
        "arc_cached_top_level_evaluation: {iterations} iterations in {elapsed:?} ({:?}/iter), count={evaluation_count}",
        elapsed / iterations
    );

    let inverse_arc = CircularArc2::try_from_center(
        Point2::new(r(3), q(13, 3)),
        p(5, 3),
        Point2::new(r(3), q(13, 6)),
        false,
    )
    .expect("inverse-witness benchmark arc is valid");
    let retained_clone = inverse_arc.clone();
    let witness = p(3, 0);
    let Classification::Decided(parameter) = inverse_arc
        .sweep_fraction(&witness, &policy)
        .expect("inverse-witness parameterization remains exact")
    else {
        panic!("inverse-witness benchmark parameter must be decided");
    };
    let started = Instant::now();
    let mut witness_count = 0_u32;
    for _ in 0..iterations {
        let Classification::Decided(point) = retained_clone
            .point_at_sweep_fraction(black_box(&parameter), &policy)
            .expect("retained inverse witness remains exact")
        else {
            panic!("retained inverse witness replay must remain decided");
        };
        black_box(point);
        witness_count += 1;
    }
    let elapsed = started.elapsed();
    println!(
        "arc_retained_inverse_witness_replay: {iterations} iterations in {elapsed:?} ({:?}/iter), count={witness_count}",
        elapsed / iterations
    );
}
