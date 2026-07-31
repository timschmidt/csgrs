use std::hint::black_box;
use std::time::Instant;

use hypercurve::{
    CircularArc2, Classification, Contour2, Curve2, CurveContext, CurveGeometry2, CurvePath2,
    CurveRegion2, LineSeg2, Point2, Real, Segment2,
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

fn large_arc_count() -> usize {
    std::env::var("HYPERCURVE_BENCH_ARC_COUNT")
        .ok()
        .and_then(|value| value.parse().ok())
        .unwrap_or(256)
        .clamp(1, i32::MAX as usize / 2)
}

fn large_arc_iterations() -> u32 {
    std::env::var("HYPERCURVE_BENCH_ARC_ITERATIONS")
        .ok()
        .and_then(|value| value.parse().ok())
        .unwrap_or(10)
        .max(1)
}

fn semicircle(start_x: i32, end_x: i32, y: i32, clockwise: bool) -> Curve2 {
    Curve2::from(
        CircularArc2::try_from_center(
            p(start_x, y),
            p(end_x, y),
            p((start_x + end_x) / 2, y),
            clockwise,
        )
        .expect("benchmark semicircle is valid"),
    )
}

fn large_arc_chain(arc_count: usize, y: i32) -> CurvePath2 {
    CurvePath2::try_new(
        (0..arc_count)
            .map(|index| {
                let start_x = i32::try_from(index * 2).unwrap();
                semicircle(start_x, start_x + 2, y, index.is_multiple_of(2))
            })
            .collect(),
    )
    .expect("benchmark arc chain is connected")
}

fn large_arc_region(arc_count: usize) -> CurveRegion2 {
    let mut curves = (0..arc_count)
        .map(|index| {
            let start_x = i32::try_from(index * 2).unwrap();
            semicircle(start_x, start_x + 2, 0, true)
        })
        .collect::<Vec<_>>();
    curves.extend((0..arc_count).rev().map(|index| {
        let start_x = i32::try_from(index * 2).unwrap();
        semicircle(start_x + 2, start_x, 0, true)
    }));
    let path = CurvePath2::try_new(curves).expect("benchmark arc boundary is connected");
    CurveRegion2::try_from_boundary_paths(&[path], &CurveContext::STRICT)
        .expect("benchmark arc region is closed")
        .into_value()
}

fn bench_large_arcs() {
    let arc_count = large_arc_count();
    let iterations = large_arc_iterations();
    let policy = CurveContext::STRICT;
    let first = large_arc_chain(arc_count, 0);
    let second = large_arc_chain(arc_count, 10_000);

    let started = Instant::now();
    let mut cold_checksum = 0_usize;
    for _ in 0..iterations {
        let path = large_arc_chain(arc_count, 0);
        cold_checksum ^= black_box(path.native_bezier_fragments().unwrap().len());
    }
    let elapsed = started.elapsed();
    println!(
        "arc_large_cold_native_promotion_{arc_count}_arcs: {iterations} iterations in {elapsed:?} ({:?}/iter), checksum={cold_checksum}",
        elapsed / iterations
    );

    first.native_bezier_fragments().unwrap();
    let started = Instant::now();
    let mut cached_checksum = 0_usize;
    for _ in 0..iterations {
        cached_checksum ^= black_box(first.native_bezier_fragments().unwrap().len());
    }
    let elapsed = started.elapsed();
    println!(
        "arc_large_cached_native_promotion_{arc_count}_arcs: {iterations} iterations in {elapsed:?} ({:?}/iter), checksum={cached_checksum}",
        elapsed / iterations
    );

    let started = Instant::now();
    let mut intersection_checksum = 0_usize;
    for _ in 0..iterations {
        let evidence = first.intersect_path(&second, &policy).unwrap();
        intersection_checksum ^= black_box(
            evidence.candidate_curve_pair_count()
                + evidence.contacts().len()
                + evidence.overlaps().len(),
        );
    }
    let elapsed = started.elapsed();
    println!(
        "arc_large_sparse_path_intersection_{arc_count}_arcs: {iterations} iterations in {elapsed:?} ({:?}/iter), checksum={intersection_checksum}",
        elapsed / iterations
    );

    let region = large_arc_region(arc_count);
    let query = Point2::new(r(1), q(1, 2));
    let warm_location = region.classify_point(&query, &policy).unwrap();
    let started = Instant::now();
    let mut containment_checksum = 0_usize;
    for _ in 0..iterations {
        containment_checksum ^= black_box(
            region
                .classify_point(&query, &policy)
                .unwrap()
                .into_value()
                .is_decided() as usize,
        );
    }
    let elapsed = started.elapsed();
    println!(
        "arc_large_region_containment_{arc_count}_arcs: {iterations} iterations in {elapsed:?} ({:?}/iter), warm={warm_location:?}, checksum={containment_checksum}",
        elapsed / iterations
    );
}

fn main() {
    if std::env::var_os("HYPERCURVE_BENCH_ARC_ONLY").is_some() {
        bench_large_arcs();
        return;
    }

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

    let started = Instant::now();
    let mut facts_checksum = 0_usize;
    for _ in 0..iterations {
        let facts = black_box(&arc).structural_facts();
        facts_checksum ^= facts.scalar_exact.exact_rational_count;
        facts_checksum ^= facts.scalar_exact.exact_power_of_two_count.rotate_left(7);
    }
    let elapsed = started.elapsed();
    println!(
        "arc_structural_facts_replay: {iterations} iterations in {elapsed:?} ({:?}/iter), checksum={facts_checksum}",
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
    let policy = CurveContext::STRICT;
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

    bench_large_arcs();
}
