use std::hint::black_box;
use std::time::Instant;

use hypercurve::{
    BezierFlatteningOptions, BezierParallelVerificationOptions, BulgeVertex2, CircularArc2,
    Classification, Contour2, CubicBezier2, Curve2, CurvePath2, CurvePolicy, CurveRegion2,
    CurveRegionLoopRole, CurveResult, CurveString2, FillRule, LineSeg2, OffsetCap, Point2,
    QuadraticBezier2, Real, Segment2,
};

fn s(value: i32) -> Real {
    value.into()
}

fn p(x: i32, y: i32) -> Point2 {
    Point2::new(s(x), s(y))
}

fn q(numerator: i32, denominator: i32) -> Real {
    (s(numerator) / s(denominator)).expect("nonzero benchmark denominator")
}

fn vertex(x: i32, y: i32, bulge: i32) -> BulgeVertex2 {
    BulgeVertex2::new(p(x, y), s(bulge))
}

fn line_segment(start_x: i32, start_y: i32, end_x: i32, end_y: i32) -> Segment2 {
    Segment2::Line(LineSeg2::try_new(p(start_x, start_y), p(end_x, end_y)).unwrap())
}

fn bench_line_offset(iterations: u32) -> CurveResult<()> {
    let line = LineSeg2::try_new(p(0, 0), p(3, 4))?;
    let started = Instant::now();
    let mut checksum = 0_usize;

    for _ in 0..iterations {
        let offset = line.offset_left(s(5))?;
        checksum += black_box(offset.start().x().to_f64_lossy().is_some() as usize);
    }

    let elapsed = started.elapsed();
    println!(
        "line_offset_left: {iterations} iterations in {elapsed:?} ({:?}/iter), checksum={checksum}",
        elapsed / iterations
    );
    Ok(())
}

fn bench_arc_offset(name: &str, segment: &Segment2, iterations: u32) -> CurveResult<()> {
    let policy = CurvePolicy::STRICT;
    let started = Instant::now();
    let mut checksum = 0_usize;

    for _ in 0..iterations {
        let Classification::Decided(offset) = segment.offset_left(s(1), &policy)? else {
            panic!("{name} became uncertain during benchmark");
        };
        checksum += black_box(offset.end().y().to_f64_lossy().is_some() as usize);
    }

    let elapsed = started.elapsed();
    println!(
        "{name}: {iterations} iterations in {elapsed:?} ({:?}/iter), checksum={checksum}",
        elapsed / iterations
    );
    Ok(())
}

fn bench_curve_string_joined_offset(iterations: u32) -> CurveResult<()> {
    let curve = CurveString2::try_new(vec![
        line_segment(0, 0, 4, 0),
        line_segment(4, 0, 4, 3),
        line_segment(4, 3, 7, 3),
    ])?;
    let policy = CurvePolicy::STRICT;
    let started = Instant::now();
    let mut total_segments = 0_usize;

    for _ in 0..iterations {
        let Classification::Decided(offset) = curve.offset_left_with_line_joins(s(1), &policy)?
        else {
            panic!("curve_string_joined_offset became uncertain during benchmark");
        };
        total_segments += black_box(offset.len());
    }

    let elapsed = started.elapsed();
    println!(
        "curve_string_joined_offset: {iterations} iterations in {elapsed:?} ({:?}/iter), total segments={total_segments}",
        elapsed / iterations
    );
    Ok(())
}

fn bench_curve_string_round_join_offset(iterations: u32) -> CurveResult<()> {
    let curve = CurveString2::try_new(vec![
        Segment2::Arc(CircularArc2::from_bulge(p(0, 0), p(2, 0), s(-1))?),
        line_segment(2, 0, 4, 0),
        line_segment(4, 0, 4, 3),
    ])?;
    let policy = CurvePolicy::STRICT;
    let started = Instant::now();
    let mut total_segments = 0_usize;

    for _ in 0..iterations {
        let Classification::Decided(offset) = curve.offset_left_with_line_joins(s(1), &policy)?
        else {
            panic!("curve_string_round_join_offset became uncertain during benchmark");
        };
        total_segments += black_box(offset.len());
    }

    let elapsed = started.elapsed();
    println!(
        "curve_string_round_join_offset: {iterations} iterations in {elapsed:?} ({:?}/iter), total segments={total_segments}",
        elapsed / iterations
    );
    Ok(())
}

fn bench_curve_string_checked_offset(iterations: u32) -> CurveResult<()> {
    let curve = CurveString2::try_new(vec![
        line_segment(0, 0, 4, 0),
        line_segment(4, 0, 4, 3),
        line_segment(4, 3, 7, 3),
    ])?;
    let policy = CurvePolicy::STRICT;
    let started = Instant::now();
    let mut total_segments = 0_usize;

    for _ in 0..iterations {
        let Classification::Decided(offset) = curve.offset_left_checked(s(1), &policy)? else {
            panic!("curve_string_checked_offset became uncertain during benchmark");
        };
        total_segments += black_box(offset.len());
    }

    let elapsed = started.elapsed();
    println!(
        "curve_string_checked_offset: {iterations} iterations in {elapsed:?} ({:?}/iter), total segments={total_segments}",
        elapsed / iterations
    );
    Ok(())
}

fn bench_curve_string_checked_offset_evidence(iterations: u32) -> CurveResult<()> {
    let curve = CurveString2::try_new(vec![
        line_segment(0, 0, 4, 0),
        line_segment(4, 0, 4, 3),
        line_segment(4, 3, 7, 3),
    ])?;
    let policy = CurvePolicy::STRICT;
    let started = Instant::now();
    let mut total_segments = 0_usize;

    for _ in 0..iterations {
        let Classification::Decided(offset) = curve.offset_left_checked(s(1), &policy)? else {
            panic!("curve_string_checked_offset benchmark became uncertain");
        };
        total_segments += black_box(offset.len());
    }

    let elapsed = started.elapsed();
    println!(
        "curve_string_checked_offset_evidence: {iterations} iterations in {elapsed:?} ({:?}/iter), total segments={total_segments}",
        elapsed / iterations
    );
    Ok(())
}

fn bench_curve_string_round_cap_outline(iterations: u32) -> CurveResult<()> {
    let curve = CurveString2::try_new(vec![
        line_segment(0, 0, 4, 0),
        line_segment(4, 0, 4, 3),
        line_segment(4, 3, 7, 3),
    ])?;
    let policy = CurvePolicy::STRICT;
    let started = Instant::now();
    let mut total_segments = 0_usize;

    for _ in 0..iterations {
        let Classification::Decided(outline) = curve.offset_outline_round_caps(s(1), &policy)?
        else {
            panic!("curve_string_round_cap_outline became uncertain during benchmark");
        };
        total_segments += black_box(outline.len());
    }

    let elapsed = started.elapsed();
    println!(
        "curve_string_round_cap_outline: {iterations} iterations in {elapsed:?} ({:?}/iter), total segments={total_segments}",
        elapsed / iterations
    );
    Ok(())
}

fn bench_curve_string_round_cap_outline_evidence(iterations: u32) -> CurveResult<()> {
    let curve = CurveString2::try_new(vec![
        line_segment(0, 0, 4, 0),
        line_segment(4, 0, 4, 3),
        line_segment(4, 3, 7, 3),
    ])?;
    let policy = CurvePolicy::STRICT;
    let started = Instant::now();
    let mut total_segments = 0_usize;

    for _ in 0..iterations {
        let Classification::Decided(outline) = curve.offset_outline_round_caps(s(1), &policy)?
        else {
            panic!("curve_string_round_cap_outline benchmark became uncertain");
        };
        total_segments += black_box(outline.len());
    }

    let elapsed = started.elapsed();
    println!(
        "curve_string_round_cap_outline_evidence: {iterations} iterations in {elapsed:?} ({:?}/iter), total segments={total_segments}",
        elapsed / iterations
    );
    Ok(())
}

fn bench_curve_string_butt_cap_outline(iterations: u32) -> CurveResult<()> {
    let curve = CurveString2::try_new(vec![
        line_segment(0, 0, 4, 0),
        line_segment(4, 0, 4, 3),
        line_segment(4, 3, 7, 3),
    ])?;
    let policy = CurvePolicy::STRICT;
    let started = Instant::now();
    let mut total_segments = 0_usize;

    for _ in 0..iterations {
        let Classification::Decided(outline) = curve.offset_outline_butt_caps(s(1), &policy)?
        else {
            panic!("curve_string_butt_cap_outline became uncertain during benchmark");
        };
        total_segments += black_box(outline.len());
    }

    let elapsed = started.elapsed();
    println!(
        "curve_string_butt_cap_outline: {iterations} iterations in {elapsed:?} ({:?}/iter), total segments={total_segments}",
        elapsed / iterations
    );
    Ok(())
}

fn bench_curve_string_square_cap_outline(iterations: u32) -> CurveResult<()> {
    let curve = CurveString2::try_new(vec![
        line_segment(0, 0, 4, 0),
        line_segment(4, 0, 4, 3),
        line_segment(4, 3, 7, 3),
    ])?;
    let policy = CurvePolicy::STRICT;
    let started = Instant::now();
    let mut total_segments = 0_usize;

    for _ in 0..iterations {
        let Classification::Decided(outline) =
            curve.offset_outline(s(1), OffsetCap::Square, &policy)?
        else {
            panic!("curve_string_square_cap_outline became uncertain during benchmark");
        };
        total_segments += black_box(outline.len());
    }

    let elapsed = started.elapsed();
    println!(
        "curve_string_square_cap_outline: {iterations} iterations in {elapsed:?} ({:?}/iter), total segments={total_segments}",
        elapsed / iterations
    );
    Ok(())
}

fn bench_contour_joined_offset(iterations: u32) -> CurveResult<()> {
    let contour = Contour2::from_bulge_vertices(&[
        vertex(0, 0, 0),
        vertex(10, 0, 0),
        vertex(10, 7, 0),
        vertex(0, 7, 0),
    ])?;
    let policy = CurvePolicy::STRICT;
    let started = Instant::now();
    let mut total_segments = 0_usize;

    for _ in 0..iterations {
        let Classification::Decided(offset) = contour.offset_left_with_line_joins(s(1), &policy)?
        else {
            panic!("contour_joined_offset became uncertain during benchmark");
        };
        total_segments += black_box(offset.len());
    }

    let elapsed = started.elapsed();
    println!(
        "contour_joined_offset: {iterations} iterations in {elapsed:?} ({:?}/iter), total segments={total_segments}",
        elapsed / iterations
    );
    Ok(())
}

fn bench_contour_checked_offset(iterations: u32) -> CurveResult<()> {
    let contour = Contour2::from_bulge_vertices(&[
        vertex(0, 0, 0),
        vertex(10, 0, 0),
        vertex(10, 7, 0),
        vertex(0, 7, 0),
    ])?;
    let policy = CurvePolicy::STRICT;
    let started = Instant::now();
    let mut total_segments = 0_usize;

    for _ in 0..iterations {
        let Classification::Decided(offset) = contour.offset_left_checked(s(1), &policy)? else {
            panic!("contour_checked_offset became uncertain during benchmark");
        };
        total_segments += black_box(offset.len());
    }

    let elapsed = started.elapsed();
    println!(
        "contour_checked_offset: {iterations} iterations in {elapsed:?} ({:?}/iter), total segments={total_segments}",
        elapsed / iterations
    );
    Ok(())
}

fn bench_contour_checked_offset_evidence(iterations: u32) -> CurveResult<()> {
    let contour = Contour2::from_bulge_vertices(&[
        vertex(0, 0, 0),
        vertex(10, 0, 0),
        vertex(10, 7, 0),
        vertex(0, 7, 0),
    ])?;
    let policy = CurvePolicy::STRICT;
    let started = Instant::now();
    let mut total_segments = 0_usize;

    for _ in 0..iterations {
        let Classification::Decided(offset) = contour.offset_left_checked(s(1), &policy)? else {
            panic!("contour_checked_offset benchmark became uncertain");
        };
        total_segments += black_box(offset.len());
    }

    let elapsed = started.elapsed();
    println!(
        "contour_checked_offset_evidence: {iterations} iterations in {elapsed:?} ({:?}/iter), total segments={total_segments}",
        elapsed / iterations
    );
    Ok(())
}

fn bench_exact_bezier_parallel_evaluation(iterations: u32) -> CurveResult<()> {
    let source = CubicBezier2::new(p(0, 0), p(1, 2), p(3, -1), p(4, 0));
    let parallel = source.parallel_left(q(1, 10))?;
    let policy = CurvePolicy::STRICT;
    let parameter = q(7, 13);
    let started = Instant::now();
    let mut checksum = 0_usize;
    for _ in 0..iterations {
        let Classification::Decided(point) = parallel.point_at(&parameter, &policy)? else {
            panic!("exact Bezier parallel evaluation became uncertain");
        };
        checksum += black_box(point.x().to_f64_lossy().is_some() as usize);
    }
    let elapsed = started.elapsed();
    println!(
        "bezier_parallel_exact_eval: {iterations} iterations in {elapsed:?} ({:?}/iter), checksum={checksum}",
        elapsed / iterations
    );
    Ok(())
}

fn bench_bezier_parallel_cusp_isolation(iterations: u32) -> CurveResult<()> {
    let source = QuadraticBezier2::new(p(0, 0), Point2::new(q(1, 2), s(0)), p(1, 1));
    let parallel = source.parallel_left(s(1))?;
    let policy = CurvePolicy::STRICT;
    let started = Instant::now();
    let mut roots = 0_usize;
    for _ in 0..iterations {
        let Classification::Decided(analysis) = parallel.singularity_analysis(&policy)? else {
            panic!("Bezier parallel cusp isolation became uncertain");
        };
        roots += black_box(analysis.parallel_cusps().len());
    }
    let elapsed = started.elapsed();
    println!(
        "bezier_parallel_cusp_isolation: {iterations} iterations in {elapsed:?} ({:?}/iter), roots={roots}",
        elapsed / iterations
    );
    Ok(())
}

fn bench_exact_ph_offset_construction(iterations: u32) -> CurveResult<()> {
    let source = CubicBezier2::new(
        p(0, 0),
        Point2::new(q(1, 3), s(0)),
        Point2::new(q(2, 3), q(1, 3)),
        Point2::new(q(2, 3), s(1)),
    );
    let parallel = source.parallel_left(q(1, 5))?;
    let policy = CurvePolicy::STRICT;
    let started = Instant::now();
    let mut degree = 0_usize;
    for _ in 0..iterations {
        let Classification::Decided(Some(offset)) =
            parallel.exact_pythagorean_hodograph_offset(&policy)?
        else {
            panic!("PH benchmark source was not recognized");
        };
        degree += black_box(offset.rational_degree());
    }
    let elapsed = started.elapsed();
    println!(
        "bezier_parallel_exact_ph: {iterations} iterations in {elapsed:?} ({:?}/iter), degree checksum={degree}",
        elapsed / iterations
    );
    Ok(())
}

fn bench_certified_bezier_parallel_construction(iterations: u32) -> CurveResult<()> {
    let source = CubicBezier2::new(p(0, 0), p(1, 2), p(2, -1), p(4, 0));
    let policy = CurvePolicy::STRICT;
    let options = BezierParallelVerificationOptions::try_new(q(1, 20), 14, &policy)?;
    let started = Instant::now();
    let mut spans = 0_usize;
    let mut leaves = 0_usize;
    for _ in 0..iterations {
        let Classification::Decided(path) =
            source.approximate_parallel_blend2d_certified(q(1, 10), &options, &policy)?
        else {
            panic!("certified Bezier parallel construction became uncertain");
        };
        spans += black_box(path.spans().len());
        leaves += black_box(path.verification_leaf_count());
    }
    let elapsed = started.elapsed();
    println!(
        "bezier_parallel_certified_blend2d_levien: {iterations} iterations in {elapsed:?} ({:?}/iter), spans={spans}, verifier leaves={leaves}",
        elapsed / iterations
    );
    Ok(())
}

fn bench_curve_region_bezier_offset_lanes(
    iterations: u32,
) -> Result<(), Box<dyn std::error::Error>> {
    let source_path = CurvePath2::try_new(vec![
        Curve2::from(QuadraticBezier2::new(p(1, 0), p(1, 1), p(0, 1))),
        Curve2::from(QuadraticBezier2::new(p(0, 1), p(-1, 1), p(-1, 0))),
        Curve2::from(QuadraticBezier2::new(p(-1, 0), p(-1, -1), p(0, -1))),
        Curve2::from(QuadraticBezier2::new(p(0, -1), p(1, -1), p(1, 0))),
    ])?;
    let source = CurveRegion2::try_from_boundary_paths_with_loop_semantics(
        &[source_path],
        &[CurveRegionLoopRole::Material],
        &[FillRule::EvenOdd],
        &CurvePolicy::STRICT,
    )?;
    let policy = CurvePolicy::STRICT;
    let verification = BezierParallelVerificationOptions::try_new(q(1, 20), 16, &policy)?;
    let flattening = BezierFlatteningOptions::try_new(q(1, 20), 16, &policy)?;

    let started = Instant::now();
    let mut certified_loops = 0_usize;
    for _ in 0..iterations {
        let Classification::Decided(result) = source.offset_with_certified_bezier_parallel(
            q(1, 10),
            &verification,
            &flattening,
            &flattening,
            &policy,
        )?
        else {
            panic!("certified CurveRegion2 Bezier offset became uncertain");
        };
        certified_loops += black_box(result.region().boundary_loops().len());
    }
    let certified_elapsed = started.elapsed();
    println!(
        "curve_region_bezier_parallel_certified: {iterations} iterations in {certified_elapsed:?} ({:?}/iter), loops={certified_loops}",
        certified_elapsed / iterations
    );

    let started = Instant::now();
    let mut fallback_loops = 0_usize;
    for _ in 0..iterations {
        let Classification::Decided(result) =
            source.offset_with_certified_segmentation(q(1, 10), &flattening, &policy)?
        else {
            panic!("segmented CurveRegion2 Bezier offset became uncertain");
        };
        fallback_loops += black_box(result.region().boundary_loops().len());
    }
    let fallback_elapsed = started.elapsed();
    println!(
        "curve_region_bezier_parallel_chord_fallback: {iterations} iterations in {fallback_elapsed:?} ({:?}/iter), loops={fallback_loops}",
        fallback_elapsed / iterations
    );
    Ok(())
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    bench_line_offset(100_000)?;

    let clockwise_arc = Segment2::Arc(CircularArc2::from_bulge(p(0, 0), p(2, 0), s(-1))?);
    bench_arc_offset("clockwise_arc_offset_left", &clockwise_arc, 100_000)?;

    let counter_clockwise_right_offset =
        Segment2::Arc(CircularArc2::from_bulge(p(0, 0), p(2, 0), s(1))?);
    let policy = CurvePolicy::STRICT;
    let started = Instant::now();
    let iterations = 100_000;
    let mut checksum = 0_usize;
    for _ in 0..iterations {
        let Classification::Decided(offset) =
            counter_clockwise_right_offset.offset_left(s(-1), &policy)?
        else {
            panic!("counter_clockwise_arc_right_offset became uncertain during benchmark");
        };
        checksum += black_box(offset.start().x().to_f64_lossy().is_some() as usize);
    }
    let elapsed = started.elapsed();
    println!(
        "counter_clockwise_arc_right_offset: {iterations} iterations in {elapsed:?} ({:?}/iter), checksum={checksum}",
        elapsed / iterations
    );

    bench_curve_string_joined_offset(100_000)?;
    bench_curve_string_round_join_offset(100_000)?;
    bench_curve_string_checked_offset(100_000)?;
    bench_curve_string_checked_offset_evidence(100_000)?;
    bench_curve_string_round_cap_outline(100_000)?;
    bench_curve_string_round_cap_outline_evidence(100_000)?;
    bench_curve_string_butt_cap_outline(100_000)?;
    bench_curve_string_square_cap_outline(100_000)?;
    bench_contour_joined_offset(100_000)?;
    bench_contour_checked_offset(100_000)?;
    bench_contour_checked_offset_evidence(100_000)?;
    bench_exact_bezier_parallel_evaluation(10_000)?;
    bench_bezier_parallel_cusp_isolation(100)?;
    bench_exact_ph_offset_construction(1_000)?;
    bench_certified_bezier_parallel_construction(100)?;
    bench_curve_region_bezier_offset_lanes(10)?;

    Ok(())
}
