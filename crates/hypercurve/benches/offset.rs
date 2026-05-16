use std::hint::black_box;
use std::time::Instant;

use hypercurve::{
    BulgeVertex2, CircularArc2, Classification, Contour2, CurvePolicy, CurveResult, CurveString2,
    LineSeg2, OffsetCap, Point2, Real, Segment2,
};

fn s(value: i32) -> Real {
    value.into()
}

fn p(x: i32, y: i32) -> Point2 {
    Point2::new(s(x), s(y))
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
        checksum += black_box(offset.start().x().to_f64_approx().is_some() as usize);
    }

    let elapsed = started.elapsed();
    println!(
        "line_offset_left: {iterations} iterations in {elapsed:?} ({:?}/iter), checksum={checksum}",
        elapsed / iterations
    );
    Ok(())
}

fn bench_arc_offset(name: &str, segment: &Segment2, iterations: u32) -> CurveResult<()> {
    let policy = CurvePolicy::certified();
    let started = Instant::now();
    let mut checksum = 0_usize;

    for _ in 0..iterations {
        let Classification::Decided(offset) = segment.offset_left(s(1), &policy)? else {
            panic!("{name} became uncertain during benchmark");
        };
        checksum += black_box(offset.end().y().to_f64_approx().is_some() as usize);
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
    let policy = CurvePolicy::certified();
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
    let policy = CurvePolicy::certified();
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
    let policy = CurvePolicy::certified();
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

fn bench_curve_string_round_cap_outline(iterations: u32) -> CurveResult<()> {
    let curve = CurveString2::try_new(vec![
        line_segment(0, 0, 4, 0),
        line_segment(4, 0, 4, 3),
        line_segment(4, 3, 7, 3),
    ])?;
    let policy = CurvePolicy::certified();
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

fn bench_curve_string_butt_cap_outline(iterations: u32) -> CurveResult<()> {
    let curve = CurveString2::try_new(vec![
        line_segment(0, 0, 4, 0),
        line_segment(4, 0, 4, 3),
        line_segment(4, 3, 7, 3),
    ])?;
    let policy = CurvePolicy::certified();
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
    let policy = CurvePolicy::certified();
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
    let policy = CurvePolicy::certified();
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
    let policy = CurvePolicy::certified();
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

fn main() -> CurveResult<()> {
    bench_line_offset(100_000)?;

    let clockwise_arc = Segment2::Arc(CircularArc2::from_bulge(p(0, 0), p(2, 0), s(-1))?);
    bench_arc_offset("clockwise_arc_offset_left", &clockwise_arc, 100_000)?;

    let counter_clockwise_right_offset =
        Segment2::Arc(CircularArc2::from_bulge(p(0, 0), p(2, 0), s(1))?);
    let policy = CurvePolicy::certified();
    let started = Instant::now();
    let iterations = 100_000;
    let mut checksum = 0_usize;
    for _ in 0..iterations {
        let Classification::Decided(offset) =
            counter_clockwise_right_offset.offset_left(s(-1), &policy)?
        else {
            panic!("counter_clockwise_arc_right_offset became uncertain during benchmark");
        };
        checksum += black_box(offset.start().x().to_f64_approx().is_some() as usize);
    }
    let elapsed = started.elapsed();
    println!(
        "counter_clockwise_arc_right_offset: {iterations} iterations in {elapsed:?} ({:?}/iter), checksum={checksum}",
        elapsed / iterations
    );

    bench_curve_string_joined_offset(100_000)?;
    bench_curve_string_round_join_offset(100_000)?;
    bench_curve_string_checked_offset(100_000)?;
    bench_curve_string_round_cap_outline(100_000)?;
    bench_curve_string_butt_cap_outline(100_000)?;
    bench_curve_string_square_cap_outline(100_000)?;
    bench_contour_joined_offset(100_000)?;
    bench_contour_checked_offset(100_000)?;

    Ok(())
}
