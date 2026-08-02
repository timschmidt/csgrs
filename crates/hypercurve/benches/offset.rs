use std::hint::black_box;
use std::time::Instant;

use hypercurve::{
    BezierFlatteningOptions, BezierParallelIntersectionCandidates2,
    BezierParallelVerificationOptions, BulgeVertex2, CircularArc2, Classification, Contour2,
    CubicBezier2, Curve2, CurveContext, CurvePath2, CurveRegion2, CurveRegionLoopRole, CurveResult,
    CurveString2, FillRule, LineSeg2, OffsetCap, Point2, QuadraticBezier2, RationalBezier2, Real,
    Segment2,
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
    let policy = CurveContext::STRICT;
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
    let policy = CurveContext::STRICT;
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
    let policy = CurveContext::STRICT;
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
    let policy = CurveContext::STRICT;
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
    let policy = CurveContext::STRICT;
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
    let policy = CurveContext::STRICT;
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
    let policy = CurveContext::STRICT;
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
    let policy = CurveContext::STRICT;
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
    let policy = CurveContext::STRICT;
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
    let policy = CurveContext::STRICT;
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
    let policy = CurveContext::STRICT;
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
    let policy = CurveContext::STRICT;
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
    let policy = CurveContext::STRICT;
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

fn bench_exact_bezier_parallel_construction(iterations: u32) -> CurveResult<()> {
    let source = CubicBezier2::new(p(0, 0), p(1, 2), p(3, -1), p(4, 0));
    let distance = q(1, 10);
    let started = Instant::now();
    let mut checksum = 0_usize;
    for _ in 0..iterations {
        let parallel = source.parallel_left(distance.clone())?;
        checksum += black_box(parallel.distance().to_f64_lossy().is_some() as usize);
    }
    let elapsed = started.elapsed();
    println!(
        "bezier_parallel_exact_construct: {iterations} iterations in {elapsed:?} ({:?}/iter), checksum={checksum}",
        elapsed / iterations
    );
    Ok(())
}

fn bench_bezier_parallel_cusp_isolation(iterations: u32) -> CurveResult<()> {
    let source = QuadraticBezier2::new(p(0, 0), Point2::new(q(1, 2), s(0)), p(1, 1));
    let parallel = source.parallel_left(s(1))?;
    let policy = CurveContext::STRICT;
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
    let policy = CurveContext::STRICT;
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

fn bench_bezier_parallel_intersections(
    name: &str,
    parallel: &hypercurve::BezierParallel2,
    other: &RationalBezier2,
    iterations: u32,
) -> CurveResult<()> {
    let policy = CurveContext::STRICT;
    let started = Instant::now();
    let mut candidate_count = 0_usize;
    for _ in 0..iterations {
        let Classification::Decided(candidates) =
            parallel.intersection_candidates(black_box(other), black_box(&policy))?
        else {
            panic!("{name} candidate projection became uncertain");
        };
        candidate_count += black_box(match candidates {
            BezierParallelIntersectionCandidates2::NoIntersection => 0,
            BezierParallelIntersectionCandidates2::Candidates {
                parallel_parameters,
                other_parameters,
            } => parallel_parameters.len() + other_parameters.len(),
            BezierParallelIntersectionCandidates2::DegenerateResultant => 1,
        });
    }
    let candidate_elapsed = started.elapsed();
    println!(
        "{name}_candidates: {iterations} iterations in {candidate_elapsed:?} ({:?}/iter), checksum={candidate_count}",
        candidate_elapsed / iterations
    );

    let started = Instant::now();
    let mut contact_count = 0_usize;
    for _ in 0..iterations {
        let Classification::Decided(contacts) =
            parallel.intersections(black_box(other), black_box(&policy))?
        else {
            panic!("{name} contact replay became uncertain");
        };
        contact_count += black_box(
            contacts.contacts().len()
                + contacts.overlaps().len()
                + usize::from(!contacts.is_complete()),
        );
    }
    let contact_elapsed = started.elapsed();
    println!(
        "{name}_contacts: {iterations} iterations in {contact_elapsed:?} ({:?}/iter), checksum={contact_count}",
        contact_elapsed / iterations
    );
    Ok(())
}

fn rationalized_parabola_parallel_target() -> CurveResult<RationalBezier2> {
    RationalBezier2::try_new(
        vec![
            Point2::new(s(0), s(1)),
            Point2::new(q(-1, 18), s(1)),
            Point2::new(q(-15, 134), q(133, 134)),
            Point2::new(q(-43, 264), q(43, 44)),
            Point2::new(q(-117, 580), q(1111, 1160)),
            Point2::new(q(-25, 112), q(211, 224)),
            Point2::new(q(-9, 40), q(301, 320)),
        ],
        vec![
            s(1),
            q(3, 4),
            q(67, 120),
            q(33, 80),
            q(29, 96),
            q(7, 32),
            q(5, 32),
        ],
    )
}

fn nonlinearly_reparameterized_parabola() -> CurveResult<RationalBezier2> {
    RationalBezier2::try_new(
        vec![
            p(0, 0),
            Point2::new(q(3, 64), s(0)),
            Point2::new(q(1, 8), q(3, 512)),
            Point2::new(q(15, 64), q(9, 256)),
            Point2::new(q(3, 8), q(9, 64)),
        ],
        vec![s(1); 5],
    )
}

fn two_turning_graph_parabolas() -> CurveResult<(RationalBezier2, RationalBezier2)> {
    let source = RationalBezier2::try_new(
        vec![
            Point2::new(q(9, 128), q(81, 16_384)),
            Point2::new(q(1, 16), q(63, 16_384)),
            Point2::new(q(23, 384), q(179, 49_152)),
            Point2::new(q(1, 16), q(63, 16_384)),
            Point2::new(q(9, 128), q(81, 16_384)),
        ],
        vec![s(1); 5],
    )?;
    let target = RationalBezier2::try_new(
        vec![
            p(0, 0),
            Point2::new(q(1, 4), s(0)),
            Point2::new(q(1, 3), q(1, 6)),
            Point2::new(q(1, 4), s(0)),
            p(0, 0),
        ],
        vec![s(1); 5],
    )?;
    Ok((source, target))
}

fn closed_implicit_oval_parabolas() -> CurveResult<(RationalBezier2, RationalBezier2)> {
    let source = RationalBezier2::try_new(
        vec![
            Point2::new(q(1, 4), q(1, 16)),
            Point2::new(s(0), q(-1, 16)),
            Point2::new(q(-1, 12), q(1, 16)),
            Point2::new(s(0), q(-1, 16)),
            Point2::new(q(1, 4), q(1, 16)),
        ],
        vec![s(1); 5],
    )?;
    let target = RationalBezier2::try_new(
        vec![
            Point2::new(q(-3, 16), q(9, 256)),
            Point2::new(q(1, 16), q(-15, 256)),
            Point2::new(q(7, 48), q(59, 768)),
            Point2::new(q(1, 16), q(-15, 256)),
            Point2::new(q(-3, 16), q(9, 256)),
        ],
        vec![s(1); 5],
    )?;
    Ok((source, target))
}

fn implicit_cusp_parabolas() -> CurveResult<(RationalBezier2, RationalBezier2)> {
    let source = RationalBezier2::try_new(
        vec![
            Point2::new(q(-1, 8), q(1, 64)),
            Point2::new(s(0), q(-1, 64)),
            Point2::new(q(1, 40), q(1, 64)),
            Point2::new(s(0), q(-1, 64)),
            Point2::new(q(-1, 40), q(1, 64)),
            Point2::new(s(0), q(-1, 64)),
            Point2::new(q(1, 8), q(1, 64)),
        ],
        vec![s(1); 7],
    )?;
    let target = RationalBezier2::try_new(
        vec![
            Point2::new(q(1, 4), q(1, 16)),
            Point2::new(s(0), q(-1, 16)),
            Point2::new(q(-1, 12), q(1, 16)),
            Point2::new(s(0), q(-1, 16)),
            Point2::new(q(1, 4), q(1, 16)),
        ],
        vec![s(1); 5],
    )?;
    Ok((source, target))
}

fn bench_bezier_parallel_boundary_parameter_fiber(iterations: u32) -> CurveResult<()> {
    let parallel = QuadraticBezier2::new(p(0, 0), p(1, 0), p(2, 0)).parallel_left(s(1))?;
    let constant = RationalBezier2::try_new(vec![p(0, 1); 5], vec![s(1); 5])?;
    let policy = CurveContext::STRICT;

    let started = Instant::now();
    let mut candidate_count = 0_usize;
    for _ in 0..iterations {
        let Classification::Decided(candidates) =
            parallel.intersection_candidates(black_box(&constant), black_box(&policy))?
        else {
            panic!("boundary parameter-fiber candidate projection became uncertain");
        };
        candidate_count += black_box(usize::from(matches!(
            candidates,
            BezierParallelIntersectionCandidates2::DegenerateResultant
        )));
    }
    let candidate_elapsed = started.elapsed();
    println!(
        "bezier_parallel_boundary_parameter_fiber_candidates: {iterations} iterations in {candidate_elapsed:?} ({:?}/iter), checksum={candidate_count}",
        candidate_elapsed / iterations
    );

    let started = Instant::now();
    let mut contact_count = 0_usize;
    for _ in 0..iterations {
        let Classification::Decided(contacts) =
            parallel.intersections(black_box(&constant), black_box(&policy))?
        else {
            panic!("boundary parameter-fiber contact replay became uncertain");
        };
        contact_count += black_box(
            contacts.contacts().len()
                + contacts.overlaps().len()
                + usize::from(!contacts.is_complete())
                + usize::from(!contacts.is_empty()),
        );
    }
    let contact_elapsed = started.elapsed();
    println!(
        "bezier_parallel_boundary_parameter_fiber_contacts: {iterations} iterations in {contact_elapsed:?} ({:?}/iter), checksum={contact_count}",
        contact_elapsed / iterations
    );
    Ok(())
}

fn bench_bezier_parallel_intersection_lanes() -> CurveResult<()> {
    let exact_parallel = QuadraticBezier2::new(p(0, 0), p(1, 0), p(2, 0)).parallel_left(s(1))?;
    let exact_target = RationalBezier2::try_new(vec![p(1, 0), p(1, 2)], vec![s(1), s(1)])?;
    bench_bezier_parallel_intersections(
        "bezier_parallel_exact_intersection",
        &exact_parallel,
        &exact_target,
        1_000,
    )?;

    let algebraic_source = QuadraticBezier2::new(p(0, 0), Point2::new(q(1, 2), s(0)), p(1, 1));
    let lifted_target = RationalBezier2::try_new(vec![p(0, 1), p(2, 0)], vec![s(1), s(1)])?;
    bench_bezier_parallel_intersections(
        "bezier_parallel_algebraic_lift",
        &algebraic_source.parallel_left(s(0))?,
        &lifted_target,
        100,
    )?;

    let selected_branch_target =
        RationalBezier2::try_new(vec![p(-1, 1), p(1, 1)], vec![s(1), s(1)])?;
    bench_bezier_parallel_intersections(
        "bezier_parallel_algebraic_selected_branch",
        &algebraic_source.parallel_left(s(1))?,
        &selected_branch_target,
        100,
    )?;

    let higher_nullity_parallel =
        QuadraticBezier2::new(p(0, 0), Point2::new(q(1, 2), s(0)), p(2, 0)).parallel_left(s(1))?;
    let higher_nullity_target =
        RationalBezier2::try_new(vec![p(1, 0), p(1, 0), p(1, 2)], vec![s(1), s(1), s(1)])?;
    bench_bezier_parallel_intersections(
        "bezier_parallel_higher_nullity_fiber",
        &higher_nullity_parallel,
        &higher_nullity_target,
        100,
    )?;

    let factored_parabola = RationalBezier2::try_new(
        vec![
            p(0, 0),
            Point2::new(q(2, 7), s(0)),
            Point2::new(q(5, 8), q(1, 4)),
            p(1, 1),
        ],
        vec![s(2), q(7, 3), q(8, 3), s(3)],
    )?;
    let ordinary_vertical = RationalBezier2::try_new(vec![p(0, 0), p(0, 2)], vec![s(1), s(1)])?;
    bench_bezier_parallel_intersections(
        "bezier_parallel_rootless_source_axis_factor",
        &factored_parabola.parallel_left(s(1))?,
        &ordinary_vertical,
        10,
    )?;

    let ordinary_parabola = QuadraticBezier2::new(p(0, 0), Point2::new(q(1, 2), s(0)), p(1, 1));
    let factored_vertical = RationalBezier2::try_new(
        vec![p(0, 0), Point2::new(s(0), q(4, 5)), p(0, 2)],
        vec![s(2), q(5, 2), s(3)],
    )?;
    bench_bezier_parallel_intersections(
        "bezier_parallel_rootless_target_axis_factor",
        &ordinary_parabola.parallel_left(s(1))?,
        &factored_vertical,
        10,
    )?;

    let line_overlap_parallel =
        QuadraticBezier2::new(p(0, 0), p(1, 0), p(2, 0)).parallel_left(s(1))?;
    let line_overlap_target = RationalBezier2::try_new(vec![p(0, 1), p(2, 1)], vec![s(1), s(1)])?;
    bench_bezier_parallel_intersections(
        "bezier_parallel_line_overlap",
        &line_overlap_parallel,
        &line_overlap_target,
        1_000,
    )?;

    let ph_overlap_source = CubicBezier2::new(
        p(0, 0),
        Point2::new(q(1, 3), s(0)),
        Point2::new(q(2, 3), q(1, 3)),
        Point2::new(q(2, 3), s(1)),
    );
    let ph_overlap_parallel = ph_overlap_source.parallel_left(s(1))?;
    let Classification::Decided(Some(ph_overlap)) =
        ph_overlap_parallel.exact_pythagorean_hodograph_offset(&CurveContext::STRICT)?
    else {
        panic!("PH overlap benchmark source was not recognized");
    };
    let ph_overlap_target = RationalBezier2::try_new(
        ph_overlap.curve().control_points().to_vec(),
        ph_overlap.curve().weights().to_vec(),
    )?;
    bench_bezier_parallel_intersections(
        "bezier_parallel_ph_overlap",
        &ph_overlap_parallel,
        &ph_overlap_target,
        10,
    )?;

    let rational_component_parallel = QuadraticBezier2::new(
        p(0, 0),
        Point2::new(q(3, 16), s(0)),
        Point2::new(q(3, 8), q(9, 64)),
    )
    .parallel_left(s(1))?;
    bench_bezier_parallel_intersections(
        "bezier_parallel_nonlinear_rational_component",
        &rational_component_parallel,
        &rationalized_parabola_parallel_target()?,
        5,
    )?;

    let implicit_component_parallel =
        nonlinearly_reparameterized_parabola()?.parallel_left(s(1))?;
    let implicit_component_target = rationalized_parabola_parallel_target()?;
    bench_bezier_parallel_intersections(
        "bezier_parallel_implicit_quadratic_component",
        &implicit_component_parallel,
        &implicit_component_target,
        100,
    )?;

    let (turning_source, turning_target) = two_turning_graph_parabolas()?;
    bench_bezier_parallel_intersections(
        "bezier_parallel_two_turning_implicit_graphs",
        &turning_source.parallel_left(s(0))?,
        &turning_target,
        5,
    )?;

    let (oval_source, oval_target) = closed_implicit_oval_parabolas()?;
    bench_bezier_parallel_intersections(
        "bezier_parallel_closed_implicit_oval",
        &oval_source.parallel_left(s(0))?,
        &oval_target,
        5,
    )?;

    let (cusp_source, cusp_target) = implicit_cusp_parabolas()?;
    bench_bezier_parallel_intersections(
        "bezier_parallel_implicit_cusp",
        &cusp_source.parallel_left(s(0))?,
        &cusp_target,
        5,
    )?;

    bench_bezier_parallel_boundary_parameter_fiber(100)?;

    let cold_iterations = 10_u32;
    let started = Instant::now();
    let mut cold_checksum = 0_usize;
    for _ in 0..cold_iterations {
        let cold_parallel = ph_overlap_source.clone().parallel_left(s(1))?;
        let Classification::Decided(contacts) =
            cold_parallel.intersections(black_box(&ph_overlap_target), &CurveContext::STRICT)?
        else {
            panic!("cold PH overlap replay became uncertain");
        };
        cold_checksum += black_box(
            (contacts.is_complete()
                && contacts.contacts().is_empty()
                && contacts.overlaps().len() == 1) as usize,
        );
    }
    let elapsed = started.elapsed();
    println!(
        "bezier_parallel_ph_overlap_cold: {cold_iterations} iterations in {elapsed:?} ({:?}/iter), checksum={cold_checksum}",
        elapsed / cold_iterations
    );
    Ok(())
}

fn bench_certified_bezier_parallel_construction(iterations: u32) -> CurveResult<()> {
    let source = CubicBezier2::new(p(0, 0), p(1, 2), p(2, -1), p(4, 0));
    let policy = CurveContext::STRICT;
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
        &CurveContext::STRICT,
    )?
    .into_value();
    let policy = CurveContext::STRICT;
    let verification = BezierParallelVerificationOptions::try_new(q(1, 20), 16, &policy)?;
    let flattening = BezierFlatteningOptions::try_new(q(1, 20), 16, &policy)?;

    let started = Instant::now();
    let mut certified_loops = 0_usize;
    for _ in 0..iterations {
        let Classification::Decided(result) = source
            .offset_with_certified_bezier_parallel(
                q(1, 10),
                &verification,
                &flattening,
                &flattening,
                &policy,
            )?
            .into_value()
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
        let Classification::Decided(result) = source
            .offset_with_certified_segmentation(q(1, 10), &flattening, &policy)?
            .into_value()
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
    if let Ok(group) = std::env::var("HYPERCURVE_OFFSET_BENCH_GROUP") {
        match group.as_str() {
            "bezier-construct" => bench_exact_bezier_parallel_construction(100_000)?,
            "bezier-eval" => bench_exact_bezier_parallel_evaluation(10_000)?,
            "bezier-ph" => bench_exact_ph_offset_construction(1_000)?,
            "bezier-carrier" => {
                bench_exact_bezier_parallel_construction(100_000)?;
                bench_exact_bezier_parallel_evaluation(10_000)?;
                bench_exact_ph_offset_construction(1_000)?;
            }
            "bezier-intersection" => bench_bezier_parallel_intersection_lanes()?,
            _ => panic!("unknown HYPERCURVE_OFFSET_BENCH_GROUP={group:?}"),
        }
        return Ok(());
    }

    bench_line_offset(100_000)?;

    let clockwise_arc = Segment2::Arc(CircularArc2::from_bulge(p(0, 0), p(2, 0), s(-1))?);
    bench_arc_offset("clockwise_arc_offset_left", &clockwise_arc, 100_000)?;

    let counter_clockwise_right_offset =
        Segment2::Arc(CircularArc2::from_bulge(p(0, 0), p(2, 0), s(1))?);
    let policy = CurveContext::STRICT;
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
    bench_exact_bezier_parallel_construction(100_000)?;
    bench_exact_bezier_parallel_evaluation(10_000)?;
    bench_bezier_parallel_cusp_isolation(100)?;
    bench_exact_ph_offset_construction(1_000)?;
    bench_certified_bezier_parallel_construction(100)?;
    bench_curve_region_bezier_offset_lanes(10)?;

    Ok(())
}
