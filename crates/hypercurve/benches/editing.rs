use std::hint::black_box;
use std::time::Instant;

use hypercurve::{
    BooleanOp, BulgeVertex2, CircularArc2, Classification, Contour2, CurveContext,
    CurveCornerMode2, CurveCornerSolutions2, CurveRegion2, CurveRegionLoopRole, CurveResult,
    CurveString2, CurveStringEndpoint2, CurveStringTrimPoint2, FillRule, LineArcRegion2, LineSeg2,
    Point2, QuadraticBezier2, RationalBezier2, Real, Segment2,
};
use hypercurve::{Curve2, CurvePath2};

fn s(value: i32) -> Real {
    value.into()
}

fn q(numerator: i32, denominator: i32) -> Real {
    (Real::from(numerator) / Real::from(denominator)).unwrap()
}

fn p(x: i32, y: i32) -> Point2 {
    Point2::new(s(x), s(y))
}

fn expect_decided<T>(classification: Classification<T>, message: &str) -> T {
    match classification {
        Classification::Decided(value) => value,
        Classification::Uncertain(_) => panic!("{message}"),
    }
}

fn corner_lane_enabled(name: &str) -> bool {
    match std::env::var("HYPERCURVE_EDIT_CORNER_LANE") {
        Ok(selected) => selected == name,
        Err(_) => true,
    }
}

fn benchmark_iterations() -> u32 {
    let iterations = match std::env::var("HYPERCURVE_EDIT_ITERATIONS") {
        Ok(iterations) => iterations
            .parse()
            .expect("HYPERCURVE_EDIT_ITERATIONS must be a positive u32"),
        Err(_) => 10_000,
    };
    assert!(iterations > 0, "benchmark iterations must be nonzero");
    iterations
}

fn vertex(x: i32, y: i32, bulge: i32) -> BulgeVertex2 {
    BulgeVertex2::new(p(x, y), s(bulge))
}

fn line_segment(start_x: i32, start_y: i32, end_x: i32, end_y: i32) -> Segment2 {
    Segment2::Line(LineSeg2::try_new(p(start_x, start_y), p(end_x, end_y)).unwrap())
}

fn line(start_x: i32, start_y: i32, end_x: i32, end_y: i32) -> LineSeg2 {
    LineSeg2::try_new(p(start_x, start_y), p(end_x, end_y)).unwrap()
}

fn rectangle(xmin: i32, ymin: i32, xmax: i32, ymax: i32) -> Contour2 {
    Contour2::from_bulge_vertices(&[
        vertex(xmin, ymin, 0),
        vertex(xmax, ymin, 0),
        vertex(xmax, ymax, 0),
        vertex(xmin, ymax, 0),
    ])
    .unwrap()
}

fn higher_order_fillet_path() -> CurvePath2 {
    CurvePath2::try_new(vec![
        Curve2::from(line(0, 0, 4, 0)),
        Curve2::from(QuadraticBezier2::new(p(4, 0), p(3, 4), p(2, 0))),
        Curve2::from(line(2, 0, 2, -2)),
        Curve2::from(line(2, -2, 0, -2)),
        Curve2::from(line(0, -2, 0, 0)),
    ])
    .expect("higher-order editing benchmark path must be connected")
}

fn subdivided_rectangle(edge_steps: i32) -> Contour2 {
    let mut vertices = Vec::with_capacity((edge_steps as usize) * 4);
    vertices.extend((0..edge_steps).map(|x| vertex(x, 0, 0)));
    vertices.extend((0..edge_steps).map(|y| vertex(edge_steps, y, 0)));
    vertices.extend((1..=edge_steps).rev().map(|x| vertex(x, edge_steps, 0)));
    vertices.extend((1..=edge_steps).rev().map(|y| vertex(0, y, 0)));
    Contour2::from_bulge_vertices(&vertices).unwrap()
}

fn bench_parameter_trim(iterations: u32) -> CurveResult<()> {
    let curve = CurveString2::try_new(vec![
        line_segment(0, 0, 10, 0),
        line_segment(10, 0, 10, 6),
        line_segment(10, 6, 16, 6),
    ])?;
    let policy = CurveContext::STRICT;
    let started = Instant::now();
    let mut total_segments = 0_usize;

    for _ in 0..iterations {
        let result = curve.trim_between_parameters(
            CurveStringTrimPoint2::new(0, q(1, 5)),
            CurveStringTrimPoint2::new(2, q(1, 2)),
            &policy,
        )?;
        let trimmed = expect_decided(result, "parameter trim benchmark should materialize");
        total_segments += black_box(trimmed.len());
    }

    let elapsed = started.elapsed();
    println!(
        "curve_string_parameter_trim: {iterations} iterations in {elapsed:?} ({:?}/iter), total segments={total_segments}",
        elapsed / iterations
    );
    Ok(())
}

fn bench_point_arc_trim(iterations: u32) -> CurveResult<()> {
    let curve = CurveString2::try_new(vec![Segment2::Arc(CircularArc2::from_bulge(
        p(0, 0),
        p(2, 0),
        s(1),
    )?)])?;
    let policy = CurveContext::STRICT;
    let started = Instant::now();
    let mut total_segments = 0_usize;

    for _ in 0..iterations {
        let result = curve.trim_between_points(&p(1, -1), &p(2, 0), &policy)?;
        let trimmed = expect_decided(
            result,
            "point-bearing arc trim benchmark should materialize",
        );
        total_segments += black_box(trimmed.len());
    }

    let elapsed = started.elapsed();
    println!(
        "curve_string_point_arc_trim: {iterations} iterations in {elapsed:?} ({:?}/iter), total segments={total_segments}",
        elapsed / iterations
    );
    Ok(())
}

fn bench_parameter_arc_trim(iterations: u32) -> CurveResult<()> {
    let curve = CurveString2::try_new(vec![Segment2::Arc(CircularArc2::from_bulge(
        p(0, 0),
        p(2, 0),
        s(1),
    )?)])?;
    let policy = CurveContext::STRICT;
    let start = CurveStringTrimPoint2::new(0, q(1, 7));
    let end = CurveStringTrimPoint2::new(0, q(5, 7));
    let started = Instant::now();
    let mut total_segments = 0_usize;

    for _ in 0..iterations {
        let result = curve.trim_between_parameters(start.clone(), end.clone(), &policy)?;
        let trimmed = expect_decided(result, "parameter arc trim benchmark should materialize");
        total_segments += black_box(trimmed.len());
    }

    let elapsed = started.elapsed();
    println!(
        "curve_string_parameter_arc_trim: {iterations} iterations in {elapsed:?} ({:?}/iter), total segments={total_segments}",
        elapsed / iterations
    );
    Ok(())
}

fn bench_curve_intersection_trim(iterations: u32) -> CurveResult<()> {
    let curve = CurveString2::try_new(vec![line_segment(0, 0, 10, 0)])?;
    let start_cutter = CurveString2::try_new(vec![line_segment(2, -1, 2, 1)])?;
    let end_cutter = CurveString2::try_new(vec![line_segment(8, -1, 8, 1)])?;
    let policy = CurveContext::STRICT;
    let started = Instant::now();
    let mut total_segments = 0_usize;

    for _ in 0..iterations {
        let result = curve.trim_between_curve_intersections(&start_cutter, &end_cutter, &policy)?;
        let trimmed = expect_decided(
            result,
            "curve-intersection trim benchmark should materialize",
        );
        total_segments += black_box(trimmed.len());
    }

    let elapsed = started.elapsed();
    println!(
        "curve_string_curve_intersection_trim: {iterations} iterations in {elapsed:?} ({:?}/iter), total segments={total_segments}",
        elapsed / iterations
    );
    Ok(())
}

fn bench_region_trim(iterations: u32) -> CurveResult<()> {
    let curve = CurveString2::try_new(vec![line_segment(-2, 1, 8, 1)])?;
    let region =
        LineArcRegion2::from_material_contours(vec![rectangle(0, 0, 2, 2), rectangle(4, 0, 6, 2)]);
    let policy = CurveContext::STRICT;
    let started = Instant::now();
    let mut total_outputs = 0_usize;

    for _ in 0..iterations {
        let result = curve.trim_inside_region(&region, &policy)?;
        let trimmed = expect_decided(result, "region trim benchmark became uncertain");
        total_outputs += black_box(trimmed.len());
    }

    let elapsed = started.elapsed();
    println!(
        "curve_string_region_trim: {iterations} iterations in {elapsed:?} ({:?}/iter), total outputs={total_outputs}",
        elapsed / iterations
    );
    Ok(())
}

fn bench_line_chamfer(iterations: u32) -> CurveResult<()> {
    let curve = CurveString2::try_new(vec![
        line_segment(0, 0, 4, 0),
        line_segment(4, 0, 4, 4),
        line_segment(4, 4, 8, 4),
    ])?;
    let policy = CurveContext::STRICT;
    let started = Instant::now();
    let mut total_segments = 0_usize;

    for _ in 0..iterations {
        let result = curve.chamfer_vertex_by_parameters(1, q(3, 4), q(1, 4), &policy)?;
        let Classification::Decided(chamfered) = result else {
            panic!("line-line chamfer benchmark became uncertain");
        };
        total_segments += black_box(chamfered.len());
    }

    let elapsed = started.elapsed();
    println!(
        "curve_string_line_chamfer: {iterations} iterations in {elapsed:?} ({:?}/iter), total segments={total_segments}",
        elapsed / iterations
    );
    Ok(())
}

fn bench_arc_chamfer(iterations: u32) -> CurveResult<()> {
    let curve = CurveString2::try_new(vec![
        Segment2::Arc(CircularArc2::from_bulge(p(0, 0), p(2, 0), s(1))?),
        Segment2::Arc(CircularArc2::from_bulge(p(2, 0), p(4, 0), s(1))?),
    ])?;
    let policy = CurveContext::STRICT;
    let started = Instant::now();
    let mut total_segments = 0_usize;

    for _ in 0..iterations {
        let result = curve.chamfer_vertex_by_parameters(1, q(5, 7), q(2, 7), &policy)?;
        let Classification::Decided(chamfered) = result else {
            panic!("arc-arc chamfer benchmark became uncertain");
        };
        total_segments += black_box(chamfered.len());
    }

    let elapsed = started.elapsed();
    println!(
        "curve_string_arc_chamfer: {iterations} iterations in {elapsed:?} ({:?}/iter), total segments={total_segments}",
        elapsed / iterations
    );
    Ok(())
}

fn bench_line_fillet(iterations: u32) -> CurveResult<()> {
    let curve = CurveString2::try_new(vec![
        line_segment(0, 0, 4, 0),
        line_segment(4, 0, 4, 4),
        line_segment(4, 4, 8, 4),
    ])?;
    let policy = CurveContext::STRICT;
    let started = Instant::now();
    let mut total_segments = 0_usize;

    for _ in 0..iterations {
        let result =
            curve.fillet_vertex_by_parameters(1, q(3, 4), q(1, 4), &p(3, 1), false, &policy)?;
        let Classification::Decided(filleted) = result else {
            panic!("line-line fillet benchmark became uncertain");
        };
        total_segments += black_box(filleted.len());
    }

    let elapsed = started.elapsed();
    println!(
        "curve_string_line_fillet: {iterations} iterations in {elapsed:?} ({:?}/iter), total segments={total_segments}",
        elapsed / iterations
    );
    Ok(())
}

fn bench_line_curve_corner_solvers(iterations: u32) {
    let path = CurvePath2::try_new(vec![
        Curve2::from(line(0, 0, 4, 0)),
        Curve2::from(line(4, 0, 4, 4)),
        Curve2::from(line(4, 4, 8, 4)),
    ])
    .expect("line corner benchmark path must be connected");
    let policy = CurveContext::STRICT;
    let previous_parameter = q(3, 4);
    let next_parameter = q(1, 4);
    let design_value = s(1);

    if corner_lane_enabled("curve_path_parameter_chamfer") {
        let started = Instant::now();
        let mut curves = 0_usize;
        for _ in 0..iterations {
            let chamfered = black_box(&path)
                .chamfer_vertex_by_parameters(
                    1,
                    previous_parameter.clone(),
                    next_parameter.clone(),
                    &policy,
                )
                .expect("parameter chamfer benchmark must remain exact")
                .into_value();
            curves += black_box(chamfered).curves().len();
        }
        let elapsed = started.elapsed();
        println!(
            "curve_path_parameter_chamfer: {iterations} iterations in {elapsed:?} ({:?}/iter), curves={curves}",
            elapsed / iterations
        );
    }

    if corner_lane_enabled("curve_path_design_chamfer") {
        let started = Instant::now();
        let mut curves = 0_usize;
        for _ in 0..iterations {
            let CurveCornerSolutions2::Unique(chamfered) = black_box(&path)
                .chamfer_vertex_by_setbacks(
                    1,
                    design_value.clone(),
                    design_value.clone(),
                    CurveCornerMode2::TrimOnly,
                    &policy,
                )
                .expect("design-parameter chamfer benchmark must remain exact")
                .into_value()
            else {
                panic!("line design-parameter chamfer benchmark must be unique");
            };
            curves += black_box(chamfered).curves().len();
        }
        let elapsed = started.elapsed();
        println!(
            "curve_path_design_chamfer: {iterations} iterations in {elapsed:?} ({:?}/iter), curves={curves}",
            elapsed / iterations
        );
    }

    if corner_lane_enabled("curve_path_parameter_fillet") {
        let started = Instant::now();
        let mut curves = 0_usize;
        for _ in 0..iterations {
            let filleted = black_box(&path)
                .fillet_vertex_by_parameters(
                    1,
                    previous_parameter.clone(),
                    next_parameter.clone(),
                    &p(3, 1),
                    false,
                    &policy,
                )
                .expect("parameter fillet benchmark must remain exact")
                .into_value();
            curves += black_box(filleted).curves().len();
        }
        let elapsed = started.elapsed();
        println!(
            "curve_path_parameter_fillet: {iterations} iterations in {elapsed:?} ({:?}/iter), curves={curves}",
            elapsed / iterations
        );
    }

    if corner_lane_enabled("curve_path_design_fillet") {
        let started = Instant::now();
        let mut curves = 0_usize;
        for _ in 0..iterations {
            let CurveCornerSolutions2::Unique(filleted) = black_box(&path)
                .fillet_vertex_by_radius(
                    1,
                    design_value.clone(),
                    CurveCornerMode2::TrimOnly,
                    &policy,
                )
                .expect("design-parameter fillet benchmark must remain exact")
                .into_value()
            else {
                panic!("line design-parameter fillet benchmark must be unique");
            };
            curves += black_box(filleted).curves().len();
        }
        let elapsed = started.elapsed();
        println!(
            "curve_path_design_fillet: {iterations} iterations in {elapsed:?} ({:?}/iter), curves={curves}",
            elapsed / iterations
        );
    }
}

fn bench_native_arc_chamfer_solvers(iterations: u32) -> CurveResult<()> {
    let policy = CurveContext::STRICT;
    let half_root_three =
        (s(3).sqrt().expect("sqrt(3) must exist") / s(2)).expect("division by two must exist");
    let next_cut = Point2::new(s(1) + &half_root_three, q(1, 2));
    let next_arc = CircularArc2::try_from_center(p(1, 0), p(2, 1), p(1, 1), false)?;
    let Classification::Decided(next_sweep) = next_arc.sweep_fraction(&next_cut, &policy)? else {
        panic!("line-arc benchmark sweep must remain exact");
    };
    let Classification::Decided(next_public_parameter) =
        next_arc.parameter_at_sweep_fraction(&next_sweep, &policy)?
    else {
        panic!("line-arc benchmark public parameter must remain exact");
    };
    let line_arc_path = CurvePath2::try_new(vec![
        Curve2::from(line(-1, 0, 1, 0)),
        Curve2::from(next_arc.clone()),
    ])
    .expect("line-arc benchmark path must remain exact");

    if corner_lane_enabled("curve_path_line_arc_parameter_chamfer") {
        let started = Instant::now();
        let mut curves = 0_usize;
        for _ in 0..iterations {
            let chamfered = black_box(&line_arc_path)
                .chamfer_vertex_by_parameters(1, q(3, 4), next_public_parameter.clone(), &policy)
                .expect("line-arc parameter chamfer must remain exact")
                .into_value();
            curves += black_box(chamfered).curves().len();
        }
        let elapsed = started.elapsed();
        println!(
            "curve_path_line_arc_parameter_chamfer: {iterations} iterations in {elapsed:?} ({:?}/iter), curves={curves}",
            elapsed / iterations
        );
    }

    if corner_lane_enabled("curve_path_line_arc_design_chamfer") {
        let started = Instant::now();
        let mut curves = 0_usize;
        for _ in 0..iterations {
            let CurveCornerSolutions2::Unique(chamfered) = black_box(&line_arc_path)
                .chamfer_vertex_by_setbacks(1, q(1, 2), s(1), CurveCornerMode2::TrimOnly, &policy)
                .expect("line-arc design chamfer must remain exact")
                .into_value()
            else {
                panic!("line-arc design chamfer must remain unique");
            };
            curves += black_box(chamfered).curves().len();
        }
        let elapsed = started.elapsed();
        println!(
            "curve_path_line_arc_design_chamfer: {iterations} iterations in {elapsed:?} ({:?}/iter), curves={curves}",
            elapsed / iterations
        );
    }

    let previous_cut = Point2::new(s(1) - &half_root_three, q(-1, 2));
    let previous_arc = CircularArc2::try_from_center(p(0, -1), p(1, 0), p(1, -1), true)?;
    let Classification::Decided(previous_sweep) =
        previous_arc.sweep_fraction(&previous_cut, &policy)?
    else {
        panic!("arc-arc benchmark previous sweep must remain exact");
    };
    let Classification::Decided(previous_public_parameter) =
        previous_arc.parameter_at_sweep_fraction(&previous_sweep, &policy)?
    else {
        panic!("arc-arc benchmark previous public parameter must remain exact");
    };
    let arc_arc_path = CurvePath2::try_new(vec![
        Curve2::from(previous_arc),
        Curve2::from(next_arc.clone()),
    ])
    .expect("arc-arc benchmark path must remain exact");

    if corner_lane_enabled("curve_path_arc_arc_parameter_chamfer") {
        let started = Instant::now();
        let mut curves = 0_usize;
        for _ in 0..iterations {
            let chamfered = black_box(&arc_arc_path)
                .chamfer_vertex_by_parameters(
                    1,
                    previous_public_parameter.clone(),
                    next_public_parameter.clone(),
                    &policy,
                )
                .expect("arc-arc parameter chamfer must remain exact")
                .into_value();
            curves += black_box(chamfered).curves().len();
        }
        let elapsed = started.elapsed();
        println!(
            "curve_path_arc_arc_parameter_chamfer: {iterations} iterations in {elapsed:?} ({:?}/iter), curves={curves}",
            elapsed / iterations
        );
    }

    if corner_lane_enabled("curve_path_arc_arc_design_chamfer") {
        let started = Instant::now();
        let mut curves = 0_usize;
        for _ in 0..iterations {
            let CurveCornerSolutions2::Unique(chamfered) = black_box(&arc_arc_path)
                .chamfer_vertex_by_setbacks(1, s(1), s(1), CurveCornerMode2::TrimOnly, &policy)
                .expect("arc-arc design chamfer must remain exact")
                .into_value()
            else {
                panic!("arc-arc design chamfer must remain unique");
            };
            curves += black_box(chamfered).curves().len();
        }
        let elapsed = started.elapsed();
        println!(
            "curve_path_arc_arc_design_chamfer: {iterations} iterations in {elapsed:?} ({:?}/iter), curves={curves}",
            elapsed / iterations
        );
    }

    let rounded_contour = Contour2::try_new(vec![
        Segment2::Line(line(-1, 0, 1, 0)),
        Segment2::Arc(next_arc),
        Segment2::Line(line(2, 1, 2, 3)),
        Segment2::Line(line(2, 3, -1, 3)),
        Segment2::Line(line(-1, 3, -1, 0)),
    ])?;
    let region = CurveRegion2::try_from_native_material_contours(vec![rounded_contour], &policy)
        .expect("line-arc benchmark region must promote")
        .into_value();

    if corner_lane_enabled("curve_region_line_arc_parameter_chamfer") {
        let started = Instant::now();
        let mut loops = 0_usize;
        for _ in 0..iterations {
            let Classification::Decided(chamfered) = black_box(&region)
                .chamfer_loop_vertex_by_parameters(0, 1, q(3, 4), next_sweep.clone(), &policy)
                .expect("line-arc region parameter chamfer must remain exact")
                .into_value()
            else {
                panic!("line-arc region parameter chamfer must remain decided");
            };
            loops += black_box(chamfered).boundary_loops().len();
        }
        let elapsed = started.elapsed();
        println!(
            "curve_region_line_arc_parameter_chamfer: {iterations} iterations in {elapsed:?} ({:?}/iter), loops={loops}",
            elapsed / iterations
        );
    }

    if corner_lane_enabled("curve_region_line_arc_design_chamfer") {
        let started = Instant::now();
        let mut loops = 0_usize;
        for _ in 0..iterations {
            let CurveCornerSolutions2::Unique(chamfered) = black_box(&region)
                .chamfer_loop_vertex_by_setbacks(
                    0,
                    1,
                    q(1, 2),
                    s(1),
                    CurveCornerMode2::TrimOnly,
                    &policy,
                )
                .expect("line-arc region design chamfer must remain exact")
                .into_value()
            else {
                panic!("line-arc region design chamfer must remain unique");
            };
            loops += black_box(chamfered).boundary_loops().len();
        }
        let elapsed = started.elapsed();
        println!(
            "curve_region_line_arc_design_chamfer: {iterations} iterations in {elapsed:?} ({:?}/iter), loops={loops}",
            elapsed / iterations
        );
    }

    Ok(())
}

fn bench_native_arc_fillet_solvers(iterations: u32) -> CurveResult<()> {
    let policy = CurveContext::STRICT;
    let radius = q(1, 2);
    let root_two = s(2).sqrt().expect("sqrt(2) must exist");
    let line_arc_center = Point2::new(s(1) - &root_two, radius.clone());
    let line_arc_previous_parameter = ((s(3) - &root_two) / s(2))?;
    let line_arc_next_contact = Point2::new(s(1) - (&root_two * q(2, 3)), q(1, 3));
    let next_arc = CircularArc2::try_from_center(p(0, 0), p(1, 1), p(1, 0), true)?;
    let Classification::Decided(line_arc_next_sweep) =
        next_arc.sweep_fraction(&line_arc_next_contact, &policy)?
    else {
        panic!("line-arc fillet benchmark sweep must remain exact");
    };
    let Classification::Decided(line_arc_next_public_parameter) =
        next_arc.parameter_at_sweep_fraction(&line_arc_next_sweep, &policy)?
    else {
        panic!("line-arc fillet benchmark public parameter must remain exact");
    };
    let line_arc_path = CurvePath2::try_new(vec![
        Curve2::from(line(-2, 0, 0, 0)),
        Curve2::from(next_arc.clone()),
    ])
    .expect("line-arc fillet benchmark path must remain exact");

    if corner_lane_enabled("curve_path_line_arc_parameter_fillet") {
        let started = Instant::now();
        let mut curves = 0_usize;
        for _ in 0..iterations {
            let filleted = black_box(&line_arc_path)
                .fillet_vertex_by_parameters(
                    1,
                    line_arc_previous_parameter.clone(),
                    line_arc_next_public_parameter.clone(),
                    &line_arc_center,
                    false,
                    &policy,
                )
                .expect("line-arc parameter fillet must remain exact")
                .into_value();
            curves += black_box(filleted).curves().len();
        }
        let elapsed = started.elapsed();
        println!(
            "curve_path_line_arc_parameter_fillet: {iterations} iterations in {elapsed:?} ({:?}/iter), curves={curves}",
            elapsed / iterations
        );
    }

    if corner_lane_enabled("curve_path_line_arc_design_fillet") {
        let started = Instant::now();
        let mut curves = 0_usize;
        for _ in 0..iterations {
            let CurveCornerSolutions2::Unique(filleted) = black_box(&line_arc_path)
                .fillet_vertex_by_radius(1, radius.clone(), CurveCornerMode2::TrimOnly, &policy)
                .expect("line-arc design fillet must remain exact")
                .into_value()
            else {
                panic!("line-arc design fillet must remain unique");
            };
            curves += black_box(filleted).curves().len();
        }
        let elapsed = started.elapsed();
        println!(
            "curve_path_line_arc_design_fillet: {iterations} iterations in {elapsed:?} ({:?}/iter), curves={curves}",
            elapsed / iterations
        );
    }

    let root_fourteen = s(14).sqrt().expect("sqrt(14) must exist");
    let arc_arc_center = Point2::new(
        ((s(2) - &root_fourteen) / s(4))?,
        ((&root_fourteen - s(2)) / s(4))?,
    );
    let previous_contact = Point2::new(
        ((s(2) - &root_fourteen) / s(6))?,
        ((&root_fourteen - s(4)) / s(6))?,
    );
    let next_contact = Point2::new(
        ((s(4) - &root_fourteen) / s(6))?,
        ((&root_fourteen - s(2)) / s(6))?,
    );
    let previous_arc = CircularArc2::try_from_center(p(-1, -1), p(0, 0), p(0, -1), true)?;
    let Classification::Decided(previous_sweep) =
        previous_arc.sweep_fraction(&previous_contact, &policy)?
    else {
        panic!("arc-arc fillet benchmark previous sweep must remain exact");
    };
    let Classification::Decided(previous_public_parameter) =
        previous_arc.parameter_at_sweep_fraction(&previous_sweep, &policy)?
    else {
        panic!("arc-arc fillet benchmark previous public parameter must remain exact");
    };
    let Classification::Decided(next_sweep) = next_arc.sweep_fraction(&next_contact, &policy)?
    else {
        panic!("arc-arc fillet benchmark next sweep must remain exact");
    };
    let Classification::Decided(next_public_parameter) =
        next_arc.parameter_at_sweep_fraction(&next_sweep, &policy)?
    else {
        panic!("arc-arc fillet benchmark next public parameter must remain exact");
    };
    let arc_arc_path = CurvePath2::try_new(vec![
        Curve2::from(previous_arc.clone()),
        Curve2::from(next_arc.clone()),
    ])
    .expect("arc-arc fillet benchmark path must remain exact");

    if corner_lane_enabled("curve_path_arc_arc_parameter_fillet") {
        let started = Instant::now();
        let mut curves = 0_usize;
        for _ in 0..iterations {
            let filleted = black_box(&arc_arc_path)
                .fillet_vertex_by_parameters(
                    1,
                    previous_public_parameter.clone(),
                    next_public_parameter.clone(),
                    &arc_arc_center,
                    false,
                    &policy,
                )
                .expect("arc-arc parameter fillet must remain exact")
                .into_value();
            curves += black_box(filleted).curves().len();
        }
        let elapsed = started.elapsed();
        println!(
            "curve_path_arc_arc_parameter_fillet: {iterations} iterations in {elapsed:?} ({:?}/iter), curves={curves}",
            elapsed / iterations
        );
    }

    if corner_lane_enabled("curve_path_arc_arc_design_fillet") {
        let started = Instant::now();
        let mut curves = 0_usize;
        for _ in 0..iterations {
            let CurveCornerSolutions2::Unique(filleted) = black_box(&arc_arc_path)
                .fillet_vertex_by_radius(1, radius.clone(), CurveCornerMode2::TrimOnly, &policy)
                .expect("arc-arc design fillet must remain exact")
                .into_value()
            else {
                panic!("arc-arc design fillet must remain unique");
            };
            curves += black_box(filleted).curves().len();
        }
        let elapsed = started.elapsed();
        println!(
            "curve_path_arc_arc_design_fillet: {iterations} iterations in {elapsed:?} ({:?}/iter), curves={curves}",
            elapsed / iterations
        );
    }

    let line_arc_contour = Contour2::try_new(vec![
        Segment2::Line(line(-2, 0, 0, 0)),
        Segment2::Arc(next_arc.clone()),
        Segment2::Line(line(1, 1, -2, 1)),
        Segment2::Line(line(-2, 1, -2, 0)),
    ])?;
    let line_arc_region =
        CurveRegion2::try_from_native_material_contours(vec![line_arc_contour], &policy)
            .expect("line-arc fillet benchmark region must promote")
            .into_value();

    if corner_lane_enabled("curve_region_line_arc_parameter_fillet") {
        let started = Instant::now();
        let mut loops = 0_usize;
        for _ in 0..iterations {
            let Classification::Decided(filleted) = black_box(&line_arc_region)
                .fillet_loop_vertex_by_parameters(
                    0,
                    1,
                    line_arc_previous_parameter.clone(),
                    line_arc_next_sweep.clone(),
                    &line_arc_center,
                    false,
                    &policy,
                )
                .expect("line-arc region parameter fillet must remain exact")
                .into_value()
            else {
                panic!("line-arc region parameter fillet must remain decided");
            };
            loops += black_box(filleted).boundary_loops().len();
        }
        let elapsed = started.elapsed();
        println!(
            "curve_region_line_arc_parameter_fillet: {iterations} iterations in {elapsed:?} ({:?}/iter), loops={loops}",
            elapsed / iterations
        );
    }

    if corner_lane_enabled("curve_region_line_arc_design_fillet") {
        let started = Instant::now();
        let mut loops = 0_usize;
        for _ in 0..iterations {
            let CurveCornerSolutions2::Unique(filleted) = black_box(&line_arc_region)
                .fillet_loop_vertex_by_radius(
                    0,
                    1,
                    radius.clone(),
                    CurveCornerMode2::TrimOnly,
                    &policy,
                )
                .expect("line-arc region design fillet must remain exact")
                .into_value()
            else {
                panic!("line-arc region design fillet must remain unique");
            };
            loops += black_box(filleted).boundary_loops().len();
        }
        let elapsed = started.elapsed();
        println!(
            "curve_region_line_arc_design_fillet: {iterations} iterations in {elapsed:?} ({:?}/iter), loops={loops}",
            elapsed / iterations
        );
    }

    let arc_arc_contour = Contour2::try_new(vec![
        Segment2::Arc(previous_arc),
        Segment2::Arc(next_arc),
        Segment2::Line(line(1, 1, -1, 1)),
        Segment2::Line(line(-1, 1, -1, -1)),
    ])?;
    let arc_arc_region =
        CurveRegion2::try_from_native_material_contours(vec![arc_arc_contour], &policy)
            .expect("arc-arc fillet benchmark region must promote")
            .into_value();

    if corner_lane_enabled("curve_region_arc_arc_parameter_fillet") {
        let started = Instant::now();
        let mut loops = 0_usize;
        for _ in 0..iterations {
            let Classification::Decided(filleted) = black_box(&arc_arc_region)
                .fillet_loop_vertex_by_parameters(
                    0,
                    1,
                    previous_sweep.clone(),
                    next_sweep.clone(),
                    &arc_arc_center,
                    false,
                    &policy,
                )
                .expect("arc-arc region parameter fillet must remain exact")
                .into_value()
            else {
                panic!("arc-arc region parameter fillet must remain decided");
            };
            loops += black_box(filleted).boundary_loops().len();
        }
        let elapsed = started.elapsed();
        println!(
            "curve_region_arc_arc_parameter_fillet: {iterations} iterations in {elapsed:?} ({:?}/iter), loops={loops}",
            elapsed / iterations
        );
    }

    if corner_lane_enabled("curve_region_arc_arc_design_fillet") {
        let started = Instant::now();
        let mut loops = 0_usize;
        for _ in 0..iterations {
            let CurveCornerSolutions2::Unique(filleted) = black_box(&arc_arc_region)
                .fillet_loop_vertex_by_radius(
                    0,
                    1,
                    radius.clone(),
                    CurveCornerMode2::TrimOnly,
                    &policy,
                )
                .expect("arc-arc region design fillet must remain exact")
                .into_value()
            else {
                panic!("arc-arc region design fillet must remain unique");
            };
            loops += black_box(filleted).boundary_loops().len();
        }
        let elapsed = started.elapsed();
        println!(
            "curve_region_arc_arc_design_fillet: {iterations} iterations in {elapsed:?} ({:?}/iter), loops={loops}",
            elapsed / iterations
        );
    }

    Ok(())
}

fn bench_retained_circle_chamfer_lane(name: &str, path: &CurvePath2, iterations: u32) {
    let policy = CurveContext::STRICT;
    let setback = q(1, 2);
    let started = Instant::now();
    let mut curves = 0_usize;
    for _ in 0..iterations {
        let CurveCornerSolutions2::Unique(chamfered) = black_box(path)
            .chamfer_vertex_by_setbacks(
                1,
                setback.clone(),
                setback.clone(),
                CurveCornerMode2::TrimOnly,
                &policy,
            )
            .expect("retained circular chamfer must remain exact")
            .into_value()
        else {
            panic!("retained circular chamfer must remain unique");
        };
        curves += black_box(chamfered).curves().len();
    }
    let elapsed = started.elapsed();
    println!(
        "{name}: {iterations} iterations in {elapsed:?} ({:?}/iter), curves={curves}",
        elapsed / iterations
    );
}

fn bench_retained_circle_fillet_lane(name: &str, path: &CurvePath2, iterations: u32) {
    let policy = CurveContext::STRICT;
    let radius = q(1, 2);
    let started = Instant::now();
    let mut curves = 0_usize;
    for _ in 0..iterations {
        let CurveCornerSolutions2::Unique(filleted) = black_box(path)
            .fillet_vertex_by_radius(1, radius.clone(), CurveCornerMode2::TrimOnly, &policy)
            .expect("retained circular fillet must remain exact")
            .into_value()
        else {
            panic!("retained circular fillet must remain unique");
        };
        curves += black_box(filleted).curves().len();
    }
    let elapsed = started.elapsed();
    println!(
        "{name}: {iterations} iterations in {elapsed:?} ({:?}/iter), curves={curves}",
        elapsed / iterations
    );
}

fn bench_retained_circle_corner_solvers(iterations: u32) -> CurveResult<()> {
    let policy = CurveContext::STRICT;
    let native_arc = CircularArc2::try_from_center(p(0, 0), p(1, 1), p(1, 0), true)?;
    let conic = native_arc
        .rational_bezier_decomposition(&policy)
        .expect("retained-circle fixture decomposition must remain exact")
        .into_value()
        .spans()[0]
        .curve()
        .clone();
    let elevated = RationalBezier2::from(conic.clone())
        .elevated_to_degree(5)
        .expect("retained-circle fixture elevation must remain exact");
    let rational_quadratic_path =
        CurvePath2::try_new(vec![Curve2::from(line(-2, 0, 0, 0)), Curve2::from(conic)])
            .expect("rational-quadratic circle benchmark path must remain exact");
    let degree_five_path = CurvePath2::try_new(vec![
        Curve2::from(line(-2, 0, 0, 0)),
        Curve2::from(elevated),
    ])
    .expect("degree-five rational circle benchmark path must remain exact");

    for (name, path, chamfer) in [
        (
            "curve_path_rational_quadratic_circle_design_chamfer",
            &rational_quadratic_path,
            true,
        ),
        (
            "curve_path_rational_quadratic_circle_design_fillet",
            &rational_quadratic_path,
            false,
        ),
        (
            "curve_path_degree5_rational_circle_design_chamfer",
            &degree_five_path,
            true,
        ),
        (
            "curve_path_degree5_rational_circle_design_fillet",
            &degree_five_path,
            false,
        ),
    ] {
        if !corner_lane_enabled(name) {
            continue;
        }
        if chamfer {
            bench_retained_circle_chamfer_lane(name, path, iterations);
        } else {
            bench_retained_circle_fillet_lane(name, path, iterations);
        }
    }

    Ok(())
}

fn bench_represented_bezier_chamfer_lane(
    name: &str,
    path: &CurvePath2,
    previous_setback: &Real,
    next_setback: &Real,
    iterations: u32,
) {
    if !corner_lane_enabled(name) {
        return;
    }
    let policy = CurveContext::STRICT;
    let started = Instant::now();
    let mut candidates = 0_usize;
    for _ in 0..iterations {
        let solutions = black_box(path)
            .chamfer_vertex_by_setbacks(
                1,
                previous_setback.clone(),
                next_setback.clone(),
                CurveCornerMode2::TrimOnly,
                &policy,
            )
            .expect("represented Bezier chamfer must remain exact")
            .into_value();
        candidates += black_box(solutions).candidate_count();
    }
    assert_ne!(candidates, 0);
    let elapsed = started.elapsed();
    println!(
        "{name}: {iterations} iterations in {elapsed:?} ({:?}/iter), candidates={candidates}",
        elapsed / iterations
    );
}

fn bench_represented_bezier_fillet_lane(
    name: &str,
    path: &CurvePath2,
    radius: &Real,
    iterations: u32,
) {
    if !corner_lane_enabled(name) {
        return;
    }
    let policy = CurveContext::STRICT;
    let started = Instant::now();
    let mut candidates = 0_usize;
    for _ in 0..iterations {
        let solutions = black_box(path)
            .fillet_vertex_by_radius(1, radius.clone(), CurveCornerMode2::TrimOnly, &policy)
            .expect("represented Bezier fillet must remain exact")
            .into_value();
        candidates += black_box(solutions).candidate_count();
    }
    assert_ne!(candidates, 0);
    let elapsed = started.elapsed();
    println!(
        "{name}: {iterations} iterations in {elapsed:?} ({:?}/iter), candidates={candidates}",
        elapsed / iterations
    );
}

fn bench_represented_bezier_region_corner_lanes(
    region: &CurveRegion2,
    two_bezier_region: &CurveRegion2,
    iterations: u32,
) {
    let policy = CurveContext::STRICT;
    let next_setback = (s(657).sqrt().expect("positive benchmark radicand") / s(16))
        .expect("nonzero benchmark divisor");
    if corner_lane_enabled("curve_region_line_quadratic_algebraic_chamfer") {
        let started = Instant::now();
        let mut candidates = 0_usize;
        for _ in 0..iterations {
            let solutions = black_box(region)
                .chamfer_loop_vertex_by_setbacks(
                    0,
                    1,
                    s(1),
                    s(1),
                    CurveCornerMode2::TrimOnly,
                    &policy,
                )
                .expect("algebraic Bezier region chamfer must retain exact carriers")
                .into_value();
            candidates += black_box(solutions).candidate_count();
        }
        assert_ne!(candidates, 0);
        let elapsed = started.elapsed();
        println!(
            "curve_region_line_quadratic_algebraic_chamfer: {iterations} iterations in {elapsed:?} ({:?}/iter), candidates={candidates}",
            elapsed / iterations
        );
    }
    if corner_lane_enabled("curve_region_two_quadratic_algebraic_chamfer") {
        let started = Instant::now();
        let mut candidates = 0_usize;
        for _ in 0..iterations {
            let solutions = black_box(two_bezier_region)
                .chamfer_loop_vertex_by_setbacks(
                    0,
                    1,
                    s(1),
                    s(1),
                    CurveCornerMode2::TrimOnly,
                    &policy,
                )
                .expect("two algebraic Bezier cuts must retain one exact chord")
                .into_value();
            candidates += black_box(solutions).candidate_count();
        }
        assert_ne!(candidates, 0);
        let elapsed = started.elapsed();
        println!(
            "curve_region_two_quadratic_algebraic_chamfer: {iterations} iterations in {elapsed:?} ({:?}/iter), candidates={candidates}",
            elapsed / iterations
        );
    }
    #[cfg(feature = "predicates")]
    if corner_lane_enabled("curve_region_line_quadratic_algebraic_regularize") {
        let CurveCornerSolutions2::Unique(chamfered) = region
            .chamfer_loop_vertex_by_setbacks(0, 1, s(1), s(1), CurveCornerMode2::TrimOnly, &policy)
            .expect("algebraic Bezier region chamfer must retain exact carriers")
            .into_value()
        else {
            panic!("algebraic Bezier region chamfer must be unique");
        };
        let started = Instant::now();
        let mut fragments = 0_usize;
        for _ in 0..iterations {
            let regularized = black_box(&chamfered)
                .regularized_region(&policy)
                .expect("one-field algebraic chamfer regularization must remain exact")
                .into_value();
            fragments += black_box(&regularized).boundary_loops()[0]
                .fragments()
                .len();
        }
        assert_ne!(fragments, 0);
        let elapsed = started.elapsed();
        println!(
            "curve_region_line_quadratic_algebraic_regularize: {iterations} iterations in {elapsed:?} ({:?}/iter), fragments={fragments}",
            elapsed / iterations
        );
    }
    if corner_lane_enabled("curve_region_two_quadratic_algebraic_disjoint_boolean") {
        let CurveCornerSolutions2::Unique(chamfered) = two_bezier_region
            .chamfer_loop_vertex_by_setbacks(0, 1, s(1), s(1), CurveCornerMode2::TrimOnly, &policy)
            .expect("two algebraic Bezier cuts must retain one exact chord")
            .into_value()
        else {
            panic!("two-Bezier algebraic region chamfer must be unique");
        };
        let distant = CurveRegion2::try_from_native_material_contours(
            vec![rectangle(10, 10, 12, 12)],
            &policy,
        )
        .expect("distant Boolean benchmark region must remain exact")
        .into_value();
        let started = Instant::now();
        let mut loops = 0_usize;
        for _ in 0..iterations {
            let results = black_box(&chamfered)
                .boolean_regions(black_box(&distant), &policy)
                .expect("disjoint algebraic chamfer Boolean must remain exact")
                .into_value();
            loops += black_box(&results).union().boundary_loops().len();
        }
        assert_ne!(loops, 0);
        let elapsed = started.elapsed();
        println!(
            "curve_region_two_quadratic_algebraic_disjoint_boolean: {iterations} iterations in {elapsed:?} ({:?}/iter), loops={loops}",
            elapsed / iterations
        );
    }
    if corner_lane_enabled("curve_region_line_quadratic_design_chamfer") {
        let started = Instant::now();
        let mut candidates = 0_usize;
        for _ in 0..iterations {
            let solutions = black_box(region)
                .chamfer_loop_vertex_by_setbacks(
                    0,
                    1,
                    s(1),
                    next_setback.clone(),
                    CurveCornerMode2::TrimOnly,
                    &policy,
                )
                .expect("represented Bezier region chamfer must remain exact")
                .into_value();
            candidates += black_box(solutions).candidate_count();
        }
        assert_ne!(candidates, 0);
        let elapsed = started.elapsed();
        println!(
            "curve_region_line_quadratic_design_chamfer: {iterations} iterations in {elapsed:?} ({:?}/iter), candidates={candidates}",
            elapsed / iterations
        );
    }
    if corner_lane_enabled("curve_region_line_quadratic_design_fillet") {
        let started = Instant::now();
        let mut candidates = 0_usize;
        for _ in 0..iterations {
            let solutions = black_box(region)
                .fillet_loop_vertex_by_radius(0, 1, q(15, 4), CurveCornerMode2::TrimOnly, &policy)
                .expect("represented Bezier region fillet must remain exact")
                .into_value();
            candidates += black_box(solutions).candidate_count();
        }
        assert_ne!(candidates, 0);
        let elapsed = started.elapsed();
        println!(
            "curve_region_line_quadratic_design_fillet: {iterations} iterations in {elapsed:?} ({:?}/iter), candidates={candidates}",
            elapsed / iterations
        );
    }
}

fn bench_represented_bezier_corner_solvers(iterations: u32) -> CurveResult<()> {
    let quadratic = QuadraticBezier2::new(p(0, 0), p(0, 1), p(1, 2));
    let quadratic_controls = quadratic
        .control_points()
        .into_iter()
        .cloned()
        .collect::<Vec<_>>();
    let quadratic_knots = vec![s(2), s(2), s(2), s(5), s(5), s(5)];
    let rational = RationalBezier2::try_new(quadratic_controls.clone(), vec![s(1), s(1), s(1)])?
        .elevated_to_degree(5)
        .expect("benchmark degree elevation must remain exact");
    let line_quadratic = CurvePath2::try_new(vec![
        Curve2::from(line(-4, 0, 0, 0)),
        Curve2::from(quadratic.clone()),
    ])
    .expect("line/quadratic benchmark path must remain exact");
    let line_rational = CurvePath2::try_new(vec![
        Curve2::from(line(-4, 0, 0, 0)),
        Curve2::from(rational),
    ])
    .expect("line/rational benchmark path must remain exact");
    let line_polynomial_spline = CurvePath2::try_new(vec![
        Curve2::from(line(-4, 0, 0, 0)),
        Curve2::try_polynomial_bspline(
            2,
            quadratic_controls.clone(),
            quadratic_knots.clone(),
            &CurveContext::STRICT,
        )
        .expect("polynomial spline benchmark carrier must remain exact")
        .into_value(),
    ])
    .expect("line/polynomial-spline benchmark path must remain exact");
    let line_nurbs = CurvePath2::try_new(vec![
        Curve2::from(line(-4, 0, 0, 0)),
        Curve2::try_nurbs(
            2,
            quadratic_controls,
            vec![s(1), s(1), s(1)],
            quadratic_knots,
            &CurveContext::STRICT,
        )
        .expect("NURBS benchmark carrier must remain exact")
        .into_value(),
    ])
    .expect("line/NURBS benchmark path must remain exact");
    let cubic_ph_pair = CurvePath2::try_new(vec![
        Curve2::from(hypercurve::CubicBezier2::new(
            p(-4, 0),
            Point2::new(-q(8, 3), s(0)),
            Point2::new(-q(4, 3), s(0)),
            p(0, 0),
        )),
        Curve2::from(hypercurve::CubicBezier2::new(
            p(0, 0),
            Point2::new(s(0), q(4, 3)),
            Point2::new(s(0), q(8, 3)),
            p(0, 4),
        )),
    ])
    .expect("exact PH cubic benchmark path must remain exact");
    let spline_ph_pair = CurvePath2::try_new(vec![
        Curve2::try_polynomial_bspline(
            3,
            vec![
                p(-4, 0),
                Point2::new(-q(8, 3), s(0)),
                Point2::new(-q(4, 3), s(0)),
                p(0, 0),
            ],
            vec![s(2), s(2), s(2), s(2), s(5), s(5), s(5), s(5)],
            &CurveContext::STRICT,
        )
        .expect("polynomial PH spline benchmark carrier must remain exact")
        .into_value(),
        Curve2::try_nurbs(
            3,
            vec![
                p(0, 0),
                Point2::new(s(0), q(4, 3)),
                Point2::new(s(0), q(8, 3)),
                p(0, 4),
            ],
            vec![s(1), s(1), s(1), s(1)],
            vec![s(7), s(7), s(7), s(7), s(11), s(11), s(11), s(11)],
            &CurveContext::STRICT,
        )
        .expect("NURBS PH spline benchmark carrier must remain exact")
        .into_value(),
    ])
    .expect("exact PH spline benchmark path must remain exact");
    let source_center = Point2::new(-q(7, 16), q(207, 512));
    let source_start = Point2::new(-q(7, 16), -q(49, 256));
    let arc_quadratic = CurvePath2::try_new(vec![
        Curve2::from(CircularArc2::try_from_center(
            source_start,
            p(0, 0),
            source_center,
            true,
        )?),
        Curve2::from(quadratic.clone()),
    ])
    .expect("arc/quadratic benchmark path must remain exact");
    let region_path = CurvePath2::try_new(vec![
        Curve2::from(line(-4, 0, 0, 0)),
        Curve2::from(quadratic),
        Curve2::from(line(1, 2, -4, 2)),
        Curve2::from(line(-4, 2, -4, 0)),
    ])
    .expect("region benchmark path must remain exact");
    let region = CurveRegion2::try_from_boundary_paths(&[region_path], &CurveContext::STRICT)
        .expect("represented Bezier benchmark region must remain exact")
        .into_value();
    let two_bezier_region_path = CurvePath2::try_new(vec![
        Curve2::from(QuadraticBezier2::new(p(-1, 2), p(0, 1), p(0, 0))),
        Curve2::from(QuadraticBezier2::new(p(0, 0), p(0, 1), p(1, 2))),
        Curve2::from(line(1, 2, -1, 2)),
    ])
    .expect("two-Bezier region benchmark path must remain exact");
    let two_bezier_region =
        CurveRegion2::try_from_boundary_paths(&[two_bezier_region_path], &CurveContext::STRICT)
            .expect("two-Bezier benchmark region must remain exact")
            .into_value();
    let next_setback = (s(657).sqrt()? / s(16))?;

    bench_represented_bezier_chamfer_lane(
        "curve_path_line_quadratic_design_chamfer",
        &line_quadratic,
        &s(1),
        &next_setback,
        iterations,
    );
    bench_represented_bezier_fillet_lane(
        "curve_path_line_quadratic_design_fillet",
        &line_quadratic,
        &q(15, 4),
        iterations,
    );
    bench_represented_bezier_chamfer_lane(
        "curve_path_line_degree5_rational_design_chamfer",
        &line_rational,
        &s(1),
        &next_setback,
        iterations,
    );
    bench_represented_bezier_fillet_lane(
        "curve_path_line_degree5_rational_design_fillet",
        &line_rational,
        &q(15, 4),
        iterations,
    );
    bench_represented_bezier_chamfer_lane(
        "curve_path_line_polynomial_spline_design_chamfer",
        &line_polynomial_spline,
        &s(1),
        &next_setback,
        iterations,
    );
    bench_represented_bezier_fillet_lane(
        "curve_path_line_polynomial_spline_design_fillet",
        &line_polynomial_spline,
        &q(15, 4),
        iterations,
    );
    bench_represented_bezier_chamfer_lane(
        "curve_path_line_nurbs_design_chamfer",
        &line_nurbs,
        &s(1),
        &next_setback,
        iterations,
    );
    bench_represented_bezier_fillet_lane(
        "curve_path_line_nurbs_design_fillet",
        &line_nurbs,
        &q(15, 4),
        iterations,
    );
    bench_represented_bezier_fillet_lane(
        "curve_path_arc_quadratic_design_fillet",
        &arc_quadratic,
        &q(5, 4),
        iterations,
    );
    bench_represented_bezier_fillet_lane(
        "curve_path_cubic_ph_pair_design_fillet",
        &cubic_ph_pair,
        &s(1),
        iterations,
    );
    bench_represented_bezier_fillet_lane(
        "curve_path_polynomial_spline_nurbs_ph_pair_design_fillet",
        &spline_ph_pair,
        &s(1),
        iterations,
    );
    bench_represented_bezier_region_corner_lanes(&region, &two_bezier_region, iterations);
    Ok(())
}

fn bench_arc_fillet(iterations: u32) -> CurveResult<()> {
    let previous_arc = CircularArc2::try_from_center(
        Point2::new(s(3), q(13, 3)),
        p(5, 3),
        Point2::new(s(3), q(13, 6)),
        false,
    )?;
    let next_arc = CircularArc2::try_from_center(
        p(5, 3),
        Point2::new(q(9, 2), q(5, 2)),
        Point2::new(q(13, 2), s(1)),
        true,
    )?;
    let policy = CurveContext::STRICT;
    let Classification::Decided(previous_param) = previous_arc.sweep_fraction(&p(3, 0), &policy)?
    else {
        panic!("previous arc fillet parameter must be decided");
    };
    let Classification::Decided(next_param) = next_arc.sweep_fraction(&p(4, 1), &policy)? else {
        panic!("next arc fillet parameter must be decided");
    };
    let curve = CurveString2::try_new(vec![Segment2::Arc(previous_arc), Segment2::Arc(next_arc)])?;
    let started = Instant::now();
    let mut total_segments = 0_usize;

    for _ in 0..iterations {
        let result = curve.fillet_vertex_by_parameters(
            1,
            previous_param.clone(),
            next_param.clone(),
            &p(3, 1),
            false,
            &policy,
        )?;
        let Classification::Decided(filleted) = result else {
            panic!("arc-arc fillet benchmark became uncertain");
        };
        total_segments += black_box(filleted.len());
    }

    let elapsed = started.elapsed();
    println!(
        "curve_string_arc_fillet: {iterations} iterations in {elapsed:?} ({:?}/iter), total segments={total_segments}",
        elapsed / iterations
    );
    Ok(())
}

fn bench_arc_extension(iterations: u32) -> CurveResult<()> {
    let curve = CurveString2::try_new(vec![Segment2::Arc(CircularArc2::try_from_center(
        p(1, 0),
        p(0, 1),
        p(0, 0),
        false,
    )?)])?;
    let policy = CurveContext::STRICT;
    let started = Instant::now();
    let mut total_segments = 0_usize;

    for _ in 0..iterations {
        let result =
            curve.extend_endpoint_to_point(CurveStringEndpoint2::End, p(-1, 0), &policy)?;
        let Classification::Decided(extended) = result else {
            panic!("arc extension benchmark became uncertain");
        };
        total_segments += black_box(extended.len());
    }

    let elapsed = started.elapsed();
    println!(
        "curve_string_arc_extension: {iterations} iterations in {elapsed:?} ({:?}/iter), total segments={total_segments}",
        elapsed / iterations
    );
    Ok(())
}

fn bench_curve_string_line_merge_evidence(iterations: u32) -> CurveResult<()> {
    let curve = CurveString2::try_new(vec![
        line_segment(0, 0, 2, 0),
        line_segment(2, 0, 5, 0),
        line_segment(5, 0, 5, 3),
        line_segment(5, 3, 5, 7),
    ])?;
    let policy = CurveContext::STRICT;
    let started = Instant::now();
    let mut total_spans = 0_usize;

    for _ in 0..iterations {
        let result = curve.merge_adjacent_collinear_lines(&policy)?;
        let Classification::Decided(merged) = result else {
            panic!("curve string line merge benchmark became uncertain");
        };
        total_spans += black_box(merged.len());
    }

    let elapsed = started.elapsed();
    println!(
        "curve_string_line_merge_evidence: {iterations} iterations in {elapsed:?} ({:?}/iter), total spans={total_spans}",
        elapsed / iterations
    );
    Ok(())
}

fn bench_curve_string_reversed_duplicate_evidence(iterations: u32) -> CurveResult<()> {
    let curve = CurveString2::try_new(vec![
        line_segment(0, 0, 2, 0),
        line_segment(2, 0, 4, 0),
        line_segment(4, 0, 2, 0),
        line_segment(2, 0, 2, 2),
    ])?;
    let started = Instant::now();
    let mut total_retained = 0_usize;

    for _ in 0..iterations {
        let result = curve.remove_adjacent_reversed_duplicates()?;
        let Classification::Decided(deduped) = result else {
            panic!("curve string reversed duplicate benchmark became uncertain");
        };
        total_retained += black_box(deduped.len());
    }

    let elapsed = started.elapsed();
    println!(
        "curve_string_reversed_duplicate_evidence: {iterations} iterations in {elapsed:?} ({:?}/iter), total retained={total_retained}",
        elapsed / iterations
    );
    Ok(())
}

fn bench_curve_string_pair_link_evidence(iterations: u32) -> CurveResult<()> {
    let first = CurveString2::try_new(vec![line_segment(0, 0, 1, 0)])?;
    let second = CurveString2::try_new(vec![line_segment(1, 0, 2, 0)])?;
    let policy = CurveContext::STRICT;
    let started = Instant::now();
    let mut total_segments = 0_usize;

    for _ in 0..iterations {
        let Classification::Decided(Some(linked)) =
            first.link_connected_endpoints(&second, &policy)?
        else {
            panic!("pair link benchmark should materialize");
        };
        total_segments += black_box(linked.len());
    }

    let elapsed = started.elapsed();
    println!(
        "curve_string_pair_link_evidence: {iterations} iterations in {elapsed:?} ({:?}/iter), total segments={total_segments}",
        elapsed / iterations
    );
    Ok(())
}

fn bench_curve_string_ordered_link_evidence(iterations: u32) -> CurveResult<()> {
    let curves = vec![
        CurveString2::try_new(vec![line_segment(0, 0, 1, 0)])?,
        CurveString2::try_new(vec![line_segment(1, 0, 2, 0)])?,
        CurveString2::try_new(vec![line_segment(2, 0, 3, 0)])?,
    ];
    let policy = CurveContext::STRICT;
    let started = Instant::now();
    let mut total_segments = 0_usize;

    for _ in 0..iterations {
        let result = CurveString2::link_ordered_connected_endpoints(curves.clone(), &policy)?;
        let linked = expect_decided(result, "ordered link benchmark should materialize");
        total_segments += black_box(linked.len());
    }

    let elapsed = started.elapsed();
    println!(
        "curve_string_ordered_link: {iterations} iterations in {elapsed:?} ({:?}/iter), total segments={total_segments}",
        elapsed / iterations
    );
    Ok(())
}

fn bench_curve_string_connect_evidence(iterations: u32) -> CurveResult<()> {
    let first = CurveString2::try_new(vec![line_segment(0, 0, 1, 0)])?;
    let second = CurveString2::try_new(vec![line_segment(3, 1, 4, 1)])?;
    let policy = CurveContext::STRICT;
    let started = Instant::now();
    let mut total_segments = 0_usize;

    for _ in 0..iterations {
        let result = first.connect_end_to_start_with_line(&second, &policy)?;
        let connected = expect_decided(result, "connect benchmark should materialize");
        total_segments += black_box(connected.len());
    }

    let elapsed = started.elapsed();
    println!(
        "curve_string_connect_evidence: {iterations} iterations in {elapsed:?} ({:?}/iter), total segments={total_segments}",
        elapsed / iterations
    );
    Ok(())
}

fn bench_boundary_contour_region_build(iterations: u32) -> CurveResult<()> {
    let material = rectangle(0, 0, 10, 10);
    let hole = rectangle(2, 2, 8, 8);
    let island = rectangle(4, 4, 6, 6);
    let policy = CurveContext::STRICT;
    let started = Instant::now();
    let mut total_roles = 0_usize;

    for _ in 0..iterations {
        let Classification::Decided(region) = LineArcRegion2::from_boundary_contours(
            vec![material.clone(), hole.clone(), island.clone()],
            &policy,
        )?
        else {
            panic!("boundary contour region build benchmark became uncertain");
        };
        total_roles += black_box(region.material_contours().len() + region.hole_contours().len());
    }

    let elapsed = started.elapsed();
    println!(
        "boundary_contour_region_build: {iterations} iterations in {elapsed:?} ({:?}/iter), total roles={total_roles}",
        elapsed / iterations
    );
    Ok(())
}

fn bench_unordered_line_segment_region_build(iterations: u32) -> CurveResult<()> {
    let lines = vec![
        line(0, 0, 10, 0),
        line(0, 10, 10, 10),
        line(0, 0, 0, 10),
        line(10, 0, 10, 10),
    ];
    let policy = CurveContext::STRICT;
    let started = Instant::now();
    let mut total_request_sources = 0_usize;
    let mut total_evidence_counts = 0_usize;
    let mut total_retained_outputs = 0_usize;
    let mut total_segments = 0_usize;
    let mut total_endpoint_checks = 0_usize;

    for _ in 0..iterations {
        let result = LineArcRegion2::arrange_unordered_line_segments_borrowed(
            &lines,
            FillRule::NonZero,
            &policy,
        )?;
        if !result.status().unwrap().is_native_exact() || result.region().is_none() {
            panic!("unordered line segment region build benchmark became non-native");
        }
        total_request_sources += black_box(result.source_segment_count());
        total_evidence_counts += black_box(result.decided_source_segment_aabb_count());
        total_evidence_counts += black_box(result.source_endpoint_bucket_count());
        total_evidence_counts += black_box(result.split_schedule_candidate_pair_count());
        total_evidence_counts += black_box(result.split_schedule_decided_disjoint_pair_count());
        total_retained_outputs += black_box(result.output_boundary_segment_count().unwrap_or(0));
        total_retained_outputs += black_box(result.output_contour_count().unwrap_or(0));
        total_segments += black_box(result.split_output_segment_count().unwrap_or_default());
        total_segments += black_box(result.output_boundary_segment_count().unwrap_or_default());
        total_segments += black_box(result.split_skipped_aabb_pair_count().unwrap_or_default());
        total_endpoint_checks += black_box(
            result
                .attempted_endpoint_connection_count()
                .unwrap_or_default(),
        );
        total_endpoint_checks += black_box(result.endpoint_graph_endpoint_count().unwrap_or(0));
        total_endpoint_checks +=
            black_box(result.endpoint_graph_structural_bucket_count().unwrap_or(0));
        total_endpoint_checks += black_box(
            result
                .endpoint_graph_max_structural_bucket_size()
                .unwrap_or(0),
        );
    }

    let elapsed = started.elapsed();
    println!(
        "unordered_line_segment_region_build: {iterations} iterations in {elapsed:?} ({:?}/iter), request sources={total_request_sources}, evidence counts={total_evidence_counts}, retained outputs={total_retained_outputs}, total segments={total_segments}, endpoint checks={total_endpoint_checks}",
        elapsed / iterations
    );
    Ok(())
}

fn bench_unordered_native_segment_region_build(iterations: u32) -> CurveResult<()> {
    let segments = vec![
        Segment2::Line(line(4, 0, 0, 0)),
        Segment2::Arc(CircularArc2::from_bulge(p(0, 0), p(4, 0), s(1))?),
    ];
    let policy = CurveContext::STRICT;
    let started = Instant::now();
    let mut total_request_sources = 0_usize;
    let mut total_evidence_counts = 0_usize;
    let mut total_retained_outputs = 0_usize;
    let mut total_segments = 0_usize;
    let mut total_endpoint_checks = 0_usize;

    for _ in 0..iterations {
        let result = LineArcRegion2::arrange_unordered_segments_borrowed(
            &segments,
            FillRule::NonZero,
            &policy,
        )?;
        if !result.status().unwrap().is_native_exact() || result.region().is_none() {
            panic!("unordered native segment region build benchmark became non-native");
        }
        total_request_sources += black_box(result.source_segment_count());
        total_evidence_counts += black_box(result.decided_source_segment_aabb_count());
        total_evidence_counts += black_box(result.source_endpoint_bucket_count());
        total_evidence_counts += black_box(result.split_schedule_candidate_pair_count());
        total_evidence_counts += black_box(result.split_schedule_predicate_candidate_pair_count());
        total_retained_outputs += black_box(result.output_boundary_segment_count().unwrap_or(0));
        total_retained_outputs += black_box(result.output_contour_count().unwrap_or(0));
        total_segments += black_box(result.split_output_segment_count().unwrap_or_default());
        total_segments += black_box(result.output_boundary_segment_count().unwrap_or_default());
        total_endpoint_checks += black_box(
            result
                .attempted_endpoint_connection_count()
                .unwrap_or_default(),
        );
        total_endpoint_checks += black_box(result.endpoint_graph_endpoint_count().unwrap_or(0));
        total_endpoint_checks +=
            black_box(result.endpoint_graph_structural_bucket_count().unwrap_or(0));
    }

    let elapsed = started.elapsed();
    println!(
        "unordered_native_segment_region_build: {iterations} iterations in {elapsed:?} ({:?}/iter), request sources={total_request_sources}, evidence counts={total_evidence_counts}, retained outputs={total_retained_outputs}, total segments={total_segments}, endpoint checks={total_endpoint_checks}",
        elapsed / iterations
    );
    Ok(())
}

fn bench_region_arrangement_immediate_replay(iterations: u32) -> CurveResult<()> {
    let lines = vec![
        line(0, 0, 10, 0),
        line(10, 0, 10, 10),
        line(10, 10, 0, 10),
        line(0, 10, 0, 0),
    ];
    let policy = CurveContext::STRICT;
    let result =
        LineArcRegion2::arrange_unordered_line_segments(lines, FillRule::NonZero, &policy)?;
    let started = Instant::now();
    let mut checksum = 0_usize;

    for _ in 0..iterations {
        let immediate = black_box(&result);
        checksum = checksum.wrapping_add(black_box(immediate.source_segment_count()));
        checksum = checksum.wrapping_add(black_box(immediate.output_segment_count().unwrap_or(0)));
    }

    let elapsed = started.elapsed();
    println!(
        "region_arrangement_immediate_replay: {iterations} iterations in {elapsed:?} ({:?}/iter), checksum={checksum}",
        elapsed / iterations
    );
    Ok(())
}

fn bench_contour_line_merge_evidence(iterations: u32) -> CurveResult<()> {
    let contour = Contour2::from_bulge_vertices(&[
        vertex(0, 0, 0),
        vertex(2, 0, 0),
        vertex(5, 0, 0),
        vertex(5, 3, 0),
        vertex(5, 7, 0),
        vertex(0, 7, 0),
    ])?;
    let policy = CurveContext::STRICT;
    let started = Instant::now();
    let mut total_spans = 0_usize;

    for _ in 0..iterations {
        let result = contour.merge_adjacent_collinear_lines(&policy)?;
        let Classification::Decided(merged) = result else {
            panic!("contour line merge benchmark became uncertain");
        };
        total_spans += black_box(merged.len());
    }

    let elapsed = started.elapsed();
    println!(
        "contour_line_merge_evidence: {iterations} iterations in {elapsed:?} ({:?}/iter), total spans={total_spans}",
        elapsed / iterations
    );
    Ok(())
}

fn bench_region_boolean(iterations: u32) -> CurveResult<()> {
    let first = LineArcRegion2::from_material_contours(vec![rectangle(0, 0, 4, 4)]);
    let second = LineArcRegion2::from_material_contours(vec![rectangle(2, -1, 6, 3)]);
    let policy = CurveContext::STRICT;
    let started = Instant::now();
    let mut total_boundary_contours = 0_usize;

    for _ in 0..iterations {
        let Classification::Decided(result) =
            first.boolean_region(&second, BooleanOp::Union, FillRule::NonZero, &policy)?
        else {
            panic!("region boolean benchmark became uncertain");
        };
        total_boundary_contours +=
            black_box(result.material_contours().len() + result.hole_contours().len());
    }

    let elapsed = started.elapsed();
    println!(
        "region_boolean: {iterations} iterations in {elapsed:?} ({:?}/iter), total boundary contours={total_boundary_contours}",
        elapsed / iterations
    );
    Ok(())
}

fn bench_contour_signed_area_cache(iterations: u32) -> CurveResult<()> {
    let contour = subdivided_rectangle(64);
    let clone = contour.clone();
    let cold_started = Instant::now();
    let cold_area = contour.signed_area()?;
    let cold_elapsed = cold_started.elapsed();
    let replay_started = Instant::now();
    let mut replay_count = 0_usize;

    for _ in 0..iterations {
        replay_count += black_box(clone.signed_area()?.is_some()) as usize;
    }

    let replay_elapsed = replay_started.elapsed();
    println!(
        "contour_signed_area_cache: cold={cold_elapsed:?}, {iterations} retained clone replays in {replay_elapsed:?} ({:?}/replay), area={}, replay count={replay_count}",
        replay_elapsed / iterations,
        cold_area.is_some(),
    );
    Ok(())
}

fn bench_curve_region_mutations(iterations: u32) -> CurveResult<()> {
    let policy = CurveContext::STRICT;
    let region =
        CurveRegion2::try_from_native_material_contours(vec![rectangle(0, 0, 4, 4)], &policy)
            .expect("benchmark rectangle must promote")
            .into_value();

    if corner_lane_enabled("curve_region_affine_transform") {
        let started = Instant::now();
        let mut transformed_loops = 0_usize;
        for _ in 0..iterations {
            let transformed = black_box(&region)
                .transform_affine(
                    &Real::zero(),
                    &-Real::one(),
                    &Real::one(),
                    &Real::zero(),
                    &Real::zero(),
                    &Real::zero(),
                    &policy,
                )
                .expect("benchmark transform must remain exact")
                .into_value();
            transformed_loops += black_box(transformed).boundary_loops().len();
        }
        let elapsed = started.elapsed();
        println!(
            "curve_region_affine_transform: {iterations} iterations in {elapsed:?} ({:?}/iter), loops={transformed_loops}",
            elapsed / iterations
        );
    }

    if corner_lane_enabled("curve_region_parameter_chamfer") {
        let started = Instant::now();
        let mut chamfered_loops = 0_usize;
        for _ in 0..iterations {
            let Classification::Decided(chamfered) = black_box(&region)
                .chamfer_loop_vertex_by_parameters(0, 1, q(3, 4), q(1, 4), &policy)
                .expect("benchmark chamfer must remain exact")
                .into_value()
            else {
                panic!("CurveRegion2 line chamfer benchmark became uncertain");
            };
            chamfered_loops += black_box(chamfered).boundary_loops().len();
        }
        let elapsed = started.elapsed();
        println!(
            "curve_region_parameter_chamfer: {iterations} iterations in {elapsed:?} ({:?}/iter), loops={chamfered_loops}",
            elapsed / iterations
        );
    }

    if corner_lane_enabled("curve_region_parameter_fillet") {
        let started = Instant::now();
        let mut filleted_loops = 0_usize;
        for _ in 0..iterations {
            let Classification::Decided(filleted) = black_box(&region)
                .fillet_loop_vertex_by_parameters(0, 1, q(3, 4), q(1, 4), &p(3, 1), false, &policy)
                .expect("benchmark fillet must remain exact")
                .into_value()
            else {
                panic!("CurveRegion2 line fillet benchmark became uncertain");
            };
            filleted_loops += black_box(filleted).boundary_loops().len();
        }
        let elapsed = started.elapsed();
        println!(
            "curve_region_parameter_fillet: {iterations} iterations in {elapsed:?} ({:?}/iter), loops={filleted_loops}",
            elapsed / iterations
        );
    }

    if corner_lane_enabled("curve_region_design_chamfer") {
        let started = Instant::now();
        let mut chamfered_loops = 0_usize;
        for _ in 0..iterations {
            let CurveCornerSolutions2::Unique(chamfered) = black_box(&region)
                .chamfer_loop_vertex_by_setbacks(
                    0,
                    1,
                    s(1),
                    s(1),
                    CurveCornerMode2::TrimOnly,
                    &policy,
                )
                .expect("benchmark design-parameter chamfer must remain exact")
                .into_value()
            else {
                panic!("CurveRegion2 design-parameter chamfer benchmark must be unique");
            };
            chamfered_loops += black_box(chamfered).boundary_loops().len();
        }
        let elapsed = started.elapsed();
        println!(
            "curve_region_design_chamfer: {iterations} iterations in {elapsed:?} ({:?}/iter), loops={chamfered_loops}",
            elapsed / iterations
        );
    }

    if corner_lane_enabled("curve_region_design_fillet") {
        let started = Instant::now();
        let mut filleted_loops = 0_usize;
        for _ in 0..iterations {
            let CurveCornerSolutions2::Unique(filleted) = black_box(&region)
                .fillet_loop_vertex_by_radius(0, 1, s(1), CurveCornerMode2::TrimOnly, &policy)
                .expect("benchmark design-parameter fillet must remain exact")
                .into_value()
            else {
                panic!("CurveRegion2 design-parameter fillet benchmark must be unique");
            };
            filleted_loops += black_box(filleted).boundary_loops().len();
        }
        let elapsed = started.elapsed();
        println!(
            "curve_region_design_fillet: {iterations} iterations in {elapsed:?} ({:?}/iter), loops={filleted_loops}",
            elapsed / iterations
        );
    }
    Ok(())
}

fn bench_higher_order_curve_edits(iterations: u32) {
    let policy = CurveContext::STRICT;
    let path = higher_order_fillet_path();
    let region = CurveRegion2::try_from_boundary_paths_with_loop_semantics(
        std::slice::from_ref(&path),
        &[CurveRegionLoopRole::Material],
        &[FillRule::NonZero],
        &policy,
    )
    .expect("higher-order editing benchmark region must promote")
    .into_value();

    let started = Instant::now();
    let mut path_chamfer_curves = 0_usize;
    for _ in 0..iterations {
        let chamfered = path
            .chamfer_vertex_by_parameters(1, q(3, 4), q(1, 2), &policy)
            .expect("higher-order path chamfer must remain exact")
            .into_value();
        path_chamfer_curves += black_box(chamfered.curves().len());
    }
    let elapsed = started.elapsed();
    println!(
        "curve_path_higher_order_chamfer: {iterations} iterations in {elapsed:?} ({:?}/iter), curves={path_chamfer_curves}",
        elapsed / iterations
    );

    let started = Instant::now();
    let mut path_fillet_curves = 0_usize;
    for _ in 0..iterations {
        let filleted = path
            .fillet_vertex_by_parameters(1, q(3, 4), q(1, 2), &p(3, 1), false, &policy)
            .expect("higher-order path fillet must remain exact")
            .into_value();
        path_fillet_curves += black_box(filleted.curves().len());
    }
    let elapsed = started.elapsed();
    println!(
        "curve_path_higher_order_fillet: {iterations} iterations in {elapsed:?} ({:?}/iter), curves={path_fillet_curves}",
        elapsed / iterations
    );

    let started = Instant::now();
    let mut region_chamfer_loops = 0_usize;
    for _ in 0..iterations {
        let Classification::Decided(chamfered) = region
            .chamfer_loop_vertex_by_parameters(0, 1, q(3, 4), q(1, 2), &policy)
            .expect("higher-order region chamfer must remain exact")
            .into_value()
        else {
            panic!("higher-order region chamfer benchmark became uncertain");
        };
        region_chamfer_loops += black_box(chamfered.boundary_loops().len());
    }
    let elapsed = started.elapsed();
    println!(
        "curve_region_higher_order_chamfer: {iterations} iterations in {elapsed:?} ({:?}/iter), loops={region_chamfer_loops}",
        elapsed / iterations
    );

    let started = Instant::now();
    let mut region_fillet_loops = 0_usize;
    for _ in 0..iterations {
        let Classification::Decided(filleted) = region
            .fillet_loop_vertex_by_parameters(0, 1, q(3, 4), q(1, 2), &p(3, 1), false, &policy)
            .expect("higher-order region fillet must remain exact")
            .into_value()
        else {
            panic!("higher-order region fillet benchmark became uncertain");
        };
        region_fillet_loops += black_box(filleted.boundary_loops().len());
    }
    let elapsed = started.elapsed();
    println!(
        "curve_region_higher_order_fillet: {iterations} iterations in {elapsed:?} ({:?}/iter), loops={region_fillet_loops}",
        elapsed / iterations
    );
}

fn main() -> CurveResult<()> {
    let iterations = benchmark_iterations();
    if let Some(selection) = std::env::var_os("HYPERCURVE_EDIT_BENCH") {
        if selection == "higher-order" {
            bench_higher_order_curve_edits(iterations);
            return Ok(());
        }
        if selection == "corner-solver" {
            bench_line_curve_corner_solvers(iterations);
            bench_curve_region_mutations(iterations)?;
            return Ok(());
        }
        if selection == "native-arc-chamfer" {
            bench_native_arc_chamfer_solvers(iterations)?;
            return Ok(());
        }
        if selection == "native-arc-fillet" {
            bench_native_arc_fillet_solvers(iterations)?;
            return Ok(());
        }
        if selection == "retained-circle-corner" {
            bench_retained_circle_corner_solvers(iterations)?;
            return Ok(());
        }
        if selection == "represented-bezier-corner" {
            bench_represented_bezier_corner_solvers(iterations)?;
            return Ok(());
        }
    }
    bench_parameter_trim(iterations)?;
    bench_parameter_arc_trim(iterations)?;
    bench_point_arc_trim(iterations)?;
    bench_curve_intersection_trim(iterations)?;
    bench_region_trim(iterations)?;
    bench_line_chamfer(iterations)?;
    bench_arc_chamfer(iterations)?;
    bench_line_fillet(iterations)?;
    bench_arc_fillet(iterations)?;
    bench_line_curve_corner_solvers(iterations);
    bench_native_arc_chamfer_solvers(iterations)?;
    bench_native_arc_fillet_solvers(iterations)?;
    bench_retained_circle_corner_solvers(iterations)?;
    bench_represented_bezier_corner_solvers(iterations)?;
    bench_curve_region_mutations(iterations)?;
    bench_higher_order_curve_edits(iterations);
    bench_arc_extension(iterations)?;
    bench_curve_string_line_merge_evidence(iterations)?;
    bench_curve_string_reversed_duplicate_evidence(iterations)?;
    bench_curve_string_pair_link_evidence(iterations)?;
    bench_curve_string_ordered_link_evidence(iterations)?;
    bench_curve_string_connect_evidence(iterations)?;
    bench_boundary_contour_region_build(1_000)?;
    bench_unordered_line_segment_region_build(1_000)?;
    bench_unordered_native_segment_region_build(1_000)?;
    bench_region_arrangement_immediate_replay(100_000)?;
    bench_contour_line_merge_evidence(1_000)?;
    bench_contour_signed_area_cache(100_000)?;
    bench_region_boolean(1_000)?;
    Ok(())
}
