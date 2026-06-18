use std::hint::black_box;
use std::time::Instant;

use hypercurve::{
    BooleanOp, BulgeVertex2, CircularArc2, Contour2, CurvePolicy, CurveResult, CurveString2,
    CurveStringEndpoint2, CurveStringTrimPoint2, FillRule, LineSeg2, Point2, Real, Region2,
    RegionBooleanQueryPath2, Segment2,
};

fn s(value: i32) -> Real {
    value.into()
}

fn q(numerator: i32, denominator: i32) -> Real {
    (Real::from(numerator) / Real::from(denominator)).unwrap()
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

fn rectangle(xmin: i32, ymin: i32, xmax: i32, ymax: i32) -> Contour2 {
    Contour2::from_bulge_vertices(&[
        vertex(xmin, ymin, 0),
        vertex(xmax, ymin, 0),
        vertex(xmax, ymax, 0),
        vertex(xmin, ymax, 0),
    ])
    .unwrap()
}

fn bench_parameter_trim(iterations: u32) -> CurveResult<()> {
    let curve = CurveString2::try_new(vec![
        line_segment(0, 0, 10, 0),
        line_segment(10, 0, 10, 6),
        line_segment(10, 6, 16, 6),
    ])?;
    let policy = CurvePolicy::certified();
    let started = Instant::now();
    let mut total_segments = 0_usize;

    for _ in 0..iterations {
        let result = curve.trim_between_parameters(
            CurveStringTrimPoint2::new(0, q(1, 5)),
            CurveStringTrimPoint2::new(2, q(1, 2)),
            &policy,
        )?;
        let trimmed = result
            .curve_string()
            .expect("parameter trim benchmark should materialize");
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
    let policy = CurvePolicy::certified();
    let started = Instant::now();
    let mut total_segments = 0_usize;

    for _ in 0..iterations {
        let result = curve.trim_between_points(&p(1, -1), &p(2, 0), &policy)?;
        let trimmed = result
            .curve_string()
            .expect("point-bearing arc trim benchmark should materialize");
        total_segments += black_box(trimmed.len());
    }

    let elapsed = started.elapsed();
    println!(
        "curve_string_point_arc_trim: {iterations} iterations in {elapsed:?} ({:?}/iter), total segments={total_segments}",
        elapsed / iterations
    );
    Ok(())
}

fn bench_curve_intersection_trim(iterations: u32) -> CurveResult<()> {
    let curve = CurveString2::try_new(vec![line_segment(0, 0, 10, 0)])?;
    let start_cutter = CurveString2::try_new(vec![line_segment(2, -1, 2, 1)])?;
    let end_cutter = CurveString2::try_new(vec![line_segment(8, -1, 8, 1)])?;
    let policy = CurvePolicy::certified();
    let started = Instant::now();
    let mut total_segments = 0_usize;

    for _ in 0..iterations {
        let result = curve.trim_between_curve_intersections(&start_cutter, &end_cutter, &policy)?;
        let trimmed = result
            .curve_string()
            .expect("curve-intersection trim benchmark should materialize");
        total_segments += black_box(trimmed.len());
    }

    let elapsed = started.elapsed();
    println!(
        "curve_string_curve_intersection_trim: {iterations} iterations in {elapsed:?} ({:?}/iter), total segments={total_segments}",
        elapsed / iterations
    );
    Ok(())
}

fn bench_prepared_curve_intersection_trim(iterations: u32) -> CurveResult<()> {
    let curve = CurveString2::try_new(vec![line_segment(0, 0, 10, 0)])?;
    let start_cutter = CurveString2::try_new(vec![line_segment(2, -1, 2, 1)])?;
    let end_cutter = CurveString2::try_new(vec![line_segment(8, -1, 8, 1)])?;
    let policy = CurvePolicy::certified();
    let prepared_curve = curve.prepare_topology_queries(&policy);
    let prepared_start = start_cutter.prepare_topology_queries(&policy);
    let prepared_end = end_cutter.prepare_topology_queries(&policy);
    let started = Instant::now();
    let mut total_segments = 0_usize;

    for _ in 0..iterations {
        let result = prepared_curve.trim_between_prepared_curve_intersections(
            &prepared_start,
            &prepared_end,
            &policy,
        )?;
        let trimmed = result
            .curve_string()
            .expect("prepared curve-intersection trim benchmark should materialize");
        total_segments += black_box(trimmed.len());
    }

    let elapsed = started.elapsed();
    println!(
        "prepared_curve_string_curve_intersection_trim: {iterations} iterations in {elapsed:?} ({:?}/iter), total segments={total_segments}",
        elapsed / iterations
    );
    Ok(())
}

fn bench_region_trim(iterations: u32) -> CurveResult<()> {
    let curve = CurveString2::try_new(vec![line_segment(-2, 1, 8, 1)])?;
    let region =
        Region2::from_material_contours(vec![rectangle(0, 0, 2, 2), rectangle(4, 0, 6, 2)]);
    let policy = CurvePolicy::certified();
    let started = Instant::now();
    let mut total_outputs = 0_usize;

    for _ in 0..iterations {
        let result = curve.trim_inside_region(&region, &policy)?;
        if !result.report().status().is_native_exact() {
            panic!("region trim benchmark became non-native");
        }
        total_outputs += black_box(result.curve_strings().len());
    }

    let elapsed = started.elapsed();
    println!(
        "curve_string_region_trim: {iterations} iterations in {elapsed:?} ({:?}/iter), total outputs={total_outputs}",
        elapsed / iterations
    );
    Ok(())
}

fn bench_prepared_region_trim(iterations: u32) -> CurveResult<()> {
    let curve = CurveString2::try_new(vec![line_segment(-2, 1, 8, 1)])?;
    let region =
        Region2::from_material_contours(vec![rectangle(0, 0, 2, 2), rectangle(4, 0, 6, 2)]);
    let policy = CurvePolicy::certified();
    let prepared_curve = curve.prepare_topology_queries(&policy);
    let prepared_region = region.prepare_topology_queries(&policy);
    let started = Instant::now();
    let mut total_outputs = 0_usize;

    for _ in 0..iterations {
        let result = prepared_curve.trim_inside_prepared_region(&prepared_region, &policy)?;
        if !result.report().status().is_native_exact() {
            panic!("prepared region trim benchmark became non-native");
        }
        total_outputs += black_box(result.curve_strings().len());
    }

    let elapsed = started.elapsed();
    println!(
        "prepared_curve_string_region_trim: {iterations} iterations in {elapsed:?} ({:?}/iter), total outputs={total_outputs}",
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
    let policy = CurvePolicy::certified();
    let started = Instant::now();
    let mut total_segments = 0_usize;

    for _ in 0..iterations {
        let result = curve.chamfer_line_line_vertex_by_parameters(1, q(3, 4), q(1, 4), &policy)?;
        let chamfered = result
            .curve_string()
            .expect("line-line chamfer benchmark should materialize");
        total_segments += black_box(chamfered.len());
    }

    let elapsed = started.elapsed();
    println!(
        "curve_string_line_chamfer: {iterations} iterations in {elapsed:?} ({:?}/iter), total segments={total_segments}",
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
    let policy = CurvePolicy::certified();
    let started = Instant::now();
    let mut total_segments = 0_usize;

    for _ in 0..iterations {
        let result = curve.fillet_line_line_vertex_by_points(
            1,
            &p(3, 0),
            &p(4, 1),
            &p(3, 1),
            false,
            &policy,
        )?;
        let filleted = result
            .curve_string()
            .expect("line-line fillet benchmark should materialize");
        total_segments += black_box(filleted.len());
    }

    let elapsed = started.elapsed();
    println!(
        "curve_string_line_fillet: {iterations} iterations in {elapsed:?} ({:?}/iter), total segments={total_segments}",
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
    let policy = CurvePolicy::certified();
    let started = Instant::now();
    let mut total_segments = 0_usize;

    for _ in 0..iterations {
        let result =
            curve.extend_endpoint_to_point(CurveStringEndpoint2::End, p(-1, 0), &policy)?;
        let extended = result
            .curve_string()
            .expect("arc extension benchmark should materialize");
        total_segments += black_box(extended.len());
    }

    let elapsed = started.elapsed();
    println!(
        "curve_string_arc_extension: {iterations} iterations in {elapsed:?} ({:?}/iter), total segments={total_segments}",
        elapsed / iterations
    );
    Ok(())
}

fn bench_boundary_contour_region_build(iterations: u32) -> CurveResult<()> {
    let material = rectangle(0, 0, 10, 10);
    let hole = rectangle(2, 2, 8, 8);
    let island = rectangle(4, 4, 6, 6);
    let policy = CurvePolicy::certified();
    let started = Instant::now();
    let mut total_roles = 0_usize;

    for _ in 0..iterations {
        let result = Region2::from_boundary_contours_with_report(
            vec![material.clone(), hole.clone(), island.clone()],
            &policy,
        )?;
        let report = result.report();
        if !report.status().is_native_exact() {
            panic!("boundary contour region build benchmark became non-native");
        }
        total_roles += black_box(report.role_reports().len());
    }

    let elapsed = started.elapsed();
    println!(
        "boundary_contour_region_build: {iterations} iterations in {elapsed:?} ({:?}/iter), total roles={total_roles}",
        elapsed / iterations
    );
    Ok(())
}

fn bench_region_boolean_report(iterations: u32) -> CurveResult<()> {
    let first = Region2::from_material_contours(vec![rectangle(0, 0, 4, 4)]);
    let second = Region2::from_material_contours(vec![rectangle(2, -1, 6, 3)]);
    let policy = CurvePolicy::certified();
    let started = Instant::now();
    let mut total_boundary_contours = 0_usize;

    for _ in 0..iterations {
        let result = first.boolean_region_with_report(
            &second,
            BooleanOp::Union,
            FillRule::NonZero,
            &policy,
        )?;
        let report = result.report();
        if !report.status().is_native_exact()
            || report.query_path() != RegionBooleanQueryPath2::Direct
            || result.region().is_none()
        {
            panic!("region boolean benchmark became non-native or used the wrong query path");
        }
        total_boundary_contours += black_box(report.boundary_contour_count().unwrap_or_default());
    }

    let elapsed = started.elapsed();
    println!(
        "region_boolean_report: {iterations} iterations in {elapsed:?} ({:?}/iter), total boundary contours={total_boundary_contours}",
        elapsed / iterations
    );
    Ok(())
}

fn bench_prepared_region_boolean_report(iterations: u32) -> CurveResult<()> {
    let first = Region2::from_material_contours(vec![rectangle(0, 0, 4, 4)]);
    let second = Region2::from_material_contours(vec![rectangle(2, -1, 6, 3)]);
    let policy = CurvePolicy::certified();
    let prepared_first = first.prepare_topology_queries(&policy);
    let prepared_second = second.prepare_topology_queries(&policy);
    let started = Instant::now();
    let mut total_boundary_contours = 0_usize;

    for _ in 0..iterations {
        let result = prepared_first.boolean_region_with_report(
            &prepared_second,
            BooleanOp::Union,
            FillRule::NonZero,
            &policy,
        )?;
        let report = result.report();
        if !report.status().is_native_exact()
            || report.query_path() != RegionBooleanQueryPath2::Prepared
            || result.region().is_none()
        {
            panic!(
                "prepared region boolean benchmark became non-native or used the wrong query path"
            );
        }
        total_boundary_contours += black_box(report.boundary_contour_count().unwrap_or_default());
    }

    let elapsed = started.elapsed();
    println!(
        "prepared_region_boolean_report: {iterations} iterations in {elapsed:?} ({:?}/iter), total boundary contours={total_boundary_contours}",
        elapsed / iterations
    );
    Ok(())
}

fn main() -> CurveResult<()> {
    let iterations = 10_000;
    bench_parameter_trim(iterations)?;
    bench_point_arc_trim(iterations)?;
    bench_curve_intersection_trim(iterations)?;
    bench_prepared_curve_intersection_trim(iterations)?;
    bench_region_trim(iterations)?;
    bench_prepared_region_trim(iterations)?;
    bench_line_chamfer(iterations)?;
    bench_line_fillet(iterations)?;
    bench_arc_extension(iterations)?;
    bench_boundary_contour_region_build(1_000)?;
    bench_region_boolean_report(1_000)?;
    bench_prepared_region_boolean_report(1_000)?;
    Ok(())
}
