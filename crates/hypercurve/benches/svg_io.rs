use std::hint::black_box;
use std::time::Instant;

use hypercurve::{
    BulgeVertex2, CircularArc2, Contour2, CurveResult, CurveString2, FillRule, LineSeg2, Point2,
    Real, Region2, Segment2, import_svg_contour_path_data_with_report,
    import_svg_path_data_with_report, import_svg_region_path_data_with_report,
};

fn s(value: i32) -> Real {
    value.into()
}

fn p(x: i32, y: i32) -> Point2 {
    Point2::new(s(x), s(y))
}

fn line(a: (i32, i32), b: (i32, i32)) -> Segment2 {
    Segment2::Line(LineSeg2::try_new(p(a.0, a.1), p(b.0, b.1)).unwrap())
}

fn rectangle(xmin: i32, ymin: i32, xmax: i32, ymax: i32) -> Contour2 {
    Contour2::from_bulge_vertices(&[
        BulgeVertex2::new(p(xmin, ymin), Real::zero()),
        BulgeVertex2::new(p(xmax, ymin), Real::zero()),
        BulgeVertex2::new(p(xmax, ymax), Real::zero()),
        BulgeVertex2::new(p(xmin, ymax), Real::zero()),
    ])
    .unwrap()
}

fn bench_curve_export(iterations: u32) -> CurveResult<()> {
    let curve = CurveString2::try_new(vec![
        line((0, 0), (10, 0)),
        line((10, 0), (10, 6)),
        Segment2::Arc(CircularArc2::from_bulge(p(10, 6), p(12, 6), -Real::one())?),
    ])?;
    let started = Instant::now();
    let mut total_bytes = 0_usize;

    for _ in 0..iterations {
        let exported = curve.to_svg_path_data_with_report()?;
        let path = exported
            .path_data()
            .expect("curve SVG export should materialize");
        total_bytes += black_box(path.len() + exported.report().segment_reports().len());
    }

    let elapsed = started.elapsed();
    println!(
        "svg_curve_export_line_arc: {iterations} iterations in {elapsed:?} ({:?}/iter), total bytes={total_bytes}",
        elapsed / iterations
    );
    Ok(())
}

fn bench_region_export(iterations: u32) -> CurveResult<()> {
    let region = Region2::new(
        vec![rectangle(0, 0, 40, 40)],
        vec![rectangle(10, 10, 20, 20)],
    );
    let started = Instant::now();
    let mut total_bytes = 0_usize;

    for _ in 0..iterations {
        let exported = region.to_svg_path_data_with_report()?;
        let path = exported
            .path_data()
            .expect("region SVG export should materialize");
        total_bytes += black_box(path.len() + exported.report().segment_reports().len());
    }

    let elapsed = started.elapsed();
    println!(
        "svg_region_export_nested_rectangles: {iterations} iterations in {elapsed:?} ({:?}/iter), total bytes={total_bytes}",
        elapsed / iterations
    );
    Ok(())
}

fn bench_open_arc_import(iterations: u32) -> CurveResult<()> {
    let started = Instant::now();
    let mut total_segments = 0_usize;

    for source_version in 0..iterations {
        let imported = import_svg_path_data_with_report(
            "M 0 0 A 1 1 0 0 0 2 0",
            31,
            source_version.into(),
            None,
        );
        let curve = imported
            .curve_string()
            .expect("exact SVG semicircle import should materialize");
        let record = imported
            .report()
            .retained_import()
            .expect("SVG import should retain source topology");
        total_segments += black_box(curve.len() + record.emitted_segment_count());
    }

    let elapsed = started.elapsed();
    println!(
        "svg_open_semicircle_import: {iterations} iterations in {elapsed:?} ({:?}/iter), total segments={total_segments}",
        elapsed / iterations
    );
    Ok(())
}

fn bench_closed_contour_import(iterations: u32) -> CurveResult<()> {
    let started = Instant::now();
    let mut total_segments = 0_usize;

    for source_version in 0..iterations {
        let imported = import_svg_contour_path_data_with_report(
            "M 0 0 A 1 1 0 0 0 2 0 Z",
            FillRule::NonZero,
            37,
            source_version.into(),
            None,
        );
        let contour = imported
            .contour()
            .expect("closed exact SVG semicircle contour import should materialize");
        total_segments += black_box(contour.len());
    }

    let elapsed = started.elapsed();
    println!(
        "svg_closed_semicircle_contour_import: {iterations} iterations in {elapsed:?} ({:?}/iter), total segments={total_segments}",
        elapsed / iterations
    );
    Ok(())
}

fn bench_region_import(iterations: u32) -> CurveResult<()> {
    let path_data = "M 0 0 L 40 0 L 40 40 L 0 40 Z M 10 10 L 20 10 L 20 20 L 10 20 Z";
    let policy = hypercurve::CurvePolicy::certified();
    let started = Instant::now();
    let mut total_contours = 0_usize;

    for source_version in 0..iterations {
        let imported = import_svg_region_path_data_with_report(
            path_data,
            FillRule::NonZero,
            41,
            source_version.into(),
            None,
            &policy,
        );
        let region = imported
            .region()
            .expect("nested SVG region import should materialize");
        total_contours +=
            black_box(region.material_contours().len() + region.hole_contours().len());
    }

    let elapsed = started.elapsed();
    println!(
        "svg_nested_region_import: {iterations} iterations in {elapsed:?} ({:?}/iter), total contours={total_contours}",
        elapsed / iterations
    );
    Ok(())
}

fn bench_unsupported_import(iterations: u32) -> CurveResult<()> {
    let started = Instant::now();
    let mut unsupported_count = 0_usize;

    for source_version in 0..iterations {
        let imported = import_svg_path_data_with_report(
            "M 0 0 C 1 0 1 1 2 1",
            43,
            source_version.into(),
            None,
        );
        if imported.curve_string().is_none() && imported.report().blocker().is_some() {
            unsupported_count += black_box(1);
        }
    }

    let elapsed = started.elapsed();
    println!(
        "svg_unsupported_cubic_import_report: {iterations} iterations in {elapsed:?} ({:?}/iter), unsupported={unsupported_count}",
        elapsed / iterations
    );
    Ok(())
}

fn main() -> CurveResult<()> {
    bench_curve_export(20_000)?;
    bench_region_export(20_000)?;
    bench_open_arc_import(20_000)?;
    bench_closed_contour_import(20_000)?;
    bench_region_import(10_000)?;
    bench_unsupported_import(20_000)?;
    Ok(())
}
