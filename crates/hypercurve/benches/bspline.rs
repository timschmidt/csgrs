use std::hint::black_box;
use std::time::Instant;

use hypercurve::{
    Classification, CurvePolicy, CurveResult, Point2, PolynomialBSplineCurve2,
    RationalBSplineCurve2, RationalQuadraticBSplineCurve2, Real,
};

fn r(value: i32) -> Real {
    value.into()
}

fn p(x: i32, y: i32) -> Point2 {
    Point2::new(r(x), r(y))
}

fn decided<T>(classification: Classification<T>) -> T {
    match classification {
        Classification::Decided(value) => value,
        Classification::Uncertain(reason) => panic!("benchmark unexpectedly uncertain: {reason:?}"),
    }
}

fn main() -> CurveResult<()> {
    let policy = CurvePolicy::certified();
    let spline = decided(PolynomialBSplineCurve2::try_new(
        3,
        vec![p(0, 0), p(1, 3), p(3, 3), p(5, 3), p(6, 0)],
        vec![r(0), r(0), r(0), r(0), r(1), r(2), r(2), r(2), r(2)],
        &policy,
    )?);

    let iterations = 20_000_u32;
    let started = Instant::now();
    let mut checksum = 0_usize;
    for _ in 0..iterations {
        let extraction = decided(spline.extract_bezier_spans(&policy)?);
        checksum ^= black_box(extraction.spans().len() + extraction.inserted_knot_count());
    }
    let elapsed = started.elapsed();
    println!(
        "bspline_bezier_extraction: {iterations} iterations in {elapsed:?} ({:?}/iter), checksum={checksum}",
        elapsed / iterations
    );

    let rational = decided(RationalQuadraticBSplineCurve2::try_new(
        vec![p(0, 0), p(2, 4), p(4, 4), p(6, 0)],
        vec![r(1), r(2), r(4), r(1)],
        vec![r(0), r(0), r(0), r(1), r(2), r(2), r(2)],
        &policy,
    )?);
    let started = Instant::now();
    let mut rational_checksum = 0_usize;
    for _ in 0..iterations {
        let extraction = decided(rational.extract_bezier_spans(&policy)?);
        rational_checksum ^= black_box(extraction.spans().len() + extraction.inserted_knot_count());
    }
    let elapsed = started.elapsed();
    println!(
        "rational_quadratic_bspline_bezier_extraction: {iterations} iterations in {elapsed:?} ({:?}/iter), checksum={rational_checksum}",
        elapsed / iterations
    );

    let rational_cubic = decided(RationalBSplineCurve2::try_new(
        3,
        vec![p(0, 0), p(1, 3), p(3, 3), p(5, 3), p(6, 0)],
        vec![r(1), r(2), r(4), r(8), r(16)],
        vec![r(0), r(0), r(0), r(0), r(1), r(2), r(2), r(2), r(2)],
        &policy,
    )?);
    let started = Instant::now();
    let mut rational_cubic_checksum = 0_usize;
    for _ in 0..iterations {
        let extraction = decided(rational_cubic.extract_bezier_spans(&policy)?);
        rational_cubic_checksum ^= black_box(
            extraction.spans().len()
                + extraction.inserted_knot_count()
                + extraction.refined_weights().len(),
        );
    }
    let elapsed = started.elapsed();
    println!(
        "rational_cubic_bspline_bezier_extraction: {iterations} iterations in {elapsed:?} ({:?}/iter), checksum={rational_cubic_checksum}",
        elapsed / iterations
    );

    let equal_weight_rational_cubic = decided(RationalBSplineCurve2::try_new(
        3,
        vec![p(0, 0), p(1, 3), p(3, 3), p(5, 3), p(6, 0)],
        vec![r(5), r(5), r(5), r(5), r(5)],
        vec![r(0), r(0), r(0), r(0), r(1), r(2), r(2), r(2), r(2)],
        &policy,
    )?);
    let started = Instant::now();
    let mut native_checksum = 0_usize;
    for _ in 0..iterations {
        let extraction = decided(equal_weight_rational_cubic.extract_bezier_spans(&policy)?);
        let report = decided(extraction.native_topology_report(&policy)?);
        let native = decided(extraction.native_subcurves(&policy)?);
        native_checksum ^= black_box(
            native.len()
                + report.span_reports().len()
                + usize::from(report.is_fully_native_exact())
                + extraction.inserted_knot_count(),
        );
    }
    let elapsed = started.elapsed();
    println!(
        "rational_cubic_bspline_native_subcurves: {iterations} iterations in {elapsed:?} ({:?}/iter), checksum={native_checksum}",
        elapsed / iterations
    );

    let started = Instant::now();
    let mut retained_status_checksum = 0_usize;
    for _ in 0..iterations {
        let extraction = decided(rational_cubic.extract_bezier_spans(&policy)?);
        let report = decided(extraction.native_topology_report(&policy)?);
        retained_status_checksum ^= black_box(
            report.span_reports().len()
                + report
                    .span_reports()
                    .iter()
                    .filter(|span| span.status().is_retained_evidence())
                    .count(),
        );
    }
    let elapsed = started.elapsed();
    println!(
        "rational_cubic_bspline_topology_status_report: {iterations} iterations in {elapsed:?} ({:?}/iter), checksum={retained_status_checksum}",
        elapsed / iterations
    );

    Ok(())
}
