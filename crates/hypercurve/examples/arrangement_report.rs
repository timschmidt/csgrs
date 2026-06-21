use hypercurve::{
    Classification, CurvePolicy, ExactCurveArrangementAttempt2, ExactCurveArrangementRequest2,
    FillRule, LineSeg2, Point2, RegionPointLocation,
};
use hyperreal::Real;

fn p(x: i32, y: i32) -> Point2 {
    Point2::new(Real::from(x), Real::from(y))
}

fn line(start_x: i32, start_y: i32, end_x: i32, end_y: i32) -> hypercurve::CurveResult<LineSeg2> {
    LineSeg2::try_new(p(start_x, start_y), p(end_x, end_y))
}

fn main() -> hypercurve::CurveResult<()> {
    let policy = CurvePolicy::certified();
    let boundary = vec![
        line(0, 0, 4, 0)?,
        line(4, 0, 4, 4)?,
        line(4, 4, 0, 4)?,
        line(0, 4, 0, 0)?,
    ];

    let request =
        ExactCurveArrangementRequest2::from_unordered_line_segments(boundary, FillRule::NonZero);
    let result = ExactCurveArrangementAttempt2::new(request).evaluate(&policy)?;
    let region = match result.region_classification() {
        Classification::Decided(region) => region,
        Classification::Uncertain(reason) => {
            panic!("arrangement blocked with retained uncertainty: {reason:?}");
        }
    };
    let report = result.arrangement_report();

    assert!(report.status().unwrap().is_native_exact());
    assert_eq!(report.source_segment_count(), 4);
    assert_eq!(report.materialized_region(), Some(true));
    assert!(matches!(
        region.classify_point(&p(2, 2), &policy),
        Classification::Decided(RegionPointLocation::Inside)
    ));

    Ok(())
}
