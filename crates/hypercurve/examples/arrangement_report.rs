use hypercurve::{
    Classification, CurvePolicy, FillRule, LineSeg2, Point2, Region2, RegionPointLocation,
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

    let (region, report) = match Region2::from_unordered_line_segments_with_arrangement_report(
        boundary,
        FillRule::NonZero,
        &policy,
    )? {
        (Classification::Decided(region), report) => (region, report),
        (Classification::Uncertain(reason), _) => {
            panic!("arrangement blocked with retained uncertainty: {reason:?}");
        }
    };

    assert!(report.status().unwrap().is_native_exact());
    assert_eq!(report.source_segment_count(), 4);
    assert_eq!(report.materialized_region(), Some(true));
    assert!(matches!(
        region.classify_point(&p(2, 2), &policy),
        Classification::Decided(RegionPointLocation::Inside)
    ));

    Ok(())
}
