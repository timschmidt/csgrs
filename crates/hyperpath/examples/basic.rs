use hyperlimit::{Point2, PredicatePolicy};
use hyperpath::{
    LinePathSegment, NetId, OffsetSide, PcbTrace, SweptLineSegment, TraceLayer,
    offset_axis_aligned_segment,
};
use hyperreal::Real;

fn main() -> Result<(), String> {
    let centerline = LinePathSegment::new(
        Point2::new(Real::from(0), Real::from(0)),
        Point2::new(Real::from(10), Real::from(0)),
    );
    let offset = offset_axis_aligned_segment(
        &centerline,
        Real::from(2),
        OffsetSide::Left,
        PredicatePolicy,
    )
    .map_err(|error| format!("offset failed: {error:?}"))?;
    assert_eq!(offset.segment.start(), &Point2::new(0.into(), 2.into()));

    let swept = SweptLineSegment::new(centerline, Real::from(1))?;
    let _trace = PcbTrace::new(NetId(1), TraceLayer(0), swept);
    Ok(())
}
