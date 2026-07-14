use hyperlimit::{Point2, PredicatePolicy};
use hyperpath::{
    LinePathSegment, NetId, OffsetSide, PcbTrace, SourceGrid, SourceLengthUnit, SweptLineSegment,
    TraceLayer, offset_axis_aligned_segment,
};
use hyperreal::Real;

fn main() -> Result<(), String> {
    let grid = SourceGrid::with_unit(1_000_000, SourceLengthUnit::Millimeter)
        .ok_or("invalid source grid")?;
    let centerline = LinePathSegment::new(
        Point2::new(
            grid.real_from_units(0).ok_or("invalid x coordinate")?,
            grid.real_from_units(0).ok_or("invalid y coordinate")?,
        ),
        Point2::new(
            grid.real_from_units(10_000_000)
                .ok_or("invalid x coordinate")?,
            grid.real_from_units(0).ok_or("invalid y coordinate")?,
        ),
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
