//! Compact boundary of a single positive-width native line stroke.

use super::*;

/// Parallel sides and endpoint caps form a simple convex boundary. Construct
/// those four exact edges directly, preserving the authored line and radius
/// identities instead of arranging a rectangle and two overlapping disks.
/// The caller has validated path connectivity, distinct endpoints, positive
/// width and corner style. Unresolved endpoint equality uses the general path.
pub(super) fn native_line_stroke(
    line: &LineSeg2,
    half_width: &Real,
    cap: OffsetCap,
    policy: &CurveContext,
) -> ExactCurveResult<CurveRegion2> {
    let error = |cause| curve_region_edit_error(CurveOperation2::Offset, cause);
    let left = line.offset_left(half_width.clone()).map_err(error)?;
    let right = line.offset_left(-half_width.clone()).map_err(error)?;
    let cap_edge = |start: Point2, end: Point2, center: &Point2| match cap {
        OffsetCap::Butt => LineSeg2::try_new(start, end)
            .map(Segment2::Line)
            .map_err(error),
        OffsetCap::Round => {
            // The two parallels are displaced by opposite radius vectors of
            // length half_width. Their endpoints are antipodal about this
            // authored endpoint; traversing right-to-left closes a CCW cap.
            Ok(Segment2::Arc(
                CircularArc2::new_with_certified_radius_and_sweep(
                    start,
                    end,
                    center.clone(),
                    half_width * half_width,
                    false,
                    crate::arc_bezier::ArcSweepKind::Semicircle,
                ),
            ))
        }
        OffsetCap::Square => unreachable!("single-line square caps use the general stroke path"),
    };
    let end_cap = cap_edge(right.end().clone(), left.end().clone(), line.end())?;
    let start_cap = cap_edge(left.start().clone(), right.start().clone(), line.start())?;
    let contour = Contour2::try_new(vec![
        Segment2::Line(right),
        end_cap,
        Segment2::Line(left.reversed()),
        start_cap,
    ])
    .map_err(error)?;
    CurveRegion2::try_from_native_contours_raw(vec![contour], vec![], policy)?
        .finish_construction(policy)
}
