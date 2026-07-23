//! Retained region boolean traversal.
//!
//! This module owns the query counterpart to the ordinary region boolean
//! pipeline. Retained region booleans keep the same event/split/classify/emit
//! stages as [`crate::region_boolean`], but route event collection and fragment
//! representative-point classification through [`crate::RegionQuery2`]
//! caches.

use crate::prepared::RegionQuery2;
use crate::{
    BooleanBoundaryLoopSet, BooleanOp, Classification, Contour2, CurvePolicy, CurveResult,
    FillRule, LineArcRegion2,
};

pub(crate) fn boolean_boundary_loops_between_prepared(
    first: &RegionQuery2<'_>,
    second: &RegionQuery2<'_>,
    op: BooleanOp,
    policy: &CurvePolicy,
) -> CurveResult<Classification<BooleanBoundaryLoopSet>> {
    let first_view = first.as_region_view();
    let second_view = second.as_region_view();
    match crate::region_boolean::boolean_boundary_between(
        &first_view,
        &second_view,
        op,
        FillRule::NonZero,
        None,
        policy,
        crate::region_boolean::BooleanBoundaryOutputKind::Loops,
        Some((first, second)),
    )? {
        Classification::Decided(output) => {
            Ok(Classification::Decided(output.into_loops().expect(
                "prepared boundary-loop query requests loop output",
            )))
        }
        Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
    }
}

pub(crate) fn boolean_boundary_contours_between_prepared(
    first: &RegionQuery2<'_>,
    second: &RegionQuery2<'_>,
    op: BooleanOp,
    fill_rule: FillRule,
    policy: &CurvePolicy,
) -> CurveResult<Classification<Vec<Contour2>>> {
    let first_view = first.as_region_view();
    let second_view = second.as_region_view();
    match crate::region_boolean::boolean_boundary_between(
        &first_view,
        &second_view,
        op,
        fill_rule,
        None,
        policy,
        crate::region_boolean::BooleanBoundaryOutputKind::Contours,
        Some((first, second)),
    )? {
        Classification::Decided(output) => {
            Ok(Classification::Decided(output.into_contours().expect(
                "prepared boundary-contour query requests contour output",
            )))
        }
        Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
    }
}

pub(crate) fn boolean_region_between_prepared(
    first: &RegionQuery2<'_>,
    second: &RegionQuery2<'_>,
    op: BooleanOp,
    fill_rule: FillRule,
    policy: &CurvePolicy,
) -> CurveResult<Classification<LineArcRegion2>> {
    boolean_region_between_prepared_impl(first, second, op, fill_rule, policy)
}

fn boolean_region_between_prepared_impl(
    first: &RegionQuery2<'_>,
    second: &RegionQuery2<'_>,
    op: BooleanOp,
    fill_rule: FillRule,
    policy: &CurvePolicy,
) -> CurveResult<Classification<LineArcRegion2>> {
    let first_view = first.as_region_view();
    let second_view = second.as_region_view();
    let boundary_events =
        crate::region_events::intersect_region_views_point_only(&first_view, &second_view, policy)?;
    if let Some(region) =
        crate::region_boolean::retained_offset_region_boolean(&first_view, &second_view, op, policy)
    {
        return Ok(Classification::Decided(region));
    }
    if op == BooleanOp::Xor {
        return xor_region_by_prepared_difference_union(first, second, fill_rule, policy);
    }
    let boundary_output = match crate::region_boolean::boolean_boundary_between(
        &first_view,
        &second_view,
        op,
        fill_rule,
        Some(&boundary_events),
        policy,
        crate::region_boolean::BooleanBoundaryOutputKind::Contours,
        Some((first, second)),
    )? {
        Classification::Decided(result) => result,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let contours = boundary_output
        .into_contours()
        .expect("prepared region Boolean requests contour boundary output");
    if boundary_events.overlap_event_count() == 0 {
        LineArcRegion2::from_directed_boolean_boundary_contours(contours, policy)
    } else {
        LineArcRegion2::from_validated_boundary_contours(contours, policy)
    }
}

fn xor_region_by_prepared_difference_union(
    first: &RegionQuery2<'_>,
    second: &RegionQuery2<'_>,
    fill_rule: FillRule,
    policy: &CurvePolicy,
) -> CurveResult<Classification<LineArcRegion2>> {
    let first_only = match boolean_region_between_prepared(
        first,
        second,
        BooleanOp::Difference,
        fill_rule,
        policy,
    )? {
        Classification::Decided(region) => region,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let second_only = match boolean_region_between_prepared(
        second,
        first,
        BooleanOp::Difference,
        fill_rule,
        policy,
    )? {
        Classification::Decided(region) => region,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };

    Ok(Classification::Decided(
        crate::region_boolean::merge_disjoint_region_bins(first_only, second_only),
    ))
}
