//! Prepared region boolean traversal.
//!
//! This module owns the prepared counterpart to the ordinary region boolean
//! pipeline. Prepared region booleans keep the same event/split/classify/emit
//! stages as [`crate::region_boolean`], but route event collection and fragment
//! representative-point classification through [`crate::PreparedRegionView2`]
//! caches.

use crate::prepared::PreparedRegionView2;
use crate::{
    BooleanBoundaryLoopSet, BooleanOp, Classification, Contour2, CurvePolicy, CurveResult,
    FillRule, LineArcRegion2, RegionBooleanResult2,
};

pub(crate) fn boolean_boundary_loops_between_prepared(
    first: &PreparedRegionView2<'_>,
    second: &PreparedRegionView2<'_>,
    op: BooleanOp,
    policy: &CurvePolicy,
) -> CurveResult<Classification<BooleanBoundaryLoopSet>> {
    let first_view = first.as_region_view();
    let second_view = second.as_region_view();
    match crate::region_boolean::boolean_boundary_between_with_pipeline_report(
        &first_view,
        &second_view,
        op,
        FillRule::NonZero,
        None,
        policy,
        false,
        crate::region_boolean::BooleanBoundaryOutputKind::Loops,
        Some((first, second)),
    )? {
        Classification::Decided((output, _, _)) => {
            Ok(Classification::Decided(output.into_loops().expect(
                "prepared boundary-loop query requests loop output",
            )))
        }
        Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
    }
}

pub(crate) fn boolean_boundary_contours_between_prepared(
    first: &PreparedRegionView2<'_>,
    second: &PreparedRegionView2<'_>,
    op: BooleanOp,
    fill_rule: FillRule,
    policy: &CurvePolicy,
) -> CurveResult<Classification<Vec<Contour2>>> {
    let first_view = first.as_region_view();
    let second_view = second.as_region_view();
    match crate::region_boolean::boolean_boundary_between_with_pipeline_report(
        &first_view,
        &second_view,
        op,
        fill_rule,
        None,
        policy,
        false,
        crate::region_boolean::BooleanBoundaryOutputKind::Contours,
        Some((first, second)),
    )? {
        Classification::Decided((output, _, _)) => {
            Ok(Classification::Decided(output.into_contours().expect(
                "prepared boundary-contour query requests contour output",
            )))
        }
        Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
    }
}

pub(crate) fn boolean_region_between_prepared(
    first: &PreparedRegionView2<'_>,
    second: &PreparedRegionView2<'_>,
    op: BooleanOp,
    fill_rule: FillRule,
    policy: &CurvePolicy,
) -> CurveResult<Classification<LineArcRegion2>> {
    Ok(
        boolean_region_between_prepared_impl(first, second, op, fill_rule, policy, false)?
            .into_region_classification(),
    )
}

fn boolean_region_between_prepared_impl(
    first: &PreparedRegionView2<'_>,
    second: &PreparedRegionView2<'_>,
    op: BooleanOp,
    fill_rule: FillRule,
    policy: &CurvePolicy,
    retain_pipeline_report: bool,
) -> CurveResult<RegionBooleanResult2> {
    let first_view = first.as_region_view();
    let second_view = second.as_region_view();
    let boundary_events = first.intersect_prepared_region(second, policy)?;
    if let Some(region) =
        crate::region_boolean::retained_offset_region_boolean(&first_view, &second_view, op, policy)
    {
        return Ok(
            crate::region_boolean::region_boolean_result_from_role_assigned_shortcut_region(
                &first_view,
                &second_view,
                op,
                fill_rule,
                crate::RegionBooleanQueryPath2::Prepared,
                &boundary_events,
                region,
                crate::RegionBooleanBoundaryContourSourcePath2::ContainmentShortcut,
            ),
        );
    }
    if op == BooleanOp::Xor {
        return match xor_region_by_prepared_difference_union(first, second, fill_rule, policy)? {
            Classification::Decided(region) => Ok(
                crate::region_boolean::region_boolean_result_from_role_assigned_shortcut_region(
                    &first_view,
                    &second_view,
                    op,
                    fill_rule,
                    crate::RegionBooleanQueryPath2::Prepared,
                    &boundary_events,
                    region,
                    crate::RegionBooleanBoundaryContourSourcePath2::XorDifferenceUnionShortcut,
                ),
            ),
            Classification::Uncertain(reason) => Ok(
                crate::region_boolean::blocked_region_boolean_result_with_prepared_cache(
                    &first_view,
                    &second_view,
                    op,
                    fill_rule,
                    crate::RegionBooleanQueryPath2::Prepared,
                    &boundary_events,
                    crate::region_boolean::retained_status_for_boolean_blocker(reason),
                    reason,
                ),
            ),
        };
    }
    let (boundary_output, boundary_contour_source_path, pipeline_report) =
        match crate::region_boolean::boolean_boundary_between_with_pipeline_report(
            &first_view,
            &second_view,
            op,
            fill_rule,
            Some(&boundary_events),
            policy,
            retain_pipeline_report,
            crate::region_boolean::BooleanBoundaryOutputKind::Contours,
            Some((first, second)),
        )? {
            Classification::Decided(result) => result,
            Classification::Uncertain(reason) => {
                return Ok(
                    crate::region_boolean::blocked_region_boolean_result_with_prepared_cache(
                        &first_view,
                        &second_view,
                        op,
                        fill_rule,
                        crate::RegionBooleanQueryPath2::Prepared,
                        &boundary_events,
                        crate::region_boolean::retained_status_for_boolean_blocker(reason),
                        reason,
                    ),
                );
            }
        };
    let contours = boundary_output
        .into_contours()
        .expect("prepared region Boolean requests contour boundary output");
    if !retain_pipeline_report {
        return Ok(
            match if boundary_events.overlap_event_count() == 0 {
                LineArcRegion2::from_directed_boolean_boundary_contours(contours, policy)?
            } else {
                LineArcRegion2::from_validated_boundary_contours(contours, policy)?
            } {
                Classification::Decided(region) => {
                    crate::region_boolean::region_boolean_result_from_role_assigned_shortcut_region(
                        &first_view,
                        &second_view,
                        op,
                        fill_rule,
                        crate::RegionBooleanQueryPath2::Prepared,
                        &boundary_events,
                        region,
                        boundary_contour_source_path,
                    )
                }
                Classification::Uncertain(reason) => {
                    crate::region_boolean::blocked_region_boolean_result_with_prepared_cache(
                        &first_view,
                        &second_view,
                        op,
                        fill_rule,
                        crate::RegionBooleanQueryPath2::Prepared,
                        &boundary_events,
                        crate::region_boolean::retained_status_for_boolean_blocker(reason),
                        reason,
                    )
                }
            },
        );
    }
    crate::region_boolean::region_boolean_result_from_boundary_contours_with_prepared_cache_and_pipeline_report(
        &first_view,
        &second_view,
        op,
        fill_rule,
        crate::RegionBooleanQueryPath2::Prepared,
        &boundary_events,
        contours,
        boundary_contour_source_path,
        pipeline_report,
        policy,
    )
}

fn xor_region_by_prepared_difference_union(
    first: &PreparedRegionView2<'_>,
    second: &PreparedRegionView2<'_>,
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
