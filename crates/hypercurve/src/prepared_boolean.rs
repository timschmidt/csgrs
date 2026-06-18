//! Prepared region boolean traversal.
//!
//! This module owns the prepared counterpart to the ordinary region boolean
//! pipeline. Prepared region booleans keep the same event/split/classify/emit
//! stages as [`crate::region_boolean`], but route event collection and fragment
//! representative-point classification through [`crate::PreparedRegionView2`]
//! caches.

use crate::prepared::{PreparedContourView2, PreparedRegionView2};
use crate::{
    BooleanBoundaryLoopSet, BooleanFragmentSelection, BooleanOp, Classification, Contour2,
    CurvePolicy, CurveResult, FillRule, Region2, RegionBooleanResult2, RegionFragmentSet,
    RegionIntersectionSet, RegionPointLocation, RegionSide,
};

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum PreparedBoundaryContactKind {
    PointOnly,
    Overlap,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum PreparedBoundaryContactResolution {
    BoundaryOnly(PreparedBoundaryContactKind),
    Containment {
        relation: crate::region_boolean::BoundaryContainmentRelation,
        contact: PreparedBoundaryContactKind,
    },
}

pub(crate) fn boolean_boundary_loops_between_prepared(
    first: &PreparedRegionView2<'_>,
    second: &PreparedRegionView2<'_>,
    op: BooleanOp,
    policy: &CurvePolicy,
) -> CurveResult<Classification<BooleanBoundaryLoopSet>> {
    let first_view = first.as_region_view();
    let second_view = second.as_region_view();
    if crate::region_boolean::same_region_view(&first_view, &second_view) {
        return Ok(Classification::Decided(
            BooleanBoundaryLoopSet::from_contours(match op {
                BooleanOp::Union | BooleanOp::Intersection => {
                    crate::region_boolean::clone_boundary_contours(&first_view)
                }
                BooleanOp::Difference | BooleanOp::Xor => Vec::new(),
            })?,
        ));
    }
    if first_view.is_empty() || second_view.is_empty() {
        return Ok(Classification::Decided(
            BooleanBoundaryLoopSet::from_contours(
                crate::region_boolean::empty_operand_boundary_contours(
                    &first_view,
                    &second_view,
                    op,
                ),
            )?,
        ));
    }
    match crate::region_boolean::coextensive_axis_rect_region_boolean(
        &first_view,
        &second_view,
        op,
        policy,
    )? {
        Classification::Decided(Some(region)) => {
            return Ok(Classification::Decided(
                BooleanBoundaryLoopSet::from_contours(
                    crate::region_boolean::clone_boundary_contours(&region.as_view()),
                )?,
            ));
        }
        Classification::Decided(None) => {}
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    }
    match boundary_contact_resolution_prepared(first, second, policy)? {
        // Shared-boundary topology is resolved to explicit closed contours above the
        // traversal graph, matching the plain-path contract from
        // `boundary_contact_boundary_contours_prepared`.
        // This follows Greiner and Hormann's split-and-classify stage for polygon
        // clipping and preserves `Boundary` uncertainty as a pre-loop signal.
        // G. Greiner and K. Hormann, "Efficient clipping of arbitrary polygons,"
        // ACM Transactions on Graphics 17(2), 71-83, 1998.
        // The containment branch mirrors the selection decomposition from
        // Martinez et al., "A new algorithm for computing Boolean operations on
        // polygons," Computers & Geosciences 35(6), 1177-1185, 2009.
        Classification::Decided(Some(PreparedBoundaryContactResolution::BoundaryOnly(kind))) => {
            return boundary_contact_boundary_contours_prepared(
                first,
                second,
                op,
                FillRule::NonZero,
                policy,
                kind,
            )
            .and_then(BooleanBoundaryLoopSet::from_contour_classification);
        }
        Classification::Decided(Some(PreparedBoundaryContactResolution::Containment {
            relation,
            contact,
        })) => {
            if let Some(contours) =
                containment_boundary_contours_prepared(first, second, op, relation)
            {
                return Ok(Classification::Decided(
                    BooleanBoundaryLoopSet::from_contours(contours)?,
                ));
            }
            if relation == crate::region_boolean::BoundaryContainmentRelation::FirstContainsSecond
                && contact == PreparedBoundaryContactKind::Overlap
                && op == BooleanOp::Difference
            {
                return containment_difference_boundary_contours_prepared(
                    first,
                    second,
                    FillRule::NonZero,
                    policy,
                )
                .and_then(BooleanBoundaryLoopSet::from_contour_classification);
            }
        }
        Classification::Decided(None) => {
            // Union overlap is decided through contour reconstruction to preserve a
            // shared-edge fast path before entering fragment traversal.
            if op == BooleanOp::Union
                && crate::region_boolean::region_boundary_has_overlap(
                    &first_view,
                    &second_view,
                    policy,
                )?
            {
                return boundary_overlap_union_contours_prepared(
                    first,
                    second,
                    BooleanOp::Union,
                    FillRule::NonZero,
                    policy,
                )
                .and_then(BooleanBoundaryLoopSet::from_contour_classification);
            }
        }
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    }
    if op == BooleanOp::Xor {
        return xor_boundary_contours_by_prepared_region(first, second, FillRule::NonZero, policy)
            .and_then(BooleanBoundaryLoopSet::from_contour_classification);
    }

    let intersections = first.intersect_prepared_region(second, policy)?;

    let fragments = match intersections.split_regions(&first_view, &second_view, policy)? {
        Classification::Decided(fragments) => fragments,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };

    // The prepared path keeps the same split/classify/traverse structure as the
    // ordinary Greiner-Hormann/Martinez-Rueda-Feito pipeline, but routes
    // fragment representative-point classification through prepared region
    // caches so repeated boolean-boundary queries do not rebuild contour boxes.
    let selection =
        match classify_fragments_with_prepared_regions(&fragments, first, second, op, policy)? {
            Classification::Decided(selection) => selection,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };

    let emitted = selection.emit_boundary_fragments(&fragments)?;
    let chains = match emitted.assemble_chains(policy) {
        Classification::Decided(chains) => chains,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };

    Ok(chains.into_closed_loops())
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
    if crate::region_boolean::same_region_view(&first_view, &second_view) {
        return Ok(Classification::Decided(match op {
            BooleanOp::Union | BooleanOp::Intersection => {
                crate::region_boolean::clone_boundary_contours(&first_view)
            }
            BooleanOp::Difference | BooleanOp::Xor => Vec::new(),
        }));
    }
    if first_view.is_empty() || second_view.is_empty() {
        return Ok(Classification::Decided(
            crate::region_boolean::empty_operand_boundary_contours(&first_view, &second_view, op),
        ));
    }
    match crate::region_boolean::coextensive_axis_rect_region_boolean(
        &first_view,
        &second_view,
        op,
        policy,
    )? {
        Classification::Decided(Some(region)) => {
            return Ok(Classification::Decided(
                crate::region_boolean::clone_boundary_contours(&region.as_view()),
            ));
        }
        Classification::Decided(None) => {}
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    }
    match boundary_contact_resolution_prepared(first, second, policy)? {
        Classification::Decided(Some(PreparedBoundaryContactResolution::BoundaryOnly(kind))) => {
            return boundary_contact_boundary_contours_prepared(
                first, second, op, fill_rule, policy, kind,
            );
        }
        Classification::Decided(Some(PreparedBoundaryContactResolution::Containment {
            relation,
            contact,
        })) => {
            if let Some(contours) =
                containment_boundary_contours_prepared(first, second, op, relation)
            {
                return Ok(Classification::Decided(contours));
            }
            if relation == crate::region_boolean::BoundaryContainmentRelation::FirstContainsSecond
                && contact == PreparedBoundaryContactKind::Overlap
                && op == BooleanOp::Difference
            {
                return containment_difference_boundary_contours_prepared(
                    first, second, fill_rule, policy,
                );
            }
        }
        Classification::Decided(None) => {}
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    }
    if op == BooleanOp::Xor {
        return xor_boundary_contours_by_prepared_region(first, second, fill_rule, policy);
    }

    match boolean_boundary_loops_between_prepared(first, second, op, policy)? {
        Classification::Decided(loops) => {
            loops.into_contours(fill_rule).map(Classification::Decided)
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
) -> CurveResult<Classification<Region2>> {
    let first_view = first.as_region_view();
    let second_view = second.as_region_view();
    if crate::region_boolean::same_region_view(&first_view, &second_view) {
        return Ok(Classification::Decided(match op {
            BooleanOp::Union | BooleanOp::Intersection => {
                crate::region_boolean::clone_region(&first_view)
            }
            BooleanOp::Difference | BooleanOp::Xor => Region2::empty(),
        }));
    }
    if first_view.is_empty() || second_view.is_empty() {
        return Ok(Classification::Decided(
            crate::region_boolean::empty_operand_region(&first_view, &second_view, op),
        ));
    }
    match crate::region_boolean::coextensive_axis_rect_region_boolean(
        &first_view,
        &second_view,
        op,
        policy,
    )? {
        Classification::Decided(Some(region)) => return Ok(Classification::Decided(region)),
        Classification::Decided(None) => {}
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    }
    match boundary_contact_resolution_prepared(first, second, policy)? {
        Classification::Decided(Some(PreparedBoundaryContactResolution::BoundaryOnly(kind))) => {
            return boundary_contact_region_prepared(first, second, op, fill_rule, policy, kind);
        }
        Classification::Decided(Some(PreparedBoundaryContactResolution::Containment {
            relation,
            contact,
        })) => {
            if let Some(region) = containment_region_prepared(first, second, op, relation) {
                return Ok(Classification::Decided(region));
            }
            if relation == crate::region_boolean::BoundaryContainmentRelation::FirstContainsSecond
                && contact == PreparedBoundaryContactKind::Overlap
                && op == BooleanOp::Difference
            {
                return match containment_difference_boundary_contours_prepared(
                    first, second, fill_rule, policy,
                )? {
                    Classification::Decided(contours) => {
                        Region2::from_boundary_contours(contours, policy)
                    }
                    Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
                };
            }
        }
        Classification::Decided(None) => {}
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    }
    if op == BooleanOp::Xor {
        return xor_region_by_prepared_difference_union(first, second, fill_rule, policy);
    }

    match boolean_boundary_contours_between_prepared(first, second, op, fill_rule, policy)? {
        Classification::Decided(contours) => Region2::from_boundary_contours(contours, policy),
        Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
    }
}

pub(crate) fn boolean_region_between_prepared_with_report(
    first: &PreparedRegionView2<'_>,
    second: &PreparedRegionView2<'_>,
    op: BooleanOp,
    fill_rule: FillRule,
    policy: &CurvePolicy,
) -> CurveResult<RegionBooleanResult2> {
    let first_view = first.as_region_view();
    let second_view = second.as_region_view();
    let contours =
        match boolean_boundary_contours_between_prepared(first, second, op, fill_rule, policy)? {
            Classification::Decided(contours) => contours,
            Classification::Uncertain(reason) => {
                return Ok(crate::region_boolean::blocked_region_boolean_result(
                    &first_view,
                    &second_view,
                    op,
                    crate::RegionBooleanQueryPath2::Prepared,
                    crate::region_boolean::retained_status_for_boolean_blocker(reason),
                    reason,
                ));
            }
        };
    crate::region_boolean::region_boolean_result_from_boundary_contours(
        &first_view,
        &second_view,
        op,
        crate::RegionBooleanQueryPath2::Prepared,
        contours,
        policy,
    )
}

fn xor_boundary_contours_by_prepared_region(
    first: &PreparedRegionView2<'_>,
    second: &PreparedRegionView2<'_>,
    fill_rule: FillRule,
    policy: &CurvePolicy,
) -> CurveResult<Classification<Vec<Contour2>>> {
    match xor_region_by_prepared_difference_union(first, second, fill_rule, policy)? {
        Classification::Decided(region) => Ok(Classification::Decided(
            crate::region_boolean::clone_boundary_contours(&region.as_view()),
        )),
        Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
    }
}

fn boundary_contact_resolution_prepared(
    first: &PreparedRegionView2<'_>,
    second: &PreparedRegionView2<'_>,
    policy: &CurvePolicy,
) -> CurveResult<Classification<Option<PreparedBoundaryContactResolution>>> {
    let intersections = first.intersect_prepared_region(second, policy)?;
    if intersections.is_empty() {
        return Ok(Classification::Decided(None));
    }

    let saw_overlap = match crate::region_boolean::boundary_contact_overlap_flag(&intersections) {
        Classification::Decided(Some(saw_overlap)) => saw_overlap,
        Classification::Decided(None) => return Ok(Classification::Decided(None)),
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };

    // Prepared boundary-contact certification follows the same degeneracy split
    // described by Foster, Hormann, and Popa, "Clipping simple polygons with
    // degenerate intersections" (2019), but reuses cached region classifiers
    // for the interior-disjointness samples.
    let disjoint_interiors = if saw_overlap {
        split_contact_interiors_are_disjoint_prepared(first, second, &intersections, policy)?
    } else {
        unsplit_contact_interiors_are_disjoint_prepared(first, second, policy)?
    };
    match disjoint_interiors {
        Classification::Decided(true) => {}
        Classification::Decided(false) => {
            return match boundary_contact_containment_relation_prepared(first, second, policy)? {
                Classification::Decided(Some(relation)) => Ok(Classification::Decided(Some(
                    PreparedBoundaryContactResolution::Containment {
                        relation,
                        contact: if saw_overlap {
                            PreparedBoundaryContactKind::Overlap
                        } else {
                            PreparedBoundaryContactKind::PointOnly
                        },
                    },
                ))),
                Classification::Decided(None) => Ok(Classification::Decided(None)),
                Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
            };
        }
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    }

    Ok(Classification::Decided(Some(
        PreparedBoundaryContactResolution::BoundaryOnly(if saw_overlap {
            PreparedBoundaryContactKind::Overlap
        } else {
            PreparedBoundaryContactKind::PointOnly
        }),
    )))
}

fn split_contact_interiors_are_disjoint_prepared(
    first: &PreparedRegionView2<'_>,
    second: &PreparedRegionView2<'_>,
    intersections: &RegionIntersectionSet,
    policy: &CurvePolicy,
) -> CurveResult<Classification<bool>> {
    let first_view = first.as_region_view();
    let second_view = second.as_region_view();
    let fragments = match intersections.split_regions(&first_view, &second_view, policy)? {
        Classification::Decided(fragments) => fragments,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };

    let mut first_has_outside_sample = false;
    let mut second_has_outside_sample = false;
    for contour_fragments in fragments.contours() {
        for fragment in contour_fragments.fragments.fragments() {
            let sample = match fragment.segment.representative_point(policy)? {
                Classification::Decided(sample) => sample,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let location = match contour_fragments.key.side {
                RegionSide::First => second.classify_point(&sample, policy),
                RegionSide::Second => first.classify_point(&sample, policy),
            };
            match location {
                Classification::Decided(RegionPointLocation::Outside) => {
                    match contour_fragments.key.side {
                        RegionSide::First => first_has_outside_sample = true,
                        RegionSide::Second => second_has_outside_sample = true,
                    }
                }
                Classification::Decided(RegionPointLocation::Boundary) => {}
                Classification::Decided(RegionPointLocation::Inside) => {
                    return Ok(Classification::Decided(false));
                }
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            }
        }
    }

    Ok(Classification::Decided(
        first_has_outside_sample && second_has_outside_sample,
    ))
}

fn unsplit_contact_interiors_are_disjoint_prepared(
    first: &PreparedRegionView2<'_>,
    second: &PreparedRegionView2<'_>,
    policy: &CurvePolicy,
) -> CurveResult<Classification<bool>> {
    let mut first_has_outside_sample = false;
    let mut second_has_outside_sample = false;

    match scan_unsplit_prepared_contact_samples(
        first.prepared_material_contours(),
        second,
        &mut first_has_outside_sample,
        policy,
    )? {
        Classification::Decided(true) => {}
        Classification::Decided(false) => return Ok(Classification::Decided(false)),
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    }
    match scan_unsplit_prepared_contact_samples(
        first.prepared_hole_contours(),
        second,
        &mut first_has_outside_sample,
        policy,
    )? {
        Classification::Decided(true) => {}
        Classification::Decided(false) => return Ok(Classification::Decided(false)),
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    }
    match scan_unsplit_prepared_contact_samples(
        second.prepared_material_contours(),
        first,
        &mut second_has_outside_sample,
        policy,
    )? {
        Classification::Decided(true) => {}
        Classification::Decided(false) => return Ok(Classification::Decided(false)),
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    }
    match scan_unsplit_prepared_contact_samples(
        second.prepared_hole_contours(),
        first,
        &mut second_has_outside_sample,
        policy,
    )? {
        Classification::Decided(true) => {}
        Classification::Decided(false) => return Ok(Classification::Decided(false)),
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    }

    Ok(Classification::Decided(
        first_has_outside_sample && second_has_outside_sample,
    ))
}

fn scan_unsplit_prepared_contact_samples(
    contours: &[PreparedContourView2<'_>],
    opposite: &PreparedRegionView2<'_>,
    has_outside_sample: &mut bool,
    policy: &CurvePolicy,
) -> CurveResult<Classification<bool>> {
    for contour in contours {
        for segment in contour.contour().segments() {
            let sample = match segment.representative_point(policy)? {
                Classification::Decided(sample) => sample,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            match opposite.classify_point(&sample, policy) {
                Classification::Decided(RegionPointLocation::Outside) => {
                    *has_outside_sample = true;
                }
                Classification::Decided(RegionPointLocation::Boundary) => {}
                Classification::Decided(RegionPointLocation::Inside) => {
                    return Ok(Classification::Decided(false));
                }
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            }
        }
    }

    Ok(Classification::Decided(true))
}

fn boundary_contact_containment_relation_prepared(
    first: &PreparedRegionView2<'_>,
    second: &PreparedRegionView2<'_>,
    policy: &CurvePolicy,
) -> CurveResult<Classification<Option<crate::region_boolean::BoundaryContainmentRelation>>> {
    let first_contains_second =
        match prepared_region_contains_region_boundary_samples(first, second, policy)? {
            Classification::Decided(contains) => contains,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
    let second_contains_first =
        match prepared_region_contains_region_boundary_samples(second, first, policy)? {
            Classification::Decided(contains) => contains,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };

    Ok(Classification::Decided(
        match (first_contains_second, second_contains_first) {
            (true, true) => Some(crate::region_boolean::BoundaryContainmentRelation::Equivalent),
            (true, false) => {
                Some(crate::region_boolean::BoundaryContainmentRelation::FirstContainsSecond)
            }
            (false, true) => {
                Some(crate::region_boolean::BoundaryContainmentRelation::SecondContainsFirst)
            }
            (false, false) => None,
        },
    ))
}

fn prepared_region_contains_region_boundary_samples(
    container: &PreparedRegionView2<'_>,
    candidate: &PreparedRegionView2<'_>,
    policy: &CurvePolicy,
) -> CurveResult<Classification<bool>> {
    crate::region_boolean::boundary_contours_inside_or_on_region(
        candidate
            .prepared_material_contours()
            .iter()
            .map(|contour| contour.contour())
            .chain(
                candidate
                    .prepared_hole_contours()
                    .iter()
                    .map(|contour| contour.contour()),
            ),
        |point| container.classify_point(point, policy),
        policy,
    )
}

fn containment_boundary_contours_prepared(
    first: &PreparedRegionView2<'_>,
    second: &PreparedRegionView2<'_>,
    op: BooleanOp,
    relation: crate::region_boolean::BoundaryContainmentRelation,
) -> Option<Vec<Contour2>> {
    let first_view = first.as_region_view();
    let second_view = second.as_region_view();
    match (relation, op) {
        (
            crate::region_boolean::BoundaryContainmentRelation::FirstContainsSecond,
            BooleanOp::Union,
        ) => Some(crate::region_boolean::clone_boundary_contours(&first_view)),
        (
            crate::region_boolean::BoundaryContainmentRelation::FirstContainsSecond,
            BooleanOp::Intersection,
        ) => Some(crate::region_boolean::clone_boundary_contours(&second_view)),
        (
            crate::region_boolean::BoundaryContainmentRelation::SecondContainsFirst,
            BooleanOp::Union,
        ) => Some(crate::region_boolean::clone_boundary_contours(&second_view)),
        (
            crate::region_boolean::BoundaryContainmentRelation::SecondContainsFirst,
            BooleanOp::Intersection,
        ) => Some(crate::region_boolean::clone_boundary_contours(&first_view)),
        (
            crate::region_boolean::BoundaryContainmentRelation::SecondContainsFirst,
            BooleanOp::Difference,
        ) => Some(Vec::new()),
        (
            crate::region_boolean::BoundaryContainmentRelation::Equivalent,
            BooleanOp::Union | BooleanOp::Intersection,
        ) => Some(crate::region_boolean::clone_boundary_contours(&first_view)),
        (
            crate::region_boolean::BoundaryContainmentRelation::Equivalent,
            BooleanOp::Difference | BooleanOp::Xor,
        ) => Some(Vec::new()),
        _ => None,
    }
}

fn containment_region_prepared(
    first: &PreparedRegionView2<'_>,
    second: &PreparedRegionView2<'_>,
    op: BooleanOp,
    relation: crate::region_boolean::BoundaryContainmentRelation,
) -> Option<Region2> {
    let first_view = first.as_region_view();
    let second_view = second.as_region_view();
    match (relation, op) {
        (
            crate::region_boolean::BoundaryContainmentRelation::FirstContainsSecond,
            BooleanOp::Union,
        ) => Some(crate::region_boolean::clone_region(&first_view)),
        (
            crate::region_boolean::BoundaryContainmentRelation::FirstContainsSecond,
            BooleanOp::Intersection,
        ) => Some(crate::region_boolean::clone_region(&second_view)),
        (
            crate::region_boolean::BoundaryContainmentRelation::SecondContainsFirst,
            BooleanOp::Union,
        ) => Some(crate::region_boolean::clone_region(&second_view)),
        (
            crate::region_boolean::BoundaryContainmentRelation::SecondContainsFirst,
            BooleanOp::Intersection,
        ) => Some(crate::region_boolean::clone_region(&first_view)),
        (
            crate::region_boolean::BoundaryContainmentRelation::SecondContainsFirst,
            BooleanOp::Difference,
        ) => Some(Region2::empty()),
        (
            crate::region_boolean::BoundaryContainmentRelation::Equivalent,
            BooleanOp::Union | BooleanOp::Intersection,
        ) => Some(crate::region_boolean::clone_region(&first_view)),
        (
            crate::region_boolean::BoundaryContainmentRelation::Equivalent,
            BooleanOp::Difference | BooleanOp::Xor,
        ) => Some(Region2::empty()),
        _ => None,
    }
}

fn containment_difference_boundary_contours_prepared(
    first: &PreparedRegionView2<'_>,
    second: &PreparedRegionView2<'_>,
    fill_rule: FillRule,
    policy: &CurvePolicy,
) -> CurveResult<Classification<Vec<Contour2>>> {
    let first_view = first.as_region_view();
    let second_view = second.as_region_view();
    let intersections = first.intersect_prepared_region(second, policy)?;
    let fragments = match intersections.split_regions(&first_view, &second_view, policy)? {
        Classification::Decided(fragments) => fragments,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let selection = match classify_fragments_with_prepared_regions(
        &fragments,
        first,
        second,
        BooleanOp::Difference,
        policy,
    )? {
        Classification::Decided(selection) => selection,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };

    crate::region_boolean::boundary_contours_dropping_unresolved(
        &fragments, &selection, fill_rule, policy,
    )
}

fn boundary_contact_boundary_contours_prepared(
    first: &PreparedRegionView2<'_>,
    second: &PreparedRegionView2<'_>,
    op: BooleanOp,
    fill_rule: FillRule,
    policy: &CurvePolicy,
    kind: PreparedBoundaryContactKind,
) -> CurveResult<Classification<Vec<Contour2>>> {
    let first_view = first.as_region_view();
    let second_view = second.as_region_view();
    Ok(Classification::Decided(match op {
        BooleanOp::Union | BooleanOp::Xor => match kind {
            PreparedBoundaryContactKind::PointOnly => {
                let mut contours = crate::region_boolean::clone_boundary_contours(&first_view);
                contours.extend(crate::region_boolean::clone_boundary_contours(&second_view));
                contours
            }
            PreparedBoundaryContactKind::Overlap => {
                return boundary_overlap_union_contours_prepared(
                    first, second, op, fill_rule, policy,
                );
            }
        },
        BooleanOp::Intersection => Vec::new(),
        BooleanOp::Difference => crate::region_boolean::clone_boundary_contours(&first_view),
    }))
}

fn boundary_overlap_union_contours_prepared(
    first: &PreparedRegionView2<'_>,
    second: &PreparedRegionView2<'_>,
    op: BooleanOp,
    fill_rule: FillRule,
    policy: &CurvePolicy,
) -> CurveResult<Classification<Vec<Contour2>>> {
    let first_view = first.as_region_view();
    let second_view = second.as_region_view();
    let intersections = first.intersect_prepared_region(second, policy)?;
    let fragments = match intersections.split_regions(&first_view, &second_view, policy)? {
        Classification::Decided(fragments) => fragments,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let selection =
        match classify_fragments_with_prepared_regions(&fragments, first, second, op, policy)? {
            Classification::Decided(selection) => selection,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };

    crate::region_boolean::boundary_contours_dropping_unresolved(
        &fragments, &selection, fill_rule, policy,
    )
}

fn boundary_contact_region_prepared(
    first: &PreparedRegionView2<'_>,
    second: &PreparedRegionView2<'_>,
    op: BooleanOp,
    fill_rule: FillRule,
    policy: &CurvePolicy,
    kind: PreparedBoundaryContactKind,
) -> CurveResult<Classification<Region2>> {
    let first_view = first.as_region_view();
    let second_view = second.as_region_view();
    Ok(Classification::Decided(match op {
        BooleanOp::Union | BooleanOp::Xor => match kind {
            PreparedBoundaryContactKind::PointOnly => {
                crate::region_boolean::merge_disjoint_region_bins(
                    crate::region_boolean::clone_region(&first_view),
                    crate::region_boolean::clone_region(&second_view),
                )
            }
            PreparedBoundaryContactKind::Overlap => {
                return match boundary_overlap_union_contours_prepared(
                    first, second, op, fill_rule, policy,
                )? {
                    Classification::Decided(contours) => {
                        Region2::from_boundary_contours(contours, policy)
                    }
                    Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
                };
            }
        },
        BooleanOp::Intersection => Region2::empty(),
        BooleanOp::Difference => crate::region_boolean::clone_region(&first_view),
    }))
}

pub(crate) fn classify_fragments_with_prepared_regions(
    fragments: &RegionFragmentSet,
    first: &PreparedRegionView2<'_>,
    second: &PreparedRegionView2<'_>,
    op: BooleanOp,
    policy: &CurvePolicy,
) -> CurveResult<Classification<BooleanFragmentSelection>> {
    fragments.classify_for_boolean_with_point_classifier(op, policy, |source_side, sample| {
        match source_side {
            RegionSide::First => second.classify_point(sample, policy),
            RegionSide::Second => first.classify_point(sample, policy),
        }
    })
}

fn xor_region_by_prepared_difference_union(
    first: &PreparedRegionView2<'_>,
    second: &PreparedRegionView2<'_>,
    fill_rule: FillRule,
    policy: &CurvePolicy,
) -> CurveResult<Classification<Region2>> {
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
