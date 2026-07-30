//! Region-level boolean boundary pipeline.
//!
//! The routines here compose the existing event, split, classify, and boundary
//! traversal stages. Boundary-only contacts and exactly paired coincident
//! fragments are regularized from certified local fill state; incomplete or
//! ambiguous overlap evidence remains explicit uncertainty.

use crate::classify::compare_reals;
use crate::region_crossing_winding::RegionLineCrossingWindingIndex;
use crate::region_fragments::split_single_material_line_regions_compact;
use crate::{
    Aabb2, BooleanBoundaryLoopSet, BooleanFragmentAction, BooleanFragmentClassification,
    BooleanFragmentSelection, BooleanOp, BulgeVertex2, Classification, Contour2,
    ContourIntersection, CurveError, CurvePolicy, CurveResult, FillRule, IntersectionKind,
    LineArcRegion2, Point2, Real, RegionFragmentSet, RegionIntersectionSet, RegionPointLocation,
    RegionSide, RegionView2, Segment2, UncertaintyReason,
};
use std::cmp::Ordering;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum BoundaryContactKind {
    PointOnly,
    Overlap,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum BoundaryContainmentRelation {
    FirstContainsSecond,
    SecondContainsFirst,
    Equivalent,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum BoundaryContactResolution {
    BoundaryOnly(BoundaryContactKind),
    Containment {
        relation: BoundaryContainmentRelation,
        contact: BoundaryContactKind,
    },
}

#[derive(Clone, Debug)]
struct AxisRect {
    min_x: Real,
    min_y: Real,
    max_x: Real,
    max_y: Real,
}

impl AxisRect {
    fn from_view(
        region: &RegionView2<'_>,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<Option<Self>>> {
        if region.material_contours().len() != 1 || !region.hole_contours().is_empty() {
            return Ok(Classification::Decided(None));
        }
        let contour = region.material_contours()[0];
        if contour.segments().len() != 4 {
            return Ok(Classification::Decided(None));
        }
        for segment in contour.segments() {
            let Segment2::Line(line) = segment else {
                return Ok(Classification::Decided(None));
            };
            let same_x = real_eq(line.start().x(), line.end().x(), policy);
            let same_y = real_eq(line.start().y(), line.end().y(), policy);
            match (same_x, same_y) {
                (Some(true), Some(false)) | (Some(false), Some(true)) => {}
                (Some(_), Some(_)) => return Ok(Classification::Decided(None)),
                _ => return Ok(Classification::Uncertain(UncertaintyReason::Ordering)),
            }
        }

        let bbox = match Aabb2::from_contour(contour, policy) {
            Ok(Classification::Decided(bbox)) => bbox,
            Ok(Classification::Uncertain(reason)) => return Ok(Classification::Uncertain(reason)),
            Err(err) => return Err(err),
        };
        Ok(Classification::Decided(Some(Self {
            min_x: bbox.min_x().clone(),
            min_y: bbox.min_y().clone(),
            max_x: bbox.max_x().clone(),
            max_y: bbox.max_y().clone(),
        })))
    }
}

fn real_eq(left: &Real, right: &Real, policy: &CurvePolicy) -> Option<bool> {
    compare_reals(left, right, policy).map(|ordering| ordering == Ordering::Equal)
}

fn real_min(left: &Real, right: &Real, policy: &CurvePolicy) -> Option<Real> {
    match compare_reals(left, right, policy)? {
        Ordering::Less | Ordering::Equal => Some(left.clone()),
        Ordering::Greater => Some(right.clone()),
    }
}

fn real_max(left: &Real, right: &Real, policy: &CurvePolicy) -> Option<Real> {
    match compare_reals(left, right, policy)? {
        Ordering::Less | Ordering::Equal => Some(right.clone()),
        Ordering::Greater => Some(left.clone()),
    }
}

fn real_lt(left: &Real, right: &Real, policy: &CurvePolicy) -> Option<bool> {
    compare_reals(left, right, policy).map(|ordering| ordering == Ordering::Less)
}

fn rect_from_bounds(min_x: Real, min_y: Real, max_x: Real, max_y: Real) -> Option<Contour2> {
    if min_x == max_x || min_y == max_y {
        return None;
    }
    Contour2::from_bulge_vertices(&[
        BulgeVertex2::new(Point2::new(min_x.clone(), min_y.clone()), Real::zero()),
        BulgeVertex2::new(Point2::new(max_x.clone(), min_y.clone()), Real::zero()),
        BulgeVertex2::new(Point2::new(max_x.clone(), max_y.clone()), Real::zero()),
        BulgeVertex2::new(Point2::new(min_x.clone(), max_y.clone()), Real::zero()),
    ])
    .ok()
}

// Regularizes the degenerate strip case where both input boundaries share a
// full collinear span. That case is the canonical failure mode highlighted by
// the degenerate-intersection clipping model, and it must be resolved in the
// geometry kernel so CAD callers receive ordinary LineArcRegion2 values rather than
// crate-local workarounds.
pub(crate) fn coextensive_axis_rect_region_boolean(
    first: &RegionView2<'_>,
    second: &RegionView2<'_>,
    op: BooleanOp,
    policy: &CurvePolicy,
) -> CurveResult<Classification<Option<LineArcRegion2>>> {
    let first = match AxisRect::from_view(first, policy)? {
        Classification::Decided(Some(rect)) => rect,
        Classification::Decided(None) => return Ok(Classification::Decided(None)),
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let second = match AxisRect::from_view(second, policy)? {
        Classification::Decided(Some(rect)) => rect,
        Classification::Decided(None) => return Ok(Classification::Decided(None)),
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };

    let same_y = real_eq(&first.min_y, &second.min_y, policy) == Some(true)
        && real_eq(&first.max_y, &second.max_y, policy) == Some(true);
    let same_x = real_eq(&first.min_x, &second.min_x, policy) == Some(true)
        && real_eq(&first.max_x, &second.max_x, policy) == Some(true);
    if !same_y && !same_x {
        return Ok(Classification::Decided(None));
    }

    if same_y {
        return match strip_boolean_region(
            first.min_x,
            first.max_x,
            second.min_x,
            second.max_x,
            first.min_y,
            first.max_y,
            true,
            op,
            policy,
        ) {
            Classification::Decided(region) => Ok(Classification::Decided(Some(region))),
            Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
        };
    }

    match strip_boolean_region(
        first.min_y,
        first.max_y,
        second.min_y,
        second.max_y,
        first.min_x,
        first.max_x,
        false,
        op,
        policy,
    ) {
        Classification::Decided(region) => Ok(Classification::Decided(Some(region))),
        Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
    }
}

#[allow(clippy::too_many_arguments)]
fn strip_boolean_region(
    first_min: Real,
    first_max: Real,
    second_min: Real,
    second_max: Real,
    cross_min: Real,
    cross_max: Real,
    horizontal: bool,
    op: BooleanOp,
    policy: &CurvePolicy,
) -> Classification<LineArcRegion2> {
    let overlap_min = real_max(&first_min, &second_min, policy).ok_or(UncertaintyReason::Ordering);
    let Ok(overlap_min) = overlap_min else {
        return Classification::Uncertain(overlap_min.unwrap_err());
    };
    let overlap_max = real_min(&first_max, &second_max, policy).ok_or(UncertaintyReason::Ordering);
    let Ok(overlap_max) = overlap_max else {
        return Classification::Uncertain(overlap_max.unwrap_err());
    };
    let overlaps = real_lt(&overlap_min, &overlap_max, policy).ok_or(UncertaintyReason::Ordering);
    let Ok(overlaps) = overlaps else {
        return Classification::Uncertain(overlaps.unwrap_err());
    };
    if !overlaps {
        let touches = match real_eq(&overlap_min, &overlap_max, policy) {
            Some(touches) => touches,
            None => return Classification::Uncertain(UncertaintyReason::Ordering),
        };
        if touches && matches!(op, BooleanOp::Union | BooleanOp::Xor) {
            // A zero-width overlap here means two same-width strips share an
            // entire edge. Regularized polygon clipping removes that internal
            // edge for union and symmetric difference; see the degenerate-intersection clipping model "Clipping simple polygons with degenerate
            // intersections". Keeping this in the rectangle fast path
            // makes it agree with the general shared-boundary resolver instead
            // of leaking two touching material contours.
            let min = real_min(&first_min, &second_min, policy).ok_or(UncertaintyReason::Ordering);
            let Ok(min) = min else {
                return Classification::Uncertain(min.unwrap_err());
            };
            let max = real_max(&first_max, &second_max, policy).ok_or(UncertaintyReason::Ordering);
            let Ok(max) = max else {
                return Classification::Uncertain(max.unwrap_err());
            };
            return required_strip_region(vec![(min, cross_min, max, cross_max, horizontal)]);
        }
        return match op {
            BooleanOp::Union | BooleanOp::Xor => required_strip_region(vec![
                (
                    first_min,
                    cross_min.clone(),
                    first_max,
                    cross_max.clone(),
                    horizontal,
                ),
                (second_min, cross_min, second_max, cross_max, horizontal),
            ]),
            BooleanOp::Difference => required_strip_region(vec![(
                first_min, cross_min, first_max, cross_max, horizontal,
            )]),
            BooleanOp::Intersection => Classification::Decided(LineArcRegion2::empty()),
        };
    }

    let contours = match op {
        BooleanOp::Union => {
            let min = real_min(&first_min, &second_min, policy).ok_or(UncertaintyReason::Ordering);
            let Ok(min) = min else {
                return Classification::Uncertain(min.unwrap_err());
            };
            let max = real_max(&first_max, &second_max, policy).ok_or(UncertaintyReason::Ordering);
            let Ok(max) = max else {
                return Classification::Uncertain(max.unwrap_err());
            };
            match required_strip_rects(vec![(min, cross_min, max, cross_max, horizontal)]) {
                Classification::Decided(contours) => contours,
                Classification::Uncertain(reason) => return Classification::Uncertain(reason),
            }
        }
        BooleanOp::Intersection => {
            match required_strip_rects(vec![(
                overlap_min,
                cross_min,
                overlap_max,
                cross_max,
                horizontal,
            )]) {
                Classification::Decided(contours) => contours,
                Classification::Uncertain(reason) => return Classification::Uncertain(reason),
            }
        }
        BooleanOp::Difference => match strip_difference_contours(
            first_min, first_max, second_min, second_max, cross_min, cross_max, horizontal, policy,
        ) {
            Classification::Decided(contours) => contours,
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        },
        BooleanOp::Xor => {
            let mut contours = match strip_difference_contours(
                first_min.clone(),
                first_max.clone(),
                second_min.clone(),
                second_max.clone(),
                cross_min.clone(),
                cross_max.clone(),
                horizontal,
                policy,
            ) {
                Classification::Decided(contours) => contours,
                Classification::Uncertain(reason) => return Classification::Uncertain(reason),
            };
            let second_contours = match strip_difference_contours(
                second_min, second_max, first_min, first_max, cross_min, cross_max, horizontal,
                policy,
            ) {
                Classification::Decided(contours) => contours,
                Classification::Uncertain(reason) => return Classification::Uncertain(reason),
            };
            contours.extend(second_contours);
            contours
        }
    };
    Classification::Decided(LineArcRegion2::from_material_contours(contours))
}

type StripRectBounds = (Real, Real, Real, Real, bool);

fn required_strip_region(bounds: Vec<StripRectBounds>) -> Classification<LineArcRegion2> {
    match required_strip_rects(bounds) {
        Classification::Decided(contours) => {
            Classification::Decided(LineArcRegion2::from_material_contours(contours))
        }
        Classification::Uncertain(reason) => Classification::Uncertain(reason),
    }
}

fn required_strip_rects(bounds: Vec<StripRectBounds>) -> Classification<Vec<Contour2>> {
    let mut contours = Vec::with_capacity(bounds.len());
    for bound in bounds {
        match required_strip_rect(bound) {
            Classification::Decided(contour) => contours.push(contour),
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        }
    }
    Classification::Decided(contours)
}

fn required_strip_rect(bounds: StripRectBounds) -> Classification<Contour2> {
    let (along_min, cross_min, along_max, cross_max, horizontal) = bounds;
    match oriented_strip_rect(along_min, cross_min, along_max, cross_max, horizontal) {
        Some(contour) => Classification::Decided(contour),
        None => Classification::Uncertain(UncertaintyReason::Unsupported),
    }
}

#[allow(clippy::too_many_arguments)]
fn strip_difference_contours(
    first_min: Real,
    first_max: Real,
    second_min: Real,
    second_max: Real,
    cross_min: Real,
    cross_max: Real,
    horizontal: bool,
    policy: &CurvePolicy,
) -> Classification<Vec<Contour2>> {
    let mut contours = Vec::new();
    let left_kept = real_lt(&first_min, &second_min, policy).ok_or(UncertaintyReason::Ordering);
    let Ok(left_kept) = left_kept else {
        return Classification::Uncertain(left_kept.unwrap_err());
    };
    if left_kept {
        let end = real_min(&first_max, &second_min, policy).ok_or(UncertaintyReason::Ordering);
        let Ok(end) = end else {
            return Classification::Uncertain(end.unwrap_err());
        };
        let has_positive_width = match real_lt(&first_min, &end, policy) {
            Some(has_positive_width) => has_positive_width,
            None => return Classification::Uncertain(UncertaintyReason::Ordering),
        };
        if has_positive_width {
            match required_strip_rect((
                first_min.clone(),
                cross_min.clone(),
                end,
                cross_max.clone(),
                horizontal,
            )) {
                Classification::Decided(contour) => contours.push(contour),
                Classification::Uncertain(reason) => return Classification::Uncertain(reason),
            }
        }
    }
    let right_kept = real_lt(&second_max, &first_max, policy).ok_or(UncertaintyReason::Ordering);
    let Ok(right_kept) = right_kept else {
        return Classification::Uncertain(right_kept.unwrap_err());
    };
    if right_kept {
        let start = real_max(&first_min, &second_max, policy).ok_or(UncertaintyReason::Ordering);
        let Ok(start) = start else {
            return Classification::Uncertain(start.unwrap_err());
        };
        let has_positive_width = match real_lt(&start, &first_max, policy) {
            Some(has_positive_width) => has_positive_width,
            None => return Classification::Uncertain(UncertaintyReason::Ordering),
        };
        if has_positive_width {
            match required_strip_rect((start, cross_min, first_max, cross_max, horizontal)) {
                Classification::Decided(contour) => contours.push(contour),
                Classification::Uncertain(reason) => return Classification::Uncertain(reason),
            }
        }
    }
    Classification::Decided(contours)
}

fn oriented_strip_rect(
    along_min: Real,
    cross_min: Real,
    along_max: Real,
    cross_max: Real,
    horizontal: bool,
) -> Option<Contour2> {
    if horizontal {
        rect_from_bounds(along_min, cross_min, along_max, cross_max)
    } else {
        rect_from_bounds(cross_min, along_min, cross_max, along_max)
    }
}

impl LineArcRegion2 {
    /// Computes closed boolean boundary loops against another owned region.
    ///
    /// This is a convenience wrapper over [`RegionView2::boolean_boundary_loops`].
    pub fn boolean_boundary_loops(
        &self,
        other: &Self,
        op: BooleanOp,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<BooleanBoundaryLoopSet>> {
        self.as_view()
            .boolean_boundary_loops(&other.as_view(), op, policy)
    }

    /// Computes checked boolean boundary contours against another owned region.
    ///
    /// The returned contours are closed result boundaries. They are not yet
    /// assigned to material or hole bins; that role assignment belongs to the
    /// later nesting pass.
    pub fn boolean_boundary_contours(
        &self,
        other: &Self,
        op: BooleanOp,
        fill_rule: FillRule,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<Vec<Contour2>>> {
        self.as_view()
            .boolean_boundary_contours(&other.as_view(), op, fill_rule, policy)
    }

    /// Computes a role-assigned boolean region against another owned region.
    ///
    /// The result is available only when the current boundary pipeline can
    /// produce closed contours and the nesting pass can classify those contours
    /// without boundary ambiguity.
    pub fn boolean_region(
        &self,
        other: &Self,
        op: BooleanOp,
        fill_rule: FillRule,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<Self>> {
        self.as_view()
            .boolean_region(&other.as_view(), op, fill_rule, policy)
    }
}

impl RegionView2<'_> {
    /// Computes closed boolean boundary loops against another region view.
    ///
    /// Algorithm note: this method wires together the standard polygon clipping
    /// stages: collect intersection events, split input boundaries at those
    /// events, classify each fragment against the opposite operand, and traverse
    /// selected directed fragments into closed loops. `hypercurve` keeps each
    /// stage explicit so uncertain tangencies, shared boundaries, and branch
    /// vertices can stop the pipeline instead of being resolved by a global
    /// epsilon.
    pub fn boolean_boundary_loops(
        &self,
        other: &RegionView2<'_>,
        op: BooleanOp,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<BooleanBoundaryLoopSet>> {
        boolean_boundary_loops_between(self, other, op, policy)
    }

    /// Computes checked boolean boundary contours against another region view.
    ///
    /// The contours are produced only after every selected boundary chain closes.
    /// Open chains and unresolved shared boundaries are returned as uncertainty.
    pub fn boolean_boundary_contours(
        &self,
        other: &RegionView2<'_>,
        op: BooleanOp,
        fill_rule: FillRule,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<Vec<Contour2>>> {
        boolean_boundary_contours_between(self, other, op, fill_rule, policy)
    }

    /// Computes a role-assigned boolean region against another region view.
    ///
    /// After boundary traversal, closed output contours are assigned to material
    /// and hole bins by containment depth. Any boundary result during nesting
    /// remains explicit uncertainty because a boundary touch means the output
    /// contour graph still needs a degeneracy-specific resolver.
    pub fn boolean_region(
        &self,
        other: &RegionView2<'_>,
        op: BooleanOp,
        fill_rule: FillRule,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<LineArcRegion2>> {
        boolean_region_between(self, other, op, fill_rule, policy)
    }
}

pub(crate) fn boolean_boundary_loops_between(
    first: &RegionView2<'_>,
    second: &RegionView2<'_>,
    op: BooleanOp,
    policy: &CurvePolicy,
) -> CurveResult<Classification<BooleanBoundaryLoopSet>> {
    match boolean_boundary_between(
        first,
        second,
        op,
        FillRule::NonZero,
        None,
        policy,
        BooleanBoundaryOutputKind::Loops,
    )? {
        Classification::Decided(output) => Ok(Classification::Decided(
            output
                .into_loops()
                .expect("boundary-loop query requests loop output"),
        )),
        Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
    }
}

pub(crate) fn boolean_boundary_contours_between(
    first: &RegionView2<'_>,
    second: &RegionView2<'_>,
    op: BooleanOp,
    fill_rule: FillRule,
    policy: &CurvePolicy,
) -> CurveResult<Classification<Vec<Contour2>>> {
    match boolean_boundary_between(
        first,
        second,
        op,
        fill_rule,
        None,
        policy,
        BooleanBoundaryOutputKind::Contours,
    )? {
        Classification::Decided(output) => Ok(Classification::Decided(
            output
                .into_contours()
                .expect("boundary-contour query requests contour output"),
        )),
        Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
    }
}
fn xor_boundary_contours_by_region(
    first: &RegionView2<'_>,
    second: &RegionView2<'_>,
    fill_rule: FillRule,
    policy: &CurvePolicy,
) -> CurveResult<Classification<Vec<Contour2>>> {
    // The checked-contour API can express the boundary loops of a symmetric
    // difference, but it cannot attach material/hole roles to them. Build the
    // role-aware region first, then expose its checked boundary contours.
    // This follows the segment-selection set identity for polygon booleans
    // while keeping ambiguous shared boundaries out of the direct traversal
    // graph until the general overlap/branch resolver lands.
    match xor_region_by_difference_union(first, second, fill_rule, policy)? {
        Classification::Decided(region) => Ok(Classification::Decided(clone_boundary_contours(
            &region.as_view(),
        ))),
        Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
    }
}

pub(crate) fn boolean_region_between(
    first: &RegionView2<'_>,
    second: &RegionView2<'_>,
    op: BooleanOp,
    fill_rule: FillRule,
    policy: &CurvePolicy,
) -> CurveResult<Classification<LineArcRegion2>> {
    boolean_region_between_impl(first, second, op, fill_rule, policy)
}

fn boolean_region_between_impl(
    first: &RegionView2<'_>,
    second: &RegionView2<'_>,
    op: BooleanOp,
    fill_rule: FillRule,
    policy: &CurvePolicy,
) -> CurveResult<Classification<LineArcRegion2>> {
    if let Some(region) = coincident_hole_component_boolean(first, second, op) {
        return Ok(Classification::Decided(region));
    }
    let boundary_events =
        crate::region_events::intersect_region_views_point_only(first, second, policy)?;
    if let Some(region) = retained_offset_region_boolean(first, second, op, policy) {
        return Ok(Classification::Decided(region));
    }
    if op == BooleanOp::Xor {
        return xor_region_by_difference_union(first, second, fill_rule, policy);
    }
    let boundary_output = match boolean_boundary_between(
        first,
        second,
        op,
        fill_rule,
        Some(&boundary_events),
        policy,
        BooleanBoundaryOutputKind::Contours,
    )? {
        Classification::Decided(result) => result,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let contours = boundary_output
        .into_contours()
        .expect("region Boolean requests contour boundary output");
    if boundary_events.overlap_event_count() == 0 {
        LineArcRegion2::from_directed_boolean_boundary_contours(contours, policy)
    } else {
        LineArcRegion2::from_validated_boundary_contours(contours, policy)
    }
}

fn coincident_hole_component_boolean(
    first: &RegionView2<'_>,
    second: &RegionView2<'_>,
    op: BooleanOp,
) -> Option<LineArcRegion2> {
    if let Some(region) = coincident_hole_component_boolean_ordered(first, second, op, true) {
        return Some(region);
    }
    coincident_hole_component_boolean_ordered(second, first, op, false)
}

fn coincident_hole_component_boolean_ordered(
    container: &RegionView2<'_>,
    component: &RegionView2<'_>,
    op: BooleanOp,
    container_is_first: bool,
) -> Option<LineArcRegion2> {
    if container.material_contours().len() != 1
        || container.hole_contours().len() != 1
        || component.material_contours().len() != 1
        || !component.hole_contours().is_empty()
        || !contours_have_same_exact_boundary(
            container.hole_contours()[0],
            component.material_contours()[0],
        )
    {
        return None;
    }

    let filled_container =
        || LineArcRegion2::from_material_contours(vec![container.material_contours()[0].clone()]);
    let container_region = || clone_region_view(container);
    let component_region = || clone_region_view(component);
    Some(match op {
        BooleanOp::Union | BooleanOp::Xor => filled_container(),
        BooleanOp::Intersection => LineArcRegion2::empty(),
        BooleanOp::Difference if container_is_first => container_region(),
        BooleanOp::Difference => component_region(),
    })
}

pub(crate) fn retained_offset_region_boolean(
    first: &RegionView2<'_>,
    second: &RegionView2<'_>,
    op: BooleanOp,
    policy: &CurvePolicy,
) -> Option<LineArcRegion2> {
    use crate::contour::RetainedContourOffsetRelation2::{
        FirstContainsSecond, SecondContainsFirst,
    };

    if first.material_contours().len() != 1
        || second.material_contours().len() != 1
        || !first.hole_contours().is_empty()
        || !second.hole_contours().is_empty()
    {
        return None;
    }
    let first_contour = first.material_contours()[0];
    let second_contour = second.material_contours()[0];
    let relation = first_contour.retained_offset_relation(second_contour, policy)?;

    Some(match (relation, op) {
        (FirstContainsSecond, BooleanOp::Union) => clone_region_view(first),
        (FirstContainsSecond, BooleanOp::Intersection) => clone_region_view(second),
        (FirstContainsSecond, BooleanOp::Difference | BooleanOp::Xor) => {
            LineArcRegion2::new(vec![first_contour.clone()], vec![second_contour.clone()])
        }
        (SecondContainsFirst, BooleanOp::Union) => clone_region_view(second),
        (SecondContainsFirst, BooleanOp::Intersection) => clone_region_view(first),
        (SecondContainsFirst, BooleanOp::Difference) => LineArcRegion2::empty(),
        (SecondContainsFirst, BooleanOp::Xor) => {
            LineArcRegion2::new(vec![second_contour.clone()], vec![first_contour.clone()])
        }
        _ => return None,
    })
}

fn clone_region_view(region: &RegionView2<'_>) -> LineArcRegion2 {
    LineArcRegion2::new(
        region
            .material_contours()
            .iter()
            .map(|contour| (*contour).clone())
            .collect(),
        region
            .hole_contours()
            .iter()
            .map(|contour| (*contour).clone())
            .collect(),
    )
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum BooleanBoundaryOutputKind {
    Loops,
    Contours,
}

pub(crate) enum BooleanBoundaryOutput {
    Loops(BooleanBoundaryLoopSet),
    Contours(Vec<Contour2>),
}

impl BooleanBoundaryOutput {
    fn from_contours(
        contours: Vec<Contour2>,
        output_kind: BooleanBoundaryOutputKind,
    ) -> CurveResult<Self> {
        match output_kind {
            BooleanBoundaryOutputKind::Loops => {
                BooleanBoundaryLoopSet::from_contours(contours).map(Self::Loops)
            }
            BooleanBoundaryOutputKind::Contours => Ok(Self::Contours(contours)),
        }
    }

    pub(crate) fn into_loops(self) -> Option<BooleanBoundaryLoopSet> {
        match self {
            Self::Loops(loops) => Some(loops),
            Self::Contours(_) => None,
        }
    }

    pub(crate) fn into_contours(self) -> Option<Vec<Contour2>> {
        match self {
            Self::Loops(_) => None,
            Self::Contours(contours) => Some(contours),
        }
    }
}

pub(crate) fn boolean_boundary_between(
    first: &RegionView2<'_>,
    second: &RegionView2<'_>,
    op: BooleanOp,
    fill_rule: FillRule,
    known_boundary_events: Option<&RegionIntersectionSet>,
    policy: &CurvePolicy,
    output_kind: BooleanBoundaryOutputKind,
) -> CurveResult<Classification<BooleanBoundaryOutput>> {
    if same_region_view(first, second) {
        return Ok(Classification::Decided(
            BooleanBoundaryOutput::from_contours(
                match op {
                    BooleanOp::Union | BooleanOp::Intersection => clone_boundary_contours(first),
                    BooleanOp::Difference | BooleanOp::Xor => Vec::new(),
                },
                output_kind,
            )?,
        ));
    }
    if first.is_empty() || second.is_empty() {
        return Ok(Classification::Decided(
            BooleanBoundaryOutput::from_contours(
                empty_operand_boundary_contours(first, second, op),
                output_kind,
            )?,
        ));
    }
    match coextensive_axis_rect_region_boolean(first, second, op, policy)? {
        Classification::Decided(Some(region)) => {
            return Ok(Classification::Decided(
                BooleanBoundaryOutput::from_contours(
                    clone_boundary_contours(&region.as_view()),
                    output_kind,
                )?,
            ));
        }
        Classification::Decided(None) => {}
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    }
    let owned_boundary_events;
    let boundary_events = match known_boundary_events {
        Some(boundary_events) => boundary_events,
        None => {
            owned_boundary_events = if output_kind == BooleanBoundaryOutputKind::Loops {
                first.intersect_region(second, policy)?
            } else {
                crate::region_events::intersect_region_views_point_only(first, second, policy)?
            };
            &owned_boundary_events
        }
    };
    match boundary_contact_resolution_from_intersections(first, second, boundary_events, policy)? {
        Classification::Decided(Some(BoundaryContactResolution::BoundaryOnly(kind))) => {
            return match boundary_contact_boundary_contours(
                first, second, op, fill_rule, policy, kind,
            )? {
                Classification::Decided(contours) => Ok(Classification::Decided(
                    BooleanBoundaryOutput::from_contours(contours, output_kind)?,
                )),
                Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
            };
        }
        Classification::Decided(Some(BoundaryContactResolution::Containment {
            relation,
            contact,
        })) => {
            if let Some(contours) = containment_boundary_contours(first, second, op, relation) {
                return Ok(Classification::Decided(
                    BooleanBoundaryOutput::from_contours(contours, output_kind)?,
                ));
            }
            if relation == BoundaryContainmentRelation::FirstContainsSecond
                && contact == BoundaryContactKind::Overlap
                && op == BooleanOp::Difference
            {
                return match containment_difference_boundary_contours(
                    first, second, fill_rule, policy,
                )? {
                    Classification::Decided(contours) => Ok(Classification::Decided(
                        BooleanBoundaryOutput::from_contours(contours, output_kind)?,
                    )),
                    Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
                };
            }
        }
        Classification::Decided(None) => {
            if op == BooleanOp::Union && region_boundary_has_overlap_in(boundary_events) {
                return match boundary_overlap_union_contours(first, second, op, fill_rule, policy)?
                {
                    Classification::Decided(contours) => Ok(Classification::Decided(
                        BooleanBoundaryOutput::from_contours(contours, output_kind)?,
                    )),
                    Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
                };
            }
        }
        Classification::Uncertain(_) => {}
    }
    if op == BooleanOp::Xor {
        return match xor_boundary_contours_by_region(first, second, fill_rule, policy)? {
            Classification::Decided(contours) => Ok(Classification::Decided(
                BooleanBoundaryOutput::from_contours(contours, output_kind)?,
            )),
            Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
        };
    }

    // Successful splitting excludes unresolved segment relations. When the
    // complete event set also has no positive-dimensional overlap, every
    // opposite-boundary contact is a retained marker endpoint; strict interior
    // samples therefore cannot lie on that boundary.
    let split_interiors_are_off_opposite_boundary =
        boundary_events.overlap_event_count() == 0 && boundary_events.uncertain_event_count() == 0;
    let crossing_windings = if split_interiors_are_off_opposite_boundary
        && RegionLineCrossingWindingIndex::event_set_may_support_propagation(boundary_events)
    {
        RegionLineCrossingWindingIndex::from_intersections(first, second, boundary_events, policy)
    } else {
        None
    };
    if let Some(crossing_windings) = crossing_windings {
        let endpoint_contacts =
            crate::region_events::RegionPointEndpointContactIndex::from_intersections(
                boundary_events,
                policy,
            );
        'compact: {
            let compact_fragments = match split_single_material_line_regions_compact(
                first,
                second,
                boundary_events,
                &crossing_windings,
                policy,
            )? {
                Classification::Decided(fragments) => fragments,
                Classification::Uncertain(UncertaintyReason::Unsupported) => break 'compact,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let compact_selection = compact_fragments
                .classify_for_boolean_with_line_crossing_winding(
                    first,
                    second,
                    op,
                    policy,
                    &endpoint_contacts,
                    &crossing_windings,
                    |source_side, sample| match source_side {
                        RegionSide::First => {
                            crate::contour::line_contour_winding_assuming_off_boundary(
                                second.material_contours()[0],
                                sample,
                                policy,
                            )
                        }
                        RegionSide::Second => {
                            crate::contour::line_contour_winding_assuming_off_boundary(
                                first.material_contours()[0],
                                sample,
                                policy,
                            )
                        }
                    },
                )?;
            if let Some(compact_selection) = compact_selection {
                let selection = match compact_selection {
                    Classification::Decided(selection) => selection,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                if output_kind == BooleanBoundaryOutputKind::Contours {
                    match selection.endpoint_chain_indices_from_compact_split(
                        &compact_fragments,
                        first,
                        second,
                        &crossing_windings,
                        policy,
                    )? {
                        Classification::Decided(Some(chain_indices)) => {
                            let contours = match selection.emit_contours_from_owned_compact_split(
                                compact_fragments,
                                first,
                                second,
                                chain_indices,
                                fill_rule,
                                &crossing_windings,
                            )? {
                                Classification::Decided(contours) => contours,
                                Classification::Uncertain(reason) => {
                                    return Ok(Classification::Uncertain(reason));
                                }
                            };
                            return Ok(Classification::Decided(BooleanBoundaryOutput::Contours(
                                contours,
                            )));
                        }
                        Classification::Decided(None) => {}
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    }
                }
                if !compact_fragments.parameters_are_materialized(&crossing_windings) {
                    break 'compact;
                }
                let emitted = selection.emit_boundary_fragments_from_owned_compact_split(
                    compact_fragments,
                    first,
                    second,
                    &crossing_windings,
                )?;
                let output = match output_kind {
                    BooleanBoundaryOutputKind::Loops => {
                        let chains = match emitted.into_assembled_chains(policy) {
                            Classification::Decided(chains) => chains,
                            Classification::Uncertain(reason) => {
                                return Ok(Classification::Uncertain(reason));
                            }
                        };
                        match chains.into_closed_loops() {
                            Classification::Decided(loops) => BooleanBoundaryOutput::Loops(loops),
                            Classification::Uncertain(reason) => {
                                return Ok(Classification::Uncertain(reason));
                            }
                        }
                    }
                    BooleanBoundaryOutputKind::Contours => {
                        match emitted.into_assembled_contours(fill_rule, policy)? {
                            Classification::Decided(contours) => {
                                BooleanBoundaryOutput::Contours(contours)
                            }
                            Classification::Uncertain(reason) => {
                                return Ok(Classification::Uncertain(reason));
                            }
                        }
                    }
                };
                return Ok(Classification::Decided(output));
            }
        }
    }

    let fragments = match boundary_events.split_regions(first, second, policy)? {
        Classification::Decided(fragments) => fragments,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    let endpoint_contacts = split_interiors_are_off_opposite_boundary.then(|| {
        crate::region_events::RegionPointEndpointContactIndex::from_intersections(
            boundary_events,
            policy,
        )
    });
    let crossing_windings = if split_interiors_are_off_opposite_boundary
        && RegionLineCrossingWindingIndex::event_set_may_support_propagation(boundary_events)
    {
        RegionLineCrossingWindingIndex::from_intersections(first, second, boundary_events, policy)
    } else {
        None
    };
    let crossing_selection_result = match (&endpoint_contacts, &crossing_windings) {
        (Some(endpoint_contacts), Some(crossing_windings)) => fragments
            .classify_for_boolean_with_line_crossing_winding(
                first,
                second,
                op,
                policy,
                endpoint_contacts,
                crossing_windings,
                |source_side, sample| match source_side {
                    RegionSide::First => {
                        crate::contour::line_contour_winding_assuming_off_boundary(
                            second.material_contours()[0],
                            sample,
                            policy,
                        )
                    }
                    RegionSide::Second => {
                        crate::contour::line_contour_winding_assuming_off_boundary(
                            first.material_contours()[0],
                            sample,
                            policy,
                        )
                    }
                },
            )?,
        _ => None,
    };
    let selection_result = match crossing_selection_result {
        Some(selection_result) => selection_result,
        None => {
            // General fragment classification queries the same immutable
            // operands repeatedly. Retain boxes and prepared predicates only
            // when the once-visiting crossing proof above is unavailable.
            let first_prepared = crate::prepared::RegionQuery2::from_region_view(first, policy);
            let second_prepared = crate::prepared::RegionQuery2::from_region_view(second, policy);
            fragments.classify_for_boolean_with_contacts_and_point_classifier(
                first,
                second,
                op,
                policy,
                endpoint_contacts.as_ref(),
                |source_side, sample| match (split_interiors_are_off_opposite_boundary, source_side)
                {
                    (true, RegionSide::First) => {
                        second_prepared.classify_point_assuming_off_boundary(sample, policy)
                    }
                    (true, RegionSide::Second) => {
                        first_prepared.classify_point_assuming_off_boundary(sample, policy)
                    }
                    (false, RegionSide::First) => second_prepared.classify_point(sample, policy),
                    (false, RegionSide::Second) => first_prepared.classify_point(sample, policy),
                },
            )?
        }
    };
    let selection = match selection_result {
        Classification::Decided(selection) => selection,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let selection = match resolve_owned_shared_boundary_selection(&fragments, selection, op)? {
        Classification::Decided(selection) => selection,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    if output_kind == BooleanBoundaryOutputKind::Contours {
        match selection.endpoint_chain_indices_from_certified_split(&fragments, policy)? {
            Classification::Decided(Some(chain_indices)) => {
                let contours = match selection.emit_contours_from_owned_certified_split(
                    fragments,
                    chain_indices,
                    fill_rule,
                )? {
                    Classification::Decided(contours) => contours,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                return Ok(Classification::Decided(BooleanBoundaryOutput::Contours(
                    contours,
                )));
            }
            Classification::Decided(None) => {}
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
    }
    let emitted = selection.emit_boundary_fragments_from_owned_certified_split(fragments)?;
    let output = match output_kind {
        BooleanBoundaryOutputKind::Loops => {
            let chains = match emitted.into_assembled_chains(policy) {
                Classification::Decided(chains) => chains,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            match chains.into_closed_loops() {
                Classification::Decided(loops) => BooleanBoundaryOutput::Loops(loops),
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
        BooleanBoundaryOutputKind::Contours => {
            match emitted.into_assembled_contours(fill_rule, policy)? {
                Classification::Decided(contours) => BooleanBoundaryOutput::Contours(contours),
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
    };
    Ok(Classification::Decided(output))
}

fn boundary_contact_resolution_from_intersections(
    first: &RegionView2<'_>,
    second: &RegionView2<'_>,
    intersections: &RegionIntersectionSet,
    policy: &CurvePolicy,
) -> CurveResult<Classification<Option<BoundaryContactResolution>>> {
    if intersections.is_empty() {
        return Ok(Classification::Decided(None));
    }

    let saw_overlap = match boundary_contact_overlap_flag(intersections) {
        Classification::Decided(Some(saw_overlap)) => saw_overlap,
        Classification::Decided(None) => return Ok(Classification::Decided(None)),
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    if !saw_overlap && intersections.point_event_count() > 1 {
        return Ok(Classification::Decided(None));
    }

    let disjoint_interiors = if saw_overlap {
        split_contact_interiors_are_disjoint(first, second, intersections, policy)?
    } else {
        unsplit_contact_interiors_are_disjoint(first, second, policy)?
    };
    match disjoint_interiors {
        Classification::Decided(true) => {}
        Classification::Decided(false) => {
            return match boundary_contact_containment_relation(first, second, policy)? {
                Classification::Decided(Some(relation)) => Ok(Classification::Decided(Some(
                    BoundaryContactResolution::Containment {
                        relation,
                        contact: if saw_overlap {
                            BoundaryContactKind::Overlap
                        } else {
                            BoundaryContactKind::PointOnly
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
        BoundaryContactResolution::BoundaryOnly(if saw_overlap {
            BoundaryContactKind::Overlap
        } else {
            BoundaryContactKind::PointOnly
        }),
    )))
}

pub(crate) fn boundary_contact_overlap_flag(
    intersections: &RegionIntersectionSet,
) -> Classification<Option<bool>> {
    let mut saw_contact = false;
    let mut saw_overlap = false;
    for pair in intersections.pairs() {
        if pair
            .intersections()
            .retained_certified_line_crossings()
            .is_some()
        {
            return Classification::Decided(None);
        }
        for event in pair.intersections.events() {
            match event {
                ContourIntersection::Point(point) => match point.kind {
                    IntersectionKind::Endpoint | IntersectionKind::Tangent => {
                        saw_contact = true;
                    }
                    IntersectionKind::Crossing | IntersectionKind::Overlap => {
                        return Classification::Decided(None);
                    }
                },
                ContourIntersection::Overlap(_) => {
                    saw_contact = true;
                    saw_overlap = true;
                }
                ContourIntersection::Uncertain(uncertain) => {
                    return Classification::Uncertain(uncertain.reason);
                }
            }
        }
    }

    Classification::Decided(saw_contact.then_some(saw_overlap))
}

fn region_boundary_has_overlap_in(intersections: &RegionIntersectionSet) -> bool {
    matches!(
        boundary_contact_overlap_flag(intersections),
        Classification::Decided(Some(true))
    )
}

fn split_contact_interiors_are_disjoint(
    first: &RegionView2<'_>,
    second: &RegionView2<'_>,
    intersections: &crate::RegionIntersectionSet,
    policy: &CurvePolicy,
) -> CurveResult<Classification<bool>> {
    let fragments = match intersections.split_regions(first, second, policy)? {
        Classification::Decided(fragments) => fragments,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };

    let mut first_has_outside_sample = false;
    let mut second_has_outside_sample = false;
    let mut blocker = None;
    for contour_fragments in fragments.contours() {
        let opposite = match contour_fragments.key.side {
            RegionSide::First => second,
            RegionSide::Second => first,
        };

        for fragment in contour_fragments.fragments.fragments() {
            let sample = match fragment.segment.representative_point(policy)? {
                Classification::Decided(sample) => sample,
                Classification::Uncertain(reason) => {
                    blocker.get_or_insert(reason);
                    continue;
                }
            };
            match opposite.classify_point(&sample, policy) {
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
                Classification::Uncertain(reason) => {
                    blocker.get_or_insert(reason);
                }
            }
        }
    }

    if let Some(reason) = blocker {
        return Ok(Classification::Uncertain(reason));
    }
    Ok(Classification::Decided(
        first_has_outside_sample && second_has_outside_sample,
    ))
}

fn unsplit_contact_interiors_are_disjoint(
    first: &RegionView2<'_>,
    second: &RegionView2<'_>,
    policy: &CurvePolicy,
) -> CurveResult<Classification<bool>> {
    let mut first_has_outside_sample = false;
    let mut second_has_outside_sample = false;

    match scan_unsplit_contact_samples(
        first.material_contours(),
        second,
        &mut first_has_outside_sample,
        policy,
    )? {
        Classification::Decided(true) => {}
        Classification::Decided(false) => return Ok(Classification::Decided(false)),
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    }
    match scan_unsplit_contact_samples(
        first.hole_contours(),
        second,
        &mut first_has_outside_sample,
        policy,
    )? {
        Classification::Decided(true) => {}
        Classification::Decided(false) => return Ok(Classification::Decided(false)),
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    }
    match scan_unsplit_contact_samples(
        second.material_contours(),
        first,
        &mut second_has_outside_sample,
        policy,
    )? {
        Classification::Decided(true) => {}
        Classification::Decided(false) => return Ok(Classification::Decided(false)),
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    }
    match scan_unsplit_contact_samples(
        second.hole_contours(),
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

fn scan_unsplit_contact_samples(
    contours: &[&Contour2],
    opposite: &RegionView2<'_>,
    has_outside_sample: &mut bool,
    policy: &CurvePolicy,
) -> CurveResult<Classification<bool>> {
    let mut blocker = None;
    for contour in contours {
        for segment in contour.segments() {
            let sample = match segment.representative_point(policy)? {
                Classification::Decided(sample) => sample,
                Classification::Uncertain(reason) => {
                    blocker.get_or_insert(reason);
                    continue;
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
                Classification::Uncertain(reason) => {
                    blocker.get_or_insert(reason);
                }
            }
        }
    }

    Ok(match blocker {
        Some(reason) => Classification::Uncertain(reason),
        None => Classification::Decided(true),
    })
}

fn boundary_contact_containment_relation(
    first: &RegionView2<'_>,
    second: &RegionView2<'_>,
    policy: &CurvePolicy,
) -> CurveResult<Classification<Option<BoundaryContainmentRelation>>> {
    let first_contains_second =
        match region_contains_region_boundary_samples(first, second, policy)? {
            Classification::Decided(contains) => contains,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
    let second_contains_first =
        match region_contains_region_boundary_samples(second, first, policy)? {
            Classification::Decided(contains) => contains,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };

    Ok(Classification::Decided(
        match (first_contains_second, second_contains_first) {
            (true, true) => Some(BoundaryContainmentRelation::Equivalent),
            (true, false) => Some(BoundaryContainmentRelation::FirstContainsSecond),
            (false, true) => Some(BoundaryContainmentRelation::SecondContainsFirst),
            (false, false) => None,
        },
    ))
}

fn region_contains_region_boundary_samples(
    container: &RegionView2<'_>,
    candidate: &RegionView2<'_>,
    policy: &CurvePolicy,
) -> CurveResult<Classification<bool>> {
    boundary_contours_inside_or_on_region(
        candidate
            .material_contours()
            .iter()
            .copied()
            .chain(candidate.hole_contours().iter().copied()),
        |point| container.classify_point(point, policy),
        policy,
    )
}

pub(crate) fn boundary_contours_inside_or_on_region<'a, I, F>(
    contours: I,
    mut classify_point: F,
    policy: &CurvePolicy,
) -> CurveResult<Classification<bool>>
where
    I: IntoIterator<Item = &'a Contour2>,
    F: FnMut(&Point2) -> Classification<RegionPointLocation>,
{
    for contour in contours {
        for segment in contour.segments() {
            // Boundary-contact containment is a conservative fast path for
            // cases with no crossing events. Sampling vertices plus each
            // fragment representative keeps the decision tied to the
            // boundary-first point-in-region classification described by
            // boundary-first winding classification, rather than an epsilon-based bounding rule.
            match point_is_inside_or_boundary(segment.start(), &mut classify_point) {
                Classification::Decided(true) => {}
                Classification::Decided(false) => return Ok(Classification::Decided(false)),
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            }
            match point_is_inside_or_boundary(segment.end(), &mut classify_point) {
                Classification::Decided(true) => {}
                Classification::Decided(false) => return Ok(Classification::Decided(false)),
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            }

            let sample = match segment.representative_point(policy)? {
                Classification::Decided(sample) => sample,
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            };
            match point_is_inside_or_boundary(&sample, &mut classify_point) {
                Classification::Decided(true) => {}
                Classification::Decided(false) => return Ok(Classification::Decided(false)),
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            }
        }
    }

    Ok(Classification::Decided(true))
}

fn point_is_inside_or_boundary<F>(point: &Point2, classify_point: &mut F) -> Classification<bool>
where
    F: FnMut(&Point2) -> Classification<RegionPointLocation>,
{
    match classify_point(point) {
        Classification::Decided(RegionPointLocation::Inside | RegionPointLocation::Boundary) => {
            Classification::Decided(true)
        }
        Classification::Decided(RegionPointLocation::Outside) => Classification::Decided(false),
        Classification::Uncertain(reason) => Classification::Uncertain(reason),
    }
}

fn containment_boundary_contours(
    first: &RegionView2<'_>,
    second: &RegionView2<'_>,
    op: BooleanOp,
    relation: BoundaryContainmentRelation,
) -> Option<Vec<Contour2>> {
    // These containment identities are regularized set identities, not graph
    // traversal guesses. They cover the subset cases the degenerate-intersection clipping model
    // separate from ordinary entry/exit traversal for degenerate polygon
    // clipping. Difference is decided immediately when the left operand
    // is contained by the right. The opposite `container - touching subset`
    // case is handled by the certified overlap rebuild below, where coincident
    // zero-area edges are dropped before the remaining boundary is assembled.
    match (relation, op) {
        (BoundaryContainmentRelation::FirstContainsSecond, BooleanOp::Union) => {
            Some(clone_boundary_contours(first))
        }
        (BoundaryContainmentRelation::FirstContainsSecond, BooleanOp::Intersection) => {
            Some(clone_boundary_contours(second))
        }
        (BoundaryContainmentRelation::SecondContainsFirst, BooleanOp::Union) => {
            Some(clone_boundary_contours(second))
        }
        (BoundaryContainmentRelation::SecondContainsFirst, BooleanOp::Intersection) => {
            Some(clone_boundary_contours(first))
        }
        (BoundaryContainmentRelation::SecondContainsFirst, BooleanOp::Difference) => {
            Some(Vec::new())
        }
        (BoundaryContainmentRelation::Equivalent, BooleanOp::Union | BooleanOp::Intersection) => {
            Some(clone_boundary_contours(first))
        }
        (BoundaryContainmentRelation::Equivalent, BooleanOp::Difference | BooleanOp::Xor) => {
            Some(Vec::new())
        }
        _ => None,
    }
}

fn containment_difference_boundary_contours(
    first: &RegionView2<'_>,
    second: &RegionView2<'_>,
    fill_rule: FillRule,
    policy: &CurvePolicy,
) -> CurveResult<Classification<Vec<Contour2>>> {
    let intersections = first.intersect_region(second, policy)?;
    let fragments = match intersections.split_regions(first, second, policy)? {
        Classification::Decided(fragments) => fragments,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let selection =
        match fragments.classify_for_boolean(first, second, BooleanOp::Difference, policy)? {
            Classification::Decided(selection) => selection,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };

    boundary_contours_resolving_shared_boundaries(
        &fragments,
        &selection,
        BooleanOp::Difference,
        fill_rule,
        policy,
    )
}

fn boundary_contact_boundary_contours(
    first: &RegionView2<'_>,
    second: &RegionView2<'_>,
    op: BooleanOp,
    fill_rule: FillRule,
    policy: &CurvePolicy,
    kind: BoundaryContactKind,
) -> CurveResult<Classification<Vec<Contour2>>> {
    // Boundary-only contacts carry no filled area. The degenerate-intersection
    // clipping model treats them separately from ordinary traversal.
    // Point-only contacts keep their
    // separate loops; shared-edge contacts must remove the coincident edge for
    // union/xor so the result does not expose an internal seam as boundary.
    Ok(Classification::Decided(match op {
        BooleanOp::Union | BooleanOp::Xor => match kind {
            BoundaryContactKind::PointOnly => {
                let mut contours = clone_boundary_contours(first);
                contours.extend(clone_boundary_contours(second));
                contours
            }
            BoundaryContactKind::Overlap => {
                return boundary_overlap_union_contours(first, second, op, fill_rule, policy);
            }
        },
        BooleanOp::Intersection => Vec::new(),
        BooleanOp::Difference => clone_boundary_contours(first),
    }))
}

fn boundary_overlap_union_contours(
    first: &RegionView2<'_>,
    second: &RegionView2<'_>,
    op: BooleanOp,
    fill_rule: FillRule,
    policy: &CurvePolicy,
) -> CurveResult<Classification<Vec<Contour2>>> {
    let intersections = first.intersect_region(second, policy)?;
    let fragments = match intersections.split_regions(first, second, policy)? {
        Classification::Decided(fragments) => fragments,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let selection = match fragments.classify_for_boolean(first, second, op, policy)? {
        Classification::Decided(selection) => selection,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };

    boundary_contours_resolving_shared_boundaries(&fragments, &selection, op, fill_rule, policy)
}

pub(crate) fn boundary_contours_resolving_shared_boundaries(
    fragments: &RegionFragmentSet,
    selection: &BooleanFragmentSelection,
    op: BooleanOp,
    fill_rule: FillRule,
    policy: &CurvePolicy,
) -> CurveResult<Classification<Vec<Contour2>>> {
    let selection = match resolve_shared_boundary_selection(fragments, selection, op)? {
        Classification::Decided(selection) => selection,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };

    let emitted = selection.emit_boundary_fragments(fragments)?;

    // Exact local fill-side ownership resolves every coincident pair before
    // traversal, following regularized fill-state clipping.
    let chains = match emitted.assemble_chains(policy) {
        Classification::Decided(chains) => chains,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };

    match chains.into_closed_loops() {
        Classification::Decided(loops) => {
            loops.into_contours(fill_rule).map(Classification::Decided)
        }
        Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
    }
}

pub(crate) fn resolve_shared_boundary_selection(
    fragments: &RegionFragmentSet,
    selection: &BooleanFragmentSelection,
    op: BooleanOp,
) -> CurveResult<Classification<BooleanFragmentSelection>> {
    let pairs = match unresolved_boundary_pairs(fragments, selection)? {
        Classification::Decided(pairs) => pairs,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let mut resolutions = Vec::with_capacity(pairs.len() * 2);
    for (left_index, right_index) in pairs {
        let left = &selection.classifications()[left_index];
        let right = &selection.classifications()[right_index];
        let (first_classification, second_classification) = if left.key.side == RegionSide::First {
            (left, right)
        } else {
            (right, left)
        };
        if first_classification.key.side != RegionSide::First
            || second_classification.key.side != RegionSide::Second
        {
            return Err(CurveError::Topology(
                "boolean shared-boundary pair does not span both operands".into(),
            ));
        }
        let first_segment = fragment_segment_for_classification(fragments, first_classification)?;
        let second_segment = fragment_segment_for_classification(fragments, second_classification)?;
        let first_left = first_classification.source_filled_side_is_left;
        let second_left = second_classification.source_filled_side_is_left;
        let same_direction = segment_images_match_directed(first_segment, second_segment);
        let normalized_second_left = if same_direction {
            second_left
        } else {
            !second_left
        };
        let action = match (
            op.apply(first_left, normalized_second_left),
            op.apply(!first_left, !normalized_second_left),
        ) {
            (true, false) => BooleanFragmentAction::KeepSourceDirection,
            (false, true) => BooleanFragmentAction::KeepReversed,
            (false, false) | (true, true) => BooleanFragmentAction::Discard,
        };
        resolutions.push((
            first_classification.key,
            first_classification.fragment_index,
            action,
        ));
        resolutions.push((
            second_classification.key,
            second_classification.fragment_index,
            BooleanFragmentAction::Discard,
        ));
    }
    selection
        .resolve_boundary_actions(&resolutions)
        .map(Classification::Decided)
}

fn resolve_owned_shared_boundary_selection(
    fragments: &RegionFragmentSet,
    selection: BooleanFragmentSelection,
    op: BooleanOp,
) -> CurveResult<Classification<BooleanFragmentSelection>> {
    if selection.count_action(BooleanFragmentAction::BoundaryNeedsResolution) == 0 {
        return Ok(Classification::Decided(selection));
    }
    resolve_shared_boundary_selection(fragments, &selection, op)
}

fn unresolved_boundary_pairs(
    fragments: &RegionFragmentSet,
    selection: &BooleanFragmentSelection,
) -> CurveResult<Classification<Vec<(usize, usize)>>> {
    let unresolved = selection
        .classifications()
        .iter()
        .enumerate()
        .filter(|classification| {
            classification.1.action == BooleanFragmentAction::BoundaryNeedsResolution
        })
        .collect::<Vec<_>>();

    if unresolved.is_empty() {
        return Ok(Classification::Decided(Vec::new()));
    }
    if unresolved.len() % 2 != 0 {
        return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
    }

    let mut paired = vec![false; unresolved.len()];
    let mut pairs = Vec::with_capacity(unresolved.len() / 2);
    for left_index in 0..unresolved.len() {
        if paired[left_index] {
            continue;
        }
        let left_segment =
            fragment_segment_for_classification(fragments, unresolved[left_index].1)?;
        let mut matched = false;
        for right_index in left_index + 1..unresolved.len() {
            if paired[right_index] {
                continue;
            }
            if unresolved[left_index].1.key.side == unresolved[right_index].1.key.side {
                continue;
            }
            let right_segment =
                fragment_segment_for_classification(fragments, unresolved[right_index].1)?;
            if segment_images_match_undirected(left_segment, right_segment) {
                paired[left_index] = true;
                paired[right_index] = true;
                pairs.push((unresolved[left_index].0, unresolved[right_index].0));
                matched = true;
                break;
            }
        }
        if !matched {
            return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
        }
    }

    Ok(Classification::Decided(pairs))
}

fn fragment_segment_for_classification<'a>(
    fragments: &'a RegionFragmentSet,
    classification: &BooleanFragmentClassification,
) -> CurveResult<&'a Segment2> {
    let contour_fragments = fragments
        .fragments_for_contour(classification.key)
        .ok_or_else(|| {
            CurveError::Topology("boolean unresolved boundary references a missing contour".into())
        })?;
    contour_fragments
        .fragments
        .fragments()
        .get(classification.fragment_index)
        .map(|fragment| &fragment.segment)
        .ok_or_else(|| {
            CurveError::Topology("boolean unresolved boundary references a missing fragment".into())
        })
}

fn segment_images_match_undirected(left: &Segment2, right: &Segment2) -> bool {
    segment_images_match_directed(left, right)
        || segment_images_match_directed(left, &right.reversed())
}

fn segment_images_match_directed(left: &Segment2, right: &Segment2) -> bool {
    match (left, right) {
        // Line equality deliberately retains construction support. Boolean
        // overlap resolution instead needs equality of the finite geometric
        // image: independently split or offset-derived fragments can have the
        // same endpoints while carrying different provenance.
        (Segment2::Line(left), Segment2::Line(right)) => {
            left.start() == right.start() && left.end() == right.end()
        }
        (Segment2::Arc(left), Segment2::Arc(right)) => left == right,
        _ => false,
    }
}

fn xor_region_by_difference_union(
    first: &RegionView2<'_>,
    second: &RegionView2<'_>,
    fill_rule: FillRule,
    policy: &CurvePolicy,
) -> CurveResult<Classification<LineArcRegion2>> {
    // Region XOR is the symmetric difference `(A - B) union (B - A)`. Using
    // the set identity lets the region-level API reuse the better-tested difference and
    // union role-assignment paths while the lower boundary graph still grows a
    // dedicated overlap/branch resolver for direct XOR traversal.
    let first_only =
        match boolean_region_between(first, second, BooleanOp::Difference, fill_rule, policy)? {
            Classification::Decided(region) => region,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
    let second_only =
        match boolean_region_between(second, first, BooleanOp::Difference, fill_rule, policy)? {
            Classification::Decided(region) => region,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };

    Ok(Classification::Decided(merge_disjoint_region_bins(
        first_only,
        second_only,
    )))
}

pub(crate) fn merge_disjoint_region_bins(
    first: LineArcRegion2,
    second: LineArcRegion2,
) -> LineArcRegion2 {
    // The two symmetric-difference halves are interior-disjoint by set
    // definition. Directly merging their signed contour bins preserves
    // boundary-only contacts that a contour-only nesting pass would reject as
    // ambiguous. After both difference regions have crossed the fill-state
    // boundary, their explicit material/hole bins can be concatenated without
    // inventing a new traversal graph.
    let mut material_contours = first.material_contours().to_vec();
    material_contours.extend(second.material_contours().iter().cloned());
    let mut hole_contours = first.hole_contours().to_vec();
    hole_contours.extend(second.hole_contours().iter().cloned());
    LineArcRegion2::new(material_contours, hole_contours)
}

pub(crate) fn same_region_view(first: &RegionView2<'_>, second: &RegionView2<'_>) -> bool {
    same_contour_multiset(first.material_contours(), second.material_contours())
        && same_contour_multiset(first.hole_contours(), second.hole_contours())
}

fn same_contour_multiset(first: &[&Contour2], second: &[&Contour2]) -> bool {
    if first.len() != second.len() {
        return false;
    }

    let mut matched = vec![false; second.len()];
    for first_contour in first {
        let Some(index) = second
            .iter()
            .enumerate()
            .find_map(|(index, second_contour)| {
                (!matched[index]
                    && contours_have_same_exact_boundary(first_contour, second_contour))
                .then_some(index)
            })
        else {
            return false;
        };
        matched[index] = true;
    }

    true
}

fn contours_have_same_exact_boundary(first: &Contour2, second: &Contour2) -> bool {
    if std::ptr::eq(first, second) {
        return true;
    }
    if let (Some(first), Some(second)) = (first.cached_signed_area(), second.cached_signed_area())
        && let (Some(first), Some(second)) =
            (first.exact_rational_ref(), second.exact_rational_ref())
        && (first.numerator() != second.numerator() || first.denominator() != second.denominator())
    {
        return false;
    }
    first.has_same_exact_boundary(second)
}

pub(crate) fn clone_boundary_contours(region: &RegionView2<'_>) -> Vec<Contour2> {
    // Exact contour-bin identity fast paths keep coincident boundaries out of
    // the general traversal graph. Degenerate polygon clipping benefits from
    // separating coincident boundaries from ordinary entry/exit traversal.
    // This fast path handles exact
    // reordered contours, cyclic start-index changes, and reversed traversal
    // within each role bin; split or otherwise equivalent-but-nonidentical
    // boundaries still belong to the future overlap resolver.
    region
        .material_contours()
        .iter()
        .chain(region.hole_contours().iter())
        .map(|contour| (*contour).clone())
        .collect()
}

pub(crate) fn empty_operand_boundary_contours(
    first: &RegionView2<'_>,
    second: &RegionView2<'_>,
    op: BooleanOp,
) -> Vec<Contour2> {
    // Empty-set identities are regularized boolean identities, so they should
    // not enter the clipping graph at all. With one empty operand, fill-state
    // transitions reduce to the nonempty operand or to the empty set.
    match (first.is_empty(), second.is_empty(), op) {
        (true, _, BooleanOp::Union | BooleanOp::Xor) => clone_boundary_contours(second),
        (_, true, BooleanOp::Union | BooleanOp::Xor | BooleanOp::Difference) => {
            clone_boundary_contours(first)
        }
        _ => Vec::new(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        BooleanFragmentAction, BooleanFragmentClassification, ContourFragment, ContourFragmentSet,
        LineSeg2, ParamRange, RegionContourFragments, RegionContourKey, RegionContourRole,
    };

    fn real(value: i32) -> Real {
        value.into()
    }

    fn point(x: i32, y: i32) -> Point2 {
        Point2::new(real(x), real(y))
    }

    fn line_segment(x0: i32, y0: i32, x1: i32, y1: i32) -> Segment2 {
        Segment2::Line(LineSeg2::try_new(point(x0, y0), point(x1, y1)).unwrap())
    }

    fn rectangle(width: i32, height: i32) -> Contour2 {
        rectangle_at(0, 0, width, height)
    }

    fn rectangle_at(min_x: i32, min_y: i32, max_x: i32, max_y: i32) -> Contour2 {
        Contour2::from_bulge_vertices(&[
            BulgeVertex2::new(point(min_x, min_y), Real::zero()),
            BulgeVertex2::new(point(max_x, min_y), Real::zero()),
            BulgeVertex2::new(point(max_x, max_y), Real::zero()),
            BulgeVertex2::new(point(min_x, max_y), Real::zero()),
        ])
        .unwrap()
    }

    fn decided_region(result: CurveResult<Classification<LineArcRegion2>>) -> LineArcRegion2 {
        match result.unwrap() {
            Classification::Decided(region) => region,
            Classification::Uncertain(reason) => {
                panic!("expected decided region Boolean, got {reason:?}")
            }
        }
    }

    fn fragment_set_for(key: RegionContourKey, segment: Segment2) -> RegionContourFragments {
        let source_segment_start_point = segment.start().clone();
        let source_segment_end_point = segment.end().clone();
        RegionContourFragments {
            key,
            fragments: ContourFragmentSet::new(vec![ContourFragment {
                source_segment_index: 0,
                source_segment_start_point,
                source_segment_end_point,
                source_range: ParamRange::new(real(0), real(1)),
                segment,
            }])
            .unwrap(),
        }
    }

    fn unresolved_boundary(key: RegionContourKey) -> BooleanFragmentClassification {
        BooleanFragmentClassification {
            key,
            fragment_index: 0,
            opposite_location: RegionPointLocation::Boundary,
            source_filled_side_is_left: true,
            action: BooleanFragmentAction::BoundaryNeedsResolution,
        }
    }

    #[test]
    fn cached_area_magnitude_rejects_only_distinct_exact_boundaries() {
        let first = rectangle(4, 3);
        let reversed = Contour2::from_validated_closed_segments(
            first
                .segments()
                .iter()
                .rev()
                .map(Segment2::reversed)
                .collect(),
            first.fill_rule(),
        );
        let different = rectangle(5, 3);
        for contour in [&first, &reversed, &different] {
            assert!(contour.signed_area().unwrap().is_some());
        }

        assert!(contours_have_same_exact_boundary(&first, &first));
        assert!(contours_have_same_exact_boundary(&first, &reversed));
        assert!(!contours_have_same_exact_boundary(&first, &different));
    }

    #[test]
    fn coincident_hole_component_uses_explicit_region_roles() {
        let outer = rectangle_at(0, 0, 4, 4);
        let hole = rectangle_at(1, 1, 3, 3);
        let ring = LineArcRegion2::new(vec![outer], vec![hole.clone()]);
        let plug = LineArcRegion2::from_material_contours(vec![hole]);
        let policy = CurvePolicy::STRICT;

        let intersection = decided_region(ring.boolean_region(
            &plug,
            BooleanOp::Intersection,
            FillRule::NonZero,
            &policy,
        ));
        assert!(intersection.is_empty());

        let union = decided_region(ring.boolean_region(
            &plug,
            BooleanOp::Union,
            FillRule::NonZero,
            &policy,
        ));
        assert_eq!(union.material_contours().len(), 1);
        assert!(union.hole_contours().is_empty());
        assert_eq!(
            union.classify_point(&point(2, 2), &policy),
            Classification::Decided(RegionPointLocation::Inside)
        );

        let ring_minus_plug = decided_region(ring.boolean_region(
            &plug,
            BooleanOp::Difference,
            FillRule::NonZero,
            &policy,
        ));
        assert_eq!(ring_minus_plug.material_contours().len(), 1);
        assert_eq!(ring_minus_plug.hole_contours().len(), 1);

        let plug_minus_ring = decided_region(plug.boolean_region(
            &ring,
            BooleanOp::Difference,
            FillRule::NonZero,
            &policy,
        ));
        assert_eq!(plug_minus_ring.material_contours().len(), 1);
        assert!(plug_minus_ring.hole_contours().is_empty());
    }

    #[test]
    fn unresolved_boundaries_require_opposite_fragment_pair_evidence() {
        let first_key = RegionContourKey::new(RegionSide::First, RegionContourRole::Material, 0);
        let fragments =
            RegionFragmentSet::new(vec![fragment_set_for(first_key, line_segment(0, 0, 1, 0))])
                .unwrap();
        let selection =
            BooleanFragmentSelection::new(vec![unresolved_boundary(first_key)]).unwrap();

        let result = unresolved_boundary_pairs(&fragments, &selection).unwrap();

        assert_eq!(
            result,
            Classification::Uncertain(UncertaintyReason::Boundary)
        );
    }

    #[test]
    fn unresolved_boundaries_retain_certified_opposite_fragment_pairs() {
        let first_key = RegionContourKey::new(RegionSide::First, RegionContourRole::Material, 0);
        let second_key = RegionContourKey::new(RegionSide::Second, RegionContourRole::Material, 0);
        let fragments = RegionFragmentSet::new(vec![
            fragment_set_for(first_key, line_segment(0, 0, 1, 0)),
            fragment_set_for(second_key, line_segment(1, 0, 0, 0)),
        ])
        .unwrap();
        let selection = BooleanFragmentSelection::new(vec![
            unresolved_boundary(first_key),
            unresolved_boundary(second_key),
        ])
        .unwrap();

        let result = unresolved_boundary_pairs(&fragments, &selection).unwrap();

        assert_eq!(result, Classification::Decided(vec![(0, 1)]));
    }

    #[test]
    fn strip_boolean_evidence_degenerate_required_rectangle_as_uncertainty() {
        assert_eq!(
            strip_boolean_region(
                real(0),
                real(1),
                real(0),
                real(1),
                real(0),
                real(0),
                true,
                BooleanOp::Union,
                &CurvePolicy::STRICT,
            ),
            Classification::Uncertain(UncertaintyReason::Unsupported)
        );
    }
}
