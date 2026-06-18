//! Region-level boolean boundary pipeline.
//!
//! The routines here compose the existing event, split, classify, and boundary
//! traversal stages. Simple boundary-only contacts are regularized here, while
//! shared-boundary cases that also involve interior containment remain explicit
//! uncertainty instead of being guessed through.

use crate::classify::compare_reals;
use crate::{
    Aabb2, BooleanBoundaryFragmentSet, BooleanBoundaryLoopSet, BooleanFragmentAction,
    BooleanFragmentClassification, BooleanFragmentSelection, BooleanOp, BulgeVertex2,
    Classification, Contour2, ContourIntersection, CurveError, CurvePolicy, CurveResult, FillRule,
    IntersectionKind, Point2, Real, Region2, RegionBoundaryContourBuildReport2, RegionFragmentSet,
    RegionIntersectionSet, RegionPointLocation, RegionSide, RegionView2, RetainedTopologyStatus,
    Segment2, UncertaintyReason,
};
use std::cmp::Ordering;

/// Report for a closed region boolean materialization.
#[derive(Clone, Debug, PartialEq)]
pub struct RegionBooleanReport2 {
    op: BooleanOp,
    fill_rule: FillRule,
    query_path: RegionBooleanQueryPath2,
    stage: RegionBooleanStage2,
    first_material_contour_count: usize,
    first_hole_contour_count: usize,
    first_boundary_segment_count: usize,
    second_material_contour_count: usize,
    second_hole_contour_count: usize,
    second_boundary_segment_count: usize,
    boundary_first_contour_count: Option<usize>,
    boundary_second_contour_count: Option<usize>,
    boundary_candidate_pair_count: usize,
    boundary_skipped_aabb_pair_count: usize,
    boundary_tested_pair_count: usize,
    boundary_intersecting_pair_count: usize,
    boundary_contour_count: Option<usize>,
    result_material_contour_count: Option<usize>,
    result_hole_contour_count: Option<usize>,
    boundary_build_report: Option<RegionBoundaryContourBuildReport2>,
    prepared_cache_report: Option<RegionBooleanPreparedCacheReport2>,
    status: RetainedTopologyStatus,
    blocker: Option<UncertaintyReason>,
}

/// Query path used by a report-bearing closed region boolean.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RegionBooleanQueryPath2 {
    /// Boolean materialization used transient region views and direct queries.
    Direct,
    /// Boolean materialization used caller-supplied prepared region views.
    Prepared,
}

/// Furthest exact materialization stage reached by a region boolean report.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RegionBooleanStage2 {
    /// The attempt stopped while extracting checked boolean boundary contours.
    BoundaryExtraction,
    /// Checked boundary contours were built and role-assigned into a region.
    RegionRoleAssignment,
}

/// Prepared-cache evidence consumed by a report-bearing region boolean.
#[derive(Clone, Debug, PartialEq)]
pub struct RegionBooleanPreparedCacheReport2 {
    first: RegionPreparedCacheAudit2,
    second: RegionPreparedCacheAudit2,
}

/// Per-operand prepared region cache inventory.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct RegionPreparedCacheAudit2 {
    freshness: RegionPreparedCacheFreshness2,
    prepared_contour_count: usize,
    prepared_segment_count: usize,
    decided_segment_box_count: usize,
    undecided_segment_box_count: usize,
    region_box_decided: bool,
}

/// Freshness claim for prepared boolean cache evidence.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RegionPreparedCacheFreshness2 {
    /// Prepared cache borrows the current source contours for this query.
    BorrowedCurrentSource,
}

/// Result of report-bearing closed region boolean materialization.
#[derive(Clone, Debug, PartialEq)]
pub struct RegionBooleanResult2 {
    region: Option<Region2>,
    report: RegionBooleanReport2,
}

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
// Foster, Hormann, and Popa, "Clipping simple polygons with degenerate
// intersections," Computers & Graphics: X 2, 100007, 2019,
// https://doi.org/10.1016/j.cagx.2019.100007, and it must be resolved in the
// geometry kernel so CAD callers receive ordinary Region2 values rather than
// crate-local workarounds.
pub(crate) fn coextensive_axis_rect_region_boolean(
    first: &RegionView2<'_>,
    second: &RegionView2<'_>,
    op: BooleanOp,
    policy: &CurvePolicy,
) -> CurveResult<Classification<Option<Region2>>> {
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
) -> Classification<Region2> {
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
            // edge for union and symmetric difference; see Foster, Hormann,
            // and Popa, "Clipping simple polygons with degenerate
            // intersections" (2019). Keeping this in the rectangle fast path
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
            BooleanOp::Intersection => Classification::Decided(Region2::empty()),
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
    Classification::Decided(Region2::from_material_contours(contours))
}

type StripRectBounds = (Real, Real, Real, Real, bool);

fn required_strip_region(bounds: Vec<StripRectBounds>) -> Classification<Region2> {
    match required_strip_rects(bounds) {
        Classification::Decided(contours) => {
            Classification::Decided(Region2::from_material_contours(contours))
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

impl Region2 {
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

    /// Computes a role-assigned boolean region and retains materialization evidence.
    pub fn boolean_region_with_report(
        &self,
        other: &Self,
        op: BooleanOp,
        fill_rule: FillRule,
        policy: &CurvePolicy,
    ) -> CurveResult<RegionBooleanResult2> {
        self.as_view()
            .boolean_region_with_report(&other.as_view(), op, fill_rule, policy)
    }
}

impl RegionView2<'_> {
    /// Computes closed boolean boundary loops against another region view.
    ///
    /// Algorithm note: this method wires together the standard polygon clipping
    /// stages: collect intersection events, split input boundaries at those
    /// events, classify each fragment against the opposite operand, and traverse
    /// selected directed fragments into closed loops. Greiner and Hormann
    /// describe split-boundary traversal after entry/exit classification
    /// (G. Greiner and K. Hormann, "Efficient clipping of arbitrary polygons,"
    /// ACM Transactions on Graphics 17(2), 71-83, 1998). Martinez, Rueda, and
    /// Feito describe boolean selection from segment classifications for
    /// general polygons (F. Martinez, A. J. Rueda, and F. R. Feito, "A new
    /// algorithm for computing Boolean operations on polygons," Computers &
    /// Geosciences 35(6), 1177-1185, 2009). `hypercurve` keeps each stage
    /// explicit so uncertain tangencies, shared boundaries, and branch vertices
    /// can stop the pipeline instead of being resolved by a global epsilon.
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
    /// and hole bins by containment depth. Hormann and Agathos discuss the
    /// point-in-polygon classification problem that underlies this nesting test
    /// (K. Hormann and A. Agathos, "The point in polygon problem for arbitrary
    /// polygons," Computational Geometry 20(3), 131-144, 2001). `hypercurve`
    /// treats any boundary result during nesting as explicit uncertainty,
    /// because a boundary touch means the output contour graph still needs a
    /// degeneracy-specific resolver.
    pub fn boolean_region(
        &self,
        other: &RegionView2<'_>,
        op: BooleanOp,
        fill_rule: FillRule,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<Region2>> {
        boolean_region_between(self, other, op, fill_rule, policy)
    }

    /// Computes a role-assigned boolean region and retains materialization evidence.
    ///
    /// The result uses the checked boundary-contour boolean path and the
    /// report-bearing contour nesting pass, so callers can audit output contour
    /// counts, final material/hole role assignment, and blockers.
    pub fn boolean_region_with_report(
        &self,
        other: &RegionView2<'_>,
        op: BooleanOp,
        fill_rule: FillRule,
        policy: &CurvePolicy,
    ) -> CurveResult<RegionBooleanResult2> {
        boolean_region_between_with_report(self, other, op, fill_rule, policy)
    }
}

impl RegionBooleanReport2 {
    /// Returns the requested boolean operation.
    pub const fn op(&self) -> BooleanOp {
        self.op
    }

    /// Returns the fill rule used to materialize boolean boundary contours.
    pub const fn fill_rule(&self) -> FillRule {
        self.fill_rule
    }

    /// Returns whether this report used direct or prepared region queries.
    pub const fn query_path(&self) -> RegionBooleanQueryPath2 {
        self.query_path
    }

    /// Returns the furthest exact materialization stage reached.
    pub const fn stage(&self) -> RegionBooleanStage2 {
        self.stage
    }

    /// Returns the first operand material contour count.
    pub const fn first_material_contour_count(&self) -> usize {
        self.first_material_contour_count
    }

    /// Returns the first operand hole contour count.
    pub const fn first_hole_contour_count(&self) -> usize {
        self.first_hole_contour_count
    }

    /// Returns total boundary segment count in the first operand.
    pub const fn first_boundary_segment_count(&self) -> usize {
        self.first_boundary_segment_count
    }

    /// Returns the second operand material contour count.
    pub const fn second_material_contour_count(&self) -> usize {
        self.second_material_contour_count
    }

    /// Returns the second operand hole contour count.
    pub const fn second_hole_contour_count(&self) -> usize {
        self.second_hole_contour_count
    }

    /// Returns total boundary segment count in the second operand.
    pub const fn second_boundary_segment_count(&self) -> usize {
        self.second_boundary_segment_count
    }

    /// Returns the first operand contour count reported by boundary events.
    pub const fn boundary_first_contour_count(&self) -> Option<usize> {
        self.boundary_first_contour_count
    }

    /// Returns the second operand contour count reported by boundary events.
    pub const fn boundary_second_contour_count(&self) -> Option<usize> {
        self.boundary_second_contour_count
    }

    /// Returns region contour-pair candidates considered before boolean splitting.
    pub const fn boundary_candidate_pair_count(&self) -> usize {
        self.boundary_candidate_pair_count
    }

    /// Returns contour-pair candidates skipped by decided disjoint boundary AABBs.
    pub const fn boundary_skipped_aabb_pair_count(&self) -> usize {
        self.boundary_skipped_aabb_pair_count
    }

    /// Returns contour-pair candidates that reached exact boundary intersection.
    pub const fn boundary_tested_pair_count(&self) -> usize {
        self.boundary_tested_pair_count
    }

    /// Returns contour pairs with nonempty retained boundary-intersection evidence.
    pub const fn boundary_intersecting_pair_count(&self) -> usize {
        self.boundary_intersecting_pair_count
    }

    /// Returns checked output boundary contour count when available.
    pub const fn boundary_contour_count(&self) -> Option<usize> {
        self.boundary_contour_count
    }

    /// Returns material contour count when a boolean region materialized.
    pub const fn result_material_contour_count(&self) -> Option<usize> {
        self.result_material_contour_count
    }

    /// Returns hole contour count when a boolean region materialized.
    pub const fn result_hole_contour_count(&self) -> Option<usize> {
        self.result_hole_contour_count
    }

    /// Returns final boundary-contour role assignment evidence, if available.
    pub const fn boundary_build_report(&self) -> Option<&RegionBoundaryContourBuildReport2> {
        self.boundary_build_report.as_ref()
    }

    /// Returns prepared-cache inventory and freshness evidence, when used.
    pub const fn prepared_cache_report(&self) -> Option<&RegionBooleanPreparedCacheReport2> {
        self.prepared_cache_report.as_ref()
    }

    /// Returns boolean-region materialization status.
    pub const fn status(&self) -> RetainedTopologyStatus {
        self.status
    }

    /// Returns the exact blocker for non-materialized boolean attempts.
    pub const fn blocker(&self) -> Option<UncertaintyReason> {
        self.blocker
    }
}

impl RegionBooleanPreparedCacheReport2 {
    /// Builds a prepared-cache evidence report from per-operand audits.
    pub(crate) const fn new(
        first: RegionPreparedCacheAudit2,
        second: RegionPreparedCacheAudit2,
    ) -> Self {
        Self { first, second }
    }

    /// Returns prepared-cache evidence for the first operand.
    pub const fn first(&self) -> &RegionPreparedCacheAudit2 {
        &self.first
    }

    /// Returns prepared-cache evidence for the second operand.
    pub const fn second(&self) -> &RegionPreparedCacheAudit2 {
        &self.second
    }
}

impl RegionPreparedCacheAudit2 {
    /// Builds per-region prepared cache evidence.
    pub(crate) const fn new(
        prepared_contour_count: usize,
        prepared_segment_count: usize,
        decided_segment_box_count: usize,
        undecided_segment_box_count: usize,
        region_box_decided: bool,
    ) -> Self {
        Self {
            freshness: RegionPreparedCacheFreshness2::BorrowedCurrentSource,
            prepared_contour_count,
            prepared_segment_count,
            decided_segment_box_count,
            undecided_segment_box_count,
            region_box_decided,
        }
    }

    /// Returns the cache freshness claim for this borrowed prepared view.
    pub const fn freshness(&self) -> RegionPreparedCacheFreshness2 {
        self.freshness
    }

    /// Returns the number of prepared material and hole contours.
    pub const fn prepared_contour_count(&self) -> usize {
        self.prepared_contour_count
    }

    /// Returns the number of prepared boundary segments.
    pub const fn prepared_segment_count(&self) -> usize {
        self.prepared_segment_count
    }

    /// Returns the number of decided segment AABBs retained by preparation.
    pub const fn decided_segment_box_count(&self) -> usize {
        self.decided_segment_box_count
    }

    /// Returns the number of source segment AABBs that remained undecided.
    pub const fn undecided_segment_box_count(&self) -> usize {
        self.undecided_segment_box_count
    }

    /// Returns whether preparation retained a decided whole-region AABB.
    pub const fn region_box_decided(&self) -> bool {
        self.region_box_decided
    }
}

impl RegionBooleanResult2 {
    /// Returns the materialized boolean region, if construction succeeded.
    pub const fn region(&self) -> Option<&Region2> {
        self.region.as_ref()
    }

    /// Consumes this result and returns the materialized boolean region.
    pub fn into_region(self) -> Option<Region2> {
        self.region
    }

    /// Returns retained boolean materialization evidence.
    pub const fn report(&self) -> &RegionBooleanReport2 {
        &self.report
    }
}

pub(crate) fn boolean_boundary_loops_between(
    first: &RegionView2<'_>,
    second: &RegionView2<'_>,
    op: BooleanOp,
    policy: &CurvePolicy,
) -> CurveResult<Classification<BooleanBoundaryLoopSet>> {
    if same_region_view(first, second) {
        return Ok(Classification::Decided(
            BooleanBoundaryLoopSet::from_contours(match op {
                BooleanOp::Union | BooleanOp::Intersection => clone_boundary_contours(first),
                BooleanOp::Difference | BooleanOp::Xor => Vec::new(),
            })?,
        ));
    }
    if first.is_empty() || second.is_empty() {
        return Ok(Classification::Decided(
            BooleanBoundaryLoopSet::from_contours(empty_operand_boundary_contours(
                first, second, op,
            ))?,
        ));
    }
    match coextensive_axis_rect_region_boolean(first, second, op, policy)? {
        Classification::Decided(Some(region)) => {
            return Ok(Classification::Decided(
                BooleanBoundaryLoopSet::from_contours(clone_boundary_contours(&region.as_view()))?,
            ));
        }
        Classification::Decided(None) => {}
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    }
    match boundary_contact_resolution(first, second, policy)? {
        // Shared-boundary topology is a known degenerate branch. Following the
        // contour-level policy in `boundary_contact_boundary_contours` keeps this
        // stage consistent with the explicit regularization used for degenerate
        // contacts in `BooleanBoundaryLoopSet` construction, which remains a
        // structural transfer after resolved contacts are decided.
        Classification::Decided(Some(BoundaryContactResolution::BoundaryOnly(kind))) => {
            return boundary_contact_boundary_contours(
                first,
                second,
                op,
                FillRule::NonZero,
                policy,
                kind,
            )
            .and_then(BooleanBoundaryLoopSet::from_contour_classification);
        }
        Classification::Decided(Some(BoundaryContactResolution::Containment {
            relation,
            contact,
        })) => {
            // This follows Martinez et al.'s selection decomposition for
            // containments and then converts the contour-level closed-result
            // set directly to role-less loops.
            // F. Martinez, A. J. Rueda, and F. R. Feito, "A new algorithm
            // for computing Boolean operations on polygons," Computers &
            // Geosciences 35(6), 1177-1185, 2009.
            if let Some(contours) = containment_boundary_contours(first, second, op, relation) {
                return Ok(Classification::Decided(
                    BooleanBoundaryLoopSet::from_contours(contours)?,
                ));
            }
            if relation == BoundaryContainmentRelation::FirstContainsSecond
                && contact == BoundaryContactKind::Overlap
                && op == BooleanOp::Difference
            {
                return containment_difference_boundary_contours(
                    first,
                    second,
                    FillRule::NonZero,
                    policy,
                )
                .and_then(BooleanBoundaryLoopSet::from_contour_classification);
            }
        }
        Classification::Decided(None) => {
            // Union overlap on a boundary-only contact retains this dedicated
            // fast path both for region correctness and to prevent shared
            // boundary leakage when no interior overlap is detected.
            if op == BooleanOp::Union && region_boundary_has_overlap(first, second, policy)? {
                return boundary_overlap_union_contours(
                    first,
                    second,
                    op,
                    FillRule::NonZero,
                    policy,
                )
                .and_then(BooleanBoundaryLoopSet::from_contour_classification);
            }
        }
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    }

    if op == BooleanOp::Xor {
        return xor_boundary_contours_by_region(first, second, FillRule::NonZero, policy)
            .and_then(BooleanBoundaryLoopSet::from_contour_classification);
    }

    let intersections = first.intersect_region(second, policy)?;

    let fragments = match intersections.split_regions(first, second, policy)? {
        Classification::Decided(fragments) => fragments,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };

    let selection = match fragments.classify_for_boolean(first, second, op, policy)? {
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

pub(crate) fn boolean_boundary_contours_between(
    first: &RegionView2<'_>,
    second: &RegionView2<'_>,
    op: BooleanOp,
    fill_rule: FillRule,
    policy: &CurvePolicy,
) -> CurveResult<Classification<Vec<Contour2>>> {
    if same_region_view(first, second) {
        return Ok(Classification::Decided(match op {
            BooleanOp::Union | BooleanOp::Intersection => clone_boundary_contours(first),
            BooleanOp::Difference | BooleanOp::Xor => Vec::new(),
        }));
    }
    if first.is_empty() || second.is_empty() {
        return Ok(Classification::Decided(empty_operand_boundary_contours(
            first, second, op,
        )));
    }
    match coextensive_axis_rect_region_boolean(first, second, op, policy)? {
        Classification::Decided(Some(region)) => {
            return Ok(Classification::Decided(clone_boundary_contours(
                &region.as_view(),
            )));
        }
        Classification::Decided(None) => {}
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    }
    match boundary_contact_resolution(first, second, policy)? {
        Classification::Decided(Some(BoundaryContactResolution::BoundaryOnly(kind))) => {
            return boundary_contact_boundary_contours(first, second, op, fill_rule, policy, kind);
        }
        Classification::Decided(Some(BoundaryContactResolution::Containment {
            relation,
            contact,
        })) => {
            if let Some(contours) = containment_boundary_contours(first, second, op, relation) {
                return Ok(Classification::Decided(contours));
            }
            if relation == BoundaryContainmentRelation::FirstContainsSecond
                && contact == BoundaryContactKind::Overlap
                && op == BooleanOp::Difference
            {
                return containment_difference_boundary_contours(first, second, fill_rule, policy);
            }
        }
        Classification::Decided(None) => {
            if op == BooleanOp::Union && region_boundary_has_overlap(first, second, policy)? {
                return boundary_overlap_union_contours(first, second, op, fill_rule, policy);
            }
        }
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    }
    if op == BooleanOp::Xor {
        return xor_boundary_contours_by_region(first, second, fill_rule, policy);
    }

    match boolean_boundary_loops_between(first, second, op, policy)? {
        Classification::Decided(loops) => {
            loops.into_contours(fill_rule).map(Classification::Decided)
        }
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
    // This follows the set identity used in Martinez, Rueda, and Feito's
    // segment-selection view of polygon booleans (F. Martinez, A. J. Rueda,
    // and F. R. Feito, "A new algorithm for computing Boolean operations on
    // polygons," Computers & Geosciences 35(6), 1177-1185, 2009) while keeping
    // remaining ambiguous shared boundaries out of the direct traversal graph
    // until the general overlap/branch resolver lands.
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
) -> CurveResult<Classification<Region2>> {
    if same_region_view(first, second) {
        return Ok(Classification::Decided(match op {
            BooleanOp::Union | BooleanOp::Intersection => clone_region(first),
            BooleanOp::Difference | BooleanOp::Xor => Region2::empty(),
        }));
    }
    if first.is_empty() || second.is_empty() {
        return Ok(Classification::Decided(empty_operand_region(
            first, second, op,
        )));
    }
    match coextensive_axis_rect_region_boolean(first, second, op, policy)? {
        Classification::Decided(Some(region)) => return Ok(Classification::Decided(region)),
        Classification::Decided(None) => {}
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    }
    match boundary_contact_resolution(first, second, policy)? {
        Classification::Decided(Some(BoundaryContactResolution::BoundaryOnly(kind))) => {
            return boundary_contact_region(first, second, op, fill_rule, policy, kind);
        }
        Classification::Decided(Some(BoundaryContactResolution::Containment {
            relation,
            contact,
        })) => {
            if let Some(region) = containment_region(first, second, op, relation) {
                return Ok(Classification::Decided(region));
            }
            if relation == BoundaryContainmentRelation::FirstContainsSecond
                && contact == BoundaryContactKind::Overlap
                && op == BooleanOp::Difference
            {
                return match containment_difference_boundary_contours(
                    first, second, fill_rule, policy,
                )? {
                    Classification::Decided(contours) => {
                        Region2::from_boundary_contours(contours, policy)
                    }
                    Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
                };
            }
        }
        Classification::Decided(None) => {
            if op == BooleanOp::Union && region_boundary_has_overlap(first, second, policy)? {
                return match boundary_overlap_union_contours(first, second, op, fill_rule, policy)?
                {
                    Classification::Decided(contours) => {
                        Region2::from_boundary_contours(contours, policy)
                    }
                    Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
                };
            }
        }
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    }
    if op == BooleanOp::Xor {
        return xor_region_by_difference_union(first, second, fill_rule, policy);
    }

    match boolean_boundary_contours_between(first, second, op, fill_rule, policy)? {
        Classification::Decided(contours) => Region2::from_boundary_contours(contours, policy),
        Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
    }
}

pub(crate) fn boolean_region_between_with_report(
    first: &RegionView2<'_>,
    second: &RegionView2<'_>,
    op: BooleanOp,
    fill_rule: FillRule,
    policy: &CurvePolicy,
) -> CurveResult<RegionBooleanResult2> {
    let boundary_events = first.intersect_region(second, policy)?;
    let contours = match boolean_boundary_contours_between(first, second, op, fill_rule, policy)? {
        Classification::Decided(contours) => contours,
        Classification::Uncertain(reason) => {
            return Ok(blocked_region_boolean_result(
                first,
                second,
                op,
                fill_rule,
                RegionBooleanQueryPath2::Direct,
                &boundary_events,
                retained_status_for_boolean_blocker(reason),
                reason,
            ));
        }
    };
    region_boolean_result_from_boundary_contours(
        first,
        second,
        op,
        fill_rule,
        RegionBooleanQueryPath2::Direct,
        &boundary_events,
        contours,
        policy,
    )
}

pub(crate) fn region_boolean_result_from_boundary_contours(
    first: &RegionView2<'_>,
    second: &RegionView2<'_>,
    op: BooleanOp,
    fill_rule: FillRule,
    query_path: RegionBooleanQueryPath2,
    boundary_events: &RegionIntersectionSet,
    contours: Vec<Contour2>,
    policy: &CurvePolicy,
) -> CurveResult<RegionBooleanResult2> {
    region_boolean_result_from_boundary_contours_with_prepared_cache(
        first,
        second,
        op,
        fill_rule,
        query_path,
        boundary_events,
        contours,
        None,
        policy,
    )
}

#[allow(clippy::too_many_arguments)]
pub(crate) fn region_boolean_result_from_boundary_contours_with_prepared_cache(
    first: &RegionView2<'_>,
    second: &RegionView2<'_>,
    op: BooleanOp,
    fill_rule: FillRule,
    query_path: RegionBooleanQueryPath2,
    boundary_events: &RegionIntersectionSet,
    contours: Vec<Contour2>,
    prepared_cache_report: Option<RegionBooleanPreparedCacheReport2>,
    policy: &CurvePolicy,
) -> CurveResult<RegionBooleanResult2> {
    let boundary_contour_count = contours.len();
    let built = Region2::from_boundary_contours_with_report(contours, policy)?;
    let status = built.report().status();
    let blocker = built.report().blocker();
    let result_material_contour_count = built.report().material_contour_count();
    let result_hole_contour_count = built.report().hole_contour_count();
    let boundary_build_report = built.report().clone();
    Ok(RegionBooleanResult2 {
        region: built.into_region(),
        report: RegionBooleanReport2 {
            op,
            fill_rule,
            query_path,
            stage: RegionBooleanStage2::RegionRoleAssignment,
            first_material_contour_count: first.material_contours().len(),
            first_hole_contour_count: first.hole_contours().len(),
            first_boundary_segment_count: region_view_boundary_segment_count(first),
            second_material_contour_count: second.material_contours().len(),
            second_hole_contour_count: second.hole_contours().len(),
            second_boundary_segment_count: region_view_boundary_segment_count(second),
            boundary_first_contour_count: boundary_events.first_contour_count(),
            boundary_second_contour_count: boundary_events.second_contour_count(),
            boundary_candidate_pair_count: boundary_events.candidate_pair_count(),
            boundary_skipped_aabb_pair_count: boundary_events.skipped_aabb_pair_count(),
            boundary_tested_pair_count: boundary_events.tested_pair_count(),
            boundary_intersecting_pair_count: boundary_events.intersecting_pair_count(),
            boundary_contour_count: Some(boundary_contour_count),
            result_material_contour_count,
            result_hole_contour_count,
            boundary_build_report: Some(boundary_build_report),
            prepared_cache_report,
            status,
            blocker,
        },
    })
}

pub(crate) fn blocked_region_boolean_result(
    first: &RegionView2<'_>,
    second: &RegionView2<'_>,
    op: BooleanOp,
    fill_rule: FillRule,
    query_path: RegionBooleanQueryPath2,
    boundary_events: &RegionIntersectionSet,
    status: RetainedTopologyStatus,
    blocker: UncertaintyReason,
) -> RegionBooleanResult2 {
    blocked_region_boolean_result_with_prepared_cache(
        first,
        second,
        op,
        fill_rule,
        query_path,
        boundary_events,
        status,
        blocker,
        None,
    )
}

#[allow(clippy::too_many_arguments)]
pub(crate) fn blocked_region_boolean_result_with_prepared_cache(
    first: &RegionView2<'_>,
    second: &RegionView2<'_>,
    op: BooleanOp,
    fill_rule: FillRule,
    query_path: RegionBooleanQueryPath2,
    boundary_events: &RegionIntersectionSet,
    status: RetainedTopologyStatus,
    blocker: UncertaintyReason,
    prepared_cache_report: Option<RegionBooleanPreparedCacheReport2>,
) -> RegionBooleanResult2 {
    RegionBooleanResult2 {
        region: None,
        report: RegionBooleanReport2 {
            op,
            fill_rule,
            query_path,
            stage: RegionBooleanStage2::BoundaryExtraction,
            first_material_contour_count: first.material_contours().len(),
            first_hole_contour_count: first.hole_contours().len(),
            first_boundary_segment_count: region_view_boundary_segment_count(first),
            second_material_contour_count: second.material_contours().len(),
            second_hole_contour_count: second.hole_contours().len(),
            second_boundary_segment_count: region_view_boundary_segment_count(second),
            boundary_first_contour_count: boundary_events.first_contour_count(),
            boundary_second_contour_count: boundary_events.second_contour_count(),
            boundary_candidate_pair_count: boundary_events.candidate_pair_count(),
            boundary_skipped_aabb_pair_count: boundary_events.skipped_aabb_pair_count(),
            boundary_tested_pair_count: boundary_events.tested_pair_count(),
            boundary_intersecting_pair_count: boundary_events.intersecting_pair_count(),
            boundary_contour_count: None,
            result_material_contour_count: None,
            result_hole_contour_count: None,
            boundary_build_report: None,
            prepared_cache_report,
            status,
            blocker: Some(blocker),
        },
    }
}

fn region_view_boundary_segment_count(region: &RegionView2<'_>) -> usize {
    region
        .material_contours()
        .iter()
        .chain(region.hole_contours().iter())
        .map(|contour| contour.segments().len())
        .sum()
}

pub(crate) fn retained_status_for_boolean_blocker(
    reason: UncertaintyReason,
) -> RetainedTopologyStatus {
    match reason {
        UncertaintyReason::Boundary | UncertaintyReason::Unsupported => {
            RetainedTopologyStatus::Unsupported
        }
        _ => RetainedTopologyStatus::Unresolved,
    }
}

fn boundary_contact_resolution(
    first: &RegionView2<'_>,
    second: &RegionView2<'_>,
    policy: &CurvePolicy,
) -> CurveResult<Classification<Option<BoundaryContactResolution>>> {
    let intersections = first.intersect_region(second, policy)?;
    if intersections.is_empty() {
        return Ok(Classification::Decided(None));
    }

    let saw_overlap = match boundary_contact_overlap_flag(&intersections) {
        Classification::Decided(Some(saw_overlap)) => saw_overlap,
        Classification::Decided(None) => return Ok(Classification::Decided(None)),
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };

    let disjoint_interiors = if saw_overlap {
        split_contact_interiors_are_disjoint(first, second, &intersections, policy)?
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

/// Tests whether the two region boundaries have any overlapping boundary segment.
///
/// This is the boundary-stage part of the classical overlap fast-path used by
/// clipping kernels: if boundaries share non-point overlap, boolean branches that
/// are sensitive to shared edges (for example Union and Difference special cases)
/// can avoid entering the full fragment traversal.
///
/// This follows the shared-boundary split analysis in Foster, Hormann, and Popa,
/// *Clipping simple polygons with degenerate intersections*, Computers & Graphics:
/// X 2, 100007, 2019.
pub(crate) fn region_boundary_has_overlap(
    first: &RegionView2<'_>,
    second: &RegionView2<'_>,
    policy: &CurvePolicy,
) -> CurveResult<bool> {
    let intersections = first.intersect_region(second, policy)?;
    Ok(matches!(
        boundary_contact_overlap_flag(&intersections),
        Classification::Decided(Some(true))
    ))
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
    for contour_fragments in fragments.contours() {
        let opposite = match contour_fragments.key.side {
            RegionSide::First => second,
            RegionSide::Second => first,
        };

        for fragment in contour_fragments.fragments.fragments() {
            let sample = match fragment.segment.representative_point(policy)? {
                Classification::Decided(sample) => sample,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
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
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            }
        }
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
    for contour in contours {
        for segment in contour.segments() {
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
            // Hormann and Agathos, "The Point in Polygon Problem for Arbitrary
            // Polygons" (2001), rather than an epsilon-based bounding rule.
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
    // traversal guesses. They cover the subset cases Foster, Hormann, and Popa
    // separate from ordinary entry/exit traversal for degenerate polygon
    // clipping (2019). Difference is decided immediately when the left operand
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

fn containment_region(
    first: &RegionView2<'_>,
    second: &RegionView2<'_>,
    op: BooleanOp,
    relation: BoundaryContainmentRelation,
) -> Option<Region2> {
    match (relation, op) {
        (BoundaryContainmentRelation::FirstContainsSecond, BooleanOp::Union) => {
            Some(clone_region(first))
        }
        (BoundaryContainmentRelation::FirstContainsSecond, BooleanOp::Intersection) => {
            Some(clone_region(second))
        }
        (BoundaryContainmentRelation::SecondContainsFirst, BooleanOp::Union) => {
            Some(clone_region(second))
        }
        (BoundaryContainmentRelation::SecondContainsFirst, BooleanOp::Intersection) => {
            Some(clone_region(first))
        }
        (BoundaryContainmentRelation::SecondContainsFirst, BooleanOp::Difference) => {
            Some(Region2::empty())
        }
        (BoundaryContainmentRelation::Equivalent, BooleanOp::Union | BooleanOp::Intersection) => {
            Some(clone_region(first))
        }
        (BoundaryContainmentRelation::Equivalent, BooleanOp::Difference | BooleanOp::Xor) => {
            Some(Region2::empty())
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

    boundary_contours_dropping_unresolved(&fragments, &selection, fill_rule, policy)
}

fn boundary_contact_boundary_contours(
    first: &RegionView2<'_>,
    second: &RegionView2<'_>,
    op: BooleanOp,
    fill_rule: FillRule,
    policy: &CurvePolicy,
    kind: BoundaryContactKind,
) -> CurveResult<Classification<Vec<Contour2>>> {
    // Boundary-only contacts carry no filled area. Foster, Hormann, and Popa
    // identify these contact degeneracies as cases that should be handled
    // separately from ordinary traversal (E. L. Foster, K. Hormann, and R. T.
    // Popa, "Clipping simple polygons with degenerate intersections,"
    // Computers & Graphics: X 2, 100007, 2019). Point-only contacts keep their
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

    boundary_contours_dropping_unresolved(&fragments, &selection, fill_rule, policy)
}

pub(crate) fn boundary_contours_dropping_unresolved(
    fragments: &RegionFragmentSet,
    selection: &BooleanFragmentSelection,
    fill_rule: FillRule,
    policy: &CurvePolicy,
) -> CurveResult<Classification<Vec<Contour2>>> {
    match certify_unresolved_boundary_pairs(fragments, selection)? {
        Classification::Decided(()) => {}
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    }

    let emitted = selection.emit_boundary_fragments(fragments)?;

    // Certified contact handlers call this only after proving that every
    // unresolved classification represents a zero-area coincident boundary
    // edge. Dropping those edges and assembling the remaining directed graph is
    // the regularized fill-state treatment described by Vatti's scanline
    // formulation (B. R. Vatti, "A generic solution to polygon clipping,"
    // Communications of the ACM 35(7), 56-63, 1992). The containment-difference
    // caller additionally uses Foster, Hormann, and Popa's degeneracy split to
    // keep positive-area overlap and branch cases out of this helper.
    let emitted =
        BooleanBoundaryFragmentSet::new(emitted.directed_fragments().to_vec(), Vec::new())?;
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

fn certify_unresolved_boundary_pairs(
    fragments: &RegionFragmentSet,
    selection: &BooleanFragmentSelection,
) -> CurveResult<Classification<()>> {
    let unresolved = selection
        .classifications()
        .iter()
        .filter(|classification| {
            classification.action == BooleanFragmentAction::BoundaryNeedsResolution
        })
        .collect::<Vec<_>>();

    if unresolved.is_empty() {
        return Ok(Classification::Decided(()));
    }
    if unresolved.len() % 2 != 0 {
        return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
    }

    let mut paired = vec![false; unresolved.len()];
    for left_index in 0..unresolved.len() {
        if paired[left_index] {
            continue;
        }
        let left_segment = fragment_segment_for_classification(fragments, unresolved[left_index])?;
        let mut matched = false;
        for right_index in left_index + 1..unresolved.len() {
            if paired[right_index] {
                continue;
            }
            if unresolved[left_index].key.side == unresolved[right_index].key.side {
                continue;
            }
            let right_segment =
                fragment_segment_for_classification(fragments, unresolved[right_index])?;
            if segment_images_match_undirected(left_segment, right_segment) {
                paired[left_index] = true;
                paired[right_index] = true;
                matched = true;
                break;
            }
        }
        if !matched {
            return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
        }
    }

    Ok(Classification::Decided(()))
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
    left == right || left == &right.reversed()
}

fn boundary_contact_region(
    first: &RegionView2<'_>,
    second: &RegionView2<'_>,
    op: BooleanOp,
    fill_rule: FillRule,
    policy: &CurvePolicy,
    kind: BoundaryContactKind,
) -> CurveResult<Classification<Region2>> {
    Ok(Classification::Decided(match op {
        BooleanOp::Union | BooleanOp::Xor => match kind {
            BoundaryContactKind::PointOnly => {
                merge_disjoint_region_bins(clone_region(first), clone_region(second))
            }
            BoundaryContactKind::Overlap => {
                return match boundary_overlap_union_contours(first, second, op, fill_rule, policy)?
                {
                    Classification::Decided(contours) => {
                        Region2::from_boundary_contours(contours, policy)
                    }
                    Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
                };
            }
        },
        BooleanOp::Intersection => Region2::empty(),
        BooleanOp::Difference => clone_region(first),
    }))
}

fn xor_region_by_difference_union(
    first: &RegionView2<'_>,
    second: &RegionView2<'_>,
    fill_rule: FillRule,
    policy: &CurvePolicy,
) -> CurveResult<Classification<Region2>> {
    // Region XOR is the symmetric difference `(A - B) union (B - A)`. Martinez,
    // Rueda, and Feito describe polygon boolean operations as combinations of
    // selected classified segments (F. Martinez, A. J. Rueda, and F. R. Feito,
    // "A new algorithm for computing Boolean operations on polygons,"
    // Computers & Geosciences 35(6), 1177-1185, 2009); using the set identity
    // here lets the region-level API reuse the better-tested difference and
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

pub(crate) fn merge_disjoint_region_bins(first: Region2, second: Region2) -> Region2 {
    // The two symmetric-difference halves are interior-disjoint by set
    // definition. Directly merging their signed contour bins preserves
    // boundary-only contacts that a contour-only nesting pass would reject as
    // ambiguous. This mirrors Vatti's fill-state view of clipping output
    // (B. R. Vatti, "A generic solution to polygon clipping," Communications
    // of the ACM 35(7), 56-63, 1992): after the two difference regions have
    // already crossed the fill-state boundary, their explicit material/hole
    // bins can be concatenated without inventing a new traversal graph.
    let mut material_contours = first.material_contours().to_vec();
    material_contours.extend(second.material_contours().iter().cloned());
    let mut hole_contours = first.hole_contours().to_vec();
    hole_contours.extend(second.hole_contours().iter().cloned());
    Region2::new(material_contours, hole_contours)
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
                (!matched[index] && first_contour.has_same_exact_boundary(second_contour))
                    .then_some(index)
            })
        else {
            return false;
        };
        matched[index] = true;
    }

    true
}

pub(crate) fn clone_boundary_contours(region: &RegionView2<'_>) -> Vec<Contour2> {
    // Exact contour-bin identity fast paths keep coincident boundaries out of
    // the general traversal graph. Foster, Hormann, and Popa show that
    // degenerate polygon clipping benefits from separating coincident-boundary
    // cases from ordinary entry/exit traversal (E. L. Foster, K. Hormann, and
    // R. T. Popa, "Clipping simple polygons with degenerate intersections,"
    // Computers & Graphics: X 2, 100007, 2019). This fast path handles exact
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

pub(crate) fn clone_region(region: &RegionView2<'_>) -> Region2 {
    // Region-level identity fast paths preserve explicit contour roles without
    // re-entering the nesting pass. Vatti describes boolean output in terms of
    // fill-state transitions (B. R. Vatti, "A generic solution to polygon
    // clipping," Communications of the ACM 35(7), 56-63, 1992); exact identity
    // and empty-set identities reduce those transitions to cloning or dropping
    // an operand. Keeping this at the `Region2` layer matters for valid input
    // regions whose explicit material bins touch and therefore cannot be
    // reconstructed by a boundary-only containment pass.
    Region2::new(
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

pub(crate) fn empty_operand_boundary_contours(
    first: &RegionView2<'_>,
    second: &RegionView2<'_>,
    op: BooleanOp,
) -> Vec<Contour2> {
    // Empty-set identities are regularized boolean identities, so they should
    // not enter the clipping graph at all. Vatti's scanline formulation
    // describes boolean construction in terms of fill-state transitions
    // (B. R. Vatti, "A generic solution to polygon clipping," Communications
    // of the ACM 35(7), 56-63, 1992); with one empty operand, those transitions
    // reduce to the nonempty operand or to the empty set.
    match (first.is_empty(), second.is_empty(), op) {
        (true, _, BooleanOp::Union | BooleanOp::Xor) => clone_boundary_contours(second),
        (_, true, BooleanOp::Union | BooleanOp::Xor | BooleanOp::Difference) => {
            clone_boundary_contours(first)
        }
        _ => Vec::new(),
    }
}

pub(crate) fn empty_operand_region(
    first: &RegionView2<'_>,
    second: &RegionView2<'_>,
    op: BooleanOp,
) -> Region2 {
    match (first.is_empty(), second.is_empty(), op) {
        (true, _, BooleanOp::Union | BooleanOp::Xor) => clone_region(second),
        (_, true, BooleanOp::Union | BooleanOp::Xor | BooleanOp::Difference) => clone_region(first),
        _ => Region2::empty(),
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

    fn fragment_set_for(key: RegionContourKey, segment: Segment2) -> RegionContourFragments {
        RegionContourFragments {
            key,
            fragments: ContourFragmentSet::new(vec![ContourFragment {
                source_segment_index: 0,
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
            action: BooleanFragmentAction::BoundaryNeedsResolution,
        }
    }

    #[test]
    fn dropping_unresolved_boundaries_requires_opposite_fragment_pair_evidence() {
        let first_key = RegionContourKey::new(RegionSide::First, RegionContourRole::Material, 0);
        let fragments =
            RegionFragmentSet::new(vec![fragment_set_for(first_key, line_segment(0, 0, 1, 0))])
                .unwrap();
        let selection =
            BooleanFragmentSelection::new(vec![unresolved_boundary(first_key)]).unwrap();

        let result = boundary_contours_dropping_unresolved(
            &fragments,
            &selection,
            FillRule::NonZero,
            &CurvePolicy::certified(),
        )
        .unwrap();

        assert_eq!(
            result,
            Classification::Uncertain(UncertaintyReason::Boundary)
        );
    }

    #[test]
    fn dropping_unresolved_boundaries_accepts_certified_opposite_fragment_pairs() {
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

        let result = boundary_contours_dropping_unresolved(
            &fragments,
            &selection,
            FillRule::NonZero,
            &CurvePolicy::certified(),
        )
        .unwrap();

        assert_eq!(result, Classification::Decided(Vec::new()));
    }

    #[test]
    fn strip_boolean_reports_degenerate_required_rectangle_as_uncertainty() {
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
                &CurvePolicy::certified(),
            ),
            Classification::Uncertain(UncertaintyReason::Unsupported)
        );
    }

    #[test]
    fn blocked_region_boolean_report_names_boundary_extraction_stage() {
        let first = Region2::from_material_contours(vec![
            Contour2::from_bulge_vertices(&[
                BulgeVertex2::new(point(0, 0), Real::zero()),
                BulgeVertex2::new(point(1, 0), Real::zero()),
                BulgeVertex2::new(point(1, 1), Real::zero()),
                BulgeVertex2::new(point(0, 1), Real::zero()),
            ])
            .unwrap(),
        ]);
        let second = Region2::from_material_contours(vec![
            Contour2::from_bulge_vertices(&[
                BulgeVertex2::new(point(2, 0), Real::zero()),
                BulgeVertex2::new(point(3, 0), Real::zero()),
                BulgeVertex2::new(point(3, 1), Real::zero()),
                BulgeVertex2::new(point(2, 1), Real::zero()),
            ])
            .unwrap(),
        ]);
        let boundary_events =
            RegionIntersectionSet::from_parts(Vec::new(), Some(1), Some(1), 1, 1, 0).unwrap();

        let result = blocked_region_boolean_result(
            &first.as_view(),
            &second.as_view(),
            BooleanOp::Union,
            FillRule::NonZero,
            RegionBooleanQueryPath2::Direct,
            &boundary_events,
            RetainedTopologyStatus::Unsupported,
            UncertaintyReason::Unsupported,
        );

        assert!(result.region().is_none());
        assert_eq!(
            result.report().stage(),
            RegionBooleanStage2::BoundaryExtraction
        );
        assert_eq!(result.report().boundary_contour_count(), None);
        assert_eq!(
            result.report().blocker(),
            Some(UncertaintyReason::Unsupported)
        );
    }
}
