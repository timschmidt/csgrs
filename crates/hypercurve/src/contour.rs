//! Closed contour topology.

use std::{cell::OnceCell, cmp::Ordering, rc::Rc};

use hyperreal::{Real, RealSign, ZeroKnowledge as ZeroStatus};

use crate::bbox::{Aabb2, aabb_decided_misses_point, decided_contour_aabb, decided_segment_aabb};
use crate::classify::{classify_oriented_line, compare_reals, real_sign};
use crate::curve_string::merge_adjacent_line_segments;
use crate::{
    BulgeVertex2, Classification, CurveError, CurvePolicy, CurveResult, CurveString2, LineSeg2,
    LineSide, Point2, Segment2, UncertaintyReason,
};

/// Fill rule used when classifying contour interiors.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum FillRule {
    /// Non-zero winding rule.
    NonZero,
    /// Even-odd winding rule.
    EvenOdd,
}

/// Point location relative to a closed contour.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ContourPointLocation {
    /// The point is outside the filled contour.
    Outside,
    /// The point lies on the contour boundary.
    Boundary,
    /// The point is inside the filled contour.
    Inside,
}

/// A closed sequence of connected native segments.
#[derive(Clone, Debug)]
pub struct Contour2 {
    curve: CurveString2,
    fill_rule: FillRule,
    offset_provenance: Option<Rc<ContourOffsetProvenance2>>,
    signed_area_cache: Rc<OnceCell<CurveResult<Option<Real>>>>,
    exact_dyadic_line_aabbs_cache: Rc<OnceCell<Option<Rc<ExactDyadicLineAabbs>>>>,
}

/// Compact line bounds whose binary64 coordinates are lossless exact dyadics.
///
/// These bounds may reject candidates, but never replace exact segment predicates.
#[derive(Clone, Copy, Debug)]
pub(crate) struct ExactF64Aabb {
    pub(crate) min_x: f64,
    pub(crate) min_y: f64,
    pub(crate) max_x: f64,
    pub(crate) max_y: f64,
}

impl ExactF64Aabb {
    pub(crate) fn is_disjoint(self, other: Self) -> bool {
        self.max_x < other.min_x
            || other.max_x < self.min_x
            || self.max_y < other.min_y
            || other.max_y < self.min_y
    }
}

/// Shared certified bounds and sweep order for an all-line contour.
#[derive(Debug)]
pub(crate) struct ExactDyadicLineAabbs {
    pub(crate) contour: ExactF64Aabb,
    pub(crate) segments: Vec<ExactF64Aabb>,
    // Bit 0 selects max x for the source start; bit 1 selects max y. Together
    // with the exact bounds this reconstructs both directed endpoints without
    // retaining four duplicate binary64 coordinates per segment.
    start_at_max: Vec<u8>,
    /// Segment indices ordered by exact dyadic minimum x. The low 16 bits hold
    /// the ordered segment index and the high 16 bits hold the segment with the
    /// greatest maximum x in the prefix through that entry. Contours exceeding
    /// the compact index range retain their bounds and use the exact fallback.
    pub(crate) min_x_order_with_prefix_max: Option<Vec<u32>>,
}

impl ExactDyadicLineAabbs {
    pub(crate) fn segment_endpoints(&self, index: usize) -> [[f64; 2]; 2] {
        debug_assert_eq!(self.segments.len(), self.start_at_max.len());
        let bounds = self.segments[index];
        let start_at_max = self.start_at_max[index];
        let (start_x, end_x) = if start_at_max & 1 == 0 {
            (bounds.min_x, bounds.max_x)
        } else {
            (bounds.max_x, bounds.min_x)
        };
        let (start_y, end_y) = if start_at_max & 2 == 0 {
            (bounds.min_y, bounds.max_y)
        } else {
            (bounds.max_y, bounds.min_y)
        };
        [[start_x, start_y], [end_x, end_y]]
    }

    #[inline]
    pub(crate) const fn ordered_segment_index(packed_entry: u32) -> usize {
        (packed_entry as u16) as usize
    }

    #[inline]
    fn prefix_max_segment_index(packed_entry: u32) -> usize {
        ((packed_entry >> 16) as u16) as usize
    }

    #[inline]
    pub(crate) fn first_possible_x_overlap(&self, min_x: f64) -> usize {
        let Some(order) = &self.min_x_order_with_prefix_max else {
            return 0;
        };
        order.partition_point(|&packed_entry| {
            self.segments[Self::prefix_max_segment_index(packed_entry)].max_x < min_x
        })
    }
}

#[derive(Debug, PartialEq)]
struct ContourOffsetSource2 {
    curve: CurveString2,
    fill_rule: FillRule,
    orientation: RealSign,
}

#[derive(Debug, PartialEq)]
struct ContourOffsetProvenance2 {
    source: Rc<ContourOffsetSource2>,
    left_distance: Real,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum RetainedContourOffsetRelation2 {
    FirstContainsSecond,
    SecondContainsFirst,
    Coincident,
    Uncertain,
}

impl PartialEq for Contour2 {
    fn eq(&self, other: &Self) -> bool {
        self.curve == other.curve && self.fill_rule == other.fill_rule
    }
}

impl Contour2 {
    /// Constructs a closed contour with the non-zero winding fill rule.
    pub fn try_new(segments: Vec<Segment2>) -> CurveResult<Self> {
        Self::try_new_with_fill_rule(segments, FillRule::NonZero)
    }

    /// Constructs a closed contour with an explicit fill rule.
    pub fn try_new_with_fill_rule(
        segments: Vec<Segment2>,
        fill_rule: FillRule,
    ) -> CurveResult<Self> {
        let curve = CurveString2::try_new(segments)?;
        validate_closed_curve_string(&curve)?;
        Ok(Self {
            curve,
            fill_rule,
            offset_provenance: None,
            signed_area_cache: Rc::new(OnceCell::new()),
            exact_dyadic_line_aabbs_cache: Rc::new(OnceCell::new()),
        })
    }

    /// Constructs a closed contour without checking connectivity or closure.
    pub fn new_unchecked(curve: CurveString2, fill_rule: FillRule) -> Self {
        Self {
            curve,
            fill_rule,
            offset_provenance: None,
            signed_area_cache: Rc::new(OnceCell::new()),
            exact_dyadic_line_aabbs_cache: Rc::new(OnceCell::new()),
        }
    }

    pub(crate) fn from_validated_closed_segments(
        segments: Vec<Segment2>,
        fill_rule: FillRule,
    ) -> Self {
        Self::new_unchecked(CurveString2::new_unchecked(segments), fill_rule)
    }

    pub(crate) fn retain_left_offset_from(
        mut self,
        source: &Self,
        distance: Real,
        policy: &CurvePolicy,
    ) -> Self {
        // A simple raw offset can re-expand after a collapse while remaining
        // self-contact free. Retain nesting only on the regular branch where
        // every output line still follows its corresponding source line.
        if self.segments().len() != source.segments().len()
            || !self
                .segments()
                .iter()
                .zip(source.segments())
                .all(|(offset, source)| match (offset, source) {
                    (Segment2::Line(offset), Segment2::Line(source)) => {
                        let (offset_x, offset_y) = offset.delta();
                        let (source_x, source_y) = source.delta();
                        let direction_dot = (&offset_x * &source_x) + (&offset_y * &source_y);
                        real_sign(&direction_dot, policy) == Some(RealSign::Positive)
                    }
                    _ => false,
                })
        {
            return self;
        }

        let provenance = match source.offset_provenance.as_ref() {
            None => {
                let Some(orientation) = source
                    .signed_area()
                    .ok()
                    .flatten()
                    .and_then(|area| real_sign(&area, policy))
                else {
                    return self;
                };
                ContourOffsetProvenance2 {
                    source: Rc::new(ContourOffsetSource2 {
                        curve: source.curve.clone(),
                        fill_rule: source.fill_rule,
                        orientation,
                    }),
                    left_distance: distance,
                }
            }
            Some(provenance) => ContourOffsetProvenance2 {
                source: provenance.source.clone(),
                left_distance: &provenance.left_distance + &distance,
            },
        };
        self.offset_provenance = Some(Rc::new(provenance));
        self
    }

    pub(crate) fn retained_offset_relation(
        &self,
        other: &Self,
        policy: &CurvePolicy,
    ) -> Option<RetainedContourOffsetRelation2> {
        let (Some(first), Some(second)) = (
            self.offset_provenance.as_ref(),
            other.offset_provenance.as_ref(),
        ) else {
            return None;
        };
        if first.source != second.source {
            return None;
        }

        Some(
            match compare_reals(&first.left_distance, &second.left_distance, policy) {
                Some(Ordering::Equal) => RetainedContourOffsetRelation2::Coincident,
                Some(ordering) => match (first.source.orientation, ordering) {
                    (RealSign::Positive, Ordering::Less)
                    | (RealSign::Negative, Ordering::Greater) => {
                        RetainedContourOffsetRelation2::FirstContainsSecond
                    }
                    (RealSign::Positive, Ordering::Greater)
                    | (RealSign::Negative, Ordering::Less) => {
                        RetainedContourOffsetRelation2::SecondContainsFirst
                    }
                    (RealSign::Zero, _) => RetainedContourOffsetRelation2::Uncertain,
                    (_, Ordering::Equal) => RetainedContourOffsetRelation2::Coincident,
                },
                None => RetainedContourOffsetRelation2::Uncertain,
            },
        )
    }

    pub(crate) fn has_retained_regular_offset_branch(&self) -> bool {
        self.offset_provenance.is_some()
    }

    /// Constructs a closed contour from exact bulge vertices.
    ///
    /// The final vertex's bulge defines the segment back to the first vertex.
    pub fn from_bulge_vertices(vertices: &[BulgeVertex2]) -> CurveResult<Self> {
        Self::from_bulge_vertices_with_fill_rule(vertices, FillRule::NonZero)
    }

    /// Constructs a closed contour from exact bulge vertices and a fill rule.
    pub fn from_bulge_vertices_with_fill_rule(
        vertices: &[BulgeVertex2],
        fill_rule: FillRule,
    ) -> CurveResult<Self> {
        if vertices.len() < 2 {
            return Err(CurveError::InsufficientVertices);
        }

        let mut segments = Vec::with_capacity(vertices.len());
        for adjacent in vertices.windows(2) {
            segments.push(adjacent[0].segment_to(&adjacent[1])?);
        }
        segments.push(vertices[vertices.len() - 1].segment_to(&vertices[0])?);
        Self::try_new_with_fill_rule(segments, fill_rule)
    }

    /// Returns the underlying closed curve string.
    pub const fn curve_string(&self) -> &CurveString2 {
        &self.curve
    }

    /// Returns the segments in contour order.
    pub fn segments(&self) -> &[Segment2] {
        self.curve.segments()
    }

    /// Returns true when two closed contours have the same exact boundary.
    ///
    /// This is an exact structural comparison, not a geometric overlap test. It
    /// accepts cyclic start-index changes and reversed traversal direction, but
    /// it still requires the same fill rule and the same unsplit segment
    /// sequence up to those two closed-contour symmetries.
    pub fn has_same_exact_boundary(&self, other: &Self) -> bool {
        self.fill_rule == other.fill_rule
            && same_exact_segment_cycle(self.segments(), other.segments())
    }

    /// Returns the fill rule.
    pub const fn fill_rule(&self) -> FillRule {
        self.fill_rule
    }

    /// Merges adjacent same-direction line segments around this closed contour.
    ///
    /// This is the closed-boundary counterpart to
    /// [`CurveString2::merge_adjacent_collinear_lines`]. It inspects the
    /// wraparound adjacency as well as interior adjacencies, preserves corners,
    /// arcs, and collinear reversals, and evidence source segment indices for
    /// every output contour segment. If any line-line support or direction
    /// predicate is unresolved, no contour is materialized.
    pub fn merge_adjacent_collinear_lines(
        &self,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<Contour2>> {
        let source_segment_count = self.segments().len();
        let mut adjacency = Vec::with_capacity(source_segment_count);
        let mut break_index = None;
        for index in 0..source_segment_count {
            let next_index = (index + 1) % source_segment_count;
            match merge_adjacent_line_segments(
                &self.segments()[index],
                &self.segments()[next_index],
                policy,
            )? {
                Classification::Decided(Some(_)) => {
                    adjacency.push(true);
                }
                Classification::Decided(None) => {
                    adjacency.push(false);
                    break_index = Some(index);
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }

        let Some(break_index) = break_index else {
            return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
        };

        let start_index = (break_index + 1) % source_segment_count;
        let mut output_segments = Vec::with_capacity(source_segment_count);
        let mut run_start_index = start_index;
        let mut current_index = start_index;
        loop {
            let next_index = (current_index + 1) % source_segment_count;
            if next_index == start_index {
                push_contour_line_merge_run(
                    self.segments(),
                    run_start_index,
                    current_index,
                    &mut output_segments,
                )?;
                break;
            }

            if !adjacency[current_index] {
                push_contour_line_merge_run(
                    self.segments(),
                    run_start_index,
                    current_index,
                    &mut output_segments,
                )?;
                run_start_index = next_index;
            }
            current_index = next_index;
        }

        Self::try_new_with_fill_rule(output_segments, self.fill_rule).map(Classification::Decided)
    }

    /// Chamfers an interior native-segment contour vertex by exact parameters.
    pub fn chamfer_vertex_by_parameters(
        &self,
        vertex_index: usize,
        previous_param: Real,
        next_param: Real,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<Contour2>> {
        if vertex_index >= self.segments().len() {
            return Err(CurveError::InvalidCurveRange);
        }
        let edit = if vertex_index == 0 {
            CurveString2::try_new(wraparound_chamfer_segments(self.segments()))?
                .chamfer_vertex_by_parameters(1, previous_param, next_param, policy)?
        } else {
            self.curve.chamfer_vertex_by_parameters(
                vertex_index,
                previous_param,
                next_param,
                policy,
            )?
        };
        self.classify_edited_curve_string(edit)
    }

    /// Chamfers an interior native-segment contour vertex by exact cut points.
    pub fn chamfer_vertex_by_points(
        &self,
        vertex_index: usize,
        previous_point: &Point2,
        next_point: &Point2,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<Contour2>> {
        if vertex_index >= self.segments().len() {
            return Err(CurveError::InvalidCurveRange);
        }
        let edit = if vertex_index == 0 {
            CurveString2::try_new(wraparound_chamfer_segments(self.segments()))?
                .chamfer_vertex_by_points(1, previous_point, next_point, policy)?
        } else {
            self.curve
                .chamfer_vertex_by_points(vertex_index, previous_point, next_point, policy)?
        };
        self.classify_edited_curve_string(edit)
    }

    /// Fillets an interior native-segment contour vertex by exact parameters and center.
    pub fn fillet_vertex_by_parameters(
        &self,
        vertex_index: usize,
        previous_param: Real,
        next_param: Real,
        center: &Point2,
        clockwise: bool,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<Contour2>> {
        if vertex_index >= self.segments().len() {
            return Err(CurveError::InvalidCurveRange);
        }
        let edit = if vertex_index == 0 {
            CurveString2::try_new(wraparound_chamfer_segments(self.segments()))?
                .fillet_vertex_by_parameters(
                    1,
                    previous_param,
                    next_param,
                    center,
                    clockwise,
                    policy,
                )?
        } else {
            self.curve.fillet_vertex_by_parameters(
                vertex_index,
                previous_param,
                next_param,
                center,
                clockwise,
                policy,
            )?
        };
        self.classify_edited_curve_string(edit)
    }

    /// Fillets an interior native-segment contour vertex by exact tangent points and center.
    pub fn fillet_vertex_by_points(
        &self,
        vertex_index: usize,
        previous_point: &Point2,
        next_point: &Point2,
        center: &Point2,
        clockwise: bool,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<Contour2>> {
        if vertex_index >= self.segments().len() {
            return Err(CurveError::InvalidCurveRange);
        }
        let edit = if vertex_index == 0 {
            CurveString2::try_new(wraparound_chamfer_segments(self.segments()))?
                .fillet_vertex_by_points(1, previous_point, next_point, center, clockwise, policy)?
        } else {
            self.curve.fillet_vertex_by_points(
                vertex_index,
                previous_point,
                next_point,
                center,
                clockwise,
                policy,
            )?
        };
        self.classify_edited_curve_string(edit)
    }

    fn classify_edited_curve_string(
        &self,
        edit: Classification<CurveString2>,
    ) -> CurveResult<Classification<Contour2>> {
        match edit {
            Classification::Decided(curve_string) => {
                Self::try_new_with_fill_rule(curve_string.into_segments(), self.fill_rule)
                    .map(Classification::Decided)
            }
            Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
        }
    }

    /// Returns this contour's exact signed area when every segment can provide
    /// a Green's-theorem boundary contribution.
    ///
    /// The returned value is `1/2 * integral(x dy - y dx)` around the closed
    /// contour. Straight segments are polynomial and always supported.
    /// Circular arcs use the circular-segment term
    /// `r^2 / 2 * (theta - sin(theta))`. Bulge arcs retain
    /// `theta = 4 atan(bulge)`; center-defined arcs derive the directed sweep
    /// from exact radial cross/dot evidence and Hyperreal `atan2`.
    ///
    /// This is the line/arc counterpart to Green's-theorem area accumulation
    /// used for Bezier moments in this crate. Keeping area facts on exact
    /// curve objects follows exact-computation discipline.
    /// The exact result is computed lazily once and shared by contour clones.
    pub fn signed_area(&self) -> CurveResult<Option<Real>> {
        self.signed_area_cache
            .get_or_init(|| compute_contour_signed_area(self.segments()))
            .clone()
    }

    pub(crate) fn cached_signed_area(&self) -> Option<&Real> {
        self.signed_area_cache.get()?.as_ref().ok()?.as_ref()
    }

    pub(crate) fn exact_dyadic_line_aabbs(
        &self,
        policy: &CurvePolicy,
    ) -> Option<Rc<ExactDyadicLineAabbs>> {
        if policy != &CurvePolicy::certified() {
            return None;
        }
        self.exact_dyadic_line_aabbs_cache
            .get_or_init(|| exact_dyadic_line_aabbs(self.segments()).map(Rc::new))
            .clone()
    }

    /// Returns the segment count.
    pub fn len(&self) -> usize {
        self.curve.len()
    }

    /// Returns true when there are no segments.
    pub fn is_empty(&self) -> bool {
        self.curve.is_empty()
    }

    /// Computes the winding number for a point not on the boundary.
    ///
    /// Boundary points return `Uncertain(Boundary)` because a Real winding
    /// number is not well-defined there. A decided bounding-box miss returns
    /// zero before boundary and winding scans; otherwise this follows
    /// boundary-first winding classification, extended to native circular arcs.
    pub fn winding_number(&self, point: &Point2, policy: &CurvePolicy) -> Classification<i32> {
        let contour_box = decided_contour_aabb(self, policy);
        let segment_boxes = decided_segment_boxes(self.segments(), policy);
        contour_winding_number_with_cached_aabbs(
            self,
            point,
            contour_box.as_ref(),
            &segment_boxes,
            policy,
        )
    }

    /// Classifies a point against this contour.
    ///
    /// The query first uses the contour bounding box as a conservative rejection
    /// test, then checks the boundary explicitly before applying the fill rule
    /// to the winding number. Keeping those stages separate makes boundary
    /// handling explicit.
    pub fn classify_point(
        &self,
        point: &Point2,
        policy: &CurvePolicy,
    ) -> Classification<ContourPointLocation> {
        let contour_box = decided_contour_aabb(self, policy);
        let segment_boxes = decided_segment_boxes(self.segments(), policy);
        classify_contour_point_with_cached_aabbs(
            self,
            point,
            contour_box.as_ref(),
            &segment_boxes,
            policy,
        )
    }

    /// Returns true when the point lies on any segment of the contour.
    ///
    /// Segment boxes are used only to skip decided misses. A box hit or
    /// uncertain ordering still falls back to exact segment containment so edge
    /// and vertex boundary cases remain explicit.
    pub fn point_on_boundary(&self, point: &Point2, policy: &CurvePolicy) -> Classification<bool> {
        let contour_box = decided_contour_aabb(self, policy);
        let segment_boxes = decided_segment_boxes(self.segments(), policy);
        point_on_contour_boundary_with_cached_aabbs(
            self,
            point,
            contour_box.as_ref(),
            &segment_boxes,
            policy,
        )
    }

    /// Collects normalized topology events against another contour.
    pub fn intersect_contour(
        &self,
        other: &Self,
        policy: &CurvePolicy,
    ) -> CurveResult<crate::ContourIntersectionSet> {
        crate::events::intersect_contours(self, other, policy)
    }

    /// Collects normalized topology events between segments of this contour.
    ///
    /// Adjacent segment endpoint contacts are ordinary contour connectivity and
    /// are filtered out. Crossings, tangencies, endpoint contacts, and overlaps
    /// that are not just the connected vertex remain in the result. This keeps
    /// the same exact pair enumeration used for contour-pair intersections,
    /// with the bounding-box candidate pruning pattern described by sweep-line scheduling.
    pub fn intersect_self(
        &self,
        policy: &CurvePolicy,
    ) -> CurveResult<crate::ContourIntersectionSet> {
        crate::events::intersect_contour_self(self, policy)
    }

    /// Splits this contour into traversal-order fragments at events from one
    /// contour-pair intersection set.
    pub fn split_at_intersections(
        &self,
        intersections: &crate::ContourIntersectionSet,
        operand: crate::ContourOperand,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<crate::ContourFragmentSet>> {
        crate::fragment::split_contour_at_intersections(self, intersections, operand, policy)
    }

    /// Splits this contour into traversal-order fragments at self-intersection
    /// events collected from this same contour.
    pub fn split_at_self_intersections(
        &self,
        intersections: &crate::ContourIntersectionSet,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<crate::ContourFragmentSet>> {
        crate::fragment::split_contour_at_self_intersections(self, intersections, policy)
    }
}

pub(crate) fn classify_contour_point_with_cached_aabbs(
    contour: &Contour2,
    point: &Point2,
    contour_box: Option<&Aabb2>,
    segment_boxes: &[Option<Aabb2>],
    policy: &CurvePolicy,
) -> Classification<ContourPointLocation> {
    // Keep the boundary-first point-in-polygon structure. Cached boxes only
    // reject decided misses; they never replace exact segment-boundary checks
    // or the winding pass.
    if contour_box_misses_point(contour_box, point, policy) {
        return Classification::Decided(ContourPointLocation::Outside);
    }

    match point_on_contour_boundary_with_cached_aabbs(
        contour,
        point,
        contour_box,
        segment_boxes,
        policy,
    ) {
        Classification::Decided(true) => {
            return Classification::Decided(ContourPointLocation::Boundary);
        }
        Classification::Decided(false) => {}
        Classification::Uncertain(reason) => return Classification::Uncertain(reason),
    }

    let winding = match contour_winding_number_unchecked_with_cached_aabb(
        contour,
        point,
        contour_box,
        segment_boxes,
        policy,
    ) {
        Classification::Decided(winding) => winding,
        Classification::Uncertain(reason) => return Classification::Uncertain(reason),
    };

    let inside = match contour.fill_rule {
        FillRule::NonZero => winding != 0,
        FillRule::EvenOdd => winding.rem_euclid(2) != 0,
    };

    Classification::Decided(if inside {
        ContourPointLocation::Inside
    } else {
        ContourPointLocation::Outside
    })
}

pub(crate) fn contour_winding_number_with_cached_aabbs(
    contour: &Contour2,
    point: &Point2,
    contour_box: Option<&Aabb2>,
    segment_boxes: &[Option<Aabb2>],
    policy: &CurvePolicy,
) -> Classification<i32> {
    if contour_box_misses_point(contour_box, point, policy) {
        return Classification::Decided(0);
    }

    match point_on_contour_boundary_with_cached_aabbs(
        contour,
        point,
        contour_box,
        segment_boxes,
        policy,
    ) {
        Classification::Decided(true) => {
            return Classification::Uncertain(UncertaintyReason::Boundary);
        }
        Classification::Decided(false) => {}
        Classification::Uncertain(reason) => return Classification::Uncertain(reason),
    }

    contour_winding_number_unchecked_with_cached_aabb(
        contour,
        point,
        contour_box,
        segment_boxes,
        policy,
    )
}

pub(crate) fn point_on_contour_boundary_with_cached_aabbs(
    contour: &Contour2,
    point: &Point2,
    contour_box: Option<&Aabb2>,
    segment_boxes: &[Option<Aabb2>],
    policy: &CurvePolicy,
) -> Classification<bool> {
    if contour_box_misses_point(contour_box, point, policy) {
        return Classification::Decided(false);
    }

    let mut blocker = None;
    for (index, segment) in contour.segments().iter().enumerate() {
        if segment_boxes
            .get(index)
            .and_then(Option::as_ref)
            .is_some_and(|bbox| aabb_decided_misses_point(bbox, point, policy))
        {
            continue;
        }

        match segment.contains_point(point, policy) {
            Classification::Decided(true) => return Classification::Decided(true),
            Classification::Decided(false) => {}
            Classification::Uncertain(reason) => {
                blocker.get_or_insert(reason);
            }
        }
    }

    match blocker {
        Some(reason) => Classification::Uncertain(reason),
        None => Classification::Decided(false),
    }
}

fn contour_winding_number_unchecked_with_cached_aabb(
    contour: &Contour2,
    point: &Point2,
    contour_box: Option<&Aabb2>,
    segment_boxes: &[Option<Aabb2>],
    policy: &CurvePolicy,
) -> Classification<i32> {
    if contour_box_misses_point(contour_box, point, policy) {
        return Classification::Decided(0);
    }

    let mut winding = 0;
    for (index, segment) in contour.segments().iter().enumerate() {
        let segment_box = segment_boxes.get(index).and_then(Option::as_ref);
        // Winding casts a horizontal ray toward positive x. A segment whose
        // certified maximum x is strictly left of the query cannot cross that
        // ray. Boundary membership has already been checked, so equality stays
        // in the exact winding path while strict separation is safe to skip.
        if segment_box.is_some_and(|bbox| {
            matches!(
                compare_reals(bbox.max_x(), point.x(), policy),
                Some(Ordering::Less)
            )
        }) {
            continue;
        }
        let delta = match segment {
            Segment2::Line(line) => {
                process_line_winding(line.start(), line.end(), segment_box, point, policy)
            }
            Segment2::Arc(arc) => process_arc_winding(arc, point, policy),
        };
        let Some(delta) = delta else {
            return Classification::Uncertain(UncertaintyReason::Ordering);
        };
        winding += delta;
    }

    Classification::Decided(winding)
}

pub(crate) fn line_contour_winding_assuming_off_boundary(
    contour: &Contour2,
    point: &Point2,
    policy: &CurvePolicy,
) -> Classification<i32> {
    let mut winding = 0;
    for segment in contour.segments() {
        let Segment2::Line(line) = segment else {
            return Classification::Uncertain(UncertaintyReason::Unsupported);
        };
        let Some(delta) = process_line_winding(line.start(), line.end(), None, point, policy)
        else {
            return Classification::Uncertain(UncertaintyReason::Ordering);
        };
        winding += delta;
    }
    Classification::Decided(winding)
}

fn contour_box_misses_point(
    contour_box: Option<&Aabb2>,
    point: &Point2,
    policy: &CurvePolicy,
) -> bool {
    contour_box.is_some_and(|bbox| aabb_decided_misses_point(bbox, point, policy))
}

fn decided_segment_boxes(segments: &[Segment2], policy: &CurvePolicy) -> Vec<Option<Aabb2>> {
    segments
        .iter()
        .map(|segment| decided_segment_aabb(segment, policy))
        .collect()
}

fn exact_dyadic_line_aabbs(segments: &[Segment2]) -> Option<ExactDyadicLineAabbs> {
    let mut bounds = Vec::with_capacity(segments.len());
    let mut start_at_max = Vec::with_capacity(segments.len());
    for segment in segments {
        let Segment2::Line(line) = segment else {
            return None;
        };
        let start_x = line.start().x().to_f64_exact_dyadic()?;
        let start_y = line.start().y().to_f64_exact_dyadic()?;
        let end_x = line.end().x().to_f64_exact_dyadic()?;
        let end_y = line.end().y().to_f64_exact_dyadic()?;
        if ![start_x, start_y, end_x, end_y]
            .into_iter()
            .all(f64::is_finite)
        {
            return None;
        }
        bounds.push(ExactF64Aabb {
            min_x: start_x.min(end_x),
            min_y: start_y.min(end_y),
            max_x: start_x.max(end_x),
            max_y: start_y.max(end_y),
        });
        start_at_max.push(u8::from(start_x > end_x) | (u8::from(start_y > end_y) << 1));
    }
    let segments = bounds;
    let mut boxes = segments.iter().copied();
    let first = boxes.next()?;
    let contour = boxes.fold(first, |bounds, segment| ExactF64Aabb {
        min_x: bounds.min_x.min(segment.min_x),
        min_y: bounds.min_y.min(segment.min_y),
        max_x: bounds.max_x.max(segment.max_x),
        max_y: bounds.max_y.max(segment.max_y),
    });
    let min_x_order_with_prefix_max = (segments.len() <= usize::from(u16::MAX) + 1).then(|| {
        let mut order = (0..segments.len())
            .map(|index| index as u32)
            .collect::<Vec<_>>();
        order.sort_unstable_by(|left, right| {
            segments[*left as usize]
                .min_x
                .total_cmp(&segments[*right as usize].min_x)
                .then_with(|| left.cmp(right))
        });
        let mut prefix_max_segment_index = order[0];
        for entry in &mut order {
            let index = *entry;
            if segments[index as usize].max_x > segments[prefix_max_segment_index as usize].max_x {
                prefix_max_segment_index = index;
            }
            *entry = index | (prefix_max_segment_index << 16);
        }
        order
    });
    Some(ExactDyadicLineAabbs {
        contour,
        segments,
        start_at_max,
        min_x_order_with_prefix_max,
    })
}

fn line_signed_area_contribution(start: &Point2, end: &Point2) -> CurveResult<Real> {
    (line_doubled_signed_area_contribution(start, end) / Real::from(2_i8)).map_err(CurveError::from)
}

fn line_doubled_signed_area_contribution(start: &Point2, end: &Point2) -> Real {
    Real::diff_of_products(start.x(), end.y(), end.x(), start.y())
}

fn compute_contour_signed_area(segments: &[Segment2]) -> CurveResult<Option<Real>> {
    if segments
        .iter()
        .all(|segment| matches!(segment, Segment2::Line(_)))
    {
        let doubled_area = segments.iter().fold(Real::zero(), |area, segment| {
            let Segment2::Line(line) = segment else {
                unreachable!("all-line contour was checked before accumulation")
            };
            area + line_doubled_signed_area_contribution(line.start(), line.end())
        });
        return (doubled_area / Real::from(2_i8))
            .map(Some)
            .map_err(CurveError::from);
    }

    let mut area = Real::zero();

    for segment in segments {
        match segment {
            Segment2::Line(line) => {
                area = &area + &line_signed_area_contribution(line.start(), line.end())?;
            }
            Segment2::Arc(arc) => match arc_signed_area_contribution(arc)? {
                Some(contribution) => area = &area + &contribution,
                None => return Ok(None),
            },
        }
    }

    Ok(Some(area))
}

fn arc_signed_area_contribution(arc: &crate::CircularArc2) -> CurveResult<Option<Real>> {
    let chord = line_signed_area_contribution(arc.start(), arc.end())?;
    let segment = match arc.bulge() {
        Some(bulge) => {
            let b2 = bulge * bulge;
            let one_plus_b2 = Real::one() + &b2;
            let sin_numerator = (Real::from(4_i8) * bulge) * (Real::one() - &b2);
            let sin_denominator = &one_plus_b2 * &one_plus_b2;
            let sin_theta = (sin_numerator / sin_denominator)?;
            let theta = Real::from(4_i8) * bulge.clone().atan()?;
            (arc.radius_squared() * (theta - sin_theta) / Real::from(2_i8))?
        }
        None => {
            let start = arc.start().delta_from(arc.center());
            let end = arc.end().delta_from(arc.center());
            let cross = (&start.0 * &end.1) - (&start.1 * &end.0);
            let dot = (&start.0 * &end.0) + (&start.1 * &end.1);
            let Some(theta) = center_arc_signed_sweep(arc, cross.clone(), dot)? else {
                return Ok(None);
            };
            ((arc.radius_squared() * theta - cross) / Real::from(2_i8))?
        }
    };
    Ok(Some(chord + segment))
}

fn center_arc_signed_sweep(
    arc: &crate::CircularArc2,
    cross: Real,
    dot: Real,
) -> CurveResult<Option<Real>> {
    let sweep = match crate::arc_bezier::classify_sweep(arc) {
        Ok(sweep) => sweep,
        Err(crate::ExactCurveError::Blocked(_)) => return Ok(None),
        Err(crate::ExactCurveError::Invalid { cause, .. }) => return Err(cause),
    };
    let theta = match sweep {
        crate::arc_bezier::ArcSweepKind::FullCircle => {
            if arc.is_clockwise() {
                -Real::tau()
            } else {
                Real::tau()
            }
        }
        crate::arc_bezier::ArcSweepKind::Semicircle => {
            if arc.is_clockwise() {
                -Real::pi()
            } else {
                Real::pi()
            }
        }
        crate::arc_bezier::ArcSweepKind::Minor => cross.atan2(dot),
        crate::arc_bezier::ArcSweepKind::Major => {
            let principal = cross.atan2(dot);
            if arc.is_clockwise() {
                principal - Real::tau()
            } else {
                principal + Real::tau()
            }
        }
    };
    Ok(Some(theta))
}

fn wraparound_chamfer_segments(segments: &[Segment2]) -> Vec<Segment2> {
    let mut rotated = Vec::with_capacity(segments.len());
    if let Some(last) = segments.last() {
        rotated.push(last.clone());
        rotated.extend(segments[..segments.len() - 1].iter().cloned());
    }
    rotated
}

fn push_contour_line_merge_run(
    source_segments: &[Segment2],
    first_source_index: usize,
    last_source_index: usize,
    output_segments: &mut Vec<Segment2>,
) -> CurveResult<()> {
    let segment = if first_source_index == last_source_index {
        source_segments[first_source_index].clone()
    } else {
        Segment2::Line(LineSeg2::try_new(
            source_segments[first_source_index].start().clone(),
            source_segments[last_source_index].end().clone(),
        )?)
    };
    output_segments.push(segment);
    Ok(())
}

fn validate_closed_curve_string(curve: &CurveString2) -> CurveResult<()> {
    match closed_curve_string_status(curve)? {
        Classification::Decided(()) => Ok(()),
        Classification::Uncertain(UncertaintyReason::Boundary) => {
            Err(CurveError::DisconnectedCurveString)
        }
        Classification::Uncertain(UncertaintyReason::RealSign) => {
            Err(CurveError::AmbiguousCurveStringConnection)
        }
        Classification::Uncertain(_) => Err(CurveError::AmbiguousCurveStringConnection),
    }
}

fn closed_curve_string_status(curve: &CurveString2) -> CurveResult<Classification<()>> {
    let start = curve.start().ok_or(CurveError::EmptyCurveString)?;
    let end = curve.end().ok_or(CurveError::EmptyCurveString)?;
    if start == end {
        return Ok(Classification::Decided(()));
    }
    Ok(closure_status_from_distance(&start.distance_squared(end)))
}

fn closure_status_from_distance(distance_squared: &Real) -> Classification<()> {
    match distance_squared.zero_status() {
        ZeroStatus::Zero => Classification::Decided(()),
        ZeroStatus::NonZero => Classification::Uncertain(UncertaintyReason::Boundary),
        ZeroStatus::Unknown => Classification::Uncertain(UncertaintyReason::RealSign),
    }
}

fn same_exact_segment_cycle(first: &[Segment2], second: &[Segment2]) -> bool {
    if first.len() != second.len() {
        return false;
    }
    if first.is_empty() {
        return true;
    }

    same_directed_segment_cycle(first, second) || same_reversed_segment_cycle(first, second)
}

fn same_directed_segment_cycle(first: &[Segment2], second: &[Segment2]) -> bool {
    let len = first.len();
    (0..len).any(|offset| {
        first
            .iter()
            .enumerate()
            .all(|(index, segment)| segment == &second[(index + offset) % len])
    })
}

fn same_reversed_segment_cycle(first: &[Segment2], second: &[Segment2]) -> bool {
    let len = first.len();
    (0..len).any(|offset| {
        first.iter().enumerate().all(|(index, segment)| {
            let reversed_index = (offset + len - 1 - index) % len;
            segment == &second[reversed_index].reversed()
        })
    })
}

fn process_line_winding(
    start: &Point2,
    end: &Point2,
    segment_box: Option<&Aabb2>,
    point: &Point2,
    policy: &CurvePolicy,
) -> Option<i32> {
    if le_real(start.y(), point.y(), policy)? {
        if !gt_real(end.y(), point.y(), policy)? {
            return Some(0);
        }
        if segment_box.is_some_and(|bbox| {
            crate::bbox::aabb_decided_strictly_right_of_point(bbox, point, policy)
        }) {
            return Some(1);
        }
        return Some(i32::from(is_left(start, end, point, policy)?));
    }
    if !le_real(end.y(), point.y(), policy)? {
        return Some(0);
    }
    if segment_box
        .is_some_and(|bbox| crate::bbox::aabb_decided_strictly_right_of_point(bbox, point, policy))
    {
        return Some(-1);
    }
    Some(if is_left(start, end, point, policy)? {
        0
    } else {
        -1
    })
}

pub(crate) fn process_arc_winding(
    arc: &crate::CircularArc2,
    point: &Point2,
    policy: &CurvePolicy,
) -> Option<i32> {
    let sweep_kind = crate::arc_bezier::classify_sweep(arc).ok()?;
    if matches!(
        sweep_kind,
        crate::arc_bezier::ArcSweepKind::Major | crate::arc_bezier::ArcSweepKind::FullCircle
    ) {
        let midpoint = match arc.retained_representative_point().as_ref().ok()? {
            Classification::Decided(midpoint) => midpoint,
            Classification::Uncertain(_) => return None,
        };
        return Some(
            process_minor_arc_winding(
                arc.start(),
                midpoint,
                arc.center(),
                arc.radius_squared_ref(),
                arc.is_clockwise(),
                point,
                policy,
            )? + process_minor_arc_winding(
                midpoint,
                arc.end(),
                arc.center(),
                arc.radius_squared_ref(),
                arc.is_clockwise(),
                point,
                policy,
            )?,
        );
    }

    process_minor_arc_winding(
        arc.start(),
        arc.end(),
        arc.center(),
        arc.radius_squared_ref(),
        arc.is_clockwise(),
        point,
        policy,
    )
}

fn process_minor_arc_winding(
    start: &Point2,
    end: &Point2,
    center: &Point2,
    radius_squared: &Real,
    clockwise: bool,
    point: &Point2,
    policy: &CurvePolicy,
) -> Option<i32> {
    // Arc winding is the circular-arc extension of the boundary-first winding
    // classifier used for polygon point containment. The tests below split the
    // arc by its endpoint chord and circle interior so the horizontal-ray count
    // changes exactly when the directed arc crosses the query ray. The
    // boundary and degeneracy discipline follows boundary-first winding classification.
    let is_ccw = !clockwise;
    let point_is_left = if is_ccw {
        is_left(start, end, point, policy)?
    } else {
        is_left_or_equal(start, end, point, policy)?
    };

    let inside_circle = point_inside_circle(center, radius_squared, point, policy)?;

    if le_real(start.y(), point.y(), policy)? {
        if gt_real(end.y(), point.y(), policy)? {
            if is_ccw {
                if point_is_left || inside_circle {
                    Some(1)
                } else {
                    Some(0)
                }
            } else if point_is_left && !inside_circle {
                Some(1)
            } else {
                Some(0)
            }
        } else if is_ccw
            && !point_is_left
            && lt_real(end.x(), point.x(), policy)?
            && lt_real(point.x(), start.x(), policy)?
            && inside_circle
        {
            Some(1)
        } else if !is_ccw
            && point_is_left
            && lt_real(start.x(), point.x(), policy)?
            && lt_real(point.x(), end.x(), policy)?
            && inside_circle
        {
            Some(-1)
        } else {
            Some(0)
        }
    } else if le_real(end.y(), point.y(), policy)? {
        if is_ccw {
            if !point_is_left && !inside_circle {
                Some(-1)
            } else {
                Some(0)
            }
        } else if point_is_left {
            if inside_circle { Some(-1) } else { Some(0) }
        } else {
            Some(-1)
        }
    } else if is_ccw
        && !point_is_left
        && lt_real(start.x(), point.x(), policy)?
        && lt_real(point.x(), end.x(), policy)?
        && inside_circle
    {
        Some(1)
    } else if !is_ccw
        && point_is_left
        && lt_real(end.x(), point.x(), policy)?
        && lt_real(point.x(), start.x(), policy)?
        && inside_circle
    {
        Some(-1)
    } else {
        Some(0)
    }
}

fn point_inside_circle(
    center: &Point2,
    radius_squared: &Real,
    point: &Point2,
    policy: &CurvePolicy,
) -> Option<bool> {
    let distance_squared = point.distance_squared(center);
    Some(matches!(
        compare_reals(&distance_squared, radius_squared, policy)?,
        Ordering::Less
    ))
}

fn is_left(start: &Point2, end: &Point2, point: &Point2, policy: &CurvePolicy) -> Option<bool> {
    match classify_oriented_line(start, end, point, policy) {
        Classification::Decided(side) => Some(side == LineSide::Left),
        Classification::Uncertain(_) => None,
    }
}

fn is_left_or_equal(
    start: &Point2,
    end: &Point2,
    point: &Point2,
    policy: &CurvePolicy,
) -> Option<bool> {
    match classify_oriented_line(start, end, point, policy) {
        Classification::Decided(side) => Some(matches!(side, LineSide::Left | LineSide::On)),
        Classification::Uncertain(_) => None,
    }
}

fn le_real(left: &Real, right: &Real, policy: &CurvePolicy) -> Option<bool> {
    Some(!matches!(
        compare_reals(left, right, policy)?,
        Ordering::Greater
    ))
}

fn lt_real(left: &Real, right: &Real, policy: &CurvePolicy) -> Option<bool> {
    Some(matches!(
        compare_reals(left, right, policy)?,
        Ordering::Less
    ))
}

fn gt_real(left: &Real, right: &Real, policy: &CurvePolicy) -> Option<bool> {
    Some(matches!(
        compare_reals(left, right, policy)?,
        Ordering::Greater
    ))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn point(x: i32, y: i32) -> Point2 {
        Point2::new(x.into(), y.into())
    }

    fn center_circle(clockwise: bool) -> Contour2 {
        Contour2::try_new(vec![
            Segment2::Arc(
                crate::CircularArc2::try_from_center(
                    point(2, 0),
                    point(-2, 0),
                    point(0, 0),
                    clockwise,
                )
                .unwrap(),
            ),
            Segment2::Arc(
                crate::CircularArc2::try_from_center(
                    point(-2, 0),
                    point(2, 0),
                    point(0, 0),
                    clockwise,
                )
                .unwrap(),
            ),
        ])
        .unwrap()
    }

    #[test]
    fn line_area_fuses_exact_products_and_preserves_symbolic_operand_order() {
        let exact_start = point(-7, 5);
        let exact_end = point(11, -13);
        assert_eq!(
            line_doubled_signed_area_contribution(&exact_start, &exact_end),
            exact_start.x() * exact_end.y() - exact_end.x() * exact_start.y()
        );

        let sqrt_two = Real::from(2).sqrt().unwrap();
        let symbolic_start = Point2::new(sqrt_two.clone(), Real::from(3));
        let symbolic_end = Point2::new(Real::from(5), -sqrt_two);
        assert_eq!(
            line_doubled_signed_area_contribution(&symbolic_start, &symbolic_end),
            symbolic_start.x() * symbolic_end.y() - symbolic_end.x() * symbolic_start.y()
        );
    }

    #[test]
    fn contour_clones_share_lazy_exact_signed_area() {
        let contour = Contour2::from_bulge_vertices(&[
            BulgeVertex2::new(point(0, 0), Real::zero()),
            BulgeVertex2::new(point(2, 0), Real::zero()),
            BulgeVertex2::new(point(2, 2), Real::zero()),
            BulgeVertex2::new(point(0, 2), Real::zero()),
        ])
        .unwrap();
        let clone = contour.clone();

        assert!(Rc::ptr_eq(
            &contour.signed_area_cache,
            &clone.signed_area_cache
        ));
        assert!(clone.signed_area_cache.get().is_none());
        assert_eq!(contour.signed_area().unwrap(), Some(Real::from(4)));
        assert_eq!(clone.signed_area().unwrap(), Some(Real::from(4)));
        assert!(clone.signed_area_cache.get().is_some());
    }

    #[test]
    fn exact_dyadic_line_bounds_use_compact_lossless_coordinates() {
        let contour = Contour2::from_bulge_vertices(&[
            BulgeVertex2::new(point(-2, 0), Real::zero()),
            BulgeVertex2::new(point(3, 0), Real::zero()),
            BulgeVertex2::new(point(3, 4), Real::zero()),
            BulgeVertex2::new(point(-2, 4), Real::zero()),
        ])
        .unwrap();
        let bounds = contour
            .exact_dyadic_line_aabbs(&CurvePolicy::certified())
            .unwrap();

        assert_eq!(size_of::<ExactF64Aabb>(), 32);
        assert_eq!(bounds.segments.len(), contour.len());
        assert_eq!(bounds.contour.min_x, -2.0);
        assert_eq!(bounds.contour.min_y, 0.0);
        assert_eq!(bounds.contour.max_x, 3.0);
        assert_eq!(bounds.contour.max_y, 4.0);
        assert_eq!(
            bounds.min_x_order_with_prefix_max.as_deref(),
            Some(&[0, 2, 3, 1][..])
        );
        assert_eq!(bounds.first_possible_x_overlap(3.0), 0);
        assert_eq!(bounds.first_possible_x_overlap(4.0), contour.len());
        assert_eq!(bounds.segment_endpoints(0), [[-2.0, 0.0], [3.0, 0.0]]);
        assert_eq!(bounds.segment_endpoints(2), [[3.0, 4.0], [-2.0, 4.0]]);
        assert_eq!(bounds.segment_endpoints(3), [[-2.0, 4.0], [-2.0, 0.0]]);
        let clone = contour.clone();
        let replay = clone
            .exact_dyadic_line_aabbs(&CurvePolicy::certified())
            .unwrap();
        assert!(Rc::ptr_eq(&bounds, &replay));
        assert!(
            contour
                .exact_dyadic_line_aabbs(&CurvePolicy::exact_symbolic())
                .is_none()
        );
    }

    #[test]
    fn exact_dyadic_line_bounds_pack_changing_prefix_maximum() {
        let contour = Contour2::from_bulge_vertices(&[
            BulgeVertex2::new(point(0, 0), Real::zero()),
            BulgeVertex2::new(point(1, 0), Real::zero()),
            BulgeVertex2::new(point(10, 1), Real::zero()),
            BulgeVertex2::new(point(10, 2), Real::zero()),
        ])
        .unwrap();
        let bounds = contour
            .exact_dyadic_line_aabbs(&CurvePolicy::certified())
            .unwrap();
        let packed = bounds.min_x_order_with_prefix_max.as_deref().unwrap();

        assert_eq!(
            packed
                .iter()
                .copied()
                .map(ExactDyadicLineAabbs::ordered_segment_index)
                .collect::<Vec<_>>(),
            [0, 3, 1, 2]
        );
        assert_eq!(
            packed
                .iter()
                .copied()
                .map(ExactDyadicLineAabbs::prefix_max_segment_index)
                .collect::<Vec<_>>(),
            [0, 3, 3, 3]
        );
        assert_eq!(bounds.first_possible_x_overlap(2.0), 1);
        assert_eq!(bounds.first_possible_x_overlap(10.0), 1);
        assert_eq!(bounds.first_possible_x_overlap(11.0), contour.len());
    }

    #[test]
    fn large_exact_polygon_signed_area_matches_closed_form() {
        let side = 256;
        let mut vertices = Vec::with_capacity(4 * side as usize);
        for x in 0..side {
            vertices.push(BulgeVertex2::new(point(x, 0), Real::zero()));
        }
        for y in 0..side {
            vertices.push(BulgeVertex2::new(point(side, y), Real::zero()));
        }
        for x in (1..=side).rev() {
            vertices.push(BulgeVertex2::new(point(x, side), Real::zero()));
        }
        for y in (1..=side).rev() {
            vertices.push(BulgeVertex2::new(point(0, y), Real::zero()));
        }

        let contour = Contour2::from_bulge_vertices(&vertices).unwrap();
        assert_eq!(contour.len(), 1_024);
        assert_eq!(
            contour.signed_area().unwrap(),
            Some(Real::from(side * side))
        );
    }

    #[test]
    fn center_defined_circle_signed_area_is_exact_in_both_orientations() {
        let expected = Real::from(4) * Real::pi();

        assert_eq!(
            center_circle(false).signed_area().unwrap(),
            Some(expected.clone())
        );
        assert_eq!(center_circle(true).signed_area().unwrap(), Some(-expected));
    }

    #[test]
    fn translated_center_defined_arc_sector_has_exact_signed_area() {
        let center = point(3, 4);
        let contour = Contour2::try_new(vec![
            Segment2::Arc(
                crate::CircularArc2::try_from_center(
                    point(4, 4),
                    point(3, 5),
                    center.clone(),
                    false,
                )
                .unwrap(),
            ),
            Segment2::Line(LineSeg2::try_new(point(3, 5), center.clone()).unwrap()),
            Segment2::Line(LineSeg2::try_new(center, point(4, 4)).unwrap()),
        ])
        .unwrap();
        let expected = (Real::pi() / Real::from(4)).unwrap();

        assert_eq!(contour.signed_area().unwrap(), Some(expected));
    }
}
