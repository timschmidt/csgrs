//! Closed contour topology.

use std::cmp::Ordering;

use hyperreal::{Real, ZeroKnowledge as ZeroStatus};

use crate::bbox::{Aabb2, aabb_decided_misses_point, decided_contour_aabb, decided_segment_aabb};
use crate::classify::{classify_oriented_line, compare_reals};
use crate::{
    BulgeVertex2, Classification, CurveError, CurvePolicy, CurveResult, CurveString2,
    CurveStringChamferReport2, LineSide, Point2, RetainedTopologyStatus, Segment2,
    UncertaintyReason,
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

/// Report for a closed-contour line-line chamfer.
#[derive(Clone, Debug, PartialEq)]
pub struct ContourChamferReport2 {
    vertex_index: usize,
    curve_string_report: CurveStringChamferReport2,
    source_segment_count: usize,
    fill_rule: FillRule,
    status: RetainedTopologyStatus,
    blocker: Option<UncertaintyReason>,
}

/// Result of a report-bearing closed-contour chamfer.
#[derive(Clone, Debug, PartialEq)]
pub struct ContourChamferResult2 {
    contour: Option<Contour2>,
    report: ContourChamferReport2,
}

/// A closed sequence of connected native segments.
#[derive(Clone, Debug, PartialEq)]
pub struct Contour2 {
    curve: CurveString2,
    fill_rule: FillRule,
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
        Ok(Self { curve, fill_rule })
    }

    /// Constructs a closed contour without checking connectivity or closure.
    pub const fn new_unchecked(curve: CurveString2, fill_rule: FillRule) -> Self {
        Self { curve, fill_rule }
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

    /// Chamfers an interior line-line contour vertex by exact parameters.
    ///
    /// `vertex_index` identifies the shared vertex between
    /// `segments[vertex_index - 1]` and `segments[vertex_index]`, with
    /// `vertex_index == 0` using the final segment as the previous segment.
    /// The underlying curve-string chamfer report is retained, and the
    /// resulting segment sequence is accepted only through the checked closed
    /// contour constructor. Wrapped vertex edits rotate the materialized closed
    /// boundary but remap retained source segment indices back to this contour.
    pub fn chamfer_line_line_vertex_by_parameters(
        &self,
        vertex_index: usize,
        previous_param: Real,
        next_param: Real,
        policy: &CurvePolicy,
    ) -> CurveResult<ContourChamferResult2> {
        if vertex_index >= self.segments().len() {
            return Err(CurveError::InvalidCurveRange);
        }
        let chamfer = if vertex_index == 0 {
            let rotated = CurveString2::try_new(wraparound_chamfer_segments(self.segments()))?;
            let mut chamfer = rotated.chamfer_line_line_vertex_by_parameters(
                1,
                previous_param,
                next_param,
                policy,
            )?;
            let source_segment_count = self.segments().len();
            chamfer.report_mut().remap_source_segment_indices(|index| {
                remap_wraparound_chamfer_source_index(index, source_segment_count)
            });
            chamfer
        } else {
            self.curve.chamfer_line_line_vertex_by_parameters(
                vertex_index,
                previous_param,
                next_param,
                policy,
            )?
        };
        let curve_string_report = chamfer.report().clone();
        let status = curve_string_report.status();
        let blocker = curve_string_report.blocker();
        let contour = match chamfer.into_curve_string() {
            Some(curve_string) => Some(Self::try_new_with_fill_rule(
                curve_string.into_segments(),
                self.fill_rule,
            )?),
            None => None,
        };
        Ok(ContourChamferResult2 {
            contour,
            report: ContourChamferReport2 {
                vertex_index,
                curve_string_report,
                source_segment_count: self.segments().len(),
                fill_rule: self.fill_rule,
                status,
                blocker,
            },
        })
    }

    /// Chamfers an interior line-line contour vertex by exact cut points.
    ///
    /// The supplied points are validated against the adjacent source line
    /// segments by the underlying curve-string operation. Materialization then
    /// goes back through the checked contour constructor, so closed topology is
    /// retained only when the resulting segment sequence is still certified.
    /// Wrapped vertex edits rotate the materialized closed boundary but remap
    /// retained source segment indices back to this contour.
    pub fn chamfer_line_line_vertex_by_points(
        &self,
        vertex_index: usize,
        previous_point: &Point2,
        next_point: &Point2,
        policy: &CurvePolicy,
    ) -> CurveResult<ContourChamferResult2> {
        if vertex_index >= self.segments().len() {
            return Err(CurveError::InvalidCurveRange);
        }
        let chamfer = if vertex_index == 0 {
            let rotated = CurveString2::try_new(wraparound_chamfer_segments(self.segments()))?;
            let mut chamfer = rotated.chamfer_line_line_vertex_by_points(
                1,
                previous_point,
                next_point,
                policy,
            )?;
            let source_segment_count = self.segments().len();
            chamfer.report_mut().remap_source_segment_indices(|index| {
                remap_wraparound_chamfer_source_index(index, source_segment_count)
            });
            chamfer
        } else {
            self.curve.chamfer_line_line_vertex_by_points(
                vertex_index,
                previous_point,
                next_point,
                policy,
            )?
        };
        let curve_string_report = chamfer.report().clone();
        let status = curve_string_report.status();
        let blocker = curve_string_report.blocker();
        let contour = match chamfer.into_curve_string() {
            Some(curve_string) => Some(Self::try_new_with_fill_rule(
                curve_string.into_segments(),
                self.fill_rule,
            )?),
            None => None,
        };
        Ok(ContourChamferResult2 {
            contour,
            report: ContourChamferReport2 {
                vertex_index,
                curve_string_report,
                source_segment_count: self.segments().len(),
                fill_rule: self.fill_rule,
                status,
                blocker,
            },
        })
    }

    /// Returns this contour's exact signed area when every segment can provide
    /// a Green's-theorem boundary contribution.
    ///
    /// The returned value is `1/2 * integral(x dy - y dx)` around the closed
    /// contour. Straight segments are polynomial and always supported.
    /// Circular arcs are supported when they carry CAD bulge data, where the
    /// circular segment term is `r^2 / 2 * (theta - sin(theta))` with
    /// `theta = 4 atan(bulge)`. Arcs constructed only from center data return
    /// `Ok(None)` until the crate grows an exact `atan2` sweep primitive.
    ///
    /// This is the line/arc counterpart to Green's-theorem area accumulation
    /// used for Bezier moments in this crate. Keeping area facts on exact
    /// curve objects follows Yap, "Towards Exact Geometric Computation,"
    /// *Computational Geometry* 7(1-2), 1997
    /// (<https://doi.org/10.1016/0925-7721(95)00040-2>).
    pub fn signed_area(&self) -> CurveResult<Option<Real>> {
        let mut area = Real::zero();

        for segment in self.segments() {
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
    /// zero before boundary and winding scans; otherwise this follows the
    /// boundary-first point-in-contour structure discussed by Hormann and
    /// Agathos, "The Point in Polygon Problem for Arbitrary Polygons"
    /// (*Computational Geometry* 20(3), 131-144, 2001), extended here to
    /// native circular-arc segments.
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
    /// to the winding number. Hormann and Agathos, "The Point in Polygon
    /// Problem for Arbitrary Polygons" (*Computational Geometry* 20(3),
    /// 131-144, 2001), survey the boundary and winding issues that motivate
    /// keeping those stages separate.
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
    /// with the bounding-box candidate pruning pattern described by Bentley
    /// and Ottmann, "Algorithms for Reporting and Counting Geometric
    /// Intersections" (1979).
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

impl ContourChamferReport2 {
    /// Returns the contour vertex index requested by the chamfer.
    pub const fn vertex_index(&self) -> usize {
        self.vertex_index
    }

    /// Returns the retained open curve-string chamfer report.
    pub const fn curve_string_report(&self) -> &CurveStringChamferReport2 {
        &self.curve_string_report
    }

    /// Returns the source contour segment count captured by this report.
    pub const fn source_segment_count(&self) -> usize {
        self.source_segment_count
    }

    /// Returns the fill rule preserved by this contour edit.
    pub const fn fill_rule(&self) -> FillRule {
        self.fill_rule
    }

    /// Returns contour chamfer materialization status.
    pub const fn status(&self) -> RetainedTopologyStatus {
        self.status
    }

    /// Returns the exact blocker for non-materialized contour chamfers.
    pub const fn blocker(&self) -> Option<UncertaintyReason> {
        self.blocker
    }
}

impl ContourChamferResult2 {
    /// Returns the materialized chamfered contour, if supported.
    pub const fn contour(&self) -> Option<&Contour2> {
        self.contour.as_ref()
    }

    /// Consumes this result and returns the materialized chamfered contour, if any.
    pub fn into_contour(self) -> Option<Contour2> {
        self.contour
    }

    /// Returns the retained contour chamfer report.
    pub const fn report(&self) -> &ContourChamferReport2 {
        &self.report
    }
}

pub(crate) fn classify_contour_point_with_cached_aabbs(
    contour: &Contour2,
    point: &Point2,
    contour_box: Option<&Aabb2>,
    segment_boxes: &[Option<Aabb2>],
    policy: &CurvePolicy,
) -> Classification<ContourPointLocation> {
    // Keep the boundary-first structure from Hormann and Agathos, "The Point
    // in Polygon Problem for Arbitrary Polygons" (Computational Geometry
    // 20(3), 131-144, 2001). Cached boxes only reject decided misses; they
    // never replace exact segment-boundary checks or the winding pass.
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

    contour_winding_number_unchecked_with_cached_aabb(contour, point, contour_box, policy)
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
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        }
    }

    Classification::Decided(false)
}

fn contour_winding_number_unchecked_with_cached_aabb(
    contour: &Contour2,
    point: &Point2,
    contour_box: Option<&Aabb2>,
    policy: &CurvePolicy,
) -> Classification<i32> {
    if contour_box_misses_point(contour_box, point, policy) {
        return Classification::Decided(0);
    }

    let mut winding = 0;
    for segment in contour.segments() {
        let delta = match segment {
            Segment2::Line(line) => process_line_winding(line.start(), line.end(), point, policy),
            Segment2::Arc(arc) => process_arc_winding(arc, point, policy),
        };
        let Some(delta) = delta else {
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

fn line_signed_area_contribution(start: &Point2, end: &Point2) -> CurveResult<Real> {
    (((start.x() * end.y()) - (end.x() * start.y())) / Real::from(2_i8)).map_err(CurveError::from)
}

fn arc_signed_area_contribution(arc: &crate::CircularArc2) -> CurveResult<Option<Real>> {
    let Some(bulge) = arc.bulge() else {
        return Ok(None);
    };

    let chord = line_signed_area_contribution(arc.start(), arc.end())?;
    let b2 = bulge * bulge;
    let one_plus_b2 = Real::one() + &b2;
    let sin_numerator = (Real::from(4_i8) * bulge) * (Real::one() - &b2);
    let sin_denominator = &one_plus_b2 * &one_plus_b2;
    let sin_theta = (sin_numerator / sin_denominator)?;
    let theta = Real::from(4_i8) * bulge.clone().atan()?;
    let segment = (arc.radius_squared() * (theta - sin_theta) / Real::from(2_i8))?;
    Ok(Some(chord + segment))
}

fn wraparound_chamfer_segments(segments: &[Segment2]) -> Vec<Segment2> {
    let mut rotated = Vec::with_capacity(segments.len());
    if let Some(last) = segments.last() {
        rotated.push(last.clone());
        rotated.extend(segments[..segments.len() - 1].iter().cloned());
    }
    rotated
}

fn remap_wraparound_chamfer_source_index(index: usize, source_segment_count: usize) -> usize {
    if index == 0 {
        source_segment_count - 1
    } else {
        index - 1
    }
}

fn validate_closed_curve_string(curve: &CurveString2) -> CurveResult<()> {
    let start = curve.start().ok_or(CurveError::EmptyCurveString)?;
    let end = curve.end().ok_or(CurveError::EmptyCurveString)?;
    match start.distance_squared(end).zero_status() {
        ZeroStatus::Zero => Ok(()),
        ZeroStatus::NonZero => Err(CurveError::DisconnectedCurveString),
        ZeroStatus::Unknown => Err(CurveError::AmbiguousCurveStringConnection),
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
    point: &Point2,
    policy: &CurvePolicy,
) -> Option<i32> {
    if le_real(start.y(), point.y(), policy)? {
        if gt_real(end.y(), point.y(), policy)? && is_left(start, end, point, policy)? {
            Some(1)
        } else {
            Some(0)
        }
    } else if le_real(end.y(), point.y(), policy)? && !is_left(start, end, point, policy)? {
        Some(-1)
    } else {
        Some(0)
    }
}

fn process_arc_winding(
    arc: &crate::CircularArc2,
    point: &Point2,
    policy: &CurvePolicy,
) -> Option<i32> {
    // Arc winding is the circular-arc extension of the boundary-first winding
    // classifier used for polygon point containment. The tests below split the
    // arc by its endpoint chord and circle interior so the horizontal-ray count
    // changes exactly when the directed arc crosses the query ray. The
    // boundary and degeneracy discipline follows Hormann and Agathos,
    // "The Point in Polygon Problem for Arbitrary Polygons" (2001).
    let start = arc.start();
    let end = arc.end();
    let is_ccw = !arc.is_clockwise();
    let point_is_left = if is_ccw {
        is_left(start, end, point, policy)?
    } else {
        is_left_or_equal(start, end, point, policy)?
    };

    let inside_circle = point_inside_circle(arc, point, policy)?;

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
    arc: &crate::CircularArc2,
    point: &Point2,
    policy: &CurvePolicy,
) -> Option<bool> {
    let distance_squared = point.distance_squared(arc.center());
    Some(matches!(
        compare_reals(&distance_squared, &arc.radius_squared(), policy)?,
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
