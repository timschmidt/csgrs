//! Ordered open curve strings.

use std::cmp::Ordering;
use std::collections::BTreeSet;
use std::iter::Copied;
use std::ops::Range;
use std::slice;

use hyperreal::{Rational, Real, RealSign};

use crate::bbox::{Aabb2, aabbs_decided_disjoint, decided_segment_aabb};
use crate::classify::{compare_reals, in_closed_unit_interval, is_zero, real_sign};
use crate::{
    ArcArcIntersection, BulgeVertex2, CircularArc2, Classification, CurveError, CurvePolicy,
    CurveResult, LineArcIntersection, LineArcOrder, LineArcRegion2, LineLineIntersection, LineSeg2,
    LineSide, ParamRange, Point2, RegionPointLocation, RegionQuery2, Segment2, SegmentIntersection,
    SegmentKind, UncertaintyReason,
};

/// One segment-pair event between two curve strings.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveStringIntersection {
    /// Segment index in the first curve string.
    pub a_segment_index: usize,
    /// Segment index in the second curve string.
    pub b_segment_index: usize,
    /// Primitive family of the first source segment.
    pub a_segment_kind: SegmentKind,
    /// Primitive family of the second source segment.
    pub b_segment_kind: SegmentKind,
    /// Exact start point of the first source segment.
    pub a_segment_start_point: Point2,
    /// Exact end point of the first source segment.
    pub a_segment_end_point: Point2,
    /// Exact start point of the second source segment.
    pub b_segment_start_point: Point2,
    /// Exact end point of the second source segment.
    pub b_segment_end_point: Point2,
    /// Segment relation for this pair.
    pub relation: SegmentIntersection,
}

/// Endpoint selector for open curve-string editing evidence.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CurveStringEndpoint2 {
    /// First point of the curve string.
    Start,
    /// Final point of the curve string.
    End,
}

/// Orientation selected when two open curve strings are linked.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CurveStringLinkKind2 {
    /// `first.end == second.start`; output is `first + second`.
    FirstEndToSecondStart,
    /// `first.end == second.end`; output is `first + reverse(second)`.
    FirstEndToSecondEnd,
    /// `first.start == second.start`; output is `reverse(first) + second`.
    FirstStartToSecondStart,
    /// `first.start == second.end`; output is `second + first`.
    FirstStartToSecondEnd,
}

/// Segment-local retained trim point on an open curve string.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveStringTrimPoint2 {
    segment_index: usize,
    param: Real,
}

/// An ordered sequence of connected native segments.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveString2 {
    segments: Vec<Segment2>,
}

impl CurveString2 {
    /// Constructs a curve string from validated connected segments.
    pub fn try_new(segments: Vec<Segment2>) -> CurveResult<Self> {
        if segments.is_empty() {
            return Err(CurveError::EmptyCurveString);
        }

        for segment in &segments {
            if segment.start() == segment.end() {
                return Err(CurveError::ZeroLengthLine);
            }
            match segment
                .start()
                .distance_squared(segment.end())
                .zero_status()
            {
                hyperreal::ZeroKnowledge::Zero => return Err(CurveError::ZeroLengthLine),
                hyperreal::ZeroKnowledge::NonZero | hyperreal::ZeroKnowledge::Unknown => {}
            }
        }

        for adjacent in segments.windows(2) {
            if adjacent[0].end() == adjacent[1].start() {
                continue;
            }
            let distance = adjacent[0].end().distance_squared(adjacent[1].start());
            match distance.zero_status() {
                hyperreal::ZeroKnowledge::Zero => {}
                hyperreal::ZeroKnowledge::NonZero => {
                    return Err(CurveError::DisconnectedCurveString);
                }
                hyperreal::ZeroKnowledge::Unknown => {
                    return Err(CurveError::AmbiguousCurveStringConnection);
                }
            }
        }

        Ok(Self { segments })
    }

    /// Constructs a curve string without checking connectivity.
    pub const fn new_unchecked(segments: Vec<Segment2>) -> Self {
        Self { segments }
    }

    /// Constructs an open curve string from exact bulge vertices.
    pub fn from_bulge_vertices(vertices: &[BulgeVertex2]) -> CurveResult<Self> {
        if vertices.len() < 2 {
            return Err(CurveError::InsufficientVertices);
        }

        let mut segments = Vec::with_capacity(vertices.len() - 1);
        for adjacent in vertices.windows(2) {
            segments.push(adjacent[0].segment_to(&adjacent[1])?);
        }
        Self::try_new(segments)
    }

    /// Returns the segments in order.
    pub fn segments(&self) -> &[Segment2] {
        &self.segments
    }

    /// Consumes the curve string and returns its segments.
    pub fn into_segments(self) -> Vec<Segment2> {
        self.segments
    }

    /// Returns the segment count.
    pub fn len(&self) -> usize {
        self.segments.len()
    }

    /// Returns true when there are no segments.
    pub fn is_empty(&self) -> bool {
        self.segments.is_empty()
    }

    /// Returns the first point of the curve string.
    pub fn start(&self) -> Option<&Point2> {
        self.segments.first().map(Segment2::start)
    }

    /// Returns the final point of the curve string.
    pub fn end(&self) -> Option<&Point2> {
        self.segments.last().map(Segment2::end)
    }

    /// Classifies whether one selected endpoint pair is exactly connected.
    pub fn endpoint_connection(
        &self,
        other: &Self,
        first_endpoint: CurveStringEndpoint2,
        second_endpoint: CurveStringEndpoint2,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<bool>> {
        Ok(self
            .classify_endpoint_pair(other, first_endpoint, second_endpoint, policy)?
            .1)
    }

    /// Links two open curve strings when exactly one endpoint pair is certified.
    pub fn link_connected_endpoints(
        &self,
        other: &Self,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<Option<CurveString2>>> {
        let pairs = self.endpoint_link_pairs(other, policy)?;
        let mut exact_kind = None;
        let mut unresolved = None;
        for pair in pairs {
            match pair.connection {
                Classification::Decided(true) => {
                    if exact_kind.replace(pair.kind).is_some() {
                        return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                    }
                }
                Classification::Decided(false) => {}
                Classification::Uncertain(reason) => unresolved = Some(reason),
            }
        }
        if let Some(reason) = unresolved {
            return Ok(Classification::Uncertain(reason));
        }
        match exact_kind {
            Some(kind) => Ok(Classification::Decided(Some(linked_curve_string(
                self, other, kind,
            )?))),
            None => Ok(Classification::Decided(None)),
        }
    }

    /// Links an ordered sequence of open curve strings by certified endpoints.
    pub fn link_ordered_connected_endpoints(
        curve_strings: Vec<Self>,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<CurveString2>> {
        let mut iter = curve_strings.into_iter();
        let Some(mut accumulated) = iter.next() else {
            return Err(CurveError::EmptyCurveString);
        };
        for next in iter {
            accumulated = match accumulated.link_connected_endpoints(&next, policy)? {
                Classification::Decided(Some(linked)) => linked,
                Classification::Decided(None) => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
        }
        Ok(Classification::Decided(accumulated))
    }

    /// Borrowed counterpart to the ordered endpoint-link operation.
    pub fn link_ordered_connected_endpoints_borrowed(
        curve_strings: &[Self],
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<CurveString2>> {
        Self::link_ordered_connected_endpoints(curve_strings.to_vec(), policy)
    }

    /// Connects self.end to other.start with an exact line segment.
    pub fn connect_end_to_start_with_line(
        &self,
        other: &Self,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<CurveString2>> {
        self.connect_endpoints_with_line(other, CurveStringLinkKind2::FirstEndToSecondStart, policy)
    }

    /// Connects a selected endpoint pair with an exact line segment.
    pub fn connect_endpoints_with_line(
        &self,
        other: &Self,
        kind: CurveStringLinkKind2,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<CurveString2>> {
        match self.endpoint_pair_for_kind(other, kind, policy)?.connection {
            Classification::Decided(true) => {
                Ok(Classification::Uncertain(UncertaintyReason::Boundary))
            }
            Classification::Decided(false) => {
                let (curve_string, _) = connected_curve_string(self, other, kind)?;
                Ok(Classification::Decided(curve_string))
            }
            Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
        }
    }

    /// Connects the uniquely nearest certified-disconnected endpoint pair.
    pub fn connect_nearest_endpoints_with_line(
        &self,
        other: &Self,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<CurveString2>> {
        let mut best: Option<(CurveStringLinkKind2, Real)> = None;
        let mut best_is_tied = false;
        for pair in self.endpoint_link_pairs(other, policy)? {
            match pair.connection {
                Classification::Decided(true) => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
                Classification::Decided(false) => {}
            }
            match best.as_ref() {
                None => best = Some((pair.kind, pair.distance_squared)),
                Some((_, best_distance)) => {
                    match compare_reals(&pair.distance_squared, best_distance, policy) {
                        Some(Ordering::Less) => {
                            best = Some((pair.kind, pair.distance_squared));
                            best_is_tied = false;
                        }
                        Some(Ordering::Equal) => best_is_tied = true,
                        Some(Ordering::Greater) => {}
                        None => {
                            return Ok(Classification::Uncertain(UncertaintyReason::Ordering));
                        }
                    }
                }
            }
        }
        if best_is_tied {
            return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
        }
        let Some((kind, _)) = best else {
            return Err(CurveError::EmptyCurveString);
        };
        let (curve_string, _) = connected_curve_string(self, other, kind)?;
        Ok(Classification::Decided(curve_string))
    }

    /// Merges adjacent same-direction line segments when collinearity is certified.
    ///
    /// This is an explicit editing utility, not constructor normalization:
    /// source segment runs are retained in the evidence, mixed line/arc topology
    /// is preserved, and collinear reversals are not collapsed because they are
    /// real authored backtracking topology. If a line-line pair cannot be
    /// classified under the active policy, the operation returns an unresolved
    /// evidence instead of guessing a merge boundary.
    pub fn merge_adjacent_collinear_lines(
        &self,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<CurveString2>> {
        let mut merged_segments = Vec::with_capacity(self.len());
        let mut current_segment = self
            .segments
            .first()
            .cloned()
            .ok_or(CurveError::EmptyCurveString)?;

        for next_segment in self.segments.iter().skip(1) {
            match merge_adjacent_line_segments(&current_segment, next_segment, policy)? {
                Classification::Decided(Some(merged)) => {
                    current_segment = Segment2::Line(merged);
                }
                Classification::Decided(None) => {
                    merged_segments.push(current_segment);
                    current_segment = next_segment.clone();
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }

        merged_segments.push(current_segment);
        CurveString2::try_new(merged_segments).map(Classification::Decided)
    }

    /// Removes adjacent exact reversed duplicate segment pairs.
    ///
    /// This is a structural de-duplication utility for authored backtracking,
    /// not an overlap resolver: only `segment == next.reversed()` is removed.
    /// Same-support partial overlaps, same-direction repeats, and geometric
    /// coincidences with different segmentation remain intact for the
    /// arrangement pipeline. If every segment cancels, no empty `CurveString2`
    /// is materialized and the evidence carries an explicit boundary blocker.
    pub fn remove_adjacent_reversed_duplicates(&self) -> CurveResult<Classification<CurveString2>> {
        let mut retained = Vec::with_capacity(self.len());

        for segment in self.segments.iter().cloned() {
            if retained
                .last()
                .is_some_and(|previous| previous == &segment.reversed())
            {
                retained.pop();
            } else {
                retained.push(segment);
            }
        }

        if retained.is_empty() {
            return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
        }

        CurveString2::try_new(retained).map(Classification::Decided)
    }

    /// Trims this open curve string between two segment-local parameters.
    pub fn trim_between_parameters(
        &self,
        start: CurveStringTrimPoint2,
        end: CurveStringTrimPoint2,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<CurveString2>> {
        validate_trim_point(self, &start, policy)?;
        validate_trim_point(self, &end, policy)?;
        match compare_trim_points(&start, &end, policy) {
            Some(Ordering::Less) => {}
            Some(Ordering::Equal | Ordering::Greater) => return Err(CurveError::InvalidCurveRange),
            None => return Ok(Classification::Uncertain(UncertaintyReason::Ordering)),
        }

        let mut trimmed_segments = Vec::new();
        for source_segment_index in start.segment_index..=end.segment_index {
            let range_start = if source_segment_index == start.segment_index {
                start.param.clone()
            } else {
                Real::zero()
            };
            let range_end = if source_segment_index == end.segment_index {
                end.param.clone()
            } else {
                Real::one()
            };
            let source_range = ParamRange::new(range_start, range_end);
            match trim_segment_by_range(
                &self.segments[source_segment_index],
                &source_range,
                policy,
            )? {
                SegmentTrimMaterialization::Materialized(segment) => {
                    trimmed_segments.push(segment);
                }
                SegmentTrimMaterialization::SkippedEmpty => {}
                SegmentTrimMaterialization::Unresolved(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }

        if trimmed_segments.is_empty() {
            return Err(CurveError::InvalidCurveRange);
        }
        CurveString2::try_new(trimmed_segments).map(Classification::Decided)
    }

    /// Trims this open curve string between two exact points on the path.
    pub fn trim_between_points(
        &self,
        start_point: &Point2,
        end_point: &Point2,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<CurveString2>> {
        let start = match locate_trim_point(self, start_point, policy)? {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let end = match locate_trim_point(self, end_point, policy)? {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };

        match compare_trim_points(&start.trim_point, &end.trim_point, policy) {
            Some(Ordering::Less) => {}
            Some(Ordering::Equal | Ordering::Greater) => return Err(CurveError::InvalidCurveRange),
            None => return Ok(Classification::Uncertain(UncertaintyReason::Ordering)),
        }

        self.trim_between_located_points(start, end, policy)
    }

    /// Trims this open curve string between exact point intersections with two cutters.
    pub fn trim_between_curve_intersections(
        &self,
        start_cutter: &Self,
        end_cutter: &Self,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<CurveString2>> {
        let start_events = self.intersect_curve_string(start_cutter, policy)?;
        let end_events = self.intersect_curve_string(end_cutter, policy)?;
        self.trim_between_curve_intersection_events(start_events, end_events, policy)
    }

    /// Retains the portions of this open curve string inside a region.
    ///
    /// This is the first arrangement-style trim-by-region slice. Boundary
    /// intersections against all material and hole contours are collected with
    /// exact segment relations, source segments are split at retained
    /// parameters, and each retained interval is classified by an exact native
    /// representative. Point hits split intervals; overlaps and undecidable
    /// segment relations remain explicit blockers because they require a
    /// higher-order boundary traversal rather than a local interval decision.
    pub fn trim_inside_region(
        &self,
        region: &LineArcRegion2,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<Vec<CurveString2>>> {
        trim_curve_string_inside_region(self, region, policy)
    }

    pub(crate) fn trim_inside_region_query(
        &self,
        region: &RegionQuery2<'_>,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<Vec<CurveString2>>> {
        trim_curve_string_inside_prepared_region(self, region, policy)
    }

    /// Chamfers one interior native-segment vertex by exact segment parameters.
    pub fn chamfer_vertex_by_parameters(
        &self,
        vertex_index: usize,
        previous_param: Real,
        next_param: Real,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<CurveString2>> {
        if vertex_index == 0 || vertex_index >= self.len() {
            return Err(CurveError::InvalidCurveRange);
        }
        let previous_segment_index = vertex_index - 1;
        let next_segment_index = vertex_index;
        let previous_trim = CurveStringTrimPoint2::new(previous_segment_index, previous_param);
        let next_trim = CurveStringTrimPoint2::new(next_segment_index, next_param);
        validate_trim_point(self, &previous_trim, policy)?;
        validate_trim_point(self, &next_trim, policy)?;

        match (
            compare_reals(previous_trim.param(), &Real::zero(), policy),
            compare_reals(previous_trim.param(), &Real::one(), policy),
            compare_reals(next_trim.param(), &Real::zero(), policy),
            compare_reals(next_trim.param(), &Real::one(), policy),
        ) {
            (
                Some(Ordering::Greater),
                Some(Ordering::Less),
                Some(Ordering::Greater),
                Some(Ordering::Less),
            ) => {}
            (Some(_), Some(_), Some(_), Some(_)) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
            _ => return Ok(Classification::Uncertain(UncertaintyReason::Ordering)),
        }

        let previous_source = &self.segments[previous_segment_index];
        let next_source = &self.segments[next_segment_index];
        let previous_cut = match segment_point_at_trim_parameter(
            previous_source,
            previous_trim.param(),
            policy,
        )? {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let next_cut =
            match segment_point_at_trim_parameter(next_source, next_trim.param(), policy)? {
                Classification::Decided(point) => point,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };

        let previous_range = ParamRange::new(Real::zero(), previous_trim.param().clone());
        let next_range = ParamRange::new(next_trim.param().clone(), Real::one());
        let previous_segment = match materialize_strict_native_range(
            previous_source,
            previous_source.start(),
            &previous_cut,
            &previous_range,
            policy,
        )? {
            Classification::Decided(segment) => segment,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let next_segment = match materialize_strict_native_range(
            next_source,
            &next_cut,
            next_source.end(),
            &next_range,
            policy,
        )? {
            Classification::Decided(segment) => segment,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let chamfer_segment = LineSeg2::try_new(previous_cut, next_cut)?;

        let mut segments = Vec::with_capacity(self.len() + 1);
        segments.extend(self.segments[..previous_segment_index].iter().cloned());
        segments.push(previous_segment);
        segments.push(Segment2::Line(chamfer_segment));
        segments.push(next_segment);
        segments.extend(self.segments[next_segment_index + 1..].iter().cloned());
        CurveString2::try_new(segments).map(Classification::Decided)
    }

    /// Chamfers one interior native-segment vertex by exact cut points.
    pub fn chamfer_vertex_by_points(
        &self,
        vertex_index: usize,
        previous_point: &Point2,
        next_point: &Point2,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<CurveString2>> {
        if vertex_index == 0 || vertex_index >= self.len() {
            return Err(CurveError::InvalidCurveRange);
        }
        let previous_segment = &self.segments[vertex_index - 1];
        let next_segment = &self.segments[vertex_index];
        let previous_param =
            match segment_chamfer_point_parameter(previous_segment, previous_point, policy)? {
                Classification::Decided(param) => param,
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            };
        let next_param = match segment_chamfer_point_parameter(next_segment, next_point, policy)? {
            Classification::Decided(param) => param,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        self.chamfer_vertex_by_parameters(vertex_index, previous_param, next_param, policy)
    }

    /// Fillets one interior native-segment vertex from exact parameters and center.
    pub fn fillet_vertex_by_parameters(
        &self,
        vertex_index: usize,
        previous_param: Real,
        next_param: Real,
        center: &Point2,
        clockwise: bool,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<CurveString2>> {
        if vertex_index == 0 || vertex_index >= self.len() {
            return Err(CurveError::InvalidCurveRange);
        }
        let previous_segment = &self.segments[vertex_index - 1];
        let next_segment = &self.segments[vertex_index];
        let previous_point =
            match segment_point_at_trim_parameter(previous_segment, &previous_param, policy)? {
                Classification::Decided(point) => point,
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            };
        let next_point = match segment_point_at_trim_parameter(next_segment, &next_param, policy)? {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        self.fillet_vertex(
            vertex_index,
            previous_param,
            next_param,
            &previous_point,
            &next_point,
            center,
            clockwise,
            policy,
        )
    }

    /// Fillets one interior native-segment vertex from exact tangent points and center.
    pub fn fillet_vertex_by_points(
        &self,
        vertex_index: usize,
        previous_point: &Point2,
        next_point: &Point2,
        center: &Point2,
        clockwise: bool,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<CurveString2>> {
        if vertex_index == 0 || vertex_index >= self.len() {
            return Err(CurveError::InvalidCurveRange);
        }
        let previous_segment = &self.segments[vertex_index - 1];
        let next_segment = &self.segments[vertex_index];
        let previous_param =
            match segment_chamfer_point_parameter(previous_segment, previous_point, policy)? {
                Classification::Decided(param) => param,
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            };
        let next_param = match segment_chamfer_point_parameter(next_segment, next_point, policy)? {
            Classification::Decided(param) => param,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        self.fillet_vertex(
            vertex_index,
            previous_param,
            next_param,
            previous_point,
            next_point,
            center,
            clockwise,
            policy,
        )
    }

    #[allow(clippy::too_many_arguments)]
    fn fillet_vertex(
        &self,
        vertex_index: usize,
        previous_param: Real,
        next_param: Real,
        previous_point: &Point2,
        next_point: &Point2,
        center: &Point2,
        clockwise: bool,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<CurveString2>> {
        let previous_segment_index = vertex_index - 1;
        let next_segment_index = vertex_index;
        let previous_segment = &self.segments[previous_segment_index];
        let next_segment = &self.segments[next_segment_index];
        let previous_trim = CurveStringTrimPoint2::new(previous_segment_index, previous_param);
        let next_trim = CurveStringTrimPoint2::new(next_segment_index, next_param);
        validate_trim_point(self, &previous_trim, policy)?;
        validate_trim_point(self, &next_trim, policy)?;

        match (
            compare_reals(previous_trim.param(), &Real::zero(), policy),
            compare_reals(previous_trim.param(), &Real::one(), policy),
            compare_reals(next_trim.param(), &Real::zero(), policy),
            compare_reals(next_trim.param(), &Real::one(), policy),
        ) {
            (
                Some(Ordering::Greater),
                Some(Ordering::Less),
                Some(Ordering::Greater),
                Some(Ordering::Less),
            ) => {}
            (Some(_), Some(_), Some(_), Some(_)) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
            _ => return Ok(Classification::Uncertain(UncertaintyReason::Ordering)),
        }

        let radius_squared = previous_point.distance_squared(center);
        match is_zero(&radius_squared, policy) {
            Some(false) => {}
            Some(true) => return Ok(Classification::Uncertain(UncertaintyReason::Boundary)),
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        }
        let radius_delta = &radius_squared - &next_point.distance_squared(center);
        match is_zero(&radius_delta, policy) {
            Some(true) => {}
            Some(false) => return Ok(Classification::Uncertain(UncertaintyReason::Boundary)),
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        }

        if let Some(reason) = segment_fillet_validation_blocker(
            previous_segment,
            previous_point,
            center,
            clockwise,
            policy,
        ) {
            return Ok(Classification::Uncertain(reason));
        }
        if let Some(reason) =
            segment_fillet_validation_blocker(next_segment, next_point, center, clockwise, policy)
        {
            return Ok(Classification::Uncertain(reason));
        }

        let previous_range = ParamRange::new(Real::zero(), previous_trim.param().clone());
        let next_range = ParamRange::new(next_trim.param().clone(), Real::one());
        let previous_output = match materialize_strict_native_range(
            previous_segment,
            previous_segment.start(),
            previous_point,
            &previous_range,
            policy,
        )? {
            Classification::Decided(segment) => segment,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let next_output = match materialize_strict_native_range(
            next_segment,
            next_point,
            next_segment.end(),
            &next_range,
            policy,
        )? {
            Classification::Decided(segment) => segment,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let fillet_segment = CircularArc2::new_with_certified_radius(
            previous_point.clone(),
            next_point.clone(),
            center.clone(),
            radius_squared,
            clockwise,
            None,
        );

        let mut segments = Vec::with_capacity(self.len() + 1);
        segments.extend(self.segments[..previous_segment_index].iter().cloned());
        segments.push(previous_output);
        segments.push(Segment2::Arc(fillet_segment));
        segments.push(next_output);
        segments.extend(self.segments[next_segment_index + 1..].iter().cloned());
        CurveString2::try_new(segments).map(Classification::Decided)
    }

    /// Extends one endpoint line segment to an exact point on its supporting line.
    pub fn extend_line_endpoint_to_point(
        &self,
        endpoint: CurveStringEndpoint2,
        target_point: Point2,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<CurveString2>> {
        self.extend_endpoint_to_point(endpoint, target_point, policy)
    }

    /// Extends one endpoint segment to an exact target point.
    pub fn extend_endpoint_to_point(
        &self,
        endpoint: CurveStringEndpoint2,
        target_point: Point2,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<CurveString2>> {
        let source_segment_index = match endpoint {
            CurveStringEndpoint2::Start => 0,
            CurveStringEndpoint2::End => self
                .len()
                .checked_sub(1)
                .ok_or(CurveError::EmptyCurveString)?,
        };
        let segment = self
            .segments
            .get(source_segment_index)
            .ok_or(CurveError::EmptyCurveString)?;
        match segment {
            Segment2::Line(line) => self.extend_line_endpoint_segment_to_point(
                endpoint,
                source_segment_index,
                line,
                target_point,
                policy,
            ),
            Segment2::Arc(arc) => self.extend_arc_endpoint_segment_to_point(
                endpoint,
                source_segment_index,
                arc,
                target_point,
                policy,
            ),
        }
    }

    fn extend_line_endpoint_segment_to_point(
        &self,
        endpoint: CurveStringEndpoint2,
        source_segment_index: usize,
        line: &LineSeg2,
        target_point: Point2,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<CurveString2>> {
        match line.classify_point(&target_point, policy) {
            Classification::Decided(crate::LineSide::On) => {}
            Classification::Decided(_) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        }

        let source_param = match line_point_parameter(line, &target_point, policy)? {
            Classification::Decided(param) => param,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let outside = match endpoint {
            CurveStringEndpoint2::Start => compare_reals(&source_param, &Real::zero(), policy),
            CurveStringEndpoint2::End => compare_reals(&source_param, &Real::one(), policy),
        };
        match (endpoint, outside) {
            (CurveStringEndpoint2::Start, Some(Ordering::Less))
            | (CurveStringEndpoint2::End, Some(Ordering::Greater)) => {}
            (_, Some(_)) => return Ok(Classification::Uncertain(UncertaintyReason::Boundary)),
            (_, None) => return Ok(Classification::Uncertain(UncertaintyReason::Ordering)),
        }

        let output_segment = match endpoint {
            CurveStringEndpoint2::Start => LineSeg2::try_new(target_point, line.end().clone())?,
            CurveStringEndpoint2::End => LineSeg2::try_new(line.start().clone(), target_point)?,
        };
        let mut segments = self.segments.clone();
        segments[source_segment_index] = Segment2::Line(output_segment);
        CurveString2::try_new(segments).map(Classification::Decided)
    }

    fn extend_arc_endpoint_segment_to_point(
        &self,
        endpoint: CurveStringEndpoint2,
        source_segment_index: usize,
        arc: &CircularArc2,
        target_point: Point2,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<CurveString2>> {
        let radius_delta = target_point.distance_squared(arc.center()) - arc.radius_squared();
        match is_zero(&radius_delta, policy) {
            Some(true) => {}
            Some(false) => return Ok(Classification::Uncertain(UncertaintyReason::Boundary)),
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        }

        match arc.contains_point(&target_point, policy) {
            Classification::Decided(false) => {}
            Classification::Decided(true) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        }

        let extended_arc = match endpoint {
            CurveStringEndpoint2::Start => CircularArc2::new_unchecked_with_radius(
                target_point,
                arc.end().clone(),
                arc.center().clone(),
                arc.radius_squared(),
                arc.is_clockwise(),
                None,
            ),
            CurveStringEndpoint2::End => CircularArc2::new_unchecked_with_radius(
                arc.start().clone(),
                target_point,
                arc.center().clone(),
                arc.radius_squared(),
                arc.is_clockwise(),
                None,
            ),
        };

        let retained_endpoint = match endpoint {
            CurveStringEndpoint2::Start => arc.start(),
            CurveStringEndpoint2::End => arc.end(),
        };
        match extended_arc.contains_point(retained_endpoint, policy) {
            Classification::Decided(true) => {}
            Classification::Decided(false) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            }
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        }

        let representative = match arc.representative_point(policy)? {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        match extended_arc.contains_point(&representative, policy) {
            Classification::Decided(true) => {}
            Classification::Decided(false) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            }
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        }

        let mut segments = self.segments.clone();
        segments[source_segment_index] = Segment2::Arc(extended_arc);
        CurveString2::try_new(segments).map(Classification::Decided)
    }

    pub(crate) fn trim_between_curve_intersection_events(
        &self,
        start_events: Vec<CurveStringIntersection>,
        end_events: Vec<CurveStringIntersection>,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<CurveString2>> {
        let start_extraction = extract_curve_trim_hits(&start_events);
        let end_extraction = extract_curve_trim_hits(&end_events);

        let start_point = match single_curve_trim_hit(&start_extraction) {
            Ok(point) => point,
            Err(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let end_point = match single_curve_trim_hit(&end_extraction) {
            Ok(point) => point,
            Err(reason) => return Ok(Classification::Uncertain(reason)),
        };

        self.trim_between_points(&start_point, &end_point, policy)
    }

    fn trim_between_located_points(
        &self,
        start: LocatedTrimPoint2,
        end: LocatedTrimPoint2,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<CurveString2>> {
        let mut trimmed_segments = Vec::new();
        for source_segment_index in start.trim_point.segment_index..=end.trim_point.segment_index {
            let range_start = if source_segment_index == start.trim_point.segment_index {
                start.trim_point.param.clone()
            } else {
                Real::zero()
            };
            let range_end = if source_segment_index == end.trim_point.segment_index {
                end.trim_point.param.clone()
            } else {
                Real::one()
            };
            let range_start_point = if source_segment_index == start.trim_point.segment_index {
                start.point.clone()
            } else {
                self.segments[source_segment_index].start().clone()
            };
            let range_end_point = if source_segment_index == end.trim_point.segment_index {
                end.point.clone()
            } else {
                self.segments[source_segment_index].end().clone()
            };
            let source_range = ParamRange::new(range_start, range_end);
            match trim_segment_by_point_range(
                &self.segments[source_segment_index],
                &source_range,
                &range_start_point,
                &range_end_point,
                policy,
            )? {
                SegmentTrimMaterialization::Materialized(segment) => {
                    trimmed_segments.push(segment);
                }
                SegmentTrimMaterialization::SkippedEmpty => {}
                SegmentTrimMaterialization::Unresolved(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }

        if trimmed_segments.is_empty() {
            return Err(CurveError::InvalidCurveRange);
        }
        CurveString2::try_new(trimmed_segments).map(Classification::Decided)
    }

    /// Collects all nonempty segment-pair intersections against another curve string.
    ///
    /// Segment axis-aligned bounding boxes are used as a conservative broad
    /// phase before exact segment intersection. A decided box non-overlap skips
    /// the pair; any box uncertainty falls back to exact topology. This keeps
    /// the exact segment relation authoritative while following the
    /// candidate-pruning role used by sweep-line intersection methods such as
    /// sweep-line scheduling.
    pub fn intersect_curve_string(
        &self,
        other: &Self,
        policy: &CurvePolicy,
    ) -> CurveResult<Vec<CurveStringIntersection>> {
        let self_boxes = self
            .segments
            .iter()
            .map(|segment| decided_segment_aabb(segment, policy))
            .collect::<Vec<_>>();
        let other_boxes = other
            .segments
            .iter()
            .map(|segment| decided_segment_aabb(segment, policy))
            .collect::<Vec<_>>();
        intersect_curve_strings_with_cached_aabbs(self, other, &self_boxes, &other_boxes, policy)
    }

    fn endpoint(&self, endpoint: CurveStringEndpoint2) -> CurveResult<&Point2> {
        match endpoint {
            CurveStringEndpoint2::Start => self.start().ok_or(CurveError::EmptyCurveString),
            CurveStringEndpoint2::End => self.end().ok_or(CurveError::EmptyCurveString),
        }
    }

    fn classify_endpoint_pair(
        &self,
        other: &Self,
        first_endpoint: CurveStringEndpoint2,
        second_endpoint: CurveStringEndpoint2,
        policy: &CurvePolicy,
    ) -> CurveResult<(Real, Classification<bool>)> {
        let first_point = self.endpoint(first_endpoint)?;
        let second_point = other.endpoint(second_endpoint)?;
        let distance_squared = first_point.distance_squared(second_point);
        let connection = match is_zero(&distance_squared, policy) {
            Some(is_connected) => Classification::Decided(is_connected),
            None => Classification::Uncertain(UncertaintyReason::RealSign),
        };
        Ok((distance_squared, connection))
    }

    fn endpoint_link_pairs(
        &self,
        other: &Self,
        policy: &CurvePolicy,
    ) -> CurveResult<[CurveStringEndpointPair2; 4]> {
        Ok([
            self.endpoint_pair_for_kind(
                other,
                CurveStringLinkKind2::FirstEndToSecondStart,
                policy,
            )?,
            self.endpoint_pair_for_kind(other, CurveStringLinkKind2::FirstEndToSecondEnd, policy)?,
            self.endpoint_pair_for_kind(
                other,
                CurveStringLinkKind2::FirstStartToSecondStart,
                policy,
            )?,
            self.endpoint_pair_for_kind(
                other,
                CurveStringLinkKind2::FirstStartToSecondEnd,
                policy,
            )?,
        ])
    }

    fn endpoint_pair_for_kind(
        &self,
        other: &Self,
        kind: CurveStringLinkKind2,
        policy: &CurvePolicy,
    ) -> CurveResult<CurveStringEndpointPair2> {
        let (first_endpoint, second_endpoint) = match kind {
            CurveStringLinkKind2::FirstEndToSecondStart => {
                (CurveStringEndpoint2::End, CurveStringEndpoint2::Start)
            }
            CurveStringLinkKind2::FirstEndToSecondEnd => {
                (CurveStringEndpoint2::End, CurveStringEndpoint2::End)
            }
            CurveStringLinkKind2::FirstStartToSecondStart => {
                (CurveStringEndpoint2::Start, CurveStringEndpoint2::Start)
            }
            CurveStringLinkKind2::FirstStartToSecondEnd => {
                (CurveStringEndpoint2::Start, CurveStringEndpoint2::End)
            }
        };
        let (distance_squared, connection) =
            self.classify_endpoint_pair(other, first_endpoint, second_endpoint, policy)?;
        Ok(CurveStringEndpointPair2 {
            kind,
            distance_squared,
            connection,
        })
    }
}

impl CurveStringTrimPoint2 {
    /// Constructs a segment-local trim point.
    pub const fn new(segment_index: usize, param: Real) -> Self {
        Self {
            segment_index,
            param,
        }
    }

    /// Returns the source segment index.
    pub const fn segment_index(&self) -> usize {
        self.segment_index
    }

    /// Returns the local segment parameter.
    pub const fn param(&self) -> &Real {
        &self.param
    }
}

enum SegmentTrimMaterialization {
    Materialized(Segment2),
    SkippedEmpty,
    Unresolved(UncertaintyReason),
}

#[derive(Clone, Debug, PartialEq)]
struct LocatedTrimPoint2 {
    trim_point: CurveStringTrimPoint2,
    point: Point2,
}

struct CurveTrimHitExtraction {
    hits: Vec<Point2>,
    blocker: Option<UncertaintyReason>,
}

#[derive(Clone, Debug, PartialEq)]
struct RegionTrimSplitPoint2 {
    trim_point: CurveStringTrimPoint2,
    point: Point2,
}

#[derive(Clone, Debug, PartialEq)]
struct RegionTrimHit2 {
    source_segment_index: usize,
    point: Point2,
    source_param: Real,
}

struct CurveStringEndpointPair2 {
    kind: CurveStringLinkKind2,
    distance_squared: Real,
    connection: Classification<bool>,
}

fn trim_curve_string_inside_region(
    curve_string: &CurveString2,
    region: &LineArcRegion2,
    policy: &CurvePolicy,
) -> CurveResult<Classification<Vec<CurveString2>>> {
    let mut boundary_hits = Vec::new();
    if let Some(blocker) =
        collect_region_trim_boundary_hits(curve_string, region, policy, &mut boundary_hits)?
    {
        return Ok(Classification::Uncertain(blocker));
    }

    trim_curve_string_inside_region_with_hits(curve_string, boundary_hits, policy, |point| {
        region.classify_point(point, policy)
    })
}

fn trim_curve_string_inside_prepared_region(
    curve_string: &CurveString2,
    region: &RegionQuery2<'_>,
    policy: &CurvePolicy,
) -> CurveResult<Classification<Vec<CurveString2>>> {
    let mut boundary_hits = Vec::new();
    if let Some(blocker) = collect_prepared_region_trim_boundary_hits(
        curve_string,
        region,
        policy,
        &mut boundary_hits,
    )? {
        return Ok(Classification::Uncertain(blocker));
    }

    trim_curve_string_inside_region_with_hits(curve_string, boundary_hits, policy, |point| {
        region.classify_point(point, policy)
    })
}

fn trim_curve_string_inside_region_with_hits(
    curve_string: &CurveString2,
    boundary_hits: Vec<RegionTrimHit2>,
    policy: &CurvePolicy,
    mut classify_point: impl FnMut(&Point2) -> Classification<RegionPointLocation>,
) -> CurveResult<Classification<Vec<CurveString2>>> {
    let mut output_segments: Vec<Vec<Segment2>> = Vec::new();
    let mut current_segments = Vec::new();

    for (source_segment_index, source_segment) in curve_string.segments().iter().enumerate() {
        let split_points = match region_trim_split_points_for_segment(
            source_segment_index,
            source_segment,
            &boundary_hits,
            policy,
        )? {
            Classification::Decided(split_points) => split_points,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };

        for window in split_points.windows(2) {
            let start = &window[0];
            let end = &window[1];
            let source_range =
                ParamRange::new(start.trim_point.param.clone(), end.trim_point.param.clone());
            let fragment = match trim_segment_by_point_range(
                source_segment,
                &source_range,
                &start.point,
                &end.point,
                policy,
            )? {
                SegmentTrimMaterialization::Materialized(fragment) => fragment,
                SegmentTrimMaterialization::SkippedEmpty => continue,
                SegmentTrimMaterialization::Unresolved(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };

            let representative = match fragment.representative_point(policy)? {
                Classification::Decided(point) => point,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };

            let location = match classify_point(&representative) {
                Classification::Decided(location) => location,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };

            match location {
                RegionPointLocation::Inside => {
                    current_segments.push(fragment);
                }
                RegionPointLocation::Outside => {
                    flush_region_trim_chain(&mut output_segments, &mut current_segments);
                }
                RegionPointLocation::Boundary => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                }
            }
        }
    }

    flush_region_trim_chain(&mut output_segments, &mut current_segments);
    let mut curve_strings = Vec::with_capacity(output_segments.len());
    for segments in output_segments {
        curve_strings.push(CurveString2::try_new(segments)?);
    }
    Ok(Classification::Decided(curve_strings))
}

fn collect_region_trim_boundary_hits(
    curve_string: &CurveString2,
    region: &LineArcRegion2,
    policy: &CurvePolicy,
    hits: &mut Vec<RegionTrimHit2>,
) -> CurveResult<Option<UncertaintyReason>> {
    let source_segment_boxes: Vec<_> = curve_string
        .segments()
        .iter()
        .map(|segment| decided_segment_aabb(segment, policy))
        .collect();
    for contour in region.material_contours() {
        if let Some(blocker) = collect_region_trim_contour_hits(
            curve_string,
            &source_segment_boxes,
            contour,
            policy,
            hits,
        )? {
            return Ok(Some(blocker));
        }
    }
    for contour in region.hole_contours() {
        if let Some(blocker) = collect_region_trim_contour_hits(
            curve_string,
            &source_segment_boxes,
            contour,
            policy,
            hits,
        )? {
            return Ok(Some(blocker));
        }
    }
    Ok(None)
}

fn collect_prepared_region_trim_boundary_hits(
    curve_string: &CurveString2,
    region: &RegionQuery2<'_>,
    policy: &CurvePolicy,
    hits: &mut Vec<RegionTrimHit2>,
) -> CurveResult<Option<UncertaintyReason>> {
    let source_segment_boxes: Vec<_> = curve_string
        .segments()
        .iter()
        .map(|segment| decided_segment_aabb(segment, policy))
        .collect();
    for contour in region.prepared_material_contours() {
        if let Some(blocker) = collect_prepared_region_trim_contour_hits(
            curve_string,
            &source_segment_boxes,
            contour,
            policy,
            hits,
        )? {
            return Ok(Some(blocker));
        }
    }
    for contour in region.prepared_hole_contours() {
        if let Some(blocker) = collect_prepared_region_trim_contour_hits(
            curve_string,
            &source_segment_boxes,
            contour,
            policy,
            hits,
        )? {
            return Ok(Some(blocker));
        }
    }
    Ok(None)
}

fn collect_region_trim_contour_hits(
    curve_string: &CurveString2,
    source_segment_boxes: &[Option<crate::Aabb2>],
    contour: &crate::Contour2,
    policy: &CurvePolicy,
    hits: &mut Vec<RegionTrimHit2>,
) -> CurveResult<Option<UncertaintyReason>> {
    let region_segment_boxes: Vec<_> = contour
        .segments()
        .iter()
        .map(|segment| decided_segment_aabb(segment, policy))
        .collect();

    for (source_segment_index, source_segment) in curve_string.segments().iter().enumerate() {
        for (region_segment_index, region_segment) in contour.segments().iter().enumerate() {
            if let (Some(Some(source_box)), Some(Some(region_box))) = (
                source_segment_boxes.get(source_segment_index),
                region_segment_boxes.get(region_segment_index),
            ) && aabbs_decided_disjoint(source_box, region_box, policy)
            {
                continue;
            }

            let relation = source_segment.intersect_segment(region_segment, policy)?;
            if let Some(blocker) = append_region_trim_hits_from_relation(
                hits,
                source_segment_index,
                source_segment,
                relation,
                policy,
            )? {
                return Ok(Some(blocker));
            }
        }
    }
    Ok(None)
}

fn collect_prepared_region_trim_contour_hits(
    curve_string: &CurveString2,
    source_segment_boxes: &[Option<crate::Aabb2>],
    contour: &crate::ContourQuery2<'_>,
    policy: &CurvePolicy,
    hits: &mut Vec<RegionTrimHit2>,
) -> CurveResult<Option<UncertaintyReason>> {
    for (source_segment_index, source_segment) in curve_string.segments().iter().enumerate() {
        for (region_segment_index, region_segment) in
            contour.contour().segments().iter().enumerate()
        {
            if let (Some(Some(source_box)), Some(Some(region_box))) = (
                source_segment_boxes.get(source_segment_index),
                contour.segment_boxes().get(region_segment_index),
            ) && aabbs_decided_disjoint(source_box, region_box, policy)
            {
                continue;
            }

            let relation = source_segment.intersect_segment(region_segment, policy)?;
            if let Some(blocker) = append_region_trim_hits_from_relation(
                hits,
                source_segment_index,
                source_segment,
                relation,
                policy,
            )? {
                return Ok(Some(blocker));
            }
        }
    }
    Ok(None)
}

fn append_region_trim_hits_from_relation(
    hits: &mut Vec<RegionTrimHit2>,
    source_segment_index: usize,
    source_segment: &Segment2,
    relation: SegmentIntersection,
    policy: &CurvePolicy,
) -> CurveResult<Option<UncertaintyReason>> {
    match relation {
        SegmentIntersection::LineLine(LineLineIntersection::None)
        | SegmentIntersection::LineArc {
            result: LineArcIntersection::None,
            ..
        }
        | SegmentIntersection::ArcArc(ArcArcIntersection::None) => Ok(None),
        SegmentIntersection::LineLine(LineLineIntersection::Point { point, a_param, .. }) => {
            push_region_trim_hit(hits, source_segment_index, point, a_param)
        }
        SegmentIntersection::LineArc {
            result: LineArcIntersection::Point(hit),
            order,
        } => match line_arc_region_trim_source_param(source_segment, order, &hit, policy)? {
            Ok(source_param) => {
                push_region_trim_hit(hits, source_segment_index, hit.point, source_param)
            }
            Err(reason) => Ok(Some(reason)),
        },
        SegmentIntersection::ArcArc(ArcArcIntersection::Point(hit)) => {
            match region_trim_source_param(source_segment, &hit.point, policy)? {
                Ok(source_param) => {
                    push_region_trim_hit(hits, source_segment_index, hit.point, source_param)
                }
                Err(reason) => Ok(Some(reason)),
            }
        }
        SegmentIntersection::LineArc {
            result: LineArcIntersection::TwoPoints { first, second },
            order,
        } => {
            let source_param =
                match line_arc_region_trim_source_param(source_segment, order, &first, policy)? {
                    Ok(param) => param,
                    Err(reason) => return Ok(Some(reason)),
                };
            push_region_trim_hit(hits, source_segment_index, first.point, source_param)?;
            let source_param =
                match line_arc_region_trim_source_param(source_segment, order, &second, policy)? {
                    Ok(param) => param,
                    Err(reason) => return Ok(Some(reason)),
                };
            push_region_trim_hit(hits, source_segment_index, second.point, source_param)
        }
        SegmentIntersection::ArcArc(ArcArcIntersection::TwoPoints { first, second }) => {
            let source_param = match region_trim_source_param(source_segment, &first.point, policy)?
            {
                Ok(param) => param,
                Err(reason) => return Ok(Some(reason)),
            };
            push_region_trim_hit(hits, source_segment_index, first.point, source_param)?;
            let source_param =
                match region_trim_source_param(source_segment, &second.point, policy)? {
                    Ok(param) => param,
                    Err(reason) => return Ok(Some(reason)),
                };
            push_region_trim_hit(hits, source_segment_index, second.point, source_param)
        }
        SegmentIntersection::LineLine(LineLineIntersection::Overlap { .. })
        | SegmentIntersection::ArcArc(ArcArcIntersection::Overlap { .. }) => {
            Ok(Some(UncertaintyReason::Unsupported))
        }
        SegmentIntersection::LineLine(LineLineIntersection::Uncertain { reason })
        | SegmentIntersection::LineArc {
            result: LineArcIntersection::Uncertain { reason },
            ..
        }
        | SegmentIntersection::ArcArc(ArcArcIntersection::Uncertain { reason }) => Ok(Some(reason)),
    }
}

fn push_region_trim_hit(
    hits: &mut Vec<RegionTrimHit2>,
    source_segment_index: usize,
    point: Point2,
    source_param: Real,
) -> CurveResult<Option<UncertaintyReason>> {
    hits.push(RegionTrimHit2 {
        source_segment_index,
        point,
        source_param,
    });
    Ok(None)
}

fn line_arc_region_trim_source_param(
    source_segment: &Segment2,
    order: LineArcOrder,
    hit: &crate::LineArcIntersectionPoint,
    policy: &CurvePolicy,
) -> CurveResult<Result<Real, UncertaintyReason>> {
    match order {
        LineArcOrder::LineThenArc => Ok(Ok(hit.line_param.clone())),
        LineArcOrder::ArcThenLine => region_trim_source_param(source_segment, &hit.point, policy),
    }
}

fn region_trim_source_param(
    source_segment: &Segment2,
    point: &Point2,
    policy: &CurvePolicy,
) -> CurveResult<Result<Real, UncertaintyReason>> {
    match segment_point_parameter(source_segment, point, policy)? {
        Classification::Decided(param) => Ok(Ok(param)),
        Classification::Uncertain(reason) => Ok(Err(reason)),
    }
}

fn region_trim_split_points_for_segment(
    source_segment_index: usize,
    source_segment: &Segment2,
    hits: &[RegionTrimHit2],
    policy: &CurvePolicy,
) -> CurveResult<Classification<Vec<RegionTrimSplitPoint2>>> {
    let mut split_points = vec![RegionTrimSplitPoint2 {
        trim_point: CurveStringTrimPoint2::new(source_segment_index, Real::zero()),
        point: source_segment.start().clone(),
    }];

    for hit in hits
        .iter()
        .filter(|hit| hit.source_segment_index == source_segment_index)
    {
        match insert_region_trim_split_point(
            &mut split_points,
            RegionTrimSplitPoint2 {
                trim_point: CurveStringTrimPoint2::new(
                    source_segment_index,
                    hit.source_param.clone(),
                ),
                point: hit.point.clone(),
            },
            policy,
        ) {
            Classification::Decided(()) => {}
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        }
    }

    match insert_region_trim_split_point(
        &mut split_points,
        RegionTrimSplitPoint2 {
            trim_point: CurveStringTrimPoint2::new(source_segment_index, Real::one()),
            point: source_segment.end().clone(),
        },
        policy,
    ) {
        Classification::Decided(()) => Ok(Classification::Decided(split_points)),
        Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
    }
}

fn insert_region_trim_split_point(
    split_points: &mut Vec<RegionTrimSplitPoint2>,
    point: RegionTrimSplitPoint2,
    policy: &CurvePolicy,
) -> Classification<()> {
    for index in 0..split_points.len() {
        let ordering = match compare_reals(
            point.trim_point.param(),
            split_points[index].trim_point.param(),
            policy,
        ) {
            Some(ordering) => ordering,
            None => return Classification::Uncertain(UncertaintyReason::Ordering),
        };
        match ordering {
            Ordering::Less => {
                split_points.insert(index, point);
                return Classification::Decided(());
            }
            Ordering::Equal => {
                return match is_zero(
                    &point.point.distance_squared(&split_points[index].point),
                    policy,
                ) {
                    Some(true) => Classification::Decided(()),
                    Some(false) => Classification::Uncertain(UncertaintyReason::Boundary),
                    None => Classification::Uncertain(UncertaintyReason::RealSign),
                };
            }
            Ordering::Greater => {}
        }
    }
    split_points.push(point);
    Classification::Decided(())
}

fn flush_region_trim_chain(
    output_segments: &mut Vec<Vec<Segment2>>,
    current_segments: &mut Vec<Segment2>,
) {
    if !current_segments.is_empty() {
        output_segments.push(std::mem::take(current_segments));
    }
}

fn extract_curve_trim_hits(events: &[CurveStringIntersection]) -> CurveTrimHitExtraction {
    let mut hits = Vec::new();
    let mut blocker = None;
    for event in events {
        match &event.relation {
            SegmentIntersection::LineLine(LineLineIntersection::None)
            | SegmentIntersection::LineArc {
                result: LineArcIntersection::None,
                ..
            }
            | SegmentIntersection::ArcArc(ArcArcIntersection::None) => {}
            SegmentIntersection::LineLine(LineLineIntersection::Point { point, .. }) => {
                hits.push(point.clone());
            }
            SegmentIntersection::LineArc {
                result: LineArcIntersection::Point(hit),
                ..
            } => {
                hits.push(hit.point.clone());
            }
            SegmentIntersection::ArcArc(ArcArcIntersection::Point(hit)) => {
                hits.push(hit.point.clone());
            }
            SegmentIntersection::LineArc {
                result: LineArcIntersection::TwoPoints { first, second },
                ..
            } => {
                hits.push(first.point.clone());
                hits.push(second.point.clone());
            }
            SegmentIntersection::ArcArc(ArcArcIntersection::TwoPoints { first, second }) => {
                hits.push(first.point.clone());
                hits.push(second.point.clone());
            }
            SegmentIntersection::LineLine(LineLineIntersection::Overlap { .. })
            | SegmentIntersection::ArcArc(ArcArcIntersection::Overlap { .. }) => {
                blocker = Some(UncertaintyReason::Unsupported);
            }
            SegmentIntersection::LineLine(LineLineIntersection::Uncertain { reason })
            | SegmentIntersection::LineArc {
                result: LineArcIntersection::Uncertain { reason },
                ..
            }
            | SegmentIntersection::ArcArc(ArcArcIntersection::Uncertain { reason }) => {
                blocker = Some(*reason);
            }
        }
    }
    CurveTrimHitExtraction { hits, blocker }
}

fn single_curve_trim_hit(extraction: &CurveTrimHitExtraction) -> Result<Point2, UncertaintyReason> {
    if let Some(blocker) = extraction.blocker {
        return Err(blocker);
    }
    match extraction.hits.as_slice() {
        [point] => Ok(point.clone()),
        _ => Err(UncertaintyReason::Boundary),
    }
}

pub(crate) fn merge_adjacent_line_segments(
    current: &Segment2,
    next: &Segment2,
    policy: &CurvePolicy,
) -> CurveResult<Classification<Option<LineSeg2>>> {
    let (Segment2::Line(current), Segment2::Line(next)) = (current, next) else {
        return Ok(Classification::Decided(None));
    };

    match current.classify_point(next.end(), policy) {
        Classification::Decided(LineSide::On) => {}
        Classification::Decided(_) => return Ok(Classification::Decided(None)),
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    }

    let (current_dx, current_dy) = current.delta();
    let (next_dx, next_dy) = next.delta();
    let dot = (&current_dx * &next_dx) + (&current_dy * &next_dy);
    match real_sign(&dot, policy) {
        Some(RealSign::Positive) => Ok(Classification::Decided(Some(LineSeg2::try_new(
            current.start().clone(),
            next.end().clone(),
        )?))),
        Some(RealSign::Zero | RealSign::Negative) => Ok(Classification::Decided(None)),
        None => Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
    }
}

fn validate_trim_point(
    curve_string: &CurveString2,
    point: &CurveStringTrimPoint2,
    policy: &CurvePolicy,
) -> CurveResult<()> {
    if point.segment_index >= curve_string.len() {
        return Err(CurveError::InvalidCurveRange);
    }
    match in_closed_unit_interval(&point.param, policy) {
        Some(true) => Ok(()),
        Some(false) => Err(CurveError::InvalidCurveParameter),
        None => Ok(()),
    }
}

fn compare_trim_points(
    start: &CurveStringTrimPoint2,
    end: &CurveStringTrimPoint2,
    policy: &CurvePolicy,
) -> Option<Ordering> {
    match start.segment_index.cmp(&end.segment_index) {
        Ordering::Less => Some(Ordering::Less),
        Ordering::Greater => Some(Ordering::Greater),
        Ordering::Equal => compare_reals(&start.param, &end.param, policy),
    }
}

fn trim_segment_by_range(
    source_segment: &Segment2,
    source_range: &ParamRange,
    policy: &CurvePolicy,
) -> CurveResult<SegmentTrimMaterialization> {
    let ordering = match compare_reals(source_range.start(), source_range.end(), policy) {
        Some(ordering) => ordering,
        None => {
            return Ok(SegmentTrimMaterialization::Unresolved(
                UncertaintyReason::Ordering,
            ));
        }
    };
    match ordering {
        Ordering::Greater => return Err(CurveError::InvalidCurveRange),
        Ordering::Equal => return Ok(SegmentTrimMaterialization::SkippedEmpty),
        Ordering::Less => {}
    }

    let is_full_range = trim_range_is_full(source_range, policy);
    match is_full_range {
        Some(true) => Ok(SegmentTrimMaterialization::Materialized(
            source_segment.clone(),
        )),
        Some(false) => match source_segment {
            Segment2::Line(line) => trim_line_segment_by_range(line, source_range),
            Segment2::Arc(arc) => trim_arc_segment_by_range(arc, source_range, policy),
        },
        None => Ok(SegmentTrimMaterialization::Unresolved(
            UncertaintyReason::Ordering,
        )),
    }
}

fn trim_line_segment_by_range(
    line: &LineSeg2,
    source_range: &ParamRange,
) -> CurveResult<SegmentTrimMaterialization> {
    let start = line.point_at(source_range.start().clone());
    let end = line.point_at(source_range.end().clone());
    LineSeg2::try_new(start, end)
        .map(Segment2::Line)
        .map(SegmentTrimMaterialization::Materialized)
}

fn trim_arc_segment_by_range(
    arc: &CircularArc2,
    source_range: &ParamRange,
    policy: &CurvePolicy,
) -> CurveResult<SegmentTrimMaterialization> {
    let start = match arc.point_at_sweep_fraction(source_range.start(), policy)? {
        Classification::Decided(point) => point,
        Classification::Uncertain(reason) => {
            return Ok(SegmentTrimMaterialization::Unresolved(reason));
        }
    };
    let end = match arc.point_at_sweep_fraction(source_range.end(), policy)? {
        Classification::Decided(point) => point,
        Classification::Uncertain(reason) => {
            return Ok(SegmentTrimMaterialization::Unresolved(reason));
        }
    };
    match arc.fragment_between_sweep_range(start, end, source_range, policy)? {
        Classification::Decided(fragment) => Ok(SegmentTrimMaterialization::Materialized(
            Segment2::Arc(fragment),
        )),
        Classification::Uncertain(reason) => Ok(SegmentTrimMaterialization::Unresolved(reason)),
    }
}

fn trim_segment_by_point_range(
    source_segment: &Segment2,
    source_range: &ParamRange,
    start_point: &Point2,
    end_point: &Point2,
    policy: &CurvePolicy,
) -> CurveResult<SegmentTrimMaterialization> {
    let ordering = match compare_reals(source_range.start(), source_range.end(), policy) {
        Some(ordering) => ordering,
        None => {
            return Ok(SegmentTrimMaterialization::Unresolved(
                UncertaintyReason::Ordering,
            ));
        }
    };
    match ordering {
        Ordering::Greater => return Err(CurveError::InvalidCurveRange),
        Ordering::Equal => return Ok(SegmentTrimMaterialization::SkippedEmpty),
        Ordering::Less => {}
    }

    match source_segment {
        Segment2::Line(_) => LineSeg2::try_new(start_point.clone(), end_point.clone())
            .map(Segment2::Line)
            .map(SegmentTrimMaterialization::Materialized),
        Segment2::Arc(arc) => {
            trim_arc_segment_by_point_range(arc, source_range, start_point, end_point, policy)
        }
    }
}

fn trim_arc_segment_by_point_range(
    source_arc: &CircularArc2,
    source_range: &ParamRange,
    start_point: &Point2,
    end_point: &Point2,
    policy: &CurvePolicy,
) -> CurveResult<SegmentTrimMaterialization> {
    match (
        source_arc.contains_point(start_point, policy),
        source_arc.contains_point(end_point, policy),
    ) {
        (Classification::Decided(true), Classification::Decided(true)) => {}
        (Classification::Decided(false), _) | (_, Classification::Decided(false)) => {
            return Err(CurveError::InvalidCurveRange);
        }
        (Classification::Uncertain(reason), _) | (_, Classification::Uncertain(reason)) => {
            return Ok(SegmentTrimMaterialization::Unresolved(reason));
        }
    }

    let distance = start_point.distance_squared(end_point);
    match is_zero(&distance, policy) {
        Some(true) => Ok(SegmentTrimMaterialization::SkippedEmpty),
        Some(false) => match source_arc.fragment_between_sweep_range(
            start_point.clone(),
            end_point.clone(),
            source_range,
            policy,
        )? {
            Classification::Decided(fragment) => Ok(SegmentTrimMaterialization::Materialized(
                Segment2::Arc(fragment),
            )),
            Classification::Uncertain(reason) => Ok(SegmentTrimMaterialization::Unresolved(reason)),
        },
        None => Ok(SegmentTrimMaterialization::Unresolved(
            UncertaintyReason::RealSign,
        )),
    }
}

fn trim_range_is_full(range: &ParamRange, policy: &CurvePolicy) -> Option<bool> {
    let start_is_zero = compare_reals(range.start(), &Real::zero(), policy)? == Ordering::Equal;
    let end_is_one = compare_reals(range.end(), &Real::one(), policy)? == Ordering::Equal;
    Some(start_is_zero && end_is_one)
}

fn locate_trim_point(
    curve_string: &CurveString2,
    point: &Point2,
    policy: &CurvePolicy,
) -> CurveResult<Classification<LocatedTrimPoint2>> {
    let mut located = Vec::new();
    for (segment_index, segment) in curve_string.segments().iter().enumerate() {
        match segment.contains_point(point, policy) {
            Classification::Decided(true) => {
                let param = match segment_point_parameter(segment, point, policy)? {
                    Classification::Decided(param) => param,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                located.push(LocatedTrimPoint2 {
                    trim_point: CurveStringTrimPoint2::new(segment_index, param),
                    point: point.clone(),
                });
            }
            Classification::Decided(false) => {}
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        }
    }

    match canonical_located_trim_point(located, policy) {
        Some(point) => Ok(Classification::Decided(point)),
        None => Ok(Classification::Uncertain(UncertaintyReason::Boundary)),
    }
}

fn canonical_located_trim_point(
    mut located: Vec<LocatedTrimPoint2>,
    policy: &CurvePolicy,
) -> Option<LocatedTrimPoint2> {
    match located.len() {
        0 => None,
        1 => located.pop(),
        _ => {
            located.sort_by(|left, right| {
                left.trim_point
                    .segment_index
                    .cmp(&right.trim_point.segment_index)
                    .then_with(|| {
                        compare_reals(&left.trim_point.param, &right.trim_point.param, policy)
                            .unwrap_or(Ordering::Equal)
                    })
            });
            if located
                .windows(2)
                .all(|window| adjacent_vertex_duplicate(&window[0], &window[1], policy))
            {
                located.pop()
            } else {
                None
            }
        }
    }
}

fn adjacent_vertex_duplicate(
    left: &LocatedTrimPoint2,
    right: &LocatedTrimPoint2,
    policy: &CurvePolicy,
) -> bool {
    if left.trim_point.segment_index + 1 != right.trim_point.segment_index {
        return false;
    }
    let left_at_end = compare_reals(&left.trim_point.param, &Real::one(), policy);
    let right_at_start = compare_reals(&right.trim_point.param, &Real::zero(), policy);
    left_at_end == Some(Ordering::Equal) && right_at_start == Some(Ordering::Equal)
}

fn segment_point_parameter(
    segment: &Segment2,
    point: &Point2,
    policy: &CurvePolicy,
) -> CurveResult<Classification<Real>> {
    match segment {
        Segment2::Line(line) => line_point_parameter(line, point, policy),
        Segment2::Arc(arc) => arc_sweep_parameter(arc, point, policy),
    }
}

fn segment_point_at_trim_parameter(
    segment: &Segment2,
    parameter: &Real,
    policy: &CurvePolicy,
) -> CurveResult<Classification<Point2>> {
    match segment {
        Segment2::Line(line) => Ok(Classification::Decided(line.point_at(parameter.clone()))),
        Segment2::Arc(arc) => arc.point_at_sweep_fraction(parameter, policy),
    }
}

fn materialize_strict_native_range(
    source: &Segment2,
    start: &Point2,
    end: &Point2,
    source_range: &ParamRange,
    policy: &CurvePolicy,
) -> CurveResult<Classification<Segment2>> {
    match source {
        Segment2::Line(_) => LineSeg2::try_new(start.clone(), end.clone())
            .map(Segment2::Line)
            .map(Classification::Decided),
        Segment2::Arc(arc) => arc
            .fragment_between_sweep_range(start.clone(), end.clone(), source_range, policy)
            .map(|fragment| fragment.map(Segment2::Arc)),
    }
}

fn segment_chamfer_point_parameter(
    segment: &Segment2,
    point: &Point2,
    policy: &CurvePolicy,
) -> CurveResult<Classification<Real>> {
    match segment.contains_point(point, policy) {
        Classification::Decided(true) => segment_point_parameter(segment, point, policy),
        Classification::Decided(false) => {
            Ok(Classification::Uncertain(UncertaintyReason::Boundary))
        }
        Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
    }
}

fn line_point_parameter(
    line: &LineSeg2,
    point: &Point2,
    policy: &CurvePolicy,
) -> CurveResult<Classification<Real>> {
    let (dx, dy) = line.delta();
    let delta = point.delta_from(line.start());
    match is_zero(&dx, policy) {
        Some(false) => (delta.0 / dx)
            .map(Classification::Decided)
            .map_err(Into::into),
        Some(true) => (delta.1 / dy)
            .map(Classification::Decided)
            .map_err(Into::into),
        None => match is_zero(&dy, policy) {
            Some(false) => (delta.1 / dy)
                .map(Classification::Decided)
                .map_err(Into::into),
            Some(true) => Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
            None => Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        },
    }
}

fn segment_fillet_validation_blocker(
    segment: &Segment2,
    tangent_point: &Point2,
    center: &Point2,
    clockwise: bool,
    policy: &CurvePolicy,
) -> Option<UncertaintyReason> {
    let (source_dx, source_dy) = match segment {
        Segment2::Line(line) => line.delta(),
        Segment2::Arc(arc) => {
            let (radius_dx, radius_dy) = tangent_point.delta_from(arc.center());
            if arc.is_clockwise() {
                (radius_dy, -radius_dx)
            } else {
                (-radius_dy, radius_dx)
            }
        }
    };
    let (radius_dx, radius_dy) = tangent_point.delta_from(center);
    let (fillet_dx, fillet_dy) = if clockwise {
        (radius_dy, -radius_dx)
    } else {
        (-radius_dy, radius_dx)
    };
    let tangent_cross = (&source_dx * &fillet_dy) - (&source_dy * &fillet_dx);
    match is_zero(&tangent_cross, policy) {
        Some(true) => {}
        Some(false) => return Some(UncertaintyReason::Boundary),
        None => return Some(UncertaintyReason::RealSign),
    }

    let direction_dot = (&source_dx * &fillet_dx) + (&source_dy * &fillet_dy);
    match real_sign(&direction_dot, policy) {
        Some(RealSign::Positive) => None,
        Some(RealSign::Zero | RealSign::Negative) => Some(UncertaintyReason::Boundary),
        None => Some(UncertaintyReason::RealSign),
    }
}

fn arc_sweep_parameter(
    arc: &CircularArc2,
    point: &Point2,
    policy: &CurvePolicy,
) -> CurveResult<Classification<Real>> {
    arc.sweep_fraction_for_incident_point(point, policy)
}

impl CurveStringIntersection {
    /// Returns the segment index in the first curve string.
    pub const fn a_segment_index(&self) -> usize {
        self.a_segment_index
    }

    /// Returns the segment index in the second curve string.
    pub const fn b_segment_index(&self) -> usize {
        self.b_segment_index
    }

    /// Returns the primitive family of the first source segment.
    pub const fn a_segment_kind(&self) -> SegmentKind {
        self.a_segment_kind
    }

    /// Returns the primitive family of the second source segment.
    pub const fn b_segment_kind(&self) -> SegmentKind {
        self.b_segment_kind
    }

    /// Returns the exact start point of the first source segment.
    pub const fn a_segment_start_point(&self) -> &Point2 {
        &self.a_segment_start_point
    }

    /// Returns the exact end point of the first source segment.
    pub const fn a_segment_end_point(&self) -> &Point2 {
        &self.a_segment_end_point
    }

    /// Returns the exact start point of the second source segment.
    pub const fn b_segment_start_point(&self) -> &Point2 {
        &self.b_segment_start_point
    }

    /// Returns the exact end point of the second source segment.
    pub const fn b_segment_end_point(&self) -> &Point2 {
        &self.b_segment_end_point
    }

    /// Returns the exact segment relation retained for this pair.
    pub const fn relation(&self) -> &SegmentIntersection {
        &self.relation
    }
}

fn linked_curve_string(
    first: &CurveString2,
    second: &CurveString2,
    kind: CurveStringLinkKind2,
) -> CurveResult<CurveString2> {
    let mut segments = Vec::with_capacity(first.len() + second.len());
    match kind {
        CurveStringLinkKind2::FirstEndToSecondStart => {
            segments.extend(first.segments().iter().cloned());
            segments.extend(second.segments().iter().cloned());
        }
        CurveStringLinkKind2::FirstEndToSecondEnd => {
            segments.extend(first.segments().iter().cloned());
            segments.extend(reversed_segments(second.segments()));
        }
        CurveStringLinkKind2::FirstStartToSecondStart => {
            segments.extend(reversed_segments(first.segments()));
            segments.extend(second.segments().iter().cloned());
        }
        CurveStringLinkKind2::FirstStartToSecondEnd => {
            segments.extend(second.segments().iter().cloned());
            segments.extend(first.segments().iter().cloned());
        }
    }

    CurveString2::try_new(segments)
}

fn connected_curve_string(
    first: &CurveString2,
    second: &CurveString2,
    kind: CurveStringLinkKind2,
) -> CurveResult<(CurveString2, usize)> {
    let mut segments = Vec::with_capacity(first.len() + 1 + second.len());
    let (connector_start, connector_end, connector_segment_index) = match kind {
        CurveStringLinkKind2::FirstEndToSecondStart => {
            segments.extend(first.segments().iter().cloned());
            let connector_segment_index = segments.len();
            let connector_start = first.end().ok_or(CurveError::EmptyCurveString)?.clone();
            let connector_end = second.start().ok_or(CurveError::EmptyCurveString)?.clone();
            (connector_start, connector_end, connector_segment_index)
        }
        CurveStringLinkKind2::FirstEndToSecondEnd => {
            segments.extend(first.segments().iter().cloned());
            let connector_segment_index = segments.len();
            let connector_start = first.end().ok_or(CurveError::EmptyCurveString)?.clone();
            let connector_end = second.end().ok_or(CurveError::EmptyCurveString)?.clone();
            (connector_start, connector_end, connector_segment_index)
        }
        CurveStringLinkKind2::FirstStartToSecondStart => {
            segments.extend(reversed_segments(first.segments()));
            let connector_segment_index = segments.len();
            let connector_start = first.start().ok_or(CurveError::EmptyCurveString)?.clone();
            let connector_end = second.start().ok_or(CurveError::EmptyCurveString)?.clone();
            (connector_start, connector_end, connector_segment_index)
        }
        CurveStringLinkKind2::FirstStartToSecondEnd => {
            segments.extend(second.segments().iter().cloned());
            let connector_segment_index = segments.len();
            let connector_start = second.end().ok_or(CurveError::EmptyCurveString)?.clone();
            let connector_end = first.start().ok_or(CurveError::EmptyCurveString)?.clone();
            (connector_start, connector_end, connector_segment_index)
        }
    };

    segments.push(Segment2::Line(LineSeg2::try_new(
        connector_start,
        connector_end,
    )?));
    match kind {
        CurveStringLinkKind2::FirstEndToSecondStart => {
            segments.extend(second.segments().iter().cloned());
        }
        CurveStringLinkKind2::FirstEndToSecondEnd => {
            segments.extend(reversed_segments(second.segments()));
        }
        CurveStringLinkKind2::FirstStartToSecondStart => {
            segments.extend(second.segments().iter().cloned());
        }
        CurveStringLinkKind2::FirstStartToSecondEnd => {
            segments.extend(first.segments().iter().cloned());
        }
    }

    CurveString2::try_new(segments).map(|curve_string| (curve_string, connector_segment_index))
}

fn reversed_segments(segments: &[Segment2]) -> Vec<Segment2> {
    segments
        .iter()
        .rev()
        .map(Segment2::reversed)
        .collect::<Vec<_>>()
}

pub(crate) fn intersect_curve_strings_with_cached_aabbs(
    first: &CurveString2,
    second: &CurveString2,
    first_segment_boxes: &[Option<Aabb2>],
    second_segment_boxes: &[Option<Aabb2>],
    policy: &CurvePolicy,
) -> CurveResult<Vec<CurveStringIntersection>> {
    let mut intersections = Vec::new();
    let x_overlap_schedule =
        curve_string_x_overlap_schedule(first_segment_boxes, second_segment_boxes);

    for (a_segment_index, a_segment) in first.segments.iter().enumerate() {
        for b_segment_index in x_overlap_schedule.as_ref().map_or_else(
            || CurveStringXOverlapCandidates::All(0..second.segments.len()),
            |schedule| schedule.candidates_for(a_segment_index),
        ) {
            let b_segment = &second.segments[b_segment_index];
            if let (Some(Some(a_box)), Some(Some(b_box))) = (
                first_segment_boxes.get(a_segment_index),
                second_segment_boxes.get(b_segment_index),
            ) && aabbs_decided_disjoint(a_box, b_box, policy)
            {
                continue;
            }

            let relation = a_segment.intersect_segment(b_segment, policy)?;
            if !relation.is_none() {
                intersections.push(CurveStringIntersection {
                    a_segment_index,
                    b_segment_index,
                    a_segment_kind: a_segment.structural_facts().kind,
                    b_segment_kind: b_segment.structural_facts().kind,
                    a_segment_start_point: a_segment.start().clone(),
                    a_segment_end_point: a_segment.end().clone(),
                    b_segment_start_point: b_segment.start().clone(),
                    b_segment_end_point: b_segment.end().clone(),
                    relation,
                });
            }
        }
    }

    Ok(intersections)
}

const CURVE_STRING_X_SWEEP_PRECISION: i32 = -32;

#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd)]
enum CurveStringXEventKind {
    FirstStart,
    SecondStart,
    FirstEnd,
    SecondEnd,
}

#[derive(Clone, Debug)]
struct CurveStringXEvent {
    coordinate: Rational,
    kind: CurveStringXEventKind,
    segment_index: usize,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub(crate) struct CurveStringXOverlapSchedule {
    candidates: Vec<Vec<usize>>,
}

impl CurveStringXOverlapSchedule {
    pub(crate) fn candidates_for(&self, first_index: usize) -> CurveStringXOverlapCandidates<'_> {
        CurveStringXOverlapCandidates::Scheduled(self.candidates[first_index].iter().copied())
    }
}

pub(crate) enum CurveStringXOverlapCandidates<'a> {
    All(Range<usize>),
    Scheduled(Copied<slice::Iter<'a, usize>>),
}

impl Iterator for CurveStringXOverlapCandidates<'_> {
    type Item = usize;

    fn next(&mut self) -> Option<Self::Item> {
        match self {
            Self::All(indices) => indices.next(),
            Self::Scheduled(indices) => indices.next(),
        }
    }
}

pub(crate) fn curve_string_x_overlap_schedule(
    first_boxes: &[Option<Aabb2>],
    second_boxes: &[Option<Aabb2>],
) -> Option<CurveStringXOverlapSchedule> {
    const MINIMUM_FLAT_PAIR_COUNT: usize = 4_096;
    const MAXIMUM_MATERIALIZED_PAIR_COUNT: usize = 1 << 20;

    let flat_pair_count = first_boxes.len().saturating_mul(second_boxes.len());
    if flat_pair_count < MINIMUM_FLAT_PAIR_COUNT {
        return None;
    }
    let first_boxes = first_boxes
        .iter()
        .map(Option::as_ref)
        .collect::<Option<Vec<_>>>()?;
    let second_boxes = second_boxes
        .iter()
        .map(Option::as_ref)
        .collect::<Option<Vec<_>>>()?;
    if sampled_exact_rational_x_overlap_is_dense(&first_boxes, &second_boxes) {
        return None;
    }
    let mut events = Vec::with_capacity(2 * (first_boxes.len() + second_boxes.len()));

    let mut push_events = |boxes: &[&Aabb2], start_kind, end_kind| -> Option<()> {
        for (segment_index, bbox) in boxes.iter().enumerate() {
            let [minimum, maximum] = conservative_x_interval(bbox)?;
            events.push(CurveStringXEvent {
                coordinate: minimum,
                kind: start_kind,
                segment_index,
            });
            events.push(CurveStringXEvent {
                coordinate: maximum,
                kind: end_kind,
                segment_index,
            });
        }
        Some(())
    };
    push_events(
        &first_boxes,
        CurveStringXEventKind::FirstStart,
        CurveStringXEventKind::FirstEnd,
    )?;
    push_events(
        &second_boxes,
        CurveStringXEventKind::SecondStart,
        CurveStringXEventKind::SecondEnd,
    )?;

    // Starts precede ends at an equal coordinate so endpoint contact remains
    // a candidate. Source indices break remaining ties deterministically.
    events.sort_by(|left, right| {
        left.coordinate
            .partial_cmp(&right.coordinate)
            .expect("rational sweep endpoints are totally ordered")
            .then_with(|| left.kind.cmp(&right.kind))
            .then_with(|| left.segment_index.cmp(&right.segment_index))
    });

    let dense_pair_limit = flat_pair_count / 2;
    let mut active_first_count = 0_usize;
    let mut active_second_count = 0_usize;
    let mut candidate_pair_count = 0_usize;
    for event in &events {
        match event.kind {
            CurveStringXEventKind::FirstStart => {
                candidate_pair_count += active_second_count;
                active_first_count += 1;
            }
            CurveStringXEventKind::SecondStart => {
                candidate_pair_count += active_first_count;
                active_second_count += 1;
            }
            CurveStringXEventKind::FirstEnd => {
                active_first_count -= 1;
            }
            CurveStringXEventKind::SecondEnd => {
                active_second_count -= 1;
            }
        }
        if candidate_pair_count > dense_pair_limit
            || candidate_pair_count > MAXIMUM_MATERIALIZED_PAIR_COUNT
        {
            return None;
        }
    }

    let mut active_first = BTreeSet::new();
    let mut active_second = BTreeSet::new();
    let mut candidates = vec![Vec::new(); first_boxes.len()];
    for event in events {
        match event.kind {
            CurveStringXEventKind::FirstStart => {
                candidates[event.segment_index].extend(active_second.iter().copied());
                active_first.insert(event.segment_index);
            }
            CurveStringXEventKind::SecondStart => {
                for &first_index in &active_first {
                    candidates[first_index].push(event.segment_index);
                }
                active_second.insert(event.segment_index);
            }
            CurveStringXEventKind::FirstEnd => {
                active_first.remove(&event.segment_index);
            }
            CurveStringXEventKind::SecondEnd => {
                active_second.remove(&event.segment_index);
            }
        }
    }

    let mut materialized_candidate_pair_count = 0;
    for row in &mut candidates {
        row.sort_unstable();
        row.dedup();
        materialized_candidate_pair_count += row.len();
    }
    debug_assert_eq!(candidate_pair_count, materialized_candidate_pair_count);
    Some(CurveStringXOverlapSchedule { candidates })
}

fn conservative_x_interval(bbox: &Aabb2) -> Option<[Rational; 2]> {
    Some([
        bbox.min_x().exact_rational_ref().cloned().or_else(|| {
            bbox.min_x()
                .certified_dyadic_interval(CURVE_STRING_X_SWEEP_PRECISION)
                .map(|interval| interval[0].clone())
        })?,
        bbox.max_x().exact_rational_ref().cloned().or_else(|| {
            bbox.max_x()
                .certified_dyadic_interval(CURVE_STRING_X_SWEEP_PRECISION)
                .map(|interval| interval[1].clone())
        })?,
    ])
}

fn sampled_exact_rational_x_overlap_is_dense(first: &[&Aabb2], second: &[&Aabb2]) -> bool {
    const SAMPLE_COUNT: usize = 8;

    fn indices(len: usize) -> Vec<usize> {
        let count = len.min(SAMPLE_COUNT);
        match count {
            0 => Vec::new(),
            1 => vec![0],
            _ => (0..count)
                .map(|index| index * (len - 1) / (count - 1))
                .collect(),
        }
    }

    let first_indices = indices(first.len());
    let second_indices = indices(second.len());
    let sampled_pair_count = first_indices.len() * second_indices.len();
    if sampled_pair_count == 0 {
        return false;
    }
    let mut overlap_count = 0_usize;
    for first_index in first_indices {
        let (Some(first_minimum), Some(first_maximum)) = (
            first[first_index].min_x().exact_rational_ref(),
            first[first_index].max_x().exact_rational_ref(),
        ) else {
            return false;
        };
        for &second_index in &second_indices {
            let (Some(second_minimum), Some(second_maximum)) = (
                second[second_index].min_x().exact_rational_ref(),
                second[second_index].max_x().exact_rational_ref(),
            ) else {
                return false;
            };
            overlap_count +=
                usize::from(first_minimum <= second_maximum && second_minimum <= first_maximum);
        }
    }
    // This sample chooses only between the sweep and the authoritative flat
    // scan. A false dense result can cost an optimization opportunity but
    // cannot reject a geometric candidate or change topology.
    overlap_count * 2 > sampled_pair_count
}
