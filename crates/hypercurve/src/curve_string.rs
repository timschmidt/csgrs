//! Ordered open curve strings.

use std::cmp::Ordering;

use hyperreal::Real;

use crate::bbox::{Aabb2, aabbs_decided_disjoint, decided_segment_aabb};
use crate::classify::{compare_reals, in_closed_unit_interval, is_zero};
use crate::{
    ArcArcIntersection, BulgeVertex2, CircularArc2, Classification, CurveError, CurvePolicy,
    CurveResult, IntersectionKind, LineArcIntersection, LineLineIntersection, LineSeg2, ParamRange,
    Point2, RetainedTopologyStatus, Segment2, SegmentIntersection, UncertaintyReason,
};

/// One segment-pair event between two curve strings.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveStringIntersection {
    /// Segment index in the first curve string.
    pub a_segment_index: usize,
    /// Segment index in the second curve string.
    pub b_segment_index: usize,
    /// Segment relation for this pair.
    pub relation: SegmentIntersection,
}

/// Endpoint selector for open curve-string editing reports.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CurveStringEndpoint2 {
    /// First point of the curve string.
    Start,
    /// Final point of the curve string.
    End,
}

/// Exact endpoint-connectivity status for two curve strings.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CurveStringEndpointConnectionStatus2 {
    /// Endpoints are certified identical and can be consumed as native topology.
    NativeExact,
    /// Endpoints are certified distinct.
    Disconnected,
    /// The active policy could not decide whether the endpoints are identical.
    Unresolved(UncertaintyReason),
}

/// Report for one tested endpoint pair.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveStringEndpointConnectionReport2 {
    first_endpoint: CurveStringEndpoint2,
    second_endpoint: CurveStringEndpoint2,
    distance_squared: crate::Real,
    status: CurveStringEndpointConnectionStatus2,
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

/// Report for an auto-link attempt between two open curve strings.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveStringLinkReport2 {
    kind: CurveStringLinkKind2,
    endpoint_report: CurveStringEndpointConnectionReport2,
    first_segment_count: usize,
    second_segment_count: usize,
    status: RetainedTopologyStatus,
}

/// A linked open curve string with retained endpoint provenance.
#[derive(Clone, Debug, PartialEq)]
pub struct LinkedCurveString2 {
    curve_string: CurveString2,
    report: CurveStringLinkReport2,
}

/// Segment-local retained trim point on an open curve string.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveStringTrimPoint2 {
    segment_index: usize,
    param: Real,
}

/// Report for one source segment range retained by a trim attempt.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveStringTrimSegmentReport2 {
    source_segment_index: usize,
    source_range: ParamRange,
    status: RetainedTopologyStatus,
}

/// Report for an open curve-string trim attempt.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveStringTrimReport2 {
    start: CurveStringTrimPoint2,
    end: CurveStringTrimPoint2,
    source_segment_count: usize,
    segment_reports: Vec<CurveStringTrimSegmentReport2>,
    status: RetainedTopologyStatus,
    blocker: Option<UncertaintyReason>,
}

/// Result of a report-bearing open curve-string trim.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveStringTrimResult2 {
    curve_string: Option<CurveString2>,
    report: CurveStringTrimReport2,
}

/// One point witness where a cutter intersects the trimmed curve string.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveStringCurveTrimHit2 {
    source_segment_index: usize,
    cutter_segment_index: usize,
    point: Point2,
    kind: IntersectionKind,
}

/// Report for a trim whose boundaries come from cutter curve intersections.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveStringCurveTrimReport2 {
    start_hits: Vec<CurveStringCurveTrimHit2>,
    end_hits: Vec<CurveStringCurveTrimHit2>,
    trim_report: Option<CurveStringTrimReport2>,
    status: RetainedTopologyStatus,
    blocker: Option<UncertaintyReason>,
}

/// Result of a report-bearing trim by two cutter curve strings.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveStringCurveTrimResult2 {
    curve_string: Option<CurveString2>,
    report: CurveStringCurveTrimReport2,
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

    /// Reports whether one endpoint pair is exactly connected.
    ///
    /// This is the small provenance unit used by open-path merge/connect/link
    /// editing. It compares endpoint squared distance through the active exact
    /// policy and records the result instead of letting callers introduce a
    /// local snapping tolerance. A disconnected report is still useful evidence:
    /// it says the topology branch was decided exactly, but no connection was
    /// available at this endpoint pair.
    pub fn endpoint_connection_report(
        &self,
        other: &Self,
        first_endpoint: CurveStringEndpoint2,
        second_endpoint: CurveStringEndpoint2,
        policy: &CurvePolicy,
    ) -> CurveResult<CurveStringEndpointConnectionReport2> {
        let first_point = self.endpoint(first_endpoint)?;
        let second_point = other.endpoint(second_endpoint)?;
        let distance_squared = first_point.distance_squared(second_point);
        let status = match is_zero(&distance_squared, policy) {
            Some(true) => CurveStringEndpointConnectionStatus2::NativeExact,
            Some(false) => CurveStringEndpointConnectionStatus2::Disconnected,
            None => CurveStringEndpointConnectionStatus2::Unresolved(UncertaintyReason::RealSign),
        };

        Ok(CurveStringEndpointConnectionReport2 {
            first_endpoint,
            second_endpoint,
            distance_squared,
            status,
        })
    }

    /// Links two open curve strings when exactly one endpoint pair is certified.
    ///
    /// The four endpoint pairings are tested exactly. A result is materialized
    /// only when one and only one pairing is [`NativeExact`]; multiple exact
    /// pairings are ambiguous open-chain topology, and any unresolved pairing
    /// prevents choosing a unique link. Certified disconnected inputs return
    /// `Decided(None)` so higher-level tools can decide whether to create an
    /// explicit connector segment rather than silently snapping.
    pub fn link_connected_endpoints(
        &self,
        other: &Self,
        policy: &CurvePolicy,
    ) -> CurveResult<crate::Classification<Option<LinkedCurveString2>>> {
        let reports = self.endpoint_link_reports(other, policy)?;
        let mut exact = Vec::new();
        let mut unresolved = None;
        for (kind, report) in reports {
            match report.status {
                CurveStringEndpointConnectionStatus2::NativeExact => exact.push((kind, report)),
                CurveStringEndpointConnectionStatus2::Disconnected => {}
                CurveStringEndpointConnectionStatus2::Unresolved(reason) => {
                    unresolved = Some(reason);
                }
            }
        }

        if exact.len() > 1 {
            return Ok(crate::Classification::Uncertain(
                UncertaintyReason::Boundary,
            ));
        }
        if let Some(reason) = unresolved {
            return Ok(crate::Classification::Uncertain(reason));
        }
        let Some((kind, endpoint_report)) = exact.pop() else {
            return Ok(crate::Classification::Decided(None));
        };

        let curve_string = linked_curve_string(self, other, kind)?;
        let report = CurveStringLinkReport2 {
            kind,
            endpoint_report,
            first_segment_count: self.len(),
            second_segment_count: other.len(),
            status: RetainedTopologyStatus::NativeExact,
        };
        Ok(crate::Classification::Decided(Some(LinkedCurveString2 {
            curve_string,
            report,
        })))
    }

    /// Trims this open curve string between two segment-local parameters.
    ///
    /// The result always carries a report. Materialization currently supports
    /// exact line subsegments and whole native segments. Partial arc ranges are
    /// retained as unsupported blockers because the existing arc split
    /// parameter is chord-projection evidence; without a point-bearing split
    /// marker or stronger arc parameter, constructing a new arc endpoint would
    /// cross the exactness boundary. This keeps future point/curve/region trim
    /// paths free to supply certified split points without letting this API
    /// guess.
    pub fn trim_between_parameters(
        &self,
        start: CurveStringTrimPoint2,
        end: CurveStringTrimPoint2,
        policy: &CurvePolicy,
    ) -> CurveResult<CurveStringTrimResult2> {
        validate_trim_point(self, &start, policy)?;
        validate_trim_point(self, &end, policy)?;
        match compare_trim_points(&start, &end, policy) {
            Some(Ordering::Less) => {}
            Some(Ordering::Equal | Ordering::Greater) => return Err(CurveError::InvalidCurveRange),
            None => {
                return Ok(blocked_trim_result(
                    self,
                    start,
                    end,
                    Vec::new(),
                    RetainedTopologyStatus::Unresolved,
                    Some(UncertaintyReason::Ordering),
                ));
            }
        }

        let mut segment_reports = Vec::new();
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
            let segment_report = CurveStringTrimSegmentReport2 {
                source_segment_index,
                source_range: source_range.clone(),
                status: RetainedTopologyStatus::NativeExact,
            };
            match trim_segment_by_range(
                &self.segments[source_segment_index],
                &source_range,
                policy,
            )? {
                SegmentTrimMaterialization::Materialized(segment) => {
                    segment_reports.push(segment_report);
                    trimmed_segments.push(segment);
                }
                SegmentTrimMaterialization::SkippedEmpty => {}
                SegmentTrimMaterialization::Unsupported(reason) => {
                    let mut segment_report = segment_report;
                    segment_report.status = RetainedTopologyStatus::Unsupported;
                    segment_reports.push(segment_report);
                    return Ok(blocked_trim_result(
                        self,
                        start,
                        end,
                        segment_reports,
                        RetainedTopologyStatus::Unsupported,
                        Some(reason),
                    ));
                }
                SegmentTrimMaterialization::Unresolved(reason) => {
                    let mut segment_report = segment_report;
                    segment_report.status = RetainedTopologyStatus::Unresolved;
                    segment_reports.push(segment_report);
                    return Ok(blocked_trim_result(
                        self,
                        start,
                        end,
                        segment_reports,
                        RetainedTopologyStatus::Unresolved,
                        Some(reason),
                    ));
                }
            }
        }

        if trimmed_segments.is_empty() {
            return Err(CurveError::InvalidCurveRange);
        }
        let curve_string = CurveString2::try_new(trimmed_segments)?;
        let report = CurveStringTrimReport2 {
            start,
            end,
            source_segment_count: self.len(),
            segment_reports,
            status: RetainedTopologyStatus::NativeExact,
            blocker: None,
        };
        Ok(CurveStringTrimResult2 {
            curve_string: Some(curve_string),
            report,
        })
    }

    /// Trims this open curve string between two exact points on the path.
    ///
    /// Unlike [`CurveString2::trim_between_parameters`], this point-bearing
    /// path can materialize partial circular arcs because the endpoints are
    /// explicit exact geometry. The source arc's center, radius, and direction
    /// are retained and replayed against point-on-arc predicates before any
    /// fragment is emitted. Repeated non-adjacent point occurrences remain an
    /// explicit boundary blocker instead of choosing a path branch silently.
    pub fn trim_between_points(
        &self,
        start_point: &Point2,
        end_point: &Point2,
        policy: &CurvePolicy,
    ) -> CurveResult<CurveStringTrimResult2> {
        let start = match locate_trim_point(self, start_point, policy)? {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => {
                return Ok(blocked_trim_result(
                    self,
                    CurveStringTrimPoint2::new(0, Real::zero()),
                    CurveStringTrimPoint2::new(0, Real::zero()),
                    Vec::new(),
                    RetainedTopologyStatus::Unresolved,
                    Some(reason),
                ));
            }
        };
        let end = match locate_trim_point(self, end_point, policy)? {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => {
                return Ok(blocked_trim_result(
                    self,
                    start.trim_point.clone(),
                    start.trim_point.clone(),
                    Vec::new(),
                    RetainedTopologyStatus::Unresolved,
                    Some(reason),
                ));
            }
        };

        match compare_trim_points(&start.trim_point, &end.trim_point, policy) {
            Some(Ordering::Less) => {}
            Some(Ordering::Equal | Ordering::Greater) => return Err(CurveError::InvalidCurveRange),
            None => {
                return Ok(blocked_trim_result(
                    self,
                    start.trim_point,
                    end.trim_point,
                    Vec::new(),
                    RetainedTopologyStatus::Unresolved,
                    Some(UncertaintyReason::Ordering),
                ));
            }
        }

        self.trim_between_located_points(start, end, policy)
    }

    /// Trims this open curve string between exact point intersections with two cutters.
    ///
    /// Each cutter must contribute exactly one point witness on this curve
    /// string. No-hit, multiple-hit, overlap, and uncertain cutter relations are
    /// retained in the report as blockers. Successful materialization delegates
    /// to [`CurveString2::trim_between_points`], so line and arc fragments are
    /// emitted only through the same point-bearing exactness checks.
    pub fn trim_between_curve_intersections(
        &self,
        start_cutter: &Self,
        end_cutter: &Self,
        policy: &CurvePolicy,
    ) -> CurveResult<CurveStringCurveTrimResult2> {
        let start_events = self.intersect_curve_string(start_cutter, policy)?;
        let end_events = self.intersect_curve_string(end_cutter, policy)?;
        let start_extraction = extract_curve_trim_hits(&start_events);
        let end_extraction = extract_curve_trim_hits(&end_events);

        let start_hit = match single_curve_trim_hit(&start_extraction) {
            Ok(hit) => hit,
            Err((status, blocker)) => {
                return Ok(blocked_curve_trim_result(
                    start_extraction.hits,
                    end_extraction.hits,
                    None,
                    status,
                    Some(blocker),
                ));
            }
        };
        let end_hit = match single_curve_trim_hit(&end_extraction) {
            Ok(hit) => hit,
            Err((status, blocker)) => {
                return Ok(blocked_curve_trim_result(
                    start_extraction.hits,
                    end_extraction.hits,
                    None,
                    status,
                    Some(blocker),
                ));
            }
        };

        let trim = self.trim_between_points(&start_hit.point, &end_hit.point, policy)?;
        let status = trim.report().status();
        let blocker = trim.report().blocker();
        let curve_string = trim.curve_string().cloned();
        Ok(CurveStringCurveTrimResult2 {
            curve_string,
            report: CurveStringCurveTrimReport2 {
                start_hits: vec![start_hit],
                end_hits: vec![end_hit],
                trim_report: Some(trim.report().clone()),
                status,
                blocker,
            },
        })
    }

    fn trim_between_located_points(
        &self,
        start: LocatedTrimPoint2,
        end: LocatedTrimPoint2,
        policy: &CurvePolicy,
    ) -> CurveResult<CurveStringTrimResult2> {
        let mut segment_reports = Vec::new();
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
            let segment_report = CurveStringTrimSegmentReport2 {
                source_segment_index,
                source_range: source_range.clone(),
                status: RetainedTopologyStatus::NativeExact,
            };
            match trim_segment_by_point_range(
                &self.segments[source_segment_index],
                &source_range,
                &range_start_point,
                &range_end_point,
                policy,
            )? {
                SegmentTrimMaterialization::Materialized(segment) => {
                    segment_reports.push(segment_report);
                    trimmed_segments.push(segment);
                }
                SegmentTrimMaterialization::SkippedEmpty => {}
                SegmentTrimMaterialization::Unsupported(reason) => {
                    let mut segment_report = segment_report;
                    segment_report.status = RetainedTopologyStatus::Unsupported;
                    segment_reports.push(segment_report);
                    return Ok(blocked_trim_result(
                        self,
                        start.trim_point,
                        end.trim_point,
                        segment_reports,
                        RetainedTopologyStatus::Unsupported,
                        Some(reason),
                    ));
                }
                SegmentTrimMaterialization::Unresolved(reason) => {
                    let mut segment_report = segment_report;
                    segment_report.status = RetainedTopologyStatus::Unresolved;
                    segment_reports.push(segment_report);
                    return Ok(blocked_trim_result(
                        self,
                        start.trim_point,
                        end.trim_point,
                        segment_reports,
                        RetainedTopologyStatus::Unresolved,
                        Some(reason),
                    ));
                }
            }
        }

        if trimmed_segments.is_empty() {
            return Err(CurveError::InvalidCurveRange);
        }
        let curve_string = CurveString2::try_new(trimmed_segments)?;
        let report = CurveStringTrimReport2 {
            start: start.trim_point,
            end: end.trim_point,
            source_segment_count: self.len(),
            segment_reports,
            status: RetainedTopologyStatus::NativeExact,
            blocker: None,
        };
        Ok(CurveStringTrimResult2 {
            curve_string: Some(curve_string),
            report,
        })
    }

    /// Collects all nonempty segment-pair intersections against another curve string.
    ///
    /// Segment axis-aligned bounding boxes are used as a conservative broad
    /// phase before exact segment intersection. A decided box non-overlap skips
    /// the pair; any box uncertainty falls back to exact topology. This keeps
    /// the exact segment relation authoritative while following the
    /// candidate-pruning role used by sweep-line intersection methods such as
    /// Bentley and Ottmann, "Algorithms for Reporting and Counting Geometric
    /// Intersections" (1979).
    pub fn intersect_curve_string(
        &self,
        other: &Self,
        policy: &CurvePolicy,
    ) -> CurveResult<Vec<CurveStringIntersection>> {
        let self_boxes: Vec<_> = self
            .segments
            .iter()
            .map(|segment| decided_segment_aabb(segment, policy))
            .collect();
        let other_boxes: Vec<_> = other
            .segments
            .iter()
            .map(|segment| decided_segment_aabb(segment, policy))
            .collect();

        intersect_curve_strings_with_cached_aabbs(self, other, &self_boxes, &other_boxes, policy)
    }

    fn endpoint(&self, endpoint: CurveStringEndpoint2) -> CurveResult<&Point2> {
        match endpoint {
            CurveStringEndpoint2::Start => self.start().ok_or(CurveError::EmptyCurveString),
            CurveStringEndpoint2::End => self.end().ok_or(CurveError::EmptyCurveString),
        }
    }

    fn endpoint_link_reports(
        &self,
        other: &Self,
        policy: &CurvePolicy,
    ) -> CurveResult<[(CurveStringLinkKind2, CurveStringEndpointConnectionReport2); 4]> {
        Ok([
            (
                CurveStringLinkKind2::FirstEndToSecondStart,
                self.endpoint_connection_report(
                    other,
                    CurveStringEndpoint2::End,
                    CurveStringEndpoint2::Start,
                    policy,
                )?,
            ),
            (
                CurveStringLinkKind2::FirstEndToSecondEnd,
                self.endpoint_connection_report(
                    other,
                    CurveStringEndpoint2::End,
                    CurveStringEndpoint2::End,
                    policy,
                )?,
            ),
            (
                CurveStringLinkKind2::FirstStartToSecondStart,
                self.endpoint_connection_report(
                    other,
                    CurveStringEndpoint2::Start,
                    CurveStringEndpoint2::Start,
                    policy,
                )?,
            ),
            (
                CurveStringLinkKind2::FirstStartToSecondEnd,
                self.endpoint_connection_report(
                    other,
                    CurveStringEndpoint2::Start,
                    CurveStringEndpoint2::End,
                    policy,
                )?,
            ),
        ])
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

impl CurveStringTrimSegmentReport2 {
    /// Returns the retained source segment index.
    pub const fn source_segment_index(&self) -> usize {
        self.source_segment_index
    }

    /// Returns the retained source parameter range.
    pub const fn source_range(&self) -> &ParamRange {
        &self.source_range
    }

    /// Returns topology status for this retained source range.
    pub const fn status(&self) -> RetainedTopologyStatus {
        self.status
    }
}

impl CurveStringTrimReport2 {
    /// Returns the requested trim start.
    pub const fn start(&self) -> &CurveStringTrimPoint2 {
        &self.start
    }

    /// Returns the requested trim end.
    pub const fn end(&self) -> &CurveStringTrimPoint2 {
        &self.end
    }

    /// Returns the source curve-string segment count captured by this report.
    pub const fn source_segment_count(&self) -> usize {
        self.source_segment_count
    }

    /// Returns retained source segment ranges considered by this trim.
    pub fn segment_reports(&self) -> &[CurveStringTrimSegmentReport2] {
        &self.segment_reports
    }

    /// Returns the trim materialization status.
    pub const fn status(&self) -> RetainedTopologyStatus {
        self.status
    }

    /// Returns the exact blocker for non-materialized trims.
    pub const fn blocker(&self) -> Option<UncertaintyReason> {
        self.blocker
    }
}

impl CurveStringTrimResult2 {
    /// Returns the materialized native trim, if this trim was supported.
    pub const fn curve_string(&self) -> Option<&CurveString2> {
        self.curve_string.as_ref()
    }

    /// Consumes this result and returns the materialized native trim, if any.
    pub fn into_curve_string(self) -> Option<CurveString2> {
        self.curve_string
    }

    /// Returns the retained trim report.
    pub const fn report(&self) -> &CurveStringTrimReport2 {
        &self.report
    }
}

impl CurveStringCurveTrimHit2 {
    /// Returns the source segment index on the trimmed curve string.
    pub const fn source_segment_index(&self) -> usize {
        self.source_segment_index
    }

    /// Returns the cutter segment index that produced this hit.
    pub const fn cutter_segment_index(&self) -> usize {
        self.cutter_segment_index
    }

    /// Returns the exact intersection point witness.
    pub const fn point(&self) -> &Point2 {
        &self.point
    }

    /// Returns the local intersection kind.
    pub const fn kind(&self) -> IntersectionKind {
        self.kind
    }
}

impl CurveStringCurveTrimReport2 {
    /// Returns retained start-cutter point witnesses.
    pub fn start_hits(&self) -> &[CurveStringCurveTrimHit2] {
        &self.start_hits
    }

    /// Returns retained end-cutter point witnesses.
    pub fn end_hits(&self) -> &[CurveStringCurveTrimHit2] {
        &self.end_hits
    }

    /// Returns the downstream point-bearing trim report, when attempted.
    pub const fn trim_report(&self) -> Option<&CurveStringTrimReport2> {
        self.trim_report.as_ref()
    }

    /// Returns trim-by-curve materialization status.
    pub const fn status(&self) -> RetainedTopologyStatus {
        self.status
    }

    /// Returns the exact blocker for non-materialized trims.
    pub const fn blocker(&self) -> Option<UncertaintyReason> {
        self.blocker
    }
}

impl CurveStringCurveTrimResult2 {
    /// Returns the materialized native trim, if supported.
    pub const fn curve_string(&self) -> Option<&CurveString2> {
        self.curve_string.as_ref()
    }

    /// Consumes this result and returns the materialized native trim, if any.
    pub fn into_curve_string(self) -> Option<CurveString2> {
        self.curve_string
    }

    /// Returns the retained curve-intersection trim report.
    pub const fn report(&self) -> &CurveStringCurveTrimReport2 {
        &self.report
    }
}

enum SegmentTrimMaterialization {
    Materialized(Segment2),
    SkippedEmpty,
    Unsupported(UncertaintyReason),
    Unresolved(UncertaintyReason),
}

#[derive(Clone, Debug, PartialEq)]
struct LocatedTrimPoint2 {
    trim_point: CurveStringTrimPoint2,
    point: Point2,
}

struct CurveTrimHitExtraction {
    hits: Vec<CurveStringCurveTrimHit2>,
    blocker: Option<(RetainedTopologyStatus, UncertaintyReason)>,
}

fn extract_curve_trim_hits(events: &[CurveStringIntersection]) -> CurveTrimHitExtraction {
    let mut hits = Vec::new();
    let mut blocker = None;
    for event in events {
        match &event.relation {
            SegmentIntersection::LineLine(LineLineIntersection::None) => {}
            SegmentIntersection::LineLine(LineLineIntersection::Point { point, kind, .. }) => {
                hits.push(curve_trim_hit(event, point.clone(), *kind));
            }
            SegmentIntersection::LineLine(LineLineIntersection::Overlap { .. }) => {
                blocker = Some((
                    RetainedTopologyStatus::Unsupported,
                    UncertaintyReason::Unsupported,
                ));
            }
            SegmentIntersection::LineLine(LineLineIntersection::Uncertain { reason }) => {
                blocker = Some((RetainedTopologyStatus::Unresolved, *reason));
            }
            SegmentIntersection::LineArc { result, .. } => match result {
                LineArcIntersection::None => {}
                LineArcIntersection::Point(hit) => {
                    hits.push(curve_trim_hit(event, hit.point.clone(), hit.kind));
                }
                LineArcIntersection::TwoPoints { first, second } => {
                    hits.push(curve_trim_hit(event, first.point.clone(), first.kind));
                    hits.push(curve_trim_hit(event, second.point.clone(), second.kind));
                }
                LineArcIntersection::Uncertain { reason } => {
                    blocker = Some((RetainedTopologyStatus::Unresolved, *reason));
                }
            },
            SegmentIntersection::ArcArc(ArcArcIntersection::None) => {}
            SegmentIntersection::ArcArc(ArcArcIntersection::Point(hit)) => {
                hits.push(curve_trim_hit(event, hit.point.clone(), hit.kind));
            }
            SegmentIntersection::ArcArc(ArcArcIntersection::TwoPoints { first, second }) => {
                hits.push(curve_trim_hit(event, first.point.clone(), first.kind));
                hits.push(curve_trim_hit(event, second.point.clone(), second.kind));
            }
            SegmentIntersection::ArcArc(ArcArcIntersection::Overlap { .. }) => {
                blocker = Some((
                    RetainedTopologyStatus::Unsupported,
                    UncertaintyReason::Unsupported,
                ));
            }
            SegmentIntersection::ArcArc(ArcArcIntersection::Uncertain { reason }) => {
                blocker = Some((RetainedTopologyStatus::Unresolved, *reason));
            }
        }
    }

    CurveTrimHitExtraction { hits, blocker }
}

fn curve_trim_hit(
    event: &CurveStringIntersection,
    point: Point2,
    kind: IntersectionKind,
) -> CurveStringCurveTrimHit2 {
    CurveStringCurveTrimHit2 {
        source_segment_index: event.a_segment_index,
        cutter_segment_index: event.b_segment_index,
        point,
        kind,
    }
}

fn single_curve_trim_hit(
    extraction: &CurveTrimHitExtraction,
) -> Result<CurveStringCurveTrimHit2, (RetainedTopologyStatus, UncertaintyReason)> {
    if let Some(blocker) = extraction.blocker {
        return Err(blocker);
    }
    match extraction.hits.len() {
        1 => Ok(extraction.hits[0].clone()),
        _ => Err((
            RetainedTopologyStatus::Unsupported,
            UncertaintyReason::Boundary,
        )),
    }
}

fn blocked_curve_trim_result(
    start_hits: Vec<CurveStringCurveTrimHit2>,
    end_hits: Vec<CurveStringCurveTrimHit2>,
    trim_report: Option<CurveStringTrimReport2>,
    status: RetainedTopologyStatus,
    blocker: Option<UncertaintyReason>,
) -> CurveStringCurveTrimResult2 {
    CurveStringCurveTrimResult2 {
        curve_string: None,
        report: CurveStringCurveTrimReport2 {
            start_hits,
            end_hits,
            trim_report,
            status,
            blocker,
        },
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
            Segment2::Arc(_) => Ok(SegmentTrimMaterialization::Unsupported(
                UncertaintyReason::Unsupported,
            )),
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
        Segment2::Arc(arc) => trim_arc_segment_by_point_range(arc, start_point, end_point, policy),
    }
}

fn trim_arc_segment_by_point_range(
    source_arc: &CircularArc2,
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
        Some(false) => Ok(SegmentTrimMaterialization::Materialized(Segment2::Arc(
            CircularArc2::new_unchecked_with_radius(
                start_point.clone(),
                end_point.clone(),
                source_arc.center().clone(),
                source_arc.radius_squared(),
                source_arc.is_clockwise(),
                None,
            ),
        ))),
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

fn blocked_trim_result(
    curve_string: &CurveString2,
    start: CurveStringTrimPoint2,
    end: CurveStringTrimPoint2,
    segment_reports: Vec<CurveStringTrimSegmentReport2>,
    status: RetainedTopologyStatus,
    blocker: Option<UncertaintyReason>,
) -> CurveStringTrimResult2 {
    CurveStringTrimResult2 {
        curve_string: None,
        report: CurveStringTrimReport2 {
            start,
            end,
            source_segment_count: curve_string.len(),
            segment_reports,
            status,
            blocker,
        },
    }
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
        Segment2::Arc(arc) => arc_chord_parameter(arc, point),
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

fn arc_chord_parameter(arc: &CircularArc2, point: &Point2) -> CurveResult<Classification<Real>> {
    let (dx, dy) = arc.end().delta_from(arc.start());
    let (px, py) = point.delta_from(arc.start());
    let numerator = (&px * &dx) + (&py * &dy);
    let denominator = (&dx * &dx) + (&dy * &dy);
    (numerator / denominator)
        .map(Classification::Decided)
        .map_err(Into::into)
}

impl CurveStringEndpointConnectionReport2 {
    /// Returns the tested endpoint on the first curve string.
    pub const fn first_endpoint(&self) -> CurveStringEndpoint2 {
        self.first_endpoint
    }

    /// Returns the tested endpoint on the second curve string.
    pub const fn second_endpoint(&self) -> CurveStringEndpoint2 {
        self.second_endpoint
    }

    /// Returns exact squared endpoint distance evidence.
    pub const fn distance_squared(&self) -> &crate::Real {
        &self.distance_squared
    }

    /// Returns the exact connectivity status for this endpoint pair.
    pub const fn status(&self) -> CurveStringEndpointConnectionStatus2 {
        self.status
    }

    /// Returns the retained-topology status corresponding to this endpoint test.
    pub const fn topology_status(&self) -> RetainedTopologyStatus {
        match self.status {
            CurveStringEndpointConnectionStatus2::NativeExact => {
                RetainedTopologyStatus::NativeExact
            }
            CurveStringEndpointConnectionStatus2::Disconnected => {
                RetainedTopologyStatus::Unsupported
            }
            CurveStringEndpointConnectionStatus2::Unresolved(_) => {
                RetainedTopologyStatus::Unresolved
            }
        }
    }
}

impl CurveStringLinkReport2 {
    /// Returns the selected link orientation.
    pub const fn kind(&self) -> CurveStringLinkKind2 {
        self.kind
    }

    /// Returns endpoint-pair evidence for the selected link.
    pub const fn endpoint_report(&self) -> &CurveStringEndpointConnectionReport2 {
        &self.endpoint_report
    }

    /// Returns the first input segment count captured by this report.
    pub const fn first_segment_count(&self) -> usize {
        self.first_segment_count
    }

    /// Returns the second input segment count captured by this report.
    pub const fn second_segment_count(&self) -> usize {
        self.second_segment_count
    }

    /// Returns the topology status of the materialized link.
    pub const fn status(&self) -> RetainedTopologyStatus {
        self.status
    }
}

impl LinkedCurveString2 {
    /// Returns the linked curve string.
    pub const fn curve_string(&self) -> &CurveString2 {
        &self.curve_string
    }

    /// Consumes this result and returns the linked curve string.
    pub fn into_curve_string(self) -> CurveString2 {
        self.curve_string
    }

    /// Returns the provenance report for this link.
    pub const fn report(&self) -> &CurveStringLinkReport2 {
        &self.report
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

    for (a_segment_index, a_segment) in first.segments.iter().enumerate() {
        for (b_segment_index, b_segment) in second.segments.iter().enumerate() {
            // This is the same conservative broad-phase used by the public
            // curve-string query. Bentley and Ottmann, "Algorithms for
            // Reporting and Counting Geometric Intersections" (1979), use
            // ordered sweep candidates for asymptotically better pruning; this
            // helper keeps the flat pair scan but lets prepared callers reuse
            // segment boxes across repeated queries.
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
                    relation,
                });
            }
        }
    }

    Ok(intersections)
}
