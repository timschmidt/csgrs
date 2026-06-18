//! Ordered open curve strings.

use std::cmp::Ordering;

use hyperreal::{Real, RealSign};

use crate::bbox::{Aabb2, aabbs_decided_disjoint, decided_segment_aabb};
use crate::classify::{compare_reals, in_closed_unit_interval, is_zero, real_sign};
use crate::{
    ArcArcIntersection, BulgeVertex2, CircularArc2, Classification, CurveError, CurvePolicy,
    CurveResult, IntersectionKind, LineArcIntersection, LineLineIntersection, LineSeg2, LineSide,
    ParamRange, Point2, PreparedRegionView2, Region2, RegionContourRole, RegionPointLocation,
    RetainedTopologyStatus, Segment2, SegmentIntersection, UncertaintyReason,
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

/// Query path used to collect curve-string intersections.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CurveStringIntersectionQueryPath2 {
    /// Intersections were collected with transient broad-phase boxes.
    Direct,
    /// Intersections were collected through caller-supplied prepared views.
    Prepared,
}

/// Report for a curve-string intersection query.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveStringIntersectionReport2 {
    first_segment_count: usize,
    second_segment_count: usize,
    candidate_pair_count: usize,
    skipped_aabb_pair_count: usize,
    tested_pair_count: usize,
    intersection_count: usize,
    query_path: CurveStringIntersectionQueryPath2,
    status: RetainedTopologyStatus,
    blocker: Option<UncertaintyReason>,
}

/// Result of report-bearing curve-string intersection collection.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveStringIntersectionResult2 {
    intersections: Vec<CurveStringIntersection>,
    report: CurveStringIntersectionReport2,
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

/// One ordered-chain link step in a batch open-path link operation.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveStringOrderedLinkStepReport2 {
    accumulated_source_indices: Vec<usize>,
    next_source_index: usize,
    link_report: Option<CurveStringLinkReport2>,
    status: RetainedTopologyStatus,
    blocker: Option<UncertaintyReason>,
}

/// Report for linking an ordered sequence of open curve strings.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveStringOrderedLinkReport2 {
    source_curve_string_count: usize,
    output_segment_count: Option<usize>,
    steps: Vec<CurveStringOrderedLinkStepReport2>,
    status: RetainedTopologyStatus,
    blocker: Option<UncertaintyReason>,
}

/// Result of report-bearing ordered open-chain linking.
#[derive(Clone, Debug, PartialEq)]
pub struct OrderedLinkedCurveString2 {
    curve_string: Option<CurveString2>,
    report: CurveStringOrderedLinkReport2,
}

/// Report for connecting `first.end` to `second.start` with an exact line segment.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveStringConnectReport2 {
    kind: Option<CurveStringLinkKind2>,
    endpoint_report: CurveStringEndpointConnectionReport2,
    first_segment_count: usize,
    second_segment_count: usize,
    connector_segment_index: Option<usize>,
    status: RetainedTopologyStatus,
    blocker: Option<UncertaintyReason>,
}

/// A connected open curve string with retained connector provenance.
#[derive(Clone, Debug, PartialEq)]
pub struct ConnectedCurveString2 {
    curve_string: Option<CurveString2>,
    report: CurveStringConnectReport2,
}

/// One retained source run emitted by a line-merge operation.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveStringLineMergeSpanReport2 {
    source_start_segment_index: usize,
    source_end_segment_index: usize,
    output_segment_index: usize,
    status: RetainedTopologyStatus,
}

/// Report for exact adjacent-line merging on an open curve string.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveStringLineMergeReport2 {
    source_segment_count: usize,
    output_segment_count: Option<usize>,
    spans: Vec<CurveStringLineMergeSpanReport2>,
    status: RetainedTopologyStatus,
    blocker: Option<UncertaintyReason>,
}

/// Result of report-bearing adjacent-line merging.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveStringLineMergeResult2 {
    curve_string: Option<CurveString2>,
    report: CurveStringLineMergeReport2,
}

/// One exact reversed duplicate pair removed from an open curve string.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveStringReversedDuplicatePairReport2 {
    first_source_segment_index: usize,
    second_source_segment_index: usize,
    status: RetainedTopologyStatus,
}

/// Report for exact adjacent reversed-duplicate removal on an open curve string.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveStringDeduplicateReport2 {
    source_segment_count: usize,
    output_segment_count: Option<usize>,
    removed_pairs: Vec<CurveStringReversedDuplicatePairReport2>,
    status: RetainedTopologyStatus,
    blocker: Option<UncertaintyReason>,
}

/// Result of report-bearing exact adjacent reversed-duplicate removal.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveStringDeduplicateResult2 {
    curve_string: Option<CurveString2>,
    report: CurveStringDeduplicateReport2,
}

/// Report for extending one open curve-string endpoint to an exact target point.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveStringExtendReport2 {
    endpoint: CurveStringEndpoint2,
    source_segment_index: usize,
    target_point: Point2,
    source_param: Option<Real>,
    source_segment_count: usize,
    status: RetainedTopologyStatus,
    blocker: Option<UncertaintyReason>,
}

/// Result of a report-bearing open curve-string extension.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveStringExtendResult2 {
    curve_string: Option<CurveString2>,
    report: CurveStringExtendReport2,
}

/// Report for a line-line chamfer at one open curve-string vertex.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveStringChamferReport2 {
    previous_segment_index: usize,
    next_segment_index: usize,
    previous_trim: CurveStringTrimPoint2,
    next_trim: CurveStringTrimPoint2,
    segment_reports: Vec<CurveStringTrimSegmentReport2>,
    chamfer_segment_index: Option<usize>,
    source_segment_count: usize,
    status: RetainedTopologyStatus,
    blocker: Option<UncertaintyReason>,
}

/// Result of a report-bearing line-line chamfer operation.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveStringChamferResult2 {
    curve_string: Option<CurveString2>,
    report: CurveStringChamferReport2,
}

/// Report for a line-line fillet at one open curve-string vertex.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveStringFilletReport2 {
    previous_segment_index: usize,
    next_segment_index: usize,
    previous_trim: CurveStringTrimPoint2,
    next_trim: CurveStringTrimPoint2,
    center: Option<Point2>,
    radius_squared: Option<Real>,
    segment_reports: Vec<CurveStringTrimSegmentReport2>,
    fillet_segment_index: Option<usize>,
    source_segment_count: usize,
    status: RetainedTopologyStatus,
    blocker: Option<UncertaintyReason>,
}

/// Result of a report-bearing line-line fillet operation.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveStringFilletResult2 {
    curve_string: Option<CurveString2>,
    report: CurveStringFilletReport2,
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

/// Intersection query path used to collect curve-trim split evidence.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CurveStringCurveTrimQueryPath2 {
    /// Intersections were collected by preparing transient broad-phase data.
    Direct,
    /// Intersections were collected through caller-supplied prepared views.
    Prepared,
}

/// Report for a trim whose boundaries come from cutter curve intersections.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveStringCurveTrimReport2 {
    start_hits: Vec<CurveStringCurveTrimHit2>,
    end_hits: Vec<CurveStringCurveTrimHit2>,
    start_intersection_report: CurveStringIntersectionReport2,
    end_intersection_report: CurveStringIntersectionReport2,
    trim_report: Option<CurveStringTrimReport2>,
    query_path: CurveStringCurveTrimQueryPath2,
    status: RetainedTopologyStatus,
    blocker: Option<UncertaintyReason>,
}

/// Result of a report-bearing trim by two cutter curve strings.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveStringCurveTrimResult2 {
    curve_string: Option<CurveString2>,
    report: CurveStringCurveTrimReport2,
}

/// One exact boundary hit used while trimming an open curve string by a region.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveStringRegionTrimHit2 {
    source_segment_index: usize,
    region_contour_role: RegionContourRole,
    region_contour_index: usize,
    region_segment_index: usize,
    point: Point2,
    source_param: Real,
    kind: IntersectionKind,
}

/// One retained source interval classified during trim-by-region.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveStringRegionTrimIntervalReport2 {
    source_segment_index: usize,
    source_range: ParamRange,
    representative_point: Option<Point2>,
    location: Option<RegionPointLocation>,
    output_curve_string_index: Option<usize>,
    output_segment_index: Option<usize>,
    status: RetainedTopologyStatus,
    blocker: Option<UncertaintyReason>,
}

/// Query path used to trim an open curve string by a region.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CurveStringRegionTrimQueryPath2 {
    /// Region boundary hits and classifications used transient broad-phase data.
    Direct,
    /// Region boundary hits and classifications reused prepared region caches.
    Prepared,
}

/// Report for retaining portions of an open curve string inside a region.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveStringRegionTrimReport2 {
    source_segment_count: usize,
    region_material_contour_count: usize,
    region_hole_contour_count: usize,
    boundary_hits: Vec<CurveStringRegionTrimHit2>,
    interval_reports: Vec<CurveStringRegionTrimIntervalReport2>,
    output_curve_string_count: Option<usize>,
    query_path: CurveStringRegionTrimQueryPath2,
    status: RetainedTopologyStatus,
    blocker: Option<UncertaintyReason>,
}

/// Result of a report-bearing trim-by-region operation.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveStringRegionTrimResult2 {
    curve_strings: Vec<CurveString2>,
    report: CurveStringRegionTrimReport2,
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

    /// Links an ordered sequence of open curve strings by certified endpoints.
    ///
    /// The caller supplies the intended chain order. Each next curve string is
    /// linked to the accumulated output through
    /// [`CurveString2::link_connected_endpoints`], so every step still requires
    /// one unique exact endpoint pair and keeps the pairwise link report. A
    /// disconnected, ambiguous, or unresolved step stops the batch and returns
    /// an explicit report instead of creating connectors or snapping endpoints.
    pub fn link_ordered_connected_endpoints(
        curve_strings: Vec<Self>,
        policy: &CurvePolicy,
    ) -> CurveResult<OrderedLinkedCurveString2> {
        let source_curve_string_count = curve_strings.len();
        let mut iter = curve_strings.into_iter().enumerate();
        let Some((first_index, mut accumulated)) = iter.next() else {
            return Err(CurveError::EmptyCurveString);
        };
        let mut accumulated_source_indices = vec![first_index];
        let mut steps = Vec::new();

        for (next_source_index, next_curve_string) in iter {
            match accumulated.link_connected_endpoints(&next_curve_string, policy)? {
                Classification::Decided(Some(linked)) => {
                    let link_report = linked.report().clone();
                    let next_accumulated_source_indices = ordered_link_source_indices(
                        &accumulated_source_indices,
                        next_source_index,
                        link_report.kind(),
                    );
                    steps.push(CurveStringOrderedLinkStepReport2 {
                        accumulated_source_indices,
                        next_source_index,
                        link_report: Some(link_report),
                        status: RetainedTopologyStatus::NativeExact,
                        blocker: None,
                    });
                    accumulated = linked.into_curve_string();
                    accumulated_source_indices = next_accumulated_source_indices;
                }
                Classification::Decided(None) => {
                    steps.push(CurveStringOrderedLinkStepReport2 {
                        accumulated_source_indices,
                        next_source_index,
                        link_report: None,
                        status: RetainedTopologyStatus::Unsupported,
                        blocker: Some(UncertaintyReason::Boundary),
                    });
                    return Ok(OrderedLinkedCurveString2 {
                        curve_string: None,
                        report: CurveStringOrderedLinkReport2 {
                            source_curve_string_count,
                            output_segment_count: None,
                            steps,
                            status: RetainedTopologyStatus::Unsupported,
                            blocker: Some(UncertaintyReason::Boundary),
                        },
                    });
                }
                Classification::Uncertain(reason) => {
                    steps.push(CurveStringOrderedLinkStepReport2 {
                        accumulated_source_indices,
                        next_source_index,
                        link_report: None,
                        status: retained_status_for_uncertainty(reason),
                        blocker: Some(reason),
                    });
                    return Ok(OrderedLinkedCurveString2 {
                        curve_string: None,
                        report: CurveStringOrderedLinkReport2 {
                            source_curve_string_count,
                            output_segment_count: None,
                            steps,
                            status: retained_status_for_uncertainty(reason),
                            blocker: Some(reason),
                        },
                    });
                }
            }
        }

        Ok(OrderedLinkedCurveString2 {
            report: CurveStringOrderedLinkReport2 {
                source_curve_string_count,
                output_segment_count: Some(accumulated.len()),
                steps,
                status: RetainedTopologyStatus::NativeExact,
                blocker: None,
            },
            curve_string: Some(accumulated),
        })
    }

    /// Connects `self.end` to `other.start` with an exact line segment.
    ///
    /// This operation is the explicit-connector counterpart to
    /// [`CurveString2::link_connected_endpoints`]. It materializes only when
    /// the endpoints are certified distinct; already-equal endpoints should be
    /// linked instead, and unresolved endpoint equality remains an explicit
    /// blocker. No tolerance snap or hidden zero-length connector is introduced.
    pub fn connect_end_to_start_with_line(
        &self,
        other: &Self,
        policy: &CurvePolicy,
    ) -> CurveResult<ConnectedCurveString2> {
        self.connect_endpoints_with_line(other, CurveStringLinkKind2::FirstEndToSecondStart, policy)
    }

    /// Connects a selected endpoint pair with an exact line segment.
    ///
    /// The selected endpoints are compared exactly first. Equal endpoints are
    /// blocked as a link case, unresolved equality stays unresolved, and only a
    /// certified positive endpoint gap materializes a connector.
    pub fn connect_endpoints_with_line(
        &self,
        other: &Self,
        kind: CurveStringLinkKind2,
        policy: &CurvePolicy,
    ) -> CurveResult<ConnectedCurveString2> {
        let endpoint_report = self.endpoint_connection_report_for_kind(other, kind, policy)?;
        match endpoint_report.status {
            CurveStringEndpointConnectionStatus2::NativeExact => {
                return Ok(blocked_connected_curve_string(
                    self,
                    other,
                    Some(kind),
                    endpoint_report,
                    RetainedTopologyStatus::Unsupported,
                    Some(UncertaintyReason::Boundary),
                ));
            }
            CurveStringEndpointConnectionStatus2::Disconnected => {}
            CurveStringEndpointConnectionStatus2::Unresolved(reason) => {
                return Ok(blocked_connected_curve_string(
                    self,
                    other,
                    Some(kind),
                    endpoint_report,
                    RetainedTopologyStatus::Unresolved,
                    Some(reason),
                ));
            }
        }

        let (curve_string, connector_segment_index) = connected_curve_string(self, other, kind)?;
        let report = CurveStringConnectReport2 {
            kind: Some(kind),
            endpoint_report,
            first_segment_count: self.len(),
            second_segment_count: other.len(),
            connector_segment_index: Some(connector_segment_index),
            status: RetainedTopologyStatus::NativeExact,
            blocker: None,
        };
        Ok(ConnectedCurveString2 {
            curve_string: Some(curve_string),
            report,
        })
    }

    /// Connects the uniquely nearest certified-disconnected endpoint pair.
    ///
    /// All four endpoint pairs are inspected with exact squared-distance
    /// evidence. Existing exact endpoint equality is blocked as a link case;
    /// unresolved endpoint equality or unresolved distance ordering blocks the
    /// auto-choice; equal nearest distances are reported as boundary ambiguity.
    pub fn connect_nearest_endpoints_with_line(
        &self,
        other: &Self,
        policy: &CurvePolicy,
    ) -> CurveResult<ConnectedCurveString2> {
        let reports = self.endpoint_link_reports(other, policy)?;
        let mut candidates = Vec::new();
        for (kind, report) in reports {
            match report.status {
                CurveStringEndpointConnectionStatus2::NativeExact => {
                    return Ok(blocked_connected_curve_string(
                        self,
                        other,
                        Some(kind),
                        report,
                        RetainedTopologyStatus::Unsupported,
                        Some(UncertaintyReason::Boundary),
                    ));
                }
                CurveStringEndpointConnectionStatus2::Disconnected => {
                    candidates.push((kind, report));
                }
                CurveStringEndpointConnectionStatus2::Unresolved(reason) => {
                    return Ok(blocked_connected_curve_string(
                        self,
                        other,
                        Some(kind),
                        report,
                        RetainedTopologyStatus::Unresolved,
                        Some(reason),
                    ));
                }
            }
        }

        let (kind, endpoint_report) = match unique_nearest_endpoint_report(candidates, policy) {
            NearestEndpointChoice::Selected(kind, report) => (kind, report),
            NearestEndpointChoice::Ambiguous(kind, report) => {
                return Ok(blocked_connected_curve_string(
                    self,
                    other,
                    Some(kind),
                    report,
                    RetainedTopologyStatus::Unsupported,
                    Some(UncertaintyReason::Boundary),
                ));
            }
            NearestEndpointChoice::Unresolved(kind, report, reason) => {
                return Ok(blocked_connected_curve_string(
                    self,
                    other,
                    Some(kind),
                    report,
                    RetainedTopologyStatus::Unresolved,
                    Some(reason),
                ));
            }
            NearestEndpointChoice::Empty => return Err(CurveError::EmptyCurveString),
        };
        let (curve_string, connector_segment_index) = connected_curve_string(self, other, kind)?;
        let report = CurveStringConnectReport2 {
            kind: Some(kind),
            endpoint_report,
            first_segment_count: self.len(),
            second_segment_count: other.len(),
            connector_segment_index: Some(connector_segment_index),
            status: RetainedTopologyStatus::NativeExact,
            blocker: None,
        };
        Ok(ConnectedCurveString2 {
            curve_string: Some(curve_string),
            report,
        })
    }

    /// Merges adjacent same-direction line segments when collinearity is certified.
    ///
    /// This is an explicit editing utility, not constructor normalization:
    /// source segment runs are retained in the report, mixed line/arc topology
    /// is preserved, and collinear reversals are not collapsed because they are
    /// real authored backtracking topology. If a line-line pair cannot be
    /// classified under the active policy, the operation returns an unresolved
    /// report instead of guessing a merge boundary.
    pub fn merge_adjacent_collinear_lines(
        &self,
        policy: &CurvePolicy,
    ) -> CurveResult<CurveStringLineMergeResult2> {
        let mut merged_segments = Vec::with_capacity(self.len());
        let mut spans = Vec::new();
        let mut current_segment = self
            .segments
            .first()
            .cloned()
            .ok_or(CurveError::EmptyCurveString)?;
        let mut current_start_index = 0_usize;

        for (next_index, next_segment) in self.segments.iter().enumerate().skip(1) {
            match merge_adjacent_line_segments(&current_segment, next_segment, policy)? {
                Classification::Decided(Some(merged)) => {
                    current_segment = Segment2::Line(merged);
                }
                Classification::Decided(None) => {
                    let output_segment_index = merged_segments.len();
                    merged_segments.push(current_segment);
                    spans.push(CurveStringLineMergeSpanReport2 {
                        source_start_segment_index: current_start_index,
                        source_end_segment_index: next_index - 1,
                        output_segment_index,
                        status: RetainedTopologyStatus::NativeExact,
                    });
                    current_segment = next_segment.clone();
                    current_start_index = next_index;
                }
                Classification::Uncertain(reason) => {
                    return Ok(CurveStringLineMergeResult2 {
                        curve_string: None,
                        report: CurveStringLineMergeReport2 {
                            source_segment_count: self.len(),
                            output_segment_count: None,
                            spans,
                            status: RetainedTopologyStatus::Unresolved,
                            blocker: Some(reason),
                        },
                    });
                }
            }
        }

        let output_segment_index = merged_segments.len();
        merged_segments.push(current_segment);
        spans.push(CurveStringLineMergeSpanReport2 {
            source_start_segment_index: current_start_index,
            source_end_segment_index: self.len() - 1,
            output_segment_index,
            status: RetainedTopologyStatus::NativeExact,
        });

        let curve_string = CurveString2::try_new(merged_segments)?;
        Ok(CurveStringLineMergeResult2 {
            report: CurveStringLineMergeReport2 {
                source_segment_count: self.len(),
                output_segment_count: Some(curve_string.len()),
                spans,
                status: RetainedTopologyStatus::NativeExact,
                blocker: None,
            },
            curve_string: Some(curve_string),
        })
    }

    /// Removes adjacent exact reversed duplicate segment pairs.
    ///
    /// This is a structural de-duplication utility for authored backtracking,
    /// not an overlap resolver: only `segment == next.reversed()` is removed.
    /// Same-support partial overlaps, same-direction repeats, and geometric
    /// coincidences with different segmentation remain intact for the
    /// arrangement pipeline. If every segment cancels, no empty `CurveString2`
    /// is materialized and the report carries an explicit boundary blocker.
    pub fn remove_adjacent_reversed_duplicates(
        &self,
    ) -> CurveResult<CurveStringDeduplicateResult2> {
        let mut retained: Vec<(usize, Segment2)> = Vec::with_capacity(self.len());
        let mut removed_pairs = Vec::new();

        for (source_index, segment) in self.segments.iter().cloned().enumerate() {
            if retained
                .last()
                .is_some_and(|(_, previous)| previous == &segment.reversed())
            {
                let (first_source_segment_index, _) = retained
                    .pop()
                    .expect("retained stack should have a previous segment");
                removed_pairs.push(CurveStringReversedDuplicatePairReport2 {
                    first_source_segment_index,
                    second_source_segment_index: source_index,
                    status: RetainedTopologyStatus::NativeExact,
                });
            } else {
                retained.push((source_index, segment));
            }
        }

        if retained.is_empty() {
            return Ok(CurveStringDeduplicateResult2 {
                curve_string: None,
                report: CurveStringDeduplicateReport2 {
                    source_segment_count: self.len(),
                    output_segment_count: None,
                    removed_pairs,
                    status: RetainedTopologyStatus::Unsupported,
                    blocker: Some(UncertaintyReason::Boundary),
                },
            });
        }

        let segments = retained
            .into_iter()
            .map(|(_, segment)| segment)
            .collect::<Vec<_>>();
        let curve_string = CurveString2::try_new(segments)?;
        Ok(CurveStringDeduplicateResult2 {
            report: CurveStringDeduplicateReport2 {
                source_segment_count: self.len(),
                output_segment_count: Some(curve_string.len()),
                removed_pairs,
                status: RetainedTopologyStatus::NativeExact,
                blocker: None,
            },
            curve_string: Some(curve_string),
        })
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
        let start_events = self.intersect_curve_string_with_report(start_cutter, policy)?;
        let end_events = self.intersect_curve_string_with_report(end_cutter, policy)?;
        self.trim_between_curve_intersection_events(
            start_events,
            end_events,
            CurveStringCurveTrimQueryPath2::Direct,
            policy,
        )
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
        region: &Region2,
        policy: &CurvePolicy,
    ) -> CurveResult<CurveStringRegionTrimResult2> {
        trim_curve_string_inside_region(self, region, policy)
    }

    pub(crate) fn trim_inside_prepared_region(
        &self,
        region: &PreparedRegionView2<'_>,
        policy: &CurvePolicy,
    ) -> CurveResult<CurveStringRegionTrimResult2> {
        trim_curve_string_inside_prepared_region(self, region, policy)
    }

    /// Chamfers one interior line-line vertex by exact segment parameters.
    ///
    /// `vertex_index` identifies the shared vertex between
    /// `segments[vertex_index - 1]` and `segments[vertex_index]`. The previous
    /// line is cut at `previous_param` and the next line is cut at
    /// `next_param`; the two cut points are connected by an exact line segment.
    /// This first chamfer slice supports only strict interior line parameters
    /// so it never deletes a neighboring segment silently.
    pub fn chamfer_line_line_vertex_by_parameters(
        &self,
        vertex_index: usize,
        previous_param: Real,
        next_param: Real,
        policy: &CurvePolicy,
    ) -> CurveResult<CurveStringChamferResult2> {
        if vertex_index == 0 || vertex_index >= self.len() {
            return Err(CurveError::InvalidCurveRange);
        }
        let previous_segment_index = vertex_index - 1;
        let next_segment_index = vertex_index;
        let previous_trim = CurveStringTrimPoint2::new(previous_segment_index, previous_param);
        let next_trim = CurveStringTrimPoint2::new(next_segment_index, next_param);
        validate_trim_point(self, &previous_trim, policy)?;
        validate_trim_point(self, &next_trim, policy)?;

        let (previous_line, next_line) = match (
            &self.segments[previous_segment_index],
            &self.segments[next_segment_index],
        ) {
            (Segment2::Line(previous), Segment2::Line(next)) => (previous, next),
            _ => {
                return Ok(blocked_chamfer_result(
                    self,
                    previous_segment_index,
                    next_segment_index,
                    previous_trim,
                    next_trim,
                    Vec::new(),
                    RetainedTopologyStatus::Unsupported,
                    Some(UncertaintyReason::Unsupported),
                ));
            }
        };

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
                return Ok(blocked_chamfer_result(
                    self,
                    previous_segment_index,
                    next_segment_index,
                    previous_trim,
                    next_trim,
                    Vec::new(),
                    RetainedTopologyStatus::Unsupported,
                    Some(UncertaintyReason::Boundary),
                ));
            }
            _ => {
                return Ok(blocked_chamfer_result(
                    self,
                    previous_segment_index,
                    next_segment_index,
                    previous_trim,
                    next_trim,
                    Vec::new(),
                    RetainedTopologyStatus::Unresolved,
                    Some(UncertaintyReason::Ordering),
                ));
            }
        }

        let previous_cut = previous_line.point_at(previous_trim.param().clone());
        let next_cut = next_line.point_at(next_trim.param().clone());
        let previous_range = ParamRange::new(Real::zero(), previous_trim.param().clone());
        let next_range = ParamRange::new(next_trim.param().clone(), Real::one());
        let previous_segment =
            LineSeg2::try_new(previous_line.start().clone(), previous_cut.clone())?;
        let chamfer_segment = LineSeg2::try_new(previous_cut, next_cut.clone())?;
        let next_segment = LineSeg2::try_new(next_cut, next_line.end().clone())?;

        let mut segments = Vec::with_capacity(self.len() + 1);
        segments.extend(self.segments[..previous_segment_index].iter().cloned());
        segments.push(Segment2::Line(previous_segment));
        let chamfer_segment_index = segments.len();
        segments.push(Segment2::Line(chamfer_segment));
        segments.push(Segment2::Line(next_segment));
        segments.extend(self.segments[next_segment_index + 1..].iter().cloned());
        let curve_string = CurveString2::try_new(segments)?;
        let segment_reports = vec![
            CurveStringTrimSegmentReport2 {
                source_segment_index: previous_segment_index,
                source_range: previous_range,
                status: RetainedTopologyStatus::NativeExact,
            },
            CurveStringTrimSegmentReport2 {
                source_segment_index: next_segment_index,
                source_range: next_range,
                status: RetainedTopologyStatus::NativeExact,
            },
        ];
        Ok(CurveStringChamferResult2 {
            curve_string: Some(curve_string),
            report: CurveStringChamferReport2 {
                previous_segment_index,
                next_segment_index,
                previous_trim,
                next_trim,
                segment_reports,
                chamfer_segment_index: Some(chamfer_segment_index),
                source_segment_count: self.len(),
                status: RetainedTopologyStatus::NativeExact,
                blocker: None,
            },
        })
    }

    /// Chamfers one interior line-line vertex by exact cut points.
    ///
    /// The supplied points are validated against the specific previous and next
    /// finite line segments adjacent to `vertex_index`. Certified point
    /// parameters are then passed to
    /// [`CurveString2::chamfer_line_line_vertex_by_parameters`], so the same
    /// strict interior-parameter and retained-range rules decide whether native
    /// topology may be materialized.
    pub fn chamfer_line_line_vertex_by_points(
        &self,
        vertex_index: usize,
        previous_point: &Point2,
        next_point: &Point2,
        policy: &CurvePolicy,
    ) -> CurveResult<CurveStringChamferResult2> {
        if vertex_index == 0 || vertex_index >= self.len() {
            return Err(CurveError::InvalidCurveRange);
        }
        let previous_segment_index = vertex_index - 1;
        let next_segment_index = vertex_index;
        let previous_zero = CurveStringTrimPoint2::new(previous_segment_index, Real::zero());
        let next_zero = CurveStringTrimPoint2::new(next_segment_index, Real::zero());
        let (previous_line, next_line) = match (
            &self.segments[previous_segment_index],
            &self.segments[next_segment_index],
        ) {
            (Segment2::Line(previous), Segment2::Line(next)) => (previous, next),
            _ => {
                return Ok(blocked_chamfer_result(
                    self,
                    previous_segment_index,
                    next_segment_index,
                    previous_zero,
                    next_zero,
                    Vec::new(),
                    RetainedTopologyStatus::Unsupported,
                    Some(UncertaintyReason::Unsupported),
                ));
            }
        };

        let previous_param =
            match line_chamfer_point_parameter(previous_line, previous_point, policy)? {
                Classification::Decided(param) => param,
                Classification::Uncertain(reason) => {
                    return Ok(blocked_chamfer_result(
                        self,
                        previous_segment_index,
                        next_segment_index,
                        previous_zero,
                        next_zero,
                        Vec::new(),
                        retained_status_for_uncertainty(reason),
                        Some(reason),
                    ));
                }
            };
        let previous_trim =
            CurveStringTrimPoint2::new(previous_segment_index, previous_param.clone());
        let next_param = match line_chamfer_point_parameter(next_line, next_point, policy)? {
            Classification::Decided(param) => param,
            Classification::Uncertain(reason) => {
                return Ok(blocked_chamfer_result(
                    self,
                    previous_segment_index,
                    next_segment_index,
                    previous_trim,
                    next_zero,
                    Vec::new(),
                    retained_status_for_uncertainty(reason),
                    Some(reason),
                ));
            }
        };

        self.chamfer_line_line_vertex_by_parameters(
            vertex_index,
            previous_param,
            next_param,
            policy,
        )
    }

    /// Fillets one interior line-line vertex from exact parameters and center.
    ///
    /// `vertex_index` identifies the shared vertex between
    /// `segments[vertex_index - 1]` and `segments[vertex_index]`. The
    /// parameters identify the tangent points on the previous and next line
    /// segments. The final materialization delegates to
    /// [`CurveString2::fillet_line_line_vertex_by_points`], so the same
    /// nonzero-radius, equal-radius, tangency, orientation, and retained-range
    /// checks decide whether native topology may be emitted.
    pub fn fillet_line_line_vertex_by_parameters(
        &self,
        vertex_index: usize,
        previous_param: Real,
        next_param: Real,
        center: &Point2,
        clockwise: bool,
        policy: &CurvePolicy,
    ) -> CurveResult<CurveStringFilletResult2> {
        if vertex_index == 0 || vertex_index >= self.len() {
            return Err(CurveError::InvalidCurveRange);
        }
        let previous_segment_index = vertex_index - 1;
        let next_segment_index = vertex_index;
        let previous_trim = CurveStringTrimPoint2::new(previous_segment_index, previous_param);
        let next_trim = CurveStringTrimPoint2::new(next_segment_index, next_param);
        validate_trim_point(self, &previous_trim, policy)?;
        validate_trim_point(self, &next_trim, policy)?;

        let (previous_line, next_line) = match (
            &self.segments[previous_segment_index],
            &self.segments[next_segment_index],
        ) {
            (Segment2::Line(previous), Segment2::Line(next)) => (previous, next),
            _ => {
                return Ok(blocked_fillet_result(
                    self,
                    previous_segment_index,
                    next_segment_index,
                    previous_trim,
                    next_trim,
                    Some(center.clone()),
                    None,
                    Vec::new(),
                    RetainedTopologyStatus::Unsupported,
                    Some(UncertaintyReason::Unsupported),
                ));
            }
        };

        let previous_point = previous_line.point_at(previous_trim.param().clone());
        let next_point = next_line.point_at(next_trim.param().clone());
        self.fillet_line_line_vertex_by_points(
            vertex_index,
            &previous_point,
            &next_point,
            center,
            clockwise,
            policy,
        )
    }

    /// Fillets one interior line-line vertex from exact tangent points and center.
    ///
    /// The tangent points must lie at strict interior parameters of the two
    /// adjacent finite line segments. The center must be equidistant from both
    /// tangent points, nonzero-radius, perpendicular to both line directions,
    /// and oriented so the inserted arc is tangent to the incoming and outgoing
    /// line traversal. Failed predicates are retained as explicit blockers.
    pub fn fillet_line_line_vertex_by_points(
        &self,
        vertex_index: usize,
        previous_point: &Point2,
        next_point: &Point2,
        center: &Point2,
        clockwise: bool,
        policy: &CurvePolicy,
    ) -> CurveResult<CurveStringFilletResult2> {
        if vertex_index == 0 || vertex_index >= self.len() {
            return Err(CurveError::InvalidCurveRange);
        }
        let previous_segment_index = vertex_index - 1;
        let next_segment_index = vertex_index;
        let previous_zero = CurveStringTrimPoint2::new(previous_segment_index, Real::zero());
        let next_zero = CurveStringTrimPoint2::new(next_segment_index, Real::zero());
        let (previous_line, next_line) = match (
            &self.segments[previous_segment_index],
            &self.segments[next_segment_index],
        ) {
            (Segment2::Line(previous), Segment2::Line(next)) => (previous, next),
            _ => {
                return Ok(blocked_fillet_result(
                    self,
                    previous_segment_index,
                    next_segment_index,
                    previous_zero,
                    next_zero,
                    Some(center.clone()),
                    None,
                    Vec::new(),
                    RetainedTopologyStatus::Unsupported,
                    Some(UncertaintyReason::Unsupported),
                ));
            }
        };

        let previous_param =
            match line_chamfer_point_parameter(previous_line, previous_point, policy)? {
                Classification::Decided(param) => param,
                Classification::Uncertain(reason) => {
                    return Ok(blocked_fillet_result(
                        self,
                        previous_segment_index,
                        next_segment_index,
                        previous_zero,
                        next_zero,
                        Some(center.clone()),
                        None,
                        Vec::new(),
                        retained_status_for_uncertainty(reason),
                        Some(reason),
                    ));
                }
            };
        let previous_trim =
            CurveStringTrimPoint2::new(previous_segment_index, previous_param.clone());
        let next_param = match line_chamfer_point_parameter(next_line, next_point, policy)? {
            Classification::Decided(param) => param,
            Classification::Uncertain(reason) => {
                return Ok(blocked_fillet_result(
                    self,
                    previous_segment_index,
                    next_segment_index,
                    previous_trim,
                    next_zero,
                    Some(center.clone()),
                    None,
                    Vec::new(),
                    retained_status_for_uncertainty(reason),
                    Some(reason),
                ));
            }
        };
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
                return Ok(blocked_fillet_result(
                    self,
                    previous_segment_index,
                    next_segment_index,
                    previous_trim,
                    next_trim,
                    Some(center.clone()),
                    None,
                    Vec::new(),
                    RetainedTopologyStatus::Unsupported,
                    Some(UncertaintyReason::Boundary),
                ));
            }
            _ => {
                return Ok(blocked_fillet_result(
                    self,
                    previous_segment_index,
                    next_segment_index,
                    previous_trim,
                    next_trim,
                    Some(center.clone()),
                    None,
                    Vec::new(),
                    RetainedTopologyStatus::Unresolved,
                    Some(UncertaintyReason::Ordering),
                ));
            }
        }

        let radius_squared = previous_point.distance_squared(center);
        match is_zero(&radius_squared, policy) {
            Some(false) => {}
            Some(true) => {
                return Ok(blocked_fillet_result(
                    self,
                    previous_segment_index,
                    next_segment_index,
                    previous_trim,
                    next_trim,
                    Some(center.clone()),
                    Some(radius_squared),
                    Vec::new(),
                    RetainedTopologyStatus::Unsupported,
                    Some(UncertaintyReason::Boundary),
                ));
            }
            None => {
                return Ok(blocked_fillet_result(
                    self,
                    previous_segment_index,
                    next_segment_index,
                    previous_trim,
                    next_trim,
                    Some(center.clone()),
                    Some(radius_squared),
                    Vec::new(),
                    RetainedTopologyStatus::Unresolved,
                    Some(UncertaintyReason::RealSign),
                ));
            }
        }

        let next_radius_squared = next_point.distance_squared(center);
        let radius_delta = &radius_squared - &next_radius_squared;
        match is_zero(&radius_delta, policy) {
            Some(true) => {}
            Some(false) => {
                return Ok(blocked_fillet_result(
                    self,
                    previous_segment_index,
                    next_segment_index,
                    previous_trim,
                    next_trim,
                    Some(center.clone()),
                    Some(radius_squared),
                    Vec::new(),
                    RetainedTopologyStatus::Unsupported,
                    Some(UncertaintyReason::Boundary),
                ));
            }
            None => {
                return Ok(blocked_fillet_result(
                    self,
                    previous_segment_index,
                    next_segment_index,
                    previous_trim,
                    next_trim,
                    Some(center.clone()),
                    Some(radius_squared),
                    Vec::new(),
                    RetainedTopologyStatus::Unresolved,
                    Some(UncertaintyReason::RealSign),
                ));
            }
        }

        if let Some(reason) = line_line_fillet_validation_blocker(
            previous_line,
            previous_point,
            center,
            clockwise,
            policy,
        ) {
            return Ok(blocked_fillet_result(
                self,
                previous_segment_index,
                next_segment_index,
                previous_trim,
                next_trim,
                Some(center.clone()),
                Some(radius_squared),
                Vec::new(),
                retained_status_for_uncertainty(reason),
                Some(reason),
            ));
        }
        if let Some(reason) =
            line_line_fillet_validation_blocker(next_line, next_point, center, clockwise, policy)
        {
            return Ok(blocked_fillet_result(
                self,
                previous_segment_index,
                next_segment_index,
                previous_trim,
                next_trim,
                Some(center.clone()),
                Some(radius_squared),
                Vec::new(),
                retained_status_for_uncertainty(reason),
                Some(reason),
            ));
        }

        let previous_range = ParamRange::new(Real::zero(), previous_trim.param().clone());
        let next_range = ParamRange::new(next_trim.param().clone(), Real::one());
        let previous_segment =
            LineSeg2::try_new(previous_line.start().clone(), previous_point.clone())?;
        let fillet_segment = CircularArc2::try_from_center(
            previous_point.clone(),
            next_point.clone(),
            center.clone(),
            clockwise,
        )?;
        let next_segment = LineSeg2::try_new(next_point.clone(), next_line.end().clone())?;

        let mut segments = Vec::with_capacity(self.len() + 1);
        segments.extend(self.segments[..previous_segment_index].iter().cloned());
        segments.push(Segment2::Line(previous_segment));
        let fillet_segment_index = segments.len();
        segments.push(Segment2::Arc(fillet_segment));
        segments.push(Segment2::Line(next_segment));
        segments.extend(self.segments[next_segment_index + 1..].iter().cloned());
        let curve_string = CurveString2::try_new(segments)?;
        let segment_reports = vec![
            CurveStringTrimSegmentReport2 {
                source_segment_index: previous_segment_index,
                source_range: previous_range,
                status: RetainedTopologyStatus::NativeExact,
            },
            CurveStringTrimSegmentReport2 {
                source_segment_index: next_segment_index,
                source_range: next_range,
                status: RetainedTopologyStatus::NativeExact,
            },
        ];
        Ok(CurveStringFilletResult2 {
            curve_string: Some(curve_string),
            report: CurveStringFilletReport2 {
                previous_segment_index,
                next_segment_index,
                previous_trim,
                next_trim,
                center: Some(center.clone()),
                radius_squared: Some(radius_squared),
                segment_reports,
                fillet_segment_index: Some(fillet_segment_index),
                source_segment_count: self.len(),
                status: RetainedTopologyStatus::NativeExact,
                blocker: None,
            },
        })
    }

    /// Extends one endpoint line segment to an exact point on its supporting line.
    ///
    /// This first extension slice deliberately supports only line endpoint
    /// segments. The target must be certified on the selected line support and
    /// outside the finite segment in the selected endpoint direction. Targets
    /// inside the existing segment, off the support, on arcs, or behind
    /// undecided predicates are retained as explicit blockers.
    pub fn extend_line_endpoint_to_point(
        &self,
        endpoint: CurveStringEndpoint2,
        target_point: Point2,
        policy: &CurvePolicy,
    ) -> CurveResult<CurveStringExtendResult2> {
        self.extend_endpoint_to_point(endpoint, target_point, policy)
    }

    /// Extends one endpoint segment to an exact target point.
    ///
    /// Lines extend along their supporting line. Circular arcs extend only when
    /// the target is certified on the same circle and outside the current
    /// finite arc. The resulting same-orientation native arc must replay both
    /// the old endpoint and an interior witness from the source arc, so the
    /// operation extends the existing topology instead of replacing it with an
    /// unrelated chord or sampled sweep.
    pub fn extend_endpoint_to_point(
        &self,
        endpoint: CurveStringEndpoint2,
        target_point: Point2,
        policy: &CurvePolicy,
    ) -> CurveResult<CurveStringExtendResult2> {
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
    ) -> CurveResult<CurveStringExtendResult2> {
        match line.classify_point(&target_point, policy) {
            Classification::Decided(crate::LineSide::On) => {}
            Classification::Decided(_) => {
                return Ok(blocked_extend_result(
                    self,
                    endpoint,
                    source_segment_index,
                    target_point,
                    None,
                    RetainedTopologyStatus::Unsupported,
                    Some(UncertaintyReason::Boundary),
                ));
            }
            Classification::Uncertain(reason) => {
                return Ok(blocked_extend_result(
                    self,
                    endpoint,
                    source_segment_index,
                    target_point,
                    None,
                    RetainedTopologyStatus::Unresolved,
                    Some(reason),
                ));
            }
        }

        let source_param = match line_point_parameter(line, &target_point, policy)? {
            Classification::Decided(param) => param,
            Classification::Uncertain(reason) => {
                return Ok(blocked_extend_result(
                    self,
                    endpoint,
                    source_segment_index,
                    target_point,
                    None,
                    RetainedTopologyStatus::Unresolved,
                    Some(reason),
                ));
            }
        };
        let outside = match endpoint {
            CurveStringEndpoint2::Start => compare_reals(&source_param, &Real::zero(), policy),
            CurveStringEndpoint2::End => compare_reals(&source_param, &Real::one(), policy),
        };
        match (endpoint, outside) {
            (CurveStringEndpoint2::Start, Some(Ordering::Less))
            | (CurveStringEndpoint2::End, Some(Ordering::Greater)) => {}
            (_, Some(_)) => {
                return Ok(blocked_extend_result(
                    self,
                    endpoint,
                    source_segment_index,
                    target_point,
                    Some(source_param),
                    RetainedTopologyStatus::Unsupported,
                    Some(UncertaintyReason::Boundary),
                ));
            }
            (_, None) => {
                return Ok(blocked_extend_result(
                    self,
                    endpoint,
                    source_segment_index,
                    target_point,
                    Some(source_param),
                    RetainedTopologyStatus::Unresolved,
                    Some(UncertaintyReason::Ordering),
                ));
            }
        }

        let mut segments = self.segments.clone();
        segments[source_segment_index] = match endpoint {
            CurveStringEndpoint2::Start => {
                Segment2::Line(LineSeg2::try_new(target_point.clone(), line.end().clone())?)
            }
            CurveStringEndpoint2::End => Segment2::Line(LineSeg2::try_new(
                line.start().clone(),
                target_point.clone(),
            )?),
        };
        let curve_string = CurveString2::try_new(segments)?;
        Ok(CurveStringExtendResult2 {
            curve_string: Some(curve_string),
            report: CurveStringExtendReport2 {
                endpoint,
                source_segment_index,
                target_point,
                source_param: Some(source_param),
                source_segment_count: self.len(),
                status: RetainedTopologyStatus::NativeExact,
                blocker: None,
            },
        })
    }

    fn extend_arc_endpoint_segment_to_point(
        &self,
        endpoint: CurveStringEndpoint2,
        source_segment_index: usize,
        arc: &CircularArc2,
        target_point: Point2,
        policy: &CurvePolicy,
    ) -> CurveResult<CurveStringExtendResult2> {
        let radius_delta = target_point.distance_squared(arc.center()) - arc.radius_squared();
        match is_zero(&radius_delta, policy) {
            Some(true) => {}
            Some(false) => {
                return Ok(blocked_extend_result(
                    self,
                    endpoint,
                    source_segment_index,
                    target_point,
                    None,
                    RetainedTopologyStatus::Unsupported,
                    Some(UncertaintyReason::Boundary),
                ));
            }
            None => {
                return Ok(blocked_extend_result(
                    self,
                    endpoint,
                    source_segment_index,
                    target_point,
                    None,
                    RetainedTopologyStatus::Unresolved,
                    Some(UncertaintyReason::RealSign),
                ));
            }
        }

        match arc.contains_point(&target_point, policy) {
            Classification::Decided(false) => {}
            Classification::Decided(true) => {
                return Ok(blocked_extend_result(
                    self,
                    endpoint,
                    source_segment_index,
                    target_point,
                    None,
                    RetainedTopologyStatus::Unsupported,
                    Some(UncertaintyReason::Boundary),
                ));
            }
            Classification::Uncertain(reason) => {
                return Ok(blocked_extend_result(
                    self,
                    endpoint,
                    source_segment_index,
                    target_point,
                    None,
                    RetainedTopologyStatus::Unresolved,
                    Some(reason),
                ));
            }
        }

        let extended_arc = match endpoint {
            CurveStringEndpoint2::Start => CircularArc2::new_unchecked_with_radius(
                target_point.clone(),
                arc.end().clone(),
                arc.center().clone(),
                arc.radius_squared(),
                arc.is_clockwise(),
                None,
            ),
            CurveStringEndpoint2::End => CircularArc2::new_unchecked_with_radius(
                arc.start().clone(),
                target_point.clone(),
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
                return Ok(blocked_extend_result(
                    self,
                    endpoint,
                    source_segment_index,
                    target_point,
                    None,
                    RetainedTopologyStatus::Unsupported,
                    Some(UncertaintyReason::Unsupported),
                ));
            }
            Classification::Uncertain(reason) => {
                return Ok(blocked_extend_result(
                    self,
                    endpoint,
                    source_segment_index,
                    target_point,
                    None,
                    RetainedTopologyStatus::Unresolved,
                    Some(reason),
                ));
            }
        }

        let representative = match arc.representative_point(policy)? {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => {
                return Ok(blocked_extend_result(
                    self,
                    endpoint,
                    source_segment_index,
                    target_point,
                    None,
                    RetainedTopologyStatus::Unresolved,
                    Some(reason),
                ));
            }
        };
        match extended_arc.contains_point(&representative, policy) {
            Classification::Decided(true) => {}
            Classification::Decided(false) => {
                return Ok(blocked_extend_result(
                    self,
                    endpoint,
                    source_segment_index,
                    target_point,
                    None,
                    RetainedTopologyStatus::Unsupported,
                    Some(UncertaintyReason::Unsupported),
                ));
            }
            Classification::Uncertain(reason) => {
                return Ok(blocked_extend_result(
                    self,
                    endpoint,
                    source_segment_index,
                    target_point,
                    None,
                    RetainedTopologyStatus::Unresolved,
                    Some(reason),
                ));
            }
        }

        let mut segments = self.segments.clone();
        segments[source_segment_index] = Segment2::Arc(extended_arc);
        let curve_string = CurveString2::try_new(segments)?;
        Ok(CurveStringExtendResult2 {
            curve_string: Some(curve_string),
            report: CurveStringExtendReport2 {
                endpoint,
                source_segment_index,
                target_point,
                source_param: None,
                source_segment_count: self.len(),
                status: RetainedTopologyStatus::NativeExact,
                blocker: None,
            },
        })
    }

    pub(crate) fn trim_between_curve_intersection_events(
        &self,
        start_events: CurveStringIntersectionResult2,
        end_events: CurveStringIntersectionResult2,
        query_path: CurveStringCurveTrimQueryPath2,
        policy: &CurvePolicy,
    ) -> CurveResult<CurveStringCurveTrimResult2> {
        let start_intersection_report = start_events.report().clone();
        let end_intersection_report = end_events.report().clone();
        let start_extraction = extract_curve_trim_hits(start_events.intersections());
        let end_extraction = extract_curve_trim_hits(end_events.intersections());

        let start_hit = match single_curve_trim_hit(&start_extraction) {
            Ok(hit) => hit,
            Err((status, blocker)) => {
                return Ok(blocked_curve_trim_result(
                    start_extraction.hits,
                    end_extraction.hits,
                    start_intersection_report,
                    end_intersection_report,
                    None,
                    query_path,
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
                    start_intersection_report,
                    end_intersection_report,
                    None,
                    query_path,
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
                start_intersection_report,
                end_intersection_report,
                trim_report: Some(trim.report().clone()),
                query_path,
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
        self.intersect_curve_string_with_report(other, policy)
            .map(|result| result.into_intersections())
    }

    /// Collects all nonempty segment-pair intersections with scan evidence.
    ///
    /// The returned report records how many segment pairs were considered,
    /// skipped by decided AABB disjointness, and tested with exact segment
    /// topology. The broad phase remains advisory: every non-skipped pair is
    /// resolved by the same exact segment intersection dispatch as
    /// [`CurveString2::intersect_curve_string`].
    pub fn intersect_curve_string_with_report(
        &self,
        other: &Self,
        policy: &CurvePolicy,
    ) -> CurveResult<CurveStringIntersectionResult2> {
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

        intersect_curve_strings_with_cached_aabbs_with_report(
            self,
            other,
            &self_boxes,
            &other_boxes,
            CurveStringIntersectionQueryPath2::Direct,
            policy,
        )
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

    fn endpoint_connection_report_for_kind(
        &self,
        other: &Self,
        kind: CurveStringLinkKind2,
        policy: &CurvePolicy,
    ) -> CurveResult<CurveStringEndpointConnectionReport2> {
        match kind {
            CurveStringLinkKind2::FirstEndToSecondStart => self.endpoint_connection_report(
                other,
                CurveStringEndpoint2::End,
                CurveStringEndpoint2::Start,
                policy,
            ),
            CurveStringLinkKind2::FirstEndToSecondEnd => self.endpoint_connection_report(
                other,
                CurveStringEndpoint2::End,
                CurveStringEndpoint2::End,
                policy,
            ),
            CurveStringLinkKind2::FirstStartToSecondStart => self.endpoint_connection_report(
                other,
                CurveStringEndpoint2::Start,
                CurveStringEndpoint2::Start,
                policy,
            ),
            CurveStringLinkKind2::FirstStartToSecondEnd => self.endpoint_connection_report(
                other,
                CurveStringEndpoint2::Start,
                CurveStringEndpoint2::End,
                policy,
            ),
        }
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

impl CurveStringChamferReport2 {
    /// Returns the previous source segment index at the chamfered vertex.
    pub const fn previous_segment_index(&self) -> usize {
        self.previous_segment_index
    }

    /// Returns the next source segment index at the chamfered vertex.
    pub const fn next_segment_index(&self) -> usize {
        self.next_segment_index
    }

    /// Returns the previous line trim point.
    pub const fn previous_trim(&self) -> &CurveStringTrimPoint2 {
        &self.previous_trim
    }

    /// Returns the next line trim point.
    pub const fn next_trim(&self) -> &CurveStringTrimPoint2 {
        &self.next_trim
    }

    /// Returns retained source ranges for the shortened adjacent line segments.
    pub fn segment_reports(&self) -> &[CurveStringTrimSegmentReport2] {
        &self.segment_reports
    }

    /// Returns the inserted chamfer segment index in the output curve string.
    pub const fn chamfer_segment_index(&self) -> Option<usize> {
        self.chamfer_segment_index
    }

    /// Returns the source curve-string segment count captured by this report.
    pub const fn source_segment_count(&self) -> usize {
        self.source_segment_count
    }

    /// Returns chamfer materialization status.
    pub const fn status(&self) -> RetainedTopologyStatus {
        self.status
    }

    /// Returns the exact blocker for non-materialized chamfers.
    pub const fn blocker(&self) -> Option<UncertaintyReason> {
        self.blocker
    }

    pub(crate) fn remap_source_segment_indices(&mut self, mut remap: impl FnMut(usize) -> usize) {
        self.previous_segment_index = remap(self.previous_segment_index);
        self.next_segment_index = remap(self.next_segment_index);
        self.previous_trim.segment_index = remap(self.previous_trim.segment_index);
        self.next_trim.segment_index = remap(self.next_trim.segment_index);
        for segment_report in &mut self.segment_reports {
            segment_report.source_segment_index = remap(segment_report.source_segment_index);
        }
    }
}

impl CurveStringChamferResult2 {
    /// Returns the materialized chamfered curve string, if supported.
    pub const fn curve_string(&self) -> Option<&CurveString2> {
        self.curve_string.as_ref()
    }

    /// Consumes this result and returns the materialized chamfered curve string, if any.
    pub fn into_curve_string(self) -> Option<CurveString2> {
        self.curve_string
    }

    /// Returns the retained chamfer report.
    pub const fn report(&self) -> &CurveStringChamferReport2 {
        &self.report
    }

    pub(crate) const fn report_mut(&mut self) -> &mut CurveStringChamferReport2 {
        &mut self.report
    }
}

impl CurveStringFilletReport2 {
    /// Returns the previous source segment index at the filleted vertex.
    pub const fn previous_segment_index(&self) -> usize {
        self.previous_segment_index
    }

    /// Returns the next source segment index at the filleted vertex.
    pub const fn next_segment_index(&self) -> usize {
        self.next_segment_index
    }

    /// Returns the previous line trim point.
    pub const fn previous_trim(&self) -> &CurveStringTrimPoint2 {
        &self.previous_trim
    }

    /// Returns the next line trim point.
    pub const fn next_trim(&self) -> &CurveStringTrimPoint2 {
        &self.next_trim
    }

    /// Returns the certified fillet center when the attempt reached center validation.
    pub const fn center(&self) -> Option<&Point2> {
        self.center.as_ref()
    }

    /// Returns the certified squared radius when the attempt reached radius validation.
    pub const fn radius_squared(&self) -> Option<&Real> {
        self.radius_squared.as_ref()
    }

    /// Returns retained source ranges for the shortened adjacent line segments.
    pub fn segment_reports(&self) -> &[CurveStringTrimSegmentReport2] {
        &self.segment_reports
    }

    /// Returns the inserted fillet arc segment index in the output curve string.
    pub const fn fillet_segment_index(&self) -> Option<usize> {
        self.fillet_segment_index
    }

    /// Returns the source curve-string segment count captured by this report.
    pub const fn source_segment_count(&self) -> usize {
        self.source_segment_count
    }

    /// Returns fillet materialization status.
    pub const fn status(&self) -> RetainedTopologyStatus {
        self.status
    }

    /// Returns the exact blocker for non-materialized fillets.
    pub const fn blocker(&self) -> Option<UncertaintyReason> {
        self.blocker
    }

    pub(crate) fn remap_source_segment_indices(&mut self, mut remap: impl FnMut(usize) -> usize) {
        self.previous_segment_index = remap(self.previous_segment_index);
        self.next_segment_index = remap(self.next_segment_index);
        self.previous_trim.segment_index = remap(self.previous_trim.segment_index);
        self.next_trim.segment_index = remap(self.next_trim.segment_index);
        for segment_report in &mut self.segment_reports {
            segment_report.source_segment_index = remap(segment_report.source_segment_index);
        }
    }
}

impl CurveStringFilletResult2 {
    /// Returns the materialized filleted curve string, if supported.
    pub const fn curve_string(&self) -> Option<&CurveString2> {
        self.curve_string.as_ref()
    }

    /// Consumes this result and returns the materialized filleted curve string, if any.
    pub fn into_curve_string(self) -> Option<CurveString2> {
        self.curve_string
    }

    /// Returns the retained fillet report.
    pub const fn report(&self) -> &CurveStringFilletReport2 {
        &self.report
    }

    pub(crate) const fn report_mut(&mut self) -> &mut CurveStringFilletReport2 {
        &mut self.report
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

    /// Returns scan evidence for the start-cutter intersection query.
    pub const fn start_intersection_report(&self) -> &CurveStringIntersectionReport2 {
        &self.start_intersection_report
    }

    /// Returns scan evidence for the end-cutter intersection query.
    pub const fn end_intersection_report(&self) -> &CurveStringIntersectionReport2 {
        &self.end_intersection_report
    }

    /// Returns the intersection query path used to collect split evidence.
    pub const fn query_path(&self) -> CurveStringCurveTrimQueryPath2 {
        self.query_path
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

impl CurveStringRegionTrimHit2 {
    /// Returns the source curve-string segment index.
    pub const fn source_segment_index(&self) -> usize {
        self.source_segment_index
    }

    /// Returns whether the hit came from a material or hole contour.
    pub const fn region_contour_role(&self) -> RegionContourRole {
        self.region_contour_role
    }

    /// Returns the contour index within its region role bin.
    pub const fn region_contour_index(&self) -> usize {
        self.region_contour_index
    }

    /// Returns the region boundary segment index.
    pub const fn region_segment_index(&self) -> usize {
        self.region_segment_index
    }

    /// Returns the exact boundary point witness.
    pub const fn point(&self) -> &Point2 {
        &self.point
    }

    /// Returns the retained source segment parameter.
    pub const fn source_param(&self) -> &Real {
        &self.source_param
    }

    /// Returns the local intersection kind.
    pub const fn kind(&self) -> IntersectionKind {
        self.kind
    }
}

impl CurveStringRegionTrimIntervalReport2 {
    /// Returns the source segment index for this retained interval.
    pub const fn source_segment_index(&self) -> usize {
        self.source_segment_index
    }

    /// Returns the retained source parameter range.
    pub const fn source_range(&self) -> &ParamRange {
        &self.source_range
    }

    /// Returns the representative point used for region classification.
    pub const fn representative_point(&self) -> Option<&Point2> {
        self.representative_point.as_ref()
    }

    /// Returns the region location of the representative, when decided.
    pub const fn location(&self) -> Option<RegionPointLocation> {
        self.location
    }

    /// Returns the output curve-string index that consumed this interval.
    pub const fn output_curve_string_index(&self) -> Option<usize> {
        self.output_curve_string_index
    }

    /// Returns the output segment index within the output curve string.
    pub const fn output_segment_index(&self) -> Option<usize> {
        self.output_segment_index
    }

    /// Returns retained topology status for this interval.
    pub const fn status(&self) -> RetainedTopologyStatus {
        self.status
    }

    /// Returns the exact blocker for this interval, if any.
    pub const fn blocker(&self) -> Option<UncertaintyReason> {
        self.blocker
    }
}

impl CurveStringRegionTrimReport2 {
    /// Returns the source segment count captured by this report.
    pub const fn source_segment_count(&self) -> usize {
        self.source_segment_count
    }

    /// Returns the number of material contours in the clipping region.
    pub const fn region_material_contour_count(&self) -> usize {
        self.region_material_contour_count
    }

    /// Returns the number of hole contours in the clipping region.
    pub const fn region_hole_contour_count(&self) -> usize {
        self.region_hole_contour_count
    }

    /// Returns exact boundary hits used as split evidence.
    pub fn boundary_hits(&self) -> &[CurveStringRegionTrimHit2] {
        &self.boundary_hits
    }

    /// Returns retained interval classifications.
    pub fn interval_reports(&self) -> &[CurveStringRegionTrimIntervalReport2] {
        &self.interval_reports
    }

    /// Returns output curve-string count when trim-by-region materialized.
    pub const fn output_curve_string_count(&self) -> Option<usize> {
        self.output_curve_string_count
    }

    /// Returns the query path used to collect boundary and classification evidence.
    pub const fn query_path(&self) -> CurveStringRegionTrimQueryPath2 {
        self.query_path
    }

    /// Returns trim-by-region materialization status.
    pub const fn status(&self) -> RetainedTopologyStatus {
        self.status
    }

    /// Returns the exact blocker for non-materialized trim-by-region attempts.
    pub const fn blocker(&self) -> Option<UncertaintyReason> {
        self.blocker
    }
}

impl CurveStringRegionTrimResult2 {
    /// Returns materialized inside curve-string chains.
    pub fn curve_strings(&self) -> &[CurveString2] {
        &self.curve_strings
    }

    /// Consumes this result and returns materialized inside curve-string chains.
    pub fn into_curve_strings(self) -> Vec<CurveString2> {
        self.curve_strings
    }

    /// Returns the retained trim-by-region report.
    pub const fn report(&self) -> &CurveStringRegionTrimReport2 {
        &self.report
    }
}

impl CurveStringExtendReport2 {
    /// Returns which endpoint was extended.
    pub const fn endpoint(&self) -> CurveStringEndpoint2 {
        self.endpoint
    }

    /// Returns the endpoint segment index in the source curve string.
    pub const fn source_segment_index(&self) -> usize {
        self.source_segment_index
    }

    /// Returns the requested exact target point.
    pub const fn target_point(&self) -> &Point2 {
        &self.target_point
    }

    /// Returns the affine parameter on the source endpoint line, when certified.
    pub const fn source_param(&self) -> Option<&Real> {
        self.source_param.as_ref()
    }

    /// Returns the source curve-string segment count captured by this report.
    pub const fn source_segment_count(&self) -> usize {
        self.source_segment_count
    }

    /// Returns extension materialization status.
    pub const fn status(&self) -> RetainedTopologyStatus {
        self.status
    }

    /// Returns the exact blocker for non-materialized extensions.
    pub const fn blocker(&self) -> Option<UncertaintyReason> {
        self.blocker
    }
}

impl CurveStringExtendResult2 {
    /// Returns the materialized extended curve string, if supported.
    pub const fn curve_string(&self) -> Option<&CurveString2> {
        self.curve_string.as_ref()
    }

    /// Consumes this result and returns the materialized extended curve string, if any.
    pub fn into_curve_string(self) -> Option<CurveString2> {
        self.curve_string
    }

    /// Returns the retained extension report.
    pub const fn report(&self) -> &CurveStringExtendReport2 {
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

#[derive(Clone, Debug, PartialEq)]
struct RegionTrimSplitPoint2 {
    trim_point: CurveStringTrimPoint2,
    point: Point2,
}

enum NearestEndpointChoice {
    Selected(CurveStringLinkKind2, CurveStringEndpointConnectionReport2),
    Ambiguous(CurveStringLinkKind2, CurveStringEndpointConnectionReport2),
    Unresolved(
        CurveStringLinkKind2,
        CurveStringEndpointConnectionReport2,
        UncertaintyReason,
    ),
    Empty,
}

fn trim_curve_string_inside_region(
    curve_string: &CurveString2,
    region: &Region2,
    policy: &CurvePolicy,
) -> CurveResult<CurveStringRegionTrimResult2> {
    let mut boundary_hits = Vec::new();
    if let Some((status, blocker)) =
        collect_region_trim_boundary_hits(curve_string, region, policy, &mut boundary_hits)?
    {
        return Ok(blocked_region_trim_result(
            curve_string,
            region.material_contours().len(),
            region.hole_contours().len(),
            boundary_hits,
            Vec::new(),
            CurveStringRegionTrimQueryPath2::Direct,
            status,
            blocker,
        ));
    }

    trim_curve_string_inside_region_with_hits(
        curve_string,
        region.material_contours().len(),
        region.hole_contours().len(),
        boundary_hits,
        CurveStringRegionTrimQueryPath2::Direct,
        policy,
        |point| region.classify_point(point, policy),
    )
}

fn trim_curve_string_inside_prepared_region(
    curve_string: &CurveString2,
    region: &PreparedRegionView2<'_>,
    policy: &CurvePolicy,
) -> CurveResult<CurveStringRegionTrimResult2> {
    let mut boundary_hits = Vec::new();
    if let Some((status, blocker)) = collect_prepared_region_trim_boundary_hits(
        curve_string,
        region,
        policy,
        &mut boundary_hits,
    )? {
        return Ok(blocked_region_trim_result(
            curve_string,
            region.material_contours().len(),
            region.hole_contours().len(),
            boundary_hits,
            Vec::new(),
            CurveStringRegionTrimQueryPath2::Prepared,
            status,
            blocker,
        ));
    }

    trim_curve_string_inside_region_with_hits(
        curve_string,
        region.material_contours().len(),
        region.hole_contours().len(),
        boundary_hits,
        CurveStringRegionTrimQueryPath2::Prepared,
        policy,
        |point| region.classify_point(point, policy),
    )
}

fn trim_curve_string_inside_region_with_hits(
    curve_string: &CurveString2,
    region_material_contour_count: usize,
    region_hole_contour_count: usize,
    boundary_hits: Vec<CurveStringRegionTrimHit2>,
    query_path: CurveStringRegionTrimQueryPath2,
    policy: &CurvePolicy,
    mut classify_point: impl FnMut(&Point2) -> Classification<RegionPointLocation>,
) -> CurveResult<CurveStringRegionTrimResult2> {
    let mut output_segments: Vec<Vec<Segment2>> = Vec::new();
    let mut current_segments = Vec::new();
    let mut interval_reports = Vec::new();

    for (source_segment_index, source_segment) in curve_string.segments().iter().enumerate() {
        let split_points = match region_trim_split_points_for_segment(
            source_segment_index,
            source_segment,
            &boundary_hits,
            policy,
        )? {
            Classification::Decided(split_points) => split_points,
            Classification::Uncertain(reason) => {
                return Ok(blocked_region_trim_result(
                    curve_string,
                    region_material_contour_count,
                    region_hole_contour_count,
                    boundary_hits,
                    interval_reports,
                    query_path,
                    retained_status_for_uncertainty(reason),
                    reason,
                ));
            }
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
                SegmentTrimMaterialization::Unsupported(reason) => {
                    interval_reports.push(CurveStringRegionTrimIntervalReport2 {
                        source_segment_index,
                        source_range,
                        representative_point: None,
                        location: None,
                        output_curve_string_index: None,
                        output_segment_index: None,
                        status: RetainedTopologyStatus::Unsupported,
                        blocker: Some(reason),
                    });
                    return Ok(blocked_region_trim_result(
                        curve_string,
                        region_material_contour_count,
                        region_hole_contour_count,
                        boundary_hits,
                        interval_reports,
                        query_path,
                        RetainedTopologyStatus::Unsupported,
                        reason,
                    ));
                }
                SegmentTrimMaterialization::Unresolved(reason) => {
                    interval_reports.push(CurveStringRegionTrimIntervalReport2 {
                        source_segment_index,
                        source_range,
                        representative_point: None,
                        location: None,
                        output_curve_string_index: None,
                        output_segment_index: None,
                        status: RetainedTopologyStatus::Unresolved,
                        blocker: Some(reason),
                    });
                    return Ok(blocked_region_trim_result(
                        curve_string,
                        region_material_contour_count,
                        region_hole_contour_count,
                        boundary_hits,
                        interval_reports,
                        query_path,
                        RetainedTopologyStatus::Unresolved,
                        reason,
                    ));
                }
            };

            let representative = match fragment.representative_point(policy)? {
                Classification::Decided(point) => point,
                Classification::Uncertain(reason) => {
                    interval_reports.push(CurveStringRegionTrimIntervalReport2 {
                        source_segment_index,
                        source_range,
                        representative_point: None,
                        location: None,
                        output_curve_string_index: None,
                        output_segment_index: None,
                        status: retained_status_for_uncertainty(reason),
                        blocker: Some(reason),
                    });
                    return Ok(blocked_region_trim_result(
                        curve_string,
                        region_material_contour_count,
                        region_hole_contour_count,
                        boundary_hits,
                        interval_reports,
                        query_path,
                        retained_status_for_uncertainty(reason),
                        reason,
                    ));
                }
            };

            let location = match classify_point(&representative) {
                Classification::Decided(location) => location,
                Classification::Uncertain(reason) => {
                    interval_reports.push(CurveStringRegionTrimIntervalReport2 {
                        source_segment_index,
                        source_range,
                        representative_point: Some(representative),
                        location: None,
                        output_curve_string_index: None,
                        output_segment_index: None,
                        status: retained_status_for_uncertainty(reason),
                        blocker: Some(reason),
                    });
                    return Ok(blocked_region_trim_result(
                        curve_string,
                        region_material_contour_count,
                        region_hole_contour_count,
                        boundary_hits,
                        interval_reports,
                        query_path,
                        retained_status_for_uncertainty(reason),
                        reason,
                    ));
                }
            };

            match location {
                RegionPointLocation::Inside => {
                    let output_curve_string_index = output_segments.len();
                    let output_segment_index = current_segments.len();
                    current_segments.push(fragment);
                    interval_reports.push(CurveStringRegionTrimIntervalReport2 {
                        source_segment_index,
                        source_range,
                        representative_point: Some(representative),
                        location: Some(location),
                        output_curve_string_index: Some(output_curve_string_index),
                        output_segment_index: Some(output_segment_index),
                        status: RetainedTopologyStatus::NativeExact,
                        blocker: None,
                    });
                }
                RegionPointLocation::Outside => {
                    flush_region_trim_chain(&mut output_segments, &mut current_segments);
                    interval_reports.push(CurveStringRegionTrimIntervalReport2 {
                        source_segment_index,
                        source_range,
                        representative_point: Some(representative),
                        location: Some(location),
                        output_curve_string_index: None,
                        output_segment_index: None,
                        status: RetainedTopologyStatus::NativeExact,
                        blocker: None,
                    });
                }
                RegionPointLocation::Boundary => {
                    interval_reports.push(CurveStringRegionTrimIntervalReport2 {
                        source_segment_index,
                        source_range,
                        representative_point: Some(representative),
                        location: Some(location),
                        output_curve_string_index: None,
                        output_segment_index: None,
                        status: RetainedTopologyStatus::Unsupported,
                        blocker: Some(UncertaintyReason::Boundary),
                    });
                    return Ok(blocked_region_trim_result(
                        curve_string,
                        region_material_contour_count,
                        region_hole_contour_count,
                        boundary_hits,
                        interval_reports,
                        query_path,
                        RetainedTopologyStatus::Unsupported,
                        UncertaintyReason::Boundary,
                    ));
                }
            }
        }
    }

    flush_region_trim_chain(&mut output_segments, &mut current_segments);
    let mut curve_strings = Vec::with_capacity(output_segments.len());
    for segments in output_segments {
        curve_strings.push(CurveString2::try_new(segments)?);
    }

    Ok(CurveStringRegionTrimResult2 {
        report: CurveStringRegionTrimReport2 {
            source_segment_count: curve_string.len(),
            region_material_contour_count,
            region_hole_contour_count,
            boundary_hits,
            interval_reports,
            output_curve_string_count: Some(curve_strings.len()),
            query_path,
            status: RetainedTopologyStatus::NativeExact,
            blocker: None,
        },
        curve_strings,
    })
}

fn collect_region_trim_boundary_hits(
    curve_string: &CurveString2,
    region: &Region2,
    policy: &CurvePolicy,
    hits: &mut Vec<CurveStringRegionTrimHit2>,
) -> CurveResult<Option<(RetainedTopologyStatus, UncertaintyReason)>> {
    for (contour_index, contour) in region.material_contours().iter().enumerate() {
        if let Some(blocker) = collect_region_trim_contour_hits(
            curve_string,
            contour,
            RegionContourRole::Material,
            contour_index,
            policy,
            hits,
        )? {
            return Ok(Some(blocker));
        }
    }
    for (contour_index, contour) in region.hole_contours().iter().enumerate() {
        if let Some(blocker) = collect_region_trim_contour_hits(
            curve_string,
            contour,
            RegionContourRole::Hole,
            contour_index,
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
    region: &PreparedRegionView2<'_>,
    policy: &CurvePolicy,
    hits: &mut Vec<CurveStringRegionTrimHit2>,
) -> CurveResult<Option<(RetainedTopologyStatus, UncertaintyReason)>> {
    for (contour_index, contour) in region.prepared_material_contours().iter().enumerate() {
        if let Some(blocker) = collect_prepared_region_trim_contour_hits(
            curve_string,
            contour,
            RegionContourRole::Material,
            contour_index,
            policy,
            hits,
        )? {
            return Ok(Some(blocker));
        }
    }
    for (contour_index, contour) in region.prepared_hole_contours().iter().enumerate() {
        if let Some(blocker) = collect_prepared_region_trim_contour_hits(
            curve_string,
            contour,
            RegionContourRole::Hole,
            contour_index,
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
    contour: &crate::Contour2,
    role: RegionContourRole,
    contour_index: usize,
    policy: &CurvePolicy,
    hits: &mut Vec<CurveStringRegionTrimHit2>,
) -> CurveResult<Option<(RetainedTopologyStatus, UncertaintyReason)>> {
    for (source_segment_index, source_segment) in curve_string.segments().iter().enumerate() {
        for (region_segment_index, region_segment) in contour.segments().iter().enumerate() {
            let relation = source_segment.intersect_segment(region_segment, policy)?;
            if let Some(blocker) = append_region_trim_hits_from_relation(
                hits,
                source_segment_index,
                source_segment,
                role,
                contour_index,
                region_segment_index,
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
    contour: &crate::PreparedContourView2<'_>,
    role: RegionContourRole,
    contour_index: usize,
    policy: &CurvePolicy,
    hits: &mut Vec<CurveStringRegionTrimHit2>,
) -> CurveResult<Option<(RetainedTopologyStatus, UncertaintyReason)>> {
    let source_segment_boxes: Vec<_> = curve_string
        .segments()
        .iter()
        .map(|segment| decided_segment_aabb(segment, policy))
        .collect();
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
                role,
                contour_index,
                region_segment_index,
                relation,
                policy,
            )? {
                return Ok(Some(blocker));
            }
        }
    }
    Ok(None)
}

#[allow(clippy::too_many_arguments)]
fn append_region_trim_hits_from_relation(
    hits: &mut Vec<CurveStringRegionTrimHit2>,
    source_segment_index: usize,
    source_segment: &Segment2,
    role: RegionContourRole,
    contour_index: usize,
    region_segment_index: usize,
    relation: SegmentIntersection,
    policy: &CurvePolicy,
) -> CurveResult<Option<(RetainedTopologyStatus, UncertaintyReason)>> {
    match relation {
        SegmentIntersection::LineLine(LineLineIntersection::None)
        | SegmentIntersection::LineArc {
            result: LineArcIntersection::None,
            ..
        }
        | SegmentIntersection::ArcArc(ArcArcIntersection::None) => Ok(None),
        SegmentIntersection::LineLine(LineLineIntersection::Point { point, kind, .. }) => {
            push_region_trim_hit(
                hits,
                source_segment_index,
                source_segment,
                role,
                contour_index,
                region_segment_index,
                point,
                kind,
                policy,
            )
        }
        SegmentIntersection::LineArc {
            result: LineArcIntersection::Point(hit),
            ..
        } => push_region_trim_hit(
            hits,
            source_segment_index,
            source_segment,
            role,
            contour_index,
            region_segment_index,
            hit.point,
            hit.kind,
            policy,
        ),
        SegmentIntersection::ArcArc(ArcArcIntersection::Point(hit)) => push_region_trim_hit(
            hits,
            source_segment_index,
            source_segment,
            role,
            contour_index,
            region_segment_index,
            hit.point,
            hit.kind,
            policy,
        ),
        SegmentIntersection::LineArc {
            result: LineArcIntersection::TwoPoints { first, second },
            ..
        } => {
            if let Some(blocker) = push_region_trim_hit(
                hits,
                source_segment_index,
                source_segment,
                role,
                contour_index,
                region_segment_index,
                first.point,
                first.kind,
                policy,
            )? {
                return Ok(Some(blocker));
            }
            push_region_trim_hit(
                hits,
                source_segment_index,
                source_segment,
                role,
                contour_index,
                region_segment_index,
                second.point,
                second.kind,
                policy,
            )
        }
        SegmentIntersection::ArcArc(ArcArcIntersection::TwoPoints { first, second }) => {
            if let Some(blocker) = push_region_trim_hit(
                hits,
                source_segment_index,
                source_segment,
                role,
                contour_index,
                region_segment_index,
                first.point,
                first.kind,
                policy,
            )? {
                return Ok(Some(blocker));
            }
            push_region_trim_hit(
                hits,
                source_segment_index,
                source_segment,
                role,
                contour_index,
                region_segment_index,
                second.point,
                second.kind,
                policy,
            )
        }
        SegmentIntersection::LineLine(LineLineIntersection::Overlap { .. })
        | SegmentIntersection::ArcArc(ArcArcIntersection::Overlap { .. }) => Ok(Some((
            RetainedTopologyStatus::Unsupported,
            UncertaintyReason::Unsupported,
        ))),
        SegmentIntersection::LineLine(LineLineIntersection::Uncertain { reason })
        | SegmentIntersection::LineArc {
            result: LineArcIntersection::Uncertain { reason },
            ..
        }
        | SegmentIntersection::ArcArc(ArcArcIntersection::Uncertain { reason }) => {
            Ok(Some((RetainedTopologyStatus::Unresolved, reason)))
        }
    }
}

#[allow(clippy::too_many_arguments)]
fn push_region_trim_hit(
    hits: &mut Vec<CurveStringRegionTrimHit2>,
    source_segment_index: usize,
    source_segment: &Segment2,
    role: RegionContourRole,
    contour_index: usize,
    region_segment_index: usize,
    point: Point2,
    kind: IntersectionKind,
    policy: &CurvePolicy,
) -> CurveResult<Option<(RetainedTopologyStatus, UncertaintyReason)>> {
    let source_param = match segment_point_parameter(source_segment, &point, policy)? {
        Classification::Decided(param) => param,
        Classification::Uncertain(reason) => {
            return Ok(Some((retained_status_for_uncertainty(reason), reason)));
        }
    };
    hits.push(CurveStringRegionTrimHit2 {
        source_segment_index,
        region_contour_role: role,
        region_contour_index: contour_index,
        region_segment_index,
        point,
        source_param,
        kind,
    });
    Ok(None)
}

fn region_trim_split_points_for_segment(
    source_segment_index: usize,
    source_segment: &Segment2,
    hits: &[CurveStringRegionTrimHit2],
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

fn blocked_region_trim_result(
    curve_string: &CurveString2,
    region_material_contour_count: usize,
    region_hole_contour_count: usize,
    boundary_hits: Vec<CurveStringRegionTrimHit2>,
    interval_reports: Vec<CurveStringRegionTrimIntervalReport2>,
    query_path: CurveStringRegionTrimQueryPath2,
    status: RetainedTopologyStatus,
    blocker: UncertaintyReason,
) -> CurveStringRegionTrimResult2 {
    CurveStringRegionTrimResult2 {
        curve_strings: Vec::new(),
        report: CurveStringRegionTrimReport2 {
            source_segment_count: curve_string.len(),
            region_material_contour_count,
            region_hole_contour_count,
            boundary_hits,
            interval_reports,
            output_curve_string_count: None,
            query_path,
            status,
            blocker: Some(blocker),
        },
    }
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
    start_intersection_report: CurveStringIntersectionReport2,
    end_intersection_report: CurveStringIntersectionReport2,
    trim_report: Option<CurveStringTrimReport2>,
    query_path: CurveStringCurveTrimQueryPath2,
    status: RetainedTopologyStatus,
    blocker: Option<UncertaintyReason>,
) -> CurveStringCurveTrimResult2 {
    CurveStringCurveTrimResult2 {
        curve_string: None,
        report: CurveStringCurveTrimReport2 {
            start_hits,
            end_hits,
            start_intersection_report,
            end_intersection_report,
            trim_report,
            query_path,
            status,
            blocker,
        },
    }
}

fn blocked_extend_result(
    curve_string: &CurveString2,
    endpoint: CurveStringEndpoint2,
    source_segment_index: usize,
    target_point: Point2,
    source_param: Option<Real>,
    status: RetainedTopologyStatus,
    blocker: Option<UncertaintyReason>,
) -> CurveStringExtendResult2 {
    CurveStringExtendResult2 {
        curve_string: None,
        report: CurveStringExtendReport2 {
            endpoint,
            source_segment_index,
            target_point,
            source_param,
            source_segment_count: curve_string.len(),
            status,
            blocker,
        },
    }
}

fn blocked_chamfer_result(
    curve_string: &CurveString2,
    previous_segment_index: usize,
    next_segment_index: usize,
    previous_trim: CurveStringTrimPoint2,
    next_trim: CurveStringTrimPoint2,
    segment_reports: Vec<CurveStringTrimSegmentReport2>,
    status: RetainedTopologyStatus,
    blocker: Option<UncertaintyReason>,
) -> CurveStringChamferResult2 {
    CurveStringChamferResult2 {
        curve_string: None,
        report: CurveStringChamferReport2 {
            previous_segment_index,
            next_segment_index,
            previous_trim,
            next_trim,
            segment_reports,
            chamfer_segment_index: None,
            source_segment_count: curve_string.len(),
            status,
            blocker,
        },
    }
}

fn blocked_fillet_result(
    curve_string: &CurveString2,
    previous_segment_index: usize,
    next_segment_index: usize,
    previous_trim: CurveStringTrimPoint2,
    next_trim: CurveStringTrimPoint2,
    center: Option<Point2>,
    radius_squared: Option<Real>,
    segment_reports: Vec<CurveStringTrimSegmentReport2>,
    status: RetainedTopologyStatus,
    blocker: Option<UncertaintyReason>,
) -> CurveStringFilletResult2 {
    CurveStringFilletResult2 {
        curve_string: None,
        report: CurveStringFilletReport2 {
            previous_segment_index,
            next_segment_index,
            previous_trim,
            next_trim,
            center,
            radius_squared,
            segment_reports,
            fillet_segment_index: None,
            source_segment_count: curve_string.len(),
            status,
            blocker,
        },
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

fn line_chamfer_point_parameter(
    line: &LineSeg2,
    point: &Point2,
    policy: &CurvePolicy,
) -> CurveResult<Classification<Real>> {
    match line.contains_point(point, policy) {
        Classification::Decided(true) => line_point_parameter(line, point, policy),
        Classification::Decided(false) => {
            Ok(Classification::Uncertain(UncertaintyReason::Boundary))
        }
        Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
    }
}

fn line_line_fillet_validation_blocker(
    line: &LineSeg2,
    tangent_point: &Point2,
    center: &Point2,
    clockwise: bool,
    policy: &CurvePolicy,
) -> Option<UncertaintyReason> {
    let (line_dx, line_dy) = line.delta();
    let (radius_dx, radius_dy) = tangent_point.delta_from(center);
    let perpendicular = (&line_dx * &radius_dx) + (&line_dy * &radius_dy);
    match is_zero(&perpendicular, policy) {
        Some(true) => {}
        Some(false) => return Some(UncertaintyReason::Boundary),
        None => return Some(UncertaintyReason::RealSign),
    }

    let (tangent_dx, tangent_dy) = if clockwise {
        (radius_dy, -radius_dx)
    } else {
        (-radius_dy, radius_dx)
    };
    let direction_dot = (&line_dx * &tangent_dx) + (&line_dy * &tangent_dy);
    match real_sign(&direction_dot, policy) {
        Some(RealSign::Positive) => None,
        Some(RealSign::Zero | RealSign::Negative) => Some(UncertaintyReason::Boundary),
        None => Some(UncertaintyReason::RealSign),
    }
}

fn retained_status_for_uncertainty(reason: UncertaintyReason) -> RetainedTopologyStatus {
    match reason {
        UncertaintyReason::Boundary | UncertaintyReason::Unsupported => {
            RetainedTopologyStatus::Unsupported
        }
        _ => RetainedTopologyStatus::Unresolved,
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

impl CurveStringIntersectionReport2 {
    pub(crate) const fn new_native_exact(
        first_segment_count: usize,
        second_segment_count: usize,
        candidate_pair_count: usize,
        skipped_aabb_pair_count: usize,
        tested_pair_count: usize,
        intersection_count: usize,
        query_path: CurveStringIntersectionQueryPath2,
    ) -> Self {
        Self {
            first_segment_count,
            second_segment_count,
            candidate_pair_count,
            skipped_aabb_pair_count,
            tested_pair_count,
            intersection_count,
            query_path,
            status: RetainedTopologyStatus::NativeExact,
            blocker: None,
        }
    }

    /// Returns the first curve-string segment count.
    pub const fn first_segment_count(&self) -> usize {
        self.first_segment_count
    }

    /// Returns the second curve-string segment count.
    pub const fn second_segment_count(&self) -> usize {
        self.second_segment_count
    }

    /// Returns the total flat segment-pair candidates before broad-phase skips.
    pub const fn candidate_pair_count(&self) -> usize {
        self.candidate_pair_count
    }

    /// Returns segment pairs skipped by decided AABB disjointness.
    pub const fn skipped_aabb_pair_count(&self) -> usize {
        self.skipped_aabb_pair_count
    }

    /// Returns segment pairs tested by exact segment topology.
    pub const fn tested_pair_count(&self) -> usize {
        self.tested_pair_count
    }

    /// Returns nonempty segment-pair intersections collected.
    pub const fn intersection_count(&self) -> usize {
        self.intersection_count
    }

    /// Returns the query path used to collect intersections.
    pub const fn query_path(&self) -> CurveStringIntersectionQueryPath2 {
        self.query_path
    }

    /// Returns intersection collection status.
    pub const fn status(&self) -> RetainedTopologyStatus {
        self.status
    }

    /// Returns the exact blocker for non-materialized intersection collection.
    pub const fn blocker(&self) -> Option<UncertaintyReason> {
        self.blocker
    }
}

impl CurveStringIntersectionResult2 {
    pub(crate) fn from_parts(
        intersections: Vec<CurveStringIntersection>,
        report: CurveStringIntersectionReport2,
    ) -> Self {
        Self {
            intersections,
            report,
        }
    }

    /// Returns collected segment-pair intersections.
    pub fn intersections(&self) -> &[CurveStringIntersection] {
        &self.intersections
    }

    /// Consumes this result and returns collected intersections.
    pub fn into_intersections(self) -> Vec<CurveStringIntersection> {
        self.intersections
    }

    /// Returns retained scan evidence for this intersection query.
    pub const fn report(&self) -> &CurveStringIntersectionReport2 {
        &self.report
    }
}

impl CurveStringOrderedLinkStepReport2 {
    /// Returns source curve-string indices already accumulated before this step.
    pub fn accumulated_source_indices(&self) -> &[usize] {
        &self.accumulated_source_indices
    }

    /// Returns the next source curve-string index consumed by this step.
    pub const fn next_source_index(&self) -> usize {
        self.next_source_index
    }

    /// Returns the pairwise link report when this step materialized.
    pub const fn link_report(&self) -> Option<&CurveStringLinkReport2> {
        self.link_report.as_ref()
    }

    /// Returns topology status for this ordered link step.
    pub const fn status(&self) -> RetainedTopologyStatus {
        self.status
    }

    /// Returns the exact blocker for non-materialized link steps.
    pub const fn blocker(&self) -> Option<UncertaintyReason> {
        self.blocker
    }
}

impl CurveStringOrderedLinkReport2 {
    /// Returns the source curve-string count captured by this report.
    pub const fn source_curve_string_count(&self) -> usize {
        self.source_curve_string_count
    }

    /// Returns the output segment count when ordered linking materialized.
    pub const fn output_segment_count(&self) -> Option<usize> {
        self.output_segment_count
    }

    /// Returns ordered link steps and their pairwise evidence.
    pub fn steps(&self) -> &[CurveStringOrderedLinkStepReport2] {
        &self.steps
    }

    /// Returns ordered-link materialization status.
    pub const fn status(&self) -> RetainedTopologyStatus {
        self.status
    }

    /// Returns the exact blocker for a non-materialized ordered link.
    pub const fn blocker(&self) -> Option<UncertaintyReason> {
        self.blocker
    }
}

impl OrderedLinkedCurveString2 {
    /// Returns the materialized ordered linked curve string, if supported.
    pub const fn curve_string(&self) -> Option<&CurveString2> {
        self.curve_string.as_ref()
    }

    /// Consumes this result and returns the linked curve string, if any.
    pub fn into_curve_string(self) -> Option<CurveString2> {
        self.curve_string
    }

    /// Returns the retained ordered-link report.
    pub const fn report(&self) -> &CurveStringOrderedLinkReport2 {
        &self.report
    }
}

impl CurveStringConnectReport2 {
    /// Returns the selected connector orientation, when one was selected.
    pub const fn kind(&self) -> Option<CurveStringLinkKind2> {
        self.kind
    }

    /// Returns endpoint equality evidence for the connector endpoints.
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

    /// Returns the inserted connector segment index in the output curve string.
    pub const fn connector_segment_index(&self) -> Option<usize> {
        self.connector_segment_index
    }

    /// Returns connector materialization status.
    pub const fn status(&self) -> RetainedTopologyStatus {
        self.status
    }

    /// Returns the exact blocker for non-materialized connections.
    pub const fn blocker(&self) -> Option<UncertaintyReason> {
        self.blocker
    }
}

impl ConnectedCurveString2 {
    /// Returns the connected curve string, if a connector was materialized.
    pub const fn curve_string(&self) -> Option<&CurveString2> {
        self.curve_string.as_ref()
    }

    /// Consumes this result and returns the connected curve string, if any.
    pub fn into_curve_string(self) -> Option<CurveString2> {
        self.curve_string
    }

    /// Returns the retained connector report.
    pub const fn report(&self) -> &CurveStringConnectReport2 {
        &self.report
    }
}

impl CurveStringLineMergeSpanReport2 {
    /// Returns the first source segment index included in this output segment.
    pub const fn source_start_segment_index(&self) -> usize {
        self.source_start_segment_index
    }

    /// Returns the final source segment index included in this output segment.
    pub const fn source_end_segment_index(&self) -> usize {
        self.source_end_segment_index
    }

    /// Returns the output segment index produced for this source run.
    pub const fn output_segment_index(&self) -> usize {
        self.output_segment_index
    }

    /// Returns retained topology status for this source run.
    pub const fn status(&self) -> RetainedTopologyStatus {
        self.status
    }
}

impl CurveStringLineMergeReport2 {
    /// Returns the source curve-string segment count captured by this report.
    pub const fn source_segment_count(&self) -> usize {
        self.source_segment_count
    }

    /// Returns the output segment count when the merge materialized.
    pub const fn output_segment_count(&self) -> Option<usize> {
        self.output_segment_count
    }

    /// Returns retained source runs for materialized output segments.
    pub fn spans(&self) -> &[CurveStringLineMergeSpanReport2] {
        &self.spans
    }

    /// Returns merge materialization status.
    pub const fn status(&self) -> RetainedTopologyStatus {
        self.status
    }

    /// Returns the exact blocker for non-materialized merge attempts.
    pub const fn blocker(&self) -> Option<UncertaintyReason> {
        self.blocker
    }
}

impl CurveStringLineMergeResult2 {
    /// Returns the materialized merged curve string, if supported.
    pub const fn curve_string(&self) -> Option<&CurveString2> {
        self.curve_string.as_ref()
    }

    /// Consumes this result and returns the materialized merged curve string, if any.
    pub fn into_curve_string(self) -> Option<CurveString2> {
        self.curve_string
    }

    /// Returns the retained line-merge report.
    pub const fn report(&self) -> &CurveStringLineMergeReport2 {
        &self.report
    }
}

impl CurveStringReversedDuplicatePairReport2 {
    /// Returns the first source segment index removed by this cancellation.
    pub const fn first_source_segment_index(&self) -> usize {
        self.first_source_segment_index
    }

    /// Returns the second source segment index removed by this cancellation.
    pub const fn second_source_segment_index(&self) -> usize {
        self.second_source_segment_index
    }

    /// Returns retained topology status for this duplicate-pair removal.
    pub const fn status(&self) -> RetainedTopologyStatus {
        self.status
    }
}

impl CurveStringDeduplicateReport2 {
    /// Returns the source curve-string segment count captured by this report.
    pub const fn source_segment_count(&self) -> usize {
        self.source_segment_count
    }

    /// Returns the output segment count when de-duplication materialized.
    pub const fn output_segment_count(&self) -> Option<usize> {
        self.output_segment_count
    }

    /// Returns exact reversed duplicate pairs removed by this operation.
    pub fn removed_pairs(&self) -> &[CurveStringReversedDuplicatePairReport2] {
        &self.removed_pairs
    }

    /// Returns de-duplication materialization status.
    pub const fn status(&self) -> RetainedTopologyStatus {
        self.status
    }

    /// Returns the exact blocker for non-materialized de-duplication attempts.
    pub const fn blocker(&self) -> Option<UncertaintyReason> {
        self.blocker
    }
}

impl CurveStringDeduplicateResult2 {
    /// Returns the materialized de-duplicated curve string, if supported.
    pub const fn curve_string(&self) -> Option<&CurveString2> {
        self.curve_string.as_ref()
    }

    /// Consumes this result and returns the materialized curve string, if any.
    pub fn into_curve_string(self) -> Option<CurveString2> {
        self.curve_string
    }

    /// Returns retained de-duplication evidence.
    pub const fn report(&self) -> &CurveStringDeduplicateReport2 {
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

fn ordered_link_source_indices(
    accumulated_source_indices: &[usize],
    next_source_index: usize,
    kind: CurveStringLinkKind2,
) -> Vec<usize> {
    let mut source_indices = Vec::with_capacity(accumulated_source_indices.len() + 1);
    match kind {
        CurveStringLinkKind2::FirstEndToSecondStart | CurveStringLinkKind2::FirstEndToSecondEnd => {
            source_indices.extend_from_slice(accumulated_source_indices);
            source_indices.push(next_source_index);
        }
        CurveStringLinkKind2::FirstStartToSecondStart => {
            source_indices.extend(accumulated_source_indices.iter().rev().copied());
            source_indices.push(next_source_index);
        }
        CurveStringLinkKind2::FirstStartToSecondEnd => {
            source_indices.push(next_source_index);
            source_indices.extend_from_slice(accumulated_source_indices);
        }
    }
    source_indices
}

fn blocked_connected_curve_string(
    first: &CurveString2,
    second: &CurveString2,
    kind: Option<CurveStringLinkKind2>,
    endpoint_report: CurveStringEndpointConnectionReport2,
    status: RetainedTopologyStatus,
    blocker: Option<UncertaintyReason>,
) -> ConnectedCurveString2 {
    ConnectedCurveString2 {
        curve_string: None,
        report: CurveStringConnectReport2 {
            kind,
            endpoint_report,
            first_segment_count: first.len(),
            second_segment_count: second.len(),
            connector_segment_index: None,
            status,
            blocker,
        },
    }
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

fn unique_nearest_endpoint_report(
    candidates: Vec<(CurveStringLinkKind2, CurveStringEndpointConnectionReport2)>,
    policy: &CurvePolicy,
) -> NearestEndpointChoice {
    let mut best: Option<(CurveStringLinkKind2, CurveStringEndpointConnectionReport2)> = None;
    for candidate in candidates {
        let Some((_, best_report)) = best.as_ref() else {
            best = Some(candidate);
            continue;
        };
        match compare_reals(
            candidate.1.distance_squared(),
            best_report.distance_squared(),
            policy,
        ) {
            Some(Ordering::Less) => best = Some(candidate),
            Some(Ordering::Greater) => {}
            Some(Ordering::Equal) => {
                return NearestEndpointChoice::Ambiguous(candidate.0, candidate.1);
            }
            None => {
                return NearestEndpointChoice::Unresolved(
                    candidate.0,
                    candidate.1,
                    UncertaintyReason::Ordering,
                );
            }
        }
    }

    match best {
        Some((kind, report)) => NearestEndpointChoice::Selected(kind, report),
        None => NearestEndpointChoice::Empty,
    }
}

fn reversed_segments(segments: &[Segment2]) -> Vec<Segment2> {
    segments
        .iter()
        .rev()
        .map(Segment2::reversed)
        .collect::<Vec<_>>()
}

pub(crate) fn intersect_curve_strings_with_cached_aabbs_with_report(
    first: &CurveString2,
    second: &CurveString2,
    first_segment_boxes: &[Option<Aabb2>],
    second_segment_boxes: &[Option<Aabb2>],
    query_path: CurveStringIntersectionQueryPath2,
    policy: &CurvePolicy,
) -> CurveResult<CurveStringIntersectionResult2> {
    let mut intersections = Vec::new();
    let candidate_pair_count = first.segments.len() * second.segments.len();
    let mut skipped_aabb_pair_count = 0_usize;
    let mut tested_pair_count = 0_usize;

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
                skipped_aabb_pair_count += 1;
                continue;
            }

            tested_pair_count += 1;
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

    let intersection_count = intersections.len();
    Ok(CurveStringIntersectionResult2 {
        intersections,
        report: CurveStringIntersectionReport2 {
            first_segment_count: first.len(),
            second_segment_count: second.len(),
            candidate_pair_count,
            skipped_aabb_pair_count,
            tested_pair_count,
            intersection_count,
            query_path,
            status: RetainedTopologyStatus::NativeExact,
            blocker: None,
        },
    })
}
