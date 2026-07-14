//! Prepared borrowed query structures for repeated topology classification.
//!
//! Prepared views cache conservative broad-phase data but do not replace exact
//! topology. They skip only decided bounding-box misses and then delegate to the
//! same segment-intersection and boundary-first contour classification used by
//! ordinary contours and regions. This keeps preparation in the candidate
//! generation role of Bentley and Ottmann's intersection-reporting framework,
//! while preserving Shewchuk-style certified predicates for topology branches.

use crate::bbox::{Aabb2, aabb_decided_misses_point, aabbs_decided_disjoint, decided_segment_aabb};
use crate::curve_string::{curve_string_intersection_relation_counts, decided_segment_box_count};
use crate::facts::{CurveStringFacts, RegionFacts};
use crate::region_events::RegionIntersectionWorkload;
use crate::{
    BooleanBoundaryLoopSet, BooleanOp, CircularArc2, CircularArc2Facts, Classification, Contour2,
    ContourIntersectionSet, ContourPointLocation, CurvePolicy, CurveResult, CurveString2,
    CurveStringCurveTrimQueryPath2, CurveStringCurveTrimResult2, CurveStringIntersection,
    CurveStringIntersectionPreparedCacheReport2, CurveStringIntersectionQueryPath2,
    CurveStringIntersectionReport2, CurveStringIntersectionResult2, CurveStringPreparedCacheAudit2,
    CurveStringRegionTrimPreparedCacheReport2, CurveStringRegionTrimResult2, FillRule, LineSeg2,
    LineSeg2Facts, LineSide, Point2, Region2, RegionBooleanResult2, RegionContourIntersection,
    RegionContourKey, RegionContourRole, RegionIntersectionSet, RegionPointLocation, RegionSide,
    RegionTrimPreparedCacheAudit2, RegionView2, Segment2, SegmentIntersection, SegmentKind,
    SegmentKindCounts, UncertaintyReason,
};

/// Prepared point-line classifier for a fixed [`LineSeg2`].
///
/// This view caches the segment's structural facts and, when the `predicates`
/// feature is enabled, the converted `hyperlimit` endpoints used by repeated
/// orientation tests. It deliberately does not own finite-segment containment
/// semantics: those remain on [`LineSeg2`], while this type accelerates the
/// exact supporting-line predicate. That split follows Yap's EGC model of
/// carrying object structure forward without moving combinatorial decisions
/// out of the predicate layer; see Yap, "Towards Exact Geometric Computation,"
/// *Computational Geometry* 7.1-2 (1997).
#[derive(Clone, Debug, PartialEq)]
pub struct PreparedLineSeg2<'a> {
    line: &'a LineSeg2,
    facts: LineSeg2Facts,
    #[cfg(feature = "predicates")]
    predicate_start: hyperlimit::Point2,
    #[cfg(feature = "predicates")]
    predicate_end: hyperlimit::Point2,
    #[cfg(feature = "predicates")]
    predicate_facts: hyperlimit::PreparedPredicateFacts,
}

impl<'a> PreparedLineSeg2<'a> {
    /// Builds a prepared borrowed line segment.
    pub fn from_line_segment(line: &'a LineSeg2) -> Self {
        let facts = line.structural_facts();
        #[cfg(feature = "predicates")]
        {
            let predicate_start = predicate_point(line.start());
            let predicate_end = predicate_point(line.end());
            let predicate_facts =
                hyperlimit::PreparedLine2::new(&predicate_start, &predicate_end).facts();
            Self {
                line,
                facts,
                predicate_start,
                predicate_end,
                predicate_facts,
            }
        }

        #[cfg(not(feature = "predicates"))]
        {
            Self { line, facts }
        }
    }

    /// Returns the borrowed source line segment.
    pub const fn line_segment(&self) -> &'a LineSeg2 {
        self.line
    }

    /// Returns conservative structural facts collected during preparation.
    ///
    /// Structural-dispatch note: future line-only curve paths can use these
    /// facts to choose axis-aligned, common-denominator, or symbolic-family
    /// batches before issuing certified orientation predicates. The facts are
    /// advisory and never replace exact predicate outcomes.
    pub const fn facts(&self) -> &LineSeg2Facts {
        &self.facts
    }

    /// Classifies a point relative to this segment's oriented supporting line.
    pub fn classify_point(&self, point: &Point2, policy: &CurvePolicy) -> Classification<LineSide> {
        #[cfg(feature = "predicates")]
        if !matches!(policy.numeric_mode, crate::NumericMode::EdgePreview) {
            // Reuse the fixed endpoint conversion and prepared facts, then let
            // hyperlimit select the exact determinant schedule. This is the
            // Shewchuk-style orientation predicate at the curve-object
            // boundary, with Yap's exact/approximate split preserved by
            // keeping EdgePreview outside the certified path.
            let query = predicate_point(point);
            return classify_prepared_line(
                &self.predicate_start,
                &self.predicate_end,
                self.predicate_facts,
                &query,
                policy,
            );
        }

        self.line.classify_point(point, policy)
    }
}

/// Prepared sweep and circle classifier for a fixed [`CircularArc2`].
///
/// The prepared arc stores the two radial oriented lines that bound the arc
/// sweep. Point-on-arc checks still compare exact squared radius first, then
/// use those prepared radial predicates for angular containment. This mirrors
/// Schneider and Eberly's circle/arc primitive decomposition while preserving
/// Yap's EGC split between exact topology predicates and approximate output
/// adapters. See Schneider and Eberly, *Geometric Tools for Computer Graphics*
/// (Morgan Kaufmann, 2002), and Yap, "Towards Exact Geometric Computation,"
/// *Computational Geometry* 7.1-2 (1997).
#[derive(Clone, Debug, PartialEq)]
pub struct PreparedCircularArc2<'a> {
    arc: &'a CircularArc2,
    facts: CircularArc2Facts,
    #[cfg(feature = "predicates")]
    predicate_center: hyperlimit::Point2,
    #[cfg(feature = "predicates")]
    predicate_start: hyperlimit::Point2,
    #[cfg(feature = "predicates")]
    predicate_end: hyperlimit::Point2,
    #[cfg(feature = "predicates")]
    center_start_facts: hyperlimit::PreparedPredicateFacts,
    #[cfg(feature = "predicates")]
    center_end_facts: hyperlimit::PreparedPredicateFacts,
}

impl<'a> PreparedCircularArc2<'a> {
    /// Builds a prepared borrowed circular arc.
    pub fn from_circular_arc(arc: &'a CircularArc2) -> Self {
        let facts = arc.structural_facts();
        #[cfg(feature = "predicates")]
        {
            let predicate_center = predicate_point(arc.center());
            let predicate_start = predicate_point(arc.start());
            let predicate_end = predicate_point(arc.end());
            let center_start_facts =
                hyperlimit::PreparedLine2::new(&predicate_center, &predicate_start).facts();
            let center_end_facts =
                hyperlimit::PreparedLine2::new(&predicate_center, &predicate_end).facts();
            Self {
                arc,
                facts,
                predicate_center,
                predicate_start,
                predicate_end,
                center_start_facts,
                center_end_facts,
            }
        }

        #[cfg(not(feature = "predicates"))]
        {
            Self { arc, facts }
        }
    }

    /// Returns the borrowed source arc.
    pub const fn circular_arc(&self) -> &'a CircularArc2 {
        self.arc
    }

    /// Returns conservative structural facts collected during preparation.
    ///
    /// Structural-dispatch note: exact-rational radius and endpoint scale facts
    /// are the right hooks for future line-circle and circle-circle batches.
    /// They select candidate kernels; certified sign and orientation predicates
    /// still decide topology.
    pub const fn facts(&self) -> &CircularArc2Facts {
        &self.facts
    }

    /// Classifies whether a point lies inside this arc's angular sweep.
    pub fn contains_sweep_point(
        &self,
        point: &Point2,
        policy: &CurvePolicy,
    ) -> Classification<bool> {
        #[cfg(feature = "predicates")]
        if !matches!(policy.numeric_mode, crate::NumericMode::EdgePreview) {
            let sweep_kind = match crate::arc_bezier::classify_sweep(self.arc, None) {
                Ok(kind) => kind,
                Err(crate::ExactCurveError::Blocked(blocker)) => {
                    return Classification::Uncertain(blocker.reason());
                }
                Err(crate::ExactCurveError::Invalid { .. }) => {
                    return Classification::Uncertain(UncertaintyReason::Predicate);
                }
            };
            if sweep_kind == crate::arc_bezier::ArcSweepKind::FullCircle {
                return Classification::Decided(true);
            }
            let query = predicate_point(point);
            let start_side = classify_prepared_line(
                &self.predicate_center,
                &self.predicate_start,
                self.center_start_facts,
                &query,
                policy,
            );
            let end_side = classify_prepared_line(
                &self.predicate_center,
                &self.predicate_end,
                self.center_end_facts,
                &query,
                policy,
            );
            let (Classification::Decided(start_side), Classification::Decided(end_side)) =
                (start_side, end_side)
            else {
                return Classification::Uncertain(UncertaintyReason::Predicate);
            };

            return self
                .arc
                .contains_classified_sweep_sides(start_side, end_side, sweep_kind);
        }

        self.arc.contains_sweep_point(point, policy)
    }

    /// Classifies whether a point lies on this finite circular arc.
    pub fn contains_point(&self, point: &Point2, policy: &CurvePolicy) -> Classification<bool> {
        let radius_delta = point.distance_squared(self.arc.center()) - self.arc.radius_squared();
        match crate::classify::is_zero(&radius_delta, policy) {
            Some(false) => Classification::Decided(false),
            Some(true) => self.contains_sweep_point(point, policy),
            None => Classification::Uncertain(UncertaintyReason::RealSign),
        }
    }
}

/// Prepared exact-predicate handle for a native segment.
///
/// This enum mirrors [`Segment2`] at the prepared-object layer. It gives curve
/// strings and contours a place to retain per-segment line/arc predicate
/// handles discovered during preparation, while keeping segment topology owned
/// by `hypercurve` and scalar/predicate decisions owned by `hyperlimit`.
#[derive(Clone, Debug, PartialEq)]
#[allow(clippy::large_enum_variant)]
pub enum PreparedSegment2<'a> {
    /// Prepared line-segment predicates.
    Line(PreparedLineSeg2<'a>),
    /// Prepared circular-arc predicates.
    Arc(PreparedCircularArc2<'a>),
}

impl<'a> PreparedSegment2<'a> {
    /// Builds a prepared borrowed segment handle.
    pub fn from_segment(segment: &'a Segment2) -> Self {
        match segment {
            Segment2::Line(line) => Self::Line(PreparedLineSeg2::from_line_segment(line)),
            Segment2::Arc(arc) => Self::Arc(PreparedCircularArc2::from_circular_arc(arc)),
        }
    }

    /// Returns whether this handle prepares a line segment.
    pub const fn is_line(&self) -> bool {
        matches!(self, Self::Line(_))
    }

    /// Returns whether this handle prepares a circular arc.
    pub const fn is_arc(&self) -> bool {
        matches!(self, Self::Arc(_))
    }

    /// Returns the primitive family prepared by this segment handle.
    pub const fn segment_kind(&self) -> SegmentKind {
        match self {
            Self::Line(_) => SegmentKind::Line,
            Self::Arc(_) => SegmentKind::Arc,
        }
    }

    /// Returns the exact start point of the prepared source segment.
    pub const fn start(&self) -> &Point2 {
        match self {
            Self::Line(line) => line.line_segment().start(),
            Self::Arc(arc) => arc.circular_arc().start(),
        }
    }

    /// Returns the exact end point of the prepared source segment.
    pub const fn end(&self) -> &Point2 {
        match self {
            Self::Line(line) => line.line_segment().end(),
            Self::Arc(arc) => arc.circular_arc().end(),
        }
    }

    /// Intersects two prepared native segment handles.
    ///
    /// This is the prepared segment-pair batch boundary used by prepared curve
    /// strings and contours. It deliberately returns the same
    /// [`SegmentIntersection`] shape as [`Segment2::intersect_segment`]: cached
    /// line and arc facts can select faster exact kernels, but finite segment
    /// topology and uncertainty remain represented by `hypercurve`'s public
    /// intersection enums. This follows Yap's EGC separation between carried
    /// object facts and certified predicate decisions; see Yap, "Towards Exact
    /// Geometric Computation," *Computational Geometry* 7.1-2 (1997).
    pub fn intersect_prepared_segment(
        &self,
        other: &PreparedSegment2<'a>,
        policy: &CurvePolicy,
    ) -> CurveResult<SegmentIntersection> {
        match (self, other) {
            (Self::Line(first), Self::Line(second)) => first
                .line_segment()
                .intersect_line(second.line_segment(), policy)
                .map(SegmentIntersection::LineLine),
            (Self::Line(line), Self::Arc(arc)) => Ok(SegmentIntersection::LineArc {
                order: crate::LineArcOrder::LineThenArc,
                result: line
                    .line_segment()
                    .intersect_arc(arc.circular_arc(), policy)?,
            }),
            (Self::Arc(arc), Self::Line(line)) => Ok(SegmentIntersection::LineArc {
                order: crate::LineArcOrder::ArcThenLine,
                result: line
                    .line_segment()
                    .intersect_arc(arc.circular_arc(), policy)?,
            }),
            (Self::Arc(first), Self::Arc(second)) => first
                .circular_arc()
                .intersect_arc(second.circular_arc(), policy)
                .map(SegmentIntersection::ArcArc),
        }
    }
}

/// A borrowed curve string with cached segment and whole-string bounding boxes.
///
/// Prepared curve strings avoid rebuilding broad-phase boxes for repeated
/// topology queries. The cache never decides a contact on its own: it skips only
/// decided disjoint boxes and keeps exact line/arc intersections authoritative.
/// This mirrors the candidate-pruning role described by Bentley and Ottmann,
/// "Algorithms for Reporting and Counting Geometric Intersections" (1979),
/// while retaining the current flat pair enumeration.
#[derive(Clone, Debug, PartialEq)]
pub struct PreparedCurveStringView2<'a> {
    curve: &'a CurveString2,
    prepared_segments: Vec<PreparedSegment2<'a>>,
    segment_boxes: Vec<Option<Aabb2>>,
    curve_box: Option<Aabb2>,
    facts: CurveStringFacts,
}

impl<'a> PreparedCurveStringView2<'a> {
    /// Builds a prepared borrowed curve string.
    pub fn from_curve_string(curve: &'a CurveString2, policy: &CurvePolicy) -> Self {
        // Structural-dispatch note: this preparation pass already visits every
        // segment. It is the natural place to retain facts such as all-line,
        // all-axis-aligned, monotone parameter ranges, or certified disjoint
        // interval buckets so later intersection queries can choose sweep-line
        // or grid-index paths instead of the current flat candidate scan.
        let segment_boxes = decided_segment_boxes(curve.segments(), policy);
        let curve_box = union_all_decided_boxes(segment_boxes.iter().map(Option::as_ref), policy);
        let facts = crate::facts::curve_string_facts(
            curve,
            segment_boxes.iter().filter(|bbox| bbox.is_some()).count(),
            curve_box.is_some(),
        );
        let prepared_segments = prepared_segments(curve.segments());

        Self {
            curve,
            prepared_segments,
            segment_boxes,
            curve_box,
            facts,
        }
    }

    /// Returns the borrowed source curve string.
    pub const fn curve_string(&self) -> &'a CurveString2 {
        self.curve
    }

    /// Returns the cached whole-curve box when every segment box was decided.
    pub const fn curve_box(&self) -> Option<&Aabb2> {
        self.curve_box.as_ref()
    }

    /// Returns cached segment boxes in source segment order.
    pub fn segment_boxes(&self) -> &[Option<Aabb2>] {
        &self.segment_boxes
    }

    /// Returns the number of prepared source segments.
    pub fn prepared_segment_count(&self) -> usize {
        self.prepared_segments.len()
    }

    /// Returns primitive-family counts for prepared source segments.
    pub fn prepared_segment_kind_counts(&self) -> SegmentKindCounts {
        prepared_segment_kind_counts(&self.prepared_segments)
    }

    /// Returns the number of segment boxes that were decided during preparation.
    pub fn decided_segment_box_count(&self) -> usize {
        self.segment_boxes
            .iter()
            .filter(|bbox| bbox.is_some())
            .count()
    }

    /// Returns the number of source segments whose preparation could not retain
    /// a decided broad-phase box.
    pub fn undecided_segment_box_count(&self) -> usize {
        self.segment_boxes.len() - self.decided_segment_box_count()
    }

    /// Returns prepared per-segment predicate handles in source segment order.
    ///
    /// These handles are retained for future all-line, line/arc, and arc/arc
    /// batches. Current broad-phase query methods still delegate through the
    /// ordinary segment APIs so behavior remains unchanged while the object
    /// cache boundary is established.
    pub fn prepared_segments(&self) -> &[PreparedSegment2<'a>] {
        &self.prepared_segments
    }

    /// Returns conservative structural facts collected while preparing.
    ///
    /// Structural-dispatch note: these facts are the intended home for future
    /// all-line, axis-aligned, common-scale, and symbolic-family routing of
    /// repeated curve-string intersection workloads. They do not certify
    /// topology; exact predicates and explicit uncertainty still do that.
    pub const fn facts(&self) -> &CurveStringFacts {
        &self.facts
    }

    /// Collects all nonempty segment-pair intersections against another
    /// prepared curve string.
    pub fn intersect_prepared_curve_string(
        &self,
        other: &PreparedCurveStringView2<'_>,
        policy: &CurvePolicy,
    ) -> CurveResult<Vec<CurveStringIntersection>> {
        self.intersect_prepared_curve_string_with_report(other, policy)
            .map(|result| result.into_intersections())
    }

    /// Collects intersections against another prepared curve string with scan evidence.
    pub fn intersect_prepared_curve_string_with_report(
        &self,
        other: &PreparedCurveStringView2<'_>,
        policy: &CurvePolicy,
    ) -> CurveResult<CurveStringIntersectionResult2> {
        intersect_prepared_segment_pairs_with_cached_aabbs(
            &self.prepared_segments,
            &other.prepared_segments,
            self.segment_boxes(),
            other.segment_boxes(),
            CurveStringIntersectionQueryPath2::Prepared,
            Some(curve_string_intersection_prepared_cache_report(self, other)),
            policy,
        )
    }

    /// Collects all nonempty segment-pair intersections against an ordinary
    /// borrowed curve string.
    pub fn intersect_curve_string(
        &self,
        other: &CurveString2,
        policy: &CurvePolicy,
    ) -> CurveResult<Vec<CurveStringIntersection>> {
        self.intersect_curve_string_with_report(other, policy)
            .map(|result| result.into_intersections())
    }

    /// Collects intersections against an ordinary curve string with scan evidence.
    pub fn intersect_curve_string_with_report(
        &self,
        other: &CurveString2,
        policy: &CurvePolicy,
    ) -> CurveResult<CurveStringIntersectionResult2> {
        let other = PreparedCurveStringView2::from_curve_string(other, policy);
        self.intersect_prepared_curve_string_with_report(&other, policy)
    }

    /// Trims the prepared source curve between point intersections with two prepared cutters.
    ///
    /// The cached broad-phase boxes in all three prepared views are reused for
    /// intersection collection; exact split validation and materialization still
    /// delegate to the source [`CurveString2`] trim pipeline.
    pub fn trim_between_prepared_curve_intersections(
        &self,
        start_cutter: &PreparedCurveStringView2<'_>,
        end_cutter: &PreparedCurveStringView2<'_>,
        policy: &CurvePolicy,
    ) -> CurveResult<CurveStringCurveTrimResult2> {
        let start_events =
            self.intersect_prepared_curve_string_with_report(start_cutter, policy)?;
        let end_events = self.intersect_prepared_curve_string_with_report(end_cutter, policy)?;
        self.curve.trim_between_curve_intersection_events(
            start_cutter.curve_string(),
            start_events,
            end_cutter.curve_string(),
            end_events,
            CurveStringCurveTrimQueryPath2::Prepared,
            policy,
        )
    }

    /// Retains portions of this prepared open curve string inside a prepared region.
    ///
    /// The region's prepared contour boxes are reused for boundary-hit
    /// collection and its prepared point classifier is reused for retained
    /// interval representatives. Exact segment intersections and native
    /// interval materialization remain delegated to the ordinary curve-string
    /// trim pipeline.
    pub fn trim_inside_prepared_region(
        &self,
        region: &PreparedRegionView2<'_>,
        policy: &CurvePolicy,
    ) -> CurveResult<CurveStringRegionTrimResult2> {
        self.curve.trim_inside_prepared_region(
            region,
            Some(curve_string_region_trim_prepared_cache_report(self, region)),
            policy,
        )
    }

    /// Retains portions of this prepared open curve string inside an ordinary region.
    pub fn trim_inside_region(
        &self,
        region: &Region2,
        policy: &CurvePolicy,
    ) -> CurveResult<CurveStringRegionTrimResult2> {
        let region = PreparedRegionView2::from_region(region, policy);
        self.trim_inside_prepared_region(&region, policy)
    }

    /// Classifies whether this prepared open curve string self-contacts.
    pub fn has_self_contacts(&self, policy: &CurvePolicy) -> CurveResult<Classification<bool>> {
        self.has_self_contacts_with_report(policy)
            .map(|result| result.has_self_contacts())
    }

    /// Classifies self contacts and retains cached broad-phase scan evidence.
    pub fn has_self_contacts_with_report(
        &self,
        policy: &CurvePolicy,
    ) -> CurveResult<crate::SelfContactResult2> {
        crate::self_intersect::segments_have_self_contacts_with_cached_aabbs_and_report(
            self.curve.segments(),
            &self.segment_boxes,
            false,
            policy,
        )
        .map(|result| {
            result.with_prepared_cache(prepared_self_contact_cache_report_for_curve(self))
        })
    }
}

/// A borrowed contour with cached contour and segment bounding boxes.
///
/// Prepared contours are useful when the same contour participates in many
/// topology queries. The cached boxes are conservative candidate filters only:
/// decided disjoint boxes skip a pair, while hits and uncertain boxes still run
/// the exact line/arc intersection code. This is the same broad-phase role that
/// Bentley and Ottmann assign to ordered geometric candidates in "Algorithms for
/// Reporting and Counting Geometric Intersections" (1979), kept here as a flat
/// pair scan until the crate grows a sweep-line index.
#[derive(Clone, Debug, PartialEq)]
pub struct PreparedContourView2<'a> {
    contour: &'a Contour2,
    prepared_segments: Vec<PreparedSegment2<'a>>,
    segment_boxes: Vec<Option<Aabb2>>,
    contour_box: Option<Aabb2>,
    facts: CurveStringFacts,
}

impl<'a> PreparedContourView2<'a> {
    /// Builds a prepared borrowed contour.
    pub fn from_contour(contour: &'a Contour2, policy: &CurvePolicy) -> Self {
        // Structural-dispatch note: contour preparation can preserve ring-level
        // facts such as convexity, orientation certainty, y-monotonicity, and
        // hole/material provenance for future triangulation and Boolean-region
        // dispatch without weakening the exact boundary classifiers.
        let segment_boxes = decided_segment_boxes(contour.segments(), policy);
        let contour_box = union_all_decided_boxes(segment_boxes.iter().map(Option::as_ref), policy);
        let facts = crate::facts::contour_facts(
            contour,
            segment_boxes.iter().filter(|bbox| bbox.is_some()).count(),
            contour_box.is_some(),
        );
        let prepared_segments = prepared_segments(contour.segments());

        Self {
            contour,
            prepared_segments,
            segment_boxes,
            contour_box,
            facts,
        }
    }

    /// Returns the borrowed source contour.
    pub const fn contour(&self) -> &'a Contour2 {
        self.contour
    }

    /// Returns the cached whole-contour box when every segment box was decided.
    pub const fn contour_box(&self) -> Option<&Aabb2> {
        self.contour_box.as_ref()
    }

    /// Returns cached segment boxes in source segment order.
    pub fn segment_boxes(&self) -> &[Option<Aabb2>] {
        &self.segment_boxes
    }

    /// Returns the number of prepared source segments.
    pub fn prepared_segment_count(&self) -> usize {
        self.prepared_segments.len()
    }

    /// Returns primitive-family counts for prepared source segments.
    pub fn prepared_segment_kind_counts(&self) -> SegmentKindCounts {
        prepared_segment_kind_counts(&self.prepared_segments)
    }

    /// Returns the number of segment boxes that were decided during preparation.
    pub fn decided_segment_box_count(&self) -> usize {
        self.segment_boxes
            .iter()
            .filter(|bbox| bbox.is_some())
            .count()
    }

    /// Returns the number of source segments whose preparation could not retain
    /// a decided broad-phase box.
    pub fn undecided_segment_box_count(&self) -> usize {
        self.segment_boxes.len() - self.decided_segment_box_count()
    }

    /// Returns prepared per-segment predicate handles in source segment order.
    pub fn prepared_segments(&self) -> &[PreparedSegment2<'a>] {
        &self.prepared_segments
    }

    /// Returns conservative structural facts collected while preparing.
    ///
    /// These facts are advisory scheduling metadata in Yap's object layer:
    /// Boolean and containment code can select specialized exact paths from
    /// them, but they are not a geometric decision by themselves.
    pub const fn facts(&self) -> &CurveStringFacts {
        &self.facts
    }

    /// Intersects two prepared contours using their cached broad-phase boxes.
    pub fn intersect_prepared_contour(
        &self,
        other: &PreparedContourView2<'_>,
        policy: &CurvePolicy,
    ) -> CurveResult<ContourIntersectionSet> {
        crate::events::intersect_contours_with_cached_aabbs(
            self.contour,
            other.contour,
            self.contour_box(),
            other.contour_box(),
            &self.segment_boxes,
            &other.segment_boxes,
            policy,
        )
    }

    /// Intersects this prepared contour against an ordinary borrowed contour.
    pub fn intersect_contour(
        &self,
        other: &Contour2,
        policy: &CurvePolicy,
    ) -> CurveResult<ContourIntersectionSet> {
        let other = PreparedContourView2::from_contour(other, policy);
        self.intersect_prepared_contour(&other, policy)
    }

    /// Collects self-intersection events using this contour's cached boxes.
    pub fn intersect_self(&self, policy: &CurvePolicy) -> CurveResult<ContourIntersectionSet> {
        crate::events::intersect_contour_self_with_cached_aabbs(
            self.contour,
            &self.segment_boxes,
            policy,
        )
    }

    /// Classifies a point against this prepared contour.
    pub fn classify_point(
        &self,
        point: &Point2,
        policy: &CurvePolicy,
    ) -> Classification<ContourPointLocation> {
        crate::contour::classify_contour_point_with_cached_aabbs(
            self.contour,
            point,
            self.contour_box(),
            &self.segment_boxes,
            policy,
        )
    }

    /// Returns true when the point lies on this prepared contour boundary.
    pub fn point_on_boundary(&self, point: &Point2, policy: &CurvePolicy) -> Classification<bool> {
        crate::contour::point_on_contour_boundary_with_cached_aabbs(
            self.contour,
            point,
            self.contour_box(),
            &self.segment_boxes,
            policy,
        )
    }

    /// Computes the winding number for a point not on this prepared boundary.
    pub fn winding_number(&self, point: &Point2, policy: &CurvePolicy) -> Classification<i32> {
        crate::contour::contour_winding_number_with_cached_aabbs(
            self.contour,
            point,
            self.contour_box(),
            &self.segment_boxes,
            policy,
        )
    }

    /// Classifies whether this prepared closed contour self-contacts.
    pub fn has_self_contacts(&self, policy: &CurvePolicy) -> CurveResult<Classification<bool>> {
        self.has_self_contacts_with_report(policy)
            .map(|result| result.has_self_contacts())
    }

    /// Classifies self contacts and retains cached broad-phase scan evidence.
    pub fn has_self_contacts_with_report(
        &self,
        policy: &CurvePolicy,
    ) -> CurveResult<crate::SelfContactResult2> {
        crate::self_intersect::segments_have_self_contacts_with_cached_aabbs_and_report(
            self.contour.segments(),
            &self.segment_boxes,
            true,
            policy,
        )
        .map(|result| {
            result.with_prepared_cache(prepared_self_contact_cache_report_for_contour(self))
        })
    }
}

/// A borrowed region view with cached contour and region bounding boxes.
///
/// This is useful when many points or intersection queries are run against the
/// same region. The cached boxes are only broad-phase filters: a decided point
/// miss contributes no depth, decided disjoint contour boxes skip intersection
/// candidates, and hits or uncertain boxes still run exact topology. Build the
/// prepared view with the same policy family used for later queries so arc
/// extrema and coordinate ordering are interpreted consistently.
#[derive(Clone, Debug, PartialEq)]
pub struct PreparedRegionView2<'a> {
    material_contours: Vec<&'a Contour2>,
    hole_contours: Vec<&'a Contour2>,
    material_prepared_contours: Vec<PreparedContourView2<'a>>,
    hole_prepared_contours: Vec<PreparedContourView2<'a>>,
    region_box: Option<Aabb2>,
    facts: RegionFacts,
}

impl<'a> PreparedRegionView2<'a> {
    /// Builds a prepared view from an owned region.
    pub fn from_region(region: &'a Region2, policy: &CurvePolicy) -> Self {
        Self::from_region_view(&region.as_view(), policy)
    }

    /// Builds a prepared view from a borrowed region view.
    pub fn from_region_view(region: &RegionView2<'a>, policy: &CurvePolicy) -> Self {
        let material_contours = region.material_contours().to_vec();
        let hole_contours = region.hole_contours().to_vec();
        let material_prepared_contours = prepared_contours(&material_contours, policy);
        let hole_prepared_contours = prepared_contours(&hole_contours, policy);
        let region_box = union_all_decided_boxes(
            material_prepared_contours
                .iter()
                .chain(hole_prepared_contours.iter())
                .map(PreparedContourView2::contour_box),
            policy,
        );
        let facts = crate::facts::region_view_facts(region, region_box.is_some());

        Self {
            material_contours,
            hole_contours,
            material_prepared_contours,
            hole_prepared_contours,
            region_box,
            facts,
        }
    }

    /// Returns the cached whole-region box when every contour box was decided.
    pub const fn region_box(&self) -> Option<&Aabb2> {
        self.region_box.as_ref()
    }

    /// Returns material contours in the prepared view.
    pub fn material_contours(&self) -> &[&'a Contour2] {
        &self.material_contours
    }

    /// Returns hole contours in the prepared view.
    pub fn hole_contours(&self) -> &[&'a Contour2] {
        &self.hole_contours
    }

    /// Reconstructs a borrowed ordinary region view over the same contours.
    ///
    /// The returned view is cheap and keeps the same contour lifetimes. It is
    /// useful when an algorithm still needs the canonical `RegionView2` shape
    /// for splitting or cloning, while prepared classifiers supply repeated
    /// point and event queries.
    pub fn as_region_view(&self) -> RegionView2<'a> {
        RegionView2::from_contours(
            self.material_contours.iter().copied(),
            self.hole_contours.iter().copied(),
        )
    }

    /// Returns prepared material contours in region-bin order.
    pub fn prepared_material_contours(&self) -> &[PreparedContourView2<'a>] {
        &self.material_prepared_contours
    }

    /// Returns prepared hole contours in region-bin order.
    pub fn prepared_hole_contours(&self) -> &[PreparedContourView2<'a>] {
        &self.hole_prepared_contours
    }

    /// Returns the number of prepared material and hole contours.
    pub fn prepared_contour_count(&self) -> usize {
        self.material_prepared_contours.len() + self.hole_prepared_contours.len()
    }

    /// Returns the number of prepared material source segments.
    pub fn prepared_material_segment_count(&self) -> usize {
        self.material_prepared_contours
            .iter()
            .map(PreparedContourView2::prepared_segment_count)
            .sum()
    }

    /// Returns primitive-family counts for prepared material source segments.
    pub fn prepared_material_segment_kind_counts(&self) -> SegmentKindCounts {
        prepared_contour_kind_counts(&self.material_prepared_contours)
    }

    /// Returns the number of prepared hole source segments.
    pub fn prepared_hole_segment_count(&self) -> usize {
        self.hole_prepared_contours
            .iter()
            .map(PreparedContourView2::prepared_segment_count)
            .sum()
    }

    /// Returns primitive-family counts for prepared hole source segments.
    pub fn prepared_hole_segment_kind_counts(&self) -> SegmentKindCounts {
        prepared_contour_kind_counts(&self.hole_prepared_contours)
    }

    /// Returns the number of prepared material and hole source segments.
    pub fn prepared_segment_count(&self) -> usize {
        self.prepared_material_segment_count() + self.prepared_hole_segment_count()
    }

    /// Returns primitive-family counts for all prepared source segments.
    pub fn prepared_segment_kind_counts(&self) -> SegmentKindCounts {
        let mut counts = self.prepared_material_segment_kind_counts();
        let hole_counts = self.prepared_hole_segment_kind_counts();
        counts.lines += hole_counts.lines;
        counts.arcs += hole_counts.arcs;
        counts
    }

    /// Returns the number of material contour segment boxes decided during preparation.
    pub fn decided_material_segment_box_count(&self) -> usize {
        self.material_prepared_contours
            .iter()
            .map(PreparedContourView2::decided_segment_box_count)
            .sum()
    }

    /// Returns the number of hole contour segment boxes decided during preparation.
    pub fn decided_hole_segment_box_count(&self) -> usize {
        self.hole_prepared_contours
            .iter()
            .map(PreparedContourView2::decided_segment_box_count)
            .sum()
    }

    /// Returns the number of retained contour segment boxes decided during preparation.
    pub fn decided_segment_box_count(&self) -> usize {
        self.decided_material_segment_box_count() + self.decided_hole_segment_box_count()
    }

    /// Returns the number of material source contour segments whose boxes stayed undecided.
    pub fn undecided_material_segment_box_count(&self) -> usize {
        self.material_prepared_contours
            .iter()
            .map(PreparedContourView2::undecided_segment_box_count)
            .sum()
    }

    /// Returns the number of hole source contour segments whose boxes stayed undecided.
    pub fn undecided_hole_segment_box_count(&self) -> usize {
        self.hole_prepared_contours
            .iter()
            .map(PreparedContourView2::undecided_segment_box_count)
            .sum()
    }

    /// Returns the number of source contour segments whose boxes stayed undecided.
    pub fn undecided_segment_box_count(&self) -> usize {
        self.undecided_material_segment_box_count() + self.undecided_hole_segment_box_count()
    }

    /// Returns conservative structural facts collected while preparing.
    ///
    /// Structural-dispatch note: this is where future region-level convexity,
    /// contour orientation certainty, all-line/all-arc partitioning, common
    /// scales, and symbolic dependencies should be shared with Boolean and
    /// containment algorithms without leaking scalar representation details.
    pub const fn facts(&self) -> &RegionFacts {
        &self.facts
    }

    /// Classifies a point against this prepared region view.
    pub fn classify_point(
        &self,
        point: &Point2,
        policy: &CurvePolicy,
    ) -> Classification<RegionPointLocation> {
        let depth = match self.signed_depth(point, policy) {
            Classification::Decided(depth) => depth,
            Classification::Uncertain(UncertaintyReason::Boundary) => {
                return Classification::Decided(RegionPointLocation::Boundary);
            }
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        };

        Classification::Decided(if depth > 0 {
            RegionPointLocation::Inside
        } else {
            RegionPointLocation::Outside
        })
    }

    /// Returns signed containment depth for a non-boundary point.
    ///
    /// This follows the same signed material-minus-hole convention as
    /// [`RegionView2::signed_depth`]. Decided cached-box misses are skipped, then
    /// candidate contours are classified with the boundary-first winding
    /// structure described by Hormann and Agathos, "The Point in Polygon Problem
    /// for Arbitrary Polygons" (2001), with this crate's circular-arc extension.
    pub fn signed_depth(&self, point: &Point2, policy: &CurvePolicy) -> Classification<i32> {
        if self
            .region_box
            .as_ref()
            .is_some_and(|bbox| aabb_decided_misses_point(bbox, point, policy))
        {
            return Classification::Decided(0);
        }

        let mut depth = 0;
        match accumulate_depth(
            &mut depth,
            &self.material_prepared_contours,
            point,
            1,
            policy,
        ) {
            Classification::Decided(()) => {}
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        }
        match accumulate_depth(&mut depth, &self.hole_prepared_contours, point, -1, policy) {
            Classification::Decided(()) => Classification::Decided(depth),
            Classification::Uncertain(reason) => Classification::Uncertain(reason),
        }
    }

    /// Collects normalized topology events against another prepared region.
    ///
    /// This reuses cached contour and segment boxes for the candidate phase and
    /// then delegates candidate pairs to the same exact line/arc intersection
    /// normalization as [`RegionView2::intersect_region`]. The cache changes the
    /// amount of repeated broad-phase work, not the topology contract.
    pub fn intersect_prepared_region(
        &self,
        other: &PreparedRegionView2<'_>,
        policy: &CurvePolicy,
    ) -> CurveResult<RegionIntersectionSet> {
        let mut pairs = Vec::new();
        let mut workload = RegionIntersectionWorkload::default();

        collect_prepared_role_pairs(
            &mut pairs,
            &mut workload,
            &self.material_prepared_contours,
            RegionContourRole::Material,
            &other.material_prepared_contours,
            RegionContourRole::Material,
            policy,
        )?;
        collect_prepared_role_pairs(
            &mut pairs,
            &mut workload,
            &self.material_prepared_contours,
            RegionContourRole::Material,
            &other.hole_prepared_contours,
            RegionContourRole::Hole,
            policy,
        )?;
        collect_prepared_role_pairs(
            &mut pairs,
            &mut workload,
            &self.hole_prepared_contours,
            RegionContourRole::Hole,
            &other.material_prepared_contours,
            RegionContourRole::Material,
            policy,
        )?;
        collect_prepared_role_pairs(
            &mut pairs,
            &mut workload,
            &self.hole_prepared_contours,
            RegionContourRole::Hole,
            &other.hole_prepared_contours,
            RegionContourRole::Hole,
            policy,
        )?;

        RegionIntersectionSet::from_parts(
            pairs,
            Some(self.prepared_contour_count()),
            Some(other.prepared_contour_count()),
            workload.candidate_pair_count,
            workload.skipped_aabb_pair_count,
            workload.tested_pair_count,
        )
    }

    /// Collects normalized topology events against an ordinary region view.
    pub fn intersect_region(
        &self,
        other: &RegionView2<'_>,
        policy: &CurvePolicy,
    ) -> CurveResult<RegionIntersectionSet> {
        let other = PreparedRegionView2::from_region_view(other, policy);
        self.intersect_prepared_region(&other, policy)
    }

    /// Computes closed boolean boundary loops against another prepared region.
    ///
    /// This prepared path runs the same split, classify, and boundary-chain
    /// traversal as [`RegionView2::boolean_boundary_loops`], but reuses cached
    /// region/contour boxes during event collection and fragment midpoint
    /// classification. Greiner and Hormann describe closed boundary traversal
    /// after intersection insertion and entry/exit classification (G. Greiner
    /// and K. Hormann, "Efficient clipping of arbitrary polygons," 1998);
    /// Martinez, Rueda, and Feito describe boolean selection from classified
    /// segments (F. Martinez, A. J. Rueda, and F. R. Feito, "A new algorithm
    /// for computing Boolean operations on polygons," 2009). Cached boxes only
    /// prune decided misses, so boundary and overlap uncertainty is preserved.
    pub fn boolean_boundary_loops(
        &self,
        other: &PreparedRegionView2<'_>,
        op: BooleanOp,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<BooleanBoundaryLoopSet>> {
        crate::prepared_boolean::boolean_boundary_loops_between_prepared(self, other, op, policy)
    }

    /// Computes closed boolean boundary loops against an ordinary region view.
    ///
    /// This is a mixed prepared/unprepared convenience path: the left operand's
    /// cache is reused, the right operand is prepared for this call, and the
    /// prepared-prepared traversal described in
    /// [`PreparedRegionView2::boolean_boundary_loops`] remains authoritative.
    /// The transient right-side cache follows the same candidate-pruning role
    /// as Bentley and Ottmann's broad-phase intersection reporting setup, while
    /// the final boundary traversal still follows the Greiner-Hormann and
    /// Martinez-Rueda-Feito split/classify/assemble model cited above.
    pub fn boolean_boundary_loops_against_region(
        &self,
        other: &RegionView2<'_>,
        op: BooleanOp,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<BooleanBoundaryLoopSet>> {
        let other = PreparedRegionView2::from_region_view(other, policy);
        self.boolean_boundary_loops(&other, op, policy)
    }

    /// Computes checked boolean boundary contours against another prepared
    /// region.
    ///
    /// This extends [`PreparedRegionView2::boolean_boundary_loops`] through the
    /// same checked-contour conversion and regularized contact fast paths used
    /// by [`RegionView2::boolean_boundary_contours`]. The prepared parts remain
    /// candidate filters only: Foster, Hormann, and Popa's degenerate
    /// clipping cases still surface as explicit boundary handling rather than
    /// as tolerance-based inside/outside choices.
    pub fn boolean_boundary_contours(
        &self,
        other: &PreparedRegionView2<'_>,
        op: BooleanOp,
        fill_rule: FillRule,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<Vec<Contour2>>> {
        crate::prepared_boolean::boolean_boundary_contours_between_prepared(
            self, other, op, fill_rule, policy,
        )
    }

    /// Computes checked boolean boundary contours against an ordinary region
    /// view.
    ///
    /// This prepares the right operand only for the duration of the call and
    /// then uses [`PreparedRegionView2::boolean_boundary_contours`]. Keeping the
    /// wrapper explicit makes one-prepared/many-unprepared workloads ergonomic
    /// without weakening the degenerate clipping behavior described by Foster,
    /// Hormann, and Popa for boundary contacts.
    pub fn boolean_boundary_contours_against_region(
        &self,
        other: &RegionView2<'_>,
        op: BooleanOp,
        fill_rule: FillRule,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<Vec<Contour2>>> {
        let other = PreparedRegionView2::from_region_view(other, policy);
        self.boolean_boundary_contours(&other, op, fill_rule, policy)
    }

    /// Computes a role-assigned boolean region against another prepared region.
    ///
    /// This is the prepared analogue of [`RegionView2::boolean_region`]. It
    /// reuses cached event and point-classification broad phases before
    /// returning to the ordinary contour-nesting pass for final material/hole
    /// assignment, preserving the Vatti-style fill-state semantics already used
    /// by the non-prepared region pipeline.
    pub fn boolean_region(
        &self,
        other: &PreparedRegionView2<'_>,
        op: BooleanOp,
        fill_rule: FillRule,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<Region2>> {
        crate::prepared_boolean::boolean_region_between_prepared(self, other, op, fill_rule, policy)
    }

    /// Computes a role-assigned boolean region and retains materialization evidence.
    ///
    /// This is the report-bearing counterpart to
    /// [`PreparedRegionView2::boolean_region`]. Prepared caches still only
    /// prune candidates; the final report comes from checked boundary contours
    /// and exact contour nesting.
    pub fn boolean_region_with_report(
        &self,
        other: &PreparedRegionView2<'_>,
        op: BooleanOp,
        fill_rule: FillRule,
        policy: &CurvePolicy,
    ) -> CurveResult<RegionBooleanResult2> {
        crate::prepared_boolean::boolean_region_between_prepared_with_report(
            self, other, op, fill_rule, policy,
        )
    }

    /// Computes a role-assigned boolean region against an ordinary region view.
    ///
    /// The right operand is prepared transiently, after which the same prepared
    /// boolean-region path assigns resolved contours to material and hole bins.
    /// The nesting step remains the Hormann-Agathos boundary-first point
    /// classification used by [`RegionView2::boolean_region`].
    pub fn boolean_region_against_region(
        &self,
        other: &RegionView2<'_>,
        op: BooleanOp,
        fill_rule: FillRule,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<Region2>> {
        let other = PreparedRegionView2::from_region_view(other, policy);
        self.boolean_region(&other, op, fill_rule, policy)
    }

    /// Computes a report-bearing boolean region against an ordinary region view.
    pub fn boolean_region_with_report_against_region(
        &self,
        other: &RegionView2<'_>,
        op: BooleanOp,
        fill_rule: FillRule,
        policy: &CurvePolicy,
    ) -> CurveResult<RegionBooleanResult2> {
        let other = PreparedRegionView2::from_region_view(other, policy);
        self.boolean_region_with_report(&other, op, fill_rule, policy)
    }
}

impl CurveString2 {
    /// Builds a prepared borrowed curve string for repeated topology queries.
    pub fn prepare_topology_queries(&self, policy: &CurvePolicy) -> PreparedCurveStringView2<'_> {
        PreparedCurveStringView2::from_curve_string(self, policy)
    }
}

impl Contour2 {
    /// Builds a prepared borrowed contour for repeated topology queries.
    pub fn prepare_topology_queries(&self, policy: &CurvePolicy) -> PreparedContourView2<'_> {
        PreparedContourView2::from_contour(self, policy)
    }
}

impl Region2 {
    /// Builds a prepared borrowed view for repeated point classification.
    pub fn prepare_point_classifier(&self, policy: &CurvePolicy) -> PreparedRegionView2<'_> {
        PreparedRegionView2::from_region(self, policy)
    }

    /// Builds a prepared borrowed view for repeated point and event queries.
    pub fn prepare_topology_queries(&self, policy: &CurvePolicy) -> PreparedRegionView2<'_> {
        PreparedRegionView2::from_region(self, policy)
    }
}

impl<'a> RegionView2<'a> {
    /// Builds a prepared borrowed view for repeated point classification.
    pub fn prepare_point_classifier(&self, policy: &CurvePolicy) -> PreparedRegionView2<'a> {
        PreparedRegionView2::from_region_view(self, policy)
    }

    /// Builds a prepared borrowed view for repeated point and event queries.
    pub fn prepare_topology_queries(&self, policy: &CurvePolicy) -> PreparedRegionView2<'a> {
        PreparedRegionView2::from_region_view(self, policy)
    }

    /// Collects normalized topology events against a prepared right operand.
    ///
    /// This preserves operand order for callers that have already prepared the
    /// second region. The left view is prepared transiently, then the prepared
    /// event collector uses cached broad-phase boxes before exact intersection
    /// normalization.
    pub fn intersect_prepared_region(
        &self,
        other: &PreparedRegionView2<'_>,
        policy: &CurvePolicy,
    ) -> CurveResult<RegionIntersectionSet> {
        let this = PreparedRegionView2::from_region_view(self, policy);
        this.intersect_prepared_region(other, policy)
    }

    /// Computes closed boolean boundary loops against a prepared right operand.
    ///
    /// Use this when the right operand is reused across many ordinary region
    /// views, especially for non-commutative operations such as difference. The
    /// transient left cache only prunes decided misses; Greiner-Hormann style
    /// boundary traversal and Martinez-Rueda-Feito fragment selection remain
    /// unchanged from [`RegionView2::boolean_boundary_loops`].
    pub fn boolean_boundary_loops_against_prepared_region(
        &self,
        other: &PreparedRegionView2<'_>,
        op: BooleanOp,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<BooleanBoundaryLoopSet>> {
        let this = PreparedRegionView2::from_region_view(self, policy);
        this.boolean_boundary_loops(other, op, policy)
    }

    /// Computes checked boolean boundary contours against a prepared right
    /// operand.
    ///
    /// The operation order is `self op other`; the prepared right operand is not
    /// swapped to the left. Degenerate shared-boundary cases keep the same
    /// explicit Foster-Hormann-Popa style uncertainty/regularization behavior
    /// as the ordinary checked-contour API.
    pub fn boolean_boundary_contours_against_prepared_region(
        &self,
        other: &PreparedRegionView2<'_>,
        op: BooleanOp,
        fill_rule: FillRule,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<Vec<Contour2>>> {
        let this = PreparedRegionView2::from_region_view(self, policy);
        this.boolean_boundary_contours(other, op, fill_rule, policy)
    }

    /// Computes a role-assigned boolean region against a prepared right
    /// operand.
    ///
    /// The prepared path still returns to the ordinary nesting classifier for
    /// material/hole assignment, so Hormann-Agathos boundary-first point
    /// classification remains the final arbiter for resolved output contours.
    pub fn boolean_region_against_prepared_region(
        &self,
        other: &PreparedRegionView2<'_>,
        op: BooleanOp,
        fill_rule: FillRule,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<Region2>> {
        let this = PreparedRegionView2::from_region_view(self, policy);
        this.boolean_region(other, op, fill_rule, policy)
    }

    /// Computes a report-bearing boolean region against a prepared right operand.
    pub fn boolean_region_with_report_against_prepared_region(
        &self,
        other: &PreparedRegionView2<'_>,
        op: BooleanOp,
        fill_rule: FillRule,
        policy: &CurvePolicy,
    ) -> CurveResult<RegionBooleanResult2> {
        let this = PreparedRegionView2::from_region_view(self, policy);
        this.boolean_region_with_report(other, op, fill_rule, policy)
    }
}

fn prepared_contours<'a>(
    contours: &[&'a Contour2],
    policy: &CurvePolicy,
) -> Vec<PreparedContourView2<'a>>
where
{
    contours
        .iter()
        .map(|contour| PreparedContourView2::from_contour(contour, policy))
        .collect()
}

fn prepared_contour_kind_counts(contours: &[PreparedContourView2<'_>]) -> SegmentKindCounts {
    let mut counts = SegmentKindCounts::default();
    for contour in contours {
        let contour_counts = prepared_segment_kind_counts(contour.prepared_segments());
        counts.lines += contour_counts.lines;
        counts.arcs += contour_counts.arcs;
    }
    counts
}

fn decided_segment_boxes(segments: &[crate::Segment2], policy: &CurvePolicy) -> Vec<Option<Aabb2>> {
    segments
        .iter()
        .map(|segment| decided_segment_aabb(segment, policy))
        .collect()
}

fn prepared_segments(segments: &[Segment2]) -> Vec<PreparedSegment2<'_>> {
    segments
        .iter()
        .map(PreparedSegment2::from_segment)
        .collect()
}

fn prepared_segment_kind_counts(segments: &[PreparedSegment2<'_>]) -> SegmentKindCounts {
    let mut counts = SegmentKindCounts::default();
    for segment in segments {
        if segment.is_line() {
            counts.lines += 1;
        } else if segment.is_arc() {
            counts.arcs += 1;
        }
    }
    counts
}

fn intersect_prepared_segment_pairs_with_cached_aabbs(
    first_prepared_segments: &[PreparedSegment2<'_>],
    second_prepared_segments: &[PreparedSegment2<'_>],
    first_segment_boxes: &[Option<Aabb2>],
    second_segment_boxes: &[Option<Aabb2>],
    query_path: CurveStringIntersectionQueryPath2,
    prepared_cache_report: Option<CurveStringIntersectionPreparedCacheReport2>,
    policy: &CurvePolicy,
) -> CurveResult<CurveStringIntersectionResult2> {
    let mut intersections = Vec::new();
    let candidate_pair_count = first_prepared_segments.len() * second_prepared_segments.len();
    let mut skipped_aabb_pair_count = 0_usize;
    let mut tested_pair_count = 0_usize;

    for (a_segment_index, a_segment) in first_prepared_segments.iter().enumerate() {
        for (b_segment_index, b_segment) in second_prepared_segments.iter().enumerate() {
            // Prepared pair batches use the same conservative broad phase as
            // ordinary curve strings. Bentley and Ottmann's sweep-line paper
            // motivates candidate pruning, but the prepared flat scan keeps
            // exact segment relations authoritative until a later index can
            // consume retained all-line/axis/monotone facts.
            if let (Some(Some(a_box)), Some(Some(b_box))) = (
                first_segment_boxes.get(a_segment_index),
                second_segment_boxes.get(b_segment_index),
            ) && aabbs_decided_disjoint(a_box, b_box, policy)
            {
                skipped_aabb_pair_count += 1;
                continue;
            }

            tested_pair_count += 1;
            let relation = match (a_segment, b_segment) {
                (PreparedSegment2::Line(_), PreparedSegment2::Line(_))
                | (PreparedSegment2::Line(_), PreparedSegment2::Arc(_))
                | (PreparedSegment2::Arc(_), PreparedSegment2::Line(_))
                | (PreparedSegment2::Arc(_), PreparedSegment2::Arc(_)) => {
                    a_segment.intersect_prepared_segment(b_segment, policy)?
                }
            };

            if !relation.is_none() {
                intersections.push(CurveStringIntersection {
                    a_segment_index,
                    b_segment_index,
                    a_segment_kind: a_segment.segment_kind(),
                    b_segment_kind: b_segment.segment_kind(),
                    a_segment_start_point: a_segment.start().clone(),
                    a_segment_end_point: a_segment.end().clone(),
                    b_segment_start_point: b_segment.start().clone(),
                    b_segment_end_point: b_segment.end().clone(),
                    relation,
                });
            }
        }
    }

    let intersection_count = intersections.len();
    let relation_counts = curve_string_intersection_relation_counts(&intersections);
    let first_decided_segment_box_count = decided_segment_box_count(first_segment_boxes);
    let second_decided_segment_box_count = decided_segment_box_count(second_segment_boxes);
    let first_undecided_segment_box_count = first_prepared_segments
        .len()
        .saturating_sub(first_decided_segment_box_count);
    let second_undecided_segment_box_count = second_prepared_segments
        .len()
        .saturating_sub(second_decided_segment_box_count);
    Ok(CurveStringIntersectionResult2::from_parts(
        intersections,
        CurveStringIntersectionReport2::new_native_exact(
            first_prepared_segments.len(),
            second_prepared_segments.len(),
            prepared_segment_kind_counts(first_prepared_segments),
            prepared_segment_kind_counts(second_prepared_segments),
            first_decided_segment_box_count,
            second_decided_segment_box_count,
            first_undecided_segment_box_count,
            second_undecided_segment_box_count,
            candidate_pair_count,
            skipped_aabb_pair_count,
            tested_pair_count,
            intersection_count,
            relation_counts.point,
            relation_counts.overlap,
            relation_counts.uncertain,
            query_path,
            prepared_cache_report,
        ),
    ))
}

fn curve_string_intersection_prepared_cache_report(
    first: &PreparedCurveStringView2<'_>,
    second: &PreparedCurveStringView2<'_>,
) -> CurveStringIntersectionPreparedCacheReport2 {
    CurveStringIntersectionPreparedCacheReport2::new(
        prepared_curve_string_cache_audit(first),
        prepared_curve_string_cache_audit(second),
    )
}

fn prepared_curve_string_cache_audit(
    curve: &PreparedCurveStringView2<'_>,
) -> CurveStringPreparedCacheAudit2 {
    CurveStringPreparedCacheAudit2::new(
        curve.prepared_segment_count(),
        curve.prepared_segment_kind_counts(),
        curve.decided_segment_box_count(),
        curve.undecided_segment_box_count(),
        curve.curve_box().is_some(),
    )
}

fn prepared_self_contact_cache_report_for_curve(
    curve: &PreparedCurveStringView2<'_>,
) -> crate::SelfContactPreparedCacheReport2 {
    crate::SelfContactPreparedCacheReport2::new(
        curve.prepared_segment_count(),
        curve.prepared_segment_kind_counts(),
        curve.decided_segment_box_count(),
        curve.undecided_segment_box_count(),
        curve.curve_box().is_some(),
    )
}

fn prepared_self_contact_cache_report_for_contour(
    contour: &PreparedContourView2<'_>,
) -> crate::SelfContactPreparedCacheReport2 {
    crate::SelfContactPreparedCacheReport2::new(
        contour.prepared_segment_count(),
        contour.prepared_segment_kind_counts(),
        contour.decided_segment_box_count(),
        contour.undecided_segment_box_count(),
        contour.contour_box().is_some(),
    )
}

fn curve_string_region_trim_prepared_cache_report(
    source: &PreparedCurveStringView2<'_>,
    region: &PreparedRegionView2<'_>,
) -> CurveStringRegionTrimPreparedCacheReport2 {
    CurveStringRegionTrimPreparedCacheReport2::new(
        prepared_curve_string_cache_audit(source),
        prepared_region_trim_cache_audit(region),
    )
}

fn prepared_region_trim_cache_audit(
    region: &PreparedRegionView2<'_>,
) -> RegionTrimPreparedCacheAudit2 {
    RegionTrimPreparedCacheAudit2::new(
        region.prepared_contour_count(),
        region.prepared_material_segment_count(),
        region.prepared_material_segment_kind_counts(),
        region.prepared_hole_segment_count(),
        region.prepared_hole_segment_kind_counts(),
        region.prepared_segment_count(),
        region.prepared_segment_kind_counts(),
        region.decided_segment_box_count(),
        region.undecided_segment_box_count(),
        region.region_box().is_some(),
    )
}

#[cfg(feature = "predicates")]
fn predicate_point(point: &Point2) -> hyperlimit::Point2 {
    hyperlimit::Point2::new(point.x().clone(), point.y().clone())
}

#[cfg(feature = "predicates")]
fn classify_prepared_line(
    from: &hyperlimit::Point2,
    to: &hyperlimit::Point2,
    facts: hyperlimit::PreparedPredicateFacts,
    point: &hyperlimit::Point2,
    policy: &CurvePolicy,
) -> Classification<LineSide> {
    let prepared = hyperlimit::PreparedLine2::from_facts(from, to, facts);
    match prepared.classify_point_with_policy(point, policy.predicate_policy) {
        hyperlimit::PredicateOutcome::Decided { value, .. } => {
            Classification::Decided(line_side_from_hyperlimit(value))
        }
        hyperlimit::PredicateOutcome::Unknown { .. } => {
            Classification::Uncertain(UncertaintyReason::Predicate)
        }
    }
}

#[cfg(feature = "predicates")]
const fn line_side_from_hyperlimit(side: hyperlimit::LineSide) -> LineSide {
    match side {
        hyperlimit::LineSide::Left => LineSide::Left,
        hyperlimit::LineSide::Right => LineSide::Right,
        hyperlimit::LineSide::On => LineSide::On,
    }
}

fn union_all_decided_boxes<'a, I>(boxes: I, policy: &CurvePolicy) -> Option<Aabb2>
where
    I: IntoIterator<Item = Option<&'a Aabb2>>,
{
    let mut boxes = boxes.into_iter();
    let first = boxes.next()??.clone();
    let mut merged = first;

    for bbox in boxes {
        let bbox = bbox?;
        let Classification::Decided(next) = merged.union(bbox, policy) else {
            return None;
        };
        merged = next;
    }

    Some(merged)
}

fn accumulate_depth(
    depth: &mut i32,
    contours: &[PreparedContourView2<'_>],
    point: &Point2,
    sign: i32,
    policy: &CurvePolicy,
) -> Classification<()> {
    for contour in contours {
        if contour
            .contour_box()
            .is_some_and(|bbox| aabb_decided_misses_point(bbox, point, policy))
        {
            continue;
        }

        match contour.classify_point(point, policy) {
            Classification::Decided(ContourPointLocation::Inside) => *depth += sign,
            Classification::Decided(ContourPointLocation::Outside) => {}
            Classification::Decided(ContourPointLocation::Boundary) => {
                return Classification::Uncertain(UncertaintyReason::Boundary);
            }
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        }
    }

    Classification::Decided(())
}

fn collect_prepared_role_pairs(
    pairs: &mut Vec<RegionContourIntersection>,
    workload: &mut RegionIntersectionWorkload,
    first_contours: &[PreparedContourView2<'_>],
    first_role: RegionContourRole,
    second_contours: &[PreparedContourView2<'_>],
    second_role: RegionContourRole,
    policy: &CurvePolicy,
) -> CurveResult<()> {
    for (first_index, first_contour) in first_contours.iter().enumerate() {
        for (second_index, second_contour) in second_contours.iter().enumerate() {
            workload.candidate_pair_count += 1;
            if let (Some(first_box), Some(second_box)) =
                (first_contour.contour_box(), second_contour.contour_box())
                && aabbs_decided_disjoint(first_box, second_box, policy)
            {
                workload.skipped_aabb_pair_count += 1;
                continue;
            }

            workload.tested_pair_count += 1;
            let intersections = first_contour.intersect_prepared_contour(second_contour, policy)?;
            if intersections.is_empty() {
                continue;
            }

            pairs.push(RegionContourIntersection {
                first: RegionContourKey::new(RegionSide::First, first_role, first_index),
                second: RegionContourKey::new(RegionSide::Second, second_role, second_index),
                intersections,
            });
        }
    }

    Ok(())
}
