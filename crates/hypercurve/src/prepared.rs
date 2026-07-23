//! Retained borrowed query structures for repeated topology classification.
//!
//! Retained views cache conservative broad-phase data but do not replace exact
//! topology. They skip only decided bounding-box misses and then delegate to the
//! same segment-intersection and boundary-first contour classification used by
//! ordinary contours and regions. This keeps query construction in the candidate
//! generation role of sweep-line scheduling intersection-event framework,
//! while preserving certified predicates for topology branches.

use std::cmp::Ordering;

use crate::bbox::{
    Aabb2, aabb_decided_misses_point, aabb_decided_strictly_right_of_point, aabbs_decided_disjoint,
    decided_segment_aabb,
};
use crate::curve_string::{CurveStringXOverlapCandidates, curve_string_x_overlap_schedule};
use crate::events::SegmentAabbXIndex;
use crate::facts::{CurveStringFacts, RegionFacts};
use crate::region_events::RegionIntersectionWorkload;
use crate::{
    BooleanBoundaryLoopSet, BooleanOp, CircularArc2, CircularArc2Facts, Classification, Contour2,
    ContourIntersectionSet, ContourPointLocation, CurvePolicy, CurveResult, CurveString2,
    CurveStringIntersection, FillRule, LineArcRegion2, LineSeg2, LineSeg2Facts, LineSide, Point2,
    RegionContourIntersection, RegionContourKey, RegionContourRole, RegionIntersectionSet,
    RegionPointLocation, RegionSide, RegionView2, Segment2, SegmentIntersection, SegmentKind,
    SegmentKindCounts, UncertaintyReason,
};

/// Retained point-line classifier for a fixed [`LineSeg2`].
///
/// This view caches the segment's structural facts and, when the `predicates`
/// feature is enabled, the converted `hyperlimit` endpoints used by repeated
/// orientation tests. It deliberately does not own finite-segment containment
/// semantics: those remain on [`LineSeg2`], while this type accelerates the
/// exact supporting-line predicate. That split follows the exactness model's EGC model of
/// carrying object structure forward without moving combinatorial decisions
/// out of the predicate layer.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct PreparedLineSeg2<'a> {
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
    /// Builds a borrowed query for a line segment.
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

    /// Classifies a point relative to this segment's oriented supporting line.
    pub fn classify_point(&self, point: &Point2, policy: &CurvePolicy) -> Classification<LineSide> {
        #[cfg(feature = "predicates")]
        if !matches!(policy.numeric_mode, crate::NumericMode::EdgePreview) {
            // Reuse the fixed endpoint conversion and query facts, then let
            // hyperlimit select the exact determinant schedule. This is the
            // certified orientation predicate at the curve-object
            // boundary, with the exactness model's exact/approximate split preserved by
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

/// Retained sweep and circle classifier for a fixed [`CircularArc2`].
///
/// The query arc stores the two radial oriented lines that bound the arc
/// sweep. Point-on-arc checks still compare exact squared radius first, then
/// use those query radial predicates for angular containment. This mirrors
/// the standard circle/arc primitive decomposition while preserving
/// the exactness model's EGC split between exact topology predicates and approximate output
/// adapters. See standard geometric constructions, and exact-computation discipline.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct PreparedCircularArc2<'a> {
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
    /// Builds a borrowed query for a circular arc.
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

    /// Classifies whether a point lies inside this arc's angular sweep.
    pub fn contains_sweep_point(
        &self,
        point: &Point2,
        policy: &CurvePolicy,
    ) -> Classification<bool> {
        #[cfg(feature = "predicates")]
        if !matches!(policy.numeric_mode, crate::NumericMode::EdgePreview) {
            let sweep_kind = match crate::arc_bezier::classify_sweep(self.arc) {
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

/// Retained exact-predicate handle for a native segment.
///
/// This enum mirrors [`Segment2`] at the query-object layer. It gives curve
/// strings and contours a place to retain per-segment line/arc predicate
/// handles discovered during query construction, while keeping segment topology owned
/// by `hypercurve` and scalar/predicate decisions owned by `hyperlimit`.
#[derive(Clone, Debug, PartialEq)]
#[allow(clippy::large_enum_variant)]
pub(crate) enum PreparedSegment2<'a> {
    /// Retained line-segment predicates.
    Line(PreparedLineSeg2<'a>),
    /// Retained circular-arc predicates.
    Arc(PreparedCircularArc2<'a>),
}

impl<'a> PreparedSegment2<'a> {
    /// Builds a borrowed query for a segment handle.
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

    /// Returns the primitive family query by this segment handle.
    pub const fn segment_kind(&self) -> SegmentKind {
        match self {
            Self::Line(_) => SegmentKind::Line,
            Self::Arc(_) => SegmentKind::Arc,
        }
    }

    /// Returns the exact start point of the query source segment.
    pub fn start(&self) -> &Point2 {
        match self {
            Self::Line(line) => line.line_segment().start(),
            Self::Arc(arc) => arc.circular_arc().start(),
        }
    }

    /// Returns the exact end point of the query source segment.
    pub fn end(&self) -> &Point2 {
        match self {
            Self::Line(line) => line.line_segment().end(),
            Self::Arc(arc) => arc.circular_arc().end(),
        }
    }

    /// Classifies whether a point lies on this finite query segment.
    pub fn contains_point(&self, point: &Point2, policy: &CurvePolicy) -> Classification<bool> {
        match self {
            Self::Line(line) => {
                let side = match line.classify_point(point, policy) {
                    Classification::Decided(side) => side,
                    Classification::Uncertain(reason) => {
                        return Classification::Uncertain(reason);
                    }
                };
                line.line_segment()
                    .contains_point_with_classified_side(point, side, policy)
            }
            Self::Arc(arc) => arc.contains_point(point, policy),
        }
    }

    /// Intersects two query native segment handles.
    ///
    /// This is the query segment-pair batch boundary used by query curve
    /// strings and contours. It deliberately returns the same
    /// [`SegmentIntersection`] shape as [`Segment2::intersect_segment`]: cached
    /// line and arc facts can select faster exact kernels, but finite segment
    /// topology and uncertainty remain represented by `hypercurve`'s public
    /// intersection enums. This follows the exactness model's EGC separation between carried
    /// object facts and certified predicate decisions.
    pub(crate) fn intersect_prepared_segment(
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
/// Retained curve strings avoid rebuilding broad-phase boxes for repeated
/// topology queries. The cache never decides a contact on its own: it skips only
/// decided disjoint boxes and keeps exact line/arc intersections authoritative.
/// This mirrors the candidate-pruning role described by sweep-line scheduling,
/// using an adaptive conservative x-interval sweep for sufficiently large,
/// sampled-sparse batches and the flat scan for small or dense workloads.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveStringQuery2<'a> {
    curve: &'a CurveString2,
    prepared_segments: Vec<PreparedSegment2<'a>>,
    segment_boxes: Vec<Option<Aabb2>>,
    curve_box: Option<Aabb2>,
    facts: CurveStringFacts,
}

impl<'a> CurveStringQuery2<'a> {
    /// Builds a borrowed query for a curve string.
    pub fn from_curve_string(curve: &'a CurveString2, policy: &CurvePolicy) -> Self {
        // Structural-dispatch note: this query construction pass already visits every
        // segment. The cached boxes feed the adaptive x-interval sweep directly;
        // richer facts such as all-line, all-axis-aligned, or monotone parameter
        // ranges can still support future narrow-phase dispatch.
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

    /// Returns the number of query source segments.
    pub fn segment_count(&self) -> usize {
        self.prepared_segments.len()
    }

    /// Returns primitive-family counts for query source segments.
    pub fn segment_kind_counts(&self) -> SegmentKindCounts {
        segment_kind_counts(&self.prepared_segments)
    }

    /// Returns the number of segment boxes that were decided during query construction.
    pub fn decided_segment_box_count(&self) -> usize {
        self.segment_boxes
            .iter()
            .filter(|bbox| bbox.is_some())
            .count()
    }

    /// Returns the number of source segments whose query construction could not retain
    /// a decided broad-phase box.
    pub fn undecided_segment_box_count(&self) -> usize {
        self.segment_boxes.len() - self.decided_segment_box_count()
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
    /// query curve string.
    pub fn intersect_query(
        &self,
        other: &CurveStringQuery2<'_>,
        policy: &CurvePolicy,
    ) -> CurveResult<Vec<CurveStringIntersection>> {
        intersect_prepared_segment_pairs_with_cached_aabbs(
            &self.prepared_segments,
            &other.prepared_segments,
            self.segment_boxes(),
            other.segment_boxes(),
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
        let other = CurveStringQuery2::from_curve_string(other, policy);
        self.intersect_query(&other, policy)
    }

    /// Trims the query source curve between point intersections with two query cutters.
    ///
    /// The cached broad-phase boxes in all three query views are reused for
    /// intersection collection; exact split validation and materialization still
    /// delegate to the source [`CurveString2`] trim pipeline.
    pub fn trim_between_query_intersections(
        &self,
        start_cutter: &CurveStringQuery2<'_>,
        end_cutter: &CurveStringQuery2<'_>,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<CurveString2>> {
        let start_events = self.intersect_query(start_cutter, policy)?;
        let end_events = self.intersect_query(end_cutter, policy)?;
        self.curve
            .trim_between_curve_intersection_events(start_events, end_events, policy)
    }

    /// Retains portions of this query open curve string inside a query region.
    ///
    /// The region's query contour boxes are reused for boundary-hit
    /// collection and its query point classifier is reused for retained
    /// interval representatives. Exact segment intersections and native
    /// interval materialization remain delegated to the ordinary curve-string
    /// trim pipeline.
    pub fn trim_inside_region_query(
        &self,
        region: &RegionQuery2<'_>,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<Vec<CurveString2>>> {
        self.curve.trim_inside_region_query(region, policy)
    }

    /// Retains portions of this query open curve string inside an ordinary region.
    pub fn trim_inside_region(
        &self,
        region: &LineArcRegion2,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<Vec<CurveString2>>> {
        let region = RegionQuery2::from_region(region, policy);
        self.trim_inside_region_query(&region, policy)
    }

    /// Classifies whether this query open curve string self-contacts.
    pub fn has_self_contacts(&self, policy: &CurvePolicy) -> CurveResult<Classification<bool>> {
        crate::self_intersect::segments_have_self_contacts_with_cached_aabbs(
            self.curve.segments(),
            &self.segment_boxes,
            false,
            policy,
        )
    }
}

/// A borrowed contour with cached contour and segment bounding boxes.
///
/// Retained contours are useful when the same contour participates in many
/// topology queries. The cached boxes are conservative candidate filters only:
/// decided disjoint boxes skip a pair, while hits and uncertain boxes still run
/// the exact line/arc intersection code. Large contours also retain their
/// balanced x-interval maximum witnesses so repeated sparse intersections do
/// not rebuild the same immutable broad-phase index.
#[derive(Clone, Debug, PartialEq)]
pub struct ContourQuery2<'a> {
    contour: &'a Contour2,
    prepared_segments: Vec<PreparedSegment2<'a>>,
    segment_boxes: Vec<Option<Aabb2>>,
    segment_x_index: Option<SegmentAabbXIndex>,
    winding_segment_indices_by_max_x: Option<Vec<usize>>,
    line_winding_index: Option<PreparedLineWindingIndex>,
    contour_box: Option<Aabb2>,
    facts: CurveStringFacts,
}

#[derive(Clone, Debug, PartialEq)]
struct PreparedLineWindingIndex {
    segment_indices_by_max_x: Vec<usize>,
    segment_indices_by_min_y: Vec<usize>,
    segment_indices_by_max_y: Vec<usize>,
    max_x_ranks: Vec<usize>,
    min_y_ranks: Vec<usize>,
    max_y_ranks: Vec<usize>,
    directions: Vec<i8>,
}

impl<'a> ContourQuery2<'a> {
    /// Builds a borrowed query for a contour.
    pub fn from_contour(contour: &'a Contour2, policy: &CurvePolicy) -> Self {
        // Structural-dispatch note: contour query construction can preserve ring-level
        // facts such as convexity, orientation certainty, y-monotonicity, and
        // hole/material provenance for future triangulation and Boolean-region
        // dispatch without weakening the exact boundary classifiers.
        let segment_boxes = decided_segment_boxes(contour.segments(), policy);
        let segment_x_index = (segment_boxes.len() >= 128)
            .then(|| {
                let mut index =
                    SegmentAabbXIndex::try_new(&segment_boxes, segment_boxes.len(), policy)?;
                index
                    .prepare_interval_queries(&segment_boxes, policy)
                    .then_some(index)
            })
            .flatten();
        let winding_segment_indices_by_max_x =
            segment_indices_sorted_by_max_x(&segment_boxes, policy);
        let line_winding_index = prepared_line_winding_index(
            contour.segments(),
            &segment_boxes,
            winding_segment_indices_by_max_x.as_deref(),
            policy,
        );
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
            segment_x_index,
            winding_segment_indices_by_max_x,
            line_winding_index,
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

    /// Returns the number of query source segments.
    pub fn segment_count(&self) -> usize {
        self.prepared_segments.len()
    }

    /// Returns primitive-family counts for query source segments.
    pub fn segment_kind_counts(&self) -> SegmentKindCounts {
        segment_kind_counts(&self.prepared_segments)
    }

    /// Returns the number of segment boxes that were decided during query construction.
    pub fn decided_segment_box_count(&self) -> usize {
        self.segment_boxes
            .iter()
            .filter(|bbox| bbox.is_some())
            .count()
    }

    /// Returns the number of source segments whose query construction could not retain
    /// a decided broad-phase box.
    pub fn undecided_segment_box_count(&self) -> usize {
        self.segment_boxes.len() - self.decided_segment_box_count()
    }

    /// Returns query per-segment predicate handles in source segment order.
    pub(crate) fn prepared_segments(&self) -> &[PreparedSegment2<'a>] {
        &self.prepared_segments
    }

    /// Returns conservative structural facts collected while preparing.
    ///
    /// These facts are advisory scheduling metadata in the exactness model's object layer:
    /// Boolean and containment code can select specialized exact paths from
    /// them, but they are not a geometric decision by themselves.
    pub const fn facts(&self) -> &CurveStringFacts {
        &self.facts
    }

    /// Intersects two query contours using compact exact-line or cached general bounds.
    pub fn intersect_query(
        &self,
        other: &ContourQuery2<'_>,
        policy: &CurvePolicy,
    ) -> CurveResult<ContourIntersectionSet> {
        if let (Some(first), Some(second)) = (
            self.contour.exact_dyadic_line_aabbs(policy),
            other.contour.exact_dyadic_line_aabbs(policy),
        ) {
            return crate::events::intersect_contours_with_exact_dyadic_line_aabbs(
                self.contour,
                other.contour,
                &first,
                &second,
                policy,
            );
        }
        crate::events::intersect_contours_with_cached_aabbs(
            self.contour,
            other.contour,
            self.contour_box(),
            other.contour_box(),
            &self.segment_boxes,
            &other.segment_boxes,
            other.segment_x_index.as_ref(),
            policy,
        )
    }

    /// Intersects this query contour against an ordinary borrowed contour.
    pub fn intersect_contour(
        &self,
        other: &Contour2,
        policy: &CurvePolicy,
    ) -> CurveResult<ContourIntersectionSet> {
        let other = ContourQuery2::from_contour(other, policy);
        self.intersect_query(&other, policy)
    }

    /// Collects self-intersection events using this contour's cached boxes.
    pub fn intersect_self(&self, policy: &CurvePolicy) -> CurveResult<ContourIntersectionSet> {
        crate::events::intersect_contour_self_with_cached_aabbs(
            self.contour,
            &self.segment_boxes,
            policy,
        )
    }

    /// Classifies a point against this query contour.
    pub fn classify_point(
        &self,
        point: &Point2,
        policy: &CurvePolicy,
    ) -> Classification<ContourPointLocation> {
        if self
            .contour_box
            .as_ref()
            .is_some_and(|bbox| aabb_decided_misses_point(bbox, point, policy))
        {
            return Classification::Decided(ContourPointLocation::Outside);
        }

        match prepared_point_on_contour_boundary(self, point, policy) {
            Classification::Decided(true) => {
                return Classification::Decided(ContourPointLocation::Boundary);
            }
            Classification::Decided(false) => {}
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        }

        let winding = match prepared_contour_winding_number_unchecked(self, point, policy) {
            Classification::Decided(winding) => winding,
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        };
        let inside = match self.contour.fill_rule() {
            FillRule::NonZero => winding != 0,
            FillRule::EvenOdd => winding.rem_euclid(2) != 0,
        };
        Classification::Decided(if inside {
            ContourPointLocation::Inside
        } else {
            ContourPointLocation::Outside
        })
    }

    // Callers of this internal path must already have certified that the
    // sample cannot lie on the contour. It deliberately skips the boundary
    // scan while retaining the same exact winding and fill-rule decision.
    pub(crate) fn classify_point_assuming_off_boundary(
        &self,
        point: &Point2,
        policy: &CurvePolicy,
    ) -> Classification<ContourPointLocation> {
        if self
            .contour_box
            .as_ref()
            .is_some_and(|bbox| aabb_decided_misses_point(bbox, point, policy))
        {
            return Classification::Decided(ContourPointLocation::Outside);
        }
        let winding = match prepared_contour_winding_number_unchecked(self, point, policy) {
            Classification::Decided(winding) => winding,
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        };
        let inside = match self.contour.fill_rule() {
            FillRule::NonZero => winding != 0,
            FillRule::EvenOdd => winding.rem_euclid(2) != 0,
        };
        Classification::Decided(if inside {
            ContourPointLocation::Inside
        } else {
            ContourPointLocation::Outside
        })
    }

    /// Returns true when the point lies on this query contour boundary.
    pub fn point_on_boundary(&self, point: &Point2, policy: &CurvePolicy) -> Classification<bool> {
        prepared_point_on_contour_boundary(self, point, policy)
    }

    /// Computes the winding number for a point not on this query boundary.
    pub fn winding_number(&self, point: &Point2, policy: &CurvePolicy) -> Classification<i32> {
        crate::contour::contour_winding_number_with_cached_aabbs(
            self.contour,
            point,
            self.contour_box(),
            &self.segment_boxes,
            policy,
        )
    }

    /// Classifies whether this query closed contour self-contacts.
    pub fn has_self_contacts(&self, policy: &CurvePolicy) -> CurveResult<Classification<bool>> {
        crate::self_intersect::segments_have_self_contacts_with_cached_aabbs(
            self.contour.segments(),
            &self.segment_boxes,
            true,
            policy,
        )
    }
}

/// A borrowed region view with cached contour and region bounding boxes.
///
/// This is useful when many points or intersection queries are run against the
/// same region. The cached boxes are only broad-phase filters: a decided point
/// miss contributes no depth, decided disjoint contour boxes skip intersection
/// candidates, and hits or uncertain boxes still run exact topology. Build the
/// query view with the same policy family used for later queries so arc
/// extrema and coordinate ordering are interpreted consistently.
#[derive(Clone, Debug, PartialEq)]
pub struct RegionQuery2<'a> {
    material_contours: Vec<&'a Contour2>,
    hole_contours: Vec<&'a Contour2>,
    material_prepared_contours: Vec<ContourQuery2<'a>>,
    hole_prepared_contours: Vec<ContourQuery2<'a>>,
    region_box: Option<Aabb2>,
    facts: RegionFacts,
}

impl<'a> RegionQuery2<'a> {
    /// Builds a query view from an owned region.
    pub fn from_region(region: &'a LineArcRegion2, policy: &CurvePolicy) -> Self {
        Self::from_region_view(&region.as_view(), policy)
    }

    /// Builds a query view from a borrowed region view.
    pub fn from_region_view(region: &RegionView2<'a>, policy: &CurvePolicy) -> Self {
        let material_contours = region.material_contours().to_vec();
        let hole_contours = region.hole_contours().to_vec();
        let material_prepared_contours = prepared_contours(&material_contours, policy);
        let hole_prepared_contours = prepared_contours(&hole_contours, policy);
        let region_box = union_all_decided_boxes(
            material_prepared_contours
                .iter()
                .chain(hole_prepared_contours.iter())
                .map(ContourQuery2::contour_box),
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

    /// Returns material contours in the query view.
    pub fn material_contours(&self) -> &[&'a Contour2] {
        &self.material_contours
    }

    /// Returns hole contours in the query view.
    pub fn hole_contours(&self) -> &[&'a Contour2] {
        &self.hole_contours
    }

    /// Reconstructs a borrowed ordinary region view over the same contours.
    ///
    /// The returned view is cheap and keeps the same contour lifetimes. It is
    /// useful when an algorithm still needs the canonical `RegionView2` shape
    /// for splitting or cloning, while query classifiers supply repeated
    /// point and event queries.
    pub fn as_region_view(&self) -> RegionView2<'a> {
        RegionView2::from_contours(
            self.material_contours.iter().copied(),
            self.hole_contours.iter().copied(),
        )
    }

    /// Returns query material contours in region-bin order.
    pub(crate) fn prepared_material_contours(&self) -> &[ContourQuery2<'a>] {
        &self.material_prepared_contours
    }

    /// Returns query hole contours in region-bin order.
    pub(crate) fn prepared_hole_contours(&self) -> &[ContourQuery2<'a>] {
        &self.hole_prepared_contours
    }

    /// Returns the number of query material and hole contours.
    pub fn contour_count(&self) -> usize {
        self.material_prepared_contours.len() + self.hole_prepared_contours.len()
    }

    /// Returns the number of query material source segments.
    pub fn material_segment_count(&self) -> usize {
        self.material_prepared_contours
            .iter()
            .map(ContourQuery2::segment_count)
            .sum()
    }

    /// Returns primitive-family counts for query material source segments.
    pub fn material_segment_kind_counts(&self) -> SegmentKindCounts {
        prepared_contour_kind_counts(&self.material_prepared_contours)
    }

    /// Returns the number of query hole source segments.
    pub fn hole_segment_count(&self) -> usize {
        self.hole_prepared_contours
            .iter()
            .map(ContourQuery2::segment_count)
            .sum()
    }

    /// Returns primitive-family counts for query hole source segments.
    pub fn hole_segment_kind_counts(&self) -> SegmentKindCounts {
        prepared_contour_kind_counts(&self.hole_prepared_contours)
    }

    /// Returns the number of query material and hole source segments.
    pub fn segment_count(&self) -> usize {
        self.material_segment_count() + self.hole_segment_count()
    }

    /// Returns primitive-family counts for all query source segments.
    pub fn segment_kind_counts(&self) -> SegmentKindCounts {
        let mut counts = self.material_segment_kind_counts();
        let hole_counts = self.hole_segment_kind_counts();
        counts.lines += hole_counts.lines;
        counts.arcs += hole_counts.arcs;
        counts
    }

    /// Returns the number of material contour segment boxes decided during query construction.
    pub fn decided_material_segment_box_count(&self) -> usize {
        self.material_prepared_contours
            .iter()
            .map(ContourQuery2::decided_segment_box_count)
            .sum()
    }

    /// Returns the number of hole contour segment boxes decided during query construction.
    pub fn decided_hole_segment_box_count(&self) -> usize {
        self.hole_prepared_contours
            .iter()
            .map(ContourQuery2::decided_segment_box_count)
            .sum()
    }

    /// Returns the number of retained contour segment boxes decided during query construction.
    pub fn decided_segment_box_count(&self) -> usize {
        self.decided_material_segment_box_count() + self.decided_hole_segment_box_count()
    }

    /// Returns the number of material source contour segments whose boxes stayed undecided.
    pub fn undecided_material_segment_box_count(&self) -> usize {
        self.material_prepared_contours
            .iter()
            .map(ContourQuery2::undecided_segment_box_count)
            .sum()
    }

    /// Returns the number of hole source contour segments whose boxes stayed undecided.
    pub fn undecided_hole_segment_box_count(&self) -> usize {
        self.hole_prepared_contours
            .iter()
            .map(ContourQuery2::undecided_segment_box_count)
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

    /// Classifies a point against this query region view.
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

    pub(crate) fn classify_point_assuming_off_boundary(
        &self,
        point: &Point2,
        policy: &CurvePolicy,
    ) -> Classification<RegionPointLocation> {
        // Complete split-marker construction can prove that an interior
        // fragment sample is not on the opposite boundary. Preserve that
        // proof here by evaluating only fill-rule winding; ordinary public
        // point queries continue to use the boundary-first classifier above.
        if self
            .region_box
            .as_ref()
            .is_some_and(|bbox| aabb_decided_misses_point(bbox, point, policy))
        {
            return Classification::Decided(RegionPointLocation::Outside);
        }

        let mut depth = 0;
        match accumulate_depth_assuming_off_boundary(
            &mut depth,
            &self.material_prepared_contours,
            point,
            1,
            policy,
        ) {
            Classification::Decided(()) => {}
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        }
        match accumulate_depth_assuming_off_boundary(
            &mut depth,
            &self.hole_prepared_contours,
            point,
            -1,
            policy,
        ) {
            Classification::Decided(()) => Classification::Decided(if depth > 0 {
                RegionPointLocation::Inside
            } else {
                RegionPointLocation::Outside
            }),
            Classification::Uncertain(reason) => Classification::Uncertain(reason),
        }
    }

    pub(crate) fn single_material_winding_assuming_off_boundary(
        &self,
        point: &Point2,
        policy: &CurvePolicy,
    ) -> Classification<i32> {
        if self.material_prepared_contours.len() != 1 || !self.hole_prepared_contours.is_empty() {
            return Classification::Uncertain(UncertaintyReason::Unsupported);
        }
        if self
            .region_box
            .as_ref()
            .is_some_and(|bbox| aabb_decided_misses_point(bbox, point, policy))
        {
            return Classification::Decided(0);
        }
        let contour = &self.material_prepared_contours[0];
        if contour
            .contour_box()
            .is_some_and(|bbox| aabb_decided_misses_point(bbox, point, policy))
        {
            return Classification::Decided(0);
        }
        prepared_contour_winding_number_unchecked(contour, point, policy)
    }

    /// Returns signed containment depth for a non-boundary point.
    ///
    /// This follows the same signed material-minus-hole convention as
    /// [`RegionView2::signed_depth`]. Decided cached-box misses are skipped, then
    /// candidate contours are classified with the boundary-first winding
    /// structure described by boundary-first winding classification, with this crate's circular-arc extension.
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

    /// Collects normalized topology events against another query region.
    ///
    /// This reuses cached contour and segment boxes for the candidate phase and
    /// then delegates candidate pairs to the same exact line/arc intersection
    /// normalization as [`RegionView2::intersect_region`]. The cache changes the
    /// amount of repeated broad-phase work, not the topology contract.
    pub fn intersect_query(
        &self,
        other: &RegionQuery2<'_>,
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
            Some(self.contour_count()),
            Some(other.contour_count()),
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
        let other = RegionQuery2::from_region_view(other, policy);
        self.intersect_query(&other, policy)
    }

    /// Computes closed boolean boundary loops against another query region.
    ///
    /// This query path runs the same split, classify, and boundary-chain
    /// traversal as [`RegionView2::boolean_boundary_loops`], but reuses cached
    /// region/contour boxes during event collection and fragment midpoint
    /// classification. Cached boxes only prune decided misses, so boundary and
    /// overlap uncertainty is preserved.
    pub fn boolean_boundary_loops(
        &self,
        other: &RegionQuery2<'_>,
        op: BooleanOp,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<BooleanBoundaryLoopSet>> {
        crate::prepared_boolean::boolean_boundary_loops_between_prepared(self, other, op, policy)
    }

    /// Computes closed boolean boundary loops against an ordinary region view.
    ///
    /// This is a mixed query/unquery convenience path: the left operand's
    /// cache is reused, the right operand is query for this call, and the
    /// query-query traversal described in
    /// [`RegionQuery2::boolean_boundary_loops`] remains authoritative.
    /// The transient right-side cache follows the same candidate-pruning role
    /// as sweep-line scheduling broad-phase intersection-event setup, while
    /// the final boundary traversal still follows the polygon-clipping and
    /// split/classify/assemble model described above.
    pub fn boolean_boundary_loops_against_region(
        &self,
        other: &RegionView2<'_>,
        op: BooleanOp,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<BooleanBoundaryLoopSet>> {
        let other = RegionQuery2::from_region_view(other, policy);
        self.boolean_boundary_loops(&other, op, policy)
    }

    /// Computes checked boolean boundary contours against another query
    /// region.
    ///
    /// This extends [`RegionQuery2::boolean_boundary_loops`] through the
    /// same checked-contour conversion and regularized contact fast paths used
    /// by [`RegionView2::boolean_boundary_contours`]. The query parts remain
    /// candidate filters only: the degenerate-intersection clipping model degenerate
    /// clipping cases still surface as explicit boundary handling rather than
    /// as tolerance-based inside/outside choices.
    pub fn boolean_boundary_contours(
        &self,
        other: &RegionQuery2<'_>,
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
    /// then uses [`RegionQuery2::boolean_boundary_contours`]. Keeping the
    /// wrapper explicit makes one-query/many-unquery workloads ergonomic
    /// without weakening the degenerate clipping behavior described by the degenerate-intersection clipping model for boundary contacts.
    pub fn boolean_boundary_contours_against_region(
        &self,
        other: &RegionView2<'_>,
        op: BooleanOp,
        fill_rule: FillRule,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<Vec<Contour2>>> {
        let other = RegionQuery2::from_region_view(other, policy);
        self.boolean_boundary_contours(&other, op, fill_rule, policy)
    }

    /// Computes a role-assigned boolean region against another query region.
    ///
    /// This is the query analogue of [`RegionView2::boolean_region`]. It
    /// reuses cached event and point-classification broad phases before
    /// returning to the ordinary contour-nesting pass for final material/hole
    /// assignment, preserving the fill-state semantics already used
    /// by the non-query region pipeline.
    pub fn boolean_region(
        &self,
        other: &RegionQuery2<'_>,
        op: BooleanOp,
        fill_rule: FillRule,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<LineArcRegion2>> {
        crate::prepared_boolean::boolean_region_between_prepared(self, other, op, fill_rule, policy)
    }

    /// Computes a role-assigned boolean region against an ordinary region view.
    ///
    /// The right operand is query transiently, after which the same query
    /// boolean-region path assigns resolved contours to material and hole bins.
    /// The nesting step remains the boundary-first winding boundary-first point
    /// classification used by [`RegionView2::boolean_region`].
    pub fn boolean_region_against_region(
        &self,
        other: &RegionView2<'_>,
        op: BooleanOp,
        fill_rule: FillRule,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<LineArcRegion2>> {
        let other = RegionQuery2::from_region_view(other, policy);
        self.boolean_region(&other, op, fill_rule, policy)
    }
}

impl CurveString2 {
    /// Borrows this curve string through a retained repeated-query index.
    pub fn query(&self, policy: &CurvePolicy) -> CurveStringQuery2<'_> {
        CurveStringQuery2::from_curve_string(self, policy)
    }
}

impl Contour2 {
    /// Borrows this contour through a retained repeated-query index.
    pub fn query(&self, policy: &CurvePolicy) -> ContourQuery2<'_> {
        ContourQuery2::from_contour(self, policy)
    }
}

impl LineArcRegion2 {
    /// Borrows this region through a retained repeated-query index.
    pub fn query(&self, policy: &CurvePolicy) -> RegionQuery2<'_> {
        RegionQuery2::from_region(self, policy)
    }
}

impl<'a> RegionView2<'a> {
    /// Borrows this region view through a retained repeated-query index.
    pub fn query(&self, policy: &CurvePolicy) -> RegionQuery2<'a> {
        RegionQuery2::from_region_view(self, policy)
    }

    /// Collects normalized topology events against a query right operand.
    ///
    /// This preserves operand order for callers that have already query the
    /// second region. The left view is query transiently, then the query
    /// event collector uses cached broad-phase boxes before exact intersection
    /// normalization.
    pub fn intersect_query(
        &self,
        other: &RegionQuery2<'_>,
        policy: &CurvePolicy,
    ) -> CurveResult<RegionIntersectionSet> {
        let this = RegionQuery2::from_region_view(self, policy);
        this.intersect_query(other, policy)
    }

    /// Computes closed boolean boundary loops against a query right operand.
    ///
    /// Use this when the right operand is reused across many ordinary region
    /// views, especially for non-commutative operations such as difference. The
    /// transient left cache only prunes decided misses; polygon-clipping style
    /// boundary traversal and fragment selection remain
    /// unchanged from [`RegionView2::boolean_boundary_loops`].
    pub fn boolean_boundary_loops_against_query(
        &self,
        other: &RegionQuery2<'_>,
        op: BooleanOp,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<BooleanBoundaryLoopSet>> {
        let this = RegionQuery2::from_region_view(self, policy);
        this.boolean_boundary_loops(other, op, policy)
    }

    /// Computes checked boolean boundary contours against a query right
    /// operand.
    ///
    /// The operation order is `self op other`; the query right operand is not
    /// swapped to the left. Degenerate shared-boundary cases keep the same
    /// explicit degenerate-intersection clipping style uncertainty/regularization behavior
    /// as the ordinary checked-contour API.
    pub fn boolean_boundary_contours_against_query(
        &self,
        other: &RegionQuery2<'_>,
        op: BooleanOp,
        fill_rule: FillRule,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<Vec<Contour2>>> {
        let this = RegionQuery2::from_region_view(self, policy);
        this.boolean_boundary_contours(other, op, fill_rule, policy)
    }

    /// Computes a role-assigned boolean region against a query right
    /// operand.
    ///
    /// Non-overlap line output reuses its certified Boolean direction for role
    /// assignment; overlap or unsupported primitive cases retain the exact
    /// nesting-classifier fallback.
    pub fn boolean_region_against_query(
        &self,
        other: &RegionQuery2<'_>,
        op: BooleanOp,
        fill_rule: FillRule,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<LineArcRegion2>> {
        let this = RegionQuery2::from_region_view(self, policy);
        this.boolean_region(other, op, fill_rule, policy)
    }
}

fn prepared_contours<'a>(
    contours: &[&'a Contour2],
    policy: &CurvePolicy,
) -> Vec<ContourQuery2<'a>>
where
{
    contours
        .iter()
        .map(|contour| ContourQuery2::from_contour(contour, policy))
        .collect()
}

fn prepared_contour_kind_counts(contours: &[ContourQuery2<'_>]) -> SegmentKindCounts {
    let mut counts = SegmentKindCounts::default();
    for contour in contours {
        let contour_counts = segment_kind_counts(contour.prepared_segments());
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

fn prepared_point_on_contour_boundary(
    contour: &ContourQuery2<'_>,
    point: &Point2,
    policy: &CurvePolicy,
) -> Classification<bool> {
    if let Some(index) = contour.segment_x_index.as_ref() {
        let mut candidates = Vec::new();
        index.collect_overlapping(
            &contour.segment_boxes,
            &Aabb2::from_point(point.clone()),
            policy,
            &mut candidates,
        );
        candidates.sort_unstable();
        return prepared_point_on_contour_boundary_candidates(
            contour,
            candidates.into_iter(),
            point,
            policy,
        );
    }
    prepared_point_on_contour_boundary_candidates(
        contour,
        0..contour.prepared_segments.len(),
        point,
        policy,
    )
}

fn prepared_point_on_contour_boundary_candidates(
    contour: &ContourQuery2<'_>,
    candidates: impl Iterator<Item = usize>,
    point: &Point2,
    policy: &CurvePolicy,
) -> Classification<bool> {
    let mut blocker = None;
    for index in candidates {
        let segment = &contour.prepared_segments[index];
        if contour
            .segment_boxes
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

fn prepared_contour_winding_number_unchecked(
    contour: &ContourQuery2<'_>,
    point: &Point2,
    policy: &CurvePolicy,
) -> Classification<i32> {
    if let Some(index) = contour.line_winding_index.as_ref()
        && let Some((max_x_start, min_y_end, max_y_start)) =
            line_winding_candidate_cuts(contour, index, point, policy)
    {
        let candidates = (0..index.directions.len()).filter(|segment_index| {
            index.max_x_ranks[*segment_index] >= max_x_start
                && index.min_y_ranks[*segment_index] < min_y_end
                && index.max_y_ranks[*segment_index] >= max_y_start
        });
        return accumulate_indexed_line_winding(contour, index, candidates, point, policy);
    }

    if let Some(candidate_indices) = sorted_winding_candidate_indices(contour, point, policy) {
        return accumulate_prepared_contour_winding(
            contour,
            candidate_indices.iter().copied(),
            point,
            policy,
        );
    }

    accumulate_prepared_contour_winding(
        contour,
        (0..contour.prepared_segments.len()).filter(|index| {
            !contour.segment_boxes[*index].as_ref().is_some_and(|bbox| {
                matches!(
                    crate::classify::compare_reals(bbox.max_x(), point.x(), policy),
                    Some(Ordering::Less)
                )
            })
        }),
        point,
        policy,
    )
}

fn accumulate_prepared_contour_winding(
    contour: &ContourQuery2<'_>,
    segment_indices: impl Iterator<Item = usize>,
    point: &Point2,
    policy: &CurvePolicy,
) -> Classification<i32> {
    let mut winding = 0;
    for index in segment_indices {
        let segment = &contour.prepared_segments[index];
        let segment_box = contour.segment_boxes.get(index).and_then(Option::as_ref);
        let delta = match segment {
            PreparedSegment2::Line(line) => prepared_line_winding(line, segment_box, point, policy),
            PreparedSegment2::Arc(arc) => {
                crate::contour::process_arc_winding(arc.circular_arc(), point, policy)
            }
        };
        let Some(delta) = delta else {
            return Classification::Uncertain(UncertaintyReason::Ordering);
        };
        winding += delta;
    }
    Classification::Decided(winding)
}

fn segment_indices_sorted_by_max_x(
    segment_boxes: &[Option<Aabb2>],
    policy: &CurvePolicy,
) -> Option<Vec<usize>> {
    if matches!(policy.numeric_mode, crate::NumericMode::EdgePreview) {
        return None;
    }
    segment_indices_sorted_by_box_coordinate(segment_boxes, policy, Aabb2::max_x)
}

fn prepared_line_winding_index(
    segments: &[Segment2],
    segment_boxes: &[Option<Aabb2>],
    segment_indices_by_max_x: Option<&[usize]>,
    policy: &CurvePolicy,
) -> Option<PreparedLineWindingIndex> {
    const MIN_INDEXED_LINE_SEGMENTS: usize = 8;
    if segments.len() < MIN_INDEXED_LINE_SEGMENTS
        || matches!(policy.numeric_mode, crate::NumericMode::EdgePreview)
    {
        return None;
    }
    let segment_indices_by_max_x = segment_indices_by_max_x?.to_vec();
    let segment_indices_by_min_y =
        segment_indices_sorted_by_box_coordinate(segment_boxes, policy, Aabb2::min_y)?;
    let segment_indices_by_max_y =
        segment_indices_sorted_by_box_coordinate(segment_boxes, policy, Aabb2::max_y)?;
    let mut directions = Vec::with_capacity(segments.len());
    for segment in segments {
        let Segment2::Line(line) = segment else {
            return None;
        };
        directions.push(
            match crate::classify::compare_reals(line.start().y(), line.end().y(), policy)? {
                Ordering::Less => 1,
                Ordering::Equal => 0,
                Ordering::Greater => -1,
            },
        );
    }

    Some(PreparedLineWindingIndex {
        max_x_ranks: sorted_segment_ranks(&segment_indices_by_max_x),
        min_y_ranks: sorted_segment_ranks(&segment_indices_by_min_y),
        max_y_ranks: sorted_segment_ranks(&segment_indices_by_max_y),
        segment_indices_by_max_x,
        segment_indices_by_min_y,
        segment_indices_by_max_y,
        directions,
    })
}

fn segment_indices_sorted_by_box_coordinate(
    segment_boxes: &[Option<Aabb2>],
    policy: &CurvePolicy,
    coordinate: crate::events::BoxCoordinate,
) -> Option<Vec<usize>> {
    if segment_boxes.iter().any(Option::is_none) {
        return None;
    }
    let mut indices: Vec<_> = (0..segment_boxes.len()).collect();
    crate::events::sort_segment_indices_by_certified_box_coordinate(
        &mut indices,
        segment_boxes,
        segment_boxes.len(),
        policy,
        coordinate,
    )
    .then_some(indices)
}

fn sorted_segment_ranks(indices: &[usize]) -> Vec<usize> {
    let mut ranks = vec![0; indices.len()];
    for (rank, segment_index) in indices.iter().copied().enumerate() {
        ranks[segment_index] = rank;
    }
    ranks
}

fn line_winding_candidate_cuts(
    contour: &ContourQuery2<'_>,
    index: &PreparedLineWindingIndex,
    point: &Point2,
    policy: &CurvePolicy,
) -> Option<(usize, usize, usize)> {
    let max_x_start = sorted_box_coordinate_partition(
        &index.segment_indices_by_max_x,
        &contour.segment_boxes,
        point.x(),
        policy,
        Aabb2::max_x,
        false,
    )?;
    let min_y_end = sorted_box_coordinate_partition(
        &index.segment_indices_by_min_y,
        &contour.segment_boxes,
        point.y(),
        policy,
        Aabb2::min_y,
        true,
    )?;
    let max_y_start = sorted_box_coordinate_partition(
        &index.segment_indices_by_max_y,
        &contour.segment_boxes,
        point.y(),
        policy,
        Aabb2::max_y,
        true,
    )?;
    Some((max_x_start, min_y_end, max_y_start))
}

fn sorted_box_coordinate_partition(
    indices: &[usize],
    segment_boxes: &[Option<Aabb2>],
    query: &crate::Real,
    policy: &CurvePolicy,
    coordinate: for<'a> fn(&'a Aabb2) -> &'a crate::Real,
    include_equal_in_lower_partition: bool,
) -> Option<usize> {
    if !matches!(policy.numeric_mode, crate::NumericMode::EdgePreview)
        && let Some(query_preview) = query.to_f64_lossy().filter(|value| value.is_finite())
    {
        let mut preview_start = 0;
        let mut preview_end = indices.len();
        let mut preview_succeeded = true;
        while preview_start < preview_end {
            let middle = preview_start + (preview_end - preview_start) / 2;
            let bbox = segment_boxes[indices[middle]].as_ref()?;
            let Some(value) = coordinate(bbox)
                .to_f64_lossy()
                .filter(|value| value.is_finite())
            else {
                preview_succeeded = false;
                break;
            };
            if value < query_preview || include_equal_in_lower_partition && value == query_preview {
                preview_start = middle + 1;
            } else {
                preview_end = middle;
            }
        }
        if preview_succeeded
            && partition_boundary_is_certified(
                indices,
                segment_boxes,
                query,
                policy,
                coordinate,
                include_equal_in_lower_partition,
                preview_start,
            )
        {
            return Some(preview_start);
        }
    }

    let mut start = 0;
    let mut end = indices.len();
    while start < end {
        let middle = start + (end - start) / 2;
        let bbox = segment_boxes[indices[middle]].as_ref()?;
        match crate::classify::compare_reals(coordinate(bbox), query, policy)? {
            Ordering::Less => start = middle + 1,
            Ordering::Equal if include_equal_in_lower_partition => start = middle + 1,
            Ordering::Equal | Ordering::Greater => end = middle,
        }
    }
    Some(start)
}

fn partition_boundary_is_certified(
    indices: &[usize],
    segment_boxes: &[Option<Aabb2>],
    query: &crate::Real,
    policy: &CurvePolicy,
    coordinate: for<'a> fn(&'a Aabb2) -> &'a crate::Real,
    include_equal_in_lower_partition: bool,
    partition: usize,
) -> bool {
    let lower_is_valid = partition == 0 || {
        let Some(bbox) = segment_boxes[indices[partition - 1]].as_ref() else {
            return false;
        };
        match crate::classify::compare_reals(coordinate(bbox), query, policy) {
            Some(Ordering::Less) => true,
            Some(Ordering::Equal) => include_equal_in_lower_partition,
            Some(Ordering::Greater) | None => false,
        }
    };
    let upper_is_valid = partition == indices.len() || {
        let Some(bbox) = segment_boxes[indices[partition]].as_ref() else {
            return false;
        };
        match crate::classify::compare_reals(coordinate(bbox), query, policy) {
            Some(Ordering::Greater) => true,
            Some(Ordering::Equal) => !include_equal_in_lower_partition,
            Some(Ordering::Less) | None => false,
        }
    };
    lower_is_valid && upper_is_valid
}

fn sorted_winding_candidate_indices<'a>(
    contour: &'a ContourQuery2<'_>,
    point: &Point2,
    policy: &CurvePolicy,
) -> Option<&'a [usize]> {
    let indices = contour.winding_segment_indices_by_max_x.as_deref()?;
    let mut start = 0;
    let mut end = indices.len();
    while start < end {
        let middle = start + (end - start) / 2;
        let bbox = contour.segment_boxes[indices[middle]].as_ref()?;
        match crate::classify::compare_reals(bbox.max_x(), point.x(), policy) {
            Some(Ordering::Less) => start = middle + 1,
            Some(Ordering::Equal | Ordering::Greater) => end = middle,
            None => return None,
        }
    }
    Some(&indices[start..])
}

fn prepared_line_winding(
    line: &PreparedLineSeg2<'_>,
    segment_box: Option<&Aabb2>,
    point: &Point2,
    policy: &CurvePolicy,
) -> Option<i32> {
    let source = line.line_segment();
    let start_at_or_below = !matches!(
        crate::classify::compare_reals(source.start().y(), point.y(), policy)?,
        Ordering::Greater
    );
    let crosses_upward = start_at_or_below
        && matches!(
            crate::classify::compare_reals(source.end().y(), point.y(), policy)?,
            Ordering::Greater
        );
    if crosses_upward {
        return prepared_line_crossing_winding(line, segment_box, point, 1, policy);
    }
    if start_at_or_below {
        return Some(0);
    }

    let end_at_or_below = !matches!(
        crate::classify::compare_reals(source.end().y(), point.y(), policy)?,
        Ordering::Greater
    );
    if !end_at_or_below {
        return Some(0);
    }
    prepared_line_crossing_winding(line, segment_box, point, -1, policy)
}

fn prepared_line_crossing_winding(
    line: &PreparedLineSeg2<'_>,
    segment_box: Option<&Aabb2>,
    point: &Point2,
    direction: i32,
    policy: &CurvePolicy,
) -> Option<i32> {
    // The y tests or retained interval index prove one ray crossing. If the
    // complete line box is strictly right of the query, that crossing is on
    // the positive ray and no orientation predicate is needed. Equality and
    // uncertain x order stay on the exact predicate path.
    if segment_box.is_some_and(|bbox| aabb_decided_strictly_right_of_point(bbox, point, policy)) {
        return Some(direction);
    }
    match (direction, line.classify_point(point, policy)) {
        (1, Classification::Decided(LineSide::Left))
        | (-1, Classification::Decided(LineSide::On | LineSide::Right)) => Some(direction),
        (1, Classification::Decided(LineSide::On | LineSide::Right))
        | (-1, Classification::Decided(LineSide::Left)) => Some(0),
        (_, Classification::Uncertain(_)) => None,
        _ => None,
    }
}

fn accumulate_indexed_line_winding(
    contour: &ContourQuery2<'_>,
    index: &PreparedLineWindingIndex,
    segment_indices: impl Iterator<Item = usize>,
    point: &Point2,
    policy: &CurvePolicy,
) -> Classification<i32> {
    let mut winding = 0;
    for segment_index in segment_indices {
        let PreparedSegment2::Line(line) = &contour.prepared_segments[segment_index] else {
            return Classification::Uncertain(UncertaintyReason::Unsupported);
        };
        let direction = i32::from(index.directions[segment_index]);
        if direction == 0 {
            continue;
        }
        let Some(delta) = prepared_line_crossing_winding(
            line,
            contour.segment_boxes[segment_index].as_ref(),
            point,
            direction,
            policy,
        ) else {
            return Classification::Uncertain(UncertaintyReason::Ordering);
        };
        winding += delta;
    }
    Classification::Decided(winding)
}

fn segment_kind_counts(segments: &[PreparedSegment2<'_>]) -> SegmentKindCounts {
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
    policy: &CurvePolicy,
) -> CurveResult<Vec<CurveStringIntersection>> {
    let mut intersections = Vec::new();
    let x_overlap_schedule =
        curve_string_x_overlap_schedule(first_segment_boxes, second_segment_boxes);

    for (a_segment_index, a_segment) in first_prepared_segments.iter().enumerate() {
        for b_segment_index in x_overlap_schedule.as_ref().map_or_else(
            || CurveStringXOverlapCandidates::All(0..second_prepared_segments.len()),
            |schedule| schedule.candidates_for(a_segment_index),
        ) {
            let b_segment = &second_prepared_segments[b_segment_index];
            if let (Some(Some(a_box)), Some(Some(b_box))) = (
                first_segment_boxes.get(a_segment_index),
                second_segment_boxes.get(b_segment_index),
            ) && aabbs_decided_disjoint(a_box, b_box, policy)
            {
                continue;
            }

            let relation = a_segment.intersect_prepared_segment(b_segment, policy)?;
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

    Ok(intersections)
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
    contours: &[ContourQuery2<'_>],
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

fn accumulate_depth_assuming_off_boundary(
    depth: &mut i32,
    contours: &[ContourQuery2<'_>],
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

        let winding = match prepared_contour_winding_number_unchecked(contour, point, policy) {
            Classification::Decided(winding) => winding,
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        };
        let inside = match contour.contour().fill_rule() {
            FillRule::NonZero => winding != 0,
            FillRule::EvenOdd => winding.rem_euclid(2) != 0,
        };
        if inside {
            *depth += sign;
        }
    }

    Classification::Decided(())
}

fn collect_prepared_role_pairs(
    pairs: &mut Vec<RegionContourIntersection>,
    workload: &mut RegionIntersectionWorkload,
    first_contours: &[ContourQuery2<'_>],
    first_role: RegionContourRole,
    second_contours: &[ContourQuery2<'_>],
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
            let intersections = first_contour.intersect_query(second_contour, policy)?;
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
