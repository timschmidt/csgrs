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
    Aabb2, aabb_decided_misses_point, aabb_decided_strictly_right_of_point, decided_segment_aabb,
};
use crate::events::SegmentAabbXIndex;
use crate::facts::{CurveStringFacts, RegionFacts};
use crate::{
    CircularArc2, CircularArc2Facts, Classification, Contour2, ContourPointLocation, CurvePolicy,
    CurveString2, FillRule, LineSeg2, LineSeg2Facts, LineSide, Point2, RegionPointLocation,
    RegionView2, Segment2, UncertaintyReason,
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
}

pub(crate) fn curve_string_facts(curve: &CurveString2, policy: &CurvePolicy) -> CurveStringFacts {
    let segment_boxes = decided_segment_boxes(curve.segments(), policy);
    let curve_box = union_all_decided_boxes(segment_boxes.iter().map(Option::as_ref), policy);
    crate::facts::curve_string_facts(
        curve,
        segment_boxes.iter().filter(|bbox| bbox.is_some()).count(),
        curve_box.is_some(),
    )
}

pub(crate) fn contour_facts(contour: &Contour2, policy: &CurvePolicy) -> CurveStringFacts {
    let segment_boxes = decided_segment_boxes(contour.segments(), policy);
    let contour_box = union_all_decided_boxes(segment_boxes.iter().map(Option::as_ref), policy);
    crate::facts::contour_facts(
        contour,
        segment_boxes.iter().filter(|bbox| bbox.is_some()).count(),
        contour_box.is_some(),
    )
}

pub(crate) fn region_view_facts(region: &RegionView2<'_>, policy: &CurvePolicy) -> RegionFacts {
    let contour_boxes = region
        .material_contours()
        .iter()
        .chain(region.hole_contours().iter())
        .map(|contour| {
            let segment_boxes = decided_segment_boxes(contour.segments(), policy);
            union_all_decided_boxes(segment_boxes.iter().map(Option::as_ref), policy)
        })
        .collect::<Vec<_>>();
    let has_decided_region_box =
        union_all_decided_boxes(contour_boxes.iter().map(Option::as_ref), policy).is_some();
    crate::facts::region_view_facts(region, has_decided_region_box)
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
pub(crate) struct ContourQuery2<'a> {
    contour: &'a Contour2,
    prepared_segments: Vec<PreparedSegment2<'a>>,
    segment_boxes: Vec<Option<Aabb2>>,
    segment_x_index: Option<SegmentAabbXIndex>,
    winding_segment_indices_by_max_x: Option<Vec<usize>>,
    line_winding_index: Option<PreparedLineWindingIndex>,
    contour_box: Option<Aabb2>,
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
        let prepared_segments = prepared_segments(contour.segments());

        Self {
            contour,
            prepared_segments,
            segment_boxes,
            segment_x_index,
            winding_segment_indices_by_max_x,
            line_winding_index,
            contour_box,
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
pub(crate) struct RegionQuery2<'a> {
    material_prepared_contours: Vec<ContourQuery2<'a>>,
    hole_prepared_contours: Vec<ContourQuery2<'a>>,
    region_box: Option<Aabb2>,
}

impl<'a> RegionQuery2<'a> {
    /// Builds a query view from a borrowed region view.
    pub fn from_region_view(region: &RegionView2<'a>, policy: &CurvePolicy) -> Self {
        let material_prepared_contours: Vec<_> = region
            .material_contours()
            .iter()
            .map(|contour| ContourQuery2::from_contour(contour, policy))
            .collect();
        let hole_prepared_contours: Vec<_> = region
            .hole_contours()
            .iter()
            .map(|contour| ContourQuery2::from_contour(contour, policy))
            .collect();
        let region_box = union_all_decided_boxes(
            material_prepared_contours
                .iter()
                .chain(hole_prepared_contours.iter())
                .map(ContourQuery2::contour_box),
            policy,
        );
        Self {
            material_prepared_contours,
            hole_prepared_contours,
            region_box,
        }
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
