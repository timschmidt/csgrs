//! Contour fragments produced from split markers.
//!
//! After intersections are inserted, fragment construction rebuilds the
//! source geometry between adjacent split markers so boolean selection can work
//! on atomic boundary pieces. This mirrors the split-then-classify structure in
//! Greiner and Hormann, "Efficient Clipping of Arbitrary Polygons" (*ACM
//! Transactions on Graphics* 17(2), 71-83, 1998), with explicit uncertainty for
//! ordering or finite-preview cases that would otherwise create invalid graph
//! topology.

use std::cmp::Ordering;

use hyperreal::Real;

use crate::classify::{
    compare_reals, compare_reals_for_split_ordering, in_closed_unit_interval, is_zero,
};
use crate::{
    CircularArc2, Classification, Contour2, ContourSplitMarkers, CurveError, CurvePolicy,
    CurveResult, NumericMode, ParamRange, Point2, Segment2, SegmentSplitMarker, UncertaintyReason,
};

/// One source-contour fragment between adjacent split markers.
#[derive(Clone, Debug, PartialEq)]
pub struct ContourFragment {
    /// Source segment index in the original contour.
    pub source_segment_index: usize,
    /// Exact start point of the original source segment.
    pub source_segment_start_point: Point2,
    /// Exact end point of the original source segment.
    pub source_segment_end_point: Point2,
    /// Parameter interval on the source segment.
    pub source_range: ParamRange,
    /// Fragment geometry in source traversal direction.
    pub segment: Segment2,
}

/// Ordered fragments from a split contour.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct ContourFragmentSet {
    fragments: Vec<ContourFragment>,
}

impl ContourFragmentSet {
    /// Constructs a fragment set from already-built fragments.
    pub fn new(fragments: Vec<ContourFragment>) -> CurveResult<Self> {
        Self::new_with_policy(fragments, &CurvePolicy::certified())
    }

    fn new_with_policy(fragments: Vec<ContourFragment>, policy: &CurvePolicy) -> CurveResult<Self> {
        validate_contour_fragments(&fragments, policy)?;
        Ok(Self { fragments })
    }

    /// Builds fragments from point-bearing contour split markers.
    pub fn from_split_markers(
        contour: &Contour2,
        markers: &ContourSplitMarkers,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<Self>> {
        if contour.len() != markers.segment_count() {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        }
        match validate_split_markers_against_contour(contour, markers, policy)? {
            Classification::Decided(()) => {}
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        }

        let mut fragments = Vec::new();
        for (segment_index, source_segment) in contour.segments().iter().enumerate() {
            let Some(segment_markers) = markers.markers_for_segment(segment_index) else {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            };

            match append_segment_fragments(
                &mut fragments,
                source_segment,
                segment_index,
                segment_markers,
                policy,
            )? {
                Classification::Decided(()) => {}
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            }
        }

        // Marker validation and adjacent-pair construction already certify
        // forward, disjoint source ranges. Re-running the generic fragment-set
        // validator here discards that ordering provenance and asks exact-real
        // arithmetic to rediscover equal shared marker parameters.
        Ok(Classification::Decided(Self { fragments }))
    }

    /// Returns fragments in contour traversal order.
    pub fn fragments(&self) -> &[ContourFragment] {
        &self.fragments
    }

    /// Consumes the set and returns the fragments.
    pub fn into_fragments(self) -> Vec<ContourFragment> {
        self.fragments
    }

    /// Returns true when no fragments were built.
    pub fn is_empty(&self) -> bool {
        self.fragments.is_empty()
    }

    /// Returns the number of fragments.
    pub fn len(&self) -> usize {
        self.fragments.len()
    }
}

fn validate_contour_fragments(
    fragments: &[ContourFragment],
    policy: &CurvePolicy,
) -> CurveResult<()> {
    for fragment in fragments {
        validate_contour_fragment_source_range(fragment, policy)?;
    }

    for (left_index, left) in fragments.iter().enumerate() {
        if fragments[left_index + 1..]
            .iter()
            .any(|right| right == left)
        {
            return Err(CurveError::Topology(
                "contour fragment set must not contain duplicate fragments".into(),
            ));
        }
        for right in &fragments[left_index + 1..] {
            validate_contour_fragment_source_ranges_disjoint(left, right, policy)?;
        }
    }
    Ok(())
}

fn validate_contour_fragment_source_range(
    fragment: &ContourFragment,
    policy: &CurvePolicy,
) -> CurveResult<()> {
    if in_closed_unit_interval(fragment.source_range.start(), policy) != Some(true)
        || in_closed_unit_interval(fragment.source_range.end(), policy) != Some(true)
    {
        return Err(CurveError::Topology(
            "contour fragment source range endpoints must be certified inside the unit interval"
                .into(),
        ));
    }
    match compare_reals_for_split_ordering(
        fragment.source_range.start(),
        fragment.source_range.end(),
        policy,
    ) {
        Some(Ordering::Less) => Ok(()),
        Some(Ordering::Equal) => Err(CurveError::Topology(
            "contour fragment source range must be positive-dimensional".into(),
        )),
        Some(Ordering::Greater) => Err(CurveError::Topology(
            "contour fragment source range must be forward in source parameter".into(),
        )),
        None => Err(CurveError::Topology(
            "contour fragment source range ordering must be certified".into(),
        )),
    }
}

fn validate_contour_fragment_source_ranges_disjoint(
    left: &ContourFragment,
    right: &ContourFragment,
    policy: &CurvePolicy,
) -> CurveResult<()> {
    if left.source_segment_index != right.source_segment_index {
        return Ok(());
    }

    let left_before_right = match compare_reals_for_split_ordering(
        left.source_range.end(),
        right.source_range.start(),
        policy,
    ) {
        Some(Ordering::Less | Ordering::Equal) => true,
        Some(Ordering::Greater) => false,
        None => {
            return Err(CurveError::Topology(
                "contour fragment source range separation must be certified".into(),
            ));
        }
    };
    let right_before_left = match compare_reals_for_split_ordering(
        right.source_range.end(),
        left.source_range.start(),
        policy,
    ) {
        Some(Ordering::Less | Ordering::Equal) => true,
        Some(Ordering::Greater) => false,
        None => {
            return Err(CurveError::Topology(
                "contour fragment source range separation must be certified".into(),
            ));
        }
    };
    if !left_before_right && !right_before_left {
        return Err(CurveError::Topology(
            "contour fragment set must not overlap retained source ranges".into(),
        ));
    }
    Ok(())
}

fn validate_split_markers_against_contour(
    contour: &Contour2,
    markers: &ContourSplitMarkers,
    policy: &CurvePolicy,
) -> CurveResult<Classification<()>> {
    for (segment_index, source_segment) in contour.segments().iter().enumerate() {
        let Some(segment_markers) = markers.markers_for_segment(segment_index) else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        for marker in segment_markers {
            if marker.segment_index != segment_index {
                return Err(CurveError::Topology(
                    "contour split marker references a different source segment".into(),
                ));
            }
            if markers.source_incidence_certified() {
                continue;
            }
            match split_marker_matches_source_segment(source_segment, marker, policy)? {
                Classification::Decided(()) => {}
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            }
        }
    }
    Ok(Classification::Decided(()))
}

fn split_marker_matches_source_segment(
    source_segment: &Segment2,
    marker: &SegmentSplitMarker,
    policy: &CurvePolicy,
) -> CurveResult<Classification<()>> {
    match source_segment {
        Segment2::Line(line) => {
            let expected = line.point_at(marker.param.clone());
            let distance = marker.point.distance_squared(&expected);
            match point_distance_is_zero(&distance, &marker.point, &expected, policy) {
                Some(true) => Ok(Classification::Decided(())),
                Some(false) if matches!(policy.numeric_mode, NumericMode::EdgePreview) => {
                    Ok(Classification::Uncertain(UncertaintyReason::Unsupported))
                }
                Some(false) => Err(CurveError::Topology(
                    "contour split marker point does not match source line parameter".into(),
                )),
                None => Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
            }
        }
        Segment2::Arc(arc) => split_marker_matches_source_arc(arc, marker, policy),
    }
}

fn point_distance_is_zero(
    distance_squared: &Real,
    left: &crate::Point2,
    right: &crate::Point2,
    policy: &CurvePolicy,
) -> Option<bool> {
    if is_zero(distance_squared, policy) == Some(true) {
        return Some(true);
    }

    if matches!(policy.numeric_mode, NumericMode::EdgePreview)
        && let (Some(distance_squared), Some(left_scale), Some(right_scale)) = (
            distance_squared.to_f64_lossy(),
            point_coordinate_scale(left),
            point_coordinate_scale(right),
        )
        && distance_squared.is_finite()
    {
        let (absolute, relative) = policy
            .tolerance
            .map(|tolerance| (tolerance.absolute, tolerance.relative))
            .unwrap_or((1e-12, 1e-12));
        let scale = left_scale.max(right_scale).max(1.0);
        let tolerance = absolute.max(relative * scale);
        return Some(distance_squared <= tolerance * tolerance);
    }

    is_zero(distance_squared, policy)
}

fn point_coordinate_scale(point: &crate::Point2) -> Option<f64> {
    let x = point.x().to_f64_lossy()?;
    let y = point.y().to_f64_lossy()?;
    if x.is_finite() && y.is_finite() {
        Some(x.abs().max(y.abs()))
    } else {
        None
    }
}

fn split_marker_matches_source_arc(
    source_arc: &CircularArc2,
    marker: &SegmentSplitMarker,
    policy: &CurvePolicy,
) -> CurveResult<Classification<()>> {
    let radius_squared = source_arc.radius_squared();
    let radius_delta = marker.point.distance_squared(source_arc.center()) - &radius_squared;
    match radius_delta_is_zero(&radius_delta, &radius_squared, policy) {
        Some(true) => {}
        Some(false) if matches!(policy.numeric_mode, NumericMode::EdgePreview) => {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        }
        Some(false) => {
            return Err(CurveError::Topology(
                "contour split marker point does not lie on source arc circle".into(),
            ));
        }
        None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
    }

    match source_arc.contains_sweep_point(&marker.point, policy) {
        Classification::Decided(true) => {}
        Classification::Decided(false)
            if matches!(policy.numeric_mode, NumericMode::EdgePreview) =>
        {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        }
        Classification::Decided(false) => {
            return Err(CurveError::Topology(
                "contour split marker point does not lie on source arc sweep".into(),
            ));
        }
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    }

    let expected_param = segment_chord_param(source_arc.start(), source_arc.end(), &marker.point)?;
    match compare_reals(&marker.param, &expected_param, policy) {
        Some(Ordering::Equal) => Ok(Classification::Decided(())),
        Some(_) if matches!(policy.numeric_mode, NumericMode::EdgePreview) => {
            Ok(Classification::Uncertain(UncertaintyReason::Unsupported))
        }
        Some(_) => Err(CurveError::Topology(
            "contour split marker parameter does not match source arc chord evidence".into(),
        )),
        None => Ok(Classification::Uncertain(UncertaintyReason::Ordering)),
    }
}

fn segment_chord_param(
    start: &crate::Point2,
    end: &crate::Point2,
    point: &crate::Point2,
) -> CurveResult<Real> {
    let (dx, dy) = end.delta_from(start);
    let (px, py) = point.delta_from(start);
    let numerator = (&px * &dx) + (&py * &dy);
    let denominator = (&dx * &dx) + (&dy * &dy);
    (numerator / denominator).map_err(Into::into)
}

pub(crate) fn split_contour_at_intersections(
    contour: &Contour2,
    intersections: &crate::ContourIntersectionSet,
    operand: crate::ContourOperand,
    policy: &CurvePolicy,
) -> CurveResult<Classification<ContourFragmentSet>> {
    validate_contour_intersection_evidence_against_contour(contour, intersections, &[operand])?;

    let markers =
        match ContourSplitMarkers::from_intersections(contour, intersections, operand, policy) {
            Classification::Decided(markers) => markers,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };

    ContourFragmentSet::from_split_markers(contour, &markers, policy)
}

pub(crate) fn split_contour_at_self_intersections(
    contour: &Contour2,
    intersections: &crate::ContourIntersectionSet,
    policy: &CurvePolicy,
) -> CurveResult<Classification<ContourFragmentSet>> {
    validate_contour_intersection_evidence_against_contour(
        contour,
        intersections,
        &[crate::ContourOperand::First, crate::ContourOperand::Second],
    )?;

    let markers = match ContourSplitMarkers::from_self_intersections(contour, intersections, policy)
    {
        Classification::Decided(markers) => markers,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };

    ContourFragmentSet::from_split_markers(contour, &markers, policy)
}

fn validate_contour_intersection_evidence_against_contour(
    contour: &Contour2,
    intersections: &crate::ContourIntersectionSet,
    operands: &[crate::ContourOperand],
) -> CurveResult<()> {
    for event in intersections.events() {
        for operand in operands {
            let Some(segment_index) = event.segment_index(*operand) else {
                return Err(CurveError::Topology(
                    "contour intersection event must carry segment index evidence".into(),
                ));
            };
            if segment_index >= contour.len() {
                return Err(CurveError::Topology(
                    "contour intersection event references segment outside supplied contour".into(),
                ));
            }
        }
    }
    Ok(())
}

fn append_segment_fragments(
    fragments: &mut Vec<ContourFragment>,
    source_segment: &Segment2,
    segment_index: usize,
    markers: &[SegmentSplitMarker],
    policy: &CurvePolicy,
) -> CurveResult<Classification<()>> {
    for adjacent in markers.windows(2) {
        let start = &adjacent[0];
        let end = &adjacent[1];

        match compare_reals_for_split_ordering(&start.param, &end.param, policy) {
            Some(Ordering::Less) => {}
            Some(Ordering::Equal) => continue,
            Some(Ordering::Greater) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Ordering));
            }
            None => return Ok(Classification::Uncertain(UncertaintyReason::Ordering)),
        }

        let distance = start.point.distance_squared(&end.point);
        match is_zero(&distance, policy) {
            Some(true) => continue,
            Some(false) => {}
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        }

        let segment = match build_fragment_segment(source_segment, start, end, policy)? {
            Classification::Decided(segment) => segment,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        fragments.push(ContourFragment {
            source_segment_index: segment_index,
            source_segment_start_point: source_segment.start().clone(),
            source_segment_end_point: source_segment.end().clone(),
            source_range: ParamRange::new(start.param.clone(), end.param.clone()),
            segment,
        });
    }

    Ok(Classification::Decided(()))
}

fn build_fragment_segment(
    source_segment: &Segment2,
    start: &SegmentSplitMarker,
    end: &SegmentSplitMarker,
    policy: &CurvePolicy,
) -> CurveResult<Classification<Segment2>> {
    if start.point == *source_segment.start() && end.point == *source_segment.end() {
        return Ok(Classification::Decided(source_segment.clone()));
    }

    match source_segment {
        Segment2::Line(line) => line
            .fragment_between(start.point.clone(), end.point.clone())
            .map(Segment2::Line)
            .map(Classification::Decided),
        Segment2::Arc(arc) => build_arc_fragment_segment(arc, start, end, policy),
    }
}

fn build_arc_fragment_segment(
    source_arc: &CircularArc2,
    start: &SegmentSplitMarker,
    end: &SegmentSplitMarker,
    policy: &CurvePolicy,
) -> CurveResult<Classification<Segment2>> {
    // Intersection points in the f64 interop comparison path can be infinitesimally off
    // the source circle after square roots and line solves. The fragment still
    // belongs to the original arc when both split markers are on that source
    // circle under the active policy, so preserve the source radius instead of
    // revalidating with the exact public constructor. This is a finite-output
    // normalization step in the sense of Hobby, "Practical Segment Intersection
    // with Finite Precision Output" (Computational Geometry 13(4), 199-214,
    // 1999), and is limited to fragment reconstruction after the source arc has
    // already supplied the circle.
    let radius_squared = source_arc.radius_squared();
    let start_radius_delta = start.point.distance_squared(source_arc.center()) - &radius_squared;
    let end_radius_delta = end.point.distance_squared(source_arc.center()) - &radius_squared;
    match (
        radius_delta_is_zero(&start_radius_delta, &radius_squared, policy),
        radius_delta_is_zero(&end_radius_delta, &radius_squared, policy),
    ) {
        (Some(true), Some(true)) => Ok(Classification::Decided(Segment2::Arc(
            CircularArc2::new_unchecked_with_radius(
                start.point.clone(),
                end.point.clone(),
                source_arc.center().clone(),
                radius_squared,
                source_arc.is_clockwise(),
                None,
            ),
        ))),
        (Some(false), _) | (_, Some(false)) => {
            Ok(Classification::Uncertain(UncertaintyReason::Unsupported))
        }
        _ => Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
    }
}

fn radius_delta_is_zero(delta: &Real, radius_squared: &Real, policy: &CurvePolicy) -> Option<bool> {
    if is_zero(delta, policy) == Some(true) {
        return Some(true);
    }

    if matches!(policy.numeric_mode, NumericMode::EdgePreview) {
        let (absolute, relative) = policy
            .tolerance
            .map(|tolerance| (tolerance.absolute, tolerance.relative))
            .unwrap_or((1e-12, 1e-12));
        let radius_scale = radius_squared
            .to_f64_lossy()
            .filter(|value| value.is_finite())
            .map(|value| value.abs().max(1.0))
            .unwrap_or(1.0);
        let tolerance = absolute.max(relative * radius_scale);
        return delta
            .to_f64_lossy()
            .filter(|value| value.is_finite())
            .map(|value| value.abs() <= tolerance);
    }

    is_zero(delta, policy)
}
