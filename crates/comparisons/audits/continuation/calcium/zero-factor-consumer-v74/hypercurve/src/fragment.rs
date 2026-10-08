//! Contour fragments produced from split markers.
//!
//! After intersections are inserted, fragment construction rebuilds the
//! source geometry between adjacent split markers so boolean selection can work
//! on atomic boundary pieces. This mirrors polygon clipping's
//! split-then-classify structure, with explicit uncertainty for ordering or
//! finite-preview cases that would otherwise create invalid graph topology.

use std::{cmp::Ordering, sync::Arc};

use hyperreal::Real;

use crate::classify::{
    compare_reals, compare_reals_for_split_ordering, in_closed_unit_interval, is_zero,
};
use crate::segment::LineSupport2;
use crate::{
    CircularArc2, Classification, Contour2, ContourSplitMarkers, CurveContext, CurveError,
    CurveResult, ParamRange, Point2, Segment2, SegmentSplitMarker, UncertaintyReason,
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
        Self::new_with_policy(fragments, &CurveContext::STRICT)
    }

    fn new_with_policy(
        fragments: Vec<ContourFragment>,
        policy: &CurveContext,
    ) -> CurveResult<Self> {
        validate_contour_fragments(&fragments, policy)?;
        Ok(Self { fragments })
    }

    /// Builds fragments from point-bearing contour split markers.
    pub fn from_split_markers(
        contour: &Contour2,
        markers: &ContourSplitMarkers,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Self>> {
        if contour.len() != markers.segment_count() {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        }
        if !markers.source_incidence_certified() {
            match validate_split_markers_against_contour(contour, markers, policy)? {
                Classification::Decided(()) => {}
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            }
        }

        let fragment_capacity = markers
            .segments()
            .iter()
            .map(|markers| markers.len().saturating_sub(1).max(1))
            .sum();
        let mut fragments = Vec::with_capacity(fragment_capacity);
        for (segment_index, source_segment) in contour.segments().iter().enumerate() {
            let Some(segment_markers) = markers.markers_for_segment(segment_index) else {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            };

            if segment_markers.is_empty() {
                fragments.push(ContourFragment {
                    source_segment_index: segment_index,
                    source_segment_start_point: source_segment.start().clone(),
                    source_segment_end_point: source_segment.end().clone(),
                    source_range: ParamRange::new(Real::zero(), Real::one()),
                    segment: source_segment.clone(),
                });
                continue;
            }

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
    policy: &CurveContext,
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
    policy: &CurveContext,
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
    policy: &CurveContext,
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
    policy: &CurveContext,
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
    policy: &CurveContext,
) -> CurveResult<Classification<()>> {
    match source_segment {
        Segment2::Line(line) => {
            let expected = line.point_at(marker.param.clone());
            let distance = marker.point.distance_squared(&expected);
            match point_distance_is_zero(&distance, &marker.point, &expected, policy) {
                Some(true) => Ok(Classification::Decided(())),
                Some(false) if policy.is_edge_preview() => {
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
    policy: &CurveContext,
) -> Option<bool> {
    if is_zero(distance_squared, policy) == Some(true) {
        return Some(true);
    }

    if policy.is_edge_preview()
        && let (Some(distance_squared), Some(left_scale), Some(right_scale)) = (
            distance_squared.to_f64_lossy(),
            point_coordinate_scale(left),
            point_coordinate_scale(right),
        )
        && distance_squared.is_finite()
    {
        let (absolute, relative) = crate::policy::preview_tolerance()
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
    policy: &CurveContext,
) -> CurveResult<Classification<()>> {
    let radius_squared = source_arc.radius_squared();
    let radius_delta = marker.point.distance_squared(source_arc.center()) - &radius_squared;
    match radius_delta_is_zero(&radius_delta, &radius_squared, policy) {
        Some(true) => {}
        Some(false) if policy.is_edge_preview() => {
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
        Classification::Decided(false) if policy.is_edge_preview() => {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        }
        Classification::Decided(false) => {
            return Err(CurveError::Topology(
                "contour split marker point does not lie on source arc sweep".into(),
            ));
        }
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    }

    let expected_param =
        match source_arc.sweep_fraction_for_incident_point(&marker.point, policy)? {
            Classification::Decided(param) => param,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
    match compare_reals(&marker.param, &expected_param, policy) {
        Some(Ordering::Equal) => Ok(Classification::Decided(())),
        Some(_) if policy.is_edge_preview() => {
            Ok(Classification::Uncertain(UncertaintyReason::Unsupported))
        }
        Some(_) => Err(CurveError::Topology(
            "contour split marker parameter does not match source arc chord evidence".into(),
        )),
        None => Ok(Classification::Uncertain(UncertaintyReason::Ordering)),
    }
}

pub(crate) fn split_contour_at_intersections(
    contour: &Contour2,
    intersections: &crate::ContourIntersectionSet,
    operand: crate::ContourOperand,
    policy: &CurveContext,
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
    policy: &CurveContext,
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
    policy: &CurveContext,
) -> CurveResult<Classification<()>> {
    // A validated marker lies at its parameter on the source. Affine line
    // parameters are injective when the source endpoints are already proven
    // distinct, so strict marker ordering also proves distinct fragment
    // endpoints. Arcs retain the geometric check because a full-circle sweep
    // can revisit its start point at a different parameter.
    let ordered_line_markers_are_distinct = matches!(
        source_segment,
        Segment2::Line(line) if line.endpoints_decided_distinct()
    );
    let unsplit_source_segment = markers.len() == 2;
    let line_support = match source_segment {
        Segment2::Line(line) if !unsplit_source_segment => Some(line.fragment_support()),
        _ => None,
    };
    for adjacent in markers.windows(2) {
        let start = &adjacent[0];
        let end = &adjacent[1];

        if !ordered_line_markers_are_distinct {
            let distance = start.point.distance_squared(&end.point);
            match is_zero(&distance, policy) {
                Some(true) => continue,
                Some(false) => {}
                None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
            }
        }

        let segment = match build_fragment_segment(
            source_segment,
            start,
            end,
            unsplit_source_segment,
            line_support.as_ref(),
            policy,
        )? {
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
    unsplit_source_segment: bool,
    line_support: Option<&Arc<LineSupport2>>,
    policy: &CurveContext,
) -> CurveResult<Classification<Segment2>> {
    // Every ContourSplitMarkers constructor certifies a strict sequence from
    // zero to one, and incidence was checked before this call. Two markers
    // therefore denote the complete source without point-equality replay.
    if unsplit_source_segment {
        return Ok(Classification::Decided(source_segment.clone()));
    }

    match source_segment {
        // `append_segment_fragments` has just certified these endpoints as
        // distinct. Preserve that proof and the shared source support instead
        // of asking exact-real arithmetic to prove it a second time.
        Segment2::Line(line) => Ok(Classification::Decided(Segment2::Line(
            line.fragment_between_after_distinct_endpoints(
                start.point.clone(),
                end.point.clone(),
                line_support
                    .expect("split line fragments have prepared source support")
                    .clone(),
            ),
        ))),
        Segment2::Arc(arc) => arc
            .fragment_between_sweep_range(
                start.point.clone(),
                end.point.clone(),
                &ParamRange::new(start.param.clone(), end.param.clone()),
                policy,
            )
            .map(|classification| classification.map(Segment2::Arc)),
    }
}

fn radius_delta_is_zero(
    delta: &Real,
    radius_squared: &Real,
    policy: &CurveContext,
) -> Option<bool> {
    if is_zero(delta, policy) == Some(true) {
        return Some(true);
    }

    if policy.is_edge_preview() {
        let (absolute, relative) = crate::policy::preview_tolerance()
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

#[cfg(test)]
mod tests {
    use std::sync::Arc;

    use super::*;
    use crate::LineSeg2;

    #[test]
    fn native_segment_storage_remains_compact() {
        #[cfg(target_pointer_width = "64")]
        {
            assert!(std::mem::size_of::<Point2>() <= 8);
            assert!(std::mem::size_of::<LineSeg2>() <= 48);
            assert!(std::mem::size_of::<crate::CircularArc2>() <= 8);
            assert!(std::mem::size_of::<Segment2>() <= 48);
        }
    }

    fn point(x: i32) -> Point2 {
        Point2::new(Real::from(x), Real::zero())
    }

    fn fraction(numerator: i32, denominator: i32) -> Real {
        (Real::from(numerator) / Real::from(denominator)).unwrap()
    }

    #[test]
    fn sibling_line_fragments_share_retained_source_support() {
        let source = Segment2::Line(LineSeg2::try_new(point(0), point(6)).unwrap());
        let marker = |coordinate, numerator| SegmentSplitMarker {
            segment_index: 0,
            param: fraction(numerator, 3),
            point: point(coordinate),
        };
        let markers = [marker(0, 0), marker(2, 1), marker(4, 2), marker(6, 3)];
        let policy = CurveContext::STRICT;
        let mut fragments = Vec::new();

        assert!(matches!(
            append_segment_fragments(&mut fragments, &source, 0, &markers, &policy).unwrap(),
            Classification::Decided(())
        ));
        let (Segment2::Line(first), Segment2::Line(last)) =
            (&fragments[0].segment, &fragments[2].segment)
        else {
            panic!("expected line fragments");
        };
        assert_eq!(
            first.retained_support_intervals_decided_disjoint(last, &policy),
            Some(true)
        );
    }

    #[test]
    fn source_line_reuses_support_and_reversal_drops_the_oriented_cache() {
        let source = LineSeg2::try_new(point(0), point(6)).unwrap();
        let original_start = source.start().clone();
        let original_end = source.end().clone();
        let first = source.fragment_support();
        let second = source.fragment_support();

        assert!(Arc::ptr_eq(&first, &second));

        let reversed = source.into_reversed();
        let reversed_support = reversed.fragment_support();
        assert!(!Arc::ptr_eq(&first, &reversed_support));
        let fragment = reversed.fragment_between_after_distinct_endpoints(
            original_end.clone(),
            point(3),
            reversed_support,
        );
        assert_eq!(fragment.support_start(), &original_end);
        assert_eq!(
            fragment.support_delta(),
            original_start.delta_from(&original_end)
        );
    }

    #[test]
    fn retained_support_interval_orders_vertical_reversed_fragments_from_endpoints() {
        let vertical_point = |y| Point2::new(Real::zero(), Real::from(y));
        let source = LineSeg2::try_new(vertical_point(0), vertical_point(6)).unwrap();
        let support = source.fragment_support();
        let first = source
            .fragment_between_after_distinct_endpoints(
                vertical_point(0),
                vertical_point(2),
                support.clone(),
            )
            .into_reversed();
        let last = source.fragment_between_after_distinct_endpoints(
            vertical_point(4),
            vertical_point(6),
            support,
        );

        assert_eq!(
            first.retained_support_intervals_decided_disjoint(&last, &CurveContext::STRICT),
            Some(true)
        );
    }
}
