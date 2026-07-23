//! Contour fragments produced from split markers.
//!
//! After intersections are inserted, fragment construction rebuilds the
//! source geometry between adjacent split markers so boolean selection can work
//! on atomic boundary pieces. This mirrors polygon clipping's
//! split-then-classify structure, with explicit uncertainty for ordering or
//! finite-preview cases that would otherwise create invalid graph topology.

use std::{cmp::Ordering, rc::Rc};

use hyperreal::Real;

use crate::classify::{
    compare_reals, compare_reals_for_split_ordering, in_closed_unit_interval, is_zero,
};
use crate::segment::LineSupport2;
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

#[derive(Debug)]
pub(crate) struct CompactLineContourFragment {
    pub(crate) source_segment_index: usize,
    geometry: CompactLineFragmentGeometry,
}

#[derive(Debug)]
enum CompactLineFragmentGeometry {
    Whole,
    Split {
        markers: Rc<ContourSplitMarkers>,
        marker_index: usize,
        source_support: Rc<LineSupport2>,
    },
}

impl CompactLineContourFragment {
    pub(crate) fn endpoints<'a>(
        &'a self,
        source_segment: &'a Segment2,
    ) -> CurveResult<(&'a Point2, &'a Point2)> {
        let Segment2::Line(source_line) = source_segment else {
            return Err(CurveError::Topology(
                "compact line fragment references a non-line source segment".into(),
            ));
        };
        Ok(match &self.geometry {
            CompactLineFragmentGeometry::Whole => (source_line.start(), source_line.end()),
            CompactLineFragmentGeometry::Split {
                markers,
                marker_index,
                ..
            } => {
                let segment_markers = markers
                    .markers_for_segment(self.source_segment_index)
                    .expect("compact fragment marker segment was validated during construction");
                (
                    &segment_markers[*marker_index].point,
                    &segment_markers[*marker_index + 1].point,
                )
            }
        })
    }

    pub(crate) fn source_parameters<'a>(
        &'a self,
        full_start: &'a Real,
        full_end: &'a Real,
    ) -> (&'a Real, &'a Real) {
        match &self.geometry {
            CompactLineFragmentGeometry::Whole => (full_start, full_end),
            CompactLineFragmentGeometry::Split {
                markers,
                marker_index,
                ..
            } => {
                let segment_markers = markers
                    .markers_for_segment(self.source_segment_index)
                    .expect("compact fragment marker segment was validated during construction");
                (
                    &segment_markers[*marker_index].param,
                    &segment_markers[*marker_index + 1].param,
                )
            }
        }
    }

    pub(crate) fn into_source_range(self) -> ParamRange {
        match self.geometry {
            CompactLineFragmentGeometry::Whole => ParamRange::new(Real::zero(), Real::one()),
            CompactLineFragmentGeometry::Split {
                markers,
                marker_index,
                ..
            } => {
                let segment_markers = markers
                    .markers_for_segment(self.source_segment_index)
                    .expect("compact fragment marker segment was validated during construction");
                ParamRange::new(
                    segment_markers[marker_index].param.clone(),
                    segment_markers[marker_index + 1].param.clone(),
                )
            }
        }
    }

    /// Builds full line geometry only after Boolean selection retains this fragment.
    pub(crate) fn materialize(&self, source_segment: &Segment2) -> CurveResult<Segment2> {
        let Segment2::Line(source_line) = source_segment else {
            return Err(CurveError::Topology(
                "compact line fragment references a non-line source segment".into(),
            ));
        };
        Ok(Segment2::Line(match &self.geometry {
            CompactLineFragmentGeometry::Split {
                markers,
                marker_index,
                source_support,
            } => {
                let segment_markers = markers
                    .markers_for_segment(self.source_segment_index)
                    .expect("compact fragment marker segment was validated during construction");
                source_line.fragment_between_with_source_range_after_distinct_endpoints(
                    segment_markers[*marker_index].point.clone(),
                    segment_markers[*marker_index + 1].point.clone(),
                    ParamRange::new(
                        segment_markers[*marker_index].param.clone(),
                        segment_markers[*marker_index + 1].param.clone(),
                    ),
                    source_support.clone(),
                )
            }
            CompactLineFragmentGeometry::Whole => source_line.clone(),
        }))
    }
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

pub(crate) fn compact_line_contour_fragments_from_split_markers(
    contour: &Contour2,
    markers: ContourSplitMarkers,
    policy: &CurvePolicy,
) -> CurveResult<Classification<Vec<CompactLineContourFragment>>> {
    if contour.len() != markers.segment_count() {
        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
    }
    if !markers.source_incidence_certified() {
        match validate_split_markers_against_contour(contour, &markers, policy)? {
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
    let markers = Rc::new(markers);
    for (segment_index, source_segment) in contour.segments().iter().enumerate() {
        let segment_markers = markers
            .markers_for_segment(segment_index)
            .expect("compact fragment marker count was validated before construction");
        let Segment2::Line(source_line) = source_segment else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        if segment_markers.is_empty() {
            fragments.push(CompactLineContourFragment {
                source_segment_index: segment_index,
                geometry: CompactLineFragmentGeometry::Whole,
            });
            continue;
        }

        // Strictly ordered parameters certify distinct adjacent points only
        // when the source line itself is already certified non-degenerate.
        // Preserve the general fragment builder's exact check for unchecked
        // or symbolically inconclusive source lines.
        let markers_are_distinct = source_line.endpoints_decided_distinct();
        if segment_markers.len() == 2 {
            if !markers_are_distinct {
                match is_zero(
                    &segment_markers[0]
                        .point
                        .distance_squared(&segment_markers[1].point),
                    policy,
                ) {
                    Some(true) => continue,
                    Some(false) => {}
                    None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
                }
            }
            fragments.push(CompactLineContourFragment {
                source_segment_index: segment_index,
                geometry: CompactLineFragmentGeometry::Whole,
            });
            continue;
        }

        let source_support = source_line.fragment_support();
        for marker_index in 0..segment_markers.len() - 1 {
            if !markers_are_distinct {
                match is_zero(
                    &segment_markers[marker_index]
                        .point
                        .distance_squared(&segment_markers[marker_index + 1].point),
                    policy,
                ) {
                    Some(true) => continue,
                    Some(false) => {}
                    None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
                }
            }
            fragments.push(CompactLineContourFragment {
                source_segment_index: segment_index,
                geometry: CompactLineFragmentGeometry::Split {
                    markers: markers.clone(),
                    marker_index,
                    source_support: source_support.clone(),
                },
            });
        }
    }

    Ok(Classification::Decided(fragments))
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

    let expected_param =
        match source_arc.sweep_fraction_for_incident_point(&marker.point, policy)? {
            Classification::Decided(param) => param,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
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
    line_support: Option<&Rc<LineSupport2>>,
    policy: &CurvePolicy,
) -> CurveResult<Classification<Segment2>> {
    // Every ContourSplitMarkers constructor certifies a strict sequence from
    // zero to one, and incidence was checked before this call. Two markers
    // therefore denote the complete source without point-equality replay.
    if unsplit_source_segment {
        return Ok(Classification::Decided(source_segment.clone()));
    }

    match source_segment {
        // `append_segment_fragments` has just certified these endpoints as
        // distinct. Preserve that proof while retaining the source range
        // instead of asking exact-real arithmetic to prove it a second time.
        Segment2::Line(line) => Ok(Classification::Decided(Segment2::Line(
            line.fragment_between_with_source_range_after_distinct_endpoints(
                start.point.clone(),
                end.point.clone(),
                ParamRange::new(start.param.clone(), end.param.clone()),
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::LineSeg2;

    #[test]
    fn compact_line_fragment_is_smaller_than_mixed_segment_geometry() {
        assert!(
            std::mem::size_of::<CompactLineContourFragment>() < std::mem::size_of::<Segment2>()
        );
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
        let policy = CurvePolicy::certified();
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
            first.retained_support_ranges_decided_disjoint(last, &policy),
            Some(true)
        );
    }

    #[test]
    fn compact_line_fragments_borrow_shared_markers_until_selection() {
        let p = |x, y| Point2::new(Real::from(x), Real::from(y));
        let contour = Contour2::try_new(vec![
            Segment2::Line(LineSeg2::try_new(p(0, 0), p(4, 0)).unwrap()),
            Segment2::Line(LineSeg2::try_new(p(4, 0), p(4, 4)).unwrap()),
            Segment2::Line(LineSeg2::try_new(p(4, 4), p(0, 4)).unwrap()),
            Segment2::Line(LineSeg2::try_new(p(0, 4), p(0, 0)).unwrap()),
        ])
        .unwrap();
        let markers = ContourSplitMarkers::new(vec![
            vec![
                SegmentSplitMarker {
                    segment_index: 0,
                    param: Real::zero(),
                    point: p(0, 0),
                },
                SegmentSplitMarker {
                    segment_index: 0,
                    param: fraction(1, 2),
                    point: p(2, 0),
                },
                SegmentSplitMarker {
                    segment_index: 0,
                    param: Real::one(),
                    point: p(4, 0),
                },
            ],
            vec![
                SegmentSplitMarker {
                    segment_index: 1,
                    param: Real::zero(),
                    point: p(4, 0),
                },
                SegmentSplitMarker {
                    segment_index: 1,
                    param: Real::one(),
                    point: p(4, 4),
                },
            ],
            vec![
                SegmentSplitMarker {
                    segment_index: 2,
                    param: Real::zero(),
                    point: p(4, 4),
                },
                SegmentSplitMarker {
                    segment_index: 2,
                    param: Real::one(),
                    point: p(0, 4),
                },
            ],
            vec![
                SegmentSplitMarker {
                    segment_index: 3,
                    param: Real::zero(),
                    point: p(0, 4),
                },
                SegmentSplitMarker {
                    segment_index: 3,
                    param: Real::one(),
                    point: p(0, 0),
                },
            ],
        ])
        .unwrap();
        let policy = CurvePolicy::certified();
        let Classification::Decided(fragments) =
            compact_line_contour_fragments_from_split_markers(&contour, markers, &policy).unwrap()
        else {
            panic!("valid exact line markers should materialize compact fragments");
        };

        assert_eq!(fragments.len(), 5);
        let (start, end) = fragments[0].endpoints(&contour.segments()[0]).unwrap();
        assert_eq!((start, end), (&p(0, 0), &p(2, 0)));
        let zero = Real::zero();
        let one = Real::one();
        assert_eq!(
            fragments[0].source_parameters(&zero, &one),
            (&zero, &fraction(1, 2))
        );
        let (whole_start, whole_end) = fragments[2].endpoints(&contour.segments()[1]).unwrap();
        assert_eq!((whole_start, whole_end), (&p(4, 0), &p(4, 4)));
        assert_eq!(fragments[2].source_parameters(&zero, &one), (&zero, &one));

        let Segment2::Line(first) = fragments[0].materialize(&contour.segments()[0]).unwrap()
        else {
            panic!("compact line fragment should materialize a line");
        };
        let Segment2::Line(second) = fragments[1].materialize(&contour.segments()[0]).unwrap()
        else {
            panic!("compact line fragment should materialize a line");
        };
        assert_eq!(
            first.retained_support_ranges_decided_disjoint(&second, &policy),
            Some(false)
        );
    }
}
