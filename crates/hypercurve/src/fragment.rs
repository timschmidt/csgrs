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

use crate::classify::{compare_reals_for_split_ordering, is_zero};
use crate::{
    CircularArc2, Classification, Contour2, ContourSplitMarkers, CurveError, CurvePolicy,
    CurveResult, LineSeg2, NumericMode, ParamRange, Segment2, SegmentSplitMarker,
    UncertaintyReason,
};

/// One source-contour fragment between adjacent split markers.
#[derive(Clone, Debug, PartialEq)]
pub struct ContourFragment {
    /// Source segment index in the original contour.
    pub source_segment_index: usize,
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
        validate_contour_fragments(&fragments)?;
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

        Ok(Classification::Decided(Self::new(fragments)?))
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

fn validate_contour_fragments(fragments: &[ContourFragment]) -> CurveResult<()> {
    for (left_index, left) in fragments.iter().enumerate() {
        if fragments[left_index + 1..]
            .iter()
            .any(|right| right == left)
        {
            return Err(CurveError::Topology(
                "contour fragment set must not contain duplicate fragments".into(),
            ));
        }
    }
    Ok(())
}

pub(crate) fn split_contour_at_intersections(
    contour: &Contour2,
    intersections: &crate::ContourIntersectionSet,
    operand: crate::ContourOperand,
    policy: &CurvePolicy,
) -> CurveResult<Classification<ContourFragmentSet>> {
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
    let markers = match ContourSplitMarkers::from_self_intersections(contour, intersections, policy)
    {
        Classification::Decided(markers) => markers,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };

    ContourFragmentSet::from_split_markers(contour, &markers, policy)
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
        Segment2::Line(_) => LineSeg2::try_new(start.point.clone(), end.point.clone())
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
