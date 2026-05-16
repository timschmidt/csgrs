//! Contour fragments produced from split markers.

use std::cmp::Ordering;

use hyperlattice::{Backend, DefaultBackend};

use crate::classify::{compare_scalars, is_zero};
use crate::{
    CircularArc2, Classification, Contour2, ContourSplitMarkers, CurvePolicy, CurveResult,
    LineSeg2, ParamRange, Segment2, SegmentSplitMarker, UncertaintyReason,
};

/// One source-contour fragment between adjacent split markers.
#[derive(Clone, Debug, PartialEq)]
pub struct ContourFragment<B: Backend = DefaultBackend> {
    /// Source segment index in the original contour.
    pub source_segment_index: usize,
    /// Parameter interval on the source segment.
    pub source_range: ParamRange<B>,
    /// Fragment geometry in source traversal direction.
    pub segment: Segment2<B>,
}

/// Ordered fragments from a split contour.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct ContourFragmentSet<B: Backend = DefaultBackend> {
    fragments: Vec<ContourFragment<B>>,
}

impl<B: Backend> ContourFragmentSet<B> {
    /// Constructs a fragment set from already-built fragments.
    pub const fn new(fragments: Vec<ContourFragment<B>>) -> Self {
        Self { fragments }
    }

    /// Builds fragments from point-bearing contour split markers.
    pub fn from_split_markers(
        contour: &Contour2<B>,
        markers: &ContourSplitMarkers<B>,
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

        Ok(Classification::Decided(Self { fragments }))
    }

    /// Returns fragments in contour traversal order.
    pub fn fragments(&self) -> &[ContourFragment<B>] {
        &self.fragments
    }

    /// Consumes the set and returns the fragments.
    pub fn into_fragments(self) -> Vec<ContourFragment<B>> {
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

pub(crate) fn split_contour_at_intersections<B: Backend>(
    contour: &Contour2<B>,
    intersections: &crate::ContourIntersectionSet<B>,
    operand: crate::ContourOperand,
    policy: &CurvePolicy,
) -> CurveResult<Classification<ContourFragmentSet<B>>> {
    let markers =
        match ContourSplitMarkers::from_intersections(contour, intersections, operand, policy) {
            Classification::Decided(markers) => markers,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };

    ContourFragmentSet::from_split_markers(contour, &markers, policy)
}

fn append_segment_fragments<B: Backend>(
    fragments: &mut Vec<ContourFragment<B>>,
    source_segment: &Segment2<B>,
    segment_index: usize,
    markers: &[SegmentSplitMarker<B>],
    policy: &CurvePolicy,
) -> CurveResult<Classification<()>> {
    for adjacent in markers.windows(2) {
        let start = &adjacent[0];
        let end = &adjacent[1];

        match compare_scalars(&start.param, &end.param, policy) {
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
            None => return Ok(Classification::Uncertain(UncertaintyReason::ScalarSign)),
        }

        let segment = build_fragment_segment(source_segment, start, end)?;
        fragments.push(ContourFragment {
            source_segment_index: segment_index,
            source_range: ParamRange::new(start.param.clone(), end.param.clone()),
            segment,
        });
    }

    Ok(Classification::Decided(()))
}

fn build_fragment_segment<B: Backend>(
    source_segment: &Segment2<B>,
    start: &SegmentSplitMarker<B>,
    end: &SegmentSplitMarker<B>,
) -> CurveResult<Segment2<B>> {
    if start.point == *source_segment.start() && end.point == *source_segment.end() {
        return Ok(source_segment.clone());
    }

    match source_segment {
        Segment2::Line(_) => {
            LineSeg2::try_new(start.point.clone(), end.point.clone()).map(Segment2::Line)
        }
        Segment2::Arc(arc) => CircularArc2::try_from_center(
            start.point.clone(),
            end.point.clone(),
            arc.center().clone(),
            arc.is_clockwise(),
        )
        .map(Segment2::Arc),
    }
}
