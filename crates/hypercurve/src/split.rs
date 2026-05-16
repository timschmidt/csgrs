//! Split markers extracted from normalized contour events.

use std::cmp::Ordering;

use hyperlattice::{Backend, DefaultBackend, Scalar};

use crate::classify::{compare_scalars, is_zero};
use crate::{
    Classification, Contour2, ContourIntersection, ContourIntersectionSet, ContourOperand,
    CurvePolicy, Point2, UncertaintyReason,
};

/// A local split parameter on one contour segment.
#[derive(Clone, Debug, PartialEq)]
pub struct SegmentSplitPoint<B: Backend = DefaultBackend> {
    /// Segment index in the source contour.
    pub segment_index: usize,
    /// Local segment parameter.
    pub param: Scalar<B>,
}

/// A local split marker with both ordering parameter and geometric point.
#[derive(Clone, Debug, PartialEq)]
pub struct SegmentSplitMarker<B: Backend = DefaultBackend> {
    /// Segment index in the source contour.
    pub segment_index: usize,
    /// Local segment ordering parameter.
    pub param: Scalar<B>,
    /// Exact split point on the source segment.
    pub point: Point2<B>,
}

/// Point-bearing split markers grouped by source contour segment.
#[derive(Clone, Debug, PartialEq)]
pub struct ContourSplitMarkers<B: Backend = DefaultBackend> {
    segment_markers: Vec<Vec<SegmentSplitMarker<B>>>,
}

impl<B: Backend> ContourSplitMarkers<B> {
    /// Constructs split markers from already-normalized per-segment markers.
    pub const fn new(segment_markers: Vec<Vec<SegmentSplitMarker<B>>>) -> Self {
        Self { segment_markers }
    }

    /// Constructs a marker set containing only each segment's endpoints.
    pub fn with_contour_endpoints(contour: &Contour2<B>) -> Self {
        let mut segment_markers = Vec::with_capacity(contour.len());
        for (segment_index, segment) in contour.segments().iter().enumerate() {
            segment_markers.push(vec![
                SegmentSplitMarker {
                    segment_index,
                    param: Scalar::<B>::zero(),
                    point: segment.start().clone(),
                },
                SegmentSplitMarker {
                    segment_index,
                    param: Scalar::<B>::one(),
                    point: segment.end().clone(),
                },
            ]);
        }

        Self { segment_markers }
    }

    /// Builds split markers from one contour-pair event set.
    pub fn from_intersections(
        contour: &Contour2<B>,
        intersections: &ContourIntersectionSet<B>,
        operand: ContourOperand,
        policy: &CurvePolicy,
    ) -> Classification<Self> {
        let mut markers = Self::with_contour_endpoints(contour);
        match markers.merge_intersections(intersections, operand, policy) {
            Classification::Decided(()) => Classification::Decided(markers),
            Classification::Uncertain(reason) => Classification::Uncertain(reason),
        }
    }

    /// Returns all segment marker bins.
    pub fn segments(&self) -> &[Vec<SegmentSplitMarker<B>>] {
        &self.segment_markers
    }

    /// Returns markers for one segment.
    pub fn markers_for_segment(&self, segment_index: usize) -> Option<&[SegmentSplitMarker<B>]> {
        self.segment_markers
            .get(segment_index)
            .map(std::vec::Vec::as_slice)
    }

    /// Returns the segment count represented by this marker set.
    pub fn segment_count(&self) -> usize {
        self.segment_markers.len()
    }

    /// Returns true when the marker set contains no segments.
    pub fn is_empty(&self) -> bool {
        self.segment_markers.is_empty()
    }

    /// Merges another contour-pair event set into this marker set.
    pub fn merge_intersections(
        &mut self,
        intersections: &ContourIntersectionSet<B>,
        operand: ContourOperand,
        policy: &CurvePolicy,
    ) -> Classification<()> {
        for event in intersections.events() {
            match self.merge_event(event, operand, policy) {
                Ok(()) => {}
                Err(reason) => return Classification::Uncertain(reason),
            }
        }

        Classification::Decided(())
    }

    /// Returns all markers flattened in segment order.
    pub fn split_markers(&self) -> Vec<SegmentSplitMarker<B>> {
        let mut split_markers = Vec::new();
        for markers in &self.segment_markers {
            for marker in markers {
                split_markers.push(marker.clone());
            }
        }
        split_markers
    }

    fn merge_event(
        &mut self,
        event: &ContourIntersection<B>,
        operand: ContourOperand,
        policy: &CurvePolicy,
    ) -> Result<(), UncertaintyReason> {
        match event {
            ContourIntersection::Point(point) => {
                let (segment_index, param) = match operand {
                    ContourOperand::First => (point.a_segment_index, point.a_param.clone()),
                    ContourOperand::Second => (point.b_segment_index, point.b_param.clone()),
                };
                self.insert_marker(
                    SegmentSplitMarker {
                        segment_index,
                        param,
                        point: point.point.clone(),
                    },
                    policy,
                )
            }
            ContourIntersection::Overlap(overlap) => {
                let (segment_index, start_param, end_param) = match operand {
                    ContourOperand::First => (
                        overlap.a_segment_index,
                        overlap.a_range.start().clone(),
                        overlap.a_range.end().clone(),
                    ),
                    ContourOperand::Second => (
                        overlap.b_segment_index,
                        overlap.b_range.start().clone(),
                        overlap.b_range.end().clone(),
                    ),
                };
                self.insert_marker(
                    SegmentSplitMarker {
                        segment_index,
                        param: start_param,
                        point: overlap.segment.start().clone(),
                    },
                    policy,
                )?;
                self.insert_marker(
                    SegmentSplitMarker {
                        segment_index,
                        param: end_param,
                        point: overlap.segment.end().clone(),
                    },
                    policy,
                )
            }
            ContourIntersection::Uncertain(uncertain) => Err(uncertain.reason),
        }
    }

    fn insert_marker(
        &mut self,
        marker: SegmentSplitMarker<B>,
        policy: &CurvePolicy,
    ) -> Result<(), UncertaintyReason> {
        let Some(markers) = self.segment_markers.get_mut(marker.segment_index) else {
            return Err(UncertaintyReason::Unsupported);
        };

        insert_unique_sorted_marker(markers, marker, policy)
    }
}

/// Per-segment split parameters for a contour.
///
/// Every segment starts with `0` and `1` so downstream fragment assembly can
/// build intervals directly after event parameters are merged in.
#[derive(Clone, Debug, PartialEq)]
pub struct ContourSplitMap<B: Backend = DefaultBackend> {
    segment_splits: Vec<Vec<Scalar<B>>>,
}

impl<B: Backend> ContourSplitMap<B> {
    /// Constructs a split map from already-normalized per-segment parameters.
    pub const fn new(segment_splits: Vec<Vec<Scalar<B>>>) -> Self {
        Self { segment_splits }
    }

    /// Constructs a split map containing only segment endpoints.
    pub fn with_segment_count(segment_count: usize) -> Self {
        let mut segment_splits = Vec::with_capacity(segment_count);
        for _ in 0..segment_count {
            segment_splits.push(vec![Scalar::<B>::zero(), Scalar::<B>::one()]);
        }
        Self { segment_splits }
    }

    /// Builds a split map from one contour-pair event set.
    pub fn from_intersections(
        segment_count: usize,
        intersections: &ContourIntersectionSet<B>,
        operand: ContourOperand,
        policy: &CurvePolicy,
    ) -> Classification<Self> {
        let mut map = Self::with_segment_count(segment_count);
        match map.merge_intersections(intersections, operand, policy) {
            Classification::Decided(()) => Classification::Decided(map),
            Classification::Uncertain(reason) => Classification::Uncertain(reason),
        }
    }

    /// Returns the split parameters for all segments.
    pub fn segments(&self) -> &[Vec<Scalar<B>>] {
        &self.segment_splits
    }

    /// Returns the split parameters for one segment.
    pub fn params_for_segment(&self, segment_index: usize) -> Option<&[Scalar<B>]> {
        self.segment_splits
            .get(segment_index)
            .map(std::vec::Vec::as_slice)
    }

    /// Returns the segment count represented by this map.
    pub fn segment_count(&self) -> usize {
        self.segment_splits.len()
    }

    /// Returns true when the map contains no segments.
    pub fn is_empty(&self) -> bool {
        self.segment_splits.is_empty()
    }

    /// Merges another contour-pair event set into this split map.
    pub fn merge_intersections(
        &mut self,
        intersections: &ContourIntersectionSet<B>,
        operand: ContourOperand,
        policy: &CurvePolicy,
    ) -> Classification<()> {
        for event in intersections.events() {
            match self.merge_event(event, operand, policy) {
                Ok(()) => {}
                Err(reason) => return Classification::Uncertain(reason),
            }
        }

        Classification::Decided(())
    }

    /// Returns all split points flattened in segment order.
    pub fn split_points(&self) -> Vec<SegmentSplitPoint<B>> {
        let mut split_points = Vec::new();
        for (segment_index, params) in self.segment_splits.iter().enumerate() {
            for param in params {
                split_points.push(SegmentSplitPoint {
                    segment_index,
                    param: param.clone(),
                });
            }
        }
        split_points
    }

    fn merge_event(
        &mut self,
        event: &ContourIntersection<B>,
        operand: ContourOperand,
        policy: &CurvePolicy,
    ) -> Result<(), UncertaintyReason> {
        match event {
            ContourIntersection::Point(point) => {
                let (segment_index, param) = match operand {
                    ContourOperand::First => (point.a_segment_index, point.a_param.clone()),
                    ContourOperand::Second => (point.b_segment_index, point.b_param.clone()),
                };
                self.insert_param(segment_index, param, policy)
            }
            ContourIntersection::Overlap(overlap) => {
                let (segment_index, start, end) = match operand {
                    ContourOperand::First => (
                        overlap.a_segment_index,
                        overlap.a_range.start().clone(),
                        overlap.a_range.end().clone(),
                    ),
                    ContourOperand::Second => (
                        overlap.b_segment_index,
                        overlap.b_range.start().clone(),
                        overlap.b_range.end().clone(),
                    ),
                };
                self.insert_param(segment_index, start, policy)?;
                self.insert_param(segment_index, end, policy)
            }
            ContourIntersection::Uncertain(uncertain) => Err(uncertain.reason),
        }
    }

    fn insert_param(
        &mut self,
        segment_index: usize,
        param: Scalar<B>,
        policy: &CurvePolicy,
    ) -> Result<(), UncertaintyReason> {
        let Some(params) = self.segment_splits.get_mut(segment_index) else {
            return Err(UncertaintyReason::Unsupported);
        };

        insert_unique_sorted(params, param, policy)
    }
}

fn insert_unique_sorted<B: Backend>(
    params: &mut Vec<Scalar<B>>,
    param: Scalar<B>,
    policy: &CurvePolicy,
) -> Result<(), UncertaintyReason> {
    for index in 0..params.len() {
        match compare_scalars(&param, &params[index], policy) {
            Some(Ordering::Equal) => return Ok(()),
            Some(Ordering::Less) => {
                params.insert(index, param);
                return Ok(());
            }
            Some(Ordering::Greater) => {}
            None => return Err(UncertaintyReason::Ordering),
        }
    }

    params.push(param);
    Ok(())
}

fn insert_unique_sorted_marker<B: Backend>(
    markers: &mut Vec<SegmentSplitMarker<B>>,
    marker: SegmentSplitMarker<B>,
    policy: &CurvePolicy,
) -> Result<(), UncertaintyReason> {
    for index in 0..markers.len() {
        match compare_scalars(&marker.param, &markers[index].param, policy) {
            Some(Ordering::Equal) => {
                let distance = marker.point.distance_squared(&markers[index].point);
                return match is_zero(&distance, policy) {
                    Some(true) => Ok(()),
                    Some(false) => Err(UncertaintyReason::Unsupported),
                    None => Err(UncertaintyReason::ScalarSign),
                };
            }
            Some(Ordering::Less) => {
                markers.insert(index, marker);
                return Ok(());
            }
            Some(Ordering::Greater) => {}
            None => return Err(UncertaintyReason::Ordering),
        }
    }

    markers.push(marker);
    Ok(())
}
