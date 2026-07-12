//! Split markers extracted from normalized contour events.
//!
//! This is the intersection-insertion stage used by later boolean selection:
//! every event becomes an ordered marker on each affected source segment before
//! fragments are rebuilt. Greiner and Hormann's clipping pipeline uses the same
//! "insert intersections, then traverse classified pieces" shape in "Efficient
//! Clipping of Arbitrary Polygons" (*ACM Transactions on Graphics* 17(2),
//! 71-83, 1998). Finite display output can disturb marker order, so the
//! preview-only comparator is intentionally isolated here; see Hobby,
//! "Practical Segment Intersection with Finite Precision Output"
//! (*Computational Geometry* 13(4), 199-214, 1999).

use std::cmp::Ordering;

use hyperreal::Real;

use crate::classify::{
    compare_reals, compare_reals_for_split_ordering, in_closed_unit_interval, is_zero,
};
use crate::{
    Classification, Contour2, ContourIntersection, ContourIntersectionSet, ContourOperand,
    CurveError, CurvePolicy, CurveResult, Point2, UncertaintyReason,
};

/// A local split parameter on one contour segment.
#[derive(Clone, Debug, PartialEq)]
pub struct SegmentSplitPoint {
    /// Segment index in the source contour.
    pub segment_index: usize,
    /// Local segment parameter.
    pub param: Real,
}

/// A local split marker with both ordering parameter and geometric point.
#[derive(Clone, Debug, PartialEq)]
pub struct SegmentSplitMarker {
    /// Segment index in the source contour.
    pub segment_index: usize,
    /// Local segment ordering parameter.
    pub param: Real,
    /// Exact split point on the source segment.
    pub point: Point2,
}

/// Point-bearing split markers grouped by source contour segment.
#[derive(Clone, Debug, PartialEq)]
pub struct ContourSplitMarkers {
    segment_markers: Vec<Vec<SegmentSplitMarker>>,
    source_incidence_certified: bool,
}

impl ContourSplitMarkers {
    /// Constructs split markers from already-normalized per-segment markers.
    pub fn new(segment_markers: Vec<Vec<SegmentSplitMarker>>) -> CurveResult<Self> {
        validate_split_markers(&segment_markers)?;
        Ok(Self {
            segment_markers,
            source_incidence_certified: false,
        })
    }

    /// Constructs a marker set containing only each segment's endpoints.
    pub fn with_contour_endpoints(contour: &Contour2) -> Self {
        let mut segment_markers = Vec::with_capacity(contour.len());
        for (segment_index, segment) in contour.segments().iter().enumerate() {
            segment_markers.push(vec![
                SegmentSplitMarker {
                    segment_index,
                    param: Real::zero(),
                    point: segment.start().clone(),
                },
                SegmentSplitMarker {
                    segment_index,
                    param: Real::one(),
                    point: segment.end().clone(),
                },
            ]);
        }

        Self {
            segment_markers,
            source_incidence_certified: true,
        }
    }

    /// Builds split markers from one contour-pair event set.
    pub fn from_intersections(
        contour: &Contour2,
        intersections: &ContourIntersectionSet,
        operand: ContourOperand,
        policy: &CurvePolicy,
    ) -> Classification<Self> {
        let mut markers = Self::with_contour_endpoints(contour);
        match markers.merge_intersections(intersections, operand, policy) {
            Classification::Decided(()) => Classification::Decided(markers),
            Classification::Uncertain(reason) => Classification::Uncertain(reason),
        }
    }

    /// Builds split markers from self-intersection events on one contour.
    ///
    /// Each retained event contributes markers for both participating source
    /// segments. Ordinary adjacent connectivity points should already have
    /// been filtered by the self-intersection collector.
    pub fn from_self_intersections(
        contour: &Contour2,
        intersections: &ContourIntersectionSet,
        policy: &CurvePolicy,
    ) -> Classification<Self> {
        let mut markers = Self::with_contour_endpoints(contour);
        match markers.merge_self_intersections(intersections, policy) {
            Classification::Decided(()) => Classification::Decided(markers),
            Classification::Uncertain(reason) => Classification::Uncertain(reason),
        }
    }

    /// Returns all segment marker bins.
    pub fn segments(&self) -> &[Vec<SegmentSplitMarker>] {
        &self.segment_markers
    }

    /// Returns markers for one segment.
    pub fn markers_for_segment(&self, segment_index: usize) -> Option<&[SegmentSplitMarker]> {
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

    pub(crate) const fn source_incidence_certified(&self) -> bool {
        self.source_incidence_certified
    }

    /// Merges another contour-pair event set into this marker set.
    pub fn merge_intersections(
        &mut self,
        intersections: &ContourIntersectionSet,
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

    /// Merges self-intersection events into this marker set.
    pub fn merge_self_intersections(
        &mut self,
        intersections: &ContourIntersectionSet,
        policy: &CurvePolicy,
    ) -> Classification<()> {
        for event in intersections.events() {
            match self.merge_event(event, ContourOperand::First, policy) {
                Ok(()) => {}
                Err(reason) => return Classification::Uncertain(reason),
            }
            match self.merge_event(event, ContourOperand::Second, policy) {
                Ok(()) => {}
                Err(reason) => return Classification::Uncertain(reason),
            }
        }

        Classification::Decided(())
    }

    /// Returns all markers flattened in segment order.
    pub fn split_markers(&self) -> Vec<SegmentSplitMarker> {
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
        event: &ContourIntersection,
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
        marker: SegmentSplitMarker,
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
pub struct ContourSplitMap {
    segment_splits: Vec<Vec<Real>>,
}

impl ContourSplitMap {
    /// Constructs a split map from already-normalized per-segment parameters.
    pub fn new(segment_splits: Vec<Vec<Real>>) -> CurveResult<Self> {
        validate_split_params(&segment_splits)?;
        Ok(Self { segment_splits })
    }

    /// Constructs a split map containing only segment endpoints.
    pub fn with_segment_count(segment_count: usize) -> Self {
        let mut segment_splits = Vec::with_capacity(segment_count);
        for _ in 0..segment_count {
            segment_splits.push(vec![Real::zero(), Real::one()]);
        }
        Self { segment_splits }
    }

    /// Builds a split map from one contour-pair event set.
    pub fn from_intersections(
        segment_count: usize,
        intersections: &ContourIntersectionSet,
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
    pub fn segments(&self) -> &[Vec<Real>] {
        &self.segment_splits
    }

    /// Returns the split parameters for one segment.
    pub fn params_for_segment(&self, segment_index: usize) -> Option<&[Real]> {
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
        intersections: &ContourIntersectionSet,
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
    pub fn split_points(&self) -> Vec<SegmentSplitPoint> {
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
        event: &ContourIntersection,
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
        param: Real,
        policy: &CurvePolicy,
    ) -> Result<(), UncertaintyReason> {
        let Some(params) = self.segment_splits.get_mut(segment_index) else {
            return Err(UncertaintyReason::Unsupported);
        };

        insert_unique_sorted(params, param, policy)
    }
}

fn insert_unique_sorted(
    params: &mut Vec<Real>,
    param: Real,
    policy: &CurvePolicy,
) -> Result<(), UncertaintyReason> {
    for index in 0..params.len() {
        match compare_reals_for_split_ordering(&param, &params[index], policy) {
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

fn insert_unique_sorted_marker(
    markers: &mut Vec<SegmentSplitMarker>,
    marker: SegmentSplitMarker,
    policy: &CurvePolicy,
) -> Result<(), UncertaintyReason> {
    for index in 0..markers.len() {
        match compare_reals_for_split_ordering(&marker.param, &markers[index].param, policy) {
            Some(Ordering::Equal) => {
                let distance = marker.point.distance_squared(&markers[index].point);
                // Equal parameters must also be the same geometric marker. Two
                // different points at one parameter would represent a broken
                // split graph, the kind of degeneracy Greiner-Hormann style
                // traversal cannot resolve by local ordering alone.
                return match is_zero(&distance, policy) {
                    Some(true) => Ok(()),
                    Some(false) => Err(UncertaintyReason::Unsupported),
                    None => Err(UncertaintyReason::RealSign),
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

fn validate_split_params(segment_splits: &[Vec<Real>]) -> CurveResult<()> {
    if segment_splits.is_empty() {
        return Err(CurveError::Topology(
            "contour split evidence must carry at least one source segment".to_owned(),
        ));
    }
    for params in segment_splits {
        validate_split_param_sequence(params)?;
    }
    Ok(())
}

fn validate_split_markers(segment_markers: &[Vec<SegmentSplitMarker>]) -> CurveResult<()> {
    if segment_markers.is_empty() {
        return Err(CurveError::Topology(
            "contour split marker evidence must carry at least one source segment".to_owned(),
        ));
    }
    for (segment_index, markers) in segment_markers.iter().enumerate() {
        validate_split_marker_sequence(segment_index, markers)?;
    }
    Ok(())
}

fn validate_split_marker_sequence(
    segment_index: usize,
    markers: &[SegmentSplitMarker],
) -> CurveResult<()> {
    if markers
        .iter()
        .any(|marker| marker.segment_index != segment_index)
    {
        return Err(CurveError::Topology(
            "contour split marker bin carries mismatched segment index evidence".to_owned(),
        ));
    }
    validate_split_param_sequence(markers.iter().map(|marker| &marker.param))
}

fn validate_split_param_sequence<'a>(
    params: impl IntoIterator<Item = &'a Real>,
) -> CurveResult<()> {
    let params = params.into_iter().collect::<Vec<_>>();
    if params.len() < 2 {
        return Err(CurveError::Topology(
            "contour split evidence must include both segment endpoints".to_owned(),
        ));
    }

    let policy = CurvePolicy::certified();
    if compare_reals(params[0], &Real::zero(), &policy) != Some(Ordering::Equal)
        || compare_reals(params[params.len() - 1], &Real::one(), &policy) != Some(Ordering::Equal)
    {
        return Err(CurveError::Topology(
            "contour split evidence must start at 0 and end at 1".to_owned(),
        ));
    }

    for param in &params {
        if in_closed_unit_interval(param, &policy) != Some(true) {
            return Err(CurveError::Topology(
                "contour split parameter evidence must lie inside the unit interval".to_owned(),
            ));
        }
    }

    for window in params.windows(2) {
        match compare_reals(window[0], window[1], &policy) {
            Some(Ordering::Less) => {}
            Some(Ordering::Equal | Ordering::Greater) => {
                return Err(CurveError::Topology(
                    "contour split parameter evidence must be strictly increasing".to_owned(),
                ));
            }
            None => {
                return Err(CurveError::Topology(
                    "contour split parameter ordering must be certified".to_owned(),
                ));
            }
        }
    }

    Ok(())
}
