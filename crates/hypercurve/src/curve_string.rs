//! Ordered open curve strings.

use crate::bbox::{Aabb2, aabbs_decided_disjoint, decided_segment_aabb};
use crate::{
    BulgeVertex2, CurveError, CurvePolicy, CurveResult, Point2, Segment2, SegmentIntersection,
};

/// One segment-pair event between two curve strings.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveStringIntersection {
    /// Segment index in the first curve string.
    pub a_segment_index: usize,
    /// Segment index in the second curve string.
    pub b_segment_index: usize,
    /// Segment relation for this pair.
    pub relation: SegmentIntersection,
}

/// An ordered sequence of connected native segments.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveString2 {
    segments: Vec<Segment2>,
}

impl CurveString2 {
    /// Constructs a curve string from validated connected segments.
    pub fn try_new(segments: Vec<Segment2>) -> CurveResult<Self> {
        if segments.is_empty() {
            return Err(CurveError::EmptyCurveString);
        }

        for adjacent in segments.windows(2) {
            let distance = adjacent[0].end().distance_squared(adjacent[1].start());
            match distance.zero_status() {
                hyperreal::ZeroKnowledge::Zero => {}
                hyperreal::ZeroKnowledge::NonZero => {
                    return Err(CurveError::DisconnectedCurveString);
                }
                hyperreal::ZeroKnowledge::Unknown => {
                    return Err(CurveError::AmbiguousCurveStringConnection);
                }
            }
        }

        Ok(Self { segments })
    }

    /// Constructs a curve string without checking connectivity.
    pub const fn new_unchecked(segments: Vec<Segment2>) -> Self {
        Self { segments }
    }

    /// Constructs an open curve string from exact bulge vertices.
    pub fn from_bulge_vertices(vertices: &[BulgeVertex2]) -> CurveResult<Self> {
        if vertices.len() < 2 {
            return Err(CurveError::InsufficientVertices);
        }

        let mut segments = Vec::with_capacity(vertices.len() - 1);
        for adjacent in vertices.windows(2) {
            segments.push(adjacent[0].segment_to(&adjacent[1])?);
        }
        Self::try_new(segments)
    }

    /// Returns the segments in order.
    pub fn segments(&self) -> &[Segment2] {
        &self.segments
    }

    /// Consumes the curve string and returns its segments.
    pub fn into_segments(self) -> Vec<Segment2> {
        self.segments
    }

    /// Returns the segment count.
    pub fn len(&self) -> usize {
        self.segments.len()
    }

    /// Returns true when there are no segments.
    pub fn is_empty(&self) -> bool {
        self.segments.is_empty()
    }

    /// Returns the first point of the curve string.
    pub fn start(&self) -> Option<&Point2> {
        self.segments.first().map(Segment2::start)
    }

    /// Returns the final point of the curve string.
    pub fn end(&self) -> Option<&Point2> {
        self.segments.last().map(Segment2::end)
    }

    /// Collects all nonempty segment-pair intersections against another curve string.
    ///
    /// Segment axis-aligned bounding boxes are used as a conservative broad
    /// phase before exact segment intersection. A decided box non-overlap skips
    /// the pair; any box uncertainty falls back to exact topology. This keeps
    /// the exact segment relation authoritative while following the
    /// candidate-pruning role used by sweep-line intersection methods such as
    /// Bentley and Ottmann, "Algorithms for Reporting and Counting Geometric
    /// Intersections" (1979).
    pub fn intersect_curve_string(
        &self,
        other: &Self,
        policy: &CurvePolicy,
    ) -> CurveResult<Vec<CurveStringIntersection>> {
        let self_boxes: Vec<_> = self
            .segments
            .iter()
            .map(|segment| decided_segment_aabb(segment, policy))
            .collect();
        let other_boxes: Vec<_> = other
            .segments
            .iter()
            .map(|segment| decided_segment_aabb(segment, policy))
            .collect();

        intersect_curve_strings_with_cached_aabbs(self, other, &self_boxes, &other_boxes, policy)
    }
}

pub(crate) fn intersect_curve_strings_with_cached_aabbs(
    first: &CurveString2,
    second: &CurveString2,
    first_segment_boxes: &[Option<Aabb2>],
    second_segment_boxes: &[Option<Aabb2>],
    policy: &CurvePolicy,
) -> CurveResult<Vec<CurveStringIntersection>> {
    let mut intersections = Vec::new();

    for (a_segment_index, a_segment) in first.segments.iter().enumerate() {
        for (b_segment_index, b_segment) in second.segments.iter().enumerate() {
            // This is the same conservative broad-phase used by the public
            // curve-string query. Bentley and Ottmann, "Algorithms for
            // Reporting and Counting Geometric Intersections" (1979), use
            // ordered sweep candidates for asymptotically better pruning; this
            // helper keeps the flat pair scan but lets prepared callers reuse
            // segment boxes across repeated queries.
            if let (Some(Some(a_box)), Some(Some(b_box))) = (
                first_segment_boxes.get(a_segment_index),
                second_segment_boxes.get(b_segment_index),
            ) && aabbs_decided_disjoint(a_box, b_box, policy)
            {
                continue;
            }

            let relation = a_segment.intersect_segment(b_segment, policy)?;
            if !relation.is_none() {
                intersections.push(CurveStringIntersection {
                    a_segment_index,
                    b_segment_index,
                    relation,
                });
            }
        }
    }

    Ok(intersections)
}
