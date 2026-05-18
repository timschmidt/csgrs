//! Exact path segment carriers.
//!
//! The first path primitive is a straight segment over `hyperlimit::Point2`.
//! It caches structural facts and exposes certified ordering along an
//! axis-aligned segment without inventing a local tolerance predicate. Segment
//! intersection and point incidence remain in `hyperlimit`, following Yap's
//! object-package recommendation for exact geometric computation.

use std::cmp::Ordering;

use hyperlimit::{
    Aabb2Facts, Certainty, Escalation, Point2, PredicateOutcome, PredicatePolicy, PreparedAabb2,
    Segment2Facts, aabb2_facts, compare_reals_with_policy, point2_equal_with_policy,
    predicate::RefinementNeed, segment2_facts,
};
use hyperreal::{Real, RealExactSetFacts, RealSign, SymbolicDependencyMask};

use crate::provenance::PathProvenance;

/// Coordinate axis used by an axis-aligned path segment.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Axis {
    /// X varies while Y is constant.
    X,
    /// Y varies while X is constant.
    Y,
}

/// Certified ordering of a point parameter along a segment.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SegmentParameterOrder {
    /// The first point occurs before the second along the segment direction.
    Before,
    /// Both points have the same segment parameter.
    Equal,
    /// The first point occurs after the second along the segment direction.
    After,
    /// The ordering could not be certified under the requested predicate policy.
    Unknown,
}

/// Cached structural facts for one line path segment.
#[derive(Clone, Debug, PartialEq)]
pub struct LinePathSegmentFacts {
    /// Structural facts from `hyperlimit` for the closed segment.
    pub segment: Segment2Facts,
    /// Exact-set facts across all four endpoint coordinates.
    pub endpoint_exact: RealExactSetFacts,
    /// Symbolic families present in endpoint coordinates.
    pub symbolic_dependencies: SymbolicDependencyMask,
    /// Certified axis alignment when available.
    pub axis_aligned: Option<Axis>,
    /// Whether the endpoints are structurally known to be the same point.
    pub known_degenerate: Option<bool>,
    /// Structural facts for the segment bounds.
    pub bounds: Aabb2Facts,
}

/// Exact straight path segment.
#[derive(Clone, Debug, PartialEq)]
pub struct LinePathSegment {
    start: Point2,
    end: Point2,
    bounds_min: Point2,
    bounds_max: Point2,
    provenance: PathProvenance,
    facts: LinePathSegmentFacts,
}

impl LinePathSegment {
    /// Construct a segment and cache endpoint facts.
    pub fn new(start: Point2, end: Point2) -> Self {
        Self::with_provenance(start, end, PathProvenance::native())
    }

    /// Construct a segment with source provenance.
    pub fn with_provenance(start: Point2, end: Point2, provenance: PathProvenance) -> Self {
        let (bounds_min, bounds_max) = bounds_for_points(&start, &end);
        let facts = line_segment_facts(&start, &end);
        Self {
            start,
            end,
            bounds_min,
            bounds_max,
            provenance,
            facts,
        }
    }

    /// Return the start point.
    pub const fn start(&self) -> &Point2 {
        &self.start
    }

    /// Return the end point.
    pub const fn end(&self) -> &Point2 {
        &self.end
    }

    /// Return cached structural facts.
    pub const fn facts(&self) -> &LinePathSegmentFacts {
        &self.facts
    }

    /// Return path source provenance.
    pub const fn provenance(&self) -> PathProvenance {
        self.provenance
    }

    /// Return the exact minimum corner of the segment bounds.
    pub const fn bounds_min(&self) -> &Point2 {
        &self.bounds_min
    }

    /// Return the exact maximum corner of the segment bounds.
    pub const fn bounds_max(&self) -> &Point2 {
        &self.bounds_max
    }

    /// Prepare the retained exact segment bounds for repeated broad-phase use.
    pub fn prepared_bounds(&self) -> PreparedAabb2<'_> {
        PreparedAabb2::from_facts(&self.bounds_min, &self.bounds_max, self.facts.bounds)
    }

    /// Return squared segment length as an exact scalar expression.
    pub fn length_squared(&self) -> Real {
        let dx = self.end.x.clone() - self.start.x.clone();
        let dy = self.end.y.clone() - self.start.y.clone();
        Real::signed_product_sum([true, true], [[&dx, &dx], [&dy, &dy]])
    }

    /// Return the exact tangent vector at the segment start.
    ///
    /// Straight-segment tangents are the retained endpoint displacement
    /// `(end - start)`. The vector is not unit-normalized; this keeps G1
    /// continuity predicates in Yap's exact object layer and avoids square-root
    /// normalization before `hyperlimit`/`hyperpath::tangent` classify joins.
    pub fn start_tangent(&self) -> Point2 {
        self.direction_vector()
    }

    /// Return the exact tangent vector at the segment end.
    pub fn end_tangent(&self) -> Point2 {
        self.direction_vector()
    }

    /// Return the exact directed displacement from start to end.
    pub fn direction_vector(&self) -> Point2 {
        Point2::new(
            self.end.x.clone() - self.start.x.clone(),
            self.end.y.clone() - self.start.y.clone(),
        )
    }

    /// Return exact axis length when the segment is certified axis-aligned.
    pub fn axis_length(&self, policy: PredicatePolicy) -> Option<Real> {
        match self.facts.axis_aligned? {
            Axis::X => absolute_difference(&self.start.x, &self.end.x, policy),
            Axis::Y => absolute_difference(&self.start.y, &self.end.y, policy),
        }
    }

    /// Compare two points by their parameter along this segment.
    ///
    /// This is intentionally limited to certified axis-aligned segments. The
    /// general case should use an exact projection or a retained construction
    /// parameter rather than deriving topology from lossy coordinates.
    pub fn compare_points_along(
        &self,
        first: &Point2,
        second: &Point2,
        policy: PredicatePolicy,
    ) -> SegmentParameterOrder {
        let coordinate_order = match self.facts.axis_aligned {
            Some(Axis::X) => compare_reals_with_policy(&first.x, &second.x, policy).value(),
            Some(Axis::Y) => compare_reals_with_policy(&first.y, &second.y, policy).value(),
            None => return SegmentParameterOrder::Unknown,
        };
        let Some(ordering) = coordinate_order else {
            return SegmentParameterOrder::Unknown;
        };
        let forward = match self.facts.axis_aligned {
            Some(Axis::X) => compare_reals_with_policy(&self.start.x, &self.end.x, policy).value(),
            Some(Axis::Y) => compare_reals_with_policy(&self.start.y, &self.end.y, policy).value(),
            None => None,
        };
        match (ordering, forward) {
            (Ordering::Equal, _) => SegmentParameterOrder::Equal,
            (_, Some(Ordering::Less)) => order_to_parameter(ordering),
            (_, Some(Ordering::Greater)) => order_to_parameter(ordering.reverse()),
            (_, Some(Ordering::Equal) | None) => SegmentParameterOrder::Unknown,
        }
    }

    /// Return whether this segment has the same endpoints as another segment.
    pub fn exact_endpoint_equal(
        &self,
        other: &Self,
        policy: PredicatePolicy,
    ) -> PredicateOutcome<bool> {
        let same_direction = point2_equal_with_policy(&self.start, &other.start, policy)
            .value()
            .zip(point2_equal_with_policy(&self.end, &other.end, policy).value())
            .map(|(a, b)| a && b);
        let reverse_direction = point2_equal_with_policy(&self.start, &other.end, policy)
            .value()
            .zip(point2_equal_with_policy(&self.end, &other.start, policy).value())
            .map(|(a, b)| a && b);
        match (same_direction, reverse_direction) {
            (Some(true), _) | (_, Some(true)) => {
                PredicateOutcome::decided(true, Certainty::Exact, Escalation::Exact)
            }
            (Some(false), Some(false)) => {
                PredicateOutcome::decided(false, Certainty::Exact, Escalation::Exact)
            }
            _ => PredicateOutcome::unknown(RefinementNeed::RealRefinement, Escalation::Undecided),
        }
    }
}

fn line_segment_facts(start: &Point2, end: &Point2) -> LinePathSegmentFacts {
    let segment = segment2_facts(start, end);
    let (bounds_min, bounds_max) = bounds_for_points(start, end);
    let coordinates = [&start.x, &start.y, &end.x, &end.y];
    let endpoint_exact = Real::exact_set_facts(coordinates);
    let symbolic_dependencies = coordinates
        .into_iter()
        .fold(SymbolicDependencyMask::NONE, |mask, value| {
            mask.union(value.detailed_facts().symbolic.dependencies)
        });
    let axis_aligned = if same_real(&start.y, &end.y) == Some(true) {
        Some(Axis::X)
    } else if same_real(&start.x, &end.x) == Some(true) {
        Some(Axis::Y)
    } else {
        None
    };
    LinePathSegmentFacts {
        segment,
        endpoint_exact,
        symbolic_dependencies,
        axis_aligned,
        known_degenerate: segment.known_degenerate(),
        bounds: aabb2_facts(&bounds_min, &bounds_max),
    }
}

fn bounds_for_points(first: &Point2, second: &Point2) -> (Point2, Point2) {
    let min_x = min_real(&first.x, &second.x).unwrap_or_else(|| first.x.clone());
    let max_x = max_real(&first.x, &second.x).unwrap_or_else(|| first.x.clone());
    let min_y = min_real(&first.y, &second.y).unwrap_or_else(|| first.y.clone());
    let max_y = max_real(&first.y, &second.y).unwrap_or_else(|| first.y.clone());
    (Point2::new(min_x, min_y), Point2::new(max_x, max_y))
}

fn min_real(first: &Real, second: &Real) -> Option<Real> {
    match compare_reals_with_policy(first, second, PredicatePolicy::default()).value()? {
        Ordering::Less | Ordering::Equal => Some(first.clone()),
        Ordering::Greater => Some(second.clone()),
    }
}

fn max_real(first: &Real, second: &Real) -> Option<Real> {
    match compare_reals_with_policy(first, second, PredicatePolicy::default()).value()? {
        Ordering::Less | Ordering::Equal => Some(second.clone()),
        Ordering::Greater => Some(first.clone()),
    }
}

fn same_real(left: &Real, right: &Real) -> Option<bool> {
    compare_reals_with_policy(left, right, PredicatePolicy::default())
        .value()
        .map(|ordering| ordering == Ordering::Equal)
}

fn absolute_difference(left: &Real, right: &Real, policy: PredicatePolicy) -> Option<Real> {
    match compare_reals_with_policy(left, right, policy).value()? {
        Ordering::Less | Ordering::Equal => Some(right.clone() - left.clone()),
        Ordering::Greater => Some(left.clone() - right.clone()),
    }
}

fn order_to_parameter(ordering: Ordering) -> SegmentParameterOrder {
    match ordering {
        Ordering::Less => SegmentParameterOrder::Before,
        Ordering::Equal => SegmentParameterOrder::Equal,
        Ordering::Greater => SegmentParameterOrder::After,
    }
}

pub(crate) fn real_sign(value: &Real) -> Option<RealSign> {
    value.structural_facts().sign
}
