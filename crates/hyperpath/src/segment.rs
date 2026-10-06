//! Exact path segment carriers.
//!
//! The first path primitive is a straight segment over `hyperlimit::Point2`.
//! It caches structural facts and exposes certified ordering along an
//! axis-aligned segment without inventing a local tolerance predicate. Segment
//! intersection and point incidence remain in `hyperlimit`, following Yap's
//! object-package recommendation for exact geometric computation.

use std::cmp::Ordering;

use hyperlimit::{
    Aabb2Facts, Point2, PredicateOutcome, PredicatePolicy, Segment2Facts, Sign, aabb2_facts,
    classify_real_sign, compare_reals, point2_equal, segment2_facts,
};
use hyperreal::{Problem, Real, RealExactSetFacts, RealSign, SymbolicDependencyMask};

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

/// Errors while constructing a line segment's ordered cached bounds.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum LinePathSegmentError {
    /// The selected predicate policy could not order an endpoint coordinate.
    PredicateUnresolved,
}

impl std::fmt::Display for LinePathSegmentError {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        formatter.write_str("line-segment endpoint ordering is unresolved")
    }
}

impl std::error::Error for LinePathSegmentError {}

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
    facts: LinePathSegmentFacts,
}

impl LinePathSegment {
    /// Construct a segment with an explicit policy for its ordered cached bounds.
    pub fn new(
        start: Point2,
        end: Point2,
        policy: PredicatePolicy,
    ) -> Result<Self, LinePathSegmentError> {
        let (bounds_min, bounds_max) = bounds_for_points(&start, &end, policy)?;
        let facts = line_segment_facts(&start, &end, &bounds_min, &bounds_max, policy);
        Ok(Self {
            start,
            end,
            bounds_min,
            bounds_max,
            facts,
        })
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

    /// Return the exact minimum corner of the segment bounds.
    pub const fn bounds_min(&self) -> &Point2 {
        &self.bounds_min
    }

    /// Return the exact maximum corner of the segment bounds.
    pub const fn bounds_max(&self) -> &Point2 {
        &self.bounds_max
    }

    /// Return squared segment length as an exact scalar expression.
    pub fn length_squared(&self) -> Real {
        let dx = self.end.x.clone() - self.start.x.clone();
        let dy = self.end.y.clone() - self.start.y.clone();
        Real::signed_product_sum([true, true], [[&dx, &dx], [&dy, &dy]])
    }

    /// Return exact Euclidean segment length as `sqrt(dx² + dy²)`.
    ///
    /// Unlike [`Self::axis_length`], this metric operation accepts any retained
    /// direction. It does not infer topology, incidence, or point ordering from
    /// the square root; those operations deliberately remain restricted to
    /// their separately certified structural predicates.
    pub fn euclidean_length(&self) -> Result<Real, Problem> {
        self.length_squared().sqrt()
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
            Some(Axis::X) => compare_reals(&first.x, &second.x, policy).value(),
            Some(Axis::Y) => compare_reals(&first.y, &second.y, policy).value(),
            None => return SegmentParameterOrder::Unknown,
        };
        let Some(ordering) = coordinate_order else {
            return SegmentParameterOrder::Unknown;
        };
        let forward = match self.facts.axis_aligned {
            Some(Axis::X) => compare_reals(&self.start.x, &self.end.x, policy).value(),
            Some(Axis::Y) => compare_reals(&self.start.y, &self.end.y, policy).value(),
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
        let same_direction = point2_equal(&self.start, &other.start, policy)
            .and(point2_equal(&self.end, &other.end, policy));
        let reverse_direction = point2_equal(&self.start, &other.end, policy).and(point2_equal(
            &self.end,
            &other.start,
            policy,
        ));
        same_direction.or(reverse_direction)
    }
}

fn line_segment_facts(
    start: &Point2,
    end: &Point2,
    bounds_min: &Point2,
    bounds_max: &Point2,
    policy: PredicatePolicy,
) -> LinePathSegmentFacts {
    let segment = segment2_facts(start, end);
    let coordinates = [&start.x, &start.y, &end.x, &end.y];
    let endpoint_exact = Real::exact_set_facts(coordinates);
    let symbolic_dependencies = coordinates
        .into_iter()
        .fold(SymbolicDependencyMask::NONE, |mask, value| {
            mask.union(value.detailed_facts().symbolic.dependencies)
        });
    let axis_aligned = if same_real(&start.y, &end.y, policy) == Some(true) {
        Some(Axis::X)
    } else if same_real(&start.x, &end.x, policy) == Some(true) {
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
        bounds: aabb2_facts(bounds_min, bounds_max),
    }
}

fn bounds_for_points(
    first: &Point2,
    second: &Point2,
    policy: PredicatePolicy,
) -> Result<(Point2, Point2), LinePathSegmentError> {
    let x_order = compare_reals(&first.x, &second.x, policy)
        .value()
        .ok_or(LinePathSegmentError::PredicateUnresolved)?;
    let y_order = compare_reals(&first.y, &second.y, policy)
        .value()
        .ok_or(LinePathSegmentError::PredicateUnresolved)?;
    let (min_x, max_x) = ordered_pair(&first.x, &second.x, x_order);
    let (min_y, max_y) = ordered_pair(&first.y, &second.y, y_order);
    Ok((Point2::new(min_x, min_y), Point2::new(max_x, max_y)))
}

fn ordered_pair(first: &Real, second: &Real, ordering: Ordering) -> (Real, Real) {
    match ordering {
        Ordering::Less | Ordering::Equal => (first.clone(), second.clone()),
        Ordering::Greater => (second.clone(), first.clone()),
    }
}

fn same_real(left: &Real, right: &Real, policy: PredicatePolicy) -> Option<bool> {
    compare_reals(left, right, policy)
        .value()
        .map(|ordering| ordering == Ordering::Equal)
}

fn absolute_difference(left: &Real, right: &Real, policy: PredicatePolicy) -> Option<Real> {
    match compare_reals(left, right, policy).value()? {
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

pub(crate) fn real_sign_with_policy(value: &Real, policy: PredicatePolicy) -> Option<RealSign> {
    classify_real_sign(value, policy)
        .value()
        .map(|sign| match sign {
            Sign::Negative => RealSign::Negative,
            Sign::Zero => RealSign::Zero,
            Sign::Positive => RealSign::Positive,
        })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn diagonal_segment_retains_exact_euclidean_length_without_axis_promotion() {
        let segment = LinePathSegment::new(
            Point2::new(Real::zero(), Real::zero()),
            Point2::new(Real::from(3), Real::from(4)),
            PredicatePolicy::STRICT,
        )
        .unwrap();

        assert_eq!(segment.facts().axis_aligned, None);
        assert_eq!(segment.axis_length(PredicatePolicy::STRICT), None);
        assert_eq!(segment.euclidean_length().unwrap(), Real::from(5));
    }
}
