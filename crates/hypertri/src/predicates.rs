//! Shared exact predicate helpers for triangulation topology.
//!
//! These wrappers keep algorithm modules from depending directly on predicate
//! provenance details. Exact code only consumes decided signs from the
//! crate-local kernel, while reusable segment topology is delegated to
//! `hyperlimit` so enum growth and exact overlap refinements stay centralized.

#![allow(dead_code)]

use crate::error::Result;
use crate::kernel::ExactKernel;
use crate::types::Point2;
use crate::types::{Sign, TriangleLocation};
pub use hyperlimit::SegmentIntersection;

/// Decide the orientation of three points.
pub(crate) fn orient2(kernel: &ExactKernel, a: &Point2, b: &Point2, c: &Point2) -> Result<Sign> {
    kernel.orient2(a, b, c)
}

/// Decide whether `point` lies inside or on the boundary of `abc`.
pub(crate) fn point_in_or_on_triangle(
    kernel: &ExactKernel,
    a: &Point2,
    b: &Point2,
    c: &Point2,
    point: &Point2,
) -> Result<bool> {
    Ok(matches!(
        kernel.classify_point_triangle(a, b, c, point)?,
        TriangleLocation::Inside | TriangleLocation::OnEdge | TriangleLocation::OnVertex
    ))
}

/// Decide whether a point lies on a closed segment.
pub(crate) fn point_on_segment(
    kernel: &ExactKernel,
    a: &Point2,
    b: &Point2,
    point: &Point2,
) -> Result<bool> {
    // This is a direct boundary predicate, not triangulation topology. Route it
    // through hyperlimit's segment classifier so the exact interval and
    // degenerate-segment rules have a single owner.
    kernel.decide(
        hyperlimit::point_on_segment(a, b, point, kernel.policy()),
        "point_on_segment",
    )
}

/// Decide whether two points have equal exact coordinates.
#[inline]
pub(crate) fn points_equal(kernel: &ExactKernel, left: &Point2, right: &Point2) -> Result<bool> {
    if let (Some(left_x), Some(right_x)) =
        (left.x.exact_rational_ref(), right.x.exact_rational_ref())
    {
        if left_x != right_x {
            return Ok(false);
        }
        if let (Some(left_y), Some(right_y)) =
            (left.y.exact_rational_ref(), right.y.exact_rational_ref())
        {
            return Ok(left_y == right_y);
        }
    }
    if left == right {
        return Ok(true);
    }

    kernel.decide(
        hyperlimit::point2_equal(left, right, kernel.policy()),
        "point2_equal",
    )
}

/// Decide whether a point is inside a closed indexed ring by even-odd parity.
pub(crate) fn point_in_ring_even_odd(
    kernel: &ExactKernel,
    vertices: &[Point2],
    ring: &[usize],
    point: &Point2,
) -> Result<bool> {
    // Hyperlimit owns the exact crossing-number and boundary predicates;
    // hypertri supplies only index topology.
    kernel.decide(
        hyperlimit::point_in_indexed_ring_even_odd(vertices, ring, point, kernel.policy()),
        "point_in_indexed_ring_even_odd",
    )
}

/// Classify two closed line segments.
pub(crate) fn segment_intersection(
    kernel: &ExactKernel,
    a: &Point2,
    b: &Point2,
    c: &Point2,
    d: &Point2,
) -> Result<SegmentIntersection> {
    // Segment intersection is the canonical topological predicate for CDT edge
    // recovery and ear visibility. Keep the four-orientation classifier in
    // hyperlimit, where determinant and interval decisions are implemented.
    // Hypertri consumes only the decided combinatorial relation.
    let outcome = hyperlimit::classify_segment_intersection(a, b, c, d, kernel.policy());
    kernel.decide(outcome, "segment_intersection")
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::context::TriangulationContext;
    use crate::types::Real;

    const APPROX: TriangulationContext =
        TriangulationContext::new(hyperlimit::PredicatePolicy::APPROXIMATE_512);

    fn kernel() -> ExactKernel {
        ExactKernel::new(&APPROX)
    }

    fn p(x: i64, y: i64) -> Point2 {
        Point2::new(Real::from(x), Real::from(y))
    }

    #[test]
    fn segment_intersection_delegates_identical_segments_to_hyperlimit() {
        assert_eq!(
            segment_intersection(&kernel(), &p(0, 0), &p(4, 0), &p(4, 0), &p(0, 0)).unwrap(),
            SegmentIntersection::Identical
        );
    }

    #[test]
    fn point_on_segment_delegates_degenerate_segments_to_hyperlimit() {
        assert!(point_on_segment(&kernel(), &p(2, 3), &p(2, 3), &p(2, 3)).unwrap());
        assert!(!point_on_segment(&kernel(), &p(2, 3), &p(2, 3), &p(2, 4)).unwrap());
    }

    #[test]
    fn point_equality_uses_numeric_not_representation_equality() {
        let left = Point2::new(Real::pi() + Real::e(), Real::zero());
        let right = Point2::new(Real::e() + Real::pi(), Real::zero());

        assert_ne!(left, right);
        assert_eq!(points_equal(&kernel(), &left, &right), Ok(true));
    }
}
