//! Shared exact predicate helpers for triangulation topology.
//!
//! These wrappers keep algorithm modules from depending directly on predicate
//! provenance details. Exact code only consumes decided signs from the
//! crate-local kernel, while reusable segment topology is delegated to
//! `hyperlimit` so enum growth and exact overlap refinements stay centralized.

#![allow(dead_code)]

use crate::error::{Error, Result};
use crate::kernel::Kernel;
use crate::types::Point2;
use crate::types::{Sign, TriangleLocation};
use hyperlimit::{PredicateOutcome, PredicatePolicy};

pub use hyperlimit::SegmentIntersection;

/// Decide the orientation of three points.
pub(crate) fn orient2d<K>(a: &Point2, b: &Point2, c: &Point2) -> Result<Sign>
where
    K: Kernel,
{
    K::orient2d(a, b, c)
}

/// Decide whether `point` lies inside or on the boundary of `abc`.
pub(crate) fn point_in_or_on_triangle<K>(
    a: &Point2,
    b: &Point2,
    c: &Point2,
    point: &Point2,
) -> Result<bool>
where
    K: Kernel,
{
    Ok(matches!(
        K::classify_point_triangle(a, b, c, point)?,
        TriangleLocation::Inside | TriangleLocation::OnEdge | TriangleLocation::OnVertex
    ))
}

/// Decide whether a point lies on a closed segment.
pub(crate) fn point_on_segment(a: &Point2, b: &Point2, point: &Point2) -> Result<bool> {
    // This is a direct boundary predicate, not triangulation topology. Route it
    // through hyperlimit's segment classifier so the exact interval and
    // degenerate-segment rules have a single owner, matching Yap's
    // object/predicate separation; see Yap, "Towards Exact Geometric
    // Computation," Computational Geometry 7.1-2 (1997).
    decide_hyperlimit_bool(
        hyperlimit::point_on_segment_with_policy(
            &predicate_point(a),
            &predicate_point(b),
            &predicate_point(point),
            triangulation_predicate_policy(),
        ),
        "point_on_segment",
    )
}

/// Decide whether a point is inside a closed indexed ring by even-odd parity.
pub(crate) fn point_in_ring_even_odd(
    vertices: &[Point2],
    ring: &[usize],
    point: &Point2,
) -> Result<bool> {
    // Hyperlimit owns the Hormann-Agathos crossing-number predicate and the
    // Yap-style exact boundary checks. Hypertri supplies only index topology.
    let predicate_vertices: Vec<_> = vertices.iter().map(predicate_point).collect();
    decide_hyperlimit_bool(
        hyperlimit::point_in_indexed_ring_even_odd_with_policy(
            &predicate_vertices,
            ring,
            &predicate_point(point),
            triangulation_predicate_policy(),
        ),
        "point_in_indexed_ring_even_odd",
    )
}

/// Classify two closed line segments.
pub(crate) fn segment_intersection(
    a: &Point2,
    b: &Point2,
    c: &Point2,
    d: &Point2,
) -> Result<SegmentIntersection> {
    // Segment intersection is the canonical topological predicate for CDT edge
    // recovery and ear visibility. Keep the four-orientation classifier in
    // hyperlimit, where the de Berg et al. classifier and Shewchuk/Yap exact
    // predicate discipline are documented near the determinant and interval
    // decisions. Hypertri consumes the decided combinatorial relation only.
    match hyperlimit::classify_segment_intersection_with_policy(
        &predicate_point(a),
        &predicate_point(b),
        &predicate_point(c),
        &predicate_point(d),
        triangulation_predicate_policy(),
    ) {
        PredicateOutcome::Decided { value, .. } => Ok(value),
        PredicateOutcome::Unknown { .. } => Err(Error::PredicateUndecided {
            predicate: "segment_intersection",
        }),
    }
}

fn predicate_point(point: &Point2) -> hyperlimit::Point2 {
    hyperlimit::Point2::new(point.x.clone(), point.y.clone())
}

const fn triangulation_predicate_policy() -> PredicatePolicy {
    PredicatePolicy {
        allow_exact: true,
        allow_refinement: true,
        max_refinement_precision: Some(-4096),
    }
}

fn decide_hyperlimit_bool(
    outcome: PredicateOutcome<bool>,
    predicate: &'static str,
) -> Result<bool> {
    match outcome {
        PredicateOutcome::Decided { value, .. } => Ok(value),
        PredicateOutcome::Unknown { .. } => Err(Error::PredicateUndecided { predicate }),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::Real;

    fn p(x: i64, y: i64) -> Point2 {
        Point2::new(Real::from(x), Real::from(y))
    }

    #[test]
    fn segment_intersection_delegates_identical_segments_to_hyperlimit() {
        assert_eq!(
            segment_intersection(&p(0, 0), &p(4, 0), &p(4, 0), &p(0, 0)).unwrap(),
            SegmentIntersection::Identical
        );
    }

    #[test]
    fn point_on_segment_delegates_degenerate_segments_to_hyperlimit() {
        assert!(point_on_segment(&p(2, 3), &p(2, 3), &p(2, 3)).unwrap());
        assert!(!point_on_segment(&p(2, 3), &p(2, 3), &p(2, 4)).unwrap());
    }
}
