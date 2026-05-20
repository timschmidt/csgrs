//! Convex polygon and halfspace classifiers.
//!
//! These helpers compose existing exact line and plane predicates. They do not
//! own polygon rings, mesh faces, or polyhedron topology; higher crates remain
//! responsible for proving that supplied boundaries are ordered and convex.

use crate::classify::{ConvexPointLocation, PlaneSide};
use crate::geometry::{Plane3, Point2, Point3};
use crate::plane::classify_point_plane_with_policy;
use crate::predicate::{Certainty, Escalation, PredicateOutcome, PredicatePolicy, Sign};
use crate::predicates::orient::orient2d_with_policy;
use crate::predicates::ring::ring_area_sign_with_policy;

/// Classify a point relative to a closed ordered convex 2D polygon.
pub fn classify_point_convex_polygon2(
    vertices: &[Point2],
    point: &Point2,
) -> PredicateOutcome<ConvexPointLocation> {
    classify_point_convex_polygon2_with_policy(vertices, point, PredicatePolicy::default())
}

/// Classify a point relative to a closed ordered convex 2D polygon with an
/// explicit predicate policy.
///
/// The polygon must be supplied as a consistently ordered convex boundary. The
/// classifier first certifies the ring orientation, then composes exact edge
/// orientation signs. This is the classical halfspace characterization of
/// convex polygons; using exact orientation signs for every edge follows Yap,
/// "Towards Exact Geometric Computation," *Computational Geometry* 7.1-2
/// (1997). Invalid nonconvex ordering is not guessed here because topology
/// ownership belongs to `hypercurve`, `hypertri`, or mesh crates.
pub fn classify_point_convex_polygon2_with_policy(
    vertices: &[Point2],
    point: &Point2,
    policy: PredicatePolicy,
) -> PredicateOutcome<ConvexPointLocation> {
    if vertices.len() < 3 {
        return PredicateOutcome::decided(
            ConvexPointLocation::Degenerate,
            Certainty::Exact,
            Escalation::Structural,
        );
    }

    let orientation = match ring_area_sign_with_policy(vertices, policy) {
        PredicateOutcome::Decided {
            value,
            certainty,
            stage,
        } => {
            if value == Sign::Zero {
                return PredicateOutcome::decided(
                    ConvexPointLocation::Degenerate,
                    certainty,
                    stage,
                );
            }
            (value, certainty, stage)
        }
        PredicateOutcome::Unknown { needed, stage } => {
            return PredicateOutcome::unknown(needed, stage);
        }
    };

    let mut certainty = orientation.1;
    let mut stage = orientation.2;
    let mut boundary = false;
    for index in 0..vertices.len() {
        let start = &vertices[index];
        let end = &vertices[(index + 1) % vertices.len()];
        let edge = match orient2d_with_policy(start, end, point, policy) {
            PredicateOutcome::Decided {
                value,
                certainty: edge_certainty,
                stage: edge_stage,
            } => {
                certainty = max_certainty(certainty, edge_certainty);
                stage = max_stage(stage, edge_stage);
                value
            }
            PredicateOutcome::Unknown { needed, stage } => {
                return PredicateOutcome::unknown(needed, stage);
            }
        };
        if edge == Sign::Zero {
            boundary = true;
        } else if edge != orientation.0 {
            return PredicateOutcome::decided(ConvexPointLocation::Outside, certainty, stage);
        }
    }

    let location = if boundary {
        ConvexPointLocation::Boundary
    } else {
        ConvexPointLocation::Inside
    };
    PredicateOutcome::decided(location, certainty, stage)
}

/// Classify a point relative to a closed convex 3D polyhedron represented as
/// oriented bounding planes.
pub fn classify_point_convex_planes3(
    planes: &[Plane3],
    point: &Point3,
) -> PredicateOutcome<ConvexPointLocation> {
    classify_point_convex_planes3_with_policy(planes, point, PredicatePolicy::default())
}

/// Classify a point relative to a closed convex 3D polyhedron represented as
/// oriented bounding planes with an explicit predicate policy.
///
/// The inside convention is `PlaneSide::Below` or `PlaneSide::On` for every
/// plane. This keeps the predicate as an exact halfspace composer instead of a
/// topology owner: callers that store faces with the opposite normal must flip
/// their planes before calling. The composition follows Yap's object-layer
/// guidance by reusing exact point-plane predicates rather than converting
/// planes to primitive approximations.
pub fn classify_point_convex_planes3_with_policy(
    planes: &[Plane3],
    point: &Point3,
    policy: PredicatePolicy,
) -> PredicateOutcome<ConvexPointLocation> {
    if planes.is_empty() {
        return PredicateOutcome::decided(
            ConvexPointLocation::Degenerate,
            Certainty::Exact,
            Escalation::Structural,
        );
    }

    let mut certainty = Certainty::Exact;
    let mut stage = Escalation::Structural;
    let mut boundary = false;
    for plane in planes {
        match classify_point_plane_with_policy(point, plane, policy) {
            PredicateOutcome::Decided {
                value,
                certainty: plane_certainty,
                stage: plane_stage,
            } => {
                certainty = max_certainty(certainty, plane_certainty);
                stage = max_stage(stage, plane_stage);
                match value {
                    PlaneSide::Above => {
                        return PredicateOutcome::decided(
                            ConvexPointLocation::Outside,
                            certainty,
                            stage,
                        );
                    }
                    PlaneSide::On => boundary = true,
                    PlaneSide::Below => {}
                }
            }
            PredicateOutcome::Unknown { needed, stage } => {
                return PredicateOutcome::unknown(needed, stage);
            }
        }
    }

    let location = if boundary {
        ConvexPointLocation::Boundary
    } else {
        ConvexPointLocation::Inside
    };
    PredicateOutcome::decided(location, certainty, stage)
}

fn max_certainty(left: Certainty, right: Certainty) -> Certainty {
    match (left, right) {
        (Certainty::Filtered, _) | (_, Certainty::Filtered) => Certainty::Filtered,
        _ => Certainty::Exact,
    }
}

fn max_stage(left: Escalation, right: Escalation) -> Escalation {
    if stage_rank(left) >= stage_rank(right) {
        left
    } else {
        right
    }
}

fn stage_rank(stage: Escalation) -> u8 {
    match stage {
        Escalation::Structural => 0,
        Escalation::Filter => 1,
        Escalation::Exact => 2,
        Escalation::Refined => 3,
        Escalation::Undecided => 4,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use hyperreal::Real;

    fn p2(x: i32, y: i32) -> Point2 {
        Point2::new(Real::from(x), Real::from(y))
    }

    fn p3(x: i32, y: i32, z: i32) -> Point3 {
        Point3::new(Real::from(x), Real::from(y), Real::from(z))
    }

    #[test]
    fn convex_polygon2_classifier_composes_edge_halfspaces() {
        let square = vec![p2(0, 0), p2(4, 0), p2(4, 4), p2(0, 4)];

        assert_eq!(
            classify_point_convex_polygon2(&square, &p2(2, 2)).value(),
            Some(ConvexPointLocation::Inside)
        );
        assert_eq!(
            classify_point_convex_polygon2(&square, &p2(4, 2)).value(),
            Some(ConvexPointLocation::Boundary)
        );
        assert_eq!(
            classify_point_convex_polygon2(&square, &p2(5, 2)).value(),
            Some(ConvexPointLocation::Outside)
        );
    }

    #[test]
    fn convex_planes3_classifier_uses_below_as_inside() {
        let planes = vec![
            Plane3::new(p3(-1, 0, 0), Real::from(0)),
            Plane3::new(p3(1, 0, 0), Real::from(-4)),
            Plane3::new(p3(0, -1, 0), Real::from(0)),
            Plane3::new(p3(0, 1, 0), Real::from(-4)),
            Plane3::new(p3(0, 0, -1), Real::from(0)),
            Plane3::new(p3(0, 0, 1), Real::from(-4)),
        ];

        assert_eq!(
            classify_point_convex_planes3(&planes, &p3(2, 2, 2)).value(),
            Some(ConvexPointLocation::Inside)
        );
        assert_eq!(
            classify_point_convex_planes3(&planes, &p3(4, 2, 2)).value(),
            Some(ConvexPointLocation::Boundary)
        );
        assert_eq!(
            classify_point_convex_planes3(&planes, &p3(5, 2, 2)).value(),
            Some(ConvexPointLocation::Outside)
        );
    }
}
