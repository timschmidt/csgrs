//! Exact 3D triangle/triangle intersection classification.
//!
//! This module composes the existing exact plane/triangle, segment/triangle,
//! and coplanar-triangle predicates into one report-bearing 3D triangle pair
//! classifier. The structure follows the orientation-predicate decomposition
//! used by Guigue and Devillers, "Fast and Robust Triangle-Triangle Overlap
//! Test Using Orientation Predicates," *Journal of Graphics Tools* 8.1 (2003):
//! reject by supporting planes, handle fully coplanar pairs by exact projected
//! 2D predicates, and otherwise inspect the triangle edges against the opposite
//! triangle. Every accepted relation is replayable from retained predicate
//! facts rather than primitive tolerances, matching Yap, "Towards Exact
//! Geometric Computation," *Computational Geometry* 7.1-2 (1997).

use crate::classify::{
    PlaneTriangleRelation, SegmentTriangleIntersection, TriangleLocation,
    TriangleTriangleIntersection,
};
use crate::geometry::Point3;
use crate::geometry::plane::{PreparedOrientedPlane3, classify_plane_triangle_with_policy};
use crate::predicate::{Escalation, PredicateOutcome, PredicatePolicy, RefinementNeed};
use crate::predicates::coplanar::{
    CoplanarTriangleClassification, CoplanarTriangleRelation, TriangleDegeneracy,
    choose_coplanar_projection, classify_coplanar_triangle_points, classify_triangle3_degeneracy,
    project_point3, project_triangle3,
};
use crate::predicates::segment::classify_segment_intersection_with_policy;
use crate::predicates::triangle::{
    classify_point_triangle_with_policy, classify_segment_triangle3_intersection_with_policy,
};

/// Structural inconsistency in a retained triangle/triangle report.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum TriangleTriangleValidationError {
    /// A degenerate relation was reported for two nondegenerate triangles, or a
    /// nondegenerate relation was reported while either source was degenerate.
    DegeneracyMismatch,
    /// A coplanar relation did not retain a coplanar classifier.
    MissingCoplanarClassification,
    /// A non-coplanar relation retained coplanar classification data.
    UnexpectedCoplanarClassification,
    /// A coplanar relation does not agree with the retained projected report.
    CoplanarRelationMismatch,
    /// A non-coplanar relation retained too few edge/triangle reports.
    MissingEdgeReports,
    /// Retained edge and plane facts derive a different relation.
    RelationMismatch,
    /// Recomputing the classifier from supplied source triangles did not
    /// reproduce this retained report.
    SourceReplayMismatch,
}

/// Certified 3D triangle/triangle classification.
#[derive(Clone, Debug, PartialEq)]
pub struct TriangleTriangleClassification {
    /// Coarse triangle-pair relation.
    pub relation: TriangleTriangleIntersection,
    /// Degeneracy report for the left triangle.
    pub left_degeneracy: TriangleDegeneracy,
    /// Degeneracy report for the right triangle.
    pub right_degeneracy: TriangleDegeneracy,
    /// Right triangle classified against the left supporting plane.
    pub right_against_left_plane: Option<PlaneTriangleRelation>,
    /// Left triangle classified against the right supporting plane.
    pub left_against_right_plane: Option<PlaneTriangleRelation>,
    /// Left triangle edges classified against the right triangle.
    pub left_edges_against_right: [Option<SegmentTriangleIntersection>; 3],
    /// Right triangle edges classified against the left triangle.
    pub right_edges_against_left: [Option<SegmentTriangleIntersection>; 3],
    /// Exact projected classifier for fully coplanar pairs.
    pub coplanar: Option<CoplanarTriangleClassification>,
}

impl TriangleTriangleClassification {
    /// Validate retained facts without replaying source coordinates.
    ///
    /// This checks that the collapsed relation follows the retained
    /// degeneracy, plane-side, edge, and coplanar reports. It is intentionally
    /// a report consistency check, not a new geometric predicate. Yap's EGC
    /// model treats this as the handoff between exact predicates and
    /// construction/topology layers.
    pub fn validate(&self) -> Result<(), TriangleTriangleValidationError> {
        if self.left_degeneracy == TriangleDegeneracy::Degenerate
            || self.right_degeneracy == TriangleDegeneracy::Degenerate
        {
            return if self.relation == TriangleTriangleIntersection::Degenerate {
                Ok(())
            } else {
                Err(TriangleTriangleValidationError::DegeneracyMismatch)
            };
        }
        if self.relation == TriangleTriangleIntersection::Degenerate {
            return Err(TriangleTriangleValidationError::DegeneracyMismatch);
        }

        if let Some(coplanar) = &self.coplanar {
            let expected = relation_from_coplanar(coplanar.relation)
                .ok_or(TriangleTriangleValidationError::CoplanarRelationMismatch)?;
            if self.relation == expected {
                coplanar
                    .validate()
                    .map_err(|_| TriangleTriangleValidationError::CoplanarRelationMismatch)
            } else {
                Err(TriangleTriangleValidationError::CoplanarRelationMismatch)
            }
        } else if matches!(
            self.relation,
            TriangleTriangleIntersection::CoplanarDisjoint
                | TriangleTriangleIntersection::CoplanarTouching
                | TriangleTriangleIntersection::CoplanarOverlapping
        ) {
            Err(TriangleTriangleValidationError::MissingCoplanarClassification)
        } else {
            if self.edge_report_count() != 6
                && self.relation != TriangleTriangleIntersection::Disjoint
            {
                return Err(TriangleTriangleValidationError::MissingEdgeReports);
            }
            if derive_non_coplanar_relation(
                self.right_against_left_plane,
                self.left_against_right_plane,
                self.left_edges_against_right,
                self.right_edges_against_left,
            ) == self.relation
            {
                Ok(())
            } else {
                Err(TriangleTriangleValidationError::RelationMismatch)
            }
        }
    }

    /// Validate this report by recomputing it from source triangles.
    pub fn validate_against_triangles(
        &self,
        left: [&Point3; 3],
        right: [&Point3; 3],
        policy: PredicatePolicy,
    ) -> Result<(), TriangleTriangleValidationError> {
        self.validate()?;
        match classify_triangle_triangle3_points_with_policy(left, right, policy) {
            PredicateOutcome::Decided { value, .. } if &value == self => Ok(()),
            _ => Err(TriangleTriangleValidationError::SourceReplayMismatch),
        }
    }

    /// Count retained edge/triangle reports.
    pub fn edge_report_count(&self) -> usize {
        self.left_edges_against_right
            .iter()
            .chain(self.right_edges_against_left.iter())
            .filter(|entry| entry.is_some())
            .count()
    }
}

/// Classify two closed 3D triangles using the default predicate policy.
pub fn classify_triangle_triangle3(
    a0: &Point3,
    a1: &Point3,
    a2: &Point3,
    b0: &Point3,
    b1: &Point3,
    b2: &Point3,
) -> PredicateOutcome<TriangleTriangleClassification> {
    classify_triangle_triangle3_with_policy(a0, a1, a2, b0, b1, b2, PredicatePolicy::default())
}

/// Classify two closed 3D triangles using an explicit predicate policy.
pub fn classify_triangle_triangle3_with_policy(
    a0: &Point3,
    a1: &Point3,
    a2: &Point3,
    b0: &Point3,
    b1: &Point3,
    b2: &Point3,
    policy: PredicatePolicy,
) -> PredicateOutcome<TriangleTriangleClassification> {
    classify_triangle_triangle3_points_with_policy([a0, a1, a2], [b0, b1, b2], policy)
}

/// Classify two closed 3D triangles supplied as borrowed point triples.
pub fn classify_triangle_triangle3_points_with_policy(
    left: [&Point3; 3],
    right: [&Point3; 3],
    policy: PredicatePolicy,
) -> PredicateOutcome<TriangleTriangleClassification> {
    crate::trace_dispatch!("hyperlimit", "triangle_triangle3", "plane-edge-composition");

    let left_degeneracy = classify_triangle3_degeneracy(left[0], left[1], left[2]).degeneracy;
    let right_degeneracy = classify_triangle3_degeneracy(right[0], right[1], right[2]).degeneracy;
    if left_degeneracy == TriangleDegeneracy::Unknown
        || right_degeneracy == TriangleDegeneracy::Unknown
    {
        return PredicateOutcome::unknown(RefinementNeed::Unsupported, Escalation::Undecided);
    }
    if left_degeneracy == TriangleDegeneracy::Degenerate
        || right_degeneracy == TriangleDegeneracy::Degenerate
    {
        return decided(TriangleTriangleClassification {
            relation: TriangleTriangleIntersection::Degenerate,
            left_degeneracy,
            right_degeneracy,
            right_against_left_plane: None,
            left_against_right_plane: None,
            left_edges_against_right: [None; 3],
            right_edges_against_left: [None; 3],
            coplanar: None,
        });
    }

    let left_plane = PreparedOrientedPlane3::new(left[0], left[1], left[2]);
    let right_plane = PreparedOrientedPlane3::new(right[0], right[1], right[2]);
    let right_against_left_plane = match classify_plane_triangle_with_policy(
        left_plane.plane(),
        right[0],
        right[1],
        right[2],
        policy,
    ) {
        PredicateOutcome::Decided { value, .. } => value,
        PredicateOutcome::Unknown { needed, stage } => {
            return PredicateOutcome::unknown(needed, stage);
        }
    };
    let left_against_right_plane = match classify_plane_triangle_with_policy(
        right_plane.plane(),
        left[0],
        left[1],
        left[2],
        policy,
    ) {
        PredicateOutcome::Decided { value, .. } => value,
        PredicateOutcome::Unknown { needed, stage } => {
            return PredicateOutcome::unknown(needed, stage);
        }
    };

    if right_against_left_plane == PlaneTriangleRelation::Coplanar
        && left_against_right_plane == PlaneTriangleRelation::Coplanar
    {
        let coplanar = classify_coplanar_triangle_points(left, right);
        let Some(relation) = relation_from_coplanar(coplanar.relation) else {
            return PredicateOutcome::unknown(RefinementNeed::Unsupported, Escalation::Undecided);
        };
        return decided(TriangleTriangleClassification {
            relation,
            left_degeneracy,
            right_degeneracy,
            right_against_left_plane: Some(right_against_left_plane),
            left_against_right_plane: Some(left_against_right_plane),
            left_edges_against_right: [None; 3],
            right_edges_against_left: [None; 3],
            coplanar: Some(coplanar),
        });
    }

    if separated_by_plane(right_against_left_plane) || separated_by_plane(left_against_right_plane)
    {
        return decided(TriangleTriangleClassification {
            relation: TriangleTriangleIntersection::Disjoint,
            left_degeneracy,
            right_degeneracy,
            right_against_left_plane: Some(right_against_left_plane),
            left_against_right_plane: Some(left_against_right_plane),
            left_edges_against_right: [None; 3],
            right_edges_against_left: [None; 3],
            coplanar: None,
        });
    }

    let mut left_edges_against_right = [None; 3];
    let mut right_edges_against_left = [None; 3];
    let mut saw_boundary = false;
    let mut saw_crossing = false;

    for (slot, edge) in triangle_edges(left).into_iter().enumerate() {
        let relation = match edge_against_triangle(edge, right, policy) {
            Ok(relation) => relation,
            Err(unknown) => return unknown,
        };
        if let Err(unknown) = absorb_edge_relation(
            relation,
            edge,
            right,
            policy,
            &mut saw_boundary,
            &mut saw_crossing,
        ) {
            return unknown;
        }
        left_edges_against_right[slot] = Some(relation);
    }
    for (slot, edge) in triangle_edges(right).into_iter().enumerate() {
        let relation = match edge_against_triangle(edge, left, policy) {
            Ok(relation) => relation,
            Err(unknown) => return unknown,
        };
        if let Err(unknown) = absorb_edge_relation(
            relation,
            edge,
            left,
            policy,
            &mut saw_boundary,
            &mut saw_crossing,
        ) {
            return unknown;
        }
        right_edges_against_left[slot] = Some(relation);
    }

    let relation = if saw_crossing {
        TriangleTriangleIntersection::NonCoplanarIntersection
    } else if saw_boundary {
        TriangleTriangleIntersection::BoundaryTouch
    } else {
        TriangleTriangleIntersection::Disjoint
    };

    decided(TriangleTriangleClassification {
        relation,
        left_degeneracy,
        right_degeneracy,
        right_against_left_plane: Some(right_against_left_plane),
        left_against_right_plane: Some(left_against_right_plane),
        left_edges_against_right,
        right_edges_against_left,
        coplanar: None,
    })
}

fn edge_against_triangle(
    edge: [&Point3; 2],
    triangle: [&Point3; 3],
    policy: PredicatePolicy,
) -> Result<SegmentTriangleIntersection, PredicateOutcome<TriangleTriangleClassification>> {
    match classify_segment_triangle3_intersection_with_policy(
        edge[0],
        edge[1],
        triangle[0],
        triangle[1],
        triangle[2],
        policy,
    ) {
        PredicateOutcome::Decided { value, .. } => Ok(value),
        PredicateOutcome::Unknown { needed, stage } => {
            Err(PredicateOutcome::unknown(needed, stage))
        }
    }
}

fn absorb_edge_relation(
    relation: SegmentTriangleIntersection,
    edge: [&Point3; 2],
    triangle: [&Point3; 3],
    policy: PredicatePolicy,
    saw_boundary: &mut bool,
    saw_crossing: &mut bool,
) -> Result<(), PredicateOutcome<TriangleTriangleClassification>> {
    match relation {
        SegmentTriangleIntersection::Disjoint => {}
        SegmentTriangleIntersection::BoundaryTouch => *saw_boundary = true,
        SegmentTriangleIntersection::Proper => *saw_crossing = true,
        SegmentTriangleIntersection::Coplanar => {
            if coplanar_segment_intersects_triangle(edge, triangle, policy)? {
                *saw_boundary = true;
            }
        }
    }
    Ok(())
}

fn coplanar_segment_intersects_triangle(
    segment: [&Point3; 2],
    triangle: [&Point3; 3],
    policy: PredicatePolicy,
) -> Result<bool, PredicateOutcome<TriangleTriangleClassification>> {
    let Some((projection, _)) = choose_coplanar_projection(triangle) else {
        return Err(PredicateOutcome::unknown(
            RefinementNeed::Unsupported,
            Escalation::Undecided,
        ));
    };
    let segment2 = [
        project_point3(segment[0], projection),
        project_point3(segment[1], projection),
    ];
    let triangle2 = project_triangle3(triangle, projection);

    for endpoint in &segment2 {
        match classify_point_triangle_with_policy(
            &triangle2[0],
            &triangle2[1],
            &triangle2[2],
            endpoint,
            policy,
        ) {
            PredicateOutcome::Decided { value, .. } => {
                if matches!(
                    value,
                    TriangleLocation::Inside
                        | TriangleLocation::OnEdge
                        | TriangleLocation::OnVertex
                ) {
                    return Ok(true);
                }
            }
            PredicateOutcome::Unknown { needed, stage } => {
                return Err(PredicateOutcome::unknown(needed, stage));
            }
        }
    }

    for edge in triangle_edges2(&triangle2) {
        match classify_segment_intersection_with_policy(
            &segment2[0],
            &segment2[1],
            edge[0],
            edge[1],
            policy,
        ) {
            PredicateOutcome::Decided { value, .. } if value.intersects() => return Ok(true),
            PredicateOutcome::Decided { .. } => {}
            PredicateOutcome::Unknown { needed, stage } => {
                return Err(PredicateOutcome::unknown(needed, stage));
            }
        }
    }
    Ok(false)
}

fn derive_non_coplanar_relation(
    right_against_left_plane: Option<PlaneTriangleRelation>,
    left_against_right_plane: Option<PlaneTriangleRelation>,
    left_edges_against_right: [Option<SegmentTriangleIntersection>; 3],
    right_edges_against_left: [Option<SegmentTriangleIntersection>; 3],
) -> TriangleTriangleIntersection {
    if right_against_left_plane.is_some_and(separated_by_plane)
        || left_against_right_plane.is_some_and(separated_by_plane)
    {
        return TriangleTriangleIntersection::Disjoint;
    }
    let mut saw_boundary = false;
    let mut saw_crossing = false;
    for relation in left_edges_against_right
        .into_iter()
        .chain(right_edges_against_left)
        .flatten()
    {
        match relation {
            SegmentTriangleIntersection::Disjoint => {}
            SegmentTriangleIntersection::BoundaryTouch => saw_boundary = true,
            SegmentTriangleIntersection::Proper => saw_crossing = true,
            SegmentTriangleIntersection::Coplanar => saw_boundary = true,
        }
    }
    if saw_crossing {
        TriangleTriangleIntersection::NonCoplanarIntersection
    } else if saw_boundary {
        TriangleTriangleIntersection::BoundaryTouch
    } else {
        TriangleTriangleIntersection::Disjoint
    }
}

fn relation_from_coplanar(
    relation: CoplanarTriangleRelation,
) -> Option<TriangleTriangleIntersection> {
    match relation {
        CoplanarTriangleRelation::Disjoint => Some(TriangleTriangleIntersection::CoplanarDisjoint),
        CoplanarTriangleRelation::Touching => Some(TriangleTriangleIntersection::CoplanarTouching),
        CoplanarTriangleRelation::Overlapping => {
            Some(TriangleTriangleIntersection::CoplanarOverlapping)
        }
        CoplanarTriangleRelation::Unknown => None,
    }
}

fn separated_by_plane(relation: PlaneTriangleRelation) -> bool {
    matches!(
        relation,
        PlaneTriangleRelation::Below | PlaneTriangleRelation::Above
    )
}

fn triangle_edges(points: [&Point3; 3]) -> [[&Point3; 2]; 3] {
    [
        [points[0], points[1]],
        [points[1], points[2]],
        [points[2], points[0]],
    ]
}

fn triangle_edges2(points: &[crate::geometry::Point2; 3]) -> [[&crate::geometry::Point2; 2]; 3] {
    [
        [&points[0], &points[1]],
        [&points[1], &points[2]],
        [&points[2], &points[0]],
    ]
}

fn decided(
    value: TriangleTriangleClassification,
) -> PredicateOutcome<TriangleTriangleClassification> {
    PredicateOutcome::decided(value, crate::Certainty::Exact, Escalation::Exact)
}

#[cfg(test)]
mod tests {
    use super::*;
    use hyperreal::Real;

    fn p3(x: i32, y: i32, z: i32) -> Point3 {
        Point3::new(Real::from(x), Real::from(y), Real::from(z))
    }

    fn classify(left: [&Point3; 3], right: [&Point3; 3]) -> TriangleTriangleClassification {
        classify_triangle_triangle3_points_with_policy(left, right, PredicatePolicy::default())
            .value()
            .expect("integer triangle pair should decide")
    }

    #[test]
    fn triangle_triangle_rejects_plane_separated_pairs() {
        let a = [p3(0, 0, 0), p3(4, 0, 0), p3(0, 4, 0)];
        let b = [p3(0, 0, 2), p3(4, 0, 2), p3(0, 4, 2)];
        let report = classify([&a[0], &a[1], &a[2]], [&b[0], &b[1], &b[2]]);

        assert_eq!(report.relation, TriangleTriangleIntersection::Disjoint);
        assert_eq!(
            report.right_against_left_plane,
            Some(PlaneTriangleRelation::Below)
        );
        assert_eq!(report.edge_report_count(), 0);
        assert_eq!(report.validate(), Ok(()));
    }

    #[test]
    fn triangle_triangle_detects_noncoplanar_crossing() {
        let a = [p3(0, 0, 0), p3(4, 0, 0), p3(0, 4, 0)];
        let b = [p3(1, 1, -1), p3(1, 1, 1), p3(3, 1, 0)];
        let report = classify([&a[0], &a[1], &a[2]], [&b[0], &b[1], &b[2]]);

        assert_eq!(
            report.relation,
            TriangleTriangleIntersection::NonCoplanarIntersection
        );
        assert!(report.edge_report_count() >= 6);
        assert_eq!(report.validate(), Ok(()));
        assert_eq!(
            report.validate_against_triangles(
                [&a[0], &a[1], &a[2]],
                [&b[0], &b[1], &b[2]],
                PredicatePolicy::default()
            ),
            Ok(())
        );
    }

    #[test]
    fn triangle_triangle_detects_boundary_vertex_touch() {
        let a = [p3(0, 0, 0), p3(4, 0, 0), p3(0, 4, 0)];
        let b = [p3(4, 0, 0), p3(4, 0, 3), p3(4, 3, 0)];
        let report = classify([&a[0], &a[1], &a[2]], [&b[0], &b[1], &b[2]]);

        assert_eq!(report.relation, TriangleTriangleIntersection::BoundaryTouch);
        assert_eq!(report.validate(), Ok(()));
    }

    #[test]
    fn triangle_triangle_resolves_coplanar_positive_overlap() {
        let a = [p3(0, 0, 0), p3(4, 0, 0), p3(0, 4, 0)];
        let b = [p3(1, 1, 0), p3(5, 1, 0), p3(1, 5, 0)];
        let report = classify([&a[0], &a[1], &a[2]], [&b[0], &b[1], &b[2]]);

        assert_eq!(
            report.relation,
            TriangleTriangleIntersection::CoplanarOverlapping
        );
        assert!(report.coplanar.is_some());
        assert_eq!(report.validate(), Ok(()));
    }

    #[test]
    fn triangle_triangle_reports_degenerate_input() {
        let a = [p3(0, 0, 0), p3(1, 1, 1), p3(2, 2, 2)];
        let b = [p3(0, 0, 0), p3(4, 0, 0), p3(0, 4, 0)];
        let report = classify([&a[0], &a[1], &a[2]], [&b[0], &b[1], &b[2]]);

        assert_eq!(report.relation, TriangleTriangleIntersection::Degenerate);
        assert_eq!(report.validate(), Ok(()));
    }
}
