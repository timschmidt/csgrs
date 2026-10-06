//! Exact 3D triangle/triangle intersection classification.
//!
//! This module composes the existing exact plane/triangle, segment/triangle,
//! and coplanar-triangle predicates into one report-bearing 3D triangle pair
//! classifier. The structure follows the orientation-predicate decomposition:
//! reject by supporting planes, handle fully coplanar pairs by exact projected
//! 2D predicates, and otherwise inspect the triangle edges against the opposite
//! triangle. Every accepted relation is replayable from retained predicate
//! facts rather than primitive tolerances.

use crate::classify::{
    PlaneSide, PlaneTriangleRelation, SegmentTriangleIntersection, TriangleLocation,
    TriangleTriangleIntersection,
};
use crate::geometry::Point3;
use crate::geometry::plane::{
    OrientedPlane3Evidence, classify_point_oriented_plane_with_evidence_and_policy,
    oriented_plane3_evidence,
};
use crate::predicate::PredicatePolicy;
use crate::predicate::{Escalation, PredicateOutcome, RefinementNeed};
use crate::predicates::coplanar::{
    CoplanarTriangleClassification, CoplanarTriangleRelation, TriangleDegeneracy,
    choose_coplanar_projection_with_policy, classify_coplanar_triangle_points_with_policy,
    classify_triangle3_degeneracy_with_policy, project_point3, project_triangle3,
};
use crate::predicates::segment::classify_segment_intersection_with_policy;
use crate::predicates::triangle::{
    classify_point_triangle_with_policy,
    classify_segment_triangle3_intersection_with_preclassified_sides,
};
use crate::resolve::resolve_composite_policy;

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
    /// a report consistency check, not a new geometric predicate. This is the
    /// handoff between exact predicates and
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
            if !matches!(
                self.relation,
                TriangleTriangleIntersection::CoplanarDisjoint
                    | TriangleTriangleIntersection::CoplanarTouching
                    | TriangleTriangleIntersection::CoplanarOverlapping
            ) {
                return Err(TriangleTriangleValidationError::UnexpectedCoplanarClassification);
            }
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
    resolve_composite_policy(policy, |policy| {
        classify_triangle_triangle3_points_impl(left, right, policy)
    })
}

fn classify_triangle_triangle3_points_impl(
    left: [&Point3; 3],
    right: [&Point3; 3],
    policy: PredicatePolicy,
) -> PredicateOutcome<TriangleTriangleClassification> {
    crate::trace_dispatch!("hyperlimit", "triangle_triangle3", "plane-edge-composition");

    let left_degeneracy =
        match classify_triangle3_degeneracy_with_policy(left[0], left[1], left[2], policy) {
            PredicateOutcome::Decided { value, .. } => value,
            PredicateOutcome::Unknown { needed, stage } => {
                return PredicateOutcome::unknown(needed, stage);
            }
        };
    let right_degeneracy =
        match classify_triangle3_degeneracy_with_policy(right[0], right[1], right[2], policy) {
            PredicateOutcome::Decided { value, .. } => value,
            PredicateOutcome::Unknown { needed, stage } => {
                return PredicateOutcome::unknown(needed, stage);
            }
        };
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

    let left_plane = oriented_plane3_evidence(left[0], left[1], left[2]);
    let right_plane = oriented_plane3_evidence(right[0], right[1], right[2]);
    let (right_against_left_plane, right_against_left_sides) =
        match classify_triangle_against_plane_evidence(&left_plane, right, policy) {
            Ok(classification) => classification,
            Err(unknown) => return unknown,
        };
    let (left_against_right_plane, left_against_right_sides) =
        match classify_triangle_against_plane_evidence(&right_plane, left, policy) {
            Ok(classification) => classification,
            Err(unknown) => return unknown,
        };

    if right_against_left_plane == PlaneTriangleRelation::Coplanar
        && left_against_right_plane == PlaneTriangleRelation::Coplanar
    {
        let coplanar = classify_coplanar_triangle_points_with_policy(left, right, policy);
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

    let mut saw_boundary = false;
    let mut saw_crossing = false;
    crate::trace_dispatch!(
        "hyperlimit",
        "triangle_triangle3",
        "reuse-plane-sides-for-edges"
    );

    let edge_reports = classify_triangle_pair_edges(
        left,
        right,
        left_against_right_sides,
        right_against_left_sides,
        left_plane.plane(),
        right_plane.plane(),
        policy,
        &mut saw_boundary,
        &mut saw_crossing,
    );
    let (left_edges_against_right, right_edges_against_left) = match edge_reports {
        Ok(reports) => reports,
        Err(unknown) => return unknown,
    };

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

type TriangleEdgeReports = [Option<SegmentTriangleIntersection>; 3];
type TrianglePairEdgeReports = (TriangleEdgeReports, TriangleEdgeReports);
type TrianglePairUnknown = PredicateOutcome<TriangleTriangleClassification>;

#[allow(clippy::too_many_arguments)]
fn classify_triangle_pair_edges(
    left: [&Point3; 3],
    right: [&Point3; 3],
    left_against_right_sides: [PlaneSide; 3],
    right_against_left_sides: [PlaneSide; 3],
    left_plane: &crate::geometry::Plane3,
    right_plane: &crate::geometry::Plane3,
    policy: PredicatePolicy,
    saw_boundary: &mut bool,
    saw_crossing: &mut bool,
) -> Result<TrianglePairEdgeReports, TrianglePairUnknown> {
    let mut reports = [[None; 3]; 2];
    for (direction, direction_reports) in reports.iter_mut().enumerate() {
        let (source, source_sides, target, target_plane) = if direction == 0 {
            (left, left_against_right_sides, right, right_plane)
        } else {
            (right, right_against_left_sides, left, left_plane)
        };
        *direction_reports = classify_triangle_edges_against_triangle(
            source,
            source_sides,
            target,
            target_plane,
            policy,
            saw_boundary,
            saw_crossing,
        )?;
    }
    let [left_reports, right_reports] = reports;
    Ok((left_reports, right_reports))
}

fn classify_triangle_edges_against_triangle(
    source: [&Point3; 3],
    source_sides: [PlaneSide; 3],
    target: [&Point3; 3],
    target_plane: &crate::geometry::Plane3,
    policy: PredicatePolicy,
    saw_boundary: &mut bool,
    saw_crossing: &mut bool,
) -> Result<TriangleEdgeReports, TrianglePairUnknown> {
    let mut reports = [None; 3];
    for (slot, (edge, endpoint_sides)) in triangle_edges(source)
        .into_iter()
        .zip(triangle_edge_sides(source_sides))
        .enumerate()
    {
        let relation = edge_against_triangle(edge, endpoint_sides, target, target_plane, policy)?;
        absorb_edge_relation(relation, edge, target, policy, saw_boundary, saw_crossing)?;
        reports[slot] = Some(relation);
    }
    Ok(reports)
}

fn edge_against_triangle(
    edge: [&Point3; 2],
    endpoint_sides: [PlaneSide; 2],
    triangle: [&Point3; 3],
    plane: &crate::geometry::Plane3,
    policy: PredicatePolicy,
) -> Result<SegmentTriangleIntersection, PredicateOutcome<TriangleTriangleClassification>> {
    match classify_segment_triangle3_intersection_with_preclassified_sides(
        edge,
        triangle,
        endpoint_sides,
        plane,
        policy,
    ) {
        PredicateOutcome::Decided { value, .. } => Ok(value),
        PredicateOutcome::Unknown { needed, stage } => {
            Err(PredicateOutcome::unknown(needed, stage))
        }
    }
}

fn classify_triangle_against_plane_evidence(
    plane: &OrientedPlane3Evidence,
    triangle: [&Point3; 3],
    policy: PredicatePolicy,
) -> Result<(PlaneTriangleRelation, [PlaneSide; 3]), PredicateOutcome<TriangleTriangleClassification>>
{
    let mut sides = [PlaneSide::On; 3];
    for (index, point) in triangle.into_iter().enumerate() {
        sides[index] =
            match classify_point_oriented_plane_with_evidence_and_policy(point, plane, policy) {
                PredicateOutcome::Decided { value, .. } => value,
                PredicateOutcome::Unknown { needed, stage } => {
                    return Err(PredicateOutcome::unknown(needed, stage));
                }
            };
    }

    let below = sides
        .iter()
        .filter(|&&side| side == PlaneSide::Below)
        .count();
    let above = sides
        .iter()
        .filter(|&&side| side == PlaneSide::Above)
        .count();
    let on = sides.iter().filter(|&&side| side == PlaneSide::On).count();
    let relation = if below == 3 {
        PlaneTriangleRelation::Below
    } else if above == 3 {
        PlaneTriangleRelation::Above
    } else if on == 3 {
        PlaneTriangleRelation::Coplanar
    } else if below > 0 && above > 0 {
        PlaneTriangleRelation::Split
    } else {
        PlaneTriangleRelation::BoundaryTouch
    };
    Ok((relation, sides))
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
    let Some(projection) = choose_coplanar_projection_with_policy(triangle, policy) else {
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
        let value = outcome_value(classify_point_triangle_with_policy(
            &triangle2[0],
            &triangle2[1],
            &triangle2[2],
            endpoint,
            policy,
        ))?;
        if matches!(
            value,
            TriangleLocation::Inside | TriangleLocation::OnEdge | TriangleLocation::OnVertex
        ) {
            return Ok(true);
        }
    }

    for edge in triangle_edges2(&triangle2) {
        let value = outcome_value(classify_segment_intersection_with_policy(
            &segment2[0],
            &segment2[1],
            edge[0],
            edge[1],
            policy,
        ))?;
        if value.intersects() {
            return Ok(true);
        }
    }
    Ok(false)
}

fn outcome_value<T, U>(outcome: PredicateOutcome<T>) -> Result<T, PredicateOutcome<U>> {
    match outcome {
        PredicateOutcome::Decided { value, .. } => Ok(value),
        PredicateOutcome::Unknown { needed, stage } => {
            Err(PredicateOutcome::unknown(needed, stage))
        }
    }
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

fn triangle_edge_sides(sides: [PlaneSide; 3]) -> [[PlaneSide; 2]; 3] {
    [
        [sides[0], sides[1]],
        [sides[1], sides[2]],
        [sides[2], sides[0]],
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

    const APPROX: PredicatePolicy = PredicatePolicy::APPROXIMATE_512;

    fn p3(x: i32, y: i32, z: i32) -> Point3 {
        Point3::new(Real::from(x), Real::from(y), Real::from(z))
    }

    fn terminal_zero() -> Real {
        let sine = Real::e().sin();
        let cosine = Real::e().cos();
        &sine * &sine + &cosine * &cosine - Real::one()
    }

    fn classify(left: [&Point3; 3], right: [&Point3; 3]) -> TriangleTriangleClassification {
        classify_triangle_triangle3_points_with_policy(left, right, APPROX)
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
            report.validate_against_triangles([&a[0], &a[1], &a[2]], [&b[0], &b[1], &b[2]], APPROX),
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

        let projected = project_triangle3(
            [&a[0], &a[1], &a[2]],
            crate::predicates::coplanar::CoplanarProjection::Xy,
        );
        let edges = triangle_edges2(&projected);
        assert_eq!(edges[0], [&projected[0], &projected[1]]);
        assert_eq!(edges[1], [&projected[1], &projected[2]]);
        assert_eq!(edges[2], [&projected[2], &projected[0]]);
    }

    #[test]
    fn triangle_triangle_reports_degenerate_input() {
        let a = [p3(0, 0, 0), p3(1, 1, 1), p3(2, 2, 2)];
        let b = [p3(0, 0, 0), p3(4, 0, 0), p3(0, 4, 0)];
        let report = classify([&a[0], &a[1], &a[2]], [&b[0], &b[1], &b[2]]);

        assert_eq!(report.relation, TriangleTriangleIntersection::Degenerate);
        assert_eq!(report.validate(), Ok(()));
    }

    #[test]
    fn triangle_triangle_covers_coplanar_disjoint_and_touching_relations() {
        let a = [p3(0, 0, 0), p3(4, 0, 0), p3(0, 4, 0)];
        let disjoint = [p3(10, 10, 0), p3(14, 10, 0), p3(10, 14, 0)];
        let touching = [p3(4, 0, 0), p3(7, 0, 0), p3(4, 3, 0)];

        let report = classify(
            [&a[0], &a[1], &a[2]],
            [&disjoint[0], &disjoint[1], &disjoint[2]],
        );
        assert_eq!(
            report.relation,
            TriangleTriangleIntersection::CoplanarDisjoint
        );
        assert_eq!(report.validate(), Ok(()));

        let report = classify(
            [&a[0], &a[1], &a[2]],
            [&touching[0], &touching[1], &touching[2]],
        );
        assert_eq!(
            report.relation,
            TriangleTriangleIntersection::CoplanarTouching
        );
        assert_eq!(report.validate(), Ok(()));
    }

    #[test]
    fn triangle_triangle_can_be_disjoint_without_plane_separation() {
        let horizontal = [p3(-2, 0, 0), p3(2, 0, 0), p3(0, 4, 0)];
        let vertical = [p3(0, 10, -2), p3(0, 10, 2), p3(0, 14, 0)];
        let report = classify(
            [&horizontal[0], &horizontal[1], &horizontal[2]],
            [&vertical[0], &vertical[1], &vertical[2]],
        );

        assert_eq!(report.relation, TriangleTriangleIntersection::Disjoint);
        assert_eq!(report.edge_report_count(), 6);
        assert!(
            !report
                .right_against_left_plane
                .is_some_and(separated_by_plane)
        );
        assert!(
            !report
                .left_against_right_plane
                .is_some_and(separated_by_plane)
        );
        assert_eq!(report.validate(), Ok(()));
    }

    #[test]
    fn coplanar_segment_helper_covers_endpoint_edge_and_disjoint_paths() {
        let triangle = [p3(0, 0, 0), p3(4, 0, 0), p3(0, 4, 0)];
        let inside = [p3(1, 1, 0), p3(6, 1, 0)];
        let crossing = [p3(-1, 1, 0), p3(5, 1, 0)];
        let outside = [p3(6, 6, 0), p3(8, 6, 0)];

        assert!(matches!(
            coplanar_segment_intersects_triangle(
                [&inside[0], &inside[1]],
                [&triangle[0], &triangle[1], &triangle[2]],
                APPROX,
            ),
            Ok(true)
        ));
        assert!(matches!(
            coplanar_segment_intersects_triangle(
                [&crossing[0], &crossing[1]],
                [&triangle[0], &triangle[1], &triangle[2]],
                APPROX,
            ),
            Ok(true)
        ));
        assert!(matches!(
            coplanar_segment_intersects_triangle(
                [&outside[0], &outside[1]],
                [&triangle[0], &triangle[1], &triangle[2]],
                APPROX,
            ),
            Ok(false)
        ));

        let degenerate = [p3(0, 0, 0), p3(1, 1, 1), p3(2, 2, 2)];
        assert!(matches!(
            coplanar_segment_intersects_triangle(
                [&inside[0], &inside[1]],
                [&degenerate[0], &degenerate[1], &degenerate[2]],
                APPROX,
            ),
            Err(PredicateOutcome::Unknown {
                needed: RefinementNeed::Unsupported,
                stage: Escalation::Undecided,
            })
        ));
    }

    #[test]
    fn plane_triangle_helper_classifies_every_coarse_side_relation() {
        let plane_points = [p3(0, 0, 0), p3(4, 0, 0), p3(0, 4, 0)];
        let plane = oriented_plane3_evidence(&plane_points[0], &plane_points[1], &plane_points[2]);
        let cases = [
            (
                [p3(0, 0, 2), p3(1, 0, 2), p3(0, 1, 2)],
                PlaneTriangleRelation::Below,
            ),
            (
                [p3(0, 0, -2), p3(1, 0, -2), p3(0, 1, -2)],
                PlaneTriangleRelation::Above,
            ),
            (
                [p3(0, 0, 0), p3(1, 0, 0), p3(0, 1, 0)],
                PlaneTriangleRelation::Coplanar,
            ),
            (
                [p3(0, 0, -1), p3(1, 0, 1), p3(0, 1, 0)],
                PlaneTriangleRelation::Split,
            ),
            (
                [p3(0, 0, 0), p3(1, 0, 1), p3(0, 1, 1)],
                PlaneTriangleRelation::BoundaryTouch,
            ),
        ];

        for (triangle, expected) in cases {
            let outcome = classify_triangle_against_plane_evidence(
                &plane,
                [&triangle[0], &triangle[1], &triangle[2]],
                APPROX,
            );
            assert!(matches!(outcome, Ok((relation, _)) if relation == expected));
        }

        let unresolved = [
            Point3::new(Real::from(0), Real::from(0), terminal_zero()),
            p3(1, 0, 1),
            p3(0, 1, 1),
        ];
        assert!(matches!(
            classify_triangle_against_plane_evidence(
                &plane,
                [&unresolved[0], &unresolved[1], &unresolved[2]],
                PredicatePolicy::STRICT,
            ),
            Err(PredicateOutcome::Unknown { .. })
        ));
    }

    #[test]
    fn retained_relation_helpers_cover_all_collapsed_outcomes() {
        assert_eq!(
            relation_from_coplanar(CoplanarTriangleRelation::Disjoint),
            Some(TriangleTriangleIntersection::CoplanarDisjoint)
        );
        assert_eq!(
            relation_from_coplanar(CoplanarTriangleRelation::Touching),
            Some(TriangleTriangleIntersection::CoplanarTouching)
        );
        assert_eq!(
            relation_from_coplanar(CoplanarTriangleRelation::Overlapping),
            Some(TriangleTriangleIntersection::CoplanarOverlapping)
        );
        assert_eq!(
            relation_from_coplanar(CoplanarTriangleRelation::Unknown),
            None
        );

        let none = [None; 3];
        assert_eq!(
            derive_non_coplanar_relation(Some(PlaneTriangleRelation::Above), None, none, none,),
            TriangleTriangleIntersection::Disjoint
        );
        assert_eq!(
            derive_non_coplanar_relation(
                Some(PlaneTriangleRelation::Split),
                Some(PlaneTriangleRelation::Split),
                [Some(SegmentTriangleIntersection::BoundaryTouch), None, None],
                none,
            ),
            TriangleTriangleIntersection::BoundaryTouch
        );
        assert_eq!(
            derive_non_coplanar_relation(
                None,
                None,
                [Some(SegmentTriangleIntersection::Proper), None, None],
                [Some(SegmentTriangleIntersection::Coplanar), None, None],
            ),
            TriangleTriangleIntersection::NonCoplanarIntersection
        );
        assert_eq!(
            derive_non_coplanar_relation(None, None, none, none),
            TriangleTriangleIntersection::Disjoint
        );
    }

    #[test]
    fn retained_report_rejects_false_degeneracy_and_unknown_coplanar_relation() {
        let a = [p3(0, 0, 0), p3(4, 0, 0), p3(0, 4, 0)];
        let b = [p3(0, 0, 2), p3(4, 0, 2), p3(0, 4, 2)];
        let mut report = classify([&a[0], &a[1], &a[2]], [&b[0], &b[1], &b[2]]);
        report.relation = TriangleTriangleIntersection::Degenerate;
        assert_eq!(
            report.validate(),
            Err(TriangleTriangleValidationError::DegeneracyMismatch)
        );

        let coplanar_b = [p3(1, 1, 0), p3(5, 1, 0), p3(1, 5, 0)];
        let mut report = classify(
            [&a[0], &a[1], &a[2]],
            [&coplanar_b[0], &coplanar_b[1], &coplanar_b[2]],
        );
        report
            .coplanar
            .as_mut()
            .expect("coplanar pair should retain its projected report")
            .relation = CoplanarTriangleRelation::Unknown;
        assert_eq!(
            report.validate(),
            Err(TriangleTriangleValidationError::CoplanarRelationMismatch)
        );
    }

    #[test]
    fn strict_composition_propagates_each_preconstruction_uncertainty_stage() {
        let planar = [p3(0, 0, 0), p3(4, 0, 0), p3(0, 4, 0)];
        let uncertain_triangle = [
            p3(0, 0, 0),
            p3(1, 0, 0),
            Point3::new(Real::from(0), terminal_zero(), Real::from(0)),
        ];
        assert!(matches!(
            classify_triangle_triangle3_points_impl(
                [
                    &uncertain_triangle[0],
                    &uncertain_triangle[1],
                    &uncertain_triangle[2]
                ],
                [&planar[0], &planar[1], &planar[2]],
                PredicatePolicy::STRICT,
            ),
            PredicateOutcome::Unknown { .. }
        ));
        assert!(matches!(
            classify_triangle_triangle3_points_impl(
                [&planar[0], &planar[1], &planar[2]],
                [
                    &uncertain_triangle[0],
                    &uncertain_triangle[1],
                    &uncertain_triangle[2]
                ],
                PredicatePolicy::STRICT,
            ),
            PredicateOutcome::Unknown { .. }
        ));

        let translated_z = terminal_zero();
        let parallel_unknown = [
            Point3::new(Real::from(0), Real::from(0), translated_z.clone()),
            Point3::new(Real::from(1), Real::from(0), translated_z.clone()),
            Point3::new(Real::from(0), Real::from(1), translated_z),
        ];
        assert_eq!(
            classify_triangle3_degeneracy_with_policy(
                &parallel_unknown[0],
                &parallel_unknown[1],
                &parallel_unknown[2],
                PredicatePolicy::STRICT,
            )
            .value(),
            Some(TriangleDegeneracy::NonDegenerate)
        );
        assert!(matches!(
            classify_triangle_triangle3_points_impl(
                [&planar[0], &planar[1], &planar[2]],
                [
                    &parallel_unknown[0],
                    &parallel_unknown[1],
                    &parallel_unknown[2]
                ],
                PredicatePolicy::STRICT,
            ),
            PredicateOutcome::Unknown { .. }
        ));

        let translated_x = terminal_zero();
        let vertical_unknown = [
            Point3::new(translated_x.clone(), Real::from(0), Real::from(-1)),
            Point3::new(translated_x.clone(), Real::from(1), Real::from(1)),
            Point3::new(translated_x, Real::from(-1), Real::from(1)),
        ];
        assert_eq!(
            classify_triangle3_degeneracy_with_policy(
                &vertical_unknown[0],
                &vertical_unknown[1],
                &vertical_unknown[2],
                PredicatePolicy::STRICT,
            )
            .value(),
            Some(TriangleDegeneracy::NonDegenerate)
        );
        assert!(matches!(
            classify_triangle_triangle3_points_impl(
                [&planar[0], &planar[1], &planar[2]],
                [
                    &vertical_unknown[0],
                    &vertical_unknown[1],
                    &vertical_unknown[2]
                ],
                PredicatePolicy::STRICT,
            ),
            PredicateOutcome::Unknown { .. }
        ));

        let plane = oriented_plane3_evidence(&planar[0], &planar[1], &planar[2]);
        assert!(matches!(
            edge_against_triangle(
                [&p3(0, 0, 1), &p3(1, 0, 1)],
                [PlaneSide::Below, PlaneSide::Above],
                [&planar[0], &planar[1], &planar[2]],
                plane.plane(),
                APPROX,
            ),
            Err(PredicateOutcome::Unknown { .. })
        ));

        let endpoint_unknown = Point3::new(Real::from(1), terminal_zero(), Real::from(0));
        assert!(matches!(
            coplanar_segment_intersects_triangle(
                [&endpoint_unknown, &p3(6, 1, 0)],
                [&planar[0], &planar[1], &planar[2]],
                PredicatePolicy::STRICT,
            ),
            Err(PredicateOutcome::Unknown { .. })
        ));

        let edge_unknown = terminal_zero();
        let outside_start = Point3::new(edge_unknown.clone(), Real::from(-1), Real::from(0));
        let outside_end = Point3::new(
            &edge_unknown + &Real::from(1),
            Real::from(-1),
            Real::from(0),
        );
        assert!(matches!(
            coplanar_segment_intersects_triangle(
                [&outside_start, &outside_end],
                [&planar[0], &planar[1], &planar[2]],
                PredicatePolicy::STRICT,
            ),
            Err(PredicateOutcome::Unknown { .. })
        ));

        let shared_shift = terminal_zero();
        let coplanar_unknown = [
            Point3::new(&Real::from(4) + &shared_shift, 0.into(), 0.into()),
            Point3::new(&Real::from(5) + &shared_shift, 0.into(), 0.into()),
            Point3::new(&Real::from(4) + &shared_shift, 1.into(), 0.into()),
        ];
        assert!(matches!(
            classify_triangle_triangle3_points_impl(
                [&planar[0], &planar[1], &planar[2]],
                [
                    &coplanar_unknown[0],
                    &coplanar_unknown[1],
                    &coplanar_unknown[2],
                ],
                PredicatePolicy::STRICT,
            ),
            PredicateOutcome::Unknown { .. }
        ));

        let vertex_shift = terminal_zero();
        let tangent_start = Point3::new(
            Real::from(-1),
            &Real::from(1) + &vertex_shift,
            Real::from(0),
        );
        let tangent_end = Point3::new(
            &Real::from(1) + &vertex_shift,
            Real::from(-1),
            Real::from(0),
        );
        assert!(matches!(
            coplanar_segment_intersects_triangle(
                [&tangent_start, &tangent_end],
                [&planar[0], &planar[1], &planar[2]],
                PredicatePolicy::STRICT,
            ),
            Err(PredicateOutcome::Unknown { .. })
        ));

        let plane = oriented_plane3_evidence(&planar[0], &planar[1], &planar[2]);
        let mut saw_boundary = false;
        let mut saw_crossing = false;
        assert!(matches!(
            classify_triangle_edges_against_triangle(
                [&endpoint_unknown, &p3(6, 1, 0), &p3(7, 1, 0)],
                [PlaneSide::On; 3],
                [&planar[0], &planar[1], &planar[2]],
                plane.plane(),
                PredicatePolicy::STRICT,
                &mut saw_boundary,
                &mut saw_crossing,
            ),
            Err(PredicateOutcome::Unknown { .. })
        ));

        let edge_shift = terminal_zero();
        let vertical_edge_unknown = [
            Point3::new(1.into(), edge_shift.clone(), (-1).into()),
            Point3::new(1.into(), edge_shift.clone(), 1.into()),
            Point3::new(1.into(), &Real::from(1) + &edge_shift, 0.into()),
        ];
        assert_eq!(
            classify_triangle3_degeneracy_with_policy(
                &vertical_edge_unknown[0],
                &vertical_edge_unknown[1],
                &vertical_edge_unknown[2],
                PredicatePolicy::STRICT,
            )
            .value(),
            Some(TriangleDegeneracy::NonDegenerate)
        );
        assert!(matches!(
            classify_triangle_triangle3_points_impl(
                [&planar[0], &planar[1], &planar[2]],
                [
                    &vertical_edge_unknown[0],
                    &vertical_edge_unknown[1],
                    &vertical_edge_unknown[2],
                ],
                PredicatePolicy::STRICT,
            ),
            PredicateOutcome::Unknown { .. }
        ));
    }
}
