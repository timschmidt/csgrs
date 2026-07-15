//! Coplanar triangle and projected 3D helper predicates.
//!
//! Coplanar triangle overlap is a 2D arrangement problem embedded in 3D. This
//! module projects triangles only onto a coordinate plane whose projected
//! orientation is certified nonzero by exact predicates; no primitive-float
//! normal magnitude or epsilon selects the projection. The overlap test is then
//! decomposed into exact 2D segment intersections and point-in-triangle
//! classifications. Representation choices may preserve structure, but
//! combinatorial claims require certified predicates.

use crate::classify::{SegmentIntersection, TriangleLocation};
use crate::geometry::{Point2, Point3};
use crate::predicate::{PredicateOutcome, PredicateUse, Sign};
use crate::predicates::orient::orient2d_report;
use crate::predicates::ring::ring_area_sign;
use crate::predicates::segment::classify_segment_intersection;
use crate::predicates::segment_plane::segment_parameter_from_axis;
use crate::predicates::triangle::classify_point_triangle;
use hyperreal::Real;

/// Coordinate projection used for exact coplanar overlap.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CoplanarProjection {
    /// Drop z and project to `(x, y)`.
    Xy,
    /// Drop y and project to `(x, z)`.
    Xz,
    /// Drop x and project to `(y, z)`.
    Yz,
}

/// Exact coplanar triangle overlap relation.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CoplanarTriangleRelation {
    /// The projected closed triangles are disjoint.
    Disjoint,
    /// The triangles touch only at vertices or edges.
    Touching,
    /// The triangles overlap with positive area, or share a positive-length
    /// collinear edge interval that requires graph construction.
    Overlapping,
    /// No certified nondegenerate projection or required predicate was decided.
    Unknown,
}

impl CoplanarTriangleRelation {
    /// Return whether this relation must be retained for graph construction.
    pub const fn needs_graph_construction(self) -> bool {
        !matches!(self, Self::Disjoint)
    }
}

/// Structural inconsistency in a projected coplanar triangle classifier.
///
/// This validates retained projection, segment relations, vertex-location
/// facts, and collapsed relation without recomputing predicates. This is the
/// required handoff from certified predicate facts to combinatorial topology.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CoplanarTriangleValidationError {
    /// A decided relation was produced without a certified projection.
    DecidedRelationWithoutProjection,
    /// An unknown relation with no projection retained downstream predicate
    /// facts.
    ProjectionlessUnknownHasFacts,
    /// A decided relation did not retain all nine edge-pair classifications.
    MissingEdgeIntersections,
    /// A decided relation did not retain all six vertex/triangle locations.
    MissingVertexLocations,
    /// Retained edge and vertex facts derive a different relation.
    RelationMismatch,
    /// Recomputing the classifier from supplied source triangles did not
    /// reproduce this retained report.
    SourceReplayMismatch,
}

/// Certified coplanar triangle overlap result.
#[derive(Clone, Debug, PartialEq)]
pub struct CoplanarTriangleClassification {
    /// Projection used for 2D predicates, or `None` when no projection was
    /// certified.
    pub projection: Option<CoplanarProjection>,
    /// Coarse overlap relation.
    pub relation: CoplanarTriangleRelation,
    /// Segment/segment relations for the nine projected edge pairs.
    pub edge_intersections: Vec<SegmentIntersection>,
    /// Locations of right-triangle vertices relative to the left triangle.
    pub right_vertices_in_left: [Option<TriangleLocation>; 3],
    /// Locations of left-triangle vertices relative to the right triangle.
    pub left_vertices_in_right: [Option<TriangleLocation>; 3],
    /// Predicate certificates retained while choosing the projection.
    pub predicates: Vec<PredicateUse>,
}

impl CoplanarTriangleClassification {
    /// Return whether the projection predicates produced exact-preserving
    /// proofs.
    pub fn projection_proof_producing(&self) -> bool {
        self.predicates
            .iter()
            .copied()
            .all(PredicateUse::is_proof_producing)
    }

    /// Validate projection, retained predicate facts, and relation coherence.
    ///
    /// Unknown results may retain a certified projection and a prefix of edge
    /// facts because the classifier exits as soon as one required projected
    /// predicate becomes undecided. Decided results must retain the complete
    /// edge and vertex facts needed to justify the collapsed relation.
    pub fn validate(&self) -> Result<(), CoplanarTriangleValidationError> {
        if self.projection.is_none() {
            return if self.relation == CoplanarTriangleRelation::Unknown
                && self.edge_intersections.is_empty()
                && self.right_vertices_in_left == [None, None, None]
                && self.left_vertices_in_right == [None, None, None]
            {
                Ok(())
            } else if self.relation == CoplanarTriangleRelation::Unknown {
                Err(CoplanarTriangleValidationError::ProjectionlessUnknownHasFacts)
            } else {
                Err(CoplanarTriangleValidationError::DecidedRelationWithoutProjection)
            };
        }

        if self.relation == CoplanarTriangleRelation::Unknown {
            return Ok(());
        }
        if self.edge_intersections.len() != 9 {
            return Err(CoplanarTriangleValidationError::MissingEdgeIntersections);
        }
        if self.right_vertices_in_left.iter().any(Option::is_none)
            || self.left_vertices_in_right.iter().any(Option::is_none)
        {
            return Err(CoplanarTriangleValidationError::MissingVertexLocations);
        }
        if derive_coplanar_triangle_relation(
            &self.edge_intersections,
            self.right_vertices_in_left,
            self.left_vertices_in_right,
        ) == self.relation
        {
            Ok(())
        } else {
            Err(CoplanarTriangleValidationError::RelationMismatch)
        }
    }

    /// Validate this report against indexed source triangles.
    ///
    /// Source replay recomputes projection selection, projected segment
    /// predicates, and point-in-triangle predicates from `points`, `left`, and
    /// `right`, then requires exact equality with the retained classifier. This
    /// is the source-aware exact-computation handoff.
    pub fn validate_against_sources(
        &self,
        points: &[Point3],
        left: [usize; 3],
        right: [usize; 3],
    ) -> Result<(), CoplanarTriangleValidationError> {
        self.validate()?;
        if !indices_in_range(points, left) || !indices_in_range(points, right) {
            return Err(CoplanarTriangleValidationError::SourceReplayMismatch);
        }
        let replay = classify_coplanar_triangles(points, left, right);
        if self == &replay {
            Ok(())
        } else {
            Err(CoplanarTriangleValidationError::SourceReplayMismatch)
        }
    }
}

/// Exact degeneracy state for a 3D triangle.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum TriangleDegeneracy {
    /// At least one coordinate projection has non-zero certified orientation.
    NonDegenerate,
    /// All coordinate projections are exactly collinear.
    Degenerate,
    /// A needed predicate could not be decided by the enabled exact route.
    Unknown,
}

/// Predicate reports retained while classifying one triangle.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct TrianglePredicateReport {
    /// Degeneracy result.
    pub degeneracy: TriangleDegeneracy,
    /// Predicate certificates used by the classification.
    pub predicates: Vec<PredicateUse>,
}

impl TrianglePredicateReport {
    /// Return whether all retained predicate routes were proof-producing.
    pub fn all_proof_producing(&self) -> bool {
        self.predicates
            .iter()
            .copied()
            .all(PredicateUse::is_proof_producing)
    }
}

/// Classify whether three exact 3D points form a non-degenerate triangle.
///
/// Degeneracy is tested by exact 2D orientation in coordinate projections. If
/// every projection has zero orientation, the three 3D points are collinear.
/// This uses exact determinant predicates in every coordinate projection.
pub fn classify_triangle3_degeneracy(
    a: &Point3,
    b: &Point3,
    c: &Point3,
) -> TrianglePredicateReport {
    let reports = [
        orient2d_report(
            &project_point3(a, CoplanarProjection::Xy),
            &project_point3(b, CoplanarProjection::Xy),
            &project_point3(c, CoplanarProjection::Xy),
        ),
        orient2d_report(
            &project_point3(a, CoplanarProjection::Xz),
            &project_point3(b, CoplanarProjection::Xz),
            &project_point3(c, CoplanarProjection::Xz),
        ),
        orient2d_report(
            &project_point3(a, CoplanarProjection::Yz),
            &project_point3(b, CoplanarProjection::Yz),
            &project_point3(c, CoplanarProjection::Yz),
        ),
    ];

    let mut predicates = Vec::with_capacity(reports.len());
    let mut all_zero = true;

    for report in reports {
        predicates.push(PredicateUse::from_certificate(report.certificate));
        match report.value() {
            Some(Sign::Positive | Sign::Negative) => {
                return TrianglePredicateReport {
                    degeneracy: TriangleDegeneracy::NonDegenerate,
                    predicates,
                };
            }
            Some(Sign::Zero) => {}
            None => all_zero = false,
        }
    }

    TrianglePredicateReport {
        degeneracy: if all_zero {
            TriangleDegeneracy::Degenerate
        } else {
            TriangleDegeneracy::Unknown
        },
        predicates,
    }
}

/// Classify two already-coplanar indexed triangles by exact projected 2D
/// predicates.
pub fn classify_coplanar_triangles(
    points: &[Point3],
    left: [usize; 3],
    right: [usize; 3],
) -> CoplanarTriangleClassification {
    if !indices_in_range(points, left) || !indices_in_range(points, right) {
        return CoplanarTriangleClassification {
            projection: None,
            relation: CoplanarTriangleRelation::Unknown,
            edge_intersections: Vec::new(),
            right_vertices_in_left: [None, None, None],
            left_vertices_in_right: [None, None, None],
            predicates: Vec::new(),
        };
    }
    let left_points = [&points[left[0]], &points[left[1]], &points[left[2]]];
    let right_points = [&points[right[0]], &points[right[1]], &points[right[2]]];
    classify_coplanar_triangle_points(left_points, right_points)
}

/// Classify two already-coplanar triangles by exact projected 2D predicates.
pub fn classify_coplanar_triangle_points(
    left: [&Point3; 3],
    right: [&Point3; 3],
) -> CoplanarTriangleClassification {
    let Some((projection, predicates)) = choose_coplanar_projection(left) else {
        return CoplanarTriangleClassification {
            projection: None,
            relation: CoplanarTriangleRelation::Unknown,
            edge_intersections: Vec::new(),
            right_vertices_in_left: [None, None, None],
            left_vertices_in_right: [None, None, None],
            predicates: Vec::new(),
        };
    };

    let left2 = project_triangle3(left, projection);
    let right2 = project_triangle3(right, projection);
    let mut edge_intersections = Vec::with_capacity(9);
    let mut saw_touch = false;
    let mut saw_overlap = false;

    for left_edge in triangle_edges2(&left2) {
        for right_edge in triangle_edges2(&right2) {
            match classify_segment_intersection(
                left_edge[0],
                left_edge[1],
                right_edge[0],
                right_edge[1],
            ) {
                PredicateOutcome::Decided { value, .. } => {
                    if value.is_proper_crossing() || value.has_positive_length_overlap() {
                        saw_overlap = true;
                    } else if value.is_endpoint_touch() {
                        saw_touch = true;
                    }
                    edge_intersections.push(value);
                }
                PredicateOutcome::Unknown { .. } => {
                    return unknown_with_projection(projection, predicates, edge_intersections);
                }
            }
        }
    }

    let right_vertices_in_left = classify_vertices_in_triangle(&left2, &right2);
    if right_vertices_in_left.iter().any(Option::is_none) {
        return unknown_with_projection(projection, predicates, edge_intersections);
    }
    let left_vertices_in_right = classify_vertices_in_triangle(&right2, &left2);
    if left_vertices_in_right.iter().any(Option::is_none) {
        return unknown_with_projection(projection, predicates, edge_intersections);
    }

    for location in right_vertices_in_left
        .iter()
        .chain(left_vertices_in_right.iter())
        .flatten()
    {
        match location {
            TriangleLocation::Inside => saw_overlap = true,
            TriangleLocation::OnEdge | TriangleLocation::OnVertex => saw_touch = true,
            TriangleLocation::Degenerate | TriangleLocation::Outside => {}
        }
    }

    let relation = if saw_overlap {
        CoplanarTriangleRelation::Overlapping
    } else if saw_touch {
        CoplanarTriangleRelation::Touching
    } else {
        CoplanarTriangleRelation::Disjoint
    };

    CoplanarTriangleClassification {
        projection: Some(projection),
        relation,
        edge_intersections,
        right_vertices_in_left,
        left_vertices_in_right,
        predicates,
    }
}

/// Choose a coordinate projection whose triangle orientation is certified
/// nonzero.
pub fn choose_coplanar_projection(
    triangle: [&Point3; 3],
) -> Option<(CoplanarProjection, Vec<PredicateUse>)> {
    let mut predicates = Vec::with_capacity(3);
    for projection in [
        CoplanarProjection::Xy,
        CoplanarProjection::Xz,
        CoplanarProjection::Yz,
    ] {
        let projected = project_triangle3(triangle, projection);
        let report = orient2d_report(&projected[0], &projected[1], &projected[2]);
        predicates.push(PredicateUse::from_certificate(report.certificate));
        if matches!(report.value(), Some(Sign::Positive | Sign::Negative)) {
            return Some((projection, predicates));
        }
    }
    None
}

/// Project one 3D point into a coordinate plane.
pub fn project_point3(point: &Point3, projection: CoplanarProjection) -> Point2 {
    match projection {
        CoplanarProjection::Xy => Point2::new(point.x.clone(), point.y.clone()),
        CoplanarProjection::Xz => Point2::new(point.x.clone(), point.z.clone()),
        CoplanarProjection::Yz => Point2::new(point.y.clone(), point.z.clone()),
    }
}

/// Project one 3D triangle into a coordinate plane.
pub fn project_triangle3(points: [&Point3; 3], projection: CoplanarProjection) -> [Point2; 3] {
    [
        project_point3(points[0], projection),
        project_point3(points[1], projection),
        project_point3(points[2], projection),
    ]
}

/// Return the signed doubled projected polygon area under a coordinate
/// projection.
pub fn projected_polygon_area2_sign(
    points: &[Point3],
    projection: CoplanarProjection,
) -> crate::PredicateOutcome<Sign> {
    let ring = points
        .iter()
        .map(|point| project_point3(point, projection))
        .collect::<Vec<_>>();
    ring_area_sign(&ring)
}

/// Return the exact doubled projected polygon area under a coordinate
/// projection.
pub fn projected_polygon_area2_value(points: &[Point3], projection: CoplanarProjection) -> Real {
    if points.len() < 3 {
        return Real::from(0);
    }
    let mut sum = Real::from(0);
    for index in 0..points.len() {
        let current = project_point3(&points[index], projection);
        let next = project_point3(&points[(index + 1) % points.len()], projection);
        sum += current.x * next.y.clone() - current.y * next.x;
    }
    sum
}

/// Return the absolute doubled projected polygon area under a coordinate
/// projection.
pub fn projected_polygon_area2_abs_value(
    points: &[Point3],
    projection: CoplanarProjection,
) -> Option<Real> {
    let signed = projected_polygon_area2_value(points, projection);
    match crate::predicates::order::compare_reals(&signed, &Real::from(0)).value()? {
        core::cmp::Ordering::Less => Some(Real::from(0) - &signed),
        core::cmp::Ordering::Equal | core::cmp::Ordering::Greater => Some(signed),
    }
}

/// Return the exact midpoint of two 3D points.
pub fn midpoint3(a: &Point3, b: &Point3) -> Point3 {
    let half = (Real::from(1) / &Real::from(2)).expect("2 is nonzero");
    Point3::new(
        (a.x.clone() + &b.x) * &half,
        (a.y.clone() + &b.y) * &half,
        (a.z.clone() + &b.z) * &half,
    )
}

/// Return the projected 2D vector from `from` to `to`.
pub fn projected_vector3(from: &Point3, to: &Point3, projection: CoplanarProjection) -> Point2 {
    let from = project_point3(from, projection);
    let to = project_point3(to, projection);
    Point2::new(to.x - &from.x, to.y - &from.y)
}

/// Return whether `left` is a smaller counter-clockwise turn from `base` than
/// `right`, using exact 2D cross/dot comparisons.
pub fn ccw_projected_turn_less(base: &Point2, left: &Point2, right: &Point2) -> Option<bool> {
    let left_bucket = ccw_turn_bucket(base, left)?;
    let right_bucket = ccw_turn_bucket(base, right)?;
    if left_bucket != right_bucket {
        return Some(left_bucket < right_bucket);
    }
    match crate::predicates::order::compare_reals(&cross2(left, right), &Real::from(0)).value()? {
        core::cmp::Ordering::Greater => Some(true),
        core::cmp::Ordering::Less | core::cmp::Ordering::Equal => Some(false),
    }
}

/// Classify a 3D point after projecting it and a 3D triangle to a coordinate
/// plane.
pub fn classify_point_projected_triangle3(
    point: &Point3,
    triangle: [&Point3; 3],
    projection: CoplanarProjection,
) -> PredicateOutcome<TriangleLocation> {
    let query = project_point3(point, projection);
    let a = project_point3(triangle[0], projection);
    let b = project_point3(triangle[1], projection);
    let c = project_point3(triangle[2], projection);
    classify_point_triangle(&a, &b, &c, &query)
}

/// Construct the exact 3D point where a segment crosses a projected 3D line.
///
/// Callers should only consume this helper after exact predicates have
/// certified the segment/line topology.
pub fn intersect_segment_with_projected_line3(
    segment_start: &Point3,
    segment_end: &Point3,
    line_start: &Point3,
    line_end: &Point3,
    projection: CoplanarProjection,
) -> Option<Point3> {
    let parameter =
        projected_line_parameter3(segment_start, segment_end, line_start, line_end, projection)?;
    Some(interpolate_projected_point3(
        segment_start,
        segment_end,
        &parameter,
    ))
}

/// Return the exact signed 2D orientation determinant.
///
/// This is the raw determinant value behind the orientation predicate. Callers
/// must still use [`orient2d_report`] or another certified sign classifier for
/// topology decisions; the value helper exists for exact construction
/// parameters that are consumed only after predicates have selected the
/// combinatorial case, preserving the predicate/construction boundary.
pub fn orient2d_value(a: &Point2, b: &Point2, c: &Point2) -> Real {
    (b.x.clone() - &a.x) * (c.y.clone() - &a.y) - (b.y.clone() - &a.y) * (c.x.clone() - &a.x)
}

/// Return the exact segment parameter for a projected 3D point.
///
/// The point is first projected with [`project_point3`], then one nonconstant
/// coordinate axis supplies the affine parameter. The helper does not certify
/// incidence by itself; callers should first use a predicate such as
/// `point_on_segment` on the projected points. This keeps construction
/// recovery behind predicate evidence.
pub fn projected_segment_parameter3(
    point: &Point3,
    start: &Point3,
    end: &Point3,
    projection: CoplanarProjection,
) -> Option<Real> {
    let point = project_point3(point, projection);
    let start = project_point3(start, projection);
    let end = project_point3(end, projection);
    segment_parameter_from_axis(&point.x, &start.x, &end.x)
        .or_else(|| segment_parameter_from_axis(&point.y, &start.y, &end.y))
}

/// Return the exact parameter where a projected 3D segment crosses a projected
/// 3D line.
///
/// The formula is `d0 / (d0 - d1)`, where `d0` and `d1` are exact projected
/// orientation determinants against the supporting line. This is a
/// construction helper, not a predicate: callers must only consume the result
/// after segment/line topology has been certified by exact predicates.
pub fn projected_line_parameter3(
    segment_start: &Point3,
    segment_end: &Point3,
    line_start: &Point3,
    line_end: &Point3,
    projection: CoplanarProjection,
) -> Option<Real> {
    let a = project_point3(line_start, projection);
    let b = project_point3(line_end, projection);
    let p0 = project_point3(segment_start, projection);
    let p1 = project_point3(segment_end, projection);
    let d0 = orient2d_value(&a, &b, &p0);
    let d1 = orient2d_value(&a, &b, &p1);
    let denominator = d0.clone() - &d1;
    if matches!(
        crate::predicates::order::compare_reals(&denominator, &Real::from(0)).value(),
        Some(core::cmp::Ordering::Equal) | None
    ) {
        return None;
    }
    (d0 / &denominator).ok()
}

fn interpolate_projected_point3(start: &Point3, end: &Point3, t: &Real) -> Point3 {
    let one_minus_t = Real::from(1) - t;
    Point3::new(
        start.x.clone() * &one_minus_t + end.x.clone() * t,
        start.y.clone() * &one_minus_t + end.y.clone() * t,
        start.z.clone() * &one_minus_t + end.z.clone() * t,
    )
}

fn ccw_turn_bucket(base: &Point2, candidate: &Point2) -> Option<u8> {
    match crate::predicates::order::compare_reals(&cross2(base, candidate), &Real::from(0))
        .value()?
    {
        core::cmp::Ordering::Greater => Some(0),
        core::cmp::Ordering::Less => Some(1),
        core::cmp::Ordering::Equal => {
            match crate::predicates::order::compare_reals(&dot2(base, candidate), &Real::from(0))
                .value()?
            {
                core::cmp::Ordering::Greater | core::cmp::Ordering::Equal => Some(0),
                core::cmp::Ordering::Less => Some(1),
            }
        }
    }
}

fn cross2(left: &Point2, right: &Point2) -> Real {
    left.x.clone() * &right.y - left.y.clone() * &right.x
}

fn dot2(left: &Point2, right: &Point2) -> Real {
    left.x.clone() * &right.x + left.y.clone() * &right.y
}

/// Derive the collapsed coplanar relation from retained edge and vertex facts.
pub fn derive_coplanar_triangle_relation(
    edge_intersections: &[SegmentIntersection],
    right_vertices_in_left: [Option<TriangleLocation>; 3],
    left_vertices_in_right: [Option<TriangleLocation>; 3],
) -> CoplanarTriangleRelation {
    let mut saw_touch = false;
    let mut saw_overlap = false;
    for relation in edge_intersections {
        if relation.is_proper_crossing() || relation.has_positive_length_overlap() {
            saw_overlap = true;
        } else if relation.is_endpoint_touch() {
            saw_touch = true;
        }
    }
    for location in right_vertices_in_left
        .iter()
        .chain(left_vertices_in_right.iter())
        .flatten()
    {
        match location {
            TriangleLocation::Inside => saw_overlap = true,
            TriangleLocation::OnEdge | TriangleLocation::OnVertex => saw_touch = true,
            TriangleLocation::Degenerate | TriangleLocation::Outside => {}
        }
    }
    if saw_overlap {
        CoplanarTriangleRelation::Overlapping
    } else if saw_touch {
        CoplanarTriangleRelation::Touching
    } else {
        CoplanarTriangleRelation::Disjoint
    }
}

fn indices_in_range(points: &[Point3], indices: [usize; 3]) -> bool {
    indices.iter().all(|&index| index < points.len())
}

fn triangle_edges2(tri: &[Point2; 3]) -> [[&Point2; 2]; 3] {
    [[&tri[0], &tri[1]], [&tri[1], &tri[2]], [&tri[2], &tri[0]]]
}

fn classify_vertices_in_triangle(
    triangle: &[Point2; 3],
    query: &[Point2; 3],
) -> [Option<TriangleLocation>; 3] {
    [
        classify_point_triangle(&triangle[0], &triangle[1], &triangle[2], &query[0]).value(),
        classify_point_triangle(&triangle[0], &triangle[1], &triangle[2], &query[1]).value(),
        classify_point_triangle(&triangle[0], &triangle[1], &triangle[2], &query[2]).value(),
    ]
}

fn unknown_with_projection(
    projection: CoplanarProjection,
    predicates: Vec<PredicateUse>,
    edge_intersections: Vec<SegmentIntersection>,
) -> CoplanarTriangleClassification {
    CoplanarTriangleClassification {
        projection: Some(projection),
        relation: CoplanarTriangleRelation::Unknown,
        edge_intersections,
        right_vertices_in_left: [None, None, None],
        left_vertices_in_right: [None, None, None],
        predicates,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Real;

    fn p3(x: i64, y: i64, z: i64) -> Point3 {
        Point3::new(Real::from(x), Real::from(y), Real::from(z))
    }

    #[test]
    fn triangle3_degeneracy_uses_projected_orientations() {
        let report = classify_triangle3_degeneracy(&p3(0, 0, 0), &p3(1, 0, 0), &p3(0, 1, 0));
        assert_eq!(report.degeneracy, TriangleDegeneracy::NonDegenerate);
        assert!(report.all_proof_producing());

        let degenerate = classify_triangle3_degeneracy(&p3(0, 0, 0), &p3(1, 1, 1), &p3(2, 2, 2));
        assert_eq!(degenerate.degeneracy, TriangleDegeneracy::Degenerate);
        assert!(degenerate.all_proof_producing());
    }

    #[test]
    fn coplanar_triangle_classifier_distinguishes_disjoint_touching_and_overlap() {
        let disjoint_points = [
            p3(0, 0, 0),
            p3(4, 0, 0),
            p3(0, 4, 0),
            p3(5, 5, 0),
            p3(7, 5, 0),
            p3(5, 7, 0),
        ];
        let touching_points = [
            p3(0, 0, 0),
            p3(4, 0, 0),
            p3(0, 4, 0),
            p3(4, 0, 0),
            p3(6, 0, 0),
            p3(4, 2, 0),
        ];
        let overlapping_points = [
            p3(0, 0, 0),
            p3(4, 0, 0),
            p3(0, 4, 0),
            p3(1, 1, 0),
            p3(5, 1, 0),
            p3(1, 5, 0),
        ];

        let disjoint = classify_coplanar_triangles(&disjoint_points, [0, 1, 2], [3, 4, 5]);
        let touching = classify_coplanar_triangles(&touching_points, [0, 1, 2], [3, 4, 5]);
        let overlapping = classify_coplanar_triangles(&overlapping_points, [0, 1, 2], [3, 4, 5]);

        assert_eq!(disjoint.relation, CoplanarTriangleRelation::Disjoint);
        assert_eq!(touching.relation, CoplanarTriangleRelation::Touching);
        assert_eq!(overlapping.relation, CoplanarTriangleRelation::Overlapping);
        disjoint
            .validate_against_sources(&disjoint_points, [0, 1, 2], [3, 4, 5])
            .unwrap();
        touching
            .validate_against_sources(&touching_points, [0, 1, 2], [3, 4, 5])
            .unwrap();
        overlapping
            .validate_against_sources(&overlapping_points, [0, 1, 2], [3, 4, 5])
            .unwrap();
    }

    #[test]
    fn projected_parameters_preserve_exact_affine_ratios() {
        let start = p3(0, 0, 0);
        let end = p3(4, 0, 0);
        let midpoint = p3(2, 0, 0);
        let half = (Real::from(1) / &Real::from(2)).unwrap();

        assert_eq!(
            projected_segment_parameter3(&midpoint, &start, &end, CoplanarProjection::Xy),
            Some(half.clone())
        );

        let crossing = projected_line_parameter3(
            &p3(0, -2, 0),
            &p3(0, 2, 0),
            &p3(-1, 0, 0),
            &p3(1, 0, 0),
            CoplanarProjection::Xy,
        );
        assert_eq!(crossing, Some(half));
    }
}
