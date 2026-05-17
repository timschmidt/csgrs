//! Exact coplanar triangle overlap classification.
//!
//! Coplanar triangle overlap is a 2D arrangement problem embedded in 3D. This
//! module projects the triangles only onto a coordinate plane whose projected
//! orientation is certified nonzero by `hyperlimit::orient2d_report`; no
//! primitive-float normal magnitude or epsilon selects the projection. The
//! actual overlap tests then use `hyperlimit` segment and point-in-triangle
//! predicates. This follows Yap, "Towards Exact Geometric Computation,"
//! *Computational Geometry* 7.1-2 (1997): representation choices may preserve
//! structure, but combinatorial claims require certified predicates.
//!
//! The decomposition into projected segment intersections plus containment is
//! the coplanar counterpart to Guigue and Devillers, "Fast and Robust
//! Triangle-Triangle Overlap Test Using Orientation Predicates," *Journal of
//! Graphics Tools* 8.1 (2003).

use hyperlimit::{
    Point2, Point3, PredicateOutcome, SegmentIntersection, Sign, TriangleLocation,
    classify_point_triangle, classify_segment_intersection, orient2d_report,
};

use super::provenance::PredicateUse;

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
    /// proofs. Segment and point predicates currently expose outcomes rather
    /// than report certificates, so they are reflected in `relation`.
    pub fn projection_proof_producing(&self) -> bool {
        self.predicates
            .iter()
            .copied()
            .all(PredicateUse::is_proof_producing)
    }
}

/// Classify two already-coplanar triangles by exact projected 2D predicates.
pub fn classify_coplanar_triangles(
    points: &[Point3],
    left: [usize; 3],
    right: [usize; 3],
) -> CoplanarTriangleClassification {
    let Some((projection, predicates)) = choose_projection(points, left) else {
        return CoplanarTriangleClassification {
            projection: None,
            relation: CoplanarTriangleRelation::Unknown,
            edge_intersections: Vec::new(),
            right_vertices_in_left: [None, None, None],
            left_vertices_in_right: [None, None, None],
            predicates: Vec::new(),
        };
    };

    let left2 = project_triangle(points, left, projection);
    let right2 = project_triangle(points, right, projection);
    let mut edge_intersections = Vec::with_capacity(9);
    let mut saw_touch = false;
    let mut saw_overlap = false;

    for left_edge in triangle_edges(&left2) {
        for right_edge in triangle_edges(&right2) {
            match classify_segment_intersection(
                left_edge[0],
                left_edge[1],
                right_edge[0],
                right_edge[1],
            ) {
                PredicateOutcome::Decided { value, .. } => {
                    if matches!(
                        value,
                        SegmentIntersection::Proper
                            | SegmentIntersection::CollinearOverlap
                            | SegmentIntersection::Identical
                    ) {
                        saw_overlap = true;
                    } else if value == SegmentIntersection::EndpointTouch {
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

fn choose_projection(
    points: &[Point3],
    tri: [usize; 3],
) -> Option<(CoplanarProjection, Vec<PredicateUse>)> {
    for projection in [
        CoplanarProjection::Xy,
        CoplanarProjection::Xz,
        CoplanarProjection::Yz,
    ] {
        let projected = project_triangle(points, tri, projection);
        let report = orient2d_report(&projected[0], &projected[1], &projected[2]);
        let predicate = PredicateUse::from_certificate(report.certificate);
        if report.value() != Some(Sign::Zero) {
            return Some((projection, vec![predicate]));
        }
    }
    None
}

fn project_triangle(
    points: &[Point3],
    tri: [usize; 3],
    projection: CoplanarProjection,
) -> [Point2; 3] {
    [
        project_point(&points[tri[0]], projection),
        project_point(&points[tri[1]], projection),
        project_point(&points[tri[2]], projection),
    ]
}

fn project_point(point: &Point3, projection: CoplanarProjection) -> Point2 {
    match projection {
        CoplanarProjection::Xy => Point2::new(point.x.clone(), point.y.clone()),
        CoplanarProjection::Xz => Point2::new(point.x.clone(), point.z.clone()),
        CoplanarProjection::Yz => Point2::new(point.y.clone(), point.z.clone()),
    }
}

fn triangle_edges(tri: &[Point2; 3]) -> [[&Point2; 2]; 3] {
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
