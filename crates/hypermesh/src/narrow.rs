//! Exact narrow-phase classification helpers.
//!
//! Full triangle/triangle intersection is deliberately not reimplemented here
//! as another local tolerance algorithm. Instead this module exposes certified
//! primitives that legacy kernels can migrate onto: classify triangle vertices
//! against an oriented face plane and retain the predicate route. Plane-side
//! orientation predicates come from `hyperlimit`, and each classification
//! retains the certificate that produced it.
//!
//! The plane-side rejection and candidate staging follows Moller, "A Fast
//! Triangle-Triangle Intersection Test," *Journal of Graphics Tools* 2.2
//! (1997). Coplanar overlap is delegated to exact orientation predicates in
//! the style of Guigue and Devillers, "Fast and Robust Triangle-Triangle
//! Overlap Test Using Orientation Predicates," *Journal of Graphics Tools* 8.1
//! (2003).

use core::cmp::Ordering;

use hyperlimit::{
    PlaneSide, Point3, classify_triangle_against_oriented_plane, compare_reals,
    triangle_plane_relation_from_sides,
};

use super::construction::{SegmentPlaneIntersection, intersect_segment_with_face_plane};
use super::error::{DiagnosticKind, MeshDiagnostic, MeshError, Severity};
use super::mesh::ExactMesh;
use hyperlimit::{
    CoplanarTriangleClassification, CoplanarTriangleRelation, classify_coplanar_triangles,
};
use hyperreal::Real;

pub use hyperlimit::{
    TrianglePlaneClassification, TrianglePlaneRelation, TrianglePlaneValidationError,
};

/// Certified coarse relation between two exact triangles.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum TriangleTriangleRelation {
    /// The second triangle lies strictly on one side of the first triangle's
    /// plane.
    SeparatedByFirstPlane,
    /// The first triangle lies strictly on one side of the second triangle's
    /// plane.
    SeparatedBySecondPlane,
    /// Both triangles are coplanar but exact projected 2D predicates prove the
    /// closed triangles are disjoint.
    CoplanarDisjoint,
    /// Both triangles are coplanar and touch at a vertex or edge.
    CoplanarTouching,
    /// Both triangles are coplanar and overlap with positive area or a
    /// positive-length edge interval.
    CoplanarOverlapping,
    /// Plane-side predicates prove a non-coplanar candidate requiring exact
    /// segment/triangle and interval ordering.
    Candidate,
    /// At least one required plane-side predicate was undecided.
    Unknown,
}

/// Structural inconsistency in a triangle/triangle classifier report.
///
/// This is the narrow-phase handoff check before mesh face-pair scheduling.
/// It verifies that the two plane classifications, optional coplanar report,
/// and retained segment/plane construction events agree with the collapsed
/// constructions remain auditable objects until validated by the consumer
/// layer.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum TriangleTriangleValidationError {
    /// A nested triangle/plane classifier was internally inconsistent.
    InvalidPlaneClassification,
    /// The two retained plane classifications do not imply the retained
    /// triangle/triangle relation before coplanar refinement.
    PlaneRelationMismatch,
    /// A coplanar relation did not retain its projected coplanar classifier.
    CoplanarRelationMissingClassifier,
    /// The retained projected coplanar classifier does not match the collapsed
    /// triangle/triangle relation.
    CoplanarRelationMismatch,
    /// A non-coplanar relation retained a coplanar classifier.
    UnexpectedCoplanarClassifier,
    /// A candidate relation did not retain three edge events for each query
    /// triangle.
    CandidateEdgeEventCountMismatch,
    /// A retained segment/plane construction event was internally invalid.
    InvalidSegmentPlaneEvent,
    /// A non-candidate relation retained segment/plane construction events.
    UnexpectedEdgeEvents,
    /// Recomputing the classifier from the supplied source triangles did not
    /// reproduce this retained report.
    SourceReplayMismatch,
}

/// Certified triangle/triangle coarse classification.
///
/// This intentionally stops before coplanar overlap and full intersection
/// other triangle's plane. Hypermesh performs that stage through
/// `hyperlimit::orient3d_report` and keeps the segment/plane construction
/// events needed by the later exact splitter.
#[derive(Clone, Debug, PartialEq)]
pub struct TriangleTriangleClassification {
    /// Coarse relation.
    pub relation: TriangleTriangleRelation,
    /// Classification of the right triangle against the left triangle's plane.
    pub right_against_left_plane: TrianglePlaneClassification,
    /// Classification of the left triangle against the right triangle's plane.
    pub left_against_right_plane: TrianglePlaneClassification,
    /// Right-triangle edge events against the left plane.
    pub right_edge_events: Vec<SegmentPlaneIntersection>,
    /// Left-triangle edge events against the right plane.
    pub left_edge_events: Vec<SegmentPlaneIntersection>,
    /// Exact projected overlap result for coplanar pairs.
    pub coplanar: Option<CoplanarTriangleClassification>,
}

impl TriangleTriangleClassification {
    /// Return whether every retained predicate route was proof-producing.
    pub fn all_proof_producing(&self) -> bool {
        self.right_against_left_plane.all_proof_producing()
            && self.left_against_right_plane.all_proof_producing()
            && self
                .right_edge_events
                .iter()
                .all(SegmentPlaneIntersection::all_proof_producing)
            && self
                .left_edge_events
                .iter()
                .all(SegmentPlaneIntersection::all_proof_producing)
            && self
                .coplanar
                .as_ref()
                .is_none_or(CoplanarTriangleClassification::projection_proof_producing)
    }

    /// Validate retained narrow-phase classifier invariants.
    ///
    /// This check is intentionally local to the classifier artifact. It does
    /// not recompute plane predicates or projected overlap; it verifies that
    /// the retained subreports and construction events are consistent with the
    /// public relation.
    pub fn validate(&self) -> Result<(), TriangleTriangleValidationError> {
        self.right_against_left_plane
            .validate()
            .map_err(|_| TriangleTriangleValidationError::InvalidPlaneClassification)?;
        self.left_against_right_plane
            .validate()
            .map_err(|_| TriangleTriangleValidationError::InvalidPlaneClassification)?;

        let plane_relation = triangle_triangle_relation(
            self.right_against_left_plane.relation,
            self.left_against_right_plane.relation,
        );
        match self.relation {
            TriangleTriangleRelation::SeparatedByFirstPlane
            | TriangleTriangleRelation::SeparatedBySecondPlane
            | TriangleTriangleRelation::Candidate => {
                if self.relation != plane_relation {
                    return Err(TriangleTriangleValidationError::PlaneRelationMismatch);
                }
                if self.coplanar.is_some() {
                    return Err(TriangleTriangleValidationError::UnexpectedCoplanarClassifier);
                }
            }
            TriangleTriangleRelation::CoplanarDisjoint
            | TriangleTriangleRelation::CoplanarTouching
            | TriangleTriangleRelation::CoplanarOverlapping => {
                if plane_relation != TriangleTriangleRelation::CoplanarOverlapping {
                    return Err(TriangleTriangleValidationError::PlaneRelationMismatch);
                }
                let Some(coplanar) = &self.coplanar else {
                    return Err(TriangleTriangleValidationError::CoplanarRelationMissingClassifier);
                };
                coplanar
                    .validate()
                    .map_err(|_| TriangleTriangleValidationError::CoplanarRelationMismatch)?;
                if triangle_relation_from_coplanar(coplanar.relation) != self.relation {
                    return Err(TriangleTriangleValidationError::CoplanarRelationMismatch);
                }
            }
            TriangleTriangleRelation::Unknown => {
                if plane_relation != TriangleTriangleRelation::Unknown
                    && !matches!(
                        self.coplanar.as_ref().map(|coplanar| coplanar.relation),
                        Some(CoplanarTriangleRelation::Unknown)
                    )
                {
                    return Err(TriangleTriangleValidationError::PlaneRelationMismatch);
                }
            }
        }

        if self.relation == TriangleTriangleRelation::Candidate {
            if self.right_edge_events.len() != 3 || self.left_edge_events.len() != 3 {
                return Err(TriangleTriangleValidationError::CandidateEdgeEventCountMismatch);
            }
            validate_segment_events(&self.right_edge_events)?;
            validate_segment_events(&self.left_edge_events)?;
        } else if !self.right_edge_events.is_empty() || !self.left_edge_events.is_empty() {
            return Err(TriangleTriangleValidationError::UnexpectedEdgeEvents);
        }
        Ok(())
    }

    /// Validate this classifier against the source point and triangle handles.
    ///
    /// Local validation proves that retained subreports agree with the coarse
    /// relation. Source replay recomputes the full narrow-phase classifier,
    /// including plane-side predicates, coplanar projection predicates, and
    /// candidate segment/plane construction events, then requires exact
    /// equality with this retained report. This is the auditable-object
    pub fn validate_against_sources(
        &self,
        points: &[Point3],
        left: [usize; 3],
        right: [usize; 3],
    ) -> Result<(), TriangleTriangleValidationError> {
        self.validate()?;
        if !indices_in_range(points, left) || !indices_in_range(points, right) {
            return Err(TriangleTriangleValidationError::SourceReplayMismatch);
        }
        let replay = classify_triangle_triangle(points, left, right);
        if self == &replay {
            Ok(())
        } else {
            Err(TriangleTriangleValidationError::SourceReplayMismatch)
        }
    }
}

/// Classify a query triangle against an oriented face plane.
pub fn classify_triangle_against_face_plane(
    points: &[Point3],
    face: [usize; 3],
    query: [usize; 3],
) -> TrianglePlaneClassification {
    classify_triangle_against_oriented_plane(
        [&points[face[0]], &points[face[1]], &points[face[2]]],
        [&points[query[0]], &points[query[1]], &points[query[2]]],
    )
}

/// Classify a mesh triangle against a retained exact face plane.
///
/// This is the cached-object counterpart to
/// [`classify_triangle_against_face_plane`]. It evaluates the unnormalized
/// determinant-form plane coefficients retained in [`super::facts::FacePlaneFacts`]
/// this shape directly: object-level numerical structure should survive so
/// later topology stages can reuse exact facts instead of reconstructing
/// normals or representative floats.
pub fn classify_mesh_triangle_against_retained_face_plane(
    plane_mesh: &ExactMesh,
    plane_face: usize,
    query_mesh: &ExactMesh,
    query_face: usize,
) -> Result<TrianglePlaneClassification, MeshError> {
    if plane_face >= plane_mesh.triangles().len() {
        return Err(index_error(
            "plane face index is out of range",
            Some(plane_face),
            None,
        ));
    }
    if query_face >= query_mesh.triangles().len() {
        return Err(index_error(
            "query face index is out of range",
            Some(query_face),
            None,
        ));
    }
    let plane = &plane_mesh.facts().faces[plane_face].plane;
    let query = query_mesh.triangles()[query_face].0;
    let mut sides = [None, None, None];
    for (side, vertex) in sides.iter_mut().zip(query) {
        let point = query_mesh.vertices()[vertex].clone();
        *side = retained_plane_side(plane, &point);
    }

    Ok(TrianglePlaneClassification {
        relation: triangle_plane_relation_from_sides(sides),
        vertex_sides: sides,
        predicates: Vec::new(),
    })
}

/// Coarsely classify two triangles using certified plane-side predicates.
pub fn classify_triangle_triangle(
    points: &[Point3],
    left: [usize; 3],
    right: [usize; 3],
) -> TriangleTriangleClassification {
    let right_against_left_plane = classify_triangle_against_face_plane(points, left, right);
    let left_against_right_plane = classify_triangle_against_face_plane(points, right, left);
    let mut relation = triangle_triangle_relation(
        right_against_left_plane.relation,
        left_against_right_plane.relation,
    );
    let coplanar = if relation == TriangleTriangleRelation::CoplanarOverlapping {
        let coplanar = classify_coplanar_triangles(points, left, right);
        relation = match coplanar.relation {
            CoplanarTriangleRelation::Disjoint => TriangleTriangleRelation::CoplanarDisjoint,
            CoplanarTriangleRelation::Touching => TriangleTriangleRelation::CoplanarTouching,
            CoplanarTriangleRelation::Overlapping => TriangleTriangleRelation::CoplanarOverlapping,
            CoplanarTriangleRelation::Unknown => TriangleTriangleRelation::Unknown,
        };
        Some(coplanar)
    } else {
        None
    };

    let (right_edge_events, left_edge_events) = if relation == TriangleTriangleRelation::Candidate {
        (
            triangle_edge_events_against_plane(points, left, right),
            triangle_edge_events_against_plane(points, right, left),
        )
    } else {
        (Vec::new(), Vec::new())
    };

    TriangleTriangleClassification {
        relation,
        right_against_left_plane,
        left_against_right_plane,
        right_edge_events,
        left_edge_events,
        coplanar,
    }
}

fn retained_plane_side(plane: &super::facts::FacePlaneFacts, point: &Point3) -> Option<PlaneSide> {
    let value = add(
        &add(
            &add(
                &mul(&plane.normal[0], &point.x),
                &mul(&plane.normal[1], &point.y),
            ),
            &mul(&plane.normal[2], &point.z),
        ),
        &plane.offset,
    );
    // `hyperlimit::orient3d_report(a, b, c, p)` uses the opposite sign
    // convention from this stored `(b - a) x (c - a)` dot-product form, so the
    // exact comparison is inverted to preserve the public `PlaneSide` contract.
    match compare_reals(&value, &Real::from(0)).value()? {
        Ordering::Less => Some(PlaneSide::Above),
        Ordering::Equal => Some(PlaneSide::On),
        Ordering::Greater => Some(PlaneSide::Below),
    }
}

fn index_error(message: &'static str, face: Option<usize>, vertex: Option<usize>) -> MeshError {
    let mut diagnostic =
        MeshDiagnostic::new(Severity::Error, DiagnosticKind::IndexOutOfBounds, message);
    if let Some(face) = face {
        diagnostic = diagnostic.with_face(face);
    }
    if let Some(vertex) = vertex {
        diagnostic = diagnostic.with_vertex(vertex);
    }
    MeshError::one(diagnostic)
}

fn add(left: &Real, right: &Real) -> Real {
    left.clone() + right
}

fn mul(left: &Real, right: &Real) -> Real {
    left.clone() * right
}

fn indices_in_range(points: &[Point3], indices: [usize; 3]) -> bool {
    indices.iter().all(|&index| index < points.len())
}

fn triangle_triangle_relation(
    right_against_left: TrianglePlaneRelation,
    left_against_right: TrianglePlaneRelation,
) -> TriangleTriangleRelation {
    if matches!(
        right_against_left,
        TrianglePlaneRelation::StrictlyAbove | TrianglePlaneRelation::StrictlyBelow
    ) {
        return TriangleTriangleRelation::SeparatedByFirstPlane;
    }
    if matches!(
        left_against_right,
        TrianglePlaneRelation::StrictlyAbove | TrianglePlaneRelation::StrictlyBelow
    ) {
        return TriangleTriangleRelation::SeparatedBySecondPlane;
    }
    if right_against_left == TrianglePlaneRelation::Unknown
        || left_against_right == TrianglePlaneRelation::Unknown
    {
        return TriangleTriangleRelation::Unknown;
    }
    if right_against_left == TrianglePlaneRelation::Coplanar
        && left_against_right == TrianglePlaneRelation::Coplanar
    {
        return TriangleTriangleRelation::CoplanarOverlapping;
    }
    TriangleTriangleRelation::Candidate
}

fn triangle_relation_from_coplanar(relation: CoplanarTriangleRelation) -> TriangleTriangleRelation {
    match relation {
        CoplanarTriangleRelation::Disjoint => TriangleTriangleRelation::CoplanarDisjoint,
        CoplanarTriangleRelation::Touching => TriangleTriangleRelation::CoplanarTouching,
        CoplanarTriangleRelation::Overlapping => TriangleTriangleRelation::CoplanarOverlapping,
        CoplanarTriangleRelation::Unknown => TriangleTriangleRelation::Unknown,
    }
}

fn validate_segment_events(
    events: &[SegmentPlaneIntersection],
) -> Result<(), TriangleTriangleValidationError> {
    for event in events {
        event
            .validate()
            .map_err(|_| TriangleTriangleValidationError::InvalidSegmentPlaneEvent)?;
    }
    Ok(())
}

fn triangle_edge_events_against_plane(
    points: &[Point3],
    plane_face: [usize; 3],
    query: [usize; 3],
) -> Vec<SegmentPlaneIntersection> {
    [
        [query[0], query[1]],
        [query[1], query[2]],
        [query[2], query[0]],
    ]
    .into_iter()
    .map(|edge| intersect_segment_with_face_plane(points, plane_face, edge))
    .collect()
}
