//! Exact winding classification for triangulated volumetric split regions.
//!
//! The construction graph already supplies exact split face regions. This
//! module adds the named-boolean semantic layer for closed volumetric meshes:
//! choose an exact interior representative point from each triangulated source
//! cell, classify that point by the closed opposite mesh's exact ray-parity
//! winding report, and retain the classification beside the output assembly.
//! The classifier tries the centroid first, then deterministic exact
//! barycentric interior witnesses if that centroid lies on the opposite
//! boundary or gives an undecided ray. This is the Yap boundary from "Towards
//! Exact Geometric Computation,"
//! *Computational Geometry* 7.1-2 (1997): a boolean face is kept, dropped, or
//! orientation-reversed only from replayable exact evidence, never from a
//! primitive-float sample or tolerance nudge.

use hyperlimit::Point3;

use super::graph::MeshSide;
use super::mesh::ExactMesh;
use super::region::{FaceRegionTriangulation, boundary_node_point};
use super::scalar::ExactReal;
use super::winding::{
    ClosedMeshWindingRelation, PointMeshWindingReport, WindingReportError,
    classify_point_against_closed_mesh_winding_report,
};

/// Exact relation between one triangulated split cell and the opposite closed
/// mesh.
#[cfg(feature = "exact-triangulation")]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ExactVolumetricRegionRelation {
    /// The opposite mesh was not a closed two-manifold.
    NotClosed,
    /// The region representative was certified strictly inside the opposite
    /// mesh.
    Inside,
    /// The region representative was certified outside the opposite mesh.
    Outside,
    /// The representative lies on the opposite boundary.
    Boundary,
    /// Exact ray parity could not decide the representative.
    Unknown,
}

impl ExactVolumetricRegionRelation {
    /// Return whether this relation can directly drive named volumetric
    /// union/intersection/difference assembly.
    pub const fn is_strictly_decided(self) -> bool {
        matches!(self, Self::Inside | Self::Outside)
    }

    /// Return whether this relation is decided enough to drive a conservative
    /// coplanar-boundary materialization policy.
    ///
    /// Boundary classifications are not strict winding facts, but they are
    /// exact outcomes: every deterministic interior witness for the retained
    /// source cell replayed to the opposite closed mesh boundary. Following
    /// Yap, "Towards Exact Geometric Computation," *Computational Geometry*
    /// 7.1-2 (1997), the boolean pipeline may consume that state only through
    /// an explicit topology policy; it must not relabel it as inside/outside.
    pub const fn is_materialization_decided(self) -> bool {
        matches!(self, Self::Inside | Self::Outside | Self::Boundary)
    }
}

/// Retained winding evidence for one triangulated split cell.
#[cfg(feature = "exact-triangulation")]
#[derive(Clone, Debug, PartialEq)]
pub struct ExactVolumetricRegionClassification {
    /// Mesh side owning the split source face.
    pub region_side: MeshSide,
    /// Source face index on [`Self::region_side`].
    pub region_face: usize,
    /// Triangle indices into the retained [`FaceRegionTriangulation`] that
    /// produced [`Self::representative`].
    ///
    /// A single source face can be divided by several exact intersection
    /// segments. Retaining the local triangle handles makes the winding
    /// decision a per-cell certificate instead of a face-wide approximation,
    /// following Yap, "Towards Exact Geometric Computation,"
    /// *Computational Geometry* 7.1-2 (1997).
    pub triangle: [usize; 3],
    /// Exact interior representative point used for winding parity.
    ///
    /// The classifier records the first deterministic barycentric witness that
    /// gives a strict inside/outside relation, falling back to the first
    /// boundary/unknown witness only when every candidate remains non-strict.
    /// This avoids treating an unlucky centroid-on-boundary event as a
    /// semantic blocker while keeping the chosen point replayable.
    pub representative: Point3,
    /// Relation derived from [`Self::winding`].
    pub relation: ExactVolumetricRegionRelation,
    /// Exact closed-mesh ray-parity report for [`Self::representative`].
    pub winding: PointMeshWindingReport,
}

impl ExactVolumetricRegionClassification {
    /// Validate local consistency between retained representative, relation,
    /// and winding report.
    ///
    /// The representative is not a free coordinate: source replay recomputes it
    /// from a retained triangulation. This local audit checks the part that can
    /// be verified without the source mesh, namely that the relation mirrors
    /// the retained exact winding report and that the winding report is itself
    /// coherent. Yap, "Towards Exact Geometric Computation,"
    /// *Computational Geometry* 7.1-2 (1997), makes this separation important:
    /// local certificate shape and source-object replay are both explicit.
    pub fn validate(&self) -> Result<(), ExactVolumetricRegionError> {
        self.winding
            .validate()
            .map_err(ExactVolumetricRegionError::Winding)?;
        if self.relation != relation_from_winding(self.winding.relation) {
            return Err(ExactVolumetricRegionError::RelationMismatch);
        }
        Ok(())
    }

    /// Validate this classification by recomputing it from the retained
    /// triangulation cell and target mesh.
    pub fn validate_against_sources(
        &self,
        triangulation: &FaceRegionTriangulation,
        target: &ExactMesh,
    ) -> Result<(), ExactVolumetricRegionError> {
        self.validate()?;
        let replay = classify_triangulated_region_triangle_against_closed_mesh(
            triangulation,
            self.triangle,
            target,
        )?;
        if self == &replay {
            Ok(())
        } else {
            Err(ExactVolumetricRegionError::SourceReplayMismatch)
        }
    }
}

/// Validation or source-replay failure for volumetric region classifications.
#[cfg(feature = "exact-triangulation")]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ExactVolumetricRegionError {
    /// The retained triangulation did not pass its exact handoff audit.
    InvalidTriangulation,
    /// The triangulation produced no output triangle from which to choose an
    /// exact representative.
    EmptyTriangulation,
    /// The chosen triangulation triangle referenced a missing boundary node.
    InvalidTriangleIndex,
    /// A retained winding report failed its local audit.
    Winding(WindingReportError),
    /// The retained relation did not match the retained winding report.
    RelationMismatch,
    /// Recomputed representative or winding evidence did not match.
    SourceReplayMismatch,
}

/// Classify the first triangulated split cell against a closed target mesh.
///
/// This compatibility entry point classifies the first certified
/// nondegenerate triangle emitted by `hypertri` for the split region. New
/// winding-materialized booleans classify every triangle through
/// [`classify_triangulated_regions_against_opposite_meshes`]. The centroid is
/// built as rational `Real` arithmetic, then classified by
/// [`classify_point_against_closed_mesh_winding_report`]. No primitive-float
/// representative enters the decision. This follows Yap, "Towards Exact
/// Geometric Computation," *Computational Geometry* 7.1-2 (1997), by keeping
/// the proposal point and the winding decision inside exact arithmetic.
#[cfg(feature = "exact-triangulation")]
pub fn classify_triangulated_region_against_closed_mesh(
    triangulation: &FaceRegionTriangulation,
    target: &ExactMesh,
) -> Result<ExactVolumetricRegionClassification, ExactVolumetricRegionError> {
    let triangle = first_triangle(triangulation)?;
    classify_triangulated_region_triangle_against_closed_mesh(triangulation, triangle, target)
}

/// Classify one exact triangulated source-face cell against a closed mesh.
///
/// The representative search starts with the exact centroid of the supplied
/// local triangle, then tries fixed exact barycentric interior witnesses if the
/// centroid is on the opposite boundary or otherwise undecided. Per-cell
/// representatives are the necessary semantic unit once a source face has been
/// subdivided by constrained intersection segments; using one sample for the
/// entire face would make inside/outside topology depend on an arbitrary
/// triangulator order. Retrying with retained exact interior witnesses follows
/// Yap, "Towards Exact Geometric Computation," *Computational Geometry*
/// 7.1-2 (1997): unresolved predicate outcomes remain explicit, but a
/// representational accident such as "centroid is on the boundary" should not
/// force an approximate perturbation.
#[cfg(feature = "exact-triangulation")]
pub fn classify_triangulated_region_triangle_against_closed_mesh(
    triangulation: &FaceRegionTriangulation,
    triangle: [usize; 3],
    target: &ExactMesh,
) -> Result<ExactVolumetricRegionClassification, ExactVolumetricRegionError> {
    triangulation
        .validate()
        .map_err(|_| ExactVolumetricRegionError::InvalidTriangulation)?;
    if !triangulation
        .triangles
        .chunks_exact(3)
        .any(|candidate| candidate == triangle)
    {
        return Err(ExactVolumetricRegionError::InvalidTriangleIndex);
    }
    let mut fallback = None;
    for representative in representative_points(triangulation, triangle)? {
        let classification =
            classify_representative(triangulation, triangle, representative, target)?;
        if classification.relation.is_strictly_decided() {
            return Ok(classification);
        }
        if fallback.is_none() {
            fallback = Some(classification);
        }
    }
    fallback.ok_or(ExactVolumetricRegionError::EmptyTriangulation)
}

/// Classify every split-region triangle against its opposite closed mesh.
#[cfg(feature = "exact-triangulation")]
pub fn classify_triangulated_regions_against_opposite_meshes(
    triangulations: &[FaceRegionTriangulation],
    left: &ExactMesh,
    right: &ExactMesh,
) -> Result<Vec<ExactVolumetricRegionClassification>, ExactVolumetricRegionError> {
    let mut classifications = Vec::new();
    for triangulation in triangulations {
        let target = match triangulation.side {
            MeshSide::Left => right,
            MeshSide::Right => left,
        };
        for triangle in triangulation.triangles.chunks_exact(3) {
            classifications.push(classify_triangulated_region_triangle_against_closed_mesh(
                triangulation,
                [triangle[0], triangle[1], triangle[2]],
                target,
            )?);
        }
    }
    Ok(classifications)
}

#[cfg(feature = "exact-triangulation")]
fn relation_from_winding(relation: ClosedMeshWindingRelation) -> ExactVolumetricRegionRelation {
    match relation {
        ClosedMeshWindingRelation::NotClosed => ExactVolumetricRegionRelation::NotClosed,
        ClosedMeshWindingRelation::Inside => ExactVolumetricRegionRelation::Inside,
        ClosedMeshWindingRelation::Outside => ExactVolumetricRegionRelation::Outside,
        ClosedMeshWindingRelation::Boundary => ExactVolumetricRegionRelation::Boundary,
        ClosedMeshWindingRelation::Unknown => ExactVolumetricRegionRelation::Unknown,
    }
}

#[cfg(feature = "exact-triangulation")]
fn first_triangle(
    triangulation: &FaceRegionTriangulation,
) -> Result<[usize; 3], ExactVolumetricRegionError> {
    let triangle = triangulation
        .triangles
        .chunks_exact(3)
        .next()
        .ok_or(ExactVolumetricRegionError::EmptyTriangulation)?;
    Ok([triangle[0], triangle[1], triangle[2]])
}

#[cfg(feature = "exact-triangulation")]
fn representative_points(
    triangulation: &FaceRegionTriangulation,
    triangle: [usize; 3],
) -> Result<Vec<Point3>, ExactVolumetricRegionError> {
    let a = boundary_node_point(
        triangulation
            .boundary
            .get(triangle[0])
            .ok_or(ExactVolumetricRegionError::InvalidTriangleIndex)?,
    );
    let b = boundary_node_point(
        triangulation
            .boundary
            .get(triangle[1])
            .ok_or(ExactVolumetricRegionError::InvalidTriangleIndex)?,
    );
    let c = boundary_node_point(
        triangulation
            .boundary
            .get(triangle[2])
            .ok_or(ExactVolumetricRegionError::InvalidTriangleIndex)?,
    );
    Ok(vec![
        barycentric_point(a, b, c, [1, 1, 1], 3),
        barycentric_point(a, b, c, [2, 1, 1], 4),
        barycentric_point(a, b, c, [1, 2, 1], 4),
        barycentric_point(a, b, c, [1, 1, 2], 4),
    ])
}

#[cfg(feature = "exact-triangulation")]
fn classify_representative(
    triangulation: &FaceRegionTriangulation,
    triangle: [usize; 3],
    representative: Point3,
    target: &ExactMesh,
) -> Result<ExactVolumetricRegionClassification, ExactVolumetricRegionError> {
    let winding = classify_point_against_closed_mesh_winding_report(&representative, target);
    winding
        .validate_against_sources(&representative, target)
        .map_err(ExactVolumetricRegionError::Winding)?;
    Ok(ExactVolumetricRegionClassification {
        region_side: triangulation.side,
        region_face: triangulation.face,
        triangle,
        representative,
        relation: relation_from_winding(winding.relation),
        winding,
    })
}

#[cfg(feature = "exact-triangulation")]
fn barycentric_point(
    a: &Point3,
    b: &Point3,
    c: &Point3,
    weights: [i64; 3],
    denominator: i64,
) -> Point3 {
    let inv = (ExactReal::from(1) / &ExactReal::from(denominator)).expect("nonzero denominator");
    Point3::new(
        weighted_real(&a.x, &b.x, &c.x, weights, &inv),
        weighted_real(&a.y, &b.y, &c.y, weights, &inv),
        weighted_real(&a.z, &b.z, &c.z, weights, &inv),
    )
}

#[cfg(feature = "exact-triangulation")]
fn weighted_real(
    a: &ExactReal,
    b: &ExactReal,
    c: &ExactReal,
    weights: [i64; 3],
    inv_denominator: &ExactReal,
) -> ExactReal {
    (a.clone() * ExactReal::from(weights[0])
        + b.clone() * ExactReal::from(weights[1])
        + c.clone() * ExactReal::from(weights[2]))
        * inv_denominator
}
