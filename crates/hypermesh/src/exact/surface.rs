//! Exact certification for lower-dimensional surface special cases.
//!
//! This module keeps sheet/surface shortcuts separate from volumetric convex
//! shortcuts. The certified cases are intentionally narrow: single coplanar
//! triangle containment, positive-area intersection, convex union, simple
//! single-loop planar-arrangement union/difference, and the convex one-corner
//! difference shapes that can be represented as an open triangle mesh. The
//! predicates are the same projected orientation and point-in-triangle facts
//! used by the coplanar overlap classifier, following
//! Yap, "Towards Exact Geometric Computation," *Computational Geometry* 7.1-2
//! (1997): topology claims are emitted only when the combinatorial relation is
//! certified, and missing output models such as holed sheets remain explicit.
//!
//! The underlying coplanar test follows the orientation-predicate style of
//! Guigue and Devillers, "Fast and Robust Triangle-Triangle Overlap Test Using
//! Orientation Predicates," *Journal of Graphics Tools* 8.1 (2003), routed
//! through `hyperlimit` by [`crate::exact::coplanar`].

use core::cmp::Ordering;

#[cfg(feature = "exact-triangulation")]
use hyperlimit::classify_point_triangle;
use hyperlimit::{
    Point2, Point3, Sign, TriangleLocation, compare_reals, orient2d_report, point_on_segment,
};

use super::coplanar::CoplanarTriangleClassification;
use super::coplanar::{CoplanarProjection, CoplanarTriangleRelation, classify_coplanar_triangles};
use super::error::{DiagnosticKind, MeshDiagnostic, MeshError, Severity};
use super::mesh::{ExactMesh, ExactPoint3, Triangle};
#[cfg(feature = "exact-triangulation")]
use super::narrow::{TrianglePlaneRelation, classify_mesh_triangle_against_retained_face_plane};
use super::narrow::{
    TriangleTriangleClassification, TriangleTriangleRelation, classify_triangle_triangle,
};
use super::provenance::SourceProvenance;
use super::scalar::ExactReal;
use super::validation::ValidationPolicy;

/// Certified containment relation between two single-triangle coplanar sheets.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CoplanarSurfaceContainment {
    /// Every left triangle vertex lies in the closed right triangle.
    LeftInsideRight,
    /// Every right triangle vertex lies in the closed left triangle.
    RightInsideLeft,
}

/// Certification status for single-triangle coplanar containment.
#[derive(Clone, Debug, PartialEq)]
pub enum CoplanarSurfaceContainmentStatus {
    /// At least one input was not exactly one triangle.
    NotSingleTriangle,
    /// The 3D triangle/triangle classifier did not certify coplanar contact.
    NotCoplanar,
    /// The projected coplanar classifier was disjoint or undecided.
    DisjointOrUnknown,
    /// Both triangles contain each other, neither contains the other, or the
    /// case belongs to a stronger same-surface/planar-arrangement path.
    AmbiguousOrIdentical,
    /// Exactly one triangle is certified inside the other.
    Certified(CoplanarSurfaceContainment),
}

impl CoplanarSurfaceContainmentStatus {
    /// Return the certified containment relation, if one was established.
    pub const fn certified(&self) -> Option<CoplanarSurfaceContainment> {
        match self {
            Self::Certified(containment) => Some(*containment),
            _ => None,
        }
    }
}

/// Auditable single-triangle coplanar containment certificate.
#[derive(Clone, Debug, PartialEq)]
pub struct CoplanarSurfaceContainmentReport {
    /// Coarse certification status.
    pub status: CoplanarSurfaceContainmentStatus,
    /// Exact 3D triangle/triangle classification, when the input shape allows
    /// that query.
    pub triangle: Option<TriangleTriangleClassification>,
    /// Projected coplanar classification, when the 3D relation reaches the
    /// coplanar stage.
    pub coplanar: Option<CoplanarTriangleClassification>,
}

/// Validation failure for a coplanar containment report.
///
/// This checks the report metadata itself: shape rejection should not retain
/// classifiers, coplanar-stage statuses should retain both the 3D and
/// projected predicate artifacts, and certified containment must have reached
/// the projected coplanar stage. Yap, "Towards Exact Geometric Computation,"
/// *Computational Geometry* 7.1-2 (1997), treats those artifacts as the
/// auditable boundary between certified topology and unsupported policy.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CoplanarSurfaceContainmentReportError {
    /// A not-single-triangle report unexpectedly retained classifier artifacts.
    UnexpectedClassifier,
    /// A report that reached the triangle classifier did not retain it.
    MissingTriangleClassifier,
    /// A report that reached projected coplanar classification did not retain it.
    MissingCoplanarClassifier,
}

impl CoplanarSurfaceContainmentReport {
    /// Return whether every retained predicate route was proof-producing.
    pub fn all_proof_producing(&self) -> bool {
        self.triangle
            .as_ref()
            .is_none_or(TriangleTriangleClassification::all_proof_producing)
            && self
                .coplanar
                .as_ref()
                .is_none_or(CoplanarTriangleClassification::projection_proof_producing)
    }

    /// Validate status and retained classifier consistency.
    pub fn validate(&self) -> Result<(), CoplanarSurfaceContainmentReportError> {
        match self.status {
            CoplanarSurfaceContainmentStatus::NotSingleTriangle => {
                if self.triangle.is_none() && self.coplanar.is_none() {
                    Ok(())
                } else {
                    Err(CoplanarSurfaceContainmentReportError::UnexpectedClassifier)
                }
            }
            CoplanarSurfaceContainmentStatus::NotCoplanar => {
                if self.triangle.is_none() {
                    Err(CoplanarSurfaceContainmentReportError::MissingTriangleClassifier)
                } else if self.coplanar.is_some() {
                    Err(CoplanarSurfaceContainmentReportError::UnexpectedClassifier)
                } else {
                    Ok(())
                }
            }
            CoplanarSurfaceContainmentStatus::DisjointOrUnknown
            | CoplanarSurfaceContainmentStatus::AmbiguousOrIdentical
            | CoplanarSurfaceContainmentStatus::Certified(_) => {
                if self.triangle.is_none() {
                    Err(CoplanarSurfaceContainmentReportError::MissingTriangleClassifier)
                } else if self.coplanar.is_none() {
                    Err(CoplanarSurfaceContainmentReportError::MissingCoplanarClassifier)
                } else {
                    Ok(())
                }
            }
        }
    }
}

/// Exact positive-area intersection of two single-triangle coplanar sheets.
///
/// The returned mesh is an open triangle mesh representing the polygonal
/// intersection surface. Lower-dimensional contacts are intentionally reported
/// as `None` because triangle meshes cannot encode a pure point or segment
/// result without a separate output channel.
#[derive(Clone, Debug, PartialEq)]
pub struct CoplanarTriangleIntersection {
    /// Projection used by the certified 2D clipping predicates.
    pub projection: CoplanarProjection,
    /// Exact 3D polygon boundary after clipping and simplification.
    pub polygon: Vec<Point3>,
    /// Exact triangulated surface mesh for the polygon.
    pub mesh: ExactMesh,
}

impl CoplanarTriangleIntersection {
    /// Validate the materialized intersection polygon and mesh.
    ///
    /// The constructor already builds this shape through exact clipping, but
    /// the fields are public so callers can inspect, serialize, or transform
    /// the artifact. This method replays the output-side invariants before a
    /// downstream consumer trusts it as topology: polygon vertices must be
    /// exact-distinct, have certified nonzero projected area, and match the
    /// fan-triangulated [`ExactMesh`]. This follows Yap, "Towards Exact
    /// Geometric Computation," *Computational Geometry* 7.1-2 (1997): a
    /// constructed geometric object should remain auditable at API handoffs.
    pub fn validate(&self) -> Result<(), MeshError> {
        validate_coplanar_surface_output(
            self.projection,
            &self.polygon,
            &self.mesh,
            "coplanar intersection",
        )
    }
}

/// Exact convex union of two single-triangle coplanar sheets.
///
/// This is deliberately narrower than a full planar arrangement. It is emitted
/// only when the union of the two closed triangles is certified to equal the
/// convex hull of their vertices; nonconvex unions and holed/difference cases
/// remain explicit unsupported topology.
#[derive(Clone, Debug, PartialEq)]
pub struct CoplanarTriangleUnion {
    /// Projection used by the certified 2D hull and coverage predicates.
    pub projection: CoplanarProjection,
    /// Exact 3D convex hull boundary.
    pub polygon: Vec<Point3>,
    /// Exact triangulated surface mesh for the convex union.
    pub mesh: ExactMesh,
}

/// Exact simple planar-arrangement output for two coplanar triangle sheets.
///
/// This is the first general planar-arrangement fragment beyond convex hull
/// shortcuts. It keeps a single simple boundary loop plus the exact
/// `hypertri` triangulation used to materialize it. Multi-loop or holed
/// arrangements remain explicit blockers until the output model can retain
/// multiple rings. The boundary construction follows the Weiler-Atherton
/// clipping idea of traversing split polygon edges, but all split points,
/// segment membership tests, and triangulation inputs are exact facts in
/// Yap's sense; see Weiler and Atherton, "Hidden Surface Removal Using Polygon
/// Area Sorting," *SIGGRAPH Computer Graphics* 11.2 (1977), and Yap,
/// "Towards Exact Geometric Computation," *Computational Geometry* 7.1-2
/// (1997).
#[cfg(feature = "exact-triangulation")]
#[derive(Clone, Debug, PartialEq)]
pub struct CoplanarTriangleArrangement {
    /// Projection used by exact 2D arrangement predicates and triangulation.
    pub projection: CoplanarProjection,
    /// Exact 3D simple boundary loop.
    pub polygon: Vec<Point3>,
    /// Exact triangulated open surface mesh.
    pub mesh: ExactMesh,
}

#[cfg(feature = "exact-triangulation")]
impl CoplanarTriangleArrangement {
    /// Validate the materialized planar-arrangement polygon and mesh.
    pub fn validate(&self) -> Result<(), MeshError> {
        validate_coplanar_surface_output(
            self.projection,
            &self.polygon,
            &self.mesh,
            "coplanar triangle planar arrangement",
        )
    }
}

/// Exact one-hole planar-arrangement output for contained coplanar triangles.
///
/// This artifact represents the narrow `outer - inner` sheet case where one
/// coplanar triangle is certified strictly inside another. It retains both
/// rings instead of flattening them into only a triangle soup, because Yap's
/// exact-computation model requires the topological structure that justified
/// the output to remain auditable. Triangulation uses `hypertri`'s
/// earcut-compatible hole index behind `exact-triangulation`; see Held,
/// "FIST: Fast Industrial-Strength Triangulation of Polygons," *Algorithmica*
/// 30 (2001), for ear-clipping triangulation of polygons with holes, with
/// exact predicates replacing tolerance decisions here.
#[cfg(feature = "exact-triangulation")]
#[derive(Clone, Debug, PartialEq)]
pub struct CoplanarTriangleHoledArrangement {
    /// Projection used by exact 2D arrangement predicates and triangulation.
    pub projection: CoplanarProjection,
    /// Exact 3D outer boundary loop.
    pub outer: Vec<Point3>,
    /// Exact 3D inner boundary loop.
    pub hole: Vec<Point3>,
    /// Exact triangulated open surface mesh.
    pub mesh: ExactMesh,
}

/// Certified equivalence of two convex coplanar surface meshes.
///
/// This is the first multi-face coplanar surface certificate. It accepts
/// different triangulations of the same convex sheet by comparing exact
/// retained-plane coplanarity, projected convex hulls, and summed projected
/// triangle areas. It deliberately does not infer nonconvex, holed, or
/// overlapping arrangements: those require a richer cell complex. This is the
/// Yap-style object-fact boundary for a multi-triangle shortcut, with the
/// convex hull step following Andrew, "Another Efficient Algorithm for Convex
/// Hulls in Two Dimensions," *Information Processing Letters* 9.5 (1979).
#[cfg(feature = "exact-triangulation")]
#[derive(Clone, Debug, PartialEq)]
pub struct CoplanarConvexSurfaceEquivalence {
    /// Projection used for hull and area certificates.
    pub projection: CoplanarProjection,
    /// Exact shared convex hull boundary.
    pub polygon: Vec<Point3>,
    /// Twice the projected area covered by the left mesh.
    pub left_area2: ExactReal,
    /// Twice the projected area covered by the right mesh.
    pub right_area2: ExactReal,
}

/// Certified containment relation between convex coplanar surface meshes.
#[cfg(feature = "exact-triangulation")]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CoplanarConvexSurfaceContainment {
    /// Every left hull vertex lies in the closed right hull.
    LeftInsideRight,
    /// Every right hull vertex lies in the closed left hull.
    RightInsideLeft,
}

/// Certified containment of two convex coplanar surface meshes.
///
/// This certificate is the multi-face counterpart to single-triangle
/// containment. It accepts only convex sheets whose summed exact projected
/// triangle areas equal their own convex hull areas, then classifies hull
/// vertices by exact projected orientation predicates. This keeps the shortcut
/// inside Yap's exact-computation boundary: nonconvex coverage, holes, and
/// overlapping triangle soups remain explicit planar-arrangement work.
#[cfg(feature = "exact-triangulation")]
#[derive(Clone, Debug, PartialEq)]
pub struct CoplanarConvexSurfaceContainmentCertificate {
    /// Projection used for hull and area certificates.
    pub projection: CoplanarProjection,
    /// Certified containment relation.
    pub relation: CoplanarConvexSurfaceContainment,
    /// Exact left convex hull.
    pub left_hull: Vec<Point3>,
    /// Exact right convex hull.
    pub right_hull: Vec<Point3>,
    /// Twice the projected area covered by the left mesh.
    pub left_area2: ExactReal,
    /// Twice the projected area covered by the right mesh.
    pub right_area2: ExactReal,
}

/// Certification status for convex coplanar surface relations.
///
/// This is a report-level status, not a fallback classifier. It records whether
/// the convex multi-face shortcut reached a certified equivalence/containment
/// relation or failed closed before a boolean shortcut could rely on it. That
/// follows Yap, "Towards Exact Geometric Computation," *Computational
/// Geometry* 7.1-2 (1997): unsupported topology remains explicit evidence
/// instead of becoming a guessed winding decision.
#[cfg(feature = "exact-triangulation")]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CoplanarConvexSurfaceReportStatus {
    /// At least one mesh was empty, or both inputs were single-triangle sheets
    /// that belong to the narrower triangle-surface report path.
    NotMultiFaceSurface,
    /// The inputs did not certify as equivalent or strictly contained convex
    /// coplanar sheets.
    NotCertified,
    /// The meshes cover the same convex coplanar surface.
    Equivalent,
    /// One convex coplanar surface is strictly contained in the other.
    Contained(CoplanarConvexSurfaceContainment),
}

/// Validation failure for a convex coplanar surface report.
#[cfg(feature = "exact-triangulation")]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CoplanarConvexSurfaceReportError {
    /// A rejected report unexpectedly retained a certificate.
    UnexpectedCertificate,
    /// An equivalent report did not retain an equivalence certificate.
    MissingEquivalenceCertificate,
    /// A contained report did not retain a containment certificate.
    MissingContainmentCertificate,
    /// A retained equivalence certificate failed its own audit.
    InvalidEquivalenceCertificate,
    /// A retained containment certificate failed its own audit.
    InvalidContainmentCertificate,
    /// The retained containment certificate relation disagreed with status.
    ContainmentRelationMismatch,
}

/// Auditable convex coplanar surface relation report.
///
/// The report keeps the certified object-fact handoff separate from boolean
/// execution. Equivalence and containment certificates retain hulls, areas, and
/// exact projection choices, while rejected statuses retain no topology output.
#[cfg(feature = "exact-triangulation")]
#[derive(Clone, Debug, PartialEq)]
pub struct CoplanarConvexSurfaceReport {
    /// Coarse certification status.
    pub status: CoplanarConvexSurfaceReportStatus,
    /// Retained equivalence certificate when `status == Equivalent`.
    pub equivalence: Option<CoplanarConvexSurfaceEquivalence>,
    /// Retained containment certificate when `status == Contained(_)`.
    pub containment: Option<CoplanarConvexSurfaceContainmentCertificate>,
}

#[cfg(feature = "exact-triangulation")]
impl CoplanarConvexSurfaceReport {
    /// Return whether this report certified an executable convex-surface
    /// shortcut.
    pub const fn is_certified(&self) -> bool {
        matches!(
            self.status,
            CoplanarConvexSurfaceReportStatus::Equivalent
                | CoplanarConvexSurfaceReportStatus::Contained(_)
        )
    }

    /// Validate status and retained certificate consistency.
    pub fn validate(&self) -> Result<(), CoplanarConvexSurfaceReportError> {
        match self.status {
            CoplanarConvexSurfaceReportStatus::NotMultiFaceSurface
            | CoplanarConvexSurfaceReportStatus::NotCertified => {
                if self.equivalence.is_none() && self.containment.is_none() {
                    Ok(())
                } else {
                    Err(CoplanarConvexSurfaceReportError::UnexpectedCertificate)
                }
            }
            CoplanarConvexSurfaceReportStatus::Equivalent => {
                if self.containment.is_some() {
                    return Err(CoplanarConvexSurfaceReportError::UnexpectedCertificate);
                }
                let certificate = self
                    .equivalence
                    .as_ref()
                    .ok_or(CoplanarConvexSurfaceReportError::MissingEquivalenceCertificate)?;
                certificate
                    .validate()
                    .map_err(|_| CoplanarConvexSurfaceReportError::InvalidEquivalenceCertificate)
            }
            CoplanarConvexSurfaceReportStatus::Contained(relation) => {
                if self.equivalence.is_some() {
                    return Err(CoplanarConvexSurfaceReportError::UnexpectedCertificate);
                }
                let certificate = self
                    .containment
                    .as_ref()
                    .ok_or(CoplanarConvexSurfaceReportError::MissingContainmentCertificate)?;
                if certificate.relation != relation {
                    return Err(CoplanarConvexSurfaceReportError::ContainmentRelationMismatch);
                }
                certificate
                    .validate()
                    .map_err(|_| CoplanarConvexSurfaceReportError::InvalidContainmentCertificate)
            }
        }
    }
}

#[cfg(feature = "exact-triangulation")]
impl CoplanarConvexSurfaceContainmentCertificate {
    /// Validate hull area and strict containment area ordering.
    pub fn validate(&self) -> Result<(), MeshError> {
        let left_hull_area =
            projected_area2_abs(&self.left_hull, self.projection).ok_or_else(|| {
                surface_validation_error(
                    "coplanar convex surface containment",
                    "left hull projected area was undecided",
                )
            })?;
        let right_hull_area =
            projected_area2_abs(&self.right_hull, self.projection).ok_or_else(|| {
                surface_validation_error(
                    "coplanar convex surface containment",
                    "right hull projected area was undecided",
                )
            })?;
        if compare_reals(&self.left_area2, &left_hull_area).value() != Some(Ordering::Equal)
            || compare_reals(&self.right_area2, &right_hull_area).value() != Some(Ordering::Equal)
        {
            return Err(surface_validation_error(
                "coplanar convex surface containment",
                "mesh area does not equal retained hull area",
            ));
        }
        let ordering = compare_reals(&left_hull_area, &right_hull_area).value();
        let valid = matches!(
            (self.relation, ordering),
            (
                CoplanarConvexSurfaceContainment::LeftInsideRight,
                Some(Ordering::Less)
            ) | (
                CoplanarConvexSurfaceContainment::RightInsideLeft,
                Some(Ordering::Greater)
            )
        );
        if valid {
            Ok(())
        } else {
            Err(surface_validation_error(
                "coplanar convex surface containment",
                "containment relation does not match strict hull area ordering",
            ))
        }
    }
}

/// Exact one-hole arrangement output for convex coplanar surface containment.
///
/// The rings are retained as exact 3D points and validated separately from the
/// triangulation so the output remains auditable in the sense of Yap, "Towards
/// Exact Geometric Computation," *Computational Geometry* 7.1-2 (1997). The
/// outer/inner rings are convex hulls certified by the monotone-chain hull of
/// Andrew, "Another Efficient Algorithm for Convex Hulls in Two Dimensions,"
/// *Information Processing Letters* 9.5 (1979).
#[cfg(feature = "exact-triangulation")]
#[derive(Clone, Debug, PartialEq)]
pub struct CoplanarConvexHoledArrangement {
    /// Projection used by exact 2D arrangement predicates and triangulation.
    pub projection: CoplanarProjection,
    /// Exact 3D outer convex hull.
    pub outer: Vec<Point3>,
    /// Exact 3D inner convex hull.
    pub hole: Vec<Point3>,
    /// Exact triangulated open surface mesh.
    pub mesh: ExactMesh,
}

#[cfg(feature = "exact-triangulation")]
impl CoplanarConvexHoledArrangement {
    /// Validate ring shape, exact projected area, and retained mesh state.
    pub fn validate(&self) -> Result<(), MeshError> {
        validate_holed_surface_output(
            self.projection,
            &self.outer,
            &self.hole,
            &self.mesh,
            "coplanar convex holed arrangement",
        )
    }
}

/// Exact simple-loop arrangement output for convex coplanar surface booleans.
///
/// This is the multi-face counterpart to [`CoplanarTriangleArrangement`]. It
/// accepts only convex input sheets whose intersection, union, or difference
/// boundary stitches into one exact simple loop. The construction follows the
/// same boundary-fragment traversal idea as Weiler-Atherton clipping for union
/// and difference, while the convex hull and input area certificates come from
/// the retained exact object facts described in
/// [`CoplanarConvexSurfaceEquivalence`]. Following Yap, "Towards Exact
/// Geometric Computation," *Computational Geometry* 7.1-2 (1997), the output
/// keeps the exact boundary loop and triangulated mesh as auditable state
/// instead of hiding the planar arrangement behind a tolerance boolean.
#[cfg(feature = "exact-triangulation")]
#[derive(Clone, Debug, PartialEq)]
pub struct CoplanarConvexArrangement {
    /// Projection used by exact 2D arrangement predicates and triangulation.
    pub projection: CoplanarProjection,
    /// Exact 3D simple boundary loop.
    pub polygon: Vec<Point3>,
    /// Exact triangulated open surface mesh.
    pub mesh: ExactMesh,
}

#[cfg(feature = "exact-triangulation")]
impl CoplanarConvexArrangement {
    /// Validate the materialized convex-surface arrangement polygon and mesh.
    pub fn validate(&self) -> Result<(), MeshError> {
        validate_coplanar_surface_output(
            self.projection,
            &self.polygon,
            &self.mesh,
            "coplanar convex surface arrangement",
        )
    }
}

#[cfg(feature = "exact-triangulation")]
impl CoplanarConvexSurfaceEquivalence {
    /// Validate the retained equivalence certificate.
    pub fn validate(&self) -> Result<(), MeshError> {
        let hull_area = projected_area2_abs(&self.polygon, self.projection).ok_or_else(|| {
            surface_validation_error(
                "coplanar convex surface equivalence",
                "shared hull projected area was undecided",
            )
        })?;
        if compare_reals(&hull_area, &ExactReal::from(0)).value() != Some(Ordering::Greater) {
            return Err(surface_validation_error(
                "coplanar convex surface equivalence",
                "shared hull has zero projected area",
            ));
        }
        if compare_reals(&self.left_area2, &hull_area).value() != Some(Ordering::Equal)
            || compare_reals(&self.right_area2, &hull_area).value() != Some(Ordering::Equal)
        {
            return Err(surface_validation_error(
                "coplanar convex surface equivalence",
                "mesh area does not equal shared hull area",
            ));
        }
        Ok(())
    }
}

#[cfg(feature = "exact-triangulation")]
impl CoplanarTriangleHoledArrangement {
    /// Validate ring shape, exact projected area, and retained mesh state.
    pub fn validate(&self) -> Result<(), MeshError> {
        validate_holed_surface_output(
            self.projection,
            &self.outer,
            &self.hole,
            &self.mesh,
            "coplanar triangle holed arrangement",
        )
    }
}

impl CoplanarTriangleUnion {
    /// Validate the materialized convex-union polygon and mesh.
    ///
    /// The union shortcut is accepted only after exact hull coverage checks,
    /// following Andrew's monotone-chain hull construction and Yap's exact
    /// computation boundary. This method validates the persisted output
    /// artifact itself: exact point distinctness, positive projected area, and
    /// fan mesh consistency.
    pub fn validate(&self) -> Result<(), MeshError> {
        validate_coplanar_surface_output(
            self.projection,
            &self.polygon,
            &self.mesh,
            "coplanar convex union",
        )
    }
}

/// Exact convex difference of two single-triangle coplanar sheets.
///
/// This is emitted only for a strict one-corner cut from the left triangle,
/// where the result is one convex polygon representable as a fan-triangulated
/// open triangle mesh. Cuts that split the surface, create holes, or require a
/// nonconvex boundary remain explicit planar-arrangement work.
#[derive(Clone, Debug, PartialEq)]
pub struct CoplanarTriangleDifference {
    /// Projection used by the certified 2D predicates.
    pub projection: CoplanarProjection,
    /// Exact 3D boundary of `left - right`.
    pub polygon: Vec<Point3>,
    /// Exact triangulated surface mesh for the difference.
    pub mesh: ExactMesh,
}

impl CoplanarTriangleDifference {
    /// Validate the materialized one-corner difference polygon and mesh.
    ///
    /// One-corner difference is a narrowly certified planar-arrangement
    /// fragment: the accepted polygon is justified by exact area conservation.
    /// This output validation keeps that fragment auditable after construction
    /// by checking projected area, exact point distinctness, and mesh fan
    /// consistency before callers reuse the artifact.
    pub fn validate(&self) -> Result<(), MeshError> {
        validate_coplanar_surface_output(
            self.projection,
            &self.polygon,
            &self.mesh,
            "coplanar one-corner difference",
        )
    }
}

/// Certify containment for two single-triangle coplanar sheets.
///
/// This is not a general planar arrangement solver. It only returns a
/// certificate when both meshes contain one triangle, the triangles are
/// certified coplanar, and all vertices of exactly one triangle are certified
/// inside or on the boundary of the other closed triangle. Identical surfaces
/// are left to the stronger same-surface certificate in the boolean layer.
pub fn certify_single_triangle_coplanar_containment(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Option<CoplanarSurfaceContainment> {
    certify_single_triangle_coplanar_containment_report(left, right)
        .status
        .certified()
}

/// Certify single-triangle coplanar containment and retain predicate artifacts.
///
/// This report is the auditable form of
/// [`certify_single_triangle_coplanar_containment`]. It keeps the 3D
/// `hyperlimit::orient3d_report`-backed triangle classifier and the projected
/// coplanar classifier beside the collapsed containment status. That matches
/// Yap, "Towards Exact Geometric Computation," *Computational Geometry* 7.1-2
/// (1997): a topology shortcut should expose the certified predicate facts
/// that justified it, and unsupported or ambiguous cases stay explicit.
pub fn certify_single_triangle_coplanar_containment_report(
    left: &ExactMesh,
    right: &ExactMesh,
) -> CoplanarSurfaceContainmentReport {
    if left.triangles().len() != 1 || right.triangles().len() != 1 {
        return CoplanarSurfaceContainmentReport {
            status: CoplanarSurfaceContainmentStatus::NotSingleTriangle,
            triangle: None,
            coplanar: None,
        };
    }

    let points = left
        .vertices()
        .iter()
        .chain(right.vertices())
        .map(|point| point.to_hyperlimit_point())
        .collect::<Vec<_>>();
    let left_tri = left.triangles()[0].0;
    let right_offset = left.vertices().len();
    let right_tri = right.triangles()[0].0.map(|index| index + right_offset);

    let classification = classify_triangle_triangle(&points, left_tri, right_tri);
    if !matches!(
        classification.relation,
        TriangleTriangleRelation::CoplanarTouching | TriangleTriangleRelation::CoplanarOverlapping
    ) {
        return CoplanarSurfaceContainmentReport {
            status: CoplanarSurfaceContainmentStatus::NotCoplanar,
            triangle: Some(classification),
            coplanar: None,
        };
    }

    let coplanar = classify_coplanar_triangles(&points, left_tri, right_tri);
    if coplanar.relation == CoplanarTriangleRelation::Unknown
        || coplanar.relation == CoplanarTriangleRelation::Disjoint
    {
        return CoplanarSurfaceContainmentReport {
            status: CoplanarSurfaceContainmentStatus::DisjointOrUnknown,
            triangle: Some(classification),
            coplanar: Some(coplanar),
        };
    }

    let left_inside_right = all_in_closed_triangle(&coplanar.left_vertices_in_right);
    let right_inside_left = all_in_closed_triangle(&coplanar.right_vertices_in_left);
    let status = match (left_inside_right, right_inside_left) {
        (true, false) => {
            CoplanarSurfaceContainmentStatus::Certified(CoplanarSurfaceContainment::LeftInsideRight)
        }
        (false, true) => {
            CoplanarSurfaceContainmentStatus::Certified(CoplanarSurfaceContainment::RightInsideLeft)
        }
        _ => CoplanarSurfaceContainmentStatus::AmbiguousOrIdentical,
    };
    CoplanarSurfaceContainmentReport {
        status,
        triangle: Some(classification),
        coplanar: Some(coplanar),
    }
}

/// Certify and materialize the positive-area intersection of two coplanar
/// single-triangle sheets.
///
/// This is the smallest exact replacement for a legacy partial-overlap case:
/// Sutherland-Hodgman style half-plane clipping is performed with
/// `hyperlimit::orient2d_report`, and edge/clip-line crossings are constructed
/// as exact `Real` ratios. The algorithmic shape follows Sutherland and
/// Hodgman, "Reentrant Polygon Clipping," *Communications of the ACM* 17.1
/// (1974), but every combinatorial decision is certified as required by Yap,
/// "Towards Exact Geometric Computation," *Computational Geometry* 7.1-2
/// (1997).
pub fn intersect_single_triangle_coplanar_surfaces(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Option<CoplanarTriangleIntersection> {
    if left.triangles().len() != 1 || right.triangles().len() != 1 {
        return None;
    }

    let points = combined_points(left, right);
    let left_tri = left.triangles()[0].0;
    let right_offset = left.vertices().len();
    let right_tri = right.triangles()[0].0.map(|index| index + right_offset);

    let classification = classify_triangle_triangle(&points, left_tri, right_tri);
    if classification.relation != TriangleTriangleRelation::CoplanarOverlapping {
        return None;
    }

    let coplanar = classify_coplanar_triangles(&points, left_tri, right_tri);
    if coplanar.relation != CoplanarTriangleRelation::Overlapping {
        return None;
    }
    let projection = coplanar.projection?;

    let left_polygon = left_tri
        .iter()
        .map(|&index| points[index].clone())
        .collect::<Vec<_>>();
    let clip_polygon = right_tri
        .iter()
        .map(|&index| points[index].clone())
        .collect::<Vec<_>>();
    let clipped = clip_convex_polygon(&left_polygon, &clip_polygon, projection)?;
    let polygon = simplify_projected_polygon(clipped, projection);
    if polygon.len() < 3 {
        return None;
    }

    let mesh = polygon_to_open_mesh(&polygon)?;
    let intersection = CoplanarTriangleIntersection {
        projection,
        polygon,
        mesh,
    };
    intersection.validate().ok()?;
    Some(intersection)
}

/// Certify and materialize a convex union of two coplanar single-triangle
/// sheets.
///
/// The candidate output is the exact convex hull of all triangle vertices.
/// Hypermesh certifies that this hull is not overclaiming the union by clipping
/// each fan triangle against both inputs and checking exact area coverage:
/// `area(left clip) + area(right clip) - area(overlap clip) == area(fan)`.
/// This preserves Yap's distinction between a constructed object and the
/// certified predicates that justify its topology. The convex-hull
/// construction is the standard monotone chain algorithm from Andrew, "Another
/// Efficient Algorithm for Convex Hulls in Two Dimensions," *Information
/// Processing Letters* 9.5 (1979), with exact comparisons and orientations.
pub fn union_single_triangle_coplanar_surfaces(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Option<CoplanarTriangleUnion> {
    if left.triangles().len() != 1 || right.triangles().len() != 1 {
        return None;
    }

    let points = combined_points(left, right);
    let left_tri = left.triangles()[0].0;
    let right_offset = left.vertices().len();
    let right_tri = right.triangles()[0].0.map(|index| index + right_offset);
    let classification = classify_triangle_triangle(&points, left_tri, right_tri);
    if !matches!(
        classification.relation,
        TriangleTriangleRelation::CoplanarTouching | TriangleTriangleRelation::CoplanarOverlapping
    ) {
        return None;
    }
    let coplanar = classify_coplanar_triangles(&points, left_tri, right_tri);
    if !matches!(
        coplanar.relation,
        CoplanarTriangleRelation::Touching | CoplanarTriangleRelation::Overlapping
    ) {
        return None;
    }
    let projection = coplanar.projection?;

    let hull = convex_hull_3d(points.clone(), projection)?;
    if hull.len() < 3
        || !fan_triangles_covered_by_inputs(&hull, &points, left_tri, right_tri, projection)?
    {
        return None;
    }
    let mesh = polygon_to_open_mesh_with_label(&hull, "exact convex coplanar triangle union")?;
    let union = CoplanarTriangleUnion {
        projection,
        polygon: hull,
        mesh,
    };
    union.validate().ok()?;
    Some(union)
}

/// Certify and materialize a simple planar-arrangement union of two coplanar
/// single-triangle sheets.
///
/// This path handles the nonconvex single-loop cases that the convex-hull
/// shortcut must reject. It splits both triangle boundaries at exact
/// intersections, keeps only edge fragments whose midpoints lie outside the
/// opposite closed triangle, stitches one boundary loop, then triangulates
/// that loop with feature-gated exact `hypertri` earcut. If the arrangement
/// has multiple loops, undecided predicates, or lower-dimensional-only
/// contact, the function returns `None` rather than weakening the topology
/// decision.
#[cfg(feature = "exact-triangulation")]
pub fn arrange_single_triangle_coplanar_union(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Option<CoplanarTriangleArrangement> {
    arrange_single_triangle_coplanar_surfaces(left, right, ArrangementOperation::Union)
}

/// Certify and materialize a simple planar-arrangement difference
/// `left - right` for two coplanar single-triangle sheets.
///
/// The accepted output is one simple boundary loop. Cases that split the left
/// triangle into multiple components or create a hole remain explicit
/// planar-arrangement blockers, because an open triangle mesh without retained
/// ring provenance would hide the topological structure Yap requires callers
/// to audit.
#[cfg(feature = "exact-triangulation")]
pub fn arrange_single_triangle_coplanar_difference(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Option<CoplanarTriangleArrangement> {
    arrange_single_triangle_coplanar_surfaces(left, right, ArrangementOperation::Difference)
}

/// Certify and materialize the contained-triangle holed difference
/// `outer - inner`.
///
/// This is the one-hole planar-arrangement counterpart to
/// [`arrange_single_triangle_coplanar_difference`]. It is accepted only when
/// the right triangle is certified inside the left triangle by projected
/// `hyperlimit` point-in-triangle facts; otherwise multi-component and
/// ambiguous cases remain explicit blockers.
#[cfg(feature = "exact-triangulation")]
pub fn arrange_single_triangle_coplanar_holed_difference(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Option<CoplanarTriangleHoledArrangement> {
    if certify_single_triangle_coplanar_containment(left, right)?
        != CoplanarSurfaceContainment::RightInsideLeft
    {
        return None;
    }
    let classification = classify_triangle_triangle(
        &combined_points(left, right),
        left.triangles()[0].0,
        right.triangles()[0]
            .0
            .map(|index| index + left.vertices().len()),
    );
    if classification.relation != TriangleTriangleRelation::CoplanarOverlapping {
        return None;
    }
    let coplanar = classify_coplanar_triangles(
        &combined_points(left, right),
        left.triangles()[0].0,
        right.triangles()[0]
            .0
            .map(|index| index + left.vertices().len()),
    );
    let projection = coplanar.projection?;
    let mut outer = left.triangles()[0]
        .0
        .iter()
        .map(|&index| left.vertices()[index].to_hyperlimit_point())
        .collect::<Vec<_>>();
    let mut hole = right.triangles()[0]
        .0
        .iter()
        .map(|&index| right.vertices()[index].to_hyperlimit_point())
        .collect::<Vec<_>>();
    orient_polygon_ccw(&mut outer, projection)?;
    orient_polygon_cw(&mut hole, projection)?;
    let mesh = polygon_to_earcut_open_mesh_with_hole(&outer, &hole, projection)?;
    let arrangement = CoplanarTriangleHoledArrangement {
        projection,
        outer,
        hole,
        mesh,
    };
    arrangement.validate().ok()?;
    Some(arrangement)
}

/// Certify that two coplanar open meshes cover the same convex surface.
///
/// This shortcut covers multi-face sheets such as a square split along
/// opposite diagonals. The certification is intentionally stronger than
/// "same plane and overlapping bounds": every face in both meshes must lie on
/// the first left face's retained exact plane, both projected convex hulls
/// must compare equal vertex-for-vertex, and the sum of projected triangle
/// areas for each mesh must equal the shared hull area. Nonconvex, holed, or
/// overlapping triangle soups fail closed and remain planar-arrangement work.
#[cfg(feature = "exact-triangulation")]
pub fn certify_coplanar_convex_surface_equivalence(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Option<CoplanarConvexSurfaceEquivalence> {
    let (projection, left_hull, right_hull, left_area, right_area) =
        convex_surface_hulls_and_areas(left, right)?;
    if !polygons_equal(&left_hull, &right_hull) {
        return None;
    }
    let hull_area = projected_area2_abs(&left_hull, projection)?;
    if compare_reals(&left_area, &hull_area).value() != Some(Ordering::Equal)
        || compare_reals(&right_area, &hull_area).value() != Some(Ordering::Equal)
    {
        return None;
    }
    let certificate = CoplanarConvexSurfaceEquivalence {
        projection,
        polygon: left_hull,
        left_area2: left_area,
        right_area2: right_area,
    };
    certificate.validate().ok()?;
    Some(certificate)
}

/// Report the certified convex coplanar surface relation, if one exists.
///
/// This function is the auditable front door for the multi-face convex surface
/// shortcuts used by named booleans. It first rejects empty/single-triangle
/// cases that belong to narrower APIs, then attempts equivalence before strict
/// containment so equal hulls do not get reported as two-way containment.
#[cfg(feature = "exact-triangulation")]
pub fn certify_coplanar_convex_surface_report(
    left: &ExactMesh,
    right: &ExactMesh,
) -> CoplanarConvexSurfaceReport {
    if left.triangles().is_empty()
        || right.triangles().is_empty()
        || (left.triangles().len() == 1 && right.triangles().len() == 1)
    {
        return CoplanarConvexSurfaceReport {
            status: CoplanarConvexSurfaceReportStatus::NotMultiFaceSurface,
            equivalence: None,
            containment: None,
        };
    }
    if let Some(equivalence) = certify_coplanar_convex_surface_equivalence(left, right) {
        return CoplanarConvexSurfaceReport {
            status: CoplanarConvexSurfaceReportStatus::Equivalent,
            equivalence: Some(equivalence),
            containment: None,
        };
    }
    if let Some(containment) = certify_coplanar_convex_surface_containment(left, right) {
        return CoplanarConvexSurfaceReport {
            status: CoplanarConvexSurfaceReportStatus::Contained(containment.relation),
            equivalence: None,
            containment: Some(containment),
        };
    }
    CoplanarConvexSurfaceReport {
        status: CoplanarConvexSurfaceReportStatus::NotCertified,
        equivalence: None,
        containment: None,
    }
}

/// Certify strict containment between two convex coplanar surface meshes.
///
/// The certificate is accepted only after both inputs prove convex coverage by
/// exact area equality with their projected hulls. Hull containment is checked
/// by exact orientation signs on every candidate inner hull vertex. Equal hulls
/// are left to [`certify_coplanar_convex_surface_equivalence`].
#[cfg(feature = "exact-triangulation")]
pub fn certify_coplanar_convex_surface_containment(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Option<CoplanarConvexSurfaceContainmentCertificate> {
    let (projection, left_hull, right_hull, left_area, right_area) =
        convex_surface_hulls_and_areas(left, right)?;
    if polygons_equal(&left_hull, &right_hull) {
        return None;
    }
    let relation = if polygon_in_closed_convex_polygon(&left_hull, &right_hull, projection)? {
        CoplanarConvexSurfaceContainment::LeftInsideRight
    } else if polygon_in_closed_convex_polygon(&right_hull, &left_hull, projection)? {
        CoplanarConvexSurfaceContainment::RightInsideLeft
    } else {
        return None;
    };
    let certificate = CoplanarConvexSurfaceContainmentCertificate {
        projection,
        relation,
        left_hull,
        right_hull,
        left_area2: left_area,
        right_area2: right_area,
    };
    certificate.validate().ok()?;
    Some(certificate)
}

/// Materialize the convex coplanar containment difference `outer - inner`.
///
/// This is the multi-face counterpart to
/// [`arrange_single_triangle_coplanar_holed_difference`]. The retained output
/// is one exact outer hull and one exact hole hull, triangulated through
/// feature-gated `hypertri` earcut. Multi-hole or nonconvex differences still
/// fail closed.
#[cfg(feature = "exact-triangulation")]
pub fn arrange_coplanar_convex_surface_holed_difference(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Option<CoplanarConvexHoledArrangement> {
    let certificate = certify_coplanar_convex_surface_containment(left, right)?;
    if certificate.relation != CoplanarConvexSurfaceContainment::RightInsideLeft {
        return None;
    }
    let mut outer = certificate.left_hull;
    let mut hole = certificate.right_hull;
    orient_polygon_ccw(&mut outer, certificate.projection)?;
    orient_polygon_cw(&mut hole, certificate.projection)?;
    let mesh = polygon_to_earcut_open_mesh_with_hole(&outer, &hole, certificate.projection)?;
    let arrangement = CoplanarConvexHoledArrangement {
        projection: certificate.projection,
        outer,
        hole,
        mesh,
    };
    arrangement.validate().ok()?;
    Some(arrangement)
}

/// Certify and materialize the positive-area intersection of convex coplanar surfaces.
///
/// This is the multi-face counterpart to
/// [`intersect_single_triangle_coplanar_surfaces`]. Both inputs must certify as
/// exact convex sheet covers, then the output boundary is the convex polygon
/// induced by retained vertices and exact edge intersections. The retained
/// polygon is accepted only when its triangulated mesh validates, keeping the
/// construction aligned with Yap's certified-object contract. The convex
/// clipping boundary follows the Sutherland-Hodgman half-plane clipping model,
/// but replaces tolerance tests with exact `hyperlimit` predicates; see
/// Sutherland and Hodgman, "Reentrant Polygon Clipping," *Communications of the
/// ACM* 17.1 (1974).
#[cfg(feature = "exact-triangulation")]
pub fn arrange_coplanar_convex_surface_intersection(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Option<CoplanarConvexArrangement> {
    if left.triangles().len() == 1 && right.triangles().len() == 1 {
        return None;
    }
    let (projection, mut left_hull, mut right_hull, _, _) =
        convex_surface_hulls_and_areas(left, right)?;
    if polygons_equal(&left_hull, &right_hull)
        || polygon_in_closed_convex_polygon(&left_hull, &right_hull, projection)?
        || polygon_in_closed_convex_polygon(&right_hull, &left_hull, projection)?
    {
        return None;
    }
    orient_polygon_ccw(&mut left_hull, projection)?;
    orient_polygon_ccw(&mut right_hull, projection)?;

    let polygon = convex_polygon_intersection_boundary(&left_hull, &right_hull, projection)?;
    if polygon.len() < 3 {
        return None;
    }
    let mesh = polygon_to_earcut_open_mesh(&polygon, projection)?;
    let arrangement = CoplanarConvexArrangement {
        projection,
        polygon,
        mesh,
    };
    arrangement.validate().ok()?;
    Some(arrangement)
}

/// Certify and materialize a simple-loop union of convex coplanar surfaces.
///
/// This is a bounded planar-arrangement port for multi-face sheets. Both
/// inputs must first certify as convex coplanar surface covers by exact hull
/// and area facts. The boundary is then formed from exact edge fragments whose
/// midpoint lies outside the opposite convex hull, stitched into one loop, and
/// accepted only when fan-triangle area coverage proves the loop equals the
/// union. The traversal follows the Weiler-Atherton boundary-fragment idea,
/// with exact `hyperlimit` orientation predicates providing Yap-style
/// certified combinatorial decisions.
#[cfg(feature = "exact-triangulation")]
pub fn arrange_coplanar_convex_surface_union(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Option<CoplanarConvexArrangement> {
    let (projection, mut left_hull, mut right_hull, _, _) =
        convex_surface_hulls_and_areas(left, right)?;
    if polygons_equal(&left_hull, &right_hull)
        || polygon_in_closed_convex_polygon(&left_hull, &right_hull, projection)?
        || polygon_in_closed_convex_polygon(&right_hull, &left_hull, projection)?
    {
        return None;
    }
    orient_polygon_ccw(&mut left_hull, projection)?;
    orient_polygon_ccw(&mut right_hull, projection)?;

    let mut fragments = Vec::new();
    collect_convex_union_boundary_fragments(&left_hull, &right_hull, projection, &mut fragments)?;
    collect_convex_union_boundary_fragments(&right_hull, &left_hull, projection, &mut fragments)?;
    let polygon = stitch_simple_loop(fragments, projection)?;
    if !convex_union_boundary_area_matches_inputs(&polygon, &left_hull, &right_hull, projection)? {
        return None;
    }
    let mesh = polygon_to_earcut_open_mesh(&polygon, projection)?;
    let arrangement = CoplanarConvexArrangement {
        projection,
        polygon,
        mesh,
    };
    arrangement.validate().ok()?;
    Some(arrangement)
}

/// Certify and materialize a simple-loop difference of convex coplanar surfaces.
///
/// This handles the multi-face convex cases where `left - right` is a single
/// simple boundary loop rather than a hole or multiple components. The
/// retained loop is stitched from exact left-boundary fragments outside the
/// right hull and reversed right-boundary fragments inside the left hull. The
/// result is accepted only when exact area proves
/// `area(output) + area(left ∩ right) == area(left)`, keeping the construction
/// inside Yap's certified object-state boundary.
#[cfg(feature = "exact-triangulation")]
pub fn arrange_coplanar_convex_surface_difference(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Option<CoplanarConvexArrangement> {
    let (projection, mut left_hull, mut right_hull, _, _) =
        convex_surface_hulls_and_areas(left, right)?;
    if polygons_equal(&left_hull, &right_hull)
        || polygon_in_closed_convex_polygon(&left_hull, &right_hull, projection)?
        || polygon_in_closed_convex_polygon(&right_hull, &left_hull, projection)?
    {
        return None;
    }
    orient_polygon_ccw(&mut left_hull, projection)?;
    orient_polygon_ccw(&mut right_hull, projection)?;

    let mut fragments = Vec::new();
    collect_convex_difference_boundary_fragments(
        &left_hull,
        &right_hull,
        projection,
        true,
        &mut fragments,
    )?;
    collect_convex_difference_boundary_fragments(
        &right_hull,
        &left_hull,
        projection,
        false,
        &mut fragments,
    )?;
    let polygon = stitch_simple_loop(fragments, projection)?;
    if !convex_difference_boundary_area_matches_inputs(
        &polygon,
        &left_hull,
        &right_hull,
        projection,
    )? {
        return None;
    }
    let mesh = polygon_to_earcut_open_mesh(&polygon, projection)?;
    let arrangement = CoplanarConvexArrangement {
        projection,
        polygon,
        mesh,
    };
    arrangement.validate().ok()?;
    Some(arrangement)
}

/// Certify and materialize a one-corner coplanar triangle difference.
///
/// This is a small planar-arrangement output case rather than a winding
/// shortcut. Hypermesh currently accepts the two convex one-corner shapes:
/// one strict left corner removed by the right triangle, or one strict left
/// corner remaining outside the right triangle. Both variants reuse the exact
/// clipped intersection polygon to find replacement vertices on the adjacent
/// left edges. The candidate output is accepted only when exact projected area
/// proves `area(output) + area(intersection) == area(left)`, following Yap's
/// requirement that constructed topology be justified by certified facts.
pub fn difference_single_triangle_coplanar_surfaces(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Option<CoplanarTriangleDifference> {
    if left.triangles().len() != 1 || right.triangles().len() != 1 {
        return None;
    }

    let points = combined_points(left, right);
    let left_tri = left.triangles()[0].0;
    let right_offset = left.vertices().len();
    let right_tri = right.triangles()[0].0.map(|index| index + right_offset);
    let classification = classify_triangle_triangle(&points, left_tri, right_tri);
    if classification.relation != TriangleTriangleRelation::CoplanarOverlapping {
        return None;
    }
    let coplanar = classify_coplanar_triangles(&points, left_tri, right_tri);
    if coplanar.relation != CoplanarTriangleRelation::Overlapping {
        return None;
    }
    let projection = coplanar.projection?;
    let intersection = intersect_single_triangle_coplanar_surfaces(left, right)?;

    let left_points = left_tri
        .iter()
        .map(|&index| points[index].clone())
        .collect::<Vec<_>>();
    let polygon = if let Some(inside_index) =
        one_strict_left_vertex_inside_right(&coplanar.left_vertices_in_right)
    {
        difference_one_corner_removed(&left_points, inside_index, &intersection, projection)?
    } else if let Some(outside_index) =
        one_strict_left_vertex_outside_right(&coplanar.left_vertices_in_right)
    {
        difference_one_corner_remaining(&left_points, outside_index, &intersection, projection)?
    } else {
        return None;
    };

    let mesh =
        polygon_to_open_mesh_with_label(&polygon, "exact one-corner coplanar triangle difference")?;
    let difference = CoplanarTriangleDifference {
        projection,
        polygon,
        mesh,
    };
    difference.validate().ok()?;
    Some(difference)
}

fn difference_one_corner_removed(
    left_points: &[Point3],
    inside_index: usize,
    intersection: &CoplanarTriangleIntersection,
    projection: CoplanarProjection,
) -> Option<Vec<Point3>> {
    if intersection.polygon.len() != 3 {
        return None;
    }
    let inside = &left_points[inside_index];
    let next_index = (inside_index + 1) % 3;
    let prev_index = (inside_index + 2) % 3;
    let next = &left_points[next_index];
    let prev = &left_points[prev_index];
    let cut_next = intersection
        .polygon
        .iter()
        .find(|point| {
            !points_equal(point, inside)
                && point_on_projected_segment(inside, next, point, projection)
        })
        .cloned()?;
    let cut_prev = intersection
        .polygon
        .iter()
        .find(|point| {
            !points_equal(point, inside)
                && point_on_projected_segment(prev, inside, point, projection)
        })
        .cloned()?;

    let polygon = simplify_projected_polygon(
        vec![cut_next, next.clone(), prev.clone(), cut_prev],
        projection,
    );
    certify_difference_area(left_points, &intersection.polygon, polygon, projection)
}

fn difference_one_corner_remaining(
    left_points: &[Point3],
    outside_index: usize,
    intersection: &CoplanarTriangleIntersection,
    projection: CoplanarProjection,
) -> Option<Vec<Point3>> {
    let outside = &left_points[outside_index];
    let next_index = (outside_index + 1) % 3;
    let prev_index = (outside_index + 2) % 3;
    let next = &left_points[next_index];
    let prev = &left_points[prev_index];
    let cut_next = intersection
        .polygon
        .iter()
        .find(|point| point_on_projected_segment(outside, next, point, projection))
        .cloned()?;
    let cut_prev = intersection
        .polygon
        .iter()
        .find(|point| point_on_projected_segment(prev, outside, point, projection))
        .cloned()?;

    let polygon = simplify_projected_polygon(vec![outside.clone(), cut_next, cut_prev], projection);
    certify_difference_area(left_points, &intersection.polygon, polygon, projection)
}

fn certify_difference_area(
    left_points: &[Point3],
    intersection: &[Point3],
    polygon: Vec<Point3>,
    projection: CoplanarProjection,
) -> Option<Vec<Point3>> {
    if polygon.len() < 3 {
        return None;
    }
    let left_area = projected_area2_abs(left_points, projection)?;
    let intersection_area = projected_area2_abs(intersection, projection)?;
    let output_area = projected_area2_abs(&polygon, projection)?;
    if compare_reals(&add(&output_area, &intersection_area), &left_area).value()
        == Some(Ordering::Equal)
    {
        Some(polygon)
    } else {
        None
    }
}

fn one_strict_left_vertex_inside_right(locations: &[Option<TriangleLocation>; 3]) -> Option<usize> {
    let mut inside = None;
    for (index, location) in locations.iter().enumerate() {
        match location {
            Some(TriangleLocation::Inside) if inside.is_none() => inside = Some(index),
            Some(TriangleLocation::Inside) => return None,
            Some(TriangleLocation::Outside) => {}
            _ => return None,
        }
    }
    inside
}

fn one_strict_left_vertex_outside_right(
    locations: &[Option<TriangleLocation>; 3],
) -> Option<usize> {
    let mut outside = None;
    for (index, location) in locations.iter().enumerate() {
        match location {
            Some(TriangleLocation::Outside) if outside.is_none() => outside = Some(index),
            Some(TriangleLocation::Outside) => return None,
            Some(TriangleLocation::Inside) => {}
            _ => return None,
        }
    }
    outside
}

fn combined_points(left: &ExactMesh, right: &ExactMesh) -> Vec<Point3> {
    left.vertices()
        .iter()
        .chain(right.vertices())
        .map(|point| point.to_hyperlimit_point())
        .collect()
}

fn all_in_closed_triangle(locations: &[Option<TriangleLocation>; 3]) -> bool {
    locations.iter().all(|location| {
        matches!(
            location,
            Some(TriangleLocation::Inside | TriangleLocation::OnEdge | TriangleLocation::OnVertex)
        )
    })
}

fn clip_convex_polygon(
    subject: &[Point3],
    clip: &[Point3],
    projection: CoplanarProjection,
) -> Option<Vec<Point3>> {
    let clip2 = clip
        .iter()
        .map(|point| project_point(point, projection))
        .collect::<Vec<_>>();
    let orientation = orient2d_report(&clip2[0], &clip2[1], &clip2[2]).value()?;
    if orientation == Sign::Zero {
        return None;
    }

    let mut output = subject.to_vec();
    for edge in 0..clip.len() {
        if output.is_empty() {
            break;
        }
        let a = &clip[edge];
        let b = &clip[(edge + 1) % clip.len()];
        let a2 = &clip2[edge];
        let b2 = &clip2[(edge + 1) % 3];
        let input = output;
        output = Vec::new();
        let mut previous = input.last()?.clone();
        let mut previous_inside =
            point_inside_or_on_edge(&previous, a2, b2, orientation, projection)?;
        for current in input {
            let current_inside =
                point_inside_or_on_edge(&current, a2, b2, orientation, projection)?;
            match (previous_inside, current_inside) {
                (true, true) => output.push(current.clone()),
                (true, false) => {
                    output.push(intersect_segment_with_projected_line(
                        &previous, &current, a, b, projection,
                    )?);
                }
                (false, true) => {
                    output.push(intersect_segment_with_projected_line(
                        &previous, &current, a, b, projection,
                    )?);
                    output.push(current.clone());
                }
                (false, false) => {}
            }
            previous = current;
            previous_inside = current_inside;
        }
    }
    Some(output)
}

fn point_inside_or_on_edge(
    point: &Point3,
    edge_start: &Point2,
    edge_end: &Point2,
    clip_orientation: Sign,
    projection: CoplanarProjection,
) -> Option<bool> {
    let projected = project_point(point, projection);
    let side = orient2d_report(edge_start, edge_end, &projected).value()?;
    Some(side == Sign::Zero || side == clip_orientation)
}

fn intersect_segment_with_projected_line(
    p0: &Point3,
    p1: &Point3,
    line_a: &Point3,
    line_b: &Point3,
    projection: CoplanarProjection,
) -> Option<Point3> {
    let a = project_point(line_a, projection);
    let b = project_point(line_b, projection);
    let q0 = project_point(p0, projection);
    let q1 = project_point(p1, projection);
    let d0 = orient2d_value(&a, &b, &q0);
    let d1 = orient2d_value(&a, &b, &q1);
    let denominator = sub(&d0, &d1);
    if compare_reals(&denominator, &ExactReal::from(0)).value() == Some(Ordering::Equal) {
        return None;
    }
    let t = (d0 / &denominator).ok()?;
    Some(interpolate3(p0, p1, &t))
}

fn simplify_projected_polygon(
    mut polygon: Vec<Point3>,
    projection: CoplanarProjection,
) -> Vec<Point3> {
    remove_duplicate_neighbors(&mut polygon);
    loop {
        let original_len = polygon.len();
        if original_len < 3 {
            return polygon;
        }
        let mut simplified = Vec::with_capacity(original_len);
        for index in 0..original_len {
            let previous = &polygon[(index + original_len - 1) % original_len];
            let current = &polygon[index];
            let next = &polygon[(index + 1) % original_len];
            let pa = project_point(previous, projection);
            let pb = project_point(current, projection);
            let pc = project_point(next, projection);
            if orient2d_report(&pa, &pb, &pc).value() != Some(Sign::Zero) {
                simplified.push(current.clone());
            }
        }
        remove_duplicate_neighbors(&mut simplified);
        if simplified.len() == original_len {
            return simplified;
        }
        polygon = simplified;
    }
}

fn remove_duplicate_neighbors(points: &mut Vec<Point3>) {
    points.dedup_by(|right, left| points_equal(left, right));
    if points.len() > 1 && points_equal(points.first().unwrap(), points.last().unwrap()) {
        points.pop();
    }
}

fn polygon_to_open_mesh(polygon: &[Point3]) -> Option<ExactMesh> {
    polygon_to_open_mesh_with_label(polygon, "exact coplanar triangle intersection")
}

fn polygon_to_open_mesh_with_label(polygon: &[Point3], label: &'static str) -> Option<ExactMesh> {
    if polygon.len() < 3 {
        return None;
    }
    let vertices = polygon
        .iter()
        .map(|point| ExactPoint3::new(point.x.clone(), point.y.clone(), point.z.clone()))
        .collect::<Vec<_>>();
    let triangles = (1..polygon.len() - 1)
        .map(|index| Triangle([0, index, index + 1]))
        .collect::<Vec<_>>();
    ExactMesh::new_with_policy(
        vertices,
        triangles,
        SourceProvenance::exact(label),
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .ok()
}

#[cfg(feature = "exact-triangulation")]
fn polygon_to_earcut_open_mesh(
    polygon: &[Point3],
    projection: CoplanarProjection,
) -> Option<ExactMesh> {
    if polygon.len() < 3 {
        return None;
    }
    let vertices2 = polygon
        .iter()
        .map(|point| match projection {
            CoplanarProjection::Xy => hypertri::ExactPoint::new(point.x.clone(), point.y.clone()),
            CoplanarProjection::Xz => hypertri::ExactPoint::new(point.x.clone(), point.z.clone()),
            CoplanarProjection::Yz => hypertri::ExactPoint::new(point.y.clone(), point.z.clone()),
        })
        .collect::<Vec<_>>();
    let indices = hypertri::earcut(&vertices2, &[]).ok()?;
    if indices.len() % 3 != 0 || indices.is_empty() {
        return None;
    }
    let vertices = polygon
        .iter()
        .map(|point| ExactPoint3::new(point.x.clone(), point.y.clone(), point.z.clone()))
        .collect::<Vec<_>>();
    let triangles = indices
        .chunks_exact(3)
        .map(|chunk| Triangle([chunk[0], chunk[1], chunk[2]]))
        .collect::<Vec<_>>();
    ExactMesh::new_with_policy(
        vertices,
        triangles,
        SourceProvenance::exact("exact coplanar triangle planar arrangement"),
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .ok()
}

#[cfg(feature = "exact-triangulation")]
fn polygon_to_earcut_open_mesh_with_hole(
    outer: &[Point3],
    hole: &[Point3],
    projection: CoplanarProjection,
) -> Option<ExactMesh> {
    if outer.len() < 3 || hole.len() < 3 {
        return None;
    }
    let points = outer.iter().chain(hole).cloned().collect::<Vec<_>>();
    let vertices2 = points
        .iter()
        .map(|point| project_for_hypertri(point, projection))
        .collect::<Vec<_>>();
    let indices = hypertri::earcut(&vertices2, &[outer.len()]).ok()?;
    if indices.len() % 3 != 0 || indices.is_empty() {
        return None;
    }
    let vertices = points
        .iter()
        .map(|point| ExactPoint3::new(point.x.clone(), point.y.clone(), point.z.clone()))
        .collect::<Vec<_>>();
    let triangles = indices
        .chunks_exact(3)
        .map(|chunk| Triangle([chunk[0], chunk[1], chunk[2]]))
        .collect::<Vec<_>>();
    ExactMesh::new_with_policy(
        vertices,
        triangles,
        SourceProvenance::exact("exact coplanar triangle holed arrangement"),
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .ok()
}

#[cfg(feature = "exact-triangulation")]
fn project_for_hypertri(point: &Point3, projection: CoplanarProjection) -> hypertri::ExactPoint {
    match projection {
        CoplanarProjection::Xy => hypertri::ExactPoint::new(point.x.clone(), point.y.clone()),
        CoplanarProjection::Xz => hypertri::ExactPoint::new(point.x.clone(), point.z.clone()),
        CoplanarProjection::Yz => hypertri::ExactPoint::new(point.y.clone(), point.z.clone()),
    }
}

fn validate_coplanar_surface_output(
    projection: CoplanarProjection,
    polygon: &[Point3],
    mesh: &ExactMesh,
    label: &'static str,
) -> Result<(), MeshError> {
    if polygon.len() < 3 {
        return Err(surface_validation_error(
            label,
            "surface polygon has fewer than three vertices",
        ));
    }
    if mesh.vertices().len() != polygon.len() {
        return Err(surface_validation_error(
            label,
            "surface mesh vertex count does not match polygon vertex count",
        ));
    }
    if mesh.triangles().is_empty() {
        return Err(surface_validation_error(
            label,
            "surface mesh has no triangulation",
        ));
    }

    for (index, point) in polygon.iter().enumerate() {
        if !points_equal(point, &mesh.vertices()[index].to_hyperlimit_point()) {
            return Err(surface_validation_error(
                label,
                "surface mesh vertex does not match polygon point",
            ));
        }
    }
    for triangle in mesh.triangles() {
        if triangle.0.iter().any(|&index| index >= polygon.len()) {
            return Err(surface_validation_error(
                label,
                "surface mesh triangle index is out of polygon range",
            ));
        }
    }
    for left in 0..polygon.len() {
        for right in left + 1..polygon.len() {
            if points_equal(&polygon[left], &polygon[right]) {
                return Err(surface_validation_error(
                    label,
                    "surface polygon repeats an exact point",
                ));
            }
        }
    }
    let Some(area) = projected_area2_abs(polygon, projection) else {
        return Err(surface_validation_error(
            label,
            "surface polygon projected area was undecided",
        ));
    };
    if compare_reals(&area, &ExactReal::from(0)).value() != Some(Ordering::Greater) {
        return Err(surface_validation_error(
            label,
            "surface polygon has zero projected area",
        ));
    }

    Ok(())
}

#[cfg(feature = "exact-triangulation")]
fn validate_holed_surface_output(
    projection: CoplanarProjection,
    outer: &[Point3],
    hole: &[Point3],
    mesh: &ExactMesh,
    label: &'static str,
) -> Result<(), MeshError> {
    if outer.len() < 3 || hole.len() < 3 {
        return Err(surface_validation_error(
            label,
            "outer and hole rings must both contain at least three vertices",
        ));
    }
    if mesh.vertices().len() != outer.len() + hole.len() {
        return Err(surface_validation_error(
            label,
            "mesh vertex count does not match retained rings",
        ));
    }
    for (index, point) in outer.iter().chain(hole).enumerate() {
        if !points_equal(point, &mesh.vertices()[index].to_hyperlimit_point()) {
            return Err(surface_validation_error(
                label,
                "mesh vertex does not match retained ring point",
            ));
        }
    }
    let outer_area = projected_area2_abs(outer, projection)
        .ok_or_else(|| surface_validation_error(label, "outer projected area was undecided"))?;
    let hole_area = projected_area2_abs(hole, projection)
        .ok_or_else(|| surface_validation_error(label, "hole projected area was undecided"))?;
    if compare_reals(&outer_area, &hole_area).value() != Some(Ordering::Greater) {
        return Err(surface_validation_error(
            label,
            "hole area must be strictly smaller than outer area",
        ));
    }
    mesh.validate_retained_state().map_err(|_| {
        surface_validation_error(label, "materialized mesh retained-state validation failed")
    })
}

#[cfg(feature = "exact-triangulation")]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum ArrangementOperation {
    Union,
    Difference,
}

#[cfg(feature = "exact-triangulation")]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum ConvexPolygonLocation {
    Inside,
    Boundary,
    Outside,
}

#[cfg(feature = "exact-triangulation")]
#[derive(Clone, Debug)]
struct DirectedFragment {
    start: Point3,
    end: Point3,
}

#[cfg(feature = "exact-triangulation")]
fn arrange_single_triangle_coplanar_surfaces(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ArrangementOperation,
) -> Option<CoplanarTriangleArrangement> {
    if left.triangles().len() != 1 || right.triangles().len() != 1 {
        return None;
    }
    let points = combined_points(left, right);
    let left_tri = left.triangles()[0].0;
    let right_offset = left.vertices().len();
    let right_tri = right.triangles()[0].0.map(|index| index + right_offset);
    let classification = classify_triangle_triangle(&points, left_tri, right_tri);
    if classification.relation != TriangleTriangleRelation::CoplanarOverlapping {
        return None;
    }
    let coplanar = classify_coplanar_triangles(&points, left_tri, right_tri);
    if coplanar.relation != CoplanarTriangleRelation::Overlapping {
        return None;
    }
    let projection = coplanar.projection?;
    let mut left_polygon = triangle_points(&points, left_tri);
    let mut right_polygon = triangle_points(&points, right_tri);
    orient_polygon_ccw(&mut left_polygon, projection)?;
    orient_polygon_ccw(&mut right_polygon, projection)?;

    if operation == ArrangementOperation::Union {
        let polygon = simple_union_boundary_by_exact_angle(
            left,
            right,
            &left_polygon,
            &right_polygon,
            projection,
        )?;
        let mesh = polygon_to_earcut_open_mesh(&polygon, projection)?;
        let arrangement = CoplanarTriangleArrangement {
            projection,
            polygon,
            mesh,
        };
        arrangement.validate().ok()?;
        return Some(arrangement);
    }

    let mut fragments = Vec::new();
    collect_boundary_fragments(
        &left_polygon,
        &right_polygon,
        projection,
        operation,
        true,
        &mut fragments,
    )?;
    collect_boundary_fragments(
        &right_polygon,
        &left_polygon,
        projection,
        operation,
        false,
        &mut fragments,
    )?;
    let polygon = stitch_simple_loop(fragments, projection)?;
    let mesh = polygon_to_earcut_open_mesh(&polygon, projection)?;
    let arrangement = CoplanarTriangleArrangement {
        projection,
        polygon,
        mesh,
    };
    arrangement.validate().ok()?;
    Some(arrangement)
}

#[cfg(feature = "exact-triangulation")]
fn simple_union_boundary_by_exact_angle(
    left_mesh: &ExactMesh,
    right_mesh: &ExactMesh,
    left: &[Point3],
    right: &[Point3],
    projection: CoplanarProjection,
) -> Option<Vec<Point3>> {
    let intersection = intersect_single_triangle_coplanar_surfaces(left_mesh, right_mesh)?;
    let center = average_point3(&intersection.polygon)?;
    let mut boundary = Vec::new();
    for point in left {
        if point_in_projected_triangle(point, right, projection)? != TriangleLocation::Inside {
            boundary.push(point.clone());
        }
    }
    for point in right {
        if point_in_projected_triangle(point, left, projection)? != TriangleLocation::Inside {
            boundary.push(point.clone());
        }
    }
    for left_edge in 0..left.len() {
        for right_edge in 0..right.len() {
            add_projected_edge_intersections(
                &left[left_edge],
                &left[(left_edge + 1) % left.len()],
                &right[right_edge],
                &right[(right_edge + 1) % right.len()],
                projection,
                &mut boundary,
            )?;
        }
    }
    boundary.sort_by(|a, b| compare_points_around_center(a, b, &center, projection));
    dedup_points(&mut boundary);
    let boundary = simplify_projected_polygon(boundary, projection);
    if boundary.len() < 3 {
        return None;
    }
    if !union_boundary_area_is_covered(&boundary, left, right, projection)? {
        return None;
    }
    Some(boundary)
}

#[cfg(feature = "exact-triangulation")]
fn average_point3(points: &[Point3]) -> Option<Point3> {
    if points.is_empty() {
        return None;
    }
    let inv_len = (ExactReal::from(1) / &ExactReal::from(points.len() as i64)).ok()?;
    let mut sum = Point3::new(ExactReal::from(0), ExactReal::from(0), ExactReal::from(0));
    for point in points {
        sum = Point3::new(
            add(&sum.x, &point.x),
            add(&sum.y, &point.y),
            add(&sum.z, &point.z),
        );
    }
    Some(Point3::new(
        mul(&sum.x, &inv_len),
        mul(&sum.y, &inv_len),
        mul(&sum.z, &inv_len),
    ))
}

#[cfg(feature = "exact-triangulation")]
fn single_retained_plane(left: &ExactMesh, right: &ExactMesh) -> Option<bool> {
    for face in 0..left.triangles().len() {
        let classification =
            classify_mesh_triangle_against_retained_face_plane(left, 0, left, face).ok()?;
        if classification.relation != TrianglePlaneRelation::Coplanar {
            return Some(false);
        }
    }
    for face in 0..right.triangles().len() {
        let classification =
            classify_mesh_triangle_against_retained_face_plane(left, 0, right, face).ok()?;
        if classification.relation != TrianglePlaneRelation::Coplanar {
            return Some(false);
        }
    }
    Some(true)
}

#[cfg(feature = "exact-triangulation")]
fn convex_surface_hulls_and_areas(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Option<(
    CoplanarProjection,
    Vec<Point3>,
    Vec<Point3>,
    ExactReal,
    ExactReal,
)> {
    if left.triangles().is_empty() || right.triangles().is_empty() {
        return None;
    }
    if left.triangles().len() == 1 && right.triangles().len() == 1 {
        return None;
    }
    if !single_retained_plane(left, right)? {
        return None;
    }
    let projection = choose_mesh_projection(left)?;
    let left_hull = convex_hull_3d(mesh_points(left), projection)?;
    let right_hull = convex_hull_3d(mesh_points(right), projection)?;
    let left_hull_area = projected_area2_abs(&left_hull, projection)?;
    let right_hull_area = projected_area2_abs(&right_hull, projection)?;
    let left_area = mesh_projected_area2(left, projection)?;
    let right_area = mesh_projected_area2(right, projection)?;
    if compare_reals(&left_area, &left_hull_area).value() != Some(Ordering::Equal)
        || compare_reals(&right_area, &right_hull_area).value() != Some(Ordering::Equal)
    {
        return None;
    }
    Some((projection, left_hull, right_hull, left_area, right_area))
}

#[cfg(feature = "exact-triangulation")]
fn choose_mesh_projection(mesh: &ExactMesh) -> Option<CoplanarProjection> {
    let triangle = mesh.triangles().first()?.0;
    let points = mesh_points(mesh);
    for projection in [
        CoplanarProjection::Xy,
        CoplanarProjection::Xz,
        CoplanarProjection::Yz,
    ] {
        let a = project_point(&points[triangle[0]], projection);
        let b = project_point(&points[triangle[1]], projection);
        let c = project_point(&points[triangle[2]], projection);
        if orient2d_report(&a, &b, &c).value()? != Sign::Zero {
            return Some(projection);
        }
    }
    None
}

#[cfg(feature = "exact-triangulation")]
fn mesh_points(mesh: &ExactMesh) -> Vec<Point3> {
    mesh.vertices()
        .iter()
        .map(ExactPoint3::to_hyperlimit_point)
        .collect()
}

#[cfg(feature = "exact-triangulation")]
fn mesh_projected_area2(mesh: &ExactMesh, projection: CoplanarProjection) -> Option<ExactReal> {
    let points = mesh_points(mesh);
    let mut area = ExactReal::from(0);
    for triangle in mesh.triangles() {
        let tri = triangle
            .0
            .iter()
            .map(|&index| points[index].clone())
            .collect::<Vec<_>>();
        area = add(&area, &projected_area2_abs(&tri, projection)?);
    }
    Some(area)
}

#[cfg(feature = "exact-triangulation")]
fn polygons_equal(left: &[Point3], right: &[Point3]) -> bool {
    if left.len() != right.len() {
        return false;
    }
    if left.is_empty() {
        return true;
    }
    (0..right.len()).any(|offset| {
        left.iter()
            .zip(right.iter().cycle().skip(offset))
            .take(left.len())
            .all(|(left, right)| points_equal(left, right))
    }) || (0..right.len()).any(|offset| {
        left.iter()
            .zip(right.iter().rev().cycle().skip(offset))
            .take(left.len())
            .all(|(left, right)| points_equal(left, right))
    })
}

#[cfg(feature = "exact-triangulation")]
fn polygon_in_closed_convex_polygon(
    inner: &[Point3],
    outer: &[Point3],
    projection: CoplanarProjection,
) -> Option<bool> {
    if inner.len() < 3 || outer.len() < 3 {
        return Some(false);
    }
    inner
        .iter()
        .map(|point| point_in_closed_convex_polygon(point, outer, projection))
        .try_fold(true, |all_inside, inside| Some(all_inside && inside?))
}

#[cfg(feature = "exact-triangulation")]
fn point_in_closed_convex_polygon(
    point: &Point3,
    polygon: &[Point3],
    projection: CoplanarProjection,
) -> Option<bool> {
    Some(convex_polygon_location(point, polygon, projection)? != ConvexPolygonLocation::Outside)
}

#[cfg(feature = "exact-triangulation")]
fn convex_polygon_location(
    point: &Point3,
    polygon: &[Point3],
    projection: CoplanarProjection,
) -> Option<ConvexPolygonLocation> {
    let mut ring = polygon.to_vec();
    orient_polygon_ccw(&mut ring, projection)?;
    let query = project_point(point, projection);
    let mut on_boundary = false;
    for edge in 0..ring.len() {
        let a = project_point(&ring[edge], projection);
        let b = project_point(&ring[(edge + 1) % ring.len()], projection);
        match orient2d_report(&a, &b, &query).value()? {
            Sign::Negative => return Some(ConvexPolygonLocation::Outside),
            Sign::Zero => on_boundary = true,
            Sign::Positive => {}
        }
    }
    Some(if on_boundary {
        ConvexPolygonLocation::Boundary
    } else {
        ConvexPolygonLocation::Inside
    })
}

#[cfg(feature = "exact-triangulation")]
fn compare_points_around_center(
    a: &Point3,
    b: &Point3,
    center: &Point3,
    projection: CoplanarProjection,
) -> Ordering {
    let center2 = project_point(center, projection);
    let a2 = project_point(a, projection);
    let b2 = project_point(b, projection);
    let av = Point2::new(sub(&a2.x, &center2.x), sub(&a2.y, &center2.y));
    let bv = Point2::new(sub(&b2.x, &center2.x), sub(&b2.y, &center2.y));
    match (upper_half(&av), upper_half(&bv)) {
        (true, false) => return Ordering::Less,
        (false, true) => return Ordering::Greater,
        _ => {}
    }
    let sign = orient2d_report(
        &Point2::new(ExactReal::from(0), ExactReal::from(0)),
        &av,
        &bv,
    )
    .value();
    match sign {
        Some(Sign::Positive) => Ordering::Less,
        Some(Sign::Negative) => Ordering::Greater,
        _ => compare_point2(&a2, &b2).unwrap_or(Ordering::Equal),
    }
}

#[cfg(feature = "exact-triangulation")]
fn upper_half(vector: &Point2) -> bool {
    match compare_reals(&vector.y, &ExactReal::from(0)).value() {
        Some(Ordering::Greater) => true,
        Some(Ordering::Less) => false,
        Some(Ordering::Equal) => {
            compare_reals(&vector.x, &ExactReal::from(0)).value() != Some(Ordering::Less)
        }
        None => true,
    }
}

#[cfg(feature = "exact-triangulation")]
fn union_boundary_area_is_covered(
    polygon: &[Point3],
    left: &[Point3],
    right: &[Point3],
    projection: CoplanarProjection,
) -> Option<bool> {
    let vertices2 = polygon
        .iter()
        .map(|point| project_for_hypertri(point, projection))
        .collect::<Vec<_>>();
    let indices = hypertri::earcut(&vertices2, &[]).ok()?;
    if indices.len() % 3 != 0 || indices.is_empty() {
        return Some(false);
    }
    for triangle in indices.chunks_exact(3) {
        let cell = vec![
            polygon[triangle[0]].clone(),
            polygon[triangle[1]].clone(),
            polygon[triangle[2]].clone(),
        ];
        if !fan_triangle_covered_by_inputs(&cell, left, right, projection)? {
            return Some(false);
        }
    }
    Some(true)
}

#[cfg(feature = "exact-triangulation")]
fn convex_union_boundary_area_matches_inputs(
    polygon: &[Point3],
    left: &[Point3],
    right: &[Point3],
    projection: CoplanarProjection,
) -> Option<bool> {
    let union_area = projected_area2_abs(polygon, projection)?;
    let left_area = projected_area2_abs(left, projection)?;
    let right_area = projected_area2_abs(right, projection)?;
    let intersection = convex_polygon_intersection_boundary(left, right, projection)?;
    let intersection_area = if intersection.len() >= 3 {
        projected_area2_abs(&intersection, projection)?
    } else {
        ExactReal::from(0)
    };
    let expected = sub(&add(&left_area, &right_area), &intersection_area);
    Some(compare_reals(&union_area, &expected).value() == Some(Ordering::Equal))
}

#[cfg(feature = "exact-triangulation")]
fn convex_difference_boundary_area_matches_inputs(
    polygon: &[Point3],
    left: &[Point3],
    right: &[Point3],
    projection: CoplanarProjection,
) -> Option<bool> {
    let output_area = projected_area2_abs(polygon, projection)?;
    let left_area = projected_area2_abs(left, projection)?;
    let intersection = convex_polygon_intersection_boundary(left, right, projection)?;
    let intersection_area = if intersection.len() >= 3 {
        projected_area2_abs(&intersection, projection)?
    } else {
        ExactReal::from(0)
    };
    let reconstructed_left = add(&output_area, &intersection_area);
    Some(compare_reals(&reconstructed_left, &left_area).value() == Some(Ordering::Equal))
}

#[cfg(feature = "exact-triangulation")]
fn convex_polygon_intersection_boundary(
    left: &[Point3],
    right: &[Point3],
    projection: CoplanarProjection,
) -> Option<Vec<Point3>> {
    let mut points = Vec::new();
    for point in left {
        if convex_polygon_location(point, right, projection)? != ConvexPolygonLocation::Outside {
            points.push(point.clone());
        }
    }
    for point in right {
        if convex_polygon_location(point, left, projection)? != ConvexPolygonLocation::Outside {
            points.push(point.clone());
        }
    }
    for left_edge in 0..left.len() {
        for right_edge in 0..right.len() {
            add_projected_edge_intersections(
                &left[left_edge],
                &left[(left_edge + 1) % left.len()],
                &right[right_edge],
                &right[(right_edge + 1) % right.len()],
                projection,
                &mut points,
            )?;
        }
    }
    dedup_points(&mut points);
    if points.len() < 3 {
        return Some(points);
    }
    let center = average_point3(&points)?;
    points.sort_by(|a, b| compare_points_around_center(a, b, &center, projection));
    dedup_points(&mut points);
    Some(simplify_projected_polygon(points, projection))
}

#[cfg(feature = "exact-triangulation")]
fn triangle_points(points: &[Point3], tri: [usize; 3]) -> Vec<Point3> {
    tri.iter().map(|&index| points[index].clone()).collect()
}

#[cfg(feature = "exact-triangulation")]
fn orient_polygon_ccw(points: &mut [Point3], projection: CoplanarProjection) -> Option<()> {
    let area = projected_area2_signed(points, projection)?;
    if compare_reals(&area, &ExactReal::from(0)).value() == Some(Ordering::Less) {
        points.reverse();
    }
    Some(())
}

#[cfg(feature = "exact-triangulation")]
fn orient_polygon_cw(points: &mut [Point3], projection: CoplanarProjection) -> Option<()> {
    let area = projected_area2_signed(points, projection)?;
    if compare_reals(&area, &ExactReal::from(0)).value() == Some(Ordering::Greater) {
        points.reverse();
    }
    Some(())
}

#[cfg(feature = "exact-triangulation")]
fn collect_boundary_fragments(
    polygon: &[Point3],
    other: &[Point3],
    projection: CoplanarProjection,
    operation: ArrangementOperation,
    from_left: bool,
    fragments: &mut Vec<DirectedFragment>,
) -> Option<()> {
    for edge in 0..polygon.len() {
        let start = &polygon[edge];
        let end = &polygon[(edge + 1) % polygon.len()];
        let mut splits = vec![start.clone(), end.clone()];
        for other_edge in 0..other.len() {
            let other_start = &other[other_edge];
            let other_end = &other[(other_edge + 1) % other.len()];
            add_projected_edge_intersections(
                start,
                end,
                other_start,
                other_end,
                projection,
                &mut splits,
            )?;
        }
        sort_points_along_segment(&mut splits, start, end, projection)?;
        dedup_points(&mut splits);
        for pair in splits.windows(2) {
            let a = &pair[0];
            let b = &pair[1];
            if points_equal(a, b) {
                continue;
            }
            let midpoint = midpoint3(a, b);
            let location = point_in_projected_triangle(&midpoint, other, projection)?;
            let keep = match (operation, from_left, location) {
                (ArrangementOperation::Union, _, TriangleLocation::Outside) => true,
                (ArrangementOperation::Difference, true, TriangleLocation::Outside) => true,
                (ArrangementOperation::Difference, false, TriangleLocation::Inside) => true,
                _ => false,
            };
            if keep {
                let (fragment_start, fragment_end) =
                    if operation == ArrangementOperation::Difference && !from_left {
                        (b.clone(), a.clone())
                    } else {
                        (a.clone(), b.clone())
                    };
                fragments.push(DirectedFragment {
                    start: fragment_start,
                    end: fragment_end,
                });
            }
        }
    }
    Some(())
}

#[cfg(feature = "exact-triangulation")]
fn collect_convex_union_boundary_fragments(
    polygon: &[Point3],
    other: &[Point3],
    projection: CoplanarProjection,
    fragments: &mut Vec<DirectedFragment>,
) -> Option<()> {
    for edge in 0..polygon.len() {
        let start = &polygon[edge];
        let end = &polygon[(edge + 1) % polygon.len()];
        let mut splits = vec![start.clone(), end.clone()];
        for other_edge in 0..other.len() {
            add_projected_edge_intersections(
                start,
                end,
                &other[other_edge],
                &other[(other_edge + 1) % other.len()],
                projection,
                &mut splits,
            )?;
        }
        sort_points_along_segment(&mut splits, start, end, projection)?;
        dedup_points(&mut splits);
        for pair in splits.windows(2) {
            let a = &pair[0];
            let b = &pair[1];
            if points_equal(a, b) {
                continue;
            }
            let midpoint = midpoint3(a, b);
            if convex_polygon_location(&midpoint, other, projection)?
                == ConvexPolygonLocation::Outside
            {
                fragments.push(DirectedFragment {
                    start: a.clone(),
                    end: b.clone(),
                });
            }
        }
    }
    Some(())
}

#[cfg(feature = "exact-triangulation")]
fn collect_convex_difference_boundary_fragments(
    polygon: &[Point3],
    other: &[Point3],
    projection: CoplanarProjection,
    from_left: bool,
    fragments: &mut Vec<DirectedFragment>,
) -> Option<()> {
    for edge in 0..polygon.len() {
        let start = &polygon[edge];
        let end = &polygon[(edge + 1) % polygon.len()];
        let mut splits = vec![start.clone(), end.clone()];
        for other_edge in 0..other.len() {
            add_projected_edge_intersections(
                start,
                end,
                &other[other_edge],
                &other[(other_edge + 1) % other.len()],
                projection,
                &mut splits,
            )?;
        }
        sort_points_along_segment(&mut splits, start, end, projection)?;
        dedup_points(&mut splits);
        for pair in splits.windows(2) {
            let a = &pair[0];
            let b = &pair[1];
            if points_equal(a, b) {
                continue;
            }
            let midpoint = midpoint3(a, b);
            let location = convex_polygon_location(&midpoint, other, projection)?;
            let keep = if from_left {
                location == ConvexPolygonLocation::Outside
            } else {
                location == ConvexPolygonLocation::Inside
            };
            if keep {
                let (start, end) = if from_left {
                    (a.clone(), b.clone())
                } else {
                    (b.clone(), a.clone())
                };
                fragments.push(DirectedFragment { start, end });
            }
        }
    }
    Some(())
}

#[cfg(feature = "exact-triangulation")]
fn add_projected_edge_intersections(
    a0: &Point3,
    a1: &Point3,
    b0: &Point3,
    b1: &Point3,
    projection: CoplanarProjection,
    splits: &mut Vec<Point3>,
) -> Option<()> {
    let a0p = project_point(a0, projection);
    let a1p = project_point(a1, projection);
    let b0p = project_point(b0, projection);
    let b1p = project_point(b1, projection);
    let a_side_b0 = orient2d_report(&a0p, &a1p, &b0p).value()?;
    let a_side_b1 = orient2d_report(&a0p, &a1p, &b1p).value()?;
    let b_side_a0 = orient2d_report(&b0p, &b1p, &a0p).value()?;
    let b_side_a1 = orient2d_report(&b0p, &b1p, &a1p).value()?;

    if a_side_b0 == Sign::Zero && point_on_projected_segment(a0, a1, b0, projection) {
        splits.push(b0.clone());
    }
    if a_side_b1 == Sign::Zero && point_on_projected_segment(a0, a1, b1, projection) {
        splits.push(b1.clone());
    }
    if b_side_a0 == Sign::Zero && point_on_projected_segment(b0, b1, a0, projection) {
        splits.push(a0.clone());
    }
    if b_side_a1 == Sign::Zero && point_on_projected_segment(b0, b1, a1, projection) {
        splits.push(a1.clone());
    }

    if signs_straddle(a_side_b0, a_side_b1) && signs_straddle(b_side_a0, b_side_a1) {
        splits.push(intersect_segment_with_projected_line(
            a0, a1, b0, b1, projection,
        )?);
    }
    Some(())
}

#[cfg(feature = "exact-triangulation")]
fn signs_straddle(left: Sign, right: Sign) -> bool {
    matches!(
        (left, right),
        (Sign::Positive, Sign::Negative) | (Sign::Negative, Sign::Positive)
    )
}

#[cfg(feature = "exact-triangulation")]
fn sort_points_along_segment(
    points: &mut [Point3],
    start: &Point3,
    end: &Point3,
    projection: CoplanarProjection,
) -> Option<()> {
    points.sort_by(|left, right| {
        let left_t = projected_segment_parameter(start, end, left, projection);
        let right_t = projected_segment_parameter(start, end, right, projection);
        match (left_t, right_t) {
            (Some(left_t), Some(right_t)) => compare_reals(&left_t, &right_t)
                .value()
                .unwrap_or(Ordering::Equal),
            _ => Ordering::Equal,
        }
    });
    Some(())
}

#[cfg(feature = "exact-triangulation")]
fn projected_segment_parameter(
    start: &Point3,
    end: &Point3,
    point: &Point3,
    projection: CoplanarProjection,
) -> Option<ExactReal> {
    let start = project_point(start, projection);
    let end = project_point(end, projection);
    let point = project_point(point, projection);
    let dx = sub(&end.x, &start.x);
    if compare_reals(&dx, &ExactReal::from(0)).value() != Some(Ordering::Equal) {
        return (sub(&point.x, &start.x) / &dx).ok();
    }
    let dy = sub(&end.y, &start.y);
    if compare_reals(&dy, &ExactReal::from(0)).value() != Some(Ordering::Equal) {
        return (sub(&point.y, &start.y) / &dy).ok();
    }
    None
}

#[cfg(feature = "exact-triangulation")]
fn dedup_points(points: &mut Vec<Point3>) {
    points.dedup_by(|right, left| points_equal(left, right));
}

#[cfg(feature = "exact-triangulation")]
fn stitch_simple_loop(
    mut fragments: Vec<DirectedFragment>,
    projection: CoplanarProjection,
) -> Option<Vec<Point3>> {
    if fragments.len() < 3 {
        return None;
    }
    let first = fragments.remove(0);
    let mut polygon = vec![first.start, first.end];
    while !fragments.is_empty() {
        let current = polygon.last()?.clone();
        let next_index = fragments
            .iter()
            .position(|fragment| points_equal(&fragment.start, &current))?;
        let next = fragments.remove(next_index);
        if points_equal(&next.end, polygon.first()?) {
            break;
        }
        polygon.push(next.end);
        if polygon.len() > 64 {
            return None;
        }
    }
    if !fragments.is_empty() {
        return None;
    }
    let polygon = simplify_projected_polygon(polygon, projection);
    if polygon.len() < 3 {
        None
    } else {
        Some(polygon)
    }
}

#[cfg(feature = "exact-triangulation")]
fn midpoint3(a: &Point3, b: &Point3) -> Point3 {
    let half = (ExactReal::from(1) / &ExactReal::from(2)).expect("2 is nonzero");
    Point3::new(
        mul(&add(&a.x, &b.x), &half),
        mul(&add(&a.y, &b.y), &half),
        mul(&add(&a.z, &b.z), &half),
    )
}

#[cfg(feature = "exact-triangulation")]
fn point_in_projected_triangle(
    point: &Point3,
    triangle: &[Point3],
    projection: CoplanarProjection,
) -> Option<TriangleLocation> {
    let query = project_point(point, projection);
    let a = project_point(&triangle[0], projection);
    let b = project_point(&triangle[1], projection);
    let c = project_point(&triangle[2], projection);
    classify_point_triangle(&a, &b, &c, &query).value()
}

#[cfg(feature = "exact-triangulation")]
fn projected_area2_signed(points: &[Point3], projection: CoplanarProjection) -> Option<ExactReal> {
    if points.len() < 3 {
        return Some(ExactReal::from(0));
    }
    let mut sum = ExactReal::from(0);
    for index in 0..points.len() {
        let current = project_point(&points[index], projection);
        let next = project_point(&points[(index + 1) % points.len()], projection);
        sum = add(
            &sum,
            &sub(&mul(&current.x, &next.y), &mul(&current.y, &next.x)),
        );
    }
    Some(sum)
}

fn surface_validation_error(label: &'static str, reason: &'static str) -> MeshError {
    MeshError::one(MeshDiagnostic::new(
        Severity::Error,
        DiagnosticKind::DegenerateTriangle,
        format!("{label} validation failed: {reason}"),
    ))
}

fn convex_hull_3d(points: Vec<Point3>, projection: CoplanarProjection) -> Option<Vec<Point3>> {
    let mut projected = points
        .into_iter()
        .map(|point| {
            let projected = project_point(&point, projection);
            (projected, point)
        })
        .collect::<Vec<_>>();
    projected.sort_by(|left, right| compare_point2(&left.0, &right.0).unwrap_or(Ordering::Equal));
    projected.dedup_by(|right, left| point2_equal(&left.0, &right.0));
    if projected.len() < 3 {
        return None;
    }

    let mut lower = Vec::<(Point2, Point3)>::new();
    for point in &projected {
        while lower.len() >= 2 {
            let sign = orient2d_report(
                &lower[lower.len() - 2].0,
                &lower[lower.len() - 1].0,
                &point.0,
            )
            .value()?;
            if sign == Sign::Positive {
                break;
            }
            lower.pop();
        }
        lower.push(point.clone());
    }

    let mut upper = Vec::<(Point2, Point3)>::new();
    for point in projected.iter().rev() {
        while upper.len() >= 2 {
            let sign = orient2d_report(
                &upper[upper.len() - 2].0,
                &upper[upper.len() - 1].0,
                &point.0,
            )
            .value()?;
            if sign == Sign::Positive {
                break;
            }
            upper.pop();
        }
        upper.push(point.clone());
    }

    lower.pop();
    upper.pop();
    lower.extend(upper);
    let hull = lower
        .into_iter()
        .map(|(_, point)| point)
        .collect::<Vec<_>>();
    if hull.len() < 3 { None } else { Some(hull) }
}

fn fan_triangles_covered_by_inputs(
    hull: &[Point3],
    points: &[Point3],
    left_tri: [usize; 3],
    right_tri: [usize; 3],
    projection: CoplanarProjection,
) -> Option<bool> {
    let left = left_tri
        .iter()
        .map(|&index| points[index].clone())
        .collect::<Vec<_>>();
    let right = right_tri
        .iter()
        .map(|&index| points[index].clone())
        .collect::<Vec<_>>();
    for index in 1..hull.len() - 1 {
        let fan = vec![
            hull[0].clone(),
            hull[index].clone(),
            hull[index + 1].clone(),
        ];
        if !fan_triangle_covered_by_inputs(&fan, &left, &right, projection)? {
            return Some(false);
        }
    }
    Some(true)
}

fn fan_triangle_covered_by_inputs(
    fan: &[Point3],
    left: &[Point3],
    right: &[Point3],
    projection: CoplanarProjection,
) -> Option<bool> {
    let fan_area = projected_area2_abs(fan, projection)?;
    let left_clip = simplify_projected_polygon(
        clip_convex_polygon(fan, left, projection).unwrap_or_default(),
        projection,
    );
    let right_clip = simplify_projected_polygon(
        clip_convex_polygon(fan, right, projection).unwrap_or_default(),
        projection,
    );
    let both_clip = if left_clip.len() >= 3 {
        simplify_projected_polygon(
            clip_convex_polygon(&left_clip, right, projection).unwrap_or_default(),
            projection,
        )
    } else {
        Vec::new()
    };
    let covered = sub(
        &add(
            &projected_area2_abs(&left_clip, projection).unwrap_or_else(|| ExactReal::from(0)),
            &projected_area2_abs(&right_clip, projection).unwrap_or_else(|| ExactReal::from(0)),
        ),
        &projected_area2_abs(&both_clip, projection).unwrap_or_else(|| ExactReal::from(0)),
    );
    Some(compare_reals(&covered, &fan_area).value() == Some(Ordering::Equal))
}

fn project_point(point: &Point3, projection: CoplanarProjection) -> Point2 {
    match projection {
        CoplanarProjection::Xy => Point2::new(point.x.clone(), point.y.clone()),
        CoplanarProjection::Xz => Point2::new(point.x.clone(), point.z.clone()),
        CoplanarProjection::Yz => Point2::new(point.y.clone(), point.z.clone()),
    }
}

fn point_on_projected_segment(
    start: &Point3,
    end: &Point3,
    point: &Point3,
    projection: CoplanarProjection,
) -> bool {
    point_on_segment(
        &project_point(start, projection),
        &project_point(end, projection),
        &project_point(point, projection),
    )
    .value()
        == Some(true)
}

fn projected_area2_abs(points: &[Point3], projection: CoplanarProjection) -> Option<ExactReal> {
    if points.len() < 3 {
        return Some(ExactReal::from(0));
    }
    let mut sum = ExactReal::from(0);
    for index in 0..points.len() {
        let current = project_point(&points[index], projection);
        let next = project_point(&points[(index + 1) % points.len()], projection);
        sum = add(
            &sum,
            &sub(&mul(&current.x, &next.y), &mul(&current.y, &next.x)),
        );
    }
    match compare_reals(&sum, &ExactReal::from(0)).value()? {
        Ordering::Less => Some(sub(&ExactReal::from(0), &sum)),
        Ordering::Equal | Ordering::Greater => Some(sum),
    }
}

fn compare_point2(left: &Point2, right: &Point2) -> Option<Ordering> {
    match compare_reals(&left.x, &right.x).value()? {
        Ordering::Equal => compare_reals(&left.y, &right.y).value(),
        ordering => Some(ordering),
    }
}

fn point2_equal(left: &Point2, right: &Point2) -> bool {
    compare_reals(&left.x, &right.x).value() == Some(Ordering::Equal)
        && compare_reals(&left.y, &right.y).value() == Some(Ordering::Equal)
}

fn orient2d_value(a: &Point2, b: &Point2, c: &Point2) -> ExactReal {
    let bax = sub(&b.x, &a.x);
    let bay = sub(&b.y, &a.y);
    let cax = sub(&c.x, &a.x);
    let cay = sub(&c.y, &a.y);
    sub(&mul(&bax, &cay), &mul(&bay, &cax))
}

fn interpolate3(p0: &Point3, p1: &Point3, t: &ExactReal) -> Point3 {
    Point3::new(
        add(&p0.x, &mul(t, &sub(&p1.x, &p0.x))),
        add(&p0.y, &mul(t, &sub(&p1.y, &p0.y))),
        add(&p0.z, &mul(t, &sub(&p1.z, &p0.z))),
    )
}

fn points_equal(left: &Point3, right: &Point3) -> bool {
    compare_reals(&left.x, &right.x).value() == Some(Ordering::Equal)
        && compare_reals(&left.y, &right.y).value() == Some(Ordering::Equal)
        && compare_reals(&left.z, &right.z).value() == Some(Ordering::Equal)
}

fn add(left: &ExactReal, right: &ExactReal) -> ExactReal {
    left.clone() + right
}

fn sub(left: &ExactReal, right: &ExactReal) -> ExactReal {
    left.clone() - right
}

fn mul(left: &ExactReal, right: &ExactReal) -> ExactReal {
    left.clone() * right
}
