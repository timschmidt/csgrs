//! Exact certification for lower-dimensional surface special cases.
//!
//! This module keeps sheet/surface shortcuts separate from volumetric convex
//! shortcuts. The certified cases are intentionally narrow: single coplanar
//! triangle containment, positive-area intersection, convex union, simple
//! single-loop planar-arrangement union/difference, one-hole and bounded
//! multi-hole differences, and the convex one-corner difference shapes that
//! can be represented as an open triangle mesh. The
//! predicates are the same projected orientation and point-in-triangle facts
//! used by the coplanar overlap classifier, following
//! Yap, "Towards Exact Geometric Computation," *Computational Geometry* 7.1-2
//! (1997): topology claims are emitted only when the combinatorial relation is
//! certified, and missing general planar-cell output models remain explicit.
//!
//! The underlying coplanar test follows the orientation-predicate style of
//! Guigue and Devillers, "Fast and Robust Triangle-Triangle Overlap Test Using
//! Orientation Predicates," *Journal of Graphics Tools* 8.1 (2003), routed
//! through `hyperlimit` by [`crate::exact::coplanar`].

use core::cmp::Ordering;

#[cfg(feature = "exact-triangulation")]
use hyperlimit::classify_point_triangle;
use hyperlimit::{
    Point2, Point3, SegmentIntersection, Sign, TriangleLocation, classify_segment_intersection,
    compare_reals, orient2d_report, point_on_segment,
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
    /// A retained 3D triangle classifier failed its own audit.
    InvalidTriangleClassifier,
    /// A retained projected coplanar classifier failed its own audit.
    InvalidCoplanarClassifier,
    /// The retained classifier relations do not justify the report status.
    StatusRelationMismatch,
    /// The retained classifiers no longer match classifiers recomputed from
    /// the supplied source meshes.
    SourceReplayMismatch,
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
    ///
    /// Presence alone is not enough: the retained 3D and projected classifiers
    /// must replay to the status that carries them. This keeps a certified
    /// containment relation tied to exact predicate artifacts, following Yap,
    /// "Towards Exact Geometric Computation," *Computational Geometry* 7.1-2
    /// (1997), instead of letting callers relabel disjoint or ambiguous
    /// classifier output as containment.
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
                let triangle = self
                    .triangle
                    .as_ref()
                    .ok_or(CoplanarSurfaceContainmentReportError::MissingTriangleClassifier)?;
                triangle.validate().map_err(|_| {
                    CoplanarSurfaceContainmentReportError::InvalidTriangleClassifier
                })?;
                if self.coplanar.is_some() {
                    return Err(CoplanarSurfaceContainmentReportError::UnexpectedClassifier);
                }
                if triangle_reached_coplanar_stage(triangle) {
                    return Err(CoplanarSurfaceContainmentReportError::StatusRelationMismatch);
                }
                Ok(())
            }
            CoplanarSurfaceContainmentStatus::DisjointOrUnknown
            | CoplanarSurfaceContainmentStatus::AmbiguousOrIdentical
            | CoplanarSurfaceContainmentStatus::Certified(_) => {
                let triangle = self
                    .triangle
                    .as_ref()
                    .ok_or(CoplanarSurfaceContainmentReportError::MissingTriangleClassifier)?;
                triangle.validate().map_err(|_| {
                    CoplanarSurfaceContainmentReportError::InvalidTriangleClassifier
                })?;
                if !triangle_reached_coplanar_stage(triangle) {
                    return Err(CoplanarSurfaceContainmentReportError::StatusRelationMismatch);
                }
                let coplanar = self
                    .coplanar
                    .as_ref()
                    .ok_or(CoplanarSurfaceContainmentReportError::MissingCoplanarClassifier)?;
                coplanar.validate().map_err(|_| {
                    CoplanarSurfaceContainmentReportError::InvalidCoplanarClassifier
                })?;
                validate_coplanar_containment_status(&self.status, coplanar)?;
                if matches!(self.status, CoplanarSurfaceContainmentStatus::Certified(_))
                    && !self.all_proof_producing()
                {
                    return Err(CoplanarSurfaceContainmentReportError::StatusRelationMismatch);
                }
                Ok(())
            }
        }
    }

    /// Validate this report against the source meshes that produced it.
    ///
    /// [`Self::validate`] checks that retained classifier artifacts agree with
    /// the stored status. This stronger source-aware check recomputes the
    /// single-triangle containment report from `left` and `right`, then
    /// compares the retained status and classifiers with that replay. This is
    /// the same exact-computation discipline Yap advocates in "Towards Exact
    /// Geometric Computation," *Computational Geometry* 7.1-2 (1997): a
    /// shortcut certificate is not only locally well-formed, it must remain
    /// attached to the source objects whose predicates justified it.
    pub fn validate_against_sources(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> Result<(), CoplanarSurfaceContainmentReportError> {
        self.validate()?;
        let replay = certify_single_triangle_coplanar_containment_report(left, right);
        if self.status == replay.status
            && self.triangle == replay.triangle
            && self.coplanar == replay.coplanar
        {
            Ok(())
        } else {
            Err(CoplanarSurfaceContainmentReportError::SourceReplayMismatch)
        }
    }
}

fn triangle_reached_coplanar_stage(classification: &TriangleTriangleClassification) -> bool {
    matches!(
        classification.relation,
        TriangleTriangleRelation::CoplanarTouching | TriangleTriangleRelation::CoplanarOverlapping
    )
}

fn validate_coplanar_containment_status(
    status: &CoplanarSurfaceContainmentStatus,
    coplanar: &CoplanarTriangleClassification,
) -> Result<(), CoplanarSurfaceContainmentReportError> {
    let left_inside_right = all_in_closed_triangle(&coplanar.left_vertices_in_right);
    let right_inside_left = all_in_closed_triangle(&coplanar.right_vertices_in_left);
    match status {
        CoplanarSurfaceContainmentStatus::DisjointOrUnknown
            if matches!(
                coplanar.relation,
                CoplanarTriangleRelation::Disjoint | CoplanarTriangleRelation::Unknown
            ) =>
        {
            Ok(())
        }
        CoplanarSurfaceContainmentStatus::AmbiguousOrIdentical
            if matches!(
                coplanar.relation,
                CoplanarTriangleRelation::Touching | CoplanarTriangleRelation::Overlapping
            ) && left_inside_right == right_inside_left =>
        {
            Ok(())
        }
        CoplanarSurfaceContainmentStatus::Certified(
            CoplanarSurfaceContainment::LeftInsideRight,
        ) if coplanar.relation == CoplanarTriangleRelation::Overlapping
            && left_inside_right
            && !right_inside_left =>
        {
            Ok(())
        }
        CoplanarSurfaceContainmentStatus::Certified(
            CoplanarSurfaceContainment::RightInsideLeft,
        ) if coplanar.relation == CoplanarTriangleRelation::Overlapping
            && right_inside_left
            && !left_inside_right =>
        {
            Ok(())
        }
        _ => Err(CoplanarSurfaceContainmentReportError::StatusRelationMismatch),
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

    /// Validate this intersection output against the source meshes.
    ///
    /// Local validation proves the retained polygon and mesh agree with each
    /// other. Source replay recomputes the exact Sutherland-Hodgman clipped
    /// intersection from `left` and `right` and requires the retained object to
    /// match that replay. This keeps the shortcut output tied to its certified
    /// predicate history, following Yap, "Towards Exact Geometric Computation,"
    /// *Computational Geometry* 7.1-2 (1997), and the clipping construction of
    /// Sutherland and Hodgman, "Reentrant Polygon Clipping," *Communications of
    /// the ACM* 17.1 (1974).
    pub fn validate_against_sources(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> Result<(), MeshError> {
        self.validate()?;
        let replay = intersect_single_triangle_coplanar_surfaces(left, right).ok_or_else(|| {
            surface_validation_error(
                "coplanar intersection",
                "source replay did not reproduce a positive-area intersection",
            )
        })?;
        if self == &replay {
            Ok(())
        } else {
            Err(surface_validation_error(
                "coplanar intersection",
                "retained intersection does not match source replay",
            ))
        }
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

/// Coplanar surface arrangement operation used for source-aware output replay.
///
/// The retained arrangement output is operation-specific: the same two source
/// sheets may have different certified loops for union, intersection, and
/// difference. Passing the operation explicitly keeps the replay check inside
/// Yap's "Towards Exact Geometric Computation," *Computational Geometry* 7.1-2
/// (1997), model of auditable computation history instead of treating a
/// triangulated sheet as a context-free mesh.
#[cfg(feature = "exact-triangulation")]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CoplanarArrangementOperation {
    /// Replay a certified coplanar union arrangement.
    Union,
    /// Replay a certified coplanar intersection arrangement.
    Intersection,
    /// Replay a certified coplanar difference arrangement.
    Difference,
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

    /// Validate this single-triangle arrangement against its source meshes.
    ///
    /// This first validates the retained loop and `hypertri` mesh, then
    /// recomputes the exact single-triangle arrangement for `operation` from
    /// the supplied sources and requires the retained artifact to match the
    /// replay. The boundary-fragment traversal follows Weiler and Atherton,
    /// "Hidden Surface Removal Using Polygon Area Sorting," *SIGGRAPH Computer
    /// Graphics* 11.2 (1977); the source replay follows Yap's exact-computation
    /// requirement that constructed topology remain attached to its predicate
    /// history.
    pub fn validate_against_sources(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
        operation: CoplanarArrangementOperation,
    ) -> Result<(), MeshError> {
        self.validate()?;
        let replay = match operation {
            CoplanarArrangementOperation::Union => {
                arrange_single_triangle_coplanar_union(left, right)
            }
            CoplanarArrangementOperation::Difference => {
                arrange_single_triangle_coplanar_difference(left, right)
            }
            CoplanarArrangementOperation::Intersection => None,
        }
        .ok_or_else(|| {
            surface_validation_error(
                "coplanar triangle planar arrangement",
                "source replay did not reproduce this arrangement operation",
            )
        })?;
        if self == &replay {
            Ok(())
        } else {
            Err(surface_validation_error(
                "coplanar triangle planar arrangement",
                "retained arrangement does not match source replay",
            ))
        }
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
    /// The retained convex-surface certificate no longer matches classifiers
    /// recomputed from the supplied source meshes.
    SourceReplayMismatch,
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

    /// Validate this report against the source meshes that produced it.
    ///
    /// Convex-surface certificates collapse several exact object facts: shared
    /// coplanarity, retained projected hulls, covered area, and containment
    /// ordering. This method first validates those retained facts locally, then
    /// recomputes the report from `left` and `right` and requires the replay
    /// to match. That keeps the shortcut certificate attached to its source
    /// objects in Yap's sense; see Yap, "Towards Exact Geometric Computation,"
    /// *Computational Geometry* 7.1-2 (1997).
    pub fn validate_against_sources(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> Result<(), CoplanarConvexSurfaceReportError> {
        self.validate()?;
        let replay = certify_coplanar_convex_surface_report(left, right);
        if self.status == replay.status
            && self.equivalence == replay.equivalence
            && self.containment == replay.containment
        {
            Ok(())
        } else {
            Err(CoplanarConvexSurfaceReportError::SourceReplayMismatch)
        }
    }
}

#[cfg(feature = "exact-triangulation")]
impl CoplanarConvexSurfaceContainmentCertificate {
    /// Validate retained hull topology, area, and containment ordering.
    ///
    /// The hull loops are part of the certificate, not derived commentary.
    /// Replaying exact loop distinctness, orientation, convexity, and
    /// containment at this API boundary follows Yap, "Towards Exact Geometric
    /// Computation," *Computational Geometry* 7.1-2 (1997): a named boolean
    /// shortcut may consume a certificate only when its retained structural
    /// facts still justify the collapsed relation.
    pub fn validate(&self) -> Result<(), MeshError> {
        validate_retained_convex_hull(
            "coplanar convex surface containment",
            &self.left_hull,
            self.projection,
        )?;
        validate_retained_convex_hull(
            "coplanar convex surface containment",
            &self.right_hull,
            self.projection,
        )?;
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
            let contained = match self.relation {
                CoplanarConvexSurfaceContainment::LeftInsideRight => {
                    polygon_in_closed_convex_polygon(
                        &self.left_hull,
                        &self.right_hull,
                        self.projection,
                    )
                }
                CoplanarConvexSurfaceContainment::RightInsideLeft => {
                    polygon_in_closed_convex_polygon(
                        &self.right_hull,
                        &self.left_hull,
                        self.projection,
                    )
                }
            }
            .ok_or_else(|| {
                surface_validation_error(
                    "coplanar convex surface containment",
                    "retained hull containment predicate was undecided",
                )
            })?;
            if !contained {
                return Err(surface_validation_error(
                    "coplanar convex surface containment",
                    "retained hulls do not satisfy the containment relation",
                ));
            }
            Ok(())
        } else {
            Err(surface_validation_error(
                "coplanar convex surface containment",
                "containment relation does not match strict hull area ordering",
            ))
        }
    }

    /// Validate this containment certificate against its source meshes.
    ///
    /// The retained hulls and projected areas are public certificate state. This
    /// method recomputes the convex coplanar containment certificate from the
    /// supplied meshes and requires the retained state to match that replay,
    /// following Yap, "Towards Exact Geometric Computation," *Computational
    /// Geometry* 7.1-2 (1997): a shortcut certificate must remain attached to
    /// the exact object facts that produced it. The hull construction follows
    /// Andrew, "Another Efficient Algorithm for Convex Hulls in Two Dimensions,"
    /// *Information Processing Letters* 9.5 (1979).
    pub fn validate_against_sources(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> Result<(), MeshError> {
        self.validate()?;
        let replay = certify_coplanar_convex_surface_containment(left, right).ok_or_else(|| {
            surface_validation_error(
                "coplanar convex surface containment",
                "source replay did not reproduce containment certificate",
            )
        })?;
        if self == &replay {
            Ok(())
        } else {
            Err(surface_validation_error(
                "coplanar convex surface containment",
                "retained certificate does not match source replay",
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

/// Exact multi-hole arrangement output for convex coplanar surface difference.
///
/// This is a bounded planar-cell promotion: one certified convex outer sheet
/// minus several disjoint certified single-triangle holes. The output keeps
/// every ring as exact topology and triangulates through `hypertri` using
/// earcut-compatible hole starts. Yap, "Towards Exact Geometric Computation,"
/// *Computational Geometry* 7.1-2 (1997), is the governing rule here: the
/// multi-hole object is accepted only while its retained rings, exact area, and
/// materialized mesh all replay from the source predicates. The triangulation
/// handoff follows Held, "FIST: Fast Industrial-Strength Triangulation of
/// Polygons," *Algorithmica* 30 (2001).
#[cfg(feature = "exact-triangulation")]
#[derive(Clone, Debug, PartialEq)]
pub struct CoplanarConvexMultiHoledArrangement {
    /// Projection used by exact 2D arrangement predicates and triangulation.
    pub projection: CoplanarProjection,
    /// Exact 3D outer convex hull.
    pub outer: Vec<Point3>,
    /// Exact 3D hole rings, each clockwise and strictly inside `outer`.
    pub holes: Vec<Vec<Point3>>,
    /// Exact triangulated open surface mesh.
    pub mesh: ExactMesh,
}

/// One retained component of a mixed coplanar difference output.
///
/// The component is either a simple outer loop or one outer loop with one or
/// more retained hole loops. It is kept as exact topology, not inferred from
/// the output triangles, following Yap, "Towards Exact Geometric Computation,"
/// *Computational Geometry* 7.1-2 (1997). The ring triangulation handoff uses
/// Held, "FIST: Fast Industrial-Strength Triangulation of Polygons,"
/// *Algorithmica* 30 (2001), through `hypertri`'s exact earcut adapter.
#[cfg(feature = "exact-triangulation")]
#[derive(Clone, Debug, PartialEq)]
pub struct CoplanarConvexHoledComponent {
    /// Exact 3D outer ring, retained counter-clockwise.
    pub outer: Vec<Point3>,
    /// Exact 3D hole rings, retained clockwise and strictly inside `outer`.
    pub holes: Vec<Vec<Point3>>,
}

/// Exact mixed component/holed coplanar difference output.
///
/// This artifact covers the bounded case where a source difference contains
/// one or more disjoint convex components and at least one component carries
/// exact holes. A component may also have bounded convex cutters when the
/// emitted remnants are still convex loops and every retained hole is assigned
/// strictly inside exactly one remnant. More tangled cut/hole interactions
/// still require a full planar subdivision. Each retained component must
/// replay from exact component decomposition, containment, disjointness, and
/// convex difference certificates before the materialized mesh is accepted,
/// matching the retained-object contract in Yap, "Towards Exact Geometric
/// Computation," *Computational Geometry* 7.1-2 (1997).
#[cfg(feature = "exact-triangulation")]
#[derive(Clone, Debug, PartialEq)]
pub struct CoplanarConvexComponentHoledArrangement {
    /// Projection used by exact 2D arrangement predicates and triangulation.
    pub projection: CoplanarProjection,
    /// Retained output components, each with an outer loop and optional holes.
    pub components: Vec<CoplanarConvexHoledComponent>,
    /// Exact triangulated open surface mesh containing all components.
    pub mesh: ExactMesh,
}

#[cfg(feature = "exact-triangulation")]
impl CoplanarConvexHoledArrangement {
    /// Validate ring shape, strict containment, projected area, and mesh state.
    ///
    /// The retained rings are the certificate for a one-hole surface output:
    /// the hole must be a positive-area ring strictly inside the outer ring,
    /// and neither ring may repeat exact points. Keeping those conditions at
    /// the public artifact boundary follows Yap, "Towards Exact Geometric
    /// Computation," *Computational Geometry* 7.1-2 (1997): downstream code
    /// should consume certified topology, not infer it from a triangle soup.
    pub fn validate(&self) -> Result<(), MeshError> {
        validate_holed_surface_output(
            self.projection,
            &self.outer,
            &self.hole,
            &self.mesh,
            "coplanar convex holed arrangement",
        )
    }

    /// Validate this one-hole convex arrangement against its source meshes.
    ///
    /// The only operation represented by this artifact is `left - right` where
    /// the right convex sheet is strictly inside the left. Recomputing that
    /// arrangement from the sources prevents a locally valid ring pair from
    /// being reused after either source hull changes. This follows Yap,
    /// "Towards Exact Geometric Computation," *Computational Geometry* 7.1-2
    /// (1997), and retains Andrew's exact convex-hull certificate boundary.
    pub fn validate_against_sources(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> Result<(), MeshError> {
        self.validate()?;
        let replay =
            arrange_coplanar_convex_surface_holed_difference(left, right).ok_or_else(|| {
                surface_validation_error(
                    "coplanar convex holed arrangement",
                    "source replay did not reproduce a one-hole difference",
                )
            })?;
        if self == &replay {
            Ok(())
        } else {
            Err(surface_validation_error(
                "coplanar convex holed arrangement",
                "retained arrangement does not match source replay",
            ))
        }
    }
}

#[cfg(feature = "exact-triangulation")]
impl CoplanarConvexMultiHoledArrangement {
    /// Validate ring shape, disjointness, projected area, and mesh state.
    pub fn validate(&self) -> Result<(), MeshError> {
        validate_multi_holed_surface_output(
            self.projection,
            &self.outer,
            &self.holes,
            &self.mesh,
            "coplanar convex multi-holed arrangement",
        )
    }

    /// Validate this multi-hole convex arrangement against its source meshes.
    ///
    /// Replaying the bounded multi-hole construction from the exact inputs
    /// prevents a locally valid set of rings from being reused after source
    /// topology changes. That keeps the artifact in Yap's retained-state
    /// model rather than treating the output mesh as detached geometry.
    pub fn validate_against_sources(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> Result<(), MeshError> {
        self.validate()?;
        let replay = arrange_coplanar_convex_surface_multi_holed_difference(left, right)
            .ok_or_else(|| {
                surface_validation_error(
                    "coplanar convex multi-holed arrangement",
                    "source replay did not reproduce a multi-hole difference",
                )
            })?;
        if self == &replay {
            Ok(())
        } else {
            Err(surface_validation_error(
                "coplanar convex multi-holed arrangement",
                "retained arrangement does not match source replay",
            ))
        }
    }
}

#[cfg(feature = "exact-triangulation")]
impl CoplanarConvexComponentHoledArrangement {
    /// Validate component rings, holes, projected area, and mesh state.
    pub fn validate(&self) -> Result<(), MeshError> {
        validate_component_holed_surface_output(
            self.projection,
            &self.components,
            &self.mesh,
            "coplanar convex component-holed arrangement",
        )
    }

    /// Validate this mixed component/holed arrangement against its sources.
    ///
    /// Recomputing the bounded construction from `left` and `right` prevents a
    /// locally valid component/hole set from being transplanted to another
    /// source pair. That is the same source-replay discipline Yap requires in
    /// "Towards Exact Geometric Computation," *Computational Geometry* 7.1-2
    /// (1997): the output mesh is accepted only while its exact construction
    /// facts remain attached to the objects that produced them.
    pub fn validate_against_sources(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> Result<(), MeshError> {
        self.validate()?;
        let replay = arrange_coplanar_convex_surface_component_holed_difference(left, right)
            .ok_or_else(|| {
                surface_validation_error(
                    "coplanar convex component-holed arrangement",
                    "source replay did not reproduce a component-holed difference",
                )
            })?;
        if self == &replay {
            Ok(())
        } else {
            Err(surface_validation_error(
                "coplanar convex component-holed arrangement",
                "retained arrangement does not match source replay",
            ))
        }
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

    /// Validate this convex simple-loop arrangement against its source meshes.
    ///
    /// The replay is operation-specific because convex coplanar intersection,
    /// union, and difference retain different boundary loops. The construction
    /// uses Sutherland-Hodgman half-plane clipping for convex intersection and
    /// Weiler-Atherton-style boundary fragments for union/difference, but this
    /// method accepts the output only when exact source replay reproduces the
    /// retained loop and mesh, following Yap's certified-state discipline.
    pub fn validate_against_sources(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
        operation: CoplanarArrangementOperation,
    ) -> Result<(), MeshError> {
        self.validate()?;
        let replay = match operation {
            CoplanarArrangementOperation::Union => {
                arrange_coplanar_convex_surface_union(left, right)
                    .or_else(|| arrange_coplanar_convex_surface_component_union(left, right))
            }
            CoplanarArrangementOperation::Intersection => {
                arrange_coplanar_convex_surface_intersection(left, right)
            }
            CoplanarArrangementOperation::Difference => {
                arrange_coplanar_convex_surface_difference(left, right)
            }
        }
        .ok_or_else(|| {
            surface_validation_error(
                "coplanar convex surface arrangement",
                "source replay did not reproduce this arrangement operation",
            )
        })?;
        if self == &replay {
            Ok(())
        } else {
            Err(surface_validation_error(
                "coplanar convex surface arrangement",
                "retained arrangement does not match source replay",
            ))
        }
    }
}

/// Exact multi-component arrangement output for convex coplanar fragments.
///
/// This artifact handles bounded multi-component planar arrangement cases:
/// convex coplanar differences whose boundary traversal splits into several
/// disjoint loops, and pairwise clipped coplanar intersections that produce
/// several disjoint positive-area components. It deliberately retains each
/// loop separately instead of flattening the result into an opaque triangle
/// soup. Difference loop construction follows the Weiler-Atherton
/// boundary-fragment traversal idea, while intersection loops use the
/// Sutherland-Hodgman convex clipping model; exact predicates and exact area
/// replay keep both output forms within Yap's exact geometric computation
/// contract. See Weiler and Atherton, "Hidden Surface Removal Using Polygon
/// Area Sorting," *SIGGRAPH Computer Graphics* 11.2 (1977), Sutherland and
/// Hodgman, "Reentrant Polygon Clipping," *Communications of the ACM* 17.1
/// (1974), and Yap, "Towards Exact Geometric Computation," *Computational
/// Geometry* 7.1-2 (1997).
#[cfg(feature = "exact-triangulation")]
#[derive(Clone, Debug, PartialEq)]
pub struct CoplanarConvexMultiArrangement {
    /// Projection used by exact 2D arrangement predicates and triangulation.
    pub projection: CoplanarProjection,
    /// Exact 3D simple boundary loops, one per connected component.
    pub polygons: Vec<Vec<Point3>>,
    /// Exact triangulated open surface mesh containing all components.
    pub mesh: ExactMesh,
}

/// Exact multi-component arrangement output with nonconvex simple loops.
///
/// This artifact is the bounded continuation of
/// [`CoplanarConvexMultiArrangement`] for component-wise differences whose
/// output remains a set of disjoint simple loops but at least one loop is not
/// strictly convex. It exists so the convex certificate does not silently
/// weaken its invariant. Construction still follows exact Weiler-Atherton
/// style boundary replay for each promoted loop, and triangulation is retained
/// through `hypertri`'s FIST-style earcut handoff. See Weiler and Atherton,
/// "Hidden Surface Removal Using Polygon Area Sorting," *SIGGRAPH Computer
/// Graphics* 11.2 (1977), Held, "FIST: Fast Industrial-Strength
/// Triangulation of Polygons," *Algorithmica* 30 (2001), and Yap, "Towards
/// Exact Geometric Computation," *Computational Geometry* 7.1-2 (1997).
#[cfg(feature = "exact-triangulation")]
#[derive(Clone, Debug, PartialEq)]
pub struct CoplanarSurfaceMultiArrangement {
    /// Projection used by exact 2D arrangement predicates and triangulation.
    pub projection: CoplanarProjection,
    /// Exact 3D simple boundary loops, one per connected component.
    pub polygons: Vec<Vec<Point3>>,
    /// Exact triangulated open surface mesh containing all components.
    pub mesh: ExactMesh,
}

/// Exact single-loop arrangement output for nonconvex coplanar surfaces.
///
/// This is the single-component counterpart to
/// [`CoplanarSurfaceMultiArrangement`]. It covers bounded cases where the
/// output is neither convex nor holed, but still has one retained simple loop
/// that can be audited directly. The first producer is the cutter/hole-contact
/// difference path: a side-attached cutter opens a strictly contained hole to
/// the outer boundary, producing one nonconvex loop rather than a ring pair.
/// The output keeps that loop as exact topology and triangulates it through
/// `hypertri`'s FIST-style earcut handoff. See Held, "FIST: Fast
/// Industrial-Strength Triangulation of Polygons," *Algorithmica* 30 (2001),
/// and Yap, "Towards Exact Geometric Computation," *Computational Geometry*
/// 7.1-2 (1997).
#[cfg(feature = "exact-triangulation")]
#[derive(Clone, Debug, PartialEq)]
pub struct CoplanarSurfaceArrangement {
    /// Projection used by exact 2D arrangement predicates and triangulation.
    pub projection: CoplanarProjection,
    /// Exact 3D simple boundary loop, retained counter-clockwise.
    pub polygon: Vec<Point3>,
    /// Exact triangulated open surface mesh for `polygon`.
    pub mesh: ExactMesh,
}

#[cfg(feature = "exact-triangulation")]
impl CoplanarConvexMultiArrangement {
    /// Validate component loops, projected area, and materialized mesh state.
    pub fn validate(&self) -> Result<(), MeshError> {
        validate_multi_surface_output(
            self.projection,
            &self.polygons,
            &self.mesh,
            "coplanar convex multi-component arrangement",
        )
    }

    /// Validate this multi-component convex union against its source meshes.
    ///
    /// The union materializer accepts only disjoint clusters whose individual
    /// convex components replay from exact source-face topology. Recomputing
    /// the cluster union from `left` and `right` keeps the retained component
    /// loops attached to the predicates and source components that produced
    /// them. This follows Yap, "Towards Exact Geometric Computation,"
    /// *Computational Geometry* 7.1-2 (1997): a multi-loop surface is
    /// certified only while its numerical/combinatorial history is replayable.
    pub fn validate_union_against_sources(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> Result<(), MeshError> {
        self.validate()?;
        let replay = arrange_coplanar_convex_surface_multi_union(left, right).ok_or_else(|| {
            surface_validation_error(
                "coplanar convex multi-component arrangement",
                "source replay did not reproduce a multi-component union",
            )
        })?;
        if self == &replay {
            Ok(())
        } else {
            Err(surface_validation_error(
                "coplanar convex multi-component arrangement",
                "retained union does not match source replay",
            ))
        }
    }

    /// Validate this multi-component convex difference against its sources.
    ///
    /// Multi-component outputs retain one exact loop per connected component.
    /// Source replay recomputes the convex difference and verifies both the
    /// component loops and the materialized mesh, so a locally valid component
    /// set cannot be transplanted to a different pair of source sheets. This is
    /// the same retained-computation contract described by Yap, "Towards Exact
    /// Geometric Computation," *Computational Geometry* 7.1-2 (1997).
    pub fn validate_against_sources(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> Result<(), MeshError> {
        self.validate()?;
        let replay =
            arrange_coplanar_convex_surface_multi_difference(left, right).ok_or_else(|| {
                surface_validation_error(
                    "coplanar convex multi-component arrangement",
                    "source replay did not reproduce a multi-component difference",
                )
            })?;
        if self == &replay {
            Ok(())
        } else {
            Err(surface_validation_error(
                "coplanar convex multi-component arrangement",
                "retained arrangement does not match source replay",
            ))
        }
    }

    /// Validate this multi-component intersection against its source meshes.
    ///
    /// Pairwise clipped intersections are accepted only when every retained
    /// component loop and the combined mesh replay from the exact source
    /// triangles. This prevents a locally valid multi-loop artifact from being
    /// reused after source topology or coordinates change, following Yap,
    /// "Towards Exact Geometric Computation," *Computational Geometry* 7.1-2
    /// (1997).
    pub fn validate_intersection_against_sources(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> Result<(), MeshError> {
        self.validate()?;
        let replay =
            arrange_coplanar_convex_surface_multi_intersection(left, right).ok_or_else(|| {
                surface_validation_error(
                    "coplanar convex multi-component arrangement",
                    "source replay did not reproduce a multi-component intersection",
                )
            })?;
        if self == &replay {
            Ok(())
        } else {
            Err(surface_validation_error(
                "coplanar convex multi-component arrangement",
                "retained intersection does not match source replay",
            ))
        }
    }
}

#[cfg(feature = "exact-triangulation")]
impl CoplanarConvexSurfaceEquivalence {
    /// Validate the retained equivalence certificate.
    ///
    /// This replays the retained convex hull as exact topology before checking
    /// area equality. The monotone-chain hull construction used to build the
    /// certificate follows Andrew, "Another Efficient Algorithm for Convex
    /// Hulls in Two Dimensions," *Information Processing Letters* 9.5 (1979);
    /// the public validation follows Yap, "Towards Exact Geometric
    /// Computation," *Computational Geometry* 7.1-2 (1997), by keeping that
    /// hull auditable instead of trusting only an aggregate area.
    pub fn validate(&self) -> Result<(), MeshError> {
        validate_retained_convex_hull(
            "coplanar convex surface equivalence",
            &self.polygon,
            self.projection,
        )?;
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

    /// Validate this equivalence certificate against its source meshes.
    ///
    /// The retained shared hull and both covered-area facts are recomputed from
    /// `left` and `right` and compared with this certificate. That prevents a
    /// locally valid hull/area tuple from being transplanted between source
    /// sheets, matching Yap's retained-computation discipline from "Towards
    /// Exact Geometric Computation," *Computational Geometry* 7.1-2 (1997). The
    /// replayed convex hull uses Andrew, "Another Efficient Algorithm for
    /// Convex Hulls in Two Dimensions," *Information Processing Letters* 9.5
    /// (1979), with exact predicate decisions.
    pub fn validate_against_sources(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> Result<(), MeshError> {
        self.validate()?;
        let replay = certify_coplanar_convex_surface_equivalence(left, right).ok_or_else(|| {
            surface_validation_error(
                "coplanar convex surface equivalence",
                "source replay did not reproduce equivalence certificate",
            )
        })?;
        if self == &replay {
            Ok(())
        } else {
            Err(surface_validation_error(
                "coplanar convex surface equivalence",
                "retained certificate does not match source replay",
            ))
        }
    }
}

#[cfg(feature = "exact-triangulation")]
impl CoplanarTriangleHoledArrangement {
    /// Validate ring shape, strict containment, projected area, and mesh state.
    ///
    /// This validates the same retained one-hole certificate used by Held's
    /// FIST-style triangulation handoff, but with exact ring predicates at the
    /// API boundary; see Held, "FIST: Fast Industrial-Strength Triangulation
    /// of Polygons," *Algorithmica* 30 (2001), and Yap, "Towards Exact
    /// Geometric Computation," *Computational Geometry* 7.1-2 (1997).
    pub fn validate(&self) -> Result<(), MeshError> {
        validate_holed_surface_output(
            self.projection,
            &self.outer,
            &self.hole,
            &self.mesh,
            "coplanar triangle holed arrangement",
        )
    }

    /// Validate this one-hole triangle arrangement against its source meshes.
    ///
    /// The artifact represents `left - right` for a strictly contained coplanar
    /// triangle. Replaying the exact arrangement from the sources ties the
    /// retained outer ring, hole ring, and `hypertri` mesh to the predicate
    /// history that produced them, following Yap, "Towards Exact Geometric
    /// Computation," *Computational Geometry* 7.1-2 (1997), and Held's
    /// FIST-style triangulation handoff for polygon-with-hole inputs.
    pub fn validate_against_sources(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> Result<(), MeshError> {
        self.validate()?;
        let replay =
            arrange_single_triangle_coplanar_holed_difference(left, right).ok_or_else(|| {
                surface_validation_error(
                    "coplanar triangle holed arrangement",
                    "source replay did not reproduce a one-hole difference",
                )
            })?;
        if self == &replay {
            Ok(())
        } else {
            Err(surface_validation_error(
                "coplanar triangle holed arrangement",
                "retained arrangement does not match source replay",
            ))
        }
    }
}

impl CoplanarTriangleUnion {
    /// Validate the materialized convex-union polygon and mesh.
    ///
    /// The union shortcut is accepted only after exact hull coverage checks,
    /// following Andrew's monotone-chain hull construction and Yap's exact
    /// computation boundary. This method validates the persisted output
    /// artifact itself: exact point distinctness, positive projected area,
    /// retained convexity, and fan mesh consistency.
    pub fn validate(&self) -> Result<(), MeshError> {
        validate_retained_convex_hull("coplanar convex union", &self.polygon, self.projection)?;
        validate_coplanar_surface_output(
            self.projection,
            &self.polygon,
            &self.mesh,
            "coplanar convex union",
        )
    }

    /// Validate this convex-union output against the source meshes.
    ///
    /// The convex union shortcut is valid only when exact coverage proves the
    /// combined triangle surface equals the retained hull. This method recomputes
    /// that Andrew monotone-chain hull and its exact coverage checks from the
    /// supplied sources before accepting the retained polygon and mesh. That is
    /// the retained-object discipline advocated by Yap, "Towards Exact Geometric
    /// Computation," *Computational Geometry* 7.1-2 (1997).
    pub fn validate_against_sources(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> Result<(), MeshError> {
        self.validate()?;
        let replay = union_single_triangle_coplanar_surfaces(left, right).ok_or_else(|| {
            surface_validation_error(
                "coplanar convex union",
                "source replay did not reproduce a convex union",
            )
        })?;
        if self == &replay {
            Ok(())
        } else {
            Err(surface_validation_error(
                "coplanar convex union",
                "retained union does not match source replay",
            ))
        }
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
    /// by checking projected area, exact point distinctness, retained
    /// convexity, and mesh fan consistency before callers reuse the artifact.
    pub fn validate(&self) -> Result<(), MeshError> {
        validate_retained_convex_hull(
            "coplanar one-corner difference",
            &self.polygon,
            self.projection,
        )?;
        validate_coplanar_surface_output(
            self.projection,
            &self.polygon,
            &self.mesh,
            "coplanar one-corner difference",
        )
    }

    /// Validate this one-corner difference against the source meshes.
    ///
    /// One-corner difference is accepted only when exact area conservation proves
    /// the retained polygon is `left - right` for the source triangles. This
    /// replay check recomputes that bounded arrangement fragment and rejects a
    /// locally valid polygon/mesh pair that no longer belongs to the supplied
    /// sources, following Yap, "Towards Exact Geometric Computation,"
    /// *Computational Geometry* 7.1-2 (1997).
    pub fn validate_against_sources(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> Result<(), MeshError> {
        self.validate()?;
        let replay =
            difference_single_triangle_coplanar_surfaces(left, right).ok_or_else(|| {
                surface_validation_error(
                    "coplanar one-corner difference",
                    "source replay did not reproduce a one-corner difference",
                )
            })?;
        if self == &replay {
            Ok(())
        } else {
            Err(surface_validation_error(
                "coplanar one-corner difference",
                "retained difference does not match source replay",
            ))
        }
    }
}

#[cfg(feature = "exact-triangulation")]
impl CoplanarSurfaceMultiArrangement {
    /// Validate nonconvex component loops, projected area, and mesh state.
    ///
    /// The retained loops must be simple, counter-clockwise, disjoint, and
    /// exactly matched by the materialized mesh boundary. Unlike
    /// [`CoplanarConvexMultiArrangement`], strict convexity is not a
    /// precondition: this artifact is the named exact boundary for simple
    /// nonconvex component outputs. That separation follows Yap, "Towards
    /// Exact Geometric Computation," *Computational Geometry* 7.1-2 (1997), by
    /// making each topology contract explicit instead of hiding it in a
    /// triangle soup.
    pub fn validate(&self) -> Result<(), MeshError> {
        validate_multi_simple_surface_output(
            self.projection,
            &self.polygons,
            &self.mesh,
            "coplanar nonconvex multi-component arrangement",
        )
    }

    /// Validate this nonconvex multi-component difference against its sources.
    ///
    /// Recomputing the bounded difference from exact source components keeps
    /// every retained simple loop attached to the source predicates that
    /// produced it, matching Yap's retained-computation requirement from
    /// "Towards Exact Geometric Computation," *Computational Geometry* 7.1-2
    /// (1997).
    pub fn validate_difference_against_sources(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> Result<(), MeshError> {
        self.validate()?;
        let replay = arrange_coplanar_surface_multi_difference(left, right).ok_or_else(|| {
            surface_validation_error(
                "coplanar nonconvex multi-component arrangement",
                "source replay did not reproduce a nonconvex multi-component difference",
            )
        })?;
        if self == &replay {
            Ok(())
        } else {
            Err(surface_validation_error(
                "coplanar nonconvex multi-component arrangement",
                "retained difference does not match source replay",
            ))
        }
    }
}

#[cfg(feature = "exact-triangulation")]
impl CoplanarSurfaceArrangement {
    /// Validate the retained nonconvex simple loop and mesh.
    ///
    /// The artifact deliberately does not require convexity. It does require
    /// one positive-area, counter-clockwise, self-disjoint loop whose
    /// triangulated mesh has exactly the same boundary. This keeps the
    /// nonconvex output inside Yap's exact-state discipline: callers receive a
    /// replayable combinatorial object, not only a triangle soup.
    pub fn validate(&self) -> Result<(), MeshError> {
        validate_coplanar_surface_output(
            self.projection,
            &self.polygon,
            &self.mesh,
            "coplanar nonconvex simple-loop arrangement",
        )
    }

    /// Validate this cutter/hole-contact difference against its sources.
    ///
    /// Source replay recomputes the bounded contact construction from the
    /// supplied meshes and requires the retained loop and materialized mesh to
    /// match. This follows Yap, "Towards Exact Geometric Computation,"
    /// *Computational Geometry* 7.1-2 (1997): a nonconvex shortcut remains
    /// certified only while the exact source topology, contact, and area facts
    /// that produced it are still present.
    pub fn validate_cutter_hole_contact_difference_against_sources(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> Result<(), MeshError> {
        self.validate()?;
        let replay = arrange_coplanar_surface_cutter_hole_contact_difference(left, right)
            .ok_or_else(|| {
                surface_validation_error(
                    "coplanar nonconvex simple-loop arrangement",
                    "source replay did not reproduce a cutter-hole contact difference",
                )
            })?;
        if self == &replay {
            Ok(())
        } else {
            Err(surface_validation_error(
                "coplanar nonconvex simple-loop arrangement",
                "retained cutter-hole contact difference does not match source replay",
            ))
        }
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

/// Materialize a convex coplanar difference with several triangle holes.
///
/// This bounded materializer handles one convex coplanar left sheet and a
/// right operand made of two or more disjoint connected convex sheets, all
/// strictly inside the left hull. It is intentionally narrower than arbitrary
/// planar-cell extraction: touching holes, nested holes, and nonconvex
/// coverage still fail closed. The accepted case retains every component hull
/// as a ring and replays exact area, matching Yap's exact-computation
/// discipline.
#[cfg(feature = "exact-triangulation")]
pub fn arrange_coplanar_convex_surface_multi_holed_difference(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Option<CoplanarConvexMultiHoledArrangement> {
    if right.triangles().len() < 2 {
        return None;
    }
    if arrange_coplanar_convex_surface_holed_difference(left, right).is_some() {
        return None;
    }

    let (projection, mut outer, _, _, _) = convex_surface_hulls_and_areas(left, left)?;
    orient_polygon_ccw(&mut outer, projection)?;
    let outer_area = projected_area2_abs(&outer, projection)?;
    let mut holes = Vec::new();
    let mut hole_area_sum = ExactReal::from(0);
    for component in connected_face_component_meshes(right)? {
        let hole_mesh = component;
        let certificate = certify_coplanar_convex_surface_containment(left, &hole_mesh)?;
        if certificate.projection != projection
            || certificate.relation != CoplanarConvexSurfaceContainment::RightInsideLeft
        {
            return None;
        }
        let mut hole = certificate.right_hull;
        orient_polygon_cw(&mut hole, projection)?;
        hole_area_sum = add(&hole_area_sum, &projected_area2_abs(&hole, projection)?);
        holes.push(hole);
    }
    if holes.len() < 2
        || compare_reals(&outer_area, &hole_area_sum).value() != Some(Ordering::Greater)
    {
        return None;
    }
    validate_component_loops_disjoint(
        &holes,
        projection,
        "coplanar convex multi-holed arrangement",
    )
    .ok()?;
    let mesh = polygon_to_earcut_open_mesh_with_holes(
        &outer,
        &holes,
        projection,
        "exact coplanar convex multi-holed arrangement",
    )?;
    let arrangement = CoplanarConvexMultiHoledArrangement {
        projection,
        outer,
        holes,
        mesh,
    };
    arrangement.validate().ok()?;
    Some(arrangement)
}

/// Split a mesh into connected face components using retained triangle edges.
///
/// This is a topology-only decomposition, not a geometric planar arrangement.
/// Components are formed by shared undirected source edges and are then
/// recertified as convex coplanar sheets before they can become holes. That
/// follows Yap's retained-state model from "Towards Exact Geometric
/// Computation," *Computational Geometry* 7.1-2 (1997): component boundaries
/// come from exact mesh topology and must replay through later predicates
/// rather than being inferred from rounded coordinates.
#[cfg(feature = "exact-triangulation")]
fn connected_face_component_meshes(mesh: &ExactMesh) -> Option<Vec<ExactMesh>> {
    if mesh.triangles().is_empty() {
        return None;
    }
    let mut visited = vec![false; mesh.triangles().len()];
    let mut components = Vec::new();
    for seed in 0..mesh.triangles().len() {
        if visited[seed] {
            continue;
        }
        let mut stack = vec![seed];
        let mut faces = Vec::new();
        visited[seed] = true;
        while let Some(face) = stack.pop() {
            faces.push(face);
            for (neighbor, seen) in visited.iter_mut().enumerate() {
                if !*seen
                    && triangles_share_edge(mesh.triangles()[face], mesh.triangles()[neighbor])
                {
                    *seen = true;
                    stack.push(neighbor);
                }
            }
        }
        components.push(component_mesh(mesh, &faces)?);
    }
    Some(components)
}

#[cfg(feature = "exact-triangulation")]
fn triangles_share_edge(left: Triangle, right: Triangle) -> bool {
    let left_edges = triangle_edges(left);
    let right_edges = triangle_edges(right);
    left_edges
        .iter()
        .any(|left| right_edges.iter().any(|right| left == right))
}

#[cfg(feature = "exact-triangulation")]
fn triangle_edges(triangle: Triangle) -> [(usize, usize); 3] {
    [
        canonical_edge(triangle.0[0], triangle.0[1]),
        canonical_edge(triangle.0[1], triangle.0[2]),
        canonical_edge(triangle.0[2], triangle.0[0]),
    ]
}

#[cfg(feature = "exact-triangulation")]
fn component_mesh(mesh: &ExactMesh, faces: &[usize]) -> Option<ExactMesh> {
    let mut vertices = Vec::new();
    let mut old_to_new: Vec<(usize, usize)> = Vec::new();
    let mut triangles = Vec::new();
    for &face in faces {
        let source = mesh.triangles().get(face)?.0;
        let mut remapped = [0; 3];
        for (slot, old) in source.into_iter().enumerate() {
            let new = if let Some((_, new)) =
                old_to_new.iter().find(|(candidate, _)| *candidate == old)
            {
                *new
            } else {
                let new = vertices.len();
                vertices.push(mesh.vertices().get(old)?.clone());
                old_to_new.push((old, new));
                new
            };
            remapped[slot] = new;
        }
        triangles.push(Triangle(remapped));
    }
    ExactMesh::new_with_policy(
        vertices,
        triangles,
        SourceProvenance::exact("exact coplanar source connected component"),
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .ok()
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

/// Certify and materialize disjoint coplanar intersection components.
///
/// This conservative materializer clips every source triangle pair with the
/// single-triangle Sutherland-Hodgman path, then accepts the result only when
/// those positive-area clips form several pairwise disjoint simple loops. It
/// is a bounded bridge toward full planar arrangements: adjacent fragments,
/// nested loops, or overlapping components still return `None` and remain
/// explicit planar-cell work. That keeps the implementation aligned with Yap,
/// "Towards Exact Geometric Computation," *Computational Geometry* 7.1-2
/// (1997), while using Sutherland and Hodgman's convex clipping construction
/// for the local component geometry.
#[cfg(feature = "exact-triangulation")]
pub fn arrange_coplanar_convex_surface_multi_intersection(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Option<CoplanarConvexMultiArrangement> {
    arrange_coplanar_convex_surface_component_intersection(left, right).or_else(|| {
        arrange_coplanar_convex_surface_pairwise_triangle_multi_intersection(left, right)
    })
}

#[cfg(feature = "exact-triangulation")]
fn arrange_coplanar_convex_surface_pairwise_triangle_multi_intersection(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Option<CoplanarConvexMultiArrangement> {
    if left.triangles().is_empty() || right.triangles().is_empty() {
        return None;
    }
    if arrange_coplanar_convex_surface_intersection(left, right).is_some()
        || intersect_single_triangle_coplanar_surfaces(left, right).is_some()
    {
        return None;
    }

    let mut projection = None;
    let mut polygons = Vec::new();
    for left_face in 0..left.triangles().len() {
        let left_triangle = single_face_mesh(left, left_face)?;
        for right_face in 0..right.triangles().len() {
            let right_triangle = single_face_mesh(right, right_face)?;
            let Some(intersection) =
                intersect_single_triangle_coplanar_surfaces(&left_triangle, &right_triangle)
            else {
                continue;
            };
            match projection {
                Some(expected) if expected != intersection.projection => return None,
                None => projection = Some(intersection.projection),
                _ => {}
            }
            let mut polygon = intersection.polygon;
            orient_polygon_ccw(&mut polygon, intersection.projection)?;
            polygons.push(polygon);
        }
    }
    if polygons.len() < 2 {
        return None;
    }
    let projection = projection?;
    let mesh = polygons_to_earcut_open_mesh(&polygons, projection)?;
    let arrangement = CoplanarConvexMultiArrangement {
        projection,
        polygons,
        mesh,
    };
    arrangement.validate().ok()?;
    Some(arrangement)
}

/// Certify disjoint intersections from exact convex source components.
///
/// This component-level path is a bounded generalization of the convex
/// coplanar intersection shortcut. It decomposes both operands by retained
/// source topology, certifies each connected component as a convex coplanar
/// sheet, and emits one exact loop per positive-area component/component
/// intersection. Boundary-only contacts are ignored for triangle-mesh
/// intersection output, while touching or overlapping output loops are
/// rejected so the general arrangement layer remains explicit. The clipping
/// step reuses Sutherland and Hodgman's convex half-plane construction, and
/// the artifact contract follows Yap, "Towards Exact Geometric Computation,"
/// *Computational Geometry* 7.1-2 (1997): exact component structure is kept
/// until every output loop and triangulation replays from source facts.
#[cfg(feature = "exact-triangulation")]
fn arrange_coplanar_convex_surface_component_intersection(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Option<CoplanarConvexMultiArrangement> {
    if arrange_coplanar_convex_surface_intersection(left, right).is_some()
        || intersect_single_triangle_coplanar_surfaces(left, right).is_some()
    {
        return None;
    }

    let left_components = connected_face_component_meshes(left)?;
    let right_components = connected_face_component_meshes(right)?;
    if left_components.len() + right_components.len() < 3 {
        return None;
    }
    let left_components = left_components
        .into_iter()
        .map(|mesh| ConvexUnionComponent::from_mesh(MultiUnionSide::Left, mesh))
        .collect::<Option<Vec<_>>>()?;
    let right_components = right_components
        .into_iter()
        .map(|mesh| ConvexUnionComponent::from_mesh(MultiUnionSide::Right, mesh))
        .collect::<Option<Vec<_>>>()?;
    let projection = left_components.first()?.projection;
    if left_components
        .iter()
        .chain(right_components.iter())
        .any(|component| component.projection != projection)
    {
        return None;
    }
    let left_hulls = left_components
        .iter()
        .map(|component| component.hull.clone())
        .collect::<Vec<_>>();
    validate_component_loops_disjoint(
        &left_hulls,
        projection,
        "coplanar convex component intersection",
    )
    .ok()?;
    let right_hulls = right_components
        .iter()
        .map(|component| component.hull.clone())
        .collect::<Vec<_>>();
    validate_component_loops_disjoint(
        &right_hulls,
        projection,
        "coplanar convex component intersection",
    )
    .ok()?;

    let mut polygons = Vec::new();
    for left_component in &left_components {
        for right_component in &right_components {
            let polygon = if polygons_equal(&left_component.hull, &right_component.hull)
                || polygon_in_closed_convex_polygon(
                    &left_component.hull,
                    &right_component.hull,
                    projection,
                )? {
                Some(left_component.hull.clone())
            } else if polygon_in_closed_convex_polygon(
                &right_component.hull,
                &left_component.hull,
                projection,
            )? {
                Some(right_component.hull.clone())
            } else {
                match convex_union_component_relation(
                    &left_component.hull,
                    &right_component.hull,
                    projection,
                )? {
                    ConvexUnionComponentRelation::Disjoint
                    | ConvexUnionComponentRelation::BoundaryOnly => None,
                    ConvexUnionComponentRelation::PositiveArea => {
                        if let Some(intersection) = arrange_coplanar_convex_surface_intersection(
                            &left_component.mesh,
                            &right_component.mesh,
                        ) {
                            Some(intersection.polygon)
                        } else {
                            intersect_single_triangle_coplanar_surfaces(
                                &left_component.mesh,
                                &right_component.mesh,
                            )
                            .map(|intersection| intersection.polygon)
                        }
                    }
                }
            };
            if let Some(mut polygon) = polygon {
                orient_polygon_ccw(&mut polygon, projection)?;
                polygons.push(polygon);
            }
        }
    }
    if polygons.len() < 2 {
        return None;
    }
    sort_polygons_for_replay(&mut polygons, projection);
    validate_component_loops_disjoint(
        &polygons,
        projection,
        "coplanar convex component intersection",
    )
    .ok()?;
    let mesh = polygons_to_earcut_open_mesh(&polygons, projection)?;
    let arrangement = CoplanarConvexMultiArrangement {
        projection,
        polygons,
        mesh,
    };
    arrangement.validate().ok()?;
    Some(arrangement)
}

#[cfg(feature = "exact-triangulation")]
fn single_face_mesh(mesh: &ExactMesh, face: usize) -> Option<ExactMesh> {
    let triangle = mesh.triangles().get(face)?.0;
    let vertices = triangle
        .iter()
        .map(|&index| mesh.vertices().get(index).cloned())
        .collect::<Option<Vec<_>>>()?;
    ExactMesh::new_with_policy(
        vertices,
        vec![Triangle([0, 1, 2])],
        SourceProvenance::exact("exact coplanar source triangle component"),
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .ok()
}

#[cfg(feature = "exact-triangulation")]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum MultiUnionSide {
    Left,
    Right,
}

#[cfg(feature = "exact-triangulation")]
#[derive(Clone, Debug)]
struct ConvexUnionComponent {
    side: MultiUnionSide,
    mesh: ExactMesh,
    projection: CoplanarProjection,
    hull: Vec<Point3>,
}

#[cfg(feature = "exact-triangulation")]
impl ConvexUnionComponent {
    fn from_mesh(side: MultiUnionSide, mesh: ExactMesh) -> Option<Self> {
        let (projection, mut hull) = convex_component_hull(&mesh)?;
        orient_polygon_ccw(&mut hull, projection)?;
        Some(Self {
            side,
            mesh,
            projection,
            hull,
        })
    }
}

#[cfg(feature = "exact-triangulation")]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum ConvexUnionComponentRelation {
    Disjoint,
    BoundaryOnly,
    PositiveArea,
}

/// Certify one source component as a convex coplanar sheet.
///
/// This helper intentionally accepts single-triangle components, unlike the
/// public multi-face convex-surface shortcut. Multi-component union clustering
/// needs to keep each exact source component as a Yap-style retained object
/// before deciding whether it is an untouched output loop or participates in a
/// two-component union. The hull certificate uses Andrew, "Another Efficient
/// Algorithm for Convex Hulls in Two Dimensions," *Information Processing
/// Letters* 9.5 (1979), with exact projected orientation predicates.
#[cfg(feature = "exact-triangulation")]
fn convex_component_hull(mesh: &ExactMesh) -> Option<(CoplanarProjection, Vec<Point3>)> {
    if mesh.triangles().is_empty() {
        return None;
    }
    for face in 0..mesh.triangles().len() {
        let classification =
            classify_mesh_triangle_against_retained_face_plane(mesh, 0, mesh, face).ok()?;
        if classification.relation != TrianglePlaneRelation::Coplanar {
            return None;
        }
    }
    let projection = choose_mesh_projection(mesh)?;
    let hull = convex_hull_3d(mesh_points(mesh), projection)?;
    let hull_area = projected_area2_abs(&hull, projection)?;
    let mesh_area = mesh_projected_area2(mesh, projection)?;
    if compare_reals(&mesh_area, &hull_area).value() == Some(Ordering::Equal) {
        Some((projection, hull))
    } else {
        None
    }
}

#[cfg(feature = "exact-triangulation")]
fn convex_union_component_relation(
    left: &[Point3],
    right: &[Point3],
    projection: CoplanarProjection,
) -> Option<ConvexUnionComponentRelation> {
    let intersection = convex_polygon_intersection_boundary(left, right, projection)?;
    if intersection.is_empty() {
        return Some(ConvexUnionComponentRelation::Disjoint);
    }
    if intersection.len() < 3 {
        return Some(ConvexUnionComponentRelation::BoundaryOnly);
    }
    let area = projected_area2_abs(&intersection, projection)?;
    match compare_reals(&area, &ExactReal::from(0)).value()? {
        Ordering::Greater => Some(ConvexUnionComponentRelation::PositiveArea),
        Ordering::Equal => Some(ConvexUnionComponentRelation::BoundaryOnly),
        Ordering::Less => None,
    }
}

#[cfg(feature = "exact-triangulation")]
fn materialize_two_component_union(
    left: &ConvexUnionComponent,
    right: &ConvexUnionComponent,
) -> Option<Vec<Point3>> {
    if left.projection != right.projection || left.side == right.side {
        return None;
    }
    if polygons_equal(&left.hull, &right.hull)
        || polygon_in_closed_convex_polygon(&right.hull, &left.hull, left.projection)?
    {
        return Some(left.hull.clone());
    }
    if polygon_in_closed_convex_polygon(&left.hull, &right.hull, left.projection)? {
        return Some(right.hull.clone());
    }
    if let Some(hull) =
        convex_union_hull_covered_by_components(&left.hull, &right.hull, left.projection)
    {
        return Some(hull);
    }
    if let Some(union) = arrange_coplanar_convex_surface_union(&left.mesh, &right.mesh) {
        return Some(union.polygon);
    }
    if let Some(union) = union_single_triangle_coplanar_surfaces(&left.mesh, &right.mesh) {
        return Some(union.polygon);
    }
    arrange_single_triangle_coplanar_union(&left.mesh, &right.mesh).map(|union| union.polygon)
}

#[cfg(feature = "exact-triangulation")]
fn materialize_component_union_group(
    components: &[ConvexUnionComponent],
    members: &[usize],
) -> Option<Vec<Point3>> {
    match members {
        [single] => Some(components[*single].hull.clone()),
        [first, second] => {
            materialize_two_component_union(&components[*first], &components[*second])
        }
        _ => materialize_rectangle_strip_union_cluster(components, members),
    }
}

/// Materialize a many-component convex union cluster as one rectangle strip.
///
/// This is deliberately weaker than a general planar arrangement. It accepts
/// only clusters whose exact projected component hulls are axis-aligned
/// rectangles sharing one interval, while the other interval is connected by
/// exact overlap/touch coverage. The output rectangle is therefore a retained
/// consequence of exact source coordinates. That is the Yap-style contract
/// from "Towards Exact Geometric Computation," *Computational Geometry* 7.1-2
/// (1997): promote topology only when the combinatorial structure is certified
/// by exact data.
#[cfg(feature = "exact-triangulation")]
fn materialize_rectangle_strip_union_cluster(
    components: &[ConvexUnionComponent],
    members: &[usize],
) -> Option<Vec<Point3>> {
    let projection = components[*members.first()?].projection;
    let rectangles = members
        .iter()
        .map(|&member| projected_axis_aligned_rectangle(&components[member].hull, projection))
        .collect::<Option<Vec<_>>>()?;
    rectangle_strip_union_polygon(&rectangles, projection, StripVariableAxis::U)
        .or_else(|| rectangle_strip_union_polygon(&rectangles, projection, StripVariableAxis::V))
}

#[cfg(feature = "exact-triangulation")]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum StripVariableAxis {
    U,
    V,
}

#[cfg(feature = "exact-triangulation")]
#[derive(Clone, Debug)]
struct ProjectedRectangle {
    min: Point2,
    max: Point2,
    dropped: ExactReal,
}

#[cfg(feature = "exact-triangulation")]
fn projected_axis_aligned_rectangle(
    polygon: &[Point3],
    projection: CoplanarProjection,
) -> Option<ProjectedRectangle> {
    if polygon.len() != 4 {
        return None;
    }
    let dropped = dropped_coordinate(polygon.first()?, projection);
    if !polygon
        .iter()
        .all(|point| real_equal(&dropped_coordinate(point, projection), &dropped))
    {
        return None;
    }
    let mut min = project_point(polygon.first()?, projection);
    let mut max = min.clone();
    for point in polygon
        .iter()
        .skip(1)
        .map(|point| project_point(point, projection))
    {
        if real_order(&point.x, &min.x)? == Ordering::Less {
            min.x = point.x.clone();
        }
        if real_order(&point.y, &min.y)? == Ordering::Less {
            min.y = point.y.clone();
        }
        if real_order(&point.x, &max.x)? == Ordering::Greater {
            max.x = point.x.clone();
        }
        if real_order(&point.y, &max.y)? == Ordering::Greater {
            max.y = point.y.clone();
        }
    }
    if real_order(&min.x, &max.x)? != Ordering::Less
        || real_order(&min.y, &max.y)? != Ordering::Less
    {
        return None;
    }
    let corners = [
        Point2::new(min.x.clone(), min.y.clone()),
        Point2::new(max.x.clone(), min.y.clone()),
        Point2::new(max.x.clone(), max.y.clone()),
        Point2::new(min.x.clone(), max.y.clone()),
    ];
    if corners.iter().all(|corner| {
        polygon
            .iter()
            .map(|point| project_point(point, projection))
            .any(|point| point2_equal(&point, corner))
    }) {
        Some(ProjectedRectangle { min, max, dropped })
    } else {
        None
    }
}

#[cfg(feature = "exact-triangulation")]
fn rectangle_strip_union_polygon(
    rectangles: &[ProjectedRectangle],
    projection: CoplanarProjection,
    variable_axis: StripVariableAxis,
) -> Option<Vec<Point3>> {
    let first = rectangles.first()?;
    if !rectangles
        .iter()
        .all(|rect| real_equal(&rect.dropped, &first.dropped))
    {
        return None;
    }

    let fixed_min = strip_fixed_min(first, variable_axis);
    let fixed_max = strip_fixed_max(first, variable_axis);
    if !rectangles.iter().all(|rect| {
        real_equal(strip_fixed_min(rect, variable_axis), fixed_min)
            && real_equal(strip_fixed_max(rect, variable_axis), fixed_max)
    }) {
        return None;
    }

    let mut intervals = rectangles
        .iter()
        .map(|rect| {
            (
                strip_variable_min(rect, variable_axis).clone(),
                strip_variable_max(rect, variable_axis).clone(),
            )
        })
        .collect::<Vec<_>>();
    sort_intervals_by_min(&mut intervals)?;
    let union_min = intervals.first()?.0.clone();
    let mut union_max = intervals.first()?.1.clone();
    for (min, max) in intervals.iter().skip(1) {
        if real_order(min, &union_max)? == Ordering::Greater {
            return None;
        }
        if real_order(max, &union_max)? == Ordering::Greater {
            union_max = max.clone();
        }
    }
    if real_order(&union_min, &union_max)? != Ordering::Less {
        return None;
    }

    let (min, max) = match variable_axis {
        StripVariableAxis::U => (
            Point2::new(union_min, fixed_min.clone()),
            Point2::new(union_max, fixed_max.clone()),
        ),
        StripVariableAxis::V => (
            Point2::new(fixed_min.clone(), union_min),
            Point2::new(fixed_max.clone(), union_max),
        ),
    };
    Some(vec![
        point_from_projection(&min.x, &min.y, &first.dropped, projection),
        point_from_projection(&max.x, &min.y, &first.dropped, projection),
        point_from_projection(&max.x, &max.y, &first.dropped, projection),
        point_from_projection(&min.x, &max.y, &first.dropped, projection),
    ])
}

#[cfg(feature = "exact-triangulation")]
fn sort_intervals_by_min(intervals: &mut [(ExactReal, ExactReal)]) -> Option<()> {
    for index in 1..intervals.len() {
        let mut cursor = index;
        while cursor > 0
            && real_order(&intervals[cursor].0, &intervals[cursor - 1].0)? == Ordering::Less
        {
            intervals.swap(cursor, cursor - 1);
            cursor -= 1;
        }
    }
    Some(())
}

#[cfg(feature = "exact-triangulation")]
fn strip_variable_min(rect: &ProjectedRectangle, axis: StripVariableAxis) -> &ExactReal {
    match axis {
        StripVariableAxis::U => &rect.min.x,
        StripVariableAxis::V => &rect.min.y,
    }
}

#[cfg(feature = "exact-triangulation")]
fn strip_variable_max(rect: &ProjectedRectangle, axis: StripVariableAxis) -> &ExactReal {
    match axis {
        StripVariableAxis::U => &rect.max.x,
        StripVariableAxis::V => &rect.max.y,
    }
}

#[cfg(feature = "exact-triangulation")]
fn strip_fixed_min(rect: &ProjectedRectangle, axis: StripVariableAxis) -> &ExactReal {
    match axis {
        StripVariableAxis::U => &rect.min.y,
        StripVariableAxis::V => &rect.min.x,
    }
}

#[cfg(feature = "exact-triangulation")]
fn strip_fixed_max(rect: &ProjectedRectangle, axis: StripVariableAxis) -> &ExactReal {
    match axis {
        StripVariableAxis::U => &rect.max.y,
        StripVariableAxis::V => &rect.max.x,
    }
}

#[cfg(feature = "exact-triangulation")]
fn dropped_coordinate(point: &Point3, projection: CoplanarProjection) -> ExactReal {
    match projection {
        CoplanarProjection::Xy => point.z.clone(),
        CoplanarProjection::Xz => point.y.clone(),
        CoplanarProjection::Yz => point.x.clone(),
    }
}

#[cfg(feature = "exact-triangulation")]
fn point_from_projection(
    u: &ExactReal,
    v: &ExactReal,
    dropped: &ExactReal,
    projection: CoplanarProjection,
) -> Point3 {
    match projection {
        CoplanarProjection::Xy => Point3::new(u.clone(), v.clone(), dropped.clone()),
        CoplanarProjection::Xz => Point3::new(u.clone(), dropped.clone(), v.clone()),
        CoplanarProjection::Yz => Point3::new(dropped.clone(), u.clone(), v.clone()),
    }
}

/// Materialize a convex hull when two components exactly cover it.
///
/// This is the multi-component counterpart to the single-triangle convex union
/// shortcut: build the Andrew monotone-chain hull of both component rings,
/// then replay every fan triangle by exact clipping against both inputs. The
/// coverage equality is a Yap-style certificate that the hull is not
/// overclaiming a gap, and the clipping pass follows Sutherland and Hodgman,
/// "Reentrant Polygon Clipping," *Communications of the ACM* 17.1 (1974).
#[cfg(feature = "exact-triangulation")]
fn convex_union_hull_covered_by_components(
    left: &[Point3],
    right: &[Point3],
    projection: CoplanarProjection,
) -> Option<Vec<Point3>> {
    let points = left.iter().chain(right.iter()).cloned().collect::<Vec<_>>();
    let hull = convex_hull_3d(points, projection)?;
    if hull.len() < 3 {
        return None;
    }
    for index in 1..hull.len() - 1 {
        let fan = vec![
            hull[0].clone(),
            hull[index].clone(),
            hull[index + 1].clone(),
        ];
        if !fan_triangle_covered_by_inputs(&fan, left, right, projection)? {
            return None;
        }
    }
    Some(hull)
}

#[cfg(feature = "exact-triangulation")]
fn sort_polygons_for_replay(polygons: &mut [Vec<Point3>], projection: CoplanarProjection) {
    polygons.sort_by(|left, right| {
        compare_point2(
            &polygon_min_projected_point(left, projection),
            &polygon_min_projected_point(right, projection),
        )
        .unwrap_or(Ordering::Equal)
    });
}

#[cfg(feature = "exact-triangulation")]
fn sort_components_for_replay(
    components: &mut [CoplanarConvexHoledComponent],
    projection: CoplanarProjection,
) {
    components.sort_by(|left, right| {
        compare_point2(
            &polygon_min_projected_point(&left.outer, projection),
            &polygon_min_projected_point(&right.outer, projection),
        )
        .unwrap_or(Ordering::Equal)
    });
}

#[cfg(feature = "exact-triangulation")]
fn polygon_min_projected_point(polygon: &[Point3], projection: CoplanarProjection) -> Point2 {
    polygon
        .iter()
        .map(|point| project_point(point, projection))
        .min_by(|left, right| compare_point2(left, right).unwrap_or(Ordering::Equal))
        .unwrap_or_else(|| Point2::new(ExactReal::from(0), ExactReal::from(0)))
}

#[cfg(feature = "exact-triangulation")]
struct UnionFind {
    parent: Vec<usize>,
}

#[cfg(feature = "exact-triangulation")]
impl UnionFind {
    fn new(len: usize) -> Self {
        Self {
            parent: (0..len).collect(),
        }
    }

    fn find(&mut self, index: usize) -> usize {
        let parent = self.parent[index];
        if parent == index {
            index
        } else {
            let root = self.find(parent);
            self.parent[index] = root;
            root
        }
    }

    fn union(&mut self, left: usize, right: usize) {
        let left_root = self.find(left);
        let right_root = self.find(right);
        if left_root != right_root {
            self.parent[right_root] = left_root;
        }
    }
}

/// Certify and materialize a simple-loop union of convex coplanar surfaces.
///
/// This is a bounded planar-arrangement port for multi-face sheets. Both
/// inputs must first certify as convex coplanar surface covers by exact hull
/// and area facts. The boundary is then formed from exact edge fragments whose
/// midpoint lies outside the opposite convex hull, stitched into one loop, and
/// accepted only when fan-triangle area coverage proves the loop equals the
/// union. Full-edge contacts can therefore materialize when the retained loop
/// replay proves a positive-area sheet, while point-only contacts remain a
/// boundary-policy case. The traversal follows the Weiler-Atherton
/// boundary-fragment idea, with exact `hyperlimit` orientation predicates
/// providing Yap-style certified combinatorial decisions.
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

/// Certify and materialize a single-loop union from several convex components.
///
/// This covers the bounded case where source topology is already
/// multi-component, but exact component clustering proves the requested union
/// has one simple boundary loop. The accepted many-component cluster is the
/// same axis-aligned rectangle-strip certificate used by
/// [`arrange_coplanar_convex_surface_multi_union`]. Following Yap, "Towards
/// Exact Geometric Computation," *Computational Geometry* 7.1-2 (1997), the
/// output loop is promoted only when exact source-coordinate intervals replay
/// the complete covered strip; general planar subdivisions still remain
/// explicit future arrangement work.
#[cfg(feature = "exact-triangulation")]
pub fn arrange_coplanar_convex_surface_component_union(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Option<CoplanarConvexArrangement> {
    let (projection, mut polygons) = coplanar_convex_surface_component_union_polygons(left, right)?;
    if polygons.len() != 1 {
        return None;
    }
    let polygon = polygons.pop()?;
    let mesh = polygon_to_earcut_open_mesh(&polygon, projection)?;
    let arrangement = CoplanarConvexArrangement {
        projection,
        polygon,
        mesh,
    };
    arrangement.validate().ok()?;
    Some(arrangement)
}

/// Certify and materialize a multi-component convex coplanar union.
///
/// This bounded planar-arrangement path handles the case where exact source
/// topology splits each operand into disjoint convex coplanar components. A
/// cluster is materialized as an unchanged convex hull, a two-component convex
/// union, or a many-component axis-aligned rectangle strip whose exact
/// intervals form one covered rectangle. Boundary-only cross-source contacts
/// are accepted only when that retained interval replay proves a full covered
/// strip; point-only contacts, non-convex component loops, and cases requiring
/// a general planar subdivision remain explicit arrangement work. The
/// clustering is retained object structure in Yap's sense; see Yap, "Towards
/// Exact Geometric Computation," *Computational Geometry* 7.1-2 (1997), and
/// the convex hull certificate follows Andrew, "Another Efficient Algorithm
/// for Convex Hulls in Two Dimensions," *Information Processing Letters* 9.5
/// (1979).
#[cfg(feature = "exact-triangulation")]
pub fn arrange_coplanar_convex_surface_multi_union(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Option<CoplanarConvexMultiArrangement> {
    let (projection, polygons) = coplanar_convex_surface_component_union_polygons(left, right)?;
    if polygons.len() < 2 {
        return None;
    }
    let mesh = polygons_to_earcut_open_mesh(&polygons, projection)?;
    let arrangement = CoplanarConvexMultiArrangement {
        projection,
        polygons,
        mesh,
    };
    arrangement.validate().ok()?;
    Some(arrangement)
}

#[cfg(feature = "exact-triangulation")]
fn coplanar_convex_surface_component_union_polygons(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Option<(CoplanarProjection, Vec<Vec<Point3>>)> {
    if arrange_coplanar_convex_surface_union(left, right).is_some()
        || certify_coplanar_convex_surface_equivalence(left, right).is_some()
        || certify_coplanar_convex_surface_containment(left, right).is_some()
    {
        return None;
    }

    let left_components = connected_face_component_meshes(left)?;
    let right_components = connected_face_component_meshes(right)?;
    if left_components.len() + right_components.len() < 3 {
        return None;
    }

    let mut components = Vec::new();
    for mesh in left_components {
        components.push(ConvexUnionComponent::from_mesh(MultiUnionSide::Left, mesh)?);
    }
    for mesh in right_components {
        components.push(ConvexUnionComponent::from_mesh(
            MultiUnionSide::Right,
            mesh,
        )?);
    }
    let projection = components.first()?.projection;
    if components
        .iter()
        .any(|component| component.projection != projection)
    {
        return None;
    }

    let left_hulls = components
        .iter()
        .filter(|component| component.side == MultiUnionSide::Left)
        .map(|component| component.hull.clone())
        .collect::<Vec<_>>();
    let right_hulls = components
        .iter()
        .filter(|component| component.side == MultiUnionSide::Right)
        .map(|component| component.hull.clone())
        .collect::<Vec<_>>();
    validate_component_loops_disjoint(
        &left_hulls,
        projection,
        "coplanar convex multi-component union",
    )
    .ok()?;
    validate_component_loops_disjoint(
        &right_hulls,
        projection,
        "coplanar convex multi-component union",
    )
    .ok()?;

    let mut union_find = UnionFind::new(components.len());
    for left_index in 0..components.len() {
        for right_index in left_index + 1..components.len() {
            let relation = convex_union_component_relation(
                &components[left_index].hull,
                &components[right_index].hull,
                projection,
            )?;
            match relation {
                ConvexUnionComponentRelation::Disjoint => {}
                ConvexUnionComponentRelation::BoundaryOnly => {
                    if components[left_index].side == components[right_index].side {
                        return None;
                    }
                    // Full-edge cross-source contacts are not topology by
                    // themselves. We only cluster them so the later
                    // rectangle-strip replay can prove a covered interval
                    // complex, preserving Yap's exact-object boundary.
                    union_find.union(left_index, right_index);
                }
                ConvexUnionComponentRelation::PositiveArea => {
                    if components[left_index].side == components[right_index].side {
                        return None;
                    }
                    union_find.union(left_index, right_index);
                }
            }
        }
    }

    let mut groups: Vec<(usize, Vec<usize>)> = Vec::new();
    for index in 0..components.len() {
        let root = union_find.find(index);
        if let Some((_, members)) = groups.iter_mut().find(|(candidate, _)| *candidate == root) {
            members.push(index);
        } else {
            groups.push((root, vec![index]));
        }
    }
    let mut polygons = Vec::with_capacity(groups.len());
    for (_, members) in groups {
        let mut polygon = materialize_component_union_group(&components, &members)?;
        orient_polygon_ccw(&mut polygon, projection)?;
        polygons.push(polygon);
    }
    sort_polygons_for_replay(&mut polygons, projection);
    validate_component_loops_disjoint(
        &polygons,
        projection,
        "coplanar convex multi-component union",
    )
    .ok()?;
    Some((projection, polygons))
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

/// Certify and materialize a multi-component convex coplanar difference.
///
/// This is the bounded multi-component counterpart to
/// [`arrange_coplanar_convex_surface_difference`]. It accepts only convex
/// coplanar sheets whose exact boundary fragments stitch into two or more
/// disjoint simple loops and whose total projected area replays to
/// `area(left) - area(left ∩ right)`. Holed outputs are handled by
/// [`arrange_coplanar_convex_surface_holed_difference`]; single-loop outputs
/// stay on the simpler arrangement path.
#[cfg(feature = "exact-triangulation")]
pub fn arrange_coplanar_convex_surface_multi_difference(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Option<CoplanarConvexMultiArrangement> {
    arrange_coplanar_convex_surface_multi_difference_convex(left, right)
        .or_else(|| arrange_coplanar_convex_surface_component_difference(left, right))
}

#[cfg(feature = "exact-triangulation")]
fn arrange_coplanar_convex_surface_multi_difference_convex(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Option<CoplanarConvexMultiArrangement> {
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
    let mut polygons = stitch_disjoint_simple_loops(fragments, projection)?;
    if polygons.len() < 2 {
        return None;
    }
    for polygon in &mut polygons {
        orient_polygon_ccw(polygon, projection)?;
    }
    if !convex_multi_difference_boundary_area_matches_inputs(
        &polygons,
        &left_hull,
        &right_hull,
        projection,
    )? {
        return None;
    }
    let mesh = polygons_to_earcut_open_mesh(&polygons, projection)?;
    let arrangement = CoplanarConvexMultiArrangement {
        projection,
        polygons,
        mesh,
    };
    arrangement.validate().ok()?;
    Some(arrangement)
}

/// Certify a component-wise coplanar difference for disjoint convex sheets.
///
/// This is a bounded slice of the arbitrary multi-face arrangement problem.
/// The left and right operands are decomposed by exact source topology into
/// disjoint connected convex sheets, and each left component is subtracted
/// independently. A left component is retained unchanged only after exact
/// separation from every right component. A partially cut component replays
/// through the existing convex difference certificates when there is one
/// cutter, or through the bounded rectangle-slab certificate when several
/// disjoint cutters span the component on one projected axis. Other
/// multi-cutter cases are accepted only when each sequential cutter still
/// replays through the existing exact convex difference certificates and
/// emits convex remnants. Boundary-only contacts, holes, nonconvex components,
/// and nonconvex multi-cutter outputs still return `None` so the general
/// arrangement layer remains explicit. This follows Yap, "Towards Exact
/// Geometric Computation,"
/// *Computational Geometry* 7.1-2 (1997): the shortcut promotes output loops
/// only from retained exact component, containment, intersection, and area
/// evidence, never from sampled polygon surgery.
#[cfg(feature = "exact-triangulation")]
fn arrange_coplanar_convex_surface_component_difference(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Option<CoplanarConvexMultiArrangement> {
    if arrange_coplanar_convex_surface_difference(left, right).is_some()
        || arrange_coplanar_convex_surface_holed_difference(left, right).is_some()
    {
        return None;
    }

    let left_components = connected_face_component_meshes(left)?;
    let right_components = connected_face_component_meshes(right)?;
    if right_components.is_empty() {
        return None;
    }

    let mut left_components = left_components
        .into_iter()
        .map(|mesh| ConvexUnionComponent::from_mesh(MultiUnionSide::Left, mesh))
        .collect::<Option<Vec<_>>>()?;
    let right_components = right_components
        .into_iter()
        .map(|mesh| ConvexUnionComponent::from_mesh(MultiUnionSide::Right, mesh))
        .collect::<Option<Vec<_>>>()?;
    let projection = left_components.first()?.projection;
    if left_components
        .iter()
        .chain(right_components.iter())
        .any(|component| component.projection != projection)
    {
        return None;
    }
    let left_hulls = left_components
        .iter()
        .map(|component| component.hull.clone())
        .collect::<Vec<_>>();
    validate_component_loops_disjoint(
        &left_hulls,
        projection,
        "coplanar convex component difference",
    )
    .ok()?;
    let right_hulls = right_components
        .iter()
        .map(|component| component.hull.clone())
        .collect::<Vec<_>>();
    validate_component_loops_disjoint(
        &right_hulls,
        projection,
        "coplanar convex component difference",
    )
    .ok()?;

    let mut polygons = Vec::new();
    for component in &mut left_components {
        let mut drop_component = false;
        let mut cutter_indices = Vec::new();
        for (right_index, right_component) in right_components.iter().enumerate() {
            if polygons_equal(&component.hull, &right_component.hull)
                || polygon_in_closed_convex_polygon(
                    &component.hull,
                    &right_component.hull,
                    projection,
                )?
            {
                if drop_component || !cutter_indices.is_empty() {
                    return None;
                }
                drop_component = true;
                continue;
            }
            if polygon_in_closed_convex_polygon(&right_component.hull, &component.hull, projection)?
            {
                return None;
            }

            match convex_union_component_relation(
                &component.hull,
                &right_component.hull,
                projection,
            )? {
                ConvexUnionComponentRelation::Disjoint => {}
                ConvexUnionComponentRelation::BoundaryOnly => return None,
                ConvexUnionComponentRelation::PositiveArea => {
                    if drop_component {
                        return None;
                    }
                    cutter_indices.push(right_index);
                }
            }
        }
        if drop_component {
            continue;
        }
        match cutter_indices.as_slice() {
            [] => polygons.push(component.hull.clone()),
            [right_index] => {
                let right_component = &right_components[*right_index];
                if let Some(difference) = arrange_coplanar_convex_surface_difference(
                    &component.mesh,
                    &right_component.mesh,
                ) {
                    polygons.push(difference.polygon);
                } else if let Some(difference) =
                    arrange_coplanar_convex_surface_multi_difference_convex(
                        &component.mesh,
                        &right_component.mesh,
                    )
                {
                    polygons.extend(difference.polygons);
                } else {
                    return None;
                }
            }
            _ => {
                let mut remnants = materialize_component_multi_cutter_difference(
                    component,
                    &cutter_indices,
                    &right_components,
                    projection,
                )?;
                polygons.append(&mut remnants);
            }
        }
    }
    if polygons.len() < 2 {
        return None;
    }
    for polygon in &mut polygons {
        orient_polygon_ccw(polygon, projection)?;
    }
    sort_polygons_for_replay(&mut polygons, projection);
    validate_component_loops_disjoint(
        &polygons,
        projection,
        "coplanar convex component difference",
    )
    .ok()?;
    let mesh = polygons_to_earcut_open_mesh(&polygons, projection)?;
    let arrangement = CoplanarConvexMultiArrangement {
        projection,
        polygons,
        mesh,
    };
    arrangement.validate().ok()?;
    Some(arrangement)
}

/// Certify a nonconvex multi-component coplanar difference.
///
/// This is the bounded output-model step beyond the convex component
/// difference certificate. Source topology is
/// still decomposed into disjoint convex components, and every cutter/remnant
/// step must replay through the existing exact convex difference certificates.
/// The only new acceptance is at the retained-output boundary: when a valid
/// result contains two or more disjoint simple loops and at least one loop is
/// nonconvex, the loops are kept in [`CoplanarSurfaceMultiArrangement`] rather
/// than rejected by the convex multi-component certificate. Hole-producing
/// cuts, boundary-only contacts, overlapping loops, and self-intersections
/// remain explicit planar-arrangement work. This follows Yap, "Towards Exact
/// Geometric Computation," *Computational Geometry* 7.1-2 (1997): the system
/// may broaden the object model only when the exact construction history and
/// output topology are both retained.
#[cfg(feature = "exact-triangulation")]
pub fn arrange_coplanar_surface_multi_difference(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Option<CoplanarSurfaceMultiArrangement> {
    let left_components = connected_face_component_meshes(left)?;
    let right_components = connected_face_component_meshes(right)?;
    if right_components.is_empty() {
        return None;
    }

    let mut left_components = left_components
        .into_iter()
        .map(|mesh| ConvexUnionComponent::from_mesh(MultiUnionSide::Left, mesh))
        .collect::<Option<Vec<_>>>()?;
    let right_components = right_components
        .into_iter()
        .map(|mesh| ConvexUnionComponent::from_mesh(MultiUnionSide::Right, mesh))
        .collect::<Option<Vec<_>>>()?;
    let projection = left_components.first()?.projection;
    if left_components
        .iter()
        .chain(right_components.iter())
        .any(|component| component.projection != projection)
    {
        return None;
    }
    let left_hulls = left_components
        .iter()
        .map(|component| component.hull.clone())
        .collect::<Vec<_>>();
    validate_component_loops_disjoint(
        &left_hulls,
        projection,
        "coplanar nonconvex multi-component difference",
    )
    .ok()?;
    let right_hulls = right_components
        .iter()
        .map(|component| component.hull.clone())
        .collect::<Vec<_>>();
    validate_component_loops_disjoint(
        &right_hulls,
        projection,
        "coplanar nonconvex multi-component difference",
    )
    .ok()?;

    let mut polygons = Vec::new();
    for component in &mut left_components {
        let mut drop_component = false;
        let mut cutter_indices = Vec::new();
        for (right_index, right_component) in right_components.iter().enumerate() {
            if polygons_equal(&component.hull, &right_component.hull)
                || polygon_in_closed_convex_polygon(
                    &component.hull,
                    &right_component.hull,
                    projection,
                )?
            {
                if drop_component || !cutter_indices.is_empty() {
                    return None;
                }
                drop_component = true;
                continue;
            }
            if polygon_in_closed_convex_polygon(&right_component.hull, &component.hull, projection)?
            {
                return None;
            }

            match convex_union_component_relation(
                &component.hull,
                &right_component.hull,
                projection,
            )? {
                ConvexUnionComponentRelation::Disjoint => {}
                ConvexUnionComponentRelation::BoundaryOnly => return None,
                ConvexUnionComponentRelation::PositiveArea => {
                    if drop_component {
                        return None;
                    }
                    cutter_indices.push(right_index);
                }
            }
        }
        if drop_component {
            continue;
        }
        match cutter_indices.as_slice() {
            [] => polygons.push(component.hull.clone()),
            [right_index] => {
                let right_component = &right_components[*right_index];
                if let Some(difference) = arrange_coplanar_convex_surface_difference(
                    &component.mesh,
                    &right_component.mesh,
                ) {
                    polygons.push(difference.polygon);
                } else if let Some(difference) =
                    arrange_coplanar_convex_surface_multi_difference_convex(
                        &component.mesh,
                        &right_component.mesh,
                    )
                {
                    polygons.extend(difference.polygons);
                } else {
                    return None;
                }
            }
            _ => {
                let mut remnants = materialize_component_multi_cutter_difference(
                    component,
                    &cutter_indices,
                    &right_components,
                    projection,
                )?;
                polygons.append(&mut remnants);
            }
        }
    }
    if polygons.len() < 2 {
        return None;
    }
    for polygon in &mut polygons {
        orient_polygon_ccw(polygon, projection)?;
    }
    sort_polygons_for_replay(&mut polygons, projection);
    validate_component_loops_disjoint(
        &polygons,
        projection,
        "coplanar nonconvex multi-component difference",
    )
    .ok()?;
    if polygons.iter().all(|polygon| {
        validate_projected_strictly_convex_loop(
            polygon,
            projection,
            "coplanar nonconvex multi-component difference",
        )
        .is_ok()
    }) {
        return None;
    }
    let mesh = polygons_to_earcut_open_mesh_with_label(
        &polygons,
        projection,
        "exact coplanar nonconvex multi-component arrangement",
    )?;
    let arrangement = CoplanarSurfaceMultiArrangement {
        projection,
        polygons,
        mesh,
    };
    arrangement.validate().ok()?;
    Some(arrangement)
}

/// Certify a bounded cutter/hole-contact coplanar difference.
///
/// This handles the narrow case that the strict component/holed arrangement
/// must reject: a side-attached cutter touches a strictly contained hole along
/// a positive-length boundary, so the result is no longer a holed component
/// but one nonconvex simple loop. The accepted source shape is intentionally
/// small and replayable: one exact axis-aligned rectangular left component,
/// one contained rectangular right component, and one rectangular right cutter
/// whose clipped material region touches the contained component and one
/// outer side. The union of the clipped cutter and hole must itself certify as
/// one rectangle attached to exactly one outer side, so output construction is
/// exact interval topology rather than sampled polygon surgery. This follows
/// Yap, "Towards Exact Geometric Computation," *Computational Geometry*
/// 7.1-2 (1997), with the orthogonal-cell reasoning matching the exact
/// rectangle decomposition model in de Berg, Cheong, van Kreveld, and
/// Overmars, *Computational Geometry: Algorithms and Applications*, 3rd ed.,
/// Chapter 2.
#[cfg(feature = "exact-triangulation")]
pub fn arrange_coplanar_surface_cutter_hole_contact_difference(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Option<CoplanarSurfaceArrangement> {
    let left_components = connected_face_component_meshes(left)?;
    let right_components = connected_face_component_meshes(right)?;
    if left_components.len() != 1 || right_components.len() != 2 {
        return None;
    }

    let left_component =
        ConvexUnionComponent::from_mesh(MultiUnionSide::Left, left_components.into_iter().next()?)?;
    let right_components = right_components
        .into_iter()
        .map(|mesh| ConvexUnionComponent::from_mesh(MultiUnionSide::Right, mesh))
        .collect::<Option<Vec<_>>>()?;
    let projection = left_component.projection;
    if right_components
        .iter()
        .any(|component| component.projection != projection)
    {
        return None;
    }
    let left_rect = projected_axis_aligned_rectangle(&left_component.hull, projection)?;

    let mut hole_index = None;
    let mut cutter_index = None;
    for (index, component) in right_components.iter().enumerate() {
        if polygon_strictly_inside_convex_polygon(
            &component.hull,
            &left_component.hull,
            projection,
        )? {
            if hole_index.replace(index).is_some() {
                return None;
            }
        } else if convex_union_component_relation(
            &left_component.hull,
            &component.hull,
            projection,
        )? == ConvexUnionComponentRelation::PositiveArea
        {
            if cutter_index.replace(index).is_some() {
                return None;
            }
        } else {
            return None;
        }
    }
    let hole = &right_components[hole_index?];
    let cutter = &right_components[cutter_index?];
    let hole_rect = projected_axis_aligned_rectangle(&hole.hull, projection)?;
    let cutter_rect = projected_axis_aligned_rectangle(&cutter.hull, projection)?;
    if !rectangles_touch_on_positive_boundary(&hole_rect, &cutter_rect)? {
        return None;
    }

    let mut clipped_cutter =
        convex_polygon_intersection_boundary(&cutter.hull, &left_component.hull, projection)?;
    if clipped_cutter.len() < 3 {
        return None;
    }
    orient_polygon_ccw(&mut clipped_cutter, projection)?;
    let clipped_cutter_rect = projected_axis_aligned_rectangle(&clipped_cutter, projection)?;
    let mut removed_polygon =
        axis_aligned_rectangle_union_polygon(&[clipped_cutter_rect, hole_rect], projection)?;
    orient_polygon_ccw(&mut removed_polygon, projection)?;
    let mut polygon = side_opened_difference_polygon(&left_rect, &removed_polygon, projection)?;
    orient_polygon_ccw(&mut polygon, projection)?;
    polygon = simplify_projected_polygon(polygon, projection);
    if validate_projected_strictly_convex_loop(
        &polygon,
        projection,
        "coplanar cutter-hole contact difference",
    )
    .is_ok()
    {
        return None;
    }

    let left_area = projected_area2_abs(&left_component.hull, projection)?;
    let removed_area = projected_area2_abs(&removed_polygon, projection)?;
    let output_area = projected_area2_abs(&polygon, projection)?;
    if compare_reals(&add(&output_area, &removed_area), &left_area).value() != Some(Ordering::Equal)
    {
        return None;
    }

    let mesh = polygon_to_earcut_open_mesh_with_label(
        &polygon,
        projection,
        "exact coplanar cutter-hole contact difference",
    )?;
    let arrangement = CoplanarSurfaceArrangement {
        projection,
        polygon,
        mesh,
    };
    arrangement.validate().ok()?;
    Some(arrangement)
}

/// Materialize a bounded multi-cutter difference for one convex component.
///
/// The first accepted path is the exact rectangular strip certificate, because
/// it promotes topology directly from retained interval facts. The fallback is
/// still deliberately conservative: apply disjoint cutters one at a time, and
/// after each step require the existing one-cutter convex difference
/// certificates to replay the emitted remnant loops. A cutter contained in a
/// remnant would create a hole, and a cutter whose result is nonconvex cannot
/// be represented by the convex component output model, so both cases stay
/// explicit planar-arrangement work. This is Yap's retained-computation model
/// applied to a bounded Weiler-Atherton style traversal: each promoted loop is
/// produced by an already audited exact arrangement fragment, not by a sampled
/// polygon clip. See Yap, "Towards Exact Geometric Computation,"
/// *Computational Geometry* 7.1-2 (1997), and Weiler and Atherton, "Hidden
/// Surface Removal Using Polygon Area Sorting," *SIGGRAPH Computer Graphics*
/// 11.2 (1977).
#[cfg(feature = "exact-triangulation")]
fn materialize_component_multi_cutter_difference(
    component: &ConvexUnionComponent,
    cutter_indices: &[usize],
    right_components: &[ConvexUnionComponent],
    projection: CoplanarProjection,
) -> Option<Vec<Vec<Point3>>> {
    if cutter_indices.len() < 2 {
        return None;
    }

    let cutters = cutter_indices
        .iter()
        .map(|&index| right_components[index].hull.clone())
        .collect::<Vec<_>>();
    if let Some(rectangle_remnants) =
        materialize_rectangle_multi_cutter_difference(&component.hull, &cutters, projection)
    {
        return Some(rectangle_remnants);
    }

    let mut remnants = vec![component.hull.clone()];
    for &right_index in cutter_indices {
        let cutter = &right_components[right_index];
        let mut next_remnants = Vec::new();
        for mut remnant in remnants {
            if polygons_equal(&remnant, &cutter.hull)
                || polygon_in_closed_convex_polygon(&remnant, &cutter.hull, projection)?
            {
                continue;
            }
            if polygon_in_closed_convex_polygon(&cutter.hull, &remnant, projection)? {
                return None;
            }

            match convex_union_component_relation(&remnant, &cutter.hull, projection)? {
                ConvexUnionComponentRelation::Disjoint => next_remnants.push(remnant),
                ConvexUnionComponentRelation::BoundaryOnly => return None,
                ConvexUnionComponentRelation::PositiveArea => {
                    orient_polygon_ccw(&mut remnant, projection)?;
                    let remnant_mesh = polygon_to_earcut_open_mesh(&remnant, projection)?;
                    if let Some(difference) =
                        arrange_coplanar_convex_surface_difference(&remnant_mesh, &cutter.mesh)
                    {
                        next_remnants.push(difference.polygon);
                    } else if let Some(difference) =
                        arrange_coplanar_convex_surface_multi_difference_convex(
                            &remnant_mesh,
                            &cutter.mesh,
                        )
                    {
                        next_remnants.extend(difference.polygons);
                    } else {
                        return None;
                    }
                }
            }
        }
        remnants = next_remnants;
        if remnants.is_empty() {
            break;
        }
    }

    Some(remnants)
}

/// Materialize a bounded multi-cutter rectangle difference.
///
/// This is a deliberately small substitute for general planar subdivision:
/// the left component must be an exact projected axis-aligned rectangle, every
/// cutter must be an exact rectangle on the same retained plane, and all
/// cutters must span the left rectangle on one fixed projected axis. The
/// output is then the exact interval difference along the other axis, emitted
/// as independent rectangle loops. This follows Yap, "Towards Exact Geometric
/// Computation," *Computational Geometry* 7.1-2 (1997): topology is promoted
/// only from exact retained interval facts. The rectangle-cell viewpoint is a
/// bounded orthogonal subdivision; see de Berg, Cheong, van Kreveld, and
/// Overmars, *Computational Geometry: Algorithms and Applications*, 3rd ed.
/// (2008), Chapter 2.
#[cfg(feature = "exact-triangulation")]
fn materialize_rectangle_multi_cutter_difference(
    left: &[Point3],
    cutters: &[Vec<Point3>],
    projection: CoplanarProjection,
) -> Option<Vec<Vec<Point3>>> {
    if cutters.len() < 2 {
        return None;
    }
    let left_rect = projected_axis_aligned_rectangle(left, projection)?;
    let cutter_rects = cutters
        .iter()
        .map(|cutter| projected_axis_aligned_rectangle(cutter, projection))
        .collect::<Option<Vec<_>>>()?;
    if !cutter_rects
        .iter()
        .all(|rect| real_equal(&rect.dropped, &left_rect.dropped))
    {
        return None;
    }

    rectangle_multi_cutter_difference_polygons(
        &left_rect,
        &cutter_rects,
        projection,
        StripVariableAxis::U,
    )
    .or_else(|| {
        rectangle_multi_cutter_difference_polygons(
            &left_rect,
            &cutter_rects,
            projection,
            StripVariableAxis::V,
        )
    })
}

#[cfg(feature = "exact-triangulation")]
fn rectangle_multi_cutter_difference_polygons(
    left: &ProjectedRectangle,
    cutters: &[ProjectedRectangle],
    projection: CoplanarProjection,
    variable_axis: StripVariableAxis,
) -> Option<Vec<Vec<Point3>>> {
    let left_fixed_min = strip_fixed_min(left, variable_axis);
    let left_fixed_max = strip_fixed_max(left, variable_axis);
    let left_variable_min = strip_variable_min(left, variable_axis);
    let left_variable_max = strip_variable_max(left, variable_axis);
    let mut cutter_intervals = Vec::with_capacity(cutters.len());
    for cutter in cutters {
        if real_order(strip_fixed_min(cutter, variable_axis), left_fixed_min)? == Ordering::Greater
            || real_order(strip_fixed_max(cutter, variable_axis), left_fixed_max)? == Ordering::Less
        {
            return None;
        }
        let cut_min = exact_max_real(strip_variable_min(cutter, variable_axis), left_variable_min)?;
        let cut_max = exact_min_real(strip_variable_max(cutter, variable_axis), left_variable_max)?;
        if real_order(&cut_min, &cut_max)? != Ordering::Less {
            return None;
        }
        cutter_intervals.push((cut_min, cut_max));
    }
    sort_intervals_by_min(&mut cutter_intervals)?;

    let mut retained_intervals = Vec::new();
    let mut cursor = left_variable_min.clone();
    for (cut_min, cut_max) in cutter_intervals {
        match real_order(&cut_min, &cursor)? {
            Ordering::Less => return None,
            Ordering::Equal => {}
            Ordering::Greater => retained_intervals.push((cursor.clone(), cut_min)),
        }
        if real_order(&cut_max, &cursor)? == Ordering::Greater {
            cursor = cut_max;
        }
    }
    if real_order(&cursor, left_variable_max)? == Ordering::Less {
        retained_intervals.push((cursor, left_variable_max.clone()));
    }
    if retained_intervals.is_empty() {
        return None;
    }

    retained_intervals
        .into_iter()
        .map(|(min, max)| {
            rectangle_interval_polygon(
                left,
                projection,
                variable_axis,
                left_fixed_min,
                left_fixed_max,
                &min,
                &max,
            )
        })
        .collect()
}

#[cfg(feature = "exact-triangulation")]
fn rectangle_interval_polygon(
    left: &ProjectedRectangle,
    projection: CoplanarProjection,
    variable_axis: StripVariableAxis,
    fixed_min: &ExactReal,
    fixed_max: &ExactReal,
    variable_min: &ExactReal,
    variable_max: &ExactReal,
) -> Option<Vec<Point3>> {
    if real_order(variable_min, variable_max)? != Ordering::Less
        || real_order(fixed_min, fixed_max)? != Ordering::Less
    {
        return None;
    }
    let (min, max) = match variable_axis {
        StripVariableAxis::U => (
            Point2::new(variable_min.clone(), fixed_min.clone()),
            Point2::new(variable_max.clone(), fixed_max.clone()),
        ),
        StripVariableAxis::V => (
            Point2::new(fixed_min.clone(), variable_min.clone()),
            Point2::new(fixed_max.clone(), variable_max.clone()),
        ),
    };
    Some(vec![
        point_from_projection(&min.x, &min.y, &left.dropped, projection),
        point_from_projection(&max.x, &min.y, &left.dropped, projection),
        point_from_projection(&max.x, &max.y, &left.dropped, projection),
        point_from_projection(&min.x, &max.y, &left.dropped, projection),
    ])
}

#[cfg(feature = "exact-triangulation")]
fn axis_aligned_rectangle_union_polygon(
    rectangles: &[ProjectedRectangle],
    projection: CoplanarProjection,
) -> Option<Vec<Point3>> {
    if rectangles.len() != 2
        || !rectangles
            .iter()
            .all(|rect| real_equal(&rect.dropped, &rectangles[0].dropped))
    {
        return None;
    }
    let mut xs = Vec::new();
    let mut ys = Vec::new();
    for rect in rectangles {
        xs.push(rect.min.x.clone());
        xs.push(rect.max.x.clone());
        ys.push(rect.min.y.clone());
        ys.push(rect.max.y.clone());
    }
    sort_reals_and_dedup(&mut xs)?;
    sort_reals_and_dedup(&mut ys)?;
    if xs.len() < 2 || ys.len() < 2 {
        return None;
    }

    let x_cells = xs.len() - 1;
    let y_cells = ys.len() - 1;
    let mut occupied = vec![false; x_cells * y_cells];
    for x in 0..x_cells {
        for y in 0..y_cells {
            if real_order(&xs[x], &xs[x + 1])? != Ordering::Less
                || real_order(&ys[y], &ys[y + 1])? != Ordering::Less
            {
                continue;
            }
            let midpoint = Point2::new(
                midpoint_real(&xs[x], &xs[x + 1]),
                midpoint_real(&ys[y], &ys[y + 1]),
            );
            occupied[x * y_cells + y] = rectangles
                .iter()
                .any(|rect| point_strictly_inside_projected_rectangle(&midpoint, rect));
        }
    }

    let mut fragments = Vec::new();
    for x in 0..x_cells {
        for y in 0..y_cells {
            if !occupied[x * y_cells + y] {
                continue;
            }
            let x0 = &xs[x];
            let x1 = &xs[x + 1];
            let y0 = &ys[y];
            let y1 = &ys[y + 1];
            let bottom_empty = y == 0 || !occupied[x * y_cells + (y - 1)];
            let top_empty = y + 1 == y_cells || !occupied[x * y_cells + (y + 1)];
            let left_empty = x == 0 || !occupied[(x - 1) * y_cells + y];
            let right_empty = x + 1 == x_cells || !occupied[(x + 1) * y_cells + y];
            if bottom_empty {
                fragments.push(DirectedFragment {
                    start: point_from_projection(x0, y0, &rectangles[0].dropped, projection),
                    end: point_from_projection(x1, y0, &rectangles[0].dropped, projection),
                });
            }
            if right_empty {
                fragments.push(DirectedFragment {
                    start: point_from_projection(x1, y0, &rectangles[0].dropped, projection),
                    end: point_from_projection(x1, y1, &rectangles[0].dropped, projection),
                });
            }
            if top_empty {
                fragments.push(DirectedFragment {
                    start: point_from_projection(x1, y1, &rectangles[0].dropped, projection),
                    end: point_from_projection(x0, y1, &rectangles[0].dropped, projection),
                });
            }
            if left_empty {
                fragments.push(DirectedFragment {
                    start: point_from_projection(x0, y1, &rectangles[0].dropped, projection),
                    end: point_from_projection(x0, y0, &rectangles[0].dropped, projection),
                });
            }
        }
    }
    stitch_simple_loop(fragments, projection)
}

#[cfg(feature = "exact-triangulation")]
fn sort_reals_and_dedup(values: &mut Vec<ExactReal>) -> Option<()> {
    for index in 1..values.len() {
        let mut cursor = index;
        while cursor > 0 && real_order(&values[cursor], &values[cursor - 1])? == Ordering::Less {
            values.swap(cursor, cursor - 1);
            cursor -= 1;
        }
    }
    values.dedup_by(|right, left| real_equal(left, right));
    Some(())
}

#[cfg(feature = "exact-triangulation")]
fn midpoint_real(left: &ExactReal, right: &ExactReal) -> ExactReal {
    let half = (ExactReal::from(1) / &ExactReal::from(2)).expect("2 is nonzero");
    mul(&add(left, right), &half)
}

#[cfg(feature = "exact-triangulation")]
fn point_strictly_inside_projected_rectangle(point: &Point2, rect: &ProjectedRectangle) -> bool {
    real_order(&rect.min.x, &point.x) == Some(Ordering::Less)
        && real_order(&point.x, &rect.max.x) == Some(Ordering::Less)
        && real_order(&rect.min.y, &point.y) == Some(Ordering::Less)
        && real_order(&point.y, &rect.max.y) == Some(Ordering::Less)
}

#[cfg(feature = "exact-triangulation")]
fn rectangles_touch_on_positive_boundary(
    left: &ProjectedRectangle,
    right: &ProjectedRectangle,
) -> Option<bool> {
    if !real_equal(&left.dropped, &right.dropped) {
        return Some(false);
    }
    let vertical_touch = (real_equal(&left.max.x, &right.min.x)
        || real_equal(&right.max.x, &left.min.x))
        && intervals_overlap_with_positive_length(
            &left.min.y,
            &left.max.y,
            &right.min.y,
            &right.max.y,
        )?;
    let horizontal_touch = (real_equal(&left.max.y, &right.min.y)
        || real_equal(&right.max.y, &left.min.y))
        && intervals_overlap_with_positive_length(
            &left.min.x,
            &left.max.x,
            &right.min.x,
            &right.max.x,
        )?;
    Some(vertical_touch || horizontal_touch)
}

#[cfg(feature = "exact-triangulation")]
fn intervals_overlap_with_positive_length(
    left_min: &ExactReal,
    left_max: &ExactReal,
    right_min: &ExactReal,
    right_max: &ExactReal,
) -> Option<bool> {
    let overlap_min = exact_max_real(left_min, right_min)?;
    let overlap_max = exact_min_real(left_max, right_max)?;
    Some(real_order(&overlap_min, &overlap_max)? == Ordering::Less)
}

#[cfg(feature = "exact-triangulation")]
fn side_opened_difference_polygon(
    outer: &ProjectedRectangle,
    removed: &[Point3],
    projection: CoplanarProjection,
) -> Option<Vec<Point3>> {
    let ox0 = &outer.min.x;
    let ox1 = &outer.max.x;
    let oy0 = &outer.min.y;
    let oy1 = &outer.max.y;
    if removed.len() < 3
        || removed
            .iter()
            .any(|point| !real_equal(&dropped_coordinate(point, projection), &outer.dropped))
    {
        return None;
    }

    let corners = [
        point_from_projection(ox0, oy0, &outer.dropped, projection),
        point_from_projection(ox1, oy0, &outer.dropped, projection),
        point_from_projection(ox1, oy1, &outer.dropped, projection),
        point_from_projection(ox0, oy1, &outer.dropped, projection),
    ];
    let side = opened_side_attachment(outer, removed, projection)?;
    let mut polygon = match side {
        OpenedSideAttachment::Left { low, high } => {
            let mut path = removed_boundary_path_reverse(removed, &high, &low)?;
            let mut out = vec![
                corners[0].clone(),
                corners[1].clone(),
                corners[2].clone(),
                corners[3].clone(),
                high,
            ];
            out.append(&mut path);
            out
        }
        OpenedSideAttachment::Right { low, high } => {
            let mut path = removed_boundary_path_reverse(removed, &low, &high)?;
            let mut out = vec![corners[0].clone(), corners[1].clone(), low];
            out.append(&mut path);
            out.push(high);
            out.push(corners[2].clone());
            out.push(corners[3].clone());
            out
        }
        OpenedSideAttachment::Bottom { low, high } => {
            let mut path = removed_boundary_path_reverse(removed, &low, &high)?;
            let mut out = vec![corners[0].clone(), low];
            out.append(&mut path);
            out.push(high);
            out.push(corners[1].clone());
            out.push(corners[2].clone());
            out.push(corners[3].clone());
            out
        }
        OpenedSideAttachment::Top { low, high } => {
            let mut path = removed_boundary_path_reverse(removed, &high, &low)?;
            let mut out = vec![
                corners[0].clone(),
                corners[1].clone(),
                corners[2].clone(),
                high,
            ];
            out.append(&mut path);
            out.push(low);
            out.push(corners[3].clone());
            out
        }
    };
    remove_duplicate_neighbors(&mut polygon);
    Some(polygon)
}

#[cfg(feature = "exact-triangulation")]
enum OpenedSideAttachment {
    Left { low: Point3, high: Point3 },
    Right { low: Point3, high: Point3 },
    Bottom { low: Point3, high: Point3 },
    Top { low: Point3, high: Point3 },
}

#[cfg(feature = "exact-triangulation")]
fn opened_side_attachment(
    outer: &ProjectedRectangle,
    removed: &[Point3],
    projection: CoplanarProjection,
) -> Option<OpenedSideAttachment> {
    let mut left = Vec::new();
    let mut right = Vec::new();
    let mut bottom = Vec::new();
    let mut top = Vec::new();
    for point in removed {
        let projected = project_point(point, projection);
        if real_order(&projected.x, &outer.min.x)? == Ordering::Less
            || real_order(&outer.max.x, &projected.x)? == Ordering::Less
            || real_order(&projected.y, &outer.min.y)? == Ordering::Less
            || real_order(&outer.max.y, &projected.y)? == Ordering::Less
        {
            return None;
        }
        if real_equal(&projected.x, &outer.min.x) {
            left.push(point.clone());
        }
        if real_equal(&projected.x, &outer.max.x) {
            right.push(point.clone());
        }
        if real_equal(&projected.y, &outer.min.y) {
            bottom.push(point.clone());
        }
        if real_equal(&projected.y, &outer.max.y) {
            top.push(point.clone());
        }
    }

    let mut attachments = Vec::new();
    if let Some((low, high)) = vertical_attachment_points(&mut left, outer, projection, true)? {
        attachments.push(OpenedSideAttachment::Left { low, high });
    }
    if let Some((low, high)) = vertical_attachment_points(&mut right, outer, projection, true)? {
        attachments.push(OpenedSideAttachment::Right { low, high });
    }
    if let Some((low, high)) = horizontal_attachment_points(&mut bottom, outer, projection, true)? {
        attachments.push(OpenedSideAttachment::Bottom { low, high });
    }
    if let Some((low, high)) = horizontal_attachment_points(&mut top, outer, projection, true)? {
        attachments.push(OpenedSideAttachment::Top { low, high });
    }
    if attachments.len() == 1 {
        attachments.pop()
    } else {
        None
    }
}

#[cfg(feature = "exact-triangulation")]
fn vertical_attachment_points(
    points: &mut Vec<Point3>,
    outer: &ProjectedRectangle,
    projection: CoplanarProjection,
    require_interior: bool,
) -> Option<Option<(Point3, Point3)>> {
    dedup_points(points);
    if points.is_empty() {
        return Some(None);
    }
    if points.len() != 2 {
        return None;
    }
    points.sort_by(|left, right| {
        real_order(
            &project_point(left, projection).y,
            &project_point(right, projection).y,
        )
        .unwrap_or(Ordering::Equal)
    });
    let low = points[0].clone();
    let high = points[1].clone();
    let low_y = project_point(&low, projection).y;
    let high_y = project_point(&high, projection).y;
    if real_order(&low_y, &high_y)? != Ordering::Less {
        return None;
    }
    if require_interior
        && (real_order(&outer.min.y, &low_y)? != Ordering::Less
            || real_order(&high_y, &outer.max.y)? != Ordering::Less)
    {
        return None;
    }
    Some(Some((low, high)))
}

#[cfg(feature = "exact-triangulation")]
fn horizontal_attachment_points(
    points: &mut Vec<Point3>,
    outer: &ProjectedRectangle,
    projection: CoplanarProjection,
    require_interior: bool,
) -> Option<Option<(Point3, Point3)>> {
    dedup_points(points);
    if points.is_empty() {
        return Some(None);
    }
    if points.len() != 2 {
        return None;
    }
    points.sort_by(|left, right| {
        real_order(
            &project_point(left, projection).x,
            &project_point(right, projection).x,
        )
        .unwrap_or(Ordering::Equal)
    });
    let low = points[0].clone();
    let high = points[1].clone();
    let low_x = project_point(&low, projection).x;
    let high_x = project_point(&high, projection).x;
    if real_order(&low_x, &high_x)? != Ordering::Less {
        return None;
    }
    if require_interior
        && (real_order(&outer.min.x, &low_x)? != Ordering::Less
            || real_order(&high_x, &outer.max.x)? != Ordering::Less)
    {
        return None;
    }
    Some(Some((low, high)))
}

#[cfg(feature = "exact-triangulation")]
fn removed_boundary_path_reverse(
    polygon: &[Point3],
    start: &Point3,
    end: &Point3,
) -> Option<Vec<Point3>> {
    let start_index = polygon
        .iter()
        .position(|point| points_equal(point, start))?;
    let end_index = polygon.iter().position(|point| points_equal(point, end))?;
    let mut path = Vec::new();
    let mut index = start_index;
    loop {
        path.push(polygon[index].clone());
        if index == end_index {
            break;
        }
        index = if index == 0 {
            polygon.len() - 1
        } else {
            index - 1
        };
        if index == start_index {
            return None;
        }
    }
    Some(path)
}

#[cfg(feature = "exact-triangulation")]
fn exact_min_real(left: &ExactReal, right: &ExactReal) -> Option<ExactReal> {
    match real_order(left, right)? {
        Ordering::Less | Ordering::Equal => Some(left.clone()),
        Ordering::Greater => Some(right.clone()),
    }
}

#[cfg(feature = "exact-triangulation")]
fn exact_max_real(left: &ExactReal, right: &ExactReal) -> Option<ExactReal> {
    match real_order(left, right)? {
        Ordering::Greater | Ordering::Equal => Some(left.clone()),
        Ordering::Less => Some(right.clone()),
    }
}

/// Certify a mixed component/holed coplanar difference.
///
/// This is the next bounded step toward arbitrary multi-component,
/// multi-hole planar arrangements. The operands are decomposed into exact
/// connected convex components. Each left component may be retained, removed,
/// cut by one right component through the existing convex difference
/// certificate, cut by several full-span rectangular slab components through
/// exact interval subtraction, cut by sequential non-rectangular convex
/// cutters when each emitted remnant still replays through the exact convex
/// difference certificates, pierced by one or more strictly contained right
/// components, or both cut and pierced when every retained hole falls strictly
/// inside one cut remnant. Holes that straddle or touch a cut boundary and
/// nonconvex multi-cutter outputs still need a full planar subdivision. This
/// preserves Yap's rule from "Towards Exact Geometric Computation,"
/// *Computational Geometry* 7.1-2 (1997): every promoted loop is justified by
/// exact source topology, containment, or area replay, and unsupported
/// combinatorics remain explicit.
#[cfg(feature = "exact-triangulation")]
pub fn arrange_coplanar_convex_surface_component_holed_difference(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Option<CoplanarConvexComponentHoledArrangement> {
    let left_components = connected_face_component_meshes(left)?;
    let right_components = connected_face_component_meshes(right)?;
    if right_components.is_empty() {
        return None;
    }

    let mut left_components = left_components
        .into_iter()
        .map(|mesh| ConvexUnionComponent::from_mesh(MultiUnionSide::Left, mesh))
        .collect::<Option<Vec<_>>>()?;
    let source_component_count = left_components.len();
    let right_components = right_components
        .into_iter()
        .map(|mesh| ConvexUnionComponent::from_mesh(MultiUnionSide::Right, mesh))
        .collect::<Option<Vec<_>>>()?;
    let projection = left_components.first()?.projection;
    if left_components
        .iter()
        .chain(right_components.iter())
        .any(|component| component.projection != projection)
    {
        return None;
    }

    let left_hulls = left_components
        .iter()
        .map(|component| component.hull.clone())
        .collect::<Vec<_>>();
    validate_component_loops_disjoint(
        &left_hulls,
        projection,
        "coplanar convex component-holed arrangement",
    )
    .ok()?;
    let right_hulls = right_components
        .iter()
        .map(|component| component.hull.clone())
        .collect::<Vec<_>>();
    validate_component_loops_disjoint(
        &right_hulls,
        projection,
        "coplanar convex component-holed arrangement",
    )
    .ok()?;

    let mut components = Vec::new();
    let mut emitted_cut = false;
    for component in &mut left_components {
        let mut dropped = false;
        let mut cut_indices = Vec::new();
        let mut holes = Vec::new();
        for (right_index, right_component) in right_components.iter().enumerate() {
            if polygons_equal(&component.hull, &right_component.hull)
                || polygon_in_closed_convex_polygon(
                    &component.hull,
                    &right_component.hull,
                    projection,
                )?
            {
                if dropped || !cut_indices.is_empty() || !holes.is_empty() {
                    return None;
                }
                dropped = true;
                continue;
            }
            if polygon_in_closed_convex_polygon(&right_component.hull, &component.hull, projection)?
            {
                if dropped {
                    return None;
                }
                let mut hole = right_component.hull.clone();
                orient_polygon_cw(&mut hole, projection)?;
                holes.push(hole);
                continue;
            }

            match convex_union_component_relation(
                &component.hull,
                &right_component.hull,
                projection,
            )? {
                ConvexUnionComponentRelation::Disjoint => {}
                ConvexUnionComponentRelation::BoundaryOnly => return None,
                ConvexUnionComponentRelation::PositiveArea => {
                    if dropped {
                        return None;
                    }
                    cut_indices.push(right_index);
                }
            }
        }

        if dropped {
            continue;
        }
        if !cut_indices.is_empty() {
            emitted_cut = true;
            let mut cut_polygons = match cut_indices.as_slice() {
                [right_index] => {
                    let right_component = &right_components[*right_index];
                    if let Some(difference) = arrange_coplanar_convex_surface_difference(
                        &component.mesh,
                        &right_component.mesh,
                    ) {
                        vec![difference.polygon]
                    } else if let Some(difference) =
                        arrange_coplanar_convex_surface_multi_difference_convex(
                            &component.mesh,
                            &right_component.mesh,
                        )
                    {
                        difference.polygons
                    } else {
                        return None;
                    }
                }
                _ => materialize_component_multi_cutter_difference(
                    component,
                    &cut_indices,
                    &right_components,
                    projection,
                )?,
            };
            for polygon in &mut cut_polygons {
                orient_polygon_ccw(polygon, projection)?;
            }
            let holes_by_cut =
                assign_holes_to_cut_component_outputs(&holes, &cut_polygons, projection)?;
            components.extend(
                cut_polygons
                    .into_iter()
                    .zip(holes_by_cut)
                    .map(|(outer, holes)| CoplanarConvexHoledComponent { outer, holes }),
            );
        } else {
            let mut outer = component.hull.clone();
            orient_polygon_ccw(&mut outer, projection)?;
            components.push(CoplanarConvexHoledComponent { outer, holes });
        }
    }
    if !emitted_cut && source_component_count < 2 {
        return None;
    }
    if components.is_empty()
        || !components
            .iter()
            .any(|component| !component.holes.is_empty())
    {
        return None;
    }
    sort_components_for_replay(&mut components, projection);
    let mesh = component_holed_components_to_earcut_open_mesh(&components, projection)?;
    let arrangement = CoplanarConvexComponentHoledArrangement {
        projection,
        components,
        mesh,
    };
    arrangement.validate().ok()?;
    Some(arrangement)
}

/// Assign retained hole rings to exact cut remnants.
///
/// A component that mixes holes with one partial cutter is still a bounded
/// arrangement: the cut itself is certified by the convex difference helper,
/// then each hole must be strictly inside exactly one emitted remnant. This
/// check is the local substitute for a full planar subdivision. Following Yap,
/// "Towards Exact Geometric Computation," *Computational Geometry* 7.1-2
/// (1997), ambiguous boundary contact or partial hole/remnant overlap returns
/// `None` rather than inventing topology from an approximate sample point.
#[cfg(feature = "exact-triangulation")]
fn assign_holes_to_cut_component_outputs(
    holes: &[Vec<Point3>],
    cut_polygons: &[Vec<Point3>],
    projection: CoplanarProjection,
) -> Option<Vec<Vec<Vec<Point3>>>> {
    let mut holes_by_cut = vec![Vec::new(); cut_polygons.len()];
    for hole in holes {
        let mut owner = None;
        for (index, polygon) in cut_polygons.iter().enumerate() {
            if polygon_strictly_inside_convex_polygon(hole, polygon, projection)? {
                if owner.is_some() {
                    return None;
                }
                owner = Some(index);
            }
        }
        holes_by_cut[owner?].push(hole.clone());
    }
    Some(holes_by_cut)
}

#[cfg(feature = "exact-triangulation")]
fn polygon_strictly_inside_convex_polygon(
    inner: &[Point3],
    outer: &[Point3],
    projection: CoplanarProjection,
) -> Option<bool> {
    if inner.len() < 3 || outer.len() < 3 {
        return Some(false);
    }
    for point in inner {
        if convex_polygon_location(point, outer, projection)? != ConvexPolygonLocation::Inside {
            return Some(false);
        }
    }
    Some(true)
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
        // Sutherland-Hodgman clipping traverses every edge of the clipping
        // polygon, not just triangle edges. Yap's retained-state discipline
        // requires the projected 2D edge used for the predicate to match the
        // exact 3D edge used for the construction; modulo `clip.len()` keeps
        // multi-face convex surface clips from replaying the wrong half-plane.
        // See Sutherland and Hodgman, "Reentrant Polygon Clipping,"
        // Communications of the ACM 17.1 (1974), and Yap, "Towards Exact
        // Geometric Computation," Computational Geometry 7.1-2 (1997).
        let b2 = &clip2[(edge + 1) % clip.len()];
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
    polygon_to_earcut_open_mesh_with_label(
        polygon,
        projection,
        "exact coplanar triangle planar arrangement",
    )
}

#[cfg(feature = "exact-triangulation")]
fn polygon_to_earcut_open_mesh_with_label(
    polygon: &[Point3],
    projection: CoplanarProjection,
    label: &'static str,
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
        SourceProvenance::exact(label),
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
fn polygon_to_earcut_open_mesh_with_holes(
    outer: &[Point3],
    holes: &[Vec<Point3>],
    projection: CoplanarProjection,
    label: &'static str,
) -> Option<ExactMesh> {
    if outer.len() < 3 || holes.len() < 2 || holes.iter().any(|hole| hole.len() < 3) {
        return None;
    }
    let mut points = outer.to_vec();
    let mut hole_indices = Vec::with_capacity(holes.len());
    for hole in holes {
        hole_indices.push(points.len());
        points.extend(hole.iter().cloned());
    }
    let vertices2 = points
        .iter()
        .map(|point| project_for_hypertri(point, projection))
        .collect::<Vec<_>>();
    let indices = hypertri::earcut(&vertices2, &hole_indices).ok()?;
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
        SourceProvenance::exact(label),
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .ok()
}

#[cfg(feature = "exact-triangulation")]
fn polygons_to_earcut_open_mesh(
    polygons: &[Vec<Point3>],
    projection: CoplanarProjection,
) -> Option<ExactMesh> {
    polygons_to_earcut_open_mesh_with_label(
        polygons,
        projection,
        "exact coplanar convex multi-component arrangement",
    )
}

#[cfg(feature = "exact-triangulation")]
fn polygons_to_earcut_open_mesh_with_label(
    polygons: &[Vec<Point3>],
    projection: CoplanarProjection,
    label: &'static str,
) -> Option<ExactMesh> {
    if polygons.is_empty() || polygons.iter().any(|polygon| polygon.len() < 3) {
        return None;
    }
    let mut vertices = Vec::new();
    let mut triangles = Vec::new();
    for polygon in polygons {
        let offset = vertices.len();
        let mesh = polygon_to_earcut_open_mesh(polygon, projection)?;
        vertices.extend(mesh.vertices().iter().cloned());
        triangles.extend(mesh.triangles().iter().map(|triangle| {
            let [a, b, c] = triangle.0;
            Triangle([a + offset, b + offset, c + offset])
        }));
    }
    ExactMesh::new_with_policy(
        vertices,
        triangles,
        SourceProvenance::exact(label),
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .ok()
}

#[cfg(feature = "exact-triangulation")]
fn component_holed_components_to_earcut_open_mesh(
    components: &[CoplanarConvexHoledComponent],
    projection: CoplanarProjection,
) -> Option<ExactMesh> {
    if components.is_empty()
        || components.iter().any(|component| {
            component.outer.len() < 3 || component.holes.iter().any(|h| h.len() < 3)
        })
    {
        return None;
    }
    let mut vertices = Vec::new();
    let mut triangles = Vec::new();
    for component in components {
        let offset = vertices.len();
        let mesh = component_holed_component_to_earcut_open_mesh(component, projection)?;
        vertices.extend(mesh.vertices().iter().cloned());
        triangles.extend(mesh.triangles().iter().map(|triangle| {
            let [a, b, c] = triangle.0;
            Triangle([a + offset, b + offset, c + offset])
        }));
    }
    ExactMesh::new_with_policy(
        vertices,
        triangles,
        SourceProvenance::exact("exact coplanar convex component-holed arrangement"),
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .ok()
}

#[cfg(feature = "exact-triangulation")]
fn component_holed_component_to_earcut_open_mesh(
    component: &CoplanarConvexHoledComponent,
    projection: CoplanarProjection,
) -> Option<ExactMesh> {
    match component.holes.len() {
        0 => polygon_to_earcut_open_mesh(&component.outer, projection),
        1 => polygon_to_earcut_open_mesh_with_hole(
            &component.outer,
            component.holes.first()?,
            projection,
        ),
        _ => polygon_to_earcut_open_mesh_with_holes(
            &component.outer,
            &component.holes,
            projection,
            "exact coplanar convex component-holed sub-arrangement",
        ),
    }
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
    validate_exact_point_set_distinct(polygon, label, "surface polygon repeats an exact point")?;
    validate_projected_simple_loop(polygon, projection, label)?;
    // The retained simple-loop artifact is an oriented boundary, not merely an
    // unordered set with a positive absolute area. Requiring counter-clockwise
    // order keeps the output compatible with the signed-area and boundary
    // replay used by later planar-cell and winding stages. This is the same
    // retained structural-information principle Yap advocates in "Towards
    // Exact Geometric Computation," Computational Geometry 7.1-2 (1997).
    validate_projected_ring_orientation(
        polygon,
        projection,
        Sign::Positive,
        label,
        "surface polygon orientation must be counter-clockwise",
    )?;
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
    let retained_rings = core::iter::once(0..polygon.len()).collect::<Vec<_>>();
    validate_mesh_edges_respect_retained_rings(
        mesh,
        projection,
        &retained_rings,
        label,
        "surface mesh edge crosses the retained polygon boundary",
    )?;
    validate_mesh_uses_all_retained_vertices(
        mesh,
        polygon.len(),
        label,
        "surface mesh leaves a retained polygon vertex unused",
    )?;
    validate_mesh_boundary_matches_retained_rings(
        mesh,
        &retained_rings,
        label,
        "surface mesh boundary does not match retained polygon boundary",
    )?;
    let mesh_area = projected_mesh_area2_abs(mesh, projection).ok_or_else(|| {
        surface_validation_error(label, "surface mesh projected area was undecided")
    })?;
    if compare_reals(&mesh_area, &area).value() != Some(Ordering::Equal) {
        return Err(surface_validation_error(
            label,
            "surface mesh projected area does not match retained polygon area",
        ));
    }
    let retained_signed_area = projected_area2_signed(polygon, projection).ok_or_else(|| {
        surface_validation_error(label, "surface polygon signed area was undecided")
    })?;
    let mesh_signed_area = projected_mesh_area2_signed(mesh, projection).ok_or_else(|| {
        surface_validation_error(label, "surface mesh signed projected area was undecided")
    })?;
    if compare_reals(&mesh_signed_area, &retained_signed_area).value() != Some(Ordering::Equal) {
        return Err(surface_validation_error(
            label,
            "surface mesh signed projected area does not match retained polygon orientation",
        ));
    }

    mesh.validate_retained_state().map_err(|_| {
        surface_validation_error(label, "materialized mesh retained-state validation failed")
    })
}

#[cfg(feature = "exact-triangulation")]
fn validate_multi_surface_output(
    projection: CoplanarProjection,
    polygons: &[Vec<Point3>],
    mesh: &ExactMesh,
    label: &'static str,
) -> Result<(), MeshError> {
    validate_multi_surface_output_with_loop_policy(projection, polygons, mesh, label, true)
}

#[cfg(feature = "exact-triangulation")]
fn validate_multi_simple_surface_output(
    projection: CoplanarProjection,
    polygons: &[Vec<Point3>],
    mesh: &ExactMesh,
    label: &'static str,
) -> Result<(), MeshError> {
    validate_multi_surface_output_with_loop_policy(projection, polygons, mesh, label, false)
}

#[cfg(feature = "exact-triangulation")]
fn validate_multi_surface_output_with_loop_policy(
    projection: CoplanarProjection,
    polygons: &[Vec<Point3>],
    mesh: &ExactMesh,
    label: &'static str,
    require_strict_convex: bool,
) -> Result<(), MeshError> {
    if polygons.len() < 2 {
        return Err(surface_validation_error(
            label,
            "multi-component surface must retain at least two loops",
        ));
    }
    if mesh.triangles().is_empty() {
        return Err(surface_validation_error(
            label,
            "multi-component surface mesh has no triangulation",
        ));
    }
    let expected_vertices = polygons.iter().map(Vec::len).sum::<usize>();
    if mesh.vertices().len() != expected_vertices {
        return Err(surface_validation_error(
            label,
            "mesh vertex count does not match retained component loops",
        ));
    }
    let mut component_ranges = Vec::with_capacity(polygons.len());
    let mut range_start = 0;
    for polygon in polygons {
        let range_end = range_start + polygon.len();
        component_ranges.push(range_start..range_end);
        range_start = range_end;
    }

    let mut retained_area = ExactReal::from(0);
    let retained_points = polygons
        .iter()
        .flat_map(|polygon| polygon.iter())
        .collect::<Vec<_>>();
    for (component, polygon) in polygons.iter().enumerate() {
        if polygon.len() < 3 {
            return Err(surface_validation_error(
                label,
                "component loop has fewer than three vertices",
            ));
        }
        validate_exact_point_set_distinct(polygon, label, "component loop repeats an exact point")?;
        let area = projected_area2_abs(polygon, projection).ok_or_else(|| {
            surface_validation_error(label, "component projected area was undecided")
        })?;
        if compare_reals(&area, &ExactReal::from(0)).value() != Some(Ordering::Greater) {
            return Err(surface_validation_error(
                label,
                "component loop has zero projected area",
            ));
        }
        validate_projected_ring_orientation(
            polygon,
            projection,
            Sign::Positive,
            label,
            "component loop orientation must be counter-clockwise",
        )?;
        validate_projected_simple_loop(polygon, projection, label)?;
        if require_strict_convex {
            validate_projected_strictly_convex_loop(polygon, projection, label)?;
        }
        retained_area = add(&retained_area, &area);
        for point in polygon {
            for other_polygon in polygons.iter().skip(component + 1) {
                for other in other_polygon {
                    if points_equal(point, other) {
                        return Err(surface_validation_error(
                            label,
                            "component loops share an exact point",
                        ));
                    }
                }
            }
        }
    }
    validate_component_loops_disjoint(polygons, projection, label)?;

    let mut vertex_offset = 0;
    for polygon in polygons {
        for (local_index, point) in polygon.iter().enumerate() {
            let mesh_point = mesh.vertices()[vertex_offset + local_index].to_hyperlimit_point();
            if !points_equal(point, &mesh_point) {
                return Err(surface_validation_error(
                    label,
                    "mesh vertex does not match retained component point",
                ));
            }
        }
        vertex_offset += polygon.len();
    }
    for triangle in mesh.triangles() {
        if triangle.0.iter().any(|&index| index >= expected_vertices) {
            return Err(surface_validation_error(
                label,
                "multi-component mesh triangle index is out of retained loop range",
            ));
        }
        if triangle
            .0
            .iter()
            .filter_map(|&index| component_for_retained_vertex(index, &component_ranges))
            .any(|component| {
                component
                    != component_for_retained_vertex(triangle.0[0], &component_ranges).unwrap()
            })
        {
            return Err(surface_validation_error(
                label,
                "multi-component mesh triangle spans retained component loops",
            ));
        }
    }
    validate_mesh_edges_respect_retained_rings(
        mesh,
        projection,
        &component_ranges,
        label,
        "multi-component mesh edge crosses a retained component loop",
    )?;
    validate_mesh_uses_all_retained_vertices(
        mesh,
        expected_vertices,
        label,
        "multi-component mesh leaves a retained loop vertex unused",
    )?;
    validate_mesh_boundary_matches_retained_rings(
        mesh,
        &component_ranges,
        label,
        "multi-component mesh boundary does not match retained component loops",
    )?;
    let mesh_area = projected_mesh_area2_abs(mesh, projection).ok_or_else(|| {
        surface_validation_error(label, "multi-component mesh projected area was undecided")
    })?;
    if compare_reals(&mesh_area, &retained_area).value() != Some(Ordering::Equal) {
        return Err(surface_validation_error(
            label,
            "multi-component mesh projected area does not match retained loop area",
        ));
    }
    let retained_signed_area = polygons
        .iter()
        .try_fold(ExactReal::from(0), |area, polygon| {
            Some(add(&area, &projected_area2_signed(polygon, projection)?))
        });
    let retained_signed_area = retained_signed_area.ok_or_else(|| {
        surface_validation_error(label, "multi-component retained signed area was undecided")
    })?;
    let mesh_signed_area = projected_mesh_area2_signed(mesh, projection).ok_or_else(|| {
        surface_validation_error(label, "multi-component mesh signed area was undecided")
    })?;
    if compare_reals(&mesh_signed_area, &retained_signed_area).value() != Some(Ordering::Equal) {
        return Err(surface_validation_error(
            label,
            "multi-component mesh signed area does not match retained loop orientation",
        ));
    }
    for left in 0..retained_points.len() {
        for right in left + 1..retained_points.len() {
            if points_equal(retained_points[left], retained_points[right]) {
                return Err(surface_validation_error(
                    label,
                    "multi-component retained loops repeat an exact point",
                ));
            }
        }
    }
    mesh.validate_retained_state().map_err(|_| {
        surface_validation_error(label, "materialized mesh retained-state validation failed")
    })
}

#[cfg(feature = "exact-triangulation")]
fn validate_component_holed_surface_output(
    projection: CoplanarProjection,
    components: &[CoplanarConvexHoledComponent],
    mesh: &ExactMesh,
    label: &'static str,
) -> Result<(), MeshError> {
    if components.is_empty()
        || !components
            .iter()
            .any(|component| !component.holes.is_empty())
    {
        return Err(surface_validation_error(
            label,
            "component-holed surface must retain at least one component and one hole",
        ));
    }
    if mesh.triangles().is_empty() {
        return Err(surface_validation_error(
            label,
            "component-holed surface mesh has no triangulation",
        ));
    }

    let mut component_ranges = Vec::with_capacity(components.len());
    let mut retained_rings = Vec::new();
    let mut hole_ranges = Vec::new();
    let mut expected_vertices = 0;
    let mut retained_area = ExactReal::from(0);
    let mut retained_signed_area = ExactReal::from(0);
    let mut outers = Vec::with_capacity(components.len());

    for component in components {
        if component.outer.len() < 3 || component.holes.iter().any(|hole| hole.len() < 3) {
            return Err(surface_validation_error(
                label,
                "component rings must contain at least three vertices",
            ));
        }
        let component_start = expected_vertices;
        let outer_start = expected_vertices;
        expected_vertices += component.outer.len();
        retained_rings.push(outer_start..expected_vertices);
        validate_exact_point_set_distinct(
            &component.outer,
            label,
            "component outer ring repeats an exact point",
        )?;
        validate_projected_simple_loop(&component.outer, projection, label)?;
        validate_projected_strictly_convex_loop(&component.outer, projection, label)?;
        validate_projected_ring_orientation(
            &component.outer,
            projection,
            Sign::Positive,
            label,
            "component outer ring orientation must be counter-clockwise",
        )?;
        let outer_area = projected_area2_abs(&component.outer, projection).ok_or_else(|| {
            surface_validation_error(label, "component outer projected area was undecided")
        })?;
        let outer_signed =
            projected_area2_signed(&component.outer, projection).ok_or_else(|| {
                surface_validation_error(label, "component outer signed area was undecided")
            })?;
        if compare_reals(&outer_area, &ExactReal::from(0)).value() != Some(Ordering::Greater) {
            return Err(surface_validation_error(
                label,
                "component outer ring has zero projected area",
            ));
        }

        let mut hole_area_sum = ExactReal::from(0);
        let mut component_signed_area = outer_signed;
        for hole in &component.holes {
            let hole_start = expected_vertices;
            expected_vertices += hole.len();
            retained_rings.push(hole_start..expected_vertices);
            hole_ranges.push(hole_start..expected_vertices);
            validate_exact_point_set_distinct(
                hole,
                label,
                "component hole repeats an exact point",
            )?;
            validate_projected_simple_loop(hole, projection, label)?;
            validate_projected_strictly_convex_loop(hole, projection, label)?;
            validate_projected_ring_orientation(
                hole,
                projection,
                Sign::Negative,
                label,
                "component hole orientation must be clockwise",
            )?;
            let hole_area = projected_area2_abs(hole, projection).ok_or_else(|| {
                surface_validation_error(label, "component hole projected area was undecided")
            })?;
            let hole_signed = projected_area2_signed(hole, projection).ok_or_else(|| {
                surface_validation_error(label, "component hole signed area was undecided")
            })?;
            if compare_reals(&hole_area, &ExactReal::from(0)).value() != Some(Ordering::Greater) {
                return Err(surface_validation_error(
                    label,
                    "component hole has zero projected area",
                ));
            }
            for point in hole {
                match convex_polygon_location(point, &component.outer, projection) {
                    Some(ConvexPolygonLocation::Inside) => {}
                    Some(ConvexPolygonLocation::Boundary | ConvexPolygonLocation::Outside) => {
                        return Err(surface_validation_error(
                            label,
                            "component hole must lie strictly inside its outer ring",
                        ));
                    }
                    None => {
                        return Err(surface_validation_error(
                            label,
                            "component hole containment predicate was undecided",
                        ));
                    }
                }
            }
            hole_area_sum = add(&hole_area_sum, &hole_area);
            component_signed_area = add(&component_signed_area, &hole_signed);
        }
        if component.holes.len() > 1 {
            validate_component_loops_disjoint(&component.holes, projection, label)?;
        }
        if compare_reals(&outer_area, &hole_area_sum).value() != Some(Ordering::Greater) {
            return Err(surface_validation_error(
                label,
                "component hole area must be strictly smaller than outer area",
            ));
        }
        retained_area = add(&retained_area, &sub(&outer_area, &hole_area_sum));
        retained_signed_area = add(&retained_signed_area, &component_signed_area);
        component_ranges.push(component_start..expected_vertices);
        outers.push(component.outer.clone());
    }
    validate_component_loops_disjoint(&outers, projection, label)?;

    if mesh.vertices().len() != expected_vertices {
        return Err(surface_validation_error(
            label,
            "mesh vertex count does not match retained component-holed rings",
        ));
    }
    let retained_points = components
        .iter()
        .flat_map(|component| {
            component
                .outer
                .iter()
                .chain(component.holes.iter().flatten())
        })
        .collect::<Vec<_>>();
    for (index, point) in retained_points.iter().enumerate() {
        if !points_equal(point, &mesh.vertices()[index].to_hyperlimit_point()) {
            return Err(surface_validation_error(
                label,
                "mesh vertex does not match retained component-holed point",
            ));
        }
    }
    for triangle in mesh.triangles() {
        if triangle.0.iter().any(|&index| index >= expected_vertices) {
            return Err(surface_validation_error(
                label,
                "component-holed mesh triangle index is out of retained ring range",
            ));
        }
        let first_component = component_for_retained_vertex(triangle.0[0], &component_ranges)
            .ok_or_else(|| {
                surface_validation_error(label, "triangle vertex has no retained component")
            })?;
        if triangle.0.iter().any(|&index| {
            component_for_retained_vertex(index, &component_ranges) != Some(first_component)
        }) {
            return Err(surface_validation_error(
                label,
                "component-holed mesh triangle spans retained components",
            ));
        }
        for hole_range in &hole_ranges {
            if triangle.0.iter().all(|index| hole_range.contains(index)) {
                return Err(surface_validation_error(
                    label,
                    "component-holed mesh triangle fills a retained hole",
                ));
            }
        }
    }
    validate_mesh_edges_respect_retained_rings(
        mesh,
        projection,
        &retained_rings,
        label,
        "component-holed mesh edge crosses a retained ring",
    )?;
    validate_mesh_uses_all_retained_vertices(
        mesh,
        expected_vertices,
        label,
        "component-holed mesh leaves a retained ring vertex unused",
    )?;
    validate_mesh_boundary_matches_retained_rings(
        mesh,
        &retained_rings,
        label,
        "component-holed mesh boundary does not match retained rings",
    )?;
    let mesh_area = projected_mesh_area2_abs(mesh, projection).ok_or_else(|| {
        surface_validation_error(label, "component-holed mesh projected area was undecided")
    })?;
    if compare_reals(&mesh_area, &retained_area).value() != Some(Ordering::Equal) {
        return Err(surface_validation_error(
            label,
            "component-holed mesh projected area does not match retained rings",
        ));
    }
    let mesh_signed_area = projected_mesh_area2_signed(mesh, projection).ok_or_else(|| {
        surface_validation_error(label, "component-holed mesh signed area was undecided")
    })?;
    if compare_reals(&mesh_signed_area, &retained_signed_area).value() != Some(Ordering::Equal) {
        return Err(surface_validation_error(
            label,
            "component-holed mesh signed area does not match retained ring orientation",
        ));
    }
    mesh.validate_retained_state().map_err(|_| {
        surface_validation_error(label, "materialized mesh retained-state validation failed")
    })
}

/// Return the retained component that owns a mesh vertex index.
///
/// Multi-component planar-arrangement artifacts are not just bags of triangles:
/// each output component retains its own loop and its own triangulation. Yap,
/// "Towards Exact Geometric Computation," *Computational Geometry* 7.1-2
/// (1997), frames this as retained numerical/combinatorial structure, so a
/// triangle may only reference vertices from one retained component. Otherwise
/// a later consumer could observe topology that was never certified by the
/// component loop predicates.
#[cfg(feature = "exact-triangulation")]
fn component_for_retained_vertex(
    index: usize,
    ranges: &[core::ops::Range<usize>],
) -> Option<usize> {
    ranges
        .iter()
        .position(|range| range.start <= index && index < range.end)
}

/// Validate that every retained output vertex participates in triangulation.
///
/// Retained boundary vertices are part of the certified output topology, not
/// spare coordinates attached to a triangle soup. Following Yap, "Towards
/// Exact Geometric Computation," *Computational Geometry* 7.1-2 (1997), the
/// materialized mesh must consume every retained vertex before area or winding
/// summaries are allowed to stand in for the object. This catches isolated
/// ring vertices directly at the API boundary instead of relying on aggregate
/// area replay to notice missing topology.
fn validate_mesh_uses_all_retained_vertices(
    mesh: &ExactMesh,
    retained_vertices: usize,
    label: &'static str,
    message: &'static str,
) -> Result<(), MeshError> {
    let mut used = vec![false; retained_vertices];
    for triangle in mesh.triangles() {
        for &index in &triangle.0 {
            if let Some(slot) = used.get_mut(index) {
                *slot = true;
            }
        }
    }
    if used.iter().any(|used| !*used) {
        return Err(surface_validation_error(label, message));
    }
    Ok(())
}

/// Validate that the mesh boundary is exactly the retained ring boundary.
///
/// A materialized surface is a triangulation of retained loops, not merely a
/// triangle soup with matching area. Following Yap, "Towards Exact Geometric
/// Computation," *Computational Geometry* 7.1-2 (1997), the retained
/// combinatorial structure must be replayable at the public artifact
/// boundary. We therefore compare the undirected boundary edges used by the
/// triangle mesh against the retained consecutive ring edges before signed
/// area summaries are allowed to certify the output. The boundary-as-chain
/// viewpoint is the standard planar subdivision invariant; see de Berg,
/// Cheong, van Kreveld, and Overmars, *Computational Geometry: Algorithms and
/// Applications*, 3rd ed. (2008), Chapter 2.
fn validate_mesh_boundary_matches_retained_rings(
    mesh: &ExactMesh,
    retained_rings: &[core::ops::Range<usize>],
    label: &'static str,
    message: &'static str,
) -> Result<(), MeshError> {
    let mut expected = Vec::new();
    for ring in retained_rings {
        let len = ring.end.saturating_sub(ring.start);
        if len < 3 {
            return Err(surface_validation_error(
                label,
                "retained boundary ring has fewer than three vertices",
            ));
        }
        for local in 0..len {
            expected.push(canonical_edge(
                ring.start + local,
                ring.start + ((local + 1) % len),
            ));
        }
    }
    expected.sort_unstable();
    if expected.windows(2).any(|window| window[0] == window[1]) {
        return Err(surface_validation_error(
            label,
            "retained boundary rings repeat an edge",
        ));
    }

    let mut edge_counts: Vec<((usize, usize), usize)> = Vec::new();
    for triangle in mesh.triangles() {
        let [a, b, c] = triangle.0;
        for edge in [
            canonical_edge(a, b),
            canonical_edge(b, c),
            canonical_edge(c, a),
        ] {
            if let Some((_, count)) = edge_counts.iter_mut().find(|(key, _)| *key == edge) {
                *count += 1;
            } else {
                edge_counts.push((edge, 1));
            }
        }
    }
    let mut actual = edge_counts
        .into_iter()
        .filter_map(|(edge, count)| (count == 1).then_some(edge))
        .collect::<Vec<_>>();
    actual.sort_unstable();

    if actual != expected {
        return Err(surface_validation_error(label, message));
    }
    Ok(())
}

fn canonical_edge(a: usize, b: usize) -> (usize, usize) {
    if a <= b { (a, b) } else { (b, a) }
}

/// Validate mesh edges against retained ring edges.
///
/// Retained loops/rings are exact topological constraints on the triangulated
/// surface. Yap, "Towards Exact Geometric Computation," *Computational
/// Geometry* 7.1-2 (1997), argues for preserving such numerical structural
/// information instead of reconstructing it from rounded output geometry. We
/// therefore reject any materialized triangle edge whose exact projected
/// segment crosses, overlaps, or touches a retained ring edge away from a
/// shared endpoint or identical retained boundary edge. The segment predicate
/// is the orientation-based closed-segment classifier used by Guigue and
/// Devillers, "Fast and Robust Triangle-Triangle Overlap Test Using
/// Orientation Predicates," *Journal of Graphics Tools* 8.1 (2003).
fn validate_mesh_edges_respect_retained_rings(
    mesh: &ExactMesh,
    projection: CoplanarProjection,
    retained_rings: &[core::ops::Range<usize>],
    label: &'static str,
    message: &'static str,
) -> Result<(), MeshError> {
    let retained_edges = retained_ring_edges(retained_rings, label)?;
    let mut mesh_edges = Vec::new();
    for triangle in mesh.triangles() {
        let [a, b, c] = triangle.0;
        mesh_edges.extend([
            canonical_edge(a, b),
            canonical_edge(b, c),
            canonical_edge(c, a),
        ]);
    }
    mesh_edges.sort_unstable();
    mesh_edges.dedup();

    for &(edge_a, edge_b) in &mesh_edges {
        for &(ring_a, ring_b) in &retained_edges {
            if canonical_edge(edge_a, edge_b) == canonical_edge(ring_a, ring_b) {
                continue;
            }
            let edge_start = mesh.vertices()[edge_a].to_hyperlimit_point();
            let edge_end = mesh.vertices()[edge_b].to_hyperlimit_point();
            let ring_start = mesh.vertices()[ring_a].to_hyperlimit_point();
            let ring_end = mesh.vertices()[ring_b].to_hyperlimit_point();
            match classify_segment_intersection(
                &project_point(&edge_start, projection),
                &project_point(&edge_end, projection),
                &project_point(&ring_start, projection),
                &project_point(&ring_end, projection),
            )
            .value()
            {
                Some(SegmentIntersection::Disjoint) => {}
                Some(SegmentIntersection::EndpointTouch)
                    if edge_a == ring_a
                        || edge_a == ring_b
                        || edge_b == ring_a
                        || edge_b == ring_b => {}
                Some(
                    SegmentIntersection::Proper
                    | SegmentIntersection::EndpointTouch
                    | SegmentIntersection::CollinearOverlap
                    | SegmentIntersection::Identical,
                ) => {
                    return Err(surface_validation_error(label, message));
                }
                None => {
                    return Err(surface_validation_error(
                        label,
                        "mesh edge/ring intersection predicate was undecided",
                    ));
                }
            }
        }
    }
    Ok(())
}

fn retained_ring_edges(
    retained_rings: &[core::ops::Range<usize>],
    label: &'static str,
) -> Result<Vec<(usize, usize)>, MeshError> {
    let mut edges = Vec::new();
    for ring in retained_rings {
        let len = ring.end.saturating_sub(ring.start);
        if len < 3 {
            return Err(surface_validation_error(
                label,
                "retained boundary ring has fewer than three vertices",
            ));
        }
        for local in 0..len {
            edges.push((ring.start + local, ring.start + ((local + 1) % len)));
        }
    }
    Ok(edges)
}

/// Validate that a retained projected loop has no non-adjacent edge contacts.
///
/// A retained boundary loop is part of the exact combinatorial state, not a
/// display hint. Following Yap, "Towards Exact Geometric Computation,"
/// *Computational Geometry* 7.1-2 (1997), the loop is accepted only when exact
/// predicates certify that non-adjacent closed edges are disjoint. The segment
/// relation is supplied by `hyperlimit`'s orientation-predicate classifier,
/// following Guigue and Devillers, "Fast and Robust Triangle-Triangle Overlap
/// Test Using Orientation Predicates," *Journal of Graphics Tools* 8.1 (2003).
fn validate_projected_simple_loop(
    polygon: &[Point3],
    projection: CoplanarProjection,
    label: &'static str,
) -> Result<(), MeshError> {
    for left_edge in 0..polygon.len() {
        let left_next = (left_edge + 1) % polygon.len();
        let left_start = project_point(&polygon[left_edge], projection);
        let left_end = project_point(&polygon[left_next], projection);
        for right_edge in left_edge + 1..polygon.len() {
            let right_next = (right_edge + 1) % polygon.len();
            if left_next == right_edge || right_next == left_edge {
                continue;
            }
            let right_start = project_point(&polygon[right_edge], projection);
            let right_end = project_point(&polygon[right_next], projection);
            match classify_segment_intersection(&left_start, &left_end, &right_start, &right_end)
                .value()
            {
                Some(SegmentIntersection::Disjoint) => {}
                Some(
                    SegmentIntersection::Proper
                    | SegmentIntersection::EndpointTouch
                    | SegmentIntersection::CollinearOverlap
                    | SegmentIntersection::Identical,
                ) => {
                    return Err(surface_validation_error(
                        label,
                        "retained loop has non-adjacent edge contact",
                    ));
                }
                None => {
                    return Err(surface_validation_error(
                        label,
                        "retained loop simplicity predicate was undecided",
                    ));
                }
            }
        }
    }
    Ok(())
}

/// Validate the signed orientation of a retained projected ring.
///
/// Ring direction is part of the combinatorial certificate for planar
/// arrangement artifacts: outer/component loops are retained counter-clockwise
/// and holes clockwise before `hypertri` receives them. Rechecking that fact
/// at the public artifact boundary prevents a caller from reversing a ring
/// while leaving the triangle soup untouched. This follows Yap, "Towards
/// Exact Geometric Computation," *Computational Geometry* 7.1-2 (1997):
/// structural numerical facts retained during construction must be replayable
/// before downstream topology consumes the object. The signed area is the
/// standard shoelace determinant; see Preparata and Shamos, *Computational
/// Geometry: An Introduction* (1985), Chapter 1.
fn validate_projected_ring_orientation(
    polygon: &[Point3],
    projection: CoplanarProjection,
    expected: Sign,
    label: &'static str,
    message: &'static str,
) -> Result<(), MeshError> {
    let area = projected_area2_signed(polygon, projection).ok_or_else(|| {
        surface_validation_error(label, "retained ring orientation was undecided")
    })?;
    let expected_ordering = match expected {
        Sign::Positive => Ordering::Greater,
        Sign::Negative => Ordering::Less,
        Sign::Zero => Ordering::Equal,
    };
    if compare_reals(&area, &ExactReal::from(0)).value() != Some(expected_ordering) {
        return Err(surface_validation_error(label, message));
    }
    Ok(())
}

/// Validate that a retained boundary is a strictly convex projected loop.
///
/// The convex coplanar arrangement shortcuts are intentionally bounded to
/// convex retained boundaries. Their later containment checks use exact
/// half-plane tests for convex polygons, so the convexity precondition is
/// certified at the retained artifact boundary. This follows Yap, "Towards
/// Exact Geometric Computation," *Computational Geometry* 7.1-2 (1997):
/// structural preconditions become explicit checked facts before another
/// certified predicate is allowed to consume them.
fn validate_projected_strictly_convex_loop(
    polygon: &[Point3],
    projection: CoplanarProjection,
    label: &'static str,
) -> Result<(), MeshError> {
    let mut ring = polygon.to_vec();
    orient_polygon_ccw(&mut ring, projection).ok_or_else(|| {
        surface_validation_error(label, "component loop orientation was undecided")
    })?;
    for index in 0..ring.len() {
        let previous = project_point(&ring[(index + ring.len() - 1) % ring.len()], projection);
        let current = project_point(&ring[index], projection);
        let next = project_point(&ring[(index + 1) % ring.len()], projection);
        match orient2d_report(&previous, &current, &next).value() {
            Some(Sign::Positive) => {}
            Some(Sign::Zero) => {
                return Err(surface_validation_error(
                    label,
                    "retained loop has a collinear projected corner",
                ));
            }
            Some(Sign::Negative) => {
                return Err(surface_validation_error(
                    label,
                    "retained loop is not strictly convex",
                ));
            }
            None => {
                return Err(surface_validation_error(
                    label,
                    "retained loop convexity predicate was undecided",
                ));
            }
        }
    }
    Ok(())
}

/// Validate a retained convex hull loop used by a shortcut certificate.
///
/// Convex-surface certificates expose hull loops as durable exact facts. A
/// caller can serialize or clone those certificates, so validation must replay
/// the hull's exact point distinctness, orientation, simplicity, and strict
/// convexity rather than accepting an area scalar by itself. This is the
/// certificate-side analogue of Yap's exact-geometric-computation boundary:
/// see Yap, "Towards Exact Geometric Computation," *Computational Geometry*
/// 7.1-2 (1997).
fn validate_retained_convex_hull(
    label: &'static str,
    hull: &[Point3],
    projection: CoplanarProjection,
) -> Result<(), MeshError> {
    if hull.len() < 3 {
        return Err(surface_validation_error(
            label,
            "retained hull has fewer than three vertices",
        ));
    }
    validate_exact_point_set_distinct(hull, label, "retained hull repeats an exact point")?;
    validate_projected_simple_loop(hull, projection, label)?;
    validate_projected_ring_orientation(
        hull,
        projection,
        Sign::Positive,
        label,
        "retained hull orientation must be counter-clockwise",
    )?;
    validate_projected_strictly_convex_loop(hull, projection, label)
}

/// Validate that retained convex components are pairwise separated.
///
/// Yap's exact-geometric-computation model treats the combinatorial structure
/// as the artifact being certified, not a byproduct of rounded coordinates:
/// see Yap, "Towards Exact Geometric Computation," *Computational Geometry*
/// 7.1-2 (1997). Multi-component convex differences therefore retain each
/// component only after exact projected segment tests and exact convex
/// containment tests prove that no two loops cross, touch, or nest. The
/// segment predicate is the orientation-based closed-segment classifier used
/// by Guigue and Devillers, "Fast and Robust Triangle-Triangle Overlap Test
/// Using Orientation Predicates," *Journal of Graphics Tools* 8.1 (2003).
#[cfg(feature = "exact-triangulation")]
fn validate_component_loops_disjoint(
    polygons: &[Vec<Point3>],
    projection: CoplanarProjection,
    label: &'static str,
) -> Result<(), MeshError> {
    for left_index in 0..polygons.len() {
        for right_index in left_index + 1..polygons.len() {
            let left = &polygons[left_index];
            let right = &polygons[right_index];

            for point in left {
                match convex_polygon_location(point, right, projection) {
                    Some(ConvexPolygonLocation::Outside) => {}
                    Some(ConvexPolygonLocation::Boundary | ConvexPolygonLocation::Inside) => {
                        return Err(surface_validation_error(
                            label,
                            "component loops overlap, touch, or nest",
                        ));
                    }
                    None => {
                        return Err(surface_validation_error(
                            label,
                            "component loop containment predicate was undecided",
                        ));
                    }
                }
            }
            for point in right {
                match convex_polygon_location(point, left, projection) {
                    Some(ConvexPolygonLocation::Outside) => {}
                    Some(ConvexPolygonLocation::Boundary | ConvexPolygonLocation::Inside) => {
                        return Err(surface_validation_error(
                            label,
                            "component loops overlap, touch, or nest",
                        ));
                    }
                    None => {
                        return Err(surface_validation_error(
                            label,
                            "component loop containment predicate was undecided",
                        ));
                    }
                }
            }

            for left_edge in 0..left.len() {
                let left_start = project_point(&left[left_edge], projection);
                let left_end = project_point(&left[(left_edge + 1) % left.len()], projection);
                for right_edge in 0..right.len() {
                    let right_start = project_point(&right[right_edge], projection);
                    let right_end =
                        project_point(&right[(right_edge + 1) % right.len()], projection);
                    match classify_segment_intersection(
                        &left_start,
                        &left_end,
                        &right_start,
                        &right_end,
                    )
                    .value()
                    {
                        Some(SegmentIntersection::Disjoint) => {}
                        Some(
                            SegmentIntersection::Proper
                            | SegmentIntersection::EndpointTouch
                            | SegmentIntersection::CollinearOverlap
                            | SegmentIntersection::Identical,
                        ) => {
                            return Err(surface_validation_error(
                                label,
                                "component loop edges intersect or touch",
                            ));
                        }
                        None => {
                            return Err(surface_validation_error(
                                label,
                                "component loop segment-intersection predicate was undecided",
                            ));
                        }
                    }
                }
            }
        }
    }
    Ok(())
}

fn validate_exact_point_set_distinct(
    points: &[Point3],
    label: &'static str,
    message: &'static str,
) -> Result<(), MeshError> {
    for left in 0..points.len() {
        for right in left + 1..points.len() {
            if points_equal(&points[left], &points[right]) {
                return Err(surface_validation_error(label, message));
            }
        }
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
    if mesh.triangles().is_empty() {
        return Err(surface_validation_error(
            label,
            "holed surface mesh has no triangulation",
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
    for triangle in mesh.triangles() {
        if triangle
            .0
            .iter()
            .any(|&index| index >= outer.len() + hole.len())
        {
            return Err(surface_validation_error(
                label,
                "holed surface mesh triangle index is out of retained ring range",
            ));
        }
        validate_holed_mesh_triangle_uses_outer_ring(triangle.0, outer.len(), label)?;
    }
    validate_mesh_edges_respect_retained_rings(
        mesh,
        projection,
        &[0..outer.len(), outer.len()..outer.len() + hole.len()],
        label,
        "holed surface mesh edge crosses a retained ring",
    )?;
    validate_mesh_uses_all_retained_vertices(
        mesh,
        outer.len() + hole.len(),
        label,
        "holed surface mesh leaves a retained ring vertex unused",
    )?;
    validate_mesh_boundary_matches_retained_rings(
        mesh,
        &[0..outer.len(), outer.len()..outer.len() + hole.len()],
        label,
        "holed surface mesh boundary does not match retained rings",
    )?;
    validate_exact_point_set_distinct(outer, label, "outer ring repeats an exact point")?;
    validate_exact_point_set_distinct(hole, label, "hole ring repeats an exact point")?;
    validate_projected_simple_loop(outer, projection, label)?;
    validate_projected_simple_loop(hole, projection, label)?;
    validate_projected_strictly_convex_loop(outer, projection, label)?;
    validate_projected_strictly_convex_loop(hole, projection, label)?;
    for outer_point in outer {
        for hole_point in hole {
            if points_equal(outer_point, hole_point) {
                return Err(surface_validation_error(
                    label,
                    "outer and hole rings share an exact point",
                ));
            }
        }
    }
    let outer_area = projected_area2_abs(outer, projection)
        .ok_or_else(|| surface_validation_error(label, "outer projected area was undecided"))?;
    let hole_area = projected_area2_abs(hole, projection)
        .ok_or_else(|| surface_validation_error(label, "hole projected area was undecided"))?;
    if compare_reals(&outer_area, &ExactReal::from(0)).value() != Some(Ordering::Greater)
        || compare_reals(&hole_area, &ExactReal::from(0)).value() != Some(Ordering::Greater)
    {
        return Err(surface_validation_error(
            label,
            "outer and hole rings must both have positive projected area",
        ));
    }
    validate_projected_ring_orientation(
        outer,
        projection,
        Sign::Positive,
        label,
        "outer ring orientation must be counter-clockwise",
    )?;
    validate_projected_ring_orientation(
        hole,
        projection,
        Sign::Negative,
        label,
        "hole ring orientation must be clockwise",
    )?;
    if compare_reals(&outer_area, &hole_area).value() != Some(Ordering::Greater) {
        return Err(surface_validation_error(
            label,
            "hole area must be strictly smaller than outer area",
        ));
    }
    let mesh_area = projected_mesh_area2_abs(mesh, projection).ok_or_else(|| {
        surface_validation_error(label, "holed surface mesh projected area was undecided")
    })?;
    let expected_area = sub(&outer_area, &hole_area);
    if compare_reals(&mesh_area, &expected_area).value() != Some(Ordering::Equal) {
        return Err(surface_validation_error(
            label,
            "holed surface mesh projected area does not match retained ring area",
        ));
    }
    let outer_signed_area = projected_area2_signed(outer, projection)
        .ok_or_else(|| surface_validation_error(label, "outer signed area was undecided"))?;
    let hole_signed_area = projected_area2_signed(hole, projection)
        .ok_or_else(|| surface_validation_error(label, "hole signed area was undecided"))?;
    let retained_signed_area = add(&outer_signed_area, &hole_signed_area);
    let mesh_signed_area = projected_mesh_area2_signed(mesh, projection).ok_or_else(|| {
        surface_validation_error(label, "holed surface mesh signed area was undecided")
    })?;
    if compare_reals(&mesh_signed_area, &retained_signed_area).value() != Some(Ordering::Equal) {
        return Err(surface_validation_error(
            label,
            "holed surface mesh signed area does not match retained ring orientation",
        ));
    }
    for point in hole {
        let Some(location) = convex_polygon_location(point, outer, projection) else {
            return Err(surface_validation_error(
                label,
                "hole containment predicate was undecided",
            ));
        };
        if location != ConvexPolygonLocation::Inside {
            return Err(surface_validation_error(
                label,
                "hole ring must be strictly inside the outer ring",
            ));
        }
    }
    mesh.validate_retained_state().map_err(|_| {
        surface_validation_error(label, "materialized mesh retained-state validation failed")
    })
}

#[cfg(feature = "exact-triangulation")]
fn validate_multi_holed_surface_output(
    projection: CoplanarProjection,
    outer: &[Point3],
    holes: &[Vec<Point3>],
    mesh: &ExactMesh,
    label: &'static str,
) -> Result<(), MeshError> {
    if outer.len() < 3 || holes.len() < 2 || holes.iter().any(|hole| hole.len() < 3) {
        return Err(surface_validation_error(
            label,
            "outer and hole rings must all contain at least three vertices",
        ));
    }
    let retained_vertices = outer.len() + holes.iter().map(Vec::len).sum::<usize>();
    if mesh.triangles().is_empty() || mesh.vertices().len() != retained_vertices {
        return Err(surface_validation_error(
            label,
            "multi-holed surface mesh does not match retained rings",
        ));
    }
    let retained_points = outer
        .iter()
        .chain(holes.iter().flat_map(|hole| hole.iter()))
        .collect::<Vec<_>>();
    for (index, point) in retained_points.iter().enumerate() {
        if !points_equal(point, &mesh.vertices()[index].to_hyperlimit_point()) {
            return Err(surface_validation_error(
                label,
                "mesh vertex does not match retained ring point",
            ));
        }
    }
    let mut ranges = Vec::with_capacity(holes.len() + 1);
    ranges.push(0..outer.len());
    let mut start = outer.len();
    for hole in holes {
        ranges.push(start..start + hole.len());
        start += hole.len();
    }
    for triangle in mesh.triangles() {
        if triangle.0.iter().any(|&index| index >= retained_vertices) {
            return Err(surface_validation_error(
                label,
                "multi-holed surface mesh triangle index is out of retained ring range",
            ));
        }
        for hole_range in ranges.iter().skip(1) {
            if triangle.0.iter().all(|index| hole_range.contains(index)) {
                return Err(surface_validation_error(
                    label,
                    "multi-holed surface mesh triangle fills a retained hole",
                ));
            }
        }
    }
    validate_mesh_edges_respect_retained_rings(
        mesh,
        projection,
        &ranges,
        label,
        "multi-holed surface mesh edge crosses a retained ring",
    )?;
    validate_mesh_uses_all_retained_vertices(
        mesh,
        retained_vertices,
        label,
        "multi-holed surface mesh leaves a retained ring vertex unused",
    )?;
    validate_mesh_boundary_matches_retained_rings(
        mesh,
        &ranges,
        label,
        "multi-holed surface mesh boundary does not match retained rings",
    )?;
    validate_exact_point_set_distinct(outer, label, "outer ring repeats an exact point")?;
    validate_projected_simple_loop(outer, projection, label)?;
    validate_projected_strictly_convex_loop(outer, projection, label)?;
    validate_projected_ring_orientation(
        outer,
        projection,
        Sign::Positive,
        label,
        "outer ring orientation must be counter-clockwise",
    )?;
    let outer_area = projected_area2_abs(outer, projection)
        .ok_or_else(|| surface_validation_error(label, "outer projected area was undecided"))?;
    let outer_signed_area = projected_area2_signed(outer, projection)
        .ok_or_else(|| surface_validation_error(label, "outer signed area was undecided"))?;
    let mut hole_area_sum = ExactReal::from(0);
    let mut retained_signed_area = outer_signed_area;
    for hole in holes {
        validate_exact_point_set_distinct(hole, label, "hole ring repeats an exact point")?;
        validate_projected_simple_loop(hole, projection, label)?;
        validate_projected_strictly_convex_loop(hole, projection, label)?;
        validate_projected_ring_orientation(
            hole,
            projection,
            Sign::Negative,
            label,
            "hole ring orientation must be clockwise",
        )?;
        let hole_area = projected_area2_abs(hole, projection)
            .ok_or_else(|| surface_validation_error(label, "hole projected area was undecided"))?;
        let hole_signed_area = projected_area2_signed(hole, projection)
            .ok_or_else(|| surface_validation_error(label, "hole signed area was undecided"))?;
        if compare_reals(&hole_area, &ExactReal::from(0)).value() != Some(Ordering::Greater) {
            return Err(surface_validation_error(
                label,
                "hole rings must have positive projected area",
            ));
        }
        hole_area_sum = add(&hole_area_sum, &hole_area);
        retained_signed_area = add(&retained_signed_area, &hole_signed_area);
        for point in hole {
            let Some(location) = convex_polygon_location(point, outer, projection) else {
                return Err(surface_validation_error(
                    label,
                    "hole containment predicate was undecided",
                ));
            };
            if location != ConvexPolygonLocation::Inside {
                return Err(surface_validation_error(
                    label,
                    "hole ring must be strictly inside the outer ring",
                ));
            }
        }
    }
    if compare_reals(&outer_area, &hole_area_sum).value() != Some(Ordering::Greater) {
        return Err(surface_validation_error(
            label,
            "combined hole area must be strictly smaller than outer area",
        ));
    }
    validate_component_loops_disjoint(holes, projection, label)?;
    let mesh_area = projected_mesh_area2_abs(mesh, projection).ok_or_else(|| {
        surface_validation_error(
            label,
            "multi-holed surface mesh projected area was undecided",
        )
    })?;
    if compare_reals(&mesh_area, &sub(&outer_area, &hole_area_sum)).value() != Some(Ordering::Equal)
    {
        return Err(surface_validation_error(
            label,
            "multi-holed surface mesh projected area does not match retained rings",
        ));
    }
    let mesh_signed_area = projected_mesh_area2_signed(mesh, projection).ok_or_else(|| {
        surface_validation_error(label, "multi-holed surface mesh signed area was undecided")
    })?;
    if compare_reals(&mesh_signed_area, &retained_signed_area).value() != Some(Ordering::Equal) {
        return Err(surface_validation_error(
            label,
            "multi-holed surface mesh signed area does not match retained ring orientation",
        ));
    }
    mesh.validate_retained_state().map_err(|_| {
        surface_validation_error(label, "materialized mesh retained-state validation failed")
    })
}

/// Validate that a holed output triangle does not fill the retained void.
///
/// One-hole planar arrangement artifacts retain the outer ring and the hole
/// ring as separate topology. A materialized triangle whose three vertices all
/// come from the hole ring is not an annulus triangle; it fills the void that
/// the retained certificate says must remain absent. Following Yap, "Towards
/// Exact Geometric Computation," *Computational Geometry* 7.1-2 (1997), this
/// combinatorial fact is checked before area replay so a downstream consumer
/// never has to infer it from aggregate signed areas.
#[cfg(feature = "exact-triangulation")]
fn validate_holed_mesh_triangle_uses_outer_ring(
    triangle: [usize; 3],
    outer_len: usize,
    label: &'static str,
) -> Result<(), MeshError> {
    if triangle.iter().all(|&index| index >= outer_len) {
        return Err(surface_validation_error(
            label,
            "holed surface mesh triangle fills the retained hole",
        ));
    }
    Ok(())
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
type ConvexSurfaceHullsAndAreas = (
    CoplanarProjection,
    Vec<Point3>,
    Vec<Point3>,
    ExactReal,
    ExactReal,
);

#[cfg(feature = "exact-triangulation")]
fn convex_surface_hulls_and_areas(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Option<ConvexSurfaceHullsAndAreas> {
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
fn convex_multi_difference_boundary_area_matches_inputs(
    polygons: &[Vec<Point3>],
    left: &[Point3],
    right: &[Point3],
    projection: CoplanarProjection,
) -> Option<bool> {
    let mut output_area = ExactReal::from(0);
    for polygon in polygons {
        output_area = add(&output_area, &projected_area2_abs(polygon, projection)?);
    }
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
            let keep = matches!(
                (operation, from_left, location),
                (ArrangementOperation::Union, _, TriangleLocation::Outside)
                    | (
                        ArrangementOperation::Difference,
                        true,
                        TriangleLocation::Outside
                    )
                    | (
                        ArrangementOperation::Difference,
                        false,
                        TriangleLocation::Inside
                    )
            );
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
fn stitch_disjoint_simple_loops(
    mut fragments: Vec<DirectedFragment>,
    projection: CoplanarProjection,
) -> Option<Vec<Vec<Point3>>> {
    if fragments.len() < 6 {
        return None;
    }
    let mut loops = Vec::new();
    while !fragments.is_empty() {
        let first = fragments.remove(0);
        let mut polygon = vec![first.start, first.end];
        loop {
            let current = polygon.last()?.clone();
            if points_equal(&current, polygon.first()?) {
                polygon.pop();
                break;
            }
            let next_index = fragments
                .iter()
                .position(|fragment| points_equal(&fragment.start, &current))?;
            let next = fragments.remove(next_index);
            polygon.push(next.end);
            if polygon.len() > 128 {
                return None;
            }
        }
        let polygon = simplify_projected_polygon(polygon, projection);
        if polygon.len() < 3 {
            return None;
        }
        loops.push(polygon);
    }
    if loops.len() < 2 { None } else { Some(loops) }
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

fn projected_mesh_area2_abs(mesh: &ExactMesh, projection: CoplanarProjection) -> Option<ExactReal> {
    let mut area = ExactReal::from(0);
    for triangle in mesh.triangles() {
        let points = triangle
            .0
            .iter()
            .map(|&index| mesh.vertices()[index].to_hyperlimit_point())
            .collect::<Vec<_>>();
        area = add(&area, &projected_area2_abs(&points, projection)?);
    }
    Some(area)
}

/// Replay the signed projected area of a materialized surface mesh.
///
/// The retained planar-arrangement rings describe both area and orientation;
/// triangle soup with the same absolute area but reversed winding is not the
/// same certified artifact. This check replays the same determinant sum used
/// for ring orientation before the mesh crosses an API boundary, following
/// Yap, "Towards Exact Geometric Computation," *Computational Geometry*
/// 7.1-2 (1997), and the oriented-area determinant described by Preparata
/// and Shamos, *Computational Geometry: An Introduction* (1985), Chapter 1.
fn projected_mesh_area2_signed(
    mesh: &ExactMesh,
    projection: CoplanarProjection,
) -> Option<ExactReal> {
    let mut area = ExactReal::from(0);
    for triangle in mesh.triangles() {
        let points = triangle
            .0
            .iter()
            .map(|&index| mesh.vertices()[index].to_hyperlimit_point())
            .collect::<Vec<_>>();
        area = add(&area, &projected_area2_signed(&points, projection)?);
    }
    Some(area)
}

fn compare_point2(left: &Point2, right: &Point2) -> Option<Ordering> {
    match compare_reals(&left.x, &right.x).value()? {
        Ordering::Equal => compare_reals(&left.y, &right.y).value(),
        ordering => Some(ordering),
    }
}

#[cfg(feature = "exact-triangulation")]
fn real_order(left: &ExactReal, right: &ExactReal) -> Option<Ordering> {
    compare_reals(left, right).value()
}

#[cfg(feature = "exact-triangulation")]
fn real_equal(left: &ExactReal, right: &ExactReal) -> bool {
    real_order(left, right) == Some(Ordering::Equal)
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
