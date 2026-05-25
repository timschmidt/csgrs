//! Exact certification for lower-dimensional surface special cases.
//!
//! This module keeps sheet/surface shortcuts separate from volumetric convex
//! shortcuts. The certified cases are intentionally narrow: single coplanar
//! triangle containment, positive-area intersection, convex union, simple
//! single-loop planar-arrangement union/difference, one-hole and bounded
//! multi-hole differences, nonconvex component-union loops, disconnected
//! nonconvex component-union multi-loops, bounded component-holed unions,
//! bounded cutter/hole openings with
//! retained strict holes, independent consumed straddling-hole split groups,
//! four-sided consumed branch groups, clipped nonconvex-source openings that
//! consume strict holes, and the convex one-corner difference shapes that can
//! be represented as an open triangle mesh. The
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
    compare_reals, interpolate_point3 as interpolate3, orient2d_report, orient2d_value,
    point_on_segment, project_point3 as project_point,
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
    ///
    /// The source component that produced this ring is convex, but a certified
    /// cutter may leave a simple nonconvex remnant. The ring itself is the
    /// retained topology certificate.
    pub outer: Vec<Point3>,
    /// Exact 3D hole rings, retained clockwise and strictly inside `outer`.
    pub holes: Vec<Vec<Point3>>,
}

/// Exact mixed component/holed coplanar surface output.
///
/// This artifact covers bounded cases where a surface boolean contains one or
/// more disjoint components and at least one component carries exact holes.
/// Difference paths retain convex or simple nonconvex remnants after bounded
/// cutter replay; union paths may retain an annulus when source-owned disk
/// components meet along exact positive-length boundary arcs and exposed
/// boundary fragments replay as one outer ring plus strict hole rings. More
/// tangled cut/hole interactions still require a full planar subdivision. Each
/// retained component must replay from exact component decomposition,
/// containment, disjointness, contact, and area certificates before the
/// materialized mesh is accepted, matching the retained-object contract in
/// Yap, "Towards Exact Geometric Computation," *Computational Geometry*
/// 7.1-2 (1997).
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

    /// Validate this component-holed union against its exact sources.
    ///
    /// The union producer accepts a bounded annular planar arrangement: source
    /// disk components may meet through exact positive-length boundary arcs,
    /// their exposed boundary must replay as one outer ring plus strict hole
    /// rings, and exact area must equal the sum of source component areas.
    /// Replaying from the sources keeps the same Yap-style retained object
    /// boundary as the difference producer, while making the operation
    /// explicit so a holed union cannot be relabeled as a subtraction. This is
    /// the retained-state contract described by Yap, "Towards Exact Geometric
    /// Computation," *Computational Geometry* 7.1-2 (1997).
    pub fn validate_union_against_sources(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> Result<(), MeshError> {
        self.validate()?;
        let replay =
            arrange_coplanar_surface_component_holed_union(left, right).ok_or_else(|| {
                surface_validation_error(
                    "coplanar component-holed union arrangement",
                    "source replay did not reproduce a component-holed union",
                )
            })?;
        if self == &replay {
            Ok(())
        } else {
            Err(surface_validation_error(
                "coplanar component-holed union arrangement",
                "retained component-holed union does not match source replay",
            ))
        }
    }

    /// Validate this component-holed intersection against its exact sources.
    ///
    /// The intersection producer is a bounded source-owned sheet clip: one
    /// operand must replay as a triangulated coplanar surface with retained
    /// boundary holes, and the other as simple source disks that lie strictly
    /// inside a source outer ring while strictly containing every retained
    /// hole they expose. Replaying from the sources keeps the retained rings
    /// tied to their exact mesh incidence and exact area predicates, following
    /// Yap, "Towards Exact Geometric Computation," *Computational Geometry*
    /// 7.1-2 (1997), instead of treating the triangulated output as an
    /// unproved planar arrangement.
    pub fn validate_intersection_against_sources(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> Result<(), MeshError> {
        self.validate()?;
        let replay = arrange_coplanar_surface_component_holed_intersection(left, right)
            .ok_or_else(|| {
                surface_validation_error(
                    "coplanar component-holed intersection arrangement",
                    "source replay did not reproduce a component-holed intersection",
                )
            })?;
        if self == &replay {
            Ok(())
        } else {
            Err(surface_validation_error(
                "coplanar component-holed intersection arrangement",
                "retained component-holed intersection does not match source replay",
            ))
        }
    }

    /// Validate this component-holed surface difference against its sources.
    ///
    /// This operation-specific replay covers bounded same-outer holed sheet
    /// subtraction in addition to the older convex-source component-holed
    /// difference. The replay keeps the artifact tied to exact retained
    /// source rings and exact area predicates, matching Yap, "Towards Exact
    /// Geometric Computation," *Computational Geometry* 7.1-2 (1997): a
    /// component-holed mesh is accepted only while the source objects still
    /// prove the topology that produced it.
    pub fn validate_surface_difference_against_sources(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> Result<(), MeshError> {
        self.validate()?;
        let replay = arrange_coplanar_convex_surface_component_holed_difference(left, right)
            .or_else(|| arrange_coplanar_surface_component_holed_difference(left, right))
            .ok_or_else(|| {
                surface_validation_error(
                    "coplanar component-holed difference arrangement",
                    "source replay did not reproduce a component-holed difference",
                )
            })?;
        if self == &replay {
            Ok(())
        } else {
            Err(surface_validation_error(
                "coplanar component-holed difference arrangement",
                "retained component-holed difference does not match source replay",
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
/// strictly convex. It is also the bounded disconnected counterpart to
/// [`CoplanarSurfaceArrangement`] for component-wise unions: each connected
/// source contact cluster is retained as its own simple loop, so a far
/// component no longer forces an otherwise certified nonconvex union back to
/// the generic planar-arrangement blocker. The artifact exists so the convex
/// certificate does not silently weaken its invariant. Construction still
/// follows exact Weiler-Atherton style boundary replay for each promoted loop,
/// and triangulation is retained through `hypertri`'s FIST-style earcut
/// handoff. See Weiler and Atherton, "Hidden Surface Removal Using Polygon
/// Area Sorting," *SIGGRAPH Computer Graphics* 11.2 (1977), Held, "FIST:
/// Fast Industrial-Strength Triangulation of Polygons," *Algorithmica* 30
/// (2001), and Yap, "Towards Exact Geometric Computation," *Computational
/// Geometry* 7.1-2 (1997).
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

/// Exact coplanar union output whose components meet only at shared vertices.
///
/// This artifact is deliberately separate from
/// [`CoplanarSurfaceMultiArrangement`]. A point-touch union has branch
/// incidence in the geometric image, but its retained mesh keeps each disk
/// component as a separate loop and separate mesh vertex at the same exact
/// coordinate. That is the bounded form of the branch-point work: exact
/// vertex-vertex contacts are certified directly, exact vertex-edge contacts
/// are first promoted to retained edge-split vertices, and edge contacts,
/// proper overlaps, nesting, and general planar subdivisions remain with the
/// stronger arrangement paths. This follows
/// Yap, "Towards Exact Geometric Computation," *Computational Geometry*
/// 7.1-2 (1997): the combinatorial contract is named and validated rather
/// than inferred from floating tolerances. The segment contact tests use the
/// same orientation-predicate model as Guigue and Devillers, "Fast and Robust
/// Triangle-Triangle Overlap Test Using Orientation Predicates," *Journal of
/// Graphics Tools* 8.1 (2003).
#[cfg(feature = "exact-triangulation")]
#[derive(Clone, Debug, PartialEq)]
pub struct CoplanarSurfacePointTouchUnion {
    /// Projection used by exact 2D contact predicates and triangulation.
    pub projection: CoplanarProjection,
    /// Exact 3D simple boundary loops, one per retained disk component.
    pub polygons: Vec<Vec<Point3>>,
    /// Exact triangulated open surface mesh containing all retained disks.
    pub mesh: ExactMesh,
}

/// Exact coplanar difference output whose retained components meet at vertices.
///
/// This is the side-cutter counterpart to [`CoplanarSurfacePointTouchUnion`].
/// A set of removed side openings may touch only at exact vertices, splitting
/// the retained image into simple components that share geometric branch
/// points. The mesh keeps each component loop independent and duplicates the
/// shared coordinates, so no halfedge incidence is invented by coordinate
/// equality alone. Positive-area/positive-length cutter groups are still
/// merged by exact boundary replay; point-only contact is accepted only as
/// lower-dimensional branch evidence after the removed openings and retained
/// components satisfy exact area and source-boundary ownership checks.
///
/// The retained-fragment construction follows Weiler and Atherton, "Hidden
/// Surface Removal Using Polygon Area Sorting," *SIGGRAPH Computer Graphics*
/// 11.2 (1977). Segment contact is certified with the orientation-predicate
/// model of Guigue and Devillers, "Fast and Robust Triangle-Triangle Overlap
/// Test Using Orientation Predicates," *Journal of Graphics Tools* 8.1
/// (2003). Yap, "Towards Exact Geometric Computation," *Computational
/// Geometry* 7.1-2 (1997), is the reason this artifact is separate from the
/// ordinary multi-difference object: the branch topology is explicit retained
/// state, not a tolerance-side effect.
#[cfg(feature = "exact-triangulation")]
#[derive(Clone, Debug, PartialEq)]
pub struct CoplanarSurfacePointTouchDifference {
    /// Projection used by exact 2D branch predicates and triangulation.
    pub projection: CoplanarProjection,
    /// Exact 3D simple boundary loops, one per retained component.
    pub polygons: Vec<Vec<Point3>>,
    /// Exact triangulated open surface mesh containing all retained disks.
    pub mesh: ExactMesh,
}

/// Exact single-loop arrangement output for nonconvex coplanar surfaces.
///
/// This is the single-component counterpart to
/// [`CoplanarSurfaceMultiArrangement`]. It covers bounded cases where the
/// output has one retained simple loop that can be audited directly. Producers
/// include cutter/hole-contact differences, where a side-attached cutter opens
/// a strictly contained hole to the outer boundary; side-cutter-only
/// differences, where exact non-rectilinear cutter openings carve one
/// nonconvex simple remnant without retained hole rings; and the bounded
/// same-outer holed subtraction whose result is one filled source-owned hole.
/// The output keeps that loop as exact topology and triangulates it through
/// `hypertri`'s FIST-style earcut handoff. See Held,
/// "FIST: Fast Industrial-Strength Triangulation of Polygons,"
/// *Algorithmica* 30 (2001), and Yap, "Towards Exact Geometric Computation,"
/// *Computational Geometry* 7.1-2 (1997).
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

    /// Validate this nonconvex multi-component union against its sources.
    ///
    /// The retained union is accepted only while exact source replay rebuilds
    /// the same disconnected contact clusters, stitched loops, and
    /// triangulated mesh. This keeps disconnected nonconvex unions in Yap's
    /// retained-computation model from "Towards Exact Geometric Computation,"
    /// *Computational Geometry* 7.1-2 (1997): consumers receive a replayable
    /// arrangement artifact instead of a detached triangle soup.
    pub fn validate_union_against_sources(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> Result<(), MeshError> {
        self.validate()?;
        let replay =
            arrange_coplanar_surface_multi_component_union(left, right).ok_or_else(|| {
                surface_validation_error(
                    "coplanar nonconvex multi-component arrangement",
                    "source replay did not reproduce a nonconvex multi-component union",
                )
            })?;
        if self == &replay {
            Ok(())
        } else {
            Err(surface_validation_error(
                "coplanar nonconvex multi-component arrangement",
                "retained union does not match source replay",
            ))
        }
    }

    /// Validate this nonconvex multi-component intersection against sources.
    ///
    /// The intersection path starts from exact pairwise convex clips and
    /// merges only positive-length adjacent clip components. Replaying from
    /// sources is therefore part of the certificate: it ties every retained
    /// loop to the source triangle pairs and exact area replay that produced
    /// it, following Yap, "Towards Exact Geometric Computation,"
    /// *Computational Geometry* 7.1-2 (1997).
    pub fn validate_intersection_against_sources(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> Result<(), MeshError> {
        self.validate()?;
        let replay = arrange_coplanar_surface_multi_component_intersection(left, right)
            .ok_or_else(|| {
                surface_validation_error(
                    "coplanar nonconvex multi-component arrangement",
                    "source replay did not reproduce a nonconvex multi-component intersection",
                )
            })?;
        if self == &replay {
            Ok(())
        } else {
            Err(surface_validation_error(
                "coplanar nonconvex multi-component arrangement",
                "retained intersection does not match source replay",
            ))
        }
    }
}

#[cfg(feature = "exact-triangulation")]
impl CoplanarSurfacePointTouchUnion {
    /// Validate branch-point component loops, exact point contacts, and mesh state.
    ///
    /// Validation allows repeated exact coordinates only across different
    /// retained loops, and only when the loops meet through exact shared
    /// vertices. Vertex-edge contacts are validated after the materializer has
    /// inserted the touched edge point into the retained loop. The retained
    /// mesh itself keeps those vertices duplicated so each disk component
    /// still has an ordinary boundary loop.
    /// This mirrors Yap's retained-state discipline from "Towards Exact
    /// Geometric Computation," *Computational Geometry* 7.1-2 (1997): the
    /// branch incidence is part of the explicit artifact contract.
    pub fn validate(&self) -> Result<(), MeshError> {
        validate_multi_surface_output_allowing_vertex_point_touches(
            self.projection,
            &self.polygons,
            &self.mesh,
            "coplanar point-touch surface union",
            false,
        )
    }

    /// Validate this point-touch union against its exact sources.
    ///
    /// The replay must reproduce the same ordered component loops and the same
    /// duplicate branch vertices. A stale object that merely has a locally
    /// valid mesh is rejected unless the source operands still certify exactly
    /// this bounded point-touch branch union.
    pub fn validate_union_against_sources(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> Result<(), MeshError> {
        self.validate()?;
        let replay = arrange_coplanar_surface_point_touch_union(left, right).ok_or_else(|| {
            surface_validation_error(
                "coplanar point-touch surface union",
                "source replay did not reproduce a point-touch union",
            )
        })?;
        if self == &replay {
            Ok(())
        } else {
            Err(surface_validation_error(
                "coplanar point-touch surface union",
                "retained union does not match source replay",
            ))
        }
    }
}

#[cfg(feature = "exact-triangulation")]
impl CoplanarSurfacePointTouchDifference {
    /// Validate branch-point difference loops, exact point contacts, and mesh state.
    ///
    /// The retained components may share exact vertex coordinates across loops
    /// but must not cross, overlap, nest, or touch along positive-length
    /// intervals. The triangulated mesh must keep those branch coordinates as
    /// separate vertices owned by separate component loops.
    pub fn validate(&self) -> Result<(), MeshError> {
        validate_multi_surface_output_allowing_vertex_point_touches(
            self.projection,
            &self.polygons,
            &self.mesh,
            "coplanar point-touch surface difference",
            false,
        )
    }

    /// Validate this point-touch difference against its exact sources.
    ///
    /// Source replay rebuilds the side-cutter point-branch certificate and
    /// requires the same retained loops and duplicate branch vertices. This
    /// keeps the artifact inside Yap's exact-computation model: a locally valid
    /// branch mesh is accepted only while the source predicates still justify
    /// exactly that topology.
    pub fn validate_difference_against_sources(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> Result<(), MeshError> {
        self.validate()?;
        let replay =
            arrange_coplanar_surface_point_touch_difference(left, right).ok_or_else(|| {
                surface_validation_error(
                    "coplanar point-touch surface difference",
                    "source replay did not reproduce a point-touch difference",
                )
            })?;
        if self == &replay {
            Ok(())
        } else {
            Err(surface_validation_error(
                "coplanar point-touch surface difference",
                "retained difference does not match source replay",
            ))
        }
    }
}

#[cfg(feature = "exact-triangulation")]
impl CoplanarSurfaceArrangement {
    /// Validate the retained simple loop and mesh.
    ///
    /// The artifact deliberately does not require convexity. It does require
    /// one positive-area, counter-clockwise, self-disjoint loop whose
    /// triangulated mesh has exactly the same boundary. This keeps the output
    /// inside Yap's exact-state discipline: callers receive a replayable
    /// combinatorial object, not only a triangle soup.
    pub fn validate(&self) -> Result<(), MeshError> {
        validate_coplanar_surface_output(
            self.projection,
            &self.polygon,
            &self.mesh,
            "coplanar simple-loop arrangement",
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

    /// Validate this side-cutter-only difference against its sources.
    ///
    /// Source replay rebuilds the clipped cutter openings, retained boundary
    /// splice, and exact area equation from the supplied meshes. The artifact
    /// is accepted only when that replay reproduces the same loop and mesh,
    /// keeping this single-loop nonconvex shortcut inside Yap's retained-state
    /// model from "Towards Exact Geometric Computation," *Computational
    /// Geometry* 7.1-2 (1997).
    pub fn validate_side_cutter_difference_against_sources(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> Result<(), MeshError> {
        self.validate()?;
        let replay =
            arrange_coplanar_surface_side_cutter_difference(left, right).ok_or_else(|| {
                surface_validation_error(
                    "coplanar nonconvex simple-loop arrangement",
                    "source replay did not reproduce a side-cutter difference",
                )
            })?;
        if self == &replay {
            Ok(())
        } else {
            Err(surface_validation_error(
                "coplanar nonconvex simple-loop arrangement",
                "retained side-cutter difference does not match source replay",
            ))
        }
    }

    /// Validate this nonconvex component-union loop against its sources.
    ///
    /// The component-union path promotes one connected contact/overlap graph
    /// of convex source components into a single simple loop. Source replay
    /// rebuilds the exact component graph, retained boundary fragments, and
    /// area certificate before accepting this copied artifact. This is the
    /// object/predicate split advocated by Yap, "Towards Exact Geometric
    /// Computation," *Computational Geometry* 7.1-2 (1997): the nonconvex
    /// topology remains certified only while its exact construction history
    /// still replays.
    pub fn validate_component_union_against_sources(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> Result<(), MeshError> {
        self.validate()?;
        let replay = arrange_coplanar_surface_component_union(left, right).ok_or_else(|| {
            surface_validation_error(
                "coplanar nonconvex simple-loop arrangement",
                "source replay did not reproduce a component-union loop",
            )
        })?;
        if self == &replay {
            Ok(())
        } else {
            Err(surface_validation_error(
                "coplanar nonconvex simple-loop arrangement",
                "retained component union does not match source replay",
            ))
        }
    }

    /// Validate this component-difference loop against its sources.
    ///
    /// This replay is intentionally narrower than arbitrary planar
    /// subtraction: it rebuilds the exact connected source components, drops
    /// wholly covered components, and then requires the one retained remnant
    /// or source-holed filled-hole component to match this loop and
    /// triangulation exactly. Keeping that construction history attached to
    /// the artifact is the certified-object boundary required by Yap,
    /// "Towards Exact Geometric Computation," *Computational Geometry* 7.1-2
    /// (1997).
    pub fn validate_component_difference_against_sources(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> Result<(), MeshError> {
        self.validate()?;
        let replay =
            arrange_coplanar_surface_component_difference(left, right).ok_or_else(|| {
                surface_validation_error(
                    "coplanar simple-loop arrangement",
                    "source replay did not reproduce a component-difference loop",
                )
            })?;
        if self == &replay {
            Ok(())
        } else {
            Err(surface_validation_error(
                "coplanar simple-loop arrangement",
                "retained component difference does not match source replay",
            ))
        }
    }

    /// Validate this nonconvex simple-loop intersection against its sources.
    ///
    /// Source replay recomputes the exact pairwise triangle clips, the
    /// positive-length contact graph, the stitched boundary loop, and the
    /// retained triangulation. That replay requirement keeps this bounded
    /// planar-cell materializer aligned with Yap's retained exact object model
    /// from "Towards Exact Geometric Computation," *Computational Geometry*
    /// 7.1-2 (1997).
    pub fn validate_intersection_against_sources(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> Result<(), MeshError> {
        self.validate()?;
        let replay =
            arrange_coplanar_surface_component_intersection(left, right).ok_or_else(|| {
                surface_validation_error(
                    "coplanar nonconvex simple-loop arrangement",
                    "source replay did not reproduce a nonconvex intersection",
                )
            })?;
        if self == &replay {
            Ok(())
        } else {
            Err(surface_validation_error(
                "coplanar nonconvex simple-loop arrangement",
                "retained intersection does not match source replay",
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

/// Materialize a convex coplanar difference with several holes.
///
/// This bounded materializer handles one convex coplanar left sheet, including
/// a single source triangle, and a right operand made of two or more disjoint
/// connected convex sheets, all strictly inside the left hull. It is
/// intentionally narrower than arbitrary planar-cell extraction: touching
/// holes, nested holes, and nonconvex coverage still fail closed. The accepted
/// case retains every component hull as a ring and replays exact area,
/// matching Yap's exact-computation discipline from "Towards Exact Geometric
/// Computation," *Computational Geometry* 7.1-2 (1997).
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

    let (projection, mut outer) = convex_outer_ring_for_multi_hole_difference(left)?;
    orient_polygon_ccw(&mut outer, projection)?;
    let outer_area = projected_area2_abs(&outer, projection)?;
    let mut holes = Vec::new();
    let mut hole_area_sum = ExactReal::from(0);
    for component in connected_face_component_meshes(right)? {
        let hole_mesh = component;
        let mut hole = contained_hole_ring_for_multi_hole_difference(left, &hole_mesh, projection)?;
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

#[cfg(feature = "exact-triangulation")]
fn convex_outer_ring_for_multi_hole_difference(
    left: &ExactMesh,
) -> Option<(CoplanarProjection, Vec<Point3>)> {
    if left.triangles().len() == 1 {
        let projection = choose_mesh_projection(left)?;
        let triangle = left.triangles()[0].0;
        let outer = triangle
            .iter()
            .map(|&index| Some(left.vertices().get(index)?.to_hyperlimit_point()))
            .collect::<Option<Vec<_>>>()?;
        return Some((projection, outer));
    }
    let (projection, outer, _, _, _) = convex_surface_hulls_and_areas(left, left)?;
    Some((projection, outer))
}

#[cfg(feature = "exact-triangulation")]
fn contained_hole_ring_for_multi_hole_difference(
    left: &ExactMesh,
    hole_mesh: &ExactMesh,
    projection: CoplanarProjection,
) -> Option<Vec<Point3>> {
    if left.triangles().len() == 1 && hole_mesh.triangles().len() == 1 {
        if certify_single_triangle_coplanar_containment(left, hole_mesh)?
            != CoplanarSurfaceContainment::RightInsideLeft
        {
            return None;
        }
        let triangle = hole_mesh.triangles()[0].0;
        return triangle
            .iter()
            .map(|&index| Some(hole_mesh.vertices().get(index)?.to_hyperlimit_point()))
            .collect::<Option<Vec<_>>>();
    }
    let certificate = certify_coplanar_convex_surface_containment(left, hole_mesh)?;
    if certificate.projection != projection
        || certificate.relation != CoplanarConvexSurfaceContainment::RightInsideLeft
    {
        return None;
    }
    Some(certificate.right_hull)
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

/// Recover the single boundary ring of an open triangulated source disk.
///
/// The ring is accepted only when every boundary vertex has degree two in the
/// boundary-edge graph and all non-boundary edges have exactly two incident
/// triangles. This is intentionally stricter than a general mesh traversal:
/// multiple rings, non-manifold seams, dangling edges, and branch vertices are
/// planar-arrangement inputs, not proof for the bounded nonconvex source
/// difference path. The check preserves Yap's retained object/state split from
/// "Towards Exact Geometric Computation," *Computational Geometry* 7.1-2
/// (1997): source topology is replayed from exact mesh incidence before any
/// coordinate predicate is used.
#[cfg(feature = "exact-triangulation")]
fn order_single_mesh_boundary_loop(mesh: &ExactMesh) -> Option<Vec<usize>> {
    let mut edge_counts: Vec<((usize, usize), usize)> = Vec::new();
    for triangle in mesh.triangles() {
        for (a, b) in [
            (triangle.0[0], triangle.0[1]),
            (triangle.0[1], triangle.0[2]),
            (triangle.0[2], triangle.0[0]),
        ] {
            let edge = canonical_edge(a, b);
            if let Some((_, count)) = edge_counts
                .iter_mut()
                .find(|(candidate, _)| *candidate == edge)
            {
                *count += 1;
            } else {
                edge_counts.push((edge, 1));
            }
        }
    }
    if edge_counts
        .iter()
        .any(|(_, count)| *count == 0 || *count > 2)
    {
        return None;
    }
    let boundary_edges = edge_counts
        .into_iter()
        .filter_map(|(edge, count)| (count == 1).then_some(edge))
        .collect::<Vec<_>>();
    if boundary_edges.len() < 3 {
        return None;
    }
    let mut boundary_vertices = Vec::with_capacity(boundary_edges.len());
    let start = boundary_edges.iter().map(|(a, b)| (*a).min(*b)).min()?;
    let mut previous = None;
    let mut current = start;
    loop {
        boundary_vertices.push(current);
        let neighbors = boundary_edges
            .iter()
            .filter_map(|(a, b)| {
                if *a == current {
                    Some(*b)
                } else if *b == current {
                    Some(*a)
                } else {
                    None
                }
            })
            .collect::<Vec<_>>();
        if neighbors.len() != 2 {
            return None;
        }
        let next = match previous {
            Some(previous) => *neighbors.iter().find(|&&candidate| candidate != previous)?,
            None => neighbors.into_iter().min()?,
        };
        if next == start {
            break;
        }
        if boundary_vertices.contains(&next) {
            return None;
        }
        previous = Some(current);
        current = next;
        if boundary_vertices.len() > boundary_edges.len() {
            return None;
        }
    }
    if boundary_vertices.len() == boundary_edges.len() {
        Some(boundary_vertices)
    } else {
        None
    }
}

/// Recover all boundary rings of one open triangulated coplanar component.
///
/// This is the multi-ring sibling of [`order_single_mesh_boundary_loop`]. It
/// deliberately stays in mesh topology until each boundary cycle has been
/// recovered: boundary edges must have one incident triangle, interior edges
/// must have two, and every boundary vertex must have degree two. That is the
/// exact structural object Yap asks algorithms to retain in "Towards Exact
/// Geometric Computation," *Computational Geometry* 7.1-2 (1997). The caller
/// is still responsible for deciding which cycle is the outer ring by exact
/// projected containment predicates; this helper only certifies incidence.
#[cfg(feature = "exact-triangulation")]
fn order_mesh_boundary_loops(mesh: &ExactMesh) -> Option<Vec<Vec<usize>>> {
    let mut edge_counts: Vec<((usize, usize), usize)> = Vec::new();
    for triangle in mesh.triangles() {
        for (a, b) in [
            (triangle.0[0], triangle.0[1]),
            (triangle.0[1], triangle.0[2]),
            (triangle.0[2], triangle.0[0]),
        ] {
            let edge = canonical_edge(a, b);
            if let Some((_, count)) = edge_counts
                .iter_mut()
                .find(|(candidate, _)| *candidate == edge)
            {
                *count += 1;
            } else {
                edge_counts.push((edge, 1));
            }
        }
    }
    if edge_counts
        .iter()
        .any(|(_, count)| *count == 0 || *count > 2)
    {
        return None;
    }
    let boundary_edges = edge_counts
        .into_iter()
        .filter_map(|(edge, count)| (count == 1).then_some(edge))
        .collect::<Vec<_>>();
    if boundary_edges.len() < 3 {
        return None;
    }

    let mut boundary_vertices = Vec::new();
    for &(a, b) in &boundary_edges {
        if !boundary_vertices.contains(&a) {
            boundary_vertices.push(a);
        }
        if !boundary_vertices.contains(&b) {
            boundary_vertices.push(b);
        }
    }
    for &vertex in &boundary_vertices {
        let degree = boundary_edges
            .iter()
            .filter(|(a, b)| *a == vertex || *b == vertex)
            .count();
        if degree != 2 {
            return None;
        }
    }

    let mut used = vec![false; boundary_edges.len()];
    let mut loops = Vec::new();
    while let Some(seed) = used.iter().position(|used| !*used) {
        let (a, b) = boundary_edges[seed];
        let start = a.min(b);
        let mut previous = None;
        let mut current = start;
        let mut loop_vertices = Vec::new();
        loop {
            loop_vertices.push(current);
            let mut candidates = boundary_edges
                .iter()
                .enumerate()
                .filter_map(|(index, (edge_a, edge_b))| {
                    if used[index] {
                        return None;
                    }
                    if *edge_a == current {
                        Some((index, *edge_b))
                    } else if *edge_b == current {
                        Some((index, *edge_a))
                    } else {
                        None
                    }
                })
                .collect::<Vec<_>>();
            candidates.sort_by_key(|(_, next)| *next);
            let (edge_index, next) = match previous {
                Some(previous) => candidates
                    .into_iter()
                    .find(|(_, candidate)| *candidate != previous)?,
                None => candidates.into_iter().next()?,
            };
            used[edge_index] = true;
            if next == start {
                break;
            }
            if loop_vertices.contains(&next) {
                return None;
            }
            previous = Some(current);
            current = next;
            if loop_vertices.len() > boundary_edges.len() {
                return None;
            }
        }
        if loop_vertices.len() < 3 {
            return None;
        }
        loops.push(loop_vertices);
    }
    if loops.is_empty() || used.iter().any(|used| !*used) {
        None
    } else {
        Some(loops)
    }
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

/// Certify and materialize one nonconvex coplanar intersection loop.
///
/// This is the adjacent face-cell counterpart to
/// [`arrange_coplanar_convex_surface_multi_intersection`]. Pairwise convex
/// triangle clips are first retained exactly, then clips that meet along
/// positive-length boundaries are replayed as one connected convex-contact
/// union loop. The shortcut is accepted only when that merged loop is simple
/// and nonconvex; convex and disjoint-convex cases stay with the narrower
/// convex intersection certificates.
///
/// The local clips use Sutherland and Hodgman's exact half-plane clipping
/// construction, while the merge uses the Weiler-Atherton boundary-fragment
/// idea already used by coplanar unions. Yap, "Towards Exact Geometric
/// Computation," *Computational Geometry* 7.1-2 (1997), is the reason the
/// promoted loop must replay from retained convex clips and exact area
/// equality rather than from sampled arrangement cells.
#[cfg(feature = "exact-triangulation")]
pub fn arrange_coplanar_surface_component_intersection(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Option<CoplanarSurfaceArrangement> {
    let (projection, mut polygons) =
        coplanar_surface_pairwise_triangle_intersection_polygons(left, right)?;
    if polygons.len() != 1 {
        return None;
    }
    let mut polygon = polygons.pop()?;
    orient_polygon_ccw(&mut polygon, projection)?;
    if validate_projected_strictly_convex_loop(
        &polygon,
        projection,
        "coplanar nonconvex intersection",
    )
    .is_ok()
    {
        return None;
    }
    let mesh = polygon_to_earcut_open_mesh_with_label(
        &polygon,
        projection,
        "exact coplanar nonconvex intersection",
    )?;
    let arrangement = CoplanarSurfaceArrangement {
        projection,
        polygon,
        mesh,
    };
    arrangement.validate().ok()?;
    Some(arrangement)
}

/// Certify and materialize several coplanar intersection loops, allowing
/// nonconvex components produced by adjacent exact face-cell clips.
///
/// Each output component is either one retained pairwise convex clip or the
/// exact convex-contact union of several clips. At least one output loop must
/// be nonconvex; an all-convex disjoint result remains the responsibility of
/// [`arrange_coplanar_convex_surface_multi_intersection`]. This keeps the
/// public artifact contract explicit while advancing the remaining planar
/// cell-arrangement work.
#[cfg(feature = "exact-triangulation")]
pub fn arrange_coplanar_surface_multi_component_intersection(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Option<CoplanarSurfaceMultiArrangement> {
    let (projection, polygons) =
        coplanar_surface_pairwise_triangle_intersection_polygons(left, right)?;
    if polygons.len() < 2 {
        return None;
    }
    if polygons.iter().all(|polygon| {
        validate_projected_strictly_convex_loop(
            polygon,
            projection,
            "coplanar nonconvex multi-component intersection",
        )
        .is_ok()
    }) {
        return None;
    }
    let mesh = polygons_to_earcut_open_mesh_with_label(
        &polygons,
        projection,
        "exact coplanar nonconvex multi-component intersection",
    )?;
    let arrangement = CoplanarSurfaceMultiArrangement {
        projection,
        polygons,
        mesh,
    };
    arrangement.validate().ok()?;
    Some(arrangement)
}

/// Certify whole-surface containment for coplanar triangulated sheets.
///
/// This is the non-single-triangle containment certificate used by named
/// booleans whose result can be copied directly from one source mesh. It
/// deliberately avoids inferring topology from a boundary sample: every
/// source triangle is clipped against every triangle of the candidate cover,
/// and the exact sum of positive-area clips must equal the source triangle's
/// exact projected area. The cover mesh is first required to have pairwise
/// interior-disjoint coplanar faces, so the replay cannot double-count
/// overlapping cover triangles.
///
/// The local clips reuse Sutherland and Hodgman's half-plane construction
/// (Sutherland and Hodgman, "Reentrant Polygon Clipping," *Communications of
/// the ACM* 17.1, 1974), while the acceptance rule follows Yap, "Towards
/// Exact Geometric Computation," *Computational Geometry* 7.1-2 (1997):
/// topology-changing containment is emitted only when exact retained
/// predicate/area facts prove whole-object coverage. In particular, retained
/// holes remain holes because their uncovered area contributes no clip area.
#[cfg(feature = "exact-triangulation")]
pub fn certify_coplanar_surface_mesh_containment(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Option<CoplanarSurfaceContainment> {
    if left.triangles().is_empty() || right.triangles().is_empty() {
        return None;
    }
    if certify_single_triangle_coplanar_containment(left, right).is_some()
        || certify_coplanar_convex_surface_containment(left, right).is_some()
        || certify_coplanar_convex_surface_equivalence(left, right).is_some()
    {
        return None;
    }
    if !single_retained_plane(left, right)? {
        return None;
    }
    let projection = choose_mesh_projection(left).or_else(|| choose_mesh_projection(right))?;
    if !coplanar_mesh_faces_have_disjoint_interiors(left, projection)?
        || !coplanar_mesh_faces_have_disjoint_interiors(right, projection)?
    {
        return None;
    }

    let left_inside_right = coplanar_mesh_area_covered_by_mesh(left, right, projection)?;
    let right_inside_left = coplanar_mesh_area_covered_by_mesh(right, left, projection)?;
    match (left_inside_right, right_inside_left) {
        (true, false) => Some(CoplanarSurfaceContainment::LeftInsideRight),
        (false, true) => Some(CoplanarSurfaceContainment::RightInsideLeft),
        (true, true) | (false, false) => None,
    }
}

/// Return whether every subject face is covered by exact coplanar cover clips.
///
/// The equality test is per subject triangle instead of whole-mesh only. That
/// makes the certificate antagonistic to false positives where one covered
/// triangle overcompensates for an uncovered triangle elsewhere, and it
/// preserves Yap's object/state boundary by replaying each retained face as
/// its own exact coverage claim.
#[cfg(feature = "exact-triangulation")]
fn coplanar_mesh_area_covered_by_mesh(
    subject: &ExactMesh,
    cover: &ExactMesh,
    projection: CoplanarProjection,
) -> Option<bool> {
    for subject_face in 0..subject.triangles().len() {
        let subject_triangle = single_face_mesh(subject, subject_face)?;
        let subject_points = mesh_points(&subject_triangle);
        let subject_area = projected_area2_abs(&subject_points, projection)?;
        if compare_reals(&subject_area, &ExactReal::from(0)).value() != Some(Ordering::Greater) {
            return Some(false);
        }
        let mut covered_area = ExactReal::from(0);
        for cover_face in 0..cover.triangles().len() {
            let cover_triangle = single_face_mesh(cover, cover_face)?;
            let Some((clip_projection, clip)) =
                pairwise_coplanar_triangle_intersection_polygon(&subject_triangle, &cover_triangle)
            else {
                continue;
            };
            if clip_projection != projection {
                return None;
            }
            let clip_area = projected_area2_abs(&clip, projection)?;
            if compare_reals(&clip_area, &ExactReal::from(0)).value() == Some(Ordering::Greater) {
                covered_area = add(&covered_area, &clip_area);
            }
        }
        if compare_reals(&covered_area, &subject_area).value() != Some(Ordering::Equal) {
            return Some(false);
        }
    }
    Some(true)
}

/// Reject cover/source meshes whose faces overlap in positive area.
///
/// Area-sum containment is only a coverage proof when same-mesh triangles are
/// interior-disjoint. Boundary contacts are fine: they carry no area in the
/// triangle-mesh result channel and match the retained triangulation model
/// used by exact open surfaces.
#[cfg(feature = "exact-triangulation")]
fn coplanar_mesh_faces_have_disjoint_interiors(
    mesh: &ExactMesh,
    projection: CoplanarProjection,
) -> Option<bool> {
    for left_face in 0..mesh.triangles().len() {
        let left_triangle = single_face_mesh(mesh, left_face)?;
        for right_face in left_face + 1..mesh.triangles().len() {
            let right_triangle = single_face_mesh(mesh, right_face)?;
            let Some((clip_projection, clip)) =
                pairwise_coplanar_triangle_intersection_polygon(&left_triangle, &right_triangle)
            else {
                continue;
            };
            if clip_projection != projection {
                return None;
            }
            let clip_area = projected_area2_abs(&clip, projection)?;
            if compare_reals(&clip_area, &ExactReal::from(0)).value() == Some(Ordering::Greater) {
                return Some(false);
            }
        }
    }
    Some(true)
}

#[cfg(feature = "exact-triangulation")]
#[derive(Clone, Debug)]
struct SourceHoledSurfaceComponent {
    outer: Vec<Point3>,
    holes: Vec<Vec<Point3>>,
}

/// Certify and materialize a bounded holed coplanar surface intersection.
///
/// This materializer covers the source-owned case left open by whole-mesh
/// containment: one operand replays as an open coplanar sheet with exact
/// boundary holes, while the other operand replays as simple coplanar source
/// disks lying strictly inside a source outer ring. A disk contributes an
/// output component only when it strictly contains at least one retained
/// source hole and is disjoint from every other source hole. Partial
/// hole/disk overlap, boundary contact, nested source holes, and any crossing
/// of a source outer boundary return `None`; those are general planar
/// arrangement inputs, not this certificate.
///
/// The retained rings are imported from mesh incidence, the disk/source
/// ownership facts are exact simple-polygon predicates, and the final area is
/// replayed by summing every pairwise triangle intersection. That is the
/// object/predicate separation advocated by Yap, "Towards Exact Geometric
/// Computation," *Computational Geometry* 7.1-2 (1997). Local triangle clips
/// use the exact Sutherland-Hodgman model already used by the convex
/// intersection path; see Sutherland and Hodgman, "Reentrant Polygon
/// Clipping," *Communications of the ACM* 17.1 (1974). Holed output
/// triangulation is delegated to `hypertri`'s exact earcut adapter, following
/// Held, "FIST: Fast Industrial-Strength Triangulation of Polygons,"
/// *Algorithmica* 30 (2001).
#[cfg(feature = "exact-triangulation")]
pub fn arrange_coplanar_surface_component_holed_intersection(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Option<CoplanarConvexComponentHoledArrangement> {
    arrange_coplanar_surface_component_holed_same_outer_intersection(left, right)
        .or_else(|| arrange_coplanar_surface_component_holed_intersection_oriented(left, right))
        .or_else(|| arrange_coplanar_surface_component_holed_intersection_oriented(right, left))
}

/// Intersect source-owned holed sheets that replay the same outer boundary.
///
/// This is the holed/holed sibling of the source-disk clip certificate. It
/// accepts only equal retained outer rings; the result keeps that outer ring
/// and the exact union of both operands' disjoint retained holes. Identical
/// holes are deduplicated, while touching, crossing, overlapping, or nested
/// non-identical holes reject to the general planar arrangement layer. This is
/// a legitimate Boolean intersection because the complement of each retained
/// hole is part of the source object: intersecting two equal-outer holed
/// sheets removes the union of their holes.
///
/// Boundary rings still come from exact mesh incidence and the final retained
/// area must equal the pairwise source-triangle intersection area. That is the
/// retained object/predicate split advocated by Yap, "Towards Exact Geometric
/// Computation," *Computational Geometry* 7.1-2 (1997). The local face clips
/// use Sutherland and Hodgman's half-plane clipping model from "Reentrant
/// Polygon Clipping," *Communications of the ACM* 17.1 (1974), and
/// triangulation follows Held, "FIST: Fast Industrial-Strength Triangulation
/// of Polygons," *Algorithmica* 30 (2001), through `hypertri`'s exact earcut
/// adapter.
#[cfg(feature = "exact-triangulation")]
fn arrange_coplanar_surface_component_holed_same_outer_intersection(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Option<CoplanarConvexComponentHoledArrangement> {
    if left.triangles().is_empty() || right.triangles().is_empty() {
        return None;
    }
    let (projection, left_components) = source_holed_surface_components_from_mesh(left)?;
    let (right_projection, right_components) = source_holed_surface_components_from_mesh(right)?;
    if projection != right_projection {
        return None;
    }
    if !coplanar_mesh_faces_have_disjoint_interiors(left, projection)?
        || !coplanar_mesh_faces_have_disjoint_interiors(right, projection)?
    {
        return None;
    }

    let mut retained_components = Vec::new();
    for left_component in &left_components {
        for right_component in &right_components {
            if polygons_equal(&left_component.outer, &right_component.outer) {
                let mut outer = left_component.outer.clone();
                orient_polygon_ccw(&mut outer, projection)?;
                let mut holes = merged_same_outer_intersection_holes(
                    left_component,
                    right_component,
                    projection,
                )?;
                sort_polygons_for_replay(&mut holes, projection);
                retained_components.push(CoplanarConvexHoledComponent { outer, holes });
                continue;
            }
            match simple_polygon_interaction(
                &left_component.outer,
                &right_component.outer,
                projection,
            )? {
                SimplePolygonInteraction::Disjoint => {}
                SimplePolygonInteraction::PointOnly | SimplePolygonInteraction::Connected => {
                    return None;
                }
            }
        }
    }
    if retained_components.is_empty() {
        return None;
    }
    sort_components_for_replay(&mut retained_components, projection);
    let retained_outers = retained_components
        .iter()
        .map(|component| component.outer.clone())
        .collect::<Vec<_>>();
    validate_simple_component_loops_disjoint(
        &retained_outers,
        projection,
        "coplanar same-outer component-holed intersection",
    )
    .ok()?;
    let intersection_area = coplanar_mesh_pairwise_intersection_area2(left, right, projection)?;
    let retained_area = component_holed_components_area2(&retained_components, projection)?;
    if compare_reals(&intersection_area, &retained_area).value() != Some(Ordering::Equal) {
        return None;
    }
    let mesh = component_holed_components_to_earcut_open_mesh(&retained_components, projection)?;
    let arrangement = CoplanarConvexComponentHoledArrangement {
        projection,
        components: retained_components,
        mesh,
    };
    arrangement.validate().ok()?;
    Some(arrangement)
}

#[cfg(feature = "exact-triangulation")]
fn merged_same_outer_intersection_holes(
    left: &SourceHoledSurfaceComponent,
    right: &SourceHoledSurfaceComponent,
    projection: CoplanarProjection,
) -> Option<Vec<Vec<Point3>>> {
    let mut holes = left.holes.clone();
    for hole in &mut holes {
        orient_polygon_cw(hole, projection)?;
    }
    for right_hole in &right.holes {
        if holes
            .iter()
            .any(|left_hole| polygons_equal(left_hole, right_hole))
        {
            continue;
        }
        for left_hole in &holes {
            match simple_polygon_interaction(left_hole, right_hole, projection)? {
                SimplePolygonInteraction::Disjoint => {}
                SimplePolygonInteraction::PointOnly | SimplePolygonInteraction::Connected => {
                    return None;
                }
            }
        }
        let mut retained = right_hole.clone();
        orient_polygon_cw(&mut retained, projection)?;
        holes.push(retained);
    }
    validate_component_loops_disjoint(
        &holes,
        projection,
        "coplanar same-outer component-holed intersection",
    )
    .ok()?;
    Some(holes)
}

#[cfg(feature = "exact-triangulation")]
fn arrange_coplanar_surface_component_holed_intersection_oriented(
    holed_source: &ExactMesh,
    clip_source: &ExactMesh,
) -> Option<CoplanarConvexComponentHoledArrangement> {
    if holed_source.triangles().is_empty() || clip_source.triangles().is_empty() {
        return None;
    }
    let (projection, holed_components) = source_holed_surface_components_from_mesh(holed_source)?;
    if !coplanar_mesh_faces_have_disjoint_interiors(holed_source, projection)?
        || !coplanar_mesh_faces_have_disjoint_interiors(clip_source, projection)?
    {
        return None;
    }
    let clip_components = connected_face_component_meshes(clip_source)?
        .into_iter()
        .map(SimpleSurfaceComponent::from_mesh)
        .collect::<Option<Vec<_>>>()?;
    if clip_components.is_empty()
        || clip_components
            .iter()
            .any(|component| component.projection != projection)
    {
        return None;
    }
    let clip_boundaries = clip_components
        .iter()
        .map(|component| component.boundary.clone())
        .collect::<Vec<_>>();
    validate_simple_component_loops_disjoint(
        &clip_boundaries,
        projection,
        "coplanar component-holed intersection clip source",
    )
    .ok()?;

    let mut retained_components = Vec::new();
    for clip_component in &clip_components {
        let mut owner = None;
        let mut touched_outer = false;
        for (source_index, source_component) in holed_components.iter().enumerate() {
            if simple_polygon_interaction(
                &clip_component.boundary,
                &source_component.outer,
                projection,
            )? != SimplePolygonInteraction::Disjoint
            {
                touched_outer = true;
            }
            if polygon_strictly_inside_simple_polygon(
                &clip_component.boundary,
                &source_component.outer,
                projection,
            )? {
                if owner.is_some() {
                    return None;
                }
                owner = Some(source_index);
            }
        }
        let Some(owner) = owner else {
            if touched_outer {
                return None;
            }
            continue;
        };
        let source_component = &holed_components[owner];
        let mut retained_holes = Vec::new();
        for hole in &source_component.holes {
            if polygon_strictly_inside_simple_polygon(hole, &clip_component.boundary, projection)? {
                let mut retained = hole.clone();
                orient_polygon_cw(&mut retained, projection)?;
                retained_holes.push(retained);
                continue;
            }
            match simple_polygon_interaction(hole, &clip_component.boundary, projection)? {
                SimplePolygonInteraction::Disjoint => {}
                SimplePolygonInteraction::PointOnly | SimplePolygonInteraction::Connected => {
                    return None;
                }
            }
        }
        if retained_holes.is_empty() {
            return None;
        }
        let mut outer = clip_component.boundary.clone();
        orient_polygon_ccw(&mut outer, projection)?;
        sort_polygons_for_replay(&mut retained_holes, projection);
        retained_components.push(CoplanarConvexHoledComponent {
            outer,
            holes: retained_holes,
        });
    }
    if retained_components.is_empty() {
        return None;
    }
    sort_components_for_replay(&mut retained_components, projection);
    let intersection_area =
        coplanar_mesh_pairwise_intersection_area2(holed_source, clip_source, projection)?;
    let retained_area = component_holed_components_area2(&retained_components, projection)?;
    if compare_reals(&intersection_area, &retained_area).value() != Some(Ordering::Equal) {
        return None;
    }
    let mesh = component_holed_components_to_earcut_open_mesh(&retained_components, projection)?;
    let arrangement = CoplanarConvexComponentHoledArrangement {
        projection,
        components: retained_components,
        mesh,
    };
    arrangement.validate().ok()?;
    Some(arrangement)
}

/// Import source-owned holed components from exact mesh boundary rings.
///
/// A connected component is accepted only when one recovered boundary loop
/// strictly contains all other recovered loops. The outer loop is oriented
/// counter-clockwise, hole loops are oriented clockwise, and exact mesh area
/// must equal `outer - holes`. This prevents a locally plausible ring set from
/// standing in for a different triangulated source, preserving Yap's retained
/// topology contract.
#[cfg(feature = "exact-triangulation")]
fn source_holed_surface_components_from_mesh(
    mesh: &ExactMesh,
) -> Option<(CoplanarProjection, Vec<SourceHoledSurfaceComponent>)> {
    let component_meshes = connected_face_component_meshes(mesh)?;
    let mut projection = None;
    let mut components = Vec::new();
    for component_mesh in component_meshes {
        if component_mesh.triangles().is_empty() {
            return None;
        }
        for face in 0..component_mesh.triangles().len() {
            let classification = classify_mesh_triangle_against_retained_face_plane(
                &component_mesh,
                0,
                &component_mesh,
                face,
            )
            .ok()?;
            if classification.relation != TrianglePlaneRelation::Coplanar {
                return None;
            }
        }
        let component_projection = choose_mesh_projection(&component_mesh)?;
        match projection {
            Some(expected) if expected != component_projection => return None,
            None => projection = Some(component_projection),
            Some(_) => {}
        }
        let rings = order_mesh_boundary_loops(&component_mesh)?;
        if rings.len() < 2 {
            return None;
        }
        let mut ring_points = rings
            .into_iter()
            .map(|ring| {
                let mut points = ring
                    .into_iter()
                    .map(|index| {
                        component_mesh
                            .vertices()
                            .get(index)
                            .map(ExactPoint3::to_hyperlimit_point)
                    })
                    .collect::<Option<Vec<_>>>()?;
                points = simplify_projected_polygon(points, component_projection);
                validate_projected_simple_loop(
                    &points,
                    component_projection,
                    "coplanar source-holed surface component",
                )
                .ok()?;
                Some(points)
            })
            .collect::<Option<Vec<_>>>()?;

        let mut outer_index = None;
        for candidate in 0..ring_points.len() {
            let mut candidate_outer = ring_points[candidate].clone();
            orient_polygon_ccw(&mut candidate_outer, component_projection)?;
            let mut contains_all = true;
            for (other_index, other) in ring_points.iter().enumerate() {
                if other_index == candidate {
                    continue;
                }
                if !polygon_strictly_inside_simple_polygon(
                    other,
                    &candidate_outer,
                    component_projection,
                )? {
                    contains_all = false;
                    break;
                }
            }
            if contains_all {
                if outer_index.is_some() {
                    return None;
                }
                outer_index = Some(candidate);
            }
        }
        let outer_index = outer_index?;
        let mut outer = ring_points.swap_remove(outer_index);
        orient_polygon_ccw(&mut outer, component_projection)?;
        let mut holes = ring_points;
        for hole in &mut holes {
            orient_polygon_cw(hole, component_projection)?;
            validate_projected_strictly_convex_loop(
                hole,
                component_projection,
                "coplanar source-holed surface component",
            )
            .ok()?;
        }
        validate_component_loops_disjoint(
            &holes,
            component_projection,
            "coplanar source-holed surface component",
        )
        .ok()?;
        let component = SourceHoledSurfaceComponent { outer, holes };
        let retained_area = component_holed_component_area2(
            &CoplanarConvexHoledComponent {
                outer: component.outer.clone(),
                holes: component.holes.clone(),
            },
            component_projection,
        )?;
        let mesh_area = mesh_projected_area2(&component_mesh, component_projection)?;
        if compare_reals(&mesh_area, &retained_area).value() != Some(Ordering::Equal) {
            return None;
        }
        components.push(component);
    }
    if components.is_empty()
        || components
            .iter()
            .all(|component| component.holes.is_empty())
    {
        return None;
    }
    let projection = projection?;
    let outers = components
        .iter()
        .map(|component| component.outer.clone())
        .collect::<Vec<_>>();
    validate_simple_component_loops_disjoint(
        &outers,
        projection,
        "coplanar source-holed surface components",
    )
    .ok()?;
    Some((projection, components))
}

#[cfg(feature = "exact-triangulation")]
fn component_holed_components_area2(
    components: &[CoplanarConvexHoledComponent],
    projection: CoplanarProjection,
) -> Option<ExactReal> {
    components
        .iter()
        .try_fold(ExactReal::from(0), |area, component| {
            Some(add(
                &area,
                &component_holed_component_area2(component, projection)?,
            ))
        })
}

#[cfg(feature = "exact-triangulation")]
fn component_holed_component_area2(
    component: &CoplanarConvexHoledComponent,
    projection: CoplanarProjection,
) -> Option<ExactReal> {
    let outer_area = projected_area2_abs(&component.outer, projection)?;
    let hole_area = component
        .holes
        .iter()
        .try_fold(ExactReal::from(0), |area, hole| {
            Some(add(&area, &projected_area2_abs(hole, projection)?))
        })?;
    if compare_reals(&outer_area, &hole_area).value() != Some(Ordering::Greater) {
        return None;
    }
    Some(sub(&outer_area, &hole_area))
}

#[cfg(feature = "exact-triangulation")]
fn coplanar_mesh_pairwise_intersection_area2(
    left: &ExactMesh,
    right: &ExactMesh,
    projection: CoplanarProjection,
) -> Option<ExactReal> {
    let mut area = ExactReal::from(0);
    for left_face in 0..left.triangles().len() {
        let left_triangle = single_face_mesh(left, left_face)?;
        for right_face in 0..right.triangles().len() {
            let right_triangle = single_face_mesh(right, right_face)?;
            let Some((clip_projection, clip)) =
                pairwise_coplanar_triangle_intersection_polygon(&left_triangle, &right_triangle)
            else {
                continue;
            };
            if clip_projection != projection {
                return None;
            }
            let clip_area = projected_area2_abs(&clip, projection)?;
            if compare_reals(&clip_area, &ExactReal::from(0)).value() == Some(Ordering::Greater) {
                area = add(&area, &clip_area);
            }
        }
    }
    Some(area)
}

/// Materialize bounded same-outer holed surface differences.
///
/// When two source-owned holed surfaces replay the same exact outer boundary,
/// `(outer - left_holes) - (outer - right_holes)` equals the portion of the
/// right holes not removed by the left holes. This certificate accepts the
/// nested/disjoint case only: every emitted component is a right retained hole
/// boundary, with strict left retained holes nested inside it. Identical
/// right/left holes and right holes strictly inside a left hole contribute no
/// area; partial overlap, point contact, edge contact, and crossing hole
/// boundaries reject to the general planar arrangement layer.
///
/// The source rings are recovered by exact mesh incidence, and the final area
/// equation is replayed as `area(left) - area(left ∩ right) == area(output)`.
/// That follows Yap, "Towards Exact Geometric Computation," *Computational
/// Geometry* 7.1-2 (1997): output topology is emitted only with retained
/// source objects and exact predicate/area proof. Local face intersections use
/// Sutherland and Hodgman, "Reentrant Polygon Clipping," *Communications of
/// the ACM* 17.1 (1974), and holed output triangulation follows Held, "FIST:
/// Fast Industrial-Strength Triangulation of Polygons," *Algorithmica* 30
/// (2001), through `hypertri`'s exact earcut adapter.
#[cfg(feature = "exact-triangulation")]
pub fn arrange_coplanar_surface_component_holed_difference(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Option<CoplanarConvexComponentHoledArrangement> {
    if left.triangles().is_empty() || right.triangles().is_empty() {
        return None;
    }
    let (projection, left_components) = source_holed_surface_components_from_mesh(left)?;
    let (right_projection, right_components) = source_holed_surface_components_from_mesh(right)?;
    if projection != right_projection {
        return None;
    }
    if !coplanar_mesh_faces_have_disjoint_interiors(left, projection)?
        || !coplanar_mesh_faces_have_disjoint_interiors(right, projection)?
    {
        return None;
    }

    let mut retained_components = Vec::new();
    for left_component in &left_components {
        for right_component in &right_components {
            if polygons_equal(&left_component.outer, &right_component.outer) {
                retained_components.extend(same_outer_holed_difference_components(
                    left_component,
                    right_component,
                    projection,
                )?);
                continue;
            }
            match simple_polygon_interaction(
                &left_component.outer,
                &right_component.outer,
                projection,
            )? {
                SimplePolygonInteraction::Disjoint => {}
                SimplePolygonInteraction::PointOnly | SimplePolygonInteraction::Connected => {
                    return None;
                }
            }
        }
    }
    if retained_components.is_empty()
        || !retained_components
            .iter()
            .any(|component| !component.holes.is_empty())
    {
        return None;
    }
    sort_components_for_replay(&mut retained_components, projection);
    validate_simple_component_loops_disjoint_allowing_vertex_point_touches(
        &retained_components
            .iter()
            .map(|component| component.outer.clone())
            .collect::<Vec<_>>(),
        projection,
        "coplanar same-outer component-holed difference",
    )
    .ok()?;

    let left_area = mesh_projected_area2(left, projection)?;
    let intersection_area = coplanar_mesh_pairwise_intersection_area2(left, right, projection)?;
    let difference_area = sub(&left_area, &intersection_area);
    let retained_area = component_holed_components_area2(&retained_components, projection)?;
    if compare_reals(&difference_area, &retained_area).value() != Some(Ordering::Equal) {
        return None;
    }
    let mesh = component_holed_components_to_earcut_open_mesh(&retained_components, projection)?;
    let arrangement = CoplanarConvexComponentHoledArrangement {
        projection,
        components: retained_components,
        mesh,
    };
    arrangement.validate().ok()?;
    Some(arrangement)
}

#[cfg(feature = "exact-triangulation")]
fn same_outer_holed_difference_components(
    left: &SourceHoledSurfaceComponent,
    right: &SourceHoledSurfaceComponent,
    projection: CoplanarProjection,
) -> Option<Vec<CoplanarConvexHoledComponent>> {
    let mut components = Vec::new();
    for right_hole in &right.holes {
        let mut swallowed = false;
        let mut holes = Vec::new();
        for left_hole in &left.holes {
            if polygons_equal(left_hole, right_hole)
                || polygon_strictly_inside_simple_polygon(right_hole, left_hole, projection)?
            {
                swallowed = true;
                break;
            }
            if polygon_strictly_inside_simple_polygon(left_hole, right_hole, projection)? {
                let mut retained = left_hole.clone();
                orient_polygon_cw(&mut retained, projection)?;
                holes.push(retained);
                continue;
            }
            match simple_polygon_interaction(left_hole, right_hole, projection)? {
                SimplePolygonInteraction::Disjoint => {}
                SimplePolygonInteraction::PointOnly | SimplePolygonInteraction::Connected => {
                    return None;
                }
            }
        }
        if swallowed {
            continue;
        }
        let mut outer = right_hole.clone();
        orient_polygon_ccw(&mut outer, projection)?;
        sort_polygons_for_replay(&mut holes, projection);
        components.push(CoplanarConvexHoledComponent { outer, holes });
    }
    Some(components)
}

/// Replay same-outer holed differences whose output contains only filled holes.
///
/// This is the no-hole sibling of
/// [`arrange_coplanar_surface_component_holed_difference`]. For two holed
/// sheets with the same retained outer ring,
/// `(outer - left_holes) - (outer - right_holes)` includes each right-side
/// retained hole that is disjoint from every left-side retained hole. Those
/// holes become ordinary filled output components, not retained holes. If a
/// right hole strictly contains a left hole, the output is holed and belongs to
/// the component-holed artifact instead; if rings overlap or touch, the case
/// remains general planar-arrangement work.
///
/// The certificate follows Yap, "Towards Exact Geometric Computation,"
/// *Computational Geometry* 7.1-2 (1997): source boundary rings are recovered
/// from exact mesh incidence, topology changes are named by retained source
/// rings, and the result is accepted only after exact area replay. The area
/// replay sums pairwise Sutherland-Hodgman triangle clips (Sutherland and
/// Hodgman, "Reentrant Polygon Clipping," *Communications of the ACM* 17.1,
/// 1974) and triangulates retained simple loops through the same FIST-style
/// earcut handoff described by Held, "FIST: Fast Industrial-Strength
/// Triangulation of Polygons," *Algorithmica* 30 (2001).
#[cfg(feature = "exact-triangulation")]
fn same_outer_holed_no_hole_difference_polygons(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Option<(CoplanarProjection, Vec<Vec<Point3>>)> {
    if left.triangles().is_empty() || right.triangles().is_empty() {
        return None;
    }
    let (projection, left_components) = source_holed_surface_components_from_mesh(left)?;
    let (right_projection, right_components) = source_holed_surface_components_from_mesh(right)?;
    if projection != right_projection {
        return None;
    }
    if !coplanar_mesh_faces_have_disjoint_interiors(left, projection)?
        || !coplanar_mesh_faces_have_disjoint_interiors(right, projection)?
    {
        return None;
    }

    let mut polygons = Vec::new();
    for left_component in &left_components {
        for right_component in &right_components {
            if polygons_equal(&left_component.outer, &right_component.outer) {
                polygons.extend(same_outer_holed_no_hole_difference_rings(
                    left_component,
                    right_component,
                    projection,
                )?);
                continue;
            }
            match simple_polygon_interaction(
                &left_component.outer,
                &right_component.outer,
                projection,
            )? {
                SimplePolygonInteraction::Disjoint => {}
                SimplePolygonInteraction::PointOnly | SimplePolygonInteraction::Connected => {
                    return None;
                }
            }
        }
    }
    if polygons.is_empty() {
        return None;
    }
    sort_polygons_for_replay(&mut polygons, projection);
    validate_simple_component_loops_disjoint(
        &polygons,
        projection,
        "coplanar same-outer holed no-hole difference",
    )
    .ok()?;

    let left_area = mesh_projected_area2(left, projection)?;
    let intersection_area = coplanar_mesh_pairwise_intersection_area2(left, right, projection)?;
    let difference_area = sub(&left_area, &intersection_area);
    let retained_area = polygons
        .iter()
        .try_fold(ExactReal::from(0), |area, polygon| {
            Some(add(&area, &projected_area2_abs(polygon, projection)?))
        })?;
    if compare_reals(&difference_area, &retained_area).value() != Some(Ordering::Equal) {
        return None;
    }
    Some((projection, polygons))
}

#[cfg(feature = "exact-triangulation")]
fn same_outer_holed_no_hole_difference_rings(
    left: &SourceHoledSurfaceComponent,
    right: &SourceHoledSurfaceComponent,
    projection: CoplanarProjection,
) -> Option<Vec<Vec<Point3>>> {
    let mut polygons = Vec::new();
    for right_hole in &right.holes {
        let mut contributes = true;
        for left_hole in &left.holes {
            if polygons_equal(left_hole, right_hole)
                || polygon_strictly_inside_simple_polygon(right_hole, left_hole, projection)?
            {
                contributes = false;
                break;
            }
            if polygon_strictly_inside_simple_polygon(left_hole, right_hole, projection)? {
                return None;
            }
            match simple_polygon_interaction(left_hole, right_hole, projection)? {
                SimplePolygonInteraction::Disjoint => {}
                SimplePolygonInteraction::PointOnly | SimplePolygonInteraction::Connected => {
                    return None;
                }
            }
        }
        if contributes {
            let mut polygon = right_hole.clone();
            orient_polygon_ccw(&mut polygon, projection)?;
            polygons.push(polygon);
        }
    }
    Some(polygons)
}

/// Replay same-outer holed unions whose holes are completely filled.
///
/// For two source-owned holed sheets with the same exact outer ring,
/// `(outer - left_holes) union (outer - right_holes)` is `outer` when every
/// left retained hole is strictly disjoint from every right retained hole. The
/// result is an ordinary simple-loop surface, so the existing component-union
/// artifact can carry it without inventing a new report. Equal, nested,
/// touching, crossing, or overlapping holes are deliberately rejected here:
/// equal/nested cases are copy/containment-shaped, while contact and overlap
/// remain planar-arrangement work.
///
/// The certificate follows Yap, "Towards Exact Geometric Computation,"
/// *Computational Geometry* 7.1-2 (1997): source rings are recovered from mesh
/// incidence, every topological claim is checked by exact predicates, and the
/// final retained outer area is replayed as
/// `area(left) + area(right) - area(left intersect right)`. The intersection
/// term is the existing exact Sutherland-Hodgman triangle clip sum
/// (Sutherland and Hodgman, "Reentrant Polygon Clipping," *Communications of
/// the ACM* 17.1, 1974); triangulation of the retained simple loop uses the
/// Held FIST-style `hypertri` handoff (Held, "FIST: Fast Industrial-Strength
/// Triangulation of Polygons," *Algorithmica* 30, 2001).
#[cfg(feature = "exact-triangulation")]
fn same_outer_holed_filled_union_polygons(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Option<(CoplanarProjection, Vec<Vec<Point3>>)> {
    if left.triangles().is_empty() || right.triangles().is_empty() {
        return None;
    }
    let (projection, left_components) = source_holed_surface_components_from_mesh(left)?;
    let (right_projection, right_components) = source_holed_surface_components_from_mesh(right)?;
    if projection != right_projection {
        return None;
    }
    if !coplanar_mesh_faces_have_disjoint_interiors(left, projection)?
        || !coplanar_mesh_faces_have_disjoint_interiors(right, projection)?
    {
        return None;
    }

    let mut polygons = Vec::new();
    for left_component in &left_components {
        for right_component in &right_components {
            if polygons_equal(&left_component.outer, &right_component.outer) {
                same_outer_holes_are_strictly_cross_disjoint(
                    &left_component.holes,
                    &right_component.holes,
                    projection,
                )?;
                let mut outer = left_component.outer.clone();
                orient_polygon_ccw(&mut outer, projection)?;
                polygons.push(outer);
                continue;
            }
            match simple_polygon_interaction(
                &left_component.outer,
                &right_component.outer,
                projection,
            )? {
                SimplePolygonInteraction::Disjoint => {}
                SimplePolygonInteraction::PointOnly | SimplePolygonInteraction::Connected => {
                    return None;
                }
            }
        }
    }
    if polygons.is_empty() {
        return None;
    }
    sort_polygons_for_replay(&mut polygons, projection);
    validate_simple_component_loops_disjoint(
        &polygons,
        projection,
        "coplanar same-outer holed filled union",
    )
    .ok()?;

    let left_area = mesh_projected_area2(left, projection)?;
    let right_area = mesh_projected_area2(right, projection)?;
    let intersection_area = coplanar_mesh_pairwise_intersection_area2(left, right, projection)?;
    let union_area = sub(&add(&left_area, &right_area), &intersection_area);
    let retained_area = polygons
        .iter()
        .try_fold(ExactReal::from(0), |area, polygon| {
            Some(add(&area, &projected_area2_abs(polygon, projection)?))
        })?;
    if compare_reals(&union_area, &retained_area).value() != Some(Ordering::Equal) {
        return None;
    }
    Some((projection, polygons))
}

#[cfg(feature = "exact-triangulation")]
fn same_outer_holes_are_strictly_cross_disjoint(
    left_holes: &[Vec<Point3>],
    right_holes: &[Vec<Point3>],
    projection: CoplanarProjection,
) -> Option<()> {
    for left_hole in left_holes {
        for right_hole in right_holes {
            if polygons_equal(left_hole, right_hole)
                || polygon_strictly_inside_simple_polygon(left_hole, right_hole, projection)?
                || polygon_strictly_inside_simple_polygon(right_hole, left_hole, projection)?
            {
                return None;
            }
            match simple_polygon_interaction(left_hole, right_hole, projection)? {
                SimplePolygonInteraction::Disjoint => {}
                SimplePolygonInteraction::PointOnly | SimplePolygonInteraction::Connected => {
                    return None;
                }
            }
        }
    }
    Some(())
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

#[cfg(feature = "exact-triangulation")]
fn coplanar_surface_pairwise_triangle_intersection_polygons(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Option<(CoplanarProjection, Vec<Vec<Point3>>)> {
    if arrange_coplanar_convex_surface_intersection(left, right).is_some()
        || arrange_coplanar_convex_surface_multi_intersection(left, right).is_some()
        || intersect_single_triangle_coplanar_surfaces(left, right).is_some()
    {
        return None;
    }

    let mut projection = None;
    let mut clips = Vec::new();
    for left_face in 0..left.triangles().len() {
        let left_triangle = single_face_mesh(left, left_face)?;
        for right_face in 0..right.triangles().len() {
            let right_triangle = single_face_mesh(right, right_face)?;
            let Some((intersection_projection, mut polygon)) =
                pairwise_coplanar_triangle_intersection_polygon(&left_triangle, &right_triangle)
            else {
                continue;
            };
            match projection {
                Some(expected) if expected != intersection_projection => return None,
                None => projection = Some(intersection_projection),
                _ => {}
            }
            orient_polygon_ccw(&mut polygon, intersection_projection)?;
            clips.push(polygon);
        }
    }
    if clips.len() < 2 {
        return None;
    }
    let projection = projection?;

    let mut contact_graph = UnionFind::new(clips.len());
    for left_index in 0..clips.len() {
        for right_index in left_index + 1..clips.len() {
            let relation = convex_union_component_relation(
                &clips[left_index],
                &clips[right_index],
                projection,
            );
            match relation? {
                ConvexUnionComponentRelation::Disjoint => {}
                ConvexUnionComponentRelation::BoundaryOnly => {
                    if convex_polygons_touch_on_positive_boundary(
                        &clips[left_index],
                        &clips[right_index],
                        projection,
                    )
                    .unwrap_or(false)
                    {
                        contact_graph.union(left_index, right_index);
                    }
                }
                ConvexUnionComponentRelation::PositiveArea => {
                    contact_graph.union(left_index, right_index);
                }
            }
        }
    }

    let mut groups: Vec<(usize, Vec<usize>)> = Vec::new();
    for index in 0..clips.len() {
        let root = contact_graph.find(index);
        if let Some((_, members)) = groups.iter_mut().find(|(candidate, _)| *candidate == root) {
            members.push(index);
        } else {
            groups.push((root, vec![index]));
        }
    }

    let mut polygons = Vec::with_capacity(groups.len());
    for (_, members) in groups {
        let mut polygon = if members.len() == 1 {
            clips[members[0]].clone()
        } else {
            let regions = members
                .iter()
                .map(|&member| clips[member].clone())
                .collect::<Vec<_>>();
            connected_convex_face_cell_union_polygon(&regions, projection)?
        };
        orient_polygon_ccw(&mut polygon, projection)?;
        polygon = simplify_projected_polygon(polygon, projection);
        validate_projected_simple_loop(&polygon, projection, "coplanar nonconvex intersection")
            .ok()?;
        polygons.push(polygon);
    }
    sort_polygons_for_replay(&mut polygons, projection);
    validate_simple_component_loops_disjoint(
        &polygons,
        projection,
        "coplanar nonconvex intersection",
    )
    .ok()?;
    Some((projection, polygons))
}

#[cfg(feature = "exact-triangulation")]
fn pairwise_coplanar_triangle_intersection_polygon(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Option<(CoplanarProjection, Vec<Point3>)> {
    if let Some(intersection) = intersect_single_triangle_coplanar_surfaces(left, right) {
        return Some((intersection.projection, intersection.polygon));
    }
    let containment = certify_single_triangle_coplanar_containment(left, right)?;
    let projection = choose_mesh_projection(left).or_else(|| choose_mesh_projection(right))?;
    let mut polygon = match containment {
        CoplanarSurfaceContainment::LeftInsideRight => left.triangles()[0]
            .0
            .iter()
            .map(|&index| left.vertices()[index].to_hyperlimit_point())
            .collect::<Vec<_>>(),
        CoplanarSurfaceContainment::RightInsideLeft => right.triangles()[0]
            .0
            .iter()
            .map(|&index| right.vertices()[index].to_hyperlimit_point())
            .collect::<Vec<_>>(),
    };
    orient_polygon_ccw(&mut polygon, projection)?;
    Some((projection, polygon))
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

/// A retained point-touch component for branch-only coplanar unions.
///
/// Convex inputs keep using [`ConvexUnionComponent`] in the original fast
/// path. This type is the nonconvex-capable fallback: each connected source
/// component is imported either as its exact convex hull or as one certified
/// simple source boundary from [`SimpleSurfaceComponent`]. The output remains
/// a list of source-owned disks, so it follows Yap's retained-object model
/// from "Towards Exact Geometric Computation," *Computational Geometry*
/// 7.1-2 (1997), instead of collapsing a branch vertex into an opaque
/// triangle soup.
#[cfg(feature = "exact-triangulation")]
#[derive(Clone, Debug)]
struct PointTouchSourceComponent {
    side: MultiUnionSide,
    projection: CoplanarProjection,
    polygon: Vec<Point3>,
}

#[cfg(feature = "exact-triangulation")]
impl PointTouchSourceComponent {
    fn from_mesh(side: MultiUnionSide, mesh: ExactMesh) -> Option<Self> {
        if let Some(component) = ConvexUnionComponent::from_mesh(side, mesh.clone()) {
            return Some(Self {
                side,
                projection: component.projection,
                polygon: component.hull,
            });
        }
        let component = SimpleSurfaceComponent::from_mesh(mesh)?;
        Some(Self {
            side,
            projection: component.projection,
            polygon: component.boundary,
        })
    }
}

/// A connected coplanar source component with one retained simple boundary.
///
/// This is the nonconvex-source counterpart to [`ConvexUnionComponent`]. It is
/// deliberately a disk-only object: the exact mesh boundary must form one
/// degree-two ring, the projected ring must be simple, and the ring area must
/// equal the sum of source-triangle areas. That is the Yap-style certificate
/// boundary for consuming a triangulated source sheet without first replacing
/// it by its convex hull; see Yap, "Towards Exact Geometric Computation,"
/// *Computational Geometry* 7.1-2 (1997). The boundary ordering itself is
/// pure mesh topology, not coordinate clustering.
#[cfg(feature = "exact-triangulation")]
#[derive(Clone, Debug)]
struct SimpleSurfaceComponent {
    projection: CoplanarProjection,
    boundary: Vec<Point3>,
    area2_abs: ExactReal,
}

#[cfg(feature = "exact-triangulation")]
impl SimpleSurfaceComponent {
    fn from_mesh(mesh: ExactMesh) -> Option<Self> {
        if mesh.triangles().is_empty() {
            return None;
        }
        for face in 0..mesh.triangles().len() {
            let classification =
                classify_mesh_triangle_against_retained_face_plane(&mesh, 0, &mesh, face).ok()?;
            if classification.relation != TrianglePlaneRelation::Coplanar {
                return None;
            }
        }
        let projection = choose_mesh_projection(&mesh)?;
        let mut boundary = order_single_mesh_boundary_loop(&mesh)?
            .into_iter()
            .map(|index| {
                mesh.vertices()
                    .get(index)
                    .map(ExactPoint3::to_hyperlimit_point)
            })
            .collect::<Option<Vec<_>>>()?;
        orient_polygon_ccw(&mut boundary, projection)?;
        boundary = simplify_projected_polygon(boundary, projection);
        validate_projected_simple_loop(
            &boundary,
            projection,
            "coplanar nonconvex source component boundary",
        )
        .ok()?;
        let boundary_area = projected_area2_abs(&boundary, projection)?;
        let mesh_area = mesh_projected_area2(&mesh, projection)?;
        if compare_reals(&mesh_area, &boundary_area).value() != Some(Ordering::Equal) {
            return None;
        }
        Some(Self {
            projection,
            boundary,
            area2_abs: boundary_area,
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

#[cfg(feature = "exact-triangulation")]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum VertexPointContactRelation {
    Disjoint,
    PointOnly,
    InvalidBoundaryContact,
}

#[cfg(feature = "exact-triangulation")]
#[derive(Clone, Debug)]
struct VertexPointContactPlan {
    relation: VertexPointContactRelation,
    left_split_points: Vec<Point3>,
    right_split_points: Vec<Point3>,
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
        _ => materialize_component_union_convex_hull_by_area(components, members)
            .or_else(|| materialize_rectangle_strip_union_cluster(components, members)),
    }
}

/// Materialize a many-component union when exact area proves convex coverage.
///
/// This is the non-rectangular sibling of the rectangle-strip certificate
/// below. Every component is already a certified convex coplanar sheet. For a
/// cluster with three or more members, we accept the convex hull only when
/// exact pairwise component relations prove there is no positive-area overlap
/// and the sum of retained component areas equals the exact hull area. Because
/// all components are subsets of that hull, area equality proves there is no
/// gap. The argument is the same retained-object discipline Yap requires in
/// "Towards Exact Geometric Computation," *Computational Geometry* 7.1-2
/// (1997): topology is promoted from exact structural facts, not from a
/// primitive-float polygon repair pass. The hull itself uses Andrew,
/// "Another Efficient Algorithm for Convex Hulls in Two Dimensions,"
/// *Information Processing Letters* 9.5 (1979).
#[cfg(feature = "exact-triangulation")]
fn materialize_component_union_convex_hull_by_area(
    components: &[ConvexUnionComponent],
    members: &[usize],
) -> Option<Vec<Point3>> {
    if members.len() < 3 {
        return None;
    }
    let projection = components[*members.first()?].projection;
    if members
        .iter()
        .any(|&member| components[member].projection != projection)
    {
        return None;
    }

    let mut boundary_contacts = 0;
    for left in 0..members.len() {
        for right in left + 1..members.len() {
            match convex_union_component_relation(
                &components[members[left]].hull,
                &components[members[right]].hull,
                projection,
            )? {
                ConvexUnionComponentRelation::Disjoint => {}
                ConvexUnionComponentRelation::BoundaryOnly => boundary_contacts += 1,
                ConvexUnionComponentRelation::PositiveArea => return None,
            }
        }
    }
    if boundary_contacts == 0 {
        return None;
    }

    let points = members
        .iter()
        .flat_map(|&member| components[member].hull.iter().cloned())
        .collect::<Vec<_>>();
    let mut hull = convex_hull_3d(points, projection)?;
    orient_polygon_ccw(&mut hull, projection)?;
    validate_retained_convex_hull(
        "coplanar convex component-union hull coverage",
        &hull,
        projection,
    )
    .ok()?;
    let hull_area = projected_area2_abs(&hull, projection)?;
    let mut component_area = ExactReal::from(0);
    for &member in members {
        component_area = add(
            &component_area,
            &projected_area2_abs(&components[member].hull, projection)?,
        );
    }
    if compare_reals(&component_area, &hull_area).value() == Some(Ordering::Equal) {
        Some(hull)
    } else {
        None
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
/// replay proves a positive-area sheet; point-only contacts are left to the
/// explicit point-touch union artifact. The traversal follows the Weiler-Atherton
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
/// strip; point-only contacts are handled by the explicit point-touch artifact,
/// while non-convex component loops and cases requiring a general planar
/// subdivision remain explicit arrangement work. The
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

/// Certify and materialize one nonconvex component-union loop.
///
/// This is a bounded step toward the remaining general planar arrangement
/// scope. Source topology is first split into disjoint convex components. If
/// all components form one connected graph through cross-source positive-area
/// overlaps or positive-length boundary contacts, their exposed boundary
/// fragments are stitched into one simple loop and checked by exact finite
/// inclusion-exclusion area replay. Same-source overlaps, point-only contacts
/// outside the explicit point-touch artifact, disconnected clusters, convex
/// outputs, holes, and branch cases that do not stitch into one loop stay on
/// the explicit planar-arrangement boundary.
/// A second source-holed path accepts the same retained simple-loop artifact
/// when two same-outer holed sheets have strictly disjoint retained holes and
/// therefore union to the filled outer boundary. That case may be convex, but
/// it still requires source-holed ring replay and exact union-area equality,
/// so it is not the ordinary convex component-union shortcut.
///
/// Boundary traversal follows the Weiler-Atherton fragment idea from Weiler
/// and Atherton, "Hidden Surface Removal Using Polygon Area Sorting,"
/// *SIGGRAPH Computer Graphics* 11.2 (1977), while the finite exact union-area
/// certificate keeps the output inside Yap's retained-state model from
/// "Towards Exact Geometric Computation," *Computational Geometry* 7.1-2
/// (1997).
#[cfg(feature = "exact-triangulation")]
pub fn arrange_coplanar_surface_component_union(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Option<CoplanarSurfaceArrangement> {
    let source_holed_filled_union = same_outer_holed_filled_union_polygons(left, right);
    let (projection, mut polygons, allow_convex, label) =
        if let Some((projection, polygons)) = source_holed_filled_union {
            (
                projection,
                polygons,
                true,
                "exact coplanar same-outer filled union",
            )
        } else {
            let (projection, polygon) = coplanar_surface_component_union_loop(left, right)?;
            (
                projection,
                vec![polygon],
                false,
                "exact coplanar nonconvex component union",
            )
        };
    if polygons.len() != 1 {
        return None;
    }
    let mut polygon = polygons.pop()?;
    if !allow_convex
        && validate_projected_strictly_convex_loop(
            &polygon,
            projection,
            "coplanar nonconvex component union",
        )
        .is_ok()
    {
        return None;
    }
    orient_polygon_ccw(&mut polygon, projection)?;
    let mesh = polygon_to_earcut_open_mesh_with_label(&polygon, projection, label)?;
    let arrangement = CoplanarSurfaceArrangement {
        projection,
        polygon,
        mesh,
    };
    arrangement.validate().ok()?;
    Some(arrangement)
}

/// Certify and materialize disconnected nonconvex component-union loops.
///
/// This is the multi-loop counterpart to
/// [`arrange_coplanar_surface_component_union`]. Exact source topology is
/// decomposed into disjoint convex components, cross-source positive-area
/// overlaps and positive-length boundary contacts form connected clusters, and
/// each non-singleton cluster is stitched from exposed convex boundary
/// fragments. The public artifact is accepted only when at least two
/// component loops remain and at least one loop is nonconvex; all-convex
/// outputs stay on [`arrange_coplanar_convex_surface_multi_union`], while
/// same-source overlap, holes, branch vertices beyond the explicit
/// point-touch artifact, and non-simple stitched loops remain explicit
/// planar-arrangement work.
///
/// The boundary-fragment construction follows Weiler and Atherton, "Hidden
/// Surface Removal Using Polygon Area Sorting," *SIGGRAPH Computer Graphics*
/// 11.2 (1977). Exact finite inclusion-exclusion area replay for each stitched
/// cluster keeps the shortcut in Yap's retained-state model from "Towards
/// Exact Geometric Computation," *Computational Geometry* 7.1-2 (1997).
#[cfg(feature = "exact-triangulation")]
pub fn arrange_coplanar_surface_multi_component_union(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Option<CoplanarSurfaceMultiArrangement> {
    let (projection, polygons) = coplanar_surface_component_union_polygons(left, right)?;
    if polygons.len() < 2 {
        return None;
    }
    if polygons.iter().all(|polygon| {
        validate_projected_strictly_convex_loop(
            polygon,
            projection,
            "coplanar nonconvex multi-component union",
        )
        .is_ok()
    }) {
        return None;
    }
    let mesh = polygons_to_earcut_open_mesh_with_label(
        &polygons,
        projection,
        "exact coplanar nonconvex multi-component union",
    )?;
    let arrangement = CoplanarSurfaceMultiArrangement {
        projection,
        polygons,
        mesh,
    };
    arrangement.validate().ok()?;
    Some(arrangement)
}

/// Certify and materialize component-holed coplanar surface unions.
///
/// This is a bounded annular slice of the remaining general planar
/// arrangement work. It decomposes the operands into exact source-owned disk
/// components, admits cross-operand positive-length boundary contacts and
/// bounded positive-area overlaps whose source disks replay either directly as
/// convex regions or as a small exact ear-clipped triangle set, rejects
/// point-only connectivity, and then stitches exposed source boundary
/// fragments into retained rings. Point-only cross contacts are retained only
/// as lower-dimensional split vertices: they never join the contact graph or
/// contribute area, but they may lie inside a group that is already connected
/// by positive-length or positive-area contact when the final rings replay.
/// Acceptance is component-local: each contact-connected source group,
/// including the two-disk case where two nonconvex source sheets form one
/// annulus, must replay as one outer loop with zero or more strict hole loops,
/// at least one emitted component must retain a hole, and every component
/// must satisfy exact area equality against the source loops that produced it.
/// Disconnected annular groups and in-group point contacts that still replay
/// as simple rings are therefore materialized in one retained object, while
/// point-only connectivity, branch-point decompositions requiring non-simple
/// retained boundaries, and general planar subdivisions remain outside this
/// bounded path.
///
/// The exposed-boundary traversal follows the Weiler-Atherton boundary
/// fragment model (Weiler and Atherton, "Hidden Surface Removal Using Polygon
/// Area Sorting," *SIGGRAPH Computer Graphics* 11.2, 1977), while the retained
/// ring triangulation uses Held, "FIST: Fast Industrial-Strength
/// Triangulation of Polygons," *Algorithmica* 30 (2001). Yap, "Towards Exact
/// Geometric Computation," *Computational Geometry* 7.1-2 (1997), is the
/// acceptance policy: no hole topology is inferred from samples or a triangle
/// soup; the retained rings, exact contacts, and exact area replay are the
/// certificate.
#[cfg(feature = "exact-triangulation")]
pub fn arrange_coplanar_surface_component_holed_union(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Option<CoplanarConvexComponentHoledArrangement> {
    if arrange_coplanar_convex_surface_union(left, right).is_some()
        || arrange_coplanar_convex_surface_component_union(left, right).is_some()
        || arrange_coplanar_convex_surface_multi_union(left, right).is_some()
        || arrange_coplanar_surface_component_union(left, right).is_some()
        || arrange_coplanar_surface_multi_component_union(left, right).is_some()
        || arrange_coplanar_surface_point_touch_union(left, right).is_some()
        || certify_coplanar_convex_surface_equivalence(left, right).is_some()
        || certify_coplanar_convex_surface_containment(left, right).is_some()
    {
        return None;
    }

    let left_components = connected_face_component_meshes(left)?;
    let right_components = connected_face_component_meshes(right)?;
    if left_components.is_empty() || right_components.is_empty() {
        return None;
    }

    let mut components = Vec::new();
    for mesh in left_components {
        components.push(PointTouchSourceComponent::from_mesh(
            MultiUnionSide::Left,
            mesh,
        )?);
    }
    for mesh in right_components {
        components.push(PointTouchSourceComponent::from_mesh(
            MultiUnionSide::Right,
            mesh,
        )?);
    }
    if components.len() < 2 {
        return None;
    }
    let projection = components.first()?.projection;
    if components
        .iter()
        .any(|component| component.projection != projection)
    {
        return None;
    }
    let mut has_non_axis_aligned_edge = false;
    for component in &components {
        has_non_axis_aligned_edge |=
            polygon_has_non_axis_aligned_edge(&component.polygon, projection)?;
    }
    if !has_non_axis_aligned_edge {
        return None;
    }

    let left_loops = components
        .iter()
        .filter(|component| component.side == MultiUnionSide::Left)
        .map(|component| component.polygon.clone())
        .collect::<Vec<_>>();
    let right_loops = components
        .iter()
        .filter(|component| component.side == MultiUnionSide::Right)
        .map(|component| component.polygon.clone())
        .collect::<Vec<_>>();
    validate_simple_component_loops_disjoint(
        &left_loops,
        projection,
        "coplanar component-holed union",
    )
    .ok()?;
    validate_simple_component_loops_disjoint(
        &right_loops,
        projection,
        "coplanar component-holed union",
    )
    .ok()?;

    let mut contact_graph = UnionFind::new(components.len());
    let mut saw_positive_cross_contact = false;
    let mut positive_area_contacts = Vec::new();
    let mut split_points = components
        .iter()
        .map(|component| component.polygon.clone())
        .collect::<Vec<_>>();
    for left_index in 0..components.len() {
        for right_index in left_index + 1..components.len() {
            let contact = simple_polygon_contact(
                &components[left_index].polygon,
                &components[right_index].polygon,
                projection,
            )?;
            if components[left_index].side == components[right_index].side {
                if contact != SimplePolygonContact::Disjoint {
                    return None;
                }
                continue;
            }
            match contact {
                SimplePolygonContact::Disjoint => {}
                SimplePolygonContact::PositiveLengthBoundary => {
                    saw_positive_cross_contact = true;
                    contact_graph.union(left_index, right_index);
                }
                SimplePolygonContact::PointOnly => {
                    let plan = simple_vertex_point_contact_plan(
                        &components[left_index].polygon,
                        &components[right_index].polygon,
                        projection,
                    )?;
                    if plan.relation != VertexPointContactRelation::PointOnly {
                        return None;
                    }
                    split_points[left_index].extend(plan.left_split_points);
                    split_points[right_index].extend(plan.right_split_points);
                }
                SimplePolygonContact::PositiveArea => {
                    saw_positive_cross_contact = true;
                    positive_area_contacts.push((left_index, right_index));
                    contact_graph.union(left_index, right_index);
                }
            }
        }
    }
    if !saw_positive_cross_contact {
        return None;
    }
    let groups = component_holed_union_contact_groups(&mut contact_graph, components.len());
    let mut retained_components = Vec::with_capacity(groups.len());
    for group in groups {
        let mut group_split_points = group
            .iter()
            .flat_map(|&member| split_points[member].iter().cloned())
            .collect::<Vec<_>>();
        dedup_points(&mut group_split_points);
        let component = component_holed_union_component_from_group(
            &components,
            &group,
            projection,
            component_holed_union_group_has_positive_area_overlap(&group, &positive_area_contacts),
            &group_split_points,
        )?;
        retained_components.push(component);
    }
    if !retained_components
        .iter()
        .any(|component| !component.holes.is_empty())
    {
        return None;
    }

    sort_components_for_replay(&mut retained_components, projection);
    let mesh = component_holed_components_to_earcut_open_mesh(&retained_components, projection)?;
    let arrangement = CoplanarConvexComponentHoledArrangement {
        projection,
        components: retained_components,
        mesh,
    };
    arrangement.validate().ok()?;
    Some(arrangement)
}

#[cfg(feature = "exact-triangulation")]
fn component_holed_union_contact_groups(
    contact_graph: &mut UnionFind,
    len: usize,
) -> Vec<Vec<usize>> {
    let mut groups: Vec<Vec<usize>> = Vec::new();
    for index in 0..len {
        let root = contact_graph.find(index);
        if let Some((_, group)) = groups
            .iter_mut()
            .enumerate()
            .find(|(_, group)| contact_graph.find(group[0]) == root)
        {
            group.push(index);
        } else {
            groups.push(vec![index]);
        }
    }
    groups
}

#[cfg(feature = "exact-triangulation")]
fn component_holed_union_group_has_positive_area_overlap(
    group: &[usize],
    positive_area_contacts: &[(usize, usize)],
) -> bool {
    positive_area_contacts
        .iter()
        .any(|(left, right)| group.contains(left) && group.contains(right))
}

/// Materialize one disconnected component-holed union group.
///
/// A group is either a copied source-owned disk with no holes or a connected
/// union of source disks whose exposed boundary fragments replay as retained
/// rings. The boundary-fragment traversal is the Weiler-Atherton area-sorting
/// model (Weiler and Atherton, "Hidden Surface Removal Using Polygon Area
/// Sorting," *SIGGRAPH Computer Graphics* 11.2, 1977), and the final retained
/// rings are triangulated by the same Held FIST-style ear clipping cited on
/// the public materializer. Following Yap, "Towards Exact Geometric
/// Computation," *Computational Geometry* 7.1-2 (1997), each group checks
/// exact area equality against only the source loops that generated it, so a
/// disconnected no-hole component cannot subsidize a holed component whose
/// stitched boundary silently filled unsupported cells. Positive-area source
/// overlaps use bounded convex inclusion-exclusion, the finite exact area
/// replay described by de Berg, Cheong, van Kreveld, and Overmars,
/// *Computational Geometry: Algorithms and Applications*, 3rd ed. (2008),
/// Chapter 2. Nonconvex positive-area overlaps are accepted only after exact
/// ear clipping decomposes each simple source loop into a small set of convex
/// retained triangles, preserving Yap's exact-object paradigm without widening
/// this shortcut into a general planar arrangement engine. Exact point
/// contacts are lower-dimensional facts: they are inserted as retained split
/// vertices on the output rings but do not contribute area or connectivity.
#[cfg(feature = "exact-triangulation")]
fn component_holed_union_component_from_group(
    components: &[PointTouchSourceComponent],
    group: &[usize],
    projection: CoplanarProjection,
    has_positive_area_overlap: bool,
    split_points: &[Point3],
) -> Option<CoplanarConvexHoledComponent> {
    let source_loops = group
        .iter()
        .map(|&index| components[index].polygon.clone())
        .collect::<Vec<_>>();
    if source_loops.len() == 1 {
        let mut outer = source_loops.into_iter().next()?;
        outer = split_polygon_at_boundary_points(&outer, split_points, projection)?;
        orient_polygon_ccw(&mut outer, projection)?;
        outer = simplify_projected_polygon(outer, projection);
        validate_projected_simple_loop(&outer, projection, "coplanar component-holed union")
            .ok()?;
        return Some(CoplanarConvexHoledComponent {
            outer,
            holes: Vec::new(),
        });
    }

    let mut fragments = Vec::new();
    for index in 0..source_loops.len() {
        collect_simple_union_boundary_fragments(index, &source_loops, projection, &mut fragments)?;
    }
    let loops = stitch_simple_union_loops(fragments, projection)?;
    let (mut outer, mut holes) = if loops.len() == 1 {
        (loops.into_iter().next()?, Vec::new())
    } else {
        component_holed_union_rings(loops, projection)?
    };
    orient_polygon_ccw(&mut outer, projection)?;
    outer = simplify_projected_polygon(outer, projection);
    for hole in &mut holes {
        orient_polygon_cw(hole, projection)?;
        *hole = simplify_projected_polygon(core::mem::take(hole), projection);
    }
    sort_polygons_for_replay(&mut holes, projection);
    outer = split_polygon_at_boundary_points(&outer, split_points, projection)?;
    orient_polygon_ccw(&mut outer, projection)?;
    outer = simplify_projected_polygon(outer, projection);
    for hole in &mut holes {
        *hole = split_polygon_at_boundary_points(hole, split_points, projection)?;
        orient_polygon_cw(hole, projection)?;
        *hole = simplify_projected_polygon(core::mem::take(hole), projection);
    }

    validate_projected_simple_loop(&outer, projection, "coplanar component-holed union").ok()?;
    for hole in &holes {
        validate_projected_simple_loop(hole, projection, "coplanar component-holed union").ok()?;
        if !polygon_strictly_inside_simple_polygon(hole, &outer, projection)? {
            return None;
        }
    }
    validate_component_loops_disjoint(&holes, projection, "coplanar component-holed union").ok()?;
    if !component_holed_union_area_matches_sources(
        &outer,
        &holes,
        &source_loops,
        projection,
        has_positive_area_overlap,
    )? {
        return None;
    }

    Some(CoplanarConvexHoledComponent { outer, holes })
}

#[cfg(feature = "exact-triangulation")]
fn polygon_has_non_axis_aligned_edge(
    polygon: &[Point3],
    projection: CoplanarProjection,
) -> Option<bool> {
    for index in 0..polygon.len() {
        let start = project_point(&polygon[index], projection);
        let end = project_point(&polygon[(index + 1) % polygon.len()], projection);
        if !real_equal(&start.x, &end.x) && !real_equal(&start.y, &end.y) {
            return Some(true);
        }
    }
    Some(false)
}

#[cfg(feature = "exact-triangulation")]
fn component_holed_union_rings(
    loops: Vec<Vec<Point3>>,
    projection: CoplanarProjection,
) -> Option<(Vec<Point3>, Vec<Vec<Point3>>)> {
    if loops.len() < 2 {
        return None;
    }
    let mut largest_index = 0;
    let mut largest_area = projected_area2_abs(loops.first()?, projection)?;
    for (index, loop_points) in loops.iter().enumerate().skip(1) {
        let area = projected_area2_abs(loop_points, projection)?;
        match compare_reals(&area, &largest_area).value()? {
            Ordering::Greater => {
                largest_index = index;
                largest_area = area;
            }
            Ordering::Equal => return None,
            Ordering::Less => {}
        }
    }
    if compare_reals(&largest_area, &ExactReal::from(0)).value()? != Ordering::Greater {
        return None;
    }
    let mut outer = None;
    let mut holes = Vec::new();
    for (index, loop_points) in loops.into_iter().enumerate() {
        if index == largest_index {
            outer = Some(loop_points);
        } else {
            holes.push(loop_points);
        }
    }
    Some((outer?, holes))
}

#[cfg(feature = "exact-triangulation")]
fn component_holed_union_area_matches_sources(
    outer: &[Point3],
    holes: &[Vec<Point3>],
    source_loops: &[Vec<Point3>],
    projection: CoplanarProjection,
    has_positive_area_overlap: bool,
) -> Option<bool> {
    let outer_area = projected_area2_abs(outer, projection)?;
    let mut hole_area = ExactReal::from(0);
    for hole in holes {
        hole_area = add(&hole_area, &projected_area2_abs(hole, projection)?);
    }
    if compare_reals(&outer_area, &hole_area).value()? != Ordering::Greater {
        return Some(false);
    }
    let retained_area = sub(&outer_area, &hole_area);
    let source_area =
        component_holed_union_source_area2(source_loops, projection, has_positive_area_overlap)?;
    Some(compare_reals(&retained_area, &source_area).value() == Some(Ordering::Equal))
}

/// Return the exact doubled area of the source-loop union for one holed group.
///
/// Positive-length-only groups use the sum of source disk areas: the disjoint
/// interiors make additivity the exact certificate, and keeping that path
/// separate prevents a boundary-contact nonconvex annulus from being rejected
/// only because its ear decomposition would exceed the bounded
/// inclusion-exclusion budget. Positive-area groups need true union area. They
/// first take the convex-region path and then fall back to exact
/// triangulation-plus-inclusion-exclusion for simple nonconvex loops.
///
/// This mirrors Yap, "Towards Exact Geometric Computation," *Computational
/// Geometry* 7.1-2 (1997): the certificate is a retained finite exact
/// computation over source-owned regions. The finite union formula is the
/// standard inclusion-exclusion proof over convex cells described by de Berg,
/// Cheong, van Kreveld, and Overmars, *Computational Geometry: Algorithms and
/// Applications*, 3rd ed. (2008), Chapter 2.
#[cfg(feature = "exact-triangulation")]
fn component_holed_union_source_area2(
    source_loops: &[Vec<Point3>],
    projection: CoplanarProjection,
    has_positive_area_overlap: bool,
) -> Option<ExactReal> {
    if !has_positive_area_overlap {
        let mut source_area = ExactReal::from(0);
        for source_loop in source_loops {
            source_area = add(&source_area, &projected_area2_abs(source_loop, projection)?);
        }
        return Some(source_area);
    }

    if source_loops.iter().all(|source_loop| {
        validate_projected_strictly_convex_loop(
            source_loop,
            projection,
            "coplanar component-holed union source area",
        )
        .is_ok()
    }) {
        convex_region_union_area_inclusion_exclusion(source_loops, projection)
    } else {
        component_holed_union_triangulated_source_area2(source_loops, projection)
    }
}

/// Replay a nonconvex positive-area source union through exact source triangles.
///
/// Each simple source loop is oriented counter-clockwise and decomposed by the
/// local exact ear clipper. Ear clipping is the finite triangulation theorem
/// of Meisters, "Polygons Have Ears," *American Mathematical Monthly* 82.6
/// (1975), implemented here with exact projected orientation and
/// point-in-triangle predicates. The resulting convex source triangles are
/// then fed to the same bounded inclusion-exclusion replay used by convex
/// sectors, so the materializer can certify small nonconvex overlaps while
/// still rejecting larger arrangements that need a real planar cell engine.
#[cfg(feature = "exact-triangulation")]
fn component_holed_union_triangulated_source_area2(
    source_loops: &[Vec<Point3>],
    projection: CoplanarProjection,
) -> Option<ExactReal> {
    let mut source_regions = Vec::new();
    for source_loop in source_loops {
        let mut polygon = source_loop.clone();
        orient_polygon_ccw(&mut polygon, projection)?;
        polygon = simplify_projected_polygon(polygon, projection);
        validate_projected_simple_loop(
            &polygon,
            projection,
            "coplanar component-holed union nonconvex source area",
        )
        .ok()?;

        if validate_projected_strictly_convex_loop(
            &polygon,
            projection,
            "coplanar component-holed union nonconvex source area",
        )
        .is_ok()
        {
            source_regions.push(polygon);
            continue;
        }

        for triangle in retained_simple_polygon_ear_clip_triangles(&polygon, projection)? {
            let mut region = triangle_points(&polygon, triangle.0);
            orient_polygon_ccw(&mut region, projection)?;
            region = simplify_projected_polygon(region, projection);
            validate_projected_strictly_convex_loop(
                &region,
                projection,
                "coplanar component-holed union source triangle",
            )
            .ok()?;
            source_regions.push(region);
        }
    }

    convex_region_union_area_inclusion_exclusion(&source_regions, projection)
}

/// Certify that two coplanar surface meshes meet on positive-length boundary
/// arcs and have no positive-area overlap.
///
/// This is the lower-dimensional counterpart to the nonconvex component-union
/// paths. It decomposes each source into exact connected surface components,
/// imports each component as either a convex hull or a certified simple
/// boundary, and then classifies every cross-source boundary relation with
/// exact segment and point-in-simple-polygon predicates. The result is only a
/// certificate: intersection is empty as a surface, and difference preserves
/// the left surface, but no additional union topology is inferred here.
///
/// The promotion rule follows Yap, "Towards Exact Geometric Computation,"
/// *Computational Geometry* 7.1-2 (1997): lower-dimensional topology is exposed
/// only after the exact combinatorics prove the absence of positive-area cells.
/// Edge contacts use the orientation-predicate segment relation of Guigue and
/// Devillers, "Fast and Robust Triangle-Triangle Overlap Test Using
/// Orientation Predicates," *Journal of Graphics Tools* 8.1 (2003), while
/// strict containment uses the same exact earcut-based simple-polygon location
/// replay used elsewhere in this module.
#[cfg(feature = "exact-triangulation")]
pub fn certify_coplanar_surface_boundary_touch(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Option<CoplanarProjection> {
    let left_component_meshes = connected_face_component_meshes(left)?;
    let right_component_meshes = connected_face_component_meshes(right)?;
    if left_component_meshes.is_empty() || right_component_meshes.is_empty() {
        return None;
    }

    let mut components = Vec::new();
    for mesh in left_component_meshes {
        components.push(PointTouchSourceComponent::from_mesh(
            MultiUnionSide::Left,
            mesh,
        )?);
    }
    for mesh in right_component_meshes {
        components.push(PointTouchSourceComponent::from_mesh(
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

    let left_loops = components
        .iter()
        .filter(|component| component.side == MultiUnionSide::Left)
        .map(|component| component.polygon.clone())
        .collect::<Vec<_>>();
    let right_loops = components
        .iter()
        .filter(|component| component.side == MultiUnionSide::Right)
        .map(|component| component.polygon.clone())
        .collect::<Vec<_>>();
    validate_simple_component_loops_disjoint(
        &left_loops,
        projection,
        "coplanar boundary-touch left components",
    )
    .ok()?;
    validate_simple_component_loops_disjoint(
        &right_loops,
        projection,
        "coplanar boundary-touch right components",
    )
    .ok()?;

    let mut saw_positive_length_boundary_touch = false;
    for left_index in 0..components.len() {
        for right_index in left_index + 1..components.len() {
            let contact = simple_polygon_contact(
                &components[left_index].polygon,
                &components[right_index].polygon,
                projection,
            )?;
            if components[left_index].side == components[right_index].side {
                if contact != SimplePolygonContact::Disjoint {
                    return None;
                }
                continue;
            }
            match contact {
                SimplePolygonContact::Disjoint | SimplePolygonContact::PointOnly => {}
                SimplePolygonContact::PositiveLengthBoundary => {
                    saw_positive_length_boundary_touch = true;
                }
                SimplePolygonContact::PositiveArea => return None,
            }
        }
    }

    saw_positive_length_boundary_touch.then_some(projection)
}

/// Certify and materialize a bounded point-touch union.
///
/// This path handles a hard branch-point case without weakening the existing
/// simple-loop and multi-loop arrangements. Source topology is decomposed into
/// exact convex coplanar components, same-operand components must remain
/// disjoint, and retained output loops may meet only through exact points.
/// Pure point contacts keep each source component as a separate loop.
/// Mixed contacts first absorb positive-area or positive-length connected
/// groups into retained simple loops, then split the surviving output loops at
/// exact vertex-edge point contacts. The output intentionally duplicates the
/// shared exact coordinate in the mesh, so downstream consumers see branch
/// incidence through the named artifact rather than through an accidental
/// welded vertex.
///
/// The construction follows Yap, "Towards Exact Geometric Computation,"
/// *Computational Geometry* 7.1-2 (1997), by promoting topology only from
/// replayable exact source facts. Segment contacts are classified with the
/// orientation-predicate model used by Guigue and Devillers, "Fast and Robust
/// Triangle-Triangle Overlap Test Using Orientation Predicates," *Journal of
/// Graphics Tools* 8.1 (2003).
#[cfg(feature = "exact-triangulation")]
pub fn arrange_coplanar_surface_point_touch_union(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Option<CoplanarSurfacePointTouchUnion> {
    if arrange_coplanar_convex_surface_union(left, right).is_some()
        || arrange_coplanar_convex_surface_component_union(left, right).is_some()
        || arrange_coplanar_surface_component_union(left, right).is_some()
        || arrange_coplanar_surface_multi_component_union(left, right).is_some()
        || arrange_coplanar_convex_surface_multi_union(left, right).is_some()
        || certify_coplanar_convex_surface_equivalence(left, right).is_some()
        || certify_coplanar_convex_surface_containment(left, right).is_some()
    {
        return None;
    }

    let left_components = connected_face_component_meshes(left)?;
    let right_components = connected_face_component_meshes(right)?;
    if left_components.is_empty() || right_components.is_empty() {
        return None;
    }
    if let Some(union) = arrange_coplanar_mixed_boundary_point_union_from_components(
        left_components.clone(),
        right_components.clone(),
    ) {
        return Some(union);
    }

    let mut components = Vec::new();
    for mesh in left_components.iter().cloned() {
        let Some(component) = ConvexUnionComponent::from_mesh(MultiUnionSide::Left, mesh) else {
            return arrange_coplanar_simple_surface_point_touch_union_from_components(
                left_components,
                right_components,
            );
        };
        components.push(component);
    }
    for mesh in right_components.iter().cloned() {
        let Some(component) = ConvexUnionComponent::from_mesh(MultiUnionSide::Right, mesh) else {
            return arrange_coplanar_simple_surface_point_touch_union_from_components(
                left_components,
                right_components,
            );
        };
        components.push(component);
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
        "coplanar point-touch surface union",
    )
    .ok()?;
    validate_component_loops_disjoint(
        &right_hulls,
        projection,
        "coplanar point-touch surface union",
    )
    .ok()?;

    let mut saw_point_touch = false;
    let mut split_points = components
        .iter()
        .map(|component| component.hull.clone())
        .collect::<Vec<_>>();
    for left_index in 0..components.len() {
        for right_index in left_index + 1..components.len() {
            match convex_union_component_relation(
                &components[left_index].hull,
                &components[right_index].hull,
                projection,
            )? {
                ConvexUnionComponentRelation::Disjoint => {
                    if components[left_index].side == components[right_index].side {
                        continue;
                    }
                    let contact = vertex_point_contact_plan(
                        &components[left_index].hull,
                        &components[right_index].hull,
                        projection,
                    )?;
                    match contact.relation {
                        VertexPointContactRelation::Disjoint => {}
                        VertexPointContactRelation::PointOnly => {
                            saw_point_touch = true;
                            split_points[left_index].extend(contact.left_split_points);
                            split_points[right_index].extend(contact.right_split_points);
                        }
                        VertexPointContactRelation::InvalidBoundaryContact => return None,
                    }
                }
                ConvexUnionComponentRelation::PositiveArea => return None,
                ConvexUnionComponentRelation::BoundaryOnly => {
                    if components[left_index].side == components[right_index].side {
                        return None;
                    }
                    if convex_polygons_touch_on_positive_boundary(
                        &components[left_index].hull,
                        &components[right_index].hull,
                        projection,
                    )? {
                        return None;
                    }
                    let contact = vertex_point_contact_plan(
                        &components[left_index].hull,
                        &components[right_index].hull,
                        projection,
                    )?;
                    match contact.relation {
                        VertexPointContactRelation::PointOnly => {
                            saw_point_touch = true;
                            split_points[left_index].extend(contact.left_split_points);
                            split_points[right_index].extend(contact.right_split_points);
                        }
                        VertexPointContactRelation::Disjoint
                        | VertexPointContactRelation::InvalidBoundaryContact => return None,
                    }
                }
            }
        }
    }
    if !saw_point_touch {
        return None;
    }

    let mut polygons = components
        .iter()
        .zip(split_points.iter_mut())
        .map(|(component, points)| {
            dedup_points(points);
            split_polygon_at_boundary_points(&component.hull, points, projection)
        })
        .collect::<Option<Vec<_>>>()?;
    for polygon in &mut polygons {
        orient_polygon_ccw(polygon, projection)?;
    }
    sort_polygons_for_replay(&mut polygons, projection);
    let mesh = weak_convex_polygons_to_open_mesh_with_label(
        &polygons,
        projection,
        "exact coplanar point-touch surface union",
    )?;
    let union = CoplanarSurfacePointTouchUnion {
        projection,
        polygons,
        mesh,
    };
    union.validate().ok()?;
    Some(union)
}

/// Materialize mixed area/edge-connected and point-touching coplanar unions.
///
/// Pure point-touch unions keep every source component as a retained output
/// loop, while pure connected contacts belong to the component-union
/// materializers. The remaining bounded case has both: some cross-source
/// components overlap in positive area or share positive-length boundary arcs
/// and must be absorbed into one simple retained loop, while other
/// cross-source contacts are exact point branches between those retained
/// loops. This helper builds the connected contact groups first, validates
/// that the final output loops meet only at exact points, and triangulates
/// each loop independently.
///
/// The construction is still a retained-object shortcut in Yap's sense from
/// "Towards Exact Geometric Computation," *Computational Geometry* 7.1-2
/// (1997): no sampled arrangement cells are inferred. Connected overlap and
/// boundary-contact groups use the same Weiler-Atherton exposed-fragment
/// stitcher used by source component unions, and point branches are split
/// using exact segment
/// predicates in the style of Guigue and Devillers, "Fast and Robust
/// Triangle-Triangle Overlap Test Using Orientation Predicates," *Journal of
/// Graphics Tools* 8.1 (2003).
#[cfg(feature = "exact-triangulation")]
fn arrange_coplanar_mixed_boundary_point_union_from_components(
    left_components: Vec<ExactMesh>,
    right_components: Vec<ExactMesh>,
) -> Option<CoplanarSurfacePointTouchUnion> {
    if left_components.is_empty() || right_components.is_empty() {
        return None;
    }

    let mut components = Vec::new();
    for mesh in left_components {
        components.push(PointTouchSourceComponent::from_mesh(
            MultiUnionSide::Left,
            mesh,
        )?);
    }
    for mesh in right_components {
        components.push(PointTouchSourceComponent::from_mesh(
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

    let left_loops = components
        .iter()
        .filter(|component| component.side == MultiUnionSide::Left)
        .map(|component| component.polygon.clone())
        .collect::<Vec<_>>();
    let right_loops = components
        .iter()
        .filter(|component| component.side == MultiUnionSide::Right)
        .map(|component| component.polygon.clone())
        .collect::<Vec<_>>();
    validate_simple_component_loops_disjoint(
        &left_loops,
        projection,
        "coplanar mixed boundary-point surface union",
    )
    .ok()?;
    validate_simple_component_loops_disjoint(
        &right_loops,
        projection,
        "coplanar mixed boundary-point surface union",
    )
    .ok()?;

    let mut contact_graph = UnionFind::new(components.len());
    let mut saw_connected_contact = false;
    let mut saw_point_touch = false;
    let mut split_points = components
        .iter()
        .map(|component| component.polygon.clone())
        .collect::<Vec<_>>();
    for left_index in 0..components.len() {
        for right_index in left_index + 1..components.len() {
            let contact = simple_polygon_contact(
                &components[left_index].polygon,
                &components[right_index].polygon,
                projection,
            )?;
            if components[left_index].side == components[right_index].side {
                if contact != SimplePolygonContact::Disjoint {
                    return None;
                }
                continue;
            }
            match contact {
                SimplePolygonContact::Disjoint => {}
                SimplePolygonContact::PointOnly => {
                    let plan = simple_vertex_point_contact_plan(
                        &components[left_index].polygon,
                        &components[right_index].polygon,
                        projection,
                    )?;
                    if plan.relation != VertexPointContactRelation::PointOnly {
                        return None;
                    }
                    saw_point_touch = true;
                    split_points[left_index].extend(plan.left_split_points);
                    split_points[right_index].extend(plan.right_split_points);
                }
                SimplePolygonContact::PositiveLengthBoundary => {
                    saw_connected_contact = true;
                    contact_graph.union(left_index, right_index);
                }
                SimplePolygonContact::PositiveArea => {
                    saw_connected_contact = true;
                    contact_graph.union(left_index, right_index);
                }
            }
        }
    }
    if !saw_connected_contact || !saw_point_touch {
        return None;
    }

    let source_loops = components
        .iter()
        .map(|component| component.polygon.clone())
        .collect::<Vec<_>>();
    let mut groups: Vec<(usize, Vec<usize>)> = Vec::new();
    for index in 0..components.len() {
        let root = contact_graph.find(index);
        if let Some((_, members)) = groups.iter_mut().find(|(candidate, _)| *candidate == root) {
            members.push(index);
        } else {
            groups.push((root, vec![index]));
        }
    }
    groups.sort_by_key(|(_, members)| members.first().copied().unwrap_or(usize::MAX));

    let mut polygons = Vec::with_capacity(groups.len());
    for (_, members) in groups {
        let mut polygon = if members.len() == 1 {
            source_loops[*members.first()?].clone()
        } else {
            materialize_simple_polygon_union_group(
                &source_loops,
                &members,
                projection,
                "coplanar mixed boundary-point surface union",
            )?
        };
        let mut group_split_points = members
            .iter()
            .flat_map(|&member| split_points[member].iter().cloned())
            .collect::<Vec<_>>();
        dedup_points(&mut group_split_points);
        polygon = split_polygon_at_boundary_points(&polygon, &group_split_points, projection)?;
        orient_polygon_ccw(&mut polygon, projection)?;
        polygon = simplify_projected_polygon(polygon, projection);
        validate_projected_simple_loop(
            &polygon,
            projection,
            "coplanar mixed boundary-point surface union",
        )
        .ok()?;
        polygons.push(polygon);
    }
    if !final_loops_have_only_point_touches(&polygons, projection)? {
        return None;
    }
    sort_polygons_for_replay(&mut polygons, projection);
    let mesh = polygons_to_retained_simple_open_mesh_with_label(
        &polygons,
        projection,
        "exact coplanar mixed boundary-point surface union",
    )?;
    let union = CoplanarSurfacePointTouchUnion {
        projection,
        polygons,
        mesh,
    };
    union.validate().ok()?;
    Some(union)
}

/// Materialize nonconvex-capable coplanar unions that meet only at points.
///
/// This is the simple-loop sibling of
/// [`arrange_coplanar_surface_point_touch_union`]'s convex-hull path. It is
/// intentionally narrow: every connected input component must certify as one
/// source-owned disk; same-side loops must be disjoint; cross-side loops may
/// only have exact vertex-vertex or vertex-edge point contacts. Vertex-edge
/// contacts are promoted to shared retained vertices before triangulation so
/// branch incidence is explicit. The segment rejection predicates follow
/// Guigue and Devillers, "Fast and Robust Triangle-Triangle Overlap Test Using
/// Orientation Predicates," *Journal of Graphics Tools* 8.1 (2003), while
/// simple-loop location and triangulation use Held, "FIST: Fast
/// Industrial-Strength Triangulation of Polygons," *Algorithmica* 30 (2001).
/// The retained branch topology follows Yap, "Towards Exact Geometric
/// Computation," *Computational Geometry* 7.1-2 (1997).
#[cfg(feature = "exact-triangulation")]
fn arrange_coplanar_simple_surface_point_touch_union_from_components(
    left_components: Vec<ExactMesh>,
    right_components: Vec<ExactMesh>,
) -> Option<CoplanarSurfacePointTouchUnion> {
    if left_components.is_empty() || right_components.is_empty() {
        return None;
    }
    let mut components = Vec::new();
    for mesh in left_components {
        components.push(PointTouchSourceComponent::from_mesh(
            MultiUnionSide::Left,
            mesh,
        )?);
    }
    for mesh in right_components {
        components.push(PointTouchSourceComponent::from_mesh(
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

    let left_loops = components
        .iter()
        .filter(|component| component.side == MultiUnionSide::Left)
        .map(|component| component.polygon.clone())
        .collect::<Vec<_>>();
    let right_loops = components
        .iter()
        .filter(|component| component.side == MultiUnionSide::Right)
        .map(|component| component.polygon.clone())
        .collect::<Vec<_>>();
    validate_simple_component_loops_disjoint(
        &left_loops,
        projection,
        "coplanar nonconvex point-touch surface union",
    )
    .ok()?;
    validate_simple_component_loops_disjoint(
        &right_loops,
        projection,
        "coplanar nonconvex point-touch surface union",
    )
    .ok()?;

    let mut saw_point_touch = false;
    let mut split_points = components
        .iter()
        .map(|component| component.polygon.clone())
        .collect::<Vec<_>>();
    for left_index in 0..components.len() {
        for right_index in left_index + 1..components.len() {
            if components[left_index].side == components[right_index].side {
                continue;
            }
            let contact = simple_vertex_point_contact_plan(
                &components[left_index].polygon,
                &components[right_index].polygon,
                projection,
            )?;
            match contact.relation {
                VertexPointContactRelation::Disjoint => {}
                VertexPointContactRelation::PointOnly => {
                    saw_point_touch = true;
                    split_points[left_index].extend(contact.left_split_points);
                    split_points[right_index].extend(contact.right_split_points);
                }
                VertexPointContactRelation::InvalidBoundaryContact => return None,
            }
        }
    }
    if !saw_point_touch {
        return None;
    }

    let mut polygons = components
        .iter()
        .zip(split_points.iter_mut())
        .map(|(component, points)| {
            dedup_points(points);
            split_polygon_at_boundary_points(&component.polygon, points, projection)
        })
        .collect::<Option<Vec<_>>>()?;
    for polygon in &mut polygons {
        orient_polygon_ccw(polygon, projection)?;
    }
    sort_polygons_for_replay(&mut polygons, projection);
    let mesh = polygons_to_retained_simple_open_mesh_with_label(
        &polygons,
        projection,
        "exact coplanar nonconvex point-touch surface union",
    )?;
    let union = CoplanarSurfacePointTouchUnion {
        projection,
        polygons,
        mesh,
    };
    union.validate().ok()?;
    Some(union)
}

#[cfg(feature = "exact-triangulation")]
fn coplanar_surface_component_union_loop(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Option<(CoplanarProjection, Vec<Point3>)> {
    if arrange_coplanar_convex_surface_union(left, right).is_some()
        || arrange_coplanar_convex_surface_component_union(left, right).is_some()
        || arrange_coplanar_convex_surface_multi_union(left, right).is_some()
        || certify_coplanar_convex_surface_equivalence(left, right).is_some()
        || certify_coplanar_convex_surface_containment(left, right).is_some()
    {
        return None;
    }

    let left_components = connected_face_component_meshes(left)?;
    let right_components = connected_face_component_meshes(right)?;
    if left_components.len() + right_components.len() < 2 {
        return None;
    }

    let mut components = Vec::new();
    for mesh in left_components.iter().cloned() {
        let Some(component) = ConvexUnionComponent::from_mesh(MultiUnionSide::Left, mesh) else {
            let (projection, mut polygons) = coplanar_simple_surface_component_union_polygons(
                left_components,
                right_components,
                "coplanar nonconvex simple-source component union",
            )?;
            return if polygons.len() == 1 {
                Some((projection, polygons.pop()?))
            } else {
                None
            };
        };
        components.push(component);
    }
    for mesh in right_components.iter().cloned() {
        let Some(component) = ConvexUnionComponent::from_mesh(MultiUnionSide::Right, mesh) else {
            let (projection, mut polygons) = coplanar_simple_surface_component_union_polygons(
                left_components,
                right_components,
                "coplanar nonconvex simple-source component union",
            )?;
            return if polygons.len() == 1 {
                Some((projection, polygons.pop()?))
            } else {
                None
            };
        };
        components.push(component);
    }
    if components.len() < 3 {
        let (projection, mut polygons) = coplanar_simple_surface_component_union_polygons(
            left_components,
            right_components,
            "coplanar nonconvex simple-source component union",
        )?;
        return if polygons.len() == 1 {
            Some((projection, polygons.pop()?))
        } else {
            None
        };
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
        "coplanar nonconvex component union",
    )
    .ok()?;
    validate_component_loops_disjoint(
        &right_hulls,
        projection,
        "coplanar nonconvex component union",
    )
    .ok()?;

    let mut contact_graph = UnionFind::new(components.len());
    for left_index in 0..components.len() {
        for right_index in left_index + 1..components.len() {
            match convex_union_component_relation(
                &components[left_index].hull,
                &components[right_index].hull,
                projection,
            )? {
                ConvexUnionComponentRelation::Disjoint => {}
                ConvexUnionComponentRelation::BoundaryOnly => {
                    if components[left_index].side == components[right_index].side
                        || !convex_polygons_touch_on_positive_boundary(
                            &components[left_index].hull,
                            &components[right_index].hull,
                            projection,
                        )?
                    {
                        return None;
                    }
                    contact_graph.union(left_index, right_index);
                }
                ConvexUnionComponentRelation::PositiveArea => {
                    if components[left_index].side == components[right_index].side {
                        return None;
                    }
                    contact_graph.union(left_index, right_index);
                }
            }
        }
    }

    let root = contact_graph.find(0);
    for index in 1..components.len() {
        if contact_graph.find(index) != root {
            return None;
        }
    }

    let regions = components
        .iter()
        .map(|component| component.hull.clone())
        .collect::<Vec<_>>();
    let mut polygon = connected_convex_contact_union_polygon(&regions, projection)?;
    orient_polygon_ccw(&mut polygon, projection)?;
    polygon = simplify_projected_polygon(polygon, projection);
    validate_projected_simple_loop(&polygon, projection, "coplanar nonconvex component union")
        .ok()?;
    Some((projection, polygon))
}

#[cfg(feature = "exact-triangulation")]
fn coplanar_surface_component_union_polygons(
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
    for mesh in left_components.iter().cloned() {
        let Some(component) = ConvexUnionComponent::from_mesh(MultiUnionSide::Left, mesh) else {
            return coplanar_simple_surface_component_union_polygons(
                left_components,
                right_components,
                "coplanar nonconvex simple-source multi-component union",
            );
        };
        components.push(component);
    }
    for mesh in right_components.iter().cloned() {
        let Some(component) = ConvexUnionComponent::from_mesh(MultiUnionSide::Right, mesh) else {
            return coplanar_simple_surface_component_union_polygons(
                left_components,
                right_components,
                "coplanar nonconvex simple-source multi-component union",
            );
        };
        components.push(component);
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
        "coplanar nonconvex multi-component union",
    )
    .ok()?;
    validate_component_loops_disjoint(
        &right_hulls,
        projection,
        "coplanar nonconvex multi-component union",
    )
    .ok()?;

    let mut contact_graph = UnionFind::new(components.len());
    for left_index in 0..components.len() {
        for right_index in left_index + 1..components.len() {
            match convex_union_component_relation(
                &components[left_index].hull,
                &components[right_index].hull,
                projection,
            )? {
                ConvexUnionComponentRelation::Disjoint => {}
                ConvexUnionComponentRelation::BoundaryOnly => {
                    if components[left_index].side == components[right_index].side
                        || !convex_polygons_touch_on_positive_boundary(
                            &components[left_index].hull,
                            &components[right_index].hull,
                            projection,
                        )?
                    {
                        return None;
                    }
                    contact_graph.union(left_index, right_index);
                }
                ConvexUnionComponentRelation::PositiveArea => {
                    if components[left_index].side == components[right_index].side {
                        return None;
                    }
                    contact_graph.union(left_index, right_index);
                }
            }
        }
    }

    let mut groups: Vec<(usize, Vec<usize>)> = Vec::new();
    for index in 0..components.len() {
        let root = contact_graph.find(index);
        if let Some((_, members)) = groups.iter_mut().find(|(candidate, _)| *candidate == root) {
            members.push(index);
        } else {
            groups.push((root, vec![index]));
        }
    }

    let mut polygons = Vec::with_capacity(groups.len());
    for (_, members) in groups {
        let mut polygon =
            materialize_component_union_group(&components, &members).or_else(|| {
                let regions = members
                    .iter()
                    .map(|&member| components[member].hull.clone())
                    .collect::<Vec<_>>();
                connected_convex_contact_union_polygon(&regions, projection)
            })?;
        orient_polygon_ccw(&mut polygon, projection)?;
        polygon = simplify_projected_polygon(polygon, projection);
        validate_projected_simple_loop(
            &polygon,
            projection,
            "coplanar nonconvex multi-component union",
        )
        .ok()?;
        polygons.push(polygon);
    }

    sort_polygons_for_replay(&mut polygons, projection);
    validate_simple_component_loops_disjoint(
        &polygons,
        projection,
        "coplanar nonconvex multi-component union",
    )
    .ok()?;
    Some((projection, polygons))
}

/// Materialize unions of simple source disks by exposed-boundary replay.
///
/// This is the nonconvex-source sibling of the convex component-union
/// materializer. Each connected source component is replayed either as a
/// convex component hull or as one exact simple source boundary. Cross-source
/// components may then be clustered only when exact segment/containment
/// predicates prove positive-area overlap or positive-length boundary contact;
/// point-only contacts stay on [`arrange_coplanar_surface_point_touch_union`].
/// A cluster is accepted only when its exposed boundary stitches into one
/// simple loop, every source loop lies in that retained loop, and exact area
/// proves the loop has not filled an unsupported hole.
///
/// The exposed-boundary traversal follows Weiler and Atherton, "Hidden
/// Surface Removal Using Polygon Area Sorting," *SIGGRAPH Computer Graphics*
/// 11.2 (1977). The promotion rule is Yap's retained-object discipline from
/// "Towards Exact Geometric Computation," *Computational Geometry* 7.1-2
/// (1997): no output topology is accepted unless the exact source loops and
/// exact replayed boundary determine it.
#[cfg(feature = "exact-triangulation")]
fn coplanar_simple_surface_component_union_polygons(
    left_component_meshes: Vec<ExactMesh>,
    right_component_meshes: Vec<ExactMesh>,
    label: &'static str,
) -> Option<(CoplanarProjection, Vec<Vec<Point3>>)> {
    if left_component_meshes.is_empty() || right_component_meshes.is_empty() {
        return None;
    }
    let mut components = Vec::new();
    for mesh in left_component_meshes {
        components.push(PointTouchSourceComponent::from_mesh(
            MultiUnionSide::Left,
            mesh,
        )?);
    }
    for mesh in right_component_meshes {
        components.push(PointTouchSourceComponent::from_mesh(
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

    let left_loops = components
        .iter()
        .filter(|component| component.side == MultiUnionSide::Left)
        .map(|component| component.polygon.clone())
        .collect::<Vec<_>>();
    let right_loops = components
        .iter()
        .filter(|component| component.side == MultiUnionSide::Right)
        .map(|component| component.polygon.clone())
        .collect::<Vec<_>>();
    validate_simple_component_loops_disjoint(&left_loops, projection, label).ok()?;
    validate_simple_component_loops_disjoint(&right_loops, projection, label).ok()?;

    let mut contact_graph = UnionFind::new(components.len());
    let mut saw_connected_cross_source = false;
    for left_index in 0..components.len() {
        for right_index in left_index + 1..components.len() {
            let interaction = simple_polygon_interaction(
                &components[left_index].polygon,
                &components[right_index].polygon,
                projection,
            )?;
            if components[left_index].side == components[right_index].side {
                if interaction != SimplePolygonInteraction::Disjoint {
                    return None;
                }
                continue;
            }
            match interaction {
                SimplePolygonInteraction::Disjoint => {}
                SimplePolygonInteraction::PointOnly => return None,
                SimplePolygonInteraction::Connected => {
                    saw_connected_cross_source = true;
                    contact_graph.union(left_index, right_index);
                }
            }
        }
    }
    if !saw_connected_cross_source {
        return None;
    }

    let mut groups: Vec<(usize, Vec<usize>)> = Vec::new();
    for index in 0..components.len() {
        let root = contact_graph.find(index);
        if let Some((_, members)) = groups.iter_mut().find(|(candidate, _)| *candidate == root) {
            members.push(index);
        } else {
            groups.push((root, vec![index]));
        }
    }
    groups.sort_by_key(|(_, members)| members.first().copied().unwrap_or(usize::MAX));

    let source_loops = components
        .iter()
        .map(|component| component.polygon.clone())
        .collect::<Vec<_>>();
    let mut polygons = Vec::with_capacity(groups.len());
    for (_, members) in groups {
        let mut polygon = if members.len() == 1 {
            source_loops[*members.first()?].clone()
        } else {
            materialize_simple_polygon_union_group(&source_loops, &members, projection, label)?
        };
        orient_polygon_ccw(&mut polygon, projection)?;
        polygon = simplify_projected_polygon(polygon, projection);
        validate_projected_simple_loop(&polygon, projection, label).ok()?;
        polygons.push(polygon);
    }
    sort_polygons_for_replay(&mut polygons, projection);
    validate_simple_component_loops_disjoint(&polygons, projection, label).ok()?;
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
/// disjoint cutters span the component on one projected axis. Other rectangular
/// multi-cutter cases can replay through exact orthogonal cells when no output
/// component has a retained hole; the convex artifact still accepts only
/// strictly convex loops, so nonconvex no-hole output is left to
/// [`CoplanarSurfaceMultiArrangement`]. Remaining multi-cutter cases are
/// accepted only when each sequential cutter still replays through the existing
/// exact convex difference certificates and emits convex remnants.
/// Point-only boundary contacts, strict interior holes, and overlapping
/// nonconvex loops still return `None` so the general arrangement layer remains
/// explicit. This follows Yap, "Towards Exact Geometric Computation,"
/// *Computational Geometry* 7.1-2 (1997): the shortcut promotes output loops
/// only from retained exact component, containment, intersection, and area
/// evidence, never from sampled polygon surgery. The orthogonal no-hole path is
/// the bounded cell arrangement of de Berg, Cheong, van Kreveld, and Overmars,
/// *Computational Geometry: Algorithms and Applications*, 3rd ed. (2008),
/// Chapter 2, consumed only after exact retained grid occupancy is known.
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
                if !convex_polygons_touch_on_positive_boundary(
                    &component.hull,
                    &right_component.hull,
                    projection,
                )? {
                    return None;
                }
                if drop_component {
                    return None;
                }
                cutter_indices.push(right_index);
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

/// Certify a multi-component coplanar difference.
///
/// This is the bounded output-model step beyond the convex component
/// difference certificate. Source topology is first decomposed into disjoint
/// convex components, and every cutter/remnant step must replay through the
/// existing exact convex difference certificates. If a source component is a
/// triangulated nonconvex simple disk instead, the bounded side-cutter path
/// below may replay its retained boundary ring directly: each removed convex
/// component must lie in the closed source ring, own positive-length contact
/// with that boundary, and pass exact fragment stitching plus area replay.
/// Rectangular multi-cutters may also enter through exact no-hole orthogonal
/// cell replay, including boundary-attached partial-height cutters that would
/// otherwise require nonconvex loop surgery. Overlapping same-source side
/// cutters are accepted only by the retained side-cutter channel replay below;
/// they are not prefiltered away because their exact union can be the
/// materialized removed object. A strict interior right component may also be
/// consumed by a cutter/hole-contact opening on the same left component, but
/// only when that local replay emits no retained holes. A consumed strict
/// hole can force this artifact even when the retained split loops themselves
/// are convex: the convex multi-difference certificate cannot name the
/// removed hole owner, so this retained multi-surface object carries that
/// topology. Same-outer source-holed differences may also emit filled
/// right-hole components when every emitted ring is disjoint from the left
/// retained holes; holed remnants stay on the component-holed artifact.
/// Hole-producing cuts, outside clipping against a nonconvex source boundary,
/// point-only boundary contacts, overlapping output loops, and
/// self-intersections remain explicit planar-arrangement work. This follows
/// Yap, "Towards Exact Geometric
/// Computation," *Computational Geometry* 7.1-2 (1997): the system may
/// broaden the object model only when the exact construction history and
/// output topology are both retained. The boundary-fragment replay is the
/// Weiler-Atherton retained-edge idea, not a sampled polygon clip; see Weiler
/// and Atherton, "Hidden Surface Removal Using Polygon Area Sorting,"
/// *SIGGRAPH Computer Graphics* 11.2 (1977).
#[cfg(feature = "exact-triangulation")]
pub fn arrange_coplanar_surface_multi_difference(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Option<CoplanarSurfaceMultiArrangement> {
    let (projection, polygons) = coplanar_surface_difference_polygons(left, right)
        .or_else(|| same_outer_holed_no_hole_difference_polygons(left, right))?;
    if polygons.len() < 2 {
        return None;
    }
    if polygons.iter().all(|polygon| {
        validate_projected_strictly_convex_loop(
            polygon,
            projection,
            "coplanar nonconvex multi-component difference",
        )
        .is_ok()
    }) && arrange_coplanar_convex_surface_multi_difference(left, right).is_some()
    {
        return None;
    }
    let mesh = polygons_to_earcut_open_mesh_with_label(
        &polygons,
        projection,
        "exact coplanar multi-component difference arrangement",
    )?;
    let arrangement = CoplanarSurfaceMultiArrangement {
        projection,
        polygons,
        mesh,
    };
    arrangement.validate().ok()?;
    Some(arrangement)
}

/// Certify a single-loop coplanar component difference.
///
/// This is the single-output counterpart to
/// [`arrange_coplanar_surface_multi_difference`]. It covers the bounded case
/// where source topology has several left components, exact component replay
/// deletes some components completely, and the only retained component is a
/// nonconvex simple loop produced by the existing cutter/remnant certificates.
/// The shortcut deliberately refuses cases already handled by
/// [`arrange_single_triangle_coplanar_difference`],
/// [`arrange_coplanar_convex_surface_difference`], and
/// [`arrange_coplanar_surface_cutter_hole_contact_difference`], and the
/// multi-cutter form of [`arrange_coplanar_surface_side_cutter_difference`],
/// and it still rejects holes, point-only branch contacts, and multiple
/// retained loops except for the separate source-holed case where exactly one
/// right retained hole becomes a filled output component. A one-cutter side
/// opening may still replay here first, preserving older convex/component
/// arrangement classification while the direct side-cutter artifact remains
/// available for explicit validation.
///
/// The loop is promoted only after exact source-component replay and retained
/// area/topology checks, following Yap, "Towards Exact Geometric
/// Computation," *Computational Geometry* 7.1-2 (1997). The boundary
/// construction underneath is the Weiler-Atherton retained-fragment model
/// cited by the multi-difference path; the same-outer source-holed case uses
/// retained mesh-incidence rings plus the exact Sutherland-Hodgman area replay
/// cited by [`same_outer_holed_no_hole_difference_polygons`].
#[cfg(feature = "exact-triangulation")]
pub fn arrange_coplanar_surface_component_difference(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Option<CoplanarSurfaceArrangement> {
    let multi_cutter_side_difference = matches!(connected_face_component_meshes(right), Some(components) if components.len() >= 2)
        && arrange_coplanar_surface_side_cutter_difference(left, right).is_some();
    if arrange_single_triangle_coplanar_difference(left, right).is_some()
        || arrange_coplanar_convex_surface_difference(left, right).is_some()
        || arrange_coplanar_surface_cutter_hole_contact_difference(left, right).is_some()
        || multi_cutter_side_difference
    {
        return None;
    }
    let source_holed_filled_hole = same_outer_holed_no_hole_difference_polygons(left, right);
    let (projection, mut polygons, allow_convex_single, label) =
        if let Some((projection, polygons)) = source_holed_filled_hole {
            (
                projection,
                polygons,
                true,
                "exact coplanar same-outer filled-hole difference",
            )
        } else {
            let (projection, polygons) = coplanar_surface_difference_polygons(left, right)?;
            (
                projection,
                polygons,
                false,
                "exact coplanar nonconvex component difference",
            )
        };
    if polygons.len() != 1 {
        return None;
    }
    let polygon = polygons.pop()?;
    if !allow_convex_single
        && validate_projected_strictly_convex_loop(
            &polygon,
            projection,
            "coplanar nonconvex component difference",
        )
        .is_ok()
    {
        return None;
    }
    let mesh = polygon_to_earcut_open_mesh_with_label(&polygon, projection, label)?;
    let arrangement = CoplanarSurfaceArrangement {
        projection,
        polygon,
        mesh,
    };
    arrangement.validate().ok()?;
    Some(arrangement)
}

/// Replay source-local coplanar difference loops for the multi-output artifacts.
///
/// This helper is the shared object builder for no-hole multi/component
/// differences. Each left component is handled independently: it may be
/// retained unchanged, dropped when exactly covered, opened by side cutters,
/// split into several loops, or consume strict right-side holes through a
/// certified removed opening. The source-local side-cutter case is important
/// for multi-component operands: a component whose side cutters replay as one
/// nonconvex loop can now be emitted beside unrelated retained source loops
/// instead of being forced through the single-loop side-cutter artifact.
///
/// The policy follows Yap, "Towards Exact Geometric Computation,"
/// *Computational Geometry* 7.1-2 (1997): this function promotes only loops
/// produced by exact source-component replay and exact area/topology checks.
/// Side-cutter openings use the Weiler-Atherton retained-fragment traversal
/// from Weiler and Atherton, "Hidden Surface Removal Using Polygon Area
/// Sorting," *SIGGRAPH Computer Graphics* 11.2 (1977).
#[cfg(feature = "exact-triangulation")]
fn coplanar_surface_difference_polygons(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Option<(CoplanarProjection, Vec<Vec<Point3>>)> {
    let left_component_meshes = connected_face_component_meshes(left)?;
    let right_component_meshes = connected_face_component_meshes(right)
        .or_else(|| triangle_piece_component_meshes(right))?;
    if right_component_meshes.is_empty() {
        return None;
    }

    let Some(mut left_components) = left_component_meshes
        .iter()
        .cloned()
        .into_iter()
        .map(|mesh| ConvexUnionComponent::from_mesh(MultiUnionSide::Left, mesh))
        .collect::<Option<Vec<_>>>()
    else {
        return coplanar_simple_surface_difference_polygons(
            left_component_meshes,
            right_component_meshes,
        );
    };
    let Some(right_components) = right_component_meshes
        .iter()
        .cloned()
        .map(|mesh| ConvexUnionComponent::from_mesh(MultiUnionSide::Right, mesh))
        .collect::<Option<Vec<_>>>()
        .or_else(|| {
            triangle_piece_component_meshes(right)?
                .into_iter()
                .map(|mesh| ConvexUnionComponent::from_mesh(MultiUnionSide::Right, mesh))
                .collect::<Option<Vec<_>>>()
        })
    else {
        return coplanar_simple_surface_difference_polygons(
            left_component_meshes,
            right_component_meshes,
        );
    };
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
    let mut polygons = Vec::new();
    for component in &mut left_components {
        let mut drop_component = false;
        let mut cutter_indices = Vec::new();
        let mut holes = Vec::new();
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
                if !convex_polygons_touch_on_positive_boundary(
                    &component.hull,
                    &right_component.hull,
                    projection,
                )? {
                    if polygon_strictly_inside_convex_polygon(
                        &right_component.hull,
                        &component.hull,
                        projection,
                    )? {
                        let mut ring = right_component.hull.clone();
                        orient_polygon_cw(&mut ring, projection)?;
                        holes.push(ComponentHoleCandidate { ring, right_index });
                        continue;
                    }
                    return None;
                }
                if drop_component {
                    return None;
                }
                cutter_indices.push(right_index);
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
        if !holes.is_empty() && cutter_indices.is_empty() {
            return None;
        }
        match cutter_indices.as_slice() {
            [] => polygons.push(component.hull.clone()),
            [right_index] => {
                if !holes.is_empty() {
                    if let Some(opened) =
                        materialize_cutter_hole_contact_multi_component_difference_consuming_holes(
                            component,
                            &cutter_indices,
                            &holes,
                            &right_components,
                            "coplanar no-hole cutter-hole contact consumed-hole difference",
                        )
                    {
                        polygons.extend(opened);
                    } else if let Some(remnants) =
                        materialize_side_cutter_multi_component_difference_consuming_holes(
                            component,
                            &cutter_indices,
                            &holes,
                            &right_components,
                            "coplanar single side-to-side consumed-hole split difference",
                        )
                    {
                        polygons.extend(remnants);
                    } else if let Some(opened) =
                        materialize_side_cutter_opening_difference_consuming_holes(
                            component,
                            &cutter_indices,
                            &holes,
                            &right_components,
                            "coplanar no-hole single side-cutter consumed-hole difference",
                        )
                    {
                        polygons.extend(opened);
                    } else {
                        return None;
                    }
                } else {
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
                    } else if let Some((_, opening)) =
                        materialize_nonrectilinear_side_cutter_opening(
                            component,
                            &cutter_indices,
                            &right_components,
                            "coplanar source-local single side-cutter opening difference",
                        )
                    {
                        polygons.push(opening);
                    } else {
                        return None;
                    }
                }
            }
            _ => {
                let mut remnants = if !holes.is_empty() {
                    if let Some(opened) =
                        materialize_cutter_hole_contact_multi_component_difference_consuming_holes(
                            component,
                            &cutter_indices,
                            &holes,
                            &right_components,
                            "coplanar no-hole cutter-hole contact consumed-hole difference",
                        )
                    {
                        opened
                    } else if let Some(remnants) =
                        materialize_side_cutter_multi_component_difference_consuming_holes(
                            component,
                            &cutter_indices,
                            &holes,
                            &right_components,
                            "coplanar nonconvex multi-component consumed-hole side-cutter difference",
                        )
                    {
                        remnants
                    } else if let Some(opened) =
                        materialize_side_cutter_opening_difference_consuming_holes(
                            component,
                            &cutter_indices,
                            &holes,
                            &right_components,
                            "coplanar no-hole side-cutter opening consumed-hole difference",
                        )
                    {
                        opened
                    } else {
                        return None;
                    }
                } else if let Some(remnants) = materialize_side_cutter_multi_component_difference(
                    component,
                    &cutter_indices,
                    &right_components,
                    "coplanar nonconvex multi-component side-cutter difference",
                ) {
                    remnants
                } else if let Some((_, opening)) = materialize_nonrectilinear_side_cutter_opening(
                    component,
                    &cutter_indices,
                    &right_components,
                    "coplanar source-local side-cutter opening difference",
                ) {
                    vec![opening]
                } else {
                    materialize_component_multi_cutter_difference(
                        component,
                        &cutter_indices,
                        &right_components,
                        projection,
                    )?
                };
                polygons.append(&mut remnants);
            }
        }
    }
    for polygon in &mut polygons {
        orient_polygon_ccw(polygon, projection)?;
    }
    sort_polygons_for_replay(&mut polygons, projection);
    validate_simple_component_loops_disjoint(
        &polygons,
        projection,
        "coplanar nonconvex multi-component difference",
    )
    .ok()?;
    Some((projection, polygons))
}

/// Replay side-attached cutters on nonconvex simple source components.
///
/// This is the bounded nonconvex-source slice of the remaining planar
/// arrangement work. Each left component must replay as one simple source disk
/// by [`SimpleSurfaceComponent`]. Each right component must replay as a convex
/// source object. For a nonconvex left component, a right object may either be
/// disjoint, remove the whole component, or be a side-attached removed loop
/// that lies in the closed source ring and owns positive-length boundary
/// contact. One or more convex cutters may also cross out of the source when
/// their clipped source intersections replay as simple side-owned openings;
/// connected overlapping clipped openings are merged by exact exposed-fragment
/// replay before subtraction. Strict interior cutters would create retained
/// holes, so no-hole artifacts still leave them to the component-holed path.
///
/// The emitted loops are stitched from retained source-boundary fragments and
/// reversed removed-boundary fragments, following Weiler and Atherton,
/// "Hidden Surface Removal Using Polygon Area Sorting," *SIGGRAPH Computer
/// Graphics* 11.2 (1977). Exact area replay is the final promotion gate, in
/// the sense of Yap, "Towards Exact Geometric Computation,"
/// *Computational Geometry* 7.1-2 (1997).
#[cfg(feature = "exact-triangulation")]
fn coplanar_simple_surface_difference_polygons(
    left_component_meshes: Vec<ExactMesh>,
    right_component_meshes: Vec<ExactMesh>,
) -> Option<(CoplanarProjection, Vec<Vec<Point3>>)> {
    let mut left_components = left_component_meshes
        .into_iter()
        .map(SimpleSurfaceComponent::from_mesh)
        .collect::<Option<Vec<_>>>()?;
    let right_components = right_component_meshes
        .into_iter()
        .map(|mesh| ConvexUnionComponent::from_mesh(MultiUnionSide::Right, mesh))
        .collect::<Option<Vec<_>>>()?;
    let projection = left_components.first()?.projection;
    if left_components
        .iter()
        .any(|component| component.projection != projection)
        || right_components
            .iter()
            .any(|component| component.projection != projection)
    {
        return None;
    }
    let left_boundaries = left_components
        .iter()
        .map(|component| component.boundary.clone())
        .collect::<Vec<_>>();
    validate_simple_component_loops_disjoint(
        &left_boundaries,
        projection,
        "coplanar nonconvex source component difference",
    )
    .ok()?;

    let mut polygons = Vec::new();
    for component in &mut left_components {
        let mut drop_component = false;
        let mut removed = Vec::new();
        let mut holes = Vec::new();
        for (right_index, right_component) in right_components.iter().enumerate() {
            if polygons_equal(&component.boundary, &right_component.hull)
                || polygon_in_closed_convex_polygon(
                    &component.boundary,
                    &right_component.hull,
                    projection,
                )?
            {
                if drop_component || !removed.is_empty() {
                    return None;
                }
                drop_component = true;
                continue;
            }
            if polygon_lies_in_closed_simple_polygon(
                &right_component.hull,
                &component.boundary,
                projection,
            )? {
                let attachment_count = simple_boundary_attachment_count(
                    &component.boundary,
                    &right_component.hull,
                    projection,
                )?;
                if attachment_count == 0 {
                    if polygon_strictly_inside_simple_polygon(
                        &right_component.hull,
                        &component.boundary,
                        projection,
                    )? {
                        let mut ring = right_component.hull.clone();
                        orient_polygon_cw(&mut ring, projection)?;
                        holes.push(ComponentHoleCandidate { ring, right_index });
                        continue;
                    }
                    return None;
                }
                let mut cutter = right_component.hull.clone();
                orient_polygon_ccw(&mut cutter, projection)?;
                removed.push(cutter);
                continue;
            }
            match simple_source_convex_region_relation(
                &component.boundary,
                &right_component.hull,
                projection,
            )? {
                SimpleSourceConvexRegionRelation::Disjoint => {}
                SimpleSourceConvexRegionRelation::BoundaryOnly => return None,
                SimpleSourceConvexRegionRelation::UnsupportedCrossing => {
                    let mut clipped = simple_source_convex_crossing_removed_openings(
                        component,
                        &right_component.hull,
                        "coplanar nonconvex source clipped side-cutter difference",
                    )?;
                    removed.append(&mut clipped);
                }
            }
        }
        if drop_component {
            continue;
        }
        if removed.is_empty() && holes.is_empty() {
            polygons.push(component.boundary.clone());
        } else if removed.is_empty() {
            return None;
        } else if !holes.is_empty() {
            let mut remnants =
                materialize_simple_source_removed_opening_hole_contact_difference_consuming_holes(
                    component,
                    &removed,
                    &holes,
                    "coplanar nonconvex source consumed-hole component difference",
                )?;
            polygons.append(&mut remnants);
        } else {
            let mut remnants =
                materialize_simple_source_side_cutter_difference(component, &removed)?;
            polygons.append(&mut remnants);
        }
    }
    for polygon in &mut polygons {
        orient_polygon_ccw(polygon, projection)?;
    }
    sort_polygons_for_replay(&mut polygons, projection);
    validate_simple_component_loops_disjoint(
        &polygons,
        projection,
        "coplanar nonconvex source component difference",
    )
    .ok()?;
    Some((projection, polygons))
}

/// Clip a crossing convex cutter against a nonconvex simple source disk.
///
/// This is a bounded outside-clipping certificate for
/// [`coplanar_simple_surface_difference_polygons`]. The general problem is a
/// planar arrangement, but one convex cutter can be clipped against one exact
/// source disk by replaying the boundary of `source ∩ cutter`: source-boundary
/// fragments whose midpoints lie in the cutter, plus cutter-boundary
/// fragments whose midpoints lie in the source. The emitted removed openings
/// are then rechecked by the existing nonconvex source side-cutter
/// materializer, including source containment, positive-length boundary
/// ownership, connected-opening union, and exact area replay.
///
/// The fragment traversal is the same retained-boundary construction as
/// Weiler and Atherton, "Hidden Surface Removal Using Polygon Area Sorting,"
/// *SIGGRAPH Computer Graphics* 11.2 (1977). Yap, "Towards Exact Geometric
/// Computation," *Computational Geometry* 7.1-2 (1997), is the reason this
/// helper promotes only stitched loops that replay exact source/cutter
/// predicates; point-only and non-simple branch outputs remain unsupported
/// instead of being selected by tolerance samples. Segment intersections use
/// the exact orientation-predicate classifier from Guigue and Devillers,
/// "Fast and Robust Triangle-Triangle Overlap Test Using Orientation
/// Predicates," *Journal of Graphics Tools* 8.1 (2003).
#[cfg(feature = "exact-triangulation")]
fn simple_source_convex_crossing_removed_openings(
    component: &SimpleSurfaceComponent,
    cutter: &[Point3],
    label: &'static str,
) -> Option<Vec<Vec<Point3>>> {
    let projection = component.projection;
    let mut cutter = cutter.to_vec();
    orient_polygon_ccw(&mut cutter, projection)?;
    validate_projected_strictly_convex_loop(&cutter, projection, label).ok()?;

    let mut fragments = Vec::new();
    collect_source_inside_convex_intersection_fragments(
        &component.boundary,
        &cutter,
        projection,
        &mut fragments,
    )?;
    collect_convex_inside_source_intersection_fragments(
        &cutter,
        &component.boundary,
        projection,
        &mut fragments,
    )?;

    let mut openings = if let Some(opening) = stitch_simple_loop(fragments.clone(), projection) {
        vec![opening]
    } else {
        stitch_disjoint_simple_loops(fragments, projection)?
    };
    if openings.is_empty() {
        return None;
    }
    for opening in &mut openings {
        orient_polygon_ccw(opening, projection)?;
        *opening = simplify_projected_polygon(opening.clone(), projection);
        validate_projected_simple_loop(opening, projection, label).ok()?;
        if !polygon_lies_in_closed_simple_polygon(opening, &component.boundary, projection)? {
            return None;
        }
        if simple_boundary_attachment_count(&component.boundary, opening, projection)? == 0 {
            return None;
        }
    }
    validate_simple_component_loops_disjoint(&openings, projection, label).ok()?;
    Some(openings)
}

#[cfg(feature = "exact-triangulation")]
fn collect_source_inside_convex_intersection_fragments(
    source: &[Point3],
    convex: &[Point3],
    projection: CoplanarProjection,
    fragments: &mut Vec<DirectedFragment>,
) -> Option<()> {
    for edge in 0..source.len() {
        let start = &source[edge];
        let end = &source[(edge + 1) % source.len()];
        let mut splits = vec![start.clone(), end.clone()];
        for other_edge in 0..convex.len() {
            add_projected_edge_intersections(
                start,
                end,
                &convex[other_edge],
                &convex[(other_edge + 1) % convex.len()],
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
            if convex_polygon_location(&midpoint, convex, projection)?
                != ConvexPolygonLocation::Outside
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
fn collect_convex_inside_source_intersection_fragments(
    convex: &[Point3],
    source: &[Point3],
    projection: CoplanarProjection,
    fragments: &mut Vec<DirectedFragment>,
) -> Option<()> {
    for edge in 0..convex.len() {
        let start = &convex[edge];
        let end = &convex[(edge + 1) % convex.len()];
        let mut splits = vec![start.clone(), end.clone()];
        for source_edge in 0..source.len() {
            add_projected_edge_intersections(
                start,
                end,
                &source[source_edge],
                &source[(source_edge + 1) % source.len()],
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
            if simple_polygon_location(&midpoint, source, projection)?
                != ConvexPolygonLocation::Outside
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
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum SimpleSourceConvexRegionRelation {
    Disjoint,
    BoundaryOnly,
    UnsupportedCrossing,
}

/// Classify a convex right component against a nonconvex source ring.
///
/// This is intentionally not a clipping predicate. If any positive-area part
/// of the right component might cross the source ring, the bounded nonconvex
/// source path rejects it so a later full planar-cell materializer can own the
/// split topology. Only certified disjoint and boundary-only cases are
/// returned here; side-attached contained cutters are handled before this
/// helper. Segment relations use the exact orientation-predicate classifier
/// described by Guigue and Devillers, "Fast and Robust Triangle-Triangle
/// Overlap Test Using Orientation Predicates," *Journal of Graphics Tools*
/// 8.1 (2003).
#[cfg(feature = "exact-triangulation")]
fn simple_source_convex_region_relation(
    source: &[Point3],
    convex: &[Point3],
    projection: CoplanarProjection,
) -> Option<SimpleSourceConvexRegionRelation> {
    let mut saw_boundary = false;
    for point in convex {
        match simple_polygon_location(point, source, projection)? {
            ConvexPolygonLocation::Inside => {
                return Some(SimpleSourceConvexRegionRelation::UnsupportedCrossing);
            }
            ConvexPolygonLocation::Boundary => saw_boundary = true,
            ConvexPolygonLocation::Outside => {}
        }
    }
    for source_edge in 0..source.len() {
        let source_start = project_point(&source[source_edge], projection);
        let source_end = project_point(&source[(source_edge + 1) % source.len()], projection);
        for convex_edge in 0..convex.len() {
            let convex_start = project_point(&convex[convex_edge], projection);
            let convex_end = project_point(&convex[(convex_edge + 1) % convex.len()], projection);
            match classify_segment_intersection(
                &source_start,
                &source_end,
                &convex_start,
                &convex_end,
            )
            .value()?
            {
                SegmentIntersection::Disjoint => {}
                SegmentIntersection::EndpointTouch => saw_boundary = true,
                SegmentIntersection::CollinearOverlap | SegmentIntersection::Identical => {
                    saw_boundary = true;
                }
                SegmentIntersection::Proper => {
                    return Some(SimpleSourceConvexRegionRelation::UnsupportedCrossing);
                }
            }
        }
    }
    if saw_boundary {
        Some(SimpleSourceConvexRegionRelation::BoundaryOnly)
    } else {
        Some(SimpleSourceConvexRegionRelation::Disjoint)
    }
}

/// Check that a cutter ring is wholly owned by a nonconvex source disk.
///
/// Vertex containment alone is not sufficient for a nonconvex source polygon:
/// a cutter edge joining two inside vertices could still leave and reenter the
/// source. This helper therefore rejects every proper source/cutter edge
/// crossing while allowing exact collinear boundary overlap for side-attached
/// openings. That keeps outside clipping out of this bounded certificate, as
/// required by Yap's exact-object discipline.
#[cfg(feature = "exact-triangulation")]
fn polygon_lies_in_closed_simple_polygon(
    polygon: &[Point3],
    source: &[Point3],
    projection: CoplanarProjection,
) -> Option<bool> {
    for point in polygon {
        if simple_polygon_location(point, source, projection)? == ConvexPolygonLocation::Outside {
            return Some(false);
        }
    }
    for source_edge in 0..source.len() {
        let source_start = project_point(&source[source_edge], projection);
        let source_end = project_point(&source[(source_edge + 1) % source.len()], projection);
        for polygon_edge in 0..polygon.len() {
            let polygon_start = project_point(&polygon[polygon_edge], projection);
            let polygon_end =
                project_point(&polygon[(polygon_edge + 1) % polygon.len()], projection);
            if classify_segment_intersection(
                &source_start,
                &source_end,
                &polygon_start,
                &polygon_end,
            )
            .value()?
                == SegmentIntersection::Proper
            {
                return Some(false);
            }
        }
    }
    Some(true)
}

#[cfg(feature = "exact-triangulation")]
fn simple_boundary_attachment_count(
    outer: &[Point3],
    removed: &[Point3],
    projection: CoplanarProjection,
) -> Option<usize> {
    convex_boundary_attachment_count(outer, removed, projection)
}

/// Certify that a nonconvex source subtraction is truly split by removals.
///
/// The simple-source retained-fragment replay may emit several loops. For the
/// consumed-hole crossing-cutter path that is a stronger topology claim than
/// an ordinary bay opening: at least one source-owned removed loop must have
/// exact positive-length attachment to two or more source boundary edges.
/// This is the nonconvex-source analogue of
/// [`certify_removed_openings_split_source_component`]. It keeps the shortcut
/// in Yap's retained-object model from "Towards Exact Geometric Computation,"
/// *Computational Geometry* 7.1-2 (1997): the split is promoted from exact
/// side ownership plus area replay, not from a sampled point in each output
/// component.
#[cfg(feature = "exact-triangulation")]
fn certify_simple_removed_openings_split_source_component(
    outer: &[Point3],
    removed_openings: &[Vec<Point3>],
    projection: CoplanarProjection,
) -> Option<()> {
    let mut max_attachment_count = 0usize;
    for opening in removed_openings {
        max_attachment_count = max_attachment_count.max(simple_boundary_attachment_count(
            outer, opening, projection,
        )?);
    }
    if max_attachment_count < 2 {
        return None;
    }
    Some(())
}

/// Stitch side-attached removed loops out of a simple nonconvex source disk.
///
/// The output may be one loop or several loops when a removed side channel
/// separates the source disk. Every retained loop must be simple and the exact
/// area equation `area(source) = sum(area(output_i)) + sum(area(removed_j))`
/// must replay before the loops are exported. This is the same retained-edge
/// contract used by the convex side-cutter path, generalized at the
/// source-boundary containment predicate. Point-only contacts between removed
/// openings are admitted only when a positive-area or positive-length contact
/// already connects those openings through the same removed group. In Yap's
/// terminology from "Towards Exact Geometric Computation," *Computational
/// Geometry* 7.1-2 (1997), the point is lower-dimensional evidence that may
/// be replayed after the exact 2D/1D ownership certificate exists; it is never
/// allowed to supply graph connectivity for the boolean topology.
#[cfg(feature = "exact-triangulation")]
fn materialize_simple_source_side_cutter_difference(
    component: &SimpleSurfaceComponent,
    removed: &[Vec<Point3>],
) -> Option<Vec<Vec<Point3>>> {
    materialize_simple_source_side_cutter_difference_core(component, removed)
        .map(|(_, polygons)| polygons)
}

#[cfg(feature = "exact-triangulation")]
fn materialize_simple_source_side_cutter_difference_core(
    component: &SimpleSurfaceComponent,
    removed: &[Vec<Point3>],
) -> Option<(Vec<Vec<Point3>>, Vec<Vec<Point3>>)> {
    if removed.is_empty() {
        return None;
    }
    let projection = component.projection;
    let mut removed = removed.to_vec();
    for polygon in &mut removed {
        orient_polygon_ccw(polygon, projection)?;
        validate_projected_simple_loop(
            polygon,
            projection,
            "coplanar nonconvex source side-cutter difference",
        )
        .ok()?;
        if !polygon_lies_in_closed_simple_polygon(polygon, &component.boundary, projection)? {
            return None;
        }
        if simple_boundary_attachment_count(&component.boundary, polygon, projection)? == 0 {
            return None;
        }
    }
    removed = merge_connected_simple_removed_openings(
        &removed,
        projection,
        "coplanar nonconvex source removed-opening union",
    )?;
    for polygon in &mut removed {
        orient_polygon_ccw(polygon, projection)?;
        validate_projected_simple_loop(
            polygon,
            projection,
            "coplanar nonconvex source side-cutter difference",
        )
        .ok()?;
        if !polygon_lies_in_closed_simple_polygon(polygon, &component.boundary, projection)? {
            return None;
        }
        if simple_boundary_attachment_count(&component.boundary, polygon, projection)? == 0 {
            return None;
        }
    }
    validate_simple_component_loops_disjoint(
        &removed,
        projection,
        "coplanar nonconvex source side-cutter difference",
    )
    .ok()?;

    let mut fragments = Vec::new();
    collect_outer_difference_fragments(&component.boundary, &removed, projection, &mut fragments)?;
    for index in 0..removed.len() {
        collect_simple_removed_difference_fragments(
            index,
            &component.boundary,
            &removed,
            projection,
            &mut fragments,
        )?;
    }
    let mut polygons = if let Some(polygon) = stitch_simple_loop(fragments.clone(), projection) {
        vec![polygon]
    } else {
        stitch_disjoint_simple_loops(fragments, projection)?
    };
    let mut output_area = ExactReal::from(0);
    for polygon in &mut polygons {
        orient_polygon_ccw(polygon, projection)?;
        *polygon = simplify_projected_polygon(polygon.clone(), projection);
        validate_projected_simple_loop(
            polygon,
            projection,
            "coplanar nonconvex source side-cutter difference",
        )
        .ok()?;
        output_area = add(&output_area, &projected_area2_abs(polygon, projection)?);
    }
    validate_simple_component_loops_disjoint(
        &polygons,
        projection,
        "coplanar nonconvex source side-cutter difference",
    )
    .ok()?;
    let mut removed_area = ExactReal::from(0);
    for polygon in &removed {
        removed_area = add(&removed_area, &projected_area2_abs(polygon, projection)?);
    }
    if compare_reals(&add(&output_area, &removed_area), &component.area2_abs).value()
        != Some(Ordering::Equal)
    {
        return None;
    }
    Some((removed, polygons))
}

/// Replay a nonconvex source side-cutter split with exact point branches.
///
/// This is deliberately separate from
/// [`materialize_simple_source_side_cutter_difference_core`]. The ordinary
/// simple-source path requires retained loops and removed openings to be
/// pairwise disjoint. A point-branch difference is a different certified
/// object: removed openings may touch only at exact vertices, and the retained
/// loops may duplicate those branch coordinates across components. Positive
/// removed contacts are still merged into simple openings first, while
/// point-only contacts are kept as branch facts rather than graph edges.
///
/// Retained fragments follow Weiler and Atherton, "Hidden Surface Removal
/// Using Polygon Area Sorting," *SIGGRAPH Computer Graphics* 11.2 (1977).
/// Segment/contact decisions are exact orientation-predicate decisions in the
/// sense of Guigue and Devillers, "Fast and Robust Triangle-Triangle Overlap
/// Test Using Orientation Predicates," *Journal of Graphics Tools* 8.1
/// (2003). The final promotion gate is Yap's exact-object rule from "Towards
/// Exact Geometric Computation," *Computational Geometry* 7.1-2 (1997): the
/// source area must equal retained plus removed area exactly.
#[cfg(feature = "exact-triangulation")]
fn materialize_simple_source_side_cutter_point_touch_difference_core(
    component: &SimpleSurfaceComponent,
    removed: &[Vec<Point3>],
    label: &'static str,
) -> Option<(Vec<Vec<Point3>>, Vec<Vec<Point3>>)> {
    if removed.len() < 2 {
        return None;
    }
    let projection = component.projection;
    let mut removed = removed.to_vec();
    for polygon in &mut removed {
        orient_polygon_ccw(polygon, projection)?;
        validate_projected_simple_loop(polygon, projection, label).ok()?;
        if !polygon_lies_in_closed_simple_polygon(polygon, &component.boundary, projection)? {
            return None;
        }
        if simple_boundary_attachment_count(&component.boundary, polygon, projection)? == 0 {
            return None;
        }
    }

    let (mut removed, saw_point_contact) =
        merge_connected_simple_removed_openings_allowing_branches(
            &removed,
            projection,
            "coplanar nonconvex source point-touch removed-opening union",
        )?;
    if !saw_point_contact || removed.len() < 2 {
        return None;
    }
    for polygon in &mut removed {
        orient_polygon_ccw(polygon, projection)?;
        *polygon = simplify_projected_polygon(polygon.clone(), projection);
        validate_projected_simple_loop(polygon, projection, label).ok()?;
        if !polygon_lies_in_closed_simple_polygon(polygon, &component.boundary, projection)? {
            return None;
        }
        if simple_boundary_attachment_count(&component.boundary, polygon, projection)? == 0 {
            return None;
        }
    }
    validate_simple_component_loops_disjoint_allowing_vertex_point_touches(
        &removed, projection, label,
    )
    .ok()?;

    let mut fragments = Vec::new();
    collect_outer_difference_fragments(&component.boundary, &removed, projection, &mut fragments)?;
    for index in 0..removed.len() {
        collect_simple_removed_difference_fragments(
            index,
            &component.boundary,
            &removed,
            projection,
            &mut fragments,
        )?;
    }
    let mut polygons = stitch_branching_simple_loops(fragments, projection)?;
    if polygons.len() < 2 || !final_loops_have_only_point_touches(&polygons, projection)? {
        return None;
    }

    let mut output_area = ExactReal::from(0);
    for polygon in &mut polygons {
        orient_polygon_ccw(polygon, projection)?;
        *polygon = simplify_projected_polygon(polygon.clone(), projection);
        validate_projected_simple_loop(polygon, projection, label).ok()?;
        output_area = add(&output_area, &projected_area2_abs(polygon, projection)?);
    }
    validate_simple_component_loops_disjoint_allowing_vertex_point_touches(
        &polygons, projection, label,
    )
    .ok()?;

    let mut removed_area = ExactReal::from(0);
    for polygon in &removed {
        removed_area = add(&removed_area, &projected_area2_abs(polygon, projection)?);
    }
    if compare_reals(&add(&output_area, &removed_area), &component.area2_abs).value()
        != Some(Ordering::Equal)
    {
        return None;
    }
    sort_polygons_for_replay(&mut polygons, projection);
    Some((removed, polygons))
}

/// Merge positive-connected simple removed openings without erasing branches.
///
/// Ordinary simple-source removal treats point-only opening contacts as
/// unsupported, because there is no single simple removed loop to subtract.
/// The point-touch artifact needs the opposite policy: positive-dimensional
/// contacts are still merged into one removed opening, but point-only contacts
/// merely authorize the branch-aware retained-loop walk. The returned boolean
/// records whether at least one exact point-only contact was observed.
#[cfg(feature = "exact-triangulation")]
fn merge_connected_simple_removed_openings_allowing_branches(
    openings: &[Vec<Point3>],
    projection: CoplanarProjection,
    label: &'static str,
) -> Option<(Vec<Vec<Point3>>, bool)> {
    if openings.len() < 2 {
        return Some((openings.to_vec(), false));
    }
    let mut contact_graph = UnionFind::new(openings.len());
    let mut saw_point_contact = false;
    for left in 0..openings.len() {
        for right in left + 1..openings.len() {
            match simple_polygon_interaction(&openings[left], &openings[right], projection)? {
                SimplePolygonInteraction::Disjoint => {}
                SimplePolygonInteraction::PointOnly => saw_point_contact = true,
                SimplePolygonInteraction::Connected => contact_graph.union(left, right),
            }
        }
    }

    let mut groups: Vec<(usize, Vec<usize>)> = Vec::new();
    for index in 0..openings.len() {
        let root = contact_graph.find(index);
        if let Some((_, members)) = groups.iter_mut().find(|(candidate, _)| *candidate == root) {
            members.push(index);
        } else {
            groups.push((root, vec![index]));
        }
    }
    groups.sort_by_key(|(_, members)| members.first().copied().unwrap_or(usize::MAX));

    let mut merged = Vec::with_capacity(groups.len());
    for (_, members) in groups {
        let mut opening = if members.len() == 1 {
            openings[members[0]].clone()
        } else {
            materialize_simple_removed_opening_union_group(openings, &members, projection, label)?
        };
        orient_polygon_ccw(&mut opening, projection)?;
        opening = simplify_projected_polygon(opening, projection);
        validate_projected_simple_loop(&opening, projection, label).ok()?;
        merged.push(opening);
    }
    Some((merged, saw_point_contact))
}

/// Merge connected removed openings before subtracting from a simple source.
///
/// Clipped crossing cutters can overlap after `source ∩ cutter` replay even
/// when each individual clipped opening is a valid side-owned removed loop.
/// The general arrangement is still out of scope, but connected openings that
/// stitch into one simple union boundary can be certified locally: exposed
/// boundary fragments from every opening are replayed, every original opening
/// must lie in the stitched union, and the union area must not exceed the sum
/// of the exact input areas. This is the same retained-object discipline Yap
/// argues for in "Towards Exact Geometric Computation," *Computational
/// Geometry* 7.1-2 (1997): the merged topology is promoted only from retained
/// exact boundary facts and exact area inequalities, not by choosing sample
/// points in overlapping bays.
///
/// Point-only contacts do not add connectivity because they introduce a branch
/// vertex for the later planar-cell extractor. A point contact is accepted
/// only when both openings are already connected by positive-area or
/// positive-length contact through other openings; this mirrors the
/// lower-dimensional-incidence policy used by the convex side-cutter and
/// cutter/hole materializers. Boundary and crossing tests use the exact
/// orientation-predicate classifier of Guigue and Devillers, "Fast and Robust
/// Triangle-Triangle Overlap Test Using Orientation Predicates," *Journal of
/// Graphics Tools* 8.1 (2003). The exposed-fragment replay follows the
/// Weiler-Atherton boundary traversal; see Weiler and Atherton, "Hidden
/// Surface Removal Using Polygon Area Sorting," *SIGGRAPH Computer Graphics*
/// 11.2 (1977).
#[cfg(feature = "exact-triangulation")]
fn merge_connected_simple_removed_openings(
    openings: &[Vec<Point3>],
    projection: CoplanarProjection,
    label: &'static str,
) -> Option<Vec<Vec<Point3>>> {
    if openings.len() < 2 {
        return Some(openings.to_vec());
    }
    let mut contact_graph = UnionFind::new(openings.len());
    let mut point_only_contacts = Vec::new();
    for left in 0..openings.len() {
        for right in left + 1..openings.len() {
            match simple_polygon_interaction(&openings[left], &openings[right], projection)? {
                SimplePolygonInteraction::Disjoint => {}
                SimplePolygonInteraction::PointOnly => point_only_contacts.push((left, right)),
                SimplePolygonInteraction::Connected => contact_graph.union(left, right),
            }
        }
    }
    for (left, right) in point_only_contacts {
        if contact_graph.find(left) != contact_graph.find(right) {
            return None;
        }
    }

    let mut groups: Vec<(usize, Vec<usize>)> = Vec::new();
    for index in 0..openings.len() {
        let root = contact_graph.find(index);
        if let Some((_, members)) = groups.iter_mut().find(|(candidate, _)| *candidate == root) {
            members.push(index);
        } else {
            groups.push((root, vec![index]));
        }
    }
    groups.sort_by_key(|(_, members)| members.first().copied().unwrap_or(usize::MAX));

    let mut merged = Vec::with_capacity(groups.len());
    for (_, group) in groups {
        if group.len() == 1 {
            merged.push(openings[group[0]].clone());
        } else {
            merged.push(materialize_simple_removed_opening_union_group(
                openings, &group, projection, label,
            )?);
        }
    }
    Some(merged)
}

#[cfg(feature = "exact-triangulation")]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum SimplePolygonInteraction {
    Disjoint,
    PointOnly,
    Connected,
}

#[cfg(feature = "exact-triangulation")]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum SimplePolygonContact {
    Disjoint,
    PointOnly,
    PositiveLengthBoundary,
    PositiveArea,
}

#[cfg(feature = "exact-triangulation")]
fn simple_polygon_interaction(
    left: &[Point3],
    right: &[Point3],
    projection: CoplanarProjection,
) -> Option<SimplePolygonInteraction> {
    let mut saw_point = false;
    for left_edge in 0..left.len() {
        let left_start = project_point(&left[left_edge], projection);
        let left_end = project_point(&left[(left_edge + 1) % left.len()], projection);
        for right_edge in 0..right.len() {
            let right_start = project_point(&right[right_edge], projection);
            let right_end = project_point(&right[(right_edge + 1) % right.len()], projection);
            match classify_segment_intersection(&left_start, &left_end, &right_start, &right_end)
                .value()?
            {
                SegmentIntersection::Proper
                | SegmentIntersection::CollinearOverlap
                | SegmentIntersection::Identical => {
                    return Some(SimplePolygonInteraction::Connected);
                }
                SegmentIntersection::EndpointTouch => saw_point = true,
                SegmentIntersection::Disjoint => {}
            }
        }
    }
    for point in left {
        if simple_polygon_location(point, right, projection)? == ConvexPolygonLocation::Inside {
            return Some(SimplePolygonInteraction::Connected);
        }
    }
    for point in right {
        if simple_polygon_location(point, left, projection)? == ConvexPolygonLocation::Inside {
            return Some(SimplePolygonInteraction::Connected);
        }
    }
    if saw_point {
        Some(SimplePolygonInteraction::PointOnly)
    } else {
        Some(SimplePolygonInteraction::Disjoint)
    }
}

/// Classify the dimensionality of contact between two retained simple loops.
///
/// Guigue and Devillers' orientation-predicate segment relation separates
/// proper crossings from collinear boundary overlap, and exact simple-polygon
/// location separates strict interior containment from lower-dimensional
/// contact. Yap's exact-computation model is the reason this returns `None`
/// rather than guessing when any predicate cannot certify the topology.
#[cfg(feature = "exact-triangulation")]
fn simple_polygon_contact(
    left: &[Point3],
    right: &[Point3],
    projection: CoplanarProjection,
) -> Option<SimplePolygonContact> {
    let mut saw_point = false;
    let mut saw_positive_length_boundary = false;
    for left_edge in 0..left.len() {
        let left_start = project_point(&left[left_edge], projection);
        let left_end = project_point(&left[(left_edge + 1) % left.len()], projection);
        for right_edge in 0..right.len() {
            let right_start = project_point(&right[right_edge], projection);
            let right_end = project_point(&right[(right_edge + 1) % right.len()], projection);
            match classify_segment_intersection(&left_start, &left_end, &right_start, &right_end)
                .value()?
            {
                SegmentIntersection::Proper => return Some(SimplePolygonContact::PositiveArea),
                SegmentIntersection::CollinearOverlap | SegmentIntersection::Identical => {
                    saw_positive_length_boundary = true;
                }
                SegmentIntersection::EndpointTouch => saw_point = true,
                SegmentIntersection::Disjoint => {}
            }
        }
    }

    let mut all_left_vertices_on_right_boundary = !left.is_empty();
    for point in left {
        match simple_polygon_location(point, right, projection)? {
            ConvexPolygonLocation::Inside => return Some(SimplePolygonContact::PositiveArea),
            ConvexPolygonLocation::Boundary => {}
            ConvexPolygonLocation::Outside => all_left_vertices_on_right_boundary = false,
        }
    }
    let mut all_right_vertices_on_left_boundary = !right.is_empty();
    for point in right {
        match simple_polygon_location(point, left, projection)? {
            ConvexPolygonLocation::Inside => return Some(SimplePolygonContact::PositiveArea),
            ConvexPolygonLocation::Boundary => {}
            ConvexPolygonLocation::Outside => all_right_vertices_on_left_boundary = false,
        }
    }

    if all_left_vertices_on_right_boundary || all_right_vertices_on_left_boundary {
        return Some(SimplePolygonContact::PositiveArea);
    }
    if saw_positive_length_boundary {
        Some(SimplePolygonContact::PositiveLengthBoundary)
    } else if saw_point {
        Some(SimplePolygonContact::PointOnly)
    } else {
        Some(SimplePolygonContact::Disjoint)
    }
}

#[cfg(feature = "exact-triangulation")]
fn final_loops_have_only_point_touches(
    polygons: &[Vec<Point3>],
    projection: CoplanarProjection,
) -> Option<bool> {
    if polygons.len() < 2 {
        return Some(false);
    }
    let mut saw_point_touch = false;
    for left in 0..polygons.len() {
        for right in left + 1..polygons.len() {
            match simple_polygon_contact(&polygons[left], &polygons[right], projection)? {
                SimplePolygonContact::Disjoint => {}
                SimplePolygonContact::PointOnly => saw_point_touch = true,
                SimplePolygonContact::PositiveLengthBoundary
                | SimplePolygonContact::PositiveArea => {
                    return Some(false);
                }
            }
        }
    }
    Some(saw_point_touch)
}

#[cfg(feature = "exact-triangulation")]
fn materialize_simple_removed_opening_union_group(
    openings: &[Vec<Point3>],
    group: &[usize],
    projection: CoplanarProjection,
    label: &'static str,
) -> Option<Vec<Point3>> {
    materialize_simple_polygon_union_group(openings, group, projection, label)
}

/// Materialize one connected union of retained simple polygon loops.
///
/// The helper is shared by removed-opening replay and nonconvex source-union
/// replay. It keeps only boundary fragments exposed to the exterior of all
/// other loops, stitches those fragments into one simple ring, verifies every
/// source loop lies in the retained ring, and rejects any candidate whose area
/// exceeds the sum of exact input areas. That last inequality is a compact
/// Yap-style promotion gate from "Towards Exact Geometric Computation,"
/// *Computational Geometry* 7.1-2 (1997): a stitched outer ring that silently
/// fills an unsupported hole cannot be accepted as a boolean result. The
/// retained-fragment traversal is the Weiler-Atherton idea from Weiler and
/// Atherton, "Hidden Surface Removal Using Polygon Area Sorting," *SIGGRAPH
/// Computer Graphics* 11.2 (1977).
#[cfg(feature = "exact-triangulation")]
fn materialize_simple_polygon_union_group(
    polygons: &[Vec<Point3>],
    group: &[usize],
    projection: CoplanarProjection,
    label: &'static str,
) -> Option<Vec<Point3>> {
    let group_openings = group
        .iter()
        .map(|&index| polygons[index].clone())
        .collect::<Vec<_>>();
    let mut fragments = Vec::new();
    for index in 0..group_openings.len() {
        collect_simple_union_boundary_fragments(
            index,
            &group_openings,
            projection,
            &mut fragments,
        )?;
    }
    let mut union_loops = stitch_simple_union_loops(fragments, projection)?;
    if union_loops.len() != 1 {
        return None;
    }
    let mut union = union_loops.pop()?;
    orient_polygon_ccw(&mut union, projection)?;
    union = simplify_projected_polygon(union, projection);
    validate_projected_simple_loop(&union, projection, label).ok()?;

    let mut input_area = ExactReal::from(0);
    for opening in &group_openings {
        if !polygon_lies_in_closed_simple_polygon(opening, &union, projection)? {
            return None;
        }
        input_area = add(&input_area, &projected_area2_abs(opening, projection)?);
    }
    let union_area = projected_area2_abs(&union, projection)?;
    match compare_reals(&union_area, &input_area).value()? {
        Ordering::Greater => None,
        Ordering::Less | Ordering::Equal => Some(union),
    }
}

#[cfg(feature = "exact-triangulation")]
fn stitch_simple_union_loops(
    mut fragments: Vec<DirectedFragment>,
    projection: CoplanarProjection,
) -> Option<Vec<Vec<Point3>>> {
    if fragments.len() < 3 {
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
            let (next_index, reversed) =
                fragments.iter().enumerate().find_map(|(index, fragment)| {
                    if points_equal(&fragment.start, &current) {
                        Some((index, false))
                    } else if points_equal(&fragment.end, &current) {
                        Some((index, true))
                    } else {
                        None
                    }
                })?;
            let next = fragments.remove(next_index);
            polygon.push(if reversed { next.start } else { next.end });
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
    Some(loops)
}

#[cfg(feature = "exact-triangulation")]
fn collect_simple_union_boundary_fragments(
    polygon_index: usize,
    polygons: &[Vec<Point3>],
    projection: CoplanarProjection,
    fragments: &mut Vec<DirectedFragment>,
) -> Option<()> {
    let polygon = polygons.get(polygon_index)?;
    for edge in 0..polygon.len() {
        let start = &polygon[edge];
        let end = &polygon[(edge + 1) % polygon.len()];
        let mut splits = vec![start.clone(), end.clone()];
        for (other_index, other) in polygons.iter().enumerate() {
            if other_index == polygon_index {
                continue;
            }
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
        }
        sort_points_along_segment(&mut splits, start, end, projection)?;
        dedup_points(&mut splits);
        for pair in splits.windows(2) {
            let a = &pair[0];
            let b = &pair[1];
            if points_equal(a, b) {
                continue;
            }
            if simple_union_fragment_is_exposed(a, b, polygon_index, polygons, projection)? {
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
fn simple_union_fragment_is_exposed(
    a: &Point3,
    b: &Point3,
    polygon_index: usize,
    polygons: &[Vec<Point3>],
    projection: CoplanarProjection,
) -> Option<bool> {
    let midpoint = midpoint3(a, b);
    for (other_index, other) in polygons.iter().enumerate() {
        if other_index == polygon_index {
            continue;
        }
        match simple_polygon_location(&midpoint, other, projection)? {
            ConvexPolygonLocation::Outside => {}
            ConvexPolygonLocation::Inside => return Some(false),
            ConvexPolygonLocation::Boundary => {
                if polygon_index < other_index
                    && segment_has_same_direction_boundary_overlap(a, b, other, projection)?
                {
                    continue;
                }
                return Some(false);
            }
        }
    }
    Some(true)
}

#[cfg(feature = "exact-triangulation")]
fn segment_has_same_direction_boundary_overlap(
    a: &Point3,
    b: &Point3,
    polygon: &[Point3],
    projection: CoplanarProjection,
) -> Option<bool> {
    let start = project_point(a, projection);
    let end = project_point(b, projection);
    for edge in 0..polygon.len() {
        let other_start = project_point(&polygon[edge], projection);
        let other_end = project_point(&polygon[(edge + 1) % polygon.len()], projection);
        match classify_segment_intersection(&start, &end, &other_start, &other_end).value()? {
            SegmentIntersection::CollinearOverlap | SegmentIntersection::Identical => {
                let dx = sub(&end.x, &start.x);
                let dy = sub(&end.y, &start.y);
                let other_dx = sub(&other_end.x, &other_start.x);
                let other_dy = sub(&other_end.y, &other_start.y);
                let dot = add(&mul(&dx, &other_dx), &mul(&dy, &other_dy));
                return Some(
                    compare_reals(&dot, &ExactReal::from(0)).value()? == Ordering::Greater,
                );
            }
            SegmentIntersection::Disjoint
            | SegmentIntersection::EndpointTouch
            | SegmentIntersection::Proper => {}
        }
    }
    Some(false)
}

#[cfg(feature = "exact-triangulation")]
fn collect_simple_removed_difference_fragments(
    removed_index: usize,
    outer: &[Point3],
    removed: &[Vec<Point3>],
    projection: CoplanarProjection,
    fragments: &mut Vec<DirectedFragment>,
) -> Option<()> {
    let polygon = removed.get(removed_index)?;
    for edge in 0..polygon.len() {
        let start = &polygon[edge];
        let end = &polygon[(edge + 1) % polygon.len()];
        let mut splits = vec![start.clone(), end.clone()];
        for outer_edge in 0..outer.len() {
            add_projected_edge_intersections(
                start,
                end,
                &outer[outer_edge],
                &outer[(outer_edge + 1) % outer.len()],
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
            if simple_polygon_location(&midpoint, outer, projection)?
                != ConvexPolygonLocation::Inside
            {
                continue;
            }
            if !point_outside_other_simple_polygons(&midpoint, removed_index, removed, projection)?
            {
                continue;
            }
            fragments.push(DirectedFragment {
                start: b.clone(),
                end: a.clone(),
            });
        }
    }
    Some(())
}

/// Certify a side-cutter-only nonconvex coplanar difference.
///
/// This is the no-hole sibling of
/// [`arrange_coplanar_convex_surface_component_holed_difference`]'s
/// non-rectilinear side-cutter opening path. A single convex source component
/// is cut by one or more side-attached convex right components. Their clipped
/// material is replayed as one or more exact removed openings, each attached
/// to the outer boundary, and the retained boundary is accepted only if it
/// stitches into one nonconvex simple loop with exact area replay:
/// `area(left) = area(output) + sum(area(removed_i))`.
///
/// The helper deliberately rejects strict interior right components, fully
/// rectangular cutter sets, point-only cutter contacts, convex remnants, and
/// multi-loop remnants. Single-triangle corner cuts stay with the older
/// triangle arrangement certificate so widening this artifact does not reorder
/// exact support classification. Rectangular cases belong to the orthogonal
/// cell materializer; branch and split cases remain for the general
/// planar-cell materializer. This follows Yap, "Towards Exact Geometric
/// Computation,"
/// *Computational Geometry* 7.1-2 (1997): topology is promoted only from
/// retained exact boundary and area facts. The boundary splice follows the
/// Weiler-Atherton retained-fragment construction; see Weiler and Atherton,
/// "Hidden Surface Removal Using Polygon Area Sorting," *SIGGRAPH Computer
/// Graphics* 11.2 (1977).
#[cfg(feature = "exact-triangulation")]
pub fn arrange_coplanar_surface_side_cutter_difference(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Option<CoplanarSurfaceArrangement> {
    let left_components = connected_face_component_meshes(left)?;
    let right_components = connected_face_component_meshes(right)?;
    if left_components.len() != 1 || right_components.is_empty() {
        return None;
    }
    if right_components.len() == 1 && left.triangles().len() == 1 && right.triangles().len() == 1 {
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

    let mut cut_indices = Vec::new();
    for (right_index, right_component) in right_components.iter().enumerate() {
        if polygons_equal(&left_component.hull, &right_component.hull)
            || polygon_in_closed_convex_polygon(
                &left_component.hull,
                &right_component.hull,
                projection,
            )?
        {
            return None;
        }
        if polygon_in_closed_convex_polygon(
            &right_component.hull,
            &left_component.hull,
            projection,
        )? {
            if convex_polygons_touch_on_positive_boundary(
                &left_component.hull,
                &right_component.hull,
                projection,
            )? {
                cut_indices.push(right_index);
                continue;
            }
            return None;
        }

        match convex_union_component_relation(
            &left_component.hull,
            &right_component.hull,
            projection,
        )? {
            ConvexUnionComponentRelation::Disjoint => {}
            ConvexUnionComponentRelation::BoundaryOnly => return None,
            ConvexUnionComponentRelation::PositiveArea => cut_indices.push(right_index),
        }
    }
    if cut_indices.is_empty() {
        return None;
    }

    let (_, polygon) = materialize_nonrectilinear_side_cutter_opening(
        &left_component,
        &cut_indices,
        &right_components,
        "coplanar side-cutter simple-loop difference",
    )?;
    let mesh = polygon_to_earcut_open_mesh_with_label(
        &polygon,
        projection,
        "exact coplanar side-cutter simple-loop difference",
    )?;
    let arrangement = CoplanarSurfaceArrangement {
        projection,
        polygon,
        mesh,
    };
    arrangement.validate().ok()?;
    Some(arrangement)
}

/// Certify a side-cutter difference whose retained components meet at points.
///
/// The ordinary side-cutter difference requires one simple retained loop, and
/// the multi-difference requires strictly disjoint retained loops. This helper
/// covers the bounded branch case between them: two or more non-rectilinear
/// clipped side openings touch only at exact vertices, so subtracting them
/// leaves retained simple loops that share those branch coordinates. The mesh
/// duplicates branch coordinates across component loops and validates the
/// exact area equation before export.
///
/// The certificate is source-local. A multi-component left operand may retain
/// unaffected convex source components while one or more other components
/// produce point-branch remnants. This is still not a general planar
/// arrangement: retained holes, boundary-only ambiguities, and non-branch
/// single-cut remnants stay on their narrower artifacts. Strict interior
/// holes may be deleted here only when exact containment proves every ring is
/// wholly owned by one removed branch opening. The retained-object split
/// follows Yap, "Towards Exact Geometric Computation," *Computational
/// Geometry* 7.1-2 (1997): every emitted loop is either an unchanged exact
/// source hull or the replay of a local branch subtraction, and every omitted
/// ring has a named removed owner. The branch subtraction itself uses the
/// Weiler-Atherton retained-fragment walk cited by
/// [`materialize_side_cutter_point_touch_difference_core`].
#[cfg(feature = "exact-triangulation")]
pub fn arrange_coplanar_surface_point_touch_difference(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Option<CoplanarSurfacePointTouchDifference> {
    let left_components = connected_face_component_meshes(left)?;
    let right_components = connected_face_component_meshes(right)?;
    if left_components.is_empty() || right_components.len() < 2 {
        return None;
    }

    let Some(left_components) = left_components
        .iter()
        .cloned()
        .map(|mesh| ConvexUnionComponent::from_mesh(MultiUnionSide::Left, mesh))
        .collect::<Option<Vec<_>>>()
    else {
        return arrange_coplanar_simple_surface_point_touch_difference(
            left_components,
            right_components,
        );
    };
    let right_components = right_components
        .into_iter()
        .map(|mesh| ConvexUnionComponent::from_mesh(MultiUnionSide::Right, mesh))
        .collect::<Option<Vec<_>>>()?;
    let projection = left_components.first()?.projection;
    if left_components
        .iter()
        .any(|component| component.projection != projection)
        || right_components
            .iter()
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
        "coplanar point-touch surface difference",
    )
    .ok()?;

    let mut polygons = Vec::new();
    let mut emitted_branch = false;
    for left_component in &left_components {
        let mut dropped = false;
        let mut cut_indices = Vec::new();
        let mut holes = Vec::new();
        for (right_index, right_component) in right_components.iter().enumerate() {
            if polygons_equal(&left_component.hull, &right_component.hull)
                || polygon_in_closed_convex_polygon(
                    &left_component.hull,
                    &right_component.hull,
                    projection,
                )?
            {
                if dropped || !cut_indices.is_empty() {
                    return None;
                }
                dropped = true;
                continue;
            }
            if polygon_in_closed_convex_polygon(
                &right_component.hull,
                &left_component.hull,
                projection,
            )? {
                if dropped {
                    return None;
                }
                if convex_polygons_touch_on_positive_boundary(
                    &left_component.hull,
                    &right_component.hull,
                    projection,
                )? {
                    cut_indices.push(right_index);
                    continue;
                }
                if polygon_strictly_inside_convex_polygon(
                    &right_component.hull,
                    &left_component.hull,
                    projection,
                )? {
                    let mut ring = right_component.hull.clone();
                    orient_polygon_cw(&mut ring, projection)?;
                    holes.push(ComponentHoleCandidate { ring, right_index });
                    continue;
                }
                return None;
            }

            match convex_union_component_relation(
                &left_component.hull,
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
        if cut_indices.is_empty() {
            if !holes.is_empty() {
                return None;
            }
            polygons.push(left_component.hull.clone());
            continue;
        }
        if cut_indices.len() < 2 {
            return None;
        }
        let mut branch_polygons = if holes.is_empty() {
            materialize_side_cutter_point_touch_difference_core(
                left_component,
                &cut_indices,
                &right_components,
                "coplanar point-touch side-cutter difference",
            )?
            .1
        } else if let Some(polygons) =
            materialize_side_cutter_point_touch_difference_consuming_holes(
                left_component,
                &cut_indices,
                &holes,
                &right_components,
                "coplanar point-touch consumed-hole side-cutter difference",
            )
        {
            polygons
        } else {
            materialize_side_cutter_point_touch_difference_consuming_hole_contacts(
                left_component,
                &cut_indices,
                &holes,
                &right_components,
                "coplanar point-touch straddling-hole side-cutter difference",
            )
            .or_else(|| {
                materialize_side_cutter_point_touch_difference_consuming_hole_contact_groups(
                    left_component,
                    &cut_indices,
                    &holes,
                    &right_components,
                    "coplanar point-touch grouped straddling-hole side-cutter difference",
                )
            })?
        };
        emitted_branch = true;
        polygons.append(&mut branch_polygons);
    }
    if !emitted_branch || polygons.is_empty() {
        return None;
    }
    for polygon in &mut polygons {
        orient_polygon_ccw(polygon, projection)?;
    }
    sort_polygons_for_replay(&mut polygons, projection);
    let mesh = polygons_to_earcut_open_mesh_with_label(
        &polygons,
        projection,
        "exact coplanar point-touch side-cutter difference",
    )?;
    let arrangement = CoplanarSurfacePointTouchDifference {
        projection,
        polygons,
        mesh,
    };
    arrangement.validate().ok()?;
    Some(arrangement)
}

/// Certify point-branch side-cutter differences on simple nonconvex disks.
///
/// The convex-source point-touch difference above proves branch topology by
/// clipping convex cutters against one convex outer boundary. This sibling
/// keeps the same public artifact but imports the left side through
/// [`SimpleSurfaceComponent`]: the exact mesh boundary must replay as one
/// source-owned disk, each removed opening must be contained in that disk and
/// own positive-length source boundary, and point-only contact between removed
/// openings is retained as branch evidence instead of connectivity.
///
/// Positive-dimensional removed contacts are merged first; point-only contacts
/// remain explicit groups. Retained output loops are then stitched from exact
/// source and removed-boundary fragments with the branch-aware
/// Weiler-Atherton walk, and the exact area equation is replayed before the
/// mesh is exported. This follows Yap's retained-object requirement from
/// "Towards Exact Geometric Computation," *Computational Geometry* 7.1-2
/// (1997): a point branch is accepted only when exact topology, containment,
/// and area facts name the output object. Strict holes may be omitted only
/// when the same exact branch replay proves that one removed opening owns the
/// entire ring. For multi-component simple-source operands this remains
/// source-local: unaffected disks are copied as exact retained loops, covered
/// disks may be dropped, and only the components with branch side cutters are
/// replayed by the branch subtraction core.
#[cfg(feature = "exact-triangulation")]
fn arrange_coplanar_simple_surface_point_touch_difference(
    left_component_meshes: Vec<ExactMesh>,
    right_component_meshes: Vec<ExactMesh>,
) -> Option<CoplanarSurfacePointTouchDifference> {
    if left_component_meshes.is_empty() || right_component_meshes.len() < 2 {
        return None;
    }
    let mut left_components = left_component_meshes
        .into_iter()
        .map(SimpleSurfaceComponent::from_mesh)
        .collect::<Option<Vec<_>>>()?;
    let right_components = right_component_meshes
        .into_iter()
        .map(|mesh| ConvexUnionComponent::from_mesh(MultiUnionSide::Right, mesh))
        .collect::<Option<Vec<_>>>()?;
    let projection = left_components.first()?.projection;
    if left_components
        .iter()
        .any(|component| component.projection != projection)
        || right_components
            .iter()
            .any(|component| component.projection != projection)
    {
        return None;
    }
    let left_boundaries = left_components
        .iter()
        .map(|component| component.boundary.clone())
        .collect::<Vec<_>>();
    validate_simple_component_loops_disjoint(
        &left_boundaries,
        projection,
        "coplanar nonconvex source point-touch side-cutter difference",
    )
    .ok()?;

    let mut polygons = Vec::new();
    let mut emitted_branch = false;
    for component in &mut left_components {
        let mut dropped = false;
        let mut removed = Vec::new();
        let mut holes = Vec::new();
        for (right_index, right_component) in right_components.iter().enumerate() {
            if polygons_equal(&component.boundary, &right_component.hull)
                || polygon_in_closed_convex_polygon(
                    &component.boundary,
                    &right_component.hull,
                    projection,
                )?
            {
                if dropped || !removed.is_empty() {
                    return None;
                }
                dropped = true;
                continue;
            }
            if polygon_lies_in_closed_simple_polygon(
                &right_component.hull,
                &component.boundary,
                projection,
            )? {
                if dropped {
                    return None;
                }
                if simple_boundary_attachment_count(
                    &component.boundary,
                    &right_component.hull,
                    projection,
                )? == 0
                {
                    if polygon_strictly_inside_simple_polygon(
                        &right_component.hull,
                        &component.boundary,
                        projection,
                    )? {
                        let mut ring = right_component.hull.clone();
                        orient_polygon_cw(&mut ring, projection)?;
                        holes.push(ComponentHoleCandidate { ring, right_index });
                        continue;
                    }
                    return None;
                }
                let mut cutter = right_component.hull.clone();
                orient_polygon_ccw(&mut cutter, projection)?;
                removed.push(cutter);
                continue;
            }
            match simple_source_convex_region_relation(
                &component.boundary,
                &right_component.hull,
                projection,
            )? {
                SimpleSourceConvexRegionRelation::Disjoint => {}
                SimpleSourceConvexRegionRelation::BoundaryOnly => return None,
                SimpleSourceConvexRegionRelation::UnsupportedCrossing => {
                    if dropped {
                        return None;
                    }
                    let mut clipped = simple_source_convex_crossing_removed_openings(
                        component,
                        &right_component.hull,
                        "coplanar nonconvex source point-touch clipped side-cutter difference",
                    )?;
                    removed.append(&mut clipped);
                }
            }
        }
        if dropped {
            continue;
        }
        if removed.is_empty() {
            if !holes.is_empty() {
                return None;
            }
            polygons.push(component.boundary.clone());
            continue;
        }
        if removed.len() < 2 {
            return None;
        }
        let mut branch_polygons = if holes.is_empty() {
            materialize_simple_source_side_cutter_point_touch_difference_core(
                component,
                &removed,
                "coplanar nonconvex source point-touch side-cutter difference",
            )?
            .1
        } else if let Some(polygons) =
            materialize_simple_source_side_cutter_point_touch_difference_consuming_holes(
                component,
                &removed,
                &holes,
                "coplanar nonconvex source point-touch consumed-hole side-cutter difference",
            )
        {
            polygons
        } else {
            materialize_simple_source_side_cutter_point_touch_difference_consuming_hole_contacts(
                component,
                &removed,
                &holes,
                "coplanar nonconvex source point-touch straddling-hole side-cutter difference",
            )
            .or_else(|| {
                materialize_simple_source_side_cutter_point_touch_difference_consuming_hole_contact_groups(
                    component,
                    &removed,
                    &holes,
                    "coplanar nonconvex source point-touch grouped straddling-hole side-cutter difference",
                )
            })?
        };
        emitted_branch = true;
        polygons.append(&mut branch_polygons);
    }
    if !emitted_branch || polygons.is_empty() {
        return None;
    }
    for polygon in &mut polygons {
        orient_polygon_ccw(polygon, projection)?;
    }
    sort_polygons_for_replay(&mut polygons, projection);
    let mesh = polygons_to_earcut_open_mesh_with_label(
        &polygons,
        projection,
        "exact coplanar nonconvex source point-touch side-cutter difference",
    )?;
    let arrangement = CoplanarSurfacePointTouchDifference {
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
/// must reject: a side-attached cutter either touches a strictly contained
/// hole along a positive-length boundary or overlaps it in positive area, so
/// the result is no longer a holed component but one nonconvex simple loop.
/// The accepted source shape is intentionally replayable: one exact
/// axis-aligned rectangular left component, one or more strictly contained
/// convex right holes, and one or more convex right cutters whose clipped
/// material regions form one or more contact/overlap chains from holes to
/// outer sides. When independent cutter-only side openings coexist with a
/// cutter/hole chain, each connected removed-region group is replayed first
/// and then the common outer boundary is opened by all groups together. The
/// two-component rectangular contact case uses exact interval-cell replay;
/// non-rectangular contact, overlap, and chain cases stitch exact simple union
/// loops from retained convex boundary fragments before the left side is
/// opened. Point coincidences inside a removed group are accepted only when
/// positive-area or positive-length contacts already connect the same group,
/// so point-only contact never supplies the ownership edge. This follows
/// Yap, "Towards Exact Geometric Computation,"
/// *Computational Geometry* 7.1-2 (1997), with the fragment
/// traversal matching the retained boundary idea from Weiler and Atherton,
/// "Hidden Surface Removal Using Polygon Area Sorting," *SIGGRAPH Computer
/// Graphics* 11.2 (1977). Orthogonal-cell rectangular replay follows de Berg,
/// Cheong, van Kreveld, and Overmars, *Computational Geometry: Algorithms and
/// Applications*, 3rd ed., Chapter 2.
#[cfg(feature = "exact-triangulation")]
pub fn arrange_coplanar_surface_cutter_hole_contact_difference(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Option<CoplanarSurfaceArrangement> {
    let left_components = connected_face_component_meshes(left)?;
    let right_components = connected_face_component_meshes(right)?;
    if left_components.len() != 1 || right_components.len() < 2 {
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
    let left_rect = projected_axis_aligned_rectangle(&left_component.hull, projection);

    let mut holes = Vec::new();
    let mut clipped_cutters = Vec::new();
    let mut removed_candidates = Vec::new();
    for (right_index, component) in right_components.iter().enumerate() {
        if polygon_strictly_inside_convex_polygon(
            &component.hull,
            &left_component.hull,
            projection,
        )? {
            let mut hole = component.hull.clone();
            orient_polygon_ccw(&mut hole, projection)?;
            removed_candidates.push(RemovedRegionCandidate {
                right_index,
                is_cutter: false,
                region: hole.clone(),
            });
            holes.push(hole);
        } else if convex_union_component_relation(
            &left_component.hull,
            &component.hull,
            projection,
        )? == ConvexUnionComponentRelation::PositiveArea
        {
            let mut clipped_cutter = convex_polygon_intersection_boundary(
                &component.hull,
                &left_component.hull,
                projection,
            )?;
            if clipped_cutter.len() < 3 {
                return None;
            }
            orient_polygon_ccw(&mut clipped_cutter, projection)?;
            removed_candidates.push(RemovedRegionCandidate {
                right_index,
                is_cutter: true,
                region: clipped_cutter.clone(),
            });
            clipped_cutters.push(clipped_cutter);
        } else {
            return None;
        }
    }
    if holes.is_empty() || clipped_cutters.is_empty() {
        return None;
    }

    let removed_regions = removed_candidates
        .iter()
        .map(|candidate| candidate.region.clone())
        .collect::<Vec<_>>();
    let all_removed_regions_are_rectangles = removed_regions
        .iter()
        .all(|region| projected_axis_aligned_rectangle(region, projection).is_some());
    let multi_opening_polygon = if all_removed_regions_are_rectangles {
        None
    } else {
        materialize_multi_cutter_hole_opening_difference(
            &left_component.hull,
            &removed_candidates,
            projection,
        )
        .or_else(|| {
            materialize_mixed_cutter_hole_and_side_opening_difference(
                &left_component.hull,
                &removed_candidates,
                projection,
            )
        })
    };
    let mut replay_removed_area = None;
    let mut polygon = if let Some(polygon) = multi_opening_polygon {
        polygon
    } else {
        let mut removed_polygon = if holes.len() == 1 && clipped_cutters.len() == 1 {
            let hole = &holes[0];
            let clipped_cutter = &clipped_cutters[0];
            let hole_rect = projected_axis_aligned_rectangle(hole, projection);
            let clipped_cutter_rect = projected_axis_aligned_rectangle(clipped_cutter, projection);
            match (hole_rect, clipped_cutter_rect) {
                (Some(hole_rect), Some(clipped_cutter_rect)) => {
                    if rectangles_touch_on_positive_boundary(&hole_rect, &clipped_cutter_rect)? {
                        axis_aligned_rectangle_union_polygon(
                            &[clipped_cutter_rect, hole_rect],
                            projection,
                        )?
                    } else {
                        return None;
                    }
                }
                _ => two_convex_cutter_hole_removed_polygon(hole, clipped_cutter, projection)?,
            }
        } else {
            if all_removed_regions_are_rectangles {
                return None;
            }
            connected_convex_contact_union_polygon_allowing_incidental_point_touches(
                &removed_regions,
                projection,
            )?
        };
        replay_removed_area = Some(projected_area2_abs(&removed_polygon, projection)?);
        orient_polygon_ccw(&mut removed_polygon, projection)?;
        if let Some(left_rect) = &left_rect {
            side_opened_difference_polygon(left_rect, &removed_polygon, projection)?
        } else {
            convex_side_opened_difference_polygon(
                &left_component.hull,
                &removed_polygon,
                projection,
            )?
        }
    };
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

    if let Some(removed_area) = replay_removed_area {
        let left_area = projected_area2_abs(&left_component.hull, projection)?;
        let output_area = projected_area2_abs(&polygon, projection)?;
        if compare_reals(&add(&output_area, &removed_area), &left_area).value()
            != Some(Ordering::Equal)
        {
            return None;
        }
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
/// explicit planar-arrangement work unless the orthogonal-cell replay proves a
/// set of no-hole simple loops. This is Yap's retained-computation model
/// applied to bounded Weiler-Atherton and orthogonal-cell traversals: each
/// promoted loop is produced by an already audited exact arrangement fragment,
/// not by a sampled polygon clip. See Yap, "Towards Exact Geometric
/// Computation," *Computational Geometry* 7.1-2 (1997), Weiler and Atherton,
/// "Hidden Surface Removal Using Polygon Area Sorting," *SIGGRAPH Computer
/// Graphics* 11.2 (1977), and de Berg, Cheong, van Kreveld, and Overmars,
/// *Computational Geometry: Algorithms and Applications*, 3rd ed. (2008),
/// Chapter 2.
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
    if let Some(rectangle_remnants) = materialize_rectangle_multi_cutter_no_hole_cell_difference(
        component,
        cutter_indices,
        right_components,
    ) {
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

/// Replay rectangular multi-cutter remnants through exact orthogonal cells.
///
/// This is a bounded bridge from the general rectangular cell materializer back
/// to the simple-loop surface artifact. The cell arrangement may produce
/// holes, but [`CoplanarSurfaceMultiArrangement`] cannot retain ring topology;
/// those cases are rejected here and left to
/// [`crate::exact::orthogonal_surface`]. Accepting only hole-free components
/// keeps the output model honest while still materializing nonconvex
/// partial-height multi-cutter remnants from exact grid occupancy. This is the
/// orthogonal arrangement model of de Berg, Cheong, van Kreveld, and Overmars,
/// *Computational Geometry: Algorithms and Applications*, 3rd ed. (2008),
/// Chapter 2, constrained by Yap's retained exact-object rule from "Towards
/// Exact Geometric Computation," *Computational Geometry* 7.1-2 (1997).
#[cfg(feature = "exact-triangulation")]
fn materialize_rectangle_multi_cutter_no_hole_cell_difference(
    component: &ConvexUnionComponent,
    cutter_indices: &[usize],
    right_components: &[ConvexUnionComponent],
) -> Option<Vec<Vec<Point3>>> {
    projected_axis_aligned_rectangle(&component.hull, component.projection)?;
    if !cutter_indices.iter().all(|&index| {
        projected_axis_aligned_rectangle(&right_components[index].hull, component.projection)
            .is_some()
    }) {
        return None;
    }

    let cutters = merge_component_meshes(
        cutter_indices
            .iter()
            .map(|&index| &right_components[index].mesh),
        "exact coplanar rectangular multi-cutter source",
    )?;
    let arrangement = super::orthogonal_surface::arrange_coplanar_orthogonal_surface_difference(
        &component.mesh,
        &cutters,
    )?;
    if arrangement
        .components
        .iter()
        .any(|component| !component.holes.is_empty())
    {
        return None;
    }
    let polygons = arrangement
        .components
        .into_iter()
        .map(|component| component.outer)
        .collect::<Vec<_>>();
    if polygons.is_empty() {
        None
    } else {
        Some(polygons)
    }
}

#[cfg(feature = "exact-triangulation")]
fn merge_component_meshes<'a>(
    meshes: impl IntoIterator<Item = &'a ExactMesh>,
    label: &'static str,
) -> Option<ExactMesh> {
    let mut vertices = Vec::new();
    let mut triangles = Vec::new();
    for mesh in meshes {
        let offset = vertices.len();
        vertices.extend(mesh.vertices().iter().cloned());
        triangles.extend(
            mesh.triangles()
                .iter()
                .map(|triangle| Triangle(triangle.0.map(|index| index + offset))),
        );
    }
    if triangles.is_empty() {
        return None;
    }
    ExactMesh::new_with_policy(
        vertices,
        triangles,
        SourceProvenance::exact(label),
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .ok()
}

/// Split a source surface into one retained convex mesh per triangle.
///
/// Most coplanar surface shortcuts consume connected convex components. A
/// connected cutter graph, however, may be intentionally nonconvex while each
/// input triangle is still an exact convex source object. The nonconvex
/// multi-difference path uses this fallback only after connected-component
/// convex import fails, and every resulting triangle piece is still replayed
/// through the same exact contact, fragment, and area certificates before it
/// can affect topology. This is Yap's retained-object rule from "Towards
/// Exact Geometric Computation," *Computational Geometry* 7.1-2 (1997): the
/// fallback preserves concrete source pieces instead of flattening the cutter
/// graph into an approximate polygon.
#[cfg(feature = "exact-triangulation")]
fn triangle_piece_component_meshes(mesh: &ExactMesh) -> Option<Vec<ExactMesh>> {
    if mesh.triangles().is_empty() {
        return None;
    }
    let mut pieces = Vec::with_capacity(mesh.triangles().len());
    for triangle in mesh.triangles() {
        let vertices = triangle
            .0
            .iter()
            .map(|&index| mesh.vertices().get(index).cloned())
            .collect::<Option<Vec<_>>>()?;
        let piece = ExactMesh::new_with_policy(
            vertices,
            vec![Triangle([0, 1, 2])],
            SourceProvenance::exact("exact coplanar triangle cutter piece"),
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .ok()?;
        pieces.push(piece);
    }
    Some(pieces)
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

/// Stitch the exact removed region for one convex cutter/hole pair.
///
/// This is the non-rectangular counterpart to
/// [`axis_aligned_rectangle_union_polygon`]. The two convex components must
/// either have positive-length boundary contact or positive-area overlap. The
/// overlap case is the bounded straddling-hole promotion: the retained hole is
/// not preserved as a hole because the side cutter removes part of it, but the
/// exact union of the two removed convex regions can still be replayed as one
/// simple boundary. The union boundary is assembled from exact fragments whose
/// midpoints lie outside the opposite convex component, in the Weiler-Atherton
/// boundary-traversal sense, while every predicate decision is exact as
/// required by Yap, "Towards Exact Geometric Computation," *Computational
/// Geometry* 7.1-2 (1997). Segment contact is classified with the
/// orientation-predicate model of Guigue and Devillers, "Fast and Robust
/// Triangle-Triangle Overlap Test Using Orientation Predicates," *Journal of
/// Graphics Tools* 8.1 (2003).
#[cfg(feature = "exact-triangulation")]
fn two_convex_cutter_hole_removed_polygon(
    hole: &[Point3],
    clipped_cutter: &[Point3],
    projection: CoplanarProjection,
) -> Option<Vec<Point3>> {
    match convex_union_component_relation(hole, clipped_cutter, projection)? {
        ConvexUnionComponentRelation::BoundaryOnly => {
            if !convex_polygons_touch_on_positive_boundary(hole, clipped_cutter, projection)? {
                return None;
            }
        }
        ConvexUnionComponentRelation::PositiveArea => {}
        ConvexUnionComponentRelation::Disjoint => return None,
    }

    let mut hole = hole.to_vec();
    let mut clipped_cutter = clipped_cutter.to_vec();
    orient_polygon_ccw(&mut hole, projection)?;
    orient_polygon_ccw(&mut clipped_cutter, projection)?;

    let mut fragments = Vec::new();
    collect_convex_union_boundary_fragments(&hole, &clipped_cutter, projection, &mut fragments)?;
    collect_convex_union_boundary_fragments(&clipped_cutter, &hole, projection, &mut fragments)?;
    let mut polygon = stitch_simple_loop(fragments, projection)?;
    orient_polygon_ccw(&mut polygon, projection)?;
    polygon = simplify_projected_polygon(polygon, projection);
    validate_projected_simple_loop(
        &polygon,
        projection,
        "coplanar cutter-hole removed convex union",
    )
    .ok()?;
    if !convex_union_boundary_area_matches_inputs(&polygon, &hole, &clipped_cutter, projection)? {
        return None;
    }
    Some(polygon)
}

/// Stitch one connected convex contact chain into a removed-region loop.
///
/// This is the multi-component sibling of
/// [`two_convex_cutter_hole_removed_polygon`]. It accepts only a
/// connected interaction graph whose convex regions either touch through
/// positive-length boundary intervals or overlap in positive area. Point-only
/// contact, disconnected holes, high-order graphs beyond the retained
/// inclusion-exclusion cap, and branch structures that do not stitch into
/// exactly one simple loop remain unsupported. The acceptance rule is the same
/// exact-state rule Yap gives in "Towards Exact Geometric Computation,"
/// *Computational Geometry* 7.1-2 (1997): the shortcut promotes topology only
/// from retained predicates and exact area replay. Boundary fragments follow
/// Weiler and Atherton, "Hidden Surface Removal Using Polygon Area Sorting,"
/// *SIGGRAPH Computer Graphics* 11.2 (1977), segment contact decisions use
/// Guigue and Devillers, "Fast and Robust Triangle-Triangle Overlap Test Using
/// Orientation Predicates," *Journal of Graphics Tools* 8.1 (2003), and the
/// bounded overlap area replay is an exact finite inclusion-exclusion
/// certificate over retained convex intersections.
#[cfg(feature = "exact-triangulation")]
fn connected_convex_contact_union_polygon(
    regions: &[Vec<Point3>],
    projection: CoplanarProjection,
) -> Option<Vec<Point3>> {
    connected_convex_union_polygon_with_contact_policy(regions, projection, true)
}

/// Materialize one connected face-cell union while allowing incidental
/// point-only contacts between non-neighbor clips.
///
/// Pairwise triangle clips from a valid triangulated sheet may meet at shared
/// vertices even when the actual cell adjacency travels through other clips.
/// Those point contacts are not branch decisions for this bounded
/// intersection materializer, because the promoted component still requires a
/// positive-length connected contact graph, a simple stitched boundary, and an
/// exact finite union-area replay. This is the same Yap retained-object rule
/// used by [`connected_convex_contact_union_polygon`], but with the weaker
/// local contact policy needed for face-cell triangulations.
#[cfg(feature = "exact-triangulation")]
fn connected_convex_face_cell_union_polygon(
    regions: &[Vec<Point3>],
    projection: CoplanarProjection,
) -> Option<Vec<Point3>> {
    connected_convex_contact_union_polygon_allowing_incidental_point_touches(regions, projection)
}

/// Stitch a positive-connected convex union with incidental point contacts.
///
/// The positive-area/positive-length graph must still connect every region;
/// point-only contacts are ignored by the connectivity graph and are allowed
/// only when the final stitched boundary is one simple loop with exact area
/// replay. This is useful for removed cutter/hole groups where one point
/// coincidence is covered by another positive overlap, and follows Yap,
/// "Towards Exact Geometric Computation," *Computational Geometry* 7.1-2
/// (1997), by making the accepted topology depend on replayed object facts
/// instead of a representative sample.
#[cfg(feature = "exact-triangulation")]
fn connected_convex_contact_union_polygon_allowing_incidental_point_touches(
    regions: &[Vec<Point3>],
    projection: CoplanarProjection,
) -> Option<Vec<Point3>> {
    connected_convex_union_polygon_with_contact_policy(regions, projection, false)
}

#[cfg(feature = "exact-triangulation")]
fn connected_convex_union_polygon_with_contact_policy(
    regions: &[Vec<Point3>],
    projection: CoplanarProjection,
    reject_point_only_boundary: bool,
) -> Option<Vec<Point3>> {
    if regions.len() < 2 {
        return None;
    }
    let mut regions = regions.to_vec();
    for region in &mut regions {
        orient_polygon_ccw(region, projection)?;
        validate_projected_strictly_convex_loop(
            region,
            projection,
            "coplanar cutter-hole removed contact chain",
        )
        .ok()?;
    }

    let mut contact_graph = UnionFind::new(regions.len());
    for left in 0..regions.len() {
        for right in left + 1..regions.len() {
            match convex_union_component_relation(&regions[left], &regions[right], projection)? {
                ConvexUnionComponentRelation::Disjoint => {}
                ConvexUnionComponentRelation::BoundaryOnly => {
                    let positive_boundary = convex_polygons_touch_on_positive_boundary(
                        &regions[left],
                        &regions[right],
                        projection,
                    );
                    if positive_boundary == Some(true) {
                        contact_graph.union(left, right);
                    } else if reject_point_only_boundary || positive_boundary.is_none() {
                        return None;
                    }
                }
                ConvexUnionComponentRelation::PositiveArea => contact_graph.union(left, right),
            }
        }
    }
    let root = contact_graph.find(0);
    for index in 1..regions.len() {
        if contact_graph.find(index) != root {
            return None;
        }
    }

    let mut fragments = Vec::new();
    for index in 0..regions.len() {
        collect_multi_convex_union_boundary_fragments(index, &regions, projection, &mut fragments)?;
    }
    let mut polygon = stitch_simple_loop(fragments, projection)?;
    orient_polygon_ccw(&mut polygon, projection)?;
    polygon = simplify_projected_polygon(polygon, projection);
    validate_projected_simple_loop(
        &polygon,
        projection,
        "coplanar cutter-hole removed contact chain",
    )
    .ok()?;
    if !multi_convex_contact_union_area_matches_inputs(&polygon, &regions, projection)? {
        return None;
    }
    Some(polygon)
}

/// Materialize several independent cutter/hole openings as one simple loop.
///
/// Each connected removed-region group must contain at least one side cutter
/// and one strict hole, and each group is first replayed as an exact union of
/// retained convex regions. The groups are then subtracted from the convex
/// outer sheet by retained boundary fragments, accepting only the case where
/// every group opens through one positive-length outer-edge attachment and the
/// final boundary stitches into one simple loop. This is a bounded
/// Weiler-Atherton-style fragment construction; see Weiler and Atherton,
/// "Hidden Surface Removal Using Polygon Area Sorting," *SIGGRAPH Computer
/// Graphics* 11.2 (1977). Yap, "Towards Exact Geometric Computation,"
/// *Computational Geometry* 7.1-2 (1997), is the reason this helper requires
/// exact contact groups and exact area replay instead of sampling a point in
/// each prospective bay.
#[cfg(feature = "exact-triangulation")]
fn materialize_multi_cutter_hole_opening_difference(
    outer: &[Point3],
    removed_regions: &[RemovedRegionCandidate],
    projection: CoplanarProjection,
) -> Option<Vec<Point3>> {
    let groups =
        removed_region_contact_groups_allowing_incidental_points(removed_regions, projection)?;
    if groups.len() < 2 {
        return None;
    }

    let mut removed_openings = Vec::with_capacity(groups.len());
    for group in &groups {
        if !group.iter().any(|&index| removed_regions[index].is_cutter)
            || !group.iter().any(|&index| !removed_regions[index].is_cutter)
        {
            return None;
        }
        removed_openings.push(
            materialize_removed_region_group_polygon_allowing_incidental_points(
                removed_regions,
                group,
                projection,
            )?,
        );
    }

    multi_side_opened_difference_polygon(
        outer,
        &removed_openings,
        projection,
        "coplanar multi-opening cutter-hole difference",
    )
}

/// Materialize mixed cutter/hole and cutter-only side openings as one loop.
///
/// This is the no-retained-hole counterpart to
/// [`materialize_cutter_hole_contact_component_holed_difference`]. At least
/// one connected removed-region group must contain both a side cutter and a
/// strict hole, so a straddling hole is consumed by exact contact topology;
/// additional connected groups may be cutter-only side openings. Hole-only
/// groups are rejected because they would be real holes, not side openings.
///
/// The promotion is intentionally narrow. Each connected group is first
/// replayed as a simple removed loop, each removed loop must have one
/// positive-length attachment to the convex outer boundary, and
/// [`multi_side_opened_difference_polygon`] checks the final exact area
/// equation. This follows Yap, "Towards Exact Geometric Computation,"
/// *Computational Geometry* 7.1-2 (1997): the hole is omitted only when the
/// exact retained contact graph names the removed owner, not when a sampled
/// point happens to lie in a bay. The boundary stitching is the same retained
/// fragment construction as Weiler and Atherton, "Hidden Surface Removal Using
/// Polygon Area Sorting," *SIGGRAPH Computer Graphics* 11.2 (1977).
#[cfg(feature = "exact-triangulation")]
fn materialize_mixed_cutter_hole_and_side_opening_difference(
    outer: &[Point3],
    removed_regions: &[RemovedRegionCandidate],
    projection: CoplanarProjection,
) -> Option<Vec<Point3>> {
    let groups =
        removed_region_contact_groups_allowing_incidental_points(removed_regions, projection)?;
    if groups.len() < 2 {
        return None;
    }

    let mut saw_mixed_group = false;
    let mut removed_openings = Vec::with_capacity(groups.len());
    for group in &groups {
        let has_cutter = group.iter().any(|&index| removed_regions[index].is_cutter);
        let has_hole = group.iter().any(|&index| !removed_regions[index].is_cutter);
        if !has_cutter {
            return None;
        }
        if has_hole {
            saw_mixed_group = true;
            removed_openings.push(
                materialize_removed_region_group_polygon_allowing_incidental_points(
                    removed_regions,
                    group,
                    projection,
                )?,
            );
        } else {
            removed_openings.push(materialize_removed_region_group_or_single_polygon(
                removed_regions,
                group,
                projection,
            )?);
        }
    }
    if !saw_mixed_group {
        return None;
    }

    multi_side_opened_difference_polygon(
        outer,
        &removed_openings,
        projection,
        "coplanar mixed cutter-hole and side-opening difference",
    )
}

/// Build removed-region contact groups while allowing incidental point touches.
///
/// Cutter/hole-contact differences sometimes produce a retained removed
/// region that is already connected by positive-area or positive-length
/// contacts, while two non-neighbor convex pieces also meet at an exact point.
/// That point is not allowed to provide connectivity: it is accepted only when
/// both incident regions are already in the same positive-connected group and
/// the later retained-fragment stitch plus exact area replay still proves one
/// simple removed boundary. This is the same object-level gate Yap requires in
/// "Towards Exact Geometric Computation," *Computational Geometry* 7.1-2
/// (1997): point coincidences may be retained as facts, but they cannot invent
/// topology.
#[cfg(feature = "exact-triangulation")]
fn removed_region_contact_groups_allowing_incidental_points(
    regions: &[RemovedRegionCandidate],
    projection: CoplanarProjection,
) -> Option<Vec<Vec<usize>>> {
    removed_region_contact_groups_with_policy(regions, projection, true)
}

/// Group removed regions while retaining exact point-only branch contacts.
///
/// Positive-area and positive-length contacts still create ordinary connected
/// removed groups. Point-only contacts are reported separately instead of
/// creating connectivity. This is the predicate split required by Yap's exact
/// computation model: a vertex contact may authorize a named branch artifact
/// only after the positive-dimensional groups and final retained loops replay
/// exactly.
#[cfg(feature = "exact-triangulation")]
fn removed_region_contact_groups_allowing_branch_points(
    regions: &[RemovedRegionCandidate],
    projection: CoplanarProjection,
) -> Option<(Vec<Vec<usize>>, bool)> {
    if regions.is_empty() {
        return None;
    }
    let mut contact_graph = UnionFind::new(regions.len());
    let mut saw_point_contact = false;
    for left in 0..regions.len() {
        for right in left + 1..regions.len() {
            let relation = convex_union_component_relation(
                &regions[left].region,
                &regions[right].region,
                projection,
            )?;
            match relation {
                ConvexUnionComponentRelation::Disjoint => {
                    if polygons_share_exact_vertex(&regions[left].region, &regions[right].region) {
                        saw_point_contact = true;
                    }
                }
                ConvexUnionComponentRelation::BoundaryOnly => {
                    let share_exact_vertex =
                        polygons_share_exact_vertex(&regions[left].region, &regions[right].region);
                    match convex_polygons_touch_on_positive_boundary(
                        &regions[left].region,
                        &regions[right].region,
                        projection,
                    ) {
                        Some(true) => contact_graph.union(left, right),
                        Some(false) => saw_point_contact = true,
                        None if share_exact_vertex => saw_point_contact = true,
                        None => return None,
                    }
                }
                ConvexUnionComponentRelation::PositiveArea => contact_graph.union(left, right),
            }
        }
    }

    let mut groups: Vec<(usize, Vec<usize>)> = Vec::new();
    for index in 0..regions.len() {
        let root = contact_graph.find(index);
        if let Some((_, members)) = groups.iter_mut().find(|(candidate, _)| *candidate == root) {
            members.push(index);
        } else {
            groups.push((root, vec![index]));
        }
    }
    groups.sort_by_key(|(_, members)| members.first().copied().unwrap_or(usize::MAX));
    Some((
        groups.into_iter().map(|(_, members)| members).collect(),
        saw_point_contact,
    ))
}

#[cfg(feature = "exact-triangulation")]
fn polygons_share_exact_vertex(left: &[Point3], right: &[Point3]) -> bool {
    left.iter().any(|left_point| {
        right
            .iter()
            .any(|right_point| points_equal(left_point, right_point))
    })
}

#[cfg(feature = "exact-triangulation")]
fn removed_region_contact_groups_with_policy(
    regions: &[RemovedRegionCandidate],
    projection: CoplanarProjection,
    allow_incidental_point_contacts: bool,
) -> Option<Vec<Vec<usize>>> {
    if regions.is_empty() {
        return None;
    }
    let mut contact_graph = UnionFind::new(regions.len());
    let mut point_contacts = Vec::new();
    for left in 0..regions.len() {
        for right in left + 1..regions.len() {
            match convex_union_component_relation(
                &regions[left].region,
                &regions[right].region,
                projection,
            )? {
                ConvexUnionComponentRelation::Disjoint => {}
                ConvexUnionComponentRelation::BoundaryOnly => {
                    match convex_polygons_touch_on_positive_boundary(
                        &regions[left].region,
                        &regions[right].region,
                        projection,
                    ) {
                        Some(true) => contact_graph.union(left, right),
                        Some(false) if allow_incidental_point_contacts => {
                            point_contacts.push((left, right));
                        }
                        Some(false) | None => return None,
                    }
                }
                ConvexUnionComponentRelation::PositiveArea => contact_graph.union(left, right),
            }
        }
    }
    for (left, right) in point_contacts {
        if contact_graph.find(left) != contact_graph.find(right) {
            return None;
        }
    }

    let mut groups: Vec<(usize, Vec<usize>)> = Vec::new();
    for index in 0..regions.len() {
        let root = contact_graph.find(index);
        if let Some((_, members)) = groups.iter_mut().find(|(candidate, _)| *candidate == root) {
            members.push(index);
        } else {
            groups.push((root, vec![index]));
        }
    }
    groups.sort_by_key(|(_, members)| members.first().copied().unwrap_or(usize::MAX));
    Some(groups.into_iter().map(|(_, members)| members).collect())
}

/// Replay one connected removed-region group as a simple boundary.
///
/// The group members are retained convex pieces: strict holes, clipped side
/// cutters, or both. The output loop is accepted only after the existing
/// connected convex-union fragment stitcher and finite inclusion-exclusion
/// area replay certify the group. Keeping this as a small helper lets the
/// single-opening, multi-opening, and component-holed paths share the same
/// removed-region proof obligation.
#[cfg(feature = "exact-triangulation")]
fn materialize_removed_region_group_polygon(
    regions: &[RemovedRegionCandidate],
    group: &[usize],
    projection: CoplanarProjection,
) -> Option<Vec<Point3>> {
    materialize_removed_region_group_polygon_with_policy(regions, group, projection, false)
}

#[cfg(feature = "exact-triangulation")]
fn materialize_removed_region_group_polygon_allowing_incidental_points(
    regions: &[RemovedRegionCandidate],
    group: &[usize],
    projection: CoplanarProjection,
) -> Option<Vec<Point3>> {
    materialize_removed_region_group_polygon_with_policy(regions, group, projection, true)
}

#[cfg(feature = "exact-triangulation")]
fn materialize_removed_region_group_polygon_with_policy(
    regions: &[RemovedRegionCandidate],
    group: &[usize],
    projection: CoplanarProjection,
    allow_incidental_point_contacts: bool,
) -> Option<Vec<Point3>> {
    if group.len() < 2 {
        return None;
    }
    let group_regions = group
        .iter()
        .map(|&index| regions[index].region.clone())
        .collect::<Vec<_>>();
    if allow_incidental_point_contacts {
        connected_convex_contact_union_polygon_allowing_incidental_point_touches(
            &group_regions,
            projection,
        )
        .or_else(|| {
            let all_members = (0..group_regions.len()).collect::<Vec<_>>();
            materialize_simple_polygon_union_group(
                &group_regions,
                &all_members,
                projection,
                "coplanar removed-region incidental-point union",
            )
        })
    } else {
        connected_convex_contact_union_polygon(&group_regions, projection)
    }
}

/// Materialize one removed-region group, allowing a single convex group.
///
/// Mixed cutter/hole groups still route through
/// [`connected_convex_contact_union_polygon`] so overlap and positive-length
/// contact replay exact inclusion-exclusion area. A single cutter-only group
/// is already one clipped convex removed region, but it is still reoriented
/// and loop-validated here before it participates in the multi-opening
/// difference certificate. This preserves Yap's object-level replay boundary
/// from "Towards Exact Geometric Computation," *Computational Geometry* 7.1-2
/// (1997): even the singleton shortcut is a retained exact region, not a
/// sampled polygon bay.
#[cfg(feature = "exact-triangulation")]
fn materialize_removed_region_group_or_single_polygon(
    regions: &[RemovedRegionCandidate],
    group: &[usize],
    projection: CoplanarProjection,
) -> Option<Vec<Point3>> {
    if group.len() == 1 {
        let mut polygon = regions[group[0]].region.clone();
        orient_polygon_ccw(&mut polygon, projection)?;
        polygon = simplify_projected_polygon(polygon, projection);
        validate_projected_simple_loop(&polygon, projection, "coplanar removed side-opening group")
            .ok()?;
        return Some(polygon);
    }
    materialize_removed_region_group_polygon(regions, group, projection)
}

/// Subtract several side-opened removed loops from one convex outer loop.
///
/// This is not a general planar arrangement. It only accepts removed loops
/// that are pairwise disjoint, lie inside the convex outer loop, and each share
/// exactly one retained positive-length segment with the relative interior of
/// one outer edge. The resulting boundary is stitched from exact outer
/// fragments outside every removed loop plus reversed removed-loop fragments
/// strictly inside the outer loop. The final area equation is checked exactly:
/// `area(output) + sum(area(removed_i)) == area(outer)`. That is the
/// object-level exactness boundary advocated by Yap, "Towards Exact Geometric
/// Computation," *Computational Geometry* 7.1-2 (1997).
#[cfg(feature = "exact-triangulation")]
fn multi_side_opened_difference_polygon(
    outer: &[Point3],
    removed: &[Vec<Point3>],
    projection: CoplanarProjection,
    label: &'static str,
) -> Option<Vec<Point3>> {
    if outer.len() < 3 || removed.len() < 2 {
        return None;
    }
    let mut outer = outer.to_vec();
    orient_polygon_ccw(&mut outer, projection)?;
    validate_projected_strictly_convex_loop(&outer, projection, label).ok()?;

    let mut removed = removed.to_vec();
    for polygon in &mut removed {
        orient_polygon_ccw(polygon, projection)?;
        validate_projected_simple_loop(polygon, projection, label).ok()?;
        for point in polygon.iter() {
            if convex_polygon_location(point, &outer, projection)? == ConvexPolygonLocation::Outside
            {
                return None;
            }
        }
        convex_opened_side_attachment(&outer, polygon, projection)?;
    }
    validate_simple_component_loops_disjoint(&removed, projection, label).ok()?;

    let mut fragments = Vec::new();
    collect_outer_difference_fragments(&outer, &removed, projection, &mut fragments)?;
    for index in 0..removed.len() {
        collect_removed_difference_fragments(index, &outer, &removed, projection, &mut fragments)?;
    }
    let mut polygon = stitch_simple_loop(fragments, projection)?;
    orient_polygon_ccw(&mut polygon, projection)?;
    polygon = simplify_projected_polygon(polygon, projection);
    validate_projected_simple_loop(&polygon, projection, label).ok()?;

    let outer_area = projected_area2_abs(&outer, projection)?;
    let output_area = projected_area2_abs(&polygon, projection)?;
    let mut removed_area = ExactReal::from(0);
    for removed_polygon in &removed {
        removed_area = add(
            &removed_area,
            &projected_area2_abs(removed_polygon, projection)?,
        );
    }
    if compare_reals(&add(&output_area, &removed_area), &outer_area).value()
        != Some(Ordering::Equal)
    {
        return None;
    }
    Some(polygon)
}

/// Subtract removed loops that may split one convex source into components.
///
/// This is the multi-output sibling of
/// [`multi_side_opened_difference_polygon`]. It is used when a certified
/// removed cutter/hole group touches more than one side of the same convex
/// source component, so the retained result can be several disjoint simple
/// outer loops instead of one opened loop. The input removed loops have
/// already been promoted by exact contact/union replay; this helper only
/// replays the source subtraction by retaining outer-boundary fragments
/// outside every removed loop and reversed removed-boundary fragments inside
/// the source. That is the Weiler-Atherton retained-boundary traversal from
/// Weiler and Atherton, "Hidden Surface Removal Using Polygon Area Sorting,"
/// *SIGGRAPH Computer Graphics* 11.2 (1977).
///
/// The promotion boundary follows Yap, "Towards Exact Geometric Computation,"
/// *Computational Geometry* 7.1-2 (1997): every removed loop must lie in the
/// closed convex source, own at least one positive-length source-boundary
/// attachment, stitch to simple disjoint retained loops, and satisfy the exact
/// area equation `area(source) = sum(area(output_i)) + sum(area(removed_j))`.
/// If any of those object-level facts fail, the later general planar-cell
/// materializer must carry the topology explicitly.
#[cfg(feature = "exact-triangulation")]
fn multi_side_opened_difference_polygons(
    outer: &[Point3],
    removed: &[Vec<Point3>],
    projection: CoplanarProjection,
    label: &'static str,
) -> Option<Vec<Vec<Point3>>> {
    if outer.len() < 3 || removed.is_empty() {
        return None;
    }
    let mut outer = outer.to_vec();
    orient_polygon_ccw(&mut outer, projection)?;
    validate_projected_strictly_convex_loop(&outer, projection, label).ok()?;

    let mut removed = removed.to_vec();
    for polygon in &mut removed {
        orient_polygon_ccw(polygon, projection)?;
        validate_projected_simple_loop(polygon, projection, label).ok()?;
        for point in polygon.iter() {
            if convex_polygon_location(point, &outer, projection)? == ConvexPolygonLocation::Outside
            {
                return None;
            }
        }
        if convex_boundary_attachment_count(&outer, polygon, projection)? == 0 {
            return None;
        }
    }
    validate_simple_component_loops_disjoint(&removed, projection, label).ok()?;

    let mut fragments = Vec::new();
    collect_outer_difference_fragments(&outer, &removed, projection, &mut fragments)?;
    for index in 0..removed.len() {
        collect_removed_difference_fragments(index, &outer, &removed, projection, &mut fragments)?;
    }
    let mut polygons = stitch_disjoint_simple_loops(fragments, projection)?;
    if polygons.is_empty() {
        return None;
    }
    let mut output_area = ExactReal::from(0);
    for polygon in &mut polygons {
        orient_polygon_ccw(polygon, projection)?;
        *polygon = simplify_projected_polygon(polygon.clone(), projection);
        validate_projected_simple_loop(polygon, projection, label).ok()?;
        output_area = add(&output_area, &projected_area2_abs(polygon, projection)?);
    }
    validate_simple_component_loops_disjoint(&polygons, projection, label).ok()?;

    let mut removed_area = ExactReal::from(0);
    for removed_polygon in &removed {
        removed_area = add(
            &removed_area,
            &projected_area2_abs(removed_polygon, projection)?,
        );
    }
    let outer_area = projected_area2_abs(&outer, projection)?;
    if compare_reals(&add(&output_area, &removed_area), &outer_area).value()
        != Some(Ordering::Equal)
    {
        return None;
    }
    sort_polygons_for_replay(&mut polygons, projection);
    Some(polygons)
}

#[cfg(feature = "exact-triangulation")]
fn collect_outer_difference_fragments(
    outer: &[Point3],
    removed: &[Vec<Point3>],
    projection: CoplanarProjection,
    fragments: &mut Vec<DirectedFragment>,
) -> Option<()> {
    for edge in 0..outer.len() {
        let start = &outer[edge];
        let end = &outer[(edge + 1) % outer.len()];
        let mut splits = vec![start.clone(), end.clone()];
        for removed_polygon in removed {
            for other_edge in 0..removed_polygon.len() {
                add_projected_edge_intersections(
                    start,
                    end,
                    &removed_polygon[other_edge],
                    &removed_polygon[(other_edge + 1) % removed_polygon.len()],
                    projection,
                    &mut splits,
                )?;
            }
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
            if point_outside_all_simple_polygons(&midpoint, removed, projection)? {
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
fn collect_removed_difference_fragments(
    removed_index: usize,
    outer: &[Point3],
    removed: &[Vec<Point3>],
    projection: CoplanarProjection,
    fragments: &mut Vec<DirectedFragment>,
) -> Option<()> {
    let polygon = removed.get(removed_index)?;
    for edge in 0..polygon.len() {
        let start = &polygon[edge];
        let end = &polygon[(edge + 1) % polygon.len()];
        let mut splits = vec![start.clone(), end.clone()];
        for outer_edge in 0..outer.len() {
            add_projected_edge_intersections(
                start,
                end,
                &outer[outer_edge],
                &outer[(outer_edge + 1) % outer.len()],
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
            if convex_polygon_location(&midpoint, outer, projection)?
                != ConvexPolygonLocation::Inside
            {
                continue;
            }
            if !point_outside_other_simple_polygons(&midpoint, removed_index, removed, projection)?
            {
                continue;
            }
            fragments.push(DirectedFragment {
                start: b.clone(),
                end: a.clone(),
            });
        }
    }
    Some(())
}

#[cfg(feature = "exact-triangulation")]
fn point_outside_all_simple_polygons(
    point: &Point3,
    polygons: &[Vec<Point3>],
    projection: CoplanarProjection,
) -> Option<bool> {
    for polygon in polygons {
        if simple_polygon_location(point, polygon, projection)? != ConvexPolygonLocation::Outside {
            return Some(false);
        }
    }
    Some(true)
}

#[cfg(feature = "exact-triangulation")]
fn point_outside_other_simple_polygons(
    point: &Point3,
    polygon_index: usize,
    polygons: &[Vec<Point3>],
    projection: CoplanarProjection,
) -> Option<bool> {
    for (index, polygon) in polygons.iter().enumerate() {
        if index == polygon_index {
            continue;
        }
        if simple_polygon_location(point, polygon, projection)? != ConvexPolygonLocation::Outside {
            return Some(false);
        }
    }
    Some(true)
}

#[cfg(feature = "exact-triangulation")]
fn collect_multi_convex_union_boundary_fragments(
    region_index: usize,
    regions: &[Vec<Point3>],
    projection: CoplanarProjection,
    fragments: &mut Vec<DirectedFragment>,
) -> Option<()> {
    let polygon = regions.get(region_index)?;
    for edge in 0..polygon.len() {
        let start = &polygon[edge];
        let end = &polygon[(edge + 1) % polygon.len()];
        let mut splits = vec![start.clone(), end.clone()];
        for (other_index, other) in regions.iter().enumerate() {
            if other_index == region_index {
                continue;
            }
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
            let mut exposed = true;
            for (other_index, other) in regions.iter().enumerate() {
                if other_index == region_index {
                    continue;
                }
                if convex_polygon_location(&midpoint, other, projection)?
                    != ConvexPolygonLocation::Outside
                {
                    exposed = false;
                    break;
                }
            }
            if exposed {
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
fn multi_convex_contact_union_area_matches_inputs(
    polygon: &[Point3],
    regions: &[Vec<Point3>],
    projection: CoplanarProjection,
) -> Option<bool> {
    let union_area = projected_area2_abs(polygon, projection)?;
    let expected = convex_region_union_area_inclusion_exclusion(regions, projection)?;
    Some(compare_reals(&union_area, &expected).value() == Some(Ordering::Equal))
}

/// Replay a bounded convex union area by finite inclusion-exclusion.
///
/// This helper is deliberately small rather than a general arrangement engine:
/// every nonempty subset is intersected exactly by repeated convex clipping,
/// and the alternating subset areas are compared with the stitched boundary
/// area by [`multi_convex_contact_union_area_matches_inputs`]. The cap keeps
/// the certificate replay bounded and auditable, preserving Yap's retained
/// exact-object discipline from "Towards Exact Geometric Computation,"
/// *Computational Geometry* 7.1-2 (1997).
#[cfg(feature = "exact-triangulation")]
fn convex_region_union_area_inclusion_exclusion(
    regions: &[Vec<Point3>],
    projection: CoplanarProjection,
) -> Option<ExactReal> {
    const MAX_INCLUSION_EXCLUSION_REGIONS: usize = 8;
    if regions.is_empty() || regions.len() > MAX_INCLUSION_EXCLUSION_REGIONS {
        return None;
    }

    let subset_count = 1usize.checked_shl(regions.len() as u32)?;
    let mut area = ExactReal::from(0);
    for mask in 1..subset_count {
        let subset_area = convex_region_subset_intersection_area(regions, projection, mask)?;
        if mask.count_ones() % 2 == 1 {
            area = add(&area, &subset_area);
        } else {
            area = sub(&area, &subset_area);
        }
    }
    Some(area)
}

#[cfg(feature = "exact-triangulation")]
fn convex_region_subset_intersection_area(
    regions: &[Vec<Point3>],
    projection: CoplanarProjection,
    mask: usize,
) -> Option<ExactReal> {
    let mut intersection = None::<Vec<Point3>>;
    for (index, region) in regions.iter().enumerate() {
        if mask & (1usize << index) == 0 {
            continue;
        }
        intersection = Some(if let Some(current) = intersection {
            simplify_projected_polygon(
                clip_convex_polygon(&current, region, projection).unwrap_or_default(),
                projection,
            )
        } else {
            region.clone()
        });
        if intersection.as_ref()?.len() < 3 {
            return Some(ExactReal::from(0));
        }
    }
    let intersection = intersection?;
    if intersection.len() < 3 {
        Some(ExactReal::from(0))
    } else {
        projected_area2_abs(&intersection, projection)
    }
}

#[cfg(feature = "exact-triangulation")]
fn convex_polygons_touch_on_positive_boundary(
    left: &[Point3],
    right: &[Point3],
    projection: CoplanarProjection,
) -> Option<bool> {
    for left_edge in 0..left.len() {
        let left_start = project_point(&left[left_edge], projection);
        let left_end = project_point(&left[(left_edge + 1) % left.len()], projection);
        for right_edge in 0..right.len() {
            let right_start = project_point(&right[right_edge], projection);
            let right_end = project_point(&right[(right_edge + 1) % right.len()], projection);
            match classify_segment_intersection(&left_start, &left_end, &right_start, &right_end)
                .value()?
            {
                SegmentIntersection::CollinearOverlap | SegmentIntersection::Identical => {
                    return Some(true);
                }
                SegmentIntersection::Disjoint
                | SegmentIntersection::EndpointTouch
                | SegmentIntersection::Proper => {}
            }
        }
    }
    Some(false)
}

#[cfg(feature = "exact-triangulation")]
fn vertex_point_contact_plan(
    left: &[Point3],
    right: &[Point3],
    projection: CoplanarProjection,
) -> Option<VertexPointContactPlan> {
    let mut touched = false;
    let mut left_split_points = left.to_vec();
    let mut right_split_points = right.to_vec();
    for point in left {
        match convex_polygon_location(point, right, projection)? {
            ConvexPolygonLocation::Outside => {}
            ConvexPolygonLocation::Inside => {
                return Some(VertexPointContactPlan::invalid());
            }
            ConvexPolygonLocation::Boundary => {
                right_split_points.push(point.clone());
                touched = true;
            }
        }
    }
    for point in right {
        match convex_polygon_location(point, left, projection)? {
            ConvexPolygonLocation::Outside => {}
            ConvexPolygonLocation::Inside => {
                return Some(VertexPointContactPlan::invalid());
            }
            ConvexPolygonLocation::Boundary => {
                left_split_points.push(point.clone());
                touched = true;
            }
        }
    }

    for left_edge in 0..left.len() {
        let left_start = project_point(&left[left_edge], projection);
        let left_end = project_point(&left[(left_edge + 1) % left.len()], projection);
        for right_edge in 0..right.len() {
            let right_start = project_point(&right[right_edge], projection);
            let right_end = project_point(&right[(right_edge + 1) % right.len()], projection);
            match classify_segment_intersection(&left_start, &left_end, &right_start, &right_end)
                .value()?
            {
                SegmentIntersection::Disjoint => {}
                SegmentIntersection::EndpointTouch => {
                    let left_a = &left[left_edge];
                    let left_b = &left[(left_edge + 1) % left.len()];
                    let right_a = &right[right_edge];
                    let right_b = &right[(right_edge + 1) % right.len()];
                    let mut edge_touched = false;
                    for point in [left_a, left_b] {
                        if point_on_projected_segment(right_a, right_b, point, projection) {
                            right_split_points.push(point.clone());
                            edge_touched = true;
                        }
                    }
                    for point in [right_a, right_b] {
                        if point_on_projected_segment(left_a, left_b, point, projection) {
                            left_split_points.push(point.clone());
                            edge_touched = true;
                        }
                    }
                    if !edge_touched {
                        return Some(VertexPointContactPlan::invalid());
                    }
                    touched = true;
                }
                SegmentIntersection::Proper
                | SegmentIntersection::CollinearOverlap
                | SegmentIntersection::Identical => {
                    return Some(VertexPointContactPlan::invalid());
                }
            }
        }
    }
    dedup_points(&mut left_split_points);
    dedup_points(&mut right_split_points);
    Some(if touched {
        VertexPointContactPlan {
            relation: VertexPointContactRelation::PointOnly,
            left_split_points,
            right_split_points,
        }
    } else {
        VertexPointContactPlan {
            relation: VertexPointContactRelation::Disjoint,
            left_split_points: Vec::new(),
            right_split_points: Vec::new(),
        }
    })
}

/// Plan exact point contacts between two retained simple polygon loops.
///
/// Unlike [`vertex_point_contact_plan`], this helper cannot use convex
/// half-space tests. It first classifies vertices with the same
/// FIST-backed simple-polygon location predicate used by nonconvex surface
/// validation, then checks every segment pair with exact orientation
/// predicates. Positive-area overlap, proper crossings, and collinear edge
/// contact are rejected; vertex-edge touches are returned as split points so
/// the caller can retain the branch vertex explicitly. This is the bounded
/// exact-computation discipline described by Yap, "Towards Exact Geometric
/// Computation," *Computational Geometry* 7.1-2 (1997), with segment
/// predicates following Guigue and Devillers (2003).
#[cfg(feature = "exact-triangulation")]
fn simple_vertex_point_contact_plan(
    left: &[Point3],
    right: &[Point3],
    projection: CoplanarProjection,
) -> Option<VertexPointContactPlan> {
    let mut touched = false;
    let mut left_split_points = left.to_vec();
    let mut right_split_points = right.to_vec();
    for point in left {
        match simple_polygon_location(point, right, projection)? {
            ConvexPolygonLocation::Outside => {}
            ConvexPolygonLocation::Inside => {
                return Some(VertexPointContactPlan::invalid());
            }
            ConvexPolygonLocation::Boundary => {
                right_split_points.push(point.clone());
                touched = true;
            }
        }
    }
    for point in right {
        match simple_polygon_location(point, left, projection)? {
            ConvexPolygonLocation::Outside => {}
            ConvexPolygonLocation::Inside => {
                return Some(VertexPointContactPlan::invalid());
            }
            ConvexPolygonLocation::Boundary => {
                left_split_points.push(point.clone());
                touched = true;
            }
        }
    }

    for left_edge in 0..left.len() {
        let left_start = project_point(&left[left_edge], projection);
        let left_end = project_point(&left[(left_edge + 1) % left.len()], projection);
        for right_edge in 0..right.len() {
            let right_start = project_point(&right[right_edge], projection);
            let right_end = project_point(&right[(right_edge + 1) % right.len()], projection);
            match classify_segment_intersection(&left_start, &left_end, &right_start, &right_end)
                .value()?
            {
                SegmentIntersection::Disjoint => {}
                SegmentIntersection::EndpointTouch => {
                    let left_a = &left[left_edge];
                    let left_b = &left[(left_edge + 1) % left.len()];
                    let right_a = &right[right_edge];
                    let right_b = &right[(right_edge + 1) % right.len()];
                    let mut edge_touched = false;
                    for point in [left_a, left_b] {
                        if point_on_projected_segment(right_a, right_b, point, projection) {
                            right_split_points.push(point.clone());
                            edge_touched = true;
                        }
                    }
                    for point in [right_a, right_b] {
                        if point_on_projected_segment(left_a, left_b, point, projection) {
                            left_split_points.push(point.clone());
                            edge_touched = true;
                        }
                    }
                    if !edge_touched {
                        return Some(VertexPointContactPlan::invalid());
                    }
                    touched = true;
                }
                SegmentIntersection::Proper
                | SegmentIntersection::CollinearOverlap
                | SegmentIntersection::Identical => {
                    return Some(VertexPointContactPlan::invalid());
                }
            }
        }
    }
    dedup_points(&mut left_split_points);
    dedup_points(&mut right_split_points);
    Some(if touched {
        VertexPointContactPlan {
            relation: VertexPointContactRelation::PointOnly,
            left_split_points,
            right_split_points,
        }
    } else {
        VertexPointContactPlan {
            relation: VertexPointContactRelation::Disjoint,
            left_split_points: Vec::new(),
            right_split_points: Vec::new(),
        }
    })
}

#[cfg(feature = "exact-triangulation")]
impl VertexPointContactPlan {
    fn invalid() -> Self {
        Self {
            relation: VertexPointContactRelation::InvalidBoundaryContact,
            left_split_points: Vec::new(),
            right_split_points: Vec::new(),
        }
    }
}

#[cfg(feature = "exact-triangulation")]
fn polygon_has_exact_vertex(polygon: &[Point3], point: &Point3) -> bool {
    polygon
        .iter()
        .any(|candidate| points_equal(candidate, point))
}

fn segments_share_exact_endpoint(
    left_start: &Point3,
    left_end: &Point3,
    right_start: &Point3,
    right_end: &Point3,
) -> bool {
    points_equal(left_start, right_start)
        || points_equal(left_start, right_end)
        || points_equal(left_end, right_start)
        || points_equal(left_end, right_end)
}

#[cfg(feature = "exact-triangulation")]
fn split_polygon_at_boundary_points(
    polygon: &[Point3],
    points: &[Point3],
    projection: CoplanarProjection,
) -> Option<Vec<Point3>> {
    let mut output = Vec::new();
    for edge in 0..polygon.len() {
        let start = &polygon[edge];
        let end = &polygon[(edge + 1) % polygon.len()];
        let mut splits = vec![start.clone(), end.clone()];
        for point in points {
            if point_on_projected_segment(start, end, point, projection) {
                splits.push(point.clone());
            }
        }
        sort_points_along_segment(&mut splits, start, end, projection)?;
        dedup_points(&mut splits);
        for point in splits
            .into_iter()
            .take_while(|point| !points_equal(point, end))
        {
            output.push(point);
        }
    }
    dedup_points(&mut output);
    if output.len() > 1 && points_equal(output.first()?, output.last()?) {
        output.pop();
    }
    if output.len() < 3 { None } else { Some(output) }
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

/// Open a convex outer loop by one retained removed-region attachment.
///
/// This is the non-axis-aligned counterpart to
/// [`side_opened_difference_polygon`]. The removed region must lie inside the
/// convex outer loop, share exactly one positive-length segment with the
/// relative interior of one outer edge, and carry that segment as a retained
/// boundary edge. The output walks the long outer boundary path around the
/// attachment and then the reverse removed-region boundary path. This is the
/// same retained-fragment discipline Yap requires in "Towards Exact Geometric
/// Computation," *Computational Geometry* 7.1-2 (1997): no sampled point or
/// floating tolerance decides topology. The boundary splice follows the
/// Weiler-Atherton clipping traversal idea from "Hidden Surface Removal Using
/// Polygon Area Sorting," *SIGGRAPH Computer Graphics* 11.2 (1977).
#[cfg(feature = "exact-triangulation")]
fn convex_side_opened_difference_polygon(
    outer: &[Point3],
    removed: &[Point3],
    projection: CoplanarProjection,
) -> Option<Vec<Point3>> {
    if outer.len() < 3 || removed.len() < 3 {
        return None;
    }
    for point in removed {
        if convex_polygon_location(point, outer, projection)? == ConvexPolygonLocation::Outside {
            return None;
        }
    }
    let attachment = convex_opened_side_attachment(outer, removed, projection)?;
    let edge_end = (attachment.edge + 1) % outer.len();
    let mut polygon = vec![attachment.end.clone(), outer[edge_end].clone()];
    let mut index = (edge_end + 1) % outer.len();
    while index != attachment.edge {
        polygon.push(outer[index].clone());
        index = (index + 1) % outer.len();
        if polygon.len() > outer.len() + 3 {
            return None;
        }
    }
    polygon.push(outer[attachment.edge].clone());
    polygon.push(attachment.start.clone());
    let mut removed_path =
        removed_boundary_path_reverse(removed, &attachment.start, &attachment.end)?;
    polygon.append(&mut removed_path);
    remove_duplicate_neighbors(&mut polygon);
    Some(polygon)
}

/// Exact evidence that a removed-region boundary edge opens one convex outer edge.
///
/// `edge` names the oriented outer edge, and `start..end` is the positive
/// parameter interval on that edge. Keeping these exact source vertices is the
/// retained-object boundary Yap argues for in "Towards Exact Geometric
/// Computation," *Computational Geometry* 7.1-2 (1997).
#[cfg(feature = "exact-triangulation")]
struct ConvexOpenedSideAttachment {
    edge: usize,
    start: Point3,
    end: Point3,
}

/// Find the unique retained positive-length attachment segment.
///
/// The generic convex opener deliberately rejects corner contact, multiple
/// boundary hits, and non-edge attachment so that the later topology splice is
/// a direct replay of one exact boundary fact rather than a planar arrangement
/// guess.
#[cfg(feature = "exact-triangulation")]
fn convex_opened_side_attachment(
    outer: &[Point3],
    removed: &[Point3],
    projection: CoplanarProjection,
) -> Option<ConvexOpenedSideAttachment> {
    let mut boundary_points = Vec::new();
    for point in removed {
        for edge in 0..outer.len() {
            let start = &outer[edge];
            let end = &outer[(edge + 1) % outer.len()];
            if !point_on_projected_segment(start, end, point, projection) {
                continue;
            }
            let parameter = projected_segment_parameter(start, end, point, projection)?;
            if real_order(&ExactReal::from(0), &parameter)? == Ordering::Less
                && real_order(&parameter, &ExactReal::from(1))? == Ordering::Less
            {
                boundary_points.push((edge, point.clone(), parameter));
            }
        }
    }
    if boundary_points.len() != 2 || boundary_points[0].0 != boundary_points[1].0 {
        return None;
    }
    if real_order(&boundary_points[1].2, &boundary_points[0].2)? == Ordering::Less {
        boundary_points.swap(0, 1);
    }
    let edge = boundary_points[0].0;
    let start = boundary_points[0].1.clone();
    let end = boundary_points[1].1.clone();
    if real_order(&boundary_points[0].2, &boundary_points[1].2)? != Ordering::Less {
        return None;
    }
    if !polygon_has_edge_between(removed, &start, &end) {
        return None;
    }
    Some(ConvexOpenedSideAttachment { edge, start, end })
}

/// Return whether a polygon carries the exact segment as one boundary edge.
///
/// The edge may be oriented either way because the caller has already oriented
/// the removed polygon and chooses the reverse traversal needed for the opened
/// loop.
#[cfg(feature = "exact-triangulation")]
fn polygon_has_edge_between(polygon: &[Point3], left: &Point3, right: &Point3) -> bool {
    (0..polygon.len()).any(|index| {
        let start = &polygon[index];
        let end = &polygon[(index + 1) % polygon.len()];
        (points_equal(start, left) && points_equal(end, right))
            || (points_equal(start, right) && points_equal(end, left))
    })
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
/// difference certificates, cut by boundary-attached partial-height rectangular
/// multi-cutters through exact orthogonal cells, pierced by one or more
/// strictly contained right components, or both cut and pierced when every
/// retained hole falls strictly inside one cut remnant. It also accepts
/// bounded side-attached cutter/hole contact groups, including several
/// independent openings, when retained fragments stitch one simple nonconvex
/// outer ring and unrelated strictly contained holes replay inside that opened
/// ring. Connected non-rectilinear side cutters are also accepted when their
/// exact clipped union opens one or more outer sides and all unrelated holes
/// replay strictly inside the opened ring. A point-branch component whose
/// local strict holes are all consumed by exact branch-opening ownership may
/// be emitted with empty hole lists when another source component retains a
/// real strict hole. The same carrier may expose grouped point-branch
/// straddling-hole components whose local rings were all consumed by a simple
/// removed-object replay; the final certificate still owns at least one
/// retained ring. A nonconvex left component may be
/// consumed through the bounded source-disk path when its mesh boundary
/// replays as one exact simple loop, each cutter is wholly source-owned with
/// positive-length side ownership, and strict holes are retained in exactly
/// one output loop or consumed by exactly one removed opening. Point-only
/// contacts, branch opening graphs, and overlapping multi-cutter outputs that
/// leave unassigned holes still need a full planar subdivision. This preserves
/// Yap's rule from
/// "Towards Exact Geometric Computation,"
/// *Computational Geometry* 7.1-2 (1997): every promoted loop is justified by
/// exact source topology, containment, or area replay, and unsupported
/// combinatorics remain explicit. The rectangular
/// multi-cutter/strict-hole replay is the bounded cell arrangement of de Berg,
/// Cheong, van Kreveld, and Overmars, *Computational Geometry: Algorithms and
/// Applications*, 3rd ed. (2008), Chapter 2, promoted only after retained cell
/// topology exposes simple outer rings and strict hole rings.
#[cfg(feature = "exact-triangulation")]
pub fn arrange_coplanar_convex_surface_component_holed_difference(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Option<CoplanarConvexComponentHoledArrangement> {
    let left_component_meshes = connected_face_component_meshes(left)?;
    let right_component_meshes = connected_face_component_meshes(right)?;
    if right_component_meshes.is_empty() {
        return None;
    }

    let Some(mut left_components) = left_component_meshes
        .iter()
        .cloned()
        .into_iter()
        .map(|mesh| ConvexUnionComponent::from_mesh(MultiUnionSide::Left, mesh))
        .collect::<Option<Vec<_>>>()
    else {
        return arrange_coplanar_simple_surface_component_holed_difference(
            left_component_meshes,
            right_component_meshes,
        );
    };
    let source_component_count = left_components.len();
    let Some(right_components) = right_component_meshes
        .iter()
        .cloned()
        .into_iter()
        .map(|mesh| ConvexUnionComponent::from_mesh(MultiUnionSide::Right, mesh))
        .collect::<Option<Vec<_>>>()
    else {
        return arrange_coplanar_simple_surface_component_holed_difference(
            left_component_meshes,
            right_component_meshes,
        );
    };
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
                if convex_polygons_touch_on_positive_boundary(
                    &component.hull,
                    &right_component.hull,
                    projection,
                )? {
                    cut_indices.push(right_index);
                } else if polygon_strictly_inside_convex_polygon(
                    &right_component.hull,
                    &component.hull,
                    projection,
                )? {
                    let mut ring = right_component.hull.clone();
                    orient_polygon_cw(&mut ring, projection)?;
                    holes.push(ComponentHoleCandidate { ring, right_index });
                } else {
                    return None;
                }
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
        let hole_rings = holes
            .iter()
            .map(|hole| hole.ring.clone())
            .collect::<Vec<_>>();
        if !cut_indices.is_empty() {
            emitted_cut = true;
            if let Some(opened_components) =
                materialize_cutter_hole_contact_component_holed_difference(
                    component,
                    &cut_indices,
                    &holes,
                    &right_components,
                )
            {
                components.extend(opened_components);
                continue;
            }
            if let Some(opened_components) =
                materialize_connected_multi_cutter_component_holed_difference(
                    component,
                    &cut_indices,
                    &holes,
                    &right_components,
                )
            {
                components.extend(opened_components);
                continue;
            }
            if let Some(split_components) = materialize_side_cutter_multi_component_holed_difference(
                component,
                &cut_indices,
                &holes,
                &right_components,
            ) {
                components.extend(split_components);
                continue;
            }
            if let Some(branch_components) =
                materialize_side_cutter_point_touch_component_holed_difference(
                    component,
                    &cut_indices,
                    &holes,
                    &right_components,
                )
            {
                components.extend(branch_components);
                continue;
            }
            if let Some(branch_components) =
                materialize_side_cutter_point_touch_component_holed_difference_consuming_hole_contacts(
                    component,
                    &cut_indices,
                    &holes,
                    &right_components,
                )
            {
                components.extend(branch_components);
                continue;
            }
            if let Some(branch_components) =
                materialize_side_cutter_point_touch_component_holed_difference_consuming_hole_contact_groups(
                    component,
                    &cut_indices,
                    &holes,
                    &right_components,
                )
            {
                components.extend(branch_components);
                continue;
            }
            if let Some(branch_polygons) =
                materialize_side_cutter_point_touch_difference_consuming_holes(
                    component,
                    &cut_indices,
                    &holes,
                    &right_components,
                    "coplanar component-holed source-local point-touch consumed-hole side-cutter split",
                )
            {
                components.extend(branch_polygons.into_iter().map(|outer| {
                    CoplanarConvexHoledComponent {
                        outer,
                        holes: Vec::new(),
                    }
                }));
                continue;
            }
            if let Some(branch_polygons) =
                materialize_side_cutter_point_touch_difference_consuming_hole_contacts(
                    component,
                    &cut_indices,
                    &holes,
                    &right_components,
                    "coplanar component-holed source-local point-touch straddling-hole side-cutter split",
                )
            {
                components.extend(branch_polygons.into_iter().map(|outer| {
                    CoplanarConvexHoledComponent {
                        outer,
                        holes: Vec::new(),
                    }
                }));
                continue;
            }
            if let Some(branch_polygons) =
                materialize_side_cutter_point_touch_difference_consuming_hole_contact_groups(
                    component,
                    &cut_indices,
                    &holes,
                    &right_components,
                    "coplanar component-holed source-local grouped point-touch straddling-hole side-cutter split",
                )
            {
                components.extend(branch_polygons.into_iter().map(|outer| {
                    CoplanarConvexHoledComponent {
                        outer,
                        holes: Vec::new(),
                    }
                }));
                continue;
            }
            if !component_relevant_right_regions_are_disjoint(
                &cut_indices,
                &holes,
                &right_components,
                projection,
            )? {
                return None;
            }
            if let Some(cell_components) =
                materialize_rectangle_multi_cutter_component_holed_cell_difference(
                    component,
                    &cut_indices,
                    &holes,
                    &right_components,
                )
            {
                components.extend(cell_components);
                continue;
            }
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
                assign_holes_to_cut_component_outputs(&hole_rings, &cut_polygons, projection)?;
            components.extend(
                cut_polygons
                    .into_iter()
                    .zip(holes_by_cut)
                    .map(|(outer, holes)| CoplanarConvexHoledComponent { outer, holes }),
            );
        } else {
            if !component_relevant_right_regions_are_disjoint(
                &cut_indices,
                &holes,
                &right_components,
                projection,
            )? {
                return None;
            }
            let mut outer = component.hull.clone();
            orient_polygon_ccw(&mut outer, projection)?;
            components.push(CoplanarConvexHoledComponent {
                outer,
                holes: hole_rings,
            });
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

/// Certify component-holed differences on nonconvex simple source disks.
///
/// This is the holed-output sibling of
/// [`coplanar_simple_surface_difference_polygons`]. It accepts only source
/// components whose retained mesh boundary replays as one simple disk, then
/// classifies convex right components as whole-component removals, strict
/// retained holes, or side-owned removed openings. A strict hole may be
/// retained only when exact simple-polygon containment assigns it to one
/// emitted output loop, and it may be omitted only when exactly one removed
/// opening strictly contains it or an exact removed-region contact group
/// connects it to a side-owned opening. A component whose local point-branch
/// holes are all consumed may still be emitted with empty hole lists when a
/// sibling component retains a strict hole; this includes grouped
/// straddling-hole point branches after the group replays as a simple removed
/// object. That is source-local retained topology carried by the
/// component-holed wrapper, not an empty holed certificate. Point-only contact
/// may be replayed only as incidental lower-dimensional evidence inside a
/// positive-connected removed group; point-only connectivity, ambiguous
/// ownership, non-simple branch outputs, and unsupported boundary-straddling
/// holes remain outside this certificate.
///
/// The source disk is retained object state in Yap's sense: topology is read
/// from mesh incidence and replayed by exact containment/area predicates; see
/// Yap, "Towards Exact Geometric Computation," *Computational Geometry* 7.1-2
/// (1997). The opened boundaries use the Weiler-Atherton retained-fragment
/// traversal cited by the no-hole nonconvex source path. Hole triangulation is
/// delegated to `hypertri`'s exact earcut adapter, following Held, "FIST: Fast
/// Industrial-Strength Triangulation of Polygons," *Algorithmica* 30 (2001).
#[cfg(feature = "exact-triangulation")]
fn arrange_coplanar_simple_surface_component_holed_difference(
    left_component_meshes: Vec<ExactMesh>,
    right_component_meshes: Vec<ExactMesh>,
) -> Option<CoplanarConvexComponentHoledArrangement> {
    let mut left_components = left_component_meshes
        .into_iter()
        .map(SimpleSurfaceComponent::from_mesh)
        .collect::<Option<Vec<_>>>()?;
    let right_components = right_component_meshes
        .into_iter()
        .map(|mesh| ConvexUnionComponent::from_mesh(MultiUnionSide::Right, mesh))
        .collect::<Option<Vec<_>>>()?;
    let projection = left_components.first()?.projection;
    if left_components
        .iter()
        .any(|component| component.projection != projection)
        || right_components
            .iter()
            .any(|component| component.projection != projection)
    {
        return None;
    }
    let source_boundaries = left_components
        .iter()
        .map(|component| component.boundary.clone())
        .collect::<Vec<_>>();
    validate_simple_component_loops_disjoint(
        &source_boundaries,
        projection,
        "coplanar nonconvex source component-holed arrangement",
    )
    .ok()?;
    let mut components = Vec::new();
    for component in &mut left_components {
        let mut dropped = false;
        let mut side_cutter_indices = Vec::new();
        let mut side_removed = Vec::new();
        let mut holes = Vec::new();
        for (right_index, right_component) in right_components.iter().enumerate() {
            if polygons_equal(&component.boundary, &right_component.hull)
                || polygon_in_closed_convex_polygon(
                    &component.boundary,
                    &right_component.hull,
                    projection,
                )?
            {
                if dropped || !side_removed.is_empty() || !holes.is_empty() {
                    return None;
                }
                dropped = true;
                continue;
            }
            if polygon_lies_in_closed_simple_polygon(
                &right_component.hull,
                &component.boundary,
                projection,
            )? {
                let attachment_count = simple_boundary_attachment_count(
                    &component.boundary,
                    &right_component.hull,
                    projection,
                )?;
                if attachment_count > 0 {
                    if dropped {
                        return None;
                    }
                    side_cutter_indices.push(right_index);
                    let mut cutter = right_component.hull.clone();
                    orient_polygon_ccw(&mut cutter, projection)?;
                    side_removed.push(cutter);
                } else if polygon_strictly_inside_simple_polygon(
                    &right_component.hull,
                    &component.boundary,
                    projection,
                )? {
                    let mut ring = right_component.hull.clone();
                    orient_polygon_cw(&mut ring, projection)?;
                    holes.push(ComponentHoleCandidate { ring, right_index });
                } else {
                    return None;
                }
                continue;
            }
            match simple_source_convex_region_relation(
                &component.boundary,
                &right_component.hull,
                projection,
            )? {
                SimpleSourceConvexRegionRelation::Disjoint => {}
                SimpleSourceConvexRegionRelation::BoundaryOnly => return None,
                SimpleSourceConvexRegionRelation::UnsupportedCrossing => {
                    if dropped {
                        return None;
                    }
                    let mut clipped = simple_source_convex_crossing_removed_openings(
                        component,
                        &right_component.hull,
                        "coplanar nonconvex source clipped component-holed arrangement",
                    )?;
                    side_removed.append(&mut clipped);
                }
            }
        }

        if dropped {
            continue;
        }
        let hole_rings = holes
            .iter()
            .map(|hole| hole.ring.clone())
            .collect::<Vec<_>>();
        if side_removed.is_empty() {
            let mut outer = component.boundary.clone();
            orient_polygon_ccw(&mut outer, projection)?;
            components.push(CoplanarConvexHoledComponent {
                outer,
                holes: hole_rings,
            });
        } else if let Some(opened) =
            materialize_simple_source_removed_opening_hole_contact_component_holed_difference(
                component,
                &side_removed,
                &holes,
            )
        {
            components.extend(opened);
        } else if let Some(opened) =
            materialize_simple_source_side_cutter_point_touch_component_holed_difference_consuming_hole_contacts(
                component,
                &side_removed,
                &holes,
                "coplanar nonconvex source component-holed point-touch straddling-hole side-cutter difference",
            )
        {
            components.extend(opened);
        } else if let Some(opened) =
            materialize_simple_source_side_cutter_point_touch_component_holed_difference_consuming_hole_contact_groups(
                component,
                &side_removed,
                &holes,
                "coplanar nonconvex source component-holed grouped point-touch straddling-hole side-cutter difference",
            )
        {
            components.extend(opened);
        } else if let Some(opened) =
            materialize_simple_source_side_cutter_point_touch_difference_consuming_holes(
                component,
                &side_removed,
                &holes,
                "coplanar nonconvex source component-holed source-local point-touch consumed-hole side-cutter difference",
            )
        {
            components.extend(opened.into_iter().map(|outer| CoplanarConvexHoledComponent {
                outer,
                holes: Vec::new(),
            }));
        } else if let Some(opened) =
            materialize_simple_source_side_cutter_point_touch_difference_consuming_hole_contacts(
                component,
                &side_removed,
                &holes,
                "coplanar nonconvex source component-holed source-local point-touch straddling-hole side-cutter difference",
            )
        {
            components.extend(opened.into_iter().map(|outer| CoplanarConvexHoledComponent {
                outer,
                holes: Vec::new(),
            }));
        } else if let Some(opened) =
            materialize_simple_source_side_cutter_point_touch_difference_consuming_hole_contact_groups(
                component,
                &side_removed,
                &holes,
                "coplanar nonconvex source component-holed source-local grouped point-touch straddling-hole side-cutter difference",
            )
        {
            components.extend(opened.into_iter().map(|outer| CoplanarConvexHoledComponent {
                outer,
                holes: Vec::new(),
            }));
        } else if side_removed.len() == side_cutter_indices.len() {
            if let Some(opened) =
                materialize_simple_source_cutter_hole_contact_component_holed_difference(
                    component,
                    &side_cutter_indices,
                    &holes,
                    &right_components,
                )
            {
                components.extend(opened);
            } else {
                components.extend(
                    materialize_simple_source_side_cutter_component_holed_difference(
                        component,
                        &side_removed,
                        &hole_rings,
                        "coplanar nonconvex source component-holed side-cutter difference",
                    )?,
                );
            }
        } else {
            components.extend(
                materialize_simple_source_side_cutter_component_holed_difference(
                    component,
                    &side_removed,
                    &hole_rings,
                    "coplanar nonconvex source clipped component-holed side-cutter difference",
                )?,
            );
        }
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

#[cfg(feature = "exact-triangulation")]
struct ComponentHoleCandidate {
    ring: Vec<Point3>,
    right_index: usize,
}

#[cfg(feature = "exact-triangulation")]
#[derive(Clone, Debug)]
struct RemovedRegionCandidate {
    right_index: usize,
    is_cutter: bool,
    region: Vec<Point3>,
}

#[cfg(feature = "exact-triangulation")]
#[derive(Clone, Debug)]
struct SimpleRemovedRegionCandidate {
    right_index: Option<usize>,
    is_cutter: bool,
    region: Vec<Point3>,
}

/// Replay clipped simple openings that consume strict holes on a source disk.
///
/// This is the crossing-cutter sibling of
/// [`materialize_simple_source_cutter_hole_contact_component_holed_difference`].
/// A cutter that crosses a nonconvex source is first clipped to one or more
/// source-owned simple removed openings. If a strict hole overlaps one of
/// those openings, the hole can be consumed only after the exact union of the
/// opening and hole replays as one simple removed loop. Unrelated strict holes
/// remain holes and must be assigned to exactly one retained output loop.
///
/// The helper deliberately uses simple-polygon contact and exposed-fragment
/// replay instead of convex-only contact facts, because clipped
/// `source ∩ cutter` openings need not remain convex. This is still a bounded
/// retained-object certificate, not a general arrangement solver: point-only
/// contacts do not connect components, non-simple unions reject, and the final
/// source subtraction must replay exact area. That follows Yap, "Towards
/// Exact Geometric Computation," *Computational Geometry* 7.1-2 (1997). The
/// exposed boundary construction is Weiler-Atherton retained traversal from
/// Weiler and Atherton, "Hidden Surface Removal Using Polygon Area Sorting,"
/// *SIGGRAPH Computer Graphics* 11.2 (1977), with segment relations certified
/// by the Guigue-Devillers orientation-predicate classifier.
#[cfg(feature = "exact-triangulation")]
fn materialize_simple_source_removed_opening_hole_contact_component_holed_difference(
    component: &SimpleSurfaceComponent,
    removed_openings: &[Vec<Point3>],
    holes: &[ComponentHoleCandidate],
) -> Option<Vec<CoplanarConvexHoledComponent>> {
    if removed_openings.is_empty() || holes.is_empty() {
        return None;
    }
    let projection = component.projection;
    let (removed_openings, consumed_holes) =
        materialize_simple_source_removed_opening_hole_contact_openings(
            component,
            removed_openings,
            holes,
            "coplanar nonconvex source clipped cutter-hole contact",
        )?;
    let (removed_openings, mut cut_polygons) =
        materialize_simple_source_side_cutter_difference_core(component, &removed_openings)?;
    for polygon in &mut cut_polygons {
        orient_polygon_ccw(polygon, projection)?;
    }
    if cut_polygons.len() > 1 {
        certify_simple_removed_openings_split_source_component(
            &component.boundary,
            &removed_openings,
            projection,
        )?;
    }
    let retained_holes = holes
        .iter()
        .filter(|hole| !consumed_holes.contains(&hole.right_index))
        .map(|hole| hole.ring.clone())
        .collect::<Vec<_>>();
    let holes_by_cut = assign_holes_to_side_cutter_split_outputs(
        &retained_holes,
        &cut_polygons,
        &removed_openings,
        projection,
    )?;
    Some(
        cut_polygons
            .into_iter()
            .zip(holes_by_cut)
            .map(|(outer, holes)| CoplanarConvexHoledComponent { outer, holes })
            .collect(),
    )
}

/// Replay nonconvex simple-source side cutters while retaining strict holes.
///
/// This helper is the component-holed counterpart to
/// [`materialize_simple_source_side_cutter_difference_core`] and
/// [`materialize_simple_source_side_cutter_point_touch_difference_core`]. It
/// first tries the ordinary disjoint-loop replay. If exact point-only contacts
/// between removed openings split the retained source into branch components,
/// it retries with the branch-aware replay and keeps those shared vertices in
/// the emitted outer loops. In both cases holes are assigned only by exact
/// strict containment in one retained loop or consumed by exactly one removed
/// opening.
///
/// That separation is the retained-object discipline from Yap, "Towards Exact
/// Geometric Computation," *Computational Geometry* 7.1-2 (1997): point
/// branches may widen the topological carrier, but strict hole ownership still
/// has to replay from exact source facts. The boundary reconstruction is the
/// Weiler-Atherton retained-fragment traversal cited by the no-hole
/// side-cutter materializers, and hole triangulation remains delegated to the
/// exact `hypertri` earcut adapter following Held's FIST algorithm.
#[cfg(feature = "exact-triangulation")]
fn materialize_simple_source_side_cutter_component_holed_difference(
    component: &SimpleSurfaceComponent,
    side_removed: &[Vec<Point3>],
    hole_rings: &[Vec<Point3>],
    label: &'static str,
) -> Option<Vec<CoplanarConvexHoledComponent>> {
    if side_removed.is_empty() || hole_rings.is_empty() {
        return None;
    }
    let projection = component.projection;
    let (removed_openings, mut cut_polygons) = if let Some(replay) =
        materialize_simple_source_side_cutter_difference_core(component, side_removed)
    {
        replay
    } else {
        materialize_simple_source_side_cutter_point_touch_difference_core(
            component,
            side_removed,
            label,
        )?
    };
    for polygon in &mut cut_polygons {
        orient_polygon_ccw(polygon, projection)?;
    }
    let holes_by_cut = assign_holes_to_side_cutter_split_outputs(
        hole_rings,
        &cut_polygons,
        &removed_openings,
        projection,
    )?;
    Some(
        cut_polygons
            .into_iter()
            .zip(holes_by_cut)
            .map(|(outer, holes)| CoplanarConvexHoledComponent { outer, holes })
            .collect(),
    )
}

/// Replay mixed retained/consumed holes on simple-source point branches.
///
/// This is the nonconvex-source counterpart to
/// [`materialize_side_cutter_point_touch_component_holed_difference_consuming_hole_contacts`].
/// A strict hole that has positive-dimensional contact with exactly one
/// source-owned branch opening is unioned into that removed opening before
/// branch replay. Strict holes disjoint from all removed openings remain
/// retained and are assigned to the emitted branch loops. Point-only hole
/// contact, contact with multiple openings, and all-consumed cases reject so
/// the no-hole point-touch or future planar-cell artifact owns the topology.
///
/// The certificate follows Yap, "Towards Exact Geometric Computation,"
/// *Computational Geometry* 7.1-2 (1997): omitted rings and retained rings
/// are both named by exact object predicates. Removed-opening unions and the
/// final branch loops are Weiler-Atherton retained-fragment replays, and
/// contact dimensionality is decided by the Guigue-Devillers
/// orientation-predicate segment classifier used by
/// [`simple_polygon_interaction`].
#[cfg(feature = "exact-triangulation")]
fn materialize_simple_source_side_cutter_point_touch_component_holed_difference_consuming_hole_contacts(
    component: &SimpleSurfaceComponent,
    side_removed: &[Vec<Point3>],
    holes: &[ComponentHoleCandidate],
    label: &'static str,
) -> Option<Vec<CoplanarConvexHoledComponent>> {
    if side_removed.len() < 2 || holes.is_empty() {
        return None;
    }
    let projection = component.projection;
    let mut openings = Vec::with_capacity(side_removed.len());
    for opening in side_removed {
        let mut region = opening.clone();
        orient_polygon_ccw(&mut region, projection)?;
        region = simplify_projected_polygon(region, projection);
        validate_projected_simple_loop(&region, projection, label).ok()?;
        if !polygon_lies_in_closed_simple_polygon(&region, &component.boundary, projection)? {
            return None;
        }
        if simple_boundary_attachment_count(&component.boundary, &region, projection)? == 0 {
            return None;
        }
        openings.push(region);
    }

    let mut holes_by_opening = vec![Vec::<Vec<Point3>>::new(); openings.len()];
    let mut retained_holes = Vec::new();
    let mut saw_consumed_contact = false;
    for hole in holes {
        if !polygon_strictly_inside_simple_polygon(&hole.ring, &component.boundary, projection)? {
            return None;
        }
        let mut hole_region = hole.ring.clone();
        orient_polygon_ccw(&mut hole_region, projection)?;
        hole_region = simplify_projected_polygon(hole_region, projection);
        validate_projected_simple_loop(&hole_region, projection, label).ok()?;

        let mut owner = None;
        for (opening_index, opening) in openings.iter().enumerate() {
            match simple_polygon_interaction(&hole_region, opening, projection)? {
                SimplePolygonInteraction::Disjoint => {}
                SimplePolygonInteraction::PointOnly => return None,
                SimplePolygonInteraction::Connected => {
                    if owner.replace(opening_index).is_some() {
                        return None;
                    }
                }
            }
        }
        if let Some(opening_index) = owner {
            saw_consumed_contact = true;
            holes_by_opening[opening_index].push(hole_region);
        } else {
            let mut retained = hole.ring.clone();
            orient_polygon_cw(&mut retained, projection)?;
            retained_holes.push(retained);
        }
    }
    if !saw_consumed_contact || retained_holes.is_empty() {
        return None;
    }

    let mut merged_openings = Vec::with_capacity(openings.len());
    for (opening, owned_holes) in openings.into_iter().zip(holes_by_opening) {
        let mut merged = if owned_holes.is_empty() {
            opening
        } else {
            let mut group_polygons = Vec::with_capacity(1 + owned_holes.len());
            group_polygons.push(opening);
            group_polygons.extend(owned_holes);
            let group = (0..group_polygons.len()).collect::<Vec<_>>();
            materialize_simple_polygon_union_group(&group_polygons, &group, projection, label)?
        };
        orient_polygon_ccw(&mut merged, projection)?;
        merged = simplify_projected_polygon(merged, projection);
        validate_projected_simple_loop(&merged, projection, label).ok()?;
        if !polygon_lies_in_closed_simple_polygon(&merged, &component.boundary, projection)? {
            return None;
        }
        if simple_boundary_attachment_count(&component.boundary, &merged, projection)? == 0 {
            return None;
        }
        merged_openings.push(merged);
    }

    let (removed_openings, mut cut_polygons) =
        materialize_simple_source_side_cutter_point_touch_difference_core(
            component,
            &merged_openings,
            label,
        )?;
    for polygon in &mut cut_polygons {
        orient_polygon_ccw(polygon, projection)?;
    }
    let holes_by_cut = assign_holes_to_side_cutter_split_outputs(
        &retained_holes,
        &cut_polygons,
        &removed_openings,
        projection,
    )?;
    if holes_by_cut.iter().all(Vec::is_empty) {
        return None;
    }
    Some(
        cut_polygons
            .into_iter()
            .zip(holes_by_cut)
            .map(|(outer, holes)| CoplanarConvexHoledComponent { outer, holes })
            .collect(),
    )
}

/// Replay grouped branch/hole objects on simple nonconvex sources.
///
/// This is the simple-source counterpart to
/// [`materialize_side_cutter_point_touch_component_holed_difference_consuming_hole_contact_groups`].
/// It handles the bounded case where a strict hole has positive-dimensional
/// contact with several source-owned removed openings, those openings plus
/// the hole replay as one simple removed object, and remaining opening groups
/// meet only at exact branch vertices. When `retain_disjoint_holes` is true,
/// strict holes disjoint from every removed group are assigned to the emitted
/// output loops; otherwise every hole must be consumed.
///
/// The policy is Yap's retained-object model from "Towards Exact Geometric
/// Computation," *Computational Geometry* 7.1-2 (1997): a deleted ring is
/// accepted only after exact predicates name the removed object that owns it,
/// while retained rings remain explicit output objects. Grouped removed
/// objects are built by the Weiler-Atherton retained-fragment construction
/// cited by [`materialize_simple_polygon_union_group`], and contact
/// dimensionality is certified by the Guigue-Devillers orientation-predicate
/// classifier exposed through [`simple_polygon_interaction`].
#[cfg(feature = "exact-triangulation")]
fn materialize_simple_source_side_cutter_point_touch_grouped_hole_replay(
    component: &SimpleSurfaceComponent,
    side_removed: &[Vec<Point3>],
    holes: &[ComponentHoleCandidate],
    retain_disjoint_holes: bool,
    label: &'static str,
) -> Option<(Vec<Vec<Point3>>, Vec<Vec<Point3>>, Vec<Vec<Point3>>)> {
    if side_removed.len() < 2 || holes.is_empty() {
        return None;
    }
    let projection = component.projection;
    let mut regions = Vec::with_capacity(side_removed.len() + holes.len());
    for opening in side_removed {
        let mut region = opening.clone();
        orient_polygon_ccw(&mut region, projection)?;
        region = simplify_projected_polygon(region, projection);
        validate_projected_simple_loop(&region, projection, label).ok()?;
        if !polygon_lies_in_closed_simple_polygon(&region, &component.boundary, projection)? {
            return None;
        }
        if simple_boundary_attachment_count(&component.boundary, &region, projection)? == 0 {
            return None;
        }
        regions.push(SimpleRemovedRegionCandidate {
            right_index: None,
            is_cutter: true,
            region,
        });
    }
    for (hole_index, hole) in holes.iter().enumerate() {
        if !polygon_strictly_inside_simple_polygon(&hole.ring, &component.boundary, projection)? {
            return None;
        }
        let mut region = hole.ring.clone();
        orient_polygon_ccw(&mut region, projection)?;
        region = simplify_projected_polygon(region, projection);
        validate_projected_simple_loop(&region, projection, label).ok()?;
        regions.push(SimpleRemovedRegionCandidate {
            right_index: Some(hole_index),
            is_cutter: false,
            region,
        });
    }

    let mut contact_graph = UnionFind::new(regions.len());
    for left in 0..regions.len() {
        for right in left + 1..regions.len() {
            match simple_polygon_interaction(
                &regions[left].region,
                &regions[right].region,
                projection,
            )? {
                SimplePolygonInteraction::Disjoint => {}
                SimplePolygonInteraction::PointOnly => {
                    if !regions[left].is_cutter || !regions[right].is_cutter {
                        return None;
                    }
                }
                SimplePolygonInteraction::Connected => contact_graph.union(left, right),
            }
        }
    }

    let mut groups: Vec<(usize, Vec<usize>)> = Vec::new();
    for index in 0..regions.len() {
        let root = contact_graph.find(index);
        if let Some((_, members)) = groups.iter_mut().find(|(candidate, _)| *candidate == root) {
            members.push(index);
        } else {
            groups.push((root, vec![index]));
        }
    }
    groups.sort_by_key(|(_, members)| members.first().copied().unwrap_or(usize::MAX));

    let mut retained_holes = Vec::new();
    let mut removed_openings = Vec::new();
    let mut consumed_holes = Vec::new();
    let mut saw_multi_opening_consumed_group = false;
    for (_, group) in groups {
        let cutter_count = group
            .iter()
            .filter(|&&index| regions[index].is_cutter)
            .count();
        let hole_count = group
            .iter()
            .filter(|&&index| !regions[index].is_cutter)
            .count();
        if cutter_count == 0 {
            if !retain_disjoint_holes || hole_count != 1 || group.len() != 1 {
                return None;
            }
            let mut retained = regions[group[0]].region.clone();
            orient_polygon_cw(&mut retained, projection)?;
            retained_holes.push(retained);
            continue;
        }
        if hole_count > 0 {
            consumed_holes.extend(group.iter().filter_map(|&index| regions[index].right_index));
            if cutter_count > 1 {
                saw_multi_opening_consumed_group = true;
            }
        }

        let mut opening = if group.len() == 1 {
            regions[group[0]].region.clone()
        } else {
            let polygons = group
                .iter()
                .map(|&index| regions[index].region.clone())
                .collect::<Vec<_>>();
            let all = (0..polygons.len()).collect::<Vec<_>>();
            materialize_simple_polygon_union_group(&polygons, &all, projection, label)?
        };
        orient_polygon_ccw(&mut opening, projection)?;
        opening = simplify_projected_polygon(opening, projection);
        validate_projected_simple_loop(&opening, projection, label).ok()?;
        if !polygon_lies_in_closed_simple_polygon(&opening, &component.boundary, projection)? {
            return None;
        }
        if simple_boundary_attachment_count(&component.boundary, &opening, projection)? == 0 {
            return None;
        }
        removed_openings.push(opening);
    }
    if !saw_multi_opening_consumed_group {
        return None;
    }
    if retain_disjoint_holes {
        if retained_holes.is_empty() {
            return None;
        }
    } else if holes
        .iter()
        .enumerate()
        .any(|(index, _)| !consumed_holes.contains(&index))
    {
        return None;
    }

    let (removed_openings, mut cut_polygons) =
        materialize_simple_source_side_cutter_point_touch_difference_core(
            component,
            &removed_openings,
            label,
        )?;
    for polygon in &mut cut_polygons {
        orient_polygon_ccw(polygon, projection)?;
        validate_projected_simple_loop(polygon, projection, label).ok()?;
    }
    validate_simple_component_loops_disjoint_allowing_vertex_point_touches(
        &cut_polygons,
        projection,
        label,
    )
    .ok()?;
    sort_polygons_for_replay(&mut cut_polygons, projection);
    Some((removed_openings, cut_polygons, retained_holes))
}

/// Replay grouped straddling-hole ownership on simple-source holed branches.
///
/// This wrapper keeps the component-holed artifact honest: disjoint strict
/// rings remain output holes and are assigned by exact containment after the
/// grouped removed openings split the source.
#[cfg(feature = "exact-triangulation")]
fn materialize_simple_source_side_cutter_point_touch_component_holed_difference_consuming_hole_contact_groups(
    component: &SimpleSurfaceComponent,
    side_removed: &[Vec<Point3>],
    holes: &[ComponentHoleCandidate],
    label: &'static str,
) -> Option<Vec<CoplanarConvexHoledComponent>> {
    let (removed_openings, cut_polygons, retained_holes) =
        materialize_simple_source_side_cutter_point_touch_grouped_hole_replay(
            component,
            side_removed,
            holes,
            true,
            label,
        )?;
    let holes_by_cut = assign_holes_to_side_cutter_split_outputs(
        &retained_holes,
        &cut_polygons,
        &removed_openings,
        component.projection,
    )?;
    if holes_by_cut.iter().all(Vec::is_empty) {
        return None;
    }
    Some(
        cut_polygons
            .into_iter()
            .zip(holes_by_cut)
            .map(|(outer, holes)| CoplanarConvexHoledComponent { outer, holes })
            .collect(),
    )
}

/// Replay nonconvex source point branches whose strict holes are all consumed.
///
/// This is the no-hole sibling of
/// [`materialize_simple_source_side_cutter_component_holed_difference`]. The
/// branch replay still owns the hard topology: point-only removed-opening
/// contacts are retained as duplicated output vertices, while positive
/// removed contacts are merged before the final retained-fragment walk. The
/// only extra permission granted here is deletion of strict source holes, and
/// that is allowed only when exact simple-polygon containment names exactly
/// one removed opening as the owner of every omitted ring.
///
/// This is the retained-object rule from Yap, "Towards Exact Geometric
/// Computation," *Computational Geometry* 7.1-2 (1997), applied to branch
/// topology: branch vertices and consumed rings are object facts carried by
/// the certificate, not sampled side effects. The boundary replay remains the
/// Weiler-Atherton traversal cited by
/// [`materialize_simple_source_side_cutter_point_touch_difference_core`].
#[cfg(feature = "exact-triangulation")]
fn materialize_simple_source_side_cutter_point_touch_difference_consuming_holes(
    component: &SimpleSurfaceComponent,
    side_removed: &[Vec<Point3>],
    holes: &[ComponentHoleCandidate],
    label: &'static str,
) -> Option<Vec<Vec<Point3>>> {
    if side_removed.len() < 2 || holes.is_empty() {
        return None;
    }
    let projection = component.projection;
    let (removed_openings, mut cut_polygons) =
        materialize_simple_source_side_cutter_point_touch_difference_core(
            component,
            side_removed,
            label,
        )?;
    for hole in holes {
        if !hole_strictly_consumed_by_one_removed_opening(
            &hole.ring,
            &removed_openings,
            projection,
        )? {
            return None;
        }
    }
    for polygon in &mut cut_polygons {
        orient_polygon_ccw(polygon, projection)?;
        validate_projected_simple_loop(polygon, projection, label).ok()?;
    }
    validate_simple_component_loops_disjoint_allowing_vertex_point_touches(
        &cut_polygons,
        projection,
        label,
    )
    .ok()?;
    sort_polygons_for_replay(&mut cut_polygons, projection);
    Some(cut_polygons)
}

/// Replay nonconvex source point branches that consume straddling holes.
///
/// This is the contact/overlap sibling of
/// [`materialize_simple_source_side_cutter_point_touch_difference_consuming_holes`].
/// A strict source hole that is not wholly inside one branch opening may still
/// be deleted when exact topology proves that it positively overlaps or
/// shares positive-length boundary with exactly one removed opening. That
/// opening and its owned holes are first replayed as one exact removed-region
/// union; only then does the branch-aware side-cutter replay stitch the final
/// retained loops. Point-only hole contact is rejected because it does not
/// name a 2D removed object, and holes touching multiple openings stay with
/// the future planar-cell extractor.
///
/// The policy is Yap's retained-object discipline from "Towards Exact
/// Geometric Computation," *Computational Geometry* 7.1-2 (1997): omitting a
/// ring requires exact object ownership, not a sampled witness. The
/// opening/hole unions and retained branch loops use the same
/// Weiler-Atherton retained-fragment replay cited by
/// [`materialize_simple_polygon_union_group`] and
/// [`materialize_simple_source_side_cutter_point_touch_difference_core`],
/// with segment contacts classified by the Guigue-Devillers
/// orientation-predicate tests used throughout this module.
#[cfg(feature = "exact-triangulation")]
fn materialize_simple_source_side_cutter_point_touch_difference_consuming_hole_contacts(
    component: &SimpleSurfaceComponent,
    side_removed: &[Vec<Point3>],
    holes: &[ComponentHoleCandidate],
    label: &'static str,
) -> Option<Vec<Vec<Point3>>> {
    if side_removed.len() < 2 || holes.is_empty() {
        return None;
    }
    let projection = component.projection;
    let mut openings = Vec::with_capacity(side_removed.len());
    for opening in side_removed {
        let mut region = opening.clone();
        orient_polygon_ccw(&mut region, projection)?;
        region = simplify_projected_polygon(region, projection);
        validate_projected_simple_loop(&region, projection, label).ok()?;
        if !polygon_lies_in_closed_simple_polygon(&region, &component.boundary, projection)? {
            return None;
        }
        if simple_boundary_attachment_count(&component.boundary, &region, projection)? == 0 {
            return None;
        }
        openings.push(region);
    }

    let mut holes_by_opening = vec![Vec::<Vec<Point3>>::new(); openings.len()];
    for hole in holes {
        if !polygon_strictly_inside_simple_polygon(&hole.ring, &component.boundary, projection)? {
            return None;
        }
        let mut hole_region = hole.ring.clone();
        orient_polygon_ccw(&mut hole_region, projection)?;
        hole_region = simplify_projected_polygon(hole_region, projection);
        validate_projected_simple_loop(&hole_region, projection, label).ok()?;

        let mut owner = None;
        for (opening_index, opening) in openings.iter().enumerate() {
            match simple_polygon_interaction(&hole_region, opening, projection)? {
                SimplePolygonInteraction::Disjoint => {}
                SimplePolygonInteraction::PointOnly => return None,
                SimplePolygonInteraction::Connected => {
                    if owner.replace(opening_index).is_some() {
                        return None;
                    }
                }
            }
        }
        holes_by_opening[owner?].push(hole_region);
    }

    let mut merged_openings = Vec::with_capacity(openings.len());
    for (opening, owned_holes) in openings.into_iter().zip(holes_by_opening) {
        let mut merged = if owned_holes.is_empty() {
            opening
        } else {
            let mut group_polygons = Vec::with_capacity(1 + owned_holes.len());
            group_polygons.push(opening);
            group_polygons.extend(owned_holes);
            let group = (0..group_polygons.len()).collect::<Vec<_>>();
            materialize_simple_polygon_union_group(&group_polygons, &group, projection, label)?
        };
        orient_polygon_ccw(&mut merged, projection)?;
        merged = simplify_projected_polygon(merged, projection);
        validate_projected_simple_loop(&merged, projection, label).ok()?;
        if !polygon_lies_in_closed_simple_polygon(&merged, &component.boundary, projection)? {
            return None;
        }
        if simple_boundary_attachment_count(&component.boundary, &merged, projection)? == 0 {
            return None;
        }
        merged_openings.push(merged);
    }

    let (_, mut cut_polygons) = materialize_simple_source_side_cutter_point_touch_difference_core(
        component,
        &merged_openings,
        label,
    )?;
    for polygon in &mut cut_polygons {
        orient_polygon_ccw(polygon, projection)?;
        validate_projected_simple_loop(polygon, projection, label).ok()?;
    }
    validate_simple_component_loops_disjoint_allowing_vertex_point_touches(
        &cut_polygons,
        projection,
        label,
    )
    .ok()?;
    sort_polygons_for_replay(&mut cut_polygons, projection);
    Some(cut_polygons)
}

/// Replay grouped straddling holes on simple-source point branches.
///
/// This is the no-hole wrapper for
/// [`materialize_simple_source_side_cutter_point_touch_grouped_hole_replay`]:
/// every strict ring must be consumed by a grouped removed object, so the
/// public artifact remains [`CoplanarSurfacePointTouchDifference`] rather than
/// an empty component-holed carrier.
#[cfg(feature = "exact-triangulation")]
fn materialize_simple_source_side_cutter_point_touch_difference_consuming_hole_contact_groups(
    component: &SimpleSurfaceComponent,
    side_removed: &[Vec<Point3>],
    holes: &[ComponentHoleCandidate],
    label: &'static str,
) -> Option<Vec<Vec<Point3>>> {
    let (_, cut_polygons, retained_holes) =
        materialize_simple_source_side_cutter_point_touch_grouped_hole_replay(
            component,
            side_removed,
            holes,
            false,
            label,
        )?;
    if !retained_holes.is_empty() {
        return None;
    }
    Some(cut_polygons)
}

/// Replay a nonconvex source difference whose strict holes are all consumed.
///
/// The public no-hole artifact must not route through a component-holed object
/// with empty hole lists. This helper shares the same clipped-opening/hole
/// contact proof as the component-holed path, then promotes only when every
/// strict hole belongs to a consumed exact union group and the retained loops
/// are disjoint simple source-owned outputs.
#[cfg(feature = "exact-triangulation")]
fn materialize_simple_source_removed_opening_hole_contact_difference_consuming_holes(
    component: &SimpleSurfaceComponent,
    removed_openings: &[Vec<Point3>],
    holes: &[ComponentHoleCandidate],
    label: &'static str,
) -> Option<Vec<Vec<Point3>>> {
    if removed_openings.is_empty() || holes.is_empty() {
        return None;
    }
    let projection = component.projection;
    let (removed_openings, consumed_holes) =
        materialize_simple_source_removed_opening_hole_contact_openings(
            component,
            removed_openings,
            holes,
            label,
        )?;
    if holes
        .iter()
        .any(|hole| !consumed_holes.contains(&hole.right_index))
    {
        return None;
    }
    let (_, mut polygons) =
        materialize_simple_source_side_cutter_difference_core(component, &removed_openings)?;
    for polygon in &mut polygons {
        orient_polygon_ccw(polygon, projection)?;
        validate_projected_simple_loop(polygon, projection, label).ok()?;
    }
    if polygons.len() > 1 {
        certify_simple_removed_openings_split_source_component(
            &component.boundary,
            &removed_openings,
            projection,
        )?;
    }
    validate_simple_component_loops_disjoint(&polygons, projection, label).ok()?;
    sort_polygons_for_replay(&mut polygons, projection);
    Some(polygons)
}

#[cfg(feature = "exact-triangulation")]
fn materialize_simple_source_removed_opening_hole_contact_openings(
    component: &SimpleSurfaceComponent,
    removed_openings: &[Vec<Point3>],
    holes: &[ComponentHoleCandidate],
    label: &'static str,
) -> Option<(Vec<Vec<Point3>>, Vec<usize>)> {
    let projection = component.projection;
    let mut regions = Vec::with_capacity(removed_openings.len() + holes.len());
    for opening in removed_openings {
        let mut region = opening.clone();
        orient_polygon_ccw(&mut region, projection)?;
        region = simplify_projected_polygon(region, projection);
        validate_projected_simple_loop(&region, projection, label).ok()?;
        if !polygon_lies_in_closed_simple_polygon(&region, &component.boundary, projection)? {
            return None;
        }
        if simple_boundary_attachment_count(&component.boundary, &region, projection)? == 0 {
            return None;
        }
        regions.push(SimpleRemovedRegionCandidate {
            right_index: None,
            is_cutter: true,
            region,
        });
    }
    for hole in holes {
        let mut region = hole.ring.clone();
        orient_polygon_ccw(&mut region, projection)?;
        region = simplify_projected_polygon(region, projection);
        validate_projected_simple_loop(&region, projection, label).ok()?;
        if !polygon_strictly_inside_simple_polygon(&hole.ring, &component.boundary, projection)? {
            return None;
        }
        regions.push(SimpleRemovedRegionCandidate {
            right_index: Some(hole.right_index),
            is_cutter: false,
            region,
        });
    }

    let groups = simple_removed_region_contact_groups(&regions, projection)?;
    let mut saw_mixed_group = false;
    let mut consumed_holes = Vec::new();
    let mut merged_openings = Vec::new();
    for group in &groups {
        let has_cutter = group.iter().any(|&index| regions[index].is_cutter);
        let has_hole = group.iter().any(|&index| !regions[index].is_cutter);
        if !has_cutter {
            continue;
        }
        let mut opening = if group.len() == 1 {
            regions[group[0]].region.clone()
        } else {
            let polygons = group
                .iter()
                .map(|&index| regions[index].region.clone())
                .collect::<Vec<_>>();
            let all = (0..polygons.len()).collect::<Vec<_>>();
            materialize_simple_polygon_union_group(&polygons, &all, projection, label)?
        };
        orient_polygon_ccw(&mut opening, projection)?;
        opening = simplify_projected_polygon(opening, projection);
        validate_projected_simple_loop(&opening, projection, label).ok()?;
        if !polygon_lies_in_closed_simple_polygon(&opening, &component.boundary, projection)? {
            return None;
        }
        if simple_boundary_attachment_count(&component.boundary, &opening, projection)? == 0 {
            return None;
        }
        if has_hole {
            saw_mixed_group = true;
            consumed_holes.extend(group.iter().filter_map(|&index| regions[index].right_index));
        }
        merged_openings.push(opening);
    }
    if !saw_mixed_group {
        return None;
    }
    Some((merged_openings, consumed_holes))
}

#[cfg(feature = "exact-triangulation")]
fn simple_removed_region_contact_groups(
    regions: &[SimpleRemovedRegionCandidate],
    projection: CoplanarProjection,
) -> Option<Vec<Vec<usize>>> {
    if regions.is_empty() {
        return None;
    }
    let mut contact_graph = UnionFind::new(regions.len());
    let mut point_contacts = Vec::new();
    for left in 0..regions.len() {
        for right in left + 1..regions.len() {
            match simple_polygon_interaction(
                &regions[left].region,
                &regions[right].region,
                projection,
            )? {
                SimplePolygonInteraction::Disjoint => {}
                SimplePolygonInteraction::PointOnly => point_contacts.push((left, right)),
                SimplePolygonInteraction::Connected => contact_graph.union(left, right),
            }
        }
    }
    for (left, right) in point_contacts {
        if contact_graph.find(left) != contact_graph.find(right) {
            return None;
        }
    }

    let mut groups: Vec<(usize, Vec<usize>)> = Vec::new();
    for index in 0..regions.len() {
        let root = contact_graph.find(index);
        if let Some((_, members)) = groups.iter_mut().find(|(candidate, _)| *candidate == root) {
            members.push(index);
        } else {
            groups.push((root, vec![index]));
        }
    }
    groups.sort_by_key(|(_, members)| members.first().copied().unwrap_or(usize::MAX));
    Some(groups.into_iter().map(|(_, members)| members).collect())
}

/// Replay cutter/hole contact groups on nonconvex simple source disks.
///
/// This is the nonconvex-source counterpart to
/// [`materialize_cutter_hole_contact_component_holed_difference`]. The source
/// boundary still comes from [`SimpleSurfaceComponent`] mesh incidence, so a
/// cutter is admitted only when its whole exact convex ring lies in the closed
/// source disk and owns positive-length source-boundary contact. Strict source
/// holes may be consumed only when the exact removed-region contact graph
/// connects them to at least one such side cutter; unrelated strict holes are
/// retained and assigned to emitted output loops after the side openings are
/// stitched.
///
/// This keeps the shortcut inside Yap's retained-object model from "Towards
/// Exact Geometric Computation," *Computational Geometry* 7.1-2 (1997):
/// consumed topology is named by exact contact groups and exact area replay,
/// never by a sampled witness point. The removed-loop and final source
/// difference boundaries are Weiler-Atherton retained-fragment traversals; see
/// Weiler and Atherton, "Hidden Surface Removal Using Polygon Area Sorting,"
/// *SIGGRAPH Computer Graphics* 11.2 (1977). Point-only connectivity remains
/// unsupported because it is a branch decision for the later planar-cell
/// extractor; incidental point contacts are admitted only inside an already
/// positive-connected removed group. Segment contact is classified by the
/// exact orientation predicates of Guigue and Devillers, "Fast and Robust
/// Triangle-Triangle Overlap Test Using Orientation Predicates," *Journal of
/// Graphics Tools* 8.1 (2003).
#[cfg(feature = "exact-triangulation")]
fn materialize_simple_source_cutter_hole_contact_component_holed_difference(
    component: &SimpleSurfaceComponent,
    cut_indices: &[usize],
    holes: &[ComponentHoleCandidate],
    right_components: &[ConvexUnionComponent],
) -> Option<Vec<CoplanarConvexHoledComponent>> {
    if cut_indices.is_empty() || holes.is_empty() {
        return None;
    }
    let projection = component.projection;
    let mut regions = Vec::with_capacity(cut_indices.len() + holes.len());
    for &right_index in cut_indices {
        let mut region = right_components.get(right_index)?.hull.clone();
        orient_polygon_ccw(&mut region, projection)?;
        validate_projected_strictly_convex_loop(
            &region,
            projection,
            "coplanar nonconvex source cutter-hole contact",
        )
        .ok()?;
        if !polygon_lies_in_closed_simple_polygon(&region, &component.boundary, projection)? {
            return None;
        }
        if simple_boundary_attachment_count(&component.boundary, &region, projection)? == 0 {
            return None;
        }
        regions.push(RemovedRegionCandidate {
            right_index,
            is_cutter: true,
            region,
        });
    }
    for hole in holes {
        let mut region = right_components.get(hole.right_index)?.hull.clone();
        orient_polygon_ccw(&mut region, projection)?;
        validate_projected_strictly_convex_loop(
            &region,
            projection,
            "coplanar nonconvex source cutter-hole contact",
        )
        .ok()?;
        if !polygon_strictly_inside_simple_polygon(&hole.ring, &component.boundary, projection)? {
            return None;
        }
        regions.push(RemovedRegionCandidate {
            right_index: hole.right_index,
            is_cutter: false,
            region,
        });
    }

    let groups = removed_region_contact_groups_allowing_incidental_points(&regions, projection)?;
    let mut saw_mixed_group = false;
    let mut removed_openings = Vec::new();
    let mut consumed_holes = Vec::new();
    for group in &groups {
        let has_cutter = group.iter().any(|&index| regions[index].is_cutter);
        let has_hole = group.iter().any(|&index| !regions[index].is_cutter);
        if !has_cutter {
            continue;
        }
        let mut opening = if has_hole {
            saw_mixed_group = true;
            materialize_removed_region_group_polygon_allowing_incidental_points(
                &regions, group, projection,
            )?
        } else {
            materialize_removed_region_group_or_single_polygon(&regions, group, projection)?
        };
        orient_polygon_ccw(&mut opening, projection)?;
        opening = simplify_projected_polygon(opening, projection);
        validate_projected_simple_loop(
            &opening,
            projection,
            "coplanar nonconvex source cutter-hole contact",
        )
        .ok()?;
        if !polygon_lies_in_closed_simple_polygon(&opening, &component.boundary, projection)? {
            return None;
        }
        if simple_boundary_attachment_count(&component.boundary, &opening, projection)? == 0 {
            return None;
        }
        if has_hole {
            consumed_holes.extend(
                group
                    .iter()
                    .copied()
                    .filter(|&index| !regions[index].is_cutter)
                    .map(|index| regions[index].right_index),
            );
        }
        removed_openings.push(opening);
    }
    if !saw_mixed_group {
        return None;
    }

    let (removed_openings, mut cut_polygons) =
        materialize_simple_source_side_cutter_difference_core(component, &removed_openings)?;
    for polygon in &mut cut_polygons {
        orient_polygon_ccw(polygon, projection)?;
    }
    let retained_holes = holes
        .iter()
        .filter(|hole| !consumed_holes.contains(&hole.right_index))
        .map(|hole| hole.ring.clone())
        .collect::<Vec<_>>();
    let holes_by_cut = assign_holes_to_side_cutter_split_outputs(
        &retained_holes,
        &cut_polygons,
        &removed_openings,
        projection,
    )?;
    Some(
        cut_polygons
            .into_iter()
            .zip(holes_by_cut)
            .map(|(outer, holes)| CoplanarConvexHoledComponent { outer, holes })
            .collect(),
    )
}

/// Check only the right components that affect one left component.
///
/// Earlier bounded component-holed replay required all right components to be
/// pairwise disjoint before the fallback cut/hole assignment could run. That
/// was unnecessarily global for multi-component differences: one left
/// component may consume an overlapping cutter/hole group while an independent
/// left component retains a strict hole. Following Yap, "Towards Exact
/// Geometric Computation," *Computational Geometry* 7.1-2 (1997), this helper
/// keeps the proof local to the exact source component whose topology is being
/// emitted. Components unrelated to that source component cannot invalidate
/// its retained loop; related components are still required to replay as
/// disjoint whenever the narrower fallback path relies on simple hole
/// assignment instead of explicit removed-region contact groups.
#[cfg(feature = "exact-triangulation")]
fn component_relevant_right_regions_are_disjoint(
    cut_indices: &[usize],
    holes: &[ComponentHoleCandidate],
    right_components: &[ConvexUnionComponent],
    projection: CoplanarProjection,
) -> Option<bool> {
    let mut indices = cut_indices.to_vec();
    for hole in holes {
        if !indices.contains(&hole.right_index) {
            indices.push(hole.right_index);
        }
    }
    if indices.len() < 2 {
        return Some(true);
    }
    let regions = indices
        .into_iter()
        .map(|index| {
            right_components
                .get(index)
                .map(|component| component.hull.clone())
        })
        .collect::<Option<Vec<_>>>()?;
    Some(
        validate_component_loops_disjoint(
            &regions,
            projection,
            "coplanar convex component-holed arrangement",
        )
        .is_ok(),
    )
}

/// Retained loops after a cutter/hole-contact split has been replayed.
///
/// This private object is intentionally smaller than
/// [`CoplanarConvexHoledComponent`]: it is the exact subtraction replay before
/// choosing whether the public artifact is component-holed or no-hole
/// multi-difference. Keeping that layer separate follows Yap, "Towards Exact
/// Geometric Computation," *Computational Geometry* 7.1-2 (1997): the
/// predicates certify contact, containment, and area first; only after that do
/// we expose the boolean object with the topology it actually owns.
#[cfg(feature = "exact-triangulation")]
struct CutterHoleContactSplitComponent {
    outer: Vec<Point3>,
    holes: Vec<Vec<Point3>>,
}

/// Replay mixed cutter/hole openings while optionally retaining strict holes.
///
/// This helper is the shared exact-object builder behind
/// [`materialize_cutter_hole_contact_component_holed_difference`] and
/// [`materialize_cutter_hole_contact_multi_component_difference_consuming_holes`].
/// It does not decide the public artifact family. A connected group of clipped
/// side cutters and strict holes is first replayed as a removed-region loop,
/// which consumes every strict hole in that group. The bounded extension here
/// is that the same component may also have independent cutter-only side
/// openings; those groups are materialized with the same retained side-opening
/// rule as [`materialize_nonrectilinear_side_cutter_opening`].
///
/// The result may contain components with no retained holes, several retained
/// holes, or several disjoint retained components produced by side-to-side
/// consumed groups. Callers then choose whether that evidence is a
/// component-holed output or a no-hole multi-difference. Boundary fragments
/// follow Weiler and Atherton, "Hidden Surface Removal Using Polygon Area
/// Sorting," *SIGGRAPH Computer Graphics* 11.2 (1977); acceptance follows
/// Yap's exact-computation boundary by requiring simple retained loops, exact
/// hole ownership, and exact area replay before promotion.
#[cfg(feature = "exact-triangulation")]
fn materialize_cutter_hole_contact_split_components(
    component: &ConvexUnionComponent,
    cut_indices: &[usize],
    holes: &[ComponentHoleCandidate],
    right_components: &[ConvexUnionComponent],
) -> Option<Vec<CutterHoleContactSplitComponent>> {
    if cut_indices.is_empty() || holes.is_empty() {
        return None;
    }
    let projection = component.projection;
    let mut regions = Vec::with_capacity(cut_indices.len() + holes.len());
    for &right_index in cut_indices {
        let mut clipped = convex_polygon_intersection_boundary(
            &right_components[right_index].hull,
            &component.hull,
            projection,
        )?;
        if clipped.len() < 3 {
            return None;
        }
        orient_polygon_ccw(&mut clipped, projection)?;
        regions.push(RemovedRegionCandidate {
            right_index,
            is_cutter: true,
            region: clipped,
        });
    }
    for hole in holes {
        let mut region = right_components[hole.right_index].hull.clone();
        orient_polygon_ccw(&mut region, projection)?;
        regions.push(RemovedRegionCandidate {
            right_index: hole.right_index,
            is_cutter: false,
            region,
        });
    }

    let groups = removed_region_contact_groups_allowing_incidental_points(&regions, projection)?;
    let mut opening_groups = Vec::new();
    let mut consumed_hole_groups = 0usize;
    for group in &groups {
        if group.iter().any(|&index| regions[index].is_cutter) {
            if group.iter().any(|&index| !regions[index].is_cutter) {
                consumed_hole_groups += 1;
            }
            opening_groups.push(group.clone());
        }
    }
    if opening_groups.is_empty() || consumed_hole_groups == 0 {
        return None;
    }

    let mut removed_openings = Vec::with_capacity(opening_groups.len());
    for group in &opening_groups {
        if group.iter().any(|&index| !regions[index].is_cutter) {
            removed_openings.push(
                materialize_removed_region_group_polygon_allowing_incidental_points(
                    &regions, group, projection,
                )?,
            );
        } else {
            removed_openings.push(materialize_removed_region_group_or_single_polygon(
                &regions, group, projection,
            )?);
        }
    }
    let opened_polygons = if opening_groups.len() == 1
        && opening_groups[0]
            .iter()
            .any(|&index| !regions[index].is_cutter)
    {
        let opening_indices = opening_groups[0]
            .iter()
            .map(|&index| regions[index].right_index)
            .collect::<Vec<_>>();
        let opening_mesh = merge_component_meshes(
            opening_indices
                .iter()
                .map(|&index| &right_components[index].mesh),
            "exact coplanar cutter-hole contact opening source",
        )?;
        if let Some(opening) =
            arrange_coplanar_surface_cutter_hole_contact_difference(&component.mesh, &opening_mesh)
        {
            vec![opening.polygon]
        } else {
            multi_side_opened_difference_polygons(
                &component.hull,
                &removed_openings,
                projection,
                "coplanar cutter-hole contact split difference",
            )?
        }
    } else if let Some(opening) = multi_side_opened_difference_polygon(
        &component.hull,
        &removed_openings,
        projection,
        "coplanar cutter-hole contact multi-opening difference",
    ) {
        vec![opening]
    } else {
        multi_side_opened_difference_polygons(
            &component.hull,
            &removed_openings,
            projection,
            "coplanar cutter-hole contact split difference",
        )?
    };
    if opened_polygons.len() > 1 {
        certify_removed_openings_split_source_component(
            &component.hull,
            &removed_openings,
            projection,
        )?;
    }

    let mut holes_by_opening = vec![Vec::new(); opened_polygons.len()];
    for hole in holes {
        let member_index = regions
            .iter()
            .position(|region| !region.is_cutter && region.right_index == hole.right_index)?;
        if opening_groups
            .iter()
            .any(|group| group.contains(&member_index))
        {
            continue;
        }
        let mut owner = None;
        for (index, opening) in opened_polygons.iter().enumerate() {
            if polygon_strictly_inside_simple_polygon(&hole.ring, opening, projection)? {
                if owner.is_some() {
                    return None;
                }
                owner = Some(index);
            }
        }
        holes_by_opening[owner?].push(hole.ring.clone());
    }
    for retained_holes in &mut holes_by_opening {
        sort_polygons_for_replay(retained_holes, projection);
    }
    Some(
        opened_polygons
            .into_iter()
            .zip(holes_by_opening)
            .map(|(outer, holes)| CutterHoleContactSplitComponent { outer, holes })
            .collect(),
    )
}

/// Replay mixed cutter/hole openings as component-holed output.
///
/// This helper is the holed-output sibling of
/// [`arrange_coplanar_surface_cutter_hole_contact_difference`]. A connected
/// group of clipped side cutters and strict holes is first replayed as a
/// removed-region loop, which consumes every strict hole in that group. The
/// bounded extension here is that the same component may also have independent
/// cutter-only side openings; those groups are materialized with the same
/// retained side-opening rule as
/// [`materialize_nonrectilinear_side_cutter_opening`]. The helper may return a
/// no-hole opened component when all holes in that source component are
/// consumed by certified side openings. That is not a standalone holed
/// certificate: [`validate_component_holed_surface_output`] still requires at
/// least one retained hole in the complete arrangement. This is the necessary
/// multi-component case where one source component is opened and another
/// independent source component still carries retained holes. A removed group
/// may also touch multiple source sides and split its own source component;
/// that multi-output case is accepted only when exact fragment replay produces
/// disjoint simple retained loops and exact projected area replays
/// `outer = sum(output_i) + sum(removed_i)`.
///
/// This covers the bounded partially straddling-hole case without claiming a
/// general planar subdivision: a hole overlapping a side-opening group is
/// consumed by that exact removed union, while unrelated strict holes are
/// retained only if exact simple-polygon containment proves they lie inside
/// the opened loop. Point-only connectivity and branch graphs still require a
/// full planar subdivision; incidental point contacts are admitted only inside
/// an already positive-connected removed group. Boundary fragments follow
/// Weiler and Atherton, "Hidden Surface Removal Using Polygon Area Sorting,"
/// *SIGGRAPH Computer Graphics* 11.2 (1977), and the replay/retained ring
/// split follows Yap, "Towards Exact Geometric Computation," *Computational
/// Geometry* 7.1-2 (1997).
#[cfg(feature = "exact-triangulation")]
fn materialize_cutter_hole_contact_component_holed_difference(
    component: &ConvexUnionComponent,
    cut_indices: &[usize],
    holes: &[ComponentHoleCandidate],
    right_components: &[ConvexUnionComponent],
) -> Option<Vec<CoplanarConvexHoledComponent>> {
    Some(
        materialize_cutter_hole_contact_split_components(
            component,
            cut_indices,
            holes,
            right_components,
        )?
        .into_iter()
        .map(|component| CoplanarConvexHoledComponent {
            outer: component.outer,
            holes: component.holes,
        })
        .collect(),
    )
}

/// Replay a cutter/hole-contact difference whose holes are all consumed.
///
/// This is the no-hole counterpart to
/// [`materialize_cutter_hole_contact_component_holed_difference`]. It accepts
/// the same exact removed-region contact groups, including side-to-side
/// groups that split the source component, but promotes them only when every
/// strict hole belongs to a consumed group and no retained hole remains in any
/// emitted component. The output is therefore a vector of plain retained
/// outer loops for [`arrange_coplanar_surface_multi_difference`], not a
/// component-holed artifact with empty hole lists.
///
/// The distinction is the object/predicate separation in Yap, "Towards Exact
/// Geometric Computation," *Computational Geometry* 7.1-2 (1997): once exact
/// contact topology proves the holes are removed, the retained boolean object
/// should expose only the no-hole surface loops it actually owns. The loop
/// construction is still the Weiler-Atherton retained-fragment traversal cited
/// by the holed sibling; see Weiler and Atherton, "Hidden Surface Removal
/// Using Polygon Area Sorting," *SIGGRAPH Computer Graphics* 11.2 (1977).
#[cfg(feature = "exact-triangulation")]
fn materialize_cutter_hole_contact_multi_component_difference_consuming_holes(
    component: &ConvexUnionComponent,
    cut_indices: &[usize],
    holes: &[ComponentHoleCandidate],
    right_components: &[ConvexUnionComponent],
    label: &'static str,
) -> Option<Vec<Vec<Point3>>> {
    if cut_indices.is_empty() || holes.is_empty() {
        return None;
    }
    let mut components = materialize_cutter_hole_contact_split_components(
        component,
        cut_indices,
        holes,
        right_components,
    )?;
    if components
        .iter()
        .any(|component| !component.holes.is_empty())
    {
        return None;
    }
    let projection = component.projection;
    let mut polygons = components
        .drain(..)
        .map(|component| component.outer)
        .collect::<Vec<_>>();
    for polygon in &mut polygons {
        orient_polygon_ccw(polygon, projection)?;
        validate_projected_simple_loop(polygon, projection, label).ok()?;
    }
    validate_simple_component_loops_disjoint(&polygons, projection, label).ok()?;
    sort_polygons_for_replay(&mut polygons, projection);
    Some(polygons)
}

/// Replay non-rectilinear side-cutter openings while retaining strict holes.
///
/// This is the cutter-only sibling of
/// [`materialize_cutter_hole_contact_component_holed_difference`]. One or
/// more side-attached convex cutters may overlap or touch along
/// positive-length boundaries inside each connected group, but a group is
/// promoted only after its clipped regions replay as one exact simple union
/// loop. Disconnected groups become independent side openings through
/// [`multi_side_opened_difference_polygon`]. The final output area must
/// satisfy `area(component) = area(opened) + sum(area(opening_i))` exactly.
/// Strict holes are then classified by exact containment: holes inside the
/// retained opened loop remain holes, while holes strictly inside exactly one
/// removed opening are consumed by that opening and omitted. A hole that is
/// split by an opening boundary remains unsupported because its ownership
/// would require a general planar-cell subdivision.
///
/// This is a bounded retained-fragment construction in the sense of Yap,
/// "Towards Exact Geometric Computation," *Computational Geometry* 7.1-2
/// (1997): no floating witness selects the bay topology. The boundary splice
/// is the same Weiler-Atherton retained-edge traversal used elsewhere in this
/// module; see Weiler and Atherton, "Hidden Surface Removal Using Polygon Area
/// Sorting," *SIGGRAPH Computer Graphics* 11.2 (1977). Fully rectangular
/// cases remain with the orthogonal cell materializer of de Berg, Cheong, van
/// Kreveld, and Overmars, *Computational Geometry: Algorithms and
/// Applications*, 3rd ed. (2008), Chapter 2, so this helper covers the
/// non-rectilinear multi-cutter gap instead of changing rectilinear shortcut
/// precedence. Branch graphs and partially consumed holes stay outside this
/// helper because they require explicit planar-cell ownership.
#[cfg(feature = "exact-triangulation")]
fn materialize_connected_multi_cutter_component_holed_difference(
    component: &ConvexUnionComponent,
    cut_indices: &[usize],
    holes: &[ComponentHoleCandidate],
    right_components: &[ConvexUnionComponent],
) -> Option<Vec<CoplanarConvexHoledComponent>> {
    if cut_indices.is_empty() || holes.is_empty() {
        return None;
    }
    let projection = component.projection;
    let (removed_openings, opening) = materialize_nonrectilinear_side_cutter_opening(
        component,
        cut_indices,
        right_components,
        "coplanar component-holed connected multi-cutter opening",
    )?;
    let retained_holes = assign_holes_to_connected_multi_cutter_opening(
        holes,
        &opening,
        &removed_openings,
        projection,
    )?;
    Some(vec![CoplanarConvexHoledComponent {
        outer: opening,
        holes: retained_holes,
    }])
}

/// Replay a no-hole side-cutter opening that consumes strict holes.
///
/// This is the no-hole counterpart to
/// [`materialize_connected_multi_cutter_component_holed_difference`]. It
/// admits the same exact non-rectilinear side-opening replay, but promotes
/// only when every strict interior right ring is wholly contained in exactly
/// one removed opening. Any retained, boundary-touching, or ambiguously owned
/// ring must stay with the component-holed or planar-cell materializer.
///
/// The rule is deliberately object-level, following Yap, "Towards Exact
/// Geometric Computation," *Computational Geometry* 7.1-2 (1997): deleting a
/// hole is a topology change justified by exact containment in a named
/// removed object. The removed/opened boundary replay is the
/// Weiler-Atherton retained-fragment traversal from Weiler and Atherton,
/// "Hidden Surface Removal Using Polygon Area Sorting," *SIGGRAPH Computer
/// Graphics* 11.2 (1977).
#[cfg(feature = "exact-triangulation")]
fn materialize_side_cutter_opening_difference_consuming_holes(
    component: &ConvexUnionComponent,
    cut_indices: &[usize],
    holes: &[ComponentHoleCandidate],
    right_components: &[ConvexUnionComponent],
    label: &'static str,
) -> Option<Vec<Vec<Point3>>> {
    if cut_indices.is_empty() || holes.is_empty() {
        return None;
    }
    let projection = component.projection;
    let Some((removed_openings, mut opening)) = materialize_nonrectilinear_side_cutter_opening(
        component,
        cut_indices,
        right_components,
        label,
    ) else {
        return None;
    };
    for hole in holes {
        if !hole_strictly_consumed_by_one_removed_opening(
            &hole.ring,
            &removed_openings,
            projection,
        )? {
            return None;
        }
    }
    orient_polygon_ccw(&mut opening, projection)?;
    if validate_projected_simple_loop(&opening, projection, label).is_err() {
        return None;
    }
    Some(vec![opening])
}

/// Replay a non-rectilinear side-cutter split while owning strict holes.
///
/// The side-cutter split helper proves the no-hole case where several
/// side-attached cutters, or one side-to-side cutter, divide one convex source
/// sheet into two or more simple retained loops. This helper lifts the same
/// exact cell evidence into the component/holed artifact: after the split
/// loops are replayed, each strict hole must be owned by exactly one retained
/// output loop or wholly consumed by exactly one removed side opening.
/// Boundary contact, overlap with several removed openings, or holes that
/// straddle a split boundary stay outside this bounded certificate.
///
/// The construction follows Yap's retained-object rule from "Towards Exact
/// Geometric Computation," *Computational Geometry* 7.1-2 (1997): the wider
/// shortcut is admitted only when retained split loops, consumed holes, and
/// retained holes replay from exact predicates. The split-loop boundary
/// traversal is the Weiler-Atherton retained-fragment construction used by
/// [`materialize_side_cutter_multi_component_difference`]; see Weiler and
/// Atherton, "Hidden Surface Removal Using Polygon Area Sorting," *SIGGRAPH
/// Computer Graphics* 11.2 (1977).
#[cfg(feature = "exact-triangulation")]
fn materialize_side_cutter_multi_component_holed_difference(
    component: &ConvexUnionComponent,
    cut_indices: &[usize],
    holes: &[ComponentHoleCandidate],
    right_components: &[ConvexUnionComponent],
) -> Option<Vec<CoplanarConvexHoledComponent>> {
    if cut_indices.is_empty() || holes.is_empty() {
        return None;
    }
    let projection = component.projection;
    let (removed_openings, cut_polygons) = materialize_side_cutter_multi_component_difference_core(
        component,
        cut_indices,
        right_components,
        "coplanar component-holed non-rectilinear side-cutter split",
    )?;
    if cut_polygons.len() < 2 {
        return None;
    }
    let hole_rings = holes
        .iter()
        .map(|hole| hole.ring.clone())
        .collect::<Vec<_>>();
    let holes_by_cut = assign_holes_to_side_cutter_split_outputs(
        &hole_rings,
        &cut_polygons,
        &removed_openings,
        projection,
    )?;
    if holes_by_cut.iter().all(Vec::is_empty) {
        return None;
    }
    Some(
        cut_polygons
            .into_iter()
            .zip(holes_by_cut)
            .map(|(outer, holes)| CoplanarConvexHoledComponent { outer, holes })
            .collect(),
    )
}

/// Replay point-branch side cutters while retaining strict holes.
///
/// This is the component-holed sibling of
/// [`materialize_side_cutter_point_touch_difference_core`]. The ordinary
/// component-holed side-cutter split requires disjoint retained loops; this
/// bounded artifact accepts the exact branch case where clipped side openings
/// meet only at retained vertices and the final outer loops duplicate those
/// branch coordinates. Strict holes are still assigned by exact containment in
/// one retained loop or consumed by exactly one removed opening.
///
/// The branch replay uses the same Weiler-Atherton retained-fragment walk as
/// the no-hole point-touch difference. The hole-ownership rule follows Yap,
/// "Towards Exact Geometric Computation," *Computational Geometry* 7.1-2
/// (1997): adding a branch carrier does not license sampled hole assignment.
/// Every retained/consumed ring is named by exact predicates before the
/// `hypertri` earcut handoff triangulates the holed components, following
/// Held, "FIST: Fast Industrial-Strength Triangulation of Polygons,"
/// *Algorithmica* 30 (2001).
#[cfg(feature = "exact-triangulation")]
fn materialize_side_cutter_point_touch_component_holed_difference(
    component: &ConvexUnionComponent,
    cut_indices: &[usize],
    holes: &[ComponentHoleCandidate],
    right_components: &[ConvexUnionComponent],
) -> Option<Vec<CoplanarConvexHoledComponent>> {
    if cut_indices.len() < 2 || holes.is_empty() {
        return None;
    }
    let projection = component.projection;
    let (removed_openings, mut cut_polygons) = materialize_side_cutter_point_touch_difference_core(
        component,
        cut_indices,
        right_components,
        "coplanar component-holed point-touch side-cutter split",
    )?;
    certify_removed_openings_collectively_split_source_component(
        &component.hull,
        &removed_openings,
        projection,
    )?;
    for polygon in &mut cut_polygons {
        orient_polygon_ccw(polygon, projection)?;
    }
    let hole_rings = holes
        .iter()
        .map(|hole| hole.ring.clone())
        .collect::<Vec<_>>();
    let holes_by_cut = assign_holes_to_side_cutter_split_outputs(
        &hole_rings,
        &cut_polygons,
        &removed_openings,
        projection,
    )?;
    if holes_by_cut.iter().all(Vec::is_empty) {
        return None;
    }
    Some(
        cut_polygons
            .into_iter()
            .zip(holes_by_cut)
            .map(|(outer, holes)| CoplanarConvexHoledComponent { outer, holes })
            .collect(),
    )
}

/// Replay mixed retained/consumed holes on convex point-branch differences.
///
/// [`materialize_side_cutter_point_touch_component_holed_difference`] handles
/// branch splits whose holes are retained in output loops or wholly contained
/// in removed openings. This sibling covers the harder mixed case where a
/// strict hole straddles one branch opening while unrelated holes remain
/// retained. Each consumed ring must have positive-dimensional contact with
/// exactly one clipped opening; that ring is unioned into the removed object
/// before branch replay, while disjoint rings are assigned to retained
/// branch loops afterward. Point-only contact and multi-opening ownership are
/// rejected.
///
/// This is a bounded retained-object certificate in Yap's sense from
/// "Towards Exact Geometric Computation," *Computational Geometry* 7.1-2
/// (1997): the omitted ring is named by exact removed-object topology, and
/// the retained rings are named by exact containment in emitted loops. The
/// removed-opening unions and final branch loops use the Weiler-Atherton
/// retained-fragment construction cited by
/// [`materialize_side_cutter_point_touch_removed_openings_core`], and contact
/// dimensionality is certified by the Guigue-Devillers orientation-predicate
/// segment classifier used throughout this module.
#[cfg(feature = "exact-triangulation")]
fn materialize_side_cutter_point_touch_component_holed_difference_consuming_hole_contacts(
    component: &ConvexUnionComponent,
    cut_indices: &[usize],
    holes: &[ComponentHoleCandidate],
    right_components: &[ConvexUnionComponent],
) -> Option<Vec<CoplanarConvexHoledComponent>> {
    if cut_indices.len() < 2 || holes.is_empty() {
        return None;
    }
    let projection = component.projection;
    let mut all_clipped_cutters_are_rectangles = true;
    let mut openings = Vec::with_capacity(cut_indices.len());
    for &right_index in cut_indices {
        let mut clipped = convex_polygon_intersection_boundary(
            &right_components.get(right_index)?.hull,
            &component.hull,
            projection,
        )?;
        if clipped.len() < 3 {
            return None;
        }
        orient_polygon_ccw(&mut clipped, projection)?;
        clipped = simplify_projected_polygon(clipped, projection);
        validate_projected_simple_loop(
            &clipped,
            projection,
            "coplanar component-holed point-touch straddling-hole side-cutter split",
        )
        .ok()?;
        all_clipped_cutters_are_rectangles &=
            projected_axis_aligned_rectangle(&clipped, projection).is_some();
        openings.push(clipped);
    }
    if all_clipped_cutters_are_rectangles {
        return None;
    }

    let mut holes_by_opening = vec![Vec::<Vec<Point3>>::new(); openings.len()];
    let mut retained_holes = Vec::new();
    let mut saw_consumed_contact = false;
    for hole in holes {
        if !polygon_strictly_inside_convex_polygon(&hole.ring, &component.hull, projection)? {
            return None;
        }
        let mut hole_region = hole.ring.clone();
        orient_polygon_ccw(&mut hole_region, projection)?;
        hole_region = simplify_projected_polygon(hole_region, projection);
        validate_projected_simple_loop(
            &hole_region,
            projection,
            "coplanar component-holed point-touch straddling-hole side-cutter split",
        )
        .ok()?;

        let mut owner = None;
        for (opening_index, opening) in openings.iter().enumerate() {
            match simple_polygon_interaction(&hole_region, opening, projection)? {
                SimplePolygonInteraction::Disjoint => {}
                SimplePolygonInteraction::PointOnly => return None,
                SimplePolygonInteraction::Connected => {
                    if owner.replace(opening_index).is_some() {
                        return None;
                    }
                }
            }
        }
        if let Some(opening_index) = owner {
            saw_consumed_contact = true;
            holes_by_opening[opening_index].push(hole_region);
        } else {
            let mut retained = hole.ring.clone();
            orient_polygon_cw(&mut retained, projection)?;
            retained_holes.push(retained);
        }
    }
    if !saw_consumed_contact || retained_holes.is_empty() {
        return None;
    }

    let mut merged_openings = Vec::with_capacity(openings.len());
    for (opening, owned_holes) in openings.into_iter().zip(holes_by_opening) {
        let mut merged = if owned_holes.is_empty() {
            opening
        } else {
            let mut group_polygons = Vec::with_capacity(1 + owned_holes.len());
            group_polygons.push(opening);
            group_polygons.extend(owned_holes);
            let group = (0..group_polygons.len()).collect::<Vec<_>>();
            materialize_simple_polygon_union_group(
                &group_polygons,
                &group,
                projection,
                "coplanar component-holed point-touch straddling-hole side-cutter split",
            )?
        };
        orient_polygon_ccw(&mut merged, projection)?;
        merged = simplify_projected_polygon(merged, projection);
        validate_projected_simple_loop(
            &merged,
            projection,
            "coplanar component-holed point-touch straddling-hole side-cutter split",
        )
        .ok()?;
        if convex_boundary_attachment_count(&component.hull, &merged, projection)? == 0 {
            return None;
        }
        for point in &merged {
            if convex_polygon_location(point, &component.hull, projection)?
                == ConvexPolygonLocation::Outside
            {
                return None;
            }
        }
        merged_openings.push(merged);
    }

    let (removed_openings, mut cut_polygons) =
        materialize_side_cutter_point_touch_removed_openings_core(
            component,
            &merged_openings,
            "coplanar component-holed point-touch straddling-hole side-cutter split",
        )?;
    certify_removed_openings_collectively_split_source_component(
        &component.hull,
        &removed_openings,
        projection,
    )?;
    for polygon in &mut cut_polygons {
        orient_polygon_ccw(polygon, projection)?;
    }
    let holes_by_cut = assign_holes_to_side_cutter_split_outputs(
        &retained_holes,
        &cut_polygons,
        &removed_openings,
        projection,
    )?;
    if holes_by_cut.iter().all(Vec::is_empty) {
        return None;
    }
    Some(
        cut_polygons
            .into_iter()
            .zip(holes_by_cut)
            .map(|(outer, holes)| CoplanarConvexHoledComponent { outer, holes })
            .collect(),
    )
}

/// Replay grouped straddling-hole ownership while retaining unrelated holes.
///
/// This is the component-holed sibling of
/// [`materialize_side_cutter_point_touch_difference_consuming_hole_contact_groups`].
/// It accepts a bounded higher-order branch case: strict rings that are
/// disjoint from every removed opening remain retained holes, while a strict
/// ring with positive-dimensional contact to several side openings may be
/// consumed when the whole cutter/ring contact group replays as one simple
/// removed object. Point-only hole contact is rejected, and hole-only contact
/// groups are rejected because they do not name a removed object.
///
/// The policy follows Yap, "Towards Exact Geometric Computation,"
/// *Computational Geometry* 7.1-2 (1997): a ring is either retained as an
/// exact output object or deleted only after exact predicates identify the
/// removed object that owns it. The grouped removed object is built by the
/// Weiler-Atherton retained-fragment construction cited by
/// [`materialize_simple_polygon_union_group`], and contact dimensionality is
/// certified by the Guigue-Devillers orientation-predicate classifier exposed
/// through [`simple_polygon_interaction`]. Final holed output is still
/// triangulated only after exact ring assignment, following Held, "FIST: Fast
/// Industrial-Strength Triangulation of Polygons," *Algorithmica* 30 (2001).
#[cfg(feature = "exact-triangulation")]
fn materialize_side_cutter_point_touch_component_holed_difference_consuming_hole_contact_groups(
    component: &ConvexUnionComponent,
    cut_indices: &[usize],
    holes: &[ComponentHoleCandidate],
    right_components: &[ConvexUnionComponent],
) -> Option<Vec<CoplanarConvexHoledComponent>> {
    if cut_indices.len() < 2 || holes.is_empty() {
        return None;
    }
    let projection = component.projection;
    let label = "coplanar component-holed grouped point-touch straddling-hole side-cutter split";
    let mut regions = Vec::with_capacity(cut_indices.len() + holes.len());
    let mut all_clipped_cutters_are_rectangles = true;
    for &right_index in cut_indices {
        let mut clipped = convex_polygon_intersection_boundary(
            &right_components.get(right_index)?.hull,
            &component.hull,
            projection,
        )?;
        if clipped.len() < 3 {
            return None;
        }
        orient_polygon_ccw(&mut clipped, projection)?;
        clipped = simplify_projected_polygon(clipped, projection);
        validate_projected_simple_loop(&clipped, projection, label).ok()?;
        all_clipped_cutters_are_rectangles &=
            projected_axis_aligned_rectangle(&clipped, projection).is_some();
        regions.push(RemovedRegionCandidate {
            right_index,
            is_cutter: true,
            region: clipped,
        });
    }
    if all_clipped_cutters_are_rectangles {
        return None;
    }
    for (hole_index, hole) in holes.iter().enumerate() {
        if !polygon_strictly_inside_convex_polygon(&hole.ring, &component.hull, projection)? {
            return None;
        }
        let mut region = hole.ring.clone();
        orient_polygon_ccw(&mut region, projection)?;
        region = simplify_projected_polygon(region, projection);
        validate_projected_simple_loop(&region, projection, label).ok()?;
        regions.push(RemovedRegionCandidate {
            right_index: hole_index,
            is_cutter: false,
            region,
        });
    }

    let mut contact_graph = UnionFind::new(regions.len());
    for left in 0..regions.len() {
        for right in left + 1..regions.len() {
            match simple_polygon_interaction(
                &regions[left].region,
                &regions[right].region,
                projection,
            )? {
                SimplePolygonInteraction::Disjoint => {}
                SimplePolygonInteraction::PointOnly => {
                    if !regions[left].is_cutter || !regions[right].is_cutter {
                        return None;
                    }
                }
                SimplePolygonInteraction::Connected => contact_graph.union(left, right),
            }
        }
    }

    let mut groups: Vec<(usize, Vec<usize>)> = Vec::new();
    for index in 0..regions.len() {
        let root = contact_graph.find(index);
        if let Some((_, members)) = groups.iter_mut().find(|(candidate, _)| *candidate == root) {
            members.push(index);
        } else {
            groups.push((root, vec![index]));
        }
    }
    groups.sort_by_key(|(_, members)| members.first().copied().unwrap_or(usize::MAX));

    let mut retained_holes = Vec::new();
    let mut removed_openings = Vec::new();
    let mut saw_multi_opening_consumed_group = false;
    for (_, group) in groups {
        let cutter_count = group
            .iter()
            .filter(|&&index| regions[index].is_cutter)
            .count();
        let hole_count = group
            .iter()
            .filter(|&&index| !regions[index].is_cutter)
            .count();
        if cutter_count == 0 {
            if hole_count != 1 || group.len() != 1 {
                return None;
            }
            let mut retained = regions[group[0]].region.clone();
            orient_polygon_cw(&mut retained, projection)?;
            retained_holes.push(retained);
            continue;
        }
        if hole_count > 0 && cutter_count > 1 {
            saw_multi_opening_consumed_group = true;
        }

        let mut opening = if group.len() == 1 {
            regions[group[0]].region.clone()
        } else {
            let polygons = group
                .iter()
                .map(|&index| regions[index].region.clone())
                .collect::<Vec<_>>();
            let all = (0..polygons.len()).collect::<Vec<_>>();
            materialize_simple_polygon_union_group(&polygons, &all, projection, label)?
        };
        orient_polygon_ccw(&mut opening, projection)?;
        opening = simplify_projected_polygon(opening, projection);
        validate_projected_simple_loop(&opening, projection, label).ok()?;
        if convex_boundary_attachment_count(&component.hull, &opening, projection)? == 0 {
            return None;
        }
        for point in &opening {
            if convex_polygon_location(point, &component.hull, projection)?
                == ConvexPolygonLocation::Outside
            {
                return None;
            }
        }
        removed_openings.push(opening);
    }
    if !saw_multi_opening_consumed_group || retained_holes.is_empty() {
        return None;
    }

    let (removed_openings, mut cut_polygons) =
        materialize_side_cutter_point_touch_removed_openings_core(
            component,
            &removed_openings,
            label,
        )?;
    certify_removed_openings_collectively_split_source_component(
        &component.hull,
        &removed_openings,
        projection,
    )?;
    for polygon in &mut cut_polygons {
        orient_polygon_ccw(polygon, projection)?;
    }
    let holes_by_cut = assign_holes_to_side_cutter_split_outputs(
        &retained_holes,
        &cut_polygons,
        &removed_openings,
        projection,
    )?;
    if holes_by_cut.iter().all(Vec::is_empty) {
        return None;
    }
    Some(
        cut_polygons
            .into_iter()
            .zip(holes_by_cut)
            .map(|(outer, holes)| CoplanarConvexHoledComponent { outer, holes })
            .collect(),
    )
}

/// Replay point-branch side cutters whose strict holes are all consumed.
///
/// [`CoplanarSurfacePointTouchDifference`] is the only existing surface
/// artifact that can honestly retain these outputs: the retained loops share
/// exact branch vertices, so a disjoint multi-difference would erase topology.
/// This helper therefore reuses
/// [`materialize_side_cutter_point_touch_difference_core`] and merely adds the
/// no-retained-hole ownership gate. Every strict hole must be wholly inside
/// exactly one removed branch opening before it may be omitted from the
/// output; retained holes stay with
/// [`materialize_side_cutter_point_touch_component_holed_difference`].
///
/// Yap's "Towards Exact Geometric Computation," *Computational Geometry*
/// 7.1-2 (1997), is the policy boundary: deleting a ring is a certified
/// object-level fact, not a consequence of a representative point. The branch
/// loops themselves are still stitched by the Weiler-Atherton retained-edge
/// replay cited by [`materialize_side_cutter_point_touch_difference_core`].
#[cfg(feature = "exact-triangulation")]
fn materialize_side_cutter_point_touch_difference_consuming_holes(
    component: &ConvexUnionComponent,
    cut_indices: &[usize],
    holes: &[ComponentHoleCandidate],
    right_components: &[ConvexUnionComponent],
    label: &'static str,
) -> Option<Vec<Vec<Point3>>> {
    if cut_indices.len() < 2 || holes.is_empty() {
        return None;
    }
    let projection = component.projection;
    let (removed_openings, mut cut_polygons) = materialize_side_cutter_point_touch_difference_core(
        component,
        cut_indices,
        right_components,
        label,
    )?;
    certify_removed_openings_collectively_split_source_component(
        &component.hull,
        &removed_openings,
        projection,
    )?;
    for hole in holes {
        if !hole_strictly_consumed_by_one_removed_opening(
            &hole.ring,
            &removed_openings,
            projection,
        )? {
            return None;
        }
    }
    for polygon in &mut cut_polygons {
        orient_polygon_ccw(polygon, projection)?;
        validate_projected_simple_loop(polygon, projection, label).ok()?;
    }
    validate_simple_component_loops_disjoint_allowing_vertex_point_touches(
        &cut_polygons,
        projection,
        label,
    )
    .ok()?;
    sort_polygons_for_replay(&mut cut_polygons, projection);
    Some(cut_polygons)
}

/// Replay convex source point branches that consume straddling holes.
///
/// This is the convex-source counterpart to
/// [`materialize_simple_source_side_cutter_point_touch_difference_consuming_hole_contacts`].
/// It admits only the bounded case where each strict source hole has
/// positive-dimensional contact with exactly one clipped branch opening. The
/// owned hole is first unioned into that removed opening, and the resulting
/// removed-opening set is replayed by the branch-aware retained-fragment
/// stitcher. Point-only hole contact and holes touching multiple openings are
/// rejected because they do not name one 2D removed object.
///
/// Yap's "Towards Exact Geometric Computation," *Computational Geometry*
/// 7.1-2 (1997), is the certificate boundary: deleting a ring is permitted
/// only after exact predicates name the removed object that owns it. The
/// opening union and branch replay use the Weiler-Atherton retained-boundary
/// construction cited by [`materialize_simple_polygon_union_group`] and
/// [`materialize_side_cutter_point_touch_removed_openings_core`]; contacts
/// are exact Guigue-Devillers orientation-predicate classifications.
#[cfg(feature = "exact-triangulation")]
fn materialize_side_cutter_point_touch_difference_consuming_hole_contacts(
    component: &ConvexUnionComponent,
    cut_indices: &[usize],
    holes: &[ComponentHoleCandidate],
    right_components: &[ConvexUnionComponent],
    label: &'static str,
) -> Option<Vec<Vec<Point3>>> {
    if cut_indices.len() < 2 || holes.is_empty() {
        return None;
    }
    let projection = component.projection;
    let mut all_clipped_cutters_are_rectangles = true;
    let mut openings = Vec::with_capacity(cut_indices.len());
    for &right_index in cut_indices {
        let mut clipped = convex_polygon_intersection_boundary(
            &right_components.get(right_index)?.hull,
            &component.hull,
            projection,
        )?;
        if clipped.len() < 3 {
            return None;
        }
        orient_polygon_ccw(&mut clipped, projection)?;
        clipped = simplify_projected_polygon(clipped, projection);
        validate_projected_simple_loop(&clipped, projection, label).ok()?;
        all_clipped_cutters_are_rectangles &=
            projected_axis_aligned_rectangle(&clipped, projection).is_some();
        openings.push(clipped);
    }
    if all_clipped_cutters_are_rectangles {
        return None;
    }

    let mut holes_by_opening = vec![Vec::<Vec<Point3>>::new(); openings.len()];
    for hole in holes {
        if !polygon_strictly_inside_convex_polygon(&hole.ring, &component.hull, projection)? {
            return None;
        }
        let mut hole_region = hole.ring.clone();
        orient_polygon_ccw(&mut hole_region, projection)?;
        hole_region = simplify_projected_polygon(hole_region, projection);
        validate_projected_simple_loop(&hole_region, projection, label).ok()?;

        let mut owner = None;
        for (opening_index, opening) in openings.iter().enumerate() {
            match simple_polygon_interaction(&hole_region, opening, projection)? {
                SimplePolygonInteraction::Disjoint => {}
                SimplePolygonInteraction::PointOnly => return None,
                SimplePolygonInteraction::Connected => {
                    if owner.replace(opening_index).is_some() {
                        return None;
                    }
                }
            }
        }
        holes_by_opening[owner?].push(hole_region);
    }

    let mut merged_openings = Vec::with_capacity(openings.len());
    for (opening, owned_holes) in openings.into_iter().zip(holes_by_opening) {
        let mut merged = if owned_holes.is_empty() {
            opening
        } else {
            let mut group_polygons = Vec::with_capacity(1 + owned_holes.len());
            group_polygons.push(opening);
            group_polygons.extend(owned_holes);
            let group = (0..group_polygons.len()).collect::<Vec<_>>();
            materialize_simple_polygon_union_group(&group_polygons, &group, projection, label)?
        };
        orient_polygon_ccw(&mut merged, projection)?;
        merged = simplify_projected_polygon(merged, projection);
        validate_projected_simple_loop(&merged, projection, label).ok()?;
        if convex_boundary_attachment_count(&component.hull, &merged, projection)? == 0 {
            return None;
        }
        for point in &merged {
            if convex_polygon_location(point, &component.hull, projection)?
                == ConvexPolygonLocation::Outside
            {
                return None;
            }
        }
        merged_openings.push(merged);
    }

    let (removed_openings, mut cut_polygons) =
        materialize_side_cutter_point_touch_removed_openings_core(
            component,
            &merged_openings,
            label,
        )?;
    certify_removed_openings_collectively_split_source_component(
        &component.hull,
        &removed_openings,
        projection,
    )?;
    for polygon in &mut cut_polygons {
        orient_polygon_ccw(polygon, projection)?;
        validate_projected_simple_loop(polygon, projection, label).ok()?;
    }
    validate_simple_component_loops_disjoint_allowing_vertex_point_touches(
        &cut_polygons,
        projection,
        label,
    )
    .ok()?;
    sort_polygons_for_replay(&mut cut_polygons, projection);
    Some(cut_polygons)
}

/// Replay point branches whose straddling holes touch several openings.
///
/// The single-opening sibling
/// [`materialize_side_cutter_point_touch_difference_consuming_hole_contacts`]
/// deliberately rejects a strict hole with positive contact against multiple
/// branch openings. This helper accepts the next bounded case: every strict
/// hole must be part of one positive-dimensional contact group that contains
/// at least one clipped side opening, at least one consumed hole must contact
/// several openings, and the resulting group union must replay as a simple
/// source-owned removed object. Point-only hole contact still rejects; exact
/// point contacts are retained only between removed opening groups, where they
/// are the named branch vertices of
/// [`materialize_side_cutter_point_touch_removed_openings_core`].
///
/// This follows Yap, "Towards Exact Geometric Computation,"
/// *Computational Geometry* 7.1-2 (1997): a deleted ring must be named by an
/// exact removed object before retained topology is emitted. The group unions
/// and branch replay use the Weiler-Atherton retained-fragment construction
/// cited by [`materialize_simple_polygon_union_group`] and
/// [`materialize_side_cutter_point_touch_removed_openings_core`]. Contact
/// dimensionality is classified by the Guigue-Devillers orientation
/// predicates exposed through [`simple_polygon_interaction`].
#[cfg(feature = "exact-triangulation")]
fn materialize_side_cutter_point_touch_difference_consuming_hole_contact_groups(
    component: &ConvexUnionComponent,
    cut_indices: &[usize],
    holes: &[ComponentHoleCandidate],
    right_components: &[ConvexUnionComponent],
    label: &'static str,
) -> Option<Vec<Vec<Point3>>> {
    if cut_indices.len() < 2 || holes.is_empty() {
        return None;
    }
    let projection = component.projection;
    let mut regions = Vec::with_capacity(cut_indices.len() + holes.len());
    let mut all_clipped_cutters_are_rectangles = true;
    for &right_index in cut_indices {
        let mut clipped = convex_polygon_intersection_boundary(
            &right_components.get(right_index)?.hull,
            &component.hull,
            projection,
        )?;
        if clipped.len() < 3 {
            return None;
        }
        orient_polygon_ccw(&mut clipped, projection)?;
        clipped = simplify_projected_polygon(clipped, projection);
        validate_projected_simple_loop(&clipped, projection, label).ok()?;
        all_clipped_cutters_are_rectangles &=
            projected_axis_aligned_rectangle(&clipped, projection).is_some();
        regions.push(RemovedRegionCandidate {
            right_index,
            is_cutter: true,
            region: clipped,
        });
    }
    if all_clipped_cutters_are_rectangles {
        return None;
    }
    for hole in holes {
        if !polygon_strictly_inside_convex_polygon(&hole.ring, &component.hull, projection)? {
            return None;
        }
        let mut region = hole.ring.clone();
        orient_polygon_ccw(&mut region, projection)?;
        region = simplify_projected_polygon(region, projection);
        validate_projected_simple_loop(&region, projection, label).ok()?;
        regions.push(RemovedRegionCandidate {
            right_index: hole.right_index,
            is_cutter: false,
            region,
        });
    }

    let mut contact_graph = UnionFind::new(regions.len());
    for left in 0..regions.len() {
        for right in left + 1..regions.len() {
            match simple_polygon_interaction(
                &regions[left].region,
                &regions[right].region,
                projection,
            )? {
                SimplePolygonInteraction::Disjoint => {}
                SimplePolygonInteraction::PointOnly => {
                    if !regions[left].is_cutter || !regions[right].is_cutter {
                        return None;
                    }
                }
                SimplePolygonInteraction::Connected => contact_graph.union(left, right),
            }
        }
    }

    let mut groups: Vec<(usize, Vec<usize>)> = Vec::new();
    for index in 0..regions.len() {
        let root = contact_graph.find(index);
        if let Some((_, members)) = groups.iter_mut().find(|(candidate, _)| *candidate == root) {
            members.push(index);
        } else {
            groups.push((root, vec![index]));
        }
    }
    groups.sort_by_key(|(_, members)| members.first().copied().unwrap_or(usize::MAX));

    let mut consumed_holes = Vec::new();
    let mut removed_openings = Vec::new();
    let mut saw_multi_opening_consumed_group = false;
    for (_, group) in groups {
        let cutter_count = group
            .iter()
            .filter(|&&index| regions[index].is_cutter)
            .count();
        let hole_count = group
            .iter()
            .filter(|&&index| !regions[index].is_cutter)
            .count();
        if cutter_count == 0 {
            return None;
        }
        if hole_count > 0 {
            consumed_holes.extend(
                group
                    .iter()
                    .copied()
                    .filter(|&index| !regions[index].is_cutter)
                    .map(|index| regions[index].right_index),
            );
            if cutter_count > 1 {
                saw_multi_opening_consumed_group = true;
            }
        }
        let mut opening = if group.len() == 1 {
            regions[group[0]].region.clone()
        } else {
            let polygons = group
                .iter()
                .map(|&index| regions[index].region.clone())
                .collect::<Vec<_>>();
            let all = (0..polygons.len()).collect::<Vec<_>>();
            materialize_simple_polygon_union_group(&polygons, &all, projection, label)?
        };
        orient_polygon_ccw(&mut opening, projection)?;
        opening = simplify_projected_polygon(opening, projection);
        validate_projected_simple_loop(&opening, projection, label).ok()?;
        if convex_boundary_attachment_count(&component.hull, &opening, projection)? == 0 {
            return None;
        }
        for point in &opening {
            if convex_polygon_location(point, &component.hull, projection)?
                == ConvexPolygonLocation::Outside
            {
                return None;
            }
        }
        removed_openings.push(opening);
    }
    if !saw_multi_opening_consumed_group
        || holes
            .iter()
            .any(|hole| !consumed_holes.contains(&hole.right_index))
    {
        return None;
    }

    let (removed_openings, mut cut_polygons) =
        materialize_side_cutter_point_touch_removed_openings_core(
            component,
            &removed_openings,
            label,
        )?;
    certify_removed_openings_collectively_split_source_component(
        &component.hull,
        &removed_openings,
        projection,
    )?;
    for polygon in &mut cut_polygons {
        orient_polygon_ccw(polygon, projection)?;
        validate_projected_simple_loop(polygon, projection, label).ok()?;
    }
    validate_simple_component_loops_disjoint_allowing_vertex_point_touches(
        &cut_polygons,
        projection,
        label,
    )
    .ok()?;
    sort_polygons_for_replay(&mut cut_polygons, projection);
    Some(cut_polygons)
}

/// Replay non-rectilinear side cutters as removed openings and one output loop.
///
/// The returned pair is `(removed_openings, output)`. Each removed opening is
/// either one clipped convex cutter or an exact retained union of a connected
/// cutter contact group; disconnected groups become independent openings.
/// Point-only coincidences inside a group are accepted only when positive
/// contacts already connect the same group and the removed boundary still
/// replays as one simple loop. The final output loop is accepted only after
/// exact simple-loop validation, nonconvexity, and exact area replay. This is
/// the reusable no-hole core for side-cutter-only differences and mixed
/// component/holed differences.
///
/// This is the retained-object discipline from Yap, "Towards Exact Geometric
/// Computation," *Computational Geometry* 7.1-2 (1997): a point coincidence is
/// retained evidence, but it is not a topological adjacency unless the
/// positive-contact graph and exact area replay already prove the object.
#[cfg(feature = "exact-triangulation")]
fn materialize_nonrectilinear_side_cutter_opening(
    component: &ConvexUnionComponent,
    cut_indices: &[usize],
    right_components: &[ConvexUnionComponent],
    label: &'static str,
) -> Option<(Vec<Vec<Point3>>, Vec<Point3>)> {
    if cut_indices.is_empty() {
        return None;
    }
    let projection = component.projection;
    let mut regions = Vec::with_capacity(cut_indices.len());
    let mut all_clipped_cutters_are_rectangles = true;
    for &right_index in cut_indices {
        let mut clipped = convex_polygon_intersection_boundary(
            &right_components[right_index].hull,
            &component.hull,
            projection,
        )?;
        if clipped.len() < 3 {
            return None;
        }
        orient_polygon_ccw(&mut clipped, projection)?;
        validate_projected_strictly_convex_loop(&clipped, projection, label).ok()?;
        all_clipped_cutters_are_rectangles &=
            projected_axis_aligned_rectangle(&clipped, projection).is_some();
        regions.push(RemovedRegionCandidate {
            right_index,
            is_cutter: true,
            region: clipped,
        });
    }
    if all_clipped_cutters_are_rectangles {
        return None;
    }

    let groups = removed_region_contact_groups_allowing_incidental_points(&regions, projection)?;
    let mut removed_openings = Vec::with_capacity(groups.len());
    for group in &groups {
        let mut opening = if group.len() == 1 {
            regions[group[0]].region.clone()
        } else {
            materialize_removed_region_group_polygon_allowing_incidental_points(
                &regions, group, projection,
            )?
        };
        orient_polygon_ccw(&mut opening, projection)?;
        opening = simplify_projected_polygon(opening, projection);
        validate_projected_simple_loop(&opening, projection, label).ok()?;
        removed_openings.push(opening);
    }
    if removed_openings.is_empty() {
        return None;
    }

    let mut opening = if removed_openings.len() == 1 {
        let removed_union = &removed_openings[0];
        if let Some(rectangle) = projected_axis_aligned_rectangle(&component.hull, projection) {
            side_opened_difference_polygon(&rectangle, removed_union, projection)?
        } else {
            convex_side_opened_difference_polygon(&component.hull, removed_union, projection)?
        }
    } else {
        multi_side_opened_difference_polygon(
            &component.hull,
            &removed_openings,
            projection,
            "coplanar component-holed multi-side-cutter opening",
        )?
    };
    orient_polygon_ccw(&mut opening, projection)?;
    opening = simplify_projected_polygon(opening, projection);
    validate_projected_simple_loop(&opening, projection, label).ok()?;
    if validate_projected_strictly_convex_loop(&opening, projection, label).is_ok() {
        return None;
    }

    let component_area = projected_area2_abs(&component.hull, projection)?;
    let opening_area = projected_area2_abs(&opening, projection)?;
    let mut removed_area = ExactReal::from(0);
    for removed_opening in &removed_openings {
        removed_area = add(
            &removed_area,
            &projected_area2_abs(removed_opening, projection)?,
        );
    }
    if compare_reals(&add(&opening_area, &removed_area), &component_area).value()
        != Some(Ordering::Equal)
    {
        return None;
    }
    Some((removed_openings, opening))
}

/// Replay side-attached cutters that split one source component into loops.
///
/// [`materialize_nonrectilinear_side_cutter_opening`] accepts the common bay
/// case where removed material opens one retained simple loop. This helper is
/// the multi-component sibling: clipped cutter groups are replayed as exact
/// removed loops, each removed loop must carry positive-length contact with
/// the convex source boundary, and the retained boundary fragments are
/// stitched into two or more disjoint simple loops. The final area equation is
/// checked exactly:
/// `area(source) = sum(area(output_i)) + sum(area(removed_j))`.
///
/// This is a bounded planar-cell promotion, not a tolerance polygon clip. The
/// retained fragments are precisely the outside portions of the convex source
/// boundary and the reversed inside portions of removed cutter groups, in the
/// Weiler-Atherton retained-boundary style; see Weiler and Atherton, "Hidden
/// Surface Removal Using Polygon Area Sorting," *SIGGRAPH Computer Graphics*
/// 11.2 (1977). The reason the helper demands exact attachment, simplicity,
/// disjointness, and area replay is Yap's object-level requirement from
/// "Towards Exact Geometric Computation," *Computational Geometry* 7.1-2
/// (1997): the shortcut may widen only when retained combinatorial facts, not
/// sampled witnesses, determine the output topology.
#[cfg(feature = "exact-triangulation")]
fn materialize_side_cutter_multi_component_difference(
    component: &ConvexUnionComponent,
    cut_indices: &[usize],
    right_components: &[ConvexUnionComponent],
    label: &'static str,
) -> Option<Vec<Vec<Point3>>> {
    materialize_side_cutter_multi_component_difference_core(
        component,
        cut_indices,
        right_components,
        label,
    )
    .map(|(_, polygons)| polygons)
}

/// Replay a no-hole side-cutter split that consumes strict interior rings.
///
/// This is the no-hole counterpart to
/// [`materialize_side_cutter_multi_component_holed_difference`]. The retained
/// geometry is still the exact multi-output side-cutter split. That split may
/// be caused by several side-attached removed openings, or by one
/// side-to-side non-rectilinear cutter whose clipped removed loop separates
/// the source. Every strict interior right component must be wholly owned by
/// exactly one removed side opening. If any ring would survive in a retained
/// output, touch a split boundary, or have multiple possible owners, this
/// helper rejects so the component/holed or later planar-cell materializer
/// owns that topology.
///
/// The distinction is Yap's retained-object discipline from "Towards Exact
/// Geometric Computation," *Computational Geometry* 7.1-2 (1997): removing a
/// source ring is a certified topology change, not a side effect of polygon
/// clipping. The retained split loops are the Weiler-Atherton fragment replay
/// described in Weiler and Atherton, "Hidden Surface Removal Using Polygon
/// Area Sorting," *SIGGRAPH Computer Graphics* 11.2 (1977).
#[cfg(feature = "exact-triangulation")]
fn materialize_side_cutter_multi_component_difference_consuming_holes(
    component: &ConvexUnionComponent,
    cut_indices: &[usize],
    holes: &[ComponentHoleCandidate],
    right_components: &[ConvexUnionComponent],
    label: &'static str,
) -> Option<Vec<Vec<Point3>>> {
    if cut_indices.is_empty() || holes.is_empty() {
        return None;
    }
    let projection = component.projection;
    let (removed_openings, polygons) = materialize_side_cutter_multi_component_difference_core(
        component,
        cut_indices,
        right_components,
        label,
    )?;
    for hole in holes {
        if !hole_strictly_consumed_by_one_removed_opening(
            &hole.ring,
            &removed_openings,
            projection,
        )? {
            return None;
        }
    }
    Some(polygons)
}

#[cfg(feature = "exact-triangulation")]
fn materialize_side_cutter_multi_component_difference_core(
    component: &ConvexUnionComponent,
    cut_indices: &[usize],
    right_components: &[ConvexUnionComponent],
    label: &'static str,
) -> Option<(Vec<Vec<Point3>>, Vec<Vec<Point3>>)> {
    if cut_indices.is_empty() {
        return None;
    }
    let projection = component.projection;
    let mut regions = Vec::with_capacity(cut_indices.len());
    let mut all_clipped_cutters_are_rectangles = true;
    for &right_index in cut_indices {
        let mut clipped = convex_polygon_intersection_boundary(
            &right_components[right_index].hull,
            &component.hull,
            projection,
        )?;
        if clipped.len() < 3 {
            return None;
        }
        orient_polygon_ccw(&mut clipped, projection)?;
        validate_projected_strictly_convex_loop(&clipped, projection, label).ok()?;
        all_clipped_cutters_are_rectangles &=
            projected_axis_aligned_rectangle(&clipped, projection).is_some();
        regions.push(RemovedRegionCandidate {
            right_index,
            is_cutter: true,
            region: clipped,
        });
    }
    if all_clipped_cutters_are_rectangles {
        return None;
    }

    let groups = removed_region_contact_groups_allowing_incidental_points(&regions, projection)?;
    let mut removed_openings = Vec::with_capacity(groups.len());
    for group in &groups {
        let mut opening = if group.len() == 1 {
            regions[group[0]].region.clone()
        } else {
            materialize_removed_region_group_polygon_allowing_incidental_points(
                &regions, group, projection,
            )?
        };
        orient_polygon_ccw(&mut opening, projection)?;
        opening = simplify_projected_polygon(opening, projection);
        validate_projected_simple_loop(&opening, projection, label).ok()?;
        if convex_boundary_attachment_count(&component.hull, &opening, projection)? == 0 {
            return None;
        }
        for point in &opening {
            if convex_polygon_location(point, &component.hull, projection)?
                == ConvexPolygonLocation::Outside
            {
                return None;
            }
        }
        removed_openings.push(opening);
    }
    if removed_openings.is_empty() {
        return None;
    }
    if validate_simple_component_loops_disjoint(&removed_openings, projection, label).is_err() {
        return None;
    }

    let mut fragments = Vec::new();
    collect_outer_difference_fragments(
        &component.hull,
        &removed_openings,
        projection,
        &mut fragments,
    )?;
    for index in 0..removed_openings.len() {
        collect_removed_difference_fragments(
            index,
            &component.hull,
            &removed_openings,
            projection,
            &mut fragments,
        )?;
    }
    let mut polygons = stitch_disjoint_simple_loops(fragments, projection)?;
    if polygons.len() < 2 {
        return None;
    }
    let mut output_area = ExactReal::from(0);
    for polygon in &mut polygons {
        orient_polygon_ccw(polygon, projection)?;
        *polygon = simplify_projected_polygon(polygon.clone(), projection);
        if validate_projected_simple_loop(polygon, projection, label).is_err() {
            return None;
        }
        output_area = add(&output_area, &projected_area2_abs(polygon, projection)?);
    }
    if validate_simple_component_loops_disjoint(&polygons, projection, label).is_err() {
        return None;
    }

    let mut removed_area = ExactReal::from(0);
    for opening in &removed_openings {
        removed_area = add(&removed_area, &projected_area2_abs(opening, projection)?);
    }
    let component_area = projected_area2_abs(&component.hull, projection)?;
    if compare_reals(&add(&output_area, &removed_area), &component_area).value()
        != Some(Ordering::Equal)
    {
        return None;
    }
    Some((removed_openings, polygons))
}

/// Replay a no-hole side-cutter split with exact vertex branch contacts.
///
/// This is deliberately separate from
/// [`materialize_side_cutter_multi_component_difference_core`]. Ordinary
/// multi-difference loops must be disjoint; this helper accepts only the
/// bounded case where clipped removed openings have at least one point-only
/// contact, the contact does not provide connectivity for the removed-region
/// groups, and the retained components validate with exact shared vertices.
/// The same Weiler-Atherton retained-fragment construction and exact area
/// replay are used, but the loop-disjointness predicate is the branch-aware
/// one. That keeps the topology promotion explicit in Yap's sense instead of
/// weakening the existing multi-difference contract.
#[cfg(feature = "exact-triangulation")]
fn materialize_side_cutter_point_touch_difference_core(
    component: &ConvexUnionComponent,
    cut_indices: &[usize],
    right_components: &[ConvexUnionComponent],
    label: &'static str,
) -> Option<(Vec<Vec<Point3>>, Vec<Vec<Point3>>)> {
    if cut_indices.len() < 2 {
        return None;
    }
    let projection = component.projection;
    let mut regions = Vec::with_capacity(cut_indices.len());
    let mut all_clipped_cutters_are_rectangles = true;
    for &right_index in cut_indices {
        let mut clipped = convex_polygon_intersection_boundary(
            &right_components[right_index].hull,
            &component.hull,
            projection,
        )?;
        if clipped.len() < 3 {
            return None;
        }
        orient_polygon_ccw(&mut clipped, projection)?;
        validate_projected_strictly_convex_loop(&clipped, projection, label).ok()?;
        all_clipped_cutters_are_rectangles &=
            projected_axis_aligned_rectangle(&clipped, projection).is_some();
        regions.push(RemovedRegionCandidate {
            right_index,
            is_cutter: true,
            region: clipped,
        });
    }
    if all_clipped_cutters_are_rectangles {
        return None;
    }

    let (groups, saw_point_contact) =
        removed_region_contact_groups_allowing_branch_points(&regions, projection)?;
    if !saw_point_contact {
        return None;
    }
    let mut removed_openings = Vec::with_capacity(groups.len());
    for group in &groups {
        let mut opening = if group.len() == 1 {
            regions[group[0]].region.clone()
        } else {
            materialize_removed_region_group_polygon_allowing_incidental_points(
                &regions, group, projection,
            )?
        };
        orient_polygon_ccw(&mut opening, projection)?;
        opening = simplify_projected_polygon(opening, projection);
        validate_projected_simple_loop(&opening, projection, label).ok()?;
        if convex_boundary_attachment_count(&component.hull, &opening, projection)? == 0 {
            return None;
        }
        for point in &opening {
            if convex_polygon_location(point, &component.hull, projection)?
                == ConvexPolygonLocation::Outside
            {
                return None;
            }
        }
        removed_openings.push(opening);
    }
    if removed_openings.len() < 2 {
        return None;
    }
    validate_simple_component_loops_disjoint_allowing_vertex_point_touches(
        &removed_openings,
        projection,
        label,
    )
    .ok()?;

    let mut fragments = Vec::new();
    collect_outer_difference_fragments(
        &component.hull,
        &removed_openings,
        projection,
        &mut fragments,
    )?;
    for index in 0..removed_openings.len() {
        collect_removed_difference_fragments(
            index,
            &component.hull,
            &removed_openings,
            projection,
            &mut fragments,
        )?;
    }
    let mut polygons = stitch_branching_simple_loops(fragments, projection)?;
    if polygons.len() < 2 {
        return None;
    }
    let mut output_area = ExactReal::from(0);
    for polygon in &mut polygons {
        orient_polygon_ccw(polygon, projection)?;
        *polygon = simplify_projected_polygon(polygon.clone(), projection);
        validate_projected_simple_loop(polygon, projection, label).ok()?;
        output_area = add(&output_area, &projected_area2_abs(polygon, projection)?);
    }
    validate_simple_component_loops_disjoint_allowing_vertex_point_touches(
        &polygons, projection, label,
    )
    .ok()?;

    let mut removed_area = ExactReal::from(0);
    for opening in &removed_openings {
        removed_area = add(&removed_area, &projected_area2_abs(opening, projection)?);
    }
    let component_area = projected_area2_abs(&component.hull, projection)?;
    if compare_reals(&add(&output_area, &removed_area), &component_area).value()
        != Some(Ordering::Equal)
    {
        return None;
    }
    sort_polygons_for_replay(&mut polygons, projection);
    Some((removed_openings, polygons))
}

/// Replay a convex source after removed openings already own their topology.
///
/// [`materialize_side_cutter_point_touch_difference_core`] starts from convex
/// right components and clips them against the source. The straddling-hole
/// path has to union a strict hole into one clipped opening before the branch
/// subtraction is meaningful, so this helper starts one step later: its input
/// openings are already exact removed objects. Positive-dimensional contacts
/// are merged, point-only contacts remain branch facts, and the final retained
/// loops must satisfy the exact area equation
/// `source = retained + removed`.
///
/// This is the same retained-object split advocated by Yap, "Towards Exact
/// Geometric Computation," *Computational Geometry* 7.1-2 (1997): predicate
/// evidence names the removed objects first, then the boolean object is
/// emitted. Boundary fragments follow Weiler and Atherton, "Hidden Surface
/// Removal Using Polygon Area Sorting," *SIGGRAPH Computer Graphics* 11.2
/// (1977), and contacts are classified by the exact orientation predicates of
/// Guigue and Devillers, "Fast and Robust Triangle-Triangle Overlap Test
/// Using Orientation Predicates," *Journal of Graphics Tools* 8.1 (2003).
#[cfg(feature = "exact-triangulation")]
fn materialize_side_cutter_point_touch_removed_openings_core(
    component: &ConvexUnionComponent,
    removed_openings: &[Vec<Point3>],
    label: &'static str,
) -> Option<(Vec<Vec<Point3>>, Vec<Vec<Point3>>)> {
    if removed_openings.len() < 2 {
        return None;
    }
    let projection = component.projection;
    let mut openings = removed_openings.to_vec();
    for opening in &mut openings {
        orient_polygon_ccw(opening, projection)?;
        *opening = simplify_projected_polygon(opening.clone(), projection);
        validate_projected_simple_loop(opening, projection, label).ok()?;
        if convex_boundary_attachment_count(&component.hull, opening, projection)? == 0 {
            return None;
        }
        for point in opening {
            if convex_polygon_location(point, &component.hull, projection)?
                == ConvexPolygonLocation::Outside
            {
                return None;
            }
        }
    }

    let (mut openings, saw_point_contact) =
        merge_connected_simple_removed_openings_allowing_branches(
            &openings,
            projection,
            "coplanar point-touch removed-opening union",
        )?;
    if !saw_point_contact || openings.len() < 2 {
        return None;
    }
    for opening in &mut openings {
        orient_polygon_ccw(opening, projection)?;
        *opening = simplify_projected_polygon(opening.clone(), projection);
        validate_projected_simple_loop(opening, projection, label).ok()?;
        if convex_boundary_attachment_count(&component.hull, opening, projection)? == 0 {
            return None;
        }
        for point in opening {
            if convex_polygon_location(point, &component.hull, projection)?
                == ConvexPolygonLocation::Outside
            {
                return None;
            }
        }
    }
    validate_simple_component_loops_disjoint_allowing_vertex_point_touches(
        &openings, projection, label,
    )
    .ok()?;

    let mut fragments = Vec::new();
    collect_outer_difference_fragments(&component.hull, &openings, projection, &mut fragments)?;
    for index in 0..openings.len() {
        collect_removed_difference_fragments(
            index,
            &component.hull,
            &openings,
            projection,
            &mut fragments,
        )?;
    }
    let mut polygons = stitch_branching_simple_loops(fragments, projection)?;
    if polygons.len() < 2 {
        return None;
    }
    let mut output_area = ExactReal::from(0);
    for polygon in &mut polygons {
        orient_polygon_ccw(polygon, projection)?;
        *polygon = simplify_projected_polygon(polygon.clone(), projection);
        validate_projected_simple_loop(polygon, projection, label).ok()?;
        output_area = add(&output_area, &projected_area2_abs(polygon, projection)?);
    }
    validate_simple_component_loops_disjoint_allowing_vertex_point_touches(
        &polygons, projection, label,
    )
    .ok()?;

    let mut removed_area = ExactReal::from(0);
    for opening in &openings {
        removed_area = add(&removed_area, &projected_area2_abs(opening, projection)?);
    }
    let component_area = projected_area2_abs(&component.hull, projection)?;
    if compare_reals(&add(&output_area, &removed_area), &component_area).value()
        != Some(Ordering::Equal)
    {
        return None;
    }
    sort_polygons_for_replay(&mut polygons, projection);
    Some((openings, polygons))
}

/// Count positive-length retained contacts between a removed loop and source.
///
/// A side-cutter split is admitted only when the removed loop owns exact
/// boundary contact with the source component. Point touches are deliberately
/// ignored: they are branch vertices in the planar subdivision and need their
/// own cell traversal. The segment relation is the same orientation-predicate
/// classifier used throughout the module, following Guigue and Devillers,
/// "Fast and Robust Triangle-Triangle Overlap Test Using Orientation
/// Predicates," *Journal of Graphics Tools* 8.1 (2003).
#[cfg(feature = "exact-triangulation")]
fn convex_boundary_attachment_count(
    outer: &[Point3],
    removed: &[Point3],
    projection: CoplanarProjection,
) -> Option<usize> {
    Some(convex_boundary_attachment_edges(outer, removed, projection)?.len())
}

#[cfg(feature = "exact-triangulation")]
fn convex_boundary_attachment_edges(
    outer: &[Point3],
    removed: &[Point3],
    projection: CoplanarProjection,
) -> Option<Vec<usize>> {
    let mut attached_outer_edges = Vec::new();
    for outer_edge in 0..outer.len() {
        let outer_start = project_point(&outer[outer_edge], projection);
        let outer_end = project_point(&outer[(outer_edge + 1) % outer.len()], projection);
        for removed_edge in 0..removed.len() {
            let removed_start = project_point(&removed[removed_edge], projection);
            let removed_end =
                project_point(&removed[(removed_edge + 1) % removed.len()], projection);
            match classify_segment_intersection(
                &outer_start,
                &outer_end,
                &removed_start,
                &removed_end,
            )
            .value()?
            {
                SegmentIntersection::CollinearOverlap | SegmentIntersection::Identical => {
                    if !attached_outer_edges.contains(&outer_edge) {
                        attached_outer_edges.push(outer_edge);
                    }
                }
                SegmentIntersection::Disjoint
                | SegmentIntersection::EndpointTouch
                | SegmentIntersection::Proper => {}
            }
        }
    }
    Some(attached_outer_edges)
}

/// Certify that retained replay is a source-splitting removed topology.
///
/// Multi-output cutter/hole replay is stronger than an ordinary side opening:
/// at least one exact removed loop must own positive-length attachment to two
/// or more source sides. This admits both independent side-to-side barriers
/// and the higher-order four-sided branch-group fixture, while rejecting a
/// speculative split caused only by point contacts or interior stitching. The
/// policy is deliberately predicate-first in Yap's sense from "Towards Exact
/// Geometric Computation," *Computational Geometry* 7.1-2 (1997): a retained
/// multi-component object is exposed only after exact side ownership has been
/// proved, with the actual rings still validated by the Weiler-Atherton
/// retained-fragment replay in [`multi_side_opened_difference_polygons`].
#[cfg(feature = "exact-triangulation")]
fn certify_removed_openings_split_source_component(
    outer: &[Point3],
    removed_openings: &[Vec<Point3>],
    projection: CoplanarProjection,
) -> Option<()> {
    let mut max_attachment_count = 0usize;
    for opening in removed_openings {
        max_attachment_count = max_attachment_count.max(convex_boundary_attachment_count(
            outer, opening, projection,
        )?);
    }
    if max_attachment_count < 2 {
        return None;
    }
    Some(())
}

/// Certify that branch openings split a source by owning several sides.
///
/// A point-branch cutter graph can contain several simple removed openings
/// rather than one merged side-to-side opening. Accepting any point contact
/// would mistake two same-side bays for a source split. This gate therefore
/// collects exact positive-length source-boundary attachments across all
/// branch openings and requires at least two distinct source edges before a
/// component-holed branch artifact can be emitted. The distinction follows
/// Yap's retained-object boundary from "Towards Exact Geometric Computation,"
/// *Computational Geometry* 7.1-2 (1997): branch topology is promoted only
/// after exact source-side ownership names the split.
#[cfg(feature = "exact-triangulation")]
fn certify_removed_openings_collectively_split_source_component(
    outer: &[Point3],
    removed_openings: &[Vec<Point3>],
    projection: CoplanarProjection,
) -> Option<()> {
    let mut attached_outer_edges = Vec::new();
    for opening in removed_openings {
        for edge in convex_boundary_attachment_edges(outer, opening, projection)? {
            if !attached_outer_edges.contains(&edge) {
                attached_outer_edges.push(edge);
            }
        }
    }
    if attached_outer_edges.len() < 2 {
        return None;
    }
    Some(())
}

/// Return whether a strict hole is wholly removed by one side opening.
///
/// This is an ownership predicate, not a polygon clipper. A hole may be
/// omitted from the retained component only when exact simple-polygon
/// containment proves the whole ring is strictly inside one removed opening.
/// Zero or two owner openings are rejected so ambiguous ownership and
/// branch-point subdivision stay explicit. This is the retained-object
/// discipline Yap argues for in "Towards Exact Geometric Computation,"
/// *Computational Geometry* 7.1-2 (1997): the output topology changes only
/// when exact source facts identify the owner of the removed ring.
#[cfg(feature = "exact-triangulation")]
fn hole_strictly_consumed_by_one_removed_opening(
    hole: &[Point3],
    removed_openings: &[Vec<Point3>],
    projection: CoplanarProjection,
) -> Option<bool> {
    let mut owner_count = 0usize;
    for removed_opening in removed_openings {
        if polygon_strictly_inside_simple_polygon(hole, removed_opening, projection)? {
            owner_count += 1;
            if owner_count > 1 {
                return Some(false);
            }
        }
    }
    Some(owner_count == 1)
}

/// Assign strict holes after side-cutter openings have been replayed.
///
/// A retained hole must be strictly inside the final opened component. A
/// consumed hole must be strictly inside one removed opening. Anything else is
/// an unowned hole: usually a straddling ring, a boundary contact, or a branch
/// case. Those remain outside this bounded certificate so a later planar-cell
/// materializer can carry the exact split topology explicitly.
#[cfg(feature = "exact-triangulation")]
fn assign_holes_to_connected_multi_cutter_opening(
    holes: &[ComponentHoleCandidate],
    opening: &[Point3],
    removed_openings: &[Vec<Point3>],
    projection: CoplanarProjection,
) -> Option<Vec<Vec<Point3>>> {
    let mut retained_holes = Vec::with_capacity(holes.len());
    for hole in holes {
        if polygon_strictly_inside_simple_polygon(&hole.ring, opening, projection)? {
            retained_holes.push(hole.ring.clone());
        } else if !hole_strictly_consumed_by_one_removed_opening(
            &hole.ring,
            removed_openings,
            projection,
        )? {
            return None;
        }
    }
    if retained_holes.is_empty() {
        return None;
    }
    sort_polygons_for_replay(&mut retained_holes, projection);
    Some(retained_holes)
}

/// Replay rectangular mixed multi-cutter/holed remnants through exact cells.
///
/// This is the component-holed counterpart to the simple-loop rectangular
/// multi-cutter bridge. When a convex source component is cut by several exact
/// rectangular components and also contains strict rectangular holes, the
/// combined removed set is an orthogonal cell arrangement. We delegate topology
/// to `orthogonal_surface`, then import only the retained simple outer rings
/// and hole rings into [`CoplanarConvexComponentHoledArrangement`]. The helper
/// rejects non-rectilinear inputs, missing retained holes, and one-cutter cases
/// so those remain on the smaller convex-difference and cutter/hole-contact
/// certificates.
///
/// The promotion rule is Yap's retained exact-object discipline from "Towards
/// Exact Geometric Computation," *Computational Geometry* 7.1-2 (1997): the
/// component-holed shortcut may widen only when exact cell occupancy carries
/// the topology. The rectilinear subdivision itself follows de Berg, Cheong,
/// van Kreveld, and Overmars, *Computational Geometry: Algorithms and
/// Applications*, 3rd ed. (2008), Chapter 2.
#[cfg(feature = "exact-triangulation")]
fn materialize_rectangle_multi_cutter_component_holed_cell_difference(
    component: &ConvexUnionComponent,
    cutter_indices: &[usize],
    holes: &[ComponentHoleCandidate],
    right_components: &[ConvexUnionComponent],
) -> Option<Vec<CoplanarConvexHoledComponent>> {
    if cutter_indices.len() < 2 || holes.is_empty() {
        return None;
    }
    projected_axis_aligned_rectangle(&component.hull, component.projection)?;
    if !cutter_indices.iter().all(|&index| {
        projected_axis_aligned_rectangle(&right_components[index].hull, component.projection)
            .is_some()
    }) || !holes.iter().all(|hole| {
        projected_axis_aligned_rectangle(
            &right_components[hole.right_index].hull,
            component.projection,
        )
        .is_some()
    }) {
        return None;
    }

    let mut removal_meshes = Vec::with_capacity(cutter_indices.len() + holes.len());
    removal_meshes.extend(
        cutter_indices
            .iter()
            .map(|&index| &right_components[index].mesh),
    );
    removal_meshes.extend(
        holes
            .iter()
            .map(|hole| &right_components[hole.right_index].mesh),
    );
    let cutters = merge_component_meshes(
        removal_meshes,
        "exact coplanar rectangular component-holed multi-cutter source",
    )?;
    let arrangement = super::orthogonal_surface::arrange_coplanar_orthogonal_surface_difference(
        &component.mesh,
        &cutters,
    )?;
    let mut components = arrangement
        .components
        .into_iter()
        .map(|component| {
            let mut outer = component.outer;
            orient_polygon_ccw(&mut outer, arrangement.projection)?;
            let mut retained_holes = component.holes;
            for hole in &mut retained_holes {
                orient_polygon_cw(hole, arrangement.projection)?;
            }
            sort_polygons_for_replay(&mut retained_holes, arrangement.projection);
            Some(CoplanarConvexHoledComponent {
                outer,
                holes: retained_holes,
            })
        })
        .collect::<Option<Vec<_>>>()?;
    if components.is_empty() {
        return None;
    }
    sort_components_for_replay(&mut components, arrangement.projection);
    let retained_hole_count = components
        .iter()
        .map(|component| component.holes.len())
        .sum::<usize>();
    if retained_hole_count != holes.len()
        || !holes.iter().all(|candidate| {
            components.iter().any(|component| {
                component
                    .holes
                    .iter()
                    .any(|retained| polygons_equal(&candidate.ring, retained))
            })
        })
    {
        return None;
    }
    Some(components)
}

/// Assign retained hole rings to exact cut remnants.
///
/// A component that mixes holes with one partial cutter is still a bounded
/// arrangement: the cut itself is certified by the convex difference helper,
/// then each hole must be strictly inside exactly one emitted remnant. Remnant
/// loops may be nonconvex simple loops, so containment is checked by exact
/// retained-edge rejection plus exact earcut coverage rather than by convex
/// half-space signs. This check is the local substitute for a full planar
/// subdivision. Following Yap, "Towards Exact Geometric Computation,"
/// *Computational Geometry* 7.1-2 (1997), ambiguous boundary contact or
/// partial hole/remnant overlap returns `None` rather than inventing topology
/// from an approximate sample point. The triangulated containment probe uses
/// Held, "FIST: Fast Industrial-Strength Triangulation of Polygons,"
/// *Algorithmica* 30 (2001), through `hypertri`'s exact earcut adapter.
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
            if polygon_strictly_inside_simple_polygon(hole, polygon, projection)? {
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

/// Assign holes to side-cutter split outputs or consume them by openings.
///
/// This is the multi-output sibling of
/// [`assign_holes_to_connected_multi_cutter_opening`]. A source hole is
/// retained only when exact simple-polygon containment proves it lies strictly
/// inside one emitted retained loop. If it is not retained, it may be omitted
/// only when [`hole_strictly_consumed_by_one_removed_opening`] proves exactly
/// one removed side opening owns the whole ring. Yap's "Towards Exact
/// Geometric Computation," *Computational Geometry* 7.1-2 (1997), is the
/// governing rule here: the topology of a hole cannot be inferred from a
/// representative sample or floating tolerance, so every unowned or multiply
/// owned ring rejects the shortcut and waits for a full planar-cell
/// materializer.
#[cfg(feature = "exact-triangulation")]
fn assign_holes_to_side_cutter_split_outputs(
    holes: &[Vec<Point3>],
    cut_polygons: &[Vec<Point3>],
    removed_openings: &[Vec<Point3>],
    projection: CoplanarProjection,
) -> Option<Vec<Vec<Vec<Point3>>>> {
    let mut holes_by_cut = vec![Vec::new(); cut_polygons.len()];
    for hole in holes {
        let mut owner = None;
        for (index, polygon) in cut_polygons.iter().enumerate() {
            if polygon_strictly_inside_simple_polygon(hole, polygon, projection)? {
                if owner.is_some() {
                    return None;
                }
                owner = Some(index);
            }
        }
        if let Some(owner) = owner {
            holes_by_cut[owner].push(hole.clone());
        } else if !hole_strictly_consumed_by_one_removed_opening(
            hole,
            removed_openings,
            projection,
        )? {
            return None;
        }
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

/// Certify strict containment of one simple ring inside another simple ring.
///
/// This is the nonconvex counterpart to
/// [`polygon_strictly_inside_convex_polygon`]. It first rejects any retained
/// edge contact using the exact orientation-predicate segment classifier of
/// Guigue and Devillers, "Fast and Robust Triangle-Triangle Overlap Test Using
/// Orientation Predicates," *Journal of Graphics Tools* 8.1 (2003). It then
/// classifies every inner vertex against an exact earcut triangulation of the
/// outer ring, using Held, "FIST: Fast Industrial-Strength Triangulation of
/// Polygons," *Algorithmica* 30 (2001). Yap's "Towards Exact Geometric
/// Computation," *Computational Geometry* 7.1-2 (1997), is the reason the
/// routine returns `None` on undecided topology instead of accepting a sampled
/// representative point.
#[cfg(feature = "exact-triangulation")]
fn polygon_strictly_inside_simple_polygon(
    inner: &[Point3],
    outer: &[Point3],
    projection: CoplanarProjection,
) -> Option<bool> {
    if inner.len() < 3 || outer.len() < 3 {
        return Some(false);
    }
    if rings_have_any_edge_contact(inner, outer, projection)? {
        return Some(false);
    }
    inner
        .iter()
        .map(|point| simple_polygon_location(point, outer, projection))
        .try_fold(true, |all_inside, location| {
            Some(all_inside && location? == ConvexPolygonLocation::Inside)
        })
}

#[cfg(feature = "exact-triangulation")]
fn rings_have_any_edge_contact(
    left: &[Point3],
    right: &[Point3],
    projection: CoplanarProjection,
) -> Option<bool> {
    for left_edge in 0..left.len() {
        let left_start = project_point(&left[left_edge], projection);
        let left_end = project_point(&left[(left_edge + 1) % left.len()], projection);
        for right_edge in 0..right.len() {
            let right_start = project_point(&right[right_edge], projection);
            let right_end = project_point(&right[(right_edge + 1) % right.len()], projection);
            match classify_segment_intersection(&left_start, &left_end, &right_start, &right_end)
                .value()?
            {
                SegmentIntersection::Disjoint => {}
                SegmentIntersection::EndpointTouch
                | SegmentIntersection::Proper
                | SegmentIntersection::CollinearOverlap
                | SegmentIntersection::Identical => return Some(true),
            }
        }
    }
    Some(false)
}

#[cfg(feature = "exact-triangulation")]
fn simple_polygon_location(
    point: &Point3,
    polygon: &[Point3],
    projection: CoplanarProjection,
) -> Option<ConvexPolygonLocation> {
    if polygon.len() < 3 {
        return None;
    }
    let query = project_point(point, projection);
    for edge in 0..polygon.len() {
        let start = project_point(&polygon[edge], projection);
        let end = project_point(&polygon[(edge + 1) % polygon.len()], projection);
        if point_on_segment(&start, &end, &query).value() == Some(true) {
            return Some(ConvexPolygonLocation::Boundary);
        }
    }

    let vertices2 = polygon
        .iter()
        .map(|point| project_for_hypertri(point, projection))
        .collect::<Vec<_>>();
    let indices = hypertri::earcut(&vertices2, &[]).ok()?;
    for triangle in indices.chunks_exact(3) {
        let cell = vec![
            polygon[triangle[0]].clone(),
            polygon[triangle[1]].clone(),
            polygon[triangle[2]].clone(),
        ];
        match point_in_projected_triangle(point, &cell, projection)? {
            TriangleLocation::Inside | TriangleLocation::OnEdge | TriangleLocation::OnVertex => {
                return Some(ConvexPolygonLocation::Inside);
            }
            TriangleLocation::Outside | TriangleLocation::Degenerate => {}
        }
    }
    Some(ConvexPolygonLocation::Outside)
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

/// Triangulate simple loops while preserving every retained boundary vertex.
///
/// `hypertri`'s FIST-style earcut handoff is the preferred triangulator for
/// nonconvex loops, following Held, "FIST: Fast Industrial-Strength
/// Triangulation of Polygons," *Algorithmica* 30 (2001). Some point-touch
/// branch certificates intentionally introduce a collinear vertex on a source
/// edge; a triangulator may omit that coordinate from triangles while still
/// covering the area. Yap's "Towards Exact Geometric Computation,"
/// *Computational Geometry* 7.1-2 (1997), requires us to retain that branch
/// vertex as combinatorial state, so this helper falls back to an exact
/// predicate ear clipper when the earcut mesh does not use all retained
/// vertices.
#[cfg(feature = "exact-triangulation")]
fn polygons_to_retained_simple_open_mesh_with_label(
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
        let mesh = polygon_to_retained_simple_open_mesh_with_label(polygon, projection, label)?;
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
fn polygon_to_retained_simple_open_mesh_with_label(
    polygon: &[Point3],
    projection: CoplanarProjection,
    label: &'static str,
) -> Option<ExactMesh> {
    if let Some(mesh) = polygon_to_earcut_open_mesh_with_label(polygon, projection, label) {
        if validate_mesh_uses_all_retained_vertices(
            &mesh,
            polygon.len(),
            label,
            "surface mesh leaves a retained branch vertex unused",
        )
        .is_ok()
        {
            return Some(mesh);
        }
    }

    let vertices = polygon
        .iter()
        .map(|point| ExactPoint3::new(point.x.clone(), point.y.clone(), point.z.clone()))
        .collect::<Vec<_>>();
    let triangles = retained_simple_polygon_ear_clip_triangles(polygon, projection)?;
    ExactMesh::new_with_policy(
        vertices,
        triangles,
        SourceProvenance::exact(label),
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .ok()
}

#[cfg(feature = "exact-triangulation")]
fn retained_simple_polygon_ear_clip_triangles(
    polygon: &[Point3],
    projection: CoplanarProjection,
) -> Option<Vec<Triangle>> {
    if polygon.len() < 3 {
        return None;
    }
    let mut ring = (0..polygon.len()).collect::<Vec<_>>();
    let mut triangles = Vec::with_capacity(polygon.len().saturating_sub(2));
    while ring.len() > 3 {
        let mut clipped = false;
        for cursor in 0..ring.len() {
            let prev = ring[(cursor + ring.len() - 1) % ring.len()];
            let curr = ring[cursor];
            let next = ring[(cursor + 1) % ring.len()];
            if !is_positive_projected_triangle(polygon, [prev, curr, next], projection)? {
                continue;
            }
            let ear = triangle_points(polygon, [prev, curr, next]);
            let mut blocked = false;
            for &candidate in &ring {
                if candidate == prev || candidate == curr || candidate == next {
                    continue;
                }
                match point_in_projected_triangle(&polygon[candidate], &ear, projection)? {
                    TriangleLocation::Outside | TriangleLocation::Degenerate => {}
                    TriangleLocation::Inside
                    | TriangleLocation::OnEdge
                    | TriangleLocation::OnVertex => {
                        blocked = true;
                        break;
                    }
                }
            }
            if blocked {
                continue;
            }
            triangles.push(Triangle([prev, curr, next]));
            ring.remove(cursor);
            clipped = true;
            break;
        }
        if !clipped {
            return None;
        }
    }
    let final_triangle = [ring[0], ring[1], ring[2]];
    if !is_positive_projected_triangle(polygon, final_triangle, projection)? {
        return None;
    }
    triangles.push(Triangle(final_triangle));
    Some(triangles)
}

#[cfg(feature = "exact-triangulation")]
fn is_positive_projected_triangle(
    polygon: &[Point3],
    triangle: [usize; 3],
    projection: CoplanarProjection,
) -> Option<bool> {
    let points = triangle_points(polygon, triangle);
    let area = projected_area2_signed(&points, projection)?;
    Some(compare_reals(&area, &ExactReal::from(0)).value() == Some(Ordering::Greater))
}

/// Triangulate weakly convex point-touch components without dropping splits.
///
/// Exact vertex-edge point-touch unions retain a new vertex on a source edge.
/// General earcut implementations may legally simplify that collinear point
/// away, but this artifact needs the split vertex as auditable branch state.
/// We therefore try fan roots until every emitted fan triangle has positive
/// exact projected area. This is still a bounded convex certificate, not a
/// general triangulator: the retained loop was produced from an exact convex
/// hull plus exact edge splits, and the fan is accepted only when exact area
/// predicates prove nondegenerate use of every retained vertex. This follows
/// Yap, "Towards Exact Geometric Computation," *Computational Geometry*
/// 7.1-2 (1997), by preserving the structural split point instead of relying
/// on downstream tolerance repair.
#[cfg(feature = "exact-triangulation")]
fn weak_convex_polygons_to_open_mesh_with_label(
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
        vertices.extend(
            polygon
                .iter()
                .map(|point| ExactPoint3::new(point.x.clone(), point.y.clone(), point.z.clone())),
        );
        let local = weak_convex_polygon_fan_triangles(polygon, projection)?;
        triangles.extend(local.into_iter().map(|triangle| {
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
fn weak_convex_polygon_fan_triangles(
    polygon: &[Point3],
    projection: CoplanarProjection,
) -> Option<Vec<Triangle>> {
    for root in 0..polygon.len() {
        let mut triangles = Vec::with_capacity(polygon.len().saturating_sub(2));
        let mut valid = true;
        for step in 1..polygon.len() - 1 {
            let a = root;
            let b = (root + step) % polygon.len();
            let c = (root + step + 1) % polygon.len();
            let cell = vec![polygon[a].clone(), polygon[b].clone(), polygon[c].clone()];
            let area = projected_area2_abs(&cell, projection)?;
            if compare_reals(&area, &ExactReal::from(0)).value() != Some(Ordering::Greater) {
                valid = false;
                break;
            }
            triangles.push(Triangle([a, b, c]));
        }
        if valid {
            return Some(triangles);
        }
    }
    None
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
    let earcut_mesh = match component.holes.len() {
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
    };
    earcut_mesh.or_else(|| component_holed_component_to_keyholed_open_mesh(component, projection))
}

/// Triangulate a one-hole retained component by opening it along a bridge.
///
/// A holed polygon can be reduced to a simple polygon by adding a visible
/// bridge between the outer ring and the hole. The construction is the same
/// keyhole idea used by Held, "FIST: Fast Industrial-Strength Triangulation
/// of Polygons," *Algorithmica* 30 (2001), but the bridge is selected with
/// exact segment and containment predicates and duplicate bridge endpoints are
/// mapped back to the retained rings. Yap, "Towards Exact Geometric
/// Computation," *Computational Geometry* 7.1-2 (1997), is the acceptance
/// rule: the bridge is not new boundary state, and the resulting mesh is
/// accepted only if retained-ring validation proves it is an interior edge.
#[cfg(feature = "exact-triangulation")]
fn component_holed_component_to_keyholed_open_mesh(
    component: &CoplanarConvexHoledComponent,
    projection: CoplanarProjection,
) -> Option<ExactMesh> {
    if component.holes.len() != 1 || component.outer.len() < 3 {
        return None;
    }
    let hole = component.holes.first()?;
    if hole.len() < 3 {
        return None;
    }
    for outer_index in 0..component.outer.len() {
        for hole_index in 0..hole.len() {
            if !component_holed_bridge_is_valid(
                &component.outer,
                hole,
                outer_index,
                hole_index,
                projection,
            )? {
                continue;
            }
            if let Some(mesh) = component_holed_keyhole_mesh_for_bridge(
                &component.outer,
                hole,
                outer_index,
                hole_index,
                projection,
            ) {
                return Some(mesh);
            }
        }
    }
    None
}

#[cfg(feature = "exact-triangulation")]
fn component_holed_keyhole_mesh_for_bridge(
    outer: &[Point3],
    hole: &[Point3],
    outer_index: usize,
    hole_index: usize,
    projection: CoplanarProjection,
) -> Option<ExactMesh> {
    let mut keyhole_points = Vec::with_capacity(outer.len() + hole.len() + 2);
    let mut index_map = Vec::with_capacity(outer.len() + hole.len() + 2);
    keyhole_points.push(outer[outer_index].clone());
    index_map.push(outer_index);
    keyhole_points.push(hole[hole_index].clone());
    index_map.push(outer.len() + hole_index);
    for step in 1..hole.len() {
        let index = (hole_index + step) % hole.len();
        keyhole_points.push(hole[index].clone());
        index_map.push(outer.len() + index);
    }
    keyhole_points.push(hole[hole_index].clone());
    index_map.push(outer.len() + hole_index);
    keyhole_points.push(outer[outer_index].clone());
    index_map.push(outer_index);
    for step in 1..outer.len() {
        let index = (outer_index + step) % outer.len();
        keyhole_points.push(outer[index].clone());
        index_map.push(index);
    }

    let vertices2 = keyhole_points
        .iter()
        .map(|point| project_for_hypertri(point, projection))
        .collect::<Vec<_>>();
    let indices = hypertri::earcut(&vertices2, &[]).ok()?;
    if indices.len() % 3 != 0 || indices.is_empty() {
        return None;
    }

    let points = outer.iter().chain(hole).cloned().collect::<Vec<_>>();
    let mut triangles = Vec::new();
    for chunk in indices.chunks_exact(3) {
        let mapped = [
            *index_map.get(chunk[0])?,
            *index_map.get(chunk[1])?,
            *index_map.get(chunk[2])?,
        ];
        if mapped[0] == mapped[1] || mapped[1] == mapped[2] || mapped[2] == mapped[0] {
            continue;
        }
        let cell = triangle_points(&points, mapped);
        let signed_area = projected_area2_signed(&cell, projection)?;
        match compare_reals(&signed_area, &ExactReal::from(0)).value()? {
            Ordering::Greater => triangles.push(Triangle(mapped)),
            Ordering::Less => triangles.push(Triangle([mapped[2], mapped[1], mapped[0]])),
            Ordering::Equal => {}
        }
    }
    if triangles.is_empty() {
        return None;
    }
    let vertices = points
        .iter()
        .map(|point| ExactPoint3::new(point.x.clone(), point.y.clone(), point.z.clone()))
        .collect::<Vec<_>>();
    ExactMesh::new_with_policy(
        vertices,
        triangles,
        SourceProvenance::exact("exact coplanar keyholed holed arrangement"),
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .ok()
}

#[cfg(feature = "exact-triangulation")]
fn component_holed_bridge_is_valid(
    outer: &[Point3],
    hole: &[Point3],
    outer_index: usize,
    hole_index: usize,
    projection: CoplanarProjection,
) -> Option<bool> {
    let a = outer.get(outer_index)?;
    let b = hole.get(hole_index)?;
    let midpoint = midpoint3(a, b);
    if simple_polygon_location(&midpoint, outer, projection)? != ConvexPolygonLocation::Inside {
        return Some(false);
    }
    if simple_polygon_location(&midpoint, hole, projection)? != ConvexPolygonLocation::Outside {
        return Some(false);
    }
    if !component_holed_bridge_respects_ring(a, b, outer, outer_index, projection)? {
        return Some(false);
    }
    if !component_holed_bridge_respects_ring(a, b, hole, hole_index, projection)? {
        return Some(false);
    }
    Some(true)
}

#[cfg(feature = "exact-triangulation")]
fn component_holed_bridge_respects_ring(
    a: &Point3,
    b: &Point3,
    ring: &[Point3],
    allowed_vertex: usize,
    projection: CoplanarProjection,
) -> Option<bool> {
    let start = project_point(a, projection);
    let end = project_point(b, projection);
    for edge in 0..ring.len() {
        let edge_start = project_point(&ring[edge], projection);
        let edge_end = project_point(&ring[(edge + 1) % ring.len()], projection);
        match classify_segment_intersection(&start, &end, &edge_start, &edge_end).value()? {
            SegmentIntersection::Disjoint => {}
            SegmentIntersection::EndpointTouch => {
                let incident = edge == allowed_vertex || (edge + 1) % ring.len() == allowed_vertex;
                if !incident {
                    return Some(false);
                }
            }
            SegmentIntersection::Proper
            | SegmentIntersection::CollinearOverlap
            | SegmentIntersection::Identical => return Some(false),
        }
    }
    Some(true)
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
    validate_multi_surface_output_with_loop_policy(projection, polygons, mesh, label, true, false)
}

#[cfg(feature = "exact-triangulation")]
fn validate_multi_simple_surface_output(
    projection: CoplanarProjection,
    polygons: &[Vec<Point3>],
    mesh: &ExactMesh,
    label: &'static str,
) -> Result<(), MeshError> {
    validate_multi_surface_output_with_loop_policy(projection, polygons, mesh, label, false, false)
}

#[cfg(feature = "exact-triangulation")]
fn validate_multi_surface_output_allowing_vertex_point_touches(
    projection: CoplanarProjection,
    polygons: &[Vec<Point3>],
    mesh: &ExactMesh,
    label: &'static str,
    require_strict_convex: bool,
) -> Result<(), MeshError> {
    validate_multi_surface_output_with_loop_policy(
        projection,
        polygons,
        mesh,
        label,
        require_strict_convex,
        true,
    )
}

#[cfg(feature = "exact-triangulation")]
fn validate_multi_surface_output_with_loop_policy(
    projection: CoplanarProjection,
    polygons: &[Vec<Point3>],
    mesh: &ExactMesh,
    label: &'static str,
    require_strict_convex: bool,
    allow_vertex_point_touches: bool,
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
                        if !allow_vertex_point_touches {
                            return Err(surface_validation_error(
                                label,
                                "component loops share an exact point",
                            ));
                        }
                    }
                }
            }
        }
    }
    if require_strict_convex {
        if allow_vertex_point_touches {
            validate_component_loops_disjoint_allowing_vertex_point_touches(
                polygons, projection, label,
            )?;
        } else {
            validate_component_loops_disjoint(polygons, projection, label)?;
        }
    } else {
        if allow_vertex_point_touches {
            validate_simple_component_loops_disjoint_allowing_vertex_point_touches(
                polygons, projection, label,
            )?;
        } else {
            validate_simple_component_loops_disjoint(polygons, projection, label)?;
        }
    }

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
    if allow_vertex_point_touches {
        validate_mesh_edges_respect_retained_rings_allowing_exact_endpoint_touches(
            mesh,
            projection,
            &component_ranges,
            label,
            "multi-component mesh edge crosses a retained component loop",
        )?;
    } else {
        validate_mesh_edges_respect_retained_rings(
            mesh,
            projection,
            &component_ranges,
            label,
            "multi-component mesh edge crosses a retained component loop",
        )?;
    }
    validate_mesh_uses_all_retained_vertices(
        mesh,
        expected_vertices,
        label,
        "multi-component mesh leaves a retained loop vertex unused",
    )?;
    if allow_vertex_point_touches {
        validate_mesh_boundary_matches_retained_rings_allowing_collinear_splits(
            mesh,
            projection,
            &component_ranges,
            label,
            "multi-component mesh boundary does not match retained component loops",
        )?;
    } else {
        validate_mesh_boundary_matches_retained_rings(
            mesh,
            &component_ranges,
            label,
            "multi-component mesh boundary does not match retained component loops",
        )?;
    }
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
                if !allow_vertex_point_touches {
                    return Err(surface_validation_error(
                        label,
                        "multi-component retained loops repeat an exact point",
                    ));
                }
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
            if !polygon_strictly_inside_simple_polygon(hole, &component.outer, projection)
                .ok_or_else(|| {
                    surface_validation_error(
                        label,
                        "component hole containment predicate was undecided",
                    )
                })?
            {
                return Err(surface_validation_error(
                    label,
                    "component hole must lie strictly inside its outer ring",
                ));
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
    validate_simple_component_loops_disjoint_allowing_vertex_point_touches(
        &outers, projection, label,
    )?;

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
    validate_mesh_edges_respect_retained_rings_allowing_exact_endpoint_touches(
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
    validate_mesh_boundary_matches_retained_rings_allowing_collinear_splits(
        mesh,
        projection,
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
    validate_mesh_edges_respect_retained_rings_with_policy(
        mesh,
        projection,
        retained_rings,
        label,
        message,
        false,
    )
}

#[cfg(feature = "exact-triangulation")]
fn validate_mesh_edges_respect_retained_rings_allowing_exact_endpoint_touches(
    mesh: &ExactMesh,
    projection: CoplanarProjection,
    retained_rings: &[core::ops::Range<usize>],
    label: &'static str,
    message: &'static str,
) -> Result<(), MeshError> {
    validate_mesh_edges_respect_retained_rings_with_policy(
        mesh,
        projection,
        retained_rings,
        label,
        message,
        true,
    )
}

fn validate_mesh_edges_respect_retained_rings_with_policy(
    mesh: &ExactMesh,
    projection: CoplanarProjection,
    retained_rings: &[core::ops::Range<usize>],
    label: &'static str,
    message: &'static str,
    allow_exact_endpoint_touches: bool,
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
                Some(SegmentIntersection::EndpointTouch)
                    if allow_exact_endpoint_touches
                        && segments_share_exact_endpoint(
                            &edge_start,
                            &edge_end,
                            &ring_start,
                            &ring_end,
                        ) => {}
                Some(SegmentIntersection::EndpointTouch)
                    if allow_exact_endpoint_touches
                        && mesh_edge_contains_retained_endpoint_touch(
                            mesh,
                            projection,
                            edge_a,
                            edge_b,
                            ring_a,
                            ring_b,
                            retained_rings,
                        ) => {}
                Some(SegmentIntersection::CollinearOverlap | SegmentIntersection::Identical)
                    if allow_exact_endpoint_touches
                        && edge_and_ring_edge_share_retained_component(
                            edge_a,
                            edge_b,
                            ring_a,
                            ring_b,
                            retained_rings,
                        ) => {}
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

fn edge_and_ring_edge_share_retained_component(
    edge_a: usize,
    edge_b: usize,
    ring_a: usize,
    ring_b: usize,
    retained_rings: &[core::ops::Range<usize>],
) -> bool {
    retained_rings.iter().any(|ring| {
        ring.contains(&edge_a)
            && ring.contains(&edge_b)
            && ring.contains(&ring_a)
            && ring.contains(&ring_b)
    })
}

fn mesh_edge_contains_retained_endpoint_touch(
    mesh: &ExactMesh,
    projection: CoplanarProjection,
    edge_a: usize,
    edge_b: usize,
    ring_a: usize,
    ring_b: usize,
    retained_rings: &[core::ops::Range<usize>],
) -> bool {
    let edge_start = mesh.vertices()[edge_a].to_hyperlimit_point();
    let edge_end = mesh.vertices()[edge_b].to_hyperlimit_point();
    let ring_start = mesh.vertices()[ring_a].to_hyperlimit_point();
    let ring_end = mesh.vertices()[ring_b].to_hyperlimit_point();
    retained_rings
        .iter()
        .filter(|ring| ring.contains(&edge_a) && ring.contains(&edge_b))
        .flat_map(|ring| ring.clone())
        .map(|index| mesh.vertices()[index].to_hyperlimit_point())
        .any(|candidate| {
            point_on_projected_segment(&edge_start, &edge_end, &candidate, projection)
                && (points_equal(&candidate, &ring_start) || points_equal(&candidate, &ring_end))
        })
}

/// Validate mesh boundary edges against retained rings with exact split points.
///
/// The point-touch union may add a retained vertex on a previously unsplit
/// convex source edge. Some triangulators legally emit the boundary as the
/// longer collinear segment while still using the split vertex in an incident
/// triangle. For this artifact only, we accept such a boundary edge when it
/// covers a same-component chain of retained collinear ring edges exactly.
/// This keeps the split point as retained Yap-style state while avoiding a
/// tolerance weld or a degenerate boundary triangle.
#[cfg(feature = "exact-triangulation")]
fn validate_mesh_boundary_matches_retained_rings_allowing_collinear_splits(
    mesh: &ExactMesh,
    projection: CoplanarProjection,
    retained_rings: &[core::ops::Range<usize>],
    label: &'static str,
    message: &'static str,
) -> Result<(), MeshError> {
    let expected = retained_ring_edges(retained_rings, label)?;
    let mut expected = expected
        .into_iter()
        .map(|(left, right)| canonical_edge(left, right))
        .collect::<Vec<_>>();
    expected.sort_unstable();
    expected.dedup();

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
    let actual = edge_counts
        .into_iter()
        .filter_map(|(edge, count)| (count == 1).then_some(edge))
        .collect::<Vec<_>>();

    let mut covered = Vec::new();
    for &(left, right) in &actual {
        if expected.binary_search(&canonical_edge(left, right)).is_ok() {
            covered.push(canonical_edge(left, right));
            continue;
        }
        let chain =
            retained_collinear_boundary_chain(mesh, projection, retained_rings, left, right)
                .ok_or_else(|| surface_validation_error(label, message))?;
        covered.extend(chain);
    }
    covered.sort_unstable();
    covered.dedup();
    if covered != expected {
        return Err(surface_validation_error(label, message));
    }
    Ok(())
}

#[cfg(feature = "exact-triangulation")]
fn retained_collinear_boundary_chain(
    mesh: &ExactMesh,
    projection: CoplanarProjection,
    retained_rings: &[core::ops::Range<usize>],
    left: usize,
    right: usize,
) -> Option<Vec<(usize, usize)>> {
    retained_collinear_boundary_chain_direction(mesh, projection, retained_rings, left, right)
        .or_else(|| {
            retained_collinear_boundary_chain_direction(
                mesh,
                projection,
                retained_rings,
                right,
                left,
            )
        })
}

#[cfg(feature = "exact-triangulation")]
fn retained_collinear_boundary_chain_direction(
    mesh: &ExactMesh,
    projection: CoplanarProjection,
    retained_rings: &[core::ops::Range<usize>],
    left: usize,
    right: usize,
) -> Option<Vec<(usize, usize)>> {
    let ring = retained_rings
        .iter()
        .find(|ring| ring.contains(&left) && ring.contains(&right))?;
    let len = ring.end.checked_sub(ring.start)?;
    let mut cursor = left.checked_sub(ring.start)?;
    let target = right.checked_sub(ring.start)?;
    let start = mesh.vertices()[left].to_hyperlimit_point();
    let end = mesh.vertices()[right].to_hyperlimit_point();
    let mut edges = Vec::new();
    for _ in 0..len {
        let next = (cursor + 1) % len;
        let vertex = ring.start + cursor;
        let next_vertex = ring.start + next;
        let point = mesh.vertices()[vertex].to_hyperlimit_point();
        let next_point = mesh.vertices()[next_vertex].to_hyperlimit_point();
        if !point_on_projected_segment(&start, &end, &point, projection)
            || !point_on_projected_segment(&start, &end, &next_point, projection)
        {
            return None;
        }
        edges.push(canonical_edge(vertex, next_vertex));
        if next == target {
            return Some(edges);
        }
        cursor = next;
    }
    None
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

/// Validate convex component loops that may meet at exact shared vertices.
///
/// This is used only by the point-touch union artifact. It intentionally does
/// not relax the ordinary multi-component invariant: exact shared retained
/// vertices are the only allowed cross-loop incidence. Vertex-edge contacts
/// must have been split into shared retained vertices before validation;
/// positive-length edge contact, crossings, and nesting are rejected with the
/// same orientation-predicate segment model cited above from Guigue and
/// Devillers (2003), keeping the branch case bounded in Yap's sense.
#[cfg(feature = "exact-triangulation")]
fn validate_component_loops_disjoint_allowing_vertex_point_touches(
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
                    Some(ConvexPolygonLocation::Inside) => {
                        return Err(surface_validation_error(
                            label,
                            "component loops overlap or nest",
                        ));
                    }
                    Some(ConvexPolygonLocation::Boundary) => {
                        if !polygon_has_exact_vertex(right, point) {
                            return Err(surface_validation_error(
                                label,
                                "component loops touch away from exact shared vertices",
                            ));
                        }
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
                    Some(ConvexPolygonLocation::Inside) => {
                        return Err(surface_validation_error(
                            label,
                            "component loops overlap or nest",
                        ));
                    }
                    Some(ConvexPolygonLocation::Boundary) => {
                        if !polygon_has_exact_vertex(left, point) {
                            return Err(surface_validation_error(
                                label,
                                "component loops touch away from exact shared vertices",
                            ));
                        }
                    }
                    None => {
                        return Err(surface_validation_error(
                            label,
                            "component loop containment predicate was undecided",
                        ));
                    }
                }
            }

            validate_loop_segments_disjoint_or_shared_vertices(left, right, projection, label)?;
        }
    }
    Ok(())
}

/// Validate that retained simple component loops are pairwise disjoint.
///
/// Component-holed outputs may now retain nonconvex outer loops produced by
/// exact convex-cutter replay. Pairwise disjointness therefore cannot use
/// convex half-space signs. The check keeps Yap's retained topology boundary:
/// exact segment contact rejects shared or crossing edges, then exact
/// triangulated point-in-polygon checks reject nesting. Segment relations use
/// Guigue and Devillers, "Fast and Robust Triangle-Triangle Overlap Test Using
/// Orientation Predicates," *Journal of Graphics Tools* 8.1 (2003), and the
/// simple-polygon interior probe uses Held, "FIST: Fast Industrial-Strength
/// Triangulation of Polygons," *Algorithmica* 30 (2001).
#[cfg(feature = "exact-triangulation")]
fn validate_simple_component_loops_disjoint(
    polygons: &[Vec<Point3>],
    projection: CoplanarProjection,
    label: &'static str,
) -> Result<(), MeshError> {
    for left_index in 0..polygons.len() {
        for right_index in left_index + 1..polygons.len() {
            let left = &polygons[left_index];
            let right = &polygons[right_index];
            if rings_have_any_edge_contact(left, right, projection).ok_or_else(|| {
                surface_validation_error(
                    label,
                    "component loop segment-intersection predicate was undecided",
                )
            })? {
                return Err(surface_validation_error(
                    label,
                    "component loop edges intersect or touch",
                ));
            }
            for point in left {
                match simple_polygon_location(point, right, projection) {
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
                match simple_polygon_location(point, left, projection) {
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
        }
    }
    Ok(())
}

/// Validate simple component loops that may meet at exact shared vertices.
///
/// This nonconvex-capable sibling mirrors
/// [`validate_component_loops_disjoint_allowing_vertex_point_touches`] while
/// using the triangulated simple-polygon location predicate. It is not used by
/// the first point-touch producer, which emits convex component hulls, but it
/// keeps the validation API semantically complete for future bounded branch
/// artifacts without weakening existing disjoint-loop validators.
#[cfg(feature = "exact-triangulation")]
fn validate_simple_component_loops_disjoint_allowing_vertex_point_touches(
    polygons: &[Vec<Point3>],
    projection: CoplanarProjection,
    label: &'static str,
) -> Result<(), MeshError> {
    for left_index in 0..polygons.len() {
        for right_index in left_index + 1..polygons.len() {
            let left = &polygons[left_index];
            let right = &polygons[right_index];
            validate_loop_segments_disjoint_or_shared_vertices(left, right, projection, label)?;
            for point in left {
                match simple_polygon_location(point, right, projection) {
                    Some(ConvexPolygonLocation::Outside) => {}
                    Some(ConvexPolygonLocation::Inside) => {
                        return Err(surface_validation_error(
                            label,
                            "component loops overlap or nest",
                        ));
                    }
                    Some(ConvexPolygonLocation::Boundary) => {
                        if !polygon_has_exact_vertex(right, point) {
                            return Err(surface_validation_error(
                                label,
                                "component loops touch away from exact shared vertices",
                            ));
                        }
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
                match simple_polygon_location(point, left, projection) {
                    Some(ConvexPolygonLocation::Outside) => {}
                    Some(ConvexPolygonLocation::Inside) => {
                        return Err(surface_validation_error(
                            label,
                            "component loops overlap or nest",
                        ));
                    }
                    Some(ConvexPolygonLocation::Boundary) => {
                        if !polygon_has_exact_vertex(left, point) {
                            return Err(surface_validation_error(
                                label,
                                "component loops touch away from exact shared vertices",
                            ));
                        }
                    }
                    None => {
                        return Err(surface_validation_error(
                            label,
                            "component loop containment predicate was undecided",
                        ));
                    }
                }
            }
        }
    }
    Ok(())
}

#[cfg(feature = "exact-triangulation")]
fn validate_loop_segments_disjoint_or_shared_vertices(
    left: &[Point3],
    right: &[Point3],
    projection: CoplanarProjection,
    label: &'static str,
) -> Result<(), MeshError> {
    for left_edge in 0..left.len() {
        let left_start = project_point(&left[left_edge], projection);
        let left_end = project_point(&left[(left_edge + 1) % left.len()], projection);
        for right_edge in 0..right.len() {
            let right_start = project_point(&right[right_edge], projection);
            let right_end = project_point(&right[(right_edge + 1) % right.len()], projection);
            match classify_segment_intersection(&left_start, &left_end, &right_start, &right_end)
                .value()
            {
                Some(SegmentIntersection::Disjoint) => {}
                Some(SegmentIntersection::EndpointTouch) => {
                    if !segments_share_exact_endpoint(
                        &left[left_edge],
                        &left[(left_edge + 1) % left.len()],
                        &right[right_edge],
                        &right[(right_edge + 1) % right.len()],
                    ) {
                        return Err(surface_validation_error(
                            label,
                            "component loops touch away from exact shared vertices",
                        ));
                    }
                }
                Some(
                    SegmentIntersection::Proper
                    | SegmentIntersection::CollinearOverlap
                    | SegmentIntersection::Identical,
                ) => {
                    return Err(surface_validation_error(
                        label,
                        "component loop edges intersect beyond exact shared vertices",
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
            let (next_index, reversed) =
                fragments.iter().enumerate().find_map(|(index, fragment)| {
                    if points_equal(&fragment.start, &current) {
                        Some((index, false))
                    } else if points_equal(&fragment.end, &current) {
                        Some((index, true))
                    } else {
                        None
                    }
                })?;
            let next = fragments.remove(next_index);
            polygon.push(if reversed { next.start } else { next.end });
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

/// Stitch retained loops through exact point-branch vertices.
///
/// Ordinary disjoint-loop stitching has only one continuation at each vertex.
/// Point-touch differences deliberately admit vertices with several incident
/// retained fragments. At those vertices the next directed fragment is the
/// outgoing edge with the smallest counter-clockwise turn from the incoming
/// edge, computed with exact projected cross/dot predicates rather than an
/// angle tolerance. This is the local planar-graph walk used by the
/// Weiler-Atherton retained-boundary construction, and the explicit branch
/// walk is the topology certificate Yap calls for in "Towards Exact Geometric
/// Computation," *Computational Geometry* 7.1-2 (1997).
#[cfg(feature = "exact-triangulation")]
fn stitch_branching_simple_loops(
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
            let previous = polygon.get(polygon.len().checked_sub(2)?)?.clone();
            let next_index =
                select_ccw_branch_continuation(&fragments, &previous, &current, projection)?;
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
fn select_ccw_branch_continuation(
    fragments: &[DirectedFragment],
    previous: &Point3,
    current: &Point3,
    projection: CoplanarProjection,
) -> Option<usize> {
    let incoming = projected_vector(previous, current, projection);
    let mut best: Option<(usize, Point2)> = None;
    for (index, fragment) in fragments.iter().enumerate() {
        if !points_equal(&fragment.start, current) {
            continue;
        }
        let candidate = projected_vector(current, &fragment.end, projection);
        if let Some((_, best_vector)) = &best {
            if ccw_turn_less(&incoming, &candidate, best_vector)? {
                best = Some((index, candidate));
            }
        } else {
            best = Some((index, candidate));
        }
    }
    best.map(|(index, _)| index)
}

#[cfg(feature = "exact-triangulation")]
fn projected_vector(from: &Point3, to: &Point3, projection: CoplanarProjection) -> Point2 {
    let from = project_point(from, projection);
    let to = project_point(to, projection);
    Point2 {
        x: sub(&to.x, &from.x),
        y: sub(&to.y, &from.y),
    }
}

#[cfg(feature = "exact-triangulation")]
fn ccw_turn_less(base: &Point2, left: &Point2, right: &Point2) -> Option<bool> {
    let left_bucket = ccw_turn_bucket(base, left)?;
    let right_bucket = ccw_turn_bucket(base, right)?;
    if left_bucket != right_bucket {
        return Some(left_bucket < right_bucket);
    }
    match compare_reals(&cross2(left, right), &ExactReal::from(0)).value()? {
        Ordering::Greater => Some(true),
        Ordering::Less => Some(false),
        Ordering::Equal => Some(false),
    }
}

#[cfg(feature = "exact-triangulation")]
fn ccw_turn_bucket(base: &Point2, candidate: &Point2) -> Option<u8> {
    match compare_reals(&cross2(base, candidate), &ExactReal::from(0)).value()? {
        Ordering::Greater => Some(0),
        Ordering::Less => Some(1),
        Ordering::Equal => {
            match compare_reals(&dot2(base, candidate), &ExactReal::from(0)).value()? {
                Ordering::Greater | Ordering::Equal => Some(0),
                Ordering::Less => Some(1),
            }
        }
    }
}

#[cfg(feature = "exact-triangulation")]
fn cross2(left: &Point2, right: &Point2) -> ExactReal {
    sub(&mul(&left.x, &right.y), &mul(&left.y, &right.x))
}

#[cfg(feature = "exact-triangulation")]
fn dot2(left: &Point2, right: &Point2) -> ExactReal {
    add(&mul(&left.x, &right.x), &mul(&left.y, &right.y))
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
