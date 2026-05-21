//! Prepared BREP query reports.
//!
//! These reports package exact predicate calls over retained BREP evidence.
//! They are scheduling and rejection surfaces, not private BREP boolean logic.

use hyperlimit::{
    Plane3, PlaneAabbRelation, PlaneSegmentRelation, PlaneSide, Point3, PredicateOutcome,
};

use crate::bounds::BrepFaceBoundsReport;
use crate::surface::BrepSurfaceKind;
use crate::topology::{BrepFaceId, BrepShell};

/// Explicit blocker for plane/face AABB preflight.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub enum BrepFacePlanePreflightBlocker {
    /// Exact face bounds could not be derived.
    FaceBoundsNotReady,
    /// `hyperlimit` could not decide the plane/AABB relation.
    UnknownPlaneAabbRelation,
}

/// Explicit blocker for segment/face-plane preflight.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub enum BrepSegmentFacePlaneBlocker {
    /// Source face was not found in the shell.
    MissingFace,
    /// Face references a surface id not present in the shell.
    MissingSurface,
    /// Face surface is not a supported exact plane.
    UnsupportedSurface,
    /// The retained plane has lossy, unknown, or invalid exact-core facts.
    SurfaceNotReady,
    /// `hyperlimit` could not decide the segment/plane relation.
    UnknownSegmentPlaneRelation,
}

/// Explicit blocker for point/face-plane preflight.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub enum BrepPointFacePlaneBlocker {
    /// Source face was not found in the shell.
    MissingFace,
    /// Face references a surface id not present in the shell.
    MissingSurface,
    /// Face surface is not a supported exact plane.
    UnsupportedSurface,
    /// The retained plane has lossy, unknown, or invalid exact-core facts.
    SurfaceNotReady,
    /// `hyperlimit` could not decide the point/plane relation.
    UnknownPointPlaneRelation,
}

/// Broad-phase plane/face report using exact face bounds.
#[derive(Clone, Debug, PartialEq)]
pub struct BrepFacePlanePreflightReport {
    /// Face that was tested.
    pub face: BrepFaceId,
    /// Exact face bounds report.
    pub bounds: BrepFaceBoundsReport,
    /// Decided plane/AABB relation when available.
    pub relation: Option<PlaneAabbRelation>,
    /// Whether the face bounds lie strictly on one side of the plane.
    pub certified_no_plane_crossing: bool,
    /// Whether the face bounds touch or cross the plane and require narrow
    /// face/trim replay.
    pub requires_narrow_phase: bool,
    /// Explicit blockers.
    pub blockers: Vec<BrepFacePlanePreflightBlocker>,
    /// Whether this preflight is ready for broad-phase scheduling.
    pub preflight_ready: bool,
}

/// Segment relation against a face's retained support plane.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct BrepSegmentFacePlaneReport {
    /// Face whose support plane was tested.
    pub face: BrepFaceId,
    /// Decided exact segment/plane relation when available.
    pub relation: Option<PlaneSegmentRelation>,
    /// Whether the segment is strictly on one side of the support plane.
    pub certified_no_plane_contact: bool,
    /// Whether the segment touches, crosses, or lies in the support plane and
    /// therefore needs exact face/trim narrow-phase replay.
    pub requires_narrow_phase: bool,
    /// Explicit blockers.
    pub blockers: Vec<BrepSegmentFacePlaneBlocker>,
    /// Whether this preflight is ready for scheduling.
    pub preflight_ready: bool,
}

/// Point relation against a face's retained support plane.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct BrepPointFacePlaneReport {
    /// Face whose support plane was tested.
    pub face: BrepFaceId,
    /// Decided exact point/plane side when available.
    pub side: Option<PlaneSide>,
    /// Whether the point is certified off the support plane.
    pub certified_off_support_plane: bool,
    /// Whether the point is exactly on the support plane.
    pub on_support_plane: bool,
    /// Whether the point is on the support plane and needs face-domain and trim
    /// replay before being accepted as on the face.
    pub requires_trim_replay: bool,
    /// Explicit blockers.
    pub blockers: Vec<BrepPointFacePlaneBlocker>,
    /// Whether this preflight is ready for scheduling.
    pub preflight_ready: bool,
}

impl BrepFacePlanePreflightReport {
    /// Classify a face's exact AABB against a plane.
    ///
    /// The implementation directly reuses `hyperlimit::PreparedPlane3` and its
    /// exact plane/AABB classifier. Following Yap, "Towards Exact Geometric
    /// Computation," *Computational Geometry* 7.1-2 (1997), a `Below` or
    /// `Above` AABB result is only a certified broad-phase rejection for plane
    /// crossing; an `Intersecting` AABB is a candidate that still needs exact
    /// surface/trim predicates before any topology changes.
    pub fn from_shell_face_plane(shell: &BrepShell, face: BrepFaceId, plane: &Plane3) -> Self {
        let bounds = shell.face_bounds_report(face);
        let mut blockers = Vec::new();
        let mut relation = None;

        if !bounds.exact_bounds_ready {
            blockers.push(BrepFacePlanePreflightBlocker::FaceBoundsNotReady);
        } else {
            let prepared_plane = plane.prepare();
            let prepared_bounds = bounds.prepare().expect("checked bounds readiness");
            match prepared_plane.classify_aabb3(
                prepared_bounds.prepared.min(),
                prepared_bounds.prepared.max(),
            ) {
                PredicateOutcome::Decided { value, .. } => {
                    relation = Some(value);
                }
                PredicateOutcome::Unknown { .. } => {
                    blockers.push(BrepFacePlanePreflightBlocker::UnknownPlaneAabbRelation);
                }
            }
        }

        let certified_no_plane_crossing = matches!(
            relation,
            Some(PlaneAabbRelation::Below | PlaneAabbRelation::Above)
        );
        let requires_narrow_phase = relation == Some(PlaneAabbRelation::Intersecting);
        let preflight_ready = blockers.is_empty();
        Self {
            face,
            bounds,
            relation,
            certified_no_plane_crossing,
            requires_narrow_phase,
            blockers,
            preflight_ready,
        }
    }
}

impl BrepSegmentFacePlaneReport {
    /// Classify a segment against the retained support plane of a face.
    ///
    /// This uses `hyperlimit::PreparedPlane3::classify_segment` over the exact
    /// planar surface stored by `hyperbrep`. Following Yap, "Towards Exact
    /// Geometric Computation," *Computational Geometry* 7.1-2 (1997), this
    /// report may reject segment/face interaction only when the segment is
    /// strictly on one side of the support plane. Crossings, endpoint touches,
    /// and coplanar segments remain narrow-phase candidates requiring exact
    /// face-domain and trim-boundary replay.
    pub fn from_shell_face_segment(
        shell: &BrepShell,
        face: BrepFaceId,
        start: &Point3,
        end: &Point3,
    ) -> Self {
        let mut blockers = Vec::new();
        let mut relation = None;
        let Some(face_record) = shell.faces.iter().find(|candidate| candidate.id == face) else {
            blockers.push(BrepSegmentFacePlaneBlocker::MissingFace);
            return Self::blocked(face, blockers);
        };
        let Some(surface) = shell
            .surfaces
            .iter()
            .find(|surface| surface.id == face_record.surface)
        else {
            blockers.push(BrepSegmentFacePlaneBlocker::MissingSurface);
            return Self::blocked(face, blockers);
        };
        if !surface.facts().exact_replay_ready {
            blockers.push(BrepSegmentFacePlaneBlocker::SurfaceNotReady);
        }
        let BrepSurfaceKind::Plane(plane) = &surface.kind else {
            blockers.push(BrepSegmentFacePlaneBlocker::UnsupportedSurface);
            return Self::blocked(face, blockers);
        };
        if blockers.is_empty() {
            match plane.prepare().classify_segment(start, end) {
                PredicateOutcome::Decided { value, .. } => {
                    relation = Some(value);
                }
                PredicateOutcome::Unknown { .. } => {
                    blockers.push(BrepSegmentFacePlaneBlocker::UnknownSegmentPlaneRelation);
                }
            }
        }

        let certified_no_plane_contact = matches!(
            relation,
            Some(PlaneSegmentRelation::Below | PlaneSegmentRelation::Above)
        );
        let requires_narrow_phase = matches!(
            relation,
            Some(
                PlaneSegmentRelation::Coplanar
                    | PlaneSegmentRelation::Crossing
                    | PlaneSegmentRelation::EndpointTouch
            )
        );
        let preflight_ready = blockers.is_empty();
        Self {
            face,
            relation,
            certified_no_plane_contact,
            requires_narrow_phase,
            blockers,
            preflight_ready,
        }
    }

    fn blocked(face: BrepFaceId, blockers: Vec<BrepSegmentFacePlaneBlocker>) -> Self {
        Self {
            face,
            relation: None,
            certified_no_plane_contact: false,
            requires_narrow_phase: false,
            preflight_ready: false,
            blockers,
        }
    }
}

impl BrepPointFacePlaneReport {
    /// Classify a point against the retained support plane of a face.
    ///
    /// This report uses `hyperlimit::PreparedPlane3::classify_point` over the
    /// face's exact planar surface. Per Yap, "Towards Exact Geometric
    /// Computation," *Computational Geometry* 7.1-2 (1997), an off-plane point
    /// is a certified rejection for point-on-face queries, while an on-plane
    /// point remains only a candidate until exact UV/domain and trim-boundary
    /// predicates replay.
    pub fn from_shell_face_point(shell: &BrepShell, face: BrepFaceId, point: &Point3) -> Self {
        let mut blockers = Vec::new();
        let mut side = None;
        let Some(face_record) = shell.faces.iter().find(|candidate| candidate.id == face) else {
            blockers.push(BrepPointFacePlaneBlocker::MissingFace);
            return Self::blocked(face, blockers);
        };
        let Some(surface) = shell
            .surfaces
            .iter()
            .find(|surface| surface.id == face_record.surface)
        else {
            blockers.push(BrepPointFacePlaneBlocker::MissingSurface);
            return Self::blocked(face, blockers);
        };
        if !surface.facts().exact_replay_ready {
            blockers.push(BrepPointFacePlaneBlocker::SurfaceNotReady);
        }
        let BrepSurfaceKind::Plane(plane) = &surface.kind else {
            blockers.push(BrepPointFacePlaneBlocker::UnsupportedSurface);
            return Self::blocked(face, blockers);
        };
        if blockers.is_empty() {
            match plane.prepare().classify_point(point) {
                PredicateOutcome::Decided { value, .. } => {
                    side = Some(value);
                }
                PredicateOutcome::Unknown { .. } => {
                    blockers.push(BrepPointFacePlaneBlocker::UnknownPointPlaneRelation);
                }
            }
        }

        let certified_off_support_plane = matches!(side, Some(PlaneSide::Below | PlaneSide::Above));
        let on_support_plane = side == Some(PlaneSide::On);
        let requires_trim_replay = on_support_plane;
        let preflight_ready = blockers.is_empty();
        Self {
            face,
            side,
            certified_off_support_plane,
            on_support_plane,
            requires_trim_replay,
            blockers,
            preflight_ready,
        }
    }

    fn blocked(face: BrepFaceId, blockers: Vec<BrepPointFacePlaneBlocker>) -> Self {
        Self {
            face,
            side: None,
            certified_off_support_plane: false,
            on_support_plane: false,
            requires_trim_replay: false,
            preflight_ready: false,
            blockers,
        }
    }
}

impl BrepShell {
    /// Classify a face's exact AABB against a plane as broad-phase evidence.
    pub fn face_plane_preflight(
        &self,
        face: BrepFaceId,
        plane: &Plane3,
    ) -> BrepFacePlanePreflightReport {
        BrepFacePlanePreflightReport::from_shell_face_plane(self, face, plane)
    }

    /// Classify a segment against a face support plane as preflight evidence.
    pub fn segment_face_plane_preflight(
        &self,
        face: BrepFaceId,
        start: &Point3,
        end: &Point3,
    ) -> BrepSegmentFacePlaneReport {
        BrepSegmentFacePlaneReport::from_shell_face_segment(self, face, start, end)
    }

    /// Classify a point against a face support plane as preflight evidence.
    pub fn point_face_plane_preflight(
        &self,
        face: BrepFaceId,
        point: &Point3,
    ) -> BrepPointFacePlaneReport {
        BrepPointFacePlaneReport::from_shell_face_point(self, face, point)
    }
}
