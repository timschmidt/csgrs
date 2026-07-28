//! BREP query evidence and reports.
//!
//! These reports package exact predicate calls over retained BREP evidence.
//! They are scheduling and rejection surfaces, not private BREP boolean logic.

use hyperlimit::{
    Plane3, PlaneAabbRelation, PlaneSegmentRelation, PlaneSide, Point3, PredicateOutcome,
    classify_plane_aabb3, classify_plane_segment,
};

use crate::bounds::BrepFaceBoundsReport;
use crate::surface::{
    BrepSurface, BrepSurfaceEvidence, BrepSurfaceKind, classify_plane_surface_point_with_evidence,
};
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

/// Explicit blocker for an immediate face-query batch.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub enum BrepFaceQueryBatchBlocker {
    /// Source face was not found in the shell.
    MissingFace,
    /// Face references a surface id not present in the shell.
    MissingSurface,
    /// Face surface is not a supported exact plane.
    UnsupportedSurface,
    /// The retained plane has lossy, unknown, or invalid exact-core facts.
    SurfaceNotReady,
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

/// Internal support-surface context for one immediate face-query batch.
#[derive(Clone, Debug)]
struct BrepFaceQueryContext<'a> {
    face: BrepFaceId,
    surface: Option<BrepSurfaceEvidence>,
    surface_source: Option<&'a BrepSurface>,
    blockers: Vec<BrepFaceQueryBatchBlocker>,
    query_ready: bool,
}

/// Aggregate diagnostics for an immediate face-query batch.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct BrepFaceQueryBatchReport {
    /// Face that was queried.
    pub face: BrepFaceId,
    /// Number of point/support-plane queries replayed.
    pub point_query_count: usize,
    /// Number of segment/support-plane queries replayed.
    pub segment_query_count: usize,
    /// Number of certified support-plane rejections.
    pub certified_rejection_count: usize,
    /// Number of query results that still require exact trim/domain replay.
    pub narrow_phase_candidate_count: usize,
    /// Whether the query inputs were exact-ready.
    pub query_ready: bool,
    /// Explicit blockers encountered while resolving the query inputs.
    pub blockers: Vec<BrepFaceQueryBatchBlocker>,
}

impl BrepFacePlanePreflightReport {
    /// Classify a face's exact AABB against a plane.
    ///
    /// The implementation calls HyperLimit's immediate exact plane/AABB
    /// classifier. A `Below` or `Above` result certifies broad-phase
    /// rejection; `Intersecting` still requires exact surface and trim
    /// predicates before any topology change.
    pub fn from_shell_face_plane(shell: &BrepShell, face: BrepFaceId, plane: &Plane3) -> Self {
        let bounds = shell.face_bounds_report(face);
        Self::from_face_bounds(face, bounds, plane)
    }

    fn from_face_bounds(face: BrepFaceId, bounds: BrepFaceBoundsReport, plane: &Plane3) -> Self {
        let mut blockers = Vec::new();
        let mut relation = None;

        if !bounds.exact_bounds_ready {
            blockers.push(BrepFacePlanePreflightBlocker::FaceBoundsNotReady);
        } else {
            let (min, max) = bounds.exact_bounds().expect("checked bounds readiness");
            match classify_plane_aabb3(plane, min, max) {
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

impl<'a> BrepFaceQueryContext<'a> {
    fn from_shell_face(shell: &'a BrepShell, face: BrepFaceId) -> Self {
        let mut blockers = Vec::new();

        let (surface_source, surface) =
            match shell.faces.iter().find(|candidate| candidate.id == face) {
                Some(face_record) => match shell
                    .surfaces
                    .iter()
                    .find(|surface| surface.id == face_record.surface)
                {
                    Some(surface) => {
                        let evidence = surface.evidence();
                        match &evidence {
                            BrepSurfaceEvidence::Plane { .. } => {}
                            BrepSurfaceEvidence::Blocked {
                                blockers: surface_blockers,
                                ..
                            } => {
                                blockers.push(BrepFaceQueryBatchBlocker::SurfaceNotReady);
                                if surface_blockers.iter().any(|blocker| {
                                    matches!(
                                        blocker,
                                        crate::surface::BrepSurfaceBlocker::UnsupportedFamily
                                    )
                                }) {
                                    blockers.push(BrepFaceQueryBatchBlocker::UnsupportedSurface);
                                }
                            }
                        }
                        (Some(surface), Some(evidence))
                    }
                    None => {
                        blockers.push(BrepFaceQueryBatchBlocker::MissingSurface);
                        (None, None)
                    }
                },
                None => {
                    blockers.push(BrepFaceQueryBatchBlocker::MissingFace);
                    (None, None)
                }
            };

        let query_ready = blockers.is_empty();
        Self {
            face,
            surface,
            surface_source,
            blockers,
            query_ready,
        }
    }

    fn point_face_plane_preflight(&self, point: &Point3) -> BrepPointFacePlaneReport {
        let mut blockers = point_blockers_from_batch(&self.blockers);
        let mut side = None;
        if blockers.is_empty() {
            match (&self.surface_source, &self.surface) {
                (
                    Some(BrepSurface {
                        id,
                        kind: BrepSurfaceKind::Plane(plane),
                    }),
                    Some(BrepSurfaceEvidence::Plane {
                        plane: plane_evidence,
                        ..
                    }),
                ) => {
                    match classify_plane_surface_point_with_evidence(
                        *id,
                        plane,
                        point,
                        plane_evidence,
                    )
                    .side
                    {
                        Some(value) => side = Some(value),
                        None => {
                            blockers.push(BrepPointFacePlaneBlocker::UnknownPointPlaneRelation);
                        }
                    }
                }
                (_, Some(BrepSurfaceEvidence::Blocked { .. })) => {
                    blockers.push(BrepPointFacePlaneBlocker::SurfaceNotReady);
                }
                _ => blockers.push(BrepPointFacePlaneBlocker::MissingSurface),
            }
        }
        BrepPointFacePlaneReport::from_parts(self.face, side, blockers)
    }

    fn segment_face_plane_preflight(
        &self,
        start: &Point3,
        end: &Point3,
    ) -> BrepSegmentFacePlaneReport {
        let mut blockers = segment_blockers_from_batch(&self.blockers);
        let mut relation = None;
        if blockers.is_empty() {
            match (&self.surface_source, &self.surface) {
                (
                    Some(BrepSurface {
                        kind: BrepSurfaceKind::Plane(plane),
                        ..
                    }),
                    Some(BrepSurfaceEvidence::Plane { .. }),
                ) => match classify_plane_segment(plane, start, end) {
                    PredicateOutcome::Decided { value, .. } => relation = Some(value),
                    PredicateOutcome::Unknown { .. } => {
                        blockers.push(BrepSegmentFacePlaneBlocker::UnknownSegmentPlaneRelation);
                    }
                },
                (_, Some(BrepSurfaceEvidence::Blocked { .. })) => {
                    blockers.push(BrepSegmentFacePlaneBlocker::SurfaceNotReady);
                }
                _ => blockers.push(BrepSegmentFacePlaneBlocker::MissingSurface),
            }
        }
        BrepSegmentFacePlaneReport::from_parts(self.face, relation, blockers)
    }

    fn batch_report(
        &self,
        points: &[Point3],
        segments: &[(&Point3, &Point3)],
    ) -> BrepFaceQueryBatchReport {
        let mut certified_rejection_count = 0;
        let mut narrow_phase_candidate_count = 0;
        for point in points {
            let report = self.point_face_plane_preflight(point);
            certified_rejection_count += usize::from(report.certified_off_support_plane);
            narrow_phase_candidate_count += usize::from(report.requires_trim_replay);
        }
        for (start, end) in segments {
            let report = self.segment_face_plane_preflight(start, end);
            certified_rejection_count += usize::from(report.certified_no_plane_contact);
            narrow_phase_candidate_count += usize::from(report.requires_narrow_phase);
        }
        BrepFaceQueryBatchReport {
            face: self.face,
            point_query_count: points.len(),
            segment_query_count: segments.len(),
            certified_rejection_count,
            narrow_phase_candidate_count,
            query_ready: self.query_ready,
            blockers: self.blockers.clone(),
        }
    }
}

impl BrepSegmentFacePlaneReport {
    /// Classify a segment against the retained support plane of a face.
    ///
    /// This uses HyperLimit's immediate segment/plane classifier over the exact
    /// retained plane. It rejects interaction only when the segment is strictly
    /// on one side; crossings, endpoint touches, and coplanar segments remain
    /// narrow-phase candidates requiring exact domain and trim replay.
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
            match classify_plane_segment(plane, start, end) {
                PredicateOutcome::Decided { value, .. } => {
                    relation = Some(value);
                }
                PredicateOutcome::Unknown { .. } => {
                    blockers.push(BrepSegmentFacePlaneBlocker::UnknownSegmentPlaneRelation);
                }
            }
        }

        Self::from_parts(face, relation, blockers)
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

    fn from_parts(
        face: BrepFaceId,
        relation: Option<PlaneSegmentRelation>,
        blockers: Vec<BrepSegmentFacePlaneBlocker>,
    ) -> Self {
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
}

impl BrepPointFacePlaneReport {
    /// Classify a point against the retained support plane of a face.
    ///
    /// This uses HyperLimit's immediate point/plane classifier over the face's
    /// exact plane. An off-plane point is a certified rejection; an on-plane
    /// point remains a candidate until exact UV-domain and trim predicates
    /// replay.
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
            match hyperlimit::classify_point_plane(point, plane) {
                PredicateOutcome::Decided { value, .. } => {
                    side = Some(value);
                }
                PredicateOutcome::Unknown { .. } => {
                    blockers.push(BrepPointFacePlaneBlocker::UnknownPointPlaneRelation);
                }
            }
        }

        Self::from_parts(face, side, blockers)
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

    fn from_parts(
        face: BrepFaceId,
        side: Option<PlaneSide>,
        blockers: Vec<BrepPointFacePlaneBlocker>,
    ) -> Self {
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

    /// Classify one face's exact bounds against several planes immediately.
    ///
    /// Bounds are derived once inside this completed batch operation and are
    /// not exposed as a separate query lifecycle.
    pub fn face_plane_preflight_batch(
        &self,
        face: BrepFaceId,
        planes: &[Plane3],
    ) -> Vec<BrepFacePlanePreflightReport> {
        let bounds = self.face_bounds_report(face);
        planes
            .iter()
            .map(|plane| {
                BrepFacePlanePreflightReport::from_face_bounds(face, bounds.clone(), plane)
            })
            .collect()
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

    /// Run point and segment support-plane preflights as one immediate batch.
    ///
    /// Surface evidence is resolved once inside the call. The returned report
    /// contains aggregate scheduling diagnostics; callers do not own a
    /// reusable query handle.
    pub fn face_query_batch_report(
        &self,
        face: BrepFaceId,
        points: &[Point3],
        segments: &[(&Point3, &Point3)],
    ) -> BrepFaceQueryBatchReport {
        BrepFaceQueryContext::from_shell_face(self, face).batch_report(points, segments)
    }
}

fn point_blockers_from_batch(
    blockers: &[BrepFaceQueryBatchBlocker],
) -> Vec<BrepPointFacePlaneBlocker> {
    blockers
        .iter()
        .map(|blocker| match blocker {
            BrepFaceQueryBatchBlocker::MissingFace => BrepPointFacePlaneBlocker::MissingFace,
            BrepFaceQueryBatchBlocker::MissingSurface => BrepPointFacePlaneBlocker::MissingSurface,
            BrepFaceQueryBatchBlocker::UnsupportedSurface => {
                BrepPointFacePlaneBlocker::UnsupportedSurface
            }
            BrepFaceQueryBatchBlocker::SurfaceNotReady => {
                BrepPointFacePlaneBlocker::SurfaceNotReady
            }
        })
        .collect()
}

fn segment_blockers_from_batch(
    blockers: &[BrepFaceQueryBatchBlocker],
) -> Vec<BrepSegmentFacePlaneBlocker> {
    blockers
        .iter()
        .map(|blocker| match blocker {
            BrepFaceQueryBatchBlocker::MissingFace => BrepSegmentFacePlaneBlocker::MissingFace,
            BrepFaceQueryBatchBlocker::MissingSurface => {
                BrepSegmentFacePlaneBlocker::MissingSurface
            }
            BrepFaceQueryBatchBlocker::UnsupportedSurface => {
                BrepSegmentFacePlaneBlocker::UnsupportedSurface
            }
            BrepFaceQueryBatchBlocker::SurfaceNotReady => {
                BrepSegmentFacePlaneBlocker::SurfaceNotReady
            }
        })
        .collect()
}
