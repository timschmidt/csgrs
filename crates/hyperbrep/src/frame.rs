//! Exact surface parameter-frame reports.
//!
//! Planar surfaces receive an exact graph frame by selecting one certifiably
//! nonzero normal coordinate as the solved axis. Periodic analytic frames,
//! seams, and poles remain explicit future work.

use std::cmp::Ordering;
use std::collections::{BTreeMap, BTreeSet};

use hyperlimit::{Plane3, Point2, Point3};
use hyperreal::{Real, RealSign};

use crate::surface::{BrepSurface, BrepSurfaceBlocker, BrepSurfaceId, BrepSurfaceKind};
use crate::topology::{BrepEdge, BrepEdgeOrientation, BrepFaceId, BrepShell, BrepVertexId};

/// Plane coordinate solved from the other two retained UV coordinates.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub enum BrepPlaneFrameAxis {
    /// Normal is structurally supported on x; UV coordinates are `(y, z)`.
    X,
    /// Normal is structurally supported on y; UV coordinates are `(z, x)`.
    Y,
    /// Normal is structurally supported on z; UV coordinates are `(x, y)`.
    Z,
}

/// Explicit blocker for preparing or using a retained surface frame.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub enum BrepSurfaceFrameBlocker {
    /// Surface provenance or family is not ready for exact replay.
    SurfaceNotReady,
    /// Surface is not a supported plane.
    UnsupportedSurface,
    /// Legacy blocker retained for report compatibility. General planes now use
    /// an exact solved-axis frame.
    NonAxisAlignedPlane,
    /// No normal coordinate could be certified nonzero for the solved axis.
    UnknownNormalPivot,
    /// Exact scalar division needed to evaluate the frame coordinate failed.
    AxisCoordinateDivisionFailed,
}

/// Explicit blocker for deriving exact face UV bounds.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub enum BrepFaceUvBoundsBlocker {
    /// Source face was not found in the shell.
    MissingFace,
    /// Face references a surface id not present in the shell.
    MissingSurface,
    /// Retained surface frame could not be derived.
    FrameNotReady,
    /// The face has no coedges from which vertices can be projected.
    EmptyBoundary,
    /// A coedge references an edge id not present in the shell.
    MissingEdge,
    /// A referenced edge starts and ends at the same vertex.
    DegenerateEdge,
    /// A referenced edge points at a vertex id not present in the shell.
    MissingVertex,
    /// UV coordinate comparison could not be certified exactly enough to order
    /// the 2D bounds.
    UnknownCoordinateOrdering,
}

/// Derived exact UV frame for one retained surface.
#[derive(Clone, Debug, PartialEq)]
pub struct BrepSurfaceFrameReport {
    /// Retained surface id.
    pub surface: BrepSurfaceId,
    /// Axis chosen for the one-hot plane normal.
    pub axis: Option<BrepPlaneFrameAxis>,
    /// Surface-preparation blockers.
    pub surface_blockers: Vec<BrepSurfaceBlocker>,
    /// Frame-specific blockers.
    pub blockers: Vec<BrepSurfaceFrameBlocker>,
    /// Whether exact UV-to-3D and 3D-to-UV replay is available.
    pub exact_frame_ready: bool,
}

/// Exact UV-to-3D evaluation report.
#[derive(Clone, Debug, PartialEq)]
pub struct BrepSurfaceFrameEvalReport {
    /// Retained surface id.
    pub surface: BrepSurfaceId,
    /// Source UV coordinate.
    pub uv: Point2,
    /// Evaluated exact model-space point when frame replay succeeds.
    pub point: Option<Point3>,
    /// Frame report used for evaluation.
    pub frame: BrepSurfaceFrameReport,
    /// Whether evaluation succeeded exactly.
    pub exact_evaluation_ready: bool,
}

/// Exact 3D-to-UV projection report.
#[derive(Clone, Debug, PartialEq)]
pub struct BrepSurfaceFrameProjectionReport {
    /// Retained surface id.
    pub surface: BrepSurfaceId,
    /// Source model-space point.
    pub point: Point3,
    /// Projected exact UV coordinate when frame replay succeeds.
    pub uv: Option<Point2>,
    /// Frame report used for projection.
    pub frame: BrepSurfaceFrameReport,
    /// Whether projection succeeded exactly.
    pub exact_projection_ready: bool,
}

/// Exact UV AABB evidence for one retained face.
#[derive(Clone, Debug, PartialEq)]
pub struct BrepFaceUvBoundsReport {
    /// Face that was requested.
    pub face: BrepFaceId,
    /// Whether the face exists in the shell.
    pub face_found: bool,
    /// Surface frame used to project boundary vertices, when the face surface
    /// exists.
    pub frame: Option<BrepSurfaceFrameReport>,
    /// Number of unique boundary vertices projected into UV.
    pub vertex_count: usize,
    /// Exact minimum UV corner when all coordinate orderings were certified.
    pub min: Option<Point2>,
    /// Exact maximum UV corner when all coordinate orderings were certified.
    pub max: Option<Point2>,
    /// Whether the UV AABB has zero extent along u.
    pub zero_u_extent: bool,
    /// Whether the UV AABB has zero extent along v.
    pub zero_v_extent: bool,
    /// Explicit blockers.
    pub blockers: Vec<BrepFaceUvBoundsBlocker>,
    /// Whether exact UV bounds are available for downstream consumers.
    pub exact_uv_bounds_ready: bool,
}

impl BrepSurfaceFrameReport {
    /// Prepare an exact graph frame for a retained planar surface.
    ///
    /// The frame is accepted only when retained object facts certify a simple
    /// algebraic map. Otherwise the uncertainty is reported instead of
    /// constructing an arbitrary floating basis.
    pub fn from_surface(surface: &BrepSurface) -> Self {
        let facts = surface.facts();
        let surface_blockers = surface_blockers(surface);
        let mut blockers = Vec::new();
        if !surface_blockers.is_empty() {
            blockers.push(BrepSurfaceFrameBlocker::SurfaceNotReady);
        }

        let axis = match &surface.kind {
            BrepSurfaceKind::Plane(plane) => select_plane_pivot(plane).or_else(|| {
                blockers.push(BrepSurfaceFrameBlocker::UnknownNormalPivot);
                None
            }),
            BrepSurfaceKind::Unsupported { .. } => {
                blockers.push(BrepSurfaceFrameBlocker::UnsupportedSurface);
                None
            }
        };
        let exact_frame_ready = blockers.is_empty() && facts.exact_replay_ready && axis.is_some();
        Self {
            surface: surface.id,
            axis: exact_frame_ready.then_some(axis).flatten(),
            surface_blockers,
            blockers,
            exact_frame_ready,
        }
    }
}

impl BrepSurface {
    /// Prepare a conservative exact parameter frame for this surface.
    pub fn frame_report(&self) -> BrepSurfaceFrameReport {
        BrepSurfaceFrameReport::from_surface(self)
    }

    /// Evaluate an exact UV coordinate through this surface frame.
    pub fn evaluate_frame_uv(&self, uv: Point2) -> BrepSurfaceFrameEvalReport {
        let mut frame = self.frame_report();
        let point = if frame.exact_frame_ready {
            match &self.kind {
                BrepSurfaceKind::Plane(plane) => {
                    match evaluate_axis_plane(plane, frame.axis.expect("ready frame axis"), &uv) {
                        Some(point) => Some(point),
                        None => {
                            frame
                                .blockers
                                .push(BrepSurfaceFrameBlocker::AxisCoordinateDivisionFailed);
                            frame.exact_frame_ready = false;
                            None
                        }
                    }
                }
                BrepSurfaceKind::Unsupported { .. } => None,
            }
        } else {
            None
        };
        BrepSurfaceFrameEvalReport {
            surface: self.id,
            uv,
            exact_evaluation_ready: point.is_some(),
            point,
            frame,
        }
    }

    /// Project a model-space point into this surface frame's UV coordinates.
    pub fn project_frame_point(&self, point: Point3) -> BrepSurfaceFrameProjectionReport {
        let frame = self.frame_report();
        let uv = frame
            .exact_frame_ready
            .then(|| project_axis_point(frame.axis.expect("ready frame axis"), &point));
        BrepSurfaceFrameProjectionReport {
            surface: self.id,
            point,
            exact_projection_ready: uv.is_some(),
            uv,
            frame,
        }
    }
}

impl BrepFaceUvBoundsReport {
    /// Derive exact UV bounds by projecting retained face boundary vertices.
    ///
    /// The UV box is accepted only when the frame and every coordinate ordering
    /// replay exactly; otherwise blockers remain visible to tessellation and
    /// construction callers.
    pub fn from_shell_face(shell: &BrepShell, face: BrepFaceId) -> Self {
        let Some(source_face) = shell.faces.iter().find(|candidate| candidate.id == face) else {
            return Self::blocked(
                face,
                false,
                None,
                Vec::new(),
                vec![BrepFaceUvBoundsBlocker::MissingFace],
            );
        };
        let surface_by_id = shell
            .surfaces
            .iter()
            .map(|surface| (surface.id, surface))
            .collect::<BTreeMap<_, _>>();
        let edge_by_id = shell
            .edges
            .iter()
            .map(|edge| (edge.id, *edge))
            .collect::<BTreeMap<_, _>>();
        let vertex_by_id = shell
            .vertices
            .iter()
            .map(|vertex| (vertex.id, vertex))
            .collect::<BTreeMap<_, _>>();

        let Some(surface) = surface_by_id.get(&source_face.surface) else {
            return Self::blocked(
                face,
                true,
                None,
                Vec::new(),
                vec![BrepFaceUvBoundsBlocker::MissingSurface],
            );
        };
        let frame = surface.frame_report();
        let mut blockers = Vec::new();
        if !frame.exact_frame_ready {
            blockers.push(BrepFaceUvBoundsBlocker::FrameNotReady);
        }

        let mut vertex_ids = BTreeSet::new();
        for face_loop in source_face.loops() {
            for coedge in &face_loop.coedges {
                let Some(edge) = edge_by_id.get(&coedge.edge) else {
                    blockers.push(BrepFaceUvBoundsBlocker::MissingEdge);
                    continue;
                };
                if edge.is_degenerate() {
                    blockers.push(BrepFaceUvBoundsBlocker::DegenerateEdge);
                }
                let (start, end) = oriented_endpoints(*edge, coedge.orientation);
                vertex_ids.insert(start);
                vertex_ids.insert(end);
            }
        }
        if vertex_ids.is_empty() {
            blockers.push(BrepFaceUvBoundsBlocker::EmptyBoundary);
        }

        let mut projected = Vec::with_capacity(vertex_ids.len());
        if frame.exact_frame_ready {
            for vertex_id in vertex_ids {
                let Some(vertex) = vertex_by_id.get(&vertex_id) else {
                    blockers.push(BrepFaceUvBoundsBlocker::MissingVertex);
                    continue;
                };
                projected.push(project_axis_point(
                    frame.axis.expect("ready frame axis"),
                    &vertex.point,
                ));
            }
        } else {
            for vertex_id in vertex_ids {
                if !vertex_by_id.contains_key(&vertex_id) {
                    blockers.push(BrepFaceUvBoundsBlocker::MissingVertex);
                }
            }
        }

        let Some((min, max)) = uv_bounds_from_points(&projected, &mut blockers) else {
            return Self::blocked(face, true, Some(frame), blockers, Vec::new());
        };
        let zero_u_extent = min.x == max.x;
        let zero_v_extent = min.y == max.y;
        let exact_uv_bounds_ready = blockers.is_empty();
        Self {
            face,
            face_found: true,
            frame: Some(frame),
            vertex_count: projected.len(),
            min: exact_uv_bounds_ready.then_some(min),
            max: exact_uv_bounds_ready.then_some(max),
            zero_u_extent,
            zero_v_extent,
            blockers,
            exact_uv_bounds_ready,
        }
    }

    fn blocked(
        face: BrepFaceId,
        face_found: bool,
        frame: Option<BrepSurfaceFrameReport>,
        mut blockers: Vec<BrepFaceUvBoundsBlocker>,
        extra_blockers: Vec<BrepFaceUvBoundsBlocker>,
    ) -> Self {
        blockers.extend(extra_blockers);
        Self {
            face,
            face_found,
            frame,
            vertex_count: 0,
            min: None,
            max: None,
            zero_u_extent: false,
            zero_v_extent: false,
            blockers,
            exact_uv_bounds_ready: false,
        }
    }
}

impl BrepShell {
    /// Derive exact UV bounds for a retained face through its surface frame.
    pub fn face_uv_bounds_report(&self, face: BrepFaceId) -> BrepFaceUvBoundsReport {
        BrepFaceUvBoundsReport::from_shell_face(self, face)
    }
}

fn surface_blockers(surface: &BrepSurface) -> Vec<BrepSurfaceBlocker> {
    let evidence = surface.evidence();
    match evidence {
        crate::surface::BrepSurfaceEvidence::Plane { .. } => Vec::new(),
        crate::surface::BrepSurfaceEvidence::Blocked { blockers, .. } => blockers,
    }
}

fn uv_bounds_from_points(
    points: &[Point2],
    blockers: &mut Vec<BrepFaceUvBoundsBlocker>,
) -> Option<(Point2, Point2)> {
    let first = points.first()?;
    let mut min = first.clone();
    let mut max = first.clone();
    for point in points.iter().skip(1) {
        update_uv_min(&mut min.x, &point.x, blockers);
        update_uv_min(&mut min.y, &point.y, blockers);
        update_uv_max(&mut max.x, &point.x, blockers);
        update_uv_max(&mut max.y, &point.y, blockers);
    }
    Some((min, max))
}

fn update_uv_min(
    current: &mut Real,
    candidate: &Real,
    blockers: &mut Vec<BrepFaceUvBoundsBlocker>,
) {
    match candidate.partial_cmp(current) {
        Some(Ordering::Less) => *current = candidate.clone(),
        Some(Ordering::Equal | Ordering::Greater) => {}
        None => blockers.push(BrepFaceUvBoundsBlocker::UnknownCoordinateOrdering),
    }
}

fn update_uv_max(
    current: &mut Real,
    candidate: &Real,
    blockers: &mut Vec<BrepFaceUvBoundsBlocker>,
) {
    match candidate.partial_cmp(current) {
        Some(Ordering::Greater) => *current = candidate.clone(),
        Some(Ordering::Equal | Ordering::Less) => {}
        None => blockers.push(BrepFaceUvBoundsBlocker::UnknownCoordinateOrdering),
    }
}

fn oriented_endpoints(
    edge: BrepEdge,
    orientation: BrepEdgeOrientation,
) -> (BrepVertexId, BrepVertexId) {
    match orientation {
        BrepEdgeOrientation::Forward => (edge.start, edge.end),
        BrepEdgeOrientation::Reversed => (edge.end, edge.start),
    }
}

fn axis_from_index(index: usize) -> BrepPlaneFrameAxis {
    match index {
        0 => BrepPlaneFrameAxis::X,
        1 => BrepPlaneFrameAxis::Y,
        2 => BrepPlaneFrameAxis::Z,
        _ => unreachable!("Point3 one-hot axis index must be in 0..3"),
    }
}

fn evaluate_axis_plane(plane: &Plane3, axis: BrepPlaneFrameAxis, uv: &Point2) -> Option<Point3> {
    let coordinate = solve_plane_axis_coordinate(plane, axis, uv)?;
    Some(match axis {
        BrepPlaneFrameAxis::X => Point3::new(coordinate, uv.x.clone(), uv.y.clone()),
        BrepPlaneFrameAxis::Y => Point3::new(uv.y.clone(), coordinate, uv.x.clone()),
        BrepPlaneFrameAxis::Z => Point3::new(uv.x.clone(), uv.y.clone(), coordinate),
    })
}

fn project_axis_point(axis: BrepPlaneFrameAxis, point: &Point3) -> Point2 {
    match axis {
        BrepPlaneFrameAxis::X => Point2::new(point.y.clone(), point.z.clone()),
        BrepPlaneFrameAxis::Y => Point2::new(point.z.clone(), point.x.clone()),
        BrepPlaneFrameAxis::Z => Point2::new(point.x.clone(), point.y.clone()),
    }
}

fn solve_plane_axis_coordinate(
    plane: &Plane3,
    axis: BrepPlaneFrameAxis,
    uv: &Point2,
) -> Option<Real> {
    let numerator = match axis {
        BrepPlaneFrameAxis::X => {
            -plane.offset.clone()
                - plane.normal.y.clone() * uv.x.clone()
                - plane.normal.z.clone() * uv.y.clone()
        }
        BrepPlaneFrameAxis::Y => {
            -plane.offset.clone()
                - plane.normal.z.clone() * uv.x.clone()
                - plane.normal.x.clone() * uv.y.clone()
        }
        BrepPlaneFrameAxis::Z => {
            -plane.offset.clone()
                - plane.normal.x.clone() * uv.x.clone()
                - plane.normal.y.clone() * uv.y.clone()
        }
    };
    let denominator = match axis {
        BrepPlaneFrameAxis::X => &plane.normal.x,
        BrepPlaneFrameAxis::Y => &plane.normal.y,
        BrepPlaneFrameAxis::Z => &plane.normal.z,
    };
    (numerator / denominator).ok()
}

fn select_plane_pivot(plane: &Plane3) -> Option<BrepPlaneFrameAxis> {
    let facts = plane.structural_facts();
    if let Some(index) = facts.normal.known_axis_index {
        return Some(axis_from_index(index));
    }
    for index in 0..3 {
        if facts.normal.known_nonzero_mask & (1 << index) != 0 {
            return Some(axis_from_index(index));
        }
    }
    for (index, coordinate) in [&plane.normal.x, &plane.normal.y, &plane.normal.z]
        .into_iter()
        .enumerate()
    {
        if matches!(
            coordinate.refine_sign_until(-64),
            Some(RealSign::Negative | RealSign::Positive)
        ) {
            return Some(axis_from_index(index));
        }
    }
    None
}
