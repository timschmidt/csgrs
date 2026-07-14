//! Exact planar face area evidence.
//!
//! Area reports deliberately expose projected twice-area rather than silently
//! normalizing by a plane-normal length. That keeps the evidence algebraic and
//! useful to exact downstream consumers while avoiding square roots.

use std::cmp::Ordering;
use std::collections::{BTreeMap, BTreeSet};

use hyperreal::Real;

use crate::surface::{BrepSurfaceKind, BrepSurfaceSource};
use crate::topology::{BrepEdge, BrepEdgeOrientation, BrepFaceId, BrepShell, BrepVertexId};

/// Coordinate plane used for a projected planar area computation.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub enum BrepAreaProjectionAxis {
    /// Project to the `yz` coordinate plane and use the x component of the
    /// polygon area vector.
    X,
    /// Project to the `zx` coordinate plane and use the y component of the
    /// polygon area vector.
    Y,
    /// Project to the `xy` coordinate plane and use the z component of the
    /// polygon area vector.
    Z,
}

/// Explicit blocker for exact planar face area evidence.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub enum BrepFaceAreaBlocker {
    /// Source face was not found in the shell.
    MissingFace,
    /// Face references a surface id not present in the shell.
    MissingSurface,
    /// Face surface is not an exact planar surface.
    UnsupportedSurface,
    /// Plane normal is not structurally known to be one-hot.
    NonAxisAlignedPlane,
    /// The nonzero normal component's sign could not be certified.
    UnknownNormalDirection,
    /// A loop has no coedges.
    EmptyLoop,
    /// A coedge references an edge id not present in the shell.
    MissingEdge,
    /// A referenced edge starts and ends at the same vertex.
    DegenerateEdge,
    /// A referenced edge points at a vertex id not present in the shell.
    MissingVertex,
    /// Consecutive oriented coedges do not form a closed vertex chain.
    BrokenLoopChain,
    /// The projected area sign could not be certified.
    UnknownAreaSign,
}

/// Exact projected area evidence for one planar face.
#[derive(Clone, Debug, PartialEq)]
pub struct BrepFaceAreaReport {
    /// Face that was requested.
    pub face: BrepFaceId,
    /// Whether the source face exists in the shell.
    pub face_found: bool,
    /// Projection axis selected from the retained plane normal.
    pub projection_axis: Option<BrepAreaProjectionAxis>,
    /// Number of loops scanned.
    pub loop_count: usize,
    /// Number of unique boundary vertices referenced by the face loops.
    pub boundary_vertex_count: usize,
    /// Signed projected twice-area oriented by the retained surface normal.
    pub signed_twice_projected_area: Option<Real>,
    /// Whether the signed projected area is exactly zero.
    pub zero_area: bool,
    /// Whether the retained loop winding has positive area relative to the
    /// retained surface normal.
    pub positive_area: bool,
    /// Whether the retained loop winding has negative area relative to the
    /// retained surface normal.
    pub negative_area: bool,
    /// Explicit blockers.
    pub blockers: Vec<BrepFaceAreaBlocker>,
    /// Whether exact projected area evidence is available.
    pub exact_area_ready: bool,
}

impl BrepFaceAreaReport {
    /// Derive exact projected twice-area for an axis-aligned planar face.
    ///
    /// This report uses the polygon area-vector identity popularized in
    /// graphics and computational geometry texts: for an ordered loop, the
    /// component-wise sum of `p_i x p_{i+1}` gives twice the projected signed
    /// area. In Yap, "Towards Exact Geometric Computation," *Computational
    /// Geometry* 7.1-2 (1997), this is the kind of algebraic certificate that
    /// should be replayed exactly instead of normalized through floating
    /// tolerances. Retained BREP loop traversal follows Mäntylä, *An
    /// Introduction to Solid Modeling* (1988).
    pub fn from_shell_face(shell: &BrepShell, face: BrepFaceId) -> Self {
        let Some(source_face) = shell.faces.iter().find(|candidate| candidate.id == face) else {
            return Self::blocked(
                face,
                false,
                Vec::new(),
                vec![BrepFaceAreaBlocker::MissingFace],
            );
        };

        let mut blockers = Vec::new();
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
                Vec::new(),
                vec![BrepFaceAreaBlocker::MissingSurface],
            );
        };
        let (projection_axis, normal_component) = match &surface.kind {
            BrepSurfaceKind::Plane(plane)
                if matches!(
                    surface.source,
                    BrepSurfaceSource::ExactConstruction | BrepSurfaceSource::ExactImport
                ) =>
            {
                let Some(axis_index) = plane.structural_facts().normal.known_axis_index else {
                    blockers.push(BrepFaceAreaBlocker::NonAxisAlignedPlane);
                    return Self::blocked(face, true, blockers, Vec::new());
                };
                let axis = axis_from_index(axis_index);
                let component = match axis {
                    BrepAreaProjectionAxis::X => &plane.normal.x,
                    BrepAreaProjectionAxis::Y => &plane.normal.y,
                    BrepAreaProjectionAxis::Z => &plane.normal.z,
                };
                (axis, component)
            }
            BrepSurfaceKind::Plane(_) | BrepSurfaceKind::Unsupported { .. } => {
                blockers.push(BrepFaceAreaBlocker::UnsupportedSurface);
                return Self::blocked(face, true, blockers, Vec::new());
            }
        };

        let mut unique_vertices = BTreeSet::new();
        let mut signed_twice_projected_area = Real::from(0);
        for face_loop in source_face.loops() {
            if face_loop.coedges.is_empty() {
                blockers.push(BrepFaceAreaBlocker::EmptyLoop);
                continue;
            }

            let mut loop_vertices = Vec::with_capacity(face_loop.coedges.len());
            let mut loop_ends = Vec::with_capacity(face_loop.coedges.len());
            for coedge in &face_loop.coedges {
                let Some(edge) = edge_by_id.get(&coedge.edge) else {
                    blockers.push(BrepFaceAreaBlocker::MissingEdge);
                    continue;
                };
                if edge.is_degenerate() {
                    blockers.push(BrepFaceAreaBlocker::DegenerateEdge);
                }
                let (start, end) = oriented_endpoints(*edge, coedge.orientation);
                unique_vertices.insert(start);
                unique_vertices.insert(end);
                loop_vertices.push(start);
                loop_ends.push(end);
            }

            for (end, next_start) in loop_ends
                .iter()
                .zip(loop_vertices.iter().cycle().skip(1))
                .take(loop_vertices.len())
            {
                if end != next_start {
                    blockers.push(BrepFaceAreaBlocker::BrokenLoopChain);
                }
            }

            for index in 0..loop_vertices.len() {
                let next_index = (index + 1) % loop_vertices.len();
                let Some(current) = vertex_by_id.get(&loop_vertices[index]) else {
                    blockers.push(BrepFaceAreaBlocker::MissingVertex);
                    continue;
                };
                let Some(next) = vertex_by_id.get(&loop_vertices[next_index]) else {
                    blockers.push(BrepFaceAreaBlocker::MissingVertex);
                    continue;
                };
                signed_twice_projected_area = &signed_twice_projected_area
                    + projected_cross_component(&current.point, &next.point, projection_axis);
            }
        }

        let zero = Real::from(0);
        let normal_sign = match normal_component.partial_cmp(&zero) {
            Some(Ordering::Greater) => 1,
            Some(Ordering::Less) => -1,
            Some(Ordering::Equal) => {
                blockers.push(BrepFaceAreaBlocker::UnsupportedSurface);
                0
            }
            None => {
                blockers.push(BrepFaceAreaBlocker::UnknownNormalDirection);
                0
            }
        };
        if normal_sign < 0 {
            signed_twice_projected_area = &zero - &signed_twice_projected_area;
        }

        let area_order = signed_twice_projected_area.partial_cmp(&zero);
        let zero_area = area_order == Some(Ordering::Equal);
        let positive_area = area_order == Some(Ordering::Greater);
        let negative_area = area_order == Some(Ordering::Less);
        if area_order.is_none() {
            blockers.push(BrepFaceAreaBlocker::UnknownAreaSign);
        }
        let exact_area_ready = blockers.is_empty();

        Self {
            face,
            face_found: true,
            projection_axis: exact_area_ready.then_some(projection_axis),
            loop_count: source_face.loops().count(),
            boundary_vertex_count: unique_vertices.len(),
            signed_twice_projected_area: exact_area_ready.then_some(signed_twice_projected_area),
            zero_area,
            positive_area,
            negative_area,
            blockers,
            exact_area_ready,
        }
    }

    fn blocked(
        face: BrepFaceId,
        face_found: bool,
        mut blockers: Vec<BrepFaceAreaBlocker>,
        extra_blockers: Vec<BrepFaceAreaBlocker>,
    ) -> Self {
        blockers.extend(extra_blockers);
        Self {
            face,
            face_found,
            projection_axis: None,
            loop_count: 0,
            boundary_vertex_count: 0,
            signed_twice_projected_area: None,
            zero_area: false,
            positive_area: false,
            negative_area: false,
            blockers,
            exact_area_ready: false,
        }
    }
}

impl BrepShell {
    /// Derive exact projected area evidence for one retained planar face.
    pub fn face_area_report(&self, face: BrepFaceId) -> BrepFaceAreaReport {
        BrepFaceAreaReport::from_shell_face(self, face)
    }
}

fn axis_from_index(index: usize) -> BrepAreaProjectionAxis {
    match index {
        0 => BrepAreaProjectionAxis::X,
        1 => BrepAreaProjectionAxis::Y,
        2 => BrepAreaProjectionAxis::Z,
        _ => unreachable!("Point3 one-hot axis index must be in 0..3"),
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

fn projected_cross_component(
    current: &hyperlimit::Point3,
    next: &hyperlimit::Point3,
    axis: BrepAreaProjectionAxis,
) -> Real {
    match axis {
        BrepAreaProjectionAxis::X => &(&current.y * &next.z) - &(&current.z * &next.y),
        BrepAreaProjectionAxis::Y => &(&current.z * &next.x) - &(&current.x * &next.z),
        BrepAreaProjectionAxis::Z => &(&current.x * &next.y) - &(&current.y * &next.x),
    }
}
