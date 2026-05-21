//! Exact face AABB evidence.
//!
//! Bounds are derived from retained BREP vertices referenced by face loops.
//! They are useful scheduling evidence for voxel, physics, packing, pathing,
//! and mesh consumers, but they are not a substitute for trim or surface
//! validation.

use std::cmp::Ordering;
use std::collections::{BTreeMap, BTreeSet};

use hyperlimit::{Aabb3Intersection, Point3, PredicateOutcome, PreparedAabb3};
use hyperreal::Real;

use crate::topology::{
    BrepEdge, BrepEdgeOrientation, BrepFaceId, BrepShell, BrepVertex, BrepVertexId,
};

/// Explicit blocker for face bounds.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub enum BrepFaceBoundsBlocker {
    /// Source face was not found in the shell.
    MissingFace,
    /// The face has no coedges from which vertices can be collected.
    EmptyBoundary,
    /// A coedge references an edge id not present in the shell.
    MissingEdge,
    /// A referenced edge starts and ends at the same vertex.
    DegenerateEdge,
    /// A referenced edge points at a vertex id not present in the shell.
    MissingVertex,
    /// Coordinate comparison could not be certified exactly enough to order an
    /// AABB axis.
    UnknownCoordinateOrdering,
}

/// Explicit blocker for face/face AABB preflight.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub enum BrepFaceAabbPreflightBlocker {
    /// The first face bounds were not ready.
    FirstBoundsNotReady,
    /// The second face bounds were not ready.
    SecondBoundsNotReady,
    /// `hyperlimit` could not decide the AABB relation.
    UnknownAabbRelation,
}

/// Explicit blocker for shell bounds.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub enum BrepShellBoundsBlocker {
    /// The shell has no retained vertices.
    EmptyShell,
    /// Coordinate comparison could not be certified exactly enough to order an
    /// AABB axis.
    UnknownCoordinateOrdering,
}

/// Exact AABB/support facts for a retained face.
#[derive(Clone, Debug, PartialEq)]
pub struct BrepFaceBoundsReport {
    /// Face that was requested.
    pub face: BrepFaceId,
    /// Whether the face exists in the shell.
    pub face_found: bool,
    /// Number of unique vertex ids referenced by the face loops.
    pub vertex_count: usize,
    /// Exact minimum corner when all coordinate orderings were certified.
    pub min: Option<Point3>,
    /// Exact maximum corner when all coordinate orderings were certified.
    pub max: Option<Point3>,
    /// Whether the AABB has zero extent along x.
    pub zero_x_extent: bool,
    /// Whether the AABB has zero extent along y.
    pub zero_y_extent: bool,
    /// Whether the AABB has zero extent along z.
    pub zero_z_extent: bool,
    /// Number of zero-extent axes.
    pub zero_extent_axis_count: usize,
    /// Explicit blockers discovered while deriving bounds.
    pub blockers: Vec<BrepFaceBoundsBlocker>,
    /// Whether the exact AABB is available for downstream consumers.
    pub exact_bounds_ready: bool,
}

/// Borrowed prepared face AABB.
#[derive(Clone, Debug)]
pub struct PreparedBrepFaceBounds<'a> {
    /// Face whose bounds were prepared.
    pub face: BrepFaceId,
    /// Prepared `hyperlimit` AABB predicate object.
    pub prepared: PreparedAabb3<'a>,
}

/// Exact AABB/support facts for a retained shell.
#[derive(Clone, Debug, PartialEq)]
pub struct BrepShellBoundsReport {
    /// Number of retained vertices.
    pub vertex_count: usize,
    /// Number of retained faces.
    pub face_count: usize,
    /// Exact minimum corner when all coordinate orderings were certified.
    pub min: Option<Point3>,
    /// Exact maximum corner when all coordinate orderings were certified.
    pub max: Option<Point3>,
    /// Number of zero-extent axes.
    pub zero_extent_axis_count: usize,
    /// Explicit blockers.
    pub blockers: Vec<BrepShellBoundsBlocker>,
    /// Whether the exact shell AABB is available for downstream consumers.
    pub exact_bounds_ready: bool,
}

/// Borrowed prepared shell AABB.
#[derive(Clone, Debug)]
pub struct PreparedBrepShellBounds<'a> {
    /// Prepared `hyperlimit` AABB predicate object.
    pub prepared: PreparedAabb3<'a>,
}

/// Broad-phase face/face AABB preflight report.
#[derive(Clone, Debug, PartialEq)]
pub struct BrepFaceAabbPreflightReport {
    /// First face.
    pub first: BrepFaceId,
    /// Second face.
    pub second: BrepFaceId,
    /// Exact bounds report for the first face.
    pub first_bounds: BrepFaceBoundsReport,
    /// Exact bounds report for the second face.
    pub second_bounds: BrepFaceBoundsReport,
    /// Decided exact AABB relation when available.
    pub relation: Option<Aabb3Intersection>,
    /// Whether the boxes are certified disjoint.
    pub certified_disjoint: bool,
    /// Whether the boxes have contact or overlap and require narrow-phase replay.
    pub requires_narrow_phase: bool,
    /// Explicit blockers.
    pub blockers: Vec<BrepFaceAabbPreflightBlocker>,
    /// Whether this preflight is ready for broad-phase scheduling.
    pub preflight_ready: bool,
}

impl BrepFaceBoundsReport {
    /// Derive exact face bounds from retained topology and vertex coordinates.
    ///
    /// The report follows Yap, "Towards Exact Geometric Computation,"
    /// *Computational Geometry* 7.1-2 (1997): AABB coordinates are accepted
    /// only when coordinate orderings are certified by `hyperreal::Real`.
    /// Unknown orderings remain explicit blockers instead of becoming
    /// primitive-float min/max tolerances. The topology source is the retained
    /// BREP edge-use graph described by Mäntylä, *An Introduction to Solid
    /// Modeling* (1988).
    pub fn from_shell_face(shell: &BrepShell, face: BrepFaceId) -> Self {
        let Some(source_face) = shell.faces.iter().find(|candidate| candidate.id == face) else {
            return Self::blocked(
                face,
                false,
                Vec::new(),
                vec![BrepFaceBoundsBlocker::MissingFace],
            );
        };
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

        let mut blockers = Vec::new();
        let mut vertex_ids = BTreeSet::new();
        for face_loop in source_face.loops() {
            for coedge in &face_loop.coedges {
                let Some(edge) = edge_by_id.get(&coedge.edge) else {
                    blockers.push(BrepFaceBoundsBlocker::MissingEdge);
                    continue;
                };
                if edge.is_degenerate() {
                    blockers.push(BrepFaceBoundsBlocker::DegenerateEdge);
                }
                let (start, end) = oriented_endpoints(*edge, coedge.orientation);
                vertex_ids.insert(start);
                vertex_ids.insert(end);
            }
        }

        if vertex_ids.is_empty() {
            blockers.push(BrepFaceBoundsBlocker::EmptyBoundary);
        }

        let mut vertices = Vec::with_capacity(vertex_ids.len());
        for vertex_id in vertex_ids {
            let Some(vertex) = vertex_by_id.get(&vertex_id) else {
                blockers.push(BrepFaceBoundsBlocker::MissingVertex);
                continue;
            };
            vertices.push(*vertex);
        }

        let Some((min, max)) = bounds_from_vertices(&vertices, &mut blockers) else {
            return Self::blocked(face, true, blockers, Vec::new());
        };
        let zero_x_extent = min.x == max.x;
        let zero_y_extent = min.y == max.y;
        let zero_z_extent = min.z == max.z;
        let zero_extent_axis_count =
            zero_x_extent as usize + zero_y_extent as usize + zero_z_extent as usize;
        let exact_bounds_ready = blockers.is_empty();

        Self {
            face,
            face_found: true,
            vertex_count: vertices.len(),
            min: exact_bounds_ready.then_some(min),
            max: exact_bounds_ready.then_some(max),
            zero_x_extent,
            zero_y_extent,
            zero_z_extent,
            zero_extent_axis_count,
            blockers,
            exact_bounds_ready,
        }
    }

    /// Prepare the exact AABB for repeated `hyperlimit` bound predicates.
    pub fn prepare(&self) -> Option<PreparedBrepFaceBounds<'_>> {
        let min = self.min.as_ref()?;
        let max = self.max.as_ref()?;
        Some(PreparedBrepFaceBounds {
            face: self.face,
            prepared: PreparedAabb3::new(min, max),
        })
    }

    fn blocked(
        face: BrepFaceId,
        face_found: bool,
        mut blockers: Vec<BrepFaceBoundsBlocker>,
        extra_blockers: Vec<BrepFaceBoundsBlocker>,
    ) -> Self {
        blockers.extend(extra_blockers);
        Self {
            face,
            face_found,
            vertex_count: 0,
            min: None,
            max: None,
            zero_x_extent: false,
            zero_y_extent: false,
            zero_z_extent: false,
            zero_extent_axis_count: 0,
            blockers,
            exact_bounds_ready: false,
        }
    }
}

impl BrepShellBoundsReport {
    /// Derive exact shell bounds from retained vertex coordinates.
    ///
    /// This is a broad-phase scheduling artifact. Per Yap, "Towards Exact
    /// Geometric Computation," *Computational Geometry* 7.1-2 (1997), the box
    /// is exact only when all coordinate orderings are certified; otherwise the
    /// report remains explicitly blocked instead of falling back to a lossy
    /// primitive-float box.
    pub fn from_shell(shell: &BrepShell) -> Self {
        if shell.vertices.is_empty() {
            return Self {
                vertex_count: 0,
                face_count: shell.faces.len(),
                min: None,
                max: None,
                zero_extent_axis_count: 0,
                blockers: vec![BrepShellBoundsBlocker::EmptyShell],
                exact_bounds_ready: false,
            };
        }

        let mut face_blockers = Vec::new();
        let vertices = shell.vertices.iter().collect::<Vec<_>>();
        let Some((min, max)) = bounds_from_vertices(&vertices, &mut face_blockers) else {
            return Self {
                vertex_count: shell.vertices.len(),
                face_count: shell.faces.len(),
                min: None,
                max: None,
                zero_extent_axis_count: 0,
                blockers: vec![BrepShellBoundsBlocker::EmptyShell],
                exact_bounds_ready: false,
            };
        };
        let blockers = face_blockers
            .into_iter()
            .filter_map(|blocker| match blocker {
                BrepFaceBoundsBlocker::UnknownCoordinateOrdering => {
                    Some(BrepShellBoundsBlocker::UnknownCoordinateOrdering)
                }
                BrepFaceBoundsBlocker::MissingFace
                | BrepFaceBoundsBlocker::EmptyBoundary
                | BrepFaceBoundsBlocker::MissingEdge
                | BrepFaceBoundsBlocker::DegenerateEdge
                | BrepFaceBoundsBlocker::MissingVertex => None,
            })
            .collect::<Vec<_>>();
        let zero_extent_axis_count =
            (min.x == max.x) as usize + (min.y == max.y) as usize + (min.z == max.z) as usize;
        let exact_bounds_ready = blockers.is_empty();
        Self {
            vertex_count: shell.vertices.len(),
            face_count: shell.faces.len(),
            min: exact_bounds_ready.then_some(min),
            max: exact_bounds_ready.then_some(max),
            zero_extent_axis_count,
            blockers,
            exact_bounds_ready,
        }
    }

    /// Prepare the exact shell AABB for repeated `hyperlimit` bound predicates.
    pub fn prepare(&self) -> Option<PreparedBrepShellBounds<'_>> {
        let min = self.min.as_ref()?;
        let max = self.max.as_ref()?;
        Some(PreparedBrepShellBounds {
            prepared: PreparedAabb3::new(min, max),
        })
    }
}

impl BrepShell {
    /// Derive exact AABB/support facts for one face.
    pub fn face_bounds_report(&self, face: BrepFaceId) -> BrepFaceBoundsReport {
        BrepFaceBoundsReport::from_shell_face(self, face)
    }

    /// Derive exact AABB/support facts for the whole shell.
    pub fn shell_bounds_report(&self) -> BrepShellBoundsReport {
        BrepShellBoundsReport::from_shell(self)
    }

    /// Classify two face AABBs as broad-phase scheduling evidence.
    ///
    /// A disjoint result can reject expensive downstream work. Touching or
    /// overlapping boxes remain only a candidate signal and must replay through
    /// face/trim/surface predicates before changing topology. This follows
    /// Yap, "Towards Exact Geometric Computation," *Computational Geometry*
    /// 7.1-2 (1997): approximate or broad-phase filters may accelerate the
    /// stack, but they do not replace exact combinatorial decisions.
    pub fn face_aabb_preflight(
        &self,
        first: BrepFaceId,
        second: BrepFaceId,
    ) -> BrepFaceAabbPreflightReport {
        let first_bounds = self.face_bounds_report(first);
        let second_bounds = self.face_bounds_report(second);
        let mut blockers = Vec::new();
        if !first_bounds.exact_bounds_ready {
            blockers.push(BrepFaceAabbPreflightBlocker::FirstBoundsNotReady);
        }
        if !second_bounds.exact_bounds_ready {
            blockers.push(BrepFaceAabbPreflightBlocker::SecondBoundsNotReady);
        }

        let mut relation = None;
        if blockers.is_empty() {
            let first_prepared = first_bounds.prepare().expect("checked bounds readiness");
            let second_prepared = second_bounds.prepare().expect("checked bounds readiness");
            match first_prepared
                .prepared
                .classify_intersection(&second_prepared.prepared)
            {
                PredicateOutcome::Decided { value, .. } => {
                    relation = Some(value);
                }
                PredicateOutcome::Unknown { .. } => {
                    blockers.push(BrepFaceAabbPreflightBlocker::UnknownAabbRelation);
                }
            }
        }

        let certified_disjoint = relation == Some(Aabb3Intersection::Disjoint);
        let requires_narrow_phase = matches!(
            relation,
            Some(Aabb3Intersection::Touching | Aabb3Intersection::Overlapping)
        );
        let preflight_ready = blockers.is_empty();
        BrepFaceAabbPreflightReport {
            first,
            second,
            first_bounds,
            second_bounds,
            relation,
            certified_disjoint,
            requires_narrow_phase,
            blockers,
            preflight_ready,
        }
    }
}

fn bounds_from_vertices(
    vertices: &[&BrepVertex],
    blockers: &mut Vec<BrepFaceBoundsBlocker>,
) -> Option<(Point3, Point3)> {
    let first = vertices.first()?;
    let mut min = first.point.clone();
    let mut max = first.point.clone();
    for vertex in vertices.iter().skip(1) {
        update_axis_min(&mut min.x, &vertex.point.x, blockers);
        update_axis_min(&mut min.y, &vertex.point.y, blockers);
        update_axis_min(&mut min.z, &vertex.point.z, blockers);
        update_axis_max(&mut max.x, &vertex.point.x, blockers);
        update_axis_max(&mut max.y, &vertex.point.y, blockers);
        update_axis_max(&mut max.z, &vertex.point.z, blockers);
    }
    Some((min, max))
}

fn update_axis_min(
    current: &mut Real,
    candidate: &Real,
    blockers: &mut Vec<BrepFaceBoundsBlocker>,
) {
    match candidate.partial_cmp(current) {
        Some(Ordering::Less) => *current = candidate.clone(),
        Some(Ordering::Equal | Ordering::Greater) => {}
        None => blockers.push(BrepFaceBoundsBlocker::UnknownCoordinateOrdering),
    }
}

fn update_axis_max(
    current: &mut Real,
    candidate: &Real,
    blockers: &mut Vec<BrepFaceBoundsBlocker>,
) {
    match candidate.partial_cmp(current) {
        Some(Ordering::Greater) => *current = candidate.clone(),
        Some(Ordering::Equal | Ordering::Less) => {}
        None => blockers.push(BrepFaceBoundsBlocker::UnknownCoordinateOrdering),
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
