//! Exact port of boolmesh `boolean03::kernel12::intersect12`.
//!
//! Legacy boolmesh schedules `Kernel12::op` by intersecting every forward
//! source halfedge AABB with the opposite mesh face collider.  This module
//! ports that scheduling loop directly over exact AABB facts and exact
//! `Kernel12::op` rows.  The broad phase is still only a scheduler: following
//! Yap, "Towards Exact Geometric Computation," *Computational Geometry* 7.1-2
//! (1997), exact boxes may reject disjoint work, but retained exact predicates
//! and accumulator witnesses decide topology.  The control flow intentionally
//! mirrors boolmesh `boolean03::kernel12::intersect12`; Moller (1997) and
//! Guigue and Devillers (2003) remain the narrow-phase triangle-intersection
//! substrate around this boolmesh stage.

#![allow(dead_code)]

use std::cmp::Ordering;

use hyperlimit::{Point3, PredicateOutcome, compare_reals};

use crate::exact::bounds::ExactAabb3;
use crate::exact::mesh::ExactMesh;

use super::kernel_frame::{ExactBoolMeshKernelFrame, build_kernel_frame};
use super::kernel02::ExactKernel02Halfedge;
use super::kernel12_op::{ExactKernel12Input, kernel12_op};
use super::{ExactBoolMeshEdgeFacePair, ExactBoolMeshFacePair, ExactBoolMeshSide, ExactReal};

/// Exact `intersect12` output before `boolean45` consumes source-edge runs.
#[derive(Clone, Debug, Default, PartialEq)]
pub(super) struct ExactKernel12IntersectTables {
    /// Left-source-edge/right-face accumulator rows.
    pub p1q2: Vec<ExactKernel12IntersectHit>,
    /// Right-source-edge/left-face accumulator rows.
    pub p2q1: Vec<ExactKernel12IntersectHit>,
}

/// One exact `Kernel12::op` row found by the boolmesh broad loop.
#[derive(Clone, Debug, PartialEq)]
pub(super) struct ExactKernel12IntersectHit {
    /// Source halfedge index in the boolmesh frame for the source operand.
    pub source_halfedge: usize,
    /// Opposite face index in the opposite operand.
    pub opposite_face: usize,
    /// Exact boolmesh edge/face ownership key.
    pub edge_face: ExactBoolMeshEdgeFacePair,
    /// Signed boolmesh `x12`/`x21` contribution.
    pub sign: i32,
    /// Exact boolmesh `v12`/`v21` witness.
    pub point: Point3,
    /// Exact source-edge parameter reconstructed from [`Self::point`].
    pub parameter: ExactReal,
}

/// Run the exact boolmesh `intersect12` loop in both directions.
pub(super) fn intersect12_exact(
    left_mesh: &ExactMesh,
    right_mesh: &ExactMesh,
) -> ExactKernel12IntersectTables {
    let left_frame = build_kernel_frame(left_mesh);
    let right_frame = build_kernel_frame(right_mesh);
    ExactKernel12IntersectTables {
        p1q2: intersect12_direction(left_mesh, right_mesh, &left_frame, &right_frame, true),
        p2q1: intersect12_direction(left_mesh, right_mesh, &left_frame, &right_frame, false),
    }
}

fn intersect12_direction(
    left_mesh: &ExactMesh,
    right_mesh: &ExactMesh,
    left_frame: &ExactBoolMeshKernelFrame,
    right_frame: &ExactBoolMeshKernelFrame,
    fwd: bool,
) -> Vec<ExactKernel12IntersectHit> {
    let (source_mesh, opposite_mesh, source_frame, opposite_frame) = if fwd {
        (left_mesh, right_mesh, left_frame, right_frame)
    } else {
        (right_mesh, left_mesh, right_frame, left_frame)
    };
    let opposite_face_bounds = face_bounds(opposite_mesh, opposite_frame);
    let expand = ExactReal::from(1);
    let input = ExactKernel12Input {
        ps_p: &left_frame.points,
        ps_q: &right_frame.points,
        hs_p: &left_frame.halfedges,
        hs_q: &right_frame.halfedges,
        ns_p: &left_frame.expansion_normals,
        ns_q: &right_frame.expansion_normals,
        expand: &expand,
        fwd,
    };

    let mut hits = Vec::new();
    for source_halfedge in 0..source_frame.source_halfedge_count() {
        let Some(source_half) = source_frame.halfedges.get(source_halfedge).copied() else {
            continue;
        };
        if !is_forward(source_half) {
            continue;
        }
        let Some(edge_bounds) = edge_bounds(&source_frame.points, source_half) else {
            continue;
        };
        for (opposite_face, face_bounds) in opposite_face_bounds.iter().enumerate() {
            if !broad_phase_keeps_pair(&edge_bounds, face_bounds) {
                continue;
            }
            let Some(hit) = kernel12_op(&input, source_halfedge, opposite_face) else {
                continue;
            };
            let Some(parameter) =
                source_edge_parameter(&source_frame.points, source_half, &hit.point)
            else {
                continue;
            };
            hits.push(ExactKernel12IntersectHit {
                source_halfedge,
                opposite_face,
                edge_face: edge_face_pair(
                    source_mesh,
                    source_halfedge,
                    source_half,
                    opposite_face,
                    fwd,
                ),
                sign: hit.sign,
                point: hit.point,
                parameter,
            });
        }
    }

    hits.sort_by(|left, right| {
        (left.source_halfedge, left.opposite_face)
            .cmp(&(right.source_halfedge, right.opposite_face))
    });
    hits
}

fn edge_face_pair(
    source_mesh: &ExactMesh,
    source_halfedge: usize,
    source_half: ExactKernel02Halfedge,
    opposite_face: usize,
    fwd: bool,
) -> ExactBoolMeshEdgeFacePair {
    let source_face = source_halfedge / 3;
    let edge = [source_half.tail, source_half.head];
    let face_pair = if fwd {
        ExactBoolMeshFacePair {
            left_face: source_face,
            right_face: opposite_face,
        }
    } else {
        ExactBoolMeshFacePair {
            left_face: opposite_face,
            right_face: source_face,
        }
    };
    debug_assert!(source_face < source_mesh.triangles().len());
    ExactBoolMeshEdgeFacePair {
        face_pair,
        edge_side: if fwd {
            ExactBoolMeshSide::Left
        } else {
            ExactBoolMeshSide::Right
        },
        source_halfedge,
        edge,
        face_side: if fwd {
            ExactBoolMeshSide::Right
        } else {
            ExactBoolMeshSide::Left
        },
        face: opposite_face,
    }
}

fn face_bounds(mesh: &ExactMesh, frame: &ExactBoolMeshKernelFrame) -> Vec<ExactAabb3> {
    mesh.triangles()
        .iter()
        .map(|triangle| {
            ExactAabb3::from_triangle([
                &frame.points[triangle.0[0]],
                &frame.points[triangle.0[1]],
                &frame.points[triangle.0[2]],
            ])
        })
        .collect()
}

fn edge_bounds(points: &[Point3], halfedge: ExactKernel02Halfedge) -> Option<ExactAabb3> {
    let tail = points.get(halfedge.tail)?;
    let head = points.get(halfedge.head)?;
    let mut bounds = ExactAabb3::point(tail);
    bounds.include(head);
    Some(bounds)
}

fn broad_phase_keeps_pair(edge: &ExactAabb3, face: &ExactAabb3) -> bool {
    match edge.classify_intersection(face) {
        PredicateOutcome::Decided { value, .. } => value.needs_narrow_phase(),
        PredicateOutcome::Unknown { .. } => true,
    }
}

fn source_edge_parameter(
    points: &[Point3],
    halfedge: ExactKernel02Halfedge,
    point: &Point3,
) -> Option<ExactReal> {
    let tail = points.get(halfedge.tail)?;
    let head = points.get(halfedge.head)?;
    let deltas = [
        sub(&head.x, &tail.x),
        sub(&head.y, &tail.y),
        sub(&head.z, &tail.z),
    ];
    let numerators = [
        sub(&point.x, &tail.x),
        sub(&point.y, &tail.y),
        sub(&point.z, &tail.z),
    ];
    for axis in 0..3 {
        if real_order(&deltas[axis], &ExactReal::from(0))? == Ordering::Equal {
            continue;
        }
        let parameter = (numerators[axis].clone() / deltas[axis].clone()).ok()?;
        if point_matches_edge_parameter(tail, head, point, &parameter) {
            return Some(parameter);
        }
        return None;
    }
    None
}

fn point_matches_edge_parameter(
    tail: &Point3,
    head: &Point3,
    point: &Point3,
    parameter: &ExactReal,
) -> bool {
    axis_matches_parameter(&tail.x, &head.x, &point.x, parameter)
        && axis_matches_parameter(&tail.y, &head.y, &point.y, parameter)
        && axis_matches_parameter(&tail.z, &head.z, &point.z, parameter)
}

fn axis_matches_parameter(
    tail: &ExactReal,
    head: &ExactReal,
    point: &ExactReal,
    parameter: &ExactReal,
) -> bool {
    let delta = sub(head, tail);
    let expected = tail.clone() + &mul(parameter, &delta);
    real_order(&expected, point) == Some(Ordering::Equal)
}

fn is_forward(halfedge: ExactKernel02Halfedge) -> bool {
    halfedge.tail < halfedge.head
}

fn real_order(left: &ExactReal, right: &ExactReal) -> Option<Ordering> {
    compare_reals(left, right).value()
}

fn sub(left: &ExactReal, right: &ExactReal) -> ExactReal {
    left.clone() - right
}

fn mul(left: &ExactReal, right: &ExactReal) -> ExactReal {
    left.clone() * right
}

#[cfg(feature = "internal-fuzzing")]
pub(super) fn internal_fuzz_probe(selector: u8) -> bool {
    if selector & 2 != 0 {
        let (left, right) = halfedge_row_key_meshes();
        let tables = intersect12_exact(&left, &right);
        return tables.p1q2.len() == 1
            && tables.p2q1.is_empty()
            && tables.p1q2[0].source_halfedge == 1
            && tables.p1q2[0].edge_face.source_halfedge == 1;
    }
    let top = 5 + i64::from(selector & 1);
    let (left, right) = open_crossing_meshes(top, 4);
    let tables = intersect12_exact(&left, &right);
    tables.p1q2.len() == 1
        && tables.p2q1.is_empty()
        && tables.p1q2[0].sign == 1
        && real_order(&tables.p1q2[0].point.z, &ExactReal::from(4)) == Some(Ordering::Equal)
}

#[cfg(any(test, feature = "internal-fuzzing"))]
fn halfedge_row_key_meshes() -> (ExactMesh, ExactMesh) {
    use crate::exact::SourceProvenance;
    use crate::exact::mesh::{ExactPoint3, Triangle};
    use crate::exact::validation::ValidationPolicy;

    let left = ExactMesh::new_with_policy(
        vec![
            ExactPoint3::new(ExactReal::from(1), ExactReal::from(1), ExactReal::from(0)),
            ExactPoint3::new(ExactReal::from(1), ExactReal::from(1), ExactReal::from(5)),
            ExactPoint3::new(ExactReal::from(2), ExactReal::from(1), ExactReal::from(5)),
        ],
        vec![Triangle([2, 0, 1])],
        SourceProvenance::exact("exact boolmesh intersect12 halfedge row fixture"),
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let right = ExactMesh::new_with_policy(
        vec![
            ExactPoint3::new(ExactReal::from(0), ExactReal::from(0), ExactReal::from(4)),
            ExactPoint3::new(ExactReal::from(4), ExactReal::from(0), ExactReal::from(4)),
            ExactPoint3::new(ExactReal::from(0), ExactReal::from(4), ExactReal::from(4)),
        ],
        vec![Triangle([0, 1, 2])],
        SourceProvenance::exact("exact boolmesh intersect12 halfedge row opposite"),
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    (left, right)
}

#[cfg(any(test, feature = "internal-fuzzing"))]
fn open_crossing_meshes(top: i64, plane_z: i64) -> (ExactMesh, ExactMesh) {
    use crate::exact::SourceProvenance;
    use crate::exact::mesh::{ExactPoint3, Triangle};
    use crate::exact::validation::ValidationPolicy;

    let left = ExactMesh::new_with_policy(
        vec![
            ExactPoint3::new(ExactReal::from(1), ExactReal::from(1), ExactReal::from(0)),
            ExactPoint3::new(ExactReal::from(1), ExactReal::from(1), ExactReal::from(top)),
            ExactPoint3::new(ExactReal::from(2), ExactReal::from(1), ExactReal::from(top)),
        ],
        vec![Triangle([0, 1, 2])],
        SourceProvenance::exact("exact boolmesh intersect12 source fixture"),
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let right = ExactMesh::new_with_policy(
        vec![
            ExactPoint3::new(
                ExactReal::from(0),
                ExactReal::from(0),
                ExactReal::from(plane_z),
            ),
            ExactPoint3::new(
                ExactReal::from(4),
                ExactReal::from(0),
                ExactReal::from(plane_z),
            ),
            ExactPoint3::new(
                ExactReal::from(0),
                ExactReal::from(4),
                ExactReal::from(plane_z),
            ),
        ],
        vec![Triangle([0, 1, 2])],
        SourceProvenance::exact("exact boolmesh intersect12 opposite fixture"),
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    (left, right)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn intersect12_loop_lowers_forward_edge_face_row_without_event_log() {
        let (left, right) = open_crossing_meshes(5, 4);
        let tables = intersect12_exact(&left, &right);

        assert_eq!(tables.p1q2.len(), 1);
        assert!(tables.p2q1.is_empty());
        let hit = &tables.p1q2[0];
        assert_eq!(hit.source_halfedge, 0);
        assert_eq!(hit.opposite_face, 0);
        assert_eq!(hit.edge_face.edge_side, ExactBoolMeshSide::Left);
        assert_eq!(hit.edge_face.source_halfedge, 0);
        assert_eq!(hit.edge_face.edge, [0, 1]);
        assert_eq!(hit.edge_face.face_side, ExactBoolMeshSide::Right);
        assert_eq!(hit.edge_face.face, 0);
        assert_eq!(hit.sign, 1);
        assert_eq!(
            real_order(&hit.point.x, &ExactReal::from(1)),
            Some(Ordering::Equal)
        );
        assert_eq!(
            real_order(&hit.point.y, &ExactReal::from(1)),
            Some(Ordering::Equal)
        );
        assert_eq!(
            real_order(&hit.point.z, &ExactReal::from(4)),
            Some(Ordering::Equal)
        );
        assert_eq!(
            real_order(
                &hit.parameter,
                &(ExactReal::from(4) / ExactReal::from(5)).unwrap()
            ),
            Some(Ordering::Equal)
        );
    }

    #[test]
    fn intersect12_loop_rejects_exactly_disjoint_edge_face_bounds() {
        let (left, right) = open_crossing_meshes(5, 9);
        let tables = intersect12_exact(&left, &right);

        assert!(tables.p1q2.is_empty());
        assert!(tables.p2q1.is_empty());
    }

    #[test]
    fn intersect12_loop_retains_boolmesh_source_halfedge_row_key() {
        let (left, right) = halfedge_row_key_meshes();

        let tables = intersect12_exact(&left, &right);

        assert_eq!(tables.p1q2.len(), 1);
        let hit = &tables.p1q2[0];
        assert_eq!(hit.source_halfedge, 1);
        assert_eq!(hit.edge_face.source_halfedge, 1);
        assert_eq!(hit.edge_face.edge, [0, 1]);
        assert_eq!(hit.edge_face.face_pair.left_face, 0);
        assert_eq!(hit.edge_face.face_pair.right_face, 0);
    }
}
