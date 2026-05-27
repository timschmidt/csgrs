//! Exact boolmesh working-frame construction.
//!
//! Legacy boolmesh kernels operate over a `Manifold` package: projected point
//! coordinates, three halfedges per source face plus paired reverse
//! halfedges, and per-vertex expansion directions.  This module builds the
//! exact counterpart from [`ExactMesh`](crate::exact::mesh::ExactMesh) so the
//! ported `Kernel02`, `Kernel11`, and `Kernel12::op` code can run against the
//! same algorithmic input model without passing through primitive `f64`.
//!
//! The halfedge layout mirrors `manifold::hmesh`: source face halfedges occupy
//! slots `3 * face + local_edge`, and their `pair` field names the opposite
//! directed source halfedge when it exists.  Expansion directions deliberately
//! keep exact unnormalized incident face-plane normals.  In legacy boolmesh the
//! smoothed vertex normals are only used as simulation-of-simplicity directions
//! in shadow ties; removing trigonometric angle weights and unit-length
//! normalization is the exact-arithmetic analogue of Yap, "Towards Exact
//! Geometric Computation," *Computational Geometry* 7.1-2 (1997): predicates
//! consume retained exact object facts instead of reintroducing rounded
//! representatives.

#![allow(dead_code)]

use std::cmp::Ordering;
use std::collections::BTreeMap;

use hyperlimit::{Point3, compare_reals};

use crate::exact::mesh::ExactMesh;

use super::ExactReal;
use super::kernel02::ExactKernel02Halfedge;

/// Exact boolmesh-style kernel input for one mesh.
#[derive(Clone, Debug, PartialEq)]
pub(super) struct ExactBoolMeshKernelFrame {
    /// Vertex positions in exact boolmesh working coordinates.
    pub points: Vec<Point3>,
    /// Directed halfedges, three source halfedges per face plus reverse rows.
    pub halfedges: Vec<ExactKernel02Halfedge>,
    /// Exact expansion directions indexed by source vertex.
    pub expansion_normals: Vec<Point3>,
    /// Number of source-face halfedges with a paired reverse row.
    pub paired_source_halfedges: usize,
    /// Number of source-face halfedges without an opposite use.
    pub boundary_source_halfedges: usize,
    /// Number of directed edge keys seen more than once.
    pub duplicate_directed_halfedges: usize,
}

impl ExactBoolMeshKernelFrame {
    /// Number of face-local halfedges before any appended boundary reverse rows.
    pub(super) fn source_halfedge_count(&self) -> usize {
        self.paired_source_halfedges + self.boundary_source_halfedges
    }

    /// Resolve a retained face-local edge into the boolmesh halfedge row.
    ///
    /// Legacy boolmesh `Kernel12::op` is addressed by a source halfedge index
    /// and an opposite face index.  Exact discovery stores the geometric edge
    /// as vertex endpoints instead.  This method is the exact-object replay
    /// bridge: it maps the retained face/edge fact back onto the boolmesh
    /// halfedge layout without re-running a floating orientation heuristic.
    /// When the retained edge is opposite the source face order, the paired
    /// reverse row is returned, matching boolmesh's forward-halfedge lowering
    /// while keeping Yap's "Towards Exact Geometric Computation,"
    /// *Computational Geometry* 7.1-2 (1997), separation between certified
    /// input facts and topology mutation.
    pub(super) fn source_halfedge_for_face_edge(
        &self,
        face: usize,
        edge: [usize; 2],
    ) -> Option<usize> {
        let base = face.checked_mul(3)?;
        for local in 0..3 {
            let index = base + local;
            let halfedge = *self.halfedges.get(index)?;
            if [halfedge.tail, halfedge.head] == edge {
                return Some(index);
            }
            if [halfedge.head, halfedge.tail] == edge {
                let pair = halfedge.pair;
                return self.halfedges.get(pair).map(|_| pair);
            }
        }
        None
    }
}

/// Build the exact boolmesh kernel frame for one mesh.
///
/// The first `3 * mesh.triangles().len()` halfedges are always face-local
/// source halfedges in triangle order.  Reverse rows are appended only as pair
/// targets, matching the way legacy boolmesh can ask a backward face edge for
/// its forward partner through `pair`.
pub(super) fn build_kernel_frame(mesh: &ExactMesh) -> ExactBoolMeshKernelFrame {
    let points = mesh
        .vertices()
        .iter()
        .map(|vertex| vertex.to_hyperlimit_point())
        .collect::<Vec<_>>();
    let source_count = mesh.triangles().len() * 3;
    let mut halfedges = Vec::with_capacity(source_count * 2);
    for triangle in mesh.triangles() {
        for edge in 0..3 {
            halfedges.push(ExactKernel02Halfedge {
                tail: triangle.0[edge],
                head: triangle.0[(edge + 1) % 3],
                pair: usize::MAX,
            });
        }
    }

    let mut directed = BTreeMap::<[usize; 2], Vec<usize>>::new();
    for (index, halfedge) in halfedges.iter().enumerate() {
        directed
            .entry([halfedge.tail, halfedge.head])
            .or_default()
            .push(index);
    }

    let mut paired_source_halfedges = 0usize;
    let mut boundary_source_halfedges = 0usize;
    let duplicate_directed_halfedges = directed
        .values()
        .filter(|indices| indices.len() > 1)
        .map(|indices| indices.len() - 1)
        .sum::<usize>();

    for source in 0..source_count {
        if halfedges[source].pair != usize::MAX {
            continue;
        }
        let reverse_key = [halfedges[source].head, halfedges[source].tail];
        let reverse_source = directed
            .get(&reverse_key)
            .and_then(|indices| indices.iter().copied().find(|index| *index != source));
        if let Some(reverse_source) = reverse_source {
            halfedges[source].pair = reverse_source;
            paired_source_halfedges += 1;
        } else {
            let pair = halfedges.len();
            halfedges[source].pair = pair;
            halfedges.push(ExactKernel02Halfedge {
                tail: halfedges[source].head,
                head: halfedges[source].tail,
                pair: source,
            });
            boundary_source_halfedges += 1;
        }
    }

    let expansion_normals = expansion_normals_from_faces(mesh);
    ExactBoolMeshKernelFrame {
        points,
        halfedges,
        expansion_normals,
        paired_source_halfedges,
        boundary_source_halfedges,
        duplicate_directed_halfedges,
    }
}

#[cfg(feature = "internal-fuzzing")]
pub(super) fn internal_fuzz_probe(selector: u8) -> bool {
    let height = 1 + i64::from(selector % 2);
    let mesh = ExactMesh::from_i64_triangles(
        &[0, 0, 0, 4, 0, 0, 0, 4, 0, 0, 0, height],
        &[0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3],
    )
    .expect("deterministic exact boolmesh frame fixture must import");
    let frame = build_kernel_frame(&mesh);
    frame.points.len() == 4
        && frame.halfedges.len() == 12
        && frame.paired_source_halfedges == 12
        && frame.boundary_source_halfedges == 0
        && frame.duplicate_directed_halfedges == 0
        && frame
            .expansion_normals
            .iter()
            .all(|normal| !point_is_zero(normal))
}

fn expansion_normals_from_faces(mesh: &ExactMesh) -> Vec<Point3> {
    let mut normals = vec![zero_point(); mesh.vertices().len()];
    let mut first_incident_normal = vec![None::<Point3>; mesh.vertices().len()];
    for (face, triangle) in mesh.triangles().iter().enumerate() {
        let normal = face_normal(mesh, face);
        for vertex in triangle.0 {
            if let Some(slot) = normals.get_mut(vertex) {
                *slot = add_points(slot, &normal);
            }
            if let Some(first) = first_incident_normal.get_mut(vertex) {
                first.get_or_insert_with(|| normal.clone());
            }
        }
    }

    for (vertex, normal) in normals.iter_mut().enumerate() {
        if point_is_zero(normal) {
            *normal = first_incident_normal[vertex].clone().unwrap_or_else(|| {
                Point3::new(ExactReal::from(1), ExactReal::from(1), ExactReal::from(1))
            });
        }
    }
    normals
}

fn face_normal(mesh: &ExactMesh, face: usize) -> Point3 {
    let normal = &mesh.facts().faces[face].plane.normal;
    Point3::new(normal[0].clone(), normal[1].clone(), normal[2].clone())
}

fn add_points(left: &Point3, right: &Point3) -> Point3 {
    Point3::new(
        left.x.clone() + &right.x,
        left.y.clone() + &right.y,
        left.z.clone() + &right.z,
    )
}

fn zero_point() -> Point3 {
    Point3::new(ExactReal::from(0), ExactReal::from(0), ExactReal::from(0))
}

fn point_is_zero(point: &Point3) -> bool {
    real_is_zero(&point.x) && real_is_zero(&point.y) && real_is_zero(&point.z)
}

fn real_is_zero(value: &ExactReal) -> bool {
    compare_reals(value, &ExactReal::from(0)).value() == Some(Ordering::Equal)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::exact::SourceProvenance;
    use crate::exact::mesh::{ExactPoint3, Triangle};
    use crate::exact::validation::ValidationPolicy;

    fn tetrahedron() -> ExactMesh {
        ExactMesh::from_i64_triangles(
            &[0, 0, 0, 4, 0, 0, 0, 4, 0, 0, 0, 4],
            &[0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3],
        )
        .unwrap()
    }

    #[test]
    fn frame_builds_boolmesh_halfedge_pairs_for_closed_tetrahedron() {
        let frame = build_kernel_frame(&tetrahedron());

        assert_eq!(frame.points.len(), 4);
        assert_eq!(frame.halfedges.len(), 12);
        assert_eq!(frame.paired_source_halfedges, 12);
        assert_eq!(frame.boundary_source_halfedges, 0);
        assert_eq!(frame.duplicate_directed_halfedges, 0);
        for index in 0..12 {
            let pair = frame.halfedges[index].pair;
            assert!(pair < 12);
            assert_eq!(frame.halfedges[pair].pair, index);
            assert_eq!(frame.halfedges[index].tail, frame.halfedges[pair].head);
            assert_eq!(frame.halfedges[index].head, frame.halfedges[pair].tail);
        }
    }

    #[test]
    fn frame_appends_reverse_rows_for_boundary_edges() {
        let mesh = ExactMesh::new_with_policy(
            vec![
                ExactPoint3::new(ExactReal::from(0), ExactReal::from(0), ExactReal::from(0)),
                ExactPoint3::new(ExactReal::from(4), ExactReal::from(0), ExactReal::from(0)),
                ExactPoint3::new(ExactReal::from(0), ExactReal::from(4), ExactReal::from(0)),
            ],
            vec![Triangle([0, 1, 2])],
            SourceProvenance::exact("exact boolmesh frame boundary fixture"),
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let frame = build_kernel_frame(&mesh);

        assert_eq!(frame.halfedges.len(), 6);
        assert_eq!(frame.paired_source_halfedges, 0);
        assert_eq!(frame.boundary_source_halfedges, 3);
        for index in 0..3 {
            let pair = frame.halfedges[index].pair;
            assert!(pair >= 3);
            assert_eq!(frame.halfedges[pair].pair, index);
        }
    }

    #[test]
    fn frame_derives_nonzero_exact_expansion_directions() {
        let frame = build_kernel_frame(&tetrahedron());

        assert_eq!(frame.expansion_normals.len(), 4);
        assert!(
            frame
                .expansion_normals
                .iter()
                .all(|normal| !point_is_zero(normal))
        );
        assert!(
            frame
                .expansion_normals
                .iter()
                .any(|normal| !real_is_zero(&normal.z)),
            "at least one expansion vector must preserve an exact z tie direction"
        );
    }

    #[test]
    fn frame_resolves_retained_face_edges_to_boolmesh_rows() {
        let frame = build_kernel_frame(&tetrahedron());

        assert_eq!(
            frame.source_halfedge_for_face_edge(0, [0, 2]),
            Some(0),
            "face-local direction should use the source halfedge row"
        );
        let reverse = frame
            .source_halfedge_for_face_edge(0, [2, 0])
            .expect("closed tetrahedron reverse edge must be paired");
        assert_ne!(reverse, 0);
        assert_eq!(frame.halfedges[reverse].tail, 2);
        assert_eq!(frame.halfedges[reverse].head, 0);
        assert!(
            frame
                .source_halfedge_for_face_edge(usize::MAX, [0, 1])
                .is_none()
        );
        assert!(
            frame
                .source_halfedge_for_face_edge(0, [0, usize::MAX])
                .is_none()
        );
    }
}
