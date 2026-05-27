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

use crate::exact::mesh::{ExactMesh, Triangle};

use super::ExactReal;
use super::kernel02::ExactKernel02Halfedge;

/// Exact boolmesh-style kernel input for one mesh.
#[derive(Clone, Debug, PartialEq)]
pub(super) struct ExactBoolMeshKernelFrame {
    /// Vertex positions in exact boolmesh working coordinates.
    pub points: Vec<Point3>,
    /// Source triangles in the frame's boolmesh working face order.
    pub triangles: Vec<Triangle>,
    /// Map from working face id to the original [`ExactMesh`] face id.
    pub source_faces: Vec<usize>,
    /// Map from working source halfedge id to the original face-local row.
    pub source_halfedges: Vec<usize>,
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
    let face_order = (0..mesh.triangles().len()).collect::<Vec<_>>();
    build_kernel_frame_with_face_order(mesh, &face_order)
}

/// Build the exact boolmesh frame with legacy manifold face scheduling.
///
/// Boolmesh's public `Manifold::new_impl` sorts source faces by a Morton code
/// before `boolean03::kernel12::intersect12` walks forward halfedges.  The row
/// order and even the edge/face pairs presented to `Kernel12::op` therefore
/// depend on that working order.  This exact scheduler ports the same idea
/// without primitive floats: centroid normalization, clamping, and 10-bit
/// Morton buckets are computed by exact comparisons.  Returned rows still carry
/// original source face ids so downstream `boolean45` can address the original
/// exact mesh.  This is Yap's exact-object boundary applied to a boolmesh
/// implementation detail: the scheduling object is replayed exactly before
/// topology tables are mutated.
pub(super) fn build_boolmesh_sorted_kernel_frame(mesh: &ExactMesh) -> ExactBoolMeshKernelFrame {
    let mut face_order = (0..mesh.triangles().len()).collect::<Vec<_>>();
    let morton = face_morton_codes(mesh);
    face_order.sort_by_key(|face| (morton[*face], *face));
    build_kernel_frame_with_face_order(mesh, &face_order)
}

fn build_kernel_frame_with_face_order(
    mesh: &ExactMesh,
    face_order: &[usize],
) -> ExactBoolMeshKernelFrame {
    let points = mesh
        .vertices()
        .iter()
        .map(|vertex| vertex.to_hyperlimit_point())
        .collect::<Vec<_>>();
    let triangles = face_order
        .iter()
        .filter_map(|face| mesh.triangles().get(*face).copied())
        .collect::<Vec<_>>();
    let source_count = triangles.len() * 3;
    let mut halfedges = Vec::with_capacity(source_count * 2);
    let mut source_halfedges = Vec::with_capacity(source_count);
    for (working_face, triangle) in triangles.iter().enumerate() {
        let original_face = face_order[working_face];
        for edge in 0..3 {
            halfedges.push(ExactKernel02Halfedge {
                tail: triangle.0[edge],
                head: triangle.0[(edge + 1) % 3],
                pair: usize::MAX,
            });
            source_halfedges.push(original_face * 3 + edge);
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
        triangles,
        source_faces: face_order.to_vec(),
        source_halfedges,
        halfedges,
        expansion_normals,
        paired_source_halfedges,
        boundary_source_halfedges,
        duplicate_directed_halfedges,
    }
}

fn face_morton_codes(mesh: &ExactMesh) -> Vec<u32> {
    let Some(bounds) = mesh.bounds().mesh.as_ref() else {
        return vec![0; mesh.triangles().len()];
    };
    mesh.triangles()
        .iter()
        .map(|triangle| {
            let vertices = triangle
                .0
                .iter()
                .filter_map(|vertex| mesh.vertices().get(*vertex))
                .map(|vertex| vertex.to_hyperlimit_point())
                .collect::<Vec<_>>();
            if vertices.len() != 3 {
                return u32::MAX;
            }
            let centroid = Point3::new(
                centroid_axis(&vertices[0].x, &vertices[1].x, &vertices[2].x),
                centroid_axis(&vertices[0].y, &vertices[1].y, &vertices[2].y),
                centroid_axis(&vertices[0].z, &vertices[1].z, &vertices[2].z),
            );
            exact_morton_code(&centroid, &bounds.min, &bounds.max)
        })
        .collect()
}

fn centroid_axis(a: &ExactReal, b: &ExactReal, c: &ExactReal) -> ExactReal {
    ((a.clone() + b.clone() + c.clone()) / ExactReal::from(3))
        .expect("constant centroid denominator is nonzero")
}

fn exact_morton_code(point: &Point3, min: &Point3, max: &Point3) -> u32 {
    let x = spread_bits_3(exact_morton_axis(&point.x, &min.x, &max.x));
    let y = spread_bits_3(exact_morton_axis(&point.y, &min.y, &max.y));
    let z = spread_bits_3(exact_morton_axis(&point.z, &min.z, &max.z));
    x * 4 + y * 2 + z
}

fn exact_morton_axis(value: &ExactReal, min: &ExactReal, max: &ExactReal) -> u32 {
    let span = max.clone() - min;
    if compare_reals(&span, &ExactReal::from(0)).value() != Some(Ordering::Greater) {
        return 0;
    }
    let scaled = ((value.clone() - min) * ExactReal::from(1024) / span)
        .expect("positive morton span is nonzero");
    if compare_reals(&scaled, &ExactReal::from(0)).value() != Some(Ordering::Greater) {
        return 0;
    }
    if compare_reals(&scaled, &ExactReal::from(1023)).value() != Some(Ordering::Less) {
        return 1023;
    }
    let mut lo = 0u32;
    let mut hi = 1023u32;
    while lo + 1 < hi {
        let mid = (lo + hi) / 2;
        if compare_reals(&scaled, &ExactReal::from(mid)).value() == Some(Ordering::Less) {
            hi = mid;
        } else {
            lo = mid;
        }
    }
    lo
}

fn spread_bits_3(mut value: u32) -> u32 {
    debug_assert!(value <= 1023);
    value = 0xFF0000FFu32 & value.wrapping_mul(0x00010001u32);
    value = 0x0F00F00Fu32 & value.wrapping_mul(0x00000101u32);
    value = 0xC30C30C3u32 & value.wrapping_mul(0x00000011u32);
    0x49249249u32 & value.wrapping_mul(0x00000005u32)
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

    fn tetrahedron_i64(a: [i64; 3], b: [i64; 3], c: [i64; 3], d: [i64; 3]) -> ExactMesh {
        ExactMesh::from_i64_triangles(
            &[
                a[0], a[1], a[2], b[0], b[1], b[2], c[0], c[1], c[2], d[0], d[1], d[2],
            ],
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

    #[test]
    fn sorted_frame_matches_boolmesh_morton_face_schedule_for_skew_tetrahedra() {
        let left = build_boolmesh_sorted_kernel_frame(&tetrahedron_i64(
            [0, 0, 0],
            [4, 0, 0],
            [0, 4, 0],
            [0, 0, 4],
        ));
        let right = build_boolmesh_sorted_kernel_frame(&tetrahedron_i64(
            [1, 1, -1],
            [3, 1, 3],
            [1, 3, 3],
            [3, 3, 1],
        ));

        assert_eq!(left.source_faces, vec![3, 1, 0, 2]);
        assert_eq!(right.source_faces, vec![0, 3, 1, 2]);
        assert_eq!(left.source_halfedges[9], 6);
        assert_eq!(right.source_halfedges[3], 9);
        assert_eq!(right.source_halfedges[6], 3);
    }
}
