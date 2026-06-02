//! Exact port of boolmesh `boolean03::kernel03`.
//!
//! Legacy boolmesh classifies retained source vertices by querying every
//! source point against candidate opposite faces, running `Kernel02::op`, and
//! accumulating the signed shadow contribution into `w03` or `w30`.  This
//! module ports that dataflow directly over exact `hyperreal`/`hyperlimit`
//! objects: the query filter is exact projected face bounds, the per-row
//! predicate is the exact [`super::kernel02::kernel02_op`] port, and the
//! output is the same integer counter consumed by `boolean45::size_output`.
//!
//! The implementation intentionally follows boolmesh's published kernel path
//! instead of the earlier axis-ray fallback.  Ties are handled through the
//! exact expansion directions in [`super::kernel_frame`], preserving the
//! simulation-of-simplicity role that boolmesh assigns to vertex normals while
//! exact predicates and constructions.

#![allow(dead_code)]

use std::cmp::Ordering;

use hyperlimit::{Point3, compare_reals};

use crate::exact::mesh::ExactMesh;

use super::kernel_frame::{ExactBoolMeshKernelFrame, build_kernel_frame};
use super::kernel02::{ExactKernel02Input, kernel02_op};
use hyperreal::Real;

/// Bidirectional `kernel03` winding counters.
#[derive(Clone, Debug, Eq, PartialEq)]
pub(super) struct ExactKernel03Winding {
    /// Left source vertices classified against the right mesh, legacy `w03`.
    pub(super) w03: Vec<i32>,
    /// Right source vertices classified against the left mesh, legacy `w30`.
    pub(super) w30: Vec<i32>,
}

/// Run exact boolmesh `kernel03` in both operand directions.
///
/// Boolmesh runs `winding03(mp, mq, expand, true)` for `w03` and
/// `winding03(mp, mq, expand, false)` for `w30`.  The exact port keeps that
/// canonical operand model so the reverse direction can reuse the canonical
/// left expansion directions for tie-breaking exactly like legacy `Kernel02`.
/// Malformed halfedge frames remain blockers.  Boundary halfedges are accepted:
/// boolmesh supplies paired reverse rows for open source edges, and the exact
/// frame does the same with replayable boundary rows so `Kernel02` can own
/// boundary endpoint and interval ties without a separate ray-style fallback.
pub(super) fn kernel03_winding(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Option<ExactKernel03Winding> {
    let left_frame = build_kernel_frame(left);
    let right_frame = build_kernel_frame(right);
    if !kernel03_frame_is_replayable(left, &left_frame)
        || !kernel03_frame_is_replayable(right, &right_frame)
    {
        return None;
    }

    let expand = Real::from(1);
    let w03 = winding03_exact(&left_frame, &right_frame, &expand, true)?;
    let w30 = winding03_exact(&left_frame, &right_frame, &expand, false)?;

    Some(ExactKernel03Winding { w03, w30 })
}

/// Port boolmesh `winding03` for one direction.
///
/// `fwd == true` classifies canonical-left vertices against canonical-right
/// faces and accumulates `+s02`; `fwd == false` classifies canonical-right
/// vertices against canonical-left faces and accumulates `-s02`.  This is the
/// same sign convention legacy boolmesh uses before `boolean45::size_output`
/// applies operation coefficients for union, intersection, and difference.
fn winding03_exact(
    left: &ExactBoolMeshKernelFrame,
    right: &ExactBoolMeshKernelFrame,
    expand: &Real,
    fwd: bool,
) -> Option<Vec<i32>> {
    let subject = if fwd { left } else { right };
    let target = if fwd { right } else { left };
    let subject_normals = if fwd {
        &left.expansion_normals
    } else {
        &right.expansion_normals
    };
    let target_normals = if fwd {
        &right.expansion_normals
    } else {
        &left.expansion_normals
    };

    let input = ExactKernel02Input {
        ps_p: &subject.points,
        ps_q: &target.points,
        hs_q: &target.halfedges,
        ns_p: subject_normals,
        ns_q: target_normals,
        expand,
        fwd,
    };

    let mut winding = vec![0i32; subject.points.len()];
    for (vertex, winding_value) in winding.iter_mut().enumerate().take(subject.points.len()) {
        let point = subject.points.get(vertex)?;
        for face in 0..(target.source_halfedge_count() / 3) {
            if !point_in_face_xy_bounds(point, target, face)? {
                continue;
            }
            if let Some(hit) = kernel02_op(&input, vertex, face) {
                *winding_value += hit.sign * if fwd { 1 } else { -1 };
            }
        }
    }
    Some(winding)
}

/// Validate that a frame is faithful enough for boolmesh `kernel03` replay.
///
/// Triangle meshes should have exactly three source halfedges per face and no
/// duplicate directed source rows.  Boundary reverse rows may be appended after
/// the source range; accepting them ports boolmesh's open-boundary halfedge
/// object model while still rejecting source-row drift before `boolean45`.
fn kernel03_frame_is_replayable(mesh: &ExactMesh, frame: &ExactBoolMeshKernelFrame) -> bool {
    let source_halfedges = mesh.triangles().len() * 3;
    frame.points.len() == mesh.vertices().len()
        && frame.source_halfedge_count() == source_halfedges
        && frame.duplicate_directed_halfedges == 0
        && frame.expansion_normals.len() == mesh.vertices().len()
}

/// Exact counterpart of the boolmesh point/face broad query.
///
/// The legacy collider tests a point query in projected `x/y` against opposite
/// face bounds before invoking `Kernel02::op`.  The filter is conservative:
/// boundary equality is included so exact edge and vertex shadows still reach
/// the kernel row that owns their tie-breaking decision.
fn point_in_face_xy_bounds(
    point: &Point3,
    frame: &ExactBoolMeshKernelFrame,
    face: usize,
) -> Option<bool> {
    let base = face.checked_mul(3)?;
    let mut min_x = None::<&Real>;
    let mut max_x = None::<&Real>;
    let mut min_y = None::<&Real>;
    let mut max_y = None::<&Real>;

    for local in 0..3 {
        let halfedge = *frame.halfedges.get(base + local)?;
        let vertex = frame.points.get(halfedge.tail)?;
        min_x = choose_min(min_x, &vertex.x);
        max_x = choose_max(max_x, &vertex.x);
        min_y = choose_min(min_y, &vertex.y);
        max_y = choose_max(max_y, &vertex.y);
    }

    let min_x = min_x?;
    let max_x = max_x?;
    let min_y = min_y?;
    let max_y = max_y?;
    Some(
        compare_reals(&point.x, min_x).value()? != Ordering::Less
            && compare_reals(&point.x, max_x).value()? != Ordering::Greater
            && compare_reals(&point.y, min_y).value()? != Ordering::Less
            && compare_reals(&point.y, max_y).value()? != Ordering::Greater,
    )
}

fn choose_min<'a>(current: Option<&'a Real>, candidate: &'a Real) -> Option<&'a Real> {
    match current {
        Some(current)
            if compare_reals(candidate, current)
                .value()
                .is_some_and(|ordering| ordering != Ordering::Less) =>
        {
            Some(current)
        }
        Some(_) | None => Some(candidate),
    }
}

fn choose_max<'a>(current: Option<&'a Real>, candidate: &'a Real) -> Option<&'a Real> {
    match current {
        Some(current)
            if compare_reals(candidate, current)
                .value()
                .is_some_and(|ordering| ordering != Ordering::Greater) =>
        {
            Some(current)
        }
        Some(_) | None => Some(candidate),
    }
}

#[cfg(feature = "internal-fuzzing")]
pub(super) fn internal_fuzz_probe(selector: u8) -> bool {
    let offset = i64::from(selector % 2);
    let inner = tetrahedron_i64(
        [1 + offset, 1, 1],
        [2 + offset, 1, 1],
        [1 + offset, 2, 1],
        [1 + offset, 1, 2],
    );
    let outer = tetrahedron_i64([0, 0, 0], [10, 0, 0], [0, 10, 0], [0, 0, 10]);
    let separated = tetrahedron_i64([20, 0, 0], [22, 0, 0], [20, 2, 0], [20, 0, 2]);
    let boundary_upper = tetrahedron_i64([1, 1, 0], [2, 1, 0], [1, 2, 0], [1, 1, 2]);
    let boundary_lower = tetrahedron_i64([0, 0, 0], [4, 0, 0], [0, 4, 0], [0, 0, -4]);

    let nested = kernel03_winding(&inner, &outer).is_some_and(|winding| {
        winding.w03 == vec![1; inner.vertices().len()]
            && winding.w30 == vec![0; outer.vertices().len()]
    });
    let apart = kernel03_winding(&inner, &separated).is_some_and(|winding| {
        winding.w03 == vec![0; inner.vertices().len()]
            && winding.w30 == vec![0; separated.vertices().len()]
    });
    let boundary = kernel03_winding(&boundary_upper, &boundary_lower).is_some_and(|winding| {
        winding.w03 == vec![-1, 0, 0, 0] && winding.w30 == vec![0; boundary_lower.vertices().len()]
    });
    nested && apart && boundary
}

#[cfg(any(test, feature = "internal-fuzzing"))]
fn tetrahedron_i64(a: [i64; 3], b: [i64; 3], c: [i64; 3], d: [i64; 3]) -> ExactMesh {
    ExactMesh::from_i64_triangles(
        &[
            a[0], a[1], a[2], b[0], b[1], b[2], c[0], c[1], c[2], d[0], d[1], d[2],
        ],
        &[0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3],
    )
    .expect("deterministic kernel03 tetrahedron fixture must import")
}

#[cfg(test)]
mod tests {
    use crate::exact::ValidationPolicy;

    use super::*;

    #[test]
    fn winding03_classifies_nested_closed_tetrahedra() {
        let inner = tetrahedron_i64([1, 1, 1], [2, 1, 1], [1, 2, 1], [1, 1, 2]);
        let outer = tetrahedron_i64([0, 0, 0], [10, 0, 0], [0, 10, 0], [0, 0, 10]);

        let winding = kernel03_winding(&inner, &outer).expect("closed nested meshes classify");
        assert_eq!(winding.w03, vec![1; inner.vertices().len()]);
        assert_eq!(winding.w30, vec![0; outer.vertices().len()]);
    }

    #[test]
    fn winding03_replays_open_boundary_source_frame() {
        let open = ExactMesh::from_i64_triangles_with_policy(
            &[0, 0, 0, 4, 0, 0, 0, 4, 0],
            &[0, 1, 2],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .expect("open triangle fixture must import");
        let closed = tetrahedron_i64([0, 0, 0], [10, 0, 0], [0, 10, 0], [0, 0, 10]);

        let open_closed =
            kernel03_winding(&open, &closed).expect("open source boundary rows should replay");
        assert_eq!(open_closed.w03, vec![1, 1, 1]);
        assert_eq!(open_closed.w30, vec![0; closed.vertices().len()]);

        let closed_open =
            kernel03_winding(&closed, &open).expect("open target boundary rows should replay");
        assert_eq!(closed_open.w03, vec![0; closed.vertices().len()]);
        assert_eq!(closed_open.w30, vec![0; open.vertices().len()]);
    }

    #[test]
    fn winding03_keeps_boundary_vertices_as_signed_boolmesh_counters() {
        let lower = tetrahedron_i64([0, 0, 0], [4, 0, 0], [0, 4, 0], [0, 0, -4]);
        let upper_vertex_on_base = tetrahedron_i64([1, 1, 0], [2, 1, 0], [1, 2, 0], [1, 1, 2]);

        let winding = kernel03_winding(&upper_vertex_on_base, &lower)
            .expect("closed coplanar-boundary mesh should emit exact boolmesh counters");
        assert_eq!(winding.w03, vec![-1, 0, 0, 0]);
        assert_eq!(winding.w30, vec![0; lower.vertices().len()]);
    }
}
