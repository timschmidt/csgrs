use nalgebra::{RowVector3 as Row3};
use crate::boolean46::TriRef;
use crate::common::{is_ccw_3d};
use crate::Halfedge;
use super::{form_loops, next_of, remove_if_folded, HalfedgeOps};

// Check around a halfedges from the same tail vertex.
// If they consist of only two tris, then their edge is collapsable.
fn record(
    hs: &[Halfedge],
    rs: &[TriRef],
    hid: usize,
    oft: usize,
) -> bool {
    let h = &hs[hid];
    if h.no_pair() || (h.tail < oft) { return false; }

    let cw_next = |i: usize| next_of(hs[i].pair);

    let     bgn = hid;
    let mut cur = cw_next(bgn);
    let     tr0 = &rs[bgn / 3];
    let mut tr1 = &rs[cur / 3];
    let mut same = tr0.same_face(tr1);
    while cur != bgn {
        cur = cw_next(cur);
        let tr2 = &rs[cur / 3];
        if !tr2.same_face(tr0) && !tr2.same_face(tr1) {
            if same { tr1 = tr2; same = false; }
            else { return false; }
        }
    }
    true
}

pub fn collapse_edge(
    hs: &mut Vec<Halfedge>,
    ps: &mut Vec<Row3<f64>>,
    ns: &mut [Row3<f64>],
    rs: &mut [TriRef],
    hid: usize,
    store: &mut Vec<usize>, // storing the halfedge data for form_loops
    epsilon: f64,
) -> bool {

    let to_rmv = &hs[hid];
    if to_rmv.no_pair() { return false; }

    let vid_keep = to_rmv.head;
    let vid_delt = to_rmv.tail;
    let pos_keep = ps[vid_keep];
    let pos_delt = ps[vid_delt];

    let tri0 = hs.tri_hids_of(hid);
    let tri1 = hs.tri_hids_of(to_rmv.pair);

    let mut bgn = hs.pair_hid_of(tri1.1); // the bgn half heading delt vert
    let     end = tri0.2;                 // the end half heading delt vert

    // check validity by orbiting start vert ccw order
    if (pos_keep - pos_delt).norm_squared() >= epsilon.powi(2) {
        let mut cur = bgn;
        let mut tr0 = &rs[to_rmv.pair / 3];
        let mut p_prev = ps[hs.head_vid_of(tri1.1)];
        while cur != to_rmv.pair {
            cur = hs.next_hid_of(cur); // incoming half around delt vert
            let p_next = ps[hs.head_vid_of(cur)];
            let r_curr = &rs[cur / 3];
            let n_curr = &ns[cur / 3];
            let n_pair = &ns[to_rmv.pair / 3];
            let ccw = |p0, p1, p2| is_ccw_3d(p0, p1, p2, n_curr, epsilon);
            if !r_curr.same_face(&tr0) {
                let tr2 = tr0;
                tr0 = &rs[hid / 3];
                if !r_curr.same_face(&tr0) { return false; }
                if tr0.mesh_id != tr2.mesh_id ||
                   tr0.face_id != tr2.face_id ||
                   n_pair.dot(n_curr) < -0.5 {
                    // Restrict collapse to co-linear edges when the edge separates faces or the edge is sharp.
                    // This ensures large shifts are not introduced parallel to the tangent plane.
                    if ccw(&p_prev, &pos_delt, &pos_keep) != 0 { return false; }
                }
            }

            // Don't collapse edge if it would cause a triangle to invert
            if ccw(&p_next, &p_prev, &pos_keep) < 0 { return false; }

            p_prev = p_next;
            cur = hs.pair_hid_of(cur); // outgoing half around delt vert
        }
    }

    // find a candidate by orbiting end verts ccw order
    let mut cur = hs.pair_hid_of(tri0.1);
    while cur != tri1.2 {
        cur = hs.next_hid_of(cur);
        store.push(cur); // storing outgoing half here
        cur = hs.pair_hid_of(cur);
    }

    ps[to_rmv.tail] = Row3::new(f64::MAX, f64::MAX, f64::MAX);
    hs.collapse_triangle(&tri1);

    let mut cur = bgn;
    while cur != end {
        cur      = hs.next_hid_of(cur);
        let pair = hs.pair_hid_of(cur);
        let head = hs.head_vid_of(cur);
        if let Some((i, &v)) = store.iter().enumerate().find(|&(_, &s)| hs.head_vid_of(s) == head) {
            form_loops(hs, ps, v, cur);
            bgn = pair;
            store.truncate(i);
        }
        cur = pair;
    }

    // do collapse
    hs.update_vid_around_star(bgn, end, vid_keep);
    hs.collapse_triangle(&tri0);
    remove_if_folded(hs, ps, bgn);

    true
}

pub fn collapse_collinear_edge(
    hs: &mut Vec<Halfedge>,
    ps: &mut Vec<Row3<f64>>,
    ns: &mut [Row3<f64>],
    rs: &mut [TriRef],
    nv: usize,
    ep: f64
) {
    loop {
        let mut flag = 0;
        let rec = (0..hs.len()).filter(|&hid| record(hs, rs, hid, nv)).collect::<Vec<_>>();
        for hid in rec {
            if collapse_edge(hs, ps, ns, rs, hid, &mut vec![], ep) { flag += 1; }
        }
        if flag == 0 { break; }
    }
}

#[test]
fn test_record() {
    let mut hs = vec![
        Halfedge::new(0, 4, 5),
        Halfedge::new(4, 1, 42),
        Halfedge::new(1, 0, 9),
        Halfedge::new(0, 2, 26),
        Halfedge::new(2, 4, 50),
        Halfedge::new(4, 0, 0),
        Halfedge::new(0, 11, 11),
        Halfedge::new(11, 9, 57),
        Halfedge::new(9, 0, 21),
        Halfedge::new(0, 1, 2),
        Halfedge::new(1, 11, 14),
        Halfedge::new(11, 0, 6),
        Halfedge::new(1, 10, 17),
        Halfedge::new(10, 11, 68),
        Halfedge::new(11, 1, 10),
        Halfedge::new(1, 3, 35),
        Halfedge::new(3, 10, 20),
        Halfedge::new(10, 1, 12),
        Halfedge::new(3, 13, 31),
        Halfedge::new(13, 10, 66),
        Halfedge::new(10, 3, 16),
        Halfedge::new(0, 9, 8),
        Halfedge::new(9, 12, 59),
        Halfedge::new(12, 0, 24),
        Halfedge::new(0, 12, 23),
        Halfedge::new(12, 2, 27),
        Halfedge::new(2, 0, 3),
        Halfedge::new(2, 12, 25),
        Halfedge::new(12, 13, 64),
        Halfedge::new(13, 2, 30),
        Halfedge::new(2, 13, 29),
        Halfedge::new(13, 3, 18),
        Halfedge::new(3, 2, 36),
        Halfedge::new(1, 7, 47),
        Halfedge::new(7, 3, 37),
        Halfedge::new(3, 1, 15),
        Halfedge::new(2, 3, 32),
        Halfedge::new(3, 7, 34),
        Halfedge::new(7, 2, 51),
        Halfedge::new(4, 6, 49),
        Halfedge::new(6, 5, 54),
        Halfedge::new(5, 4, 43),
        Halfedge::new(1, 4, 1),
        Halfedge::new(4, 5, 41),
        Halfedge::new(5, 1, 45),
        Halfedge::new(1, 5, 44),
        Halfedge::new(5, 7, 56),
        Halfedge::new(7, 1, 33),
        Halfedge::new(2, 6, 53),
        Halfedge::new(6, 4, 39),
        Halfedge::new(4, 2, 4),
        Halfedge::new(2, 7, 38),
        Halfedge::new(7, 6, 55),
        Halfedge::new(6, 2, 48),
        Halfedge::new(5, 6, 40),
        Halfedge::new(6, 7, 52),
        Halfedge::new(7, 5, 46),
        Halfedge::new(9, 11, 7),
        Halfedge::new(11, 12, 61),
        Halfedge::new(12, 9, 22),
        Halfedge::new(8, 12, 65),
        Halfedge::new(12, 11, 58),
        Halfedge::new(11, 8, 69),
        Halfedge::new(8, 13, 71),
        Halfedge::new(13, 12, 28),
        Halfedge::new(12, 8, 60),
        Halfedge::new(10, 13, 19),
        Halfedge::new(13, 11, 70),
        Halfedge::new(11, 10, 13),
        Halfedge::new(8, 11, 62),
        Halfedge::new(11, 13, 67),
        Halfedge::new(13, 8, 63),
    ];

    let mut ps = vec![
        Row3::new(-0.8, -1., -1.),
        Row3::new(-0.8, -1., 1.),
        Row3::new(-0.8, 1., -1.),
        Row3::new(-0.8, 1., 1.),
        Row3::new(1.2, -1., -1.),
        Row3::new(1.2, -1., 1.),
        Row3::new(1.2, 1., -1.),
        Row3::new(1.2, 1., 1.),
        Row3::new(0., 0., 0.),
        Row3::new(-0.8, -0.14641, -0.14641),
        Row3::new(-0.8, 0.2, 0.2),
        Row3::new(-0.8, -0.34641, 0.2),
        Row3::new(-0.8, 0., -0.4),
        Row3::new(-0.8, 0.34641, 0.2),
    ];

    let mut ns = vec![
        Row3::new(-0., -1., -0.),
        Row3::new(-0., -0., -1.),
        Row3::new(-1., -0., 0.),
        Row3::new(-1., -0., 0.),
        Row3::new(-1., -0., 0.),
        Row3::new(-1., -0., 0.),
        Row3::new(-1., -0., -0.),
        Row3::new(-1., -0., -0.),
        Row3::new(-1., -0., -0.),
        Row3::new(-1., -0., -0.),
        Row3::new(-1., -0., -0.),
        Row3::new(0., 0., 1.),
        Row3::new(0., 1., 0.),
        Row3::new(1., 0., 0.),
        Row3::new(0., -1., 0.),
        Row3::new(-0., 0., 1.),
        Row3::new(0., 0., -1.),
        Row3::new(-0., 1., 0.),
        Row3::new(1., 0., -0.),
        Row3::new(-0.242536, 0.840168, 0.485071),
        Row3::new(-0.242536, 0.840168, 0.485071),
        Row3::new(-0.242536, -0.840168, 0.485071),
        Row3::new(-0.242536, -3.22877e-20, -0.970143),
        Row3::new(-0.242536, -3.22877e-20, -0.970143),
    ];

    let mut refs = vec![
        TriRef{origin_id: -1, mesh_id: 1, face_id: 0, coplanar_id: 3},
        TriRef{origin_id: -1, mesh_id: 1, face_id: 0, coplanar_id: 5},
        TriRef{origin_id: -1, mesh_id: 1, face_id: 0, coplanar_id: 2},
        TriRef{origin_id: -1, mesh_id: 1, face_id: 0, coplanar_id: 2},
        TriRef{origin_id: -1, mesh_id: 1, face_id: 0, coplanar_id: 2},
        TriRef{origin_id: -1, mesh_id: 1, face_id: 0, coplanar_id: 2},
        TriRef{origin_id: -1, mesh_id: 1, face_id: 0, coplanar_id: 2},
        TriRef{origin_id: -1, mesh_id: 1, face_id: 0, coplanar_id: 2},
        TriRef{origin_id: -1, mesh_id: 1, face_id: 0, coplanar_id: 2},
        TriRef{origin_id: -1, mesh_id: 1, face_id: 0, coplanar_id: 2},
        TriRef{origin_id: -1, mesh_id: 1, face_id: 0, coplanar_id: 2},
        TriRef{origin_id: -1, mesh_id: 1, face_id: 0, coplanar_id: 1},
        TriRef{origin_id: -1, mesh_id: 1, face_id: 0, coplanar_id: 0},
        TriRef{origin_id: -1, mesh_id: 1, face_id: 0, coplanar_id: 4},
        TriRef{origin_id: -1, mesh_id: 1, face_id: 0, coplanar_id: 3},
        TriRef{origin_id: -1, mesh_id: 1, face_id: 0, coplanar_id: 1},
        TriRef{origin_id: -1, mesh_id: 1, face_id: 0, coplanar_id: 5},
        TriRef{origin_id: -1, mesh_id: 1, face_id: 0, coplanar_id: 0},
        TriRef{origin_id: -1, mesh_id: 1, face_id: 0, coplanar_id: 4},
        TriRef{origin_id: -1, mesh_id: 2, face_id: 0, coplanar_id: 0},
        TriRef{origin_id: -1, mesh_id: 2, face_id: 0, coplanar_id: 0},
        TriRef{origin_id: -1, mesh_id: 2, face_id: 0, coplanar_id: 3},
        TriRef{origin_id: -1, mesh_id: 2, face_id: 0, coplanar_id: 2},
        TriRef{origin_id: -1, mesh_id: 2, face_id: 0, coplanar_id: 2},
    ];

    collapse_collinear_edge(
        &mut hs,
        &mut ps,
        &mut ns,
        &mut refs,
        9,
        1e-6
    );

    let hs_out = vec![
        (0, 4, 5),
        (4, 1, 42),
        (1, 0, 9),
        (0, 2, 26),
        (2, 4, 50),
        (4, 0, 0),
        (0, 11, 11),
        (11, 12, 61),
        (12, 0, 24),
        (0, 1, 2),
        (1, 11, 17),
        (11, 0, 6),
        (-1, -1, -1),
        (-1, -1, -1),
        (-1, -1, -1),
        (1, 3, 35),
        (3, 11, 20),
        (11, 1, 10),
        (3, 13, 31),
        (13, 11, 70),
        (11, 3, 16),
        (-1, -1, -1),
        (-1, -1, -1),
        (-1, -1, -1),
        (0, 12, 8),
        (12, 2, 27),
        (2, 0, 3),
        (2, 12, 25),
        (12, 13, 64),
        (13, 2, 30),
        (2, 13, 29),
        (13, 3, 18),
        (3, 2, 36),
        (1, 7, 47),
        (7, 3, 37),
        (3, 1, 15),
        (2, 3, 32),
        (3, 7, 34),
        (7, 2, 51),
        (4, 6, 49),
        (6, 5, 54),
        (5, 4, 43),
        (1, 4, 1),
        (4, 5, 41),
        (5, 1, 45),
        (1, 5, 44),
        (5, 7, 56),
        (7, 1, 33),
        (2, 6, 53),
        (6, 4, 39),
        (4, 2, 4),
        (2, 7, 38),
        (7, 6, 55),
        (6, 2, 48),
        (5, 6, 40),
        (6, 7, 52),
        (7, 5, 46),
        (-1, -1, -1),
        (-1, -1, -1),
        (-1, -1, -1),
        (8, 12, 65),
        (12, 11, 7),
        (11, 8, 69),
        (8, 13, 71),
        (13, 12, 28),
        (12, 8, 60),
        (-1, -1, -1),
        (-1, -1, -1),
        (-1, -1, -1),
        (8, 11, 62),
        (11, 13, 19),
        (13, 8, 63),
    ];

    let ps_out = vec![
        Row3::new(-0.8, -1., -1.),
        Row3::new(-0.8, -1., 1.),
        Row3::new(-0.8, 1., -1.),
        Row3::new(-0.8, 1., 1.),
        Row3::new(1.2, -1., -1.),
        Row3::new(1.2, -1., 1.),
        Row3::new(1.2, 1., -1.),
        Row3::new(1.2, 1., 1.),
        Row3::new(0., 0., 0.),
        Row3::new(f64::MAX, f64::MAX, f64::MAX),
        Row3::new(f64::MAX, f64::MAX, f64::MAX),
        Row3::new(-0.8, -0.34641, 0.2),
        Row3::new(-0.8, 0., -0.4),
        Row3::new(-0.8, 0.34641, 0.2),
    ];

    for i in 0..hs.len() {
        let h = &hs[i];
        let a = hs[i].tail;
        let b = hs[i].head;
        let c = hs[i].pair;
        let (d, e, f) = hs_out[i];
        if h.no_tail() { assert_eq!(d, -1); } else { assert_eq!(a as i32, d); }
        if h.no_head() { assert_eq!(e, -1); } else { assert_eq!(b as i32, e); }
        if h.no_pair() { assert_eq!(f, -1); } else { assert_eq!(c as i32, f); }
    }

    for i in 0..ps.len() {
        let p0 = &ps[i];
        let p1 = &ps_out[i];
        if p0.x == f64::MAX && p0.y == f64::MAX && p0.z == f64::MAX { assert_eq!(p1.x, f64::MAX); }
        else { assert!((p0 - p1).norm() < 1e-6); }
    }
}