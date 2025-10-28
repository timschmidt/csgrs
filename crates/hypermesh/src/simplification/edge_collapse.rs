use nalgebra::{RowVector3 as Row3};
use crate::common::{TriRef, is_ccw_3d};
use crate::Halfedge;
use super::{form_loops, next_of, remove_if_folded, HalfedgeOps};

// Check around a halfedges from the same tail vertex.
// If they consist of only two tris, then their edge is collapsable.
fn record_if_collinear(
    hs: &[Halfedge],
    rs: &[TriRef],
    hid: usize,
    nv: usize,
) -> bool {
    let h = &hs[hid];
    if h.no_pair() || (h.tail < nv) { return false; }

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

fn record_if_short(
    hs: &[Halfedge],
    ps: &[Row3<f64>],
    hid: usize,
    nv: usize,
    ep: f64,
) -> bool {
    let h = &hs[hid];
    if h.no_pair() || (h.tail < nv && h.head < nv) { return false; }
    let d = ps[hs[hid].head] - ps[hs[hid].tail];
    d.norm_squared() < ep.powi(2)

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

pub fn collapse_collinear_edges(
    hs: &mut Vec<Halfedge>,
    ps: &mut Vec<Row3<f64>>,
    ns: &mut [Row3<f64>],
    rs: &mut [TriRef],
    nv: usize,
    ep: f64
) {
    loop {
        let mut flag = 0;
        let rec = (0..hs.len())
            .filter(|&hid| record_if_collinear(hs, rs, hid, nv)).collect::<Vec<_>>();
        for hid in rec {
            if collapse_edge(hs, ps, ns, rs, hid, &mut vec![], ep) { flag += 1; }
        }
        if flag == 0 { break; }
    }
}

pub fn collapse_short_edges(
    hs: &mut Vec<Halfedge>,
    ps: &mut Vec<Row3<f64>>,
    ns: &mut [Row3<f64>],
    rs: &mut [TriRef],
    nv: usize,
    ep: f64
) {
    loop {
        let mut flag = 0;
        let rec = (0..hs.len()).filter(|&hid| record_if_short(hs, ps, hid, nv, ep)).collect::<Vec<_>>();
        for hid in rec {
            if collapse_edge(hs, ps, ns, rs, hid, &mut vec![], ep) { flag += 1; }
        }
        if flag == 0 { break; }
    }
}