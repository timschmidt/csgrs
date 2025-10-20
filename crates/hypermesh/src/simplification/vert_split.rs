use crate::Halfedge;
use nalgebra::{RowVector3 as Row3};
use super::HalfedgeOps;

fn split_pinched_vert(
    hs: &mut [Halfedge],
    ps: &mut Vec<Row3<f64>>
) {
    let mut v_processed = vec![false; ps.len()];
    let mut h_processed = vec![false; hs.len()];

    for hid in 0..hs.len() {
        if h_processed[hid] { continue; }
        let mut vid = hs[hid].tail;
        if vid == usize::MAX { continue; }
        if v_processed[vid] {
            ps.push(ps[vid]);
            vid = ps.len() - 1;
        } else { v_processed[vid] = true; }

        // loop halfedges around their tail ccw way
        let mut cur = hid;
        loop {
            cur = hs.next_hid_of(hs[cur].pair);
            h_processed[cur] = true;
            hs[cur].tail = vid;
            hs[hs[cur].pair].head = vid;
            if cur == hid { break; }
        }
    }
}
