use nalgebra::{RowVector2, RowVector3};
use crate::common::{Halfedge, Tref, next_of};
mod edge_dedup;
mod edge_swap;
mod edge_collapse;
mod test;
use edge_collapse::*;
type Row3f = RowVector3<f64>;
type Row2f = RowVector2<f64>;


pub fn simplify_topology(
    hs: &mut Vec<Halfedge>,
    ps: &mut Vec<Row3f>,
    ns: &mut [Row3f],
    rs: &mut [Tref],
    nv: usize,
    ep: f64,
) {
    split_pinched_vert(hs, ps);
    collapse_short_edges(hs, ps, ns, rs, nv, ep);
    collapse_collinear_edges(hs, ps, ns, rs, nv, ep);
}

pub(in crate::simplification) fn is01_longest_2d(
    p0: &Row2f,
    p1: &Row2f,
    p2: &Row2f
) -> bool {
    let e01 = (*p1 - *p0).norm_squared();
    let e12 = (*p2 - *p1).norm_squared();
    let e20 = (*p0 - *p2).norm_squared();
    e01 > e12 && e01 > e20
}

// When bgn halfedge and end halfedge are heading to the same vertex,
// and if collapsing the tail vertex as well, this function creates two loops.
// Beware the needless loop is not necessarily eliminated from the mesh because
// halfedges of the tail side might be connected to other triangles (would be folded though).
pub(in crate::simplification) fn form_loops(
    hs: &mut Vec<Halfedge>,
    ps: &mut Vec<Row3f>,
    bgn: usize,
    end: usize
) {
    ps.push(ps[hs.tail_vid_of(bgn)]);
    ps.push(ps[hs.head_vid_of(bgn)]);
    let bgn_vid = ps.len() - 2;
    let end_vid = ps.len() - 1;

    let bgn_pair = hs.pair_hid_of(bgn);
    let end_pair = hs.pair_hid_of(end);

    hs.update_vid_around_star(bgn_pair, end_pair, bgn_vid);
    hs.update_vid_around_star(end, bgn, end_vid);

    hs[bgn].pair = end_pair;
    hs[end_pair].pair = bgn;
    hs[end].pair = bgn_pair;
    hs[bgn_pair].pair = end;

    remove_if_folded(hs, ps, end);
}

// Removing fold paired triangles from the mesh. The process is either of:
// 1. Non-2-manifold, two triangles are completely isolated from the mesh
// 2. Non-2-manifold, one vertex is isolated from the mesh
// 3. Topologically valid, but just two vertex positions are the same
// Beware the case that triangles only connected at a vertex but not by halfedge
// are eliminated by split_pinched_vert and dedupe_edges functions.
pub(in crate::simplification) fn remove_if_folded(
    hs: &mut [Halfedge],
    ps: &mut Vec<Row3f>,
    hid: usize
) {
    let (i0, i1, i2) = hs.tri_hids_of(hid);
    let (j0, j1, j2) = hs.tri_hids_of(hs.pair_hid_of(hid));

    if hs[i1].no_pair() || hs.head_vid_of(i1) != hs.head_vid_of(j1) { return; }

    let nan = Row3f::new(f64::MAX, f64::MAX, f64::MAX);
    match (hs.pair_hid_of(i1) == j2, hs.pair_hid_of(i2) == j1) {
        (true, true)  => for i in [i0, i1, i2] { ps[hs.tail_vid_of(i)] = nan; },
        (true, false) => { ps[hs.tail_vid_of(i1)] = nan; }
        (false, true) => { ps[hs.tail_vid_of(j1)] = nan; }
        _ => {} // topo valid
    }
    hs.pair_up(hs[i1].pair, hs[j2].pair);
    hs.pair_up(hs[i2].pair, hs[j1].pair);
    for i in [i0, i1, i2] { hs[i] = Halfedge::default(); }
    for j in [j0, j1, j2] { hs[j] = Halfedge::default(); }
}


fn split_pinched_vert(
    hs: &mut [Halfedge],
    ps: &mut Vec<Row3f>
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

pub(in crate::simplification) trait HalfedgeOps {
    fn next_of(&self, curr: usize) -> &Halfedge;
    fn pair_of(&self, curr: usize) -> &Halfedge;
    fn next_hid_of(&self, curr: usize) -> usize;
    fn pair_hid_of(&self, curr: usize) -> usize;
    fn head_vid_of(&self, curr: usize) -> usize;
    fn tail_vid_of(&self, curr: usize) -> usize;
    fn tri_hids_of(&self, curr: usize) -> (usize, usize, usize);
    fn pair_up(&mut self, hid0: usize, hid1: usize);
    fn update_vid_around_star(&mut self, vid: usize, bgn: usize, end: usize);
    fn collapse_triangle(&mut self, hids: &(usize, usize, usize));
    fn loop_ccw<F>(&mut self, hid: usize, func: F) where F: FnMut(&mut [Halfedge], usize);
}

impl HalfedgeOps for [Halfedge] {
    fn next_of(&self, curr: usize) -> &Halfedge { &self[self.next_hid_of(curr)] }
    fn pair_of(&self, curr: usize) -> &Halfedge { &self[self[curr].pair] }
    fn next_hid_of(&self, curr: usize) -> usize { let mut c = curr + 1; if c % 3 == 0 { c -= 3; } c }
    fn pair_hid_of(&self, curr: usize) -> usize { self[curr].pair }
    fn head_vid_of(&self, curr: usize) -> usize { self[curr].head }
    fn tail_vid_of(&self, curr: usize) -> usize { self[curr].tail }

    fn tri_hids_of(&self, curr: usize) -> (usize, usize, usize) {
        let next = next_of(curr);
        let prev = next_of(next);
        (curr, next, prev)
    }

    fn pair_up(&mut self, hid0: usize, hid1: usize) {
        self[hid0].pair = hid1;
        self[hid1].pair = hid0;
    }

    fn update_vid_around_star(
        &mut self,
        bgn: usize, // incoming bgn halfedge id (inclusive)
        end: usize, // incoming end halfedge id (exclusive)
        vid: usize, // alternative vid
    ) {
        let mut cur = bgn;
        while cur != end {
            self[cur].head = vid; cur = self.next_hid_of(cur);
            self[cur].tail = vid; cur = self.pair_hid_of(cur);
            assert_ne!(cur, bgn);
        }
    }

    fn collapse_triangle(&mut self, hids: &(usize, usize, usize)) {
        if self[hids.1].no_pair() { return; }
        let pair1 = self.pair_hid_of(hids.1);
        let pair2 = self.pair_hid_of(hids.2);
        self[pair1].pair = pair2;
        self[pair2].pair = pair1;
        for i in [hids.0, hids.1, hids.2] { self[i] = Halfedge::default(); }
    }

    fn loop_ccw<F>(&mut self, hid: usize, mut func: F) where F: FnMut(&mut [Halfedge], usize) {
        let mut cur = hid;
        loop {
            if self[cur].no_tail() || self[cur].no_head() { continue; }
            func(self, cur);
            cur = next_of(self[cur].pair);
            if cur == hid { break; }
        }
    }
}