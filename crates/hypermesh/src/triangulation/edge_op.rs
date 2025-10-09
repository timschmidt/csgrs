use nalgebra::{RowVector3 as Row3};
use crate::boolean46::TriRef;
use crate::common::{get_axis_aligned_projection, is_ccw_2d, is_ccw_3d};
use crate::Halfedge;

fn next_of(curr: usize) -> usize {
    let mut curr = curr + 1;
    if curr % 3 == 0 { curr -= 3;}
    curr
}

trait HalfedgeOps {
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
        bgn: usize, // bgn halfedge id (incoming)
        end: usize, // end halfedge id (incoming)
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
}

// In the event that the edge collapse would create a non-manifold edge, instead
// we duplicate the two verts and attach the manifolds the other way across this edge.
fn form_loop(
    cur: usize, // halfedge id
    end: usize,
    pos: &mut Vec<Row3<f64>>,
    hs: &mut [Halfedge],
) {
    pos.push(pos[hs.tail_vid_of(cur)]);
    pos.push(pos[hs.head_vid_of(cur)]);
    let bgn_vid = pos.len() - 2;
    let end_vid = pos.len() - 1;

    let old_match = hs.pair_hid_of(cur);
    let new_match = hs.pair_hid_of(end);

    hs.update_vid_around_star(old_match, new_match, bgn_vid);
    hs.update_vid_around_star(end, cur, end_vid);

    hs[cur].pair = new_match;
    hs[new_match].pair = cur;

    hs[end].pair = old_match;
    hs[old_match].pair = end;

    remove_if_folded(hs, pos, end);
}

// Removing fold paired triangles from the mesh. The process is either of:
// 1. Non-2-manifold, two triangles are completely isolated from the mesh
// 2. Non-2-manifold, one vertex is isolated from the mesh
// 3. Topologically valid, but just two vertex positions are the same
// Beware the case that triangles only connected at a vertex but not by halfedge
// are eliminated by split_pinched_vert and dedupe_edges functions.
fn remove_if_folded(hs: &mut [Halfedge], pos: &mut [Row3<f64>], hid: usize) {
    let (i0, i1, i2) = hs.tri_hids_of(hid);
    let (j0, j1, j2) = hs.tri_hids_of(hs.pair_hid_of(hid));

    if hs[i1].no_pair() || hs.head_vid_of(i1) != hs.head_vid_of(j1) { return; }

    let nan = Row3::new(f64::MAX, f64::MAX, f64::MAX);
    match (hs.pair_hid_of(i1) == j2, hs.pair_hid_of(i2) == j1) {
        (true, true)  => for i in [i0, i1, i2] { pos[hs.tail_vid_of(i)] = nan; },
        (true, false) => { pos[hs.tail_vid_of(i1)] = nan; }
        (false, true) => { pos[hs.tail_vid_of(j1)] = nan; }
        _ => {} // topo valid
    }
    hs.pair_up(hs[i1].pair, hs[j2].pair);
    hs.pair_up(hs[i2].pair, hs[j1].pair);
    for i in [i0, i1, i2] { hs[i] = Halfedge::default(); }
    for j in [j0, j1, j2] { hs[j] = Halfedge::default(); }
}

pub trait Predecessor {
    fn hs(&self) -> &[Halfedge];
    fn nv(&self) -> usize;
    fn rec(&mut self, hid: usize) -> bool;
    fn invalid(& self, hid: usize) -> bool {
        let h = &self.hs()[hid];
        h.no_pair() || (h.tail < self.nv() && h.head < self.nv())
    }
}

struct ShortEdge<'a> {
    pos: &'a[Row3<f64>],
    halfs: &'a[Halfedge],
    epsilon: f64,
    first_new_vert: usize,
}

struct RedundantEdge<'a> {
    halfs: &'a[Halfedge],
    trefs: &'a[TriRef],
    epsilon: f64,
    first_new_vert: usize,
}

struct SwappableEdge<'a> {
    pos: &'a[Row3<f64>],
    fnmls: &'a[Row3<f64>],
    halfs: &'a[Halfedge],
    tolerance: f64,
    first_new_vert: usize,
}

impl <'a> Predecessor for ShortEdge<'a> {
    fn hs(&self) -> &[Halfedge] { self.halfs }
    fn nv(&self) -> usize { self.first_new_vert }

    fn rec(&mut self, hid: usize) -> bool {
        if self.invalid(hid) { return false; }
        let h = &self.halfs[hid];
        let d = self.pos[h.head] - self.pos[h.tail];
        d.norm_squared() < self.epsilon.powi(2)
    }
}

impl <'a> Predecessor for RedundantEdge<'a> {
    fn hs(&self) -> &[Halfedge] { self.halfs }
    fn nv(&self) -> usize { self.first_new_vert }

    // Check around a halfedges from the same tail vertex.
    // If they consist of only two tris, then their edge is collapsable.
    fn rec(&mut self, hid: usize) -> bool {
        if self.invalid(hid) { return false; }
        let cw_next = |i: usize| next_of(self.halfs[i].pair);
        let     bgn = hid;
        let mut cur = cw_next(bgn);
        let     tr0 = &self.trefs[bgn / 3];
        let mut tr1 = &self.trefs[cur / 3];
        let mut same = tr0.same_face(tr1);
        while cur != bgn {
            cur = cw_next(cur);
            let tr2 = &self.trefs[cur / 3];
            if !tr2.same_face(tr0) &&
               !tr2.same_face(tr1) {
                if same { tr1 = tr2; same = false; }
                else { return false; }
            }
        }
        true
    }
}

impl <'a> Predecessor for SwappableEdge<'a> {
    fn hs(&self) -> &[Halfedge] { self.halfs }
    fn nv(&self) -> usize { self.first_new_vert }

    fn rec(&mut self, hid: usize) -> bool {
        if self.invalid(hid) { return false; }
        false
    }
}

fn compute_flags<F>(n: usize, pred: &mut dyn Predecessor, mut func: F)->() where F: FnMut(usize) {
    let mut store = vec![];
    for i in 0..n { if pred.rec(i) { store.push(i); } }
    for i in store { func(i); }
}

struct Simplifier<'a> {
    pos: &'a [Row3<f64>],
    halfs: &'a [Halfedge],
    fnmls: &'a [Row3<f64>],
}

impl <'a> Simplifier<'a> {
    pub fn collapse_collinear_edge(&self, epsilon: f64) {
        //let se = RedundantEdge {halfs: self.halfs, epsilon: epsilon, trefs: &[], first_new_vert: 0};
        //compute_flags();
    }
}

fn collapse_edge(
    hid: usize,
    trefs: &[TriRef],
    hs: &mut [Halfedge],
    pos: &mut Vec<Row3<f64>>,
    fnmls: &mut [Row3<f64>],
    store: &mut Vec<usize>,
    epsilon: f64,
) -> bool {
    if hs[hid].no_pair() { return false; }
    let vid_keep = hs.head_vid_of(hid);
    let pos_keep = pos[vid_keep];
    let pos_delt = pos[hs.tail_vid_of(hid)];
    let pair = hs.pair_hid_of(hid);
    let n_pair = fnmls[pair / 3];
    let tri0 = hs.tri_hids_of(hid);
    let tri1 = hs.tri_hids_of(pair);

    let mut bgn = hs.pair_hid_of(tri1.1); // the bgn half heading delt vert
    let     end = tri0.2;                 // the end half heading delt vert

    // check validity by orbiting start vert ccw order
    if (pos_keep - pos_delt).norm_squared() >= epsilon.powi(2) {
        let mut cur = bgn;
        let mut tr0 = &trefs[pair / 3];
        let mut p_prev = pos[hs.head_vid_of(tri1.1)];
        while cur != pair {
            cur = hs.next_hid_of(cur); // incoming half around delt vert
            let p_next = pos[hs.head_vid_of(cur)];
            let r_curr = &trefs[cur / 3];
            let n_curr = &fnmls[cur / 3];
            let ccw = |p0, p1, p2| { is_ccw_3d(p0, p1, p2, n_curr, epsilon) };
            if !r_curr.same_face(&tr0) {
                let tr2 = tr0;
                tr0 = &trefs[hid / 3];
                if !r_curr.same_face(&tr0) { return false; }
                if tr0.mesh_id != tr2.mesh_id || tr0.face_id != tr2.face_id || n_pair.dot(n_curr) < -0.5 {
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
    let mut curr = hs.pair_hid_of(tri0.1);
    while curr != tri1.2 {
        curr = hs.next_hid_of(curr);
        store.push(curr); // storing outgoing half here
        curr = hs.pair_hid_of(curr);
    }

    let mut cur = bgn;
    while cur != end {
        cur      = hs.next_hid_of(cur);
        let pair = hs.pair_hid_of(cur);
        let head = hs.head_vid_of(cur);
        if let Some((i, &v)) = store.iter().enumerate()
            .find(|&(_, &s)| hs.head_vid_of(s) == head)
        {
            form_loop(v, cur, pos, hs);
            bgn = pair;
            store.truncate(i);
        }
        cur = pair;
    }

    // do collapse
    hs.update_vid_around_star(bgn, end, vid_keep);
    hs.collapse_triangle(&tri0);
    remove_if_folded(hs, pos, bgn);

    true
}

fn swap_edge() {}

fn split_pinched_vert() {}

fn dedupe_edges() {}