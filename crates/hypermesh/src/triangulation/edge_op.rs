use std::collections::HashMap;
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


// When bgn halfedge and end halfedge are heading to the same vertex,
// and if collapsing the tail vertex as well, this function creates two loops.
// Beware the needless loop is not necessarily eliminated from the mesh because
// halfedges of the tail side might be connected to other triangles (would be folded though).
fn form_loops(
    bgn: usize,
    end: usize,
    pos: &mut Vec<Row3<f64>>,
    hs:  &mut [Halfedge],
) {
    pos.push(pos[hs.tail_vid_of(bgn)]);
    pos.push(pos[hs.head_vid_of(bgn)]);
    let bgn_vid = pos.len() - 2;
    let end_vid = pos.len() - 1;

    let bgn_pair = hs.pair_hid_of(bgn);
    let end_pair = hs.pair_hid_of(end);

    hs.update_vid_around_star(bgn_pair, end_pair, bgn_vid);
    hs.update_vid_around_star(end, bgn, end_vid);

    hs[bgn].pair = end_pair;
    hs[end_pair].pair = bgn;
    hs[end].pair = bgn_pair;
    hs[bgn_pair].pair = end;

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
    new_vid: usize,
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
    fn nv(&self) -> usize { self.new_vid }

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
    pos: &'a mut [Row3<f64>],
    halfs: &'a mut [Halfedge],
    trefs: &'a [TriRef],
    fnmls: &'a mut [Row3<f64>],
}

impl <'a> Simplifier<'a> {
    pub fn collapse_collinear_edge(&mut self, new_vid: usize, epsilon: f64) {
        let mut se = RedundantEdge {
            halfs: &self.halfs,
            trefs: &self.trefs,
            new_vid,
            epsilon,
        };

        let mut n_flag = 0;
        let mut store = vec![];

        loop {
            // todo: need to fix double mut borrow problem
            compute_flags(self.halfs.len(), &mut se, |hid: usize| {
                if collapse_edge(
                    hid,
                    &self.trefs,
                    &mut self.halfs,
                    &mut self.pos,
                    &mut self.fnmls,
                    &mut store,
                    epsilon,
                ) {
                    n_flag += 1;
                }
            });
            if n_flag == 0 { break; }
        }
    }
}

fn collapse_edge(
    hid: usize,
    trefs: &[TriRef],
    hs: &mut [Halfedge],
    pos: &mut [Row3<f64>],
    fnmls: &mut [Row3<f64>],
    store: &mut Vec<usize>, // storing the halfedge data for form_loops
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
        if let Some((i, &v)) = store.iter().enumerate().find(|&(_, &s)| hs.head_vid_of(s) == head) {
            //form_loops(v, cur, pos, hs);
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

// Split a vertex when it is pinned to two fan triangles.
fn split_pinched_vert(halfs: &mut [Halfedge], pos: &mut Vec<Row3<f64>>) {
    let mut v_processed = vec![false; pos.len()];
    let mut h_processed = vec![false; halfs.len()];

    for hid in 0..halfs.len() {
        if h_processed[hid] { continue; }
        let mut vid = halfs[hid].tail;
        if vid == usize::MAX { continue; }
        if v_processed[vid] {
            pos.push(pos[vid]);
            vid = pos.len() - 1;
        } else { v_processed[vid] = true; }

        // loop halfedges around their tail ccw way
        let mut cur = hid;
        loop {
            cur = halfs.next_hid_of(halfs[cur].pair);
            h_processed[cur] = true;
            halfs[cur].tail = vid;
            halfs[halfs[cur].pair].head = vid;
            if cur == hid { break; }
        }
    }
}

// Deduplicate the given 4-manifold edge by duplicating end_vert, thus making the
// edges distinct. Also, duplicates start_vert if it becomes pinched.
// todo: need more detailed check.
fn dedupe_edge(
    hid: usize,
    pos: &mut Vec<Row3<f64>>,
    halfs: &mut Vec<Halfedge>,
    fnmls: &mut Vec<Row3<f64>>,
    trefs: &mut Vec<TriRef>,
) {
    // 1: (imagine a Riemann surface)
    // Orbit around head vert to check that the surface crosses over at the same halfedge.
    // In that case, split the head vert and create new triangles at first.
    // It is still crossover as a surface, but it's not a 4-manifold anymore.
    // For here we do not care either the tail vert is attached or not (care it in the 3rd step).
    let tail = halfs[hid].tail;
    let head = halfs[hid].head;
    let opp = halfs[halfs.next_hid_of(hid)].pair;
    let mut cur = halfs[halfs.next_hid_of(hid)].pair;
    while cur != hid {
        if halfs.tail_vid_of(cur) == tail {
            pos.push(pos[head]);
            let copy = pos.len() - 1;
            cur = halfs[halfs.next_hid_of(cur)].pair;
            halfs.update_vid_around_star(cur, opp, copy);

            let nh = halfs.len();
            let pair = halfs[cur].pair;
            halfs.push(Halfedge { tail: head, head: copy, ..Default::default() });
            halfs.push(Halfedge { tail: copy, head: halfs.tail_vid_of(cur), ..Default::default() });
            halfs.push(Halfedge { tail: halfs.tail_vid_of(cur), head: head, ..Default::default() });
            halfs.pair_up(nh + 2, pair);
            halfs.pair_up(nh + 1, cur);

            let nh = halfs.len();
            let pair = halfs[opp].pair;
            halfs.push(Halfedge { tail: copy, head: head, ..Default::default() });
            halfs.push(Halfedge { tail: head, head: halfs.tail_vid_of(opp), ..Default::default() });
            halfs.push(Halfedge { tail: halfs.tail_vid_of(opp), head: copy, ..Default::default() });
            halfs.pair_up(nh + 2, pair);
            halfs.pair_up(nh + 1, opp);

            // Pair first new halfedge of second tri with first of first tri
            halfs.pair_up(nh, nh - 3);

            // Push per-face data if present
            trefs.push(trefs[cur / 3].clone());
            fnmls.push(fnmls[cur / 3].clone());
            trefs.push(trefs[opp / 3].clone());
            fnmls.push(fnmls[opp / 3].clone());
            break;
        }
        cur = halfs[halfs.next_hid_of(cur)].pair;
    }

    // 2: Pinch the head vert if 1 does not happen.
    if cur == hid {
        // Separate topological unit needs no new faces to be split
        let new_vert = pos.len();
        pos.push(pos[head]);
        // Duplicate per-face data if present
        fnmls.push(fnmls[head].clone());
        trefs.push(trefs[head].clone());
        // Rewire the entire star around NextHalfedge(current) to new_vert
        let start = halfs.next_hid_of(cur);
        let mut e = start;
        loop {
            halfs[e].tail = new_vert;
            let p = halfs[e].pair;
            halfs[p].head = new_vert;
            e = halfs.next_hid_of(p);
            if e == start { break; }
        }
    }
    // 3: Pinch the tail vert anyway.
    let pair = halfs[hid].pair;
    let mut current = halfs[halfs.next_hid_of(pair)].pair;
    while current != pair {
        if halfs[current].tail == head { break; }
        current = halfs[halfs.next_hid_of(current)].pair;
    }
    if current == pair {
        // Split the pinched vert the previous split created.
        let new_vert = pos.len();
        pos.push(pos[head]);
        // Duplicate per-face data if present
        fnmls.push(fnmls[head].clone());
        trefs.push(trefs[head].clone());
        let start = halfs.next_hid_of(current);
        let mut e = start;
        loop {
            halfs[e].tail = new_vert;
            let p = halfs[e].pair;
            halfs[p].head = new_vert;
            e = halfs.next_hid_of(p);
            if e == start { break; }
        }
    }
}

fn dedupe_edges(halfs: &mut [Halfedge]) {
    if halfs.is_empty() { return; }
    loop {
        let mut local = vec![false; halfs.len()];
        let mut dups = Vec::new();

        // Process halfedges grouped by the same tail (start) vertex.
        // Use Vec for up to ~32 neighbors, then switch to a HashMap to avoid quadratic behavior.
        for hid in 0..halfs.len() {
            if local[hid] || halfs[hid].no_tail() || halfs[hid].no_head() { continue; }
            let mut vec: Vec<(usize, usize)> = Vec::new();
            let mut map: HashMap<usize, usize> = HashMap::new();

            // 1: for the star around tail(i), find the minimal index for each head vertex.
            halfs.loop_ccw(hid, |hs, cur|{
                local[cur] = true; // todo: this must come before "if halfs[cur].no_tail()... { continue; }" statement!
                let head = hs[cur].head;
                if map.is_empty() {
                    if let Some(p) = vec.iter_mut().find(|p| p.0 == head) { p.1 = p.1.min(cur); }
                    else {
                        vec.push((head, cur));
                        if vec.len() > 32 { for (k, v) in vec.drain(..) { map.insert(k, v); } }
                    }
                } else { map.entry(head).and_modify(|m| { if cur < *m { *m = cur; } }).or_insert(cur); }
            });

            // 2: flag duplicates (all with the same tail/head except the minimal-index representative).
            halfs.loop_ccw(hid, |hs, cur|{
                let head = hs[cur].head;
                let mini = if map.is_empty() { vec.iter().find(|p| p.0 == head).map(|p| p.1) }
                           else { map.get(&head).copied() };
                if mini.is_some_and(|id| id != cur) { dups.push(cur); }
            });
        }

        dups.sort_unstable();
        dups.dedup();

        let mut num_flagged = 0usize;
        for &hid in &dups {
            //dedupe_edge();
            let _ = hid;
            num_flagged += 1;
        }

        if num_flagged == 0 { break; }
    }
}
