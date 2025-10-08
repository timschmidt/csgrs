use nalgebra::{RowVector3 as Row3};
use crate::boolean46::TriRef;
use crate::common::{get_axis_aligned_projection, is_ccw_2d, is_ccw_3d};
use crate::Halfedge;

fn next_of(curr: usize) -> usize {
    let mut curr = curr + 1;
    if curr % 3 == 0 { curr -= 3;}
    curr
}

/// todo: reorder is required beforehand
/// halfedge indices of a triangle containing halfedge idx
fn halfs_of(
    halfs: &[Halfedge], curr: usize) -> (usize, usize, usize) {
    let next = next_of(curr);
    let prev = next_of(next);
    (curr, next, prev)
}

fn pair_up(
    hid0: usize,
    hid1: usize,
    halfs: &mut [Halfedge],
) {
    halfs[hid0].pair = hid1;
    halfs[hid1].pair = hid0;
}

fn update_vert(
    vid: usize, // vert id to replace
    bgn: usize, // bgn halfedge id
    end: usize, // end halfedge id
    halfs: &mut [Halfedge]
) {
    let mut curr = bgn;
    while curr != end {
        halfs[curr].head = vid;
        curr = next_of(curr);
        halfs[curr].tail = vid;
        curr = halfs[curr].pair;
        assert_ne!(curr, bgn);
    }
}

/// In the event that the edge collapse would create a non-manifold edge, instead
/// we duplicate the two verts and attach the manifolds the other way across this edge.
fn form_loop(
    cur: usize,
    end: usize,
    pos: &mut Vec<Row3<f64>>,
    halfs: &mut [Halfedge],
) {
    let bgn_vid = pos.len();
    let end_vid = pos.len() + 1;
    pos.push(pos[halfs[cur].tail]);
    pos.push(pos[halfs[cur].head]);

    let old_match = halfs[cur].pair;
    let new_match = halfs[end].pair;

    update_vert(bgn_vid, old_match, new_match, halfs);
    update_vert(end_vid, end, cur, halfs);

    halfs[cur].pair = new_match;
    halfs[end].pair = old_match;
    halfs[new_match].pair = cur;
    halfs[old_match].pair = end;

    remove_if_folded(halfs, pos, end);
}

fn collapse_triangle(halfs: &mut [Halfedge], hids: &(usize, usize, usize)) {
    if halfs[hids.1].no_pair() { return; }
    let pair1 = halfs[hids.1].pair;
    let pair2 = halfs[hids.2].pair;
    halfs[pair1].pair = pair2;
    halfs[pair2].pair = pair1;
    for i in [hids.0, hids.1, hids.2] { halfs[i] = Halfedge::default(); }
}

fn remove_if_folded(halfs: &mut [Halfedge], pos: &mut [Row3<f64>], hid: usize) {
    let (i0, i1, i2) = halfs_of(halfs, hid);
    let (j0, j1, j2) = halfs_of(halfs, halfs[hid].pair);
    let nan = Row3::new(f64::MAX, f64::MAX, f64::MAX);
    if halfs[i1].no_pair() { return; }

    if halfs[i1].head == halfs[j1].head {
        match (halfs[i1].pair == j2, halfs[i2].pair == j1) {
            (true, true) => for i in [i0, i1, i2] { pos[halfs[i].tail] = nan; },
            (true, false) => { pos[halfs[i1].tail] = nan; }
            (false, true) => { pos[halfs[j1].tail] = nan; }
            _ => {}
        }
    }

    pair_up(halfs[i1].pair, halfs[j2].pair, halfs);
    pair_up(halfs[i2].pair, halfs[j1].pair, halfs);
    for i in [i0, i1, i2] { halfs[i] = Halfedge::default(); }
    for j in [j0, j1, j2] { halfs[j] = Halfedge::default(); }
}

pub trait Predecessor {
    fn rec(&mut self, idx: usize) -> bool;
}

struct ShortEdge<'a> {
    pos: &'a[Row3<f64>],
    halfs: &'a[Halfedge],
    epsilon: f64,
    first_new_vert: usize,
}

struct RedundantEdge<'a> {
    halfs: &'a[Halfedge],
    half2face: &'a[TriRef],
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
    fn rec(&mut self, idx: usize) -> bool {
        let h = &self.halfs[idx];
        let i = self.first_new_vert;
        if h.no_pair() || (h.tail < i && h.head < i) { return false; }
        (self.pos[h.head] - self.pos[h.tail]).norm_squared() < self.epsilon.powi(2)
    }
}

impl <'a> Predecessor for RedundantEdge<'a> {
    fn rec(&mut self, idx: usize) -> bool {
        let h = &self.halfs[idx];
        let i = self.first_new_vert;
        if h.no_pair() || (h.tail < i && h.head < i) { return false; }
        false
    }
}

impl <'a> Predecessor for SwappableEdge<'a> {
    fn rec(&mut self, idx: usize) -> bool {
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

fn collapse_edge(
    hid: usize,
    trefs: &[TriRef],
    halfs: &mut [Halfedge],
    pos: &mut Vec<Row3<f64>>,
    fnmls: &mut [Row3<f64>],
    store: &mut Vec<usize>,
    epsilon: f64,
) -> bool {
    let tgt = &halfs[hid];
    if tgt.no_pair() { return false; }

    let head = tgt.head;
    let tail = tgt.tail;
    let pair = tgt.pair;
    let p_new = pos[head];
    let p_old = pos[tail];
    let halfs0 = halfs_of(halfs, hid);
    let halfs1 = halfs_of(halfs, pair);
    let diff = p_new - p_old;
    let flag = diff.norm_squared() < epsilon.powi(2);

    // orbit start vert
    let mut init = halfs[halfs1.1].pair;
    if !flag {
        let mut curr = init;
        let mut tref0 = &trefs[pair / 3];
        let mut p_last = pos[halfs[halfs1.1].head];
        while curr != halfs1.0 {
            curr = next_of(curr);
            let p_next = pos[halfs[curr].head];
            let tref1 = &trefs[curr / 3];
            let ccw = |p0, p1, p2| { is_ccw_3d(p0, p1, p2, &fnmls[curr / 3], epsilon) };
            if !tref1.same_face(&tref0) {
                let old_ref = tref0;
                tref0 = &trefs[hid / 3];
                if !tref1.same_face(&tref0) { return false; }
                if ccw(&p_last, &p_old, &p_new) != 0 { return false; } //todo: meshID check and so on...
            }
            if ccw(&p_next, &p_last, &p_new) < 0 { return false; }
            p_last = p_next;
            curr = halfs[curr].pair;
        }
    }

    // orbit end vert
    let mut curr = halfs[halfs0.1].pair;
    while curr != halfs1.2 {
        curr = next_of(curr);
        store.push(curr);
        curr = halfs[curr].pair;
    }

    // orbit start vert
    curr = init;
    while curr != halfs0.2 {
        curr = next_of(curr);

        let vert = halfs[curr].head;
        let next = halfs[curr].pair;
        for i in 0..store.len() {
            if vert == halfs[store[i]].head {
                form_loop(store[i], curr, pos, halfs);
                init = next;
                //store.resize();
                break;
            }
        }
        curr = next;
    }
    update_vert(tgt.head, init, halfs0.2, halfs);
    collapse_triangle(halfs, &halfs0);
    remove_if_folded(halfs, pos, init);

    true
}