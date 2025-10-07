use nalgebra::{RowVector2 as Row2, RowVector3 as Row3};
use crate::boolean46::TriRef;
use crate::Halfedge;

fn next_of(curr: usize) -> usize {
    let mut curr = curr + 1;
    if curr % 3 == 0 { curr -= 3;}
    curr
}

/// todo: reorder is required beforehand
/// halfedge indices of a triangle containing halfedge idx
fn halfs_of(halfs: &[Halfedge], curr: usize) -> (usize, usize, usize) {
    let next = next_of(curr);
    let prev = next_of(next);
    (curr, next, prev)
}

fn pair_up(halfs: &mut [Halfedge], hid0: usize, hid1: usize) {
    halfs[hid0].pair = hid1 as i32;
    halfs[hid1].pair = hid0 as i32;
}

fn collapse_triangle(halfs: &mut [Halfedge], hids: &(usize, usize, usize)) {
    if halfs[hids.1].no_pair() { return; }
    let pair1 = halfs[hids.1].pair;
    let pair2 = halfs[hids.2].pair;
    halfs[pair1 as usize].pair = pair2;
    halfs[pair2 as usize].pair = pair1;
    for i in [hids.0, hids.1, hids.2] { halfs[i] = Halfedge::default(); }
}

fn remove_if_folded(halfs: &mut [Halfedge], pos: &mut [Row3<f64>], hid: usize) {
    let (i0, i1, i2) = halfs_of(halfs, hid);
    let (j0, j1, j2) = halfs_of(halfs, halfs[hid].pair as usize);
    let nan = Row3::new(f64::MAX, f64::MAX, f64::MAX);
    if halfs[i1].pair == -1 { return; }

    if halfs[i1].head == halfs[j1].head {
        match (halfs[i1].pair == j2 as i32, halfs[i2].pair == j1 as i32) {
            (true, true) => for i in [i0, i1, i2] { pos[halfs[i].tail as usize] = nan; },
            (true, false) => { pos[halfs[i1].tail as usize] = nan; }
            (false, true) => { pos[halfs[j1].tail as usize] = nan; }
            _ => {}
        }
    }

    pair_up(halfs, halfs[i1].pair as usize, halfs[j2].pair as usize);
    pair_up(halfs, halfs[i2].pair as usize, halfs[j1].pair as usize);
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
        let i = self.first_new_vert as i32;
        if h.pair < 0 || (h.tail < i && h.head < i) { return false; }

        let d = self.pos[h.head as usize] - self.pos[h.tail as usize];
        d.norm_squared() < self.epsilon * self.epsilon
    }
}

impl <'a> Predecessor for RedundantEdge<'a> {
    fn rec(&mut self, idx: usize) -> bool {
        let h = &self.halfs[idx];
        let i = self.first_new_vert as i32;
        if h.pair < 0 || (h.tail < i && h.head < i) { return false; }
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

fn collapse_edge() {

}