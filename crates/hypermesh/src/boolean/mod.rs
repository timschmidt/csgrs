pub mod intersect;
pub mod shadow;
pub mod kernel02;
pub mod kernel11;
pub mod kernel12;
pub mod boolean46;
pub mod test_data;

use nalgebra::{RowVector3, Vector3};
use crate::boolean::kernel02::Kernel02;
use crate::boolean::kernel11::Kernel11;
use crate::boolean::kernel12::Kernel12;
use crate::bounds::BoundingBox;
use crate::collider::{Collider, Recorder};
use crate::manifold::Manifold;

/**
 * The notation in these files is abbreviated due to the complexity of the
 * functions involved. The key is that the input manifolds are P and Q, while
 * the output is R, and these letters in both upper and lower case refer to
 * these objects. Operations are based on dimensionality: vert: 0, edge: 1,
 * face: 2, solid: 3. X denotes a winding-number type quantity from the source
 * paper of this algorithm, while S is closely related but includes only the
 * subset of X values which "shadow" (are on the correct side of).
 *
 * Nearly everything here is sparse arrays, where, for instance, each pair in
 * p2q1 refers to a face index of P interacting with a halfedge index of Q.
 * Adjacent arrays like x21 refer to the values of X corresponding to each
 * sparse index pair.
 *
 * Note many functions are designed to work symmetrically, for instance, for both
 * p2q1 and p1q2. Inside these functions P and Q are marked as though the
 * function is forwards, but it may include a Boolean "reverse" that indicates P
 * and Q have been swapped.
 */

// 1st check point
pub fn simple_boolean() {}

struct SimpleRecorder<F> where F: FnMut(usize, usize) {
    callback: F,
}

impl<F> SimpleRecorder<F> where F: FnMut(usize, usize) {
    fn new(callback: F) -> Self {
        Self { callback }
    }
}

impl<'a, F> Recorder for SimpleRecorder<F> where F: FnMut(usize, usize) {
    fn record(&mut self, query_idx: usize, leaf_idx: usize) {
        (self.callback)(query_idx, leaf_idx);
    }
}



struct Intersection12Recorder<'a> {
    pub mfd_a: &'a Manifold,
    pub mfd_b: &'a Manifold,
    pub k12: &'a Kernel12<'a>,
    pub forward: bool,
    pub p1q2: Vec<[i32; 2]>,
    pub x12: Vec<i32>,
    pub v12: Vec<RowVector3<f64>>,
}

impl <'a> Recorder for Intersection12Recorder<'a> {
    fn record(&mut self, query_idx: usize, leaf_idx: usize) {
        let h = &self.mfd_a.hmesh.halfs[query_idx];
        let (x12, op_v12) = self.k12.op(h.id, leaf_idx);
        if let Some(v12) = op_v12 {
            println!("hid: {}, lid: {}, x12: {}, v12: {:?}", h.id, leaf_idx, x12, v12);
            if self.forward { self.p1q2.push([query_idx as i32, leaf_idx as i32]); }
            else            { self.p1q2.push([leaf_idx as i32, query_idx as i32]); }
            self.x12.push(x12);
            self.v12.push(v12);
        }
    }
}


pub fn intersect12 (
    p: &Manifold,
    q: &Manifold,
    p1q2: &mut Vec<[i32; 2]>,
    expand: f64,
    forward: bool
) -> (Vec<i32>, Vec<RowVector3<f64>>) {
    let a = if forward { p } else { q };
    let b = if forward { q } else { p };

    let k02 = Kernel02{
        vpos_p: &a.hmesh.verts.iter().map(|v| v.pos()).collect::<Vec<_>>(),
        vpos_q: &b.hmesh.verts.iter().map(|v| v.pos()).collect::<Vec<_>>(),
        half_q: &b.hmesh.halfs,
        normal: &p.hmesh.verts.iter().map(|v| v.normal()).collect::<Vec<_>>(),
        expand,
        forward
    };
    let k11 = Kernel11{
        vpos_p: &p.hmesh.verts.iter().map(|v| v.pos()).collect::<Vec<_>>(),
        half_p: &p.hmesh.halfs,
        vpos_q: &q.hmesh.verts.iter().map(|v| v.pos()).collect::<Vec<_>>(),
        half_q: &q.hmesh.halfs,
        normal: &p.hmesh.verts.iter().map(|v| v.normal()).collect::<Vec<_>>(),
        expand,
    };

    let k12 = Kernel12{
        verts_p: &a.hmesh.verts,
        halfs_p: &a.hmesh.halfs,
        halfs_q: &b.hmesh.halfs,
        forward,
        k02,
        k11,
    };

    let bboxes = a.hmesh.halfs.iter()
        .filter(|h| h.tail().id < h.head().id)
        .map(|h| BoundingBox::new(h.id, &vec![h.tail().pos(), h.head().pos()])).collect::<Vec<_>>();

    let mut rec = Intersection12Recorder{
        mfd_a: a,
        mfd_b: b,
        k12: &k12,
        p1q2: Vec::new(),
        x12: Vec::new(),
        v12: Vec::new(),
        forward,
    };

     b.collider.collision(&bboxes, &mut rec);

    let mut x12: Vec<i32> = vec![];
    let mut v12: Vec<RowVector3<f64>> = vec![];
    let mut seq: Vec<usize> = (0..rec.p1q2.len()).collect();
    seq.sort_by(|&a, &b| (rec.p1q2[a][0], rec.p1q2[a][1]).cmp(&(rec.p1q2[b][0], rec.p1q2[b][1])));

    for i in 0..seq.len() {
        p1q2.push(rec.p1q2[seq[i]]);
        x12.push(rec.x12[seq[i]]);
        v12.push(rec.v12[seq[i]]);
    }

    (x12, v12)
}


pub fn winding03(p: &Manifold, q: &Manifold, expand: f64, forward: bool) -> Vec<i32> {
    let a = if forward {p} else {q};
    let b = if forward {q} else {p};

    let mut w03 = vec![0; a.hmesh.verts.len()];
    let k02 = Kernel02 {
        vpos_p: &a.hmesh.verts.iter().map(|v| v.pos()).collect::<Vec<_>>(),
        vpos_q: &b.hmesh.verts.iter().map(|v| v.pos()).collect::<Vec<_>>(),
        half_q: &b.hmesh.halfs,
        normal: &p.hmesh.verts.iter().map(|v| v.normal()).collect::<Vec<_>>(),
        expand,
        forward,
    };

    let bboxes = a.hmesh.verts.iter().map(|v| BoundingBox::new(v.id, &vec![v.pos(), v.pos()])).collect::<Vec<_>>();
    let mut rec = SimpleRecorder::new(
        |a, b| {
            let (s02, z02) = k02.op(a, b);
            if z02.is_some() { w03[a] += s02 * if forward {1} else {-1}; }
        }
    );
    b.collider.collision(&bboxes, &mut rec);

    w03
}

pub struct Boolean3<'a> {
    pub mfd_p: &'a Manifold,
    pub mfd_q: &'a Manifold,
    pub p1q2: Vec<[i32; 2]>,
    pub p2q1: Vec<[i32; 2]>,
    pub x12: Vec<i32>,
    pub x21: Vec<i32>,
    pub w03: Vec<i32>,
    pub w30: Vec<i32>,
    pub v12: Vec<RowVector3<f64>>,
    pub v21: Vec<RowVector3<f64>>,
}

impl<'a> Boolean3<'a> {
    /*
fn new(&self, p: &'a Manifold, q: &'a Manifold, op :OpType) -> Self {
            self.mfd_p = p;
            self.mfd_q = q;
            self.expand_p = if op == OpType::Add {1.} else {0.};
            self.valid = true;

            // todo assert mfd_p has bounds

            // Level 3
            // Build up the intersection of the edges and triangles, keeping only those
            // that intersect and  the direction the edge is passing through the triangle.
            (self.x12, self.v12) = intersect12();
            (self.x21, self.v21) = intersect12();

            // Sum up the winding numbers of all vertices.
            self.w03 = Winding03(inP, inQ, expandP_, true);
            self.w30 = Winding03(inP, inQ, expandP_, false);
            }
    */
}

#[derive(PartialEq)]
pub enum OpType { Add, Subtract, Intersect }

#[cfg(test)]
mod tests;
mod triangulate;

