mod intersect;
mod shadow;
mod kernel02;
mod kernel11;
mod kernel12;
use nalgebra::Vector3;
use crate::boolean::kernel02::Kernel02;
use crate::boolean::kernel11::Kernel11;
use crate::boolean::kernel12::Kernel12;
use crate::bounds::BoundingBox;
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

struct Intersection12Recorder<'a> {
    pub k12: &'a Kernel12<'a>,
    pub forward: bool,
    pub p1q2: Vec<[i64; 2]>,
    pub x12: Vec<i64>,
    pub v12: Vec<Vector3<f64>>,
}


fn intersect12 (
    p: &Manifold,
    q: &Manifold,
    p1q2: &mut Vec<[i64; 2]>,
    expand_p: f64,
    forward: bool
) -> (Vec<i64>, Vec<Vector3<f64>>) {
    let a = if forward { p } else { q };
    let b = if forward { q } else { p };

    let k02 = Kernel02{
        verts_p: &a.hmesh.verts,
        verts_q: &b.hmesh.verts,
        halfs_q: &b.hmesh.halfs,
        expand_p,
        forward
    };
    let k11 = Kernel11{
        verts_p: &p.hmesh.verts,
        halfs_p: &p.hmesh.halfs,
        verts_q: &q.hmesh.verts,
        halfs_q: &q.hmesh.halfs,
        expand_p,
    };

    let k12 = Kernel12{
        halfs_p: &a.hmesh.halfs,
        halfs_q: &b.hmesh.halfs,
        verts_p: &a.hmesh.verts,
        forward,
        k02,
        k11,
    };

    let bbs = a.hmesh.edges.iter().map(|e| BoundingBox::new(
        e.vert0().pos().transpose(),
        e.vert1().pos().transpose()
    ));

    let rec = Intersection12Recorder{
        k12: &k12,
        p1q2: Vec::new(),
        x12: Vec::new(),
        v12: Vec::new(),
        forward,
};

    // todo: record collisions
    // b.collider_.Collisions<false, Box, Kernel12Recorder>(AEdgeBB.cview(), recorder);

    let mut x12: Vec<i64> = vec![];
    let mut v12: Vec<Vector3<f64>> = vec![];
    let mut seq: Vec<usize> = (0..p1q2.len()).collect();
    seq.sort_by(|&a, &b| (rec.p1q2[a][0], rec.p1q2[a][1]).cmp(&(rec.p1q2[b][0], rec.p1q2[b][1])));

    for i in 0..seq.len() {
        p1q2.push(rec.p1q2[seq[i]]);
        x12.push(rec.x12[seq[i]]);
        v12.push(rec.v12[seq[i]]);
    }

    (x12, v12)
}


fn winding03(p: &Manifold, q: &Manifold, expand_p: f64, forward: bool) -> Vec<i64> {
    let a = if forward {p} else {q};
    let b = if forward {q} else {p};

    let w03 = vec![];
    let k02 = Kernel02 {
        verts_p: &a.hmesh.verts,
        verts_q: &b.hmesh.verts,
        halfs_q: &b.hmesh.halfs,
        expand_p,
        forward,
    };

    // todo: record collisions
    // auto recorder = MakeSimpleRecorder(f);
    // b.collider_.Collisions<false>(a.vertPos_.cview(), recorder);

    w03
}

struct Boolean3<'a> {
    mfd_p: &'a Manifold,
    mfd_q: &'a Manifold,
    expand_p: f64,
    p1q2: Vec<[i32; 2]>, 
    p2q1: Vec<[i32; 2]>, 
    x12: Vec<i32>,
    x21: Vec<i32>,
    w03: Vec<i32>,
    w30: Vec<i32>,
    v12: Vec<Vector3<f64>>,
    v21: Vec<Vector3<f64>>,
    valid: bool,
}

/*
impl<'a> Boolean3<'a> {
    fn new(&self, p: &'a Manifold, q: &'a Manifold, op :OpType) -> Self {
        self.mfd_p = p;
        self.mfd_q = q;
        self.expand_p = if op == OpType::Add {1.} else {0.};
        self.valid = true;

        // todo assert mfd_p has bounds

        // Level 3
        // Build up the intersection of the edges and triangles, keeping only those
        // that intersect and record the direction the edge is passing through the triangle.
        (self.x12, self.v12) = intersect12();
        (self.x21, self.v21) = intersect12();

        // Sum up the winding numbers of all vertices.
        self.w03 = Winding03(inP, inQ, expandP_, true);
        self.w30 = Winding03(inP, inQ, expandP_, false);
    }

    pub fn result() {

    }
}
*/

pub enum OpType { Add, Subtract, Intersect }

