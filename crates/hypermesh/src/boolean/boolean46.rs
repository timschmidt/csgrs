use nalgebra::{RowVector3, Vector3};
use num_traits::real::Real;
use crate::boolean::Boolean3;
use crate::{Half, Manifold, OpType};

fn duplicate_verts(
    inclusion : &[i32],
    vert_r    : &[usize],
    vert_pos_p: &[Vector3<f64>],
    vert_pos_r: &mut [Vector3<f64>],
    vid: usize
) {
    let n = inclusion[vid].abs() as usize;
    for i in 0..n {
        vert_pos_r[vert_r[vid] + i] = vert_pos_p[vid];
    }
}


// consider moving this to a util function module
// not checked yet
fn inclusive_scan(input: &[i32], output: &mut [i32]) {
    if input.is_empty() || output.is_empty() { return; }
    let mut sum = 0;
    for (i, &v) in input.iter().enumerate() {
        sum += v;
        if i < output.len() { output[i] = sum; }
    }
}

fn size_output(
    mfd_r: &mut Manifold,
    mfd_p: &Manifold,
    mfd_q: &Manifold,
    i03: &[i32],
    i30: &[i32],
    i12: &[i32],
    i21: &[i32],
    p1q2: Vec<[i32; 2]>,
    p2q1: Vec<[i32; 2]>,
    invert_q: bool,
) -> (Vec<i32>, Vec<i32>) {
    let mut side_p = vec![0; mfd_p.hmesh.n_face];
    let mut side_q = vec![0; mfd_q.hmesh.n_face];
    let nfp = mfd_p.hmesh.n_face;
    let nfq = mfd_q.hmesh.n_face;
    let nfr = nfp + nfq;

    for h in mfd_p.hmesh.halfs.iter() { side_p[h.face().id] += i03[h.tail().id]; }
    for h in mfd_q.hmesh.halfs.iter() { side_q[h.face().id] += i30[h.tail().id]; }

    for i in 0..i12.len() {
        let hp = &mfd_p.hmesh.halfs[p1q2[i][0] as usize];
        let inclusion = i12[i].abs();
        side_p[hp.face().id] += inclusion;
        side_p[hp.twin().face().id] += inclusion;
        side_q[p1q2[i][1] as usize] += inclusion;
    }

    for i in 0..i21.len() {
        let hq = &mfd_p.hmesh.halfs[p2q1[i][1] as usize];
        let inclusion = i21[i].abs();
        side_q[hq.face().id] += inclusion;
        side_q[hq.twin().face().id] += inclusion;
        side_p[p2q1[i][0] as usize] += inclusion;
    }

    // a map from face_p and face_q to face_r
    let mut face_pq2r = vec![0; nfr + 1];
    let side_pq = [&side_p[..], &side_q[..]].concat();
    let keep_fs = side_pq.iter().map(|&x| if x > 0 { 1 } else { 0 }).collect::<Vec<i32>>();
    inclusive_scan(&keep_fs, &mut face_pq2r[1..]);
    let n_face_r = *face_pq2r.last().unwrap() as usize;
    face_pq2r.truncate(nfr);
    mfd_r.f_normal.resize(n_face_r, RowVector3::zeros());

    // fill the face normals face by face...
    let mut fid_r = 0;
    for f in mfd_p.hmesh.faces.iter() { if side_p[f.id] > 0 { mfd_r.f_normal[fid_r] = f.normal().clone(); fid_r += 1; } }
    for f in mfd_q.hmesh.faces.iter() { if side_q[f.id] > 0 { mfd_r.f_normal[fid_r] = f.normal().clone(); fid_r += 1; } }

    // count the number of halfedges todo: very suspicious...
    mfd_r.n_halfs = face_pq2r.iter().sum::<i32>() as usize;
    let mut nh_per_f = vec![];
    inclusive_scan(
        &side_pq.iter().filter(|s| **s > 0).map(|s| *s).collect::<Vec<_>>(),
        nh_per_f.as_mut_slice()
    );

    (nh_per_f, face_pq2r)
}

fn add_new_edge_verts() {

}

impl<'a> Boolean3<'a> {
    pub fn get_result(&self, op: OpType) -> () {
        let c1 = if op == OpType::Intersect {0} else {1};
        let c2 = if op == OpType::Add       {1} else {0};
        let c3 = if op == OpType::Intersect {1} else {-1};

        let i12: Vec<i32> = self.x12.iter().map(|v| c3 * v).collect();
        let i21: Vec<i32> = self.x12.iter().map(|v| c3 * v).collect();
        let w03: Vec<i32> = self.w03.iter().map(|v| c1 + c3 * v).collect();
        let w30: Vec<i32> = self.w30.iter().map(|v| c1 + c3 * v).collect();


    }
}
