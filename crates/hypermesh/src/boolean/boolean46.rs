use nalgebra::{RowVector3, Vector3};
use std::collections::HashMap;
use std::mem;
use crate::boolean::Boolean3;
use crate::{Half, Halfedge, Manifold, MfdBuffer, OpType};
use crate::bounds::BoundingBox;

fn duplicate_verts(
    inclusion : &[i32],
    vert_r    : &[i32],
    vert_pos_p: &[Vector3<f64>],
    vert_pos_r: &mut [Vector3<f64>],
    vid: usize
) {
    let n = inclusion[vid].abs() as usize;
    for i in 0..n {
        vert_pos_r[vert_r[vid] as usize + i] = vert_pos_p[vid];
    }
}

// consider moving this to a util function module
// not checked yet
fn inclusive_scan(input: &[i32], output: &mut [i32], offset: i32) {
    if input.is_empty() || output.is_empty() { return; }
    let mut sum = offset;
    for (i, &v) in input.iter().enumerate() {
        sum += v;
        if i < output.len() { output[i] = sum; }
    }
}

// consider moving this to a util function module
// not checked yet
fn exclusive_scan(input: &[i32], output: &mut [i32], offset: i32) {
    if input.is_empty() || output.is_empty() { return; }
    let mut sum = offset;
    output[0] = 0;
    for i in 1..input.len() {
        sum += input[i - 1];
        if i < output.len() { output[i] = sum; }
    }
}

fn size_output(
    mfd_r: &mut MfdBuffer,
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
        let hq = &mfd_q.hmesh.halfs[p2q1[i][1] as usize];
        let inclusion = i21[i].abs();
        side_q[hq.face().id] += inclusion;
        side_q[hq.twin().face().id] += inclusion;
        side_p[p2q1[i][0] as usize] += inclusion;
    }

    // a map from face_p and face_q to face_r
    let mut face_pq2r = vec![0; nfr + 1];
    let side_pq = [&side_p[..], &side_q[..]].concat();
    let keep_fs = side_pq.iter().map(|&x| if x > 0 { 1 } else { 0 }).collect::<Vec<i32>>();
    inclusive_scan(&keep_fs, &mut face_pq2r[1..], 0);
    let n_face_r = *face_pq2r.last().unwrap() as usize;
    face_pq2r.truncate(nfr);
    mfd_r.fnmls.resize(n_face_r, RowVector3::zeros());

    // fill the face normals face by face...
    let mut fid_r = 0;
    for f in mfd_p.hmesh.faces.iter() { if side_p[f.id] > 0 { mfd_r.fnmls[fid_r] = f.normal().clone(); fid_r += 1; } }
    for f in mfd_q.hmesh.faces.iter() { if side_q[f.id] > 0 { mfd_r.fnmls[fid_r] = f.normal().clone(); fid_r += 1; } }

    // starting half idx per face todo: very suspicious...
    mfd_r.halfs = vec![Halfedge::default(); face_pq2r.iter().sum::<i32>() as usize];
    let mut ih_per_f = vec![];
    inclusive_scan(
        &side_pq.iter().filter(|s| **s > 0).map(|s| *s).collect::<Vec<_>>(),
        ih_per_f.as_mut_slice(), 0
    );

    (ih_per_f, face_pq2r)
}

#[derive(Clone)]
struct EdgePos {
    val: f64, // dot value of edge
    vid: usize,
    cid: usize, // collision_id
    is_tail: bool
}

#[derive(Clone)]
struct TriRef {
    mesh_id: usize,
    face_id: usize,
}

fn add_new_edge_verts(
    p1q2: Vec<[i32; 2]>,
    i12: &[i32],
    v12_r: &[i32],
    halfs_p: &[Half],
    forward: bool,
    offset: usize,
    edges_pos: &mut HashMap<usize, Vec<EdgePos>>,
    edges_new: &mut HashMap<(usize, usize), Vec<EdgePos>>,
) {
    for i in 0..p1q2.len() {
        let hid_p = p1q2[i][if forward {0} else {1}] as usize;
        let fid_q = p1q2[i][if forward {1} else {0}] as usize;
        let vid_r = v12_r[i] as usize;
        let inclusion = i12[i];

        let h0 = &halfs_p[hid_p];
        let h1 = h0.twin();
        let key_l = if forward { (h0.face().id, fid_q) } else { (fid_q, h0.face().id) };
        let key_r = if forward { (h1.face().id, fid_q) } else { (fid_q, h1.face().id) };
        let mut direction = inclusion < 0;
        edges_pos.insert(hid_p, vec![]);
        edges_new.insert(key_l, vec![]);
        edges_new.insert(key_r, vec![]);

        // need to check what is going on here...
        for j in 0..inclusion.abs() as usize {
            edges_pos.get_mut(&hid_p).unwrap().push(EdgePos{ val: 0., vid: i + offset, cid: 0, is_tail: direction });
            direction = !direction;
            edges_new.get_mut(&key_l).unwrap().push(EdgePos{ val: 0., vid: vid_r + j, cid: i + offset, is_tail: direction ^ forward });
            direction = !direction;
            edges_new.get_mut(&key_r).unwrap().push(EdgePos{ val: 0., vid: vid_r + j, cid: i + offset, is_tail: direction ^ !forward });
        }
    }
}

fn pair_up(
    edge_pos: &Vec<EdgePos>
) -> Vec<Halfedge> {
    assert_eq!(edge_pos.len() % 2, 0);
    let ne = edge_pos.len() / 2;

    //auto middle = std::partition(edgePos.begin(), edgePos.end(), [](EdgePos x) { return x.isStart; });
    //auto cmp = [](EdgePos a, EdgePos b) {
    //    return a.edgePos < b.edgePos ||
    //        // we also sort by collisionId to make things deterministic
    //        (a.edgePos == b.edgePos && a.collisionId < b.collisionId);
    //};
    //std::stable_sort(edgePos.begin(), middle, cmp);
    //std::stable_sort(middle, edgePos.end(), cmp);
    //std::vector<Halfedge> edges;
    //for (size_t i = 0; i < nEdges; ++i)
    //edges.push_back({edgePos[i].vert, edgePos[i + nEdges].vert, -1});
    //return edges;

    panic!()
}

fn append_partial_edges(
    mfd_r: &mut MfdBuffer,
    face_ptr_r: &mut [i32],
    half_ref: &mut [TriRef],
    whole_he_p: &mut [bool],
    mfd_p_half: &[Half],
    i03: &[i32],
    vp2r: &[usize],
    edges_pos: &mut HashMap<usize, Vec<EdgePos>>,
    forward: bool,
) {
    for e in edges_pos {
        let hid_p = e.0.clone();
        let pos_p = e.1;
        let h = &mfd_p_half[hid_p];
        whole_he_p[h.id] = false;
        whole_he_p[h.twin().id] = false;
        let dif: RowVector3<f64> = h.head().pos() - h.tail().pos();

        for p in pos_p.iter_mut() {
            p.val = dif.dot(&mfd_r.pos[p.vid]);
        }

        let i_tail = i03[h.tail().id];
        let i_head = i03[h.head().id];
        let p_tail = mfd_r.pos[vp2r[h.tail().id]];
        let p_head = mfd_r.pos[vp2r[h.head().id]];

        for i in 0..i_tail.abs() as usize {
            pos_p.push(EdgePos{
                val: p_tail.dot(&dif),
                vid: vp2r[h.tail().id] + i, // need to check
                cid: usize::MAX,
                is_tail: i_tail > 0
            });
        }

        for i in 0..i_head.abs() as usize {
            pos_p.push(EdgePos{
                val: p_head.dot(&dif),
                vid: vp2r[h.head().id] + i, // need to check
                cid: usize::MAX,
                is_tail: i_tail < 0
            });
        }

        let mut edges = pair_up(&pos_p);
        let face_l = h.face();
        let face_r = h.twin().face();

        // Negative inclusion means the halfedges are reversed, which means our
        // reference is now to the endVert instead of the startVert, which is one
        // position advanced CCW. This is only valid if this is a retained vert; it
        // will be ignored later if the vert is new.

        let fw_ref = TriRef{ mesh_id: if forward {0} else {1}, face_id: face_l.id};
        let bk_ref = TriRef{ mesh_id: if forward {0} else {1}, face_id: face_r.id};

        for h in edges.iter_mut() {
            face_ptr_r[face_l.id] += 1;
            face_ptr_r[face_r.id] += 1;
            let fw_edge = face_ptr_r[face_l.id];
            let bk_edge = face_ptr_r[face_l.id];

            h.pair = bk_edge;
            mfd_r.halfs[fw_edge as usize] = h.clone();
            half_ref[fw_edge as usize] = fw_ref.clone();

            //std::swap(e.startVert, e.endVert);
            mem::swap(&mut h.tail, &mut h.head);
            h.pair = fw_edge;
            mfd_r.halfs[bk_edge as usize] = h.clone();
            half_ref[bk_edge as usize] = bk_ref.clone();
        }
    }
}

fn append_new_edges(
    mfd_r: &mut MfdBuffer,
    face_ptr_r: &mut[i32],
    edges_new: &mut HashMap<(usize, usize), Vec<EdgePos>>,
    half_ref: &mut [TriRef],
    face_pq2r: &[i32],
    nfaces_p: usize,
) {
    for val in edges_new.into_iter() { // values??
        let (face_p, face_q) = val.0;
        let epos = val.1;
        let mut bbox = BoundingBox::new(vec![]);
        for ep in epos.iter() {
            bbox.union(mfd_r.pos[ep.vid]);
        }

        let d = bbox.longest_dim();
        for ep in epos.iter_mut() {
            ep.val = mfd_r.pos[ep.vid][d];
        }

        let mut edges = pair_up(&epos);
        let face_l = face_pq2r[*face_p] as usize;
        let face_r = face_pq2r[*face_q + nfaces_p] as usize;

        // Negative inclusion means the halfedges are reversed, which means our
        // reference is now to the endVert instead of the startVert, which is one
        // position advanced CCW. This is only valid if this is a retained vert; it
        // will be ignored later if the vert is new.

        let fw_ref = TriRef{ mesh_id: 0, face_id: *face_p};
        let bk_ref = TriRef{ mesh_id: 1, face_id: *face_q};

        for h in edges.iter_mut() {
            let fw_edge = { let ptr = face_ptr_r[face_l]; face_ptr_r[face_l] += 1; ptr };
            let bk_edge = { let ptr = face_ptr_r[face_r]; face_ptr_r[face_r] += 1; ptr };

            h.pair = bk_edge;
            mfd_r.halfs[fw_edge as usize] = h.clone();
            half_ref[fw_edge as usize] = fw_ref.clone();

            mem::swap(&mut h.tail, &mut h.head);
            h.pair = fw_edge;
            mfd_r.halfs[bk_edge as usize] = h.clone();
            half_ref[bk_edge as usize] = bk_ref.clone();
        }
    }
}

fn append_whole_edges(
    mfd_r: &mut MfdBuffer,
    mfd_p_half: &[Halfedge],
    face_ptr_r: &mut[i32],
    whole_he_p: &[bool],
    half_ref: &mut [TriRef],
    i03: &[i32],
    fid_p2r: &[usize],
    vid_p2r: &[usize],
    forward: bool,
) {
    for i in 0..mfd_r.halfs.len() {
        // duplicate halfedge
        if !whole_he_p[i] { continue; }
        let mut h = mfd_p_half[i].clone();
        if !h.is_forward() { continue; }

        let inclusion = i03[h.tail as usize];
        if inclusion == 0 { continue; }
        if inclusion < 0 { mem::swap(&mut h.tail, &mut h.head); }

        h.tail = vid_p2r[h.tail as usize] as i32;
        h.head = vid_p2r[h.head as usize] as i32;

        let fid_l = i / 3;
        let fid_r = (h.pair / 3) as usize;
        let face_l = fid_p2r[fid_l];
        let face_r = fid_p2r[fid_r];
        let fw_ref = TriRef{ mesh_id: if forward {0} else {1}, face_id: fid_l};
        let bk_ref = TriRef{ mesh_id: if forward {0} else {1}, face_id: fid_r};

        for _ in 0..inclusion.abs() as usize {
            let fw_edge = face_ptr_r[face_l] + 1;
            let bk_edge = face_ptr_r[face_r] + 1;
            h.pair = bk_edge;
            mfd_r.halfs[fw_edge as usize] = h.clone();
            mfd_r.halfs[bk_edge as usize] = Halfedge{tail: h.head, head: h.tail, pair: fw_edge};
            half_ref[fw_edge as usize] = fw_ref.clone();
            half_ref[bk_edge as usize] = bk_ref.clone();
            h.tail += 1;
            h.head += 1;
        }
    }
}

impl<'a> Boolean3<'a> {
    pub fn get_result(&self, op: OpType) -> () {
        let c1 = if op == OpType::Intersect {0} else {1};
        let c2 = if op == OpType::Add       {1} else {0};
        let c3 = if op == OpType::Intersect {1} else {-1};

        let i12: Vec<i32> = self.x12.iter().map(|v| c3 * v).collect();
        let i21: Vec<i32> = self.x12.iter().map(|v| c3 * v).collect();
        let i03: Vec<i32> = self.w03.iter().map(|v| c1 + c3 * v).collect();
        let i30: Vec<i32> = self.w30.iter().map(|v| c1 + c3 * v).collect();

        let vpos_p = &vec![]; // need to be filled in
        let vpos_q = &vec![]; // need to be filled in
        let nv_p = self.mfd_p.hmesh.n_vert;
        let nh_p = self.mfd_p.hmesh.n_half;
        let nv_q = self.mfd_q.hmesh.n_vert;
        let nh_q = self.mfd_q.hmesh.n_half;
        let mut nv_r = 0;
        let mut vpos_r = vec![];

        let mut v_p2r: Vec<i32> = vec![0; nv_p];
        let mut v_q2r: Vec<i32> = vec![0; nv_q];
        let mut v_12r: Vec<i32> = vec![0; self.v12.len()];
        let mut v_21r: Vec<i32> = vec![0; self.v21.len()];

        exclusive_scan(&i03.iter().map(|i| i.abs()).collect::<Vec<_>>(), &mut v_p2r, nv_r);
        nv_r = v_p2r.last().unwrap().clone().abs() + i03.last().unwrap().abs(); // why adding i03 back??
        let nv_rp = nv_r;

        exclusive_scan(&i30.iter().map(|i| i.abs()).collect::<Vec<_>>(), &mut v_q2r, nv_r);
        nv_r = v_q2r.last().unwrap().clone().abs() + i03.last().unwrap().abs(); // why adding i30 back??
        let nv_rq = nv_r - nv_rp;

        exclusive_scan(&i12.iter().map(|i| i.abs()).collect::<Vec<_>>(), &mut v_12r, nv_r);
        nv_r = v_12r.last().unwrap().clone().abs() + i12.last().unwrap().abs(); 
        let nv_12 = nv_r - nv_rp - nv_rq;

        exclusive_scan(&i21.iter().map(|i| i.abs()).collect::<Vec<_>>(), &mut v_21r, nv_r);
        nv_r = v_21r.last().unwrap().clone().abs() + i21.last().unwrap().abs();
        let nv_21 = nv_r - nv_rp - nv_rq - nv_12;

        for i in 0..nv_p      { duplicate_verts(&i03, &v_p2r, vpos_p, &mut vpos_r, i); }
        for i in 0..nv_q      { duplicate_verts(&i30, &v_q2r, vpos_q, &mut vpos_r, i); }
        for i in 0..i12.len() { duplicate_verts(&i12, &v_12r, &self.v12, &mut vpos_r, i); }
        for i in 0..i21.len() { duplicate_verts(&i21, &v_21r, &self.v21, &mut vpos_r, i); }

        let mut whole_he_p = vec![true; nh_p];
        let mut whole_he_q = vec![true; nh_q];
    }
}

#[cfg(test)]
mod test_boolean_result {
    use crate::boolean::{tests, Boolean3};

    #[test]
    fn boolean_test() {
        let mfd_p = tests::gen_tet_a();
        let mfd_q = tests::gen_tet_c();


        //let boolean = Boolean3{
        //    mfd_p,
        //    mfd_q,
        //};
    }
}