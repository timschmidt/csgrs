use nalgebra::{RowVector3, Vector3};
use std::collections::HashMap;
use std::mem;
use crate::boolean::Boolean3;
use crate::{Half, Halfedge, Manifold, OpType};
use crate::bounds::BoundingBox;

fn duplicate_verts(
    inclusion : &[i32],
    vert_r    : &[i32],
    vert_pos_p: &[RowVector3<f64>],
    vert_pos_r: &mut [RowVector3<f64>],
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
fn exclusive_scan(input: &[i32], output: &mut [i32], offset: i32) {
    if input.is_empty() || output.is_empty() { return; }
    let mut sum = offset;
    output[0] = sum;
    for i in 1..input.len() {
        sum += input[i - 1];
        if i < output.len() { output[i] = sum; }
    }
}

fn size_output(
    mfd_p: &Manifold,
    mfd_q: &Manifold,
    i03: &[i32],
    i30: &[i32],
    i12: &[i32],
    i21: &[i32],
    p1q2: &Vec<[i32; 2]>,
    p2q1: &Vec<[i32; 2]>,
    invert_q: bool,
    fnmls: &mut Vec<RowVector3<f64>>,
) -> (Vec<i32>, Vec<i32>) {
    let mut side_p = vec![0; mfd_p.hmesh.n_face];
    let mut side_q = vec![0; mfd_q.hmesh.n_face];
    let nfp = mfd_p.hmesh.n_face;
    let nfq = mfd_q.hmesh.n_face;

    // equivalent to CountVerts
    for h in mfd_p.hmesh.halfs.iter() { side_p[h.face().id] += i03[h.tail().id].abs(); }
    for h in mfd_q.hmesh.halfs.iter() { side_q[h.face().id] += i30[h.tail().id].abs(); }

    // equivalent to CountNewVerts
    for i in 0..i12.len() {
        let h = &mfd_p.hmesh.halfs[p1q2[i][0] as usize];
        let inc = i12[i].abs();
        side_p[h.face().id] += inc;
        side_p[h.twin().face().id] += inc;
        side_q[p1q2[i][1] as usize] += inc;
    }

    for i in 0..i21.len() {
        let h = &mfd_q.hmesh.halfs[p2q1[i][1] as usize];
        let inc = i21[i].abs();
        side_q[h.face().id] += inc;
        side_q[h.twin().face().id] += inc;
        side_p[p2q1[i][0] as usize] += inc;
    }

    // a map from face_p and face_q to face_r
    let mut face_pq2r = vec![0; nfp + nfq + 1];
    let side_pq = [&side_p[..], &side_q[..]].concat();
    //println!("side_pq: {:?}", side_pq);

    let keep_fs = side_pq.iter().map(|&x| if x > 0 { 1 } else { 0 }).collect::<Vec<i32>>();

    inclusive_scan(&keep_fs, &mut face_pq2r[1..], 0);
    let n_face_r = *face_pq2r.last().unwrap() as usize;
    face_pq2r.truncate(nfp + nfq);
    fnmls.resize(n_face_r, RowVector3::zeros());

    let mut fid_r = 0;
    for f in mfd_p.hmesh.faces.iter() { if side_p[f.id] > 0 { fnmls[fid_r] = f.normal().clone(); fid_r += 1; } }
    for f in mfd_q.hmesh.faces.iter() { if side_q[f.id] > 0 { fnmls[fid_r] = f.normal().clone() * if invert_q {-1.} else {1.}; fid_r += 1; } }

    // starting half idx per face todo: very suspicious...
    //mfd_r.halfs = vec![Halfedge::default(); face_pq2r.iter().sum::<i32>() as usize];
    let truncated = side_pq.iter().filter(|s| **s > 0).map(|s| *s).collect::<Vec<i32>>();
    let mut ih_per_f = vec![0; truncated.len()];

    inclusive_scan(&truncated, &mut ih_per_f, 0);
    ih_per_f.insert(0, 0);

    (ih_per_f, face_pq2r)
}

#[derive(Clone, Debug)]
struct EdgePos {
    val: f64, // dot value of edge
    vid: usize,
    cid: usize, // collision_id
    is_tail: bool
}

#[derive(Clone, Debug)]
struct TriRef {
    mesh_id: usize,
    origin_id: i32,
    face_id: usize,
    coplanar_id: i32,
}

fn add_new_edge_verts(
    p1q2: &Vec<[i32; 2]>,
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
        edges_pos.entry(hid_p).or_insert_with(Vec::new);
        edges_new.entry(key_l).or_insert_with(Vec::new);
        edges_new.entry(key_r).or_insert_with(Vec::new);
        let dir0 = direction ^ !forward;
        let dir1 = direction ^ forward;

        //println!("-----");
        for j in 0..inclusion.abs() as usize {
            //println!("inclusion: {}, direction: {}, vert: {}, edgeP: {}, faceQ: {}", inclusion, direction, vid_r, hid_p, fid_q);
            edges_pos.get_mut(&hid_p).unwrap().push(EdgePos { val: 0., vid: vid_r + j, cid: i + offset, is_tail: direction });
        }

        direction = !direction;

        for j in 0..inclusion.abs() as usize {
            //println!("inclusion: {}, direction: {}, vert: {}, edgeP: {}, faceQ: {}", inclusion, direction, vid_r, hid_p, fid_q);
            edges_new.get_mut(&key_r).unwrap().push(EdgePos{ val: 0., vid: vid_r + j, cid: i + offset, is_tail: dir0 });
        }

        direction = !direction;

        for j in 0..inclusion.abs() as usize {
            //println!("inclusion: {}, direction: {}, vert: {}, edgeP: {}, faceQ: {}", inclusion, direction, vid_r, hid_p, fid_q);
            edges_new.get_mut(&key_l).unwrap().push(EdgePos{ val: 0., vid: vid_r + j, cid: i + offset, is_tail: dir1 });
        }
    }
}

// creating a partial halfedges from a list of positions,
// it's very confusing, but it's not aiming to pair twins (pair is -1),
// more like to say pairing sta-end vertex and make a halfedge
fn pair_up(edge_pos: &mut Vec<EdgePos>) -> Vec<Halfedge> {
    assert_eq!(edge_pos.len() % 2, 0);
    let ne = edge_pos.len() / 2;
    let mid_idx = {
        let mut sta_idx = 0;
        let mut end_idx = edge_pos.len();

        while sta_idx < end_idx {
            if edge_pos[sta_idx].is_tail {
                sta_idx += 1;
            } else {
                end_idx -= 1;
                edge_pos.swap(sta_idx, end_idx);
            }
        }
        sta_idx
    };

    let cmp = |a: &EdgePos, b: &EdgePos| {
        a.val.partial_cmp(&b.val)
            .unwrap_or(std::cmp::Ordering::Equal)
            .then_with(|| a.cid.cmp(&b.cid))
    };

    edge_pos[..mid_idx].sort_by(cmp);
    edge_pos[mid_idx..].sort_by(cmp);

    let mut edges = Vec::with_capacity(ne);
    for i in 0..ne {
        edges.push(Halfedge {
            tail: edge_pos[i].vid as i32,
            head: edge_pos[i + ne].vid as i32,
            pair: -1,
        });
    }

    edges
}

fn append_partial_edges(
    i03: &[i32],                                 //
    half_p: &[Half],                             // halfedge of mfd_p
    vid_p2r: &[i32],                             // the table from vid of mfd_p to vid of mfd_r
    fid_p2r: &[i32],
    pos_res: &[RowVector3<f64>],                // the vert pos of mfd_r, already fulfilled so far
    forward: bool,                               //
    half_res: &mut [Halfedge],                   // the halfedge data of mfd_r, empty yet
    half_tri: &mut [TriRef],                     // the table from halfedge of mfd_r to triangle information
    half_pos: &mut HashMap<usize, Vec<EdgePos>>, //
    face_ptr_r: &mut [i32],                      //
    whole_flag: &mut [bool],                     // a flag to find out a halfedge from mfd_p is entirely usable in mfd_r
) {
    for e in half_pos {
        let hid_p = e.0.clone();
        println!("edgeP: {}, ", hid_p);
        let mut hpos_p = e.1;
        let h = &half_p[hid_p];
        whole_flag[h.id] = false;
        whole_flag[h.twin().id] = false;

        // assigning 0-1 value to hpos_p
        let dif = h.head().pos() - h.tail().pos();
        for p in hpos_p.iter_mut() {
            p.val = dif.dot(&pos_res[p.vid]);
        }

        let inc_tail = i03[h.tail().id]; // mostly 0 or 1
        let inc_head = i03[h.head().id]; // mostly 0 or 1
        let p_tail = pos_res[vid_p2r[h.tail().id] as usize];
        let p_head = pos_res[vid_p2r[h.head().id] as usize];

        for i in 0..inc_tail.abs() as usize {
            hpos_p.push(EdgePos{
                val: p_tail.dot(&dif),
                vid: vid_p2r[h.tail().id] as usize + i,
                cid: usize::MAX,
                is_tail: inc_tail > 0
            });
        }

        for i in 0..inc_head.abs() as usize {
            hpos_p.push(EdgePos{
                val: p_head.dot(&dif),
                vid: vid_p2r[h.head().id] as usize + i,
                cid: usize::MAX,
                is_tail: inc_head < 0
            });
        }

        let mut half_seq = pair_up(&mut hpos_p);
        let fp_l = h.face();
        let fp_r = h.twin().face();
        let fid_l = fid_p2r[fp_l.id] as usize;
        let fid_r = fid_p2r[fp_r.id] as usize;

        //println!("half_seq: {:?}, ", half_seq);
        //println!("face_l id: {}, ", fid_l);
        //println!("face_r id: {}, ", fid_r);

        // Negative inclusion means the halfedges are reversed, which means our
        // reference is now to the endVert instead of the startVert, which is one
        // position advanced CCW. This is only valid if this is a retained vert;
        // it will be ignored later if the vert is new.

        let fw_tri = TriRef{ mesh_id: if forward {0} else {1}, face_id: fp_l.id, origin_id: -1, coplanar_id: -1 };
        let bk_tri = TriRef{ mesh_id: if forward {0} else {1}, face_id: fp_r.id, origin_id: -1, coplanar_id: -1 };

        for h in half_seq.iter_mut() {
            let fw_edge = face_ptr_r[fid_l];
            let bk_edge = face_ptr_r[fid_r];
            face_ptr_r[fid_l] += 1;
            face_ptr_r[fid_r] += 1;
            half_res[fw_edge as usize] = Halfedge{ tail: h.tail, head: h.head, pair: bk_edge };
            half_res[bk_edge as usize] = Halfedge{ tail: h.head, head: h.tail, pair: fw_edge };
            half_tri[fw_edge as usize] = fw_tri.clone();
            half_tri[bk_edge as usize] = bk_tri.clone();
        }

        //println!("face_ptr_r: {:?}, ", face_ptr_r);
        //println!("half_res:");
        //for h in half_res.iter() { println!("h: {:?}, ", h); }
        //println!("half_tri:");
        //for t in half_tri.iter() { println!("t: {:?}, ", t); }
    }
}

fn append_new_edges(
    pos_res: &[RowVector3<f64>],                // the vert pos of mfd_r, already fulfilled so far
    fid_pq2r: &[i32],
    nfaces_p: usize,
    face_ptr_r: &mut[i32],
    half_new: &mut HashMap<(usize, usize), Vec<EdgePos>>,
    half_res: &mut [Halfedge],                   // the halfedge data of mfd_r, empty yet
    half_tri: &mut [TriRef],
) {
    for v in half_new.into_iter() {
        let (fid_p, fid_q) = v.0;
        let mut epos = v.1;
        let mut bbox = BoundingBox::new(usize::MAX, &vec![]);
        for p in epos.iter() { bbox.union(&pos_res[p.vid]); }

        println!("fid_p: {}, fid_q: {}", fid_p, fid_q);

        let d = bbox.longest_dim();
        for p in epos.iter_mut() {
            p.val = pos_res[p.vid][d];
        }

        let mut half_seq = pair_up(&mut epos);
        let fid_l = fid_pq2r[*fid_p] as usize;
        let fid_r = fid_pq2r[*fid_q + nfaces_p] as usize;
        let fw_ref = TriRef{ mesh_id: 0, face_id: *fid_p, origin_id: -1, coplanar_id: -1 };
        let bk_ref = TriRef{ mesh_id: 1, face_id: *fid_q, origin_id: -1, coplanar_id: -1 };

        for h in half_seq.iter_mut() {
            let fw_edge = face_ptr_r[fid_l];
            let bk_edge = face_ptr_r[fid_r];
            face_ptr_r[fid_l] += 1;
            face_ptr_r[fid_r] += 1;
            half_res[fw_edge as usize] = Halfedge{tail: h.tail, head: h.head, pair: bk_edge};
            half_res[bk_edge as usize] = Halfedge{tail: h.head, head: h.tail, pair: fw_edge};
            half_tri[fw_edge as usize] = fw_ref.clone();
            half_tri[bk_edge as usize] = bk_ref.clone();
        }
    }
}

fn append_whole_edges(
    i03: &[i32],
    half_p: &[Half],
    fid_p2r: &[i32],
    vid_p2r: &[i32],
    whole_flag: &[bool],
    forward: bool,
    face_ptr_r: &mut[i32],
    half_res: &mut [Halfedge],
    half_ref: &mut [TriRef],
) {
    for i in 0..half_p.len() {
        if !whole_flag[i] { continue; }
        let hp = half_p[i].clone();
        let mut h = Halfedge {
            tail: hp.tail().id as i32,
            head: hp.head().id as i32,
            pair: hp.twin().id as i32
        };
        if !h.is_forward() { continue; }

        let inc = i03[h.tail as usize];
        if inc == 0 { continue; }
        if inc < 0 { mem::swap(&mut h.tail, &mut h.head); }

        h.tail = vid_p2r[h.tail as usize];
        h.head = vid_p2r[h.head as usize];

        let fp_l = hp.face().id;
        let fp_r = hp.twin().face().id;
        let fid_l = fid_p2r[fp_l];
        let fid_r = fid_p2r[fp_r];
        let fw_ref = TriRef{ mesh_id: if forward {0} else {1}, face_id: fp_l, origin_id: -1, coplanar_id: -1 };
        let bk_ref = TriRef{ mesh_id: if forward {0} else {1}, face_id: fp_r, origin_id: -1, coplanar_id: -1 };

        for j in 0..inc.abs() as usize {
            let fw_edge = face_ptr_r[fid_l as usize];
            let bk_edge = face_ptr_r[fid_r as usize];
            face_ptr_r[fid_l as usize] += 1;
            face_ptr_r[fid_r as usize] += 1;
            half_res[fw_edge as usize] = Halfedge{ tail: h.tail, head: h.head, pair: bk_edge };
            half_res[bk_edge as usize] = Halfedge{ tail: h.head, head: h.tail, pair: fw_edge };
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
        let i21: Vec<i32> = self.x21.iter().map(|v| c3 * v).collect();
        let i03: Vec<i32> = self.w03.iter().map(|v| c1 + c3 * v).collect();
        let i30: Vec<i32> = self.w30.iter().map(|v| c2 + c3 * v).collect();

        let nv_p = self.mfd_p.hmesh.n_vert;
        let nh_p = self.mfd_p.hmesh.n_half;
        let nf_p = self.mfd_p.hmesh.n_face;
        let nv_q = self.mfd_q.hmesh.n_vert;
        let nh_q = self.mfd_q.hmesh.n_half;
        let mut nv_r = 0;
        let mut vid_p2r = vec![0; nv_p];
        let mut vid_q2r = vec![0; nv_q];
        let mut vid_12r = vec![0; self.v12.len()];
        let mut vid_21r = vec![0; self.v21.len()];

        exclusive_scan(&i03.iter().map(|i| i.abs()).collect::<Vec<_>>(), &mut vid_p2r, nv_r);
        nv_r = vid_p2r.last().unwrap().clone().abs() + i03.last().unwrap().abs();
        let nv_rp = nv_r;

        exclusive_scan(&i30.iter().map(|i| i.abs()).collect::<Vec<_>>(), &mut vid_q2r, nv_r);
        nv_r = vid_q2r.last().unwrap().clone().abs() + i30.last().unwrap().abs();
        let nv_rq = nv_r - nv_rp;

        if self.v12.len() > 0 {
            exclusive_scan(&i12.iter().map(|i| i.abs()).collect::<Vec<_>>(), &mut vid_12r, nv_r);
            nv_r = vid_12r.last().unwrap().clone().abs() + i12.last().unwrap().abs();
        }
        let nv_12 = nv_r - nv_rp - nv_rq;

        if self.v21.len() > 0 {
            exclusive_scan(&i21.iter().map(|i| i.abs()).collect::<Vec<_>>(), &mut vid_21r, nv_r);
            nv_r = vid_21r.last().unwrap().clone().abs() + i21.last().unwrap().abs();
        }
        let nv_21 = nv_r - nv_rp - nv_rq - nv_12;

        let mut vpos_p = vec![];
        let mut vpos_q = vec![];
        let mut vpos_r = vec![RowVector3::zeros(); nv_r as usize];
        for r in self.mfd_p.hmesh.pos.row_iter() { vpos_p.push(RowVector3::new(r[0], r[1], r[2])); }
        for r in self.mfd_q.hmesh.pos.row_iter() { vpos_q.push(RowVector3::new(r[0], r[1], r[2])); }

        for i in 0..nv_p  { duplicate_verts(&i03, &vid_p2r, &vpos_p, &mut vpos_r, i); }
        for i in 0..nv_q  { duplicate_verts(&i30, &vid_q2r, &vpos_q, &mut vpos_r, i); }
        for i in 0..nv_12 { duplicate_verts(&i12, &vid_12r, &self.v12, &mut vpos_r, i as usize); }
        for i in 0..nv_21 { duplicate_verts(&i21, &vid_21r, &self.v21, &mut vpos_r, i as usize); }


        let mut half_pos_p: HashMap<usize, Vec<EdgePos>> = HashMap::new();
        let mut half_pos_q: HashMap<usize, Vec<EdgePos>> = HashMap::new();
        let mut half_new: HashMap<(usize, usize), Vec<EdgePos>> = HashMap::new();
        add_new_edge_verts(&self.p1q2, &i12, &vid_12r, &self.mfd_p.hmesh.halfs, true, 0, &mut half_pos_p, &mut half_new);
        add_new_edge_verts(&self.p2q1, &i21, &vid_21r, &self.mfd_q.hmesh.halfs, false, self.p1q2.len(), &mut half_pos_q, &mut half_new);

        //println!("edge_pos_p: {:#?}", edges_pos_p);
        //println!("edge_pos_q: {:#?}", edges_pos_q);
        //println!("edge_new: {:#?}", edges_new);

        let mut fnmls = vec![];
        let (ih_per_f, fid_pq2r) = size_output(
            &self.mfd_p,
            &self.mfd_q,
            &i03,
            &i30,
            &i12,
            &i21,
            &self.p1q2,
            &self.p2q1,
            op == OpType::Subtract,
            &mut fnmls
        );

        //println!("ih_per_face: {:?}", ih_per_f);
        //println!("face_pq2r: {:?}", face_pq2r);
        //for n in fnmls.iter() { println!("n: ({}, {}, {})", n.x, n.y, n.z); }

        let nh = ih_per_f.last().unwrap().clone() as usize;
        let mut face_ptr_r = ih_per_f.clone();
        let mut whole_flag_p = vec![true; nh_p];
        let mut whole_flag_q = vec![true; nh_q];
        let mut half_tri = vec![TriRef{ mesh_id: 0, face_id: 0, origin_id: 0, coplanar_id: 0}; nh];
        let mut half_res = vec![Halfedge {tail : 0, head: 0, pair: 0}; nh];
        let fid_p2r = &fid_pq2r[0..nf_p];
        let fid_q2r = &fid_pq2r[nf_p..];

        append_partial_edges(&i03, &self.mfd_p.hmesh.halfs, &vid_p2r, fid_p2r, &vpos_r, true,  &mut half_res, &mut half_tri, &mut half_pos_p, &mut face_ptr_r, &mut whole_flag_p);
        append_partial_edges(&i30, &self.mfd_q.hmesh.halfs, &vid_q2r, fid_q2r, &vpos_r, false, &mut half_res, &mut half_tri, &mut half_pos_q, &mut face_ptr_r, &mut whole_flag_q);

        //println!("====== half_res");
        //for h in half_res.iter() { println!("h: {:?}", h); }

        append_new_edges(&vpos_r, &fid_pq2r, nf_p, &mut face_ptr_r, &mut half_new, &mut half_res, &mut half_tri);

        //println!("====== half_res");
        //for h in half_res.iter() { println!("h: {:?}", h); }

        let fid_p2r = &fid_pq2r[0..nf_p];
        let fid_q2r = &fid_pq2r[nf_p..];

        append_whole_edges(&i03, &self.mfd_p.hmesh.halfs, fid_p2r, &vid_p2r, &whole_flag_p, true,  &mut face_ptr_r, &mut half_res, &mut half_tri);
        append_whole_edges(&i30, &self.mfd_q.hmesh.halfs, fid_q2r, &vid_q2r, &whole_flag_q, false, &mut face_ptr_r, &mut half_res, &mut half_tri);

        println!("====== half_res");
        for h in half_res.iter() { println!("h: {:?}", h); }

        //println!("====== half_tri");   for t in half_tri.iter() { println!("t: {:?}", t); }
        //println!("====== half_pos_p"); for h in half_pos_p.iter() { println!("h: {:?}", h); }
        //println!("====== half_pos_q"); for h in half_pos_q.iter() { println!("h: {:?}", h); }
        //println!("====== half_new");   for h in half_new.iter() { println!("h: {:?}", h); }
    }
}
