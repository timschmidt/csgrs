use std::collections::HashMap;
use std::mem;
use crate::boolean03::Boolean03;
use crate::common::{face_of, Half, OpType, Tref, Row3f};
use crate::manifold::Manifold;
use crate::bounds::BBox;

fn duplicate_verts(
    inclusion: &[i32],
    vert_r: &[i32],
    ps_p: &[Row3f],
    ps_r: &mut [Row3f],
    vid: usize
) {
    let n = inclusion[vid].abs() as usize;
    for i in 0..n {
        ps_r[vert_r[vid] as usize + i] = ps_p[vid];
    }
}

// consider moving this to a util function module
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
    mp: &Manifold,
    mq: &Manifold,
    i03: &[i32],
    i30: &[i32],
    i12: &[i32],
    i21: &[i32],
    p1q2: &Vec<[i32; 2]>,
    p2q1: &Vec<[i32; 2]>,
    invert_q: bool,
    fns: &mut Vec<Row3f>,
) -> (Vec<i32>, Vec<i32>) {
    let mut side_p = vec![0; mp.nt];
    let mut side_q = vec![0; mq.nt];

    // equivalent to CountVerts
    for (i, h) in mp.hs.iter().enumerate() { side_p[face_of(i)] += i03[h.tail].abs(); }
    for (i, h) in mq.hs.iter().enumerate() { side_q[face_of(i)] += i30[h.tail].abs(); }

    // equivalent to CountNewVerts
    for i in 0..i12.len() {
        let hid0 = p1q2[i][0] as usize;
        let hid1 = mp.hs[hid0].pair;
        let inc = i12[i].abs();
        side_p[face_of(hid0)] += inc;
        side_p[face_of(hid1)] += inc;
        side_q[p1q2[i][1] as usize] += inc;
    }

    for i in 0..i21.len() {
        let hid0 = p2q1[i][1] as usize;
        let hid1 = mq.hs[hid0].pair;
        let inc = i21[i].abs();
        side_q[face_of(hid0)] += inc;
        side_q[face_of(hid1)] += inc;
        side_p[p2q1[i][0] as usize] += inc;
    }

    // a map from face_p and face_q to face_r
    let mut face_pq2r = vec![0; mp.nt + mq.nt + 1];
    let side_pq = [&side_p[..], &side_q[..]].concat();
    let keep_fs = side_pq.iter().map(|&x| if x > 0 { 1 } else { 0 }).collect::<Vec<i32>>();

    inclusive_scan(&keep_fs, &mut face_pq2r[1..], 0);
    let n_face_r = *face_pq2r.last().unwrap() as usize;
    face_pq2r.truncate(mp.nt + mq.nt);
    fns.resize(n_face_r, Row3f::zeros());

    let mut fid_r = 0;
    for (i, n) in mp.fns.iter().enumerate() { if side_p[i] > 0 { fns[fid_r] = *n; fid_r += 1; } }
    for (i, n) in mq.fns.iter().enumerate() { if side_q[i] > 0 { fns[fid_r] = *n * if invert_q {-1.} else {1.}; fid_r += 1; } }

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
    val: f64,     // dot value of edge
    vid: usize,   //
    cid: usize,   // collision_id
    is_tail: bool //
}

fn add_new_edge_verts(
    p1q2: &Vec<[i32; 2]>,
    i12: &[i32],
    v12_r: &[i32],
    hs_p: &[Half],
    fwd: bool,
    offset: usize,
    edges_pos: &mut HashMap<usize, Vec<EdgePos>>,
    edges_new: &mut HashMap<(usize, usize), Vec<EdgePos>>,
) {
    for i in 0..p1q2.len() {
        let hid_p = p1q2[i][if fwd {0} else {1}] as usize;
        let fid_q = p1q2[i][if fwd {1} else {0}] as usize;
        let vid_r = v12_r[i] as usize;
        let inclusion = i12[i];

        let hid0 = hid_p;
        let hid1 = hs_p[hid_p].pair;
        let key_l = if fwd { (face_of(hid0), fid_q) } else { (fid_q, face_of(hid0)) };
        let key_r = if fwd { (face_of(hid1), fid_q) } else { (fid_q, face_of(hid1)) };
        let mut direction = inclusion < 0;
        edges_pos.entry(hid_p).or_insert_with(Vec::new);
        edges_new.entry(key_l).or_insert_with(Vec::new);
        edges_new.entry(key_r).or_insert_with(Vec::new);
        let dir0 = direction ^ !fwd;
        let dir1 = direction ^ fwd;

        for j in 0..inclusion.abs() as usize {
            edges_pos.get_mut(&hid_p).unwrap().push(EdgePos{ val: 0., vid: vid_r + j, cid: i + offset, is_tail: direction });
        }

        direction = !direction;

        for j in 0..inclusion.abs() as usize {
            edges_new.get_mut(&key_r).unwrap().push(EdgePos{ val: 0., vid: vid_r + j, cid: i + offset, is_tail: dir0 });
        }

        direction = !direction;

        for j in 0..inclusion.abs() as usize {
            edges_new.get_mut(&key_l).unwrap().push(EdgePos{ val: 0., vid: vid_r + j, cid: i + offset, is_tail: dir1 });
        }
    }
}

// Creating a partial halfedges from a list of positions.
// It's very confusing, but it's not aiming to pair twins (pair is -1).
// It's more likely to say pairing sta-end vertex and make a halfedge
fn pair_up(edge_pos: &mut Vec<EdgePos>) -> Vec<Half> {
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
        edges.push(Half {
            tail: edge_pos[i].vid,
            head: edge_pos[i + ne].vid,
            pair: usize::MAX,
        });
    }

    edges
}

// kernel22
fn append_partial_edges(
    i03: &[i32],                                 //
    half_p: &[Half],                             // halfedges in mfd_p
    vpos_p: &[Row3f],                            //
    vid_p2r: &[i32],                             // map from vid in mfd_p to vid in mfd_r
    fid_p2r: &[i32],                             // map from fid in mfd_p to fid in mfd_r
    pos_res: &[Row3f],                           // the vert pos of mfd_r, already fulfilled so far
    forward: bool,                               //
    half_res: &mut [Half],                       // halfedge data of mfd_r, empty yet
    half_tri: &mut [Tref],                       // map from halfedge in mfd_r to triangle info
    half_pos: &mut HashMap<usize, Vec<EdgePos>>, //
    face_ptr_r: &mut [i32],                      //
    whole_flag: &mut [bool],                     // a flag to find out a halfedge from mfd_p is entirely usable in mfd_r
) {
    for e in half_pos {
        let hid_p = e.0.clone();
        let mut hpos_p = e.1;
        let h = &half_p[hid_p];
        whole_flag[hid_p] = false;
        whole_flag[h.pair] = false;

        // assigning 0-1 value to hpos_p
        let dif = vpos_p[h.head] - vpos_p[h.tail];
        for p in hpos_p.iter_mut() {
            p.val = dif.dot(&pos_res[p.vid]);
        }

        let inc_tail = i03[h.tail]; // mostly 0 or 1
        let inc_head = i03[h.head]; // mostly 0 or 1
        let p_tail = pos_res[vid_p2r[h.tail] as usize];
        let p_head = pos_res[vid_p2r[h.head] as usize];

        for i in 0..inc_tail.abs() as usize {
            hpos_p.push(EdgePos{
                val: p_tail.dot(&dif),
                vid: vid_p2r[h.tail] as usize + i,
                cid: usize::MAX,
                is_tail: inc_tail > 0
            });
        }

        for i in 0..inc_head.abs() as usize {
            hpos_p.push(EdgePos{
                val: p_head.dot(&dif),
                vid: vid_p2r[h.head] as usize + i,
                cid: usize::MAX,
                is_tail: inc_head < 0
            });
        }

        let mut half_seq = pair_up(&mut hpos_p);
        let fp_l = face_of(hid_p);
        let fp_r = face_of(h.pair);
        let fid_l = fid_p2r[fp_l] as usize;
        let fid_r = fid_p2r[fp_r] as usize;

        // Negative inclusion means the halfedges are reversed, which means our
        // reference is now to the endVert instead of the startVert, which is one
        // position advanced CCW. This is only valid if this is a retained vert;
        // it will be ignored later if the vert is new.

        let fw_tri = Tref{ mesh_id: if forward {0} else {1}, face_id: fp_l, ..Default::default() };
        let bk_tri = Tref{ mesh_id: if forward {0} else {1}, face_id: fp_r, ..Default::default() };

        for h in half_seq.iter_mut() {
            let fw_edge = face_ptr_r[fid_l] as usize;
            let bk_edge = face_ptr_r[fid_r] as usize;
            face_ptr_r[fid_l] += 1;
            face_ptr_r[fid_r] += 1;
            half_res[fw_edge] = Half{ tail: h.tail, head: h.head, pair: bk_edge };
            half_res[bk_edge] = Half{ tail: h.head, head: h.tail, pair: fw_edge };
            half_tri[fw_edge] = fw_tri.clone();
            half_tri[bk_edge] = bk_tri.clone();
        }
    }
}

// kernel13
fn append_new_edges(
    pos_res: &[Row3f],                                     // the vert pos of mfd_r, already fulfilled so far
    fid_pq2r: &[i32],                                      //
    nfaces_p: usize,                                       //
    face_ptr_r: &mut[i32],                                 //
    half_new: &mut HashMap<(usize, usize), Vec<EdgePos>>,  //
    half_res: &mut [Half],                                 // the halfedge data of mfd_r, empty yet
    half_tri: &mut [Tref],                                 //
) {
    for v in half_new.into_iter() {
        let (fid_p, fid_q) = v.0;
        let mut epos = v.1;
        let mut bbox = BBox::new(usize::MAX, &vec![]);
        for p in epos.iter() { bbox.union(&pos_res[p.vid]); }

        let d = bbox.longest_dim();
        for p in epos.iter_mut() {
            p.val = pos_res[p.vid][d];
        }

        let mut half_seq = pair_up(&mut epos);
        let fid_l = fid_pq2r[*fid_p] as usize;
        let fid_r = fid_pq2r[*fid_q + nfaces_p] as usize;
        let fw_ref = Tref{ mesh_id: 0, face_id: *fid_p, ..Default::default() };
        let bk_ref = Tref{ mesh_id: 1, face_id: *fid_q, ..Default::default() };

        for h in half_seq.iter_mut() {
            let fw_edge = face_ptr_r[fid_l];
            let bk_edge = face_ptr_r[fid_r];
            face_ptr_r[fid_l] += 1;
            face_ptr_r[fid_r] += 1;
            half_res[fw_edge as usize] = Half{tail: h.tail, head: h.head, pair: bk_edge as usize};
            half_res[bk_edge as usize] = Half{tail: h.head, head: h.tail, pair: fw_edge as usize};
            half_tri[fw_edge as usize] = fw_ref.clone();
            half_tri[bk_edge as usize] = bk_ref.clone();
        }
    }
}

// kernel31
fn append_whole_edges(
    i03: &[i32],
    half_p: &[Half],
    fid_p2r: &[i32],
    vid_p2r: &[i32],
    whole_flag: &[bool],
    forward: bool,
    face_ptr_r: &mut[i32],
    half_res: &mut [Half],
    half_ref: &mut [Tref],
) {
    for (i, hp) in half_p.iter().enumerate() {
        if !whole_flag[i] || !hp.is_forward() { continue; }

        let mut h = hp.clone();
        let inc = i03[h.tail];
        if inc == 0 { continue; }
        if inc < 0 { mem::swap(&mut h.tail, &mut h.head); }

        h.tail = vid_p2r[h.tail] as usize;
        h.head = vid_p2r[h.head] as usize;

        let fp_l = face_of(i);
        let fp_r = face_of(hp.pair);
        let fid_l = fid_p2r[fp_l];
        let fid_r = fid_p2r[fp_r];
        let fw_ref = Tref{ mesh_id: if forward {0} else {1}, face_id: fp_l, ..Default::default() };
        let bk_ref = Tref{ mesh_id: if forward {0} else {1}, face_id: fp_r, ..Default::default() };

        for _ in 0..inc.abs() as usize {
            let fw_edge = face_ptr_r[fid_l as usize];
            let bk_edge = face_ptr_r[fid_r as usize];
            face_ptr_r[fid_l as usize] += 1;
            face_ptr_r[fid_r as usize] += 1;
            half_res[fw_edge as usize] = Half{ tail: h.tail, head: h.head, pair: bk_edge as usize };
            half_res[bk_edge as usize] = Half{ tail: h.head, head: h.tail, pair: fw_edge as usize };
            half_ref[fw_edge as usize] = fw_ref.clone();
            half_ref[bk_edge as usize] = bk_ref.clone();
            h.tail += 1;
            h.head += 1;
        }
    }
}

pub struct Boolean45 {
    pub ps: Vec<Row3f>,
    pub ns: Vec<Row3f>,
    pub hs: Vec<Half>,
    pub rs: Vec<Tref>,
    pub initial_hid_per_faces: Vec<i32>,
    pub nv_from_p: usize,
    pub nv_from_q: usize,
}

pub fn boolean45(
    mp: &Manifold,
    mq: &Manifold,
    b03: &Boolean03,
    op: &OpType
) -> Boolean45 {
    let c1 = if op == &OpType::Intersect {0} else {1};
    let c2 = if op == &OpType::Add       {1} else {0};
    let c3 = if op == &OpType::Intersect {1} else {-1};
    let i12: Vec<i32> = b03.x12.iter().map(|v| c3 * v).collect();
    let i21: Vec<i32> = b03.x21.iter().map(|v| c3 * v).collect();
    let i03: Vec<i32> = b03.w03.iter().map(|v| c1 + c3 * v).collect();
    let i30: Vec<i32> = b03.w30.iter().map(|v| c2 + c3 * v).collect();
    let nv_p = mp.nv;
    let nh_p = mp.nh;
    let nf_p = mp.nt;
    let nv_q = mq.nv;
    let nh_q = mq.nh;
    let mut nv_r = 0;
    let mut vid_p2r = vec![0; nv_p];
    let mut vid_q2r = vec![0; nv_q];
    let mut vid_12r = vec![0; b03.v12.len()];
    let mut vid_21r = vec![0; b03.v21.len()];

    exclusive_scan(&i03.iter().map(|i| i.abs()).collect::<Vec<_>>(), &mut vid_p2r, nv_r);
    nv_r = vid_p2r.last().unwrap().clone().abs() + i03.last().unwrap().abs();
    let nv_rp = nv_r;

    exclusive_scan(&i30.iter().map(|i| i.abs()).collect::<Vec<_>>(), &mut vid_q2r, nv_r);
    nv_r = vid_q2r.last().unwrap().clone().abs() + i30.last().unwrap().abs();
    let nv_rq = nv_r - nv_rp;

    if b03.v12.len() > 0 {
        exclusive_scan(&i12.iter().map(|i| i.abs()).collect::<Vec<_>>(), &mut vid_12r, nv_r);
        nv_r = vid_12r.last().unwrap().clone().abs() + i12.last().unwrap().abs();
    }
    let nv_12 = nv_r - nv_rp - nv_rq;

    if b03.v21.len() > 0 {
        exclusive_scan(&i21.iter().map(|i| i.abs()).collect::<Vec<_>>(), &mut vid_21r, nv_r);
        nv_r = vid_21r.last().unwrap().clone().abs() + i21.last().unwrap().abs();
    }
    let nv_21 = nv_r - nv_rp - nv_rq - nv_12;

    let mut ps_r = vec![Row3f::zeros(); nv_r as usize];

    for i in 0..nv_p  { duplicate_verts(&i03, &vid_p2r, &mp.ps, &mut ps_r, i); }
    for i in 0..nv_q  { duplicate_verts(&i30, &vid_q2r, &mq.ps, &mut ps_r, i); }
    for i in 0..nv_12 { duplicate_verts(&i12, &vid_12r, &b03.v12, &mut ps_r, i as usize); }
    for i in 0..nv_21 { duplicate_verts(&i21, &vid_21r, &b03.v21, &mut ps_r, i as usize); }

    let mut half_pos_p: HashMap<usize, Vec<EdgePos>> = HashMap::new();
    let mut half_pos_q: HashMap<usize, Vec<EdgePos>> = HashMap::new();
    let mut half_new: HashMap<(usize, usize), Vec<EdgePos>> = HashMap::new();
    add_new_edge_verts(&b03.p1q2, &i12, &vid_12r, &mp.hs, true, 0, &mut half_pos_p, &mut half_new);
    add_new_edge_verts(&b03.p2q1, &i21, &vid_21r, &mq.hs, false, b03.p1q2.len(), &mut half_pos_q, &mut half_new);

    let mut fnmls = vec![];
    let (ih_per_f, fid_pq2r) = size_output(mp, mq, &i03, &i30, &i12, &i21, &b03.p1q2, &b03.p2q1, op == &OpType::Subtract, &mut fnmls);

    let nh = ih_per_f.last().unwrap().clone() as usize;
    let mut face_ptr_r = ih_per_f.clone();
    let mut whole_flag_p = vec![true; nh_p];
    let mut whole_flag_q = vec![true; nh_q];
    let mut half_tri = vec![Tref::default(); nh];
    let mut half_res = vec![Half::default(); nh];
    let fid_p2r = &fid_pq2r[0..nf_p];
    let fid_q2r = &fid_pq2r[nf_p..];

    append_partial_edges(&i03, &mp.hs, &mp.ps, &vid_p2r, fid_p2r, &ps_r, true,  &mut half_res, &mut half_tri, &mut half_pos_p, &mut face_ptr_r, &mut whole_flag_p);
    append_partial_edges(&i30, &mq.hs, &mq.ps, &vid_q2r, fid_q2r, &ps_r, false, &mut half_res, &mut half_tri, &mut half_pos_q, &mut face_ptr_r, &mut whole_flag_q);

    append_new_edges(&ps_r, &fid_pq2r, nf_p, &mut face_ptr_r, &mut half_new, &mut half_res, &mut half_tri);

    append_whole_edges(&i03, &mp.hs, fid_p2r, &vid_p2r, &whole_flag_p, true,  &mut face_ptr_r, &mut half_res, &mut half_tri);
    append_whole_edges(&i30, &mq.hs, fid_q2r, &vid_q2r, &whole_flag_q, false, &mut face_ptr_r, &mut half_res, &mut half_tri);

    Boolean45 {
        ps: ps_r,
        ns: fnmls,
        hs: half_res,
        rs: half_tri,
        nv_from_p: nv_rp as usize,
        nv_from_q: nv_rq as usize,
        initial_hid_per_faces: ih_per_f
    }

}
