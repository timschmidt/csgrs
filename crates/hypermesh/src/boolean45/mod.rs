use std::mem;
use std::collections::HashMap;
use crate::boolean03::Boolean03;
use crate::common::{face_of, Half, Tref, Row3f};
use crate::manifold::Manifold;
use crate::bounds::BBox;
use crate::OpType;

fn duplicate_verts(
    inc : &[i32],
    vt_r: &[i32],
    ps_p: &[Row3f],
    ps_r: &mut [Row3f],
    vid: usize
) {
    let n = inc[vid].abs() as usize;
    for i in 0..n {
        ps_r[vt_r[vid] as usize + i] = ps_p[vid];
    }
}

fn inclusive_scan(input: &[i32], output: &mut [i32], offset: i32) {
    if input.is_empty() || output.is_empty() { return; }
    let mut sum = offset;
    for (i, &v) in input.iter().enumerate() {
        sum += v;
        if i < output.len() { output[i] = sum; }
    }
}

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
    p1q2: &Vec<[usize; 2]>,
    p2q1: &Vec<[usize; 2]>,
    fns: &mut Vec<Row3f>,
    inv: bool, // whether to invert mesh of q
) -> (Vec<i32>, Vec<i32>) {
    let mut side_p = vec![0; mp.nf];
    let mut side_q = vec![0; mq.nf];

    // equivalent to CountVerts
    for (i, h) in mp.hs.iter().enumerate() { side_p[face_of(i)] += i03[h.tail].abs(); }
    for (i, h) in mq.hs.iter().enumerate() { side_q[face_of(i)] += i30[h.tail].abs(); }

    // equivalent to CountNewVerts
    for i in 0..i12.len() {
        let hid0 = p1q2[i][0];
        let hid1 = mp.hs[hid0].pair;
        let inc = i12[i].abs();
        side_p[face_of(hid0)] += inc;
        side_p[face_of(hid1)] += inc;
        side_q[p1q2[i][1]] += inc;
    }

    for i in 0..i21.len() {
        let hid0 = p2q1[i][1];
        let hid1 = mq.hs[hid0].pair;
        let inc = i21[i].abs();
        side_q[face_of(hid0)] += inc;
        side_q[face_of(hid1)] += inc;
        side_p[p2q1[i][0]] += inc;
    }

    // a map from face_p and face_q to face_r
    let mut face_pq2r = vec![0; mp.nf + mq.nf + 1];
    let side_pq = [&side_p[..], &side_q[..]].concat();
    let keep_fs = side_pq.iter().map(|&x| if x > 0 { 1 } else { 0 }).collect::<Vec<_>>();

    inclusive_scan(&keep_fs, &mut face_pq2r[1..], 0);
    let nf_r = *face_pq2r.last().unwrap() as usize;
    face_pq2r.truncate(mp.nf + mq.nf);
    fns.resize(nf_r, Row3f::zeros());

    let mut fid_r = 0;
    for (i, n) in mp.face_normals.iter().enumerate() { if side_p[i] > 0 { fns[fid_r] = *n; fid_r += 1; } }
    for (i, n) in mq.face_normals.iter().enumerate() { if side_q[i] > 0 { fns[fid_r] = *n * if inv {-1.} else {1.}; fid_r += 1; } }

    let truncated = side_pq.iter().filter(|s| **s > 0).map(|s| *s).collect::<Vec<_>>();
    let mut ih_per_f = vec![0; truncated.len()];

    inclusive_scan(&truncated, &mut ih_per_f, 0);
    ih_per_f.insert(0, 0);

    (ih_per_f, face_pq2r)
}

// Sort of intermediate data store for halfedge creation
#[derive(Clone, Debug)]
struct EdgePt {
    val: f64,     // dot value of edge
    vid: usize,   // vertex id
    cid: usize,   // collision id
    is_tail: bool //
}

fn add_new_edge_verts(
    p1q2: &Vec<[usize; 2]>,
    i12: &[i32],
    v12_r: &[i32],
    hs_p: &[Half],
    pt_old: &mut HashMap<usize, Vec<EdgePt>>,
    pt_new: &mut HashMap<(usize, usize), Vec<EdgePt>>,
    fwd: bool,
    oft: usize,
) {
    for i in 0..p1q2.len() {
        let hid_p = p1q2[i][if fwd {0} else {1}];
        let fid_q = p1q2[i][if fwd {1} else {0}];
        let vid_r = v12_r[i] as usize;
        let inc = i12[i];
        let hid0 = hid_p;
        let hid1 = hs_p[hid_p].pair;
        let key_l = if fwd { (face_of(hid0), fid_q) } else { (fid_q, face_of(hid0)) };
        let key_r = if fwd { (face_of(hid1), fid_q) } else { (fid_q, face_of(hid1)) };
        let dir = inc < 0;
        pt_old.entry(hid_p).or_insert_with(Vec::new);
        pt_new.entry(key_l).or_insert_with(Vec::new);
        pt_new.entry(key_r).or_insert_with(Vec::new);
        let dir0 = dir ^ !fwd;
        let dir1 = dir ^ fwd;
        let inc_ = inc.abs() as usize;
        for j in 0..inc_ { pt_old.get_mut(&hid_p).unwrap().push(EdgePt{ val: 0., vid: vid_r + j, cid: i + oft, is_tail: dir  }); }
        for j in 0..inc_ { pt_new.get_mut(&key_r).unwrap().push(EdgePt{ val: 0., vid: vid_r + j, cid: i + oft, is_tail: dir0 }); }
        for j in 0..inc_ { pt_new.get_mut(&key_l).unwrap().push(EdgePt{ val: 0., vid: vid_r + j, cid: i + oft, is_tail: dir1 }); }
    }
}

// Creating a partial halfedges from a list of positions.
// It's very confusing, but it's not aiming to pair twins (pair is -1).
// It's more likely to say pairing sta-end vertex and make a halfedge
fn pair_up(pts: &mut Vec<EdgePt>) -> Vec<Half> {
    assert_eq!(pts.len() % 2, 0);
    let nh = pts.len() / 2;
    let mid_idx = {
        let mut sta_idx = 0;
        let mut end_idx = pts.len();

        while sta_idx < end_idx {
            if pts[sta_idx].is_tail { sta_idx += 1; }
            else { end_idx -= 1; pts.swap(sta_idx, end_idx); }
        }
        sta_idx
    };

    let cmp = |a: &EdgePt, b: &EdgePt| {
        a.val.partial_cmp(&b.val)
            .unwrap_or(std::cmp::Ordering::Equal)
            .then_with(|| a.cid.cmp(&b.cid))
    };
    pts[..mid_idx].sort_by(cmp);
    pts[mid_idx..].sort_by(cmp);

    let mut edges = Vec::with_capacity(nh);
    for i in 0..nh {
        edges.push(Half::new_without_pair(pts[i].vid, pts[i + nh].vid));
    }
    edges
}

fn append_partial_edges(
    i03: &[i32],                            //
    hs_p: &[Half],                          // halfedges in mfd_p
    ps_p: &[Row3f],                         //
    ps_r: &[Row3f],                         // the vert pos of mfd_r, already fulfilled so far
    vid_p2r: &[i32],                        // map from vid in mfd_p to vid in mfd_r
    fid_p2r: &[i32],                        // map from fid in mfd_p to fid in mfd_r
    fwd: bool,                              //
    hs_r: &mut [Half],                      // halfedge data of mfd_r, empty yet
    rs_r: &mut [Tref],                      // map from halfedge in mfd_r to triangle info
    pt_p: &mut HashMap<usize, Vec<EdgePt>>, //
    face_ptr_r: &mut [i32],                 //
    whole_flag: &mut [bool],                // a flag to find out a halfedge from mfd_p is entirely usable in mfd_r
) {
    for (hid_p, pt) in pt_p {
        let mut hpos_p = pt;
        let h = &hs_p[*hid_p];
        whole_flag[*hid_p] = false;
        whole_flag[h.pair] = false;

        // assigning 0-1 value to hpos_p
        let dif = ps_p[h.head] - ps_p[h.tail];
        for p in hpos_p.iter_mut() { p.val = dif.dot(&ps_r[p.vid]); }

        let i_tail = i03[h.tail]; // mostly 0 or 1
        let i_head = i03[h.head]; // mostly 0 or 1
        let p_tail = ps_r[vid_p2r[h.tail] as usize];
        let p_head = ps_r[vid_p2r[h.head] as usize];

        for i in 0..i_tail.abs() as usize { hpos_p.push(EdgePt{ val: p_tail.dot(&dif), vid: vid_p2r[h.tail] as usize + i, cid: usize::MAX, is_tail: i_tail > 0 }); }
        for i in 0..i_head.abs() as usize { hpos_p.push(EdgePt{ val: p_head.dot(&dif), vid: vid_p2r[h.head] as usize + i, cid: usize::MAX, is_tail: i_head < 0 }); }

        let mut half_seq = pair_up(&mut hpos_p);
        let fp_l = face_of(*hid_p);
        let fp_r = face_of(h.pair);
        let fid_l = fid_p2r[fp_l] as usize;
        let fid_r = fid_p2r[fp_r] as usize;

        // Negative inclusion means the halfedges are reversed, which means our
        // reference is now to the head instead of the tail, which is one
        // position advanced CCW. This is only valid if this is a retained vert;
        // it will be ignored later if the vert is new.

        let fw_tri = Tref{ mid: if fwd {0} else {1}, fid: fp_l, ..Default::default() };
        let bk_tri = Tref{ mid: if fwd {0} else {1}, fid: fp_r, ..Default::default() };

        for h in half_seq.iter_mut() {
            let fw_edge = face_ptr_r[fid_l] as usize;
            let bk_edge = face_ptr_r[fid_r] as usize;
            face_ptr_r[fid_l] += 1;
            face_ptr_r[fid_r] += 1;
            hs_r[fw_edge] = Half::new(h.tail, h.head, bk_edge);
            hs_r[bk_edge] = Half::new(h.head, h.tail, fw_edge);
            rs_r[fw_edge] = fw_tri.clone();
            rs_r[bk_edge] = bk_tri.clone();
        }
    }
}

fn append_new_edges(
    ps_r: &[Row3f],                                    // the vert pos of mfd_r, already fulfilled so far
    fid_pq2r: &[i32],                                  //
    nf_p: usize,                                       // num of faces in mfd_p
    face_ptr_r: &mut[i32],                             //
    pt_new: &mut HashMap<(usize, usize), Vec<EdgePt>>, //
    hs_r: &mut [Half],                                 // the halfedge data of mfd_r, empty yet
    rs_r: &mut [Tref],                                 //
) {
    for ((fid_p, fid_q), pt_init) in pt_new.into_iter() {
        let mut pt = pt_init;
        let mut bb = BBox::default();
        for p in pt.iter() { bb.union(&ps_r[p.vid]); }

        let d = bb.longest_dim();
        for p in pt.iter_mut() {
            p.val = ps_r[p.vid][d];
        }

        let mut half_seq = pair_up(&mut pt);
        let fid_l = fid_pq2r[*fid_p] as usize;
        let fid_r = fid_pq2r[*fid_q + nf_p] as usize;
        let fw_ref = Tref { mid: 0, fid: *fid_p, ..Default::default() };
        let bk_ref = Tref { mid: 1, fid: *fid_q, ..Default::default() };

        for h in half_seq.iter_mut() {
            let fw_edge = face_ptr_r[fid_l] as usize;
            let bk_edge = face_ptr_r[fid_r] as usize;
            face_ptr_r[fid_l] += 1;
            face_ptr_r[fid_r] += 1;
            hs_r[fw_edge] = Half::new(h.tail, h.head, bk_edge);
            hs_r[bk_edge] = Half::new(h.head, h.tail, fw_edge);
            rs_r[fw_edge] = fw_ref.clone();
            rs_r[bk_edge] = bk_ref.clone();
        }
    }
}

fn append_whole_edges(
    i03: &[i32],
    half_p: &[Half],
    fid_p2r: &[i32],
    vid_p2r: &[i32],
    whole_flag: &[bool],
    fwd: bool,
    face_ptr_r: &mut[i32],
    hs_r: &mut[Half],
    rs_r: &mut[Tref],
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
        let fid_l = fid_p2r[fp_l] as usize;
        let fid_r = fid_p2r[fp_r] as usize;
        let fw_ref = Tref{ mid: if fwd {0} else {1}, fid: fp_l, ..Default::default() };
        let bk_ref = Tref{ mid: if fwd {0} else {1}, fid: fp_r, ..Default::default() };

        for _ in 0..inc.abs() as usize {
            let fw_edge = face_ptr_r[fid_l] as usize;
            let bk_edge = face_ptr_r[fid_r] as usize;
            face_ptr_r[fid_l] += 1;
            face_ptr_r[fid_r] += 1;
            hs_r[fw_edge] = Half::new(h.tail, h.head, bk_edge);
            hs_r[bk_edge] = Half::new(h.head, h.tail, fw_edge);
            rs_r[fw_edge] = fw_ref.clone();
            rs_r[bk_edge] = bk_ref.clone();
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
    pub hid_per_f: Vec<i32>,
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
    let mut nv = 0;
    let mut vid_p2r = vec![0; mp.nv];
    let mut vid_q2r = vec![0; mq.nv];
    let mut vid_12r = vec![0; b03.v12.len()];
    let mut vid_21r = vec![0; b03.v21.len()];

    exclusive_scan(&i03.iter().map(|i| i.abs()).collect::<Vec<_>>(), &mut vid_p2r, nv);
    nv = vid_p2r.last().unwrap().clone().abs() + i03.last().unwrap().abs();
    let nv_rp = nv;

    exclusive_scan(&i30.iter().map(|i| i.abs()).collect::<Vec<_>>(), &mut vid_q2r, nv);
    nv = vid_q2r.last().unwrap().clone().abs() + i30.last().unwrap().abs();
    let nv_rq = nv - nv_rp;

    if b03.v12.len() > 0 {
        exclusive_scan(&i12.iter().map(|i| i.abs()).collect::<Vec<_>>(), &mut vid_12r, nv);
        nv = vid_12r.last().unwrap().clone().abs() + i12.last().unwrap().abs();
    }
    let nv_12 = nv - nv_rp - nv_rq;

    if b03.v21.len() > 0 {
        exclusive_scan(&i21.iter().map(|i| i.abs()).collect::<Vec<_>>(), &mut vid_21r, nv);
        nv = vid_21r.last().unwrap().clone().abs() + i21.last().unwrap().abs();
    }
    let nv_21 = nv - nv_rp - nv_rq - nv_12;

    let mut ps_r = vec![Row3f::zeros(); nv as usize];

    for i in 0..mp.nv { duplicate_verts(&i03, &vid_p2r, &mp.ps, &mut ps_r, i); }
    for i in 0..mq.nv { duplicate_verts(&i30, &vid_q2r, &mq.ps, &mut ps_r, i); }
    for i in 0..nv_12 { duplicate_verts(&i12, &vid_12r, &b03.v12, &mut ps_r, i as usize); }
    for i in 0..nv_21 { duplicate_verts(&i21, &vid_21r, &b03.v21, &mut ps_r, i as usize); }

    let mut pt_p   = HashMap::new();
    let mut pt_q   = HashMap::new();
    let mut pt_new = HashMap::new();
    add_new_edge_verts(&b03.p1q2, &i12, &vid_12r, &mp.hs, &mut pt_p, &mut pt_new, true, 0);
    add_new_edge_verts(&b03.p2q1, &i21, &vid_21r, &mq.hs, &mut pt_q, &mut pt_new, false, b03.p1q2.len());

    let mut ns_r = vec![];
    let inv = op == &OpType::Subtract;
    let (hid_per_f, fid_pq2r) = size_output(mp, mq, &i03, &i30, &i12, &i21, &b03.p1q2, &b03.p2q1, &mut ns_r, inv);

    let nh = hid_per_f.last().unwrap().clone() as usize;
    let mut face_ptr_r = hid_per_f.clone();
    let mut whole_flag_p = vec![true; mp.nh];
    let mut whole_flag_q = vec![true; mq.nh];
    let mut rs_r = vec![Tref::default(); nh];
    let mut hs_r = vec![Half::default(); nh];
    let fid_p2r = &fid_pq2r[0..mp.nf];
    let fid_q2r = &fid_pq2r[mp.nf..];

    append_partial_edges(&i03, &mp.hs, &mp.ps, &ps_r, &vid_p2r, fid_p2r, true,  &mut hs_r, &mut rs_r, &mut pt_p, &mut face_ptr_r, &mut whole_flag_p);
    append_partial_edges(&i30, &mq.hs, &mq.ps, &ps_r, &vid_q2r, fid_q2r, false, &mut hs_r, &mut rs_r, &mut pt_q, &mut face_ptr_r, &mut whole_flag_q);

    append_new_edges(&ps_r, &fid_pq2r, mp.nf, &mut face_ptr_r, &mut pt_new, &mut hs_r, &mut rs_r);

    append_whole_edges(&i03, &mp.hs, fid_p2r, &vid_p2r, &whole_flag_p, true,  &mut face_ptr_r, &mut hs_r, &mut rs_r);
    append_whole_edges(&i30, &mq.hs, fid_q2r, &vid_q2r, &whole_flag_q, false, &mut face_ptr_r, &mut hs_r, &mut rs_r);

    Boolean45 {
        ps: ps_r,
        ns: ns_r,
        hs: hs_r,
        rs: rs_r,
        hid_per_f,
        nv_from_p: nv_rp as usize,
        nv_from_q: nv_rq as usize,
    }
}
