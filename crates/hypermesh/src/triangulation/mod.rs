pub mod ear_clip;
pub mod flat_tree;
use anyhow::Result;
use std::collections::{BTreeMap, VecDeque};
use crate::boolean45::Boolean45;
use crate::common::{Half, Tref, get_axis_aligned_projection, is_ccw_3d, Row3f, Row3u, Row2f, Mat23};
use crate::Manifold;
use crate::triangulation::ear_clip::EarClip;

pub struct Triangulation {
    pub ts: Vec<Row3u>,
    pub ns: Vec<Row3f>,
    pub rs: Vec<Tref>,
}

pub fn triangulate(
    mp: &Manifold,
    mq: &Manifold,
    b45: &Boolean45,
    eps: f64
) -> Result<Triangulation> {
    let mut ts = vec![];
    let mut ns = vec![];
    let mut rs = vec![];
    for fid in 0..b45.initial_hid_per_faces.len() - 1 {
        let hid = b45.initial_hid_per_faces[fid] as usize;
        let t = process_face(&b45, fid, eps);
        let r = b45.rs[hid].clone();
        let n = b45.ns[fid].clone();
        rs.extend(vec![r; t.len()]);
        ns.extend(vec![n; t.len()]);
        ts.extend(t);
    }
    update_reference(mp, mq, &mut rs);
    Ok(Triangulation { ts, ns, rs })
}

fn process_face(
    b45: &Boolean45,
    fid: usize,
    eps: f64
) -> Vec<Row3u> {
    let e0 = b45.initial_hid_per_faces[fid] as usize;
    let e1 = b45.initial_hid_per_faces[fid + 1] as usize;
    match e1 - e0 {
        3 =>  single_triangulate(&b45, e0),
        4 =>  square_triangulate(&b45, fid, eps),
        _ => general_triangulate(&b45, fid, eps),
    }
}

fn update_reference(
    mp: &Manifold,
    mq: &Manifold,
    rs: &mut[Tref],
) {
    for r in rs.iter_mut() {
        let fid = r.face_id;
        let pq = r.mesh_id == 0;
        r.face_id = 0; // todo: see original code and it's always -1
        r.planar_id = if pq { mp.coplanar[fid] }
        else  { mq.coplanar[fid] };
    }
}


fn get_indices(hs: &[Half], t: &Row3u) -> Row3u {
    Row3u::new(hs[t.x].tail, hs[t.y].tail, hs[t.z].tail)
}

// This function considers vertex-joint cases like Hierholzer's algorithm.
// https://algorithms.discrete.ma.tum.de/graph-algorithms/hierholzer/index_en.html
// But not sure when some inner loops in an outer loop case happen, or two separate loops could be happened.
// Solved: Inner loop is always cw, so when ear clipping comes, it is guaranteed to be a single concave loop.
fn assemble_halfs(hs: &[Half], hid_f: &[i32], fid: usize) -> Vec<Vec<usize>> {
    let bgn = hid_f[fid] as usize;
    let end = hid_f[fid + 1] as usize;
    let num = end - bgn;
    let mut v2h = BTreeMap::new();

    for i in bgn..bgn + num {
        let id = hs[i].tail;
        v2h.entry(id).or_insert_with(VecDeque::new).push_front(i);
    }

    let mut loops: Vec<Vec<usize>> = vec![];
    let mut hid0 = 0;
    let mut hid1 = 0;
    loop {
        if hid1 == hid0 {
            if v2h.is_empty() { break; }
            hid0 = v2h.first_entry().unwrap().get().back().copied().unwrap();
            hid1 = hid0;
            loops.push(Vec::new());
        }
        loops.last_mut().unwrap().push(hid1);
        hid1 = v2h.get_mut(&hs[hid1].head).unwrap().pop_back().unwrap();
        v2h.retain(|_, vq| !vq.is_empty());
    }
    loops
}

fn single_triangulate(
    b45: &Boolean45,
    hid: usize
) -> Vec<Row3u> {
    let mut idcs = [hid, hid + 1, hid + 2];
    let mut tails = vec![];
    let mut heads = vec![];
    for id in idcs.iter() {
        tails.push(b45.hs[*id].tail);
        heads.push(b45.hs[*id].head);
    }
    if heads[0] == tails[2] { idcs.swap(1, 2); }

    vec![Row3u::new(
        b45.hs[idcs[0]].tail,
        b45.hs[idcs[1]].tail,
        b45.hs[idcs[2]].tail,
    )]
}

fn square_triangulate(
    b45: &Boolean45,
    fid: usize,
    eps: f64
) -> Vec<Row3u> {
    let ccw = |tri: Row3u| {
        is_ccw_3d(
            &b45.ps[b45.hs[tri[0]].tail],
            &b45.ps[b45.hs[tri[1]].tail],
            &b45.ps[b45.hs[tri[2]].tail],
            &b45.ns[fid],
            eps
        ) >= 0
    };

    let q = &assemble_halfs(&b45.hs, &b45.initial_hid_per_faces, fid)[0];
    let tris = vec![
        vec![Row3u::new(q[0], q[1], q[2]), Row3u::new(q[0], q[2], q[3])],
        vec![Row3u::new(q[1], q[2], q[3]), Row3u::new(q[0], q[1], q[3])],
    ];
    let mut choice: usize = 0;

    if !(ccw(tris[0][0]) && ccw(tris[0][1])) {
        choice = 1;
    } else if ccw(tris[1][0]) && ccw(tris[1][1]) {
        let diag0 = b45.ps[b45.hs[q[0]].tail] - b45.ps[b45.hs[q[2]].tail];
        let diag1 = b45.ps[b45.hs[q[1]].tail] - b45.ps[b45.hs[q[3]].tail];
        if diag0.norm() > diag1.norm() { choice = 1; }
    }

    tris[choice].iter().map(|t| get_indices(&b45.hs, t)).collect()
}

fn general_triangulate(
    b45: &Boolean45,
    fid: usize,
    eps: f64
) -> Vec<Row3u> {
    let proj  = get_axis_aligned_projection(&b45.ns[fid]);
    let loops = assemble_halfs(&b45.hs, &b45.initial_hid_per_faces, fid);
    let polys = project_polygons(&b45.hs, &b45.ps, &loops, &proj);
    let tris  = EarClip::new(&polys, eps).triangulate();
    tris.iter().map(|t| get_indices(&b45.hs, t)).collect()
}


#[derive(Debug, Clone)]
pub struct PolyVert {
    pub pos: Row2f,
    pub idx: usize
}

// Add the vertex position projection to the indexed polygons.
fn project_polygons(
    hs: &[Half],
    ps: &[Row3f],
    polys: &Vec<Vec<usize>>,
    prj: &Mat23
) -> Vec<Vec<PolyVert>> {
    polys.iter().map(|poly|
        poly.iter().map(|&e| {
            let i = hs[e].tail;
            let p = prj * ps[i].transpose();
            PolyVert { pos: p.transpose(), idx: e }
        }).collect()).collect()
}



