pub mod ear_clip;
pub mod flat_tree;
use anyhow::Result;
use std::collections::{BTreeMap, VecDeque};
use crate::boolean45::Boolean45;
use crate::common::{Half, Tref, get_axis_aligned_projection, is_ccw_3d, Row3f, Row3u, Row2f, next_of};
use crate::Manifold;
use crate::triangulation::ear_clip::EarClip;

pub struct Triangulation {
    pub hs: Vec<Half>,
    pub rs: Vec<Tref>,
    pub ns: Vec<Row3f>,
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
    for fid in 0..b45.hid_per_f.len() - 1 {
        let hid = b45.hid_per_f[fid] as usize;
        let t = process_face(&b45, fid, eps);
        let r = b45.rs[hid].clone();
        let n = b45.ns[fid].clone();
        rs.extend(vec![r; t.len()]);
        ns.extend(vec![n; t.len()]);
        ts.extend(t);
    }
    update_reference(mp, mq, &mut rs);
    Ok(Triangulation { hs: compute_halfs(&mut ts), ns, rs })
}

fn process_face(
    b45: &Boolean45,
    fid: usize,
    eps: f64
) -> Vec<Row3u> {
    let e0 = b45.hid_per_f[fid] as usize;
    let e1 = b45.hid_per_f[fid + 1] as usize;
    match e1 - e0 {
        3 =>  single_triangulate(&b45, e0),
        4 =>  square_triangulate(&b45, fid, eps),
        _ => general_triangulate(&b45, fid, eps),
    }
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

    let q = &assemble_halfs(&b45.hs, &b45.hid_per_f, fid)[0];
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

    tris[choice].iter().map(|t| Row3u::new(
        b45.hs[t.x].tail,
        b45.hs[t.y].tail,
        b45.hs[t.z].tail
    )).collect()
}

fn general_triangulate(
    b45: &Boolean45,
    fid: usize,
    eps: f64
) -> Vec<Row3u> {
    let proj  = get_axis_aligned_projection(&b45.ns[fid]);
    let loops = assemble_halfs(&b45.hs, &b45.hid_per_f, fid);
    let polys = loops.iter().map(|poly|
        poly.iter().map(|&e| {
            let i = b45.hs[e].tail;
            let p = (proj * b45.ps[i].transpose()).transpose();
            Pt { pos: p, idx: e }
        }).collect()
    ).collect();

    EarClip::new(&polys, eps).triangulate().iter().map(|t| Row3u::new(
        b45.hs[t.x].tail,
        b45.hs[t.y].tail,
        b45.hs[t.z].tail
    )).collect()
}


#[derive(Debug, Clone)]
pub struct Pt {
    pub pos: Row2f,
    pub idx: usize
}

fn update_reference(
    mp: &Manifold,
    mq: &Manifold,
    rs: &mut[Tref],
) {
    for r in rs.iter_mut() {
        let fid = r.fid;
        let pq = r.mid == 0;
        r.fid = 0; // see the original code and it's always -1
        r.pid = if pq { mp.coplanar[fid] }
        else  { mq.coplanar[fid] };
    }
}

fn compute_halfs(ts: &Vec<Row3u>) -> Vec<Half> {
    let nh = ts.len() * 3;
    let ne = nh / 2;
    let nt = nh / 3;
    let remove_flag = usize::MAX - 1;
    let mut hs  = vec![Half::default(); nh];
    let mut ids = (0..nh).collect::<Vec<_>>();
    let mut key = vec![0u64; nh];

    for t in 0..ts.len() {
        for i in 0..3 {
            let j = (i + 1) % 3;
            let e = t * 3 + i;
            let i0 = ts[t][i];
            let i1 = ts[t][j];
            hs[e].tail = i0;
            hs[e].head = i1;
            let a = std::cmp::min(i0, i1) as u64;
            let b = std::cmp::max(i0, i1) as u64;
            let f = if i0 < i1 { 1u64 } else { 0u64 } << 63;
            key[e] = f | (a << 32) | b;
        }
    }

    ids.sort_by_key(|&i| key[i]);

    // By sorting forward and backward halfedges by key,
    // now halfedges of the same mini ids are sorted in a sequence.
    // It treats the triangle overlap case here, also considers 4-manifold case.
    let mut step = |i: usize, consecutive_ini: usize| -> usize {
        let i0 = ids[i];
        let h0 = hs[i0].clone();
        let j = i + ne;
        let mut k = consecutive_ini + ne;
        loop {
            if k >= nh { break; }
            let i1 = ids[k];
            let h1 = hs[i1].clone();

            if !(h0.tail == h1.head && h0.head == h1.tail) { break; }
            if hs[next_of(i0)].head == hs[next_of(i1)].head { // overlap
                hs[i0].pair = remove_flag;
                hs[i1].pair = remove_flag;
                if k != j { ids.swap(j, k); }
                break;
            }
            k += 1;
        }

        if i + 1 == ne { return consecutive_ini; }
        let i2 = ids[i + 1];
        let h2 = hs[i2].clone();
        if h0.tail == h2.tail && h0.head == h2.head { consecutive_ini } else { i + 1 }
    };

    let mut ini = 0;
    for i in 0..ne { ini = step(i, ini); }

    for i in 0..ne {
        let i0 = ids[i];
        let i1 = ids[i + ne];
        if hs[i0].pair != remove_flag {
            hs[i0].pair = i1;
            hs[i1].pair = i0;
        } else {
            hs[i0] = Half::default();
            hs[i1] = Half::default();
        }
    }

    // reorder halfedges: step 1
    for t in 0..nt {
        let i = t * 3;
        let f = [hs[i].clone(), hs[i + 1].clone(), hs[i + 2].clone(), ];
        let mut mini = 0;
        if f[1].tail < f[mini].tail { mini = 1; }
        if f[2].tail < f[mini].tail { mini = 2; }
        for j in 0..3 { hs[i + j] = f[(mini + j) % 3].clone(); }
    }

    // reorder halfedges: step 2
    for t in 0..nt {
        for i in t * 3..(t + 1) * 3 {
            let tail = hs[i].tail;
            let pair = hs[i].pair;
            if pair == remove_flag || pair >= hs.len() { continue; }
            let j = (pair / 3) * 3;
            let f = (0..3).find(|&k| hs[j + k].head == tail);
            if let Some(k) = f { hs[i].pair = j + k; }
        }
    }
    hs
}



