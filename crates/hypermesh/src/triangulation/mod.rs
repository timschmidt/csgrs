pub mod ear_clip;
pub mod flat_tree;
pub mod tri_halfs;

use std::collections::{BTreeMap, VecDeque};
use crate::boolean45::Boolean45;
use crate::{Manifold, Vec2, Vec3, Vec3u, Half, Tref, get_aa_proj_matrix, compute_aa_proj, is_ccw_3d, Real};
use crate::triangulation::ear_clip::EarClip;
use crate::triangulation::tri_halfs::tri_halfs_single;
#[cfg(feature = "rayon")] use rayon::prelude::*;
#[cfg(feature = "rayon")] use crate::triangulation::tri_halfs::tri_halfs_multi;

pub struct Triangulation {
    pub hs: Vec<Half>,
    pub rs: Vec<Tref>,
    pub ns: Vec<Vec3>,
}

pub fn triangulate(
    mp: &Manifold,
    mq: &Manifold,
    b45: &Boolean45,
    eps: Real,
) -> Result<Triangulation, String> {

    #[cfg(feature = "rayon")] {
        let (mut ts, mut rs, ns) = (0..b45.hid_per_f.len() - 1)
            .into_par_iter()
            .map(|fid| {
                let hid = b45.hid_per_f[fid] as usize;
                let ts_ = process_face(&b45, fid, eps);
                let rs_ = vec![b45.rs[hid].clone(); ts_.len()];
                let ns_ = vec![b45.ns[fid].clone(); ts_.len()];
                (ts_, rs_, ns_)
            })
            .reduce(
                || (vec![], vec![], vec![]),
                |mut acc, (mut ts_, mut rs_, mut ns_)| {
                    acc.0.append(&mut ts_);
                    acc.1.append(&mut rs_);
                    acc.2.append(&mut ns_);
                    acc
                },
            );
        update_reference(mp, mq, &mut rs);
        Ok(Triangulation { hs: tri_halfs_multi(&mut ts), ns, rs })
    }

    #[cfg(not(feature = "rayon"))] {
        let mut ts = vec![];
        let mut ns = vec![];
        let mut rs = vec![];

        for fid in 0..b45.hid_per_f.len() - 1 {
            let hid = b45.hid_per_f[fid] as usize;
            let t = crate::triangulation::process_face(&b45, fid, eps);
            let r = b45.rs[hid].clone();
            let n = b45.ns[fid].clone();
            rs.extend(vec![r; t.len()]);
            ns.extend(vec![n; t.len()]);
            ts.extend(t);
        }
        crate::triangulation::update_reference(mp, mq, &mut rs);
        Ok(Triangulation { hs: crate::triangulation::tri_halfs::tri_halfs_single(&mut ts), ns, rs })
    }

}

fn process_face(
    b45: &Boolean45,
    fid: usize,
    eps: Real
) -> Vec<Vec3u> {
    let e0 = b45.hid_per_f[fid] as usize;
    let e1 = b45.hid_per_f[fid + 1] as usize;
    match e1 - e0 {
        3 =>  single_triangulate(&b45, e0),
        4 =>  square_triangulate(&b45, fid, eps),
        _ => general_triangulate(&b45, fid, eps),
    }
}

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
) -> Vec<Vec3u> {
    let mut idcs = [hid, hid + 1, hid + 2];
    let mut tails = vec![];
    let mut heads = vec![];
    for id in idcs.iter() {
        tails.push(b45.hs[*id].tail);
        heads.push(b45.hs[*id].head);
    }
    if heads[0] == tails[2] { idcs.swap(1, 2); }

    vec![Vec3u::new(
        b45.hs[idcs[0]].tail,
        b45.hs[idcs[1]].tail,
        b45.hs[idcs[2]].tail,
    )]
}

fn square_triangulate(
    b45: &Boolean45,
    fid: usize,
    eps: Real
) -> Vec<Vec3u> {
    let ccw = |tri: Vec3u| {
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
        vec![Vec3u::new(q[0], q[1], q[2]), Vec3u::new(q[0], q[2], q[3])],
        vec![Vec3u::new(q[1], q[2], q[3]), Vec3u::new(q[0], q[1], q[3])],
    ];
    let mut choice: usize = 0;

    if !(ccw(tris[0][0]) && ccw(tris[0][1])) {
        choice = 1;
    } else if ccw(tris[1][0]) && ccw(tris[1][1]) {
        let diag0 = b45.ps[b45.hs[q[0]].tail] - b45.ps[b45.hs[q[2]].tail];
        let diag1 = b45.ps[b45.hs[q[1]].tail] - b45.ps[b45.hs[q[3]].tail];
        if diag0.length() > diag1.length() { choice = 1; }
    }

    tris[choice].iter().map(|t| Vec3u::new(
        b45.hs[t.x].tail,
        b45.hs[t.y].tail,
        b45.hs[t.z].tail
    )).collect()
}

fn general_triangulate(
    b45: &Boolean45,
    fid: usize,
    eps: Real
) -> Vec<Vec3u> {
    let proj  = get_aa_proj_matrix(&b45.ns[fid]);
    let loops = assemble_halfs(&b45.hs, &b45.hid_per_f, fid);
    let polys = loops.iter().map(|poly|
        poly.iter().map(|&e| {
            let i = b45.hs[e].tail;
            let p = compute_aa_proj(&proj, &b45.ps[i]);
            Pt { pos: p, idx: e }
        }).collect()
    ).collect();

    EarClip::new(&polys, eps).triangulate().iter().map(|t| Vec3u::new(
        b45.hs[t.x].tail,
        b45.hs[t.y].tail,
        b45.hs[t.z].tail
    )).collect()
}


#[derive(Debug, Clone)]
pub struct Pt {
    pub pos: Vec2,
    pub idx: usize
}

fn update_reference(
    mp: &Manifold,
    mq: &Manifold,
    rs: &mut[Tref],
) {
    for r in rs.iter_mut() {
        let fid = r.fid;
        let pq  = r.mid == 0;
        r.fid = 0; // see the original code and it's always -1
        r.pid = if pq { mp.coplanar[fid] } else { mq.coplanar[fid] };
    }
}




