use crate::{Vec3, Vec2u, Vec3u};
#[cfg(feature = "rayon")] use rayon::prelude::*;

/// Hmesh preserves the order of pos and idx in any cases.
/// Edges are ordered so as the edge is forward (tail idx < head idx)
#[derive(Debug, Clone)]
pub(in crate::manifold) struct Hmesh {
    pub nv: usize,
    pub nf: usize,
    pub nh: usize,
    pub twin: Vec<usize>,
    pub head: Vec<usize>,
    pub tail: Vec<usize>,
    pub half: Vec<usize>,
    pub vns: Vec<Vec3>,
    pub fns: Vec<Vec3>,
}

fn edge_topology(
    pos: &[Vec3],
    idx: &[Vec3u],
    e2v: &mut Vec<Vec2u>,
    e2f: &mut Vec<Vec2u>,
    f2e: &mut Vec<Vec3u>,
) -> Result<(), String> {
    if pos.len() == 0 { return Err("empty pos matrix".into()); }
    if idx.len() == 0 { return Err("empty idx matrix".into()); }

    let mut ett: Vec<[usize; 4]> = vec![];

    for i in 0..idx.len() {
    for j in 0..3 {
        let mut v1 = idx[i][j];
        let mut v2 = idx[i][(j + 1) % 3];
        if v1 > v2 { std::mem::swap(&mut v1, &mut v2); }
        ett.push([v1, v2, i, j]);
    }}
    ett.sort();

    let mut ne = 1;
    for i in 0..ett.len() - 1 {
        if !(ett[i][0] == ett[i + 1][0] && ett[i][1] == ett[i + 1][1]) { ne += 1; }
    }

    e2v.resize(ne, Vec2u::MAX);
    e2f.resize(ne, Vec2u::MAX);
    f2e.resize(idx.len(), Vec3u::MAX);
    ne = 0;

    let mut i = 0;
    while i < ett.len() {
        if i == ett.len() - 1 || !((ett[i][0] == ett[i+1][0]) && (ett[i][1] == ett[i + 1][1])) {
            // Border edge
            let [v1, v2, i, j] = ett[i];
            e2v[ne][0] = v1;
            e2v[ne][1] = v2;
            e2f[ne][0] = i;
            f2e[i][j]  = ne;
        } else {
            let r1 = ett[i];
            let r2 = ett[i + 1];
            e2v[ne][0] = r1[0];
            e2v[ne][1] = r1[1];
            e2f[ne][0] = r1[2];
            e2f[ne][1] = r2[2];
            f2e[r1[2]][r1[3]] = ne;
            f2e[r2[2]][r2[3]] = ne;
            i += 1; // skip the next one
        }
        ne += 1;
        i += 1;
    }

    for i in 0..e2f.len() {
        let fid = e2f[i][0];
        let mut flip = true;
        for j in 0..3 {
            if idx[fid][j] == e2v[i][0] && idx[fid][(j + 1) % 3] == e2v[i][1] {
                flip = false;
            }
        }

        if flip {
            let tmp = e2f[i][0];
            e2f[i][0] = e2f[i][1];
            e2f[i][1] = tmp;
        }
    }
    Ok(())
}

impl Hmesh {
    pub fn new(
        pos: &[Vec3],
        idx: &[Vec3u],
    ) -> Result<Self, String> {
        let mut e2v = Default::default();
        let mut e2f = Default::default();
        let mut f2e = Default::default();
        edge_topology(pos, idx, &mut e2v, &mut e2f, &mut f2e)?;

        let nv = pos.len();
        let nf = idx.len();
        let ne = e2v.len();
        let nh = e2v.len() * 2;
        let np = 3;
        let mut v2h  = vec![usize::MAX; nv];
        let mut e2h  = vec![usize::MAX; ne];
        let mut f2h  = vec![usize::MAX; nf];
        let mut next = vec![usize::MAX; nh];
        let mut prev = vec![usize::MAX; nh];
        let mut twin = vec![usize::MAX; nh];
        let mut head = vec![usize::MAX; nh];
        let mut tail = vec![usize::MAX; nh];
        let mut edge = vec![usize::MAX; nh];
        let mut face = vec![usize::MAX; nh];

        for it in 0..nf {
        for ip in 0..np {
            let ih_bgn = it * np;
            let iv = idx[it][ip];
            let ie = f2e[it][ip];
            let ih = ih_bgn + ip;
            next[ih] = ih_bgn + (ip + 1) % np;
            prev[ih] = ih_bgn + (ip + np - 1) % np;
            head[ih] = idx[it][(ip + 1) % np];
            tail[ih] = iv;
            edge[ih] = ie;
            face[ih] = it;
            if f2h[it] == usize::MAX { f2h[it] = ih; }
            if v2h[iv] == usize::MAX { v2h[iv] = ih; }
            if e2h[ie] == usize::MAX { e2h[ie] = ih; }
            else {
                twin[ih] = e2h[ie];
                twin[e2h[ie]] = ih;
            }
        }}

        if twin.iter().any(|v| v == &usize::MAX) {
            return Err("Input mesh must not contain boundary edges.".into());
        }

        let mut half = vec![];
        for i in 0..nh { half.push(i); }
        let mut vns = vec![Vec3::ZERO; nv];
        let mut fns = vec![Vec3::ZERO; nf];
        let mut fas = vec![0.; nf];

        #[cfg(feature = "rayon")]
        fns.par_iter_mut()
            .zip(fas.par_iter_mut())
            .enumerate()
            .for_each(|(i, (fn_, fa_))| {
                let ih = f2h[i];
                let p2 = pos[head[ih]];
                let p1 = pos[tail[ih]];
                let p0 = pos[tail[prev[ih]]];
                let x = p2 - p1;
                let t = (p1 - p0) * -1.;
                let n = x.cross(t);
                *fn_ = n.normalize();
                *fa_ = n.length() * 0.5;
            });

        #[cfg(not(feature = "rayon"))]
        for i in 0..nf {
            let ih = f2h[i];
            let p2 = pos[head[ih]];
            let p1 = pos[tail[ih]];
            let p0 = pos[tail[prev[ih]]];
            let x = p2 - p1;
            let t = (p1 - p0) * -1.;
            let n = x.cross(t);
            fns[i] = n.normalize();
            fas[i] = n.length() * 0.5;
        }


        for i in 0..nf {
        for j in 0..3  {
            let vi = idx[i][j];
            vns[vi] += fns[i] * fas[i];
        }}


        #[cfg(feature = "rayon")]
        vns.par_iter_mut().for_each(|n| *n = n.normalize_or_zero());

        #[cfg(not(feature = "rayon"))]
        for n in &mut vns { *n = n.normalize_or_zero(); }

        Ok(Hmesh{ nv, nf, nh, twin, head, tail, half, vns, fns })
    }
}
