//--- Copyright (C) 2025 Saki Komikado <komietty@gmail.com>,
//--- This Source Code Form is subject to the terms of the Mozilla Public License v.2.0.

pub mod hmesh;
pub mod bounds;
pub mod collider;

use std::cmp::Ordering;
use bounds::BBox;
use crate::collider::{morton_code, MortonCollider, K_NO_CODE};
use crate::{Real, Half, Vec3, Vec3u, K_PRECISION, next_of};
use super::hmesh::Hmesh;
#[cfg(feature = "rayon")] use rayon::prelude::*;

#[derive(Clone)]
pub struct Manifold {
    pub ps: Vec<Vec3>,            // positions
    pub hs: Vec<Half>,            // halfedges
    pub nv: usize,                // number of vertices
    pub nf: usize,                // number of faces
    pub nh: usize,                // number of halfedges
    pub eps: Real,                // epsilon
    pub tol: Real,                // tolerance
    pub bounding_box: BBox,       //
    pub face_normals: Vec<Vec3>,  //
    pub vert_normals: Vec<Vec3>,  //
    pub original_idx: Vec<usize>, //
    pub collider: MortonCollider, //
    pub coplanar: Vec<i32>,       // indices of coplanar faces
}

impl Manifold {
    pub fn new(pos: &[f64], idx: &[usize]) -> Result<Self, String> {
        Self::new_impl(
            pos.chunks(3).map(|p| Vec3::new(p[0] as Real, p[1] as Real, p[2] as Real)).collect(),
            idx.chunks(3).map(|i| Vec3u::new(i[0], i[1], i[2])).collect(),
            None, None
        )
    }

    pub fn new_impl(
        ps : Vec<Vec3>,
        idx: Vec<Vec3u>,
        eps: Option<Real>,
        tol: Option<Real>,
    ) -> Result<Self, String> {
        let bb = BBox::new(None, &ps);
        let (mut f_bb, mut f_mt) = compute_face_morton(&ps, &idx, &bb);
        let hm = sort_faces(&ps, &idx, &mut f_bb, &mut f_mt)?;
        let hs = hm.half.iter().map(|&i| Half::new(hm.tail[i], hm.head[i], hm.twin[i])).collect::<Vec<_>>();

        let mut e = K_PRECISION * bb.scale();
        e = if e.is_finite() { e } else { -1. };
        let eps = if eps.is_some() { eps.unwrap() } else { e };
        let tol = if tol.is_some() { tol.unwrap() } else { e };
        let collider = MortonCollider::new(&f_bb, &f_mt);
        let coplanar = compute_coplanar_idx(&ps, &hm.fns, &hs, eps);

        let mfd = Manifold {
            nv: hm.nv,
            nf: hm.nf,
            nh: hm.nh,
            ps,
            hs,
            bounding_box: bb,
            vert_normals: hm.vns,
            face_normals: hm.fns,
            original_idx: vec![],
            eps,
            tol,
            collider,
            coplanar,
        };

        if !mfd.is_manifold() { return Err("The input mesh is not manifold".into()); }
        Ok(mfd)
    }

    pub fn get_indices(&self) -> Vec<usize> { self.hs.iter().map(|h| h.tail).collect::<Vec<_>>() }

    pub fn set_epsilon(&mut self, min_epsilon: Real, use_single: bool) {
        let scl = self.bounding_box.scale();
        let mut e = min_epsilon.max(K_PRECISION * scl);
        e = if e.is_finite() { e } else { -1. };
        let t = if use_single { e.max(Real::EPSILON * scl) } else { e };
        self.eps = e;
        self.tol = self.tol.max(t);
    }

    pub fn is_manifold(&self) -> bool {
        self.hs.iter().enumerate().all(|(i, h)| {
            if h.tail().is_none() || h.head().is_none() { return true; }
            return match h.pair() {
                None => { false },
                Some(pair) => {
                    let mut good = true;
                    good &= self.hs[pair].pair() == Some(i);
                    good &= h.tail != h.head;
                    good &= h.tail == self.hs[pair].head;
                    good &= h.head == self.hs[pair].tail;
                    good
                }
            }
        })
    }
}

fn compute_face_morton(
    pos: &[Vec3],
    idx: &[Vec3u],
    bb: &BBox
) -> (Vec<BBox>, Vec<u32>) {
    let n = idx.len();
    let mut bbs = vec![BBox::default(); n];
    let mut mts = vec![0; n];

    #[cfg(feature = "rayon")] {
        bbs.par_iter_mut()
            .zip(mts.par_iter_mut())
            .zip(idx.par_iter())
            .for_each(|((bb_, mt_), f)| {
                let p0 = pos[f.x];
                let p1 = pos[f.y];
                let p2 = pos[f.z];
                bb_.union(&p0);
                bb_.union(&p1);
                bb_.union(&p2);
                *mt_ = morton_code(&((p0 + p1 + p2) / 3.), bb);
            });
    }

    #[cfg(not(feature = "rayon"))] {
        for (i, f) in idx.iter().enumerate() {
            let p0 = pos[f.x];
            let p1 = pos[f.y];
            let p2 = pos[f.z];
            bbs[i].union(&p0);
            bbs[i].union(&p1);
            bbs[i].union(&p2);
            mts[i] = morton_code(&((p0 + p1 + p2) / 3.), bb);
        }
    }


    (bbs, mts)
}

fn sort_faces(
    pos: &[Vec3],
    idx: &[Vec3u],
    face_bboxes: &mut Vec<BBox>,
    face_morton: &mut Vec<u32>
) -> Result<Hmesh, String> {
    let mut map = (0..face_morton.len()).collect::<Vec<_>>();
    map.sort_by_key(|&i| face_morton[i]);
    *face_bboxes = map.iter().map(|&i| face_bboxes[i].clone()).collect::<Vec<_>>();
    *face_morton = map.iter().map(|&i| face_morton[i]).collect::<Vec<_>>();

    Hmesh::new(pos, &map.iter().map(|&i| idx[i]).collect::<Vec<_>>())
}

fn compute_coplanar_idx(
    ps: &[Vec3],
    ns: &[Vec3],
    hs: &[Half],
    tol: Real
) -> Vec<i32> {
    let nt = hs.len() / 3;
    let mut priority = vec![];
    let mut res = vec![-1; nt];

    for t in 0..nt {
        let i = t * 3;
        let area = if hs[i].tail().is_none() { 0.} else {
            let p0 = ps[hs[i].tail];
            let p1 = ps[hs[i].head];
            let p2 = ps[hs[i + 1].head];
            (p1 - p0).cross(p2 - p0).length_squared()
        };
        priority.push((area, t));
    }

    priority.sort_by(|a, b| b.0.partial_cmp(&a.0).unwrap_or(Ordering::Equal));

    let mut interior = vec![];
    for (_, t) in priority.iter() {
        if res[*t] != -1 { continue; }
        res[*t] = *t as i32;

        let i = t * 3;
        let p = ps[hs[i].tail];
        let n = ns[*t];

        interior.clear();
        interior.extend_from_slice(&[i, i + 1, i + 2]);

        while let Some(hi) = interior.pop() {
            let h1 = next_of(hs[hi].pair);
            let t1 = h1 / 3;

            if res[t1] != -1 { continue; }

            if (ps[hs[h1].head] - p).dot(n).abs() < tol {
                res[t1] = *t as i32;
                if interior.last().copied() == Some(hs[h1].pair) { interior.pop(); }
                else { interior.push(h1); }
                interior.push(next_of(h1));
            }
        }
    }
    res
}

pub fn cleanup_unused_verts(
    ps: &mut Vec<Vec3>,
    hs: &mut Vec<Half>
) {
    let bb = BBox::new(None, ps);
    let mt = ps.iter().map(|p| morton_code(&p, &bb)).collect::<Vec<_>>();

    let mut new2old = (0..ps.len()).collect::<Vec<_>>();
    let mut old2new = vec![0; ps.len()];
    new2old.sort_by_key(|&i| mt[i]);
    for (new, &old) in new2old.iter().enumerate() { old2new[old] = new; }

    // reindex verts
    for h in hs.iter_mut() {
        if h.pair().is_none() { continue; }
        h.tail = old2new[h.tail];
        h.head = old2new[h.head];
    }

    // truncate pos container
    let nv = new2old
        .iter()
        .position(|&v| mt[v] >= K_NO_CODE)
        .unwrap_or(new2old.len());

    new2old.truncate(nv);

    *ps = new2old.iter().map(|&i| ps[i]).collect();
    *hs = hs.iter().filter(|h| h.pair().is_some()).map(|h| h.clone()).collect();
}

