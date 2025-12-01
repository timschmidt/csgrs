use std::mem;
use rayon::prelude::*;
use crate::bounds::{BBox, Query};
use crate::common::{Half, Row3f};
use crate::Manifold;
use super::kernel01::intersect;
use super::kernel02::Kernel02;
use super::kernel11::Kernel11;

pub struct Kernel12<'a> {
    pub hs_p: &'a[Half],
    pub hs_q: &'a[Half],
    pub ps_p: &'a[Row3f],
    pub k02: Kernel02<'a>,
    pub k11: Kernel11<'a>,
    pub fwd: bool,
}

impl<'a> Kernel12<'a> {
    pub fn op (&self, p1: usize, q2: usize) -> Option<(i32, Row3f)> {
        let mut x12 = 0;
        let mut xzy_lr0 = [Row3f::zeros(); 2];
        let mut xzy_lr1 = [Row3f::zeros(); 2];
        let mut shadow_ = false;
        let h = self.hs_p[p1].clone();

        let mut k = 0;

        for vid in [h.tail, h.head].iter() {
            if let Some((s, z)) = self.k02.op(*vid, q2) {
                let f = (*vid == h.tail) == self.fwd;
                x12 += s * if f { 1 } else { -1 };
                if k < 2 && (k == 0 || (s != 0) != shadow_) {
                    shadow_ = s != 0;
                    xzy_lr0[k] = self.ps_p[*vid];
                    let temp = xzy_lr0[k].y;
                    xzy_lr0[k].y = xzy_lr0[k].z;
                    xzy_lr0[k].z = temp;
                    xzy_lr1[k] = xzy_lr0[k].clone();
                    xzy_lr1[k].y = z;
                    k += 1;
                }
            }
        }

        for i in 0..3 {
            let q1 = 3 * q2 + i;
            let h = &self.hs_q[q1];
            let q1f = if h.is_forward() { q1 } else { h.pair };
            let op = if self.fwd { self.k11.op(p1, q1f) } else { self.k11.op(q1f, p1) };
            if let Some((s, xyzz)) = op {
                x12 -= s * if h.is_forward() { 1 } else { -1 };
                if k < 2 && (k == 0 || (s != 0) != shadow_) {
                    shadow_ = s != 0;
                    xzy_lr0[k].x = xyzz.x;
                    xzy_lr0[k].y = xyzz.z;
                    xzy_lr0[k].z = xyzz.y;
                    xzy_lr1[k] = xzy_lr0[k];
                    xzy_lr1[k].y = xyzz.w;
                    if !self.fwd { mem::swap(&mut xzy_lr0[k].y, &mut xzy_lr1[k].y); }
                    k += 1;
                }
            }
        }

        if x12 == 0 { return None }

        assert_eq!(k, 2, "Boolean manifold error: v12");
        let xzyy = intersect(xzy_lr0[0], xzy_lr0[1], xzy_lr1[0], xzy_lr1[1]);
        Some((x12, Row3f::new(xzyy[0], xzyy[2], xzyy[1])))
    }
}

pub fn intersect12 (
    mp: &Manifold,
    mq: &Manifold,
    p1q2: &mut Vec<[usize; 2]>,
    expand: f64,
    fwd: bool
) -> (Vec<i32>, Vec<Row3f>) {
    let ma = if fwd { mp } else { mq };
    let mb = if fwd { mq } else { mp };

    let k02 = Kernel02{ ps_p: &ma.ps, ps_q: &mb.ps, hs_q: &mb.hs, ns: &mp.vert_normals, expand, fwd };
    let k11 = Kernel11{ ps_p: &mp.ps, ps_q: &mq.ps, hs_p: &mp.hs, hs_q: &mq.hs, ns: &mp.vert_normals, expand };
    let k12 = Kernel12{ ps_p: &ma.ps, hs_p: &ma.hs, hs_q: &mb.hs, fwd, k02, k11 };

    let bbs = ma.hs.iter()
        .enumerate()
        .filter(|(_, h)| h.is_forward())
        .map(|(i, h)| Query::Bb(BBox::new(Some(i), &vec![ma.ps[h.tail], ma.ps[h.head]])))
        .collect::<Vec<Query>>();

    let mut x12_  = vec![];
    let mut v12_  = vec![];
    let mut p1q2_ = vec![];
    let mut x12 = vec![];
    let mut v12 = vec![];
    let mut rec = |a, b| {
            if let Some((x, v)) = k12.op(a, b) {
                if fwd { p1q2_.push([a, b]); }
                else   { p1q2_.push([b, a]); }
                x12_.push(x);
                v12_.push(v);
            }
        };

    mb.collider.collision(&bbs, &mut rec);

    let mut seq = (0..p1q2_.len()).collect::<Vec<_>>();
    seq.sort_by(|&a, &b| (p1q2_[a][0], p1q2_[a][1]).cmp(&(p1q2_[b][0], p1q2_[b][1])));
    for i in 0..seq.len() {
        p1q2.push(p1q2_[seq[i]]);
        x12.push(x12_[seq[i]]);
        v12.push(v12_[seq[i]]);
    }

    (x12, v12)
}