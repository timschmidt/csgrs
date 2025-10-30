use nalgebra::{RowVector3};
use std::mem;
use crate::bounds::{BBox, Query};
use crate::collider::Recorder;
use crate::common::Half;
use crate::Manifold;
use super::kernel01::intersect;
use super::kernel02::Kernel02;
use super::kernel11::Kernel11;
type Row3f = RowVector3<f64>;

pub struct Kernel12<'a> {
    pub hs_p: &'a[Half],
    pub hs_q: &'a[Half],
    pub ps_p: &'a[Row3f],
    pub k02: Kernel02<'a>,
    pub k11: Kernel11<'a>,
    pub fwd: bool,
}

impl<'a> Kernel12<'a> {
    pub fn op (&self, p1: usize, q2: usize) -> (i32, Option<Row3f>) {
        let mut x12 = 0;
        let mut xzy_lr0 = [Row3f::zeros(); 2];
        let mut xzy_lr1 = [Row3f::zeros(); 2];
        let mut shadows = false;
        let h = self.hs_p[p1].clone();

        let mut k = 0;

        for vid in [h.tail, h.head].iter() {
            let (s, op_z) = self.k02.op(*vid, q2);
            if let Some(z) = op_z {
                let f = (*vid == h.tail) == self.fwd;
                x12 += s * if f { 1 } else { -1 };
                if k < 2 && (k == 0 || (s != 0) != shadows) {
                    shadows = s != 0;
                    xzy_lr0[k] = self.ps_p[*vid];
                    // Swap y and z
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
            let half = &self.hs_q[q1];
            let q1f = if half.is_forward() { q1 } else { half.pair };

            let (s, op_xyzz) = if self.fwd { self.k11.op(p1, q1f) } else { self.k11.op(q1f, p1) };

            if let Some(xyzz) = op_xyzz {
                x12 -= s * if half.is_forward() { 1 } else { -1 };
                if k < 2 && (k == 0 || (s != 0) != shadows) {
                    shadows = s != 0;
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

        if x12 == 0 { return (0, None); } // No intersection

        assert_eq!(k, 2, "Boolean manifold error: v12");
        let xzyy = intersect(xzy_lr0[0], xzy_lr0[1], xzy_lr1[0], xzy_lr1[1]);
        (x12, Some(Row3f::new(xzyy[0], xzyy[2], xzyy[1])))
    }
}

pub fn intersect12 (
    mp: &Manifold,
    mq: &Manifold,
    p1q2: &mut Vec<[i32; 2]>,
    expand: f64,
    fwd: bool
) -> (Vec<i32>, Vec<Row3f>) {
    let a = if fwd { mp } else { mq };
    let b = if fwd { mq } else { mp };

    let k02 = Kernel02{
        ps_p: &a.ps,
        ps_q: &b.ps,
        hs_q: &b.hs,
        ns: &mp.vns,
        expand,
        fwd
    };
    let k11 = Kernel11{
        ps_p: &mp.ps,
        ps_q: &mq.ps,
        hs_p: &mp.hs,
        hs_q: &mq.hs,
        ns: &mp.vns,
        expand,
    };

    let k12 = Kernel12{
        ps_p: &a.ps,
        hs_p: &a.hs,
        hs_q: &b.hs,
        fwd,
        k02,
        k11,
    };

    let bbs = a.hs.iter()
        .enumerate()
        .filter(|(_, h)| h.tail < h.head)
        .map(|(i, h)| Query::Bb(BBox::new(i, &vec![a.ps[h.tail], a.ps[h.head]])))
        .collect::<Vec<_>>();

    let mut rec = Intersection12Recorder{ k12: &k12, x12: vec![], v12: vec![], p1q2: vec![], fwd };
    b.collider.collision(&bbs, &mut rec);

    let mut x12 = vec![];
    let mut v12 = vec![];
    let mut seq = (0..rec.p1q2.len()).collect::<Vec<_>>();
    seq.sort_by(|&a, &b| (rec.p1q2[a][0], rec.p1q2[a][1]).cmp(&(rec.p1q2[b][0], rec.p1q2[b][1])));

    for i in 0..seq.len() {
        p1q2.push(rec.p1q2[seq[i]]);
        x12.push(rec.x12[seq[i]]);
        v12.push(rec.v12[seq[i]]);
    }

    (x12, v12)
}

struct Intersection12Recorder<'a> {
    pub k12: &'a Kernel12<'a>,
    pub fwd: bool,
    pub x12: Vec<i32>,
    pub v12: Vec<Row3f>,
    pub p1q2: Vec<[i32; 2]>,
}

impl <'a> Recorder for Intersection12Recorder<'a> {
    fn record(&mut self, query_idx: usize, leaf_idx: usize) {
        let (x12, op_v12) = self.k12.op(query_idx, leaf_idx);
        if let Some(v12) = op_v12 {
            if self.fwd { self.p1q2.push([query_idx as i32, leaf_idx as i32]); }
            else        { self.p1q2.push([leaf_idx as i32, query_idx as i32]); }
            self.x12.push(x12);
            self.v12.push(v12);
        }
    }
}

