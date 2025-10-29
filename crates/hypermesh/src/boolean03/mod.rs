pub mod intersect;
pub mod shadow;
pub mod kernel02;
pub mod kernel11;
pub mod kernel12;
use nalgebra::{RowVector3};
use kernel02::Kernel02;
use kernel11::Kernel11;
use kernel12::Kernel12;
use crate::bounds::{BoundingBox, Query}; // todo: fix wired import
use crate::collider::{Recorder}; // todo: same
use crate::common::OpType;
use crate::manifold::Manifold;
type Row3f = RowVector3<f64>;

pub struct Boolean03<'a> {
    pub mfd_p: &'a Manifold,
    pub mfd_q: &'a Manifold,
    pub p1q2: Vec<[i32; 2]>,
    pub p2q1: Vec<[i32; 2]>,
    pub x12: Vec<i32>,
    pub x21: Vec<i32>,
    pub w03: Vec<i32>,
    pub w30: Vec<i32>,
    pub v12: Vec<Row3f>,
    pub v21: Vec<Row3f>,
}

impl <'a> Boolean03<'a> {
    pub fn new(
        mfd_p: &'a Manifold,
        mfd_q: &'a Manifold,
        operation: &OpType,
    ) -> Self {
        let expand = if operation == &OpType::Add { 1. } else { -1. };
        let mut p1q2 = vec![];
        let mut p2q1 = vec![];
        let (x12, v12) = intersect12(mfd_p, mfd_q, &mut p1q2, expand, true);
        let (x21, v21) = intersect12(mfd_p, mfd_q, &mut p2q1, expand, false);
        let w03 = winding03(mfd_p, mfd_q, expand, true);
        let w30 = winding03(mfd_p, mfd_q, expand, false);
        Self { mfd_p, mfd_q, p1q2, p2q1, x12, x21, w03, w30, v12, v21 }
    }
}

struct SimpleRecorder<F> where F: FnMut(usize, usize) {
    callback: F,
}

impl<F> SimpleRecorder<F> where F: FnMut(usize, usize) {
    fn new(callback: F) -> Self {
        Self { callback }
    }
}

impl<'a, F> Recorder for SimpleRecorder<F> where F: FnMut(usize, usize) {
    fn record(&mut self, query_idx: usize, leaf_idx: usize) {
        (self.callback)(query_idx, leaf_idx);
    }
}

struct Intersection12Recorder<'a> {
    pub mfd_a: &'a Manifold,
    pub mfd_b: &'a Manifold,
    pub k12: &'a Kernel12<'a>,
    pub forward: bool,
    pub p1q2: Vec<[i32; 2]>,
    pub x12: Vec<i32>,
    pub v12: Vec<Row3f>,
}

impl <'a> Recorder for Intersection12Recorder<'a> {
    fn record(&mut self, query_idx: usize, leaf_idx: usize) {
        let (x12, op_v12) = self.k12.op(query_idx, leaf_idx);
        if let Some(v12) = op_v12 {
            if self.forward { self.p1q2.push([query_idx as i32, leaf_idx as i32]); }
            else            { self.p1q2.push([leaf_idx as i32, query_idx as i32]); }
            self.x12.push(x12);
            self.v12.push(v12);
        }
    }
}

pub fn intersect12 (
    p: &Manifold,
    q: &Manifold,
    p1q2: &mut Vec<[i32; 2]>,
    expand: f64,
    forward: bool
) -> (Vec<i32>, Vec<Row3f>) {
    let a = if forward { p } else { q };
    let b = if forward { q } else { p };

    let k02 = Kernel02{
        vpos_p: &a.pos,
        vpos_q: &b.pos,
        half_q: &b.hs,
        normal: &p.vert_normals,
        expand,
        forward
    };
    let k11 = Kernel11{
        vpos_p: &p.pos,
        half_p: &p.hs,
        vpos_q: &q.pos,
        half_q: &q.hs,
        normal: &p.vert_normals,
        expand,
    };

    let k12 = Kernel12{
        vpos_p: &a.pos,
        half_p: &a.hs,
        half_q: &b.hs,
        forward,
        k02,
        k11,
    };

    let bbs = a.hs.iter()
        .enumerate()
        .filter(|(_, h)| h.tail < h.head)
        .map(|(i, h)|
            Query::Bb(BoundingBox::new(i, &vec![a.pos[h.tail], a.pos[h.head]]))
            ).collect::<Vec<_>>();

    let mut rec = Intersection12Recorder{
        mfd_a: a,
        mfd_b: b,
        k12: &k12,
        p1q2: vec![],
        x12: vec![],
        v12: vec![],
        forward,
    };

     b.collider.collision(&bbs, &mut rec);

    let mut x12: Vec<i32>   = vec![];
    let mut v12: Vec<Row3f> = vec![];
    let mut seq: Vec<usize> = (0..rec.p1q2.len()).collect();
    seq.sort_by(|&a, &b| (rec.p1q2[a][0], rec.p1q2[a][1]).cmp(&(rec.p1q2[b][0], rec.p1q2[b][1])));

    for i in 0..seq.len() {
        p1q2.push(rec.p1q2[seq[i]]);
        x12.push(rec.x12[seq[i]]);
        v12.push(rec.v12[seq[i]]);
    }

    (x12, v12)
}

pub fn winding03(p: &Manifold, q: &Manifold, expand: f64, forward: bool) -> Vec<i32> {
    let a = if forward {p} else {q};
    let b = if forward {q} else {p};

    let mut w03 = vec![0; a.nv()];
    let k02 = Kernel02 {
        vpos_p: &a.pos,
        vpos_q: &b.pos,
        half_q: &b.hs,
        normal: &p.vert_normals,
        expand,
        forward,
    };

    let bbs = a.pos.iter()
        .enumerate()
        .map(|(i, p)|
        Query::Pt(BoundingBox::new(i, &vec![*p, *p]))
    ).collect::<Vec<_>>();
    let mut rec = SimpleRecorder::new(
        |a, b| {
            let (s02, z02) = k02.op(a, b);
            if z02.is_some() { w03[a] += s02 * if forward {1} else {-1}; }
        }
    );
    b.collider.collision(&bbs, &mut rec);
    w03
}
