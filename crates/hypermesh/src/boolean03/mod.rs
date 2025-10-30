pub mod op01;
pub mod kernel02;
pub mod kernel11;
pub mod kernel12;
pub mod op03;
pub mod op12;
use crate::boolean03::op03::winding03;
use crate::boolean03::op12::intersect12;
use crate::common::OpType;
use crate::manifold::Manifold;
type Row3f = nalgebra::RowVector3<f64>;

pub struct Boolean03 {
    pub p1q2: Vec<[i32; 2]>,
    pub p2q1: Vec<[i32; 2]>,
    pub x12: Vec<i32>,
    pub x21: Vec<i32>,
    pub w03: Vec<i32>,
    pub w30: Vec<i32>,
    pub v12: Vec<Row3f>,
    pub v21: Vec<Row3f>,
}

impl Boolean03 {
    pub fn new(
        mfd_p: &Manifold,
        mfd_q: &Manifold,
        operation: &OpType,
    ) -> Self {
        let expand = if operation == &OpType::Add { 1. } else { -1. };
        let mut p1q2 = vec![];
        let mut p2q1 = vec![];
        let (x12, v12) = intersect12(mfd_p, mfd_q, &mut p1q2, expand, true);
        let (x21, v21) = intersect12(mfd_p, mfd_q, &mut p2q1, expand, false);
        let w03 = winding03(mfd_p, mfd_q, expand, true);
        let w30 = winding03(mfd_p, mfd_q, expand, false);
        Self { p1q2, p2q1, x12, x21, w03, w30, v12, v21 }
    }
}

