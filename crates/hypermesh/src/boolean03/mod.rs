pub mod kernel01;
pub mod kernel02;
pub mod kernel11;
pub mod kernel12;
pub mod kernel03;
use crate::boolean03::kernel03::winding03;
use crate::boolean03::kernel12::intersect12;
use crate::common::{OpType, Row3f};
use crate::manifold::Manifold;

pub struct Boolean03 {
    pub p1q2: Vec<[usize; 2]>,
    pub p2q1: Vec<[usize; 2]>,
    pub x12: Vec<i32>,
    pub x21: Vec<i32>,
    pub w03: Vec<i32>,
    pub w30: Vec<i32>,
    pub v12: Vec<Row3f>,
    pub v21: Vec<Row3f>,
}

pub fn boolean03(
    mp: &Manifold,
    mq: &Manifold,
    operation: &OpType,
) -> Boolean03 {
    let expand = if operation == &OpType::Add { 1. } else { -1. };
    let mut p1q2 = vec![];
    let mut p2q1 = vec![];
    //let (x12, v12) = intersect12(mp, mq, &mut p1q2, expand, true);
    //let (x21, v21) = intersect12(mp, mq, &mut p2q1, expand, false);
    //let w03 = winding03(mp, mq, expand, true);
    //let w30 = winding03(mp, mq, expand, false);

    let ((x12, v12, w03), (x21, v21, w30)) = rayon::join(
        || {
            let i = intersect12(mp, mq, &mut p1q2, expand, true);
            let w = winding03(mp, mq, expand, true);
            (i.0, i.1, w)
        },
        || {
            let i = intersect12(mp, mq, &mut p2q1, expand, false);
            let w = winding03(mp, mq, expand, false);
            (i.0, i.1, w)
        },
    );
    Boolean03 { p1q2, p2q1, x12, x21, w03, w30, v12, v21 }
}

