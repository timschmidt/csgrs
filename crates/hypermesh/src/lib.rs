pub mod manifold;
pub mod triangulation;
pub mod simplification;
pub mod common;
pub mod boolean03;
pub mod boolean45;
mod tests;
use crate::boolean03::boolean03;
use crate::boolean45::boolean45;
use crate::simplification::simplify_topology;
use crate::triangulation::triangulate;
pub use crate::manifold::*;
pub use crate::common::*;

pub fn compute_boolean_from_raw_data(
    pos0: Vec<Real>,
    idx0: Vec<usize>,
    pos1: Vec<Real>,
    idx1: Vec<usize>,
    op_type: usize
) -> Result<Manifold, String>{
    let mp = Manifold::new(&pos0, &idx0)?;
    let mq = Manifold::new(&pos1, &idx1)?;
    let op = match op_type {
        0 => OpType::Add,
        1 => OpType::Subtract,
        2 => OpType::Intersect,
        _ => return Err("Invalid op_type".into())
    };

    compute_boolean(&mp, &mq, op)
}

pub fn compute_boolean(
    mp: &Manifold,
    mq: &Manifold,
    op: OpType,
) -> Result<Manifold, String> {
    let eps = mp.eps.max(mq.eps);
    let tol = mp.tol.max(mq.tol);

    let     b03 = boolean03(mp, mq, &op);
    let mut b45 = boolean45(mp, mq, &b03, &op);
    let mut trg = triangulate(mp, mq, &b45, eps)?;

    simplify_topology(
        &mut trg.hs,
        &mut b45.ps,
        &mut trg.ns,
        &mut trg.rs,
        b45.nv_from_p,
        b45.nv_from_q,
        eps
    );

    cleanup_unused_verts(
        &mut b45.ps,
        &mut trg.hs
    );

    Manifold::new_impl(
        &b45.ps,
        &trg.hs
            .chunks(3)
            .map(|hs| Vec3u::new(hs[0].tail, hs[1].tail, hs[2].tail))
            .collect(),
        Some(eps),
        Some(tol)
    )
}





