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

pub fn compute_boolean(
    mp: &Manifold,
    mq: &Manifold,
    op: OpType,
) -> anyhow::Result<Manifold> {
    let eps = mp.eps.max(mq.eps);
    let tol = mp.tol.max(mq.tol);

    let now = std::time::Instant::now();

    let     b03 = boolean03(mp, mq, &op);

    println!("b03: {:?}", now.elapsed());
    let now = std::time::Instant::now();

    let mut b45 = boolean45(mp, mq, &b03, &op);

    println!("b45: {:?}", now.elapsed());
    let now = std::time::Instant::now();

    let mut trg = triangulate(mp, mq, &b45, eps)?;

    println!("trg: {:?}", now.elapsed());
    let now = std::time::Instant::now();

    simplify_topology(
        &mut trg.hs,
        &mut b45.ps,
        &mut trg.ns,
        &mut trg.rs,
        b45.nv_from_p,
        b45.nv_from_q,
        eps
    );

    println!("smp: {:?}", now.elapsed());
    let now = std::time::Instant::now();

    cleanup_unused_verts(
        &mut b45.ps,
        &mut trg.hs
    );

    println!("cln: {:?}", now.elapsed());

    Manifold::new_impl(
        &b45.ps,
        &trg.hs
            .chunks(3)
            .map(|hs| Row3u::new(hs[0].tail, hs[1].tail, hs[2].tail))
            .collect(),
        Some(eps),
        Some(tol)
    )
}





