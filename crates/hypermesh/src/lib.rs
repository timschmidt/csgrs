pub mod manifold;
pub mod triangulation;
pub mod simplification;
pub mod common;
mod tests;
mod boolean03;
mod boolean45;
use crate::boolean03::boolean03;
use crate::boolean45::boolean45;
use crate::common::OpType;
use crate::simplification::simplify_topology;
use crate::triangulation::triangulate;
pub use crate::manifold::*;

pub fn compute_boolean(
    a: &Manifold,
    b: &Manifold,
    op: OpType,
) -> anyhow::Result<Manifold> {
    let eps = a.eps.max(b.eps);
    let tol = a.tol.max(b.tol);

    let     b03 = boolean03(a, b, &op);
    let mut b45 = boolean45(a, b, &b03, &op);
    let mut trg = triangulate(a, b, &b45, eps)?;

    simplify_topology(
        &mut trg.hs,
        &mut b45.ps,
        &mut trg.ns,
        &mut trg.rs,
        b45.nv_from_p + b45.nv_from_q,
        eps
    );

    cleanup_unused_verts(
        &mut b45.ps,
        &mut trg.hs
    );

    Manifold::new(
        &b45.ps.iter().flatten().copied().collect::<Vec<_>>(),
        &trg.hs.iter().map(|h| h.tail).collect::<Vec<_>>(),
        Some(eps),
        Some(tol)
    )
}

