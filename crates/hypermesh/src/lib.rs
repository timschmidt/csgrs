pub mod manifold;
pub mod triangulation;
pub mod simplification;
pub mod common;
mod tests;
mod boolean03;
mod boolean45;
use crate::boolean03::boolean03;
use crate::boolean45::boolean45;
use crate::common::{OpType, Row3u};
use crate::simplification::simplify_topology;
use crate::triangulation::triangulate;
pub use crate::manifold::*;
use crate::sanitize::{compute_halfs, cleanup_unused_verts};

pub fn compute_boolean(
    a: &Manifold,
    b: &Manifold,
    op: OpType,
) -> anyhow::Result<Manifold> {
    let eps = 1e-12; // todo temporally, must calculated from a and b

    let     b03 = boolean03(a, b, &op);
    let mut b45 = boolean45(a, b, &b03, &op);
    let mut trg = triangulate(a, b, &b45, eps)?;
    let mut hs = compute_halfs(&trg.ts);

    simplify_topology(
        &mut hs,
        &mut b45.ps,
        &mut trg.ns,
        &mut trg.rs,
        b45.nv_from_p + b45.nv_from_q,
        eps
    );

    cleanup_unused_verts(&mut b45.ps, &mut hs);

    Manifold::new(
        &b45.ps.iter().flatten().copied().collect::<Vec<_>>(),
        &hs.iter().map(|h| h.tail).collect::<Vec<_>>(),
    )
}

