pub mod manifold;
pub mod triangulation;
pub mod simplification;
pub mod common;
mod tests;
mod boolean03;
mod boolean45;
use crate::boolean03::boolean03;
use crate::boolean45::boolean45;
use crate::common::{OpType, Row3f, Row3u};
use crate::simplification::simplify_topology;
use crate::triangulation::triangulate;
pub use crate::manifold::*;
use crate::sanitize::{compute_halfs, sanitize_unused_verts};

pub fn compute_boolean(
    a: &Manifold,
    b: &Manifold,
    op: OpType,
) -> (Vec<Row3f>, Vec<Row3u>) {
    let eps = 1e-12; // todo temporally, must calculated from a and b

    let     b03 = boolean03(a, b, &op);
    let mut b45 = boolean45(a, b, &b03, &op);
    let mut trg = triangulate(a, b, &b45, eps).unwrap();
    let mut hs = compute_halfs(&trg.ts);

    simplify_topology(
        &mut hs,
        &mut b45.ps,
        &mut trg.ns,
        &mut trg.rs,
        b45.nv_from_p + b45.nv_from_q,
        eps
    );

    sanitize_unused_verts(&mut b45.ps, &mut hs);

    let mut ts = vec![];
    for t in hs.chunks_exact(3) {
        if t[0].pair().is_none() { continue; }
        ts.push(Row3u::new(t[0].tail, t[1].tail, t[2].tail));
    }

    (b45.ps, ts)
}

