pub mod manifold;
pub mod triangulation;
pub mod simplification;
pub mod common;
mod tests;
mod boolean03;
mod boolean45;
mod sanitization;

use nalgebra::RowVector3;
use crate::boolean03::boolean03;
use crate::boolean45::boolean45;
use crate::common::{OpType, Row3f, Row3u};
use crate::simplification::simplify_topology;
use crate::sanitization::{compute_halfs, update_reference};
use crate::triangulation::triangulate;
pub use crate::manifold::*;

pub fn compute_boolean(
    a: &Manifold,
    b: &Manifold,
    op: OpType,
) -> (Vec<Row3f>, Vec<Row3u>) {
    let epsilon = 1e-12; // todo temporally, must calculated from a and b

    let     b03 = boolean03(a, b, &op);
    let mut b45 = boolean45(&a, &b, &b03, &op);
    let mut tri = triangulate(&b45, epsilon).unwrap();

    update_reference(&a, &b, &mut tri.rs);
    let mut hs = compute_halfs(&tri.ts);

    simplify_topology(
        &mut hs,
        &mut b45.ps,
        &mut tri.ns,
        &mut tri.rs,
        b45.nv_from_p + b45.nv_from_q,
        epsilon
    );

    let mut tris_out = vec![];

    for tri in hs.chunks_exact(3) {
        let (i0, i1, i2) = (tri[0].tail, tri[1].tail, tri[2].tail);
        if i0 == usize::MAX { continue; }
        tris_out.push(Row3u::new(i0, i1, i2));
    }

    for v in b45.ps.iter_mut() {
        if v.x > 1e10 { *v = Row3f::zeros(); }
    }

    let mut temp = vec![];
    for i in 0..tri.ts.len() {
        if tri.rs[i].mesh_id == 0 && (tri.rs[i].face_id == 2 || tri.rs[i].face_id == 3) {
            temp.push(tri.ts[i]);
        }
    }

    (b45.ps, tris_out)
}

