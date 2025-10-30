pub mod manifold;
pub mod triangulation;
pub mod simplification;
pub mod common;
mod tests;
mod boolean03;
mod boolean45;

use nalgebra::RowVector3;
use crate::boolean03::Boolean03;
use crate::boolean45::Boolean45;
use crate::common::{OpType, Tref};
pub use crate::manifold::*;
use crate::simplification::simplify_topology;
use crate::triangulation::halfedge::{compute_halfs, reorder_halfedges};
use crate::triangulation::Triangulator;
type Row3f = RowVector3<f64>;
type Row3u = RowVector3<usize>;

pub fn compute_boolean(
    a: &Manifold,
    b: &Manifold,
    op: OpType,
) -> (Vec<Row3f>, Vec<Row3u>) {
    let epsilon = 1e-12; // todo temporally!!!

    let b03 = Boolean03::new(a, b, &op); println!("b03 passed");
    let b45 = Boolean45::new(&a, &b, &b03, &op); println!("b45 passed");
    let trg = Triangulator {
        vpos: &b45.ps,
        fnmls: &b45.ns,
        halfs: &b45.hs,
        trefs: &b45.rs,
        hid_f: &b45.initial_hid_per_faces,
        epsilon
    };

    let (tris, mut ns, mut rs) = trg.triangulate(false).unwrap();

    println!("trg passed");
    (b45.ps, tris)

    /*
    update_reference(&a, &b, &mut rs);

    let mut ps = b45.ps;
    let mut hs = compute_halfs(&tris);

    reorder_halfedges(&mut hs);

    simplify_topology(
        &mut hs,
        &mut ps,
        &mut ns,
        &mut rs,
        b45.nv_from_p + b45.nv_from_q,
        epsilon
    );
    println!("simplify passed");

    let mut tris_out = vec![];

    for tri in hs.chunks_exact(3) {
        let (i0, i1, i2) = (tri[0].tail, tri[1].tail, tri[2].tail);
        if i0 == usize::MAX { continue; }
        tris_out.push(Row3u::new(i0, i1, i2));
    }

    for v in ps.iter_mut() {
        if v.x > 1e10 { *v = Row3f::zeros(); }
    }

    let mut temp = vec![];
    for i in 0..tris.len() {
        if rs[i].mesh_id == 0 && (rs[i].face_id == 2 || rs[i].face_id == 3) {
            temp.push(tris[i]);
        }
    }

    (ps, tris_out)
    */
}

fn update_reference(
    mfd_p: &Manifold,
    mfd_q: &Manifold,
    tref: &mut[Tref],
) {
    for r in tref.iter_mut() {
        let fid = r.face_id;
        let pq = r.mesh_id == 0;
        r.face_id = 0; // todo: see original code and it's always -1
        r.planar_id = if pq { mfd_p.coplanar[fid] }
        else  { mfd_q.coplanar[fid] };
    }
}

