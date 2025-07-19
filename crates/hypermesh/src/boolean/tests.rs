use std::sync::Arc;
use nalgebra::DMatrix;
use crate::{Hmesh, Manifold};

pub fn gen_tet_a() -> Arc<Hmesh> {
    let mut pos = DMatrix::zeros(4, 3);
    let mut idx = DMatrix::zeros(4, 3);
    pos.row_mut(0).copy_from_slice(&[0., -1., -1.]);
    pos.row_mut(1).copy_from_slice(&[0.866025, -1., 0.5]);
    pos.row_mut(2).copy_from_slice(&[-0.866025, -1., 0.5]);
    pos.row_mut(3).copy_from_slice(&[0., 1., 0.]);
    idx.row_mut(0).copy_from_slice(&[0, 3, 1]);
    idx.row_mut(1).copy_from_slice(&[0, 1, 2]);
    idx.row_mut(2).copy_from_slice(&[1, 3, 2]);
    idx.row_mut(3).copy_from_slice(&[2, 3, 0]);
    Hmesh::new(pos, idx)
}

pub fn gen_tet_c() -> Arc<Hmesh> {
    let mut pos = DMatrix::zeros(4, 3);
    let mut idx = DMatrix::zeros(4, 3);
    pos.row_mut(0).copy_from_slice(&[-2., -0., -1.]);
    pos.row_mut(1).copy_from_slice(&[-2., -0.866025, 0.5]);
    pos.row_mut(2).copy_from_slice(&[-2., 0.866025, 0.5]);
    pos.row_mut(3).copy_from_slice(&[0., 0., 0.]);
    idx.row_mut(0).copy_from_slice(&[0, 3, 1]);
    idx.row_mut(1).copy_from_slice(&[0, 1, 2]);
    idx.row_mut(2).copy_from_slice(&[1, 3, 2]);
    idx.row_mut(3).copy_from_slice(&[2, 3, 0]);
    Hmesh::new(pos, idx)
}

pub fn gen_mfd_tet_a() -> Manifold {
    let m = gen_tet_a();
    Manifold {
        hmesh: Arc::try_unwrap(m).unwrap(),
        b
    }
}