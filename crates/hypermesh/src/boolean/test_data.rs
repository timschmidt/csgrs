use std::sync::Arc;
use nalgebra::DMatrix;
use crate::Hmesh;

pub fn gen_tet_a() -> Arc<Hmesh> {
    let pos = vec![-0.866025, -1., 0.5, 0., -1., -1., 0.866025, -1., 0.5, 0., 1., 0.];
    let idx = vec![0, 3, 1, 0, 1, 2, 1, 3, 2, 2, 3, 0];
    let tails = vec![0, 3, 1, 1, 2, 0, 1, 3, 2, 2, 3, 0];
    let heads = vec![3, 1, 0, 2, 0, 1, 3, 2, 1, 3, 0, 2];
    Hmesh::new_with_halfs(
        DMatrix::from_row_slice(4, 3, &pos),
        DMatrix::from_row_slice(4, 3, &idx),
        tails,
        heads
    )
}

pub fn gen_tet_c() -> Arc<Hmesh> {
    let pos = vec![-2., -0., -1., -2., -0.866025, 0.5, -2., 0.866025, 0.5, 0., 0., 0.];
    let idx = vec![0, 3, 1, 0, 1, 2, 1, 3, 2, 2, 3, 0];
    let tails = vec![0, 3, 1, 1, 2, 0, 1, 3, 2, 2, 3, 0];
    let heads = vec![3, 1, 0, 2, 0, 1, 3, 2, 1, 3, 0, 2];
    Hmesh::new_with_halfs(
        DMatrix::from_row_slice(4, 3, &pos),
        DMatrix::from_row_slice(4, 3, &idx),
        tails,
        heads
    )
}
