use std::sync::Arc;
use nalgebra::DMatrix;
use crate::Hmesh;

pub fn gen_tri_a() -> Arc<Hmesh> {
    let pos = vec![0., 0., 0., 1., 0., 0., 0., 1., 0.];
    let idx = vec![0, 1, 2];
    let tails = vec![0, 1, 2, 1, 2, 0];
    let heads = vec![1, 2, 0, 0, 1, 2];
    let f_nor = vec![0., 0., 1.];
    let v_nor = vec![0., 0., 1., 0., 0., 1., 0., 0., 1.];
    Hmesh::new_for_boolean_test(
        DMatrix::from_row_slice(3, 3, &pos),
        DMatrix::from_row_slice(1, 3, &idx),
        tails,
        heads,
        DMatrix::from_row_slice(3, 3, &v_nor),
        DMatrix::from_row_slice(1, 3, &f_nor),
    )
}


pub fn gen_tri_c() -> Arc<Hmesh> {
    let pos = vec![0., -0.5, 0., 1., -0.5, 0., 0., 1., 1.];
    let idx = vec![0, 1, 2];
    let tails = vec![0, 1, 2, 1, 2, 0];
    let heads = vec![1, 2, 0, 0, 1, 2];
    let f_nor = vec![0., 0., 1.];
    let v_nor = vec![0., 0., 1., 0., 0., 1., 0., 0., 1.];
    Hmesh::new_for_boolean_test(
        DMatrix::from_row_slice(3, 3, &pos),
        DMatrix::from_row_slice(1, 3, &idx),
        tails,
        heads,
        DMatrix::from_row_slice(3, 3, &v_nor),
        DMatrix::from_row_slice(1, 3, &f_nor),
    )
}

pub fn gen_tet_a() -> Arc<Hmesh> {
    let pos = vec![-0.866025, -1., 0.5, 0., -1., -1., 0.866025, -1., 0.5, 0., 1., 0.];
    let idx = vec![0, 3, 1, 1, 2, 0, 1, 3, 2, 2, 3, 0];
    let tails = vec![0, 3, 1, 1, 2, 0, 1, 3, 2, 2, 3, 0];
    let heads = vec![3, 1, 0, 2, 0, 1, 3, 2, 1, 3, 0, 2];
    let f_nor = vec![-0.840168, 0.242536, -0.485071, 0., -1., 0., 0.840168, 0.242536, -0.485071, 0., 0.242536, 0.970143];
    let v_nor = vec![-0.798417, -0.387351, 0.460966, 0., -0.387351, -0.921932, 0.798417, -0.387351, 0.460966, 0., 1., 0.];
    Hmesh::new_for_boolean_test(
        DMatrix::from_row_slice(4, 3, &pos),
        DMatrix::from_row_slice(4, 3, &idx),
        tails,
        heads,
        DMatrix::from_row_slice(4, 3, &v_nor),
        DMatrix::from_row_slice(4, 3, &f_nor),
    )
}

pub fn gen_tet_b() -> Arc<Hmesh> {
    let pos = vec![-1., -0.866025, 0.5, -1., 0., -1., -1., 0.866025, 0.5, 1., 0., 0.];
    let idx = vec![1, 3, 0, 1, 0, 2, 2, 3, 1, 0, 3, 2];
    let tails = vec![1, 3, 0, 1, 0, 2, 2, 3, 1, 0, 3, 2];
    let heads = vec![3, 0, 1, 0, 2, 1, 3, 1, 2, 3, 2, 0];
    let f_nor = vec![0.242536, -0.840168, -0.485071, -1., 0., 0., 0.242536, 0.840168, -0.485071, 0.242536, 0., 0.970143];
    let v_nor = vec![-0.387351, -0.798417, 0.460966, -0.387351, 0., -0.921932, -0.387351, 0.798417, 0.460966, 1., 0., 0.];
    Hmesh::new_for_boolean_test(
        DMatrix::from_row_slice(4, 3, &pos),
        DMatrix::from_row_slice(4, 3, &idx),
        tails,
        heads,
        DMatrix::from_row_slice(4, 3, &v_nor),
        DMatrix::from_row_slice(4, 3, &f_nor),
    )
}

pub fn gen_tet_c() -> Arc<Hmesh> {
    let pos = vec![-2., -0.866025, 0.5, -2., -0., -1., -2., 0.866025, 0.5, 0., 0., 0.];
    let idx = vec![1, 3, 0, 1, 0, 2, 2, 3, 1, 0, 3, 2];
    let tails = vec![1, 3, 0, 1, 0, 2, 2, 3, 1, 0, 3, 2];
    let heads = vec![3, 0, 1, 0, 2, 1, 3, 1, 2, 3, 2, 0];
    let f_nor = vec![0.242536, -0.840168, -0.485071, -1., 0., 0., 0.242536, 0.840168, -0.485071, 0.242536, 0., 0.970143];
    let v_nor = vec![-0.387351, -0.798417, 0.460966, -0.387351, 0., -0.921932, -0.387351, 0.798417, 0.460966, 1., 0., 0.];
    Hmesh::new_for_boolean_test(
        DMatrix::from_row_slice(4, 3, &pos),
        DMatrix::from_row_slice(4, 3, &idx),
        tails,
        heads,
        DMatrix::from_row_slice(4, 3, &v_nor),
        DMatrix::from_row_slice(4, 3, &f_nor),
    )
}
