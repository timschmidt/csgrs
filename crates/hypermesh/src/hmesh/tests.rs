use nalgebra::{DMatrix};
use crate::Hmesh;

#[test]
fn minimal_quad_case() {
    let mut pos = DMatrix::zeros(4, 3);
    let mut idx = DMatrix::zeros(2, 3);
    pos.row_mut(0).copy_from_slice(&[0., 0., 0.]);
    pos.row_mut(1).copy_from_slice(&[1., 0., 0.]);
    pos.row_mut(2).copy_from_slice(&[1., 1., 0.]);
    pos.row_mut(3).copy_from_slice(&[0., 1., 0.]);
    idx.row_mut(0).copy_from_slice(&[0, 1, 2]);
    idx.row_mut(1).copy_from_slice(&[2, 3, 0]);
    let hmesh = Hmesh::new(pos, idx);
    assert_eq!(hmesh.n_vert, 4);
    assert_eq!(hmesh.n_face, 2);
    assert_eq!(hmesh.n_edge, 5);
    assert_eq!(hmesh.n_half, 10);

    for h in hmesh.halfs.iter() {
        let p1 = h.tail().pos();
        let p2 = h.head().pos();
        println!("id: {}, next: {}. prev: {}, twin: {}, tail: {:?}, head: {:?}, boundary: {}", 
                 h.id, h.next().id, h.prev().id, h.twin().id, p1, p2, h.is_boundary());
    }
    
}

#[test]
fn quad_with_hole_case() {
    let (models, _) = tobj::load_obj(
        "assets/models/quad_with_hole.obj",
        &tobj::LoadOptions { ..Default::default() },
    ).expect("Failed to OBJ load file");
    let model = &models[0];
    let mesh = &model.mesh;

    let pos_buf: Vec<f64>   = mesh.positions.iter().map(|&v| v as f64).collect();
    let idx_buf: Vec<usize> = mesh.indices.iter().map(|&v| v as usize).collect();

    let pos: DMatrix<f64>   = DMatrix::from_row_slice(mesh.positions.len() / 3, 3, &pos_buf).into();
    let idx: DMatrix<usize> = DMatrix::from_row_slice(mesh.indices.len() / 3, 3, &idx_buf).into();
    let hmesh = Hmesh::new(pos, idx);
    assert_eq!(hmesh.n_vert, 16);
    assert_eq!(hmesh.n_face, 16);
    assert_eq!(hmesh.n_edge, 32);
    assert_eq!(hmesh.n_vert + hmesh.n_face - hmesh.n_edge, 0);

    for h in hmesh.halfs.iter() {
        println!("id: {}, next: {}. prev: {}, twin: {}, boundary: {}",
                 h.id, h.next().id, h.prev().id, h.twin().id, h.is_boundary());
    }
}