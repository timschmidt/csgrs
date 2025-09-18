use nalgebra::DMatrix;
use crate::{intersect12, winding03, Boolean3, Hmesh, Manifold, OpType};

#[test]
fn test() {
    let (m0, _) = tobj::load_obj("assets/models/cube_x_plus.obj", &tobj::LoadOptions { ..Default::default() }).expect("failed");
    let (m1, _) = tobj::load_obj("assets/models/tet_b.obj", &tobj::LoadOptions { ..Default::default() }).expect("failed");

    let mut hms_ = vec![];
    for (m, s) in vec![(m0, 1.), (m1, 1.)] {
        let mesh = &m[0].mesh;
        let pos_buf = mesh.positions.iter().map(|&v| (v * s) as f64).collect::<Vec<f64>>();
        let idx_buf = mesh.indices.iter().map(|&v| v as usize).collect::<Vec<usize>>();
        let hm = Hmesh::new(
            DMatrix::from_row_slice(mesh.positions.len() / 3, 3, &pos_buf).into(),
            DMatrix::from_row_slice(mesh.indices.len() / 3, 3, &idx_buf).into()
        );
        hms_.push(hm);
    }

    let mf0 = Manifold::new(&hms_[0]);
    let mf1 = Manifold::new(&hms_[1]);
    let mfs = vec![&mf0, &mf1];

    let expand = -1.;
    let mut p1q2 = vec![];
    let mut p2q1 = vec![];
    let (x12, v12) = intersect12(mfs[0], mfs[1], &mut p1q2, expand, true);
    let (x21, v21) = intersect12(mfs[0], mfs[1], &mut p2q1, expand, false);
    let w03 = winding03(mfs[0], mfs[1], expand, true);
    let w30 = winding03(mfs[0], mfs[1], expand, false);

    let boolean = Boolean3{
        mfd_p: &mfs[0], mfd_q: &mfs[1],
        p1q2, p2q1, x12, x21, w03, w30, v12, v21 };
    let (pos, halfs, tris) = boolean.get_result(OpType::Subtract);
}