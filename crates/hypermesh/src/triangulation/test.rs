use nalgebra::{DMatrix, RowVector2 as Row2};
use crate::{intersect12, winding03, Boolean3, Hmesh, Manifold, OpType};
use crate::common::PolyVert;
use crate::ear_clip::EarClip;

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

#[test]
fn triangulation_test_1() {
    let loop0 = vec![
        PolyVert{pos: Row2::new(0., 0.), idx: 0},
        PolyVert{pos: Row2::new(2., 0.), idx: 1},
        PolyVert{pos: Row2::new(2., 2.), idx: 2},
        PolyVert{pos: Row2::new(0., 2.), idx: 3},
    ];

    let loop1 = vec![
        PolyVert{pos: Row2::new(0.5, 0.5), idx: 0},
        PolyVert{pos: Row2::new(1., 0.), idx: 1},
        PolyVert{pos: Row2::new(1., 1.), idx: 2},
    ];

    let mut ec = EarClip::new(&vec![loop0], 1e-12);
    ec.triangulate();
    for v in ec.polygon.iter() {
        println!("pos : {:?}, idx: {}, idx_l: {}, idx_r: {}",
                 v.borrow().pos, v.borrow().idx, v.borrow().idx_l(), v.borrow().idx_r());
    }

    for v in ec.simples.iter() {
        println!("simples: {}", v.borrow().idx);
    }
}