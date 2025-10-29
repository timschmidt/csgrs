use std::collections::BTreeMap;
use std::sync::Arc;
use nalgebra::{DMatrix, RowVector3};
use crate::hmesh::Hmesh;
use crate::{Manifold, OpType};
use crate::boolean03::{intersect12, winding03, Boolean03};
use crate::boolean45::Boolean45;

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
        None,
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
        None,
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
        None,
        DMatrix::from_row_slice(4, 3, &v_nor),
        DMatrix::from_row_slice(4, 3, &f_nor),
    )
}


#[cfg(test)]
mod kernel11_tests {
    use nalgebra::RowVector4;
    use crate::boolean03::kernel11::Kernel11;
    use crate::Manifold;
    use crate::tests::{gen_tet_a, gen_tet_c};

    #[test]
    fn kernel11_test() {
        let mfd_p = Manifold::new(&gen_tet_a()).unwrap();
        let mfd_q = Manifold::new(&gen_tet_c()).unwrap();
        let k11 = Kernel11 {
            vpos_p: &mfd_p.pos,
            vpos_q: &mfd_q.pos,
            half_p: &mfd_p.hs,
            half_q: &mfd_q.hs,
            normal: &mfd_p.vert_normals,
            expand: 1.,
        };
        let (s, z) = k11.op(0, 9);
        assert_eq!(s, 0);
        assert!(z.is_some() && (z.unwrap() - RowVector4::new(-0.532938,-0.230769,0.307692,0.133235)).norm() < 1e-6);
    }
}


#[test]
fn test_tet_sub_inclusion_case(){
    let expand = -1.;
    let hm_p = gen_tet_a();
    let hm_q = gen_tet_c();
    let mfd_p = Manifold::new(&hm_p).unwrap();
    let mfd_q = Manifold::new(&hm_q).unwrap();
    let mut p1q2 = vec![];
    let mut p2q1 = vec![];
    let (x12, v12) = intersect12(&mfd_p, &mfd_q, &mut p1q2, expand, true);
    let (x21, v21) = intersect12(&mfd_p, &mfd_q, &mut p2q1, expand, false);
    let w03 = winding03(&mfd_p, &mfd_q, expand, true);
    let w30 = winding03(&mfd_p, &mfd_q, expand, false);

    assert_eq!(w03, vec![0, 0, 0, 0]);
    assert_eq!(w30, vec![0, 0, 0, 1]);
    assert_eq!(x12.len(), 0);
    assert_eq!(v12.len(), 0);
    assert_eq!(x21, vec![-1, -1, -1]);
    let v21_ = vec![
        RowVector3::new(-0.224009, 0., -0.112005),
        RowVector3::new(-0.294367, 0.127465, 0.0735918),
        RowVector3::new(-0.395087, -0.171077, 0.0987716),
    ];
    for i in 0..3 {
        assert!((v21[i] - v21_[i]).norm() < 1e-6);
    }
    let op = OpType::Subtract;
    let b03 = Boolean03{ mfd_p: &mfd_p, mfd_q: &mfd_q, p1q2, p2q1, x12, x21, w03, w30, v12, v21 };
    let b45 = Boolean45::new(&b03, &op);
}

#[test]
fn test_tet_sub_penetration_case(){
    let expand = -1.;
    let hm_p = gen_tet_a();
    let hm_q = gen_tet_b();
    let mfd_p = Manifold::new(&hm_p).unwrap();
    let mfd_q = Manifold::new(&hm_q).unwrap();
    let mut p1q2 = vec![];
    let mut p2q1 = vec![];
    let (x12, v12) = intersect12(&mfd_p, &mfd_q, &mut p1q2, -1., true);
    let (x21, v21) = intersect12(&mfd_p, &mfd_q, &mut p2q1, -1., false);
    let w03 = winding03(&mfd_p, &mfd_q, expand, true);
    let w30 = winding03(&mfd_p, &mfd_q, expand, false);

    let v12_ = vec![
        RowVector3::new(-0.763707, -0.763707, 0.440927),
        RowVector3::new(-0.242656, 0.439609, 0.140098),
        RowVector3::new(0.,0.,-0.5),
        RowVector3::new(0.,0.,-0.5)
    ];
    let v21_ = vec![
        RowVector3::new(0.302169,0.302169,0.174458),
        RowVector3::new(0.439609,-0.242656,0.140098),
        RowVector3::new(0.302169,0.302169,0.174458),
        RowVector3::new(-0.763707,-0.763707,0.440927)
    ];

    assert_eq!(w03, vec![0, 0, 0, 0]);
    assert_eq!(w30, vec![0, 0, 0, 0]);
    assert_eq!(x12, vec![-1, 1, -1, 1]);
    assert_eq!(x21, vec![1, 1, -1, -1]);
    for i in 0..3 {
        assert!((v12[i] - v12_[i]).norm() < 1e-6);
        assert!((v21[i] - v21_[i]).norm() < 1e-6);
    }

    let op = OpType::Subtract;
    let b03 = Boolean03{ mfd_p: &mfd_p, mfd_q: &mfd_q, p1q2, p2q1, x12, x21, w03, w30, v12, v21 };
    let b45 = Boolean45::new(&b03, &op);
}

#[cfg(test)]
mod collider_test {
    /*
    use nalgebra::RowVector3;
    use crate::collider::{, MortonCollider};
    use crate::{intersect12, Manifold};
    use crate::bounds::BoundingBox;

    #[test]
    fn morton_code_test() {
        let v = spread_bits_3(341.333 as u32);
        assert_eq!(v, 17043521);
        let v = spread_bits_3(1023);
        assert_eq!(v, 153391689);
        let v = spread_bits_3(682.667 as u32);
        assert_eq!(v, 136348168);
    }

    #[test]
    fn morton_radix_tree_test() {
        let expand = -1.;
        let hm_p = test_data::gen_tet_a();
        let mfd_p = Manifold::new(&hm_p);
        println!("===================");
        let hm_q = test_data::gen_tet_c();
        let mfd_q = Manifold::new(&hm_q);

        let mut p1q2 = vec![];
        let mut p2q1 = vec![];
        let (x12, v12) = intersect12(&mfd_p, &mfd_q, &mut p1q2, expand, true);
        let (x21, v21) = intersect12(&mfd_p, &mfd_q, &mut p2q1, expand, false);
        //println!("x21: {:?}", x21);
        //println!("v21: {:?}", v21);
    }

    #[test]
    fn morton_radix_tree_test_0() {
        let mut leaf_bb = vec![];
        let mut leaf_mt = vec![];

        leaf_bb.push(BoundingBox::new(usize::MAX, &vec![RowVector3::new(-0.8,-1.,-1.), RowVector3::new(1.2,-1.,1.)]));
        leaf_bb.push(BoundingBox::new(usize::MAX, &vec![RowVector3::new(-0.8,-1.,-1.), RowVector3::new(1.2,1.,-1.)]));
        leaf_bb.push(BoundingBox::new(usize::MAX, &vec![RowVector3::new(-0.8,-1.,-1.), RowVector3::new(-0.8,1.,1.)]));
        leaf_bb.push(BoundingBox::new(usize::MAX, &vec![RowVector3::new(-0.8,-1.,-1.), RowVector3::new(-0.8,1.,1.)]));
        leaf_bb.push(BoundingBox::new(usize::MAX, &vec![RowVector3::new(-0.8,-1.,1. ), RowVector3::new(1.2,1.,1.)]));
        leaf_bb.push(BoundingBox::new(usize::MAX, &vec![RowVector3::new(-0.8,1.,-1. ), RowVector3::new(1.2,1.,1.)]));
        leaf_bb.push(BoundingBox::new(usize::MAX, &vec![RowVector3::new(1.2,-1.,-1. ), RowVector3::new(1.2,1.,1.)]));
        leaf_bb.push(BoundingBox::new(usize::MAX, &vec![RowVector3::new(-0.8,-1.,-1.), RowVector3::new(1.2,-1.,1.)]));
        leaf_bb.push(BoundingBox::new(usize::MAX, &vec![RowVector3::new(-0.8,-1.,1. ), RowVector3::new(1.2,1.,1.)]));
        leaf_bb.push(BoundingBox::new(usize::MAX, &vec![RowVector3::new(-0.8,-1.,-1.), RowVector3::new(1.2,1.,-1.)]));
        leaf_bb.push(BoundingBox::new(usize::MAX, &vec![RowVector3::new(-0.8,1.,-1. ), RowVector3::new(1.2,1.,1.)]));
        leaf_bb.push(BoundingBox::new(usize::MAX, &vec![RowVector3::new(1.2,-1.,-1. ), RowVector3::new(1.2,1.,1.)]));
        leaf_mt.push(85217605);
        leaf_mt.push(102261126);
        leaf_mt.push(170435210);
        leaf_mt.push(289739857);
        leaf_mt.push(494262109);
        leaf_mt.push(511305630);
        leaf_mt.push(664697319);
        leaf_mt.push(681740840);
        leaf_mt.push(732871403);
        leaf_mt.push(818089008);
        leaf_mt.push(869219571);
        leaf_mt.push(1022611260);

        let table = vec![11, 0, 8, 1, 10, 4, 5, 3, 6, 2, 7, 9];

        let mut leaf_bb_alt = leaf_bb.clone();
        let mut leaf_mt_alt = leaf_mt.clone();
        for i in 0..table.len() {
            leaf_bb_alt[i] = leaf_bb[table[i]].clone();
            leaf_mt_alt[i] = leaf_mt[table[i]].clone();
            //leaf_bb_alt[i] = leaf_bb[(i + 1) % leaf_bb.len()].clone();
            //leaf_mt_alt[i] = leaf_mt[(i + 1) % leaf_bb.len()].clone();
        }
        //for i in 1..leaf_mt.len() {
        //    assert!(leaf_mt_alt[i - 1] <= leaf_mt_alt[i]);
        //}

        let col = MortonCollider::new(leaf_bb_alt.as_slice(), leaf_mt_alt.as_slice());
    }
    */
}

#[cfg(test)]
mod test_simplification {
    use crate::simplification::edge_collapse::collapse_collinear_edges;

    #[test]
    fn test_collapse() {
        use crate::common::{Half, Tref};
        use nalgebra::{RowVector3 as Row3};

        let mut hs = vec![
            Half::new(0, 4, 5),
            Half::new(4, 1, 42),
            Half::new(1, 0, 9),
            Half::new(0, 2, 26),
            Half::new(2, 4, 50),
            Half::new(4, 0, 0),
            Half::new(0, 11, 11),
            Half::new(11, 9, 57),
            Half::new(9, 0, 21),
            Half::new(0, 1, 2),
            Half::new(1, 11, 14),
            Half::new(11, 0, 6),
            Half::new(1, 10, 17),
            Half::new(10, 11, 68),
            Half::new(11, 1, 10),
            Half::new(1, 3, 35),
            Half::new(3, 10, 20),
            Half::new(10, 1, 12),
            Half::new(3, 13, 31),
            Half::new(13, 10, 66),
            Half::new(10, 3, 16),
            Half::new(0, 9, 8),
            Half::new(9, 12, 59),
            Half::new(12, 0, 24),
            Half::new(0, 12, 23),
            Half::new(12, 2, 27),
            Half::new(2, 0, 3),
            Half::new(2, 12, 25),
            Half::new(12, 13, 64),
            Half::new(13, 2, 30),
            Half::new(2, 13, 29),
            Half::new(13, 3, 18),
            Half::new(3, 2, 36),
            Half::new(1, 7, 47),
            Half::new(7, 3, 37),
            Half::new(3, 1, 15),
            Half::new(2, 3, 32),
            Half::new(3, 7, 34),
            Half::new(7, 2, 51),
            Half::new(4, 6, 49),
            Half::new(6, 5, 54),
            Half::new(5, 4, 43),
            Half::new(1, 4, 1),
            Half::new(4, 5, 41),
            Half::new(5, 1, 45),
            Half::new(1, 5, 44),
            Half::new(5, 7, 56),
            Half::new(7, 1, 33),
            Half::new(2, 6, 53),
            Half::new(6, 4, 39),
            Half::new(4, 2, 4),
            Half::new(2, 7, 38),
            Half::new(7, 6, 55),
            Half::new(6, 2, 48),
            Half::new(5, 6, 40),
            Half::new(6, 7, 52),
            Half::new(7, 5, 46),
            Half::new(9, 11, 7),
            Half::new(11, 12, 61),
            Half::new(12, 9, 22),
            Half::new(8, 12, 65),
            Half::new(12, 11, 58),
            Half::new(11, 8, 69),
            Half::new(8, 13, 71),
            Half::new(13, 12, 28),
            Half::new(12, 8, 60),
            Half::new(10, 13, 19),
            Half::new(13, 11, 70),
            Half::new(11, 10, 13),
            Half::new(8, 11, 62),
            Half::new(11, 13, 67),
            Half::new(13, 8, 63),
        ];

        let mut ps = vec![
            Row3::new(-0.8, -1., -1.),
            Row3::new(-0.8, -1., 1.),
            Row3::new(-0.8, 1., -1.),
            Row3::new(-0.8, 1., 1.),
            Row3::new(1.2, -1., -1.),
            Row3::new(1.2, -1., 1.),
            Row3::new(1.2, 1., -1.),
            Row3::new(1.2, 1., 1.),
            Row3::new(0., 0., 0.),
            Row3::new(-0.8, -0.14641, -0.14641),
            Row3::new(-0.8, 0.2, 0.2),
            Row3::new(-0.8, -0.34641, 0.2),
            Row3::new(-0.8, 0., -0.4),
            Row3::new(-0.8, 0.34641, 0.2),
        ];

        let mut ns = vec![
            Row3::new(-0., -1., -0.),
            Row3::new(-0., -0., -1.),
            Row3::new(-1., -0., 0.),
            Row3::new(-1., -0., 0.),
            Row3::new(-1., -0., 0.),
            Row3::new(-1., -0., 0.),
            Row3::new(-1., -0., -0.),
            Row3::new(-1., -0., -0.),
            Row3::new(-1., -0., -0.),
            Row3::new(-1., -0., -0.),
            Row3::new(-1., -0., -0.),
            Row3::new(0., 0., 1.),
            Row3::new(0., 1., 0.),
            Row3::new(1., 0., 0.),
            Row3::new(0., -1., 0.),
            Row3::new(-0., 0., 1.),
            Row3::new(0., 0., -1.),
            Row3::new(-0., 1., 0.),
            Row3::new(1., 0., -0.),
            Row3::new(-0.242536, 0.840168, 0.485071),
            Row3::new(-0.242536, 0.840168, 0.485071),
            Row3::new(-0.242536, -0.840168, 0.485071),
            Row3::new(-0.242536, -3.22877e-20, -0.970143),
            Row3::new(-0.242536, -3.22877e-20, -0.970143),
        ];

        let mut refs = vec![
            Tref{mesh_id: 1, face_id: 0, origin_id: -2, planar_id: 3},
            Tref{mesh_id: 1, face_id: 0, origin_id: -1, planar_id: 5},
            Tref{mesh_id: 1, face_id: 0, origin_id: -1, planar_id: 2},
            Tref{mesh_id: 1, face_id: 0, origin_id: -1, planar_id: 2},
            Tref{mesh_id: 1, face_id: 0, origin_id: -1, planar_id: 2},
            Tref{mesh_id: 1, face_id: 0, origin_id: -1, planar_id: 2},
            Tref{mesh_id: 1, face_id: 0, origin_id: -1, planar_id: 2},
            Tref{mesh_id: 1, face_id: 0, origin_id: -1, planar_id: 2},
            Tref{mesh_id: 1, face_id: 0, origin_id: -1, planar_id: 2},
            Tref{mesh_id: 1, face_id: 0, origin_id: -1, planar_id: 2},
            Tref{mesh_id: 1, face_id: 0, origin_id: -1, planar_id: 2},
            Tref{mesh_id: 1, face_id: 0, origin_id: -1, planar_id: 1},
            Tref{mesh_id: 1, face_id: 0, origin_id: -1, planar_id: 0},
            Tref{mesh_id: 1, face_id: 0, origin_id: -1, planar_id: 4},
            Tref{mesh_id: 1, face_id: 0, origin_id: -1, planar_id: 3},
            Tref{mesh_id: 1, face_id: 0, origin_id: -1, planar_id: 1},
            Tref{mesh_id: 1, face_id: 0, origin_id: -1, planar_id: 5},
            Tref{mesh_id: 1, face_id: 0, origin_id: -1, planar_id: 0},
            Tref{mesh_id: 1, face_id: 0, origin_id: -1, planar_id: 4},
            Tref{mesh_id: 2, face_id: 0, origin_id: -1, planar_id: 0},
            Tref{mesh_id: 2, face_id: 0, origin_id: -1, planar_id: 0},
            Tref{mesh_id: 2, face_id: 0, origin_id: -1, planar_id: 3},
            Tref{mesh_id: 2, face_id: 0, origin_id: -1, planar_id: 2},
            Tref{mesh_id: 2, face_id: 0, origin_id: -1, planar_id: 2},
        ];

        collapse_collinear_edges(
            &mut hs,
            &mut ps,
            &mut ns,
            &mut refs,
            9,
            1e-6
        );

        let hs_out = vec![
            (0, 4, 5),
            (4, 1, 42),
            (1, 0, 9),
            (0, 2, 26),
            (2, 4, 50),
            (4, 0, 0),
            (0, 11, 11),
            (11, 12, 61),
            (12, 0, 24),
            (0, 1, 2),
            (1, 11, 17),
            (11, 0, 6),
            (-1, -1, -1),
            (-1, -1, -1),
            (-1, -1, -1),
            (1, 3, 35),
            (3, 11, 20),
            (11, 1, 10),
            (3, 13, 31),
            (13, 11, 70),
            (11, 3, 16),
            (-1, -1, -1),
            (-1, -1, -1),
            (-1, -1, -1),
            (0, 12, 8),
            (12, 2, 27),
            (2, 0, 3),
            (2, 12, 25),
            (12, 13, 64),
            (13, 2, 30),
            (2, 13, 29),
            (13, 3, 18),
            (3, 2, 36),
            (1, 7, 47),
            (7, 3, 37),
            (3, 1, 15),
            (2, 3, 32),
            (3, 7, 34),
            (7, 2, 51),
            (4, 6, 49),
            (6, 5, 54),
            (5, 4, 43),
            (1, 4, 1),
            (4, 5, 41),
            (5, 1, 45),
            (1, 5, 44),
            (5, 7, 56),
            (7, 1, 33),
            (2, 6, 53),
            (6, 4, 39),
            (4, 2, 4),
            (2, 7, 38),
            (7, 6, 55),
            (6, 2, 48),
            (5, 6, 40),
            (6, 7, 52),
            (7, 5, 46),
            (-1, -1, -1),
            (-1, -1, -1),
            (-1, -1, -1),
            (8, 12, 65),
            (12, 11, 7),
            (11, 8, 69),
            (8, 13, 71),
            (13, 12, 28),
            (12, 8, 60),
            (-1, -1, -1),
            (-1, -1, -1),
            (-1, -1, -1),
            (8, 11, 62),
            (11, 13, 19),
            (13, 8, 63),
        ];

        let ps_out = vec![
            Row3::new(-0.8, -1., -1.),
            Row3::new(-0.8, -1., 1.),
            Row3::new(-0.8, 1., -1.),
            Row3::new(-0.8, 1., 1.),
            Row3::new(1.2, -1., -1.),
            Row3::new(1.2, -1., 1.),
            Row3::new(1.2, 1., -1.),
            Row3::new(1.2, 1., 1.),
            Row3::new(0., 0., 0.),
            Row3::new(f64::MAX, f64::MAX, f64::MAX),
            Row3::new(f64::MAX, f64::MAX, f64::MAX),
            Row3::new(-0.8, -0.34641, 0.2),
            Row3::new(-0.8, 0., -0.4),
            Row3::new(-0.8, 0.34641, 0.2),
        ];

        for i in 0..hs.len() {
            let h = &hs[i];
            let a = hs[i].tail;
            let b = hs[i].head;
            let c = hs[i].pair;
            let (d, e, f) = hs_out[i];
            if h.tail().is_none() { assert_eq!(d, -1); } else { assert_eq!(a as i32, d); }
            if h.head().is_none() { assert_eq!(e, -1); } else { assert_eq!(b as i32, e); }
            if h.pair().is_none() { assert_eq!(f, -1); } else { assert_eq!(c as i32, f); }
        }

        for i in 0..ps.len() {
            let p0 = &ps[i];
            let p1 = &ps_out[i];
            if p0.x == f64::MAX && p0.y == f64::MAX && p0.z == f64::MAX { assert_eq!(p1.x, f64::MAX); }
            else { assert!((p0 - p1).norm() < 1e-6); }
        }
    }
}