use nalgebra::RowVector3;
use crate::boolean::{intersect12, winding03, Boolean3};
use crate::{Manifold, OpType};
use crate::test_data::{gen_tet_a, gen_tet_b, gen_tet_c};

#[test]
fn test_tet_sub_inclusion_case(){
    let expand = -1.;
    let hm_p = gen_tet_a();
    let hm_q = gen_tet_c();
    let mfd_p = Manifold::new(&hm_p);
    let mfd_q = Manifold::new(&hm_q);
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

    let boolean = Boolean3{
        mfd_p: &mfd_p, mfd_q: &mfd_q,
        p1q2, p2q1, x12, x21, w03, w30, v12, v21 };

    boolean.get_result(OpType::Subtract);
}

#[test]
fn test_tet_sub_penetration_case(){
    let expand = -1.;
    let hm_p = gen_tet_a();
    let hm_q = gen_tet_b();
    let mfd_p = Manifold::new(&hm_p);
    let mfd_q = Manifold::new(&hm_q);
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

    let boolean = Boolean3{
        mfd_p: &mfd_p, mfd_q: &mfd_q,
        p1q2, p2q1, x12, x21, w03, w30, v12, v21 };

    boolean.get_result(OpType::Subtract);
}
