use hyperreal::Real;

pub(crate) fn exact_normal_positive() -> Real {
    let root_two = Real::from(2).sqrt().unwrap();
    let root_two_over_pi = (root_two.clone() / Real::pi()).unwrap();
    let half = (Real::from(1) / Real::from(2)).unwrap();
    let shared_offset = root_two.clone() * Real::from(3) + half;
    let contact = (((root_two.clone() * Real::from(4) - shared_offset.clone()) * Real::pi())
        * root_two_over_pi.clone()
        / Real::from(4))
    .unwrap();
    let domain = (((root_two * Real::from(2) - shared_offset) * Real::pi()) * root_two_over_pi
        / Real::from(4))
    .unwrap()
        + Real::one();
    contact - domain + Real::from(2).powi_i64(-3000).unwrap()
}
