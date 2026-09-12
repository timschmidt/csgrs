use hyperreal::Real;

// Exactly 2^-3000, retained as a symbolic geometric expression. Tests require
// the intended value and domain behavior, not a particular layer at which its
// proof becomes available: scalar normal forms may improve independently of
// the solver's strict predicate fallback.
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
        + Real::from(1);
    contact - domain + Real::from(2).powi_i64(-3000).unwrap()
}

pub(crate) fn terminal_zero() -> Real {
    let sine = Real::e().sin();
    let cosine = Real::e().cos();
    &sine * &sine + &cosine * &cosine - Real::one()
}
