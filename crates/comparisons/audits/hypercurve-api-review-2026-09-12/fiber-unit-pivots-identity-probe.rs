use hyperreal::Real;
use hyperlimit::{compare_reals, PredicatePolicy};
use hypersolve::{AlgebraicRootRepresentation, BivariatePolynomial, IsolatedRootInterval,
    subresultant_in_algebraic_fiber, sign_at_selected_root, validate_algebraic_root_representation};
fn main() {
    let half=(Real::one()/Real::from(2)).unwrap();
    let a=half.sqrt().unwrap();
    let n=(Real::one()+Real::from(4)*&a).sqrt().unwrap();
    let inv=(Real::one()/n).unwrap();
    let c=Real::one()-Real::from(2)*&inv;
    let d=&a+&inv;
    let mut root=AlgebraicRootRepresentation::from_exact_value(&a);
    root.interval=IsolatedRootInterval{lower:Real::zero(),upper:Real::one(),exact_root:None,distinct_root_count:1};
    root.validation=validate_algebraic_root_representation(&root,PredicatePolicy::STRICT);
    assert!(root.is_valid());
    let p=|values:Vec<Real>| BivariatePolynomial::new(vec![values]);
    let fiber=BivariatePolynomial::new(vec![vec![Real::zero(),Real::zero(),Real::one()],vec![-Real::one()]]);
    let h=[p(vec![Real::zero(),c.clone()]),p(vec![Real::from(2)*&d-Real::one()]),p(vec![Real::zero()]),p(vec![Real::from(-2)])];
    let k=[p(vec![&d*&d-Real::one(),Real::zero(),&c*&c]),p(vec![Real::zero(),Real::from(-2)*&c]),p(vec![Real::one()-Real::from(2)*&d]),p(vec![Real::zero()]),p(vec![Real::one()])];
    let start=std::time::Instant::now();
    let resultant=subresultant_in_algebraic_fiber(&h,&k,0,&fiber,&root).unwrap();
    println!("construction_ms={}",start.elapsed().as_secs_f64()*1000.);
    let [resultant]=resultant.as_slice() else {panic!("one resultant")};
    println!("coefficient_lengths={:?}",resultant.coefficients.iter().map(Vec::len).collect::<Vec<_>>());
    assert!(resultant.coefficients.iter().all(|row|row.len()==1));
    let coefficients=resultant.coefficients.iter().map(|row|row[0].clone()).collect::<Vec<_>>();
    println!("coefficient_signs={:?}",coefficients.iter().map(|v| compare_reals(v,&Real::zero(),PredicatePolicy::STRICT).value()).collect::<Vec<_>>());
    println!("selected_sign={:?}",sign_at_selected_root(&root.polynomial_coefficients,&coefficients,&root.interval));
    let value=Real::eval_poly(&coefficients,&a);
    println!("witness_sign={:?}",compare_reals(&value,&Real::zero(),PredicatePolicy::STRICT).value());
    println!("witness_rational={:?}",value.exact_rational_normal_form());
    let factored=(Real::from(4)*&a+Real::one())*&inv*&inv-Real::one();
    println!("factored_sign={:?}",compare_reals(&factored,&Real::zero(),PredicatePolicy::STRICT).value());
}
