use hyperreal::Real;
use hypersolve::{AlgebraicField, AlgebraicFieldError, AlgebraicRootRepresentation as Root, IsolatedRootInterval, PredicatePolicy, validate_algebraic_root_representation};
fn r(n: i32) -> Real { Real::from(n) }
fn q(n: i32, d: i32) -> Real { (r(n)/r(d)).unwrap() }
fn selected(coefficients: Vec<Real>, lower: Real, upper: Real) -> Root {
    let mut root = Root::from_exact_value(&r(0));
    root.polynomial_coefficients = coefficients;
    root.interval = IsolatedRootInterval { lower, upper, exact_root:None, distinct_root_count:1 };
    root.validation = validate_algebraic_root_representation(&root,PredicatePolicy::STRICT);
    assert!(root.is_valid()); root
}
fn main() {
    // (beta^2-2)(beta-3); the selected beta is positive sqrt(2).
    let beta = selected(vec![r(6),r(-2),r(-3),r(1)],r(1),r(2));
    let mut foreign = AlgebraicField::new(&beta).unwrap();
    let value = foreign.rational_function(&beta,vec![r(0),r(1)],vec![r(1)]).unwrap();
    // Proving this finite inverse removes the foreign factor beta-3.
    let inverse = foreign.rational_function(&beta,vec![r(1)],vec![r(-3),r(1)]).unwrap();
    assert_eq!(foreign.sign(&inverse).unwrap(),std::cmp::Ordering::Less);
    let retained = foreign.finish(value).unwrap();
    assert_eq!(retained.selected_root().polynomial_coefficients.len(),3);
    println!("premise source_degree=3 retained_degree=2");
    let mut blocked = 0;
    for (index,(alpha,scale)) in [
        (selected(vec![r(-3),r(0),r(1)],r(1),r(2)),q(2,3).sqrt().unwrap()),
        (selected(vec![r(6),r(-4),r(-12),r(8)],q(1,2),r(1)),r(2)),
    ].into_iter().enumerate() {
        let mut field = AlgebraicField::new(&alpha).unwrap();
        let expected = field.rational_function(&alpha,vec![r(0),scale],vec![r(1)]).unwrap();
        match field.subtract(&retained,&expected) {
            Ok(difference) => { assert_eq!(field.sign(&difference).unwrap(),std::cmp::Ordering::Equal); println!("case={index} decided=true"); }
            Err(AlgebraicFieldError::Undecided) => {blocked+=1; println!("case={index} blocked=Undecided");}
            Err(error) => panic!("unexpected error {error:?}"),
        }
    }
    println!("complete requests=2 blocked={blocked}");
}
