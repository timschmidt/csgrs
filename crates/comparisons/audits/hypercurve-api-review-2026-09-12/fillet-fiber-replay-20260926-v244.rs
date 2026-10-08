use hyperreal::Real;
use hyperlimit::PredicatePolicy;
use hypersolve::{AlgebraicRootRepresentation, AlgebraicRootValidationReport, AlgebraicRootValidationStatus, IsolatedRootInterval, SymbolId, BivariatePolynomial, CurveResultantParameter, AlgebraicFiberRootIsolationConfig, isolate_bivariate_fiber_roots_at_algebraic_parameter, validate_algebraic_root_representation};
use std::time::Instant;
fn main() {
    let text=std::fs::read_to_string(std::env::args().nth(1).expect("fixture path")).unwrap();
    let mut lines=text.lines();
    assert_eq!(lines.next(),Some("HFIBER1"));
    let count:usize=lines.next().unwrap().parse().unwrap();
    let coefficients=(0..count).map(|_|Real::from_json(lines.next().unwrap()).unwrap()).collect::<Vec<_>>();
    let lower=Real::from_json(lines.next().unwrap()).unwrap();
    let upper=Real::from_json(lines.next().unwrap()).unwrap();
    let exact=lines.next().unwrap()=="1";
    let exact_root=exact.then(||Real::from_json(lines.next().unwrap()).unwrap());
    let fiber_lower=Real::from_json(lines.next().unwrap()).unwrap();
    let fiber_upper=Real::from_json(lines.next().unwrap()).unwrap();
    let count:usize=lines.next().unwrap().parse().unwrap();
    let rows=(0..count).map(|_|{
        let count:usize=lines.next().unwrap().parse().unwrap();
        (0..count).map(|_|Real::from_json(lines.next().unwrap()).unwrap()).collect::<Vec<_>>()
    }).collect::<Vec<_>>();
    assert!(lines.next().is_none());
    let mut root=AlgebraicRootRepresentation{constraint_index:0,symbol:SymbolId(0),interval_index:0,polynomial_coefficients:coefficients,interval:IsolatedRootInterval{lower,upper,exact_root,distinct_root_count:1},validation:AlgebraicRootValidationReport{status:AlgebraicRootValidationStatus::Valid,message:None}};
    root.validation=validate_algebraic_root_representation(&root,PredicatePolicy::STRICT);
    assert!(root.is_valid());
    println!("replay admitted modulus_degree={} fiber_rows={} fiber_columns={}",root.polynomial_coefficients.len()-1,rows.len(),rows.iter().map(Vec::len).max().unwrap());
    let start=Instant::now();
    let report=isolate_bivariate_fiber_roots_at_algebraic_parameter(&BivariatePolynomial::new(rows),CurveResultantParameter::First,&root,&fiber_lower,&fiber_upper,AlgebraicFiberRootIsolationConfig{max_subdivision_depth:512,refinement_steps:8},PredicatePolicy::STRICT);
    println!("status={:?} roots={} sturm={} subdivisions={} refinements={} message={:?} elapsed={:?}",report.status,report.intervals.len(),report.sturm_sequence_length,report.subdivision_steps,report.retained_refinement_steps,report.message,start.elapsed());
}
