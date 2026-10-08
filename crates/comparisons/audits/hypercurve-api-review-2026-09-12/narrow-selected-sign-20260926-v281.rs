use hyperreal::Real;
use hyperlimit::PredicatePolicy;
use hypersolve::{AlgebraicRootRepresentation, AlgebraicRootValidationReport, AlgebraicRootValidationStatus, IsolatedRootInterval, SymbolId};
use std::time::Instant;
fn main() {
    let args=std::env::args().collect::<Vec<_>>();
    let alpha=Real::one()+Real::from(2).sqrt().unwrap();
    let root=alpha.clone().sqrt().unwrap();
    let bounds=root.certified_dyadic_interval(-4500).unwrap();
    let lower=Real::new(bounds[0].clone()); let upper=Real::new(bounds[1].clone());
    let defining=vec![Real::zero(),-alpha.clone(),Real::zero(),Real::one()];
    let predicate=vec![Real::from(2)*&alpha,&alpha*&root,&alpha-Real::from(3)];
    let mut interval=IsolatedRootInterval{lower:lower.clone(),upper:upper.clone(),exact_root:None,distinct_root_count:1};
    if args[2]!="native" {
        let bits:i32=args[2].parse().unwrap();
        interval.lower=Real::new(lower.certified_dyadic_interval(-bits).unwrap()[0].clone());
        interval.upper=Real::new(upper.certified_dyadic_interval(-bits).unwrap()[1].clone());
        assert!(hyperlimit::compare_reals(&interval.lower,&lower,PredicatePolicy::STRICT).value().unwrap().is_le());
        assert!(hyperlimit::compare_reals(&interval.upper,&upper,PredicatePolicy::STRICT).value().unwrap().is_ge());
        let root=AlgebraicRootRepresentation{constraint_index:0,symbol:SymbolId(0),interval_index:0,polynomial_coefficients:defining.clone(),interval:interval.clone(),validation:AlgebraicRootValidationReport{status:AlgebraicRootValidationStatus::Valid,message:None}};
        let start=Instant::now();let validation=hypersolve::validate_algebraic_root_representation(&root,PredicatePolicy::STRICT);
        println!("mode={} validation={:?} elapsed={:?}",args[2],validation.status,start.elapsed());
        if validation.status!=AlgebraicRootValidationStatus::Valid{return;}
    }
    let start=Instant::now();
    let sign=hypersolve::sign_at_selected_root(&defining,&predicate,&interval);
    println!("mode={} sign={:?} elapsed={:?}",args[2],sign,start.elapsed());
}
