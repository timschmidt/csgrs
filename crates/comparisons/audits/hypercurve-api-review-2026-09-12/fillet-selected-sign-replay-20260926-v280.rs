use hyperreal::Real;
use hyperlimit::PredicatePolicy;
use hypersolve::{AlgebraicRootRepresentation, AlgebraicRootValidationReport, AlgebraicRootValidationStatus, IsolatedRootInterval, SymbolId};
use std::time::Instant;
fn main() {
    let args=std::env::args().collect::<Vec<_>>();
    let text=std::fs::read_to_string(&args[1]).unwrap(); let mut lines=text.lines();
    assert_eq!(lines.next(),Some("HSIGN1"));
    let count:usize=lines.next().unwrap().parse().unwrap();
    let defining=(0..count).map(|_|Real::from_json(lines.next().unwrap()).unwrap()).collect::<Vec<_>>();
    let lower=Real::from_json(lines.next().unwrap()).unwrap();
    let upper=Real::from_json(lines.next().unwrap()).unwrap();
    let count:usize=lines.next().unwrap().parse().unwrap();
    let predicate=(0..count).map(|_|Real::from_json(lines.next().unwrap()).unwrap()).collect::<Vec<_>>();assert!(lines.next().is_none());
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
