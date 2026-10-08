use hyperreal::Real;
use hyperlimit::PredicatePolicy;
use hypersolve::{Problem,Expr,SymbolId,Constraint,VariableBall,context_from_problem,certify_multivariate_quadratic_krawczyk_box,MultivariateQuadraticKrawczykStatus};
use std::{hint::black_box,time::Instant};
use num::{BigInt,BigRational,Zero};
#[allow(dead_code)]
mod reference {
    include!("/tmp/hyper-inverse-audit.nizGgs/src/main.rs");
    pub fn exact(x:&Real)->BigRational {oracle(x)}
    pub fn counts(enabled:bool)->(usize,usize) {COUNT.store(enabled,AO::Relaxed);(ALLOCS.load(AO::Relaxed),BYTES.load(AO::Relaxed))}
}
fn main() {
    let args:Vec<_>=std::env::args().collect();
    let n:usize=args[2].parse().unwrap();
    let radius=(Real::one()/Real::from(16)).unwrap();
    let problems:Vec<_>=(0..13).map(|seed| {
        let mut problem=Problem::default();
        let offsets:Vec<_>=(0..n).map(|i| {
            problem.add_variable(format!("x{i}"),Real::one());
            Expr::symbol(SymbolId(i as u32),format!("x{i}"))-Expr::int(1)
        }).collect();
        for i in 0..n {
            let mut expr=offsets[i].clone().powi(2);
            for(j,offset)in offsets.iter().enumerate() {
                expr=expr+Expr::int(if i==j {(2*n+seed+1) as i64}else{1})*offset.clone();
            }
            problem.add_constraint(Constraint::equality(format!("row{i}"),expr));
        }
        problem
    }).collect();
    let analyses:Vec<_>=problems.iter().map(|p|p.analyze()).collect();
    let contexts:Vec<_>=problems.iter().map(context_from_problem).collect();
    let radii:Vec<_>=(0..n).map(|i|VariableBall{symbol:SymbolId(i as u32),radius:radius.clone()}).collect();
    let evaluate=|i:usize|certify_multivariate_quadratic_krawczyk_box(&analyses[i],&contexts[i],&radii,PredicatePolicy::STRICT);
    for seed in 0..13 {
        let report=evaluate(seed);assert_eq!(report.status,MultivariateQuadraticKrawczykStatus::CertifiedUniqueRoot);
        assert_eq!(report.variables.len(),n);
        // A=dI+11^T, so |A^-1| has row sum (d+2n-2)/(d(d+n)).
        let d=2*n+seed;
        let sum=BigRational::new(BigInt::from(d+2*n-2),BigInt::from(d*(d+n)));
        let r=reference::exact(&radius);
        for variable in &report.variables {
            assert_eq!(reference::exact(&variable.step),BigRational::zero());
            assert_eq!(reference::exact(&variable.image_radius),&sum*&r*&r);
            assert_eq!(reference::exact(&variable.contraction_bound),BigRational::from_integer(BigInt::from(2))*&sum*&r);
        }
    }
    if args[1]=="check" {println!("13 standard quadratic proofs and all fields pass n={n}");return;}
    let run=|calls:usize| {let start=Instant::now();for k in 0..calls {black_box(evaluate(k%13));}start.elapsed().as_secs_f64()/calls as f64};
    if args[1]=="alloc" {reference::counts(true);run(13);let(allocations,bytes)=reference::counts(false);println!("{}",serde_json::json!({"n":n,"kind":"dense","allocations":allocations,"bytes":bytes,"calls":13}));return;}
    run(13);let calls=(((0.07/run(13)) as usize/13).max(1)*13).min(13000);
    println!("0,{n},dense,{calls},{}",run(calls)*1e9);
}
