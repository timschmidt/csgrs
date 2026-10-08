use hyperreal::Real;
use hyperlimit::PredicatePolicy;
use hypersolve::{Problem,Expr,SymbolId,Constraint,VariableBall,context_from_problem,certify_affine_krawczyk_box,AffineKrawczykStatus};
use std::{hint::black_box,time::Instant};
#[allow(dead_code)]
mod reference {
    include!("/tmp/hyper-inverse-audit.nizGgs/src/main.rs");
    pub fn input(n:usize,s:usize,k:&str)->Matrix {matrix(n,s,k)}
    pub fn exact(x:&Real)->BigRational {oracle(x)}
    pub fn counts(enabled:bool)->(usize,usize) {COUNT.store(enabled,AO::Relaxed);(ALLOCS.load(AO::Relaxed),BYTES.load(AO::Relaxed))}
}
fn main() {
    let args:Vec<_>=std::env::args().collect();
    let n:usize=args[2].parse().unwrap();let kind=&args[3];
    let roots:Vec<_>=(0..n).map(|i|(Real::from((i+1) as u64)/Real::from(16)).unwrap()).collect();
    let problems:Vec<_>=(0..13).map(|seed| {
        let mut problem=Problem::default();
        for i in 0..n {problem.add_variable(format!("x{i}"),Real::zero());}
        for (i,row) in reference::input(n,seed,kind).into_iter().enumerate() {
            let mut expr=Expr::int(0);
            for(j,a)in row.into_iter().enumerate() {
                expr=expr+Expr::real(a)*(Expr::symbol(SymbolId(j as u32),format!("x{j}"))-Expr::real(roots[j].clone()));
            }
            problem.add_constraint(Constraint::equality(format!("row{i}"),expr));
        }
        problem
    }).collect();
    let analyses:Vec<_>=problems.iter().map(|p|p.analyze()).collect();
    let contexts:Vec<_>=problems.iter().map(context_from_problem).collect();
    let radii:Vec<_>=(0..n).map(|i|VariableBall{symbol:SymbolId(i as u32),radius:(Real::from(n as u64)/Real::from(16)).unwrap()}).collect();
    let evaluate=|i:usize|certify_affine_krawczyk_box(&analyses[i],&contexts[i],&radii,PredicatePolicy::STRICT);
    for seed in 0..13 {
        let report=evaluate(seed);assert_eq!(report.status,AffineKrawczykStatus::CertifiedUniqueRoot);
        assert_eq!(report.steps.len(),n);
        for (i,step) in report.steps.iter().enumerate() {
            assert_eq!(reference::exact(&step.step),reference::exact(&roots[i]));
            assert_eq!(reference::exact(&step.certified_root),reference::exact(&roots[i]));
        }
    }
    if args[1]=="check" {println!("13 affine proofs and all nonzero root/step fields pass n={n} {kind}");return;}
    let run=|calls:usize| {let start=Instant::now();for k in 0..calls {black_box(evaluate(k%13));}start.elapsed().as_secs_f64()/calls as f64};
    if args[1]=="alloc" {reference::counts(true);run(13);let(allocations,bytes)=reference::counts(false);println!("{}",serde_json::json!({"n":n,"kind":kind,"allocations":allocations,"bytes":bytes,"calls":13}));return;}
    run(13);let calls=(((0.07/run(13)) as usize/13).max(1)*13).min(13000);
    println!("0,{n},{kind},{calls},{}",run(calls)*1e9);
}
