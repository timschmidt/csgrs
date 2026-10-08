use std::{hint::black_box,time::Instant,collections::BTreeSet};
use hyperreal::Real;
use hyperlimit::PredicatePolicy;
use hypersolve::{Problem,Expr,SymbolId,BernsteinSubdivisionConfig,BernsteinSubdivisionStatus as Status,BernsteinSubdivisionIntervalStatus as IntervalStatus,subdivide_bernstein_univariate_polynomial_interval_expr};
use num::{BigInt,BigRational,One,Zero};
#[allow(dead_code)]
mod reference {
    include!("main.rs");
    pub fn exact_value(x:&Real)->BigRational {exact(x)}
    pub fn scalar(q:&BigRational)->Real {real(q)}
    pub fn counts(enabled:bool)->(usize,usize) {COUNT.store(enabled,Ordering::Relaxed);(ALLOCS.load(Ordering::Relaxed),BYTES.load(Ordering::Relaxed))}
}
fn ratio(n:usize,d:usize)->BigRational {BigRational::new(BigInt::from(n),BigInt::from(d))}
fn fixture(n:usize,seed:usize,kind:&str)->(Expr,Vec<BigRational>) {
    let x=Expr::symbol(SymbolId(0),"x");
    let roots:Vec<_>=(0..n).map(|i|match kind {
        "cluster"=>ratio(1,3)+ratio(i+1+seed,1<<20),
        "repeated"=>ratio(1,3)+ratio(seed,64),
        "endpoint"=>if i==0{BigRational::zero()}else if i==n-1{BigRational::one()}else{ratio(16*i+seed,16*(n-1))},
        "outside"=>ratio(2,1)+ratio(16*(i+1)+seed,16*(n+1)),
        _=>ratio(16*(i+1)+seed,16*(n+1)),
    }).collect();
    let mut expression=Expr::int(1);
    if kind=="positive" {
        assert_eq!(n%2,0);
        let delta=BigRational::new(BigInt::one(),BigInt::one()<<(20+seed));
        let factor=(x-Expr::real(reference::scalar(&ratio(1,2)))).powi(2)+Expr::real(reference::scalar(&delta));
        for _ in 0..n/2{expression=expression*factor.clone();}
    } else {
        for root in &roots {expression=expression*(x.clone()-Expr::real(reference::scalar(root)));}
    }
    expression=Expr::int(if seed%2==0{(seed+1)as i64}else{-((seed+1)as i64)})*expression;
    (expression,if kind=="positive"{Vec::new()}else{roots})
}
fn main() {
    let args:Vec<_>=std::env::args().collect();let n:usize=args[2].parse().unwrap();let kind=&args[3];let depth:usize=args[4].parse().unwrap();
    let inputs:Vec<_>=(0..13).map(|seed|fixture(n,seed,kind)).collect();
    let mut problem=Problem::default();problem.add_variable("x",Real::zero());
    let evaluate=|seed:usize|subdivide_bernstein_univariate_polynomial_interval_expr(0,&inputs[seed].0,&problem,Real::zero(),Real::one(),BernsteinSubdivisionConfig{policy:PredicatePolicy::STRICT,max_depth:depth});
    for (seed,(_,roots)) in inputs.iter().enumerate() {
        let report=evaluate(seed);
        assert!(matches!(report.status,Status::Completed|Status::DepthLimit),"{n}/{kind}/{seed}: {:?}",report.status);
        assert_eq!(report.degree,Some(n));assert_eq!(report.symbol,Some(SymbolId(0)));
        let mut cursor=BigRational::zero();let mut points=BTreeSet::new();let mut depth_limited=false;
        for interval in &report.intervals {
            let lo=reference::exact_value(&interval.lower);let hi=reference::exact_value(&interval.upper);
            assert!(lo>=BigRational::zero()&&hi<=BigRational::one()&&lo<=hi);
            let inside=roots.iter().filter(|r|**r>lo&&**r<hi).count();
            if interval.status==IntervalStatus::EndpointRoot {
                assert_eq!(lo,hi);assert!(roots.contains(&lo));assert!(points.insert(lo.clone()));
                assert_eq!(interval.exact_root.as_ref().map(reference::exact_value),Some(lo));
                assert_eq!(interval.variation_bound,Some(0));continue;
            }
            assert!(lo<hi);assert_eq!(lo,cursor);cursor=hi.clone();assert!(interval.exact_root.is_none());
            let variation=interval.variation_bound.unwrap();assert!(inside<=variation);assert_eq!(inside%2,variation%2);
            match interval.status {
                IntervalStatus::Empty=>{assert_eq!(inside,0);assert_eq!(variation,0);},
                IntervalStatus::Isolating=>{assert_eq!(inside,1);assert_eq!(variation,1);assert!(!roots.contains(&lo)&&!roots.contains(&hi));},
                IntervalStatus::DepthLimit=>{depth_limited=true;assert_eq!(&hi-&lo,BigRational::new(BigInt::one(),BigInt::one()<<depth));},
                IntervalStatus::EndpointRoot=>unreachable!(),
            }
        }
        assert_eq!(cursor,BigRational::one());assert_eq!(report.status==Status::DepthLimit,depth_limited);
        for root in roots.iter().filter(|r|**r>=BigRational::zero()&&**r<=BigRational::one()) {
            assert!(points.contains(root)||report.intervals.iter().any(|i|i.status!=IntervalStatus::Empty&&reference::exact_value(&i.lower)<*root&&*root<reference::exact_value(&i.upper)));
        }
        if args[1]=="check" {
            let intervals:Vec<_>=report.intervals.iter().map(|i|serde_json::json!({"lower":reference::exact_value(&i.lower).to_string(),"upper":reference::exact_value(&i.upper).to_string(),"status":format!("{:?}",i.status),"variation":i.variation_bound,"root":i.exact_root.as_ref().map(|r|reference::exact_value(r).to_string())})).collect();
            println!("{}",serde_json::json!({"degree":n,"kind":kind,"seed":seed,"depth":depth,"status":format!("{:?}",report.status),"intervals":intervals,"message":report.message}));
        }
    }
    if args[1]=="check"{return;}
    let run=|calls:usize| {let start=Instant::now();for k in 0..calls{black_box(evaluate(k%13));}start.elapsed().as_secs_f64()/calls as f64};
    if args[1]=="alloc" {
        reference::counts(true);run(13);let(allocations,bytes)=reference::counts(false);
        println!("{}",serde_json::json!({"degree":n,"kind":kind,"depth":depth,"calls":13,"allocations":allocations,"bytes":bytes}));return;
    }
    run(13);let calls=(((0.07/run(13))as usize/13).max(1)*13).min(13000);
    let rounds:usize=args.get(5).map(|s|s.parse().unwrap()).unwrap_or(1);
    for round in 0..rounds {println!("{round},{n},{kind},{depth},{calls},{}",run(calls)*1e9);}
}
