use std::{hint::black_box,time::Instant};
use hyperreal::Real;
use hyperlimit::PredicatePolicy;
use hypersolve::{Problem,Expr,Constraint,SymbolId,BernsteinSubdivisionConfig,
    BernsteinSubdivisionStatus as Status,BernsteinSubdivisionIntervalStatus as IntervalStatus,
    subdivide_bernstein_univariate_polynomial_interval_roots};
#[allow(dead_code)]
mod reference {
    include!("main.rs");
    pub fn counts(enabled:bool)->(usize,usize) {
        COUNT.store(enabled,Ordering::Relaxed);
        (ALLOCS.load(Ordering::Relaxed),BYTES.load(Ordering::Relaxed))
    }
}
fn main() {
    let mode=std::env::args().nth(1).unwrap();
    // Identical to certification.rs univariate_quadratic_problem(16).
    let x=Expr::symbol(SymbolId(0),"x");
    let mut problem=Problem::default();problem.add_variable("x",Real::from(3));
    for index in 0..16 {
        let scale=index as i64+1;
        problem.add_constraint(Constraint::equality(format!("quadratic {index}"),
            x.clone()*x.clone()*Expr::int(scale)-x.clone()*Expr::int(2*scale)+Expr::int(scale)));
    }
    let analysis=problem.analyze();
    let evaluate=||subdivide_bernstein_univariate_polynomial_interval_roots(
        black_box(&analysis),Real::zero(),Real::from(4),
        BernsteinSubdivisionConfig{policy:PredicatePolicy::APPROXIMATE_512,max_depth:8});
    let reports=evaluate();assert_eq!(reports.len(),16);
    for report in &reports {
        assert_eq!(report.status,Status::Completed);assert_eq!(report.degree,Some(2));
        let mut cursor=Real::zero();let mut roots=0;
        for interval in &report.intervals {
            if interval.status==IntervalStatus::EndpointRoot {
                roots+=1;assert_eq!(interval.exact_root,Some(Real::one()));
                assert_eq!(interval.lower,Real::one());assert_eq!(interval.upper,Real::one());
            } else {
                assert_eq!(interval.status,IntervalStatus::Empty);
                assert_eq!(interval.lower,cursor);cursor=interval.upper.clone();
                assert!(interval.exact_root.is_none());
            }
            assert_eq!(interval.variation_bound,Some(0));
        }
        assert_eq!(roots,1);assert_eq!(cursor,Real::from(4));
    }
    if mode=="check"{println!("{reports:?}");return;}
    drop(reports);
    let run=|calls:usize|{let start=Instant::now();for _ in 0..calls{black_box(evaluate());}start.elapsed().as_secs_f64()/calls as f64};
    if mode=="alloc" {
        reference::counts(true);run(13);let(allocations,bytes)=reference::counts(false);
        println!("{}",serde_json::json!({"degree":16,"kind":"batch","depth":8,"calls":13,"allocations":allocations,"bytes":bytes}));return;
    }
    run(13);let calls=(((0.07/run(13))as usize/13).max(1)*13).min(13000);
    println!("0,16,batch,8,{calls},{}",run(calls)*1e9);
}
