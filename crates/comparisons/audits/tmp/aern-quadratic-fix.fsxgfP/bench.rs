use hyperreal::{Real,Rational};
use hypersolve::{Constraint,Expr,Problem,SymbolId,QuadraticResidual,UnivariateQuadraticResidual};
use std::alloc::{GlobalAlloc,Layout,System};
use std::hint::black_box;
use std::sync::atomic::{AtomicBool,AtomicU64,Ordering::Relaxed};
use std::time::Instant;
struct Counter;
static TRACK: AtomicBool=AtomicBool::new(false);
static CALLS: AtomicU64=AtomicU64::new(0);
static BYTES: AtomicU64=AtomicU64::new(0);
unsafe impl GlobalAlloc for Counter {
    unsafe fn alloc(&self, l:Layout)->*mut u8 {
        if TRACK.load(Relaxed) { CALLS.fetch_add(1,Relaxed);BYTES.fetch_add(l.size() as u64,Relaxed); }
        unsafe {System.alloc(l)}
    }
    unsafe fn alloc_zeroed(&self,l:Layout)->*mut u8 {
        if TRACK.load(Relaxed) { CALLS.fetch_add(1,Relaxed);BYTES.fetch_add(l.size() as u64,Relaxed); }
        unsafe {System.alloc_zeroed(l)}
    }
    unsafe fn realloc(&self,p:*mut u8,l:Layout,n:usize)->*mut u8 {
        if TRACK.load(Relaxed) { CALLS.fetch_add(1,Relaxed);BYTES.fetch_add(n as u64,Relaxed); }
        unsafe {System.realloc(p,l,n)}
    }
    unsafe fn dealloc(&self,p:*mut u8,l:Layout) {unsafe {System.dealloc(p,l)}}
}
#[global_allocator] static ALLOC:Counter=Counter;
fn symbol(i:u32)->Expr {Expr::symbol(SymbolId(i),format!("x{i}"))}
fn main() {
    let args:Vec<_>=std::env::args().collect();
    let case=&args[1];let mode=&args[2];let iterations:u64=args[3].parse().unwrap();
    let x=symbol(0);let y=symbol(1);
    let count=if case=="multi_dense32" {32} else {8};
    let mut problem=Problem::default();
    for i in 0..count {problem.add_variable(format!("x{i}"),Real::from(i+1));}
    let wide=Real::from(Rational::fraction(i64::MAX,1_000_000_007).unwrap());
    let cubic=x.clone().powi(2)*x.clone();
    let expression=match case.as_str() {
        "uni_square"|"multi_square"=>x.clone().powi(2)-Expr::int(2),
        "uni_factored"=>(x.clone()+Expr::int(3))*(x.clone()-Expr::int(2)),
        "uni_wide"=>(x.clone()-Expr::real(wide)).powi(2),
        "uni_pi"|"multi_pi"=>(x.clone()-Expr::real(Real::pi())).powi(2),
        "uni_cancellation"|"multi_cancellation"=>cubic.clone()-x.clone()*x.clone().powi(2)+x.clone(),
        "uni_scaled_cancellation"=>cubic.clone()*Expr::int(2)-cubic.clone()-cubic+x.clone(),
        "uni_bad_degree"|"multi_bad_degree"=>x.clone().powi(2)*(x.clone()-x.clone().powi(2)),
        "multi_bad_monomial"=>x.clone().powi(2)*(x.clone()-y.clone()),
        "multi_cross"=>x.clone()*y.clone()*Expr::int(5)+x.clone().powi(2)*Expr::int(2)-y.clone()*Expr::int(7)+Expr::int(11),
        "multi_dense8"|"multi_dense32"=>(1..count).fold(x.clone(),|sum,i|sum+symbol(i)).powi(2),
        "multi_zero_terms"=>(x.clone()-x.clone()+y.clone()-y.clone()+Expr::int(1)).powi(2),
        "analyze_uni16"|"analyze_multi16"=>x.clone(),
        _=>panic!("unknown case"),
    };
    for i in 1..=16 {
        let residual=if case=="analyze_multi16" {
            x.clone()*y.clone()*Expr::int(i)+x.clone().powi(2)*Expr::int(i+1)-y.clone()*Expr::int(i+2)+Expr::int(i)
        } else {x.clone()*x.clone()*Expr::int(i)-x.clone()*Expr::int(2*i)+Expr::int(i)};
        problem.add_constraint(Constraint::equality(format!("row{i}"),residual));
    }
    let run=|| {
        if case.starts_with("analyze") {black_box(black_box(&problem).analyze());}
        else if case.starts_with("uni_") {black_box(UnivariateQuadraticResidual::from_expr(black_box(&expression),black_box(&problem)));}
        else {black_box(QuadraticResidual::from_expr(black_box(&expression),black_box(&problem)));}
    };
    if mode=="result" {
        if case.starts_with("analyze") {println!("{:?}",problem.analyze().facts());}
        else if case.starts_with("uni_") {println!("{:?}",UnivariateQuadraticResidual::from_expr(&expression,&problem));}
        else {println!("{:?}",QuadraticResidual::from_expr(&expression,&problem));}
        return;
    }
    for _ in 0..8 {run();}
    TRACK.store(mode=="alloc",Relaxed);let start=Instant::now();
    for _ in 0..iterations {run();}
    let ns=start.elapsed().as_nanos();TRACK.store(false,Relaxed);
    println!("{{\"iterations\":{iterations},\"ns\":{ns},\"allocations\":{},\"requested_bytes\":{}}}",CALLS.load(Relaxed),BYTES.load(Relaxed));
}
