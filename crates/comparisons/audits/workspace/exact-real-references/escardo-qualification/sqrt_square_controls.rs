use hyperreal::{Computable, Rational, Real};
use rug::{Float,Integer,float::Round};
use num::Zero;
use std::{alloc::{GlobalAlloc,Layout,System},env,hint::black_box,
    sync::atomic::{AtomicBool,AtomicU64,Ordering},time::Instant};
static ENABLED:AtomicBool=AtomicBool::new(false);
static ALLOC:AtomicU64=AtomicU64::new(0);
static BYTES:AtomicU64=AtomicU64::new(0);
struct Counter;
unsafe impl GlobalAlloc for Counter {
 unsafe fn alloc(&self,l:Layout)->*mut u8 {if ENABLED.load(Ordering::Relaxed) {ALLOC.fetch_add(1,Ordering::Relaxed);BYTES.fetch_add(l.size() as u64,Ordering::Relaxed);} unsafe {System.alloc(l)}}
 unsafe fn dealloc(&self,p:*mut u8,l:Layout) {unsafe {System.dealloc(p,l)}}
 unsafe fn realloc(&self,p:*mut u8,l:Layout,n:usize)->*mut u8 {if ENABLED.load(Ordering::Relaxed) {ALLOC.fetch_add(1,Ordering::Relaxed);BYTES.fetch_add(n as u64,Ordering::Relaxed);} unsafe {System.realloc(p,l,n)}}
}
#[global_allocator] static GLOBAL:Counter=Counter;
fn r(n:i32,d:i32)->Computable {Computable::rational(Rational::fraction(n.into(),d as u64).unwrap())}
fn source(case:&str,index:i32)->Computable {
 let q=r(1+index%7,8);
 match case {
  "positive"=>q.sin().add(r(-1,16)),
  "negative"=>q.sin().add(r(-1,1)),
  "zero"|"tiny-positive"|"tiny-negative"=>{
   let z=q.clone().sin().square().add(q.cos().square()).add(r(-1,1));
   let tiny=Computable::rational(format!("1/{}",Integer::from(1)<<256).parse().unwrap());
   match case {"tiny-positive"=>z.add(tiny),
      "tiny-negative"=>z.add(tiny.negate()),_=>z}
  },
  "sqrt-control"=>r(2+index%7,1),
  _=>panic!("case"),
 }
}
fn make(case:&str,index:i32)->Computable {
 let x=source(case,index);
 if case=="sqrt-control" {x.sqrt()} else {x.square().sqrt()}
}
fn oracle(case:&str,index:i32)->(Float,Float) {
 let p=8192;
 if case=="zero" {return(Float::with_val(p,0),Float::with_val(p,0));}
 if case.starts_with("tiny-") {let x:Float=Float::with_val(p,1)>>256;return(x.clone(),x);}
 let q=if case=="sqrt-control" {Float::with_val(p,2+index%7)} else {Float::with_val(p,1+index%7)/8};
 let mut lo=q.clone();let mut hi=q;
 if case=="sqrt-control" {lo.sqrt_round(Round::Down);hi.sqrt_round(Round::Up);}
 else {
  lo.sin_round(Round::Down);hi.sin_round(Round::Up);
  let c=if case=="positive" {Float::with_val(p,1)/16} else {Float::with_val(p,1)};
  lo=Float::with_val_round(p,&lo-&c,Round::Down).0;
  hi=Float::with_val_round(p,&hi-&c,Round::Up).0;
  if hi<0 {let neg= -hi; hi= -lo;lo=neg;}
 }
 (lo,hi)
}
fn check(x:&Computable,case:&str,index:i32,bits:i32) {
 let (lo,hi)=oracle(case,index);
 let got=Float::with_val(8192,Integer::from_str_radix(&x.approx(-bits).to_string(),10).unwrap());
 let lo=lo<<bits;let hi=hi<<bits;
 assert!(Float::with_val(8192,&got-&lo).abs()<=1 && Float::with_val(8192,&got-&hi).abs()<=1,"{case} {index} {bits}");
}
fn cpu()->f64 {unsafe {let mut t=libc::timespec{tv_sec:0,tv_nsec:0};assert_eq!(libc::clock_gettime(libc::CLOCK_PROCESS_CPUTIME_ID,&mut t),0);t.tv_sec as f64+t.tv_nsec as f64*1e-9}}
fn main() {
 let a:Vec<_>=env::args().collect();
 if a[1]=="check" {
  let mut count=0;
  for case in ["positive","negative","zero","tiny-positive","tiny-negative","sqrt-control"] {
   for index in 0..7 {
    let x=make(case,index);
    if index==0 {println!("shape {case}: {}",serde_json::to_string(&x).unwrap());}
    for bits in [-16,-1,0,1,2,8,16,32,64,128,255,256,257,512,1024,2048,8,256,0] {check(&x,case,index,bits);count+=1;}
    let restored:Computable=serde_json::from_str(&serde_json::to_string(&x).unwrap()).unwrap();
    for bits in [16,128,1024,0] {check(&restored,case,index,bits);count+=1;}
   }
  }
  println!("PASS direct/history/serde MPFR and exact-zero controls={count}");
  // Real::abs must remain productive for a mathematically zero opaque value.
  let q=Real::from(Rational::fraction(1,8).unwrap());
  let s=q.clone().sin();let c=q.cos();let zero=&(&s*&s)+&(&c*&c)-Real::from(1);
  let abs=zero.abs();
  assert!(abs.to_f64_lossy().unwrap().abs()<=1e-15);
  println!("PASS public Real::abs opaque-zero lossy boundary=1");
  return;
 }
 let case=&a[1];let bits:i32=a[2].parse().unwrap();let seed:i32=a[3].parse().unwrap();let reps:i32=a[4].parse().unwrap();
 let count=a.get(5).is_some_and(|s|s=="count");
 // Warm constants and loader outside the measurement; every measured DAG is fresh.
 black_box(make(case,seed).approx(-bits));
 ENABLED.store(count,Ordering::Relaxed);ALLOC.store(0,Ordering::Relaxed);BYTES.store(0,Ordering::Relaxed);
 let w=Instant::now();let c=cpu();let mut checksum=num::BigInt::zero();
 for index in 0..reps {checksum+=black_box(make(black_box(case),black_box(seed+index))).approx(-bits);}
 let elapsed=cpu()-c;let wall=w.elapsed().as_secs_f64();
 ENABLED.store(false,Ordering::Relaxed);
 println!("{}\t{case}\t{bits}\t{seed}\t{reps}\t{count}\t{elapsed}\t{wall}\t{}\t{}\t{}",env!("CARGO_PKG_NAME"),ALLOC.load(Ordering::Relaxed),BYTES.load(Ordering::Relaxed),checksum);
 for index in 0..7 {check(&make(case,index),case,index,bits);}
}
