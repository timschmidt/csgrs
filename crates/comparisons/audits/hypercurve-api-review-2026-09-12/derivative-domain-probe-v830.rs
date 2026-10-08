use hypercurve::{BezierAlgebraicParameter2, BezierParameterInterval, BezierParameterPolynomial, Classification, CurveContext, HomogeneousControl2, RationalBezier2, Real};
use std::alloc::{GlobalAlloc, Layout, System};
use std::cmp::Ordering;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering::Relaxed};
use std::time::Instant;
static ENABLED: AtomicBool = AtomicBool::new(false);
static CALLS: AtomicU64 = AtomicU64::new(0);
static BYTES: AtomicU64 = AtomicU64::new(0);
struct Counting;
unsafe impl GlobalAlloc for Counting {
 unsafe fn alloc(&self, layout: Layout) -> *mut u8 { if ENABLED.load(Relaxed) { CALLS.fetch_add(1,Relaxed); BYTES.fetch_add(layout.size() as u64,Relaxed); } unsafe { System.alloc(layout) } }
 unsafe fn alloc_zeroed(&self, layout: Layout) -> *mut u8 { if ENABLED.load(Relaxed) { CALLS.fetch_add(1,Relaxed); BYTES.fetch_add(layout.size() as u64,Relaxed); } unsafe { System.alloc_zeroed(layout) } }
 unsafe fn realloc(&self, ptr: *mut u8, layout: Layout, size: usize) -> *mut u8 { if ENABLED.load(Relaxed) { CALLS.fetch_add(1,Relaxed); BYTES.fetch_add(size as u64,Relaxed); } unsafe { System.realloc(ptr,layout,size) } }
 unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) { unsafe { System.dealloc(ptr,layout) } }
}
#[global_allocator] static ALLOCATOR: Counting = Counting;
fn decided<T>(v: Classification<T>) -> T {match v { Classification::Decided(x)=>x, Classification::Uncertain(r)=>panic!("unexpected blocker: {r:?}") }}
fn choose(n: usize,k: usize) -> u64 { (0..k.min(n-k)).fold(1,|v,j|v*(n-j) as u64/(j+1) as u64) }
fn curve(m: usize, policy: &CurveContext) -> RationalBezier2 {
 // F=(1+t^2)^m; (X,Y,W)=(tF,F,(1+t)F), so C=(t/(1+t),1/(1+t)).
 let degree=2*m+1;
 let controls=(0..=degree).map(|i|{
  let mut x=Real::zero();let mut y=Real::zero();
  for j in 0..=m {
   let k=2*j;let c=Real::from(choose(m,j));
   if k<=i {y=y+(&c*Real::from(choose(i,k))/Real::from(choose(degree,k))).unwrap();}
   if k<i {x=x+(&c*Real::from(choose(i,k+1))/Real::from(choose(degree,k+1))).unwrap();}
  }
  let weight=&x+&y;HomogeneousControl2::new(x,y,weight)
 }).collect();
 decided(RationalBezier2::from_homogeneous_controls(controls,policy).unwrap())
}
fn parameter(policy: &CurveContext) -> BezierAlgebraicParameter2 {
 let polynomial=decided(BezierParameterPolynomial::try_new_power_basis(vec![Real::from(-1),Real::zero(),Real::from(2)],policy).unwrap());
 let half=(Real::one()/Real::from(2)).unwrap();
 let interval=decided(BezierParameterInterval::try_new(half,Real::one(),policy).unwrap());
 decided(BezierAlgebraicParameter2::try_isolate(polynomial,interval,policy).unwrap())
}
fn pole_curve(policy: &CurveContext) -> RationalBezier2 {
 let m=3; let degree=2*m+3;
 let mut f=vec![Real::zero();degree];
 for j in 0..=m { f[2*j]=&f[2*j]-Real::from(choose(m,j));f[2*j+2]=&f[2*j+2]+Real::from(3*choose(m,j)); }
 let controls=(0..=degree).map(|i|{
  let mut x=Real::zero();let mut y=Real::zero();
  for (k,c) in f.iter().enumerate() {
   if k<=i {y=y+(c*Real::from(choose(i,k))/Real::from(choose(degree,k))).unwrap();}
   if k<i {x=x+(c*Real::from(choose(i,k+1))/Real::from(choose(degree,k+1))).unwrap();}
  }
  let weight=&x+&y;HomogeneousControl2::new(x,y,weight)
 }).collect();
 decided(RationalBezier2::from_homogeneous_controls(controls,policy).unwrap())
}
fn main(){
 for (policy_id,policy) in [CurveContext::STRICT,CurveContext::APPROXIMATE_512].iter().enumerate(){
  let curve=pole_curve(policy);
  for finite in [true,false] {
   let polynomial=decided(BezierParameterPolynomial::try_new_power_basis(vec![Real::one(),Real::zero(),Real::from(-5),Real::zero(),Real::from(6)],policy).unwrap());
   let (lower,upper)=if finite {((Real::from(2)/Real::from(3)).unwrap(),(Real::from(3)/Real::from(4)).unwrap())} else {((Real::one()/Real::from(2)).unwrap(),(Real::from(3)/Real::from(5)).unwrap())};
   let interval=decided(BezierParameterInterval::try_new(lower,upper,policy).unwrap());
   let parameter=decided(BezierAlgebraicParameter2::try_isolate(polynomial,interval,policy).unwrap());
   let result=curve.derivatives_at_algebraic_parameter(&parameter,8,policy).unwrap();
   if finite {
    let images=decided(result);let t=(Real::from(2).sqrt().unwrap()/Real::from(2)).unwrap();let base=Real::one()+t;let mut power=base.clone();let mut factorial=1_u64;
    for (i,image) in images.iter().enumerate(){let k=i+1;factorial*=k as u64;power=power*&base;let dy=(Real::from(if k%2==0{1}else{-1})*Real::from(factorial)/&power).unwrap();for (coordinate,expected)in[(image.dx().unwrap(),-dy.clone()),(image.dy().unwrap(),dy)]{assert_eq!(coordinate.compare_to_real(&expected,policy),Classification::Decided(Ordering::Equal),"derivative {k}");}}
   } else {assert!(matches!(result,Classification::Uncertain(hypercurve::UncertaintyReason::Boundary)));}
   println!("policy={policy_id} finite={finite} correct=true");
  }
 }
 println!("derivative_domain_probe_complete");
}
