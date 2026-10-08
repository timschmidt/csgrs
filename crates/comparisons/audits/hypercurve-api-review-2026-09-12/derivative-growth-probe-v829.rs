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
fn main(){
 for (policy_id,policy) in [CurveContext::STRICT,CurveContext::APPROXIMATE_512].iter().enumerate(){
  for (m,order) in [(0,3),(0,8),(3,3),(3,8),(7,3),(7,8)] {
   for sample in 0..3 {
    let curve=curve(m,policy);let parameter=parameter(policy);
    // Measure a fresh selected-parameter derivative request, including source basis setup.
    CALLS.store(0,Relaxed);BYTES.store(0,Relaxed);let clock=Instant::now();ENABLED.store(true,Relaxed);
    let result=curve.derivatives_at_algebraic_parameter(&parameter,order,policy);
    ENABLED.store(false,Relaxed);let ns=clock.elapsed().as_nanos();let calls=CALLS.load(Relaxed);let bytes=BYTES.load(Relaxed);
    let images=decided(result.unwrap());assert_eq!(images.len(),order);
    let t=(Real::from(2).sqrt().unwrap()/Real::from(2)).unwrap();let base=Real::one()+t;let mut power=base.clone();let mut factorial=1_u64;
    let mut maximum_denominator=0;
    for (i,image)in images.iter().enumerate(){
     let k=i+1;factorial*=k as u64;power=power*&base;
     let dy=(Real::from(if k%2==0 {1}else{-1})*Real::from(factorial)/&power).unwrap();
     for (coordinate,expected)in [(image.dx().unwrap(),-dy.clone()),(image.dy().unwrap(),dy)]{
      assert_eq!(coordinate.compare_to_real(&expected,policy),Classification::Decided(Ordering::Equal),"derivative {k}");
      maximum_denominator=maximum_denominator.max(coordinate.denominator_coefficients().len());
     }
    }
    println!("policy={policy_id} degree={} order={order} sample={sample} ns={ns} allocations={calls} requested_bytes={bytes} max_image_denominator={maximum_denominator} exact=true",curve.degree());
   }
  }
 }
 println!("derivative_growth_probe_complete");
}
