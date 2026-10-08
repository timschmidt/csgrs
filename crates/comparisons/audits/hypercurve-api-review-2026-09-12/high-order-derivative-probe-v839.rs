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
fn main(){
 let policy=CurveContext::STRICT;
 for (degree,order) in [(8,16),(40,80)] {
  let controls=(0..=degree).map(|i| HomogeneousControl2::new(
   (Real::from(i as u64)/Real::from(degree as u64)).unwrap(),Real::one(),Real::from(if i==degree{2}else{1})
  )).collect();
  let curve=decided(RationalBezier2::from_homogeneous_controls(controls,&policy).unwrap());
  let polynomial=decided(BezierParameterPolynomial::try_new_power_basis(vec![Real::zero(),Real::one()],&policy).unwrap());
  let interval=decided(BezierParameterInterval::try_new(Real::from(-1),Real::one(),&policy).unwrap());
  let parameter=decided(BezierAlgebraicParameter2::try_isolate(polynomial,interval,&policy).unwrap());
  println!("starting degree={degree} order={order}");
  CALLS.store(0,Relaxed);BYTES.store(0,Relaxed);let clock=Instant::now();ENABLED.store(true,Relaxed);
  let result=curve.derivatives_at_algebraic_parameter(&parameter,order,&policy);
  ENABLED.store(false,Relaxed);let ns=clock.elapsed().as_nanos();let allocations=CALLS.load(Relaxed);let requested_bytes=BYTES.load(Relaxed);
  let images=decided(result.unwrap());assert_eq!(images.len(),order);let mut factorial=Real::one();
  // C=(t/(1+t^degree),1/(1+t^degree)); Taylor coefficients at zero are +/-1 or 0.
  for (index,image)in images.iter().enumerate(){
   let k=index+1;factorial*=Real::from(k as u64);
   let dy=if k%degree==0 {if(k/degree)%2==0{factorial.clone()}else{-factorial.clone()}}else{Real::zero()};
   let dx=if(k-1)%degree==0{if((k-1)/degree)%2==0{factorial.clone()}else{-factorial.clone()}}else{Real::zero()};
   for(coordinate,expected)in[(image.dx().unwrap(),dx),(image.dy().unwrap(),dy)]{assert_eq!(coordinate.compare_to_real(&expected,&policy),Classification::Decided(Ordering::Equal),"derivative {k}");}
  }
  println!("degree={degree} order={order} ns={ns} allocations={allocations} requested_bytes={requested_bytes} exact=true");
 }
 println!("high_order_derivative_probe_complete");
}
