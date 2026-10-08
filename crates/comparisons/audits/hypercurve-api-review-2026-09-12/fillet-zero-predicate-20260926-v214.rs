use hyperreal::Real;
fn main() {
 let a=Real::from(2_i8).sqrt().unwrap();
 let b=Real::from(3_i8).sqrt().unwrap();
 let sum=&a+&b;
 let zero=&sum*&sum-(Real::from(5_i8)+Real::from(2_i8)*Real::from(6_i8).sqrt().unwrap());
 println!("structural_zero={:?}",zero.zero_status());
 for policy in [hypersolve::PredicatePolicy::STRICT,hypersolve::PredicatePolicy::APPROXIMATE_512] {
  println!("predicate={:?}",hypersolve::classify_real_sign_predicate(&zero,policy));
 }
}
