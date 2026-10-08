use hyperreal::Real;
use hypersolve::{IsolatedRootInterval, sign_at_selected_root};
fn main() {
 let root=(Real::one()+Real::from(2).sqrt().unwrap()).sqrt().unwrap();
 let interval=IsolatedRootInterval { lower:Real::one(), upper:Real::from(2), exact_root:None, distinct_root_count:1 };
 let defining=[-&root*Real::from(6),Real::from(6),-&root*Real::from(11),Real::from(11),-&root*Real::from(6),Real::from(6),-&root,Real::one()];
 for shift in [1,0,-1] {
  let coefficients=[2,-3,4,-2,5].map(Real::from);
  let mut query=vec![Real::zero();coefficients.len()+1];
  for (i,c) in coefficients.into_iter().enumerate() {query[i]-=&c*&root;query[i+1]+=c;}
  query[0]+=Real::from(shift);
  println!("query begin shift={shift}");
  println!("query end shift={shift} result={:?}",sign_at_selected_root(&defining,&query,&interval));
 }
}
