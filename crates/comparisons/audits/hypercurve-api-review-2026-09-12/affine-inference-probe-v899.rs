use hyperreal::Real;
use hypersolve::{AlgebraicRootRepresentation as Root,PredicatePolicy as Policy,IsolatedRootInterval,validate_algebraic_root_representation,transform_algebraic_root_affine,algebraic_root_affine_relation,represented_root_sign,compare_algebraic_root_representations_by_difference,AlgebraicRootRefinementComparisonConfig};
fn r(x:i32)->Real{Real::from(x)}
fn q(n:i32,d:i32)->Real{(r(n)/r(d)).unwrap()}
fn sign(x:&Real)->Option<std::cmp::Ordering>{represented_root_sign(&Root::from_exact_value(x),Policy::STRICT)}
fn main(){
 let mut root=Root::from_exact_value(&r(0));root.polynomial_coefficients=vec![-Real::pi(),r(0),r(0),r(4)];root.interval=IsolatedRootInterval{lower:r(0),upper:r(1),exact_root:None,distinct_root_count:1};root.validation=validate_algebraic_root_representation(&root,Policy::STRICT);assert!(root.is_valid());
 for(si,scale)in[q(1,2),r(-2)].into_iter().enumerate(){for(oi,offset)in[r(0),Real::pi()].into_iter().enumerate(){
  let mapped=transform_algebraic_root_affine(&root,scale.clone(),offset.clone(),Policy::STRICT);println!("case={si}:{oi} mapped={:?}",mapped.status);let target=mapped.representation.unwrap();
  println!("case={si}:{oi} relation_present={}",algebraic_root_affine_relation(&root,&target).is_some());
  let mean=|p:&Root| -((p.polynomial_coefficients[2].clone()/p.polynomial_coefficients[3].clone()).unwrap()/r(3)).unwrap();
  let lm=mean(&root);let rm=mean(&target);println!("case={si}:{oi} mean_matches={:?}",sign(&(&rm-&offset)));
  let left=transform_algebraic_root_affine(&root,r(1),-lm.clone(),Policy::STRICT);let right=transform_algebraic_root_affine(&target,r(1),-rm.clone(),Policy::STRICT);println!("case={si}:{oi} center_status={:?}:{:?}",left.status,right.status);
  let left=left.representation.unwrap();let right=right.representation.unwrap();
  for k in [1,0]{let l=(left.polynomial_coefficients[k].clone()/root.polynomial_coefficients[3].clone()).unwrap();let rr=(right.polynomial_coefficients[k].clone()/target.polynomial_coefficients[3].clone()).unwrap();println!("case={si}:{oi} coefficient={k} signs={:?}:{:?} rational={}:{}",sign(&l),sign(&rr),l.exact_rational_ref().is_some(),rr.exact_rational_ref().is_some());
   if k==0{let ratio=(rr/l).unwrap();println!("case={si}:{oi} ratio_rational={} ratio_normal_form={} ratio_expected={:?}",ratio.exact_rational_ref().is_some(),ratio.exact_rational_normal_form().is_some(),sign(&(&ratio-(&scale*&scale*&scale))));let candidate=ratio.root_n(3).unwrap();println!("case={si}:{oi} candidate_rational={} candidate_normal_form={} candidate_matches={:?}",candidate.exact_rational_ref().is_some(),candidate.exact_rational_normal_form().is_some(),sign(&(&candidate-&scale)));let shift=&rm-&candidate*&lm;let proposed=transform_algebraic_root_affine(&root,candidate,shift,Policy::STRICT);println!("case={si}:{oi} proposed={:?}",proposed.status);if let Some(proposed)=proposed.representation{let compared=compare_algebraic_root_representations_by_difference(&proposed,&target,AlgebraicRootRefinementComparisonConfig{policy:Policy::STRICT,..Default::default()});println!("case={si}:{oi} selected_equality={:?}:{:?}",compared.comparison.status,compared.comparison.ordering);}}
  }
 }}println!("complete cases=4");
}
