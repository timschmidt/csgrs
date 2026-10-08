use hyperreal::Real;
use hypersolve::{AlgebraicRootRepresentation as Root,IsolatedRootInterval,PredicatePolicy,RootIsolationConfig,AlgebraicRootRefinementComparisonConfig,validate_algebraic_root_representation,refine_isolated_univariate_polynomial_interval,transform_algebraic_root_polynomial_image,compare_algebraic_root_representations_by_difference};
fn r(x:i32)->Real {Real::from(x)}
fn main(){
 // H(x)=x^2-x^3; P(x)=2*H(x)*(H(x)-1/2).
 // On (1/2,1], H is nonnegative and at most 4/27<1/2.
 // Therefore P has exactly the selected root x=1, and its H-image is zero.
 let mut source=Root::from_exact_value(&r(0));source.polynomial_coefficients=vec![r(0),r(0),r(-1),r(1),r(2),r(-4),r(2)];
 source.interval=IsolatedRootInterval{lower:(r(1)/r(2)).unwrap(),upper:r(1),exact_root:None,distinct_root_count:1};
 source.validation=validate_algebraic_root_representation(&source,PredicatePolicy::STRICT);assert!(source.is_valid());
 let proof=refine_isolated_univariate_polynomial_interval(&source.polynomial_coefficients,&source.interval,RootIsolationConfig{policy:PredicatePolicy::STRICT,max_interval_width:None,max_refinement_steps:4});
 assert!(proof.refined_interval.as_ref().and_then(|i|i.exact_root.as_ref())==Some(&r(1)));
 assert!(source.exact_point_witness().is_none());println!("premise selected_source_is_one=true cached_witness=false exact_image_is_zero=true");
 let mut wrong=0;let mut blocked=0;
 for(index,policy)in[PredicatePolicy::STRICT,PredicatePolicy::APPROXIMATE_512].into_iter().enumerate(){
  let report=transform_algebraic_root_polynomial_image(&source,&[r(0),r(0),r(1),r(-1)],policy);println!("case={index} transform={:?}",report.status);
  if let Some(image)=report.representation {let cmp=compare_algebraic_root_representations_by_difference(&image,&Root::from_exact_value(&r(0)),AlgebraicRootRefinementComparisonConfig{policy:PredicatePolicy::STRICT,..Default::default()});println!("case={index} comparison={:?} ordering={:?}",cmp.comparison.status,cmp.comparison.ordering);if cmp.comparison.ordering!=Some(std::cmp::Ordering::Equal){wrong+=1;}}
  else{blocked+=1;}
 }
 println!("complete cases=2 wrong={wrong} blocked={blocked}");
}
