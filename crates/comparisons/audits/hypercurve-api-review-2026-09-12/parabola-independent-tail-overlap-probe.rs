use hypercurve::{Point2,Real,RationalBezier2,CurveContext};
fn main() {
 let x=Real::from(5).sqrt().unwrap()-Real::from(2); let t=x.clone().sqrt().unwrap();
 let source=RationalBezier2::try_new(vec![Point2::from_values(0,0),Point2::from_values(0,1),Point2::from_values(1,2)],vec![Real::one();3]).unwrap();
 let tail=RationalBezier2::try_new(vec![Point2::new(x,Real::from(2)*&t),Point2::new(t.clone(),Real::one()+&t),Point2::from_values(1,2)],vec![Real::one();3]).unwrap();
 for policy in [CurveContext::STRICT,CurveContext::APPROXIMATE_512] {
  println!("policy {policy:?}");
  println!("candidates {:?}", source.intersection_candidates(&tail,&policy));
  println!("contacts {:?}", source.intersection_contacts(&tail,&policy));
 }
}
