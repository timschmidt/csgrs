use hypercurve::{Classification, CurveContext, Point2, QuadraticBezier2, RationalQuadraticBezier2, RationalBezier2, Real};
fn main() {
 let p=|x:i8,y:i8| Point2::from_values(x,y);
 let source=QuadraticBezier2::new(p(-2,0),p(0,0),p(2,0));
 let circle:RationalBezier2=RationalQuadraticBezier2::try_unit_end_weights(p(1,0),p(1,1),p(0,1),(Real::from(2_i8).sqrt().unwrap()/Real::from(2_i8)).unwrap()).unwrap().into();
 for policy in [CurveContext::STRICT,CurveContext::APPROXIMATE_512] {
  let parallel=source.parallel_left(Real::one()).unwrap();
  println!("policy {policy:?}");
  match parallel.intersections(&circle,&policy).unwrap() { Classification::Decided(s)=>println!("parallel complete={} contacts={}",s.is_complete(),s.contacts().len()),Classification::Uncertain(r)=>println!("parallel uncertain {r:?}") }
  let Classification::Decided(Some(offset))=parallel.exact_pythagorean_hodograph_offset(&policy).unwrap() else {panic!("the translated line is rational")};
  println!("rational degree {}",offset.curve().degree());
  println!("rational contacts {:?}",offset.curve().intersection_contacts(&circle,&policy));
 }
}
