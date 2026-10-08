use hypercurve::*;
fn p(x: i32, y: i32) -> Point2 { Point2::from_values(x,y) }
fn main() {
 let mut incomplete=0;
 let mut singleton_for_parameter_rectangle=0;
 for policy in [CurveContext::STRICT,CurveContext::APPROXIMATE_512] {
  let point=Curve2::from(QuadraticBezier2::new(p(0,0),p(0,0),p(0,0)));
  let crossing=Curve2::from(LineSeg2::try_new(p(-1,0),p(1,0)).unwrap());
  let ending=Curve2::from(LineSeg2::try_new(p(0,0),p(1,0)).unwrap());
  for (name,other) in [("interior",&crossing),("endpoint",&ending),("point",&point)] {
   for swapped in [false,true] {
    let (a,b)=if swapped {(other,&point)}else{(&point,other)};
    let outcome=a.intersect_curve(b,&policy).unwrap();
    assert_eq!(outcome.certainty,CurveCertainty::Certified);
    let r=outcome.value;
    println!("{policy:?} {name} swapped={swapped} complete={} contacts={} overlaps={} components={} blockers={}",r.is_complete(),r.contacts().len(),r.overlaps().len(),r.parameter_components().len(),r.blockers().len());
    incomplete+=usize::from(!r.is_complete());
    singleton_for_parameter_rectangle+=usize::from(name=="point" && r.is_complete() && !r.contacts().is_empty() && r.parameter_components().is_empty());
   }
  }
 }
 println!("{{\"incomplete_queries\":{incomplete},\"singleton_for_parameter_rectangle\":{singleton_for_parameter_rectangle}}}");
}
