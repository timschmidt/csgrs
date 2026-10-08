use hypercurve::*;
fn p(x:i32,y:i32)->Point2 { Point2::from_values(x,y) }
fn main() {
 let policy=CurveContext::STRICT;
 let source=CurvePath2::try_new(vec![
   LineSeg2::try_new(p(-4,0),p(0,0)).unwrap().into(),
   QuadraticBezier2::new(p(0,0),p(0,1),p(1,2)).into(),
 ]).unwrap();
 let generated=source.fillet_vertex_by_radius(1,(Real::one()/Real::from(4)).unwrap(),CurveCornerMode2::TrimOnly,&policy).unwrap();
 assert_eq!(generated.certainty,CurveCertainty::Certified);
 let CurveCornerSolutions2::Unique(path)=generated.value else {panic!("unique fillet")};
 for (i,curve) in path.curves().iter().enumerate() {
  println!("curve={i} family={:?} native={}",curve.family(),curve.geometry().is_some());
  let result=curve.intersect_curve(curve,&policy);
  match result {
   Ok(outcome)=>println!("self {i}: certainty={:?} contacts={} overlaps={} blockers={:?}",outcome.certainty,outcome.value.contacts().len(),outcome.value.overlaps().len(),outcome.value.blockers()),
   Err(error)=>println!("self {i}: error={error:?}"),
  }
 }
}
