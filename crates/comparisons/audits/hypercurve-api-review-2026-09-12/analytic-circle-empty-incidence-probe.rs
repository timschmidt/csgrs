use hypercurve::{CurveContext,Point2,QuadraticBezier2,Real};
fn main(){
 let source=QuadraticBezier2::new(Point2::from_values(0,0),Point2::from_values(1,0),Point2::from_values(1,1));
 let q=(Real::one()/Real::from(4_i8)).unwrap();
 for d in [q.clone(),-q] {
  let radius=Real::one()-&d;
  let parallel=source.parallel_left(d.clone()).unwrap();
  let incidence=parallel.circle_incidence(&Point2::from_values(1,2),&(&radius*&radius),&[],&CurveContext::STRICT).unwrap();
  println!("d={:?} incidence={:?}",d.exact_rational_ref(),incidence.map(|p|p.len()));
 }
}
