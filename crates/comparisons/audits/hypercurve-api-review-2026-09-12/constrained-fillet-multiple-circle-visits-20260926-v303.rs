use hypercurve::{Curve2, CurveContext, CurveCornerMode2, CurveFillet2, CurveFilletContact2, CurvePath2, LineSeg2, Point2, Real};
fn q(n:i64,d:i64)->Real{(Real::from(n)/Real::from(d)).unwrap()}
fn main(){
 let policy=CurveContext::STRICT;
 let points=[(1,0),(1,1),(0,1),(-1,1),(-1,0),(-1,-1),(0,-1),(1,-1),(1,0),(1,1),(0,1),(-1,1),(-1,0),(-1,-1),(0,-1),(1,-1),(1,0)].into_iter().map(|(x,y)|Point2::from_values(x,y)).collect();
 let weights=(0..17).map(|i|Real::from(1_i64 << (i/2))).collect();
 let mut knots=vec![Real::zero();3];for i in 1..8{knots.extend([Real::from(i),Real::from(i)]);}knots.extend([Real::from(8),Real::from(8),Real::from(8)]);
 let circle=Curve2::try_nurbs(2,points,weights,knots,&policy).unwrap().value;
 let path=CurvePath2::try_new(vec![circle,LineSeg2::try_new(Point2::from_values(1,0),Point2::from_values(6,0)).unwrap().into()]).unwrap();
 for parameter in [q(10,3),q(22,3)] {
  let mut request=CurveFillet2::new(Real::from(4)); request.contacts[0]=Some(CurveFilletContact2::Parameter(parameter.clone().into()));
  let result=path.fillet_vertex(1,&request,CurveCornerMode2::TrimOnly,&policy).unwrap();
  println!("parameter {} candidate_count {}", if parameter==q(10,3){"first turn"}else{"second turn"},result.value.candidate_count());
 }
}
