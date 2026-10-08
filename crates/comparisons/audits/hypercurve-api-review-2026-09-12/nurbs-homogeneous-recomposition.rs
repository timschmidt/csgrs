use hypercurve::*;
fn main() {
 for policy in [CurveContext::STRICT,CurveContext::APPROXIMATE_512] {
  let half=(Real::one()/Real::from(2)).unwrap();
  let source=NurbsCurve2::try_new(2,vec![Point2::from_values(0,0),Point2::from_values(1,1),Point2::from_values(2,0)],
    vec![Real::one(),-half,Real::one()],vec![Real::zero(),Real::zero(),Real::zero(),Real::one(),Real::one(),Real::one()],&policy).unwrap().into_value();
  let spans=source.degree_elevation(3,&policy).unwrap().into_value();
  println!("homogeneous span degree={} finite bounds={}",spans.spans()[0].curve().degree(),spans.spans()[0].curve().certified_bounds(&policy).is_ok());
  match source.elevated_to_degree(3,&policy) {
   Ok(_) => println!("NURBS recomposition succeeded"),
   Err(error) => println!("NURBS recomposition: {error:?}"),
  }
 }
}
