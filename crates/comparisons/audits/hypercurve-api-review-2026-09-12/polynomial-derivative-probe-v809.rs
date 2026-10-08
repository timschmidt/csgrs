use hypercurve::{BezierAlgebraicParameter2, BezierParameterPolynomial, BezierParameterInterval, BezierAlgebraicEndpointImage2, Classification, CurveContext, CubicBezier2, QuadraticBezier2, RationalBezier2, Point2, Real};
fn decided<T>(v: Classification<T>)->T {match v {Classification::Decided(v)=>v,Classification::Uncertain(r)=>panic!("blocked: {r:?}")}}
fn main(){
 for (pi, policy) in [CurveContext::STRICT,CurveContext::APPROXIMATE_512].iter().enumerate(){
  let polynomial=decided(BezierParameterPolynomial::try_new_power_basis(vec![Real::from(-1),Real::zero(),Real::from(2)],policy).unwrap());
  let interval=decided(BezierParameterInterval::try_new((Real::one()/Real::from(2)).unwrap(),Real::one(),policy).unwrap());
  let root=decided(BezierAlgebraicParameter2::try_isolate(polynomial,interval,policy).unwrap());
  for (ci, coefficient)in [Real::from(2).sqrt().unwrap(),Real::pi()].into_iter().enumerate(){
   let p=|x:Real,y:i32|Point2::new(x,Real::from(y));
   let controls=vec![p(Real::zero(),0),p(Real::zero(),1),p(coefficient.clone(),2)];
   let quad=QuadraticBezier2::new(controls[0].clone(),controls[1].clone(),controls[2].clone());
   let image=decided(quad.point_at_algebraic_parameter(&root,policy).unwrap());
   let endpoint=decided(BezierAlgebraicEndpointImage2::quadratic(&quad,&root,policy).unwrap());
   let general=RationalBezier2::try_new(controls,vec![Real::one();3]).unwrap();
   let equivalent=decided(general.point_at_algebraic_parameter(&root,policy).unwrap());
   println!("policy={pi} coefficient={ci} quadratic={:?} endpoint={:?} equivalent={:?}",image.status(),decided(endpoint.point().unwrap()).status(),equivalent.status());
   println!("derivative policy={pi} coefficient={ci} family=quad native={:?} general={:?} endpoint_exact={}",quad.tangent_at_algebraic_parameter(&root,policy).unwrap().status(),general.tangent_at_algebraic_parameter(&root,policy).unwrap().status(),endpoint.is_exact());
   let controls=vec![p(Real::zero(),0),p(Real::zero(),1),p(Real::zero(),2),p(coefficient,3)];
   let cubic=CubicBezier2::new(controls[0].clone(),controls[1].clone(),controls[2].clone(),controls[3].clone());
   let image=decided(cubic.point_at_algebraic_parameter(&root,policy).unwrap());
   let endpoint=decided(BezierAlgebraicEndpointImage2::cubic(&cubic,&root,policy).unwrap());
   let general=RationalBezier2::try_new(controls,vec![Real::one();4]).unwrap();
   let equivalent=decided(general.point_at_algebraic_parameter(&root,policy).unwrap());
   println!("policy={pi} coefficient={ci} cubic={:?} endpoint={:?} equivalent={:?}",image.status(),decided(endpoint.point().unwrap()).status(),equivalent.status());
   println!("derivative policy={pi} coefficient={ci} family=cubic native={:?} general={:?} endpoint_exact={}",cubic.tangent_at_algebraic_parameter(&root,policy).unwrap().status(),general.tangent_at_algebraic_parameter(&root,policy).unwrap().status(),endpoint.is_exact());
  }
 }
 println!("polynomial_derivative_probe_complete");
}
