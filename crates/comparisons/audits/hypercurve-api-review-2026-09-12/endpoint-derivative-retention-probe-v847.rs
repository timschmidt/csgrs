use hypercurve::{BezierAlgebraicEndpointImage2,BezierAlgebraicImageStatus,BezierAlgebraicParameter2,BezierParameterInterval,BezierParameterPolynomial,Classification,CurveContext,Point2,RationalBezier2,RationalQuadraticBezier2,Real};
fn decided<T>(v:Classification<T>)->T { match v { Classification::Decided(x)=>x,Classification::Uncertain(reason)=>panic!("unexpected blocker: {reason:?}") } }
fn q(n:i32,d:i32)->Real {(Real::from(n)/Real::from(d)).unwrap()}
fn main(){
 for (policy_index,policy) in [CurveContext::STRICT,CurveContext::APPROXIMATE_512].into_iter().enumerate(){
  let polynomial=decided(BezierParameterPolynomial::try_new_power_basis(vec![-Real::pi(),Real::zero(),Real::zero(),Real::from(4)],&policy).unwrap());
  let interval=decided(BezierParameterInterval::try_new(Real::zero(),Real::one(),&policy).unwrap());
  let parameter=decided(BezierAlgebraicParameter2::try_isolate(polynomial,interval,&policy).unwrap());
  let conic=RationalQuadraticBezier2::try_new(Point2::from_values(1,0),Point2::new(q(2,3),q(1,3)),Point2::new(q(1,2),q(1,2)),Real::one(),q(3,2),Real::from(2)).unwrap();
  let general=RationalBezier2::from(conic.clone());
  for (family,endpoint,direct)in [
   (0,BezierAlgebraicEndpointImage2::rational_quadratic(&conic,&parameter,&policy),conic.derivatives_at_algebraic_parameter(&parameter,3,&policy)),
   (1,BezierAlgebraicEndpointImage2::rational(&general,&parameter,&policy),general.derivatives_at_algebraic_parameter(&parameter,3,&policy))
  ]{
   let endpoint=decided(endpoint.unwrap());let direct=decided(direct.unwrap());assert_eq!(direct.len(),3);
   // C=(1/(1+t),t/(1+t)); all three derivatives exist at the positive root.
   for (index,stored)in [Some(decided(endpoint.tangent().unwrap())),endpoint.second_derivative(),endpoint.third_derivative()].into_iter().enumerate(){
    let image=&direct[index];let retained=image.status()==BezierAlgebraicImageStatus::RetainedRationalExpression;
    println!("policy={policy_index} family={family} order={} direct_retained={retained} endpoint_present={} same_image={}",index+1,stored.is_some(),stored==Some(image));
   }
  }
 }
 println!("endpoint_derivative_retention_probe_complete");
}
