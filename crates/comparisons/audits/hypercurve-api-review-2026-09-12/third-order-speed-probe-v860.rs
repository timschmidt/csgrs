use hypercurve::{BezierAlgebraicParameter2,BezierAlgebraicTangentVector2,BezierAlgebraicSameTangentOrderStatus as Status,BezierParameterPolynomial,BezierParameterInterval,BezierTangentTurnOrdering2 as Turn,Classification,CurveContext,Point2,QuadraticBezier2,Real,compare_algebraic_same_tangent_third_order};
fn decided<T>(value:Classification<T>)->T{match value{Classification::Decided(value)=>value,Classification::Uncertain(reason)=>panic!("unexpected blocker {reason:?}")}}
fn vector(x:i32,y:i32,parameter:&BezierAlgebraicParameter2,policy:&CurveContext)->BezierAlgebraicTangentVector2{
 let curve=QuadraticBezier2::new(Point2::from_values(0,0),Point2::new((Real::from(x)/Real::from(2)).unwrap(),(Real::from(y)/Real::from(2)).unwrap()),Point2::from_values(x,y));
 let image=decided(curve.tangent_at_algebraic_parameter(parameter,policy).unwrap());BezierAlgebraicTangentVector2::from_image(&image).expect("constant vector coordinates")
}
fn main(){
 let mut failures=0;
 for (policy_index,policy)in [CurveContext::STRICT,CurveContext::APPROXIMATE_512].into_iter().enumerate(){
  let polynomial=decided(BezierParameterPolynomial::try_new_power_basis(vec![Real::from(-1),Real::from(2)],&policy).unwrap());
  let interval=decided(BezierParameterInterval::try_new(Real::zero(),Real::one(),&policy).unwrap());
  let parameter=decided(BezierAlgebraicParameter2::try_isolate(polynomial,interval,&policy).unwrap());
  for (name,second_speed,second_third,expected_status,expected_order)in [
   ("same_graph",2,48,Status::SameDirection,None),
   ("lower_graph",2,24,Status::Ordered,Some(Turn::SecondBeforeFirst)),
   ("higher_graph",1,12,Status::Ordered,Some(Turn::FirstBeforeSecond)),
  ]{
   // (t,t^3) and (s*u,c*u^3) have graph coefficients 1 and c/s^3.
   let tangent=vector(1,0,&parameter,&policy);let third=vector(0,6,&parameter,&policy);
   let second=vector(second_speed,0,&parameter,&policy);let second_derivative=vector(0,second_third,&parameter,&policy);
   let result=decided(compare_algebraic_same_tangent_third_order(&tangent,&third,&second,&second_derivative,&policy));
   let matches=result.status==expected_status&&result.ordering==expected_order;failures+=usize::from(!matches);
   println!("case={name} policy={policy_index} status={:?} ordering={:?} expected_status={expected_status:?} expected_order={expected_order:?} matches={matches}",result.status,result.ordering);
  }
 }
 println!("complete cases=6 mismatches={failures}");
}
