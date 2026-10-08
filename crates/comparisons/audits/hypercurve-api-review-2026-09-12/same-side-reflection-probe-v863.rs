use hypercurve::{BezierAlgebraicParameter2,BezierAlgebraicTangentVector2,BezierAlgebraicSameTangentOrderStatus as Status,BezierParameterPolynomial,BezierParameterInterval,BezierTangentTurnOrdering2 as Turn,Classification,CurveContext,Point2,QuadraticBezier2,Real,compare_algebraic_same_tangent_third_order,compare_algebraic_same_tangent_second_order};
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
  for order in [2_u32,3]{
   for sign in [1_i32,-1]{
    for swap in [false,true]{
     let (first_k,second_k)=if swap{(2,1)}else{(1,2)};
     let factor=if order==2{2}else{6};
     let tangent=vector(1,0,&parameter,&policy);
     let first=vector(0,sign*factor*first_k,&parameter,&policy);
     let second=vector(0,sign*factor*second_k,&parameter,&policy);
     let result=decided(if order==2{compare_algebraic_same_tangent_second_order(&tangent,&first,&tangent,&second,&policy)}else{compare_algebraic_same_tangent_third_order(&tangent,&first,&tangent,&second,&policy)});
     // At x>0, reflection reverses the order of (x,k*x^n) rays.
     let expected=if (sign==1)!=swap{Turn::FirstBeforeSecond}else{Turn::SecondBeforeFirst};
     let matches=result.status==Status::Ordered&&result.ordering==Some(expected);failures+=usize::from(!matches);
     println!("order={order} sign={sign} swap={swap} policy={policy_index} status={:?} ordering={:?} expected={expected:?} matches={matches}",result.status,result.ordering);
    }
   }
  }
 }
 println!("complete cases=16 mismatches={failures}");
}
