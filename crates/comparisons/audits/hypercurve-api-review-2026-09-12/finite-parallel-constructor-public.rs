use hypercurve::*;
fn p(x:i32,y:i32)->Point2{Point2::from_values(x,y)}
fn q(n:i32,d:i32)->Real{(Real::from(n)/Real::from(d)).unwrap()}
fn exact<T>(value:Classification<T>)->T{match value{Classification::Decided(v)=>v,Classification::Uncertain(e)=>panic!("{e:?}")}}
fn main(){
 let mut failures=0;
 for policy in [CurveContext::STRICT,CurveContext::APPROXIMATE_512]{
  for finite in [false,true]{
   let source=if finite{CubicBezier2::new(p(-1,0),Point2::new((-1).into(),q(-1,3)),Point2::new(q(-2,3),q(-2,3)),p(0,0))}
   else{CubicBezier2::new(p(3,-6),Point2::new(q(-7,3),q(26,3)),Point2::new(q(-7,3),q(-26,3)),p(3,6))};
   for distance in [Real::zero(),q(1,10)]{
    let parallel=source.parallel_left(distance.clone()).unwrap();
    let range=exact(BezierParameterRange2::try_new(BezierParameter2::Exact(if finite{(-2).into()}else{0.into()}),BezierParameter2::Exact(if finite{2.into()}else{1.into()}),&policy).unwrap());
    let result=BezierParallelFragment2::try_new(parallel,range,&policy);
    let valid=matches!(&result,Ok(Classification::Decided(_)));
    println!("finite={finite} distance={distance:?} policy={policy:?} valid={valid} result={result:?}");
    if !valid{failures+=1;}
   }
  }
 }
 println!("failures={failures}");assert_eq!(failures,0);
}
