use hypercurve::{BezierParallelFragment2,BezierParameter2,BezierParameterRange2,Classification,CubicBezier2,CurveCertainty,CurveContext,CurveCornerMode2,CurveFillet2,CurvePath2,LineSeg2,Point2,Real};
fn q(n:i64,d:i64)->Real{(Real::from(n)/Real::from(d)).unwrap()}
fn decided<T>(value:Classification<T>)->T{match value{Classification::Decided(value)=>value,Classification::Uncertain(reason)=>panic!("fixture classification {reason:?}")}}
fn check(policy:CurveContext,nonzero:bool){
 let source=CubicBezier2::new(Point2::from_values(0,0),Point2::from_values(0,0),Point2::new(q(1,3),Real::zero()),Point2::from_values(1,1));
 let distance=if nonzero{q(1,8)}else{Real::zero()};let parallel=source.parallel_left(distance.clone()).unwrap();
 let start=decided(parallel.point_at(&q(1,4),&policy).unwrap());
 // At t=1/2 the primitive tangent is (4/5,3/5). The clockwise
 // fillet center must be start.x+r horizontally from its vertical line,
 // and C(1/2)+(distance-r)*(-3/5,4/5) on the source normal.
 let radius=((q(1,4)-q(3,5)*&distance-start.x())/q(2,5)).unwrap();
 let center=Point2::new(start.x()+&radius,q(1,8)+q(4,5)*(&distance-&radius));
 let contact=Point2::new(q(1,4)-q(3,5)*distance.clone(),q(1,8)+q(4,5)*distance);
 let range=decided(BezierParameterRange2::try_new(BezierParameter2::Exact(q(1,4)),BezierParameter2::Exact(Real::one()),&policy).unwrap());
 let fragment=decided(BezierParallelFragment2::try_new(parallel,range,&policy).unwrap());
 let line=LineSeg2::try_new(Point2::new(start.x().clone(),Real::from(-2)),start).unwrap();
 let path=CurvePath2::try_new(vec![line.into(),fragment.into()]).unwrap();let mut request=CurveFillet2::new(radius);request.center=Some(center.into());
 let result=path.fillet_vertex(1,&request,CurveCornerMode2::TrimOnly,&policy).unwrap_or_else(|_|panic!("stationarity outside the retained source range cannot reject its exact fillet"));
 assert!(result.certainty==CurveCertainty::Certified);let solutions=result.into_value();assert_eq!(solutions.candidate_count(),1);let solution=solutions.into_solutions().pop().unwrap();
 let result=solution.curves()[1].end().coincides_with(&contact.into(),&policy);assert!(result.certainty==CurveCertainty::Certified && result.value==Classification::Decided(true));
}
#[test]fn retained_regular_source_range_strict(){check(CurveContext::STRICT,false)}
#[test]fn retained_regular_source_range_approximate(){check(CurveContext::APPROXIMATE_512,false)}
#[test]fn retained_nonzero_parallel_range_strict(){check(CurveContext::STRICT,true)}
#[test]fn retained_nonzero_parallel_range_approximate(){check(CurveContext::APPROXIMATE_512,true)}
