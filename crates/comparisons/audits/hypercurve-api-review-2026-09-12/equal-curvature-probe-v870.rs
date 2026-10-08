use hypercurve::{BezierArrangementFragment2,BezierArrangementGraph2,BezierParameter2,BezierSplitFragment2,BezierSubcurve2,BooleanOp,Classification,CubicBezier2,Curve2,CurveCertainty,CurveContext,CurvePath2,CurvePoint2,CurveRegion2,FillRule,LineSeg2,Point2,QuadraticBezier2,Real,RegionPointLocation};
fn q(n:i32,d:i32)->Real{(Real::from(n)/Real::from(d)).unwrap()}
fn curve(k:i32)->CubicBezier2{CubicBezier2::new(Point2::from_values(0,0),Point2::new(q(1,3),Real::zero()),Point2::new(q(2,3),q(1,3)),Point2::from_values(1,1+k))}
fn fragment(index:usize,curve:BezierSubcurve2)->BezierArrangementFragment2{BezierArrangementFragment2::new(index,0,BezierSplitFragment2::Materialized{start:BezierParameter2::Exact(Real::zero()),end:BezierParameter2::Exact(Real::one()),curve})}
fn cap(k:i32,policy:&CurveContext)->CurveRegion2{
 let path=CurvePath2::try_new_with_policy(vec![Curve2::from(curve(k)),Curve2::from(LineSeg2::try_new(Point2::from_values(1,1+k),Point2::from_values(1,0)).unwrap()),Curve2::from(LineSeg2::try_new(Point2::from_values(1,0),Point2::from_values(0,0)).unwrap())],policy).unwrap().into_value();
 let region=CurveRegion2::try_from_boundary_paths(&[path],FillRule::NonZero,policy).unwrap();assert_eq!(region.certainty,CurveCertainty::Certified);region.value
}
fn main(){
 let mut blocked=0;let mut operations=0;
 for (pi,policy)in [CurveContext::STRICT,CurveContext::APPROXIMATE_512].into_iter().enumerate(){
  let incoming=BezierSubcurve2::Quadratic(QuadraticBezier2::new(Point2::from_values(-1,0),Point2::new(q(-1,2),Real::zero()),Point2::from_values(0,0)));
  let graph=BezierArrangementGraph2::new(vec![fragment(0,incoming),fragment(1,BezierSubcurve2::Cubic(curve(1))),fragment(2,BezierSubcurve2::Cubic(curve(2)))]).unwrap();
  for (kind,value)in [("native",graph.traverse_with_tangent_order(&policy)),("retained",graph.traverse_retained_with_tangent_order(&policy))]{
   match value{Classification::Decided(v)=>{assert_eq!(v.chains()[0].fragment_indices(),[0,1]);println!("policy={pi} arrangement={kind} decided=true");},Classification::Uncertain(reason)=>{blocked+=1;println!("policy={pi} arrangement={kind} blocked={reason:?}");}}
  }
  let inner=cap(1,&policy);let outer=cap(2,&policy);
  for (op,expected)in [(BooleanOp::Union,[true,true,false]),(BooleanOp::Intersection,[true,false,false]),(BooleanOp::Difference,[false,true,false]),(BooleanOp::Xor,[false,true,false])]{
   match outer.boolean_region(&inner,op,&policy){
    Ok(value)=>{assert_eq!(value.certainty,CurveCertainty::Certified);for (y,inside)in [q(1,8),q(7,16),q(5,8)].into_iter().zip(expected){let loc=value.value.classify_point(&CurvePoint2::from(Point2::new(q(1,2),y)),&policy).unwrap();assert_eq!(loc.certainty,CurveCertainty::Certified);assert_eq!(loc.value,Classification::Decided(if inside{RegionPointLocation::Inside}else{RegionPointLocation::Outside}));}operations+=1;println!("policy={pi} boolean={op:?} certified=true");},
    Err(hypercurve::ExactCurveError::Blocked(b))=>println!("policy={pi} boolean={op:?} blocked={:?}",b.reason()),
    Err(_)=>panic!("unexpected invalid region operation"),
   }
  }
 }
 println!("complete arrangement_requests=4 blocked={blocked} certified_booleans={operations}");
}
