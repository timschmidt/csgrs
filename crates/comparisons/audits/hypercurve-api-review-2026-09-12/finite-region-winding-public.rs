use hypercurve::*;
fn p(x:i32,y:i32)->Point2 { Point2::from_values(x,y) }
fn q(n:i32,d:i32)->Real {(Real::from(n)/Real::from(d)).unwrap()}
fn exact<T>(v:Classification<T>)->T {match v {Classification::Decided(v)=>v,Classification::Uncertain(e)=>panic!("{e:?}")}}
fn certified<T>(v:CurveOutcome<T>)->T {assert_eq!(v.certainty,CurveCertainty::Certified);v.value}
fn same(a:&CurvePoint2,b:&CurvePoint2,policy:&CurveContext){assert!(exact(certified(a.coincides_with(b,policy))));}
fn cap(shift:i32,policy:&CurveContext)->CurveRegion2 {
 let s=Real::from(shift);
 let source=QuadraticBezier2::new(Point2::new(-(&s*&s),-&s),Point2::new(-(&s*&s)+&s,q(1,2)-&s),Point2::new(-((Real::one()-&s)*(Real::one()-&s)),Real::one()-&s));
 let curve=BezierSplitFragment2::RetainedBezier{reversed:false,source_curve:BezierSubcurve2::Quadratic(source),start:BezierParameter2::Exact(s.clone()),end:BezierParameter2::Exact(&s+Real::one()),start_image:None,end_image:None};
 let boundary=CurveRegionBoundaryLoop2::new(vec![curve,BezierSplitFragment2::AlgebraicChord(exact(BezierAlgebraicChord2::try_new(p(-1,1).into(),p(0,0).into(),policy).unwrap()))],policy).unwrap();
 CurveRegion2::try_new_with_loop_topology(vec![boundary],vec![CurveRegionLoopRole::Material],vec![FillRule::NonZero],vec![CurveBoundaryInteriorSide2::Left]).unwrap()
}
fn main() {
 for policy in [CurveContext::STRICT,CurveContext::APPROXIMATE_512] {
  for shift in [0,1,-2] {
   let region=cap(shift,&policy);
   for n in [-5,-4,-3,-2,-1,0,1] {
    let inside=region.classify_point(&Point2::new(q(n,8),q(1,2)),&policy);
    println!("shift={shift} x={n}/8 policy={policy:?} location={inside:?}");
   }
  }
 }
}
