use hypercurve::*;
fn q(n:i32,d:i32)->Real{(Real::from(n)/Real::from(d)).unwrap()}
fn p(x:i32,y:i32)->Point2{Point2::from_values(x,y)}
fn decided<T>(x:Classification<T>)->T{match x{Classification::Decided(x)=>x,Classification::Uncertain(r)=>panic!("{r:?}")}}
fn analytic_cap(distance: Real, dx: i32, policy: &CurveContext) -> CurveRegion2 {
    let point=|x,y|p(x+dx,y);
    let parallel=QuadraticBezier2::new(point(0,0),point(2,2),point(4,0)).parallel_left(distance).unwrap();
    let left=decided(parallel.point_at(&Real::zero(),policy).unwrap());
    let right=decided(parallel.point_at(&Real::one(),policy).unwrap());
    let range=decided(BezierParameterRange2::try_new(BezierParameter2::Exact(Real::one()),BezierParameter2::Exact(Real::zero()),policy).unwrap());
    let analytic=decided(BezierParallelFragment2::try_new(parallel,range,policy).unwrap());
    let lower_left=Point2::new(left.x().clone(),(-2).into());
    let lower_right=Point2::new(right.x().clone(),(-2).into());
    let line=|a:Point2,b:Point2| BezierSplitFragment2::Materialized {
        start:BezierParameter2::Exact(Real::zero()),end:BezierParameter2::Exact(Real::one()),
        curve:BezierSubcurve2::Quadratic(QuadraticBezier2::new(a.clone(),a.lerp(&b,q(1,2)),b)),
    };
    let boundary=CurveRegionBoundaryLoop2::new(vec![BezierSplitFragment2::AnalyticParallel(analytic),line(left,lower_left.clone()),line(lower_left,lower_right.clone()),line(lower_right,right)],policy).unwrap();
    CurveRegion2::try_new_with_loop_topology(vec![boundary],vec![CurveRegionLoopRole::Material],vec![FillRule::NonZero],vec![CurveBoundaryInteriorSide2::Left]).unwrap()
}

fn main(){
 let policy=CurveContext::STRICT;
 let cap=analytic_cap(q(1,2),0,&policy);
 let line=Curve2::from(LineSeg2::try_new(p(1,-3),p(1,3)).unwrap());
 let start=std::time::Instant::now();
 let result=line.trim_inside_region_with_parameters(&cap,&policy).unwrap();
 assert_eq!(result.certainty,CurveCertainty::Certified);
 assert_eq!(result.value.len(),1);
 println!("trim elapsed_ms={} fragments={}",start.elapsed().as_millis(),result.value.len());
}
