use hypercurve::*;
fn q(n:i32,d:i32)->Real {(Real::from(n)/Real::from(d)).unwrap()}
fn p(x:i32,y:i32)->Point2 {Point2::from_values(x,y)}
fn exact<T>(value:Classification<T>)->T {match value {Classification::Decided(value)=>value, Classification::Uncertain(reason)=>panic!("{reason:?}")}}
fn certified<T>(value:CurveOutcome<T>)->T {assert_eq!(value.certainty,CurveCertainty::Certified);value.value}
fn exported(parallel:BezierParallel2,side:CurveBoundaryInteriorSide2,policy:&CurveContext)->(Curve2,Curve2) {
 let start=exact(parallel.point_at(&Real::zero(),policy).unwrap());
 let end=exact(parallel.point_at(&Real::one(),policy).unwrap());
 let range=exact(BezierParameterRange2::try_new(Real::zero().into(),Real::one().into(),policy).unwrap());
 let fragment=exact(BezierParallelFragment2::try_new(parallel,range,policy).unwrap());
 let chord=exact(BezierAlgebraicChord2::try_new(end.into(),start.into(),policy).unwrap());
 let boundary=CurveRegionBoundaryLoop2::new(vec![BezierSplitFragment2::AnalyticParallel(fragment),BezierSplitFragment2::AlgebraicChord(chord)],policy).unwrap();
 let region=CurveRegion2::try_new_with_loop_topology(vec![boundary],vec![CurveRegionLoopRole::Material],vec![FillRule::NonZero],vec![side]).unwrap();
 let paths=certified(region.boundary_paths(policy).unwrap());
 assert_eq!(paths.len(),1); assert_eq!(paths[0].curves().len(),2);
 (paths[0].curves()[0].clone(),paths[0].curves()[1].clone())
}
fn main(){
 let mut incomplete=0;
 for policy in [CurveContext::STRICT,CurveContext::APPROXIMATE_512]{
  let a=QuadraticBezier2::new(p(0,0),p(1,0),p(2,1)).parallel_left(Real::one()).unwrap();
  let b=QuadraticBezier2::new(p(1,1),p(1,2),p(2,3)).parallel_left(Real::one()).unwrap();
  let line=LineSeg2::try_new(Point2::new((-1).into(),q(3,2)),Point2::new(3.into(),q(3,2))).unwrap();
  let rational=RationalBezier2::try_new(vec![line.start().clone(),line.end().clone()],vec![Real::one();2]).unwrap();
  let native=exact(a.intersections(&rational,&policy).unwrap());assert!(native.is_complete());assert_eq!(native.contacts().len(),1);
  let native=exact(a.parallel_intersections(&b,&policy).unwrap());assert!(native.is_complete());assert!(native.contacts().iter().any(|c|c.first_parameter().scalar()==Some(&Real::zero())&&c.second_parameter().scalar()==Some(&Real::zero())));
  let (first,chord)=exported(a,CurveBoundaryInteriorSide2::Left,&policy);
  let (second,_)=exported(b,CurveBoundaryInteriorSide2::Right,&policy);
  assert_eq!(certified(first.start().coincides_with(&second.start(),&policy)),Classification::Decided(true));
  let line=Curve2::from(line);
  for (name,other) in [("rational",&line),("chord",&chord),("parallel",&second)]{
   for swapped in [false,true]{
    let (left,right)=if swapped {(other,&first)}else{(&first,other)};
    let result=certified(left.intersect_curve(right,&policy).unwrap());
    println!("{policy:?} {name} swapped={swapped} complete={} contacts={} overlaps={} blockers={}",result.is_complete(),result.contacts().len(),result.overlaps().len(),result.blockers().len());
    assert!(!result.is_complete());incomplete+=1;
   }
  }
 }
 println!("{{\"incomplete_common_queries\":{incomplete},\"lower_kernels_complete\":true}}");
}
