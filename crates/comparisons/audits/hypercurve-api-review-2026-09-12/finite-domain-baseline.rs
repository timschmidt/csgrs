use hypercurve::*;
fn q(n:i32,d:i32)->Real {(Real::from(n)/Real::from(d)).unwrap()}
fn p(x:i32,y:i32)->Point2 {Point2::from_values(x,y)}
fn exact<T>(x:Classification<T>)->T {match x {Classification::Decided(x)=>x,Classification::Uncertain(e)=>panic!("{e:?}")}}
fn certified<T>(x:CurveOutcome<T>)->T {assert_eq!(x.certainty,CurveCertainty::Certified);x.value}
fn exterior(start:Real,end:Real,policy:&CurveContext)->(Curve2,Curve2) {
 let source=QuadraticBezier2::new(p(0,0),Point2::new(q(1,2),Real::zero()),p(1,1));
 let a=source.point_at(start.clone());let b=source.point_at(end.clone());
 let fragment=BezierSplitFragment2::RetainedBezier{reversed:false,start:BezierParameter2::Exact(start),end:BezierParameter2::Exact(end),source_curve:BezierSubcurve2::Quadratic(source),start_image:None,end_image:None};
 let chord=exact(BezierAlgebraicChord2::try_new(b.into(),a.into(),policy).unwrap());
 let boundary=CurveRegionBoundaryLoop2::new(vec![fragment,BezierSplitFragment2::AlgebraicChord(chord)],policy).unwrap();
 let region=CurveRegion2::try_new_with_loop_topology(vec![boundary],vec![CurveRegionLoopRole::Material],vec![FillRule::NonZero],vec![CurveBoundaryInteriorSide2::Left]).unwrap();
 let paths=exact(certified(region.boundary_paths(policy).unwrap()));
 (paths[0].curves()[0].clone(),paths[0].curves()[1].clone())
}
fn main(){let mut complete=0;let mut incomplete=0;
 for policy in [CurveContext::STRICT,CurveContext::APPROXIMATE_512]{
  let (a,chord)=exterior(Real::one(),Real::from(2),&policy);
  let (b,_)=exterior(q(3,2),q(5,2),&policy);
  println!("exterior domain: {:?}",a.parameter_domain());
  let line=Curve2::from(LineSeg2::try_new(Point2::new(q(3,2),Real::zero()),Point2::new(q(3,2),Real::from(4))).unwrap());
  for (name,b) in [("native line",&line),("retained chord",&chord),("same rational support",&b)] {
   for (a,b) in [(&a,b),(b,&a)] {
    let result=certified(a.intersect_curve(b,&policy).unwrap());
    println!("{name}: complete={} contacts={} overlaps={} blockers={:?}",result.is_complete(),result.contacts().len(),result.overlaps().len(),result.blockers());
    if result.is_complete(){complete+=1}else{incomplete+=1}
   }
  }
 }
 println!("{{\"complete\":{complete},\"incomplete\":{incomplete}}}");
}
