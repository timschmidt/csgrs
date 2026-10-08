use hypercurve::*;
fn q(n: i32,d: i32)->Real{(Real::from(n)/Real::from(d)).unwrap()}
#[track_caller]
fn exact<T>(v:Classification<T>)->T{match v{Classification::Decided(v)=>v,Classification::Uncertain(e)=>panic!("{e:?}")}}
fn certified<T>(v:CurveOutcome<T>)->T{assert_eq!(v.certainty,CurveCertainty::Certified);v.value}
#[track_caller]
fn same(a:&CurvePoint2,b:&CurvePoint2,policy:&CurveContext){assert!(exact(certified(a.coincides_with(b,policy))));}
#[track_caller]
fn analytic_line(line:LineSeg2,distance:Real,policy:&CurveContext)->Curve2 {
 let source=line.offset_left(-distance.clone()).unwrap();
 let parallel=QuadraticBezier2::from_line_segment(source).parallel_left(distance).unwrap();
 let range=exact(BezierParameterRange2::try_new(BezierParameter2::Exact(Real::zero()),BezierParameter2::Exact(Real::one()),policy).unwrap());
 let fragment=exact(BezierParallelFragment2::try_new(parallel,range,policy).unwrap());
 let outer=line.offset_left(Real::from(2)).unwrap();
 let mut edges=vec![BezierSplitFragment2::AnalyticParallel(fragment)];
 for(a,b)in[(line.end(),outer.end()),(outer.end(),outer.start()),(outer.start(),line.start())]{
  edges.push(BezierSplitFragment2::AlgebraicChord(exact(BezierAlgebraicChord2::try_new(a.clone().into(),b.clone().into(),policy).unwrap())));
 }
 let boundary=CurveRegionBoundaryLoop2::new(edges,policy).unwrap();
 let region=CurveRegion2::try_new_with_loop_topology(vec![boundary],vec![CurveRegionLoopRole::Material],vec![FillRule::NonZero],vec![CurveBoundaryInteriorSide2::Left]).unwrap();
 let curve=exact(certified(region.boundary_paths(policy).unwrap()))[0].curves()[0].clone();
 assert!(curve.geometry().is_none());
 same(&curve.start(),&line.start().clone().into(),policy);
 same(&curve.end(),&line.end().clone().into(),policy);
 curve
}
fn main(){
 let(mut queries,mut empty,mut replays,mut topologies)=(0,0,0,0);
 for policy in [CurveContext::STRICT,CurveContext::APPROXIMATE_512]{
  for [a,b,c,d] in [[1,0,0,1],[0,-1,1,0],[-1,0,0,1],[3,-4,4,3],[3,4,4,-3],[1,1,-1,1]]{
   eprintln!("layout={a},{b},{c},{d} policy={policy:?}");
   let scale=Real::from(a*a+c*c).sqrt().unwrap();
   let point=|x:Real,y:Real|Point2::new(Real::from(a)*&x+Real::from(b)*&y+Real::from(2),Real::from(c)*x+Real::from(d)*y-Real::from(3));
   let p=|x:i32,y:i32|point(x.into(),y.into());
   let path=CurvePath2::try_new(vec![LineSeg2::try_new(p(-4,0),p(0,0)).unwrap().into(),QuadraticBezier2::new(p(0,0),p(0,1),p(1,2)).into()]).unwrap();
   let CurveCornerSolutions2::Unique(path)=certified(path.fillet_vertex_by_radius(1,scale*q(1,4),CurveCornerMode2::TrimOnly,&policy).unwrap()) else{panic!("unique fillet")};
   let source=&path.curves()[1];let endpoint=source.start();
   for reversed in [false,true]{
    let circle=if reversed{certified(source.reversed(&policy).unwrap())}else{source.clone()};
    for line in [analytic_line(LineSeg2::try_new(p(-1,0),p(1,0)).unwrap(),q(1,3),&policy),analytic_line(LineSeg2::try_new(p(2,0),p(-2,0)).unwrap(),-q(1,3),&policy),analytic_line(LineSeg2::try_new(p(-3,0),p(1,0)).unwrap(),q(2,3),&policy)]{
     for(first,second)in[(&circle,&line),(&line,&circle)]{
      let result=certified(first.intersect_curve(second,&policy).unwrap());assert!(result.is_complete(),"{:?}",result.blockers());
      let[contact]=result.contacts()else{panic!("one tangent contact")};assert_eq!(contact.tangent_cross_sign(),Some(RealSign::Zero));assert!(!contact.is_certified_transverse());same(contact.point(),&endpoint,&policy);
      queries+=1;
      for(curve,location)in[(first,contact.first()),(second,contact.second())]{let parameter=exact(location.parameter(&policy).unwrap());let point=certified(curve.point_at(&parameter,&policy).unwrap());same(&point,contact.point(),&policy);replays+=1;}
      let topology=certified(first.intersection_topology(second,&policy).unwrap());assert!(topology.result().is_complete());topologies+=1;
     }
    }
    // The supporting circle's opposite tangent cannot become this endpoint.
    let opposite=analytic_line(LineSeg2::try_new(point((-1).into(),q(1,2)),point(1.into(),q(1,2))).unwrap(),q(1,3),&policy);
    for(first,second)in[(&circle,&opposite),(&opposite,&circle)]{
     let result=certified(first.intersect_curve(second,&policy).unwrap());assert!(result.is_complete(),"{:?}",result.blockers());assert!(result.contacts().is_empty());assert!(result.overlaps().is_empty());empty+=1;
    }
   }
  }
 }
 println!("{{\"tangent_queries\":{queries},\"opposite_tangent_exclusions\":{empty},\"point_replays\":{replays},\"topologies\":{topologies}}}");
}
