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
fn main(){
 let(mut queries,mut replays,mut topologies,mut splits,mut child_queries,mut region_queries,mut boolean_reentries)=(0,0,0,0,0,0,0);
 for policy in [CurveContext::STRICT,CurveContext::APPROXIMATE_512]{
  let path=CurvePath2::try_new(vec![LineSeg2::try_new(p(-4,0),p(0,0)).unwrap().into(),QuadraticBezier2::new(p(0,0),p(0,1),p(1,2)).into(),LineSeg2::try_new(p(1,2),p(-4,0)).unwrap().into()]).unwrap();
  let CurveCornerSolutions2::Unique(path)=certified(path.fillet_vertex_by_radius(1,q(1,4),CurveCornerMode2::TrimOnly,&policy).unwrap())else{panic!("unique fillet")};
  let original_circle=&path.curves()[1];
  for shift in [0,1,-2]{
   let cap=cap(shift,&policy);
   let paths=exact(certified(cap.boundary_paths(&policy).unwrap()));let target=&paths[0].curves()[0];
   for reverse_circle in [false,true]{
    let circle=if reverse_circle{certified(original_circle.reversed(&policy).unwrap())}else{original_circle.clone()};
    for reverse_target in [false,true]{
     let target=if reverse_target{certified(target.reversed(&policy).unwrap())}else{target.clone()};
     for(a,b)in[(&circle,&target),(&target,&circle)]{
      println!("chart={shift} reverse_circle={reverse_circle} reverse_target={reverse_target} policy={policy:?}");
      let result=certified(a.intersect_curve(b,&policy).unwrap());assert!(result.is_complete(),"{:?}",result.blockers());assert!(result.overlaps().is_empty());let[contact]=result.contacts()else{panic!("one contact")};queries+=1;
      for(curve,location,other)in[(a,contact.first(),b),(b,contact.second(),a)]{
       let parameter=exact(location.parameter(&policy).unwrap());same(&certified(curve.point_at(&parameter,&policy).unwrap()),contact.point(),&policy);replays+=1;
       let(left,right)=certified(curve.split_at(parameter,&policy).unwrap());splits+=1;
       for piece in [left,right]{let child=certified(piece.intersect_curve(other,&policy).unwrap());assert!(child.is_complete(),"{:?}",child.blockers());let[child_contact]=child.contacts()else{panic!("one split contact")};same(child_contact.point(),contact.point(),&policy);child_queries+=1;}
      }
      assert!(certified(a.intersection_topology(b,&policy).unwrap()).result().is_complete());topologies+=1;
     }
    }
   }
   let region=certified(CurveRegion2::try_from_boundary_paths(std::slice::from_ref(&path),&policy).unwrap());
   println!("normalize filleted region chart={shift} policy={policy:?}");
   let region=certified(region.regularized_region(&policy).unwrap());
   println!("normalize cap chart={shift} policy={policy:?}");
   let cap=certified(cap.regularized_region(&policy).unwrap());
   for(a,b)in[(&region,&cap),(&cap,&region)]{
    println!("region chart={shift} policy={policy:?}");
    let _report=certified(a.intersect_region(b,&policy).unwrap());region_queries+=1;
    let intersection=certified(a.boolean_region(b,BooleanOp::Intersection,&policy).unwrap());
    let _reentry=certified(intersection.intersect_region(a,&policy).unwrap());boolean_reentries+=1;
   }
  }
 }
 assert_eq!((queries,replays,topologies,splits,child_queries,region_queries,boolean_reentries),(48,96,48,96,192,12,12));
 println!("{{\"queries\":{queries},\"point_replays\":{replays},\"topologies\":{topologies},\"splits\":{splits},\"child_queries\":{child_queries},\"region_queries\":{region_queries},\"boolean_reentries\":{boolean_reentries}}}");
}
