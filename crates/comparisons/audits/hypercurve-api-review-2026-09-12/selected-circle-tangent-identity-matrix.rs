use hypercurve::*;
fn q(n: i32,d: i32)->Real{(Real::from(n)/Real::from(d)).unwrap()}
fn exact<T>(v:Classification<T>)->T{match v{Classification::Decided(v)=>v,Classification::Uncertain(e)=>panic!("{e:?}")}}
fn certified<T>(v:CurveOutcome<T>)->T{assert_eq!(v.certainty,CurveCertainty::Certified);v.value}
fn same(a:&CurvePoint2,b:&CurvePoint2,policy:&CurveContext){assert!(exact(certified(a.coincides_with(b,policy))));}
fn main(){
 let(mut queries,mut empty,mut replays,mut topologies)=(0,0,0,0);
 for policy in [CurveContext::STRICT,CurveContext::APPROXIMATE_512]{
  for [a,b,c,d,scale] in [[1,0,0,1,1],[0,-1,1,0,1],[-1,0,0,1,1],[3,-4,4,3,5],[3,4,4,-3,5]]{
   let point=|x:Real,y:Real|Point2::new(Real::from(a)*&x+Real::from(b)*&y+Real::from(2),Real::from(c)*x+Real::from(d)*y-Real::from(3));
   let p=|x:i32,y:i32|point(x.into(),y.into());
   let path=CurvePath2::try_new(vec![LineSeg2::try_new(p(-4,0),p(0,0)).unwrap().into(),QuadraticBezier2::new(p(0,0),p(0,1),p(1,2)).into()]).unwrap();
   let CurveCornerSolutions2::Unique(path)=certified(path.fillet_vertex_by_radius(1,q(scale,4),CurveCornerMode2::TrimOnly,&policy).unwrap()) else{panic!("unique fillet")};
   let source=&path.curves()[1];let endpoint=source.start();
   for reversed in [false,true]{
    let circle=if reversed{certified(source.reversed(&policy).unwrap())}else{source.clone()};
    for line in [Curve2::from(LineSeg2::try_new(p(-1,0),p(1,0)).unwrap()),Curve2::from(LineSeg2::try_new(p(2,0),p(-2,0)).unwrap()),Curve2::from(QuadraticBezier2::new(p(-3,0),p(-1,0),p(1,0)))]{
     for(first,second)in[(&circle,&line),(&line,&circle)]{
      let result=certified(first.intersect_curve(second,&policy).unwrap());assert!(result.is_complete(),"{:?}",result.blockers());
      let[contact]=result.contacts()else{panic!("one tangent contact")};assert_eq!(contact.tangent_cross_sign(),Some(RealSign::Zero));assert!(!contact.is_certified_transverse());same(contact.point(),&endpoint,&policy);
      queries+=1;
      for(curve,location)in[(first,contact.first()),(second,contact.second())]{let parameter=exact(location.parameter(&policy).unwrap());let point=certified(curve.point_at(&parameter,&policy).unwrap());same(&point,contact.point(),&policy);replays+=1;}
      let topology=certified(first.intersection_topology(second,&policy).unwrap());assert!(topology.result().is_complete());topologies+=1;
     }
    }
    // The supporting circle's opposite tangent cannot become this endpoint.
    let opposite=Curve2::from(LineSeg2::try_new(point((-1).into(),q(1,2)),point(1.into(),q(1,2))).unwrap());
    for(first,second)in[(&circle,&opposite),(&opposite,&circle)]{
     let result=certified(first.intersect_curve(second,&policy).unwrap());assert!(result.is_complete(),"{:?}",result.blockers());assert!(result.contacts().is_empty());assert!(result.overlaps().is_empty());empty+=1;
    }
   }
  }
 }
 println!("{{\"tangent_queries\":{queries},\"opposite_tangent_exclusions\":{empty},\"point_replays\":{replays},\"topologies\":{topologies}}}");
}
