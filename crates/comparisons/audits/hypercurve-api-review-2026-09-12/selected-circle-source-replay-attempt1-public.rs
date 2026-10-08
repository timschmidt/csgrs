use hypercurve::*;
fn p(x:i32,y:i32)->Point2{Point2::from_values(x,y)}
fn q(n:i32,d:i32)->Real{(Real::from(n)/Real::from(d)).unwrap()}
#[track_caller]
fn exact<T>(v:Classification<T>)->T{match v{Classification::Decided(v)=>v,Classification::Uncertain(r)=>panic!("{r:?}")}}
#[track_caller]
fn certified<T>(v:CurveOutcome<T>)->T{assert_eq!(v.certainty,CurveCertainty::Certified);v.value}
#[track_caller]
fn same(a:&CurvePoint2,b:&CurvePoint2,policy:&CurveContext){assert!(exact(certified(a.coincides_with(b,policy))));}
fn main(){
 let(mut queries,mut replays,mut comparisons,mut orders,mut topologies)=(0,0,0,0,0);
 for policy in [CurveContext::STRICT,CurveContext::APPROXIMATE_512]{
  let path=CurvePath2::try_new(vec![LineSeg2::try_new(p(-4,0),p(0,0)).unwrap().into(),QuadraticBezier2::new(p(0,0),p(0,1),p(1,2)).into()]).unwrap();
  let CurveCornerSolutions2::Unique(path)=certified(path.fillet_vertex_by_radius(1,q(1,4),CurveCornerMode2::TrimOnly,&policy).unwrap()) else{panic!("unique fillet")};
  let circle=&path.curves()[1];
  let controls=[p(0,0),Point2::new(Real::zero(),q(1,2)),p(-1,1)];
  let knots=vec![Real::zero(),Real::zero(),Real::zero(),Real::one(),Real::one(),Real::one()];
  let curves=vec![
   Curve2::from(QuadraticBezier2::new(controls[0].clone(),controls[1].clone(),controls[2].clone())),
   Curve2::from(CubicBezier2::new(p(0,0),Point2::new(Real::zero(),q(1,3)),Point2::new(q(-1,3),q(2,3)),p(-1,1))),
   Curve2::from(RationalBezier2::try_new(controls.to_vec(),vec![Real::one();3]).unwrap().elevated_to_degree(5).unwrap()),
   certified(Curve2::try_polynomial_bspline(2,controls.to_vec(),knots.clone(),&policy).unwrap()),
   certified(Curve2::try_nurbs(2,controls.to_vec(),vec![Real::one();3],knots,&policy).unwrap()),
  ];
  for source in curves {
   println!("family={:?} policy={policy:?}",source.family());
   for reverse_circle in [false,true]{
    let circle=if reverse_circle{certified(circle.reversed(&policy).unwrap())}else{circle.clone()};
    for reverse_source in [false,true]{
     let source=if reverse_source{certified(source.reversed(&policy).unwrap())}else{source.clone()};
     for(first,second)in[(&circle,&source),(&source,&circle)]{
      let result=certified(first.intersect_curve(second,&policy).unwrap());assert!(result.is_complete(),"{:?}",result.blockers());
      let[contact]=result.contacts()else{panic!("one crossing")};assert!(contact.is_certified_transverse());queries+=1;
      for(curve,location)in[(first,contact.first()),(second,contact.second())]{
       let parameter=exact(location.parameter(&policy).unwrap());let point=certified(curve.point_at(&parameter,&policy).unwrap());replays+=1;
       for(a,b)in[(&point,contact.point()),(contact.point(),&point)]{
        same(a,b,&policy);comparisons+=1;
        for axis in [Axis2::X,Axis2::Y]{assert_eq!(exact(certified(a.compare_coordinate(b,axis,&policy).unwrap())),std::cmp::Ordering::Equal);orders+=1;}
       }
      }
      assert!(!exact(certified(contact.point().coincides_with(&source.start(),&policy))));
      let topology=certified(first.intersection_topology(second,&policy).unwrap());assert!(topology.result().is_complete());topologies+=1;
     }
    }
   }
  }
 }
 println!("{{\"queries\":{queries},\"point_replays\":{replays},\"equality_queries\":{comparisons},\"coordinate_orders\":{orders},\"topologies\":{topologies}}}");
}
