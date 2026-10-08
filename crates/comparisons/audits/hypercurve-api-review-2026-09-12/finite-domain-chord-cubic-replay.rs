use hypercurve::*;
fn q(n:i32,d:i32)->Real {(Real::from(n)/Real::from(d)).unwrap()}
fn p(x:i32,y:i32)->Point2 {Point2::from_values(x,y)}
fn exact<T>(x:Classification<T>)->T {match x {Classification::Decided(x)=>x,Classification::Uncertain(e)=>panic!("{e:?}")}}
fn certified<T>(x:CurveOutcome<T>)->T {assert_eq!(x.certainty,CurveCertainty::Certified);x.value}
fn chord(a:Point2,b:Point2,policy:&CurveContext)->BezierSplitFragment2 {
 BezierSplitFragment2::AlgebraicChord(exact(BezierAlgebraicChord2::try_new(a.into(),b.into(),policy).unwrap()))
}
fn export(fragments:Vec<BezierSplitFragment2>,policy:&CurveContext)->Vec<Curve2> {
 let boundary=CurveRegionBoundaryLoop2::new(fragments,policy).unwrap();
 let region=CurveRegion2::try_new_with_loop_topology(vec![boundary],vec![CurveRegionLoopRole::Material],vec![FillRule::NonZero],vec![CurveBoundaryInteriorSide2::Left]).unwrap();
 exact(certified(region.boundary_paths(policy).unwrap()))[0].curves().to_vec()
}
fn chord_curve(a:Point2,b:Point2,policy:&CurveContext)->Curve2 {
 let c=Point2::new(a.x()-(b.y()-a.y()),a.y()+(b.x()-a.x()));
 export(vec![chord(a.clone(),b.clone(),policy),chord(b,c.clone(),policy),chord(c,a,policy)],policy).remove(0)
}
fn oriented(c:&Curve2,reverse:bool,policy:&CurveContext)->Curve2 {if reverse {certified(c.reversed(policy).unwrap())} else {c.clone()}}
fn same(a:&CurvePoint2,b:&CurvePoint2,policy:&CurveContext){assert_eq!(certified(a.coincides_with(b,policy)),Classification::Decided(true));}
fn main(){let(mut queries,mut replays,mut splits)=(0,0,0);
 for policy in [CurveContext::STRICT,CurveContext::APPROXIMATE_512]{
  let source=CubicBezier2::new(p(0,-2),Point2::new(q(1,3),(-2).into()),Point2::new(q(2,3),(-2).into()),p(1,-1));
  let start=source.point_at(Real::one());let end=source.point_at(Real::from(2));
  let fragment=BezierSplitFragment2::RetainedBezier{reversed:false,start:BezierParameter2::Exact(Real::one()),end:BezierParameter2::Exact(Real::from(2)),source_curve:BezierSubcurve2::Cubic(source),start_image:None,end_image:None};
  let source=export(vec![fragment,chord(end,start,&policy)],&policy).remove(0);
  let cutter=chord_curve(p(0,0),p(3,0),&policy);
  let polynomial=exact(BezierParameterPolynomial::try_new_power_basis(vec![(-2).into(),0.into(),0.into(),1.into()],&policy).unwrap());
  let roots=exact(polynomial.isolate_incident_ray_roots(&Real::one(),BezierParameterRayDirection2::Increasing,&policy).unwrap());assert_eq!(roots.len(),1);
  let root=CurveParameter2::from(roots[0].clone());
  for reversed in [false,true]{let source=oriented(&source,reversed,&policy);
   for (a,b) in [(&source,&cutter),(&cutter,&source)]{
    let result=certified(a.intersect_curve(b,&policy).unwrap());queries+=1;assert!(result.is_complete(),"{:?}",result.blockers());assert_eq!(result.contacts().len(),1);
    for contact in result.contacts(){for (curve,location) in [(a,contact.first()),(b,contact.second())]{let parameter=exact(location.parameter(&policy).unwrap());same(&certified(curve.point_at(&parameter,&policy).unwrap()),contact.point(),&policy);replays+=1;}}
   }
   let result=certified(source.intersect_curve(&cutter,&policy).unwrap());queries+=1;let parameter=exact(result.contacts()[0].first().parameter(&policy).unwrap());
   assert_eq!(certified(parameter.compare(&root,&policy).unwrap()),Classification::Decided(std::cmp::Ordering::Equal));
   let parts=certified(source.split_at(parameter,&policy).unwrap());splits+=1;
   for part in [parts.0,parts.1]{for _ in 0..4 {let result=certified(part.intersect_curve(&cutter,&policy).unwrap());queries+=1;assert!(result.is_complete(),"{:?}",result.blockers());assert_eq!(result.contacts().len(),1);}}
  }
 }
 println!("{{\"queries\":{queries},\"point_replays\":{replays},\"splits\":{splits}}}");
}
