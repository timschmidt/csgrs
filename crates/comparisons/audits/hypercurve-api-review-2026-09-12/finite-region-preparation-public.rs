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
 let mut failed=0;
 let mut queries=0;
 for policy in [CurveContext::STRICT,CurveContext::APPROXIMATE_512] {
  let vertical=Curve2::from(QuadraticBezier2::new(Point2::new(q(-1,4),Real::zero()),Point2::new(q(-1,4),q(1,2)),Point2::new(q(-1,4),Real::one())));
  for shift in [0,1,-2] {
   let region=cap(shift,&policy);
   let paths=exact(certified(region.boundary_paths(&policy).unwrap()));
   let boundary=&paths[0].curves()[0];
   for reversed in [false,true] {
    let source=if reversed {certified(vertical.reversed(&policy).unwrap())}else{vertical.clone()};
    for (a,b) in [(&source,boundary),(boundary,&source)] {
     let result=certified(a.intersect_curve(b,&policy).unwrap());assert!(result.is_complete());assert_eq!(result.contacts().len(),1);
     let c=&result.contacts()[0];
     for (curve,location) in [(a,c.first()),(b,c.second())] {
      let parameter=exact(location.parameter(&policy).unwrap());same(&certified(curve.point_at(&parameter,&policy).unwrap()),c.point(),&policy);
     }
     queries+=1;
    }
    let fragments=certified(source.trim_inside_region_with_parameters(&region,&policy).unwrap());
    let ranges:Vec<_>=fragments.iter().map(|f|f.represented_parameter_range()).collect();
    let expected=if reversed {(q(1,2),q(3,4))}else{(q(1,4),q(1,2))};
    println!("shift={shift} reversed={reversed} policy={policy:?} ranges={ranges:?}");
    if ranges!=vec![Some(expected)] {failed+=1;}
   }
  }
 }
 println!("queries={queries} failures={failed}");assert_eq!(failed,0);
}
