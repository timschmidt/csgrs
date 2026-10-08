use hypercurve::{BooleanOp, Classification, Curve2, CurveCertainty, CurveContext, CurveCornerMode2, CurveCornerSolutions2, CurveOutcome, CurvePath2, CurveRegion2, CurveRegionLoopRole, FillRule, LineSeg2, Point2, QuadraticBezier2, Real, RegionPointLocation};
fn p(x:i64,y:i64)->Point2 { Point2::from_values(x,y) }
fn line(a:Point2,b:Point2)->Curve2 { LineSeg2::try_new(a,b).unwrap().into() }
fn certified<T>(outcome:CurveOutcome<T>)->T { assert_eq!(outcome.certainty,CurveCertainty::Certified); outcome.value }
fn decided<T>(value:Classification<T>)->T { match value { Classification::Decided(v)=>v, Classification::Uncertain(r)=>panic!("uncertain: {r:?}") } }
fn region(path:CurvePath2, policy:&CurveContext)->CurveRegion2 { certified(CurveRegion2::try_from_boundary_paths_with_loop_semantics(&[path],&[CurveRegionLoopRole::Material],&[FillRule::NonZero],policy).unwrap()) }
fn main() {
 let mut failures=0;
 for policy in [CurveContext::STRICT,CurveContext::APPROXIMATE_512] {
  for double in [false,true] {
   println!("case {policy:?} double={double}");
   let started=std::time::Instant::now();
   let path=if double {
    CurvePath2::try_new(vec![QuadraticBezier2::new(p(-1,2),p(0,1),p(0,0)).into(),QuadraticBezier2::new(p(0,0),p(0,1),p(1,2)).into(),line(p(1,2),p(-1,2))]).unwrap()
   } else {
    CurvePath2::try_new(vec![line(p(-4,0),p(0,0)),QuadraticBezier2::new(p(0,0),p(0,1),p(1,2)).into(),line(p(1,2),p(-4,2)),line(p(-4,2),p(-4,0))]).unwrap()
   };
   let source=region(path,&policy);
   let CurveCornerSolutions2::Unique(chamfered)=certified(source.chamfer_loop_vertex_by_setbacks(0,1,Real::one(),Real::one(),CurveCornerMode2::TrimOnly,&policy).unwrap()) else {panic!("unique chamfer");};
   println!("chamfer generated {:?}",started.elapsed());
   // Q(t)=(t^2,2t), so unit setback gives t^4+4t^2=1.
   // The positive solution has t^2=sqrt(5)-2.
   let x=Real::from(5).sqrt().unwrap()-Real::from(2);
   let t=x.clone().sqrt().unwrap();
   let right=Point2::new(x.clone(),Real::from(2)*&t);
   let selected_contact=right.clone();
   let right_control=Point2::new(t.clone(),Real::one()+&t);
   let right_tail=Curve2::from(QuadraticBezier2::new(right.clone(),right_control,p(1,2)));
   let expected_path=if double {
    let left=Point2::new(-x,Real::from(2)*&t);
    CurvePath2::try_new(vec![QuadraticBezier2::new(p(-1,2),Point2::new(-t.clone(),Real::one()+&t),left.clone()).into(),line(left,right),right_tail,line(p(1,2),p(-1,2))]).unwrap()
   } else {
    CurvePath2::try_new(vec![line(p(-4,0),p(-1,0)),line(p(-1,0),right),right_tail,line(p(1,2),p(-4,2)),line(p(-4,2),p(-4,0))]).unwrap()
   };
   let expected_curves=expected_path.curves().to_vec();
   let expected=region(expected_path,&policy);
   println!("independent region admitted {:?}",started.elapsed());
   let paths=decided(certified(chamfered.boundary_paths(&policy).unwrap()));
   for (index, curve) in paths.iter().flat_map(CurvePath2::curves).enumerate() {
     println!("endpoint {index} {:?} {:?}", curve.start().coincides_with(&selected_contact.clone().into(), &policy), curve.end().coincides_with(&selected_contact.clone().into(), &policy));
   }
   for (i, first) in paths.iter().flat_map(CurvePath2::curves).enumerate() {
    for (j, second) in expected_curves.iter().enumerate() {
      match first.intersect_curve(second, &policy) {
       Ok(outcome)=> {
        let result=certified(outcome);
        for blocker in result.blockers() { println!("pair ({i},{j}) {:?}/{:?} blocker {:?}",first.family(),second.family(),blocker.kind()); }
        if !result.overlaps().is_empty() {println!("pair ({i},{j}) overlaps={}",result.overlaps().len());}
       },
       Err(error)=>println!("pair ({i},{j}) error {error:?}"),
      }
    }
   }
   let restored=certified(CurveRegion2::try_from_boundary_paths_with_loop_semantics(&paths,&[CurveRegionLoopRole::Material],&[FillRule::NonZero],&policy).unwrap());
   println!("boundary paths restored {:?}",started.elapsed());
   for (label,actual) in [("generated",&chamfered),("restored",&restored)] {
    let probe=if double {p(0,1)} else {p(-2,1)};
    assert_eq!(certified(actual.classify_point(&probe,&policy).unwrap()),Classification::Decided(RegionPointLocation::Inside));
    assert_eq!(certified(actual.classify_point(&p(0,0),&policy).unwrap()),Classification::Decided(RegionPointLocation::Outside));
    hyperreal::dispatch_trace::reset();
    let result=hyperreal::dispatch_trace::with_recording(|| actual.boolean_region(&expected,BooleanOp::Xor,&policy));
    for entry in hyperreal::dispatch_trace::take() { if entry.layer=="hypercurve" { println!("trace {label} {} {} {}",entry.operation,entry.path,entry.count); } }
    match result {
      Ok(outcome)=>{let xor=certified(outcome); assert!(xor.is_empty(),"{label} differs from independent geometry"); println!("{label} XOR empty {:?}",started.elapsed());},
      Err(error)=>{failures+=1; println!("{label} XOR failed: {error:?}");}
    }
   }
  }
 }
assert_eq!(failures,0);
}
