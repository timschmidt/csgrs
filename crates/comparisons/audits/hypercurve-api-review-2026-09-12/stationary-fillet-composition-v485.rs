use hypercurve::{BooleanOp, CircularArc2, Classification, CubicBezier2, Curve2, CurveCertainty, CurveContext, CurveCornerMode2, CurveFillet2, CurveFilletContact2, CurvePath2, CurveRegion2, CurveRegionLoopRole, FillRule, LineSeg2, OffsetCornerStyle2, Point2, Real};
fn q(n:i64,d:i64)->Real{(Real::from(n)/Real::from(d)).unwrap()}
fn p(x:i64,y:i64)->Point2{Point2::from_values(x,y)}
fn line(a:Point2,b:Point2)->Curve2{LineSeg2::try_new(a,b).unwrap().into()}
fn region(path:CurvePath2,policy:&CurveContext)->CurveRegion2 {
 let result=CurveRegion2::try_from_boundary_paths_with_loop_semantics(&[path],&[CurveRegionLoopRole::Material],&[FillRule::NonZero],policy).unwrap_or_else(|_|panic!("exact fillet path must enter region topology"));
 assert!(result.certainty==CurveCertainty::Certified);result.into_value()
}
fn regions(policy:CurveContext)->(CurveRegion2,CurveRegion2){
 let source=CubicBezier2::new(p(1,-1),Point2::new(q(-1,3),Real::one()),Point2::new(q(-1,3),-Real::one()),p(1,1));
 let path=CurvePath2::try_new(vec![source.into(),line(p(1,1),p(-3,-2))]).unwrap();
 let mut request=CurveFillet2::new(Real::one());request.center=Some(p(0,-1).into());request.contacts[0]=Some(CurveFilletContact2::Parameter(q(1,2).into()));
 let outcome=path.fillet_vertex(1,&request,CurveCornerMode2::TrimOnly,&policy).unwrap_or_else(|_|panic!("stationary fillet must close"));assert!(outcome.certainty==CurveCertainty::Certified);
 let solution=outcome.into_value().into_solutions().pop().unwrap();
 let mut curves=solution.curves().to_vec();curves.push(line(p(-3,-2),p(1,-1)));let actual=region(CurvePath2::try_new(curves).unwrap(),&policy);
 let contact=Point2::new(q(-3,5),q(-1,5));
 let expected=region(CurvePath2::try_new(vec![CubicBezier2::new(p(1,-1),Point2::new(q(1,3),Real::zero()),p(0,0),p(0,0)).into(),CircularArc2::try_from_center(p(0,0),contact.clone(),p(0,-1),false).unwrap().into(),line(contact,p(-3,-2)),line(p(-3,-2),p(1,-1))]).unwrap(),&policy);
 (actual,expected)
}
fn compare(actual:&CurveRegion2,expected:&CurveRegion2,policy:&CurveContext){
 let xor=actual.boolean_region(expected,BooleanOp::Xor,policy).unwrap_or_else(|_|panic!("independent exact regions must compare"));assert!(xor.certainty==CurveCertainty::Certified);assert!(xor.value.is_empty());
}
fn topology(policy:CurveContext){let(actual,expected)=regions(policy);compare(&actual,&expected,&policy);let paths=actual.boundary_paths(&policy).unwrap();assert!(paths.certainty==CurveCertainty::Certified);let Classification::Decided(paths)=paths.value else{panic!("boundary path remains exact")};assert_eq!(paths.len(),1);compare(&region(paths[0].clone(),&policy),&expected,&policy);}
fn offset(policy:CurveContext){let(actual,expected)=regions(policy);let offset=|region:&CurveRegion2|{let outcome=region.offset(q(1,20),&OffsetCornerStyle2::Round,&policy).unwrap_or_else(|_|panic!("offset of stationary fillet must close"));assert!(outcome.certainty==CurveCertainty::Certified);outcome.into_value()};compare(&offset(&actual),&offset(&expected),&policy);}
#[test]fn stationary_fillet_region_strict(){topology(CurveContext::STRICT)}
#[test]fn stationary_fillet_region_approximate(){topology(CurveContext::APPROXIMATE_512)}
#[test]fn stationary_fillet_offset_strict(){offset(CurveContext::STRICT)}
#[test]fn stationary_fillet_offset_approximate(){offset(CurveContext::APPROXIMATE_512)}
