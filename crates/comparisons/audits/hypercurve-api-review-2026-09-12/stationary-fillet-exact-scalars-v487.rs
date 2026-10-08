use hypercurve::{Classification,CubicBezier2,CurveCertainty,CurveContext,CurveCornerMode2,CurveFillet2,CurveFilletContact2,CurvePath2,LineSeg2,Point2,Real};
fn q(n:i64,d:i64)->Real{(Real::from(n)/Real::from(d)).unwrap()}
fn check(policy:CurveContext){
 let scale=Real::from(2).sqrt().unwrap();let tx=Real::from(3).sqrt().unwrap();let ty=Real::from(5).sqrt().unwrap();
 let a=&scale*q(3,5);let b=&scale*q(4,5);
 let point=|x:Real,y:Real|Point2::new(&tx+&a*&x-&b*&y,&ty+&b*x+&a*y);
 for reversed in [false,true]{for mode in [CurveCornerMode2::TrimOnly,CurveCornerMode2::TrimOrExtend]{
  let source=CubicBezier2::new(point(Real::one(),-Real::one()),point(q(-1,3),Real::one()),point(q(-1,3),-Real::one()),point(Real::one(),Real::one()));
  let line=LineSeg2::try_new(point(Real::one(),Real::one()),point(Real::from(-3),Real::from(-2))).unwrap();
  let path=CurvePath2::try_new(vec![source.into(),line.into()]).unwrap();let path=if reversed{path.reversed(&policy).unwrap().into_value()}else{path};
  let mut request=CurveFillet2::new(scale.clone());request.center=Some(point(Real::zero(),-Real::one()).into());request.contacts[usize::from(reversed)]=Some(CurveFilletContact2::Parameter(q(1,2).into()));
  let outcome=path.fillet_vertex(1,&request,mode,&policy).unwrap_or_else(|_|panic!("one-sided exact frames must not require rational coefficients"));assert!(outcome.certainty==CurveCertainty::Certified);let solutions=outcome.into_value();assert_eq!(solutions.candidate_count(),1);
  let solution=solutions.into_solutions().pop().unwrap();assert_eq!(solution.curves().len(),3);let arc=&solution.curves()[1];let contacts=[point(Real::zero(),Real::zero()),point(q(-3,5),q(-1,5))];
  for (actual,expected)in[(arc.start(),&contacts[usize::from(reversed)]),(arc.end(),&contacts[usize::from(!reversed)])]{let result=actual.coincides_with(&expected.clone().into(),&policy);assert!(result.certainty==CurveCertainty::Certified && result.value==Classification::Decided(true));}
 }}
}
#[test]fn stationary_contact_exact_scalars_strict(){check(CurveContext::STRICT)}
#[test]fn stationary_contact_exact_scalars_approximate(){check(CurveContext::APPROXIMATE_512)}
