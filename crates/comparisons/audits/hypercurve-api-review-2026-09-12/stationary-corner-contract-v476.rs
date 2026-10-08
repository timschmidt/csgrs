use hypercurve::{
    Classification, CubicBezier2, Curve2, CurveCertainty, CurveContext, CurveCornerMode2,
    CurveFillet2, CurveFilletContact2, CurvePath2, ExactCurveError, LineSeg2, Point2, QuadraticBezier2,
    RationalBezier2, Real,
};
fn q(n:i64,d:i64)->Real{(Real::from(n)/Real::from(d)).unwrap()}
fn point(x:Real,y:Real)->Point2{Point2::new(x,y)}
fn source(kind:u8)->Curve2 {
    match kind {
        0=>QuadraticBezier2::new(Point2::from_values(0,0),point(q(1,2),Real::zero()),Point2::from_values(1,1)).into(),
        1=>RationalBezier2::try_new(vec![Point2::from_values(0,0),Point2::from_values(0,0),point(q(1,6),Real::zero()),point(q(1,2),Real::zero()),Point2::from_values(1,1)],vec![Real::one();5]).unwrap().into(),
        2=>CubicBezier2::new(Point2::from_values(0,0),Point2::from_values(0,0),point(q(1,3),Real::zero()),Point2::from_values(1,1)).into(),
        _=>unreachable!(),
    }
}
fn check(kind:u8,policy:CurveContext){
    // Kind 0 is (t,t^2), kind 1 is the same image (t^2,t^4), and
    // kind 2 is the one-sided cusp (t^2,t^3). Both stationary charts have
    // the exact right-hand tangent +x at their authored start.
    let (radius,cx,cy,x,y)=if kind==2 {(q(5,8),q(5,8),q(-3,8),q(1,4),q(1,8))}
        else{(q(15,16),q(15,16),q(-39,64),q(3,8),q(9,64))};
    let contact0=point(Real::zero(),cy.clone());let contact1=point(x,y);
    // Independently fixed rational circle: horizontal radial at the line
    // contact, and a (3,4,5) normal at the curved contact.
    let mut request=CurveFillet2::new(radius);request.center=Some(point(cx,cy).into());
    let path=CurvePath2::try_new(vec![LineSeg2::try_new(Point2::from_values(0,-2),Point2::from_values(0,0)).unwrap().into(),source(kind)]).unwrap();
    let outcome=match path.fillet_vertex(1,&request,CurveCornerMode2::TrimOnly,&policy){
        Ok(outcome)=>outcome,
        Err(ExactCurveError::Blocked(blocker))=>{eprintln!("blocked {:?}",blocker.reason());panic!("a rational tangent circle is independently known")},
        Err(_)=>panic!("unexpected corner-input error"),
    };
    assert!(outcome.certainty==CurveCertainty::Certified);
    let solutions=outcome.into_value();
    eprintln!("candidate_count={} no_solution={:?}",solutions.candidate_count(),solutions.no_solution_reason());
    assert_eq!(solutions.candidate_count(),1);
    let solution=solutions.into_solutions().pop().unwrap();assert_eq!(solution.curves().len(),3);
    let arc=&solution.curves()[1];
    let first=arc.start().coincides_with(&contact0.into(),&policy);
    let second=arc.end().coincides_with(&contact1.into(),&policy);
    assert!(first.certainty==CurveCertainty::Certified && first.value==Classification::Decided(true));
    assert!(second.certainty==CurveCertainty::Certified && second.value==Classification::Decided(true));
}
#[test]fn regular_parabola_strict(){check(0,CurveContext::STRICT)}
#[test]fn stationary_reparameterization_strict(){check(1,CurveContext::STRICT)}
#[test]fn one_sided_cusp_strict(){check(2,CurveContext::STRICT)}
#[test]fn regular_parabola_approximate(){check(0,CurveContext::APPROXIMATE_512)}
#[test]fn stationary_reparameterization_approximate(){check(1,CurveContext::APPROXIMATE_512)}
#[test]fn one_sided_cusp_approximate(){check(2,CurveContext::APPROXIMATE_512)}

fn interior_stationary_contact(policy:CurveContext){
    // C(t)=(t^2,t^3), -1<=t<=1, with public t=2*u-1. The
    // previous retained branch has tangent -x at u=1/2, although C'=0.
    let source=CubicBezier2::new(Point2::from_values(1,-1),point(q(-1,3),Real::one()),point(q(-1,3),-Real::one()),Point2::from_values(1,1));
    let next=LineSeg2::try_new(Point2::from_values(1,1),Point2::from_values(-3,-2)).unwrap();
    let path=CurvePath2::try_new(vec![source.into(),next.into()]).unwrap();
    let mut request=CurveFillet2::new(Real::one());
    request.center=Some(Point2::from_values(0,-1).into());
    request.contacts[0]=Some(CurveFilletContact2::Parameter(q(1,2).into()));
    let outcome=match path.fillet_vertex(1,&request,CurveCornerMode2::TrimOnly,&policy){
        Ok(outcome)=>outcome,
        Err(ExactCurveError::Blocked(blocker))=>{eprintln!("blocked {:?}",blocker.reason());panic!("one-sided contact has a rational normal and tangent circle")},
        Err(_)=>panic!("unexpected interior-contact input error"),
    };
    assert!(outcome.certainty==CurveCertainty::Certified);
    let solutions=outcome.into_value();
    eprintln!("candidate_count={} no_solution={:?}",solutions.candidate_count(),solutions.no_solution_reason());
    assert_eq!(solutions.candidate_count(),1);
    let solution=solutions.into_solutions().pop().unwrap();assert_eq!(solution.curves().len(),3);
    let arc=&solution.curves()[1];
    let first=arc.start().coincides_with(&Point2::from_values(0,0).into(),&policy);
    let second=arc.end().coincides_with(&point(q(-3,5),q(-1,5)).into(),&policy);
    assert!(first.certainty==CurveCertainty::Certified && first.value==Classification::Decided(true));
    assert!(second.certainty==CurveCertainty::Certified && second.value==Classification::Decided(true));
}
#[test]fn interior_stationary_contact_strict(){interior_stationary_contact(CurveContext::STRICT)}
#[test]fn interior_stationary_contact_approximate(){interior_stationary_contact(CurveContext::APPROXIMATE_512)}
