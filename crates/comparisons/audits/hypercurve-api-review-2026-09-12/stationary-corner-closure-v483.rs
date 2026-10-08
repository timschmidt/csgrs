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
fn check_mode(kind:u8,policy:CurveContext,reversed:bool,mode:CurveCornerMode2){
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
    let path=if reversed { path.reversed(&policy).unwrap().into_value() } else { path };
    let (contact0,contact1)=if reversed {(contact1,contact0)}else{(contact0,contact1)};
    let outcome=match path.fillet_vertex(1,&request,mode,&policy){
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
fn check(kind:u8,policy:CurveContext){check_reversed(kind,policy,false)}
#[test]fn stationary_reparameterization_reversed_strict(){check_reversed(1,CurveContext::STRICT,true)}
#[test]fn stationary_reparameterization_reversed_approximate(){check_reversed(1,CurveContext::APPROXIMATE_512,true)}
#[test]fn one_sided_cusp_reversed_strict(){check_reversed(2,CurveContext::STRICT,true)}
#[test]fn one_sided_cusp_reversed_approximate(){check_reversed(2,CurveContext::APPROXIMATE_512,true)}
#[test]fn regular_parabola_strict(){check(0,CurveContext::STRICT)}
#[test]fn stationary_reparameterization_strict(){check(1,CurveContext::STRICT)}
#[test]fn one_sided_cusp_strict(){check(2,CurveContext::STRICT)}
#[test]fn regular_parabola_approximate(){check(0,CurveContext::APPROXIMATE_512)}
#[test]fn stationary_reparameterization_approximate(){check(1,CurveContext::APPROXIMATE_512)}
#[test]fn one_sided_cusp_approximate(){check(2,CurveContext::APPROXIMATE_512)}

fn interior_stationary_contact_mode(policy:CurveContext,reversed:bool,mode:CurveCornerMode2){
    // C(t)=(t^2,t^3), -1<=t<=1, with public t=2*u-1. The
    // previous retained branch has tangent -x at u=1/2, although C'=0.
    let source=CubicBezier2::new(Point2::from_values(1,-1),point(q(-1,3),Real::one()),point(q(-1,3),-Real::one()),Point2::from_values(1,1));
    let next=LineSeg2::try_new(Point2::from_values(1,1),Point2::from_values(-3,-2)).unwrap();
    let path=CurvePath2::try_new(vec![source.into(),next.into()]).unwrap();
    let mut request=CurveFillet2::new(Real::one());
    request.center=Some(Point2::from_values(0,-1).into());
    request.contacts[usize::from(reversed)]=Some(CurveFilletContact2::Parameter(q(1,2).into()));
    let path=if reversed { path.reversed(&policy).unwrap().into_value() } else { path };
    let outcome=match path.fillet_vertex(1,&request,mode,&policy){
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
    let contacts=[Point2::from_values(0,0),point(q(-3,5),q(-1,5))];
    let first=arc.start().coincides_with(&contacts[usize::from(reversed)].clone().into(),&policy);
    let second=arc.end().coincides_with(&contacts[usize::from(!reversed)].clone().into(),&policy);
    assert!(first.certainty==CurveCertainty::Certified && first.value==Classification::Decided(true));
    assert!(second.certainty==CurveCertainty::Certified && second.value==Classification::Decided(true));
}
fn interior_stationary_contact(policy:CurveContext){interior_stationary_contact_reversed(policy,false)}
#[test]fn interior_stationary_contact_reversed_strict(){interior_stationary_contact_reversed(CurveContext::STRICT,true)}
#[test]fn interior_stationary_contact_reversed_approximate(){interior_stationary_contact_reversed(CurveContext::APPROXIMATE_512,true)}
#[test]fn interior_stationary_contact_strict(){interior_stationary_contact(CurveContext::STRICT)}
#[test]fn interior_stationary_contact_approximate(){interior_stationary_contact(CurveContext::APPROXIMATE_512)}

fn opposite_stationary_sheet_mode(policy:CurveContext,reversed:bool,mode:CurveCornerMode2){
    // C(t)=(t²,t³), -1<=t<=2. At u=1/3 the retained previous
    // branch approaches the origin in direction -x. The circle centered at
    // (0,1) has the opposite source normal there. Its other contact is the
    // strictly interior point (-4/5,8/5) on the line, so endpoint rejection
    // cannot substitute for the missing normal-sheet decision.
    let curve=CubicBezier2::new(Point2::from_values(1,-1),Point2::from_values(-1,2),Point2::from_values(0,-4),Point2::from_values(4,8));
    let line=LineSeg2::try_new(Point2::from_values(4,8),Point2::from_values(-2,0)).unwrap();
    let path=CurvePath2::try_new(vec![curve.into(),line.into()]).unwrap();
    let path=if reversed{path.reversed(&policy).unwrap().into_value()}else{path};
    let mut request=CurveFillet2::new(Real::one());request.center=Some(Point2::from_values(0,1).into());
    request.contacts[usize::from(reversed)]=Some(CurveFilletContact2::Parameter(if reversed{q(2,3)}else{q(1,3)}.into()));
    let outcome=path.fillet_vertex(1,&request,mode,&policy).unwrap_or_else(|_|panic!("opposite one-sided normal must be decidable"));
    assert!(outcome.certainty==CurveCertainty::Certified);
    assert_eq!(outcome.into_value().candidate_count(),0);
}
#[test]fn opposite_stationary_sheet_strict(){opposite_stationary_sheet(CurveContext::STRICT,false)}
#[test]fn opposite_stationary_sheet_approximate(){opposite_stationary_sheet(CurveContext::APPROXIMATE_512,false)}
#[test]fn opposite_stationary_sheet_reversed_strict(){opposite_stationary_sheet(CurveContext::STRICT,true)}
#[test]fn opposite_stationary_sheet_reversed_approximate(){opposite_stationary_sheet(CurveContext::APPROXIMATE_512,true)}

fn check_reversed(kind:u8,policy:CurveContext,reversed:bool){
    for mode in [CurveCornerMode2::TrimOnly,CurveCornerMode2::TrimOrExtend]{check_mode(kind,policy,reversed,mode)}
}
fn interior_stationary_contact_reversed(policy:CurveContext,reversed:bool){
    for mode in [CurveCornerMode2::TrimOnly,CurveCornerMode2::TrimOrExtend]{interior_stationary_contact_mode(policy,reversed,mode)}
}
fn opposite_stationary_sheet(policy:CurveContext,reversed:bool){
    for mode in [CurveCornerMode2::TrimOnly,CurveCornerMode2::TrimOrExtend]{opposite_stationary_sheet_mode(policy,reversed,mode)}
}
