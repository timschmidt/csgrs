use hypercurve::{Classification, Curve2, CurveCertainty, CurveContext, CurveCornerMode2, CurveFamily2, CurveFillet2, CurveFilletContact2, CurvePath2, ExactCurveError, LineSeg2, Point2, QuadraticBezier2, Real};
fn q(n:i32,d:i32)->Real {(Real::from(n)/Real::from(d)).unwrap()}
fn main() {
    let p=Point2::from_values;
    for (name,policy) in [("strict",CurveContext::STRICT),("approximate",CurveContext::APPROXIMATE_512)] {
        let original=CurvePath2::try_new(vec![
            LineSeg2::try_new(p(-4,0),p(0,0)).unwrap().into(),
            Curve2::from(QuadraticBezier2::new(p(0,0),p(0,1),p(1,2))),
        ]).unwrap();
        let generated=original.fillet_vertex(1,&CurveFillet2::new(Real::one()),CurveCornerMode2::TrimOnly,&policy).unwrap();
        assert_eq!(generated.certainty,CurveCertainty::Certified);
        assert_eq!(generated.value.candidate_count(),1);
        let circle=generated.value.solutions()[0].curves().iter().find(|c| c.family()==CurveFamily2::CircularArc && c.geometry().is_none()).expect("retained generated fillet circle");
        let source=circle.subcurve(q(1,16).into(),q(3,16).into(),&policy).unwrap().value;
        let (first,second)=source.split_at(q(1,8).into(),&policy).unwrap().value;
        let path=CurvePath2::try_new(vec![first,second]).unwrap();
        let contacts=[q(3,32),q(5,32)].map(|t| circle.point_at(&t.into(),&policy).unwrap().value);
        for (point,curve) in contacts.iter().zip(path.curves()) {
            assert_eq!(curve.point_at(&(if curve==&path.curves()[0] {q(3,32)} else {q(5,32)}).into(),&policy).unwrap().value.same_point(point,&policy),Classification::Decided(true));
        }
        println!("policy={name} retained_contacts={}",contacts.iter().filter(|p| p.coordinates().is_none()).count());
        for reversed in [false,true] {
            let path=if reversed {path.reversed(&policy).unwrap().value} else {path.clone()};
            for constraint in ["free","point","parameter"] {
                let mut request=CurveFillet2::new(Real::one());
                if constraint=="point" {let mut points=contacts.clone();if reversed {points.reverse();}request.contacts=points.map(|point| Some(CurveFilletContact2::Point(point)));}
                if constraint=="parameter" {let mut parameters=[q(3,32),q(5,32)];if reversed {parameters.reverse();}request.contacts=parameters.map(|parameter| Some(CurveFilletContact2::Parameter(parameter.into())));}
                for mode in [CurveCornerMode2::TrimOnly,CurveCornerMode2::TrimOrExtend] {
                    match path.fillet_vertex(1,&request,mode,&policy) {
                        Ok(result)=>println!("policy={name} reversed={reversed} constraint={constraint} mode={mode:?} count={} reason={:?}",result.value.candidate_count(),result.value.no_solution_reason()),
                        Err(ExactCurveError::Blocked(blocker))=>println!("policy={name} reversed={reversed} constraint={constraint} mode={mode:?} blocked={:?}",blocker.reason()),
                        Err(ExactCurveError::Invalid {cause,..})=>println!("policy={name} reversed={reversed} constraint={constraint} mode={mode:?} invalid={cause}"),
                    }
                }
            }
        }
    }
}
