use hypercurve::{BezierParameterPolynomial, CircularArc2, Classification, Curve2, CurveContext, CurveCornerMode2, CurveFillet2, CurveFilletContact2, CurvePath2, ExactCurveError, Point2, RationalBezier2, Real};
fn main() {
    let p = Point2::from_values;
    let original = CurvePath2::try_new(vec![
        CircularArc2::try_from_center(p(1,0), p(0,1), p(0,0), false).unwrap().into(),
        CircularArc2::try_from_center(p(0,1), p(-1,0), p(0,0), false).unwrap().into(),
    ]).unwrap();
    for (name, policy) in [("strict", CurveContext::STRICT), ("approximate", CurveContext::APPROXIMATE_512)] {
        let Classification::Decided(polynomial) = BezierParameterPolynomial::try_new_power_basis(vec![-Real::one(),Real::zero(),Real::from(2)], &policy).unwrap() else { panic!("polynomial"); };
        let Classification::Decided(roots) = polynomial.isolate_unit_interval_roots(&policy).unwrap() else { panic!("root"); };
        assert_eq!(roots.len(),1);
        let contacts = [1,-1].map(|sign| {
            let source: Curve2 = RationalBezier2::try_new(vec![p(sign,0),p(sign,1),p(0,1)],vec![Real::one(),Real::one(),Real::from(2)]).unwrap().into();
            let point = source.point_at(&roots[0].clone().into(), &policy).unwrap().value;
            assert!(point.coordinates().is_none()); point
        });
        for reversed in [false,true] {
            let path = if reversed { original.reversed(&policy).unwrap().value } else { original.clone() };
            let mut contacts = contacts.clone(); if reversed { contacts.reverse(); }
            let mut request=CurveFillet2::new(Real::one());
            request.contacts=contacts.map(|point| Some(CurveFilletContact2::Point(point)));
            for mode in [CurveCornerMode2::TrimOnly,CurveCornerMode2::TrimOrExtend] {
                match path.fillet_vertex(1,&request,mode,&policy) {
                    Ok(outcome)=>println!("policy={name} reversed={reversed} mode={mode:?} count={} reason={:?}",outcome.value.candidate_count(),outcome.value.no_solution_reason()),
                    Err(ExactCurveError::Blocked(blocker))=>println!("policy={name} reversed={reversed} mode={mode:?} blocked={:?}",blocker.reason()),
                    Err(ExactCurveError::Invalid { cause, .. })=>println!("policy={name} reversed={reversed} mode={mode:?} invalid={cause}"),
                }
            }
        }
    }
}
