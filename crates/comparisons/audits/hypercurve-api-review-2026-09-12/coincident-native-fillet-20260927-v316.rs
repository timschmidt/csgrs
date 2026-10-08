use hypercurve::{CircularArc2, Curve2, CurveContext, CurveCornerMode2, CurveFillet2,
    CurveFilletContact2, CurvePath2, ExactCurveError, LineSeg2, Point2, Real};

fn main() {
    let p = Point2::from_values;
    // These two curves form three sides of a rectangle. The second curve
    // is one authored degree-one NURBS with a vertical and a westward span.
    let next = Curve2::try_nurbs(
        1,
        vec![p(0, 0), p(0, 2), p(-3, 2)],
        vec![Real::one(); 3],
        [0, 0, 1, 2, 2].into_iter().map(Real::from).collect(),
        &CurveContext::STRICT,
    ).unwrap().value;
    let source = CurvePath2::try_new(vec![
        LineSeg2::try_new(p(-3, 0), p(0, 0)).unwrap().into(), next,
    ]).unwrap();

    // For every -3 < x < 0, center (x,1) gives a unit semicircle with
    // eastward tangent at (x,0) and westward tangent at (x,2). Both contacts
    // lie strictly inside their authored curves. This is one exact witness.
    let witness = CurvePath2::try_new(vec![
        LineSeg2::try_new(p(-3, 0), p(-1, 0)).unwrap().into(),
        CircularArc2::try_from_center(p(-1, 0), p(-1, 2), p(-1, 1), false)
            .unwrap().into(),
        LineSeg2::try_new(p(-1, 2), p(-3, 2)).unwrap().into(),
    ]).unwrap();
    assert_eq!(witness.curves().len(), 3);
    println!("independent_exact_semicircle_witness=true");

    for (name, policy) in [("strict", CurveContext::STRICT), ("approximate", CurveContext::APPROXIMATE_512)] {
        for reversed in [false, true] {
            let path = if reversed { source.reversed(&policy).unwrap().value } else { source.clone() };
            for mode in ["radius", "center", "contact", "both"] {
                let mut request = CurveFillet2::new(Real::one());
                if mode == "center" || mode == "both" {
                    request.center = Some(p(-1, 1).into());
                }
                if mode == "contact" || mode == "both" {
                    request.contacts[usize::from(!reversed)] =
                        Some(CurveFilletContact2::Point(p(-1, 2).into()));
                }
                match path.fillet_vertex(1, &request, CurveCornerMode2::TrimOnly, &policy) {
                    Ok(outcome) => println!("policy={name} reversed={reversed} request={mode} count={} reason={:?}",
                        outcome.value.candidate_count(), outcome.value.no_solution_reason()),
                    Err(ExactCurveError::Blocked(blocker)) => println!("policy={name} reversed={reversed} request={mode} blocked={:?}", blocker.reason()),
                    Err(ExactCurveError::Invalid { cause, .. }) => println!("policy={name} reversed={reversed} request={mode} invalid={cause}"),
                }
            }
        }
    }
}
