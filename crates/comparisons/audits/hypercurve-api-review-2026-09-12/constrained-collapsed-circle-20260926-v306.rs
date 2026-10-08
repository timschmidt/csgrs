use hypercurve::{
    CircularArc2, CurveContext, CurveCornerMode2, CurveFillet2, CurveFilletContact2,
    CurvePath2, Point2, Real,
};

fn q(n: i64, d: i64) -> Real {
    (Real::from(n) / Real::from(d)).unwrap()
}

fn main() {
    let center = Point2::from_values(0, 0);
    let path = CurvePath2::try_new(vec![
        CircularArc2::try_from_center(
            Point2::from_values(1, 0),
            Point2::from_values(0, 1),
            center.clone(),
            false,
        )
        .unwrap()
        .into(),
        CircularArc2::try_from_center(
            Point2::from_values(0, 1),
            Point2::from_values(-1, 0),
            center.clone(),
            false,
        )
        .unwrap()
        .into(),
    ])
    .unwrap();
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for constrained in [false, true] {
            let mut request = CurveFillet2::new(Real::one());
            request.center = Some(center.clone().into());
            if constrained {
                request.contacts = [
                    Some(CurveFilletContact2::Point(
                        Point2::new(q(3, 5), q(4, 5)).into(),
                    )),
                    Some(CurveFilletContact2::Point(
                        Point2::new(q(-3, 5), q(4, 5)).into(),
                    )),
                ];
            }
            let result = path
                .fillet_vertex(1, &request, CurveCornerMode2::TrimOnly, &policy)
                .unwrap();
            println!(
                "contacts={} count={} reason={:?} certainty={:?}",
                constrained,
                result.value.candidate_count(),
                result.value.no_solution_reason(),
                result.certainty,
            );
        }
    }
}
