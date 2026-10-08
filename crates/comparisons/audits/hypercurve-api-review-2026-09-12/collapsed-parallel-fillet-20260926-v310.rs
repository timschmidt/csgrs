use hypercurve::{CircularArc2, CurveContext, CurveCornerMode2, CurveFillet2, CurveFilletContact2,
    CurvePath2, ExactCurveError, Point2, QuadraticBezier2, Real};
fn q(n:i64,d:i64)->Real{(Real::from(n)/Real::from(d)).unwrap()}
fn main(){
    let source=CurvePath2::try_new(vec![
        CircularArc2::try_from_center(Point2::from_values(0,-1),Point2::from_values(-1,0),Point2::from_values(0,0),true).unwrap().into(),
        QuadraticBezier2::new(Point2::from_values(-1,0),Point2::from_values(0,2),Point2::from_values(1,0)).into(),
    ]).unwrap();
    for policy in [CurveContext::STRICT,CurveContext::APPROXIMATE_512] {
        for reversed in [false,true] {
            let path=if reversed {source.reversed(&policy).unwrap().value} else {source.clone()};
            for constrained in [false,true] {
                let mut request=CurveFillet2::new(Real::one());
                if constrained { request.contacts[usize::from(reversed)]=Some(CurveFilletContact2::Point(Point2::new(q(-3,5),q(-4,5)).into())); }
                let outcome=path.fillet_vertex(1,&request,CurveCornerMode2::TrimOnly,&policy);
                match outcome {
                    Ok(outcome)=>println!("reversed={reversed} constrained={constrained} count={} reason={:?}",outcome.value.candidate_count(),outcome.value.no_solution_reason()),
                    Err(ExactCurveError::Blocked(blocker))=>println!("reversed={reversed} constrained={constrained} blocked={:?}",blocker.reason()),
                    Err(ExactCurveError::Invalid{cause,..})=>println!("reversed={reversed} constrained={constrained} invalid={cause}"),
                }
            }
        }
    }
}
