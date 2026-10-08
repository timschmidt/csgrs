use hypercurve::{CircularArc2, Curve2, CurveContext, CurveCornerMode2, CurveError,
    CurveFillet2, CurveFilletContact2, CurvePath2, ExactCurveError, Point2, Real};

fn q(n: i32, d: i32) -> Real { (Real::from(n) / Real::from(d)).unwrap() }
fn p(x: i32, y: i32) -> Point2 { Point2::from_values(x, y) }
fn main() {
    // The second authored curve first crosses the annulus radially, then
    // follows the outer circle clockwise. Its first span is an elevated line.
    let next = Curve2::try_nurbs(
        2,
        vec![p(1, 0), Point2::new(q(3,2), Real::zero()), p(2,0), p(2,-2), p(0,-2)],
        vec![Real::one(), Real::one(), Real::one(), q(1,2).sqrt().unwrap(), Real::one()],
        [0,0,0,1,1,2,2,2].into_iter().map(Real::from).collect(),
        &CurveContext::STRICT,
    ).unwrap().value;
    let source = CurvePath2::try_new(vec![
        CircularArc2::try_from_center(p(0,-1), p(1,0), p(0,0), false).unwrap().into(), next,
    ]).unwrap();
    // For every interior angle -pi/2 < theta < 0, the radius-1/2 circle
    // centered at (3/2)(cos(theta),sin(theta)) gives an exact nonzero fillet.
    // This independent witness uses the rational direction (3/5,-4/5).
    let inner = Point2::new(q(3,5),q(-4,5));
    let outer = Point2::new(q(6,5),q(-8,5));
    let center = Point2::new(q(9,10),q(-6,5));
    let witness = CurvePath2::try_new(vec![
        CircularArc2::try_from_center(p(0,-1),inner.clone(),p(0,0),false).unwrap().into(),
        CircularArc2::try_from_center(inner.clone(),outer.clone(),center.clone(),true).unwrap().into(),
        CircularArc2::try_from_center(outer.clone(),p(0,-2),p(0,0),true).unwrap().into(),
    ]).unwrap();
    assert_eq!(witness.curves().len(),3);
    println!("independent_exact_semicircle_witness=true");
    for (name,policy) in [("strict",CurveContext::STRICT),("approximate",CurveContext::APPROXIMATE_512)] {
        for reversed in [false,true] {
            let path=if reversed {source.reversed(&policy).unwrap().value} else {source.clone()};
            for mode in [CurveCornerMode2::TrimOnly,CurveCornerMode2::TrimOrExtend] {
                for constraint in ["radius","center","contact","both","parameter","bad-center"] {
                    let mut request=CurveFillet2::new(q(1,2));
                    if constraint=="center" || constraint=="both" {request.center=Some(center.clone().into());}
                    if constraint=="bad-center" {request.center=Some(p(0,0).into());}
                    if constraint=="contact" || constraint=="both" {
                        request.contacts[usize::from(!reversed)]=Some(CurveFilletContact2::Point(outer.clone().into()));
                    }
                    if constraint=="parameter" {
                        request.contacts[usize::from(!reversed)]=Some(CurveFilletContact2::Parameter(q(if reversed {1} else {3},2).into()));
                    }
                    match path.fillet_vertex(1,&request,mode,&policy) {
                        Ok(outcome)=>println!("policy={name} reversed={reversed} mode={mode:?} request={constraint} count={} reason={:?} certainty={:?}",outcome.value.candidate_count(),outcome.value.no_solution_reason(),outcome.certainty),
                        Err(ExactCurveError::Blocked(blocker))=>println!("policy={name} reversed={reversed} mode={mode:?} request={constraint} blocked={:?}",blocker.reason()),
                        Err(ExactCurveError::Invalid {cause:CurveError::FilletConstraintRequired,..})=>println!("policy={name} reversed={reversed} mode={mode:?} request={constraint} constraint-required"),
                        Err(ExactCurveError::Invalid {cause,..})=>println!("policy={name} reversed={reversed} mode={mode:?} request={constraint} invalid={cause}"),
                    }
                }
            }
        }
    }
}
