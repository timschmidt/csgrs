use hypercurve::{BezierAlgebraicChord2, CircularArc2, Classification, Curve2,
    CurveContext, CurveCornerMode2, CurveError, CurveFillet2, CurveFilletContact2,
    CurvePath2, ExactCurveError, LineSeg2, Point2, Real};
fn p(x: i32, y: i32) -> Point2 { Point2::from_values(x,y) }
fn main() {
    // The second curve joins the two horizontal supports, then follows
    // x(u)=-2u-u², y(u)=2. Its upper straight span is nonlinearly parameterized.
    let next=Curve2::try_nurbs(2,
        vec![p(0,0),p(0,1),p(0,2),p(-1,2),p(-3,2)],
        vec![Real::one();5], [0,0,0,1,1,2,2,2].into_iter().map(Real::from).collect(),
        &CurveContext::STRICT).unwrap().value;
    let witness=CurvePath2::try_new(vec![
        LineSeg2::try_new(p(-3,0),p(-2,0)).unwrap().into(),
        CircularArc2::try_from_center(p(-2,0),p(-2,2),p(-2,1),false).unwrap().into(),
        LineSeg2::try_new(p(-2,2),p(-3,2)).unwrap().into(),
    ]).unwrap();
    assert_eq!(witness.curves().len(),3);
    println!("independent_nonzero_semicircle_witness=true");
    for retained in [false,true] {
        let first=if retained {
            let Classification::Decided(chord)=BezierAlgebraicChord2::try_new(p(-3,0).into(),p(0,0).into(),&CurveContext::STRICT).unwrap() else {panic!("exact chord")};
            Curve2::from(chord)
        } else {Curve2::from(LineSeg2::try_new(p(-3,0),p(0,0)).unwrap())};
        let source=CurvePath2::try_new(vec![first,next.clone()]).unwrap();
        for (name,policy) in [("strict",CurveContext::STRICT),("approximate",CurveContext::APPROXIMATE_512)] {
            for reversed in [false,true] {
                let path=if reversed {source.reversed(&policy).unwrap().value} else {source.clone()};
                for mode in [CurveCornerMode2::TrimOnly,CurveCornerMode2::TrimOrExtend] {
                    for constraint in ["radius","center","point","parameter"] {
                        let mut request=CurveFillet2::new(Real::one());
                        if constraint=="center" {request.center=Some(p(-2,1).into());}
                        if constraint=="point" {request.contacts[usize::from(!reversed)]=Some(CurveFilletContact2::Point(p(-2,2).into()));}
                        if constraint=="parameter" {
                            let t=Real::from(3).sqrt().unwrap();
                            request.contacts[usize::from(!reversed)]=Some(CurveFilletContact2::Parameter((if reversed {Real::from(2)-t}else{t}).into()));
                        }
                        match path.fillet_vertex(1,&request,mode,&policy) {
                            Ok(result)=>println!("retained={retained} policy={name} reversed={reversed} mode={mode:?} request={constraint} count={} reason={:?} certainty={:?}",result.value.candidate_count(),result.value.no_solution_reason(),result.certainty),
                            Err(ExactCurveError::Invalid{cause:CurveError::FilletConstraintRequired,..})=>println!("retained={retained} policy={name} reversed={reversed} mode={mode:?} request={constraint} constraint-required"),
                            Err(error)=>println!("retained={retained} policy={name} reversed={reversed} mode={mode:?} request={constraint} error={error}"),
                        }
                    }
                }
            }
        }
    }
}
