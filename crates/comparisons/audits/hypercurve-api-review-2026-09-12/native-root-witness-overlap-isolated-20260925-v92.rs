use hypercurve::{CircularArc2, Classification, Curve2, CurveContext, CurveCornerMode2,
    CurveCornerSolutions2, CurvePath2, Point2, QuadraticBezier2, Real,
    CurveCertainty, CurveIntersectionPairBlockerKind2};
fn q(numerator: i32, denominator: i32) -> Real { (Real::from(numerator) / Real::from(denominator)).unwrap() }
fn p(x: i32, y: i32) -> Point2 { Point2::new(Real::from(x), Real::from(y)) }
fn main() {
    // P(t) = (t, t^2) and
    // Q(s) = (1, 1) + (-31/65, -86/325)s + (-48/65, -43/325)s^2.
    // Their left parallels at distance 1/2 meet at the exact parameters
    // t = 6/5 and s = -1, outside both authored spans but inside their
    // endpoint-adjacent regular cells.
    let previous_cut = Point2::new(q(6, 5), q(36, 25));
    let next_cut = Point2::new(q(48, 65), q(368, 325));
    let expected_center = Point2::new(q(48, 65), q(1061, 650));
    let path = CurvePath2::try_new(vec![
        Curve2::from(QuadraticBezier2::new(
            p(0, 0),
            Point2::new(q(1, 2), Real::zero()),
            p(1, 1),
        )),
        Curve2::from(QuadraticBezier2::new(
            p(1, 1),
            Point2::new(q(99, 130), q(282, 325)),
            Point2::new(-q(14, 65), q(196, 325)),
        )),
    ])
    .unwrap();

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for reversed in [false, true] {
            let path = if reversed {
                path.clone().reversed(&policy).unwrap().into_value()
            } else {
                path.clone()
            };
            let result = path
                .fillet_vertex_by_radius(1, q(1, 2), CurveCornerMode2::TrimOrExtend, &policy)
                .expect("both regular Bezier incident extensions must be solved exactly");
            assert_eq!(result.certainty, CurveCertainty::Certified);
            let has_expected = |candidate: &CurvePath2| {
                let curves = candidate.curves();
                println!("candidate: approximate={} reversed={reversed} pieces={} families={:?}", policy == CurveContext::APPROXIMATE_512, curves.len(), curves.iter().map(Curve2::family).collect::<Vec<_>>());
                if curves.len() < 3 {
                    return false;
                }
                let (expected_previous, expected_next) = if reversed {
                    (&next_cut, &previous_cut)
                } else {
                    (&previous_cut, &next_cut)
                };
                for (actual, expected) in [
                    (curves[0].end(), expected_previous),
                    (curves.last().unwrap().start(), expected_next),
                ] {
                    let same = actual.coincides_with(&expected.clone().into(), &policy);
                    assert_eq!(same.certainty, CurveCertainty::Certified);
                    println!("endpoint match: {:?}", same.value);
                    if same.value != Classification::Decided(true) {
                        return false;
                    }
                }
                let expected = Curve2::from(
                    CircularArc2::try_from_center(
                        expected_previous.clone(),
                        expected_next.clone(),
                        expected_center.clone(),
                        reversed,
                    )
                    .unwrap(),
                );
                let compare = |left: &hypercurve::CurveParameter2,
                               right: &hypercurve::CurveParameter2| {
                    let order = left.compare(right, &policy).unwrap();
                    assert_eq!(order.certainty, CurveCertainty::Certified);
                    match order.value {
                        Classification::Decided(order) => order,
                        Classification::Uncertain(reason) => {
                            panic!("circle coverage order: {reason:?}")
                        }
                    }
                };
                let ascending = |range: &hypercurve::CurveParameterRange2| {
                    let (start, end) = (range.start().clone(), range.end().clone());
                    if compare(&start, &end).is_gt() {
                        (end, start)
                    } else {
                        (start, end)
                    }
                };
                // A retained major arc may use more than one circular chart.
                // Certify the complete trace against an independent exact arc,
                // including its center, finite sweep, and traversal direction.
                for (piece_index, piece) in curves[1..curves.len() - 1].iter().enumerate() {
                    hyperreal::dispatch_trace::reset();
                    let overlap = hyperreal::dispatch_trace::with_recording(|| piece.intersect_curve(&expected, &policy)).unwrap();
                    let trace = hyperreal::dispatch_trace::take_trace();
                    println!("approximate={}; reversed={reversed}; piece={piece_index}; materialized={}; complete={}; contacts={}; overlaps={}; blockers={}",
                        policy == CurveContext::APPROXIMATE_512, piece.geometry().is_some(), overlap.value.is_complete(), overlap.value.contacts().len(),
                        overlap.value.overlaps().len(), overlap.value.blockers().len());
                    for blocker in overlap.value.blockers() {
                        let label = match blocker.kind() {
                            CurveIntersectionPairBlockerKind2::Uncertain(reason) => format!("uncertain {reason:?}"),
                            CurveIntersectionPairBlockerKind2::IncompleteReplay { .. } => "incomplete replay".to_owned(),
                            CurveIntersectionPairBlockerKind2::SharedComponent => "shared component".to_owned(),
                        };
                        println!("span pair {}, {}: {label}", blocker.first_span_index(), blocker.second_span_index());
                    }
                    if !overlap.value.is_complete() { println!("trace: {trace:?}"); }
                    assert_eq!(overlap.certainty, CurveCertainty::Certified);
                    assert!(overlap.value.is_complete());
                    let mut ranges = overlap
                        .value
                        .overlaps()
                        .iter()
                        .map(|overlap| {
                            assert_eq!(overlap.first_span_index(), 0);
                            assert!(overlap.includes_start() && overlap.includes_end());
                            assert_eq!(
                                overlap.orientation(),
                                hypercurve::RationalBezierOverlapOrientation2::Same
                            );
                            ascending(overlap.first_range())
                        })
                        .collect::<Vec<_>>();
                    ranges.sort_by(|left, right| compare(&left.0, &right.0));
                    let (mut cursor, end) = ascending(piece.parameter_domain());
                    for (lower, upper) in ranges {
                        if compare(&lower, &cursor).is_gt() {
                            return false;
                        }
                        if compare(&upper, &cursor).is_gt() {
                            cursor = upper;
                        }
                    }
                    if !compare(&cursor, &end).is_eq() {
                        return false;
                    }
                }
                true
            };
            match result.into_value() {
                CurveCornerSolutions2::Unique(candidate) => assert!(has_expected(&candidate)),
                CurveCornerSolutions2::Multiple(candidates) => {
                    assert!(candidates.iter().any(has_expected));
                }
                CurveCornerSolutions2::NoSolution(reason) => {
                    panic!("the exact projective Bezier fillet was lost: {reason:?}")
                }
            }
        }
    }
}
