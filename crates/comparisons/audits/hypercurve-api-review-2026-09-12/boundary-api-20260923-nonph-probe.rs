use hypercurve::*;
fn p(x: i64, y: i64) -> Point2 {
    Point2::new(Real::from(x), Real::from(y))
}

fn q(numerator: i64, denominator: i64) -> Real {
    (Real::from(numerator) / Real::from(denominator)).unwrap()
}

fn square(min_x: i64, min_y: i64, max_x: i64, max_y: i64) -> Contour2 {
    Contour2::try_new(vec![
        Segment2::Line(LineSeg2::try_new(p(min_x, min_y), p(max_x, min_y)).unwrap()),
        Segment2::Line(LineSeg2::try_new(p(max_x, min_y), p(max_x, max_y)).unwrap()),
        Segment2::Line(LineSeg2::try_new(p(max_x, max_y), p(min_x, max_y)).unwrap()),
        Segment2::Line(LineSeg2::try_new(p(min_x, max_y), p(min_x, min_y)).unwrap()),
    ])
    .unwrap()
}

fn certified<T>(outcome: CurveOutcome<T>) -> T {
    assert_eq!(outcome.certainty, CurveCertainty::Certified);
    outcome.value
}

fn main() {
    let end = Point2::new(-q(14, 65), q(196, 325));
    let path = CurvePath2::try_new(vec![
        Curve2::from(QuadraticBezier2::new(
            p(0, 0),
            Point2::new(q(1, 2), Real::zero()),
            p(1, 1),
        )),
        Curve2::from(QuadraticBezier2::new(
            p(1, 1),
            Point2::new(q(99, 130), q(282, 325)),
            end.clone(),
        )),
        Curve2::from(LineSeg2::try_new(end, p(0, 0)).unwrap()),
    ])
    .unwrap();

    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for reversed in [false, true] {
            eprintln!("case policy={policy:?}, reversed={reversed}");
            let (oriented_path, vertex_index) = if reversed {
                (path.clone().reversed(&policy).unwrap().into_value(), 2)
            } else {
                (path.clone(), 1)
            };
            let source = CurveRegion2::try_from_boundary_paths(
                std::slice::from_ref(&oriented_path),
                &policy,
            )
            .unwrap()
            .into_value();
            eprintln!("admitted; begin fillet");
            let result = source
                .fillet_loop_vertex_by_radius(
                    0,
                    vertex_index,
                    q(2, 5),
                    CurveCornerMode2::TrimOrExtend,
                    &policy,
                )
                .expect("the algebraic Bezier-pair incident cells must remain retained");
            assert_eq!(result.certainty, CurveCertainty::Certified);
            eprintln!("fillet candidates={}", result.value.candidate_count());
            let candidates = match result.into_value() {
                CurveCornerSolutions2::Unique(candidate) => vec![candidate],
                CurveCornerSolutions2::Multiple(candidates) => candidates,
                CurveCornerSolutions2::NoSolution(reason) => {
                    panic!("the projective algebraic fillet was lost: {reason:?}")
                }
            };
            let has_projective_selected_circle = |candidate: &&CurveRegion2| {
                let paths = candidate.boundary_paths(&policy).expect("exact boundary paths");
                assert_eq!(paths.certainty, CurveCertainty::Certified);
                let Classification::Decided(paths) = paths.value else { panic!("retained paths must be decided"); };
                let fragments = paths[0].curves();
                eprintln!("candidate curves: {:?}", fragments.iter().map(|curve| (curve.family(), curve.geometry().is_some(), curve.parameter_domain().scalar_endpoints().is_some())).collect::<Vec<_>>());
                fragments
                    .iter()
                    .filter(|fragment| {
                        fragment.geometry().is_none()
                            && matches!(
                                fragment.family(),
                                CurveFamily2::QuadraticBezier
                                    | CurveFamily2::CubicBezier
                                    | CurveFamily2::RationalQuadraticBezier
                                    | CurveFamily2::RationalBezier
                            )
                    })
                    .count()
                    >= 2
                    && fragments
                        .iter()
                        .any(|fragment| fragment.family() == CurveFamily2::CircularArc)
            };
            let filleted = candidates
                .iter()
                .find(has_projective_selected_circle)
                .expect("both projective algebraic cuts and the selected circle must be retained");
            eprintln!("selected candidate; begin point classification");
            assert_eq!(
                certified(filleted.classify_point(&p(10, 10), &policy).unwrap()),
                Classification::Decided(RegionPointLocation::Outside),
            );
            eprintln!("point classified; admit distant region");
            let distant =
                CurveRegion2::try_from_native_material_contours(vec![square(8, 8, 9, 9)], &policy)
                    .unwrap()
                    .into_value();
            eprintln!("begin Boolean reentry");
            let replay = filleted
                .boolean_regions(&distant, &policy)
                .expect("the projective algebraic fillet must re-enter the Boolean kernel")
                .into_value();
            assert!(replay.intersection().is_empty());
            assert_eq!(replay.union().boundary_loops().len(), 2);
            eprintln!("case complete");
        }
    }
}

