use hypercurve::{
    BezierAlgebraicParameter2, BezierParameter2, BezierParameterInterval,
    BezierParameterPolynomial, Curve2, CurveCertainty, CurveError, CurveParameter2,
    CurvePointDefinition2, CurvePointLocations2, ExactCurveError, LineSeg2, Point2,
    PredicatePolicy, QuadraticBezier2, Real, Similarity2,
};
use std::cmp::Ordering;

fn q(n: i32, d: i32) -> Real {
    (Real::from(n) / Real::from(d)).unwrap()
}
fn selected() -> BezierParameter2 {
    BezierParameter2::Algebraic(
        BezierAlgebraicParameter2::try_isolate(
            BezierParameterPolynomial::try_new_power_basis(vec![
                (-1).into(),
                0.into(),
                0.into(),
                2.into(),
            ])
            .unwrap(),
            BezierParameterInterval::try_new(q(3, 4), q(7, 8)).unwrap(),
        )
        .unwrap(),
    )
}

#[test]
fn nurbs_coordinate_images_replay_their_shared_selected_root() {
    let p = Point2::from_values;
    let curve = Curve2::try_nurbs(
        2,
        vec![p(4, 0), p(5, 1), p(4, 2)],
        vec![Real::one(); 3],
        vec![0.into(), 0.into(), 0.into(), 1.into(), 1.into(), 1.into()],
    )
    .unwrap();
    let line = Curve2::from(
        LineSeg2::try_new(
            Point2::new(q(17, 4), (-3).into()),
            Point2::new(q(17, 4), 3.into()),
        )
        .unwrap(),
    );
    for contacts in [
        curve.intersect_curve(&line).unwrap(),
        line.intersect_curve(&curve).unwrap(),
    ] {
        assert!(contacts.is_complete());
        assert_eq!(contacts.contacts().len(), 2);
        for contact in contacts.contacts() {
            let point = contact.point();
            let cold = point.definition().unwrap().into_point().unwrap();
            assert!(cold.coincides_with(point).unwrap());
            let original_parameter = point
                .affine_form(0.into(), 1.into(), 0.into())
                .parameter()
                .unwrap();
            let replayed_parameter = cold
                .affine_form(0.into(), 1.into(), 0.into())
                .parameter()
                .unwrap();
            assert!(original_parameter.as_bezier_parameter().is_some());
            assert!(replayed_parameter.as_bezier_parameter().is_some());
            assert_eq!(
                original_parameter.compare(&replayed_parameter).unwrap(),
                Ordering::Equal
            );
            let order = cold
                .affine_form(0.into(), 1.into(), 0.into())
                .compare_to_real(&Real::one())
                .unwrap();
            assert_ne!(order, Ordering::Equal);
            // x=17/4 gives y=1 +/- sqrt(1/2) on this quadratic spline.
            let offset = q(1, 2).sqrt().unwrap();
            let y = if order == Ordering::Less {
                Real::one() - offset
            } else {
                Real::one() + offset
            };
            assert!(
                cold.coincides_with(&Point2::new(q(17, 4), y).into())
                    .unwrap()
            );
        }
    }
}

#[test]
fn independent_selected_chord_intersection_definition_replays() {
    let x: CurveParameter2 = selected().into();
    let y: CurveParameter2 = BezierParameter2::Algebraic(
        BezierAlgebraicParameter2::try_isolate(
            BezierParameterPolynomial::try_new_power_basis(vec![
                (-1).into(),
                0.into(),
                0.into(),
                0.into(),
                0.into(),
                2.into(),
            ])
            .unwrap(),
            BezierParameterInterval::try_new(q(3, 4), q(7, 8)).unwrap(),
        )
        .unwrap(),
    )
    .into();
    let vertical = Curve2::try_line(
        x.affine_point(1.into(), 0.into(), 0.into(), 0.into())
            .unwrap(),
        x.affine_point(1.into(), 0.into(), 0.into(), 2.into())
            .unwrap(),
    )
    .unwrap();
    let horizontal = Curve2::try_line(
        y.affine_point(0.into(), 0.into(), 1.into(), 0.into())
            .unwrap(),
        y.affine_point(0.into(), 2.into(), 1.into(), 0.into())
            .unwrap(),
    )
    .unwrap();
    let contacts = vertical.intersect_curve(&horizontal).unwrap();
    assert!(contacts.is_complete());
    assert_eq!(contacts.contacts().len(), 1);
    let point = contacts.contacts()[0].point();
    assert!(point.coordinates().is_none());
    let definition = point
        .definition()
        .expect("retain both exact supporting lines");
    assert!(matches!(
        definition,
        CurvePointDefinition2::SupportingLineIntersection { .. }
    ));
    let cold = definition.into_point().unwrap();
    assert!(cold.coincides_with(point).unwrap());
    for (a, b, expected) in [(1, 0, &x), (0, 1, &y)] {
        assert_eq!(
            cold.affine_form(a.into(), b.into(), 0.into())
                .parameter()
                .unwrap()
                .compare(expected)
                .unwrap(),
            Ordering::Equal
        );
    }
}

#[test]
fn supporting_line_definition_proves_uniqueness_and_infinite_domain() {
    let p = |x: i32, y: i32| -> hypercurve::CurvePoint2 { Point2::from_values(x, y).into() };
    let first = [p(0, 0), p(1, 0)];
    let second = [p(2, 1), p(2, 2)];
    for (first, second) in [
        (first.clone(), second.clone()),
        ([first[1].clone(), first[0].clone()], second.clone()),
        (second.clone(), first.clone()),
    ] {
        for policy in [
            PredicatePolicy::STRICT,
            PredicatePolicy::APPROXIMATE_512,
            PredicatePolicy::STRICT,
        ] {
            let outcome = hypercurve::evaluate_under(policy, || {
                CurvePointDefinition2::SupportingLineIntersection {
                    first: first.clone(),
                    second: second.clone(),
                }
                .into_point()
            });
            assert_eq!(outcome.certainty(), CurveCertainty::Certified);
            let point = outcome.into_unverified().unwrap();
            assert!(point.coincides_with(&p(2, 0)).unwrap());
            let parameter = point
                .affine_form(1.into(), 0.into(), 0.into())
                .parameter()
                .unwrap();
            let form = parameter
                .affine_form_source()
                .expect("preserve both support authorities even with represented endpoints");
            let [a, b, c] = form.coefficients().clone();
            let cold_parameter = form
                .source()
                .definition()
                .unwrap()
                .into_point()
                .unwrap()
                .affine_form(a, b, c)
                .parameter()
                .unwrap();
            assert!(cold_parameter.affine_form_source().is_some());
            assert_eq!(cold_parameter.compare(&parameter).unwrap(), Ordering::Equal);
            assert!(
                point
                    .definition()
                    .unwrap()
                    .into_point()
                    .unwrap()
                    .coincides_with(&p(2, 0))
                    .unwrap()
            );
        }
    }
    for second in [[p(0, 1), p(1, 1)], [p(2, 0), p(3, 0)]] {
        assert!(matches!(
            CurvePointDefinition2::SupportingLineIntersection {
                first: first.clone(),
                second
            }
            .into_point(),
            Err(ExactCurveError::Invalid {
                cause: CurveError::ParallelLineSupports,
                ..
            })
        ));
    }
    assert!(matches!(
        CurvePointDefinition2::SupportingLineIntersection {
            first: [p(0, 0), p(0, 0)],
            second
        }
        .into_point(),
        Err(ExactCurveError::Invalid {
            cause: CurveError::ZeroLengthLine,
            ..
        })
    ));
    let gap: Real = "1e-12".parse().unwrap();
    let point = CurvePointDefinition2::SupportingLineIntersection {
        first,
        second: [p(0, 1), Point2::new(1.into(), Real::one() - &gap).into()],
    }
    .into_point()
    .unwrap();
    assert!(
        point
            .coincides_with(&Point2::new((Real::one() / gap).unwrap(), Real::zero()).into())
            .unwrap()
    );
}

#[test]
fn correlated_point_definitions_replay_and_preserve_affine_coordinates() {
    let t: CurveParameter2 = selected().into();
    let p = t
        .affine_point(1.into(), 0.into(), 0.into(), 3.into())
        .unwrap();
    assert!(matches!(
        p.definition().unwrap(),
        CurvePointDefinition2::Rational { .. }
    ));
    let p = p.definition().unwrap().into_point().unwrap();
    assert_eq!(
        p.affine_form(1.into(), 0.into(), 0.into())
            .parameter()
            .unwrap()
            .compare(&t)
            .unwrap(),
        Ordering::Equal
    );
    assert_eq!(
        p.affine_form(0.into(), 1.into(), 0.into())
            .parameter()
            .unwrap()
            .scalar(),
        Some(&3.into())
    );
    let expression = || CurvePointDefinition2::RationalExpression {
        parameter: selected(),
        x_numerator: vec![7.into(), (-2).into()],
        y_numerator: vec![3.into()],
        denominator: vec![1.into()],
    };
    for policy in [
        PredicatePolicy::STRICT,
        PredicatePolicy::APPROXIMATE_512,
        PredicatePolicy::STRICT,
    ] {
        let result = hypercurve::evaluate_under(policy, || expression().into_point());
        assert_eq!(result.certainty(), CurveCertainty::Certified);
        let image = result.into_unverified().unwrap();
        assert!(matches!(
            image.definition().unwrap(),
            CurvePointDefinition2::RationalExpression { .. }
        ));
        let cold = image.definition().unwrap().into_point().unwrap();
        assert_eq!(
            cold.affine_form(1.into(), 0.into(), 0.into())
                .parameter()
                .unwrap()
                .compare(&t.affine_image(&(-2).into(), &7.into()).unwrap())
                .unwrap(),
            Ordering::Equal
        );
        let reflected = CurvePointDefinition2::Similarity {
            source: cold,
            transform: Similarity2::try_from_real_affine(
                (-1).into(),
                0.into(),
                0.into(),
                1.into(),
                9.into(),
                2.into(),
            )
            .unwrap(),
        }
        .into_point()
        .unwrap();
        assert_eq!(
            reflected
                .affine_form(0.into(), 1.into(), 0.into())
                .parameter()
                .unwrap()
                .scalar(),
            Some(&5.into())
        );
        assert_eq!(
            reflected
                .affine_form(1.into(), 0.into(), 0.into())
                .parameter()
                .unwrap()
                .compare(&t.affine_image(&2.into(), &2.into()).unwrap())
                .unwrap(),
            Ordering::Equal
        );
    }
}

#[test]
fn point_definition_poles_and_undefined_normals_fail_before_recovery() {
    for denominator in [
        vec![],
        vec![0.into()],
        vec![(-1).into(), 0.into(), 0.into(), 2.into()],
    ] {
        let result = CurvePointDefinition2::RationalExpression {
            parameter: selected(),
            x_numerator: vec![1.into()],
            y_numerator: vec![0.into()],
            denominator,
        }
        .into_point();
        assert!(
            matches!(
                result,
                Err(ExactCurveError::Invalid {
                    cause: CurveError::InvalidCurveParameter,
                    ..
                })
            ),
            "{result:?}"
        );
    }
    let stationary = QuadraticBezier2::new(
        Point2::from_values(0, 0),
        Point2::from_values(0, 0),
        Point2::from_values(1, 0),
    );
    let normal = CurvePointDefinition2::Parallel {
        source: stationary.parallel_left(1.into()).unwrap(),
        parameter: Real::zero().into(),
    }
    .into_point();
    assert!(normal.is_err());
    let point = CurveParameter2::from(selected())
        .affine_point(1.into(), 0.into(), 0.into(), 2.into())
        .unwrap();
    assert!(
        point
            .definition()
            .unwrap()
            .into_point()
            .unwrap()
            .coincides_with(&point)
            .unwrap()
    );
}

#[test]
fn retained_chord_locations_prove_incidence_and_finite_membership() {
    let t: CurveParameter2 = selected().into();
    let point = |xoff: Real, y: Real| t.affine_point(1.into(), xoff, 0.into(), y).unwrap();
    let chord = Curve2::try_line(point(0.into(), 0.into()), point(0.into(), 3.into())).unwrap();
    assert!(chord.geometry().is_none());
    let gap: Real = "1e-12".parse().unwrap();
    for curve in [chord.clone(), chord.reversed().unwrap()] {
        for y in [Real::zero(), q(3, 2), Real::from(3)] {
            let p = point(0.into(), y);
            let CurvePointLocations2::Locations(locations) = curve.point_locations(&p).unwrap()
            else {
                panic!("finite line");
            };
            assert_eq!(locations.len(), 1);
            assert!(
                curve
                    .point_at(&locations[0].parameter().unwrap())
                    .unwrap()
                    .coincides_with(&p)
                    .unwrap()
            );
        }
        for p in [
            point(gap.clone(), q(3, 2)),
            point(0.into(), -&gap),
            point(0.into(), Real::from(3) + &gap),
        ] {
            let CurvePointLocations2::Locations(locations) = curve.point_locations(&p).unwrap()
            else {
                panic!("finite line");
            };
            assert!(locations.is_empty());
        }
    }
}

#[test]
fn affine_line_retraction_does_not_accept_an_off_support_source() {
    let parallel = QuadraticBezier2::new(
        Point2::from_values(0, 0),
        Point2::from_values(2, 2),
        Point2::from_values(4, 0),
    )
    .parallel_left(1.into())
    .unwrap();
    let point = CurvePointDefinition2::Parallel {
        source: parallel,
        parameter: selected().into(),
    }
    .into_point()
    .unwrap();
    let parameter = point
        .affine_form(q(1, 4), 0.into(), 0.into())
        .parameter()
        .unwrap();
    assert!(parameter.affine_form_source().is_some());
    let line = Curve2::from(
        LineSeg2::try_new(Point2::from_values(0, 0), Point2::from_values(4, 0)).unwrap(),
    );
    let image = line.point_at(&parameter).unwrap();
    assert_eq!(
        image
            .affine_form(0.into(), 1.into(), 0.into())
            .compare_to_real(&0.into())
            .unwrap(),
        Ordering::Equal
    );
    assert_eq!(
        point
            .affine_form(0.into(), 1.into(), 0.into())
            .compare_to_real(&0.into())
            .unwrap(),
        Ordering::Greater
    );
}

#[test]
fn selected_parallel_subrange_replays_regular_support() {
    let select = |coefficients: &[i32]| {
        BezierParameter2::Algebraic(
            BezierAlgebraicParameter2::try_isolate(
                BezierParameterPolynomial::try_new_power_basis(
                    coefficients.iter().copied().map(Real::from).collect(),
                )
                .unwrap(),
                BezierParameterInterval::try_new(q(1, 8), q(1, 4)).unwrap(),
            )
            .unwrap(),
        )
    };
    let first = select(&[-16, 64, 448, -1024, 1024]);
    let second = select(&[-1, -10, 90, -160, 128]);
    assert_eq!(
        first
            .cmp_by_refinement(&second)
            .expect("retained endpoint order"),
        Ordering::Less
    );
    let range = hypercurve::BezierParameterRange2::try_new(first.clone(), second.clone())
        .expect("oriented range");
    let reversed = hypercurve::BezierParameterRange2::try_new(second.clone(), first.clone())
        .expect("reversed range");
    assert_eq!(
        reversed.start().cmp_by_refinement(reversed.end()).unwrap(),
        Ordering::Greater
    );
    assert!(matches!(
        hypercurve::BezierParameterRange2::try_new(first.clone(), first.clone()),
        Err(ExactCurveError::Invalid {
            cause: CurveError::InvalidBezierRange,
            ..
        })
    ));
    let parallel = QuadraticBezier2::new(
        Point2::from_values(0, 0),
        Point2::from_values(2, 2),
        Point2::from_values(4, 0),
    )
    .parallel_left(1.into())
    .unwrap();
    let domain = hypercurve::CurveParameterRange2::try_new(first.into(), second.into()).unwrap();
    let analysis = parallel
        .singularity_analysis(&domain)
        .expect("complete regularity on retained domain");
    assert!(analysis.source_is_regular());
    assert!(analysis.parallel_is_cusp_free());
    let curve =
        Curve2::try_analytic_parallel(parallel, range).expect("checked parallel construction");
    assert!(curve.analytic_parallel().is_some());
}
