use super::*;

#[test]
fn independent_fiber_queries_preserve_multiplicity_and_actual_base_dependencies() {
    let base = source(false, false).defining_fiber().1.clone();
    // A repeated univariate support still keeps its original defining
    // polynomial and both root selections. Queries depending on alpha must
    // continue to consume alpha=sqrt(2), even though this support does not.
    let repeated = p(&[&[9, 0, -6, 0, 1]]);
    for negative in [false, true] {
        let root = IsolatedRootInterval {
            lower: if negative { (-2).into() } else { 1.into() },
            upper: if negative { (-1).into() } else { 2.into() },
            exact_root: None,
            distinct_root_count: 1,
        };
        let Classification::Decided(mut roots) =
            BezierAlgebraicSelectedFiberParameter2::from_checked_roots(
                repeated.clone(),
                base.clone(),
                &[root],
                &CurveContext::STRICT,
            )
            .unwrap()
        else {
            panic!("checked repeated root");
        };
        let selected = roots.pop().unwrap();
        for (query, expected) in [
            (p(&[&[-3, 0, 1]]), RealSign::Zero),
            (
                p(&[&[0, 1]]),
                if negative {
                    RealSign::Negative
                } else {
                    RealSign::Positive
                },
            ),
            (p(&[&[0], &[1]]), RealSign::Positive),
            (p(&[&[-2], &[], &[1]]), RealSign::Zero),
            (p(&[&[0, 0, 1], &[-1]]), RealSign::Positive),
        ] {
            assert_eq!(
                selected
                    .predicate_sign(&query, &CurveContext::STRICT)
                    .unwrap(),
                Classification::Decided(expected)
            );
        }
        assert_eq!(selected.defining_fiber(), (&repeated, &base));
    }
    // The root relation itself depends on the selected base: u^2=alpha.
    // Treating it as independent would change the point to u=0.
    let incidence = p(&[&[0, 0, 1], &[-1]]);
    let root = IsolatedRootInterval {
        lower: 1.into(),
        upper: 2.into(),
        exact_root: None,
        distinct_root_count: 1,
    };
    let Classification::Decided(mut roots) =
        BezierAlgebraicSelectedFiberParameter2::from_checked_roots(
            incidence.clone(),
            base.clone(),
            &[root],
            &CurveContext::STRICT,
        )
        .unwrap()
    else {
        panic!("checked dependent root");
    };
    let selected = roots.pop().unwrap();
    assert_eq!(
        selected
            .predicate_sign(&p(&[&[-2, 0, 0, 0, 1]]), &CurveContext::STRICT)
            .unwrap(),
        Classification::Decided(RealSign::Zero)
    );
    assert_eq!(
        selected
            .predicate_sign(&p(&[&[-3, 0, 1]]), &CurveContext::STRICT)
            .unwrap(),
        Classification::Decided(RealSign::Negative)
    );
    assert_eq!(selected.defining_fiber(), (&incidence, &base));
}

#[test]
fn selected_fiber_order_consumes_certified_native_evidence_in_either_direction() {
    let native = BezierParameter2::Exact(Real::from(3).sqrt().unwrap());
    let tiny = (Real::one() / Real::from(1_u64 << 40)).unwrap();
    let tiny = &tiny * &tiny;
    for offset in [-tiny.clone(), Real::zero(), tiny] {
        for reverse in [false, true] {
            let selected = source(false, false);
            let selected = value(selected.affine_image_unbounded(
                &Real::one(),
                &offset,
                &CurveContext::STRICT,
            ));
            let known = source(false, true);
            let definition = (
                selected.defining_fiber().0.clone(),
                selected.defining_fiber().1.clone(),
            );
            // Establish the native value through the existing exact predicate;
            // no test-injected cache flag is a substitute for that proof.
            assert_eq!(
                known
                    .cmp_bezier_parameter(&native, &CurveContext::STRICT)
                    .unwrap(),
                Classification::Decided(std::cmp::Ordering::Equal)
            );
            assert!(known.retained_bezier_parameter().is_some());
            let expected = offset
                .exact_rational_ref()
                .unwrap()
                .partial_cmp(Real::zero().exact_rational_ref().unwrap())
                .unwrap();
            let (first, second, expected) = if reverse {
                (&known, &selected, expected.reverse())
            } else {
                (&selected, &known, expected)
            };
            let result = hypersolve::work_budget::with_budget(
                hypersolve::work_budget::WorkBudget {
                    refinements: 512,
                    promotions: 64,
                },
                || first.cmp_by_refinement(second, &CurveContext::STRICT),
            );
            assert_eq!(
                result.result.unwrap().unwrap(),
                Classification::Decided(expected)
            );
            assert_eq!(selected.defining_fiber(), (&definition.0, &definition.1));
        }
    }
    let negative = source(true, false);
    let positive = source(false, true);
    assert_eq!(
        positive
            .cmp_bezier_parameter(&native, &CurveContext::STRICT)
            .unwrap(),
        Classification::Decided(std::cmp::Ordering::Equal)
    );
    assert_eq!(
        negative
            .cmp_by_refinement(&positive, &CurveContext::STRICT)
            .unwrap(),
        Classification::Decided(std::cmp::Ordering::Less)
    );
}

fn p(rows: &[&[i32]]) -> BivariatePolynomial {
    BivariatePolynomial::new(
        rows.iter()
            .map(|row| row.iter().copied().map(Real::from).collect())
            .collect(),
    )
}
fn source(negative: bool, reducible: bool) -> BezierAlgebraicSelectedFiberParameter2 {
    let coefficients = if reducible {
        vec![6.into(), (-2).into(), (-3).into(), 1.into()]
    } else {
        vec![(-2).into(), 0.into(), 1.into()]
    };
    let retained = BezierAlgebraicParameter2::try_isolate(
        BezierParameterPolynomial::try_new_power_basis(coefficients).unwrap(),
        BezierParameterInterval::try_new(1.into(), 2.into()).unwrap(),
    )
    .unwrap();
    let root = IsolatedRootInterval {
        lower: if negative { (-2).into() } else { 1.into() },
        upper: if negative { (-1).into() } else { 2.into() },
        exact_root: None,
        distinct_root_count: 1,
    };
    let Classification::Decided(mut roots) =
        BezierAlgebraicSelectedFiberParameter2::from_checked_roots(
            p(&[&[-3, 0, 1]]),
            retained,
            &[root],
            &CurveContext::STRICT,
        )
        .unwrap()
    else {
        panic!("checked source")
    };
    roots.pop().unwrap()
}
fn value(
    result: CurveResult<Classification<BezierAlgebraicSelectedFiberParameter2>>,
) -> BezierAlgebraicSelectedFiberParameter2 {
    let Classification::Decided(value) = result.unwrap() else {
        panic!("exact image")
    };
    value
}

#[test]
fn rational_images_select_conjugates_reject_poles_and_keep_base_identity() {
    for reducible in [false, true] {
        for negative in [false, true] {
            let source = source(negative, reducible);
            let image = value(source.rational_image_in_retained_field(
                &p(&[&[0, 1], &[1]]),
                &p(&[&[1, 1]]),
                &CurveContext::STRICT,
            ));
            assert!(image.data.representations.bezier.get().is_none());
            assert_eq!(
                image.data.authority.data.retained_parameter,
                source.data.authority.data.retained_parameter
            );
            let order = image
                .cmp_bezier_parameter(&BezierParameter2::Exact(Real::one()), &CurveContext::STRICT)
                .unwrap();
            assert_eq!(
                order,
                Classification::Decided(if negative {
                    std::cmp::Ordering::Less
                } else {
                    std::cmp::Ordering::Greater
                })
            );
            // Noninjective image: x^2+y^2=5 on both selected conjugates.
            let constant = value(source.rational_image_in_retained_field(
                &p(&[&[0, 0, 1], &[], &[1]]),
                &p(&[&[1]]),
                &CurveContext::STRICT,
            ));
            assert_eq!(
                constant
                    .cmp_bezier_parameter(&BezierParameter2::Exact(5.into()), &CurveContext::STRICT)
                    .unwrap(),
                Classification::Decided(std::cmp::Ordering::Equal)
            );
            for numerator in [p(&[&[1]]), p(&[&[-3, 0, 1]])] {
                assert_eq!(
                    source
                        .rational_image_in_retained_field(
                            &numerator,
                            &p(&[&[-3, 0, 1]]),
                            &CurveContext::STRICT
                        )
                        .unwrap_err(),
                    CurveError::from(hyperreal::Problem::DivideByZero)
                );
            }
            value(source.rational_image_in_retained_field(
                &p(&[&[0, 1], &[1]]),
                &p(&[&[1, 1]]),
                &CurveContext::STRICT,
            ));
        }
    }
}

#[test]
fn near_pole_images_refine_source_bounds_and_recover_after_exhaustion() {
    let source = source(false, false);
    let q = (Real::from(1_414_213_562_373_i64) / Real::from(1_000_000_000_000_i64)).unwrap();
    let denominator = BivariatePolynomial::new(vec![vec![-q.clone()], vec![Real::one()]]);
    let numerator = p(&[&[1]]);
    let scale = (Real::one() / (Real::from(2) - &q * &q)).unwrap();
    let expected =
        BezierParameter2::Algebraic(source.data.authority.data.retained_parameter.clone())
            .affine_image_unbounded(&scale, &(&q * &scale), &CurveContext::STRICT)
            .unwrap();
    let Classification::Decided(expected) = expected else {
        panic!("exact inverse identity")
    };
    // 1/(alpha-q)=(alpha+q)/(2-q^2); the initial base interval straddles q.
    let image = value(source.rational_image_in_retained_field(
        &numerator,
        &denominator,
        &CurveContext::STRICT,
    ));
    assert_eq!(
        image
            .cmp_bezier_parameter(&expected, &CurveContext::STRICT)
            .unwrap(),
        Classification::Decided(std::cmp::Ordering::Equal)
    );
    let denied =
        hypersolve::work_budget::with_budget(hypersolve::work_budget::WorkBudget::ZERO, || {
            assert!(matches!(
                source
                    .rational_image_in_retained_field(
                        &numerator,
                        &denominator,
                        &CurveContext::STRICT
                    )
                    .unwrap(),
                Classification::Uncertain(UncertaintyReason::BudgetExhausted)
            ));
        });
    assert!(denied.result.is_err());
    value(source.rational_image_in_retained_field(&numerator, &denominator, &CurveContext::STRICT));
}

#[test]
fn affine_source_identity_survives_refinement_without_a_cycle() {
    let selected = source(false, false);
    let definition = crate::CurvePointDefinition2::PolynomialFiber {
        equation: selected.data.authority.data.incidence.clone(),
        parameter: BezierParameter2::Algebraic(
            selected.data.authority.data.retained_parameter.clone(),
        )
        .into(),
        root: selected.data.root.clone(),
    };
    let point = definition.into_point().unwrap();
    let CurvePointData2::Polynomial(data) = &point.0 else {
        panic!("polynomial")
    };
    let weak = Arc::downgrade(data);
    let value = point
        .affine_form(1.into(), 1.into(), 0.into())
        .parameter()
        .unwrap();
    let refined = match value
        .as_selected_fiber()
        .unwrap()
        .refined(8, &CurveContext::STRICT)
        .unwrap()
    {
        Classification::Decided(value) => value,
        _ => panic!("refinement"),
    };
    assert!(
        refined
            .affine_form_source()
            .unwrap()
            .source()
            .shares_storage(&point)
    );
    drop(point);
    drop(value);
    assert!(weak.upgrade().is_some());
    drop(refined);
    assert!(weak.upgrade().is_none());
}
