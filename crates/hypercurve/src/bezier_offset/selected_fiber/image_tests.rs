use super::*;

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
        ),
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
