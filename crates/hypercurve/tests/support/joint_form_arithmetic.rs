use hypercurve::*;
use hypersolve::{BivariatePolynomial, IsolatedRootInterval};
use std::cmp::Ordering;

fn q(n: i32, d: i32) -> Real {
    (Real::from(n) / Real::from(d)).unwrap()
}
fn p(rows: &[&[i32]]) -> BivariatePolynomial {
    BivariatePolynomial::new(
        rows.iter()
            .map(|r| r.iter().copied().map(Real::from).collect())
            .collect(),
    )
}
fn one() -> BivariatePolynomial {
    p(&[&[1]])
}
fn x() -> BivariatePolynomial {
    p(&[&[], &[1]])
}
fn y() -> BivariatePolynomial {
    p(&[&[0, 1]])
}
fn base() -> BezierParameter2 {
    BezierParameter2::Algebraic(
        BezierAlgebraicParameter2::try_isolate(
            BezierParameterPolynomial::try_new_power_basis(vec![q(-1, 8), 0.into(), 1.into()])
                .unwrap(),
            BezierParameterInterval::try_new(q(1, 3), q(1, 2)).unwrap(),
        )
        .unwrap(),
    )
}
fn joint() -> CurvePoint2 {
    // u²=1/8 and 16v³=u, with both positive roots selected together.
    CurvePointDefinition2::PolynomialFiber {
        equation: p(&[&[0, 0, 0, 16], &[-1]]),
        parameter: base(),
        root: IsolatedRootInterval {
            lower: q(1, 4),
            upper: q(3, 8),
            exact_root: None,
            distinct_root_count: 1,
        },
    }
    .into_point()
    .unwrap()
}
fn form(point: &CurvePoint2, n: BivariatePolynomial) -> CurvePointRationalForm2 {
    point.rational_form(n, one()).unwrap()
}
fn equal(form: &CurvePointRationalForm2, rhs: Real) {
    assert_eq!(form.compare_to_real(&rhs).unwrap(), Ordering::Equal);
}

pub fn polynomial_images_and_original_source_arithmetic() {
    let source = joint();
    assert!(source.coordinates().is_none());
    let mapped = source
        .rational_image(&RationalMap2::new([x(), y()], p(&[&[1], &[1]])))
        .unwrap();
    let u = form(&source, x());
    let v = form(&source, y());
    let image_u = mapped.rational_form(x(), p(&[&[1], &[-1]])).unwrap();
    let sum = image_u.add(&u).unwrap();
    equal(&sum.multiply(&sum).unwrap(), q(1, 2));
    equal(&image_u.divide(&u).unwrap(), 1.into());
    equal(&image_u.subtract(&u).unwrap(), 0.into());
    assert_eq!(sum.operands().unwrap().0.source(), &mapped);
    assert_eq!(sum.operands().unwrap().1.source(), &source);
    let turn = Similarity2::try_from_real_affine(
        0.into(),
        (-2).into(),
        2.into(),
        0.into(),
        7.into(),
        (-11).into(),
    )
    .unwrap();
    let turned = CurvePointDefinition2::Similarity {
        source: source.clone(),
        transform: turn,
    }
    .into_point()
    .unwrap();
    let turned_v = turned
        .rational_form(p(&[&[-7], &[1]]), p(&[&[-2]]))
        .unwrap();
    equal(&turned_v.subtract(&v).unwrap(), 0.into());
    let cold = mapped.definition().unwrap().into_point().unwrap();
    let cold_u = cold.rational_form(x(), p(&[&[1], &[-1]])).unwrap();
    equal(&cold_u.multiply(&u).unwrap(), q(1, 8));
    let residual = image_u.subtract(&u).unwrap();
    let gap = (Real::one() / Real::from(1_000_000_000_000u64)).unwrap();
    assert_eq!(residual.compare_to_real(&gap).unwrap(), Ordering::Less);
    assert_eq!(residual.compare_to_real(&-gap).unwrap(), Ordering::Greater);
}

pub fn native_parameter_fields_share_rational_queries() {
    let t: CurveParameter2 = BezierParameter2::Algebraic(
        BezierAlgebraicParameter2::try_isolate(
            BezierParameterPolynomial::try_new_power_basis(vec![
                q(-1, 5),
                0.into(),
                0.into(),
                1.into(),
            ])
            .unwrap(),
            BezierParameterInterval::try_new(q(1, 2), q(3, 4)).unwrap(),
        )
        .unwrap(),
    )
    .into();
    let curve = |coords: Vec<(Real, Real)>| -> Curve2 {
        RationalBezier2::try_new(
            coords.into_iter().map(|(x, y)| Point2::new(x, y)).collect(),
            vec![1.into(); 4],
        )
        .unwrap()
        .into()
    };
    let a = curve(vec![
        (0.into(), 0.into()),
        (q(1, 3), 0.into()),
        (q(2, 3), q(1, 3)),
        (1.into(), 1.into()),
    ]);
    let b = curve(vec![
        (0.into(), 0.into()),
        (0.into(), 0.into()),
        (q(1, 3), 0.into()),
        (1.into(), 1.into()),
    ]);
    let pa = a.point_at(&t).unwrap();
    let pb = b.point_at(&t).unwrap();
    assert!(pa.coordinates().is_none() && pb.coordinates().is_none());
    let ta = form(&pa, x());
    let tb2 = form(&pb, x());
    equal(&ta.multiply(&tb2).unwrap(), q(1, 5));
    equal(&form(&pa, y()).subtract(&tb2).unwrap(), 0.into());
    let va = a
        .derivatives_at(&t, 1)
        .unwrap()
        .pop()
        .unwrap()
        .coordinate_image()
        .unwrap();
    let vb = b
        .derivatives_at(&t, 1)
        .unwrap()
        .pop()
        .unwrap()
        .coordinate_image()
        .unwrap();
    let twice_t = form(&va, y());
    let thrice_t2 = form(&vb, y());
    equal(&twice_t.multiply(&thrice_t2).unwrap(), q(6, 5));
    let cold = pb.definition().unwrap().into_point().unwrap();
    equal(&ta.multiply(&form(&cold, x())).unwrap(), q(1, 5));
}

pub fn constants_divisors_and_authority_guards() {
    let source = joint();
    let u = form(&source, x());
    let authored: CurvePoint2 = Point2::new(3.into(), 4.into()).into();
    let five = form(&authored, p(&[&[-2, 1], &[1]]));
    equal(&u.add(&five).unwrap().subtract(&u).unwrap(), 5.into());
    equal(&five.divide(&u).unwrap().multiply(&u).unwrap(), 5.into());
    let zero = u.subtract(&u).unwrap();
    for result in [u.divide(&zero), zero.divide(&zero)] {
        assert!(matches!(
            result,
            Err(ExactCurveError::Invalid {
                cause: CurveError::ZeroRationalPointDenominator,
                ..
            })
        ));
    }
    let pole = p(&[&[-1], &[], &[8]]);
    assert!(matches!(
        source.rational_form(pole.clone(), pole),
        Err(ExactCurveError::Invalid {
            cause: CurveError::ZeroRationalPointDenominator,
            ..
        })
    ));
    let multiplied = u.scaled(&0.into()).unwrap().add(&five).unwrap();
    equal(&multiplied, 5.into());
    assert!(multiplied.operands().is_some());
}

pub fn different_fiber_selectors_do_not_merge() {
    let point = |lower, upper| {
        CurvePointDefinition2::PolynomialFiber {
            equation: p(&[&[0, 0, 1], &[-1]]),
            parameter: base(),
            root: IsolatedRootInterval {
                lower,
                upper,
                exact_root: None,
                distinct_root_count: 1,
            },
        }
        .into_point()
        .unwrap()
    };
    let negative = point(q(-3, 4), q(-1, 2));
    let positive = point(q(1, 2), q(3, 4));
    let result = form(&negative, y()).multiply(&form(&positive, y()));
    assert!(
        matches!(result,Err(ExactCurveError::Blocked(ref b)) if b.reason()==UncertaintyReason::Unsupported)
    );
    // Constant values can enter either field without identifying the roots.
    equal(
        &form(&negative, one())
            .multiply(&form(&positive, one()))
            .unwrap(),
        1.into(),
    );
}

pub fn policy_budget_and_finite_composition_recover() {
    let source = joint();
    let v = form(&source, y());
    for policy in [
        PredicatePolicy::STRICT,
        PredicatePolicy::APPROXIMATE_512,
        PredicatePolicy::STRICT,
    ] {
        let result = evaluate_under(policy, || {
            let mut power = v.clone();
            for _ in 1..6 {
                power = power.multiply(&v).unwrap();
            }
            equal(&power, q(1, 2048));
            assert!(
                power
                    .numerator()
                    .coefficients
                    .iter()
                    .map(Vec::len)
                    .sum::<usize>()
                    <= 7
            );
        });
        assert_eq!(result.certainty(), CurveCertainty::Certified);
    }
    let query = source.rational_form(x(), p(&[&[1], &[1]])).unwrap();
    let denied = hypersolve::work_budget::with_budget(
        hypersolve::work_budget::WorkBudget {
            refinements: 0,
            promotions: 0,
        },
        || {
            let result = query.add(&v).and_then(|value| value.bounds());
            assert!(
                matches!(result,Err(ExactCurveError::Blocked(ref b)) if b.reason()==UncertaintyReason::BudgetExhausted)
            );
        },
    );
    assert!(denied.result.is_err());
    query.add(&v).unwrap().bounds().unwrap();
    equal(&query.divide(&query).unwrap(), 1.into());
}

pub fn common_denominator_composition_retains_bounded_degree() {
    let source = joint();
    let denominator = p(&[&[1], &[1]]);
    let term = source.rational_form(x(), denominator.clone()).unwrap();
    let mut sum = term.clone();
    for _ in 0..12 {
        sum = sum.add(&term).unwrap();
    }
    assert_eq!(
        sum.compare(&term.scaled(&13.into()).unwrap()).unwrap(),
        Ordering::Equal
    );
    assert!(
        sum.denominator().coefficients.len() <= 2,
        "repeated additions in one checked denominator must not grow its degree: {}",
        sum.denominator().coefficients.len()
    );
    assert!(sum.numerator().coefficients.len() <= 2);
    assert!(
        sum.operands().is_some(),
        "original operand guards remain retained"
    );
}

pub fn shared_operand_debug_does_not_expand_the_dag() {
    let source = joint();
    let term = source.rational_form(x(), p(&[&[1], &[1]])).unwrap();
    let mut sum = term.clone();
    for _ in 0..20 {
        sum = sum.add(&sum).unwrap();
    }
    equal(
        &sum.subtract(&term.scaled(&Real::from(1u64 << 20)).unwrap())
            .unwrap(),
        0.into(),
    );
    assert!(sum.operands().is_some());
    let source_text = format!("{term:?}");
    let derived_text = format!("{sum:?}");
    assert!(
        derived_text.len() <= source_text.len() + 256,
        "Debug must describe the query without recursively expanding shared operand guards"
    );
}
