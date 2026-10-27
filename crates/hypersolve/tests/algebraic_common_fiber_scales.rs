//! Nonzero algebraic equation scales preserve a selected common fiber.

use hyperlimit::{Certainty, PredicatePolicy};
use hyperreal::Real;
use hypersolve::algebraic::{
    AlgebraicRootArithmeticOp, AlgebraicRootArithmeticStatus, AlgebraicRootRepresentation,
    AlgebraicRootValidationReport, AlgebraicRootValidationStatus,
    arithmetic_algebraic_root_representations, validate_algebraic_root_representation,
};
use hypersolve::algebraic_fiber::{
    AlgebraicFiberProjectionStatus, AlgebraicFiberRootCountStatus, AlgebraicField,
    AlgebraicFieldError, count_bivariate_common_fiber_roots_at_algebraic_parameter,
    project_bivariate_fiber_at_algebraic_parameter,
};
use hypersolve::root_isolation::IsolatedRootInterval;
use hypersolve::symbolic::SymbolId;
use hypersolve::{BivariatePolynomial, CurveResultantParameter};

fn q(n: i32, d: i32) -> Real {
    (Real::from(n) / Real::from(d)).unwrap()
}

fn retained_root() -> AlgebraicRootRepresentation {
    // Eliminating y from y^2=x and y^5+5y+5=x gives this equation.
    // Its unique root in (3/5,7/10) has the negative common ordinate.
    let mut root = AlgebraicRootRepresentation {
        constraint_index: 0,
        symbol: SymbolId(0),
        interval_index: 0,
        polynomial_coefficients: vec![
            (-25).into(),
            35.into(),
            (-1).into(),
            10.into(),
            0.into(),
            1.into(),
        ],
        interval: IsolatedRootInterval {
            lower: q(3, 5),
            upper: q(7, 10),
            exact_root: None,
            distinct_root_count: 1,
        },
        validation: AlgebraicRootValidationReport {
            status: AlgebraicRootValidationStatus::Valid,
            message: None,
        },
    };
    root.validation = validate_algebraic_root_representation(&root, PredicatePolicy::STRICT);
    assert!(root.is_valid());
    root
}

fn check_count(scale: Real) {
    let root = retained_root();
    let g = BivariatePolynomial::new(vec![vec![0.into(), 0.into(), 1.into()], vec![(-1).into()]]);
    let f = BivariatePolynomial::new(vec![
        vec![
            Real::from(5) * &scale,
            Real::from(5) * &scale,
            0.into(),
            0.into(),
            0.into(),
            scale.clone(),
        ],
        vec![-scale],
    ]);
    let report = count_bivariate_common_fiber_roots_at_algebraic_parameter(
        &g,
        &f,
        CurveResultantParameter::First,
        &root,
        &(-1).into(),
        &q(-1, 2),
        PredicatePolicy::STRICT,
    );
    assert_eq!(report.status, AlgebraicFiberRootCountStatus::Counted);
    assert_eq!(report.certainty, Certainty::Exact);
    assert_eq!(report.distinct_root_count, Some(1));
}

#[test]
fn unscaled_quintic_common_fiber_is_exact() {
    check_count(Real::one());
}

#[test]
fn common_fiber_count_preserves_algebraic_constant_scaling() {
    let s = Real::from(2).root_n(7).unwrap();
    let square = &s * &s;
    let fourth = &square * &square;
    let fifth = &fourth * &s;
    check_count(fifth);
}

fn composed_scaled_quintic(delta: Real) -> BivariatePolynomial {
    use hypersolve::bivariate_arithmetic::bivariate_compose;
    let s = Real::from(2).root_n(7).unwrap();
    let square = &s * &s;
    let fourth = &square * &square;
    let fifth = &fourth * &s;
    let world = BivariatePolynomial::new(vec![
        vec![
            Real::from(5) * &fifth,
            Real::from(5) * &fourth,
            0.into(),
            0.into(),
            0.into(),
            Real::one() + delta,
        ],
        vec![-fourth],
    ]);
    let x = BivariatePolynomial::new(vec![vec![], vec![s.clone()]]);
    let y = BivariatePolynomial::new(vec![vec![0.into(), s]]);
    bivariate_compose(&world, &x, &y)
}

fn check_composed_count(f: BivariatePolynomial, count: usize) {
    let g = BivariatePolynomial::new(vec![vec![0.into(), 0.into(), 1.into()], vec![(-1).into()]]);
    let report = count_bivariate_common_fiber_roots_at_algebraic_parameter(
        &g,
        &f,
        CurveResultantParameter::First,
        &retained_root(),
        &(-1).into(),
        &q(-1, 2),
        PredicatePolicy::STRICT,
    );
    assert_eq!(
        report.status,
        AlgebraicFiberRootCountStatus::Counted,
        "{report:?}"
    );
    assert_eq!(report.certainty, Certainty::Exact);
    assert_eq!(report.distinct_root_count, Some(count));
}

#[test]
fn common_fiber_count_preserves_composed_seventh_root_scale() {
    check_composed_count(composed_scaled_quintic(Real::zero()), 1);
}

#[test]
fn common_fiber_count_keeps_nearby_distinct_composed_equations() {
    let unit = q(1, 1 << 20);
    let delta = &unit * &unit * &unit * &unit;
    // At the retained negative common ordinate, changing just the leading
    // coefficient adds delta*s^5*y^5 != 0. The original shared root is lost.
    for delta in [delta.clone(), -delta] {
        check_composed_count(composed_scaled_quintic(delta), 0);
    }
}

#[test]
fn common_fiber_count_keeps_signed_scales_and_transposed_parameters() {
    use hypersolve::bivariate_arithmetic::bivariate_scale;
    let f = bivariate_scale(composed_scaled_quintic(Real::zero()), &(-3).into());
    let g = BivariatePolynomial::new(vec![vec![0.into(), 0.into(), 1.into()], vec![(-1).into()]]);
    let transpose = |p: &BivariatePolynomial| {
        let width = p.coefficients.iter().map(Vec::len).max().unwrap_or(0);
        BivariatePolynomial::new(
            (0..width)
                .map(|j| {
                    p.coefficients
                        .iter()
                        .map(|row| row.get(j).cloned().unwrap_or_else(Real::zero))
                        .collect()
                })
                .collect(),
        )
    };
    let f = transpose(&f);
    let g = transpose(&g);
    for (first, second) in [(&f, &g), (&g, &f)] {
        let report = count_bivariate_common_fiber_roots_at_algebraic_parameter(
            first,
            second,
            CurveResultantParameter::Second,
            &retained_root(),
            &(-1).into(),
            &q(-1, 2),
            PredicatePolicy::STRICT,
        );
        assert_eq!(
            report.status,
            AlgebraicFiberRootCountStatus::Counted,
            "{report:?}"
        );
        assert_eq!(report.certainty, Certainty::Exact);
        assert_eq!(report.distinct_root_count, Some(1));
    }
}

#[test]
fn common_fiber_count_preserves_vanishing_parameter_factors_and_selector_validation() {
    use hypersolve::bivariate_arithmetic::bivariate_multiply;
    let root = retained_root();
    let factor = BivariatePolynomial::new(
        root.polynomial_coefficients
            .iter()
            .cloned()
            .map(|c| vec![c])
            .collect(),
    );
    let vanished = bivariate_multiply(&composed_scaled_quintic(Real::zero()), &factor);
    let g = BivariatePolynomial::new(vec![vec![0.into(), 0.into(), 1.into()], vec![(-1).into()]]);
    // The second equation is identically zero on this selected fiber. Keep
    // the negative root of g even though its positive root is outside the box.
    let report = count_bivariate_common_fiber_roots_at_algebraic_parameter(
        &g,
        &vanished,
        CurveResultantParameter::First,
        &root,
        &(-1).into(),
        &q(-1, 2),
        PredicatePolicy::STRICT,
    );
    assert_eq!(
        report.status,
        AlgebraicFiberRootCountStatus::Counted,
        "{report:?}"
    );
    assert_eq!(report.certainty, Certainty::Exact);
    assert_eq!(report.distinct_root_count, Some(1));
    let mut invalid = root;
    invalid.interval.lower = 2.into();
    invalid.interval.upper = 3.into();
    let report = count_bivariate_common_fiber_roots_at_algebraic_parameter(
        &g,
        &vanished,
        CurveResultantParameter::First,
        &invalid,
        &(-1).into(),
        &q(-1, 2),
        PredicatePolicy::STRICT,
    );
    assert_eq!(
        report.status,
        AlgebraicFiberRootCountStatus::InvalidEvidence
    );
}

fn declared_root(coefficients: Vec<Real>, lower: Real, upper: Real) -> AlgebraicRootRepresentation {
    let mut root = retained_root();
    root.polynomial_coefficients = coefficients;
    root.interval.lower = lower;
    root.interval.upper = upper;
    root.validation = validate_algebraic_root_representation(&root, PredicatePolicy::STRICT);
    // Structural validation does not certify the claimed isolation count.
    assert_eq!(root.validation.status, AlgebraicRootValidationStatus::Valid);
    root
}

#[test]
fn selected_root_replay_rejects_empty_and_multiple_isolators() {
    let unit = q(1, 1 << 20);
    let delta = &unit * &unit * &unit * &unit;
    let invalid = [
        declared_root(vec![(-1).into(), 0.into(), 1.into()], (-2).into(), 2.into()),
        declared_root(vec![1.into(), (-2).into(), 1.into()], (-2).into(), 0.into()),
        declared_root(
            vec![-Real::from(2).sqrt().unwrap(), 0.into(), 1.into()],
            2.into(),
            3.into(),
        ),
        declared_root(vec![-Real::pi(), 1.into()], 4.into(), 5.into()),
        declared_root(
            vec![Real::one() + &delta, -Real::from(2) - delta, 1.into()],
            0.into(),
            2.into(),
        ),
        declared_root(vec![(-1).into(), 1.into()], 1.into(), 2.into()),
    ];
    let constant = BivariatePolynomial::new(vec![vec![1.into()]]);
    for root in invalid {
        assert!(hypersolve::algebraic_fiber::AlgebraicField::new(&root).is_err());
        for policy in [
            PredicatePolicy::STRICT,
            PredicatePolicy::APPROXIMATE_512,
            PredicatePolicy::STRICT,
        ] {
            let report = count_bivariate_common_fiber_roots_at_algebraic_parameter(
                &constant,
                &constant,
                CurveResultantParameter::First,
                &root,
                &Real::zero(),
                &Real::one(),
                policy,
            );
            assert_eq!(
                report.status,
                AlgebraicFiberRootCountStatus::InvalidEvidence
            );
        }
    }
}

#[test]
fn selected_root_replay_preserves_owned_endpoints_repetitions_and_exact_coefficients() {
    let valid = [
        // Zero is excluded; the upper endpoint one is owned.
        declared_root(vec![0.into(), (-1).into(), 1.into()], 0.into(), 1.into()),
        declared_root(vec![1.into(), (-2).into(), 1.into()], 0.into(), 2.into()),
        declared_root(
            vec![-Real::from(2).sqrt().unwrap(), 0.into(), 1.into()],
            1.into(),
            2.into(),
        ),
        declared_root(vec![-Real::pi(), 1.into()], 3.into(), 4.into()),
    ];
    let constant = BivariatePolynomial::new(vec![vec![1.into()]]);
    for root in valid {
        assert!(hypersolve::algebraic_fiber::AlgebraicField::new(&root).is_ok());
        for policy in [
            PredicatePolicy::STRICT,
            PredicatePolicy::APPROXIMATE_512,
            PredicatePolicy::STRICT,
        ] {
            let report = count_bivariate_common_fiber_roots_at_algebraic_parameter(
                &constant,
                &constant,
                CurveResultantParameter::First,
                &root,
                &Real::zero(),
                &Real::one(),
                policy,
            );
            assert_eq!(
                report.status,
                AlgebraicFiberRootCountStatus::Counted,
                "{report:?}"
            );
            assert_eq!(report.certainty, Certainty::Exact);
            assert_eq!(report.distinct_root_count, Some(0));
        }
    }
}

#[test]
fn selected_root_replay_checks_uncached_singleton_membership() {
    let constant = BivariatePolynomial::new(vec![vec![1.into()]]);
    for point in [q(1, 2), Real::from(2).sqrt().unwrap(), Real::pi()] {
        let neighbor = &point + Real::one();
        let coefficients = vec![&point * &neighbor, -(&point + &neighbor), Real::one()];
        for (selected, valid) in [(point.clone(), true), (&point + Real::from(2), false)] {
            let root = declared_root(coefficients.clone(), selected.clone(), selected);
            let field = AlgebraicField::new(&root);
            if valid {
                assert!(field.is_ok(), "{field:?}");
            } else {
                assert_eq!(field.err(), Some(AlgebraicFieldError::InvalidEvidence));
            }
            let projection = project_bivariate_fiber_at_algebraic_parameter(
                &constant,
                CurveResultantParameter::First,
                &root,
                8,
            );
            assert_eq!(
                projection.status,
                if valid {
                    AlgebraicFiberProjectionStatus::Constructed
                } else {
                    AlgebraicFiberProjectionStatus::InvalidEvidence
                }
            );
        }
    }
}

#[test]
fn structural_negation_does_not_certify_a_selected_field() {
    // Preserve the original structural-negation generator's minimized case.
    // x^2 + 7 has no real root, before or after reflecting (0, 1].
    let root = declared_root(vec![7.into(), 0.into(), 1.into()], 0.into(), 1.into());
    let report = arithmetic_algebraic_root_representations(
        &root,
        None,
        AlgebraicRootArithmeticOp::Negate,
        PredicatePolicy::APPROXIMATE_512,
    );
    assert_eq!(
        report.status,
        AlgebraicRootArithmeticStatus::ComputedRepresentation
    );
    let reflected = report.result_representation.unwrap();
    assert_eq!(
        reflected.polynomial_coefficients,
        root.polynomial_coefficients
    );
    assert_eq!(reflected.interval.lower, Real::from(-1));
    assert_eq!(reflected.interval.upper, Real::zero());
    for candidate in [&root, &reflected] {
        assert_eq!(
            AlgebraicField::new(candidate).err(),
            Some(AlgebraicFieldError::InvalidEvidence)
        );
    }
}
