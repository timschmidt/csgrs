use super::*;

fn fraction(n: i32, d: i32) -> Real {
    (Real::from(n) / Real::from(d)).unwrap()
}
fn retained(coefficients: Vec<Real>) -> BezierAlgebraicParameter2 {
    let Classification::Decided(polynomial) =
        BezierParameterPolynomial::try_new_power_basis_with_policy(
            coefficients,
            &CurveContext::STRICT,
        )
        .unwrap()
    else {
        panic!("exact polynomial")
    };
    let Classification::Decided(roots) = polynomial
        .isolate_unit_interval_roots_with_policy(&CurveContext::STRICT)
        .unwrap()
    else {
        panic!("exact root isolation")
    };
    let [BezierParameter2::Algebraic(root)] = roots.as_slice() else {
        panic!("one irrational positive root")
    };
    root.clone()
}
fn selected(
    coefficients: Vec<Real>,
    incidence: BivariatePolynomial,
    upper_branch: bool,
) -> BezierAlgebraicSelectedFiberParameter2 {
    let alpha = retained(coefficients);
    let authority = BezierAlgebraicSelectedFiberAuthority2::new(
        incidence,
        alpha.clone(),
        &CurveContext::STRICT,
    );
    let parameter = authority.parameter(IsolatedRootInterval {
        lower: if upper_branch {
            fraction(1, 2)
        } else {
            fraction(1, 4)
        },
        upper: if upper_branch {
            fraction(3, 4)
        } else {
            fraction(1, 2)
        },
        exact_root: None,
        distinct_root_count: 1,
    });
    drop(authority);
    drop(alpha);
    parameter
}
fn base() -> Vec<Real> {
    vec![-fraction(1, 2), Real::zero(), Real::one()]
}
fn incidence(gauge: i32, repeated: bool) -> BivariatePolynomial {
    // I=16(t-1/2)^2-alpha. Both selected branches are inside (1/4,3/4).
    let rows = if repeated {
        vec![vec![16, -128, 384, -512, 256], vec![-8, 32, -32], vec![1]]
    } else {
        vec![vec![4, -16, 16], vec![-1]]
    };
    BivariatePolynomial::new(
        rows.into_iter()
            .map(|r| r.into_iter().map(|n| Real::from(n * gauge)).collect())
            .collect(),
    )
}
fn predicate(shift: i32) -> BivariatePolynomial {
    // On I=0 and alpha^2=1/2, (4t-2)^4-1/2+shift is exactly shift.
    BivariatePolynomial::new(vec![vec![
        fraction(31, 2) + Real::from(shift),
        Real::from(-128),
        Real::from(384),
        Real::from(-512),
        Real::from(256),
    ]])
}
fn assert_sign(
    parameter: &BezierAlgebraicSelectedFiberParameter2,
    predicate: &BivariatePolynomial,
    sign: RealSign,
    policy: &CurveContext,
) {
    let outcome = crate::evaluate_under(policy.predicate_policy(), || {
        parameter.predicate_sign(predicate, policy)
    });
    assert_eq!(outcome.certainty(), crate::CurveCertainty::Certified);
    assert_eq!(
        outcome.certified().unwrap().unwrap(),
        Classification::Decided(sign)
    );
}
#[test]
fn shifted_fiber_identities_preserve_both_branches_gauges_and_source_drop() {
    for gauge in [1, -7] {
        for upper in [false, true] {
            let point = selected(base(), incidence(gauge, false), upper);
            for policy in [
                CurveContext::STRICT,
                CurveContext::APPROXIMATE_512,
                CurveContext::STRICT,
            ] {
                for (shift, sign) in [
                    (-1, RealSign::Negative),
                    (0, RealSign::Zero),
                    (1, RealSign::Positive),
                ] {
                    assert_sign(&point, &predicate(shift), sign, &policy);
                }
                let branch = BivariatePolynomial::new(vec![vec![-fraction(1, 2), Real::one()]]);
                assert_sign(
                    &point,
                    &branch,
                    if upper {
                        RealSign::Positive
                    } else {
                        RealSign::Negative
                    },
                    &policy,
                );
            }
        }
    }
}
#[test]
fn changed_source_does_not_reuse_an_incidence_zero() {
    let point = selected(
        vec![-fraction(1, 3), Real::zero(), Real::one()],
        incidence(1, false),
        true,
    );
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        assert_sign(&point, &predicate(0), RealSign::Negative, &policy);
    }
}
#[test]
fn repeated_incidence_keeps_selected_zero_without_a_zero_quotient_residue() {
    for gauge in [1, -7] {
        for upper in [false, true] {
            let point = selected(base(), incidence(gauge, true), upper);
            for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
                assert_sign(&point, &predicate(0), RealSign::Zero, &policy);
            }
        }
    }
}
#[test]
fn reducible_base_and_specialized_degree_drop_keep_exact_selection() {
    // (alpha^2-1/2)(alpha-2), with only +sqrt(1/2) selected in the unit interval.
    let point = selected(
        vec![Real::one(), -fraction(1, 2), Real::from(-2), Real::one()],
        incidence(1, false),
        false,
    );
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        assert_sign(&point, &predicate(0), RealSign::Zero, &policy);
    }
    let alpha = retained(base());
    // The apparent quintic is t-alpha after specialization; leading degree drops.
    let equation = BivariatePolynomial::new(vec![
        vec![
            Real::zero(),
            Real::one(),
            Real::zero(),
            Real::zero(),
            Real::zero(),
            -fraction(1, 2),
        ],
        vec![-Real::one()],
        vec![
            Real::zero(),
            Real::zero(),
            Real::zero(),
            Real::zero(),
            Real::zero(),
            Real::one(),
        ],
    ]);
    let point = BezierAlgebraicSelectedFiberAuthority2::new(equation, alpha, &CurveContext::STRICT)
        .parameter(IsolatedRootInterval {
            lower: fraction(1, 2),
            upper: Real::one(),
            exact_root: None,
            distinct_root_count: 1,
        });
    let query = BivariatePolynomial::new(vec![vec![-fraction(1, 2), Real::zero(), Real::one()]]);
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        assert_sign(&point, &query, RealSign::Zero, &policy);
    }
}
#[test]
fn bounded_pass_retry_keeps_the_exact_zero_and_qualification() {
    let point = selected(base(), incidence(1, false), true);
    let query = predicate(0);
    for policy in [
        CurveContext::STRICT,
        CurveContext::APPROXIMATE_512,
        CurveContext::STRICT,
    ] {
        let bounded = crate::evaluate_under(policy.predicate_policy(), || {
            policy.bounded_exact_predicate_pass(|| point.predicate_sign(&query, &policy))
        });
        assert_eq!(bounded.certainty(), crate::CurveCertainty::Certified);
        assert!(matches!(
            bounded.certified().unwrap().unwrap(),
            Classification::Uncertain(UncertaintyReason::Predicate)
                | Classification::Decided(RealSign::Zero)
        ));
        assert_sign(&point, &query, RealSign::Zero, &policy);
    }
}
#[test]
fn qualified_terminal_cannot_become_a_strict_fiber_zero() {
    let point = selected(base(), incidence(1, false), true);
    let query = BivariatePolynomial::new(vec![vec![
        crate::test_support::terminally_unresolved_zero(),
    ]]);
    for policy in [
        CurveContext::STRICT,
        CurveContext::APPROXIMATE_512,
        CurveContext::STRICT,
    ] {
        let outcome = crate::evaluate_under(policy.predicate_policy(), || {
            point.predicate_sign(&query, &policy)
        });
        if policy == CurveContext::STRICT {
            assert_eq!(outcome.certainty(), crate::CurveCertainty::Certified);
            assert!(matches!(
                outcome.certified().unwrap().unwrap(),
                Classification::Uncertain(_)
            ));
        } else {
            assert_eq!(
                outcome.certainty(),
                crate::CurveCertainty::Approximate512Consumed
            );
            assert_eq!(
                outcome.into_unverified().unwrap(),
                Classification::Decided(RealSign::Zero)
            );
        }
    }
}

#[test]
fn nearby_nonzero_fiber_residues_remain_distinct() {
    let point = selected(base(), incidence(1, false), true);
    let mut epsilon = Real::one();
    for _ in 0..120 {
        epsilon = (epsilon / Real::from(2)).unwrap();
    }
    for (delta, expected) in [
        (epsilon.clone(), RealSign::Positive),
        (-epsilon, RealSign::Negative),
    ] {
        let mut query = predicate(0);
        query.coefficients[0][0] += delta;
        for policy in [
            CurveContext::STRICT,
            CurveContext::APPROXIMATE_512,
            CurveContext::STRICT,
        ] {
            assert_sign(&point, &query, expected, &policy);
        }
    }
}

#[test]
fn nonrational_coefficient_field_preserves_exact_zero() {
    let gamma = fraction(1, 2).sqrt().unwrap();
    // alpha^3+alpha=gamma has exactly one root in (0,1): its derivative
    // is positive, and its values at 0 and 1 have opposite signs.
    let point = selected(
        vec![-&gamma, Real::one(), Real::zero(), Real::one()],
        incidence(1, false),
        true,
    );
    // I=0 gives (4t-2)^2=alpha, so this expanded polynomial is zero.
    let query = BivariatePolynomial::new(vec![vec![
        Real::from(68) - gamma,
        Real::from(-784),
        Real::from(3856),
        Real::from(-10240),
        Real::from(15360),
        Real::from(-12288),
        Real::from(4096),
    ]]);
    for policy in [
        CurveContext::STRICT,
        CurveContext::APPROXIMATE_512,
        CurveContext::STRICT,
    ] {
        assert_sign(&point, &query, RealSign::Zero, &policy);
    }
}
