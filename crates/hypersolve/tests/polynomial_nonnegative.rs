//! Exact sign laws, including repeated and endpoint roots and narrow pockets.
use hyperreal::Real;
use hypersolve::certify_univariate_polynomial_nonnegative as certify_with_policy;

fn certify(polynomial: &[Real], lower: &Real, upper: &Real) -> Option<bool> {
    certify_with_policy(
        polynomial,
        lower,
        upper,
        hyperlimit::PredicatePolicy::STRICT,
    )
}

fn q(a: i64, b: i64) -> Real {
    (Real::from(a) / Real::from(b)).unwrap()
}

fn product(a: &[Real], b: &[Real]) -> Vec<Real> {
    let mut result = vec![Real::zero(); a.len() + b.len() - 1];
    for (i, a) in a.iter().enumerate() {
        for (j, b) in b.iter().enumerate() {
            result[i + j] += a * b;
        }
    }
    result
}

#[test]
fn nonnegative_certificate_distinguishes_zeros_and_sign_changes() {
    let zero = Real::zero();
    let one = Real::one();
    for (polynomial, expected) in [
        (vec![0.into()], true),
        (vec![1.into()], true),
        (vec![(-1).into()], false),
        (vec![0.into(), 1.into()], true),
        (vec![0.into(), 1.into(), (-1).into()], true),
        (vec![0.into(), (-1).into(), 1.into()], false),
        (vec![q(1, 4), (-1).into(), 1.into()], true),
        (vec![-q(1, 4), 1.into(), (-1).into()], false),
        (vec![-q(1, 2), 1.into()], false),
    ] {
        assert_eq!(
            certify(&polynomial, &zero, &one),
            Some(expected),
            "{polynomial:?}"
        );
    }
    assert_eq!(certify(&[], &zero, &one), None);
    assert_eq!(certify(std::slice::from_ref(&one), &one, &zero), None);
    assert_eq!(
        certify(&[-one.clone(), one.clone()], &one, &one),
        Some(true)
    );
}

#[test]
fn nonnegative_certificate_separates_multiple_algebraic_and_nearby_roots() {
    let zero = Real::zero();
    let one = Real::one();
    let sqrt_two = Real::from(2).sqrt().unwrap();
    let roots = [
        q(1, 8),
        q(1, 3),
        (&sqrt_two / Real::from(2)).unwrap(),
        q(7, 8),
    ];
    let mut square = vec![one.clone()];
    for root in roots {
        let factor = vec![-root, one.clone()];
        square = product(&square, &product(&factor, &factor));
    }
    assert_eq!(certify(&square, &zero, &one), Some(true));
    let negative = square.iter().map(|x| -x).collect::<Vec<_>>();
    assert_eq!(certify(&negative, &zero, &one), Some(false));

    // Endpoints and a coarse interior grid are positive. Only complete root
    // separation detects this negative pocket; the coefficients are exact.
    let epsilon = (Real::one() / Real::from(1_u128 << 80)).unwrap();
    let a = q(1, 3);
    let pocket = product(&[-a.clone(), one.clone()], &[-(&a + &epsilon), one.clone()]);
    assert_eq!(certify(&pocket, &zero, &one), Some(false));
    assert_eq!(certify(&product(&pocket, &pocket), &zero, &one), Some(true));
}

#[test]
fn nonnegative_certificate_matches_complete_rational_root_sign_cells() {
    // Every polynomial has exactly the authored root multiset. Integer sign
    // parity in the four complementary cells is an independent complete oracle.
    for code in 0..243 {
        let mut code = code;
        let mut polynomial = vec![Real::one()];
        let mut signs = [1_i32; 4];
        for root_index in 0..5 {
            let multiplicity = code % 3;
            code /= 3;
            for _ in 0..multiplicity {
                polynomial = product(&polynomial, &[-q(root_index, 4), Real::one()]);
            }
            for (cell, sign) in signs.iter_mut().enumerate() {
                if (2 * cell + 1) < 2 * root_index as usize && multiplicity % 2 == 1 {
                    *sign = -*sign;
                }
            }
        }
        assert_eq!(
            certify(&polynomial, &Real::zero(), &Real::one()),
            Some(signs.iter().all(|s| *s > 0))
        );
        let negative = polynomial.iter().map(|x| -x).collect::<Vec<_>>();
        assert_eq!(
            certify(&negative, &Real::zero(), &Real::one()),
            Some(signs.iter().all(|s| *s < 0))
        );
    }
}

#[test]
fn nonnegative_certificate_keeps_unknown_after_a_terminal_policy_query() {
    let zero = (Real::e() + Real::one()).sin()
        - Real::e().sin() * Real::one().cos()
        - Real::e().cos() * Real::one().sin();
    assert!(matches!(
        hyperlimit::compare_reals(&zero, &Real::zero(), hyperlimit::PredicatePolicy::STRICT),
        hyperlimit::PredicateOutcome::Unknown { .. }
    ));
    assert_eq!(
        certify(std::slice::from_ref(&zero), &Real::zero(), &Real::one()),
        None
    );
    let terminal = hyperlimit::compare_reals(
        &zero,
        &Real::zero(),
        hyperlimit::PredicatePolicy::APPROXIMATE_512,
    );
    assert!(matches!(
        terminal,
        hyperlimit::PredicateOutcome::Decided {
            certainty: hyperlimit::Certainty::Approximate,
            ..
        }
    ));
    assert_eq!(
        certify_with_policy(
            std::slice::from_ref(&zero),
            &Real::zero(),
            &Real::one(),
            hyperlimit::PredicatePolicy::APPROXIMATE_512,
        ),
        None
    );
    assert_eq!(certify(&[zero], &Real::zero(), &Real::one()), None);
    assert_eq!(
        certify(&[Real::zero(), Real::one()], &Real::zero(), &Real::one()),
        Some(true)
    );
    assert_eq!(
        certify_with_policy(
            &[Real::zero(), Real::one()],
            &Real::zero(),
            &Real::one(),
            hyperlimit::PredicatePolicy::APPROXIMATE_512,
        ),
        Some(true)
    );
}
