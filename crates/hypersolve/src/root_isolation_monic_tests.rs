use super::*;

#[test]
fn monic_normalization_keeps_scalar_and_failed_reciprocal_boundaries() {
    let policy = PredicatePolicy::STRICT;
    assert!(gcd_monic_normalize(Vec::new(), policy).is_none());
    assert_eq!(
        gcd_monic_normalize(vec![Real::zero()], policy),
        Some(vec![Real::zero()])
    );
    for value in [Real::one(), Real::from(-7), Real::from(2).ln().unwrap()] {
        assert_eq!(
            gcd_monic_normalize(vec![value], policy),
            Some(vec![Real::one()])
        );
    }
    let unresolved = crate::test_support::terminal_zero();
    assert_eq!(
        compare_reals(&unresolved, &Real::zero(), policy).value(),
        None
    );
    assert!(gcd_monic_normalize(vec![unresolved.clone()], policy).is_none());
    // Noncanonical inputs are outside the helper's caller contract, but must
    // still not turn a failed leading reciprocal into a literal unit.
    for leading in [Real::zero(), unresolved] {
        assert!(gcd_monic_normalize(vec![Real::one(), leading], policy).is_none());
    }
}

#[test]
fn monic_normalization_preserves_lower_coefficients_and_certified_unit() {
    let policy = PredicatePolicy::STRICT;
    let log = Real::from(2).ln().unwrap();
    for leading in [
        Real::from(3),
        Real::from(-3),
        Real::pi(),
        log.clone(),
        &log * &log,
        Real::from(2).sqrt().unwrap(),
        crate::test_support::exact_normal_positive(),
    ] {
        let inverse = reciprocal_real(&leading, policy).unwrap().value().unwrap();
        for degree in [1, 2, 3, 7, 8, 16, 31, 32, 64] {
            let mut coefficients: Vec<_> = (0..degree)
                .map(|i| match i % 4 {
                    0 => Real::zero(),
                    1 => Real::from(-3),
                    2 => log.clone(),
                    _ => crate::test_support::terminal_zero(),
                })
                .collect();
            let expected: Vec<_> = coefficients.iter().map(|c| c * &inverse).collect();
            coefficients.push(leading.clone());
            let normalized = gcd_monic_normalize(coefficients, policy).unwrap();
            assert_eq!(normalized.len(), degree + 1);
            assert_eq!(&normalized[..degree], expected.as_slice());
            assert_eq!(
                normalized[degree].exact_rational_ref(),
                Some(&HyperRational::one())
            );
        }
    }
}

#[test]
fn square_free_log_factors_do_not_rebuild_an_opaque_leading_unit() {
    let policy = PredicatePolicy::STRICT;
    for code in [5, 11, 14] {
        let log = Real::from(2).ln().unwrap();
        let roots = [Real::zero(), log.clone(), log + Real::one()];
        let mut polynomial = vec![Real::from(if code % 2 == 0 { 2 } else { -2 })];
        let mut expected = polynomial.clone();
        let mut digits = code;
        let multiply = |p: &mut Vec<Real>, root: &Real| {
            let mut next = vec![Real::zero(); p.len() + 1];
            for (i, coefficient) in p.iter().enumerate() {
                next[i] -= coefficient * root;
                next[i + 1] += coefficient;
            }
            *p = next;
        };
        for root in roots {
            let exponent = digits % 3;
            digits /= 3;
            for _ in 0..exponent {
                multiply(&mut polynomial, &root);
            }
            if exponent != 0 {
                multiply(&mut expected, &root);
            }
        }
        let actual = square_free_part(polynomial, policy).unwrap();
        assert_eq!(actual.len(), expected.len());
        // These particular formulas preserve the input scale. The general
        // square-free API is only a same-distinct-roots contract.
        for (a, b) in actual.iter().zip(&expected) {
            assert_eq!(compare_reals(a, b, policy).value(), Some(Ordering::Equal));
        }
    }
}
