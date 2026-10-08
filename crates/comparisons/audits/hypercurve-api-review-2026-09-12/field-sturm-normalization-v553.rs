#[cfg(test)]
mod field_sturm_normalization_regression {
    use super::*;

    #[test]
    fn positive_field_scaling_preserves_roots_and_repeated_endpoint_ownership() {
        let alpha = Real::from(2).sqrt().unwrap();
        let beta = Real::from(3).sqrt().unwrap();
        let roots = [
            Real::from(-3),
            Real::from(-2),
            -&beta,
            Real::from(-1),
            Real::from(1),
            alpha.clone(),
            Real::from(2),
            Real::from(3),
        ];
        for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
            for negative in [false, true] {
                let gauge = Real::from(2).powi_i64(128).unwrap() * (Real::one() + &alpha);
                let mut polynomial = vec![if negative { -gauge } else { gauge }];
                // Eight independently known distinct roots, with alpha repeated.
                for root in roots.iter().chain(std::iter::once(&alpha)) {
                    let mut product = vec![Real::zero(); polynomial.len() + 1];
                    for (power, coefficient) in polynomial.iter().enumerate() {
                        product[power] -= coefficient * root;
                        product[power + 1] += coefficient;
                    }
                    polynomial = product;
                }
                let chain = UnivariateSturmSequence::new(&polynomial, policy)
                    .expect("positive normalization keeps the exact coefficient field usable");
                for (lower, upper, expected) in [
                    (Real::from(-4), Real::from(4), 8),
                    (Real::from(-4), alpha.clone(), 6),
                    (alpha.clone(), Real::from(4), 2),
                    (-beta.clone(), alpha.clone(), 3),
                ] {
                    assert_eq!(
                        chain.count_distinct_roots(&lower, &upper, policy),
                        Some(expected)
                    );
                }
                assert_eq!(
                    chain.classify_point(&alpha, policy),
                    Some(UnivariateSturmPoint::Root)
                );
                let terminal = chain.terminal_polynomial();
                assert_eq!(terminal.len(), 2, "only alpha is a repeated factor");
                assert_eq!(
                    compare_reals(&Real::eval_poly(terminal, &alpha), &Real::zero(), policy)
                        .value(),
                    Some(Ordering::Equal),
                );
                assert!(matches!(
                    compare_reals(&Real::eval_poly(terminal, &beta), &Real::zero(), policy).value(),
                    Some(Ordering::Less | Ordering::Greater),
                ));
            }
        }
    }
}
