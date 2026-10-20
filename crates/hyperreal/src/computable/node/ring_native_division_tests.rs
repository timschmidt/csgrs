#[cfg(test)]
mod ring_native_division_tests {
    use super::*;

    fn corpus(seed: i32) -> RingPolynomial {
        let root = Computable::one();
        let mut recognition = RingRecognition::new(&root);
        let mut result = RingPolynomial::default();
        for (index, term) in [vec![], vec![0], vec![1], vec![0, 1], vec![1, 1]]
            .into_iter()
            .enumerate()
        {
            let coefficient = (seed * (index as i32 + 3) + 7) % 11 - 5;
            recognition
                .insert(&mut result, term, Rational::from(coefficient))
                .unwrap();
        }
        result
    }

    #[test]
    fn graded_division_preserves_exact_products_and_agrees_with_the_existing_divider() {
        let root = Computable::one();
        for a in 0..12 {
            for b in 0..12 {
                let (divisor, expected) = (corpus(a), corpus(b));
                let product = RingRecognition::new(&root)
                    .multiply(&divisor, &expected)
                    .unwrap();
                let old = RingRecognition::new(&root)
                    .divide_exact_polynomial(&product, &divisor)
                    .unwrap();
                let mut native = RingRecognition::new(&root);
                native.native_relations = true;
                let new = native.divide_exact_polynomial(&product, &divisor).unwrap();
                assert_eq!(old.terms, expected.terms);
                assert_eq!(new.terms, expected.terms);
                // Arbitrary non-product pairs also exercise nonzero remainders.
                let old = RingRecognition::new(&root).divide_exact_polynomial(&expected, &divisor);
                let new = native.divide_exact_polynomial(&expected, &divisor);
                assert_eq!(old.map(|v| v.terms), new.map(|v| v.terms));
            }
        }
    }

    #[test]
    fn graded_division_respects_zero_divisors_work_and_coefficient_limits() {
        let root = Computable::one();
        let mut native = RingRecognition::new(&root);
        native.native_relations = true;
        let zero = RingPolynomial::default();
        let nonzero = corpus(3);
        assert!(native.divide_exact_polynomial(&nonzero, &zero).is_none());
        assert!(
            native
                .divide_exact_polynomial(&zero, &nonzero)
                .unwrap()
                .terms
                .is_empty()
        );
        let large =
            RingRecognition::constant(Rational::from_bigint(BigInt::one() << 2048_usize)).unwrap();
        let tiny = RingRecognition::constant(
            Rational::from_bigint(BigInt::one() << 2048_usize)
                .inverse()
                .unwrap(),
        )
        .unwrap();
        assert!(native.divide_exact_polynomial(&large, &tiny).is_none());
        native.operations_left = 2;
        assert!(native.divide_exact_polynomial(&nonzero, &nonzero).is_none());
        let mut native = RingRecognition::new(&root);
        native.native_relations = true;
        assert_eq!(
            native
                .divide_exact_polynomial(&nonzero, &nonzero)
                .unwrap()
                .terms,
            vec![(vec![], Rational::one())]
        );
    }
}
