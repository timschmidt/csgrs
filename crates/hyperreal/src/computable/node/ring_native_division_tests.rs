#[cfg(test)]
/// Qualification of native graded formal division against known products, the earlier divider and limits.
mod ring_native_division_tests {
    use super::*;

    /// Build a deterministic bounded two-variable polynomial from five possible monomials.
    /// Test seeds 0..12 select signed integer coefficients; the standard insertion
    /// helper merges/omits zero terms and establishes stored order. No atom values
    /// are assigned because these fixtures exercise formal polynomial division.
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
    /// Check 144 constructed products divide back to their known factor with both division strategies.
    /// Also compare both strategies on 144 arbitrary non-product pairs, including
    /// nonzero remainders. The explicit factor assertions check algebraic expectations;
    /// agreement of the two implementation paths alone is not an independent oracle.
    /// The native arbitrary-pair attempt retains work spent on its preceding product.
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
    /// Reject the zero polynomial divisor, return zero over a nonzero polynomial, and enforce caps.
    /// The quotient 2^2048 / 2^-2048 exceeds coefficient admission, while a two-charge
    /// attempt cannot divide the ordinary fixture by itself. A fresh normal-budget
    /// attempt must return one; scratch resets do not imply automatic replenishment.
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
