impl Real {
    fn tower_computable(&self) -> Computable {
        if let Some(rational) = self.exact_rational() {
            return Computable::rational(rational);
        }
        if self.rational.sign() == Sign::NoSign {
            return Computable::zero();
        }
        let payload = self.computable_ref().clone();
        if self.rational.is_one() {
            payload
        } else {
            payload.multiply_rational(self.rational.clone())
        }
    }

    /// Sign proved by reduction in `Q(sqrt(d))(sqrt(a + b sqrt(d)))`.
    ///
    /// `None` means this value is outside that tower or its square relations do
    /// not decide the sign. The original value is unchanged.
    pub fn quadratic_tower_sign(&self) -> Option<RealSign> {
        self.tower_computable().quadratic_tower_sign()
    }

    /// Evaluate `coefficients[0] + coefficients[1] t + ...` at `t = ±sqrt(square)`.
    ///
    /// The root is the principal square root when `positive_root` is set.
    /// `None` leaves the value to the general algebraic cascade.
    pub fn sign_polynomial_at_rational_square_root(
        coefficients: &[Self],
        square: &Rational,
        positive_root: bool,
    ) -> Option<RealSign> {
        let coefficients = coefficients
            .iter()
            .map(Self::tower_computable)
            .collect::<Vec<_>>();
        Computable::sign_polynomial_at_rational_square_root(&coefficients, square, positive_root)
    }
}

#[cfg(test)]
mod quadratic_tower_real_tests {
    use super::*;

    #[test]
    fn polynomial_vanishes_at_both_square_roots_of_one_half() {
        let half = Rational::fraction(1, 2).unwrap();
        let coefficients = [Real::from(half.clone().neg()), Real::zero(), Real::one()];
        assert_eq!(
            Real::sign_polynomial_at_rational_square_root(&coefficients, &half, true),
            Some(RealSign::Zero)
        );
        assert_eq!(
            Real::sign_polynomial_at_rational_square_root(&coefficients, &half, false),
            Some(RealSign::Zero)
        );
    }

    #[test]
    fn polynomial_linear_term_follows_the_selected_square_root() {
        let half = Rational::fraction(1, 2).unwrap();
        let coefficients = [Real::zero(), Real::one()];
        assert_eq!(
            Real::sign_polynomial_at_rational_square_root(&coefficients, &half, true),
            Some(RealSign::Positive)
        );
        assert_eq!(
            Real::sign_polynomial_at_rational_square_root(&coefficients, &half, false),
            Some(RealSign::Negative)
        );
    }

    #[test]
    fn scaled_nested_normal_square_cancels() {
        let alpha = Real::from(Rational::fraction(1, 2).unwrap())
            .sqrt()
            .unwrap();
        let radicand = Real::one() + Real::from(4) * &alpha;
        let norm = radicand.clone().sqrt().unwrap();
        let quotient = (&radicand / &(norm.clone() * &norm)).unwrap();
        let difference = quotient - Real::one();
        assert_eq!(difference.quadratic_tower_sign(), Some(RealSign::Zero));
    }
}
