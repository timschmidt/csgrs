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

    /// Rational center `s` when this value is `s + r√q` with `r > 0` and only
    /// one real choice of `q`. The conjugate `s - r√q` is the other real root.
    pub fn quadratic_tower_positive_rational_branch(&self) -> Option<Rational> {
        self.tower_computable()
            .quadratic_tower_positive_rational_branch()
    }

    /// Rational polynomial satisfied by a value in the quadratic tower.
    ///
    /// Coefficients run from low degree to high. `None` means the value is
    /// outside the tower or its integers exceed the reduction bound.
    pub fn quadratic_tower_annihilating_polynomial(&self) -> Option<Vec<Rational>> {
        self.tower_computable()
            .quadratic_tower_annihilating_polynomial()
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
    fn independently_normalized_chord_normals_replay_exactly() {
        let q = |n: i32, d: i32| (Real::from(n) / Real::from(d)).unwrap();
        for (a, b) in [(11, 13), (3, 7), (2, 5)] {
            let alpha = q(1, a).sqrt().unwrap();
            let beta = q(1, b).sqrt().unwrap();
            let dx = Real::one() - &alpha;
            let speed = (&dx * &dx + &beta * &beta).sqrt().unwrap();
            let reduced_square = Real::one() + q(1, a) + q(1, b) - Real::from(2) * &alpha;
            let other_speed = reduced_square.clone().sqrt().unwrap();
            let nx = (-&beta / &speed).unwrap();
            let ny = (&dx / &speed).unwrap();
            for difference in [
                &nx + (&beta / &other_speed).unwrap(),
                &nx * &nx * &reduced_square - &beta * &beta,
                &nx * &nx + &ny * &ny - Real::one(),
            ] {
                assert_eq!(difference.quadratic_tower_sign(), Some(RealSign::Zero));
                assert_eq!((-&difference).quadratic_tower_sign(), Some(RealSign::Zero));
                let epsilon = q(1, 1 << 20);
                assert_eq!(
                    (&difference + &epsilon).quadratic_tower_sign(),
                    Some(RealSign::Positive)
                );
                assert_eq!(
                    (&difference - epsilon).quadratic_tower_sign(),
                    Some(RealSign::Negative)
                );
            }
        }
    }

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
    fn shifted_fourth_root_has_a_rational_annihilator() {
        let beta = Real::from(Rational::fraction(1, 2).unwrap())
            .sqrt()
            .unwrap()
            .sqrt()
            .unwrap();
        let value = Real::from(2) + &beta;
        assert_eq!(
            value.quadratic_tower_positive_rational_branch(),
            Some(Rational::new(2))
        );
        assert_eq!(
            value.quadratic_tower_annihilating_polynomial(),
            Some(vec![
                Rational::fraction(31, 2).unwrap(),
                Rational::new(-32),
                Rational::new(24),
                Rational::new(-8),
                Rational::one(),
            ])
        );
        assert_eq!(
            beta.quadratic_tower_annihilating_polynomial(),
            Some(vec![
                Rational::fraction(-1, 2).unwrap(),
                Rational::zero(),
                Rational::zero(),
                Rational::zero(),
                Rational::one(),
            ])
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
