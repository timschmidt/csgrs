impl Real {
    /// Rebuild this value from its exact quadratic-tower reduction.
    ///
    /// Values in `Q(sqrt(d))(sqrt(a + b sqrt(d)))` can accumulate a large
    /// arithmetic graph while their reduced field basis remains fixed. This
    /// returns an equal, shallow `Real`, including a rational payload when the
    /// radicals cancel. Principal-root branches and the outer rational scale
    /// are preserved; no numerical approximation is used.
    ///
    /// `None` means this reduction did not cover the expression. It
    /// does not limit the original exact value or its subsequent operations.
    pub fn compact_quadratic_tower(&self) -> Option<Self> {
        if self.exact_rational_ref().is_some() {
            return Some(self.clone());
        }
        let [even, odd, radicand] = self.computable_ref().quadratic_tower_parts()?;
        let odd_is_zero = odd[0].is_zero() && odd[1].is_zero();
        let quadratic = |[constant, scale, square]: [Rational; 3]| -> Option<Self> {
            let constant = Self::new(constant);
            if scale.is_zero() {
                Some(constant)
            } else {
                Some(constant + Self::new(scale) * Self::new(square).sqrt().ok()?)
            }
        };
        let mut compact = quadratic(even)?;
        if !odd_is_zero {
            let radicand = quadratic(radicand)?;
            // Retain its algebraic sign before the public square-root domain
            // check, so reconstruction cannot start scalar refinement.
            radicand.quadratic_tower_sign()?;
            compact += quadratic(odd)? * radicand.sqrt().ok()?;
        }
        compact *= Self::new(self.rational.clone());
        if let Some(signal) = self.abort_signal() {
            compact.abort(signal.clone());
        }
        Some(compact)
    }

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
        let scale_sign = real_sign_from_num(self.rational.sign());
        if scale_sign == RealSign::Zero || matches!(self.class, One) {
            return Some(scale_sign);
        }
        // Query the shared payload so its reduction survives this call. The
        // outer rational factor changes only the sign, not the required field.
        multiply_public_sign(
            Some(scale_sign),
            self.computable_ref().quadratic_tower_sign(),
        )
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
    /// outside the supported tower reduction.
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
    fn compact_towers_preserve_nested_roots_after_repeated_arithmetic() {
        let inner = Real::from(5).sqrt().unwrap();
        let outer = (Real::from(3) + &inner).sqrt().unwrap();
        let expected = Real::one() + &inner + (Real::from(2) - &inner) * &outer;
        let divisor = &outer + Real::one();
        let mut value = expected.clone();
        for _ in 0..8 {
            value = (((&value + Real::one()) * &divisor - &divisor) / &divisor).unwrap();
        }
        for scale in [
            Rational::fraction(7, 13).unwrap(),
            Rational::fraction(-7, 13).unwrap(),
        ] {
            let scaled = &value * Real::new(scale.clone());
            let compact = scaled.compact_quadratic_tower().unwrap();
            assert!(compact.exact_rational_ref().is_none());
            assert_eq!(
                (&compact - &expected * Real::new(scale)).quadratic_tower_sign(),
                Some(RealSign::Zero),
            );
            let again = compact.compact_quadratic_tower().unwrap();
            assert_eq!(
                (again - compact).quadratic_tower_sign(),
                Some(RealSign::Zero),
            );
        }
    }

    #[test]
    fn compact_towers_promote_proven_rationals_and_leave_other_fields_available() {
        let a = Real::from(3).sqrt().unwrap();
        let b = Real::from(7).sqrt().unwrap();
        let sum = &a + &b;
        let rational = &sum * &sum - Real::from(2) * Real::from(21).sqrt().unwrap();
        let compact = rational.compact_quadratic_tower().unwrap();
        assert_eq!(compact.exact_rational_ref(), Some(&Rational::new(10)));
        let zero = rational - Real::from(10);
        assert_eq!(
            zero.compact_quadratic_tower().unwrap().exact_rational_ref(),
            Some(&Rational::zero()),
        );

        let outside = a + b + Real::from(2).sqrt().unwrap();
        assert!(outside.compact_quadratic_tower().is_none());
        assert_eq!(outside.immediate_sign(), Some(RealSign::Positive));
        assert!(Real::pi().compact_quadratic_tower().is_none());
    }

    #[test]
    fn scaled_tower_sign_retains_the_shared_payload_proof() {
        let root = Real::from(3).sqrt().unwrap() + Real::from(7).sqrt().unwrap();
        let expanded = Real::from(10) + Real::from(2) * Real::from(21).sqrt().unwrap();
        let difference = &root * &root - expanded;
        for scale in [
            Rational::fraction(7, 13).unwrap(),
            Rational::fraction(-7, 13).unwrap(),
        ] {
            let value = &difference * Real::from(scale);
            let shared = value.clone();
            assert_eq!(value.quadratic_tower_sign(), Some(RealSign::Zero));
            assert_eq!(shared.immediate_sign(), Some(RealSign::Zero));
            for delta in [Real::one(), -Real::one()] {
                let shifted = &value + &delta;
                assert_eq!(shifted.quadratic_tower_sign(), delta.immediate_sign());
            }
        }
    }

    #[test]
    fn biquadratic_bases_share_sums_products_and_inverses() {
        for (a, b) in [(3, 7), (2, 5), (11, 13)] {
            let x = Real::from(a).sqrt().unwrap();
            let y = Real::from(b).sqrt().unwrap();
            let xy = Real::from(a * b).sqrt().unwrap();
            for (x, y) in [(&x, &y), (&y, &x)] {
                let sum = x + y;
                assert_eq!(sum.quadratic_tower_sign(), Some(RealSign::Positive));
                let mixed = Real::one() + y;
                let product = &mixed * x;
                let cancellation = &product * x - (x * x) * &mixed;
                assert_eq!(cancellation.quadratic_tower_sign(), Some(RealSign::Zero));
                let expanded = Real::from(a + b) + Real::from(2) * &xy;
                let difference = &sum * &sum - expanded;
                assert_eq!(difference.quadratic_tower_sign(), Some(RealSign::Zero));
                let full = Real::one() + x + y + &xy;
                let factored = (Real::one() + x) * (Real::one() + y);
                assert_eq!(
                    (&full - &factored).quadratic_tower_sign(),
                    Some(RealSign::Zero)
                );
                let reciprocal = (Real::one() / &full).unwrap();
                let difference = reciprocal * factored - Real::one();
                assert_eq!(difference.quadratic_tower_sign(), Some(RealSign::Zero));
                for (delta, expected) in [
                    (Real::one(), RealSign::Positive),
                    (-Real::one(), RealSign::Negative),
                ] {
                    assert_eq!((&difference + delta).quadratic_tower_sign(), Some(expected));
                }
            }
        }
        let outside = Real::from(2).sqrt().unwrap()
            + Real::from(3).sqrt().unwrap()
            + Real::from(5).sqrt().unwrap();
        assert_eq!(outside.quadratic_tower_sign(), None);
    }

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
