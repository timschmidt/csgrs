#[cfg(test)]
mod atan_pi_anchor_tests {
    use super::*;

    fn anchors() -> [(Computable, Rational); 3] {
        let three = Computable::rational(Rational::new(3)).sqrt();
        [
            (Computable::one(), Rational::fraction(1, 2).unwrap()),
            (three.clone(), Rational::fraction(3, 4).unwrap()),
            (three.inverse(), Rational::fraction(1, 4).unwrap()),
        ]
    }

    fn assert_square(value: Computable, expected: &Rational) {
        assert_eq!(
            value
                .square()
                .add(Computable::rational(-expected.clone()))
                .sign_until(0),
            Some(RealSign::Zero)
        );
    }

    #[test]
    fn atan_pi_anchors_preserve_facts_through_periods_and_complements() {
        let periods = [
            -(BigInt::one() << 256_usize),
            BigInt::from(-17),
            BigInt::from(-1),
            BigInt::zero(),
            BigInt::one(),
            BigInt::from(16),
            (BigInt::one() << 256_usize) + BigInt::one(),
        ];
        for (argument, sine_square) in anchors() {
            let cosine_square = Rational::one() - sine_square.clone();
            for sign in [-1, 1] {
                for period in &periods {
                    let principal = argument
                        .clone()
                        .multiply_rational(Rational::new(sign))
                        .atan();
                    let angle = principal.add(
                        Computable::pi().multiply_rational(Rational::from_bigint(period.clone())),
                    );
                    let parity = if period.bit(0) { -1 } else { 1 };
                    for (composed, squares, signs) in [
                        (
                            angle.clone(),
                            [sine_square.clone(), cosine_square.clone()],
                            [sign * parity, parity],
                        ),
                        (
                            Computable::pi().shift_right(1).add(angle.clone().negate()),
                            [cosine_square.clone(), sine_square.clone()],
                            [parity, sign * parity],
                        ),
                        (
                            Computable::pi()
                                .shift_right(1)
                                .negate()
                                .add(angle.clone().negate()),
                            [cosine_square.clone(), sine_square.clone()],
                            [-parity, -sign * parity],
                        ),
                    ] {
                        assert!(composed.cached().is_none());
                        for warm in [false, true] {
                            if warm {
                                let _ = composed.approx(-128);
                            }
                            for (i, value) in [composed.clone().sin(), composed.clone().cos()]
                                .into_iter()
                                .enumerate()
                            {
                                assert_eq!(
                                    value.sign_until(0),
                                    Some(if signs[i] > 0 {
                                        RealSign::Positive
                                    } else {
                                        RealSign::Negative
                                    })
                                );
                                assert_square(value.clone(), &squares[i]);
                                if !warm {
                                    assert!(composed.cached().is_none());
                                }
                                let _ = value.approx(-256);
                                assert_square(value, &squares[i]);
                            }
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn atan_pi_anchor_reader_declines_unproved_correspondences() {
        for denominator in [1, 2, 5, 7, 8, 12, 1009] {
            let angle =
                Computable::pi().multiply_rational(Rational::fraction(1, denominator).unwrap());
            assert!(angle.pi_atan_trig_argument(true).is_none());
        }
        let epsilon = Rational::from_bigint_fraction(BigInt::one(), BigUint::one() << 512).unwrap();
        for (argument, square) in anchors() {
            let principal = argument.atan();
            for sign in [-1, 1] {
                let offset = &epsilon * Rational::new(sign);
                let angle = principal.clone().add(Computable::rational(offset));
                assert!(angle.pi_atan_trig_argument(true).is_none());
                let delta = angle
                    .sin()
                    .square()
                    .add(Computable::rational(-square.clone()));
                // The positive principal anchors are inside (0, pi/2), where
                // sine squared is strictly increasing. The offset is nonzero.
                assert_ne!(delta.sign_until(0), Some(RealSign::Zero));
                assert_eq!(
                    delta.sign_until(-1024),
                    Some(if sign > 0 {
                        RealSign::Positive
                    } else {
                        RealSign::Negative
                    })
                );
            }
        }
        let doubled = Computable::rational(Rational::fraction(1, 2).unwrap())
            .atan()
            .shift_left(1);
        assert!(doubled.pi_atan_trig_argument(true).is_none());
        let offset = Computable::pi().shift_right(2).add(Computable::one());
        assert!(offset.pi_atan_trig_argument(true).is_none());
    }

    #[test]
    fn atan_pi_anchor_facts_survive_concurrent_refinement() {
        let values: Vec<_> = anchors()
            .into_iter()
            .map(|(argument, square)| (argument.atan(), square))
            .collect();
        let threads: Vec<_> = (0..16)
            .map(|worker| {
                let values = values.clone();
                std::thread::spawn(move || {
                    for (angle, sine_square) in values {
                        if worker % 2 == 0 {
                            let _ = angle.approx(-128);
                        }
                        assert_square(angle.clone().sin(), &sine_square);
                        assert_square(angle.cos(), &(Rational::one() - sine_square));
                    }
                })
            })
            .collect();
        for thread in threads {
            thread.join().unwrap();
        }
        for (angle, sine_square) in values {
            assert_square(angle.clone().sin(), &sine_square);
            assert_square(angle.cos(), &(Rational::one() - sine_square));
        }
    }
}
