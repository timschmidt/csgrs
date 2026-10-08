
    #[test]
    fn quadratic_norms_preserve_wide_cancellation_and_products() {
        for a in -5..=5 {
            for b in -5..=5 {
                let value = Quad {
                    rational: Rational::new(a),
                    scale: Rational::new(b),
                    disc: Some(Rational::new(4)),
                };
                assert_eq!(value.sign(), Some(rational_sign(&Rational::new(a + 2 * b))));
            }
        }
        let epsilon = Rational::from_bigint(BigInt::one() << 1024).inverse().unwrap();
        for direction in [-1, 1] {
            let value = Quad {
                rational: Rational::new(2) + &epsilon * Rational::new(direction),
                scale: Rational::new(-1),
                disc: Some(Rational::new(4)),
            };
            let expected = if direction < 0 { RealSign::Negative } else { RealSign::Positive };
            assert_eq!(value.sign(), Some(expected));
            assert_eq!(value.neg().sign(), Some(if direction < 0 { RealSign::Positive } else { RealSign::Negative }));
        }
        for a in [-7, -1, 0, 1, 7] {
            for b in [-7, -1, 0, 1, 7] {
                let value = Quad {
                    rational: Rational::new(a),
                    scale: Rational::new(b),
                    disc: Some(Rational::new(5)),
                };
                let conjugate = Quad { scale: Rational::new(-b), ..value.clone() };
                let norm = value.clone().mul(conjugate).unwrap();
                assert_eq!(norm.rational, Rational::new(a * a - 5 * b * b));
                assert!(norm.scale.is_zero());
                if a != 0 || b != 0 {
                    let inverse = value.clone().inverse().unwrap();
                    let identity = value.mul(inverse).unwrap();
                    assert_eq!(identity.rational, Rational::one());
                    assert!(identity.scale.is_zero());
                }
            }
        }
    }

    #[test]
    fn tower_signs_preserve_equal_opposed_and_zero_coefficient_branches() {
        for (even, odd, expected) in [
            (0, 0, RealSign::Zero), (0, 1, RealSign::Positive),
            (0, -1, RealSign::Negative), (1, 1, RealSign::Positive),
            (-1, -1, RealSign::Negative), (1, -1, RealSign::Negative),
            (-1, 1, RealSign::Positive), (2, -1, RealSign::Positive),
            (-2, 1, RealSign::Negative),
        ] {
            let value = Tower {
                even: Quad::rational(Rational::new(even)),
                odd: Quad::rational(Rational::new(odd)),
                radicand: Some(Quad::rational(Rational::new(2))),
            };
            assert_eq!(value.sign(), Some(expected));
        }
        for direction in [-1, 1] {
            let value = Tower {
                even: Quad::rational(Rational::new(2 * direction)),
                odd: Quad::rational(Rational::new(-direction)),
                radicand: Some(Quad::rational(Rational::new(4))),
            };
            assert_eq!(value.sign(), Some(RealSign::Zero));
        }
        let cancelled = Quad { rational: Rational::new(-2), scale: Rational::one(), disc: Some(Rational::new(4)) };
        for (even, odd, expected) in [
            (cancelled.clone(), Quad::rational(Rational::new(-1)), RealSign::Negative),
            (Quad::rational(Rational::one()), cancelled, RealSign::Positive),
        ] {
            let value = Tower { even, odd, radicand: Some(Quad::rational(Rational::new(2))) };
            assert_eq!(value.sign(), Some(expected));
        }
    }
