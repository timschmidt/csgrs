#[cfg(test)]
mod radical_eighth_angle_tests {
    use super::*;

    fn raw(approximation: Approximation) -> Computable {
        Computable {
            internal: Arc::new(Node::new(
                approximation,
                BoundCache::Invalid,
                ExactSignCache::Invalid,
            )),
            signal: None,
        }
    }
    fn number(n: i32) -> Computable {
        raw(Approximation::Ratio(Rational::from(n)))
    }
    fn add(a: Computable, b: Computable) -> Computable {
        raw(Approximation::Add(a, b))
    }
    fn mul(a: Computable, b: Computable) -> Computable {
        raw(Approximation::Multiply(a, b))
    }
    fn neg(a: Computable) -> Computable {
        raw(Approximation::Negate(a))
    }
    fn square(a: Computable) -> Computable {
        mul(a.clone(), a)
    }
    fn sine(n: i64, d: u64) -> Computable {
        let pi = raw(Approximation::Constant(SharedConstant::Pi));
        let argument = if (n, d) == (1, 8) {
            raw(Approximation::Offset(pi, -3))
        } else {
            mul(
                pi,
                raw(Approximation::Ratio(Rational::fraction(n, d).unwrap())),
            )
        };
        raw(Approximation::PrescaledSin(argument))
    }

    #[test]
    fn eighth_angle_tower_proves_selected_roots_and_products() {
        let (x, y) = (sine(1, 8), sine(3, 8));
        let q = raw(Approximation::Constant(SharedConstant::Sqrt2));
        let identities = [
            add(add(square(x.clone()), square(y.clone())), number(-1)),
            add(
                add(mul(number(4), square(x.clone())), q.clone()),
                number(-2),
            ),
            add(
                add(mul(number(4), square(y.clone())), neg(q.clone())),
                number(-2),
            ),
            add(mul(number(4), mul(x.clone(), y.clone())), neg(q)),
        ];
        for identity in identities {
            for (residual, sign) in [
                (-7, RealSign::Negative),
                (0, RealSign::Zero),
                (11, RealSign::Positive),
            ] {
                let expression = add(identity.clone(), number(residual));
                assert_eq!(expression.radical_tower_sign(), Some(sign));
                assert_eq!(expression.radical_tower_sign(), Some(sign));
                assert!(expression.cached().is_none(), "proof must not refine");
            }
        }
        assert_eq!(x.radical_tower_sign(), Some(RealSign::Positive));
        assert_eq!(y.radical_tower_sign(), Some(RealSign::Positive));
        assert!(matches!(
            x.internal.approximation,
            Approximation::PrescaledSin(_)
        ));
        assert!(matches!(
            y.internal.approximation,
            Approximation::PrescaledSin(_)
        ));
    }

    #[test]
    fn eighth_angle_tower_proves_the_full_hull_determinant() {
        // Independently expanded determinant of (0,r,0), (a,a,-r),
        // (0,-rx,-ry), (a,a+ry,-rx): a*r²*(x²+y²-1).
        let (x, y) = (sine(1, 8), sine(3, 8));
        let a = raw(Approximation::Ratio(Rational::fraction(3, 2).unwrap()));
        let r = raw(Approximation::Ratio(Rational::fraction(2, 5).unwrap()));
        let expression = mul(
            mul(a, square(r)),
            add(add(square(x), square(y)), number(-1)),
        );
        assert_eq!(expression.radical_tower_sign(), Some(RealSign::Zero));
    }

    #[test]
    fn eighth_angle_tower_rejects_unsupported_leaves_and_invalid_domains() {
        for angle in [sine(1, 7), sine(5, 8), sine(9, 8), sine(1, 16)] {
            assert_eq!(angle.radical_tower_sign(), None);
            assert!(matches!(
                angle.internal.facts.exact_sign(),
                ExactSignCache::Invalid
            ));
        }
        let zero = add(add(square(sine(1, 8)), square(sine(3, 8))), number(-1));
        let inverse = raw(Approximation::Inverse(zero));
        assert_eq!(inverse.radical_tower_sign(), None);
        assert!(matches!(
            inverse.internal.facts.exact_sign(),
            ExactSignCache::Invalid
        ));
    }
}
