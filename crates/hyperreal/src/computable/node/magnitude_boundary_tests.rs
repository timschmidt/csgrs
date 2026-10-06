#[cfg(test)]
mod magnitude_boundary_tests {
    use super::*;

    #[test]
    fn absent_nonzero_magnitude_does_not_certify_zero() {
        for sign in [Sign::Plus, Sign::Minus] {
            let bound = BoundInfo::with_sign(sign, None);
            assert_eq!(bound.known_sign(), Some(sign));
            for result in [
                bound,
                bound.negate(),
                bound.inverse(),
                bound.square(),
                bound.map_msd(|exponent| exponent.checked_add(1)),
                bound.multiply(BoundInfo::with_sign(Sign::Plus, Some(0))),
                bound.add(BoundInfo::Zero),
            ] {
                assert_ne!(result, BoundInfo::Zero);
                assert_eq!(result.known_msd(), None);
                assert_eq!(result.planning_msd(), None);
                assert_eq!(result.magnitude_bits(), None);
            }
        }
        assert_eq!(BoundInfo::Zero.known_msd(), Some(None));
        assert_eq!(BoundInfo::Zero.planning_msd(), Some(None));
    }

    #[test]
    fn bound_transforms_check_the_complete_exponent() {
        let exponents = [
            i32::MIN,
            i32::MIN + 1,
            -1_073_741_824,
            -1,
            0,
            1,
            i32::MAX / 2,
            i32::MAX,
        ];
        for exponent in exponents {
            let bound = BoundInfo::with_sign(Sign::Plus, Some(exponent));
            let inverse = bound.inverse();
            assert_eq!(
                inverse.magnitude_bits().map(|bits| bits.msd),
                i32::try_from(1_i64 - i64::from(exponent)).ok()
            );
            assert_eq!(inverse.known_sign(), Some(Sign::Plus));
            let square = bound.square();
            assert_eq!(
                square.magnitude_bits().map(|bits| bits.msd),
                i32::try_from(2 * i64::from(exponent)).ok()
            );
            assert_eq!(square.known_sign(), Some(Sign::Plus));
            assert_eq!(
                bound.sqrt().magnitude_bits(),
                Some(MagnitudeBits {
                    msd: exponent.div_euclid(2),
                    exact_msd: true
                })
            );
            for offset in exponents {
                let expected = i32::try_from(i64::from(exponent) + i64::from(offset)).ok();
                let shifted = bound.map_msd(|msd| msd.checked_add(offset));
                assert_eq!(shifted.magnitude_bits().map(|bits| bits.msd), expected);
                assert_eq!(shifted.known_sign(), Some(Sign::Plus));
                let product = bound.multiply(BoundInfo::with_sign(Sign::Minus, Some(offset)));
                assert_eq!(product.magnitude_bits().map(|bits| bits.msd), expected);
                assert_eq!(product.known_sign(), Some(Sign::Minus));
            }
        }
        assert_eq!(
            BoundInfo::with_sign(Sign::Plus, Some(i32::MAX))
                .add(BoundInfo::with_sign(Sign::Minus, Some(i32::MAX))),
            BoundInfo::Unknown,
        );
    }

    #[test]
    fn approximation_bound_retains_sign_without_wrapped_magnitude() {
        for (precision, integer, expected) in [
            (i32::MAX, 2, None),
            (i32::MAX, -2, None),
            (i32::MAX - 1, 2, Some(i32::MAX)),
            (i32::MIN, 2, Some(i32::MIN + 1)),
        ] {
            let value = BigInt::from(integer);
            let bound = Computable::bound_from_approx(precision, &value);
            assert_eq!(bound.known_sign(), Some(value.sign()));
            assert_eq!(bound.magnitude_bits().map(|bits| bits.msd), expected);
            assert_eq!(bound.known_msd(), None);
            assert_eq!(bound.planning_msd(), expected.map(Some));
        }
    }
}
