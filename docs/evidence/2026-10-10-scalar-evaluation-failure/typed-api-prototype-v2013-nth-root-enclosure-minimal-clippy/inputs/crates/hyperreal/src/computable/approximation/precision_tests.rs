use super::*;

#[test]
fn approximation_msd_preserves_representable_endpoints() {
    for bits in [1_u32, 2, 3, 31, 32, 63, 64, 128, 256] {
        let first = BigInt::one() << (bits - 1);
        let last = (&first << 1) - BigInt::one();
        for precision in [
            i32::MIN,
            i32::MIN + 1,
            -129,
            -1,
            0,
            1,
            i32::MAX - (bits as i32 - 1),
        ] {
            let expected = i64::from(precision) + i64::from(bits) - 1;
            for magnitude in [&first, &last] {
                for input in [magnitude.clone(), -magnitude] {
                    assert_eq!(
                        i64::from(msd_from_appr(precision, &input).unwrap()),
                        expected
                    );
                }
            }
        }
    }
}

#[test]
fn approximation_msd_does_not_wrap_an_unrepresentable_exponent() {
    for input in [BigInt::from(-2), BigInt::from(2)] {
        assert_eq!(msd_from_appr(i32::MAX, &input), Err(Problem::OutOfRange));
    }
}

#[test]
fn direct_nth_root_bounds_preserve_small_enclosures() {
    for degree in 3..=Computable::MAX_DIRECT_NTH_ROOT_DEGREE {
        for sample in 2_u32..=1_024 {
            let sample = BigUint::from(sample);
            let (lower, upper) = integer_nth_root_bounds(&sample, degree);
            let low = &sample - BigUint::one();
            let high = &sample + BigUint::one();
            // Check the defining floor/ceiling inequalities independently of
            // the branch used to recover the lower endpoint.
            assert!(lower.pow(degree) <= low);
            assert!((&lower + BigUint::one()).pow(degree) > low);
            assert!(upper.pow(degree) >= high);
            assert!((&upper - BigUint::one()).pow(degree) < high);
        }
    }
}

#[test]
fn direct_nth_root_bounds_preserve_wide_power_neighbors() {
    for degree in 3..=Computable::MAX_DIRECT_NTH_ROOT_DEGREE {
        let full_size_root_bits = 65_568_u32.div_ceil(degree) as usize;
        for root_bits in [
            1_usize,
            2,
            7,
            8,
            15,
            16,
            31,
            32,
            63,
            64,
            127,
            128,
            511,
            1_024,
            4_096,
            full_size_root_bits,
        ] {
            for low_bits in [7_u32, 8] {
                let root = (BigUint::one() << root_bits) + BigUint::from(low_bits);
                let perfect_power = root.pow(degree);
                for delta in -2_i32..=2 {
                    let sample = if delta < 0 {
                        &perfect_power - BigUint::from(delta.unsigned_abs())
                    } else {
                        &perfect_power + BigUint::from(delta as u32)
                    };
                    let (lower, upper) = integer_nth_root_bounds(&sample, degree);
                    // Retain the previous two-root construction as a
                    // differential oracle for both exact integer endpoints.
                    let reference_lower = (&sample - BigUint::one()).nth_root(degree);
                    let high = &sample + BigUint::one();
                    let mut reference_upper = high.nth_root(degree);
                    if reference_upper.pow(degree) < high {
                        reference_upper += BigUint::one();
                    }
                    assert!(
                        lower == reference_lower && upper == reference_upper,
                        "degree {degree}, root bits {root_bits}, low bits {low_bits}, delta {delta}"
                    );
                }
            }
        }
    }
}
