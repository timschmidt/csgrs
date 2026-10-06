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
                    assert_eq!(i64::from(msd_from_appr(precision, &input)), expected);
                }
            }
        }
    }
}

#[test]
fn approximation_msd_does_not_wrap_an_unrepresentable_exponent() {
    for input in [BigInt::from(-2), BigInt::from(2)] {
        assert!(std::panic::catch_unwind(|| msd_from_appr(i32::MAX, &input)).is_err());
    }
}
