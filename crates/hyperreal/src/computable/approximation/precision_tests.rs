//! Boundary regressions for converting integer-sample bit lengths to precision exponents.
//! These test the checked exponent helper, not every kernel at every `i32` precision.

use super::*;

#[test]
/// Check both signs and the first/last integers of nine selected bit widths.
/// Seven precision choices per width include representable extreme exponents;
/// all 252 comparisons use an independent `i64` addition for the expected result.
/// Duplicate endpoint patterns at width one remain intentional loop executions.
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
/// Require a panic for each sign of the two-bit sample at maximum precision.
/// Its exponent is `i32::MAX + 1`; the checked helper must reject it rather than
/// wrap. This verifies the existing panic contract, not a typed error outcome.
fn approximation_msd_does_not_wrap_an_unrepresentable_exponent() {
    for input in [BigInt::from(-2), BigInt::from(2)] {
        assert!(std::panic::catch_unwind(|| msd_from_appr(i32::MAX, &input)).is_err());
    }
}
