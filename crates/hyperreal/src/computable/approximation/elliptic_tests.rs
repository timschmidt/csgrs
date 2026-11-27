use super::*;

#[test]
/// Check 3,520 finite directed quotient cases plus coarse and preexisting-abort kernel requests.
/// Use exact Rational expectations for numerators 0..19, denominators 1..11 and
/// shifts -8..7. Both RF/RD requests encode physical arguments (1,1,1), not a
/// singular (1,0,0). Require zero at maximum coarse precision and after an already
/// asserted stop; these cases do not measure active cancellation latency.
fn directed_quotients_and_coarse_requests_keep_their_bounds() {
    for numerator in 0..20_u32 {
        for denominator in 1..12_u32 {
            for shift in -8..8 {
                let [lo, hi] = elliptic_quotient(numerator.into(), denominator.into(), shift);
                let exact = Rational::fraction(numerator.into(), denominator.into()).unwrap()
                    * Rational::new(2).powi(shift.into()).unwrap();
                assert!(Rational::from_bigint(BigInt::from(lo)) <= exact);
                assert!(exact <= Rational::from_bigint(BigInt::from(hi)));
            }
        }
    }
    let arguments = [Computable::one(), Computable::zero(), Computable::zero()];
    for second in [false, true] {
        assert_eq!(
            elliptic_carlson(&None, &arguments, second, i32::MAX),
            BigInt::zero()
        );
        let signal = Some(std::sync::Arc::new(std::sync::atomic::AtomicBool::new(
            true,
        )));
        assert_eq!(
            elliptic_carlson(&signal, &arguments, second, -512),
            BigInt::zero()
        );
    }
}

#[test]
/// Compare admitted mean bounds to analytic repeated-argument RF formulas evaluated with MPFR.
/// Try 972 combinations of six x/y values, three endpoint uncertainties, three
/// even grids and three output grids; skipped candidates remain unqualified.
/// Require more than 500 admitted cases without asserting an exact count.
/// Use monotonicity to compare each interval's two corners at 768-bit working
/// precision. This independent nearest-rounded oracle is not a directed MPFR
/// proof or an exhaustive enclosure test. Register thread-local backend cleanup.
fn rf_mean_enclosure_matches_independent_repeated_argument_integrals() {
    use rug::Float;
    crate::mpfr_test::cleanup_on_thread_exit();
    // RF(x,y,y) reduces directly to an inverse circular/hyperbolic function.
    // No duplication or mean/spread bound is used by this reference.
    /// Evaluate RF(x,y,y) on a positive repeated-argument fixture at physical scale `2^q`.
    /// Use reciprocal square root on the diagonal, acos when `x < y` and acosh when `x > y`,
    /// then apply homogeneity by `2^(-q/2)`. The test supplies even q and positive
    /// integers. This 768-bit MPFR formula does not use Carlson duplication or the
    /// mean/spread enclosure being checked.
    fn reference(x: u32, y: u32, q: i32) -> Float {
        let mut value = if x == y {
            Float::with_val(768, y).sqrt().recip()
        } else if x < y {
            (Float::with_val(768, x) / y).sqrt().acos() / Float::with_val(768, y - x).sqrt()
        } else {
            (Float::with_val(768, x) / y).sqrt().acosh() / Float::with_val(768, x - y).sqrt()
        };
        value >>= q / 2;
        value
    }
    let mut admitted = 0;
    for x in 1..7_u32 {
        for y in 1..7_u32 {
            for uncertainty in [0, 1, 3] {
                let bounds = [x, y, y].map(|v| [v.into(), (v + uncertainty).into()]);
                for q in [-8, 0, 8] {
                    for output_grid in [-32, -3, 4] {
                        let Some([lo, hi]) = elliptic_rf_mean_enclosure(&bounds, q, output_grid)
                        else {
                            continue;
                        };
                        admitted += 1;
                        let physical = |n: BigUint| {
                            let n: rug::Integer = n.to_string().parse().unwrap();
                            let mut n = Float::with_val(768, n);
                            n <<= output_grid as i32;
                            n
                        };
                        assert!(physical(lo) <= reference(x + uncertainty, y + uncertainty, q));
                        assert!(physical(hi) >= reference(x, y, q));
                    }
                }
            }
        }
    }
    assert!(admitted > 500);
}

#[test]
/// Require a two-unit enclosure near the diagonal and decline three zero/wide domains.
/// Use relative spread 2^-66 around physical mean one, and check that the
/// -128-grid interval also contains the diagonal reference one. The slightly
/// spread RF is not asserted equal to that reference. Rejection fixtures exercise
/// only mean-bound applicability, not invalidity of their underlying RF values.
fn rf_mean_enclosure_resolves_small_spreads_and_declines_wide_domains() {
    let centre = BigUint::one() << 256_usize;
    let spread = BigUint::one() << 190_usize;
    let bounds = [&centre - &spread, centre.clone(), &centre + &spread].map(|v| [v.clone(), v]);
    let [lo, hi] = elliptic_rf_mean_enclosure(&bounds, -256, -128).unwrap();
    // Relative deviations are 2^-66: the whole-integral error is quadratic,
    // so this enclosure already meets the ordinary two-unit stopping gate.
    assert!(&hi - &lo <= BigUint::from(2_u8));
    assert!(lo <= BigUint::one() << 128_usize);
    assert!(hi >= BigUint::one() << 128_usize);
    for values in [[0_u32, 0, 0], [0, 1, 1], [1, 1, 3]] {
        let bounds = values.map(|v| [v.into(), v.into()]);
        assert!(elliptic_rf_mean_enclosure(&bounds, -8, -32).is_none());
    }
}
