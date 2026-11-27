#[cfg(test)]
/// Independent directed-MPFR enclosures for raw rational ln(1+r) and binary-scaled logarithm nodes.
mod log_rational_range_tests {
    use super::*;
    use rug::{Float, Integer, Rational as Q, float::Round};

    /// Create a raw logarithm node with empty facts and no signal to exercise its stored residual/shift directly.
    fn raw(kind: Approximation) -> Computable {
        Computable {
            internal: Arc::new(Node::new(
                kind,
                BoundCache::Invalid,
                ExactSignCache::Invalid,
            )),
            signal: None,
        }
    }

    /// Enclose ln(1+residual) + shift*ln(2) with directed 2048-bit MPFR logarithms.
    /// Use exact GMP rational arithmetic for the integer-weighted combination and
    /// swap the ln(2) bounds for negative shifts. Fixtures require residual > -1;
    /// finite MPFR endpoints justify conversion unwraps. No Hyperreal logarithm supplies the reference.
    fn oracle(residual: &Q, shift: i32) -> [Q; 2] {
        let argument = Q::from(residual + 1);
        let mut lo = Float::with_val_round(2048, &argument, Round::Down).0;
        let mut hi = Float::with_val_round(2048, &argument, Round::Up).0;
        lo.ln_round(Round::Down);
        hi.ln_round(Round::Up);
        let mut ln2_lo = Float::with_val(2048, 2);
        let mut ln2_hi = Float::with_val(2048, 2);
        ln2_lo.ln_round(Round::Down);
        ln2_hi.ln_round(Round::Up);
        let mut scaled_lo = ln2_lo.to_rational().unwrap() * shift;
        let mut scaled_hi = ln2_hi.to_rational().unwrap() * shift;
        if shift < 0 {
            std::mem::swap(&mut scaled_lo, &mut scaled_hi);
        }
        [
            lo.to_rational().unwrap() + scaled_lo,
            hi.to_rational().unwrap() + scaled_hi,
        ]
    }

    /// Require the Hyperreal interval (approximation ±1)*2^p to contain the entire independent rational enclosure.
    /// Convert the integer through decimal text exactly and form the dyadic unit as a
    /// GMP rational. Fixture precisions avoid i32::MIN and enormous shifts; this helper
    /// is not a general precision-domain validator or a correctly rounded integer test.
    fn encloses(value: &Computable, p: i32, bounds: &[Q; 2]) {
        let approximation = Integer::from_str_radix(&value.approx(p).to_string(), 10).unwrap();
        let unit = if p > 0 {
            Q::from(Integer::from(1) << p)
        } else {
            Q::from((1, Integer::from(1) << -p))
        };
        let center = Q::from(approximation) * &unit;
        assert!(
            Q::from(&center - &unit) <= bounds[0] && Q::from(&center + &unit) >= bounds[1],
            "p={p}, center={center}, unit={unit}"
        );
    }

    /// Build fresh raw nodes and one reused node for ten precision requests from 2 down to -512 and back.
    /// Use None for ln(1+r) and Some for ln(1+r)+shift*ln(2), converting the fixture
    /// rational through exact text. Compare all twenty approximations with one
    /// independent enclosure; fresh nodes do not reset any process-wide constant caches.
    fn exercise(residual: &Q, shift: Option<i32>) {
        let bounds = oracle(residual, shift.unwrap_or(0));
        let r: Rational = residual.to_string().parse().unwrap();
        let build = || {
            raw(match shift {
                None => Approximation::PrescaledLnRational(r.clone()),
                Some(shift) => Approximation::BinaryScaledLnRational {
                    residual: r.clone(),
                    shift,
                },
            })
        };
        let warm = build();
        for p in [2, 1, 0, -1, -32, -128, -512, -8, 2, 0] {
            encloses(&build(), p, &bounds);
            encloses(&warm, p, &bounds);
        }
    }

    #[test]
    /// Exercise 22 exact residuals, including zero, both signs of eight small/near-unit values and five large positives.
    /// Check each in unscaled and zero-shift form through the twenty-query helper,
    /// for 880 enclosure assertions. Register MPFR cache cleanup on thread exit.
    fn rational_log_series_enclose_small_large_and_near_minus_one_residuals() {
        crate::mpfr_test::cleanup_on_thread_exit();
        let mut inputs = vec![Q::from(0)];
        for q in [
            Q::from((1, 16)),
            Q::from((1, 3)),
            Q::from((127, 256)),
            Q::from((1, 2)),
            Q::from((129, 256)),
            Q::from((3, 4)),
            Q::from(((Integer::from(1) << 512) - 1, Integer::from(1) << 512)),
            Q::from((1, Integer::from(1) << 512)),
        ] {
            inputs.extend([q.clone(), -q]);
        }
        inputs.extend([
            Q::from(1),
            Q::from(4),
            Q::from(1000),
            Q::from(65535),
            Q::from((Integer::from(1) << 512) - 1),
        ]);
        for residual in inputs {
            exercise(&residual, None);
            exercise(&residual, Some(0));
        }
    }

    #[test]
    /// Exercise eleven residuals at nine shifts, including both i32 endpoints.
    /// The shifts multiply ln(2); they do not request a binary integer allocation
    /// of that width. Check all 1980 fresh/warm enclosure assertions at the helper
    /// precisions and register MPFR thread-cache cleanup.
    fn binary_scaled_logs_preserve_residuals_and_full_width_shifts() {
        crate::mpfr_test::cleanup_on_thread_exit();
        for residual in [
            Q::from(0),
            Q::from((1, 16)),
            Q::from((-1, 16)),
            Q::from((1, 2)),
            Q::from((-1, 2)),
            Q::from((-15, 16)),
            Q::from(4),
            Q::from(65535),
            Q::from((-65535, 65536)),
            Q::from((Integer::from(1) << 512) - 1),
            Q::from((1 - (Integer::from(1) << 512), Integer::from(1) << 512)),
        ] {
            for shift in [i32::MIN, -4096, -17, -3, 0, 3, 17, 4096, i32::MAX] {
                exercise(&residual, Some(shift));
            }
        }
    }
}
