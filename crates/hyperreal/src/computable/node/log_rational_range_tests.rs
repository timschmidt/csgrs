#[cfg(test)]
mod log_rational_range_tests {
    use super::*;
    use rug::{Float, Integer, Rational as Q, float::Round};

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
