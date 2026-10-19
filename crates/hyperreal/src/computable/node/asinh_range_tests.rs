#[cfg(test)]
mod asinh_range_tests {
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

    fn oracle(q: &Q) -> [Q; 2] {
        let mut lo = Float::with_val_round(2048, q, Round::Down).0;
        let mut hi = Float::with_val_round(2048, q, Round::Up).0;
        lo.asinh_round(Round::Down);
        hi.asinh_round(Round::Up);
        [lo.to_rational().unwrap(), hi.to_rational().unwrap()]
    }

    fn encloses(value: &Computable, p: i32, bounds: &[Q; 2]) {
        let a = Integer::from_str_radix(&value.approx(p).to_string(), 10).unwrap();
        let unit = if p > 0 {
            Q::from(Integer::from(1) << p)
        } else {
            Q::from((1, Integer::from(1) << -p))
        };
        let center = Q::from(a) * &unit;
        assert!(
            Q::from(&center - &unit) <= bounds[0] && Q::from(&center + &unit) >= bounds[1],
            "p={p}, center={center}, unit={unit}"
        );
    }

    #[test]
    fn both_asinh_series_reduce_large_inputs_and_preserve_small_inputs() {
        crate::mpfr_test::cleanup_on_thread_exit();
        let mut inputs = vec![Q::from(0)];
        for q in [
            Q::from((1, 16)),
            Q::from((1, 3)),
            Q::from((127, 256)),
            Q::from((1, 2)),
            Q::from((129, 256)),
            Q::from((3, 4)),
            Q::from(1),
            Q::from(4),
            Q::from(1000),
            Q::from(Integer::from(1) << 512),
            Q::from((1, Integer::from(1) << 512)),
        ] {
            inputs.extend([q.clone(), -q]);
        }
        for q in inputs {
            let bounds = oracle(&q);
            let r: Rational = q.to_string().parse().unwrap();
            for rational in [false, true] {
                let build = || {
                    raw(if rational {
                        Approximation::AsinhRational(r.clone())
                    } else {
                        Approximation::PrescaledAsinh(Computable::rational(r.clone()))
                    })
                };
                let warm = build();
                for p in [1, 0, -1, -32, -128, -512, -8, 1] {
                    encloses(&build(), p, &bounds);
                    encloses(&warm, p, &bounds);
                }
            }
        }
    }

    #[test]
    fn asinh_range_proof_accounts_for_every_allowed_sample_rounding() {
        crate::mpfr_test::cleanup_on_thread_exit();
        let mut checks = 0;
        // The original series' actual working precisions, including the
        // coarse request whose argument is now sampled before returning zero.
        for (p, s) in [
            (1_i32, -12_i32),
            (0, -12),
            (-1, -13),
            (-32, -46),
            (-128, -144),
        ] {
            let threshold = Integer::from(1) << (-s - 1);
            for delta in -8_i32..=8 {
                for negative in [false, true] {
                    let q = Q::from((threshold.clone() * 4 + delta, Integer::from(1) << (-s + 2)));
                    let q = if negative { -q } else { q };
                    let base = threshold.clone() + delta.div_euclid(4);
                    let base = if negative { -base } else { base };
                    let bounds = oracle(&q);
                    for rounding in -1..=1 {
                        let sample: Integer = base.clone() + rounding;
                        let unit = Q::from((1, Integer::from(1) << -s));
                        if (Q::from(sample.clone()) * &unit - &q).abs() > unit {
                            continue;
                        }
                        let input = raw(Approximation::Add(
                            Computable::rational(q.to_string().parse().unwrap()),
                            Computable::zero(),
                        ));
                        input
                            .internal
                            .store_cache_value(s, sample.to_string().parse().unwrap());
                        let value = raw(Approximation::PrescaledAsinh(input));
                        for precision in [p, -256, p] {
                            encloses(&value, precision, &bounds);
                            checks += 1;
                        }
                    }
                }
            }
        }
        assert_eq!(checks, 1170);
    }
}
