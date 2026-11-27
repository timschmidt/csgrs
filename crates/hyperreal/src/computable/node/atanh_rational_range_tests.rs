/// Qualify imported rational atanh tags inside the open real domain (-1,1).
/// Directed MPFR bounds cover tiny, series-boundary and endpoint-adjacent inputs,
/// precision reuse and entry cancellation/retry. Endpoints themselves are excluded;
/// these finite checks do not establish arbitrary-input resource bounds.
#[cfg(test)]
mod atanh_rational_range_tests {
    use super::*;
    use rug::{Float, Integer, Rational as Q, float::Round};

    /// Build an AtanhRational node without constructor normalization, initial facts or a signal.
    /// Parse the Rug rational exactly; the caller must supply -1<q<1. The stored tag
    /// alone proves neither the domain nor eligibility for the small-argument series.
    fn imported(q: &Q) -> Computable {
        Computable {
            internal: Arc::new(Node::new(
                Approximation::AtanhRational(q.to_string().parse().unwrap()),
                BoundCache::Invalid,
                ExactSignCache::Invalid,
            )),
            signal: None,
        }
    }

    /// Enclose atanh(q) by monotone outward MPFR evaluation at 2048 bits.
    /// Require -1<q<1 and rounded input endpoints still strictly inside that domain,
    /// as the fixtures ensure. Return exact rational endpoints; infinities/NaNs panic
    /// on conversion, so this helper is not an oracle for invalid or arbitrary near-endpoint data.
    fn oracle(q: &Q) -> [Q; 2] {
        let mut lo = Float::with_val_round(2048, q, Round::Down).0;
        let mut hi = Float::with_val_round(2048, q, Round::Up).0;
        lo.atanh_round(Round::Down);
        hi.atanh_round(Round::Up);
        [lo.to_rational().unwrap(), hi.to_rational().unwrap()]
    }

    /// Require the integer approximation's closed one-unit interval to contain the entire oracle interval.
    /// Interpret the returned integer exactly as a*2^p, with error radius 2^p, using Rug rationals.
    /// This checks containment, not correct rounding or equality of cold and warm samples.
    /// Fixture precisions must permit the signed shifts (in particular p must not be i32::MIN);
    /// conversion or a failed enclosure panics. The supplied bounds must already be valid.
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

    /// Check 25 signed inputs strictly inside (-1,1) against directed MPFR bounds.
    /// Include zero, the one-eighth series boundary, 2^-512 and distances 2^-64/2^-512
    /// from either endpoint. Twelve requests check fresh and reused imported nodes;
    /// the test name's endpoint inputs are adjacent values, not the undefined endpoints.
    #[test]
    fn imported_rational_atanh_encloses_tiny_boundary_and_endpoint_inputs() {
        crate::mpfr_test::cleanup_on_thread_exit();
        let mut inputs = vec![Q::from(0)];
        for q in [
            Q::from((1, 16)),
            Q::from((15, 128)),
            Q::from((1, 8)),
            Q::from((17, 128)),
            Q::from((1, 3)),
            Q::from((1, 2)),
            Q::from((3, 4)),
            Q::from((31, 32)),
            Q::from((63, 64)),
            Q::from((1, Integer::from(1) << 512)),
        ] {
            inputs.extend([q.clone(), -q]);
        }
        for bits in [64, 512] {
            let q = Q::from(1) - Q::from((1, Integer::from(1) << bits));
            inputs.extend([q.clone(), -q]);
        }
        assert_eq!(inputs.len(), 25);
        for q in inputs {
            let bounds = oracle(&q);
            let warm = imported(&q);
            for p in [4, 2, 1, 0, -1, -32, -128, -512, -8, 2, 1, 0] {
                encloses(&imported(&q), p, &bounds);
                encloses(&warm, p, &bounds);
            }
        }
    }

    /// Check entry cancellation and retry for both signs of 31/32 and 1-2^-512.
    /// An aborted request must leave the queried node uncached. Clearing the shared flag
    /// permits a -128 retry agreeing with the subsequent warm call and enclosing MPFR;
    /// the returned abort placeholder and mid-computation cancellation are not asserted.
    #[test]
    fn rational_atanh_range_reduction_recovers_after_cancellation() {
        use std::sync::atomic::{AtomicBool, Ordering};
        crate::mpfr_test::cleanup_on_thread_exit();
        for magnitude in [
            Q::from((31, 32)),
            Q::from(1) - Q::from((1, Integer::from(1) << 512)),
        ] {
            for q in [magnitude.clone(), -magnitude] {
                let value = imported(&q);
                let signal = Arc::new(AtomicBool::new(true));
                let _ = value.approx_signal(&Some(signal.clone()), -128);
                assert!(value.cached().is_none());
                signal.store(false, Ordering::Relaxed);
                let resumed = value.approx_signal(&Some(signal), -128);
                assert_eq!(resumed, value.approx(-128));
                encloses(&value, -128, &oracle(&q));
            }
        }
    }
}
