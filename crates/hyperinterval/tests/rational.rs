#![cfg(feature = "rational")]

use hyperinterval::{IntervalError, RationalInterval, SignSet};
use hyperreal::Rational;

#[path = "support/legacy_rational_product.rs"]
mod legacy;

fn r(n: i64, d: u64) -> Rational {
    Rational::fraction(n, d).unwrap()
}
fn interval(a: i64, b: i64) -> RationalInterval {
    RationalInterval::new(r(a, 3), r(b, 3)).unwrap()
}

#[test]
fn owned_bounds_reconstruct_exact_values_and_preserve_empty() {
    assert_eq!(RationalInterval::empty().into_bounds(), None);
    for (a, b) in [(-7, -2), (-3, 4), (0, 0), (1, 8)] {
        let source = interval(a, b);
        let (lower, upper) = source.clone().into_bounds().unwrap();
        assert_eq!((lower.clone(), upper.clone()), (r(a, 3), r(b, 3)));
        let restored = RationalInterval::new(lower, upper).unwrap();
        assert_eq!(restored, source);
        assert_eq!(restored.add(&interval(1, 2)), interval(a + 1, b + 2));
        assert!(!restored.interior_subset_of(&RationalInterval::empty()));
    }
}

#[test]
fn positive_empty_products_and_quotients_do_not_hide_invalid_operands() {
    let empty = RationalInterval::empty();
    let positive = interval(1, 7);
    for (a, b) in [(&empty, &positive), (&positive, &empty), (&empty, &empty)] {
        let product = a.positive_product(b).unwrap();
        let quotient = a.positive_quotient(b).unwrap();
        assert_eq!(product, empty);
        assert_eq!(quotient, empty);
        assert_eq!(product.positive_quotient(&positive).unwrap(), empty);
        assert_eq!(quotient.hull(&positive), positive);
    }
    for invalid in [
        interval(-3, -1),
        interval(-1, 1),
        interval(0, 0),
        interval(0, 1),
    ] {
        for (a, b) in [(&empty, &invalid), (&invalid, &empty)] {
            assert_eq!(
                a.positive_product(b),
                Err(IntervalError::NonPositiveLowerBound)
            );
            assert_eq!(
                a.positive_quotient(b),
                Err(IntervalError::NonPositiveLowerBound)
            );
        }
    }
}

#[test]
fn rational_product_matches_independent_four_corner_oracle_and_legacy() {
    let numerators = [-13, -7, -1, 0, 1, 7, 13];
    let endpoints = numerators.map(|n| r(n, 6));
    for (i, a) in endpoints.iter().enumerate() {
        for (bi, b) in endpoints[i..].iter().enumerate() {
            for (j, c) in endpoints.iter().enumerate() {
                for (di, d) in endpoints[j..].iter().enumerate() {
                    let left = RationalInterval::new(a.clone(), b.clone()).unwrap();
                    let right = RationalInterval::new(c.clone(), d.clone()).unwrap();
                    let actual = left.mul(&right);
                    let (legacy_lower, legacy_upper) =
                        legacy::rational_interval_product(a, b, c, d);
                    assert_eq!(actual.bounds(), Some((&legacy_lower, &legacy_upper)));
                    // Integer numerators over the common denominator give an
                    // independent exact oracle, not another Rational mul wrapper.
                    let nums = [
                        numerators[i],
                        numerators[i + bi],
                        numerators[j],
                        numerators[j + di],
                    ];
                    let [aa, bb, cc, dd] = nums;
                    let products = [aa * cc, aa * dd, bb * cc, bb * dd];
                    assert_eq!(
                        actual.lower(),
                        Some(&r(*products.iter().min().unwrap(), 36))
                    );
                    assert_eq!(
                        actual.upper(),
                        Some(&r(*products.iter().max().unwrap(), 36))
                    );
                }
            }
        }
    }
}

#[cfg(feature = "dispatch-trace")]
#[test]
fn product_dispatch_and_reducer_counts_match_baseline() {
    use hyperreal::dispatch_trace;
    const KEY: &str = "HYPERINTERVAL_TRACE_BACKEND";
    if let Ok(backend) = std::env::var(KEY) {
        assert!(backend == "legacy" || backend == "shared");
        let large: Rational = "123456789012345678901234567890123456789/9876543210987654321"
            .parse()
            .unwrap();
        let values = [-&large, r(-1, 7), r(0, 1), r(1, 7), large];
        for (i, a) in values.iter().enumerate() {
            for b in &values[i..] {
                for (j, c) in values.iter().enumerate() {
                    for d in &values[j..] {
                        let left = RationalInterval::new(a.clone(), b.clone()).unwrap();
                        let right = RationalInterval::new(c.clone(), d.clone()).unwrap();
                        for _ in 0..2 {
                            dispatch_trace::reset();
                            dispatch_trace::reset_rational_stats();
                            let result = dispatch_trace::with_recording(|| {
                                if backend == "legacy" {
                                    legacy::rational_interval_product(a, b, c, d)
                                } else {
                                    let product = left.mul(&right);
                                    let (lo, hi) = product.bounds().unwrap();
                                    (lo.clone(), hi.clone())
                                }
                            });
                            println!("TRACE {:?} {:?}", dispatch_trace::take_trace(), result);
                        }
                    }
                }
            }
        }
        return;
    }
    // Separate processes isolate global small-rational interning as well as
    // per-value caches. Both traverse the same cold/warm sequence, without
    // another test or the baseline priming facts consumed by the candidate.
    let run = |backend| {
        let output = std::process::Command::new(std::env::current_exe().unwrap())
            .args([
                "--exact",
                "product_dispatch_and_reducer_counts_match_baseline",
                "--nocapture",
            ])
            .env(KEY, backend)
            .output()
            .unwrap();
        assert!(
            output.status.success(),
            "{}",
            String::from_utf8_lossy(&output.stderr)
        );
        String::from_utf8(output.stdout)
            .unwrap()
            .lines()
            .filter(|line| line.starts_with("TRACE "))
            .map(str::to_owned)
            .collect::<Vec<_>>()
    };
    let old = run("legacy");
    let new = run("shared");
    assert_eq!(old.len(), 450);
    assert_eq!(new, old);
}

#[test]
fn exact_images_set_laws_and_empty_domain_precedence() {
    let a = interval(-6, 3);
    let b = interval(3, 9);
    assert_eq!(a.add(&b), interval(-3, 12));
    assert_eq!(a.sub(&b), interval(-15, 0));
    assert_eq!(a.neg(), interval(-3, 6));
    assert_eq!(a.square(), interval(0, 12));
    assert_eq!(interval(-6, -3).square(), interval(3, 12));
    assert_eq!(b.square(), interval(3, 27));
    assert_eq!(a.hull(&b), interval(-6, 9));
    assert_eq!(a.intersection(&b), interval(3, 3));
    assert!(a.overlaps(&b));
    assert!(!a.strictly_left_of(&b));
    assert!(interval(-6, 2).strictly_left_of(&b));
    assert!(a.subset_of(&a.hull(&b)));
    assert!(interval(-5, 2).interior_subset_of(&a));
    assert!(!a.interior_subset_of(&a));
    assert_eq!(a.signs(), SignSet::Mixed);
    assert_eq!(a.midpoint(), Some(r(-1, 2)));
    assert_eq!(a.width(), Some(r(3, 1)));
    assert_eq!(a.radius(), Some(r(3, 2)));
    assert_eq!(a.affine(&r(-2, 1), &r(1, 1)), interval(-3, 15));
    assert_eq!(
        b.reciprocal().unwrap(),
        RationalInterval::new(r(1, 3), r(1, 1)).unwrap()
    );
    assert_eq!(
        interval(-9, -3).reciprocal().unwrap(),
        RationalInterval::new(r(-1, 1), r(-1, 3)).unwrap()
    );
    assert_eq!(a.div(&b).unwrap(), a);
    assert_eq!(b.positive_product(&b).unwrap(), b.mul(&b));
    assert_eq!(b.positive_quotient(&b).unwrap(), b.div(&b).unwrap());
    assert_eq!(
        a.positive_product(&b),
        Err(IntervalError::NonPositiveLowerBound)
    );
    let empty = RationalInterval::empty();
    assert!(empty.is_empty());
    assert!(!empty.is_singleton());
    assert!(empty.subset_of(&a));
    assert!(empty.interior_subset_of(&empty));
    assert!(!a.subset_of(&empty));
    assert!(empty.disjoint(&empty));
    assert!(!empty.strictly_left_of(&a));
    assert_eq!(empty.signs(), SignSet::Empty);
    assert_eq!(empty.width(), None);
    assert_eq!(empty.midpoint(), None);
    assert_eq!(empty.hull(&a), a);
    for result in [
        empty.add(&a),
        empty.sub(&a),
        empty.mul(&a),
        empty.neg(),
        empty.square(),
        empty.intersection(&a),
        empty.div(&b).unwrap(),
        a.div(&empty).unwrap(),
    ] {
        assert!(result.is_empty());
    }
    assert_eq!(empty.div(&a), Err(IntervalError::ContainsZero));
    assert_eq!(
        empty.positive_quotient(&a),
        Err(IntervalError::NonPositiveLowerBound)
    );
    assert_eq!(
        RationalInterval::from_nominal_uncertainty(&r(1, 3), &r(-1, 3)),
        Err(IntervalError::NegativeRadius)
    );
    assert_eq!(
        RationalInterval::from_nominal_uncertainty(&r(1, 3), &r(2, 3)).unwrap(),
        interval(-1, 3)
    );
    assert_eq!(
        RationalInterval::new(r(1, 1), r(0, 1)),
        Err(IntervalError::ReversedEndpoints)
    );
}

#[cfg(feature = "serde")]
#[test]
fn rational_replay_validates_order_and_preserves_sets() {
    for value in [
        RationalInterval::empty(),
        interval(-7, 11),
        RationalInterval::point(r(5, 17)),
    ] {
        let bytes = serde_json::to_vec(&value).unwrap();
        assert_eq!(
            serde_json::from_slice::<RationalInterval>(&bytes).unwrap(),
            value
        );
        assert_eq!(
            serde_json::to_vec(&serde_json::from_slice::<RationalInterval>(&bytes).unwrap())
                .unwrap(),
            bytes
        );
    }
    let reversed = serde_json::to_vec(&Some((r(1, 1), r(-1, 1)))).unwrap();
    assert!(serde_json::from_slice::<RationalInterval>(&reversed).is_err());
}
