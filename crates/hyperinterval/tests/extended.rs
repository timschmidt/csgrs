#[cfg(feature = "dyadic")]
mod dyadic {
    use hyperinterval::{
        DyadicBudget, DyadicInterval as D, ExtendedInterval as X, ExtendedIntervalView as View,
        IntervalError, SignSet,
    };
    type E = X<D>;
    fn budget() -> DyadicBudget {
        DyadicBudget::new(4096, 10_000_000_000)
    }
    fn p(n: i64) -> D {
        D::point(n.into(), 0).unwrap()
    }
    #[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd)]
    enum End {
        Minus,
        Value(i64),
        Plus,
    }
    fn build(pair: Option<(End, End)>) -> E {
        match pair {
            None => E::empty(),
            Some((End::Minus, End::Plus)) => E::entire(),
            Some((End::Minus, End::Value(b))) => E::below(p(b)).unwrap(),
            Some((End::Value(a), End::Plus)) => E::above(p(a)).unwrap(),
            Some((End::Value(a), End::Value(b))) => {
                E::from_finite(D::new(a.into(), b.into(), 0).unwrap())
            }
            _ => unreachable!(),
        }
    }
    #[test]
    fn extended_set_laws_match_independent_endpoint_order() {
        use End::{Minus as M, Plus as P, Value as V};
        let cases = [
            None,
            Some((M, P)),
            Some((M, V(-2))),
            Some((M, V(0))),
            Some((M, V(2))),
            Some((V(-2), P)),
            Some((V(0), P)),
            Some((V(2), P)),
            Some((V(-2), V(-1))),
            Some((V(-1), V(1))),
            Some((V(0), V(0))),
            Some((V(1), V(2))),
        ];
        for a in cases {
            let x = build(a);
            assert_eq!(x.neg(&mut budget()).unwrap().neg(&mut budget()).unwrap(), x);
            for point in -3..=3 {
                let expected = a.is_some_and(|(lo, hi)| lo <= V(point) && V(point) <= hi);
                assert_eq!(
                    x.contains_point(&point.into(), 0, &mut budget()).unwrap(),
                    expected
                );
            }
            for b in cases {
                let y = build(b);
                let hull = match (a, b) {
                    (None, x) | (x, None) => x,
                    (Some((a, b)), Some((c, d))) => Some((a.min(c), b.max(d))),
                };
                let intersection = match (a, b) {
                    (Some((a, b)), Some((c, d))) => {
                        let (lo, hi) = (a.max(c), b.min(d));
                        if lo <= hi { Some((lo, hi)) } else { None }
                    }
                    _ => None,
                };
                let subset = match (a, b) {
                    (None, _) => true,
                    (_, None) => false,
                    (Some((a, b)), Some((c, d))) => c <= a && b <= d,
                };
                assert_eq!(x.hull(&y, &mut budget()).unwrap(), build(hull));
                assert_eq!(
                    x.intersection(&y, &mut budget()).unwrap(),
                    build(intersection)
                );
                assert_eq!(x.subset_of(&y, &mut budget()).unwrap(), subset);
            }
        }
    }
    #[test]
    fn ray_ingress_and_finite_conversion_fail_explicitly() {
        assert_eq!(
            E::below(D::empty()),
            Err(IntervalError::NonSingletonEndpoint)
        );
        assert_eq!(
            E::above(D::new(0.into(), 1.into(), 0).unwrap()),
            Err(IntervalError::NonSingletonEndpoint)
        );
        assert_eq!(E::from_finite(D::empty()), E::empty());
        assert!(E::empty().finite_range(&mut budget()).unwrap().is_empty());
        assert_eq!(
            E::from_finite(p(2)).finite_range(&mut budget()).unwrap(),
            p(2)
        );
        for x in [
            E::entire(),
            E::below(p(0)).unwrap(),
            E::above(p(0)).unwrap(),
        ] {
            assert_eq!(
                x.finite_range(&mut budget()),
                Err(IntervalError::UnboundedImage)
            );
        }
        assert_eq!(E::empty().signs(), SignSet::Empty);
        assert_eq!(E::entire().signs(), SignSet::Mixed);
        for (n, below, above) in [
            (-1, SignSet::Negative, SignSet::Mixed),
            (0, SignSet::NegativeOrZero, SignSet::ZeroOrPositive),
            (1, SignSet::Mixed, SignSet::Positive),
        ] {
            assert_eq!(E::below(p(n)).unwrap().signs(), below);
            assert_eq!(E::above(p(n)).unwrap().signs(), above);
        }
    }
    #[test]
    fn extremes_do_not_expand_irrelevant_scale_gaps() {
        let top = E::below(D::point(1.into(), i32::MAX).unwrap()).unwrap();
        assert!(
            top.contains_point(&1.into(), i32::MIN, &mut DyadicBudget::new(1, 10))
                .unwrap()
        );
        assert!(
            top.contains_point(&0.into(), i32::MIN, &mut DyadicBudget::new(1, 10))
                .unwrap()
        );
        let below = E::below(D::point((-1).into(), i32::MIN).unwrap()).unwrap();
        assert!(
            below
                .contains_point(&(-1).into(), i32::MAX, &mut DyadicBudget::new(1, 10))
                .unwrap()
        );
        assert!(
            !below
                .contains_point(&0.into(), 0, &mut DyadicBudget::new(1, 10))
                .unwrap()
        );
        for (point, e, expected) in [(2, -1, true), (3, -1, false), (1, 0, true)] {
            assert_eq!(
                E::below(p(1))
                    .unwrap()
                    .contains_point(&point.into(), e, &mut budget())
                    .unwrap(),
                expected
            );
        }
        let a = E::from_finite(D::new(0.into(), 1.into(), i32::MIN).unwrap());
        assert_eq!(
            a.intersection(&E::above(p(0)).unwrap(), &mut budget())
                .unwrap(),
            a
        );
        assert!(matches!(a.view(), View::Finite(_)));
        assert_eq!(
            top.contains_point(&1.into(), 0, &mut DyadicBudget::new(1, 0)),
            Err(IntervalError::WorkExhausted)
        );
        assert!(top.contains_point(&1.into(), 0, &mut budget()).unwrap());
    }
    #[test]
    fn exponential_consumes_every_extended_shape() {
        assert!(E::empty().exp(0, &mut budget()).unwrap().range.is_empty());
        assert_eq!(
            E::entire().exp(0, &mut budget()).unwrap().range,
            E::above(p(0)).unwrap()
        );
        assert_eq!(
            E::below(p(0))
                .unwrap()
                .exp(i32::MIN, &mut budget())
                .unwrap()
                .range,
            E::from_finite(D::new(0.into(), 1.into(), 0).unwrap())
        );
        assert_eq!(
            E::above(p(0))
                .unwrap()
                .exp(-32, &mut budget())
                .unwrap()
                .range,
            E::above(p(1)).unwrap()
        );
        assert_eq!(
            E::from_finite(p(0)).exp(-32, &mut budget()).unwrap().range,
            E::from_finite(p(1))
        );
    }
    #[cfg(feature = "serde")]
    #[test]
    fn replay_validates_and_normalizes_extended_states() {
        for x in [
            E::empty(),
            E::entire(),
            E::from_finite(p(2)),
            E::below(p(-1)).unwrap(),
            E::above(p(0)).unwrap(),
        ] {
            let bytes = serde_json::to_vec(&x).unwrap();
            let restored: E = serde_json::from_slice(&bytes).unwrap();
            assert_eq!(restored, x);
            assert_eq!(serde_json::to_vec(&restored).unwrap(), bytes);
            assert_eq!(
                restored.neg(&mut budget()).unwrap(),
                x.neg(&mut budget()).unwrap()
            );
        }
        assert_eq!(
            serde_json::from_str::<E>("{\"Finite\":null}").unwrap(),
            E::empty()
        );
        for kind in ["Below", "Above"] {
            for bound in [D::empty(), D::new(0.into(), 1.into(), 0).unwrap()] {
                let bytes = format!("{{\"{kind}\":{}}}", serde_json::to_string(&bound).unwrap());
                assert!(serde_json::from_str::<E>(&bytes).is_err());
            }
        }
    }
}
#[cfg(feature = "fixed")]
#[test]
fn fixed_backend_is_sealed_and_retains_point_validation() {
    use hyperinterval::{ExtendedInterval, FixedInterval, IntervalError};
    type E = ExtendedInterval<FixedInterval<i32, 30>>;
    fn is_copy<T: Copy>() {}
    is_copy::<E>();
    assert!(E::above(FixedInterval::point(0).unwrap()).is_ok());
    assert_eq!(
        E::below(FixedInterval::new(0, 1).unwrap()),
        Err(IntervalError::NonSingletonEndpoint)
    );
}
#[cfg(feature = "rational")]
#[test]
fn rational_backend_retains_exact_point_validation() {
    use hyperinterval::{ExtendedInterval, IntervalError, RationalInterval};
    type E = ExtendedInterval<RationalInterval>;
    assert!(E::below(RationalInterval::point(0.into())).is_ok());
    assert_eq!(
        E::above(RationalInterval::new(0.into(), 1.into()).unwrap()),
        Err(IntervalError::NonSingletonEndpoint)
    );
}
#[cfg(all(feature = "float-enclosure", target_arch = "x86_64"))]
#[test]
fn ieee_backends_retain_qualified_ingress() {
    use hyperinterval::{ExtendedInterval, F32Enclosure, F64Enclosure, IntervalError};
    assert!(ExtendedInterval::below(F32Enclosure::point(0.0).unwrap()).is_ok());
    assert!(ExtendedInterval::above(F64Enclosure::point(0.0).unwrap()).is_ok());
    assert_eq!(
        ExtendedInterval::above(F64Enclosure::new(0.0, 1.0).unwrap()),
        Err(IntervalError::NonSingletonEndpoint)
    );
}
