use hyperreal::{Problem, Rational, Real};

fn opaque_zero() -> Real {
    Real::e().sin().powi(2.into()).unwrap() + Real::e().cos().powi(2.into()).unwrap() - Real::one()
}

#[test]
fn positive_x_does_not_require_a_y_sign() {
    let y = opaque_zero();
    assert_eq!(
        y.certified_sign_until(Real::PARTIAL_CMP_MIN_PRECISION)
            .sign(),
        None
    );
    for x in [Real::one(), Real::pi(), Real::one() + &y] {
        let expected = (y.clone() / &x).unwrap().atan().unwrap();
        let actual = y.clone().try_atan2(x.clone()).unwrap();
        assert_eq!(
            (actual.clone() - &expected).exact_rational_normal_form(),
            Some(Rational::zero())
        );
        #[cfg(feature = "serde")]
        assert_eq!(actual.to_json(), expected.to_json());
        assert_eq!(y.clone().atan2(x), actual);
    }
}

#[test]
fn unresolved_y_on_the_left_or_origin_keeps_typed_exhaustion() {
    for y in [
        opaque_zero(),
        opaque_zero() + "1e-1000".parse::<Real>().unwrap(),
        opaque_zero() - "1e-1000".parse::<Real>().unwrap(),
    ] {
        assert_eq!(
            y.certified_sign_until(Real::PARTIAL_CMP_MIN_PRECISION)
                .sign(),
            None
        );
        assert_eq!(y.clone().try_atan2(-Real::one()), Err(Problem::Exhausted));
        assert_eq!(y.clone().try_atan2(Real::zero()), Err(Problem::Exhausted));
        let actual = y.clone().try_atan2(Real::one()).unwrap();
        let expected = y.atan().unwrap();
        assert_eq!(
            (actual - expected).exact_rational_normal_form(),
            Some(Rational::zero())
        );
    }
}

#[test]
fn previous_axes_and_quadrants_retain_their_exact_forms() {
    for y in [-3_i32, 0, 3] {
        for x in [-2, 0, 2] {
            let expected = if x == 0 {
                (Real::from(y.signum()) * Real::pi() / Real::from(2)).unwrap()
            } else if y == 0 && x < 0 {
                Real::pi()
            } else {
                let base = (Real::from(y) / Real::from(x)).unwrap().atan().unwrap();
                if x > 0 {
                    base
                } else {
                    base + Real::from(y.signum()) * Real::pi()
                }
            };
            let actual = Real::from(y).try_atan2(Real::from(x)).unwrap();
            assert_eq!(actual, expected);
            #[cfg(feature = "serde")]
            assert_eq!(actual.to_json(), expected.to_json());
        }
    }
}

#[test]
fn axes_with_refinement_proven_signs_keep_exact_pi_multiples() {
    let half = (Real::one() / Real::from(2)).unwrap();
    for sign in [-1, 1] {
        let y = opaque_zero() + Real::from(sign) * &half;
        assert_eq!(y.structural_facts().sign, None);
        let expected = (Real::from(sign) * Real::pi() / Real::from(2)).unwrap();
        assert_eq!(y.try_atan2(Real::zero()).unwrap(), expected);
    }
    let x = opaque_zero() - half;
    assert_eq!(x.structural_facts().sign, None);
    assert_eq!(Real::zero().try_atan2(x).unwrap(), Real::pi());
}

#[test]
fn retained_opaque_argument_keeps_abort_and_precision_semantics() {
    use std::sync::{
        Arc,
        atomic::{AtomicBool, Ordering},
    };
    let signal = Arc::new(AtomicBool::new(false));
    let mut y = opaque_zero();
    y.abort(signal.clone());
    let value = y.try_atan2(Real::one()).unwrap();
    let mut previous: Option<[Rational; 2]> = None;
    for bits in [32, 512, 64, 256] {
        let current = value.certified_rational_interval(-bits).unwrap();
        if let Some(old) = previous {
            assert!(old[0] <= current[1] && current[0] <= old[1]);
        }
        previous = Some(current);
    }
    signal.store(true, Ordering::Relaxed);
    assert!(value.certified_rational_interval(-1024).is_none());
    signal.store(false, Ordering::Relaxed);
    std::thread::scope(|scope| {
        for bits in [32, 128, 256, 512] {
            let value = &value;
            scope.spawn(move || assert!(value.certified_rational_interval(-bits).is_some()));
        }
    });
}

#[test]
fn known_nonzero_y_reuses_computable_half_angle_when_x_is_unresolved() {
    for sign in [-1, 1] {
        let value = Real::from(sign).try_atan2(opaque_zero()).unwrap();
        let expected = (Real::from(sign) * Real::pi() / Real::from(2)).unwrap();
        let [lo, hi] = (value - expected)
            .certified_rational_interval(-160)
            .unwrap();
        let bound = Rational::fraction(1, 2).unwrap().powi(128.into()).unwrap();
        assert!(lo >= -bound.clone() && hi <= bound);
    }
}

#[test]
fn original_small_offsets_keep_certified_left_quadrants() {
    for sign in [-1, 1] {
        let y = opaque_zero() + Real::from(sign) * "1e-320".parse::<Real>().unwrap();
        assert_eq!(
            y.certified_sign_until(Real::PARTIAL_CMP_MIN_PRECISION)
                .sign(),
            Some(if sign > 0 {
                hyperreal::RealSign::Positive
            } else {
                hyperreal::RealSign::Negative
            })
        );
        let expected = (-y.clone()).atan().unwrap() + Real::from(sign) * Real::pi();
        let actual = y.try_atan2(-Real::one()).unwrap();
        assert_eq!(
            (actual - expected).exact_rational_normal_form(),
            Some(Rational::zero())
        );
    }
}

#[test]
fn deferred_atan_keeps_input_cancellation_cold_and_after_warming() {
    use std::sync::{
        Arc,
        atomic::{AtomicBool, Ordering},
    };
    for warm in [false, true] {
        let signal = Arc::new(AtomicBool::new(!warm));
        let mut input = Real::e().sin();
        input.abort(signal.clone());
        let values = [
            input.clone().atan().unwrap(),
            input.try_atan2(Real::one()).unwrap(),
        ];
        if warm {
            for value in &values {
                assert!(value.certified_rational_interval(-128).is_some());
            }
        }
        signal.store(true, Ordering::Relaxed);
        for value in &values {
            assert!(value.certified_rational_interval(-512).is_none());
        }
        signal.store(false, Ordering::Relaxed);
        let expected = Real::e()
            .sin()
            .atan()
            .unwrap()
            .certified_rational_interval(-512)
            .unwrap();
        for value in values {
            let actual = value.certified_rational_interval(-512).unwrap();
            assert!(actual[0] <= expected[1] && expected[0] <= actual[1]);
        }
    }
}
