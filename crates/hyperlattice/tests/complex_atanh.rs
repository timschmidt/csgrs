use hyperlattice::{Complex, Problem, Rational, Real, RealSign};

#[test]
fn axes_and_principal_cuts_have_explicit_semantics() {
    for x in [
        Real::zero(),
        "0.5".parse().unwrap(),
        "-0.5".parse().unwrap(),
    ] {
        assert_eq!(
            Complex::from(x.clone()).atanh().unwrap(),
            Complex::from(x.atanh().unwrap())
        );
    }
    for y in [
        Real::zero(),
        Real::pi(),
        -Real::e(),
        "1e300".parse().unwrap(),
    ] {
        assert_eq!(
            Complex::new(Real::zero(), y.clone()).atanh().unwrap(),
            Complex::new(Real::zero(), y.atan().unwrap())
        );
    }
    for x in [Real::one(), -Real::one()] {
        assert_eq!(Complex::from(x).atanh(), Err(Problem::Infinity));
    }
    for x in [Real::from(2), Real::from(-2)] {
        assert_eq!(Complex::from(x).atanh(), Err(Problem::NotANumber));
    }
}

#[path = "../../hyperreal/tests/support/mpfr.rs"]
mod mpfr;
#[path = "support/complex_atanh_oracle.rs"]
mod oracle;
#[path = "../../hyperreal/tests/support/representations.rs"]
mod representations;

#[test]
fn independent_series_and_reciprocal_enclosures() {
    mpfr::cleanup_on_thread_exit();
    for x in ["-0.5", "0", "0.5"] {
        for y in ["-0.125", "0", "0.125"] {
            oracle::compare(&Complex::new(x.parse().unwrap(), y.parse().unwrap()));
        }
    }
    for x in ["-4", "0", "4", "1e300", "-1e300"] {
        for y in ["-2", "2"] {
            oracle::compare(&Complex::new(x.parse().unwrap(), y.parse().unwrap()));
        }
    }
}

#[test]
fn every_scalar_family_reaches_the_independent_reference() {
    mpfr::cleanup_on_thread_exit();
    for case in representations::optimized_certificate_representatives() {
        assert_eq!(
            case.value.detailed_facts().symbolic.kind,
            case.public_kind,
            "{}",
            case.certificate
        );
        let small = (&case.value / (Real::from(4) * (&case.value + Real::one()))).unwrap();
        for sign in [-1, 1] {
            let z = Complex::new(Real::from(sign) * &small, (&small / Real::from(2)).unwrap());
            oracle::compare(&z);
            oracle::compare(&z.reciprocal().unwrap());
        }
    }
}

fn close(a: &Real, b: &Real, bits: i32) {
    let [lo, hi] = (a - b).certified_rational_interval(-bits - 16).unwrap();
    let tolerance = Rational::fraction(1, 2).unwrap().powi(bits.into()).unwrap();
    assert!(
        lo >= -tolerance.clone() && hi <= tolerance,
        "difference [{lo}, {hi}]"
    );
}

fn complex_close(a: &Complex, b: &Complex, bits: i32) {
    close(&a.re, &b.re, bits);
    close(&a.im, &b.im, bits);
}

fn forward_tanh(z: Complex) -> Complex {
    let twice_re = Real::from(2) * z.re;
    let twice_im = Real::from(2) * z.im;
    let denominator = twice_re.clone().cosh().unwrap() + twice_im.clone().cos();
    Complex::new(
        (twice_re.sinh().unwrap() / &denominator).unwrap(),
        (twice_im.sin() / denominator).unwrap(),
    )
}

#[test]
fn conjugation_parity_and_forward_composition_preserve_the_branch() {
    for (x, y) in [
        ("0.125", "0.25"),
        ("4", "2"),
        ("-4", "2"),
        ("0.75", "-0.75"),
        ("1", "1"),
    ] {
        let z = Complex::new(x.parse().unwrap(), y.parse().unwrap());
        let value = z.clone().atanh().unwrap();
        complex_close(
            &z.clone().conjugate().atanh().unwrap(),
            &value.clone().conjugate(),
            192,
        );
        complex_close(&(-z.clone()).atanh().unwrap(), &(-value.clone()), 192);
        complex_close(&forward_tanh(value.clone()), &z, 192);
        let margin = (Real::pi() / Real::from(2)).unwrap() - value.im.abs();
        assert_eq!(
            margin.certified_sign_until(-256).sign(),
            Some(RealSign::Positive)
        );
    }
}

#[test]
fn both_banks_and_tiny_pole_offsets_remain_distinct() {
    for text in ["1e-40", "1e-320"] {
        let delta: Real = text.parse().unwrap();
        let delta2 = &delta * &delta;
        for sx in [-1, 1] {
            for sy in [-1, 1] {
                let z = Complex::new(Real::from(sx), Real::from(sy) * &delta);
                let result = z.atanh().unwrap();
                let main = (Real::from(sx)
                    * (Real::from(2).ln().unwrap() - delta.clone().ln().unwrap())
                    / Real::from(2))
                .unwrap();
                close(
                    &((&result.re - main) / &delta2).unwrap(),
                    &(Real::from(sx) / Real::from(16)).unwrap(),
                    64,
                );
                let main = (Real::from(sy) * Real::pi() / Real::from(4)).unwrap();
                close(
                    &((&result.im - main) / &delta).unwrap(),
                    &(Real::from(sy) / Real::from(4)).unwrap(),
                    64,
                );
                let z = Complex::new(Real::from(2 * sx), Real::from(sy) * &delta);
                let result = z.atanh().unwrap();
                let main = (Real::from(sx) * Real::from(3).ln().unwrap() / Real::from(2)).unwrap();
                close(
                    &((&result.re - main) / &delta2).unwrap(),
                    &(Real::from(-2 * sx) / Real::from(9)).unwrap(),
                    64,
                );
                let main = (Real::from(sy) * Real::pi() / Real::from(2)).unwrap();
                close(
                    &((&result.im - main) / &delta).unwrap(),
                    &(Real::from(-sy) / Real::from(3)).unwrap(),
                    64,
                );
            }
        }
    }
}

#[test]
fn unknown_signs_are_not_silently_classified_as_cuts() {
    let zero = Real::e().sin().powi(2.into()).unwrap() + Real::e().cos().powi(2.into()).unwrap()
        - Real::one();
    assert_eq!(zero.immediate_sign(), None);
    assert_eq!(
        zero.certified_sign_until(Real::PARTIAL_CMP_MIN_PRECISION)
            .sign(),
        None
    );
    assert_eq!(
        Complex::new(Real::from(2), zero.clone()).atanh(),
        Err(Problem::Exhausted)
    );
    let x: Real = "0.25".parse().unwrap();
    complex_close(
        &Complex::new(x.clone(), zero.clone()).atanh().unwrap(),
        &Complex::from(x.atanh().unwrap()),
        192,
    );
    let y: Real = "0.125".parse().unwrap();
    complex_close(
        &Complex::new(zero, y.clone()).atanh().unwrap(),
        &Complex::new(Real::zero(), y.atan().unwrap()),
        192,
    );
}

#[test]
fn precision_cancellation_and_concurrent_readers_preserve_values() {
    use std::sync::{
        Arc,
        atomic::{AtomicBool, Ordering},
    };
    let value = Complex::new(
        (Real::pi() / Real::from(8)).unwrap(),
        (Real::e() / Real::from(8)).unwrap(),
    )
    .atanh()
    .unwrap();
    for component in [&value.re, &value.im] {
        let mut previous: Option<[Rational; 2]> = None;
        for bits in [32, 512, 64, 256] {
            let current = component.certified_rational_interval(-bits).unwrap();
            if let Some(old) = previous {
                assert!(old[0] <= current[1] && current[0] <= old[1]);
            }
            previous = Some(current);
        }
        let signal = Arc::new(AtomicBool::new(true));
        let mut aborted = component.clone();
        aborted.abort(signal.clone());
        assert!(aborted.certified_rational_interval(-1024).is_none());
        signal.store(false, Ordering::Relaxed);
        close(&aborted, component, 256);
        std::thread::scope(|scope| {
            for bits in [32, 128, 256, 512] {
                scope
                    .spawn(move || assert!(component.certified_rational_interval(-bits).is_some()));
            }
        });
    }
}

#[cfg(feature = "serde")]
#[test]
fn serialized_components_replay_the_selected_branch() {
    for (x, y) in [("0.125", "0.25"), ("-4", "2"), ("4", "-2"), ("1", "0.0001")] {
        let value = Complex::new(x.parse().unwrap(), y.parse().unwrap())
            .atanh()
            .unwrap();
        let values = [value.re, value.im];
        let serialized = serde_json::to_string(&values).unwrap();
        let decoded: [Real; 2] = serde_json::from_str(&serialized).unwrap();
        for (a, b) in values.iter().zip(&decoded) {
            close(a, b, 256);
        }
        assert_eq!(serialized, serde_json::to_string(&decoded).unwrap());
    }
}

#[test]
fn unit_circle_without_a_norm_zero_certificate_remains_admitted() {
    let angle = Real::e();
    let z = Complex::new(angle.clone().cos(), angle.sin());
    let value = z.clone().atanh().unwrap();
    close(&value.im, &(Real::pi() / Real::from(4)).unwrap(), 192);
    complex_close(&forward_tanh(value), &z, 192);
}

#[test]
fn tiny_values_and_huge_reciprocal_corrections_are_not_erased() {
    let tiny: Real = "1e-320".parse().unwrap();
    let huge: Real = "1e320".parse().unwrap();
    for sign in [-1, 1] {
        let value = Complex::new(tiny.clone(), Real::from(sign) * &tiny)
            .atanh()
            .unwrap();
        close(&(value.re / &tiny).unwrap(), &Real::one(), 128);
        close(&(value.im / &tiny).unwrap(), &Real::from(sign), 128);
        let value = Complex::new(huge.clone(), Real::from(sign) * &huge)
            .atanh()
            .unwrap();
        close(&(value.re * &huge), &"0.5".parse().unwrap(), 128);
        let limit = (Real::from(sign) * Real::pi() / Real::from(2)).unwrap();
        close(
            &((value.im - limit) * &huge),
            &(Real::from(-sign) / Real::from(2)).unwrap(),
            128,
        );
    }
}
