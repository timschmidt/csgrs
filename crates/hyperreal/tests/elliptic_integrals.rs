use hyperreal::{Problem, Rational, Real, RealSign};
use rug::{Float, float::Constant};
use std::sync::{
    Arc,
    atomic::{AtomicBool, Ordering},
};

#[path = "support/mpfr.rs"]
mod mpfr;
#[path = "support/representations.rs"]
mod representations;

fn mpfr_value(value: &Real) -> Float {
    let [lo, hi] = value.certified_rational_interval(-896).unwrap();
    let midpoint = (lo + hi) / Rational::new(2);
    let n: rug::Integer = midpoint.numerator().to_string().parse().unwrap();
    let d: rug::Integer = midpoint.denominator().to_string().parse().unwrap();
    let rational = rug::Rational::from((n, d));
    Float::with_val(
        1024,
        if midpoint.sign() == num::bigint::Sign::Minus {
            -rational
        } else {
            rational
        },
    )
}

fn from_mpfr(value: Float) -> Real {
    value.to_rational().unwrap().to_string().parse().unwrap()
}

// Independent expansion of the defining integrand after u=1/sqrt(t+z).
// For our corpus 0 <= A,B <= 3/4, the coefficients of
// ((1-A*u^2)(1-B*u^2))^(-1/2) are <= (3/4)^n.
// 2048 terms therefore leave < 2^-840, before MPFR rounding at 1024 bits.
fn integrand_series(x: &Real, y: &Real, z: &Real) -> [Real; 2] {
    let z = mpfr_value(z);
    let a = Float::with_val(1024, 1) - mpfr_value(x) / &z;
    let b = Float::with_val(1024, 1) - mpfr_value(y) / &z;
    let sum = Float::with_val(1024, &a + &b);
    let product = Float::with_val(1024, &a * &b);
    let mut previous = Float::with_val(1024, 0);
    let mut coefficient = Float::with_val(1024, 1);
    let mut rf = Float::with_val(1024, 0);
    let mut rd = Float::with_val(1024, 0);
    for n in 0..2048 {
        rf += Float::with_val(1024, &coefficient / (2 * n + 1));
        rd += Float::with_val(1024, &coefficient / (2 * n + 3));
        let next = (Float::with_val(1024, &sum * &coefficient) * (2 * n + 1) / 2
            - Float::with_val(1024, &product * &previous) * n)
            / (n + 1);
        previous = coefficient;
        coefficient = next;
    }
    [
        from_mpfr(rf / z.clone().sqrt()),
        from_mpfr(rd * 3 / z.clone().sqrt() / z),
    ]
}

fn close(actual: &Real, expected: &Real, bits: i32) {
    let [lo, hi] = (actual - expected)
        .certified_rational_interval(-bits - 8)
        .unwrap();
    let tolerance = Rational::fraction(1, 2).unwrap().powi(bits.into()).unwrap();
    assert!(
        lo >= -tolerance.clone() && hi <= tolerance,
        "error [{lo}, {hi}]"
    );
}

#[test]
fn diagonal_and_complete_circular_values() {
    let zero = Real::zero();
    for integer in [1, 4, 9, 25] {
        let x = Real::from(integer);
        let inverse_root = x.clone().sqrt().unwrap().inverse().unwrap();
        assert_eq!(Real::elliptic_rf(&x, &x, &x).unwrap(), inverse_root);
        assert_eq!(
            Real::elliptic_rd(&x, &x, &x).unwrap(),
            (&inverse_root / &x).unwrap()
        );
        close(
            &Real::elliptic_rf(&zero, &x, &x).unwrap(),
            &((&inverse_root * Real::pi()) / Real::from(2)).unwrap(),
            192,
        );
        close(
            &Real::elliptic_rd(&zero, &x, &x).unwrap(),
            &((Real::from(3) * Real::pi() * &inverse_root) / (Real::from(4) * x)).unwrap(),
            192,
        );
    }
}

#[test]
fn invalid_domains_are_rejected_before_identities() {
    for (x, y, z) in [
        (-1, 1, 1),
        (1, -1, 1),
        (1, 1, -1),
        (0, 0, 1),
        (0, 1, 0),
        (1, 0, 0),
        (0, 0, 0),
    ] {
        let [x, y, z] = [x, y, z].map(Real::from);
        assert_eq!(Real::elliptic_rf(&x, &y, &z), Err(Problem::NotANumber));
        assert_eq!(Real::elliptic_rd(&x, &y, &z), Err(Problem::NotANumber));
    }
    assert_eq!(
        Real::elliptic_rd(&Real::one(), &Real::one(), &Real::zero()),
        Err(Problem::NotANumber)
    );
}

#[test]
fn symmetric_and_homogeneous_composition() {
    let [x, y, z] = [2, 3, 5].map(Real::from);
    let rf = Real::elliptic_rf(&x, &y, &z).unwrap();
    let rd = Real::elliptic_rd(&x, &y, &z).unwrap();
    assert_eq!(rf.certified_sign_until(0).sign(), Some(RealSign::Positive));
    assert_eq!(rd.certified_sign_until(0).sign(), Some(RealSign::Positive));
    for (a, b, c) in [(&x, &z, &y), (&z, &y, &x), (&y, &x, &z)] {
        close(&Real::elliptic_rf(a, b, c).unwrap(), &rf, 192);
    }
    close(&Real::elliptic_rd(&y, &x, &z).unwrap(), &rd, 192);
    for scale in [
        Real::from(7),
        "1e-40".parse().unwrap(),
        "1e40".parse().unwrap(),
    ] {
        let root = scale.clone().sqrt().unwrap();
        close(
            &(Real::elliptic_rf(&(&x * &scale), &(&y * &scale), &(&z * &scale)).unwrap() * &root),
            &rf,
            160,
        );
        close(
            &(Real::elliptic_rd(&(&x * &scale), &(&y * &scale), &(&z * &scale)).unwrap()
                * &root
                * &scale),
            &rd,
            160,
        );
    }
    // Carlson's cyclic sum follows directly by integrating the derivative of
    // 1/sqrt((t+x)(t+y)(t+z)), independently of the duplication algorithm.
    let sum = &rd + Real::elliptic_rd(&y, &z, &x).unwrap() + Real::elliptic_rd(&z, &x, &y).unwrap();
    close(
        &sum,
        &(Real::from(3) / (&x * &y * &z).sqrt().unwrap()).unwrap(),
        192,
    );
}

#[test]
fn independent_mpfr_integrand_series_all_certificate_families() {
    mpfr::cleanup_on_thread_exit();
    for case in representations::optimized_certificate_representatives() {
        eprintln!("independent elliptic oracle: {}", case.certificate);
        assert_eq!(
            case.value.detailed_facts().symbolic.kind,
            case.public_kind,
            "{}",
            case.certificate
        );
        let square = &case.value * &case.value;
        let x = Real::one() + (&square / (Real::one() + &square)).unwrap();
        let y = Real::from(3);
        let z = Real::from(4);
        let [rf, rd] = integrand_series(&x, &y, &z);
        close(&Real::elliptic_rf(&x, &y, &z).unwrap(), &rf, 512);
        close(&Real::elliptic_rd(&x, &y, &z).unwrap(), &rd, 512);
    }
}

#[test]
fn complete_first_kind_agrees_with_mpfr_agm_near_singular_endpoint() {
    mpfr::cleanup_on_thread_exit();
    for value in ["1e-160", "1e-40", "0.001", "0.5", "2", "1e40"] {
        let y: Real = value.parse().unwrap();
        let agm = Float::with_val(1024, 1).agm(&mpfr_value(&y).sqrt());
        let expected = from_mpfr(Float::with_val(1024, Constant::Pi) / (agm * 2));
        close(
            &Real::elliptic_rf(&Real::zero(), &y, &Real::one()).unwrap(),
            &expected,
            256,
        );
    }
}

#[test]
fn extreme_uniform_scaling_stays_refinable() {
    let [x, y, z] = [2, 3, 5].map(Real::from);
    let rf = Real::elliptic_rf(&x, &y, &z).unwrap();
    let rd = Real::elliptic_rd(&x, &y, &z).unwrap();
    for scale in ["1e-320", "1e320"] {
        let scale: Real = scale.parse().unwrap();
        let root = scale.clone().sqrt().unwrap();
        close(
            &(Real::elliptic_rf(&(&x * &scale), &(&y * &scale), &(&z * &scale)).unwrap() * &root),
            &rf,
            96,
        );
        close(
            &(Real::elliptic_rd(&(&x * &scale), &(&y * &scale), &(&z * &scale)).unwrap()
                * root
                * scale),
            &rd,
            96,
        );
    }
}

#[test]
fn precision_sequences_abort_and_concurrent_readers() {
    let [x, y, z] = [2, 3, 4].map(Real::from);
    for value in [
        Real::elliptic_rf(&x, &y, &z).unwrap(),
        Real::elliptic_rd(&x, &y, &z).unwrap(),
    ] {
        let mut previous: Option<[Rational; 2]> = None;
        for bits in [32, 512, 64, 256] {
            let interval = value.certified_rational_interval(-bits).unwrap();
            assert!(interval[0] > Rational::zero());
            if let Some(old) = previous {
                assert!(old[0] <= interval[1] && interval[0] <= old[1]);
            }
            previous = Some(interval);
        }
        let signal = Arc::new(AtomicBool::new(true));
        let mut aborted = value.clone();
        aborted.abort(signal.clone());
        assert!(aborted.certified_rational_interval(-128).is_none());
        signal.store(false, Ordering::Relaxed);
        let expected = value.certified_rational_interval(-256).unwrap();
        let actual = aborted.certified_rational_interval(-256).unwrap();
        assert!(expected[0] <= actual[1] && actual[0] <= expected[1]);
        std::thread::scope(|scope| {
            for bits in [32, 128, 256, 512] {
                let value = value.clone();
                scope.spawn(move || {
                    let [lo, hi] = value.certified_rational_interval(-bits).unwrap();
                    assert!(lo > Rational::zero() && hi > lo);
                });
            }
        });
    }
    let opaque_zero = Real::e().sin().powi(2.into()).unwrap()
        + Real::e().cos().powi(2.into()).unwrap()
        - Real::one();
    assert_eq!(
        Real::elliptic_rf(&opaque_zero, &y, &z),
        Err(Problem::Exhausted)
    );
    assert_eq!(
        Real::elliptic_rd(&x, &y, &opaque_zero),
        Err(Problem::Exhausted)
    );
}

#[cfg(feature = "serde")]
#[test]
fn serialization_preserves_expression_sign_and_structural_identity() {
    let [x, y, z] = [2, 3, 5].map(Real::from);
    for value in [
        Real::elliptic_rf(&x, &y, &z).unwrap(),
        Real::elliptic_rd(&x, &y, &z).unwrap(),
    ] {
        let cold = value.to_json();
        let decoded = Real::from_json(&cold).unwrap();
        assert_eq!(decoded, value);
        assert_eq!(
            decoded.certified_sign_until(0).sign(),
            Some(RealSign::Positive)
        );
        let _ = value.certified_rational_interval(-512).unwrap();
        assert_eq!(cold, value.to_json());
        let mut bytes = Vec::new();
        ciborium::ser::into_writer(&value, &mut bytes).unwrap();
        let decoded: Real = ciborium::de::from_reader(bytes.as_slice()).unwrap();
        assert_eq!(decoded, value);
        close(&decoded, &value, 256);
    }
}
