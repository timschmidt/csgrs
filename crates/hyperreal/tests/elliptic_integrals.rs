use hyperreal::{Problem, Rational, Real, RealSign};
use rug::{Float, float::Constant};
use std::sync::{
    Arc,
    atomic::{AtomicBool, Ordering},
};

#[path = "support/mpfr.rs"]
/// Register thread-local cleanup for the development-only MPFR oracle backend.
mod mpfr;
#[path = "support/representations.rs"]
/// Supply the shared finite inventory of positive scalar construction recipes.
mod representations;

/// Round the midpoint of a -896 Real enclosure to a 1024-bit MPFR oracle input.
/// Transfer numerator, denominator and sign explicitly instead of parsing mixed
/// fraction Display text. The result is a rounded reference value, not a directed
/// MPFR enclosure; callers register backend cleanup before using it.
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

/// Adopt a finite MPFR value's exact binary rational as a test reference Real.
/// This conversion preserves that rounded oracle value, not the exact integral
/// it approximates. Nonfinite values or invalid rational text fail the fixture.
fn from_mpfr(value: Float) -> Real {
    value.to_rational().unwrap().to_string().parse().unwrap()
}

/// Compute independent RF/RD test references from a transformed defining-integrand series.
/// Normalize with u=sqrt(z/(t+z)). For the fixtures, z=4 and
/// A=1-x/z, B=1-y/z lie in `[0,3/4]`. Expand
/// `((1-A*u^2)*(1-B*u^2))^(-1/2)` using its two-term coefficient recurrence.
/// Weight 2048 coefficients by 1/(2n+1) for RF or 1/(2n+3) for RD, then apply
/// 1/sqrt(z) or 3/z^(3/2). This differs from the native duplication algorithm.
///
/// The geometric coefficient bound gives an analytic omitted tail below 2^-840
/// for this corpus. Inputs and arithmetic use nearest-rounded 1024-bit MPFR;
/// the tail statement alone is not a bound on all rounding error. Return exact
/// rationals of the final rounded references, never production integral results.
/// The caller owns the fixture range and MPFR cleanup registration.
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

/// Require an eight-guard-bit enclosure of actual-expected inside +/-2^(-bits).
/// Compare exact rational endpoints for absolute-error qualification. This relies
/// on the scalar enclosure API and does not assert relative accuracy or identical
/// rounding of the native result and the reference.
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
/// Check exact diagonal collapses and 192-bit circular endpoint identities at four positive squares.
/// Use x=1,4,9,25 for RF/RD diagonal values and RF(0,x,x), RD(0,x,x) formulas
/// involving pi. Invalid zero diagonals are tested separately before shortcuts.
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
/// Reject seven negative or multiply-zero triples for both integrals and RD's zero third argument.
/// Require typed NotANumber, including on the all-zero diagonal. This finite
/// corpus does not cover mixed invalid and unresolved sign-query precedence.
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
/// Check structural positivity, supported permutations, homogeneity and the cyclic RD sum.
/// Start from (2,3,5); compare permutations and cyclic identity at 192 bits and
/// scales 7,1e-40,1e40 at 160 bits. The cyclic expectation comes from integrating
/// the derivative of the reciprocal square-root product, independently of duplication.
/// RF is fully symmetric; RD only exchanges its first two arguments.
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
/// Compare RF/RD with independent 2048-term references for all 22 scalar fixture recipes.
/// Check each public structural category, then map the positive value v to
/// x=1+v^2/(1+v^2), with y=3 and z=4. Demand 512-bit absolute agreement.
/// This mapping supplies the series range; the finite recipe inventory does not
/// cover every value or possible retained expression.
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
/// Compare RF(0,y,1) to pi/(2*AGM(1,sqrt(y))) for six positive y fixtures.
/// Values span 1e-160 through 1e40, including 0.001,0.5 and 2; require 256-bit
/// absolute agreement with the 1024-bit MPFR reference. No y=0 divergence is
/// admitted, and the oracle is independent of duplication. Register backend cleanup.
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
/// Check normalized RF/RD homogeneity at uniform scales 1e-320 and 1e320.
/// Use (2,3,5), retaining exact square-root scale compensation before a 96-bit
/// absolute comparison. This qualifies finite extreme fixtures without claiming
/// bounded work for arbitrary magnitudes or relative error on unnormalized outputs.
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
/// Exercise RF/RD refinement, preexisting abort/retry, four concurrent readers and unresolved domains.
/// Request 32,512,64,256 fractional bits and require positive, pairwise-overlapping
/// successive intervals; overlap is weaker than nestedness or an independent oracle.
/// After clearing the abort flag, compare replay intervals for overlap. Concurrent
/// readers require positive nondegenerate bounds. A trigonometric opaque zero must
/// return Exhausted in unresolved domain positions rather than be assumed zero.
/// This is same-process cache evidence, not mid-evaluation cancellation latency.
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
/// Check RF/RD JSON and CBOR replay at (2,3,5), including positivity and stable JSON after warming.
/// Require structural equality, a zero-bit positive sign, and 256-bit agreement
/// of CBOR replay with the original. These same-process comparisons distinguish
/// serialization stability from approximation behavior, but do not qualify hostile
/// payloads, independent mathematical equality or fresh-process cache isolation.
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
