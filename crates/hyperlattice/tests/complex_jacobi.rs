//! Complex Jacobi regression witnesses for native relations, continuation and replay.
//!
//! Exact rational-normal-form assertions are distinct from finite-tolerance checks
//! and independent local ODE enclosure comparisons. The finite recipe list and
//! same-process serialization do not establish exhaustive representation coverage
//! or fresh-process replay.

use hyperlattice::{Complex, Problem, Rational, Real};
use std::sync::{
    Arc,
    atomic::{AtomicBool, Ordering},
};

/// Shared thread-local MPFR cache cleanup for the independent ODE test.
#[path = "../../hyperreal/tests/support/mpfr.rs"]
mod mpfr;
/// Shared labeled scalar recipes; tests assert public kinds, not distinct private tags or a coverage count.
#[path = "../../hyperreal/tests/support/representations.rs"]
mod representations;
/// Development-only reference implementation, separate from production rectangular continuation.
mod support {
    /// Local Taylor-ODE interval oracle and component comparison helpers.
    pub mod complex_jacobi_oracle;
}

/// Require both components to have the specified integer exact rational normal forms; unavailable symbolic proof fails the assertion.
fn exact(value: &Complex, real: i32, imaginary: i32) {
    assert_eq!(
        value.re.exact_rational_normal_form(),
        Some(Rational::from(real))
    );
    assert_eq!(
        value.im.exact_rational_normal_form(),
        Some(Rational::from(imaginary))
    );
}

/// Enclose each component difference at -bits-8 and require containment in ±2^-bits.
///
/// This is a finite absolute tolerance, not exact equality. Unavailable enclosures
/// or invalid arithmetic panic; callers use small positive bit counts.
fn close(actual: &Complex, expected: &Complex, bits: i32) {
    let tolerance = Rational::fraction(1, 2).unwrap().powi(bits.into()).unwrap();
    for (a, b) in [(&actual.re, &expected.re), (&actual.im, &expected.im)] {
        let [lo, hi] = (a - b).certified_rational_interval(-bits - 8).unwrap();
        assert!(
            lo >= -tolerance.clone() && hi <= tolerance,
            "error [{lo}, {hi}]"
        );
    }
}

/// Borrow the six scalar components in sn.re, sn.im, cn.re, cn.im, dn.re, dn.im order for evaluation and serialization.
fn components(values: &(Complex, Complex, Complex)) -> [&Real; 6] {
    [
        &values.0.re,
        &values.0.im,
        &values.1.re,
        &values.1.im,
        &values.2.re,
        &values.2.im,
    ]
}

/// Check structural agreement with real evaluation for m=1/2,-3,0, then conjugation and odd-sn/even-cn,dn parity within 2^-160 at one nonreal argument.
#[test]
fn real_axis_conjugation_and_parity_preserve_the_native_branch() {
    for text in ["0.5", "-3", "0"] {
        let m: Real = text.parse().unwrap();
        let x: Real = "0.125".parse().unwrap();
        let real = x.clone().jacobi_sncndn(&m).unwrap();
        let complex = Complex::from(x.clone()).jacobi_sncndn(&m).unwrap();
        assert_eq!(
            complex,
            (
                Complex::from(real.0),
                Complex::from(real.1),
                Complex::from(real.2)
            )
        );
        let z = Complex::new(x, "0.0625".parse().unwrap());
        let original = z.clone().jacobi_sncndn(&m).unwrap();
        let conjugate = z.clone().conjugate().jacobi_sncndn(&m).unwrap();
        let negative = (-z).jacobi_sncndn(&m).unwrap();
        for (actual, expected) in [conjugate.0, conjugate.1, conjugate.2].into_iter().zip([
            original.0.clone(),
            original.1.clone(),
            original.2.clone(),
        ]) {
            close(&actual, &expected.conjugate(), 160);
        }
        for (actual, expected) in [negative.0, negative.1, negative.2].into_iter().zip([
            -original.0,
            original.1,
            original.2,
        ]) {
            close(&actual, &expected, 160);
        }
    }
}

/// Check successive enclosure overlap in a 32→512→64→256 sequence for six components at m=-3.
///
/// An already-set abort signal must prevent a -128 enclosure; clearing it restores
/// 2^-256 closeness. Four reader threads require available intervals, without
/// cross-reader equality assertions. Finally require exact square-sum relations.
#[test]
fn cache_precision_order_abort_recovery_and_concurrent_readers() {
    let values = Complex::new("0.25".parse().unwrap(), "0.125".parse().unwrap())
        .jacobi_sncndn(&Real::from(-3))
        .unwrap();
    for value in components(&values) {
        let mut previous: Option<[Rational; 2]> = None;
        for bits in [32, 512, 64, 256] {
            let current = value.certified_rational_interval(-bits).unwrap();
            if let Some(old) = previous {
                assert!(old[0] <= current[1] && current[0] <= old[1]);
            }
            previous = Some(current);
        }
        let signal = Arc::new(AtomicBool::new(true));
        let mut aborted = value.clone();
        aborted.abort(signal.clone());
        assert!(aborted.certified_rational_interval(-128).is_none());
        signal.store(false, Ordering::Relaxed);
        close(&Complex::from(aborted), &Complex::from(value.clone()), 256);
        std::thread::scope(|scope| {
            for bits in [32, 128, 256, 512] {
                scope.spawn(move || assert!(value.certified_rational_interval(-bits).is_some()));
            }
        });
    }
    relations(&values, &Real::from(-3));
}

/// Use sin(e)^2+cos(e)^2-1 as mathematically zero with unresolved sign.
///
/// Require Exhausted for a nonreal zero-parameter branch and an unresolved m=1
/// boundary, real-axis admission at m=0, and UnknownZero at an unresolved pole.
/// Unknown knowledge is not evidence that the fixture is nonzero.
#[test]
fn unresolved_parameters_and_poles_keep_distinct_errors() {
    let zero = Real::e().sin().powi(2.into()).unwrap() + Real::e().cos().powi(2.into()).unwrap()
        - Real::one();
    let half: Real = "0.5".parse().unwrap();
    assert_eq!(Complex::i().jacobi_sncndn(&zero), Err(Problem::Exhausted));
    assert!(
        Complex::zero().jacobi_sncndn(&zero).is_ok(),
        "the real axis needs no sign split at m=0"
    );
    assert_eq!(
        Complex::zero().jacobi_sncndn(&(Real::one() - &zero)),
        Err(Problem::Exhausted)
    );
    let k = Real::elliptic_k(&half).unwrap();
    assert_eq!(
        Complex::new(zero, k).jacobi_sncndn(&half),
        Err(Problem::UnknownZero)
    );
}

/// Translate both coordinates by 10^30 common period cycles for m=1/2 and -3 and compare within 2^-128.
///
/// For z=10^-80*(1-i), require sn close to z at 700 bits, cn/dn close to one at
/// 500 bits, and exact square-sum relations. These fixtures are not uniform bounds
/// for all translations or small arguments.
#[test]
fn large_period_translations_and_small_arguments_remain_consumable() {
    let cycles: Real = "1e30".parse().unwrap();
    let small = Complex::new("0.125".parse().unwrap(), "0.0625".parse().unwrap());
    for m in ["0.5".parse::<Real>().unwrap(), Real::from(-3)] {
        let real_period = Real::from(4) * Real::elliptic_k(&m).unwrap();
        let complement = Real::one() - &m;
        let imaginary_period = if m == Real::from(-3) {
            (Real::from(4) * Real::elliptic_k(&complement.clone().inverse().unwrap()).unwrap()
                / complement.sqrt().unwrap())
            .unwrap()
        } else {
            Real::from(4) * Real::elliptic_k(&complement).unwrap()
        };
        let shifted = Complex::new(
            &small.re + &cycles * real_period,
            &small.im + &cycles * imaginary_period,
        )
        .jacobi_sncndn(&m)
        .unwrap();
        let expected = small.clone().jacobi_sncndn(&m).unwrap();
        for (a, b) in [shifted.0, shifted.1, shifted.2]
            .into_iter()
            .zip([expected.0, expected.1, expected.2])
        {
            close(&a, &b, 128);
        }
        let tiny = Complex::new("1e-80".parse().unwrap(), "-1e-80".parse().unwrap());
        let values = tiny.clone().jacobi_sncndn(&m).unwrap();
        close(&values.0, &tiny, 700);
        close(&values.1, &Complex::one(), 500);
        close(&values.2, &Complex::one(), 500);
        relations(&values, &m);
    }
}

/// For real offsets 10^-20 and 10^-60 near selected poles at m=1/2 and -3, require six -128 enclosures and exact square-sum relations; no independent value oracle is used here.
#[test]
fn exact_near_pole_offsets_do_not_become_poles() {
    for text in ["1e-20", "1e-60"] {
        let offset: Real = text.parse().unwrap();
        for m in ["0.5".parse::<Real>().unwrap(), Real::from(-3)] {
            let (x, y) = if m == Real::from(-3) {
                (
                    Real::elliptic_k(&m).unwrap() + &offset,
                    (Real::elliptic_k(&"0.25".parse().unwrap()).unwrap() / Real::from(2)).unwrap(),
                )
            } else {
                (offset.clone(), Real::elliptic_k(&m).unwrap())
            };
            let values = Complex::new(x, y).jacobi_sncndn(&m).unwrap();
            for value in components(&values) {
                assert!(value.certified_rational_interval(-128).is_some());
            }
            relations(&values, &m);
        }
    }
}

/// At z=1000i and m=0 compare sn/cn with exponential formulas at 128 bits, require exact dn=1, and consume all results in exact square-sum relations.
#[test]
fn zero_parameter_supports_large_imaginary_arguments() {
    let y = Real::from(1000);
    let values = Complex::new(Real::zero(), y.clone())
        .jacobi_sncndn(&Real::zero())
        .unwrap();
    let exponential = y.exp().unwrap();
    let reciprocal = exponential.clone().inverse().unwrap();
    close(
        &values.0,
        &Complex::new(
            Real::zero(),
            ((&exponential - &reciprocal) / Real::from(2)).unwrap(),
        ),
        128,
    );
    close(
        &values.1,
        &Complex::from(((exponential + reciprocal) / Real::from(2)).unwrap()),
        128,
    );
    exact(&values.2, 1, 0);
    relations(&values, &Real::zero());
}

/// Round-trip six components for m=1/2,-3,pi/8 through JSON in the same process.
///
/// Independently evaluate original and decoded intervals at -256 before any
/// structural cancellation, require overlap and 2^-256 closeness, then exact
/// square-sum relations and deterministic text. This does not establish replay
/// in a fresh process or preserve a separately serialized complex proof receipt.
#[cfg(feature = "serde")]
#[test]
fn serialized_components_replay_values_and_exact_relations() {
    for m in [
        "0.5".parse().unwrap(),
        Real::from(-3),
        (Real::pi() / Real::from(8)).unwrap(),
    ] {
        let original = Complex::new(Real::e(), "0.125".parse().unwrap())
            .jacobi_sncndn(&m)
            .unwrap();
        let serialized = serde_json::to_string(&components(&original)).unwrap();
        let decoded: [Real; 6] = serde_json::from_str(&serialized).unwrap();
        let [a, b, c, d, e, f] = decoded;
        let decoded = (Complex::new(a, b), Complex::new(c, d), Complex::new(e, f));
        // Serde deliberately omits scalar caches. Comparing identical
        // carriers can cancel before evaluating either one; independently
        // replay every component, including its inverse-domain obligations.
        for (decoded, original) in components(&decoded).into_iter().zip(components(&original)) {
            let [a, b] = decoded.certified_rational_interval(-256).unwrap();
            let [c, d] = original.certified_rational_interval(-256).unwrap();
            assert!(a <= d && c <= b, "serialized component, parameter {m}");
        }
        for (a, b) in [&decoded.0, &decoded.1, &decoded.2].into_iter().zip([
            &original.0,
            &original.1,
            &original.2,
        ]) {
            close(a, b, 256);
        }
        relations(&decoded, &m);
        assert_eq!(
            serialized,
            serde_json::to_string(&components(&decoded)).unwrap()
        );
    }
}

/// Require exact unit and elliptic square-sum normal forms for five parameters at z=1/4+i/8.
#[test]
fn complex_jacobi_relations_survive_ordinary_arithmetic() {
    for text in ["0.5", "0.75", "-3", "-0.5", "0"] {
        let parameter: Real = text.parse().unwrap();
        let argument = Complex::new("0.25".parse().unwrap(), "0.125".parse().unwrap());
        relations(&argument.jacobi_sncndn(&parameter).unwrap(), &parameter);
    }
}

/// Require sn²+cn²=1, dn²+m*sn²=1, and dn²-m*cn²-(1-m)=0 through ordinary complex arithmetic.
///
/// Each component must reduce to an exact rational normal form. The third identity
/// is algebraically dependent but exercises a different expression shape. These
/// relations alone do not select a unique Jacobi value or branch.
fn relations((sn, cn, dn): &(Complex, Complex, Complex), parameter: &Real) {
    exact(&(sn * sn + cn * cn), 1, 0);
    exact(
        &(dn * dn + Complex::from(parameter.clone()) * (sn * sn)),
        1,
        0,
    );
    exact(
        &(dn * dn
            - Complex::from(parameter.clone()) * (cn * cn)
            - Complex::from(Real::one() - parameter)),
        0,
        0,
    );
}

/// Visit the shared finite recipes, assert each public kind, and require exact square-sum relations with scaled positive, negative and zero parameters; no distinct-private-tag count is asserted.
#[test]
fn native_families_preserve_complex_relations() {
    for case in representations::optimized_certificate_representatives() {
        assert_eq!(
            case.value.detailed_facts().symbolic.kind,
            case.public_kind,
            "{}",
            case.certificate
        );
        let small = (&case.value / (Real::from(4) * (&case.value + Real::one()))).unwrap();
        for parameter in [small.clone(), -small.clone(), Real::zero()] {
            let values = Complex::new(case.value.clone(), small.clone())
                .jacobi_sncndn(&parameter)
                .unwrap();
            relations(&values, &parameter);
        }
    }
}

/// Require NotANumber at the zero complex argument for m=1 and m=2, preserving parameter validation before value shortcuts.
#[test]
fn invalid_parameters_are_checked_before_zero_shortcuts() {
    for parameter in [Real::one(), Real::from(2)] {
        assert_eq!(
            Complex::zero().jacobi_sncndn(&parameter),
            Err(Problem::NotANumber)
        );
    }
}

/// Compare all six components against the local ODE oracle at twenty parameter/argument combinations, with both signs, axes and m=0; this is finite enclosure evidence, not exact equality.
#[test]
fn independent_complex_ode_enclosures() {
    mpfr::cleanup_on_thread_exit();
    for parameter in ["0.5", "0.75", "-3", "-0.5", "0"] {
        for (re, im) in [
            ("0.125", "0.0625"),
            ("-0.125", "0.0625"),
            ("0", "-0.125"),
            ("0.125", "0"),
        ] {
            support::complex_jacobi_oracle::compare(
                &Complex::new(re.parse().unwrap(), im.parse().unwrap()),
                &parameter.parse().unwrap(),
                false,
            );
        }
    }
}

/// Check exact finite quarter-period values, including complex dn=0, and DivideByZero at selected true poles.
///
/// Uses m=1/2 and -3. Negative-parameter sn/cn assertions check zero components
/// and squared magnitudes, not independently their remaining signs.
#[test]
fn quarter_periods_distinguish_finite_values_from_poles() {
    let half: Real = "0.5".parse().unwrap();
    let k = Real::elliptic_k(&half).unwrap();
    assert_eq!(
        Complex::new(Real::zero(), k.clone()).jacobi_sncndn(&half),
        Err(Problem::DivideByZero)
    );
    let (sn, cn, dn) = Complex::new(k.clone(), k.clone())
        .jacobi_sncndn(&half)
        .unwrap();
    assert_eq!(sn.re, Real::from(2).sqrt().unwrap());
    assert!(sn.im.definitely_zero());
    exact(&cn, 0, -1);
    exact(&dn, 0, 0);
    let (sn, cn, dn) = Complex::new(Real::zero(), Real::from(2) * k)
        .jacobi_sncndn(&half)
        .unwrap();
    exact(&sn, 0, 0);
    exact(&cn, -1, 0);
    exact(&dn, -1, 0);

    let m = Real::from(-3);
    let quarter = (Real::elliptic_k(&"0.25".parse().unwrap()).unwrap() / Real::from(2)).unwrap();
    let (sn, cn, dn) = Complex::new(Real::zero(), quarter.clone())
        .jacobi_sncndn(&m)
        .unwrap();
    assert!(sn.re.definitely_zero());
    assert_eq!(
        (&sn.im * &sn.im).exact_rational_normal_form(),
        Some(Rational::fraction(1, 3).unwrap())
    );
    assert!(cn.im.definitely_zero());
    assert_eq!(
        (&cn.re * &cn.re).exact_rational_normal_form(),
        Some(Rational::fraction(4, 3).unwrap())
    );
    exact(&dn, 0, 0);
    assert_eq!(
        Complex::new(Real::elliptic_k(&m).unwrap(), quarter).jacobi_sncndn(&m),
        Err(Problem::DivideByZero)
    );
}
