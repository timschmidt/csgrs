use hyperreal::{Problem, Rational, Real, RealSign};
use std::sync::{
    Arc,
    atomic::{AtomicBool, Ordering},
};

#[path = "support/inverse_legendre_oracle.rs"]
/// Supply a residual-checked MPFR inverse reference independent of native Carlson refinement.
mod inverse_oracle;
#[path = "support/legendre_oracle.rs"]
/// Supply shared forward-series and complete-AGM references plus exact reference-value transfer.
mod legendre_oracle;
#[path = "support/mpfr.rs"]
/// Register thread-local cleanup for the development MPFR backend.
mod mpfr;
use legendre_oracle as oracle;
#[path = "support/representations.rs"]
/// Supply the finite shared inventory of 22 positive scalar construction recipes.
mod representations;

/// Select Jacobi amplitude or inverse E for a valid fixture; fail the test on a typed construction error.
fn inverse(value: Real, m: &Real, second: bool) -> Real {
    if second {
        value.inverse_elliptic_e(m)
    } else {
        value.jacobi_amplitude(m)
    }
    .unwrap()
}

/// Select the corresponding unwrapped forward F/E operation for a valid fixture.
fn forward(value: Real, m: &Real, second: bool) -> Real {
    if second {
        value.elliptic_e(m)
    } else {
        value.elliptic_f(m)
    }
    .unwrap()
}

/// Select the complete F/E value used to construct period fixtures and independent expectations.
fn complete(m: &Real, second: bool) -> Real {
    if second {
        Real::elliptic_e_complete(m)
    } else {
        Real::elliptic_k(m)
    }
    .unwrap()
}

/// Require an eight-guard-bit enclosure of actual-expected within +/-2^(-bits).
/// Use exact rational endpoint comparisons for absolute-error qualification,
/// conditional on the scalar enclosure contract; this is not correct-rounding
/// or relative-error certification.
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
/// Preserve three circular input values exactly and reject m=1,2 before zero-input shortcuts.
/// Use zero, e and 1e320 for both inverses at m=0; invalid parameter assertions
/// require typed NotANumber and keep the common m<1 domain explicit.
fn inverse_circular_and_invalid_parameter_contract() {
    for value in [Real::zero(), Real::e(), "1e320".parse().unwrap()] {
        assert_eq!(
            value.clone().jacobi_amplitude(&Real::zero()).unwrap(),
            value
        );
        assert_eq!(
            value.clone().inverse_elliptic_e(&Real::zero()).unwrap(),
            value
        );
    }
    for parameter in [Real::one(), Real::from(2)] {
        assert_eq!(
            Real::zero().jacobi_amplitude(&parameter),
            Err(Problem::NotANumber)
        );
        assert_eq!(
            Real::zero().inverse_elliptic_e(&parameter),
            Err(Problem::NotANumber)
        );
    }
}

#[test]
/// Check 88 inverse evaluations from independent series points across 22 scalar recipes.
/// Validate each structural category, form s=v/(4*(v+1)) and phi=pi+s, and use
/// parameters +/-s for both kinds. Import rounded MPFR integral references as
/// fresh exact rational values, preventing paired native forward-node cancellation.
/// Demand 512-bit absolute agreement and register MPFR thread cleanup.
fn fresh_independent_integrals_cover_all_scalar_families() {
    mpfr::cleanup_on_thread_exit();
    for case in representations::optimized_certificate_representatives() {
        eprintln!("Inverse Legendre independent oracle: {}", case.certificate);
        assert_eq!(case.value.detailed_facts().symbolic.kind, case.public_kind);
        let small = (&case.value / (Real::from(4) * (&case.value + Real::one()))).unwrap();
        let phi = Real::pi() + &small;
        for m in [small.clone(), -small] {
            // Import a fresh exact rational MPFR point: no paired native node
            // exists for a structural inverse shortcut to cancel.
            let integrals = oracle::series(&phi, &m).map(oracle::native);
            for (second, value) in [false, true].into_iter().zip(integrals) {
                close(&inverse(value, &m, second), &phi, 512);
            }
        }
    }
}

#[test]
/// Recover 24 signed principal/quarter/period fixtures from independent forward-series values.
/// Use parameters +/-3/4 and amplitudes +/-5/16, +/-1.8 and +/-7.5 for both
/// kinds, with 256-bit absolute comparisons. Register MPFR cleanup; the finite
/// corpus does not enumerate all period choices or seam behavior.
fn independently_sampled_principal_and_odd_quarter_roots() {
    mpfr::cleanup_on_thread_exit();
    for text in ["0.75", "-0.75"] {
        let m: Real = text.parse().unwrap();
        for text in ["0.3125", "-0.3125", "1.8", "-1.8", "7.5", "-7.5"] {
            let phi: Real = text.parse().unwrap();
            let values = oracle::series(&phi, &m).map(oracle::native);
            for (second, value) in [false, true].into_iter().zip(values) {
                close(&inverse(value, &m, second), &phi, 256);
            }
        }
    }
}

#[test]
/// Compare twelve fresh rational inverse inputs with the independent safeguarded MPFR solver.
/// Use parameters 1/2,-3/4, values 5/16,-1.8,3.5 and both kinds at 512-bit
/// absolute accuracy. Register backend cleanup. The oracle's residual threshold
/// is a separate assertion from the native/reference comparison.
fn independent_inverse_oracle_accepts_fresh_rational_inputs() {
    mpfr::cleanup_on_thread_exit();
    for m in ["0.5", "-0.75"] {
        let m: Real = m.parse().unwrap();
        for x in ["0.3125", "-1.8", "3.5"] {
            let x: Real = x.parse().unwrap();
            for second in [false, true] {
                let expected = oracle::native(inverse_oracle::inverse(&x, &m, second));
                close(&inverse(x.clone(), &m, second), &expected, 512);
            }
        }
    }
}

#[test]
/// Check normalized inverse increments for +/-1e-320 displacements from a complete integral.
/// At m=3/4 compare the displacement divided by its input offset with inverse
/// endpoint derivatives one half for F and two for E, at 96 bits. Snapping the
/// represented offsets to the exact quarter identity would fail this check.
fn tiny_offsets_from_quarter_periods_are_never_snapped() {
    let m: Real = "0.75".parse().unwrap();
    let half_pi = (Real::pi() / Real::from(2)).unwrap();
    for second in [false, true] {
        for text in ["1e-320", "-1e-320"] {
            let delta: Real = text.parse().unwrap();
            let value = inverse(complete(&m, second) + &delta, &m, second);
            let derivative: Real = if second { "2" } else { "0.5" }.parse().unwrap();
            close(&((value - &half_pi) / delta).unwrap(), &derivative, 96);
        }
    }
}

#[test]
/// Check signed quarter inverses and a first-kind half-period identity using AGM references.
/// Use complements 1/4,4,1e-160,1e160; both signed complete references recover
/// +/-pi/2 for both kinds. Independently expect atan(c^(-1/4)) at K/2 for Jacobi
/// amplitude. Comparisons use 192-bit absolute accuracy and MPFR cleanup; the
/// reference points are rounded oracle values, not retained native complete nodes.
fn agm_quarter_periods_and_jacobi_half_period_are_independent() {
    mpfr::cleanup_on_thread_exit();
    let half_pi = (Real::pi() / Real::from(2)).unwrap();
    for text in ["0.25", "4", "1e-160", "1e160"] {
        let c: Real = text.parse().unwrap();
        let m = Real::one() - &c;
        let [k, e] = oracle::complete(&c).map(oracle::native);
        for (second, value) in [(false, k.clone()), (true, e)] {
            for sign in [-1, 1] {
                close(
                    &inverse(Real::from(sign) * &value, &m, second),
                    &(Real::from(sign) * &half_pi),
                    192,
                );
            }
        }
        // F(atan(c^(-1/4)) | 1-c) = K/2, by the quadratic transformation.
        let angle = (Real::one() / c.sqrt().unwrap().sqrt().unwrap())
            .unwrap()
            .atan()
            .unwrap();
        close(
            &(k / Real::from(2)).unwrap().jacobi_amplitude(&m).unwrap(),
            &angle,
            192,
        );
    }
}

#[test]
/// Verify exact quarter multiples, both composition directions and rejection of mismatched relations.
/// At m=3/4 require exact identities for counts -7,1,2,1e320; check five preimages
/// at 512 bits and exact public carriers for selected rational/pi results.
/// Changing the parameter or integral kind must produce a certified nonzero
/// difference for the selected 5/16 fixture, not an accidental cancellation.
fn defining_relations_and_exact_quarter_multiples_survive() {
    let m: Real = "0.75".parse().unwrap();
    let half_pi = (Real::pi() / Real::from(2)).unwrap();
    for second in [false, true] {
        for text in ["-7", "1", "2", "1e320"] {
            let n: Real = text.parse().unwrap();
            assert_eq!(
                inverse(&n * complete(&m, second), &m, second),
                &n * &half_pi
            );
        }
        for value in [
            Real::zero(),
            Real::from(7),
            half_pi.clone(),
            Real::e(),
            Real::from(2).sqrt().unwrap(),
        ] {
            close(
                &inverse(forward(value.clone(), &m, second), &m, second),
                &value,
                512,
            );
            close(
                &forward(inverse(value.clone(), &m, second), &m, second),
                &value,
                512,
            );
        }
        assert_eq!(
            inverse(forward(Real::from(7), &m, second), &m, second),
            Real::from(7)
        );
        assert_eq!(
            forward(inverse(half_pi.clone(), &m, second), &m, second),
            half_pi
        );
        let x: Real = "0.3125".parse().unwrap();
        let other: Real = "-0.5".parse().unwrap();
        for mismatched in [
            inverse(forward(x.clone(), &m, second), &other, second),
            inverse(forward(x.clone(), &m, second), &m, !second),
            forward(inverse(x.clone(), &m, second), &other, second),
            forward(inverse(x.clone(), &m, second), &m, !second),
        ] {
            assert!(
                (&mismatched - &x)
                    .certified_sign_until(-128)
                    .sign()
                    .is_some_and(|sign| sign != RealSign::Zero)
            );
        }
    }
}

#[test]
/// Check normalized inverse behavior at signed 1e320 and 1e-320 inputs with m=1/2.
/// For both kinds compare the large ratio with pi/(2Q) and the tiny ratio with
/// one at 128 bits. These finite asymptotic fixtures neither wrap the inverse
/// nor establish a resource bound for arbitrary magnitudes.
fn huge_and_tiny_inputs_keep_the_unwrapped_real_domain() {
    let m: Real = "0.5".parse().unwrap();
    for second in [false, true] {
        for sign in [-1, 1] {
            let huge = Real::from(sign) * "1e320".parse::<Real>().unwrap();
            let tiny = Real::from(sign) * "1e-320".parse::<Real>().unwrap();
            close(
                &(inverse(huge.clone(), &m, second) / huge).unwrap(),
                &(Real::pi() / (Real::from(2) * complete(&m, second))).unwrap(),
                128,
            );
            close(
                &(inverse(tiny.clone(), &m, second) / tiny).unwrap(),
                &Real::one(),
                128,
            );
        }
    }
}

#[test]
/// Reject an unproved positive complement even for zero input, while admitting an undecided zero value.
/// Use an opaque trigonometric zero as the complement to require Exhausted from
/// both public inverses. As an input at known m=1/2 it remains valid and agrees
/// with zero at 128 bits; input uncertainty and parameter admission are distinct.
fn unresolved_domain_is_not_a_zero_shortcut() {
    let opaque_zero = Real::e().sin().powi(2.into()).unwrap()
        + Real::e().cos().powi(2.into()).unwrap()
        - Real::one();
    let m = Real::one() - &opaque_zero;
    assert_eq!(Real::zero().jacobi_amplitude(&m), Err(Problem::Exhausted));
    assert_eq!(Real::zero().inverse_elliptic_e(&m), Err(Problem::Exhausted));
    for second in [false, true] {
        close(
            &inverse(opaque_zero.clone(), &"0.5".parse().unwrap(), second),
            &Real::zero(),
            128,
        );
    }
}

#[test]
/// Check sign retention, overlapping refinement, preexisting abort/retry and four shared readers.
/// For both inverses at value e and m=-3, request 32,512,64,256 bits and require
/// successive overlap rather than nesting. Clearing an asserted flag recovers
/// 256-bit agreement; concurrent readers require positive lower endpoints.
/// This same-process test does not measure interruption latency or inspect caches.
fn precision_order_cancellation_recovery_and_concurrent_readers() {
    let m: Real = "-3".parse().unwrap();
    for second in [false, true] {
        let value = inverse(Real::e(), &m, second);
        assert_eq!(
            value.certified_sign_until(0).sign(),
            Some(RealSign::Positive)
        );
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
        close(&aborted, &value, 256);
        std::thread::scope(|scope| {
            for bits in [32, 128, 256, 512] {
                let value = value.clone();
                scope.spawn(move || {
                    assert!(value.certified_rational_interval(-bits).unwrap()[0] > Rational::zero())
                });
            }
        });
    }
}

#[cfg(feature = "serde")]
#[test]
/// Check same-process JSON/CBOR replay and both forward/inverse relations at signed e and m=3/4.
/// Require structural identity, matching signs, forward recovery at 512 bits,
/// unchanged JSON after warming and CBOR agreement at 256 bits. Reimport a
/// forward expression before inversion as well. These assertions do not qualify
/// hostile payloads or fresh-process cache isolation.
fn serialization_preserves_values_and_both_defining_relations() {
    let m: Real = "0.75".parse().unwrap();
    for second in [false, true] {
        for x in [Real::e(), -Real::e()] {
            let value = inverse(x.clone(), &m, second);
            let cold = value.to_json();
            let decoded = Real::from_json(&cold).unwrap();
            assert_eq!(value, decoded);
            assert_eq!(
                decoded.certified_sign_until(0).sign(),
                x.certified_sign_until(0).sign()
            );
            close(&forward(decoded, &m, second), &x, 512);
            let _ = value.certified_rational_interval(-256).unwrap();
            assert_eq!(cold, value.to_json());
            let mut bytes = Vec::new();
            ciborium::ser::into_writer(&value, &mut bytes).unwrap();
            let decoded: Real = ciborium::de::from_reader(bytes.as_slice()).unwrap();
            assert_eq!(value, decoded);
            close(&decoded, &value, 256);
            let forward_json = forward(x.clone(), &m, second).to_json();
            close(
                &inverse(Real::from_json(&forward_json).unwrap(), &m, second),
                &x,
                512,
            );
        }
    }
}
