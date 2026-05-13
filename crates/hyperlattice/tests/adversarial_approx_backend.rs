#![cfg(feature = "approx-backend")]

use hyperlattice::{
    ApproxBackend, Matrix3, Problem, Scalar, ScalarFacts, ScalarSign, Vector3, ZeroStatus,
};
use proptest::prelude::*;

type AScalar = Scalar<ApproxBackend>;
type AMatrix3 = Matrix3<ApproxBackend>;
type AVector3 = Vector3<ApproxBackend>;

fn a(value: f64) -> AScalar {
    AScalar::try_from(value).unwrap()
}

fn interval(value: f64, epsilon: f64) -> AScalar {
    AScalar::approx(value, epsilon).unwrap()
}

fn finite_center() -> impl Strategy<Value = f64> {
    -1.0e6_f64..=1.0e6
}

#[test]
fn approximate_intervals_preserve_unknown_zero_through_checked_guards() {
    let unknown = interval(0.0, 0.25);
    let touching = interval(0.25, 0.25);
    let positive = interval(1.0, 0.25);
    let negative = interval(-1.0, 0.25);

    assert_eq!(unknown.zero_status(), ZeroStatus::Unknown);
    assert_eq!(touching.zero_status(), ZeroStatus::Unknown);
    assert_eq!(positive.zero_status(), ZeroStatus::NonZero);
    assert_eq!(negative.zero_status(), ZeroStatus::NonZero);
    assert_eq!(
        hyperlattice::reciprocal_checked(unknown),
        Err(Problem::UnknownZero)
    );
    assert_eq!(
        hyperlattice::reciprocal_checked(touching),
        Err(Problem::UnknownZero)
    );
}

#[test]
fn approximate_structural_facts_never_overclaim_interval_signs() {
    assert_eq!(
        interval(0.0, 10.0).structural_facts(),
        ScalarFacts {
            sign: None,
            zero: ZeroStatus::Unknown,
            exact_rational: false,
            magnitude: None,
        }
    );

    let positive = interval(10.0, 1.0).structural_facts();
    assert_eq!(positive.sign, Some(ScalarSign::Positive));
    assert_eq!(positive.zero, ZeroStatus::NonZero);
    assert!(positive.magnitude.is_some());

    let negative = interval(-10.0, 1.0).structural_facts();
    assert_eq!(negative.sign, Some(ScalarSign::Negative));
    assert_eq!(negative.zero, ZeroStatus::NonZero);
    assert!(negative.magnitude.is_some());
}

#[test]
fn approximate_matrix_checked_paths_reject_unknown_pivots_and_divisors() {
    let unknown = interval(0.0, 0.5);
    let matrix = AMatrix3::new([
        [unknown, a(0.0), a(0.0)],
        [a(0.0), a(1.0), a(0.0)],
        [a(0.0), a(0.0), a(1.0)],
    ]);

    assert_eq!(matrix.clone().inverse_checked(), Err(Problem::UnknownZero));
    assert_eq!(
        AMatrix3::identity().div_matrix_checked(matrix),
        Err(Problem::UnknownZero)
    );
    assert_eq!(
        AMatrix3::identity().div_scalar_checked(interval(0.0, 0.1)),
        Err(Problem::UnknownZero)
    );
}

#[test]
fn approximate_vector_normalize_distinguishes_zero_unknown_and_known_nonzero() {
    assert_eq!(
        AVector3::zero().normalize_checked(),
        Err(Problem::DivideByZero)
    );

    let unknown = AVector3::new([interval(0.0, 0.25), a(0.0), a(0.0)]);
    assert_eq!(unknown.normalize_checked(), Err(Problem::UnknownZero));

    let known = AVector3::new([a(3.0), a(4.0), a(0.0)]);
    let unit = known.normalize_checked().unwrap();
    assert_eq!(unit.dot(&unit).zero_status(), ZeroStatus::NonZero);
    let norm_squared = unit.dot(&unit).to_f64_approx().unwrap();
    assert!((norm_squared - 1.0).abs() <= 1.0e-12);
}

#[test]
fn approximate_large_small_arithmetic_keeps_conservative_facts() {
    let huge = interval(1.0e200, 1.0e180);
    let tiny = interval(1.0e-200, 1.0e-220);
    let mixed = huge.clone() + tiny;
    let canceled = huge.clone() - huge;

    assert_eq!(mixed.zero_status(), ZeroStatus::NonZero);
    assert_eq!(mixed.structural_facts().sign, Some(ScalarSign::Positive));
    assert_eq!(canceled.zero_status(), ZeroStatus::Unknown);
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(256))]

    #[test]
    fn generated_approx_intervals_do_not_overclaim_zero_or_sign(center in finite_center(), epsilon in 0.0_f64..=1.0e6) {
        let value = interval(center, epsilon);
        let facts = value.structural_facts();

        prop_assert_eq!(value.zero_status(), facts.zero);
        if center.abs() <= epsilon {
            prop_assert_eq!(facts.zero, ZeroStatus::Unknown);
            prop_assert_eq!(facts.sign, None);
        } else {
            prop_assert_eq!(facts.zero, ZeroStatus::NonZero);
            prop_assert_eq!(
                facts.sign,
                Some(if center > 0.0 { ScalarSign::Positive } else { ScalarSign::Negative })
            );
        }
    }

    #[test]
    fn generated_approx_checked_division_rejects_only_intervals_touching_zero(center in finite_center(), epsilon in 0.0_f64..=1.0e6) {
        let divisor = interval(center, epsilon);
        let result = AVector3::new([a(1.0), a(2.0), a(3.0)]).div_scalar_checked(divisor);

        if center.abs() <= epsilon {
            prop_assert_eq!(result, Err(Problem::UnknownZero));
        } else {
            prop_assert!(result.is_ok());
        }
    }
}
