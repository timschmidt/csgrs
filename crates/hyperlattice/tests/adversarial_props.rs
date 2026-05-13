mod common;

use common::frac;
use hyperlattice::{Complex, Matrix3, Matrix4, Problem, Scalar, ScalarSign, Vector3, ZeroStatus};
use proptest::prelude::*;

fn scalar_i(value: i32) -> Scalar {
    value.into()
}

fn nonzero_i32() -> impl Strategy<Value = i32> {
    (-16_i32..=16).prop_filter("nonzero", |value| *value != 0)
}

fn small_scalar() -> impl Strategy<Value = Scalar> {
    prop_oneof![
        (-64_i32..=64).prop_map(scalar_i),
        (-64_i64..=64, 1_u64..=64).prop_map(|(n, d)| frac(n, d)),
        Just(Scalar::pi()),
        Just(Scalar::e()),
        Just(hyperlattice::sqrt(scalar_i(2)).unwrap()),
    ]
}

fn small_exact_scalar() -> impl Strategy<Value = Scalar> {
    prop_oneof![
        (-64_i32..=64).prop_map(scalar_i),
        (-64_i64..=64, 1_u64..=64).prop_map(|(n, d)| frac(n, d)),
    ]
}

fn upper_triangular3() -> impl Strategy<Value = Matrix3> {
    (
        nonzero_i32(),
        -8_i32..=8,
        -8_i32..=8,
        nonzero_i32(),
        -8_i32..=8,
        nonzero_i32(),
    )
        .prop_map(|(d0, a01, a02, d1, a12, d2)| {
            Matrix3::new([
                [scalar_i(d0), scalar_i(a01), scalar_i(a02)],
                [scalar_i(0), scalar_i(d1), scalar_i(a12)],
                [scalar_i(0), scalar_i(0), scalar_i(d2)],
            ])
        })
}

fn upper_triangular4() -> impl Strategy<Value = Matrix4> {
    (
        nonzero_i32(),
        -4_i32..=4,
        -4_i32..=4,
        -4_i32..=4,
        nonzero_i32(),
        -4_i32..=4,
        -4_i32..=4,
        nonzero_i32(),
        -4_i32..=4,
        nonzero_i32(),
    )
        .prop_map(|(d0, a01, a02, a03, d1, a12, a13, d2, a23, d3)| {
            Matrix4::new([
                [scalar_i(d0), scalar_i(a01), scalar_i(a02), scalar_i(a03)],
                [scalar_i(0), scalar_i(d1), scalar_i(a12), scalar_i(a13)],
                [scalar_i(0), scalar_i(0), scalar_i(d2), scalar_i(a23)],
                [scalar_i(0), scalar_i(0), scalar_i(0), scalar_i(d3)],
            ])
        })
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(256))]

    #[test]
    fn scalar_owned_and_borrowed_arithmetic_paths_are_equivalent(a in small_exact_scalar(), b in small_exact_scalar()) {
        prop_assert_eq!(&a + &b, a.clone() + b.clone());
        prop_assert_eq!(&a - &b, a.clone() - b.clone());
        prop_assert_eq!(&a * &b, a.clone() * b.clone());

        if b.zero_status() == ZeroStatus::NonZero {
            prop_assert_eq!((&a / &b).unwrap(), (a.clone() / b.clone()).unwrap());
            prop_assert_eq!((a.clone() / &b).unwrap(), (a.clone() / b.clone()).unwrap());
            prop_assert_eq!((&a / b.clone()).unwrap(), (a / b).unwrap());
        }
    }

    #[test]
    fn scalar_fact_invariants_hold_for_generated_mixed_forms(a in small_scalar(), b in small_scalar(), c in small_scalar()) {
        let value = (a.clone() * b.clone()) - (a * b) + c;
        let facts = value.structural_facts();

        prop_assert_eq!(value.structural_facts(), facts);
        prop_assert_eq!(value.zero_status(), facts.zero);
        prop_assert_eq!(value.definitely_zero(), facts.zero == ZeroStatus::Zero);
        if facts.zero == ZeroStatus::Zero {
            prop_assert_eq!(facts.sign, Some(ScalarSign::Zero));
            prop_assert!(facts.magnitude.is_none());
        }
        if facts.zero == ZeroStatus::NonZero {
            prop_assert_ne!(facts.sign, Some(ScalarSign::Zero));
            prop_assert!(facts.magnitude.is_some());
        }
    }

    #[test]
    fn complex_owned_and_borrowed_paths_are_equivalent(ar in small_exact_scalar(), ai in small_exact_scalar(), br in small_exact_scalar(), bi in small_exact_scalar()) {
        let a = Complex::new(ar, ai);
        let b = Complex::new(br, bi);

        prop_assert_eq!(&a + &b, a.clone() + b.clone());
        prop_assert_eq!(&a - &b, a.clone() - b.clone());
        prop_assert_eq!(&a * &b, a.clone() * b.clone());

        if b.norm_squared().zero_status() == ZeroStatus::NonZero {
            prop_assert_eq!((&a / &b).unwrap(), (a / b).unwrap());
        }
    }

    #[test]
    fn vector_dot_is_symmetric_and_distributive_for_exact_inputs(a0 in small_exact_scalar(), a1 in small_exact_scalar(), a2 in small_exact_scalar(), b0 in small_exact_scalar(), b1 in small_exact_scalar(), b2 in small_exact_scalar(), c0 in small_exact_scalar(), c1 in small_exact_scalar(), c2 in small_exact_scalar()) {
        let a = Vector3::new([a0, a1, a2]);
        let b = Vector3::new([b0, b1, b2]);
        let c = Vector3::new([c0, c1, c2]);

        prop_assert_eq!(a.dot(&b), b.dot(&a));
        prop_assert_eq!(a.dot(&(b.clone() + c.clone())), a.dot(&b) + a.dot(&c));
    }

    #[test]
    fn upper_triangular_matrix3_inverse_division_and_power_invariants(matrix in upper_triangular3()) {
        let inverse = matrix.clone().inverse_checked().unwrap();
        let determinant = matrix.determinant();

        prop_assert_eq!(matrix.clone() * inverse.clone(), Matrix3::identity());
        prop_assert_eq!(inverse * matrix.clone(), Matrix3::identity());
        prop_assert_eq!(matrix.transpose().transpose(), matrix.clone());
        prop_assert_eq!(determinant.zero_status(), ZeroStatus::NonZero);
        prop_assert_eq!((matrix.clone() / matrix.clone()).unwrap(), Matrix3::identity());
        prop_assert_eq!(matrix.clone().div_matrix_checked(matrix.clone()).unwrap(), Matrix3::identity());
        prop_assert_eq!(matrix.clone().powi_checked(0).unwrap(), Matrix3::identity());
        prop_assert_eq!(matrix.clone().powi_checked(-1).unwrap(), matrix.inverse_checked().unwrap());
    }

    #[test]
    fn upper_triangular_matrix4_inverse_division_and_power_invariants(matrix in upper_triangular4()) {
        let inverse = matrix.clone().inverse_checked().unwrap();
        let determinant = matrix.determinant();

        prop_assert_eq!(matrix.clone() * inverse.clone(), Matrix4::identity());
        prop_assert_eq!(inverse * matrix.clone(), Matrix4::identity());
        prop_assert_eq!(matrix.transpose().transpose(), matrix.clone());
        prop_assert_eq!(determinant.zero_status(), ZeroStatus::NonZero);
        prop_assert_eq!((matrix.clone() / matrix.clone()).unwrap(), Matrix4::identity());
        prop_assert_eq!(matrix.clone().div_matrix_checked(matrix.clone()).unwrap(), Matrix4::identity());
        prop_assert_eq!(matrix.clone().powi_checked(0).unwrap(), Matrix4::identity());
        prop_assert_eq!(matrix.clone().powi_checked(-1).unwrap(), matrix.inverse_checked().unwrap());
    }

    #[test]
    fn triangular_determinant_product_identity_holds(a in upper_triangular3(), b in upper_triangular3()) {
        prop_assert_eq!(
            (a.clone() * b.clone()).determinant(),
            a.determinant() * b.determinant()
        );
    }

    #[test]
    fn singular_matrix3_duplicate_rows_are_rejected_by_checked_inverse(a in -8_i32..=8, b in -8_i32..=8, c in -8_i32..=8) {
        let singular = Matrix3::new([
            [scalar_i(a), scalar_i(b), scalar_i(c)],
            [scalar_i(a), scalar_i(b), scalar_i(c)],
            [scalar_i(0), scalar_i(0), scalar_i(1)],
        ]);

        prop_assert!(matches!(
            singular.inverse_checked(),
            Err(Problem::DivideByZero | Problem::UnknownZero)
        ));
    }
}
