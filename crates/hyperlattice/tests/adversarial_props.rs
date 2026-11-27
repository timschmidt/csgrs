//! Generated ownership, scalar-fact, vector and triangular-matrix invariants.
//!
//! Each property requests 256 Proptest cases. Rational-only generators support exact
//! structural comparisons; a separate generator adds π, e and sqrt(2) for fact checks.
//! Matrix generators enforce nonzero integer diagonals and upper-triangular shape.
//! The suite's conditional errors and composition identities are scoped to those
//! strategies, not all Real representations or arbitrary matrix support.

/// Shared exact rational, scalar-status and abort fixtures used by this integration target.
mod common;

use common::frac;
use hyperlattice::{Complex, Matrix3, Matrix4, Problem, Real, RealSign, Vector3, ZeroKnowledge};
use proptest::prelude::*;

/// Imports a generated i32 exactly into Real for rational fixtures and constants.
fn scalar_i(value: i32) -> Real {
    value.into()
}

/// Generates integers in inclusive `[-16,16]`, rejecting zero for invertible matrix diagonals.
fn nonzero_i32() -> impl Strategy<Value = i32> {
    (-16_i32..=16).prop_filter("nonzero", |value| *value != 0)
}

/// Chooses a bounded integer, bounded rational, π, e or sqrt(2) for scalar-fact properties.
///
/// Integers range over `[-64,64]`; rational numerators use that range and denominators
/// use `[1,64]`. This finite symbolic vocabulary does not cover arbitrary Real forms.
fn small_scalar() -> impl Strategy<Value = Real> {
    prop_oneof![
        (-64_i32..=64).prop_map(scalar_i),
        (-64_i64..=64, 1_u64..=64).prop_map(|(n, d)| frac(n, d)),
        Just(Real::pi()),
        Just(Real::e()),
        Just(Real::sqrt(scalar_i(2)).unwrap()),
    ]
}

/// Chooses integers in `[-64,64]` or rationals with numerator in `[-64,64]` and denominator in `[1,64]`.
///
/// Both branches are rational. Here exact names the rational-only strategy; the
/// symbolic values in small_scalar are also retained as exact definitions.
fn small_exact_scalar() -> impl Strategy<Value = Real> {
    prop_oneof![
        (-64_i32..=64).prop_map(scalar_i),
        (-64_i64..=64, 1_u64..=64).prop_map(|(n, d)| frac(n, d)),
    ]
}

/// Generates invertible upper-triangular 3×3 integer matrices.
///
/// Diagonal entries are nonzero in `[-16,16]`; strict upper entries range over `[-8,8]`.
/// The lower triangle is exactly zero, and the diagonal product is nonzero.
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

/// Generates invertible upper-triangular 4×4 integer matrices.
///
/// Uses nonzero diagonals in `[-16,16]`, strict upper entries in `[-4,4]` and exact
/// zeros below the diagonal; negative diagonals remain valid.
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

    /// Compares scalar ownership forms on generated rational pairs, conditioning division on known nonzero status.
    #[test]
    fn scalar_owned_and_borrowed_arithmetic_paths_are_equivalent(a in small_exact_scalar(), b in small_exact_scalar()) {
        prop_assert_eq!(&a + &b, a.clone() + b.clone());
        prop_assert_eq!(&a - &b, a.clone() - b.clone());
        prop_assert_eq!(&a * &b, a.clone() * b.clone());

        if b.zero_status() == ZeroKnowledge::NonZero {
            prop_assert_eq!((&a / &b).unwrap(), (a.clone() / b.clone()).unwrap());
            prop_assert_eq!((a.clone() / &b).unwrap(), (a.clone() / b.clone()).unwrap());
            prop_assert_eq!((&a / b.clone()).unwrap(), (a / b).unwrap());
        }
    }

    /// Checks repeated fact queries and zero/sign/magnitude consistency for (a*b)-(a*b)+c.
    ///
    /// Uses the mixed rational/symbolic strategy. The assertions do not independently
    /// require the resulting representation or facts to equal those of c.
    #[test]
    fn scalar_fact_invariants_hold_for_generated_mixed_forms(a in small_scalar(), b in small_scalar(), c in small_scalar()) {
        let value = (a.clone() * b.clone()) - (a * b) + c;
        let facts = value.structural_facts();

        prop_assert_eq!(value.structural_facts(), facts);
        prop_assert_eq!(value.zero_status(), facts.zero);
        prop_assert_eq!(value.definitely_zero(), facts.zero == ZeroKnowledge::Zero);
        if facts.zero == ZeroKnowledge::Zero {
            prop_assert_eq!(facts.sign, Some(RealSign::Zero));
            prop_assert!(facts.magnitude.is_none());
        }
        if facts.zero == ZeroKnowledge::NonZero {
            prop_assert_ne!(facts.sign, Some(RealSign::Zero));
            prop_assert!(facts.magnitude.is_some());
        }
    }

    /// Compares complex ownership forms with generated rational components.
    ///
    /// Division is asserted only when the right squared norm is known nonzero.
    #[test]
    fn complex_owned_and_borrowed_paths_are_equivalent(ar in small_exact_scalar(), ai in small_exact_scalar(), br in small_exact_scalar(), bi in small_exact_scalar()) {
        let a = Complex::new(ar, ai);
        let b = Complex::new(br, bi);

        prop_assert_eq!(&a + &b, a.clone() + b.clone());
        prop_assert_eq!(&a - &b, a.clone() - b.clone());
        prop_assert_eq!(&a * &b, a.clone() * b.clone());

        if b.norm_squared().zero_status() == ZeroKnowledge::NonZero {
            prop_assert_eq!((&a / &b).unwrap(), (a / b).unwrap());
        }
    }

    /// Checks symmetry and distributivity of dot products for three generated rational Vector3 values.
    #[test]
    fn vector_dot_is_symmetric_and_distributive_for_exact_inputs(a0 in small_exact_scalar(), a1 in small_exact_scalar(), a2 in small_exact_scalar(), b0 in small_exact_scalar(), b1 in small_exact_scalar(), b2 in small_exact_scalar(), c0 in small_exact_scalar(), c1 in small_exact_scalar(), c2 in small_exact_scalar()) {
        let a = Vector3::new([a0, a1, a2]);
        let b = Vector3::new([b0, b1, b2]);
        let c = Vector3::new([c0, c1, c2]);

        prop_assert_eq!(a.dot(&b), b.dot(&a));
        prop_assert_eq!(a.dot(&(b.clone() + c.clone())), a.dot(&b) + a.dot(&c));
    }

    /// Checks two-sided inverse, transpose, determinant nonzeroness, self-division and powers on generated 3×3 triangles.
    ///
    /// The generator ensures invertibility; inverse and power comparisons share matrix
    /// implementation paths rather than an independently computed inverse.
    #[test]
    fn upper_triangular_matrix3_inverse_division_and_power_invariants(matrix in upper_triangular3()) {
        let inverse = matrix.clone().inverse_checked().unwrap();
        let determinant = matrix.determinant();

        prop_assert_eq!(matrix.clone() * inverse.clone(), Matrix3::identity());
        prop_assert_eq!(inverse * matrix.clone(), Matrix3::identity());
        prop_assert_eq!(matrix.transpose().transpose(), matrix.clone());
        prop_assert_eq!(determinant.zero_status(), ZeroKnowledge::NonZero);
        prop_assert_eq!((matrix.clone() / matrix.clone()).unwrap(), Matrix3::identity());
        prop_assert_eq!(matrix.clone().div_matrix_checked(matrix.clone()).unwrap(), Matrix3::identity());
        prop_assert_eq!(matrix.clone().powi_checked(0).unwrap(), Matrix3::identity());
        prop_assert_eq!(matrix.clone().powi_checked(-1).unwrap(), matrix.inverse_checked().unwrap());
    }

    /// Checks the corresponding checked inverse/division/power relations for generated 4×4 upper triangles.
    ///
    /// Requires both inverse products to equal identity and the determinant to be known nonzero.
    #[test]
    fn upper_triangular_matrix4_inverse_division_and_power_invariants(matrix in upper_triangular4()) {
        let inverse = matrix.clone().inverse_checked().unwrap();
        let determinant = matrix.determinant();

        prop_assert_eq!(matrix.clone() * inverse.clone(), Matrix4::identity());
        prop_assert_eq!(inverse * matrix.clone(), Matrix4::identity());
        prop_assert_eq!(matrix.transpose().transpose(), matrix.clone());
        prop_assert_eq!(determinant.zero_status(), ZeroKnowledge::NonZero);
        prop_assert_eq!((matrix.clone() / matrix.clone()).unwrap(), Matrix4::identity());
        prop_assert_eq!(matrix.clone().div_matrix_checked(matrix.clone()).unwrap(), Matrix4::identity());
        prop_assert_eq!(matrix.clone().powi_checked(0).unwrap(), Matrix4::identity());
        prop_assert_eq!(matrix.clone().powi_checked(-1).unwrap(), matrix.inverse_checked().unwrap());
    }

    /// Checks multiplicativity of the determinant on two generated invertible 3×3 upper triangles.
    #[test]
    fn triangular_determinant_product_identity_holds(a in upper_triangular3(), b in upper_triangular3()) {
        prop_assert_eq!(
            (a.clone() * b.clone()).determinant(),
            a.determinant() * b.determinant()
        );
    }

    /// Generates equal first two rows in `[-8,8]` and accepts DivideByZero or UnknownZero from checked inversion.
    ///
    /// This demands rejection, not a definite-zero classification in every generated case.
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
