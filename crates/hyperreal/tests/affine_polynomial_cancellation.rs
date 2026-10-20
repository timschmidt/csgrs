//! Ring identities behind symbolic affine planes, with independent controls.
use hyperreal::{Rational, Real, RealSign};

fn affine_residual() -> Real {
    let origin = Real::from(6) - Real::from(2).sqrt().unwrap();
    let u = Real::from(6) - Real::from(2) * (Real::pi() / Real::from(8)).unwrap().sin();
    let v = Real::from(6)
        - Real::from(2)
            * ((Real::pi() * Real::from(3)) / Real::from(8))
                .unwrap()
                .sin();
    let nx = Real::from(6) * (&v - &origin);
    let ny = Real::from(-6) * (&u - &origin);
    let offset = -Real::dot2_refs([&nx, &ny], [&origin, &origin]);
    Real::signed_product_sum([true; 3], [[&nx, &u], [&ny, &v], [&offset, &Real::one()]])
}

#[test]
fn affine_plane_identity_has_an_exact_rational_proof() {
    // h(v-o)u-h(u-o)v-[h(v-o)o-h(u-o)o] expands to zero.
    let value = affine_residual();
    assert_eq!(value.exact_rational_normal_form(), Some(Rational::zero()));
    assert_eq!(value.certified_sign_until(0).sign(), Some(RealSign::Zero));
}

#[test]
fn exact_displacements_survive_affine_cancellation() {
    for numerator in [-1_i64, 1] {
        let displacement = Rational::fraction(numerator, 1_000_000_000_000_u64).unwrap();
        let value = affine_residual() + Real::from(displacement.clone());
        assert_eq!(value.exact_rational_normal_form(), Some(displacement));
        assert_eq!(
            value.certified_sign_until(0).sign(),
            Some(if numerator < 0 {
                RealSign::Negative
            } else {
                RealSign::Positive
            }),
        );
    }
}

fn atoms() -> [Real; 3] {
    [
        (Real::pi() / Real::from(8)).unwrap().sin(),
        Real::e().sin(),
        Real::from(2).sqrt().unwrap(),
    ]
}

#[test]
fn independently_rebuilt_atoms_obey_distributivity() {
    let [a, b, c] = atoms();
    let [x, y, z] = atoms();
    // (a+b)(a+c) - a^2 - ab - ac - bc = 0, without trig relations.
    let value = (&a + &b) * (&a + &c) - (&x * &x + &x * &y + &x * &z + &y * &z);
    assert_eq!(value.exact_rational_normal_form(), Some(Rational::zero()));
}

#[test]
fn unit_circle_proofs_preserve_offsets_and_unproved_angle_addition() {
    // Preserve the original e-angle fixture, now with its additional exact
    // unit-circle proof; signed offsets rule out treating near-zero as zero.
    for numerator in [-1, 0, 1] {
        let sine = Real::e().sin();
        let cosine = Real::e().cos();
        let original = &sine * &sine + &cosine * &cosine - Real::one();
        let offset = Rational::fraction(numerator, 1_000_000_000_000_u64).unwrap();
        let value = original + Real::from(offset.clone());
        assert_eq!(value.exact_rational_normal_form(), Some(offset));
        assert_eq!(
            value.certified_sign_until(-512).sign(),
            Some(match numerator {
                -1 => RealSign::Negative,
                0 => RealSign::Zero,
                _ => RealSign::Positive,
            })
        );
    }
    // This separate exact identity still requires an unavailable angle-
    // addition proof. Failure to recognize it cannot certify a zero.
    let value = (Real::e() + Real::one()).sin()
        - Real::e().sin() * Real::one().cos()
        - Real::e().cos() * Real::one().sin();
    assert_eq!(value.exact_rational_normal_form(), None);
    assert_eq!(value.certified_sign_until(-512).sign(), None);
}

#[test]
fn a_proved_identity_survives_precision_changes_and_concurrent_queries() {
    let value = affine_residual();
    let clone = value.clone();
    assert_eq!(value.exact_rational_normal_form(), Some(Rational::zero()));
    for precision in [0, -1100, 0] {
        assert_eq!(
            clone.certified_sign_until(precision).sign(),
            Some(RealSign::Zero)
        );
    }
    std::thread::scope(|scope| {
        for _ in 0..4 {
            scope.spawn(|| {
                assert_eq!(clone.exact_rational_normal_form(), Some(Rational::zero()));
                assert_eq!(clone.certified_sign_until(0).sign(), Some(RealSign::Zero));
            });
        }
    });
}

#[cfg(feature = "serde")]
#[test]
fn a_polynomial_proof_preserves_the_authoritative_serialized_expression() {
    let value = affine_residual();
    let before = serde_json::to_string(&value).unwrap();
    assert_eq!(value.exact_rational_normal_form(), Some(Rational::zero()));
    assert_eq!(serde_json::to_string(&value).unwrap(), before);
    let restored: Real = serde_json::from_str(&before).unwrap();
    assert_eq!(
        restored.exact_rational_normal_form(),
        Some(Rational::zero())
    );
    assert_eq!(
        restored.certified_sign_until(0).sign(),
        Some(RealSign::Zero)
    );
}
