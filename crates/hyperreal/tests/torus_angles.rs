//! Exact principal-angle identities required by repeated native BREP subdivision.
use hyperreal::{Real, ZeroKnowledge};
fn q(n: i32, d: i32) -> Real {
    (Real::from(n) / Real::from(d)).unwrap()
}

#[test]
fn rational_right_triangle_angles_retain_exact_affine_identities() {
    for (opposite, adjacent, hypotenuse) in [
        (3, 4, 5),
        (4, 3, 5),
        (5, 12, 13),
        (8, 15, 17),
        (7, 24, 25),
        (20, 21, 29),
    ] {
        for sign in [-1, 1] {
            let sine = q(sign * opposite, hypotenuse).asin().unwrap();
            let tangent = q(sign * opposite, adjacent).atan().unwrap();
            for coefficient in [Real::one(), q(3, 4), q(-7, 8), q(255, 256)] {
                for shift in [
                    -Real::from(100) * Real::tau(),
                    Real::zero(),
                    Real::from(100) * Real::tau(),
                ] {
                    let left = &coefficient * &sine + &shift;
                    let right = &coefficient * &tangent + shift;
                    assert_eq!(
                        (&left - &right).zero_status(),
                        ZeroKnowledge::Zero,
                        "triangle {opposite}/{adjacent}/{hypotenuse}, sign={sign}"
                    );
                    let gap = q(1, 1_000_000) * q(1, 1_000_000);
                    assert_eq!((&left - &right + gap).zero_status(), ZeroKnowledge::NonZero);
                }
            }
        }
    }
}

#[cfg(feature = "serde")]
#[test]
fn serialized_torus_second_split_phase_replays_without_cached_facts() {
    let [left, right]: [Real; 2] =
        serde_json::from_str(include_str!("fixtures/torus-subdivision-angles.json")).unwrap();
    assert_eq!((left - right).zero_status(), ZeroKnowledge::Zero);
}

#[test]
fn quadratic_tangents_and_wrong_quadrants_keep_their_branch() {
    use hyperreal::{CertifiedRealSign, RealSign};
    for (opposite, hypotenuse) in [(1, 6), (3, 8), (2, 7), (5, 9)] {
        for sign in [-1, 1] {
            let adjacent = Real::from(hypotenuse * hypotenuse - opposite * opposite)
                .sqrt()
                .unwrap();
            let tangent = (Real::from(sign * opposite) / adjacent)
                .unwrap()
                .atan()
                .unwrap();
            let sine = q(sign * opposite, hypotenuse).asin().unwrap();
            assert_eq!((&tangent - &sine).zero_status(), ZeroKnowledge::Zero);
        }
    }
    let half_pi = (Real::pi() / Real::from(2)).unwrap();
    for (a, b, expected) in [
        (3, 4, RealSign::Zero),
        (-3, 4, RealSign::Positive),
        (3, -4, RealSign::Positive),
        (-3, -4, RealSign::Positive),
    ] {
        let angle = q(a, 5).acos().unwrap() + q(b, 5).acos().unwrap() - &half_pi;
        assert!(
            matches!(angle.certified_sign_until(-160),CertifiedRealSign::Known{sign,..} if sign==expected)
        );
    }
    let sine = q(3, 5).asin().unwrap();
    let gap = q(1, 1_000_000) * q(1, 1_000_000);
    for (perturbation, expected) in [
        (gap.clone(), RealSign::Positive),
        (-gap, RealSign::Negative),
    ] {
        let angle = (q(3, 4) + perturbation).atan().unwrap() - &sine;
        assert!(
            matches!(angle.certified_sign_until(-160),CertifiedRealSign::Known{sign,..} if sign==expected)
        );
    }
}
