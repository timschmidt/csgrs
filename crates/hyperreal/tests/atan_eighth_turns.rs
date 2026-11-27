//! Exact-coordinate witnesses for odd eighth-turn rotations of principal arctangents.
//!
//! Independent Pythagorean/surd formulas test retained relations, reconstructed
//! angle parameters and same-process scalar replay. Recognition declines are
//! kept distinct from proofs that a value is nonalgebraic.

use hyperreal::{Rational, Real};

/// Construct an exact n/d Real for these fixtures, assuming d is positive.
/// The integer conversions are exact; a nonpositive denominator violates the
/// helper contract and panics through conversion or rational validation.
fn q(n: i32, d: i32) -> Real {
    Real::new(Rational::fraction(n.into(), d.try_into().unwrap()).unwrap())
}

/// Require the difference to reduce to an exact rational zero certificate.
/// A failure to find that certificate fails the test; finite-precision proximity
/// or matching sampled coordinates cannot satisfy this helper.
fn equal(actual: Real, expected: Real) {
    assert_eq!(
        (actual - expected).exact_rational_normal_form(),
        Some(Rational::zero())
    );
}

#[test]
/// Check 64 angles from four signed Pythagorean ratios and sixteen odd k in [-15,15].
/// For atan(n/d)+k*pi/4, rotate the independently derived (n+d,d-n)/(h*sqrt(2))
/// coordinates by exact quadrant symmetry. Require both coordinate differences
/// to reduce to rational zero, for 128 exact assertions.
fn rotated_partial_circle_coordinates_retain_an_exact_quadratic_proof() {
    let root_two = Real::from(2).sqrt().unwrap();
    for (n, d, h) in [(4, 3, 5), (-4, 3, 5), (3, 4, 5), (5, 12, 13)] {
        for k in (-15_i32..=15).filter(|k| k % 2 != 0) {
            let theta = q(n, d).atan().unwrap() + q(k, 4) * Real::pi();
            let a = (q(n + d, h) / &root_two).unwrap();
            let b = (q(d - n, h) / &root_two).unwrap();
            let (sin, cos) = match ((k - 1) / 2).rem_euclid(4) {
                0 => (a, b),
                1 => (b, -a),
                2 => (-a, -b),
                _ => (-b, a),
            };
            equal(theta.clone().sin(), sin);
            equal(theta.cos(), cos);
        }
    }
}

#[test]
/// Reconstruct an angle by interpolating one quarter of an exactly retained pi interval.
/// With a=atan(4/3) and b=(a-pi)+tau, the expression a+(b-a)/4 equals a+pi/4.
/// Require sine=7/(5*sqrt(2)) and cosine=-1/(5*sqrt(2)) by exact difference
/// certificates, exercising consumption after source-interval reconstruction.
fn reconstructed_source_intervals_preserve_the_same_rotated_coordinates() {
    let a = q(4, 3).atan().unwrap();
    let b = (&a - Real::pi()) + Real::tau();
    let theta = &a + (b - &a) * q(1, 4);
    equal(
        theta.clone().sin(),
        (Real::from(7) / (Real::from(5) * Real::from(2).sqrt().unwrap())).unwrap(),
    );
    equal(
        theta.cos(),
        (-Real::one() / (Real::from(5) * Real::from(2).sqrt().unwrap())).unwrap(),
    );
}

#[test]
/// Check both signs of sqrt(2) as atan arguments followed by a pi/4 rotation.
/// Require coordinates (1+x)/sqrt(6) and (1-x)/sqrt(6). Separately perturb an
/// atan(4/3)+pi/4 angle by 10^-9 and require both quadratic projections to decline.
/// Those declines test conservative recognition, not nonalgebraicity or a typed
/// domain-error result; all constructed source angles are defined.
fn diagonal_rotations_keep_quadratic_arguments_and_ordinary_domain_results() {
    for sign in [-1, 1] {
        let argument = Real::from(sign) * Real::from(2).sqrt().unwrap();
        let angle = argument.clone().atan().unwrap() + q(1, 4) * Real::pi();
        let denominator = Real::from(6).sqrt().unwrap();
        equal(
            angle.clone().sin(),
            ((Real::one() + &argument) / &denominator).unwrap(),
        );
        equal(
            angle.cos(),
            ((Real::one() - &argument) / denominator).unwrap(),
        );
    }
    let angle = q(4, 3).atan().unwrap() + q(1, 4) * Real::pi() + q(1, 1_000_000_000);
    assert!(angle.clone().sin().exact_quadratic_normal_form().is_none());
    assert!(angle.cos().exact_quadratic_normal_form().is_none());
}

#[cfg(feature = "serde")]
#[test]
/// Replay atan(4/3)+5*pi/4 through JSON and require two exact coordinate certificates.
/// The expected coordinates are -7/(5*sqrt(2)) and 1/(5*sqrt(2)). This is
/// same-process semantic replay; it does not assert byte stability or reset caches.
fn reconstructed_diagonal_rotations_survive_scalar_replay() {
    let angle = q(4, 3).atan().unwrap() + q(5, 4) * Real::pi();
    let text = serde_json::to_string(&angle).unwrap();
    let replay: Real = serde_json::from_str(&text).unwrap();
    let denominator = Real::from(5) * Real::from(2).sqrt().unwrap();
    equal(
        replay.clone().sin(),
        (-Real::from(7) / &denominator).unwrap(),
    );
    equal(replay.cos(), (Real::one() / denominator).unwrap());
}
