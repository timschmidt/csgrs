//! Exact-coordinate and cotangent witnesses for half-pi rotations of principal arctangents.
//!
//! Rational Pythagorean triples and signed quadratic arguments supply independent
//! expected coordinates. The optional JSON case checks same-process replay of a
//! parameter map, separately from the construction-only cases.

use hyperreal::{Rational, Real};

/// Construct an exact n/d Real for these fixtures, assuming d is positive.
/// The integer conversions are exact; a nonpositive denominator violates the
/// helper contract and panics through conversion or rational validation.
fn q(n: i32, d: i32) -> Real {
    Real::new(Rational::fraction(n.into(), d.try_into().unwrap()).unwrap())
}

/// Rotate a known (sine,cosine) pair by turns*pi/2 using nonnegative modulo four.
/// This is the fixture's exact quadrant oracle, including negative turn counts;
/// it does not call the trigonometric implementation under test.
fn expected(turns: i32, sin: Real, cos: Real) -> (Real, Real) {
    match turns.rem_euclid(4) {
        0 => (sin, cos),
        1 => (cos, -sin),
        2 => (-sin, -cos),
        _ => (-cos, sin),
    }
}

#[test]
/// Check six signed Pythagorean ratios across nineteen half-pi rotations, giving 114 angles.
/// Compare returned Real sine/cosine values with exact rational coordinates and
/// cotangent with their quotient, for 342 equality assertions. These nonzero
/// coordinates make every tested cotangent defined; this is no pole-error fixture.
fn rational_atan_quarter_turns_retain_exact_coordinates_and_cotangent() {
    for (n, d) in [(3, 4), (-3, 4), (4, 3), (-4, 3), (5, 12), (-5, 12)] {
        let hypotenuse = if d == 12 { 13 } else { 5 };
        for turns in -9..=9 {
            let angle = q(n, d).atan().unwrap() + q(turns, 2) * Real::pi();
            let (sin, cos) = expected(turns, q(n, hypotenuse), q(d, hypotenuse));
            assert_eq!(
                angle.clone().sin(),
                sin,
                "sin: atan({n}/{d}) + {turns}*pi/2"
            );
            assert_eq!(
                angle.clone().cos(),
                cos,
                "cos: atan({n}/{d}) + {turns}*pi/2"
            );
            assert_eq!(angle.cot().unwrap(), (cos / sin).unwrap());
        }
    }
}

#[cfg(feature = "serde")]
#[test]
/// Replay nineteen tau-scaled parameter maps of atan(4/3) through JSON.
/// Require the reimported sine/cosine values to equal independently rotated 4/5
/// and 3/5, giving 38 assertions. Replay remains in one process and does not
/// assert serialization-byte stability or fresh-cache behavior.
fn quarter_turn_parameter_maps_survive_native_scalar_replay() {
    for turns in -9..=9 {
        let source = q(4, 3).atan().unwrap();
        let angle = &source + (q(turns, 4) * Real::tau());
        let json = serde_json::to_string(&angle).unwrap();
        let replay: Real = serde_json::from_str(&json).unwrap();
        let (sin, cos) = expected(turns, q(4, 5), q(3, 5));
        assert_eq!(replay.clone().sin(), sin);
        assert_eq!(replay.cos(), cos);
    }
}

#[test]
/// Check atan(+/-sqrt(2)) across nineteen half-pi rotations for each sign.
/// Use +/-sqrt(6)/3 and sqrt(3)/3 as the independent starting coordinates.
/// Require both returned coordinates and their cotangent quotient to compare
/// equal, for 114 assertions on 38 defined angles; no sampled oracle is used.
fn quadratic_atan_quarter_turns_retain_exact_coordinates_and_cotangent() {
    for sign in [-1, 1] {
        let x = Real::from(sign) * Real::from(2).sqrt().unwrap();
        let sine = (Real::from(sign) * Real::from(6).sqrt().unwrap() / Real::from(3)).unwrap();
        let cosine = (Real::from(3).sqrt().unwrap() / Real::from(3)).unwrap();
        for turns in -9..=9 {
            let angle = x.clone().atan().unwrap() + q(turns, 2) * Real::pi();
            let (sin, cos) = expected(turns, sine.clone(), cosine.clone());
            assert_eq!(angle.clone().sin(), sin);
            assert_eq!(angle.clone().cos(), cos);
            assert_eq!(angle.cot().unwrap(), (cos / sin).unwrap());
        }
    }
}
