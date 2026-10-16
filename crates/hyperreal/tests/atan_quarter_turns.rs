use hyperreal::{Rational, Real};

fn q(n: i32, d: i32) -> Real {
    Real::new(Rational::fraction(n.into(), d.try_into().unwrap()).unwrap())
}

fn expected(turns: i32, sin: Real, cos: Real) -> (Real, Real) {
    match turns.rem_euclid(4) {
        0 => (sin, cos),
        1 => (cos, -sin),
        2 => (-sin, -cos),
        _ => (-cos, sin),
    }
}

#[test]
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
