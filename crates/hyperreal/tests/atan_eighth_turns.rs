use hyperreal::{Rational, Real};

fn q(n: i32, d: i32) -> Real {
    Real::new(Rational::fraction(n.into(), d.try_into().unwrap()).unwrap())
}

fn equal(actual: Real, expected: Real) {
    assert_eq!(
        (actual - expected).exact_rational_normal_form(),
        Some(Rational::zero())
    );
}

#[test]
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
