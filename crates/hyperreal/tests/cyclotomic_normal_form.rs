//! Exact zeros among trigonometric values at rational multiples of pi.
//!
//! Each identity below holds in a cyclotomic field but not for opaque atoms:
//! its arguments differ, so structural matching and `sin^2 + cos^2 = 1` for
//! equal arguments do not apply.

use hyperreal::{Rational, Real, ZeroKnowledge};

fn q(numerator: i64, denominator: u64) -> Real {
    Real::new(Rational::fraction(numerator, denominator).unwrap())
}

fn turn(numerator: i64, denominator: u64) -> Real {
    Real::pi() * q(numerator, denominator)
}

fn sin(numerator: i64, denominator: u64) -> Real {
    turn(numerator, denominator).sin()
}

fn cos(numerator: i64, denominator: u64) -> Real {
    turn(numerator, denominator).cos()
}

fn assert_zero(value: Real) {
    assert_eq!(value.exact_rational_normal_form(), Some(Rational::zero()));
    assert_eq!(value.zero_status(), ZeroKnowledge::Zero);
}

fn assert_unproved(value: Real) {
    assert_eq!(value.exact_rational_normal_form(), None);
}

// Unit-sphere vertex at longitude 2*pi*i/segments and colatitude pi*j/stacks,
// built exactly as csgrs samples spheres.
fn sphere_vertex(segments: u64, stacks: u64, i: i64, j: i64) -> [Real; 3] {
    let longitude = Real::tau() * q(i, segments);
    let colatitude = turn(j, stacks);
    let (lon_sin, lon_cos) = (longitude.clone().sin(), longitude.cos());
    let (lat_sin, lat_cos) = (colatitude.clone().sin(), colatitude.cos());
    [&lon_cos * &lat_sin, lat_cos, &lon_sin * &lat_sin]
}

fn orient3(points: [&[Real; 3]; 4]) -> Real {
    let [a, b, c, d] = points;
    let difference = |p: &[Real; 3], k: usize| &p[k] - &a[k];
    let (b0, b1, b2) = (difference(b, 0), difference(b, 1), difference(b, 2));
    let (c0, c1, c2) = (difference(c, 0), difference(c, 1), difference(c, 2));
    let (d0, d1, d2) = (difference(d, 0), difference(d, 1), difference(d, 2));
    &b0 * (&c1 * &d2 - &c2 * &d1) - &b1 * (&c0 * &d2 - &c2 * &d0) + &b2 * (&c0 * &d1 - &c1 * &d0)
}

#[test]
fn complementary_and_multiple_angle_identities_are_exact_zeros() {
    // cos(pi/5) is stored as sin(3 pi/10): the Pythagorean pair has
    // different arguments.
    assert_zero(&sin(1, 5) * &sin(1, 5) + &cos(1, 5) * &cos(1, 5) - Real::one());
    assert_zero(sin(2, 7) - Real::from(2) * sin(1, 7) * cos(1, 7));
    assert_zero(cos(2, 9) - (Real::from(2) * cos(1, 9) * cos(1, 9) - Real::one()));
    // sin(3x) = 3 sin(x) - 4 sin(x)^3 at x = pi/11.
    let s = sin(1, 11);
    assert_zero(sin(3, 11) - (Real::from(3) * &s - Real::from(4) * &s * &s * &s));
}

#[test]
fn heptagon_cosine_relation_is_an_exact_zero() {
    // cos(pi/7) - cos(2 pi/7) + cos(3 pi/7) = 1/2 has no angle-sum structure.
    assert_zero(cos(1, 7) - cos(2, 7) + cos(3, 7) - q(1, 2));
}

#[test]
fn closed_form_radicals_meet_symbolic_trigonometry() {
    // sin(pi/12) has a closed form in sqrt(6) and sqrt(2); the angle
    // difference uses sqrt(3) and sqrt(2) separately.
    assert_zero(sin(1, 12) - (sin(1, 3) * cos(1, 4) - cos(1, 3) * sin(1, 4)));
    // sin(pi/8)^2 = (2 - sqrt(2))/4 relates a symbolic atom to a radical.
    let two = Real::from(2);
    let root_two = two.clone().sqrt().unwrap();
    assert_zero(&sin(1, 8) * &sin(1, 8) - ((two - root_two) / Real::from(4)).unwrap());
}

#[test]
fn proved_nonzero_reciprocals_cancel() {
    let tangent = (sin(1, 9) / cos(1, 9)).unwrap();
    assert_zero(tangent * cos(1, 9) - sin(1, 9));
    let secant = (Real::one() / cos(2, 13)).unwrap();
    assert_zero(&secant * &secant * (&cos(2, 13) * &cos(2, 13)) - Real::one());
    // cos(pi/5) = (1 + sqrt(5))/4 satisfies 4c^2 - 2c - 1 = 0, so
    // 1/cos(pi/5) = 4 cos(pi/5) - 2. Only the field relation proves it.
    let secant = (Real::one() / cos(1, 5)).unwrap();
    assert_zero(secant - (Real::from(4) * cos(1, 5) - Real::from(2)));
}

#[test]
fn sampled_sphere_quads_and_stereographic_coincidences_are_coplanar() {
    // A latitude quad is an isosceles trapezoid.
    for (segments, stacks, i, j) in [(32, 16, 3, 5), (17, 9, 4, 2), (17, 9, 16, 7)] {
        let points = [
            sphere_vertex(segments, stacks, i, j),
            sphere_vertex(segments, stacks, i + 1, j),
            sphere_vertex(segments, stacks, i + 1, j + 1),
            sphere_vertex(segments, stacks, i, j + 1),
        ];
        assert_zero(orient3([&points[0], &points[1], &points[2], &points[3]]));
    }
    // The 32 x 16 south pole, P(5, 14), P(13, 8) and P(11, 8) are coplanar
    // because cot(pi/16) = tan(7 pi/16): stereographic projection from the
    // pole maps them onto one line. This coincidence stopped strict hulls.
    let pole = [Real::zero(), -Real::one(), Real::zero()];
    let points = [
        sphere_vertex(32, 16, 5, 14),
        sphere_vertex(32, 16, 13, 8),
        sphere_vertex(32, 16, 11, 8),
    ];
    assert_zero(orient3([&pole, &points[0], &points[1], &points[2]]));
    // The same configuration with a neighbouring point is not coplanar.
    let moved = sphere_vertex(32, 16, 12, 8);
    assert_unproved(orient3([&pole, &points[0], &points[1], &moved]));
}

#[test]
fn rational_affine_images_keep_coplanarity() {
    let scale = q(27, 20);
    let shift = [q(11, 20), q(1, 4), q(1, 5)];
    let map =
        |point: [Real; 3]| -> [Real; 3] { std::array::from_fn(|k| &scale * &point[k] + &shift[k]) };
    let points = [
        map(sphere_vertex(32, 16, 7, 3)),
        map(sphere_vertex(32, 16, 8, 3)),
        map(sphere_vertex(32, 16, 8, 4)),
        map(sphere_vertex(32, 16, 7, 4)),
    ];
    assert_zero(orient3([&points[0], &points[1], &points[2], &points[3]]));
}

#[test]
fn nonzero_and_unsupported_values_remain_unproved() {
    // Distinct values, a zero perturbed below every refinement limit, and a
    // zero outside the cyclotomic domain (the sine addition identity at e).
    assert_unproved(sin(1, 5) - sin(1, 7));
    let tiny = (0..12).fold(Real::one(), |power, _| power * q(1, 1 << 62));
    assert_unproved(cos(1, 7) - cos(2, 7) + cos(3, 7) - q(1, 2) + tiny);
    let e = Real::e();
    assert_unproved(
        (e.clone() + Real::one()).sin()
            - e.clone().sin() * Real::one().cos()
            - e.cos() * Real::one().sin(),
    );
}

#[test]
fn field_order_limit_declines_without_claiming() {
    // Q(zeta_20014) exceeds the admitted field order; the identity is true
    // but stays unproved rather than exhausting an unbounded computation.
    assert_unproved(sin(2, 10007) - Real::from(2) * sin(1, 10007) * cos(1, 10007));
}

#[test]
fn proofs_are_stable_across_cold_and_warm_queries() {
    let value = cos(1, 7) - cos(2, 7) + cos(3, 7) - q(1, 2);
    let copy = value.clone();
    assert_zero(value);
    assert_zero(copy);
}
