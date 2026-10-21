//! Strict predicates over sampled-sphere coordinates.
//!
//! Sampled circles and spheres place vertices at sines and cosines of
//! rational multiples of pi. Their exact coplanarities and collinearities
//! are zeros in a cyclotomic field. These tests reproduce the predicates that
//! stopped strict convex hulls and Booleans of the README sphere.

use hyperlimit::{
    Certainty, LineSide, Point2, Point3, PredicateOutcome, PredicatePolicy, Sign,
    classify_point_line_with_orientation, line2_orientation, orient2, orient3,
};
use hyperreal::{Rational, Real};

fn q(numerator: i64, denominator: u64) -> Real {
    Real::new(Rational::fraction(numerator, denominator).unwrap())
}

// csgrs sphere vertex: longitude 2*pi*i/segments, colatitude pi*j/stacks,
// scaled by `radius` and translated by `shift`.
fn vertex(segments: u64, stacks: u64, i: i64, j: i64, radius: &Real, shift: &[Real; 3]) -> Point3 {
    let longitude = Real::tau() * q(i, segments);
    let colatitude = Real::pi() * q(j, stacks);
    let (lon_sin, lon_cos) = (longitude.clone().sin(), longitude.cos());
    let (lat_sin, lat_cos) = (colatitude.clone().sin(), colatitude.cos());
    Point3::new(
        radius * &lon_cos * &lat_sin + &shift[0],
        radius * &lat_cos + &shift[1],
        radius * &lon_sin * &lat_sin + &shift[2],
    )
}

// Where the line through `a` and `b` meets z = 0, projected to (x, y).
fn crossing(a: &Point3, b: &Point3) -> Point2 {
    let t = (-a.z.clone() / (&b.z - &a.z)).unwrap();
    Point2::new(&a.x + &t * (&b.x - &a.x), &a.y + &t * (&b.y - &a.y))
}

fn assert_exact<T: PartialEq + std::fmt::Debug>(outcome: PredicateOutcome<T>, expected: T) {
    match outcome {
        PredicateOutcome::Decided {
            value, certainty, ..
        } => {
            assert_eq!(value, expected);
            assert_eq!(certainty, Certainty::Exact);
        }
        PredicateOutcome::Unknown { .. } => panic!("strict predicate must decide"),
    }
}

#[test]
fn sampled_sphere_coincident_planes_are_certified_zeros() {
    let unit = Real::one();
    let origin = [Real::zero(), Real::zero(), Real::zero()];
    let pole = Point3::new(Real::zero(), -Real::one(), Real::zero());
    // Not a sampled quad: cot(pi/16) = tan(7 pi/16) puts these four on one
    // plane through the south pole of the 32 x 16 sphere.
    let a = vertex(32, 16, 5, 14, &unit, &origin);
    let b = vertex(32, 16, 13, 8, &unit, &origin);
    let c = vertex(32, 16, 11, 8, &unit, &origin);
    assert_exact(
        orient3(&pole, &a, &b, &c, PredicatePolicy::STRICT),
        Sign::Zero,
    );
    let off = vertex(32, 16, 12, 8, &unit, &origin);
    let PredicateOutcome::Decided { value, .. } =
        orient3(&pole, &a, &b, &off, PredicatePolicy::STRICT)
    else {
        panic!("a nonzero orientation must decide");
    };
    assert_ne!(value, Sign::Zero);
}

#[test]
fn plane_crossings_of_one_sampled_quad_are_certified_collinear() {
    // The README fixture: sphere(1.35, segments, stacks) translated by
    // (0.55, 0.25, 0.2), cut by the cube face z = 0. Every crossing of a
    // quad's edge lines with that face lies on one line.
    let radius = Real::try_from(1.35).unwrap();
    let shift = [
        Real::try_from(0.55).unwrap(),
        Real::try_from(0.25).unwrap(),
        Real::try_from(0.2).unwrap(),
    ];
    for (segments, stacks, i, j) in [(32, 16, 20, 7), (17, 9, 11, 3)] {
        let p = [
            vertex(segments, stacks, i, j, &radius, &shift),
            vertex(segments, stacks, i + 1, j, &radius, &shift),
            vertex(segments, stacks, i + 1, j + 1, &radius, &shift),
            vertex(segments, stacks, i, j + 1, &radius, &shift),
        ];
        let first = crossing(&p[0], &p[3]);
        let diagonal = crossing(&p[0], &p[2]);
        let last = crossing(&p[1], &p[2]);
        assert_exact(
            orient2(&first, &diagonal, &last, PredicatePolicy::STRICT),
            Sign::Zero,
        );
        // Hypermesh classifies against a retained line orientation.
        let line = line2_orientation(&first, &diagonal);
        assert_exact(
            classify_point_line_with_orientation(
                &first,
                &diagonal,
                &last,
                &line,
                PredicatePolicy::STRICT,
            ),
            LineSide::On,
        );
        // A crossing from the next quad is off that line.
        let next = vertex(segments, stacks, i + 2, j, &radius, &shift);
        let off = crossing(&next, &p[2]);
        let PredicateOutcome::Decided { value, .. } =
            orient2(&first, &diagonal, &off, PredicatePolicy::STRICT)
        else {
            panic!("a nonzero orientation must decide");
        };
        assert_ne!(value, Sign::Zero);
    }
}
