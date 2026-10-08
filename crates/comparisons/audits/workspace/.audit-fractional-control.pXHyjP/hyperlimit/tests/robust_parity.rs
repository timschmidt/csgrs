use hyperlimit::{Point2, Point3, Sign, incircle2, insphere3, orient2, orient3};
use proptest::prelude::*;
use robust::{Coord, Coord3D};

const APPROX: hyperlimit::PredicatePolicy = hyperlimit::PredicatePolicy::APPROXIMATE_512;

fn point2(values: &[f64]) -> Point2 {
    Point2::try_from_f64_array([values[0], values[1]]).unwrap()
}

fn point3(values: &[f64]) -> Point3 {
    Point3::try_from_f64_array([values[0], values[1], values[2]]).unwrap()
}

fn coord2(values: &[f64]) -> Coord<f64> {
    Coord {
        x: values[0],
        y: values[1],
    }
}

fn coord3(values: &[f64]) -> Coord3D<f64> {
    Coord3D {
        x: values[0],
        y: values[1],
        z: values[2],
    }
}

fn array2(values: &[f64]) -> [f64; 2] {
    [values[0], values[1]]
}

fn array3(values: &[f64]) -> [f64; 3] {
    [values[0], values[1], values[2]]
}

fn apfp_coord2(values: &[f64]) -> apfp::geometry::f64::Coord {
    apfp::geometry::f64::Coord::new(values[0], values[1])
}

fn robust_sign(value: f64) -> Sign {
    if value > 0.0 {
        Sign::Positive
    } else if value < 0.0 {
        Sign::Negative
    } else {
        Sign::Zero
    }
}

fn apfp_sign(value: apfp::geometry::Orientation) -> Sign {
    match value {
        apfp::geometry::Orientation::Clockwise => Sign::Negative,
        apfp::geometry::Orientation::CoLinear => Sign::Zero,
        apfp::geometry::Orientation::CounterClockwise => Sign::Positive,
    }
}

#[test]
fn adaptive_competitors_agree_on_boundaries_and_near_degeneracies() {
    let orient2_cases = [
        ([0.0, 0.0], [1.0, 1.0], [2.0, 2.0]),
        ([-1.0, -1.0], [1.0, 1.0], [0.25, 0.25 + 1.0e-15]),
        (
            [1.0e100, 1.0e100],
            [1.0e100 + 1.0e85, 1.0e100],
            [1.0e100, 1.0e100 + 1.0e85],
        ),
    ];
    for (a, b, c) in orient2_cases {
        let expected = robust_sign(robust::orient2d(coord2(&a), coord2(&b), coord2(&c)));
        assert_eq!(
            robust_sign(geometry_predicates::orient2d(a, b, c)),
            expected,
        );
        assert_eq!(
            apfp_sign(apfp::geometry::f64::orient2d(
                &apfp_coord2(&a),
                &apfp_coord2(&b),
                &apfp_coord2(&c),
            )),
            expected,
        );
        assert_eq!(
            orient2(&point2(&a), &point2(&b), &point2(&c), APPROX).value(),
            Some(expected),
        );
    }

    let incircle_cases = [
        ([1.0, 0.0], [0.0, 1.0], [-1.0, 0.0], [0.0, -1.0]),
        ([1.0, 0.0], [0.0, 1.0], [-1.0, 0.0], [0.0, -1.0 + 1.0e-15]),
        ([1.0, 0.0], [0.0, 1.0], [-1.0, 0.0], [0.0, -1.0 - 1.0e-15]),
    ];
    for (a, b, c, d) in incircle_cases {
        let expected = robust_sign(robust::incircle(
            coord2(&a),
            coord2(&b),
            coord2(&c),
            coord2(&d),
        ));
        assert_eq!(
            robust_sign(geometry_predicates::incircle(a, b, c, d)),
            expected,
        );
        assert_eq!(
            apfp_sign(apfp::geometry::f64::incircle(
                &apfp_coord2(&a),
                &apfp_coord2(&b),
                &apfp_coord2(&c),
                &apfp_coord2(&d),
            )),
            expected,
        );
        assert_eq!(
            incircle2(&point2(&a), &point2(&b), &point2(&c), &point2(&d), APPROX,).value(),
            Some(expected),
        );
    }

    let orient3_cases = [
        (
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.25, 0.25, 0.0],
        ),
        (
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.25, 0.25, 1.0e-15],
        ),
    ];
    for (a, b, c, d) in orient3_cases {
        let expected = robust_sign(robust::orient3d(
            coord3(&a),
            coord3(&b),
            coord3(&c),
            coord3(&d),
        ));
        assert_eq!(
            robust_sign(geometry_predicates::orient3d(a, b, c, d)),
            expected,
        );
        assert_eq!(
            orient3(&point3(&a), &point3(&b), &point3(&c), &point3(&d), APPROX,).value(),
            Some(expected),
        );
    }

    let sphere = (
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
        [-1.0, 0.0, 0.0],
    );
    for e in [[0.0, -1.0, 0.0], [0.0, -1.0 + 1.0e-15, 0.0]] {
        let (a, b, c, d) = sphere;
        let expected = robust_sign(robust::insphere(
            coord3(&a),
            coord3(&b),
            coord3(&c),
            coord3(&d),
            coord3(&e),
        ));
        assert_eq!(
            robust_sign(geometry_predicates::insphere(a, b, c, d, e)),
            expected,
        );
        assert_eq!(
            insphere3(
                &point3(&a),
                &point3(&b),
                &point3(&c),
                &point3(&d),
                &point3(&e),
                APPROX,
            )
            .value(),
            Some(expected),
        );
    }
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(512))]

    #[test]
    fn orient2d_matches_shewchuk_adaptive_predicate(
        coordinates in prop::collection::vec(-1.0e6_f64..1.0e6, 6)
    ) {
        let a = point2(&coordinates[0..2]);
        let b = point2(&coordinates[2..4]);
        let c = point2(&coordinates[4..6]);
        let expected = robust_sign(robust::orient2d(
            coord2(&coordinates[0..2]),
            coord2(&coordinates[2..4]),
            coord2(&coordinates[4..6]),
        ));
        prop_assert_eq!(
            robust_sign(geometry_predicates::orient2d(
                array2(&coordinates[0..2]),
                array2(&coordinates[2..4]),
                array2(&coordinates[4..6]),
            )),
            expected,
        );
        prop_assert_eq!(
            apfp_sign(apfp::geometry::f64::orient2d(
                &apfp_coord2(&coordinates[0..2]),
                &apfp_coord2(&coordinates[2..4]),
                &apfp_coord2(&coordinates[4..6]),
            )),
            expected,
        );
        prop_assert_eq!(orient2(&a, &b, &c, APPROX).value(), Some(expected));
    }

    #[test]
    fn orient3d_matches_shewchuk_adaptive_predicate(
        coordinates in prop::collection::vec(-1.0e4_f64..1.0e4, 12)
    ) {
        let a = point3(&coordinates[0..3]);
        let b = point3(&coordinates[3..6]);
        let c = point3(&coordinates[6..9]);
        let d = point3(&coordinates[9..12]);
        let expected = robust_sign(robust::orient3d(
            coord3(&coordinates[0..3]),
            coord3(&coordinates[3..6]),
            coord3(&coordinates[6..9]),
            coord3(&coordinates[9..12]),
        ));
        prop_assert_eq!(
            robust_sign(geometry_predicates::orient3d(
                array3(&coordinates[0..3]),
                array3(&coordinates[3..6]),
                array3(&coordinates[6..9]),
                array3(&coordinates[9..12]),
            )),
            expected,
        );
        prop_assert_eq!(orient3(&a, &b, &c, &d, APPROX).value(), Some(expected));
    }

    #[test]
    fn incircle2d_matches_shewchuk_adaptive_predicate(
        coordinates in prop::collection::vec(-1.0e3_f64..1.0e3, 8)
    ) {
        let a = point2(&coordinates[0..2]);
        let b = point2(&coordinates[2..4]);
        let c = point2(&coordinates[4..6]);
        let d = point2(&coordinates[6..8]);
        let expected = robust_sign(robust::incircle(
            coord2(&coordinates[0..2]),
            coord2(&coordinates[2..4]),
            coord2(&coordinates[4..6]),
            coord2(&coordinates[6..8]),
        ));
        prop_assert_eq!(
            robust_sign(geometry_predicates::incircle(
                array2(&coordinates[0..2]),
                array2(&coordinates[2..4]),
                array2(&coordinates[4..6]),
                array2(&coordinates[6..8]),
            )),
            expected,
        );
        prop_assert_eq!(
            apfp_sign(apfp::geometry::f64::incircle(
                &apfp_coord2(&coordinates[0..2]),
                &apfp_coord2(&coordinates[2..4]),
                &apfp_coord2(&coordinates[4..6]),
                &apfp_coord2(&coordinates[6..8]),
            )),
            expected,
        );
        prop_assert_eq!(incircle2(&a, &b, &c, &d, APPROX).value(), Some(expected));
    }

    #[test]
    fn insphere3d_matches_shewchuk_adaptive_predicate(
        coordinates in prop::collection::vec(-100.0_f64..100.0, 15)
    ) {
        let a = point3(&coordinates[0..3]);
        let b = point3(&coordinates[3..6]);
        let c = point3(&coordinates[6..9]);
        let d = point3(&coordinates[9..12]);
        let e = point3(&coordinates[12..15]);
        let expected = robust_sign(robust::insphere(
            coord3(&coordinates[0..3]),
            coord3(&coordinates[3..6]),
            coord3(&coordinates[6..9]),
            coord3(&coordinates[9..12]),
            coord3(&coordinates[12..15]),
        ));
        prop_assert_eq!(
            robust_sign(geometry_predicates::insphere(
                array3(&coordinates[0..3]),
                array3(&coordinates[3..6]),
                array3(&coordinates[6..9]),
                array3(&coordinates[9..12]),
                array3(&coordinates[12..15]),
            )),
            expected,
        );
        prop_assert_eq!(insphere3(&a, &b, &c, &d, &e, APPROX).value(), Some(expected));
    }
}
