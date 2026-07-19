use hyperlimit::{Point2, Point3, Sign, incircle2d, insphere3d, orient2d, orient3d};
use proptest::prelude::*;
use robust::{Coord, Coord3D};

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

fn robust_sign(value: f64) -> Sign {
    if value > 0.0 {
        Sign::Positive
    } else if value < 0.0 {
        Sign::Negative
    } else {
        Sign::Zero
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
        prop_assert_eq!(orient2d(&a, &b, &c).value(), Some(expected));
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
        prop_assert_eq!(orient3d(&a, &b, &c, &d).value(), Some(expected));
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
        prop_assert_eq!(incircle2d(&a, &b, &c, &d).value(), Some(expected));
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
        prop_assert_eq!(insphere3d(&a, &b, &c, &d, &e).value(), Some(expected));
    }
}
