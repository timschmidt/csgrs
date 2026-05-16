use hyperlimit::{
    Plane3, PlaneSide, Point2, Point3, PredicateOutcome, Sign, classify_point_line,
    classify_point_plane, incircle2d, orient2d, orient2d_batch, orient3d,
};
use proptest::prelude::*;

type Real = hyperreal::Real;

fn real(value: f64) -> Real {
    Real::try_from(value).expect("finite generated scalar")
}

fn p2(x: f64, y: f64) -> Point2 {
    Point2::new(real(x), real(y))
}

fn p3(x: f64, y: f64, z: f64) -> Point3 {
    Point3::new(real(x), real(y), real(z))
}

fn p2i(x: i32, y: i32) -> Point2 {
    Point2::new(Real::from(x), Real::from(y))
}

fn translate2i(point: &Point2, dx: i32, dy: i32) -> Point2 {
    let dx = Real::from(dx);
    let dy = Real::from(dy);
    Point2::new(&point.x + &dx, &point.y + &dy)
}

fn scale2i(point: &Point2, scale: i32) -> Point2 {
    let scale = Real::from(scale);
    Point2::new(&point.x * &scale, &point.y * &scale)
}

fn reflect_x(point: &Point2) -> Point2 {
    Point2::new(-&point.x, point.y.clone())
}

fn small_coord() -> impl Strategy<Value = f64> {
    (-10_000_i32..=10_000).prop_map(|value| value as f64)
}

fn point2() -> impl Strategy<Value = Point2> {
    (small_coord(), small_coord()).prop_map(|(x, y)| p2(x, y))
}

fn point3() -> impl Strategy<Value = Point3> {
    (small_coord(), small_coord(), small_coord()).prop_map(|(x, y, z)| p3(x, y, z))
}

fn value<T: Copy>(outcome: PredicateOutcome<T>) -> Option<T> {
    outcome.value()
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(256))]

    #[test]
    fn orient2d_is_cyclic_and_reverses_on_swap(a in point2(), b in point2(), c in point2()) {
        let abc = value(orient2d(&a, &b, &c));
        let bca = value(orient2d(&b, &c, &a));
        let acb = value(orient2d(&a, &c, &b));

        prop_assert_eq!(abc, bca);
        if let (Some(sign), Some(swapped)) = (abc, acb) {
            prop_assert_eq!(sign.reversed(), swapped);
        }
    }

    #[test]
    fn classify_point_line_matches_orient2d_sign(a in point2(), b in point2(), c in point2()) {
        let orient = value(orient2d(&a, &b, &c));
        let side = value(classify_point_line(&a, &b, &c));

        if let Some(sign) = orient {
            prop_assert_eq!(side, Some(sign.into()));
        }
    }

    #[test]
    fn orient2d_batch_matches_scalar_for_generated_cases(cases in prop::collection::vec((point2(), point2(), point2()), 0..64)) {
        prop_assert_eq!(
            orient2d_batch(&cases),
            cases
                .iter()
                .map(|(a, b, c)| orient2d(a, b, c))
                .collect::<Vec<_>>()
        );
    }

    #[test]
    fn orient3d_reverses_when_two_vertices_swap(a in point3(), b in point3(), c in point3(), d in point3()) {
        let abcd = value(orient3d(&a, &b, &c, &d));
        let bacd = value(orient3d(&b, &a, &c, &d));

        if let (Some(sign), Some(swapped)) = (abcd, bacd) {
            prop_assert_eq!(sign.reversed(), swapped);
        }
    }

    #[test]
    fn axis_aligned_plane_classification_matches_coordinate_difference(x in small_coord(), y in small_coord(), z in small_coord(), plane_z in small_coord()) {
        let point = p3(x, y, z);
        let plane = Plane3::new(p3(0.0, 0.0, 1.0), real(-plane_z));
        let expected = if z > plane_z {
            PlaneSide::Above
        } else if z < plane_z {
            PlaneSide::Below
        } else {
            PlaneSide::On
        };

        prop_assert_eq!(value(classify_point_plane(&point, &plane)), Some(expected));
    }

    #[test]
    fn generated_collinear_points_classify_as_zero_on_oriented_line(x0 in -1000_i32..=1000, y0 in -1000_i32..=1000, dx in -100_i32..=100, dy in -100_i32..=100, t in -100_i32..=100) {
        let a = p2(x0 as f64, y0 as f64);
        let b = p2((x0 + dx) as f64, (y0 + dy) as f64);
        let c = p2((x0 + t * dx) as f64, (y0 + t * dy) as f64);

        if let Some(sign) = value(orient2d(&a, &b, &c)) {
            prop_assert_eq!(sign, Sign::Zero);
        }
    }

    #[test]
    fn generated_coplanar_axis_points_classify_as_zero_on_plane(x in small_coord(), y in small_coord()) {
        let a = p3(0.0, 0.0, 0.0);
        let b = p3(1.0, 0.0, 0.0);
        let c = p3(0.0, 1.0, 0.0);
        let d = p3(x, y, 0.0);

        if let Some(sign) = value(orient3d(&a, &b, &c, &d)) {
            prop_assert_eq!(sign, Sign::Zero);
        }
    }

    #[test]
    fn incircle_unit_circle_axis_cases_track_radius_squared(x in -4_i32..=4, y in -4_i32..=4) {
        let a = p2(1.0, 0.0);
        let b = p2(0.0, 1.0);
        let c = p2(-1.0, 0.0);
        let d = p2(x as f64, y as f64);
        let radius_squared = x * x + y * y;
        let expected = if radius_squared < 1 {
            Sign::Positive
        } else if radius_squared > 1 {
            Sign::Negative
        } else {
            Sign::Zero
        };

        if let Some(sign) = value(incircle2d(&a, &b, &c, &d)) {
            prop_assert_eq!(sign, expected);
        }
    }

    #[test]
    fn orient2d_scaling_and_reflection_follow_determinant_sign(
        ax in -32_i32..32, ay in -32_i32..32,
        bx in -32_i32..32, by in -32_i32..32,
        cx in -32_i32..32, cy in -32_i32..32,
        scale in 1_i32..16,
    ) {
        // Metamorphic predicate tests are a direct fit for Yap's exact
        // computation model: the determinant identity is the oracle, not a
        // sampled approximate coordinate. See Yap, "Towards Exact Geometric
        // Computation," Computational Geometry 7.1-2 (1997). The sign change
        // under reflection is the standard orientation determinant law used by
        // robust predicates; see Shewchuk, "Adaptive Precision Floating-Point
        // Arithmetic and Fast Robust Geometric Predicates," DCG 18.3 (1997).
        let a = p2i(ax, ay);
        let b = p2i(bx, by);
        let c = p2i(cx, cy);

        let scaled = (
            scale2i(&a, scale),
            scale2i(&b, scale),
            scale2i(&c, scale),
        );
        prop_assert_eq!(
            value(orient2d(&a, &b, &c)),
            value(orient2d(&scaled.0, &scaled.1, &scaled.2))
        );

        let reflected = (reflect_x(&a), reflect_x(&b), reflect_x(&c));
        if let (Some(sign), Some(reflected_sign)) = (
            value(orient2d(&a, &b, &c)),
            value(orient2d(&reflected.0, &reflected.1, &reflected.2)),
        ) {
            prop_assert_eq!(sign.reversed(), reflected_sign);
        }
    }

    #[test]
    fn incircle2d_translation_and_positive_scaling_preserve_sign(
        ax in -8_i32..8, ay in -8_i32..8,
        bx in -8_i32..8, by in -8_i32..8,
        cx in -8_i32..8, cy in -8_i32..8,
        dx in -8_i32..8, dy in -8_i32..8,
        tx in -16_i32..16, ty in -16_i32..16,
        scale in 1_i32..8,
    ) {
        let a = p2i(ax, ay);
        let b = p2i(bx, by);
        let c = p2i(cx, cy);
        let d = p2i(dx, dy);
        let moved_scaled = (
            scale2i(&translate2i(&a, tx, ty), scale),
            scale2i(&translate2i(&b, tx, ty), scale),
            scale2i(&translate2i(&c, tx, ty), scale),
            scale2i(&translate2i(&d, tx, ty), scale),
        );

        prop_assert_eq!(
            value(incircle2d(&a, &b, &c, &d)),
            value(incircle2d(
                &moved_scaled.0,
                &moved_scaled.1,
                &moved_scaled.2,
                &moved_scaled.3,
            ))
        );
    }
}
