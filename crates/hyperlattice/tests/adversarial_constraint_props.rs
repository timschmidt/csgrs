use proptest::prelude::*;
use realistic_blas::{Scalar, Vector3, ZeroStatus};

fn s(value: i32) -> Scalar {
    value.into()
}

fn point3() -> impl Strategy<Value = Vector3> {
    (-128_i32..=128, -128_i32..=128, -128_i32..=128)
        .prop_map(|(x, y, z)| Vector3::new([s(x), s(y), s(z)]))
}

fn nonzero_direction3() -> impl Strategy<Value = Vector3> {
    point3().prop_filter("nonzero direction", |value| {
        value.dot(value).zero_status() == ZeroStatus::NonZero
    })
}

fn squared_distance(a: &Vector3, b: &Vector3) -> Scalar {
    let delta = a - b;
    delta.dot(&delta)
}

fn midpoint_residual(a: &Vector3, b: &Vector3, midpoint: &Vector3) -> Vector3 {
    midpoint.clone() * s(2) - (a + b)
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(256))]

    #[test]
    fn distance_equality_residual_is_exact_zero_for_identical_points(point in point3()) {
        let residual = squared_distance(&point, &point);

        prop_assert_eq!(residual.clone(), s(0));
        prop_assert_eq!(residual.zero_status(), ZeroStatus::Zero);
    }

    #[test]
    fn distance_residual_is_translation_invariant(a in point3(), b in point3(), offset in point3()) {
        let translated_a = a.clone() + offset.clone();
        let translated_b = b.clone() + offset;

        prop_assert_eq!(
            squared_distance(&a, &b),
            squared_distance(&translated_a, &translated_b)
        );
    }

    #[test]
    fn midpoint_constraint_residual_is_exact_zero(a in point3(), b in point3()) {
        let midpoint = ((a.clone() + b.clone()) / s(2)).unwrap();
        let residual = midpoint_residual(&a, &b, &midpoint);

        prop_assert_eq!(residual.clone(), Vector3::zero());
        for lane in 0..3 {
            prop_assert_eq!(residual[lane].zero_status(), ZeroStatus::Zero);
        }
    }

    #[test]
    fn perpendicularity_residual_is_exact_zero_for_constructed_orthogonal_vectors(x in -64_i32..=64, y in -64_i32..=64) {
        let lhs = Vector3::new([s(x), s(y), s(0)]);
        let rhs = Vector3::new([s(-y), s(x), s(0)]);
        let residual = lhs.dot(&rhs);

        prop_assert_eq!(residual.clone(), s(0));
        prop_assert_eq!(residual.zero_status(), ZeroStatus::Zero);
    }

    #[test]
    fn projection_residual_is_zero_after_projecting_onto_axis(point in point3(), axis in nonzero_direction3()) {
        let scale = (point.dot(&axis) / axis.dot(&axis)).unwrap();
        let projection = axis.clone() * scale;
        let residual = (point - projection).dot(&axis);

        prop_assert_eq!(residual.clone(), s(0));
        prop_assert_eq!(residual.zero_status(), ZeroStatus::Zero);
    }
}
