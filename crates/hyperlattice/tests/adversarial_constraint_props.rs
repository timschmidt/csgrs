//! Generated exact residual checks for elementary 3D coordinate constraints.
//!
//! Uses integer Vector3 coordinates in `[-128,128]`, except the perpendicular-pair
//! coordinates in `[-64,64]`. Each property requests 256 cases with Proptest's remaining
//! configuration defaults. Midpoints and projections may introduce exact rationals.
//! These are bounded-domain algebraic checks, not a constraint solver or exhaustive
//! coverage of symbolic coordinates, degeneracies or resource behavior.

use hyperlattice::{Real, Vector3, ZeroKnowledge};
use proptest::prelude::*;

/// Imports an i32 generator coordinate exactly into Real without floating-point conversion.
fn s(value: i32) -> Real {
    value.into()
}

/// Generates three independent integer coordinates in inclusive `[-128,128]` as a Vector3.
///
/// The name denotes coordinate data; the returned carrier is not Point3.
fn point3() -> impl Strategy<Value = Vector3> {
    (-128_i32..=128, -128_i32..=128, -128_i32..=128)
        .prop_map(|(x, y, z)| Vector3::new([s(x), s(y), s(z)]))
}

/// Filters the coordinate strategy to vectors whose squared norm has known nonzero status.
///
/// Accepts no zero vector and performs no unit normalization. Filtering uses the
/// same exact dot/fact implementation later consumed by the projection property.
fn nonzero_direction3() -> impl Strategy<Value = Vector3> {
    point3().prop_filter("nonzero direction", |value| {
        value.dot(value).zero_status() == ZeroKnowledge::NonZero
    })
}

/// Constructs the exact squared Euclidean distance as (a-b)·(a-b), without taking a square root.
fn squared_distance(a: &Vector3, b: &Vector3) -> Real {
    let delta = a - b;
    delta.dot(&delta)
}

/// Returns the vector constraint residual 2*midpoint-(a+b), which vanishes for an exact midpoint.
fn midpoint_residual(a: &Vector3, b: &Vector3, midpoint: &Vector3) -> Vector3 {
    midpoint.clone() * s(2) - (a + b)
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(256))]

    /// Requires self-distance to equal scalar zero and retain known-zero status for generated coordinates.
    #[test]
    fn distance_equality_residual_is_exact_zero_for_identical_points(point in point3()) {
        let residual = squared_distance(&point, &point);

        prop_assert_eq!(residual.clone(), s(0));
        prop_assert_eq!(residual.zero_status(), ZeroKnowledge::Zero);
    }

    /// Checks squared distance is unchanged when the same generated integer offset is added to both points.
    #[test]
    fn distance_residual_is_translation_invariant(a in point3(), b in point3(), offset in point3()) {
        let translated_a = a.clone() + offset.clone();
        let translated_b = b.clone() + offset;

        prop_assert_eq!(
            squared_distance(&a, &b),
            squared_distance(&translated_a, &translated_b)
        );
    }

    /// Constructs (a+b)/2 and checks its midpoint residual is the zero vector with known-zero lanes.
    #[test]
    fn midpoint_constraint_residual_is_exact_zero(a in point3(), b in point3()) {
        let midpoint = ((a.clone() + b.clone()) / s(2)).unwrap();
        let residual = midpoint_residual(&a, &b, &midpoint);

        prop_assert_eq!(residual.clone(), Vector3::zero());
        for lane in 0..3 {
            prop_assert_eq!(residual[lane].zero_status(), ZeroKnowledge::Zero);
        }
    }

    /// Checks (x,y,0)·(-y,x,0)=0 and known-zero status for x,y in inclusive `[-64,64]`.
    ///
    /// Zero vectors are allowed; the assertion concerns orthogonality, not valid unit axes.
    #[test]
    fn perpendicularity_residual_is_exact_zero_for_constructed_orthogonal_vectors(x in -64_i32..=64, y in -64_i32..=64) {
        let lhs = Vector3::new([s(x), s(y), s(0)]);
        let rhs = Vector3::new([s(-y), s(x), s(0)]);
        let residual = lhs.dot(&rhs);

        prop_assert_eq!(residual.clone(), s(0));
        prop_assert_eq!(residual.zero_status(), ZeroKnowledge::Zero);
    }

    /// Checks that subtracting axis*(point·axis)/(axis·axis) leaves an orthogonal residual.
    ///
    /// The generator establishes known nonzero squared norm; axes need not be unit.
    /// Both scalar equality to zero and known-zero status are required.
    #[test]
    fn projection_residual_is_zero_after_projecting_onto_axis(point in point3(), axis in nonzero_direction3()) {
        let scale = (point.dot(&axis) / axis.dot(&axis)).unwrap();
        let projection = axis.clone() * scale;
        let residual = (point - projection).dot(&axis);

        prop_assert_eq!(residual.clone(), s(0));
        prop_assert_eq!(residual.zero_status(), ZeroKnowledge::Zero);
    }
}
