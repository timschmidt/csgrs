//! Exact same-axis rotation cancellation through strict Boolean composition.
//! Binary64 imports and decimal literals are distinct exact input families.

#![cfg(feature = "curve")]

#[allow(dead_code)]
#[path = "support/downstream_workloads.rs"]
mod support;

use csgrs::{GeometryCertainty, GeometryContext, Real, TriangleMesh, solid};
use hyperlimit::Sign;
use std::cmp::Ordering;
use support::{assert_certified_closed_manifold, r, six_volume, strict_order, strict_sign};

fn assert_unit_cube(mesh: &TriangleMesh) {
    assert!(!mesh.triangles.is_empty());
    assert_certified_closed_manifold(mesh, "compound rotation union");
    for point in mesh.positions.iter() {
        for coordinate in [&point.x, &point.y, &point.z] {
            assert_ne!(strict_order(coordinate, &r(0)), Ordering::Less);
            assert_ne!(strict_order(coordinate, &r(1)), Ordering::Greater);
        }
    }
    // An independent six-half-space oracle: every triangle belongs to one
    // cube support plane and points outward. No triangulation/count equality.
    for triangle in mesh.triangles.iter() {
        let [a, b, c] = triangle
            .indices()
            .map(|index| mesh.positions[index].to_vector());
        let normal = (&b - &a).cross(&(&c - &a));
        let mut support = false;
        for axis in 0..3 {
            for side in [0, 1] {
                if [&a, &b, &c]
                    .into_iter()
                    .all(|point| strict_order(&point.0[axis], &r(side)) == Ordering::Equal)
                {
                    assert_eq!(
                        strict_sign(&normal.0[axis]),
                        if side == 0 {
                            Sign::Negative
                        } else {
                            Sign::Positive
                        }
                    );
                    for other in 0..3 {
                        if other != axis {
                            assert_eq!(strict_sign(&normal.0[other]), Sign::Zero);
                        }
                    }
                    support = true;
                }
            }
        }
        assert!(support, "triangle is not on an outward cube support plane");
    }
    // Closed, outward boundary on the cube's six planes with unit volume
    // excludes missing material and extra closed coverings of those planes.
    assert_eq!(strict_order(&six_volume(mesh), &r(6)), Ordering::Equal);
}

fn compound_union(binary64: bool) {
    let angles = if binary64 {
        [13.1_f64, 17.3_f64, -13.1_f64, -17.3_f64].map(|value| Real::try_from(value).unwrap())
    } else {
        ["13.1", "17.3", "-13.1", "-17.3"].map(|value| value.parse::<Real>().unwrap())
    };
    let cube = solid::cube(r(1)).expect("unit cube");
    let mut rotated = cube.clone();
    for angle in angles {
        rotated = solid::try_rotate(&rotated, r(0), r(0), angle).expect("finite z rotation");
    }
    // Do not classify intermediate coordinates or compare the two meshes
    // before the original Boolean: such queries can warm exact proof caches.
    let outcome = solid::boolean_with_context(
        &cube,
        &rotated,
        hypermesh::BooleanOp::Union,
        &GeometryContext::STRICT,
    )
    .unwrap_or_else(|error| {
        panic!("original strict compound-rotation union did not complete: {error}")
    });
    assert_eq!(outcome.certainty, GeometryCertainty::Certified);
    let mesh = outcome.into_value();
    assert_unit_cube(&mesh);
    let empty = solid::boolean_with_context(
        &mesh,
        &cube,
        hypermesh::BooleanOp::Difference,
        &GeometryContext::STRICT,
    )
    .unwrap_or_else(|error| {
        panic!("certified cube union could not continue to exact difference: {error}")
    });
    assert_eq!(empty.certainty, GeometryCertainty::Certified);
    assert!(empty.value.triangles.is_empty() && empty.value.positions.is_empty());
    let restored = solid::boolean_with_context(
        &empty.value,
        &cube,
        hypermesh::BooleanOp::Union,
        &GeometryContext::STRICT,
    )
    .unwrap();
    assert_eq!(restored.certainty, GeometryCertainty::Certified);
    assert_unit_cube(&restored.value);
}

#[test]
fn binary64_compound_round_trip_union_matches_unit_cube() {
    compound_union(true);
}

#[test]
fn decimal_compound_round_trip_union_matches_unit_cube() {
    compound_union(false);
}

#[test]
fn compound_rotation_families_on_each_axis_retain_exact_continuation() {
    let families = [
        [13.1_f64, 17.3_f64, -13.1_f64, -17.3_f64],
        [13.1_f64, 17.3_f64, -17.3_f64, -13.1_f64],
        [-7.7_f64, 33.8_f64, 7.7_f64, -33.8_f64],
    ];
    for axis in 0..3 {
        for (family, angles) in families.into_iter().enumerate() {
            for context in [GeometryContext::STRICT, GeometryContext::APPROXIMATE_512] {
                let cube = solid::cube(r(1)).unwrap();
                let mut rotated = cube.clone();
                for angle in angles {
                    let mut degrees = [r(0), r(0), r(0)];
                    degrees[axis] = Real::try_from(angle).unwrap();
                    let [x, y, z] = degrees;
                    rotated = solid::try_rotate(&rotated, x, y, z).unwrap();
                }
                // Exact signed angles sum to zero on one axis. No intermediate
                // coordinate/sign queries prewarm the original union.
                let union = solid::boolean_with_context(
                    &cube,
                    &rotated,
                    hypermesh::BooleanOp::Union,
                    &context,
                )
                .unwrap_or_else(|error| panic!("axis {axis} family {family}: {error}"));
                assert_eq!(union.certainty, GeometryCertainty::Certified);
                assert_unit_cube(&union.value);
                let empty = solid::boolean_with_context(
                    &union.value,
                    &cube,
                    hypermesh::BooleanOp::Difference,
                    &context,
                )
                .unwrap();
                assert_eq!(empty.certainty, GeometryCertainty::Certified);
                assert!(empty.value.triangles.is_empty() && empty.value.positions.is_empty());
                let restored = solid::boolean_with_context(
                    &empty.value,
                    &cube,
                    hypermesh::BooleanOp::Union,
                    &GeometryContext::STRICT,
                )
                .unwrap();
                assert_eq!(restored.certainty, GeometryCertainty::Certified);
                assert_unit_cube(&restored.value);
            }
        }
    }
}
