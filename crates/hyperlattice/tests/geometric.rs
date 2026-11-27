//! Exact rational examples of column-vector affine geometry in two and three dimensions.
//!
//! Homogeneous weight one denotes a point and zero a direction. Explicit coordinate,
//! norm, and determinant expectations check the chosen fixtures; inverse roundtrips
//! share the implementation under test and are not independent inverse oracles.
//! These cases do not cover general projective normalization or singular transforms.

/// Shared exact integer and rational literals for the geometric fixtures.
mod common;

use common::{frac, r};
use hyperlattice::{Matrix3, Matrix4, Vector3, Vector4};

/// Construct the homogeneous 3×3 matrix sending `(u,v,w)` to `(u+x*w,v+y*w,w)`.
///
/// Integer translation parameters are imported exactly; no normalization is applied.
fn translation_2d(x: i32, y: i32) -> Matrix3 {
    Matrix3::new([[r(1), r(0), r(x)], [r(0), r(1), r(y)], [r(0), r(0), r(1)]])
}

/// Construct `diag(x,y,1)` for integer scaling of two spatial coordinates.
///
/// Zero and negative scales are accepted; invertibility is not a constructor condition.
fn scale_2d(x: i32, y: i32) -> Matrix3 {
    Matrix3::new([[r(x), r(0), r(0)], [r(0), r(y), r(0)], [r(0), r(0), r(1)]])
}

/// Construct the homogeneous 4×4 matrix adding `(x*w,y*w,z*w)` to the spatial coordinates.
///
/// The weight is retained, so weight-zero directions are unchanged.
fn translation_3d(x: i32, y: i32, z: i32) -> Matrix4 {
    Matrix4::new([
        [r(1), r(0), r(0), r(x)],
        [r(0), r(1), r(0), r(y)],
        [r(0), r(0), r(1), r(z)],
        [r(0), r(0), r(0), r(1)],
    ])
}

/// Construct `diag(x,y,z,1)` with exact integer scales, including zero or negative inputs.
fn scale_3d(x: i32, y: i32, z: i32) -> Matrix4 {
    Matrix4::new([
        [r(x), r(0), r(0), r(0)],
        [r(0), r(y), r(0), r(0)],
        [r(0), r(0), r(z), r(0)],
        [r(0), r(0), r(0), r(1)],
    ])
}

/// Check `(6,8,0)` normalizes to `(3/5,4/5,0)` and has squared norm one.
///
/// The positive norm is ten, so this fixture has an independent rational expectation.
#[test]
fn vector3_normalize_preserves_direction_and_unit_length() {
    let direction = Vector3::new([r(6), r(8), r(0)]);

    let unit = direction.normalize().unwrap();

    assert_eq!(unit, Vector3::new([frac(3, 5), frac(4, 5), r(0)]));
    assert_eq!(unit.dot(&unit), r(1));
}

/// Check four-coordinate dot products of the x/y axes and `(3,4,5,1)`.
///
/// These are ordinary Vector4 dot products; no projective metric is inferred.
#[test]
fn vector4_dot_product_separates_orthogonal_homogeneous_axes() {
    let x_axis = Vector4::new([r(1), r(0), r(0), r(0)]);
    let y_axis = Vector4::new([r(0), r(1), r(0), r(0)]);
    let point = Vector4::new([r(3), r(4), r(5), r(1)]);

    assert_eq!(x_axis.dot(&y_axis), r(0));
    assert_eq!(point.dot(&x_axis), r(3));
    assert_eq!(point.dot(&y_axis), r(4));
}

/// Check `T(5,-2)*S(3,4)` sends the weight-one point `(2,3,1)` to `(11,10,1)`.
///
/// The right factor acts first under the column-vector convention.
#[test]
fn matrix3_affine_transform_scales_then_translates_points() {
    let transform = translation_2d(5, -2) * scale_2d(3, 4);
    let point = Vector3::new([r(2), r(3), r(1)]);

    assert_eq!(transform * point, Vector3::new([r(11), r(10), r(1)]));
}

/// Check translation `(5,-2)` leaves the weight-zero direction `(2,3,0)` unchanged.
#[test]
fn matrix3_translation_does_not_move_directions() {
    let transform = translation_2d(5, -2);
    let direction = Vector3::new([r(2), r(3), r(0)]);

    assert_eq!(transform * direction.clone(), direction);
}

/// Check the translated diagonal scale `(3,-5)` has determinant -15.
///
/// The determinant is the signed planar area factor; its magnitude is the area ratio.
#[test]
fn matrix3_determinant_tracks_area_scale_not_translation() {
    let transform = translation_2d(9, -4) * scale_2d(3, -5);

    assert_eq!(transform.determinant(), r(-15));
}

/// Roundtrip one point through `T(7,-3)*S(2,5)` and its computed inverse.
///
/// This is a composition consistency check, using the same matrix/scalar kernel twice.
#[test]
fn matrix3_inverse_round_trips_affine_points() {
    let transform = translation_2d(7, -3) * scale_2d(2, 5);
    let point = Vector3::new([r(4), r(-2), r(1)]);
    let transformed = transform.clone() * point.clone();

    assert_eq!(transform.inverse().unwrap() * transformed, point);
}

/// Check cubing translation `(2,-3)` sends `(1,1,1)` to `(7,-8,1)` exactly.
#[test]
fn matrix3_integer_power_repeats_translation() {
    let transform = translation_2d(2, -3);
    let point = Vector3::new([r(1), r(1), r(1)]);

    assert_eq!(
        (transform ^ 3).unwrap() * point,
        Vector3::new([r(7), r(-8), r(1)])
    );
}

/// Check translation `(10,-4,7)` moves `(1,2,3,1)` to `(11,-2,10,1)` and retains `(1,2,3,0)`.
#[test]
fn matrix4_homogeneous_translation_moves_points_not_directions() {
    let transform = translation_3d(10, -4, 7);
    let point = Vector4::new([r(1), r(2), r(3), r(1)]);
    let direction = Vector4::new([r(1), r(2), r(3), r(0)]);

    assert_eq!(
        transform.clone() * point,
        Vector4::new([r(11), r(-2), r(10), r(1)])
    );
    assert_eq!(transform * direction.clone(), direction);
}

/// Roundtrip `(7,-2,5,1)` through `T(3,-5,11)*S(2,3,4)` and its computed inverse.
///
/// This checks composition on one point, without independently asserting inverse entries.
#[test]
fn matrix4_inverse_round_trips_scaled_translated_points() {
    let transform = translation_3d(3, -5, 11) * scale_3d(2, 3, 4);
    let point = Vector4::new([r(7), r(-2), r(5), r(1)]);
    let transformed = transform.clone() * point.clone();

    assert_eq!(transform.inverse().unwrap() * transformed, point);
}

/// Check translated spatial scales `(-2,3,4)` have determinant -24, the signed volume factor.
#[test]
fn matrix4_determinant_tracks_volume_scale_not_translation() {
    let transform = translation_3d(3, 4, 5) * scale_3d(-2, 3, 4);

    assert_eq!(transform.determinant(), r(-24));
}
