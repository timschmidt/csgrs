mod common;

use common::{frac, r};
use hyperlattice::{Matrix3, Matrix4, Vector3, Vector4};

fn translation_2d(x: i32, y: i32) -> Matrix3 {
    Matrix3::new([[r(1), r(0), r(x)], [r(0), r(1), r(y)], [r(0), r(0), r(1)]])
}

fn scale_2d(x: i32, y: i32) -> Matrix3 {
    Matrix3::new([[r(x), r(0), r(0)], [r(0), r(y), r(0)], [r(0), r(0), r(1)]])
}

fn translation_3d(x: i32, y: i32, z: i32) -> Matrix4 {
    Matrix4::new([
        [r(1), r(0), r(0), r(x)],
        [r(0), r(1), r(0), r(y)],
        [r(0), r(0), r(1), r(z)],
        [r(0), r(0), r(0), r(1)],
    ])
}

fn scale_3d(x: i32, y: i32, z: i32) -> Matrix4 {
    Matrix4::new([
        [r(x), r(0), r(0), r(0)],
        [r(0), r(y), r(0), r(0)],
        [r(0), r(0), r(z), r(0)],
        [r(0), r(0), r(0), r(1)],
    ])
}

#[test]
fn vector3_normalize_preserves_direction_and_unit_length() {
    let direction = Vector3::new([r(6), r(8), r(0)]);

    let unit = direction.normalize().unwrap();

    assert_eq!(unit, Vector3::new([frac(3, 5), frac(4, 5), r(0)]));
    assert_eq!(unit.dot(&unit), r(1));
}

#[test]
fn vector4_dot_product_separates_orthogonal_homogeneous_axes() {
    let x_axis = Vector4::new([r(1), r(0), r(0), r(0)]);
    let y_axis = Vector4::new([r(0), r(1), r(0), r(0)]);
    let point = Vector4::new([r(3), r(4), r(5), r(1)]);

    assert_eq!(x_axis.dot(&y_axis), r(0));
    assert_eq!(point.dot(&x_axis), r(3));
    assert_eq!(point.dot(&y_axis), r(4));
}

#[test]
fn matrix3_affine_transform_scales_then_translates_points() {
    let transform = translation_2d(5, -2) * scale_2d(3, 4);
    let point = Vector3::new([r(2), r(3), r(1)]);

    assert_eq!(transform * point, Vector3::new([r(11), r(10), r(1)]));
}

#[test]
fn matrix3_translation_does_not_move_directions() {
    let transform = translation_2d(5, -2);
    let direction = Vector3::new([r(2), r(3), r(0)]);

    assert_eq!(transform * direction.clone(), direction);
}

#[test]
fn matrix3_determinant_tracks_area_scale_not_translation() {
    let transform = translation_2d(9, -4) * scale_2d(3, -5);

    assert_eq!(transform.determinant(), r(-15));
}

#[test]
fn matrix3_inverse_round_trips_affine_points() {
    let transform = translation_2d(7, -3) * scale_2d(2, 5);
    let point = Vector3::new([r(4), r(-2), r(1)]);
    let transformed = transform.clone() * point.clone();

    assert_eq!(transform.inverse().unwrap() * transformed, point);
}

#[test]
fn matrix3_integer_power_repeats_translation() {
    let transform = translation_2d(2, -3);
    let point = Vector3::new([r(1), r(1), r(1)]);

    assert_eq!(
        (transform ^ 3).unwrap() * point,
        Vector3::new([r(7), r(-8), r(1)])
    );
}

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

#[test]
fn matrix4_inverse_round_trips_scaled_translated_points() {
    let transform = translation_3d(3, -5, 11) * scale_3d(2, 3, 4);
    let point = Vector4::new([r(7), r(-2), r(5), r(1)]);
    let transformed = transform.clone() * point.clone();

    assert_eq!(transform.inverse().unwrap() * transformed, point);
}

#[test]
fn matrix4_determinant_tracks_volume_scale_not_translation() {
    let transform = translation_3d(3, 4, 5) * scale_3d(-2, 3, 4);

    assert_eq!(transform.determinant(), r(-24));
}
