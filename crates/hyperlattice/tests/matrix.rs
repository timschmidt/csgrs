mod common;

use common::{abort_signal, frac, r, unknown_zero};
use hyperlattice::{
    Matrix3, Matrix4, Problem, Scalar, ScalarSign, SignedAxis4, Vector3, Vector4, ZeroStatus, zero,
};

fn assert_singular_error<T: std::fmt::Debug>(result: Result<T, Problem>) {
    assert!(matches!(
        result,
        Err(Problem::DivideByZero | Problem::UnknownZero)
    ));
}

#[test]
fn matrix3_inverse_and_power() {
    let matrix = Matrix3::new([[r(1), r(2), r(3)], [r(0), r(1), r(4)], [r(5), r(6), r(0)]]);

    assert_eq!(matrix.determinant(), r(1));
    assert_eq!(
        matrix.clone() * matrix.clone().inverse().unwrap(),
        Matrix3::identity()
    );
    assert_eq!((matrix.clone() ^ 0).unwrap(), Matrix3::identity());
    assert_eq!((matrix.clone() ^ 1).unwrap(), matrix);
}

#[test]
fn matrix3_negative_power_matches_repeated_inverse_product() {
    let matrix = Matrix3::new([[r(2), r(1), r(0)], [r(0), r(3), r(1)], [r(1), r(0), r(2)]]);
    let inverse = matrix.clone().inverse().unwrap();

    assert_eq!(matrix.powi(-2).unwrap(), inverse.clone() * inverse);
}

#[test]
fn matrix4_identity_and_vector_multiply() {
    let vector = Vector4::new([r(1), r(2), r(3), r(4)]);
    assert_eq!(Matrix4::identity() * vector.clone(), vector);
}

#[test]
fn matrix4_translated_diagonal_affine_preserves_point_direction_semantics() {
    let transform = Matrix4::new([
        [r(2), r(0), r(0), r(100)],
        [r(0), r(3), r(0), r(200)],
        [r(0), r(0), r(4), r(300)],
        [r(0), r(0), r(0), r(1)],
    ]);
    let direction = Vector4::new([r(5), r(7), r(11), r(0)]);
    let point = Vector4::new([r(5), r(7), r(11), r(1)]);

    let transformed_direction = Vector4::new([r(10), r(21), r(44), r(0)]);
    let transformed_point = Vector4::new([r(110), r(221), r(344), r(1)]);

    assert_eq!(transform.clone() * direction.clone(), transformed_direction);
    assert_eq!(
        transform.clone().transform_vec4_direction(&direction),
        transformed_direction
    );
    assert_eq!(transform.clone() * point.clone(), transformed_point);
    assert_eq!(transform.transform_vec4_point(&point), transformed_point);

    assert_eq!(
        transform
            .transform_vec4_batch(&[direction.clone(), Vector4::new([r(13), r(17), r(19), r(0)])]),
        vec![
            transformed_direction.clone(),
            Vector4::new([r(26), r(51), r(76), r(0)]),
        ]
    );
    assert_eq!(
        transform.transform_vec4_batch(&[point.clone(), Vector4::new([r(13), r(17), r(19), r(1)])]),
        vec![
            transformed_point.clone(),
            Vector4::new([r(126), r(251), r(376), r(1)]),
        ]
    );

    let handle = transform.transform_vec4_handle();
    assert_eq!(
        handle.transform_vector_batch(&[
            direction.clone(),
            Vector4::new([r(13), r(17), r(19), r(0)])
        ]),
        vec![
            transformed_direction,
            Vector4::new([r(26), r(51), r(76), r(0)]),
        ]
    );
    assert_eq!(
        handle.transform_vector_batch(&[point.clone(), Vector4::new([r(13), r(17), r(19), r(1)])]),
        vec![
            transformed_point,
            Vector4::new([r(126), r(251), r(376), r(1)]),
        ]
    );
}

#[test]
fn matrix4_assumed_homogeneous_batches_match_single_lane_transforms() {
    let transform = Matrix4::new([
        [r(2), r(0), r(0), r(100)],
        [r(0), r(3), r(0), r(200)],
        [r(0), r(0), r(4), r(300)],
        [r(0), r(0), r(0), r(1)],
    ]);
    let directions = [
        Vector4::new([r(5), r(7), r(11), r(0)]),
        Vector4::new([r(13), r(17), r(19), r(0)]),
        Vector4::new([r(23), r(29), r(31), r(0)]),
    ];
    let points = [
        Vector4::new([r(5), r(7), r(11), r(1)]),
        Vector4::new([r(13), r(17), r(19), r(1)]),
        Vector4::new([r(23), r(29), r(31), r(1)]),
    ];

    let expected_directions = directions
        .iter()
        .map(|vector| transform.transform_vec4_direction(vector))
        .collect::<Vec<_>>();
    let expected_points = points
        .iter()
        .map(|vector| transform.transform_vec4_point(vector))
        .collect::<Vec<_>>();
    let handle = transform.transform_vec4_handle();

    assert_eq!(
        handle.transform_direction_batch(&directions),
        expected_directions
    );
    assert_eq!(
        transform.transform_vec4_direction_batch(&directions),
        expected_directions
    );
    assert_eq!(handle.transform_point_batch(&points), expected_points);
    assert_eq!(
        transform.transform_vec4_point_batch(&points),
        expected_points
    );

    let identity = Matrix4::identity();
    let identity_handle = identity.transform_vec4_handle();
    assert_eq!(
        identity.transform_vec4_direction(&directions[0]),
        directions[0]
    );
    assert_eq!(identity.transform_vec4_point(&points[0]), points[0]);
    assert_eq!(
        identity_handle.transform_direction_vector(&directions[1]),
        directions[1]
    );
    assert_eq!(
        identity_handle.transform_point_vector(&points[1]),
        points[1]
    );
    assert_eq!(
        identity.transform_vec4_with(&directions[2]).materialize(),
        directions[2]
    );
    assert_eq!(
        identity.transform_vec4_with(&points[2]).materialize(),
        points[2]
    );
    assert_eq!(
        identity_handle.transform_direction_batch(&directions),
        directions.to_vec()
    );
    assert_eq!(
        identity_handle.transform_point_batch(&points),
        points.to_vec()
    );
}

#[test]
fn matrix4_inverse_and_determinant_handle_general_integer_matrix() {
    let matrix = Matrix4::new([
        [r(1), r(2), r(3), r(4)],
        [r(0), r(1), r(4), r(2)],
        [r(5), r(6), r(0), r(1)],
        [r(2), r(7), r(1), r(3)],
    ]);

    assert_eq!(matrix.determinant(), r(-198));
    assert_eq!(
        matrix.clone() * matrix.inverse().unwrap(),
        Matrix4::identity()
    );
}

#[test]
fn matrix4_negative_power_matches_repeated_inverse_product() {
    let matrix = Matrix4::new([
        [r(2), r(0), r(1), r(0)],
        [r(1), r(3), r(0), r(1)],
        [r(0), r(2), r(1), r(0)],
        [r(1), r(0), r(0), r(2)],
    ]);
    let inverse = matrix.clone().inverse().unwrap();

    assert_eq!(matrix.powi(-2).unwrap(), inverse.clone() * inverse);
}

#[test]
fn matrix4_prepared_negative_power_matches_ordinary_power() {
    let matrix = Matrix4::new([
        [r(2), r(0), r(1), r(0)],
        [r(1), r(3), r(0), r(1)],
        [r(0), r(2), r(1), r(0)],
        [r(1), r(0), r(0), r(2)],
    ]);
    let signal = abort_signal();
    let mut prepared = matrix.prepare_right_divisor();

    assert_eq!(prepared.powi(-2).unwrap(), matrix.clone().powi(-2).unwrap());
    assert_eq!(
        prepared.powi_checked(-2).unwrap(),
        matrix.clone().powi_checked(-2).unwrap()
    );
    assert_eq!(
        prepared.powi_checked_with_abort(-2, &signal).unwrap(),
        matrix.powi_checked_with_abort(-2, &signal).unwrap()
    );
}

#[cfg(feature = "hyperreal-backend")]
#[test]
fn targeted_fractional_matrix_forms_round_trip_exactly() {
    let dyadic3 = Matrix3::new([
        [frac(9, 8), frac(3, 16), frac(-5, 8)],
        [frac(7, 4), frac(-11, 8), frac(13, 16)],
        [frac(5, 8), frac(17, 16), frac(19, 8)],
    ]);
    let equal_den4 = Matrix4::new([
        [frac(11, 10), frac(2, 10), frac(3, 10), frac(4, 10)],
        [frac(5, 10), frac(17, 10), frac(7, 10), frac(-8, 10)],
        [frac(9, 10), frac(-10, 10), frac(23, 10), frac(12, 10)],
        [frac(-13, 10), frac(14, 10), frac(-15, 10), frac(19, 10)],
    ]);
    let mixed_prime4 = Matrix4::new([
        [frac(7, 3), frac(-5, 7), frac(11, 13), frac(13, 5)],
        [frac(17, 11), frac(-19, 17), frac(23, 19), frac(-29, 23)],
        [frac(31, 29), frac(37, 31), frac(-41, 37), frac(43, 41)],
        [frac(-47, 43), frac(53, 47), frac(59, 53), frac(61, 59)],
    ]);

    let dyadic3_inverse = dyadic3.clone().inverse().unwrap();
    assert_eq!(dyadic3.clone() * dyadic3_inverse, Matrix3::identity());
    assert_eq!(
        dyadic3.clone().powi(-2).unwrap(),
        dyadic3.clone().inverse().unwrap() * dyadic3.inverse().unwrap()
    );

    let equal_den4_inverse = equal_den4.clone().inverse().unwrap();
    assert_eq!(equal_den4.clone() * equal_den4_inverse, Matrix4::identity());
    assert_eq!(
        equal_den4.clone().powi(-2).unwrap(),
        equal_den4.clone().inverse().unwrap() * equal_den4.inverse().unwrap()
    );

    let mixed_prime4_inverse = mixed_prime4.clone().inverse().unwrap();
    assert_eq!(
        mixed_prime4.clone() * mixed_prime4_inverse,
        Matrix4::identity()
    );
}

#[test]
fn matrix_scalar_add_and_subtract_are_componentwise() {
    let matrix = Matrix3::new([[r(1), r(2), r(3)], [r(4), r(5), r(6)], [r(7), r(8), r(9)]]);

    assert_eq!(
        matrix.clone() + r(1),
        Matrix3::new([[r(2), r(3), r(4)], [r(5), r(6), r(7)], [r(8), r(9), r(10)],])
    );
    assert_eq!(
        matrix - r(2),
        Matrix3::new([[r(-1), r(0), r(1)], [r(2), r(3), r(4)], [r(5), r(6), r(7)],])
    );
}

#[test]
fn matrix_display_forwards_real_formatting() {
    let half = frac(1, 2);
    let quarter = frac(1, 4);
    let matrix = Matrix3::new([
        [half, r(2), r(3)],
        [r(4), quarter, r(6)],
        [r(7), r(8), r(9)],
    ]);

    #[cfg(feature = "hyperreal-backend")]
    assert_eq!(format!("{matrix}"), "[[1/2, 2, 3], [4, 1/4, 6], [7, 8, 9]]");
    #[cfg(not(feature = "hyperreal-backend"))]
    assert_eq!(
        format!("{matrix}"),
        "[[0.5, 2, 3], [4, 0.25, 6], [7, 8, 9]]"
    );
    assert_eq!(
        format!("{matrix:#}"),
        "[[0.5, 2, 3], [4, 0.25, 6], [7, 8, 9]]"
    );
}

#[test]
fn checked_matrix_inverse_rejects_singular_matrices() {
    let singular = Matrix3::new([[r(1), r(2), r(3)], [r(1), r(2), r(3)], [r(0), r(0), r(1)]]);
    let invertible = Matrix3::new([[r(1), r(2), r(3)], [r(0), r(1), r(4)], [r(5), r(6), r(0)]]);
    let signal = abort_signal();

    assert_singular_error(singular.clone().inverse());
    assert_singular_error(singular.clone().reciprocal());
    assert_singular_error(singular.clone().powi(-1));
    assert_singular_error(Matrix3::identity() / singular.clone());
    assert_eq!(Matrix3::identity() / zero(), Err(Problem::DivideByZero));
    assert_singular_error(singular.inverse_checked());
    assert_eq!(
        invertible.clone() * invertible.clone().inverse_checked().unwrap(),
        Matrix3::identity()
    );
    assert_eq!(
        invertible.clone()
            * invertible
                .clone()
                .inverse_checked_with_abort(&signal)
                .unwrap(),
        Matrix3::identity()
    );
    assert_eq!(
        invertible
            .clone()
            .powi_checked_with_abort(-1, &signal)
            .unwrap(),
        invertible.inverse_checked_with_abort(&signal).unwrap()
    );
}

#[test]
fn matrix4_diagonal_inverse_uses_only_diagonal_reciprocals() {
    let matrix = Matrix4::new([
        [r(2), r(0), r(0), r(0)],
        [r(0), r(4), r(0), r(0)],
        [r(0), r(0), r(5), r(0)],
        [r(0), r(0), r(0), r(1)],
    ]);
    let expected = Matrix4::new([
        [frac(1, 2), r(0), r(0), r(0)],
        [r(0), frac(1, 4), r(0), r(0)],
        [r(0), r(0), frac(1, 5), r(0)],
        [r(0), r(0), r(0), r(1)],
    ]);

    assert_eq!(matrix.clone().reciprocal().unwrap(), expected);
    assert_eq!(matrix.clone().inverse_checked().unwrap(), expected);
    assert_eq!(
        matrix.inverse_checked_with_abort(&abort_signal()).unwrap(),
        expected
    );
}

#[test]
fn matrix4_uniform_scale_inverse_reuses_one_reciprocal_semantics() {
    let matrix = Matrix4::new([
        [r(6), r(0), r(0), r(0)],
        [r(0), r(6), r(0), r(0)],
        [r(0), r(0), r(6), r(0)],
        [r(0), r(0), r(0), r(6)],
    ]);
    let expected = Matrix4::new([
        [frac(1, 6), r(0), r(0), r(0)],
        [r(0), frac(1, 6), r(0), r(0)],
        [r(0), r(0), frac(1, 6), r(0)],
        [r(0), r(0), r(0), frac(1, 6)],
    ]);

    assert_eq!(matrix.clone().reciprocal().unwrap(), expected);
    assert_eq!(matrix.clone().inverse_checked().unwrap(), expected);
    assert_eq!(
        matrix.inverse_checked_with_abort(&abort_signal()).unwrap(),
        expected
    );
}

#[test]
fn matrix4_known_uniform_scale_inverse_matches_matrix_inverse() {
    let scale = r(6);
    let matrix = Matrix4::uniform_scale(scale);
    let expected = Matrix4::new([
        [frac(1, 6), r(0), r(0), r(0)],
        [r(0), frac(1, 6), r(0), r(0)],
        [r(0), r(0), frac(1, 6), r(0)],
        [r(0), r(0), r(0), frac(1, 6)],
    ]);

    assert_eq!(matrix.clone().reciprocal().unwrap(), expected);
    assert_eq!(Matrix4::uniform_scale_inverse(r(6)).unwrap(), expected);
}

#[test]
fn matrix4_known_affine_translation_matches_generic_paths() {
    let translation = [r(3), r(-5), r(7)];
    let matrix = Matrix4::affine_translation(translation.clone());
    let expected_inverse = Matrix4::affine_translation([r(-3), r(5), r(-7)]);

    assert_eq!(matrix.clone().reciprocal().unwrap(), expected_inverse);
    assert_eq!(
        Matrix4::affine_translation_inverse(translation.clone()),
        expected_inverse
    );

    let numerator = Matrix4::new([
        [r(2), r(6), r(15), r(28)],
        [r(10), r(12), r(25), r(42)],
        [r(14), r(18), r(35), r(56)],
        [r(22), r(24), r(45), r(70)],
    ]);

    assert_eq!(
        numerator.clone().div_affine_translation(translation),
        (numerator / matrix).unwrap()
    );
}

#[test]
fn matrix4_known_affine_orthonormal_matches_generic_paths() {
    let linear = [[r(0), r(-1), r(0)], [r(1), r(0), r(0)], [r(0), r(0), r(1)]];
    let translation = [r(3), r(-5), r(7)];
    let matrix = Matrix4::affine_orthonormal(linear.clone(), translation.clone());
    let expected_inverse = Matrix4::new([
        [r(0), r(1), r(0), r(5)],
        [r(-1), r(0), r(0), r(3)],
        [r(0), r(0), r(1), r(-7)],
        [r(0), r(0), r(0), r(1)],
    ]);

    assert_eq!(matrix.clone().reciprocal().unwrap(), expected_inverse);
    assert_eq!(
        Matrix4::affine_orthonormal_inverse(linear.clone(), translation.clone()),
        expected_inverse
    );

    let numerator = Matrix4::new([
        [r(2), r(6), r(15), r(28)],
        [r(10), r(12), r(25), r(42)],
        [r(14), r(18), r(35), r(56)],
        [r(22), r(24), r(45), r(70)],
    ]);

    assert_eq!(
        numerator
            .clone()
            .div_affine_orthonormal(linear, translation),
        (numerator / matrix).unwrap()
    );
}

#[test]
fn matrix4_known_signed_permutation_matches_generic_paths() {
    let rows = [
        SignedAxis4::PosY,
        SignedAxis4::NegX,
        SignedAxis4::PosW,
        SignedAxis4::NegZ,
    ];
    let matrix = Matrix4::signed_permutation(rows);
    let expected_inverse = Matrix4::new([
        [r(0), r(-1), r(0), r(0)],
        [r(1), r(0), r(0), r(0)],
        [r(0), r(0), r(0), r(-1)],
        [r(0), r(0), r(1), r(0)],
    ]);

    assert_eq!(matrix.clone().reciprocal().unwrap(), expected_inverse);
    assert_eq!(Matrix4::signed_permutation_inverse(rows), expected_inverse);

    let numerator = Matrix4::new([
        [r(2), r(6), r(15), r(28)],
        [r(10), r(12), r(25), r(42)],
        [r(14), r(18), r(35), r(56)],
        [r(22), r(24), r(45), r(70)],
    ]);

    assert_eq!(
        numerator.clone().div_signed_permutation(rows),
        (numerator / matrix.clone()).unwrap()
    );

    let vector = Vector4::new([r(2), r(3), r(5), r(7)]);
    let batch = [vector.clone(), Vector4::new([r(11), r(13), r(17), r(19)])];

    assert_eq!(
        Matrix4::transform_signed_permutation_vector(rows, &vector),
        matrix.clone() * vector
    );
    assert_eq!(
        Matrix4::transform_signed_permutation_batch(rows, &batch),
        batch
            .iter()
            .cloned()
            .map(|item| matrix.clone() * item)
            .collect::<Vec<_>>()
    );
}

#[test]
fn matrix4_known_diagonal_inverse_matches_matrix_inverse() {
    let diagonal = [r(2), r(3), r(5), r(7)];
    let matrix = Matrix4::diagonal(diagonal);
    let expected = Matrix4::new([
        [frac(1, 2), r(0), r(0), r(0)],
        [r(0), frac(1, 3), r(0), r(0)],
        [r(0), r(0), frac(1, 5), r(0)],
        [r(0), r(0), r(0), frac(1, 7)],
    ]);

    assert_eq!(matrix.clone().reciprocal().unwrap(), expected);
    assert_eq!(
        Matrix4::diagonal_inverse([r(2), r(3), r(5), r(7)]).unwrap(),
        expected
    );
}

#[test]
fn matrix4_known_diagonal_division_matches_matrix_division() {
    let numerator = Matrix4::new([
        [r(2), r(6), r(15), r(28)],
        [r(10), r(12), r(25), r(42)],
        [r(14), r(18), r(35), r(56)],
        [r(22), r(24), r(45), r(70)],
    ]);
    let diagonal = [r(2), r(3), r(5), r(7)];
    let divisor = Matrix4::diagonal(diagonal.clone());

    assert_eq!(
        numerator.clone().div_diagonal(diagonal).unwrap(),
        (numerator / divisor).unwrap()
    );
}

#[test]
fn matrix4_known_diagonal_div_vector_matches_matrix_division() {
    let numerator = Matrix4::new([
        [r(2), r(6), r(15), r(28)],
        [r(10), r(12), r(25), r(42)],
        [r(14), r(18), r(35), r(56)],
        [r(22), r(24), r(45), r(70)],
    ]);
    let diagonal = [r(2), r(3), r(5), r(7)];
    let vector = Vector4::new([r(5), r(7), r(11), r(13)]);
    let divisor = Matrix4::diagonal(diagonal.clone());

    assert_eq!(
        numerator.div_diagonal_vector(diagonal, &vector).unwrap(),
        (numerator / divisor).unwrap() * vector
    );
}

#[test]
fn matrix4_known_diagonal_div_vector_direction_matches_matrix_division() {
    let numerator = Matrix4::new([
        [r(2), r(6), r(15), r(28)],
        [r(10), r(12), r(25), r(42)],
        [r(14), r(18), r(35), r(56)],
        [r(22), r(24), r(45), r(70)],
    ]);
    let diagonal = [r(2), r(3), r(5), r(7)];
    let vector = Vector4::new([r(5), r(7), r(11), r(0)]);
    let divisor = Matrix4::diagonal(diagonal.clone());

    assert_eq!(
        numerator.div_diagonal_vector(diagonal, &vector).unwrap(),
        (numerator / divisor).unwrap() * vector
    );
}

#[test]
fn matrix4_known_diagonal_div_vector_direction_only_matches_matrix_division() {
    let numerator = Matrix4::new([
        [r(2), r(6), r(15), r(28)],
        [r(10), r(12), r(25), r(42)],
        [r(14), r(18), r(35), r(56)],
        [r(22), r(24), r(45), r(70)],
    ]);
    let diagonal = [r(2), r(3), r(5), r(7)];
    let vector = Vector4::new([r(5), r(7), r(11), r(0)]);
    let divisor = Matrix4::diagonal(diagonal.clone());

    assert_eq!(
        numerator
            .div_diagonal_direction_vector(diagonal, &vector)
            .unwrap(),
        (numerator / divisor).unwrap() * vector
    );
}

#[test]
fn matrix4_known_diagonal_div_vector_point_matches_matrix_division() {
    let numerator = Matrix4::new([
        [r(2), r(6), r(15), r(28)],
        [r(10), r(12), r(25), r(42)],
        [r(14), r(18), r(35), r(56)],
        [r(22), r(24), r(45), r(70)],
    ]);
    let diagonal = [r(2), r(3), r(5), r(1)];
    let vector = Vector4::new([r(5), r(7), r(11), r(1)]);
    let divisor = Matrix4::diagonal(diagonal.clone());

    assert_eq!(
        numerator.div_diagonal_vector(diagonal, &vector).unwrap(),
        (numerator / divisor).unwrap() * vector
    );
}

#[test]
fn matrix4_known_diagonal_div_vector_point_scaled_w() {
    let numerator = Matrix4::new([
        [r(2), r(6), r(15), r(28)],
        [r(10), r(12), r(25), r(42)],
        [r(14), r(18), r(35), r(56)],
        [r(22), r(24), r(45), r(70)],
    ]);
    let diagonal = [r(2), r(3), r(5), r(7)];
    let vector = Vector4::new([r(5), r(7), r(11), r(1)]);
    let divisor = Matrix4::diagonal(diagonal.clone());

    assert_eq!(
        numerator.div_diagonal_vector(diagonal, &vector).unwrap(),
        (numerator / divisor).unwrap() * vector
    );
}

#[test]
fn matrix4_known_upper_triangular_inverse_matches_matrix_inverse() {
    let matrix = Matrix4::new([
        [r(2), r(3), r(5), r(7)],
        [r(0), r(11), r(13), r(17)],
        [r(0), r(0), r(19), r(23)],
        [r(0), r(0), r(0), r(29)],
    ]);
    let inverse = matrix.clone().reciprocal().unwrap();

    assert_eq!(matrix.clone() * inverse.clone(), Matrix4::identity());
    assert_eq!(matrix.clone().upper_triangular_inverse().unwrap(), inverse);
    assert_eq!(
        matrix.clone().upper_triangular_inverse_checked().unwrap(),
        inverse
    );
    assert_eq!(matrix.clone().inverse_checked().unwrap(), inverse);
    assert_eq!(
        matrix
            .clone()
            .upper_triangular_inverse_checked_with_abort(&abort_signal())
            .unwrap(),
        inverse
    );
    assert_eq!(
        matrix.inverse_checked_with_abort(&abort_signal()).unwrap(),
        inverse
    );
}

#[test]
fn matrix4_known_lower_triangular_inverse_matches_matrix_inverse() {
    let matrix = Matrix4::new([
        [r(2), r(0), r(0), r(0)],
        [r(3), r(11), r(0), r(0)],
        [r(5), r(13), r(19), r(0)],
        [r(7), r(17), r(23), r(29)],
    ]);
    let inverse = matrix.clone().reciprocal().unwrap();

    assert_eq!(matrix.clone() * inverse.clone(), Matrix4::identity());
    assert_eq!(matrix.clone().lower_triangular_inverse().unwrap(), inverse);
    assert_eq!(
        matrix.clone().lower_triangular_inverse_checked().unwrap(),
        inverse
    );
    assert_eq!(matrix.clone().inverse_checked().unwrap(), inverse);
    assert_eq!(
        matrix
            .clone()
            .lower_triangular_inverse_checked_with_abort(&abort_signal())
            .unwrap(),
        inverse
    );
    assert_eq!(
        matrix.inverse_checked_with_abort(&abort_signal()).unwrap(),
        inverse
    );
}

#[test]
fn matrix4_known_upper_triangular_inverse_checked_rejects_singular_divisor() {
    let matrix = Matrix4::new([
        [r(2), r(3), r(5), r(7)],
        [r(0), r(0), r(13), r(17)],
        [r(0), r(0), r(19), r(23)],
        [r(0), r(0), r(0), r(29)],
    ]);

    assert_singular_error(matrix.clone().upper_triangular_inverse_checked());
    assert_singular_error(
        matrix
            .clone()
            .upper_triangular_inverse_checked_with_abort(&abort_signal()),
    );
    assert_singular_error(matrix.clone().inverse_checked());
    assert_singular_error(matrix.inverse_checked_with_abort(&abort_signal()));
}

#[test]
fn matrix4_known_lower_triangular_inverse_checked_rejects_singular_divisor() {
    let matrix = Matrix4::new([
        [r(2), r(0), r(0), r(0)],
        [r(3), r(11), r(0), r(0)],
        [r(5), r(13), r(0), r(0)],
        [r(7), r(17), r(23), r(29)],
    ]);

    assert_singular_error(matrix.clone().lower_triangular_inverse_checked());
    assert_singular_error(
        matrix
            .clone()
            .lower_triangular_inverse_checked_with_abort(&abort_signal()),
    );
    assert_singular_error(matrix.clone().inverse_checked());
    assert_singular_error(matrix.inverse_checked_with_abort(&abort_signal()));
}

#[test]
fn matrix4_known_upper_triangular_div_matrix_matches_matrix_division() {
    let numerator = Matrix4::new([
        [r(2), r(6), r(15), r(28)],
        [r(10), r(12), r(25), r(42)],
        [r(14), r(18), r(35), r(56)],
        [r(22), r(24), r(45), r(70)],
    ]);
    let divisor = Matrix4::new([
        [r(2), r(3), r(5), r(7)],
        [r(0), r(11), r(13), r(17)],
        [r(0), r(0), r(19), r(23)],
        [r(0), r(0), r(0), r(29)],
    ]);

    let expected = numerator.clone() * divisor.clone().reciprocal().unwrap();
    assert_eq!(
        numerator
            .clone()
            .div_upper_triangular(divisor.clone())
            .unwrap(),
        expected
    );
    assert_eq!(numerator.clone() / divisor.clone(), Ok(expected));
}

#[test]
fn matrix4_known_lower_triangular_div_matrix_matches_matrix_division() {
    let numerator = Matrix4::new([
        [r(2), r(6), r(15), r(28)],
        [r(10), r(12), r(25), r(42)],
        [r(14), r(18), r(35), r(56)],
        [r(22), r(24), r(45), r(70)],
    ]);
    let divisor = Matrix4::new([
        [r(2), r(0), r(0), r(0)],
        [r(3), r(11), r(0), r(0)],
        [r(5), r(13), r(19), r(0)],
        [r(7), r(17), r(23), r(29)],
    ]);

    let expected = numerator.clone() * divisor.clone().reciprocal().unwrap();
    assert_eq!(
        numerator
            .clone()
            .div_lower_triangular(divisor.clone())
            .unwrap(),
        expected
    );
    assert_eq!(numerator.clone() / divisor.clone(), Ok(expected));
}

#[test]
fn matrix4_known_upper_triangular_div_matrix_checked_matches_ordinary() {
    let numerator = Matrix4::new([
        [r(2), r(6), r(15), r(28)],
        [r(10), r(12), r(25), r(42)],
        [r(14), r(18), r(35), r(56)],
        [r(22), r(24), r(45), r(70)],
    ]);
    let divisor = Matrix4::new([
        [r(2), r(3), r(5), r(7)],
        [r(0), r(11), r(13), r(17)],
        [r(0), r(0), r(19), r(23)],
        [r(0), r(0), r(0), r(29)],
    ]);
    let expected = (numerator.clone() / divisor.clone()).unwrap();

    assert_eq!(
        numerator
            .clone()
            .div_matrix_checked(divisor.clone())
            .unwrap(),
        expected
    );
    assert_eq!(
        numerator
            .clone()
            .div_matrix_checked_with_abort(divisor, &abort_signal())
            .unwrap(),
        expected
    );
}

#[test]
fn matrix4_known_upper_triangular_div_matrix_checked_with_prepared_matches_ordinary() {
    let numerator = Matrix4::new([
        [r(4), r(2), r(7), r(11)],
        [r(1), r(3), r(0), r(13)],
        [r(5), r(8), r(6), r(17)],
        [r(7), r(9), r(11), r(19)],
    ]);
    let divisor = Matrix4::new([
        [r(2), r(3), r(5), r(7)],
        [r(0), r(11), r(13), r(17)],
        [r(0), r(0), r(19), r(23)],
        [r(0), r(0), r(0), r(29)],
    ]);
    let expected = (numerator.clone() / divisor.clone()).unwrap();
    let mut prepared = divisor.prepare_right_divisor();

    assert_eq!(
        numerator
            .clone()
            .div_matrix_with_prepared(&mut prepared)
            .unwrap(),
        expected
    );
    let mut prepared = divisor.prepare_right_divisor();
    assert_eq!(
        numerator
            .clone()
            .div_matrix_checked_with_prepared_with_abort(&mut prepared, &abort_signal())
            .unwrap(),
        expected
    );
}

#[test]
fn matrix4_known_lower_triangular_div_matrix_checked_with_prepared_matches_ordinary() {
    let numerator = Matrix4::new([
        [r(4), r(2), r(7), r(11)],
        [r(1), r(3), r(0), r(13)],
        [r(5), r(8), r(6), r(17)],
        [r(7), r(9), r(11), r(19)],
    ]);
    let divisor = Matrix4::new([
        [r(2), r(0), r(0), r(0)],
        [r(3), r(11), r(0), r(0)],
        [r(5), r(13), r(19), r(0)],
        [r(7), r(17), r(23), r(29)],
    ]);
    let expected = (numerator.clone() / divisor.clone()).unwrap();
    let mut prepared = divisor.prepare_right_divisor();

    assert_eq!(
        numerator
            .clone()
            .div_matrix_with_prepared(&mut prepared)
            .unwrap(),
        expected
    );
    let mut prepared = divisor.prepare_right_divisor();
    assert_eq!(
        numerator
            .clone()
            .div_matrix_checked_with_prepared_with_abort(&mut prepared, &abort_signal())
            .unwrap(),
        expected
    );
}

#[test]
fn matrix3_known_uniform_scale_inverse_matches_matrix_inverse() {
    let scale = r(6);
    let matrix = Matrix3::uniform_scale(scale);
    let expected = Matrix3::new([
        [frac(1, 6), r(0), r(0)],
        [r(0), frac(1, 6), r(0)],
        [r(0), r(0), frac(1, 6)],
    ]);

    assert_eq!(matrix.clone().reciprocal().unwrap(), expected);
    assert_eq!(Matrix3::uniform_scale_inverse(r(6)).unwrap(), expected);
}

#[test]
fn matrix3_known_diagonal_inverse_matches_matrix_inverse() {
    let diagonal = [r(2), r(3), r(5)];
    let matrix = Matrix3::diagonal(diagonal);
    let expected = Matrix3::new([
        [frac(1, 2), r(0), r(0)],
        [r(0), frac(1, 3), r(0)],
        [r(0), r(0), frac(1, 5)],
    ]);

    assert_eq!(matrix.clone().reciprocal().unwrap(), expected);
    assert_eq!(
        Matrix3::diagonal_inverse([r(2), r(3), r(5)]).unwrap(),
        expected
    );
}

#[test]
fn matrix3_known_upper_triangular_inverse_matches_matrix_inverse() {
    let matrix = Matrix3::new([[r(2), r(3), r(5)], [r(0), r(7), r(11)], [r(0), r(0), r(13)]]);
    let expected = Matrix3::new([
        [frac(1, 2), frac(-3, 14), frac(-1, 91)],
        [r(0), frac(1, 7), frac(-11, 91)],
        [r(0), r(0), frac(1, 13)],
    ]);

    assert_eq!(matrix.clone().reciprocal().unwrap(), expected);
    assert_eq!(matrix.clone().upper_triangular_inverse().unwrap(), expected);
    assert_eq!(
        matrix.clone().upper_triangular_inverse_checked().unwrap(),
        expected
    );
    assert_eq!(matrix.clone().inverse_checked().unwrap(), expected);
    assert_eq!(
        matrix
            .clone()
            .upper_triangular_inverse_checked_with_abort(&abort_signal())
            .unwrap(),
        expected
    );
    assert_eq!(
        matrix.inverse_checked_with_abort(&abort_signal()).unwrap(),
        expected
    );
}

#[test]
fn matrix3_known_lower_triangular_inverse_matches_matrix_inverse() {
    let matrix = Matrix3::new([[r(2), r(0), r(0)], [r(3), r(5), r(0)], [r(7), r(11), r(13)]]);
    let expected = Matrix3::new([
        [frac(1, 2), r(0), r(0)],
        [frac(-3, 10), frac(1, 5), r(0)],
        [frac(-1, 65), frac(-11, 65), frac(1, 13)],
    ]);

    assert_eq!(matrix.clone().reciprocal().unwrap(), expected);
    assert_eq!(matrix.clone().lower_triangular_inverse().unwrap(), expected);
    assert_eq!(
        matrix.clone().lower_triangular_inverse_checked().unwrap(),
        expected
    );
    assert_eq!(matrix.clone().inverse_checked().unwrap(), expected);
    assert_eq!(
        matrix
            .clone()
            .lower_triangular_inverse_checked_with_abort(&abort_signal())
            .unwrap(),
        expected
    );
    assert_eq!(
        matrix.inverse_checked_with_abort(&abort_signal()).unwrap(),
        expected
    );
}

#[test]
fn matrix3_known_upper_triangular_inverse_checked_rejects_singular_divisor() {
    let matrix = Matrix3::new([[r(2), r(3), r(5)], [r(0), r(0), r(11)], [r(0), r(0), r(13)]]);

    assert_singular_error(matrix.clone().upper_triangular_inverse_checked());
    assert_singular_error(
        matrix
            .clone()
            .upper_triangular_inverse_checked_with_abort(&abort_signal()),
    );
    assert_singular_error(matrix.clone().inverse_checked());
    assert_singular_error(matrix.inverse_checked_with_abort(&abort_signal()));
}

#[test]
fn matrix3_known_lower_triangular_inverse_checked_rejects_singular_divisor() {
    let matrix = Matrix3::new([[r(2), r(0), r(0)], [r(3), r(11), r(0)], [r(5), r(13), r(0)]]);

    assert_singular_error(matrix.clone().lower_triangular_inverse_checked());
    assert_singular_error(
        matrix
            .clone()
            .lower_triangular_inverse_checked_with_abort(&abort_signal()),
    );
    assert_singular_error(matrix.clone().inverse_checked());
    assert_singular_error(matrix.inverse_checked_with_abort(&abort_signal()));
}

#[test]
fn matrix3_known_upper_triangular_div_matrix_matches_matrix_division() {
    let numerator = Matrix3::new([
        [r(2), r(6), r(15)],
        [r(10), r(12), r(25)],
        [r(14), r(18), r(35)],
    ]);
    let divisor = Matrix3::new([[r(2), r(3), r(5)], [r(0), r(7), r(11)], [r(0), r(0), r(13)]]);
    let expected = numerator.clone() * divisor.clone().reciprocal().unwrap();

    assert_eq!(
        numerator
            .clone()
            .div_upper_triangular(divisor.clone())
            .unwrap(),
        expected
    );
    assert_eq!(numerator.clone() / divisor.clone(), Ok(expected));
}

#[test]
fn matrix3_known_lower_triangular_div_matrix_matches_matrix_division() {
    let numerator = Matrix3::new([
        [r(2), r(6), r(15)],
        [r(10), r(12), r(25)],
        [r(14), r(18), r(35)],
    ]);
    let divisor = Matrix3::new([[r(2), r(0), r(0)], [r(3), r(5), r(0)], [r(7), r(11), r(13)]]);
    let expected = numerator.clone() * divisor.clone().reciprocal().unwrap();

    assert_eq!(
        numerator
            .clone()
            .div_lower_triangular(divisor.clone())
            .unwrap(),
        expected
    );
    assert_eq!(numerator.clone() / divisor.clone(), Ok(expected));
}

#[test]
fn matrix3_known_upper_triangular_div_matrix_checked_matches_ordinary() {
    let numerator = Matrix3::new([[r(3), r(1), r(4)], [r(1), r(5), r(9)], [r(2), r(6), r(5)]]);
    let divisor = Matrix3::new([[r(2), r(3), r(5)], [r(0), r(7), r(11)], [r(0), r(0), r(13)]]);
    let expected = numerator
        .clone()
        .div_matrix_checked(divisor.clone())
        .unwrap();
    let signal = abort_signal();

    let ordinary = (numerator.clone() / divisor.clone()).unwrap();
    assert_eq!(expected, ordinary);
    assert_eq!(
        numerator
            .clone()
            .div_upper_triangular_checked(divisor.clone())
            .unwrap(),
        ordinary
    );
    assert_eq!(
        numerator
            .div_upper_triangular_checked_with_abort(divisor, &signal)
            .unwrap(),
        ordinary,
    );
}

#[test]
fn matrix3_known_upper_triangular_div_matrix_checked_with_prepared_matches_ordinary() {
    let numerator = Matrix3::new([[r(4), r(2), r(7)], [r(1), r(3), r(0)], [r(5), r(8), r(6)]]);
    let divisor = Matrix3::new([[r(2), r(3), r(5)], [r(0), r(7), r(11)], [r(0), r(0), r(13)]]);
    let mut prepared = divisor.prepare_right_divisor();
    let expected = (numerator.clone() / divisor.clone()).unwrap();

    assert_eq!(
        numerator
            .clone()
            .div_matrix_with_prepared(&mut prepared)
            .unwrap(),
        expected
    );
    let mut prepared = divisor.prepare_right_divisor();
    let signal = abort_signal();
    assert_eq!(
        numerator
            .clone()
            .div_matrix_checked_with_prepared_with_abort(&mut prepared, &signal)
            .unwrap(),
        expected
    );
}

#[test]
fn matrix3_known_diagonal_division_matches_matrix_division() {
    let numerator = Matrix3::new([
        [r(2), r(6), r(15)],
        [r(10), r(12), r(25)],
        [r(14), r(18), r(35)],
    ]);
    let diagonal = [r(2), r(3), r(5)];
    let divisor = Matrix3::diagonal(diagonal.clone());

    assert_eq!(
        numerator.clone().div_diagonal(diagonal).unwrap(),
        (numerator / divisor).unwrap()
    );
}

#[test]
fn matrix3_known_diagonal_div_vector_matches_matrix_division() {
    let numerator = Matrix3::new([
        [r(2), r(6), r(15)],
        [r(10), r(12), r(25)],
        [r(14), r(18), r(35)],
    ]);
    let diagonal = [r(2), r(3), r(5)];
    let vector = Vector3::new([r(5), r(7), r(11)]);
    let divisor = Matrix3::diagonal(diagonal.clone());

    assert_eq!(
        numerator.div_diagonal_vector(diagonal, &vector).unwrap(),
        (numerator / divisor).unwrap() * vector
    );
}

#[test]
fn matrix3_division_solves_right_division() {
    let numerator = Matrix3::new([[r(3), r(1), r(4)], [r(1), r(5), r(9)], [r(2), r(6), r(5)]]);
    let divisor = Matrix3::new([[r(2), r(0), r(1)], [r(1), r(3), r(0)], [r(0), r(2), r(1)]]);

    let quotient = (numerator.clone() / divisor.clone()).unwrap();

    assert_eq!(quotient * divisor, numerator);
}

#[test]
fn checked_matrix3_division_matches_ordinary_right_division() {
    let numerator = Matrix3::new([[r(4), r(2), r(7)], [r(0), r(3), r(1)], [r(5), r(8), r(6)]]);
    let divisor = Matrix3::new([[r(1), r(2), r(0)], [r(0), r(1), r(3)], [r(2), r(0), r(1)]]);
    let signal = abort_signal();

    let ordinary = (numerator.clone() / divisor.clone()).unwrap();

    assert_eq!(
        numerator
            .clone()
            .div_matrix_checked(divisor.clone())
            .unwrap(),
        ordinary
    );
    assert_eq!(
        numerator
            .div_matrix_checked_with_abort(divisor, &signal)
            .unwrap(),
        ordinary
    );
}

#[test]
fn matrix3_division_with_prepared_divisor_reuses_cache() {
    let numerator1 = Matrix3::new([[r(3), r(1), r(4)], [r(1), r(5), r(9)], [r(2), r(6), r(5)]]);
    let numerator2 = Matrix3::new([[r(2), r(7), r(1)], [r(8), r(2), r(8)], [r(1), r(8), r(2)]]);
    let divisor = Matrix3::new([[r(2), r(0), r(1)], [r(1), r(3), r(0)], [r(0), r(2), r(1)]]);
    let expected1 = (numerator1.clone() / divisor.clone()).unwrap();
    let expected2 = (numerator2.clone() / divisor.clone()).unwrap();
    let mut prepared = divisor.prepare_right_divisor();

    assert_eq!(
        numerator1.div_matrix_with_prepared(&mut prepared).unwrap(),
        expected1
    );
    assert_eq!(
        numerator2.div_matrix_with_prepared(&mut prepared).unwrap(),
        expected2
    );
}

#[test]
fn matrix3_division_checked_and_checked_abort_with_prepared_divisor_match_ordinary() {
    let numerator = Matrix3::new([[r(4), r(2), r(7)], [r(0), r(3), r(1)], [r(5), r(8), r(6)]]);
    let divisor = Matrix3::new([[r(1), r(2), r(0)], [r(0), r(1), r(3)], [r(2), r(0), r(1)]]);
    let signal = abort_signal();
    let expected = (numerator.clone() / divisor.clone()).unwrap();
    let expected_checked = numerator
        .clone()
        .div_matrix_checked(divisor.clone())
        .unwrap();
    assert_eq!(expected, expected_checked);

    let mut prepared = divisor.prepare_right_divisor();
    assert_eq!(
        numerator
            .clone()
            .div_matrix_checked_with_prepared(&mut prepared)
            .unwrap(),
        expected
    );
    assert_eq!(
        numerator
            .clone()
            .div_matrix_checked_with_prepared_with_abort(&mut prepared, &signal)
            .unwrap(),
        expected_checked
    );
    assert_eq!(
        prepared.inverse().unwrap(),
        divisor.clone().inverse().unwrap()
    );
    assert_eq!(
        prepared.inverse_checked().unwrap(),
        divisor.clone().inverse_checked().unwrap()
    );
    assert_eq!(
        prepared.inverse_checked_with_abort(&signal).unwrap(),
        divisor.clone().inverse_checked_with_abort(&signal).unwrap()
    );

    let mut prepared = divisor.prepare_right_divisor();
    assert_eq!(
        prepared.reciprocal().unwrap(),
        divisor.clone().reciprocal().unwrap()
    );
    assert_eq!(
        prepared.reciprocal_checked().unwrap(),
        divisor.clone().reciprocal_checked().unwrap()
    );
    assert_eq!(
        prepared.reciprocal_checked_with_abort(&signal).unwrap(),
        divisor.clone().reciprocal_checked().unwrap()
    );
}

#[test]
fn matrix4_division_solves_right_division() {
    let numerator = Matrix4::new([
        [r(1), r(3), r(5), r(7)],
        [r(2), r(4), r(6), r(8)],
        [r(9), r(7), r(5), r(3)],
        [r(8), r(6), r(4), r(2)],
    ]);
    let divisor = Matrix4::new([
        [r(2), r(0), r(1), r(0)],
        [r(1), r(3), r(0), r(1)],
        [r(0), r(2), r(1), r(0)],
        [r(1), r(0), r(0), r(2)],
    ]);

    let quotient = (numerator.clone() / divisor.clone()).unwrap();

    assert_eq!(quotient * divisor, numerator);
}

#[test]
fn matrix4_division_with_prepared_divisor_reuses_cache() {
    let numerator1 = Matrix4::new([
        [r(1), r(2), r(3), r(4)],
        [r(5), r(6), r(7), r(8)],
        [r(9), r(10), r(11), r(12)],
        [r(13), r(14), r(15), r(16)],
    ]);
    let numerator2 = Matrix4::new([
        [r(2), r(4), r(6), r(8)],
        [r(10), r(12), r(14), r(16)],
        [r(18), r(20), r(22), r(24)],
        [r(26), r(28), r(30), r(32)],
    ]);
    let divisor = Matrix4::new([
        [r(2), r(0), r(1), r(0)],
        [r(1), r(3), r(0), r(1)],
        [r(0), r(2), r(1), r(0)],
        [r(1), r(0), r(0), r(2)],
    ]);
    let expected1 = (numerator1.clone() / divisor.clone()).unwrap();
    let expected2 = (numerator2.clone() / divisor.clone()).unwrap();
    let mut prepared = divisor.prepare_right_divisor();

    assert_eq!(
        numerator1.div_matrix_with_prepared(&mut prepared).unwrap(),
        expected1
    );
    assert_eq!(
        numerator2.div_matrix_with_prepared(&mut prepared).unwrap(),
        expected2
    );
}

#[test]
fn matrix4_division_checked_and_checked_abort_with_prepared_divisor_match_ordinary() {
    let numerator = Matrix4::new([
        [r(1), r(3), r(5), r(7)],
        [r(2), r(4), r(6), r(8)],
        [r(9), r(7), r(5), r(3)],
        [r(8), r(6), r(4), r(2)],
    ]);
    let divisor = Matrix4::new([
        [r(2), r(0), r(1), r(0)],
        [r(1), r(3), r(0), r(1)],
        [r(0), r(2), r(1), r(0)],
        [r(1), r(0), r(0), r(2)],
    ]);
    let signal = abort_signal();
    let expected = (numerator.clone() / divisor.clone()).unwrap();
    let expected_checked = numerator
        .clone()
        .div_matrix_checked(divisor.clone())
        .unwrap();
    assert_eq!(expected, expected_checked);

    let mut prepared = divisor.prepare_right_divisor();
    assert_eq!(
        numerator
            .clone()
            .div_matrix_checked_with_prepared(&mut prepared)
            .unwrap(),
        expected
    );
    assert_eq!(
        numerator
            .clone()
            .div_matrix_checked_with_prepared_with_abort(&mut prepared, &signal)
            .unwrap(),
        expected_checked
    );

    assert_eq!(
        prepared.inverse().unwrap(),
        divisor.clone().inverse().unwrap()
    );
    assert_eq!(
        prepared.inverse_checked().unwrap(),
        divisor.clone().inverse_checked().unwrap()
    );
    assert_eq!(
        prepared.inverse_checked_with_abort(&signal).unwrap(),
        divisor.clone().inverse_checked_with_abort(&signal).unwrap()
    );

    let mut prepared = divisor.prepare_right_divisor();
    assert_eq!(
        prepared.reciprocal().unwrap(),
        divisor.clone().reciprocal().unwrap()
    );
    assert_eq!(
        prepared.reciprocal_checked().unwrap(),
        divisor.clone().reciprocal_checked().unwrap()
    );
    assert_eq!(
        prepared.reciprocal_checked_with_abort(&signal).unwrap(),
        divisor.inverse_checked_with_abort(&signal).unwrap()
    );
}

#[test]
fn checked_matrix_inverse_rejects_unknown_zero_pivots() {
    let matrix = Matrix3::new([
        [unknown_zero(), r(0), r(0)],
        [r(0), r(1), r(0)],
        [r(0), r(0), r(1)],
    ]);
    let signal = abort_signal();

    assert_eq!(matrix.clone().inverse_checked(), Err(Problem::UnknownZero));
    assert_eq!(
        matrix.clone().inverse_checked_with_abort(&signal),
        Err(Problem::UnknownZero)
    );
    assert_eq!(
        Matrix3::identity().div_matrix_checked_with_abort(matrix, &signal),
        Err(Problem::UnknownZero)
    );
}

#[test]
fn checked_matrix_scalar_division_accepts_abort_signal() {
    let matrix = Matrix3::new([
        [r(2), r(4), r(6)],
        [r(8), r(10), r(12)],
        [r(14), r(16), r(18)],
    ]);
    let signal = abort_signal();

    assert_eq!(
        matrix.div_scalar_checked_with_abort(r(2), &signal).unwrap(),
        Matrix3::new([[r(1), r(2), r(3)], [r(4), r(5), r(6)], [r(7), r(8), r(9)]])
    );
}

#[cfg(not(feature = "hyperreal-backend"))]
#[test]
fn ordinary_matrix_scalar_division_returns_unknown_zero() {
    let divisor = hyperlattice::Scalar::approx(0.0, 0.25).unwrap();

    assert_eq!(Matrix3::identity() / divisor, Err(Problem::UnknownZero));
}

#[test]
fn ordinary_matrix_inverse_prefers_known_nonzero_pivots() {
    let matrix = Matrix3::new([
        [unknown_zero(), r(0), r(1)],
        [r(1), r(1), r(0)],
        [r(0), r(1), r(1)],
    ]);

    assert!(matrix.inverse().is_ok());
}

#[cfg(feature = "hyperreal-backend")]
#[test]
fn hyperreal_matrix_transform_preserves_exact_rational_facts() {
    let permutation = Matrix3::new([
        [frac(0, 1), frac(1, 1), frac(0, 1)],
        [frac(1, 1), frac(0, 1), frac(0, 1)],
        [frac(0, 1), frac(0, 1), frac(1, 1)],
    ]);
    let input = Vector3::new([frac(1, 2), frac(-3, 4), frac(5, 6)]);
    let output = permutation * input;

    assert_eq!(output, Vector3::new([frac(-3, 4), frac(1, 2), frac(5, 6)]));
    assert!(output[0].structural_facts().exact_rational);
    assert!(output[1].structural_facts().exact_rational);
    assert!(output[2].structural_facts().exact_rational);
    assert_eq!(
        output[0].structural_facts().sign,
        Some(ScalarSign::Negative)
    );
    assert_eq!(
        output[1].structural_facts().sign,
        Some(ScalarSign::Positive)
    );
    assert_eq!(
        output[2].structural_facts().sign,
        Some(ScalarSign::Positive)
    );
    assert_eq!(output[0].zero_status(), ZeroStatus::NonZero);
    assert_eq!(output[1].zero_status(), ZeroStatus::NonZero);
}

#[cfg(feature = "hyperreal-backend")]
#[test]
fn hyperreal_matrix_transform_identity_preserves_facts() {
    let identity = Matrix3::new([
        [frac(1, 1), frac(0, 1), frac(0, 1)],
        [frac(0, 1), frac(1, 1), frac(0, 1)],
        [frac(0, 1), frac(0, 1), frac(1, 1)],
    ]);
    let input = Vector3::new([frac(3, 4), frac(-2, 3), frac(0, 1)]);

    let output = identity * input.clone();

    assert_eq!(output, input);
    assert!(output[0].structural_facts().exact_rational);
    assert!(output[1].structural_facts().exact_rational);
    assert!(output[2].structural_facts().exact_rational);
    assert_eq!(
        output[0].structural_facts().sign,
        Some(ScalarSign::Positive)
    );
    assert_eq!(
        output[1].structural_facts().sign,
        Some(ScalarSign::Negative)
    );
    assert_eq!(output[2].structural_facts().sign, Some(ScalarSign::Zero));
}

#[cfg(feature = "hyperreal-backend")]
#[test]
fn hyperreal_matrix_transform_diagonal_and_translation_semantics() {
    let scale_translate = Matrix4::new([
        [frac(-3, 2), frac(0, 1), frac(0, 1), frac(0, 1)],
        [frac(0, 1), frac(4, 1), frac(0, 1), frac(0, 1)],
        [frac(0, 1), frac(0, 1), frac(0, 1), frac(0, 1)],
        [frac(0, 1), frac(0, 1), frac(0, 1), frac(1, 1)],
    ]);
    let input = Vector4::new([frac(2, 3), frac(-1, 2), frac(-9, 1), frac(1, 1)]);

    let output = scale_translate * input;

    assert_eq!(
        output,
        Vector4::new([frac(-1, 1), frac(-2, 1), frac(0, 1), frac(1, 1)]),
    );
    assert!(output[0].structural_facts().exact_rational);
    assert!(output[1].structural_facts().exact_rational);
    assert!(output[2].structural_facts().exact_rational);
    assert!(output[3].structural_facts().exact_rational);
    assert_eq!(
        output[0].structural_facts().sign,
        Some(ScalarSign::Negative)
    );
    assert_eq!(
        output[1].structural_facts().sign,
        Some(ScalarSign::Negative)
    );
    assert_eq!(output[2].structural_facts().sign, Some(ScalarSign::Zero));
    assert_eq!(
        output[3].structural_facts().sign,
        Some(ScalarSign::Positive)
    );
    assert_eq!(output[2].zero_status(), ZeroStatus::Zero);
}

#[cfg(feature = "hyperreal-backend")]
#[test]
fn hyperreal_matrix_transform_preserves_homogeneous_direction_points_semantics() {
    let translation = Matrix4::new([
        [frac(1, 1), frac(0, 1), frac(0, 1), frac(11, 10)],
        [frac(0, 1), frac(1, 1), frac(0, 1), frac(-3, 2)],
        [frac(0, 1), frac(0, 1), frac(1, 1), frac(7, 5)],
        [frac(0, 1), frac(0, 1), frac(0, 1), frac(1, 1)],
    ]);
    let point = Vector4::new([frac(6, 1), frac(7, 1), frac(8, 1), frac(1, 1)]);
    let direction = Vector4::new([frac(6, 1), frac(7, 1), frac(8, 1), frac(0, 1)]);

    let translated_point = translation.clone() * point.clone();
    let transformed_direction = translation * direction.clone();

    assert_eq!(
        translated_point,
        Vector4::new([frac(71, 10), frac(11, 2), frac(47, 5), frac(1, 1)])
    );
    assert_eq!(transformed_direction, direction);
    for index in 0..4 {
        assert!(translated_point[index].structural_facts().exact_rational);
        assert!(
            transformed_direction[index]
                .structural_facts()
                .exact_rational
        );
    }
    assert_eq!(
        transformed_direction[3].structural_facts().zero,
        ZeroStatus::Zero
    );
    assert_eq!(
        translated_point[3].structural_facts().zero,
        ZeroStatus::NonZero
    );
}

#[cfg(feature = "hyperreal-backend")]
#[test]
fn hyperreal_matrix_transform_direction_zero_lane_facts() {
    let linear_matrix = Matrix4::new([
        [frac(-3, 2), frac(0, 1), frac(7, 2), frac(11, 10)],
        [frac(0, 1), frac(0, 1), frac(0, 1), frac(-3, 2)],
        [frac(0, 1), frac(4, 1), frac(0, 1), frac(7, 5)],
        [frac(0, 1), frac(0, 1), frac(0, 1), frac(1, 1)],
    ]);
    let direction = Vector4::new([frac(0, 1), frac(7, 1), frac(0, 1), frac(0, 1)]);

    let transformed_direction = linear_matrix * direction;

    // Keep zero-lane intent explicit so future constructor refactors
    // don’t silently broaden symbolic structure on known direction inputs.
    assert_eq!(transformed_direction[0], frac(-3, 2) * frac(0, 1));
    assert_eq!(transformed_direction[1], frac(0, 1));
    assert_eq!(transformed_direction[2], frac(28, 1));
    assert_eq!(transformed_direction[3], frac(0, 1));
    assert_eq!(transformed_direction[0].zero_status(), ZeroStatus::Zero);
    assert_eq!(transformed_direction[1].zero_status(), ZeroStatus::Zero);
    assert_eq!(transformed_direction[3].zero_status(), ZeroStatus::Zero);
    assert_eq!(transformed_direction[2].zero_status(), ZeroStatus::NonZero);
}

#[cfg(feature = "hyperreal-backend")]
#[test]
fn hyperreal_matrix_transform_symbolic_no_translation_zero_lane_facts() {
    let symbolic_linear = Matrix4::new([
        [Scalar::pi(), frac(0, 1), Scalar::e(), frac(0, 1)],
        [frac(0, 1), frac(0, 1), frac(0, 1), frac(0, 1)],
        [frac(0, 1), frac(-2, 1), frac(0, 1), frac(0, 1)],
        [frac(0, 1), frac(0, 1), frac(0, 1), frac(1, 1)],
    ]);
    let symbolic_direction = Vector4::new([frac(3, 1), frac(7, 1), frac(0, 1), frac(0, 1)]);

    let transformed = symbolic_linear * symbolic_direction;

    // Symbolic no-translation path should keep this as a pure 3-term directional
    // transform and preserve lane-level public facts.
    // This guards against future constructor changes that would accidentally
    // materialize symbolic terms as full 4-lane work and erase demand-driven
    // zero/sign separation.
    assert_eq!(transformed[0].zero_status(), ZeroStatus::NonZero);
    assert_eq!(transformed[1].zero_status(), ZeroStatus::Zero);
    assert_eq!(transformed[2].zero_status(), ZeroStatus::NonZero);
    assert_eq!(transformed[3].zero_status(), ZeroStatus::Zero);
    assert_eq!(
        transformed[0].structural_facts().sign,
        Some(ScalarSign::Positive)
    );
    assert_eq!(
        transformed[1].structural_facts().sign,
        Some(ScalarSign::Zero)
    );
    assert_eq!(
        transformed[2].structural_facts().sign,
        Some(ScalarSign::Negative)
    );
    assert!(!transformed[0].structural_facts().exact_rational);
    assert!(transformed[2].structural_facts().exact_rational);
}

#[cfg(feature = "hyperreal-backend")]
#[test]
fn hyperreal_matrix_transform_propagates_zero_and_sign() {
    let matrix = Matrix3::new([
        [frac(-2, 1), frac(0, 1), frac(0, 1)],
        [frac(0, 1), frac(0, 1), frac(0, 1)],
        [frac(0, 1), frac(0, 1), frac(5, 2)],
    ]);
    let input = Vector3::new([frac(3, 1), frac(7, 1), frac(-4, 1)]);

    let transformed = matrix * input;

    assert_eq!(
        transformed,
        Vector3::new([frac(-6, 1), frac(0, 1), frac(-10, 1)])
    );
    assert_eq!(
        transformed[0].structural_facts().sign,
        Some(ScalarSign::Negative)
    );
    assert_eq!(
        transformed[1].structural_facts().sign,
        Some(ScalarSign::Zero)
    );
    assert_eq!(
        transformed[2].structural_facts().sign,
        Some(ScalarSign::Negative)
    );
    assert_eq!(transformed[0].zero_status(), ZeroStatus::NonZero);
    assert_eq!(transformed[1].zero_status(), ZeroStatus::Zero);
    assert_eq!(transformed[2].zero_status(), ZeroStatus::NonZero);
}

#[test]
fn matrix_transform_batch_matches_pointwise_transform() {
    let matrix3 = Matrix3::new([[r(1), r(2), r(3)], [r(0), r(-2), r(1)], [r(-1), r(4), r(2)]]);
    let vectors3 = [
        Vector3::new([r(3), r(0), r(-2)]),
        Vector3::new([r(-4), r(5), r(7)]),
    ];
    let expected3 = [
        matrix3.clone() * vectors3[0].clone(),
        matrix3.clone() * vectors3[1].clone(),
    ];

    assert_eq!(matrix3.transform_vec3_batch(&vectors3), expected3);

    let matrix4 = Matrix4::new([
        [r(1), r(2), r(3), r(1)],
        [r(0), r(-2), r(1), r(4)],
        [r(-1), r(4), r(2), r(3)],
        [r(0), r(0), r(0), r(1)],
    ]);
    let vectors4 = [
        Vector4::new([r(3), r(0), r(-2), r(1)]),
        Vector4::new([r(-4), r(5), r(7), r(1)]),
        Vector4::new([r(2), r(7), r(-3), r(0)]),
    ];
    let expected4 = [
        matrix4.clone() * vectors4[0].clone(),
        matrix4.clone() * vectors4[1].clone(),
        matrix4.clone() * vectors4[2].clone(),
    ];

    assert_eq!(matrix4.transform_vec4_batch(&vectors4), expected4);
}

#[test]
fn matrix_transform_handles_materialize_equivalent_to_transform() {
    let matrix3 = Matrix3::new([[r(1), r(2), r(3)], [r(0), r(-1), r(4)], [r(-2), r(5), r(7)]]);
    let vector3 = Vector3::new([r(3), r(0), r(-2)]);
    let matrix3_handle = matrix3.transform_vec3_handle();
    let vector3_handle = matrix3_handle.vector(&vector3);

    assert_eq!(
        matrix3_handle.transform_vector(&vector3),
        matrix3.clone() * vector3.clone()
    );
    assert_eq!(
        vector3_handle.materialize(),
        matrix3.clone() * vector3.clone()
    );
    let vector3_with = matrix3.transform_vec3_with(&vector3);
    assert_eq!(vector3_with.materialize(), matrix3 * vector3);

    let matrix4 = Matrix4::new([
        [r(1), r(2), r(3), r(4)],
        [r(0), r(-1), r(5), r(6)],
        [r(7), r(8), r(9), r(2)],
        [r(0), r(0), r(0), r(1)],
    ]);
    let vector4 = Vector4::new([r(3), r(0), r(-2), r(1)]);
    let matrix4_handle = matrix4.transform_vec4_handle();
    let vector4_handle = matrix4_handle.vector(&vector4);

    assert_eq!(
        matrix4_handle.transform_vector(&vector4),
        matrix4.clone() * vector4.clone()
    );
    assert_eq!(
        vector4_handle.materialize(),
        matrix4.clone() * vector4.clone()
    );
    let vector4_with = matrix4.transform_vec4_with(&vector4);
    assert_eq!(vector4_with.materialize(), matrix4 * vector4);
}
