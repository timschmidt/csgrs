use exactcore_hyper_comparison::{
    ComplexBinary, LinearBinary, area2, complex_binary, complex_norm_squared, matrix_adjugate,
    matrix_binary, matrix_determinant, matrix_transpose, vector_binary, vector_cross3, vector_dot,
    vector_norm,
};
use hyperlattice::{Complex, Matrix3, Matrix4, Rational, Real, Vector2, Vector3, Vector4};
use proptest::prelude::*;

fn r(value: i64) -> Real {
    value.into()
}

fn rr(text: &str) -> Real {
    Real::new(text.parse::<Rational>().unwrap())
}

fn exact(value: &Real) -> Rational {
    value
        .exact_rational_normal_form()
        .unwrap_or_else(|| panic!("expected exact rational, got {value}"))
}

fn assert_exact_text(value: &Real, expected: &str, context: &str) {
    assert_eq!(
        exact(value),
        expected.parse::<Rational>().unwrap(),
        "{context}"
    );
}

fn assert_close(left: f64, right: f64, context: &str) {
    let scale = 1.0_f64.max(left.abs()).max(right.abs());
    assert!(
        left.is_finite() && right.is_finite() && (left - right).abs() <= 2.0e-12 * scale,
        "{context}: exactCore={left:.17e}, Hyperlattice={right:.17e}"
    );
}

fn assert_real_vector(core: &[f64], hyper: &[Real], context: &str) {
    assert_eq!(core.len(), hyper.len());
    for (index, (core, hyper)) in core.iter().zip(hyper).enumerate() {
        assert_close(
            *core,
            hyper.to_f64_lossy().unwrap(),
            &format!("{context}[{index}]"),
        );
    }
}

fn matrix3(values: [i64; 9]) -> Matrix3 {
    Matrix3::new([
        [r(values[0]), r(values[1]), r(values[2])],
        [r(values[3]), r(values[4]), r(values[5])],
        [r(values[6]), r(values[7]), r(values[8])],
    ])
}

fn matrix4(values: [i64; 16]) -> Matrix4 {
    Matrix4::new([
        [r(values[0]), r(values[1]), r(values[2]), r(values[3])],
        [r(values[4]), r(values[5]), r(values[6]), r(values[7])],
        [r(values[8]), r(values[9]), r(values[10]), r(values[11])],
        [r(values[12]), r(values[13]), r(values[14]), r(values[15])],
    ])
}

fn flatten3(matrix: &Matrix3) -> Vec<Real> {
    matrix.0.iter().flatten().cloned().collect()
}

fn flatten4(matrix: &Matrix4) -> Vec<Real> {
    matrix.0.iter().flatten().cloned().collect()
}

#[test]
fn complex_cartesian_arithmetic_and_norm_match() {
    let cases = [
        (("1/3", "-2/5"), ("7/11", "13/17")),
        (("0", "1"), ("1", "0")),
        (("123/97", "-41/53"), ("-19/29", "23/31")),
    ];

    for (left, right) in cases {
        let a = Complex::new(rr(left.0), rr(left.1));
        let b = Complex::new(rr(right.0), rr(right.1));
        let operations = [
            (ComplexBinary::Add, &a + &b),
            (ComplexBinary::Subtract, &a - &b),
            (ComplexBinary::Multiply, &a * &b),
            (ComplexBinary::Divide, (&a / &b).unwrap()),
        ];

        for (operation, hyper) in operations {
            let (core_re, core_im) = complex_binary(operation, left, right).unwrap();
            assert_exact_text(&hyper.re, &core_re, &format!("{operation:?}.re"));
            assert_exact_text(&hyper.im, &core_im, &format!("{operation:?}.im"));
        }

        let core_norm = complex_norm_squared(left.0, left.1).unwrap();
        assert_exact_text(&a.norm_squared(), &core_norm, "complex norm squared");
    }
}

#[test]
fn vector_arithmetic_dot_norm_wedge_and_cross_match() {
    let left2 = [3, -4];
    let right2 = [7, 11];
    let a2 = Vector2::new(left2.map(r));
    let b2 = Vector2::new(right2.map(r));
    assert_real_vector(
        &vector_binary(LinearBinary::Add, &left2, &right2),
        &(&a2 + &b2).0,
        "vector2 add",
    );
    assert_real_vector(
        &vector_binary(LinearBinary::Subtract, &left2, &right2),
        &(&a2 - &b2).0,
        "vector2 subtract",
    );
    assert_close(
        vector_dot(&left2, &right2),
        a2.dot(&b2).to_f64_lossy().unwrap(),
        "vector2 dot",
    );
    assert_close(
        vector_norm(&left2),
        a2.norm().to_f64_lossy().unwrap(),
        "vector2 norm",
    );
    assert_close(
        area2([0, 0, left2[0], left2[1], right2[0], right2[1]]),
        a2.wedge(&b2).to_f64_lossy().unwrap(),
        "vector2 wedge",
    );

    let left3 = [3, -4, 5];
    let right3 = [7, 11, -13];
    let a3 = Vector3::new(left3.map(r));
    let b3 = Vector3::new(right3.map(r));
    assert_real_vector(
        &vector_binary(LinearBinary::Add, &left3, &right3),
        &(&a3 + &b3).0,
        "vector3 add",
    );
    assert_real_vector(
        &vector_binary(LinearBinary::Subtract, &left3, &right3),
        &(&a3 - &b3).0,
        "vector3 subtract",
    );
    assert_close(
        vector_dot(&left3, &right3),
        a3.dot(&b3).to_f64_lossy().unwrap(),
        "vector3 dot",
    );
    assert_close(
        vector_norm(&left3),
        a3.norm().to_f64_lossy().unwrap(),
        "vector3 norm",
    );
    assert_real_vector(
        &vector_cross3(left3, right3),
        &a3.cross(&b3).0,
        "vector3 cross",
    );

    let left4 = [3, -4, 5, -6];
    let right4 = [7, 11, -13, 17];
    let a4 = Vector4::new(left4.map(r));
    let b4 = Vector4::new(right4.map(r));
    assert_real_vector(
        &vector_binary(LinearBinary::Add, &left4, &right4),
        &(&a4 + &b4).0,
        "vector4 add",
    );
    assert_real_vector(
        &vector_binary(LinearBinary::Subtract, &left4, &right4),
        &(&a4 - &b4).0,
        "vector4 subtract",
    );
    assert_close(
        vector_dot(&left4, &right4),
        a4.dot(&b4).to_f64_lossy().unwrap(),
        "vector4 dot",
    );
    assert_close(
        vector_norm(&left4),
        a4.norm().to_f64_lossy().unwrap(),
        "vector4 norm",
    );
}

#[test]
fn vector_squared_norm_and_zero_vectors_match() {
    let vectors: [&[i64]; 4] = [&[0, 0], &[0, 0, 0], &[3, 4, 0], &[1, -2, 3, -4]];
    for values in vectors {
        let core_squared = vector_dot(values, values);
        let core_norm = vector_norm(values);
        let (hyper_squared, hyper_norm) = match values.len() {
            2 => {
                let vector = Vector2::new([r(values[0]), r(values[1])]);
                (vector.norm_squared(), vector.norm())
            }
            3 => {
                let vector = Vector3::new([r(values[0]), r(values[1]), r(values[2])]);
                (vector.norm_squared(), vector.norm())
            }
            4 => {
                let vector = Vector4::new([r(values[0]), r(values[1]), r(values[2]), r(values[3])]);
                (vector.norm_squared(), vector.norm())
            }
            _ => unreachable!(),
        };
        assert_close(
            core_squared,
            hyper_squared.to_f64_lossy().unwrap(),
            "squared norm",
        );
        assert_close(core_norm, hyper_norm.to_f64_lossy().unwrap(), "norm");
    }
}

fn check_matrix3(left: [i64; 9], right: [i64; 9]) {
    let a = matrix3(left);
    let b = matrix3(right);
    for (operation, hyper) in [
        (LinearBinary::Add, &a + &b),
        (LinearBinary::Subtract, &a - &b),
        (LinearBinary::Multiply, &a * &b),
    ] {
        assert_real_vector(
            &matrix_binary(operation, &left, &right),
            &flatten3(&hyper),
            &format!("matrix3 {operation:?}"),
        );
    }
    assert_real_vector(
        &matrix_transpose(&left),
        &flatten3(&a.transpose()),
        "matrix3 transpose",
    );
    assert_close(
        matrix_determinant(&left),
        a.determinant().to_f64_lossy().unwrap(),
        "matrix3 determinant",
    );

    let (core_det, core_adjugate) = matrix_adjugate(&left);
    if core_det != 0.0 {
        let inverse = a.inverse().unwrap();
        let expected_inverse: Vec<f64> =
            core_adjugate.iter().map(|value| value / core_det).collect();
        assert_real_vector(&expected_inverse, &flatten3(&inverse), "matrix3 inverse");
    } else {
        assert!(a.inverse().is_err());
    }
}

fn check_matrix4(left: [i64; 16], right: [i64; 16]) {
    let a = matrix4(left);
    let b = matrix4(right);
    for (operation, hyper) in [
        (LinearBinary::Add, &a + &b),
        (LinearBinary::Subtract, &a - &b),
        (LinearBinary::Multiply, &a * &b),
    ] {
        assert_real_vector(
            &matrix_binary(operation, &left, &right),
            &flatten4(&hyper),
            &format!("matrix4 {operation:?}"),
        );
    }
    assert_real_vector(
        &matrix_transpose(&left),
        &flatten4(&a.transpose()),
        "matrix4 transpose",
    );
    assert_close(
        matrix_determinant(&left),
        a.determinant().to_f64_lossy().unwrap(),
        "matrix4 determinant",
    );

    let (core_det, core_adjugate) = matrix_adjugate(&left);
    if core_det != 0.0 {
        let inverse = a.inverse().unwrap();
        let expected_inverse: Vec<f64> =
            core_adjugate.iter().map(|value| value / core_det).collect();
        assert_real_vector(&expected_inverse, &flatten4(&inverse), "matrix4 inverse");
    } else {
        assert!(a.inverse().is_err());
    }
}

#[test]
fn matrix3_operations_determinant_and_inverse_match() {
    check_matrix3([2, -1, 3, 4, 0, 5, -2, 7, 1], [1, 2, 0, -3, 4, 1, 5, -2, 6]);
    check_matrix3([1, 2, 3, 2, 4, 6, 7, 8, 9], [9, 8, 7, 6, 5, 4, 3, 2, 1]);
}

#[test]
fn matrix4_operations_determinant_and_inverse_match() {
    check_matrix4(
        [2, 1, 0, 3, -1, 4, 2, 0, 5, 0, 3, 1, 2, -2, 1, 6],
        [1, 0, 2, -1, 3, 1, 0, 4, -2, 5, 1, 0, 0, 2, 3, 1],
    );
    check_matrix4(
        [1, 2, 3, 4, 2, 4, 6, 8, 0, 1, 0, 1, 5, 6, 7, 8],
        [8, 7, 6, 5, 4, 3, 2, 1, 1, 0, 1, 0, 2, 3, 4, 5],
    );
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(128))]

    #[test]
    fn vector3_integer_properties_match(left in prop::array::uniform3(-100_i64..=100), right in prop::array::uniform3(-100_i64..=100)) {
        let a = Vector3::new(left.map(r));
        let b = Vector3::new(right.map(r));
        prop_assert_eq!(vector_dot(&left, &right), a.dot(&b).to_f64_lossy().unwrap());
        prop_assert_eq!(vector_cross3(left, right), a.cross(&b).0.map(|value| value.to_f64_lossy().unwrap()));
        prop_assert_eq!(vector_binary(LinearBinary::Add, &left, &right), (&a + &b).0.map(|value| value.to_f64_lossy().unwrap()));
        prop_assert_eq!(vector_binary(LinearBinary::Subtract, &left, &right), (&a - &b).0.map(|value| value.to_f64_lossy().unwrap()));
    }

    #[test]
    fn matrix3_integer_determinants_match(values in prop::array::uniform9(-12_i64..=12)) {
        // CORE's legacy Bareiss implementation performs untracked column swaps.
        // Exercise its correctly implemented no-swap path here; the swap defect
        // is retained as a separate ignored characterization test below.
        prop_assume!(values[0] != 0);
        prop_assume!(values[0] * values[4] - values[1] * values[3] != 0);
        let core = matrix_determinant(&values);
        let hyper = matrix3(values).determinant().to_f64_lossy().unwrap();
        prop_assert_eq!(core, hyper);
    }
}

#[test]
#[ignore = "known exactCorelib Bareiss column-pivot defect; run with --ignored to characterize"]
fn exactcore_matrix_determinant_loses_column_swap_sign_and_position() {
    let values = [0, 0, -1, 0, 1, 0, 1, 0, 0];
    let core = matrix_determinant(&values);
    let hyper = matrix3(values).determinant().to_f64_lossy().unwrap();
    assert_eq!(
        core, hyper,
        "exactCore currently returns 0 while the determinant is 1"
    );
}
