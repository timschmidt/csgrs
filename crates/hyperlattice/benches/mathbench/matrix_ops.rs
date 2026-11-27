#[allow(clippy::needless_range_loop)]
fn bench_matrix_operations_for<F>(
    group: &mut BenchmarkGroup<'_, criterion::measurement::WallTime>,
    label: &str,
    make_scalar: F,
) where
    F: Copy + Fn(f64) -> Real,
{
    let lhs3_cases = sample_mat3_cases().map(|value| blas_mat3_with(value, make_scalar));
    let rhs3_cases = sample_mat3_b_cases().map(|value| blas_mat3_with(value, make_scalar));
    let lhs3_affine_cases = sample_mat3_affine_cases().map(|value| blas_mat3_with(value, make_scalar));
    let rhs3_affine_cases = sample_mat3_affine_b_cases().map(|value| blas_mat3_with(value, make_scalar));
    let lhs3_affine_translation_cases =
        sample_mat3_affine_translation_cases().map(|value| blas_mat3_with(value, make_scalar));
    let rhs3_affine_translation_cases =
        sample_mat3_affine_translation_b_cases().map(|value| blas_mat3_with(value, make_scalar));
    let lhs4_cases = sample_mat4_cases().map(|value| blas_mat4_with(value, make_scalar));
    let rhs4_cases = sample_mat4_b_cases().map(|value| blas_mat4_with(value, make_scalar));
    let lhs4_affine_cases = sample_mat4_affine_cases().map(|value| blas_mat4_with(value, make_scalar));
    let rhs4_affine_cases = sample_mat4_affine_b_cases().map(|value| blas_mat4_with(value, make_scalar));
    let lhs4_affine_translation_cases =
        sample_mat4_affine_translation_cases().map(|value| blas_mat4_with(value, make_scalar));
    let rhs4_affine_translation_cases =
        sample_mat4_affine_translation_b_cases().map(|value| blas_mat4_with(value, make_scalar));
    let vector3_cases = sample_vec3_cases().map(|value| blas_vec3_with(value, make_scalar));
    let vector4_cases = sample_vec4_cases().map(|value| blas_vec4_with(value, make_scalar));
    let uniform_scale_matrix3 = Matrix3::new([
        [make_scalar(2.0), make_scalar(0.0), make_scalar(0.0)],
        [make_scalar(0.0), make_scalar(2.0), make_scalar(0.0)],
        [make_scalar(0.0), make_scalar(0.0), make_scalar(2.0)],
    ]);
    let upper_triangular_matrix3 = Matrix3::new([
        [make_scalar(2.0), make_scalar(3.0), make_scalar(5.0)],
        [make_scalar(0.0), make_scalar(7.0), make_scalar(11.0)],
        [make_scalar(0.0), make_scalar(0.0), make_scalar(13.0)],
    ]);
    let lower_triangular_matrix3 = Matrix3::new([
        [make_scalar(2.0), make_scalar(0.0), make_scalar(0.0)],
        [make_scalar(3.0), make_scalar(5.0), make_scalar(0.0)],
        [make_scalar(7.0), make_scalar(11.0), make_scalar(13.0)],
    ]);
    let upper_triangular_matrix4 = Matrix4::new([
        [make_scalar(2.0), make_scalar(3.0), make_scalar(5.0), make_scalar(7.0)],
        [make_scalar(0.0), make_scalar(11.0), make_scalar(13.0), make_scalar(17.0)],
        [make_scalar(0.0), make_scalar(0.0), make_scalar(19.0), make_scalar(23.0)],
        [make_scalar(0.0), make_scalar(0.0), make_scalar(0.0), make_scalar(29.0)],
    ]);
    let lower_triangular_matrix4 = Matrix4::new([
        [make_scalar(2.0), make_scalar(0.0), make_scalar(0.0), make_scalar(0.0)],
        [make_scalar(3.0), make_scalar(11.0), make_scalar(0.0), make_scalar(0.0)],
        [make_scalar(5.0), make_scalar(13.0), make_scalar(19.0), make_scalar(0.0)],
        [make_scalar(7.0), make_scalar(17.0), make_scalar(23.0), make_scalar(29.0)],
    ]);
    let diagonal_values3 = [make_scalar(2.0), make_scalar(3.0), make_scalar(5.0)];
    let translated_diagonal_direction_matrix = Matrix4::new([
        [make_scalar(2.0), make_scalar(0.0), make_scalar(0.0), make_scalar(100.0)],
        [make_scalar(0.0), make_scalar(3.0), make_scalar(0.0), make_scalar(200.0)],
        [make_scalar(0.0), make_scalar(0.0), make_scalar(4.0), make_scalar(300.0)],
        [make_scalar(0.0), make_scalar(0.0), make_scalar(0.0), make_scalar(1.0)],
    ]);
    let diagonal_affine_matrix = Matrix4::new([
        [make_scalar(2.0), make_scalar(0.0), make_scalar(0.0), make_scalar(0.0)],
        [make_scalar(0.0), make_scalar(3.0), make_scalar(0.0), make_scalar(0.0)],
        [make_scalar(0.0), make_scalar(0.0), make_scalar(4.0), make_scalar(0.0)],
        [make_scalar(0.0), make_scalar(0.0), make_scalar(0.0), make_scalar(1.0)],
    ]);
    let uniform_scale_matrix4 = Matrix4::new([
        [make_scalar(2.0), make_scalar(0.0), make_scalar(0.0), make_scalar(0.0)],
        [make_scalar(0.0), make_scalar(2.0), make_scalar(0.0), make_scalar(0.0)],
        [make_scalar(0.0), make_scalar(0.0), make_scalar(2.0), make_scalar(0.0)],
        [make_scalar(0.0), make_scalar(0.0), make_scalar(0.0), make_scalar(2.0)],
    ]);
    let uniform_scale_value = make_scalar(2.0);
    let translation_values4 = [make_scalar(3.0), make_scalar(-5.0), make_scalar(7.0)];
    let orthonormal_linear4 = [
        [make_scalar(0.0), make_scalar(-1.0), make_scalar(0.0)],
        [make_scalar(1.0), make_scalar(0.0), make_scalar(0.0)],
        [make_scalar(0.0), make_scalar(0.0), make_scalar(1.0)],
    ];
    let signed_permutation_rows4 = [
        SignedAxis4::PosY,
        SignedAxis4::NegX,
        SignedAxis4::PosW,
        SignedAxis4::NegZ,
    ];
    let diagonal_values4 = [
        make_scalar(2.0),
        make_scalar(3.0),
        make_scalar(5.0),
        make_scalar(7.0),
    ];
    let identity_matrix4 = Matrix4::identity();
    let translated_diagonal_direction = Vector4::new([
        make_scalar(5.0),
        make_scalar(7.0),
        make_scalar(11.0),
        make_scalar(0.0),
    ]);
    let translated_diagonal_point = Vector4::new([
        make_scalar(5.0),
        make_scalar(7.0),
        make_scalar(11.0),
        make_scalar(1.0),
    ]);
    let translated_diagonal_direction_batch = [
        Vector4::new([make_scalar(5.0), make_scalar(7.0), make_scalar(11.0), make_scalar(0.0)]),
        Vector4::new([make_scalar(13.0), make_scalar(17.0), make_scalar(19.0), make_scalar(0.0)]),
        Vector4::new([make_scalar(23.0), make_scalar(29.0), make_scalar(31.0), make_scalar(0.0)]),
        Vector4::new([make_scalar(37.0), make_scalar(41.0), make_scalar(43.0), make_scalar(0.0)]),
    ];
    let translated_diagonal_point_batch = [
        Vector4::new([make_scalar(5.0), make_scalar(7.0), make_scalar(11.0), make_scalar(1.0)]),
        Vector4::new([make_scalar(13.0), make_scalar(17.0), make_scalar(19.0), make_scalar(1.0)]),
        Vector4::new([make_scalar(23.0), make_scalar(29.0), make_scalar(31.0), make_scalar(1.0)]),
        Vector4::new([make_scalar(37.0), make_scalar(41.0), make_scalar(43.0), make_scalar(1.0)]),
    ];
    let scalar_cases = [
        make_scalar(2.0),
        make_scalar(1.0e-9),
        make_scalar(-1.0e9),
        make_scalar(std::f64::consts::PI),
    ];
    let signal = abort_signal();

    let profile_input = match label {
        "hyperreal" => Some("from-f64"),
        "hyperreal-rational" => Some("rational"),
        _ => None,
    };
    if let Some(input) = profile_input {
        trace_matrix_profile_row("mat3", "reciprocal", input, lhs3_cases.len(), || {
            for value in &lhs3_cases {
                black_box(black_box(value.clone()).reciprocal().unwrap());
            }
        });
        trace_matrix_profile_row("mat3", "uniform_scale_reciprocal", input, 1, || {
            black_box(black_box(uniform_scale_matrix3.clone()).reciprocal().unwrap());
        });
        trace_matrix_profile_row("mat3", "known_uniform_scale_inverse", input, 1, || {
            black_box(
                Matrix3::uniform_scale_inverse(black_box(uniform_scale_value.clone()))
                    .unwrap(),
            );
        });
        trace_matrix_profile_row("mat3", "known_diagonal_inverse", input, 1, || {
            black_box(Matrix3::diagonal_inverse(black_box(diagonal_values3.clone())).unwrap());
        });
        trace_matrix_profile_row("mat3", "known_upper_triangular_inverse", input, 1, || {
            black_box(black_box(upper_triangular_matrix3.clone()).upper_triangular_inverse().unwrap());
        });
        trace_matrix_profile_row("mat3", "known_lower_triangular_inverse", input, 1, || {
            black_box(black_box(lower_triangular_matrix3.clone()).lower_triangular_inverse().unwrap());
        });
        // Keep checked triangular inverse rows in the same structural dispatch family
        // so we can validate the O(n²) branch under error-aware APIs.
        // See Golub and Van Loan, *Matrix Computations*; Yap, "Towards Exact
        // Geometric Computation", 1997.
        trace_matrix_profile_row("mat3", "known_upper_triangular_inverse_checked", input, 1, || {
            black_box(
                black_box(upper_triangular_matrix3.clone())
                    .upper_triangular_inverse_checked()
                    .unwrap(),
            );
        });
        trace_matrix_profile_row("mat3", "known_lower_triangular_inverse_checked", input, 1, || {
            black_box(
                black_box(lower_triangular_matrix3.clone())
                    .lower_triangular_inverse_checked()
                    .unwrap(),
            );
        });
        trace_matrix_profile_row("mat3", "known_diagonal_div_matrix", input, 1, || {
            black_box(
                black_box(lhs3_cases[0].clone())
                    .div_diagonal(black_box(diagonal_values3.clone()))
                    .unwrap(),
            );
        });
        trace_matrix_profile_row("mat3", "known_diagonal_div_vector", input, 1, || {
            black_box(
                black_box(&lhs3_cases[0])
                    .div_diagonal_vector(
                        black_box(diagonal_values3.clone()),
                        black_box(&vector3_cases[0]),
                    )
                    .unwrap(),
            );
        });
        trace_matrix_profile_row("mat3", "known_uniform_diagonal_div_vector", input, 1, || {
            black_box(
                black_box(&lhs3_cases[0])
                    .div_diagonal_vector(
                        black_box([
                            uniform_scale_value.clone(),
                            uniform_scale_value.clone(),
                            uniform_scale_value.clone(),
                        ]),
                        black_box(&vector3_cases[0]),
                    )
                    .unwrap(),
            );
        });
        trace_matrix_profile_row("mat3", "reciprocal_checked", input, lhs3_cases.len(), || {
            for value in &lhs3_cases {
                black_box(black_box(value.clone()).reciprocal_checked().unwrap());
            }
        });
        trace_matrix_profile_row("mat3", "inverse_checked", input, lhs3_cases.len(), || {
            for value in &lhs3_cases {
                black_box(black_box(value.clone()).inverse_checked().unwrap());
            }
        });
        trace_matrix_profile_row(
            "mat3",
            "inverse_checked_abort",
            input,
            lhs3_cases.len(),
            || {
                for value in &lhs3_cases {
                    black_box(
                        black_box(value.clone())
                            .inverse_checked_with_abort(&signal)
                            .unwrap(),
                    );
                }
            },
        );
        trace_matrix_profile_row("mat3", "div_matrix", input, lhs3_cases.len(), || {
            for index in 0..lhs3_cases.len() {
                black_box(
                    (black_box(lhs3_cases[index].clone()) / black_box(rhs3_cases[index].clone()))
                        .unwrap(),
                );
            }
        });
        trace_matrix_profile_row("mat3", "div_matrix_checked", input, lhs3_cases.len(), || {
            for index in 0..lhs3_cases.len() {
                black_box(
                    black_box(lhs3_cases[index].clone())
                        .div_matrix_checked(black_box(rhs3_cases[index].clone()))
                        .unwrap(),
                );
            }
        });
        trace_matrix_profile_row(
            "mat3",
            "div_matrix_checked_abort",
            input,
            lhs3_cases.len(),
            || {
                for index in 0..lhs3_cases.len() {
                    black_box(
                        black_box(lhs3_cases[index].clone())
                            .div_matrix_checked_with_abort(
                                black_box(rhs3_cases[index].clone()),
                                &signal,
                            )
                            .unwrap(),
                    );
                }
            },
        );
        trace_matrix_profile_row("mat3", "direct_div_matrix", input, lhs3_cases.len(), || {
            let divisor = black_box(rhs3_cases[0].clone());
            for index in 0..lhs3_cases.len() {
                black_box(
                    black_box(lhs3_cases[index].clone())
                        .div_matrix_checked(black_box(divisor.clone()))
                        .unwrap(),
                );
            }
        });
        trace_matrix_profile_row(
            "mat3",
            "direct_div_matrix_checked",
            input,
            lhs3_cases.len(),
            || {
                let divisor = black_box(rhs3_cases[0].clone());
                for index in 0..lhs3_cases.len() {
                    black_box(
                        black_box(lhs3_cases[index].clone())
                            .div_matrix_checked(black_box(divisor.clone()))
                            .unwrap(),
                    );
                }
            },
        );
        trace_matrix_profile_row(
            "mat3",
            "direct_div_matrix_checked_abort",
            input,
            lhs3_cases.len(),
            || {
                let divisor = black_box(rhs3_cases[0].clone());
                for index in 0..lhs3_cases.len() {
                    black_box(
                        black_box(lhs3_cases[index].clone())
                            .div_matrix_checked_with_abort(black_box(divisor.clone()), &signal)
                            .unwrap(),
                    );
                }
            },
        );
        trace_matrix_profile_row("mat3", "direct_inverse", input, lhs3_cases.len(), || {
            let divisor = black_box(rhs3_cases[0].clone());
            for _ in 0..lhs3_cases.len() {
                black_box(divisor.clone().inverse().unwrap());
            }
        });
        trace_matrix_profile_row("mat3", "direct_reciprocal", input, lhs3_cases.len(), || {
            let divisor = black_box(rhs3_cases[0].clone());
            for _ in 0..lhs3_cases.len() {
                black_box(divisor.clone().reciprocal().unwrap());
            }
        });
        trace_matrix_profile_row(
            "mat3",
            "direct_inverse_checked",
            input,
            lhs3_cases.len(),
            || {
                let divisor = black_box(rhs3_cases[0].clone());
                for _ in 0..lhs3_cases.len() {
                    black_box(divisor.clone().inverse_checked().unwrap());
                }
            },
        );
        trace_matrix_profile_row(
            "mat3",
            "direct_reciprocal_checked",
            input,
            lhs3_cases.len(),
            || {
                let divisor = black_box(rhs3_cases[0].clone());
                for _ in 0..lhs3_cases.len() {
                    black_box(divisor.clone().reciprocal_checked().unwrap());
                }
            },
        );
        trace_matrix_profile_row(
            "mat3",
            "direct_inverse_checked_abort",
            input,
            lhs3_cases.len(),
            || {
                let divisor = black_box(rhs3_cases[0].clone());
                for _ in 0..lhs3_cases.len() {
                    black_box(divisor.clone().inverse_checked_with_abort(&signal).unwrap());
                }
            },
        );
        trace_matrix_profile_row(
            "mat3",
            "direct_reciprocal_checked_abort",
            input,
            lhs3_cases.len(),
            || {
                let divisor = black_box(rhs3_cases[0].clone());
                for _ in 0..lhs3_cases.len() {
                    black_box(divisor.clone().inverse_checked_with_abort(&signal).unwrap());
                }
            },
        );
        trace_matrix_profile_row(
            "mat3",
            "inverse_checked_affine",
            input,
            lhs3_affine_cases.len(),
            || {
                for value in &lhs3_affine_cases {
                    black_box(black_box(value.clone()).inverse_checked().unwrap());
                }
            },
        );
        trace_matrix_profile_row(
            "mat3",
            "div_matrix_affine",
            input,
            lhs3_affine_cases.len(),
            || {
                for index in 0..lhs3_affine_cases.len() {
                    black_box(
                        (black_box(lhs3_affine_cases[index].clone())
                            / black_box(rhs3_affine_cases[index].clone()))
                        .unwrap(),
                    );
                }
            },
        );
        trace_matrix_profile_row(
            "mat3",
            "affine_div_matrix_translation",
            input,
            lhs3_affine_translation_cases.len(),
            || {
                for index in 0..lhs3_affine_translation_cases.len() {
                    black_box(
                        (black_box(lhs3_affine_translation_cases[index].clone())
                            / black_box(rhs3_affine_translation_cases[index].clone()))
                        .unwrap(),
                    );
                }
            },
        );
        trace_matrix_profile_row("mat3", "powi", input, lhs3_cases.len(), || {
            for value in &lhs3_cases {
                black_box(black_box(value.clone()).powi(3).unwrap());
            }
        });
        trace_matrix_profile_row("mat3", "powi_checked", input, lhs3_cases.len(), || {
            for value in &lhs3_cases {
                black_box(black_box(value.clone()).powi_checked(3).unwrap());
            }
        });
        trace_matrix_profile_row("mat3", "transform_vec3", input, lhs3_cases.len(), || {
            for index in 0..lhs3_cases.len() {
                black_box(
                    black_box(lhs3_cases[index].clone())
                        * black_box(vector3_cases[index].clone()),
                );
            }
        });
        trace_matrix_profile_row("mat3", "transform_vec3_direct", input, lhs3_cases.len(), || {
            for index in 0..lhs3_cases.len() {
                let matrix = black_box(&lhs3_cases[index]);
                black_box(matrix.transform_vec3(black_box(&vector3_cases[index])));
            }
        });
        trace_matrix_profile_row("mat3", "transform_vec3_batch", input, vector3_cases.len(), || {
            let matrix = black_box(&lhs3_cases[0]);
            black_box(matrix.transform_vec3_batch(black_box(&vector3_cases)));
        });
        trace_matrix_profile_row("mat3", "powi_negative", input, lhs3_cases.len(), || {
            for value in &lhs3_cases {
                black_box(black_box(value.clone()).powi(-2).unwrap());
            }
        });
        trace_matrix_profile_row(
            "mat3",
            "powi_checked_negative",
            input,
            lhs3_cases.len(),
            || {
                for value in &lhs3_cases {
                    black_box(black_box(value.clone()).powi_checked(-2).unwrap());
                }
            },
        );
        trace_matrix_profile_row("mat4", "reciprocal", input, lhs4_cases.len(), || {
            for value in &lhs4_cases {
                black_box(black_box(value.clone()).reciprocal().unwrap());
            }
        });
        trace_matrix_profile_row("mat4", "diagonal_reciprocal", input, 1, || {
            black_box(black_box(diagonal_affine_matrix.clone()).reciprocal().unwrap());
        });
        trace_matrix_profile_row("mat4", "known_translation_inverse", input, 1, || {
            black_box(Matrix4::affine_translation_inverse(black_box(
                translation_values4.clone(),
            )));
        });
        trace_matrix_profile_row("mat4", "known_translation_div_matrix", input, 1, || {
            black_box(
                black_box(lhs4_cases[0].clone())
                    .div_affine_translation(black_box(translation_values4.clone())),
            );
        });
        trace_matrix_profile_row("mat4", "known_orthonormal_inverse", input, 1, || {
            black_box(Matrix4::affine_orthonormal_inverse(
                black_box(orthonormal_linear4.clone()),
                black_box(translation_values4.clone()),
            ));
        });
        trace_matrix_profile_row("mat4", "known_orthonormal_div_matrix", input, 1, || {
            black_box(black_box(lhs4_cases[0].clone()).div_affine_orthonormal(
                black_box(orthonormal_linear4.clone()),
                black_box(translation_values4.clone()),
            ));
        });
        trace_matrix_profile_row("mat4", "known_signed_permutation_inverse", input, 1, || {
            black_box(Matrix4::signed_permutation_inverse(black_box(
                signed_permutation_rows4,
            )));
        });
        trace_matrix_profile_row("mat4", "known_signed_permutation_div_matrix", input, 1, || {
            black_box(
                black_box(lhs4_cases[0].clone())
                    .div_signed_permutation(black_box(signed_permutation_rows4)),
            );
        });
        trace_matrix_profile_row("mat4", "known_signed_permutation_transform", input, 1, || {
            black_box(Matrix4::transform_signed_permutation_vector(
                black_box(signed_permutation_rows4),
                black_box(&vector4_cases[0]),
            ));
        });
        trace_matrix_profile_row("mat4", "known_signed_permutation_batch", input, 1, || {
            black_box(Matrix4::transform_signed_permutation_batch(
                black_box(signed_permutation_rows4),
                black_box(&translated_diagonal_point_batch),
            ));
        });
        trace_matrix_profile_row("mat4", "uniform_scale_reciprocal", input, 1, || {
            black_box(black_box(uniform_scale_matrix4.clone()).reciprocal().unwrap());
        });
        trace_matrix_profile_row("mat4", "known_uniform_scale_inverse", input, 1, || {
            black_box(
                Matrix4::uniform_scale_inverse(black_box(uniform_scale_value.clone()))
                    .unwrap(),
            );
        });
        trace_matrix_profile_row("mat4", "known_diagonal_inverse", input, 1, || {
            black_box(Matrix4::diagonal_inverse(black_box(diagonal_values4.clone())).unwrap());
        });
        trace_matrix_profile_row("mat4", "known_upper_triangular_inverse", input, 1, || {
            black_box(black_box(upper_triangular_matrix4.clone()).upper_triangular_inverse().unwrap());
        });
        trace_matrix_profile_row("mat4", "known_lower_triangular_inverse", input, 1, || {
            black_box(black_box(lower_triangular_matrix4.clone()).lower_triangular_inverse().unwrap());
        });
        // Keep checked triangular inverse rows in the same structural dispatch family
        // so we can validate the O(n²) branch under error-aware APIs.
        // See Golub and Van Loan, *Matrix Computations*; Yap, "Towards Exact
        // Geometric Computation", 1997.
        trace_matrix_profile_row("mat4", "known_upper_triangular_inverse_checked", input, 1, || {
            black_box(
                black_box(upper_triangular_matrix4.clone())
                    .upper_triangular_inverse_checked()
                    .unwrap(),
            );
        });
        trace_matrix_profile_row("mat4", "known_lower_triangular_inverse_checked", input, 1, || {
            black_box(
                black_box(lower_triangular_matrix4.clone())
                    .lower_triangular_inverse_checked()
                    .unwrap(),
            );
        });
        trace_matrix_profile_row("mat4", "known_diagonal_div_matrix", input, 1, || {
            black_box(
                black_box(lhs4_cases[0].clone())
                    .div_diagonal(black_box(diagonal_values4.clone()))
                    .unwrap(),
            );
        });
        trace_matrix_profile_row("mat4", "known_upper_triangular_div_matrix", input, 1, || {
            black_box(
            black_box(lhs4_cases[0].clone())
                .div_upper_triangular(black_box(upper_triangular_matrix4.clone()))
                .unwrap(),
        );
        });
        trace_matrix_profile_row("mat4", "known_lower_triangular_div_matrix", input, 1, || {
            black_box(
            black_box(lhs4_cases[0].clone())
                .div_lower_triangular(black_box(lower_triangular_matrix4.clone()))
                .unwrap(),
        );
        });
        trace_matrix_profile_row("mat4", "known_diagonal_div_vector", input, 1, || {
            black_box(
                black_box(&lhs4_cases[0])
                    .div_diagonal_vector(
                        black_box(diagonal_values4.clone()),
                        black_box(&vector4_cases[0]),
                    )
                    .unwrap(),
            );
        });
        trace_matrix_profile_row("mat4", "known_uniform_diagonal_div_vector", input, 1, || {
            black_box(
                black_box(&lhs4_cases[0])
                    .div_diagonal_vector(
                        black_box([
                            uniform_scale_value.clone(),
                            uniform_scale_value.clone(),
                            uniform_scale_value.clone(),
                            uniform_scale_value.clone(),
                        ]),
                        black_box(&vector4_cases[0]),
                    )
                    .unwrap(),
            );
        });
        trace_matrix_profile_row(
            "mat4",
            "known_diagonal_div_vector_direction",
            input,
            1,
            || {
                black_box(
                    black_box(&lhs4_cases[0])
                        .div_diagonal_vector(
                            black_box(diagonal_values4.clone()),
                            black_box(&translated_diagonal_direction),
                        )
                        .unwrap(),
                );
            },
        );
        trace_matrix_profile_row(
            "mat4",
            "known_uniform_diagonal_div_vector_direction",
            input,
            1,
            || {
                black_box(
                    black_box(&lhs4_cases[0])
                        .div_diagonal_vector(
                            black_box([
                                uniform_scale_value.clone(),
                                uniform_scale_value.clone(),
                                uniform_scale_value.clone(),
                                uniform_scale_value.clone(),
                            ]),
                            black_box(&translated_diagonal_direction),
                        )
                        .unwrap(),
                );
            },
        );
        trace_matrix_profile_row("mat4", "known_diagonal_div_vector_point", input, 1, || {
            black_box(
                black_box(&lhs4_cases[0])
                    .div_diagonal_vector(
                        black_box(diagonal_values4.clone()),
                        black_box(&translated_diagonal_point),
                    )
                    .unwrap(),
            );
        });
        trace_matrix_profile_row("mat4", "known_uniform_diagonal_div_vector_point", input, 1, || {
            black_box(
                black_box(&lhs4_cases[0])
                    .div_diagonal_vector(
                        black_box([
                            uniform_scale_value.clone(),
                            uniform_scale_value.clone(),
                            uniform_scale_value.clone(),
                            uniform_scale_value.clone(),
                        ]),
                        black_box(&translated_diagonal_point),
                    )
                    .unwrap(),
            );
        });
        trace_matrix_profile_row("mat4", "reciprocal_checked", input, lhs4_cases.len(), || {
            for value in &lhs4_cases {
                black_box(black_box(value.clone()).reciprocal_checked().unwrap());
            }
        });
        trace_matrix_profile_row("mat4", "inverse_checked", input, lhs4_cases.len(), || {
            for value in &lhs4_cases {
                black_box(black_box(value.clone()).inverse_checked().unwrap());
            }
        });
        trace_matrix_profile_row(
            "mat4",
            "inverse_checked_abort",
            input,
            lhs4_cases.len(),
            || {
                for value in &lhs4_cases {
                    black_box(
                        black_box(value.clone())
                            .inverse_checked_with_abort(&signal)
                            .unwrap(),
                    );
                }
            },
        );
        trace_matrix_profile_row("mat4", "div_matrix", input, lhs4_cases.len(), || {
            for index in 0..lhs4_cases.len() {
                black_box(
                    (black_box(lhs4_cases[index].clone()) / black_box(rhs4_cases[index].clone()))
                        .unwrap(),
                );
            }
        });
        trace_matrix_profile_row("mat4", "div_matrix_checked", input, lhs4_cases.len(), || {
            for index in 0..lhs4_cases.len() {
                black_box(
                    black_box(lhs4_cases[index].clone())
                        .div_matrix_checked(black_box(rhs4_cases[index].clone()))
                        .unwrap(),
                );
            }
        });
        trace_matrix_profile_row(
            "mat4",
            "div_matrix_checked_abort",
            input,
            lhs4_cases.len(),
            || {
                for index in 0..lhs4_cases.len() {
                    black_box(
                        black_box(lhs4_cases[index].clone())
                            .div_matrix_checked_with_abort(
                                black_box(rhs4_cases[index].clone()),
                                &signal,
                            )
                            .unwrap(),
                    );
                }
            },
        );
        trace_matrix_profile_row("mat4", "direct_div_matrix", input, lhs4_cases.len(), || {
            let divisor = black_box(rhs4_cases[0].clone());
            for index in 0..lhs4_cases.len() {
                black_box(
                    black_box(lhs4_cases[index].clone())
                        .div_matrix_checked(black_box(divisor.clone()))
                        .unwrap(),
                );
            }
        });
        trace_matrix_profile_row(
            "mat4",
            "direct_div_matrix_checked",
            input,
            lhs4_cases.len(),
            || {
                let divisor = black_box(rhs4_cases[0].clone());
                for index in 0..lhs4_cases.len() {
                    black_box(
                        black_box(lhs4_cases[index].clone())
                            .div_matrix_checked(black_box(divisor.clone()))
                            .unwrap(),
                    );
                }
            },
        );
        trace_matrix_profile_row(
            "mat4",
            "direct_div_matrix_checked_abort",
            input,
            lhs4_cases.len(),
            || {
                let divisor = black_box(rhs4_cases[0].clone());
                for index in 0..lhs4_cases.len() {
                    black_box(
                        black_box(lhs4_cases[index].clone())
                            .div_matrix_checked_with_abort(black_box(divisor.clone()), &signal)
                            .unwrap(),
                    );
                }
            },
        );
        trace_matrix_profile_row("mat4", "direct_inverse", input, lhs4_cases.len(), || {
            let divisor = black_box(rhs4_cases[0].clone());
            for _ in 0..lhs4_cases.len() {
                black_box(divisor.clone().inverse().unwrap());
            }
        });
        trace_matrix_profile_row("mat4", "direct_reciprocal", input, lhs4_cases.len(), || {
            let divisor = black_box(rhs4_cases[0].clone());
            for _ in 0..lhs4_cases.len() {
                black_box(divisor.clone().reciprocal().unwrap());
            }
        });
        trace_matrix_profile_row(
            "mat4",
            "direct_inverse_checked",
            input,
            lhs4_cases.len(),
            || {
                let divisor = black_box(rhs4_cases[0].clone());
                for _ in 0..lhs4_cases.len() {
                    black_box(divisor.clone().inverse_checked().unwrap());
                }
            },
        );
        trace_matrix_profile_row(
            "mat4",
            "direct_reciprocal_checked",
            input,
            lhs4_cases.len(),
            || {
                let divisor = black_box(rhs4_cases[0].clone());
                for _ in 0..lhs4_cases.len() {
                    black_box(divisor.clone().reciprocal_checked().unwrap());
                }
            },
        );
        trace_matrix_profile_row(
            "mat4",
            "direct_inverse_checked_abort",
            input,
            lhs4_cases.len(),
            || {
                let divisor = black_box(rhs4_cases[0].clone());
                for _ in 0..lhs4_cases.len() {
                    black_box(divisor.clone().inverse_checked_with_abort(&signal).unwrap());
                }
            },
        );
        trace_matrix_profile_row(
            "mat4",
            "direct_reciprocal_checked_abort",
            input,
            lhs4_cases.len(),
            || {
                let divisor = black_box(rhs4_cases[0].clone());
                for _ in 0..lhs4_cases.len() {
                    black_box(divisor.clone().inverse_checked_with_abort(&signal).unwrap());
                }
            },
        );
        trace_matrix_profile_row("mat4", "direct_powi_negative", input, lhs4_cases.len(), || {
            let divisor = black_box(rhs4_cases[0].clone());
            for _ in 0..lhs4_cases.len() {
                black_box(divisor.clone().powi(-2).unwrap());
            }
        });
        trace_matrix_profile_row(
            "mat4",
            "direct_powi_negative_one",
            input,
            lhs4_cases.len(),
            || {
                let divisor = black_box(rhs4_cases[0].clone());
                for _ in 0..lhs4_cases.len() {
                    black_box(divisor.clone().powi(-1).unwrap());
                }
            },
        );
        trace_matrix_profile_row(
            "mat4",
            "inverse_checked_affine",
            input,
            lhs4_affine_cases.len(),
            || {
                for value in &lhs4_affine_cases {
                    black_box(black_box(value.clone()).inverse_checked().unwrap());
                }
            },
        );
        trace_matrix_profile_row(
            "mat4",
            "div_matrix_affine",
            input,
            lhs4_affine_cases.len(),
            || {
                for index in 0..lhs4_affine_cases.len() {
                    black_box(
                        (black_box(lhs4_affine_cases[index].clone()) / black_box(rhs4_affine_cases[index].clone()))
                        .unwrap(),
                    );
                }
            },
        );
        trace_matrix_profile_row(
            "mat4",
            "affine_div_matrix_translation",
            input,
            lhs4_affine_translation_cases.len(),
            || {
                for index in 0..lhs4_affine_translation_cases.len() {
                    black_box(
                        (black_box(lhs4_affine_translation_cases[index].clone())
                            / black_box(rhs4_affine_translation_cases[index].clone()))
                        .unwrap(),
                    );
                }
            },
        );
        trace_matrix_profile_row(
            "mat4",
            "affine_div_matrix_checked",
            input,
            lhs4_affine_cases.len(),
            || {
                for index in 0..lhs4_affine_cases.len() {
                    black_box(
                        black_box(lhs4_affine_cases[index].clone())
                            .div_matrix_checked(black_box(rhs4_affine_cases[index].clone()))
                            .unwrap(),
                    );
                }
            },
        );
        trace_matrix_profile_row(
            "mat4",
            "affine_div_matrix_checked_abort",
            input,
            lhs4_affine_cases.len(),
            || {
                for index in 0..lhs4_affine_cases.len() {
                    black_box(
                        black_box(lhs4_affine_cases[index].clone())
                            .div_matrix_checked_with_abort(
                                black_box(rhs4_affine_cases[index].clone()),
                                &signal,
                            )
                            .unwrap(),
                    );
                }
            },
        );
        trace_matrix_profile_row(
            "mat4",
            "affine_div_matrix_translation_checked",
            input,
            lhs4_affine_translation_cases.len(),
            || {
                for index in 0..lhs4_affine_translation_cases.len() {
                    black_box(
                        black_box(lhs4_affine_translation_cases[index].clone())
                            .div_matrix_checked(
                                black_box(rhs4_affine_translation_cases[index].clone()),
                            )
                            .unwrap(),
                    );
                }
            },
        );
        trace_matrix_profile_row(
            "mat4",
            "affine_div_matrix_translation_checked_abort",
            input,
            lhs4_affine_translation_cases.len(),
            || {
                for index in 0..lhs4_affine_translation_cases.len() {
                    black_box(
                        black_box(lhs4_affine_translation_cases[index].clone())
                            .div_matrix_checked_with_abort(
                                black_box(rhs4_affine_translation_cases[index].clone()),
                                &signal,
                            )
                            .unwrap(),
                    );
                }
            },
        );
        trace_matrix_profile_row("mat4", "powi", input, lhs4_cases.len(), || {
            for value in &lhs4_cases {
                black_box(black_box(value.clone()).powi(3).unwrap());
            }
        });
        trace_matrix_profile_row("mat4", "powi_checked", input, lhs4_cases.len(), || {
            for value in &lhs4_cases {
                black_box(black_box(value.clone()).powi_checked(3).unwrap());
            }
        });
        trace_matrix_profile_row("mat4", "transform_vec4", input, lhs4_cases.len(), || {
            for index in 0..lhs4_cases.len() {
                black_box(
                    black_box(lhs4_cases[index].clone())
                        * black_box(vector4_cases[index].clone()),
                );
            }
        });
        trace_matrix_profile_row(
            "mat4",
            "translated_diagonal_direction_transform",
            input,
            1,
            || {
                black_box(
                    black_box(&translated_diagonal_direction_matrix)
                        .transform_vec4_direction(black_box(&translated_diagonal_direction)),
                );
            },
        );
        trace_matrix_profile_row(
            "mat4",
            "translated_diagonal_point_transform",
            input,
            1,
            || {
                black_box(
                    black_box(&translated_diagonal_direction_matrix)
                        .transform_vec4_point(black_box(&translated_diagonal_point)),
                );
            },
        );
        trace_matrix_profile_row("mat4", "identity_direction_transform", input, 1, || {
            black_box(
                black_box(&identity_matrix4)
                    .transform_vec4_direction(black_box(&translated_diagonal_direction)),
            );
        });
        trace_matrix_profile_row("mat4", "identity_point_transform", input, 1, || {
            black_box(
                black_box(&identity_matrix4)
                    .transform_vec4_point(black_box(&translated_diagonal_point)),
            );
        });
        trace_matrix_profile_row("mat4", "identity_direction_transform_direct", input, 1, || {
            let matrix = black_box(&identity_matrix4);
            black_box(matrix.transform_vec4_direction(black_box(&translated_diagonal_direction)));
        });
        trace_matrix_profile_row("mat4", "identity_point_transform_direct", input, 1, || {
            let matrix = black_box(&identity_matrix4);
            black_box(matrix.transform_vec4_point(black_box(&translated_diagonal_point)));
        });
        trace_matrix_profile_row("mat4", "identity_direction_transform_generic", input, 1, || {
            black_box(
                black_box(&identity_matrix4)
                    .transform_vec4(black_box(&translated_diagonal_direction)),
            );
        });
        trace_matrix_profile_row("mat4", "identity_point_transform_generic", input, 1, || {
            black_box(
                black_box(&identity_matrix4)
                    .transform_vec4(black_box(&translated_diagonal_point)),
            );
        });
        trace_matrix_profile_row(
            "mat4",
            "translated_diagonal_direction_transform_generic",
            input,
            1,
            || {
                black_box(
                    black_box(&translated_diagonal_direction_matrix)
                        .transform_vec4(black_box(&translated_diagonal_direction)),
                );
            },
        );
        trace_matrix_profile_row(
            "mat4",
            "translated_diagonal_point_transform_generic",
            input,
            1,
            || {
                black_box(
                    black_box(&translated_diagonal_direction_matrix)
                        .transform_vec4(black_box(&translated_diagonal_point)),
                );
            },
        );
        trace_matrix_profile_row(
            "mat4",
            "translated_diagonal_direction_batch",
            input,
            translated_diagonal_direction_batch.len(),
            || {
                let matrix = black_box(&translated_diagonal_direction_matrix);
                black_box(matrix.transform_vec4_batch(black_box(&translated_diagonal_direction_batch)));
            },
        );
        trace_matrix_profile_row(
            "mat4",
            "translated_diagonal_point_batch",
            input,
            translated_diagonal_point_batch.len(),
            || {
                let matrix = black_box(&translated_diagonal_direction_matrix);
                black_box(matrix.transform_vec4_batch(black_box(&translated_diagonal_point_batch)));
            },
        );
        trace_matrix_profile_row(
            "mat4",
            "translated_diagonal_direction_batch_assumed",
            input,
            translated_diagonal_direction_batch.len(),
            || {
                let matrix = black_box(&translated_diagonal_direction_matrix);
                black_box(matrix.transform_vec4_direction_batch(black_box(&translated_diagonal_direction_batch)));
            },
        );
        trace_matrix_profile_row(
            "mat4",
            "translated_diagonal_direction_batch_public_assumed",
            input,
            translated_diagonal_direction_batch.len(),
            || {
                black_box(
                    black_box(&translated_diagonal_direction_matrix)
                        .transform_vec4_direction_batch(black_box(&translated_diagonal_direction_batch)),
                );
            },
        );
        trace_matrix_profile_row(
            "mat4",
            "translated_diagonal_point_batch_assumed",
            input,
            translated_diagonal_point_batch.len(),
            || {
                let matrix = black_box(&translated_diagonal_direction_matrix);
                black_box(matrix.transform_vec4_point_batch(black_box(&translated_diagonal_point_batch)));
            },
        );
        trace_matrix_profile_row(
            "mat4",
            "translated_diagonal_point_batch_public_assumed",
            input,
            translated_diagonal_point_batch.len(),
            || {
                black_box(
                    black_box(&translated_diagonal_direction_matrix)
                        .transform_vec4_point_batch(black_box(&translated_diagonal_point_batch)),
                );
            },
        );
        trace_matrix_profile_row(
            "mat4",
            "diagonal_direction_batch",
            input,
            translated_diagonal_direction_batch.len(),
            || {
                let matrix = black_box(&diagonal_affine_matrix);
                black_box(matrix.transform_vec4_batch(black_box(&translated_diagonal_direction_batch)));
            },
        );
        trace_matrix_profile_row(
            "mat4",
            "diagonal_point_batch",
            input,
            translated_diagonal_point_batch.len(),
            || {
                let matrix = black_box(&diagonal_affine_matrix);
                black_box(matrix.transform_vec4_batch(black_box(&translated_diagonal_point_batch)));
            },
        );
        trace_matrix_profile_row(
            "mat4",
            "identity_direction_batch_assumed",
            input,
            translated_diagonal_direction_batch.len(),
            || {
                let matrix = black_box(&identity_matrix4);
                black_box(matrix.transform_vec4_direction_batch(black_box(&translated_diagonal_direction_batch)));
            },
        );
        trace_matrix_profile_row(
            "mat4",
            "identity_point_batch_assumed",
            input,
            translated_diagonal_point_batch.len(),
            || {
                let matrix = black_box(&identity_matrix4);
                black_box(matrix.transform_vec4_point_batch(black_box(&translated_diagonal_point_batch)));
            },
        );
        trace_matrix_profile_row("mat4", "powi_negative", input, lhs4_cases.len(), || {
            for value in &lhs4_cases {
                black_box(black_box(value.clone()).powi(-2).unwrap());
            }
        });
        trace_matrix_profile_row(
            "mat4",
            "powi_checked_negative",
            input,
            lhs4_cases.len(),
            || {
                for value in &lhs4_cases {
                    black_box(black_box(value.clone()).powi_checked(-2).unwrap());
                }
            },
        );
    }

    trace_dispatch_row(format!("matrix_ops/{label}/mat3 reciprocal"), || {
        for value in &lhs3_cases {
            black_box(black_box(value.clone()).reciprocal().unwrap());
        }
    });
    trace_dispatch_row(format!("matrix_ops/{label}/mat3 uniform_scale_reciprocal"), || {
        black_box(black_box(uniform_scale_matrix3.clone()).reciprocal().unwrap());
    });
    trace_dispatch_row(format!("matrix_ops/{label}/mat3 known_uniform_scale_inverse"), || {
        black_box(
            Matrix3::uniform_scale_inverse(black_box(uniform_scale_value.clone())).unwrap(),
        );
    });
    trace_dispatch_row(format!("matrix_ops/{label}/mat3 known_diagonal_inverse"), || {
        black_box(Matrix3::diagonal_inverse(black_box(diagonal_values3.clone())).unwrap());
    });
    trace_dispatch_row(format!("matrix_ops/{label}/mat3 known_upper_triangular_inverse"), || {
        black_box(black_box(upper_triangular_matrix3.clone()).upper_triangular_inverse().unwrap());
    });
    trace_dispatch_row(format!("matrix_ops/{label}/mat3 known_lower_triangular_inverse"), || {
        black_box(black_box(lower_triangular_matrix3.clone()).lower_triangular_inverse().unwrap());
    });
    // Keep checked triangular inverse rows in the trace profile to validate the same
    // dispatch branch under `inverse_checked` semantics.
    trace_dispatch_row(format!("matrix_ops/{label}/mat3 known_upper_triangular_inverse_checked"), || {
        black_box(black_box(upper_triangular_matrix3.clone()).upper_triangular_inverse_checked().unwrap());
    });
    trace_dispatch_row(format!("matrix_ops/{label}/mat3 known_lower_triangular_inverse_checked"), || {
        black_box(black_box(lower_triangular_matrix3.clone()).lower_triangular_inverse_checked().unwrap());
    });
    trace_dispatch_row(format!("matrix_ops/{label}/mat3 known_diagonal_div_matrix"), || {
        black_box(
            black_box(lhs3_cases[0].clone())
                .div_diagonal(black_box(diagonal_values3.clone()))
                .unwrap(),
        );
    });
    trace_dispatch_row(format!("matrix_ops/{label}/mat3 known_upper_triangular_div_matrix"), || {
        black_box(
            black_box(lhs3_cases[0].clone())
                .div_upper_triangular(black_box(upper_triangular_matrix3.clone()))
                .unwrap(),
        );
    });
    trace_dispatch_row(format!("matrix_ops/{label}/mat3 known_lower_triangular_div_matrix"), || {
        black_box(
            black_box(lhs3_cases[0].clone())
                .div_lower_triangular(black_box(lower_triangular_matrix3.clone()))
                .unwrap(),
        );
    });
    trace_dispatch_row(format!("matrix_ops/{label}/mat3 known_diagonal_div_vector"), || {
        black_box(
            black_box(&lhs3_cases[0])
                .div_diagonal_vector(
                    black_box(diagonal_values3.clone()),
                    black_box(&vector3_cases[0]),
                )
                .unwrap(),
        );
    });
    trace_dispatch_row(
        format!("matrix_ops/{label}/mat3 known_uniform_diagonal_div_vector"),
        || {
            black_box(
                black_box(&lhs3_cases[0])
                    .div_diagonal_vector(
                        black_box([
                            uniform_scale_value.clone(),
                            uniform_scale_value.clone(),
                            uniform_scale_value.clone(),
                        ]),
                        black_box(&vector3_cases[0]),
                    )
                    .unwrap(),
            );
        },
    );
    trace_dispatch_row(format!("matrix_ops/{label}/mat3 reciprocal_checked"), || {
        for value in &lhs3_cases {
            black_box(black_box(value.clone()).reciprocal_checked().unwrap());
        }
    });
    trace_dispatch_row(format!("matrix_ops/{label}/mat3 inverse_checked"), || {
        for value in &lhs3_cases {
            black_box(black_box(value.clone()).inverse_checked().unwrap());
        }
    });
    trace_dispatch_row(format!("matrix_ops/{label}/mat3 inverse_checked_abort"), || {
        for value in &lhs3_cases {
            black_box(
                black_box(value.clone())
                    .inverse_checked_with_abort(&signal)
                    .unwrap(),
            );
        }
    });
    trace_dispatch_row(format!("matrix_ops/{label}/mat3 powi"), || {
        for value in &lhs3_cases {
            black_box(black_box(value.clone()).powi(3).unwrap());
        }
    });
    trace_dispatch_row(format!("matrix_ops/{label}/mat3 powi_checked"), || {
        for value in &lhs3_cases {
            black_box(black_box(value.clone()).powi_checked(3).unwrap());
        }
    });
    trace_dispatch_row(format!("matrix_ops/{label}/mat3 powi_checked_abort"), || {
        for value in &lhs3_cases {
            black_box(
                black_box(value.clone())
                    .powi_checked_with_abort(3, &signal)
                    .unwrap(),
            );
        }
    });
    trace_dispatch_row(format!("matrix_ops/{label}/mat3 powi_negative"), || {
        for value in &lhs3_cases {
            black_box(black_box(value.clone()).powi(-2).unwrap());
        }
    });
    trace_dispatch_row(
        format!("matrix_ops/{label}/mat3 powi_negative_one"),
        || {
            for value in &lhs3_cases {
                black_box(black_box(value.clone()).powi(-1).unwrap());
            }
        },
    );
    trace_dispatch_row(format!("matrix_ops/{label}/mat3 powi_checked_negative"), || {
        for value in &lhs3_cases {
            black_box(black_box(value.clone()).powi_checked(-2).unwrap());
        }
    });
    trace_dispatch_row(format!("matrix_ops/{label}/mat3 transform_vec3_direct"), || {
        for index in 0..lhs3_cases.len() {
            let matrix = black_box(&lhs3_cases[index]);
            black_box(matrix.transform_vec3(black_box(&vector3_cases[index])));
        }
    });
    trace_dispatch_row(format!("matrix_ops/{label}/mat3 transform_vec3_batch"), || {
        let matrix = black_box(&lhs3_cases[0]);
        black_box(matrix.transform_vec3_batch(black_box(&vector3_cases)));
    });
    trace_dispatch_row(format!("matrix_ops/{label}/mat3 div_matrix_checked"), || {
        for index in 0..lhs3_cases.len() {
            black_box(
                black_box(lhs3_cases[index].clone())
                    .div_matrix_checked(black_box(rhs3_cases[index].clone()))
                    .unwrap(),
            );
        }
    });
    trace_dispatch_row(
        format!("matrix_ops/{label}/mat3 div_matrix_checked_abort"),
        || {
            for index in 0..lhs3_cases.len() {
                black_box(
                    black_box(lhs3_cases[index].clone())
                        .div_matrix_checked_with_abort(black_box(rhs3_cases[index].clone()), &signal)
                        .unwrap(),
                );
            }
        },
    );
    trace_dispatch_row(format!("matrix_ops/{label}/mat3 direct_div_matrix"), || {
        let divisor = black_box(rhs3_cases[0].clone());
        for index in 0..lhs3_cases.len() {
            black_box(
                black_box(lhs3_cases[index].clone())
                    .div_matrix_checked(black_box(divisor.clone()))
                    .unwrap(),
            );
        }
    });
    trace_dispatch_row(format!("matrix_ops/{label}/mat3 direct_inverse"), || {
        let divisor = black_box(rhs3_cases[0].clone());
        for _ in 0..lhs3_cases.len() {
            black_box(divisor.clone().inverse().unwrap());
        }
    });
    trace_dispatch_row(format!("matrix_ops/{label}/mat3 direct_reciprocal"), || {
        let divisor = black_box(rhs3_cases[0].clone());
        for _ in 0..lhs3_cases.len() {
            black_box(divisor.clone().reciprocal().unwrap());
        }
    });
    trace_dispatch_row(format!("matrix_ops/{label}/mat3 direct_inverse_checked"), || {
        let divisor = black_box(rhs3_cases[0].clone());
        for _ in 0..lhs3_cases.len() {
            black_box(divisor.clone().inverse_checked().unwrap());
        }
    });
    trace_dispatch_row(
        format!("matrix_ops/{label}/mat3 direct_reciprocal_checked"),
        || {
            let divisor = black_box(rhs3_cases[0].clone());
            for _ in 0..lhs3_cases.len() {
                black_box(divisor.clone().reciprocal_checked().unwrap());
            }
        },
    );
    trace_dispatch_row(
        format!("matrix_ops/{label}/mat3 direct_inverse_checked_abort"),
        || {
            let divisor = black_box(rhs3_cases[0].clone());
            for _ in 0..lhs3_cases.len() {
                black_box(divisor.clone().inverse_checked_with_abort(&signal).unwrap());
            }
        },
    );
    trace_dispatch_row(
        format!("matrix_ops/{label}/mat3 direct_reciprocal_checked_abort"),
        || {
            let divisor = black_box(rhs3_cases[0].clone());
            for _ in 0..lhs3_cases.len() {
                black_box(divisor.clone().inverse_checked_with_abort(&signal).unwrap());
            }
        },
    );
    trace_dispatch_row(format!("matrix_ops/{label}/mat3 direct_div_matrix_checked"), || {
        let divisor = black_box(rhs3_cases[0].clone());
        for index in 0..lhs3_cases.len() {
            black_box(
                black_box(lhs3_cases[index].clone())
                    .div_matrix_checked(black_box(divisor.clone()))
                    .unwrap(),
            );
        }
    });
    trace_dispatch_row(
        format!("matrix_ops/{label}/mat3 direct_div_matrix_checked_abort"),
        || {
            let divisor = black_box(rhs3_cases[0].clone());
            for index in 0..lhs3_cases.len() {
                black_box(
                    black_box(lhs3_cases[index].clone())
                        .div_matrix_checked_with_abort(black_box(divisor.clone()), &signal)
                        .unwrap(),
                );
            }
        },
    );
    trace_dispatch_row(format!("matrix_ops/{label}/mat3 div_matrix"), || {
        for index in 0..lhs3_cases.len() {
            black_box(
                (black_box(lhs3_cases[index].clone()) / black_box(rhs3_cases[index].clone()))
                    .unwrap(),
            );
        }
    });
    trace_dispatch_row(format!("matrix_ops/{label}/mat3 affine_inverse"), || {
        for value in &lhs3_affine_cases {
            black_box(black_box(value.clone()).inverse_checked().unwrap());
        }
    });
    trace_dispatch_row(format!("matrix_ops/{label}/mat3 affine_div_matrix"), || {
        for index in 0..lhs3_affine_cases.len() {
            black_box(
                (black_box(lhs3_affine_cases[index].clone())
                    / black_box(rhs3_affine_cases[index].clone()))
                    .unwrap(),
            );
        }
    });
    trace_dispatch_row(format!("matrix_ops/{label}/mat3 affine_div_matrix_translation"), || {
        for index in 0..lhs3_affine_translation_cases.len() {
            black_box(
                (black_box(lhs3_affine_translation_cases[index].clone())
                    / black_box(rhs3_affine_translation_cases[index].clone()))
                    .unwrap(),
            );
        }
    });
    trace_dispatch_row(format!("matrix_ops/{label}/mat3 bitxor"), || {
        for value in &lhs3_cases {
            black_box((black_box(value.clone()) ^ 3).unwrap());
        }
    });
    trace_dispatch_row(format!("matrix_ops/{label}/mat4 reciprocal"), || {
        for value in &lhs4_cases {
            black_box(black_box(value.clone()).reciprocal().unwrap());
        }
    });
    trace_dispatch_row(format!("matrix_ops/{label}/mat4 diagonal_reciprocal"), || {
        black_box(black_box(diagonal_affine_matrix.clone()).reciprocal().unwrap());
    });
    trace_dispatch_row(format!("matrix_ops/{label}/mat4 known_translation_inverse"), || {
        black_box(Matrix4::affine_translation_inverse(black_box(
            translation_values4.clone(),
        )));
    });
    trace_dispatch_row(format!("matrix_ops/{label}/mat4 known_translation_div_matrix"), || {
        black_box(
            black_box(lhs4_cases[0].clone())
                .div_affine_translation(black_box(translation_values4.clone())),
        );
    });
    trace_dispatch_row(format!("matrix_ops/{label}/mat4 known_orthonormal_inverse"), || {
        black_box(Matrix4::affine_orthonormal_inverse(
            black_box(orthonormal_linear4.clone()),
            black_box(translation_values4.clone()),
        ));
    });
    trace_dispatch_row(format!("matrix_ops/{label}/mat4 known_orthonormal_div_matrix"), || {
        black_box(black_box(lhs4_cases[0].clone()).div_affine_orthonormal(
            black_box(orthonormal_linear4.clone()),
            black_box(translation_values4.clone()),
        ));
    });
    trace_dispatch_row(
        format!("matrix_ops/{label}/mat4 known_signed_permutation_inverse"),
        || {
            black_box(Matrix4::signed_permutation_inverse(black_box(
                signed_permutation_rows4,
            )));
        },
    );
    trace_dispatch_row(
        format!("matrix_ops/{label}/mat4 known_signed_permutation_div_matrix"),
        || {
            black_box(
                black_box(lhs4_cases[0].clone())
                    .div_signed_permutation(black_box(signed_permutation_rows4)),
            );
        },
    );
    trace_dispatch_row(
        format!("matrix_ops/{label}/mat4 known_signed_permutation_transform"),
        || {
            black_box(Matrix4::transform_signed_permutation_vector(
                black_box(signed_permutation_rows4),
                black_box(&vector4_cases[0]),
            ));
        },
    );
    trace_dispatch_row(
        format!("matrix_ops/{label}/mat4 known_signed_permutation_batch"),
        || {
            black_box(Matrix4::transform_signed_permutation_batch(
                black_box(signed_permutation_rows4),
                black_box(&translated_diagonal_point_batch),
            ));
        },
    );
    trace_dispatch_row(format!("matrix_ops/{label}/mat4 uniform_scale_reciprocal"), || {
        black_box(black_box(uniform_scale_matrix4.clone()).reciprocal().unwrap());
    });
    trace_dispatch_row(format!("matrix_ops/{label}/mat4 known_uniform_scale_inverse"), || {
        black_box(
            Matrix4::uniform_scale_inverse(black_box(uniform_scale_value.clone())).unwrap(),
        );
    });
    trace_dispatch_row(format!("matrix_ops/{label}/mat4 known_diagonal_inverse"), || {
        black_box(Matrix4::diagonal_inverse(black_box(diagonal_values4.clone())).unwrap());
    });
    trace_dispatch_row(format!("matrix_ops/{label}/mat4 known_upper_triangular_inverse"), || {
        black_box(black_box(upper_triangular_matrix4.clone()).upper_triangular_inverse().unwrap());
    });
    trace_dispatch_row(format!("matrix_ops/{label}/mat4 known_lower_triangular_inverse"), || {
        black_box(black_box(lower_triangular_matrix4.clone()).lower_triangular_inverse().unwrap());
    });
    // Keep checked triangular inverse rows in the trace profile to validate the same
    // dispatch branch under `inverse_checked` semantics.
    trace_dispatch_row(format!("matrix_ops/{label}/mat4 known_upper_triangular_inverse_checked"), || {
        black_box(black_box(upper_triangular_matrix4.clone()).upper_triangular_inverse_checked().unwrap());
    });
    trace_dispatch_row(format!("matrix_ops/{label}/mat4 known_lower_triangular_inverse_checked"), || {
        black_box(black_box(lower_triangular_matrix4.clone()).lower_triangular_inverse_checked().unwrap());
    });
    trace_dispatch_row(format!("matrix_ops/{label}/mat4 known_diagonal_div_matrix"), || {
        black_box(
            black_box(lhs4_cases[0].clone())
                .div_diagonal(black_box(diagonal_values4.clone()))
                .unwrap(),
        );
    });
    trace_dispatch_row(format!("matrix_ops/{label}/mat4 known_upper_triangular_div_matrix"), || {
        black_box(
            black_box(lhs4_cases[0].clone())
                .div_upper_triangular(black_box(upper_triangular_matrix4.clone()))
                .unwrap(),
        );
    });
    trace_dispatch_row(format!("matrix_ops/{label}/mat4 known_lower_triangular_div_matrix"), || {
        black_box(
            black_box(lhs4_cases[0].clone())
                .div_lower_triangular(black_box(lower_triangular_matrix4.clone()))
                .unwrap(),
        );
    });
    trace_dispatch_row(format!("matrix_ops/{label}/mat4 known_diagonal_div_vector"), || {
        black_box(
            black_box(&lhs4_cases[0])
                .div_diagonal_vector(
                    black_box(diagonal_values4.clone()),
                    black_box(&vector4_cases[0]),
                )
                .unwrap(),
        );
    });
    trace_dispatch_row(
        format!("matrix_ops/{label}/mat4 known_uniform_diagonal_div_vector"),
        || {
            black_box(
                black_box(&lhs4_cases[0])
                    .div_diagonal_vector(
                        black_box([
                            uniform_scale_value.clone(),
                            uniform_scale_value.clone(),
                            uniform_scale_value.clone(),
                            uniform_scale_value.clone(),
                        ]),
                        black_box(&vector4_cases[0]),
                    )
                    .unwrap(),
            );
        },
    );
    trace_dispatch_row(format!(
        "matrix_ops/{label}/mat4 known_diagonal_div_vector_direction",
    ), || {
        black_box(
            black_box(&lhs4_cases[0])
                .div_diagonal_vector(
                    black_box(diagonal_values4.clone()),
                    black_box(&translated_diagonal_direction),
                )
                .unwrap(),
        );
    });
    trace_dispatch_row(format!(
        "matrix_ops/{label}/mat4 known_uniform_diagonal_div_vector_direction",
    ), || {
        black_box(
            black_box(&lhs4_cases[0])
                .div_diagonal_vector(
                    black_box([
                        uniform_scale_value.clone(),
                        uniform_scale_value.clone(),
                        uniform_scale_value.clone(),
                        uniform_scale_value.clone(),
                    ]),
                    black_box(&translated_diagonal_direction),
                )
                .unwrap(),
        );
    });
    trace_dispatch_row(format!(
        "matrix_ops/{label}/mat4 known_diagonal_div_vector_direction_only",
    ), || {
        black_box(
            black_box(&lhs4_cases[0])
                .div_diagonal_direction_vector(
                    black_box(diagonal_values4.clone()),
                    black_box(&translated_diagonal_direction),
                )
                .unwrap(),
        );
    });
    trace_dispatch_row(format!(
        "matrix_ops/{label}/mat4 known_diagonal_div_vector_point",
    ), || {
        black_box(
            black_box(&lhs4_cases[0])
                .div_diagonal_vector(
                    black_box(diagonal_values4.clone()),
                    black_box(&translated_diagonal_point),
                )
                .unwrap(),
        );
    });
    trace_dispatch_row(format!(
        "matrix_ops/{label}/mat4 known_uniform_diagonal_div_vector_point",
    ), || {
        black_box(
            black_box(&lhs4_cases[0])
                .div_diagonal_vector(
                    black_box([
                        uniform_scale_value.clone(),
                        uniform_scale_value.clone(),
                        uniform_scale_value.clone(),
                        uniform_scale_value.clone(),
                    ]),
                    black_box(&translated_diagonal_point),
                )
                .unwrap(),
        );
    });
    trace_dispatch_row(format!("matrix_ops/{label}/mat4 reciprocal_checked"), || {
        for value in &lhs4_cases {
            black_box(black_box(value.clone()).reciprocal_checked().unwrap());
        }
    });
    trace_dispatch_row(format!("matrix_ops/{label}/mat4 inverse_checked"), || {
        for value in &lhs4_cases {
            black_box(black_box(value.clone()).inverse_checked().unwrap());
        }
    });
    trace_dispatch_row(format!("matrix_ops/{label}/mat4 inverse_checked_abort"), || {
        for value in &lhs4_cases {
            black_box(
                black_box(value.clone())
                    .inverse_checked_with_abort(&signal)
                    .unwrap(),
            );
        }
    });
    trace_dispatch_row(format!("matrix_ops/{label}/mat4 powi"), || {
        for value in &lhs4_cases {
            black_box(black_box(value.clone()).powi(3).unwrap());
        }
    });
    trace_dispatch_row(format!("matrix_ops/{label}/mat4 powi_checked"), || {
        for value in &lhs4_cases {
            black_box(black_box(value.clone()).powi_checked(3).unwrap());
        }
    });
    trace_dispatch_row(format!("matrix_ops/{label}/mat4 powi_checked_abort"), || {
        for value in &lhs4_cases {
            black_box(
                black_box(value.clone())
                    .powi_checked_with_abort(3, &signal)
                    .unwrap(),
            );
        }
    });
    trace_dispatch_row(format!("matrix_ops/{label}/mat4 powi_negative"), || {
        for value in &lhs4_cases {
            black_box(black_box(value.clone()).powi(-2).unwrap());
        }
    });
    trace_dispatch_row(
        format!("matrix_ops/{label}/mat4 powi_negative_one"),
        || {
            for value in &lhs4_cases {
                black_box(black_box(value.clone()).powi(-1).unwrap());
            }
        },
    );
    trace_dispatch_row(format!("matrix_ops/{label}/mat4 powi_checked_negative"), || {
        for value in &lhs4_cases {
            black_box(black_box(value.clone()).powi_checked(-2).unwrap());
        }
    });
    trace_dispatch_row(
        format!("matrix_ops/{label}/mat4 div_matrix_checked"),
        || {
            for index in 0..lhs4_cases.len() {
                black_box(
                    black_box(lhs4_cases[index].clone())
                        .div_matrix_checked(black_box(rhs4_cases[index].clone()))
                        .unwrap(),
                );
            }
        },
    );
    trace_dispatch_row(
        format!("matrix_ops/{label}/mat4 div_matrix_checked_abort"),
        || {
            for index in 0..lhs4_cases.len() {
                black_box(
                    black_box(lhs4_cases[index].clone())
                        .div_matrix_checked_with_abort(black_box(rhs4_cases[index].clone()), &signal)
                        .unwrap(),
                );
            }
        },
    );
    trace_dispatch_row(format!("matrix_ops/{label}/mat4 direct_div_matrix"), || {
        let divisor = black_box(rhs4_cases[0].clone());
        for index in 0..lhs4_cases.len() {
            black_box(
                black_box(lhs4_cases[index].clone())
                    .div_matrix_checked(black_box(divisor.clone()))
                    .unwrap(),
            );
        }
    });
    trace_dispatch_row(
        format!("matrix_ops/{label}/mat4 direct_div_matrix_exact_left"),
        || {
            let divisor = black_box(rhs4_cases[0].clone());
            for index in 0..lhs4_cases.len() {
                black_box(
                    black_box(lhs4_cases[index].clone())
                        .div_matrix_checked(black_box(divisor.clone()))
                        .unwrap(),
                );
            }
        },
    );
    trace_dispatch_row(format!("matrix_ops/{label}/mat4 direct_div_matrix_checked"), || {
        let divisor = black_box(rhs4_cases[0].clone());
        for index in 0..lhs4_cases.len() {
            black_box(
                black_box(lhs4_cases[index].clone())
                    .div_matrix_checked(black_box(divisor.clone()))
                    .unwrap(),
            );
        }
    });
    trace_dispatch_row(
        format!("matrix_ops/{label}/mat4 direct_div_matrix_checked_abort"),
        || {
            let divisor = black_box(rhs4_cases[0].clone());
            for index in 0..lhs4_cases.len() {
                black_box(
                    black_box(lhs4_cases[index].clone())
                        .div_matrix_checked_with_abort(black_box(divisor.clone()), &signal)
                        .unwrap(),
                );
            }
        },
    );
    trace_dispatch_row(format!("matrix_ops/{label}/mat4 direct_inverse"), || {
        let divisor = black_box(rhs4_cases[0].clone());
        for _ in 0..lhs4_cases.len() {
            black_box(divisor.clone().inverse().unwrap());
        }
    });
    trace_dispatch_row(
        format!("matrix_ops/{label}/mat4 direct_reciprocal"),
        || {
            let divisor = black_box(rhs4_cases[0].clone());
            for _ in 0..lhs4_cases.len() {
                black_box(divisor.clone().reciprocal().unwrap());
            }
        },
    );
    trace_dispatch_row(format!("matrix_ops/{label}/mat4 direct_inverse_checked"), || {
        let divisor = black_box(rhs4_cases[0].clone());
        for _ in 0..lhs4_cases.len() {
            black_box(divisor.clone().inverse_checked().unwrap());
        }
    });
    trace_dispatch_row(
        format!("matrix_ops/{label}/mat4 direct_reciprocal_checked"),
        || {
            let divisor = black_box(rhs4_cases[0].clone());
            for _ in 0..lhs4_cases.len() {
                black_box(divisor.clone().reciprocal_checked().unwrap());
            }
        },
    );
    trace_dispatch_row(
        format!("matrix_ops/{label}/mat4 direct_inverse_checked_abort"),
        || {
            let divisor = black_box(rhs4_cases[0].clone());
            for _ in 0..lhs4_cases.len() {
                black_box(divisor.clone().inverse_checked_with_abort(&signal).unwrap());
            }
        },
    );
    trace_dispatch_row(
        format!("matrix_ops/{label}/mat4 direct_reciprocal_checked_abort"),
        || {
            let divisor = black_box(rhs4_cases[0].clone());
            for _ in 0..lhs4_cases.len() {
                black_box(divisor.clone().inverse_checked_with_abort(&signal).unwrap());
            }
        },
    );
    trace_dispatch_row(format!("matrix_ops/{label}/mat4 direct_powi_negative"), || {
        let divisor = black_box(rhs4_cases[0].clone());
        for _ in 0..lhs4_cases.len() {
            black_box(divisor.clone().powi(-2).unwrap());
        }
    });
    trace_dispatch_row(
        format!("matrix_ops/{label}/mat4 direct_powi_negative_one"),
        || {
            let divisor = black_box(rhs4_cases[0].clone());
            for _ in 0..lhs4_cases.len() {
                black_box(divisor.clone().powi(-1).unwrap());
            }
        },
    );
    trace_dispatch_row(format!("matrix_ops/{label}/mat4 div_matrix"), || {
        for index in 0..lhs4_cases.len() {
            black_box(
                (black_box(lhs4_cases[index].clone()) / black_box(rhs4_cases[index].clone()))
                    .unwrap(),
            );
        }
    });
    trace_dispatch_row(format!("matrix_ops/{label}/mat4 affine_inverse"), || {
        for value in &lhs4_affine_cases {
            black_box(black_box(value.clone()).inverse_checked().unwrap());
        }
    });
    trace_dispatch_row(format!("matrix_ops/{label}/mat4 affine_div_matrix"), || {
        for index in 0..lhs4_affine_cases.len() {
            black_box(
                (black_box(lhs4_affine_cases[index].clone())
                    / black_box(rhs4_affine_cases[index].clone()))
                    .unwrap(),
            );
        }
    });
    trace_dispatch_row(format!("matrix_ops/{label}/mat4 affine_div_matrix_checked"), || {
        for index in 0..lhs4_affine_cases.len() {
            black_box(
                black_box(lhs4_affine_cases[index].clone())
                    .div_matrix_checked(black_box(rhs4_affine_cases[index].clone()))
                    .unwrap(),
            );
        }
    });
    trace_dispatch_row(
        format!("matrix_ops/{label}/mat4 affine_div_matrix_checked_abort"),
        || {
            for index in 0..lhs4_affine_cases.len() {
                black_box(
                    black_box(lhs4_affine_cases[index].clone())
                        .div_matrix_checked_with_abort(
                            black_box(rhs4_affine_cases[index].clone()),
                            &signal,
                        )
                        .unwrap(),
                );
            }
        },
    );
    trace_dispatch_row(
        format!("matrix_ops/{label}/mat4 affine_div_matrix_translation_checked"),
        || {
            for index in 0..lhs4_affine_translation_cases.len() {
                black_box(
                    black_box(lhs4_affine_translation_cases[index].clone())
                        .div_matrix_checked(black_box(rhs4_affine_translation_cases[index].clone()))
                        .unwrap(),
                );
            }
        },
    );
    trace_dispatch_row(
        format!("matrix_ops/{label}/mat4 affine_div_matrix_translation_checked_abort"),
        || {
            for index in 0..lhs4_affine_translation_cases.len() {
                black_box(
                    black_box(lhs4_affine_translation_cases[index].clone())
                        .div_matrix_checked_with_abort(
                            black_box(rhs4_affine_translation_cases[index].clone()),
                            &signal,
                        )
                        .unwrap(),
                );
            }
        },
    );
    trace_dispatch_row(format!("matrix_ops/{label}/mat4 affine_div_matrix_translation"), || {
        for index in 0..lhs4_affine_translation_cases.len() {
            black_box(
                (black_box(lhs4_affine_translation_cases[index].clone())
                    / black_box(rhs4_affine_translation_cases[index].clone()))
                    .unwrap(),
            );
        }
    });
    trace_dispatch_row(
        format!("matrix_ops/{label}/mat4 translated_diagonal_direction_transform"),
        || {
            black_box(
                black_box(&translated_diagonal_direction_matrix)
                    .transform_vec4_direction(black_box(&translated_diagonal_direction)),
            );
        },
    );
    trace_dispatch_row(
        format!("matrix_ops/{label}/mat4 translated_diagonal_point_transform"),
        || {
            black_box(
                black_box(&translated_diagonal_direction_matrix)
                    .transform_vec4_point(black_box(&translated_diagonal_point)),
            );
        },
    );
    trace_dispatch_row(
        format!("matrix_ops/{label}/mat4 identity_direction_transform"),
        || {
            black_box(
                black_box(&identity_matrix4)
                    .transform_vec4_direction(black_box(&translated_diagonal_direction)),
            );
        },
    );
    trace_dispatch_row(
        format!("matrix_ops/{label}/mat4 identity_point_transform"),
        || {
            black_box(
                black_box(&identity_matrix4)
                    .transform_vec4_point(black_box(&translated_diagonal_point)),
            );
        },
    );
    trace_dispatch_row(
        format!("matrix_ops/{label}/mat4 identity_direction_transform_direct"),
        || {
            let matrix = black_box(&identity_matrix4);
            black_box(matrix.transform_vec4_direction(black_box(&translated_diagonal_direction)));
        },
    );
    trace_dispatch_row(
        format!("matrix_ops/{label}/mat4 identity_point_transform_direct"),
        || {
            let matrix = black_box(&identity_matrix4);
            black_box(matrix.transform_vec4_point(black_box(&translated_diagonal_point)));
        },
    );
    trace_dispatch_row(
        format!("matrix_ops/{label}/mat4 identity_direction_transform_generic"),
        || {
            black_box(
                black_box(&identity_matrix4)
                    .transform_vec4(black_box(&translated_diagonal_direction)),
            );
        },
    );
    trace_dispatch_row(
        format!("matrix_ops/{label}/mat4 identity_point_transform_generic"),
        || {
            black_box(
                black_box(&identity_matrix4)
                    .transform_vec4(black_box(&translated_diagonal_point)),
            );
        },
    );
    trace_dispatch_row(
        format!("matrix_ops/{label}/mat4 translated_diagonal_direction_transform_generic"),
        || {
            black_box(
                black_box(&translated_diagonal_direction_matrix)
                    .transform_vec4(black_box(&translated_diagonal_direction)),
            );
        },
    );
    trace_dispatch_row(
        format!("matrix_ops/{label}/mat4 translated_diagonal_point_transform_generic"),
        || {
            black_box(
                black_box(&translated_diagonal_direction_matrix)
                    .transform_vec4(black_box(&translated_diagonal_point)),
            );
        },
    );
    trace_dispatch_row(
        format!("matrix_ops/{label}/mat4 translated_diagonal_direction_batch"),
        || {
            let matrix = black_box(&translated_diagonal_direction_matrix);
            black_box(matrix.transform_vec4_batch(black_box(&translated_diagonal_direction_batch)));
        },
    );
    trace_dispatch_row(
        format!("matrix_ops/{label}/mat4 translated_diagonal_point_batch"),
        || {
            let matrix = black_box(&translated_diagonal_direction_matrix);
            black_box(matrix.transform_vec4_batch(black_box(&translated_diagonal_point_batch)));
        },
    );
    trace_dispatch_row(
        format!("matrix_ops/{label}/mat4 translated_diagonal_direction_batch_assumed"),
        || {
            let matrix = black_box(&translated_diagonal_direction_matrix);
            black_box(matrix.transform_vec4_direction_batch(black_box(&translated_diagonal_direction_batch)));
        },
    );
    trace_dispatch_row(
        format!("matrix_ops/{label}/mat4 translated_diagonal_direction_batch_public_assumed"),
        || {
            black_box(
                black_box(&translated_diagonal_direction_matrix)
                    .transform_vec4_direction_batch(black_box(&translated_diagonal_direction_batch)),
            );
        },
    );
    trace_dispatch_row(
        format!("matrix_ops/{label}/mat4 translated_diagonal_point_batch_assumed"),
        || {
            let matrix = black_box(&translated_diagonal_direction_matrix);
            black_box(matrix.transform_vec4_point_batch(black_box(&translated_diagonal_point_batch)));
        },
    );
    trace_dispatch_row(
        format!("matrix_ops/{label}/mat4 translated_diagonal_point_batch_public_assumed"),
        || {
            black_box(
                black_box(&translated_diagonal_direction_matrix)
                    .transform_vec4_point_batch(black_box(&translated_diagonal_point_batch)),
            );
        },
    );
    trace_dispatch_row(
        format!("matrix_ops/{label}/mat4 diagonal_direction_batch"),
        || {
            let matrix = black_box(&diagonal_affine_matrix);
            black_box(matrix.transform_vec4_batch(black_box(&translated_diagonal_direction_batch)));
        },
    );
    trace_dispatch_row(
        format!("matrix_ops/{label}/mat4 diagonal_point_batch"),
        || {
            let matrix = black_box(&diagonal_affine_matrix);
            black_box(matrix.transform_vec4_batch(black_box(&translated_diagonal_point_batch)));
        },
    );
    trace_dispatch_row(
        format!("matrix_ops/{label}/mat4 identity_direction_batch_assumed"),
        || {
            let matrix = black_box(&identity_matrix4);
            black_box(matrix.transform_vec4_direction_batch(black_box(&translated_diagonal_direction_batch)));
        },
    );
    trace_dispatch_row(
        format!("matrix_ops/{label}/mat4 identity_point_batch_assumed"),
        || {
            let matrix = black_box(&identity_matrix4);
            black_box(matrix.transform_vec4_point_batch(black_box(&translated_diagonal_point_batch)));
        },
    );
    trace_dispatch_row(format!("matrix_ops/{label}/mat4 bitxor"), || {
        for value in &lhs4_cases {
            black_box((black_box(value.clone()) ^ 3).unwrap());
        }
    });

    group.bench_function(format!("{label}/mat3 new"), |b| {
        let raw_cases = sample_mat3_cases();
        let cursor = Cell::new(0);
        b.iter(|| black_box(blas_mat3_with(*next_case(&raw_cases, &cursor), make_scalar)))
    });
    group.bench_function(format!("{label}/mat3 zero"), |b| {
        b.iter(|| black_box(Matrix3::zero()))
    });
    group.bench_function(format!("{label}/mat3 identity"), |b| {
        b.iter(|| black_box(Matrix3::identity()))
    });
    group.bench_function(format!("{label}/mat3 transpose"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| black_box(black_box(next_case(&lhs3_cases, &cursor)).transpose()))
    });
    group.bench_function(format!("{label}/mat3 reciprocal"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                black_box(next_case(&lhs3_cases, &cursor).clone())
                    .reciprocal()
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/mat3 uniform_scale_reciprocal"), |b| {
        b.iter(|| black_box(black_box(uniform_scale_matrix3.clone()).reciprocal().unwrap()))
    });
    group.bench_function(format!("{label}/mat3 known_uniform_scale_inverse"), |b| {
        b.iter(|| {
            black_box(
                Matrix3::uniform_scale_inverse(black_box(uniform_scale_value.clone()))
                    .unwrap(),
            )
        })
    });
        group.bench_function(format!("{label}/mat3 known_diagonal_inverse"), |b| {
            b.iter(|| {
                black_box(Matrix3::diagonal_inverse(black_box(diagonal_values3.clone())).unwrap())
            })
        });
    group.bench_function(format!("{label}/mat3 known_upper_triangular_inverse"), |b| {
        b.iter(|| black_box(black_box(upper_triangular_matrix3.clone()).upper_triangular_inverse().unwrap()))
    });
    group.bench_function(format!("{label}/mat3 known_lower_triangular_inverse"), |b| {
        b.iter(|| black_box(black_box(lower_triangular_matrix3.clone()).lower_triangular_inverse().unwrap()))
    });
    // Bench checked triangular inverse variants to keep error-aware structural
    // dispatch observable at steady state.
    // Golub & Van Loan, *Matrix Computations* (4th ed.); Yap, 1997.
    group.bench_function(format!("{label}/mat3 known_upper_triangular_inverse_checked"), |b| {
        b.iter(|| {
            black_box(
                black_box(upper_triangular_matrix3.clone())
                    .upper_triangular_inverse_checked()
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/mat3 known_lower_triangular_inverse_checked"), |b| {
        b.iter(|| {
            black_box(
                black_box(lower_triangular_matrix3.clone())
                    .lower_triangular_inverse_checked()
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/mat3 known_diagonal_div_matrix"), |b| {
        b.iter(|| {
            black_box(
                black_box(lhs3_cases[0].clone())
                    .div_diagonal(black_box(diagonal_values3.clone()))
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/mat3 known_upper_triangular_div_matrix"), |b| {
        b.iter(|| {
            black_box(
                black_box(lhs3_cases[0].clone())
                    .div_upper_triangular(black_box(upper_triangular_matrix3.clone()))
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/mat3 known_lower_triangular_div_matrix"), |b| {
        b.iter(|| {
            black_box(
                black_box(lhs3_cases[0].clone())
                    .div_lower_triangular(black_box(lower_triangular_matrix3.clone()))
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/mat3 known_diagonal_div_vector"), |b| {
        b.iter(|| {
            black_box(
                black_box(&lhs3_cases[0])
                    .div_diagonal_vector(
                        black_box(diagonal_values3.clone()),
                        black_box(&vector3_cases[0]),
                    )
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/mat3 known_uniform_diagonal_div_vector"), |b| {
        b.iter(|| {
            black_box(
                black_box(&lhs3_cases[0])
                    .div_diagonal_vector(
                        black_box([
                            uniform_scale_value.clone(),
                            uniform_scale_value.clone(),
                            uniform_scale_value.clone(),
                        ]),
                        black_box(&vector3_cases[0]),
                    )
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/mat3 reciprocal_checked"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                black_box(next_case(&lhs3_cases, &cursor).clone())
                    .reciprocal_checked()
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/mat3 inverse_checked"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                black_box(next_case(&lhs3_cases, &cursor).clone())
                    .inverse_checked()
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/mat3 inverse_checked_abort"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                black_box(next_case(&lhs3_cases, &cursor).clone())
                    .inverse_checked_with_abort(&signal)
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/mat3 powi"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                black_box(next_case(&lhs3_cases, &cursor).clone())
                    .powi(3)
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/mat3 powi_checked"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                black_box(next_case(&lhs3_cases, &cursor).clone())
                    .powi_checked(3)
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/mat3 powi_checked_abort"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                black_box(next_case(&lhs3_cases, &cursor).clone())
                    .powi_checked_with_abort(3, &signal)
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/mat3 powi_negative"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                black_box(next_case(&lhs3_cases, &cursor).clone())
                    .powi(-2)
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/mat3 powi_negative_one"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                black_box(next_case(&lhs3_cases, &cursor).clone())
                    .powi(-1)
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/mat3 powi_checked_negative"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                black_box(next_case(&lhs3_cases, &cursor).clone())
                    .powi_checked(-2)
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/mat3 div_scalar_checked"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % lhs3_cases.len());
            black_box(
                black_box(lhs3_cases[index].clone())
                    .div_scalar_checked(black_box(scalar_cases[index].clone()))
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/mat3 div_scalar_checked_abort"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % lhs3_cases.len());
            black_box(
                black_box(lhs3_cases[index].clone())
                    .div_scalar_checked_with_abort(black_box(scalar_cases[index].clone()), &signal)
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/mat3 div_matrix_checked"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % lhs3_cases.len());
            black_box(
                black_box(lhs3_cases[index].clone())
                    .div_matrix_checked(black_box(rhs3_cases[index].clone()))
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/mat3 div_matrix_checked_abort"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % lhs3_cases.len());
            black_box(
                black_box(lhs3_cases[index].clone())
                    .div_matrix_checked_with_abort(black_box(rhs3_cases[index].clone()), &signal)
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/mat3 direct_div_matrix"), |b| {
        let cursor = Cell::new(0);
        let divisor = rhs3_cases[0].clone();
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % lhs3_cases.len());
            black_box(
                black_box(lhs3_cases[index].clone())
                    .div_matrix_checked(black_box(divisor.clone()))
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/mat3 direct_div_matrix_checked"), |b| {
        let cursor = Cell::new(0);
        let divisor = rhs3_cases[0].clone();
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % lhs3_cases.len());
            black_box(
                black_box(lhs3_cases[index].clone())
                    .div_matrix_checked(black_box(divisor.clone()))
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/mat3 direct_div_matrix_checked_abort"), |b| {
        let cursor = Cell::new(0);
        let divisor = rhs3_cases[0].clone();
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % lhs3_cases.len());
            black_box(
                black_box(lhs3_cases[index].clone())
                    .div_matrix_checked_with_abort(black_box(divisor.clone()), &signal)
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/mat3 affine_div_matrix_translation"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % lhs3_affine_translation_cases.len());
            black_box(
                (black_box(lhs3_affine_translation_cases[index].clone())
                    / black_box(rhs3_affine_translation_cases[index].clone()))
                    .unwrap(),
            );
        })
    });
    group.bench_function(format!("{label}/mat3 affine_div_matrix"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % lhs3_affine_cases.len());
            black_box(
                (black_box(lhs3_affine_cases[index].clone()) / black_box(rhs3_affine_cases[index].clone()))
                    .unwrap(),
            );
        })
    });
    group.bench_function(format!("{label}/mat3 affine_inverse"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                black_box(next_case(&lhs3_affine_cases, &cursor).clone())
                    .inverse_checked()
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/mat3 add"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % lhs3_cases.len());
            black_box(black_box(lhs3_cases[index].clone()) + black_box(rhs3_cases[index].clone()))
        })
    });
    group.bench_function(format!("{label}/mat3 add_scalar"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % lhs3_cases.len());
            black_box(black_box(lhs3_cases[index].clone()) + black_box(scalar_cases[index].clone()))
        })
    });
    group.bench_function(format!("{label}/mat3 sub"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % lhs3_cases.len());
            black_box(black_box(lhs3_cases[index].clone()) - black_box(rhs3_cases[index].clone()))
        })
    });
    group.bench_function(format!("{label}/mat3 sub_scalar"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % lhs3_cases.len());
            black_box(black_box(lhs3_cases[index].clone()) - black_box(scalar_cases[index].clone()))
        })
    });
    group.bench_function(format!("{label}/mat3 neg"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| black_box(-black_box(next_case(&lhs3_cases, &cursor).clone())))
    });
    group.bench_function(format!("{label}/mat3 mul_scalar"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % lhs3_cases.len());
            black_box(black_box(lhs3_cases[index].clone()) * black_box(scalar_cases[index].clone()))
        })
    });
    group.bench_function(format!("{label}/mat3 div_scalar"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % lhs3_cases.len());
            black_box(
                (black_box(lhs3_cases[index].clone()) / black_box(scalar_cases[index].clone()))
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/mat3 div_matrix"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % lhs3_cases.len());
            black_box(
                (black_box(lhs3_cases[index].clone()) / black_box(rhs3_cases[index].clone()))
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/mat3 bitxor"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| black_box((black_box(next_case(&lhs3_cases, &cursor).clone()) ^ 3).unwrap()))
    });

    group.bench_function(format!("{label}/mat4 zero"), |b| {
        b.iter(|| black_box(Matrix4::zero()))
    });
    group.bench_function(format!("{label}/mat4 identity"), |b| {
        b.iter(|| black_box(Matrix4::identity()))
    });
    group.bench_function(format!("{label}/mat4 transpose"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| black_box(black_box(next_case(&lhs4_cases, &cursor)).transpose()))
    });
    group.bench_function(format!("{label}/mat4 reciprocal"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                black_box(next_case(&lhs4_cases, &cursor).clone())
                    .reciprocal()
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/mat4 diagonal_reciprocal"), |b| {
        b.iter(|| black_box(black_box(diagonal_affine_matrix.clone()).reciprocal().unwrap()))
    });
    group.bench_function(format!("{label}/mat4 known_translation_inverse"), |b| {
        b.iter(|| {
            black_box(Matrix4::affine_translation_inverse(black_box(
                translation_values4.clone(),
            )))
        })
    });
    group.bench_function(format!("{label}/mat4 known_translation_div_matrix"), |b| {
        b.iter(|| {
            black_box(
                black_box(lhs4_cases[0].clone())
                    .div_affine_translation(black_box(translation_values4.clone())),
            )
        })
    });
    group.bench_function(format!("{label}/mat4 known_orthonormal_inverse"), |b| {
        b.iter(|| {
            black_box(Matrix4::affine_orthonormal_inverse(
                black_box(orthonormal_linear4.clone()),
                black_box(translation_values4.clone()),
            ))
        })
    });
    group.bench_function(format!("{label}/mat4 known_orthonormal_div_matrix"), |b| {
        b.iter(|| {
            black_box(black_box(lhs4_cases[0].clone()).div_affine_orthonormal(
                black_box(orthonormal_linear4.clone()),
                black_box(translation_values4.clone()),
            ))
        })
    });
    group.bench_function(format!("{label}/mat4 known_signed_permutation_inverse"), |b| {
        b.iter(|| {
            black_box(Matrix4::signed_permutation_inverse(black_box(
                signed_permutation_rows4,
            )))
        })
    });
    group.bench_function(format!("{label}/mat4 known_signed_permutation_div_matrix"), |b| {
        b.iter(|| {
            black_box(
                black_box(lhs4_cases[0].clone())
                    .div_signed_permutation(black_box(signed_permutation_rows4)),
            )
        })
    });
    group.bench_function(format!("{label}/mat4 known_signed_permutation_transform"), |b| {
        b.iter(|| {
            black_box(Matrix4::transform_signed_permutation_vector(
                black_box(signed_permutation_rows4),
                black_box(&vector4_cases[0]),
            ))
        })
    });
    group.bench_function(format!("{label}/mat4 known_signed_permutation_batch"), |b| {
        b.iter(|| {
            black_box(Matrix4::transform_signed_permutation_batch(
                black_box(signed_permutation_rows4),
                black_box(&translated_diagonal_point_batch),
            ))
        })
    });
    group.bench_function(format!("{label}/mat4 uniform_scale_reciprocal"), |b| {
        b.iter(|| black_box(black_box(uniform_scale_matrix4.clone()).reciprocal().unwrap()))
    });
    group.bench_function(format!("{label}/mat4 known_uniform_scale_inverse"), |b| {
        b.iter(|| {
            black_box(
                Matrix4::uniform_scale_inverse(black_box(uniform_scale_value.clone()))
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/mat4 known_diagonal_inverse"), |b| {
        b.iter(|| {
            black_box(Matrix4::diagonal_inverse(black_box(diagonal_values4.clone())).unwrap())
        })
    });
    group.bench_function(format!("{label}/mat4 known_upper_triangular_inverse"), |b| {
        b.iter(|| {
            black_box(
                black_box(upper_triangular_matrix4.clone())
                    .upper_triangular_inverse()
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/mat4 known_lower_triangular_inverse"), |b| {
        b.iter(|| {
            black_box(
                black_box(lower_triangular_matrix4.clone())
                    .lower_triangular_inverse()
                    .unwrap(),
            )
        })
    });
    // Bench checked triangular inverse variants to keep error-aware structural
    // dispatch observable at steady state.
    // Golub & Van Loan, *Matrix Computations* (4th ed.); Yap, 1997.
    group.bench_function(format!("{label}/mat4 known_upper_triangular_inverse_checked"), |b| {
        b.iter(|| {
            black_box(
                black_box(upper_triangular_matrix4.clone())
                    .upper_triangular_inverse_checked()
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/mat4 known_lower_triangular_inverse_checked"), |b| {
        b.iter(|| {
            black_box(
                black_box(lower_triangular_matrix4.clone())
                    .lower_triangular_inverse_checked()
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/mat4 known_diagonal_div_matrix"), |b| {
        b.iter(|| {
            black_box(
                black_box(lhs4_cases[0].clone())
                    .div_diagonal(black_box(diagonal_values4.clone()))
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/mat4 known_upper_triangular_div_matrix"), |b| {
        b.iter(|| {
            black_box(
                black_box(lhs4_cases[0].clone())
                    .div_upper_triangular(black_box(upper_triangular_matrix4.clone()))
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/mat4 known_lower_triangular_div_matrix"), |b| {
        b.iter(|| {
            black_box(
                black_box(lhs4_cases[0].clone())
                    .div_lower_triangular(black_box(lower_triangular_matrix4.clone()))
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/mat4 known_diagonal_div_vector"), |b| {
        b.iter(|| {
            black_box(
                black_box(&lhs4_cases[0])
                    .div_diagonal_vector(
                        black_box(diagonal_values4.clone()),
                        black_box(&vector4_cases[0]),
                    )
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/mat4 known_uniform_diagonal_div_vector"), |b| {
        b.iter(|| {
            black_box(
                black_box(&lhs4_cases[0])
                    .div_diagonal_vector(
                        black_box([
                            uniform_scale_value.clone(),
                            uniform_scale_value.clone(),
                            uniform_scale_value.clone(),
                            uniform_scale_value.clone(),
                        ]),
                        black_box(&vector4_cases[0]),
                    )
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/mat4 known_diagonal_div_vector_direction"), |b| {
        b.iter(|| {
            black_box(
                black_box(&lhs4_cases[0])
                    .div_diagonal_vector(
                        black_box(diagonal_values4.clone()),
                        black_box(&translated_diagonal_direction),
                    )
                    .unwrap(),
            )
        })
    });
    group.bench_function(
        format!("{label}/mat4 known_uniform_diagonal_div_vector_direction"),
        |b| {
            b.iter(|| {
                black_box(
                    black_box(&lhs4_cases[0])
                        .div_diagonal_vector(
                            black_box([
                                uniform_scale_value.clone(),
                                uniform_scale_value.clone(),
                                uniform_scale_value.clone(),
                                uniform_scale_value.clone(),
                            ]),
                            black_box(&translated_diagonal_direction),
                        )
                        .unwrap(),
                )
            })
        },
    );
    group.bench_function(format!("{label}/mat4 known_diagonal_div_vector_direction_only"), |b| {
        b.iter(|| {
            black_box(
                black_box(&lhs4_cases[0])
                    .div_diagonal_direction_vector(
                        black_box(diagonal_values4.clone()),
                        black_box(&translated_diagonal_direction),
                    )
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/mat4 known_diagonal_div_vector_point"), |b| {
        b.iter(|| {
            black_box(
                black_box(&lhs4_cases[0])
                    .div_diagonal_vector(
                        black_box(diagonal_values4.clone()),
                        black_box(&translated_diagonal_point),
                    )
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/mat4 known_uniform_diagonal_div_vector_point"), |b| {
        b.iter(|| {
            black_box(
                black_box(&lhs4_cases[0])
                    .div_diagonal_vector(
                        black_box([
                            uniform_scale_value.clone(),
                            uniform_scale_value.clone(),
                            uniform_scale_value.clone(),
                            uniform_scale_value.clone(),
                        ]),
                        black_box(&translated_diagonal_point),
                    )
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/mat4 reciprocal_checked"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                black_box(next_case(&lhs4_cases, &cursor).clone())
                    .reciprocal_checked()
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/mat4 inverse_checked"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                black_box(next_case(&lhs4_cases, &cursor).clone())
                    .inverse_checked()
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/mat4 inverse_checked_abort"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                black_box(next_case(&lhs4_cases, &cursor).clone())
                    .inverse_checked_with_abort(&signal)
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/mat4 powi"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                black_box(next_case(&lhs4_cases, &cursor).clone())
                    .powi(3)
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/mat4 powi_checked"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                black_box(next_case(&lhs4_cases, &cursor).clone())
                    .powi_checked(3)
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/mat4 powi_checked_abort"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                black_box(next_case(&lhs4_cases, &cursor).clone())
                    .powi_checked_with_abort(3, &signal)
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/mat4 powi_negative"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                black_box(next_case(&lhs4_cases, &cursor).clone())
                    .powi(-2)
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/mat4 powi_negative_one"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                black_box(next_case(&lhs4_cases, &cursor).clone())
                    .powi(-1)
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/mat4 powi_checked_negative"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                black_box(next_case(&lhs4_cases, &cursor).clone())
                    .powi_checked(-2)
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/mat4 div_scalar_checked"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % lhs4_cases.len());
            black_box(
                black_box(lhs4_cases[index].clone())
                    .div_scalar_checked(black_box(scalar_cases[index].clone()))
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/mat4 div_scalar_checked_abort"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % lhs4_cases.len());
            black_box(
                black_box(lhs4_cases[index].clone())
                    .div_scalar_checked_with_abort(black_box(scalar_cases[index].clone()), &signal)
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/mat4 add"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % lhs4_cases.len());
            black_box(black_box(lhs4_cases[index].clone()) + black_box(rhs4_cases[index].clone()))
        })
    });
    group.bench_function(format!("{label}/mat4 add_scalar"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % lhs4_cases.len());
            black_box(black_box(lhs4_cases[index].clone()) + black_box(scalar_cases[index].clone()))
        })
    });
    group.bench_function(format!("{label}/mat4 sub"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % lhs4_cases.len());
            black_box(black_box(lhs4_cases[index].clone()) - black_box(rhs4_cases[index].clone()))
        })
    });
    group.bench_function(format!("{label}/mat4 sub_scalar"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % lhs4_cases.len());
            black_box(black_box(lhs4_cases[index].clone()) - black_box(scalar_cases[index].clone()))
        })
    });
    group.bench_function(format!("{label}/mat4 neg"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| black_box(-black_box(next_case(&lhs4_cases, &cursor).clone())))
    });
    group.bench_function(format!("{label}/mat4 mul_scalar"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % lhs4_cases.len());
            black_box(black_box(lhs4_cases[index].clone()) * black_box(scalar_cases[index].clone()))
        })
    });
    group.bench_function(format!("{label}/mat4 div_scalar"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % lhs4_cases.len());
            black_box(
                (black_box(lhs4_cases[index].clone()) / black_box(scalar_cases[index].clone()))
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/mat4 div_matrix"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % lhs4_cases.len());
            black_box(
                (black_box(lhs4_cases[index].clone()) / black_box(rhs4_cases[index].clone()))
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/mat4 affine_div_matrix_translation"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % lhs4_affine_translation_cases.len());
            black_box(
                (black_box(lhs4_affine_translation_cases[index].clone())
                    / black_box(rhs4_affine_translation_cases[index].clone()))
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/mat4 affine_div_matrix_checked"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % lhs4_affine_cases.len());
            black_box(
                black_box(lhs4_affine_cases[index].clone())
                    .div_matrix_checked(black_box(rhs4_affine_cases[index].clone()))
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/mat4 affine_div_matrix_checked_abort"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % lhs4_affine_cases.len());
            black_box(
                black_box(lhs4_affine_cases[index].clone())
                    .div_matrix_checked_with_abort(
                        black_box(rhs4_affine_cases[index].clone()),
                        &signal,
                    )
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/mat4 affine_div_matrix_translation_checked"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % lhs4_affine_translation_cases.len());
            black_box(
                black_box(lhs4_affine_translation_cases[index].clone())
                    .div_matrix_checked(black_box(rhs4_affine_translation_cases[index].clone()))
                    .unwrap(),
            )
        })
    });
    group.bench_function(
        format!("{label}/mat4 affine_div_matrix_translation_checked_abort"),
        |b| {
            let cursor = Cell::new(0);
            b.iter(|| {
                let index = cursor.get();
                cursor.set((index + 1) % lhs4_affine_translation_cases.len());
                black_box(
                    black_box(lhs4_affine_translation_cases[index].clone())
                        .div_matrix_checked_with_abort(
                            black_box(rhs4_affine_translation_cases[index].clone()),
                            &signal,
                        )
                        .unwrap(),
                )
            })
        },
    );
    group.bench_function(format!("{label}/mat4 div_matrix_checked"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % lhs4_cases.len());
            black_box(
                black_box(lhs4_cases[index].clone())
                    .div_matrix_checked(black_box(rhs4_cases[index].clone()))
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/mat4 div_matrix_checked_abort"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % lhs4_cases.len());
            black_box(
                black_box(lhs4_cases[index].clone())
                    .div_matrix_checked_with_abort(black_box(rhs4_cases[index].clone()), &signal)
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/mat4 direct_div_matrix"), |b| {
        let cursor = Cell::new(0);
        let divisor = rhs4_cases[0].clone();
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % lhs4_cases.len());
            black_box(
                black_box(lhs4_cases[index].clone())
                    .div_matrix_checked(black_box(divisor.clone()))
                    .unwrap(),
            )
        })
    });
    group.bench_function(
        format!("{label}/mat4 direct_div_matrix_exact_left"),
        |b| {
            let cursor = Cell::new(0);
            let divisor = rhs4_cases[0].clone();
            b.iter(|| {
                let index = cursor.get();
                cursor.set((index + 1) % lhs4_cases.len());
                black_box(
                    black_box(lhs4_cases[index].clone())
                        .div_matrix_checked(black_box(divisor.clone()))
                        .unwrap(),
                )
            })
        },
    );
    group.bench_function(format!("{label}/mat4 direct_div_matrix_checked"), |b| {
        let cursor = Cell::new(0);
        let divisor = rhs4_cases[0].clone();
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % lhs4_cases.len());
            black_box(
                black_box(lhs4_cases[index].clone())
                    .div_matrix_checked(black_box(divisor.clone()))
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/mat4 direct_div_matrix_checked_abort"), |b| {
        let cursor = Cell::new(0);
        let divisor = rhs4_cases[0].clone();
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % lhs4_cases.len());
            black_box(
                black_box(lhs4_cases[index].clone())
                    .div_matrix_checked_with_abort(black_box(divisor.clone()), &signal)
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/mat4 direct_inverse"), |b| {
        let divisor = rhs4_cases[0].clone();
        b.iter(|| black_box(divisor.clone().inverse().unwrap()))
    });
    group.bench_function(format!("{label}/mat4 direct_reciprocal"), |b| {
        let divisor = rhs4_cases[0].clone();
        b.iter(|| black_box(divisor.clone().reciprocal().unwrap()))
    });
    group.bench_function(format!("{label}/mat4 direct_inverse_checked"), |b| {
        let divisor = rhs4_cases[0].clone();
        b.iter(|| black_box(divisor.clone().inverse_checked().unwrap()))
    });
    group.bench_function(format!("{label}/mat4 direct_reciprocal_checked"), |b| {
        let divisor = rhs4_cases[0].clone();
        b.iter(|| black_box(divisor.clone().reciprocal_checked().unwrap()))
    });
    group.bench_function(format!("{label}/mat4 direct_inverse_checked_abort"), |b| {
        let divisor = rhs4_cases[0].clone();
        b.iter(|| black_box(divisor.clone().inverse_checked_with_abort(&signal).unwrap()))
    });
    group.bench_function(
        format!("{label}/mat4 direct_reciprocal_checked_abort"),
        |b| {
            let divisor = rhs4_cases[0].clone();
            b.iter(|| black_box(divisor.clone().inverse_checked_with_abort(&signal).unwrap()))
        },
    );
    group.bench_function(format!("{label}/mat4 direct_powi_negative"), |b| {
        let divisor = rhs4_cases[0].clone();
        b.iter(|| black_box(divisor.clone().powi(-2).unwrap()))
    });
    group.bench_function(format!("{label}/mat4 direct_powi_negative_one"), |b| {
        let divisor = rhs4_cases[0].clone();
        b.iter(|| black_box(divisor.clone().powi(-1).unwrap()))
    });
    group.bench_function(
        format!("{label}/mat4 translated_diagonal_direction_transform"),
        |b| {
            b.iter(|| {
                black_box(
                    translated_diagonal_direction_matrix
                        .transform_vec4_direction(black_box(&translated_diagonal_direction)),
                )
            })
        },
    );
    group.bench_function(format!("{label}/mat4 translated_diagonal_direction_batch"), |b| {
        let matrix = &translated_diagonal_direction_matrix;
        b.iter(|| black_box(matrix.transform_vec4_batch(black_box(&translated_diagonal_direction_batch))))
    });
    group.bench_function(format!("{label}/mat4 translated_diagonal_direction_batch_assumed"), |b| {
        let matrix = &translated_diagonal_direction_matrix;
        b.iter(|| black_box(matrix.transform_vec4_direction_batch(black_box(&translated_diagonal_direction_batch))))
    });
    group.bench_function(
        format!("{label}/mat4 translated_diagonal_direction_batch_public_assumed"),
        |b| {
            b.iter(|| {
                black_box(
                    translated_diagonal_direction_matrix
                        .transform_vec4_direction_batch(black_box(&translated_diagonal_direction_batch)),
                )
            })
        },
    );
    group.bench_function(format!("{label}/mat4 identity_direction_transform"), |b| {
        b.iter(|| {
            black_box(
                identity_matrix4.transform_vec4_direction(black_box(&translated_diagonal_direction)),
            )
        })
    });
    group.bench_function(format!("{label}/mat4 identity_direction_transform_direct"), |b| {
        let matrix = &identity_matrix4;
        b.iter(|| {
            black_box(matrix.transform_vec4_direction(black_box(
                &translated_diagonal_direction,
            )))
        })
    });
    group.bench_function(format!("{label}/mat4 identity_point_transform"), |b| {
        b.iter(|| {
            black_box(identity_matrix4.transform_vec4_point(black_box(
                &translated_diagonal_point,
            )))
        })
    });
    group.bench_function(format!("{label}/mat4 identity_point_transform_direct"), |b| {
        let matrix = &identity_matrix4;
        b.iter(|| black_box(matrix.transform_vec4_point(black_box(&translated_diagonal_point))))
    });
    group.bench_function(format!("{label}/mat4 identity_direction_transform_generic"), |b| {
        b.iter(|| {
            black_box(
                identity_matrix4
                    .transform_vec4(black_box(&translated_diagonal_direction)),
            )
        })
    });
    group.bench_function(format!("{label}/mat4 identity_point_transform_generic"), |b| {
        b.iter(|| {
            black_box(
                identity_matrix4
                    .transform_vec4(black_box(&translated_diagonal_point)),
            )
        })
    });
    group.bench_function(
        format!("{label}/mat4 translated_diagonal_direction_transform_generic"),
        |b| {
            b.iter(|| {
                black_box(
                    translated_diagonal_direction_matrix
                        .transform_vec4(black_box(&translated_diagonal_direction)),
                )
            })
        },
    );
    group.bench_function(
        format!("{label}/mat4 translated_diagonal_point_transform_generic"),
        |b| {
            b.iter(|| {
                black_box(
                    translated_diagonal_direction_matrix
                        .transform_vec4(black_box(&translated_diagonal_point)),
                )
            })
        },
    );
    group.bench_function(format!("{label}/mat4 translated_diagonal_point_batch"), |b| {
        let matrix = &translated_diagonal_direction_matrix;
        b.iter(|| black_box(matrix.transform_vec4_batch(black_box(&translated_diagonal_point_batch))))
    });
    group.bench_function(format!("{label}/mat4 translated_diagonal_point_batch_assumed"), |b| {
        let matrix = &translated_diagonal_direction_matrix;
        b.iter(|| black_box(matrix.transform_vec4_point_batch(black_box(&translated_diagonal_point_batch))))
    });
    group.bench_function(
        format!("{label}/mat4 translated_diagonal_point_batch_public_assumed"),
        |b| {
            b.iter(|| {
                black_box(
                    translated_diagonal_direction_matrix
                        .transform_vec4_point_batch(black_box(&translated_diagonal_point_batch)),
                )
            })
        },
    );
    group.bench_function(format!("{label}/mat4 identity_direction_batch_assumed"), |b| {
        let matrix = &identity_matrix4;
        b.iter(|| black_box(matrix.transform_vec4_direction_batch(black_box(&translated_diagonal_direction_batch))))
    });
    group.bench_function(format!("{label}/mat4 identity_point_batch_assumed"), |b| {
        let matrix = &identity_matrix4;
        b.iter(|| black_box(matrix.transform_vec4_point_batch(black_box(&translated_diagonal_point_batch))))
    });
    group.bench_function(format!("{label}/mat4 bitxor"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| black_box((black_box(next_case(&lhs4_cases, &cursor).clone()) ^ 3).unwrap()))
    });
}

/// Registers the matrix_ops comparison group for two native models and three adapters.
///
/// Calls the native registrar first with s (exact import of binary-float values),
/// then with qr (rational parsing of each float's displayed decimal string). The
/// second model can therefore denote a different input; neither recovers an intended
/// symbolic pi from a floating constant. External registrars follow in Numerica-label,
/// GMP and Atom order, each adding forty rows under the inherited Criterion settings.
///
/// The Numerica label uses Symbolica Float, GMP uses Rug/MPFR, and both request 128
/// at construction; the Atom context stores 128 without using it. This wrapper closes
/// the group but asserts no shared numerical accuracy, output-materialization contract,
/// certainty receipt or dispatch schedule across those differently implemented models.
fn bench_matrix_operations(c: &mut Criterion) {
    let mut group = c.benchmark_group("matrix_ops");
    bench_matrix_operations_for::<_>(&mut group, "hyperreal", s);
    bench_matrix_operations_for::<_>(&mut group, "hyperreal-rational", qr);
    bench_numerica_matrix_operations(&mut group, "numerica128");
    bench_gmp_matrix_operations(&mut group, "gmp_mpfr128");
    bench_symbolica_matrix_operations(&mut group, "symbolica");
    group.finish();
}

/// Registers twenty-four fixed-form timing rows and matching native diagnostic hooks.
///
/// Builds all four TargetedMatrixForm fixtures once through make_ratio, then processes
/// dyadic_dense, equal_decimal_den, mixed_prime_den and sparse_integer in that order.
/// Copy/Fn does not prove converter purity or preservation of zeros, ratios or labels;
/// conversion owns input meaning and failures. Labels describe authored coefficient
/// patterns, not actual reduced/intermediate denominators or guaranteed dispatch.
///
/// Each form contributes reciprocal, power -2 and right matrix division for both
/// sizes. Six enabled/selected hooks precede the form's six timing registrations,
/// requesting one operation each on the same retained inputs. There is no cursor or
/// cache reset; earlier hooks or measurements may affect reusable scalar evidence.
/// Ordinary iter includes input clones, operation, Result unwrap and output destruction.
/// Fixture construction is outside timing, and no lossy coordinate export is requested.
///
/// No value, count, nonsingularity or reducer-path assertion is made. Invalid callback
/// outputs or operation failures can panic at unwrap. There is no policy/certainty
/// receipt, cancellation input or total work/memory bound. This separate matrix_forms
/// group studies intended coefficient patterns without adding external comparison rows;
/// historical timing notes are not a current-revision completion/performance guarantee.
fn bench_targeted_matrix_forms_for<F>(
    group: &mut BenchmarkGroup<'_, criterion::measurement::WallTime>,
    label: &str,
    make_ratio: F,
) where
    F: Copy + Fn(i64, u64) -> Real,
{
    let forms = targeted_matrix_forms_with(make_ratio);
    for form in forms {
        let TargetedMatrixForm {
            name,
            lhs3,
            rhs3,
            lhs4,
            rhs4,
        } = form;

        // Pattern labels describe the authored ratio tables, not guaranteed reducer paths.
        // Conversion, fraction reduction and intermediate cofactors can change denominators.
        // This separate group records selected diagnostic work without asserting its counts.
        // The former 2026-05-09 timing/dispatch narrative is retained, with its unverified
        // current-revision status, in the Rustdoc audit ledger's matrix-helper record.
        trace_dispatch_row(
            format!("matrix_forms/{label}/{name}/mat3 reciprocal"),
            || {
                black_box(black_box(lhs3.clone()).reciprocal().unwrap());
            },
        );
        trace_dispatch_row(
            format!("matrix_forms/{label}/{name}/mat3 powi_negative"),
            || {
                black_box(black_box(lhs3.clone()).powi(-2).unwrap());
            },
        );
        trace_dispatch_row(
            format!("matrix_forms/{label}/{name}/mat3 div_matrix"),
            || {
                black_box((black_box(lhs3.clone()) / black_box(rhs3.clone())).unwrap());
            },
        );
        trace_dispatch_row(
            format!("matrix_forms/{label}/{name}/mat4 reciprocal"),
            || {
                black_box(black_box(lhs4.clone()).reciprocal().unwrap());
            },
        );
        trace_dispatch_row(
            format!("matrix_forms/{label}/{name}/mat4 powi_negative"),
            || {
                black_box(black_box(lhs4.clone()).powi(-2).unwrap());
            },
        );
        trace_dispatch_row(
            format!("matrix_forms/{label}/{name}/mat4 div_matrix"),
            || {
                black_box((black_box(lhs4.clone()) / black_box(rhs4.clone())).unwrap());
            },
        );

        group.bench_function(format!("{label}/{name}/mat3 reciprocal"), |b| {
            b.iter(|| black_box(black_box(lhs3.clone()).reciprocal().unwrap()))
        });
        group.bench_function(format!("{label}/{name}/mat3 powi_negative"), |b| {
            b.iter(|| black_box(black_box(lhs3.clone()).powi(-2).unwrap()))
        });
        group.bench_function(format!("{label}/{name}/mat3 div_matrix"), |b| {
            b.iter(|| black_box((black_box(lhs3.clone()) / black_box(rhs3.clone())).unwrap()))
        });
        group.bench_function(format!("{label}/{name}/mat4 reciprocal"), |b| {
            b.iter(|| black_box(black_box(lhs4.clone()).reciprocal().unwrap()))
        });
        group.bench_function(format!("{label}/{name}/mat4 powi_negative"), |b| {
            b.iter(|| black_box(black_box(lhs4.clone()).powi(-2).unwrap()))
        });
        group.bench_function(format!("{label}/{name}/mat4 div_matrix"), |b| {
            b.iter(|| black_box((black_box(lhs4.clone()) / black_box(rhs4.clone())).unwrap()))
        });
    }
}

/// Rounds an integer ratio through f64, then imports that finite binary value exactly.
///
/// Both integer casts and the division occur before Real::try_from; the result need
/// not equal numerator/denominator as an exact rational. Large integers can lose bits
/// at the casts, and a non-dyadic quotient is rounded by floating division. Every
/// nonzero u64 denominator gives a finite quotient over this integer input range.
/// A zero denominator produces infinity or NaN and panics at unwrap. The dyadic name
/// describes the imported finite float, not a promise that the authored ratio was dyadic.
fn dyadic_from_f64_ratio(numerator: i64, denominator: u64) -> Real {
    Real::try_from(numerator as f64 / denominator as f64).unwrap()
}

/// Registers forty-eight targeted native rows, with forty-eight available trace hooks.
///
/// The hyperreal label uses dyadic_from_f64_ratio, rounding casts/division before exact
/// float import. The hyperreal-rational label uses q for exact integer ratios, not qr's
/// float-to-decimal route used by matrix_ops. Current small dyadic/integer tables agree
/// between these models; decimal/prime tables can differ. Each model contributes four
/// forms times six operations under inherited Criterion settings. No cross-model value,
/// reducer-count, output-accuracy or performance assertion is made before group finish.
fn bench_targeted_matrix_forms(c: &mut Criterion) {
    let mut group = c.benchmark_group("matrix_forms");
    bench_targeted_matrix_forms_for::<_>(&mut group, "hyperreal", dyadic_from_f64_ratio);
    bench_targeted_matrix_forms_for::<_>(&mut group, "hyperreal-rational", q);
    group.finish();
}

/// Registers forty floating matrix rows under the historical Numerica comparison label.
///
/// Uses the local Symbolica Float adapter with requested construction precision 128,
/// not a separate Numerica library. Ordinary scalar/matrix adapter work supplies the
/// numerical behavior; shared row names do not establish equivalence to native exact
/// operations or an independently certified accuracy contract.
///
/// Registers twenty-four Mat3 rows and sixteen Mat4 rows. Mat3 has construction,
/// zero, identity, transpose, four inverse labels, four cube labels, three scalar-
/// division labels, three right-division labels and six elementwise/scalar operations.
/// Mat4 has zero/identity/transpose, two inverse labels, three cube labels and eight
/// other operations; it has no construction row here. Every power/bitxor label calls
/// powi(3), and checked/abort variants call the same ordinary adapter methods without
/// native guards, typed Results or cancellation signals. Matrix division is lhs times
/// inverse(rhs); scalar division maps over every entry.
///
/// Retains four paired matrix cases and four scalar values outside timing. Scalars
/// are imported from 2, 1e-9, -1e9 and floating PI. Each cycling row initializes its
/// cursor outside iter, then measures selection/index advancement and borrowed adapter
/// calls, including internal cloning and result destruction. The Mat3 new row instead
/// imports a rotating raw case inside iter; zero/identity construct inside iter. There
/// is no explicit output float conversion, numerical oracle, dispatch hook/count check,
/// certainty receipt or total-work/memory limit. Adapter domain failures are not given
/// an independent wrapper here; Mat4 inverse retains its no-pivoting restriction.
fn bench_numerica_matrix_operations(
    group: &mut BenchmarkGroup<'_, criterion::measurement::WallTime>,
    label: &str,
) {
    let ctx = numerica_engine::Ctx::new(128);
    let lhs3_cases = sample_mat3_cases().map(|value| numerica_engine::Mat3::new(&ctx, value.m));
    let rhs3_cases = sample_mat3_b_cases().map(|value| numerica_engine::Mat3::new(&ctx, value.m));
    let lhs4_cases = sample_mat4_cases().map(|value| numerica_engine::Mat4::new(&ctx, value.m));
    let rhs4_cases = sample_mat4_b_cases().map(|value| numerica_engine::Mat4::new(&ctx, value.m));
    let scalar_cases = [2.0, 1.0e-9, -1.0e9, std::f64::consts::PI].map(|value| ctx.f(value));

    group.bench_function(format!("{label}/mat3 new"), |b| {
        let raw_cases = sample_mat3_cases();
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(numerica_engine::Mat3::new(
                &ctx,
                next_case(&raw_cases, &cursor).m,
            ))
        })
    });
    group.bench_function(format!("{label}/mat3 zero"), |b| {
        b.iter(|| black_box(numerica_engine::Mat3::zero(&ctx)))
    });
    group.bench_function(format!("{label}/mat3 identity"), |b| {
        b.iter(|| black_box(numerica_engine::Mat3::identity(&ctx)))
    });
    group.bench_function(format!("{label}/mat3 transpose"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| black_box(next_case(&lhs3_cases, &cursor).transpose()))
    });
    for name in [
        "reciprocal",
        "reciprocal_checked",
        "inverse_checked",
        "inverse_checked_abort",
    ] {
        group.bench_function(format!("{label}/mat3 {name}"), |b| {
            let cursor = Cell::new(0);
            b.iter(|| black_box(next_case(&lhs3_cases, &cursor).inverse(&ctx)))
        });
    }
    for name in ["powi", "powi_checked", "powi_checked_abort", "bitxor"] {
        group.bench_function(format!("{label}/mat3 {name}"), |b| {
            let cursor = Cell::new(0);
            b.iter(|| black_box(next_case(&lhs3_cases, &cursor).powi(3, &ctx)))
        });
    }
    for name in [
        "div_scalar_checked",
        "div_scalar_checked_abort",
        "div_scalar",
    ] {
        group.bench_function(format!("{label}/mat3 {name}"), |b| {
            let cursor = Cell::new(0);
            b.iter(|| {
                let index = cursor.get();
                cursor.set((index + 1) % lhs3_cases.len());
                black_box(lhs3_cases[index].map_scalar(
                    &scalar_cases[index],
                    &ctx,
                    numerica_engine::Ctx::div,
                ))
            })
        });
    }
    for name in [
        "div_matrix_checked",
        "div_matrix_checked_abort",
        "div_matrix",
    ] {
        group.bench_function(format!("{label}/mat3 {name}"), |b| {
            let cursor = Cell::new(0);
            b.iter(|| {
                let index = cursor.get();
                cursor.set((index + 1) % lhs3_cases.len());
                black_box(lhs3_cases[index].div_matrix(&rhs3_cases[index], &ctx))
            })
        });
    }
    for name in [
        "add",
        "add_scalar",
        "sub",
        "sub_scalar",
        "neg",
        "mul_scalar",
    ] {
        group.bench_function(format!("{label}/mat3 {name}"), |b| {
            let cursor = Cell::new(0);
            b.iter(|| {
                let index = cursor.get();
                cursor.set((index + 1) % lhs3_cases.len());
                black_box(match name {
                    "add" => lhs3_cases[index].combine(
                        &rhs3_cases[index],
                        &ctx,
                        numerica_engine::Ctx::add,
                    ),
                    "add_scalar" => lhs3_cases[index].map_scalar(
                        &scalar_cases[index],
                        &ctx,
                        numerica_engine::Ctx::add,
                    ),
                    "sub" => lhs3_cases[index].combine(
                        &rhs3_cases[index],
                        &ctx,
                        numerica_engine::Ctx::sub,
                    ),
                    "sub_scalar" => lhs3_cases[index].map_scalar(
                        &scalar_cases[index],
                        &ctx,
                        numerica_engine::Ctx::sub,
                    ),
                    "neg" => lhs3_cases[index].neg(&ctx),
                    _ => lhs3_cases[index].map_scalar(
                        &scalar_cases[index],
                        &ctx,
                        numerica_engine::Ctx::mul,
                    ),
                })
            })
        });
    }

    group.bench_function(format!("{label}/mat4 zero"), |b| {
        b.iter(|| black_box(numerica_engine::Mat4::zero(&ctx)))
    });
    group.bench_function(format!("{label}/mat4 identity"), |b| {
        b.iter(|| black_box(numerica_engine::Mat4::identity(&ctx)))
    });
    group.bench_function(format!("{label}/mat4 transpose"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| black_box(next_case(&lhs4_cases, &cursor).transpose()))
    });
    for name in ["reciprocal", "reciprocal_checked"] {
        group.bench_function(format!("{label}/mat4 {name}"), |b| {
            let cursor = Cell::new(0);
            b.iter(|| black_box(next_case(&lhs4_cases, &cursor).inverse(&ctx)))
        });
    }
    for name in ["powi", "powi_checked", "bitxor"] {
        group.bench_function(format!("{label}/mat4 {name}"), |b| {
            let cursor = Cell::new(0);
            b.iter(|| black_box(next_case(&lhs4_cases, &cursor).powi(3, &ctx)))
        });
    }
    for name in [
        "div_scalar",
        "add",
        "add_scalar",
        "sub",
        "sub_scalar",
        "neg",
        "mul_scalar",
        "div_matrix",
    ] {
        group.bench_function(format!("{label}/mat4 {name}"), |b| {
            let cursor = Cell::new(0);
            b.iter(|| {
                let index = cursor.get();
                cursor.set((index + 1) % lhs4_cases.len());
                black_box(match name {
                    "div_scalar" => lhs4_cases[index].map_scalar(
                        &scalar_cases[index],
                        &ctx,
                        numerica_engine::Ctx::div,
                    ),
                    "add" => lhs4_cases[index].combine(
                        &rhs4_cases[index],
                        &ctx,
                        numerica_engine::Ctx::add,
                    ),
                    "add_scalar" => lhs4_cases[index].map_scalar(
                        &scalar_cases[index],
                        &ctx,
                        numerica_engine::Ctx::add,
                    ),
                    "sub" => lhs4_cases[index].combine(
                        &rhs4_cases[index],
                        &ctx,
                        numerica_engine::Ctx::sub,
                    ),
                    "sub_scalar" => lhs4_cases[index].map_scalar(
                        &scalar_cases[index],
                        &ctx,
                        numerica_engine::Ctx::sub,
                    ),
                    "neg" => lhs4_cases[index].neg(&ctx),
                    "mul_scalar" => lhs4_cases[index].map_scalar(
                        &scalar_cases[index],
                        &ctx,
                        numerica_engine::Ctx::mul,
                    ),
                    _ => lhs4_cases[index].div_matrix(&rhs4_cases[index], &ctx),
                })
            })
        });
    }
}

/// Registers forty matrix rows through the local Rug/MPFR adapter at requested precision 128.
///
/// Backend scalar operations clone their inputs internally, and composite adapter
/// formulas add their own work. These costs remain inside the timed call; shared
/// native/external row names do not establish identical guards or completed work.
///
/// Registers twenty-four Mat3 rows and sixteen Mat4 rows. Mat3 has construction,
/// zero, identity, transpose, four inverse labels, four cube labels, three scalar-
/// division labels, three right-division labels and six elementwise/scalar operations.
/// Mat4 has zero/identity/transpose, two inverse labels, three cube labels and eight
/// other operations; it has no construction row here. Every power/bitxor label calls
/// powi(3), and checked/abort variants call the same ordinary adapter methods without
/// native guards, typed Results or cancellation signals. Matrix division is lhs times
/// inverse(rhs); scalar division maps over every entry.
///
/// Retains four paired matrix cases and four scalar values outside timing. Scalars
/// are imported from 2, 1e-9, -1e9 and floating PI. Each cycling row initializes its
/// cursor outside iter, then measures selection/index advancement and borrowed adapter
/// calls, including internal cloning and result destruction. The Mat3 new row instead
/// imports a rotating raw case inside iter; zero/identity construct inside iter. There
/// is no explicit output float conversion, numerical oracle, dispatch hook/count check,
/// certainty receipt or total-work/memory limit. Adapter domain failures are not given
/// an independent wrapper here; Mat4 inverse retains its no-pivoting restriction.
fn bench_gmp_matrix_operations(
    group: &mut BenchmarkGroup<'_, criterion::measurement::WallTime>,
    label: &str,
) {
    let ctx = gmp_engine::Ctx::new(128);
    let lhs3_cases = sample_mat3_cases().map(|value| gmp_engine::Mat3::new(&ctx, value.m));
    let rhs3_cases = sample_mat3_b_cases().map(|value| gmp_engine::Mat3::new(&ctx, value.m));
    let lhs4_cases = sample_mat4_cases().map(|value| gmp_engine::Mat4::new(&ctx, value.m));
    let rhs4_cases = sample_mat4_b_cases().map(|value| gmp_engine::Mat4::new(&ctx, value.m));
    let scalar_cases = [2.0, 1.0e-9, -1.0e9, std::f64::consts::PI].map(|value| ctx.f(value));

    group.bench_function(format!("{label}/mat3 new"), |b| {
        let raw_cases = sample_mat3_cases();
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(gmp_engine::Mat3::new(
                &ctx,
                next_case(&raw_cases, &cursor).m,
            ))
        })
    });
    group.bench_function(format!("{label}/mat3 zero"), |b| {
        b.iter(|| black_box(gmp_engine::Mat3::zero(&ctx)))
    });
    group.bench_function(format!("{label}/mat3 identity"), |b| {
        b.iter(|| black_box(gmp_engine::Mat3::identity(&ctx)))
    });
    group.bench_function(format!("{label}/mat3 transpose"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| black_box(next_case(&lhs3_cases, &cursor).transpose()))
    });
    for name in [
        "reciprocal",
        "reciprocal_checked",
        "inverse_checked",
        "inverse_checked_abort",
    ] {
        group.bench_function(format!("{label}/mat3 {name}"), |b| {
            let cursor = Cell::new(0);
            b.iter(|| black_box(next_case(&lhs3_cases, &cursor).inverse(&ctx)))
        });
    }
    for name in ["powi", "powi_checked", "powi_checked_abort", "bitxor"] {
        group.bench_function(format!("{label}/mat3 {name}"), |b| {
            let cursor = Cell::new(0);
            b.iter(|| black_box(next_case(&lhs3_cases, &cursor).powi(3, &ctx)))
        });
    }
    for name in [
        "div_scalar_checked",
        "div_scalar_checked_abort",
        "div_scalar",
    ] {
        group.bench_function(format!("{label}/mat3 {name}"), |b| {
            let cursor = Cell::new(0);
            b.iter(|| {
                let index = cursor.get();
                cursor.set((index + 1) % lhs3_cases.len());
                black_box(lhs3_cases[index].map_scalar(
                    &scalar_cases[index],
                    &ctx,
                    gmp_engine::Ctx::div,
                ))
            })
        });
    }
    for name in [
        "div_matrix_checked",
        "div_matrix_checked_abort",
        "div_matrix",
    ] {
        group.bench_function(format!("{label}/mat3 {name}"), |b| {
            let cursor = Cell::new(0);
            b.iter(|| {
                let index = cursor.get();
                cursor.set((index + 1) % lhs3_cases.len());
                black_box(lhs3_cases[index].div_matrix(&rhs3_cases[index], &ctx))
            })
        });
    }
    for name in [
        "add",
        "add_scalar",
        "sub",
        "sub_scalar",
        "neg",
        "mul_scalar",
    ] {
        group.bench_function(format!("{label}/mat3 {name}"), |b| {
            let cursor = Cell::new(0);
            b.iter(|| {
                let index = cursor.get();
                cursor.set((index + 1) % lhs3_cases.len());
                black_box(match name {
                    "add" => {
                        lhs3_cases[index].combine(&rhs3_cases[index], &ctx, gmp_engine::Ctx::add)
                    }
                    "add_scalar" => lhs3_cases[index].map_scalar(
                        &scalar_cases[index],
                        &ctx,
                        gmp_engine::Ctx::add,
                    ),
                    "sub" => {
                        lhs3_cases[index].combine(&rhs3_cases[index], &ctx, gmp_engine::Ctx::sub)
                    }
                    "sub_scalar" => lhs3_cases[index].map_scalar(
                        &scalar_cases[index],
                        &ctx,
                        gmp_engine::Ctx::sub,
                    ),
                    "neg" => lhs3_cases[index].neg(&ctx),
                    _ => lhs3_cases[index].map_scalar(
                        &scalar_cases[index],
                        &ctx,
                        gmp_engine::Ctx::mul,
                    ),
                })
            })
        });
    }

    group.bench_function(format!("{label}/mat4 zero"), |b| {
        b.iter(|| black_box(gmp_engine::Mat4::zero(&ctx)))
    });
    group.bench_function(format!("{label}/mat4 identity"), |b| {
        b.iter(|| black_box(gmp_engine::Mat4::identity(&ctx)))
    });
    group.bench_function(format!("{label}/mat4 transpose"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| black_box(next_case(&lhs4_cases, &cursor).transpose()))
    });
    for name in ["reciprocal", "reciprocal_checked"] {
        group.bench_function(format!("{label}/mat4 {name}"), |b| {
            let cursor = Cell::new(0);
            b.iter(|| black_box(next_case(&lhs4_cases, &cursor).inverse(&ctx)))
        });
    }
    for name in ["powi", "powi_checked", "bitxor"] {
        group.bench_function(format!("{label}/mat4 {name}"), |b| {
            let cursor = Cell::new(0);
            b.iter(|| black_box(next_case(&lhs4_cases, &cursor).powi(3, &ctx)))
        });
    }
    for name in [
        "div_scalar",
        "add",
        "add_scalar",
        "sub",
        "sub_scalar",
        "neg",
        "mul_scalar",
        "div_matrix",
    ] {
        group.bench_function(format!("{label}/mat4 {name}"), |b| {
            let cursor = Cell::new(0);
            b.iter(|| {
                let index = cursor.get();
                cursor.set((index + 1) % lhs4_cases.len());
                black_box(match name {
                    "div_scalar" => lhs4_cases[index].map_scalar(
                        &scalar_cases[index],
                        &ctx,
                        gmp_engine::Ctx::div,
                    ),
                    "add" => {
                        lhs4_cases[index].combine(&rhs4_cases[index], &ctx, gmp_engine::Ctx::add)
                    }
                    "add_scalar" => lhs4_cases[index].map_scalar(
                        &scalar_cases[index],
                        &ctx,
                        gmp_engine::Ctx::add,
                    ),
                    "sub" => {
                        lhs4_cases[index].combine(&rhs4_cases[index], &ctx, gmp_engine::Ctx::sub)
                    }
                    "sub_scalar" => lhs4_cases[index].map_scalar(
                        &scalar_cases[index],
                        &ctx,
                        gmp_engine::Ctx::sub,
                    ),
                    "neg" => lhs4_cases[index].neg(&ctx),
                    "mul_scalar" => lhs4_cases[index].map_scalar(
                        &scalar_cases[index],
                        &ctx,
                        gmp_engine::Ctx::mul,
                    ),
                    _ => lhs4_cases[index].div_matrix(&rhs4_cases[index], &ctx),
                })
            })
        });
    }
}

/// Registers forty Atom matrix rows, constructing expressions without requesting numerical output.
///
/// The local context stores 128 but never uses it to set precision. Fixtures import
/// precomputed f64 values; they are not recovered exact decimal or symbolic-constant
/// sources. Atom simplification may occur during construction, but these rows do not
/// request evaluation to the accuracy of either floating adapter or the native model.
///
/// Registers twenty-four Mat3 rows and sixteen Mat4 rows. Mat3 has construction,
/// zero, identity, transpose, four inverse labels, four cube labels, three scalar-
/// division labels, three right-division labels and six elementwise/scalar operations.
/// Mat4 has zero/identity/transpose, two inverse labels, three cube labels and eight
/// other operations; it has no construction row here. Every power/bitxor label calls
/// powi(3), and checked/abort variants call the same ordinary adapter methods without
/// native guards, typed Results or cancellation signals. Matrix division is lhs times
/// inverse(rhs); scalar division maps over every entry.
///
/// Retains four paired matrix cases and four scalar values outside timing. Scalars
/// are imported from 2, 1e-9, -1e9 and floating PI. Each cycling row initializes its
/// cursor outside iter, then measures selection/index advancement and borrowed adapter
/// calls, including internal cloning and result destruction. The Mat3 new row instead
/// imports a rotating raw case inside iter; zero/identity construct inside iter. There
/// is no explicit output float conversion, numerical oracle, dispatch hook/count check,
/// certainty receipt or total-work/memory limit. Adapter domain failures are not given
/// an independent wrapper here; Mat4 inverse retains its no-pivoting restriction.
fn bench_symbolica_matrix_operations(
    group: &mut BenchmarkGroup<'_, criterion::measurement::WallTime>,
    label: &str,
) {
    let ctx = symbolica_engine::Ctx::new(128);
    let lhs3_cases = sample_mat3_cases().map(|value| symbolica_engine::Mat3::new(&ctx, value.m));
    let rhs3_cases = sample_mat3_b_cases().map(|value| symbolica_engine::Mat3::new(&ctx, value.m));
    let lhs4_cases = sample_mat4_cases().map(|value| symbolica_engine::Mat4::new(&ctx, value.m));
    let rhs4_cases = sample_mat4_b_cases().map(|value| symbolica_engine::Mat4::new(&ctx, value.m));
    let scalar_cases = [2.0, 1.0e-9, -1.0e9, std::f64::consts::PI].map(|value| ctx.f(value));

    group.bench_function(format!("{label}/mat3 new"), |b| {
        let raw_cases = sample_mat3_cases();
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(symbolica_engine::Mat3::new(
                &ctx,
                next_case(&raw_cases, &cursor).m,
            ))
        })
    });
    group.bench_function(format!("{label}/mat3 zero"), |b| {
        b.iter(|| black_box(symbolica_engine::Mat3::zero(&ctx)))
    });
    group.bench_function(format!("{label}/mat3 identity"), |b| {
        b.iter(|| black_box(symbolica_engine::Mat3::identity(&ctx)))
    });
    group.bench_function(format!("{label}/mat3 transpose"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| black_box(next_case(&lhs3_cases, &cursor).transpose()))
    });
    for name in [
        "reciprocal",
        "reciprocal_checked",
        "inverse_checked",
        "inverse_checked_abort",
    ] {
        group.bench_function(format!("{label}/mat3 {name}"), |b| {
            let cursor = Cell::new(0);
            b.iter(|| black_box(next_case(&lhs3_cases, &cursor).inverse(&ctx)))
        });
    }
    for name in ["powi", "powi_checked", "powi_checked_abort", "bitxor"] {
        group.bench_function(format!("{label}/mat3 {name}"), |b| {
            let cursor = Cell::new(0);
            b.iter(|| black_box(next_case(&lhs3_cases, &cursor).powi(3, &ctx)))
        });
    }
    for name in [
        "div_scalar_checked",
        "div_scalar_checked_abort",
        "div_scalar",
    ] {
        group.bench_function(format!("{label}/mat3 {name}"), |b| {
            let cursor = Cell::new(0);
            b.iter(|| {
                let index = cursor.get();
                cursor.set((index + 1) % lhs3_cases.len());
                black_box(lhs3_cases[index].map_scalar(
                    &scalar_cases[index],
                    &ctx,
                    symbolica_engine::Ctx::div,
                ))
            })
        });
    }
    for name in [
        "div_matrix_checked",
        "div_matrix_checked_abort",
        "div_matrix",
    ] {
        group.bench_function(format!("{label}/mat3 {name}"), |b| {
            let cursor = Cell::new(0);
            b.iter(|| {
                let index = cursor.get();
                cursor.set((index + 1) % lhs3_cases.len());
                black_box(lhs3_cases[index].div_matrix(&rhs3_cases[index], &ctx))
            })
        });
    }
    for name in [
        "add",
        "add_scalar",
        "sub",
        "sub_scalar",
        "neg",
        "mul_scalar",
    ] {
        group.bench_function(format!("{label}/mat3 {name}"), |b| {
            let cursor = Cell::new(0);
            b.iter(|| {
                let index = cursor.get();
                cursor.set((index + 1) % lhs3_cases.len());
                black_box(match name {
                    "add" => lhs3_cases[index].combine(
                        &rhs3_cases[index],
                        &ctx,
                        symbolica_engine::Ctx::add,
                    ),
                    "add_scalar" => lhs3_cases[index].map_scalar(
                        &scalar_cases[index],
                        &ctx,
                        symbolica_engine::Ctx::add,
                    ),
                    "sub" => lhs3_cases[index].combine(
                        &rhs3_cases[index],
                        &ctx,
                        symbolica_engine::Ctx::sub,
                    ),
                    "sub_scalar" => lhs3_cases[index].map_scalar(
                        &scalar_cases[index],
                        &ctx,
                        symbolica_engine::Ctx::sub,
                    ),
                    "neg" => lhs3_cases[index].neg(&ctx),
                    _ => lhs3_cases[index].map_scalar(
                        &scalar_cases[index],
                        &ctx,
                        symbolica_engine::Ctx::mul,
                    ),
                })
            })
        });
    }

    group.bench_function(format!("{label}/mat4 zero"), |b| {
        b.iter(|| black_box(symbolica_engine::Mat4::zero(&ctx)))
    });
    group.bench_function(format!("{label}/mat4 identity"), |b| {
        b.iter(|| black_box(symbolica_engine::Mat4::identity(&ctx)))
    });
    group.bench_function(format!("{label}/mat4 transpose"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| black_box(next_case(&lhs4_cases, &cursor).transpose()))
    });
    for name in ["reciprocal", "reciprocal_checked"] {
        group.bench_function(format!("{label}/mat4 {name}"), |b| {
            let cursor = Cell::new(0);
            b.iter(|| black_box(next_case(&lhs4_cases, &cursor).inverse(&ctx)))
        });
    }
    for name in ["powi", "powi_checked", "bitxor"] {
        group.bench_function(format!("{label}/mat4 {name}"), |b| {
            let cursor = Cell::new(0);
            b.iter(|| black_box(next_case(&lhs4_cases, &cursor).powi(3, &ctx)))
        });
    }
    for name in [
        "div_scalar",
        "add",
        "add_scalar",
        "sub",
        "sub_scalar",
        "neg",
        "mul_scalar",
        "div_matrix",
    ] {
        group.bench_function(format!("{label}/mat4 {name}"), |b| {
            let cursor = Cell::new(0);
            b.iter(|| {
                let index = cursor.get();
                cursor.set((index + 1) % lhs4_cases.len());
                black_box(match name {
                    "div_scalar" => lhs4_cases[index].map_scalar(
                        &scalar_cases[index],
                        &ctx,
                        symbolica_engine::Ctx::div,
                    ),
                    "add" => lhs4_cases[index].combine(
                        &rhs4_cases[index],
                        &ctx,
                        symbolica_engine::Ctx::add,
                    ),
                    "add_scalar" => lhs4_cases[index].map_scalar(
                        &scalar_cases[index],
                        &ctx,
                        symbolica_engine::Ctx::add,
                    ),
                    "sub" => lhs4_cases[index].combine(
                        &rhs4_cases[index],
                        &ctx,
                        symbolica_engine::Ctx::sub,
                    ),
                    "sub_scalar" => lhs4_cases[index].map_scalar(
                        &scalar_cases[index],
                        &ctx,
                        symbolica_engine::Ctx::sub,
                    ),
                    "neg" => lhs4_cases[index].neg(&ctx),
                    "mul_scalar" => lhs4_cases[index].map_scalar(
                        &scalar_cases[index],
                        &ctx,
                        symbolica_engine::Ctx::mul,
                    ),
                    _ => lhs4_cases[index].div_matrix(&rhs4_cases[index], &ctx),
                })
            })
        });
    }
}
