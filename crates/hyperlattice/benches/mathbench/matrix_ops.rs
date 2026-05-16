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
        trace_matrix_profile_row(
            "mat3",
            "known_upper_triangular_inverse_checked_abort",
            input,
            1,
            || {
                black_box(
                    black_box(upper_triangular_matrix3.clone())
                        .upper_triangular_inverse_checked_with_abort(&signal)
                        .unwrap(),
                );
            },
        );
        trace_matrix_profile_row("mat3", "known_lower_triangular_inverse_checked", input, 1, || {
            black_box(
                black_box(lower_triangular_matrix3.clone())
                    .lower_triangular_inverse_checked()
                    .unwrap(),
            );
        });
        trace_matrix_profile_row(
            "mat3",
            "known_lower_triangular_inverse_checked_abort",
            input,
            1,
            || {
                black_box(
                    black_box(lower_triangular_matrix3.clone())
                        .lower_triangular_inverse_checked_with_abort(&signal)
                        .unwrap(),
                );
            },
        );
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
        trace_matrix_profile_row("mat3", "prepared_div_matrix", input, lhs3_cases.len(), || {
            let mut prepared = black_box(rhs3_cases[0].prepare_right_divisor());
            for index in 0..lhs3_cases.len() {
                black_box(
                    black_box(lhs3_cases[index].clone())
                        .div_matrix_with_prepared(&mut prepared)
                        .unwrap(),
                );
            }
        });
        trace_matrix_profile_row(
            "mat3",
            "prepared_div_matrix_checked",
            input,
            lhs3_cases.len(),
            || {
                let mut prepared = black_box(rhs3_cases[0].prepare_right_divisor());
                for index in 0..lhs3_cases.len() {
                    black_box(
                        black_box(lhs3_cases[index].clone())
                            .div_matrix_checked_with_prepared(&mut prepared)
                            .unwrap(),
                    );
                }
            },
        );
        trace_matrix_profile_row(
            "mat3",
            "prepared_div_matrix_checked_abort",
            input,
            lhs3_cases.len(),
            || {
                let mut prepared = black_box(rhs3_cases[0].prepare_right_divisor());
                for index in 0..lhs3_cases.len() {
                    black_box(
                        black_box(lhs3_cases[index].clone())
                            .div_matrix_checked_with_prepared_with_abort(
                                &mut prepared,
                                &signal,
                            )
                            .unwrap(),
                    );
                }
            },
        );
        trace_matrix_profile_row("mat3", "prepared_inverse", input, lhs3_cases.len(), || {
            let mut prepared = black_box(rhs3_cases[0].prepare_right_divisor());
            for _ in 0..lhs3_cases.len() {
                black_box(prepared.inverse().unwrap());
            }
        });
        trace_matrix_profile_row("mat3", "prepared_reciprocal", input, lhs3_cases.len(), || {
            let mut prepared = black_box(rhs3_cases[0].prepare_right_divisor());
            for _ in 0..lhs3_cases.len() {
                black_box(prepared.reciprocal().unwrap());
            }
        });
        trace_matrix_profile_row(
            "mat3",
            "prepared_inverse_checked",
            input,
            lhs3_cases.len(),
            || {
                let mut prepared = black_box(rhs3_cases[0].prepare_right_divisor());
                for _ in 0..lhs3_cases.len() {
                    black_box(prepared.inverse_checked().unwrap());
                }
            },
        );
        trace_matrix_profile_row(
            "mat3",
            "prepared_reciprocal_checked",
            input,
            lhs3_cases.len(),
            || {
                let mut prepared = black_box(rhs3_cases[0].prepare_right_divisor());
                for _ in 0..lhs3_cases.len() {
                    black_box(prepared.reciprocal_checked().unwrap());
                }
            },
        );
        trace_matrix_profile_row(
            "mat3",
            "prepared_inverse_checked_abort",
            input,
            lhs3_cases.len(),
            || {
                let mut prepared = black_box(rhs3_cases[0].prepare_right_divisor());
                for _ in 0..lhs3_cases.len() {
                    black_box(prepared.inverse_checked_with_abort(&signal).unwrap());
                }
            },
        );
        trace_matrix_profile_row(
            "mat3",
            "prepared_reciprocal_checked_abort",
            input,
            lhs3_cases.len(),
            || {
                let mut prepared = black_box(rhs3_cases[0].prepare_right_divisor());
                for _ in 0..lhs3_cases.len() {
                    black_box(prepared.reciprocal_checked_with_abort(&signal).unwrap());
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
        trace_matrix_profile_row("mat3", "transform_vec3_handle", input, lhs3_cases.len(), || {
            for index in 0..lhs3_cases.len() {
                let handle = black_box(&lhs3_cases[index]).transform_vec3_handle();
                black_box(handle.transform_vector(black_box(&vector3_cases[index])));
            }
        });
        trace_matrix_profile_row("mat3", "transform_vec3_batch", input, vector3_cases.len(), || {
            let handle = black_box(&lhs3_cases[0]).transform_vec3_handle();
            black_box(handle.transform_vector_batch(black_box(&vector3_cases)));
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
        trace_matrix_profile_row(
            "mat4",
            "known_upper_triangular_inverse_checked_abort",
            input,
            1,
            || {
                black_box(
                    black_box(upper_triangular_matrix4.clone())
                        .upper_triangular_inverse_checked_with_abort(&signal)
                        .unwrap(),
                );
            },
        );
        trace_matrix_profile_row("mat4", "known_lower_triangular_inverse_checked", input, 1, || {
            black_box(
                black_box(lower_triangular_matrix4.clone())
                    .lower_triangular_inverse_checked()
                    .unwrap(),
            );
        });
        trace_matrix_profile_row(
            "mat4",
            "known_lower_triangular_inverse_checked_abort",
            input,
            1,
            || {
                black_box(
                    black_box(lower_triangular_matrix4.clone())
                        .lower_triangular_inverse_checked_with_abort(&signal)
                        .unwrap(),
                );
            },
        );
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
        trace_matrix_profile_row("mat4", "prepared_div_matrix", input, lhs4_cases.len(), || {
            let mut prepared = black_box(rhs4_cases[0].prepare_right_divisor());
            for index in 0..lhs4_cases.len() {
                black_box(
                    black_box(lhs4_cases[index].clone())
                        .div_matrix_with_prepared(&mut prepared)
                        .unwrap(),
                );
            }
        });
        trace_matrix_profile_row(
            "mat4",
            "prepared_div_matrix_checked",
            input,
            lhs4_cases.len(),
            || {
                let mut prepared = black_box(rhs4_cases[0].prepare_right_divisor());
                for index in 0..lhs4_cases.len() {
                    black_box(
                        black_box(lhs4_cases[index].clone())
                            .div_matrix_checked_with_prepared(&mut prepared)
                            .unwrap(),
                    );
                }
            },
        );
        trace_matrix_profile_row(
            "mat4",
            "prepared_div_matrix_checked_abort",
            input,
            lhs4_cases.len(),
            || {
                let mut prepared = black_box(rhs4_cases[0].prepare_right_divisor());
                for index in 0..lhs4_cases.len() {
                    black_box(
                        black_box(lhs4_cases[index].clone())
                            .div_matrix_checked_with_prepared_with_abort(
                                &mut prepared,
                                &signal,
                            )
                            .unwrap(),
                    );
                }
            },
        );
        trace_matrix_profile_row("mat4", "prepared_inverse", input, lhs4_cases.len(), || {
            let mut prepared = black_box(rhs4_cases[0].prepare_right_divisor());
            for _ in 0..lhs4_cases.len() {
                black_box(prepared.inverse().unwrap());
            }
        });
        trace_matrix_profile_row("mat4", "prepared_reciprocal", input, lhs4_cases.len(), || {
            let mut prepared = black_box(rhs4_cases[0].prepare_right_divisor());
            for _ in 0..lhs4_cases.len() {
                black_box(prepared.reciprocal().unwrap());
            }
        });
        trace_matrix_profile_row(
            "mat4",
            "prepared_inverse_checked",
            input,
            lhs4_cases.len(),
            || {
                let mut prepared = black_box(rhs4_cases[0].prepare_right_divisor());
                for _ in 0..lhs4_cases.len() {
                    black_box(prepared.inverse_checked().unwrap());
                }
            },
        );
        trace_matrix_profile_row(
            "mat4",
            "prepared_reciprocal_checked",
            input,
            lhs4_cases.len(),
            || {
                let mut prepared = black_box(rhs4_cases[0].prepare_right_divisor());
                for _ in 0..lhs4_cases.len() {
                    black_box(prepared.reciprocal_checked().unwrap());
                }
            },
        );
        trace_matrix_profile_row(
            "mat4",
            "prepared_inverse_checked_abort",
            input,
            lhs4_cases.len(),
            || {
                let mut prepared = black_box(rhs4_cases[0].prepare_right_divisor());
                for _ in 0..lhs4_cases.len() {
                    black_box(prepared.inverse_checked_with_abort(&signal).unwrap());
                }
            },
        );
        trace_matrix_profile_row(
            "mat4",
            "prepared_reciprocal_checked_abort",
            input,
            lhs4_cases.len(),
            || {
                let mut prepared = black_box(rhs4_cases[0].prepare_right_divisor());
                for _ in 0..lhs4_cases.len() {
                    black_box(prepared.reciprocal_checked_with_abort(&signal).unwrap());
                }
            },
        );
        trace_matrix_profile_row("mat4", "prepared_powi_negative", input, lhs4_cases.len(), || {
            let mut prepared = black_box(rhs4_cases[0].prepare_right_divisor());
            for _ in 0..lhs4_cases.len() {
                black_box(prepared.powi(-2).unwrap());
            }
        });
        trace_matrix_profile_row(
            "mat4",
            "prepared_powi_negative_one",
            input,
            lhs4_cases.len(),
            || {
                let mut prepared = black_box(rhs4_cases[0].prepare_right_divisor());
                for _ in 0..lhs4_cases.len() {
                    black_box(prepared.powi(-1).unwrap());
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
        trace_matrix_profile_row("mat4", "identity_direction_transform_handle", input, 1, || {
            let handle = black_box(&identity_matrix4).transform_vec4_handle();
            black_box(handle.transform_direction_vector(black_box(&translated_diagonal_direction)));
        });
        trace_matrix_profile_row("mat4", "identity_point_transform_handle", input, 1, || {
            let handle = black_box(&identity_matrix4).transform_vec4_handle();
            black_box(handle.transform_point_vector(black_box(&translated_diagonal_point)));
        });
        trace_matrix_profile_row("mat4", "identity_direction_materialize", input, 1, || {
            black_box(
                black_box(&identity_matrix4)
                    .transform_vec4_with(black_box(&translated_diagonal_direction))
                    .materialize(),
            );
        });
        trace_matrix_profile_row("mat4", "identity_point_materialize", input, 1, || {
            black_box(
                black_box(&identity_matrix4)
                    .transform_vec4_with(black_box(&translated_diagonal_point))
                    .materialize(),
            );
        });
        trace_matrix_profile_row(
            "mat4",
            "translated_diagonal_direction_materialize",
            input,
            1,
            || {
                black_box(
                    black_box(&translated_diagonal_direction_matrix)
                        .transform_vec4_with(black_box(&translated_diagonal_direction))
                        .materialize(),
                );
            },
        );
        trace_matrix_profile_row(
            "mat4",
            "translated_diagonal_point_materialize",
            input,
            1,
            || {
                black_box(
                    black_box(&translated_diagonal_direction_matrix)
                        .transform_vec4_with(black_box(&translated_diagonal_point))
                        .materialize(),
                );
            },
        );
        trace_matrix_profile_row(
            "mat4",
            "translated_diagonal_direction_batch",
            input,
            translated_diagonal_direction_batch.len(),
            || {
                let handle = black_box(&translated_diagonal_direction_matrix).transform_vec4_handle();
                black_box(handle.transform_vector_batch(black_box(&translated_diagonal_direction_batch)));
            },
        );
        trace_matrix_profile_row(
            "mat4",
            "translated_diagonal_point_batch",
            input,
            translated_diagonal_point_batch.len(),
            || {
                let handle = black_box(&translated_diagonal_direction_matrix).transform_vec4_handle();
                black_box(handle.transform_vector_batch(black_box(&translated_diagonal_point_batch)));
            },
        );
        trace_matrix_profile_row(
            "mat4",
            "translated_diagonal_direction_batch_assumed",
            input,
            translated_diagonal_direction_batch.len(),
            || {
                let handle = black_box(&translated_diagonal_direction_matrix).transform_vec4_handle();
                black_box(handle.transform_direction_batch(black_box(&translated_diagonal_direction_batch)));
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
                let handle = black_box(&translated_diagonal_direction_matrix).transform_vec4_handle();
                black_box(handle.transform_point_batch(black_box(&translated_diagonal_point_batch)));
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
                let handle = black_box(&diagonal_affine_matrix).transform_vec4_handle();
                black_box(handle.transform_vector_batch(black_box(&translated_diagonal_direction_batch)));
            },
        );
        trace_matrix_profile_row(
            "mat4",
            "diagonal_point_batch",
            input,
            translated_diagonal_point_batch.len(),
            || {
                let handle = black_box(&diagonal_affine_matrix).transform_vec4_handle();
                black_box(handle.transform_vector_batch(black_box(&translated_diagonal_point_batch)));
            },
        );
        trace_matrix_profile_row(
            "mat4",
            "identity_direction_batch_assumed",
            input,
            translated_diagonal_direction_batch.len(),
            || {
                let handle = black_box(&identity_matrix4).transform_vec4_handle();
                black_box(handle.transform_direction_batch(black_box(&translated_diagonal_direction_batch)));
            },
        );
        trace_matrix_profile_row(
            "mat4",
            "identity_point_batch_assumed",
            input,
            translated_diagonal_point_batch.len(),
            || {
                let handle = black_box(&identity_matrix4).transform_vec4_handle();
                black_box(handle.transform_point_batch(black_box(&translated_diagonal_point_batch)));
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
    trace_dispatch_row(
        format!("matrix_ops/{label}/mat3 known_upper_triangular_inverse_checked_abort"),
        || {
                black_box(
                    black_box(upper_triangular_matrix3.clone())
                    .upper_triangular_inverse_checked_with_abort(&signal)
                    .unwrap(),
                );
        },
    );
    trace_dispatch_row(format!("matrix_ops/{label}/mat3 known_lower_triangular_inverse_checked"), || {
        black_box(black_box(lower_triangular_matrix3.clone()).lower_triangular_inverse_checked().unwrap());
    });
    trace_dispatch_row(
        format!("matrix_ops/{label}/mat3 known_lower_triangular_inverse_checked_abort"),
        || {
                black_box(
                    black_box(lower_triangular_matrix3.clone())
                    .lower_triangular_inverse_checked_with_abort(&signal)
                    .unwrap(),
                );
        },
    );
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
    trace_dispatch_row(format!("matrix_ops/{label}/mat3 transform_vec3_handle"), || {
        for index in 0..lhs3_cases.len() {
            let handle = black_box(&lhs3_cases[index]).transform_vec3_handle();
            black_box(handle.transform_vector(black_box(&vector3_cases[index])));
        }
    });
    trace_dispatch_row(format!("matrix_ops/{label}/mat3 transform_vec3_batch"), || {
        let handle = black_box(&lhs3_cases[0]).transform_vec3_handle();
        black_box(handle.transform_vector_batch(black_box(&vector3_cases)));
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
    trace_dispatch_row(format!("matrix_ops/{label}/mat3 prepared_div_matrix"), || {
        let mut prepared = black_box(rhs3_cases[0].prepare_right_divisor());
        for index in 0..lhs3_cases.len() {
            black_box(
                black_box(lhs3_cases[index].clone())
                    .div_matrix_with_prepared(&mut prepared)
                    .unwrap(),
            );
        }
    });
    trace_dispatch_row(format!("matrix_ops/{label}/mat3 prepared_inverse"), || {
        let mut prepared = black_box(rhs3_cases[0].prepare_right_divisor());
        for _ in 0..lhs3_cases.len() {
            black_box(prepared.inverse().unwrap());
        }
    });
    trace_dispatch_row(format!("matrix_ops/{label}/mat3 prepared_reciprocal"), || {
        let mut prepared = black_box(rhs3_cases[0].prepare_right_divisor());
        for _ in 0..lhs3_cases.len() {
            black_box(prepared.reciprocal().unwrap());
        }
    });
    trace_dispatch_row(format!("matrix_ops/{label}/mat3 prepared_inverse_checked"), || {
        let mut prepared = black_box(rhs3_cases[0].prepare_right_divisor());
        for _ in 0..lhs3_cases.len() {
            black_box(prepared.inverse_checked().unwrap());
        }
    });
    trace_dispatch_row(
        format!("matrix_ops/{label}/mat3 prepared_reciprocal_checked"),
        || {
            let mut prepared = black_box(rhs3_cases[0].prepare_right_divisor());
            for _ in 0..lhs3_cases.len() {
                black_box(prepared.reciprocal_checked().unwrap());
            }
        },
    );
    trace_dispatch_row(
        format!("matrix_ops/{label}/mat3 prepared_inverse_checked_abort"),
        || {
            let mut prepared = black_box(rhs3_cases[0].prepare_right_divisor());
            for _ in 0..lhs3_cases.len() {
                black_box(prepared.inverse_checked_with_abort(&signal).unwrap());
            }
        },
    );
    trace_dispatch_row(
        format!("matrix_ops/{label}/mat3 prepared_reciprocal_checked_abort"),
        || {
            let mut prepared = black_box(rhs3_cases[0].prepare_right_divisor());
            for _ in 0..lhs3_cases.len() {
                black_box(prepared.reciprocal_checked_with_abort(&signal).unwrap());
            }
        },
    );
    trace_dispatch_row(format!("matrix_ops/{label}/mat3 prepared_div_matrix_checked"), || {
        let mut prepared = black_box(rhs3_cases[0].prepare_right_divisor());
        for index in 0..lhs3_cases.len() {
            black_box(
                black_box(lhs3_cases[index].clone())
                    .div_matrix_checked_with_prepared(&mut prepared)
                    .unwrap(),
            );
        }
    });
    trace_dispatch_row(
        format!("matrix_ops/{label}/mat3 prepared_div_matrix_checked_abort"),
        || {
            let mut prepared = black_box(rhs3_cases[0].prepare_right_divisor());
            for index in 0..lhs3_cases.len() {
                black_box(
                    black_box(lhs3_cases[index].clone())
                        .div_matrix_checked_with_prepared_with_abort(
                            &mut prepared,
                            &signal,
                        )
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
    trace_dispatch_row(
        format!("matrix_ops/{label}/mat4 known_upper_triangular_inverse_checked_abort"),
        || {
                black_box(
                    black_box(upper_triangular_matrix4.clone())
                    .upper_triangular_inverse_checked_with_abort(&signal)
                    .unwrap(),
                );
        },
    );
    trace_dispatch_row(format!("matrix_ops/{label}/mat4 known_lower_triangular_inverse_checked"), || {
        black_box(black_box(lower_triangular_matrix4.clone()).lower_triangular_inverse_checked().unwrap());
    });
    trace_dispatch_row(
        format!("matrix_ops/{label}/mat4 known_lower_triangular_inverse_checked_abort"),
        || {
                black_box(
                    black_box(lower_triangular_matrix4.clone())
                    .lower_triangular_inverse_checked_with_abort(&signal)
                    .unwrap(),
                );
        },
    );
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
    trace_dispatch_row(format!("matrix_ops/{label}/mat4 prepared_div_matrix"), || {
        let mut prepared = black_box(rhs4_cases[0].prepare_right_divisor());
        for index in 0..lhs4_cases.len() {
            black_box(
                black_box(lhs4_cases[index].clone())
                    .div_matrix_with_prepared(&mut prepared)
                    .unwrap(),
            );
        }
    });
    trace_dispatch_row(
        format!("matrix_ops/{label}/mat4 prepared_div_matrix_exact_left"),
        || {
            let mut prepared = black_box(rhs4_cases[0].prepare_right_divisor());
            for index in 0..lhs4_cases.len() {
                black_box(
                    black_box(lhs4_cases[index].clone())
                        .div_exact_rational_matrix_with_prepared(&mut prepared)
                        .unwrap(),
                );
            }
        },
    );
    trace_dispatch_row(format!("matrix_ops/{label}/mat4 prepared_div_matrix_checked"), || {
        let mut prepared = black_box(rhs4_cases[0].prepare_right_divisor());
        for index in 0..lhs4_cases.len() {
            black_box(
                black_box(lhs4_cases[index].clone())
                    .div_matrix_checked_with_prepared(&mut prepared)
                    .unwrap(),
            );
        }
    });
    trace_dispatch_row(
        format!("matrix_ops/{label}/mat4 prepared_div_matrix_checked_abort"),
        || {
            let mut prepared = black_box(rhs4_cases[0].prepare_right_divisor());
            for index in 0..lhs4_cases.len() {
                black_box(
                    black_box(lhs4_cases[index].clone())
                        .div_matrix_checked_with_prepared_with_abort(
                            &mut prepared,
                            &signal,
                        )
                        .unwrap(),
                );
            }
        },
    );
    trace_dispatch_row(format!("matrix_ops/{label}/mat4 prepared_inverse"), || {
        let mut prepared = black_box(rhs4_cases[0].prepare_right_divisor());
        for _ in 0..lhs4_cases.len() {
            black_box(prepared.inverse().unwrap());
        }
    });
    trace_dispatch_row(
        format!("matrix_ops/{label}/mat4 prepared_reciprocal"),
        || {
            let mut prepared = black_box(rhs4_cases[0].prepare_right_divisor());
            for _ in 0..lhs4_cases.len() {
                black_box(prepared.reciprocal().unwrap());
            }
        },
    );
    trace_dispatch_row(format!("matrix_ops/{label}/mat4 prepared_inverse_checked"), || {
        let mut prepared = black_box(rhs4_cases[0].prepare_right_divisor());
        for _ in 0..lhs4_cases.len() {
            black_box(prepared.inverse_checked().unwrap());
        }
    });
    trace_dispatch_row(
        format!("matrix_ops/{label}/mat4 prepared_reciprocal_checked"),
        || {
            let mut prepared = black_box(rhs4_cases[0].prepare_right_divisor());
            for _ in 0..lhs4_cases.len() {
                black_box(prepared.reciprocal_checked().unwrap());
            }
        },
    );
    trace_dispatch_row(
        format!("matrix_ops/{label}/mat4 prepared_inverse_checked_abort"),
        || {
            let mut prepared = black_box(rhs4_cases[0].prepare_right_divisor());
            for _ in 0..lhs4_cases.len() {
                black_box(prepared.inverse_checked_with_abort(&signal).unwrap());
            }
        },
    );
    trace_dispatch_row(
        format!("matrix_ops/{label}/mat4 prepared_reciprocal_checked_abort"),
        || {
            let mut prepared = black_box(rhs4_cases[0].prepare_right_divisor());
            for _ in 0..lhs4_cases.len() {
                black_box(prepared.reciprocal_checked_with_abort(&signal).unwrap());
            }
        },
    );
    trace_dispatch_row(format!("matrix_ops/{label}/mat4 prepared_powi_negative"), || {
        let mut prepared = black_box(rhs4_cases[0].prepare_right_divisor());
        for _ in 0..lhs4_cases.len() {
            black_box(prepared.powi(-2).unwrap());
        }
    });
    trace_dispatch_row(
        format!("matrix_ops/{label}/mat4 prepared_powi_negative_one"),
        || {
            let mut prepared = black_box(rhs4_cases[0].prepare_right_divisor());
            for _ in 0..lhs4_cases.len() {
                black_box(prepared.powi(-1).unwrap());
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
        format!("matrix_ops/{label}/mat4 identity_direction_transform_handle"),
        || {
            let handle = black_box(&identity_matrix4).transform_vec4_handle();
            black_box(handle.transform_direction_vector(black_box(&translated_diagonal_direction)));
        },
    );
    trace_dispatch_row(
        format!("matrix_ops/{label}/mat4 identity_point_transform_handle"),
        || {
            let handle = black_box(&identity_matrix4).transform_vec4_handle();
            black_box(handle.transform_point_vector(black_box(&translated_diagonal_point)));
        },
    );
    trace_dispatch_row(
        format!("matrix_ops/{label}/mat4 identity_direction_materialize"),
        || {
            black_box(
                black_box(&identity_matrix4)
                    .transform_vec4_with(black_box(&translated_diagonal_direction))
                    .materialize(),
            );
        },
    );
    trace_dispatch_row(
        format!("matrix_ops/{label}/mat4 identity_point_materialize"),
        || {
            black_box(
                black_box(&identity_matrix4)
                    .transform_vec4_with(black_box(&translated_diagonal_point))
                .materialize(),
            );
        },
    );
    trace_dispatch_row(
        format!("matrix_ops/{label}/mat4 translated_diagonal_direction_materialize"),
        || {
            black_box(
                black_box(&translated_diagonal_direction_matrix)
                    .transform_vec4_with(black_box(&translated_diagonal_direction))
                    .materialize(),
            );
        },
    );
    trace_dispatch_row(
        format!("matrix_ops/{label}/mat4 translated_diagonal_point_materialize"),
        || {
            black_box(
                black_box(&translated_diagonal_direction_matrix)
                    .transform_vec4_with(black_box(&translated_diagonal_point))
                    .materialize(),
            );
        },
    );
    trace_dispatch_row(
        format!("matrix_ops/{label}/mat4 translated_diagonal_direction_batch"),
        || {
            let handle = black_box(&translated_diagonal_direction_matrix).transform_vec4_handle();
            black_box(handle.transform_vector_batch(black_box(&translated_diagonal_direction_batch)));
        },
    );
    trace_dispatch_row(
        format!("matrix_ops/{label}/mat4 translated_diagonal_point_batch"),
        || {
            let handle = black_box(&translated_diagonal_direction_matrix).transform_vec4_handle();
            black_box(handle.transform_vector_batch(black_box(&translated_diagonal_point_batch)));
        },
    );
    trace_dispatch_row(
        format!("matrix_ops/{label}/mat4 translated_diagonal_direction_batch_assumed"),
        || {
            let handle = black_box(&translated_diagonal_direction_matrix).transform_vec4_handle();
            black_box(handle.transform_direction_batch(black_box(&translated_diagonal_direction_batch)));
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
            let handle = black_box(&translated_diagonal_direction_matrix).transform_vec4_handle();
            black_box(handle.transform_point_batch(black_box(&translated_diagonal_point_batch)));
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
            let handle = black_box(&diagonal_affine_matrix).transform_vec4_handle();
            black_box(handle.transform_vector_batch(black_box(&translated_diagonal_direction_batch)));
        },
    );
    trace_dispatch_row(
        format!("matrix_ops/{label}/mat4 diagonal_point_batch"),
        || {
            let handle = black_box(&diagonal_affine_matrix).transform_vec4_handle();
            black_box(handle.transform_vector_batch(black_box(&translated_diagonal_point_batch)));
        },
    );
    trace_dispatch_row(
        format!("matrix_ops/{label}/mat4 identity_direction_batch_assumed"),
        || {
            let handle = black_box(&identity_matrix4).transform_vec4_handle();
            black_box(handle.transform_direction_batch(black_box(&translated_diagonal_direction_batch)));
        },
    );
    trace_dispatch_row(
        format!("matrix_ops/{label}/mat4 identity_point_batch_assumed"),
        || {
            let handle = black_box(&identity_matrix4).transform_vec4_handle();
            black_box(handle.transform_point_batch(black_box(&translated_diagonal_point_batch)));
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
    // Bench checked/abort triangular inverse variants to keep error-aware structural
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
    group.bench_function(
        format!("{label}/mat3 known_upper_triangular_inverse_checked_abort"),
        |b| {
            b.iter(|| {
                black_box(
                    black_box(upper_triangular_matrix3.clone())
                        .upper_triangular_inverse_checked_with_abort(&signal)
                        .unwrap(),
                )
            })
        },
    );
    group.bench_function(format!("{label}/mat3 known_lower_triangular_inverse_checked"), |b| {
        b.iter(|| {
            black_box(
                black_box(lower_triangular_matrix3.clone())
                    .lower_triangular_inverse_checked()
                    .unwrap(),
            )
        })
    });
    group.bench_function(
        format!("{label}/mat3 known_lower_triangular_inverse_checked_abort"),
        |b| {
            b.iter(|| {
                black_box(
                    black_box(lower_triangular_matrix3.clone())
                        .lower_triangular_inverse_checked_with_abort(&signal)
                        .unwrap(),
                )
            })
        },
    );
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
    group.bench_function(format!("{label}/mat3 prepared_div_matrix"), |b| {
        let cursor = Cell::new(0);
        let mut prepared = rhs3_cases[0].prepare_right_divisor();
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % lhs3_cases.len());
            black_box(
                black_box(lhs3_cases[index].clone())
                    .div_matrix_with_prepared(&mut prepared)
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/mat3 prepared_div_matrix_checked"), |b| {
        let cursor = Cell::new(0);
        let mut prepared = rhs3_cases[0].prepare_right_divisor();
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % lhs3_cases.len());
            black_box(
                black_box(lhs3_cases[index].clone())
                    .div_matrix_checked_with_prepared(&mut prepared)
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/mat3 prepared_div_matrix_checked_abort"), |b| {
        let cursor = Cell::new(0);
        let mut prepared = rhs3_cases[0].prepare_right_divisor();
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % lhs3_cases.len());
            black_box(
                black_box(lhs3_cases[index].clone())
                    .div_matrix_checked_with_prepared_with_abort(
                        &mut prepared,
                        &signal,
                    )
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
    // Bench checked/abort triangular inverse variants to keep error-aware structural
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
    group.bench_function(
        format!("{label}/mat4 known_upper_triangular_inverse_checked_abort"),
        |b| {
            b.iter(|| {
                black_box(
                    black_box(upper_triangular_matrix4.clone())
                        .upper_triangular_inverse_checked_with_abort(&signal)
                        .unwrap(),
                )
            })
        },
    );
    group.bench_function(format!("{label}/mat4 known_lower_triangular_inverse_checked"), |b| {
        b.iter(|| {
            black_box(
                black_box(lower_triangular_matrix4.clone())
                    .lower_triangular_inverse_checked()
                    .unwrap(),
            )
        })
    });
    group.bench_function(
        format!("{label}/mat4 known_lower_triangular_inverse_checked_abort"),
        |b| {
            b.iter(|| {
                black_box(
                    black_box(lower_triangular_matrix4.clone())
                        .lower_triangular_inverse_checked_with_abort(&signal)
                        .unwrap(),
                )
            })
        },
    );
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
    group.bench_function(format!("{label}/mat4 prepared_div_matrix"), |b| {
        let cursor = Cell::new(0);
        let mut prepared = rhs4_cases[0].prepare_right_divisor();
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % lhs4_cases.len());
            black_box(
                black_box(lhs4_cases[index].clone())
                    .div_matrix_with_prepared(&mut prepared)
                    .unwrap(),
            )
        })
    });
    group.bench_function(
        format!("{label}/mat4 prepared_div_matrix_exact_left"),
        |b| {
            let cursor = Cell::new(0);
            let mut prepared = rhs4_cases[0].prepare_right_divisor();
            b.iter(|| {
                let index = cursor.get();
                cursor.set((index + 1) % lhs4_cases.len());
                black_box(
                    black_box(lhs4_cases[index].clone())
                        .div_exact_rational_matrix_with_prepared(&mut prepared)
                        .unwrap(),
                )
            })
        },
    );
    group.bench_function(format!("{label}/mat4 prepared_div_matrix_checked"), |b| {
        let cursor = Cell::new(0);
        let mut prepared = rhs4_cases[0].prepare_right_divisor();
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % lhs4_cases.len());
            black_box(
                black_box(lhs4_cases[index].clone())
                    .div_matrix_checked_with_prepared(&mut prepared)
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/mat4 prepared_div_matrix_checked_abort"), |b| {
        let cursor = Cell::new(0);
        let mut prepared = rhs4_cases[0].prepare_right_divisor();
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % lhs4_cases.len());
            black_box(
                black_box(lhs4_cases[index].clone())
                    .div_matrix_checked_with_prepared_with_abort(
                        &mut prepared,
                        &signal,
                    )
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/mat4 prepared_inverse"), |b| {
        let mut prepared = rhs4_cases[0].prepare_right_divisor();
        b.iter(|| black_box(prepared.inverse().unwrap()))
    });
    group.bench_function(format!("{label}/mat4 prepared_reciprocal"), |b| {
        let mut prepared = rhs4_cases[0].prepare_right_divisor();
        b.iter(|| black_box(prepared.reciprocal().unwrap()))
    });
    group.bench_function(format!("{label}/mat4 prepared_inverse_checked"), |b| {
        let mut prepared = rhs4_cases[0].prepare_right_divisor();
        b.iter(|| black_box(prepared.inverse_checked().unwrap()))
    });
    group.bench_function(format!("{label}/mat4 prepared_reciprocal_checked"), |b| {
        let mut prepared = rhs4_cases[0].prepare_right_divisor();
        b.iter(|| black_box(prepared.reciprocal_checked().unwrap()))
    });
    group.bench_function(format!("{label}/mat4 prepared_inverse_checked_abort"), |b| {
        let mut prepared = rhs4_cases[0].prepare_right_divisor();
        b.iter(|| black_box(prepared.inverse_checked_with_abort(&signal).unwrap()))
    });
    group.bench_function(
        format!("{label}/mat4 prepared_reciprocal_checked_abort"),
        |b| {
            let mut prepared = rhs4_cases[0].prepare_right_divisor();
            b.iter(|| black_box(prepared.reciprocal_checked_with_abort(&signal).unwrap()))
        },
    );
    group.bench_function(format!("{label}/mat4 prepared_powi_negative"), |b| {
        let mut prepared = rhs4_cases[0].prepare_right_divisor();
        b.iter(|| black_box(prepared.powi(-2).unwrap()))
    });
    group.bench_function(format!("{label}/mat4 prepared_powi_negative_one"), |b| {
        let mut prepared = rhs4_cases[0].prepare_right_divisor();
        b.iter(|| black_box(prepared.powi(-1).unwrap()))
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
        let handle = translated_diagonal_direction_matrix.transform_vec4_handle();
        b.iter(|| black_box(handle.transform_vector_batch(black_box(&translated_diagonal_direction_batch))))
    });
    group.bench_function(format!("{label}/mat4 translated_diagonal_direction_batch_assumed"), |b| {
        let handle = translated_diagonal_direction_matrix.transform_vec4_handle();
        b.iter(|| black_box(handle.transform_direction_batch(black_box(&translated_diagonal_direction_batch))))
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
    group.bench_function(format!("{label}/mat4 identity_direction_transform_handle"), |b| {
        let handle = identity_matrix4.transform_vec4_handle();
        b.iter(|| {
            black_box(handle.transform_direction_vector(black_box(
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
    group.bench_function(format!("{label}/mat4 identity_point_transform_handle"), |b| {
        let handle = identity_matrix4.transform_vec4_handle();
        b.iter(|| black_box(handle.transform_point_vector(black_box(&translated_diagonal_point))))
    });
    group.bench_function(format!("{label}/mat4 identity_direction_materialize"), |b| {
        b.iter(|| {
            black_box(
                identity_matrix4
                    .transform_vec4_with(black_box(&translated_diagonal_direction))
                    .materialize(),
            )
        })
    });
    group.bench_function(format!("{label}/mat4 identity_point_materialize"), |b| {
        b.iter(|| {
            black_box(
                identity_matrix4
                    .transform_vec4_with(black_box(&translated_diagonal_point))
                    .materialize(),
            )
        })
    });
    group.bench_function(
        format!("{label}/mat4 translated_diagonal_direction_materialize"),
        |b| {
            b.iter(|| {
                black_box(
                    translated_diagonal_direction_matrix
                        .transform_vec4_with(black_box(&translated_diagonal_direction))
                        .materialize(),
                )
            })
        },
    );
    group.bench_function(
        format!("{label}/mat4 translated_diagonal_point_materialize"),
        |b| {
            b.iter(|| {
                black_box(
                    translated_diagonal_direction_matrix
                        .transform_vec4_with(black_box(&translated_diagonal_point))
                        .materialize(),
                )
            })
        },
    );
    group.bench_function(format!("{label}/mat4 translated_diagonal_point_batch"), |b| {
        let handle = translated_diagonal_direction_matrix.transform_vec4_handle();
        b.iter(|| black_box(handle.transform_vector_batch(black_box(&translated_diagonal_point_batch))))
    });
    group.bench_function(format!("{label}/mat4 translated_diagonal_point_batch_assumed"), |b| {
        let handle = translated_diagonal_direction_matrix.transform_vec4_handle();
        b.iter(|| black_box(handle.transform_point_batch(black_box(&translated_diagonal_point_batch))))
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
        let handle = identity_matrix4.transform_vec4_handle();
        b.iter(|| black_box(handle.transform_direction_batch(black_box(&translated_diagonal_direction_batch))))
    });
    group.bench_function(format!("{label}/mat4 identity_point_batch_assumed"), |b| {
        let handle = identity_matrix4.transform_vec4_handle();
        b.iter(|| black_box(handle.transform_point_batch(black_box(&translated_diagonal_point_batch))))
    });
    group.bench_function(format!("{label}/mat4 bitxor"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| black_box((black_box(next_case(&lhs4_cases, &cursor).clone()) ^ 3).unwrap()))
    });
}

fn bench_matrix_operations(c: &mut Criterion) {
    let mut group = c.benchmark_group("matrix_ops");
    bench_matrix_operations_for::<_>(
        &mut group,
        "hyperreal",
        s,
    );
    bench_matrix_operations_for::<_>(&mut group, "hyperreal-rational", qr);
    bench_numerica_matrix_operations(&mut group, "numerica128");
    bench_symbolica_matrix_operations(&mut group, "symbolica");
    group.finish();
}

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

        // These fixtures deliberately split the signed-product reducer into
        // structural cases:
        //
        // - `dyadic_dense`: f64-origin and binary rationals should use the
        //   shift-only dyadic denominator path.
        // - `equal_decimal_den`: decimal exact rationals should use the
        //   equal-product-denominator shortcut.
        // - `mixed_prime_den`: varied prime denominators force the LCM path.
        // - `sparse_integer`: zero cofactors test term skipping and the
        //   integer/no-denominator path.
        //
        // The group is intentionally separate from `matrix_ops`; it exists to
        // trace and benchmark reduction machinery choices without changing the
        // stable comparison table.
        //
        // 2026-05-09 targeted Criterion, 150 samples/6s:
        // hyperreal-rational mat4 reciprocal was dyadic 10.83 us, equal-den
        // 26.61 us, mixed-prime 54.64 us, sparse-integer 7.21 us; mat4
        // div_matrix was 16.08 us, 53.94 us, 102.48 us, and 10.47 us. The
        // approximate guard stayed flat by shape, roughly 79-80 ns for mat3
        // reciprocal and 145-146 ns for mat4 reciprocal, confirming the
        // product-sum reducer remains gated away from compact scalars.
        // Dispatch trace confirmed dyadic/sparse use `dyadic-shared-denominator`,
        // mixed-prime uses `lcm-shared-denominator`, and decimal denominators
        // mostly fall through to LCM after intermediate cofactors alter the
        // exact product denominators.
        trace_dispatch_row(format!("matrix_forms/{label}/{name}/mat3 reciprocal"), || {
            black_box(black_box(lhs3.clone()).reciprocal().unwrap());
        });
        trace_dispatch_row(format!("matrix_forms/{label}/{name}/mat3 powi_negative"), || {
            black_box(black_box(lhs3.clone()).powi(-2).unwrap());
        });
        trace_dispatch_row(format!("matrix_forms/{label}/{name}/mat3 div_matrix"), || {
            black_box((black_box(lhs3.clone()) / black_box(rhs3.clone())).unwrap());
        });
        trace_dispatch_row(format!("matrix_forms/{label}/{name}/mat4 reciprocal"), || {
            black_box(black_box(lhs4.clone()).reciprocal().unwrap());
        });
        trace_dispatch_row(format!("matrix_forms/{label}/{name}/mat4 powi_negative"), || {
            black_box(black_box(lhs4.clone()).powi(-2).unwrap());
        });
        trace_dispatch_row(format!("matrix_forms/{label}/{name}/mat4 div_matrix"), || {
            black_box((black_box(lhs4.clone()) / black_box(rhs4.clone())).unwrap());
        });

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

fn dyadic_from_f64_ratio(numerator: i64, denominator: u64) -> Real {
    Real::try_from(numerator as f64 / denominator as f64).unwrap()
}

fn bench_targeted_matrix_forms(c: &mut Criterion) {
    let mut group = c.benchmark_group("matrix_forms");
    bench_targeted_matrix_forms_for::<_>(
        &mut group,
        "hyperreal",
        dyadic_from_f64_ratio,
    );
    bench_targeted_matrix_forms_for::<_>(
        &mut group,
        "hyperreal-rational",
        q,
    );
    group.finish();
}

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
                    "add" => lhs3_cases[index].combine(&rhs3_cases[index], &ctx, numerica_engine::Ctx::add),
                    "add_scalar" => lhs3_cases[index].map_scalar(
                        &scalar_cases[index],
                        &ctx,
                        numerica_engine::Ctx::add,
                    ),
                    "sub" => lhs3_cases[index].combine(&rhs3_cases[index], &ctx, numerica_engine::Ctx::sub),
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
                    "add" => lhs4_cases[index].combine(&rhs4_cases[index], &ctx, numerica_engine::Ctx::add),
                    "add_scalar" => lhs4_cases[index].map_scalar(
                        &scalar_cases[index],
                        &ctx,
                        numerica_engine::Ctx::add,
                    ),
                    "sub" => lhs4_cases[index].combine(&rhs4_cases[index], &ctx, numerica_engine::Ctx::sub),
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
                    "add" => lhs3_cases[index].combine(&rhs3_cases[index], &ctx, symbolica_engine::Ctx::add),
                    "add_scalar" => lhs3_cases[index].map_scalar(
                        &scalar_cases[index],
                        &ctx,
                        symbolica_engine::Ctx::add,
                    ),
                    "sub" => lhs3_cases[index].combine(&rhs3_cases[index], &ctx, symbolica_engine::Ctx::sub),
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
                    "add" => lhs4_cases[index].combine(&rhs4_cases[index], &ctx, symbolica_engine::Ctx::add),
                    "add_scalar" => lhs4_cases[index].map_scalar(
                        &scalar_cases[index],
                        &ctx,
                        symbolica_engine::Ctx::add,
                    ),
                    "sub" => lhs4_cases[index].combine(&rhs4_cases[index], &ctx, symbolica_engine::Ctx::sub),
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
