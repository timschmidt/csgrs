#[derive(Clone, Copy, Debug)]
struct SampleVec3 {
    x: f64,
    y: f64,
    z: f64,
}

#[derive(Clone, Copy, Debug)]
struct SampleVec4 {
    x: f64,
    y: f64,
    z: f64,
    w: f64,
}

#[derive(Clone, Copy, Debug)]
struct SampleMat3 {
    m: [[f64; 3]; 3],
}

#[derive(Clone, Copy, Debug)]
struct SampleMat4 {
    m: [[f64; 4]; 4],
}

fn sample_vec3() -> SampleVec3 {
    SampleVec3 {
        x: 1.23456789012345,
        y: -2.34567890123456,
        z: 3.45678901234567,
    }
}

fn sample_vec3_cases() -> [SampleVec3; 4] {
    [
        sample_vec3(),
        SampleVec3 {
            x: 1.0e-9,
            y: -2.0e-9,
            z: 3.0e-9,
        },
        SampleVec3 {
            x: 1.0e9,
            y: 1.0,
            z: -1.0e9,
        },
        SampleVec3 {
            x: std::f64::consts::PI,
            y: -std::f64::consts::E,
            z: 1.0e-12,
        },
    ]
}

fn sample_vec3_b() -> SampleVec3 {
    SampleVec3 {
        x: -0.98765432101234,
        y: 4.21098765432109,
        z: -5.67890123456789,
    }
}

fn sample_vec3_b_cases() -> [SampleVec3; 4] {
    [
        sample_vec3_b(),
        SampleVec3 {
            x: -3.0e-9,
            y: 5.0e-9,
            z: -7.0e-9,
        },
        SampleVec3 {
            x: -1.0e9,
            y: 2.0,
            z: 1.0e9,
        },
        SampleVec3 {
            x: -std::f64::consts::FRAC_1_PI,
            y: std::f64::consts::SQRT_2,
            z: -1.0e-12,
        },
    ]
}

fn sample_vec4() -> SampleVec4 {
    SampleVec4 {
        x: 3.0,
        y: 4.0,
        z: 5.0,
        w: 1.0,
    }
}

fn sample_vec4_cases() -> [SampleVec4; 4] {
    [
        sample_vec4(),
        SampleVec4 {
            x: 1.0e-9,
            y: -2.0e-9,
            z: 3.0e-9,
            w: -4.0e-9,
        },
        SampleVec4 {
            x: 1.0e9,
            y: -1.0e9,
            z: 1.0,
            w: -1.0,
        },
        SampleVec4 {
            x: std::f64::consts::PI,
            y: -std::f64::consts::E,
            z: std::f64::consts::SQRT_2,
            w: 1.0e-12,
        },
    ]
}

fn sample_vec4_b_cases() -> [SampleVec4; 4] {
    [
        SampleVec4 {
            x: -1.0,
            y: 2.0,
            z: -3.0,
            w: 4.0,
        },
        SampleVec4 {
            x: -4.0e-9,
            y: 3.0e-9,
            z: -2.0e-9,
            w: 1.0e-9,
        },
        SampleVec4 {
            x: -1.0e9,
            y: 1.0e9,
            z: -2.0,
            w: 2.0,
        },
        SampleVec4 {
            x: -std::f64::consts::FRAC_1_PI,
            y: std::f64::consts::FRAC_2_PI,
            z: -std::f64::consts::FRAC_2_SQRT_PI,
            w: 1.0e-12,
        },
    ]
}

fn sample_mat3() -> SampleMat3 {
    SampleMat3 {
        m: [[1.2, 0.3, -0.7], [2.1, -1.5, 0.9], [0.4, 3.3, 2.2]],
    }
}

fn sample_mat3_cases() -> [SampleMat3; 4] {
    [
        sample_mat3(),
        SampleMat3 {
            m: [
                [1.0, 1.0, 1.0],
                [1.0, 1.0 + 1.0e-6, 1.0],
                [1.0, 1.0, 1.0 + 2.0e-6],
            ],
        },
        SampleMat3 {
            m: [[1.0e6, 2.0, -3.0], [4.0, -1.0e-6, 6.0], [-7.0, 8.0, 9.0e3]],
        },
        SampleMat3 {
            m: [
                [std::f64::consts::PI, -std::f64::consts::E, 1.0e-9],
                [std::f64::consts::SQRT_2, 1.0, -2.0],
                [3.0, -5.0, 8.0],
            ],
        },
    ]
}

fn sample_mat3_b() -> SampleMat3 {
    SampleMat3 {
        m: [[-0.8, 1.1, 0.5], [2.7, 0.6, -1.4], [3.2, -0.9, 1.8]],
    }
}

fn sample_mat3_b_cases() -> [SampleMat3; 4] {
    [
        sample_mat3_b(),
        SampleMat3 {
            m: [
                [2.0, 2.0, 2.0],
                [2.0, 2.0 + 3.0e-6, 2.0],
                [2.0, 2.0, 2.0 + 5.0e-6],
            ],
        },
        SampleMat3 {
            m: [
                [-1.0e5, 3.0, 5.0],
                [7.0, 1.0e-6, -11.0],
                [13.0, -17.0, 1.0e4],
            ],
        },
        SampleMat3 {
            m: [
                [
                    -std::f64::consts::FRAC_1_PI,
                    std::f64::consts::FRAC_2_PI,
                    -1.0e-9,
                ],
                [std::f64::consts::FRAC_2_SQRT_PI, -1.0, 2.0],
                [-3.0, 5.0, -8.0],
            ],
        },
    ]
}

fn sample_mat4() -> SampleMat4 {
    SampleMat4 {
        m: [
            [1.0, 2.0, 3.0, 4.0],
            [0.0, 1.0, 4.0, 2.0],
            [5.0, 6.0, 0.0, 1.0],
            [2.0, 7.0, 1.0, 3.0],
        ],
    }
}

fn sample_mat4_cases() -> [SampleMat4; 4] {
    [
        sample_mat4(),
        SampleMat4 {
            m: [
                [1.0, 1.0, 1.0, 1.0],
                [1.0, 1.0 + 1.0e-6, 1.0, 1.0],
                [1.0, 1.0, 1.0 + 2.0e-6, 1.0],
                [1.0, 1.0, 1.0, 1.0 + 3.0e-6],
            ],
        },
        SampleMat4 {
            m: [
                [1.0e6, 2.0, -3.0, 4.0],
                [5.0, -1.0e-6, 7.0, -8.0],
                [9.0, -10.0, 1.0e4, 12.0],
                [-13.0, 14.0, -15.0, 1.0e-3],
            ],
        },
        SampleMat4 {
            m: [
                [std::f64::consts::PI, -std::f64::consts::E, 1.0e-9, 2.0],
                [std::f64::consts::SQRT_2, 1.0, -2.0, 3.0],
                [5.0, -8.0, 13.0, -21.0],
                [34.0, -55.0, 89.0, 144.0],
            ],
        },
    ]
}

fn sample_mat4_b() -> SampleMat4 {
    SampleMat4 {
        m: [
            [2.0, 0.0, 1.0, 3.0],
            [3.0, 5.0, 7.0, 11.0],
            [11.0, 13.0, 17.0, 19.0],
            [23.0, 29.0, 31.0, 37.0],
        ],
    }
}

fn sample_mat4_b_cases() -> [SampleMat4; 4] {
    [
        sample_mat4_b(),
        SampleMat4 {
            m: [
                [2.0, 2.0, 2.0, 2.0],
                [2.0, 2.0 + 5.0e-6, 2.0, 2.0],
                [2.0, 2.0, 2.0 + 7.0e-6, 2.0],
                [2.0, 2.0, 2.0, 2.0 + 11.0e-6],
            ],
        },
        SampleMat4 {
            m: [
                [-1.0e5, 3.0, 5.0, -7.0],
                [11.0, 1.0e-6, -13.0, 17.0],
                [-19.0, 23.0, 1.0e4, -29.0],
                [31.0, -37.0, 41.0, -1.0e-3],
            ],
        },
        SampleMat4 {
            m: [
                [
                    -std::f64::consts::FRAC_1_PI,
                    std::f64::consts::FRAC_2_PI,
                    -1.0e-9,
                    -2.0,
                ],
                [std::f64::consts::FRAC_2_SQRT_PI, -1.0, 2.0, -3.0],
                [-5.0, 8.0, -13.0, 21.0],
                [-34.0, 55.0, -89.0, -144.0],
            ],
        },
    ]
}

fn next_case<'a, T>(cases: &'a [T], cursor: &Cell<usize>) -> &'a T {
    let index = cursor.get();
    cursor.set((index + 1) % cases.len());
    &cases[index]
}

fn blas_vec3<B: Backend>(value: SampleVec3) -> Vector3<B> {
    blas_vec3_with(value, s::<B>)
}

fn blas_vec3_with<B, F>(value: SampleVec3, make_scalar: F) -> Vector3<B>
where
    B: Backend,
    F: Copy + Fn(f64) -> Scalar<B>,
{
    Vector3::new([
        make_scalar(value.x),
        make_scalar(value.y),
        make_scalar(value.z),
    ])
}

fn blas_vec4<B: Backend>(value: SampleVec4) -> Vector4<B> {
    blas_vec4_with(value, s::<B>)
}

fn blas_vec4_with<B, F>(value: SampleVec4, make_scalar: F) -> Vector4<B>
where
    B: Backend,
    F: Copy + Fn(f64) -> Scalar<B>,
{
    Vector4::new([
        make_scalar(value.x),
        make_scalar(value.y),
        make_scalar(value.z),
        make_scalar(value.w),
    ])
}

fn blas_mat3<B: Backend>(value: SampleMat3) -> Matrix3<B> {
    blas_mat3_with(value, s::<B>)
}

fn blas_mat3_with<B, F>(value: SampleMat3, make_scalar: F) -> Matrix3<B>
where
    B: Backend,
    F: Copy + Fn(f64) -> Scalar<B>,
{
    Matrix3::new(value.m.map(|row| row.map(make_scalar)))
}

fn blas_mat4<B: Backend>(value: SampleMat4) -> Matrix4<B> {
    blas_mat4_with(value, s::<B>)
}

fn blas_mat4_with<B, F>(value: SampleMat4, make_scalar: F) -> Matrix4<B>
where
    B: Backend,
    F: Copy + Fn(f64) -> Scalar<B>,
{
    Matrix4::new(value.m.map(|row| row.map(make_scalar)))
}

fn blas_vec3_rational() -> Vector3<HyperrealBackend> {
    Vector3::new([
        q(123_456_789_012_345, 100_000_000_000_000),
        q(-234_567_890_123_456, 100_000_000_000_000),
        q(345_678_901_234_567, 100_000_000_000_000),
    ])
}

fn blas_vec3_b_rational() -> Vector3<HyperrealBackend> {
    Vector3::new([
        q(-98_765_432_101_234, 100_000_000_000_000),
        q(421_098_765_432_109, 100_000_000_000_000),
        q(-567_890_123_456_789, 100_000_000_000_000),
    ])
}

fn blas_vec4_rational() -> Vector4<HyperrealBackend> {
    Vector4::new([3.into(), 4.into(), 5.into(), 1.into()])
}

fn blas_mat3_rational() -> Matrix3<HyperrealBackend> {
    Matrix3::new([
        [q(12, 10), q(3, 10), q(-7, 10)],
        [q(21, 10), q(-15, 10), q(9, 10)],
        [q(4, 10), q(33, 10), q(22, 10)],
    ])
}

fn blas_mat3_b_rational() -> Matrix3<HyperrealBackend> {
    Matrix3::new([
        [q(-8, 10), q(11, 10), q(5, 10)],
        [q(27, 10), q(6, 10), q(-14, 10)],
        [q(32, 10), q(-9, 10), q(18, 10)],
    ])
}

fn blas_mat4_rational() -> Matrix4<HyperrealBackend> {
    Matrix4::new([
        [1.into(), 2.into(), 3.into(), 4.into()],
        [0.into(), 1.into(), 4.into(), 2.into()],
        [5.into(), 6.into(), 0.into(), 1.into()],
        [2.into(), 7.into(), 1.into(), 3.into()],
    ])
}

fn blas_mat4_b_rational() -> Matrix4<HyperrealBackend> {
    Matrix4::new([
        [2.into(), 0.into(), 1.into(), 3.into()],
        [3.into(), 5.into(), 7.into(), 11.into()],
        [11.into(), 13.into(), 17.into(), 19.into()],
        [23.into(), 29.into(), 31.into(), 37.into()],
    ])
}

fn ratio_matrix3_with<B, F>(entries: [[(i64, u64); 3]; 3], make_ratio: F) -> Matrix3<B>
where
    B: Backend,
    F: Copy + Fn(i64, u64) -> Scalar<B>,
{
    Matrix3::new(entries.map(|row| row.map(|(n, d)| make_ratio(n, d))))
}

fn ratio_matrix4_with<B, F>(entries: [[(i64, u64); 4]; 4], make_ratio: F) -> Matrix4<B>
where
    B: Backend,
    F: Copy + Fn(i64, u64) -> Scalar<B>,
{
    Matrix4::new(entries.map(|row| row.map(|(n, d)| make_ratio(n, d))))
}

#[derive(Clone)]
struct TargetedMatrixForm<B: Backend> {
    name: &'static str,
    lhs3: Matrix3<B>,
    rhs3: Matrix3<B>,
    lhs4: Matrix4<B>,
    rhs4: Matrix4<B>,
}

fn targeted_matrix_forms_with<B, F>(make_ratio: F) -> [TargetedMatrixForm<B>; 4]
where
    B: Backend,
    F: Copy + Fn(i64, u64) -> Scalar<B>,
{
    [
        TargetedMatrixForm {
            name: "dyadic_dense",
            lhs3: ratio_matrix3_with(
                [
                    [(9, 8), (3, 16), (-5, 8)],
                    [(7, 4), (-11, 8), (13, 16)],
                    [(5, 8), (17, 16), (19, 8)],
                ],
                make_ratio,
            ),
            rhs3: ratio_matrix3_with(
                [
                    [(11, 8), (-3, 16), (7, 8)],
                    [(5, 4), (13, 8), (-9, 16)],
                    [(1, 8), (15, 16), (17, 8)],
                ],
                make_ratio,
            ),
            lhs4: ratio_matrix4_with(
                [
                    [(9, 8), (3, 16), (-5, 8), (7, 4)],
                    [(11, 8), (-13, 16), (15, 8), (5, 16)],
                    [(-7, 8), (17, 16), (19, 8), (-3, 4)],
                    [(5, 8), (21, 16), (-11, 8), (23, 16)],
                ],
                make_ratio,
            ),
            rhs4: ratio_matrix4_with(
                [
                    [(13, 8), (5, 16), (7, 8), (-9, 4)],
                    [(3, 8), (17, 16), (-5, 8), (11, 16)],
                    [(19, 8), (-7, 16), (23, 8), (5, 4)],
                    [(-1, 8), (9, 16), (15, 8), (25, 16)],
                ],
                make_ratio,
            ),
        },
        TargetedMatrixForm {
            name: "equal_decimal_den",
            lhs3: ratio_matrix3_with(
                [
                    [(12, 10), (3, 10), (-7, 10)],
                    [(21, 10), (-15, 10), (9, 10)],
                    [(4, 10), (33, 10), (22, 10)],
                ],
                make_ratio,
            ),
            rhs3: ratio_matrix3_with(
                [
                    [(-8, 10), (11, 10), (5, 10)],
                    [(27, 10), (6, 10), (-14, 10)],
                    [(32, 10), (-9, 10), (18, 10)],
                ],
                make_ratio,
            ),
            lhs4: ratio_matrix4_with(
                [
                    [(11, 10), (2, 10), (3, 10), (4, 10)],
                    [(5, 10), (17, 10), (7, 10), (-8, 10)],
                    [(9, 10), (-10, 10), (23, 10), (12, 10)],
                    [(-13, 10), (14, 10), (-15, 10), (19, 10)],
                ],
                make_ratio,
            ),
            rhs4: ratio_matrix4_with(
                [
                    [(20, 10), (3, 10), (5, 10), (-7, 10)],
                    [(11, 10), (25, 10), (-13, 10), (17, 10)],
                    [(-19, 10), (23, 10), (31, 10), (-29, 10)],
                    [(31, 10), (-37, 10), (41, 10), (43, 10)],
                ],
                make_ratio,
            ),
        },
        TargetedMatrixForm {
            name: "mixed_prime_den",
            lhs3: ratio_matrix3_with(
                [
                    [(7, 3), (-5, 7), (11, 13)],
                    [(13, 5), (17, 11), (-19, 17)],
                    [(-23, 19), (29, 23), (31, 29)],
                ],
                make_ratio,
            ),
            rhs3: ratio_matrix3_with(
                [
                    [(5, 3), (7, 5), (-11, 7)],
                    [(13, 11), (-17, 13), (19, 17)],
                    [(23, 19), (-29, 23), (37, 31)],
                ],
                make_ratio,
            ),
            lhs4: ratio_matrix4_with(
                [
                    [(7, 3), (-5, 7), (11, 13), (13, 5)],
                    [(17, 11), (-19, 17), (23, 19), (-29, 23)],
                    [(31, 29), (37, 31), (-41, 37), (43, 41)],
                    [(-47, 43), (53, 47), (59, 53), (61, 59)],
                ],
                make_ratio,
            ),
            rhs4: ratio_matrix4_with(
                [
                    [(11, 3), (13, 5), (-17, 7), (19, 11)],
                    [(23, 13), (-29, 17), (31, 19), (37, 23)],
                    [(-41, 29), (43, 31), (47, 37), (-53, 41)],
                    [(59, 43), (-61, 47), (67, 53), (71, 59)],
                ],
                make_ratio,
            ),
        },
        TargetedMatrixForm {
            name: "sparse_integer",
            lhs3: ratio_matrix3_with(
                [
                    [(2, 1), (0, 1), (1, 1)],
                    [(0, 1), (3, 1), (1, 1)],
                    [(1, 1), (0, 1), (2, 1)],
                ],
                make_ratio,
            ),
            rhs3: ratio_matrix3_with(
                [
                    [(3, 1), (1, 1), (0, 1)],
                    [(1, 1), (2, 1), (1, 1)],
                    [(0, 1), (1, 1), (4, 1)],
                ],
                make_ratio,
            ),
            lhs4: ratio_matrix4_with(
                [
                    [(2, 1), (0, 1), (1, 1), (0, 1)],
                    [(1, 1), (3, 1), (0, 1), (1, 1)],
                    [(0, 1), (2, 1), (1, 1), (0, 1)],
                    [(1, 1), (0, 1), (0, 1), (2, 1)],
                ],
                make_ratio,
            ),
            rhs4: ratio_matrix4_with(
                [
                    [(3, 1), (0, 1), (1, 1), (0, 1)],
                    [(0, 1), (2, 1), (1, 1), (1, 1)],
                    [(1, 1), (0, 1), (4, 1), (0, 1)],
                    [(0, 1), (1, 1), (0, 1), (5, 1)],
                ],
                make_ratio,
            ),
        },
    ]
}
