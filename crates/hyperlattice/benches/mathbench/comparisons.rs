fn bench_blas_vectors<B: Backend>(
    group: &mut BenchmarkGroup<'_, criterion::measurement::WallTime>,
    label: &str,
    lhs_cases: [SampleVec3; 4],
    rhs_cases: [SampleVec3; 4],
) {
    let blas_lhs_cases = lhs_cases.map(blas_vec3::<B>);
    let blas_rhs_cases = rhs_cases.map(blas_vec3::<B>);
    group.bench_function(format!("{label}/vec3 dot"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % blas_lhs_cases.len());
            black_box(black_box(&blas_lhs_cases[index]).dot(black_box(&blas_rhs_cases[index])))
        })
    });
    group.bench_function(format!("{label}/vec3 magnitude"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                black_box(next_case(&blas_lhs_cases, &cursor))
                    .magnitude()
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/vec3 normalize"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                black_box(next_case(&blas_lhs_cases, &cursor))
                    .normalize()
                    .unwrap(),
            )
        })
    });
}

fn bench_vectors(c: &mut Criterion) {
    let mut group = c.benchmark_group("vectors");
    let lhs_cases = sample_vec3_cases();
    let rhs_cases = sample_vec3_b_cases();

    bench_blas_vectors::<ApproxBackend>(&mut group, "approx", lhs_cases, rhs_cases);
    bench_blas_vectors::<HyperrealBackend>(&mut group, "hyperreal", lhs_cases, rhs_cases);

    {
        let rational_lhs = blas_vec3_rational();
        let rational_rhs = blas_vec3_b_rational();
        group.bench_function("hyperreal-rational/vec3 dot", |b| {
            b.iter(|| black_box(black_box(&rational_lhs).dot(black_box(&rational_rhs))))
        });
        group.bench_function("hyperreal-rational/vec3 magnitude", |b| {
            b.iter(|| black_box(black_box(&rational_lhs).magnitude().unwrap()))
        });
        group.bench_function("hyperreal-rational/vec3 normalize", |b| {
            b.iter(|| black_box(black_box(&rational_lhs).normalize().unwrap()))
        });
    }

    let mut astro_ctx = astro_backend::Ctx::new(128);
    let astro_lhs_cases =
        lhs_cases.map(|value| astro_backend::Vec3::new(&astro_ctx, value.x, value.y, value.z));
    let astro_rhs_cases =
        rhs_cases.map(|value| astro_backend::Vec3::new(&astro_ctx, value.x, value.y, value.z));
    group.bench_function("astro128/vec3 dot", |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % astro_lhs_cases.len());
            black_box(astro_lhs_cases[index].clone())
                .dot(black_box(&astro_rhs_cases[index]), &astro_ctx)
        })
    });
    group.bench_function("astro128/vec3 magnitude", |b| {
        let cursor = Cell::new(0);
        b.iter(|| black_box(next_case(&astro_lhs_cases, &cursor).clone()).magnitude(&astro_ctx))
    });
    group.bench_function("astro128/vec3 normalize", |b| {
        let cursor = Cell::new(0);
        b.iter(|| black_box(next_case(&astro_lhs_cases, &cursor).clone()).normalize(&astro_ctx))
    });

    let numerica_ctx = numerica_backend::Ctx::new(128);
    let numerica_lhs_cases =
        lhs_cases.map(|value| numerica_backend::Vec3::new(&numerica_ctx, value.x, value.y, value.z));
    let numerica_rhs_cases =
        rhs_cases.map(|value| numerica_backend::Vec3::new(&numerica_ctx, value.x, value.y, value.z));
    group.bench_function("numerica128/vec3 dot", |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % numerica_lhs_cases.len());
            black_box(numerica_lhs_cases[index].clone())
                .dot(black_box(&numerica_rhs_cases[index]), &numerica_ctx)
        })
    });
    group.bench_function("numerica128/vec3 magnitude", |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(next_case(&numerica_lhs_cases, &cursor).clone()).magnitude(&numerica_ctx)
        })
    });
    group.bench_function("numerica128/vec3 normalize", |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(next_case(&numerica_lhs_cases, &cursor).clone()).normalize(&numerica_ctx)
        })
    });

    let symbolica_ctx = symbolica_backend::Ctx::new(128);
    let symbolica_lhs_cases =
        lhs_cases.map(|value| symbolica_backend::Vec3::new(&symbolica_ctx, value.x, value.y, value.z));
    let symbolica_rhs_cases =
        rhs_cases.map(|value| symbolica_backend::Vec3::new(&symbolica_ctx, value.x, value.y, value.z));
    group.bench_function("symbolica/vec3 dot", |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % symbolica_lhs_cases.len());
            black_box(symbolica_lhs_cases[index].clone())
                .dot(black_box(&symbolica_rhs_cases[index]), &symbolica_ctx)
        })
    });
    group.bench_function("symbolica/vec3 magnitude", |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(next_case(&symbolica_lhs_cases, &cursor).clone()).magnitude(&symbolica_ctx)
        })
    });
    group.bench_function("symbolica/vec3 normalize", |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(next_case(&symbolica_lhs_cases, &cursor).clone()).normalize(&symbolica_ctx)
        })
    });

    // Keep the mutable astro context live in this group so its constants cache
    // remains initialized before the trig benchmarks construct their own cache.
    black_box(&mut astro_ctx);
    group.finish();
}

fn bench_blas_matrix3<B: Backend>(
    group: &mut BenchmarkGroup<'_, criterion::measurement::WallTime>,
    label: &str,
    lhs_cases: [SampleMat3; 4],
    rhs_cases: [SampleMat3; 4],
    vector_cases: [SampleVec3; 4],
) {
    let blas_lhs_cases = lhs_cases.map(blas_mat3::<B>);
    let blas_rhs_cases = rhs_cases.map(blas_mat3::<B>);
    let blas_vector_cases = vector_cases.map(blas_vec3::<B>);
    group.bench_function(format!("{label}/mat3 determinant"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| black_box(black_box(next_case(&blas_lhs_cases, &cursor)).determinant()))
    });
    group.bench_function(format!("{label}/mat3 inverse"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                black_box(next_case(&blas_lhs_cases, &cursor).clone())
                    .inverse()
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/mat3 mul mat3"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % blas_lhs_cases.len());
            black_box(
                black_box(blas_lhs_cases[index].clone()) * black_box(blas_rhs_cases[index].clone()),
            )
        })
    });
    group.bench_function(format!("{label}/mat3 transform vec3"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % blas_lhs_cases.len());
            black_box(
                black_box(blas_lhs_cases[index].clone())
                    * black_box(blas_vector_cases[index].clone()),
            )
        })
    });
}

fn bench_matrix3(c: &mut Criterion) {
    let mut group = c.benchmark_group("matrix3");
    let lhs_cases = sample_mat3_cases();
    let rhs_cases = sample_mat3_b_cases();
    let vector_cases = sample_vec3_cases();

    bench_blas_matrix3::<ApproxBackend>(&mut group, "approx", lhs_cases, rhs_cases, vector_cases);
    bench_blas_matrix3::<HyperrealBackend>(
        &mut group,
        "hyperreal",
        lhs_cases,
        rhs_cases,
        vector_cases,
    );

    {
        let rational_lhs = blas_mat3_rational();
        let rational_rhs = blas_mat3_b_rational();
        let rational_vector = blas_vec3_rational();
        group.bench_function("hyperreal-rational/mat3 determinant", |b| {
            b.iter(|| black_box(black_box(&rational_lhs).determinant()))
        });
        group.bench_function("hyperreal-rational/mat3 inverse", |b| {
            b.iter(|| black_box(black_box(rational_lhs.clone()).inverse().unwrap()))
        });
        group.bench_function("hyperreal-rational/mat3 mul mat3", |b| {
            b.iter(|| black_box(black_box(rational_lhs.clone()) * black_box(rational_rhs.clone())))
        });
        group.bench_function("hyperreal-rational/mat3 transform vec3", |b| {
            b.iter(|| {
                black_box(black_box(rational_lhs.clone()) * black_box(rational_vector.clone()))
            })
        });
    }

    let astro_ctx = astro_backend::Ctx::new(128);
    let astro_lhs_cases = lhs_cases.map(|value| astro_backend::Mat3::new(&astro_ctx, value.m));
    let astro_rhs_cases = rhs_cases.map(|value| astro_backend::Mat3::new(&astro_ctx, value.m));
    let astro_vector_cases =
        vector_cases.map(|value| astro_backend::Vec3::new(&astro_ctx, value.x, value.y, value.z));
    group.bench_function("astro128/mat3 determinant", |b| {
        let cursor = Cell::new(0);
        b.iter(|| black_box(next_case(&astro_lhs_cases, &cursor).clone()).determinant(&astro_ctx))
    });
    group.bench_function("astro128/mat3 inverse", |b| {
        let cursor = Cell::new(0);
        b.iter(|| black_box(next_case(&astro_lhs_cases, &cursor).clone()).inverse(&astro_ctx))
    });
    group.bench_function("astro128/mat3 mul mat3", |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % astro_lhs_cases.len());
            black_box(astro_lhs_cases[index].clone())
                .mul_mat3(black_box(&astro_rhs_cases[index]), &astro_ctx)
        })
    });
    group.bench_function("astro128/mat3 transform vec3", |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % astro_lhs_cases.len());
            black_box(astro_lhs_cases[index].clone())
                .transform_vec3(black_box(&astro_vector_cases[index]), &astro_ctx)
        })
    });

    let numerica_ctx = numerica_backend::Ctx::new(128);
    let numerica_lhs_cases =
        lhs_cases.map(|value| numerica_backend::Mat3::new(&numerica_ctx, value.m));
    let numerica_rhs_cases =
        rhs_cases.map(|value| numerica_backend::Mat3::new(&numerica_ctx, value.m));
    let numerica_vector_cases = vector_cases
        .map(|value| numerica_backend::Vec3::new(&numerica_ctx, value.x, value.y, value.z));
    group.bench_function("numerica128/mat3 determinant", |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(next_case(&numerica_lhs_cases, &cursor).clone())
                .determinant(&numerica_ctx)
        })
    });
    group.bench_function("numerica128/mat3 inverse", |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(next_case(&numerica_lhs_cases, &cursor).clone()).inverse(&numerica_ctx)
        })
    });
    group.bench_function("numerica128/mat3 mul mat3", |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % numerica_lhs_cases.len());
            black_box(numerica_lhs_cases[index].clone())
                .mul_mat3(black_box(&numerica_rhs_cases[index]), &numerica_ctx)
        })
    });
    group.bench_function("numerica128/mat3 transform vec3", |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % numerica_lhs_cases.len());
            black_box(numerica_lhs_cases[index].clone())
                .transform_vec3(black_box(&numerica_vector_cases[index]), &numerica_ctx)
        })
    });

    let symbolica_ctx = symbolica_backend::Ctx::new(128);
    let symbolica_lhs_cases =
        lhs_cases.map(|value| symbolica_backend::Mat3::new(&symbolica_ctx, value.m));
    let symbolica_rhs_cases =
        rhs_cases.map(|value| symbolica_backend::Mat3::new(&symbolica_ctx, value.m));
    let symbolica_vector_cases = vector_cases
        .map(|value| symbolica_backend::Vec3::new(&symbolica_ctx, value.x, value.y, value.z));
    group.bench_function("symbolica/mat3 determinant", |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(next_case(&symbolica_lhs_cases, &cursor).clone())
                .determinant(&symbolica_ctx)
        })
    });
    group.bench_function("symbolica/mat3 inverse", |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(next_case(&symbolica_lhs_cases, &cursor).clone()).inverse(&symbolica_ctx)
        })
    });
    group.bench_function("symbolica/mat3 mul mat3", |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % symbolica_lhs_cases.len());
            black_box(symbolica_lhs_cases[index].clone())
                .mul_mat3(black_box(&symbolica_rhs_cases[index]), &symbolica_ctx)
        })
    });
    group.bench_function("symbolica/mat3 transform vec3", |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % symbolica_lhs_cases.len());
            black_box(symbolica_lhs_cases[index].clone())
                .transform_vec3(black_box(&symbolica_vector_cases[index]), &symbolica_ctx)
        })
    });

    group.finish();
}

fn bench_blas_matrix4<B: Backend>(
    group: &mut BenchmarkGroup<'_, criterion::measurement::WallTime>,
    label: &str,
    lhs_cases: [SampleMat4; 4],
    rhs_cases: [SampleMat4; 4],
    vector_cases: [SampleVec4; 4],
) {
    let blas_lhs_cases = lhs_cases.map(blas_mat4::<B>);
    let blas_rhs_cases = rhs_cases.map(blas_mat4::<B>);
    let blas_vector_cases = vector_cases.map(blas_vec4::<B>);
    group.bench_function(format!("{label}/mat4 determinant"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| black_box(black_box(next_case(&blas_lhs_cases, &cursor)).determinant()))
    });
    group.bench_function(format!("{label}/mat4 inverse"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                black_box(next_case(&blas_lhs_cases, &cursor).clone())
                    .inverse()
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/mat4 mul mat4"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % blas_lhs_cases.len());
            black_box(
                black_box(blas_lhs_cases[index].clone()) * black_box(blas_rhs_cases[index].clone()),
            )
        })
    });
    group.bench_function(format!("{label}/mat4 transform vec4"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % blas_lhs_cases.len());
            black_box(
                black_box(blas_lhs_cases[index].clone())
                    * black_box(blas_vector_cases[index].clone()),
            )
        })
    });
}

fn bench_matrix4(c: &mut Criterion) {
    let mut group = c.benchmark_group("matrix4");
    let lhs_cases = sample_mat4_cases();
    let rhs_cases = sample_mat4_b_cases();
    let vector_cases = sample_vec4_cases();

    bench_blas_matrix4::<ApproxBackend>(&mut group, "approx", lhs_cases, rhs_cases, vector_cases);
    bench_blas_matrix4::<HyperrealBackend>(
        &mut group,
        "hyperreal",
        lhs_cases,
        rhs_cases,
        vector_cases,
    );

    {
        let rational_lhs = blas_mat4_rational();
        let rational_rhs = blas_mat4_b_rational();
        let rational_vector = blas_vec4_rational();
        group.bench_function("hyperreal-rational/mat4 determinant", |b| {
            b.iter(|| black_box(black_box(&rational_lhs).determinant()))
        });
        group.bench_function("hyperreal-rational/mat4 inverse", |b| {
            b.iter(|| black_box(black_box(rational_lhs.clone()).inverse().unwrap()))
        });
        group.bench_function("hyperreal-rational/mat4 mul mat4", |b| {
            b.iter(|| black_box(black_box(rational_lhs.clone()) * black_box(rational_rhs.clone())))
        });
        group.bench_function("hyperreal-rational/mat4 transform vec4", |b| {
            b.iter(|| {
                black_box(black_box(rational_lhs.clone()) * black_box(rational_vector.clone()))
            })
        });
    }

    let astro_ctx = astro_backend::Ctx::new(128);
    let astro_lhs_cases = lhs_cases.map(|value| astro_backend::Mat4::new(&astro_ctx, value.m));
    let astro_rhs_cases = rhs_cases.map(|value| astro_backend::Mat4::new(&astro_ctx, value.m));
    let astro_vector_cases = vector_cases
        .map(|value| astro_backend::Vec4::new(&astro_ctx, value.x, value.y, value.z, value.w));
    group.bench_function("astro128/mat4 determinant", |b| {
        let cursor = Cell::new(0);
        b.iter(|| black_box(next_case(&astro_lhs_cases, &cursor).clone()).determinant(&astro_ctx))
    });
    group.bench_function("astro128/mat4 inverse", |b| {
        let cursor = Cell::new(0);
        b.iter(|| black_box(next_case(&astro_lhs_cases, &cursor).clone()).inverse(&astro_ctx))
    });
    group.bench_function("astro128/mat4 mul mat4", |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % astro_lhs_cases.len());
            black_box(astro_lhs_cases[index].clone())
                .mul_mat4(black_box(&astro_rhs_cases[index]), &astro_ctx)
        })
    });
    group.bench_function("astro128/mat4 transform vec4", |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % astro_lhs_cases.len());
            black_box(astro_lhs_cases[index].clone())
                .transform_vec4(black_box(&astro_vector_cases[index]), &astro_ctx)
        })
    });

    let numerica_ctx = numerica_backend::Ctx::new(128);
    let numerica_lhs_cases =
        lhs_cases.map(|value| numerica_backend::Mat4::new(&numerica_ctx, value.m));
    let numerica_rhs_cases =
        rhs_cases.map(|value| numerica_backend::Mat4::new(&numerica_ctx, value.m));
    let numerica_vector_cases = vector_cases.map(|value| {
        numerica_backend::Vec4::new(&numerica_ctx, value.x, value.y, value.z, value.w)
    });
    group.bench_function("numerica128/mat4 determinant", |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(next_case(&numerica_lhs_cases, &cursor).clone())
                .determinant(&numerica_ctx)
        })
    });
    group.bench_function("numerica128/mat4 inverse", |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(next_case(&numerica_lhs_cases, &cursor).clone()).inverse(&numerica_ctx)
        })
    });
    group.bench_function("numerica128/mat4 mul mat4", |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % numerica_lhs_cases.len());
            black_box(numerica_lhs_cases[index].clone())
                .mul_mat4(black_box(&numerica_rhs_cases[index]), &numerica_ctx)
        })
    });
    group.bench_function("numerica128/mat4 transform vec4", |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % numerica_lhs_cases.len());
            black_box(numerica_lhs_cases[index].clone())
                .transform_vec4(black_box(&numerica_vector_cases[index]), &numerica_ctx)
        })
    });

    let symbolica_ctx = symbolica_backend::Ctx::new(128);
    let symbolica_lhs_cases =
        lhs_cases.map(|value| symbolica_backend::Mat4::new(&symbolica_ctx, value.m));
    let symbolica_rhs_cases =
        rhs_cases.map(|value| symbolica_backend::Mat4::new(&symbolica_ctx, value.m));
    let symbolica_vector_cases = vector_cases.map(|value| {
        symbolica_backend::Vec4::new(&symbolica_ctx, value.x, value.y, value.z, value.w)
    });
    group.bench_function("symbolica/mat4 determinant", |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(next_case(&symbolica_lhs_cases, &cursor).clone())
                .determinant(&symbolica_ctx)
        })
    });
    group.bench_function("symbolica/mat4 inverse", |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(next_case(&symbolica_lhs_cases, &cursor).clone()).inverse(&symbolica_ctx)
        })
    });
    group.bench_function("symbolica/mat4 mul mat4", |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % symbolica_lhs_cases.len());
            black_box(symbolica_lhs_cases[index].clone())
                .mul_mat4(black_box(&symbolica_rhs_cases[index]), &symbolica_ctx)
        })
    });
    group.bench_function("symbolica/mat4 transform vec4", |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % symbolica_lhs_cases.len());
            black_box(symbolica_lhs_cases[index].clone())
                .transform_vec4(black_box(&symbolica_vector_cases[index]), &symbolica_ctx)
        })
    });

    group.finish();
}

#[derive(Clone, Copy)]
struct TrigCase {
    name: &'static str,
    value: f64,
}

fn trig_cases() -> [TrigCase; 6] {
    [
        TrigCase {
            name: "0.1",
            value: 0.1,
        },
        TrigCase {
            name: "1.23456789",
            value: 1.23456789,
        },
        TrigCase {
            name: "1e6",
            value: 1.0e6,
        },
        TrigCase {
            name: "1e30",
            value: 1.0e30,
        },
        TrigCase {
            name: "pi_7",
            value: std::f64::consts::PI / 7.0,
        },
        TrigCase {
            name: "1000pi_eps",
            value: 1000.0 * std::f64::consts::PI + 1.0e-20,
        },
    ]
}

fn inverse_unit_cases() -> [TrigCase; 4] {
    [
        TrigCase {
            name: "0.5",
            value: 0.5,
        },
        TrigCase {
            name: "neg_0.999999",
            value: -0.999_999,
        },
        TrigCase {
            name: "0.999999",
            value: 0.999_999,
        },
        TrigCase {
            name: "1e-12",
            value: 1.0e-12,
        },
    ]
}

fn inverse_real_cases() -> [TrigCase; 4] {
    [
        TrigCase {
            name: "0.5",
            value: 0.5,
        },
        TrigCase {
            name: "neg_1e-12",
            value: -1.0e-12,
        },
        TrigCase {
            name: "1e6",
            value: 1.0e6,
        },
        TrigCase {
            name: "neg_1e6",
            value: -1.0e6,
        },
    ]
}

fn inverse_acosh_cases() -> [TrigCase; 4] {
    [
        TrigCase {
            name: "9",
            value: 9.0,
        },
        TrigCase {
            name: "1_plus_1e-12",
            value: 1.0 + 1.0e-12,
        },
        TrigCase {
            name: "1e6",
            value: 1.0e6,
        },
        TrigCase {
            name: "e",
            value: std::f64::consts::E,
        },
    ]
}

fn one_e_minus_20() -> HyperrealScalar {
    "0.00000000000000000001".parse::<Rational>().unwrap().into()
}

fn trig_rational(case: TrigCase) -> HyperrealScalar {
    match case.name {
        "0.1" => q(1, 10),
        "1.23456789" => q(123_456_789, 100_000_000),
        "1e6" => 1_000_000.into(),
        "1e30" => 1_000_000_000_000_000_000_000_000_000_000_i128.into(),
        "pi_7" => (HyperrealScalar::pi() / HyperrealScalar::from(7)).unwrap(),
        "1000pi_eps" => HyperrealScalar::pi() * HyperrealScalar::from(1000) + one_e_minus_20(),
        _ => unreachable!("all trig cases are covered"),
    }
}

fn inverse_rational(case: TrigCase) -> HyperrealScalar {
    match case.name {
        "0.5" => q(1, 2),
        "neg_0.999999" => q(-999_999, 1_000_000),
        "0.999999" => q(999_999, 1_000_000),
        "1e-12" => q(1, 1_000_000_000_000),
        "neg_1e-12" => q(-1, 1_000_000_000_000),
        "1e6" => 1_000_000.into(),
        "neg_1e6" => (-1_000_000).into(),
        "9" => 9.into(),
        "1_plus_1e-12" => q(1_000_000_000_001, 1_000_000_000_000),
        "e" => HyperrealScalar::e(),
        _ => unreachable!("all inverse cases are covered"),
    }
}

fn bench_blas_scalar_trig<B: Backend>(
    group: &mut BenchmarkGroup<'_, criterion::measurement::WallTime>,
    label: &str,
) {
    for case in trig_cases() {
        let blas_value = s::<B>(case.value);
        group.bench_function(format!("{label}/{}/sin", case.name), |b| {
            b.iter(|| black_box(realistic_blas::sin(black_box(blas_value.clone()))))
        });
        group.bench_function(format!("{label}/{}/cos", case.name), |b| {
            b.iter(|| black_box(realistic_blas::cos(black_box(blas_value.clone()))))
        });
    }
    for case in inverse_unit_cases() {
        let blas_value = s::<B>(case.value);
        group.bench_function(format!("{label}/{}/asin", case.name), |b| {
            b.iter(|| black_box(realistic_blas::asin(black_box(blas_value.clone())).unwrap()))
        });
        group.bench_function(format!("{label}/{}/acos", case.name), |b| {
            b.iter(|| black_box(realistic_blas::acos(black_box(blas_value.clone())).unwrap()))
        });
        group.bench_function(format!("{label}/{}/atanh", case.name), |b| {
            b.iter(|| black_box(realistic_blas::atanh(black_box(blas_value.clone())).unwrap()))
        });
    }
    for case in inverse_real_cases() {
        let blas_value = s::<B>(case.value);
        group.bench_function(format!("{label}/{}/atan", case.name), |b| {
            b.iter(|| black_box(realistic_blas::atan(black_box(blas_value.clone())).unwrap()))
        });
        group.bench_function(format!("{label}/{}/asinh", case.name), |b| {
            b.iter(|| black_box(realistic_blas::asinh(black_box(blas_value.clone())).unwrap()))
        });
    }
    for case in inverse_acosh_cases() {
        let blas_value = s::<B>(case.value);
        group.bench_function(format!("{label}/{}/acosh", case.name), |b| {
            b.iter(|| black_box(realistic_blas::acosh(black_box(blas_value.clone())).unwrap()))
        });
    }
}

fn bench_scalar_trig(c: &mut Criterion) {
    let mut group = c.benchmark_group("scalar_trig");

    bench_blas_scalar_trig::<ApproxBackend>(&mut group, "approx");
    bench_blas_scalar_trig::<HyperrealBackend>(&mut group, "hyperreal");

    for case in trig_cases() {
        let rational_value = trig_rational(case);
        group.bench_function(format!("hyperreal-rational/{}/sin", case.name), |b| {
            b.iter(|| black_box(realistic_blas::sin(black_box(rational_value.clone()))))
        });
        group.bench_function(format!("hyperreal-rational/{}/cos", case.name), |b| {
            b.iter(|| black_box(realistic_blas::cos(black_box(rational_value.clone()))))
        });
    }
    for case in inverse_unit_cases() {
        let rational_value = inverse_rational(case);
        group.bench_function(format!("hyperreal-rational/{}/asin", case.name), |b| {
            b.iter(|| black_box(realistic_blas::asin(black_box(rational_value.clone())).unwrap()))
        });
        group.bench_function(format!("hyperreal-rational/{}/acos", case.name), |b| {
            b.iter(|| black_box(realistic_blas::acos(black_box(rational_value.clone())).unwrap()))
        });
        group.bench_function(format!("hyperreal-rational/{}/atanh", case.name), |b| {
            b.iter(|| black_box(realistic_blas::atanh(black_box(rational_value.clone())).unwrap()))
        });
    }
    for case in inverse_real_cases() {
        let rational_value = inverse_rational(case);
        group.bench_function(format!("hyperreal-rational/{}/atan", case.name), |b| {
            b.iter(|| black_box(realistic_blas::atan(black_box(rational_value.clone())).unwrap()))
        });
        group.bench_function(format!("hyperreal-rational/{}/asinh", case.name), |b| {
            b.iter(|| black_box(realistic_blas::asinh(black_box(rational_value.clone())).unwrap()))
        });
    }
    for case in inverse_acosh_cases() {
        let rational_value = inverse_rational(case);
        group.bench_function(format!("hyperreal-rational/{}/acosh", case.name), |b| {
            b.iter(|| black_box(realistic_blas::acosh(black_box(rational_value.clone())).unwrap()))
        });
    }

    let mut astro_ctx = astro_backend::Ctx::new(128);
    for case in trig_cases() {
        let astro_value = astro_ctx.f(case.value);
        group.bench_function(format!("astro128/{}/sin", case.name), |b| {
            b.iter(|| astro_ctx.sin(black_box(&astro_value)))
        });
        group.bench_function(format!("astro128/{}/cos", case.name), |b| {
            b.iter(|| astro_ctx.cos(black_box(&astro_value)))
        });
    }
    for case in inverse_unit_cases() {
        let astro_value = astro_ctx.f(case.value);
        group.bench_function(format!("astro128/{}/asin", case.name), |b| {
            b.iter(|| astro_ctx.asin(black_box(&astro_value)))
        });
        group.bench_function(format!("astro128/{}/acos", case.name), |b| {
            b.iter(|| astro_ctx.acos(black_box(&astro_value)))
        });
        group.bench_function(format!("astro128/{}/atanh", case.name), |b| {
            b.iter(|| astro_ctx.atanh(black_box(&astro_value)))
        });
    }
    for case in inverse_real_cases() {
        let astro_value = astro_ctx.f(case.value);
        group.bench_function(format!("astro128/{}/atan", case.name), |b| {
            b.iter(|| astro_ctx.atan(black_box(&astro_value)))
        });
        group.bench_function(format!("astro128/{}/asinh", case.name), |b| {
            b.iter(|| astro_ctx.asinh(black_box(&astro_value)))
        });
    }
    for case in inverse_acosh_cases() {
        let astro_value = astro_ctx.f(case.value);
        group.bench_function(format!("astro128/{}/acosh", case.name), |b| {
            b.iter(|| astro_ctx.acosh(black_box(&astro_value)))
        });
    }

    let numerica_ctx = numerica_backend::Ctx::new(128);
    for case in trig_cases() {
        let numerica_value = numerica_ctx.f(case.value);
        group.bench_function(format!("numerica128/{}/sin", case.name), |b| {
            b.iter(|| numerica_ctx.sin(black_box(&numerica_value)))
        });
        group.bench_function(format!("numerica128/{}/cos", case.name), |b| {
            b.iter(|| numerica_ctx.cos(black_box(&numerica_value)))
        });
    }
    for case in inverse_unit_cases() {
        let numerica_value = numerica_ctx.f(case.value);
        group.bench_function(format!("numerica128/{}/asin", case.name), |b| {
            b.iter(|| numerica_ctx.asin(black_box(&numerica_value)))
        });
        group.bench_function(format!("numerica128/{}/acos", case.name), |b| {
            b.iter(|| numerica_ctx.acos(black_box(&numerica_value)))
        });
        group.bench_function(format!("numerica128/{}/atanh", case.name), |b| {
            b.iter(|| numerica_ctx.atanh(black_box(&numerica_value)))
        });
    }
    for case in inverse_real_cases() {
        let numerica_value = numerica_ctx.f(case.value);
        group.bench_function(format!("numerica128/{}/atan", case.name), |b| {
            b.iter(|| numerica_ctx.atan(black_box(&numerica_value)))
        });
        group.bench_function(format!("numerica128/{}/asinh", case.name), |b| {
            b.iter(|| numerica_ctx.asinh(black_box(&numerica_value)))
        });
    }
    for case in inverse_acosh_cases() {
        let numerica_value = numerica_ctx.f(case.value);
        group.bench_function(format!("numerica128/{}/acosh", case.name), |b| {
            b.iter(|| numerica_ctx.acosh(black_box(&numerica_value)))
        });
    }

    let symbolica_ctx = symbolica_backend::Ctx::new(128);
    for case in trig_cases() {
        let symbolica_value = symbolica_ctx.f(case.value);
        group.bench_function(format!("symbolica/{}/sin", case.name), |b| {
            b.iter(|| symbolica_ctx.sin(black_box(&symbolica_value)))
        });
        group.bench_function(format!("symbolica/{}/cos", case.name), |b| {
            b.iter(|| symbolica_ctx.cos(black_box(&symbolica_value)))
        });
    }
    for case in inverse_unit_cases() {
        let symbolica_value = symbolica_ctx.f(case.value);
        group.bench_function(format!("symbolica/{}/asin", case.name), |b| {
            b.iter(|| symbolica_ctx.asin(black_box(&symbolica_value)))
        });
        group.bench_function(format!("symbolica/{}/acos", case.name), |b| {
            b.iter(|| symbolica_ctx.acos(black_box(&symbolica_value)))
        });
        group.bench_function(format!("symbolica/{}/atanh", case.name), |b| {
            b.iter(|| symbolica_ctx.atanh(black_box(&symbolica_value)))
        });
    }
    for case in inverse_real_cases() {
        let symbolica_value = symbolica_ctx.f(case.value);
        group.bench_function(format!("symbolica/{}/atan", case.name), |b| {
            b.iter(|| symbolica_ctx.atan(black_box(&symbolica_value)))
        });
        group.bench_function(format!("symbolica/{}/asinh", case.name), |b| {
            b.iter(|| symbolica_ctx.asinh(black_box(&symbolica_value)))
        });
    }
    for case in inverse_acosh_cases() {
        let symbolica_value = symbolica_ctx.f(case.value);
        group.bench_function(format!("symbolica/{}/acosh", case.name), |b| {
            b.iter(|| symbolica_ctx.acosh(black_box(&symbolica_value)))
        });
    }

    group.finish();
}

fn abort_signal() -> realistic_blas::AbortSignal {
    std::sync::Arc::new(std::sync::atomic::AtomicBool::new(false))
}
