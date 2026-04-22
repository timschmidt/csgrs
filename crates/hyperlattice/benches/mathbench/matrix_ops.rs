fn bench_matrix_operations_for<B, F>(
    group: &mut BenchmarkGroup<'_, criterion::measurement::WallTime>,
    label: &str,
    make_scalar: F,
) where
    B: Backend,
    F: Copy + Fn(f64) -> Scalar<B>,
{
    let lhs3_cases = sample_mat3_cases().map(|value| blas_mat3_with(value, make_scalar));
    let rhs3_cases = sample_mat3_b_cases().map(|value| blas_mat3_with(value, make_scalar));
    let lhs4_cases = sample_mat4_cases().map(|value| blas_mat4_with(value, make_scalar));
    let rhs4_cases = sample_mat4_b_cases().map(|value| blas_mat4_with(value, make_scalar));
    let scalar_cases = [
        make_scalar(2.0),
        make_scalar(1.0e-9),
        make_scalar(-1.0e9),
        make_scalar(std::f64::consts::PI),
    ];
    let signal = abort_signal();

    group.bench_function(format!("{label}/mat3 new"), |b| {
        let raw_cases = sample_mat3_cases();
        let cursor = Cell::new(0);
        b.iter(|| black_box(blas_mat3_with(*next_case(&raw_cases, &cursor), make_scalar)))
    });
    group.bench_function(format!("{label}/mat3 zero"), |b| {
        b.iter(|| black_box(Matrix3::<B>::zero()))
    });
    group.bench_function(format!("{label}/mat3 identity"), |b| {
        b.iter(|| black_box(Matrix3::<B>::identity()))
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
        b.iter(|| black_box(Matrix4::<B>::zero()))
    });
    group.bench_function(format!("{label}/mat4 identity"), |b| {
        b.iter(|| black_box(Matrix4::<B>::identity()))
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
    group.bench_function(format!("{label}/mat4 bitxor"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| black_box((black_box(next_case(&lhs4_cases, &cursor).clone()) ^ 3).unwrap()))
    });
}

fn bench_matrix_operations(c: &mut Criterion) {
    let mut group = c.benchmark_group("matrix_ops");
    bench_matrix_operations_for::<ApproxBackend, _>(&mut group, "approx", s::<ApproxBackend>);
    bench_matrix_operations_for::<RealisticBackend, _>(
        &mut group,
        "realistic",
        s::<RealisticBackend>,
    );
    bench_matrix_operations_for::<RealisticBackend, _>(&mut group, "realistic-rational", qr);
    bench_astro_matrix_operations(&mut group, "astro128");
    bench_arp_matrix_operations(&mut group, "arp128");
    group.finish();
}

fn bench_astro_matrix_operations(
    group: &mut BenchmarkGroup<'_, criterion::measurement::WallTime>,
    label: &str,
) {
    let ctx = astro_backend::Ctx::new(128);
    let lhs3_cases = sample_mat3_cases().map(|value| astro_backend::Mat3::new(&ctx, value.m));
    let rhs3_cases = sample_mat3_b_cases().map(|value| astro_backend::Mat3::new(&ctx, value.m));
    let lhs4_cases = sample_mat4_cases().map(|value| astro_backend::Mat4::new(&ctx, value.m));
    let rhs4_cases = sample_mat4_b_cases().map(|value| astro_backend::Mat4::new(&ctx, value.m));
    let scalar_cases = [2.0, 1.0e-9, -1.0e9, std::f64::consts::PI].map(|value| ctx.f(value));

    group.bench_function(format!("{label}/mat3 new"), |b| {
        let raw_cases = sample_mat3_cases();
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(astro_backend::Mat3::new(
                &ctx,
                next_case(&raw_cases, &cursor).m,
            ))
        })
    });
    group.bench_function(format!("{label}/mat3 zero"), |b| {
        b.iter(|| black_box(astro_backend::Mat3::zero(&ctx)))
    });
    group.bench_function(format!("{label}/mat3 identity"), |b| {
        b.iter(|| black_box(astro_backend::Mat3::identity(&ctx)))
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
                    astro_backend::Ctx::div,
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
                        lhs3_cases[index].combine(&rhs3_cases[index], &ctx, astro_backend::Ctx::add)
                    }
                    "add_scalar" => lhs3_cases[index].map_scalar(
                        &scalar_cases[index],
                        &ctx,
                        astro_backend::Ctx::add,
                    ),
                    "sub" => {
                        lhs3_cases[index].combine(&rhs3_cases[index], &ctx, astro_backend::Ctx::sub)
                    }
                    "sub_scalar" => lhs3_cases[index].map_scalar(
                        &scalar_cases[index],
                        &ctx,
                        astro_backend::Ctx::sub,
                    ),
                    "neg" => lhs3_cases[index].neg(&ctx),
                    _ => lhs3_cases[index].map_scalar(
                        &scalar_cases[index],
                        &ctx,
                        astro_backend::Ctx::mul,
                    ),
                })
            })
        });
    }

    group.bench_function(format!("{label}/mat4 zero"), |b| {
        b.iter(|| black_box(astro_backend::Mat4::zero(&ctx)))
    });
    group.bench_function(format!("{label}/mat4 identity"), |b| {
        b.iter(|| black_box(astro_backend::Mat4::identity(&ctx)))
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
                        astro_backend::Ctx::div,
                    ),
                    "add" => {
                        lhs4_cases[index].combine(&rhs4_cases[index], &ctx, astro_backend::Ctx::add)
                    }
                    "add_scalar" => lhs4_cases[index].map_scalar(
                        &scalar_cases[index],
                        &ctx,
                        astro_backend::Ctx::add,
                    ),
                    "sub" => {
                        lhs4_cases[index].combine(&rhs4_cases[index], &ctx, astro_backend::Ctx::sub)
                    }
                    "sub_scalar" => lhs4_cases[index].map_scalar(
                        &scalar_cases[index],
                        &ctx,
                        astro_backend::Ctx::sub,
                    ),
                    "neg" => lhs4_cases[index].neg(&ctx),
                    "mul_scalar" => lhs4_cases[index].map_scalar(
                        &scalar_cases[index],
                        &ctx,
                        astro_backend::Ctx::mul,
                    ),
                    _ => lhs4_cases[index].div_matrix(&rhs4_cases[index], &ctx),
                })
            })
        });
    }
}

fn bench_arp_matrix_operations(
    group: &mut BenchmarkGroup<'_, criterion::measurement::WallTime>,
    label: &str,
) {
    let ctx = arp_backend::Ctx::new(128);
    let lhs3_cases = sample_mat3_cases().map(|value| arp_backend::Mat3::new(&ctx, value.m));
    let rhs3_cases = sample_mat3_b_cases().map(|value| arp_backend::Mat3::new(&ctx, value.m));
    let lhs4_cases = sample_mat4_cases().map(|value| arp_backend::Mat4::new(&ctx, value.m));
    let rhs4_cases = sample_mat4_b_cases().map(|value| arp_backend::Mat4::new(&ctx, value.m));
    let scalar_cases = [2.0, 1.0e-9, -1.0e9, std::f64::consts::PI].map(|value| ctx.f(value));

    group.bench_function(format!("{label}/mat3 new"), |b| {
        let raw_cases = sample_mat3_cases();
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(arp_backend::Mat3::new(
                &ctx,
                next_case(&raw_cases, &cursor).m,
            ))
        })
    });
    group.bench_function(format!("{label}/mat3 zero"), |b| {
        b.iter(|| black_box(arp_backend::Mat3::zero(&ctx)))
    });
    group.bench_function(format!("{label}/mat3 identity"), |b| {
        b.iter(|| black_box(arp_backend::Mat3::identity(&ctx)))
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
                    arp_backend::Ctx::div,
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
                        lhs3_cases[index].combine(&rhs3_cases[index], &ctx, arp_backend::Ctx::add)
                    }
                    "add_scalar" => lhs3_cases[index].map_scalar(
                        &scalar_cases[index],
                        &ctx,
                        arp_backend::Ctx::add,
                    ),
                    "sub" => {
                        lhs3_cases[index].combine(&rhs3_cases[index], &ctx, arp_backend::Ctx::sub)
                    }
                    "sub_scalar" => lhs3_cases[index].map_scalar(
                        &scalar_cases[index],
                        &ctx,
                        arp_backend::Ctx::sub,
                    ),
                    "neg" => lhs3_cases[index].neg(&ctx),
                    _ => lhs3_cases[index].map_scalar(
                        &scalar_cases[index],
                        &ctx,
                        arp_backend::Ctx::mul,
                    ),
                })
            })
        });
    }

    group.bench_function(format!("{label}/mat4 zero"), |b| {
        b.iter(|| black_box(arp_backend::Mat4::zero(&ctx)))
    });
    group.bench_function(format!("{label}/mat4 identity"), |b| {
        b.iter(|| black_box(arp_backend::Mat4::identity(&ctx)))
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
                        arp_backend::Ctx::div,
                    ),
                    "add" => {
                        lhs4_cases[index].combine(&rhs4_cases[index], &ctx, arp_backend::Ctx::add)
                    }
                    "add_scalar" => lhs4_cases[index].map_scalar(
                        &scalar_cases[index],
                        &ctx,
                        arp_backend::Ctx::add,
                    ),
                    "sub" => {
                        lhs4_cases[index].combine(&rhs4_cases[index], &ctx, arp_backend::Ctx::sub)
                    }
                    "sub_scalar" => lhs4_cases[index].map_scalar(
                        &scalar_cases[index],
                        &ctx,
                        arp_backend::Ctx::sub,
                    ),
                    "neg" => lhs4_cases[index].neg(&ctx),
                    "mul_scalar" => lhs4_cases[index].map_scalar(
                        &scalar_cases[index],
                        &ctx,
                        arp_backend::Ctx::mul,
                    ),
                    _ => lhs4_cases[index].div_matrix(&rhs4_cases[index], &ctx),
                })
            })
        });
    }
}

