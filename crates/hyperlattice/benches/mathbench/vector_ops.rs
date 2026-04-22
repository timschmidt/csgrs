fn bench_vector_operations_for<B, F>(
    group: &mut BenchmarkGroup<'_, criterion::measurement::WallTime>,
    label: &str,
    make_scalar: F,
) where
    B: Backend,
    F: Copy + Fn(f64) -> Scalar<B>,
{
    let lhs3_cases = sample_vec3_cases().map(|value| blas_vec3_with(value, make_scalar));
    let rhs3_cases = sample_vec3_b_cases().map(|value| blas_vec3_with(value, make_scalar));
    let lhs4_cases = sample_vec4_cases().map(|value| blas_vec4_with(value, make_scalar));
    let rhs4_cases = sample_vec4_b_cases().map(|value| blas_vec4_with(value, make_scalar));
    let scalar_cases = [
        make_scalar(2.0),
        make_scalar(1.0e-9),
        make_scalar(-1.0e9),
        make_scalar(std::f64::consts::PI),
    ];
    let signal = abort_signal();

    group.bench_function(format!("{label}/vec3 new"), |b| {
        let raw_cases = sample_vec3_cases();
        let cursor = Cell::new(0);
        b.iter(|| black_box(blas_vec3_with(*next_case(&raw_cases, &cursor), make_scalar)))
    });
    group.bench_function(format!("{label}/vec3 zero"), |b| {
        b.iter(|| black_box(Vector3::<B>::zero()))
    });
    group.bench_function(format!("{label}/vec3 dot_abort"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % lhs3_cases.len());
            black_box(
                black_box(&lhs3_cases[index])
                    .dot_with_abort(black_box(&rhs3_cases[index]), &signal),
            )
        })
    });
    group.bench_function(format!("{label}/vec3 magnitude_abort"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                black_box(next_case(&lhs3_cases, &cursor))
                    .magnitude_with_abort(&signal)
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/vec3 normalize_checked"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                black_box(next_case(&lhs3_cases, &cursor))
                    .normalize_checked()
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/vec3 normalize_checked_abort"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                black_box(next_case(&lhs3_cases, &cursor))
                    .normalize_checked_with_abort(&signal)
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/vec3 div_scalar_checked"), |b| {
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
    group.bench_function(format!("{label}/vec3 div_scalar_checked_abort"), |b| {
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
    group.bench_function(format!("{label}/vec3 add"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % lhs3_cases.len());
            black_box(black_box(lhs3_cases[index].clone()) + black_box(rhs3_cases[index].clone()))
        })
    });
    group.bench_function(format!("{label}/vec3 add_scalar"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % lhs3_cases.len());
            black_box(black_box(lhs3_cases[index].clone()) + black_box(scalar_cases[index].clone()))
        })
    });
    group.bench_function(format!("{label}/vec3 sub"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % lhs3_cases.len());
            black_box(black_box(lhs3_cases[index].clone()) - black_box(rhs3_cases[index].clone()))
        })
    });
    group.bench_function(format!("{label}/vec3 sub_scalar"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % lhs3_cases.len());
            black_box(black_box(lhs3_cases[index].clone()) - black_box(scalar_cases[index].clone()))
        })
    });
    group.bench_function(format!("{label}/vec3 neg"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| black_box(-black_box(next_case(&lhs3_cases, &cursor).clone())))
    });
    group.bench_function(format!("{label}/vec3 mul_scalar"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % lhs3_cases.len());
            black_box(black_box(lhs3_cases[index].clone()) * black_box(scalar_cases[index].clone()))
        })
    });
    group.bench_function(format!("{label}/vec3 div_scalar"), |b| {
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

    group.bench_function(format!("{label}/vec4 dot"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % lhs4_cases.len());
            black_box(black_box(&lhs4_cases[index]).dot(black_box(&rhs4_cases[index])))
        })
    });
    group.bench_function(format!("{label}/vec4 magnitude"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                black_box(next_case(&lhs4_cases, &cursor))
                    .magnitude()
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/vec4 normalize"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                black_box(next_case(&lhs4_cases, &cursor))
                    .normalize()
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/vec4 add"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % lhs4_cases.len());
            black_box(black_box(lhs4_cases[index].clone()) + black_box(rhs4_cases[index].clone()))
        })
    });
    group.bench_function(format!("{label}/vec4 add_scalar"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % lhs4_cases.len());
            black_box(black_box(lhs4_cases[index].clone()) + black_box(scalar_cases[index].clone()))
        })
    });
    group.bench_function(format!("{label}/vec4 sub"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % lhs4_cases.len());
            black_box(black_box(lhs4_cases[index].clone()) - black_box(rhs4_cases[index].clone()))
        })
    });
    group.bench_function(format!("{label}/vec4 sub_scalar"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % lhs4_cases.len());
            black_box(black_box(lhs4_cases[index].clone()) - black_box(scalar_cases[index].clone()))
        })
    });
    group.bench_function(format!("{label}/vec4 neg"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| black_box(-black_box(next_case(&lhs4_cases, &cursor).clone())))
    });
    group.bench_function(format!("{label}/vec4 mul_scalar"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % lhs4_cases.len());
            black_box(black_box(lhs4_cases[index].clone()) * black_box(scalar_cases[index].clone()))
        })
    });
    group.bench_function(format!("{label}/vec4 div_scalar"), |b| {
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
}

fn bench_vector_operations(c: &mut Criterion) {
    let mut group = c.benchmark_group("vector_ops");
    bench_vector_operations_for::<ApproxBackend, _>(&mut group, "approx", s::<ApproxBackend>);
    bench_vector_operations_for::<RealisticBackend, _>(
        &mut group,
        "realistic",
        s::<RealisticBackend>,
    );
    bench_vector_operations_for::<RealisticBackend, _>(&mut group, "realistic-rational", qr);
    bench_astro_vector_operations(&mut group, "astro128");
    bench_arp_vector_operations(&mut group, "arp128");
    group.finish();
}

fn bench_astro_vector_operations(
    group: &mut BenchmarkGroup<'_, criterion::measurement::WallTime>,
    label: &str,
) {
    let ctx = astro_backend::Ctx::new(128);
    let lhs3_cases =
        sample_vec3_cases().map(|value| astro_backend::Vec3::new(&ctx, value.x, value.y, value.z));
    let rhs3_cases = sample_vec3_b_cases()
        .map(|value| astro_backend::Vec3::new(&ctx, value.x, value.y, value.z));
    let lhs4_cases = sample_vec4_cases()
        .map(|value| astro_backend::Vec4::new(&ctx, value.x, value.y, value.z, value.w));
    let rhs4_cases = sample_vec4_b_cases()
        .map(|value| astro_backend::Vec4::new(&ctx, value.x, value.y, value.z, value.w));
    let scalar_cases = [2.0, 1.0e-9, -1.0e9, std::f64::consts::PI].map(|value| ctx.f(value));

    group.bench_function(format!("{label}/vec3 new"), |b| {
        let raw_cases = sample_vec3_cases();
        let cursor = Cell::new(0);
        b.iter(|| {
            let value = *next_case(&raw_cases, &cursor);
            black_box(astro_backend::Vec3::new(&ctx, value.x, value.y, value.z))
        })
    });
    group.bench_function(format!("{label}/vec3 zero"), |b| {
        b.iter(|| black_box(astro_backend::Vec3::zero(&ctx)))
    });
    group.bench_function(format!("{label}/vec3 dot_abort"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % lhs3_cases.len());
            black_box(lhs3_cases[index].dot(&rhs3_cases[index], &ctx))
        })
    });
    for name in [
        "vec3 magnitude_abort",
        "vec3 normalize_checked",
        "vec3 normalize_checked_abort",
    ] {
        group.bench_function(format!("{label}/{name}"), |b| {
            let cursor = Cell::new(0);
            b.iter(|| {
                let value = next_case(&lhs3_cases, &cursor);
                black_box(if name == "vec3 magnitude_abort" {
                    let magnitude = value.magnitude(&ctx);
                    astro_backend::Vec3 {
                        x: magnitude,
                        y: ctx.zero(),
                        z: ctx.zero(),
                    }
                } else {
                    value.normalize(&ctx)
                })
            })
        });
    }
    for name in ["vec3 div_scalar_checked", "vec3 div_scalar_checked_abort"] {
        group.bench_function(format!("{label}/{name}"), |b| {
            let cursor = Cell::new(0);
            b.iter(|| {
                let index = cursor.get();
                cursor.set((index + 1) % lhs3_cases.len());
                black_box(lhs3_cases[index].div_scalar(&scalar_cases[index], &ctx))
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
        "div_scalar",
    ] {
        group.bench_function(format!("{label}/vec3 {name}"), |b| {
            let cursor = Cell::new(0);
            b.iter(|| {
                let index = cursor.get();
                cursor.set((index + 1) % lhs3_cases.len());
                black_box(match name {
                    "add" => lhs3_cases[index].add(&rhs3_cases[index], &ctx),
                    "add_scalar" => lhs3_cases[index].add_scalar(&scalar_cases[index], &ctx),
                    "sub" => lhs3_cases[index].sub(&rhs3_cases[index], &ctx),
                    "sub_scalar" => lhs3_cases[index].sub_scalar(&scalar_cases[index], &ctx),
                    "neg" => lhs3_cases[index].neg(&ctx),
                    "mul_scalar" => lhs3_cases[index].mul_scalar(&scalar_cases[index], &ctx),
                    _ => lhs3_cases[index].div_scalar(&scalar_cases[index], &ctx),
                })
            })
        });
    }
    for name in [
        "dot",
        "magnitude",
        "normalize",
        "add",
        "add_scalar",
        "sub",
        "sub_scalar",
        "neg",
        "mul_scalar",
        "div_scalar",
    ] {
        group.bench_function(format!("{label}/vec4 {name}"), |b| {
            let cursor = Cell::new(0);
            b.iter(|| {
                let index = cursor.get();
                cursor.set((index + 1) % lhs4_cases.len());
                black_box(match name {
                    "dot" => {
                        let dot = lhs4_cases[index].dot(&rhs4_cases[index], &ctx);
                        astro_backend::Vec4 {
                            x: dot,
                            y: ctx.zero(),
                            z: ctx.zero(),
                            w: ctx.zero(),
                        }
                    }
                    "magnitude" => {
                        let magnitude = lhs4_cases[index].magnitude(&ctx);
                        astro_backend::Vec4 {
                            x: magnitude,
                            y: ctx.zero(),
                            z: ctx.zero(),
                            w: ctx.zero(),
                        }
                    }
                    "normalize" => lhs4_cases[index].normalize(&ctx),
                    "add" => lhs4_cases[index].add(&rhs4_cases[index], &ctx),
                    "add_scalar" => lhs4_cases[index].add_scalar(&scalar_cases[index], &ctx),
                    "sub" => lhs4_cases[index].sub(&rhs4_cases[index], &ctx),
                    "sub_scalar" => lhs4_cases[index].sub_scalar(&scalar_cases[index], &ctx),
                    "neg" => lhs4_cases[index].neg(&ctx),
                    "mul_scalar" => lhs4_cases[index].mul_scalar(&scalar_cases[index], &ctx),
                    _ => lhs4_cases[index].div_scalar(&scalar_cases[index], &ctx),
                })
            })
        });
    }
}

fn bench_arp_vector_operations(
    group: &mut BenchmarkGroup<'_, criterion::measurement::WallTime>,
    label: &str,
) {
    let ctx = arp_backend::Ctx::new(128);
    let lhs3_cases =
        sample_vec3_cases().map(|value| arp_backend::Vec3::new(&ctx, value.x, value.y, value.z));
    let rhs3_cases =
        sample_vec3_b_cases().map(|value| arp_backend::Vec3::new(&ctx, value.x, value.y, value.z));
    let lhs4_cases = sample_vec4_cases()
        .map(|value| arp_backend::Vec4::new(&ctx, value.x, value.y, value.z, value.w));
    let rhs4_cases = sample_vec4_b_cases()
        .map(|value| arp_backend::Vec4::new(&ctx, value.x, value.y, value.z, value.w));
    let scalar_cases = [2.0, 1.0e-9, -1.0e9, std::f64::consts::PI].map(|value| ctx.f(value));

    group.bench_function(format!("{label}/vec3 new"), |b| {
        let raw_cases = sample_vec3_cases();
        let cursor = Cell::new(0);
        b.iter(|| {
            let value = *next_case(&raw_cases, &cursor);
            black_box(arp_backend::Vec3::new(&ctx, value.x, value.y, value.z))
        })
    });
    group.bench_function(format!("{label}/vec3 zero"), |b| {
        b.iter(|| black_box(arp_backend::Vec3::zero(&ctx)))
    });
    group.bench_function(format!("{label}/vec3 dot_abort"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % lhs3_cases.len());
            black_box(lhs3_cases[index].dot(&rhs3_cases[index], &ctx))
        })
    });
    for name in [
        "vec3 magnitude_abort",
        "vec3 normalize_checked",
        "vec3 normalize_checked_abort",
    ] {
        group.bench_function(format!("{label}/{name}"), |b| {
            let cursor = Cell::new(0);
            b.iter(|| {
                let value = next_case(&lhs3_cases, &cursor);
                black_box(if name == "vec3 magnitude_abort" {
                    let magnitude = value.magnitude(&ctx);
                    arp_backend::Vec3 {
                        x: magnitude,
                        y: ctx.zero(),
                        z: ctx.zero(),
                    }
                } else {
                    value.normalize(&ctx)
                })
            })
        });
    }
    for name in ["vec3 div_scalar_checked", "vec3 div_scalar_checked_abort"] {
        group.bench_function(format!("{label}/{name}"), |b| {
            let cursor = Cell::new(0);
            b.iter(|| {
                let index = cursor.get();
                cursor.set((index + 1) % lhs3_cases.len());
                black_box(lhs3_cases[index].div_scalar(&scalar_cases[index], &ctx))
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
        "div_scalar",
    ] {
        group.bench_function(format!("{label}/vec3 {name}"), |b| {
            let cursor = Cell::new(0);
            b.iter(|| {
                let index = cursor.get();
                cursor.set((index + 1) % lhs3_cases.len());
                black_box(match name {
                    "add" => lhs3_cases[index].add(&rhs3_cases[index], &ctx),
                    "add_scalar" => lhs3_cases[index].add_scalar(&scalar_cases[index], &ctx),
                    "sub" => lhs3_cases[index].sub(&rhs3_cases[index], &ctx),
                    "sub_scalar" => lhs3_cases[index].sub_scalar(&scalar_cases[index], &ctx),
                    "neg" => lhs3_cases[index].neg(&ctx),
                    "mul_scalar" => lhs3_cases[index].mul_scalar(&scalar_cases[index], &ctx),
                    _ => lhs3_cases[index].div_scalar(&scalar_cases[index], &ctx),
                })
            })
        });
    }
    for name in [
        "dot",
        "magnitude",
        "normalize",
        "add",
        "add_scalar",
        "sub",
        "sub_scalar",
        "neg",
        "mul_scalar",
        "div_scalar",
    ] {
        group.bench_function(format!("{label}/vec4 {name}"), |b| {
            let cursor = Cell::new(0);
            b.iter(|| {
                let index = cursor.get();
                cursor.set((index + 1) % lhs4_cases.len());
                black_box(match name {
                    "dot" => {
                        let dot = lhs4_cases[index].dot(&rhs4_cases[index], &ctx);
                        arp_backend::Vec4 {
                            x: dot,
                            y: ctx.zero(),
                            z: ctx.zero(),
                            w: ctx.zero(),
                        }
                    }
                    "magnitude" => {
                        let magnitude = lhs4_cases[index].magnitude(&ctx);
                        arp_backend::Vec4 {
                            x: magnitude,
                            y: ctx.zero(),
                            z: ctx.zero(),
                            w: ctx.zero(),
                        }
                    }
                    "normalize" => lhs4_cases[index].normalize(&ctx),
                    "add" => lhs4_cases[index].add(&rhs4_cases[index], &ctx),
                    "add_scalar" => lhs4_cases[index].add_scalar(&scalar_cases[index], &ctx),
                    "sub" => lhs4_cases[index].sub(&rhs4_cases[index], &ctx),
                    "sub_scalar" => lhs4_cases[index].sub_scalar(&scalar_cases[index], &ctx),
                    "neg" => lhs4_cases[index].neg(&ctx),
                    "mul_scalar" => lhs4_cases[index].mul_scalar(&scalar_cases[index], &ctx),
                    _ => lhs4_cases[index].div_scalar(&scalar_cases[index], &ctx),
                })
            })
        });
    }
}

