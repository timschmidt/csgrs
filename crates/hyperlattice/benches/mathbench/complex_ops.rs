fn bench_complex_operations_for<B, F>(
    group: &mut BenchmarkGroup<'_, criterion::measurement::WallTime>,
    label: &str,
    make_scalar: F,
) where
    B: Backend,
    F: Copy + Fn(f64) -> Scalar<B>,
{
    let lhs_cases = [
        Complex::new(make_scalar(3.0), make_scalar(4.0)),
        Complex::new(make_scalar(1.0e-9), make_scalar(-1.0e-9)),
        Complex::new(make_scalar(1.0e9), make_scalar(-1.0)),
        Complex::new(
            make_scalar(std::f64::consts::PI),
            make_scalar(-std::f64::consts::E),
        ),
    ];
    let rhs_cases = [
        Complex::new(make_scalar(1.5), make_scalar(-2.0)),
        Complex::new(make_scalar(-1.0e-9), make_scalar(2.0e-9)),
        Complex::new(make_scalar(-1.0e9), make_scalar(2.0)),
        Complex::new(
            make_scalar(std::f64::consts::SQRT_2),
            make_scalar(std::f64::consts::FRAC_1_PI),
        ),
    ];
    let real_cases = [
        make_scalar(2.0),
        make_scalar(1.0e-9),
        make_scalar(-1.0e9),
        make_scalar(std::f64::consts::PI),
    ];

    group.bench_function(format!("{label}/zero"), |b| {
        b.iter(|| black_box(Complex::<B>::zero()))
    });
    group.bench_function(format!("{label}/one"), |b| {
        b.iter(|| black_box(Complex::<B>::one()))
    });
    group.bench_function(format!("{label}/i"), |b| {
        b.iter(|| black_box(Complex::<B>::i()))
    });
    group.bench_function(format!("{label}/free_i"), |b| {
        b.iter(|| black_box(Complex::<B>::i()))
    });
    group.bench_function(format!("{label}/conjugate"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| black_box(black_box(next_case(&lhs_cases, &cursor).clone()).conjugate()))
    });
    group.bench_function(format!("{label}/norm_squared"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| black_box(black_box(next_case(&lhs_cases, &cursor)).norm_squared()))
    });
    group.bench_function(format!("{label}/reciprocal"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                black_box(next_case(&lhs_cases, &cursor).clone())
                    .reciprocal()
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/reciprocal_checked"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                black_box(next_case(&lhs_cases, &cursor).clone())
                    .reciprocal_checked()
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/powi"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                black_box(next_case(&lhs_cases, &cursor).clone())
                    .powi(5)
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/powi_checked"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                black_box(next_case(&lhs_cases, &cursor).clone())
                    .powi_checked(5)
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/div_checked"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % lhs_cases.len());
            black_box(
                black_box(lhs_cases[index].clone())
                    .div_checked(black_box(rhs_cases[index].clone()))
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/div_real_checked"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % lhs_cases.len());
            black_box(
                black_box(lhs_cases[index].clone())
                    .div_real_checked(black_box(real_cases[index].clone()))
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/from_scalar"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(Complex::from(black_box(
                next_case(&real_cases, &cursor).clone(),
            )))
        })
    });
    group.bench_function(format!("{label}/add"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % lhs_cases.len());
            black_box(black_box(lhs_cases[index].clone()) + black_box(rhs_cases[index].clone()))
        })
    });
    group.bench_function(format!("{label}/sub"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % lhs_cases.len());
            black_box(black_box(lhs_cases[index].clone()) - black_box(rhs_cases[index].clone()))
        })
    });
    group.bench_function(format!("{label}/neg"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| black_box(-black_box(next_case(&lhs_cases, &cursor).clone())))
    });
    group.bench_function(format!("{label}/mul"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % lhs_cases.len());
            black_box(black_box(lhs_cases[index].clone()) * black_box(rhs_cases[index].clone()))
        })
    });
    group.bench_function(format!("{label}/div"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % lhs_cases.len());
            black_box(
                (black_box(lhs_cases[index].clone()) / black_box(rhs_cases[index].clone()))
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/div_real"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % lhs_cases.len());
            black_box(
                (black_box(lhs_cases[index].clone()) / black_box(real_cases[index].clone()))
                    .unwrap(),
            )
        })
    });
}

fn bench_complex_operations(c: &mut Criterion) {
    let mut group = c.benchmark_group("complex_ops");
    bench_complex_operations_for::<ApproxBackend, _>(&mut group, "approx", s::<ApproxBackend>);
    bench_complex_operations_for::<HyperrealBackend, _>(
        &mut group,
        "realistic",
        s::<HyperrealBackend>,
    );
    bench_complex_operations_for::<HyperrealBackend, _>(&mut group, "realistic-rational", qr);
    bench_astro_complex_operations(&mut group, "astro128");
    bench_arp_complex_operations(&mut group, "arp128");
    group.finish();
}

fn bench_astro_complex_operations(
    group: &mut BenchmarkGroup<'_, criterion::measurement::WallTime>,
    label: &str,
) {
    let ctx = astro_backend::Ctx::new(128);
    let lhs_cases = [
        astro_backend::Complex::new(&ctx, 3.0, 4.0),
        astro_backend::Complex::new(&ctx, 1.0e-9, -1.0e-9),
        astro_backend::Complex::new(&ctx, 1.0e9, -1.0),
        astro_backend::Complex::new(&ctx, std::f64::consts::PI, -std::f64::consts::E),
    ];
    let rhs_cases = [
        astro_backend::Complex::new(&ctx, 1.5, -2.0),
        astro_backend::Complex::new(&ctx, -1.0e-9, 2.0e-9),
        astro_backend::Complex::new(&ctx, -1.0e9, 2.0),
        astro_backend::Complex::new(&ctx, std::f64::consts::SQRT_2, std::f64::consts::FRAC_1_PI),
    ];
    let real_cases = [2.0, 1.0e-9, -1.0e9, std::f64::consts::PI].map(|value| ctx.f(value));

    group.bench_function(format!("{label}/zero"), |b| {
        b.iter(|| black_box(astro_backend::Complex::zero(&ctx)))
    });
    group.bench_function(format!("{label}/one"), |b| {
        b.iter(|| black_box(astro_backend::Complex::one(&ctx)))
    });
    group.bench_function(format!("{label}/i"), |b| {
        b.iter(|| black_box(astro_backend::Complex::i(&ctx)))
    });
    group.bench_function(format!("{label}/free_i"), |b| {
        b.iter(|| black_box(astro_backend::Complex::i(&ctx)))
    });
    group.bench_function(format!("{label}/conjugate"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| black_box(next_case(&lhs_cases, &cursor).conjugate(&ctx)))
    });
    group.bench_function(format!("{label}/norm_squared"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| black_box(next_case(&lhs_cases, &cursor).norm_squared(&ctx)))
    });
    for name in ["reciprocal", "reciprocal_checked"] {
        group.bench_function(format!("{label}/{name}"), |b| {
            let cursor = Cell::new(0);
            b.iter(|| black_box(next_case(&lhs_cases, &cursor).reciprocal(&ctx)))
        });
    }
    for name in ["powi", "powi_checked"] {
        group.bench_function(format!("{label}/{name}"), |b| {
            let cursor = Cell::new(0);
            b.iter(|| black_box(next_case(&lhs_cases, &cursor).powi(5, &ctx)))
        });
    }
    group.bench_function(format!("{label}/div_checked"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % lhs_cases.len());
            black_box(lhs_cases[index].div(&rhs_cases[index], &ctx))
        })
    });
    group.bench_function(format!("{label}/div_real_checked"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % lhs_cases.len());
            black_box(lhs_cases[index].div_real(&real_cases[index], &ctx))
        })
    });
    group.bench_function(format!("{label}/from_scalar"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(astro_backend::Complex::from_scalar(
                next_case(&real_cases, &cursor),
                &ctx,
            ))
        })
    });
    group.bench_function(format!("{label}/add"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % lhs_cases.len());
            black_box(lhs_cases[index].add(&rhs_cases[index], &ctx))
        })
    });
    group.bench_function(format!("{label}/sub"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % lhs_cases.len());
            black_box(lhs_cases[index].sub(&rhs_cases[index], &ctx))
        })
    });
    group.bench_function(format!("{label}/neg"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| black_box(next_case(&lhs_cases, &cursor).neg(&ctx)))
    });
    group.bench_function(format!("{label}/mul"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % lhs_cases.len());
            black_box(lhs_cases[index].mul(&rhs_cases[index], &ctx))
        })
    });
    group.bench_function(format!("{label}/div"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % lhs_cases.len());
            black_box(lhs_cases[index].div(&rhs_cases[index], &ctx))
        })
    });
    group.bench_function(format!("{label}/div_real"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % lhs_cases.len());
            black_box(lhs_cases[index].div_real(&real_cases[index], &ctx))
        })
    });
}

fn bench_arp_complex_operations(
    group: &mut BenchmarkGroup<'_, criterion::measurement::WallTime>,
    label: &str,
) {
    let ctx = arp_backend::Ctx::new(128);
    let lhs_cases = [
        arp_backend::Complex::new(&ctx, 3.0, 4.0),
        arp_backend::Complex::new(&ctx, 1.0e-9, -1.0e-9),
        arp_backend::Complex::new(&ctx, 1.0e9, -1.0),
        arp_backend::Complex::new(&ctx, std::f64::consts::PI, -std::f64::consts::E),
    ];
    let rhs_cases = [
        arp_backend::Complex::new(&ctx, 1.5, -2.0),
        arp_backend::Complex::new(&ctx, -1.0e-9, 2.0e-9),
        arp_backend::Complex::new(&ctx, -1.0e9, 2.0),
        arp_backend::Complex::new(&ctx, std::f64::consts::SQRT_2, std::f64::consts::FRAC_1_PI),
    ];
    let real_cases = [2.0, 1.0e-9, -1.0e9, std::f64::consts::PI].map(|value| ctx.f(value));

    group.bench_function(format!("{label}/zero"), |b| {
        b.iter(|| black_box(arp_backend::Complex::zero(&ctx)))
    });
    group.bench_function(format!("{label}/one"), |b| {
        b.iter(|| black_box(arp_backend::Complex::one(&ctx)))
    });
    group.bench_function(format!("{label}/i"), |b| {
        b.iter(|| black_box(arp_backend::Complex::i(&ctx)))
    });
    group.bench_function(format!("{label}/free_i"), |b| {
        b.iter(|| black_box(arp_backend::Complex::i(&ctx)))
    });
    group.bench_function(format!("{label}/conjugate"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| black_box(next_case(&lhs_cases, &cursor).conjugate(&ctx)))
    });
    group.bench_function(format!("{label}/norm_squared"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| black_box(next_case(&lhs_cases, &cursor).norm_squared(&ctx)))
    });
    for name in ["reciprocal", "reciprocal_checked"] {
        group.bench_function(format!("{label}/{name}"), |b| {
            let cursor = Cell::new(0);
            b.iter(|| black_box(next_case(&lhs_cases, &cursor).reciprocal(&ctx)))
        });
    }
    for name in ["powi", "powi_checked"] {
        group.bench_function(format!("{label}/{name}"), |b| {
            let cursor = Cell::new(0);
            b.iter(|| black_box(next_case(&lhs_cases, &cursor).powi(5, &ctx)))
        });
    }
    group.bench_function(format!("{label}/div_checked"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % lhs_cases.len());
            black_box(lhs_cases[index].div(&rhs_cases[index], &ctx))
        })
    });
    group.bench_function(format!("{label}/div_real_checked"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % lhs_cases.len());
            black_box(lhs_cases[index].div_real(&real_cases[index], &ctx))
        })
    });
    group.bench_function(format!("{label}/from_scalar"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(arp_backend::Complex::from_scalar(
                next_case(&real_cases, &cursor),
                &ctx,
            ))
        })
    });
    group.bench_function(format!("{label}/add"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % lhs_cases.len());
            black_box(lhs_cases[index].add(&rhs_cases[index], &ctx))
        })
    });
    group.bench_function(format!("{label}/sub"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % lhs_cases.len());
            black_box(lhs_cases[index].sub(&rhs_cases[index], &ctx))
        })
    });
    group.bench_function(format!("{label}/neg"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| black_box(next_case(&lhs_cases, &cursor).neg(&ctx)))
    });
    group.bench_function(format!("{label}/mul"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % lhs_cases.len());
            black_box(lhs_cases[index].mul(&rhs_cases[index], &ctx))
        })
    });
    group.bench_function(format!("{label}/div"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % lhs_cases.len());
            black_box(lhs_cases[index].div(&rhs_cases[index], &ctx))
        })
    });
    group.bench_function(format!("{label}/div_real"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % lhs_cases.len());
            black_box(lhs_cases[index].div_real(&real_cases[index], &ctx))
        })
    });
}
