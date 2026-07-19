fn bench_complex_operations_for<F>(
    group: &mut BenchmarkGroup<'_, criterion::measurement::WallTime>,
    label: &str,
    make_scalar: F,
) where
    F: Copy + Fn(f64) -> Real,
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

    trace_dispatch_cases(format!("complex_ops/{label}/powi"), &lhs_cases, |value| {
        let _ = black_box(value.clone().powi(5).unwrap());
    });
    trace_dispatch_cases(
        format!("complex_ops/{label}/mul"),
        &[0_usize, 1, 2, 3],
        |index| {
            let _ = black_box(lhs_cases[*index].clone() * rhs_cases[*index].clone());
        },
    );
    trace_dispatch_cases(
        format!("complex_ops/{label}/powi_negative_one"),
        &lhs_cases,
        |value| {
            let _ = black_box(value.clone().powi(-1).unwrap());
        },
    );
    trace_dispatch_cases(
        format!("complex_ops/{label}/powi_checked"),
        &lhs_cases,
        |value| {
            let _ = black_box(value.clone().powi_checked(5).unwrap());
        },
    );
    trace_dispatch_cases(
        format!("complex_ops/{label}/powi_checked_negative_one"),
        &lhs_cases,
        |value| {
            let _ = black_box(value.clone().powi_checked(-1).unwrap());
        },
    );

    group.bench_function(format!("{label}/zero"), |b| {
        b.iter(|| black_box(Complex::zero()))
    });
    group.bench_function(format!("{label}/one"), |b| {
        b.iter(|| black_box(Complex::one()))
    });
    group.bench_function(format!("{label}/i"), |b| {
        b.iter(|| black_box(Complex::i()))
    });
    group.bench_function(format!("{label}/free_i"), |b| {
        b.iter(|| black_box(Complex::i()))
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
    group.bench_function(format!("{label}/powi_negative_one"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                black_box(next_case(&lhs_cases, &cursor).clone())
                    .powi(-1)
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
    group.bench_function(format!("{label}/powi_checked_negative_one"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                black_box(next_case(&lhs_cases, &cursor).clone())
                    .powi_checked(-1)
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
    bench_complex_operations_for::<_>(
        &mut group,
        "hyperreal",
        s,
    );
    bench_complex_operations_for::<_>(&mut group, "hyperreal-rational", qr);
    bench_numerica_complex_operations(&mut group, "numerica128");
    bench_symbolica_complex_operations(&mut group, "symbolica");
    group.finish();
}

fn varying_complex_mul_values(index: u64) -> [f64; 4] {
    let delta = index as f64 * f64::from_bits((1023_u64 - 40) << 52);
    [
        3.25 + delta,
        -2.125 + delta * 2.0,
        1.75 - delta,
        0.625 + delta * 3.0,
    ]
}

fn bench_cold_complex_mul_for<F>(
    group: &mut BenchmarkGroup<'_, criterion::measurement::WallTime>,
    label: &str,
    make_scalar: F,
) where
    F: Copy + Fn(f64) -> Real,
{
    group.bench_function(format!("{label}/varying"), |b| {
        let sequence = Cell::new(0_u64);
        b.iter_batched(
            || {
                let index = sequence.get();
                sequence.set(index + 1);
                let [ar, ai, br, bi] = varying_complex_mul_values(index);
                (
                    Complex::new(make_scalar(ar), make_scalar(ai)),
                    Complex::new(make_scalar(br), make_scalar(bi)),
                )
            },
            |(lhs, rhs)| black_box(black_box(lhs) * black_box(rhs)),
            BatchSize::SmallInput,
        )
    });
}

fn bench_complex_mul_cold(c: &mut Criterion) {
    let mut group = c.benchmark_group("complex_mul_cold");
    trace_dispatch_cases(
        "complex_mul_cold/hyperreal/varying",
        &[1_000_003_u64],
        |index| {
            let [ar, ai, br, bi] = varying_complex_mul_values(*index);
            let lhs = Complex::new(s(ar), s(ai));
            let rhs = Complex::new(s(br), s(bi));
            let _ = black_box(lhs * rhs);
        },
    );
    trace_dispatch_cases(
        "complex_mul_cold/hyperreal-rational/varying",
        &[1_000_003_u64],
        |index| {
            let [ar, ai, br, bi] = varying_complex_mul_values(*index);
            let lhs = Complex::new(qr(ar), qr(ai));
            let rhs = Complex::new(qr(br), qr(bi));
            let _ = black_box(lhs * rhs);
        },
    );
    bench_cold_complex_mul_for(&mut group, "hyperreal", s);
    bench_cold_complex_mul_for(&mut group, "hyperreal-rational", qr);

    let numerica_ctx = numerica_engine::Ctx::new(128);
    group.bench_function("numerica128/varying", |b| {
        let sequence = Cell::new(0_u64);
        b.iter_batched(
            || {
                let index = sequence.get();
                sequence.set(index + 1);
                let [ar, ai, br, bi] = varying_complex_mul_values(index);
                (
                    numerica_engine::Complex::new(&numerica_ctx, ar, ai),
                    numerica_engine::Complex::new(&numerica_ctx, br, bi),
                )
            },
            |(lhs, rhs)| black_box(lhs.mul(&rhs, &numerica_ctx)),
            BatchSize::SmallInput,
        )
    });

    let symbolica_ctx = symbolica_engine::Ctx::new(128);
    group.bench_function("symbolica/varying", |b| {
        let sequence = Cell::new(0_u64);
        b.iter_batched(
            || {
                let index = sequence.get();
                sequence.set(index + 1);
                let [ar, ai, br, bi] = varying_complex_mul_values(index);
                (
                    symbolica_engine::Complex::new(&symbolica_ctx, ar, ai),
                    symbolica_engine::Complex::new(&symbolica_ctx, br, bi),
                )
            },
            |(lhs, rhs)| black_box(lhs.mul(&rhs, &symbolica_ctx)),
            BatchSize::SmallInput,
        )
    });
    group.finish();
}

macro_rules! bench_external_complex_operations {
    ($engine:ident, $group:expr, $label:expr) => {{
        let ctx = $engine::Ctx::new(128);
        let lhs_cases = [
            $engine::Complex::new(&ctx, 3.0, 4.0),
            $engine::Complex::new(&ctx, 1.0e-9, -1.0e-9),
            $engine::Complex::new(&ctx, 1.0e9, -1.0),
            $engine::Complex::new(&ctx, std::f64::consts::PI, -std::f64::consts::E),
        ];
        let rhs_cases = [
            $engine::Complex::new(&ctx, 1.5, -2.0),
            $engine::Complex::new(&ctx, -1.0e-9, 2.0e-9),
            $engine::Complex::new(&ctx, -1.0e9, 2.0),
            $engine::Complex::new(&ctx, std::f64::consts::SQRT_2, std::f64::consts::FRAC_1_PI),
        ];
        let real_cases = [2.0, 1.0e-9, -1.0e9, std::f64::consts::PI].map(|value| ctx.f(value));

        $group.bench_function(format!("{}/zero", $label), |b| {
            b.iter(|| black_box($engine::Complex::zero(&ctx)))
        });
        $group.bench_function(format!("{}/one", $label), |b| {
            b.iter(|| black_box($engine::Complex::one(&ctx)))
        });
        $group.bench_function(format!("{}/i", $label), |b| {
            b.iter(|| black_box($engine::Complex::i(&ctx)))
        });
        $group.bench_function(format!("{}/free_i", $label), |b| {
            b.iter(|| black_box($engine::Complex::i(&ctx)))
        });
        $group.bench_function(format!("{}/conjugate", $label), |b| {
            let cursor = Cell::new(0);
            b.iter(|| black_box(next_case(&lhs_cases, &cursor).conjugate(&ctx)))
        });
        $group.bench_function(format!("{}/norm_squared", $label), |b| {
            let cursor = Cell::new(0);
            b.iter(|| black_box(next_case(&lhs_cases, &cursor).norm_squared(&ctx)))
        });
        for name in ["reciprocal", "reciprocal_checked"] {
            $group.bench_function(format!("{}/{}", $label, name), |b| {
                let cursor = Cell::new(0);
                b.iter(|| black_box(next_case(&lhs_cases, &cursor).reciprocal(&ctx)))
            });
        }
        for name in ["powi", "powi_checked"] {
            $group.bench_function(format!("{}/{}", $label, name), |b| {
                let cursor = Cell::new(0);
                b.iter(|| black_box(next_case(&lhs_cases, &cursor).powi(5, &ctx)))
            });
        }
        for name in ["powi_negative_one", "powi_checked_negative_one"] {
            $group.bench_function(format!("{}/{}", $label, name), |b| {
                let cursor = Cell::new(0);
                b.iter(|| black_box(next_case(&lhs_cases, &cursor).powi(1, &ctx).reciprocal(&ctx)))
            });
        }
        $group.bench_function(format!("{}/div_checked", $label), |b| {
            let cursor = Cell::new(0);
            b.iter(|| {
                let index = cursor.get();
                cursor.set((index + 1) % lhs_cases.len());
                black_box(lhs_cases[index].div(&rhs_cases[index], &ctx))
            })
        });
        $group.bench_function(format!("{}/div_real_checked", $label), |b| {
            let cursor = Cell::new(0);
            b.iter(|| {
                let index = cursor.get();
                cursor.set((index + 1) % lhs_cases.len());
                black_box(lhs_cases[index].div_real(&real_cases[index], &ctx))
            })
        });
        $group.bench_function(format!("{}/from_scalar", $label), |b| {
            let cursor = Cell::new(0);
            b.iter(|| {
                black_box($engine::Complex::from_scalar(
                    black_box(next_case(&real_cases, &cursor)),
                    &ctx,
                ))
            })
        });
        for name in ["add", "sub", "mul", "div"] {
            $group.bench_function(format!("{}/{}", $label, name), |b| {
                let cursor = Cell::new(0);
                b.iter(|| {
                    let index = cursor.get();
                    cursor.set((index + 1) % lhs_cases.len());
                    black_box(match name {
                        "add" => lhs_cases[index].add(&rhs_cases[index], &ctx),
                        "sub" => lhs_cases[index].sub(&rhs_cases[index], &ctx),
                        "mul" => lhs_cases[index].mul(&rhs_cases[index], &ctx),
                        _ => lhs_cases[index].div(&rhs_cases[index], &ctx),
                    })
                })
            });
        }
        $group.bench_function(format!("{}/neg", $label), |b| {
            let cursor = Cell::new(0);
            b.iter(|| black_box(next_case(&lhs_cases, &cursor).neg(&ctx)))
        });
        $group.bench_function(format!("{}/div_real", $label), |b| {
            let cursor = Cell::new(0);
            b.iter(|| {
                let index = cursor.get();
                cursor.set((index + 1) % lhs_cases.len());
                black_box(lhs_cases[index].div_real(&real_cases[index], &ctx))
            })
        });
    }};
}

fn bench_numerica_complex_operations(
    group: &mut BenchmarkGroup<'_, criterion::measurement::WallTime>,
    label: &str,
) {
    bench_external_complex_operations!(numerica_engine, group, label);
}

fn bench_symbolica_complex_operations(
    group: &mut BenchmarkGroup<'_, criterion::measurement::WallTime>,
    label: &str,
) {
    bench_external_complex_operations!(symbolica_engine, group, label);
}
