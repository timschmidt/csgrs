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
