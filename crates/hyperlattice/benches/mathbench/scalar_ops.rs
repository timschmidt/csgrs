fn bench_scalar_operations_for<B, F>(
    group: &mut BenchmarkGroup<'_, criterion::measurement::WallTime>,
    label: &str,
    make_scalar: F,
) where
    B: Backend,
    F: Copy + Fn(f64) -> Scalar<B>,
{
    let arithmetic_cases = [
        (make_scalar(2.5), make_scalar(1.25)),
        (make_scalar(1.0e-12), make_scalar(-1.0e-12)),
        (make_scalar(1.0e9), make_scalar(1.0e-9)),
        (make_scalar(-2.75), make_scalar(0.125)),
    ];
    let pow_cases = [
        (make_scalar(2.5), make_scalar(1.25)),
        (make_scalar(1.0e-12), make_scalar(3.5)),
        (make_scalar(1.0e9), make_scalar(0.25)),
        (
            make_scalar(std::f64::consts::E),
            make_scalar(std::f64::consts::FRAC_1_PI),
        ),
    ];
    let reciprocal_cases = [
        make_scalar(1.25),
        make_scalar(1.0e-12),
        make_scalar(-1.0e12),
        make_scalar(std::f64::consts::PI),
    ];
    let positive_cases = [
        make_scalar(9.0),
        make_scalar(1.0e-12),
        make_scalar(1.0e12),
        make_scalar(std::f64::consts::E),
    ];
    let trig_cases = [
        make_scalar(0.5),
        make_scalar(std::f64::consts::PI / 7.0),
        make_scalar(1.0e6),
        make_scalar(1000.0 * std::f64::consts::PI + 1.0e-20),
    ];
    let hyperbolic_cases = [
        make_scalar(0.5),
        make_scalar(-1.0e-12),
        make_scalar(20.0),
        make_scalar(-20.0),
    ];
    let unit_interval_cases = [
        make_scalar(0.5),
        make_scalar(-0.999_999),
        make_scalar(0.999_999),
        make_scalar(1.0e-12),
    ];
    let acosh_cases = [
        make_scalar(9.0),
        make_scalar(1.0 + 1.0e-12),
        make_scalar(1.0e6),
        make_scalar(std::f64::consts::E),
    ];
    let zero_status_cases = [
        make_scalar(2.5),
        Scalar::<B>::zero(),
        make_scalar(1.0e-12),
        make_scalar(-1.0e12),
    ];
    let signal = abort_signal();

    group.bench_function(format!("{label}/zero"), |b| {
        b.iter(|| black_box(Scalar::<B>::zero()))
    });
    group.bench_function(format!("{label}/one"), |b| {
        b.iter(|| black_box(Scalar::<B>::one()))
    });
    group.bench_function(format!("{label}/e"), |b| {
        b.iter(|| black_box(Scalar::<B>::e()))
    });
    group.bench_function(format!("{label}/pi"), |b| {
        b.iter(|| black_box(Scalar::<B>::pi()))
    });
    group.bench_function(format!("{label}/tau"), |b| {
        b.iter(|| black_box(Scalar::<B>::tau()))
    });
    group.bench_function(format!("{label}/add"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let (lhs, rhs) = next_case(&arithmetic_cases, &cursor);
            black_box(black_box(lhs.clone()) + black_box(rhs.clone()))
        })
    });
    group.bench_function(format!("{label}/sub"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let (lhs, rhs) = next_case(&arithmetic_cases, &cursor);
            black_box(black_box(lhs.clone()) - black_box(rhs.clone()))
        })
    });
    group.bench_function(format!("{label}/neg"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| black_box(-black_box(next_case(&reciprocal_cases, &cursor).clone())))
    });
    group.bench_function(format!("{label}/mul"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let (lhs, rhs) = next_case(&arithmetic_cases, &cursor);
            black_box(black_box(lhs.clone()) * black_box(rhs.clone()))
        })
    });
    group.bench_function(format!("{label}/div"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let (lhs, rhs) = next_case(&arithmetic_cases, &cursor);
            black_box((black_box(lhs.clone()) / black_box(rhs.clone())).unwrap())
        })
    });
    group.bench_function(format!("{label}/reciprocal"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                realistic_blas::reciprocal(black_box(
                    next_case(&reciprocal_cases, &cursor).clone(),
                ))
                .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/reciprocal_checked"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                realistic_blas::reciprocal_checked(black_box(
                    next_case(&reciprocal_cases, &cursor).clone(),
                ))
                .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/reciprocal_checked_abort"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                realistic_blas::reciprocal_checked_with_abort(
                    black_box(next_case(&reciprocal_cases, &cursor).clone()),
                    &signal,
                )
                .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/pow"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let (lhs, rhs) = next_case(&pow_cases, &cursor);
            black_box(realistic_blas::pow(black_box(lhs.clone()), black_box(rhs.clone())).unwrap())
        })
    });
    group.bench_function(format!("{label}/powi"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                realistic_blas::powi(black_box(next_case(&reciprocal_cases, &cursor).clone()), 5)
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/exp"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                realistic_blas::exp(black_box(next_case(&unit_interval_cases, &cursor).clone()))
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/ln"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                realistic_blas::ln(black_box(next_case(&positive_cases, &cursor).clone())).unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/log10"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                realistic_blas::log10(black_box(next_case(&positive_cases, &cursor).clone()))
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/log10_abort"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                realistic_blas::log10_with_abort(
                    black_box(next_case(&positive_cases, &cursor).clone()),
                    &signal,
                )
                .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/sqrt"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                realistic_blas::sqrt(black_box(next_case(&positive_cases, &cursor).clone()))
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/sin"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(realistic_blas::sin(black_box(
                next_case(&trig_cases, &cursor).clone(),
            )))
        })
    });
    group.bench_function(format!("{label}/cos"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(realistic_blas::cos(black_box(
                next_case(&trig_cases, &cursor).clone(),
            )))
        })
    });
    group.bench_function(format!("{label}/tan"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                realistic_blas::tan(black_box(next_case(&trig_cases, &cursor).clone())).unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/sinh"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                realistic_blas::sinh(black_box(next_case(&hyperbolic_cases, &cursor).clone()))
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/cosh"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                realistic_blas::cosh(black_box(next_case(&hyperbolic_cases, &cursor).clone()))
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/tanh"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                realistic_blas::tanh(black_box(next_case(&hyperbolic_cases, &cursor).clone()))
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/asin"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                realistic_blas::asin(black_box(next_case(&unit_interval_cases, &cursor).clone()))
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/asin_abort"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                realistic_blas::asin_with_abort(
                    black_box(next_case(&unit_interval_cases, &cursor).clone()),
                    &signal,
                )
                .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/acos"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                realistic_blas::acos(black_box(next_case(&unit_interval_cases, &cursor).clone()))
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/acos_abort"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                realistic_blas::acos_with_abort(
                    black_box(next_case(&unit_interval_cases, &cursor).clone()),
                    &signal,
                )
                .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/atan"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                realistic_blas::atan(black_box(next_case(&trig_cases, &cursor).clone())).unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/atan_abort"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                realistic_blas::atan_with_abort(
                    black_box(next_case(&trig_cases, &cursor).clone()),
                    &signal,
                )
                .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/asinh"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                realistic_blas::asinh(black_box(next_case(&trig_cases, &cursor).clone())).unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/asinh_abort"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                realistic_blas::asinh_with_abort(
                    black_box(next_case(&trig_cases, &cursor).clone()),
                    &signal,
                )
                .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/acosh"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                realistic_blas::acosh(black_box(next_case(&acosh_cases, &cursor).clone())).unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/acosh_abort"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                realistic_blas::acosh_with_abort(
                    black_box(next_case(&acosh_cases, &cursor).clone()),
                    &signal,
                )
                .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/atanh"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                realistic_blas::atanh(black_box(next_case(&unit_interval_cases, &cursor).clone()))
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/atanh_abort"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                realistic_blas::atanh_with_abort(
                    black_box(next_case(&unit_interval_cases, &cursor).clone()),
                    &signal,
                )
                .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/zero_status"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(realistic_blas::zero_status(black_box(next_case(
                &zero_status_cases,
                &cursor,
            ))))
        })
    });
    group.bench_function(format!("{label}/zero_status_abort"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(realistic_blas::zero_status_with_abort(
                black_box(next_case(&zero_status_cases, &cursor)),
                &signal,
            ))
        })
    });
}

fn bench_astro_scalar_operations(
    group: &mut BenchmarkGroup<'_, criterion::measurement::WallTime>,
    label: &str,
) {
    let mut ctx = astro_backend::Ctx::new(128);
    let arithmetic_cases = [
        (2.5, 1.25),
        (1.0e-12, -1.0e-12),
        (1.0e9, 1.0e-9),
        (-2.75, 0.125),
    ]
    .map(|(lhs, rhs)| (ctx.f(lhs), ctx.f(rhs)));
    let pow_cases = [
        (2.5, 1.25),
        (1.0e-12, 3.5),
        (1.0e9, 0.25),
        (std::f64::consts::E, std::f64::consts::FRAC_1_PI),
    ]
    .map(|(lhs, rhs)| (ctx.f(lhs), ctx.f(rhs)));
    let reciprocal_cases = [1.25, 1.0e-12, -1.0e12, std::f64::consts::PI].map(|value| ctx.f(value));
    let positive_cases = [9.0, 1.0e-12, 1.0e12, std::f64::consts::E].map(|value| ctx.f(value));
    let trig_cases = [
        0.5,
        std::f64::consts::PI / 7.0,
        1.0e6,
        1000.0 * std::f64::consts::PI + 1.0e-20,
    ]
    .map(|value| ctx.f(value));
    let hyperbolic_cases = [0.5, -1.0e-12, 20.0, -20.0].map(|value| ctx.f(value));
    let unit_interval_cases = [0.5, -0.999_999, 0.999_999, 1.0e-12].map(|value| ctx.f(value));
    let acosh_cases = [9.0, 1.0 + 1.0e-12, 1.0e6, std::f64::consts::E].map(|value| ctx.f(value));
    let zero_status_cases = [ctx.f(2.5), ctx.zero(), ctx.f(1.0e-12), ctx.f(-1.0e12)];

    group.bench_function(format!("{label}/zero"), |b| {
        b.iter(|| black_box(ctx.zero()))
    });
    group.bench_function(format!("{label}/one"), |b| b.iter(|| black_box(ctx.one())));
    group.bench_function(format!("{label}/e"), |b| b.iter(|| black_box(ctx.e())));
    group.bench_function(format!("{label}/pi"), |b| b.iter(|| black_box(ctx.pi())));
    group.bench_function(format!("{label}/tau"), |b| b.iter(|| black_box(ctx.tau())));
    group.bench_function(format!("{label}/add"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let (lhs, rhs) = next_case(&arithmetic_cases, &cursor);
            black_box(ctx.add(black_box(lhs), black_box(rhs)))
        })
    });
    group.bench_function(format!("{label}/sub"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let (lhs, rhs) = next_case(&arithmetic_cases, &cursor);
            black_box(ctx.sub(black_box(lhs), black_box(rhs)))
        })
    });
    group.bench_function(format!("{label}/neg"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| black_box(ctx.neg(black_box(next_case(&reciprocal_cases, &cursor)))))
    });
    group.bench_function(format!("{label}/mul"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let (lhs, rhs) = next_case(&arithmetic_cases, &cursor);
            black_box(ctx.mul(black_box(lhs), black_box(rhs)))
        })
    });
    group.bench_function(format!("{label}/div"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let (lhs, rhs) = next_case(&arithmetic_cases, &cursor);
            black_box(ctx.div(black_box(lhs), black_box(rhs)))
        })
    });
    for name in [
        "reciprocal",
        "reciprocal_checked",
        "reciprocal_checked_abort",
    ] {
        group.bench_function(format!("{label}/{name}"), |b| {
            let cursor = Cell::new(0);
            b.iter(|| black_box(ctx.reciprocal(black_box(next_case(&reciprocal_cases, &cursor)))))
        });
    }
    group.bench_function(format!("{label}/pow"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let (lhs, rhs) = next_case(&pow_cases, &cursor);
            black_box(ctx.pow(black_box(lhs), black_box(rhs)))
        })
    });
    group.bench_function(format!("{label}/powi"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| black_box(ctx.powi(black_box(next_case(&reciprocal_cases, &cursor)), 5)))
    });
    group.bench_function(format!("{label}/exp"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| black_box(ctx.exp(black_box(next_case(&unit_interval_cases, &cursor)))))
    });
    for name in ["ln", "log10", "log10_abort", "sqrt"] {
        group.bench_function(format!("{label}/{name}"), |b| {
            let cursor = Cell::new(0);
            b.iter(|| {
                let value = black_box(next_case(&positive_cases, &cursor));
                black_box(match name {
                    "ln" => ctx.ln(value),
                    "sqrt" => ctx.sqrt(value),
                    _ => ctx.log10(value),
                })
            })
        });
    }
    for (name, values) in [
        ("sin", &trig_cases[..]),
        ("cos", &trig_cases[..]),
        ("tan", &trig_cases[..]),
        ("sinh", &hyperbolic_cases[..]),
        ("cosh", &hyperbolic_cases[..]),
        ("tanh", &hyperbolic_cases[..]),
        ("asin", &unit_interval_cases[..]),
        ("asin_abort", &unit_interval_cases[..]),
        ("acos", &unit_interval_cases[..]),
        ("acos_abort", &unit_interval_cases[..]),
        ("atan", &trig_cases[..]),
        ("atan_abort", &trig_cases[..]),
        ("asinh", &trig_cases[..]),
        ("asinh_abort", &trig_cases[..]),
        ("acosh", &acosh_cases[..]),
        ("acosh_abort", &acosh_cases[..]),
        ("atanh", &unit_interval_cases[..]),
        ("atanh_abort", &unit_interval_cases[..]),
    ] {
        group.bench_function(format!("{label}/{name}"), |b| {
            let cursor = Cell::new(0);
            b.iter(|| {
                let value = black_box(next_case(values, &cursor));
                black_box(match name {
                    "sin" => ctx.sin(value),
                    "cos" => ctx.cos(value),
                    "tan" => ctx.tan(value),
                    "sinh" => ctx.sinh(value),
                    "cosh" => ctx.cosh(value),
                    "tanh" => ctx.tanh(value),
                    "asin" | "asin_abort" => ctx.asin(value),
                    "acos" | "acos_abort" => ctx.acos(value),
                    "atan" | "atan_abort" => ctx.atan(value),
                    "asinh" | "asinh_abort" => ctx.asinh(value),
                    "acosh" | "acosh_abort" => ctx.acosh(value),
                    _ => ctx.atanh(value),
                })
            })
        });
    }
    for name in ["zero_status", "zero_status_abort"] {
        group.bench_function(format!("{label}/{name}"), |b| {
            let cursor = Cell::new(0);
            b.iter(|| black_box(ctx.is_zero(black_box(next_case(&zero_status_cases, &cursor)))))
        });
    }
}

fn bench_scalar_operations(c: &mut Criterion) {
    let mut group = c.benchmark_group("scalar_ops");
    bench_scalar_operations_for::<ApproxBackend, _>(&mut group, "approx", s::<ApproxBackend>);
    bench_scalar_operations_for::<HyperrealBackend, _>(
        &mut group,
        "hyperreal",
        s::<HyperrealBackend>,
    );
    bench_scalar_operations_for::<HyperrealBackend, _>(&mut group, "hyperreal-rational", qr);
    bench_astro_scalar_operations(&mut group, "astro128");
    bench_numerica_scalar_operations(&mut group, "numerica128");
    bench_symbolica_scalar_operations(&mut group, "symbolica");
    group.finish();
}

fn bench_numerica_scalar_operations(
    group: &mut BenchmarkGroup<'_, criterion::measurement::WallTime>,
    label: &str,
) {
    let ctx = numerica_backend::Ctx::new(128);
    let arithmetic_cases = [
        (2.5, 1.25),
        (1.0e-12, -1.0e-12),
        (1.0e9, 1.0e-9),
        (-2.75, 0.125),
    ]
    .map(|(lhs, rhs)| (ctx.f(lhs), ctx.f(rhs)));
    let pow_cases = [
        (2.5, 1.25),
        (1.0e-12, 3.5),
        (1.0e9, 0.25),
        (std::f64::consts::E, std::f64::consts::FRAC_1_PI),
    ]
    .map(|(lhs, rhs)| (ctx.f(lhs), ctx.f(rhs)));
    let reciprocal_cases = [1.25, 1.0e-12, -1.0e12, std::f64::consts::PI].map(|value| ctx.f(value));
    let positive_cases = [9.0, 1.0e-12, 1.0e12, std::f64::consts::E].map(|value| ctx.f(value));
    let trig_cases = [
        0.5,
        std::f64::consts::PI / 7.0,
        1.0e6,
        1000.0 * std::f64::consts::PI + 1.0e-20,
    ]
    .map(|value| ctx.f(value));
    let hyperbolic_cases = [0.5, -1.0e-12, 20.0, -20.0].map(|value| ctx.f(value));
    let unit_interval_cases = [0.5, -0.999_999, 0.999_999, 1.0e-12].map(|value| ctx.f(value));
    let acosh_cases = [9.0, 1.0 + 1.0e-12, 1.0e6, std::f64::consts::E].map(|value| ctx.f(value));
    let zero_status_cases = [ctx.f(2.5), ctx.zero(), ctx.f(1.0e-12), ctx.f(-1.0e12)];

    group.bench_function(format!("{label}/zero"), |b| b.iter(|| black_box(ctx.zero())));
    group.bench_function(format!("{label}/one"), |b| b.iter(|| black_box(ctx.one())));
    group.bench_function(format!("{label}/e"), |b| b.iter(|| black_box(ctx.e())));
    group.bench_function(format!("{label}/pi"), |b| b.iter(|| black_box(ctx.pi())));
    group.bench_function(format!("{label}/tau"), |b| b.iter(|| black_box(ctx.tau())));
    group.bench_function(format!("{label}/add"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let (lhs, rhs) = next_case(&arithmetic_cases, &cursor);
            black_box(ctx.add(black_box(lhs), black_box(rhs)))
        })
    });
    group.bench_function(format!("{label}/sub"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let (lhs, rhs) = next_case(&arithmetic_cases, &cursor);
            black_box(ctx.sub(black_box(lhs), black_box(rhs)))
        })
    });
    group.bench_function(format!("{label}/neg"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| black_box(ctx.neg(black_box(next_case(&reciprocal_cases, &cursor)))))
    });
    group.bench_function(format!("{label}/mul"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let (lhs, rhs) = next_case(&arithmetic_cases, &cursor);
            black_box(ctx.mul(black_box(lhs), black_box(rhs)))
        })
    });
    group.bench_function(format!("{label}/div"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let (lhs, rhs) = next_case(&arithmetic_cases, &cursor);
            black_box(ctx.div(black_box(lhs), black_box(rhs)))
        })
    });
    for name in [
        "reciprocal",
        "reciprocal_checked",
        "reciprocal_checked_abort",
    ] {
        group.bench_function(format!("{label}/{name}"), |b| {
            let cursor = Cell::new(0);
            b.iter(|| black_box(ctx.reciprocal(black_box(next_case(&reciprocal_cases, &cursor)))))
        });
    }
    group.bench_function(format!("{label}/pow"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let (lhs, rhs) = next_case(&pow_cases, &cursor);
            black_box(ctx.pow(black_box(lhs), black_box(rhs)))
        })
    });
    group.bench_function(format!("{label}/powi"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| black_box(ctx.powi(black_box(next_case(&reciprocal_cases, &cursor)), 5)))
    });
    group.bench_function(format!("{label}/exp"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| black_box(ctx.exp(black_box(next_case(&unit_interval_cases, &cursor)))))
    });
    for name in ["ln", "log10", "log10_abort", "sqrt"] {
        group.bench_function(format!("{label}/{name}"), |b| {
            let cursor = Cell::new(0);
            b.iter(|| {
                let value = black_box(next_case(&positive_cases, &cursor));
                black_box(match name {
                    "ln" => ctx.ln(value),
                    "sqrt" => ctx.sqrt(value),
                    _ => ctx.log10(value),
                })
            })
        });
    }
    for (name, values) in [
        ("sin", &trig_cases[..]),
        ("cos", &trig_cases[..]),
        ("tan", &trig_cases[..]),
        ("sinh", &hyperbolic_cases[..]),
        ("cosh", &hyperbolic_cases[..]),
        ("tanh", &hyperbolic_cases[..]),
        ("asin", &unit_interval_cases[..]),
        ("asin_abort", &unit_interval_cases[..]),
        ("acos", &unit_interval_cases[..]),
        ("acos_abort", &unit_interval_cases[..]),
        ("atan", &trig_cases[..]),
        ("atan_abort", &trig_cases[..]),
        ("asinh", &trig_cases[..]),
        ("asinh_abort", &trig_cases[..]),
        ("acosh", &acosh_cases[..]),
        ("acosh_abort", &acosh_cases[..]),
        ("atanh", &unit_interval_cases[..]),
        ("atanh_abort", &unit_interval_cases[..]),
    ] {
        group.bench_function(format!("{label}/{name}"), |b| {
            let cursor = Cell::new(0);
            b.iter(|| {
                let value = black_box(next_case(values, &cursor));
                black_box(match name {
                    "sin" => ctx.sin(value),
                    "cos" => ctx.cos(value),
                    "tan" => ctx.tan(value),
                    "sinh" => ctx.sinh(value),
                    "cosh" => ctx.cosh(value),
                    "tanh" => ctx.tanh(value),
                    "asin" | "asin_abort" => ctx.asin(value),
                    "acos" | "acos_abort" => ctx.acos(value),
                    "atan" | "atan_abort" => ctx.atan(value),
                    "asinh" | "asinh_abort" => ctx.asinh(value),
                    "acosh" | "acosh_abort" => ctx.acosh(value),
                    _ => ctx.atanh(value),
                })
            })
        });
    }
    for name in ["zero_status", "zero_status_abort"] {
        group.bench_function(format!("{label}/{name}"), |b| {
            let cursor = Cell::new(0);
            b.iter(|| black_box(ctx.is_zero(black_box(next_case(&zero_status_cases, &cursor)))))
        });
    }
}

fn bench_symbolica_scalar_operations(
    group: &mut BenchmarkGroup<'_, criterion::measurement::WallTime>,
    label: &str,
) {
    let ctx = symbolica_backend::Ctx::new(128);
    let arithmetic_cases = [
        (2.5, 1.25),
        (1.0e-12, -1.0e-12),
        (1.0e9, 1.0e-9),
        (-2.75, 0.125),
    ]
    .map(|(lhs, rhs)| (ctx.f(lhs), ctx.f(rhs)));
    let pow_cases = [
        (2.5, 1.25),
        (1.0e-12, 3.5),
        (1.0e9, 0.25),
        (std::f64::consts::E, std::f64::consts::FRAC_1_PI),
    ]
    .map(|(lhs, rhs)| (ctx.f(lhs), ctx.f(rhs)));
    let reciprocal_cases = [1.25, 1.0e-12, -1.0e12, std::f64::consts::PI].map(|value| ctx.f(value));
    let positive_cases = [9.0, 1.0e-12, 1.0e12, std::f64::consts::E].map(|value| ctx.f(value));
    let trig_cases = [
        0.5,
        std::f64::consts::PI / 7.0,
        1.0e6,
        1000.0 * std::f64::consts::PI + 1.0e-20,
    ]
    .map(|value| ctx.f(value));
    let hyperbolic_cases = [0.5, -1.0e-12, 20.0, -20.0].map(|value| ctx.f(value));
    let unit_interval_cases = [0.5, -0.999_999, 0.999_999, 1.0e-12].map(|value| ctx.f(value));
    let acosh_cases = [9.0, 1.0 + 1.0e-12, 1.0e6, std::f64::consts::E].map(|value| ctx.f(value));
    let zero_status_cases = [ctx.f(2.5), ctx.zero(), ctx.f(1.0e-12), ctx.f(-1.0e12)];

    group.bench_function(format!("{label}/zero"), |b| b.iter(|| black_box(ctx.zero())));
    group.bench_function(format!("{label}/one"), |b| b.iter(|| black_box(ctx.one())));
    group.bench_function(format!("{label}/e"), |b| b.iter(|| black_box(ctx.e())));
    group.bench_function(format!("{label}/pi"), |b| b.iter(|| black_box(ctx.pi())));
    group.bench_function(format!("{label}/tau"), |b| b.iter(|| black_box(ctx.tau())));
    group.bench_function(format!("{label}/add"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let (lhs, rhs) = next_case(&arithmetic_cases, &cursor);
            black_box(ctx.add(black_box(lhs), black_box(rhs)))
        })
    });
    group.bench_function(format!("{label}/sub"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let (lhs, rhs) = next_case(&arithmetic_cases, &cursor);
            black_box(ctx.sub(black_box(lhs), black_box(rhs)))
        })
    });
    group.bench_function(format!("{label}/neg"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| black_box(ctx.neg(black_box(next_case(&reciprocal_cases, &cursor)))))
    });
    group.bench_function(format!("{label}/mul"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let (lhs, rhs) = next_case(&arithmetic_cases, &cursor);
            black_box(ctx.mul(black_box(lhs), black_box(rhs)))
        })
    });
    group.bench_function(format!("{label}/div"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let (lhs, rhs) = next_case(&arithmetic_cases, &cursor);
            black_box(ctx.div(black_box(lhs), black_box(rhs)))
        })
    });
    for name in [
        "reciprocal",
        "reciprocal_checked",
        "reciprocal_checked_abort",
    ] {
        group.bench_function(format!("{label}/{name}"), |b| {
            let cursor = Cell::new(0);
            b.iter(|| black_box(ctx.reciprocal(black_box(next_case(&reciprocal_cases, &cursor)))))
        });
    }
    group.bench_function(format!("{label}/pow"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let (lhs, rhs) = next_case(&pow_cases, &cursor);
            black_box(ctx.pow(black_box(lhs), black_box(rhs)))
        })
    });
    group.bench_function(format!("{label}/powi"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| black_box(ctx.powi(black_box(next_case(&reciprocal_cases, &cursor)), 5)))
    });
    group.bench_function(format!("{label}/exp"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| black_box(ctx.exp(black_box(next_case(&unit_interval_cases, &cursor)))))
    });
    for name in ["ln", "log10", "log10_abort", "sqrt"] {
        group.bench_function(format!("{label}/{name}"), |b| {
            let cursor = Cell::new(0);
            b.iter(|| {
                let value = black_box(next_case(&positive_cases, &cursor));
                black_box(match name {
                    "ln" => ctx.ln(value),
                    "sqrt" => ctx.sqrt(value),
                    _ => ctx.log10(value),
                })
            })
        });
    }
    for (name, values) in [
        ("sin", &trig_cases[..]),
        ("cos", &trig_cases[..]),
        ("tan", &trig_cases[..]),
        ("sinh", &hyperbolic_cases[..]),
        ("cosh", &hyperbolic_cases[..]),
        ("tanh", &hyperbolic_cases[..]),
        ("asin", &unit_interval_cases[..]),
        ("asin_abort", &unit_interval_cases[..]),
        ("acos", &unit_interval_cases[..]),
        ("acos_abort", &unit_interval_cases[..]),
        ("atan", &trig_cases[..]),
        ("atan_abort", &trig_cases[..]),
        ("asinh", &trig_cases[..]),
        ("asinh_abort", &trig_cases[..]),
        ("acosh", &acosh_cases[..]),
        ("acosh_abort", &acosh_cases[..]),
        ("atanh", &unit_interval_cases[..]),
        ("atanh_abort", &unit_interval_cases[..]),
    ] {
        group.bench_function(format!("{label}/{name}"), |b| {
            let cursor = Cell::new(0);
            b.iter(|| {
                let value = black_box(next_case(values, &cursor));
                black_box(match name {
                    "sin" => ctx.sin(value),
                    "cos" => ctx.cos(value),
                    "tan" => ctx.tan(value),
                    "sinh" => ctx.sinh(value),
                    "cosh" => ctx.cosh(value),
                    "tanh" => ctx.tanh(value),
                    "asin" | "asin_abort" => ctx.asin(value),
                    "acos" | "acos_abort" => ctx.acos(value),
                    "atan" | "atan_abort" => ctx.atan(value),
                    "asinh" | "asinh_abort" => ctx.asinh(value),
                    "acosh" | "acosh_abort" => ctx.acosh(value),
                    _ => ctx.atanh(value),
                })
            })
        });
    }
    for name in ["zero_status", "zero_status_abort"] {
        group.bench_function(format!("{label}/{name}"), |b| {
            let cursor = Cell::new(0);
            b.iter(|| black_box(ctx.is_zero(black_box(next_case(&zero_status_cases, &cursor)))))
        });
    }
}
