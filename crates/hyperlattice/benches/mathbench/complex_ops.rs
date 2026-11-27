/// Register 21 native complex-operation rows for one scalar-conversion label.
///
/// Constructs four retained left/right complex pairs and four real divisors before
/// timing: moderate values, tiny values, large mixed scales and rounded floating
/// constants. All constants reach make_scalar as binary64 first. The caller chooses
/// exact binary import, decimal-string reconstruction or another meaning; arbitrary
/// converters can invalidate the nonzero assumptions behind existing unwraps.
///
/// | Row suffix | Measured operation |
/// | --- | --- |
/// | zero, one | Construct the additive/multiplicative identity. |
/// | i, free_i | Both call Complex::i; free_i is a duplicate label, not a free-function call. |
/// | conjugate | Clone and conjugate the current left value. |
/// | norm_squared | Borrow the current left value and construct its squared modulus. |
/// | reciprocal, reciprocal_checked | Clone and unwrap the native reciprocal or checked reciprocal. |
/// | powi, powi_checked | Clone and unwrap exponent five through the selected API. |
/// | powi_negative_one, powi_checked_negative_one | Clone and unwrap exponent minus one through the selected API. |
/// | div_checked, div_real_checked | Clone and divide by the aligned complex or real divisor, unwrapping the checked result. |
/// | from_scalar | Clone the rotating real fixture and embed it with zero imaginary part. |
/// | add, sub, mul | Clone aligned left/right values and apply the owned operator. |
/// | neg | Clone and negate the rotating left value. |
/// | div, div_real | Clone and divide by the aligned complex or real fixture, unwrapping. |
///
/// Nonconstant rows cycle through four retained cases with per-callback cursors.
/// Selection, cursor updates, explicit clones and returned-result destruction are
/// inside ordinary Criterion iter timing; fixture construction is outside. Evidence
/// can be reused across retained inputs. Errors panic at unwrap; no value, exact
/// count or dispatch-path expectation is asserted, and no policy receipt is selected.
///
/// Before registering measured rows, seven optional trace hooks each traverse four
/// cases once: powi, mul, div, div_checked, powi_negative_one, powi_checked and
/// powi_checked_negative_one. Their setup is outside recording but cloning and
/// operations are inside. Trace filtering/enablement belongs to trace_dispatch_cases;
/// these probes are not per-iteration timing or a complete trace of all 21 rows.
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
        format!("complex_ops/{label}/div"),
        &[0_usize, 1, 2, 3],
        |index| {
            let _ = black_box((lhs_cases[*index].clone() / rhs_cases[*index].clone()).unwrap());
        },
    );
    trace_dispatch_cases(
        format!("complex_ops/{label}/div_checked"),
        &[0_usize, 1, 2, 3],
        |index| {
            let _ = black_box(
                lhs_cases[*index]
                    .clone()
                    .div_checked(rhs_cases[*index].clone())
                    .unwrap(),
            );
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
    group.bench_function(format!("{label}/i"), |b| b.iter(|| black_box(Complex::i())));
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

/// Register complex_ops for two native input models and three comparison adapters.
///
/// Native hyperreal uses s (exact binary64 import); hyperreal-rational uses qr
/// (exact rational parsing of the float's displayed decimal). The other labels use
/// the local Numerica, Rug/GMP/MPFR and Symbolica adapters. Each supplies 21 row names,
/// giving 105 registrations, with group timing settings inherited from Criterion.
/// Shared names do not establish equal input values, precision, checked semantics,
/// algorithms or completed numerical contracts; this function compares no results.
fn bench_complex_operations(c: &mut Criterion) {
    let mut group = c.benchmark_group("complex_ops");
    bench_complex_operations_for::<_>(&mut group, "hyperreal", s);
    bench_complex_operations_for::<_>(&mut group, "hyperreal-rational", qr);
    bench_numerica_complex_operations(&mut group, "numerica128");
    bench_gmp_complex_operations(&mut group, "gmp_mpfr128");
    bench_symbolica_complex_operations(&mut group, "symbolica");
    group.finish();
}

/// Generate four binary64 components from delta=(index as f64)*2^-40.
///
/// Returns (3.25+delta, -2.125+2*delta, 1.75-delta, 0.625+3*delta), interpreted as
/// left real/imaginary and right real/imaginary parts. The from_bits expression
/// constructs the exact power-of-two step; casts, multiplies and additions still
/// use binary64 arithmetic. These are drifting deterministic inputs, not random
/// samples or proof of distinct cache keys. In particular indices 2^53 and 2^53+1
/// cast to the same float and therefore produce identical arrays. All u64 inputs
/// produce finite components; this helper itself does not increment the sequence.
fn varying_complex_mul_values(index: u64) -> [f64; 4] {
    let delta = index as f64 * f64::from_bits((1023_u64 - 40) << 52);
    [
        3.25 + delta,
        -2.125 + delta * 2.0,
        1.75 - delta,
        0.625 + delta * 3.0,
    ]
}

/// Register one native varying-input multiplication row with untimed batched setup.
///
/// Each invocation of Criterion's benchmark closure starts its Cell sequence at
/// zero. Setup advances that u64, generates four float components and converts them
/// into two fresh Complex carriers. Only the owned product and its consumed-input
/// destruction are in the measured routine; returned-output destruction follows
/// the measured batch interval. SmallInput controls batching, not a resource limit.
///
/// The name cold refers to reconstructing inputs instead of cloning retained ones;
/// no scalar/process/hardware cache is cleared, and generated values need not stay
/// unique. Conversion meaning and failure belong to make_scalar. The ordinary
/// sequence increment has no explicit overflow handling or total-work bound.
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

/// Register five varying-input product rows and two native diagnostic samples.
///
/// Native rows use s and qr; external rows construct local adapter contexts with
/// argument 128. Each measured row reconstructs its own sequence from zero in
/// SmallInput setup, then performs one multiplication per routine invocation.
/// External products borrow rhs while the native product consumes both carriers;
/// the harness does not establish matching allocation, arithmetic or certainty costs.
///
/// Optional native trace hooks each use only index 1,000,003 and include component
/// generation and construction inside recording, unlike the measured rows' untimed
/// setup. They do not characterize the whole varying sequence. All five row names
/// end in varying; no numerical equality, output precision or cold-cache assertion
/// is made, and no local sample/time overrides are supplied.
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

    let gmp_ctx = gmp_engine::Ctx::new(128);
    group.bench_function("gmp_mpfr128/varying", |b| {
        let sequence = Cell::new(0_u64);
        b.iter_batched(
            || {
                let index = sequence.get();
                sequence.set(index + 1);
                let [ar, ai, br, bi] = varying_complex_mul_values(index);
                (
                    gmp_engine::Complex::new(&gmp_ctx, ar, ai),
                    gmp_engine::Complex::new(&gmp_ctx, br, bi),
                )
            },
            |(lhs, rhs)| black_box(lhs.mul(&rhs, &gmp_ctx)),
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

/// Expand 21 comparison rows for a local engine module, group and label.
///
/// The engine identifier must supply Ctx and Complex adapters with the invoked
/// constructors/arithmetic methods. Creates one context with argument 128 and
/// retains the same four float-origin fixture shapes as the native family. Group
/// and label expressions are used repeatedly as written, not evaluated once by
/// this macro; current callers pass stable variables. Expands a statement block,
/// without defining additional functions or types.
///
/// Rows bearing _checked names reuse the ordinary adapter operations and add no
/// structural nonzero guard or typed Result. Both negative-one names call powi(1)
/// then reciprocal; the local adapters implement positive powers by multiplying
/// from one repeatedly. This is not the native signed-exponent schedule. free_i
/// repeats the i constructor. All cases are black-boxed without value assertions.
///
/// Each nonconstant row uses its own rotating cursor and borrowed retained inputs;
/// internal adapter cloning still contributes to timing. Native explicit operand
/// clones and native error checks therefore need not match this work. There are no
/// trace hooks here. Numeric meaning, precision, failures and initialization remain
/// adapter-dependent; passing 128 alone does not establish a common exact contract.
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
                b.iter(|| {
                    black_box(
                        next_case(&lhs_cases, &cursor)
                            .powi(1, &ctx)
                            .reciprocal(&ctx),
                    )
                })
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

/// Instantiate the external-row macro for the local numerica_engine adapter.
///
/// This adapter uses Symbolica's Float backend with requested precision 128, not a
/// separate Numerica crate. Float-origin inputs and rounded arithmetic have no native
/// exactness/checked-result equivalence asserted by this wrapper.
fn bench_numerica_complex_operations(
    group: &mut BenchmarkGroup<'_, criterion::measurement::WallTime>,
    label: &str,
) {
    bench_external_complex_operations!(numerica_engine, group, label);
}

/// Instantiate the external-row macro for Rug's GMP/MPFR adapter at requested precision 128.
///
/// Inputs originate as binary64; higher arithmetic precision does not recover exact
/// authored decimal or symbolic constants. The wrapper adds no result validation or
/// checked-operation guard beyond the shared macro's ordinary adapter calls.
fn bench_gmp_complex_operations(
    group: &mut BenchmarkGroup<'_, criterion::measurement::WallTime>,
    label: &str,
) {
    bench_external_complex_operations!(gmp_engine, group, label);
}

/// Instantiate the external-row macro for the local Symbolica Atom adapter.
///
/// Ctx stores 128, but its complex path uses Atom::num(f64) ingress and Atom
/// arithmetic without consuming that field as a precision setting. The label does
/// not certify a 128-bit evaluation or native Real equivalence. Initialization and
/// license handling occur in the driver; this wrapper does not validate results.
fn bench_symbolica_complex_operations(
    group: &mut BenchmarkGroup<'_, criterion::measurement::WallTime>,
    label: &str,
) {
    bench_external_complex_operations!(symbolica_engine, group, label);
}
