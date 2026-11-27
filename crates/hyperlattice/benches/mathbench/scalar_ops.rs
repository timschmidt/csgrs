/// Registers forty native scalar rows over retained four-case input families.
///
/// `make_scalar` constructs fixtures before timing: callers use `s` for exact
/// binary-float ingress or `qr` for displayed-decimal rational authoring. Numeric
/// expressions, including `PI / 7` and `1000 * PI + 1e-20`, are evaluated as f64
/// first; the latter loses its tiny addend before conversion. Imported `E` and
/// `PI` fixtures are not the symbolic constants returned by the separate native
/// `e`, `pi` and `tau` constructor rows, which do not use `make_scalar`.
///
/// | Retained family | Consumers |
/// | --- | --- |
/// | Signed arithmetic pairs, including cancellation and disparate scales | add, sub, mul, div |
/// | Positive bases with fractional exponents | pow |
/// | Nonzero signed scalars | neg, reciprocal variants, powers 5 and -1 |
/// | Positive scalars | ln, log10 variants, sqrt |
/// | Half, rounded PI/7, one million, rounded 1000 PI | sin, cos, tan, atan/asinh variants |
/// | Half, negative tiny, positive/negative twenty | sinh, cosh, tanh |
/// | Scalars strictly between -1 and 1 | exp, asin/acos/atanh variants |
/// | Scalars greater than 1 | acosh variants |
/// | Three nonzero scalars and an explicit native zero | zero_status |
///
/// These domain descriptions apply to the supplied converters, not arbitrary
/// `F`. Native fallible operations are unwrapped and can panic if conversion or
/// evaluation fails. Checked reciprocal additionally requires known nonzero
/// status, rejecting both zero and unknown without refinement. `zero_status`
/// returns structural/cached `ZeroKnowledge`, not a general equality decision.
///
/// Ordinary `iter` times case selection, cursor advancement, operand clones,
/// arithmetic and output destruction. Each cycling row has its own cursor. Constants
/// are constructed inside timing; other inputs are retained and may share
/// evidence across iterations. Abort rows attach a cloned, clear signal to a
/// cloned operand inside timing, including any attachment/cache work. The flag
/// is never set here: these rows measure attachment plus the operation, not
/// cancellation response. Arithmetic outputs remain scalar objects; no primitive-float
/// conversion, common numerical accuracy target or policy receipt is requested.
///
/// Thirty-one trace hooks precede timed registrations. When enabled and selected,
/// each visits four cases: add/sub/mul perform two operations per case, sqrt
/// three, and all others one. Trace neg/mul/div borrow inputs whereas measured
/// rows clone them; trace counts do not describe an identical timed iteration.
/// Constants, reciprocal variants and zero_status have no trace hooks here.
/// No numerical or dispatch-count assertions or total-work/memory bounds are
/// imposed. There is no native zero_status_abort row in this schedule.
fn bench_scalar_operations_for<F>(
    group: &mut BenchmarkGroup<'_, criterion::measurement::WallTime>,
    label: &str,
    make_scalar: F,
) where
    F: Copy + Fn(f64) -> Real,
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
        Real::zero(),
        make_scalar(1.0e-12),
        make_scalar(-1.0e12),
    ];
    let signal = abort_signal();

    trace_dispatch_cases(
        format!("scalar_ops/{label}/add"),
        &arithmetic_cases,
        |(lhs, rhs)| {
            let _ = black_box(lhs.clone() + rhs.clone());
            let _ = black_box(lhs.clone() + rhs.clone());
        },
    );
    trace_dispatch_cases(
        format!("scalar_ops/{label}/sub"),
        &arithmetic_cases,
        |(lhs, rhs)| {
            let _ = black_box(lhs.clone() - rhs.clone());
            let _ = black_box(lhs.clone() - rhs.clone());
        },
    );
    trace_dispatch_cases(
        format!("scalar_ops/{label}/neg"),
        &reciprocal_cases,
        |value| {
            let _ = black_box(-value);
        },
    );
    trace_dispatch_cases(
        format!("scalar_ops/{label}/mul"),
        &arithmetic_cases,
        |(lhs, rhs)| {
            let _ = black_box(lhs * rhs);
            let _ = black_box(lhs * rhs);
        },
    );
    trace_dispatch_cases(
        format!("scalar_ops/{label}/div"),
        &arithmetic_cases,
        |(lhs, rhs)| {
            let _ = black_box((lhs / rhs).unwrap());
        },
    );

    trace_dispatch_cases(
        format!("scalar_ops/{label}/pow"),
        &pow_cases,
        |(lhs, rhs)| {
            let _ = black_box(Real::pow(lhs.clone(), rhs.clone()).unwrap());
        },
    );
    trace_dispatch_cases(
        format!("scalar_ops/{label}/powi"),
        &reciprocal_cases,
        |value| {
            let _ = black_box(Real::powi_i64(value.clone(), 5).unwrap());
        },
    );
    trace_dispatch_cases(
        format!("scalar_ops/{label}/powi_negative_one"),
        &reciprocal_cases,
        |value| {
            let _ = black_box(Real::powi_i64(value.clone(), -1).unwrap());
        },
    );
    trace_dispatch_cases(
        format!("scalar_ops/{label}/exp"),
        &unit_interval_cases,
        |value| {
            let _ = black_box(Real::exp(value.clone()).unwrap());
        },
    );
    trace_dispatch_cases(format!("scalar_ops/{label}/ln"), &positive_cases, |value| {
        let _ = black_box(Real::ln(value.clone()).unwrap());
    });
    trace_dispatch_cases(
        format!("scalar_ops/{label}/log10"),
        &positive_cases,
        |value| {
            let _ = black_box(Real::log10(value.clone()).unwrap());
        },
    );
    trace_dispatch_cases(
        format!("scalar_ops/{label}/log10_abort"),
        &positive_cases,
        |value| {
            let _ = black_box(
                {
                    let mut value = value.clone();
                    value.abort(signal.clone());
                    value.log10()
                }
                .unwrap(),
            );
        },
    );
    trace_dispatch_cases(
        format!("scalar_ops/{label}/sqrt"),
        &positive_cases,
        |value| {
            let _ = black_box(Real::sqrt(value.clone()).unwrap());
            let _ = black_box(Real::sqrt(value.clone()).unwrap());
            let _ = black_box(Real::sqrt(value.clone()).unwrap());
        },
    );
    trace_dispatch_cases(format!("scalar_ops/{label}/sin"), &trig_cases, |value| {
        let _ = black_box(Real::sin(value.clone()));
    });
    trace_dispatch_cases(format!("scalar_ops/{label}/cos"), &trig_cases, |value| {
        let _ = black_box(Real::cos(value.clone()));
    });
    trace_dispatch_cases(format!("scalar_ops/{label}/tan"), &trig_cases, |value| {
        let _ = black_box(Real::tan(value.clone()).unwrap());
    });
    trace_dispatch_cases(
        format!("scalar_ops/{label}/sinh"),
        &hyperbolic_cases,
        |value| {
            let _ = black_box(Real::sinh(value.clone()).unwrap());
        },
    );
    trace_dispatch_cases(
        format!("scalar_ops/{label}/cosh"),
        &hyperbolic_cases,
        |value| {
            let _ = black_box(Real::cosh(value.clone()).unwrap());
        },
    );
    trace_dispatch_cases(
        format!("scalar_ops/{label}/tanh"),
        &hyperbolic_cases,
        |value| {
            let _ = black_box(Real::tanh(value.clone()).unwrap());
        },
    );
    trace_dispatch_cases(
        format!("scalar_ops/{label}/asin"),
        &unit_interval_cases,
        |value| {
            let _ = black_box(Real::asin(value.clone()).unwrap());
        },
    );
    trace_dispatch_cases(
        format!("scalar_ops/{label}/asin_abort"),
        &unit_interval_cases,
        |value| {
            let _ = black_box(
                {
                    let mut value = value.clone();
                    value.abort(signal.clone());
                    value.asin()
                }
                .unwrap(),
            );
        },
    );
    trace_dispatch_cases(
        format!("scalar_ops/{label}/acos"),
        &unit_interval_cases,
        |value| {
            let _ = black_box(Real::acos(value.clone()).unwrap());
        },
    );
    trace_dispatch_cases(
        format!("scalar_ops/{label}/acos_abort"),
        &unit_interval_cases,
        |value| {
            let _ = black_box(
                {
                    let mut value = value.clone();
                    value.abort(signal.clone());
                    value.acos()
                }
                .unwrap(),
            );
        },
    );
    trace_dispatch_cases(format!("scalar_ops/{label}/atan"), &trig_cases, |value| {
        let _ = black_box(Real::atan(value.clone()).unwrap());
    });
    trace_dispatch_cases(
        format!("scalar_ops/{label}/atan_abort"),
        &trig_cases,
        |value| {
            let _ = black_box(
                {
                    let mut value = value.clone();
                    value.abort(signal.clone());
                    value.atan()
                }
                .unwrap(),
            );
        },
    );
    trace_dispatch_cases(format!("scalar_ops/{label}/asinh"), &trig_cases, |value| {
        let _ = black_box(Real::asinh(value.clone()).unwrap());
    });
    trace_dispatch_cases(
        format!("scalar_ops/{label}/asinh_abort"),
        &trig_cases,
        |value| {
            let _ = black_box(
                {
                    let mut value = value.clone();
                    value.abort(signal.clone());
                    value.asinh()
                }
                .unwrap(),
            );
        },
    );
    trace_dispatch_cases(format!("scalar_ops/{label}/acosh"), &acosh_cases, |value| {
        let _ = black_box(Real::acosh(value.clone()).unwrap());
    });
    trace_dispatch_cases(
        format!("scalar_ops/{label}/acosh_abort"),
        &acosh_cases,
        |value| {
            let _ = black_box(
                {
                    let mut value = value.clone();
                    value.abort(signal.clone());
                    value.acosh()
                }
                .unwrap(),
            );
        },
    );
    trace_dispatch_cases(
        format!("scalar_ops/{label}/atanh"),
        &unit_interval_cases,
        |value| {
            let _ = black_box(Real::atanh(value.clone()).unwrap());
        },
    );
    trace_dispatch_cases(
        format!("scalar_ops/{label}/atanh_abort"),
        &unit_interval_cases,
        |value| {
            let _ = black_box(
                {
                    let mut value = value.clone();
                    value.abort(signal.clone());
                    value.atanh()
                }
                .unwrap(),
            );
        },
    );

    group.bench_function(format!("{label}/zero"), |b| {
        b.iter(|| black_box(Real::zero()))
    });
    group.bench_function(format!("{label}/one"), |b| {
        b.iter(|| black_box(Real::one()))
    });
    group.bench_function(format!("{label}/e"), |b| b.iter(|| black_box(Real::e())));
    group.bench_function(format!("{label}/pi"), |b| b.iter(|| black_box(Real::pi())));
    group.bench_function(format!("{label}/tau"), |b| {
        b.iter(|| black_box(Real::tau()))
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
                Real::inverse(black_box(next_case(&reciprocal_cases, &cursor).clone())).unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/reciprocal_checked"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                hyperlattice::reciprocal_checked(black_box(
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
                {
                    let mut value = black_box(next_case(&reciprocal_cases, &cursor).clone());
                    value.abort(signal.clone());
                    hyperlattice::reciprocal_checked(value)
                }
                .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/pow"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let (lhs, rhs) = next_case(&pow_cases, &cursor);
            black_box(Real::pow(black_box(lhs.clone()), black_box(rhs.clone())).unwrap())
        })
    });
    group.bench_function(format!("{label}/powi"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                Real::powi_i64(black_box(next_case(&reciprocal_cases, &cursor).clone()), 5)
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/powi_negative_one"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                Real::powi_i64(black_box(next_case(&reciprocal_cases, &cursor).clone()), -1)
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/exp"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                Real::exp(black_box(next_case(&unit_interval_cases, &cursor).clone())).unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/ln"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(Real::ln(black_box(next_case(&positive_cases, &cursor).clone())).unwrap())
        })
    });
    group.bench_function(format!("{label}/log10"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(Real::log10(black_box(next_case(&positive_cases, &cursor).clone())).unwrap())
        })
    });
    group.bench_function(format!("{label}/log10_abort"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                {
                    let mut value = black_box(next_case(&positive_cases, &cursor).clone());
                    value.abort(signal.clone());
                    value.log10()
                }
                .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/sqrt"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(Real::sqrt(black_box(next_case(&positive_cases, &cursor).clone())).unwrap())
        })
    });
    group.bench_function(format!("{label}/sin"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(Real::sin(black_box(
                next_case(&trig_cases, &cursor).clone(),
            )))
        })
    });
    group.bench_function(format!("{label}/cos"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(Real::cos(black_box(
                next_case(&trig_cases, &cursor).clone(),
            )))
        })
    });
    group.bench_function(format!("{label}/tan"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| black_box(Real::tan(black_box(next_case(&trig_cases, &cursor).clone())).unwrap()))
    });
    group.bench_function(format!("{label}/sinh"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(Real::sinh(black_box(next_case(&hyperbolic_cases, &cursor).clone())).unwrap())
        })
    });
    group.bench_function(format!("{label}/cosh"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(Real::cosh(black_box(next_case(&hyperbolic_cases, &cursor).clone())).unwrap())
        })
    });
    group.bench_function(format!("{label}/tanh"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(Real::tanh(black_box(next_case(&hyperbolic_cases, &cursor).clone())).unwrap())
        })
    });
    group.bench_function(format!("{label}/asin"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                Real::asin(black_box(next_case(&unit_interval_cases, &cursor).clone())).unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/asin_abort"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                {
                    let mut value = black_box(next_case(&unit_interval_cases, &cursor).clone());
                    value.abort(signal.clone());
                    value.asin()
                }
                .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/acos"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                Real::acos(black_box(next_case(&unit_interval_cases, &cursor).clone())).unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/acos_abort"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                {
                    let mut value = black_box(next_case(&unit_interval_cases, &cursor).clone());
                    value.abort(signal.clone());
                    value.acos()
                }
                .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/atan"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(Real::atan(black_box(next_case(&trig_cases, &cursor).clone())).unwrap())
        })
    });
    group.bench_function(format!("{label}/atan_abort"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                {
                    let mut value = black_box(next_case(&trig_cases, &cursor).clone());
                    value.abort(signal.clone());
                    value.atan()
                }
                .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/asinh"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(Real::asinh(black_box(next_case(&trig_cases, &cursor).clone())).unwrap())
        })
    });
    group.bench_function(format!("{label}/asinh_abort"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                {
                    let mut value = black_box(next_case(&trig_cases, &cursor).clone());
                    value.abort(signal.clone());
                    value.asinh()
                }
                .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/acosh"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(Real::acosh(black_box(next_case(&acosh_cases, &cursor).clone())).unwrap())
        })
    });
    group.bench_function(format!("{label}/acosh_abort"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                {
                    let mut value = black_box(next_case(&acosh_cases, &cursor).clone());
                    value.abort(signal.clone());
                    value.acosh()
                }
                .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/atanh"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                Real::atanh(black_box(next_case(&unit_interval_cases, &cursor).clone())).unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/atanh_abort"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                {
                    let mut value = black_box(next_case(&unit_interval_cases, &cursor).clone());
                    value.abort(signal.clone());
                    value.atanh()
                }
                .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/zero_status"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(Real::zero_status(black_box(next_case(
                &zero_status_cases,
                &cursor,
            ))))
        })
    });
}

/// Registers and finishes the 203-row scalar comparison group.
///
/// Two native input models contribute forty rows each, and each of three external
/// adapters contributes forty-one under inherited Criterion settings. External
/// schedules additionally include zero_status_abort and use ordinary adapter
/// operations for checked/abort names. Only native registration offers diagnostic
/// trace hooks. Shared labels do not establish equal errors, cancellation behavior,
/// input values, numeric precision or measured work; see the individual registrars.
fn bench_scalar_operations(c: &mut Criterion) {
    let mut group = c.benchmark_group("scalar_ops");
    bench_scalar_operations_for::<_>(&mut group, "hyperreal", s);
    bench_scalar_operations_for::<_>(&mut group, "hyperreal-rational", qr);
    bench_numerica_scalar_operations(&mut group, "numerica128");
    bench_gmp_scalar_operations(&mut group, "gmp_mpfr128");
    bench_symbolica_scalar_operations(&mut group, "symbolica");
    group.finish();
}

/// Separates four retained square-root inputs into twenty backend/case rows.
///
/// The cases are 9, the binary-float literal 1e-12, 1e12 and imported binary-float E.
/// The two perfect cases have roots 3 and 1e6. Decimal authoring through `qr`
/// makes the tiny case exactly 1/10¹² with root 1/10⁶; `s` instead preserves its
/// binary-float value. Imported E is not the symbolic constant e.
///
/// Inputs and three external contexts with argument 128 are created outside timing.
/// Each ordinary iteration clones a native scalar and unwraps its square root, or
/// borrows an external input for the adapter's sqrt, then destroys the output.
/// There is no cursor, trace hook, output conversion or exact-value assertion.
/// Native results may retain exact/lazy definitions, floating adapters return their
/// numeric results, and Symbolica returns Atom expressions; these are not a common
/// precision/materialization contract. The group uses inherited Criterion settings.
fn bench_scalar_sqrt_cases(c: &mut Criterion) {
    let mut group = c.benchmark_group("scalar_sqrt_cases");
    let numerica_ctx = numerica_engine::Ctx::new(128);
    let gmp_ctx = gmp_engine::Ctx::new(128);
    let symbolica_ctx = symbolica_engine::Ctx::new(128);

    for (name, value) in [
        ("perfect_9", 9.0),
        ("tiny_1e_12", 1.0e-12),
        ("perfect_1e12", 1.0e12),
        ("e_import", std::f64::consts::E),
    ] {
        let hyperreal = s(value);
        let hyperreal_rational = qr(value);
        let numerica = numerica_ctx.f(value);
        let gmp = gmp_ctx.f(value);
        let symbolica = symbolica_ctx.f(value);

        group.bench_function(format!("hyperreal/{name}"), |b| {
            b.iter(|| black_box(Real::sqrt(black_box(hyperreal.clone())).unwrap()))
        });
        group.bench_function(format!("hyperreal-rational/{name}"), |b| {
            b.iter(|| black_box(Real::sqrt(black_box(hyperreal_rational.clone())).unwrap()))
        });
        group.bench_function(format!("numerica128/{name}"), |b| {
            b.iter(|| black_box(numerica_ctx.sqrt(black_box(&numerica))))
        });
        group.bench_function(format!("gmp_mpfr128/{name}"), |b| {
            b.iter(|| black_box(gmp_ctx.sqrt(black_box(&gmp))))
        });
        group.bench_function(format!("symbolica/{name}"), |b| {
            b.iter(|| black_box(symbolica_ctx.sqrt(black_box(&symbolica))))
        });
    }
    group.finish();
}

/// Registers sixty retained-input hyperbolic rows without a requested float output.
///
/// Each of half, negative 1e-12, twenty and negative twenty is paired with sinh,
/// cosh and tanh for two native models and three adapters. Inputs and external
/// contexts with argument 128 precede timing. Native `s` preserves binary-float
/// values; `qr` authors displayed-decimal rationals. Ordinary iterations time
/// operation selection, native clones or borrowed adapter calls, and destruction
/// of the result. Native Results are unwrapped; no numeric oracle, trace hook,
/// cancellation signal or total-work bound is installed.
///
/// Native results can retain exact/lazy definitions, floating adapters compute
/// numeric outputs, and Symbolica builds Atom expressions (hyperbolics through
/// exponentials). No common materialization or precision is enforced. Compare
/// [`bench_scalar_hyperbolic_f64_cases`] for rows that also request float conversion.
fn bench_scalar_hyperbolic_cases(c: &mut Criterion) {
    let mut group = c.benchmark_group("scalar_hyperbolic_cases");
    let numerica_ctx = numerica_engine::Ctx::new(128);
    let gmp_ctx = gmp_engine::Ctx::new(128);
    let symbolica_ctx = symbolica_engine::Ctx::new(128);

    for (name, value) in [
        ("half", 0.5),
        ("negative_tiny", -1.0e-12),
        ("positive_20", 20.0),
        ("negative_20", -20.0),
    ] {
        let hyperreal = s(value);
        let hyperreal_rational = qr(value);
        let numerica = numerica_ctx.f(value);
        let gmp = gmp_ctx.f(value);
        let symbolica = symbolica_ctx.f(value);

        for operation in ["sinh", "cosh", "tanh"] {
            group.bench_function(format!("hyperreal/{operation}/{name}"), |b| {
                b.iter(|| {
                    let value = black_box(hyperreal.clone());
                    black_box(
                        match operation {
                            "sinh" => Real::sinh(value),
                            "cosh" => Real::cosh(value),
                            _ => Real::tanh(value),
                        }
                        .unwrap(),
                    )
                })
            });
            group.bench_function(format!("hyperreal-rational/{operation}/{name}"), |b| {
                b.iter(|| {
                    let value = black_box(hyperreal_rational.clone());
                    black_box(
                        match operation {
                            "sinh" => Real::sinh(value),
                            "cosh" => Real::cosh(value),
                            _ => Real::tanh(value),
                        }
                        .unwrap(),
                    )
                })
            });
            group.bench_function(format!("numerica128/{operation}/{name}"), |b| {
                b.iter(|| {
                    black_box(match operation {
                        "sinh" => numerica_ctx.sinh(black_box(&numerica)),
                        "cosh" => numerica_ctx.cosh(black_box(&numerica)),
                        _ => numerica_ctx.tanh(black_box(&numerica)),
                    })
                })
            });
            group.bench_function(format!("gmp_mpfr128/{operation}/{name}"), |b| {
                b.iter(|| {
                    black_box(match operation {
                        "sinh" => gmp_ctx.sinh(black_box(&gmp)),
                        "cosh" => gmp_ctx.cosh(black_box(&gmp)),
                        _ => gmp_ctx.tanh(black_box(&gmp)),
                    })
                })
            });
            group.bench_function(format!("symbolica/{operation}/{name}"), |b| {
                b.iter(|| {
                    black_box(match operation {
                        "sinh" => symbolica_ctx.sinh(black_box(&symbolica)),
                        "cosh" => symbolica_ctx.cosh(black_box(&symbolica)),
                        _ => symbolica_ctx.tanh(black_box(&symbolica)),
                    })
                })
            });
        }
    }
    group.finish();
}

/// Registers forty-eight hyperbolic rows including explicit lossy f64 output.
///
/// Uses the same four values and three operations as
/// [`bench_scalar_hyperbolic_cases`], but only the two native models and the
/// Symbolica Float/Numerica and Rug/MPFR adapters. The Symbolica Atom backend is
/// absent. Inputs and 128-precision external contexts are retained outside timing.
/// Native iterations clone, construct the operation, and unwrap `to_f64_lossy`;
/// external iterations borrow the input, compute, and call the Float's `to_f64`.
/// Thus conversion and intermediate destruction are measured alongside arithmetic.
///
/// Native scalar errors or failed float conversion panic through unwrap. A finite
/// rounded float is an approximate view, not an exact scalar or certainty receipt;
/// no error bound, inter-backend equality or correct-rounding assertion is checked
/// here. The group uses ordinary `iter` and inherited Criterion settings, without
/// trace hooks, cancellation or a separate total-work/memory limit.
fn bench_scalar_hyperbolic_f64_cases(c: &mut Criterion) {
    let mut group = c.benchmark_group("scalar_hyperbolic_f64_cases");
    let numerica_ctx = numerica_engine::Ctx::new(128);
    let gmp_ctx = gmp_engine::Ctx::new(128);

    for (name, value) in [
        ("half", 0.5),
        ("negative_tiny", -1.0e-12),
        ("positive_20", 20.0),
        ("negative_20", -20.0),
    ] {
        let hyperreal = s(value);
        let hyperreal_rational = qr(value);
        let numerica = numerica_ctx.f(value);
        let gmp = gmp_ctx.f(value);

        for operation in ["sinh", "cosh", "tanh"] {
            group.bench_function(format!("hyperreal/{operation}/{name}"), |b| {
                b.iter(|| {
                    let value = black_box(hyperreal.clone());
                    let result = match operation {
                        "sinh" => Real::sinh(value),
                        "cosh" => Real::cosh(value),
                        _ => Real::tanh(value),
                    }
                    .unwrap();
                    black_box(result.to_f64_lossy().unwrap())
                })
            });
            group.bench_function(format!("hyperreal-rational/{operation}/{name}"), |b| {
                b.iter(|| {
                    let value = black_box(hyperreal_rational.clone());
                    let result = match operation {
                        "sinh" => Real::sinh(value),
                        "cosh" => Real::cosh(value),
                        _ => Real::tanh(value),
                    }
                    .unwrap();
                    black_box(result.to_f64_lossy().unwrap())
                })
            });
            group.bench_function(format!("numerica128/{operation}/{name}"), |b| {
                b.iter(|| {
                    let result = match operation {
                        "sinh" => numerica_ctx.sinh(black_box(&numerica)),
                        "cosh" => numerica_ctx.cosh(black_box(&numerica)),
                        _ => numerica_ctx.tanh(black_box(&numerica)),
                    };
                    black_box(result.to_f64())
                })
            });
            group.bench_function(format!("gmp_mpfr128/{operation}/{name}"), |b| {
                b.iter(|| {
                    let result = match operation {
                        "sinh" => gmp_ctx.sinh(black_box(&gmp)),
                        "cosh" => gmp_ctx.cosh(black_box(&gmp)),
                        _ => gmp_ctx.tanh(black_box(&gmp)),
                    };
                    black_box(result.to_f64())
                })
            });
        }
    }
    group.finish();
}

/// Registers five retained-input exp(128) rows and two native diagnostic hooks.
///
/// Native inputs use exact binary 128 and the ratio 128/1, which denote the same
/// integer. External contexts receive argument 128 and import that integer before
/// timing. Ordinary iterations clone the native input and unwrap exp, or borrow
/// it for the adapter, then destroy the result. Each enabled/selected native trace
/// hook constructs exp once before timing, allowing retained evidence to be reused.
///
/// This isolates large-integer exponential construction/computation by backend;
/// it does not request conversion of exact/lazy or Atom results to a numeric
/// approximation, enforce common precision, or assert values/dispatch counts.
/// Inherited Criterion settings apply, with no cancellation or total-work bound.
fn bench_large_integer_exp(c: &mut Criterion) {
    let mut group = c.benchmark_group("scalar_large_integer_exp");
    let hyperreal = s(128.0);
    let hyperreal_rational = q(128, 1);
    let numerica_ctx = numerica_engine::Ctx::new(128);
    let numerica = numerica_ctx.f(128.0);
    let gmp_ctx = gmp_engine::Ctx::new(128);
    let gmp = gmp_ctx.f(128.0);
    let symbolica_ctx = symbolica_engine::Ctx::new(128);
    let symbolica = symbolica_ctx.f(128.0);

    trace_dispatch_row("scalar_large_integer_exp/hyperreal/exp_128", || {
        black_box(Real::exp(hyperreal.clone()).unwrap());
    });
    trace_dispatch_row(
        "scalar_large_integer_exp/hyperreal-rational/exp_128",
        || {
            black_box(Real::exp(hyperreal_rational.clone()).unwrap());
        },
    );

    group.bench_function("hyperreal/exp_128", |b| {
        b.iter(|| black_box(Real::exp(black_box(hyperreal.clone())).unwrap()))
    });
    group.bench_function("hyperreal-rational/exp_128", |b| {
        b.iter(|| black_box(Real::exp(black_box(hyperreal_rational.clone())).unwrap()))
    });
    group.bench_function("numerica128/exp_128", |b| {
        b.iter(|| black_box(numerica_ctx.exp(black_box(&numerica))))
    });
    group.bench_function("gmp_mpfr128/exp_128", |b| {
        b.iter(|| black_box(gmp_ctx.exp(black_box(&gmp))))
    });
    group.bench_function("symbolica/exp_128", |b| {
        b.iter(|| black_box(symbolica_ctx.exp(black_box(&symbolica))))
    });
    group.finish();
}

/// Registers forty-one scalar rows through the local Symbolica Float adapter.
///
/// The Numerica label uses `numerica_engine` and requests precision 128, not a
/// separate Numerica crate. Four-case families parallel the native schedule but
/// import binary-float values; even the tiny perturbation of 1000 PI is already
/// rounded away. Constants are constructed inside timing; other inputs are retained.
/// Ordinary `iter` times selection, borrowed adapter calls (including internal
/// cloning/allocation) and output destruction, without native trace hooks.
///
/// Reciprocal/checked/checked_abort all call the same reciprocal method; negative
/// exponent -1 also calls reciprocal. Abort-suffixed transcendental rows use their
/// ordinary methods without a signal. Both zero_status names return Boolean
/// `is_zero`, not native three-way ZeroKnowledge; zero_status_abort has no native
/// row. Adapter log10 constructs ln(x)/ln(10), and atan uses atan2(x,1). No native
/// guards, typed Results, policy receipts or numerical/dispatch assertions apply.
fn bench_numerica_scalar_operations(
    group: &mut BenchmarkGroup<'_, criterion::measurement::WallTime>,
    label: &str,
) {
    let ctx = numerica_engine::Ctx::new(128);
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
    group.bench_function(format!("{label}/powi_negative_one"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| black_box(ctx.reciprocal(black_box(next_case(&reciprocal_cases, &cursor)))))
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

/// Registers forty-one scalar rows through Rug/MPFR with requested precision 128.
///
/// Binary-float fixture families are retained outside timing; ordinary `iter`
/// includes selection, borrowed adapter calls with their internal clones, and
/// result destruction. Constant rows construct their values inside timing.
/// Reciprocal/checked/checked_abort all use ordinary reciprocal, as does negative
/// exponent -1. Other abort names call ordinary methods without cancellation flags.
/// Both zero_status names call Boolean `is_zero`; zero_status_abort is additional
/// to the native schedule. GMP uses its native log10 and atan adapter methods.
///
/// These labels do not provide native guards, typed Results, three-way zero facts,
/// policy receipts or trace hooks. No numerical equality/accuracy or dispatch-count
/// assertion establishes equivalence with the native or other external workloads.
fn bench_gmp_scalar_operations(
    group: &mut BenchmarkGroup<'_, criterion::measurement::WallTime>,
    label: &str,
) {
    let ctx = gmp_engine::Ctx::new(128);
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
    group.bench_function(format!("{label}/powi_negative_one"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| black_box(ctx.reciprocal(black_box(next_case(&reciprocal_cases, &cursor)))))
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

/// Registers forty-one scalar rows constructing Symbolica Atom expressions.
///
/// Binary-float fixtures enter through `Atom::num` before timing; the stored 128
/// context argument is not used as a numeric precision setting by these methods.
/// The e/pi constant rows also import binary-float constants, and tau multiplies
/// the imported pi by two, unlike the native symbolic constants. Ordinary `iter`
/// times selection, borrowed adapter operations and destruction without requesting
/// a numeric evaluation or common precision. Constant construction is timed.
///
/// Reciprocal/checked/checked_abort and exponent -1 use ordinary reciprocal.
/// Abort-suffixed transcendental rows use ordinary methods without signals. The
/// adapter constructs log10 as a log quotient, tan as sin/cos, hyperbolics through
/// exponentials and inverse functions through log/radical formulas, including
/// complex intermediates. Branch/value agreement is not asserted by these rows.
/// Both zero_status names compare with the zero Atom and return bool, not native
/// ZeroKnowledge; zero_status_abort has no native counterpart. No native guards,
/// typed Results, certainty receipts, trace hooks or correctness assertions apply.
fn bench_symbolica_scalar_operations(
    group: &mut BenchmarkGroup<'_, criterion::measurement::WallTime>,
    label: &str,
) {
    let ctx = symbolica_engine::Ctx::new(128);
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
    group.bench_function(format!("{label}/powi_negative_one"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| black_box(ctx.reciprocal(black_box(next_case(&reciprocal_cases, &cursor)))))
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
