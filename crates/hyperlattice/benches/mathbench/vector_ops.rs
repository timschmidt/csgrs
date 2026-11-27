/// Register 31 native vector rows for one scalar-conversion label: 18 vec3 and 13 vec4.
///
/// Builds four-case dense/sparse left/right arrays and four scalar divisors before
/// timing. make_scalar receives binary64 fixtures; s preserves their exact binary
/// values, while qr authors rationals from displayed decimals. Arbitrary callbacks
/// can invalidate nonzero/domain assumptions. Four-dimensional lanes are Euclidean
/// coordinates here, including w; no homogeneous projection or point validation runs.
///
/// | Rows | Actual operation and cancellation flag |
/// | --- | --- |
/// | vec3 new, zero | Convert rotating raw coordinates during timing, or construct zero. |
/// | vec3 dot_abort | Dense paired dot with a clear flag. |
/// | vec3/vec4 dot_sparse | Paired dot over the sparse corpus, including zero vectors. |
/// | vec3/vec4 dot_abort_sparse | Sparse paired dot with the flag already set. |
/// | vec3/vec4 dot_abort_dense | Dense paired dot with the flag already set. |
/// | vec3 magnitude_abort | Magnitude with a clear flag; unwrap the result. |
/// | vec3 normalize_checked | Checked normalization of dense nonzero fixtures; unwrap. |
/// | vec3 normalize_checked_abort | Checked normalization with a clear flag; unwrap. |
/// | vec3 div_scalar_checked, div_scalar_checked_abort | Checked scalar division, with a clear flag in the abort variant; clone operands and unwrap. |
/// | vec4 dot, magnitude, normalize | Ordinary dense Euclidean operations; unwrap magnitude/normalization. |
/// | vec3/vec4 add, sub, neg | Owned componentwise operations on cloned retained inputs. |
/// | vec3/vec4 add_scalar, sub_scalar, mul_scalar, div_scalar | Componentwise scalar operations on cloned operands; unwrap division. |
///
/// The two flags are created outside timing and never toggled during iteration.
/// A set flag exercises cooperative attachment/dispatch; dot_with_abort returns a
/// Real and exact shortcuts can still finish. No cancellation outcome or subsequent
/// forced evaluation is asserted. Sparse zero vectors are used for dot work, not
/// normalization. Errors at unwrap/expect panic, without a policy certainty receipt.
///
/// Each nonconstant row rotates its four inputs with a per-callback cursor. Ordinary
/// iter timing includes selection, cursor updates, explicit clones, operation work
/// and returned-output destruction. Fixture setup is excluded except for vec3 new;
/// retained scalar evidence can be reused. No value/path/count oracle or independent
/// total-work/memory bound is supplied.
///
/// Eight optional trace hooks precede measured registration: sparse dot, active-flag
/// sparse/dense dot in both dimensions, then vec3 checked normalization with and
/// without the clear flag. Each loops once over four retained cases. These are
/// untimed diagnostic passes, not a complete trace of every measured row.
fn bench_vector_operations_for<F>(
    group: &mut BenchmarkGroup<'_, criterion::measurement::WallTime>,
    label: &str,
    make_scalar: F,
) where
    F: Copy + Fn(f64) -> Real,
{
    let lhs3_cases = sample_vec3_cases().map(|value| blas_vec3_with(value, make_scalar));
    let rhs3_cases = sample_vec3_b_cases().map(|value| blas_vec3_with(value, make_scalar));
    let lhs4_cases = sample_vec4_cases().map(|value| blas_vec4_with(value, make_scalar));
    let rhs4_cases = sample_vec4_b_cases().map(|value| blas_vec4_with(value, make_scalar));
    let sparse_lhs3_cases =
        sample_vec3_sparse_cases().map(|value| blas_vec3_with(value, make_scalar));
    let sparse_rhs3_cases =
        sample_vec3_sparse_b_cases().map(|value| blas_vec3_with(value, make_scalar));
    let sparse_lhs4_cases =
        sample_vec4_sparse_cases().map(|value| blas_vec4_with(value, make_scalar));
    let sparse_rhs4_cases =
        sample_vec4_sparse_b_cases().map(|value| blas_vec4_with(value, make_scalar));
    let scalar_cases = [
        make_scalar(2.0),
        make_scalar(1.0e-9),
        make_scalar(-1.0e9),
        make_scalar(std::f64::consts::PI),
    ];
    let signal = abort_signal();
    let active_signal = abort_signal();
    active_signal.store(true, std::sync::atomic::Ordering::Relaxed);

    trace_dispatch_row(format!("vector_ops/{label}/vec3 dot_sparse"), || {
        for index in 0..sparse_lhs3_cases.len() {
            black_box(
                black_box(&sparse_lhs3_cases[index]).dot(black_box(&sparse_rhs3_cases[index])),
            );
        }
    });
    trace_dispatch_row(format!("vector_ops/{label}/vec3 dot_abort_sparse"), || {
        for index in 0..sparse_lhs3_cases.len() {
            black_box(
                black_box(&sparse_lhs3_cases[index])
                    .dot_with_abort(black_box(&sparse_rhs3_cases[index]), &active_signal),
            );
        }
    });
    trace_dispatch_row(format!("vector_ops/{label}/vec3 dot_abort_dense"), || {
        for index in 0..lhs3_cases.len() {
            black_box(
                black_box(&lhs3_cases[index])
                    .dot_with_abort(black_box(&rhs3_cases[index]), &active_signal),
            );
        }
    });
    trace_dispatch_row(format!("vector_ops/{label}/vec4 dot_sparse"), || {
        for index in 0..sparse_lhs4_cases.len() {
            black_box(
                black_box(&sparse_lhs4_cases[index]).dot(black_box(&sparse_rhs4_cases[index])),
            );
        }
    });
    trace_dispatch_row(format!("vector_ops/{label}/vec4 dot_abort_sparse"), || {
        for index in 0..sparse_lhs4_cases.len() {
            black_box(
                black_box(&sparse_lhs4_cases[index])
                    .dot_with_abort(black_box(&sparse_rhs4_cases[index]), &active_signal),
            );
        }
    });
    trace_dispatch_row(format!("vector_ops/{label}/vec4 dot_abort_dense"), || {
        for index in 0..lhs4_cases.len() {
            black_box(
                black_box(&lhs4_cases[index])
                    .dot_with_abort(black_box(&rhs4_cases[index]), &active_signal),
            );
        }
    });
    trace_dispatch_row(format!("vector_ops/{label}/vec3 normalize_checked"), || {
        for value in &lhs3_cases {
            black_box(
                black_box(value)
                    .normalize_checked()
                    .expect("benchmark vectors have known nonzero norms"),
            );
        }
    });
    trace_dispatch_row(
        format!("vector_ops/{label}/vec3 normalize_checked_abort"),
        || {
            for value in &lhs3_cases {
                black_box(
                    black_box(value)
                        .normalize_checked_with_abort(&signal)
                        .expect("benchmark vectors have known nonzero norms"),
                );
            }
        },
    );

    group.bench_function(format!("{label}/vec3 new"), |b| {
        let raw_cases = sample_vec3_cases();
        let cursor = Cell::new(0);
        b.iter(|| black_box(blas_vec3_with(*next_case(&raw_cases, &cursor), make_scalar)))
    });
    group.bench_function(format!("{label}/vec3 zero"), |b| {
        b.iter(|| black_box(Vector3::zero()))
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
    group.bench_function(format!("{label}/vec3 dot_sparse"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % sparse_lhs3_cases.len());
            black_box(
                black_box(&sparse_lhs3_cases[index]).dot(black_box(&sparse_rhs3_cases[index])),
            )
        })
    });
    group.bench_function(format!("{label}/vec3 dot_abort_sparse"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % sparse_lhs3_cases.len());
            black_box(
                black_box(&sparse_lhs3_cases[index])
                    .dot_with_abort(black_box(&sparse_rhs3_cases[index]), &active_signal),
            )
        })
    });
    group.bench_function(format!("{label}/vec3 dot_abort_dense"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % lhs3_cases.len());
            black_box(
                black_box(&lhs3_cases[index])
                    .dot_with_abort(black_box(&rhs3_cases[index]), &active_signal),
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
    group.bench_function(format!("{label}/vec4 dot_sparse"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % sparse_lhs4_cases.len());
            black_box(
                black_box(&sparse_lhs4_cases[index]).dot(black_box(&sparse_rhs4_cases[index])),
            )
        })
    });
    group.bench_function(format!("{label}/vec4 dot_abort_sparse"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % sparse_lhs4_cases.len());
            black_box(
                black_box(&sparse_lhs4_cases[index])
                    .dot_with_abort(black_box(&sparse_rhs4_cases[index]), &active_signal),
            )
        })
    });
    group.bench_function(format!("{label}/vec4 dot_abort_dense"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % lhs4_cases.len());
            black_box(
                black_box(&lhs4_cases[index])
                    .dot_with_abort(black_box(&rhs4_cases[index]), &active_signal),
            )
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

/// Register vector_ops for two native input models and three external adapters.
///
/// Native s/qr routes register 31 rows each; Numerica, Rug/GMP/MPFR and Symbolica
/// routes register 25 each, for 137 total. Each external set omits the six native
/// sparse/active-dot rows. All group sample/time settings come from Criterion.
/// Shared names do not certify equal numeric inputs, output materialization,
/// cancellation behavior, checked guards, arithmetic precision or completed work.
/// This family adds no cross-backend result comparison or performance assertion.
fn bench_vector_operations(c: &mut Criterion) {
    let mut group = c.benchmark_group("vector_ops");
    bench_vector_operations_for::<_>(&mut group, "hyperreal", s);
    bench_vector_operations_for::<_>(&mut group, "hyperreal-rational", qr);
    bench_numerica_vector_operations(&mut group, "numerica128");
    bench_gmp_vector_operations(&mut group, "gmp_mpfr128");
    bench_symbolica_vector_operations(&mut group, "symbolica");
    group.finish();
}

/// Register 25 float-adapter rows under the Numerica comparison label.
///
/// The local adapter uses Symbolica's Float backend with requested precision 128.
/// Four retained dense left/right vectors per dimension and four scalars originate
/// from binary64; no sparse fixtures or diagnostic trace hooks are registered here.
/// Except for constructors, rows rotate retained inputs and call borrowed adapter
/// methods. Internal scalar cloning/allocation and ordinary iter output destruction
/// remain measured; no numerical or accuracy assertions are made.
///
/// Fifteen vec3 names cover new, zero, dot_abort, magnitude_abort, both checked
/// normalization labels and nine arithmetic/scalar-division labels. Ten vec4 names
/// cover dot, magnitude, normalize and seven arithmetic/scalar operations. Abort
/// names do not pass a flag; checked names reuse ordinary normalization or division
/// without native structural guards or typed Results.
///
/// vec3 magnitude_abort returns Vec3(magnitude,0,0), while vec4 dot/magnitude return
/// Vec4(result,0,0,0) so each branch has the vector output type. Those extra zero
/// constructions and vector destruction are measured and differ from native scalar
/// outputs. The remaining operations return their adapter's natural scalar/vector
/// carrier; these rows do not establish the same completed contract as native rows.
fn bench_numerica_vector_operations(
    group: &mut BenchmarkGroup<'_, criterion::measurement::WallTime>,
    label: &str,
) {
    let ctx = numerica_engine::Ctx::new(128);
    let lhs3_cases = sample_vec3_cases()
        .map(|value| numerica_engine::Vec3::new(&ctx, value.x, value.y, value.z));
    let rhs3_cases = sample_vec3_b_cases()
        .map(|value| numerica_engine::Vec3::new(&ctx, value.x, value.y, value.z));
    let lhs4_cases = sample_vec4_cases()
        .map(|value| numerica_engine::Vec4::new(&ctx, value.x, value.y, value.z, value.w));
    let rhs4_cases = sample_vec4_b_cases()
        .map(|value| numerica_engine::Vec4::new(&ctx, value.x, value.y, value.z, value.w));
    let scalar_cases = [2.0, 1.0e-9, -1.0e9, std::f64::consts::PI].map(|value| ctx.f(value));

    group.bench_function(format!("{label}/vec3 new"), |b| {
        let raw_cases = sample_vec3_cases();
        let cursor = Cell::new(0);
        b.iter(|| {
            let value = *next_case(&raw_cases, &cursor);
            black_box(numerica_engine::Vec3::new(&ctx, value.x, value.y, value.z))
        })
    });
    group.bench_function(format!("{label}/vec3 zero"), |b| {
        b.iter(|| black_box(numerica_engine::Vec3::zero(&ctx)))
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
                    numerica_engine::Vec3 {
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
    for name in [
        "add",
        "add_scalar",
        "sub",
        "sub_scalar",
        "neg",
        "mul_scalar",
        "div_scalar_checked",
        "div_scalar_checked_abort",
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
                        numerica_engine::Vec4 {
                            x: dot,
                            y: ctx.zero(),
                            z: ctx.zero(),
                            w: ctx.zero(),
                        }
                    }
                    "magnitude" => {
                        let magnitude = lhs4_cases[index].magnitude(&ctx);
                        numerica_engine::Vec4 {
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

/// Register the 25 dense-vector comparison rows using Rug/GMP/MPFR at precision 128.
///
/// Uses the same fixed binary64-origin shapes and name schedule as the Numerica
/// wrapper: fifteen vec3 rows and ten vec4 rows, with no sparse or trace rows. It
/// does not restore authored decimal/symbolic input values from those floats.
/// Per-row cursors, borrowed retained operands, adapter work and output destruction
/// are inside ordinary iter timing; vec3 new includes coordinate construction.
///
/// Abort-labeled rows pass no cancellation flag, and checked labels call ordinary
/// methods. vec3 magnitude_abort packages its scalar into Vec3(magnitude,0,0);
/// vec4 dot and magnitude package into Vec4(result,0,0,0), adding zero-lane and
/// carrier work absent from native scalar outputs. Results are black-boxed without
/// value/error/precision assertions or a common-contract guarantee.
fn bench_gmp_vector_operations(
    group: &mut BenchmarkGroup<'_, criterion::measurement::WallTime>,
    label: &str,
) {
    let ctx = gmp_engine::Ctx::new(128);
    let lhs3_cases =
        sample_vec3_cases().map(|value| gmp_engine::Vec3::new(&ctx, value.x, value.y, value.z));
    let rhs3_cases =
        sample_vec3_b_cases().map(|value| gmp_engine::Vec3::new(&ctx, value.x, value.y, value.z));
    let lhs4_cases = sample_vec4_cases()
        .map(|value| gmp_engine::Vec4::new(&ctx, value.x, value.y, value.z, value.w));
    let rhs4_cases = sample_vec4_b_cases()
        .map(|value| gmp_engine::Vec4::new(&ctx, value.x, value.y, value.z, value.w));
    let scalar_cases = [2.0, 1.0e-9, -1.0e9, std::f64::consts::PI].map(|value| ctx.f(value));

    group.bench_function(format!("{label}/vec3 new"), |b| {
        let raw_cases = sample_vec3_cases();
        let cursor = Cell::new(0);
        b.iter(|| {
            let value = *next_case(&raw_cases, &cursor);
            black_box(gmp_engine::Vec3::new(&ctx, value.x, value.y, value.z))
        })
    });
    group.bench_function(format!("{label}/vec3 zero"), |b| {
        b.iter(|| black_box(gmp_engine::Vec3::zero(&ctx)))
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
                    gmp_engine::Vec3 {
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
    for name in [
        "add",
        "add_scalar",
        "sub",
        "sub_scalar",
        "neg",
        "mul_scalar",
        "div_scalar_checked",
        "div_scalar_checked_abort",
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
                        gmp_engine::Vec4 {
                            x: dot,
                            y: ctx.zero(),
                            z: ctx.zero(),
                            w: ctx.zero(),
                        }
                    }
                    "magnitude" => {
                        let magnitude = lhs4_cases[index].magnitude(&ctx);
                        gmp_engine::Vec4 {
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

/// Register the 25 dense-vector comparison rows using the local Symbolica Atom adapter.
///
/// Stores context argument 128, but vector construction/arithmetic use Atom::num(f64)
/// and Atom operations rather than that field as a precision setting. Inputs remain
/// float-origin; this does not certify a 128-bit evaluation or native Real equality.
/// Initialization/license behavior is owned by the driver and dependency.
///
/// The fifteen vec3 and ten vec4 names follow the other external wrappers, omitting
/// native sparse/active-dot rows. Abort labels carry no flag; checked labels reuse
/// ordinary methods without native guards or typed Results. Magnitude in the vec3
/// abort-named row and dot/magnitude in vec4 are embedded in vectors with remaining
/// lanes zero, and that construction/destruction is part of timing. Other rows
/// rotate borrowed retained inputs, with conversion included in vec3 new. No trace,
/// numerical-result, certification or equal-work assertion is added.
fn bench_symbolica_vector_operations(
    group: &mut BenchmarkGroup<'_, criterion::measurement::WallTime>,
    label: &str,
) {
    let ctx = symbolica_engine::Ctx::new(128);
    let lhs3_cases = sample_vec3_cases()
        .map(|value| symbolica_engine::Vec3::new(&ctx, value.x, value.y, value.z));
    let rhs3_cases = sample_vec3_b_cases()
        .map(|value| symbolica_engine::Vec3::new(&ctx, value.x, value.y, value.z));
    let lhs4_cases = sample_vec4_cases()
        .map(|value| symbolica_engine::Vec4::new(&ctx, value.x, value.y, value.z, value.w));
    let rhs4_cases = sample_vec4_b_cases()
        .map(|value| symbolica_engine::Vec4::new(&ctx, value.x, value.y, value.z, value.w));
    let scalar_cases = [2.0, 1.0e-9, -1.0e9, std::f64::consts::PI].map(|value| ctx.f(value));

    group.bench_function(format!("{label}/vec3 new"), |b| {
        let raw_cases = sample_vec3_cases();
        let cursor = Cell::new(0);
        b.iter(|| {
            let value = *next_case(&raw_cases, &cursor);
            black_box(symbolica_engine::Vec3::new(&ctx, value.x, value.y, value.z))
        })
    });
    group.bench_function(format!("{label}/vec3 zero"), |b| {
        b.iter(|| black_box(symbolica_engine::Vec3::zero(&ctx)))
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
                    symbolica_engine::Vec3 {
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
    for name in [
        "add",
        "add_scalar",
        "sub",
        "sub_scalar",
        "neg",
        "mul_scalar",
        "div_scalar_checked",
        "div_scalar_checked_abort",
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
                        symbolica_engine::Vec4 {
                            x: dot,
                            y: ctx.zero(),
                            z: ctx.zero(),
                            w: ctx.zero(),
                        }
                    }
                    "magnitude" => {
                        let magnitude = lhs4_cases[index].magnitude(&ctx);
                        symbolica_engine::Vec4 {
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
