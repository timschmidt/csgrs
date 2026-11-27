/// Registers sixty native rows for borrowed arithmetic and selected ownership mixes.
///
/// Four retained cases per operand family are constructed through `make_scalar`
/// before timing. The caller supplies either exact binary-float ingress (`s`) or
/// displayed-decimal rational authoring (`qr`); an arbitrary converter need not
/// preserve the nonzero divisors or invertible matrices intended by the fixtures.
/// Rows cover twenty scalar ownership/operation combinations, seven operations
/// per vector dimension, nine per matrix dimension, two matrix-vector products,
/// and six complex operations. Matrix division is the right quotient `A * B⁻¹`;
/// vector/matrix scalar arithmetic acts on every component, and four-dimensional
/// matrix-vector products retain all four lanes without projective division.
///
/// Scalar `owned_ref` and `ref_owned` use `iter_batched` with `SmallInput`:
/// selection, cursor advancement and cloning the owned operand occur in setup,
/// outside the measured routine, as does destruction of the returned result.
/// Their `_with_clone` counterparts use ordinary `iter`, which times selection,
/// cursor advancement, the explicit clone, arithmetic and result destruction.
/// The difference between these rows therefore includes more than clone cost.
/// Scalar `refs` borrows both operands. All vector/matrix scalar rows clone the
/// left operand inside timing and borrow the scalar; complex `div_real_ref` does
/// likewise. Other vector, matrix and complex rows use borrowed operands. All
/// these remaining rows use ordinary `iter` over the retained cases, allowing
/// source/evidence reuse rather than reconstructing cold inputs each iteration.
///
/// Thirty diagnostic trace rows precede their associated measurements: four
/// scalar, eight vector, twelve matrix, two matrix-vector and four complex rows.
/// Each visits the four cases; add/subtract hooks repeat each case three times,
/// whereas other hooks perform one operation per case. These repetitions belong
/// to tracing, not to each timed iteration. There are no value, dispatch-count or
/// certainty-receipt assertions, cancellation signals, or total-work/memory limits.
/// Native fallible divisions are unwrapped and panic if a fixture/converter makes
/// them fail. This is a timing schedule, not exhaustive ownership-API qualification.
fn bench_borrowed_operations_for<F>(
    group: &mut BenchmarkGroup<'_, criterion::measurement::WallTime>,
    label: &str,
    make_scalar: F,
) where
    F: Copy + Fn(f64) -> Real,
{
    let scalar_pairs = [
        (make_scalar(2.5), make_scalar(1.25)),
        (make_scalar(1.0e-12), make_scalar(-1.0e-12)),
        (make_scalar(1.0e9), make_scalar(1.0e-9)),
        (make_scalar(-2.75), make_scalar(0.125)),
    ];
    let scalar_cases = [
        make_scalar(2.0),
        make_scalar(1.0e-9),
        make_scalar(-1.0e9),
        make_scalar(std::f64::consts::PI),
    ];
    let vec3_lhs = sample_vec3_cases().map(|value| blas_vec3_with(value, make_scalar));
    let vec3_rhs = sample_vec3_b_cases().map(|value| blas_vec3_with(value, make_scalar));
    let vec4_lhs = sample_vec4_cases().map(|value| blas_vec4_with(value, make_scalar));
    let vec4_rhs = sample_vec4_b_cases().map(|value| blas_vec4_with(value, make_scalar));
    let mat3_lhs = sample_mat3_cases().map(|value| blas_mat3_with(value, make_scalar));
    let mat3_rhs = sample_mat3_b_cases().map(|value| blas_mat3_with(value, make_scalar));
    let mat4_lhs = sample_mat4_cases().map(|value| blas_mat4_with(value, make_scalar));
    let mat4_rhs = sample_mat4_b_cases().map(|value| blas_mat4_with(value, make_scalar));
    let complex_lhs = [
        Complex::new(make_scalar(3.0), make_scalar(4.0)),
        Complex::new(make_scalar(1.0e-9), make_scalar(-1.0e-9)),
        Complex::new(make_scalar(1.0e9), make_scalar(-1.0)),
        Complex::new(
            make_scalar(std::f64::consts::PI),
            make_scalar(-std::f64::consts::E),
        ),
    ];
    let complex_rhs = [
        Complex::new(make_scalar(1.5), make_scalar(-2.0)),
        Complex::new(make_scalar(-1.0e-9), make_scalar(2.0e-9)),
        Complex::new(make_scalar(-1.0e9), make_scalar(2.0)),
        Complex::new(
            make_scalar(std::f64::consts::SQRT_2),
            make_scalar(std::f64::consts::FRAC_1_PI),
        ),
    ];

    for name in ["add", "sub", "mul", "div"] {
        trace_dispatch_row(format!("borrowed_ops/{label}/scalar {name} refs"), || {
            for (lhs, rhs) in &scalar_pairs {
                let repetitions = if matches!(name, "add" | "sub") { 3 } else { 1 };
                for _ in 0..repetitions {
                    match name {
                        "add" => black_box(black_box(lhs) + black_box(rhs)),
                        "sub" => black_box(black_box(lhs) - black_box(rhs)),
                        "mul" => black_box(black_box(lhs) * black_box(rhs)),
                        _ => black_box((black_box(lhs) / black_box(rhs)).unwrap()),
                    };
                }
            }
        });
        group.bench_function(format!("{label}/scalar {name} owned_ref"), |b| {
            let cursor = Cell::new(0);
            b.iter_batched(
                || {
                    let (lhs, rhs) = next_case(&scalar_pairs, &cursor);
                    (lhs.clone(), rhs)
                },
                |(lhs, rhs)| match name {
                    "add" => black_box(black_box(lhs) + black_box(rhs)),
                    "sub" => black_box(black_box(lhs) - black_box(rhs)),
                    "mul" => black_box(black_box(lhs) * black_box(rhs)),
                    _ => black_box((black_box(lhs) / black_box(rhs)).unwrap()),
                },
                BatchSize::SmallInput,
            )
        });
        group.bench_function(format!("{label}/scalar {name} ref_owned"), |b| {
            let cursor = Cell::new(0);
            b.iter_batched(
                || {
                    let (lhs, rhs) = next_case(&scalar_pairs, &cursor);
                    (lhs, rhs.clone())
                },
                |(lhs, rhs)| match name {
                    "add" => black_box(black_box(lhs) + black_box(rhs)),
                    "sub" => black_box(black_box(lhs) - black_box(rhs)),
                    "mul" => black_box(black_box(lhs) * black_box(rhs)),
                    _ => black_box((black_box(lhs) / black_box(rhs)).unwrap()),
                },
                BatchSize::SmallInput,
            )
        });
        group.bench_function(format!("{label}/scalar {name} refs"), |b| {
            let cursor = Cell::new(0);
            b.iter(|| {
                let (lhs, rhs) = next_case(&scalar_pairs, &cursor);
                match name {
                    "add" => black_box(black_box(lhs) + black_box(rhs)),
                    "sub" => black_box(black_box(lhs) - black_box(rhs)),
                    "mul" => black_box(black_box(lhs) * black_box(rhs)),
                    _ => black_box((black_box(lhs) / black_box(rhs)).unwrap()),
                }
            })
        });
        group.bench_function(format!("{label}/scalar {name} owned_ref_with_clone"), |b| {
            let cursor = Cell::new(0);
            b.iter(|| {
                let (lhs, rhs) = next_case(&scalar_pairs, &cursor);
                match name {
                    "add" => black_box(black_box(lhs.clone()) + black_box(rhs)),
                    "sub" => black_box(black_box(lhs.clone()) - black_box(rhs)),
                    "mul" => black_box(black_box(lhs.clone()) * black_box(rhs)),
                    _ => black_box((black_box(lhs.clone()) / black_box(rhs)).unwrap()),
                }
            })
        });
        group.bench_function(format!("{label}/scalar {name} ref_owned_with_clone"), |b| {
            let cursor = Cell::new(0);
            b.iter(|| {
                let (lhs, rhs) = next_case(&scalar_pairs, &cursor);
                match name {
                    "add" => black_box(black_box(lhs) + black_box(rhs.clone())),
                    "sub" => black_box(black_box(lhs) - black_box(rhs.clone())),
                    "mul" => black_box(black_box(lhs) * black_box(rhs.clone())),
                    _ => black_box((black_box(lhs) / black_box(rhs.clone())).unwrap()),
                }
            })
        });
    }

    /// Registers seven native vector rows and four diagnostic traces per invocation.
    ///
    /// Captures `group`, `label` and `scalar_cases`; the dimension literal names
    /// rows, while the paired identifiers supply retained vectors. Both arrays
    /// and scalar cases must support every index of the nonempty left array.
    /// Add/subtract borrow both vectors and negation borrows its input. Scalar
    /// add/subtract/multiply/divide clone the left vector inside each ordinary
    /// timed iteration and borrow the scalar. Division unwraps its result.
    /// Trace add/subtract repeats each pair three times; scalar multiply/divide
    /// traces each pair once. Each timing row has its own cycling cursor.
    macro_rules! bench_vector_borrowed {
        ($dimension:literal, $lhs:ident, $rhs:ident) => {
            trace_dispatch_row(
                format!("borrowed_ops/{label}/{} add refs", $dimension),
                || {
                    for index in 0..$lhs.len() {
                        for _ in 0..3 {
                            black_box(black_box(&$lhs[index]) + black_box(&$rhs[index]));
                        }
                    }
                },
            );
            trace_dispatch_row(
                format!("borrowed_ops/{label}/{} sub refs", $dimension),
                || {
                    for index in 0..$lhs.len() {
                        for _ in 0..3 {
                            black_box(black_box(&$lhs[index]) - black_box(&$rhs[index]));
                        }
                    }
                },
            );
            trace_dispatch_row(
                format!("borrowed_ops/{label}/{} mul_scalar_ref", $dimension),
                || {
                    for index in 0..$lhs.len() {
                        black_box(black_box($lhs[index].clone()) * black_box(&scalar_cases[index]));
                    }
                },
            );
            trace_dispatch_row(
                format!("borrowed_ops/{label}/{} div_scalar_ref", $dimension),
                || {
                    for index in 0..$lhs.len() {
                        black_box(
                            (black_box($lhs[index].clone()) / black_box(&scalar_cases[index]))
                                .unwrap(),
                        );
                    }
                },
            );
            group.bench_function(format!("{label}/{} add refs", $dimension), |b| {
                let cursor = Cell::new(0);
                b.iter(|| {
                    let index = cursor.get();
                    cursor.set((index + 1) % $lhs.len());
                    black_box(black_box(&$lhs[index]) + black_box(&$rhs[index]))
                })
            });
            group.bench_function(format!("{label}/{} sub refs", $dimension), |b| {
                let cursor = Cell::new(0);
                b.iter(|| {
                    let index = cursor.get();
                    cursor.set((index + 1) % $lhs.len());
                    black_box(black_box(&$lhs[index]) - black_box(&$rhs[index]))
                })
            });
            group.bench_function(format!("{label}/{} neg ref", $dimension), |b| {
                let cursor = Cell::new(0);
                b.iter(|| black_box(-black_box(next_case(&$lhs, &cursor))))
            });
            group.bench_function(format!("{label}/{} add_scalar_ref", $dimension), |b| {
                let cursor = Cell::new(0);
                b.iter(|| {
                    let index = cursor.get();
                    cursor.set((index + 1) % $lhs.len());
                    black_box(black_box($lhs[index].clone()) + black_box(&scalar_cases[index]))
                })
            });
            group.bench_function(format!("{label}/{} sub_scalar_ref", $dimension), |b| {
                let cursor = Cell::new(0);
                b.iter(|| {
                    let index = cursor.get();
                    cursor.set((index + 1) % $lhs.len());
                    black_box(black_box($lhs[index].clone()) - black_box(&scalar_cases[index]))
                })
            });
            group.bench_function(format!("{label}/{} mul_scalar_ref", $dimension), |b| {
                let cursor = Cell::new(0);
                b.iter(|| {
                    let index = cursor.get();
                    cursor.set((index + 1) % $lhs.len());
                    black_box(black_box($lhs[index].clone()) * black_box(&scalar_cases[index]))
                })
            });
            group.bench_function(format!("{label}/{} div_scalar_ref", $dimension), |b| {
                let cursor = Cell::new(0);
                b.iter(|| {
                    let index = cursor.get();
                    cursor.set((index + 1) % $lhs.len());
                    black_box(
                        (black_box($lhs[index].clone()) / black_box(&scalar_cases[index])).unwrap(),
                    )
                })
            });
        };
    }

    /// Registers nine native matrix rows and six diagnostic traces per invocation.
    ///
    /// Captures `group`, `label` and `scalar_cases`, using the dimension literal
    /// only in names. Nonempty paired arrays and scalar cases must support every
    /// left-array index. Borrowed matrix arithmetic includes the matrix product
    /// and right quotient `A * B⁻¹`; scalar arithmetic acts on every entry after
    /// cloning the left matrix inside timing. Negation borrows its input, and
    /// both division forms unwrap. Every row uses ordinary `iter` and its own
    /// cycling cursor. Add/subtract trace each pair three times, while matrix
    /// multiply/divide and scalar multiply/divide trace each pair once.
    macro_rules! bench_matrix_borrowed {
        ($dimension:literal, $lhs:ident, $rhs:ident) => {
            trace_dispatch_row(
                format!("borrowed_ops/{label}/{} add refs", $dimension),
                || {
                    for index in 0..$lhs.len() {
                        for _ in 0..3 {
                            black_box(black_box(&$lhs[index]) + black_box(&$rhs[index]));
                        }
                    }
                },
            );
            trace_dispatch_row(
                format!("borrowed_ops/{label}/{} sub refs", $dimension),
                || {
                    for index in 0..$lhs.len() {
                        for _ in 0..3 {
                            black_box(black_box(&$lhs[index]) - black_box(&$rhs[index]));
                        }
                    }
                },
            );
            trace_dispatch_row(
                format!("borrowed_ops/{label}/{} mul refs", $dimension),
                || {
                    for index in 0..$lhs.len() {
                        black_box(black_box(&$lhs[index]) * black_box(&$rhs[index]));
                    }
                },
            );
            trace_dispatch_row(
                format!("borrowed_ops/{label}/{} div refs", $dimension),
                || {
                    for index in 0..$lhs.len() {
                        black_box((black_box(&$lhs[index]) / black_box(&$rhs[index])).unwrap());
                    }
                },
            );
            trace_dispatch_row(
                format!("borrowed_ops/{label}/{} mul_scalar_ref", $dimension),
                || {
                    for index in 0..$lhs.len() {
                        black_box(black_box($lhs[index].clone()) * black_box(&scalar_cases[index]));
                    }
                },
            );
            trace_dispatch_row(
                format!("borrowed_ops/{label}/{} div_scalar_ref", $dimension),
                || {
                    for index in 0..$lhs.len() {
                        black_box(
                            (black_box($lhs[index].clone()) / black_box(&scalar_cases[index]))
                                .unwrap(),
                        );
                    }
                },
            );
            group.bench_function(format!("{label}/{} add refs", $dimension), |b| {
                let cursor = Cell::new(0);
                b.iter(|| {
                    let index = cursor.get();
                    cursor.set((index + 1) % $lhs.len());
                    black_box(black_box(&$lhs[index]) + black_box(&$rhs[index]))
                })
            });
            group.bench_function(format!("{label}/{} sub refs", $dimension), |b| {
                let cursor = Cell::new(0);
                b.iter(|| {
                    let index = cursor.get();
                    cursor.set((index + 1) % $lhs.len());
                    black_box(black_box(&$lhs[index]) - black_box(&$rhs[index]))
                })
            });
            group.bench_function(format!("{label}/{} mul refs", $dimension), |b| {
                let cursor = Cell::new(0);
                b.iter(|| {
                    let index = cursor.get();
                    cursor.set((index + 1) % $lhs.len());
                    black_box(black_box(&$lhs[index]) * black_box(&$rhs[index]))
                })
            });
            group.bench_function(format!("{label}/{} div refs", $dimension), |b| {
                let cursor = Cell::new(0);
                b.iter(|| {
                    let index = cursor.get();
                    cursor.set((index + 1) % $lhs.len());
                    black_box((black_box(&$lhs[index]) / black_box(&$rhs[index])).unwrap())
                })
            });
            group.bench_function(format!("{label}/{} neg ref", $dimension), |b| {
                let cursor = Cell::new(0);
                b.iter(|| black_box(-black_box(next_case(&$lhs, &cursor))))
            });
            group.bench_function(format!("{label}/{} add_scalar_ref", $dimension), |b| {
                let cursor = Cell::new(0);
                b.iter(|| {
                    let index = cursor.get();
                    cursor.set((index + 1) % $lhs.len());
                    black_box(black_box($lhs[index].clone()) + black_box(&scalar_cases[index]))
                })
            });
            group.bench_function(format!("{label}/{} sub_scalar_ref", $dimension), |b| {
                let cursor = Cell::new(0);
                b.iter(|| {
                    let index = cursor.get();
                    cursor.set((index + 1) % $lhs.len());
                    black_box(black_box($lhs[index].clone()) - black_box(&scalar_cases[index]))
                })
            });
            group.bench_function(format!("{label}/{} mul_scalar_ref", $dimension), |b| {
                let cursor = Cell::new(0);
                b.iter(|| {
                    let index = cursor.get();
                    cursor.set((index + 1) % $lhs.len());
                    black_box(black_box($lhs[index].clone()) * black_box(&scalar_cases[index]))
                })
            });
            group.bench_function(format!("{label}/{} div_scalar_ref", $dimension), |b| {
                let cursor = Cell::new(0);
                b.iter(|| {
                    let index = cursor.get();
                    cursor.set((index + 1) % $lhs.len());
                    black_box(
                        (black_box($lhs[index].clone()) / black_box(&scalar_cases[index])).unwrap(),
                    )
                })
            });
        };
    }

    bench_vector_borrowed!("vec3", vec3_lhs, vec3_rhs);
    bench_vector_borrowed!("vec4", vec4_lhs, vec4_rhs);
    bench_matrix_borrowed!("mat3", mat3_lhs, mat3_rhs);
    bench_matrix_borrowed!("mat4", mat4_lhs, mat4_rhs);

    trace_dispatch_row(
        format!("borrowed_ops/{label}/mat3 transform_vec refs"),
        || {
            for index in 0..mat3_lhs.len() {
                black_box(black_box(&mat3_lhs[index]) * black_box(&vec3_lhs[index]));
            }
        },
    );
    group.bench_function(format!("{label}/mat3 transform_vec refs"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % mat3_lhs.len());
            black_box(black_box(&mat3_lhs[index]) * black_box(&vec3_lhs[index]))
        })
    });
    trace_dispatch_row(
        format!("borrowed_ops/{label}/mat4 transform_vec refs"),
        || {
            for index in 0..mat4_lhs.len() {
                black_box(black_box(&mat4_lhs[index]) * black_box(&vec4_lhs[index]));
            }
        },
    );
    group.bench_function(format!("{label}/mat4 transform_vec refs"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % mat4_lhs.len());
            black_box(black_box(&mat4_lhs[index]) * black_box(&vec4_lhs[index]))
        })
    });

    for name in ["add", "sub", "mul", "div"] {
        trace_dispatch_row(format!("borrowed_ops/{label}/complex {name} refs"), || {
            for index in 0..complex_lhs.len() {
                let repetitions = if matches!(name, "add" | "sub") { 3 } else { 1 };
                for _ in 0..repetitions {
                    match name {
                        "add" => black_box(
                            black_box(&complex_lhs[index]) + black_box(&complex_rhs[index]),
                        ),
                        "sub" => black_box(
                            black_box(&complex_lhs[index]) - black_box(&complex_rhs[index]),
                        ),
                        "mul" => black_box(
                            black_box(&complex_lhs[index]) * black_box(&complex_rhs[index]),
                        ),
                        _ => black_box(
                            (black_box(&complex_lhs[index]) / black_box(&complex_rhs[index]))
                                .unwrap(),
                        ),
                    };
                }
            }
        });
        group.bench_function(format!("{label}/complex {name} refs"), |b| {
            let cursor = Cell::new(0);
            b.iter(|| {
                let index = cursor.get();
                cursor.set((index + 1) % complex_lhs.len());
                match name {
                    "add" => {
                        black_box(black_box(&complex_lhs[index]) + black_box(&complex_rhs[index]))
                    }
                    "sub" => {
                        black_box(black_box(&complex_lhs[index]) - black_box(&complex_rhs[index]))
                    }
                    "mul" => {
                        black_box(black_box(&complex_lhs[index]) * black_box(&complex_rhs[index]))
                    }
                    _ => black_box(
                        (black_box(&complex_lhs[index]) / black_box(&complex_rhs[index])).unwrap(),
                    ),
                }
            })
        });
    }
    group.bench_function(format!("{label}/complex neg ref"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| black_box(-black_box(next_case(&complex_lhs, &cursor))))
    });
    group.bench_function(format!("{label}/complex div_real_ref"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % complex_lhs.len());
            black_box(
                (black_box(complex_lhs[index].clone()) / black_box(&scalar_cases[index])).unwrap(),
            )
        })
    });
}

/// Registers and finishes the 300-row `borrowed_ops` Criterion group.
///
/// Each of two native input models and three external adapters contributes sixty
/// rows under the inherited Criterion settings. Native `s` preserves binary-float
/// values; `qr` authors their displayed-decimal rationals, so the inputs need not
/// be equal. External ownership labels do not promise the native ownership or
/// batching behavior: see `bench_external_borrowed_operations`. Only native
/// registrations emit the sixty diagnostic trace rows, distinct from timing.
fn bench_borrowed_operations(c: &mut Criterion) {
    let mut group = c.benchmark_group("borrowed_ops");
    bench_borrowed_operations_for::<_>(&mut group, "hyperreal", s);
    bench_borrowed_operations_for::<_>(&mut group, "hyperreal-rational", qr);
    bench_numerica_borrowed_operations(&mut group, "numerica128");
    bench_gmp_borrowed_operations(&mut group, "gmp_mpfr128");
    bench_symbolica_borrowed_operations(&mut group, "symbolica");
    group.finish();
}

/// Registers sixty adapter rows sharing the native borrowed-operation row names.
///
/// Evaluates the group and label expressions once, creates the selected engine
/// context with argument 128, and retains four cases per operand family before
/// timing. The engine identifier supplies the scalar, vector, matrix and complex
/// methods; its wrapper documents the meaning of that context argument. The
/// schedule has twenty scalar rows, fourteen vector rows, eighteen matrix rows,
/// two matrix-vector products and six complex rows, without native trace hooks.
///
/// Every row uses ordinary `iter`, timing cursor/selection, adapter work and output
/// destruction. Scalar `owned_ref`, `ref_owned` and `refs` all call the same adapter
/// methods with both operands borrowed; their names do not select different Rust
/// ownership signatures. The `_with_clone` rows clone the named side inside timing
/// and then borrow that clone. Unlike native mixed-ownership scalar rows, none use
/// batched setup. Vector/matrix scalar operations and complex real division borrow
/// their receivers without an explicit caller clone; adapter implementations can
/// still clone or allocate internally (matrix scalar mapping clones the entries).
///
/// Matrix addition/subtraction and scalar arithmetic act entrywise; matrix
/// multiplication and right division use `A * B` and `A * B⁻¹`. Matrix-vector
/// products retain all lanes without projective division. Native division guards,
/// typed Results, cancellation, policy receipts and correctness assertions are
/// absent here; backend division behavior applies. Matching row names establish
/// neither identical measured work nor a common exactness/completion contract.
macro_rules! bench_external_borrowed_operations {
    ($engine:ident, $group:expr, $label:expr) => {{
        let group = $group;
        let label = $label;
        let ctx = $engine::Ctx::new(128);
        let scalar_pairs = [
            (ctx.f(2.5), ctx.f(1.25)),
            (ctx.f(1.0e-12), ctx.f(-1.0e-12)),
            (ctx.f(1.0e9), ctx.f(1.0e-9)),
            (ctx.f(-2.75), ctx.f(0.125)),
        ];
        let scalar_cases = [2.0, 1.0e-9, -1.0e9, std::f64::consts::PI].map(|value| ctx.f(value));
        let vec3_lhs =
            sample_vec3_cases().map(|value| $engine::Vec3::new(&ctx, value.x, value.y, value.z));
        let vec3_rhs =
            sample_vec3_b_cases().map(|value| $engine::Vec3::new(&ctx, value.x, value.y, value.z));
        let vec4_lhs = sample_vec4_cases()
            .map(|value| $engine::Vec4::new(&ctx, value.x, value.y, value.z, value.w));
        let vec4_rhs = sample_vec4_b_cases()
            .map(|value| $engine::Vec4::new(&ctx, value.x, value.y, value.z, value.w));
        let mat3_lhs = sample_mat3_cases().map(|value| $engine::Mat3::new(&ctx, value.m));
        let mat3_rhs = sample_mat3_b_cases().map(|value| $engine::Mat3::new(&ctx, value.m));
        let mat4_lhs = sample_mat4_cases().map(|value| $engine::Mat4::new(&ctx, value.m));
        let mat4_rhs = sample_mat4_b_cases().map(|value| $engine::Mat4::new(&ctx, value.m));
        let complex_lhs = [
            $engine::Complex::new(&ctx, 3.0, 4.0),
            $engine::Complex::new(&ctx, 1.0e-9, -1.0e-9),
            $engine::Complex::new(&ctx, 1.0e9, -1.0),
            $engine::Complex::new(&ctx, std::f64::consts::PI, -std::f64::consts::E),
        ];
        let complex_rhs = [
            $engine::Complex::new(&ctx, 1.5, -2.0),
            $engine::Complex::new(&ctx, -1.0e-9, 2.0e-9),
            $engine::Complex::new(&ctx, -1.0e9, 2.0),
            $engine::Complex::new(&ctx, std::f64::consts::SQRT_2, std::f64::consts::FRAC_1_PI),
        ];

        for name in ["add", "sub", "mul", "div"] {
            group.bench_function(format!("{label}/scalar {name} owned_ref"), |b| {
                let cursor = Cell::new(0);
                b.iter(|| {
                    let (lhs, rhs) = next_case(&scalar_pairs, &cursor);
                    black_box(match name {
                        "add" => ctx.add(lhs, rhs),
                        "sub" => ctx.sub(lhs, rhs),
                        "mul" => ctx.mul(lhs, rhs),
                        _ => ctx.div(lhs, rhs),
                    })
                })
            });
            group.bench_function(format!("{label}/scalar {name} ref_owned"), |b| {
                let cursor = Cell::new(0);
                b.iter(|| {
                    let (lhs, rhs) = next_case(&scalar_pairs, &cursor);
                    black_box(match name {
                        "add" => ctx.add(lhs, rhs),
                        "sub" => ctx.sub(lhs, rhs),
                        "mul" => ctx.mul(lhs, rhs),
                        _ => ctx.div(lhs, rhs),
                    })
                })
            });
            group.bench_function(format!("{label}/scalar {name} refs"), |b| {
                let cursor = Cell::new(0);
                b.iter(|| {
                    let (lhs, rhs) = next_case(&scalar_pairs, &cursor);
                    black_box(match name {
                        "add" => ctx.add(lhs, rhs),
                        "sub" => ctx.sub(lhs, rhs),
                        "mul" => ctx.mul(lhs, rhs),
                        _ => ctx.div(lhs, rhs),
                    })
                })
            });
            group.bench_function(format!("{label}/scalar {name} owned_ref_with_clone"), |b| {
                let cursor = Cell::new(0);
                b.iter(|| {
                    let (lhs, rhs) = next_case(&scalar_pairs, &cursor);
                    let lhs = lhs.clone();
                    black_box(match name {
                        "add" => ctx.add(&lhs, rhs),
                        "sub" => ctx.sub(&lhs, rhs),
                        "mul" => ctx.mul(&lhs, rhs),
                        _ => ctx.div(&lhs, rhs),
                    })
                })
            });
            group.bench_function(format!("{label}/scalar {name} ref_owned_with_clone"), |b| {
                let cursor = Cell::new(0);
                b.iter(|| {
                    let (lhs, rhs) = next_case(&scalar_pairs, &cursor);
                    let rhs = rhs.clone();
                    black_box(match name {
                        "add" => ctx.add(lhs, &rhs),
                        "sub" => ctx.sub(lhs, &rhs),
                        "mul" => ctx.mul(lhs, &rhs),
                        _ => ctx.div(lhs, &rhs),
                    })
                })
            });
        }

        /// Registers seven borrowed adapter vector rows for one dimension.
        ///
        /// Captures the enclosing expansion's group, label, context and scalar cases.
        /// The dimension literal supplies names; nonempty paired arrays and scalar
        /// cases must support every left-array index. Add/subtract, negation and four
        /// componentwise scalar operations borrow their inputs without explicit
        /// caller clones, though adapters may clone internally. Each ordinary timed
        /// iteration advances its row's cursor; no diagnostic traces or native
        /// division-result guards are emitted.
        macro_rules! bench_vector_borrowed {
            ($dimension:literal, $lhs:ident, $rhs:ident) => {
                group.bench_function(format!("{label}/{} add refs", $dimension), |b| {
                    let cursor = Cell::new(0);
                    b.iter(|| {
                        let index = cursor.get();
                        cursor.set((index + 1) % $lhs.len());
                        black_box($lhs[index].add(&$rhs[index], &ctx))
                    })
                });
                group.bench_function(format!("{label}/{} sub refs", $dimension), |b| {
                    let cursor = Cell::new(0);
                    b.iter(|| {
                        let index = cursor.get();
                        cursor.set((index + 1) % $lhs.len());
                        black_box($lhs[index].sub(&$rhs[index], &ctx))
                    })
                });
                group.bench_function(format!("{label}/{} neg ref", $dimension), |b| {
                    let cursor = Cell::new(0);
                    b.iter(|| black_box(next_case(&$lhs, &cursor).neg(&ctx)))
                });
                group.bench_function(format!("{label}/{} add_scalar_ref", $dimension), |b| {
                    let cursor = Cell::new(0);
                    b.iter(|| {
                        let index = cursor.get();
                        cursor.set((index + 1) % $lhs.len());
                        black_box($lhs[index].add_scalar(&scalar_cases[index], &ctx))
                    })
                });
                group.bench_function(format!("{label}/{} sub_scalar_ref", $dimension), |b| {
                    let cursor = Cell::new(0);
                    b.iter(|| {
                        let index = cursor.get();
                        cursor.set((index + 1) % $lhs.len());
                        black_box($lhs[index].sub_scalar(&scalar_cases[index], &ctx))
                    })
                });
                group.bench_function(format!("{label}/{} mul_scalar_ref", $dimension), |b| {
                    let cursor = Cell::new(0);
                    b.iter(|| {
                        let index = cursor.get();
                        cursor.set((index + 1) % $lhs.len());
                        black_box($lhs[index].mul_scalar(&scalar_cases[index], &ctx))
                    })
                });
                group.bench_function(format!("{label}/{} div_scalar_ref", $dimension), |b| {
                    let cursor = Cell::new(0);
                    b.iter(|| {
                        let index = cursor.get();
                        cursor.set((index + 1) % $lhs.len());
                        black_box($lhs[index].div_scalar(&scalar_cases[index], &ctx))
                    })
                });
            };
        }

        bench_vector_borrowed!("vec3", vec3_lhs, vec3_rhs);
        bench_vector_borrowed!("vec4", vec4_lhs, vec4_rhs);

        group.bench_function(format!("{label}/mat3 add refs"), |b| {
            let cursor = Cell::new(0);
            b.iter(|| {
                let index = cursor.get();
                cursor.set((index + 1) % mat3_lhs.len());
                black_box(mat3_lhs[index].combine(&mat3_rhs[index], &ctx, $engine::Ctx::add))
            })
        });
        group.bench_function(format!("{label}/mat3 sub refs"), |b| {
            let cursor = Cell::new(0);
            b.iter(|| {
                let index = cursor.get();
                cursor.set((index + 1) % mat3_lhs.len());
                black_box(mat3_lhs[index].combine(&mat3_rhs[index], &ctx, $engine::Ctx::sub))
            })
        });
        group.bench_function(format!("{label}/mat3 mul refs"), |b| {
            let cursor = Cell::new(0);
            b.iter(|| {
                let index = cursor.get();
                cursor.set((index + 1) % mat3_lhs.len());
                black_box(mat3_lhs[index].mul_mat3(&mat3_rhs[index], &ctx))
            })
        });
        group.bench_function(format!("{label}/mat3 div refs"), |b| {
            let cursor = Cell::new(0);
            b.iter(|| {
                let index = cursor.get();
                cursor.set((index + 1) % mat3_lhs.len());
                black_box(mat3_lhs[index].div_matrix(&mat3_rhs[index], &ctx))
            })
        });
        group.bench_function(format!("{label}/mat3 neg ref"), |b| {
            let cursor = Cell::new(0);
            b.iter(|| black_box(next_case(&mat3_lhs, &cursor).neg(&ctx)))
        });
        group.bench_function(format!("{label}/mat3 add_scalar_ref"), |b| {
            let cursor = Cell::new(0);
            b.iter(|| {
                let index = cursor.get();
                cursor.set((index + 1) % mat3_lhs.len());
                black_box(mat3_lhs[index].map_scalar(&scalar_cases[index], &ctx, $engine::Ctx::add))
            })
        });
        group.bench_function(format!("{label}/mat3 sub_scalar_ref"), |b| {
            let cursor = Cell::new(0);
            b.iter(|| {
                let index = cursor.get();
                cursor.set((index + 1) % mat3_lhs.len());
                black_box(mat3_lhs[index].map_scalar(&scalar_cases[index], &ctx, $engine::Ctx::sub))
            })
        });
        group.bench_function(format!("{label}/mat3 mul_scalar_ref"), |b| {
            let cursor = Cell::new(0);
            b.iter(|| {
                let index = cursor.get();
                cursor.set((index + 1) % mat3_lhs.len());
                black_box(mat3_lhs[index].map_scalar(&scalar_cases[index], &ctx, $engine::Ctx::mul))
            })
        });
        group.bench_function(format!("{label}/mat3 div_scalar_ref"), |b| {
            let cursor = Cell::new(0);
            b.iter(|| {
                let index = cursor.get();
                cursor.set((index + 1) % mat3_lhs.len());
                black_box(mat3_lhs[index].map_scalar(&scalar_cases[index], &ctx, $engine::Ctx::div))
            })
        });

        group.bench_function(format!("{label}/mat4 add refs"), |b| {
            let cursor = Cell::new(0);
            b.iter(|| {
                let index = cursor.get();
                cursor.set((index + 1) % mat4_lhs.len());
                black_box(mat4_lhs[index].combine(&mat4_rhs[index], &ctx, $engine::Ctx::add))
            })
        });
        group.bench_function(format!("{label}/mat4 sub refs"), |b| {
            let cursor = Cell::new(0);
            b.iter(|| {
                let index = cursor.get();
                cursor.set((index + 1) % mat4_lhs.len());
                black_box(mat4_lhs[index].combine(&mat4_rhs[index], &ctx, $engine::Ctx::sub))
            })
        });
        group.bench_function(format!("{label}/mat4 mul refs"), |b| {
            let cursor = Cell::new(0);
            b.iter(|| {
                let index = cursor.get();
                cursor.set((index + 1) % mat4_lhs.len());
                black_box(mat4_lhs[index].mul_mat4(&mat4_rhs[index], &ctx))
            })
        });
        group.bench_function(format!("{label}/mat4 div refs"), |b| {
            let cursor = Cell::new(0);
            b.iter(|| {
                let index = cursor.get();
                cursor.set((index + 1) % mat4_lhs.len());
                black_box(mat4_lhs[index].div_matrix(&mat4_rhs[index], &ctx))
            })
        });
        group.bench_function(format!("{label}/mat4 neg ref"), |b| {
            let cursor = Cell::new(0);
            b.iter(|| black_box(next_case(&mat4_lhs, &cursor).neg(&ctx)))
        });
        group.bench_function(format!("{label}/mat4 add_scalar_ref"), |b| {
            let cursor = Cell::new(0);
            b.iter(|| {
                let index = cursor.get();
                cursor.set((index + 1) % mat4_lhs.len());
                black_box(mat4_lhs[index].map_scalar(&scalar_cases[index], &ctx, $engine::Ctx::add))
            })
        });
        group.bench_function(format!("{label}/mat4 sub_scalar_ref"), |b| {
            let cursor = Cell::new(0);
            b.iter(|| {
                let index = cursor.get();
                cursor.set((index + 1) % mat4_lhs.len());
                black_box(mat4_lhs[index].map_scalar(&scalar_cases[index], &ctx, $engine::Ctx::sub))
            })
        });
        group.bench_function(format!("{label}/mat4 mul_scalar_ref"), |b| {
            let cursor = Cell::new(0);
            b.iter(|| {
                let index = cursor.get();
                cursor.set((index + 1) % mat4_lhs.len());
                black_box(mat4_lhs[index].map_scalar(&scalar_cases[index], &ctx, $engine::Ctx::mul))
            })
        });
        group.bench_function(format!("{label}/mat4 div_scalar_ref"), |b| {
            let cursor = Cell::new(0);
            b.iter(|| {
                let index = cursor.get();
                cursor.set((index + 1) % mat4_lhs.len());
                black_box(mat4_lhs[index].map_scalar(&scalar_cases[index], &ctx, $engine::Ctx::div))
            })
        });

        group.bench_function(format!("{label}/mat3 transform_vec refs"), |b| {
            let cursor = Cell::new(0);
            b.iter(|| {
                let index = cursor.get();
                cursor.set((index + 1) % mat3_lhs.len());
                black_box(mat3_lhs[index].transform_vec3(&vec3_lhs[index], &ctx))
            })
        });
        group.bench_function(format!("{label}/mat4 transform_vec refs"), |b| {
            let cursor = Cell::new(0);
            b.iter(|| {
                let index = cursor.get();
                cursor.set((index + 1) % mat4_lhs.len());
                black_box(mat4_lhs[index].transform_vec4(&vec4_lhs[index], &ctx))
            })
        });

        for name in ["add", "sub", "mul", "div"] {
            group.bench_function(format!("{label}/complex {name} refs"), |b| {
                let cursor = Cell::new(0);
                b.iter(|| {
                    let index = cursor.get();
                    cursor.set((index + 1) % complex_lhs.len());
                    black_box(match name {
                        "add" => complex_lhs[index].add(&complex_rhs[index], &ctx),
                        "sub" => complex_lhs[index].sub(&complex_rhs[index], &ctx),
                        "mul" => complex_lhs[index].mul(&complex_rhs[index], &ctx),
                        _ => complex_lhs[index].div(&complex_rhs[index], &ctx),
                    })
                })
            });
        }
        group.bench_function(format!("{label}/complex neg ref"), |b| {
            let cursor = Cell::new(0);
            b.iter(|| black_box(next_case(&complex_lhs, &cursor).neg(&ctx)))
        });
        group.bench_function(format!("{label}/complex div_real_ref"), |b| {
            let cursor = Cell::new(0);
            b.iter(|| {
                let index = cursor.get();
                cursor.set((index + 1) % complex_lhs.len());
                black_box(complex_lhs[index].div_real(&scalar_cases[index], &ctx))
            })
        });
    }};
}

/// Instantiates the sixty-row adapter schedule using Symbolica's `Float` backend.
///
/// The `numerica128` label denotes the local `numerica_engine`, not a separate
/// Numerica dependency. Its context requests 128-bit precision for imported
/// binary-float fixtures and arithmetic. The shared macro's borrowed ownership
/// semantics and ordinary timing apply, without native exactness certification.
fn bench_numerica_borrowed_operations(
    group: &mut BenchmarkGroup<'_, criterion::measurement::WallTime>,
    label: &str,
) {
    bench_external_borrowed_operations!(numerica_engine, group, label);
}

/// Instantiates the sixty-row adapter schedule using Rug/MPFR at 128-bit precision.
///
/// Retained inputs originate as binary floats. The shared macro times borrowed
/// adapter arithmetic and its explicit clone variants using ordinary `iter`;
/// these labels do not reproduce native mixed-ownership batching or typed errors.
fn bench_gmp_borrowed_operations(
    group: &mut BenchmarkGroup<'_, criterion::measurement::WallTime>,
    label: &str,
) {
    bench_external_borrowed_operations!(gmp_engine, group, label);
}

/// Instantiates the sixty-row adapter schedule using Symbolica `Atom` expressions.
///
/// The context stores the supplied 128 but these scalar, linear and complex
/// operations do not use it as a precision setting. Fixtures enter through
/// `Atom::num` from binary floats. Borrowed adapter calls and clone variants follow
/// the shared macro; no common 128-bit or native certification contract is asserted.
fn bench_symbolica_borrowed_operations(
    group: &mut BenchmarkGroup<'_, criterion::measurement::WallTime>,
    label: &str,
) {
    bench_external_borrowed_operations!(symbolica_engine, group, label);
}
