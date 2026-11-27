/// Registers three native Vec3 comparisons over four retained binary-float cases.
///
/// Imports each coordinate exactly through blas_vec3 before timing. Dot pairs
/// equally indexed vectors; magnitude and normalization visit the left cases.
/// Each ordinary iteration advances its own cursor, borrows the inputs and
/// destroys the output inside timing. No primitive-float output is requested;
/// scalar objects and evidence may be shared across iterations.
///
/// Three enabled/selected trace hooks each perform one operation per case before
/// timing. Magnitude/normalization Results are unwrapped, so invalid conversion
/// or a failing scalar/zero-norm operation can panic. The helper does not validate
/// the arrays or assert values, dispatch counts, cancellation or total resource bounds.
fn bench_blas_vectors(
    group: &mut BenchmarkGroup<'_, criterion::measurement::WallTime>,
    label: &str,
    lhs_cases: [SampleVec3; 4],
    rhs_cases: [SampleVec3; 4],
) {
    let blas_lhs_cases = lhs_cases.map(blas_vec3);
    let blas_rhs_cases = rhs_cases.map(blas_vec3);
    trace_dispatch_row(format!("vectors/{label}/vec3 dot"), || {
        for index in 0..blas_lhs_cases.len() {
            black_box(black_box(&blas_lhs_cases[index]).dot(black_box(&blas_rhs_cases[index])));
        }
    });
    trace_dispatch_row(format!("vectors/{label}/vec3 magnitude"), || {
        for value in &blas_lhs_cases {
            black_box(black_box(value).magnitude().unwrap());
        }
    });
    trace_dispatch_row(format!("vectors/{label}/vec3 normalize"), || {
        for value in &blas_lhs_cases {
            black_box(black_box(value).normalize().unwrap());
        }
    });
    group.bench_function(format!("{label}/vec3 dot"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % blas_lhs_cases.len());
            black_box(black_box(&blas_lhs_cases[index]).dot(black_box(&blas_rhs_cases[index])))
        })
    });
    group.bench_function(format!("{label}/vec3 magnitude"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                black_box(next_case(&blas_lhs_cases, &cursor))
                    .magnitude()
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/vec3 normalize"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                black_box(next_case(&blas_lhs_cases, &cursor))
                    .normalize()
                    .unwrap(),
            )
        })
    });
}

/// Offers one untimed Vec4 normalization hook over four exactly imported cases.
///
/// Constructs native inputs even when tracing is disabled or filtered out. If
/// selected/enabled, borrows and normalizes each four-coordinate vector and unwraps
/// the result; all four lanes participate in its Euclidean norm. This registers no
/// Criterion row and asserts neither output values nor normalization dispatch counts.
fn trace_blas_vec4_normalize(label: &str, lhs_cases: [SampleVec4; 4]) {
    let blas_lhs_cases = lhs_cases.map(blas_vec4);
    trace_dispatch_row(format!("vectors/{label}/vec4 normalize"), || {
        for value in &blas_lhs_cases {
            black_box(black_box(value).normalize().unwrap());
        }
    });
}

/// Registers fifteen Vec3 timing rows and offers eight native diagnostic hooks.
///
/// Dot, magnitude and normalization run for two native models and three adapters
/// under inherited Criterion settings. The binary-native and external rows rotate
/// through four retained cases. The explicit-rational rows instead reuse one fixed
/// authored decimal vector pair, not a decimal conversion of all four cases.
/// Native rows borrow their operands; external rows explicitly clone the left
/// vector inside ordinary iter before borrowing it for adapter work. Construction
/// of retained fixtures/contexts is outside timing, while cursor work where present,
/// explicit clones, operations and result destruction are measured.
///
/// Four hooks per native model cover the three Vec3 operations and Vec4 normalization;
/// the binary hooks visit four cases and rational hooks one fixed case. Vec4
/// normalization is diagnostic only, without a timing row. Tracing may reuse
/// retained evidence later consumed by measured work. Native Results are unwrapped.
///
/// Numerica labels use Symbolica Float and GMP uses Rug/MPFR, both with context
/// argument 128; the Atom adapter stores that argument without using it as a
/// numeric precision setting here. There are no external trace hooks, numeric
/// equality/accuracy assertions, certainty receipts or total-work bounds. Matching
/// operation labels do not establish equal inputs, cloning cost or output evaluation.
fn bench_vectors(c: &mut Criterion) {
    let mut group = c.benchmark_group("vectors");
    let lhs_cases = sample_vec3_cases();
    let rhs_cases = sample_vec3_b_cases();
    let lhs4_cases = sample_vec4_cases();

    bench_blas_vectors(&mut group, "hyperreal", lhs_cases, rhs_cases);
    trace_blas_vec4_normalize("hyperreal", lhs4_cases);

    {
        let rational_lhs = blas_vec3_rational();
        let rational_rhs = blas_vec3_b_rational();
        trace_dispatch_row("vectors/hyperreal-rational/vec3 dot", || {
            black_box(black_box(&rational_lhs).dot(black_box(&rational_rhs)));
        });
        trace_dispatch_row("vectors/hyperreal-rational/vec3 magnitude", || {
            black_box(black_box(&rational_lhs).magnitude().unwrap());
        });
        trace_dispatch_row("vectors/hyperreal-rational/vec3 normalize", || {
            black_box(black_box(&rational_lhs).normalize().unwrap());
        });
        let rational_lhs4 = blas_vec4_rational();
        trace_dispatch_row("vectors/hyperreal-rational/vec4 normalize", || {
            black_box(black_box(&rational_lhs4).normalize().unwrap());
        });
        group.bench_function("hyperreal-rational/vec3 dot", |b| {
            b.iter(|| black_box(black_box(&rational_lhs).dot(black_box(&rational_rhs))))
        });
        group.bench_function("hyperreal-rational/vec3 magnitude", |b| {
            b.iter(|| black_box(black_box(&rational_lhs).magnitude().unwrap()))
        });
        group.bench_function("hyperreal-rational/vec3 normalize", |b| {
            b.iter(|| black_box(black_box(&rational_lhs).normalize().unwrap()))
        });
    }
    let numerica_ctx = numerica_engine::Ctx::new(128);
    let numerica_lhs_cases =
        lhs_cases.map(|value| numerica_engine::Vec3::new(&numerica_ctx, value.x, value.y, value.z));
    let numerica_rhs_cases =
        rhs_cases.map(|value| numerica_engine::Vec3::new(&numerica_ctx, value.x, value.y, value.z));
    group.bench_function("numerica128/vec3 dot", |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % numerica_lhs_cases.len());
            black_box(numerica_lhs_cases[index].clone())
                .dot(black_box(&numerica_rhs_cases[index]), &numerica_ctx)
        })
    });
    group.bench_function("numerica128/vec3 magnitude", |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(next_case(&numerica_lhs_cases, &cursor).clone()).magnitude(&numerica_ctx)
        })
    });
    group.bench_function("numerica128/vec3 normalize", |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(next_case(&numerica_lhs_cases, &cursor).clone()).normalize(&numerica_ctx)
        })
    });

    let gmp_ctx = gmp_engine::Ctx::new(128);
    let gmp_lhs_cases =
        lhs_cases.map(|value| gmp_engine::Vec3::new(&gmp_ctx, value.x, value.y, value.z));
    let gmp_rhs_cases =
        rhs_cases.map(|value| gmp_engine::Vec3::new(&gmp_ctx, value.x, value.y, value.z));
    group.bench_function("gmp_mpfr128/vec3 dot", |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % gmp_lhs_cases.len());
            black_box(gmp_lhs_cases[index].clone()).dot(black_box(&gmp_rhs_cases[index]), &gmp_ctx)
        })
    });
    group.bench_function("gmp_mpfr128/vec3 magnitude", |b| {
        let cursor = Cell::new(0);
        b.iter(|| black_box(next_case(&gmp_lhs_cases, &cursor).clone()).magnitude(&gmp_ctx))
    });
    group.bench_function("gmp_mpfr128/vec3 normalize", |b| {
        let cursor = Cell::new(0);
        b.iter(|| black_box(next_case(&gmp_lhs_cases, &cursor).clone()).normalize(&gmp_ctx))
    });

    let symbolica_ctx = symbolica_engine::Ctx::new(128);
    let symbolica_lhs_cases = lhs_cases
        .map(|value| symbolica_engine::Vec3::new(&symbolica_ctx, value.x, value.y, value.z));
    let symbolica_rhs_cases = rhs_cases
        .map(|value| symbolica_engine::Vec3::new(&symbolica_ctx, value.x, value.y, value.z));
    group.bench_function("symbolica/vec3 dot", |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % symbolica_lhs_cases.len());
            black_box(symbolica_lhs_cases[index].clone())
                .dot(black_box(&symbolica_rhs_cases[index]), &symbolica_ctx)
        })
    });
    group.bench_function("symbolica/vec3 magnitude", |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(next_case(&symbolica_lhs_cases, &cursor).clone()).magnitude(&symbolica_ctx)
        })
    });
    group.bench_function("symbolica/vec3 normalize", |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(next_case(&symbolica_lhs_cases, &cursor).clone()).normalize(&symbolica_ctx)
        })
    });
    group.finish();
}

/// Registers four native Mat3 comparisons and matching four-case diagnostic hooks.
///
/// Imports the supplied matrix/vector coordinates as their exact binary-float
/// values before timing. Determinant borrows a matrix; inverse clones and consumes
/// it; matrix/matrix and matrix/vector multiplication clone both inputs. All four
/// ordinary iter rows cycle through retained cases, measuring selection, explicit
/// clones, operations and output destruction. Matrix-vector products use all three
/// coordinates without homogeneous division, and no primitive-float output is requested.
///
/// Each selected/enabled trace hook performs its operation on all four cases once
/// before timing, with the same ownership choices. Inverse Results are unwrapped;
/// this helper does not prevalidate nonsingularity or catch scalar failures. No
/// value, dispatch-count, policy-receipt or total-work assertion qualifies the rows.
fn bench_blas_matrix3(
    group: &mut BenchmarkGroup<'_, criterion::measurement::WallTime>,
    label: &str,
    lhs_cases: [SampleMat3; 4],
    rhs_cases: [SampleMat3; 4],
    vector_cases: [SampleVec3; 4],
) {
    let blas_lhs_cases = lhs_cases.map(blas_mat3);
    let blas_rhs_cases = rhs_cases.map(blas_mat3);
    let blas_vector_cases = vector_cases.map(blas_vec3);
    trace_dispatch_row(format!("matrix3/{label}/mat3 determinant"), || {
        for value in &blas_lhs_cases {
            black_box(black_box(value).determinant());
        }
    });
    trace_dispatch_row(format!("matrix3/{label}/mat3 inverse"), || {
        for value in &blas_lhs_cases {
            black_box(black_box(value.clone()).inverse().unwrap());
        }
    });
    trace_dispatch_row(format!("matrix3/{label}/mat3 mul mat3"), || {
        for index in 0..blas_lhs_cases.len() {
            black_box(
                black_box(blas_lhs_cases[index].clone()) * black_box(blas_rhs_cases[index].clone()),
            );
        }
    });
    trace_dispatch_row(format!("matrix3/{label}/mat3 transform vec3"), || {
        for index in 0..blas_lhs_cases.len() {
            black_box(
                black_box(blas_lhs_cases[index].clone())
                    * black_box(blas_vector_cases[index].clone()),
            );
        }
    });
    group.bench_function(format!("{label}/mat3 determinant"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| black_box(black_box(next_case(&blas_lhs_cases, &cursor)).determinant()))
    });
    group.bench_function(format!("{label}/mat3 inverse"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                black_box(next_case(&blas_lhs_cases, &cursor).clone())
                    .inverse()
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/mat3 mul mat3"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % blas_lhs_cases.len());
            black_box(
                black_box(blas_lhs_cases[index].clone()) * black_box(blas_rhs_cases[index].clone()),
            )
        })
    });
    group.bench_function(format!("{label}/mat3 transform vec3"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % blas_lhs_cases.len());
            black_box(
                black_box(blas_lhs_cases[index].clone())
                    * black_box(blas_vector_cases[index].clone()),
            )
        })
    });
}

/// Registers thirty-five Mat3 timing rows and offers twenty-three native trace hooks.
///
/// Four binary-native baseline rows and four per external adapter rotate through
/// retained four-case fixtures. Eleven fixed rational and eight fixed symbolic
/// rows add baseline operations and selected transform consumers. The rational
/// matrices use authored tenths and the vector long decimal ratios, rather than
/// converting the full floating-case corpus. Symbolic matrix/vector entries use
/// pi, e and rational coefficients. No cross-backend input/evaluation equivalence
/// follows from labels. Numerica/Symbolica Float and Rug/MPFR contexts request 128;
/// the Atom adapter does not use its stored 128 as a precision setting here.
///
/// Ordinary iter measures selection for cycling rows, explicit operand clones,
/// operations, requested consumers and destruction. Native determinants borrow,
/// inverses consume clones, and ordinary products clone both inputs. External rows
/// explicitly clone the left matrix, then borrow adapter inputs. Batch rows clone
/// the matrix and borrow four copies of one vector. transform_vec3_batch constructs
/// all four output vectors immediately and gathers matrix facts once per call;
/// this is not a retained batch handle or a persistent matrix cache.
///
/// | Native transform consumer | Requests after constructing the output |
/// | --- | --- |
/// | Plain single/batch | No explicit float conversion or fact query |
/// | Single one-coord/all-coord approx | One / all three coordinates as optional lossy f64 |
/// | Batch one-coord approx | Coordinate zero of output vector zero only |
/// | Rational batch all-coord approx | All twelve coordinates of four vectors |
/// | Symbolic batch all-coord approx | Nine coordinates of the first three vectors only |
/// | Either batch structural facts | Sign and zero status for the first nine coordinates only |
/// | Rational single sign/zero facts | Sign and zero status for all three coordinates |
/// | Symbolic single sign refinement | refine_sign_until(0) for each of three coordinates |
///
/// Approximation/sign Options are black-boxed without requiring Some; structural
/// facts may remain unknown. Query names and trace collection are diagnostics, not
/// assertions that unrelated coordinates avoided computation or that a particular
/// dispatch route ran. Native inverse Results are unwrapped. Four binary hooks visit
/// all four cases; nineteen fixed rational/symbolic hooks mirror their consumers
/// before timing. No external hooks, correctness/dispatch-count assertions, policy
/// receipts, cancellation signals or total-work/memory limits are added here.
fn bench_matrix3(c: &mut Criterion) {
    let mut group = c.benchmark_group("matrix3");
    let lhs_cases = sample_mat3_cases();
    let rhs_cases = sample_mat3_b_cases();
    let vector_cases = sample_vec3_cases();

    bench_blas_matrix3(&mut group, "hyperreal", lhs_cases, rhs_cases, vector_cases);

    {
        let rational_lhs = blas_mat3_rational();
        let rational_rhs = blas_mat3_b_rational();
        let rational_vector = blas_vec3_rational();
        let rational_vec3_batch = [
            rational_vector.clone(),
            rational_vector.clone(),
            rational_vector.clone(),
            rational_vector.clone(),
        ];
        let symbolic_mat3 = Matrix3::new([
            [Real::pi(), Real::e(), q(-5, 7)],
            [q(13, 8), q(-3, 4), Real::pi()],
            [q(7, 3), Real::e(), q(11, 5)],
        ]);
        let symbolic_vector = Vector3::new([Real::pi(), q(-2, 3), Real::e()]);
        let symbolic_vec3_batch = [
            symbolic_vector.clone(),
            symbolic_vector.clone(),
            symbolic_vector.clone(),
            symbolic_vector.clone(),
        ];
        trace_dispatch_row("matrix3/hyperreal-rational/mat3 determinant", || {
            black_box(black_box(&rational_lhs).determinant());
        });
        trace_dispatch_row("matrix3/hyperreal-rational/mat3 inverse", || {
            black_box(black_box(rational_lhs.clone()).inverse().unwrap());
        });
        trace_dispatch_row("matrix3/hyperreal-rational/mat3 mul mat3", || {
            black_box(black_box(rational_lhs.clone()) * black_box(rational_rhs.clone()));
        });
        trace_dispatch_row("matrix3/hyperreal-rational/mat3 transform vec3", || {
            black_box(black_box(rational_lhs.clone()) * black_box(rational_vector.clone()));
        });
        // The immediate batch shares identity/diagonal facts within this invocation.
        trace_dispatch_row(
            "matrix3/hyperreal-rational/mat3 transform vec3 batch",
            || {
                black_box(
                    black_box(rational_lhs.clone())
                        .transform_vec3_batch(black_box(&rational_vec3_batch)),
                );
            },
        );
        // Request one lane after batch construction; this hook asserts no demand counts.
        trace_dispatch_row(
            "matrix3/hyperreal-rational/mat3 transform vec3 batch one-coord approx",
            || {
                let transformed = black_box(
                    black_box(rational_lhs.clone())
                        .transform_vec3_batch(black_box(&rational_vec3_batch)),
                );
                black_box(transformed[0][0].to_f64_lossy());
            },
        );
        trace_dispatch_row(
            "matrix3/hyperreal-rational/mat3 transform vec3 batch all-coord approx",
            || {
                let transformed = black_box(
                    black_box(rational_lhs.clone())
                        .transform_vec3_batch(black_box(&rational_vec3_batch)),
                );
                black_box((
                    transformed[0][0].to_f64_lossy(),
                    transformed[0][1].to_f64_lossy(),
                    transformed[0][2].to_f64_lossy(),
                    transformed[1][0].to_f64_lossy(),
                    transformed[1][1].to_f64_lossy(),
                    transformed[1][2].to_f64_lossy(),
                    transformed[2][0].to_f64_lossy(),
                    transformed[2][1].to_f64_lossy(),
                    transformed[2][2].to_f64_lossy(),
                    transformed[3][0].to_f64_lossy(),
                    transformed[3][1].to_f64_lossy(),
                    transformed[3][2].to_f64_lossy(),
                ));
            },
        );
        // Request one component; compare traces separately to assess approximation work.
        trace_dispatch_row(
            "matrix3/hyperreal-rational/mat3 transform vec3 one-coord approx",
            || {
                let transformed =
                    black_box(black_box(rational_lhs.clone()) * black_box(rational_vector.clone()));
                black_box(transformed[0].to_f64_lossy());
            },
        );
        trace_dispatch_row(
            "matrix3/hyperreal-rational/mat3 transform vec3 all-coord approx",
            || {
                let transformed =
                    black_box(black_box(rational_lhs.clone()) * black_box(rational_vector.clone()));
                black_box((
                    transformed[0].to_f64_lossy(),
                    transformed[1].to_f64_lossy(),
                    transformed[2].to_f64_lossy(),
                ));
            },
        );
        // Exercise symbolic transform construction without asserting its dispatch path.
        trace_dispatch_row("matrix3/hyperreal-symbolic/mat3 transform vec3", || {
            black_box(black_box(symbolic_mat3.clone()) * black_box(symbolic_vector.clone()));
        });
        // Request only the first symbolic coordinate's approximate output.
        trace_dispatch_row(
            "matrix3/hyperreal-symbolic/mat3 transform vec3 one-coord approx",
            || {
                let transformed = black_box(
                    black_box(symbolic_mat3.clone()) * black_box(symbolic_vector.clone()),
                );
                black_box(transformed[0].to_f64_lossy());
            },
        );
        trace_dispatch_row(
            "matrix3/hyperreal-symbolic/mat3 transform vec3 all-coord approx",
            || {
                let transformed = black_box(
                    black_box(symbolic_mat3.clone()) * black_box(symbolic_vector.clone()),
                );
                black_box((
                    transformed[0].to_f64_lossy(),
                    transformed[1].to_f64_lossy(),
                    transformed[2].to_f64_lossy(),
                ));
            },
        );
        // Build all four symbolic output vectors, then request selected coordinates.
        trace_dispatch_row(
            "matrix3/hyperreal-symbolic/mat3 transform vec3 batch",
            || {
                black_box(
                    black_box(symbolic_mat3.clone())
                        .transform_vec3_batch(black_box(&symbolic_vec3_batch)),
                );
            },
        );
        trace_dispatch_row(
            "matrix3/hyperreal-symbolic/mat3 transform vec3 batch one-coord approx",
            || {
                let transformed = black_box(
                    black_box(symbolic_mat3.clone())
                        .transform_vec3_batch(black_box(&symbolic_vec3_batch)),
                );
                black_box(transformed[0][0].to_f64_lossy());
            },
        );
        trace_dispatch_row(
            "matrix3/hyperreal-symbolic/mat3 transform vec3 batch all-coord approx",
            || {
                let transformed = black_box(
                    black_box(symbolic_mat3.clone())
                        .transform_vec3_batch(black_box(&symbolic_vec3_batch)),
                );
                black_box((
                    transformed[0][0].to_f64_lossy(),
                    transformed[0][1].to_f64_lossy(),
                    transformed[0][2].to_f64_lossy(),
                    transformed[1][0].to_f64_lossy(),
                    transformed[1][1].to_f64_lossy(),
                    transformed[1][2].to_f64_lossy(),
                    transformed[2][0].to_f64_lossy(),
                    transformed[2][1].to_f64_lossy(),
                    transformed[2][2].to_f64_lossy(),
                ));
            },
        );
        // Build the batch, then query sign/zero facts on its first three vectors.
        trace_dispatch_row(
            "matrix3/hyperreal-rational/mat3 transform vec3 batch structural facts",
            || {
                let transformed = black_box(
                    black_box(rational_lhs.clone())
                        .transform_vec3_batch(black_box(&rational_vec3_batch)),
                );
                let _ = black_box((
                    transformed[0][0].structural_facts().sign,
                    transformed[0][1].structural_facts().sign,
                    transformed[0][2].structural_facts().sign,
                    transformed[1][0].structural_facts().sign,
                    transformed[1][1].structural_facts().sign,
                    transformed[1][2].structural_facts().sign,
                    transformed[2][0].structural_facts().sign,
                    transformed[2][1].structural_facts().sign,
                    transformed[2][2].structural_facts().sign,
                    transformed[0][0].zero_status(),
                    transformed[0][1].zero_status(),
                    transformed[0][2].zero_status(),
                    transformed[1][0].zero_status(),
                    transformed[1][1].zero_status(),
                    transformed[1][2].zero_status(),
                    transformed[2][0].zero_status(),
                    transformed[2][1].zero_status(),
                    transformed[2][2].zero_status(),
                ));
            },
        );
        // Query symbolic sign/zero facts without explicitly requesting float output.
        trace_dispatch_row(
            "matrix3/hyperreal-symbolic/mat3 transform vec3 batch structural facts",
            || {
                let transformed = black_box(
                    black_box(symbolic_mat3.clone())
                        .transform_vec3_batch(black_box(&symbolic_vec3_batch)),
                );
                let _ = black_box((
                    transformed[0][0].structural_facts().sign,
                    transformed[0][1].structural_facts().sign,
                    transformed[0][2].structural_facts().sign,
                    transformed[1][0].structural_facts().sign,
                    transformed[1][1].structural_facts().sign,
                    transformed[1][2].structural_facts().sign,
                    transformed[2][0].structural_facts().sign,
                    transformed[2][1].structural_facts().sign,
                    transformed[2][2].structural_facts().sign,
                    transformed[0][0].zero_status(),
                    transformed[0][1].zero_status(),
                    transformed[0][2].zero_status(),
                    transformed[1][0].zero_status(),
                    transformed[1][1].zero_status(),
                    transformed[1][2].zero_status(),
                    transformed[2][0].zero_status(),
                    transformed[2][1].zero_status(),
                    transformed[2][2].zero_status(),
                ));
            },
        );
        // Construct the transform, then query facts without explicit float conversion.
        trace_dispatch_row(
            "matrix3/hyperreal-rational/mat3 transform vec3 sign/zero facts",
            || {
                let transformed =
                    black_box(black_box(rational_lhs.clone()) * black_box(rational_vector.clone()));
                black_box((
                    transformed[0].structural_facts().sign,
                    transformed[1].structural_facts().sign,
                    transformed[2].structural_facts().sign,
                    transformed[0].zero_status(),
                    transformed[1].zero_status(),
                    transformed[2].zero_status(),
                ));
            },
        );
        // Request sign refinement on each coordinate; no refinement-work assertion follows.
        trace_dispatch_row(
            "matrix3/hyperreal-symbolic/mat3 transform vec3 sign refinement",
            || {
                let transformed = black_box(
                    black_box(symbolic_mat3.clone()) * black_box(symbolic_vector.clone()),
                );
                black_box((
                    transformed[0].refine_sign_until(0),
                    transformed[1].refine_sign_until(0),
                    transformed[2].refine_sign_until(0),
                ));
            },
        );
        group.bench_function("hyperreal-rational/mat3 determinant", |b| {
            b.iter(|| black_box(black_box(&rational_lhs).determinant()))
        });
        group.bench_function("hyperreal-rational/mat3 inverse", |b| {
            b.iter(|| black_box(black_box(rational_lhs.clone()).inverse().unwrap()))
        });
        group.bench_function("hyperreal-rational/mat3 mul mat3", |b| {
            b.iter(|| black_box(black_box(rational_lhs.clone()) * black_box(rational_rhs.clone())))
        });
        group.bench_function("hyperreal-rational/mat3 transform vec3", |b| {
            b.iter(|| {
                black_box(black_box(rational_lhs.clone()) * black_box(rational_vector.clone()))
            })
        });
        group.bench_function("hyperreal-rational/mat3 transform vec3 batch", |b| {
            b.iter(|| {
                black_box(
                    black_box(rational_lhs.clone())
                        .transform_vec3_batch(black_box(&rational_vec3_batch)),
                )
            })
        });
        group.bench_function(
            "hyperreal-rational/mat3 transform vec3 batch one-coord approx",
            |b| {
                b.iter(|| {
                    let transformed = black_box(
                        black_box(rational_lhs.clone())
                            .transform_vec3_batch(black_box(&rational_vec3_batch)),
                    );
                    black_box(transformed[0][0].to_f64_lossy())
                })
            },
        );
        group.bench_function(
            "hyperreal-rational/mat3 transform vec3 batch all-coord approx",
            |b| {
                b.iter(|| {
                    let transformed = black_box(
                        black_box(rational_lhs.clone())
                            .transform_vec3_batch(black_box(&rational_vec3_batch)),
                    );
                    black_box((
                        transformed[0][0].to_f64_lossy(),
                        transformed[0][1].to_f64_lossy(),
                        transformed[0][2].to_f64_lossy(),
                        transformed[1][0].to_f64_lossy(),
                        transformed[1][1].to_f64_lossy(),
                        transformed[1][2].to_f64_lossy(),
                        transformed[2][0].to_f64_lossy(),
                        transformed[2][1].to_f64_lossy(),
                        transformed[2][2].to_f64_lossy(),
                        transformed[3][0].to_f64_lossy(),
                        transformed[3][1].to_f64_lossy(),
                        transformed[3][2].to_f64_lossy(),
                    ))
                })
            },
        );
        group.bench_function(
            "hyperreal-rational/mat3 transform vec3 sign/zero facts",
            |b| {
                b.iter(|| {
                    let transformed = black_box(
                        black_box(rational_lhs.clone()) * black_box(rational_vector.clone()),
                    );
                    black_box((
                        transformed[0].structural_facts().sign,
                        transformed[1].structural_facts().sign,
                        transformed[2].structural_facts().sign,
                        transformed[0].zero_status(),
                        transformed[1].zero_status(),
                        transformed[2].zero_status(),
                    ))
                })
            },
        );
        group.bench_function(
            "hyperreal-rational/mat3 transform vec3 one-coord approx",
            |b| {
                b.iter(|| {
                    let transformed = black_box(
                        black_box(rational_lhs.clone()) * black_box(rational_vector.clone()),
                    );
                    black_box(transformed[0].to_f64_lossy())
                })
            },
        );
        group.bench_function(
            "hyperreal-rational/mat3 transform vec3 all-coord approx",
            |b| {
                b.iter(|| {
                    let transformed = black_box(
                        black_box(rational_lhs.clone()) * black_box(rational_vector.clone()),
                    );
                    black_box((
                        transformed[0].to_f64_lossy(),
                        transformed[1].to_f64_lossy(),
                        transformed[2].to_f64_lossy(),
                    ))
                })
            },
        );
        group.bench_function(
            "hyperreal-symbolic/mat3 transform vec3 sign refinement",
            |b| {
                b.iter(|| {
                    let transformed = black_box(
                        black_box(symbolic_mat3.clone()) * black_box(symbolic_vector.clone()),
                    );
                    black_box((
                        transformed[0].refine_sign_until(0),
                        transformed[1].refine_sign_until(0),
                        transformed[2].refine_sign_until(0),
                    ))
                })
            },
        );
        // Timed symbolic benchmark to mirror rational behavior while exercising
        // non-rational fact propagation.
        group.bench_function("hyperreal-symbolic/mat3 transform vec3", |b| {
            b.iter(|| {
                black_box(black_box(symbolic_mat3.clone()) * black_box(symbolic_vector.clone()))
            })
        });
        group.bench_function(
            "hyperreal-symbolic/mat3 transform vec3 one-coord approx",
            |b| {
                b.iter(|| {
                    let transformed = black_box(
                        black_box(symbolic_mat3.clone()) * black_box(symbolic_vector.clone()),
                    );
                    black_box(transformed[0].to_f64_lossy())
                })
            },
        );
        group.bench_function(
            "hyperreal-symbolic/mat3 transform vec3 all-coord approx",
            |b| {
                b.iter(|| {
                    let transformed = black_box(
                        black_box(symbolic_mat3.clone()) * black_box(symbolic_vector.clone()),
                    );
                    black_box((
                        transformed[0].to_f64_lossy(),
                        transformed[1].to_f64_lossy(),
                        transformed[2].to_f64_lossy(),
                    ))
                })
            },
        );
        group.bench_function("hyperreal-symbolic/mat3 transform vec3 batch", |b| {
            b.iter(|| {
                black_box(
                    black_box(symbolic_mat3.clone())
                        .transform_vec3_batch(black_box(&symbolic_vec3_batch)),
                )
            })
        });
        group.bench_function(
            "hyperreal-symbolic/mat3 transform vec3 batch one-coord approx",
            |b| {
                b.iter(|| {
                    let transformed = black_box(
                        black_box(symbolic_mat3.clone())
                            .transform_vec3_batch(black_box(&symbolic_vec3_batch)),
                    );
                    black_box(transformed[0][0].to_f64_lossy())
                })
            },
        );
        group.bench_function(
            "hyperreal-symbolic/mat3 transform vec3 batch all-coord approx",
            |b| {
                b.iter(|| {
                    let transformed = black_box(
                        black_box(symbolic_mat3.clone())
                            .transform_vec3_batch(black_box(&symbolic_vec3_batch)),
                    );
                    black_box((
                        transformed[0][0].to_f64_lossy(),
                        transformed[0][1].to_f64_lossy(),
                        transformed[0][2].to_f64_lossy(),
                        transformed[1][0].to_f64_lossy(),
                        transformed[1][1].to_f64_lossy(),
                        transformed[1][2].to_f64_lossy(),
                        transformed[2][0].to_f64_lossy(),
                        transformed[2][1].to_f64_lossy(),
                        transformed[2][2].to_f64_lossy(),
                    ))
                })
            },
        );
        group.bench_function(
            "hyperreal-rational/mat3 transform vec3 batch structural facts",
            |b| {
                b.iter(|| {
                    let transformed = black_box(
                        black_box(rational_lhs.clone())
                            .transform_vec3_batch(black_box(&rational_vec3_batch)),
                    );
                    black_box((
                        transformed[0][0].structural_facts().sign,
                        transformed[0][1].structural_facts().sign,
                        transformed[0][2].structural_facts().sign,
                        transformed[1][0].structural_facts().sign,
                        transformed[1][1].structural_facts().sign,
                        transformed[1][2].structural_facts().sign,
                        transformed[2][0].structural_facts().sign,
                        transformed[2][1].structural_facts().sign,
                        transformed[2][2].structural_facts().sign,
                        transformed[0][0].zero_status(),
                        transformed[0][1].zero_status(),
                        transformed[0][2].zero_status(),
                        transformed[1][0].zero_status(),
                        transformed[1][1].zero_status(),
                        transformed[1][2].zero_status(),
                        transformed[2][0].zero_status(),
                        transformed[2][1].zero_status(),
                        transformed[2][2].zero_status(),
                    ))
                })
            },
        );
        group.bench_function(
            "hyperreal-symbolic/mat3 transform vec3 batch structural facts",
            |b| {
                b.iter(|| {
                    let transformed = black_box(
                        black_box(symbolic_mat3.clone())
                            .transform_vec3_batch(black_box(&symbolic_vec3_batch)),
                    );
                    black_box((
                        transformed[0][0].structural_facts().sign,
                        transformed[0][1].structural_facts().sign,
                        transformed[0][2].structural_facts().sign,
                        transformed[1][0].structural_facts().sign,
                        transformed[1][1].structural_facts().sign,
                        transformed[1][2].structural_facts().sign,
                        transformed[2][0].structural_facts().sign,
                        transformed[2][1].structural_facts().sign,
                        transformed[2][2].structural_facts().sign,
                        transformed[0][0].zero_status(),
                        transformed[0][1].zero_status(),
                        transformed[0][2].zero_status(),
                        transformed[1][0].zero_status(),
                        transformed[1][1].zero_status(),
                        transformed[1][2].zero_status(),
                        transformed[2][0].zero_status(),
                        transformed[2][1].zero_status(),
                        transformed[2][2].zero_status(),
                    ))
                })
            },
        );
    }
    let numerica_ctx = numerica_engine::Ctx::new(128);
    let numerica_lhs_cases =
        lhs_cases.map(|value| numerica_engine::Mat3::new(&numerica_ctx, value.m));
    let numerica_rhs_cases =
        rhs_cases.map(|value| numerica_engine::Mat3::new(&numerica_ctx, value.m));
    let numerica_vector_cases = vector_cases
        .map(|value| numerica_engine::Vec3::new(&numerica_ctx, value.x, value.y, value.z));
    group.bench_function("numerica128/mat3 determinant", |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(next_case(&numerica_lhs_cases, &cursor).clone()).determinant(&numerica_ctx)
        })
    });
    group.bench_function("numerica128/mat3 inverse", |b| {
        let cursor = Cell::new(0);
        b.iter(|| black_box(next_case(&numerica_lhs_cases, &cursor).clone()).inverse(&numerica_ctx))
    });
    group.bench_function("numerica128/mat3 mul mat3", |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % numerica_lhs_cases.len());
            black_box(numerica_lhs_cases[index].clone())
                .mul_mat3(black_box(&numerica_rhs_cases[index]), &numerica_ctx)
        })
    });
    group.bench_function("numerica128/mat3 transform vec3", |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % numerica_lhs_cases.len());
            black_box(numerica_lhs_cases[index].clone())
                .transform_vec3(black_box(&numerica_vector_cases[index]), &numerica_ctx)
        })
    });

    let gmp_ctx = gmp_engine::Ctx::new(128);
    let gmp_lhs_cases = lhs_cases.map(|value| gmp_engine::Mat3::new(&gmp_ctx, value.m));
    let gmp_rhs_cases = rhs_cases.map(|value| gmp_engine::Mat3::new(&gmp_ctx, value.m));
    let gmp_vector_cases =
        vector_cases.map(|value| gmp_engine::Vec3::new(&gmp_ctx, value.x, value.y, value.z));
    group.bench_function("gmp_mpfr128/mat3 determinant", |b| {
        let cursor = Cell::new(0);
        b.iter(|| black_box(next_case(&gmp_lhs_cases, &cursor).clone()).determinant(&gmp_ctx))
    });
    group.bench_function("gmp_mpfr128/mat3 inverse", |b| {
        let cursor = Cell::new(0);
        b.iter(|| black_box(next_case(&gmp_lhs_cases, &cursor).clone()).inverse(&gmp_ctx))
    });
    group.bench_function("gmp_mpfr128/mat3 mul mat3", |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % gmp_lhs_cases.len());
            black_box(gmp_lhs_cases[index].clone())
                .mul_mat3(black_box(&gmp_rhs_cases[index]), &gmp_ctx)
        })
    });
    group.bench_function("gmp_mpfr128/mat3 transform vec3", |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % gmp_lhs_cases.len());
            black_box(gmp_lhs_cases[index].clone())
                .transform_vec3(black_box(&gmp_vector_cases[index]), &gmp_ctx)
        })
    });

    let symbolica_ctx = symbolica_engine::Ctx::new(128);
    let symbolica_lhs_cases =
        lhs_cases.map(|value| symbolica_engine::Mat3::new(&symbolica_ctx, value.m));
    let symbolica_rhs_cases =
        rhs_cases.map(|value| symbolica_engine::Mat3::new(&symbolica_ctx, value.m));
    let symbolica_vector_cases = vector_cases
        .map(|value| symbolica_engine::Vec3::new(&symbolica_ctx, value.x, value.y, value.z));
    group.bench_function("symbolica/mat3 determinant", |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(next_case(&symbolica_lhs_cases, &cursor).clone()).determinant(&symbolica_ctx)
        })
    });
    group.bench_function("symbolica/mat3 inverse", |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(next_case(&symbolica_lhs_cases, &cursor).clone()).inverse(&symbolica_ctx)
        })
    });
    group.bench_function("symbolica/mat3 mul mat3", |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % symbolica_lhs_cases.len());
            black_box(symbolica_lhs_cases[index].clone())
                .mul_mat3(black_box(&symbolica_rhs_cases[index]), &symbolica_ctx)
        })
    });
    group.bench_function("symbolica/mat3 transform vec3", |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % symbolica_lhs_cases.len());
            black_box(symbolica_lhs_cases[index].clone())
                .transform_vec3(black_box(&symbolica_vector_cases[index]), &symbolica_ctx)
        })
    });

    group.finish();
}

/// Registers seven binary-native Mat4 rows and matching four-case trace hooks.
///
/// Imports supplied matrices/vectors exactly from their finite binary-float
/// coordinates before timing. Dense determinant, inverse, matrix product and
/// homogeneous vector product use the supplied corpora. Three sparse rows use
/// the shared diagonal/triangular corpus for determinant, inverse and product;
/// both sparse operands are separately imported copies of the same cases, so
/// each sparse product squares its corresponding matrix.
///
/// Each ordinary iter row cycles through four retained cases. Determinants borrow,
/// inverses consume a clone, and products clone both operands; selection, explicit
/// clones, operations and output destruction are timed. All four homogeneous
/// output coordinates are retained without perspective division or requested float
/// conversion. Inverse Results are unwrapped; invalid ingress or inverse failure
/// can panic, and arbitrary supplied matrices are not prevalidated here.
///
/// Each enabled/selected hook visits all four cases once before timing, permitting
/// evidence reuse. Sparse names do not assert a dispatch route. There are no value,
/// path-count, policy-receipt, cancellation or total-work/memory checks in this helper.
fn bench_blas_matrix4(
    group: &mut BenchmarkGroup<'_, criterion::measurement::WallTime>,
    label: &str,
    lhs_cases: [SampleMat4; 4],
    rhs_cases: [SampleMat4; 4],
    vector_cases: [SampleVec4; 4],
) {
    let blas_lhs_cases = lhs_cases.map(blas_mat4);
    let blas_rhs_cases = rhs_cases.map(blas_mat4);
    let blas_vector_cases = vector_cases.map(blas_vec4);
    let sparse_mat4_cases = sample_mat4_sparse_cases();
    let blas_sparse_cases = sparse_mat4_cases.map(blas_mat4);
    let blas_sparse_rhs_cases = sparse_mat4_cases.map(blas_mat4);
    trace_dispatch_row(format!("matrix4/{label}/mat4 determinant"), || {
        for value in &blas_lhs_cases {
            black_box(black_box(value).determinant());
        }
    });
    trace_dispatch_row(format!("matrix4/{label}/mat4 inverse"), || {
        for value in &blas_lhs_cases {
            black_box(black_box(value.clone()).inverse().unwrap());
        }
    });
    trace_dispatch_row(format!("matrix4/{label}/mat4 mul mat4"), || {
        for index in 0..blas_lhs_cases.len() {
            black_box(
                black_box(blas_lhs_cases[index].clone()) * black_box(blas_rhs_cases[index].clone()),
            );
        }
    });
    trace_dispatch_row(format!("matrix4/{label}/mat4 mul mat4 sparse"), || {
        for index in 0..blas_sparse_cases.len() {
            black_box(
                black_box(blas_sparse_cases[index].clone())
                    * black_box(blas_sparse_rhs_cases[index].clone()),
            );
        }
    });
    trace_dispatch_row(format!("matrix4/{label}/mat4 transform vec4"), || {
        for index in 0..blas_lhs_cases.len() {
            black_box(
                black_box(blas_lhs_cases[index].clone())
                    * black_box(blas_vector_cases[index].clone()),
            );
        }
    });
    trace_dispatch_row(format!("matrix4/{label}/mat4 determinant sparse"), || {
        for value in &blas_sparse_cases {
            black_box(black_box(value).determinant());
        }
    });
    trace_dispatch_row(format!("matrix4/{label}/mat4 inverse sparse"), || {
        for value in &blas_sparse_cases {
            black_box(black_box(value.clone()).inverse().unwrap());
        }
    });
    group.bench_function(format!("{label}/mat4 determinant"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| black_box(black_box(next_case(&blas_lhs_cases, &cursor)).determinant()))
    });
    group.bench_function(format!("{label}/mat4 inverse"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                black_box(next_case(&blas_lhs_cases, &cursor).clone())
                    .inverse()
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/mat4 determinant sparse"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| black_box(black_box(next_case(&blas_sparse_cases, &cursor)).determinant()))
    });
    group.bench_function(format!("{label}/mat4 inverse sparse"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(
                black_box(next_case(&blas_sparse_cases, &cursor).clone())
                    .inverse()
                    .unwrap(),
            )
        })
    });
    group.bench_function(format!("{label}/mat4 mul mat4"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % blas_lhs_cases.len());
            black_box(
                black_box(blas_lhs_cases[index].clone()) * black_box(blas_rhs_cases[index].clone()),
            )
        })
    });
    group.bench_function(format!("{label}/mat4 mul mat4 sparse"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % blas_sparse_cases.len());
            black_box(
                black_box(blas_sparse_cases[index].clone())
                    * black_box(blas_sparse_rhs_cases[index].clone()),
            )
        })
    });
    group.bench_function(format!("{label}/mat4 transform vec4"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % blas_lhs_cases.len());
            black_box(
                black_box(blas_lhs_cases[index].clone())
                    * black_box(blas_vector_cases[index].clone()),
            )
        })
    });
}

/// Registers sixty-one Mat4 timing rows and offers forty-nine native diagnostic hooks.
///
/// Seven binary-native rows cover four dense and three sparse operations. Each of
/// three external adapters adds only four dense rows; these schedules cycle through
/// four retained cases. Twenty-two fixed hyperreal-rational and twenty fixed
/// hyperreal-symbolic rows add baseline operations and transform consumers. Those
/// labels name workload families, not a guarantee that every operand is rational
/// or symbolic, and the fixed inputs are not the complete rotating corpus.
///
/// The fixed rational baseline matrices and vector have integer entries. General
/// symbolic transforms use pi/e/rational matrix entries and vector weight 9/2.
/// Both direction variables contain the rational vector (3/2,-11/7,9/4,0), even
/// under the symbolic label. The rational-labeled point row uses a rational matrix
/// with the symbolic point (pi,5/3,7/2,1), satisfying transform_vec4_point's input
/// precondition. Transform outputs retain four homogeneous coordinates; no row performs
/// perspective division. An input direction need not remain a direction under
/// these general homogeneous matrices: its output weight is -23/4 for the fixed rational
/// matrix and 3161/1008 for the full symbolic matrix.
///
/// The no-translation matrices have an entirely zero final column, including the
/// bottom-right entry; both are singular, not standard affine homogeneous matrices.
/// The rational one also has a zero final row and always outputs zero weight.
/// The symbolic one keeps three nonzero coefficients in that row. These rows use
/// ordinary matrix/vector multiplication; no inverse or Cartesian conversion of
/// those singular matrices is attempted.
///
/// Ordinary iter includes cursor work where present, explicit clones, operations,
/// requested consumers and destruction. Native determinants borrow, inverses
/// consume clones, and ordinary products clone both sides. External baselines
/// clone the left matrix before borrowed adapter calls. Batch rows clone a matrix
/// and borrow four copies of one vector; each call constructs four outputs with
/// matrix/final-column facts shared within that call, not a persistent batch cache.
/// Fixtures and contexts are retained outside timing. Numerica/Symbolica Float and
/// Rug/MPFR request precision 128; Atom methods do not use their stored 128 as a
/// numeric precision setting. Output evaluation and cloning costs differ by engine.
///
/// | Transform consumer | Explicit requests after construction |
/// | --- | --- |
/// | Plain single/batch/point/direction | No primitive-float conversion or fact query |
/// | Single one-coord/all-coord approx | One / all four coordinates as optional lossy f64 |
/// | Batch one-coord/all-coord approx | First coordinate of first vector / all sixteen coordinates |
/// | Single structural or sign/zero facts | Sign and zero status for all four coordinates |
/// | Batch structural facts, including direction batches | Sign and zero status for all sixteen coordinates |
/// | Symbolic single sign refinement | refine_sign_until(0) on all four coordinates |
///
/// Approximation/sign Options and unknown structural facts are black-boxed without
/// requiring success. Native inverse Results are unwrapped. Seven binary hooks
/// visit four cases each; forty-two fixed native hooks mirror their timing rows.
/// These diagnostics assert neither demand isolation nor dispatch counts or
/// numerical correctness, and add no policy receipt, cancellation signal or total
/// resource bound. Context argument 128 is not a bound on exact scalar work.
fn bench_matrix4(c: &mut Criterion) {
    let mut group = c.benchmark_group("matrix4");
    let lhs_cases = sample_mat4_cases();
    let rhs_cases = sample_mat4_b_cases();
    let vector_cases = sample_vec4_cases();

    bench_blas_matrix4(&mut group, "hyperreal", lhs_cases, rhs_cases, vector_cases);

    {
        let rational_lhs = blas_mat4_rational();
        let rational_rhs = blas_mat4_b_rational();
        let rational_vector = blas_vec4_rational();
        let symbolic_mat4 = Matrix4::new([
            [Real::pi(), Real::e(), q(-5, 7), q(3, 2)],
            [q(13, 8), q(-3, 4), Real::pi(), q(2, 5)],
            [q(7, 3), Real::e(), q(-11, 13), q(1, 7)],
            [q(5, 8), q(-4, 9), q(2, 3), q(1, 1)],
        ]);
        let symbolic_linear_lhs = Matrix4::new([
            [Real::pi(), Real::e(), q(-5, 7), q(0, 1)],
            [q(13, 8), q(-3, 4), Real::pi(), q(0, 1)],
            [q(7, 3), Real::e(), q(-11, 13), q(0, 1)],
            [q(5, 8), q(-4, 9), q(2, 3), q(0, 1)],
        ]);
        let rational_linear_lhs = Matrix4::new([
            [q(2, 1), q(-1, 3), q(5, 6), q(0, 1)],
            [q(-3, 2), q(7, 5), q(1, 4), q(0, 1)],
            [q(9, 7), q(4, 3), q(11, 5), q(0, 1)],
            [q(0, 1), q(0, 1), q(0, 1), q(0, 1)],
        ]);
        let symbolic_vector = Vector4::new([Real::pi(), q(-2, 3), Real::e(), q(9, 2)]);
        let symbolic_vec_batch = [
            symbolic_vector.clone(),
            symbolic_vector.clone(),
            symbolic_vector.clone(),
            symbolic_vector.clone(),
        ];
        let rational_direction = Vector4::new([q(3, 2), q(-11, 7), q(9, 4), q(0, 1)]);
        let rational_direction_batch = [
            rational_direction.clone(),
            rational_direction.clone(),
            rational_direction.clone(),
            rational_direction.clone(),
        ];
        let rational_vec_batch = [
            rational_vector.clone(),
            rational_vector.clone(),
            rational_vector.clone(),
            rational_vector.clone(),
        ];
        let symbolic_point = Vector4::new([Real::pi(), q(5, 3), q(7, 2), q(1, 1)]);
        let symbolic_direction = Vector4::new([q(3, 2), q(-11, 7), q(9, 4), q(0, 1)]);
        let symbolic_direction_batch = [
            symbolic_direction.clone(),
            symbolic_direction.clone(),
            symbolic_direction.clone(),
            symbolic_direction.clone(),
        ];
        trace_dispatch_row("matrix4/hyperreal-rational/mat4 determinant", || {
            black_box(black_box(&rational_lhs).determinant());
        });
        trace_dispatch_row("matrix4/hyperreal-rational/mat4 inverse", || {
            black_box(black_box(rational_lhs.clone()).inverse().unwrap());
        });
        trace_dispatch_row("matrix4/hyperreal-rational/mat4 mul mat4", || {
            black_box(black_box(rational_lhs.clone()) * black_box(rational_rhs.clone()));
        });
        trace_dispatch_row("matrix4/hyperreal-rational/mat4 transform vec4", || {
            black_box(black_box(rational_lhs.clone()) * black_box(rational_vector.clone()));
        });
        trace_dispatch_row(
            "matrix4/hyperreal-rational/mat4 transform vec4 batch",
            || {
                black_box(
                    black_box(rational_lhs.clone())
                        .transform_vec4_batch(black_box(&rational_vec_batch)),
                );
            },
        );
        // Request one coordinate after batch construction; no cost bound is asserted.
        trace_dispatch_row(
            "matrix4/hyperreal-rational/mat4 transform vec4 batch one-coord approx",
            || {
                let transformed = black_box(
                    black_box(rational_lhs.clone())
                        .transform_vec4_batch(black_box(&rational_vec_batch)),
                );
                black_box(transformed[0][0].to_f64_lossy());
            },
        );
        trace_dispatch_row(
            "matrix4/hyperreal-rational/mat4 transform vec4 batch all-coord approx",
            || {
                let transformed = black_box(
                    black_box(rational_lhs.clone())
                        .transform_vec4_batch(black_box(&rational_vec_batch)),
                );
                black_box((
                    transformed[0][0].to_f64_lossy(),
                    transformed[0][1].to_f64_lossy(),
                    transformed[0][2].to_f64_lossy(),
                    transformed[0][3].to_f64_lossy(),
                    transformed[1][0].to_f64_lossy(),
                    transformed[1][1].to_f64_lossy(),
                    transformed[1][2].to_f64_lossy(),
                    transformed[1][3].to_f64_lossy(),
                    transformed[2][0].to_f64_lossy(),
                    transformed[2][1].to_f64_lossy(),
                    transformed[2][2].to_f64_lossy(),
                    transformed[2][3].to_f64_lossy(),
                    transformed[3][0].to_f64_lossy(),
                    transformed[3][1].to_f64_lossy(),
                    transformed[3][2].to_f64_lossy(),
                    transformed[3][3].to_f64_lossy(),
                ));
            },
        );
        // Construct the batch, then request sign/zero facts without float conversion.
        trace_dispatch_row(
            "matrix4/hyperreal-rational/mat4 transform vec4 batch structural facts",
            || {
                let transformed = black_box(
                    black_box(rational_lhs.clone())
                        .transform_vec4_batch(black_box(&rational_vec_batch)),
                );
                let _ = black_box((
                    transformed[0][0].structural_facts().sign,
                    transformed[0][1].structural_facts().sign,
                    transformed[0][2].structural_facts().sign,
                    transformed[0][3].structural_facts().sign,
                    transformed[1][0].structural_facts().sign,
                    transformed[1][1].structural_facts().sign,
                    transformed[1][2].structural_facts().sign,
                    transformed[1][3].structural_facts().sign,
                    transformed[2][0].structural_facts().sign,
                    transformed[2][1].structural_facts().sign,
                    transformed[2][2].structural_facts().sign,
                    transformed[2][3].structural_facts().sign,
                    transformed[3][0].structural_facts().sign,
                    transformed[3][1].structural_facts().sign,
                    transformed[3][2].structural_facts().sign,
                    transformed[3][3].structural_facts().sign,
                    transformed[0][0].zero_status(),
                    transformed[0][1].zero_status(),
                    transformed[0][2].zero_status(),
                    transformed[0][3].zero_status(),
                    transformed[1][0].zero_status(),
                    transformed[1][1].zero_status(),
                    transformed[1][2].zero_status(),
                    transformed[1][3].zero_status(),
                    transformed[2][0].zero_status(),
                    transformed[2][1].zero_status(),
                    transformed[2][2].zero_status(),
                    transformed[2][3].zero_status(),
                    transformed[3][0].zero_status(),
                    transformed[3][1].zero_status(),
                    transformed[3][2].zero_status(),
                    transformed[3][3].zero_status(),
                ));
            },
        );
        // Input weights are zero; the last column contributes nothing to these products.
        trace_dispatch_row(
            "matrix4/hyperreal-rational/mat4 transform vec4 batch direction",
            || {
                black_box(
                    black_box(rational_lhs.clone())
                        .transform_vec4_batch(black_box(&rational_direction_batch)),
                );
            },
        );
        trace_dispatch_row(
            "matrix4/hyperreal-rational/mat4 transform vec4 batch direction one-coord approx",
            || {
                let transformed = black_box(
                    black_box(rational_lhs.clone())
                        .transform_vec4_batch(black_box(&rational_direction_batch)),
                );
                black_box(transformed[0][0].to_f64_lossy());
            },
        );
        trace_dispatch_row(
            "matrix4/hyperreal-rational/mat4 transform vec4 batch direction all-coord approx",
            || {
                let transformed = black_box(
                    black_box(rational_lhs.clone())
                        .transform_vec4_batch(black_box(&rational_direction_batch)),
                );
                black_box((
                    transformed[0][0].to_f64_lossy(),
                    transformed[0][1].to_f64_lossy(),
                    transformed[0][2].to_f64_lossy(),
                    transformed[0][3].to_f64_lossy(),
                    transformed[1][0].to_f64_lossy(),
                    transformed[1][1].to_f64_lossy(),
                    transformed[1][2].to_f64_lossy(),
                    transformed[1][3].to_f64_lossy(),
                    transformed[2][0].to_f64_lossy(),
                    transformed[2][1].to_f64_lossy(),
                    transformed[2][2].to_f64_lossy(),
                    transformed[2][3].to_f64_lossy(),
                    transformed[3][0].to_f64_lossy(),
                    transformed[3][1].to_f64_lossy(),
                    transformed[3][2].to_f64_lossy(),
                    transformed[3][3].to_f64_lossy(),
                ));
            },
        );
        // Query every direction-batch coordinate without explicitly requesting float output.
        trace_dispatch_row(
            "matrix4/hyperreal-rational/mat4 transform vec4 batch direction structural facts",
            || {
                let transformed = black_box(
                    black_box(rational_lhs.clone())
                        .transform_vec4_batch(black_box(&rational_direction_batch)),
                );
                let _ = black_box((
                    transformed[0][0].structural_facts().sign,
                    transformed[0][1].structural_facts().sign,
                    transformed[0][2].structural_facts().sign,
                    transformed[0][3].structural_facts().sign,
                    transformed[1][0].structural_facts().sign,
                    transformed[1][1].structural_facts().sign,
                    transformed[1][2].structural_facts().sign,
                    transformed[1][3].structural_facts().sign,
                    transformed[2][0].structural_facts().sign,
                    transformed[2][1].structural_facts().sign,
                    transformed[2][2].structural_facts().sign,
                    transformed[2][3].structural_facts().sign,
                    transformed[3][0].structural_facts().sign,
                    transformed[3][1].structural_facts().sign,
                    transformed[3][2].structural_facts().sign,
                    transformed[3][3].structural_facts().sign,
                    transformed[0][0].zero_status(),
                    transformed[0][1].zero_status(),
                    transformed[0][2].zero_status(),
                    transformed[0][3].zero_status(),
                    transformed[1][0].zero_status(),
                    transformed[1][1].zero_status(),
                    transformed[1][2].zero_status(),
                    transformed[1][3].zero_status(),
                    transformed[2][0].zero_status(),
                    transformed[2][1].zero_status(),
                    transformed[2][2].zero_status(),
                    transformed[2][3].zero_status(),
                    transformed[3][0].zero_status(),
                    transformed[3][1].zero_status(),
                    transformed[3][2].zero_status(),
                    transformed[3][3].zero_status(),
                ));
            },
        );
        trace_dispatch_row(
            "matrix4/hyperreal-rational/mat4 transform vec4 no-translation",
            || {
                black_box(
                    black_box(rational_linear_lhs.clone()) * black_box(rational_vector.clone()),
                );
            },
        );
        // Compare explicit approximation requests after constructing the transform.
        trace_dispatch_row(
            "matrix4/hyperreal-rational/mat4 transform vec4 no-translation one-coord approx",
            || {
                let transformed = black_box(
                    black_box(rational_linear_lhs.clone()) * black_box(rational_vector.clone()),
                );
                black_box(transformed[0].to_f64_lossy());
            },
        );
        trace_dispatch_row(
            "matrix4/hyperreal-rational/mat4 transform vec4 no-translation all-coord approx",
            || {
                let transformed = black_box(
                    black_box(rational_linear_lhs.clone()) * black_box(rational_vector.clone()),
                );
                black_box((
                    transformed[0].to_f64_lossy(),
                    transformed[1].to_f64_lossy(),
                    transformed[2].to_f64_lossy(),
                    transformed[3].to_f64_lossy(),
                ));
            },
        );
        // Request structural facts without explicit float conversion or work assertions.
        trace_dispatch_row(
            "matrix4/hyperreal-rational/mat4 transform vec4 no-translation structural facts",
            || {
                let transformed = black_box(
                    black_box(rational_linear_lhs.clone()) * black_box(rational_vector.clone()),
                );
                let _ = black_box((
                    transformed[0].structural_facts().sign,
                    transformed[1].structural_facts().sign,
                    transformed[2].structural_facts().sign,
                    transformed[3].structural_facts().sign,
                    transformed[0].zero_status(),
                    transformed[1].zero_status(),
                    transformed[2].zero_status(),
                    transformed[3].zero_status(),
                ));
            },
        );
        // Build the transform before querying sign/zero facts, without explicit float output.
        trace_dispatch_row(
            "matrix4/hyperreal-rational/mat4 transform vec4 sign/zero facts",
            || {
                let transformed =
                    black_box(black_box(rational_lhs.clone()) * black_box(rational_vector.clone()));
                black_box((
                    transformed[0].structural_facts().sign,
                    transformed[1].structural_facts().sign,
                    transformed[2].structural_facts().sign,
                    transformed[3].structural_facts().sign,
                    transformed[0].zero_status(),
                    transformed[1].zero_status(),
                    transformed[2].zero_status(),
                    transformed[3].zero_status(),
                ));
            },
        );
        trace_dispatch_row(
            "matrix4/hyperreal-rational/mat4 transform vec4 one-coord approx",
            || {
                let transformed =
                    black_box(black_box(rational_lhs.clone()) * black_box(rational_vector.clone()));
                black_box(transformed[0].to_f64_lossy());
            },
        );
        trace_dispatch_row(
            "matrix4/hyperreal-rational/mat4 transform vec4 all-coord approx",
            || {
                let transformed =
                    black_box(black_box(rational_lhs.clone()) * black_box(rational_vector.clone()));
                black_box((
                    transformed[0].to_f64_lossy(),
                    transformed[1].to_f64_lossy(),
                    transformed[2].to_f64_lossy(),
                    transformed[3].to_f64_lossy(),
                ));
            },
        );
        // Exercise symbolic matrix/vector products without asserting a dispatch route.
        trace_dispatch_row("matrix4/hyperreal-symbolic/mat4 transform vec4", || {
            black_box(black_box(symbolic_mat4.clone()) * black_box(symbolic_vector.clone()));
        });
        trace_dispatch_row(
            "matrix4/hyperreal-symbolic/mat4 transform vec4 one-coord approx",
            || {
                let transformed = black_box(
                    black_box(symbolic_mat4.clone()) * black_box(symbolic_vector.clone()),
                );
                black_box(transformed[0].to_f64_lossy());
            },
        );
        trace_dispatch_row(
            "matrix4/hyperreal-symbolic/mat4 transform vec4 all-coord approx",
            || {
                let transformed = black_box(
                    black_box(symbolic_mat4.clone()) * black_box(symbolic_vector.clone()),
                );
                black_box((
                    transformed[0].to_f64_lossy(),
                    transformed[1].to_f64_lossy(),
                    transformed[2].to_f64_lossy(),
                    transformed[3].to_f64_lossy(),
                ));
            },
        );
        // Build the immediate symbolic batch, then request selected output coordinates.
        trace_dispatch_row(
            "matrix4/hyperreal-symbolic/mat4 transform vec4 batch",
            || {
                black_box(
                    black_box(symbolic_mat4.clone())
                        .transform_vec4_batch(black_box(&symbolic_vec_batch)),
                );
            },
        );
        trace_dispatch_row(
            "matrix4/hyperreal-symbolic/mat4 transform vec4 batch one-coord approx",
            || {
                let transformed = black_box(
                    black_box(symbolic_mat4.clone())
                        .transform_vec4_batch(black_box(&symbolic_vec_batch)),
                );
                black_box(transformed[0][0].to_f64_lossy());
            },
        );
        trace_dispatch_row(
            "matrix4/hyperreal-symbolic/mat4 transform vec4 batch all-coord approx",
            || {
                let transformed = black_box(
                    black_box(symbolic_mat4.clone())
                        .transform_vec4_batch(black_box(&symbolic_vec_batch)),
                );
                black_box((
                    transformed[0][0].to_f64_lossy(),
                    transformed[0][1].to_f64_lossy(),
                    transformed[0][2].to_f64_lossy(),
                    transformed[0][3].to_f64_lossy(),
                    transformed[1][0].to_f64_lossy(),
                    transformed[1][1].to_f64_lossy(),
                    transformed[1][2].to_f64_lossy(),
                    transformed[1][3].to_f64_lossy(),
                    transformed[2][0].to_f64_lossy(),
                    transformed[2][1].to_f64_lossy(),
                    transformed[2][2].to_f64_lossy(),
                    transformed[2][3].to_f64_lossy(),
                    transformed[3][0].to_f64_lossy(),
                    transformed[3][1].to_f64_lossy(),
                    transformed[3][2].to_f64_lossy(),
                    transformed[3][3].to_f64_lossy(),
                ));
            },
        );
        // Share matrix facts within this immediate batch of zero-weight inputs.
        trace_dispatch_row(
            "matrix4/hyperreal-symbolic/mat4 transform vec4 batch direction",
            || {
                black_box(
                    black_box(symbolic_mat4.clone())
                        .transform_vec4_batch(black_box(&symbolic_direction_batch)),
                );
            },
        );
        // Request all batch sign/zero facts; no float conversion or dispatch assertion follows.
        trace_dispatch_row(
            "matrix4/hyperreal-symbolic/mat4 transform vec4 batch direction structural facts",
            || {
                let transformed = black_box(
                    black_box(symbolic_mat4.clone())
                        .transform_vec4_batch(black_box(&symbolic_direction_batch)),
                );
                let _ = black_box((
                    transformed[0][0].structural_facts().sign,
                    transformed[0][1].structural_facts().sign,
                    transformed[0][2].structural_facts().sign,
                    transformed[0][3].structural_facts().sign,
                    transformed[1][0].structural_facts().sign,
                    transformed[1][1].structural_facts().sign,
                    transformed[1][2].structural_facts().sign,
                    transformed[1][3].structural_facts().sign,
                    transformed[2][0].structural_facts().sign,
                    transformed[2][1].structural_facts().sign,
                    transformed[2][2].structural_facts().sign,
                    transformed[2][3].structural_facts().sign,
                    transformed[3][0].structural_facts().sign,
                    transformed[3][1].structural_facts().sign,
                    transformed[3][2].structural_facts().sign,
                    transformed[3][3].structural_facts().sign,
                    transformed[0][0].zero_status(),
                    transformed[0][1].zero_status(),
                    transformed[0][2].zero_status(),
                    transformed[0][3].zero_status(),
                    transformed[1][0].zero_status(),
                    transformed[1][1].zero_status(),
                    transformed[1][2].zero_status(),
                    transformed[1][3].zero_status(),
                    transformed[2][0].zero_status(),
                    transformed[2][1].zero_status(),
                    transformed[2][2].zero_status(),
                    transformed[2][3].zero_status(),
                    transformed[3][0].zero_status(),
                    transformed[3][1].zero_status(),
                    transformed[3][2].zero_status(),
                    transformed[3][3].zero_status(),
                ));
            },
        );
        // Construct the symbolic batch before requesting its sign/zero facts.
        trace_dispatch_row(
            "matrix4/hyperreal-symbolic/mat4 transform vec4 batch structural facts",
            || {
                let transformed = black_box(
                    black_box(symbolic_mat4.clone())
                        .transform_vec4_batch(black_box(&symbolic_vec_batch)),
                );
                let _ = black_box((
                    transformed[0][0].structural_facts().sign,
                    transformed[0][1].structural_facts().sign,
                    transformed[0][2].structural_facts().sign,
                    transformed[0][3].structural_facts().sign,
                    transformed[1][0].structural_facts().sign,
                    transformed[1][1].structural_facts().sign,
                    transformed[1][2].structural_facts().sign,
                    transformed[1][3].structural_facts().sign,
                    transformed[2][0].structural_facts().sign,
                    transformed[2][1].structural_facts().sign,
                    transformed[2][2].structural_facts().sign,
                    transformed[2][3].structural_facts().sign,
                    transformed[3][0].structural_facts().sign,
                    transformed[3][1].structural_facts().sign,
                    transformed[3][2].structural_facts().sign,
                    transformed[3][3].structural_facts().sign,
                    transformed[0][0].zero_status(),
                    transformed[0][1].zero_status(),
                    transformed[0][2].zero_status(),
                    transformed[0][3].zero_status(),
                    transformed[1][0].zero_status(),
                    transformed[1][1].zero_status(),
                    transformed[1][2].zero_status(),
                    transformed[1][3].zero_status(),
                    transformed[2][0].zero_status(),
                    transformed[2][1].zero_status(),
                    transformed[2][2].zero_status(),
                    transformed[2][3].zero_status(),
                    transformed[3][0].zero_status(),
                    transformed[3][1].zero_status(),
                    transformed[3][2].zero_status(),
                    transformed[3][3].zero_status(),
                ));
            },
        );
        trace_dispatch_row(
            "matrix4/hyperreal-symbolic/mat4 transform vec4 batch direction one-coord approx",
            || {
                let transformed = black_box(
                    black_box(symbolic_mat4.clone())
                        .transform_vec4_batch(black_box(&symbolic_direction_batch)),
                );
                black_box(transformed[0][0].to_f64_lossy());
            },
        );
        trace_dispatch_row(
            "matrix4/hyperreal-symbolic/mat4 transform vec4 batch direction all-coord approx",
            || {
                let transformed = black_box(
                    black_box(symbolic_mat4.clone())
                        .transform_vec4_batch(black_box(&symbolic_direction_batch)),
                );
                black_box((
                    transformed[0][0].to_f64_lossy(),
                    transformed[0][1].to_f64_lossy(),
                    transformed[0][2].to_f64_lossy(),
                    transformed[0][3].to_f64_lossy(),
                    transformed[1][0].to_f64_lossy(),
                    transformed[1][1].to_f64_lossy(),
                    transformed[1][2].to_f64_lossy(),
                    transformed[1][3].to_f64_lossy(),
                    transformed[2][0].to_f64_lossy(),
                    transformed[2][1].to_f64_lossy(),
                    transformed[2][2].to_f64_lossy(),
                    transformed[2][3].to_f64_lossy(),
                    transformed[3][0].to_f64_lossy(),
                    transformed[3][1].to_f64_lossy(),
                    transformed[3][2].to_f64_lossy(),
                    transformed[3][3].to_f64_lossy(),
                ));
            },
        );
        trace_dispatch_row(
            "matrix4/hyperreal-symbolic/mat4 transform vec4 sign refinement",
            || {
                let transformed = black_box(
                    black_box(symbolic_mat4.clone()) * black_box(symbolic_vector.clone()),
                );
                black_box((
                    transformed[0].refine_sign_until(0),
                    transformed[1].refine_sign_until(0),
                    transformed[2].refine_sign_until(0),
                    transformed[3].refine_sign_until(0),
                ));
            },
        );
        // A zero final column removes its homogeneous contribution; this matrix is singular.
        trace_dispatch_row(
            "matrix4/hyperreal-symbolic/mat4 transform vec4 no-translation",
            || {
                black_box(
                    black_box(symbolic_linear_lhs.clone()) * black_box(symbolic_vector.clone()),
                );
            },
        );
        trace_dispatch_row(
            "matrix4/hyperreal-symbolic/mat4 transform vec4 no-translation one-coord approx",
            || {
                let transformed = black_box(
                    black_box(symbolic_linear_lhs.clone()) * black_box(symbolic_vector.clone()),
                );
                black_box(transformed[0].to_f64_lossy());
            },
        );
        trace_dispatch_row(
            "matrix4/hyperreal-symbolic/mat4 transform vec4 no-translation all-coord approx",
            || {
                let transformed = black_box(
                    black_box(symbolic_linear_lhs.clone()) * black_box(symbolic_vector.clone()),
                );
                black_box((
                    transformed[0].to_f64_lossy(),
                    transformed[1].to_f64_lossy(),
                    transformed[2].to_f64_lossy(),
                    transformed[3].to_f64_lossy(),
                ));
            },
        );
        trace_dispatch_row(
            "matrix4/hyperreal-symbolic/mat4 transform vec4 no-translation structural facts",
            || {
                let transformed = black_box(
                    black_box(symbolic_linear_lhs.clone()) * black_box(symbolic_vector.clone()),
                );
                let _ = black_box((
                    transformed[0].structural_facts().sign,
                    transformed[1].structural_facts().sign,
                    transformed[2].structural_facts().sign,
                    transformed[3].structural_facts().sign,
                    transformed[0].zero_status(),
                    transformed[1].zero_status(),
                    transformed[2].zero_status(),
                    transformed[3].zero_status(),
                ));
            },
        );
        // Point/direction semantics are predicate-relevant workloads in this stack.
        trace_dispatch_row(
            "matrix4/hyperreal-rational/mat4 transform point vec4",
            || {
                black_box(
                    black_box(rational_lhs.clone())
                        .transform_vec4_point(black_box(&symbolic_point)),
                );
            },
        );
        trace_dispatch_row(
            "matrix4/hyperreal-rational/mat4 transform direction vec4",
            || {
                black_box(black_box(rational_lhs.clone()) * black_box(symbolic_direction.clone()));
            },
        );
        // Query direction-result facts without explicitly requesting float output.
        trace_dispatch_row(
            "matrix4/hyperreal-rational/mat4 transform direction vec4 structural facts",
            || {
                let transformed = black_box(
                    black_box(rational_lhs.clone()) * black_box(symbolic_direction.clone()),
                );
                let _ = black_box((
                    transformed[0].structural_facts().sign,
                    transformed[1].structural_facts().sign,
                    transformed[2].structural_facts().sign,
                    transformed[3].structural_facts().sign,
                    transformed[0].zero_status(),
                    transformed[1].zero_status(),
                    transformed[2].zero_status(),
                    transformed[3].zero_status(),
                ));
            },
        );
        // The input has w=0; the projective output is not required to have zero weight.
        trace_dispatch_row(
            "matrix4/hyperreal-symbolic/mat4 transform direction vec4",
            || {
                black_box(black_box(symbolic_mat4.clone()) * black_box(symbolic_direction.clone()));
            },
        );
        // Query all four symbolic direction-result coordinates for sign/zero facts.
        trace_dispatch_row(
            "matrix4/hyperreal-symbolic/mat4 transform direction vec4 structural facts",
            || {
                let transformed = black_box(
                    black_box(symbolic_mat4.clone()) * black_box(symbolic_direction.clone()),
                );
                let _ = black_box((
                    transformed[0].structural_facts().sign,
                    transformed[1].structural_facts().sign,
                    transformed[2].structural_facts().sign,
                    transformed[3].structural_facts().sign,
                    transformed[0].zero_status(),
                    transformed[1].zero_status(),
                    transformed[2].zero_status(),
                    transformed[3].zero_status(),
                ));
            },
        );
        trace_dispatch_row(
            "matrix4/hyperreal-symbolic/mat4 transform direction vec4 one-coord approx",
            || {
                let transformed = black_box(
                    black_box(symbolic_mat4.clone()) * black_box(symbolic_direction.clone()),
                );
                black_box(transformed[0].to_f64_lossy());
            },
        );
        trace_dispatch_row(
            "matrix4/hyperreal-symbolic/mat4 transform direction vec4 all-coord approx",
            || {
                let transformed = black_box(
                    black_box(symbolic_mat4.clone()) * black_box(symbolic_direction.clone()),
                );
                black_box((
                    transformed[0].to_f64_lossy(),
                    transformed[1].to_f64_lossy(),
                    transformed[2].to_f64_lossy(),
                    transformed[3].to_f64_lossy(),
                ));
            },
        );
        group.bench_function("hyperreal-rational/mat4 determinant", |b| {
            b.iter(|| black_box(black_box(&rational_lhs).determinant()))
        });
        group.bench_function("hyperreal-rational/mat4 inverse", |b| {
            b.iter(|| black_box(black_box(rational_lhs.clone()).inverse().unwrap()))
        });
        group.bench_function("hyperreal-rational/mat4 mul mat4", |b| {
            b.iter(|| black_box(black_box(rational_lhs.clone()) * black_box(rational_rhs.clone())))
        });
        group.bench_function("hyperreal-rational/mat4 transform vec4", |b| {
            b.iter(|| {
                black_box(black_box(rational_lhs.clone()) * black_box(rational_vector.clone()))
            })
        });
        group.bench_function("hyperreal-rational/mat4 transform vec4 batch", |b| {
            b.iter(|| {
                black_box(
                    black_box(rational_lhs.clone())
                        .transform_vec4_batch(black_box(&rational_vec_batch)),
                )
            })
        });
        group.bench_function(
            "hyperreal-rational/mat4 transform vec4 batch one-coord approx",
            |b| {
                b.iter(|| {
                    let transformed = black_box(
                        black_box(rational_lhs.clone())
                            .transform_vec4_batch(black_box(&rational_vec_batch)),
                    );
                    black_box(transformed[0][0].to_f64_lossy())
                })
            },
        );
        group.bench_function(
            "hyperreal-rational/mat4 transform vec4 batch structural facts",
            |b| {
                b.iter(|| {
                    let transformed = black_box(
                        black_box(rational_lhs.clone())
                            .transform_vec4_batch(black_box(&rational_vec_batch)),
                    );
                    black_box((
                        transformed[0][0].structural_facts().sign,
                        transformed[0][1].structural_facts().sign,
                        transformed[0][2].structural_facts().sign,
                        transformed[0][3].structural_facts().sign,
                        transformed[1][0].structural_facts().sign,
                        transformed[1][1].structural_facts().sign,
                        transformed[1][2].structural_facts().sign,
                        transformed[1][3].structural_facts().sign,
                        transformed[2][0].structural_facts().sign,
                        transformed[2][1].structural_facts().sign,
                        transformed[2][2].structural_facts().sign,
                        transformed[2][3].structural_facts().sign,
                        transformed[3][0].structural_facts().sign,
                        transformed[3][1].structural_facts().sign,
                        transformed[3][2].structural_facts().sign,
                        transformed[3][3].structural_facts().sign,
                        transformed[0][0].zero_status(),
                        transformed[0][1].zero_status(),
                        transformed[0][2].zero_status(),
                        transformed[0][3].zero_status(),
                        transformed[1][0].zero_status(),
                        transformed[1][1].zero_status(),
                        transformed[1][2].zero_status(),
                        transformed[1][3].zero_status(),
                        transformed[2][0].zero_status(),
                        transformed[2][1].zero_status(),
                        transformed[2][2].zero_status(),
                        transformed[2][3].zero_status(),
                        transformed[3][0].zero_status(),
                        transformed[3][1].zero_status(),
                        transformed[3][2].zero_status(),
                        transformed[3][3].zero_status(),
                    ))
                })
            },
        );
        group.bench_function(
            "hyperreal-rational/mat4 transform vec4 batch all-coord approx",
            |b| {
                b.iter(|| {
                    let transformed = black_box(
                        black_box(rational_lhs.clone())
                            .transform_vec4_batch(black_box(&rational_vec_batch)),
                    );
                    black_box((
                        transformed[0][0].to_f64_lossy(),
                        transformed[0][1].to_f64_lossy(),
                        transformed[0][2].to_f64_lossy(),
                        transformed[0][3].to_f64_lossy(),
                        transformed[1][0].to_f64_lossy(),
                        transformed[1][1].to_f64_lossy(),
                        transformed[1][2].to_f64_lossy(),
                        transformed[1][3].to_f64_lossy(),
                        transformed[2][0].to_f64_lossy(),
                        transformed[2][1].to_f64_lossy(),
                        transformed[2][2].to_f64_lossy(),
                        transformed[2][3].to_f64_lossy(),
                        transformed[3][0].to_f64_lossy(),
                        transformed[3][1].to_f64_lossy(),
                        transformed[3][2].to_f64_lossy(),
                        transformed[3][3].to_f64_lossy(),
                    ))
                })
            },
        );
        group.bench_function(
            "hyperreal-rational/mat4 transform vec4 batch direction",
            |b| {
                b.iter(|| {
                    black_box(
                        black_box(rational_lhs.clone())
                            .transform_vec4_batch(black_box(&rational_direction_batch)),
                    )
                })
            },
        );
        group.bench_function(
            "hyperreal-rational/mat4 transform vec4 batch direction one-coord approx",
            |b| {
                b.iter(|| {
                    let transformed = black_box(
                        black_box(rational_lhs.clone())
                            .transform_vec4_batch(black_box(&rational_direction_batch)),
                    );
                    black_box(transformed[0][0].to_f64_lossy())
                })
            },
        );
        group.bench_function(
            "hyperreal-rational/mat4 transform vec4 batch direction all-coord approx",
            |b| {
                b.iter(|| {
                    let transformed = black_box(
                        black_box(rational_lhs.clone())
                            .transform_vec4_batch(black_box(&rational_direction_batch)),
                    );
                    black_box((
                        transformed[0][0].to_f64_lossy(),
                        transformed[0][1].to_f64_lossy(),
                        transformed[0][2].to_f64_lossy(),
                        transformed[0][3].to_f64_lossy(),
                        transformed[1][0].to_f64_lossy(),
                        transformed[1][1].to_f64_lossy(),
                        transformed[1][2].to_f64_lossy(),
                        transformed[1][3].to_f64_lossy(),
                        transformed[2][0].to_f64_lossy(),
                        transformed[2][1].to_f64_lossy(),
                        transformed[2][2].to_f64_lossy(),
                        transformed[2][3].to_f64_lossy(),
                        transformed[3][0].to_f64_lossy(),
                        transformed[3][1].to_f64_lossy(),
                        transformed[3][2].to_f64_lossy(),
                        transformed[3][3].to_f64_lossy(),
                    ))
                })
            },
        );
        // Measure batch construction plus sign/zero queries without explicit float output.
        group.bench_function(
            "hyperreal-rational/mat4 transform vec4 batch direction structural facts",
            |b| {
                b.iter(|| {
                    let transformed = black_box(
                        black_box(rational_lhs.clone())
                            .transform_vec4_batch(black_box(&rational_direction_batch)),
                    );
                    black_box((
                        transformed[0][0].structural_facts().sign,
                        transformed[0][1].structural_facts().sign,
                        transformed[0][2].structural_facts().sign,
                        transformed[0][3].structural_facts().sign,
                        transformed[1][0].structural_facts().sign,
                        transformed[1][1].structural_facts().sign,
                        transformed[1][2].structural_facts().sign,
                        transformed[1][3].structural_facts().sign,
                        transformed[2][0].structural_facts().sign,
                        transformed[2][1].structural_facts().sign,
                        transformed[2][2].structural_facts().sign,
                        transformed[2][3].structural_facts().sign,
                        transformed[3][0].structural_facts().sign,
                        transformed[3][1].structural_facts().sign,
                        transformed[3][2].structural_facts().sign,
                        transformed[3][3].structural_facts().sign,
                        transformed[0][0].zero_status(),
                        transformed[0][1].zero_status(),
                        transformed[0][2].zero_status(),
                        transformed[0][3].zero_status(),
                        transformed[1][0].zero_status(),
                        transformed[1][1].zero_status(),
                        transformed[1][2].zero_status(),
                        transformed[1][3].zero_status(),
                        transformed[2][0].zero_status(),
                        transformed[2][1].zero_status(),
                        transformed[2][2].zero_status(),
                        transformed[2][3].zero_status(),
                        transformed[3][0].zero_status(),
                        transformed[3][1].zero_status(),
                        transformed[3][2].zero_status(),
                        transformed[3][3].zero_status(),
                    ))
                })
            },
        );
        group.bench_function(
            "hyperreal-rational/mat4 transform vec4 no-translation",
            |b| {
                b.iter(|| {
                    black_box(
                        black_box(rational_linear_lhs.clone()) * black_box(rational_vector.clone()),
                    )
                })
            },
        );
        group.bench_function(
            "hyperreal-rational/mat4 transform vec4 no-translation one-coord approx",
            |b| {
                b.iter(|| {
                    let transformed = black_box(
                        black_box(rational_linear_lhs.clone()) * black_box(rational_vector.clone()),
                    );
                    black_box(transformed[0].to_f64_lossy())
                })
            },
        );
        group.bench_function(
            "hyperreal-rational/mat4 transform vec4 no-translation all-coord approx",
            |b| {
                b.iter(|| {
                    let transformed = black_box(
                        black_box(rational_linear_lhs.clone()) * black_box(rational_vector.clone()),
                    );
                    black_box((
                        transformed[0].to_f64_lossy(),
                        transformed[1].to_f64_lossy(),
                        transformed[2].to_f64_lossy(),
                        transformed[3].to_f64_lossy(),
                    ))
                })
            },
        );
        group.bench_function(
            "hyperreal-rational/mat4 transform vec4 no-translation structural facts",
            |b| {
                b.iter(|| {
                    let transformed = black_box(
                        black_box(rational_linear_lhs.clone()) * black_box(rational_vector.clone()),
                    );
                    black_box((
                        transformed[0].structural_facts().sign,
                        transformed[1].structural_facts().sign,
                        transformed[2].structural_facts().sign,
                        transformed[3].structural_facts().sign,
                        transformed[0].zero_status(),
                        transformed[1].zero_status(),
                        transformed[2].zero_status(),
                        transformed[3].zero_status(),
                    ))
                })
            },
        );
        group.bench_function(
            "hyperreal-rational/mat4 transform vec4 sign/zero facts",
            |b| {
                b.iter(|| {
                    let transformed = black_box(
                        black_box(rational_lhs.clone()) * black_box(rational_vector.clone()),
                    );
                    black_box((
                        transformed[0].structural_facts().sign,
                        transformed[1].structural_facts().sign,
                        transformed[2].structural_facts().sign,
                        transformed[3].structural_facts().sign,
                        transformed[0].zero_status(),
                        transformed[1].zero_status(),
                        transformed[2].zero_status(),
                        transformed[3].zero_status(),
                    ))
                })
            },
        );
        group.bench_function(
            "hyperreal-rational/mat4 transform vec4 one-coord approx",
            |b| {
                b.iter(|| {
                    let transformed = black_box(
                        black_box(rational_lhs.clone()) * black_box(rational_vector.clone()),
                    );
                    black_box(transformed[0].to_f64_lossy())
                })
            },
        );
        group.bench_function(
            "hyperreal-rational/mat4 transform vec4 all-coord approx",
            |b| {
                b.iter(|| {
                    let transformed = black_box(
                        black_box(rational_lhs.clone()) * black_box(rational_vector.clone()),
                    );
                    black_box((
                        transformed[0].to_f64_lossy(),
                        transformed[1].to_f64_lossy(),
                        transformed[2].to_f64_lossy(),
                        transformed[3].to_f64_lossy(),
                    ))
                })
            },
        );
        // Measure the symbolic zero-column product; this row asserts no demand behavior.
        group.bench_function(
            "hyperreal-symbolic/mat4 transform vec4 no-translation",
            |b| {
                b.iter(|| {
                    black_box(
                        black_box(symbolic_linear_lhs.clone()) * black_box(symbolic_vector.clone()),
                    )
                })
            },
        );
        group.bench_function(
            "hyperreal-symbolic/mat4 transform vec4 no-translation one-coord approx",
            |b| {
                b.iter(|| {
                    let transformed = black_box(
                        black_box(symbolic_linear_lhs.clone()) * black_box(symbolic_vector.clone()),
                    );
                    black_box(transformed[0].to_f64_lossy())
                })
            },
        );
        group.bench_function(
            "hyperreal-symbolic/mat4 transform vec4 no-translation all-coord approx",
            |b| {
                b.iter(|| {
                    let transformed = black_box(
                        black_box(symbolic_linear_lhs.clone()) * black_box(symbolic_vector.clone()),
                    );
                    black_box((
                        transformed[0].to_f64_lossy(),
                        transformed[1].to_f64_lossy(),
                        transformed[2].to_f64_lossy(),
                        transformed[3].to_f64_lossy(),
                    ))
                })
            },
        );
        group.bench_function(
            "hyperreal-symbolic/mat4 transform vec4 sign refinement",
            |b| {
                b.iter(|| {
                    let transformed = black_box(
                        black_box(symbolic_mat4.clone()) * black_box(symbolic_vector.clone()),
                    );
                    black_box((
                        transformed[0].refine_sign_until(0),
                        transformed[1].refine_sign_until(0),
                        transformed[2].refine_sign_until(0),
                        transformed[3].refine_sign_until(0),
                    ))
                })
            },
        );
        // Time symbolic construction and the explicitly requested consumers separately.
        group.bench_function("hyperreal-symbolic/mat4 transform vec4", |b| {
            b.iter(|| {
                black_box(black_box(symbolic_mat4.clone()) * black_box(symbolic_vector.clone()))
            })
        });
        group.bench_function(
            "hyperreal-symbolic/mat4 transform vec4 one-coord approx",
            |b| {
                b.iter(|| {
                    let transformed = black_box(
                        black_box(symbolic_mat4.clone()) * black_box(symbolic_vector.clone()),
                    );
                    black_box(transformed[0].to_f64_lossy())
                })
            },
        );
        group.bench_function(
            "hyperreal-symbolic/mat4 transform vec4 all-coord approx",
            |b| {
                b.iter(|| {
                    let transformed = black_box(
                        black_box(symbolic_mat4.clone()) * black_box(symbolic_vector.clone()),
                    );
                    black_box((
                        transformed[0].to_f64_lossy(),
                        transformed[1].to_f64_lossy(),
                        transformed[2].to_f64_lossy(),
                        transformed[3].to_f64_lossy(),
                    ))
                })
            },
        );
        group.bench_function("hyperreal-symbolic/mat4 transform vec4 batch", |b| {
            b.iter(|| {
                black_box(
                    black_box(symbolic_mat4.clone())
                        .transform_vec4_batch(black_box(&symbolic_vec_batch)),
                )
            })
        });
        group.bench_function(
            "hyperreal-symbolic/mat4 transform vec4 batch structural facts",
            |b| {
                b.iter(|| {
                    let transformed = black_box(
                        black_box(symbolic_mat4.clone())
                            .transform_vec4_batch(black_box(&symbolic_vec_batch)),
                    );
                    black_box((
                        transformed[0][0].structural_facts().sign,
                        transformed[0][1].structural_facts().sign,
                        transformed[0][2].structural_facts().sign,
                        transformed[0][3].structural_facts().sign,
                        transformed[1][0].structural_facts().sign,
                        transformed[1][1].structural_facts().sign,
                        transformed[1][2].structural_facts().sign,
                        transformed[1][3].structural_facts().sign,
                        transformed[2][0].structural_facts().sign,
                        transformed[2][1].structural_facts().sign,
                        transformed[2][2].structural_facts().sign,
                        transformed[2][3].structural_facts().sign,
                        transformed[3][0].structural_facts().sign,
                        transformed[3][1].structural_facts().sign,
                        transformed[3][2].structural_facts().sign,
                        transformed[3][3].structural_facts().sign,
                        transformed[0][0].zero_status(),
                        transformed[0][1].zero_status(),
                        transformed[0][2].zero_status(),
                        transformed[0][3].zero_status(),
                        transformed[1][0].zero_status(),
                        transformed[1][1].zero_status(),
                        transformed[1][2].zero_status(),
                        transformed[1][3].zero_status(),
                        transformed[2][0].zero_status(),
                        transformed[2][1].zero_status(),
                        transformed[2][2].zero_status(),
                        transformed[2][3].zero_status(),
                        transformed[3][0].zero_status(),
                        transformed[3][1].zero_status(),
                        transformed[3][2].zero_status(),
                        transformed[3][3].zero_status(),
                    ))
                })
            },
        );
        group.bench_function(
            "hyperreal-symbolic/mat4 transform vec4 batch one-coord approx",
            |b| {
                b.iter(|| {
                    let transformed = black_box(
                        black_box(symbolic_mat4.clone())
                            .transform_vec4_batch(black_box(&symbolic_vec_batch)),
                    );
                    black_box(transformed[0][0].to_f64_lossy())
                })
            },
        );
        group.bench_function(
            "hyperreal-symbolic/mat4 transform vec4 batch all-coord approx",
            |b| {
                b.iter(|| {
                    let transformed = black_box(
                        black_box(symbolic_mat4.clone())
                            .transform_vec4_batch(black_box(&symbolic_vec_batch)),
                    );
                    black_box((
                        transformed[0][0].to_f64_lossy(),
                        transformed[0][1].to_f64_lossy(),
                        transformed[0][2].to_f64_lossy(),
                        transformed[0][3].to_f64_lossy(),
                        transformed[1][0].to_f64_lossy(),
                        transformed[1][1].to_f64_lossy(),
                        transformed[1][2].to_f64_lossy(),
                        transformed[1][3].to_f64_lossy(),
                        transformed[2][0].to_f64_lossy(),
                        transformed[2][1].to_f64_lossy(),
                        transformed[2][2].to_f64_lossy(),
                        transformed[2][3].to_f64_lossy(),
                        transformed[3][0].to_f64_lossy(),
                        transformed[3][1].to_f64_lossy(),
                        transformed[3][2].to_f64_lossy(),
                        transformed[3][3].to_f64_lossy(),
                    ))
                })
            },
        );
        group.bench_function(
            "hyperreal-symbolic/mat4 transform vec4 batch direction",
            |b| {
                b.iter(|| {
                    black_box(
                        black_box(symbolic_mat4.clone())
                            .transform_vec4_batch(black_box(&symbolic_direction_batch)),
                    )
                })
            },
        );
        group.bench_function(
            "hyperreal-symbolic/mat4 transform vec4 batch direction one-coord approx",
            |b| {
                b.iter(|| {
                    let transformed = black_box(
                        black_box(symbolic_mat4.clone())
                            .transform_vec4_batch(black_box(&symbolic_direction_batch)),
                    );
                    black_box(transformed[0][0].to_f64_lossy())
                })
            },
        );
        group.bench_function(
            "hyperreal-symbolic/mat4 transform vec4 batch direction all-coord approx",
            |b| {
                b.iter(|| {
                    let transformed = black_box(
                        black_box(symbolic_mat4.clone())
                            .transform_vec4_batch(black_box(&symbolic_direction_batch)),
                    );
                    black_box((
                        transformed[0][0].to_f64_lossy(),
                        transformed[0][1].to_f64_lossy(),
                        transformed[0][2].to_f64_lossy(),
                        transformed[0][3].to_f64_lossy(),
                        transformed[1][0].to_f64_lossy(),
                        transformed[1][1].to_f64_lossy(),
                        transformed[1][2].to_f64_lossy(),
                        transformed[1][3].to_f64_lossy(),
                        transformed[2][0].to_f64_lossy(),
                        transformed[2][1].to_f64_lossy(),
                        transformed[2][2].to_f64_lossy(),
                        transformed[2][3].to_f64_lossy(),
                        transformed[3][0].to_f64_lossy(),
                        transformed[3][1].to_f64_lossy(),
                        transformed[3][2].to_f64_lossy(),
                        transformed[3][3].to_f64_lossy(),
                    ))
                })
            },
        );
        // Construct all four outputs, then query sixteen signs and sixteen zero statuses.
        group.bench_function(
            "hyperreal-symbolic/mat4 transform vec4 batch direction structural facts",
            |b| {
                b.iter(|| {
                    let transformed = black_box(
                        black_box(symbolic_mat4.clone())
                            .transform_vec4_batch(black_box(&symbolic_direction_batch)),
                    );
                    black_box((
                        transformed[0][0].structural_facts().sign,
                        transformed[0][1].structural_facts().sign,
                        transformed[0][2].structural_facts().sign,
                        transformed[0][3].structural_facts().sign,
                        transformed[1][0].structural_facts().sign,
                        transformed[1][1].structural_facts().sign,
                        transformed[1][2].structural_facts().sign,
                        transformed[1][3].structural_facts().sign,
                        transformed[2][0].structural_facts().sign,
                        transformed[2][1].structural_facts().sign,
                        transformed[2][2].structural_facts().sign,
                        transformed[2][3].structural_facts().sign,
                        transformed[3][0].structural_facts().sign,
                        transformed[3][1].structural_facts().sign,
                        transformed[3][2].structural_facts().sign,
                        transformed[3][3].structural_facts().sign,
                        transformed[0][0].zero_status(),
                        transformed[0][1].zero_status(),
                        transformed[0][2].zero_status(),
                        transformed[0][3].zero_status(),
                        transformed[1][0].zero_status(),
                        transformed[1][1].zero_status(),
                        transformed[1][2].zero_status(),
                        transformed[1][3].zero_status(),
                        transformed[2][0].zero_status(),
                        transformed[2][1].zero_status(),
                        transformed[2][2].zero_status(),
                        transformed[2][3].zero_status(),
                        transformed[3][0].zero_status(),
                        transformed[3][1].zero_status(),
                        transformed[3][2].zero_status(),
                        transformed[3][3].zero_status(),
                    ))
                })
            },
        );
        group.bench_function(
            "hyperreal-symbolic/mat4 transform vec4 no-translation structural facts",
            |b| {
                b.iter(|| {
                    let transformed = black_box(
                        black_box(symbolic_linear_lhs.clone()) * black_box(symbolic_vector.clone()),
                    );
                    black_box((
                        transformed[0].structural_facts().sign,
                        transformed[1].structural_facts().sign,
                        transformed[2].structural_facts().sign,
                        transformed[3].structural_facts().sign,
                        transformed[0].zero_status(),
                        transformed[1].zero_status(),
                        transformed[2].zero_status(),
                        transformed[3].zero_status(),
                    ))
                })
            },
        );
        group.bench_function("hyperreal-symbolic/mat4 transform direction vec4", |b| {
            b.iter(|| {
                black_box(black_box(symbolic_mat4.clone()) * black_box(symbolic_direction.clone()))
            })
        });
        group.bench_function(
            "hyperreal-symbolic/mat4 transform direction vec4 one-coord approx",
            |b| {
                b.iter(|| {
                    let transformed = black_box(
                        black_box(symbolic_mat4.clone()) * black_box(symbolic_direction.clone()),
                    );
                    black_box(transformed[0].to_f64_lossy())
                })
            },
        );
        group.bench_function(
            "hyperreal-symbolic/mat4 transform direction vec4 all-coord approx",
            |b| {
                b.iter(|| {
                    let transformed = black_box(
                        black_box(symbolic_mat4.clone()) * black_box(symbolic_direction.clone()),
                    );
                    black_box((
                        transformed[0].to_f64_lossy(),
                        transformed[1].to_f64_lossy(),
                        transformed[2].to_f64_lossy(),
                        transformed[3].to_f64_lossy(),
                    ))
                })
            },
        );
        // Measure symbolic direction construction plus structural-fact queries.
        group.bench_function(
            "hyperreal-symbolic/mat4 transform direction vec4 structural facts",
            |b| {
                b.iter(|| {
                    let transformed = black_box(
                        black_box(symbolic_mat4.clone()) * black_box(symbolic_direction.clone()),
                    );
                    black_box((
                        transformed[0].structural_facts().sign,
                        transformed[1].structural_facts().sign,
                        transformed[2].structural_facts().sign,
                        transformed[3].structural_facts().sign,
                        transformed[0].zero_status(),
                        transformed[1].zero_status(),
                        transformed[2].zero_status(),
                        transformed[3].zero_status(),
                    ))
                })
            },
        );
        // Directly benchmark point/direction transform semantics with one shared matrix.
        group.bench_function("hyperreal-rational/mat4 transform point vec4", |b| {
            b.iter(|| {
                black_box(
                    black_box(rational_lhs.clone())
                        .transform_vec4_point(black_box(&symbolic_point)),
                )
            })
        });
        group.bench_function("hyperreal-rational/mat4 transform direction vec4", |b| {
            b.iter(|| {
                black_box(black_box(rational_lhs.clone()) * black_box(symbolic_direction.clone()))
            })
        });
        // Measure rational-matrix direction construction and facts without float conversion.
        group.bench_function(
            "hyperreal-rational/mat4 transform direction vec4 structural facts",
            |b| {
                b.iter(|| {
                    let transformed = black_box(
                        black_box(rational_lhs.clone()) * black_box(symbolic_direction.clone()),
                    );
                    black_box((
                        transformed[0].structural_facts().sign,
                        transformed[1].structural_facts().sign,
                        transformed[2].structural_facts().sign,
                        transformed[3].structural_facts().sign,
                        transformed[0].zero_status(),
                        transformed[1].zero_status(),
                        transformed[2].zero_status(),
                        transformed[3].zero_status(),
                    ))
                })
            },
        );
    }
    let numerica_ctx = numerica_engine::Ctx::new(128);
    let numerica_lhs_cases =
        lhs_cases.map(|value| numerica_engine::Mat4::new(&numerica_ctx, value.m));
    let numerica_rhs_cases =
        rhs_cases.map(|value| numerica_engine::Mat4::new(&numerica_ctx, value.m));
    let numerica_vector_cases = vector_cases
        .map(|value| numerica_engine::Vec4::new(&numerica_ctx, value.x, value.y, value.z, value.w));
    group.bench_function("numerica128/mat4 determinant", |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(next_case(&numerica_lhs_cases, &cursor).clone()).determinant(&numerica_ctx)
        })
    });
    group.bench_function("numerica128/mat4 inverse", |b| {
        let cursor = Cell::new(0);
        b.iter(|| black_box(next_case(&numerica_lhs_cases, &cursor).clone()).inverse(&numerica_ctx))
    });
    group.bench_function("numerica128/mat4 mul mat4", |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % numerica_lhs_cases.len());
            black_box(numerica_lhs_cases[index].clone())
                .mul_mat4(black_box(&numerica_rhs_cases[index]), &numerica_ctx)
        })
    });
    group.bench_function("numerica128/mat4 transform vec4", |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % numerica_lhs_cases.len());
            black_box(numerica_lhs_cases[index].clone())
                .transform_vec4(black_box(&numerica_vector_cases[index]), &numerica_ctx)
        })
    });

    let gmp_ctx = gmp_engine::Ctx::new(128);
    let gmp_lhs_cases = lhs_cases.map(|value| gmp_engine::Mat4::new(&gmp_ctx, value.m));
    let gmp_rhs_cases = rhs_cases.map(|value| gmp_engine::Mat4::new(&gmp_ctx, value.m));
    let gmp_vector_cases = vector_cases
        .map(|value| gmp_engine::Vec4::new(&gmp_ctx, value.x, value.y, value.z, value.w));
    group.bench_function("gmp_mpfr128/mat4 determinant", |b| {
        let cursor = Cell::new(0);
        b.iter(|| black_box(next_case(&gmp_lhs_cases, &cursor).clone()).determinant(&gmp_ctx))
    });
    group.bench_function("gmp_mpfr128/mat4 inverse", |b| {
        let cursor = Cell::new(0);
        b.iter(|| black_box(next_case(&gmp_lhs_cases, &cursor).clone()).inverse(&gmp_ctx))
    });
    group.bench_function("gmp_mpfr128/mat4 mul mat4", |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % gmp_lhs_cases.len());
            black_box(gmp_lhs_cases[index].clone())
                .mul_mat4(black_box(&gmp_rhs_cases[index]), &gmp_ctx)
        })
    });
    group.bench_function("gmp_mpfr128/mat4 transform vec4", |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % gmp_lhs_cases.len());
            black_box(gmp_lhs_cases[index].clone())
                .transform_vec4(black_box(&gmp_vector_cases[index]), &gmp_ctx)
        })
    });

    let symbolica_ctx = symbolica_engine::Ctx::new(128);
    let symbolica_lhs_cases =
        lhs_cases.map(|value| symbolica_engine::Mat4::new(&symbolica_ctx, value.m));
    let symbolica_rhs_cases =
        rhs_cases.map(|value| symbolica_engine::Mat4::new(&symbolica_ctx, value.m));
    let symbolica_vector_cases = vector_cases.map(|value| {
        symbolica_engine::Vec4::new(&symbolica_ctx, value.x, value.y, value.z, value.w)
    });
    group.bench_function("symbolica/mat4 determinant", |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(next_case(&symbolica_lhs_cases, &cursor).clone()).determinant(&symbolica_ctx)
        })
    });
    group.bench_function("symbolica/mat4 inverse", |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            black_box(next_case(&symbolica_lhs_cases, &cursor).clone()).inverse(&symbolica_ctx)
        })
    });
    group.bench_function("symbolica/mat4 mul mat4", |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % symbolica_lhs_cases.len());
            black_box(symbolica_lhs_cases[index].clone())
                .mul_mat4(black_box(&symbolica_rhs_cases[index]), &symbolica_ctx)
        })
    });
    group.bench_function("symbolica/mat4 transform vec4", |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % symbolica_lhs_cases.len());
            black_box(symbolica_lhs_cases[index].clone())
                .transform_vec4(black_box(&symbolica_vector_cases[index]), &symbolica_ctx)
        })
    });

    group.finish();
}

/// Named scalar fixture pairing a precomputed float with an exact-builder selector.
///
/// The native binary/external paths import value; exact authored builders dispatch
/// solely on name and ignore value. No constructor checks that the two agree or
/// that a name is recognized. Copying a case copies only the static label and f64.
#[derive(Clone, Copy)]
struct TrigCase {
    /// Stable row/report label and exact-builder match key, not a parsed expression.
    name: &'static str,
    /// Already-rounded binary-float input for binary-native and external adapters.
    value: f64,
}

/// Returns six fixed sin/cos cases, including large range-reduction arguments.
///
/// Values are 0.1, 1.23456789, 1e6, 1e30, floating PI/7 and floating 1000*PI+1e-20.
/// Computation happens in f64: 1e30 is not the exact decimal integer, PI is not
/// symbolic pi, and the last expression loses its tiny addend. trig_rational uses
/// the names to author different exact inputs. Returned order controls registration
/// and report order; these fixtures alone do not qualify range-reduction accuracy.
fn trig_cases() -> [TrigCase; 6] {
    [
        TrigCase {
            name: "0.1",
            value: 0.1,
        },
        TrigCase {
            name: "1.23456789",
            value: 1.23456789,
        },
        TrigCase {
            name: "1e6",
            value: 1.0e6,
        },
        TrigCase {
            name: "1e30",
            value: 1.0e30,
        },
        TrigCase {
            name: "pi_7",
            value: std::f64::consts::PI / 7.0,
        },
        TrigCase {
            name: "1000pi_eps",
            value: 1000.0 * std::f64::consts::PI + 1.0e-20,
        },
    ]
}

/// Returns half, negative/positive 0.999999 and 1e-12 for asin/acos/atanh rows.
///
/// All four floats lie strictly between -1 and 1; endpoints are not exercised.
/// Names select exact decimal fractions in inverse_rational, which need not equal
/// the stored binary values. This is a fixed interior-domain corpus, not a guard.
fn inverse_unit_cases() -> [TrigCase; 4] {
    [
        TrigCase {
            name: "0.5",
            value: 0.5,
        },
        TrigCase {
            name: "neg_0.999999",
            value: -0.999_999,
        },
        TrigCase {
            name: "0.999999",
            value: 0.999_999,
        },
        TrigCase {
            name: "1e-12",
            value: 1.0e-12,
        },
    ]
}

/// Returns half, negative 1e-12 and positive/negative 1e6 for atan/asinh rows.
///
/// These finite inputs span signs and scales. inverse_rational authors values by
/// name; the tiny decimal differs from its binary-float value. No infinity or
/// invalid input is included, and order is reused by the report builder.
fn inverse_real_cases() -> [TrigCase; 4] {
    [
        TrigCase {
            name: "0.5",
            value: 0.5,
        },
        TrigCase {
            name: "neg_1e-12",
            value: -1.0e-12,
        },
        TrigCase {
            name: "1e6",
            value: 1.0e6,
        },
        TrigCase {
            name: "neg_1e6",
            value: -1.0e6,
        },
    ]
}

/// Returns four floats above one for acosh: 9, 1+1e-12, 1e6 and imported E.
///
/// The near-one expression is rounded before import. inverse_rational instead
/// constructs exactly 1+1/10^12 and symbolic e for those names. The domain boundary
/// at one and below-domain failures are not exercised by this list.
fn inverse_acosh_cases() -> [TrigCase; 4] {
    [
        TrigCase {
            name: "9",
            value: 9.0,
        },
        TrigCase {
            name: "1_plus_1e-12",
            value: 1.0 + 1.0e-12,
        },
        TrigCase {
            name: "1e6",
            value: 1.0e6,
        },
        TrigCase {
            name: "e",
            value: std::f64::consts::E,
        },
    ]
}

/// Parses the exact rational 1/10^20 used in the symbolic range-reduction fixture.
///
/// The decimal literal avoids a float intermediate and the u64 denominator limit
/// of q. Parsing is repeated on each call; unwrap relies on this fixed valid text.
/// Adding it to symbolic 1000*pi retains a term that the f64 case expression loses.
fn one_e_minus_20() -> Real {
    "0.00000000000000000001".parse::<Rational>().unwrap().into()
}

/// Authors the six trig inputs from their names, including two nonrational inputs.
///
/// Decimal names map to exact ratios/integers, including integer 10^30. pi_7 maps
/// to symbolic pi/7, and 1000pi_eps to symbolic 1000*pi plus exact 1/10^20. Thus the
/// historical rational label does not mean every result is rational or equal to
/// the binary case value. The payload is ignored, even if nonfinite or inconsistent.
/// Unknown names panic through unreachable; this is a closed fixture selector, not
/// a general parser or validation/conversion API. Constant division is unwrapped.
fn trig_rational(case: TrigCase) -> Real {
    match case.name {
        "0.1" => q(1, 10),
        "1.23456789" => q(123_456_789, 100_000_000),
        "1e6" => 1_000_000.into(),
        "1e30" => 1_000_000_000_000_000_000_000_000_000_000_i128.into(),
        "pi_7" => (Real::pi() / Real::from(7)).unwrap(),
        "1000pi_eps" => Real::pi() * Real::from(1000) + one_e_minus_20(),
        _ => unreachable!("all trig cases are covered"),
    }
}

/// Authors exact inverse-function inputs from ten recognized names, ignoring value.
///
/// Uses fractions/integers for the decimal cases, exactly 1+1/10^12 for the near-one
/// case and symbolic e for the e case. The historical name does not restrict output
/// to rationals. It does not validate a float payload or determine which inverse
/// operation is applicable; each caller's case list supplies that domain contract.
/// A name outside the union of the three inverse lists panics through unreachable.
fn inverse_rational(case: TrigCase) -> Real {
    match case.name {
        "0.5" => q(1, 2),
        "neg_0.999999" => q(-999_999, 1_000_000),
        "0.999999" => q(999_999, 1_000_000),
        "1e-12" => q(1, 1_000_000_000_000),
        "neg_1e-12" => q(-1, 1_000_000_000_000),
        "1e6" => 1_000_000.into(),
        "neg_1e6" => (-1_000_000).into(),
        "9" => 9.into(),
        "1_plus_1e-12" => q(1_000_000_000_001, 1_000_000_000_000),
        "e" => Real::e(),
        _ => unreachable!("all inverse cases are covered"),
    }
}

/// Registers thirty-six native binary-input rows and matching diagnostic hooks.
///
/// Six sin/cos cases give twelve rows, four asin/acos/atanh cases twelve, four
/// atan/asinh cases eight, and four acosh cases four. Each case is imported through
/// s outside timing and then retained for its own row; there is no rotating cursor.
/// Ordinary iter measures cloning, scalar operation and output destruction without
/// requesting a primitive-float approximation or a common accuracy target.
///
/// Each enabled/selected hook invokes its operation once before registering that
/// timing row, allowing retained source/evidence reuse. Inverse-function Results
/// are unwrapped. No numerical, range-reduction, dispatch-count or certainty-receipt
/// assertion is made, and there is no cancellation input or total-work/memory limit.
fn bench_blas_scalar_trig(
    group: &mut BenchmarkGroup<'_, criterion::measurement::WallTime>,
    label: &str,
) {
    for case in trig_cases() {
        let blas_value = s(case.value);
        trace_dispatch_row(format!("scalar_trig/{label}/{}/sin", case.name), || {
            let _ = black_box(Real::sin(black_box(blas_value.clone())));
        });
        group.bench_function(format!("{label}/{}/sin", case.name), |b| {
            b.iter(|| black_box(Real::sin(black_box(blas_value.clone()))))
        });
        trace_dispatch_row(format!("scalar_trig/{label}/{}/cos", case.name), || {
            let _ = black_box(Real::cos(black_box(blas_value.clone())));
        });
        group.bench_function(format!("{label}/{}/cos", case.name), |b| {
            b.iter(|| black_box(Real::cos(black_box(blas_value.clone()))))
        });
    }
    for case in inverse_unit_cases() {
        let blas_value = s(case.value);
        trace_dispatch_row(format!("scalar_trig/{label}/{}/asin", case.name), || {
            let _ = black_box(Real::asin(black_box(blas_value.clone())).unwrap());
        });
        group.bench_function(format!("{label}/{}/asin", case.name), |b| {
            b.iter(|| black_box(Real::asin(black_box(blas_value.clone())).unwrap()))
        });
        trace_dispatch_row(format!("scalar_trig/{label}/{}/acos", case.name), || {
            let _ = black_box(Real::acos(black_box(blas_value.clone())).unwrap());
        });
        group.bench_function(format!("{label}/{}/acos", case.name), |b| {
            b.iter(|| black_box(Real::acos(black_box(blas_value.clone())).unwrap()))
        });
        trace_dispatch_row(format!("scalar_trig/{label}/{}/atanh", case.name), || {
            let _ = black_box(Real::atanh(black_box(blas_value.clone())).unwrap());
        });
        group.bench_function(format!("{label}/{}/atanh", case.name), |b| {
            b.iter(|| black_box(Real::atanh(black_box(blas_value.clone())).unwrap()))
        });
    }
    for case in inverse_real_cases() {
        let blas_value = s(case.value);
        trace_dispatch_row(format!("scalar_trig/{label}/{}/atan", case.name), || {
            let _ = black_box(Real::atan(black_box(blas_value.clone())).unwrap());
        });
        group.bench_function(format!("{label}/{}/atan", case.name), |b| {
            b.iter(|| black_box(Real::atan(black_box(blas_value.clone())).unwrap()))
        });
        trace_dispatch_row(format!("scalar_trig/{label}/{}/asinh", case.name), || {
            let _ = black_box(Real::asinh(black_box(blas_value.clone())).unwrap());
        });
        group.bench_function(format!("{label}/{}/asinh", case.name), |b| {
            b.iter(|| black_box(Real::asinh(black_box(blas_value.clone())).unwrap()))
        });
    }
    for case in inverse_acosh_cases() {
        let blas_value = s(case.value);
        trace_dispatch_row(format!("scalar_trig/{label}/{}/acosh", case.name), || {
            let _ = black_box(Real::acosh(black_box(blas_value.clone())).unwrap());
        });
        group.bench_function(format!("{label}/{}/acosh", case.name), |b| {
            b.iter(|| black_box(Real::acosh(black_box(blas_value.clone())).unwrap()))
        });
    }
}

/// Registers 180 fixed-case trig/inverse rows and seventy-two native trace hooks.
///
/// Each of two native models and three external engines contributes thirty-six
/// rows under inherited Criterion settings. The binary-native path and external
/// adapters import each case's precomputed float. The historical hyperreal-rational
/// path instead uses name-based exact builders: decimal fractions, integer 10^30,
/// symbolic pi/7, symbolic 1000*pi+1/10^20, and symbolic e where selected. Matching
/// labels therefore compare different intended inputs in several cases.
///
/// Fixture construction and external contexts precede timing. Ordinary iter clones
/// native scalars but borrows external adapter inputs, including any adapter-internal
/// work; result destruction is measured. No output float conversion is requested.
/// Numerica labels use Symbolica Float and GMP uses Rug/MPFR at requested precision
/// 128; the Atom adapter stores 128 without using it as a precision setting here.
/// Its inverse formulas use log/radical expressions, including complex intermediates;
/// native/adapter branch agreement, errors and accuracy are not checked. Numerica's
/// atan uses atan2(x,1), while GMP uses its atan operation.
///
/// Thirty-six hooks per native model construct one operation per case/row before
/// timing. External rows have no hooks. Native inverse Results are unwrapped; there
/// are no value/path-count assertions, cancellation signals, policy receipts or
/// total-work bounds. Scalar-object construction and floating computation do not
/// constitute a common output-materialization contract merely by sharing row names.
fn bench_scalar_trig(c: &mut Criterion) {
    let mut group = c.benchmark_group("scalar_trig");

    bench_blas_scalar_trig(&mut group, "hyperreal");

    for case in trig_cases() {
        let rational_value = trig_rational(case);
        trace_dispatch_row(
            format!("scalar_trig/hyperreal-rational/{}/sin", case.name),
            || {
                let _ = black_box(Real::sin(black_box(rational_value.clone())));
            },
        );
        group.bench_function(format!("hyperreal-rational/{}/sin", case.name), |b| {
            b.iter(|| black_box(Real::sin(black_box(rational_value.clone()))))
        });
        trace_dispatch_row(
            format!("scalar_trig/hyperreal-rational/{}/cos", case.name),
            || {
                let _ = black_box(Real::cos(black_box(rational_value.clone())));
            },
        );
        group.bench_function(format!("hyperreal-rational/{}/cos", case.name), |b| {
            b.iter(|| black_box(Real::cos(black_box(rational_value.clone()))))
        });
    }
    for case in inverse_unit_cases() {
        let rational_value = inverse_rational(case);
        trace_dispatch_row(
            format!("scalar_trig/hyperreal-rational/{}/asin", case.name),
            || {
                let _ = black_box(Real::asin(black_box(rational_value.clone())).unwrap());
            },
        );
        group.bench_function(format!("hyperreal-rational/{}/asin", case.name), |b| {
            b.iter(|| black_box(Real::asin(black_box(rational_value.clone())).unwrap()))
        });
        trace_dispatch_row(
            format!("scalar_trig/hyperreal-rational/{}/acos", case.name),
            || {
                let _ = black_box(Real::acos(black_box(rational_value.clone())).unwrap());
            },
        );
        group.bench_function(format!("hyperreal-rational/{}/acos", case.name), |b| {
            b.iter(|| black_box(Real::acos(black_box(rational_value.clone())).unwrap()))
        });
        trace_dispatch_row(
            format!("scalar_trig/hyperreal-rational/{}/atanh", case.name),
            || {
                let _ = black_box(Real::atanh(black_box(rational_value.clone())).unwrap());
            },
        );
        group.bench_function(format!("hyperreal-rational/{}/atanh", case.name), |b| {
            b.iter(|| black_box(Real::atanh(black_box(rational_value.clone())).unwrap()))
        });
    }
    for case in inverse_real_cases() {
        let rational_value = inverse_rational(case);
        trace_dispatch_row(
            format!("scalar_trig/hyperreal-rational/{}/atan", case.name),
            || {
                let _ = black_box(Real::atan(black_box(rational_value.clone())).unwrap());
            },
        );
        group.bench_function(format!("hyperreal-rational/{}/atan", case.name), |b| {
            b.iter(|| black_box(Real::atan(black_box(rational_value.clone())).unwrap()))
        });
        trace_dispatch_row(
            format!("scalar_trig/hyperreal-rational/{}/asinh", case.name),
            || {
                let _ = black_box(Real::asinh(black_box(rational_value.clone())).unwrap());
            },
        );
        group.bench_function(format!("hyperreal-rational/{}/asinh", case.name), |b| {
            b.iter(|| black_box(Real::asinh(black_box(rational_value.clone())).unwrap()))
        });
    }
    for case in inverse_acosh_cases() {
        let rational_value = inverse_rational(case);
        trace_dispatch_row(
            format!("scalar_trig/hyperreal-rational/{}/acosh", case.name),
            || {
                let _ = black_box(Real::acosh(black_box(rational_value.clone())).unwrap());
            },
        );
        group.bench_function(format!("hyperreal-rational/{}/acosh", case.name), |b| {
            b.iter(|| black_box(Real::acosh(black_box(rational_value.clone())).unwrap()))
        });
    }
    let numerica_ctx = numerica_engine::Ctx::new(128);
    for case in trig_cases() {
        let numerica_value = numerica_ctx.f(case.value);
        group.bench_function(format!("numerica128/{}/sin", case.name), |b| {
            b.iter(|| numerica_ctx.sin(black_box(&numerica_value)))
        });
        group.bench_function(format!("numerica128/{}/cos", case.name), |b| {
            b.iter(|| numerica_ctx.cos(black_box(&numerica_value)))
        });
    }
    for case in inverse_unit_cases() {
        let numerica_value = numerica_ctx.f(case.value);
        group.bench_function(format!("numerica128/{}/asin", case.name), |b| {
            b.iter(|| numerica_ctx.asin(black_box(&numerica_value)))
        });
        group.bench_function(format!("numerica128/{}/acos", case.name), |b| {
            b.iter(|| numerica_ctx.acos(black_box(&numerica_value)))
        });
        group.bench_function(format!("numerica128/{}/atanh", case.name), |b| {
            b.iter(|| numerica_ctx.atanh(black_box(&numerica_value)))
        });
    }
    for case in inverse_real_cases() {
        let numerica_value = numerica_ctx.f(case.value);
        group.bench_function(format!("numerica128/{}/atan", case.name), |b| {
            b.iter(|| numerica_ctx.atan(black_box(&numerica_value)))
        });
        group.bench_function(format!("numerica128/{}/asinh", case.name), |b| {
            b.iter(|| numerica_ctx.asinh(black_box(&numerica_value)))
        });
    }
    for case in inverse_acosh_cases() {
        let numerica_value = numerica_ctx.f(case.value);
        group.bench_function(format!("numerica128/{}/acosh", case.name), |b| {
            b.iter(|| numerica_ctx.acosh(black_box(&numerica_value)))
        });
    }

    let gmp_ctx = gmp_engine::Ctx::new(128);
    for case in trig_cases() {
        let gmp_value = gmp_ctx.f(case.value);
        group.bench_function(format!("gmp_mpfr128/{}/sin", case.name), |b| {
            b.iter(|| gmp_ctx.sin(black_box(&gmp_value)))
        });
        group.bench_function(format!("gmp_mpfr128/{}/cos", case.name), |b| {
            b.iter(|| gmp_ctx.cos(black_box(&gmp_value)))
        });
    }
    for case in inverse_unit_cases() {
        let gmp_value = gmp_ctx.f(case.value);
        group.bench_function(format!("gmp_mpfr128/{}/asin", case.name), |b| {
            b.iter(|| gmp_ctx.asin(black_box(&gmp_value)))
        });
        group.bench_function(format!("gmp_mpfr128/{}/acos", case.name), |b| {
            b.iter(|| gmp_ctx.acos(black_box(&gmp_value)))
        });
        group.bench_function(format!("gmp_mpfr128/{}/atanh", case.name), |b| {
            b.iter(|| gmp_ctx.atanh(black_box(&gmp_value)))
        });
    }
    for case in inverse_real_cases() {
        let gmp_value = gmp_ctx.f(case.value);
        group.bench_function(format!("gmp_mpfr128/{}/atan", case.name), |b| {
            b.iter(|| gmp_ctx.atan(black_box(&gmp_value)))
        });
        group.bench_function(format!("gmp_mpfr128/{}/asinh", case.name), |b| {
            b.iter(|| gmp_ctx.asinh(black_box(&gmp_value)))
        });
    }
    for case in inverse_acosh_cases() {
        let gmp_value = gmp_ctx.f(case.value);
        group.bench_function(format!("gmp_mpfr128/{}/acosh", case.name), |b| {
            b.iter(|| gmp_ctx.acosh(black_box(&gmp_value)))
        });
    }

    let symbolica_ctx = symbolica_engine::Ctx::new(128);
    for case in trig_cases() {
        let symbolica_value = symbolica_ctx.f(case.value);
        group.bench_function(format!("symbolica/{}/sin", case.name), |b| {
            b.iter(|| symbolica_ctx.sin(black_box(&symbolica_value)))
        });
        group.bench_function(format!("symbolica/{}/cos", case.name), |b| {
            b.iter(|| symbolica_ctx.cos(black_box(&symbolica_value)))
        });
    }
    for case in inverse_unit_cases() {
        let symbolica_value = symbolica_ctx.f(case.value);
        group.bench_function(format!("symbolica/{}/asin", case.name), |b| {
            b.iter(|| symbolica_ctx.asin(black_box(&symbolica_value)))
        });
        group.bench_function(format!("symbolica/{}/acos", case.name), |b| {
            b.iter(|| symbolica_ctx.acos(black_box(&symbolica_value)))
        });
        group.bench_function(format!("symbolica/{}/atanh", case.name), |b| {
            b.iter(|| symbolica_ctx.atanh(black_box(&symbolica_value)))
        });
    }
    for case in inverse_real_cases() {
        let symbolica_value = symbolica_ctx.f(case.value);
        group.bench_function(format!("symbolica/{}/atan", case.name), |b| {
            b.iter(|| symbolica_ctx.atan(black_box(&symbolica_value)))
        });
        group.bench_function(format!("symbolica/{}/asinh", case.name), |b| {
            b.iter(|| symbolica_ctx.asinh(black_box(&symbolica_value)))
        });
    }
    for case in inverse_acosh_cases() {
        let symbolica_value = symbolica_ctx.f(case.value);
        group.bench_function(format!("symbolica/{}/acosh", case.name), |b| {
            b.iter(|| symbolica_ctx.acosh(black_box(&symbolica_value)))
        });
    }

    group.finish();
}

/// Allocates a fresh, initially clear shared atomic cancellation flag for benchmarks.
///
/// Clones share the same flag; separate calls allocate independent flags. This
/// helper neither attaches the signal to an operation nor sets it, starts a worker,
/// or installs a deadline/resource limit. Callers choose whether and when to set it;
/// creating or attaching a clear signal alone does not exercise cancellation.
fn abort_signal() -> hyperlattice::AbortSignal {
    std::sync::Arc::new(std::sync::atomic::AtomicBool::new(false))
}
