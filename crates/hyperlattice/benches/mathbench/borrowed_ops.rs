fn bench_borrowed_operations_for<B, F>(
    group: &mut BenchmarkGroup<'_, criterion::measurement::WallTime>,
    label: &str,
    make_scalar: F,
) where
    B: Backend,
    F: Copy + Fn(f64) -> Scalar<B>,
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
        group.bench_function(format!("{label}/scalar {name} owned_ref"), |b| {
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
        group.bench_function(format!("{label}/scalar {name} ref_owned"), |b| {
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
    }

    macro_rules! bench_vector_borrowed {
        ($dimension:literal, $lhs:ident, $rhs:ident) => {
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
                        (black_box($lhs[index].clone()) / black_box(&scalar_cases[index]))
                            .unwrap(),
                    )
                })
            });
        };
    }

    macro_rules! bench_matrix_borrowed {
        ($dimension:literal, $lhs:ident, $rhs:ident) => {
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
                        (black_box($lhs[index].clone()) / black_box(&scalar_cases[index]))
                            .unwrap(),
                    )
                })
            });
        };
    }

    bench_vector_borrowed!("vec3", vec3_lhs, vec3_rhs);
    bench_vector_borrowed!("vec4", vec4_lhs, vec4_rhs);
    bench_matrix_borrowed!("mat3", mat3_lhs, mat3_rhs);
    bench_matrix_borrowed!("mat4", mat4_lhs, mat4_rhs);

    group.bench_function(format!("{label}/mat3 transform_vec refs"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % mat3_lhs.len());
            black_box(black_box(&mat3_lhs[index]) * black_box(&vec3_lhs[index]))
        })
    });
    group.bench_function(format!("{label}/mat4 transform_vec refs"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % mat4_lhs.len());
            black_box(black_box(&mat4_lhs[index]) * black_box(&vec4_lhs[index]))
        })
    });

    for name in ["add", "sub", "mul", "div"] {
        group.bench_function(format!("{label}/complex {name} refs"), |b| {
            let cursor = Cell::new(0);
            b.iter(|| {
                let index = cursor.get();
                cursor.set((index + 1) % complex_lhs.len());
                match name {
                    "add" => black_box(black_box(&complex_lhs[index]) + black_box(&complex_rhs[index])),
                    "sub" => black_box(black_box(&complex_lhs[index]) - black_box(&complex_rhs[index])),
                    "mul" => black_box(black_box(&complex_lhs[index]) * black_box(&complex_rhs[index])),
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
            black_box((black_box(complex_lhs[index].clone()) / black_box(&scalar_cases[index])).unwrap())
        })
    });
}

fn bench_borrowed_operations(c: &mut Criterion) {
    let mut group = c.benchmark_group("borrowed_ops");
    bench_borrowed_operations_for::<ApproxBackend, _>(&mut group, "approx", s::<ApproxBackend>);
    bench_borrowed_operations_for::<HyperrealBackend, _>(
        &mut group,
        "hyperreal",
        s::<HyperrealBackend>,
    );
    bench_borrowed_operations_for::<HyperrealBackend, _>(&mut group, "hyperreal-rational", qr);
    bench_symbolica_borrowed_operations(&mut group, "symbolica");
    group.finish();
}

fn bench_symbolica_borrowed_operations(
    group: &mut BenchmarkGroup<'_, criterion::measurement::WallTime>,
    label: &str,
) {
    let ctx = symbolica_backend::Ctx::new(128);
    let scalar_pairs = [
        (ctx.f(2.5), ctx.f(1.25)),
        (ctx.f(1.0e-12), ctx.f(-1.0e-12)),
        (ctx.f(1.0e9), ctx.f(1.0e-9)),
        (ctx.f(-2.75), ctx.f(0.125)),
    ];
    let scalar_cases = [2.0, 1.0e-9, -1.0e9, std::f64::consts::PI].map(|value| ctx.f(value));
    let vec3_lhs =
        sample_vec3_cases().map(|value| symbolica_backend::Vec3::new(&ctx, value.x, value.y, value.z));
    let vec3_rhs = sample_vec3_b_cases()
        .map(|value| symbolica_backend::Vec3::new(&ctx, value.x, value.y, value.z));
    let vec4_lhs = sample_vec4_cases()
        .map(|value| symbolica_backend::Vec4::new(&ctx, value.x, value.y, value.z, value.w));
    let vec4_rhs = sample_vec4_b_cases()
        .map(|value| symbolica_backend::Vec4::new(&ctx, value.x, value.y, value.z, value.w));
    let mat3_lhs = sample_mat3_cases().map(|value| symbolica_backend::Mat3::new(&ctx, value.m));
    let mat3_rhs = sample_mat3_b_cases().map(|value| symbolica_backend::Mat3::new(&ctx, value.m));
    let mat4_lhs = sample_mat4_cases().map(|value| symbolica_backend::Mat4::new(&ctx, value.m));
    let mat4_rhs = sample_mat4_b_cases().map(|value| symbolica_backend::Mat4::new(&ctx, value.m));
    let complex_lhs = [
        symbolica_backend::Complex::new(&ctx, 3.0, 4.0),
        symbolica_backend::Complex::new(&ctx, 1.0e-9, -1.0e-9),
        symbolica_backend::Complex::new(&ctx, 1.0e9, -1.0),
        symbolica_backend::Complex::new(&ctx, std::f64::consts::PI, -std::f64::consts::E),
    ];
    let complex_rhs = [
        symbolica_backend::Complex::new(&ctx, 1.5, -2.0),
        symbolica_backend::Complex::new(&ctx, -1.0e-9, 2.0e-9),
        symbolica_backend::Complex::new(&ctx, -1.0e9, 2.0),
        symbolica_backend::Complex::new(&ctx, std::f64::consts::SQRT_2, std::f64::consts::FRAC_1_PI),
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
    }

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
            black_box(mat3_lhs[index].combine(&mat3_rhs[index], &ctx, symbolica_backend::Ctx::add))
        })
    });
    group.bench_function(format!("{label}/mat3 sub refs"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % mat3_lhs.len());
            black_box(mat3_lhs[index].combine(&mat3_rhs[index], &ctx, symbolica_backend::Ctx::sub))
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
            black_box(mat3_lhs[index].map_scalar(&scalar_cases[index], &ctx, symbolica_backend::Ctx::add))
        })
    });
    group.bench_function(format!("{label}/mat3 sub_scalar_ref"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % mat3_lhs.len());
            black_box(mat3_lhs[index].map_scalar(&scalar_cases[index], &ctx, symbolica_backend::Ctx::sub))
        })
    });
    group.bench_function(format!("{label}/mat3 mul_scalar_ref"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % mat3_lhs.len());
            black_box(mat3_lhs[index].map_scalar(&scalar_cases[index], &ctx, symbolica_backend::Ctx::mul))
        })
    });
    group.bench_function(format!("{label}/mat3 div_scalar_ref"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % mat3_lhs.len());
            black_box(mat3_lhs[index].map_scalar(&scalar_cases[index], &ctx, symbolica_backend::Ctx::div))
        })
    });

    group.bench_function(format!("{label}/mat4 add refs"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % mat4_lhs.len());
            black_box(mat4_lhs[index].combine(&mat4_rhs[index], &ctx, symbolica_backend::Ctx::add))
        })
    });
    group.bench_function(format!("{label}/mat4 sub refs"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % mat4_lhs.len());
            black_box(mat4_lhs[index].combine(&mat4_rhs[index], &ctx, symbolica_backend::Ctx::sub))
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
            black_box(mat4_lhs[index].map_scalar(&scalar_cases[index], &ctx, symbolica_backend::Ctx::add))
        })
    });
    group.bench_function(format!("{label}/mat4 sub_scalar_ref"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % mat4_lhs.len());
            black_box(mat4_lhs[index].map_scalar(&scalar_cases[index], &ctx, symbolica_backend::Ctx::sub))
        })
    });
    group.bench_function(format!("{label}/mat4 mul_scalar_ref"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % mat4_lhs.len());
            black_box(mat4_lhs[index].map_scalar(&scalar_cases[index], &ctx, symbolica_backend::Ctx::mul))
        })
    });
    group.bench_function(format!("{label}/mat4 div_scalar_ref"), |b| {
        let cursor = Cell::new(0);
        b.iter(|| {
            let index = cursor.get();
            cursor.set((index + 1) % mat4_lhs.len());
            black_box(mat4_lhs[index].map_scalar(&scalar_cases[index], &ctx, symbolica_backend::Ctx::div))
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
}
