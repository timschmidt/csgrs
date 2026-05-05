use criterion::{BatchSize, Criterion, black_box, criterion_group, criterion_main};
use hyperreal::{Computable, Rational, Real};
use num::bigint::{BigInt, BigUint};

#[path = "support/bench_docs.rs"]
mod bench_docs;

use bench_docs::{BenchDoc, BenchGroupDoc};

const SCALAR_MICRO_GROUPS: &[BenchGroupDoc] = &[
    BenchGroupDoc {
        name: "raw_cache_hit_cost",
        description: "Cost of cold and cached `Computable::approx` calls for simple values.",
        benches: &[
            BenchDoc {
                name: "zero",
                description: "Cached approximation request for exact zero.",
            },
            BenchDoc {
                name: "one",
                description: "Cached approximation request for exact one.",
            },
            BenchDoc {
                name: "two",
                description: "Cached approximation request for exact two.",
            },
            BenchDoc {
                name: "e",
                description: "Cached approximation request for Euler's constant.",
            },
            BenchDoc {
                name: "pi",
                description: "Cached approximation request for pi.",
            },
            BenchDoc {
                name: "tau",
                description: "Cached approximation request for two pi.",
            },
        ],
    },
    BenchGroupDoc {
        name: "structural_query_speed",
        description: "Speed of public structural queries across exact, transcendental, and composite `Real` values.",
        benches: &[
            BenchDoc {
                name: "zero_zero_status",
                description: "Checks zero/nonzero facts for exact zero.",
            },
            BenchDoc {
                name: "zero_sign_query",
                description: "Reads sign facts for exact zero.",
            },
            BenchDoc {
                name: "zero_msd_query",
                description: "Reads magnitude facts for exact zero.",
            },
            BenchDoc {
                name: "zero_structural_facts",
                description: "Computes full structural facts for exact zero.",
            },
            BenchDoc {
                name: "one_zero_status",
                description: "Checks zero/nonzero facts for exact one.",
            },
            BenchDoc {
                name: "one_sign_query",
                description: "Reads sign facts for exact one.",
            },
            BenchDoc {
                name: "one_msd_query",
                description: "Reads magnitude facts for exact one.",
            },
            BenchDoc {
                name: "one_structural_facts",
                description: "Computes full structural facts for exact one.",
            },
            BenchDoc {
                name: "negative_zero_status",
                description: "Checks zero/nonzero facts for an exact negative integer.",
            },
            BenchDoc {
                name: "negative_sign_query",
                description: "Reads sign facts for an exact negative integer.",
            },
            BenchDoc {
                name: "negative_msd_query",
                description: "Reads magnitude facts for an exact negative integer.",
            },
            BenchDoc {
                name: "negative_structural_facts",
                description: "Computes full structural facts for an exact negative integer.",
            },
            BenchDoc {
                name: "tiny_exact_zero_status",
                description: "Checks zero/nonzero facts for a tiny exact rational.",
            },
            BenchDoc {
                name: "tiny_exact_sign_query",
                description: "Reads sign facts for a tiny exact rational.",
            },
            BenchDoc {
                name: "tiny_exact_msd_query",
                description: "Reads magnitude facts for a tiny exact rational.",
            },
            BenchDoc {
                name: "tiny_exact_structural_facts",
                description: "Computes full structural facts for a tiny exact rational.",
            },
            BenchDoc {
                name: "pi_zero_status",
                description: "Checks zero/nonzero facts for pi.",
            },
            BenchDoc {
                name: "pi_sign_query",
                description: "Reads sign facts for pi.",
            },
            BenchDoc {
                name: "pi_msd_query",
                description: "Reads magnitude facts for pi.",
            },
            BenchDoc {
                name: "pi_structural_facts",
                description: "Computes full structural facts for pi.",
            },
            BenchDoc {
                name: "e_zero_status",
                description: "Checks zero/nonzero facts for e.",
            },
            BenchDoc {
                name: "e_sign_query",
                description: "Reads sign facts for e.",
            },
            BenchDoc {
                name: "e_msd_query",
                description: "Reads magnitude facts for e.",
            },
            BenchDoc {
                name: "e_structural_facts",
                description: "Computes full structural facts for e.",
            },
            BenchDoc {
                name: "tau_zero_status",
                description: "Checks zero/nonzero facts for tau.",
            },
            BenchDoc {
                name: "tau_sign_query",
                description: "Reads sign facts for tau.",
            },
            BenchDoc {
                name: "tau_msd_query",
                description: "Reads magnitude facts for tau.",
            },
            BenchDoc {
                name: "tau_structural_facts",
                description: "Computes full structural facts for tau.",
            },
            BenchDoc {
                name: "sqrt_two_zero_status",
                description: "Checks zero/nonzero facts for sqrt(2).",
            },
            BenchDoc {
                name: "sqrt_two_sign_query",
                description: "Reads sign facts for sqrt(2).",
            },
            BenchDoc {
                name: "sqrt_two_msd_query",
                description: "Reads magnitude facts for sqrt(2).",
            },
            BenchDoc {
                name: "sqrt_two_structural_facts",
                description: "Computes full structural facts for sqrt(2).",
            },
            BenchDoc {
                name: "pi_minus_three_zero_status",
                description: "Checks zero/nonzero facts for pi - 3.",
            },
            BenchDoc {
                name: "pi_minus_three_sign_query",
                description: "Reads sign facts for pi - 3.",
            },
            BenchDoc {
                name: "pi_minus_three_msd_query",
                description: "Reads magnitude facts for pi - 3.",
            },
            BenchDoc {
                name: "pi_minus_three_structural_facts",
                description: "Computes full structural facts for pi - 3.",
            },
            BenchDoc {
                name: "dense_expr_zero_status",
                description: "Checks zero/nonzero facts for a dense composite expression.",
            },
            BenchDoc {
                name: "dense_expr_sign_query",
                description: "Reads sign facts for a dense composite expression.",
            },
            BenchDoc {
                name: "dense_expr_msd_query",
                description: "Reads magnitude facts for a dense composite expression.",
            },
            BenchDoc {
                name: "dense_expr_structural_facts",
                description: "Computes full structural facts for a dense composite expression.",
            },
        ],
    },
    BenchGroupDoc {
        name: "pure_scalar_algorithm_speed",
        description: "Core scalar algorithms that do not require high-precision transcendental approximation.",
        benches: &[
            BenchDoc {
                name: "rational_add",
                description: "Adds two nontrivial rational values.",
            },
            BenchDoc {
                name: "rational_mul",
                description: "Multiplies two nontrivial rational values.",
            },
            BenchDoc {
                name: "rational_div",
                description: "Divides two nontrivial rational values.",
            },
            BenchDoc {
                name: "real_exact_add",
                description: "Adds exact rational-backed `Real` values.",
            },
            BenchDoc {
                name: "real_exact_mul",
                description: "Multiplies exact rational-backed `Real` values.",
            },
            BenchDoc {
                name: "real_exact_div",
                description: "Divides exact rational-backed `Real` values.",
            },
            BenchDoc {
                name: "real_exact_sqrt_reduce",
                description: "Reduces an exact square-root expression.",
            },
            BenchDoc {
                name: "real_exact_ln_reduce",
                description: "Reduces an exact logarithm of a power of two.",
            },
        ],
    },
    BenchGroupDoc {
        name: "borrowed_op_overhead",
        description: "Borrowed versus owned operation overhead for rational and real operands.",
        benches: &[
            BenchDoc {
                name: "rational_clone_pair",
                description: "Clones two rational values.",
            },
            BenchDoc {
                name: "rational_add_refs",
                description: "Adds rational references.",
            },
            BenchDoc {
                name: "rational_add_owned",
                description: "Adds owned rational values.",
            },
            BenchDoc {
                name: "real_clone_pair",
                description: "Clones two `Real` values.",
            },
            BenchDoc {
                name: "real_add_refs",
                description: "Adds `Real` references.",
            },
            BenchDoc {
                name: "real_add_owned",
                description: "Adds owned `Real` values.",
            },
        ],
    },
    BenchGroupDoc {
        name: "dense_algebra",
        description: "Small dense algebra kernels that stress repeated exact and symbolic operations.",
        benches: &[
            BenchDoc {
                name: "rational_dot_64",
                description: "Computes a 64-element rational dot product.",
            },
            BenchDoc {
                name: "rational_matmul_8",
                description: "Computes an 8x8 rational matrix multiply.",
            },
            BenchDoc {
                name: "real_dot_36",
                description: "Computes a 36-element dot product over symbolic `Real` values.",
            },
            BenchDoc {
                name: "real_matmul_6",
                description: "Computes a 6x6 matrix multiply over symbolic `Real` values.",
            },
        ],
    },
];

fn rational(n: i64, d: u64) -> Rational {
    Rational::fraction(n, d).unwrap()
}

fn real(n: i64, d: u64) -> Real {
    Real::new(rational(n, d))
}

fn tau_computable() -> Computable {
    Computable::pi().multiply(Computable::rational(Rational::new(2)))
}

fn warm_cache(value: &Computable, precision: i32) {
    black_box(value.approx(precision));
}

fn structural_values() -> Vec<(&'static str, Real)> {
    let tiny = Real::new(
        Rational::from_bigint_fraction(BigInt::from(1), BigUint::from(1_u8) << 160).unwrap(),
    );
    let tau = Real::pi() * Real::new(Rational::new(2));
    let pi_minus_three = Real::pi() - Real::new(Rational::new(3));
    let sqrt_two = Real::new(Rational::new(2)).sqrt().unwrap();
    let dense_expr = ((Real::pi() * real(7, 8)) + sqrt_two.clone()) * real(3, 5);

    vec![
        ("zero", Real::zero()),
        ("one", Real::new(Rational::one())),
        ("negative", Real::new(Rational::new(-7))),
        ("tiny_exact", tiny),
        ("pi", Real::pi()),
        ("e", Real::e()),
        ("tau", tau),
        ("sqrt_two", sqrt_two),
        ("pi_minus_three", pi_minus_three),
        ("dense_expr", dense_expr),
    ]
}

fn rational_dot(left: &[Rational], right: &[Rational]) -> Rational {
    left.iter()
        .zip(right)
        .fold(Rational::zero(), |acc, (left, right)| {
            acc + black_box(left) * black_box(right)
        })
}

fn real_dot(left: &[Real], right: &[Real]) -> Real {
    left.iter()
        .zip(right)
        .fold(Real::zero(), |acc, (left, right)| {
            acc + (black_box(left) * black_box(right))
        })
}

fn rational_matmul_8(left: &[Rational], right: &[Rational]) -> Vec<Rational> {
    let n = 8;
    let mut out = vec![Rational::zero(); n * n];
    for row in 0..n {
        for col in 0..n {
            let mut sum = Rational::zero();
            for k in 0..n {
                sum = sum + black_box(&left[row * n + k]) * black_box(&right[k * n + col]);
            }
            out[row * n + col] = sum;
        }
    }
    out
}

fn real_matmul_6(left: &[Real], right: &[Real]) -> Vec<Real> {
    let n = 6;
    let mut out = vec![Real::zero(); n * n];
    for row in 0..n {
        for col in 0..n {
            let mut sum = Real::zero();
            for k in 0..n {
                sum = sum + (black_box(&left[row * n + k]) * black_box(&right[k * n + col]));
            }
            out[row * n + col] = sum;
        }
    }
    out
}

fn bench_raw_cache_hit_cost(c: &mut Criterion) {
    bench_docs::write_benchmark_docs(
        "scalar_micro",
        "Microbenchmarks for scalar operations, structural queries, cache hits, and dense exact arithmetic.",
        SCALAR_MICRO_GROUPS,
    );

    let mut group = c.benchmark_group("raw_cache_hit_cost");
    let precision = -128;
    let cached_precision = -256;
    let values = [
        ("zero", Computable::rational(Rational::zero())),
        ("one", Computable::one()),
        ("two", Computable::rational(Rational::new(2))),
        ("e", Computable::rational(Rational::one()).exp()),
        ("pi", Computable::pi()),
        ("tau", tau_computable()),
    ];

    for (name, value) in values {
        warm_cache(&value, cached_precision);
        group.bench_function(name, |b| {
            b.iter(|| black_box(value.approx(black_box(precision))))
        });
    }

    group.finish();
}

fn bench_structural_query_speed(c: &mut Criterion) {
    let mut group = c.benchmark_group("structural_query_speed");

    for (name, value) in structural_values() {
        black_box(value.structural_facts());

        group.bench_function(format!("{name}_zero_status"), |b| {
            b.iter(|| black_box(black_box(&value).zero_status()))
        });
        group.bench_function(format!("{name}_sign_query"), |b| {
            b.iter(|| black_box(black_box(&value).structural_facts().sign))
        });
        group.bench_function(format!("{name}_msd_query"), |b| {
            b.iter(|| black_box(black_box(&value).structural_facts().magnitude))
        });
        group.bench_function(format!("{name}_structural_facts"), |b| {
            b.iter(|| black_box(black_box(&value).structural_facts()))
        });
    }

    group.finish();
}

fn bench_pure_scalar_algorithm_speed(c: &mut Criterion) {
    let mut group = c.benchmark_group("pure_scalar_algorithm_speed");
    let lhs = rational(123_456_789, 987_654_321);
    let rhs = rational(987_654_321, 123_456_789);
    let exact_real_lhs = Real::new(lhs.clone());
    let exact_real_rhs = Real::new(rhs.clone());
    let sqrt_input = Real::new(Rational::new(18));
    let ln_input = Real::new(Rational::new(1024));

    group.bench_function("rational_add", |b| {
        b.iter(|| black_box(black_box(&lhs) + black_box(&rhs)))
    });
    group.bench_function("rational_mul", |b| {
        b.iter(|| black_box(black_box(&lhs) * black_box(&rhs)))
    });
    group.bench_function("rational_div", |b| {
        b.iter(|| black_box(black_box(&lhs) / black_box(&rhs)))
    });
    group.bench_function("real_exact_add", |b| {
        b.iter(|| black_box(black_box(&exact_real_lhs) + black_box(&exact_real_rhs)))
    });
    group.bench_function("real_exact_mul", |b| {
        b.iter(|| black_box(black_box(&exact_real_lhs) * black_box(&exact_real_rhs)))
    });
    group.bench_function("real_exact_div", |b| {
        b.iter(|| black_box((black_box(&exact_real_lhs) / black_box(&exact_real_rhs)).unwrap()))
    });
    group.bench_function("real_exact_sqrt_reduce", |b| {
        b.iter_batched(
            || sqrt_input.clone(),
            |value| black_box(value.sqrt().unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("real_exact_ln_reduce", |b| {
        b.iter_batched(
            || ln_input.clone(),
            |value| black_box(value.ln().unwrap()),
            BatchSize::SmallInput,
        )
    });

    group.finish();
}

fn bench_borrowed_op_overhead(c: &mut Criterion) {
    let mut group = c.benchmark_group("borrowed_op_overhead");
    let rational_lhs = rational(123_456_789, 987_654_321);
    let rational_rhs = rational(987_654_321, 123_456_789);
    let real_lhs = Real::pi() * real(7, 8);
    let real_rhs = Real::e() * real(5, 6);

    group.bench_function("rational_clone_pair", |b| {
        b.iter(|| {
            black_box((
                black_box(&rational_lhs).clone(),
                black_box(&rational_rhs).clone(),
            ))
        })
    });
    group.bench_function("rational_add_refs", |b| {
        b.iter(|| black_box(black_box(&rational_lhs) + black_box(&rational_rhs)))
    });
    group.bench_function("rational_add_owned", |b| {
        b.iter_batched(
            || (rational_lhs.clone(), rational_rhs.clone()),
            |(left, right)| black_box(left + right),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("real_clone_pair", |b| {
        b.iter(|| black_box((black_box(&real_lhs).clone(), black_box(&real_rhs).clone())))
    });
    group.bench_function("real_add_refs", |b| {
        b.iter(|| black_box(black_box(&real_lhs) + black_box(&real_rhs)))
    });
    group.bench_function("real_add_owned", |b| {
        b.iter_batched(
            || (real_lhs.clone(), real_rhs.clone()),
            |(left, right)| black_box(left + right),
            BatchSize::SmallInput,
        )
    });

    group.finish();
}

fn bench_dense_algebra(c: &mut Criterion) {
    let mut group = c.benchmark_group("dense_algebra");
    let rational_left: Vec<_> = (1..=64).map(|n| rational(n, (n as u64 % 11) + 2)).collect();
    let rational_right: Vec<_> = (65..=128)
        .map(|n| rational(n, (n as u64 % 13) + 2))
        .collect();
    let real_left: Vec<_> = (1..=36)
        .map(|n| Real::pi() * real(n, (n as u64 % 7) + 2))
        .collect();
    let real_right: Vec<_> = (37..=72)
        .map(|n| Real::e() * real(n, (n as u64 % 5) + 2))
        .collect();

    group.bench_function("rational_dot_64", |b| {
        b.iter(|| {
            black_box(rational_dot(
                black_box(&rational_left),
                black_box(&rational_right),
            ))
        })
    });
    group.bench_function("rational_matmul_8", |b| {
        b.iter(|| {
            black_box(rational_matmul_8(
                black_box(&rational_left),
                black_box(&rational_right),
            ))
        })
    });
    group.bench_function("real_dot_36", |b| {
        b.iter(|| black_box(real_dot(black_box(&real_left), black_box(&real_right))))
    });
    group.bench_function("real_matmul_6", |b| {
        b.iter(|| black_box(real_matmul_6(black_box(&real_left), black_box(&real_right))))
    });

    group.finish();
}

criterion_group!(
    benches,
    bench_raw_cache_hit_cost,
    bench_structural_query_speed,
    bench_pure_scalar_algorithm_speed,
    bench_borrowed_op_overhead,
    bench_dense_algebra
);
criterion_main!(benches);
