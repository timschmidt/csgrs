use criterion::{BatchSize, Criterion, black_box, criterion_group, criterion_main};
use hyperreal::{Rational, Real, Simple};
use num::bigint::BigInt;

#[path = "support/bench_docs.rs"]
mod bench_docs;

use bench_docs::{BenchDoc, BenchGroupDoc};

const LIBRARY_PERF_GROUPS: &[BenchGroupDoc] = &[
    BenchGroupDoc {
        name: "real_format",
        description: "Formatting costs for important irrational `Real` values.",
        benches: &[
            BenchDoc {
                name: "pi_lower_exp_32",
                description: "Formats pi with 32 digits in lower-exponential form.",
            },
            BenchDoc {
                name: "pi_display_alt_32",
                description: "Formats pi with alternate decimal display at 32 digits.",
            },
            BenchDoc {
                name: "sqrt_two_display_alt_32",
                description: "Formats sqrt(2) with alternate decimal display at 32 digits.",
            },
        ],
    },
    BenchGroupDoc {
        name: "real_constants",
        description: "Construction cost for shared mathematical constants.",
        benches: &[
            BenchDoc {
                name: "pi",
                description: "Constructs the symbolic pi value.",
            },
            BenchDoc {
                name: "e",
                description: "Constructs the symbolic Euler constant value.",
            },
        ],
    },
    BenchGroupDoc {
        name: "simple",
        description: "Parser and evaluator costs for the `Simple` expression language.",
        benches: &[
            BenchDoc {
                name: "parse_nested",
                description: "Parses a nested expression with powers, trig, and constants.",
            },
            BenchDoc {
                name: "eval_nested",
                description: "Evaluates a parsed mixed symbolic/numeric expression.",
            },
            BenchDoc {
                name: "eval_constants",
                description: "Evaluates repeated built-in constants.",
            },
            BenchDoc {
                name: "eval_exact",
                description: "Evaluates a rational-only expression through exact shortcuts.",
            },
            BenchDoc {
                name: "eval_nested_exact",
                description: "Evaluates a nested rational-only expression through exact shortcuts.",
            },
        ],
    },
    BenchGroupDoc {
        name: "real_powi",
        description: "Integer exponentiation for exact and irrational `Real` values.",
        benches: &[
            BenchDoc {
                name: "exact_17",
                description: "Raises an exact rational-backed `Real` to the 17th power.",
            },
            BenchDoc {
                name: "irrational_17",
                description: "Raises sqrt(3) to the 17th power with symbolic simplification.",
            },
        ],
    },
    BenchGroupDoc {
        name: "rational_powi",
        description: "Integer exponentiation for `Rational`.",
        benches: &[BenchDoc {
            name: "exact_17",
            description: "Raises a rational value to the 17th power.",
        }],
    },
    BenchGroupDoc {
        name: "real_exact_trig",
        description: "Exact and symbolic trig construction for known pi multiples.",
        benches: &[
            BenchDoc {
                name: "sin_pi_6",
                description: "Computes sin(pi/6) via exact shortcut.",
            },
            BenchDoc {
                name: "cos_pi_3",
                description: "Computes cos(pi/3) via exact shortcut.",
            },
            BenchDoc {
                name: "tan_pi_5",
                description: "Builds tan(pi/5), a nontrivial symbolic tangent.",
            },
        ],
    },
    BenchGroupDoc {
        name: "real_general_trig",
        description: "General trig construction for irrational arguments.",
        benches: &[
            BenchDoc {
                name: "tan_sqrt_2",
                description: "Builds tan(sqrt(2)).",
            },
            BenchDoc {
                name: "tan_pi_sqrt_2_over_5",
                description: "Builds tangent of an irrational multiple of pi.",
            },
        ],
    },
    BenchGroupDoc {
        name: "real_exact_inverse_trig",
        description: "Exact inverse trig shortcuts and symbolic inverse trig recognition.",
        benches: &[
            BenchDoc {
                name: "asin_1_2",
                description: "Recognizes asin(1/2) as pi/6.",
            },
            BenchDoc {
                name: "asin_minus_1_2",
                description: "Recognizes asin(-1/2) as -pi/6.",
            },
            BenchDoc {
                name: "asin_sqrt_2_over_2",
                description: "Recognizes asin(sqrt(2)/2) as pi/4.",
            },
            BenchDoc {
                name: "asin_sin_pi_5",
                description: "Inverts a symbolic sin(pi/5).",
            },
            BenchDoc {
                name: "acos_1",
                description: "Recognizes acos(1) as zero.",
            },
            BenchDoc {
                name: "acos_minus_1",
                description: "Recognizes acos(-1) as pi.",
            },
            BenchDoc {
                name: "acos_1_2",
                description: "Recognizes acos(1/2) as pi/3.",
            },
            BenchDoc {
                name: "atan_1",
                description: "Recognizes atan(1) as pi/4.",
            },
            BenchDoc {
                name: "atan_sqrt_3_over_3",
                description: "Recognizes atan(sqrt(3)/3) as pi/6.",
            },
            BenchDoc {
                name: "atan_tan_pi_5",
                description: "Inverts a symbolic tan(pi/5).",
            },
        ],
    },
    BenchGroupDoc {
        name: "real_general_inverse_trig",
        description: "General inverse trig construction, domain errors, and atan range reduction.",
        benches: &[
            BenchDoc {
                name: "asin_7_10",
                description: "Builds asin(7/10) through the rational-specialized path.",
            },
            BenchDoc {
                name: "asin_sqrt_2_over_3",
                description: "Builds asin(sqrt(2)/3) through the general path.",
            },
            BenchDoc {
                name: "acos_7_10",
                description: "Builds acos(7/10) through the rational-specialized asin path.",
            },
            BenchDoc {
                name: "acos_sqrt_2_over_3",
                description: "Builds acos(sqrt(2)/3) through the general path.",
            },
            BenchDoc {
                name: "asin_11_10_error",
                description: "Rejects rational asin input outside [-1, 1].",
            },
            BenchDoc {
                name: "acos_11_10_error",
                description: "Rejects rational acos input outside [-1, 1].",
            },
            BenchDoc {
                name: "atan_8",
                description: "Builds atan(8), exercising large-argument reduction.",
            },
            BenchDoc {
                name: "atan_sqrt_2",
                description: "Builds atan(sqrt(2)).",
            },
        ],
    },
    BenchGroupDoc {
        name: "real_inverse_hyperbolic",
        description: "Inverse hyperbolic construction, exact exits, stable ln1p forms, and domain errors.",
        benches: &[
            BenchDoc {
                name: "asinh_0",
                description: "Recognizes asinh(0) as zero.",
            },
            BenchDoc {
                name: "asinh_1_2",
                description: "Builds asinh(1/2) through the stable moderate-input path.",
            },
            BenchDoc {
                name: "asinh_sqrt_2",
                description: "Builds asinh(sqrt(2)) without cancellation-prone log construction.",
            },
            BenchDoc {
                name: "asinh_minus_1_2",
                description: "Uses odd symmetry for negative asinh input.",
            },
            BenchDoc {
                name: "asinh_1_000_000",
                description: "Builds asinh for a large positive rational.",
            },
            BenchDoc {
                name: "acosh_1",
                description: "Recognizes acosh(1) as zero.",
            },
            BenchDoc {
                name: "acosh_2",
                description: "Builds acosh(2) through the stable moderate-input path.",
            },
            BenchDoc {
                name: "acosh_sqrt_2",
                description: "Builds acosh(sqrt(2)) through square-root domain specialization.",
            },
            BenchDoc {
                name: "acosh_1_000_000",
                description: "Builds acosh for a large positive rational.",
            },
            BenchDoc {
                name: "atanh_0",
                description: "Recognizes atanh(0) as zero.",
            },
            BenchDoc {
                name: "atanh_1_2",
                description: "Builds exact-rational atanh(1/2).",
            },
            BenchDoc {
                name: "atanh_minus_1_2",
                description: "Builds exact-rational atanh(-1/2).",
            },
            BenchDoc {
                name: "atanh_sqrt_half",
                description: "Recognizes atanh(sqrt(2)/2) as asinh(1).",
            },
            BenchDoc {
                name: "atanh_9_10",
                description: "Builds exact-rational atanh near the upper domain boundary.",
            },
            BenchDoc {
                name: "atanh_1_error",
                description: "Rejects atanh(1) at the rational domain boundary.",
            },
        ],
    },
    BenchGroupDoc {
        name: "simple_inverse_functions",
        description: "Parsed/evaluated inverse trig and inverse hyperbolic expressions that should succeed.",
        benches: &[
            BenchDoc {
                name: "asin_1_2",
                description: "Evaluates `(asin 1/2)`.",
            },
            BenchDoc {
                name: "acos_1_2",
                description: "Evaluates `(acos 1/2)`.",
            },
            BenchDoc {
                name: "atan_1",
                description: "Evaluates `(atan 1)`.",
            },
            BenchDoc {
                name: "asin_general",
                description: "Evaluates `(asin 7/10)`.",
            },
            BenchDoc {
                name: "acos_general",
                description: "Evaluates `(acos 7/10)`.",
            },
            BenchDoc {
                name: "atan_general",
                description: "Evaluates `(atan 8)`.",
            },
            BenchDoc {
                name: "asinh_1_2",
                description: "Evaluates `(asinh 1/2)`.",
            },
            BenchDoc {
                name: "asinh_sqrt_2",
                description: "Evaluates `(asinh (sqrt 2))`.",
            },
            BenchDoc {
                name: "acosh_2",
                description: "Evaluates `(acosh 2)`.",
            },
            BenchDoc {
                name: "acosh_sqrt_2",
                description: "Evaluates `(acosh (sqrt 2))`.",
            },
            BenchDoc {
                name: "atanh_1_2",
                description: "Evaluates `(atanh 1/2)`.",
            },
            BenchDoc {
                name: "atanh_minus_1_2",
                description: "Evaluates `(atanh -1/2)`.",
            },
        ],
    },
    BenchGroupDoc {
        name: "simple_inverse_error_functions",
        description: "Parsed/evaluated inverse function expressions that should fail quickly with `NotANumber`.",
        benches: &[
            BenchDoc {
                name: "asin_11_10",
                description: "Rejects `(asin 11/10)`.",
            },
            BenchDoc {
                name: "acos_sqrt_2",
                description: "Rejects `(acos (sqrt 2))`.",
            },
            BenchDoc {
                name: "acosh_0",
                description: "Rejects `(acosh 0)`.",
            },
            BenchDoc {
                name: "acosh_minus_2",
                description: "Rejects `(acosh -2)`.",
            },
            BenchDoc {
                name: "atanh_1",
                description: "Rejects `(atanh 1)`.",
            },
            BenchDoc {
                name: "atanh_sqrt_2",
                description: "Rejects `(atanh (sqrt 2))`.",
            },
        ],
    },
    BenchGroupDoc {
        name: "real_exact_ln",
        description: "Exact logarithm construction and simplification for rational inputs.",
        benches: &[
            BenchDoc {
                name: "ln_1024",
                description: "Recognizes ln(1024) as 10 ln(2).",
            },
            BenchDoc {
                name: "ln_1_8",
                description: "Recognizes ln(1/8) as -3 ln(2).",
            },
            BenchDoc {
                name: "ln_1000",
                description: "Simplifies ln(1000) via small integer logarithm factors.",
            },
        ],
    },
    BenchGroupDoc {
        name: "real_exact_exp_log10",
        description: "Exact inverse relationships among exp, ln, and log10.",
        benches: &[
            BenchDoc {
                name: "exp_ln_1000",
                description: "Simplifies exp(ln(1000)) back to 1000.",
            },
            BenchDoc {
                name: "exp_ln_1_8",
                description: "Simplifies exp(ln(1/8)) back to 1/8.",
            },
            BenchDoc {
                name: "log10_1000",
                description: "Recognizes log10(1000) as 3.",
            },
            BenchDoc {
                name: "log10_1_1000",
                description: "Recognizes log10(1/1000) as -3.",
            },
        ],
    },
];

fn bench_real_format(c: &mut Criterion) {
    bench_docs::write_benchmark_docs(
        "library_perf",
        "Library-level Criterion benchmarks for public `Rational`, `Real`, and `Simple` behavior.",
        LIBRARY_PERF_GROUPS,
    );

    let mut group = c.benchmark_group("real_format");
    let pi = Real::pi();
    let sqrt_two = Real::new(Rational::new(2)).sqrt().unwrap();

    group.bench_function("pi_lower_exp_32", |b| {
        b.iter(|| black_box(format!("{:.32e}", black_box(&pi))))
    });
    group.bench_function("pi_display_alt_32", |b| {
        b.iter(|| black_box(format!("{:#.32}", black_box(&pi))))
    });
    group.bench_function("sqrt_two_display_alt_32", |b| {
        b.iter(|| black_box(format!("{:#.32}", black_box(&sqrt_two))))
    });

    group.finish();
}

fn bench_real_constants(c: &mut Criterion) {
    let mut group = c.benchmark_group("real_constants");

    group.bench_function("pi", |b| b.iter(|| black_box(Real::pi())));
    group.bench_function("e", |b| b.iter(|| black_box(Real::e())));

    group.finish();
}

fn bench_simple(c: &mut Criterion) {
    let mut group = c.benchmark_group("simple");
    let source = "(* (+ pi pi) (pow (+ 3/2 4/7) 9/2) (sin (/ 1 5)))";
    let parsed: Simple = source.parse().unwrap();
    let constants_source = "(+ pi e pi e pi e pi e)";
    let constants_parsed: Simple = constants_source.parse().unwrap();
    let exact_source = "(/ (* (+ 7/5 11/7 13/9) (- 19/11 1/3)) 23/17)";
    let exact_parsed: Simple = exact_source.parse().unwrap();
    let nested_exact_source =
        "(* (+ (/ 7 5) (/ 11 7) (/ 13 9)) (pow (+ 5/4 3/8) 9) (/ (- 19/11 1/3) (+ 1/7 2/7)))";
    let nested_exact_parsed: Simple = nested_exact_source.parse().unwrap();

    group.bench_function("parse_nested", |b| {
        b.iter(|| black_box(black_box(source).parse::<Simple>().unwrap()))
    });
    group.bench_function("eval_nested", |b| {
        b.iter_batched(
            || parsed.clone(),
            |expr| black_box(expr.evaluate(&Default::default()).unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("eval_constants", |b| {
        b.iter_batched(
            || constants_parsed.clone(),
            |expr| black_box(expr.evaluate(&Default::default()).unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("eval_exact", |b| {
        b.iter_batched(
            || exact_parsed.clone(),
            |expr| black_box(expr.evaluate(&Default::default()).unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("eval_nested_exact", |b| {
        b.iter_batched(
            || nested_exact_parsed.clone(),
            |expr| black_box(expr.evaluate(&Default::default()).unwrap()),
            BatchSize::SmallInput,
        )
    });

    group.finish();
}

fn bench_real_powi(c: &mut Criterion) {
    let mut group = c.benchmark_group("real_powi");
    let exact = Real::new(Rational::fraction(7, 5).unwrap());
    let irrational = Real::new(Rational::new(3)).sqrt().unwrap();
    let exp = BigInt::from(17_u8);
    let negative_one = BigInt::from(-1_i8);

    group.bench_function("exact_17", |b| {
        b.iter_batched(
            || exact.clone(),
            |value| black_box(value.powi(exp.clone()).unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("irrational_17", |b| {
        b.iter_batched(
            || irrational.clone(),
            |value| black_box(value.powi(exp.clone()).unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("pi_negative_one", |b| {
        b.iter_batched(
            Real::pi,
            |value| black_box(value.powi(negative_one.clone()).unwrap()),
            BatchSize::SmallInput,
        )
    });

    group.finish();
}

fn bench_rational_powi(c: &mut Criterion) {
    let mut group = c.benchmark_group("rational_powi");
    let value = Rational::fraction(7, 5).unwrap();
    let exp = BigInt::from(17_u8);

    group.bench_function("exact_17", |b| {
        b.iter_batched(
            || value.clone(),
            |value| black_box(value.powi(exp.clone()).unwrap()),
            BatchSize::SmallInput,
        )
    });

    group.finish();
}

fn bench_real_exact_trig(c: &mut Criterion) {
    let mut group = c.benchmark_group("real_exact_trig");
    let pi_sixth = Real::pi() * Real::new(Rational::fraction(1, 6).unwrap());
    let pi_third = Real::pi() * Real::new(Rational::fraction(1, 3).unwrap());
    let pi_fifth = Real::pi() * Real::new(Rational::fraction(1, 5).unwrap());

    group.bench_function("sin_pi_6", |b| {
        b.iter_batched(
            || pi_sixth.clone(),
            |value| black_box(value.sin()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("cos_pi_3", |b| {
        b.iter_batched(
            || pi_third.clone(),
            |value| black_box(value.cos()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("tan_pi_5", |b| {
        b.iter_batched(
            || pi_fifth.clone(),
            |value| black_box(value.tan().unwrap()),
            BatchSize::SmallInput,
        )
    });

    group.finish();
}

fn bench_real_general_trig(c: &mut Criterion) {
    let mut group = c.benchmark_group("real_general_trig");
    let sqrt_two = Real::new(Rational::new(2)).sqrt().unwrap();
    let pi = Real::pi();
    let irrational_pi_mix = (pi * sqrt_two.clone()) / Real::new(Rational::new(5));
    let irrational_pi_mix = irrational_pi_mix.unwrap();

    group.bench_function("tan_sqrt_2", |b| {
        b.iter_batched(
            || sqrt_two.clone(),
            |value| black_box(value.tan().unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("tan_pi_sqrt_2_over_5", |b| {
        b.iter_batched(
            || irrational_pi_mix.clone(),
            |value| black_box(value.tan().unwrap()),
            BatchSize::SmallInput,
        )
    });

    group.finish();
}

fn bench_real_exact_inverse_trig(c: &mut Criterion) {
    let mut group = c.benchmark_group("real_exact_inverse_trig");
    let half = Real::new(Rational::fraction(1, 2).unwrap());
    let minus_half = Real::new(Rational::fraction(-1, 2).unwrap());
    let one = Real::new(Rational::one());
    let minus_one = Real::new(Rational::new(-1));
    let sqrt_two_over_two =
        Real::new(Rational::fraction(1, 2).unwrap()) * Real::new(Rational::new(2)).sqrt().unwrap();
    let sqrt_three_over_three =
        Real::new(Rational::fraction(1, 3).unwrap()) * Real::new(Rational::new(3)).sqrt().unwrap();
    let sin_pi_fifth = (Real::pi() * Real::new(Rational::fraction(1, 5).unwrap())).sin();
    let tan_pi_fifth = (Real::pi() * Real::new(Rational::fraction(1, 5).unwrap()))
        .tan()
        .unwrap();

    group.bench_function("asin_1_2", |b| {
        b.iter_batched(
            || half.clone(),
            |value| black_box(value.asin().unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("asin_minus_1_2", |b| {
        b.iter_batched(
            || minus_half.clone(),
            |value| black_box(value.asin().unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("asin_sqrt_2_over_2", |b| {
        b.iter_batched(
            || sqrt_two_over_two.clone(),
            |value| black_box(value.asin().unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("asin_sin_pi_5", |b| {
        b.iter_batched(
            || sin_pi_fifth.clone(),
            |value| black_box(value.asin().unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("acos_1", |b| {
        b.iter_batched(
            || one.clone(),
            |value| black_box(value.acos().unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("acos_minus_1", |b| {
        b.iter_batched(
            || minus_one.clone(),
            |value| black_box(value.acos().unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("acos_1_2", |b| {
        b.iter_batched(
            || half.clone(),
            |value| black_box(value.acos().unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("atan_1", |b| {
        b.iter_batched(
            || one.clone(),
            |value| black_box(value.atan().unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("atan_sqrt_3_over_3", |b| {
        b.iter_batched(
            || sqrt_three_over_three.clone(),
            |value| black_box(value.atan().unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("atan_tan_pi_5", |b| {
        b.iter_batched(
            || tan_pi_fifth.clone(),
            |value| black_box(value.atan().unwrap()),
            BatchSize::SmallInput,
        )
    });

    group.finish();
}

fn bench_real_general_inverse_trig(c: &mut Criterion) {
    let mut group = c.benchmark_group("real_general_inverse_trig");
    let rational_in_domain = Real::new(Rational::fraction(7, 10).unwrap());
    let rational_out_of_domain = Real::new(Rational::fraction(11, 10).unwrap());
    let irrational_in_domain =
        Real::new(Rational::new(2)).sqrt().unwrap() / Real::new(Rational::new(3));
    let irrational_in_domain = irrational_in_domain.unwrap();
    let atan_large = Real::new(Rational::new(8));
    let atan_irrational = Real::new(Rational::new(2)).sqrt().unwrap();

    group.bench_function("asin_7_10", |b| {
        b.iter_batched(
            || rational_in_domain.clone(),
            |value| black_box(value.asin().unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("asin_sqrt_2_over_3", |b| {
        b.iter_batched(
            || irrational_in_domain.clone(),
            |value| black_box(value.asin().unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("acos_7_10", |b| {
        b.iter_batched(
            || rational_in_domain.clone(),
            |value| black_box(value.acos().unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("acos_sqrt_2_over_3", |b| {
        b.iter_batched(
            || irrational_in_domain.clone(),
            |value| black_box(value.acos().unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("asin_11_10_error", |b| {
        b.iter_batched(
            || rational_out_of_domain.clone(),
            |value| black_box(value.asin().unwrap_err()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("acos_11_10_error", |b| {
        b.iter_batched(
            || rational_out_of_domain.clone(),
            |value| black_box(value.acos().unwrap_err()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("atan_8", |b| {
        b.iter_batched(
            || atan_large.clone(),
            |value| black_box(value.atan().unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("atan_sqrt_2", |b| {
        b.iter_batched(
            || atan_irrational.clone(),
            |value| black_box(value.atan().unwrap()),
            BatchSize::SmallInput,
        )
    });

    group.finish();
}

fn bench_real_inverse_hyperbolic(c: &mut Criterion) {
    let mut group = c.benchmark_group("real_inverse_hyperbolic");
    let zero = Real::zero();
    let half = Real::new(Rational::fraction(1, 2).unwrap());
    let minus_half = Real::new(Rational::fraction(-1, 2).unwrap());
    let nine_tenths = Real::new(Rational::fraction(9, 10).unwrap());
    let two = Real::new(Rational::new(2));
    let million = Real::new(Rational::new(1_000_000));
    let sqrt_two = Real::new(Rational::new(2)).sqrt().unwrap();
    let sqrt_half = sqrt_two.clone() * Real::new(Rational::fraction(1, 2).unwrap());

    group.bench_function("asinh_0", |b| {
        b.iter_batched(
            || zero.clone(),
            |value| black_box(value.asinh().unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("asinh_1_2", |b| {
        b.iter_batched(
            || half.clone(),
            |value| black_box(value.asinh().unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("asinh_sqrt_2", |b| {
        b.iter_batched(
            || sqrt_two.clone(),
            |value| black_box(value.asinh().unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("asinh_minus_1_2", |b| {
        b.iter_batched(
            || minus_half.clone(),
            |value| black_box(value.asinh().unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("asinh_1_000_000", |b| {
        b.iter_batched(
            || million.clone(),
            |value| black_box(value.asinh().unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("acosh_1", |b| {
        b.iter_batched(
            || Real::new(Rational::one()),
            |value| black_box(value.acosh().unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("acosh_2", |b| {
        b.iter_batched(
            || two.clone(),
            |value| black_box(value.acosh().unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("acosh_sqrt_2", |b| {
        b.iter_batched(
            || sqrt_two.clone(),
            |value| black_box(value.acosh().unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("acosh_1_000_000", |b| {
        b.iter_batched(
            || million.clone(),
            |value| black_box(value.acosh().unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("atanh_0", |b| {
        b.iter_batched(
            || zero.clone(),
            |value| black_box(value.atanh().unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("atanh_1_2", |b| {
        b.iter_batched(
            || half.clone(),
            |value| black_box(value.atanh().unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("atanh_minus_1_2", |b| {
        b.iter_batched(
            || minus_half.clone(),
            |value| black_box(value.atanh().unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("atanh_sqrt_half", |b| {
        b.iter_batched(
            || sqrt_half.clone(),
            |value| black_box(value.atanh().unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("atanh_9_10", |b| {
        b.iter_batched(
            || nine_tenths.clone(),
            |value| black_box(value.atanh().unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("atanh_1_error", |b| {
        b.iter_batched(
            || Real::new(Rational::one()),
            |value| black_box(value.atanh().unwrap_err()),
            BatchSize::SmallInput,
        )
    });

    group.finish();
}

fn bench_simple_inverse_functions(c: &mut Criterion) {
    let mut group = c.benchmark_group("simple_inverse_functions");
    let expressions: [(&str, Simple); 12] = [
        ("asin_1_2", "(asin 1/2)".parse().unwrap()),
        ("acos_1_2", "(acos 1/2)".parse().unwrap()),
        ("atan_1", "(atan 1)".parse().unwrap()),
        ("asin_general", "(asin 7/10)".parse().unwrap()),
        ("acos_general", "(acos 7/10)".parse().unwrap()),
        ("atan_general", "(atan 8)".parse().unwrap()),
        ("asinh_1_2", "(asinh 1/2)".parse().unwrap()),
        ("asinh_sqrt_2", "(asinh (sqrt 2))".parse().unwrap()),
        ("acosh_2", "(acosh 2)".parse().unwrap()),
        ("acosh_sqrt_2", "(acosh (sqrt 2))".parse().unwrap()),
        ("atanh_1_2", "(atanh 1/2)".parse().unwrap()),
        ("atanh_minus_1_2", "(atanh -1/2)".parse().unwrap()),
    ];

    for (name, expr) in expressions {
        group.bench_function(name, |b| {
            b.iter_batched(
                || expr.clone(),
                |expr| black_box(expr.evaluate(&Default::default()).unwrap()),
                BatchSize::SmallInput,
            )
        });
    }

    group.finish();
}

fn bench_simple_inverse_error_functions(c: &mut Criterion) {
    let mut group = c.benchmark_group("simple_inverse_error_functions");
    let expressions: [(&str, Simple); 6] = [
        ("asin_11_10", "(asin 11/10)".parse().unwrap()),
        ("acos_sqrt_2", "(acos (sqrt 2))".parse().unwrap()),
        ("acosh_0", "(acosh 0)".parse().unwrap()),
        ("acosh_minus_2", "(acosh -2)".parse().unwrap()),
        ("atanh_1", "(atanh 1)".parse().unwrap()),
        ("atanh_sqrt_2", "(atanh (sqrt 2))".parse().unwrap()),
    ];

    for (name, expr) in expressions {
        group.bench_function(name, |b| {
            b.iter_batched(
                || expr.clone(),
                |expr| black_box(expr.evaluate(&Default::default()).unwrap_err()),
                BatchSize::SmallInput,
            )
        });
    }

    group.finish();
}

fn bench_real_exact_ln(c: &mut Criterion) {
    let mut group = c.benchmark_group("real_exact_ln");
    let ln_1024 = Real::new(Rational::new(1024));
    let ln_eighth = Real::new(Rational::fraction(1, 8).unwrap());
    let ln_1000 = Real::new(Rational::new(1000));

    group.bench_function("ln_1024", |b| {
        b.iter_batched(
            || ln_1024.clone(),
            |value| black_box(value.ln().unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("ln_1_8", |b| {
        b.iter_batched(
            || ln_eighth.clone(),
            |value| black_box(value.ln().unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("ln_1000", |b| {
        b.iter_batched(
            || ln_1000.clone(),
            |value| black_box(value.ln().unwrap()),
            BatchSize::SmallInput,
        )
    });

    group.finish();
}

fn bench_real_exact_exp_log10(c: &mut Criterion) {
    let mut group = c.benchmark_group("real_exact_exp_log10");
    let exp_ln_1000 = Real::new(Rational::new(1000)).ln().unwrap();
    let exp_ln_eighth = Real::new(Rational::fraction(1, 8).unwrap()).ln().unwrap();
    let log10_1000 = Real::new(Rational::new(1000));
    let log10_milli = Real::new(Rational::fraction(1, 1000).unwrap());

    group.bench_function("exp_ln_1000", |b| {
        b.iter_batched(
            || exp_ln_1000.clone(),
            |value| black_box(value.exp().unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("exp_ln_1_8", |b| {
        b.iter_batched(
            || exp_ln_eighth.clone(),
            |value| black_box(value.exp().unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("log10_1000", |b| {
        b.iter_batched(
            || log10_1000.clone(),
            |value| black_box(value.log10().unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("log10_1_1000", |b| {
        b.iter_batched(
            || log10_milli.clone(),
            |value| black_box(value.log10().unwrap()),
            BatchSize::SmallInput,
        )
    });

    group.finish();
}

criterion_group!(
    benches,
    bench_real_format,
    bench_real_constants,
    bench_simple,
    bench_real_powi,
    bench_rational_powi,
    bench_real_exact_trig,
    bench_real_general_trig,
    bench_real_exact_inverse_trig,
    bench_real_general_inverse_trig,
    bench_real_inverse_hyperbolic,
    bench_simple_inverse_functions,
    bench_simple_inverse_error_functions,
    bench_real_exact_ln,
    bench_real_exact_exp_log10
);
criterion_main!(benches);
