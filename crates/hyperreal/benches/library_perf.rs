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
    BenchGroupDoc {
        name: "real_stable_scalar_substrate",
        description: "Stable scalar constructors that preserve small residuals, dominance, roots, rational powers, and certified integer decisions.",
        benches: &[
            BenchDoc {
                name: "ln_1p_tiny",
                description: "Builds ln(1 + tiny) without first adding one generically.",
            },
            BenchDoc {
                name: "ln_1m_tiny",
                description: "Builds ln(1 - tiny) through the log1p companion path.",
            },
            BenchDoc {
                name: "expm1_tiny",
                description: "Builds exp(tiny) - 1 through the dedicated expm1 node.",
            },
            BenchDoc {
                name: "softplus_large_positive",
                description: "Builds softplus for a dominant positive input.",
            },
            BenchDoc {
                name: "softplus_large_negative",
                description: "Builds softplus for a dominant negative input.",
            },
            BenchDoc {
                name: "logaddexp_dominant",
                description: "Builds logaddexp when one side is certifiably dominant.",
            },
            BenchDoc {
                name: "logsubexp_near",
                description: "Builds logsubexp for a certifiably positive but small log-space difference.",
            },
            BenchDoc {
                name: "sigmoid_large_positive",
                description: "Builds a large positive sigmoid through the stable tail path.",
            },
            BenchDoc {
                name: "logit_near_one",
                description: "Builds logit close to the upper probability boundary.",
            },
            BenchDoc {
                name: "sqrt1pm1_tiny",
                description: "Builds sqrt(1 + tiny) - 1 through the stable helper.",
            },
            BenchDoc {
                name: "sqrt1m1_tiny",
                description: "Builds sqrt(1 - tiny) - 1 through the stable helper.",
            },
            BenchDoc {
                name: "cbrt_negative_perfect",
                description: "Collapses a negative perfect cube.",
            },
            BenchDoc {
                name: "root_n_perfect_fourth",
                description: "Collapses an exact fourth root.",
            },
            BenchDoc {
                name: "pow_rational_negative_odd_denominator",
                description: "Routes a negative rational base through odd-root symmetry.",
            },
            BenchDoc {
                name: "floor_certified_rational",
                description: "Certifies rational floor structurally.",
            },
            BenchDoc {
                name: "rem_euclid_certified_rational",
                description: "Computes rational Euclidean remainder through certified quotient floor.",
            },
        ],
    },
    BenchGroupDoc {
        name: "real_geometry_polynomial_substrate",
        description: "Geometry-facing scalar helpers for rational-turn trig, removable small-angle limits, vectors, product sums, and polynomial forms.",
        benches: &[
            BenchDoc {
                name: "sin_pi_one_sixth",
                description: "Uses exact rational-turn sine.",
            },
            BenchDoc {
                name: "cos_pi_one_fourth",
                description: "Uses exact rational-turn cosine.",
            },
            BenchDoc {
                name: "tan_pi_one_third",
                description: "Uses exact rational-turn tangent.",
            },
            BenchDoc {
                name: "sinc_zero",
                description: "Returns the removable sinc limit at zero.",
            },
            BenchDoc {
                name: "sinc_tiny",
                description: "Builds sinc for a tiny exact input.",
            },
            BenchDoc {
                name: "sinc_pi_half",
                description: "Builds normalized sinc for an exact half turn.",
            },
            BenchDoc {
                name: "cosc_tiny",
                description: "Builds the small-angle (1 - cos x) / x^2 helper.",
            },
            BenchDoc {
                name: "atan2_axis",
                description: "Classifies an axis-aligned atan2 input exactly.",
            },
            BenchDoc {
                name: "atan2_quadrant",
                description: "Builds a quadrant-correct atan2 expression.",
            },
            BenchDoc {
                name: "hypot2_3_4",
                description: "Collapses a 3-4-5 norm through exact dot products.",
            },
            BenchDoc {
                name: "hypot3_2_3_6",
                description: "Collapses a 2-3-6 norm through exact dot products.",
            },
            BenchDoc {
                name: "hypot_minus_tiny",
                description: "Uses rationalized hypot-minus for cancellation resistance.",
            },
            BenchDoc {
                name: "mul_add_zero_product",
                description: "Skips a known-zero product lane.",
            },
            BenchDoc {
                name: "sum_products_dense",
                description: "Builds a dense product sum.",
            },
            BenchDoc {
                name: "diff_of_products_near_cancel",
                description: "Preserves determinant-like product difference structure.",
            },
            BenchDoc {
                name: "eval_poly_horner",
                description: "Evaluates a polynomial through Horner form.",
            },
            BenchDoc {
                name: "eval_rational_poly",
                description: "Evaluates numerator and denominator polynomial forms before division.",
            },
        ],
    },
    BenchGroupDoc {
        name: "real_normal_scientific_substrate",
        description: "Gaussian tail helpers and exact/finite scientific special-function forms added for higher numerical workloads.",
        benches: &[
            BenchDoc {
                name: "erfc_zero",
                description: "Takes the exact erfc(0) exit.",
            },
            BenchDoc {
                name: "erfcx_tail",
                description: "Builds scaled erfc in a positive tail.",
            },
            BenchDoc {
                name: "normal_sf_tail",
                description: "Builds standard-normal upper-tail probability.",
            },
            BenchDoc {
                name: "pnorm_upper_tail",
                description: "Builds the upper-tail alias.",
            },
            BenchDoc {
                name: "log_pnorm_tail",
                description: "Builds lower log-CDF tail form.",
            },
            BenchDoc {
                name: "log_normal_sf_tail",
                description: "Builds upper log-survival tail form.",
            },
            BenchDoc {
                name: "log_dnorm_large",
                description: "Builds analytic log-density at a large input.",
            },
            BenchDoc {
                name: "normal_interval_narrow",
                description: "Builds a narrow interval mass without spelling pnorm subtraction.",
            },
            BenchDoc {
                name: "erfinv_mid",
                description: "Builds inverse error function through qnorm transform.",
            },
            BenchDoc {
                name: "erfcinv_tail",
                description: "Builds inverse complementary error function through tail qnorm transform.",
            },
            BenchDoc {
                name: "qnorm_upper_tail",
                description: "Builds inverse survival quantile.",
            },
            BenchDoc {
                name: "normal_pdf_parametric",
                description: "Standardizes exactly before density construction.",
            },
            BenchDoc {
                name: "normal_survival_parametric",
                description: "Standardizes exactly before upper-tail construction.",
            },
            BenchDoc {
                name: "normal_mills_tail",
                description: "Builds Mills ratio through erfcx identity.",
            },
            BenchDoc {
                name: "normal_hazard_tail",
                description: "Builds reciprocal Mills hazard.",
            },
            BenchDoc {
                name: "hermite_8",
                description: "Builds an exact probabilists' Hermite polynomial.",
            },
            BenchDoc {
                name: "dnorm_derivative_4",
                description: "Combines exact Hermite polynomial with normal density.",
            },
            BenchDoc {
                name: "standard_normal_moment_12",
                description: "Uses double-factorial closed form.",
            },
            BenchDoc {
                name: "normal_interval_moment_3",
                description: "Uses interval mass and density-boundary recurrence.",
            },
            BenchDoc {
                name: "truncated_normal_mean",
                description: "Builds truncated-normal mean from stable interval mass.",
            },
            BenchDoc {
                name: "gamma_integer",
                description: "Uses exact integer gamma closed form.",
            },
            BenchDoc {
                name: "gamma_half_integer",
                description: "Uses exact half-integer gamma closed form.",
            },
            BenchDoc {
                name: "lgamma_half_integer",
                description: "Logs the absolute half-integer gamma value.",
            },
            BenchDoc {
                name: "beta_integer",
                description: "Builds beta through exact gamma ratio.",
            },
            BenchDoc {
                name: "ln_beta_half_integer",
                description: "Builds log beta through lgamma sum.",
            },
            BenchDoc {
                name: "regularized_beta_mid",
                description: "Uses finite positive-integer beta binomial tail.",
            },
            BenchDoc {
                name: "regularized_beta_q_mid",
                description: "Uses finite positive-integer beta upper-tail form.",
            },
            BenchDoc {
                name: "regularized_gamma_p_half",
                description: "Uses half-integer incomplete-gamma recurrence.",
            },
            BenchDoc {
                name: "regularized_gamma_q_integer",
                description: "Uses integer incomplete-gamma recurrence.",
            },
            BenchDoc {
                name: "chi_square_sf",
                description: "Wraps regularized upper gamma for chi-square upper tail.",
            },
        ],
    },
    BenchGroupDoc {
        name: "simple_new_function_surface",
        description: "Parser and evaluator coverage for the newly exposed stable scalar, geometry, normal, and scientific functions.",
        benches: &[
            BenchDoc {
                name: "stable_log_exp_bundle",
                description: "Evaluates log1p/log1m/expm1/softplus/logaddexp/logsubexp/sigmoid/logit together.",
            },
            BenchDoc {
                name: "geometry_bundle",
                description: "Evaluates rational-turn trig, small-angle helpers, vector norms, product sums, and polynomials together.",
            },
            BenchDoc {
                name: "normal_bundle",
                description: "Evaluates normal tails, log tails, interval mass, inverse tails, and moments together.",
            },
            BenchDoc {
                name: "scientific_bundle",
                description: "Evaluates gamma, beta, regularized gamma/beta, and chi-square forms together.",
            },
            BenchDoc {
                name: "error_bundle",
                description: "Exercises fast domain failures for new public functions.",
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

fn bench_real_stable_scalar_substrate(c: &mut Criterion) {
    let mut group = c.benchmark_group("real_stable_scalar_substrate");
    let tiny = real(1, 1_000_000_000_000);
    let near_one = real(999_999, 1_000_000);
    let large = Real::from(64_i32);
    let negative_large = Real::from(-64_i32);
    let root_base = Real::from(-27_i32);
    let perfect_fourth = Real::from(81_i32);
    let negative_pow = Real::from(-8_i32);
    let odd_third = rational(1, 3);
    let rational_floor = real(7, 3);
    let rational_rem = real(-17, 5);
    let modulus = Real::from(3_i32);

    group.bench_function("ln_1p_tiny", |b| {
        b.iter_batched(
            || tiny.clone(),
            |value| black_box(value.ln_1p().unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("ln_1m_tiny", |b| {
        b.iter_batched(
            || tiny.clone(),
            |value| black_box(value.ln_1m().unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("expm1_tiny", |b| {
        b.iter_batched(
            || tiny.clone(),
            |value| black_box(value.expm1()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("softplus_large_positive", |b| {
        b.iter_batched(
            || large.clone(),
            |value| black_box(value.softplus().unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("softplus_large_negative", |b| {
        b.iter_batched(
            || negative_large.clone(),
            |value| black_box(value.softplus().unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("logaddexp_dominant", |b| {
        b.iter_batched(
            || (large.clone(), tiny.clone()),
            |(a, b)| black_box(Real::logaddexp(&a, &b).unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("logsubexp_near", |b| {
        b.iter_batched(
            || (Real::one(), near_one.clone()),
            |(a, b)| black_box(Real::logsubexp(&a, &b).unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("sigmoid_large_positive", |b| {
        b.iter_batched(
            || large.clone(),
            |value| black_box(value.sigmoid().unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("logit_near_one", |b| {
        b.iter_batched(
            || near_one.clone(),
            |value| black_box(value.logit().unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("sqrt1pm1_tiny", |b| {
        b.iter_batched(
            || tiny.clone(),
            |value| black_box(value.sqrt1pm1().unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("sqrt1m1_tiny", |b| {
        b.iter_batched(
            || tiny.clone(),
            |value| black_box(value.sqrt1m1().unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("cbrt_negative_perfect", |b| {
        b.iter_batched(
            || root_base.clone(),
            |value| black_box(value.cbrt().unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("root_n_perfect_fourth", |b| {
        b.iter_batched(
            || perfect_fourth.clone(),
            |value| black_box(value.root_n(4).unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("pow_rational_negative_odd_denominator", |b| {
        b.iter_batched(
            || (negative_pow.clone(), odd_third.clone()),
            |(value, exponent)| black_box(value.pow_rational(exponent).unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("floor_certified_rational", |b| {
        b.iter_batched(
            || rational_floor.clone(),
            |value| black_box(value.floor_certified().unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("rem_euclid_certified_rational", |b| {
        b.iter_batched(
            || (rational_rem.clone(), modulus.clone()),
            |(value, modulus)| black_box(value.rem_euclid_certified(&modulus).unwrap()),
            BatchSize::SmallInput,
        )
    });

    group.finish();
}

fn bench_real_geometry_polynomial_substrate(c: &mut Criterion) {
    let mut group = c.benchmark_group("real_geometry_polynomial_substrate");
    let tiny = real(1, 1_000_000_000_000);
    let half = real(1, 2);
    let one_sixth = real(1, 6);
    let one_fourth = real(1, 4);
    let one_third = real(1, 3);
    let zero = Real::zero();
    let one = Real::one();
    let minus_one = -Real::one();
    let three = Real::from(3_i32);
    let four = Real::from(4_i32);
    let six = Real::from(6_i32);
    let coeffs = vec![
        Real::from(5_i32),
        real(-7, 3),
        Real::pi(),
        Real::from(11_i32),
        real(13, 17),
    ];
    let den_coeffs = vec![Real::one(), real(1, 5), real(1, 7)];
    let dense_left = vec![Real::pi(), Real::e(), Real::from(7_i32), Real::from(11_i32)];
    let dense_right = vec![Real::from(2_i32), Real::pi(), Real::e(), Real::from(13_i32)];

    group.bench_function("sin_pi_one_sixth", |b| {
        b.iter_batched(
            || one_sixth.clone(),
            |value| black_box(value.sin_pi()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("cos_pi_one_fourth", |b| {
        b.iter_batched(
            || one_fourth.clone(),
            |value| black_box(value.cos_pi()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("tan_pi_one_third", |b| {
        b.iter_batched(
            || one_third.clone(),
            |value| black_box(value.tan_pi().unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("sinc_zero", |b| {
        b.iter_batched(
            Real::zero,
            |value| black_box(value.sinc().unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("sinc_tiny", |b| {
        b.iter_batched(
            || tiny.clone(),
            |value| black_box(value.sinc().unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("sinc_pi_half", |b| {
        b.iter_batched(
            || half.clone(),
            |value| black_box(value.sinc_pi().unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("cosc_tiny", |b| {
        b.iter_batched(
            || tiny.clone(),
            |value| black_box(value.cosc().unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("atan2_axis", |b| {
        b.iter_batched(
            || (zero.clone(), minus_one.clone()),
            |(y, x)| black_box(y.atan2(x)),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("atan2_quadrant", |b| {
        b.iter_batched(
            || (one.clone(), minus_one.clone()),
            |(y, x)| black_box(y.atan2(x)),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("hypot2_3_4", |b| {
        b.iter_batched(
            || (three.clone(), four.clone()),
            |(x, y)| black_box(Real::hypot2(&x, &y).unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("hypot3_2_3_6", |b| {
        b.iter_batched(
            || (Real::from(2_i32), three.clone(), six.clone()),
            |(x, y, z)| black_box(Real::hypot3(&x, &y, &z).unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("hypot_minus_tiny", |b| {
        b.iter_batched(
            || (Real::from(1_000_000_i32), tiny.clone()),
            |(x, y)| black_box(Real::hypot_minus(&x, &y).unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("mul_add_zero_product", |b| {
        b.iter_batched(
            || (Real::zero(), Real::pi(), Real::e()),
            |(a, b, c)| black_box(Real::mul_add(&a, &b, &c)),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("sum_products_dense", |b| {
        b.iter_batched(
            || (dense_left.clone(), dense_right.clone()),
            |(left, right)| black_box(Real::sum_products(&left, &right).unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("diff_of_products_near_cancel", |b| {
        b.iter(|| {
            black_box(Real::diff_of_products(
                &real(10_000_001, 10_000_000),
                &real(9_999_999, 10_000_000),
                &Real::one(),
                &Real::one(),
            ))
        })
    });
    group.bench_function("eval_poly_horner", |b| {
        b.iter_batched(
            || (coeffs.clone(), half.clone()),
            |(coeffs, x)| black_box(Real::eval_poly(&coeffs, &x)),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("eval_rational_poly", |b| {
        b.iter_batched(
            || (coeffs.clone(), den_coeffs.clone(), half.clone()),
            |(num, den, x)| black_box(Real::eval_rational_poly(&num, &den, &x).unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.finish();
}

fn bench_real_normal_scientific_substrate(c: &mut Criterion) {
    let mut group = c.benchmark_group("real_normal_scientific_substrate");
    let zero = Real::zero();
    let half = real(1, 2);
    let one = Real::one();
    let two = Real::from(2_i32);
    let three = Real::from(3_i32);
    let tail = Real::from(6_i32);
    let lo = real(1, 10);
    let hi = real(100_000_001, 1_000_000_000);
    let mean = Real::from(2_i32);
    let sigma = real(3, 2);
    let gamma_half = real(1, 2);
    let gamma_three_half = real(3, 2);

    group.bench_function("erfc_zero", |b| {
        b.iter_batched(
            Real::zero,
            |value| black_box(value.erfc()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("erfcx_tail", |b| {
        b.iter_batched(
            || tail.clone(),
            |value| black_box(value.erfcx().unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("normal_sf_tail", |b| {
        b.iter_batched(
            || tail.clone(),
            |value| black_box(value.normal_sf().unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("pnorm_upper_tail", |b| {
        b.iter_batched(
            || tail.clone(),
            |value| black_box(value.pnorm_upper().unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("log_pnorm_tail", |b| {
        b.iter_batched(
            || -tail.clone(),
            |value| black_box(value.log_pnorm().unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("log_normal_sf_tail", |b| {
        b.iter_batched(
            || tail.clone(),
            |value| black_box(value.log_normal_sf().unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("log_dnorm_large", |b| {
        b.iter_batched(
            || tail.clone(),
            |value| black_box(value.log_dnorm().unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("normal_interval_narrow", |b| {
        b.iter_batched(
            || (lo.clone(), hi.clone()),
            |(lo, hi)| black_box(Real::normal_interval(&lo, &hi).unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("erfinv_mid", |b| {
        b.iter_batched(
            || half.clone(),
            |value| black_box(value.erfinv().unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("erfcinv_tail", |b| {
        b.iter_batched(
            || real(1, 1_000_000),
            |value| black_box(value.erfcinv().unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("qnorm_upper_tail", |b| {
        b.iter_batched(
            || real(1, 1_000_000),
            |value| black_box(value.qnorm_upper().unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("normal_pdf_parametric", |b| {
        b.iter_batched(
            || (tail.clone(), mean.clone(), sigma.clone()),
            |(x, mean, sigma)| black_box(x.normal_pdf(&mean, &sigma).unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("normal_survival_parametric", |b| {
        b.iter_batched(
            || (tail.clone(), mean.clone(), sigma.clone()),
            |(x, mean, sigma)| black_box(x.normal_survival(&mean, &sigma).unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("normal_mills_tail", |b| {
        b.iter_batched(
            || tail.clone(),
            |value| black_box(value.normal_mills().unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("normal_hazard_tail", |b| {
        b.iter_batched(
            || tail.clone(),
            |value| black_box(value.normal_hazard().unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("hermite_8", |b| {
        b.iter_batched(
            || two.clone(),
            |value| black_box(Real::hermite_probabilists(8, &value)),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("dnorm_derivative_4", |b| {
        b.iter_batched(
            || two.clone(),
            |value| black_box(value.dnorm_derivative(4).unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("standard_normal_moment_12", |b| {
        b.iter(|| black_box(Real::standard_normal_moment(12)))
    });
    group.bench_function("normal_interval_moment_3", |b| {
        b.iter_batched(
            || (zero.clone(), one.clone()),
            |(lo, hi)| black_box(Real::normal_interval_moment(&lo, &hi, 3).unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("truncated_normal_mean", |b| {
        b.iter_batched(
            || (zero.clone(), one.clone()),
            |(lo, hi)| black_box(Real::truncated_normal_mean(&lo, &hi).unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("gamma_integer", |b| {
        b.iter_batched(
            || Real::from(8_i32),
            |value| black_box(value.gamma().unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("gamma_half_integer", |b| {
        b.iter_batched(
            || gamma_half.clone(),
            |value| black_box(value.gamma().unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("lgamma_half_integer", |b| {
        b.iter_batched(
            || gamma_three_half.clone(),
            |value| black_box(value.lgamma().unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("beta_integer", |b| {
        b.iter_batched(
            || (two.clone(), three.clone()),
            |(a, b)| black_box(Real::beta(&a, &b).unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("ln_beta_half_integer", |b| {
        b.iter_batched(
            || (gamma_half.clone(), gamma_three_half.clone()),
            |(a, b)| black_box(Real::ln_beta(&a, &b).unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("regularized_beta_mid", |b| {
        b.iter_batched(
            || (two.clone(), three.clone(), half.clone()),
            |(a, b, x)| black_box(Real::regularized_beta(&a, &b, &x).unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("regularized_beta_q_mid", |b| {
        b.iter_batched(
            || (two.clone(), three.clone(), half.clone()),
            |(a, b, x)| black_box(Real::regularized_beta_q(&a, &b, &x).unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("regularized_gamma_p_half", |b| {
        b.iter_batched(
            || (gamma_three_half.clone(), one.clone()),
            |(a, x)| black_box(Real::regularized_gamma_p(&a, &x).unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("regularized_gamma_q_integer", |b| {
        b.iter_batched(
            || (three.clone(), one.clone()),
            |(a, x)| black_box(Real::regularized_gamma_q(&a, &x).unwrap()),
            BatchSize::SmallInput,
        )
    });
    group.bench_function("chi_square_sf", |b| {
        b.iter_batched(
            || two.clone(),
            |x| black_box(Real::chi_square_sf(&x, 5).unwrap()),
            BatchSize::SmallInput,
        )
    });

    group.finish();
}

fn bench_simple_new_function_surface(c: &mut Criterion) {
    let mut group = c.benchmark_group("simple_new_function_surface");
    let expressions: [(&str, Simple); 5] = [
        (
            "stable_log_exp_bundle",
            "(+ (ln_1p 1/1000000000000) (ln_1m 1/1000000000000) (expm1 1/1000000000000) (softplus 64) (logaddexp 64 0) (logsubexp 1 999999/1000000) (sigmoid 64) (logit 999999/1000000))"
                .parse()
                .unwrap(),
        ),
        (
            "geometry_bundle",
            "(+ (sin_pi 1/6) (cos_pi 1/4) (tan_pi 1/3) (sinc 1/1000000000000) (sinc_pi 1/2) (cosc 1/1000000000000) (hypot2 3 4) (hypot3 2 3 6) (hypot_minus 1000000 1/1000000) (mul_add 0 pi e) (sum_products pi 2 e pi 7 e) (diff_of_products 10000001/10000000 9999999/10000000 1 1) (eval_poly 1/2 5 -7/3 pi 11) (eval_rational_poly 1/2 3 5 -7/3 pi 1 1/5 1/7))"
                .parse()
                .unwrap(),
        ),
        (
            "normal_bundle",
            "(+ (erfc 0) (erfcx 6) (normal_sf 6) (pnorm_upper 6) (log_pnorm -6) (log_normal_sf 6) (log_dnorm 6) (normal_interval 1/10 100000001/1000000000) (erfinv 1/2) (erfcinv 1/1000000) (qnorm_upper 1/1000000) (normal_pdf 6 2 3/2) (normal_survival 6 2 3/2) (normal_mills 6) (normal_hazard 6) (hermite_probabilists 8 2) (dnorm_derivative 4 2) (standard_normal_moment 12) (normal_interval_moment 0 1 3) (truncated_normal_mean 0 1))"
                .parse()
                .unwrap(),
        ),
        (
            "scientific_bundle",
            "(+ (gamma 8) (gamma 1/2) (lgamma 3/2) (beta 2 3) (ln_beta 1/2 3/2) (regularized_beta 2 3 1/2) (regularized_beta_q 2 3 1/2) (regularized_gamma_p 3/2 1) (regularized_gamma_q 3 1) (chi_square_sf 2 5))"
                .parse()
                .unwrap(),
        ),
        (
            "error_bundle",
            "(+ (ln_1p -1) (logsubexp 1 2) (root_n -16 2) (normal_sf 11) (gamma 0) (regularized_beta 0 1 1/2) (regularized_gamma_p 1 -1))"
                .parse()
                .unwrap(),
        ),
    ];

    for (name, expr) in expressions {
        if name == "error_bundle" {
            group.bench_function(name, |b| {
                b.iter_batched(
                    || expr.clone(),
                    |expr| black_box(expr.evaluate(&Default::default()).unwrap_err()),
                    BatchSize::SmallInput,
                )
            });
        } else {
            group.bench_function(name, |b| {
                b.iter_batched(
                    || expr.clone(),
                    |expr| black_box(expr.evaluate(&Default::default()).unwrap()),
                    BatchSize::SmallInput,
                )
            });
        }
    }

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
    bench_real_exact_exp_log10,
    bench_real_stable_scalar_substrate,
    bench_real_geometry_polynomial_substrate,
    bench_real_normal_scientific_substrate,
    bench_simple_new_function_surface
);
criterion_main!(benches);
