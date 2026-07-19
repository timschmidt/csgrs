//! Public numeric API comparisons against Rug's GMP/MPFR backend.
//!
//! Exact rational rows use GMP-backed `rug::Rational`. Real and computable
//! rows use MPFR at 128-bit precision. Hyperreal-only structural certificate,
//! cache-introspection, abort, and prepared-filter APIs have no GMP analogue
//! and are intentionally covered by the existing native benchmark suites.

use criterion::{
    BenchmarkGroup, BenchmarkId, Criterion, black_box, criterion_group, criterion_main,
};
use hyperreal::{Computable, Rational, Real};
use num::bigint::{BigInt, BigUint};
use rug::{Float, Integer, Rational as GmpRational, float::Constant, integer::Order, ops::Pow};

const GMP_PRECISION: u32 = 128;

fn real(value: f64) -> Real {
    Real::try_from(value).expect("finite benchmark value")
}

fn gmp(value: f64) -> Float {
    Float::with_val(GMP_PRECISION, value)
}

fn rational(numerator: i64, denominator: u64) -> Rational {
    Rational::fraction(numerator, denominator).expect("nonzero benchmark denominator")
}

fn gmp_rational(numerator: i64, denominator: u64) -> GmpRational {
    GmpRational::from((numerator, denominator))
}

fn bench_real_unary<H, G>(
    group: &mut BenchmarkGroup<'_, criterion::measurement::WallTime>,
    name: &str,
    input: f64,
    hyperreal: H,
    gmp_mpfr: G,
) where
    H: Fn(Real) -> Real,
    G: Fn(Float) -> Float,
{
    let hyperreal_input = real(input);
    group.bench_function(BenchmarkId::new("hyperreal", name), |b| {
        b.iter(|| black_box(hyperreal(black_box(hyperreal_input.clone()))))
    });

    let gmp_input = gmp(input);
    group.bench_function(BenchmarkId::new("gmp_mpfr128", name), |b| {
        b.iter(|| black_box(gmp_mpfr(black_box(gmp_input.clone()))))
    });
}

fn bench_real_binary<H, G>(
    group: &mut BenchmarkGroup<'_, criterion::measurement::WallTime>,
    name: &str,
    lhs: f64,
    rhs: f64,
    hyperreal: H,
    gmp_mpfr: G,
) where
    H: Fn(Real, Real) -> Real,
    G: Fn(Float, Float) -> Float,
{
    let hyperreal_lhs = real(lhs);
    let hyperreal_rhs = real(rhs);
    group.bench_function(BenchmarkId::new("hyperreal", name), |b| {
        b.iter(|| {
            black_box(hyperreal(
                black_box(hyperreal_lhs.clone()),
                black_box(hyperreal_rhs.clone()),
            ))
        })
    });

    let gmp_lhs = gmp(lhs);
    let gmp_rhs = gmp(rhs);
    group.bench_function(BenchmarkId::new("gmp_mpfr128", name), |b| {
        b.iter(|| {
            black_box(gmp_mpfr(
                black_box(gmp_lhs.clone()),
                black_box(gmp_rhs.clone()),
            ))
        })
    });
}

fn bench_real_ternary<H, G>(
    group: &mut BenchmarkGroup<'_, criterion::measurement::WallTime>,
    name: &str,
    inputs: [f64; 3],
    hyperreal: H,
    gmp_mpfr: G,
) where
    H: Fn(Real, Real, Real) -> Real,
    G: Fn(Float, Float, Float) -> Float,
{
    let hyperreal_inputs = inputs.map(real);
    group.bench_function(BenchmarkId::new("hyperreal", name), |b| {
        b.iter(|| {
            black_box(hyperreal(
                black_box(hyperreal_inputs[0].clone()),
                black_box(hyperreal_inputs[1].clone()),
                black_box(hyperreal_inputs[2].clone()),
            ))
        })
    });

    let gmp_inputs = inputs.map(gmp);
    group.bench_function(BenchmarkId::new("gmp_mpfr128", name), |b| {
        b.iter(|| {
            black_box(gmp_mpfr(
                black_box(gmp_inputs[0].clone()),
                black_box(gmp_inputs[1].clone()),
                black_box(gmp_inputs[2].clone()),
            ))
        })
    });
}

fn gmp_sqrt_two() -> Float {
    Float::with_val(GMP_PRECISION, 2).sqrt()
}

fn gmp_sqrt_two_pi() -> Float {
    let mut two_pi = Float::with_val(GMP_PRECISION, Constant::Pi);
    two_pi *= 2;
    two_pi.sqrt()
}

fn gmp_standard_normal_pdf(x: Float) -> Float {
    let mut exponent = x.clone() * &x;
    exponent /= -2;
    exponent.exp() / gmp_sqrt_two_pi()
}

fn gmp_standard_normal_cdf(x: Float) -> Float {
    let scaled = -x / gmp_sqrt_two();
    scaled.erfc() / 2
}

fn gmp_standard_normal_sf(x: Float) -> Float {
    let scaled = x / gmp_sqrt_two();
    scaled.erfc() / 2
}

fn gmp_regularized_gamma_q(a: Float, x: Float) -> Float {
    a.clone().gamma_inc(&x) / a.gamma()
}

fn gmp_regularized_gamma_p(a: Float, x: Float) -> Float {
    Float::with_val(GMP_PRECISION, 1) - gmp_regularized_gamma_q(a, x)
}

fn gmp_sum(values: &[Float]) -> Float {
    values.iter().fold(gmp(0.0), |sum, value| sum + value)
}

fn gmp_dot<const N: usize>(left: &[Float; N], right: &[Float; N]) -> Float {
    (0..N).fold(gmp(0.0), |sum, index| {
        sum + left[index].clone() * &right[index]
    })
}

fn gmp_erfinv(target: Float) -> Float {
    let mut estimate = target.clone();
    let derivative_scale = gmp(2.0) / Float::with_val(GMP_PRECISION, Constant::Pi).sqrt();
    for _ in 0..8 {
        let error = estimate.clone().erf() - &target;
        let exponent: Float = -(estimate.clone() * &estimate);
        let derivative = derivative_scale.clone() * exponent.exp();
        estimate -= error / derivative;
    }
    estimate
}

fn gmp_standard_normal_quantile(probability: Float) -> Float {
    let erf_argument = probability * 2 - 1;
    gmp_sqrt_two() * gmp_erfinv(erf_argument)
}

fn gmp_hermite_probabilists(n: usize, x: &Float) -> Float {
    if n == 0 {
        return gmp(1.0);
    }
    let mut previous = gmp(1.0);
    let mut current = x.clone();
    for k in 1..n {
        let next = x.clone() * &current - previous * k;
        previous = current;
        current = next;
    }
    current
}

fn gmp_normal_interval_moment(lo: &Float, hi: &Float, n: usize) -> Float {
    let mass = gmp_standard_normal_cdf(hi.clone()) - gmp_standard_normal_cdf(lo.clone());
    if n == 0 {
        return mass;
    }
    let phi_lo = gmp_standard_normal_pdf(lo.clone());
    let phi_hi = gmp_standard_normal_pdf(hi.clone());
    let first = phi_lo.clone() - &phi_hi;
    if n == 1 {
        return first;
    }

    let mut two_back = mass;
    let mut one_back = first;
    for degree in 2..=n {
        let boundary = lo.clone().pow((degree - 1) as u32) * &phi_lo
            - hi.clone().pow((degree - 1) as u32) * &phi_hi;
        let current = boundary + two_back * (degree - 1);
        two_back = one_back;
        one_back = current;
    }
    one_back
}

fn binomial(n: u32, k: u32) -> u64 {
    let k = k.min(n - k);
    (0..k).fold(1_u64, |value, index| {
        value * u64::from(n - index) / u64::from(index + 1)
    })
}

fn gmp_regularized_beta_integer(a: u32, b: u32, x: Float) -> Float {
    let n = a + b - 1;
    let one_minus_x = gmp(1.0) - &x;
    (a..=n).fold(gmp(0.0), |sum, k| {
        sum + x.clone().pow(k) * one_minus_x.clone().pow(n - k) * binomial(n, k)
    })
}

fn bench_rational_api(c: &mut Criterion) {
    let mut group = c.benchmark_group("gmp_rational_api");
    let lhs = rational(355, 113);
    let rhs = rational(-22, 7);
    let gmp_lhs = gmp_rational(355, 113);
    let gmp_rhs = gmp_rational(-22, 7);

    macro_rules! pair {
        ($name:literal, $hyperreal:expr, $gmp:expr) => {
            group.bench_function(BenchmarkId::new("hyperreal", $name), |b| {
                b.iter(|| black_box($hyperreal))
            });
            group.bench_function(BenchmarkId::new("gmp", $name), |b| {
                b.iter(|| black_box($gmp))
            });
        };
    }

    pair!("zero", Rational::zero(), GmpRational::new());
    pair!("one", Rational::one(), GmpRational::from(1));
    pair!("from_integer", Rational::new(17), GmpRational::from(17));
    pair!("from_fraction", rational(355, 113), gmp_rational(355, 113));
    pair!("add", &lhs + &rhs, &gmp_lhs + &gmp_rhs);
    pair!("sub", &lhs - &rhs, &gmp_lhs - &gmp_rhs);
    pair!("mul", &lhs * &rhs, &gmp_lhs * &gmp_rhs);
    pair!("div", &lhs / &rhs, &gmp_lhs / &gmp_rhs);
    pair!("neg", -&lhs, -&gmp_lhs);
    pair!(
        "inverse",
        lhs.clone().inverse().unwrap(),
        gmp_lhs.clone().recip()
    );
    pair!(
        "powi_17",
        lhs.clone().powi(BigInt::from(17)).unwrap(),
        gmp_lhs.clone().pow(17_u32)
    );
    pair!("trunc", lhs.trunc(), gmp_lhs.clone().trunc());
    pair!(
        "fract",
        lhs.fract(),
        gmp_lhs.clone() - gmp_lhs.clone().trunc()
    );
    pair!("is_integer", lhs.is_integer(), gmp_lhs.is_integer());
    pair!("is_zero", lhs.is_zero(), gmp_lhs == 0);
    pair!("is_one", lhs.is_one(), gmp_lhs == 1);
    pair!("is_negative", lhs.is_negative(), gmp_lhs < 0);
    pair!("is_positive", lhs.is_positive(), gmp_lhs > 0);
    pair!("sign", lhs.sign(), gmp_lhs.cmp0());
    pair!(
        "numerator",
        lhs.numerator().clone(),
        gmp_lhs.numer().clone()
    );
    pair!(
        "denominator",
        lhs.denominator().clone(),
        gmp_lhs.denom().clone()
    );
    pair!(
        "is_dyadic",
        lhs.is_dyadic(),
        gmp_lhs.denom().is_power_of_two()
    );
    pair!("to_f64", lhs.dyadic_to_f64_exact(), gmp_lhs.to_f64());
    pair!(
        "to_integer",
        lhs.to_big_integer(),
        gmp_lhs.is_integer().then(|| gmp_lhs.numer().clone())
    );
    pair!(
        "ordering",
        lhs.partial_cmp(&rhs),
        gmp_lhs.partial_cmp(&gmp_rhs)
    );
    pair!(
        "average_pair",
        Rational::average_pair(&lhs, &rhs),
        (gmp_lhs.clone() + &gmp_rhs) / 2
    );

    let h_left = [rational(2, 3), rational(5, 7)];
    let h_right = [rational(11, 13), rational(17, 19)];
    let g_left = [gmp_rational(2, 3), gmp_rational(5, 7)];
    let g_right = [gmp_rational(11, 13), gmp_rational(17, 19)];
    pair!(
        "dot2",
        Rational::signed_product_sum2(
            [true, true],
            [[&h_left[0], &h_right[0]], [&h_left[1], &h_right[1]]],
        ),
        GmpRational::from(&g_left[0] * &g_right[0]) + GmpRational::from(&g_left[1] * &g_right[1])
    );

    let h_complex_left = [rational(2, 3), rational(5, 7)];
    let h_complex_right = [rational(11, 13), rational(-17, 19)];
    let g_complex_left = [gmp_rational(2, 3), gmp_rational(5, 7)];
    let g_complex_right = [gmp_rational(11, 13), gmp_rational(-17, 19)];
    pair!(
        "complex_product",
        Rational::complex_product_components(
            [&h_complex_left[0], &h_complex_left[1]],
            [&h_complex_right[0], &h_complex_right[1]],
        ),
        (
            g_complex_left[0].clone() * &g_complex_right[0]
                - g_complex_left[1].clone() * &g_complex_right[1],
            g_complex_left[0].clone() * &g_complex_right[1]
                + g_complex_left[1].clone() * &g_complex_right[0],
        )
    );
    pair!(
        "complex_quotient",
        Rational::complex_quotient_components(
            [&h_complex_left[0], &h_complex_left[1]],
            [&h_complex_right[0], &h_complex_right[1]],
        )
        .unwrap(),
        {
            let denominator = g_complex_right[0].clone() * &g_complex_right[0]
                + g_complex_right[1].clone() * &g_complex_right[1];
            (
                (g_complex_left[0].clone() * &g_complex_right[0]
                    + g_complex_left[1].clone() * &g_complex_right[1])
                    / &denominator,
                (g_complex_left[1].clone() * &g_complex_right[0]
                    - g_complex_left[0].clone() * &g_complex_right[1])
                    / denominator,
            )
        }
    );

    let large: BigInt = BigInt::from(1_u64) << 192;
    let large_denominator: BigUint = BigUint::from(1_u64) << 127;
    pair!(
        "from_bigint",
        Rational::from_bigint(large.clone()),
        GmpRational::from(rug::Integer::from(1) << 192)
    );
    pair!(
        "from_bigint_fraction",
        Rational::from_bigint_fraction(large.clone(), large_denominator.clone()).unwrap(),
        GmpRational::from((rug::Integer::from(1) << 192, rug::Integer::from(1) << 127))
    );

    let exact_square = rational(81, 16);
    let gmp_exact_square = gmp_rational(81, 16);
    pair!("is_perfect_power", exact_square.is_perfect_power(), {
        gmp_exact_square.numer().is_perfect_power() && gmp_exact_square.denom().is_perfect_power()
    });
    pair!("perfect_nth_root", exact_square.perfect_nth_root(4), {
        let (numerator, numerator_remainder) = gmp_exact_square
            .numer()
            .clone()
            .root_rem(rug::Integer::new(), 4);
        let (denominator, denominator_remainder) = gmp_exact_square
            .denom()
            .clone()
            .root_rem(rug::Integer::new(), 4);
        (numerator_remainder == 0 && denominator_remainder == 0)
            .then(|| GmpRational::from((numerator, denominator)))
    });
    pair!(
        "extract_square_reduced",
        exact_square.clone().extract_square_reduced(),
        (
            gmp_exact_square
                .numer()
                .clone()
                .root_rem(rug::Integer::new(), 2),
            gmp_exact_square
                .denom()
                .clone()
                .root_rem(rug::Integer::new(), 2),
        )
    );
    pair!(
        "extract_square_will_succeed",
        exact_square.extract_square_will_succeed(),
        {
            let (_, numerator_remainder) = gmp_exact_square
                .numer()
                .clone()
                .root_rem(rug::Integer::new(), 2);
            let (_, denominator_remainder) = gmp_exact_square
                .denom()
                .clone()
                .root_rem(rug::Integer::new(), 2);
            numerator_remainder == 0 && denominator_remainder == 0
        }
    );
    pair!(
        "same_denominator",
        lhs.same_denominator(&rational(17, 113)),
        gmp_lhs.denom() == gmp_rational(17, 113).denom()
    );
    pair!(
        "shifted_big_integer",
        lhs.shifted_big_integer(12),
        (gmp_lhs.clone() << 12_u32).trunc().numer().clone()
    );

    let shared_h = [
        rational(2, 31),
        rational(3, 31),
        rational(5, 31),
        rational(7, 31),
    ];
    let shared_g = [
        gmp_rational(2, 31),
        gmp_rational(3, 31),
        gmp_rational(5, 31),
        gmp_rational(7, 31),
    ];
    pair!(
        "signed_product_sum",
        Rational::signed_product_sum(
            [true, false],
            [[&shared_h[0], &shared_h[1]], [&shared_h[2], &shared_h[3]]],
        ),
        shared_g[0].clone() * &shared_g[1] - shared_g[2].clone() * &shared_g[3]
    );
    pair!(
        "signed_product_sum_shared_denominator",
        Rational::signed_product_sum_shared_denominator(
            [true, false],
            [[&shared_h[0], &shared_h[1]], [&shared_h[2], &shared_h[3]]],
        ),
        Some(shared_g[0].clone() * &shared_g[1] - shared_g[2].clone() * &shared_g[3])
    );
    pair!(
        "signed_product_sum_ordering",
        Rational::signed_product_sum_ordering(
            [true, false],
            [[&shared_h[0], &shared_h[1]], [&shared_h[2], &shared_h[3]]],
        ),
        (shared_g[0].clone() * &shared_g[1] - shared_g[2].clone() * &shared_g[3]).cmp0()
    );
    let shared_h_refs = shared_h.iter().collect::<Vec<_>>();
    group.bench_function(BenchmarkId::new("hyperreal", "mean_refs"), |b| {
        b.iter(|| black_box(Rational::mean_refs(black_box(&shared_h_refs)).unwrap()))
    });
    group.bench_function(BenchmarkId::new("gmp", "mean_refs"), |b| {
        b.iter(|| {
            let sum = shared_g.iter().fold(GmpRational::new(), |mut sum, value| {
                sum += value;
                sum
            });
            black_box(sum / 4)
        })
    });

    group.finish();
}

fn bench_real_arithmetic_api(c: &mut Criterion) {
    let mut group = c.benchmark_group("gmp_real_arithmetic_api");

    bench_real_binary(&mut group, "add", 3.25, -1.125, |a, b| a + b, |a, b| a + b);
    bench_real_binary(&mut group, "sub", 3.25, -1.125, |a, b| a - b, |a, b| a - b);
    bench_real_binary(&mut group, "mul", 3.25, -1.125, |a, b| a * b, |a, b| a * b);
    bench_real_binary(
        &mut group,
        "div",
        3.25,
        -1.125,
        |a, b| (a / b).unwrap(),
        |a, b| a / b,
    );
    bench_real_binary(
        &mut group,
        "pow",
        2.5,
        1.25,
        |a, b| a.pow(b).unwrap(),
        |a, b| a.pow(&b),
    );
    bench_real_binary(
        &mut group,
        "rem_euclid",
        -7.5,
        2.0,
        |a, b| a.rem_euclid_certified(&b).unwrap(),
        |a, b| {
            let q = (a.clone() / &b).floor();
            a - q * b
        },
    );
    bench_real_unary(&mut group, "neg", 3.25, |x| -x, |x| -x);
    bench_real_unary(&mut group, "abs", -3.25, |x| x.abs(), |x| x.abs());
    bench_real_unary(
        &mut group,
        "inverse",
        3.25,
        |x| x.inverse().unwrap(),
        Float::recip,
    );
    bench_real_unary(
        &mut group,
        "powi_17",
        1.25,
        |x| x.powi_i64(17).unwrap(),
        |x| x.pow(17_u32),
    );
    bench_real_unary(
        &mut group,
        "floor",
        3.75,
        |x| Real::integer(x.floor_certified().unwrap()),
        Float::floor,
    );
    bench_real_unary(
        &mut group,
        "ceil",
        3.25,
        |x| Real::integer(x.ceil_certified().unwrap()),
        Float::ceil,
    );
    bench_real_unary(
        &mut group,
        "trunc",
        -3.75,
        |x| Real::integer(x.trunc_certified().unwrap()),
        Float::trunc,
    );
    bench_real_unary(
        &mut group,
        "round",
        3.75,
        |x| Real::integer(x.round_certified().unwrap()),
        Float::round,
    );
    bench_real_unary(
        &mut group,
        "fract",
        3.75,
        |x| x.fract_certified().unwrap(),
        Float::fract,
    );
    bench_real_unary(
        &mut group,
        "to_radians",
        45.0,
        |x| x.to_radians(),
        |x| x * Float::with_val(GMP_PRECISION, Constant::Pi) / 180,
    );
    bench_real_unary(
        &mut group,
        "to_degrees",
        0.75,
        |x| x.to_degrees(),
        |x| x * 180 / Float::with_val(GMP_PRECISION, Constant::Pi),
    );

    group.finish();
}

fn bench_real_elementary_api(c: &mut Criterion) {
    let mut group = c.benchmark_group("gmp_real_elementary_api");

    macro_rules! unary_result {
        ($name:literal, $input:expr, $hyper:ident, $gmp:ident) => {
            bench_real_unary(
                &mut group,
                $name,
                $input,
                |x| x.$hyper().unwrap(),
                Float::$gmp,
            );
        };
    }
    macro_rules! unary_value {
        ($name:literal, $input:expr, $hyper:ident, $gmp:ident) => {
            bench_real_unary(&mut group, $name, $input, Real::$hyper, Float::$gmp);
        };
    }

    unary_result!("sqrt", 2.0, sqrt, sqrt);
    unary_result!("cbrt", -2.0, cbrt, cbrt);
    bench_real_unary(
        &mut group,
        "root_n_5",
        17.0,
        |x| x.root_n(5).unwrap(),
        |x| x.root(5),
    );
    unary_result!("exp", 0.75, exp, exp);
    unary_result!("ln", 1.75, ln, ln);
    unary_result!("log2", 17.0, log2, log2);
    unary_result!("log10", 17.0, log10, log10);
    unary_result!("ln_1p", 1.0e-9, ln_1p, ln_1p);
    bench_real_unary(
        &mut group,
        "ln_1m",
        1.0e-9,
        |x| x.ln_1m().unwrap(),
        |x| (Float::with_val(GMP_PRECISION, 1) - x).ln(),
    );
    unary_result!("log1p", 1.0e-9, log1p, ln_1p);
    bench_real_unary(
        &mut group,
        "log1m",
        1.0e-9,
        |x| x.log1m().unwrap(),
        |x| (Float::with_val(GMP_PRECISION, 1) - x).ln(),
    );
    unary_value!("expm1", 1.0e-9, expm1, exp_m1);
    unary_value!("sin", 0.75, sin, sin);
    unary_value!("cos", 0.75, cos, cos);
    unary_result!("tan", 0.75, tan, tan);
    unary_result!("asin", 0.75, asin, asin);
    unary_result!("acos", 0.75, acos, acos);
    unary_result!("atan", 0.75, atan, atan);
    unary_result!("sinh", 0.75, sinh, sinh);
    unary_result!("cosh", 0.75, cosh, cosh);
    unary_result!("tanh", 0.75, tanh, tanh);
    unary_result!("asinh", 0.75, asinh, asinh);
    unary_result!("acosh", 1.75, acosh, acosh);
    unary_result!("atanh", 0.75, atanh, atanh);
    bench_real_unary(&mut group, "sin_pi", 0.75, Real::sin_pi, Float::sin_pi);
    bench_real_unary(&mut group, "cos_pi", 0.75, Real::cos_pi, Float::cos_pi);
    bench_real_unary(
        &mut group,
        "tan_pi",
        0.75,
        |x| x.tan_pi().unwrap(),
        Float::tan_pi,
    );
    bench_real_unary(
        &mut group,
        "sinc",
        0.75,
        |x| x.sinc().unwrap(),
        |x| x.clone().sin() / x,
    );
    bench_real_unary(
        &mut group,
        "sinc_pi",
        0.75,
        |x| x.sinc_pi().unwrap(),
        |x| x.clone().sin_pi() / x,
    );
    bench_real_unary(
        &mut group,
        "cosc",
        0.75,
        |x| x.cosc().unwrap(),
        |x| (Float::with_val(GMP_PRECISION, 1) - x.clone().cos()) / x,
    );
    unary_value!("erf", 0.75, erf, erf);
    unary_value!("erfc", 0.75, erfc, erfc);
    bench_real_unary(
        &mut group,
        "erfinv",
        0.5,
        |x| x.erfinv().unwrap(),
        gmp_erfinv,
    );
    bench_real_unary(
        &mut group,
        "erfcinv",
        0.5,
        |x| x.erfcinv().unwrap(),
        |x| gmp_erfinv(gmp(1.0) - x),
    );
    unary_result!("gamma", 3.5, gamma, gamma);
    unary_result!("lgamma", 3.5, lgamma, ln_gamma);

    bench_real_binary(&mut group, "atan2", 0.75, -0.5, Real::atan2, |y, x| {
        y.atan2(&x)
    });
    bench_real_binary(
        &mut group,
        "logaddexp",
        20.0,
        19.5,
        |a, b| Real::logaddexp(&a, &b).unwrap(),
        |a, b| (a.exp() + b.exp()).ln(),
    );
    bench_real_binary(
        &mut group,
        "logsubexp",
        20.0,
        19.5,
        |a, b| Real::logsubexp(&a, &b).unwrap(),
        |a, b| (a.exp() - b.exp()).ln(),
    );
    bench_real_binary(
        &mut group,
        "hypot2",
        3.0,
        4.0,
        |a, b| Real::hypot2(&a, &b).unwrap(),
        |a, b| a.hypot(&b),
    );
    bench_real_ternary(
        &mut group,
        "hypot3",
        [2.0, 3.0, 6.0],
        |x, y, z| Real::hypot3(&x, &y, &z).unwrap(),
        |x, y, z| x.hypot(&y).hypot(&z),
    );
    bench_real_binary(
        &mut group,
        "hypot_minus",
        5.0,
        4.0,
        |x, y| Real::hypot_minus(&x, &y).unwrap(),
        |x, y| x.clone().hypot(&y) - x,
    );
    bench_real_binary(
        &mut group,
        "beta",
        2.5,
        3.5,
        |a, b| Real::beta(&a, &b).unwrap(),
        |a, b| {
            let sum = a.clone() + &b;
            a.gamma() * b.gamma() / sum.gamma()
        },
    );
    bench_real_binary(
        &mut group,
        "ln_beta",
        2.5,
        3.5,
        |a, b| Real::ln_beta(&a, &b).unwrap(),
        |a, b| {
            let sum = a.clone() + &b;
            a.ln_gamma() + b.ln_gamma() - sum.ln_gamma()
        },
    );
    bench_real_binary(
        &mut group,
        "lbeta",
        2.5,
        3.5,
        |a, b| Real::lbeta(&a, &b).unwrap(),
        |a, b| {
            let sum = a.clone() + &b;
            a.ln_gamma() + b.ln_gamma() - sum.ln_gamma()
        },
    );
    bench_real_unary(
        &mut group,
        "pow_rational_5_over_3",
        2.5,
        |x| x.pow_rational(rational(5, 3)).unwrap(),
        |x| x.pow(gmp(5.0) / 3),
    );

    group.finish();
}

fn bench_real_derived_api(c: &mut Criterion) {
    let mut group = c.benchmark_group("gmp_real_derived_api");

    bench_real_unary(
        &mut group,
        "sqrt1pm1",
        1.0e-9,
        |x| x.sqrt1pm1().unwrap(),
        |x| (Float::with_val(GMP_PRECISION, 1) + x).sqrt() - 1,
    );
    bench_real_unary(
        &mut group,
        "sqrt1m1",
        1.0e-9,
        |x| x.sqrt1m1().unwrap(),
        |x| (Float::with_val(GMP_PRECISION, 1) - x).sqrt() - 1,
    );
    bench_real_unary(
        &mut group,
        "softplus",
        20.0,
        |x| x.softplus().unwrap(),
        |x| (Float::with_val(GMP_PRECISION, 1) + x.exp()).ln(),
    );
    bench_real_unary(
        &mut group,
        "sigmoid",
        -7.0,
        |x| x.sigmoid().unwrap(),
        |x| Float::with_val(GMP_PRECISION, 1) / (Float::with_val(GMP_PRECISION, 1) + (-x).exp()),
    );
    bench_real_unary(
        &mut group,
        "logit",
        0.75,
        |x| x.logit().unwrap(),
        |x| {
            let one_minus_x = Float::with_val(GMP_PRECISION, 1) - &x;
            (x / one_minus_x).ln()
        },
    );
    bench_real_unary(
        &mut group,
        "erfcx",
        4.0,
        |x| x.erfcx().unwrap(),
        |x| (x.clone() * &x).exp() * x.erfc(),
    );
    bench_real_unary(
        &mut group,
        "dnorm",
        1.25,
        |x| x.dnorm().unwrap(),
        gmp_standard_normal_pdf,
    );
    bench_real_unary(
        &mut group,
        "pnorm",
        1.25,
        |x| x.pnorm().unwrap(),
        gmp_standard_normal_cdf,
    );
    bench_real_unary(
        &mut group,
        "pnorm_upper",
        1.25,
        |x| x.pnorm_upper().unwrap(),
        gmp_standard_normal_sf,
    );
    bench_real_unary(
        &mut group,
        "normal_sf",
        6.0,
        |x| x.normal_sf().unwrap(),
        gmp_standard_normal_sf,
    );
    bench_real_unary(
        &mut group,
        "log_pnorm",
        -6.0,
        |x| x.log_pnorm().unwrap(),
        |x| gmp_standard_normal_cdf(x).ln(),
    );
    bench_real_unary(
        &mut group,
        "log_normal_sf",
        6.0,
        |x| x.log_normal_sf().unwrap(),
        |x| gmp_standard_normal_sf(x).ln(),
    );
    bench_real_unary(
        &mut group,
        "log_dnorm",
        6.0,
        |x| x.log_dnorm().unwrap(),
        |x| gmp_standard_normal_pdf(x).ln(),
    );
    bench_real_binary(
        &mut group,
        "normal_interval",
        -0.25,
        0.75,
        |lo, hi| Real::normal_interval(&lo, &hi).unwrap(),
        |lo, hi| gmp_standard_normal_cdf(hi) - gmp_standard_normal_cdf(lo),
    );
    bench_real_binary(
        &mut group,
        "pnorm_diff",
        -0.25,
        0.75,
        |lo, hi| Real::pnorm_diff(&lo, &hi).unwrap(),
        |lo, hi| gmp_standard_normal_cdf(hi) - gmp_standard_normal_cdf(lo),
    );
    bench_real_ternary(
        &mut group,
        "normal_pdf",
        [1.5, 0.25, 1.75],
        |x, mean, sigma| x.normal_pdf(&mean, &sigma).unwrap(),
        |x, mean, sigma| gmp_standard_normal_pdf((x - mean) / &sigma) / sigma,
    );
    bench_real_ternary(
        &mut group,
        "normal_cdf",
        [1.5, 0.25, 1.75],
        |x, mean, sigma| x.normal_cdf(&mean, &sigma).unwrap(),
        |x, mean, sigma| gmp_standard_normal_cdf((x - mean) / sigma),
    );
    bench_real_ternary(
        &mut group,
        "normal_survival",
        [1.5, 0.25, 1.75],
        |x, mean, sigma| x.normal_survival(&mean, &sigma).unwrap(),
        |x, mean, sigma| gmp_standard_normal_sf((x - mean) / sigma),
    );
    bench_real_unary(
        &mut group,
        "normal_mills",
        6.0,
        |x| x.normal_mills().unwrap(),
        |x| gmp_standard_normal_sf(x.clone()) / gmp_standard_normal_pdf(x),
    );
    bench_real_unary(
        &mut group,
        "normal_hazard",
        6.0,
        |x| x.normal_hazard().unwrap(),
        |x| gmp_standard_normal_pdf(x.clone()) / gmp_standard_normal_sf(x),
    );
    bench_real_unary(
        &mut group,
        "normal_log_hazard",
        6.0,
        |x| x.normal_log_hazard().unwrap(),
        |x| gmp_standard_normal_pdf(x.clone()).ln() - gmp_standard_normal_sf(x).ln(),
    );
    bench_real_unary(
        &mut group,
        "normal_inverse_mills",
        0.0,
        |x| x.normal_inverse_mills().unwrap(),
        |x| gmp_standard_normal_cdf(x.clone()) / gmp_standard_normal_pdf(x),
    );
    bench_real_unary(
        &mut group,
        "qnorm",
        0.75,
        |x| x.qnorm().unwrap(),
        gmp_standard_normal_quantile,
    );
    bench_real_unary(
        &mut group,
        "qnorm_upper",
        0.25,
        |x| x.qnorm_upper().unwrap(),
        |x| gmp_standard_normal_quantile(gmp(1.0) - x),
    );
    bench_real_ternary(
        &mut group,
        "normal_quantile",
        [0.75, 1.25, 2.5],
        |probability, mean, sigma| probability.normal_quantile(&mean, &sigma).unwrap(),
        |probability, mean, sigma| mean + sigma * gmp_standard_normal_quantile(probability),
    );
    bench_real_binary(
        &mut group,
        "regularized_gamma_p",
        2.5,
        3.75,
        |a, x| Real::regularized_gamma_p(&a, &x).unwrap(),
        gmp_regularized_gamma_p,
    );
    bench_real_binary(
        &mut group,
        "regularized_gamma_q",
        2.5,
        3.75,
        |a, x| Real::regularized_gamma_q(&a, &x).unwrap(),
        gmp_regularized_gamma_q,
    );
    bench_real_unary(
        &mut group,
        "chi_square_cdf_k5",
        7.5,
        |x| Real::chi_square_cdf(&x, 5).unwrap(),
        |x| gmp_regularized_gamma_p(gmp(2.5), x / 2),
    );
    bench_real_unary(
        &mut group,
        "chi_square_sf_k5",
        7.5,
        |x| Real::chi_square_sf(&x, 5).unwrap(),
        |x| gmp_regularized_gamma_q(gmp(2.5), x / 2),
    );
    bench_real_ternary(
        &mut group,
        "regularized_beta_integer",
        [3.0, 4.0, 0.4],
        |a, b, x| Real::regularized_beta(&a, &b, &x).unwrap(),
        |_, _, x| gmp_regularized_beta_integer(3, 4, x),
    );
    bench_real_ternary(
        &mut group,
        "regularized_beta_q_integer",
        [3.0, 4.0, 0.4],
        |a, b, x| Real::regularized_beta_q(&a, &b, &x).unwrap(),
        |_, _, x| gmp(1.0) - gmp_regularized_beta_integer(3, 4, x),
    );
    bench_real_unary(
        &mut group,
        "hermite_probabilists_n6",
        1.25,
        |x| Real::hermite_probabilists(6, &x),
        |x| gmp_hermite_probabilists(6, &x),
    );
    bench_real_unary(
        &mut group,
        "dnorm_derivative_n6",
        1.25,
        |x| x.dnorm_derivative(6).unwrap(),
        |x| gmp_hermite_probabilists(6, &x) * gmp_standard_normal_pdf(x),
    );
    bench_real_unary(
        &mut group,
        "gaussian_derivative_n6",
        1.25,
        |x| x.gaussian_derivative(6).unwrap(),
        |x| gmp_hermite_probabilists(6, &x) * gmp_standard_normal_pdf(x),
    );

    group.bench_function(
        BenchmarkId::new("hyperreal", "standard_normal_moment_n8"),
        |b| b.iter(|| black_box(Real::standard_normal_moment(8))),
    );
    group.bench_function(
        BenchmarkId::new("gmp_mpfr128", "standard_normal_moment_n8"),
        |b| b.iter(|| black_box(gmp(105.0))),
    );
    bench_real_binary(
        &mut group,
        "normal_interval_moment_n4",
        -0.5,
        1.25,
        |lo, hi| Real::normal_interval_moment(&lo, &hi, 4).unwrap(),
        |lo, hi| gmp_normal_interval_moment(&lo, &hi, 4),
    );
    bench_real_binary(
        &mut group,
        "truncated_normal_mean",
        -0.5,
        1.25,
        |lo, hi| Real::truncated_normal_mean(&lo, &hi).unwrap(),
        |lo, hi| {
            let mass = gmp_standard_normal_cdf(hi.clone()) - gmp_standard_normal_cdf(lo.clone());
            (gmp_standard_normal_pdf(lo) - gmp_standard_normal_pdf(hi)) / mass
        },
    );
    bench_real_binary(
        &mut group,
        "truncated_normal_variance",
        -0.5,
        1.25,
        |lo, hi| Real::truncated_normal_variance(&lo, &hi).unwrap(),
        |lo, hi| {
            let mass = gmp_standard_normal_cdf(hi.clone()) - gmp_standard_normal_cdf(lo.clone());
            let phi_lo = gmp_standard_normal_pdf(lo.clone());
            let phi_hi = gmp_standard_normal_pdf(hi.clone());
            let mean = (phi_lo.clone() - &phi_hi) / &mass;
            gmp(1.0) + (lo * phi_lo - hi * phi_hi) / mass - mean.clone() * mean
        },
    );

    group.finish();
}

fn bench_real_linear_algebra_api(c: &mut Criterion) {
    let mut group = c.benchmark_group("gmp_real_linear_algebra_api");

    bench_real_ternary(
        &mut group,
        "mul_add",
        [1.25, -3.5, 0.75],
        |a, b, c| Real::mul_add(&a, &b, &c),
        |a, b, c| a * b + c,
    );

    let h_values = [real(1.25), real(-3.5), real(0.75), real(2.0)];
    let g_values = [gmp(1.25), gmp(-3.5), gmp(0.75), gmp(2.0)];
    group.bench_function(BenchmarkId::new("hyperreal", "diff_of_products"), |b| {
        b.iter(|| {
            black_box(Real::diff_of_products(
                &h_values[0],
                &h_values[1],
                &h_values[2],
                &h_values[3],
            ))
        })
    });
    group.bench_function(BenchmarkId::new("gmp_mpfr128", "diff_of_products"), |b| {
        b.iter(|| {
            black_box(g_values[0].clone() * &g_values[1] - g_values[2].clone() * &g_values[3])
        })
    });

    let h_left = [real(1.25), real(-3.5), real(0.75), real(2.0)];
    let h_right = [real(2.0), real(0.75), real(-3.5), real(1.25)];
    let g_left = [gmp(1.25), gmp(-3.5), gmp(0.75), gmp(2.0)];
    let g_right = [gmp(2.0), gmp(0.75), gmp(-3.5), gmp(1.25)];
    group.bench_function(BenchmarkId::new("hyperreal", "sum_products"), |b| {
        b.iter(|| black_box(Real::sum_products(&h_left, &h_right).unwrap()))
    });
    group.bench_function(BenchmarkId::new("gmp_mpfr128", "sum_products"), |b| {
        b.iter(|| {
            black_box((0..4).fold(gmp(0.0), |sum, index| {
                sum + g_left[index].clone() * &g_right[index]
            }))
        })
    });

    macro_rules! linear_pair {
        ($name:literal, $hyperreal:expr, $gmp:expr) => {
            group.bench_function(BenchmarkId::new("hyperreal", $name), |b| {
                b.iter(|| black_box($hyperreal))
            });
            group.bench_function(BenchmarkId::new("gmp_mpfr128", $name), |b| {
                b.iter(|| black_box($gmp))
            });
        };
    }

    linear_pair!(
        "dot2_refs",
        Real::dot2_refs([&h_left[0], &h_left[1]], [&h_right[0], &h_right[1]]),
        gmp_dot(
            &[g_left[0].clone(), g_left[1].clone()],
            &[g_right[0].clone(), g_right[1].clone()],
        )
    );
    linear_pair!(
        "active_dot2_refs",
        Real::active_dot2_refs([&h_left[0], &h_left[1]], [&h_right[0], &h_right[1]]),
        gmp_dot(
            &[g_left[0].clone(), g_left[1].clone()],
            &[g_right[0].clone(), g_right[1].clone()],
        )
    );
    linear_pair!(
        "dot3_refs",
        Real::dot3_refs(
            [&h_left[0], &h_left[1], &h_left[2]],
            [&h_right[0], &h_right[1], &h_right[2]],
        ),
        gmp_dot(
            &[g_left[0].clone(), g_left[1].clone(), g_left[2].clone()],
            &[g_right[0].clone(), g_right[1].clone(), g_right[2].clone(),],
        )
    );
    linear_pair!(
        "active_dot3_refs",
        Real::active_dot3_refs(
            [&h_left[0], &h_left[1], &h_left[2]],
            [&h_right[0], &h_right[1], &h_right[2]],
        ),
        gmp_dot(
            &[g_left[0].clone(), g_left[1].clone(), g_left[2].clone()],
            &[g_right[0].clone(), g_right[1].clone(), g_right[2].clone(),],
        )
    );
    linear_pair!(
        "dot4_refs",
        Real::dot4_refs(
            [&h_left[0], &h_left[1], &h_left[2], &h_left[3]],
            [&h_right[0], &h_right[1], &h_right[2], &h_right[3]],
        ),
        gmp_dot(&g_left, &g_right)
    );
    linear_pair!(
        "active_dot4_refs",
        Real::active_dot4_refs(
            [&h_left[0], &h_left[1], &h_left[2], &h_left[3]],
            [&h_right[0], &h_right[1], &h_right[2], &h_right[3]],
        ),
        gmp_dot(&g_left, &g_right)
    );
    linear_pair!(
        "linear_combination3_refs",
        Real::linear_combination3_refs(
            [&h_left[0], &h_left[1], &h_left[2]],
            [&h_right[0], &h_right[1], &h_right[2]],
        ),
        gmp_dot(
            &[g_left[0].clone(), g_left[1].clone(), g_left[2].clone()],
            &[g_right[0].clone(), g_right[1].clone(), g_right[2].clone(),],
        )
    );
    linear_pair!(
        "active_linear_combination3_refs",
        Real::active_linear_combination3_refs(
            [&h_left[0], &h_left[1], &h_left[2]],
            [&h_right[0], &h_right[1], &h_right[2]],
        ),
        gmp_dot(
            &[g_left[0].clone(), g_left[1].clone(), g_left[2].clone()],
            &[g_right[0].clone(), g_right[1].clone(), g_right[2].clone(),],
        )
    );
    linear_pair!(
        "linear_combination4_refs",
        Real::linear_combination4_refs(
            [&h_left[0], &h_left[1], &h_left[2], &h_left[3]],
            [&h_right[0], &h_right[1], &h_right[2], &h_right[3]],
        ),
        gmp_dot(&g_left, &g_right)
    );
    linear_pair!(
        "active_linear_combination4_refs",
        Real::active_linear_combination4_refs(
            [&h_left[0], &h_left[1], &h_left[2], &h_left[3]],
            [&h_right[0], &h_right[1], &h_right[2], &h_right[3]],
        ),
        gmp_dot(&g_left, &g_right)
    );

    let offset = real(0.5);
    let g_offset = gmp(0.5);
    linear_pair!(
        "affine_combination3_refs",
        Real::affine_combination3_refs(
            [&h_left[0], &h_left[1], &h_left[2]],
            [&h_right[0], &h_right[1], &h_right[2]],
            &offset,
        ),
        gmp_dot(
            &[g_left[0].clone(), g_left[1].clone(), g_left[2].clone()],
            &[g_right[0].clone(), g_right[1].clone(), g_right[2].clone(),],
        ) + &g_offset
    );
    linear_pair!(
        "affine_combination4_refs",
        Real::affine_combination4_refs(
            [&h_left[0], &h_left[1], &h_left[2], &h_left[3]],
            [&h_right[0], &h_right[1], &h_right[2], &h_right[3]],
            &offset,
        ),
        gmp_dot(&g_left, &g_right) + &g_offset
    );
    linear_pair!(
        "signed_product_sum",
        Real::signed_product_sum(
            [true, false],
            [[&h_left[0], &h_right[0]], [&h_left[1], &h_right[1]]],
        ),
        g_left[0].clone() * &g_right[0] - g_left[1].clone() * &g_right[1]
    );
    linear_pair!(
        "active_signed_product_sum",
        Real::active_signed_product_sum(
            [true, false],
            [[&h_left[0], &h_right[0]], [&h_left[1], &h_right[1]]],
        ),
        g_left[0].clone() * &g_right[0] - g_left[1].clone() * &g_right[1]
    );

    let h_coeffs = [real(1.0), real(-2.0), real(3.0), real(-4.0)];
    let g_coeffs = [gmp(1.0), gmp(-2.0), gmp(3.0), gmp(-4.0)];
    let h_x = real(0.75);
    let g_x = gmp(0.75);
    group.bench_function(BenchmarkId::new("hyperreal", "eval_poly"), |b| {
        b.iter(|| black_box(Real::eval_poly(&h_coeffs, &h_x)))
    });
    group.bench_function(BenchmarkId::new("gmp_mpfr128", "eval_poly"), |b| {
        b.iter(|| {
            black_box(
                g_coeffs
                    .iter()
                    .rev()
                    .fold(gmp(0.0), |acc, coefficient| acc * &g_x + coefficient),
            )
        })
    });

    let h_den_coeffs = [real(1.0), real(0.5), real(0.25)];
    let g_den_coeffs = [gmp(1.0), gmp(0.5), gmp(0.25)];
    linear_pair!(
        "eval_rational_poly",
        Real::eval_rational_poly(&h_coeffs, &h_den_coeffs, &h_x).unwrap(),
        {
            let numerator = g_coeffs
                .iter()
                .rev()
                .fold(gmp(0.0), |acc, coefficient| acc * &g_x + coefficient);
            let denominator = g_den_coeffs
                .iter()
                .rev()
                .fold(gmp(0.0), |acc, coefficient| acc * &g_x + coefficient);
            numerator / denominator
        }
    );

    group.finish();
}

fn bench_real_collection_and_conversion_api(c: &mut Criterion) {
    let mut group = c.benchmark_group("gmp_real_collection_and_conversion_api");
    let hyperreal_values = [real(1.25), real(-3.5), real(0.75), real(2.0)];
    let gmp_values = [gmp(1.25), gmp(-3.5), gmp(0.75), gmp(2.0)];

    macro_rules! pair {
        ($name:literal, $hyperreal:expr, $gmp:expr) => {
            group.bench_function(BenchmarkId::new("hyperreal", $name), |b| {
                b.iter(|| black_box($hyperreal))
            });
            group.bench_function(BenchmarkId::new("gmp_mpfr128", $name), |b| {
                b.iter(|| black_box($gmp))
            });
        };
    }

    pair!("zero", Real::zero(), gmp(0.0));
    pair!("one", Real::one(), gmp(1.0));
    pair!("new_rational", Real::new(rational(7, 4)), gmp(7.0) / 4);
    pair!(
        "integer_bigint",
        Real::integer(BigInt::from(1_u64) << 160),
        Float::with_val(GMP_PRECISION, rug::Integer::from(1) << 160)
    );
    pair!(
        "pi",
        Real::pi(),
        Float::with_val(GMP_PRECISION, Constant::Pi)
    );
    pair!("e", Real::e(), gmp(1.0).exp());
    pair!(
        "tau",
        Real::tau(),
        Float::with_val(GMP_PRECISION, Constant::Pi) * 2
    );

    group.bench_function(BenchmarkId::new("hyperreal", "sum_owned"), |b| {
        b.iter(|| black_box(Real::sum_owned(hyperreal_values.clone())))
    });
    group.bench_function(BenchmarkId::new("gmp_mpfr128", "sum_owned"), |b| {
        b.iter(|| black_box(gmp_sum(&gmp_values)))
    });
    group.bench_function(BenchmarkId::new("hyperreal", "sum_refs"), |b| {
        b.iter(|| black_box(Real::sum_refs(hyperreal_values.iter())))
    });
    group.bench_function(BenchmarkId::new("gmp_mpfr128", "sum_refs"), |b| {
        b.iter(|| black_box(gmp_sum(&gmp_values)))
    });
    group.bench_function(BenchmarkId::new("hyperreal", "mean"), |b| {
        b.iter(|| black_box(Real::mean(&hyperreal_values).unwrap()))
    });
    group.bench_function(BenchmarkId::new("gmp_mpfr128", "mean"), |b| {
        b.iter(|| black_box(gmp_sum(&gmp_values) / 4))
    });
    group.bench_function(BenchmarkId::new("hyperreal", "sample_stddev"), |b| {
        b.iter(|| black_box(Real::sample_stddev(&hyperreal_values).unwrap()))
    });
    group.bench_function(BenchmarkId::new("gmp_mpfr128", "sample_stddev"), |b| {
        b.iter(|| {
            let mean = gmp_sum(&gmp_values) / 4;
            let variance: Float = gmp_values.iter().fold(gmp(0.0), |sum, value| {
                let delta: Float = value.clone() - &mean;
                sum + delta.clone() * delta
            }) / 3;
            black_box(variance.sqrt())
        })
    });

    let origin = real(1.25);
    let t = real(0.75);
    let delta = real(-3.5);
    let g_origin = gmp(1.25);
    let g_t = gmp(0.75);
    let g_delta = gmp(-3.5);
    group.bench_function(BenchmarkId::new("hyperreal", "affine"), |b| {
        b.iter(|| black_box(Real::affine(&origin, &t, &delta)))
    });
    group.bench_function(BenchmarkId::new("gmp_mpfr128", "affine"), |b| {
        b.iter(|| black_box(g_origin.clone() + g_t.clone() * &g_delta))
    });

    let value = real(0.75);
    let g_value = gmp(0.75);
    group.bench_function(BenchmarkId::new("hyperreal", "to_f64_lossy"), |b| {
        b.iter(|| black_box(value.to_f64_lossy().unwrap()))
    });
    group.bench_function(BenchmarkId::new("gmp_mpfr128", "to_f64_lossy"), |b| {
        b.iter(|| black_box(g_value.to_f64()))
    });
    group.bench_function(BenchmarkId::new("hyperreal", "to_f32_lossy"), |b| {
        b.iter(|| black_box(value.to_f32_lossy().unwrap()))
    });
    group.bench_function(BenchmarkId::new("gmp_mpfr128", "to_f32_lossy"), |b| {
        b.iter(|| black_box(g_value.to_f32()))
    });
    group.bench_function(BenchmarkId::new("hyperreal", "inverse_ref"), |b| {
        b.iter(|| black_box(value.inverse_ref().unwrap()))
    });
    group.bench_function(BenchmarkId::new("gmp_mpfr128", "inverse_ref"), |b| {
        b.iter(|| black_box(g_value.clone().recip()))
    });
    pair!("is_integer", value.is_integer(), g_value.is_integer());
    pair!("is_finite", value.is_finite(), g_value.is_finite());
    pair!(
        "to_f64_exact_dyadic",
        value.to_f64_exact_dyadic(),
        Some(g_value.to_f64())
    );

    let other = real(-3.5);
    let g_other = gmp(-3.5);
    pair!(
        "min",
        value.min(&other).clone(),
        g_value.clone().min(&g_other)
    );
    pair!(
        "max",
        value.max(&other).clone(),
        g_value.clone().max(&g_other)
    );

    group.finish();
}

fn bench_computable_api(c: &mut Criterion) {
    let mut group = c.benchmark_group("gmp_computable_api_p128");

    macro_rules! unary {
        ($name:literal, $input:expr, $hyper:ident, $gmp:ident) => {{
            let hyperreal_input = Computable::rational(rational($input, 4));
            group.bench_function(BenchmarkId::new("hyperreal", $name), |b| {
                b.iter(|| black_box(hyperreal_input.clone().$hyper().approx(-128)))
            });
            let gmp_input = gmp(f64::from($input) / 4.0);
            group.bench_function(BenchmarkId::new("gmp_mpfr128", $name), |b| {
                b.iter(|| black_box(gmp_input.clone().$gmp()))
            });
        }};
    }

    macro_rules! unary_custom {
        ($name:literal, $numerator:expr, $denominator:expr, $hyperreal:expr, $gmp:expr) => {{
            let hyperreal_input = Computable::rational(rational($numerator, $denominator));
            group.bench_function(BenchmarkId::new("hyperreal", $name), |b| {
                b.iter(|| {
                    let input = hyperreal_input.clone();
                    black_box(($hyperreal)(input).approx(-128))
                })
            });
            let gmp_input = gmp($numerator as f64 / $denominator as f64);
            group.bench_function(BenchmarkId::new("gmp_mpfr128", $name), |b| {
                b.iter(|| {
                    let input = gmp_input.clone();
                    black_box(($gmp)(input))
                })
            });
        }};
    }

    unary!("sqrt", 3, sqrt, sqrt);
    unary!("inverse", 3, inverse, recip);
    unary!("exp", 3, exp, exp);
    unary!("expm1", 3, expm1, exp_m1);
    unary!("ln", 7, ln, ln);
    unary!("sin", 3, sin, sin);
    unary!("cos", 3, cos, cos);
    unary!("tan", 3, tan, tan);
    unary!("asin", 3, asin, asin);
    unary!("acos", 3, acos, acos);
    unary!("atan", 3, atan, atan);
    unary!("asinh", 3, asinh, asinh);
    unary!("acosh", 7, acosh, acosh);
    unary!("atanh", 3, atanh, atanh);
    unary!("erf", 3, erf, erf);
    unary!("erfc", 3, erfc, erfc);
    unary_custom!("erfcx", 3, 4, Computable::erfcx, |x: Float| (x.clone()
        * &x)
        .exp()
        * x.erfc());
    unary_custom!("pnorm", 3, 4, Computable::pnorm, gmp_standard_normal_cdf);
    unary_custom!("dnorm", 3, 4, Computable::dnorm, gmp_standard_normal_pdf);
    unary_custom!(
        "normal_sf",
        3,
        4,
        Computable::normal_sf,
        gmp_standard_normal_sf
    );
    unary_custom!("log_pnorm", -6, 4, Computable::log_pnorm, |x: Float| {
        gmp_standard_normal_cdf(x).ln()
    });
    unary_custom!(
        "log_normal_sf",
        6,
        4,
        Computable::log_normal_sf,
        |x: Float| gmp_standard_normal_sf(x).ln()
    );
    unary_custom!("log_dnorm", 6, 4, Computable::log_dnorm, |x: Float| {
        gmp_standard_normal_pdf(x).ln()
    });

    let h_lhs = Computable::rational(rational(3, 4));
    let h_rhs = Computable::rational(rational(7, 4));
    let g_lhs = gmp(0.75);
    let g_rhs = gmp(1.75);
    macro_rules! binary {
        ($name:literal, $hyperreal:expr, $gmp:expr) => {
            group.bench_function(BenchmarkId::new("hyperreal", $name), |b| {
                b.iter(|| black_box(($hyperreal).approx(-128)))
            });
            group.bench_function(BenchmarkId::new("gmp_mpfr128", $name), |b| {
                b.iter(|| black_box($gmp))
            });
        };
    }
    binary!("negate", h_lhs.clone().negate(), -g_lhs.clone());
    binary!("square", h_lhs.clone().square(), g_lhs.clone().square());
    binary!(
        "multiply",
        h_lhs.clone().multiply(h_rhs.clone()),
        g_lhs.clone() * &g_rhs
    );
    binary!(
        "add",
        h_lhs.clone().add(h_rhs.clone()),
        g_lhs.clone() + &g_rhs
    );
    binary!(
        "atan2",
        h_lhs.clone().atan2(h_rhs.clone()),
        g_lhs.clone().atan2(&g_rhs)
    );

    group.bench_function(BenchmarkId::new("hyperreal", "normal_interval"), |b| {
        b.iter(|| black_box(Computable::normal_interval(h_lhs.clone(), h_rhs.clone()).approx(-128)))
    });
    group.bench_function(BenchmarkId::new("gmp_mpfr128", "normal_interval"), |b| {
        b.iter(|| {
            black_box(
                gmp_standard_normal_cdf(g_rhs.clone()) - gmp_standard_normal_cdf(g_lhs.clone()),
            )
        })
    });

    let h_quantile_probability = Computable::rational(Rational::new(2)).pnorm();
    let quantile_seed = BigInt::from((1.9999_f64 * f64::from(1_u32 << 13)).round() as i64);
    let g_quantile_probability = gmp_standard_normal_cdf(gmp(2.0));
    group.bench_function(BenchmarkId::new("hyperreal", "normal_quantile"), |b| {
        b.iter(|| {
            black_box(
                Computable::normal_quantile(
                    h_quantile_probability.clone(),
                    quantile_seed.clone(),
                    -13,
                )
                .approx(-128),
            )
        })
    });
    group.bench_function(BenchmarkId::new("gmp_mpfr128", "normal_quantile"), |b| {
        b.iter(|| black_box(gmp_standard_normal_quantile(g_quantile_probability.clone())))
    });

    group.bench_function(BenchmarkId::new("hyperreal", "try_compare_to"), |b| {
        b.iter(|| black_box(h_lhs.try_compare_to(&h_rhs)))
    });
    group.bench_function(BenchmarkId::new("gmp_mpfr128", "try_compare_to"), |b| {
        b.iter(|| black_box(g_lhs.partial_cmp(&g_rhs)))
    });
    group.bench_function(BenchmarkId::new("hyperreal", "compare_absolute"), |b| {
        b.iter(|| black_box(h_lhs.compare_absolute(&h_rhs, -128)))
    });
    group.bench_function(BenchmarkId::new("gmp_mpfr128", "compare_absolute"), |b| {
        b.iter(|| black_box(g_lhs.partial_cmp(&g_rhs).unwrap()))
    });
    group.bench_function(BenchmarkId::new("hyperreal", "sign"), |b| {
        b.iter(|| black_box(h_lhs.sign()))
    });
    group.bench_function(BenchmarkId::new("gmp_mpfr128", "sign"), |b| {
        b.iter(|| black_box(g_lhs.cmp0()))
    });
    group.bench_function(BenchmarkId::new("hyperreal", "sign_until"), |b| {
        b.iter(|| black_box(h_lhs.sign_until(-128)))
    });
    group.bench_function(BenchmarkId::new("gmp_mpfr128", "sign_until"), |b| {
        b.iter(|| black_box(Some(g_lhs.cmp0())))
    });
    group.bench_function(BenchmarkId::new("hyperreal", "zero_status"), |b| {
        b.iter(|| black_box(h_lhs.zero_status()))
    });
    group.bench_function(BenchmarkId::new("gmp_mpfr128", "zero_status"), |b| {
        b.iter(|| black_box(g_lhs == 0))
    });

    for (name, hyperreal, gmp_mpfr) in [
        (
            "pi",
            Computable::pi(),
            Float::with_val(GMP_PRECISION, Constant::Pi),
        ),
        (
            "e",
            Computable::e(),
            Float::with_val(GMP_PRECISION, 1).exp(),
        ),
        (
            "tau",
            Computable::tau(),
            Float::with_val(GMP_PRECISION, Constant::Pi) * 2,
        ),
    ] {
        group.bench_function(BenchmarkId::new("hyperreal", name), |b| {
            b.iter(|| black_box(hyperreal.clone().approx(-128)))
        });
        group.bench_function(BenchmarkId::new("gmp_mpfr128", name), |b| {
            b.iter(|| black_box(gmp_mpfr.clone()))
        });
    }

    group.finish();
}

fn bench_magnitude_backend_algorithms(c: &mut Criterion) {
    let mut group = c.benchmark_group("gmp_magnitude_algorithms");
    for bits in [4096_usize, 16_384, 65_536] {
        let num_left = (BigUint::from(1_u8) << bits) - 159_u8;
        let num_right = (BigUint::from(1_u8) << bits) - 173_u8;
        let gmp_left = (Integer::from(1) << bits) - 159;
        let gmp_right = (Integer::from(1) << bits) - 173;
        group.bench_function(BenchmarkId::new("num_bigint_mul", bits), |b| {
            b.iter(|| black_box(black_box(&num_left) * black_box(&num_right)))
        });
        group.bench_function(BenchmarkId::new("gmp_mul", bits), |b| {
            b.iter(|| black_box(Integer::from(black_box(&gmp_left) * black_box(&gmp_right))))
        });
        group.bench_function(BenchmarkId::new("gmp_roundtrip_mul", bits), |b| {
            b.iter(|| {
                let left_bytes = black_box(&num_left).to_bytes_le();
                let right_bytes = black_box(&num_right).to_bytes_le();
                let left = Integer::from_digits(&left_bytes, Order::Lsf);
                let right = Integer::from_digits(&right_bytes, Order::Lsf);
                let product = left * right;
                black_box(BigUint::from_bytes_le(&product.to_digits(Order::Lsf)))
            })
        });

        let num_dividend = (&num_left << bits) + &num_right;
        let gmp_dividend = (Integer::from(&gmp_left) << bits) + &gmp_right;
        group.bench_function(BenchmarkId::new("num_bigint_div", bits), |b| {
            b.iter(|| black_box(black_box(&num_dividend) / black_box(&num_right)))
        });
        group.bench_function(BenchmarkId::new("gmp_div", bits), |b| {
            b.iter(|| {
                black_box(Integer::from(
                    black_box(&gmp_dividend) / black_box(&gmp_right),
                ))
            })
        });
        group.bench_function(BenchmarkId::new("gmp_roundtrip_div", bits), |b| {
            b.iter(|| {
                let dividend_bytes = black_box(&num_dividend).to_bytes_le();
                let divisor_bytes = black_box(&num_right).to_bytes_le();
                let dividend = Integer::from_digits(&dividend_bytes, Order::Lsf);
                let divisor = Integer::from_digits(&divisor_bytes, Order::Lsf);
                let quotient = dividend / divisor;
                black_box(BigUint::from_bytes_le(&quotient.to_digits(Order::Lsf)))
            })
        });
    }
    group.finish();
}

criterion_group!(
    benches,
    bench_rational_api,
    bench_real_arithmetic_api,
    bench_real_elementary_api,
    bench_real_derived_api,
    bench_real_linear_algebra_api,
    bench_real_collection_and_conversion_api,
    bench_computable_api,
    bench_magnitude_backend_algorithms,
);
criterion_main!(benches);
