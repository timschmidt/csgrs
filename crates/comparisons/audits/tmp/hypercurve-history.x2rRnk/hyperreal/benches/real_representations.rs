//! Competitive construction, cloning, export, and certification benchmarks for
//! every optimized `Real` certificate representation.
//!
//! The MPFR rows compute the same mathematical value at a fixed 192-bit
//! precision. Hyperreal construction retains an exact symbolic value, so the
//! comparison intentionally distinguishes exact-symbolic work from fixed-
//! precision approximation rather than claiming identical semantics.

use criterion::{BenchmarkId, Criterion, Throughput, black_box, criterion_group, criterion_main};
use hyperreal::{Rational, Real};
use rug::{Float, float::Constant};

const MPFR_PRECISION: u32 = 192;

#[derive(Clone, Copy)]
enum Recipe {
    One,
    Pi,
    PiPow,
    PiInv,
    PiExp,
    PiInvExp,
    PiSqrt,
    ConstProduct,
    ConstOffset,
    ConstProductSqrt,
    Sqrt,
    Exp,
    Ln,
    LnAffine,
    LnProduct,
    Log10,
    Log2,
    SinPi,
    TanPi,
    Irrational,
}

const RECIPES: [Recipe; 20] = [
    Recipe::One,
    Recipe::Pi,
    Recipe::PiPow,
    Recipe::PiInv,
    Recipe::PiExp,
    Recipe::PiInvExp,
    Recipe::PiSqrt,
    Recipe::ConstProduct,
    Recipe::ConstOffset,
    Recipe::ConstProductSqrt,
    Recipe::Sqrt,
    Recipe::Exp,
    Recipe::Ln,
    Recipe::LnAffine,
    Recipe::LnProduct,
    Recipe::Log10,
    Recipe::Log2,
    Recipe::SinPi,
    Recipe::TanPi,
    Recipe::Irrational,
];

impl Recipe {
    const fn name(self) -> &'static str {
        match self {
            Self::One => "one",
            Self::Pi => "pi",
            Self::PiPow => "pi_pow",
            Self::PiInv => "pi_inv",
            Self::PiExp => "pi_exp",
            Self::PiInvExp => "pi_inv_exp",
            Self::PiSqrt => "pi_sqrt",
            Self::ConstProduct => "const_product",
            Self::ConstOffset => "const_offset",
            Self::ConstProductSqrt => "const_product_sqrt",
            Self::Sqrt => "sqrt",
            Self::Exp => "exp",
            Self::Ln => "ln",
            Self::LnAffine => "ln_affine",
            Self::LnProduct => "ln_product",
            Self::Log10 => "log10",
            Self::Log2 => "log2",
            Self::SinPi => "sin_pi",
            Self::TanPi => "tan_pi",
            Self::Irrational => "irrational",
        }
    }

    fn hyperreal(self) -> Real {
        match self {
            Self::One => Real::new(Rational::fraction(3, 2).unwrap()),
            Self::Pi => Real::pi(),
            Self::PiPow => {
                let pi = Real::pi();
                &pi * &pi
            }
            Self::PiInv => Real::pi().inverse().expect("pi is nonzero"),
            Self::PiExp => Real::pi() * Real::e(),
            Self::PiInvExp => (Real::e() / Real::pi()).expect("pi is nonzero"),
            Self::PiSqrt => Real::pi() * Real::from(2).sqrt().expect("positive radicand"),
            Self::ConstProduct => {
                let pi = Real::pi();
                (&pi * &pi) * Real::e()
            }
            Self::ConstOffset => Real::pi() - Real::from(3),
            Self::ConstProductSqrt => {
                let pi = Real::pi();
                (&pi * &pi) * Real::e() * Real::from(2).sqrt().expect("positive radicand")
            }
            Self::Sqrt => Real::from(2).sqrt().expect("positive radicand"),
            Self::Exp => Real::from(2).exp().expect("finite exponential"),
            Self::Ln => Real::from(3).ln().expect("positive input"),
            Self::LnAffine => (Real::from(2) * Real::e())
                .ln()
                .expect("positive logarithm input"),
            Self::LnProduct => {
                Real::from(2).ln().expect("positive input")
                    * Real::from(3).ln().expect("positive input")
            }
            Self::Log10 => Real::from(2).log10().expect("positive input"),
            Self::Log2 => Real::from(3).log2().expect("positive input"),
            Self::SinPi => Real::new(Rational::fraction(1, 5).unwrap()).sin_pi(),
            Self::TanPi => Real::new(Rational::fraction(1, 5).unwrap())
                .tan_pi()
                .expect("not a tangent pole"),
            Self::Irrational => Real::one().sin(),
        }
    }

    fn mpfr(self) -> Float {
        let pi = || Float::with_val(MPFR_PRECISION, Constant::Pi);
        let e = || Float::with_val(MPFR_PRECISION, 1).exp();
        let sqrt_two = || Float::with_val(MPFR_PRECISION, 2).sqrt();
        let ln_two = || Float::with_val(MPFR_PRECISION, 2).ln();
        let ln_three = || Float::with_val(MPFR_PRECISION, 3).ln();

        match self {
            Self::One => Float::with_val(MPFR_PRECISION, 1.5),
            Self::Pi => pi(),
            Self::PiPow => {
                let value = pi();
                Float::with_val(MPFR_PRECISION, &value * &value)
            }
            Self::PiInv => Float::with_val(MPFR_PRECISION, 1) / pi(),
            Self::PiExp => pi() * e(),
            Self::PiInvExp => e() / pi(),
            Self::PiSqrt => pi() * sqrt_two(),
            Self::ConstProduct => {
                let value = pi();
                Float::with_val(MPFR_PRECISION, &value * &value) * e()
            }
            Self::ConstOffset => pi() - 3,
            Self::ConstProductSqrt => {
                let value = pi();
                Float::with_val(MPFR_PRECISION, &value * &value) * e() * sqrt_two()
            }
            Self::Sqrt => sqrt_two(),
            Self::Exp => Float::with_val(MPFR_PRECISION, 2).exp(),
            Self::Ln => ln_three(),
            Self::LnAffine => (Float::with_val(MPFR_PRECISION, 2) * e()).ln(),
            Self::LnProduct => ln_two() * ln_three(),
            Self::Log10 => Float::with_val(MPFR_PRECISION, 2).log10(),
            Self::Log2 => Float::with_val(MPFR_PRECISION, 3).log2(),
            Self::SinPi => {
                let mut value = pi();
                value /= 5;
                value.sin()
            }
            Self::TanPi => {
                let mut value = pi();
                value /= 5;
                value.tan()
            }
            Self::Irrational => Float::with_val(MPFR_PRECISION, 1).sin(),
        }
    }
}

fn benchmark_representations(criterion: &mut Criterion) {
    assert_eq!(RECIPES.len(), 20, "update every representation benchmark");

    let mut construction = criterion.benchmark_group("real_representation_construction_export");
    construction.throughput(Throughput::Elements(1));
    for recipe in RECIPES {
        let hyperreal_id = BenchmarkId::new("hyperreal_exact", recipe.name());
        construction.bench_function(hyperreal_id, |bencher| {
            bencher.iter(|| black_box(black_box(recipe).hyperreal().to_f64_lossy()));
        });

        let mpfr_id = BenchmarkId::new("mpfr192", recipe.name());
        construction.bench_function(mpfr_id, |bencher| {
            bencher.iter(|| black_box(black_box(recipe).mpfr().to_f64()));
        });
    }
    construction.finish();

    let mut prepared = criterion.benchmark_group("real_representation_prepared");
    prepared.throughput(Throughput::Elements(1));
    for recipe in RECIPES {
        let hyperreal = recipe.hyperreal();
        let mpfr = recipe.mpfr();
        let hyperreal_value = hyperreal
            .to_f64_lossy()
            .expect("finite representative has a binary64 approximation");
        let mpfr_value = mpfr.to_f64();
        let tolerance = 32.0 * f64::EPSILON * mpfr_value.abs().max(1.0);
        assert!(
            (hyperreal_value - mpfr_value).abs() <= tolerance,
            "{} recipes must describe the same value",
            recipe.name(),
        );

        prepared.bench_function(
            BenchmarkId::new("hyperreal_clone", recipe.name()),
            |bencher| bencher.iter(|| black_box(black_box(&hyperreal).clone())),
        );
        prepared.bench_function(
            BenchmarkId::new("mpfr192_clone", recipe.name()),
            |bencher| {
                bencher.iter(|| black_box(black_box(&mpfr).clone()));
            },
        );
        prepared.bench_function(
            BenchmarkId::new("hyperreal_cached_f64", recipe.name()),
            |bencher| bencher.iter(|| black_box(black_box(&hyperreal).to_f64_lossy())),
        );
        prepared.bench_function(BenchmarkId::new("mpfr192_f64", recipe.name()), |bencher| {
            bencher.iter(|| black_box(black_box(&mpfr).to_f64()))
        });
        prepared.bench_function(
            BenchmarkId::new("hyperreal_certified_192", recipe.name()),
            |bencher| {
                bencher.iter(|| {
                    black_box(
                        black_box(&hyperreal)
                            .certified_dyadic_interval(-192)
                            .expect("finite representative is certifiable"),
                    )
                });
            },
        );
    }
    prepared.finish();
}

criterion_group!(benches, benchmark_representations);
criterion_main!(benches);
