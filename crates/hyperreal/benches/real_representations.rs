//! Fixed scalar recipe timings for construction/export, cloning and enclosure queries.
//!
//! Twenty recipes exercise named rational, symbolic and opaque constructions. This
//! is not the complete current certificate inventory: the separate shared fixture
//! catalogue also has Pow2 and Pow10 cases. No private representation tag is asserted
//! here. Recipe names describe intended constructions, not proved runtime classes.
//!
//! Construction rows include lossy binary64 export and temporary destruction. MPFR
//! uses rounded 192-bit points, while Hyperreal starts with exact values and can
//! refine or use symbolic float shortcuts. Prepared setup compares binary64 views
//! within a tolerance; it does not establish exact equality or enclosure correctness.
//! Warm objects and shared constants can retain evidence across iterations.
//!
//! Criterion 0.5.1 generates `benches`, configuring a default runner from CLI options
//! and invoking the workload then the shared report callback, and `main`, invoking
//! that group and the final summary. Those two generated functions lack attached
//! item documentation; their coverage gap is retained in the repository audit.
//! Sampling settings do not impose a total-work or memory bound, and no explicit
//! MPFR cache-cleanup guard is installed by this benchmark.

use criterion::{BenchmarkId, Criterion, Throughput, black_box, criterion_group, criterion_main};
use hyperreal::{Rational, Real};
use rug::{Float, float::Constant};

/// Shared stored-result report callback, run after the benchmark group completes.
#[path = "support/benchmark_report.rs"]
mod benchmark_report;

/// MPFR significand precision for every reference recipe: 192 binary bits.
///
/// This controls rounded point arithmetic, not a directed error enclosure, exact
/// symbolic retention or a uniform relative-error guarantee for compound recipes.
const MPFR_PRECISION: u32 = 192;

/// A fieldless selector for one fixed pair of native and MPFR constructions.
///
/// Derived Clone/Copy duplicate only this selector, not a Real graph or Float.
/// Names resemble certificate families but no private-class check is performed.
/// The twenty selectors are a finite workload catalogue, not an exhaustive enum
/// variant or scalar-domain qualification.
#[derive(Clone, Copy)]
enum Recipe {
    /// The exact rational 3/2; the label names the rational carrier, not the value one.
    One,
    /// The exact constant pi.
    Pi,
    /// The product pi*pi, representing pi squared.
    PiPow,
    /// The reciprocal 1/pi, with its nonzero divisor assumed by the fixture.
    PiInv,
    /// The product pi*e.
    PiExp,
    /// The quotient e/pi.
    PiInvExp,
    /// The product pi*sqrt(2), taking the positive square root.
    PiSqrt,
    /// The constant product pi^2*e.
    ConstProduct,
    /// The positive constant difference pi-3.
    ConstOffset,
    /// The product pi^2*e*sqrt(2) with positive square root.
    ConstProductSqrt,
    /// The positive square root of two.
    Sqrt,
    /// The natural exponential exp(2), equal to e squared.
    Exp,
    /// The natural logarithm ln(3).
    Ln,
    /// The natural logarithm ln(2*e), mathematically 1+ln(2).
    LnAffine,
    /// The product ln(2)*ln(3).
    LnProduct,
    /// The base-ten logarithm of two.
    Log10,
    /// The base-two logarithm of three.
    Log2,
    /// The sine of pi/5, constructed natively from the exact ratio 1/5.
    SinPi,
    /// The tangent of pi/5, away from a tangent pole.
    TanPi,
    /// The sine of one radian, used as an opaque computable example.
    Irrational,
}

/// Stable iteration order for twenty fixed recipes in both benchmark groups.
///
/// The length assertion checks this catalogue size, not completeness against the
/// kernel's representation set. Pow2/Pow10 are absent; each listed recipe supplies
/// two construction/export and five prepared cases, totaling 140 registered cases.
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

/// Map each selector to a stable label and its two mathematical constructions.
///
/// Native construction preserves an exact value; the MPFR implementation evaluates
/// a rounded point of the same intended expression at the fixed precision.
impl Recipe {
    /// Stable lower-snake-case component of the Criterion benchmark ID.
    ///
    /// This is metadata, not inspection or certification of the resulting Real class.
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

    /// Construct the selected exact value using rational literals and native operations.
    ///
    /// All fixed logarithms, roots, reciprocals and tangent inputs meet their domains;
    /// construction errors panic through expect/unwrap. Constants and derived values
    /// may share existing evidence, so a new call does not promise cold global caches.
    /// No lossy conversion is performed until the caller explicitly requests one.
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

    /// Evaluate the selected expression as a nearest-rounded 192-bit MPFR point.
    ///
    /// Every primitive constant/function and composed operation rounds at the stored
    /// precision. The rational 3/2 starts from its exact binary64 value 1.5; trig recipes
    /// first round pi and divide by five before evaluating. These points are numerical
    /// references without directed error bounds or native exact-value certificates.
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

/// Register 40 construction/export and 100 prepared cases over the twenty recipes.
///
/// Construction cases include native or MPFR creation, binary64 export and temporary
/// value destruction inside iter. Native Option results are black-boxed without
/// unwrapping in that loop. Prepared setup constructs one value per backend, unwraps
/// the native float view and requires agreement within
/// 32*f64::EPSILON*max(abs(mpfr_value),1). This floating check is not exact equality
/// or proof of any particular internal representation.
///
/// Prepared cases time native/MPFR clones, repeated float exports, and native
/// certified_rational_interval(-192). Setup warms the native float view, not an
/// explicit -192 enclosure; enclosure queries can strengthen retained evidence as
/// iterations proceed. The -192 argument is computable precision, not a checked
/// uniform 192-bit relative error or interval width. The returned interval is
/// unwrapped but its endpoints are not compared with MPFR or an independent oracle.
///
/// Criterion iter includes result destruction; clones may share native graphs.
/// Throughput counts one recipe operation, regardless of algebraic cost or output
/// contract. Panics enforce the fixed fixture's setup/availability assumptions,
/// not a general completion envelope. Report aggregation runs separately afterward.
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
                            .certified_rational_interval(-192)
                            .expect("finite representative is certifiable"),
                    )
                });
            },
        );
    }
    prepared.finish();
}

criterion_group!(
    benches,
    benchmark_representations,
    benchmark_report::finish_benchmark_report
);
criterion_main!(benches);
