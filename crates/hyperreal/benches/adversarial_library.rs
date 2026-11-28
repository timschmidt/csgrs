//! Deterministic adversarial timing probes, retained history and promoted-case scoring.
//!
//! The raw probe keeps the fastest of five wall-clock calls, including result drop,
//! without checking mathematical values or unwrapping generic results. Construction,
//! approximation, conversion and declined/error outcomes are different workloads.
//! History retains maxima of these minima after lossy text interchange; scores are
//! not correctness certificates or matched-output performance comparisons.
//!
//! Report paths use the process working directory. Direct writers are unlocked and
//! non-atomic; read failures often become empty history and write failures are logged.
//! Imported labels are leaked for static lifetimes. Row-count limits do not bound
//! input files, individual evaluation work, allocation or cancellation latency.
//!
//! The Criterion callback requires --bench and suppresses --test/--list/--help.
//! The skip-report environment variable suppresses wrapper writes, not collection
//! or scoring. Criterion 0.5.1 generates undocumented benches/main functions to
//! configure the runner, call this workload and the shared report callback, and
//! print its final summary. Their item-doc gaps remain in the repository audit.

use criterion::{Criterion, black_box, criterion_group, criterion_main};
use hyperreal::{Computable, Rational, Real};
use num::bigint::{BigInt, BigUint};
use std::fs;
use std::hint;
use std::ops::Neg;
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

/// Shared Criterion result aggregator, called after this workload by the generated runner.
#[path = "support/benchmark_report.rs"]
mod benchmark_report;

/// Slow-history filename joined to the process working directory, not the manifest path.
const REPORT_NAME: &str = "slow_performers.txt";
/// Tab-separated promoted-list filename used by this writer and the companion transcendental bench.
const PROMOTED_NAME: &str = "promoted_slow_offenders.txt";
/// Working-directory Markdown file receiving the promoted-score section.
const BENCHMARKS_NAME: &str = "benchmarks.md";
/// Five sequential calls per raw probe; the minimum elapsed time is retained without result validation.
const SAMPLE_REPEATS: usize = 5;
/// Maximum retained history/table rows (1,000); not a read-size, evaluation or memory limit.
const REPORT_LIMIT: usize = 1_000;
/// Collection target of 20,000 rows, filled by generated cases after fixed cases; not a work budget.
const TARGET_CASES: usize = 20_000;
/// One lowest-ranked retained case released and at most one first-pass replacement per refresh.
const PROMOTION_ROTATION: usize = 1;
/// Desired promoted-list cap of 100; insufficient eligible replacements can leave fewer cases.
const PROMOTED_TARGET: usize = 100;
/// Literal first opening marker used to replace a score section in Markdown.
const SCORE_SECTION_BEGIN: &str = "<!-- BEGIN promoted_slow_offender_score -->";
/// Literal closing marker searched after the first opening score marker.
const SCORE_SECTION_END: &str = "<!-- END promoted_slow_offender_score -->";
/// Metadata-line prefix storing the unsigned current average in decimal nanoseconds.
const SCORE_NANOS_PREFIX: &str = "<!-- promoted_slow_score_nanos:";
/// Metadata-line prefix storing the preceding average used for the displayed delta.
const SCORE_PREVIOUS_NANOS_PREFIX: &str = "<!-- promoted_slow_previous_score_nanos:";
/// Metadata-line prefix storing the signed current-minus-previous average for the next derivative.
const SCORE_DELTA_PREFIX: &str = "<!-- promoted_slow_score_delta_nanos:";

/// One labeled timing observation or retained historical maximum.
///
/// Derived Clone copies static labels and the duration and clones the owned input
/// string. Labels identify workloads without proving their domain, outcome or
/// mathematical equivalence; imported labels may have leaked process lifetime.
#[derive(Clone)]
struct TimedCase {
    /// Static family label; imported Markdown labels are leaked allocations, not validated scalar tags.
    family: &'static str,
    /// Static operation label used for grouping and optional promotion dispatch.
    operation: &'static str,
    /// Owned human-readable input text, sometimes a generated seed plus a rational; not a general replay format.
    input: String,
    /// Raw best-of-five duration, parsed historical duration or their maximum, depending on the producer.
    nanos: u128,
}

/// Construct an exact fixture fraction, panicking on a zero denominator.
///
/// No float conversion or timer is introduced here; callers supply the domain and
/// may invoke this constructor inside their measured closures.
fn rational(n: i64, d: u64) -> Rational {
    Rational::fraction(n, d).unwrap()
}

/// Construct an exact signed-bigint fraction with an unsigned denominator.
///
/// Unwrap construction, so zero denominators panic. Integer allocation/reduction
/// cost is inherited from Rational and has no additional bound here.
fn rational_big(n: BigInt, d: BigUint) -> Rational {
    Rational::from_bigint_fraction(n, d).unwrap()
}

/// Generate a signed rational coefficient times an exact binary scale.
///
/// Use magnitude ((seed mod 997)+1)/((seed mod 251)+1), negative for odd seeds,
/// and multiply by 2^exponent. Current callers use exponents -128 through 128.
/// Negative i32::MIN cannot be negated safely and panics in the checked path;
/// large shifts also have no allocation bound. This is fixture construction.
fn scaled_rational(seed: u64, exponent: i32) -> Rational {
    let numerator_seed = i64::try_from((seed % 997) + 1).expect("small numerator fits i64");
    let denominator_seed = (seed % 251) + 1;
    let sign = if seed & 1 == 0 { 1 } else { -1 };
    if exponent >= 0 {
        rational_big(
            BigInt::from(sign * numerator_seed) << exponent,
            BigUint::from(denominator_seed),
        )
    } else {
        rational_big(
            BigInt::from(sign * numerator_seed),
            BigUint::from(denominator_seed) << usize::try_from(-exponent).unwrap(),
        )
    }
}

/// Deterministically map a seed to a nonzero signed dyadic-scaled rational.
///
/// The exponent is seed mod 193 minus 96. Wrapping products 37*seed and 53*seed
/// select numerator magnitude 1..=1009 and denominator 1..=251; odd seeds negate
/// the numerator. Exact shifts preserve the value. This finite formula is not a
/// randomness, full-domain or independent-distribution guarantee.
fn generated_rational(seed: u64) -> Rational {
    let exponent = i32::try_from(seed % 193).expect("small exponent fits i32") - 96;
    let numerator = BigInt::from(((seed.wrapping_mul(37) % 1_009) + 1) as i64);
    let numerator = if seed & 1 == 0 { numerator } else { -numerator };
    let denominator = BigUint::from((seed.wrapping_mul(53) % 251) + 1);
    if exponent >= 0 {
        rational_big(
            numerator << usize::try_from(exponent).expect("nonnegative exponent fits usize"),
            denominator,
        )
    } else {
        rational_big(
            numerator,
            denominator << usize::try_from(-exponent).expect("negative exponent fits usize"),
        )
    }
}

/// Map wrapping seed products to an exact rational in the closed range -999..=999.
///
/// The numerator is (73*seed mod 1999)-999 and denominator (97*seed mod 997)+1.
/// The result may be zero and need not lie in the unit interval; seed products wrap
/// before the modulus, so components are deterministic and correlated.
fn bounded_generated_rational(seed: u64) -> Rational {
    rational(
        i64::try_from(seed.wrapping_mul(73) % 1_999).expect("small numerator fits i64") - 999,
        (seed.wrapping_mul(97) % 997) + 1,
    )
}

/// Return the exact fixture value 999999/1000000, below one by 10^-6.
fn near_one() -> Rational {
    rational(999_999, 1_000_000)
}

/// Return the exact positive fixture value 10^-12; its decimal scale is not binary64 input.
fn tiny() -> Rational {
    rational(1, 1_000_000_000_000)
}

/// Return the exact fixture value 1+10^-12 for just-above-one domains.
fn one_plus_tiny() -> Rational {
    Rational::one() + tiny()
}

/// Wrap a rational as an exact Computable source without requesting approximation.
fn computable(r: Rational) -> Computable {
    Computable::rational(r)
}

/// Wrap a rational in the exact rational-backed Real carrier.
fn real(r: Rational) -> Real {
    Real::new(r)
}

/// Measure five calls of a closure and retain the smallest elapsed nanoseconds.
///
/// Start the clock before f(), black-box its result and drop that result before
/// reading elapsed time. Captured input setup and label construction occur outside
/// this timer, but work performed by the closure remains inside. Repeated calls can
/// warm shared caches; this is not five independent cold samples. No result/domain,
/// certainty or completion check is performed: Result/Option errors can be timed,
/// and panics propagate. No timeout or cancellation guard is installed.
fn time_case<T>(
    family: &'static str,
    operation: &'static str,
    input: String,
    mut f: impl FnMut() -> T,
) -> TimedCase {
    let mut best = u128::MAX;
    for _ in 0..SAMPLE_REPEATS {
        let start = Instant::now();
        black_box(f());
        let elapsed = start.elapsed().as_nanos();
        best = best.min(elapsed);
    }
    TimedCase {
        family,
        operation,
        input,
        nanos: best,
    }
}

/// Build 37 exact rational fixtures: eleven literals plus two seeds at 13 binary scales.
///
/// Include zero, both signs, tiny values, 355/113, +/-10^30 and scales from -128 to
/// 128. This is a fixed list without deduplication or an exhaustive scalar domain.
fn rational_inputs() -> Vec<Rational> {
    let mut inputs = vec![
        Rational::zero(),
        Rational::one(),
        rational(-1, 1),
        tiny(),
        rational(-1, 1_000_000_000_000),
        rational(7, 10),
        rational(-7, 10),
        rational(355, 113),
        rational(-355, 113),
        Rational::from_bigint(BigInt::from(10_u8).pow(30)),
        Rational::from_bigint(-BigInt::from(10_u8).pow(30)),
    ];
    for exponent in [-128, -64, -32, -16, -8, -1, 0, 1, 8, 16, 32, 64, 128] {
        inputs.push(scaled_rational(
            (exponent as i64).unsigned_abs() + 17,
            exponent,
        ));
        inputs.push(scaled_rational(
            (exponent as i64).unsigned_abs() + 83,
            exponent,
        ));
    }
    inputs
}

/// Label the 37 rational sources and append four computable constructions.
///
/// The extras are pi, e, 2^256*pi+7/5 and pi/2-2^-40. Construction can reuse global
/// constants; the returned labels and expressions are prepared outside each raw
/// probe's clock, without a cold-cache or structural-distinctness guarantee.
fn computable_inputs() -> Vec<(String, Computable)> {
    let mut inputs: Vec<(String, Computable)> = rational_inputs()
        .into_iter()
        .enumerate()
        .map(|(i, r)| (format!("rational[{i}]={r}"), computable(r)))
        .collect();
    inputs.push(("pi".to_owned(), Computable::pi()));
    inputs.push(("e".to_owned(), Computable::e()));
    inputs.push((
        "huge_pi_plus_offset".to_owned(),
        Computable::pi()
            .multiply(computable(Rational::from_bigint(BigInt::from(1_u8) << 256)))
            .add(computable(rational(7, 5))),
    ));
    inputs.push((
        "near_half_pi_minus_2^-40".to_owned(),
        Computable::pi()
            .multiply(computable(rational(1, 2)))
            .add(computable(rational_big(
                BigInt::from(-1_i8),
                BigUint::from(1_u8) << 40,
            ))),
    ));
    inputs
}

/// Label 37 rational-backed values and append six symbolic/near-boundary fixtures.
///
/// Add pi, e, positive sqrt(2), pi+10^-12, 1+10^-12 and 1-10^-6. The root result is
/// unwrapped for its valid fixed domain; no class or numerical-value assertion
/// accompanies the 43-item catalogue.
fn real_inputs() -> Vec<(String, Real)> {
    let mut inputs: Vec<(String, Real)> = rational_inputs()
        .into_iter()
        .enumerate()
        .map(|(i, r)| (format!("rational[{i}]={r}"), real(r)))
        .collect();
    inputs.push(("pi".to_owned(), Real::pi()));
    inputs.push(("e".to_owned(), Real::e()));
    inputs.push(("sqrt2".to_owned(), real(Rational::new(2)).sqrt().unwrap()));
    inputs.push(("pi_plus_tiny".to_owned(), Real::pi() + real(tiny())));
    inputs.push(("one_plus_tiny".to_owned(), real(one_plus_tiny())));
    inputs.push(("near_one".to_owned(), real(near_one())));
    inputs
}

/// Append raw rational clone, seventh-power, conversion and adjacent-pair timings.
///
/// Time each of 37 inputs, then add/multiply adjacent pairs and divide when the
/// right operand is structurally nonzero. Input/label setup is outside each raw
/// clock; powi/conversion results are not checked and no exact output is retained.
/// The repeated minimum is a latency observation, not a correctness receipt.
fn collect_rational_cases(out: &mut Vec<TimedCase>) {
    let inputs = rational_inputs();
    for (i, value) in inputs.iter().enumerate() {
        out.push(time_case(
            "rational",
            "clone",
            format!("{i}:{value}"),
            || value.clone(),
        ));
        out.push(time_case(
            "rational",
            "powi_7",
            format!("{i}:{value}"),
            || value.clone().powi(BigInt::from(7_u8)),
        ));
        out.push(time_case(
            "rational",
            "to_i64",
            format!("{i}:{value}"),
            || i64::try_from(value.clone()),
        ));
    }
    for pair in inputs.windows(2) {
        let left = pair[0].clone();
        let right = pair[1].clone();
        let input = format!("{left} | {right}");
        out.push(time_case("rational", "add", input.clone(), || {
            &left + &right
        }));
        out.push(time_case("rational", "mul", input.clone(), || {
            &left * &right
        }));
        if right.sign() != num::bigint::Sign::NoSign {
            out.push(time_case("rational", "div", input, || &left / &right));
        }
    }
}

/// Append fixed native approximation probes at -96 or -128 computable precision.
///
/// For 41 prepared sources, time approximation, trig, sqrt(x^2) and ln(x^2+1).
/// The historical ln_abs_plus_one label means ln(x^2+1), not ln(abs(x)+1).
/// Additional bounded inverse-trig/atanh, asinh and above-one acosh fixtures probe
/// endpoint/tail behavior. Requested approximation and any closure-side cloning or
/// construction are timed. Returned integers are not independently validated and
/// cancellation status is not checked; precision is not a total-work bound.
fn collect_computable_cases(out: &mut Vec<TimedCase>) {
    let inputs = computable_inputs();
    for (name, value) in &inputs {
        out.push(time_case("computable", "approx_p96", name.clone(), || {
            value.approx(-96)
        }));
        out.push(time_case("computable", "sin_p96", name.clone(), || {
            value.clone().sin().approx(-96)
        }));
        out.push(time_case("computable", "cos_p96", name.clone(), || {
            value.clone().cos().approx(-96)
        }));
        out.push(time_case("computable", "tan_p96", name.clone(), || {
            value.clone().tan().approx(-96)
        }));
        out.push(time_case("computable", "sqrt_p96", name.clone(), || {
            value.clone().square().sqrt().approx(-96)
        }));
        out.push(time_case(
            "computable",
            "ln_abs_plus_one_p96",
            name.clone(),
            || {
                value
                    .clone()
                    .square()
                    .add(Computable::one())
                    .ln()
                    .approx(-96)
            },
        ));
    }

    let inverse_inputs = [
        ("zero", Rational::zero()),
        ("tiny", tiny()),
        ("mid", rational(7, 10)),
        ("near_one", near_one()),
        ("near_minus_one", near_one().neg()),
    ];
    for (name, value) in inverse_inputs {
        let c = computable(value);
        out.push(time_case("computable", "asin_p96", name.to_owned(), || {
            c.clone().asin().approx(-96)
        }));
        out.push(time_case("computable", "acos_p96", name.to_owned(), || {
            c.clone().acos().approx(-96)
        }));
        out.push(time_case("computable", "atan_p96", name.to_owned(), || {
            c.clone().atan().approx(-96)
        }));
        out.push(time_case(
            "computable",
            "atanh_p128",
            name.to_owned(),
            || c.clone().atanh().approx(-128),
        ));
    }
    for (name, value) in [
        ("tiny", tiny()),
        ("mid", rational(1, 2)),
        ("large", Rational::new(1_000_000)),
        ("large_negative", Rational::new(-1_000_000)),
    ] {
        let c = computable(value);
        out.push(time_case(
            "computable",
            "asinh_p128",
            name.to_owned(),
            || c.clone().asinh().approx(-128),
        ));
    }
    for (name, value) in [
        ("one_plus_tiny", one_plus_tiny()),
        ("two", Rational::new(2)),
        ("large", Rational::new(1_000_000)),
    ] {
        let c = computable(value);
        out.push(time_case(
            "computable",
            "acosh_p128",
            name.to_owned(),
            || c.clone().acosh().approx(-128),
        ));
    }
}

/// Append fixed Real observations, adjacent arithmetic and stable-function constructions.
///
/// The 43-source loop times clone, facts, lossy f64 conversion, trig, sqrt(x^2)
/// and ln(x^2+1); adjacent pairs supply arithmetic. Additional cases cover tiny
/// stable formulas, exact roots/trig endpoints, tails, distribution functions,
/// geometry helpers and polynomial evaluation. Build labels do not imply zero
/// admission/refinement work. Fallible results are black-boxed without success or
/// certainty assertions; no subsequent numerical evaluation is requested by most
/// construction rows. Prepared sources/labels are outside the raw timers; inline
/// argument construction inside closures, such as hypot fixture values, is timed.
fn collect_real_cases(out: &mut Vec<TimedCase>) {
    let inputs = real_inputs();
    for (name, value) in &inputs {
        out.push(time_case("real", "clone", name.clone(), || value.clone()));
        out.push(time_case("real", "zero_status", name.clone(), || {
            value.zero_status()
        }));
        out.push(time_case("real", "structural_facts", name.clone(), || {
            value.structural_facts()
        }));
        out.push(time_case("real", "to_f64", name.clone(), || {
            f64::from(value.clone())
        }));
        out.push(time_case("real", "sin_build", name.clone(), || {
            value.clone().sin()
        }));
        out.push(time_case("real", "cos_build", name.clone(), || {
            value.clone().cos()
        }));
        out.push(time_case("real", "tan_build", name.clone(), || {
            value.clone().tan()
        }));
        out.push(time_case("real", "sqrt_build", name.clone(), || {
            (value.clone() * value.clone()).sqrt()
        }));
        out.push(time_case(
            "real",
            "ln_abs_plus_one_build",
            name.clone(),
            || ((value.clone() * value.clone()) + Real::one()).ln(),
        ));
    }
    for pair in inputs.windows(2) {
        let left = pair[0].1.clone();
        let right = pair[1].1.clone();
        let input = format!("{} | {}", pair[0].0, pair[1].0);
        out.push(time_case("real", "add", input.clone(), || &left + &right));
        out.push(time_case("real", "mul", input.clone(), || &left * &right));
        out.push(time_case("real", "div", input, || &left / &right));
    }

    let zero = Real::zero();
    let one = Real::one();
    let tiny_real = real(tiny());
    let near_one_real = real(near_one());
    let minus_tiny_real = real(-tiny());
    let large = real(Rational::new(64));
    let negative_large = real(Rational::new(-64));
    let tail = real(Rational::new(6));
    let normal_lo = real(rational(1, 10));
    let normal_hi = real(rational(100_000_001, 1_000_000_000));
    let half = real(rational(1, 2));
    let two = real(Rational::new(2));
    let three = real(Rational::new(3));
    let gamma_half = real(rational(1, 2));
    let gamma_three_half = real(rational(3, 2));
    let coeffs = vec![
        real(Rational::new(5)),
        real(rational(-7, 3)),
        Real::pi(),
        real(Rational::new(11)),
        real(rational(13, 17)),
    ];
    let den_coeffs = vec![Real::one(), real(rational(1, 5)), real(rational(1, 7))];

    for (operation, input, case) in [
        ("ln_1p_build", "tiny", tiny_real.clone()),
        ("ln_1p_build", "minus_tiny", minus_tiny_real.clone()),
        ("ln_1m_build", "tiny", tiny_real.clone()),
        ("expm1_build", "tiny", tiny_real.clone()),
        ("softplus_build", "large_positive", large.clone()),
        ("softplus_build", "large_negative", negative_large.clone()),
        ("sigmoid_build", "large_positive", large.clone()),
        ("logit_build", "near_one", near_one_real.clone()),
        ("sqrt1pm1_build", "tiny", tiny_real.clone()),
        ("sqrt1m1_build", "tiny", tiny_real.clone()),
        ("cbrt_build", "negative_perfect", real(Rational::new(-27))),
        ("root_n_build", "perfect_fourth", real(Rational::new(81))),
        (
            "pow_rational_build",
            "negative_odd_denominator",
            real(Rational::new(-8)),
        ),
        ("sin_pi_build", "one_sixth", real(rational(1, 6))),
        ("cos_pi_build", "one_fourth", real(rational(1, 4))),
        ("tan_pi_build", "one_third", real(rational(1, 3))),
        ("sinc_build", "zero", zero.clone()),
        ("sinc_build", "tiny", tiny_real.clone()),
        ("sinc_pi_build", "half", half.clone()),
        ("cosc_build", "tiny", tiny_real.clone()),
        ("erfc_build", "tail", tail.clone()),
        ("erfcx_build", "tail", tail.clone()),
        ("normal_sf_build", "tail", tail.clone()),
        ("log_normal_sf_build", "tail", tail.clone()),
        ("gamma_build", "half", gamma_half.clone()),
        ("lgamma_build", "three_half", gamma_three_half.clone()),
    ] {
        let name = input.to_owned();
        match operation {
            "ln_1p_build" => out.push(time_case("real", operation, name, || case.clone().ln_1p())),
            "ln_1m_build" => out.push(time_case("real", operation, name, || case.clone().ln_1m())),
            "expm1_build" => out.push(time_case("real", operation, name, || case.clone().expm1())),
            "softplus_build" => out.push(time_case("real", operation, name, || {
                case.clone().softplus()
            })),
            "sigmoid_build" => out.push(time_case("real", operation, name, || {
                case.clone().sigmoid()
            })),
            "logit_build" => out.push(time_case("real", operation, name, || case.clone().logit())),
            "sqrt1pm1_build" => out.push(time_case("real", operation, name, || {
                case.clone().sqrt1pm1()
            })),
            "sqrt1m1_build" => out.push(time_case("real", operation, name, || {
                case.clone().sqrt1m1()
            })),
            "cbrt_build" => out.push(time_case("real", operation, name, || case.clone().cbrt())),
            "root_n_build" => out.push(time_case("real", operation, name, || {
                case.clone().root_n(4)
            })),
            "pow_rational_build" => out.push(time_case("real", operation, name, || {
                case.clone().pow_rational(rational(1, 3))
            })),
            "sin_pi_build" => {
                out.push(time_case("real", operation, name, || case.clone().sin_pi()))
            }
            "cos_pi_build" => {
                out.push(time_case("real", operation, name, || case.clone().cos_pi()))
            }
            "tan_pi_build" => {
                out.push(time_case("real", operation, name, || case.clone().tan_pi()))
            }
            "sinc_build" => out.push(time_case("real", operation, name, || case.clone().sinc())),
            "sinc_pi_build" => out.push(time_case("real", operation, name, || {
                case.clone().sinc_pi()
            })),
            "cosc_build" => out.push(time_case("real", operation, name, || case.clone().cosc())),
            "erfc_build" => out.push(time_case("real", operation, name, || case.clone().erfc())),
            "erfcx_build" => out.push(time_case("real", operation, name, || case.clone().erfcx())),
            "normal_sf_build" => out.push(time_case("real", operation, name, || {
                case.clone().normal_sf()
            })),
            "log_normal_sf_build" => out.push(time_case("real", operation, name, || {
                case.clone().log_normal_sf()
            })),
            "gamma_build" => out.push(time_case("real", operation, name, || case.clone().gamma())),
            "lgamma_build" => {
                out.push(time_case("real", operation, name, || case.clone().lgamma()))
            }
            _ => unreachable!("all operations handled above"),
        }
    }

    out.push(time_case(
        "real",
        "logaddexp_build",
        "dominant_large_vs_tiny".to_owned(),
        || Real::logaddexp(&large, &tiny_real),
    ));
    out.push(time_case(
        "real",
        "logsubexp_build",
        "one_vs_near_one".to_owned(),
        || Real::logsubexp(&one, &near_one_real),
    ));
    out.push(time_case(
        "real",
        "atan2_axis_build",
        "zero_negative_x".to_owned(),
        || zero.clone().atan2(-one.clone()),
    ));
    out.push(time_case(
        "real",
        "atan2_quadrant_build",
        "positive_y_negative_x".to_owned(),
        || one.clone().atan2(-one.clone()),
    ));
    out.push(time_case("real", "hypot2_build", "3_4".to_owned(), || {
        Real::hypot2(&real(Rational::new(3)), &real(Rational::new(4)))
    }));
    out.push(time_case(
        "real",
        "hypot3_build",
        "2_3_6".to_owned(),
        || {
            Real::hypot3(
                &real(Rational::new(2)),
                &real(Rational::new(3)),
                &real(Rational::new(6)),
            )
        },
    ));
    out.push(time_case(
        "real",
        "hypot_minus_build",
        "large_tiny".to_owned(),
        || Real::hypot_minus(&real(Rational::new(1_000_000)), &tiny_real),
    ));
    out.push(time_case(
        "real",
        "rem_euclid_certified_build",
        "negative_rational_mod_positive".to_owned(),
        || real(rational(-17, 5)).rem_euclid_certified(&real(Rational::new(3))),
    ));
    out.push(time_case(
        "real",
        "normal_interval_build",
        "narrow".to_owned(),
        || Real::normal_interval(&normal_lo, &normal_hi),
    ));
    out.push(time_case(
        "real",
        "normal_interval_moment_build",
        "zero_one_third".to_owned(),
        || Real::normal_interval_moment(&zero, &one, 3),
    ));
    out.push(time_case(
        "real",
        "truncated_normal_mean_build",
        "zero_one".to_owned(),
        || Real::truncated_normal_mean(&zero, &one),
    ));
    out.push(time_case(
        "real",
        "regularized_beta_build",
        "2_3_half".to_owned(),
        || Real::regularized_beta(&two, &three, &half),
    ));
    out.push(time_case(
        "real",
        "regularized_gamma_p_build",
        "3_half_1".to_owned(),
        || Real::regularized_gamma_p(&gamma_three_half, &one),
    ));
    out.push(time_case(
        "real",
        "chi_square_sf_build",
        "x2_k5".to_owned(),
        || Real::chi_square_sf(&two, 5),
    ));
    out.push(time_case(
        "real",
        "eval_poly_build",
        "mixed_coeffs".to_owned(),
        || Real::eval_poly(&coeffs, &half),
    ));
    out.push(time_case(
        "real",
        "eval_rational_poly_build",
        "mixed_coeffs".to_owned(),
        || Real::eval_rational_poly(&coeffs, &den_coeffs, &half),
    ));
}

/// Append deterministic generated rows until the existing vector reaches 20,000.
///
/// Restart seed at zero and cycle through fifteen operations: rational arithmetic/
/// power, computable trig/log/square-root/atan approximations, and Real arithmetic/
/// construction. Generate both nonzero wide operands and a bounded operand each
/// iteration; format their displayed values with the seed. Right rational divisors
/// are nonzero by construction. sqrt rows use sqrt(x^2), and ln rows use ln(x^2+1).
/// Despite the name, no randomness, fuzz feedback, oracle or result assertion is
/// used. Existing rows count toward the target; already-full vectors are unchanged.
/// The limit bounds row count, not individual arithmetic work or peak allocation.
fn collect_generated_fuzz_cases(out: &mut Vec<TimedCase>) {
    let mut seed = 0_u64;
    while out.len() < TARGET_CASES {
        let left = generated_rational(seed.wrapping_mul(2).wrapping_add(1));
        let right = generated_rational(seed.wrapping_mul(2).wrapping_add(2));
        let bounded = bounded_generated_rational(seed);
        let operation = seed % 15;
        match operation {
            0 => {
                let input = format!("generated[{seed}] {left} | {right}");
                out.push(time_case("rational", "generated_add", input, || {
                    &left + &right
                }));
            }
            1 => {
                let input = format!("generated[{seed}] {left} | {right}");
                out.push(time_case("rational", "generated_mul", input, || {
                    &left * &right
                }));
            }
            2 => {
                let input = format!("generated[{seed}] {left} | {right}");
                out.push(time_case("rational", "generated_div", input, || {
                    &left / &right
                }));
            }
            3 => {
                let input = format!("generated[{seed}] {left}");
                out.push(time_case("rational", "generated_powi_5", input, || {
                    left.clone().powi(BigInt::from(5_u8))
                }));
            }
            4 => {
                let input = format!("generated[{seed}] {bounded}");
                let value = computable(bounded);
                out.push(time_case("computable", "generated_sin_p96", input, || {
                    value.clone().sin().approx(-96)
                }));
            }
            5 => {
                let input = format!("generated[{seed}] {bounded}");
                let value = computable(bounded);
                out.push(time_case("computable", "generated_cos_p96", input, || {
                    value.clone().cos().approx(-96)
                }));
            }
            6 => {
                let input = format!("generated[{seed}] {bounded}");
                let value = computable(bounded);
                out.push(time_case("computable", "generated_tan_p96", input, || {
                    value.clone().tan().approx(-96)
                }));
            }
            7 => {
                let input = format!("generated[{seed}] {bounded}");
                let value = computable(bounded);
                out.push(time_case(
                    "computable",
                    "generated_ln_abs_plus_one_p96",
                    input,
                    || {
                        value
                            .clone()
                            .square()
                            .add(Computable::one())
                            .ln()
                            .approx(-96)
                    },
                ));
            }
            8 => {
                let input = format!("generated[{seed}] {bounded}");
                let value = computable(bounded);
                out.push(time_case("computable", "generated_sqrt_p96", input, || {
                    value.clone().square().sqrt().approx(-96)
                }));
            }
            9 => {
                let input = format!("generated[{seed}] {bounded}");
                let value = computable(bounded);
                out.push(time_case("computable", "generated_atan_p96", input, || {
                    value.clone().atan().approx(-96)
                }));
            }
            10 => {
                let input = format!("generated[{seed}] {left} | {right}");
                let left = real(left);
                let right = real(right);
                out.push(time_case("real", "generated_add", input, || &left + &right));
            }
            11 => {
                let input = format!("generated[{seed}] {left} | {right}");
                let left = real(left);
                let right = real(right);
                out.push(time_case("real", "generated_mul", input, || &left * &right));
            }
            12 => {
                let input = format!("generated[{seed}] {left} | {right}");
                let left = real(left);
                let right = real(right);
                out.push(time_case("real", "generated_div", input, || &left / &right));
            }
            13 => {
                let input = format!("generated[{seed}] {bounded}");
                let value = real(bounded);
                out.push(time_case("real", "generated_sin_build", input, || {
                    value.clone().sin()
                }));
            }
            _ => {
                let input = format!("generated[{seed}] {bounded}");
                let value = real(bounded);
                out.push(time_case(
                    "real",
                    "generated_ln_abs_plus_one_build",
                    input,
                    || ((value.clone() * value.clone()) + Real::one()).ln(),
                ));
            }
        }
        seed = seed.wrapping_add(1);
    }
}

/// Run fixed rational, computable and Real probes, then fill the generated catalogue.
///
/// Truncate to 20,000 after collection and stably sort by descending raw duration.
/// The whole process includes setup, formatting, result drops and nested clocks;
/// inside its Criterion aggregate this all contributes to measured time. No
/// numerical correctness, error-result rejection or completion budget is supplied.
fn collect_all_cases() -> Vec<TimedCase> {
    let mut cases = Vec::new();
    collect_rational_cases(&mut cases);
    collect_computable_cases(&mut cases);
    collect_real_cases(&mut cases);
    collect_generated_fuzz_cases(&mut cases);
    cases.truncate(TARGET_CASES);
    cases.sort_by_key(|case| std::cmp::Reverse(case.nanos));
    cases
}

/// Format integer nanoseconds as ns or three-decimal us/ms/s at decimal thresholds.
///
/// Below 1,000, print the integer exactly. Larger values convert to f64 and round
/// text, so persisted durations can lose nanosecond precision or large-integer bits.
/// This is report formatting, not exact statistical interchange.
fn format_duration(nanos: u128) -> String {
    if nanos < 1_000 {
        format!("{nanos} ns")
    } else if nanos < 1_000_000 {
        format!("{:.3} us", nanos as f64 / 1_000.0)
    } else if nanos < 1_000_000_000 {
        format!("{:.3} ms", nanos as f64 / 1_000_000.0)
    } else {
        format!("{:.3} s", nanos as f64 / 1_000_000_000.0)
    }
}

/// Read a float and one ns/us/ms/s unit, ignoring any further whitespace tokens.
///
/// Multiply, round and cast to u128. Negative values and NaN become zero; positive
/// overflow/infinity saturate at u128::MAX. Missing/unparseable fields or unknown
/// units return None. No finiteness, sign, provenance or trailing-text check is made.
fn parse_duration_nanos(value: &str) -> Option<u128> {
    let mut parts = value.split_whitespace();
    let number = parts.next()?.parse::<f64>().ok()?;
    let unit = parts.next()?;
    let multiplier = match unit {
        "ns" => 1.0,
        "us" => 1_000.0,
        "ms" => 1_000_000.0,
        "s" => 1_000_000_000.0,
        _ => return None,
    };
    Some((number * multiplier).round() as u128)
}

/// Return text between the first two backticks, or None if either delimiter is absent; no escaping is decoded.
fn extract_tick_value(value: &str) -> Option<String> {
    let start = value.find('`')? + 1;
    let end = value[start..].find('`')? + start;
    Some(value[start..end].to_owned())
}

/// Join REPORT_NAME to current_dir; despite the name, no crate/manifest location is consulted. Panic if cwd lookup fails.
fn crate_report_path() -> PathBuf {
    std::env::current_dir()
        .expect("bench runner has a current directory")
        .join(REPORT_NAME)
}

/// Join PROMOTED_NAME to current_dir; callers choose cwd, and lookup failure panics.
fn crate_promoted_path() -> PathBuf {
    std::env::current_dir()
        .expect("bench runner has a current directory")
        .join(PROMOTED_NAME)
}

/// Join BENCHMARKS_NAME to current_dir, without checking file existence or crate ownership; lookup failure panics.
fn crate_benchmarks_path() -> PathBuf {
    std::env::current_dir()
        .expect("bench runner has a current directory")
        .join(BENCHMARKS_NAME)
}

/// Extract rows from the simple Markdown table, treating every read error as empty history.
///
/// Split lines on literal pipes, reject any line containing --- or Rank (even in
/// input text), and parse duration plus
/// backtick fields. Embedded pipes are not escaped by the writer and can make its
/// own rows unreadable. Generated labels without a space are skipped. No family or
/// operation validation is performed. Accepted family/operation strings are leaked
/// to obtain static lifetimes; reading a report is not bounded by REPORT_LIMIT.
fn read_existing_cases(report_path: &Path) -> Vec<TimedCase> {
    let Ok(report) = fs::read_to_string(report_path) else {
        return Vec::new();
    };
    let mut cases = Vec::new();
    for line in report.lines() {
        if !line.starts_with('|') || line.contains("---") || line.contains("Rank") {
            continue;
        }
        let columns: Vec<_> = line.split('|').map(str::trim).collect();
        if columns.len() < 6 {
            continue;
        }
        let Some(nanos) = parse_duration_nanos(columns[2]) else {
            continue;
        };
        let Some(family) = extract_tick_value(columns[3]) else {
            continue;
        };
        let Some(operation) = extract_tick_value(columns[4]) else {
            continue;
        };
        let Some(input) = extract_tick_value(columns[5]) else {
            continue;
        };
        if input.starts_with("generated[") && !input.contains(' ') {
            continue;
        }
        cases.push(TimedCase {
            family: Box::leak(family.into_boxed_str()),
            operation: Box::leak(operation.into_boxed_str()),
            input,
            nanos,
        });
    }
    cases
}

/// Merge parsed history and current rows by exact family/operation/input text.
///
/// For each equal key keep the maximum duration, then stably sort descending and
/// retain at most 1,000 rows. Equal-duration ordering follows the preceding key
/// sort. These are maxima of parsed history and fresh probe minima, not a full
/// worst-ever record: prior truncation, failed parsing and rounded text can lose
/// observations. Imported labels remain leaked even when their rows are discarded.
fn merge_historical_cases(report_path: &Path, current: &[TimedCase]) -> Vec<TimedCase> {
    let mut merged = read_existing_cases(report_path);
    merged.extend_from_slice(current);
    merged.sort_by(|left, right| {
        (left.family, left.operation, left.input.as_str()).cmp(&(
            right.family,
            right.operation,
            right.input.as_str(),
        ))
    });

    let mut deduped: Vec<TimedCase> = Vec::new();
    for case in merged {
        if let Some(last) = deduped.last_mut()
            && last.family == case.family
            && last.operation == case.operation
            && last.input == case.input
        {
            last.nanos = last.nanos.max(case.nanos);
            continue;
        }
        deduped.push(case);
    }
    deduped.sort_by_key(|case| std::cmp::Reverse(case.nanos));
    deduped.truncate(REPORT_LIMIT);
    deduped
}

/// Recognize computable generated log/atan/tan/sin/cos rows with parseable inputs.
///
/// The log operation is ln(x^2+1). This checks labels and textual parseability, not
/// agreement between seed and value, mathematical output correctness or cost.
/// Zero denominators can panic in the parser instead of returning false.
fn is_promotable(case: &TimedCase) -> bool {
    case.family == "computable"
        && matches!(
            case.operation,
            "generated_ln_abs_plus_one_p96"
                | "generated_atan_p96"
                | "generated_tan_p96"
                | "generated_sin_p96"
                | "generated_cos_p96"
        )
        && parse_generated_input(&case.input).is_some()
}

/// Parse the exact prefix `generated[seed]` followed by a space and limited rational text.
///
/// Require a u64 seed, but do not regenerate/validate the value from it. Delegate
/// numeric parsing, including its zero-denominator panic; malformed text otherwise
/// returns None. This is not the general Rational or native-geometry wire grammar.
fn parse_generated_input(input: &str) -> Option<(u64, Rational)> {
    let rest = input.strip_prefix("generated[")?;
    let (seed, rational) = rest.split_once("] ")?;
    let seed = seed.parse().ok()?;
    let rational = parse_generated_rational_text(rational)?;
    Some((seed, rational))
}

/// Parse an i64 integer, i64/u64 fraction or signed mixed number used by reports.
///
/// For a negative whole part, subtract the parsed fraction; otherwise add it.
/// No canonical-sign check is made: textual -0 loses its sign, and signed fraction
/// numerators are accepted. Fraction construction unwraps and panics on denominator
/// zero even though syntax failures return None. Large integers outside these
/// machine types, decimals and pair inputs are not admitted.
fn parse_generated_rational_text(value: &str) -> Option<Rational> {
    if let Some((whole, fraction)) = value.split_once(' ') {
        let whole = whole.parse::<i64>().ok()?;
        let (numerator, denominator) = fraction.split_once('/')?;
        let numerator = numerator.parse::<i64>().ok()?;
        let denominator = denominator.parse::<u64>().ok()?;
        let fraction = rational(numerator, denominator);
        return Some(if whole < 0 {
            Rational::new(whole) - fraction
        } else {
            Rational::new(whole) + fraction
        });
    }
    if let Some((numerator, denominator)) = value.split_once('/') {
        return Some(rational(
            numerator.parse::<i64>().ok()?,
            denominator.parse::<u64>().ok()?,
        ));
    }
    Some(Rational::new(value.parse::<i64>().ok()?))
}

/// Use operation and exact input text as the promoted identity; family is deliberately absent from this key.
fn promoted_case_key(case: &TimedCase) -> (&str, &str) {
    (case.operation, case.input.as_str())
}

/// Read tab-separated operation/input/u128 timing rows, then keep promotable ones.
///
/// Read errors become an empty list; blank/comment lines are skipped. A fourth
/// name field is optional and ignored here, though the companion consumer requires
/// it. Imported operations are leaked before eligibility filtering; duplicates are
/// retained, file size/row count are unbounded, and malformed zero denominators can
/// panic through the rational parser.
fn read_promoted_cases(promoted_path: &Path) -> Vec<TimedCase> {
    let Ok(contents) = fs::read_to_string(promoted_path) else {
        return Vec::new();
    };
    contents
        .lines()
        .filter(|line| !line.trim().is_empty() && !line.trim_start().starts_with('#'))
        .filter_map(|line| {
            let mut parts = line.splitn(4, '\t');
            let operation = parts.next()?.to_owned();
            let input = parts.next()?.to_owned();
            let nanos = parts.next()?.parse::<u128>().ok()?;
            let _name = parts.next();
            Some(TimedCase {
                family: "computable",
                operation: Box::leak(operation.into_boxed_str()),
                input,
                nanos,
            })
        })
        .filter(is_promotable)
        .collect()
}

/// Reprobe a recognized row and retain the maximum of its old and fresh minimum.
///
/// Return None on unsupported text/operation; parser/evaluation panics propagate.
/// This preserves historical slow scores for rotation, not current latency alone.
fn time_promoted_case(case: &TimedCase) -> Option<TimedCase> {
    let timed = time_promoted_case_current(case)?;
    Some(TimedCase {
        nanos: timed.nanos.max(case.nanos),
        ..timed
    })
}

/// Time one of five parsed computable operations afresh at precision -96.
///
/// Ignore the input family and seed/value consistency; use the parsed rational to
/// build ln(x^2+1), atan, tan, sin or cos. Prepare the source before timing but clone,
/// construct and approximate inside each of five calls. Unknown operations or text
/// return None, while construction/parser panics propagate. Do not retain the old
/// duration or validate approximation values/cancellation here.
fn time_promoted_case_current(case: &TimedCase) -> Option<TimedCase> {
    let (_, rational) = parse_generated_input(&case.input)?;
    let operation = case.operation;
    let input = case.input.clone();
    let value = computable(rational);
    let timed = match operation {
        "generated_ln_abs_plus_one_p96" => time_case("computable", operation, input, || {
            value
                .clone()
                .square()
                .add(Computable::one())
                .ln()
                .approx(-96)
        }),
        "generated_atan_p96" => time_case("computable", operation, input, || {
            value.clone().atan().approx(-96)
        }),
        "generated_tan_p96" => time_case("computable", operation, input, || {
            value.clone().tan().approx(-96)
        }),
        "generated_sin_p96" => time_case("computable", operation, input, || {
            value.clone().sin().approx(-96)
        }),
        "generated_cos_p96" => time_case("computable", operation, input, || {
            value.clone().cos().approx(-96)
        }),
        _ => return None,
    };
    Some(timed)
}

/// Build a companion Criterion label from a recognized operation, seed and rational.
///
/// Normalize the rational through Display, encode minus as neg_, spaces/slashes as
/// underscores and other nonalphanumeric characters as underscores; append _p96.
/// This is not a collision-proof or general reversible encoding. Family is ignored;
/// unsupported operations/text return None, with parser panics still possible.
fn promoted_bench_name(case: &TimedCase) -> Option<String> {
    let (seed, rational) = parse_generated_input(&case.input)?;
    let op = match case.operation {
        "generated_ln_abs_plus_one_p96" => "ln",
        "generated_atan_p96" => "atan",
        "generated_tan_p96" => "tan",
        "generated_sin_p96" => "sin",
        "generated_cos_p96" => "cos",
        _ => return None,
    };
    let rational = rational.to_string();
    let mut name = String::new();
    name.push_str(op);
    name.push_str("_generated_");
    name.push_str(&seed.to_string());
    name.push('_');
    for ch in rational.chars() {
        match ch {
            '-' => name.push_str("neg_"),
            ' ' | '/' => name.push('_'),
            c if c.is_ascii_alphanumeric() => name.push(c),
            _ => name.push('_'),
        }
    }
    name.push_str("_p96");
    Some(name)
}

/// Reprobe promoted history, release its fastest retained-score row, then refill.
///
/// Scores are maxima of prior and new best-of-five timings. Prefer the first
/// eligible historical candidate not already retained or just released, then
/// backfill to 100 in supplied history order. The usual caller supplies descending
/// history; this helper does not sort candidates before selection. Released keys
/// cannot reenter in this refresh, so a sparse set can shrink. Existing duplicates
/// are not removed. Finally sort descending and truncate to 100; this function
/// returns rows without writing the file.
fn rotate_promoted_cases(promoted_path: &Path, historical: &[TimedCase]) -> Vec<TimedCase> {
    let mut retained: Vec<_> = read_promoted_cases(promoted_path)
        .into_iter()
        .filter_map(|case| time_promoted_case(&case))
        .collect();
    retained.sort_by_key(|case| std::cmp::Reverse(case.nanos));
    let removed = retained.len().min(PROMOTION_ROTATION);
    let released = if removed > 0 {
        retained.split_off(retained.len() - removed)
    } else {
        Vec::new()
    };

    let mut added = 0;
    for candidate in historical.iter().filter(|case| is_promotable(case)) {
        if retained
            .iter()
            .any(|case| promoted_case_key(case) == promoted_case_key(candidate))
            || released
                .iter()
                .any(|case| promoted_case_key(case) == promoted_case_key(candidate))
        {
            continue;
        }
        retained.push(candidate.clone());
        added += 1;
        if added == PROMOTION_ROTATION {
            break;
        }
    }

    for candidate in historical.iter().filter(|case| is_promotable(case)) {
        if retained.len() >= PROMOTED_TARGET {
            break;
        }
        if retained
            .iter()
            .any(|case| promoted_case_key(case) == promoted_case_key(candidate))
            || released
                .iter()
                .any(|case| promoted_case_key(case) == promoted_case_key(candidate))
        {
            continue;
        }
        retained.push(candidate.clone());
    }

    retained.sort_by_key(|case| std::cmp::Reverse(case.nanos));
    retained.truncate(PROMOTED_TARGET);
    retained
}

/// Write a fresh tab-separated promoted list, omitting rows without a generated name.
///
/// Use the supplied order and durations, with operation/input text unescaped. The
/// fourth name supports the companion benchmark loader. The direct unlocked write
/// is non-atomic and logs errors without propagating failure; this helper does not
/// honor the skip-report variable or enforce the advertised 100-entry count.
fn write_promoted_cases(promoted_path: &Path, cases: &[TimedCase]) {
    let mut out = String::new();
    out.push_str("# Generated by `cargo bench --bench adversarial_library`.\n");
    out.push_str("# Each report refresh promotes the worst promotable historical offender and releases the fastest regular promoted case.\n");
    out.push_str("# The regular promoted set is backfilled from worst historical offenders until it reaches 100 entries when enough promotable cases exist.\n");
    out.push_str("# Format: operation<TAB>input<TAB>worst_nanos<TAB>criterion_name\n");
    for case in cases {
        let Some(name) = promoted_bench_name(case) else {
            continue;
        };
        out.push_str(case.operation);
        out.push('\t');
        out.push_str(&case.input);
        out.push('\t');
        out.push_str(&case.nanos.to_string());
        out.push('\t');
        out.push_str(&name);
        out.push('\n');
    }
    if let Err(error) = fs::write(promoted_path, out) {
        eprintln!(
            "failed to write promoted slow offender list {}: {error}",
            promoted_path.display()
        );
    }
}

/// Stored/current mean comparison with the number of eligible timed rows.
///
/// Derived Clone/Copy duplicate integer fields; Debug prints them without checking
/// arithmetic or workload comparability. Delta and its change can compare different
/// rosters, and this record carries no correctness or statistical confidence proof.
#[derive(Clone, Copy, Debug)]
struct PromotedScore {
    /// Current timed row count; metadata loading instead substitutes the target 100.
    cases: usize,
    /// Previous stored mean, or current mean when no usable predecessor exists.
    previous_nanos: u128,
    /// Current integer-truncated mean of best-of-five probes, or a parsed stored mean.
    average_nanos: u128,
    /// Current mean minus previous mean in nanoseconds; unchecked signed arithmetic can overflow.
    delta_nanos: i128,
    /// Change from the previously recorded delta; not divided by elapsed time and not a physical derivative.
    derivative_nanos: i128,
}

/// Return the integer-truncated mean of a nonempty slice's stored durations.
///
/// Sum with saturating u128 addition before division; on overflow this understates
/// the mathematical mean. Empty input returns None. No statistical weighting,
/// outlier handling or completion/certainty validation is supplied.
fn average_nanos(cases: &[TimedCase]) -> Option<u128> {
    if cases.is_empty() {
        return None;
    }
    let total = cases
        .iter()
        .fold(0_u128, |sum, case| sum.saturating_add(case.nanos));
    Some(total / cases.len() as u128)
}

/// Reprobe every eligible file row and compare its current mean with stored score metadata.
///
/// Do not retain historic timing maxima here. Sort current rows descending and
/// average their minima; an empty set returns None. Duplicates and more than 100
/// rows can be scored. Read previous average/delta from working-directory Markdown;
/// if unavailable, use current average and zero delta. Compute current-minus-prior
/// and its change, not a time derivative. Casts to i128 and signed subtraction are
/// unchecked, so extreme values can wrap or overflow/panic by build mode.
/// Roster, hardware and cache comparability are not established; no files are written.
fn score_promoted_cases(promoted_path: &Path) -> Option<(PromotedScore, Vec<TimedCase>)> {
    let mut timed: Vec<_> = read_promoted_cases(promoted_path)
        .iter()
        .filter_map(time_promoted_case_current)
        .collect();
    timed.sort_by_key(|case| std::cmp::Reverse(case.nanos));
    let average_nanos = average_nanos(&timed)?;
    let previous = read_previous_promoted_score(&crate_benchmarks_path());
    let previous_score = previous
        .map(|score| score.average_nanos)
        .unwrap_or(average_nanos);
    let previous_delta = previous.map(|score| score.delta_nanos).unwrap_or(0);
    let delta_nanos = average_nanos as i128 - previous_score as i128;
    let derivative_nanos = delta_nanos - previous_delta;
    Some((
        PromotedScore {
            cases: timed.len(),
            previous_nanos: previous_score,
            average_nanos,
            delta_nanos,
            derivative_nanos,
        },
        timed,
    ))
}

/// Return the first parseable signed integer after a trimmed prefix and before -->.
///
/// Scan all lines, ignoring malformed candidates. No section scope, uniqueness or
/// metadata provenance is checked, so unrelated matching lines can supply a value.
fn parse_metadata_i128(contents: &str, prefix: &str) -> Option<i128> {
    contents.lines().find_map(|line| {
        let value = line.trim().strip_prefix(prefix)?.trim();
        let value = value.strip_suffix("-->")?.trim();
        value.parse().ok()
    })
}

/// Load score metadata, returning None on file/read/current-score or unsigned conversion failure.
///
/// Missing/malformed previous average defaults to current; missing delta defaults
/// to zero. Negative averages are rejected by u128 conversion. Synthesize cases=100
/// and derivative=0 rather than reading them. The actual prior roster and its
/// comparability with current measurements are not recovered.
fn read_previous_promoted_score(benchmarks_path: &Path) -> Option<PromotedScore> {
    let contents = fs::read_to_string(benchmarks_path).ok()?;
    let average_nanos = parse_metadata_i128(&contents, SCORE_NANOS_PREFIX)?;
    let previous_nanos =
        parse_metadata_i128(&contents, SCORE_PREVIOUS_NANOS_PREFIX).unwrap_or(average_nanos);
    let delta_nanos = parse_metadata_i128(&contents, SCORE_DELTA_PREFIX).unwrap_or(0);
    Some(PromotedScore {
        cases: PROMOTED_TARGET,
        previous_nanos: previous_nanos.try_into().ok()?,
        average_nanos: average_nanos.try_into().ok()?,
        delta_nanos,
        derivative_nanos: 0,
    })
}

/// Format a signed nanosecond delta with a minus for negatives and no explicit plus; unsigned_abs also handles i128::MIN.
fn format_signed_duration(nanos: i128) -> String {
    if nanos < 0 {
        format!("-{}", format_duration(nanos.unsigned_abs()))
    } else {
        format_duration(nanos as u128)
    }
}

/// Render metadata, score metrics and the first ten supplied rows between fixed markers.
///
/// The prose always says 100 offenders, but the metric uses score.cases and no
/// length/score consistency is checked. Only input backticks are replaced with
/// apostrophes; pipes and other Markdown syntax are unescaped. Row order is trusted.
/// Durations are rounded report text, while decimal metadata stores the integers.
fn promoted_score_section(score: PromotedScore, timed: &[TimedCase]) -> String {
    let mut out = String::new();
    out.push_str(SCORE_SECTION_BEGIN);
    out.push('\n');
    out.push_str("## `promoted_slow_offender_score`\n\n");
    out.push_str("Average timing score for the current 100 promoted slow offenders. The score is the average current best-of-five wall-clock probe across the promoted set; lower is better. Delta compares with the previous score recorded in this file, and derivative is the change in delta.\n\n");
    out.push_str(&format!(
        "{} {} -->\n",
        SCORE_NANOS_PREFIX, score.average_nanos
    ));
    out.push_str(&format!(
        "{} {} -->\n",
        SCORE_PREVIOUS_NANOS_PREFIX, score.previous_nanos
    ));
    out.push_str(&format!(
        "{} {} -->\n\n",
        SCORE_DELTA_PREFIX, score.delta_nanos
    ));
    out.push_str("| Metric | Value |\n");
    out.push_str("| --- | ---: |\n");
    out.push_str(&format!("| Cases scored | {} |\n", score.cases));
    out.push_str(&format!(
        "| Average score | {} |\n",
        format_duration(score.average_nanos)
    ));
    out.push_str(&format!(
        "| Delta | {} |\n",
        format_signed_duration(score.delta_nanos)
    ));
    out.push_str(&format!(
        "| Delta derivative | {} |\n\n",
        format_signed_duration(score.derivative_nanos)
    ));
    out.push_str("| Rank | Current Time | Operation | Input |\n");
    out.push_str("| ---: | ---: | --- | --- |\n");
    for (rank, case) in timed.iter().take(10).enumerate() {
        out.push_str(&format!(
            "| {} | {} | `{}` | `{}` |\n",
            rank + 1,
            format_duration(case.nanos),
            case.operation,
            case.input.replace('`', "'")
        ));
    }
    out.push('\n');
    out.push_str(SCORE_SECTION_END);
    out.push('\n');
    out
}

/// Replace the first complete score marker pair, preserving all surrounding bytes.
///
/// If the opening marker or its following close is missing, prepend the new section
/// and retain the entire old document, including any malformed opening marker.
/// For a matched pair, the new section's final newline and the old suffix are both
/// kept; repeated replacement can accumulate blank lines. No uniqueness, Markdown
/// validation or bytewise idempotence is guaranteed.
fn replace_section(contents: &str, section: &str) -> String {
    let Some(start) = contents.find(SCORE_SECTION_BEGIN) else {
        let mut out = String::new();
        out.push_str(section);
        out.push('\n');
        out.push_str(contents);
        return out;
    };
    let Some(relative_end) = contents[start..].find(SCORE_SECTION_END) else {
        let mut out = String::new();
        out.push_str(section);
        out.push('\n');
        out.push_str(contents);
        return out;
    };
    let end = start + relative_end + SCORE_SECTION_END.len();
    let mut out = String::new();
    out.push_str(&contents[..start]);
    out.push_str(section);
    out.push_str(&contents[end..]);
    out
}

/// Write the score section into working-directory Markdown unless the skip variable exists.
///
/// Even an empty skip value suppresses work. Otherwise any read failure substitutes
/// an empty document, then direct replacement/write occurs without locking or atomic
/// rename. Write errors are logged and not propagated; no --bench argument is required
/// by this helper itself.
fn update_benchmarks_score(score: PromotedScore, timed: &[TimedCase]) {
    if std::env::var_os("HYPERREAL_SKIP_BENCHMARK_REPORTS").is_some() {
        return;
    }
    let path = crate_benchmarks_path();
    let section = promoted_score_section(score, timed);
    let contents = fs::read_to_string(&path).unwrap_or_default();
    let updated = replace_section(&contents, &section);
    if let Err(error) = fs::write(&path, updated) {
        eprintln!(
            "failed to write promoted slow offender score {}: {error}",
            path.display()
        );
    }
}

/// Append at most 1,000 ranked rows in supplied order under a Worst Time heading.
///
/// Replace input backticks with apostrophes, but do not escape embedded pipes or
/// validate label/timing meaning. The simple history reader can therefore discard
/// rows emitted here. Rank is positional and duration text can lose precision.
fn write_case_table(out: &mut String, cases: &[TimedCase]) {
    out.push_str("| Rank | Worst Time | Family | Operation | Input |\n");
    out.push_str("| ---: | ---: | --- | --- | --- |\n");
    for (rank, case) in cases.iter().take(REPORT_LIMIT).enumerate() {
        out.push_str(&format!(
            "| {} | {} | `{}` | `{}` | `{}` |\n",
            rank + 1,
            format_duration(case.nanos),
            case.family,
            case.operation,
            case.input.replace('`', "'")
        ));
    }
}

/// Merge history, rotate/write promoted cases, then overwrite the requested history report.
///
/// The promoted path always comes from cwd, even when report_path points elsewhere.
/// Rotation can run new probes; the helper ignores the skip-report environment flag.
/// It writes promoted data before the report, without a transaction or rollback.
/// Read failures may discard history; direct write errors are logged rather than
/// propagated. The worst-ever wording describes retained parsed maxima, not all
/// past samples or a correctness-qualified performance record.
fn write_report_to(report_path: &Path, title: &str, cases: &[TimedCase]) {
    let historical = merge_historical_cases(report_path, cases);
    let promoted = rotate_promoted_cases(&crate_promoted_path(), &historical);
    write_promoted_cases(&crate_promoted_path(), &promoted);
    let mut out = String::new();
    out.push_str("# ");
    out.push_str(title);
    out.push_str("\n\n");
    out.push_str("Generated by `cargo bench --bench adversarial_library`. Timings are best-of-five wall-clock probes for deterministic adversarial cases, intended to identify edges for promotion into focused Criterion rows.\n\n");
    out.push_str(&format!(
        "Latest run sampled {} deterministic cases. The table below intentionally merges the latest run with previous crate-local history by `family + operation + input`, updates each case to its worst observed time, and keeps only the {} worst cases. This report is a worst-ever edge finder, not a current-run leaderboard.\n\n",
        cases.len(),
        REPORT_LIMIT
    ));
    out.push_str("## Worst Performers\n\n");
    write_case_table(&mut out, &historical);
    out.push('\n');
    out.push_str("## Query Hints\n\n");
    out.push_str("- Search by family, operation, or literal input fragment.\n");
    out.push_str(
        "- Promote repeated top offenders into dedicated Criterion rows before optimizing.\n",
    );
    out.push_str("- Keep pathological panics/overflows as named tests once isolated.\n");
    out.push_str("- `promoted_slow_offenders.txt` rotates the regular promoted Criterion group: each report refresh promotes the worst promotable historical offender, releases the fastest promoted case, and backfills to 100 regular entries from the worst-offenders history.\n");

    if let Err(error) = fs::write(report_path, out) {
        eprintln!(
            "failed to write slow performer report {}: {error}",
            report_path.display()
        );
    }
}

/// Refresh the working-directory history/promoted files unless the skip variable is present.
///
/// Presence, including an empty value, returns immediately. Otherwise delegate to
/// the direct writer; no CLI guard or typed success result is supplied here.
fn write_report(cases: &[TimedCase]) {
    if std::env::var_os("HYPERREAL_SKIP_BENCHMARK_REPORTS").is_some() {
        return;
    }
    write_report_to(
        &crate_report_path(),
        "Hyperreal Slow Performer History",
        cases,
    );
}

/// Collect/update once, then register aggregate collection and current-score timings.
///
/// Return unless --bench is present, or if --test/--list/--help is present. Initial
/// collection, history/promotion writing and score update precede Criterion timing.
/// Skip-report presence suppresses wrapper writes but not collection/scoring.
///
/// Each collection iteration repeats all 20,000 raw best-of-five probes, setup,
/// formatting, sorting and drops; only the row count is black-boxed. Score iterations
/// reread promoted inputs and prior metadata, reprobe and drop their rows. Empty/
/// unreadable/unsupported promoted sets become a timed zero sentinel, not a completed
/// 100-case success. Neither aggregate validates scalar outputs. Ten samples and
/// 100ms warm-up/400ms measurement settings are statistical scheduling, not hard
/// work/memory bounds. Panics propagate, while report-write errors only log.
fn bench_adversarial_library(c: &mut Criterion) {
    let args: Vec<_> = std::env::args().collect();
    if !args.iter().any(|arg| arg == "--bench")
        || args
            .iter()
            .any(|arg| arg == "--test" || arg == "--list" || arg == "--help")
    {
        return;
    }
    let cases = collect_all_cases();
    write_report(&cases);
    if let Some((score, timed)) = score_promoted_cases(&crate_promoted_path()) {
        update_benchmarks_score(score, &timed);
    }

    let mut group = c.benchmark_group("adversarial_library_fuzz");
    group.sample_size(10);
    group.warm_up_time(Duration::from_millis(100));
    group.measurement_time(Duration::from_millis(400));
    group.bench_function("collect_slow_performer_history", |b| {
        b.iter(|| {
            let cases = collect_all_cases();
            hint::black_box(cases.len())
        })
    });
    group.finish();

    let mut score_group = c.benchmark_group("promoted_slow_offender_score");
    score_group.sample_size(10);
    score_group.warm_up_time(Duration::from_millis(100));
    score_group.measurement_time(Duration::from_millis(400));
    score_group.bench_function("score_promoted_100", |b| {
        b.iter(|| {
            let Some((score, timed)) = score_promoted_cases(&crate_promoted_path()) else {
                return 0_u128;
            };
            hint::black_box((score.average_nanos, timed.len()));
            score.average_nanos
        })
    });
    score_group.finish();
}

criterion_group!(
    benches,
    bench_adversarial_library,
    benchmark_report::finish_benchmark_report
);
criterion_main!(benches);
