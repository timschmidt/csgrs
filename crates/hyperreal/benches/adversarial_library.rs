use criterion::{Criterion, black_box, criterion_group, criterion_main};
use hyperreal::{Computable, Rational, Real};
use num::bigint::{BigInt, BigUint};
use std::fs;
use std::hint;
use std::ops::Neg;
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

const REPORT_NAME: &str = "slow_performers.txt";
const PROMOTED_NAME: &str = "promoted_slow_offenders.txt";
const BENCHMARKS_NAME: &str = "benchmarks.md";
const SAMPLE_REPEATS: usize = 5;
const REPORT_LIMIT: usize = 1_000;
const TARGET_CASES: usize = 20_000;
const PROMOTION_ROTATION: usize = 1;
const PROMOTED_TARGET: usize = 100;
const SCORE_SECTION_BEGIN: &str = "<!-- BEGIN promoted_slow_offender_score -->";
const SCORE_SECTION_END: &str = "<!-- END promoted_slow_offender_score -->";
const SCORE_NANOS_PREFIX: &str = "<!-- promoted_slow_score_nanos:";
const SCORE_PREVIOUS_NANOS_PREFIX: &str = "<!-- promoted_slow_previous_score_nanos:";
const SCORE_DELTA_PREFIX: &str = "<!-- promoted_slow_score_delta_nanos:";

#[derive(Clone)]
struct TimedCase {
    family: &'static str,
    operation: &'static str,
    input: String,
    nanos: u128,
}

fn rational(n: i64, d: u64) -> Rational {
    Rational::fraction(n, d).unwrap()
}

fn rational_big(n: BigInt, d: BigUint) -> Rational {
    Rational::from_bigint_fraction(n, d).unwrap()
}

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

fn bounded_generated_rational(seed: u64) -> Rational {
    rational(
        i64::try_from(seed.wrapping_mul(73) % 1_999).expect("small numerator fits i64") - 999,
        (seed.wrapping_mul(97) % 997) + 1,
    )
}

fn near_one() -> Rational {
    rational(999_999, 1_000_000)
}

fn tiny() -> Rational {
    rational(1, 1_000_000_000_000)
}

fn one_plus_tiny() -> Rational {
    Rational::one() + tiny()
}

fn computable(r: Rational) -> Computable {
    Computable::rational(r)
}

fn real(r: Rational) -> Real {
    Real::new(r)
}

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
            f64::try_from(value.clone())
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
}

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

fn collect_all_cases() -> Vec<TimedCase> {
    let mut cases = Vec::new();
    collect_rational_cases(&mut cases);
    collect_computable_cases(&mut cases);
    collect_real_cases(&mut cases);
    collect_generated_fuzz_cases(&mut cases);
    cases.truncate(TARGET_CASES);
    cases.sort_by(|left, right| right.nanos.cmp(&left.nanos));
    cases
}

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

fn extract_tick_value(value: &str) -> Option<String> {
    let start = value.find('`')? + 1;
    let end = value[start..].find('`')? + start;
    Some(value[start..end].to_owned())
}

fn crate_report_path() -> PathBuf {
    std::env::current_dir()
        .expect("bench runner has a current directory")
        .join(REPORT_NAME)
}

fn crate_promoted_path() -> PathBuf {
    std::env::current_dir()
        .expect("bench runner has a current directory")
        .join(PROMOTED_NAME)
}

fn crate_benchmarks_path() -> PathBuf {
    std::env::current_dir()
        .expect("bench runner has a current directory")
        .join(BENCHMARKS_NAME)
}

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
    deduped.sort_by(|left, right| right.nanos.cmp(&left.nanos));
    deduped.truncate(REPORT_LIMIT);
    deduped
}

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

fn parse_generated_input(input: &str) -> Option<(u64, Rational)> {
    let rest = input.strip_prefix("generated[")?;
    let (seed, rational) = rest.split_once("] ")?;
    let seed = seed.parse().ok()?;
    let rational = parse_generated_rational_text(rational)?;
    Some((seed, rational))
}

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

fn promoted_case_key(case: &TimedCase) -> (&str, &str) {
    (case.operation, case.input.as_str())
}

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

fn time_promoted_case(case: &TimedCase) -> Option<TimedCase> {
    let timed = time_promoted_case_current(case)?;
    Some(TimedCase {
        nanos: timed.nanos.max(case.nanos),
        ..timed
    })
}

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

fn rotate_promoted_cases(promoted_path: &Path, historical: &[TimedCase]) -> Vec<TimedCase> {
    let mut retained: Vec<_> = read_promoted_cases(promoted_path)
        .into_iter()
        .filter_map(|case| time_promoted_case(&case))
        .collect();
    retained.sort_by(|left, right| right.nanos.cmp(&left.nanos));
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

    retained.sort_by(|left, right| right.nanos.cmp(&left.nanos));
    retained.truncate(PROMOTED_TARGET);
    retained
}

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

#[derive(Clone, Copy, Debug)]
struct PromotedScore {
    cases: usize,
    previous_nanos: u128,
    average_nanos: u128,
    delta_nanos: i128,
    derivative_nanos: i128,
}

fn average_nanos(cases: &[TimedCase]) -> Option<u128> {
    if cases.is_empty() {
        return None;
    }
    let total = cases
        .iter()
        .fold(0_u128, |sum, case| sum.saturating_add(case.nanos));
    Some(total / cases.len() as u128)
}

fn score_promoted_cases(promoted_path: &Path) -> Option<(PromotedScore, Vec<TimedCase>)> {
    let mut timed: Vec<_> = read_promoted_cases(promoted_path)
        .iter()
        .filter_map(time_promoted_case_current)
        .collect();
    timed.sort_by(|left, right| right.nanos.cmp(&left.nanos));
    let average_nanos = average_nanos(&timed)?;
    let previous = read_previous_promoted_score(&crate_benchmarks_path());
    let previous_score = previous
        .map(|score| score.previous_nanos)
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

fn parse_metadata_i128(contents: &str, prefix: &str) -> Option<i128> {
    for line in contents.lines() {
        let value = line.trim().strip_prefix(prefix)?.trim();
        let value = value.strip_suffix("-->")?.trim();
        return value.parse().ok();
    }
    None
}

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

fn format_signed_duration(nanos: i128) -> String {
    if nanos < 0 {
        format!("-{}", format_duration(nanos.unsigned_abs()))
    } else {
        format_duration(nanos as u128)
    }
}

fn promoted_score_section(score: PromotedScore, timed: &[TimedCase]) -> String {
    let mut out = String::new();
    out.push_str(SCORE_SECTION_BEGIN);
    out.push('\n');
    out.push_str("## `promoted_slow_offender_score`\n\n");
    out.push_str("Deterministic lexicase score for the current 100 promoted slow offenders. The score is the average current best-of-five wall-clock probe across the promoted set; lower is better. Delta compares with the previous score recorded in this file, and derivative is the change in delta.\n\n");
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

fn update_benchmarks_score(score: PromotedScore, timed: &[TimedCase]) {
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

fn write_report(cases: &[TimedCase]) {
    write_report_to(
        &crate_report_path(),
        "Hyperreal Slow Performer History",
        cases,
    );
}

fn bench_adversarial_library(c: &mut Criterion) {
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

criterion_group!(benches, bench_adversarial_library);
criterion_main!(benches);
