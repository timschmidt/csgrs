use std::collections::BTreeMap;
use std::env;
use std::fs;
use std::hint::black_box;
use std::ops::Neg;

use hyperreal::{Computable, Rational, Real};
use num::{BigInt, BigUint};

fn trace_row(
    rows: &mut BTreeMap<String, Vec<hyperreal::dispatch_trace::DispatchCount>>,
    filters: &[String],
    name: impl Into<String>,
    sample: impl FnOnce(),
) {
    let name = name.into();
    if !filters.is_empty() && !filters.iter().any(|filter| name.contains(filter)) {
        return;
    }

    eprintln!("trace start: {name}");
    hyperreal::dispatch_trace::reset();
    hyperreal::dispatch_trace::with_recording(|| {
        sample();
    });
    let counts = hyperreal::dispatch_trace::take();
    if !counts.is_empty() {
        eprintln!("trace done:  {name} ({} paths)", counts.len());
        rows.insert(name, counts);
    } else {
        eprintln!("trace done:  {name} (0 paths)");
    }
}

fn real_from_f64(value: f64) -> Real {
    Real::try_from(value).expect("finite f64 imports exactly")
}

fn rational(numerator: i64, denominator: u64) -> Rational {
    Rational::fraction(numerator, denominator).unwrap()
}

fn rational_big(numerator: BigInt, denominator: BigUint) -> Rational {
    Rational::from_bigint_fraction(numerator, denominator).unwrap()
}

fn computable(rational: Rational) -> Computable {
    Computable::rational(rational)
}

fn tiny() -> Rational {
    Rational::fraction(1, 1_000_000_000_000_u64).unwrap()
}

fn near_one() -> Rational {
    Rational::fraction(999_999, 1_000_000).unwrap()
}

fn one_plus_tiny() -> Rational {
    Rational::one() + tiny()
}

fn huge_pi_plus_offset() -> Computable {
    Computable::pi()
        .multiply(computable(Rational::from_bigint(BigInt::from(1_u8) << 512)))
        .add(computable(rational(7, 5)))
}

fn near_half_pi() -> Computable {
    let offset = rational_big(BigInt::from(1_u8), BigUint::from(1_u8) << 40);
    Computable::pi()
        .multiply(computable(rational(1, 2)))
        .add(computable(offset).negate())
}

fn trace_computable_approx(
    rows: &mut BTreeMap<String, Vec<hyperreal::dispatch_trace::DispatchCount>>,
    filters: &[String],
    name: impl Into<String>,
    input: Computable,
    precision: i32,
    op: impl FnOnce(Computable) -> Computable,
) {
    trace_row(rows, filters, name, || {
        black_box(op(input).approx(precision));
    });
}

fn collect_rows(
    filters: &[String],
) -> BTreeMap<String, Vec<hyperreal::dispatch_trace::DispatchCount>> {
    let mut rows = BTreeMap::new();

    trace_row(&mut rows, filters, "real/constants", || {
        black_box(Real::zero());
        black_box(Real::one());
        black_box(Real::pi());
        black_box(Real::tau());
        black_box(Real::e());
    });
    trace_row(&mut rows, filters, "real/arithmetic/exact", || {
        let lhs = Real::from(3);
        let rhs = Real::from(7);
        black_box(&lhs + &rhs);
        black_box(&lhs - &rhs);
        black_box(&lhs * &rhs);
        black_box((&lhs / &rhs).unwrap());
    });
    trace_row(&mut rows, filters, "real/trig/general", || {
        let x = real_from_f64(1.23456789);
        black_box(x.clone().sin());
        black_box(x.clone().cos());
        black_box(x.clone().tan().unwrap());
    });
    trace_row(&mut rows, filters, "real/trig/large", || {
        let x = real_from_f64(1.0e30);
        black_box(x.clone().sin());
        black_box(x.clone().cos());
    });
    trace_row(&mut rows, filters, "real/trig/large-exact-rational", || {
        let million = Real::from(1_000_000_i32);
        let e30 = Real::new(Rational::new(10).powi(BigInt::from(30)).unwrap());
        black_box(million.clone().sin());
        black_box(million.clone().cos());
        black_box(million.tan().unwrap());
        black_box(e30.clone().sin());
        black_box(e30.clone().cos());
        black_box(e30.tan().unwrap());
    });
    trace_row(&mut rows, filters, "real/inverse_trig", || {
        let tiny = real_from_f64(1.0e-12);
        let near_one = real_from_f64(0.999999);
        black_box(tiny.clone().asin().unwrap());
        black_box(tiny.clone().acos().unwrap());
        black_box(tiny.clone().atanh().unwrap());
        black_box(near_one.clone().asin().unwrap());
        black_box(near_one.clone().acos().unwrap());
    });
    trace_row(&mut rows, filters, "real/inverse_trig/mid-domain", || {
        let mid = real_from_f64(0.7);
        black_box(mid.clone().asin().unwrap());
        black_box(mid.clone().acos().unwrap());
        black_box(mid.clone().atan().unwrap());
        black_box(mid.atanh().unwrap());
    });
    trace_row(&mut rows, filters, "real/hyperbolic_log_exp", || {
        let x = real_from_f64(1.25);
        black_box(x.clone().exp().unwrap());
        black_box(x.clone().ln().unwrap());
        black_box(x.clone().asinh().unwrap());
        black_box(x.clone().acosh().unwrap());
    });
    trace_row(&mut rows, filters, "real/sqrt_scaled_rational", || {
        black_box(Real::new(Rational::new(18)).sqrt().unwrap());
    });
    trace_row(&mut rows, filters, "real/sqrt_scaled_exp", || {
        let value = Real::new(Rational::new(18)) * Real::new(Rational::new(2)).exp().unwrap();
        black_box(value.sqrt().unwrap());
    });
    trace_row(&mut rows, filters, "real/structural_queries", || {
        let pi_minus_three = Real::pi() - Real::from(3);
        black_box(pi_minus_three.zero_status());
        black_box(pi_minus_three.structural_facts());
    });
    trace_row(&mut rows, filters, "computable/constants", || {
        black_box(Computable::pi());
        black_box(Computable::tau());
        black_box(Computable::e());
    });
    trace_row(&mut rows, filters, "computable/trig", || {
        let x = Computable::rational(Rational::try_from(1.23456789_f64).unwrap());
        black_box(x.clone().sin().approx(-96));
        black_box(x.clone().cos().approx(-96));
    });
    trace_row(&mut rows, filters, "computable/trig/large", || {
        let x = Computable::rational(Rational::try_from(1.0e30_f64).unwrap());
        black_box(x.clone().sin().approx(-96));
        black_box(x.clone().cos().approx(-96));
    });
    trace_row(
        &mut rows,
        filters,
        "computable/trig/large-exact-rational",
        || {
            let million = Computable::rational(Rational::new(1_000_000));
            let e30 = Computable::rational(Rational::new(10).powi(BigInt::from(30)).unwrap());
            black_box(million.clone().sin().approx(-96));
            black_box(million.clone().cos().approx(-96));
            black_box(million.tan().approx(-96));
            black_box(e30.clone().sin().approx(-96));
            black_box(e30.clone().cos().approx(-96));
            black_box(e30.tan().approx(-96));
        },
    );
    trace_row(&mut rows, filters, "computable/inverse_trig", || {
        let x = Computable::rational(Rational::try_from(1.0e-12_f64).unwrap());
        black_box(x.clone().asin().approx(-96));
        black_box(x.clone().acos().approx(-96));
        black_box(x.clone().atanh().approx(-96));
    });
    trace_row(
        &mut rows,
        filters,
        "computable/inverse_trig/mid-domain",
        || {
            let x = Computable::rational(Rational::try_from(0.7_f64).unwrap());
            black_box(x.clone().asin().approx(-96));
            black_box(x.clone().acos().approx(-96));
            black_box(x.clone().atan().approx(-96));
            black_box(x.atanh().approx(-96));
        },
    );
    trace_row(
        &mut rows,
        filters,
        "computable/basic_transcendentals",
        || {
            black_box(computable(rational(7, 5)).exp().approx(-128));
            black_box(computable(rational(11, 7)).ln().approx(-128));
            black_box(computable(Rational::new(1024)).ln().approx(-128));
            black_box(
                computable(rational_big(
                    BigInt::from(1_u8),
                    BigUint::from(1_u8) << 1024,
                ))
                .ln()
                .approx(-128),
            );
            black_box(computable(Rational::new(2)).sqrt().approx(-128));
        },
    );
    trace_row(&mut rows, filters, "computable/ln_smooth_rational", || {
        black_box(computable(rational(45, 14)).ln().approx(-128));
    });
    trace_row(
        &mut rows,
        filters,
        "computable/sqrt_squarefree_rational",
        || {
            black_box(computable(Rational::new(12)).sqrt().approx(-128));
        },
    );
    trace_row(
        &mut rows,
        filters,
        "computable/ln_nonsmooth_rational",
        || {
            black_box(computable(rational(11, 13)).ln().approx(-128));
        },
    );
    let trig_p = -96;
    let tiny_input = computable(tiny());
    let medium_input = computable(rational(7, 5));
    let f64_input = computable(Rational::try_from(1.23456789_f64).unwrap());
    let million_input = computable(Rational::new(1_000_000));
    let e30_input = computable(Rational::from_bigint(BigInt::from(10_u8).pow(30)));
    let huge_pi_input = huge_pi_plus_offset();
    let near_pole_input = near_half_pi();
    trace_computable_approx(
        &mut rows,
        filters,
        "computable/trig_adversarial/sin_tiny",
        tiny_input.clone(),
        trig_p,
        Computable::sin,
    );
    trace_computable_approx(
        &mut rows,
        filters,
        "computable/trig_adversarial/cos_tiny",
        tiny_input.clone(),
        trig_p,
        Computable::cos,
    );
    trace_computable_approx(
        &mut rows,
        filters,
        "computable/trig_adversarial/tan_tiny",
        tiny_input,
        trig_p,
        Computable::tan,
    );
    trace_computable_approx(
        &mut rows,
        filters,
        "computable/trig_adversarial/sin_medium",
        medium_input.clone(),
        trig_p,
        Computable::sin,
    );
    trace_computable_approx(
        &mut rows,
        filters,
        "computable/trig_adversarial/cos_medium",
        medium_input.clone(),
        trig_p,
        Computable::cos,
    );
    trace_computable_approx(
        &mut rows,
        filters,
        "computable/trig_adversarial/tan_medium",
        medium_input,
        trig_p,
        Computable::tan,
    );
    trace_computable_approx(
        &mut rows,
        filters,
        "computable/trig_adversarial/sin_f64_exact",
        f64_input.clone(),
        trig_p,
        Computable::sin,
    );
    trace_computable_approx(
        &mut rows,
        filters,
        "computable/trig_adversarial/cos_f64_exact",
        f64_input,
        trig_p,
        Computable::cos,
    );
    trace_computable_approx(
        &mut rows,
        filters,
        "computable/trig_adversarial/sin_1e6",
        million_input.clone(),
        trig_p,
        Computable::sin,
    );
    trace_computable_approx(
        &mut rows,
        filters,
        "computable/trig_adversarial/cos_1e6",
        million_input.clone(),
        trig_p,
        Computable::cos,
    );
    trace_computable_approx(
        &mut rows,
        filters,
        "computable/trig_adversarial/tan_1e6",
        million_input,
        trig_p,
        Computable::tan,
    );
    trace_computable_approx(
        &mut rows,
        filters,
        "computable/trig_adversarial/sin_1e30",
        e30_input.clone(),
        trig_p,
        Computable::sin,
    );
    trace_computable_approx(
        &mut rows,
        filters,
        "computable/trig_adversarial/cos_1e30",
        e30_input.clone(),
        trig_p,
        Computable::cos,
    );
    trace_computable_approx(
        &mut rows,
        filters,
        "computable/trig_adversarial/tan_1e30",
        e30_input,
        trig_p,
        Computable::tan,
    );
    trace_computable_approx(
        &mut rows,
        filters,
        "computable/trig_adversarial/sin_huge_pi_plus_offset",
        huge_pi_input.clone(),
        trig_p,
        Computable::sin,
    );
    trace_computable_approx(
        &mut rows,
        filters,
        "computable/trig_adversarial/cos_huge_pi_plus_offset",
        huge_pi_input.clone(),
        trig_p,
        Computable::cos,
    );
    trace_computable_approx(
        &mut rows,
        filters,
        "computable/trig_adversarial/tan_huge_pi_plus_offset",
        huge_pi_input,
        trig_p,
        Computable::tan,
    );
    trace_computable_approx(
        &mut rows,
        filters,
        "computable/trig_adversarial/tan_near_half_pi",
        near_pole_input,
        trig_p,
        Computable::tan,
    );
    let inverse_zero = computable(Rational::zero());
    let inverse_tiny = computable(tiny());
    let inverse_mid = computable(rational(7, 10));
    let inverse_near_one = computable(near_one());
    let inverse_near_minus_one = computable(near_one().neg());
    let inverse_large = computable(Rational::new(8));
    let inverse_huge = computable(Rational::from_bigint(BigInt::from(10_u8).pow(30)));
    trace_computable_approx(
        &mut rows,
        filters,
        "computable/inverse_trig_adversarial/asin_zero",
        inverse_zero.clone(),
        trig_p,
        Computable::asin,
    );
    trace_computable_approx(
        &mut rows,
        filters,
        "computable/inverse_trig_adversarial/acos_zero",
        inverse_zero.clone(),
        trig_p,
        Computable::acos,
    );
    trace_computable_approx(
        &mut rows,
        filters,
        "computable/inverse_trig_adversarial/atan_zero",
        inverse_zero,
        trig_p,
        Computable::atan,
    );
    trace_computable_approx(
        &mut rows,
        filters,
        "computable/inverse_trig_adversarial/asin_tiny",
        inverse_tiny.clone(),
        trig_p,
        Computable::asin,
    );
    trace_computable_approx(
        &mut rows,
        filters,
        "computable/inverse_trig_adversarial/acos_tiny",
        inverse_tiny.clone(),
        trig_p,
        Computable::acos,
    );
    trace_computable_approx(
        &mut rows,
        filters,
        "computable/inverse_trig_adversarial/atan_tiny",
        inverse_tiny,
        trig_p,
        Computable::atan,
    );
    trace_computable_approx(
        &mut rows,
        filters,
        "computable/inverse_trig_adversarial/asin_mid",
        inverse_mid.clone(),
        trig_p,
        Computable::asin,
    );
    trace_computable_approx(
        &mut rows,
        filters,
        "computable/inverse_trig_adversarial/acos_mid",
        inverse_mid.clone(),
        trig_p,
        Computable::acos,
    );
    trace_computable_approx(
        &mut rows,
        filters,
        "computable/inverse_trig_adversarial/atan_mid",
        inverse_mid,
        trig_p,
        Computable::atan,
    );
    trace_computable_approx(
        &mut rows,
        filters,
        "computable/inverse_trig_adversarial/asin_near_one",
        inverse_near_one.clone(),
        trig_p,
        Computable::asin,
    );
    trace_computable_approx(
        &mut rows,
        filters,
        "computable/inverse_trig_adversarial/acos_near_one",
        inverse_near_one,
        trig_p,
        Computable::acos,
    );
    trace_computable_approx(
        &mut rows,
        filters,
        "computable/inverse_trig_adversarial/asin_near_minus_one",
        inverse_near_minus_one.clone(),
        trig_p,
        Computable::asin,
    );
    trace_computable_approx(
        &mut rows,
        filters,
        "computable/inverse_trig_adversarial/acos_near_minus_one",
        inverse_near_minus_one,
        trig_p,
        Computable::acos,
    );
    trace_computable_approx(
        &mut rows,
        filters,
        "computable/inverse_trig_adversarial/atan_large",
        inverse_large,
        trig_p,
        Computable::atan,
    );
    trace_computable_approx(
        &mut rows,
        filters,
        "computable/inverse_trig_adversarial/atan_huge",
        inverse_huge,
        trig_p,
        Computable::atan,
    );
    let hyper_p = -128;
    let hyper_tiny = computable(tiny());
    let hyper_mid = computable(rational(1, 2));
    let hyper_large = computable(Rational::new(1_000_000));
    let hyper_large_negative = computable(Rational::new(-1_000_000));
    let hyper_one_plus_tiny = computable(one_plus_tiny());
    let hyper_sqrt_two = computable(Rational::new(2)).sqrt();
    let hyper_two = computable(Rational::new(2));
    let hyper_near_one = computable(near_one());
    let hyper_near_minus_one = computable(near_one().neg());
    trace_computable_approx(
        &mut rows,
        filters,
        "computable/inverse_hyperbolic_adversarial/asinh_tiny",
        hyper_tiny.clone(),
        hyper_p,
        Computable::asinh,
    );
    trace_computable_approx(
        &mut rows,
        filters,
        "computable/inverse_hyperbolic_adversarial/asinh_mid",
        hyper_mid.clone(),
        hyper_p,
        Computable::asinh,
    );
    trace_computable_approx(
        &mut rows,
        filters,
        "computable/inverse_hyperbolic_adversarial/asinh_large",
        hyper_large.clone(),
        hyper_p,
        Computable::asinh,
    );
    trace_computable_approx(
        &mut rows,
        filters,
        "computable/inverse_hyperbolic_adversarial/asinh_large_negative",
        hyper_large_negative,
        hyper_p,
        Computable::asinh,
    );
    trace_computable_approx(
        &mut rows,
        filters,
        "computable/inverse_hyperbolic_adversarial/acosh_one_plus_tiny",
        hyper_one_plus_tiny,
        hyper_p,
        Computable::acosh,
    );
    trace_computable_approx(
        &mut rows,
        filters,
        "computable/inverse_hyperbolic_adversarial/acosh_sqrt_two",
        hyper_sqrt_two,
        hyper_p,
        Computable::acosh,
    );
    trace_computable_approx(
        &mut rows,
        filters,
        "computable/inverse_hyperbolic_adversarial/acosh_two",
        hyper_two,
        hyper_p,
        Computable::acosh,
    );
    trace_computable_approx(
        &mut rows,
        filters,
        "computable/inverse_hyperbolic_adversarial/acosh_large",
        hyper_large,
        hyper_p,
        Computable::acosh,
    );
    trace_computable_approx(
        &mut rows,
        filters,
        "computable/inverse_hyperbolic_adversarial/atanh_tiny",
        hyper_tiny,
        hyper_p,
        Computable::atanh,
    );
    trace_computable_approx(
        &mut rows,
        filters,
        "computable/inverse_hyperbolic_adversarial/atanh_mid",
        hyper_mid,
        hyper_p,
        Computable::atanh,
    );
    trace_computable_approx(
        &mut rows,
        filters,
        "computable/inverse_hyperbolic_adversarial/atanh_near_one",
        hyper_near_one,
        hyper_p,
        Computable::atanh,
    );
    trace_computable_approx(
        &mut rows,
        filters,
        "computable/inverse_hyperbolic_adversarial/atanh_near_minus_one",
        hyper_near_minus_one,
        hyper_p,
        Computable::atanh,
    );

    rows
}

fn write_report(rows: &BTreeMap<String, Vec<hyperreal::dispatch_trace::DispatchCount>>) {
    let mut out = String::new();
    out.push_str("# Hyperreal Dispatch Trace\n\n");
    out.push_str("Generated by running `cargo bench --bench dispatch_trace --features dispatch-trace`. This runner samples dispatch paths directly and does not execute Criterion timing loops or update `benchmarks.md`. Pass row-name substrings after `--` to trace a subset, for example `cargo bench --bench dispatch_trace --features dispatch-trace -- computable/trig_adversarial/sin_1e30`.\n\n");
    out.push_str("| Trace Row | Layer | Operation | Path | Count |\n");
    out.push_str("| --- | --- | --- | --- | ---: |\n");
    for (row, counts) in rows {
        for count in counts {
            out.push_str(&format!(
                "| `{}` | `{}` | `{}` | `{}` | {} |\n",
                row, count.layer, count.operation, count.path, count.count
            ));
        }
    }

    if let Err(error) = fs::write("dispatch_trace.md", out) {
        eprintln!("failed to update dispatch_trace.md: {error}");
    }
}

fn main() {
    let filters: Vec<String> = env::args()
        .skip(1)
        .filter(|arg| !arg.is_empty() && !arg.starts_with('-'))
        .collect();
    if filters.is_empty() {
        eprintln!("dispatch trace filters: <all rows>");
    } else {
        eprintln!("dispatch trace filters: {}", filters.join(", "));
    }

    let rows = collect_rows(&filters);
    write_report(&rows);
    eprintln!("updated dispatch_trace.md from {} trace rows", rows.len());
}
