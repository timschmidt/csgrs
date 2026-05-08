use std::collections::BTreeMap;
use std::fs;
use std::hint::black_box;

use hyperreal::{Computable, Rational, Real};
use num::BigInt;

fn trace_row(
    rows: &mut BTreeMap<String, Vec<hyperreal::dispatch_trace::DispatchCount>>,
    name: impl Into<String>,
    sample: impl FnOnce(),
) {
    hyperreal::dispatch_trace::reset();
    hyperreal::dispatch_trace::with_recording(|| {
        sample();
    });
    let counts = hyperreal::dispatch_trace::take();
    if !counts.is_empty() {
        rows.insert(name.into(), counts);
    }
}

fn real_from_f64(value: f64) -> Real {
    Real::try_from(value).expect("finite f64 imports exactly")
}

fn collect_rows() -> BTreeMap<String, Vec<hyperreal::dispatch_trace::DispatchCount>> {
    let mut rows = BTreeMap::new();

    trace_row(&mut rows, "real/constants", || {
        black_box(Real::zero());
        black_box(Real::one());
        black_box(Real::pi());
        black_box(Real::tau());
        black_box(Real::e());
    });
    trace_row(&mut rows, "real/arithmetic/exact", || {
        let lhs = Real::from(3);
        let rhs = Real::from(7);
        black_box(&lhs + &rhs);
        black_box(&lhs - &rhs);
        black_box(&lhs * &rhs);
        black_box((&lhs / &rhs).unwrap());
    });
    trace_row(&mut rows, "real/trig/general", || {
        let x = real_from_f64(1.23456789);
        black_box(x.clone().sin());
        black_box(x.clone().cos());
        black_box(x.clone().tan().unwrap());
    });
    trace_row(&mut rows, "real/trig/large", || {
        let x = real_from_f64(1.0e30);
        black_box(x.clone().sin());
        black_box(x.clone().cos());
    });
    trace_row(&mut rows, "real/trig/large-exact-rational", || {
        let million = Real::from(1_000_000_i32);
        let e30 = Real::new(Rational::new(10).powi(BigInt::from(30)).unwrap());
        black_box(million.clone().sin());
        black_box(million.clone().cos());
        black_box(million.tan().unwrap());
        black_box(e30.clone().sin());
        black_box(e30.clone().cos());
        black_box(e30.tan().unwrap());
    });
    trace_row(&mut rows, "real/inverse_trig", || {
        let tiny = real_from_f64(1.0e-12);
        let near_one = real_from_f64(0.999999);
        black_box(tiny.clone().asin().unwrap());
        black_box(tiny.clone().acos().unwrap());
        black_box(tiny.clone().atanh().unwrap());
        black_box(near_one.clone().asin().unwrap());
        black_box(near_one.clone().acos().unwrap());
    });
    trace_row(&mut rows, "real/inverse_trig/mid-domain", || {
        let mid = real_from_f64(0.7);
        black_box(mid.clone().asin().unwrap());
        black_box(mid.clone().acos().unwrap());
        black_box(mid.clone().atan().unwrap());
        black_box(mid.atanh().unwrap());
    });
    trace_row(&mut rows, "real/hyperbolic_log_exp", || {
        let x = real_from_f64(1.25);
        black_box(x.clone().exp().unwrap());
        black_box(x.clone().ln().unwrap());
        black_box(x.clone().asinh().unwrap());
        black_box(x.clone().acosh().unwrap());
    });
    trace_row(&mut rows, "real/structural_queries", || {
        let pi_minus_three = Real::pi() - Real::from(3);
        black_box(pi_minus_three.zero_status());
        black_box(pi_minus_three.structural_facts());
    });
    trace_row(&mut rows, "computable/constants", || {
        black_box(Computable::pi());
        black_box(Computable::tau());
        black_box(Computable::e());
    });
    trace_row(&mut rows, "computable/trig", || {
        let x = Computable::rational(Rational::try_from(1.23456789_f64).unwrap());
        black_box(x.clone().sin().approx(-96));
        black_box(x.clone().cos().approx(-96));
    });
    trace_row(&mut rows, "computable/trig/large", || {
        let x = Computable::rational(Rational::try_from(1.0e30_f64).unwrap());
        black_box(x.clone().sin().approx(-96));
        black_box(x.clone().cos().approx(-96));
    });
    trace_row(&mut rows, "computable/trig/large-exact-rational", || {
        let million = Computable::rational(Rational::new(1_000_000));
        let e30 = Computable::rational(Rational::new(10).powi(BigInt::from(30)).unwrap());
        black_box(million.clone().sin().approx(-96));
        black_box(million.clone().cos().approx(-96));
        black_box(million.tan().approx(-96));
        black_box(e30.clone().sin().approx(-96));
        black_box(e30.clone().cos().approx(-96));
        black_box(e30.tan().approx(-96));
    });
    trace_row(&mut rows, "computable/inverse_trig", || {
        let x = Computable::rational(Rational::try_from(1.0e-12_f64).unwrap());
        black_box(x.clone().asin().approx(-96));
        black_box(x.clone().acos().approx(-96));
        black_box(x.clone().atanh().approx(-96));
    });
    trace_row(&mut rows, "computable/inverse_trig/mid-domain", || {
        let x = Computable::rational(Rational::try_from(0.7_f64).unwrap());
        black_box(x.clone().asin().approx(-96));
        black_box(x.clone().acos().approx(-96));
        black_box(x.clone().atan().approx(-96));
        black_box(x.atanh().approx(-96));
    });

    rows
}

fn write_report(rows: &BTreeMap<String, Vec<hyperreal::dispatch_trace::DispatchCount>>) {
    let mut out = String::new();
    out.push_str("# Hyperreal Dispatch Trace\n\n");
    out.push_str("Generated by running `cargo bench --bench dispatch_trace --features dispatch-trace`. This runner samples dispatch paths directly and does not execute Criterion timing loops or update `benchmarks.md`.\n\n");
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
    let rows = collect_rows();
    write_report(&rows);
    eprintln!("updated dispatch_trace.md from {} trace rows", rows.len());
}
