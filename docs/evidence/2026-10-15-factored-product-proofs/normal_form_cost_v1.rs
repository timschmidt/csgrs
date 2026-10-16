//! Exact completion, allocations and query cost; unresolved rows stay visible.
use hyperreal::{Rational, Real};
use std::hint::black_box;
use std::sync::atomic::Ordering::Relaxed;
use std::time::Instant;

include!("counted_allocator_v1.rs");

fn affine_residual() -> Real {
    let origin = Real::from(6) - Real::from(2).sqrt().unwrap();
    let u = Real::from(6) - Real::from(2) * (Real::pi() / Real::from(8)).unwrap().sin();
    let v = Real::from(6)
        - Real::from(2) * ((Real::pi() * Real::from(3)) / Real::from(8)).unwrap().sin();
    let nx = Real::from(6) * (&v - &origin);
    let ny = Real::from(-6) * (&u - &origin);
    let offset = -Real::dot2_refs([&nx, &ny], [&origin, &origin]);
    // h(v-o)u - h(u-o)v - h(v-o)o + h(u-o)o = 0.
    Real::signed_product_sum([true; 3], [[&nx, &u], [&ny, &v], [&offset, &Real::one()]])
}

fn quotient_residual() -> Real {
    let origin = Real::from(6) - Real::from(2).sqrt().unwrap();
    let u = Real::from(6) - Real::from(2) * (Real::pi() / Real::from(8)).unwrap().sin();
    let v = Real::from(6)
        - Real::from(2) * ((Real::pi() * Real::from(3)) / Real::from(8)).unwrap().sin();
    // Both denominators are 6(v-u) < 0; (u+v)(u-v)=u^2-v^2.
    let expanded = Real::from(6) * (&v - &origin) - Real::from(6) * (&u - &origin);
    let reduced = Real::from(6) * (&v - &u);
    (((&u + &v) * (&u - &v)) / expanded).unwrap() - ((&u * &u - &v * &v) / reduced).unwrap()
}

fn factor_residual() -> Real {
    let a = Real::from(4) + Real::e().sin();
    let b = Real::one() + (Real::pi() / Real::from(8)).unwrap().sin();
    // a > 3, b < 2; (a^2-b^2)/(a-b)=a+b, hence equal eighth powers.
    let mut quotient = ((&a * &a - &b * &b) / (&a - &b)).unwrap();
    let mut polynomial = a + b;
    for _ in 0..3 {
        quotient = &quotient * &quotient;
        polynomial = &polynomial * &polynomial;
    }
    quotient - polynomial
}

fn shared_residual() -> Real {
    let [a, b, c, d] = [3, 5, 7, 11]
        .map(|denominator| Real::from(Rational::fraction(1, denominator).unwrap()).sin());
    let six = Real::from(6);
    let a = Real::from(2) * a + &six;
    let x = Real::from(2) * b + &six;
    let z = &a + ((&x - &a) / &six).unwrap();
    let u = Real::from(2) * c;
    let d = Real::from(2) * d + &six;
    let y = &d + ((&u + &six - &d) / &six).unwrap();
    let h = &z * (&u - &six) - (y - Real::from(12)) * &x;
    let s = (&six - &u) * &h * &x + &x * &h * (&u + &six);
    let common = &z * &h;
    let left = -(&h * s * &common);
    let right = &h * (&x * &h) * &common;
    // s=12*x*h, left=-12*right, so these scaled nested sums cancel.
    Real::from(3) * (&left + Real::from(5) * &right)
        + Real::from(7) * (left + Real::from(15) * right)
}

fn value(case: &str) -> Real {
    match case {
        "affine" => affine_residual(),
        "quotient" => quotient_residual(),
        "factor" => factor_residual(),
        "shared" => shared_residual(),
        "shared_negative" | "shared_positive" => {
            let numerator = if case == "shared_negative" { -1 } else { 1 };
            shared_residual()
                + Real::from(Rational::fraction(numerator, 1_000_000_000_000).unwrap())
        },
        "unresolved_trig" => {
            let sine = Real::e().sin();
            let cosine = Real::e().cos();
            &sine * &sine + &cosine * &cosine - Real::one()
        },
        _ => panic!("unknown case"),
    }
}

fn evaluate(case: &str, value: &Real) -> (usize, usize) {
    let constructed_rational = usize::from(value.exact_rational().is_some());
    let result = black_box(value.exact_rational_normal_form());
    if let Some(rational) = result {
        let numerator = match case {
            "shared_negative" => -1,
            "shared_positive" => 1,
            "unresolved_trig" => {
                panic!("unexpected change to unsupported trigonometric proof")
            },
            _ => 0,
        };
        let expected = Rational::fraction(numerator, 1_000_000_000_000).unwrap();
        assert_eq!(rational, expected);
        (1, constructed_rational)
    } else {
        (0, constructed_rational)
    }
}

fn main() {
    let args: Vec<_> = std::env::args().collect();
    let candidate = match args[1].as_str() {
        "baseline" => false,
        "candidate" => true,
        _ => panic!("unknown revision"),
    };
    let iterations: usize = args[2].parse().unwrap();
    assert!((1..=4096).contains(&iterations));
    for case in [
        "affine",
        "quotient",
        "factor",
        "shared",
        "shared_negative",
        "shared_positive",
        "unresolved_trig",
    ] {
        for prepared in [false, true] {
            let retained = value(case);
            let perform = || {
                if prepared {
                    evaluate(case, &retained)
                } else {
                    evaluate(case, &value(case))
                }
            };
            for _ in 0..32 {
                black_box(perform());
            }
            let live = LIVE.load(Relaxed);
            ALLOCATIONS.store(0, Relaxed);
            REQUESTED.store(0, Relaxed);
            PEAK.store(live, Relaxed);
            let started = Instant::now();
            let (mut certified, mut constructed_rational) = (0, 0);
            for _ in 0..iterations {
                let counts = black_box(perform());
                certified += counts.0;
                constructed_rational += counts.1;
            }
            let nanoseconds = started.elapsed().as_nanos();
            let allocations = ALLOCATIONS.load(Relaxed);
            let requested_bytes = REQUESTED.load(Relaxed);
            let peak_live_bytes = PEAK.load(Relaxed).saturating_sub(live);
            let retained_bytes = LIVE.load(Relaxed) as i128 - live as i128;
            if candidate && case != "unresolved_trig" {
                assert_eq!(certified, iterations, "candidate must complete {case}");
            }
            println!(
                "{{\"case\":\"{case}\",\"prepared\":{prepared},\"iterations\":{iterations},\"certified\":{certified},\"unknown\":{},\"constructed_rational\":{constructed_rational},\"nanoseconds\":{nanoseconds},\"allocations\":{allocations},\"requested_bytes\":{requested_bytes},\"peak_live_bytes\":{peak_live_bytes},\"retained_bytes\":{retained_bytes}}}",
                iterations - certified
            );
        }
    }
}
