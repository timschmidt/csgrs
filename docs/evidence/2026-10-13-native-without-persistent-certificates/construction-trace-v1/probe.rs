//! Existing public dispatch/rational counters; no elapsed-cost measurement.
use hyperreal::{Rational, Real, dispatch_trace};
use std::hint::black_box;

fn measure<T>(name: &str, count: usize, body: impl FnOnce() -> T) -> T {
    dispatch_trace::reset();
    let result = dispatch_trace::with_recording(body);
    let snapshot = dispatch_trace::take_trace();
    let stats = snapshot.rational;
    let dispatch: Vec<_> = snapshot.dispatch.into_iter().map(|entry| {
        serde_json::json!({"layer": entry.layer, "operation": entry.operation,
                           "path": entry.path, "count": entry.count})
    }).collect();
    println!("{}", serde_json::json!({
        "trace_phase": name, "iterations": count, "dispatch": dispatch,
        "rational": {
            "temporary_rationals": stats.temporary_rationals,
            "reductions": stats.reductions, "gcds": stats.gcds,
            "peak_operand_bits": stats.peak_operand_bits,
            "common_factors": {"none": stats.common_factors.none,
                "power_of_two": stats.common_factors.power_of_two,
                "small": stats.common_factors.small, "medium": stats.common_factors.medium,
                "large": stats.common_factors.large},
            "gcd_operand_widths": {"both_u64": stats.gcd_operand_widths.both_u64,
                "mixed_u64_u128": stats.gcd_operand_widths.mixed_u64_u128,
                "both_u128": stats.gcd_operand_widths.both_u128,
                "arbitrary_precision": stats.gcd_operand_widths.arbitrary_precision}
        }
    }));
    black_box(result)
}

fn inputs(index: usize) -> (i64, i64) {
    ((index % 7) as i64 - 3, [-3, -2, -1, 1, 2, 3][index % 6])
}

fn fixture(kind: usize, index: usize) -> Real {
    let (a, b) = inputs(index);
    let a = Real::from(a);
    let b = Real::from(b);
    let root = Real::from(13).sqrt().unwrap();
    match kind {
        0 => a + b * root,
        1 => -(a + b * root) * Real::new(Rational::fraction(1, 8).unwrap()),
        2 => {
            // (sqrt(2) + sqrt(3))^2 = 5 + 2*sqrt(6).
            let sum = Real::from(2).sqrt().unwrap() + Real::from(3).sqrt().unwrap();
            &sum * &sum
        },
        3 => ((a + root) * Real::pi() / Real::pi()).unwrap(),
        4 => Real::from(2).sqrt().unwrap() + Real::from(3).sqrt().unwrap(),
        5 => Real::new(Rational::fraction(3, 4).unwrap()).atan().unwrap() + a,
        6 => {
            // -pi * (pi - (atan(3/4) + pi)) * pi / pi^2 = atan(3/4).
            let angle = Real::new(Rational::fraction(3, 4).unwrap()).atan().unwrap();
            let pi = Real::pi();
            let shifted = &angle + &pi;
            let transported = (&pi * (&pi - shifted) * &pi / (&pi * &pi)).unwrap();
            -transported - angle + a
        },
        7 => (&a + &root) * (&a - &root) + b * root,
        _ => unreachable!(),
    }
}

fn expected(kind: usize, index: usize) -> Option<(Rational, Rational, Rational)> {
    let (a, b) = inputs(index);
    let (constant, scale, radicand) = match kind {
        0 => (Rational::new(a), Rational::new(b), 13),
        1 => (
            Rational::fraction(-a, 8).unwrap(),
            Rational::fraction(-b, 8).unwrap(),
            13,
        ),
        2 => (Rational::new(5), Rational::new(2), 6),
        3 => (Rational::new(a), Rational::one(), 13),
        4 | 5 => return None,
        6 => (Rational::new(a), Rational::zero(), 0),
        7 => (Rational::new(a * a - 13), Rational::new(b), 13),
        _ => unreachable!(),
    };
    Some((constant, scale, Rational::new(radicand)))
}

fn verify(name: &str, kind: usize, answers: &[Option<(Rational, Rational, Rational)>]) {
    let mut certified = 0_usize;
    let mut hash = 0xcbf29ce484222325_u64;
    for (index, answer) in answers.iter().enumerate() {
        assert_eq!(*answer, expected(kind, index), "{name}, item {index}");
        let witness = match answer {
            Some((a, b, d)) => {
                certified += 1;
                format!("{a};{b};{d}\n")
            },
            None => "Unknown\n".to_owned(),
        };
        for byte in witness.bytes() {
            hash = (hash ^ u64::from(byte)).wrapping_mul(0x100000001b3);
        }
    }
    println!(
        "{{\"witness\":\"{name}\",\"certified\":{certified},\"unknown\":{},\"quadratic_parts_fnv1a64\":\"{hash:016x}\"}}",
        answers.len() - certified
    );
}

fn main() {
    let count: usize = std::env::args().nth(1).unwrap().parse().unwrap();
    for (kind, name) in [
        "affine_surd",
        "negated_scaled_surd",
        "multiradical_square",
        "pi_transport",
        "mixed_field_decline",
        "opaque_decline",
        "opaque_cancellation",
        "conjugate_then_affine",
    ]
    .into_iter()
    .enumerate()
    {
        let values = measure(&format!("{name}_construction"), count, || {
            (0..count)
                .map(|index| fixture(kind, index))
                .collect::<Vec<_>>()
        });
        let non_leaf = values.iter().filter(|v| v.exact_rational().is_none()).count();
        println!("{{\"fixture\":\"{name}\",\"non_leaf_inputs\":{non_leaf}}}");
        for phase in ["first", "warm"] {
            let query = format!("{name}_{phase}");
            let answers = measure(&query, count, || {
                values
                    .iter()
                    .map(|value| black_box(value).exact_quadratic_normal_form())
                    .collect::<Vec<_>>()
            });
            verify(&query, kind, &answers);
        }
        let cloned = values.clone();
        let query = format!("{name}_cloned");
        let answers = measure(&query, count, || {
            cloned
                .iter()
                .map(|value| black_box(value).exact_quadratic_normal_form())
                .collect::<Vec<_>>()
        });
        verify(&query, kind, &answers);
    }
}
