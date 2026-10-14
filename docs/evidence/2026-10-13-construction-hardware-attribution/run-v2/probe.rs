//! Hardware attribution only; libraries and fixture/witness suffix are frozen.
#[path = "../counters_v1.rs"]
mod counters;

use hyperreal::{Rational, Real};
use std::cell::RefCell;
use std::hint::black_box;

thread_local! {
    static GROUP: RefCell<Option<Result<counters::Group, String>>> = const { RefCell::new(None) };
}

fn measure<T>(name: &str, count: usize, body: impl FnOnce() -> T) -> T {
    let (result, empty, observation) = GROUP.with(|slot| {
        let mut slot = slot.borrow_mut();
        let group = slot
            .get_or_insert_with(|| counters::Group::open().map_err(|error| error.to_string()));
        match group {
            Ok(group) => {
                let (_, empty) = group.observe(|| black_box(()));
                let (result, observation) = group.observe(body);
                (result, empty, observation)
            },
            Err(error) => {
                let (_, empty) = counters::unavailable(|| black_box(()), error);
                let (result, observation) = counters::unavailable(body, error);
                (result, empty, observation)
            },
        }
    });
    println!(
        "{}",
        serde_json::json!({
            "hardware_phase": name, "iterations": count,
            "empty_window": empty.json(), "work_window": observation.json(),
        })
    );
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
