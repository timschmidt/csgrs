// Independent MPFR interval oracle, shared byte-for-byte by both variants.
// This validates the documented absolute error contract, not relative accuracy.
use hyperreal::{Computable, Rational, RealSign};
use num::{BigInt, BigUint, One};
use rug::{Float, Integer, Rational as Q, float::Round};
use std::sync::{Arc, atomic::{AtomicBool, Ordering}};

fn inputs() -> Vec<Rational> {
    let mut values: Vec<_> = (-64..=64).map(|n| Rational::fraction(n, 16).unwrap()).collect();
    values.extend((-7..=7).map(|n| Rational::fraction(n, 3).unwrap()));
    for bits in [20, 256, 1024] {
        let q = Rational::from_bigint_fraction(BigInt::one(), BigUint::one() << bits).unwrap();
        values.extend([q.clone(), -q]);
    }
    for n in [8, 12, 16, 24, 32] {
        values.extend([Rational::new(n), Rational::new(-n)]);
    }
    values
}

fn reference(q: &Rational, complement: bool) -> (Q, Q) {
    // Hyper exposes an unsigned numerator magnitude and a separate sign.
    let mut numerator = Integer::from_str_radix(&q.numerator().to_string(), 10).unwrap();
    if q.sign() == num::bigint::Sign::Minus { numerator = -numerator; }
    let exact = Q::from((
        numerator,
        Integer::from_str_radix(&q.denominator().to_string(), 10).unwrap(),
    ));
    let mut lower = Float::with_val_round(2304, &exact, Round::Down).0;
    let mut upper = Float::with_val_round(2304, &exact, Round::Up).0;
    if complement {
        // erfc decreases, so reverse input enclosure endpoints first.
        std::mem::swap(&mut lower, &mut upper);
        lower.erfc_round(Round::Down);
        upper.erfc_round(Round::Up);
    } else {
        lower.erf_round(Round::Down);
        upper.erf_round(Round::Up);
    }
    (lower.to_rational().unwrap(), upper.to_rational().unwrap())
}

fn build(q: &Rational, complement: bool) -> Computable {
    let input = Computable::rational(q.clone());
    if complement { input.erfc() } else { input.erf() }
}

fn check(value: &Computable, p: i32, bounds: &(Q, Q), context: &str) {
    let unit = if p <= 0 { Q::from((1, Integer::from(1) << -p)) }
        else { Q::from(Integer::from(1) << p) };
    let actual = Integer::from_str_radix(&value.approx(p).to_string(), 10).unwrap();
    let center = Q::from(actual) * &unit;
    assert!(Q::from(&center - &unit) <= bounds.0 &&
        Q::from(&center + &unit) >= bounds.1, "one-ulp enclosure failed: {context}, p={p}");
}

fn oracle() {
    let inputs = inputs();
    let mut checks = 0;
    for (case, q) in inputs.iter().enumerate() {
        for complement in [false, true] {
            let bounds = reference(q, complement);
            for lifecycle in ["fresh", "mixed", "serde"] {
                let prepared = build(q, complement);
                let prepared = if lifecycle == "serde" {
                    serde_json::from_str(&serde_json::to_string(&prepared).unwrap()).unwrap()
                } else { prepared };
                for p in [4, 0, -32, -128, -512, -64] {
                    let value = if lifecycle == "fresh" { build(q, complement) } else { prepared.clone() };
                    check(&value, p, &bounds, &format!("case={case}, erfc={complement}, {lifecycle}"));
                    checks += 1;
                }
            }
            // Resolve the tiny input and deep tail beyond the common grid's
            // absolute precision, rather than accepting zero as sufficient.
            if case >= 144 {
                for p in [-1152, -1600, -96] {
                    check(&build(q, complement), p, &bounds, &format!("deep case={case}, erfc={complement}"));
                    checks += 1;
                }
            }
        }
    }
    println!("{{\"suite\":\"oracle\",\"inputs\":{},\"enclosure_checks\":{checks}}}", inputs.len());
}

fn facts() {
    let mut lost = 0;
    for (case, input) in [
        Computable::rational(Rational::new(-2)),
        Computable::rational(Rational::new(100)).sin().negate(),
        Computable::rational(Rational::new(2)).sqrt().negate(),
        Computable::pi().negate(),
    ].into_iter().enumerate() {
        let value = input.erfc();
        let before = value.structural_facts().sign;
        let encoded = serde_json::to_string(&value).unwrap();
        let restored: Computable = serde_json::from_str(&encoded).unwrap();
        let after = restored.structural_facts().sign;
        println!("{{\"suite\":\"facts\",\"case\":{case},\"before\":\"{before:?}\",\"after\":\"{after:?}\",\"serialized_bytes\":{}}}", encoded.len());
        assert_eq!(before, Some(RealSign::Positive));
        if after != Some(RealSign::Positive) { lost += 1; }
    }
    assert_eq!(lost, 0, "previously structural positive facts lost across serialization");
}

fn state() {
    let mut checks = 0;
    for q in [Rational::fraction(1, 3).unwrap(), Rational::new(-2), Rational::new(8)] {
        for complement in [false, true] {
            let bounds = reference(&q, complement);
            let value = build(&q, complement);
            let signal = Arc::new(AtomicBool::new(true));
            let _ = value.approx_signal(&Some(signal.clone()), -256);
            signal.store(false, Ordering::Relaxed);
            for p in [-32, -512, -128] {
                check(&value, p, &bounds, "cancel then retry");
                checks += 1;
            }
            let shared = build(&q, complement);
            std::thread::scope(|scope| {
                let handles: Vec<_> = (0..4).map(|thread| {
                    let value = shared.clone();
                    let bounds = &bounds;
                    scope.spawn(move || {
                        for p in [-32, -256, -64, -512, -128].into_iter().cycle().skip(thread).take(10) {
                            check(&value, p, bounds, "concurrent refinement");
                        }
                    })
                }).collect();
                for handle in handles { handle.join().unwrap(); }
            });
            checks += 40;
        }
    }
    // A mathematically zero input without an exact-rational leaf must remain
    // valid even when zero is not decided structurally.
    let x = Computable::rational(Rational::new(2)).sqrt();
    let opaque_zero = x.clone().sin().square().add(x.cos().square()).add(Computable::one().negate());
    for complement in [false, true] {
        let value = if complement { opaque_zero.clone().erfc() } else { opaque_zero.clone().erf() };
        let exact = Q::from(if complement { 1 } else { 0 });
        let bounds = (exact.clone(), exact);
        let restored: Computable = serde_json::from_str(&serde_json::to_string(&value).unwrap()).unwrap();
        for input in [&value, &restored] {
            for p in [0, -64, -256] {
                check(input, p, &bounds, "opaque-zero input");
                checks += 1;
            }
        }
    }
    // Nearby but different arguments must not cancel structurally. erfc is
    // strictly decreasing, so this residual is negative, never exact zero.
    let q = Rational::fraction(1, 3).unwrap();
    let epsilon = Rational::from_bigint_fraction(BigInt::one(), BigUint::one() << 100usize).unwrap();
    let residual = build(&q, false).add(build(&(q + epsilon), true)).add(Computable::one().negate());
    assert!(matches!(residual.sign_until(-64), None | Some(RealSign::Negative)));
    assert_eq!(residual.sign_until(-256), Some(RealSign::Negative));
    assert_eq!(residual.sign_until(-32), Some(RealSign::Negative));
    println!("{{\"suite\":\"state\",\"enclosure_checks\":{checks}}}");
}

fn main() {
    match std::env::args().nth(1).as_deref() {
        Some("oracle") => oracle(),
        Some("facts") => facts(),
        Some("state") => state(),
        _ => panic!("expected oracle|facts|state"),
    }
}
