use hyperreal::{Computable, Rational, RealSign};
use num::{BigInt, BigUint, One};
use rug::{Float, Integer, Rational as Q, float::Round};
use std::sync::{
    Arc,
    atomic::{AtomicBool, Ordering},
};

fn exact(q: &Rational) -> Q {
    let mut n = Integer::from_str_radix(&q.numerator().to_string(), 10).unwrap();
    if q.sign() == num::bigint::Sign::Minus {
        n = -n;
    }
    Q::from((
        n,
        Integer::from_str_radix(&q.denominator().to_string(), 10).unwrap(),
    ))
}

fn inputs() -> Vec<Rational> {
    let mut values: Vec<_> = (-16..=16)
        .map(|n| Rational::fraction(n, 2).unwrap())
        .collect();
    for bits in [20, 256, 1024] {
        let q = Rational::from_bigint_fraction(BigInt::one(), BigUint::one() << bits).unwrap();
        values.extend([q.clone(), -q]);
    }
    values.extend([-128, -32, 32, 128].map(Rational::new));
    values
}

fn source(q: &Rational, sine: bool, shift: i32) -> Computable {
    let x = Computable::rational(q.clone());
    let x = if sine { x.sin() } else { x };
    let scale = if shift >= 0 {
        Rational::from_bigint_fraction(BigInt::one() << shift as usize, BigUint::one()).unwrap()
    } else {
        Rational::from_bigint_fraction(BigInt::one(), BigUint::one() << (-shift) as usize).unwrap()
    };
    x.exp().multiply(Computable::rational(scale))
}

fn reference(q: &Rational, sine: bool, shift: i32) -> (Q, Q) {
    let q = exact(q);
    let mut lo = Float::with_val_round(2304, &q, Round::Down).0;
    let mut hi = Float::with_val_round(2304, &q, Round::Up).0;
    // Every input here is an exactly representable dyadic. This guard is
    // essential before using sin on both endpoints outside a monotone range.
    assert_eq!(lo.to_rational().unwrap(), q);
    assert_eq!(hi.to_rational().unwrap(), q);
    if sine {
        lo.sin_round(Round::Down);
        hi.sin_round(Round::Up);
    }
    lo.exp_round(Round::Down);
    hi.exp_round(Round::Up);
    lo <<= shift;
    hi <<= shift;
    lo.sqrt_round(Round::Down);
    hi.sqrt_round(Round::Up);
    (lo.to_rational().unwrap(), hi.to_rational().unwrap())
}

fn check(value: &Computable, p: i32, bounds: &(Q, Q), context: &str) {
    let unit = if p <= 0 {
        Q::from((1, Integer::from(1) << -p))
    } else {
        Q::from(Integer::from(1) << p)
    };
    let actual = Integer::from_str_radix(&value.approx(p).to_string(), 10).unwrap();
    let center = Q::from(actual) * &unit;
    assert!(
        Q::from(&center - &unit) <= bounds.0 && Q::from(&center + &unit) >= bounds.1,
        "one-ulp failure {context}, p={p}"
    );
}

fn oracle() {
    let mut checks = 0;
    for (case, q) in inputs().iter().enumerate() {
        for sine in [false, true] {
            for shift in [-31, -2, -1, 0, 1, 2, 31] {
                let bounds = reference(q, sine, shift);
                for lifecycle in [
                    "fresh",
                    "mixed",
                    "hot-operand",
                    "input-serde",
                    "output-serde",
                ] {
                    let mut operand = source(q, sine, shift);
                    if lifecycle == "hot-operand" {
                        let _ = operand.approx(-768);
                    }
                    if lifecycle == "input-serde" {
                        operand = serde_json::from_str(&serde_json::to_string(&operand).unwrap())
                            .unwrap();
                    }
                    let mut value = operand.sqrt();
                    if lifecycle == "output-serde" {
                        value =
                            serde_json::from_str(&serde_json::to_string(&value).unwrap()).unwrap();
                    }
                    assert_eq!(value.structural_facts().sign, Some(RealSign::Positive));
                    for p in [0, -32, -128, -512, -8] {
                        let value = if lifecycle == "fresh" {
                            source(q, sine, shift).sqrt()
                        } else {
                            value.clone()
                        };
                        check(
                            &value,
                            p,
                            &bounds,
                            &format!("case={case}, sine={sine}, shift={shift}, {lifecycle}"),
                        );
                        checks += 1;
                    }
                }
            }
        }
    }
    println!("{{\"suite\":\"oracle\",\"inputs\":43,\"enclosure_checks\":{checks}}}");
}

fn state() {
    let mut checks = 0;
    for q in [-8, -1, 0, 1, 8, 32].map(Rational::new) {
        let bounds = reference(&q, true, 3);
        let value = source(&q, true, 3).sqrt();
        let signal = Arc::new(AtomicBool::new(true));
        let _ = value.approx_signal(&Some(signal.clone()), -512);
        signal.store(false, Ordering::Relaxed);
        for p in [-32, -512, -128] {
            check(&value, p, &bounds, "cancel/retry");
            checks += 1;
        }
        let value = source(&q, true, 3).sqrt();
        std::thread::scope(|scope| {
            let handles: Vec<_> = (0..4)
                .map(|thread| {
                    let value = value.clone();
                    let bounds = &bounds;
                    scope.spawn(move || {
                        for p in [-32, -256, -64, -512, -128]
                            .into_iter()
                            .cycle()
                            .skip(thread)
                            .take(10)
                        {
                            check(&value, p, bounds, "concurrent root refinement");
                        }
                    })
                })
                .collect();
            for handle in handles {
                handle.join().unwrap();
            }
        });
        checks += 40;
    }
    // A non-rationally represented exact zero exponent must still produce one.
    let x = Computable::rational(Rational::new(2)).sqrt();
    let z = x
        .clone()
        .sin()
        .square()
        .add(x.cos().square())
        .add(Computable::one().negate());
    let value = z.exp().sqrt();
    for p in [0, -64, -256] {
        check(&value, p, &(Q::from(1), Q::from(1)), "opaque zero");
        checks += 1;
    }
    println!("{{\"suite\":\"state\",\"enclosure_checks\":{checks}}}");
}

fn main() {
    match std::env::args().nth(1).as_deref() {
        Some("oracle") => oracle(),
        Some("state") => state(),
        _ => panic!("expected oracle|state"),
    }
}
