// Exact rational enclosure utilities shared by native inverse kernels.
fn interval_dyadic(value: BigInt, p: Precision) -> Rational {
    if p < 0 {
        Rational::from_bigint_fraction(value, BigUint::one() << p.unsigned_abs() as usize)
            .expect("dyadic denominator is nonzero")
    } else {
        Rational::from_bigint(value << p as usize)
    }
}

fn interval_of(c: &Computable, signal: &Option<Signal>, p: Precision) -> [Rational; 2] {
    if let Some(exact) = c.exact_rational() {
        return [exact.clone(), exact];
    }
    let a = c.approx_signal(signal, p);
    [interval_dyadic(&a - 1, p), interval_dyadic(a + 1, p)]
}

fn interval_min(a: Rational, b: Rational) -> Rational {
    if a < b { a } else { b }
}

fn interval_max(a: Rational, b: Rational) -> Rational {
    if a > b { a } else { b }
}

fn interval_clamp(a: Rational, k: &Rational) -> Rational {
    interval_max(-k.clone(), interval_min(k.clone(), a))
}
