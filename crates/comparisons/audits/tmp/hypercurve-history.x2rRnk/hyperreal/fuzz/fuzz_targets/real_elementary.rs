//! Fuzz lazy elementary-function construction, domains, and evaluation.

#![no_main]

use arbitrary::Arbitrary;
use hyperreal::{Rational, Real, RealStructuralFacts, ZeroKnowledge};
use libfuzzer_sys::fuzz_target;
use num::BigInt;

#[derive(Clone, Copy, Debug, Arbitrary)]
struct Input {
    numerator: i16,
    denominator: u8,
    selector: u8,
    exponent: i8,
}

fn assert_facts_refine(before: RealStructuralFacts, after: RealStructuralFacts) {
    if let Some(sign) = before.sign {
        assert_eq!(after.sign, Some(sign));
    }
    if before.zero != ZeroKnowledge::Unknown {
        assert_eq!(after.zero, before.zero);
    }
    assert_eq!(after.exact_rational, before.exact_rational);
}

fn force(value: Result<Real, hyperreal::Problem>) {
    if let Ok(value) = value {
        let facts = value.structural_facts();
        let first = value.to_f64_lossy();
        let second = value.to_f64_lossy();
        assert!(first.is_none_or(f64::is_finite));
        assert!(second.is_none_or(f64::is_finite));
        assert_facts_refine(facts, value.structural_facts());
        let _ = value.certified_sign_until(-96);
        assert_facts_refine(facts, value.structural_facts());
    }
}

fuzz_target!(|input: Input| {
    let numerator = i64::from(input.numerator) % 17;
    let denominator = u64::from(input.denominator) + 1;
    let rational = Rational::fraction(numerator, denominator).expect("positive denominator");
    let value = Real::new(rational);

    match input.selector % 24 {
        0 => force(value.clone().sqrt()),
        1 => force(value.clone().cbrt()),
        2 => force(
            value
                .clone()
                .root_n(u32::from(input.exponent.unsigned_abs() % 8) + 1),
        ),
        3 => force(value.clone().exp()),
        4 => force(value.clone().ln()),
        5 => force(value.clone().ln_1p()),
        6 => force(value.clone().log10()),
        7 => force(value.clone().log2()),
        8 => force(value.clone().tan()),
        9 => force(value.clone().asin()),
        10 => force(value.clone().acos()),
        11 => force(value.clone().atan()),
        12 => force(value.clone().asinh()),
        13 => force(value.clone().acosh()),
        14 => force(value.clone().atanh()),
        15 => force(value.clone().erfcx()),
        16 => force(value.clone().normal_sf()),
        17 => force(value.clone().qnorm()),
        18 => force(value.clone().gamma()),
        19 => force(value.clone().lgamma()),
        20 => {
            let exponent = i64::from(input.exponent.clamp(-8, 8));
            let machine = value.clone().powi_i64(exponent);
            let arbitrary = value.clone().powi(BigInt::from(exponent));
            assert_eq!(machine, arbitrary);
            force(machine);
        }
        21 => force(
            value
                .clone()
                .pow_rational(Rational::fraction(i64::from(input.exponent), 7).unwrap()),
        ),
        22 => {
            let result = value.clone().sin();
            let _ = result.to_f64_lossy();
            let _ = value.clone().cos().to_f64_lossy();
        }
        _ => {
            let _ = value.clone().erf().to_f64_lossy();
            let _ = value.erfc().to_f64_lossy();
        }
    }
});
