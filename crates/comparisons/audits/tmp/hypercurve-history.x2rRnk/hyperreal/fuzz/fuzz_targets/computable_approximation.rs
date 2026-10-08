//! Fuzz direct Computable graph construction and repeatable refinement.

#![no_main]

use arbitrary::Arbitrary;
use hyperreal::{Computable, Rational};
use libfuzzer_sys::fuzz_target;

#[derive(Clone, Copy, Debug, Arbitrary)]
struct Input {
    a: i16,
    b: i16,
    denominator: u8,
    selector: u8,
}

fuzz_target!(|input: Input| {
    let denominator = u64::from(input.denominator) + 1;
    let a = Computable::rational(
        Rational::fraction(i64::from(input.a) % 65, denominator).expect("positive denominator"),
    );
    let b = Computable::rational(
        Rational::fraction(i64::from(input.b) % 65, denominator).expect("positive denominator"),
    );

    let value = match input.selector % 14 {
        0 => a.clone().add(b.clone()),
        1 => a.clone().multiply(b.clone()),
        2 => a.clone().square(),
        3 => a.clone().sin(),
        4 => a.clone().cos(),
        5 => a.clone().tan(),
        6 => a.clone().exp(),
        7 => a.clone().atan(),
        8 => a.clone().asinh(),
        9 => a.clone().erf(),
        10 => a.clone().erfc(),
        11 => a.clone().pnorm(),
        12 => Computable::normal_interval(a.clone(), b.clone()),
        _ => a.clone().negate(),
    };

    let coarse = value.approx(-32);
    assert_eq!(coarse, value.approx(-32));
    let fine = value.approx(-96);
    assert_eq!(fine, value.approx(-96));
    let _ = value.structural_facts();
    let _ = value.zero_status();
    let _ = value.sign_until(-96);
});
