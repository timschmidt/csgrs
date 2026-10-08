use hyperreal::{Rational, Real};
use std::hash::{Hash, Hasher};

fn hash(value: &Rational) -> u64 {
    let mut state = std::collections::hash_map::DefaultHasher::new();
    value.hash(&mut state);
    state.finish()
}

fn main() {
    let half = Real::new(Rational::fraction(1, 2).unwrap());
    // P(t) = 2/3 + t^4*(t-1/2), hence P(1/2) = 2/3 exactly.
    let coefficients = [
        Real::new(Rational::fraction(2, 3).unwrap()),
        Real::zero(),
        Real::zero(),
        Real::zero(),
        -&half,
        Real::one(),
    ];
    let value = Real::eval_poly(&coefficients, &half);
    let actual = value.exact_rational_ref().unwrap() + Rational::one();
    let expected = Rational::fraction(5, 3).unwrap();
    let equal = actual == expected;
    let canonical_parts = actual.numerator() == expected.numerator()
        && actual.denominator() == expected.denominator();
    let equal_hashes = hash(&actual) == hash(&expected);
    println!("numeric_equal={equal}, canonical_parts={canonical_parts}, equal_hashes={equal_hashes}");
    assert!(equal && canonical_parts && equal_hashes);
}
