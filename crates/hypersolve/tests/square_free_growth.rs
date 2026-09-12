use hyperreal::{Rational, Real};
use hypersolve::{PredicatePolicy, square_free_part};
use num::BigInt;

#[test]
fn wide_repeated_factors_preserve_the_exact_square_free_scale() {
    let b = (BigInt::from(1) << 128_usize) + 13_u8;
    let a = -(&b * 2_u8) + 1_u8;
    let d = (BigInt::from(1) << 96_usize) + 19_u8;
    let c = &d + 7_u8;
    let mut polynomial = vec![BigInt::from(1)];
    for (constant, linear, multiplicity) in [(&a, &b, 12), (&c, &d, 8)] {
        for _ in 0..multiplicity {
            let mut product = vec![BigInt::from(0); polynomial.len() + 1];
            for (power, coefficient) in polynomial.iter().enumerate() {
                product[power] += coefficient * constant;
                product[power + 1] += coefficient * linear;
            }
            polynomial = product;
        }
    }
    let scale = Rational::fraction(-5, 7).unwrap();
    let input = polynomial
        .into_iter()
        .map(|coefficient| Real::new(Rational::from_bigint(coefficient) * &scale))
        .collect();
    // Dividing by the monic gcd retains the input leading coefficient.
    let expected_scale = &scale * Rational::from_bigint(b.pow(11) * d.pow(7));
    let expected = [&a * &c, &a * &d + &b * &c, &b * &d]
        .into_iter()
        .map(|coefficient| Real::new(Rational::from_bigint(coefficient) * &expected_scale))
        .collect::<Vec<_>>();
    assert_eq!(
        square_free_part(input, PredicatePolicy::STRICT),
        Some(expected)
    );
}
