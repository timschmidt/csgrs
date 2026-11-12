//! Differential inputs for the unit-bearing measurement arithmetic migration.
use hyperreal::Rational;

/// Independently reconstructed inputs, including large exact rational data.
pub fn sample(index: usize) -> (Rational, Rational) {
    let ratio = |a, b| Rational::fraction(a, b).unwrap();
    match index % 8 {
        0 => (0.into(), 0.into()),
        1 => (10.into(), ratio(1, 20)),
        2 => (ratio(-17, 3), ratio(23, 7)),
        3 => (ratio(1, 1 << 30), ratio(3, 1 << 29)),
        4 => (ratio(i64::MAX, 13), ratio(i64::MAX - 2, 17)),
        5 => {
            let n: Rational = "123456789012345678901234567890123456789012345678901"
                .parse()
                .unwrap();
            (n.clone() / Rational::from(19), n / Rational::from(23))
        }
        6 => {
            let n: Rational = format!("1{}7", "0".repeat(1200)).parse().unwrap();
            (n.clone() / Rational::from(29), n / Rational::from(31))
        }
        _ => (ratio(11, 13), ratio(-1, 17)),
    }
}
