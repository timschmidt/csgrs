//! Exact value transfer to the independent GMP oracle; no floating conversion.
use hyperreal::Rational;
use rug::Rational as Mpq;

pub fn mpq(value: &Rational) -> Mpq {
    let numerator: rug::Integer = value.numerator().to_string().parse().unwrap();
    let denominator: rug::Integer = value.denominator().to_string().parse().unwrap();
    Mpq::from((
        if value.is_negative() {
            -numerator
        } else {
            numerator
        },
        denominator,
    ))
}
