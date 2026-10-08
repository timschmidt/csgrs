use hyperreal::{Rational, Real};

fn main() {
    let half = Real::new(Rational::fraction(1, 2).unwrap());
    let eighth = Real::new(Rational::fraction(1, 8).unwrap());
    let alpha = half.clone().sqrt().unwrap();
    let selected = -&half + (&alpha + &half * &half).sqrt().unwrap();
    let lower = &selected - &eighth;
    let upper = &selected + &eighth;
    let midpoint = Real::average_pair(&lower, &upper);
    let coefficients = [&alpha * Real::pi(), -Real::pi(), -Real::pi()];
    for (name, point) in [
        ("direct", selected.clone()),
        ("average", midpoint.clone()),
        ("average_plus_zero", &midpoint + Real::zero()),
        ("zero_plus_average", Real::zero() + &midpoint),
        ("distributed_average", &half * &lower + &half * &upper),
        ("fused_average", Real::mul_add(&half, &lower, &(&half * &upper))),
    ] {
        let (leading, rest) = coefficients.split_last().unwrap();
        let old = rest.iter().rev().fold(leading.clone(), |value, coefficient| {
            value * &point + coefficient
        });
        println!(
            "{name}: old={:?}, native={:?}, difference={:?}",
            old.certified_sign_until(-512).sign(),
            Real::eval_poly(&coefficients, &point).certified_sign_until(-512).sign(),
            (&point - &selected).certified_sign_until(-512).sign(),
        );
    }
}
