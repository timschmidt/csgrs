use hyperreal::{Rational, Real, ZeroKnowledge};

#[test]
fn quadratic_normal_form_recovers_values_after_independent_radical_cancellation() {
    let two = Real::from(2).sqrt().unwrap();
    let three = Real::from(3).sqrt().unwrap();
    let mixed = &two + &three;
    let six = ((&mixed * &mixed - Real::from(5)) / Real::from(2)).unwrap();
    for constant in [-9, 0, 7] {
        for scale in [-5, 1, 4] {
            let value = Real::from(constant) + Real::from(scale) * &six;
            let (a, b, d) = value.exact_quadratic_normal_form().unwrap();
            assert_eq!(a, Rational::from(constant));
            assert_eq!(&b * &b * &d, Rational::from(scale * scale * 6));
            assert_eq!(b.sign(), Rational::from(scale).sign());
            let reconstructed = Real::new(a) + Real::new(b) * Real::new(d).sqrt().unwrap();
            assert_eq!((reconstructed - value).zero_status(), ZeroKnowledge::Zero);
        }
    }
}

#[test]
fn quadratic_normal_form_handles_rational_scale_and_declines_other_exact_values() {
    let root = Real::from(5).sqrt().unwrap();
    let value = ((Real::from(-3) + root) / Real::from(7)).unwrap();
    let square = &value * &value;
    let (a, b, d) = square.exact_quadratic_normal_form().unwrap();
    assert_eq!(a, Rational::fraction(2, 7).unwrap());
    assert_eq!(&b * &b * d, Rational::fraction(180, 2401).unwrap());
    assert!(b < Rational::zero());

    let rational = (Real::from(11) / Real::from(13)).unwrap();
    assert_eq!(
        rational.exact_quadratic_normal_form(),
        Some((
            Rational::fraction(11, 13).unwrap(),
            Rational::zero(),
            Rational::zero()
        )),
    );
    for value in [
        Real::pi(),
        Real::e().sin(),
        Real::from(2).sqrt().unwrap() + Real::from(3).sqrt().unwrap(),
    ] {
        assert!(value.exact_quadratic_normal_form().is_none());
    }
}
