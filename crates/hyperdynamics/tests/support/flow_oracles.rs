use hyperreal::Rational;

// For 0<=t<=3/2, terms decrease from degree two onward. After twelve terms the
// next (positive) term bounds the alternating tail; all operations here are rational.
pub fn alternating_trig(t: &Rational, sine: bool) -> (Rational, Rational) {
    assert!(*t >= Rational::zero() && *t <= Rational::from(3) / Rational::from(2));
    let mut term = if sine { t.clone() } else { Rational::one() };
    let mut sum = Rational::zero();
    for k in 0..12 {
        sum = sum + &term;
        let a = 2 * k + if sine { 2 } else { 1 };
        term = -term * t * t / Rational::from(a * (a + 1));
    }
    assert!(!term.is_negative());
    (sum.clone(), sum + term)
}
