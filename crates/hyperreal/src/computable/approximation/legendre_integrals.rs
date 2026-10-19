// See docs/admission-legendre-integrals.md. The principal helper is private:
// its caller proves |amplitude| < pi/2 by a strict-error near-integer reduction.
fn elliptic_legendre_principal(
    amplitude: Computable,
    log_complement: Computable,
    second: bool,
) -> Computable {
    let sine = amplitude.clone().sin();
    let cosine = amplitude.cos();
    let complement = log_complement.exp();
    let sine_squared = sine.clone().square();
    let y = cosine.clone().square().add(complement.clone().multiply(sine_squared.clone()));
    // y = cos²(r) + c*sin²(r) > 0 for every real r and c > 0.
    let log_y = y.clone().ln();
    let first = Computable::elliptic_carlson(
        [cosine.clone(), log_y.clone(), Computable::zero()],
        second,
    );
    if !second {
        return sine.multiply(first);
    }
    let other = Computable::elliptic_carlson(
        [cosine.clone(), Computable::zero(), log_y],
        true,
    );
    let correction = complement.multiply(sine_squared).multiply(first.add(other)).multiply(
        Computable::rational(Rational::fraction(1, 3).expect("nonzero denominator")),
    );
    sine.multiply(cosine.multiply(y.sqrt().inverse()).add(correction))
}

fn elliptic_legendre(
    signal: &Option<Signal>,
    arguments: &[Computable; 2],
    second: bool,
    p: Precision,
) -> BigInt {
    if should_stop(signal) {
        return BigInt::zero();
    }
    let [amplitude, log_complement] = arguments;
    let half_pi = Computable::pi().shift_left(-1);
    let phase = amplitude.clone().multiply(Computable::pi_inverse_constant()).shift_left(1)
        .approx_signal(signal, 0);
    // Never use an interrupted approximation as a principal-chart witness.
    if should_stop(signal) {
        return BigInt::zero();
    }
    let odd = !(&phase & BigInt::one()).is_zero();
    let periods = Computable::rational(Rational::from_bigint(phase));
    let remainder = amplitude.clone().add(periods.clone().multiply(half_pi).negate());
    let local = elliptic_legendre_principal(
        remainder,
        if odd { log_complement.clone().negate() } else { log_complement.clone() },
        second,
    );
    let local = if odd {
        let exponent = log_complement.clone().shift_left(-1);
        local.multiply(if second { exponent } else { exponent.negate() }.exp())
    } else {
        local
    };
    let complete = Computable::elliptic_legendre_complete(log_complement.clone(), second);
    crate::trace_dispatch!("computable", "elliptic_legendre", "certified-period-reduction");
    periods.multiply(complete).add(local).approx_signal(signal, p)
}
