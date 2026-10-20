fn jacobi(signal: &Option<Signal>, arguments: &[Computable; 2], kind: usize, p: Precision) -> BigInt {
    if should_stop(signal) { return BigInt::zero(); }
    let [mut amplitude, mut log_complement] = arguments.clone();
    if let Some(signal) = signal {
        amplitude.abort(signal.clone());
        log_complement.abort(signal.clone());
    }
    // The quotient is approximated as one exact expression. Dividing two
    // fixed-precision point estimates would lose arbitrarily many periods
    // for huge amplitudes. Its strict error <1 proves |residual| < pi/2.
    let phase = amplitude.clone().multiply(Computable::pi_inverse_constant()).shift_left(1)
        .approx_signal(signal, 0);
    if should_stop(signal) { return BigInt::zero(); }
    let quadrant = (&phase & BigInt::from(3)).to_usize().expect("quadrant in 0..4");
    let residual = amplitude.add(Computable::pi().shift_left(-1)
        .multiply(Computable::rational(Rational::from_bigint(phase))).negate());
    let sine = residual.clone().sin();
    let cosine = residual.cos();
    let (sine, cosine) = match quadrant {
        0 => (sine, cosine),
        1 => (cosine, sine.negate()),
        2 => (sine.negate(), cosine.negate()),
        _ => (cosine.negate(), sine),
    };
    let value = match kind {
        0 => sine,
        1 => cosine,
        // A positive sum: individual components never use 1-sn² or 1-m sn².
        2 => cosine.square().add(
            log_complement.exp().multiply(sine.square()),
        ).sqrt(),
        _ => unreachable!("three Jacobi components"),
    };
    if should_stop(signal) { return BigInt::zero(); }
    value.approx_signal(signal, p)
}
