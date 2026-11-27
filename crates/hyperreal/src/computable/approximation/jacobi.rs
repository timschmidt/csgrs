/// Evaluate one retained Jacobi component from a finite amplitude and log complement.
/// The payload is `[phi, ln(c)]`, c>0; kinds 0/1/2 mean sin(phi), cos(phi),
/// and the positive square root of cos(phi)^2+c*sin(phi)^2. The inverse from
/// Jacobi argument u to phi is already retained by the caller, not solved here.
/// This kernel trusts defined real children and the component selector.
///
/// Sample the exact quotient 2*phi/pi at precision zero, obtaining integer n
/// with at most one unit of error. The exact residual phi-n*pi/2 therefore lies
/// in the closed half-pi chart. Rotate its sine/cosine by n modulo four, including
/// negative n. For dn use a positive weighted square sum rather than subtracting
/// nearly equal terms; the expression stays valid at chart endpoints as well.
///
/// Attach an explicit signal to both cloned children when present, poll before
/// and after phase selection and before final evaluation, and delegate the final
/// request at binary precision p to the constructed exact expression. Stop paths return zero
/// for the caller to reject; they do not certify a zero component. A kind outside
/// 0..=2 panics unless an earlier stop returns. Child refinement and integer/graph
/// growth have no new total-work, heap-byte or cancellation-latency bound here.
fn jacobi(
    signal: &Option<Signal>,
    arguments: &[Computable; 2],
    kind: usize,
    p: Precision,
) -> BigInt {
    if should_stop(signal) {
        return BigInt::zero();
    }
    let [mut amplitude, mut log_complement] = arguments.clone();
    if let Some(signal) = signal {
        amplitude.abort(signal.clone());
        log_complement.abort(signal.clone());
    }
    // The quotient is approximated as one exact expression. Dividing two
    // fixed-precision point estimates would lose arbitrarily many periods
    // for huge amplitudes. Its error <=1 proves |residual| <= pi/2.
    let phase = amplitude
        .clone()
        .multiply(Computable::pi_inverse_constant())
        .shift_left(1)
        .approx_signal(signal, 0);
    if should_stop(signal) {
        return BigInt::zero();
    }
    let quadrant = (&phase & BigInt::from(3))
        .to_usize()
        .expect("quadrant in 0..4");
    let residual = amplitude.add(
        Computable::pi()
            .shift_left(-1)
            .multiply(Computable::rational(Rational::from_bigint(phase)))
            .negate(),
    );
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
        2 => cosine
            .square()
            .add(log_complement.exp().multiply(sine.square()))
            .sqrt(),
        _ => unreachable!("three Jacobi components"),
    };
    if should_stop(signal) {
        return BigInt::zero();
    }
    value.approx_signal(signal, p)
}
