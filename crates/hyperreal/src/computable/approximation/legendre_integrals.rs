// See docs/admission-legendre-integrals.md for the integral identities.
// Completed one-unit phase approximations suffice for the closed principal chart.
/// Construct a principal-chart Legendre expression for amplitude in `[-pi/2,pi/2]` and c=exp(log_complement)>0.
/// Put S=sin(amplitude), C=cos(amplitude), Y=C^2+c*S^2. F is
/// `S*RF(C^2,Y,1)`. E is `S*C/sqrt(Y) + c*S^3*(RD(C^2,Y,1)+RD(C^2,1,Y))/3`.
/// Within the chart C is nonnegative and Y is positive, so E's nonzero terms
/// have S's sign. The formulas extend to both endpoints: C=0 leaves admitted
/// Carlson arguments and the signed complete value.
///
/// The caller supplies the chart and valid finite children; this helper neither
/// reduces nor validates the amplitude. Carlson receives encoded root/log
/// arguments. The exact expression retains its sources; signal propagation,
/// child approximation and resource limits are inherited from the constructors.
fn elliptic_legendre_principal(
    amplitude: Computable,
    log_complement: Computable,
    second: bool,
) -> Computable {
    let sine = amplitude.clone().sin();
    let cosine = amplitude.cos();
    let complement = log_complement.exp();
    let sine_squared = sine.clone().square();
    let y = cosine
        .clone()
        .square()
        .add(complement.clone().multiply(sine_squared.clone()));
    // y = cos²(r) + c*sin²(r) > 0 for every real r and c > 0.
    let log_y = y.clone().ln();
    let first =
        Computable::elliptic_carlson([cosine.clone(), log_y.clone(), Computable::zero()], second);
    if !second {
        return sine.multiply(first);
    }
    let other = Computable::elliptic_carlson([cosine.clone(), Computable::zero(), log_y], true);
    let correction = complement
        .multiply(sine_squared)
        .multiply(first.add(other))
        .multiply(Computable::rational(
            Rational::fraction(1, 3).expect("nonzero denominator"),
        ));
    sine.multiply(cosine.multiply(y.sqrt().inverse()).add(correction))
}

/// Evaluate an unwrapped F/E node by quarter-period reduction, returning `2^p` units.
/// The payload is `[amplitude, log(1-m)]`. Sample 2*amplitude/pi at precision
/// zero to choose an integer n; the completed one-unit approximation contract
/// gives a remainder in the closed principal chart. No nearest-integer, floor
/// or strict-less-than-one error decision is required. The original amplitude
/// remains in the retained subtraction, not replaced by its sample.
///
/// Add n complete integrals. For odd n, negate the local log complement and
/// multiply the principal F by exp(-log(c)/2), or E by exp(log(c)/2); even n
/// uses c directly. These identities hold for signed n and both chart endpoints.
/// Evaluate the resulting exact expression with the supplied signal at p.
///
/// Check cancellation before and after sampling the phase so an interrupted
/// sample cannot serve as a chart witness. An early zero is an abort placeholder;
/// the outer caller must reject interrupted publication. No later construction
/// step adds a stop poll here, and child precision, work and allocation limits
/// remain applicable without a new cumulative budget.
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
    let phase = amplitude
        .clone()
        .multiply(Computable::pi_inverse_constant())
        .shift_left(1)
        .approx_signal(signal, 0);
    // Never use an interrupted approximation as a principal-chart witness.
    if should_stop(signal) {
        return BigInt::zero();
    }
    let odd = !(&phase & BigInt::one()).is_zero();
    let periods = Computable::rational(Rational::from_bigint(phase));
    let remainder = amplitude
        .clone()
        .add(periods.clone().multiply(half_pi).negate());
    let local = elliptic_legendre_principal(
        remainder,
        if odd {
            log_complement.clone().negate()
        } else {
            log_complement.clone()
        },
        second,
    );
    let local = if odd {
        let exponent = log_complement.clone().shift_left(-1);
        local.multiply(if second { exponent } else { exponent.negate() }.exp())
    } else {
        local
    };
    let complete = Computable::elliptic_legendre_complete(log_complement.clone(), second);
    crate::trace_dispatch!(
        "computable",
        "elliptic_legendre",
        "certified-period-reduction"
    );
    periods
        .multiply(complete)
        .add(local)
        .approx_signal(signal, p)
}
