fn cos(signal: &Option<Signal>, c: &Computable, p: Precision) -> BigInt {
    // Kernel precondition: |c| < 1. Argument reduction and exact pi-multiple
    // handling happen before this node is constructed. Keeping range reduction
    // outside the Taylor kernel is the same split used by multi-precision
    // sin/cos algorithms.
    if p >= 1 {
        return signed::ONE.deref().clone();
    }
    let iterations_needed = -p / 2 + 4;

    if should_stop(signal) {
        return signed::ONE.deref().clone();
    }

    //  Claim: each intermediate term is accurate
    //  to 2*2^calc_precision.
    //  Total rounding error in series computation is
    //  2*iterations_needed*2^calc_precision,
    //  exclusive of error in op.
    let calc_precision = p - bound_log2(2 * iterations_needed) - 4; // for error in op, truncation.
    let op_prec = p - 2;
    let op_appr = c.approx_signal(signal, op_prec);
    let op_squared = scale(&op_appr * &op_appr, op_prec);

    // Error in argument results in error of < 1/4 ulp.
    // Cumulative arithmetic rounding error is < 1/16 ulp.
    // Series truncation error < 1/16 ulp.
    // Final rounding error is <= 1/2 ulp.
    // Thus final error is < 1 ulp.

    // Keep the truncation guard allocation-free across Taylor iterations.
    let max_trunc_error = BigUint::one()
        << usize::try_from(p - 4 - calc_precision).expect("truncation shift is nonnegative");
    let mut n = 0;
    let mut current_term = signed::ONE.deref() << (-calc_precision);
    let mut current_sum = current_term.clone();

    while current_term.magnitude() > &max_trunc_error {
        if should_stop(signal) {
            break;
        }
        n += 2;

        /* current_term = - current_term * op_squared / n * (n - 1)   */
        current_term = scale(current_term * &op_squared, op_prec);
        current_term /= -(n * (n - 1));

        current_sum += &current_term;
    }
    scale(current_sum, calc_precision - p)
}

// Compute cosine of an exact rational |r| < 1 without allocating a temporary
// Ratio node. This preserves the same Taylor algorithm as `cos` while keeping
// the stored rational symbolic until the final requested precision. 2026-05
// numerical_micro targeted runs showed the direct rational feed removes cold
// approximation setup from small exact-rational trig rows without changing the
// cached path.
fn cos_rational(signal: &Option<Signal>, r: &Rational, p: Precision) -> BigInt {
    if p >= 1 {
        return signed::ONE.deref().clone();
    }
    let iterations_needed = -p / 2 + 4;

    if should_stop(signal) {
        return signed::ONE.deref().clone();
    }

    let calc_precision = p - bound_log2(2 * iterations_needed) - 5;
    let op_prec = p - 2;
    let op_appr = ratio(r, op_prec);
    let op_squared = scale(&op_appr * &op_appr, op_prec);

    let max_trunc_error = BigUint::one()
        << usize::try_from(p - 4 - calc_precision).expect("truncation shift is nonnegative");
    let mut n = 0;
    let mut current_term = signed::ONE.deref() << (-calc_precision);
    let mut current_sum = current_term.clone();

    while current_term.magnitude() > &max_trunc_error {
        if should_stop(signal) {
            break;
        }
        n += 2;

        current_term = scale(current_term * &op_squared, op_prec);
        current_term /= -(n * (n - 1));

        current_sum += &current_term;
    }
    scale(current_sum, calc_precision - p)
}

fn large_rational_half_pi_multiple(signal: &Option<Signal>, r: &Rational) -> BigInt {
    // Deferred large-rational trig needs the same nearest-half-pi quotient as
    // Computable::sin/cos, but rebuilding a Ratio node just to call the generic
    // reducer was the remaining hot path in exact 1e6/1e30 benchmarks. This is
    // the exact-rational Payne-Hanek quotient estimate from the constructor
    // layer, with the residual correction performed directly from cached pi.
    if let Some(multiple) = fixed_small_large_rational_half_pi_multiple(r) {
        return multiple;
    }

    let mut multiple = Computable::half_pi_multiple_exact_rational(r)
        .unwrap_or_else(|| Computable::rational(r.clone()).half_pi_multiple());
    let rough_appr = large_rational_half_pi_residual(signal, r, &multiple, -1);

    if rough_appr >= *crate::computable::signed::TWO {
        multiple += 1;
    } else if rough_appr <= -crate::computable::signed::TWO.deref().clone() {
        multiple -= 1;
    }

    multiple
}

fn fixed_small_large_rational_half_pi_multiple(r: &Rational) -> Option<BigInt> {
    let msd = r.msd_exact();

    // For |r| in [7/2, 39/10], the nearest half-pi multiple is exactly +/-2.
    // The interval lies strictly between 3*pi/4 and 5*pi/4. It covers the
    // historical promoted tangent tail immediately below the old 79/20
    // deferred-reduction boundary, while retaining an exact sector certificate.
    if msd == Some(1) {
        if r >= SEVEN_HALVES_RATIONAL.deref() && r <= THIRTY_NINE_TENTHS_RATIONAL.deref() {
            crate::trace_dispatch!("computable_approx", "trig", "fixed-half-pi-multiple-2");
            return Some(BigInt::from(2));
        }
        if r <= NEG_SEVEN_HALVES_RATIONAL.deref() && r >= NEG_THIRTY_NINE_TENTHS_RATIONAL.deref() {
            crate::trace_dispatch!("computable_approx", "trig", "fixed-half-pi-multiple-neg2");
            return Some(BigInt::from(-2));
        }
    }

    // For |r| in [79/20, 4], the nearest half-pi multiple is exactly +/-3.
    // This catches tan rows just below the large-rational threshold while
    // staying above 5*pi/4.
    if r >= SEVENTY_NINE_TWENTIETHS_RATIONAL.deref() && r <= FOUR_RATIONAL.deref() {
        crate::trace_dispatch!("computable_approx", "trig", "fixed-half-pi-multiple-3");
        return Some(BigInt::from(3));
    }
    if r <= NEG_SEVENTY_NINE_TWENTIETHS_RATIONAL.deref() && r >= NEG_FOUR_RATIONAL.deref() {
        crate::trace_dispatch!("computable_approx", "trig", "fixed-half-pi-multiple-neg3");
        return Some(BigInt::from(-3));
    }

    // For |r| in [4, 27/5], the nearest half-pi multiple is exactly +/-3.
    // The upper bound is kept conservatively below 7*pi/4, so the shortcut
    // remains a cheap certificate rather than a fresh pi-dependent comparison.
    if msd != Some(2) {
        return None;
    }
    if r >= FOUR_RATIONAL.deref() && r <= TWENTY_SEVEN_FIFTHS_RATIONAL.deref() {
        crate::trace_dispatch!("computable_approx", "trig", "fixed-half-pi-multiple-3");
        return Some(BigInt::from(3));
    }
    if r <= NEG_FOUR_RATIONAL.deref() && r >= NEG_TWENTY_SEVEN_FIFTHS_RATIONAL.deref() {
        crate::trace_dispatch!("computable_approx", "trig", "fixed-half-pi-multiple-neg3");
        return Some(BigInt::from(-3));
    }
    // For |r| in [11/2, 7], the nearest half-pi multiple is exactly +/-4.
    // The lower bound is above 7*pi/4 and the upper bound is below 9*pi/4,
    // covering the next promoted tan cluster without a pi-dependent probe.
    if r >= ELEVEN_HALVES_RATIONAL.deref() && r <= SEVEN_RATIONAL.deref() {
        crate::trace_dispatch!("computable_approx", "trig", "fixed-half-pi-multiple-4");
        return Some(BigInt::from(4));
    }
    if r <= NEG_ELEVEN_HALVES_RATIONAL.deref() && r >= NEG_SEVEN_RATIONAL.deref() {
        crate::trace_dispatch!("computable_approx", "trig", "fixed-half-pi-multiple-neg4");
        return Some(BigInt::from(-4));
    }
    // For |r| in [7, 17/2], the nearest half-pi multiple is exactly +/-5.
    // This sits between 9*pi/4 and 11*pi/4 and covers the next promoted tan
    // cluster without opening the full large-rational Payne-Hanek path.
    if r >= SEVEN_RATIONAL.deref() && r <= SEVENTEEN_HALVES_RATIONAL.deref() {
        crate::trace_dispatch!("computable_approx", "trig", "fixed-half-pi-multiple-5");
        return Some(BigInt::from(5));
    }
    if r <= NEG_SEVEN_RATIONAL.deref() && r >= NEG_SEVENTEEN_HALVES_RATIONAL.deref() {
        crate::trace_dispatch!("computable_approx", "trig", "fixed-half-pi-multiple-neg5");
        return Some(BigInt::from(-5));
    }
    None
}

fn large_rational_half_pi_residual(
    signal: &Option<Signal>,
    r: &Rational,
    multiple: &BigInt,
    p: Precision,
) -> BigInt {
    // Approximate r - multiple*pi/2 at precision p without allocating
    // Add(Multiply(Pi, k), Ratio(r)). This is the performance-critical part of
    // the direct large-rational kernels; it keeps the mathematical reduction
    // identical to the generic path while avoiding expression graph setup.
    let extra = 3;
    let work_precision = p - extra;
    let rational = ratio(r, work_precision);
    if multiple == signed::FOUR.deref() || multiple == NEG_FOUR_BIGINT.deref() {
        crate::trace_dispatch!("computable_approx", "trig", "fixed-half-pi-residual-two-pi");
        let pi_precision = work_precision - 6;
        let pi = Computable::pi().approx_signal(signal, pi_precision);
        let two_pi = scale(pi, pi_precision - work_precision + 1);
        let half_pi_multiple = if multiple.sign() == Sign::Minus {
            -two_pi
        } else {
            two_pi
        };
        return scale(rational - half_pi_multiple, -extra);
    }
    let multiple_msd = i32::try_from(multiple.magnitude().bits().saturating_sub(1))
        .expect("large trig quotient bits should fit in i32");
    let pi_precision = work_precision - multiple_msd - 4;
    let pi = Computable::pi().approx_signal(signal, pi_precision);
    let half_pi_multiple = scale(pi * multiple, pi_precision - work_precision - 1);
    scale(rational - half_pi_multiple, -extra)
}

fn large_rational_quadrant(multiple: &BigInt) -> BigInt {
    ((multiple % crate::computable::signed::FOUR.deref()) + crate::computable::signed::FOUR.deref())
        % crate::computable::signed::FOUR.deref()
}

fn large_rational_quarter_pi_residual(
    signal: &Option<Signal>,
    r: &Rational,
    multiple: &BigInt,
    quarter_sign: i8,
    p: Precision,
) -> BigInt {
    let extra = 3;
    let work_precision = p - extra;
    let rational = ratio(r, work_precision);
    let quarter_multiple: BigInt = (multiple << 1) + BigInt::from(quarter_sign);
    let multiple_msd = i32::try_from(quarter_multiple.magnitude().bits().saturating_sub(1))
        .expect("large trig quotient bits should fit in i32");
    let pi_precision = work_precision - multiple_msd - 5;
    let pi = Computable::pi().approx_signal(signal, pi_precision);
    let quarter_pi_multiple = scale(pi * quarter_multiple, pi_precision - work_precision - 2);
    scale(rational - quarter_pi_multiple, -extra)
}

fn sin_cos_scaled_argument(
    signal: &Option<Signal>,
    op_appr: BigInt,
    op_prec: Precision,
    p: Precision,
) -> (BigInt, BigInt) {
    if p >= 1 {
        return (Zero::zero(), signed::ONE.deref().clone());
    }
    let iterations_needed = -p / 2 + 4;

    if should_stop(signal) {
        return (Zero::zero(), signed::ONE.deref().clone());
    }

    let calc_precision = p - bound_log2(2 * iterations_needed) - 5;
    let op_squared = scale(&op_appr * &op_appr, op_prec);

    let max_trunc_error = BigUint::one()
        << usize::try_from(p - 4 - calc_precision).expect("truncation shift is nonnegative");

    let mut sin_n = 1;
    let mut sin_term = scale(op_appr, op_prec - calc_precision);
    let mut sin_sum = sin_term.clone();

    while sin_term.magnitude() > &max_trunc_error {
        if should_stop(signal) {
            break;
        }
        sin_n += 2;
        sin_term = scale(sin_term * &op_squared, op_prec);
        sin_term /= -(sin_n * (sin_n - 1));
        sin_sum += &sin_term;
    }

    let mut cos_n = 0;
    let mut cos_term = signed::ONE.deref() << (-calc_precision);
    let mut cos_sum = cos_term.clone();

    while cos_term.magnitude() > &max_trunc_error {
        if should_stop(signal) {
            break;
        }
        cos_n += 2;
        cos_term = scale(cos_term * &op_squared, op_prec);
        cos_term /= -(cos_n * (cos_n - 1));
        cos_sum += &cos_term;
    }

    (
        scale(sin_sum, calc_precision - p),
        scale(cos_sum, calc_precision - p),
    )
}

fn divide_scaled(numerator: BigInt, denominator: BigInt, p: Precision) -> BigInt {
    let abs_denominator = denominator.abs();
    if abs_denominator.is_zero() {
        panic!("ArithmeticException");
    }

    let scaled_numerator = if denominator.sign() == Sign::Minus {
        -numerator << -p
    } else {
        numerator << -p
    };
    let adjustment = &abs_denominator >> 1;

    if scaled_numerator.sign() == Sign::Minus {
        let rounded: BigInt = ((-scaled_numerator) + adjustment) / abs_denominator;
        -rounded
    } else {
        (scaled_numerator + adjustment) / abs_denominator
    }
}

fn divide_scaled_refining<F>(
    signal: &Option<Signal>,
    p: Precision,
    mut components: F,
) -> BigInt
where
    F: FnMut(Precision) -> (BigInt, BigInt),
{
    let mut working_precision = p - 8;

    loop {
        let (numerator, denominator) = components(working_precision);
        if !denominator.is_zero() {
            let denominator_msd = working_precision
                + denominator.magnitude().bits() as Precision
                - 1;
            // Quotient sensitivity is proportional to 1/d^2. Each bit that
            // |d| lies below one therefore costs two source bits.
            let required_precision = p + 2 * denominator_msd.min(0) - 8;
            if working_precision <= required_precision {
                return divide_scaled(numerator, denominator, p);
            }
            working_precision = required_precision;
        } else {
            working_precision -= 8;
        }

        if should_stop(signal) {
            return Zero::zero();
        }
        if p - working_precision > 16_384 {
            panic!("ArithmeticException");
        }
    }
}

fn tan_scaled_argument(
    signal: &Option<Signal>,
    op_appr: BigInt,
    op_prec: Precision,
    p: Precision,
) -> BigInt {
    let (sin_appr, cos_appr) = sin_cos_scaled_argument(signal, op_appr, op_prec, p);
    divide_scaled(sin_appr, cos_appr, p)
}

fn tan_large_rational_quarter_pi(
    signal: &Option<Signal>,
    r: &Rational,
    multiple: &BigInt,
    p: Precision,
) -> Option<BigInt> {
    let quadrant = large_rational_quadrant(multiple).to_u8()?;

    let rough_residual = large_rational_half_pi_residual(signal, r, multiple, -8);
    let quarter_sign = if rough_residual >= *QUARTER_PI_TAN_RESIDUAL_THRESHOLD.deref() {
        1
    } else if rough_residual <= -QUARTER_PI_TAN_RESIDUAL_THRESHOLD.deref().clone() {
        -1
    } else {
        return None;
    };

    crate::trace_dispatch!("computable_approx", "tan", "quarter-pi-large-rational");
    let working_prec = p - 8;
    let op_prec = working_prec - 2;
    let delta = large_rational_quarter_pi_residual(signal, r, multiple, quarter_sign, op_prec);
    let tan_delta = tan_scaled_argument(signal, delta, op_prec, working_prec);
    let one = signed::ONE.deref() << -working_prec;
    let direct_tangent_form = (quadrant % 2 == 0) == (quarter_sign > 0);
    let (numerator, denominator) = if direct_tangent_form {
        (one.clone() + &tan_delta, one - tan_delta)
    } else {
        (tan_delta.clone() - &one, one + tan_delta)
    };
    Some(divide_scaled(numerator, denominator, p))
}

fn cos_large_rational_residual(
    signal: &Option<Signal>,
    r: &Rational,
    multiple: &BigInt,
    p: Precision,
) -> BigInt {
    // Same Taylor kernel as cos(|x| < 1), but the argument approximation comes
    // from the direct residual above instead of a child Computable node.
    if p >= 1 {
        return signed::ONE.deref().clone();
    }
    let iterations_needed = -p / 2 + 4;

    if should_stop(signal) {
        return signed::ONE.deref().clone();
    }

    let calc_precision = p - bound_log2(2 * iterations_needed) - 5;
    let op_prec = p - 2;
    let op_appr = large_rational_half_pi_residual(signal, r, multiple, op_prec);
    let op_squared = scale(&op_appr * &op_appr, op_prec);

    let max_trunc_error = BigUint::one()
        << usize::try_from(p - 4 - calc_precision).expect("truncation shift is nonnegative");
    let mut n = 0;
    let mut current_term = signed::ONE.deref() << (-calc_precision);
    let mut current_sum = current_term.clone();

    while current_term.magnitude() > &max_trunc_error {
        if should_stop(signal) {
            break;
        }
        n += 2;
        current_term = scale(current_term * &op_squared, op_prec);
        current_term /= -(n * (n - 1));
        current_sum += &current_term;
    }
    scale(current_sum, calc_precision - p)
}

fn sin_large_rational_residual(
    signal: &Option<Signal>,
    r: &Rational,
    multiple: &BigInt,
    p: Precision,
) -> BigInt {
    // Same Taylor kernel as sin(|x| < 1), fed by the direct large-rational
    // residual to avoid constructing a generic reduced Computable expression.
    if p >= 1 {
        return Zero::zero();
    }
    let iterations_needed = -p / 2 + 4;

    if should_stop(signal) {
        return Zero::zero();
    }

    let calc_precision = p - bound_log2(2 * iterations_needed) - 4;
    let op_prec = p - 2;
    let op_appr = large_rational_half_pi_residual(signal, r, multiple, op_prec);
    let op_squared = scale(&op_appr * &op_appr, op_prec);

    let max_trunc_error = BigUint::one()
        << usize::try_from(p - 4 - calc_precision).expect("truncation shift is nonnegative");
    let mut n = 1;
    let mut current_term = scale(op_appr.clone(), op_prec - calc_precision);
    let mut current_sum = current_term.clone();

    while current_term.magnitude() > &max_trunc_error {
        if should_stop(signal) {
            break;
        }
        n += 2;
        current_term = scale(current_term * &op_squared, op_prec);
        current_term /= -(n * (n - 1));
        current_sum += &current_term;
    }
    scale(current_sum, calc_precision - p)
}

fn sin_cos_large_rational_residual(
    signal: &Option<Signal>,
    r: &Rational,
    multiple: &BigInt,
    p: Precision,
) -> (BigInt, BigInt) {
    if p >= 1 {
        return (Zero::zero(), signed::ONE.deref().clone());
    }
    let iterations_needed = -p / 2 + 4;

    if should_stop(signal) {
        return (Zero::zero(), signed::ONE.deref().clone());
    }

    let calc_precision = p - bound_log2(2 * iterations_needed) - 4;
    let op_prec = p - 2;
    let op_appr = large_rational_half_pi_residual(signal, r, multiple, op_prec);
    let op_squared = scale(&op_appr * &op_appr, op_prec);

    let max_trunc_error = BigUint::one()
        << usize::try_from(p - 4 - calc_precision).expect("truncation shift is nonnegative");

    let mut sin_n = 1;
    let mut sin_term = scale(op_appr, op_prec - calc_precision);
    let mut sin_sum = sin_term.clone();

    while sin_term.magnitude() > &max_trunc_error {
        if should_stop(signal) {
            break;
        }
        sin_n += 2;
        sin_term = scale(sin_term * &op_squared, op_prec);
        sin_term /= -(sin_n * (sin_n - 1));
        sin_sum += &sin_term;
    }

    let mut cos_n = 0;
    let mut cos_term = signed::ONE.deref() << (-calc_precision);
    let mut cos_sum = cos_term.clone();

    while cos_term.magnitude() > &max_trunc_error {
        if should_stop(signal) {
            break;
        }
        cos_n += 2;
        cos_term = scale(cos_term * &op_squared, op_prec);
        cos_term /= -(cos_n * (cos_n - 1));
        cos_sum += &cos_term;
    }

    (
        scale(sin_sum, calc_precision - p),
        scale(cos_sum, calc_precision - p),
    )
}

fn cos_large_rational(signal: &Option<Signal>, r: &Rational, p: Precision) -> BigInt {
    // Construction-included benches pay heavily for eager half-pi reduction on
    // large exact rationals. Use the direct residual kernels here so the public
    // constructor can stay lazy without recursing back through Computable::cos.
    let multiple = large_rational_half_pi_multiple(signal, r);
    match large_rational_quadrant(&multiple).to_u8() {
        Some(0) => cos_large_rational_residual(signal, r, &multiple, p),
        Some(1) => -sin_large_rational_residual(signal, r, &multiple, p),
        Some(2) => -cos_large_rational_residual(signal, r, &multiple, p),
        Some(3) => sin_large_rational_residual(signal, r, &multiple, p),
        _ => unreachable!("quadrant reduction is modulo four"),
    }
}

fn half_pi_minus_rational(signal: &Option<Signal>, r: &Rational, p: Precision) -> BigInt {
    // Specialized residual for exact medium trig inputs. It performs the same
    // guarded subtraction as the generic Add(Offset(pi), -r) path, but without
    // allocating or querying a composite expression on every cold approximation.
    let extra = 2;
    let work_precision = p - extra;
    let half_pi = Computable::pi().approx_signal(signal, work_precision + 1);
    let rational = ratio(r, work_precision);
    scale(half_pi - rational, -extra)
}

// Compute cosine of pi/2 - r for exact 1 <= r < 3/2.
fn cos_half_pi_minus_rational(signal: &Option<Signal>, r: &Rational, p: Precision) -> BigInt {
    if p >= 1 {
        return signed::ONE.deref().clone();
    }
    let iterations_needed = -p / 2 + 4;

    if should_stop(signal) {
        return signed::ONE.deref().clone();
    }

    let calc_precision = p - bound_log2(2 * iterations_needed) - 4;
    let op_prec = p - 2;
    // Compute the exact rational residual directly from cached pi. The generic
    // equivalent would allocate a short Add tree before entering this same series.
    let op_appr = half_pi_minus_rational(signal, r, op_prec);
    let op_squared = scale(&op_appr * &op_appr, op_prec);

    // Keep the truncation guard allocation-free across Taylor iterations.
    let max_trunc_error = BigUint::one()
        << usize::try_from(p - 4 - calc_precision).expect("truncation shift is nonnegative");
    let mut n = 0;
    let mut current_term = signed::ONE.deref() << (-calc_precision);
    let mut current_sum = current_term.clone();

    while current_term.magnitude() > &max_trunc_error {
        if should_stop(signal) {
            break;
        }
        n += 2;

        current_term = scale(current_term * &op_squared, op_prec);
        current_term /= -(n * (n - 1));

        current_sum += &current_term;
    }
    scale(current_sum, calc_precision - p)
}

// Compute sine of |c| < 1
// uses a Taylor series expansion.
fn sin(signal: &Option<Signal>, c: &Computable, p: Precision) -> BigInt {
    // Kernel precondition: |c| < 1. The caller keeps large arguments out of this
    // Taylor loop so huge sin/cos rows spend time in reduction, not series setup.
    // Use a reduced-argument series so huge inputs spend time in reduction, not setup.
    if p >= 1 {
        return Zero::zero();
    }
    let iterations_needed = -p / 2 + 4;

    if should_stop(signal) {
        return Zero::zero();
    }

    //  Claim: each intermediate term is accurate
    //  to 2*2^calc_precision.
    //  Total rounding error in series computation is
    //  2*iterations_needed*2^calc_precision,
    //  exclusive of error in op.
    let calc_precision = p - bound_log2(2 * iterations_needed) - 4; // for error in op, truncation.
    let op_prec = p - 2;
    let op_appr = c.approx_signal(signal, op_prec);
    let op_squared = scale(&op_appr * &op_appr, op_prec);

    // Error in argument results in error of < 1/4 ulp.
    // Cumulative arithmetic rounding error is < 1/16 ulp.
    // Series truncation error < 1/16 ulp.
    // Final rounding error is <= 1/2 ulp.
    // Thus final error is < 1 ulp.

    // Keep the truncation guard allocation-free across Taylor iterations.
    let max_trunc_error = BigUint::one()
        << usize::try_from(p - 4 - calc_precision).expect("truncation shift is nonnegative");
    let mut n = 1;
    let mut current_term = scale(op_appr.clone(), op_prec - calc_precision);
    let mut current_sum = current_term.clone();

    while current_term.magnitude() > &max_trunc_error {
        if should_stop(signal) {
            break;
        }
        n += 2;

        /* current_term = - current_term * op_squared / n * (n - 1)   */
        current_term = scale(current_term * &op_squared, op_prec);
        current_term /= -(n * (n - 1));

        current_sum += &current_term;
    }
    scale(current_sum, calc_precision - p)
}

fn sin_rational(signal: &Option<Signal>, r: &Rational, p: Precision) -> BigInt {
    // Direct exact-rational variant of `sin`. It avoids allocating a temporary
    // Computable leaf for PrescaledSinRational while still rounding the rational
    // argument exactly once at the requested guard precision.
    if p >= 1 {
        return Zero::zero();
    }
    let iterations_needed = -p / 2 + 4;

    if should_stop(signal) {
        return Zero::zero();
    }

    let calc_precision = p - bound_log2(2 * iterations_needed) - 4;
    let op_prec = p - 2;
    let op_appr = ratio(r, op_prec);
    let op_squared = scale(&op_appr * &op_appr, op_prec);

    let max_trunc_error = BigUint::one()
        << usize::try_from(p - 4 - calc_precision).expect("truncation shift is nonnegative");
    let mut n = 1;
    let mut current_term = scale(op_appr.clone(), op_prec - calc_precision);
    let mut current_sum = current_term.clone();

    while current_term.magnitude() > &max_trunc_error {
        if should_stop(signal) {
            break;
        }
        n += 2;

        current_term = scale(current_term * &op_squared, op_prec);
        current_term /= -(n * (n - 1));

        current_sum += &current_term;
    }
    scale(current_sum, calc_precision - p)
}

fn sin_large_rational(signal: &Option<Signal>, r: &Rational, p: Precision) -> BigInt {
    // Same lazy public-construction policy as cosine. The direct residual
    // arithmetic avoids allocating the generic reduced expression tree while
    // preserving the standard quadrant identities.
    let multiple = large_rational_half_pi_multiple(signal, r);
    match large_rational_quadrant(&multiple).to_u8() {
        Some(0) => sin_large_rational_residual(signal, r, &multiple, p),
        Some(1) => cos_large_rational_residual(signal, r, &multiple, p),
        Some(2) => -sin_large_rational_residual(signal, r, &multiple, p),
        Some(3) => -cos_large_rational_residual(signal, r, &multiple, p),
        _ => unreachable!("quadrant reduction is modulo four"),
    }
}

// Compute sine of pi/2 - r for exact 1 <= r < 3/2.
fn sin_half_pi_minus_rational(signal: &Option<Signal>, r: &Rational, p: Precision) -> BigInt {
    if p >= 1 {
        return Zero::zero();
    }
    let iterations_needed = -p / 2 + 4;

    if should_stop(signal) {
        return Zero::zero();
    }

    let calc_precision = p - bound_log2(2 * iterations_needed) - 4;
    let op_prec = p - 2;
    // Compute the exact rational residual directly from cached pi. The generic
    // equivalent would allocate a short Add tree before entering this same series.
    let op_appr = half_pi_minus_rational(signal, r, op_prec);
    let op_squared = scale(&op_appr * &op_appr, op_prec);

    // Keep the truncation guard allocation-free across Taylor iterations.
    let max_trunc_error = BigUint::one()
        << usize::try_from(p - 4 - calc_precision).expect("truncation shift is nonnegative");
    let mut n = 1;
    let mut current_term = scale(op_appr.clone(), op_prec - calc_precision);
    let mut current_sum = current_term.clone();

    while current_term.magnitude() > &max_trunc_error {
        if should_stop(signal) {
            break;
        }
        n += 2;

        current_term = scale(current_term * &op_squared, op_prec);
        current_term /= -(n * (n - 1));

        current_sum += &current_term;
    }
    scale(current_sum, calc_precision - p)
}

fn sin_cos_half_pi_minus_rational(
    signal: &Option<Signal>,
    r: &Rational,
    p: Precision,
) -> (BigInt, BigInt) {
    // Shared complement kernel for cot(pi/2-r): compute the exact-rational
    // residual once, then feed both Taylor sums. This keeps the medium tangent
    // shortcut from paying two pi/rational subtractions per approximation.
    if p >= 1 {
        return (Zero::zero(), signed::ONE.deref().clone());
    }
    let iterations_needed = -p / 2 + 4;

    if should_stop(signal) {
        return (Zero::zero(), signed::ONE.deref().clone());
    }

    let calc_precision = p - bound_log2(2 * iterations_needed) - 4;
    let op_prec = p - 2;
    let op_appr = half_pi_minus_rational(signal, r, op_prec);
    let op_squared = scale(&op_appr * &op_appr, op_prec);

    let max_trunc_error = BigUint::one()
        << usize::try_from(p - 4 - calc_precision).expect("truncation shift is nonnegative");

    let mut sin_n = 1;
    let mut sin_term = scale(op_appr, op_prec - calc_precision);
    let mut sin_sum = sin_term.clone();

    while sin_term.magnitude() > &max_trunc_error {
        if should_stop(signal) {
            break;
        }
        sin_n += 2;
        sin_term = scale(sin_term * &op_squared, op_prec);
        sin_term /= -(sin_n * (sin_n - 1));
        sin_sum += &sin_term;
    }

    let mut cos_n = 0;
    let mut cos_term = signed::ONE.deref() << (-calc_precision);
    let mut cos_sum = cos_term.clone();

    while cos_term.magnitude() > &max_trunc_error {
        if should_stop(signal) {
            break;
        }
        cos_n += 2;
        cos_term = scale(cos_term * &op_squared, op_prec);
        cos_term /= -(cos_n * (cos_n - 1));
        cos_sum += &cos_term;
    }

    (
        scale(sin_sum, calc_precision - p),
        scale(cos_sum, calc_precision - p),
    )
}

fn cot_half_pi_minus_rational(signal: &Option<Signal>, r: &Rational, p: Precision) -> BigInt {
    // tan(r) near pi/2 is cot(pi/2-r). Reusing the direct exact-rational
    // residual for both numerator and denominator avoids the generic
    // PrescaledCot(Offset(pi/2, -r)) expression graph.
    if p >= 1 {
        return Zero::zero();
    }

    divide_scaled_refining(signal, p, |working_precision| {
        let (sin_appr, cos_appr) =
            sin_cos_half_pi_minus_rational(signal, r, working_precision);
        (cos_appr, sin_appr)
    })
}

// Compute tangent of |c| < 1.
// This uses the direct quotient tan(x) = sin(x) / cos(x),
// but computes both approximations locally to avoid building
// separate Computable trees for sin, cos, inverse, and multiply.
fn tan(signal: &Option<Signal>, c: &Computable, p: Precision) -> BigInt {
    // Kernel precondition: |c| < 1 and not near a pole. The constructor rewrites
    // near-pi/2 inputs to cot(complement) before this quotient is used.
    if p >= 1 {
        return Zero::zero();
    }

    divide_scaled_refining(signal, p, |working_precision| {
        (
            sin(signal, c, working_precision),
            cos(signal, c, working_precision),
        )
    })
}

fn tan_rational(signal: &Option<Signal>, r: &Rational, p: Precision) -> BigInt {
    // Same local quotient as `tan`, but both numerator and denominator consume
    // the stored exact rational directly. This keeps exact-rational tangent
    // approximation lazy without rebuilding child Ratio nodes.
    if p >= 1 {
        return Zero::zero();
    }

    divide_scaled_refining(signal, p, |working_precision| {
        (
            sin_rational(signal, r, working_precision),
            cos_rational(signal, r, working_precision),
        )
    })
}

fn tan_large_rational(signal: &Option<Signal>, r: &Rational, p: Precision) -> BigInt {
    // Large tangent is evaluated as a local sin/cos quotient after the same
    // direct half-pi reduction used by sin/cos. This keeps exact large rationals
    // off the generic pi-reduction path and avoids constructing inverse nodes.
    crate::trace_dispatch!("computable_approx", "tan", "large-rational-direct-quotient");
    if p >= 1 {
        return Zero::zero();
    }

    let multiple = large_rational_half_pi_multiple(signal, r);
    if let Some(reduced) = tan_large_rational_quarter_pi(signal, r, &multiple, p) {
        return reduced;
    }
    divide_scaled_refining(signal, p, |working_precision| {
        let (sin_residual, cos_residual) =
            sin_cos_large_rational_residual(signal, r, &multiple, working_precision);
        match large_rational_quadrant(&multiple).to_u8() {
            Some(0) => (sin_residual, cos_residual),
            Some(1) => (cos_residual, -sin_residual),
            Some(2) => (-sin_residual, -cos_residual),
            Some(3) => (-cos_residual, sin_residual),
            _ => unreachable!("quadrant reduction is modulo four"),
        }
    })
}

// Compute cotangent of |c| < 1.
// This mirrors tan(x) = sin(x) / cos(x), but flips the quotient so
// tan(pi/2 - x) can avoid building an extra inverse Computable node.
fn cot(signal: &Option<Signal>, c: &Computable, p: Precision) -> BigInt {
    // Used only after a tangent complement reduction, where sin(c) should be
    // safely away from zero.
    if p >= 1 {
        return Zero::zero();
    }

    divide_scaled_refining(signal, p, |working_precision| {
        (
            cos(signal, c, working_precision),
            sin(signal, c, working_precision),
        )
    })
}

// Compute an approximation of ln(1+x) to precision p.
// This assumes |x| < 1/2.
// It uses ln(1+x) = 2 * atanh(x / (2 + x)),
// whose odd-power series converges substantially faster
// than the direct Taylor series when x is near 1/2.
