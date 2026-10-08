fn ln(signal: &Option<Signal>, c: &Computable, p: Precision) -> BigInt {
    // Kernel precondition: this computes ln(1+x), not arbitrary ln(x). Public
    // construction keeps |x| < 1/2 by inversion, sqrt scaling, and powers of two.
    // The atanh transform is a standard log argument reduction for faster odd
    // power-series convergence.
    if p >= 0 {
        return Zero::zero();
    }

    let iterations_needed = -p / 2 + 4;
    let calc_precision = p - bound_log2(2 * iterations_needed) - 4;
    let op_prec = calc_precision - 3;
    let op_appr = c.approx_signal(signal, op_prec);
    let scaled_x = scale(op_appr, op_prec - calc_precision);
    let scaled_one = signed::ONE.deref() << -calc_precision;
    let denominator = (&scaled_one << 1) + &scaled_x;

    let numerator = &scaled_x << -calc_precision;
    let y: BigInt = if numerator.sign() == Sign::Minus {
        let rounded: BigInt = ((-&numerator) + (&denominator >> 1)) / &denominator;
        -rounded
    } else {
        (&numerator + (&denominator >> 1)) / &denominator
    };

    let y_squared = scale(&y * &y, calc_precision);
    let mut current_power = y.clone();
    let mut current_term = y.clone();
    let mut sum = current_term.clone();
    let mut n = 1;

    // Keep the atanh-transformed ln series from allocating an absolute BigInt
    // on every odd term.
    let max_trunc_error = BigUint::one()
        << usize::try_from(p - 4 - calc_precision).expect("truncation shift is nonnegative");

    while current_term.magnitude() > &max_trunc_error {
        if should_stop(signal) {
            break;
        }
        n += 2;
        current_power = scale(current_power * &y_squared, calc_precision);
        current_term = &current_power / n;
        sum += &current_term;
    }

    scale(sum << 1, calc_precision - p)
}

fn ln_rational(signal: &Option<Signal>, r: &Rational, p: Precision) -> BigInt {
    // Exact-rational ln1p paths already know the small residual. Feed it
    // directly into the same atanh-transformed series instead of wrapping it in
    // a temporary Ratio child and calling back through Computable::approx.
    if p >= 0 {
        return Zero::zero();
    }

    let iterations_needed = -p / 2 + 4;
    let calc_precision = p - bound_log2(2 * iterations_needed) - 4;
    let op_prec = calc_precision - 3;
    let op_appr = ratio(r, op_prec);
    let scaled_x = scale(op_appr, op_prec - calc_precision);
    let scaled_one = signed::ONE.deref() << -calc_precision;
    let denominator = (&scaled_one << 1) + &scaled_x;

    let numerator = &scaled_x << -calc_precision;
    let y: BigInt = if numerator.sign() == Sign::Minus {
        let rounded: BigInt = ((-&numerator) + (&denominator >> 1)) / &denominator;
        -rounded
    } else {
        (&numerator + (&denominator >> 1)) / &denominator
    };

    let y_squared = scale(&y * &y, calc_precision);
    let mut current_power = y.clone();
    let mut current_term = y.clone();
    let mut sum = current_term.clone();
    let mut n = 1;

    let max_trunc_error = BigUint::one()
        << usize::try_from(p - 4 - calc_precision).expect("truncation shift is nonnegative");

    while current_term.magnitude() > &max_trunc_error {
        if should_stop(signal) {
            break;
        }
        n += 2;
        current_power = scale(current_power * &y_squared, calc_precision);
        current_term = &current_power / n;
        sum += &current_term;
    }

    scale(sum << 1, calc_precision - p)
}

fn binary_scaled_ln_rational(
    signal: &Option<Signal>,
    residual: &Rational,
    shift: i32,
    p: Precision,
) -> BigInt {
    // Exact-rational range reduction often produces ln(2^k * (1+x)) for small
    // rational x. Keep that known shape inside one approximation node instead
    // of constructing a transient add/multiply graph around the shared ln2
    // constant for every adversarial ln(1+x^2) row.
    crate::trace_dispatch!("computable_approx", "ln", "binary-scaled-rational");
    if shift == 0 {
        return ln_rational(signal, residual, p);
    }

    let sum_precision = p - 2;
    let residual_appr = ln_rational(signal, residual, sum_precision);
    let shift_abs = shift.unsigned_abs();
    let shift_msd = i32::try_from(u32::BITS - 1 - shift_abs.leading_zeros())
        .expect("shift magnitude bits fit in i32");
    let ln2_precision = sum_precision - shift_msd - 3;
    let ln2_appr = Computable::ln2().approx_signal(signal, ln2_precision);
    let ln2_term = scale(
        BigInt::from(shift) * ln2_appr,
        ln2_precision - sum_precision,
    );

    scale(residual_appr + ln2_term, -2)
}

// Approximate the Arctangent of 1/n where n is some small integer > base
// what is "base" in this context?
