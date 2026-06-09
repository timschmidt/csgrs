fn acosh_near_one(signal: &Option<Signal>, c: &Computable, p: Precision) -> BigInt {
    // Near one, acosh(x) is ln1p((x - 1) + sqrt(x^2 - 1)). Deferring the graph
    // keeps construction cheap for endpoint-adjacent scalar rows without
    // changing the cancellation-avoiding approximation identity.
    let one = Computable::one();
    let shifted = c.clone().add(one.clone().negate());
    let radicand = c.clone().square().add(one.negate());
    shifted
        .add(radicand.sqrt())
        .ln_1p()
        .approx_signal(signal, p)
}

fn acosh_direct(signal: &Option<Signal>, c: &Computable, p: Precision) -> BigInt {
    // Large acosh inputs use ln(x + sqrt(x^2 - 1)); this node is used by Real
    // construction paths where allocating that graph eagerly is the bottleneck.
    let one = Computable::one();
    let radicand = c.clone().square().add(one.negate());
    c.clone().add(radicand.sqrt()).ln().approx_signal(signal, p)
}

fn asinh_near_zero(signal: &Option<Signal>, c: &Computable, p: Precision) -> BigInt {
    // Near zero, asinh(x) is evaluated through ln1p(x + x^2/(sqrt(1+x^2)+1)).
    // This deferred node removes construction overhead but preserves the
    // cancellation-resistant formula used by the public Real path.
    let square = c.clone().square();
    let one = Computable::one();
    let denominator = square.clone().add(one.clone()).sqrt().add(one);
    c.clone()
        .add(square.multiply(denominator.inverse()))
        .ln_1p()
        .approx_signal(signal, p)
}

fn asinh_direct(signal: &Option<Signal>, c: &Computable, p: Precision) -> BigInt {
    // Large asinh inputs use ln(x + sqrt(1+x^2)); deferring the direct identity
    // keeps scalar construction from allocating the sqrt/log graph eagerly.
    let radicand = c.clone().square().add(Computable::one());
    c.clone().add(radicand.sqrt()).ln().approx_signal(signal, p)
}

// Approximate asinh(c) for small |c|.
fn asinh_computable(signal: &Option<Signal>, c: &Computable, p: Precision) -> BigInt {
    // Dedicated tiny-argument asinh series. It avoids constructing the generic
    // ln(x + sqrt(1+x^2)) or ln1p expression for exact tiny rational inputs.
    if p >= 1 {
        return Zero::zero();
    }

    let iterations_needed: i32 = -p / 2 + 4;
    let calc_precision = p - bound_log2(2 * iterations_needed) - 5;
    let op_prec = calc_precision - 3;
    let op_appr = c.approx_signal(signal, op_prec);
    let op_squared = scale(&op_appr * &op_appr, op_prec);

    // This is the asin recurrence with alternating sign:
    // asinh(x) = x - x^3/6 + 3x^5/40 - ...
    let max_trunc_error = BigUint::one()
        << usize::try_from(p - 4 - calc_precision).expect("truncation shift is nonnegative");
    let mut current_term = scale(op_appr, op_prec - calc_precision);
    let mut sum = current_term.clone();
    let mut n = 0_i32;

    while current_term.magnitude() > &max_trunc_error {
        if should_stop(signal) {
            break;
        }
        n += 1;
        current_term = scale(current_term * &op_squared, op_prec);
        let numerator = -((2 * n - 1) * (2 * n - 1));
        let denominator = (2 * n) * (2 * n + 1);
        current_term *= numerator;
        current_term /= denominator;
        sum += &current_term;
    }

    scale(sum, calc_precision - p)
}

fn asinh_rational(signal: &Option<Signal>, r: &Rational, p: Precision) -> BigInt {
    // Direct exact-rational variant of the tiny asinh series. It is the same
    // recurrence as `asinh_computable`, but it feeds the stored Rational
    // straight to the final precision request instead of allocating a temporary
    // Ratio node and child cache.
    if p >= 1 {
        return Zero::zero();
    }

    let iterations_needed: i32 = -p / 2 + 4;
    let calc_precision = p - bound_log2(2 * iterations_needed) - 5;
    let op_prec = calc_precision - 3;
    let op_appr = ratio(r, op_prec);
    let op_squared = scale(&op_appr * &op_appr, op_prec);

    let max_trunc_error = BigUint::one()
        << usize::try_from(p - 4 - calc_precision).expect("truncation shift is nonnegative");
    let mut current_term = scale(op_appr, op_prec - calc_precision);
    let mut sum = current_term.clone();
    let mut n = 0_i32;

    while current_term.magnitude() > &max_trunc_error {
        if should_stop(signal) {
            break;
        }
        n += 1;
        current_term = scale(current_term * &op_squared, op_prec);
        let numerator = -((2 * n - 1) * (2 * n - 1));
        let denominator = (2 * n) * (2 * n + 1);
        current_term *= numerator;
        current_term /= denominator;
        sum += &current_term;
    }

    scale(sum, calc_precision - p)
}

fn atanh_direct(signal: &Option<Signal>, c: &Computable, p: Precision) -> BigInt {
    // Endpoint atanh construction should not eagerly allocate the full exact
    // log-ratio graph. Approximation still uses the stable identity
    // atanh(x) = 1/2 * ln((1+x)/(1-x)).
    let one = Computable::one();
    let numerator = one.clone().add(c.clone());
    let denominator = one.add(c.clone().negate());
    numerator
        .multiply(denominator.inverse())
        .ln()
        .multiply(Computable::rational(HALF_RATIONAL.clone()))
        .approx_signal(signal, p)
}

// Approximate atanh(c) for small |c|.
fn atanh_computable(signal: &Option<Signal>, c: &Computable, p: Precision) -> BigInt {
    // Dedicated tiny-argument atanh series, also reused by the ln1p kernel after
    // it transforms ln(1+x) into 2*atanh(x/(2+x)).
    if p >= 1 {
        return Zero::zero();
    }

    let iterations_needed: i32 = -p / 2 + 4;
    let calc_precision = p - bound_log2(2 * iterations_needed) - 5;
    let op_prec = calc_precision - 3;
    let op_appr = c.approx_signal(signal, op_prec);
    let op_squared = scale(&op_appr * &op_appr, op_prec);

    // Borrowed magnitude checks matter here because tiny inverse-hyperbolic
    // benches run many short series from cold caches.
    let max_trunc_error = BigUint::one()
        << usize::try_from(p - 4 - calc_precision).expect("truncation shift is nonnegative");
    let mut current_power = scale(op_appr, op_prec - calc_precision);
    let mut current_term = current_power.clone();
    let mut sum = current_term.clone();
    let mut n = 1_i32;

    while current_term.magnitude() > &max_trunc_error {
        if should_stop(signal) {
            break;
        }
        n += 2;
        current_power = scale(current_power * &op_squared, op_prec);
        current_term = &current_power / n;
        sum += &current_term;
    }

    scale(sum, calc_precision - p)
}

fn atanh_rational(signal: &Option<Signal>, r: &Rational, p: Precision) -> BigInt {
    // Direct exact-rational variant of the tiny atanh series. This mirrors the
    // direct rational trig kernels: preserve the symbolic rational payload and
    // round only once at the requested working precision.
    if p >= 1 {
        return Zero::zero();
    }

    let iterations_needed: i32 = -p / 2 + 4;
    let calc_precision = p - bound_log2(2 * iterations_needed) - 5;
    let op_prec = calc_precision - 3;
    let op_appr = ratio(r, op_prec);
    let op_squared = scale(&op_appr * &op_appr, op_prec);

    let max_trunc_error = BigUint::one()
        << usize::try_from(p - 4 - calc_precision).expect("truncation shift is nonnegative");
    let mut current_power = scale(op_appr, op_prec - calc_precision);
    let mut current_term = current_power.clone();
    let mut sum = current_term.clone();
    let mut n = 1_i32;

    while current_term.magnitude() > &max_trunc_error {
        if should_stop(signal) {
            break;
        }
        n += 2;
        current_power = scale(current_power * &op_squared, op_prec);
        current_term = &current_power / n;
        sum += &current_term;
    }

    scale(sum, calc_precision - p)
}
