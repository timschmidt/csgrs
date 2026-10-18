fn to_prec(n: &BigInt) -> Precision {
    n.to_i32().unwrap_or(Precision::MAX)
}

fn erf_series(signal: &Option<Signal>, op: &Computable, p: Precision) -> BigInt {
    let rough_x = op.approx_signal(signal, -10);
    let x_sq_approx = (&rough_x * &rough_x) >> 20;

    let n_estimate = {
        let estimate = &x_sq_approx + BigInt::from(-p) + BigInt::from(10);
        if estimate < BigInt::one() {
            BigInt::one()
        } else {
            estimate
        }
    };
    let magnitude_bits = to_prec(&((&x_sq_approx * BigInt::from(3)) / BigInt::from(2))) + 2;
    let guard_bits = (n_estimate.magnitude().bits() as Precision) + magnitude_bits + 4;
    let calc_precision = p - guard_bits;
    let op_prec = calc_precision - 8;
    let op_appr = op.approx_signal(signal, op_prec);
    let max_trunc_error = signed::ONE.deref() << (p - 4 - calc_precision);

    let mut n: i64 = 0;
    let mut current_term = scale(op_appr.clone(), op_prec - calc_precision);
    let mut current_sum = current_term.clone();
    loop {
        if should_stop(signal) {
            break;
        }
        let prev = current_term;
        let mut t = scale(&prev * &op_appr, op_prec);
        t = scale(t * &op_appr, op_prec);
        t = (t * signed::TWO.deref()) / BigInt::from(2 * n + 3);
        n += 1;
        if t.is_zero() {
            break;
        }

        let prev_abs = prev.abs();
        let cur_abs = t.abs();
        if cur_abs < prev_abs {
            let denom = &prev_abs - &cur_abs;
            if &cur_abs * &prev_abs < &max_trunc_error * &denom {
                break;
            }
        }
        current_sum += &t;
        current_term = t;
    }
    scale(current_sum, calc_precision - p)
}

fn erfc(signal: &Option<Signal>, op: &Computable, p: Precision) -> BigInt {
    Computable::one()
        .add(op.clone().erf().negate())
        .approx_signal(signal, p)
}

fn normal_sf(signal: &Option<Signal>, op: &Computable, p: Precision) -> BigInt {
    op.clone().negate().pnorm().approx_signal(signal, p)
}

fn normal_interval(
    signal: &Option<Signal>,
    lo: &Computable,
    hi: &Computable,
    p: Precision,
) -> BigInt {
    hi.clone()
        .pnorm()
        .add(lo.clone().pnorm().negate())
        .approx_signal(signal, p)
}

fn log_pnorm(signal: &Option<Signal>, op: &Computable, p: Precision) -> BigInt {
    op.clone().pnorm().ln().approx_signal(signal, p)
}

fn log_normal_sf(signal: &Option<Signal>, op: &Computable, p: Precision) -> BigInt {
    op.clone().normal_sf().ln().approx_signal(signal, p)
}

fn log_dnorm(signal: &Option<Signal>, op: &Computable, p: Precision) -> BigInt {
    let neg_half_x_sq = op.clone().square().shift_left(-1).negate();
    let log_sqrt_2pi = Computable::pi().shift_left(1).ln().shift_left(-1);
    neg_half_x_sq
        .add(log_sqrt_2pi.negate())
        .approx_signal(signal, p)
}

// Compare a dyadic trial point with the quantile, or certify an interval
// around it. The seed and Newton step choose trial points; only CDF bounds
// justify returning an approximation. Each approximation has error <= 1 ulp,
// so a difference greater than two ulps proves the corresponding strict order.
fn normal_quantile_probe(
    signal: &Option<Signal>,
    p: &Computable,
    trial: &BigInt,
    working_precision: Precision,
) -> Option<std::cmp::Ordering> {
    use std::cmp::Ordering;

    let lower = Computable::integer(trial - 2)
        .shift_left(working_precision)
        .pnorm();
    let upper = Computable::integer(trial + 2)
        .shift_left(working_precision)
        .pnorm();
    let center = Computable::integer(trial.clone())
        .shift_left(working_precision)
        .pnorm();
    // This is a planning hint only. Small tail densities need extra CDF bits;
    // all decisions below still require disjoint numerical enclosures.
    let magnitude = scale(trial.abs(), working_precision)
        .to_u32()
        .unwrap_or(64)
        .min(64);
    let guard = ((magnitude + 2).pow(2) + 8) as Precision;
    let mut precision = working_precision.saturating_sub(guard).min(-16);
    loop {
        if should_stop(signal) {
            return None;
        }
        let probability = p.approx_signal(signal, precision);
        let lo = lower.approx_signal(signal, precision);
        let hi = upper.approx_signal(signal, precision);
        if should_stop(signal) {
            return None;
        }
        if &probability - &lo > BigInt::from(2) && &hi - &probability > BigInt::from(2) {
            return Some(Ordering::Equal);
        }
        if lo - &probability > BigInt::from(2) {
            return Some(Ordering::Greater);
        }
        if &probability - hi > BigInt::from(2) {
            return Some(Ordering::Less);
        }
        // If a bracket endpoint equals the root, its strict comparison cannot
        // terminate. The center is then separated from the root instead.
        let middle = center.approx_signal(signal, precision);
        if should_stop(signal) {
            return None;
        }
        if &middle - &probability > BigInt::from(2) {
            return Some(Ordering::Greater);
        }
        if probability - middle > BigInt::from(2) {
            return Some(Ordering::Less);
        }
        precision = precision.saturating_sub(16);
    }
}

fn normal_quantile_bisect(
    signal: &Option<Signal>,
    p: &Computable,
    mut lower: BigInt,
    mut upper: BigInt,
    working_precision: Precision,
) -> BigInt {
    use std::cmp::Ordering;

    while &upper - &lower > BigInt::from(4) {
        let trial = (&lower + &upper) >> 1;
        match normal_quantile_probe(signal, p, &trial, working_precision) {
            Some(Ordering::Less) => lower = trial,
            Some(Ordering::Greater) => upper = trial,
            Some(Ordering::Equal) => return scale(trial, -2),
            None => return BigInt::zero(),
        }
    }
    // The root lies in [lower, upper], whose width is at most one result ulp.
    // Round its exact midpoint: half the width plus half an ulp is <= one ulp.
    scale(lower + upper, -3)
}

fn normal_quantile(
    signal: &Option<Signal>,
    p: &Computable,
    seed: &BigInt,
    seed_prec: Precision,
    prec: Precision,
) -> BigInt {
    use std::cmp::Ordering;

    if should_stop(signal) {
        return BigInt::zero();
    }
    if let Some(exact) = p.exact_normal_quantile() {
        return Computable::rational(exact).approx_signal(signal, prec);
    }
    if prec > 0 {
        return scale(normal_quantile(signal, p, seed, seed_prec, 0), -prec);
    }
    let working_precision = prec - 2;
    if prec < -16 {
        let previous_precision = prec / 2 - 6;
        let previous = normal_quantile(signal, p, seed, seed_prec, previous_precision);
        if should_stop(signal) {
            return BigInt::zero();
        }
        let xn = Computable::integer(previous.clone()).shift_left(previous_precision);
        let residual = xn.clone().pnorm().add(p.clone().negate());
        let correction = residual.multiply(xn.clone().dnorm().inverse());
        let trial = xn
            .add(correction.negate())
            .approx_signal(signal, working_precision);
        let shift = previous_precision - working_precision;
        let lower = (&previous - 1) << shift;
        let upper = (&previous + 1) << shift;
        // A previous certified approximation supplies a bracket even if a
        // Newton proposal fails or leaves the interval. No convergence estimate
        // or fixed bound on the quantile's magnitude is used as a certificate.
        if trial >= lower
            && trial <= upper
            && normal_quantile_probe(signal, p, &trial, working_precision) == Some(Ordering::Equal)
        {
            return scale(trial, -2);
        }
        return normal_quantile_bisect(signal, p, lower, upper, working_precision);
    }

    // Avoid materializing a huge untrusted seed. This only selects a starting
    // point: every finite quantile remains reachable by the expanding search.
    let trial = if seed.is_zero()
        || i64::from(seed_prec) + i64::try_from(seed.bits()).unwrap_or(i64::MAX / 2) > 5
    {
        BigInt::zero()
    } else {
        scale(seed.clone(), seed_prec - working_precision)
    };
    if normal_quantile_probe(signal, p, &trial, working_precision) == Some(Ordering::Equal) {
        return scale(trial, -2);
    }

    let unit = BigInt::one() << working_precision.unsigned_abs();
    let mut lower = -&unit;
    let mut upper = unit;
    loop {
        match normal_quantile_probe(signal, p, &lower, working_precision) {
            Some(Ordering::Less) => break,
            Some(Ordering::Greater) => lower <<= 1,
            Some(Ordering::Equal) => return scale(lower, -2),
            None => return BigInt::zero(),
        }
    }
    loop {
        match normal_quantile_probe(signal, p, &upper, working_precision) {
            Some(Ordering::Greater) => break,
            Some(Ordering::Less) => upper <<= 1,
            Some(Ordering::Equal) => return scale(upper, -2),
            None => return BigInt::zero(),
        }
    }
    normal_quantile_bisect(signal, p, lower, upper, working_precision)
}
