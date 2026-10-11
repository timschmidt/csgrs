fn to_prec(n: &BigInt) -> Result<Precision, Problem> {
    n.to_i32().ok_or(Problem::OutOfRange)
}

fn erf_series(signal: &Option<Signal>, op: &Computable, p: Precision) -> Result<BigInt, Problem> {
    let p = crate::computable::checked_approximation_precision(p)?;

    Ok({
        let rough_x = op.approx_signal(signal, -10)?;
        let x_sq_approx = (&rough_x * &rough_x) >> 20;

        let n_estimate = {
            let estimate = &x_sq_approx + BigInt::from(-p) + BigInt::from(10);
            if estimate < BigInt::one() {
                BigInt::one()
            } else {
                estimate
            }
        };
        let magnitude_bits = to_prec(&((&x_sq_approx * BigInt::from(3)) / BigInt::from(2)))?
            .checked_add(2)
            .ok_or(Problem::OutOfRange)?;
        let guard_bits = precision_from_i64(
            i64::try_from(n_estimate.magnitude().bits()).map_err(|_| Problem::OutOfRange)?
                + i64::from(magnitude_bits)
                + 4,
        )?;
        let calc_precision = p.checked_sub(guard_bits).ok_or(Problem::OutOfRange)?;
        let op_prec = calc_precision.checked_sub(8).ok_or(Problem::OutOfRange)?;
        let op_appr = op.approx_signal(signal, op_prec)?;
        let max_trunc_error = signed::ONE.deref() << (p - 4 - calc_precision);

        let mut n: i64 = 0;
        let mut current_term = scale(op_appr.clone(), op_prec - calc_precision);
        let mut current_sum = current_term.clone();
        loop {
            if should_stop(signal) {
                return Err(Problem::Exhausted);
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
    })
}

fn erfc(signal: &Option<Signal>, op: &Computable, p: Precision) -> Result<BigInt, Problem> {
    let p = crate::computable::checked_approximation_precision(p)?;

    Ok({
        Computable::one()
            .add(op.clone().erf().negate())
            .approx_signal(signal, p)?
    })
}

fn normal_sf(signal: &Option<Signal>, op: &Computable, p: Precision) -> Result<BigInt, Problem> {
    let p = crate::computable::checked_approximation_precision(p)?;

    op.clone().negate().pnorm().approx_signal(signal, p)
}

fn normal_interval(
    signal: &Option<Signal>,
    lo: &Computable,
    hi: &Computable,
    p: Precision,
) -> Result<BigInt, Problem> {
    let p = crate::computable::checked_approximation_precision(p)?;

    Ok({
        hi.clone()
            .pnorm()
            .add(lo.clone().pnorm().negate())
            .approx_signal(signal, p)?
    })
}

fn log_pnorm(signal: &Option<Signal>, op: &Computable, p: Precision) -> Result<BigInt, Problem> {
    let p = crate::computable::checked_approximation_precision(p)?;

    op.clone().pnorm().ln().approx_signal(signal, p)
}

fn log_normal_sf(
    signal: &Option<Signal>,
    op: &Computable,
    p: Precision,
) -> Result<BigInt, Problem> {
    let p = crate::computable::checked_approximation_precision(p)?;

    op.clone().normal_sf().ln().approx_signal(signal, p)
}

fn log_dnorm(signal: &Option<Signal>, op: &Computable, p: Precision) -> Result<BigInt, Problem> {
    let p = crate::computable::checked_approximation_precision(p)?;

    Ok({
        let neg_half_x_sq = op.clone().square().shift_left(-1).negate();
        let log_sqrt_2pi = Computable::pi().shift_left(1).ln().shift_left(-1);
        neg_half_x_sq
            .add(log_sqrt_2pi.negate())
            .approx_signal(signal, p)?
    })
}

fn normal_quantile_cdf_sample(
    signal: &Option<Signal>,
    sample: &BigInt,
    grid_precision: Precision,
    function_precision: Precision,
) -> Result<BigInt, Problem> {
    Computable::integer(sample.clone())
        .shift_left(grid_precision)
        .pnorm()
        .approx_signal(signal, function_precision)
}

fn normal_quantile_encloses(
    signal: &Option<Signal>,
    sample: &BigInt,
    grid_precision: Precision,
    function_precision: Precision,
    probability_sample: &BigInt,
) -> Result<bool, Problem> {
    // Each CDF/probability sample has one unit of error. Monotonicity of Phi
    // then certifies the complete one-unit quantile enclosure, independently
    // of the accuracy of the seed or the Newton proposal that chose `sample`.
    let lower =
        normal_quantile_cdf_sample(signal, &(sample - 1_u8), grid_precision, function_precision)?;
    if lower + 2_u8 > *probability_sample {
        return Ok(false);
    }
    let upper =
        normal_quantile_cdf_sample(signal, &(sample + 1_u8), grid_precision, function_precision)?;
    Ok(upper - 2_u8 >= *probability_sample)
}

fn normal_quantile(
    signal: &Option<Signal>,
    p: &Computable,
    seed: &BigInt,
    seed_prec: Precision,
    prec: Precision,
) -> Result<BigInt, Problem> {
    let prec = crate::computable::checked_approximation_precision(prec)?;
    if should_stop(signal) {
        return Err(Problem::Exhausted);
    }
    if let Some(probability) = p.bounded_rational_value() {
        if probability.sign() != Sign::Plus || probability.numerator() >= probability.denominator()
        {
            return Err(Problem::NotANumber);
        }
        if probability.is_one_half() {
            return Ok(BigInt::zero());
        }
    }

    // First prove the open probability domain. This sample also supplies a
    // fixed comparison precision for expanding the initial finite bracket.
    let mut probability_precision: Precision = -8;
    let probability_sample = loop {
        let sample = p.approx_signal(signal, probability_precision)?;
        let one = BigInt::one() << probability_precision.unsigned_abs();
        if sample > BigInt::one() && &sample + 1_u8 < one {
            break sample;
        }
        if sample < -BigInt::one() || sample > one + 1_u8 {
            return Err(Problem::NotANumber);
        }
        probability_precision = probability_precision
            .checked_mul(2)
            .ok_or(Problem::Exhausted)?;
        crate::computable::checked_approximation_precision(probability_precision)?;
    };

    // Coarse requests still need a finite quantile certificate. Solve at unit
    // precision before coarsening, avoiding both seed trust and MAX overflow.
    let target = prec.min(0);
    let mut radius: Precision = 1;
    let guard_bits = loop {
        if should_stop(signal) {
            return Err(Problem::Exhausted);
        }
        let expanded = radius.checked_add(1).ok_or(Problem::Exhausted)?;
        let guard = expanded
            .checked_mul(expanded)
            .and_then(|square| square.checked_add(8))
            .ok_or(Problem::Exhausted)?;
        crate::computable::checked_approximation_precision(
            target.checked_sub(guard).ok_or(Problem::OutOfRange)?,
        )?;
        let lower =
            normal_quantile_cdf_sample(signal, &BigInt::from(-radius), 0, probability_precision)?;
        let upper =
            normal_quantile_cdf_sample(signal, &BigInt::from(radius), 0, probability_precision)?;
        if lower + 2_u8 <= probability_sample && upper - 2_u8 >= probability_sample {
            break guard;
        }
        radius = radius.checked_mul(2).ok_or(Problem::Exhausted)?;
    };

    // On [-radius-1, radius+1], pi<4 and e<4 give
    // phi(x) > 2^(-(radius+1)^2-2). These guard bits make the CDF probes
    // decisive around a sufficiently accurate grid point. All returned values
    // still require explicit CDF enclosures or an already certified bracket.
    let mut grid_precision = target.max(-16);
    let mut lower = BigInt::from(-radius) << grid_precision.unsigned_abs();
    let mut upper = BigInt::from(radius) << grid_precision.unsigned_abs();
    // Metadata is only an optional bounded hint. Extreme precision or width
    // cannot trigger a large shift, a recursive base case, or a numeric answer.
    let mut current = if (-64..=0).contains(&seed_prec) && seed.magnitude().bits() <= 128 {
        scale(seed.clone(), seed_prec - grid_precision)
    } else {
        BigInt::zero()
    };
    loop {
        let mut function_precision = crate::computable::checked_approximation_precision(
            grid_precision
                .checked_sub(guard_bits)
                .ok_or(Problem::OutOfRange)?,
        )?;
        let mut probability_sample = p.approx_signal(signal, function_precision)?;
        let mut use_newton = true;
        let certified = loop {
            if should_stop(signal) {
                return Err(Problem::Exhausted);
            }
            if &upper - &lower <= BigInt::from(2_u8) {
                // Integer endpoints at most two grid units apart fit in the
                // one-unit enclosure about their rounded midpoint.
                break (&lower + &upper) / 2_u8;
            }
            if current <= lower || current >= upper {
                current = (&lower + &upper) / 2_u8;
            }
            let point = Computable::integer(current.clone()).shift_left(grid_precision);
            let cdf = point
                .clone()
                .pnorm()
                .approx_signal(signal, function_precision)?;
            let difference = cdf - &probability_sample;
            if difference.abs() <= BigInt::from(2_u8) {
                if normal_quantile_encloses(
                    signal,
                    &current,
                    grid_precision,
                    function_precision,
                    &probability_sample,
                )? {
                    break current;
                }
                function_precision = crate::computable::checked_approximation_precision(
                    function_precision
                        .checked_sub(16)
                        .ok_or(Problem::OutOfRange)?,
                )?;
                probability_sample = p.approx_signal(signal, function_precision)?;
                continue;
            }
            if difference.sign() == Sign::Minus {
                lower = current.clone();
            } else {
                upper = current.clone();
            }
            let density = point.dnorm().approx_signal(signal, function_precision)?;
            if density.sign() != Sign::Plus {
                return Err(Problem::Exhausted);
            }
            let numerator = difference << grid_precision.unsigned_abs();
            let correction_magnitude =
                (numerator.magnitude() + (density.magnitude() >> 1_usize)) / density.magnitude();
            let correction = BigInt::from_biguint(numerator.sign(), correction_magnitude);
            let proposal = &current - correction;
            let inside = proposal > lower && proposal < upper;
            if inside
                && normal_quantile_encloses(
                    signal,
                    &proposal,
                    grid_precision,
                    function_precision,
                    &probability_sample,
                )?
            {
                break proposal;
            }
            // A midpoint at least every other unsuccessful step guarantees
            // bracket contraction even for arbitrary or adversarial seeds.
            current = if use_newton && inside {
                proposal
            } else {
                (&lower + &upper) / 2_u8
            };
            use_newton = !use_newton;
        };
        if grid_precision == target {
            return Ok(if prec > 0 {
                scale(certified, -prec)
            } else {
                certified
            });
        }
        lower = lower.max(&certified - 1_u8);
        upper = upper.min(&certified + 1_u8);
        let next_precision = (2 * grid_precision + 8).max(target);
        let shift = grid_precision - next_precision;
        lower <<= shift;
        upper <<= shift;
        current = certified << shift;
        grid_precision = next_precision;
    }
}
