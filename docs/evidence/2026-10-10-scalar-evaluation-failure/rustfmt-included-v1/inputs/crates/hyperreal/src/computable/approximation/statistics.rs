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

    Ok({ op.clone().negate().pnorm().approx_signal(signal, p)? })
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

    Ok({ op.clone().pnorm().ln().approx_signal(signal, p)? })
}

fn log_normal_sf(
    signal: &Option<Signal>,
    op: &Computable,
    p: Precision,
) -> Result<BigInt, Problem> {
    let p = crate::computable::checked_approximation_precision(p)?;

    Ok({ op.clone().normal_sf().ln().approx_signal(signal, p)? })
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

const NORMAL_QUANTILE_RESULT_MSD_BOUND: Precision = 5;

fn normal_quantile(
    signal: &Option<Signal>,
    p: &Computable,
    seed: &BigInt,
    seed_prec: Precision,
    prec: Precision,
) -> Result<BigInt, Problem> {
    let prec = crate::computable::checked_approximation_precision(prec)?;

    Ok({
        if should_stop(signal) {
            return Err(Problem::Exhausted);
        }
        if prec >= seed_prec {
            return Ok(scale(seed.clone(), seed_prec - prec));
        }

        let appr_prec = precision_from_i64(
            (i64::from(NORMAL_QUANTILE_RESULT_MSD_BOUND) + i64::from(prec)) / 2 - 6,
        )?;
        let xn_int = normal_quantile(signal, p, seed, seed_prec, appr_prec)?;
        let xn = Computable::integer(xn_int).shift_left(appr_prec);
        let fx = xn.clone().pnorm();
        let phi_xn = xn.clone().dnorm();
        let x_next = xn.add(
            fx.add(p.clone().negate())
                .multiply(phi_xn.inverse())
                .negate(),
        );

        (x_next.approx_signal(signal, prec - 2)? + signed::TWO.deref()) >> 2
    })
}
