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

const NORMAL_QUANTILE_RESULT_MSD_BOUND: Precision = 5;

fn normal_quantile(
    signal: &Option<Signal>,
    p: &Computable,
    seed: &BigInt,
    seed_prec: Precision,
    prec: Precision,
) -> BigInt {
    if prec >= seed_prec || should_stop(signal) {
        return scale(seed.clone(), seed_prec - prec);
    }

    let appr_prec = (NORMAL_QUANTILE_RESULT_MSD_BOUND + prec) / 2 - 6;
    let xn_int = normal_quantile(signal, p, seed, seed_prec, appr_prec);
    let xn = Computable::integer(xn_int).shift_left(appr_prec);
    let fx = xn.clone().pnorm();
    let phi_xn = xn.clone().dnorm();
    let x_next = xn.add(fx.add(p.clone().negate()).multiply(phi_xn.inverse()).negate());

    (x_next.approx_signal(signal, prec - 2) + signed::TWO.deref()) >> 2
}
