fn exp(signal: &Option<Signal>, c: &Computable, p: Precision) -> BigInt {
    if should_stop(signal) {
        return Zero::zero();
    }
    // Construction can defer range reduction, and bounded ln(2) correction
    // can fail. Serialized nodes also reach this boundary directly. Prove the
    // small-series domain before its coarse-zero shortcut or fixed error budget.
    let rough = (!c.exp_argument_is_known_small()).then(|| c.approx_signal(signal, -4));
    if let Some(rough) = rough.filter(|value| value.magnitude() > signed::EIGHT.magnitude()) {
        if should_stop(signal) {
            return Zero::zero();
        }
        if let Some(reduced) = c.reduced_exp() {
            return reduced.approx_signal(signal, p);
        }
        // |16*c - rough| < 1; with b = bits(|rough|), scaling by 2^(b-3)
        // puts |c| below 9/16. Reconstruct through the certified square kernel.
        let steps = i32::try_from(rough.magnitude().bits())
            .expect("exponential input magnitude fits precision")
            - 3;
        let mut reduced = c.clone().shift_left(-steps).exp();
        for _ in 0..steps {
            reduced = reduced.square();
        }
        return reduced.approx_signal(signal, p);
    }
    // Here |c| < 9/16 (or < 1/2 from exact cached facts), including the
    // one-unit rough-approximation error. In particular exp(c) < 2, so zero
    // satisfies the coarse request below.
    if p >= 1 {
        return Zero::zero();
    }

    let iterations_needed = -p / 2 + 2;
    //  Claim: each intermediate term is accurate
    //  to 2*2^calc_precision.
    //  Total rounding error in series computation is
    //  2*iterations_needed*2^calc_precision,
    //  exclusive of error in op.
    let calc_precision = p - bound_log2(2 * iterations_needed) - 4; // for error in op, truncation.
    let op_prec = p - 3;

    let op_appr = c.approx_signal(signal, op_prec);

    // Error in argument results in error of < 3/8 ulp.
    // Sum of term eval. rounding error is < 1/16 ulp.
    // Series truncation error < 1/16 ulp.
    // Final rounding error is <= 1/2 ulp.
    // Thus final error is < 1 ulp.
    let scaled_1 = signed::ONE.deref() << -calc_precision;

    // The loop compares borrowed magnitudes. Calling `abs()` here allocates a
    // fresh BigInt every term and shows up in cold transcendental benches.
    let max_trunc_error = BigUint::one()
        << usize::try_from(p - 4 - calc_precision).expect("truncation shift is nonnegative");
    let mut current_term = scaled_1.clone();
    let mut sum = scaled_1;
    let mut n: i32 = 0;

    while current_term.magnitude() > &max_trunc_error {
        if should_stop(signal) {
            break;
        }
        n += 1;
        current_term = scale(current_term * &op_appr, op_prec) / n;
        sum += &current_term;
    }

    scale(sum, calc_precision - p)
}

fn expm1(signal: &Option<Signal>, c: &Computable, p: Precision) -> BigInt {
    // For x <= 0, -1 < exp(x) - 1 <= 0. Use a retained sign proof without
    // evaluating the operand or walking its expression graph.
    if p >= 1
        && matches!(
            c.immediate_sign(),
            Some(crate::RealSign::Negative | crate::RealSign::Zero)
        )
    {
        return Zero::zero();
    }

    let low_prec = -4;
    let rough = c.approx_signal(signal, low_prec);
    // rough <= 8 proves c < 9/16, including the one-unit error. There is
    // no lower-bound requirement: exp(c) - 1 is always greater than -1.
    if p >= 1 && rough <= *signed::EIGHT {
        return Zero::zero();
    }
    if rough > *signed::EIGHT || rough < -signed::EIGHT.clone() {
        return c
            .clone()
            .exp()
            .add(Computable::one().negate())
            .approx_signal(signal, p);
    }

    let iterations_needed = -p / 2 + 2;
    let calc_precision = p - bound_log2(2 * iterations_needed) - 4;
    let op_prec = p - 3;
    let op_appr = c.approx_signal(signal, op_prec);
    let max_trunc_error = BigUint::one()
        << usize::try_from(p - 4 - calc_precision).expect("truncation shift is nonnegative");
    let mut n: i32 = 1;
    let mut current_term = scale(op_appr.clone(), op_prec - calc_precision);
    let mut sum = current_term.clone();

    while current_term.magnitude() > &max_trunc_error {
        if should_stop(signal) {
            break;
        }
        n += 1;
        current_term = scale(current_term * &op_appr, op_prec) / n;
        sum += &current_term;
    }

    scale(sum, calc_precision - p)
}

fn sqrt(signal: &Option<Signal>, c: &Computable, p: Precision) -> BigInt {
    // Sqrt uses a fixed-size integer sqrt for moderate precision and recursive
    // Newton refinement for deeper requests. This avoids pulling in floating
    // approximations while keeping high-precision sqrt from scaling quadratically.
    // Newton sqrt/reciprocal-sqrt refinement is the standard arbitrary-precision strategy.
    // Larger integer seeds cost more than one extra Newton refinement near
    // the machine-word boundary; the measured crossover is 59 result bits.
    let fp_prec: i32 = 59;
    let fp_op_prec: i32 = 150;

    let max_prec_needed = p.saturating_mul(2).saturating_sub(1);
    let (known_sign, planned_msd) = c.planning_sign_and_msd();
    if known_sign == Some(Sign::NoSign) {
        return Zero::zero();
    }
    let msd = match planned_msd {
        Some(Some(msd)) => msd,
        _ => match c.msd(max_prec_needed) {
            Some(msd) => msd,
            None => {
                let rough = c.approx_signal(signal, max_prec_needed);
                if rough.is_zero() {
                    return Zero::zero();
                }
                rough.magnitude().bits() as Precision - 1 + max_prec_needed
            }
        },
    };

    if msd <= max_prec_needed {
        return Zero::zero();
    }

    if should_stop(signal) {
        return signed::ONE.deref().clone();
    }

    let result_msd = msd / 2;
    let result_digits = result_msd - p;

    if result_digits > fp_prec {
        // Compute less precise approximation and use a Newton iter.
        let appr_digits = result_digits / 2 + 6;
        // This should be conservative.  Is fewer enough?
        let appr_prec = result_msd - appr_digits;

        let last_appr = sqrt(signal, c, appr_prec);
        let prod_prec = 2 * appr_prec;

        let op_appr = c.approx_signal(signal, prod_prec);

        // Slightly fewer might be enough;
        // Compute (last_appr * last_appr + op_appr)/(last_appr/2)
        // while adjusting the scaling to make everything work

        let prod_prec_scaled_numerator = (&last_appr * &last_appr) + op_appr;
        let scaled_numerator = scale(prod_prec_scaled_numerator, appr_prec - p);

        let shifted_result = scaled_numerator / last_appr;

        (shifted_result + signed::ONE.deref()) / signed::TWO.deref()
    } else {
        // If A approximates 2^(2*g)*x within one, then floor(sqrt(A))
        // approximates 2^g*sqrt(x) within two, including near zero. Four
        // guard bits and final rounding therefore give error below 5/8 ulp.
        // Size the integer root to this request. Retain the established seed
        // below if the precision arithmetic cannot represent this request.
        if let Some(op_prec) = p.checked_sub(4).and_then(|prec| prec.checked_mul(2)) {
            let scaled_bi_appr = c.approx_signal(signal, op_prec);
            return scale(scaled_bi_appr.sqrt(), -4);
        }

        // Use an approximation from the Num crate
        // Make sure all precisions are even
        let op_prec = (msd - fp_op_prec) & !1;
        let working_prec = op_prec - fp_op_prec;

        let scaled_bi_appr = c.approx_signal(signal, op_prec) << fp_op_prec;

        let scaled_sqrt = scaled_bi_appr.sqrt();

        let shift_count = working_prec / 2 - p;
        shift(scaled_sqrt, shift_count)
    }
}

fn nth_root(signal: &Option<Signal>, c: &Computable, degree: u32, p: Precision) -> BigInt {
    // The only construction and deserialization paths validate this range.
    // Retain a release-mode guard because approximation kernels are the final
    // trust boundary for an expression graph.
    assert!(
        (3..=Computable::MAX_DIRECT_NTH_ROOT_DEGREE).contains(&degree),
        "invalid direct nth-root degree {degree}"
    );

    // Let t = root(c, degree) * 2^-p and z = t * 2^GUARD_BITS. Asking the
    // child for precision degree * (p - GUARD_BITS) produces an integer A
    // within one of z^degree. Therefore floor_root(max(A - 1, 0)) and
    // ceil_root(A + 1) enclose z. An integer interval of width two crosses at
    // most two nonnegative perfect powers, so rounding its midpoint after the
    // guard-bit shift stays strictly within one unit of t.
    const GUARD_BITS: Precision = 4;
    let Some(child_precision) = p
        .checked_sub(GUARD_BITS)
        .and_then(|precision| precision.checked_mul(Precision::try_from(degree).ok()?))
    else {
        // The public bounded-degree constructor makes this reachable only for
        // an extreme precision request. Preserve total behavior through the
        // established exp/ln kernel rather than wrapping precision arithmetic.
        return c
            .clone()
            .ln()
            .multiply(Computable::rational(
                Rational::from_bigint_fraction(BigInt::one(), BigUint::from(degree)).unwrap(),
            ))
            .exp()
            .approx_signal(signal, p);
    };

    let approximation = c.approx_signal(signal, child_precision);
    if approximation <= BigInt::one() {
        return BigInt::zero();
    }

    let lower_power = approximation.magnitude() - BigUint::one();
    let upper_power = approximation.magnitude() + BigUint::one();
    let lower = lower_power.nth_root(degree);
    let mut upper = upper_power.nth_root(degree);
    if upper.pow(degree) < upper_power {
        upper += BigUint::one();
    }

    let rounding = BigUint::one() << usize::try_from(GUARD_BITS).unwrap();
    BigInt::from((lower + upper + rounding) >> usize::try_from(GUARD_BITS + 1).unwrap())
}

// Compute cosine of |c| < 1
// uses a Taylor series expansion.
