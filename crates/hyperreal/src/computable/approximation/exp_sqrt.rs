fn exp(signal: &Option<Signal>, c: &Computable, p: Precision) -> BigInt {
    // Kernel precondition: caller has reduced |c| below roughly 1/2. The series
    // is intentionally simple here; range reduction belongs in `Computable::exp`.
    // That split mirrors standard multiple-precision exp algorithms: reduce
    // first, evaluate the Taylor series on the reduced input, and reconstruct.
    // See Brent, https://doi.org/10.1145/321941.321944.
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
    if p >= 1 {
        return Zero::zero();
    }

    let low_prec = -4;
    let rough = c.approx_signal(signal, low_prec);
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
    // Newton sqrt/reciprocal-sqrt refinement is the standard arbitrary-precision
    // strategy described in Brent/Zimmermann, Secs. 1.5 and 4.2:
    // https://maths-people.anu.edu.au/~brent/pd/mca-cup-0.5.9.pdf.
    let fp_prec: i32 = 140;
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

// Compute cosine of |c| < 1
// uses a Taylor series expansion.
