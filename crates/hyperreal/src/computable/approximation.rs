use crate::computable::{scale, shift, signed, Precision};
use crate::Computable;
use crate::Rational;
use num::bigint::Sign;
use num::{BigInt, BigUint, Signed};
use num::{One, Zero};
use std::ops::Deref;

#[derive(Clone, Debug)]
pub(super) enum Approximation {
    Int(BigInt),
    Inverse(Computable),
    Negate(Computable),
    Add(Computable, Computable),
    Multiply(Computable, Computable),
    Square(Computable),
    Ratio(Rational),
    Offset(Computable, i32),
    Exp(Computable),
    Sqrt(Computable),
    PrescaledLn(Computable),
    IntegralAtan(BigInt),
}

impl Approximation {
    pub fn approximate(&self, p: Precision) -> BigInt {
        use Approximation::*;

        match self {
            Int(i) => scale(i.clone(), -p),
            Inverse(c) => inverse(c, p),
            Negate(c) => -c.approx(p),
            Add(c1, c2) => add(c1, c2, p),
            Multiply(c1, c2) => multiply(c1, c2, p),
            Square(c) => square(c, p),
            Ratio(r) => ratio(r, p),
            Offset(c, n) => offset(c, *n, p),
            Exp(c) => exp(c, p),
            Sqrt(c) => sqrt(c, p),
            PrescaledLn(c) => ln(c, p),
            IntegralAtan(i) => atan(i, p),
        }
    }
}

fn inverse(c: &Computable, p: Precision) -> BigInt {
    let msd = c.iter_msd();
    let inv_msd = 1 - msd;
    let digits_needed = inv_msd - p + 3;
    let prec_needed = msd - digits_needed;
    let log_scale_factor = -p - prec_needed;

    if log_scale_factor < 0 {
        return Zero::zero();
    }

    let dividend = signed::ONE.deref() << log_scale_factor;
    let scaled_divisor = c.approx(prec_needed);
    let abs_scaled_divisor = scaled_divisor.abs();
    let adj_dividend = dividend + (&abs_scaled_divisor >> 1);
    let result: BigInt = adj_dividend / abs_scaled_divisor;

    if scaled_divisor.sign() == Sign::Minus {
        -result
    } else {
        result
    }
}

fn add(c1: &Computable, c2: &Computable, p: Precision) -> BigInt {
    scale(c1.approx(p - 2) + c2.approx(p - 2), -2)
}

fn multiply(c1: &Computable, c2: &Computable, p: Precision) -> BigInt {
    let half_prec = (p >> 1) - 1;

    match c1.msd(half_prec) {
        None => match c2.msd(half_prec) {
            None => Zero::zero(),
            Some(msd_op2) => {
                let prec1 = p - msd_op2 - 3;
                let appr1 = c1.approx(prec1);

                if appr1.sign() == Sign::NoSign {
                    return Zero::zero();
                }

                let msd_op1 = c1.known_msd();
                let prec2 = p - msd_op1 - 3;
                let appr2 = c2.approx(prec2);

                let scale_digits = prec2 + prec1 - p;
                scale(appr2 * appr1, scale_digits)
            }
        },
        Some(msd_op1) => {
            let prec2 = p - msd_op1 - 3;
            let appr2 = c2.approx(prec2);

            if appr2.sign() == Sign::NoSign {
                return Zero::zero();
            }

            let msd_op2 = c2.known_msd();
            let prec1 = p - msd_op2 - 3;
            let appr1 = c1.approx(prec1);

            let scale_digits = prec1 + prec2 - p;
            scale(appr1 * appr2, scale_digits)
        }
    }
}

fn square(c: &Computable, p: Precision) -> BigInt {
    let half_prec = (p >> 1) - 1;
    let prec2 = match c.msd(half_prec) {
        None => {
            return Zero::zero();
        }
        Some(msd) => p - msd - 3,
    };

    let appr2 = c.approx(prec2);

    if appr2.sign() == Sign::NoSign {
        return Zero::zero();
    }

    let msd_op2 = c.known_msd();
    let prec1 = p - msd_op2 - 3;
    let appr1 = c.approx(prec1);

    let scale_digits = prec1 + prec2 - p;
    scale(appr1 * appr2, scale_digits)
}

fn ratio(r: &Rational, p: Precision) -> BigInt {
    if p >= 0 {
        scale(r.shifted_big_integer(0), -p)
    } else {
        r.shifted_big_integer(-p)
    }
}

fn offset(c: &Computable, n: i32, p: Precision) -> BigInt {
    c.approx(p - n)
}

fn bound_log2(n: i32) -> i32 {
    let abs_n = n.abs();
    let ln2 = 2.0_f64.ln();
    let n_plus_1: f64 = (abs_n + 1).into();
    let ans: f64 = (n_plus_1.ln() / ln2).ceil();
    ans as i32
}

/* Only intended for Computable values < 0.5, others will be pre-scaled
 * in Computable::exp */
fn exp(c: &Computable, p: Precision) -> BigInt {
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

    let op_appr = c.approx(op_prec);

    // Error in argument results in error of < 3/8 ulp.
    // Sum of term eval. rounding error is < 1/16 ulp.
    // Series truncation error < 1/16 ulp.
    // Final rounding error is <= 1/2 ulp.
    // Thus final error is < 1 ulp.
    let scaled_1 = signed::ONE.deref() << -calc_precision;

    let max_trunc_error = signed::ONE.deref() << (p - 4 - calc_precision);
    let mut current_term = scaled_1.clone();
    let mut sum = scaled_1;
    let mut n = BigInt::zero();

    while current_term.abs() > max_trunc_error {
        n += signed::ONE.deref();
        current_term = scale(current_term * &op_appr, op_prec) / &n;
        sum += &current_term;
    }

    scale(sum, calc_precision - p)
}

fn sqrt(c: &Computable, p: Precision) -> BigInt {
    let fp_prec: i32 = 50;
    let fp_op_prec: i32 = 60;

    let max_prec_needed = 2 * p - 1;
    let msd = c.msd(max_prec_needed).unwrap_or(Precision::MIN);

    if msd <= max_prec_needed {
        return Zero::zero();
    }

    let result_msd = msd / 2;
    let result_digits = result_msd - p;

    if result_digits > fp_prec {
        // Compute less precise approximation and use a Newton iter.
        let appr_digits = result_digits / 2 + 6;
        // This should be conservative.  Is fewer enough?
        let appr_prec = result_msd - appr_digits;

        let last_appr = sqrt(c, appr_prec);
        let prod_prec = 2 * appr_prec;

        let op_appr = c.approx(prod_prec);

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

        let scaled_bi_appr = c.approx(op_prec) << fp_op_prec;

        let scaled_sqrt = scaled_bi_appr.sqrt();

        let shift_count = working_prec / 2 - p;
        shift(scaled_sqrt, shift_count)
    }
}

// Compute an approximation of ln(1+x) to precision p.
// This assumes |x| < 1/2.
// It uses a Taylor series expansion.
// Unfortunately there appears to be no way to take
// advantage of old information.
// Note: this is known to be a bad algorithm for
// floating point.  Unfortunately, other alternatives
// appear to require precomputed tabular information.
fn ln(c: &Computable, p: Precision) -> BigInt {
    if p >= 0 {
        return Zero::zero();
    }

    let iterations_needed = -p;
    let calc_precision = p - bound_log2(2 * iterations_needed) - 4;
    let op_prec = p - 3;
    let op_appr = c.approx(op_prec);

    let mut x_nth = scale(op_appr.clone(), op_prec - calc_precision);
    let mut current_term = x_nth.clone();
    let mut sum = current_term.clone();

    let mut n = 1;
    let mut sign = 1;

    let max_trunc_error = signed::ONE.deref() << (p - 4 - calc_precision);

    while current_term.abs() > max_trunc_error {
        n += 1;
        sign = -sign;
        x_nth = scale(&x_nth * &op_appr, op_prec);

        let divisor: BigInt = (n * sign).into();
        current_term = &x_nth / divisor;
        sum += &current_term;
    }

    scale(sum, calc_precision - p)
}

// Approximate the Arctangent of 1/n where n is some small integer > base
// what is "base" in this context?
fn atan(i: &BigInt, p: Precision) -> BigInt {
    if p >= 1 {
        return Zero::zero();
    }

    let iterations_needed: i32 = -p / 2 + 2; // conservative estimate > 0.
                                             // from Java implementation description:

    // Claim: each intermediate term is accurate
    // to 2*base^calc_precision.
    // Total rounding error in series computation is
    // 2*iterations_needed*base^calc_precision,
    // exclusive of error in op.

    let calc_precision = p - bound_log2(2 * iterations_needed) - 2;
    // Error in argument results in error of < 3/8 ulp.
    // Cumulative arithmetic rounding error is < 1/4 ulp.
    // Series truncation error < 1/4 ulp.
    // Final rounding error is <= 1/2 ulp.
    // Thus final error is < 1 ulp.

    let max_trunc_error: BigUint = BigUint::one() << (p - 2 - calc_precision);

    let scaled_1 = signed::ONE.deref() << (-calc_precision);
    let big_op_squared: BigInt = i * i;
    let inverse: BigInt = scaled_1 / i;

    let mut current_power = inverse.clone();
    let mut current_term = inverse.clone();
    let mut sum = inverse;

    let mut sign = 1;
    let mut n = 1;

    // TODO good place to halt computation
    while *current_term.magnitude() > max_trunc_error {
        n += 2;
        current_power /= &big_op_squared;
        sign = -sign;
        let signed_n: BigInt = (n * sign).into();
        current_term = &current_power / signed_n;
        sum += &current_term;
    }

    scale(sum, calc_precision - p)
}
