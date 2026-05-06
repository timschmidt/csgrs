use crate::Computable;
use crate::Rational;
use crate::computable::{Precision, Signal, scale, shift, should_stop, signed};
use num::bigint::Sign;
use num::{BigInt, BigUint, Signed};
use num::{One, Zero};
use serde::Deserialize;
use serde::Serialize;
use std::ops::Deref;

#[derive(Clone, Debug, Serialize, Deserialize)]
pub(super) enum Approximation {
    Int(BigInt),
    Constant(SharedConstant),
    Inverse(Computable),
    Negate(Computable),
    Add(Computable, Computable),
    Multiply(Computable, Computable),
    Square(Computable),
    Ratio(Rational),
    Offset(Computable, i32),
    PrescaledExp(Computable),
    Sqrt(Computable),
    PrescaledLn(Computable),
    IntegralAtan(BigInt),
    PrescaledAtan(Computable),
    PrescaledAsin(Computable),
    PrescaledAtanh(Computable),
    PrescaledCos(Computable),
    PrescaledSin(Computable),
    PrescaledTan(Computable),
    PrescaledCot(Computable),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub(super) enum SharedConstant {
    E,
    Pi,
    Tau,
    Ln2,
    Ln3,
    Ln5,
    Ln6,
    Ln7,
    Ln10,
    Sqrt2,
    Sqrt3,
}

impl SharedConstant {
    pub(super) const COUNT: usize = 11;

    pub(super) fn cache_index(self) -> usize {
        match self {
            Self::E => 0,
            Self::Pi => 1,
            Self::Tau => 2,
            Self::Ln2 => 3,
            Self::Ln3 => 4,
            Self::Ln5 => 5,
            Self::Ln6 => 6,
            Self::Ln7 => 7,
            Self::Ln10 => 8,
            Self::Sqrt2 => 9,
            Self::Sqrt3 => 10,
        }
    }
}

impl Approximation {
    pub fn approximate(&self, signal: &Option<Signal>, p: Precision) -> BigInt {
        use Approximation::*;

        match self {
            Int(i) => scale(i.clone(), -p),
            Constant(c) => c.approximate(signal, p),
            Inverse(c) => inverse(signal, c, p),
            Negate(c) => -c.approx_signal(signal, p),
            Add(c1, c2) => add(signal, c1, c2, p),
            Multiply(c1, c2) => multiply(signal, c1, c2, p),
            Square(c) => square(signal, c, p),
            Ratio(r) => ratio(r, p),
            Offset(c, n) => offset(signal, c, *n, p),
            PrescaledExp(c) => exp(signal, c, p),
            Sqrt(c) => sqrt(signal, c, p),
            PrescaledLn(c) => ln(signal, c, p),
            IntegralAtan(i) => atan(signal, i, p),
            PrescaledAtan(c) => atan_computable(signal, c, p),
            PrescaledAsin(c) => asin_computable(signal, c, p),
            PrescaledAtanh(c) => atanh_computable(signal, c, p),
            PrescaledCos(c) => cos(signal, c, p),
            PrescaledSin(c) => sin(signal, c, p),
            PrescaledTan(c) => tan(signal, c, p),
            PrescaledCot(c) => cot(signal, c, p),
        }
    }
}

impl SharedConstant {
    fn approximate(self, signal: &Option<Signal>, p: Precision) -> BigInt {
        match self {
            Self::E => e(p),
            Self::Pi => pi(signal, p),
            Self::Tau => pi(signal, p - 1),
            Self::Ln2 => ln2(signal, p),
            Self::Ln3 => ln_constant(signal, Rational::new(3), p),
            Self::Ln5 => ln_constant(signal, Rational::new(5), p),
            Self::Ln6 => ln_constant(signal, Rational::new(6), p),
            Self::Ln7 => ln_constant(signal, Rational::new(7), p),
            Self::Ln10 => ln_constant(signal, Rational::new(10), p),
            Self::Sqrt2 => sqrt_constant(signal, Rational::new(2), p),
            Self::Sqrt3 => sqrt_constant(signal, Rational::new(3), p),
        }
    }
}

fn raw(kind: Approximation) -> Computable {
    Computable {
        internal: Box::new(kind),
        cache: std::cell::RefCell::new(crate::computable::Cache::Invalid),
        bound: std::cell::RefCell::new(crate::computable::BoundCache::Invalid),
        exact_sign: std::cell::RefCell::new(crate::computable::ExactSignCache::Invalid),
        signal: None,
    }
}

fn pi(signal: &Option<Signal>, p: Precision) -> BigInt {
    let atan5 = Computable::prescaled_atan(BigInt::from(5_u8));
    let atan_239 = Computable::prescaled_atan(BigInt::from(239_u16));
    let four = Computable::integer(BigInt::from(4_u8));
    let four_atan5 = four.clone().multiply(atan5);
    let sum = four_atan5.add(atan_239.negate());
    four.multiply(sum).approx_signal(signal, p)
}

fn ln2(signal: &Option<Signal>, p: Precision) -> BigInt {
    let prescaled_9 = raw(Approximation::PrescaledLn(Computable::rational(
        Rational::fraction(1, 9).unwrap(),
    )));
    let prescaled_24 = raw(Approximation::PrescaledLn(Computable::rational(
        Rational::fraction(1, 24).unwrap(),
    )));
    let prescaled_80 = raw(Approximation::PrescaledLn(Computable::rational(
        Rational::fraction(1, 80).unwrap(),
    )));

    let ln2_1 = Computable::integer(BigInt::from(7_u8)).multiply(prescaled_9);
    let ln2_2 = Computable::integer(BigInt::from(2_u8)).multiply(prescaled_24);
    let ln2_3 = Computable::integer(BigInt::from(3_u8)).multiply(prescaled_80);

    ln2_1
        .add(ln2_2.negate())
        .add(ln2_3)
        .approx_signal(signal, p)
}

fn ln_constant(signal: &Option<Signal>, n: Rational, p: Precision) -> BigInt {
    Computable::rational(n).ln().approx_signal(signal, p)
}

fn sqrt_constant(signal: &Option<Signal>, n: Rational, p: Precision) -> BigInt {
    raw(Approximation::Sqrt(Computable::rational(n))).approx_signal(signal, p)
}

fn e_terms_for_precision(p: Precision) -> u32 {
    let needed_bits = if p < 0 { (-p) as u64 + 4 } else { 4 };
    let mut factorial = BigUint::one();
    let mut n = 0_u32;
    loop {
        let next = &factorial * BigUint::from(n + 1);
        if next.bits() > needed_bits {
            return n;
        }
        factorial = next;
        n += 1;
    }
}

// Returns (P, Q) for sum_{k=a}^{b-1} 1 / prod_{j=a}^k j == P / Q
fn e_binary_split(a: u32, b: u32) -> (BigUint, BigUint) {
    if b - a == 1 {
        return (BigUint::one(), BigUint::from(a));
    }

    let mid = a + (b - a) / 2;
    let (left_p, left_q) = e_binary_split(a, mid);
    let (right_p, right_q) = e_binary_split(mid, b);
    (left_p * &right_q + right_p, left_q * right_q)
}

fn rounded_ratio(numerator: BigUint, denominator: BigUint, p: Precision) -> BigInt {
    if p <= 0 {
        let shift = usize::try_from(-p).expect("precision shift should fit in usize");
        let dividend = numerator << shift;
        BigInt::from((dividend + (&denominator >> 1)) / denominator)
    } else {
        let shift = usize::try_from(p).expect("precision shift should fit in usize");
        let scaled_denominator = denominator << shift;
        BigInt::from((numerator + (&scaled_denominator >> 1)) / scaled_denominator)
    }
}

fn e(p: Precision) -> BigInt {
    let terms = e_terms_for_precision(p);
    if terms == 0 {
        return rounded_ratio(BigUint::one(), BigUint::one(), p);
    }

    let (tail_p, tail_q) = e_binary_split(1, terms + 1);
    rounded_ratio(tail_q.clone() + tail_p, tail_q, p)
}

fn inverse(signal: &Option<Signal>, c: &Computable, p: Precision) -> BigInt {
    let msd = c.iter_msd();
    let inv_msd = 1 - msd;
    let digits_needed = inv_msd - p + 3;
    let prec_needed = msd - digits_needed;
    let log_scale_factor = -p - prec_needed;

    if log_scale_factor < 0 {
        return Zero::zero();
    }

    let dividend = signed::ONE.deref() << log_scale_factor;
    let scaled_divisor = c.approx_signal(signal, prec_needed);
    let abs_scaled_divisor = scaled_divisor.abs();
    let adj_dividend = dividend + (&abs_scaled_divisor >> 1);
    let result: BigInt = adj_dividend / abs_scaled_divisor;

    if scaled_divisor.sign() == Sign::Minus {
        -result
    } else {
        result
    }
}

fn add(signal: &Option<Signal>, c1: &Computable, c2: &Computable, p: Precision) -> BigInt {
    let extra = 4;
    let cutoff = p - extra;
    let (sign1, msd1) = c1.planning_sign_and_msd();
    let (sign2, msd2) = c2.planning_sign_and_msd();
    if sign1 == Some(Sign::NoSign) {
        return c2.approx_signal(signal, p);
    }
    if sign2 == Some(Sign::NoSign) {
        return c1.approx_signal(signal, p);
    }
    let msd1 = msd1.unwrap_or_else(|| c1.msd(cutoff));
    let msd2 = msd2.unwrap_or_else(|| c2.msd(cutoff));

    match (msd1, msd2) {
        (None, None) => return Zero::zero(),
        (None, Some(_)) if sign2.is_some_and(|sign| sign != Sign::NoSign) => {
            return scale(c2.approx_signal(signal, p - extra), -extra);
        }
        (Some(_), None) if sign1.is_some_and(|sign| sign != Sign::NoSign) => {
            return scale(c1.approx_signal(signal, p - extra), -extra);
        }
        (Some(left), Some(_right))
            if sign1 == sign2
                && sign2.is_some_and(|sign| sign != Sign::NoSign)
                && left < cutoff =>
        {
            return scale(c2.approx_signal(signal, p - extra), -extra);
        }
        (Some(_left), Some(right))
            if sign1 == sign2
                && sign1.is_some_and(|sign| sign != Sign::NoSign)
                && right < cutoff =>
        {
            return scale(c1.approx_signal(signal, p - extra), -extra);
        }
        _ => (),
    }

    scale(
        c1.approx_signal(signal, p - 2) + c2.approx_signal(signal, p - 2),
        -2,
    )
}

fn msd_from_appr(prec: Precision, appr: &BigInt) -> Precision {
    prec + appr.magnitude().bits() as Precision - 1
}

fn multiply_with_known_msd(
    signal: &Option<Signal>,
    known: &Computable,
    known_msd: Precision,
    other: &Computable,
    p: Precision,
) -> BigInt {
    let prec_other = p - known_msd - 3;
    let appr_other = other.approx_signal(signal, prec_other);

    if appr_other.sign() == Sign::NoSign {
        return Zero::zero();
    }

    let msd_other = msd_from_appr(prec_other, &appr_other);
    let prec_known = p - msd_other - 3;
    let appr_known = known.approx_signal(signal, prec_known);

    let scale_digits = prec_known + prec_other - p;
    scale(appr_known * appr_other, scale_digits)
}

fn multiply(signal: &Option<Signal>, c1: &Computable, c2: &Computable, p: Precision) -> BigInt {
    let half_prec = (p >> 1) - 1;
    let (sign1, msd1) = c1.planning_sign_and_msd();
    let (sign2, msd2) = c2.planning_sign_and_msd();
    if sign1 == Some(Sign::NoSign) || sign2 == Some(Sign::NoSign) {
        return Zero::zero();
    }
    let msd1 = msd1.unwrap_or_else(|| c1.msd(half_prec));
    let msd2 = msd2.unwrap_or_else(|| c2.msd(half_prec));

    match (msd1, msd2) {
        (None, None) => Zero::zero(),
        (Some(msd_op1), None) => multiply_with_known_msd(signal, c1, msd_op1, c2, p),
        (None, Some(msd_op2)) => multiply_with_known_msd(signal, c2, msd_op2, c1, p),
        (Some(msd_op1), Some(msd_op2)) if msd_op2 > msd_op1 => {
            multiply_with_known_msd(signal, c2, msd_op2, c1, p)
        }
        (Some(msd_op1), Some(_msd_op2)) => multiply_with_known_msd(signal, c1, msd_op1, c2, p),
    }
}

fn square(signal: &Option<Signal>, c: &Computable, p: Precision) -> BigInt {
    let half_prec = (p >> 1) - 1;
    let (sign, msd) = c.planning_sign_and_msd();
    if sign == Some(Sign::NoSign) {
        return Zero::zero();
    }
    let msd = match msd.unwrap_or_else(|| c.msd(half_prec)) {
        None => {
            return Zero::zero();
        }
        Some(msd) => msd,
    };
    let prec = p - msd - 3;

    let appr = c.approx_signal(signal, prec);

    if appr.sign() == Sign::NoSign {
        return Zero::zero();
    }

    let scale_digits = prec + prec - p;
    scale(&appr * &appr, scale_digits)
}

fn ratio(r: &Rational, p: Precision) -> BigInt {
    if p >= 0 {
        scale(r.shifted_big_integer(0), -p)
    } else {
        r.shifted_big_integer(-p)
    }
}

fn offset(signal: &Option<Signal>, c: &Computable, n: i32, p: Precision) -> BigInt {
    c.approx_signal(signal, p - n)
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
fn exp(signal: &Option<Signal>, c: &Computable, p: Precision) -> BigInt {
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

    let max_trunc_error = signed::ONE.deref() << (p - 4 - calc_precision);
    let mut current_term = scaled_1.clone();
    let mut sum = scaled_1;
    let mut n: i32 = 0;

    while current_term.abs() > max_trunc_error {
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
    let fp_prec: i32 = 140;
    let fp_op_prec: i32 = 150;

    let max_prec_needed = p.saturating_mul(2).saturating_sub(1);
    let msd = c
        .planning_msd()
        .unwrap_or_else(|| c.msd(max_prec_needed))
        .unwrap_or(Precision::MIN);

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
fn cos(signal: &Option<Signal>, c: &Computable, p: Precision) -> BigInt {
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

    let max_trunc_error = signed::ONE.deref() << (p - 4 - calc_precision);
    let mut n = 0;
    let mut current_term = signed::ONE.deref() << (-calc_precision);
    let mut current_sum = current_term.clone();

    while current_term.abs() > max_trunc_error {
        if should_stop(signal) {
            break;
        }
        n += 2;

        /* current_term = - current_term * op_squared / n * (n - 1)   */
        current_term = scale(current_term * &op_squared, op_prec);
        let divisor: BigInt = (-(n * (n - 1))).into();
        current_term /= divisor;

        current_sum += &current_term;
    }
    scale(current_sum, calc_precision - p)
}

// Compute sine of |c| < 1
// uses a Taylor series expansion.
fn sin(signal: &Option<Signal>, c: &Computable, p: Precision) -> BigInt {
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

    let max_trunc_error = signed::ONE.deref() << (p - 4 - calc_precision);
    let mut n = 1;
    let mut current_term = scale(op_appr.clone(), op_prec - calc_precision);
    let mut current_sum = current_term.clone();

    while current_term.abs() > max_trunc_error {
        if should_stop(signal) {
            break;
        }
        n += 2;

        /* current_term = - current_term * op_squared / n * (n - 1)   */
        current_term = scale(current_term * &op_squared, op_prec);
        let divisor: BigInt = (-(n * (n - 1))).into();
        current_term /= divisor;

        current_sum += &current_term;
    }
    scale(current_sum, calc_precision - p)
}

// Compute tangent of |c| < 1.
// This uses the direct quotient tan(x) = sin(x) / cos(x),
// but computes both approximations locally to avoid building
// separate Computable trees for sin, cos, inverse, and multiply.
fn tan(signal: &Option<Signal>, c: &Computable, p: Precision) -> BigInt {
    if p >= 1 {
        return Zero::zero();
    }

    let working_prec = p - 8;
    let sin_appr = sin(signal, c, working_prec);
    let cos_appr = cos(signal, c, working_prec);
    let abs_cos = cos_appr.abs();

    if abs_cos.is_zero() {
        panic!("ArithmeticException");
    }

    let scaled_numerator = sin_appr << -p;
    let adjustment = &abs_cos >> 1;

    if scaled_numerator.sign() == Sign::Minus {
        let rounded: BigInt = ((-scaled_numerator) + adjustment) / abs_cos;
        -rounded
    } else {
        (scaled_numerator + adjustment) / abs_cos
    }
}

// Compute cotangent of |c| < 1.
// This mirrors tan(x) = sin(x) / cos(x), but flips the quotient so
// tan(pi/2 - x) can avoid building an extra inverse Computable node.
fn cot(signal: &Option<Signal>, c: &Computable, p: Precision) -> BigInt {
    if p >= 1 {
        return Zero::zero();
    }

    let working_prec = p - 8;
    let sin_appr = sin(signal, c, working_prec);
    let cos_appr = cos(signal, c, working_prec);
    let abs_sin = sin_appr.abs();

    if abs_sin.is_zero() {
        panic!("ArithmeticException");
    }

    let scaled_numerator = cos_appr << -p;
    let adjustment = &abs_sin >> 1;

    if scaled_numerator.sign() == Sign::Minus {
        let rounded: BigInt = ((-scaled_numerator) + adjustment) / abs_sin;
        -rounded
    } else {
        (scaled_numerator + adjustment) / abs_sin
    }
}

// Compute an approximation of ln(1+x) to precision p.
// This assumes |x| < 1/2.
// It uses ln(1+x) = 2 * atanh(x / (2 + x)),
// whose odd-power series converges substantially faster
// than the direct Taylor series when x is near 1/2.
fn ln(signal: &Option<Signal>, c: &Computable, p: Precision) -> BigInt {
    if p >= 0 {
        return Zero::zero();
    }

    let iterations_needed = -p / 2 + 4;
    let calc_precision = p - bound_log2(2 * iterations_needed) - 6;
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

    let max_trunc_error = signed::ONE.deref() << (p - 4 - calc_precision);

    while current_term.abs() > max_trunc_error {
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

// Approximate the Arctangent of 1/n where n is some small integer > base
// what is "base" in this context?
fn atan(signal: &Option<Signal>, i: &BigInt, p: Precision) -> BigInt {
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

    while *current_term.magnitude() > max_trunc_error {
        if should_stop(signal) {
            break;
        }
        n += 2;
        current_power /= &big_op_squared;
        sign = -sign;
        let signed_n: BigInt = (n * sign).into();
        current_term = &current_power / signed_n;
        sum += &current_term;
    }

    scale(sum, calc_precision - p)
}

// Approximate atan(c) for |c| < 1/2.
fn atan_computable(signal: &Option<Signal>, c: &Computable, p: Precision) -> BigInt {
    if p >= 1 {
        return Zero::zero();
    }

    let iterations_needed: i32 = -p / 2 + 4;
    let calc_precision = p - bound_log2(2 * iterations_needed) - 5;
    let op_prec = calc_precision - 3;
    let op_appr = c.approx_signal(signal, op_prec);
    let op_squared = scale(&op_appr * &op_appr, op_prec);

    let max_trunc_error = signed::ONE.deref() << (p - 4 - calc_precision);
    let mut current_term = scale(op_appr, op_prec - calc_precision);
    let mut sum = current_term.clone();
    let mut n = 1;

    while current_term.abs() > max_trunc_error {
        if should_stop(signal) {
            break;
        }
        n += 2;
        current_term = scale(current_term * &op_squared, op_prec);
        current_term *= -(n - 2);
        current_term /= n;
        sum += &current_term;
    }

    scale(sum, calc_precision - p)
}

// Approximate asin(c) for small |c|.
fn asin_computable(signal: &Option<Signal>, c: &Computable, p: Precision) -> BigInt {
    if p >= 1 {
        return Zero::zero();
    }

    let iterations_needed: i32 = -p / 2 + 4;
    let calc_precision = p - bound_log2(2 * iterations_needed) - 5;
    let op_prec = calc_precision - 3;
    let op_appr = c.approx_signal(signal, op_prec);
    let op_squared = scale(&op_appr * &op_appr, op_prec);

    let max_trunc_error = signed::ONE.deref() << (p - 4 - calc_precision);
    let mut current_term = scale(op_appr, op_prec - calc_precision);
    let mut sum = current_term.clone();
    let mut n = 0_i32;

    while current_term.abs() > max_trunc_error {
        if should_stop(signal) {
            break;
        }
        n += 1;
        current_term = scale(current_term * &op_squared, op_prec);
        let numerator = (2 * n - 1) * (2 * n - 1);
        let denominator = (2 * n) * (2 * n + 1);
        current_term *= numerator;
        current_term /= denominator;
        sum += &current_term;
    }

    scale(sum, calc_precision - p)
}

// Approximate atanh(c) for small |c|.
fn atanh_computable(signal: &Option<Signal>, c: &Computable, p: Precision) -> BigInt {
    if p >= 1 {
        return Zero::zero();
    }

    let iterations_needed: i32 = -p / 2 + 4;
    let calc_precision = p - bound_log2(2 * iterations_needed) - 5;
    let op_prec = calc_precision - 3;
    let op_appr = c.approx_signal(signal, op_prec);
    let op_squared = scale(&op_appr * &op_appr, op_prec);

    let max_trunc_error = signed::ONE.deref() << (p - 4 - calc_precision);
    let mut current_power = scale(op_appr, op_prec - calc_precision);
    let mut current_term = current_power.clone();
    let mut sum = current_term.clone();
    let mut n = 1_i32;

    while current_term.abs() > max_trunc_error {
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
