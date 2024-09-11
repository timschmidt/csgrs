use crate::Rational;
use num::{bigint::Sign, BigInt, BigUint};
use num::{One, Zero};
use std::ops::Deref;

pub type Precision = i32;

use std::cell::RefCell;

#[derive(Clone, Debug, PartialEq)]
enum Cache {
    Invalid,
    Valid((Precision, BigInt)),
}

/// Computable approximation of a Real number
#[derive(Debug)]
pub struct Computable {
    internal: Box<dyn Approximation>,
    cache: RefCell<Cache>,
}

mod rationals {
    use crate::Rational;
    use std::sync::LazyLock;

    pub(super) static SHORT_9: LazyLock<Rational> = LazyLock::new(|| Rational::fraction(1, 9));
    pub(super) static SHORT_24: LazyLock<Rational> = LazyLock::new(|| Rational::fraction(1, 24));
    pub(super) static SHORT_80: LazyLock<Rational> = LazyLock::new(|| Rational::fraction(1, 80));
}

mod big {
    use num::One;
    use num::{bigint::ToBigInt, BigInt};
    use std::sync::LazyLock;

    pub(super) static MINUS_ONE: LazyLock<BigInt> =
        LazyLock::new(|| ToBigInt::to_bigint(&-1).unwrap());
    pub(super) static ONE: LazyLock<BigInt> = LazyLock::new(BigInt::one);
    pub(super) static TWO: LazyLock<BigInt> = LazyLock::new(|| ToBigInt::to_bigint(&2).unwrap());
    pub(super) static THREE: LazyLock<BigInt> = LazyLock::new(|| ToBigInt::to_bigint(&3).unwrap());
    pub(super) static FOUR: LazyLock<BigInt> = LazyLock::new(|| ToBigInt::to_bigint(&4).unwrap());
    pub(super) static FIVE: LazyLock<BigInt> = LazyLock::new(|| ToBigInt::to_bigint(&5).unwrap());
    pub(super) static SEVEN: LazyLock<BigInt> = LazyLock::new(|| ToBigInt::to_bigint(&7).unwrap());
    pub(super) static EIGHT: LazyLock<BigInt> = LazyLock::new(|| ToBigInt::to_bigint(&8).unwrap());
    pub(super) static TWENTY_FOUR: LazyLock<BigInt> =
        LazyLock::new(|| ToBigInt::to_bigint(&24).unwrap());
    pub(super) static SIXTY_FOUR: LazyLock<BigInt> =
        LazyLock::new(|| ToBigInt::to_bigint(&64).unwrap());
    pub(super) static TWO_THREE_NINE: LazyLock<BigInt> =
        LazyLock::new(|| ToBigInt::to_bigint(&239).unwrap());
}

impl Computable {
    /// Exactly one
    pub fn one() -> Self {
        Self {
            internal: Box::new(Int(BigInt::one())),
            cache: RefCell::new(Cache::Invalid),
        }
    }

    /// Approximate π, the ratio of a circle's circumference to its diameter
    pub fn pi() -> Self {
        let atan5 = Self::prescaled_atan(big::FIVE.clone());
        let atan_239 = Self::prescaled_atan(big::TWO_THREE_NINE.clone());
        let four = Self::integer(big::FOUR.clone());
        let four_atan5 = Self::multiply(four, atan5);
        let neg = Self::negate(atan_239);
        let sum = Self::add(four_atan5, neg);
        let four = Self::integer(big::FOUR.clone());
        Self::multiply(four, sum)
    }

    /// Any Rational
    pub fn rational(r: Rational) -> Self {
        Self {
            internal: Box::new(Ratio(r)),
            cache: RefCell::new(Cache::Invalid),
        }
    }

    pub fn e(r: Rational) -> Self {
        let rational = Self::rational(r);
        Self::exp(rational)
    }

    /// Square root of any Rational
    pub fn sqrt_rational(r: Rational) -> Self {
        let rational = Self::rational(r);
        Self::sqrt(rational)
    }

    pub fn exp(self) -> Self {
        let low_prec: Precision = -10;
        let rough_appr: BigInt = self.approx(low_prec);
        if rough_appr.sign() == Sign::Minus {
            return self.negate().exp().inverse();
        }
        if rough_appr > *big::TWO {
            let square_root = self.shift_right(1).exp();
            square_root.square()
        } else {
            Self {
                internal: Box::new(Exp(self)),
                cache: RefCell::new(Cache::Invalid),
            }
        }
    }

    fn ln2() -> Self {
        let prescaled_9 = Self::rational(rationals::SHORT_9.clone()).prescaled_ln();
        let prescaled_24 = Self::rational(rationals::SHORT_24.clone()).prescaled_ln();
        let prescaled_80 = Self::rational(rationals::SHORT_80.clone()).prescaled_ln();

        let ln2_1 = Self::integer(big::SEVEN.clone()).multiply(prescaled_9);
        let ln2_2 = Self::integer(big::TWO.clone()).multiply(prescaled_24);
        let ln2_3 = Self::integer(big::THREE.clone()).multiply(prescaled_80);

        let neg_ln2_2 = ln2_2.negate();

        ln2_1.add(neg_ln2_2).add(ln2_3)
    }

    pub fn ln(self) -> Self {
        // Sixteenths, ie 8 == 0.5, 24 == 1.5
        let low_ln_limit = big::EIGHT.deref();
        let high_ln_limit = big::TWENTY_FOUR.deref();

        let low_prec = -4;
        let rough_appr = self.approx(low_prec);
        if rough_appr < BigInt::zero() {
            panic!("ArithmeticException");
        }
        if rough_appr <= *low_ln_limit {
            return self.inverse().ln().negate();
        }
        if rough_appr >= *high_ln_limit {
            // Sixteenths, ie 64 == 4.0
            let sixty_four = big::SIXTY_FOUR.deref();

            if rough_appr <= *sixty_four {
                let quarter = self.sqrt().sqrt().ln();
                return quarter.shift_left(2);
            } else {
                let extra_bits: i32 = (rough_appr.bits() - 3).try_into().expect(
                    "Approximation should have few enough bits to fit in a 32-bit signed integer",
                );
                let scaled_result = self.shift_right(extra_bits).ln();
                let extra: BigInt = extra_bits.into();
                return scaled_result.add(Self::integer(extra).multiply(Self::ln2()));
            }
        }

        let minus_one = Self::integer(big::MINUS_ONE.clone());
        let fraction = Self::add(self, minus_one);
        Self::prescaled_ln(fraction)
    }

    fn prescaled_ln(self) -> Self {
        Self {
            internal: Box::new(PrescaledLn(self)),
            cache: RefCell::new(Cache::Invalid),
        }
    }

    pub fn sqrt(self) -> Self {
        Self {
            internal: Box::new(Sqrt(self)),
            cache: RefCell::new(Cache::Invalid),
        }
    }

    fn prescaled_atan(n: BigInt) -> Self {
        Self {
            internal: Box::new(PrescaledAtan(n)),
            cache: RefCell::new(Cache::Invalid),
        }
    }

    pub fn negate(self) -> Self {
        Self {
            internal: Box::new(Negate(self)),
            cache: RefCell::new(Cache::Invalid),
        }
    }

    pub fn inverse(self) -> Self {
        Self {
            internal: Box::new(Inverse(self)),
            cache: RefCell::new(Cache::Invalid),
        }
    }

    fn shift_left(self, n: i32) -> Self {
        Self {
            internal: Box::new(Shift::new(self, n)),
            cache: RefCell::new(Cache::Invalid),
        }
    }

    fn shift_right(self, n: i32) -> Self {
        Self {
            internal: Box::new(Shift::new(self, -n)),
            cache: RefCell::new(Cache::Invalid),
        }
    }

    pub fn square(self) -> Self {
        Self {
            internal: Box::new(Square(self)),
            cache: RefCell::new(Cache::Invalid),
        }
    }

    pub fn multiply(self, other: Self) -> Self {
        Self {
            internal: Box::new(Multiply::new(self, other)),
            cache: RefCell::new(Cache::Invalid),
        }
    }

    #[allow(clippy::should_implement_trait)]
    pub fn add(self, other: Computable) -> Self {
        Self {
            internal: Box::new(Add::new(self, other)),
            cache: RefCell::new(Cache::Invalid),
        }
    }

    fn integer(n: BigInt) -> Self {
        Self {
            internal: Box::new(Int(n)),
            cache: RefCell::new(Cache::Invalid),
        }
    }
}

impl Computable {
    /// An approximation of this Computable scaled to a specific precision
    ///
    /// The approximation is scaled (thus, a larger value for more negative p)
    /// and should be accurate to within +/- 1 at the scale provided
    ///
    /// Example: 0.875 is between 0 and 1 with zero bits of extra precision
    /// ```
    /// use realistic::{Rational,Computable};
    /// use num::{Zero,One};
    /// use num::bigint::{BigInt,ToBigInt};
    /// let n = Rational::fraction(7, 8);
    /// let comp = Computable::rational(n);
    /// assert!((BigInt::zero() ..= BigInt::one()).contains(&comp.approx(0)));
    /// ```
    ///
    /// Example: π * 2³ is a bit more than 25
    /// ```
    /// use realistic::{Rational,Computable};
    /// use num::{Zero,One};
    /// use num::bigint::{BigInt,ToBigInt};
    /// let pi = Computable::pi();
    /// assert!((ToBigInt::to_bigint(&25).unwrap() ..= ToBigInt::to_bigint(&26).unwrap()).contains(&pi.approx(-3)));
    /// ```
    pub fn approx(&self, p: Precision) -> BigInt {
        // Check precision is OK?

        if let Cache::Valid((cache_prec, cache_appr)) = self.cache.clone().into_inner() {
            if p >= cache_prec {
                return scale(cache_appr, cache_prec - p);
            }
        }
        let result = self.internal.approximate(p);
        self.cache.replace(Cache::Valid((p, result.clone())));
        result
    }

    pub fn sign(&self) -> Sign {
        if let Cache::Valid((_prec, cache_appr)) = self.cache.clone().into_inner() {
            let sign = cache_appr.sign();
            if sign != Sign::NoSign {
                return sign;
            }
        }
        let mut sign = Sign::NoSign;
        let mut p = 0;
        while p > -1000 && sign == Sign::NoSign {
            let appr = self.approx(p);
            p -= 10;
            sign = appr.sign();
        }
        sign
    }

    fn cached(&self) -> Option<(Precision, BigInt)> {
        if let Cache::Valid((cache_prec, cache_appr)) = self.cache.clone().into_inner() {
            Some((cache_prec, cache_appr))
        } else {
            None
        }
    }
}

use core::cmp::Ordering;

impl Computable {
    /// Do not call this function if `self` and `other` may be the same
    pub fn compare_to(&self, other: &Self) -> Ordering {
        let mut tolerance = -20;
        while tolerance > Precision::MIN {
            let order = self.compare_absolute(other, tolerance);
            if order != Ordering::Equal {
                return order;
            }
            tolerance *= 2;
        }
        panic!("Apparently called Computable::compare_to on equal values");
    }

    pub fn compare_absolute(&self, other: &Self, tolerance: Precision) -> Ordering {
        let needed = tolerance - 1;
        let this = self.approx(needed);
        let alt = other.approx(needed);
        let max = alt.clone() + big::ONE.deref();
        let min = alt.clone() - big::ONE.deref();
        if this > max {
            Ordering::Greater
        } else if this < min {
            Ordering::Less
        } else {
            Ordering::Equal
        }
    }

    /// Most Significant Digit (Bit) ?
    /// May panic or give incorrect answers if not yet discovered
    fn known_msd(&self) -> Precision {
        if let Some((prec, appr)) = self.cached() {
            let length = appr.magnitude().bits() as Precision;
            prec + length - 1
        } else {
            panic!("Expected valid cache state for known MSD but it's invalid")
        }
    }

    /// MSD - but Precision::MIN if as yet undiscovered
    fn msd(&self, p: Precision) -> Option<Precision> {
        let cache = self.cached();
        let mut try_once = false;

        if cache.is_none() {
            try_once = true;
        } else if let Some((_prec, appr)) = cache {
            let one = big::ONE.deref();
            let minus_one = big::MINUS_ONE.deref();

            if appr > *minus_one && appr < *one {
                try_once = true;
            }
        }

        if try_once {
            let appr = self.approx(p - 1);
            if appr.magnitude() < &BigUint::one() {
                return None;
            }
        }

        Some(self.known_msd())
    }

    /// MSD but iteratively without a guess as to precision
    fn iter_msd(&self) -> Precision {
        let mut prec = 0;

        // prec = 0, -16, -40, -76, etc.
        loop {
            let msd = self.msd(prec);
            if let Some(msd) = msd {
                return msd;
            }
            prec = (prec * 3) / 2 - 16;
            if prec <= Precision::MIN + 30 {
                break;
            }
        }
        self.msd(Precision::MIN)
            .expect("Should have a known MSD by now")
    }
}

fn shift(n: BigInt, p: Precision) -> BigInt {
    match 0.cmp(&p) {
        Ordering::Greater => n >> -p,
        Ordering::Equal => n,
        Ordering::Less => n << p,
    }
}

/// Scale n by p bits, rounding if this makes n smaller
/// e.g. scale(10, 2) == 40
///      scale(10, -2) == 3
fn scale(n: BigInt, p: Precision) -> BigInt {
    if p >= 0 {
        n << p
    } else {
        let adj = shift(n, p + 1) + big::ONE.deref();
        adj >> 1
    }
}

trait Approximation: core::fmt::Debug {
    /* TODO maybe provide some mechanism to request computation stops? */

    /// result is within 1 but scaled by 2 ^ p
    /// So e.g. pi with p=0 is 3, pi with p=2 = 314
    fn approximate(&self, p: Precision) -> BigInt;
}

#[derive(Debug)]
struct Int(BigInt);

impl Approximation for Int {
    fn approximate(&self, p: Precision) -> BigInt {
        scale(self.0.clone(), -p)
    }
}

#[derive(Debug)]
struct Inverse(Computable);

use num::Signed;

impl Approximation for Inverse {
    fn approximate(&self, p: Precision) -> BigInt {
        let msd = self.0.iter_msd();
        let inv_msd = 1 - msd;
        let digits_needed = inv_msd - p + 3;
        let prec_needed = msd - digits_needed;
        let log_scale_factor = -p - prec_needed;

        if log_scale_factor < 0 {
            return Zero::zero();
        }

        let dividend = big::ONE.deref() << log_scale_factor;
        let scaled_divisor = self.0.approx(prec_needed);
        let abs_scaled_divisor = scaled_divisor.abs();
        let adj_dividend = dividend + (&abs_scaled_divisor >> 1);
        let result: BigInt = adj_dividend / abs_scaled_divisor;

        if scaled_divisor.sign() == Sign::Minus {
            -result
        } else {
            result
        }
    }
}

#[derive(Debug)]
struct Negate(Computable);

impl Approximation for Negate {
    fn approximate(&self, p: Precision) -> BigInt {
        -self.0.approx(p)
    }
}

#[derive(Debug)]
struct Add {
    a: Computable,
    b: Computable,
}

impl Add {
    fn new(a: Computable, b: Computable) -> Self {
        Self { a, b }
    }
}

impl Approximation for Add {
    fn approximate(&self, p: Precision) -> BigInt {
        scale(self.a.approx(p - 2) + self.b.approx(p - 2), -2)
    }
}

#[derive(Debug)]
struct Multiply {
    a: Computable,
    b: Computable,
}

impl Multiply {
    fn new(a: Computable, b: Computable) -> Self {
        Self { a, b }
    }
}

impl Approximation for Multiply {
    fn approximate(&self, p: Precision) -> BigInt {
        let half_prec = (p >> 1) - 1;

        match self.a.msd(half_prec) {
            None => match self.b.msd(half_prec) {
                None => Zero::zero(),
                Some(msd_op2) => {
                    let prec1 = p - msd_op2 - 3;
                    let appr1 = self.a.approx(prec1);

                    if appr1.sign() == Sign::NoSign {
                        return Zero::zero();
                    }

                    let msd_op1 = self.a.known_msd();
                    let prec2 = p - msd_op1 - 3;
                    let appr2 = self.b.approx(prec2);

                    let scale_digits = prec2 + prec1 - p;
                    scale(appr2 * appr1, scale_digits)
                }
            },
            Some(msd_op1) => {
                let prec2 = p - msd_op1 - 3;
                let appr2 = self.b.approx(prec2);

                if appr2.sign() == Sign::NoSign {
                    return Zero::zero();
                }

                let msd_op2 = self.b.known_msd();
                let prec1 = p - msd_op2 - 3;
                let appr1 = self.a.approx(prec1);

                let scale_digits = prec1 + prec2 - p;
                scale(appr1 * appr2, scale_digits)
            }
        }
    }
}

#[derive(Debug)]
struct Square(Computable);

impl Approximation for Square {
    fn approximate(&self, p: Precision) -> BigInt {
        let half_prec = (p >> 1) - 1;
        let prec2 = match self.0.msd(half_prec) {
            None => {
                return Zero::zero();
            }
            Some(msd) => p - msd - 3,
        };

        let appr2 = self.0.approx(prec2);

        if appr2.sign() == Sign::NoSign {
            return Zero::zero();
        }

        let msd_op2 = self.0.known_msd();
        let prec1 = p - msd_op2 - 3;
        let appr1 = self.0.approx(prec1);

        let scale_digits = prec1 + prec2 - p;
        scale(appr1 * appr2, scale_digits)
    }
}

#[derive(Debug)]
struct Shift {
    a: Computable,
    n: i32,
}

impl Shift {
    fn new(a: Computable, n: i32) -> Self {
        Self { a, n }
    }
}

impl Approximation for Shift {
    fn approximate(&self, p: Precision) -> BigInt {
        self.a.approx(p - self.n)
    }
}

#[derive(Debug)]
struct Ratio(Rational);

impl Approximation for Ratio {
    fn approximate(&self, p: Precision) -> BigInt {
        if p >= 0 {
            scale(self.0.shifted_big_integer(0), -p)
        } else {
            self.0.shifted_big_integer(-p)
        }
    }
}

#[derive(Debug)]
struct Exp(Computable);

/// Only intended for Computable values < 0.5, others will be pre-scaled
/// in Computable::exp
impl Approximation for Exp {
    fn approximate(&self, p: Precision) -> BigInt {
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

        let op_appr = self.0.approx(op_prec);

        // Error in argument results in error of < 3/8 ulp.
        // Sum of term eval. rounding error is < 1/16 ulp.
        // Series truncation error < 1/16 ulp.
        // Final rounding error is <= 1/2 ulp.
        // Thus final error is < 1 ulp.
        let scaled_1 = big::ONE.deref() << -calc_precision;

        let max_trunc_error = big::ONE.deref() << (p - 4 - calc_precision);
        let mut current_term = scaled_1.clone();
        let mut sum = scaled_1;
        let mut n = BigInt::zero();

        while current_term.abs() > max_trunc_error {
            n += big::ONE.deref();
            current_term = scale(current_term * &op_appr, op_prec) / &n;
            sum += &current_term;
        }

        scale(sum, calc_precision - p)
    }
}

#[derive(Debug)]
struct PrescaledLn(Computable);

// Compute an approximation of ln(1+x) to precision p.
// This assumes |x| < 1/2.
// It uses a Taylor series expansion.
// Unfortunately there appears to be no way to take
// advantage of old information.
// Note: this is known to be a bad algorithm for
// floating point.  Unfortunately, other alternatives
// appear to require precomputed tabular information.
impl Approximation for PrescaledLn {
    fn approximate(&self, p: Precision) -> BigInt {
        if p >= 0 {
            return Zero::zero();
        }

        let iterations_needed = -p;
        let calc_precision = p - bound_log2(2 * iterations_needed) - 4;
        let op_prec = p - 3;
        let op_appr = self.0.approx(op_prec);

        let mut x_nth = scale(op_appr.clone(), op_prec - calc_precision);
        let mut current_term = x_nth.clone();
        let mut sum = current_term.clone();

        let mut n = 1;
        let mut sign = 1;

        let max_trunc_error = big::ONE.deref() << (p - 4 - calc_precision);

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
}

#[derive(Debug)]
struct Sqrt(Computable);

impl Approximation for Sqrt {
    fn approximate(&self, p: Precision) -> BigInt {
        let fp_prec: i32 = 50;
        let fp_op_prec: i32 = 60;

        let max_prec_needed = 2 * p - 1;
        let msd = self.0.msd(max_prec_needed).unwrap_or(Precision::MIN);

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

            let last_appr = self.approximate(appr_prec);
            let prod_prec = 2 * appr_prec;

            let op_appr = self.0.approx(prod_prec);

            // Slightly fewer might be enough;
            // Compute (last_appr * last_appr + op_appr)/(last_appr/2)
            // while adjusting the scaling to make everything work

            let prod_prec_scaled_numerator = (&last_appr * &last_appr) + op_appr;
            let scaled_numerator = scale(prod_prec_scaled_numerator, appr_prec - p);

            let shifted_result = scaled_numerator / last_appr;

            (shifted_result + big::ONE.deref()) / big::TWO.deref()
        } else {
            // Use an approximation from the Num crate
            // Make sure all precisions are even
            let op_prec = (msd - fp_op_prec) & !1;
            let working_prec = op_prec - fp_op_prec;

            let scaled_bi_appr = self.0.approx(op_prec) << fp_op_prec;

            let scaled_sqrt = scaled_bi_appr.sqrt();

            let shift_count = working_prec / 2 - p;
            shift(scaled_sqrt, shift_count)
        }
    }
}

fn bound_log2(n: i32) -> i32 {
    let abs_n = n.abs();
    let ln2 = 2.0_f64.ln();
    let n_plus_1: f64 = (abs_n + 1).into();
    let ans: f64 = (n_plus_1.ln() / ln2).ceil();
    ans as i32
}

// PrescaledAtan(n) is the Arctangent of 1/n where n is some small integer > base
// what is "base" in this context?
#[derive(Debug)]
struct PrescaledAtan(BigInt);

impl Approximation for PrescaledAtan {
    fn approximate(&self, p: Precision) -> BigInt {
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

        let scaled_1 = big::ONE.deref() << (-calc_precision);
        let big_op_squared: BigInt = &self.0 * &self.0;
        let inverse: BigInt = scaled_1 / &self.0;

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
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn compare() {
        let six: BigInt = "6".parse().unwrap();
        let five: BigInt = "5".parse().unwrap();
        let four: BigInt = "4".parse().unwrap();
        let six = Computable::integer(six.clone());
        let five = Computable::integer(five.clone());
        let four = Computable::integer(four.clone());

        assert_eq!(six.compare_to(&five), Ordering::Greater);
        assert_eq!(five.compare_to(&six), Ordering::Less);
        assert_eq!(four.compare_to(&six), Ordering::Less);
    }

    #[test]
    fn bigger() {
        let six: BigInt = "6".parse().unwrap();
        let five: BigInt = "5".parse().unwrap();
        let four: BigInt = "4".parse().unwrap();
        let a = Computable::integer(six.clone());
        let b = Computable::integer(five.clone());
        assert_eq!(a.compare_absolute(&b, 0), Ordering::Greater);
        let c = Computable::integer(four.clone());
        assert_eq!(c.compare_absolute(&a, 0), Ordering::Less);
        assert_eq!(b.compare_absolute(&b, 0), Ordering::Equal);
    }

    #[test]
    fn shifted() {
        let one = BigInt::one();
        let two = &one + &one;
        assert_eq!(one, shift(two, -1));
    }

    #[test]
    fn prec() {
        let nine: BigInt = "9".parse().unwrap();
        let five: BigInt = "5".parse().unwrap();
        let two: BigInt = "2".parse().unwrap();
        let one = BigInt::one();
        let a = Computable::integer(nine.clone());
        assert_eq!(nine, a.approx(0));
        assert_eq!(five, a.approx(1));
        assert_eq!(two, a.approx(2));
        assert_eq!(one, a.approx(3));
        assert_eq!(Cache::Valid((0, nine)), a.cache.into_inner());
    }

    #[test]
    fn prec_pi() {
        let three: BigInt = "3".parse().unwrap();
        let six: BigInt = "6".parse().unwrap();
        let thirteen: BigInt = "13".parse().unwrap();
        let four_zero_two: BigInt = "402".parse().unwrap();
        let a = Computable::pi();
        assert_eq!(four_zero_two, a.approx(-7));
        assert_eq!(three, a.approx(0));
        assert_eq!(six, a.approx(-1));
        assert_eq!(thirteen, a.approx(-2));
        assert_eq!(Cache::Valid((-7, four_zero_two)), a.cache.into_inner());
    }

    #[test]
    fn prec_atan_5() {
        let five: BigInt = "5".parse().unwrap();
        let atan_5 = Computable::prescaled_atan(five);
        let two_zero_two: BigInt = "202".parse().unwrap();
        assert_eq!(two_zero_two, atan_5.approx(-10));
        let at_twenty: BigInt = "206984".parse().unwrap();
        assert_eq!(at_twenty, atan_5.approx(-20));
    }

    #[test]
    fn prec_atan_239() {
        let two_three_nine: BigInt = "239".parse().unwrap();
        let atan_239 = Computable::prescaled_atan(two_three_nine);
        let four: BigInt = "4".parse().unwrap();
        assert_eq!(four, atan_239.approx(-10));
        let at_twenty: BigInt = "4387".parse().unwrap();
        assert_eq!(at_twenty, atan_239.approx(-20));
    }

    #[test]
    fn msd() {
        let one: BigInt = "1".parse().unwrap();
        let a = Computable::integer(one.clone());
        assert_eq!(Some(0), a.msd(-4));
        let three: BigInt = "3".parse().unwrap();
        let d = Computable::integer(three.clone());
        assert_eq!(Some(1), d.msd(-4));
        let five: BigInt = "5".parse().unwrap();
        let e = Computable::integer(five.clone());
        assert_eq!(Some(2), e.msd(-4));
        let seven: BigInt = "7".parse().unwrap();
        let f = Computable::integer(seven.clone());
        assert_eq!(Some(2), f.msd(-4));
        let eight: BigInt = "8".parse().unwrap();
        let g = Computable::integer(eight.clone());
        assert_eq!(Some(3), g.msd(-4));
    }

    #[test]
    fn negate() {
        let fifteen: BigInt = "15".parse().unwrap();
        let a = Computable::integer(fifteen.clone());
        let b = Computable::negate(a);
        let answer: BigInt = "-8".parse().unwrap();
        assert_eq!(answer, b.approx(1));
    }

    #[test]
    fn multiply() {
        let four: BigInt = "4".parse().unwrap();
        let five: BigInt = "5".parse().unwrap();
        let a = Computable::integer(four);
        let b = Computable::prescaled_atan(five);
        let m = Computable::multiply(a, b);
        let answer: BigInt = "809".parse().unwrap();
        assert_eq!(answer, m.approx(-10));
    }

    #[test]
    fn multiply_opposite() {
        let four: BigInt = "4".parse().unwrap();
        let five: BigInt = "5".parse().unwrap();
        let a = Computable::integer(four);
        let b = Computable::prescaled_atan(five);
        let m = Computable::multiply(b, a);
        let answer: BigInt = "809".parse().unwrap();
        assert_eq!(answer, m.approx(-10));
    }

    #[test]
    fn rational() {
        let sixth: Rational = "1/6".parse().unwrap();
        let c = Computable::rational(sixth);
        let zero = BigInt::zero();
        let one = BigInt::one();
        let ten: BigInt = "10".parse().unwrap();
        let eighty_five: BigInt = "85".parse().unwrap();
        assert_eq!(zero, c.approx(0));
        assert_eq!(zero, c.approx(-1));
        assert_eq!(zero, c.approx(-2));
        assert_eq!(one, c.approx(-3));
        assert_eq!(ten, c.approx(-6));
        assert_eq!(eighty_five, c.approx(-9));
    }

    #[test]
    fn scaled_ln1() {
        let zero = Computable::integer(BigInt::zero());
        let ln = Computable {
            internal: Box::new(PrescaledLn(zero)),
            cache: RefCell::new(Cache::Invalid),
        };
        let zero = BigInt::zero();
        assert_eq!(zero, ln.approx(100));
    }

    #[test]
    fn scaled_ln1_4() {
        let zero_4: Rational = "0.4".parse().unwrap();
        let rational = Computable::rational(zero_4);
        let ln = Computable {
            internal: Box::new(PrescaledLn(rational)),
            cache: RefCell::new(Cache::Invalid),
        };
        let five: BigInt = "5".parse().unwrap();
        assert_eq!(five, ln.approx(-4));
    }

    #[test]
    fn ln() {
        let five: BigInt = "5".parse().unwrap();
        let integer = Computable::integer(five);
        let ln = Computable::ln(integer);
        let correct: BigInt = "1769595698905".parse().unwrap();
        assert_eq!(ln.approx(-40), correct);
    }

    #[test]
    fn ln_sqrt_pi() {
        let pi = Computable::pi();
        let sqrt = Computable::sqrt(pi);
        let ln = Computable::ln(sqrt);
        let correct: BigInt = "629321910077".parse().unwrap();
        assert_eq!(ln.approx(-40), correct);
    }

    #[test]
    fn add() {
        let three: BigInt = "3".parse().unwrap();
        let five: BigInt = "5".parse().unwrap();
        let a = Computable::integer(three);
        let b = Computable::integer(five);
        let c = Computable::add(a, b);
        let answer: BigInt = "256".parse().unwrap();
        assert_eq!(answer, c.approx(-5));
    }

    #[test]
    fn scale_up() {
        let ten: BigInt = "10".parse().unwrap();
        let three: BigInt = "3".parse().unwrap();
        assert_eq!(ten, scale(ten.clone(), 0));
        let a = scale(ten.clone(), -2);
        assert_eq!(three, a);
        let forty: BigInt = "40".parse().unwrap();
        let b = scale(ten.clone(), 2);
        assert_eq!(forty, b);
    }
}
