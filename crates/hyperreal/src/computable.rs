use crate::BoundedRational;
use num_bigint::{BigInt, BigUint, Sign};
use num_traits::{One, Zero};

pub type Precision = i32;

use std::cell::RefCell;

#[derive(Clone, Debug, PartialEq)]
enum Cache {
    Invalid,
    Valid((Precision, BigInt)),
}

#[derive(Debug)]
pub struct Computable {
    internal: Box<dyn Approximation>,
    cache: RefCell<Cache>,
}

impl Clone for Computable {
    fn clone(&self) -> Self {
        /* FIXME: this provides a placeholder instead of actually cloning the box */
        let internal = Box::new(Placeholder);
        Self {
            internal,
            cache: self.cache.clone(),
        }
    }
}

impl Computable {
    pub fn one() -> Self {
        Self {
            internal: Box::new(Int(BigInt::one())),
            cache: RefCell::new(Cache::Invalid),
        }
    }

    pub fn pi() -> Self {
        let five: BigInt = "5".parse().unwrap();
        let atan5 = Self::atan(five);
        let two_three_nine: BigInt = "239".parse().unwrap();
        let atan_239 = Self::atan(two_three_nine);
        let four: BigInt = "4".parse().unwrap();
        let four = Self::integer(four);
        let four_atan5 = Self::multiply(four, atan5);
        let neg = Self::negate(atan_239);
        let sum = Self::add(four_atan5, neg);
        let four: BigInt = "4".parse().unwrap();
        let four = Self::integer(four);
        Self::multiply(four, sum)
    }

    pub fn rational(r: BoundedRational) -> Self {
        Self {
            internal: Box::new(Rational(r)),
            cache: RefCell::new(Cache::Invalid),
        }
    }

    pub fn e(r: BoundedRational) -> Self {
        let rational = Self::rational(r);
        Self::exp(rational)
    }

    pub fn exp(self) -> Self {
        let low_prec: Precision = -10;
        let rough_appr: BigInt = self.approx(low_prec);
        if rough_appr.sign() == Sign::Minus {
            return self.negate().exp().inverse();
        }
        if rough_appr > "2".parse().unwrap() {
            let square_root = self.shift_right(1).exp();
            square_root.square()
        } else {
            Self {
                internal: Box::new(Exp(self)),
                cache: RefCell::new(Cache::Invalid),
            }
        }
    }

    pub fn sqrt(r: BoundedRational) -> Self {
        let rational = Self::rational(r);
        Self::sqrt_computable(rational)
    }

    pub fn sqrt_computable(self) -> Self {
        Self {
            internal: Box::new(Sqrt(self)),
            cache: RefCell::new(Cache::Invalid),
        }
    }

    fn atan(n: BigInt) -> Self {
        Self {
            internal: Box::new(Atan(n)),
            cache: RefCell::new(Cache::Invalid),
        }
    }

    fn negate(self) -> Self {
        Self {
            internal: Box::new(Negate(self)),
            cache: RefCell::new(Cache::Invalid),
        }
    }

    fn inverse(self) -> Self {
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

    // Caution: currently a > b (or maybe a >= b) or else
    fn multiply(a: Computable, b: Computable) -> Self {
        Self {
            internal: Box::new(Multiply::new(a, b)),
            cache: RefCell::new(Cache::Invalid),
        }
    }

    fn add(a: Computable, b: Computable) -> Self {
        Self {
            internal: Box::new(Add::new(a, b)),
            cache: RefCell::new(Cache::Invalid),
        }
    }

    fn integer(n: BigInt) -> Self {
        Self {
            internal: Box::new(Int(n)),
            cache: RefCell::new(Cache::Invalid),
        }
    }

    pub fn todo() -> Self {
        Self {
            internal: Box::new(Placeholder),
            cache: RefCell::new(Cache::Invalid),
        }
    }
}

impl Computable {
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
        let max = alt.clone() + BigInt::one();
        let min = alt.clone() - BigInt::one();
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
    fn msd(&self, p: Precision) -> Precision {
        let big1: BigInt = One::one();
        let bigm1: BigInt = "-1".parse().unwrap();

        let cache = self.cached();
        let mut try_once = false;

        if cache.is_none() {
            try_once = true;
        } else if let Some((_prec, appr)) = cache {
            if appr > bigm1 && appr < big1 {
                try_once = true;
            }
        }

        if try_once {
            let appr = self.approx(p - 1);
            if appr.magnitude() < &BigUint::one() {
                return Precision::MIN;
            }
        }

        self.known_msd()
    }

    /// MSD but iteratively without a guess as to precision
    fn iter_msd(&self) -> Precision {
        let mut prec = 0;

        // prec = 0, -16, -40, -76, etc.
        loop {
            let msd = self.msd(prec);
            if msd != Precision::MIN {
                return msd;
            }
            prec = (prec * 3) / 2 - 16;
            if prec <= Precision::MIN + 30 {
                break;
            }
        }
        self.msd(Precision::MIN)
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
        let one: BigInt = One::one();
        let adj = shift(n, p + 1) + one;
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
struct Placeholder;

impl Approximation for Placeholder {
    fn approximate(&self, _p: Precision) -> BigInt {
        todo!("Placeholder instead of an actual Computable Real")
    }
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

use num_traits::Signed;

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

        let dividend = BigInt::one() << log_scale_factor;
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
    /// NB: a should have a larger magnitude than b
    /// if that can't be arranged, we need to swap them to make approximate work as intended
    fn new(a: Computable, b: Computable) -> Self {
        Self { a, b }
    }
}

impl Approximation for Multiply {
    fn approximate(&self, p: Precision) -> BigInt {
        let half_prec = (p >> 1) - 1;
        let msd_op1 = self.a.msd(half_prec);

        if msd_op1 == Precision::MIN {
            let msd_op2 = self.b.msd(half_prec);
            if msd_op2 == Precision::MIN {
                return Zero::zero();
            } else {
                panic!("Multiply(A,B) expects A has larger magnitude than B");
            }
        }
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

#[derive(Debug)]
struct Square(Computable);

impl Approximation for Square {
    fn approximate(&self, p: Precision) -> BigInt {
        let half_prec = (p >> 1) - 1;
        let msd = self.0.msd(half_prec);

        if msd == Precision::MIN {
            return Zero::zero();
        }

        let prec2 = p - msd - 3;
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
struct Rational(BoundedRational);

impl Approximation for Rational {
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
        let scaled_1 = BigInt::one() << -calc_precision;

        let max_trunc_error = BigInt::one() << (p - 4 - calc_precision);
        let mut current_term = scaled_1.clone();
        let mut current_sum = scaled_1;
        let mut n = BigInt::zero();

        while current_term.abs() > max_trunc_error {
            n += BigInt::one();
            current_term = scale(current_term * &op_appr, op_prec);
            current_term /= &n;
            current_sum += &current_term;
        }

        scale(current_sum, calc_precision - p)
    }
}

#[derive(Debug)]
struct Sqrt(Computable);

impl Approximation for Sqrt {
    fn approximate(&self, p: Precision) -> BigInt {
        let fp_prec: i32 = 50;
        let fp_op_prec: i32 = 60;

        let max_prec_needed = 2 * p - 1;
        let msd = self.0.msd(max_prec_needed);

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

            //// BigInteger last_appr = get_appr(appr_prec);
            let last_appr = self.approximate(appr_prec);

            let prod_prec = 2 * appr_prec;

            //// BigInteger op_appr = op.get_appr(prod_prec);
            let op_appr = self.0.approx(prod_prec);

            // Slightly fewer might be enough;
            // Compute (last_appr * last_appr + op_appr)/(last_appr/2)
            // while adjusting the scaling to make everything work

            let prod_prec_scaled_numerator = (&last_appr * &last_appr) + op_appr;
            let scaled_numerator = scale(prod_prec_scaled_numerator, appr_prec - p);

            let shifted_result = scaled_numerator / last_appr;

            let two: BigInt = "2".parse().unwrap();
            (shifted_result + BigInt::one()) / two
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

//// static int bound_log2(int n) {
////	int abs_n = Math.abs(n);
////	return (int)Math.ceil(Math.log((double)(abs_n + 1))/Math.log(2.0));
//// }

fn bound_log2(n: i32) -> i32 {
    let abs_n = n.abs();
    let ln2 = 2.0_f64.ln();
    let n_plus_1: f64 = (abs_n + 1).into();
    let ans: f64 = (n_plus_1.ln() / ln2).ceil();
    ans as i32
}

// Atan(n) is the Arctangent of 1/n where n is some small integer > base
// what is "base" in this context?
#[derive(Debug)]
struct Atan(BigInt);

impl Approximation for Atan {
    fn approximate(&self, p: Precision) -> BigInt {
        if p >= 1 {
            return Zero::zero();
        }

        let iterations_needed: i32 = -p / 2 + 2; // conservative estimate > 0.
                                                 //  Claim: each intermediate term is accurate
                                                 //  to 2*base^calc_precision.
                                                 //  Total rounding error in series computation is
                                                 //  2*iterations_needed*base^calc_precision,
                                                 //  exclusive of error in op.

        //// int calc_precision = p - bound_log2(2*iterations_needed)
        ////		       - 2; // for error in op, truncation.

        let calc_precision = p - bound_log2(2 * iterations_needed) - 2;

        // Error in argument results in error of < 3/8 ulp.
        // Cumulative arithmetic rounding error is < 1/4 ulp.
        // Series truncation error < 1/4 ulp.
        // Final rounding error is <= 1/2 ulp.
        // Thus final error is < 1 ulp.

        //// BigInteger scaled_1 = big1.shiftLeft(-calc_precision);
        let scaled_1: BigInt = <BigInt as One>::one() << (-calc_precision);

        //// BigInteger big_op = BigInteger.valueOf(op);
        //// BigInteger big_op_squared = BigInteger.valueOf(op*op);
        let big_op_squared: BigInt = &self.0 * &self.0;

        //// BigInteger op_inverse = scaled_1.divide(big_op);
        let op_inverse: BigInt = scaled_1 / &self.0;
        //// BigInteger current_power = op_inverse;
        let mut current_power: BigInt = op_inverse.clone();

        //// BigInteger current_term = op_inverse;
        let mut current_term: BigInt = op_inverse.clone();

        //// BigInteger current_sum = op_inverse;
        let mut current_sum: BigInt = op_inverse.clone();

        //// int current_sign = 1;
        let mut current_sign = 1;
        //// int n = 1;
        let mut n = 1;

        //// BigInteger max_trunc_error = big1.shiftLeft(p - 2 - calc_precision);
        let max_trunc_error: BigUint = <BigUint as One>::one() << (p - 2 - calc_precision);

        //// while (current_term.abs().compareTo(max_trunc_error) >= 0) {
        while *current_term.magnitude() > max_trunc_error {
            //// if (Thread.interrupted() || please_stop) throw new AbortedError();
            n += 2;
            current_power = current_power / &big_op_squared;
            current_sign = -current_sign;
            let signed_n: BigInt = (current_sign * n).into();
            current_term = &current_power / signed_n;
            current_sum = current_sum + &current_term;
        }

        scale(current_sum, calc_precision - p)
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
        let one: BigInt = One::one();
        let two = one.clone() + one.clone();
        assert_eq!(one, shift(two, -1));
    }

    #[test]
    fn prec() {
        let nine: BigInt = "9".parse().unwrap();
        let five: BigInt = "5".parse().unwrap();
        let two: BigInt = "2".parse().unwrap();
        let one: BigInt = One::one();
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
        let atan_5 = Computable::atan(five);
        let two_zero_two: BigInt = "202".parse().unwrap();
        assert_eq!(two_zero_two, atan_5.approx(-10));
        let at_twenty: BigInt = "206984".parse().unwrap();
        assert_eq!(at_twenty, atan_5.approx(-20));
    }

    #[test]
    fn prec_atan_239() {
        let two_three_nine: BigInt = "239".parse().unwrap();
        let atan_239 = Computable::atan(two_three_nine);
        let four: BigInt = "4".parse().unwrap();
        assert_eq!(four, atan_239.approx(-10));
        let at_twenty: BigInt = "4387".parse().unwrap();
        assert_eq!(at_twenty, atan_239.approx(-20));
    }

    #[test]
    fn msd() {
        let one: BigInt = "1".parse().unwrap();
        let a = Computable::integer(one.clone());
        assert_eq!(0, a.msd(-4));
        let three: BigInt = "3".parse().unwrap();
        let d = Computable::integer(three.clone());
        assert_eq!(1, d.msd(-4));
        let five: BigInt = "5".parse().unwrap();
        let e = Computable::integer(five.clone());
        assert_eq!(2, e.msd(-4));
        let seven: BigInt = "7".parse().unwrap();
        let f = Computable::integer(seven.clone());
        assert_eq!(2, f.msd(-4));
        let eight: BigInt = "8".parse().unwrap();
        let g = Computable::integer(eight.clone());
        assert_eq!(3, g.msd(-4));
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
        let b = Computable::atan(five);
        let m = Computable::multiply(a, b);
        let answer: BigInt = "809".parse().unwrap();
        assert_eq!(answer, m.approx(-10));
    }

    #[test]
    fn rational() {
        let sixth: BoundedRational = "1/6".parse().unwrap();
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
