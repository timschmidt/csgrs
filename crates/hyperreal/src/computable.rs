use num_bigint::{BigInt, BigUint};
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
        Self {
            internal: Box::new(Pi),
            cache: RefCell::new(Cache::Invalid),
        }
    }

    pub fn atan(n: BigInt) -> Self {
        Self {
            internal: Box::new(Atan(n)),
            cache: RefCell::new(Cache::Invalid),
        }
    }

    #[cfg(test)]
    fn integer(n: BigInt) -> Self {
        Self {
            internal: Box::new(Int(n)),
            cache: RefCell::new(Cache::Invalid),
        }
    }

    pub fn placeholder() -> Self {
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

#[derive(Clone, Debug)]
struct Placeholder;

impl Approximation for Placeholder {
    fn approximate(&self, _p: Precision) -> BigInt {
        todo!("Placeholder instead of an actual Computable Real")
    }
}

#[derive(Clone, Debug)]
struct Int(BigInt);

impl Approximation for Int {
    fn approximate(&self, p: Precision) -> BigInt {
        scale(self.0.clone(), -p)
    }
}

#[derive(Clone, Debug)]
struct Pi;

impl Approximation for Pi {
    fn approximate(&self, p: Precision) -> BigInt {
        if p < -50 {
            todo!("Pi representation is not precise enough for this use");
        }
        let pi_64: BigInt = "3537118876014219".parse().unwrap();
        scale(pi_64, -50 - p)
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
#[derive(Clone, Debug)]
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
    }

    #[test]
    fn prec_atan_239() {
        let two_three_nine: BigInt = "239".parse().unwrap();
        let atan_239 = Computable::atan(two_three_nine);
        let four: BigInt = "4".parse().unwrap();
        assert_eq!(four, atan_239.approx(-10));
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
