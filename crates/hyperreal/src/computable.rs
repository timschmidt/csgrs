use num_bigint::BigInt;
use num_traits::One;

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
    /* maybe provide some mechanism to request computation stops? */

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
    fn approximate(&self, _p: Precision) -> BigInt {
        todo!()
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
    fn zero() {
        assert_eq!(0, 0);
    }
}
