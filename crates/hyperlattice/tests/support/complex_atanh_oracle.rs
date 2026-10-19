//! Independent principal atanh reference: its local power series with an
//! explicit geometric tail, plus reciprocal continuation for large nonreal z.
//! MPFR interval arithmetic encloses input conversion and every operation.
//! No logarithm or atan2 formula is used by this oracle.
use hyperlattice::{Complex, Rational, Real};
use rug::{Float, float::Round};

const PRECISION: u32 = 768;
const DEGREE: usize = 384;

#[derive(Clone)]
pub(crate) struct Interval {
    lo: Float,
    hi: Float,
}

impl Interval {
    fn integer(value: i32) -> Self {
        Self {
            lo: Float::with_val(PRECISION, value),
            hi: Float::with_val(PRECISION, value),
        }
    }

    fn rational(value: &Rational) -> Self {
        let integer = |text: String, round| {
            Float::with_val_round(PRECISION, Float::parse(text).unwrap(), round).0
        };
        let lo_n = integer(value.numerator().to_string(), Round::Down);
        let hi_n = integer(value.numerator().to_string(), Round::Up);
        let lo_d = integer(value.denominator().to_string(), Round::Down);
        let hi_d = integer(value.denominator().to_string(), Round::Up);
        let result = Self {
            lo: Float::with_val_round(PRECISION, &lo_n / &hi_d, Round::Down).0,
            hi: Float::with_val_round(PRECISION, &hi_n / &lo_d, Round::Up).0,
        };
        if value < &Rational::zero() {
            result.neg()
        } else {
            result
        }
    }

    fn native(value: &Real) -> Self {
        let [lo, hi] = value.certified_rational_interval(-850).unwrap();
        Self {
            lo: Self::rational(&lo).lo,
            hi: Self::rational(&hi).hi,
        }
    }

    fn add(&self, other: &Self) -> Self {
        Self {
            lo: Float::with_val_round(PRECISION, &self.lo + &other.lo, Round::Down).0,
            hi: Float::with_val_round(PRECISION, &self.hi + &other.hi, Round::Up).0,
        }
    }

    fn neg(&self) -> Self {
        Self {
            lo: -self.hi.clone(),
            hi: -self.lo.clone(),
        }
    }

    fn mul(&self, other: &Self) -> Self {
        if (self.lo == 0 && self.hi == 0) || (other.lo == 0 && other.hi == 0) {
            return Self::integer(0);
        }
        let pairs = [
            (&self.lo, &other.lo),
            (&self.lo, &other.hi),
            (&self.hi, &other.lo),
            (&self.hi, &other.hi),
        ];
        let lo = pairs
            .iter()
            .map(|(a, b)| Float::with_val_round(PRECISION, *a * *b, Round::Down).0)
            .reduce(|a, b| if a < b { a } else { b })
            .unwrap();
        let hi = pairs
            .iter()
            .map(|(a, b)| Float::with_val_round(PRECISION, *a * *b, Round::Up).0)
            .reduce(|a, b| if a > b { a } else { b })
            .unwrap();
        Self { lo, hi }
    }

    fn div_count(&self, count: usize) -> Self {
        let count = u32::try_from(count).unwrap();
        Self {
            lo: Float::with_val_round(PRECISION, &self.lo / count, Round::Down).0,
            hi: Float::with_val_round(PRECISION, &self.hi / count, Round::Up).0,
        }
    }

    fn magnitude(&self) -> Float {
        let lo = self.lo.clone().abs();
        let hi = self.hi.clone().abs();
        if lo > hi { lo } else { hi }
    }
}

impl Interval {
    fn div_positive(&self, other: &Self) -> Self {
        assert!(other.lo > 0);
        self.mul(&Self {
            lo: Float::with_val_round(PRECISION, 1 / &other.hi, Round::Down).0,
            hi: Float::with_val_round(PRECISION, 1 / &other.lo, Round::Up).0,
        })
    }

    fn half_pi() -> Self {
        let (lo, hi) = (
            Float::with_val_round(PRECISION, rug::float::Constant::Pi, Round::Down).0,
            Float::with_val_round(PRECISION, rug::float::Constant::Pi, Round::Up).0,
        );
        Self {
            lo: lo / 2,
            hi: hi / 2,
        }
    }

    fn with_tail(mut self, tail: &Float) -> Self {
        self.lo = Float::with_val_round(PRECISION, &self.lo - tail, Round::Down).0;
        self.hi = Float::with_val_round(PRECISION, &self.hi + tail, Round::Up).0;
        self
    }
}

#[derive(Clone)]
struct Pair(Interval, Interval);

impl Pair {
    fn mul(&self, other: &Self) -> Self {
        Self(
            self.0.mul(&other.0).add(&self.1.mul(&other.1).neg()),
            self.0.mul(&other.1).add(&self.1.mul(&other.0)),
        )
    }

    fn radius(&self) -> Float {
        Float::with_val_round(
            PRECISION,
            self.0.magnitude() + self.1.magnitude(),
            Round::Up,
        )
        .0
    }

    fn reciprocal(&self) -> Self {
        let norm = self.0.mul(&self.0).add(&self.1.mul(&self.1));
        Self(self.0.div_positive(&norm), self.1.neg().div_positive(&norm))
    }
}

pub(crate) fn reference(argument: &Complex) -> [Interval; 2] {
    let original = Pair(
        Interval::native(&argument.re),
        Interval::native(&argument.im),
    );
    let large = original.radius() > Float::with_val(PRECISION, 0.75);
    let z = if large {
        original.reciprocal()
    } else {
        original.clone()
    };
    let q = z.radius();
    assert!(
        q <= Float::with_val(PRECISION, 0.75),
        "outside local or reciprocal oracle domain"
    );
    let z2 = z.mul(&z);
    let mut power = z.clone();
    let mut sum = Pair(Interval::integer(0), Interval::integer(0));
    for n in 0..=DEGREE {
        sum.0 = sum.0.add(&power.0.div_count(2 * n + 1));
        sum.1 = sum.1.add(&power.1.div_count(2 * n + 1));
        power = power.mul(&z2);
    }
    let mut tail = Float::with_val(PRECISION, 1);
    for _ in 0..2 * DEGREE + 3 {
        tail = Float::with_val_round(PRECISION, &tail * &q, Round::Up).0;
    }
    let q2 = Float::with_val_round(PRECISION, &q * &q, Round::Up).0;
    let denominator = Float::with_val_round(PRECISION, 1 - q2, Round::Down).0;
    let tail = Float::with_val_round(PRECISION, tail / denominator, Round::Up).0;
    sum.0 = sum.0.with_tail(&tail);
    sum.1 = sum.1.with_tail(&tail);
    if large {
        let correction = if original.1.lo > 0 {
            Interval::half_pi()
        } else {
            assert!(
                original.1.hi < 0,
                "reciprocal continuation needs nonreal input"
            );
            Interval::half_pi().neg()
        };
        sum.1 = sum.1.add(&correction);
    }
    [sum.0, sum.1]
}

impl Interval {
    pub(crate) fn agrees_with(&self, native: &[Rational; 2], bits: u32) -> bool {
        let lo = Self::rational(&native[0]).lo;
        let hi = Self::rational(&native[1]).hi;
        let width = Float::with_val(PRECISION, 1) >> bits;
        lo <= self.hi
            && self.lo <= hi
            && Float::with_val(PRECISION, hi - lo) < width
            && Float::with_val(PRECISION, &self.hi - &self.lo) < width
    }
}

pub(crate) fn compare(argument: &Complex) {
    let expected = reference(argument);
    let actual = argument.clone().atanh().unwrap();
    for (value, expected) in [actual.re, actual.im].into_iter().zip(expected) {
        let native = value.certified_rational_interval(-240).unwrap();
        assert!(
            expected.agrees_with(&native, 200),
            "disjoint or wide native/reference enclosures"
        );
    }
}
