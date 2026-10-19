//! Independent local Jacobi oracle, using the ODE in DLMF 22.13.1 (table).
//!
//! s'=c*d, c'=-s*d, d'=-m*s*c, with (s,c,d)(0)=(0,1,1).
//! MPFR interval arithmetic encloses coefficient and Horner rounding. If
//! A=max(1,|m|), induction on the convolution recurrence bounds every Taylor
//! coefficient by A^n. For q=A*(|x|+|y|)<1 the omitted tail after degree N
//! is at most q^(N+1)/(1-q). This is a local oracle, not analytic continuation;
//! it deliberately shares no rectangular, amplitude or elliptic-integral
//! algorithm with production. https://dlmf.nist.gov/22.13.T1

use hyperlattice::{Complex, Rational, Real};
use rug::{Float, float::Round};

const PRECISION: u32 = 768;
const DEGREE: usize = 512;

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

pub(crate) fn reference(argument: &Complex, parameter: &Real) -> [Interval; 6] {
    let x = Interval::native(&argument.re);
    let y = Interval::native(&argument.im);
    let m = Interval::native(parameter);
    let magnitude = m.magnitude();
    let a = if magnitude > 1 {
        magnitude
    } else {
        Float::with_val(PRECISION, 1)
    };
    let (mx, my) = (x.magnitude(), y.magnitude());
    let radius = Float::with_val_round(PRECISION, &mx + &my, Round::Up).0;
    let q = Float::with_val_round(PRECISION, &a * &radius, Round::Up).0;
    assert!(
        q <= Float::with_val(PRECISION, 0.75),
        "outside this local oracle's domain"
    );
    let mut coefficients = [
        vec![Interval::integer(0)],
        vec![Interval::integer(1)],
        vec![Interval::integer(1)],
    ];
    for n in 1..=DEGREE {
        let mut sums: [Interval; 3] = std::array::from_fn(|_| Interval::integer(0));
        for j in 0..n {
            let k = n - 1 - j;
            for (sum, (first, second)) in sums.iter_mut().zip([(1, 2), (0, 2), (0, 1)]) {
                *sum = sum.add(&coefficients[first][j].mul(&coefficients[second][k]));
            }
        }
        coefficients[0].push(sums[0].div_count(n));
        coefficients[1].push(sums[1].neg().div_count(n));
        coefficients[2].push(m.mul(&sums[2]).neg().div_count(n));
    }
    let mut power = Float::with_val(PRECISION, 1);
    for _ in 0..=DEGREE {
        power = Float::with_val_round(PRECISION, &power * &q, Round::Up).0;
    }
    let remaining = Float::with_val_round(PRECISION, 1 - &q, Round::Down).0;
    let tail = Float::with_val_round(PRECISION, &power / &remaining, Round::Up).0;
    let tail = Interval {
        lo: -tail.clone(),
        hi: tail,
    };
    let mut result: [Interval; 6] = std::array::from_fn(|_| Interval::integer(0));
    for (terms, pair) in coefficients.iter().zip(result.as_chunks_mut::<2>().0) {
        let mut re = Interval::integer(0);
        let mut im = Interval::integer(0);
        for coefficient in terms.iter().rev() {
            let next_re = re.mul(&x).add(&im.mul(&y).neg()).add(coefficient);
            im = re.mul(&y).add(&im.mul(&x));
            re = next_re;
        }
        pair[0] = re.add(&tail);
        pair[1] = im.add(&tail);
    }
    result
}

impl Interval {
    pub(crate) fn agrees_with(&self, actual: &[Rational; 2], bits: u32) -> bool {
        let native = Interval {
            lo: Interval::rational(&actual[0]).lo,
            hi: Interval::rational(&actual[1]).hi,
        };
        let tolerance = Float::with_val(PRECISION, 1) >> bits;
        self.lo <= self.hi
            && native.lo <= native.hi
            && Float::with_val_round(PRECISION, &self.hi - &self.lo, Round::Up).0 < tolerance
            && Float::with_val_round(PRECISION, &native.hi - &native.lo, Round::Up).0 < tolerance
            && native.lo <= self.hi
            && self.lo <= native.hi
    }
}

pub(crate) fn compare(argument: &Complex, parameter: &Real) {
    let (sn, cn, dn) = argument.clone().jacobi_sncndn(parameter).unwrap();
    for (reference, value) in reference(argument, parameter)
        .iter()
        .zip([sn.re, sn.im, cn.re, cn.im, dn.re, dn.im])
    {
        let actual = value.certified_rational_interval(-224).unwrap();
        assert!(
            reference.agrees_with(&actual, 200),
            "native [{}, {}], ODE [{}, {}]",
            actual[0],
            actual[1],
            reference.lo,
            reference.hi
        );
    }
}
