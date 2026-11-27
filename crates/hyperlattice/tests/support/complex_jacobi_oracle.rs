//! Independent local Taylor-ODE oracle for Jacobi triplets and epsilon.
//!
//! The [Jacobi differential system](https://dlmf.nist.gov/22.13.T1) is
//! s'=c*d, c'=-s*d, d'=-m*s*c with (s,c,d)(0)=(0,1,1); epsilon'=d² starts at zero.
//! For A=max(1,|m|), induction bounds degree-n coefficients by A^n: each
//! convolution has n terms bounded by A^(n-1), division by n cancels their count,
//! and only d' multiplies by m. Epsilon's coefficient is bounded by A^(n-1)<=A^n.
//! Thus q=A*(|x|+|y|)<1 gives tail q^(N+1)/(1-q) after degree N.
//!
//! Directed MPFR endpoint arithmetic encloses coefficients, Horner evaluation and
//! the tail for valid finite input intervals in the asserted local domain. Native
//! Real interval evaluation supplies the inputs; independence concerns the ODE
//! formula, not input evaluation. There is no rectangular/amplitude algorithm,
//! analytic continuation, or production total-work guarantee in this helper.

use hyperlattice::{Complex, Rational, Real};
use rug::{Float, float::Round};

/// MPFR significand precision of 768 bits for endpoints; this does not limit native input-expression work.
const PRECISION: u32 = 768;
/// Largest retained Taylor degree, 512; coefficients include degree zero and the tail starts at power 513.
const DEGREE: usize = 512;

/// Private closed MPFR endpoint pair; methods assume ordered finite endpoints, and construction performs no general invariant validation.
#[derive(Clone)]
pub(crate) struct Interval {
    /// Lower endpoint, assumed finite and no greater than hi.
    lo: Float,
    /// Upper endpoint, assumed finite and no less than lo.
    hi: Float,
}

impl Interval {
    /// Represent an i32 with equal exactly representable endpoints at the working precision.
    fn integer(value: i32) -> Self {
        Self {
            lo: Float::with_val(PRECISION, value),
            hi: Float::with_val(PRECISION, value),
        }
    }

    /// Enclose the signed rational using directed conversion of unsigned numerator and positive denominator magnitudes.
    ///
    /// Lower-n/upper-d and upper-n/lower-d bound the positive quotient; negative
    /// values negate and swap the endpoints. Parsing/conversion failures panic.
    /// The Rational owner supplies the nonzero-denominator invariant.
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

    /// Unwrap a native rational enclosure at -850 and convert its lower/upper endpoints outward; this shares the production scalar evaluator.
    fn native(value: &Real) -> Self {
        let [lo, hi] = value.certified_rational_interval(-850).unwrap();
        Self {
            lo: Self::rational(&lo).lo,
            hi: Self::rational(&hi).hi,
        }
    }

    /// Add corresponding borrowed endpoints with Down/Up rounding, enclosing the sum of valid finite intervals.
    fn add(&self, other: &Self) -> Self {
        Self {
            lo: Float::with_val_round(PRECISION, &self.lo + &other.lo, Round::Down).0,
            hi: Float::with_val_round(PRECISION, &self.hi + &other.hi, Round::Up).0,
        }
    }

    /// Negate and swap endpoints exactly, reversing their order without magnitude rounding.
    fn neg(&self) -> Self {
        Self {
            lo: -self.hi.clone(),
            hi: -self.lo.clone(),
        }
    }

    /// Enclose all four endpoint products with directed rounding and select the lower minimum and upper maximum.
    ///
    /// An exact zero operand short-circuits products. Assumes valid finite intervals;
    /// repeated operands retain no correlation and are not a specialized interval square.
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

    /// Divide endpoints outward by a checked usize→u32 count; recurrence callers supply 1..=DEGREE, while this helper does not itself reject zero.
    fn div_count(&self, count: usize) -> Self {
        let count = u32::try_from(count).unwrap();
        Self {
            lo: Float::with_val_round(PRECISION, &self.lo / count, Round::Down).0,
            hi: Float::with_val_round(PRECISION, &self.hi / count, Round::Up).0,
        }
    }

    /// Return max(abs(lo),abs(hi)); for valid finite endpoints this bounds magnitude without further arithmetic rounding.
    fn magnitude(&self) -> Float {
        let lo = self.lo.clone().abs();
        let hi = self.hi.clone().abs();
        if lo > hi { lo } else { hi }
    }
}

/// Return six intervals in sn.re, sn.im, cn.re, cn.im, dn.re, dn.im order, or two epsilon intervals when requested.
///
/// Compute outward q=A*(maxabs(x)+maxabs(y)) and assert q<=3/4. The reference
/// requires this local domain, not m<1; compare additionally needs production's
/// parameter domain. Build all three triplet coefficient arrays through DEGREE,
/// optionally integrate the dn² convolution, and evaluate each chosen polynomial
/// by complex interval Horner. Add the symmetric geometric tail to both components.
///
/// Borrowed arithmetic retains directed rounding for radius, tail denominator,
/// quotient and endpoint widths. Coefficients cost O(DEGREE²) interval operations;
/// retained coefficient arrays cost O(DEGREE) endpoints, separate from native
/// refinement and MPFR storage/work. Invalid domain asserts and unavailable input
/// bounds unwrap-panic; no typed exhaustion or explicit cancellation is added.
pub(crate) fn reference(argument: &Complex, parameter: &Real, epsilon: bool) -> Vec<Interval> {
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
    let mut epsilon_coefficients = vec![Interval::integer(0)];
    for n in 1..=DEGREE {
        let mut sums: [Interval; 3] = std::array::from_fn(|_| Interval::integer(0));
        let mut epsilon_sum = Interval::integer(0);
        for j in 0..n {
            let k = n - 1 - j;
            for (sum, (first, second)) in sums.iter_mut().zip([(1, 2), (0, 2), (0, 1)]) {
                *sum = sum.add(&coefficients[first][j].mul(&coefficients[second][k]));
            }
            if epsilon {
                epsilon_sum = epsilon_sum.add(&coefficients[2][j].mul(&coefficients[2][k]));
            }
        }
        coefficients[0].push(sums[0].div_count(n));
        coefficients[1].push(sums[1].neg().div_count(n));
        coefficients[2].push(m.mul(&sums[2]).neg().div_count(n));
        if epsilon {
            epsilon_coefficients.push(epsilon_sum.div_count(n));
        }
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
    let terms = if epsilon {
        std::slice::from_ref(&epsilon_coefficients)
    } else {
        &coefficients
    };
    let mut result = vec![Interval::integer(0); terms.len() * 2];
    for (terms, pair) in terms.iter().zip(result.as_chunks_mut::<2>().0) {
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
    /// Require ordered reference/native intervals, upward-rounded widths below 2^-bits, and endpoint overlap.
    ///
    /// Native rational endpoints are converted outward. The dyadic threshold is at
    /// MPFR precision; callers use ordinary bit counts within its exponent range.
    /// Passing establishes narrow overlap, not exact equality of represented values.
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

/// Construct the production output first, then the independent reference, and compare every component.
///
/// Production bounds use -224 and agrees_with requires widths below 2^-200.
/// The length assertion prevents silent zip truncation. Missing bounds, native
/// errors or failed comparisons panic. Inputs share native refinement with the
/// reference; the independent evidence is the local ODE construction.
pub(crate) fn compare(argument: &Complex, parameter: &Real, epsilon: bool) {
    let components = if epsilon {
        let value = argument.clone().jacobi_epsilon(parameter).unwrap();
        vec![value.re, value.im]
    } else {
        let (sn, cn, dn) = argument.clone().jacobi_sncndn(parameter).unwrap();
        vec![sn.re, sn.im, cn.re, cn.im, dn.re, dn.im]
    };
    let expected = reference(argument, parameter, epsilon);
    assert_eq!(expected.len(), components.len());
    for (reference, value) in expected.iter().zip(components) {
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
