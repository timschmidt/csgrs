//! Independent-formula atanh comparison helper using MPFR rectangles and a truncated series.
//!
//! Uses the [local series](https://dlmf.nist.gov/4.38.E5) and
//! [reciprocal continuation](https://dlmf.nist.gov/4.38.E6), not production log/atan2.
//! Input bounds still come from native Real interval evaluation. The intended tail
//! bound is q^(2*DEGREE+3)/(1-q^2) for an upper L1 radius q<1, discarding the
//! positive odd-denominator factors conservatively.
//!
//! The implementation does not currently certify every such bound: Pair::radius
//! adds owned Floats with nearest rounding before the outer Round::Up conversion.
//! The tail denominator and final division likewise finish owned arithmetic before
//! the requested directed conversion. Those conversions cannot recover lost bits.
//! Width checks also use nearest-rounded differences. These existing limitations
//! must be resolved before treating the helper as a uniformly certified oracle;
//! passing comparisons remain finite overlap/width observations, not exact equality.

use hyperlattice::{Complex, Rational, Real};
use rug::{Float, float::Round};

/// MPFR significand precision (768 bits) for reference arithmetic; not a bound on native input-expression work.
const PRECISION: u32 = 768;
/// Largest series index n=384: 385 terms through z^769, with the first omitted power z^771.
const DEGREE: usize = 384;

/// Private pair of MPFR endpoints intended as a closed enclosure; construction does not validate ordering, finiteness, or correct outward rounding of every caller.
#[derive(Clone)]
pub(crate) struct Interval {
    /// Intended lower endpoint; not independently validated on construction.
    lo: Float,
    /// Intended upper endpoint; not independently validated on construction.
    hi: Float,
}

impl Interval {
    /// Create equal 768-bit endpoints for an i32, exactly representable at this precision.
    fn integer(value: i32) -> Self {
        Self {
            lo: Float::with_val(PRECISION, value),
            hi: Float::with_val(PRECISION, value),
        }
    }

    /// Convert unsigned numerator/positive denominator magnitudes with directed rounding, then restore the rational sign.
    ///
    /// Bound n/d by lower-n/upper-d and upper-n/lower-d. Rational owns reduction and
    /// nonzero-denominator invariants; parsing or conversion failures panic. Negative
    /// values negate and swap the positive-magnitude endpoints.
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

    /// Request native bounds at -850, unwrap success, and convert the lower/upper rational endpoints outward; this input conversion depends on the production scalar evaluator.
    fn native(value: &Real) -> Self {
        let [lo, hi] = value.certified_rational_interval(-850).unwrap();
        Self {
            lo: Self::rational(&lo).lo,
            hi: Self::rational(&hi).hi,
        }
    }

    /// Add corresponding borrowed endpoints with Down/Up rounding, preserving a sum enclosure for valid finite input intervals.
    fn add(&self, other: &Self) -> Self {
        Self {
            lo: Float::with_val_round(PRECISION, &self.lo + &other.lo, Round::Down).0,
            hi: Float::with_val_round(PRECISION, &self.hi + &other.hi, Round::Up).0,
        }
    }

    /// Negate and swap endpoints; exact sign reversal preserves enclosure ordering.
    fn neg(&self) -> Self {
        Self {
            lo: -self.hi.clone(),
            hi: -self.lo.clone(),
        }
    }

    /// Multiply all four borrowed endpoint pairs with directed rounding and take min/max.
    ///
    /// An exactly zero operand returns an exact zero interval before products. Assumes
    /// ordered finite endpoints; repeated operands retain no correlation, so this is
    /// interval multiplication rather than a specialized nonnegative interval square.
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

    /// Divide borrowed endpoints outward by count after a checked usize→u32 conversion; callers provide positive odd divisors, but zero is not locally rejected.
    fn div_count(&self, count: usize) -> Self {
        let count = u32::try_from(count).unwrap();
        Self {
            lo: Float::with_val_round(PRECISION, &self.lo / count, Round::Down).0,
            hi: Float::with_val_round(PRECISION, &self.hi / count, Round::Up).0,
        }
    }

    /// Return max(abs(lo),abs(hi)); for a valid interval this bounds absolute value without further arithmetic rounding.
    fn magnitude(&self) -> Float {
        let lo = self.lo.clone().abs();
        let hi = self.hi.clone().abs();
        if lo > hi { lo } else { hi }
    }
}

impl Interval {
    /// Assert a strictly positive lower denominator bound, form its directed reciprocal interval, then multiply; intervals crossing zero are rejected by assertion.
    fn div_positive(&self, other: &Self) -> Self {
        assert!(other.lo > 0);
        self.mul(&Self {
            lo: Float::with_val_round(PRECISION, 1 / &other.hi, Round::Down).0,
            hi: Float::with_val_round(PRECISION, 1 / &other.lo, Round::Up).0,
        })
    }

    /// Enclose MPFR pi using Down/Up, then divide endpoints by exactly representable two; this binary scaling is exact for these finite values.
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

    /// Widen both endpoints outward by the supplied tail; correctness requires a nonnegative valid tail bound, which this method does not verify.
    fn with_tail(mut self, tail: &Float) -> Self {
        self.lo = Float::with_val_round(PRECISION, &self.lo - tail, Round::Down).0;
        self.hi = Float::with_val_round(PRECISION, &self.hi + tail, Round::Up).0;
        self
    }
}

/// Real/imaginary MPFR rectangles for the independent series; no branch or correlation evidence is stored.
#[derive(Clone)]
struct Pair(
    #[doc = "Real-component interval."] Interval,
    #[doc = "Imaginary-component interval."] Interval,
);

impl Pair {
    /// Multiply complex rectangles with the standard four-product formula; interval dependency can widen results, and no production Complex arithmetic is used.
    fn mul(&self, other: &Self) -> Self {
        Self(
            self.0.mul(&other.0).add(&self.1.mul(&other.1).neg()),
            self.0.mul(&other.1).add(&self.1.mul(&other.0)),
        )
    }

    /// Compute max-absolute-real plus max-absolute-imaginary at the working precision.
    ///
    /// The mathematical sum bounds modulus, but the owned Float addition rounds to
    /// nearest before with_val_round(..., Up). Consequently this implementation is not
    /// always an upper bound: components 1 and 2^-800 yield 1 at 768 bits. Used for
    /// domain selection and the tail; the outer conversion does not repair that loss.
    fn radius(&self) -> Float {
        Float::with_val_round(
            PRECISION,
            self.0.magnitude() + self.1.magnitude(),
            Round::Up,
        )
        .0
    }

    /// Form conjugate divided by the interval sum of component squares; requires that computed denominator have a positive lower bound, not merely that the exact argument is nonzero.
    fn reciprocal(&self) -> Self {
        let norm = self.0.mul(&self.0).add(&self.1.mul(&self.1));
        Self(self.0.div_positive(&norm), self.1.neg().div_positive(&norm))
    }
}

/// Compute local-series or reciprocal-series rectangles for a restricted input domain.
///
/// If the original computed L1 radius exceeds 3/4, invert the rectangle. Assert the
/// selected computed radius is <=3/4; many otherwise valid atanh inputs fail this
/// oracle-domain assertion. Sum 385 odd-power terms, then widen by the intended
/// geometric tail. The current radius and owned denominator/division rounding gaps
/// prevent a blanket certified-tail claim; see the module contract.
///
/// Reciprocal continuation adds +i*pi/2 for a strictly positive original imaginary
/// interval, or -i*pi/2 for a strictly negative one; an unresolved/zero interval
/// panics. Thus real exterior-cut points are not assigned a bank. Returns two
/// rectangles, not an exact result or typed failure. Native input refinement,
/// allocation, and MPFR work have no separate total-resource limit.
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
    /// Test endpoint overlap and two rounded widths strictly below 2^-bits.
    ///
    /// Convert the native rational endpoints outward, then compare the two intervals.
    /// Widths use nearest rounding rather than upward rounding; they are not certified
    /// upper widths. Assumes ordered intervals and does not repair reference rounding
    /// gaps. Overlap and small widths alone do not establish exact equality.
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

/// Build the reference and production result, then require narrow overlapping component intervals.
///
/// Production bounds use -240; agrees_with uses width threshold 2^-200. Errors and
/// missing bounds unwrap-panic, and failed comparison asserts. The reference formula
/// is independent, but input bounds and production result share scalar machinery;
/// the documented oracle rounding gaps remain part of this test's evidence limits.
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
