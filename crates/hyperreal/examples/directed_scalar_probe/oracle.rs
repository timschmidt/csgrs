//! Fixed 1024-bit directed MPFR reference for the generated exact scalar recipe.
//!
//! This path avoids Hyperreal evaluation when computing bounds; Hyperreal rationals
//! only receive the final exact endpoints. It shares MPFR and recipe formulas with
//! the variable-precision benchmark adapter, so those paths are not independent
//! of each other. Bounds rely on directed rounding and the documented generated
//! domains; overlap with another interval is only a contradiction check.
//!
//! Precision controls rounding width, not a total-work, memory or cancellation
//! budget. Invalid generated invariants panic rather than producing typed domain
//! outcomes. This development adapter is not a general interval arithmetic API.

use super::recipe::{OffsetFamily, Recipe};
use hyperreal::{Rational, RealSign};
use rug::{
    Float, Rational as Mpq,
    float::{Constant, Round},
};
use std::cmp::Ordering;

/// Fixed significand width used for every reference operation; not a total-resource limit.
const PRECISION: u32 = 1024;

/// Closed outward-rounded endpoint pair for finite generated expressions.
/// Internal callers maintain ordered endpoints at one common precision; the struct
/// does not validate those invariants at construction.
struct Interval {
    /// Lower endpoint rounded toward negative infinity.
    lo: Float,
    /// Upper endpoint rounded toward positive infinity.
    hi: Float,
}

/// Directed arithmetic used by the finite recipe grammar; domain assumptions remain with callers.
impl Interval {
    /// Parse exact rational text and round separately down/up into an enclosing pair.
    /// Panics on malformed text. Both endpoints use the fixed 1024-bit significand width.
    fn rational(text: &str) -> Self {
        let value = text.parse::<Mpq>().expect("exact generated rational");
        Self {
            lo: Float::with_val_round(PRECISION, &value, Round::Down).0,
            hi: Float::with_val_round(PRECISION, &value, Round::Up).0,
        }
    }

    /// Enclose pi using MPFR's directed constant rounding at the fixed reference precision.
    fn pi() -> Self {
        Self {
            lo: Float::with_val_round(PRECISION, Constant::Pi, Round::Down).0,
            hi: Float::with_val_round(PRECISION, Constant::Pi, Round::Up).0,
        }
    }

    /// Subtract intervals using lo-other.hi and hi-other.lo with outward rounding.
    /// Assumes both inputs retain the fixed-precision ordered-endpoint invariant.
    fn sub(self, other: Self) -> Self {
        Self {
            lo: Float::with_val_round(PRECISION, &self.lo - &other.hi, Round::Down).0,
            hi: Float::with_val_round(PRECISION, &self.hi - &other.lo, Round::Up).0,
        }
    }

    /// Add corresponding endpoints with outward rounding.
    /// Assumes both inputs retain the fixed-precision ordered-endpoint invariant.
    fn add(self, other: Self) -> Self {
        Self {
            lo: Float::with_val_round(PRECISION, &self.lo + &other.lo, Round::Down).0,
            hi: Float::with_val_round(PRECISION, &self.hi + &other.hi, Round::Up).0,
        }
    }

    /// Bound a product of either sign by all four endpoint products: take the minimum
    /// of downward-rounded corners and maximum of upward-rounded corners. Correlation
    /// between operands is discarded, which may widen the enclosure. Assume valid finite
    /// inputs; NaN corner comparison panics.
    fn mul(self, other: Self) -> Self {
        let corners = [
            (&self.lo, &other.lo),
            (&self.lo, &other.hi),
            (&self.hi, &other.lo),
            (&self.hi, &other.hi),
        ];
        let lo = corners
            .iter()
            .map(|(a, b)| Float::with_val_round(PRECISION, *a * *b, Round::Down).0)
            .min_by(|a, b| a.partial_cmp(b).expect("finite interval corner"))
            .unwrap();
        let hi = corners
            .iter()
            .map(|(a, b)| Float::with_val_round(PRECISION, *a * *b, Round::Up).0)
            .max_by(|a, b| a.partial_cmp(b).expect("finite interval corner"))
            .unwrap();
        Self { lo, hi }
    }

    /// Require an ordered input interval strictly on one side of zero, then enclose
    /// its reciprocal as `[1/hi, 1/lo]` with outward rounding. Panic if the interval
    /// contains zero; no splitting or symbolic nonzero proof is attempted.
    fn inverse(self) -> Self {
        assert!(
            self.lo > 0 || self.hi < 0,
            "oracle denominator contains zero"
        );
        let one = Float::with_val(PRECISION, 1);
        Self {
            lo: Float::with_val_round(PRECISION, &one / &self.hi, Round::Down).0,
            hi: Float::with_val_round(PRECISION, &one / &self.lo, Round::Up).0,
        }
    }

    // All selected transcendental functions are increasing on their generated
    // domains. Trigonometric inputs are checked below against an independent
    // directed lower bound for pi/2 before this method is used.
    /// Apply a caller-selected increasing function down at lo and up at hi; discard
    /// its returned rounding-order indicators and assert finite ordered results.
    /// The caller must prove monotonicity and domain validity over the entire interval;
    /// these endpoint assertions alone do not establish either precondition.
    fn increasing(mut self, operation: fn(&mut Float, Round) -> Ordering) -> Self {
        operation(&mut self.lo, Round::Down);
        operation(&mut self.hi, Round::Up);
        assert!(self.lo.is_finite() && self.hi.is_finite() && self.lo <= self.hi);
        self
    }

    /// Require the whole interval strictly inside (0, pi/2), using a directed lower
    /// bound for pi/2, then apply the supplied increasing operation. Intended callbacks
    /// are sin and tan; an arbitrary function pointer is not validated for monotonicity.
    /// A boundary-touching or out-of-range enclosure panics even if its exact source
    /// value could be proved admissible with tighter bounds.
    fn positive_trig(self, operation: fn(&mut Float, Round) -> Ordering) -> Self {
        let half_pi = Self::pi().mul(Self::rational("1/2"));
        assert!(
            self.lo > 0 && self.hi < half_pi.lo,
            "oracle trig domain is not monotone"
        );
        self.increasing(operation)
    }

    /// Convert each finite endpoint to an exact GMP rational and parse its rational text
    /// into Hyperreal. No lossy float conversion is used. Nonfinite endpoints or an
    /// unexpected parser failure panic; endpoint order is inherited, not rechecked.
    fn rational_bounds(self) -> [Rational; 2] {
        [self.lo, self.hi].map(|x| {
            x.to_rational()
                .expect("finite MPFR endpoint")
                .to_string()
                .parse()
                .expect("exact rational MPFR endpoint")
        })
    }
}

/// Evaluate the selected recipe at 1024 significand bits, apply its signed rational
/// scale, and return exact Hyperreal rational endpoints. Unknown representation
/// names return a string error.
///
/// Assumes the fields and domains produced by Recipe::new: positive primes/exponents,
/// angle strictly between zero and one half, positive constant-offset inner forms,
/// and well-formed nonzero scale. Near-pi subtraction keeps the authored decimal
/// rational offset. Reciprocal, monotonic-function and endpoint-conversion guards
/// can panic for invalid manually changed fields. Repeated pi factors are evaluated
/// as written; this is not Hyperreal's symbolic normalization or a private-class check.
/// No total-work limit or cancellation hook is installed.
pub fn bounds(recipe: &Recipe) -> Result<[Rational; 2], String> {
    let rational = |n: u32, d: u32| Interval::rational(&format!("{n}/{d}"));
    let prime = || rational(recipe.prime, 1);
    let exponent = || rational(recipe.exponent_numerator, recipe.exponent_denominator);
    let angle = || rational(recipe.angle_numerator, recipe.angle_denominator);
    let pi_power = || (1..recipe.pi_power).fold(Interval::pi(), |x, _| x.mul(Interval::pi()));
    let exp = || exponent().increasing(Float::exp_round);
    let root = || prime().increasing(Float::sqrt_round);
    let log = || prime().increasing(Float::ln_round);
    let value = match recipe.input.bucket.as_str() {
        "One" => rational(1, 1),
        "Pi" => Interval::pi(),
        "PiPow" => pi_power(),
        "PiInv" => Interval::pi().inverse(),
        "PiExp" => Interval::pi().mul(exp()),
        "PiInvExp" => exp().mul(Interval::pi().inverse()),
        "PiSqrt" => Interval::pi().mul(root()),
        "ConstProduct" => pi_power().mul(exp()),
        "ConstOffset" => {
            let base = match recipe.offset_family {
                OffsetFamily::PiSubtract => Interval::pi(),
                OffsetFamily::ESubtract => rational(1, 1).increasing(Float::exp_round),
                OffsetFamily::ProductAdd => pi_power().mul(exp()),
                OffsetFamily::InverseProductAdd => exp().mul(Interval::pi().inverse()),
            };
            base.add(Interval::rational(&recipe.constant_offset))
        }
        "ConstProductSqrt" => pi_power().mul(exp()).mul(root()),
        "Sqrt" => root(),
        "Exp" => exp(),
        "Ln" => log(),
        "LnAffine" => prime().mul(exp()).increasing(Float::ln_round),
        "LnProduct" => log().mul(rational(recipe.other_prime, 1).increasing(Float::ln_round)),
        "Log10" => prime().increasing(Float::log10_round),
        "Log2" => rational(recipe.angle_denominator, 1).increasing(Float::log2_round),
        "Pow10" => angle().increasing(Float::exp10_round),
        "Pow2" => angle().increasing(Float::exp2_round),
        "SinPi" => angle().mul(Interval::pi()).positive_trig(Float::sin_round),
        "TanPi" => angle().mul(Interval::pi()).positive_trig(Float::tan_round),
        "Irrational" if recipe.generic_near_pi => {
            Interval::pi().sub(Interval::rational(&recipe.pi_offset))
        }
        "Irrational" => angle().positive_trig(Float::sin_round),
        other => return Err(format!("unsupported oracle representation: {other}")),
    };
    Ok(value
        .mul(Interval::rational(&recipe.scale))
        .rational_bounds())
}

/// Return Positive when the lower bound is positive, otherwise Negative when the
/// upper bound is negative; return an inconclusive string when zero is enclosed.
/// Assumes an ordered enclosure. Does not validate order or return a certified zero,
/// and a manually reversed pair can produce an unjustified sign.
pub fn sign(bounds: &[Rational; 2]) -> Result<RealSign, String> {
    if bounds[0] > Rational::zero() {
        Ok(RealSign::Positive)
    } else if bounds[1] < Rational::zero() {
        Ok(RealSign::Negative)
    } else {
        Err("independent oracle inconclusive at 1024 bits".into())
    }
}
