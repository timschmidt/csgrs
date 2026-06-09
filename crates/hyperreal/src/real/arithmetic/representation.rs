use std::sync::Arc;
use std::sync::atomic::AtomicBool;

pub type Signal = Arc<AtomicBool>;

/// (More) Real numbers
///
/// This type is functionally the product of a [`Computable`] number
/// and a [`Rational`].
///
/// Internally the rational scale is kept beside a lightweight symbolic
/// class certificate. Many hot operations inspect that certificate before
/// folding into the generic computable graph; do not eagerly combine the fields
/// unless a generic kernel really needs it.
///
/// # Examples
///
/// Even a normal rational can be parsed as a Real
/// ```
/// use hyperreal::{Real, Rational};
/// let half: Real = "0.5".parse().unwrap();
/// assert_eq!(half, Rational::fraction(1, 2).unwrap());
/// ```
///
/// Simple arithmetic
/// ```
/// use hyperreal::Real;
/// let two_pi = Real::pi() + Real::pi();
/// let four: Real = "4".parse().unwrap();
/// let four_pi = four * Real::pi();
/// let answer = (four_pi / two_pi).unwrap();
/// let two = hyperreal::Rational::new(2);
/// assert_eq!(answer, Real::new(two));
/// ```
///
/// Conversion
/// ```
/// use hyperreal::{Real, Rational};
/// let nine: Real = 9.into();
/// let three = Rational::new(3);
/// let answer = nine.sqrt().unwrap();
/// assert_eq!(answer, three);
/// ```
#[derive(Debug)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct Real {
    pub(super) rational: Rational,
    pub(super) class: Class,
    // Pure exact rationals do not need a computable payload. Leaving this empty
    // avoids allocating a fresh Computable::one() sentinel for every rational
    // scalar produced by dense algebra and matrix kernels; folding materializes
    // the rational leaf only when a generic approximation kernel actually needs it.
    pub(super) computable: Option<Computable>,
    #[cfg_attr(feature = "serde", serde(skip))]
    pub(super) signal: Option<Signal>,
    #[cfg_attr(feature = "serde", serde(skip, default))]
    pub(super) primitive_approx_cache: Cell<PrimitiveApproxCache>,
}

impl Clone for Real {
    fn clone(&self) -> Self {
        // `Computable` caches are accelerators, not semantic state. Most Real
        // clones in hyperlattice matrix kernels are cold exact symbols, so
        // cloning the full payload just to preserve an empty cache is wasted
        // work. Rebuild exact symbolic computables from the compact class
        // certificate; keep opaque irrational payloads and abort-attached values
        // as true clones because their graph shape or signal cannot be inferred.
        let computable =
            if self.signal.is_some() || matches!(self.class, Irrational | ConstOffset(_)) {
                // ConstOffset payloads are shallow enough to clone and expensive
                // enough to rebuild that scalar rows like 1000*pi+eps regress if
                // every clone reconstructs cached pi plus a rational offset tree.
                self.computable.clone()
            } else if matches!(self.class, One) {
                None
            } else {
                Some(self.class.computable_certificate())
            };

        Self {
            rational: self.rational.clone(),
            class: self.class.clone(),
            computable,
            signal: self.signal.clone(),
            primitive_approx_cache: Cell::new(self.primitive_approx_cache.get()),
        }
    }
}

impl Real {
    /// Refinement floor used by the `PartialOrd` implementation.
    ///
    /// The value is deliberately a bounded exact-real policy: comparisons never
    /// return an approximate result, but generic callers get substantially more
    /// than structural facts before `partial_cmp` reports `None`.
    pub const PARTIAL_CMP_MIN_PRECISION: i32 = -2048;

    fn exact_rational_unchecked(rational: Rational) -> Real {
        Self {
            rational,
            class: One,
            computable: None,
            signal: None,
            primitive_approx_cache: Cell::new(PrimitiveApproxCache::Empty),
        }
    }

    fn computable_ref(&self) -> &Computable {
        self.computable
            .as_ref()
            .expect("non-rational Real classes carry a computable payload")
    }

    pub(super) fn computable_clone(&self) -> Computable {
        self.computable
            .clone()
            .unwrap_or_else(|| self.class.computable_certificate())
    }

    pub(super) fn into_computable(self) -> Computable {
        self.computable
            .unwrap_or_else(|| self.class.computable_certificate())
    }

    /// Provide an atomic flag to signal early abort of calculations.
    /// The provided flag can be used e.g. from another execution thread.
    /// Aborted calculations may have incorrect results.
    pub fn abort(&mut self, s: Signal) {
        self.signal = Some(s.clone());
        self.primitive_approx_cache.set(PrimitiveApproxCache::Empty);
        if let Some(computable) = &mut self.computable {
            computable.abort(s);
        }
    }

    /// Zero, the additive identity.
    pub fn zero() -> Real {
        crate::trace_dispatch!("real", "constructor", "zero");
        Self::exact_rational_unchecked(Rational::zero())
    }

    /// One, the multiplicative identity.
    pub fn one() -> Real {
        crate::trace_dispatch!("real", "constructor", "one");
        Self::exact_rational_unchecked(Rational::one())
    }

    /// The specified [`Rational`] as a Real.
    pub fn new(rational: Rational) -> Real {
        crate::trace_dispatch!("real", "constructor", "rational");
        Self::exact_rational_unchecked(rational)
    }

    /// Returns the exact sum of owned Real values.
    pub fn sum_owned(values: impl IntoIterator<Item = Real>) -> Real {
        crate::trace_dispatch!("real", "aggregate", "sum-owned");
        values
            .into_iter()
            .fold(Self::zero(), |sum, value| sum + value)
    }

    /// Returns the exact sum of borrowed Real values.
    pub fn sum_refs<'a>(values: impl IntoIterator<Item = &'a Real>) -> Real {
        crate::trace_dispatch!("real", "aggregate", "sum-refs");
        values
            .into_iter()
            .fold(Self::zero(), |sum, value| sum + value)
    }

    /// Evaluates `origin + t * delta` without crossing a primitive-float boundary.
    pub fn affine(origin: &Real, t: &Real, delta: &Real) -> Real {
        crate::trace_dispatch!("real", "aggregate", "affine");
        origin + &(t * delta)
    }

    /// Returns the absolute value of this Real.
    ///
    /// Known signs are handled structurally. If the sign cannot be certified at
    /// a cheap bounded precision, the result is represented as `sqrt(x^2)` in
    /// the computable-real graph so the operation remains exact without making
    /// a discontinuous sign decision.
    pub fn abs(&self) -> Real {
        match self.certified_sign_until(0).sign() {
            Some(RealSign::Negative) => {
                crate::trace_dispatch!("real", "abs", "known-negative");
                -self
            }
            Some(RealSign::Positive) => {
                crate::trace_dispatch!("real", "abs", "known-positive");
                self.clone()
            }
            Some(RealSign::Zero) => {
                crate::trace_dispatch!("real", "abs", "known-zero");
                Self::zero()
            }
            None => {
                crate::trace_dispatch!("real", "abs", "sqrt-square-fallback");
                Self::irrational_from_computable(self.fold_ref().square().sqrt())
            }
        }
    }

    /// Converts degrees to radians exactly as `degrees * pi / 180`.
    pub fn to_radians(&self) -> Real {
        crate::trace_dispatch!("real", "angle-conversion", "to-radians");
        let factor =
            (&Self::pi() / &Self::from(180_u32)).expect("180 is a nonzero exact degree scale");
        self * &factor
    }

    /// Converts radians to degrees exactly as `radians * 180 / pi`.
    pub fn to_degrees(&self) -> Real {
        crate::trace_dispatch!("real", "angle-conversion", "to-degrees");
        let factor =
            (&Self::from(180_u32) / &Self::pi()).expect("pi is a nonzero exact radian scale");
        self * &factor
    }

    /// Returns the exact arithmetic mean of a non-empty Real slice.
    pub fn mean(values: &[Real]) -> Option<Real> {
        crate::trace_dispatch!("real", "aggregate", "mean");
        if values.is_empty() {
            return None;
        }
        let count = Real::from(u64::try_from(values.len()).ok()?);
        (Self::sum_refs(values.iter()) / count).ok()
    }

    /// Returns the exact sample standard deviation of a Real slice.
    pub fn sample_stddev(values: &[Real]) -> Option<Real> {
        crate::trace_dispatch!("real", "aggregate", "sample-stddev");
        if values.len() < 2 {
            return None;
        }
        let mean = Self::mean(values)?;
        let sum_squared = values.iter().fold(Self::zero(), |sum, value| {
            let delta = value - &mean;
            sum + delta.clone() * delta
        });
        let divisor = Real::from(u64::try_from(values.len() - 1).ok()?);
        (sum_squared / divisor).ok()?.sqrt().ok()
    }

    /// π, the ratio of a circle's circumference to its diameter.
    pub fn pi() -> Real {
        crate::trace_dispatch!("real", "constructor", "cached-pi");
        constants::pi()
    }

    /// τ, the ratio of a circle's circumference to its radius.
    pub fn tau() -> Real {
        crate::trace_dispatch!("real", "constructor", "cached-tau");
        constants::tau()
    }

    /// e, Euler's number and the base of the natural logarithm function.
    pub fn e() -> Real {
        crate::trace_dispatch!("real", "constructor", "cached-e");
        constants::e()
    }

    /// Returns the smaller value according to certified partial ordering.
    ///
    /// If refinement cannot decide the order at the standard partial-order
    /// precision, this keeps `self`, matching the conservative behavior of
    /// [`PartialOrd`] callers that treat incomparability as no improvement.
    pub fn min<'r>(&'r self, other: &'r Real) -> &'r Real {
        match self.partial_cmp(other) {
            Some(Ordering::Greater) => other,
            _ => self,
        }
    }

    /// Returns the larger value according to certified partial ordering.
    ///
    /// If refinement cannot decide the order at the standard partial-order
    /// precision, this keeps `self`, matching the conservative behavior of
    /// [`PartialOrd`] callers that treat incomparability as no improvement.
    pub fn max<'r>(&'r self, other: &'r Real) -> &'r Real {
        match self.partial_cmp(other) {
            Some(Ordering::Less) => other,
            _ => self,
        }
    }

    /// Returns true for every constructed hyperreal value.
    ///
    /// `Real` has no NaN or infinity inhabitants; failed primitive conversions
    /// are rejected before construction.
    pub const fn is_finite(&self) -> bool {
        true
    }
}

impl AsRef<Real> for Real {
    fn as_ref(&self) -> &Real {
        self
    }
}

// Tan(r) is a repeating shape
// returns whether to negate, and the (if necessary reflected) fraction
// 0 < r < 0.5
// Never actually used for exact zero or half
fn tan_curve(r: Rational) -> (bool, Rational) {
    let mut s = r.fract();
    let mut flip = false;
    if s.sign() == Sign::Minus {
        flip = true;
        s = s.neg();
    }
    if s > *rationals::HALF {
        (!flip, Rational::one() - s)
    } else {
        (flip, s)
    }
}

// Sin(r) is a single curve, then reflected, then both halves negated
// returns whether to negate, and the (if necessary reflected) fraction
// 0 < r < 0.5
// Never actually used for exact zero or half
pub(crate) fn curve(r: Rational) -> (bool, Rational) {
    // Reduce rational multiples of pi to the first half-turn and record the
    // sign separately. SinPi/TanPi classes depend on this canonical argument so
    // equivalent exact trig values compare cheaply.
    if r.sign() == Sign::Minus {
        let (neg, s) = curve(r.neg());
        return (!neg, s);
    }
    let whole = r.shifted_big_integer(0);
    let mut s = r.fract();
    if s > *rationals::HALF {
        s = Rational::one() - s;
    }
    (whole.bit(0), s)
}

fn sin_pi_neg(r: Rational) -> bool {
    // For exact sin(q*pi) table entries, the integer part alone determines the
    // sign after the rational argument has been normalized.
    if r.sign() == Sign::Minus {
        return !sin_pi_neg(r.neg());
    }
    r.shifted_big_integer(0).bit(0)
}

