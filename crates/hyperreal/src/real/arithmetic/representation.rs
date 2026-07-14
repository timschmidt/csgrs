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
    #[cfg_attr(feature = "serde", serde(skip, default))]
    pub(super) primitive_approx_cache: AtomicPrimitiveApproxCache,
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
            if self.abort_signal().is_some() || matches!(self.class, Irrational | ConstOffset(_)) {
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
            primitive_approx_cache: AtomicPrimitiveApproxCache::new(
                self.primitive_approx_cache.get(),
            ),
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
    const CERTIFIED_INTEGER_COMPARE_TOLERANCE: i32 = -1024;

    fn exact_rational_unchecked(rational: Rational) -> Real {
        Self {
            rational,
            class: One,
            computable: None,
            primitive_approx_cache: AtomicPrimitiveApproxCache::new(PrimitiveApproxCache::Empty),
        }
    }

    pub(super) fn same_symbolic_basis(&self, other: &Self) -> bool {
        match (&self.class, &other.class) {
            (Irrational, Irrational) => match (&self.computable, &other.computable) {
                (Some(left), Some(right)) => Computable::internal_structural_eq(left, right),
                _ => false,
            },
            _ => self.class == other.class,
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
        self.primitive_approx_cache.set(PrimitiveApproxCache::Empty);
        let computable = self
            .computable
            .get_or_insert_with(|| self.class.computable_certificate());
        computable.abort(s);
    }

    pub(super) fn abort_signal(&self) -> Option<&Signal> {
        self.computable
            .as_ref()
            .and_then(|computable| computable.signal.as_ref())
    }

    #[cfg(any(feature = "cached-f32-approx", feature = "cached-f64-approx"))]
    pub(super) fn is_aborted(&self) -> bool {
        use std::sync::atomic::Ordering::Relaxed;

        self.abort_signal()
            .is_some_and(|signal| signal.load(Relaxed))
    }

    pub(super) fn inherit_abort(&self, mut computable: Computable) -> Computable {
        if let Some(signal) = self.abort_signal() {
            computable.abort(signal.clone());
        }
        computable
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

    /// The specified integer as a Real.
    pub fn integer(integer: BigInt) -> Real {
        crate::trace_dispatch!("real", "constructor", "bigint");
        Self::exact_rational_unchecked(
            Rational::from_bigint_fraction(integer, BigUint::from(1_u8))
                .expect("integer denominator is nonzero"),
        )
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

    fn exact_rational_floor(rational: &Rational) -> BigInt {
        let mut value = rational
            .trunc()
            .to_big_integer()
            .expect("truncated rational is an integer");
        if rational.sign() == Sign::Minus && !rational.fract().is_zero() {
            value -= 1;
        }
        value
    }

    fn exact_rational_ceil(rational: &Rational) -> BigInt {
        let mut value = rational
            .trunc()
            .to_big_integer()
            .expect("truncated rational is an integer");
        if rational.sign() == Sign::Plus && !rational.fract().is_zero() {
            value += 1;
        }
        value
    }

    fn certified_floor_candidate(&self, candidate: &BigInt) -> Option<BigInt> {
        let lower = Self::integer(candidate.clone());
        let upper = Self::integer(candidate + BigInt::from(1_u8));

        let lower_ok = matches!(
            self.certified_cmp_until(&lower, Self::CERTIFIED_INTEGER_COMPARE_TOLERANCE),
            CertifiedRealOrdering::Known {
                ordering: Ordering::Equal | Ordering::Greater,
                ..
            }
        );
        if !lower_ok {
            return None;
        }

        let upper_ok = matches!(
            self.certified_cmp_until(&upper, Self::CERTIFIED_INTEGER_COMPARE_TOLERANCE),
            CertifiedRealOrdering::Known {
                ordering: Ordering::Less,
                ..
            }
        );
        upper_ok.then(|| candidate.clone())
    }

    /// Certified floor as an integer.
    ///
    /// Returns [`Problem::Exhausted`] when bounded exact-real refinement cannot
    /// certify the integer boundary.
    pub fn floor_certified(&self) -> Result<BigInt, Problem> {
        if let Some(rational) = self.exact_rational_ref() {
            crate::trace_dispatch!("real", "integer-rounding", "floor-exact-rational");
            return Ok(Self::exact_rational_floor(rational));
        }

        crate::trace_dispatch!("real", "integer-rounding", "floor-certified");
        let estimate = self.fold_ref().approx(0);
        for offset in -4_i8..=4 {
            let candidate = &estimate + BigInt::from(offset);
            if let Some(floor) = self.certified_floor_candidate(&candidate) {
                return Ok(floor);
            }
        }
        Err(Problem::Exhausted)
    }

    /// Certified ceiling as an integer.
    pub fn ceil_certified(&self) -> Result<BigInt, Problem> {
        if let Some(rational) = self.exact_rational_ref() {
            crate::trace_dispatch!("real", "integer-rounding", "ceil-exact-rational");
            return Ok(Self::exact_rational_ceil(rational));
        }

        crate::trace_dispatch!("real", "integer-rounding", "ceil-certified");
        let floor = self.floor_certified()?;
        let floor_real = Self::integer(floor.clone());
        match self.certified_eq_until(&floor_real, Self::CERTIFIED_INTEGER_COMPARE_TOLERANCE) {
            CertifiedRealEquality::Equal { .. } => Ok(floor),
            CertifiedRealEquality::NotEqual { .. } => Ok(floor + 1),
            CertifiedRealEquality::Unknown { .. } => Err(Problem::Exhausted),
        }
    }

    /// Certified truncation toward zero as an integer.
    pub fn trunc_certified(&self) -> Result<BigInt, Problem> {
        if let Some(rational) = self.exact_rational_ref() {
            crate::trace_dispatch!("real", "integer-rounding", "trunc-exact-rational");
            return Ok(rational
                .trunc()
                .to_big_integer()
                .expect("truncated rational is an integer"));
        }

        crate::trace_dispatch!("real", "integer-rounding", "trunc-certified");
        match self.certified_sign_until(Self::CERTIFIED_INTEGER_COMPARE_TOLERANCE) {
            CertifiedRealSign::Known {
                sign: RealSign::Negative,
                ..
            } => self.ceil_certified(),
            CertifiedRealSign::Known {
                sign: RealSign::Zero,
                ..
            } => Ok(BigInt::from(0_u8)),
            CertifiedRealSign::Known {
                sign: RealSign::Positive,
                ..
            } => self.floor_certified(),
            CertifiedRealSign::Unknown { .. } => Err(Problem::Exhausted),
        }
    }

    /// Certified nearest integer, with ties rounded away from zero.
    pub fn round_certified(&self) -> Result<BigInt, Problem> {
        crate::trace_dispatch!("real", "integer-rounding", "round-certified");
        let half = Self::new(Rational::fraction(1, 2).expect("2 is nonzero"));
        match self.certified_sign_until(Self::CERTIFIED_INTEGER_COMPARE_TOLERANCE) {
            CertifiedRealSign::Known {
                sign: RealSign::Negative,
                ..
            } => (self - &half).ceil_certified(),
            CertifiedRealSign::Known {
                sign: RealSign::Zero,
                ..
            } => Ok(BigInt::from(0_u8)),
            CertifiedRealSign::Known {
                sign: RealSign::Positive,
                ..
            } => (self + &half).floor_certified(),
            CertifiedRealSign::Unknown { .. } => Err(Problem::Exhausted),
        }
    }

    /// Certified fractional part after truncation toward zero.
    pub fn fract_certified(&self) -> Result<Real, Problem> {
        crate::trace_dispatch!("real", "integer-rounding", "fract-certified");
        let trunc = Self::integer(self.trunc_certified()?);
        Ok(self - &trunc)
    }

    /// Certified Euclidean remainder for positive modulus.
    pub fn rem_euclid_certified(&self, modulus: &Real) -> Result<Real, Problem> {
        crate::trace_dispatch!("real", "integer-rounding", "rem-euclid-certified");
        match modulus.certified_cmp_until(
            &Self::zero(),
            Self::CERTIFIED_INTEGER_COMPARE_TOLERANCE,
        ) {
            CertifiedRealOrdering::Known {
                ordering: Ordering::Greater,
                ..
            } => {}
            CertifiedRealOrdering::Known { .. } => return Err(Problem::NotANumber),
            CertifiedRealOrdering::Unknown { .. } => return Err(Problem::Exhausted),
        }

        let quotient = (self / modulus)?;
        let quotient_floor = Self::integer(quotient.floor_certified()?);
        Ok(self - &(&quotient_floor * modulus))
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
