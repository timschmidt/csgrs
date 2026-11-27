use std::borrow::Cow;
use std::sync::Arc;
use std::sync::atomic::AtomicBool;

/// Shared cooperative-abort flag: setting it requests interruption, not a typed failure result.
/// Clones share the flag; attaching one does not itself set it or impose a time bound.
pub type Signal = Arc<AtomicBool>;

/// An exact rational scale paired with a named symbolic basis or an opaque computable definition.
///
/// This type is functionally the product of a [`Computable`] number
/// and a [`Rational`].
///
/// Internally the rational scale is kept beside a lightweight symbolic
/// class certificate. Many hot operations inspect that certificate before
/// folding into the generic computable graph; do not eagerly combine the fields
/// unless a generic kernel really needs it.
///
/// A missing payload is valid for a named certificate, which can reconstruct its
/// basis; an opaque class must retain its definition. The payload represents the
/// basis, not the already-scaled value. Primitive float caches are disposable
/// projections, never replacements for this definition. Clone may materialize a
/// lazy named payload while retaining value semantics; allocation identity and
/// identical representation bytes are not the Clone contract.
///
/// With serde, input definitions are checked through the private wire carrier;
/// output omits live caches and abort signals. Proof precision limits do not
/// provide a total-work or memory budget. Aborted approximations follow the
/// documented limitations of [`Real::abort`].
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
#[cfg_attr(feature = "serde", derive(Serialize))]
pub struct Real {
    /// Exact multiplicative scale; kept separate so native rational/symbolic operations can reuse it.
    pub(super) rational: Rational,
    /// Basis certificate used for structural decisions and lazy payload reconstruction.
    pub(super) class: Class,
    // Pure exact rationals do not need a computable payload. Leaving this empty
    // avoids allocating a fresh Computable::one() sentinel for every rational
    // scalar produced by dense algebra and matrix kernels; folding materializes
    // the rational leaf only when a generic approximation kernel actually needs it.
    /// Optional basis graph and attached signal; required for opaque definitions, lazy for named ones.
    pub(super) computable: Option<Computable>,
    /// Synchronized disposable float view, excluded from exact meaning and serialized form.
    #[cfg_attr(feature = "serde", serde(skip, default))]
    pub(super) primitive_approx_cache: AtomicPrimitiveApproxCache,
}

/// Untrusted wire fields, decoded before the Real invariant is admitted.
/// Named certificates allow an omitted/null payload; an opaque class does not.
/// No live primitive cache or abort signal is accepted from serialized input.
#[cfg(feature = "serde")]
#[derive(Deserialize)]
#[serde(rename = "Real")]
pub(crate) struct RealWire {
    /// Exact outer scale, already decoded by Rational's own validator.
    rational: Rational,
    /// Proposed basis certificate whose parameters still require validation.
    class: Class,
    /// Optional proposed basis graph; partial-operation domains and named-class agreement remain unproved.
    computable: Option<Computable>,
}

/// Admission failure category plus static context for direct Serde diagnostics.
/// Convenience JSON/CBOR methods keep problem and discard detail.
#[cfg(feature = "serde")]
pub(crate) struct RealDecodeError {
    /// Typed scalar cause, distinguishing invalid definitions from unresolved proof obligations.
    pub(crate) problem: Problem,
    /// Fixed description of the failed admission stage, without embedding source payload text.
    detail: &'static str,
}

#[cfg(feature = "serde")]
/// Format both the scalar cause and the admission stage for Serde's custom error.
impl std::fmt::Display for RealDecodeError {
    /// Write problem, a colon, and the detail; propagate any formatter failure.
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}: {}", self.problem, self.detail)
    }
}

#[cfg(feature = "serde")]
/// Admit a decoded definition before it can become a usable Real.
impl RealWire {
    /// Check opaque payload presence, class parameters, then payload domains in that order.
    /// A zero scale never bypasses these obligations. For named classes with a
    /// payload, accept structural equality or a proved defined-product equality
    /// with the reconstructed basis. A proved mismatch is ParseError; unavailable
    /// equality is UnknownCertificate, including nonrational unresolved cases.
    /// A recognized rational contradicting a proved-irrational named class or nonunit One
    /// payload is also rejected. Opaque payloads need no named-basis comparison.
    /// Preserve admitted fields and start an empty primitive cache. These proofs
    /// may allocate/refine; there is no total-work budget or partial-success result.
    pub(crate) fn validate(self) -> Result<Real, RealDecodeError> {
        // Symbolic classes can reconstruct a missing lazy payload from their
        // certificate. An opaque class has no such fallback, including when
        // the outer scale happens to be zero.
        if matches!(self.class, Irrational) && self.computable.is_none() {
            return Err(RealDecodeError {
                problem: Problem::ParseError,
                detail: "Real Irrational class requires a Computable payload",
            });
        }
        self.class
            .validate_serialized_parameters()
            .map_err(|problem| RealDecodeError {
                problem,
                detail: "Class parameter certificate is invalid or unproved",
            })?;
        // Prove operand domains before any payload comparison refines the
        // graph, and regardless of the outer scale: 0 * sqrt(-pi) is not zero.
        if let Some(payload) = &self.computable {
            payload
                .prove_imported_domains()
                .map_err(|problem| RealDecodeError {
                    problem,
                    detail: "Computable payload operation domain is invalid or unproved",
                })?;
        }
        if !matches!(self.class, Irrational)
            && let Some(payload) = &self.computable
        {
            let expected = self.class.computable_certificate();
            if !Computable::internal_structural_eq(payload, &expected) {
                let equality = payload.compare_defined_positive_products(&expected);
                let problem = match equality {
                    Some(Sign::NoSign) => None,
                    Some(Sign::Minus | Sign::Plus) => Some(Problem::ParseError),
                    None => {
                        if let Some(rational) = payload.exact_rational()
                            && (self.class.definitely_irrational()
                                || matches!(self.class, One) && !rational.is_one())
                        {
                            Some(Problem::ParseError)
                        } else {
                            Some(Problem::UnknownCertificate)
                        }
                    }
                };
                if let Some(problem) = problem {
                    return Err(RealDecodeError {
                        problem,
                        detail: "Class and Computable payload equality is not certified",
                    });
                }
            }
        }
        Ok(Real {
            rational: self.rational,
            class: self.class,
            computable: self.computable,
            primitive_approx_cache: AtomicPrimitiveApproxCache::default(),
        })
    }
}

#[cfg(feature = "serde")]
/// Deserialize through the untrusted wire carrier and the same invariant admission as wrappers.
impl<'de> Deserialize<'de> for Real {
    /// Decode fields, validate, and convert the category/context into the deserializer's custom error.
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        RealWire::deserialize(deserializer)?
            .validate()
            .map_err(serde::de::Error::custom)
    }
}

/// Preserve value and reusable graph evidence while selecting a cheap retained representation.
impl Clone for Real {
    /// Pure rationals stay payload-free; copy their primitive view only if either
    /// cache feature is enabled. Other cases always copy the primitive-view snapshot.
    /// Preserve an existing signaled, opaque or ConstOffset payload; discard an
    /// unsignaled One payload; otherwise clone or reconstruct the named basis.
    /// Computable cloning shares its node/caches and any attached signal, while
    /// primitive views are stored independently in the new Real.
    #[inline]
    fn clone(&self) -> Self {
        if matches!(self.class, One) && self.computable.is_none() {
            // Exact rationals dominate dense geometry carriers. They have no
            // computable payload or abort signal, and their class certificate
            // is the unit variant, so avoid the generic symbolic-clone
            // decision tree and enum clone entirely.
            return Self {
                rational: self.rational.clone(),
                class: One,
                computable: None,
                primitive_approx_cache: {
                    #[cfg(any(feature = "cached-f32-approx", feature = "cached-f64-approx"))]
                    {
                        AtomicPrimitiveApproxCache::new(self.primitive_approx_cache.get())
                    }
                    #[cfg(not(any(feature = "cached-f32-approx", feature = "cached-f64-approx")))]
                    {
                        AtomicPrimitiveApproxCache::new(PrimitiveApproxCache::Empty)
                    }
                },
            };
        }

        // Computable clones share immutable nodes and synchronized caches.
        // Reuse the payload when present, retaining the certificate fallback
        // for exact symbols whose payload has not been materialized.
        let computable =
            if self.abort_signal().is_some() || matches!(self.class, Irrational | ConstOffset(_)) {
                // ConstOffset payloads are shallow enough to clone and expensive
                // enough to rebuild that scalar rows like 1000*pi+eps regress if
                // every clone reconstructs cached pi plus a rational offset tree.
                self.computable.clone()
            } else if matches!(self.class, One) {
                None
            } else {
                Some(self.computable_clone())
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

/// Native construction, representation reuse, exact arithmetic helpers and qualified decisions.
impl Real {
    /// Refinement floor used by the `PartialOrd` implementation.
    ///
    /// Comparisons return certified order or None, with refinement requests
    /// limited by this binary precision floor. It is not a total-work, time or
    /// memory bound. Native partial-operation admission also uses this floor.
    pub const PARTIAL_CMP_MIN_PRECISION: i32 = -2048;
    /// Binary precision floor for integer boundary/sign decisions; not a total-work limit.
    const CERTIFIED_INTEGER_COMPARE_TOLERANCE: i32 = -1024;

    /// Wrap an already-valid Rational as the unit basis with no payload and an empty primitive cache.
    /// No float projection or additional domain test is needed; rational invariants belong to its owner.
    fn exact_rational_unchecked(rational: Rational) -> Real {
        Self {
            rational,
            class: One,
            computable: None,
            primitive_approx_cache: AtomicPrimitiveApproxCache::new(PrimitiveApproxCache::Empty),
        }
    }

    /// Test basis identity without comparing rational scales or proving arbitrary value equality.
    /// Two opaque bases require present structurally equal graphs; all other pairs compare class certificates.
    /// A false result does not prove distinct mathematical values.
    pub(super) fn same_symbolic_basis(&self, other: &Self) -> bool {
        match (&self.class, &other.class) {
            (Irrational, Irrational) => match (&self.computable, &other.computable) {
                (Some(left), Some(right)) => Computable::internal_structural_eq(left, right),
                _ => false,
            },
            _ => self.class == other.class,
        }
    }

    /// Borrow the unscaled basis graph if present, otherwise construct it from the class certificate.
    /// Missing opaque payloads violate the invariant and panic in certificate reconstruction.
    /// Use fold_ref when the rational scale must be included.
    fn computable_ref(&self) -> Cow<'_, Computable> {
        // A named certificate may retain its recipe without a materialized
        // payload. Borrow existing graphs so their proofs and signals survive;
        // only reconstruct when the graph is absent.
        match &self.computable {
            Some(computable) => Cow::Borrowed(computable),
            None => Cow::Owned(self.class.computable_certificate()),
        }
    }

    /// Clone/share the existing unscaled basis graph or reconstruct a lazy named certificate.
    /// Preserve a retained graph signal; a missing opaque graph is an invariant violation.
    pub(super) fn computable_clone(&self) -> Computable {
        self.computable
            .clone()
            .unwrap_or_else(|| self.class.computable_certificate())
    }

    /// Consume and move the unscaled basis payload, reconstructing a named certificate if absent.
    /// Discard the outer rational scale and primitive view; this is not full-value folding.
    pub(super) fn into_computable(self) -> Computable {
        self.computable
            .unwrap_or_else(|| self.class.computable_certificate())
    }

    /// Provide an atomic flag to signal early abort of calculations.
    /// The provided flag can be used e.g. from another execution thread.
    /// Aborted calculations may have incorrect results.
    /// This clears only this Real's primitive view, materializes a missing named
    /// basis payload if needed, and replaces its attached signal. It does not set
    /// the flag, synchronously stop work, clear shared node caches, or return a
    /// cancellation receipt. Existing clones retain their own signal attachment.
    pub fn abort(&mut self, s: Signal) {
        self.primitive_approx_cache.set(PrimitiveApproxCache::Empty);
        let computable = self
            .computable
            .get_or_insert_with(|| self.class.computable_certificate());
        computable.abort(s);
    }

    /// The abort flag attached with [`Real::abort`], if any.
    ///
    /// Consumers that derive a new value for refinement can reattach it, so a
    /// cancellation request still reaches the derived calculation.
    pub fn abort_signal(&self) -> Option<&Signal> {
        self.computable
            .as_ref()
            .and_then(|computable| computable.signal.as_ref())
    }

    /// Read the attached flag with relaxed ordering, or return false when no flag is attached.
    /// This is a cooperative request observation, not a record that evaluation actually stopped.
    pub(super) fn is_aborted(&self) -> bool {
        use std::sync::atomic::Ordering::Relaxed;

        self.abort_signal()
            .is_some_and(|signal| signal.load(Relaxed))
    }

    /// Attach this Real's signal to the supplied graph when present, replacing its previous signal.
    /// With no source signal, keep the graph's own signal unchanged; do not combine cancellation sources.
    pub(super) fn inherit_abort(&self, mut computable: Computable) -> Computable {
        if let Some(signal) = self.abort_signal() {
            computable.abort(signal.clone());
        }
        computable
    }

    /// Zero, the additive identity, as an exact rational with no computable payload or signal.
    pub fn zero() -> Real {
        crate::trace_dispatch!("real", "constructor", "zero");
        Self::exact_rational_unchecked(Rational::zero())
    }

    /// One, the multiplicative identity, as an exact rational with an empty primitive view.
    pub fn one() -> Real {
        crate::trace_dispatch!("real", "constructor", "one");
        Self::exact_rational_unchecked(Rational::one())
    }

    /// Transfer the specified [`Rational`] into a payload-free Real without changing its exact value.
    pub fn new(rational: Rational) -> Real {
        crate::trace_dispatch!("real", "constructor", "rational");
        Self::exact_rational_unchecked(rational)
    }

    /// Wrap an arbitrary-size signed integer over denominator one, with no float conversion.
    /// Construction expects the constant nonzero-denominator invariant.
    pub fn integer(integer: BigInt) -> Real {
        crate::trace_dispatch!("real", "constructor", "bigint");
        Self::exact_rational_unchecked(
            Rational::from_bigint_fraction(integer, BigUint::from(1_u8))
                .expect("integer denominator is nonzero"),
        )
    }

    /// Returns the exact sum of owned Real values.
    ///
    /// Proven-long heterogeneous iterators use a streaming balanced reducer so
    /// computable guard precision grows logarithmically with the number of
    /// terms. A homogeneous rational or symbolic prefix first collapses to one
    /// scaled value. Short and size-unknown iterators retain the lower-overhead
    /// sequential path. The current threshold is a lower size hint of 256.
    /// Empty iterators yield zero. Finite inputs are expected: there is no total
    /// iteration/coefficient-growth budget, and cache/representation layout can
    /// differ from a sequential fold even when the exact sum agrees.
    pub fn sum_owned(values: impl IntoIterator<Item = Real>) -> Real {
        crate::trace_dispatch!("real", "aggregate", "sum-owned");
        real_sum(values.into_iter())
    }

    /// Returns the exact sum of borrowed Real values.
    ///
    /// Proven-long iterators use the same streaming balanced reducer as
    /// [`Self::sum_owned`]. The short path adds references without cloning each
    /// input; the balanced path clones inputs. Empty input yields zero.
    pub fn sum_refs<'a>(values: impl IntoIterator<Item = &'a Real>) -> Real {
        crate::trace_dispatch!("real", "aggregate", "sum-refs");
        real_sum_borrowed(values.into_iter())
    }

    /// Evaluate `origin + t * delta` by exact multiplication followed by addition.
    /// No restriction to t in [0, 1] is imposed; extrapolation is allowed.
    pub fn affine(origin: &Real, t: &Real, delta: &Real) -> Real {
        crate::trace_dispatch!("real", "aggregate", "affine");
        origin + &(t * delta)
    }

    /// Returns the absolute value of this Real.
    ///
    /// Known signs are handled structurally. If the sign cannot be certified at
    /// a cheap bounded precision, the result is represented as `sqrt(x^2)` in
    /// the computable-real graph so the operation remains exact without making
    /// a discontinuous sign decision. The query floor is zero, not a total-work
    /// bound. The known-zero branch returns a fresh zero; no separate certainty
    /// receipt or cancellation result accompanies this value.
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

    /// Truncate the exact rational and subtract one only for a negative noninteger.
    /// The BigInt conversion expects the invariant that truncation returns an integer.
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

    /// Truncate the exact rational and add one only for a positive noninteger.
    /// This uses no approximate comparison; integer extraction is an internal invariant.
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

    /// Certify candidate <= self first, then self < candidate+1, using precision floor -1024.
    /// Clone the candidate only if both comparisons prove the half-open interval; None
    /// conflates a refuted candidate with an unresolved comparison and is not a value proof.
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

    /// Return either the floor or the ceiling of this value.
    ///
    /// This is a total multivalued alternative to the directional certified
    /// rounding methods: it never has to decide whether a non-rational value is
    /// exactly on an integer boundary. The particular adjacent integer is
    /// deliberately unspecified and need not be the nearest integer. Recognized
    /// rationals use floor; other values use the folded graph's approximation at
    /// precision -2 and integer scaling. The adjacency guarantee assumes valid,
    /// uninterrupted evaluation; one precision request does not bound total work.
    pub fn near_integer(&self) -> BigInt {
        if let Some(rational) = self.exact_rational_ref() {
            crate::trace_dispatch!("real", "integer-rounding", "near-integer-exact-rational");
            return Self::exact_rational_floor(rational);
        }

        crate::trace_dispatch!("real", "integer-rounding", "near-integer-computable");
        self.fold_ref().near_integer()
    }

    /// Certified floor as an integer.
    ///
    /// After ordinary bounded comparisons, an exact rational normal form can
    /// certify an otherwise unresolved integer boundary using the same value.
    /// Returns [`Problem::Exhausted`] when bounded exact-real refinement cannot
    /// certify the integer boundary. Recognized rationals take the exact path.
    /// Otherwise try the nine candidates estimate-4 through estimate+4 around
    /// approx(0), testing at floor -1024, then attempt exact rational normalization.
    /// Exhausted here reports failure of that decision schedule, not a reached
    /// total-work/memory budget. No certificate receipt is returned with the integer.
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
        // Refinement may expose exact facts needed by symbolic normalization.
        // A failed comparison is not a proof that the rational form is absent.
        if let Some(rational) = self.exact_rational_normal_form() {
            crate::trace_dispatch!("real", "integer-rounding", "floor-rational-normal-form");
            return Ok(Self::exact_rational_floor(&rational));
        }
        Err(Problem::Exhausted)
    }

    /// Certified ceiling as an integer, with an exact rational fast path.
    /// Otherwise certify floor, then equality to it at precision -1024: equality
    /// keeps floor, inequality adds one. An unresolved equality tries rational
    /// normalization; unresolved work or failed floor propagates Problem::Exhausted.
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
            CertifiedRealEquality::Unknown { .. } => {
                let rational = self
                    .exact_rational_normal_form()
                    .ok_or(Problem::Exhausted)?;
                crate::trace_dispatch!("real", "integer-rounding", "ceil-rational-normal-form");
                Ok(Self::exact_rational_ceil(&rational))
            }
        }
    }

    /// Certified truncation toward zero as an integer.
    /// Rational storage truncates directly. Otherwise certify sign at -1024,
    /// choose ceiling for negative values and floor for positive values, or return
    /// zero for proved zero. An unresolved sign returns Problem::Exhausted.
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
    /// Certify sign at -1024, then ceil(x-1/2) for negative or floor(x+1/2)
    /// for positive values. Proved zero returns zero; unresolved sign or directional
    /// rounding returns Problem::Exhausted. This is not IEEE ties-to-even rounding.
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

    /// Certified fractional part x-trunc(x), retaining the original sign for nonzero fractions.
    /// Propagate truncation failure; success is an exact Real difference, not a
    /// floating remainder or the always-nonnegative fractional convention.
    pub fn fract_certified(&self) -> Result<Real, Problem> {
        crate::trace_dispatch!("real", "integer-rounding", "fract-certified");
        let trunc = Self::integer(self.trunc_certified()?);
        Ok(self - &trunc)
    }

    /// Certified Euclidean remainder x-floor(x/modulus)*modulus for positive modulus.
    /// Check modulus > 0 first at -1024: proved nonpositive gives NotANumber,
    /// unresolved order gives Exhausted. Then propagate division/floor failures.
    /// Successful valid evaluation lies in [0, modulus); no approximate quotient is used.
    pub fn rem_euclid_certified(&self, modulus: &Real) -> Result<Real, Problem> {
        crate::trace_dispatch!("real", "integer-rounding", "rem-euclid-certified");
        match modulus.certified_cmp_until(&Self::zero(), Self::CERTIFIED_INTEGER_COMPARE_TOLERANCE)
        {
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

    /// Exact sum divided by the slice length.
    /// Return None for empty input, a length not representable as u64, or a
    /// division failure. The Option discards the scalar error category; summation
    /// uses sum_refs and has no aggregate work/allocation budget.
    pub fn mean(values: &[Real]) -> Option<Real> {
        crate::trace_dispatch!("real", "aggregate", "mean");
        if values.is_empty() {
            return None;
        }
        let count = Real::from(u64::try_from(values.len()).ok()?);
        (Self::sum_refs(values.iter()) / count).ok()
    }

    /// Compute sqrt(sum((x-mean)^2)/(n-1)), using Bessel's sample divisor.
    /// Require at least two values. Return None for a failed mean, u64 divisor
    /// conversion, division or root admission; error categories are discarded.
    /// Squared deviations accumulate sequentially in input order. Exact does not
    /// imply a rational answer or a bounded total-work/memory contract.
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
    /// Clone the thread-local canonical definition, retaining exact symbolic structure.
    pub fn pi() -> Real {
        crate::trace_dispatch!("real", "constructor", "cached-pi");
        constants::pi()
    }

    /// τ, the ratio of a circle's circumference to its radius.
    /// Clone the thread-local canonical definition, retaining exact symbolic structure.
    pub fn tau() -> Real {
        crate::trace_dispatch!("real", "constructor", "cached-tau");
        constants::tau()
    }

    /// e, Euler's number and the base of the natural logarithm function.
    /// Clone the thread-local canonical definition, retaining exact symbolic structure.
    pub fn e() -> Real {
        crate::trace_dispatch!("real", "constructor", "cached-e");
        constants::e()
    }

    /// Returns the smaller value according to certified partial ordering.
    ///
    /// If refinement cannot decide the order at the standard partial-order
    /// precision, this keeps `self`, matching the conservative behavior of
    /// [`PartialOrd`] callers that treat incomparability as no improvement.
    /// Equality also keeps self; this returns an operand reference, not a proof
    /// that it is the mathematical minimum when the order remains unresolved.
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
    /// Equality also keeps self; an unresolved result is not a proved maximum.
    pub fn max<'r>(&'r self, other: &'r Real) -> &'r Real {
        match self.partial_cmp(other) {
            Some(Ordering::Less) => other,
            _ => self,
        }
    }

    /// Returns true for every constructed hyperreal value.
    ///
    /// `Real` has no NaN or infinity inhabitants; failed primitive conversions
    /// are rejected before construction. This constant true query does not inspect
    /// a graph, validate a certificate, or check whether evaluation was aborted.
    pub const fn is_finite(&self) -> bool {
        true
    }
}

/// Borrow the same Real without cloning, materialization or cache queries.
impl AsRef<Real> for Real {
    /// Return self with its existing lifetime and representation.
    fn as_ref(&self) -> &Real {
        self
    }
}

/// Reduce rational pi-multiples for tangent to a magnitude in [0, 1/2].
/// Discard the integer period, make the signed fractional part positive, then
/// reflect fractions above 1/2 and toggle the sign. Return the sign flag and
/// reduced argument; the caller handles zero and the 1/2 pole. This helper does
/// not certify tangent's domain or exclude boundary inputs.
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

/// Reduce a rational pi-multiple to sine's [0, 1/2] principal argument and sign.
/// Reflect the fractional part above 1/2, use integer parity for positive turns,
/// and apply odd symmetry for negative inputs. Boundaries are accepted; a sign
/// flag at a zero argument is not a proof of a nonzero sine. Callers own exact
/// endpoint/table specialization before constructing an interior SinPi class.
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

/// Sign flag for exact nonzero sine-table magnitudes at rational pi-multiples.
/// Use integer-part parity for nonnegative inputs and toggle for negative ones.
/// This helper neither reduces the magnitude nor detects integer zeros; callers
/// supply the applicable table case, so the flag alone is not a sign certificate.
fn sin_pi_neg(r: Rational) -> bool {
    // For exact sin(q*pi) table entries, the integer part alone determines the
    // sign after the rational argument has been normalized.
    if r.sign() == Sign::Minus {
        return !sin_pi_neg(r.neg());
    }
    r.shifted_big_integer(0).bit(0)
}
