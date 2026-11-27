/// Retained positive product pi^n * exp(q), with a signed integral pi exponent.
/// Native factories prefer smaller variants when possible; wire admission may retain equivalent nonspecialized forms.
#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub(crate) struct ConstProductClass {
    // Signed pi power lets reciprocal products such as 1/pi and e^q/pi remain
    // symbolic instead of falling into a generic inverse node.
    /// Signed pi exponent; negative powers represent reciprocals without changing the outer Real scale.
    pi_power: i16,
    /// Exact rational exponent of e, including zero for a pure pi power.
    exp_power: Rational,
}

/// Retained positive sum pi^n * exp(q) + offset.
/// Native construction uses cheap positivity certificates; wire admission can also use bounded sign refinement.
#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub(crate) struct ConstOffsetClass {
    // Invariant: the inner value pi^n*e^q + offset is constructed only when
    // cheaply certified positive. The outer Real.rational carries any sign.
    /// Signed exponent of pi in the positive multiplicative term.
    pi_power: i16,
    /// Exact rational exponent of e in that term.
    exp_power: Rational,
    /// Exact additive rational; positivity concerns the entire basis, not this field alone.
    offset: Rational,
}

/// Factored positive product pi^n * exp(q) * sqrt(r).
/// Keeping the root separate allows subsequent products and inverses to reuse its defining relation.
#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub(crate) struct ConstProductSqrtClass {
    // Factored sqrt products are positive internally. Keeping the sqrt separate
    // allows later multiplication/division to cancel it exactly.
    /// Signed pi exponent, independent of the rational outer scale.
    pi_power: i16,
    /// Exact rational exponential factor.
    exp_power: Rational,
    /// Positive rational under the square root; without pi/e factors it must not be a perfect rational square.
    radicand: Rational,
}

/// Positive basis offset + ln(base), retaining the additive logarithmic definition.
/// Native construction requires base > 1 and nonnegative offset; wire admission also accepts other proved-positive sums.
#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub(crate) struct LnAffineClass {
    // Constructed only for positive offset + ln(base). This preserves the
    // nonzero/sign invariant shared by all non-Irrational classes.
    /// Exact rational additive term; wire-validated values need not have nonnegative offset.
    offset: Rational,
    /// Positive rational unequal to one, with positivity of the entire sum established separately.
    base: Rational,
}

/// Positive product ln(left) * ln(right).
/// Native construction sorts bases; wire validation requires positive nonunit bases on the same side of one but does not sort them.
#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub(crate) struct LnProductClass {
    // Bases are sorted at construction so products compare and combine without
    // considering operand order.
    /// First exact rational log base; native factories put the lesser base here.
    left: Rational,
    /// Second exact rational log base, whose log has the same nonzero sign as the first.
    right: Rational,
}

/// Exact basis certificates kept separately from Real's rational scale.
/// Named variants denote positive nonzero bases after native construction or
/// wire validation. The opaque Irrational tag has no such sign/rationality claim.
/// A certificate is not the whole scaled value: a zero outer scale is still zero.
/// Derived decoding alone does not admit these invariants; RealWire validates them.
/// Equality recognizes matching retained forms, not all mathematical identities.
#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub(crate) enum Class {
    /// Unit basis; the outer Real rational scale carries the entire value.
    One,
    /// The positive transcendental constant pi.
    Pi,
    /// Positive integral pi power; admitted exponent is at least two.
    PiPow(
        /// Exponent n in pi^n; zero and one use simpler native variants.
        u8,
    ),
    /// The positive reciprocal 1/pi.
    PiInv,
    /// Product pi * exp(q); no general irrationality claim follows for a nonzero rational q.
    PiExp(
        /// Exact exponential exponent q; wire admission permits zero.
        Rational,
    ),
    /// Product exp(q)/pi, retaining the reciprocal pi factor.
    PiInvExp(
        /// Exact rational exponent of e; mathematical rationality need not be known.
        Rational,
    ),
    /// Product pi * sqrt(r) for a positive rational r, including perfect squares.
    PiSqrt(
        /// Positive exact radicand; pi supplies the transcendental factor.
        Rational,
    ),
    /// General signed-pi-power/exponential product, boxed to keep uncommon parameters indirect.
    ConstProduct(
        /// Retained multiplicative parameters, possibly equivalent to a dedicated wire variant.
        Box<ConstProductClass>,
    ),
    /// Proved-positive constant product plus a rational offset.
    ConstOffset(
        /// Retained product/offset parameters; the outer Real scale supplies the overall sign.
        Box<ConstOffsetClass>,
    ),
    /// Positive pi/exponential product with an explicit rational square root.
    ConstProductSqrt(
        /// Factored definition whose root relation can be reused by multiplication/division.
        Box<ConstProductSqrtClass>,
    ),
    /// Square root of a positive rational with no perfect rational square root.
    Sqrt(
        /// Exact nonsquare radicand; not restricted to integral or square-free inputs by wire admission.
        Rational,
    ),
    /// Positive exp(q) with nonzero rational q; exp(0) uses One in native construction.
    Exp(
        /// Nonzero exact exponent, guaranteeing a transcendental basis.
        Rational,
    ),
    /// Positive natural logarithm of a rational greater than one.
    Ln(
        /// Exact base greater than one; logarithm is nonzero and transcendental.
        Rational,
    ),
    /// Proved-positive rational offset plus a natural logarithm.
    LnAffine(
        /// Additive definition whose sign is established at construction or wire admission.
        Box<LnAffineClass>,
    ),
    /// Positive product of two nonzero real logarithms, without a general irrationality claim.
    LnProduct(
        /// Native sorted or wire-preserved pair of bases with same-sign logarithms.
        Box<LnProductClass>,
    ),
    /// Positive base-ten logarithm excluding bases with an exact integer base-ten logarithm.
    Log10(
        /// Rational base greater than one; this excludes powers of ten, not all multiples of ten.
        Rational,
    ),
    /// Positive binary logarithm excluding bases with an exact integer binary logarithm.
    Log2(
        /// Rational base greater than one, excluding integral powers of two.
        Rational,
    ),
    /// Positive ten raised to a noninteger rational exponent.
    Pow10(
        /// Exact nonintegral exponent; native simplification may choose a root specialization.
        Rational,
    ),
    /// Positive two raised to a noninteger rational exponent.
    Pow2(
        /// Exact nonintegral exponent, retained without primitive-float projection.
        Rational,
    ),
    /// Positive sine of pi*q for 0 < q < 1/2, excluding q=1/6.
    /// Native tables also simplify additional special angles; wire admission need not do so.
    SinPi(
        /// Exact principal pi-multiple; excluded 1/6 would give the rational one-half.
        Rational,
    ),
    /// Positive tangent of pi*q for 0 < q < 1/2, excluding q=1/4.
    /// Native tables also simplify additional special angles; the half-turn pole is outside this domain.
    TanPi(
        /// Exact principal pi-multiple; excluded 1/4 would give the rational one.
        Rational,
    ),
    /// Opaque basis requiring a separate Computable payload.
    /// The name is not proof of irrationality, sign or nonzero value; even this tag compared with itself is not a Class equality proof.
    Irrational,
}

use Class::*;
/// Disposable primitive projection state, not an exact scalar certificate.
/// Some values must be finite; None records a conversion overflow. F64 exists independently of cache feature flags.
#[derive(Clone, Copy, Debug, Default)]
pub(super) enum PrimitiveApproxCache {
    /// No reusable primitive projection has been stored.
    #[default]
    Empty,
    /// Feature-enabled binary32 projection or overflow result.
    #[cfg(feature = "cached-f32-approx")]
    F32(
        /// Finite binary32 bits, or None for overflow.
        Option<f32>,
    ),
    /// Binary64 projection or overflow result; also used when cached-f64-approx is disabled.
    F64(
        /// Finite binary64 bits, or None for overflow.
        Option<f64>,
    ),
}

/// One atomic word holding disposable primitive projections with binary64 preference.
///
/// Exact-real values never convert to NaN, so reserved quiet-NaN bit patterns
/// can represent the empty and overflow states without increasing `Real`'s
/// layout. A tagged negative-NaN range carries cached finite `f32` values;
/// every other finite bit pattern is a cached `f64`.
/// Callers must supply finite Some values; encoding does not validate that rule.
/// Reserved NaNs would collide with tags, so this is not an arbitrary-float codec.
/// Relaxed accesses synchronize this word only, not unrelated data. Same-rank
/// writes may replace one another; reset is unconditional and can race with writers.
/// The weak-CAS retry loop has no per-call iteration or cancellation bound.
pub(super) struct AtomicPrimitiveApproxCache(
    /// Tagged bits; only the encoder/decoder define their interpretation.
    std::sync::atomic::AtomicU64,
);

/// Encode, load and update the single-word projection cache without numeric proof work.
impl AtomicPrimitiveApproxCache {
    /// Reserved positive quiet-NaN bit pattern for an empty view.
    const EMPTY: u64 = 0x7ff8_0000_0000_0001;
    #[cfg(feature = "cached-f32-approx")]
    /// Reserved quiet-NaN bit pattern for binary32 overflow.
    const F32_NONE: u64 = 0x7ff8_0000_0000_0002;
    /// Reserved quiet-NaN bit pattern for binary64 overflow.
    const F64_NONE: u64 = 0x7ff8_0000_0000_0003;
    #[cfg(feature = "cached-f32-approx")]
    /// High-word negative-NaN tag; the low 32 bits contain a finite binary32 encoding.
    const F32_TAG: u64 = 0xffff_ffff_0000_0000;

    /// Encode one caller-admitted view into a fresh atomic word; do not validate finite-value preconditions.
    pub(super) fn new(value: PrimitiveApproxCache) -> Self {
        Self(std::sync::atomic::AtomicU64::new(Self::encode(value)))
    }

    #[inline]
    /// Load relaxed and decode one atomic snapshot, without refinement or synchronization of other state.
    pub(super) fn get(&self) -> PrimitiveApproxCache {
        Self::decode(self.0.load(std::sync::atomic::Ordering::Relaxed))
    }

    /// Clear unconditionally for Empty; otherwise retry weak compare-exchange while the stored rank is no higher.
    /// F64 outranks F32 and Empty; equal ranks may replace. A concurrent higher rank stops the update.
    /// This prioritizes representation width, not a certified accuracy order, and has no bounded retry count.
    pub(super) fn set(&self, value: PrimitiveApproxCache) {
        let encoded = Self::encode(value);
        if matches!(value, PrimitiveApproxCache::Empty) {
            self.0.store(encoded, std::sync::atomic::Ordering::Relaxed);
            return;
        }

        // Keep a concurrently-computed f64 in preference to f32. Conversion
        // caches are accelerators only, so relaxed ordering is sufficient.
        let mut current = self.0.load(std::sync::atomic::Ordering::Relaxed);
        while Self::rank(current) <= Self::rank(encoded) {
            match self.0.compare_exchange_weak(
                current,
                encoded,
                std::sync::atomic::Ordering::Relaxed,
                std::sync::atomic::Ordering::Relaxed,
            ) {
                Ok(_) => break,
                Err(observed) => current = observed,
            }
        }
    }

    /// Encode finite views or sentinel states; callers own the finite-Some invariant and source association.
    fn encode(value: PrimitiveApproxCache) -> u64 {
        match value {
            PrimitiveApproxCache::Empty => Self::EMPTY,
            #[cfg(feature = "cached-f32-approx")]
            PrimitiveApproxCache::F32(Some(value)) => Self::F32_TAG | u64::from(value.to_bits()),
            #[cfg(feature = "cached-f32-approx")]
            PrimitiveApproxCache::F32(None) => Self::F32_NONE,
            PrimitiveApproxCache::F64(Some(value)) => value.to_bits(),
            PrimitiveApproxCache::F64(None) => Self::F64_NONE,
        }
    }

    /// Check Empty, feature-enabled F32 sentinels/tag, then F64 overflow, otherwise reinterpret as F64 bits.
    /// This is inverse encoding on admitted states, not validation of arbitrary bit patterns.
    fn decode(value: u64) -> PrimitiveApproxCache {
        if value == Self::EMPTY {
            return PrimitiveApproxCache::Empty;
        }
        #[cfg(feature = "cached-f32-approx")]
        {
            if value == Self::F32_NONE {
                return PrimitiveApproxCache::F32(None);
            }
            if value & 0xffff_ffff_0000_0000 == Self::F32_TAG {
                return PrimitiveApproxCache::F32(Some(f32::from_bits(value as u32)));
            }
        }
        if value == Self::F64_NONE {
            return PrimitiveApproxCache::F64(None);
        }
        PrimitiveApproxCache::F64(Some(f64::from_bits(value)))
    }

    /// Decode the state and rank Empty=0, F32=1, F64=2, including overflow states.
    /// The ordering is cache preference, not scalar magnitude or mathematical certainty.
    fn rank(value: u64) -> u8 {
        match Self::decode(value) {
            PrimitiveApproxCache::Empty => 0,
            #[cfg(feature = "cached-f32-approx")]
            PrimitiveApproxCache::F32(_) => 1,
            PrimitiveApproxCache::F64(_) => 2,
        }
    }
}

/// Default to an empty projection, without evaluating the exact source.
impl Default for AtomicPrimitiveApproxCache {
    /// Initialize the reserved Empty state.
    fn default() -> Self {
        Self::new(PrimitiveApproxCache::Empty)
    }
}

/// Format the current decoded projection snapshot rather than the tagged storage word.
impl std::fmt::Debug for AtomicPrimitiveApproxCache {
    /// Delegate formatting to one decoded snapshot; concurrent writes may change a later snapshot.
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        self.get().fmt(formatter)
    }
}

/// Recognize identical named representations, comparing retained rational parameters in order.
/// Different tags never match; two Irrational tags also return false because the
/// graph identity lives elsewhere. Thus this is deliberately not reflexive for
/// the opaque marker and cannot implement Eq or serve as semantic scalar equality.
impl PartialEq for Class {
    /// Compare matching tag parameters, short-circuiting in field order; return false for every other pair.
    fn eq(&self, other: &Self) -> bool {
        match (self, other) {
            (One, One) => true,
            (Pi, Pi) => true,
            (PiPow(r), PiPow(s)) => r == s,
            (PiInv, PiInv) => true,
            (PiExp(r), PiExp(s)) => r == s,
            (PiInvExp(r), PiInvExp(s)) => r == s,
            (PiSqrt(r), PiSqrt(s)) => r == s,
            (ConstProduct(left), ConstProduct(right)) => {
                left.pi_power == right.pi_power && left.exp_power == right.exp_power
            }
            (ConstOffset(left), ConstOffset(right)) => {
                left.pi_power == right.pi_power
                    && left.exp_power == right.exp_power
                    && left.offset == right.offset
            }
            (ConstProductSqrt(left), ConstProductSqrt(right)) => {
                left.pi_power == right.pi_power
                    && left.exp_power == right.exp_power
                    && left.radicand == right.radicand
            }
            (Sqrt(r), Sqrt(s)) => r == s,
            (Exp(r), Exp(s)) => r == s,
            (Ln(r), Ln(s)) => r == s,
            (LnAffine(left), LnAffine(right)) => {
                left.offset == right.offset && left.base == right.base
            }
            (LnProduct(left), LnProduct(right)) => {
                left.left == right.left && left.right == right.right
            }
            (Log10(r), Log10(s)) => r == s,
            (Log2(r), Log2(s)) => r == s,
            (Pow10(r), Pow10(s)) => r == s,
            (Pow2(r), Pow2(s)) => r == s,
            (SinPi(r), SinPi(s)) => r == s,
            (TanPi(r), TanPi(s)) => r == s,
            (_, _) => false,
        }
    }
}

/// Construct/decompose exact retained bases and validate their admitted parameter domains.
impl Class {
    /// Whether this exact symbolic class proves that its value is irrational.
    ///
    /// Keep this deliberately narrower than "non-rational class". Products
    /// involving both powers of pi and powers of e, and products of logarithms,
    /// include familiar constants whose rationality is not known. A single
    /// transcendental factor times a nonzero algebraic factor is safe, as are
    /// the retained algebraic roots and rational-angle trigonometric values.
    /// This assumes admitted parameters and concerns the basis only, not a zero
    /// outer Real scale. False means no irrationality proof here, not rationality.
    /// Mixed pi/e products remain conservative; constant-product tests use the
    /// presence of their factors and the admitted nonsquare-root invariant.
    fn definitely_irrational(&self) -> bool {
        match self {
            One | PiExp(_) | PiInvExp(_) | LnProduct(_) | Irrational => false,
            Pi | PiPow(_) | PiInv | PiSqrt(_) | Sqrt(_) | Exp(_) | Ln(_) | LnAffine(_)
            | Log10(_) | Log2(_) | Pow10(_) | Pow2(_) | SinPi(_) | TanPi(_) => true,
            ConstProduct(product) => {
                let has_pi = product.pi_power != 0;
                let has_exp = product.exp_power.sign() != Sign::NoSign;
                has_pi ^ has_exp
            }
            ConstOffset(offset) => {
                let has_pi = offset.pi_power != 0;
                let has_exp = offset.exp_power.sign() != Sign::NoSign;
                has_pi ^ has_exp
            }
            ConstProductSqrt(product) => {
                let has_pi = product.pi_power != 0;
                let has_exp = product.exp_power.sign() != Sign::NoSign;
                !(has_pi && has_exp)
            }
        }
    }

    /// Recognize only the simple Ln variant for cheap two-log combination; reject wider logarithmic forms.
    fn is_ln(&self) -> bool {
        // Only simple `Ln(base)` values participate in the two-log sum/difference
        // collapse. Wider log classes intentionally skip that shortcut so the
        // simplifier stays cheap on non-log algebra.
        matches!(self, Ln(_))
    }

    /// Normalize exp(0) to One; otherwise pair the retained nonzero rational exponent with its exact computable definition.
    fn make_exp(br: Rational) -> (Class, Computable) {
        // `e^0` is the rational one. Normalizing here keeps exp-products from
        // leaving behind a symbolic class that would slow equality and sign checks.
        if br == *rationals::ZERO {
            (One, Computable::one())
        } else {
            (Exp(br.clone()), Computable::exp_rational(br))
        }
    }

    /// Build pi^power with one multiplication per additional factor; zero returns one.
    /// The u16 parameter bounds the loop count, not graph evaluation cost or memory independently.
    /// This is a left-associated construction, not exponentiation by squaring.
    fn pi_power_computable(power: u16) -> Computable {
        // Retained symbolic pi powers use a linear multiply chain.
        // Generic powers use their separate pow machinery.
        if power == 0 {
            return Computable::one();
        }
        let mut value = Computable::pi();
        for _ in 1..power {
            value = value.multiply(Computable::pi());
        }
        value
    }

    /// Use One for exponent zero, Pi for one, otherwise PiPow with the positive-power graph.
    fn make_pi_power(power: u8) -> (Class, Computable) {
        match power {
            0 => (One, Computable::one()),
            1 => (Pi, Computable::pi()),
            _ => (PiPow(power), Self::pi_power_computable(u16::from(power))),
        }
    }

    /// Delegate pi*exp(q) to constant-product normalization, allowing q=0 to collapse to Pi.
    fn make_pi_exp(br: Rational) -> (Class, Computable) {
        Self::make_const_product(1, br)
    }

    /// Construct pi^n*exp(q) using dedicated unit, exponential, small positive power and reciprocal forms first.
    /// Then prefer PiExp/PiInvExp for n=+/-1, otherwise retain boxed parameters and an exact graph.
    /// Zero q omits the exponential graph; i16 exponent bounds do not bound subsequent evaluation work.
    fn make_const_product(pi_power: i16, exp_power: Rational) -> (Class, Computable) {
        // Normalize back to the smaller dedicated variants whenever possible.
        // Rarer signed pi powers stay boxed; direct `pi`, `1/pi`, `e^q`,
        // and one-pi-factor products have smaller dedicated variants.
        if pi_power == 0 {
            return Self::make_exp(exp_power);
        }
        if exp_power == *rationals::ZERO {
            if let Ok(power) = u8::try_from(pi_power) {
                return Self::make_pi_power(power);
            }
            if pi_power == -1 {
                return (PiInv, Computable::pi().inverse());
            }
        }
        if pi_power == 1 {
            return (
                PiExp(exp_power.clone()),
                Computable::pi().multiply(Computable::exp_rational(exp_power)),
            );
        }
        if pi_power == -1 {
            return (
                PiInvExp(exp_power.clone()),
                Computable::pi()
                    .inverse()
                    .multiply(Computable::exp_rational(exp_power)),
            );
        }
        let pi = Self::signed_pi_power_computable(pi_power);
        let computable = if exp_power == *rationals::ZERO {
            pi
        } else {
            pi.multiply(Computable::exp_rational(exp_power.clone()))
        };
        (
            ConstProduct(Box::new(ConstProductClass {
                pi_power,
                exp_power: exp_power.clone(),
            })),
            computable,
        )
    }

    /// Cheap sufficient positivity test for pi^n*exp(q)+offset. Nonnegative offsets always pass.
    /// For negative offsets, admit only pi-minus-threshold with threshold <=3 or e-minus-threshold <=2.
    /// False means this recognizer declined; it is not a proof of nonpositivity.
    fn const_offset_positive_certified(
        pi_power: i16,
        exp_power: &Rational,
        offset: &Rational,
    ) -> bool {
        // `ConstOffset` participates in the same zero/sign fast path as the
        // purely multiplicative classes, so only construct it when the inner
        // value is cheaply certified positive. This deliberately covers the hot
        // `pi - 3` and `e - 2` style reductions and leaves near-cancellation
        // cases on the generic computable path.
        if offset.sign() != Sign::Minus {
            return true;
        }
        let threshold = -offset.clone();
        if exp_power == &*rationals::ZERO && pi_power == 1 {
            threshold <= Rational::new(3)
        } else if exp_power == &*rationals::ONE && pi_power == 0 {
            threshold <= Rational::new(2)
        } else {
            false
        }
    }

    /// Normalize zero offset to a constant product; decline nonzero pure-rational offsets or unproved positivity.
    /// Otherwise retain the exact sum with a positive-basis certificate. None selects caller fallback, not invalid input.
    fn make_const_offset(
        pi_power: i16,
        exp_power: Rational,
        offset: Rational,
    ) -> Option<(Class, Computable)> {
        if offset == *rationals::ZERO {
            return Some(Self::make_const_product(pi_power, exp_power));
        }
        if pi_power == 0 && exp_power == *rationals::ZERO {
            return None;
        }
        if !Self::const_offset_positive_certified(pi_power, &exp_power, &offset) {
            return None;
        }
        let (_, constant) = Self::make_const_product(pi_power, exp_power.clone());
        let computable = Computable::add(constant, Computable::rational(offset.clone()));
        Some((
            ConstOffset(Box::new(ConstOffsetClass {
                pi_power,
                exp_power,
                offset,
            })),
            computable,
        ))
    }

    /// Construct nonnegative pi powers directly; negative powers invert the positive unsigned magnitude.
    /// unsigned_abs handles i16::MIN without signed negation overflow; the basis pi is nonzero.
    fn signed_pi_power_computable(power: i16) -> Computable {
        // Negative pi powers are represented by inverting the positive cached
        // pi-chain. This is cheaper than creating an opaque reciprocal product
        // and lets later multiplication cancel back to the exact classes.
        if power >= 0 {
            return Self::pi_power_computable(power as u16);
        }
        Self::pi_power_computable(power.unsigned_abs()).inverse()
    }

    /// Extract (signed pi power, rational e exponent) from the eight admitted multiplicative variants.
    /// Clone exponent data as needed; None is a representation mismatch, not proof of a different mathematical family.
    fn const_product_parts(&self) -> Option<(i16, Rational)> {
        // A lightweight deconstructor for the pi^n * e^q family. This is the
        // single point that lets newer classes (`1/pi`, `e^q/pi`) participate
        // in multiplication/division reductions without growing more arms.
        match self {
            One => Some((0, Rational::zero())),
            Pi => Some((1, Rational::zero())),
            PiPow(power) => Some((i16::from(*power), Rational::zero())),
            PiInv => Some((-1, Rational::zero())),
            Exp(exp) => Some((0, exp.clone())),
            PiExp(exp) => Some((1, exp.clone())),
            PiInvExp(exp) => Some((-1, exp.clone())),
            ConstProduct(product) => Some((product.pi_power, product.exp_power.clone())),
            _ => None,
        }
    }

    /// Extract an explicit offset or extend a recognized multiplicative form with exact zero offset.
    fn const_offset_parts(&self) -> Option<(i16, Rational, Rational)> {
        // Treat plain constant products as offset-zero values. This lets
        // rational addition/subtraction share one normalization path while the
        // zero-offset case normalizes back to the smaller multiplicative class.
        match self {
            ConstOffset(offset) => Some((
                offset.pi_power,
                offset.exp_power.clone(),
                offset.offset.clone(),
            )),
            _ => {
                let (pi_power, exp_power) = self.const_product_parts()?;
                Some((pi_power, exp_power, Rational::zero()))
            }
        }
    }

    /// Cheap tag gate for pi/exponential forms and existing offsets; exclude One and unrelated root/log/trig forms.
    fn can_take_const_offset(&self) -> bool {
        // Gate the offset probe so ordinary sqrt/log/trig additions do not pay
        // for a const-product deconstruction they can never use. This gate was
        // added after borrowed matrix multiplication benchmarks caught the cost.
        matches!(
            self,
            Pi | PiPow(_)
                | PiInv
                | Exp(_)
                | PiExp(_)
                | PiInvExp(_)
                | ConstProduct(_)
                | ConstOffset(_)
        )
    }

    /// Decompose both bases, checked-subtract their i16 pi powers, subtract rational e exponents and normalize.
    /// None reports an unmatched representation or exponent overflow so the caller can fall back; no certificate wraps.
    fn divide_const_products(left: &Class, right: &Class) -> Option<(Class, Computable)> {
        // General quotient closure for symbolic constants. The checked integer
        // arithmetic is a guardrail for rare constructed powers; overflow falls
        // back to the generic computable path rather than wrapping a certificate.
        let (left_pi, left_exp) = left.const_product_parts()?;
        let (right_pi, right_exp) = right.const_product_parts()?;
        let pi_power = left_pi.checked_sub(right_pi)?;
        Some(Self::make_const_product(pi_power, left_exp - right_exp))
    }

    /// Decompose both bases, checked-add their i16 pi powers, add rational e exponents and normalize.
    /// None preserves caller fallback on unmatched forms or finite-tag overflow.
    fn multiply_const_products(left: &Class, right: &Class) -> Option<(Class, Computable)> {
        // Product closure mirrors division and is kept behind the exact special
        // forms in `Mul`; this retains very small hot arms while still catching
        // less common signed-power products.
        let (left_pi, left_exp) = left.const_product_parts()?;
        let (right_pi, right_exp) = right.const_product_parts()?;
        let pi_power = left_pi.checked_add(right_pi)?;
        Some(Self::make_const_product(pi_power, left_exp + right_exp))
    }

    /// Pair pi*sqrt(r) with retained PiSqrt parameters; the caller must supply a positive rational radicand.
    /// This helper performs no separate admission or square-factor normalization.
    fn make_pi_sqrt(r: Rational) -> (Class, Computable) {
        // `pi * sqrt(n)` appears in algebra kernels; keeping it symbolic lets
        // repeated products collapse before falling back to generic computables.
        (
            PiSqrt(r.clone()),
            Computable::pi().multiply(Computable::sqrt_rational(r)),
        )
    }

    /// Reuse shared sqrt(2)/sqrt(3) constants after exact i64 recognition; otherwise build the rational-root graph.
    /// The caller supplies a valid radicand; this helper does not prove positivity.
    fn const_product_sqrt_computable(radicand: &Rational) -> Computable {
        // Composite `pi^n*e^q*sqrt(r)` values are often cloned through matrix
        // rows. For the dominant square-free residuals 2 and 3, reuse the
        // canonical shared sqrt computable so costly approximations are warmed
        // once and then multiplied by the symbolic pi/e factors. Plain `sqrt(2)`
        // and `pi*sqrt(2)` stay on their direct constructors above; those
        // tighter one-node paths benchmark faster for isolated scalar use.
        radicand
            .to_integer_i64()
            .and_then(Computable::sqrt_constant)
            .unwrap_or_else(|| Computable::sqrt_rational(radicand.clone()))
    }

    /// Normalize radicand one, then prefer plain Sqrt or PiSqrt when the pi/e factors permit it.
    /// Otherwise retain the boxed factored product and root graph. Callers own positive-radicand
    /// and nonsquare plain-Sqrt invariants; this routine does not extract arbitrary square factors.
    fn make_const_product_sqrt(
        pi_power: i16,
        exp_power: Rational,
        radicand: Rational,
    ) -> (Class, Computable) {
        // This is a factored positive class: the rational scale carries all sign
        // information, while the inner pi/e/sqrt product stays non-zero. Keeping
        // the common `pi*sqrt(n)` and plain `sqrt(n)` variants avoids regressing
        // the tight scalar arms.
        if radicand == *rationals::ONE {
            return Self::make_const_product(pi_power, exp_power);
        }
        if pi_power == 0 && exp_power == *rationals::ZERO {
            return (Sqrt(radicand.clone()), Computable::sqrt_rational(radicand));
        }
        if pi_power == 1 && exp_power == *rationals::ZERO {
            return Self::make_pi_sqrt(radicand);
        }
        let (_, constant) = Self::make_const_product(pi_power, exp_power.clone());
        let computable = constant.multiply(Self::const_product_sqrt_computable(&radicand));
        (
            ConstProductSqrt(Box::new(ConstProductSqrtClass {
                pi_power,
                exp_power,
                radicand,
            })),
            computable,
        )
    }

    /// Extract explicit root-factor parameters from Sqrt, PiSqrt or ConstProductSqrt only.
    /// Do not treat an implicit unit root in a plain product as an explicit factor.
    fn const_product_sqrt_parts(&self) -> Option<(i16, Rational, Rational)> {
        // Deconstructor for classes that contain an explicit sqrt factor. It is
        // intentionally narrower than `const_product_parts` so hot pi/e products
        // do not pay sqrt-specific reductions.
        match self {
            Sqrt(radicand) => Some((0, Rational::zero(), radicand.clone())),
            PiSqrt(radicand) => Some((1, Rational::zero(), radicand.clone())),
            ConstProductSqrt(product) => Some((
                product.pi_power,
                product.exp_power.clone(),
                product.radicand.clone(),
            )),
            _ => None,
        }
    }

    /// Recognize exactly the three tags handled by the root-product deconstructor.
    fn has_const_product_sqrt_factor(&self) -> bool {
        // Cheap prefilter before calling the heavier sqrt-product deconstructor
        // in multiply/divide fallbacks.
        matches!(self, Sqrt(_) | PiSqrt(_) | ConstProductSqrt(_))
    }

    /// Reuse shared log constants for exact integer bases 2, 3, 5, 6, 7 and 10; otherwise build ln(base).
    /// Callers supply a positive rational base. The unwrap follows the enumerated constant table invariant.
    fn ln_computable(base: &Rational) -> Computable {
        // Common logarithm constants share Computable caches. Dense symbolic
        // expressions repeatedly build ln(2), ln(3), etc.; routing them here
        // avoids independent approximation caches for the same mathematical value.
        // The integer extraction is deliberately structural and non-approximating:
        // keep exact reduction in front of numerical refinement.
        if let Some(base) = base.to_integer_i64() {
            match base {
                2 | 3 | 5 | 6 | 7 | 10 => return Computable::ln_constant(base as u32).unwrap(),
                _ => {}
            }
        }
        Computable::rational(base.clone()).ln()
    }

    /// Build exp(exponent*ln(base)) exactly, assuming the caller supplies a positive rational base.
    /// The helper does not choose root specializations or validate that precondition.
    fn rational_base_power_computable(base: &Rational, exponent: &Rational) -> Computable {
        Self::ln_computable(base)
            .multiply(Computable::rational(exponent.clone()))
            .exp()
    }

    /// Admit only base>1 and nonnegative offset; zero offset collapses to Ln.
    /// Other sums return None even when mathematically positive, leaving caller fallback available.
    fn make_ln_affine(offset: Rational, base: Rational) -> Option<(Class, Computable)> {
        // Like `ConstOffset`, this additive form is only admitted when the
        // inner value is structurally positive. That keeps sign and zero queries
        // as cheap as other exact classes while avoiding a generic add node for
        // ln(a*e^q) with positive q.
        if base <= *rationals::ONE || offset.sign() == Sign::Minus {
            return None;
        }
        if offset == *rationals::ZERO {
            return Some((Ln(base.clone()), Self::ln_computable(&base)));
        }
        let computable = Computable::add(
            Computable::rational(offset.clone()),
            Self::ln_computable(&base),
        );
        Some((
            LnAffine(Box::new(LnAffineClass { offset, base })),
            computable,
        ))
    }

    /// Sort bases using exact rational order and retain ln(left)*ln(right) with the matching graph.
    /// Callers must supply positive nonunit bases with same-sign logarithms; sorting is not domain validation.
    fn make_ln_product(left: Rational, right: Rational) -> (Class, Computable) {
        // Sort factors so ln(a)*ln(b) and ln(b)*ln(a) compare equal and share
        // downstream structural facts.
        let (left, right) = if left <= right {
            (left, right)
        } else {
            (right, left)
        };
        (
            LnProduct(Box::new(LnProductClass {
                left: left.clone(),
                right: right.clone(),
            })),
            Self::ln_computable(&left).multiply(Self::ln_computable(&right)),
        )
    }

    /// Reconstruct the unscaled named basis from retained parameters, reusing constant helpers where available.
    /// This does not validate parameters, include the outer Real scale or attach its abort signal.
    /// Opaque Irrational lacks a recipe and panics: its owning Real must retain a payload.
    /// Graphs/allocations may grow with stored exponents; no total resource budget is supplied.
    pub(super) fn computable_certificate(&self) -> Computable {
        // Materialize a missing computable payload from its exact symbolic
        // certificate. Existing immutable payloads can be shared by cloning.
        match self {
            One => Computable::one(),
            Pi => Computable::pi(),
            PiPow(power) => Self::pi_power_computable(u16::from(*power)),
            PiInv => Computable::pi().inverse(),
            Exp(exp) => Computable::exp_rational(exp.clone()),
            PiExp(exp) => Computable::pi().multiply(Computable::exp_rational(exp.clone())),
            PiInvExp(exp) => Computable::pi()
                .inverse()
                .multiply(Computable::exp_rational(exp.clone())),
            ConstProduct(product) => {
                let pi = Self::signed_pi_power_computable(product.pi_power);
                if product.exp_power == *rationals::ZERO {
                    pi
                } else {
                    pi.multiply(Computable::exp_rational(product.exp_power.clone()))
                }
            }
            ConstOffset(offset) => {
                let constant =
                    Self::make_const_product(offset.pi_power, offset.exp_power.clone()).1;
                Computable::add(constant, Computable::rational(offset.offset.clone()))
            }
            Sqrt(radicand) => Computable::sqrt_squarefree_rational(radicand.clone()),
            PiSqrt(radicand) => {
                Computable::pi().multiply(Computable::sqrt_squarefree_rational(radicand.clone()))
            }
            ConstProductSqrt(product) => {
                let constant =
                    Self::make_const_product(product.pi_power, product.exp_power.clone()).1;
                constant.multiply(Computable::sqrt_squarefree_rational(
                    product.radicand.clone(),
                ))
            }
            Ln(base) => Self::ln_computable(base),
            LnAffine(affine) => Computable::add(
                Computable::rational(affine.offset.clone()),
                Self::ln_computable(&affine.base),
            ),
            LnProduct(product) => {
                Self::ln_computable(&product.left).multiply(Self::ln_computable(&product.right))
            }
            Log10(base) => {
                Self::ln_computable(base).multiply(Self::ln_computable(&rationals::TEN).inverse())
            }
            Log2(base) => {
                Self::ln_computable(base).multiply(Self::ln_computable(&rationals::TWO).inverse())
            }
            Pow10(exponent) => Self::rational_base_power_computable(&rationals::TEN, exponent),
            Pow2(exponent) => Self::rational_base_power_computable(&rationals::TWO, exponent),
            SinPi(rational) => {
                let argument =
                    Computable::multiply(Computable::pi(), Computable::rational(rational.clone()));
                Computable::prescaled_sin(argument)
            }
            TanPi(rational) => {
                let argument =
                    Computable::multiply(Computable::pi(), Computable::rational(rational.clone()));
                Computable::prescaled_tan(argument)
            }
            Irrational => panic!("opaque irrational classes must carry their Computable payload"),
        }
    }

    #[cfg(feature = "serde")]
    /// Check named-basis domain and cheap fact invariants without requiring native canonicalization.
    /// Admit equivalent constant products, perfect PiSqrt radicands, unsorted same-sign log pairs
    /// and extra irrational special-angle forms when valid. Reject invalid shape parameters with ParseError.
    /// Additive positivity may require certificate-only refinement and return UnknownCertificate.
    /// Payload presence, domains and agreement are separate RealWire obligations; this check alone does not admit a Real.
    fn validate_serialized_parameters(&self) -> Result<(), Problem> {
        let positive_root = |r: &Rational| r.sign() == Sign::Plus;
        let irrational_root = |r: &Rational| positive_root(r) && r.perfect_nth_root(2).is_none();
        let proper_log = |r: &Rational| r > &*rationals::ONE;
        let interior_angle = |r: &Rational| r.sign() == Sign::Plus && r < &*rationals::HALF;
        let valid = match self {
            One | Pi | PiInv | PiExp(_) | PiInvExp(_) | ConstProduct(_) | Irrational => true,
            PiPow(power) => *power >= 2,
            PiSqrt(r) => positive_root(r),
            Sqrt(r) => irrational_root(r),
            Exp(r) => r.sign() != Sign::NoSign,
            Ln(r) => proper_log(r),
            Log10(r) => proper_log(r) && Real::rational_integer_log(r, 10).is_none(),
            Log2(r) => proper_log(r) && Real::rational_integer_log(r, 2).is_none(),
            Pow10(r) | Pow2(r) => !r.is_integer(),
            SinPi(r) => interior_angle(r) && r != &*rationals::SIXTH,
            TanPi(r) => interior_angle(r) && r != &*rationals::QUARTER,
            ConstProductSqrt(product) => {
                positive_root(&product.radicand)
                    && (product.pi_power != 0
                        || product.exp_power.sign() != Sign::NoSign
                        || irrational_root(&product.radicand))
            }
            LnProduct(product) => {
                product.left.sign() == Sign::Plus
                    && product.right.sign() == Sign::Plus
                    && !product.left.is_one()
                    && !product.right.is_one()
                    && (proper_log(&product.left) == proper_log(&product.right))
            }
            LnAffine(affine) => {
                if affine.base.sign() != Sign::Plus || affine.base.is_one() {
                    false
                } else if proper_log(&affine.base) && affine.offset.sign() != Sign::Minus {
                    true
                } else {
                    return self.validate_serialized_positive_sum();
                }
            }
            ConstOffset(offset) => {
                if Self::const_offset_positive_certified(
                    offset.pi_power,
                    &offset.exp_power,
                    &offset.offset,
                ) {
                    true
                } else {
                    return self.validate_serialized_positive_sum();
                }
            }
        };
        if valid {
            Ok(())
        } else {
            Err(Problem::ParseError)
        }
    }

    #[cfg(feature = "serde")]
    /// Reconstruct from rational class parameters alone and require a positive sign at precision floor -256.
    /// Zero/negative returns ParseError, unresolved sign UnknownCertificate. Never trust the supplied
    /// wire payload as its own certificate; precision is bounded here, total evaluation work is not.
    fn validate_serialized_positive_sum(&self) -> Result<(), Problem> {
        // Only construct from already defined rational parameters. Never refine
        // an untrusted supplied payload to validate its own certificate. Unlike
        // rejecting every noncanonical sum, an undecided sign preserves the
        // distinction between a missing proof and a mathematically invalid tag.
        match self.computable_certificate().sign_until(-256) {
            Some(RealSign::Positive) => Ok(()),
            Some(RealSign::Negative | RealSign::Zero) => Err(Problem::ParseError),
            None => Err(Problem::UnknownCertificate),
        }
    }
}
