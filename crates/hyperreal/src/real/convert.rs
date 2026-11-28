//! Exact numeric ingress, retained-expression folding and explicit primitive-float egress.
//!
//! Integer and finite IEEE inputs keep their exact values; decimal-looking float
//! literals retain their binary fractions, so 0.1_f64 is not the rational 1/10.
//! Float export is a separate boundary. Owned From conversions use guarded
//! significand rounding and can return infinities; borrowed lossy methods return
//! finite views or None and can return zero when a sign remains unresolved.
//! Neither route is a geometric predicate or a directed enclosure.
//!
//! Primitive caches are views of the retained Real, not replacements for it.
//! Binary64 cache use is unconditional; feature gates primarily affect binary32
//! caching/reuse here. Precision floors and guard-bit schedules do not bound
//! total evaluation work or memory. Attached abort signals are transported to
//! computable evaluations, but cancellation has no typed result in these APIs:
//! the Real::abort contract permits incorrect interrupted approximations.

use crate::RealSign;
use crate::real::{Class, PrimitiveApproxCache};
use crate::{Computable, Problem, Rational, Real};

/// Generate exact ingress for each listed signed/unsigned fixed-width integer type.
/// Zero and one use dedicated Real constructors; other values delegate to Rational.
macro_rules! impl_integer_conversion {
    ($T:ty) => {
        /// Preserve the integer value without passing through a floating-point representation.
        impl From<$T> for Real {
            /// Recognize zero, then one, then wrap the scalar owner's exact integer conversion.
            #[inline]
            fn from(n: $T) -> Real {
                // Integer identity conversion is a public hot path through
                // hyperlattice and hyperlimit. Keep 0 and 1 on dedicated
                // Real constructors instead of paying BigInt rational import.
                if n == 0 {
                    return Real::zero();
                }
                if n == 1 {
                    return Real::one();
                }
                // Let Rational pick the signed/unsigned primitive path; it
                // avoids materializing a BigInt just to build Real::new.
                Real::new(Rational::from(n))
            }
        }
    };
}

impl_integer_conversion!(i8);
impl_integer_conversion!(i16);
impl_integer_conversion!(i32);
impl_integer_conversion!(i64);
impl_integer_conversion!(i128);
impl_integer_conversion!(u8);
impl_integer_conversion!(u16);
impl_integer_conversion!(u32);
impl_integer_conversion!(u64);
impl_integer_conversion!(u128);

/// Transfer an exact rational into the native rational Real representation.
impl From<Rational> for Real {
    /// Delegate to Real::new without floating projection or symbolic fitting.
    fn from(rational: Rational) -> Real {
        Real::new(rational)
    }
}

/// Import a finite binary32 value as its exact dyadic rational, including subnormals.
/// Mathematical zero has no sign, but the seeded binary64 view retains input zero bits.
impl TryFrom<f32> for Real {
    /// Infinity and NotANumber are preserved from the rational decoder.
    type Error = Problem;

    /// Decode exactly, mark binary64 representability, and seed the widened input view.
    /// Every finite f32 widens exactly to f64; no decimal interpretation is involved.
    fn try_from(n: f32) -> Result<Real, Self::Error> {
        // Import floats as exact dyadic rationals. That preserves structural
        // facts and avoids immediately lowering user data to Computable nodes.
        let rational: Rational = n.try_into()?;
        rational.mark_exact_f64_view();
        let real = Real::new(rational);
        real.primitive_approx_cache
            .set(PrimitiveApproxCache::F64(Some(f64::from(n))));
        Ok(real)
    }
}

/// Import a finite binary64 value exactly; reject infinities and NaNs with Problem.
impl TryFrom<f64> for Real {
    /// Scalar import errors, distinguishing Infinity from NotANumber.
    type Error = Problem;

    /// Retain the rational decoder's exact-origin fact and seed the original binary64 bits.
    /// The mathematical zero is canonical even when the view preserves negative zero.
    fn try_from(n: f64) -> Result<Real, Self::Error> {
        // Same exact dyadic import as f32; the public Real constructor keeps the
        // value in the rational class so matrix inputs stay cheap.
        let rational: Rational = n.try_into()?;
        let real = Real::new(rational);
        real.primitive_approx_cache
            .set(PrimitiveApproxCache::F64(Some(n)));
        Ok(real)
    }
}

/// Materialize the full exact value as a computable graph when a generic kernel needs it.
impl Real {
    /// Borrow the value, combine its rational scale with its class only as needed,
    /// and reattach any abort signal. Unit scale reuses the computable certificate;
    /// the rational class becomes a rational node, otherwise multiply the class by
    /// its exact scale. Source meaning is unchanged; caches/graph allocation can occur.
    #[inline]
    pub(crate) fn fold_ref(&self) -> Computable {
        use crate::real::Class;

        // Keep the rational scale separate until a generic computable kernel is
        // unavoidable. Folding `a * class` eagerly would erase exact classes
        // that sign, sqrt, log, and trig shortcuts can still exploit.
        let mut c = if self.rational.is_one() {
            self.computable_clone()
        } else if self.class == Class::One {
            Computable::rational(self.rational.clone())
        } else {
            self.computable_clone()
                .multiply_rational(self.rational.clone())
        };

        if let Some(s) = self.abort_signal() {
            c.abort(s.clone());
        }
        c
    }

    /// Consume the same scaled definition, moving its payload where possible.
    /// An absent payload is reconstructed from the class certificate, not replaced
    /// by one. Preserve the payload's abort signal and discard the primitive view.
    #[inline]
    pub(crate) fn fold(self) -> Computable {
        // Owned folding mirrors `fold_ref` but moves the computable when the
        // rational scale is one; scalar transcendental kernels hit this path.
        // A missing payload still carries the symbolic class, not just one.
        let crate::Real {
            rational,
            class,
            computable,
            primitive_approx_cache: _,
        } = self;
        let signal = computable
            .as_ref()
            .and_then(|computable| computable.signal.as_ref())
            .cloned();
        let mut c = if rational.is_one() {
            computable.unwrap_or_else(|| class.computable_certificate())
        } else if class == crate::real::Class::One {
            Computable::rational(rational)
        } else {
            computable
                .unwrap_or_else(|| class.computable_certificate())
                .multiply_rational(rational)
        };
        if let Some(signal) = signal {
            c.abort(signal);
        }
        c
    }
}

use crate::computable::Precision;

/// Stably round a certified exact-real approximation to an unsigned binary integer.
///
/// `Computable::approx(p)` is within one integer unit of the value scaled by
/// `2^-p`. Evaluate below the destination precision and round both ends of that
/// closed error interval. Agreement certifies that rounding decision. Try 8, 24,
/// then 64 guard bits; an unresolved final transition chooses the even adjacent
/// integer without proving that the exact value is a midpoint. This deterministic
/// terminal is a lossy choice, not a universal correctly-rounded certificate.
///
/// Internal callers supply a float-sized significand and fitting precision shifts;
/// magnitude/rounded-result overflow panics at the checked u128/u64 conversions.
/// The three precision requests do not bound each computable evaluation's work,
/// and an aborted approximation is subject to the caller's cancellation contract.
#[inline(never)]
fn stable_rounded_magnitude(c: &Computable, precision: Precision) -> u64 {
    for guard in [8_u32, 24, 64] {
        let approximation = c.approx(precision - guard as Precision);
        let magnitude: u128 = approximation
            .magnitude()
            .try_into()
            .expect("float significand plus guard bits should fit in a u128");
        let truncated = magnitude >> guard;
        let halfway = 1_u128 << (guard - 1);
        let remainder = magnitude & ((halfway << 1) - 1);
        // For an even truncated integer the halfway value stays below the
        // transition; for an odd integer it rounds up. The approximation's
        // closed +/-1 interval is certified whenever both endpoints remain on
        // one side of that transition.
        let transition = halfway + u128::from(truncated & 1 == 0);
        if remainder + 1 < transition {
            return truncated
                .try_into()
                .expect("rounded float significand should fit in a u64");
        }
        if remainder.saturating_sub(1) >= transition {
            return (truncated + 1)
                .try_into()
                .expect("rounded float significand should fit in a u64");
        }
        if guard == 64 {
            return (truncated + u128::from(truncated & 1 != 0))
                .try_into()
                .expect("rounded float significand should fit in a u64");
        }
    }
    unreachable!()
}

/// Return binary32 fraction/exponent fields from guarded magnitude rounding.
/// Use the fixed -149 subnormal grid at msd <= -126; otherwise request msd-24
/// and re-evaluate after each rounding carry. The caller supplies an admitted MSD
/// estimate and handles sign/range; this helper neither reports uncertainty nor
/// creates a signed float. Exponent one handles subnormal-to-normal carry.
#[inline(never)]
fn sig_exp_32_stable(c: &Computable, mut msd: Precision) -> (u32, u32) {
    /// Mask of stored fraction bits; the implicit normal leading bit is excluded.
    const SIG_BITS: u32 = 0x007f_ffff;
    /// Carry threshold beyond the normal significand window, used to repair the exponent.
    const OVERSIZE: u32 = SIG_BITS.next_power_of_two() << 1;

    if msd <= -126 {
        // Subnormal output needs the fixed minimum precision, independent of
        // the discovered MSD.
        let sig = stable_rounded_magnitude(c, -149) as u32;
        // It is possible for that top bit to be set, so we're not a denormal
        if sig > SIG_BITS {
            (sig & SIG_BITS, 1)
        } else {
            (sig, 0)
        }
    } else {
        // Normal output requests just enough bits for the f32 significand, then
        // repairs the exponent if rounding carried into a new top bit.
        let mut sig = stable_rounded_magnitude(c, msd - 24) as u32;
        // Almost (but not quite) two orders of binary magnitude range
        while sig >= OVERSIZE {
            msd += 1;
            sig = stable_rounded_magnitude(c, msd - 24) as u32;
        }
        (sig & SIG_BITS, (126 + msd) as u32)
    }
}

/// Return binary32 fields using one approximation at the selected output scale.
/// The normal path repairs oversized significands by shifting the existing value;
/// it does not use guard-bit rounding or re-evaluate after carry. Subnormals use
/// precision -149. Internal caller preconditions bound the integer conversion;
/// violations panic. These fields are an approximate view, not a certified bound.
fn sig_exp_32(c: Computable, mut msd: Precision) -> (u32, u32) {
    /// Mask of stored fraction bits; the implicit normal leading bit is excluded.
    const SIG_BITS: u32 = 0x007f_ffff;
    /// Carry threshold beyond the normal significand window, used to repair the exponent.
    const OVERSIZE: u32 = SIG_BITS.next_power_of_two() << 1;

    if msd <= -126 {
        let sig = c
            .approx(-149)
            .magnitude()
            .try_into()
            .expect("Magnitude of the top bits should fit in a u32");
        if sig > SIG_BITS {
            (sig & SIG_BITS, 1)
        } else {
            (sig, 0)
        }
    } else {
        let mut sig: u32 = c
            .approx(msd - 24)
            .magnitude()
            .try_into()
            .expect("Magnitude of the top bits should fit in a u32");
        while sig >= OVERSIZE {
            msd += 1;
            sig >>= 1;
        }
        (sig & SIG_BITS, (126 + msd) as u32)
    }
}

/// Consume Real into a lossy binary32 value, including signed zero or infinity.
/// Guarded rounding has an even fallback at an unresolved final transition.
impl From<Real> for f32 {
    /// Fold the exact expression, search magnitude down to -150, then determine
    /// sign down to -2000. A missing magnitude gives signed zero when negativity
    /// is established; unresolved sign elsewhere gives positive zero. Oversize
    /// values saturate to signed infinity when sign is known. No failure/certainty
    /// object accompanies the bits, and cancellation can invalidate the approximation.
    fn from(r: Real) -> f32 {
        /// Primitive IEEE sign-bit mask, separate from exponent and fraction fields.
        const NEG_BITS: u32 = 0x8000_0000;
        /// Primitive IEEE exponent-field mask; trailing zeros locate its packing shift.
        const EXP_BITS: u32 = 0x7f80_0000;
        /// Mask of stored fraction bits; the implicit normal leading bit is excluded.
        const SIG_BITS: u32 = 0x007f_ffff;
        debug_assert_eq!(NEG_BITS + EXP_BITS + SIG_BITS, u32::MAX);

        let c = r.fold();
        let Some(msd) = c.iter_msd_stop(-150) else {
            // Below the f32 subnormal floor, round to signed zero.
            return if matches!(c.sign_until(-2000), Some(RealSign::Negative)) {
                -0.0
            } else {
                0.0
            };
        };
        if msd > 127 {
            // Above the finite f32 range, saturate to signed infinity.
            return match c.sign_until(-2000) {
                Some(RealSign::Negative) => f32::NEG_INFINITY,
                Some(RealSign::Positive) => f32::INFINITY,
                Some(RealSign::Zero) | None => 0.0,
            };
        }
        let (sig_bits, exp) = sig_exp_32_stable(&c, msd);
        let neg_bits = match c.sign_until(-2000) {
            Some(RealSign::Negative) => NEG_BITS,
            Some(RealSign::Positive) => 0,
            Some(RealSign::Zero) | None => return 0.0,
        };
        let exp_bits: u32 = exp << EXP_BITS.trailing_zeros();
        let bits = neg_bits | exp_bits | sig_bits;
        f32::from_bits(bits)
    }
}

/// Borrowed finite binary32 projections with optional primitive-view caching.
impl Real {
    /// Return a finite borrowed lossy `f32` approximation, or `None` on overflow.
    ///
    /// This is an explicit primitive-float edge for rendering, IO, display,
    /// statistics, and external-library adapters. It is not a certified sign,
    /// ordering, equality, or topology predicate. Call
    /// [`Real::certified_sign_until`] or the higher-level predicate crates when
    /// a decision needs proof, separating certified decisions from approximate
    /// numerical views.
    ///
    /// Some(0.0) can mean unresolved sign rather than a proved zero. The generic
    /// route searches sign/magnitude to -150 and uses unguarded significand bits;
    /// a plain rational uses a binary64 shortcut only away from binary32 ties,
    /// otherwise the rational owner's correctly-rounded conversion. There is no
    /// universal directed-error or correctly-rounded guarantee for symbolic inputs.
    ///
    /// With either cache feature, read reusable F64 views; cached-f32-approx also
    /// reads/writes F32 entries. F64(None) implies binary32 overflow. Finite F64
    /// proposals are reused only through the neighborhood test. Store F32 only
    /// into an empty, non-aborted cache; an existing F64 entry is not downgraded.
    #[inline]
    pub fn to_f32_lossy(&self) -> Option<f32> {
        #[cfg(any(feature = "cached-f32-approx", feature = "cached-f64-approx"))]
        match self.primitive_approx_cache.get() {
            #[cfg(feature = "cached-f32-approx")]
            PrimitiveApproxCache::F32(value) => return value,
            PrimitiveApproxCache::F64(None) => return None,
            PrimitiveApproxCache::F64(Some(value)) => {
                if let Some(value) = unambiguous_f32_from_f64(value) {
                    return value;
                }
            }
            PrimitiveApproxCache::Empty => {}
        }

        let value = self.to_f32_lossy_uncached();
        #[cfg(feature = "cached-f32-approx")]
        if !self.is_aborted()
            && matches!(
                self.primitive_approx_cache.get(),
                PrimitiveApproxCache::Empty
            )
        {
            self.primitive_approx_cache
                .set(PrimitiveApproxCache::F32(value));
        }
        value
    }

    /// Compute a binary32 projection without consulting the primitive cache first.
    /// Plain nonzero rationals attempt binary64 narrowing then exact rational tie
    /// resolution. In the f64-only feature configuration this can still store a
    /// proposal: "uncached" does not mean cache-pure. The generic path folds before
    /// sign refinement, preserves known underflow sign, maps unknown sign to +0,
    /// and rejects nonfinite assembled output. Abort is not a typed outcome.
    fn to_f32_lossy_uncached(&self) -> Option<f32> {
        /// Primitive IEEE sign-bit mask, separate from exponent and fraction fields.
        const NEG_BITS: u32 = 0x8000_0000;
        /// Primitive IEEE exponent-field mask; trailing zeros locate its packing shift.
        const EXP_BITS: u32 = 0x7f80_0000;

        if matches!(self.class, Class::One)
            && self.computable.is_none()
            && self.abort_signal().is_none()
            && let Some(value) = self.rational.to_f64_lossy()
            && value != 0.0
        {
            // Reuse direct rational binary64 conversion before narrowing at
            // this explicitly lossy IO boundary. Wide quotient rounding or
            // lazy canonicalization can allocate. Zero falls through so the general
            // path can retain the sign of an underflowed negative value.
            // Narrowing the correctly rounded binary64 value is exact unless
            // it sits at or next to a binary32 tie, where rounding twice could
            // pick the wrong neighbour; round the exact rational there.
            let narrowed = unambiguous_f32_from_f64(value)
                .unwrap_or_else(|| self.rational.to_f32_correctly_rounded());
            #[cfg(all(feature = "cached-f64-approx", not(feature = "cached-f32-approx")))]
            self.primitive_approx_cache
                .set(PrimitiveApproxCache::F64(Some(value)));
            return narrowed;
        }

        let c = self.fold_ref();
        let sign = match self.refine_sign_until(-150) {
            Some(sign) => sign,
            None => return Some(0.0),
        };
        let neg = match sign {
            RealSign::Zero => return Some(0.0),
            RealSign::Positive => 0,
            RealSign::Negative => 1,
        };

        let Some(msd) = c.iter_msd_stop(-150) else {
            return Some(match neg {
                0 => 0.0,
                1 => -0.0,
                _ => unreachable!(),
            });
        };
        if msd > 127 {
            return None;
        }
        let (sig_bits, exp) = sig_exp_32(c, msd);
        let neg_bits: u32 = neg << NEG_BITS.trailing_zeros();
        let exp_bits: u32 = exp << EXP_BITS.trailing_zeros();
        let bits = neg_bits | exp_bits | sig_bits;
        let value = f32::from_bits(bits);
        value.is_finite().then_some(value)
    }
}

/// Reuses a binary64 approximation only when its complete one-ULP neighborhood
/// maps to the same binary32 result. This rejects binary32 midpoints and their
/// immediate neighbors, where narrowing an approximate proposal could select
/// a different adjacent value. Signed zero deliberately falls through so the
/// exact sign can be retained.
/// Outer None declines reuse (including nonfinite input); Some(None) establishes
/// overflow of all three narrowed proposals; Some(Some(x)) supplies the common
/// finite result. This only examines neighboring proposals, not an exact Real.
fn unambiguous_f32_from_f64(value: f64) -> Option<Option<f32>> {
    if value == 0.0 || !value.is_finite() {
        return None;
    }
    let value32 = value as f32;
    let lower32 = value.next_down() as f32;
    let upper32 = value.next_up() as f32;
    (lower32.to_bits() == value32.to_bits() && upper32.to_bits() == value32.to_bits())
        .then(|| value32.is_finite().then_some(value32))
}

/// Binary64 counterpart of sig_exp_32_stable: fraction/exponent fields with guarded
/// rounding, a -1074 subnormal grid and msd-53 normal requests. Re-evaluate on
/// significand carry; the caller owns sign, range and cancellation interpretation.
#[inline(never)]
fn sig_exp_64_stable(c: &Computable, mut msd: Precision) -> (u64, u64) {
    /// Mask of stored fraction bits; the implicit normal leading bit is excluded.
    const SIG_BITS: u64 = 0x000f_ffff_ffff_ffff;
    /// Carry threshold beyond the normal significand window, used to repair the exponent.
    const OVERSIZE: u64 = SIG_BITS.next_power_of_two() << 1;

    if msd <= -1022 {
        // Subnormal f64 path mirrors f32 with the wider significand and lower
        // minimum precision.
        let sig = stable_rounded_magnitude(c, -1074);
        if sig > SIG_BITS {
            (sig & SIG_BITS, 1)
        } else {
            (sig, 0)
        }
    } else {
        // Normal f64 path requests 53 useful bits and handles one-bit carry from
        // rounding by shifting the significand and bumping the exponent.
        let mut sig = stable_rounded_magnitude(c, msd - 53);
        // Almost (but not quite) two orders of binary magnitude range
        while sig >= OVERSIZE {
            msd += 1;
            sig = stable_rounded_magnitude(c, msd - 53);
        }
        (sig & SIG_BITS, (1022 + msd) as u64)
    }
}

/// Binary64 fields from one approximate magnitude, repairing normal carry by shifts.
/// Use precision -1074 for subnormals or msd-53 for normal values. This is the
/// unguarded borrowed-export path; internal significand-fit violations panic.
#[inline(never)]
fn sig_exp_64(c: Computable, mut msd: Precision) -> (u64, u64) {
    /// Mask of stored fraction bits; the implicit normal leading bit is excluded.
    const SIG_BITS: u64 = 0x000f_ffff_ffff_ffff;
    /// Carry threshold beyond the normal significand window, used to repair the exponent.
    const OVERSIZE: u64 = SIG_BITS.next_power_of_two() << 1;

    if msd <= -1022 {
        let sig = c
            .approx(-1074)
            .magnitude()
            .try_into()
            .expect("Magnitude of the top bits should fit in a u64");
        if sig > SIG_BITS {
            (sig & SIG_BITS, 1)
        } else {
            (sig, 0)
        }
    } else {
        let mut sig: u64 = c
            .approx(msd - 53)
            .magnitude()
            .try_into()
            .expect("Magnitude of the top bits should fit in a u64");
        while sig >= OVERSIZE {
            msd += 1;
            sig >>= 1;
        }
        (sig & SIG_BITS, (1022 + msd) as u64)
    }
}

/// Consume Real into a lossy binary64 value through guarded computable rounding.
impl From<Real> for f64 {
    /// Search magnitude down to -1075 and sign down to -2000. Below the magnitude
    /// floor, retain negative zero only with known negative sign; above range,
    /// return signed infinity only with known sign. An unresolved sign maps to +0.
    /// This bypasses borrowed symbolic shortcuts and primitive-view caches and
    /// returns no typed cancellation, overflow or rounding-certainty information.
    fn from(r: Real) -> f64 {
        /// Primitive IEEE sign-bit mask, separate from exponent and fraction fields.
        const NEG_BITS: u64 = 0x8000_0000_0000_0000;
        /// Primitive IEEE exponent-field mask; trailing zeros locate its packing shift.
        const EXP_BITS: u64 = 0x7ff0_0000_0000_0000;
        /// Mask of stored fraction bits; the implicit normal leading bit is excluded.
        const SIG_BITS: u64 = 0x000f_ffff_ffff_ffff;
        debug_assert_eq!(NEG_BITS + EXP_BITS + SIG_BITS, u64::MAX);

        let c = r.fold();
        let Some(msd) = c.iter_msd_stop(-1075) else {
            // Too small for f64, including subnormal precision.
            return if matches!(c.sign_until(-2000), Some(RealSign::Negative)) {
                -0.0
            } else {
                0.0
            };
        };
        if msd > 1023 {
            // Too large for finite f64.
            return match c.sign_until(-2000) {
                Some(RealSign::Negative) => f64::NEG_INFINITY,
                Some(RealSign::Positive) => f64::INFINITY,
                Some(RealSign::Zero) | None => 0.0,
            };
        }
        let (sig_bits, exp) = sig_exp_64_stable(&c, msd);
        let neg_bits = match c.sign_until(-2000) {
            Some(RealSign::Negative) => NEG_BITS,
            Some(RealSign::Positive) => 0,
            Some(RealSign::Zero) | None => return 0.0,
        };
        let exp_bits: u64 = exp << EXP_BITS.trailing_zeros();
        let bits = neg_bits | exp_bits | sig_bits;
        f64::from_bits(bits)
    }
}

/// Exact representable binary64 views and separate lossy binary64 projections.
impl Real {
    /// Return an `f64` only when this value is an exact dyadic rational whose
    /// conversion is lossless.
    /// Requires recognized rational storage and finite exact representability,
    /// not just dyadic mathematical value or a cached approximate float. None is
    /// a declined exact view, not proof of irrationality. This may populate the
    /// rational's retained fact and Real's primitive cache. Attached cancellation
    /// remains subject to Real::abort, rather than a distinct result variant.
    pub fn to_f64_exact_dyadic(&self) -> Option<f64> {
        self.exact_dyadic_f64_cached()
    }

    /// Require a rational representation, then reuse its retained exact-f64 fact
    /// before inspecting denominator shape. Without that fact, reject definitely
    /// non-dyadic denominators and let Rational prove exact representability.
    /// Cache successful bits. A primitive cache entry alone is never the proof;
    /// the fact belongs to the immutable rational value, possibly shared by clones.
    #[inline]
    pub(crate) fn exact_dyadic_f64_cached(&self) -> Option<f64> {
        let rational = self.exact_rational_ref()?;
        // Primitive imports retain both their exact-origin fact and their
        // already-populated primitive cache. Check that hot path before
        // inspecting the denominator so certified geometry filters do not
        // repeatedly walk BigUint storage for values they have already seen.
        if rational.has_exact_f64_view() {
            return self.to_f64_lossy();
        }
        // Every non-unit dyadic denominator is even. Reject the common odd
        // rational schedules before attempting an exact conversion.
        if !rational.denominator_could_be_dyadic() {
            return None;
        }

        let value = rational.dyadic_to_f64_exact()?;
        self.primitive_approx_cache
            .set(PrimitiveApproxCache::F64(Some(value)));
        Some(value)
    }

    /// Return a finite borrowed lossy `f64` approximation, or `None` on overflow.
    ///
    /// This is an explicit primitive-float edge for rendering, IO, display,
    /// statistics, and external-library adapters. It may round, underflow to
    /// signed zero, or fail on overflow. It must not be used as a certified
    /// predicate or topology decision. For proof-producing sign information use
    /// [`Real::certified_sign_until`]; for geometric predicates use
    /// `hyperlimit`.
    ///
    /// The API name makes the approximation boundary visible, matching Yap's
    /// exact-computation rule that approximate values are views with proof
    /// obligations, not replacements for exact decisions.
    ///
    /// F64 cache lookup and storage occur even without cached-f64-approx. A miss
    /// can upgrade an F32 entry; store finite/overflow results only when not aborted.
    /// Symbolic shortcuts use ordinary host floating arithmetic. The generic path
    /// can return positive zero on unresolved sign or absent magnitude, so negative
    /// underflow sign is not uniformly preserved across all representations. No
    /// directed enclosure, universal nearest rounding or typed cancellation is promised.
    #[inline]
    pub fn to_f64_lossy(&self) -> Option<f64> {
        if let PrimitiveApproxCache::F64(value) = self.primitive_approx_cache.get() {
            return value;
        }

        let value = self.to_f64_lossy_uncached();
        if !self.is_aborted() {
            self.primitive_approx_cache
                .set(PrimitiveApproxCache::F64(value));
        }
        value
    }

    /// Try plain rational conversion, unit-scale symbolic constants, then scaled
    /// symbolic host-float formulas before the generic folded graph. Failed or
    /// nonfinite shortcuts fall through. Generic sign/magnitude searches stop at
    /// -1075; missing sign/magnitude returns +0, while overflow returns None.
    /// Does not read/write the primitive view cache, but scalar/node caches may
    /// still be populated. This method is neither a proof API nor an abort status.
    fn to_f64_lossy_uncached(&self) -> Option<f64> {
        /// Primitive IEEE sign-bit mask, separate from exponent and fraction fields.
        const NEG_BITS: u64 = 0x8000_0000_0000_0000;
        /// Primitive IEEE exponent-field mask; trailing zeros locate its packing shift.
        const EXP_BITS: u64 = 0x7ff0_0000_0000_0000;

        use crate::real::Class;

        if matches!(self.class, Class::One)
            && self.computable.is_none()
            && self.abort_signal().is_none()
            && let fast @ Some(_) = self.rational.to_f64_lossy()
        {
            // Exact rationals can often be rounded to f64 without touching the
            // lazy computable tree. This matters for matrix/predicate code that
            // asks for approximate centers of plain scalar data.
            return fast;
        }

        if self.rational.is_one() {
            let value = match &self.class {
                Class::Pi => Some(std::f64::consts::PI),
                Class::PiInv => Some(1.0 / std::f64::consts::PI),
                Class::PiPow(power) => Some(std::f64::consts::PI.powi(i32::from(*power))),
                Class::Sqrt(radicand) => radicand.to_f64_lossy().map(f64::sqrt),
                _ => None,
            };
            if let Some(value) = value
                && value.is_finite()
            {
                return Some(value);
            }
        } else if self.rational.is_two() && matches!(self.class, Class::Pi) {
            return Some(std::f64::consts::TAU);
        }

        if let Some(scale) = self.rational.to_f64_lossy() {
            let value = match &self.class {
                Class::Pi => Some(scale * std::f64::consts::PI),
                Class::PiInv => Some(scale / std::f64::consts::PI),
                Class::PiPow(power) => Some(scale * std::f64::consts::PI.powi(i32::from(*power))),
                Class::Exp(exp) => exp.to_f64_lossy().map(|exp| scale * exp.exp()),
                Class::Pow10(exp) => exp.to_f64_lossy().map(|exp| scale * 10_f64.powf(exp)),
                Class::Pow2(exp) => exp.to_f64_lossy().map(|exp| scale * 2_f64.powf(exp)),
                Class::PiExp(exp) => exp
                    .to_f64_lossy()
                    .map(|exp| scale * std::f64::consts::PI * exp.exp()),
                Class::PiInvExp(exp) => exp
                    .to_f64_lossy()
                    .map(|exp| scale / std::f64::consts::PI * exp.exp()),
                Class::Sqrt(radicand) => radicand
                    .to_f64_lossy()
                    .map(|radicand| scale * radicand.sqrt()),
                Class::PiSqrt(radicand) => radicand
                    .to_f64_lossy()
                    .map(|radicand| scale * std::f64::consts::PI * radicand.sqrt()),
                _ => None,
            };
            if let Some(value) = value
                && value.is_finite()
            {
                return Some(value);
            }
        }

        let c = self.fold_ref();
        let sign = match self.refine_sign_until(-1075) {
            // Borrowed conversion refuses to do unbounded refinement for sign.
            // Returning signed zero here keeps approximate-center users from
            // accidentally forcing exact evaluation of unresolved expressions.
            Some(sign) => sign,
            None => return Some(0.0),
        };
        let neg = match sign {
            RealSign::Zero => return Some(0.0),
            RealSign::Positive => 0,
            RealSign::Negative => 1,
        };

        let Some(msd) = c.iter_msd_stop(-1075) else {
            // Magnitude below f64's minimum representable scale.
            return Some(0.0);
        };
        if msd > 1023 {
            // Unlike `From<Real> for f64`, borrowed approximate conversion
            // reports overflow as None so callers can distinguish saturation.
            return None;
        }
        let (sig_bits, exp) = sig_exp_64(c, msd);
        let neg_bits: u64 = neg << NEG_BITS.trailing_zeros();
        let exp_bits: u64 = exp << EXP_BITS.trailing_zeros();
        let bits = neg_bits | exp_bits | sig_bits;
        let value = f64::from_bits(bits);
        value.is_finite().then_some(value)
    }
}

/// Exact ingress, float-range, rounding, cache and cancellation regression witnesses.
/// Property generators cover bounded rationals; MPFR supplies independent selected-case rounding.
#[cfg(test)]
mod tests {
    use num::bigint::ToBigInt;
    use num::{BigInt, One};
    #[cfg(any(feature = "cached-f32-approx", feature = "cached-f64-approx"))]
    use proptest::prelude::*;

    use super::*;

    #[cfg(any(feature = "cached-f32-approx", feature = "cached-f64-approx"))]
    /// Generate reduced rationals from bounded signed numerators and positive denominators.
    /// This finite-range distribution is not uniform over distinct rational values.
    fn finite_rational_strategy() -> impl Strategy<Value = Rational> {
        (-1_000_000_i64..=1_000_000, 1_u64..=1_000_000).prop_map(|(numerator, denominator)| {
            Rational::fraction(numerator, denominator).unwrap()
        })
    }

    #[cfg(feature = "cached-f64-approx")]
    /// Reject zero from the bounded rational strategy for negation-cache witnesses.
    fn nonzero_finite_rational_strategy() -> impl Strategy<Value = Rational> {
        finite_rational_strategy().prop_filter("nonzero rational", |r| !r.is_zero())
    }

    #[test]
    /// Check both positive IEEE zeros import as the exact Real zero; this does not compare zero sign bits.
    fn zero() {
        let f: f32 = 0.0;
        let d: f64 = 0.0;
        let a: Real = f.try_into().unwrap();
        let b: Real = d.try_into().unwrap();
        let zero = Real::zero();
        assert_eq!(a, zero);
        assert_eq!(b, zero);
    }

    #[test]
    /// Require typed Infinity errors for selected positive f32 and negative f64 infinities.
    fn infinity() {
        let f = f32::INFINITY;
        let d = f64::NEG_INFINITY;
        let a: Problem = <f32 as TryInto<Real>>::try_into(f).unwrap_err();
        let b: Problem = <f64 as TryInto<Real>>::try_into(d).unwrap_err();
        assert_eq!(a, Problem::Infinity);
        assert_eq!(b, Problem::Infinity);
    }

    #[test]
    /// Require typed NotANumber errors for the canonical NaN of each primitive width.
    fn nans() {
        let f = f32::NAN;
        let d = f64::NAN;
        let a: Problem = <f32 as TryInto<Real>>::try_into(f).unwrap_err();
        let b: Problem = <f64 as TryInto<Real>>::try_into(d).unwrap_err();
        assert_eq!(a, Problem::NotANumber);
        assert_eq!(b, Problem::NotANumber);
    }

    #[test]
    /// Require both owned exports to preserve exactly representable positive one-half.
    fn half_to_float() {
        let half = Real::new(Rational::fraction(1, 2).unwrap());
        let f: f32 = half.clone().into();
        let d: f64 = half.into();
        assert_eq!(f, 0.5);
        assert_eq!(d, 0.5);
    }

    #[test]
    /// Require both primitive ingress paths to produce the exact rational one-half.
    fn half_from_float() {
        let half = 0.5_f32;
        let correct = Real::new(Rational::fraction(1, 2).unwrap());
        let answer: Real = half.try_into().unwrap();
        assert_eq!(answer, correct);
        let half = 0.5_f64;
        let correct = Real::new(Rational::fraction(1, 2).unwrap());
        let answer: Real = half.try_into().unwrap();
        assert_eq!(answer, correct);
    }

    #[test]
    /// Require both owned exports to preserve the sign and value of negative one-half.
    fn negative_half() {
        let half = Real::new(Rational::fraction(-1, 2).unwrap());
        let f: f32 = half.clone().into();
        let d: f64 = half.into();
        assert_eq!(f, -0.5);
        assert_eq!(d, -0.5);
    }

    #[test]
    /// Check exact arithmetic after float ingress: imported 27 divided by 81 equals rational one-third.
    fn rational() {
        let f: f32 = 27.0;
        let d: f32 = 81.0;
        let a: Real = f.try_into().unwrap();
        let b: Real = d.try_into().unwrap();
        let third = Real::new(Rational::fraction(1, 3).unwrap());
        let answer = (a / b).unwrap();
        assert_eq!(answer, third);
    }

    #[test]
    /// Divide each least-positive subnormal by three and require owned export to underflow to zero.
    fn too_small() {
        let r: Real = f32::from_bits(1).try_into().unwrap();
        let s = r * Real::new(Rational::fraction(1, 3).unwrap());
        let f: f32 = s.into();
        assert_eq!(f, 0.0_f32);
        let r: Real = f64::from_bits(1).try_into().unwrap();
        let s = r * Real::new(Rational::fraction(1, 3).unwrap());
        let f: f64 = s.into();
        assert_eq!(f, 0.0_f64);
    }

    #[test]
    /// Check the exact dyadic fraction encoded by the selected binary32 decimal-looking literal.
    fn repr_f32() {
        let f: f32 = 1.234_567_9;
        let a: Real = f.try_into().unwrap();
        let correct = Real::new(Rational::fraction(5178153, 4194304).unwrap());
        assert_eq!(a, correct);
    }

    #[test]
    /// Check the exact dyadic fraction encoded by the selected binary64 decimal-looking literal.
    fn repr_f64() {
        let f: f64 = 1.23456789;
        let a: Real = f.try_into().unwrap();
        let correct = Real::new(Rational::fraction(5559999489367579, 4503599627370496).unwrap());
        assert_eq!(a, correct);
    }

    #[test]
    /// Check owned folding of nine at coarse then unit precision, including repeated cached approximation.
    fn fold_nine() {
        let nine = Real::new(Rational::new(9));
        let c = nine.fold();
        assert_eq!(c.approx(3), BigInt::one());
        let nine: BigInt = ToBigInt::to_bigint(&9).unwrap();
        assert_eq!(c.approx(0), nine);
    }

    #[test]
    /// Check mathematical positive-zero round trips through both widths, without bitwise signed-zero assertions.
    fn zero_roundtrip() {
        let zero = 0.0_f32;
        let zero: Real = zero.try_into().unwrap();
        assert_eq!(zero, Real::zero());
        let zero: f32 = zero.into();
        assert_eq!(zero, 0.0);
        let zero = 0.0_f64;
        let zero: Real = zero.try_into().unwrap();
        assert_eq!(zero, Real::zero());
        let zero: f64 = zero.into();
        assert_eq!(zero, 0.0);
    }

    /// Import through TryInto and export through owned From; panic if the selected input fails import.
    fn roundtrip<T>(f: T) -> T
    where
        T: TryInto<Real> + From<Real>,
        <T as TryInto<Real>>::Error: std::fmt::Debug,
    {
        let mid: Real = f.try_into().unwrap();
        mid.into()
    }

    #[test]
    /// Require the largest positive and negative finite values of both widths to survive owned round trips.
    fn big_roundtrip() {
        assert_eq!(f32::MAX, roundtrip(f32::MAX));
        assert_eq!(f64::MAX, roundtrip(f64::MAX));
        assert_eq!(f32::MIN, roundtrip(f32::MIN));
        assert_eq!(f64::MIN, roundtrip(f64::MIN));
    }

    #[test]
    /// Require each minimum positive normal and its triple to survive owned round trips.
    fn small_roundtrip() {
        assert_eq!(f32::MIN_POSITIVE * 3.0, roundtrip(f32::MIN_POSITIVE * 3.0));
        assert_eq!(f64::MIN_POSITIVE * 3.0, roundtrip(f64::MIN_POSITIVE * 3.0));
        assert_eq!(f32::MIN_POSITIVE, roundtrip(f32::MIN_POSITIVE));
        assert_eq!(f64::MIN_POSITIVE, roundtrip(f64::MIN_POSITIVE));
    }

    #[test]
    /// Check four selected decimal-looking/large finite literals through exact ingress and owned export.
    fn arbitrary_roundtrip() {
        assert_eq!(0.123_456_79_f32, roundtrip(0.123_456_79_f32));
        assert_eq!(987654321_f32, roundtrip(987654321_f32));
        assert_eq!(0.123456789_f64, roundtrip(0.123456789_f64));
        assert_eq!(987654321_f64, roundtrip(987654321_f64));
    }

    #[test]
    /// Check retained exact-origin evidence, cached input bits and lossless view reuse after cloning.
    fn exact_f64_import_seeds_and_reuses_the_certified_primitive_view() {
        let source = 0.123_456_789_f64;
        let value = Real::try_from(source).unwrap();
        assert!(value.rational.has_exact_f64_view());
        assert!(matches!(
            value.primitive_approx_cache.get(),
            PrimitiveApproxCache::F64(Some(cached)) if cached.to_bits() == source.to_bits()
        ));
        assert_eq!(
            value.exact_dyadic_f64_cached().map(f64::to_bits),
            Some(source.to_bits())
        );

        let cloned = value.clone();
        assert_eq!(
            cloned.exact_dyadic_f64_cached().map(f64::to_bits),
            Some(source.to_bits())
        );
        assert!(matches!(
            cloned.primitive_approx_cache.get(),
            PrimitiveApproxCache::F64(Some(cached)) if cached.to_bits() == source.to_bits()
        ));
    }

    #[test]
    /// Require one-third to decline the exact binary64 view without marking eligibility or filling the cache.
    fn certified_primitive_view_rejects_non_dyadic_rationals() {
        let value = Real::new(Rational::fraction(1, 3).unwrap());
        assert_eq!(value.exact_dyadic_f64_cached(), None);
        assert!(!value.rational.has_exact_f64_view());
        assert!(matches!(
            value.primitive_approx_cache.get(),
            PrimitiveApproxCache::Empty
        ));
    }

    #[test]
    /// Check formatting and owned round-trip equality for each greatest primitive value below two.
    fn almost_two() {
        // Largest f32 which is smaller than two
        let h = f32::from_bits(0x3fff_ffff);
        assert_eq!(format!("{h:#.7}"), "1.9999999");
        let r: Real = h.try_into().unwrap();
        assert_eq!(format!("{r:#.7}"), "1.9999999");
        let j: f32 = r.into();
        assert_eq!(h, j);
        // Largest f64 which is smaller than two
        let h = f64::from_bits(0x3fff_ffff_ffff_ffff);
        assert_eq!(format!("{h:#.16}"), "1.9999999999999998");
        let r: Real = h.try_into().unwrap();
        assert_eq!(format!("{r:#.16}"), "1.9999999999999998");
        let j: f64 = r.into();
        assert_eq!(h, j);
    }

    #[test]
    /// Require selected small and near-normal subnormals to retain all bits through owned conversion.
    fn subnormal_roundtrip() {
        let before = 1.234e-310_f64;
        assert_ne!(before, 0.0);
        assert_eq!(before, roundtrip(before));
        let before = 1.234e-41_f32;
        assert_ne!(before, 0.0);
        assert_eq!(before, roundtrip(before));
        // Large but still subnormal
        let sub = f32::from_bits(0x7c0000);
        assert_eq!(sub, roundtrip(sub));
        let sub = f64::from_bits(0x000f_ffff_0000_0000);
        assert_eq!(sub, roundtrip(sub));
    }

    // Sometimes during conversion the approximation fits without shifting, that's fine
    // but none of our other tests for f32 catch that
    #[test]
    /// Check a trigonometric binary32 result and warmed/cold owned bit agreement for it and negative atanh(1/2).
    fn bit_conversion() {
        let value = || {
            let r = Real::new(Rational::fraction(2, 5).unwrap()) * Real::pi();
            r.sin()
        };
        let f: f32 = value().into();
        assert_eq!(f, 0.951_056_54);

        let warmed = value();
        let c = warmed.fold_ref();
        assert_eq!(c.sign_until(-2000), Some(RealSign::Positive));
        assert_eq!(f32::from(warmed).to_bits(), f.to_bits());

        let negative_value = || {
            Real::new(Rational::fraction(-1, 2).unwrap())
                .atanh()
                .unwrap()
        };
        let expected = f64::from(negative_value()).to_bits();
        let warmed = negative_value();
        assert_eq!(
            warmed.fold_ref().sign_until(-2000),
            Some(RealSign::Negative)
        );
        assert_eq!(f64::from(warmed).to_bits(), expected);
    }

    // Our Pi isn't exactly equal to the IEEE approximations since it's more accurate
    #[test]
    /// Require owned pi exports to lie within one ULP of the standard primitive constants; not exact equality.
    fn pi() {
        let f: f32 = Real::pi().into();
        assert!(std::f32::consts::PI.to_bits().abs_diff(f.to_bits()) < 2);
        let f: f64 = Real::pi().into();
        assert!(std::f64::consts::PI.to_bits().abs_diff(f.to_bits()) < 2);
    }

    #[test]
    /// Check owned binary32 export of exact u64::MAX against the primitive integer cast.
    fn max_u64_f32() {
        let max_u64: Rational = u64::MAX.into();
        let r = Real::new(max_u64);
        let f: f32 = r.into();
        assert_eq!(f, u64::MAX as f32);
    }

    #[test]
    /// Check owned binary64 export of exact u64::MAX against the primitive integer cast.
    fn max_u64_f64() {
        let max_u64: Rational = u64::MAX.into();
        let r = Real::new(max_u64);
        let d: f64 = r.into();
        assert_eq!(d, u64::MAX as f64);
    }

    #[test]
    /// Check borrowed half/third values and a one-ULP pi comparison on selected finite inputs.
    fn borrowed_f64_lossy_finite_values() {
        let half = Real::new(Rational::fraction(1, 2).unwrap());
        assert_eq!(half.to_f64_lossy(), Some(0.5));

        let one_third = Real::new(Rational::fraction(1, 3).unwrap());
        assert_eq!(one_third.to_f64_lossy(), Some(1.0 / 3.0));

        let pi = Real::pi().to_f64_lossy().unwrap();
        assert!(std::f64::consts::PI.to_bits().abs_diff(pi.to_bits()) < 2);
    }

    #[test]
    /// Require repeated borrowed export bits to agree for zero, a rational, negative pi and sqrt(2).
    fn borrowed_lossy_float_exports_are_repeatable() {
        let values = [
            Real::from(0),
            Real::new(Rational::fraction(1, 7).unwrap()),
            -Real::pi(),
            Real::from(2).sqrt().unwrap(),
        ];

        for value in values {
            let first_f32 = value.to_f32_lossy().map(f32::to_bits);
            let second_f32 = value.to_f32_lossy().map(f32::to_bits);
            assert_eq!(first_f32, second_f32);

            let first_f64 = value.to_f64_lossy().map(f64::to_bits);
            let second_f64 = value.to_f64_lossy().map(f64::to_bits);
            assert_eq!(first_f64, second_f64);
        }
    }

    #[test]
    /// Use directed 4096-bit MPFR rational bounds that agree on binary64 rounding.
    /// Require exact selected subnormals, at most one ULP for other fixtures, correct sign,
    /// repeatability and exact-view reuse from a sibling rational; this is a finite corpus.
    fn borrowed_f64_preserves_values_with_large_denominators() {
        use num::{BigUint, One};
        use rug::{Float, Integer, Rational as GmpRational, float::Round, integer::Order};
        crate::mpfr_test::cleanup_on_thread_exit();

        let check = |numerator: BigUint, denominator: BigUint, exact_float: bool| {
            let exact = GmpRational::from((
                Integer::from_digits(&numerator.to_u64_digits(), Order::Lsf),
                Integer::from_digits(&denominator.to_u64_digits(), Order::Lsf),
            ));
            let lower = Float::with_val_round(4096, &exact, Round::Down).0.to_f64();
            let upper = Float::with_val_round(4096, &exact, Round::Up).0.to_f64();
            assert_eq!(
                lower.to_bits(),
                upper.to_bits(),
                "oracle must resolve rounding"
            );
            for negative in [false, true] {
                let mut rational = Rational::from_bigint_fraction(
                    BigInt::from(numerator.clone()),
                    denominator.clone(),
                )
                .unwrap();
                if negative {
                    rational = -rational;
                }
                let value = Real::new(rational.clone());
                let expected = if negative && lower != 0.0 {
                    -lower
                } else {
                    lower
                };
                let actual = value.to_f64_lossy().unwrap();
                // Borrowed exports may differ by one ULP from nearest rounding.
                // Exact subnormals must survive without losing any bits.
                assert!(
                    actual.abs().to_bits().abs_diff(expected.abs().to_bits())
                        <= u64::from(!exact_float),
                    "{value}: {actual:e} versus {expected:e}"
                );
                assert_eq!(actual.is_sign_negative(), negative);
                assert_eq!(
                    value.to_f64_lossy().map(f64::to_bits),
                    Some(actual.to_bits())
                );
                if exact_float {
                    // A sibling can retain the exact-view fact before this Real
                    // is built. Its certified export must still be lossless.
                    assert_eq!(rational.dyadic_to_f64_exact(), Some(expected));
                    assert_eq!(Real::new(rational).to_f64_exact_dyadic(), Some(expected));
                }
            }
        };

        for units in [1_u64, 2, 3, (1 << 26) + 1, 1 << 51, (1 << 52) - 1] {
            check(BigUint::from(units), BigUint::one() << 1074_usize, true);
        }
        let numerator = (BigUint::one() << 54_usize) - BigUint::one();
        let denominator = BigUint::one() << 1076_usize;
        check(numerator.clone(), denominator.clone(), false);
        check(numerator * 3_u8, denominator * 3_u8 + BigUint::one(), false);
        for exponent in [1000_usize, 1022, 1040, 1050, 1070, 1074] {
            check(BigUint::one(), (BigUint::one() << exponent) * 3_u8, false);
        }
        check(BigUint::one(), (BigUint::one() << 2000_usize) + 3_u8, false);
        for exponent in [1024_usize, 1050, 1074, 1500] {
            check(
                (BigUint::one() << 1000_usize) + BigUint::one(),
                (BigUint::one() << exponent) + BigUint::one(),
                false,
            );
        }
    }

    #[test]
    /// Check +/-2^-1200 underflow with signed zero and +/-2^1200 overflow as None on the rational path.
    fn borrowed_f64_lossy_underflow_and_overflow() {
        let tiny = Real::new(
            Rational::from_bigint_fraction(BigInt::from(1), num::BigUint::from(1_u8) << 1200)
                .unwrap(),
        );
        assert_eq!(tiny.to_f64_lossy(), Some(0.0));

        let negative_tiny = -tiny;
        assert_eq!(
            negative_tiny.to_f64_lossy().map(f64::to_bits),
            Some((-0.0_f64).to_bits())
        );

        let huge = Real::new(Rational::from_bigint(BigInt::from(1_u8) << 1200));
        assert_eq!(huge.to_f64_lossy(), None);

        let negative_huge = -huge;
        assert_eq!(negative_huge.to_f64_lossy(), None);
    }

    #[test]
    /// Check negative sqrt(2) retains sign and meets the selected numerical error bound.
    fn borrowed_f64_lossy_tracks_negative_finite_values() {
        let value = -(Real::new(Rational::new(2)).sqrt().unwrap());
        let approx = value.to_f64_lossy().unwrap();
        assert!(approx.is_sign_negative());
        assert!((approx + std::f64::consts::SQRT_2).abs() < 1e-15);
    }

    #[test]
    /// Check 11-sqrt(2) remains positive and numerically close despite a negative symbolic tail.
    fn borrowed_f64_lossy_keeps_positive_translation_over_negative_symbolic_tail() {
        let sqrt_two = Real::from(2).sqrt().unwrap();
        let value = Real::from(10) + (Real::from(1) - sqrt_two);

        let approx = value.to_f64_lossy().unwrap();

        assert!(approx.is_sign_positive());
        assert!((approx - (11.0 - std::f64::consts::SQRT_2)).abs() < 1e-12);
    }

    #[test]
    #[cfg(all(feature = "cached-f32-approx", feature = "cached-f64-approx"))]
    /// With both cache features, require pi to populate F32 then upgrade to F64 while retaining fixture accuracy.
    fn primitive_approx_cache_fills_and_upgrades() {
        let value = Real::pi();

        let f32_value = value.to_f32_lossy().unwrap();
        assert!(std::f32::consts::PI.to_bits().abs_diff(f32_value.to_bits()) < 2);
        assert!(matches!(
            value.primitive_approx_cache.get(),
            PrimitiveApproxCache::F32(Some(_))
        ));

        let f64_value = value.to_f64_lossy().unwrap();
        assert!(std::f64::consts::PI.to_bits().abs_diff(f64_value.to_bits()) < 2);
        assert!(matches!(
            value.primitive_approx_cache.get(),
            PrimitiveApproxCache::F64(Some(_))
        ));
    }

    #[test]
    #[cfg(feature = "cached-f64-approx")]
    /// Check ordinary narrowing, zero/tie declines and certain overflow in the three-proposal reuse helper.
    fn f64_neighborhood_only_reuses_unambiguous_f32_rounding() {
        assert_eq!(
            unambiguous_f32_from_f64(1.1_f64),
            Some(Some(1.1_f64 as f32))
        );
        assert_eq!(unambiguous_f32_from_f64(0.0), None);
        assert_eq!(unambiguous_f32_from_f64(-0.0), None);

        let lower = 1.0_f32;
        let upper = f32::from_bits(lower.to_bits() + 1);
        let midpoint = (f64::from(lower) + f64::from(upper)) * 0.5;
        assert_eq!(unambiguous_f32_from_f64(midpoint), None);
        assert_eq!(unambiguous_f32_from_f64(f64::MAX), Some(None));
    }

    #[test]
    #[cfg(all(feature = "cached-f64-approx", not(feature = "cached-f32-approx")))]
    /// In the f64-only configuration, require rational binary32 export to retain/reuse an F64 proposal.
    fn rational_f32_export_retains_its_f64_proposal() {
        let value = Real::new(Rational::fraction(1, 7).unwrap());
        let expected = value.to_f32_lossy();
        assert!(matches!(
            value.primitive_approx_cache.get(),
            PrimitiveApproxCache::F64(Some(_))
        ));
        assert_eq!(value.to_f32_lossy(), expected);
    }

    #[test]
    /// Require binary64 overflow to populate F64(None), including builds without cache features.
    fn primitive_approx_cache_keeps_overflow_state() {
        let huge = Real::new(Rational::from_bigint(BigInt::from(1_u8) << 1200));

        assert_eq!(huge.to_f64_lossy(), None);
        assert!(matches!(
            huge.primitive_approx_cache.get(),
            PrimitiveApproxCache::F64(None)
        ));
    }

    #[test]
    #[cfg(feature = "cached-f32-approx")]
    /// Require enabled binary32 caching to retain overflow as F32(None).
    fn primitive_approx_cache_keeps_f32_overflow_state() {
        let huge = Real::new(Rational::from_bigint(BigInt::from(1_u8) << 200));

        assert_eq!(huge.to_f32_lossy(), None);
        assert!(matches!(
            huge.primitive_approx_cache.get(),
            PrimitiveApproxCache::F32(None)
        ));
    }

    #[test]
    /// Attaching an unset abort flag must clear an existing primitive cache, before cancellation is requested.
    fn abort_invalidates_primitive_approx_cache() {
        use std::sync::Arc;
        use std::sync::atomic::AtomicBool;

        let mut value = Real::pi();
        assert!(value.to_f64_lossy().is_some());
        assert!(matches!(
            value.primitive_approx_cache.get(),
            PrimitiveApproxCache::F64(Some(_))
        ));

        value.abort(Arc::new(AtomicBool::new(false)));
        assert!(matches!(
            value.primitive_approx_cache.get(),
            PrimitiveApproxCache::Empty
        ));
    }

    #[test]
    /// Check Arc identity of an attached unset flag through rational materialization, cloning and owned folding.
    fn exact_real_abort_signal_survives_materialization_clone_and_fold() {
        use std::sync::Arc;
        use std::sync::atomic::AtomicBool;

        let signal = Arc::new(AtomicBool::new(false));
        let mut value = Real::new(Rational::fraction(1, 3).unwrap());
        assert!(value.computable.is_none());

        value.abort(signal.clone());
        assert!(
            value
                .abort_signal()
                .is_some_and(|attached| Arc::ptr_eq(attached, &signal))
        );

        let clone = value.clone();
        assert!(
            clone
                .abort_signal()
                .is_some_and(|attached| Arc::ptr_eq(attached, &signal))
        );
        let folded = clone.fold();
        assert!(
            folded
                .signal
                .as_ref()
                .is_some_and(|attached| Arc::ptr_eq(attached, &signal))
        );
    }

    #[test]
    #[cfg(feature = "cached-f64-approx")]
    /// With an already-set abort flag, require a lossy call to leave the cache empty; no result accuracy is asserted.
    fn aborted_lossy_conversion_is_not_cached() {
        use std::sync::Arc;
        use std::sync::atomic::AtomicBool;

        let signal = Arc::new(AtomicBool::new(true));
        let mut value = Real::pi();
        value.abort(signal);
        let _ = value.to_f64_lossy();

        assert!(matches!(
            value.primitive_approx_cache.get(),
            PrimitiveApproxCache::Empty
        ));
    }

    #[test]
    #[cfg(all(feature = "cached-f32-approx", feature = "cached-f64-approx"))]
    /// Run eight scoped threads with 128 alternating-width reads each and require uncached fixture values.
    /// The final cache must be F64; this finite stress check is not a memory-model proof.
    fn primitive_approx_cache_is_safe_under_concurrent_mixed_reads() {
        use std::sync::Arc;

        let value = Arc::new(Real::new(Rational::fraction(1, 3).unwrap()));
        let expected_f32 = value.to_f32_lossy_uncached();
        let expected_f64 = value.to_f64_lossy_uncached();
        std::thread::scope(|scope| {
            for index in 0..8 {
                let value = Arc::clone(&value);
                scope.spawn(move || {
                    for _ in 0..128 {
                        if index % 2 == 0 {
                            assert_eq!(value.to_f32_lossy(), expected_f32);
                        } else {
                            assert_eq!(value.to_f64_lossy(), expected_f64);
                        }
                    }
                });
            }
        });

        assert!(matches!(
            value.primitive_approx_cache.get(),
            PrimitiveApproxCache::F64(Some(_))
        ));
    }

    #[cfg(any(feature = "cached-f32-approx", feature = "cached-f64-approx"))]
    proptest! {
        #![proptest_config(ProptestConfig::with_cases(512))]

        #[test]
        #[cfg(feature = "cached-f64-approx")]
        /// For generated bounded rationals, require repeated binary64 cache hits to match fresh uncached bits.
        fn cached_f64_matches_uncached_for_rationals(rational in finite_rational_strategy()) {
            let value = Real::new(rational.clone());
            let expected = Real::new(rational).to_f64_lossy_uncached();

            prop_assert_eq!(value.to_f64_lossy().map(f64::to_bits), expected.map(f64::to_bits));
            prop_assert_eq!(value.to_f64_lossy().map(f64::to_bits), expected.map(f64::to_bits));
            prop_assert!(matches!(
                value.primitive_approx_cache.get(),
                PrimitiveApproxCache::F64(_)
            ));
        }

        #[test]
        #[cfg(feature = "cached-f64-approx")]
        /// Warm an F64 proposal and require binary32 reuse/fallback to match fresh uncached rational export.
        fn cached_f64_proposal_matches_uncached_f32_export(rational in finite_rational_strategy()) {
            let value = Real::new(rational.clone());
            let expected = Real::new(rational).to_f32_lossy_uncached();

            let _ = value.to_f64_lossy();
            prop_assert_eq!(value.to_f32_lossy().map(f32::to_bits), expected.map(f32::to_bits));
        }

        #[test]
        #[cfg(feature = "cached-f32-approx")]
        /// For generated bounded rationals, require repeated binary32 bits and the F32 cache tag.
        fn cached_f32_matches_uncached_for_rationals(rational in finite_rational_strategy()) {
            let value = Real::new(rational.clone());
            let expected = Real::new(rational).to_f32_lossy_uncached();

            prop_assert_eq!(value.to_f32_lossy().map(f32::to_bits), expected.map(f32::to_bits));
            prop_assert_eq!(value.to_f32_lossy().map(f32::to_bits), expected.map(f32::to_bits));
            prop_assert!(matches!(
                value.primitive_approx_cache.get(),
                PrimitiveApproxCache::F32(_)
            ));
        }

        #[test]
        #[cfg(all(feature = "cached-f32-approx", feature = "cached-f64-approx"))]
        /// Require both generated rational projections to match uncached bits while the cache upgrades from F32 to F64.
        fn f32_then_f64_cache_upgrade_matches_uncached(rational in finite_rational_strategy()) {
            let value = Real::new(rational.clone());
            let expected_f32 = Real::new(rational.clone()).to_f32_lossy_uncached();
            let expected_f64 = Real::new(rational).to_f64_lossy_uncached();

            prop_assert_eq!(value.to_f32_lossy().map(f32::to_bits), expected_f32.map(f32::to_bits));
            prop_assert!(matches!(
                value.primitive_approx_cache.get(),
                PrimitiveApproxCache::F32(_)
            ));
            prop_assert_eq!(value.to_f64_lossy().map(f64::to_bits), expected_f64.map(f64::to_bits));
            prop_assert!(matches!(
                value.primitive_approx_cache.get(),
                PrimitiveApproxCache::F64(_)
            ));
        }

        #[test]
        #[cfg(feature = "cached-f64-approx")]
        /// For nonzero generated rationals with distinct opposite export bits, require both negation forms
        /// to clear the primitive view and match a freshly negated uncached value.
        fn cached_f64_is_not_reused_after_negation(rational in nonzero_finite_rational_strategy()) {
            let value = Real::new(rational.clone());
            let cached_positive = value.to_f64_lossy();
            let negated_ref = -&value;
            let expected_ref = Real::new(-rational.clone()).to_f64_lossy_uncached();

            prop_assume!(cached_positive.map(f64::to_bits) != expected_ref.map(f64::to_bits));
            prop_assert!(matches!(
                negated_ref.primitive_approx_cache.get(),
                PrimitiveApproxCache::Empty
            ));
            prop_assert_eq!(negated_ref.to_f64_lossy().map(f64::to_bits), expected_ref.map(f64::to_bits));

            let owned = Real::new(rational.clone());
            let _ = owned.to_f64_lossy();
            let negated_owned = -owned;
            let expected_owned = Real::new(-rational).to_f64_lossy_uncached();
            prop_assert!(matches!(
                negated_owned.primitive_approx_cache.get(),
                PrimitiveApproxCache::Empty
            ));
            prop_assert_eq!(negated_owned.to_f64_lossy().map(f64::to_bits), expected_owned.map(f64::to_bits));
        }
    }
}
