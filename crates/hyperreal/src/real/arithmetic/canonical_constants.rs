/// Process-wide lazy exact rationals used in symbolic comparisons and special-value tables.
/// Fixed nonzero denominators justify the initializer unwraps. Values are never
/// authored through floating point; LazyLock initializes each independently on
/// first access, while Rational may retain its own shared accelerator facts.
pub(crate) mod rationals {
    use crate::Rational;
    use std::sync::LazyLock;

    /// Exact 1/2 for half-turn boundaries, roots and inverse-function special cases.
    pub(crate) static HALF: LazyLock<Rational> =
        LazyLock::new(|| Rational::fraction(1, 2).unwrap());
    /// Exact unit for class normalization and multiplicative identities.
    pub(crate) static ONE: LazyLock<Rational> = LazyLock::new(|| Rational::new(1));
    /// Exact 1/3 for retained rational special-angle formulas.
    pub(crate) static THIRD: LazyLock<Rational> =
        LazyLock::new(|| Rational::fraction(1, 3).unwrap());
    /// Exact integer three for pi-offset and small-constant recognition.
    pub(crate) static THREE: LazyLock<Rational> = LazyLock::new(|| Rational::new(3));
    /// Exact integer two for root, binary-log and half-angle formulas.
    pub(crate) static TWO: LazyLock<Rational> = LazyLock::new(|| Rational::new(2));
    // These tiny rationals sit on exact trig/inverse-trig hot paths. Reusing
    // them avoids repeated BigUint construction while preserving exact symbolic
    // dispatch. This is the same "cheap certificate before refinement" pattern
    // used by exact geometric predicates.
    /// Exact 1/4 for the unit-tangent pi-multiple and related inverse tables.
    pub(crate) static QUARTER: LazyLock<Rational> =
        LazyLock::new(|| Rational::fraction(1, 4).unwrap());
    /// Exact 1/6 for the rational sine half-value and related special angles.
    pub(crate) static SIXTH: LazyLock<Rational> =
        LazyLock::new(|| Rational::fraction(1, 6).unwrap());
    /// Exact integer six for small-denominator special forms.
    pub(crate) static SIX: LazyLock<Rational> = LazyLock::new(|| Rational::new(6));
    /// Exact 7/8 threshold selecting endpoint-adjacent rational atanh construction.
    pub(crate) static SEVEN_EIGHTHS: LazyLock<Rational> =
        LazyLock::new(|| Rational::fraction(7, 8).unwrap());
    /// Exact additive identity for exponent/offset normalization.
    pub(crate) static ZERO: LazyLock<Rational> = LazyLock::new(Rational::zero);
    /// Exact integer ten for common logarithm and power certificates.
    pub(crate) static TEN: LazyLock<Rational> = LazyLock::new(|| Rational::new(10));
}

/// Thread-local exact Real templates and selected special-form recognizers.
/// Accessors return clones, retaining named certificates and shared computable
/// nodes where present. A returned Real's primitive view is independent of the
/// template; these templates start empty and no accessor fills their primitive
/// cache. Lower-layer shared constants have their own cross-thread approximation
/// caches, so thread-local templates do not imply thread-local numeric evidence.
///
/// Initialization materializes exact recipes, not requested-precision samples.
/// Constant denominators and enumerated root/log tables justify unwraps. First
/// use may allocate; cloning is not a promise of allocation-free access or a
/// total-work bound. Most helpers attach no signal; pi_minus_three explicitly
/// transports its caller's signal to the returned clone.
mod constants {
    use super::{
        AtomicPrimitiveApproxCache, Class, ConstOffsetClass, LnAffineClass, PrimitiveApproxCache,
    };
    use crate::{Computable, Rational, Real};
    thread_local! {
        // These are the canonical internal constants. Public constructors clone
        // these symbolic/computable forms instead of rebuilding exact classes and
        // caches on every call. This keeps common pi/log/sqrt certificates in
        // the symbolic layer and delays approximation until `Computable::approx`.
        /// Pi basis with unit scale and a computable pi definition; no primitive view is precomputed.
        static PI: Real = Real {
            rational: Rational::one(),
            class: Class::Pi,
            computable: Some(Computable::pi()),
            primitive_approx_cache: AtomicPrimitiveApproxCache::new(PrimitiveApproxCache::Empty),
        };
        /// Exact 2*pi represented by rational scale two and the same Pi basis family.
        static TAU: Real = Real {
            rational: Rational::new(2),
            class: Class::Pi,
            computable: Some(Computable::pi()),
            primitive_approx_cache: AtomicPrimitiveApproxCache::new(PrimitiveApproxCache::Empty),
        };
        /// Exact pi/2 with rational scale 1/2, used by principal inverse-function branches.
        static PI_OVER_TWO: Real = Real {
            rational: Rational::fraction(1, 2).unwrap(),
            class: Class::Pi,
            computable: Some(Computable::pi()),
            primitive_approx_cache: AtomicPrimitiveApproxCache::new(PrimitiveApproxCache::Empty),
        };
        /// Exact pi/3 with rational scale 1/3 for recognized inverse-angle results.
        static PI_OVER_THREE: Real = Real {
            rational: Rational::fraction(1, 3).unwrap(),
            class: Class::Pi,
            computable: Some(Computable::pi()),
            primitive_approx_cache: AtomicPrimitiveApproxCache::new(PrimitiveApproxCache::Empty),
        };
        /// Exact pi/4 with rational scale 1/4, including unit atan results.
        static PI_OVER_FOUR: Real = Real {
            rational: Rational::fraction(1, 4).unwrap(),
            class: Class::Pi,
            computable: Some(Computable::pi()),
            primitive_approx_cache: AtomicPrimitiveApproxCache::new(PrimitiveApproxCache::Empty),
        };
        /// Exact pi/6 with rational scale 1/6, including asin(1/2).
        static PI_OVER_SIX: Real = Real {
            rational: Rational::fraction(1, 6).unwrap(),
            class: Class::Pi,
            computable: Some(Computable::pi()),
            primitive_approx_cache: AtomicPrimitiveApproxCache::new(PrimitiveApproxCache::Empty),
        };
        /// Payload-free exact rational 1/2 for trigonometric table results.
        static HALF: Real = Real::new(Rational::fraction(1, 2).unwrap());
        /// Positive sqrt(2)/2 with Sqrt(2) basis and a shared lower-layer sqrt(2) constant.
        static SQRT_TWO_OVER_TWO: Real = Real {
            rational: Rational::fraction(1, 2).unwrap(),
            class: Class::Sqrt(Rational::new(2)),
            computable: Some(Computable::sqrt_constant(2).unwrap()),
            primitive_approx_cache: AtomicPrimitiveApproxCache::new(PrimitiveApproxCache::Empty),
        };
        /// Positive sqrt(3)/2 with scale 1/2 and a shared sqrt(3) basis definition.
        static SQRT_THREE_OVER_TWO: Real = Real {
            rational: Rational::fraction(1, 2).unwrap(),
            class: Class::Sqrt(Rational::new(3)),
            computable: Some(Computable::sqrt_constant(3).unwrap()),
            primitive_approx_cache: AtomicPrimitiveApproxCache::new(PrimitiveApproxCache::Empty),
        };
        /// Positive sqrt(3) with unit rational scale, used by exact tangent and root cases.
        static SQRT_THREE: Real = Real {
            rational: Rational::one(),
            class: Class::Sqrt(Rational::new(3)),
            computable: Some(Computable::sqrt_constant(3).unwrap()),
            primitive_approx_cache: AtomicPrimitiveApproxCache::new(PrimitiveApproxCache::Empty),
        };
        /// Positive sqrt(3)/3, retaining its exact root basis for reciprocal-tangent cases.
        static SQRT_THREE_OVER_THREE: Real = Real {
            rational: Rational::fraction(1, 3).unwrap(),
            class: Class::Sqrt(Rational::new(3)),
            computable: Some(Computable::sqrt_constant(3).unwrap()),
            primitive_approx_cache: AtomicPrimitiveApproxCache::new(PrimitiveApproxCache::Empty),
        };
        /// Positive sqrt(6)/3; its basis uses a rational-root graph rather than the dedicated sqrt(2)/sqrt(3) table.
        static SQRT_SIX_OVER_THREE: Real = Real {
            rational: Rational::fraction(1, 3).unwrap(),
            class: Class::Sqrt(Rational::new(6)),
            computable: Some(Computable::sqrt_rational(Rational::new(6))),
            primitive_approx_cache: AtomicPrimitiveApproxCache::new(PrimitiveApproxCache::Empty),
        };
        /// Positive pi*sqrt(2) with a PiSqrt certificate and product graph.
        static PI_SQRT_TWO: Real = Real {
            rational: Rational::one(),
            class: Class::PiSqrt(Rational::new(2)),
            computable: Some(Computable::multiply(
                Computable::pi(),
                Computable::sqrt_constant(2).unwrap(),
            )),
            primitive_approx_cache: AtomicPrimitiveApproxCache::new(PrimitiveApproxCache::Empty),
        };
        /// Positive ln(2), retaining its exact base and shared computable logarithm constant.
        static LN2: Real = Real {
            rational: Rational::one(),
            class: Class::Ln(Rational::new(2)),
            computable: Some(Computable::ln_constant(2).unwrap()),
            primitive_approx_cache: AtomicPrimitiveApproxCache::new(PrimitiveApproxCache::Empty),
        };
        /// Positive ln(3), retaining its exact base and shared computable logarithm constant.
        static LN3: Real = Real {
            rational: Rational::one(),
            class: Class::Ln(Rational::new(3)),
            computable: Some(Computable::ln_constant(3).unwrap()),
            primitive_approx_cache: AtomicPrimitiveApproxCache::new(PrimitiveApproxCache::Empty),
        };
        /// Exact ln(3)/2 with rational scale 1/2; used by atanh(1/2) and its odd reflection.
        static HALF_LN3: Real = Real {
            rational: Rational::fraction(1, 2).unwrap(),
            class: Class::Ln(Rational::new(3)),
            computable: Some(Computable::ln_constant(3).unwrap()),
            primitive_approx_cache: AtomicPrimitiveApproxCache::new(PrimitiveApproxCache::Empty),
        };
        /// Positive ln(5), available to recognized integer-power logarithms.
        static LN5: Real = Real {
            rational: Rational::one(),
            class: Class::Ln(Rational::new(5)),
            computable: Some(Computable::ln_constant(5).unwrap()),
            primitive_approx_cache: AtomicPrimitiveApproxCache::new(PrimitiveApproxCache::Empty),
        };
        /// Positive ln(6), available to recognized integer-power logarithms.
        static LN6: Real = Real {
            rational: Rational::one(),
            class: Class::Ln(Rational::new(6)),
            computable: Some(Computable::ln_constant(6).unwrap()),
            primitive_approx_cache: AtomicPrimitiveApproxCache::new(PrimitiveApproxCache::Empty),
        };
        /// Positive ln(7), available to recognized integer-power logarithms.
        static LN7: Real = Real {
            rational: Rational::one(),
            class: Class::Ln(Rational::new(7)),
            computable: Some(Computable::ln_constant(7).unwrap()),
            primitive_approx_cache: AtomicPrimitiveApproxCache::new(PrimitiveApproxCache::Empty),
        };
        /// Positive ln(10), also serving base-ten logarithm conversion.
        static LN10: Real = Real {
            rational: Rational::one(),
            class: Class::Ln(Rational::new(10)),
            computable: Some(Computable::ln_constant(10).unwrap()),
            primitive_approx_cache: AtomicPrimitiveApproxCache::new(PrimitiveApproxCache::Empty),
        };
        /// Exact positive pi-3 in ConstOffset form; positivity follows from pi>3.
        static PI_MINUS_THREE: Real = Real {
            rational: Rational::one(),
            class: Class::ConstOffset(Box::new(ConstOffsetClass {
                pi_power: 1,
                exp_power: Rational::zero(),
                offset: Rational::new(-3),
            })),
            computable: Some(Computable::add(
                Computable::pi(),
                Computable::rational(Rational::new(-3)),
            )),
            primitive_approx_cache: AtomicPrimitiveApproxCache::new(PrimitiveApproxCache::Empty),
        };
        /// Exact positive 1+ln(2) in LnAffine form, retaining the offset and logarithmic base.
        static ONE_PLUS_LN2: Real = Real {
            rational: Rational::one(),
            class: Class::LnAffine(Box::new(LnAffineClass {
                offset: Rational::one(),
                base: Rational::new(2),
            })),
            computable: Some(Computable::add(
                Computable::one(),
                Computable::ln_constant(2).unwrap(),
            )),
            primitive_approx_cache: AtomicPrimitiveApproxCache::new(PrimitiveApproxCache::Empty),
        };
        /// Exact exp(1), using the named Exp certificate and shared e computable constant.
        static E: Real = Real {
            rational: Rational::one(),
            class: Class::Exp(Rational::one()),
            computable: Some(Computable::e_constant()),
            primitive_approx_cache: AtomicPrimitiveApproxCache::new(PrimitiveApproxCache::Empty),
        };
    }

    /// Clone the thread-local exact rational 1/2 template; no computable payload is required.
    pub(super) fn half() -> Real {
        HALF.with(|real| real.clone())
    }

    /// Clone the retained ln(3)/2 form used by exact inverse-hyperbolic special cases.
    pub(super) fn half_ln3() -> Real {
        HALF_LN3.with(|real| real.clone())
    }

    /// Clone the thread-local Pi template, preserving its exact basis and reusable graph.
    pub(super) fn pi() -> Real {
        PI.with(|real| real.clone())
    }

    /// Recognize exactly n=+/-1 with d in {2, 3, 4, 6}, returning the signed canonical pi fraction.
    /// Other tuples return None for caller fallback, including unreduced equivalents such as (2, 4).
    /// No rational reduction or generic denominator validation occurs here; d=0 also declines.
    pub(super) fn pi_fraction(n: i64, d: u64) -> Option<Real> {
        // Exact inverse trig repeatedly returns these pi fractions. Reusing the
        // cached symbolic Real avoids rational construction and a multiply by pi
        // on the dispatch path while preserving the public representation.
        match (n, d) {
            (1, 2) => Some(PI_OVER_TWO.with(|real| real.clone())),
            (-1, 2) => Some(PI_OVER_TWO.with(|real| -real.clone())),
            (1, 3) => Some(PI_OVER_THREE.with(|real| real.clone())),
            (-1, 3) => Some(PI_OVER_THREE.with(|real| -real.clone())),
            (1, 4) => Some(PI_OVER_FOUR.with(|real| real.clone())),
            (-1, 4) => Some(PI_OVER_FOUR.with(|real| -real.clone())),
            (1, 6) => Some(PI_OVER_SIX.with(|real| real.clone())),
            (-1, 6) => Some(PI_OVER_SIX.with(|real| -real.clone())),
            _ => None,
        }
    }

    /// Clone exact 2*pi from its scaled Pi template, without constructing a floating tau value.
    pub(super) fn tau() -> Real {
        TAU.with(|real| real.clone())
    }

    /// Clone positive sqrt(2)/2 with its retained root certificate.
    pub(super) fn sqrt_two_over_two() -> Real {
        SQRT_TWO_OVER_TWO.with(|real| real.clone())
    }

    /// Clone positive sqrt(3)/2 with its retained root certificate.
    pub(super) fn sqrt_three_over_two() -> Real {
        SQRT_THREE_OVER_TWO.with(|real| real.clone())
    }

    /// Clone positive sqrt(3) from the thread-local root template.
    pub(super) fn sqrt_three() -> Real {
        SQRT_THREE.with(|real| real.clone())
    }

    /// Clone positive sqrt(3)/3; its exact root relation survives later arithmetic.
    pub(super) fn sqrt_three_over_three() -> Real {
        SQRT_THREE_OVER_THREE.with(|real| real.clone())
    }

    /// Clone positive sqrt(6)/3 and its reusable rational-root graph.
    pub(super) fn sqrt_six_over_three() -> Real {
        SQRT_SIX_OVER_THREE.with(|real| real.clone())
    }

    /// Clone the retained pi*sqrt(2) product certificate and computable graph.
    pub(super) fn pi_sqrt_two() -> Real {
        PI_SQRT_TWO.with(|real| real.clone())
    }

    /// Recognize only radicands two and three. Two constructs a new Real around the
    /// shared computable sqrt(2) constant; three clones the existing template. Return
    /// None for every other integer, including valid roots that the caller can construct.
    pub(super) fn sqrt_constant(n: i64) -> Option<Real> {
        match n {
            2 => Some(Real {
                rational: Rational::one(),
                class: Class::Sqrt(Rational::new(2)),
                computable: Some(Computable::sqrt_constant(2).unwrap()),
                primitive_approx_cache: AtomicPrimitiveApproxCache::new(
                    PrimitiveApproxCache::Empty,
                ),
            }),
            3 => Some(sqrt_three()),
            _ => None,
        }
    }

    /// Recognize bases 2, 3, 5, 6, 7 and 10; clone their templates and replace only the rational scale.
    /// The coefficient may be negative or zero; zero retains an Ln basis rather than canonicalizing to a payload-free zero.
    /// The template primitive view is empty, so this scale change cannot reuse a stale projected value.
    /// Unsupported bases return None even when their mathematical logarithm exists.
    pub(super) fn scaled_ln(base: u32, coefficient: i64) -> Option<Real> {
        // Return clones of canonical cached ln constants with only the rational
        // scale adjusted. This is cheaper than constructing a fresh Ln class and
        // computable cache for every recognized log power.
        let mut value = match base {
            2 => LN2.with(|real| real.clone()),
            3 => LN3.with(|real| real.clone()),
            5 => LN5.with(|real| real.clone()),
            6 => LN6.with(|real| real.clone()),
            7 => LN7.with(|real| real.clone()),
            10 => LN10.with(|real| real.clone()),
            _ => return None,
        };
        value.rational = Rational::new(coefficient);
        Some(value)
    }

    /// Clone the fixed pi-3 template and transport the argument's abort signal if present.
    /// The argument is used only for signal inheritance; it is not numerically subtracted.
    /// Callers must select this helper only for the intended pi-minus-three construction.
    pub(super) fn pi_minus_three(pi: &Real) -> Real {
        let mut shifted = PI_MINUS_THREE.with(|real| real.clone());
        if let Some(signal) = pi.abort_signal() {
            shifted.abort(signal.clone());
        }
        shifted
    }

    /// Clone exact 1+ln(2), retaining its positive additive logarithm certificate.
    pub(super) fn one_plus_ln2() -> Real {
        ONE_PLUS_LN2.with(|real| real.clone())
    }

    /// Clone exact exp(1), retaining the named exponential basis and shared constant graph.
    pub(super) fn e() -> Real {
        E.with(|real| real.clone())
    }
}

/// Process-wide lazy signed integer identities for exact exponent dispatch.
mod signed {
    use num::{BigInt, One};
    use std::sync::LazyLock;

    // The identity constructor is clearer and avoids a `ToBigInt` temporary.
    /// Exact signed integer one for exponent identity comparisons.
    pub(super) static ONE: LazyLock<BigInt> = LazyLock::new(BigInt::one);
}

/// Process-wide lazy unsigned integers for denominator and exponent recognition.
/// Small primitive literals are widened exactly into BigUint on first access.
mod unsigned {
    use num::BigUint;
    use num::One;
    use std::sync::LazyLock;

    // Small exact constants use the narrow primitive that contains them; the
    // bigint constructors can widen directly from there.
    /// Exact unsigned integer 1 for denominator/identity table comparisons.
    pub(super) static ONE: LazyLock<BigUint> = LazyLock::new(BigUint::one);
    /// Exact unsigned integer 2 for denominator/identity table comparisons.
    pub(super) static TWO: LazyLock<BigUint> = LazyLock::new(|| BigUint::from(2_u8));
    /// Exact unsigned integer 3 for denominator/identity table comparisons.
    pub(super) static THREE: LazyLock<BigUint> = LazyLock::new(|| BigUint::from(3_u8));
    /// Exact unsigned integer 4 for denominator/identity table comparisons.
    pub(super) static FOUR: LazyLock<BigUint> = LazyLock::new(|| BigUint::from(4_u8));
    /// Exact unsigned integer 6 for denominator/identity table comparisons.
    pub(super) static SIX: LazyLock<BigUint> = LazyLock::new(|| BigUint::from(6_u8));
}
