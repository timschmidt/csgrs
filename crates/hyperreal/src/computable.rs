use crate::computable::approximation::{Approximation, SharedConstant};
use crate::{MagnitudeBits, Rational, RealSign, RealStructuralFacts, ZeroKnowledge};
use core::cmp::Ordering;
use num::Signed;
use num::{BigInt, BigUint, bigint::Sign};
use num::{One, Zero};
use serde::{Deserialize, Serialize};
use std::{
    cell::RefCell,
    ops::{Deref, Neg},
    sync::LazyLock,
};

mod approximation;
mod format;

pub type Precision = i32;

#[derive(Clone, Debug, PartialEq, Default)]
enum Cache {
    #[default]
    Invalid,
    Valid((Precision, BigInt)),
}

#[derive(Clone, Debug, PartialEq, Default)]
enum BoundCache {
    #[default]
    Invalid,
    Valid(BoundInfo),
}

#[derive(Clone, Copy, Debug, PartialEq, Default)]
enum ExactSignCache {
    #[default]
    Invalid,
    Unknown,
    Valid(Sign),
}

#[derive(Clone, Debug, PartialEq, Eq)]
enum BoundInfo {
    // Unknown means the expression may still be zero or either sign; callers
    // must not use it to short-circuit exact predicates.
    Unknown,
    // Exact structural zero, usually from rational leaves or annihilating
    // products.
    Zero,
    // NonZero may have an unknown sign or inexact MSD. That is still enough for
    // zero-status fast paths and precision planning.
    NonZero {
        sign: Option<Sign>,
        msd: Option<Precision>,
        exact_msd: bool,
    },
}

impl BoundInfo {
    fn with_sign(sign: Sign, msd: Option<Precision>) -> Self {
        Self::with_sign_msd(sign, msd, true)
    }

    fn with_sign_msd(sign: Sign, msd: Option<Precision>, exact_msd: bool) -> Self {
        match sign {
            Sign::NoSign => Self::Zero,
            _ => Self::NonZero {
                sign: Some(sign),
                msd,
                exact_msd,
            },
        }
    }

    fn from_rational(r: &Rational) -> Self {
        match r.msd_exact() {
            Some(msd) => Self::with_sign(r.sign(), Some(msd)),
            None => Self::Zero,
        }
    }

    fn map_msd(self, f: impl FnOnce(Precision) -> Precision) -> Self {
        match self {
            Self::NonZero {
                sign,
                msd,
                exact_msd,
            } => Self::NonZero {
                sign,
                msd: msd.map(f),
                exact_msd,
            },
            other => other,
        }
    }

    fn negate(self) -> Self {
        match self {
            Self::NonZero {
                sign: Some(Sign::Plus),
                msd,
                exact_msd,
            } => Self::NonZero {
                sign: Some(Sign::Minus),
                msd,
                exact_msd,
            },
            Self::NonZero {
                sign: Some(Sign::Minus),
                msd,
                exact_msd,
            } => Self::NonZero {
                sign: Some(Sign::Plus),
                msd,
                exact_msd,
            },
            other => other,
        }
    }

    fn inverse(self) -> Self {
        match self {
            Self::NonZero { sign, msd, .. } => Self::NonZero {
                sign,
                msd: msd.map(|value| 1 - value),
                exact_msd: false,
            },
            other => other,
        }
    }

    fn square(self) -> Self {
        match self {
            Self::Zero => Self::Zero,
            Self::NonZero { msd, .. } => Self::NonZero {
                sign: Some(Sign::Plus),
                msd: msd.map(|value| value * 2),
                exact_msd: false,
            },
            Self::Unknown => Self::Unknown,
        }
    }

    fn sqrt(self) -> Self {
        match self {
            Self::Zero => Self::Zero,
            Self::NonZero {
                sign: Some(Sign::Plus),
                msd,
                ..
            } => Self::NonZero {
                sign: Some(Sign::Plus),
                msd: msd.map(|value| value / 2),
                exact_msd: false,
            },
            _ => Self::Unknown,
        }
    }

    fn multiply(self, other: Self) -> Self {
        match (self, other) {
            (Self::Zero, _) | (_, Self::Zero) => Self::Zero,
            (
                Self::NonZero {
                    sign: left_sign,
                    msd: left_msd,
                    ..
                },
                Self::NonZero {
                    sign: right_sign,
                    msd: right_msd,
                    ..
                },
            ) => {
                let sign = match (left_sign, right_sign) {
                    (Some(Sign::Plus), Some(Sign::Plus))
                    | (Some(Sign::Minus), Some(Sign::Minus)) => Some(Sign::Plus),
                    (Some(Sign::Plus), Some(Sign::Minus))
                    | (Some(Sign::Minus), Some(Sign::Plus)) => Some(Sign::Minus),
                    _ => None,
                };
                let msd = match (left_msd, right_msd) {
                    (Some(left), Some(right)) => Some(left + right),
                    _ => None,
                };
                Self::NonZero {
                    sign,
                    msd,
                    exact_msd: false,
                }
            }
            _ => Self::Unknown,
        }
    }

    fn add(self, other: Self) -> Self {
        // Addition can certify sign when operands share a sign or one MSD
        // dominates an opposite-signed operand. Near-cancellation deliberately
        // returns Unknown so callers fall back to refinement.
        match (self, other) {
            (Self::Zero, other) | (other, Self::Zero) => other,
            (
                Self::NonZero {
                    sign: left_sign,
                    msd: left_msd,
                    ..
                },
                Self::NonZero {
                    sign: right_sign,
                    msd: right_msd,
                    ..
                },
            ) => {
                let sign = match (left_sign, right_sign) {
                    (Some(left), Some(right)) if left == right => Some(left),
                    (Some(Sign::Plus), Some(Sign::Minus))
                    | (Some(Sign::Minus), Some(Sign::Plus)) => match (left_msd, right_msd) {
                        (Some(left), Some(right)) if left > right + 1 => left_sign,
                        (Some(left), Some(right)) if right > left + 1 => right_sign,
                        _ => None,
                    },
                    _ => None,
                };
                let msd = match (left_sign, right_sign, left_msd, right_msd) {
                    (_, _, Some(left), Some(right)) if left > right => Some(left),
                    (_, _, Some(left), Some(right)) if right > left => Some(right),
                    (Some(left_sign), Some(right_sign), Some(left), Some(right))
                        if left_sign != right_sign && left > right + 1 =>
                    {
                        Some(left)
                    }
                    (Some(left_sign), Some(right_sign), Some(left), Some(right))
                        if left_sign != right_sign && right > left + 1 =>
                    {
                        Some(right)
                    }
                    _ => None,
                };
                match sign {
                    Some(sign) => Self::NonZero {
                        sign: Some(sign),
                        msd,
                        exact_msd: false,
                    },
                    None if msd.is_some() => Self::NonZero {
                        sign: None,
                        msd,
                        exact_msd: false,
                    },
                    None => Self::Unknown,
                }
            }
            _ => Self::Unknown,
        }
    }

    fn known_msd(&self) -> Option<Option<Precision>> {
        match self {
            Self::Unknown => None,
            Self::Zero => Some(None),
            Self::NonZero {
                msd,
                exact_msd: true,
                ..
            } => Some(*msd),
            Self::NonZero { .. } => None,
        }
    }

    fn planning_msd(&self) -> Option<Option<Precision>> {
        match self {
            Self::Unknown => None,
            Self::Zero => Some(None),
            Self::NonZero { msd, .. } => Some(*msd),
        }
    }

    fn known_sign(&self) -> Option<Sign> {
        match self {
            Self::Zero => Some(Sign::NoSign),
            Self::NonZero { sign, .. } => *sign,
            Self::Unknown => None,
        }
    }

    fn magnitude_bits(&self) -> Option<MagnitudeBits> {
        match self {
            Self::NonZero {
                msd: Some(msd),
                exact_msd,
                ..
            } => Some(MagnitudeBits {
                msd: *msd,
                exact_msd: *exact_msd,
            }),
            _ => None,
        }
    }
}

impl SharedConstant {
    fn bound_info(self) -> BoundInfo {
        // Coarse but exact-enough MSD facts for shared constants. These feed
        // structural queries and trig/ln reduction planning without forcing a
        // cached approximation.
        let msd = match self {
            SharedConstant::E | SharedConstant::Pi | SharedConstant::Ln10 => Some(1),
            SharedConstant::Tau => Some(2),
            SharedConstant::Ln2 => Some(-1),
            SharedConstant::Ln3
            | SharedConstant::Ln5
            | SharedConstant::Ln6
            | SharedConstant::Ln7
            | SharedConstant::Sqrt2
            | SharedConstant::Sqrt3 => Some(0),
        };
        BoundInfo::with_sign(Sign::Plus, msd)
    }

    fn interval(self) -> (Rational, Rational) {
        // Narrow rational intervals used only for constant+rational sign
        // certificates. They are intentionally cheap and hand-picked, not a
        // replacement for high-precision approximation.
        match self {
            Self::E => (
                Rational::fraction(271_828, 100_000).unwrap(),
                Rational::fraction(271_829, 100_000).unwrap(),
            ),
            Self::Pi => (
                Rational::fraction(333, 106).unwrap(),
                Rational::fraction(355, 113).unwrap(),
            ),
            Self::Tau => (
                Rational::fraction(333, 53).unwrap(),
                Rational::fraction(710, 113).unwrap(),
            ),
            Self::Ln2 => (
                Rational::fraction(69, 100).unwrap(),
                Rational::fraction(7, 10).unwrap(),
            ),
            Self::Ln3 => (
                Rational::fraction(109, 100).unwrap(),
                Rational::fraction(11, 10).unwrap(),
            ),
            Self::Ln5 => (
                Rational::fraction(160, 100).unwrap(),
                Rational::fraction(161, 100).unwrap(),
            ),
            Self::Ln6 => (
                Rational::fraction(179, 100).unwrap(),
                Rational::fraction(18, 10).unwrap(),
            ),
            Self::Ln7 => (
                Rational::fraction(194, 100).unwrap(),
                Rational::fraction(195, 100).unwrap(),
            ),
            Self::Ln10 => (
                Rational::fraction(230, 100).unwrap(),
                Rational::fraction(231, 100).unwrap(),
            ),
            Self::Sqrt2 => (
                Rational::fraction(141, 100).unwrap(),
                Rational::fraction(142, 100).unwrap(),
            ),
            Self::Sqrt3 => (
                Rational::fraction(173, 100).unwrap(),
                Rational::fraction(174, 100).unwrap(),
            ),
        }
    }
}

fn negate_sign(sign: Sign) -> Sign {
    match sign {
        Sign::Plus => Sign::Minus,
        Sign::Minus => Sign::Plus,
        Sign::NoSign => Sign::NoSign,
    }
}

fn public_sign(sign: Sign) -> RealSign {
    match sign {
        Sign::Minus => RealSign::Negative,
        Sign::NoSign => RealSign::Zero,
        Sign::Plus => RealSign::Positive,
    }
}

use std::sync::Arc;
use std::sync::atomic::AtomicBool;

pub type Signal = Arc<AtomicBool>;

fn should_stop(signal: &Option<Signal>) -> bool {
    use std::sync::atomic::Ordering::*;
    signal.as_ref().is_some_and(|s| s.load(Relaxed))
}

thread_local! {
    // Constants are value objects, so separate `Computable::pi()` calls are
    // common. Sharing only their approximation cache avoids rebuilding large
    // constant approximations in scalar and matrix workloads while keeping the
    // public `Computable` instances independently owned.
    static SHARED_CONSTANT_CACHES: RefCell<Vec<Cache>> =
        RefCell::new(vec![Cache::Invalid; SharedConstant::COUNT]);
}

/// Computable approximation of a Real number.
///
/// This is a demand-driven exact-real representation: every node can produce an
/// integer approximation at a requested binary precision, and caches store only
/// approximations proven for that node. The model follows the constructive/exact
/// real arithmetic approach in Boehm et al., "Exact real arithmetic: a case
/// study in higher order programming", https://doi.org/10.1145/319838.319860.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Computable {
    internal: Box<Approximation>,
    #[serde(skip)]
    cache: RefCell<Cache>,
    #[serde(skip)]
    bound: RefCell<BoundCache>,
    #[serde(skip)]
    exact_sign: RefCell<ExactSignCache>,
    #[serde(skip)]
    signal: Option<Signal>,
}

mod signed {
    use num::One;
    use num::{BigInt, bigint::ToBigInt};
    use std::sync::LazyLock;

    pub(super) static MINUS_ONE: LazyLock<BigInt> =
        LazyLock::new(|| ToBigInt::to_bigint(&-1).unwrap());
    pub(super) static ONE: LazyLock<BigInt> = LazyLock::new(BigInt::one);
    pub(super) static TWO: LazyLock<BigInt> = LazyLock::new(|| ToBigInt::to_bigint(&2).unwrap());
    pub(super) static FOUR: LazyLock<BigInt> = LazyLock::new(|| ToBigInt::to_bigint(&4).unwrap());
    pub(super) static SIX: LazyLock<BigInt> = LazyLock::new(|| ToBigInt::to_bigint(&6).unwrap());
    pub(super) static EIGHT: LazyLock<BigInt> = LazyLock::new(|| ToBigInt::to_bigint(&8).unwrap());
    pub(super) static SIXTEEN: LazyLock<BigInt> =
        LazyLock::new(|| ToBigInt::to_bigint(&16).unwrap());
    pub(super) static TWENTY_FOUR: LazyLock<BigInt> =
        LazyLock::new(|| ToBigInt::to_bigint(&24).unwrap());
    pub(super) static SIXTY_FOUR: LazyLock<BigInt> =
        LazyLock::new(|| ToBigInt::to_bigint(&64).unwrap());
}

mod unsigned {
    use num::One;
    use num::{BigUint, bigint::ToBigUint};
    use std::sync::LazyLock;

    pub(super) static ONE: LazyLock<BigUint> = LazyLock::new(BigUint::one);
    pub(super) static TWO: LazyLock<BigUint> = LazyLock::new(|| ToBigUint::to_biguint(&2).unwrap());
    pub(super) static TEN: LazyLock<BigUint> =
        LazyLock::new(|| ToBigUint::to_biguint(&10).unwrap());
    pub(super) static FIVE: LazyLock<BigUint> =
        LazyLock::new(|| ToBigUint::to_biguint(&5).unwrap());
    pub(super) static SIX: LazyLock<BigUint> = LazyLock::new(|| ToBigUint::to_biguint(&6).unwrap());
}

static HALF_PI_SHORTCUT_RATIONAL_LIMIT: LazyLock<Rational> =
    LazyLock::new(|| Rational::fraction(3, 2).unwrap());
static INVERSE_ENDPOINT_RATIONAL_THRESHOLD: LazyLock<Rational> =
    LazyLock::new(|| Rational::fraction(7, 8).unwrap());

impl Computable {
    /// Exactly zero.
    pub fn zero() -> Computable {
        Self {
            internal: Box::new(Approximation::Int(BigInt::zero())),
            cache: RefCell::new(Cache::Invalid),
            bound: RefCell::new(BoundCache::Valid(BoundInfo::Zero)),
            exact_sign: RefCell::new(ExactSignCache::Valid(Sign::NoSign)),
            signal: None,
        }
    }

    /// Exactly one.
    pub fn one() -> Computable {
        Self {
            internal: Box::new(Approximation::One),
            cache: RefCell::new(Cache::Invalid),
            bound: RefCell::new(BoundCache::Valid(BoundInfo::with_sign(Sign::Plus, Some(0)))),
            exact_sign: RefCell::new(ExactSignCache::Valid(Sign::Plus)),
            signal: None,
        }
    }

    /// Approximate π, the ratio of a circle's circumference to its diameter.
    pub fn pi() -> Computable {
        Self::shared_constant(SharedConstant::Pi)
    }

    /// Approximate τ, the ratio of a circle's circumference to its radius.
    pub fn tau() -> Computable {
        Self::shared_constant(SharedConstant::Tau)
    }

    /// Approximate e, Euler's number and the base of the natural logarithm.
    pub fn e() -> Computable {
        Self::e_constant()
    }

    pub(crate) fn e_constant() -> Computable {
        Self::shared_constant(SharedConstant::E)
    }

    pub(crate) fn ln_constant(base: u32) -> Option<Computable> {
        // Common logarithms are shared constants so repeated symbolic ln forms
        // reuse one approximation cache across cloned Real values.
        let constant = match base {
            2 => SharedConstant::Ln2,
            3 => SharedConstant::Ln3,
            5 => SharedConstant::Ln5,
            6 => SharedConstant::Ln6,
            7 => SharedConstant::Ln7,
            10 => SharedConstant::Ln10,
            _ => return None,
        };
        Some(Self::shared_constant(constant))
    }

    pub(crate) fn sqrt_constant(n: i64) -> Option<Computable> {
        // sqrt(2) and sqrt(3) are exact trig outputs; caching them prevents
        // fresh sqrt kernels in every sin/cos special form.
        let constant = match n {
            2 => SharedConstant::Sqrt2,
            3 => SharedConstant::Sqrt3,
            _ => return None,
        };
        Some(Self::shared_constant(constant))
    }

    pub(crate) fn prescaled_sin(value: Computable) -> Computable {
        // Caller promises argument reduction has already happened. Keeping this
        // constructor private prevents large arguments from entering the Taylor
        // kernel directly.
        Self {
            internal: Box::new(Approximation::PrescaledSin(value)),
            cache: RefCell::new(Cache::Invalid),
            bound: RefCell::new(BoundCache::Invalid),
            exact_sign: RefCell::new(ExactSignCache::Invalid),
            signal: None,
        }
    }

    pub(crate) fn prescaled_cos(value: Computable) -> Computable {
        // Same reduced-argument contract as prescaled_sin. Cosine has exact
        // zero/one shortcuts in the public constructor, so this stays a raw
        // approximation node for already-small residuals.
        Self {
            internal: Box::new(Approximation::PrescaledCos(value)),
            cache: RefCell::new(Cache::Invalid),
            bound: RefCell::new(BoundCache::Invalid),
            exact_sign: RefCell::new(ExactSignCache::Invalid),
            signal: None,
        }
    }

    pub(crate) fn cos_large_rational_deferred(rational: Rational) -> Computable {
        // Real::cos for large plain rationals defers the expensive half-pi
        // reduction until digits are requested. This keeps construction and
        // structural queries cheap while preserving the canonical reducer for
        // actual approximation.
        Self {
            internal: Box::new(Approximation::CosLargeRational(rational)),
            cache: RefCell::new(Cache::Invalid),
            bound: RefCell::new(BoundCache::Invalid),
            exact_sign: RefCell::new(ExactSignCache::Invalid),
            signal: None,
        }
    }

    fn prescaled_cos_half_pi_minus_rational(rational: Rational) -> Computable {
        // sin(x) for exact medium rational x is cos(pi/2 - x). Keeping the
        // residual as one node avoids the generic Add/Offset/Negate stack in
        // the cold scalar f64 and 7/5 benchmarks.
        let internal = Approximation::PrescaledCosHalfPiMinusRational(rational);
        Self {
            internal: Box::new(internal),
            cache: RefCell::new(Cache::Invalid),
            bound: RefCell::new(BoundCache::Invalid),
            exact_sign: RefCell::new(ExactSignCache::Invalid),
            signal: None,
        }
    }

    fn prescaled_sin_half_pi_minus_rational(rational: Rational) -> Computable {
        // cos(x) for exact medium rational x is sin(pi/2 - x). This mirrors the
        // cosine shortcut above and keeps common dyadic imports off the generic
        // composite residual path.
        let internal = Approximation::PrescaledSinHalfPiMinusRational(rational);
        Self {
            internal: Box::new(internal),
            cache: RefCell::new(Cache::Invalid),
            bound: RefCell::new(BoundCache::Invalid),
            exact_sign: RefCell::new(ExactSignCache::Invalid),
            signal: None,
        }
    }

    pub(crate) fn sin_large_rational_deferred(rational: Rational) -> Computable {
        // Same lazy-construction policy as cos_large_rational_deferred. The
        // stored rational is exact, so approximation can rebuild the normal
        // Computable::sin path without changing numerical semantics.
        Self {
            internal: Box::new(Approximation::SinLargeRational(rational)),
            cache: RefCell::new(Cache::Invalid),
            bound: RefCell::new(BoundCache::Invalid),
            exact_sign: RefCell::new(ExactSignCache::Invalid),
            signal: None,
        }
    }

    pub(crate) fn prescaled_tan(value: Computable) -> Computable {
        // Same reduced-argument contract as prescaled_sin; tangent additionally
        // relies on the public constructor to handle near-pole complements.
        Self {
            internal: Box::new(Approximation::PrescaledTan(value)),
            cache: RefCell::new(Cache::Invalid),
            bound: RefCell::new(BoundCache::Invalid),
            exact_sign: RefCell::new(ExactSignCache::Invalid),
            signal: None,
        }
    }

    fn prescaled_asinh(value: Computable) -> Computable {
        // Tiny exact-rational asinh inputs use a direct odd-power series. This
        // keeps public construction cheap for scalar endpoint benches and only
        // enters the kernel after |x| has been structurally certified tiny.
        Self {
            internal: Box::new(Approximation::PrescaledAsinh(value)),
            cache: RefCell::new(Cache::Invalid),
            bound: RefCell::new(BoundCache::Invalid),
            exact_sign: RefCell::new(ExactSignCache::Invalid),
            signal: None,
        }
    }

    fn acos_positive(value: Computable) -> Computable {
        // For x >= 0, acos(x) is reduced with 2*atan(sqrt((1-x)/(1+x))).
        // A single deferred node avoids allocating that whole formula during
        // public construction of endpoint-heavy inverse trig expressions.
        Self {
            internal: Box::new(Approximation::AcosPositive(value)),
            cache: RefCell::new(Cache::Invalid),
            bound: RefCell::new(BoundCache::Invalid),
            exact_sign: RefCell::new(ExactSignCache::Valid(Sign::Plus)),
            signal: None,
        }
    }

    pub(crate) fn atanh_direct_deferred(value: Computable) -> Computable {
        // Endpoint atanh uses a deferred ln-ratio node. This keeps construction
        // cheap for predicate/scalar benches while preserving the same
        // approximation identity when a numeric value is requested.
        Self {
            internal: Box::new(Approximation::AtanhDirect(value)),
            cache: RefCell::new(Cache::Invalid),
            bound: RefCell::new(BoundCache::Invalid),
            exact_sign: RefCell::new(ExactSignCache::Invalid),
            signal: None,
        }
    }

    pub(crate) fn acosh_near_one_deferred(value: Computable) -> Computable {
        // Near-one acosh uses a deferred ln1p/sqrt reduction. That avoids
        // building the reduction graph during scalar construction while keeping
        // the cancellation-resistant approximation path.
        Self {
            internal: Box::new(Approximation::AcoshNearOne(value)),
            cache: RefCell::new(Cache::Invalid),
            bound: RefCell::new(BoundCache::Invalid),
            exact_sign: RefCell::new(ExactSignCache::Valid(Sign::Plus)),
            signal: None,
        }
    }

    pub(crate) fn acosh_direct_deferred(value: Computable) -> Computable {
        // Large acosh uses a deferred direct ln/sqrt identity for Real scalar
        // construction. The public Computable kernel keeps the eager graph for
        // approximation-heavy callers.
        Self {
            internal: Box::new(Approximation::AcoshDirect(value)),
            cache: RefCell::new(Cache::Invalid),
            bound: RefCell::new(BoundCache::Invalid),
            exact_sign: RefCell::new(ExactSignCache::Valid(Sign::Plus)),
            signal: None,
        }
    }

    pub(crate) fn asinh_near_zero_deferred(value: Computable) -> Computable {
        // Moderate/tiny asinh inputs use a deferred ln1p reduction so public
        // construction stays lightweight while approximation still avoids
        // cancellation near zero.
        let sign = value.exact_sign();
        Self {
            internal: Box::new(Approximation::AsinhNearZero(value)),
            cache: RefCell::new(Cache::Invalid),
            bound: RefCell::new(BoundCache::Invalid),
            exact_sign: RefCell::new(sign.map_or(ExactSignCache::Invalid, ExactSignCache::Valid)),
            signal: None,
        }
    }

    pub(crate) fn asinh_direct_deferred(value: Computable) -> Computable {
        // Large asinh inputs use a deferred direct ln/sqrt identity. The caller
        // chooses this only after sign and size reduction, so no extra probing
        // is needed during construction.
        let sign = value.exact_sign();
        Self {
            internal: Box::new(Approximation::AsinhDirect(value)),
            cache: RefCell::new(Cache::Invalid),
            bound: RefCell::new(BoundCache::Invalid),
            exact_sign: RefCell::new(sign.map_or(ExactSignCache::Invalid, ExactSignCache::Valid)),
            signal: None,
        }
    }

    fn shared_constant(constant: SharedConstant) -> Computable {
        // Shared constants start with valid structural facts. Approximation
        // values are cached globally per thread, but the bound/sign caches can
        // be initialized directly on each lightweight wrapper.
        Self {
            internal: Box::new(Approximation::Constant(constant)),
            cache: RefCell::new(Cache::Invalid),
            bound: RefCell::new(BoundCache::Valid(constant.bound_info())),
            exact_sign: RefCell::new(ExactSignCache::Valid(Sign::Plus)),
            signal: None,
        }
    }

    /// Any Rational.
    pub fn rational(r: Rational) -> Computable {
        if r.sign() == Sign::NoSign {
            // Canonicalize rational zero at construction time. This exposes
            // exact sign/zero facts immediately and avoids a Ratio leaf in the
            // many higher-level code paths that still call `rational(0)`.
            return Self::zero();
        }
        if r.is_one() {
            // Route rational one through the dedicated One node so callers that
            // import exact f64/integer identities get the same cheap constructor
            // and structural facts as `Computable::one()`.
            return Self::one();
        }
        Self {
            internal: Box::new(Approximation::Ratio(r)),
            cache: RefCell::new(Cache::Invalid),
            bound: RefCell::new(BoundCache::Invalid),
            exact_sign: RefCell::new(ExactSignCache::Invalid),
            signal: None,
        }
    }
}

impl Computable {
    pub(crate) fn exp_rational(r: Rational) -> Self {
        if r.is_one() {
            // e^1 is hot enough to route to the shared e cache.
            Self::e_constant()
        } else {
            let rational = Self::rational(r);
            Self::exp(rational)
        }
    }

    fn shared_constant_kind(&self) -> Option<SharedConstant> {
        match &*self.internal {
            Approximation::Constant(constant) => Some(*constant),
            _ => None,
        }
    }

    fn power_of_two_rational(shift: Precision) -> Rational {
        if shift >= 0 {
            Rational::from_bigint(BigInt::one() << shift as usize)
        } else {
            Rational::from_bigint_fraction(BigInt::one(), BigUint::one() << (-shift) as usize)
                .unwrap()
        }
    }

    fn shared_constant_term(&self) -> Option<(SharedConstant, Rational)> {
        // Recognize "exact rational scale times one shared constant" through
        // lightweight wrappers. This supports pi-3/e-2 style sign certificates
        // without needing a full symbolic Real class.
        match &*self.internal {
            Approximation::Constant(constant) => Some((*constant, Rational::one())),
            Approximation::Negate(child) => {
                let (constant, scale) = child.shared_constant_term()?;
                Some((constant, scale.neg()))
            }
            Approximation::Offset(child, shift) => {
                let (constant, scale) = child.shared_constant_term()?;
                Some((constant, scale * Self::power_of_two_rational(*shift)))
            }
            Approximation::Multiply(left, right) => {
                if let Some(scale) = left.exact_rational() {
                    let (constant, inner_scale) = right.shared_constant_term()?;
                    return Some((constant, scale * inner_scale));
                }
                if let Some(scale) = right.exact_rational() {
                    let (constant, inner_scale) = left.shared_constant_term()?;
                    return Some((constant, scale * inner_scale));
                }
                None
            }
            _ => None,
        }
    }

    fn bound_from_strict_interval(lower: Rational, upper: Rational) -> BoundInfo {
        // Convert an interval that excludes zero into a reusable sign/MSD
        // certificate. If the interval crosses zero, preserve correctness by
        // returning Unknown.
        let zero = Rational::zero();
        let (sign, magnitude_lower, magnitude_upper) = if lower > zero {
            (Sign::Plus, lower, upper)
        } else if upper < zero {
            (Sign::Minus, upper.neg(), lower.neg())
        } else {
            return BoundInfo::Unknown;
        };

        let lower_msd = magnitude_lower.msd_exact();
        let upper_msd = magnitude_upper.msd_exact();
        let (msd, exact_msd) = match (lower_msd, upper_msd) {
            (Some(lower), Some(upper)) if lower == upper => (Some(lower), true),
            (Some(lower), Some(upper)) => (Some(lower.max(upper)), false),
            _ => (None, false),
        };

        BoundInfo::with_sign_msd(sign, msd, exact_msd)
    }

    fn constant_rational_sum_bound(
        term: &(SharedConstant, Rational),
        rational: &Rational,
    ) -> BoundInfo {
        // Specialized structural bound for c*K + q where K is a shared constant.
        // This is the computable-side companion to Real's ConstOffset class and
        // keeps generic Add nodes for pi-3 from needing approximation refinement.
        let (constant, scale) = term;
        let (lower, upper) = constant.interval();
        let scaled_lower = lower * scale;
        let scaled_upper = upper * scale;
        let (lower, upper) = if scaled_lower <= scaled_upper {
            (scaled_lower, scaled_upper)
        } else {
            (scaled_upper, scaled_lower)
        };

        Self::bound_from_strict_interval(lower + rational, upper + rational)
    }

    fn cached_at_precision(&self, p: Precision) -> Option<BigInt> {
        // A cached value at precision q can answer any less precise request p
        // by shifting, but not a more precise one. Shared constants use the
        // thread-local cache; other nodes keep their cache beside the node.
        if let Some(constant) = self.shared_constant_kind() {
            if let Some(cached) = Self::cached_shared_constant_at_precision(constant, p) {
                return Some(cached);
            }
            if constant == SharedConstant::Tau
                && let Some(cached) =
                    Self::cached_shared_constant_at_precision(SharedConstant::Pi, p - 1)
            {
                // tau is exactly 2*pi, so a pi approximation at precision p-1
                // is already a tau approximation at precision p. Populate the
                // tau cache from pi instead of re-running the Machin pi kernel
                // when callers ask for tau after pi has been warmed.
                Self::store_shared_constant_cache_value(SharedConstant::Tau, p, cached.clone());
                return Some(cached);
            }
            return None;
        }

        let cache = self.cache.borrow();
        let cached = if let Cache::Valid((cache_prec, cache_appr)) = &*cache {
            Some((*cache_prec, cache_appr.clone()))
        } else {
            None
        }?;

        if p >= cached.0 {
            Some(scale(cached.1, cached.0 - p))
        } else {
            None
        }
    }

    fn cached_shared_constant_at_precision(
        constant: SharedConstant,
        p: Precision,
    ) -> Option<BigInt> {
        SHARED_CONSTANT_CACHES.with(|caches| {
            let caches = caches.borrow();
            let Cache::Valid((cache_prec, cache_appr)) = &caches[constant.cache_index()] else {
                return None;
            };
            if p >= *cache_prec {
                Some(scale(cache_appr.clone(), *cache_prec - p))
            } else {
                None
            }
        })
    }

    fn store_shared_constant_cache_value(constant: SharedConstant, p: Precision, value: BigInt) {
        SHARED_CONSTANT_CACHES.with(|caches| {
            caches.borrow_mut()[constant.cache_index()] = Cache::Valid((p, value));
        });
    }

    fn store_cache_value(&self, p: Precision, value: BigInt) {
        // Store only exact node approximation results, not temporary scaled
        // values. For shared constants this updates the global thread-local
        // cache so every cloned constant wrapper benefits.
        if let Some(constant) = self.shared_constant_kind() {
            Self::store_shared_constant_cache_value(constant, p, value);
        } else {
            self.cache.replace(Cache::Valid((p, value)));
        }
    }

    fn cached_bound(&self) -> Option<BoundInfo> {
        let bound = self.bound.borrow();
        match &*bound {
            BoundCache::Invalid => None,
            BoundCache::Valid(info) => Some(info.clone()),
        }
    }

    fn store_bound(&self, info: &BoundInfo) {
        // Unknown facts are intentionally not cached; a later approximation may
        // discover a real sign/MSD and should be allowed to populate the cache.
        if *info != BoundInfo::Unknown {
            self.bound.replace(BoundCache::Valid(info.clone()));
        }
    }

    fn bound_from_approx(prec: Precision, appr: &BigInt) -> BoundInfo {
        // Approximation values with magnitude <= 1 are within the allowed error
        // band, so they cannot certify sign or nonzero status.
        if appr.abs() <= BigInt::one() {
            BoundInfo::Unknown
        } else {
            BoundInfo::with_sign_msd(
                appr.sign(),
                Some(prec + appr.magnitude().bits() as Precision - 1),
                false,
            )
        }
    }

    fn cheap_bound_shallow(&self, budget: usize) -> Option<BoundInfo> {
        // First try a shallow recursive walk. It is faster for common small
        // trees and avoids allocating the explicit stack used by deep chains.
        if let Some(info) = self.cached_bound() {
            return Some(info);
        }
        if budget == 0 {
            return None;
        }
        let info = match &*self.internal {
            Approximation::One => Some(BoundInfo::with_sign(Sign::Plus, Some(0))),
            Approximation::Int(n) => Some(if n.sign() == Sign::NoSign {
                BoundInfo::Zero
            } else {
                BoundInfo::with_sign(n.sign(), Some(n.magnitude().bits() as Precision - 1))
            }),
            Approximation::Constant(constant) => Some(constant.bound_info()),
            Approximation::Ratio(r) => Some(BoundInfo::from_rational(r)),
            Approximation::Negate(child) => {
                child.cheap_bound_shallow(budget - 1).map(BoundInfo::negate)
            }
            Approximation::Offset(child, n) => child
                .cheap_bound_shallow(budget - 1)
                .map(|bound| bound.map_msd(|value| value + *n)),
            Approximation::Inverse(child) => child
                .cheap_bound_shallow(budget - 1)
                .map(BoundInfo::inverse),
            Approximation::Square(child) => {
                child.cheap_bound_shallow(budget - 1).map(BoundInfo::square)
            }
            Approximation::Sqrt(child) => {
                child.cheap_bound_shallow(budget - 1).map(BoundInfo::sqrt)
            }
            Approximation::Multiply(left, right) => {
                let left = left.cheap_bound_shallow(budget - 1)?;
                let right = right.cheap_bound_shallow(budget - 1)?;
                Some(left.multiply(right))
            }
            Approximation::Add(left, right) => {
                let left = left.cheap_bound_shallow(budget - 1)?;
                let right = right.cheap_bound_shallow(budget - 1)?;
                Some(left.add(right))
            }
            _ => Some(if let Some((prec, appr)) = self.cached() {
                Self::bound_from_approx(prec, &appr)
            } else {
                BoundInfo::Unknown
            }),
        };
        if let Some(ref value) = info {
            self.store_bound(value);
        }
        info
    }

    fn cheap_bound(&self) -> BoundInfo {
        const SHALLOW_BOUND_BUDGET: usize = 24;

        // The public structural API leans on this method heavily. It must stay
        // conservative: a false NonZero or sign certificate is a correctness
        // bug, while Unknown only costs later refinement.
        if let Some(info) = self.cached_bound() {
            return info;
        }

        if let Some(bound) = self.cheap_bound_shallow(SHALLOW_BOUND_BUDGET) {
            return bound;
        }

        enum Frame<'a> {
            Eval(&'a Computable),
            FinishNegate,
            FinishOffset(i32),
            FinishInverse,
            FinishSquare,
            FinishSqrt,
            FinishAdd,
            FinishMultiply,
        }

        fn direct_bound(node: &Computable) -> Option<BoundInfo> {
            match &*node.internal {
                Approximation::One => Some(BoundInfo::with_sign(Sign::Plus, Some(0))),
                Approximation::Int(n) => Some(if n.sign() == Sign::NoSign {
                    BoundInfo::Zero
                } else {
                    BoundInfo::with_sign(n.sign(), Some(n.magnitude().bits() as Precision - 1))
                }),
                Approximation::Constant(constant) => Some(constant.bound_info()),
                Approximation::Ratio(r) => Some(BoundInfo::from_rational(r)),
                Approximation::Negate(_)
                | Approximation::Offset(_, _)
                | Approximation::Inverse(_)
                | Approximation::Square(_)
                | Approximation::Sqrt(_)
                | Approximation::Add(_, _)
                | Approximation::Multiply(_, _) => None,
                _ => Some(if let Some((prec, appr)) = node.cached() {
                    Computable::bound_from_approx(prec, &appr)
                } else {
                    BoundInfo::Unknown
                }),
            }
        }

        let mut frames = vec![Frame::Eval(self)];
        let mut values: Vec<BoundInfo> = Vec::new();

        // Deep addition/multiplication chains are common after algebra kernels.
        // Use an explicit stack so structural fact discovery cannot recurse
        // through thousands of nodes.
        while let Some(frame) = frames.pop() {
            match frame {
                Frame::Eval(node) => {
                    if let Some(bound) = direct_bound(node) {
                        values.push(bound);
                        continue;
                    }

                    match &*node.internal {
                        Approximation::Negate(child) => {
                            frames.push(Frame::FinishNegate);
                            frames.push(Frame::Eval(child));
                        }
                        Approximation::Offset(child, n) => {
                            frames.push(Frame::FinishOffset(*n));
                            frames.push(Frame::Eval(child));
                        }
                        Approximation::Inverse(child) => {
                            frames.push(Frame::FinishInverse);
                            frames.push(Frame::Eval(child));
                        }
                        Approximation::Square(child) => {
                            frames.push(Frame::FinishSquare);
                            frames.push(Frame::Eval(child));
                        }
                        Approximation::Sqrt(child) => {
                            frames.push(Frame::FinishSqrt);
                            frames.push(Frame::Eval(child));
                        }
                        Approximation::Add(left, right) => {
                            frames.push(Frame::FinishAdd);
                            frames.push(Frame::Eval(right));
                            frames.push(Frame::Eval(left));
                        }
                        Approximation::Multiply(left, right) => {
                            frames.push(Frame::FinishMultiply);
                            frames.push(Frame::Eval(right));
                            frames.push(Frame::Eval(left));
                        }
                        _ => unreachable!("direct_bound should handle non-structural nodes"),
                    }
                }
                Frame::FinishNegate => {
                    let value = values.pop().expect("negate bound should exist");
                    values.push(value.negate());
                }
                Frame::FinishOffset(offset) => {
                    let value = values.pop().expect("offset bound should exist");
                    values.push(value.map_msd(|msd| msd + offset));
                }
                Frame::FinishInverse => {
                    let value = values.pop().expect("inverse bound should exist");
                    values.push(value.inverse());
                }
                Frame::FinishSquare => {
                    let value = values.pop().expect("square bound should exist");
                    values.push(value.square());
                }
                Frame::FinishSqrt => {
                    let value = values.pop().expect("sqrt bound should exist");
                    values.push(value.sqrt());
                }
                Frame::FinishAdd => {
                    let right = values.pop().expect("add rhs bound should exist");
                    let left = values.pop().expect("add lhs bound should exist");
                    values.push(left.add(right));
                }
                Frame::FinishMultiply => {
                    let right = values.pop().expect("multiply rhs bound should exist");
                    let left = values.pop().expect("multiply lhs bound should exist");
                    values.push(left.multiply(right));
                }
            }
        }

        let result = values
            .pop()
            .expect("bound evaluation should produce a result");
        self.store_bound(&result);
        result
    }

    fn exact_sign(&self) -> Option<Sign> {
        // `exact_sign` is stronger than "current approximation sign": it means
        // the expression shape or a separated cached approximation proves the
        // sign. Unknown is cached separately so impossible structural proofs do
        // not repeat on every predicate query.
        let cached_sign = *self.exact_sign.borrow();
        match cached_sign {
            ExactSignCache::Valid(sign) => return Some(sign),
            ExactSignCache::Unknown => {
                if let Some((_, appr)) = self.cached()
                    && appr.abs() > BigInt::one()
                {
                    let sign = appr.sign();
                    self.exact_sign.replace(ExactSignCache::Valid(sign));
                    return Some(sign);
                }
                return None;
            }
            ExactSignCache::Invalid => {}
        }

        enum Frame<'a> {
            Eval(&'a Computable),
            FinishNegate(&'a Computable),
            FinishOffset(&'a Computable),
            FinishInverse(&'a Computable),
            FinishSquare(&'a Computable),
            FinishSqrt(&'a Computable),
            FinishAdd(&'a Computable),
            FinishMultiply(&'a Computable),
        }

        fn cached_exact_sign(node: &Computable) -> Option<Option<Sign>> {
            let cached_sign = *node.exact_sign.borrow();
            match cached_sign {
                ExactSignCache::Invalid => None,
                ExactSignCache::Unknown => {
                    if let Some((_, appr)) = node.cached()
                        && appr.abs() > BigInt::one()
                    {
                        let sign = appr.sign();
                        node.exact_sign.replace(ExactSignCache::Valid(sign));
                        Some(Some(sign))
                    } else {
                        Some(None)
                    }
                }
                ExactSignCache::Valid(sign) => Some(Some(sign)),
            }
        }

        fn exact_sign_direct(node: &Computable) -> Option<Option<Sign>> {
            // Direct cases either know their sign structurally or are known not
            // to be structurally decidable without visiting children.
            if let Some(sign) = cached_exact_sign(node) {
                return Some(sign);
            }

            if let Some((_, appr)) = node.cached()
                && appr.abs() > BigInt::one()
            {
                return Some(Some(appr.sign()));
            }

            match &*node.internal {
                Approximation::One => Some(Some(Sign::Plus)),
                Approximation::Int(n) => Some(Some(n.sign())),
                Approximation::Constant(_) => Some(Some(Sign::Plus)),
                Approximation::Ratio(r) => Some(Some(r.sign())),
                Approximation::IntegralAtan(n) => Some(Some(n.sign())),
                Approximation::AcosPositive(_)
                | Approximation::AcoshNearOne(_)
                | Approximation::AcoshDirect(_) => Some(Some(Sign::Plus)),
                Approximation::PrescaledAtan(child)
                | Approximation::PrescaledAsin(child)
                | Approximation::AsinhNearZero(child)
                | Approximation::AsinhDirect(child)
                | Approximation::PrescaledAsinh(child)
                | Approximation::AtanhDirect(child)
                | Approximation::PrescaledAtanh(child) => Some(child.exact_sign()),
                Approximation::Negate(_)
                | Approximation::Offset(_, _)
                | Approximation::Inverse(_)
                | Approximation::Square(_)
                | Approximation::Sqrt(_)
                | Approximation::Add(_, _)
                | Approximation::Multiply(_, _) => None,
                _ => Some(None),
            }
        }

        fn store_exact_sign(node: &Computable, sign: Option<Sign>) {
            node.exact_sign.replace(match sign {
                Some(sign) => ExactSignCache::Valid(sign),
                None => ExactSignCache::Unknown,
            });
        }

        let mut frames = vec![Frame::Eval(self)];
        let mut values: Vec<Option<Sign>> = Vec::new();

        // Mirror cheap_bound's nonrecursive traversal for deep structural
        // expressions. This matters for predicate-heavy code that asks only for
        // sign and never needs numeric approximation.
        while let Some(frame) = frames.pop() {
            match frame {
                Frame::Eval(node) => {
                    if let Some(sign) = exact_sign_direct(node) {
                        store_exact_sign(node, sign);
                        values.push(sign);
                        continue;
                    }

                    match &*node.internal {
                        Approximation::Negate(child) => {
                            frames.push(Frame::FinishNegate(node));
                            frames.push(Frame::Eval(child));
                        }
                        Approximation::Offset(child, _) => {
                            frames.push(Frame::FinishOffset(node));
                            frames.push(Frame::Eval(child));
                        }
                        Approximation::Inverse(child) => {
                            frames.push(Frame::FinishInverse(node));
                            frames.push(Frame::Eval(child));
                        }
                        Approximation::Square(child) => {
                            frames.push(Frame::FinishSquare(node));
                            frames.push(Frame::Eval(child));
                        }
                        Approximation::Sqrt(child) => {
                            frames.push(Frame::FinishSqrt(node));
                            frames.push(Frame::Eval(child));
                        }
                        Approximation::Add(left, right) => {
                            frames.push(Frame::FinishAdd(node));
                            frames.push(Frame::Eval(right));
                            frames.push(Frame::Eval(left));
                        }
                        Approximation::Multiply(left, right) => {
                            frames.push(Frame::FinishMultiply(node));
                            frames.push(Frame::Eval(right));
                            frames.push(Frame::Eval(left));
                        }
                        _ => unreachable!("exact_sign_direct should handle non-structural nodes"),
                    }
                }
                Frame::FinishNegate(node) => {
                    let value = values.pop().expect("negate sign should exist");
                    let result = value.map(negate_sign);
                    store_exact_sign(node, result);
                    values.push(result);
                }
                Frame::FinishOffset(node) => {
                    let value = values.pop().expect("offset sign should exist");
                    store_exact_sign(node, value);
                    values.push(value);
                }
                Frame::FinishInverse(node) => {
                    let value = values.pop().expect("inverse sign should exist");
                    let result = match value {
                        Some(Sign::Plus) => Some(Sign::Plus),
                        Some(Sign::Minus) => Some(Sign::Minus),
                        _ => None,
                    };
                    store_exact_sign(node, result);
                    values.push(result);
                }
                Frame::FinishSquare(node) => {
                    let value = values.pop().expect("square sign should exist");
                    let result = match value {
                        Some(Sign::NoSign) => Some(Sign::NoSign),
                        Some(_) => Some(Sign::Plus),
                        None => None,
                    };
                    store_exact_sign(node, result);
                    values.push(result);
                }
                Frame::FinishSqrt(node) => {
                    let value = values.pop().expect("sqrt sign should exist");
                    let result = match value {
                        Some(Sign::NoSign) => Some(Sign::NoSign),
                        Some(Sign::Plus) => Some(Sign::Plus),
                        _ => None,
                    };
                    store_exact_sign(node, result);
                    values.push(result);
                }
                Frame::FinishAdd(node) => {
                    let right = values.pop().expect("add rhs sign should exist");
                    let left = values.pop().expect("add lhs sign should exist");
                    let result = match (left, right) {
                        (Some(Sign::NoSign), sign) | (sign, Some(Sign::NoSign)) => sign,
                        (Some(Sign::Plus), Some(Sign::Plus)) => Some(Sign::Plus),
                        (Some(Sign::Minus), Some(Sign::Minus)) => Some(Sign::Minus),
                        _ => None,
                    };
                    store_exact_sign(node, result);
                    values.push(result);
                }
                Frame::FinishMultiply(node) => {
                    let right = values.pop().expect("multiply rhs sign should exist");
                    let left = values.pop().expect("multiply lhs sign should exist");
                    let result = match (left, right) {
                        (Some(Sign::NoSign), _) | (_, Some(Sign::NoSign)) => Some(Sign::NoSign),
                        (Some(Sign::Plus), Some(Sign::Plus))
                        | (Some(Sign::Minus), Some(Sign::Minus)) => Some(Sign::Plus),
                        (Some(Sign::Plus), Some(Sign::Minus))
                        | (Some(Sign::Minus), Some(Sign::Plus)) => Some(Sign::Minus),
                        _ => None,
                    };
                    store_exact_sign(node, result);
                    values.push(result);
                }
            }
        }

        let result = values
            .pop()
            .expect("exact sign evaluation should produce a result");
        store_exact_sign(self, result);
        result
    }

    #[cfg(test)]
    pub(super) fn planning_msd(&self) -> Option<Option<Precision>> {
        self.cheap_bound().planning_msd()
    }

    pub(super) fn planning_sign_and_msd(&self) -> (Option<Sign>, Option<Option<Precision>>) {
        let bound = self.cheap_bound();
        (bound.known_sign(), bound.planning_msd())
    }

    fn exact_rational(&self) -> Option<Rational> {
        // Only exact leaf nodes are exposed here. Keeping this narrow prevents
        // constructor shortcuts from accidentally forcing approximation of a
        // composite just to discover that it is not rational.
        match &*self.internal {
            Approximation::One => Some(Rational::one()),
            Approximation::Int(n) => Some(Rational::from_bigint(n.clone())),
            Approximation::Ratio(r) => Some(r.clone()),
            _ => None,
        }
    }

    fn integer_ratio_nearest(&self, divisor: Computable) -> BigInt {
        // Low-precision nearest-integer quotient used only for range reduction.
        // It deliberately asks for a rough result and relies on caller-side
        // correction instead of doing an expensive exact division. The trig
        // callers use this as hyperreal's exact-real analogue of Payne-Hanek
        // radian reduction: compute enough quotient bits, then correct from the
        // residual. Payne/Hanek: https://doi.org/10.1145/1057600.1057602.
        let quotient = self.clone().multiply(divisor.inverse());
        scale(quotient.approx(-4), -4)
    }

    /// Natural Exponential function, raise Euler's Number to this number.
    pub fn exp(self) -> Computable {
        if self.exact_rational().as_ref().is_some_and(Rational::is_one) {
            // e^1 is the shared cached constant, not a fresh PrescaledExp node.
            return Self::e_constant();
        }
        if self
            .exact_rational()
            .is_some_and(|r| r.sign() == Sign::NoSign)
        {
            // e^0 is exact and must stay outside the approximation pipeline.
            return Self::one();
        }
        let low_prec: Precision = -4;
        let rough_appr: BigInt = self.approx(low_prec);
        // At precision -4, an approximation outside +/-8 implies |x| > 0.5.
        if rough_appr > *signed::EIGHT || rough_appr < -signed::EIGHT.clone() {
            // Keep the Taylor kernel near zero by subtracting k*ln(2), then reapply
            // the scale as a binary shift. This avoids slow huge-argument series work.
            // This is the standard exp argument reduction described in Brent,
            // https://doi.org/10.1145/321941.321944.
            let ln2 = Self::ln2();
            let mut multiple = self.integer_ratio_nearest(ln2.clone());

            loop {
                let adjustment = ln2
                    .clone()
                    .multiply(Self::rational(Rational::from_bigint(multiple.clone())).negate());
                let reduced = self.clone().add(adjustment);
                let reduced_appr = reduced.approx(low_prec);

                if reduced_appr > *signed::EIGHT {
                    multiple += 1;
                    continue;
                }
                if reduced_appr < -signed::EIGHT.clone() {
                    multiple -= 1;
                    continue;
                }

                return reduced.exp().shift_left(
                    multiple
                        .try_into()
                        .expect("binary exponent should fit in i32"),
                );
            }
        }

        Self {
            internal: Box::new(Approximation::PrescaledExp(self)),
            cache: RefCell::new(Cache::Invalid),
            bound: RefCell::new(BoundCache::Invalid),
            exact_sign: RefCell::new(ExactSignCache::Invalid),
            signal: None,
        }
    }

    /// Calculate nearby multiple of pi.
    fn pi_multiple(&self) -> BigInt {
        // Use one low-precision quotient and a cheap correction instead of a full
        // high-precision division. Trig reduction calls this on hot paths; the
        // quotient/residual structure is the same problem addressed by
        // Payne-Hanek range reduction, https://doi.org/10.1145/1057600.1057602.
        let mut multiple = self.integer_ratio_nearest(Self::pi());
        let adjustment =
            Self::pi().multiply(Self::rational(Rational::from_bigint(multiple.clone())).negate());
        let rough_appr = self.clone().add(adjustment).approx(-1);

        if rough_appr >= *signed::SIX {
            multiple += 1;
        } else if rough_appr <= -signed::SIX.clone() {
            multiple -= 1;
        }

        multiple
    }

    /// Calculate nearby multiple of pi/2.
    fn half_pi_multiple(&self) -> BigInt {
        // Same nearest-multiple trick as `pi_multiple`, specialized for the quadrant
        // reductions used by sin/cos. Exact-rational inputs first try an integer
        // quotient against cached pi so huge arguments avoid constructing
        // x*(pi/2)^-1, then the residual correction validates the quadrant.
        let half_pi = Self::pi().shift_right(1);
        let mut multiple = self
            .exact_rational()
            .and_then(|rational| Self::half_pi_multiple_exact_rational(&rational))
            .unwrap_or_else(|| self.integer_ratio_nearest(half_pi.clone()));
        let adjustment =
            half_pi.multiply(Self::rational(Rational::from_bigint(multiple.clone())).negate());
        let rough_appr = self.clone().add(adjustment).approx(-1);

        if rough_appr >= *signed::TWO {
            multiple += 1;
        } else if rough_appr <= -signed::TWO.clone() {
            multiple -= 1;
        }

        multiple
    }

    fn half_pi_multiple_exact_rational(rational: &Rational) -> Option<BigInt> {
        // Large exact rationals are the hot scalar sin/cos construction path.
        // Estimate round(2*x/pi) with one cached pi approximation and integer
        // arithmetic, then let half_pi_multiple's residual correction validate
        // the result. This avoids building and approximating x * (pi/2)^-1.
        // It is a lightweight exact-rational variant of Payne-Hanek radian
        // reduction: https://doi.org/10.1145/1057600.1057602.
        let msd = rational.msd_exact()?;
        if msd < 3 {
            return None;
        }

        let precision_bits = msd.checked_add(16)?.max(16);
        let shift = usize::try_from(precision_bits).ok()?;
        let pi_scaled = Self::pi().approx(-precision_bits).to_biguint()?;
        let numerator = rational.numerator() << (shift + 1);
        let denominator = rational.denominator() * &pi_scaled;
        if denominator.is_zero() {
            return None;
        }

        let rounded = (&numerator + (&denominator >> 1_usize)) / &denominator;
        Some(BigInt::from_biguint(rational.sign(), rounded))
    }

    fn medium_half_pi_multiple(rough_appr: &BigInt) -> BigInt {
        // For medium arguments the rough approximation already distinguishes the only
        // useful half-pi multiples, avoiding a second approximation of x/(pi/2).
        let positive = rough_appr.sign() != Sign::Minus;
        let magnitude = rough_appr.magnitude();
        let multiple = if magnitude < unsigned::FIVE.deref() {
            signed::ONE.clone()
        } else {
            signed::TWO.clone()
        };

        if positive { multiple } else { -multiple }
    }

    fn known_msd_for_trig_reduction(&self) -> Option<Option<Precision>> {
        // Trig construction used to call `approx(-1)` before deciding whether
        // an argument was already small or definitely huge. Exact rationals and
        // shared constants already carry enough structural MSD information, so
        // using it here avoids an extra approximation pass for generic scalar
        // sin/cos rows such as 1e6 and 1e30.
        match &*self.internal {
            Approximation::One => Some(Some(0)),
            Approximation::Int(n) => Some(if n.sign() == Sign::NoSign {
                None
            } else {
                Some(n.magnitude().bits() as Precision - 1)
            }),
            Approximation::Ratio(r) => Some(r.msd_exact()),
            Approximation::Constant(constant) => constant.bound_info().known_msd(),
            _ => self.cheap_bound().known_msd(),
        }
    }

    fn trig_reduction_msd(&self) -> Option<Precision> {
        self.known_msd_for_trig_reduction().flatten()
    }

    fn exact_rational_half_pi_shortcut_magnitude(rational: &Rational) -> Option<Rational> {
        // Exact rationals with 1 <= |x| < 3/2 are the awkward medium trig rows:
        // not small enough for direct sin/cos, but close enough to pi/2 that a
        // dedicated residual beats full half-pi reduction and generic Add setup.
        if rational.msd_exact() != Some(0) {
            return None;
        }

        let magnitude = if rational.sign() == Sign::Minus {
            -rational.clone()
        } else {
            rational.clone()
        };
        if magnitude < *HALF_PI_SHORTCUT_RATIONAL_LIMIT {
            Some(magnitude)
        } else {
            None
        }
    }

    fn cos_reduced_by_half_pi(self, multiplier: BigInt) -> Computable {
        let adjustment = Self::pi()
            .shift_right(1)
            .multiply(Self::rational(Rational::from_bigint(multiplier.clone())).negate());
        let reduced = self.add(adjustment);
        // Reduce the nearest half-pi multiple modulo four and dispatch by the
        // exact sin/cos symmetry. This keeps the residual kernel below one radian.
        let quadrant =
            ((&multiplier % signed::FOUR.deref()) + signed::FOUR.deref()) % signed::FOUR.deref();

        if quadrant.is_zero() {
            reduced.cos()
        } else if quadrant == *signed::ONE {
            reduced.sin().negate()
        } else if quadrant == *signed::TWO {
            reduced.cos().negate()
        } else {
            reduced.sin()
        }
    }

    /// Cosine of this number.
    pub fn cos(self) -> Computable {
        if let Some(rational) = self.exact_rational() {
            if rational.sign() == Sign::NoSign {
                return Self::one();
            }
            if let Some(magnitude) = Self::exact_rational_half_pi_shortcut_magnitude(&rational) {
                // cos(r) = sin(pi/2 - |r|) for exact medium positive/negative
                // rationals, keeping the generic subtraction node out of the path.
                return Self::prescaled_sin_half_pi_minus_rational(magnitude);
            }
        }
        if let Some(msd) = self.trig_reduction_msd() {
            if msd < 0 {
                // Known |x| < 1: go directly to the prescaled Taylor kernel.
                // The fallback rough approximation stays in place for unknown
                // magnitudes where structural bounds are not trustworthy.
                return Self::prescaled_cos(self);
            }
            if msd >= 3 {
                // Known |x| >= 8: skip the preliminary `approx(-1)` and go
                // straight to half-pi reduction. This is the hot large-argument
                // path for generic sin/cos benchmarks.
                let multiplier = Self::half_pi_multiple(&self);
                return self.cos_reduced_by_half_pi(multiplier);
            }
        }
        let rough_appr = self.approx(-1);
        let abs_rough_appr = rough_appr.magnitude();

        if abs_rough_appr < unsigned::TWO.deref() {
            return Self::prescaled_cos(self);
        }

        let multiplier = if abs_rough_appr < unsigned::SIX.deref() {
            // Medium arguments can reuse the rough quadrant table. Larger values need the
            // more expensive nearest-half-pi reduction to keep the residual small.
            Self::medium_half_pi_multiple(&rough_appr)
        } else {
            Self::half_pi_multiple(&self)
        };
        self.cos_reduced_by_half_pi(multiplier)
    }

    /// Sine of this number.
    pub fn sin(self) -> Computable {
        if let Some(rational) = self.exact_rational() {
            if rational.sign() == Sign::NoSign {
                return Self::zero();
            }
            if let Some(magnitude) = Self::exact_rational_half_pi_shortcut_magnitude(&rational) {
                // sin(r) = +/-cos(pi/2 - |r|) in the same exact medium window
                // used by cosine, preserving odd symmetry outside the kernel.
                let result = Self::prescaled_cos_half_pi_minus_rational(magnitude);
                return if rational.sign() == Sign::Minus {
                    result.negate()
                } else {
                    result
                };
            }
        }
        if let Some(msd) = self.trig_reduction_msd() {
            if msd < 0 {
                // Known |x| < 1: direct prescaled sine avoids reduction setup.
                return Self::prescaled_sin(self);
            }
            if msd >= 3 {
                // Known large input: avoid the extra rough approximation and
                // reduce by half-pi immediately.
                let multiplier = Self::half_pi_multiple(&self);
                let adjustment = Self::pi()
                    .shift_right(1)
                    .multiply(Self::rational(Rational::from_bigint(multiplier.clone())).negate());
                let reduced = self.add(adjustment);
                let quadrant = ((&multiplier % signed::FOUR.deref()) + signed::FOUR.deref())
                    % signed::FOUR.deref();

                if quadrant.is_zero() {
                    return reduced.sin();
                } else if quadrant == *signed::ONE {
                    return reduced.cos();
                } else if quadrant == *signed::TWO {
                    return reduced.sin().negate();
                } else {
                    return reduced.cos().negate();
                }
            }
        }
        let rough_appr = self.approx(-1);
        let abs_rough_appr = rough_appr.magnitude();

        if abs_rough_appr < unsigned::TWO.deref() {
            return Self::prescaled_sin(self);
        }

        if abs_rough_appr < unsigned::SIX.deref() {
            // Medium sine inputs are rewritten through exact symmetries instead of going
            // through the generic half-pi division path.
            let multiplier = Self::medium_half_pi_multiple(&rough_appr);
            if multiplier == *signed::ONE {
                return Self::pi().shift_right(1).add(self.negate()).cos();
            } else if multiplier == *signed::MINUS_ONE {
                return Self::pi().shift_right(1).add(self).cos().negate();
            } else if multiplier == *signed::TWO {
                return Self::pi().add(self.negate()).sin();
            } else {
                return Self::pi().add(self).sin().negate();
            }
        }

        let multiplier = Self::half_pi_multiple(&self);
        let adjustment = Self::pi()
            .shift_right(1)
            .multiply(Self::rational(Rational::from_bigint(multiplier.clone())).negate());
        let reduced = self.add(adjustment);
        let quadrant =
            ((&multiplier % signed::FOUR.deref()) + signed::FOUR.deref()) % signed::FOUR.deref();

        if quadrant.is_zero() {
            reduced.sin()
        } else if quadrant == *signed::ONE {
            reduced.cos()
        } else if quadrant == *signed::TWO {
            reduced.sin().negate()
        } else {
            reduced.cos().negate()
        }
    }

    /// Tangent of this number.
    pub fn tan(self) -> Computable {
        if self
            .exact_rational()
            .is_some_and(|r| r.sign() == Sign::NoSign)
        {
            return Self::zero();
        }
        if self.planning_sign_and_msd().0 == Some(Sign::Minus) {
            // Odd symmetry lets known-negative values reuse the positive reducer
            // without paying a low-precision approximation just to discover sign.
            return self.negate().tan().negate();
        }
        if let Some(msd) = self.trig_reduction_msd() {
            if msd < 0 {
                // Known |x| < 1: enter the tangent quotient kernel directly.
                return Self {
                    internal: Box::new(Approximation::PrescaledTan(self)),
                    cache: RefCell::new(Cache::Invalid),
                    bound: RefCell::new(BoundCache::Invalid),
                    exact_sign: RefCell::new(ExactSignCache::Invalid),
                    signal: None,
                };
            }
        }
        let rough_appr = self.approx(-1);
        if rough_appr.sign() == Sign::Minus {
            return self.negate().tan().negate();
        }

        let abs_rough_appr = rough_appr.magnitude();

        if abs_rough_appr < unsigned::TWO.deref() {
            return Self {
                internal: Box::new(Approximation::PrescaledTan(self)),
                cache: RefCell::new(Cache::Invalid),
                bound: RefCell::new(BoundCache::Invalid),
                exact_sign: RefCell::new(ExactSignCache::Invalid),
                signal: None,
            };
        }

        if abs_rough_appr < unsigned::FIVE.deref() {
            // Near pi/2, cotangent of the complement converges faster and avoids the
            // unstable generic tan series at the pole.
            let complement = Self::pi().shift_right(1).add(self.negate());
            return Self {
                internal: Box::new(Approximation::PrescaledCot(complement)),
                cache: RefCell::new(Cache::Invalid),
                bound: RefCell::new(BoundCache::Invalid),
                exact_sign: RefCell::new(ExactSignCache::Invalid),
                signal: None,
            };
        }

        if abs_rough_appr < unsigned::SIX.deref() {
            // Near pi, reflect back to a small tangent argument.
            return Self::pi().add(self.negate()).tan().negate();
        }

        let multiplier = Self::pi_multiple(&self);
        let adjustment =
            Self::pi().multiply(Self::rational(Rational::from_bigint(multiplier)).negate());
        self.add(adjustment).tan()
    }

    fn ln2() -> Self {
        Self::shared_constant(SharedConstant::Ln2)
    }

    fn ln_exact_rational(rational: Rational) -> Self {
        // Internal exact-rational log constructor for reductions that already
        // have a positive rational argument. It reuses the shared small-log
        // constants instead of building fresh generic PrescaledLn trees.
        if rational.is_one() {
            return Self::zero();
        }
        if rational.sign() == Sign::Minus || rational.sign() == Sign::NoSign {
            panic!("ArithmeticException");
        }
        if rational < Rational::one() {
            return Self::ln_exact_rational(rational.inverse().unwrap()).negate();
        }
        if rational == Rational::new(2) {
            return Self::ln_constant(2).unwrap();
        }
        if rational == Rational::new(3) {
            return Self::ln_constant(3).unwrap();
        }
        if rational == Rational::new(5) {
            return Self::ln_constant(5).unwrap();
        }
        if rational == Rational::new(6) {
            return Self::ln_constant(6).unwrap();
        }
        if rational == Rational::new(7) {
            return Self::ln_constant(7).unwrap();
        }
        if rational == Rational::new(10) {
            return Self::ln_constant(10).unwrap();
        }
        Self::rational(rational).ln()
    }

    /// Natural logarithm of this number.
    pub fn ln(self) -> Computable {
        if self.exact_rational().is_some_and(|r| r.is_one()) {
            return Self::zero();
        }
        if let Approximation::Ratio(r) = &*self.internal
            && r.sign() == Sign::Plus
        {
            let (shift, reduced) = r.factor_two_powers();
            if shift != 0 {
                // ln(r * 2^k) = ln(r) + k ln(2). Pulling dyadic scale out keeps
                // f64-derived rationals on a cheap symbolic/log path.
                let reduced_ln = if reduced.is_one() {
                    Self::integer(BigInt::zero())
                } else {
                    Self::rational(reduced).ln()
                };
                let shift: BigInt = shift.into();
                return reduced_ln.add(Self::integer(shift).multiply(Self::ln2()));
            }
        }

        // Sixteenths, ie 8 == 0.5, 24 == 1.5
        let low_ln_limit = signed::EIGHT.deref();
        let high_ln_limit = signed::TWENTY_FOUR.deref();

        let low_prec = -4;
        let rough_appr = self.approx(low_prec);
        if rough_appr < BigInt::zero() {
            panic!("ArithmeticException");
        }
        if rough_appr <= *low_ln_limit {
            // For values below 0.5, invert and negate so the prescaled ln1p kernel sees a
            // better-conditioned argument.
            return self.inverse().ln().negate();
        }
        if rough_appr >= *high_ln_limit {
            // Sixteenths, ie 64 == 4.0
            let sixty_four = signed::SIXTY_FOUR.deref();

            if rough_appr <= *sixty_four {
                // Moderate large values use repeated sqrt: ln(x) = 4 ln(sqrt(sqrt(x))).
                // That is cheaper than running ln1p far from one. This is a
                // local low-overhead form of logarithm argument reduction; see
                // Brent/Zimmermann Ch. 4:
                // https://maths-people.anu.edu.au/~brent/pd/mca-cup-0.5.9.pdf.
                let quarter = self.sqrt().sqrt().ln();
                return quarter.shift_left(2);
            } else {
                // Very large values are scaled by powers of two before ln1p, then the
                // binary exponent is added back as k ln(2). This keeps the
                // final ln1p kernel in its documented convergence interval.
                let mut extra_bits: i32 = (rough_appr.bits() - 5).try_into().expect(
                    "Approximation should have few enough bits to fit in a 32-bit signed integer",
                );

                let mut scaled = self.clone().shift_right(extra_bits);
                let mut scaled_rough = scaled.approx(low_prec);
                // The final branch below computes ln(1+x), and requires |x| < 1/2.
                // A bit-length estimate can leave scaled values in [1.5, 2), so
                // verify the low-precision scaled value before recursing.
                while scaled_rough >= *high_ln_limit {
                    extra_bits = extra_bits.checked_add(1).expect(
                        "Approximation should have few enough bits to fit in a 32-bit signed integer",
                    );
                    scaled = self.clone().shift_right(extra_bits);
                    scaled_rough = scaled.approx(low_prec);
                }

                let scaled_result = scaled.ln();
                let extra: BigInt = extra_bits.into();
                return scaled_result.add(Self::integer(extra).multiply(Self::ln2()));
            }
        }

        let minus_one = Self::integer(signed::MINUS_ONE.clone());
        let fraction = Self::add(self, minus_one);
        // Final path is ln(1+x), where the prior reductions keep |x| small enough for the
        // prescaled series.
        Self::prescaled_ln(fraction)
    }

    fn prescaled_ln(self) -> Self {
        // Private constructor for ln(1+x). Public ln range reduction must run
        // first so this node never sees an arbitrary positive value.
        Self {
            internal: Box::new(Approximation::PrescaledLn(self)),
            cache: RefCell::new(Cache::Invalid),
            bound: RefCell::new(BoundCache::Invalid),
            exact_sign: RefCell::new(ExactSignCache::Invalid),
            signal: None,
        }
    }

    pub(crate) fn ln_1p(self) -> Self {
        // Exposed internally for inverse-hyperbolic endpoint transforms that
        // have already constructed the small x in ln(1+x).
        self.prescaled_ln()
    }

    pub(crate) fn sqrt_rational(r: Rational) -> Self {
        // Preserve the rational leaf so sqrt can still collapse perfect
        // rational squares before allocating a generic Sqrt node.
        let rational = Self::rational(r);
        Self::sqrt(rational)
    }

    /// Square root of this number.
    pub fn sqrt(self) -> Computable {
        if let Approximation::Square(child) = self.internal.as_ref() {
            // sqrt(x^2) can collapse to abs(x) when the sign is structurally known.
            match child.exact_sign() {
                Some(Sign::Plus) => return child.clone(),
                Some(Sign::Minus) => return child.clone().negate(),
                Some(Sign::NoSign) => return Self::zero(),
                None => {}
            }
        }
        if let Approximation::Multiply(left, right) = self.internal.as_ref() {
            let reduced = |scale: Rational, square_side: &Computable| {
                // Recognize c*x^2 where c is an exact square, preserving the symbolic x
                // instead of introducing a generic sqrt node.
                let (root, rest) = scale.extract_square_reduced();
                if !rest.is_one() {
                    return None;
                }
                let Approximation::Square(child) = square_side.internal.as_ref() else {
                    return None;
                };
                match child.exact_sign() {
                    Some(Sign::Plus) => Some(child.clone().multiply(Self::rational(root))),
                    Some(Sign::Minus) => {
                        Some(child.clone().negate().multiply(Self::rational(root)))
                    }
                    Some(Sign::NoSign) => Some(Self::zero()),
                    None => None,
                }
            };

            if let Some(scale) = left.exact_rational()
                && let Some(value) = reduced(scale, right)
            {
                return value;
            }
            if let Some(scale) = right.exact_rational()
                && let Some(value) = reduced(scale, left)
            {
                return value;
            }
        }
        if let Some(rational) = self.exact_rational()
            && rational.sign() != Sign::Minus
            && rational.extract_square_will_succeed()
        {
            // Perfect rational squares stay exact.
            let (root, rest) = rational.extract_square_reduced();
            if rest.is_one() {
                return Self::rational(root);
            }
        }
        Self {
            internal: Box::new(Approximation::Sqrt(self)),
            cache: RefCell::new(Cache::Invalid),
            bound: RefCell::new(BoundCache::Invalid),
            exact_sign: RefCell::new(ExactSignCache::Invalid),
            signal: None,
        }
    }

    fn prescaled_atan(n: BigInt) -> Self {
        // atan(1/n) kernel used by pi and atan reduction constants. Passing the
        // denominator as an integer keeps the series loop division-only.
        Self {
            internal: Box::new(Approximation::IntegralAtan(n)),
            cache: RefCell::new(Cache::Invalid),
            bound: RefCell::new(BoundCache::Invalid),
            exact_sign: RefCell::new(ExactSignCache::Invalid),
            signal: None,
        }
    }

    /// Arctangent of this number.
    pub fn atan(self) -> Computable {
        if let Some(rational) = self.exact_rational() {
            if rational.sign() == Sign::NoSign {
                return Self::zero();
            }
            if rational.sign() == Sign::Plus
                && let Some(msd) = rational.msd_exact()
            {
                if msd <= -2 {
                    // Exact |x| < 1/2 can bypass the rough p=-4 probe.
                    return Self {
                        internal: Box::new(Approximation::PrescaledAtan(self)),
                        cache: RefCell::new(Cache::Invalid),
                        bound: RefCell::new(BoundCache::Invalid),
                        exact_sign: RefCell::new(ExactSignCache::Invalid),
                        signal: None,
                    };
                }
                if msd >= 1 {
                    // Exact |x| >= 2 is safely in the reciprocal branch.
                    return Self::pi()
                        .shift_right(1)
                        .add(self.inverse().atan().negate());
                }
            }
        }
        if self.exact_sign() == Some(Sign::Minus) {
            return self.negate().atan().negate();
        }

        let rough_appr = self.approx(-4);
        if rough_appr <= *signed::EIGHT {
            // Small atan arguments use the prescaled series directly.
            return Self {
                internal: Box::new(Approximation::PrescaledAtan(self)),
                cache: RefCell::new(Cache::Invalid),
                bound: RefCell::new(BoundCache::Invalid),
                exact_sign: RefCell::new(ExactSignCache::Invalid),
                signal: None,
            };
        }

        let one = Self::one();
        let half = one.clone().shift_right(1);
        if rough_appr <= *signed::SIXTEEN {
            // For middle-sized arguments, subtract atan(1/2) before recursing. This keeps
            // the residual small without jumping all the way to the reciprocal identity.
            // This follows the range-reduce-before-series pattern in Brent,
            // https://doi.org/10.1145/321941.321944.
            let numerator = self.clone().add(half.clone().negate());
            let denominator = one.add(self.multiply(half));
            return Self::prescaled_atan(BigInt::from(2_u8))
                .add(numerator.multiply(denominator.inverse()).atan());
        }

        // Large positive atan uses pi/2 - atan(1/x), which converges faster.
        Self::pi()
            .shift_right(1)
            .add(self.inverse().atan().negate())
    }

    /// Inverse sine of this number.
    pub fn asin(self) -> Computable {
        if let Some(rational) = self.exact_rational() {
            match rational.sign() {
                Sign::NoSign => return Self::zero(),
                Sign::Minus => return self.negate().asin().negate(),
                Sign::Plus => {
                    if rational.msd_exact().is_some_and(|msd| msd <= -4) {
                        // Tiny asin(x) is handled by its dedicated series; the generic
                        // atan transform builds extra sqrt/division nodes.
                        return Self {
                            internal: Box::new(Approximation::PrescaledAsin(self)),
                            cache: RefCell::new(Cache::Invalid),
                            bound: RefCell::new(BoundCache::Invalid),
                            exact_sign: RefCell::new(ExactSignCache::Invalid),
                            signal: None,
                        };
                    }
                    if rational >= *INVERSE_ENDPOINT_RATIONAL_THRESHOLD {
                        // Near 1, use pi/2 - acos(x); acos has the endpoint transform.
                        return Self::pi().shift_right(1).add(self.acos().negate());
                    }
                }
            }
        }
        if self.exact_sign() == Some(Sign::Minus) {
            return self.negate().asin().negate();
        }

        let one = Self::one();
        let denominator = one
            .clone()
            .add(self.clone().square().negate())
            .sqrt()
            .add(one);
        self.multiply(denominator.inverse()).atan().shift_left(1)
    }

    /// Inverse cosine of this number.
    pub fn acos(self) -> Computable {
        if let Some(rational) = self.exact_rational() {
            if rational.is_one() {
                return Self::zero();
            }
            if rational == Rational::new(-1) {
                return Self::pi();
            }
            if rational.sign() == Sign::NoSign {
                return Self::pi().shift_right(1);
            }
            let rational_sign = rational.sign();
            let magnitude = if rational_sign == Sign::Minus {
                rational.neg()
            } else {
                rational
            };
            if magnitude.msd_exact().is_some_and(|msd| msd <= -4) {
                return Self::pi().shift_right(1).add(self.asin().negate());
            }
            if rational_sign == Sign::Minus && magnitude >= *INVERSE_ENDPOINT_RATIONAL_THRESHOLD {
                // Negative endpoint values mirror the positive endpoint transform.
                // Building pi - acos(|x|) avoids the longer pi/2 - asin(-x) chain.
                return Self::pi().add(self.negate().acos().negate());
            }
        }

        if self.exact_sign() == Some(Sign::Plus) {
            // For positive values, acos(x) = 2 atan(sqrt((1-x)/(1+x))). This is the
            // endpoint-friendly path for values near 1.
            return Self::acos_positive(self);
        }

        Self::pi().shift_right(1).add(self.asin().negate())
    }

    /// Inverse hyperbolic sine of this number.
    pub fn asinh(self) -> Computable {
        let exact_rational = self.exact_rational();
        if exact_rational
            .as_ref()
            .is_some_and(|r| r.sign() == Sign::NoSign)
        {
            return Self::zero();
        }
        let (known_sign, planned_msd) = self.planning_sign_and_msd();
        if exact_rational
            .as_ref()
            .is_some_and(|r| r.sign() == Sign::Minus)
            || known_sign == Some(Sign::Minus)
        {
            return self.negate().asinh().negate();
        }
        let exact_tiny = exact_rational
            .as_ref()
            .and_then(Rational::msd_exact)
            .is_some_and(|msd| msd <= -4);
        let exact_large = exact_rational
            .as_ref()
            .and_then(Rational::msd_exact)
            .is_some_and(|msd| msd >= 3);
        if exact_tiny {
            return Self::prescaled_asinh(self);
        }
        if exact_large {
            let radicand = self.clone().square().add(Self::one());
            return self.add(radicand.sqrt()).ln();
        }
        let known_msd = planned_msd.flatten();
        if known_msd.is_none_or(|msd| msd < 3) && self.approx(-4) <= BigInt::from(64_u8) {
            // Direct Computable approximation benches include construction in
            // the measured work, and the eager graph caches its children better
            // than a deferred Real-only wrapper.
            let square = self.clone().square();
            let denominator = square.clone().add(Self::one()).sqrt().add(Self::one());
            return self.add(square.multiply(denominator.inverse())).ln_1p();
        }

        let radicand = self.clone().square().add(Self::one());
        self.add(radicand.sqrt()).ln()
    }

    /// Inverse hyperbolic cosine of this number. The caller is responsible for
    /// ensuring the input is in-domain.
    pub fn acosh(self) -> Computable {
        let exact_rational_msd = match self.internal.as_ref() {
            Approximation::One => return Self::zero(),
            Approximation::Ratio(r) => {
                if r.is_one() {
                    return Self::zero();
                }
                r.msd_exact()
            }
            Approximation::Int(n) => {
                if n == signed::ONE.deref() {
                    return Self::zero();
                }
                if n.sign() == Sign::NoSign {
                    None
                } else {
                    Some(n.magnitude().bits() as Precision - 1)
                }
            }
            _ => None,
        };
        if exact_rational_msd.is_some_and(|msd| msd >= 3) {
            // Large exact rationals skip the low-precision near-one probe and
            // use the direct acosh identity.
            let one = Self::one();
            let radicand = self.clone().square().add(one.negate());
            return self.add(radicand.sqrt()).ln();
        }
        let known_msd = self.planning_sign_and_msd().1.flatten();
        if known_msd.is_none_or(|msd| msd < 3) && self.approx(-4) <= BigInt::from(64_u8) {
            // Keep the public Computable kernel eager for approximation-heavy
            // benches; Real uses a deferred wrapper when construction alone is
            // the hot path.
            let one = Self::one();
            let shifted = self.clone().add(one.clone().negate());
            let radicand = self.square().add(one.negate());
            return shifted.add(radicand.sqrt()).ln_1p();
        }

        // Generic identity for already validated large inputs.
        let one = Self::one();
        let radicand = self.clone().square().add(one.negate());
        self.add(radicand.sqrt()).ln()
    }

    /// Inverse hyperbolic tangent of this number. The caller is responsible for
    /// ensuring the input is in-domain.
    pub fn atanh(self) -> Computable {
        if let Some(rational) = self.exact_rational() {
            match rational.sign() {
                Sign::NoSign => return Self::zero(),
                Sign::Minus => return self.negate().atanh().negate(),
                Sign::Plus => {
                    if rational.msd_exact().is_some_and(|msd| msd <= -4) {
                        // Tiny atanh(x) is best served by the direct odd series.
                        return Self {
                            internal: Box::new(Approximation::PrescaledAtanh(self)),
                            cache: RefCell::new(Cache::Invalid),
                            bound: RefCell::new(BoundCache::Invalid),
                            exact_sign: RefCell::new(ExactSignCache::Invalid),
                            signal: None,
                        };
                    }
                    if !rational.is_one() {
                        // For exact rationals, atanh(x) is one exact ln ratio.
                        // That keeps common factors in the logarithm constructor
                        // instead of building a generic quotient Computable first.
                        let one = Rational::one();
                        let ratio = (one.clone() + rational.clone()) / (one - rational);
                        return Self::ln_exact_rational(ratio)
                            .multiply(Self::rational(Rational::fraction(1, 2).unwrap()));
                    }
                }
            }
        }
        if self.exact_sign() == Some(Sign::Minus) {
            return self.negate().atanh().negate();
        }

        // General formula 1/2 * ln((1+x)/(1-x)). Tiny exact rationals avoid
        // this path because the odd atanh series has much less setup.
        let one = Self::one();
        let numerator = one.clone().add(self.clone());
        let denominator = one.add(self.negate());
        numerator
            .multiply(denominator.inverse())
            .ln()
            .multiply(Self::rational(Rational::fraction(1, 2).unwrap()))
    }

    /// Negate this number.
    pub fn negate(self) -> Computable {
        if let Some(rational) = self.exact_rational() {
            // Keep exact leaves exact; a Negate node would hide cheap rational
            // sign/MSD facts.
            return Self::rational(rational.neg());
        }
        if let Approximation::Negate(child) = self.internal.as_ref() {
            // Double negation cancels at construction time so exact sign walks
            // and approximation stacks stay shallow.
            return child.clone();
        }
        Self {
            internal: Box::new(Approximation::Negate(self)),
            cache: RefCell::new(Cache::Invalid),
            bound: RefCell::new(BoundCache::Invalid),
            exact_sign: RefCell::new(ExactSignCache::Invalid),
            signal: None,
        }
    }

    /// Multiplicative inverse of this number.
    pub fn inverse(self) -> Computable {
        if let Some(rational) = self.exact_rational()
            && let Ok(inverse) = rational.inverse()
        {
            // Exact rational reciprocals stay exact; this is common in BLAS
            // division by scalar constants.
            return Self::rational(inverse);
        }
        if let Approximation::Negate(child) = self.internal.as_ref()
            && child.exact_sign().is_some_and(|sign| sign != Sign::NoSign)
        {
            // 1/(-x) = -(1/x). The nonzero sign guard avoids manufacturing a
            // reciprocal of a value that may be zero.
            return child.clone().inverse().negate();
        }
        if let Approximation::Offset(child, n) = self.internal.as_ref()
            && child.exact_sign().is_some_and(|sign| sign != Sign::NoSign)
        {
            // 1/(x*2^n) = (1/x)*2^-n, preserving the cheap binary scale.
            return child.clone().inverse().shift_left(-n);
        }
        if let Approximation::Inverse(child) = self.internal.as_ref()
            && child.exact_sign().is_some_and(|sign| sign != Sign::NoSign)
        {
            // Inverse of inverse collapses only when the inner value is
            // structurally nonzero.
            return child.clone();
        }
        Self {
            internal: Box::new(Approximation::Inverse(self)),
            cache: RefCell::new(Cache::Invalid),
            bound: RefCell::new(BoundCache::Invalid),
            exact_sign: RefCell::new(ExactSignCache::Invalid),
            signal: None,
        }
    }

    fn shift_left(self, n: i32) -> Self {
        if n == 0 {
            return self;
        }
        if let Approximation::Offset(child, inner) = self.internal.as_ref() {
            // Combine nested binary offsets rather than growing a chain of
            // no-op-ish wrappers.
            return child.clone().shift_left(inner + n);
        }
        Self {
            internal: Box::new(Approximation::Offset(self, n)),
            cache: RefCell::new(Cache::Invalid),
            bound: RefCell::new(BoundCache::Invalid),
            exact_sign: RefCell::new(ExactSignCache::Invalid),
            signal: None,
        }
    }

    fn shift_right(self, n: i32) -> Self {
        self.shift_left(-n)
    }

    /// Square of this number.
    pub fn square(self) -> Self {
        if let Some(rational) = self.exact_rational() {
            // Exact rationals can square without approximation or expression growth.
            return Self::rational(rational.clone() * rational);
        }
        if let Approximation::Negate(child) = self.internal.as_ref() {
            // (-x)^2 is x^2; dropping the negate avoids an extra node in repeated products.
            return child.clone().square();
        }
        if let Approximation::Sqrt(child) = self.internal.as_ref() {
            match child.exact_sign() {
                // sqrt(x)^2 can collapse only when x is structurally known nonnegative.
                Some(Sign::Plus) | Some(Sign::NoSign) => return child.clone(),
                _ => {}
            }
        }
        if let Approximation::Offset(child, n) = self.internal.as_ref() {
            // (x * 2^n)^2 is x^2 * 2^(2n); keeping powers of two as offsets is much
            // cheaper than multiplying by an exact rational scale.
            return child.clone().square().shift_left(n * 2);
        }
        if let Approximation::Multiply(left, right) = &*self.internal {
            if let Some(scale) = left.exact_rational() {
                // Peel exact scales out of products before squaring symbolic factors.
                return right
                    .clone()
                    .square()
                    .multiply(Self::rational(scale.clone() * scale));
            }
            if let Some(scale) = right.exact_rational() {
                return left
                    .clone()
                    .square()
                    .multiply(Self::rational(scale.clone() * scale));
            }
        }
        Self {
            internal: Box::new(Approximation::Square(self)),
            cache: RefCell::new(Cache::Invalid),
            bound: RefCell::new(BoundCache::Invalid),
            exact_sign: RefCell::new(ExactSignCache::Invalid),
            signal: None,
        }
    }

    /// Multiply this number by some other number.
    pub fn multiply(self, other: Computable) -> Computable {
        let left_exact = self.exact_rational();
        let right_exact = other.exact_rational();

        if matches!(left_exact.as_ref(), Some(r) if r.sign() == Sign::NoSign)
            || matches!(right_exact.as_ref(), Some(r) if r.sign() == Sign::NoSign)
        {
            // Zero annihilates without preserving the other expression tree.
            return Self::zero();
        }
        if matches!(left_exact.as_ref(), Some(r) if r.is_one()) {
            // Multiplication by +/-1 stays as identity/negate so downstream exact-sign
            // queries still see the original structure.
            return other;
        }
        if matches!(right_exact.as_ref(), Some(r) if r.is_one()) {
            return self;
        }
        if matches!(left_exact.as_ref(), Some(r) if r.is_minus_one()) {
            return other.negate();
        }
        if matches!(right_exact.as_ref(), Some(r) if r.is_minus_one()) {
            return self.negate();
        }
        if let Some((shift, sign)) = left_exact.as_ref().and_then(Rational::power_of_two_shift) {
            // Dyadic scales are represented as binary offsets, avoiding generic multiply
            // evaluation during approximation.
            let shifted = other.shift_left(shift);
            return if sign == Sign::Minus {
                shifted.negate()
            } else {
                shifted
            };
        }
        if let Some((shift, sign)) = right_exact.as_ref().and_then(Rational::power_of_two_shift) {
            let shifted = self.shift_left(shift);
            return if sign == Sign::Minus {
                shifted.negate()
            } else {
                shifted
            };
        }
        if let (Some(left), Some(right)) = (left_exact.as_ref(), right_exact.as_ref()) {
            // Collapse purely exact products immediately.
            return Self::rational(left.clone() * right.clone());
        }
        if let Some(scale) = left_exact.as_ref()
            && let Approximation::Multiply(inner_left, inner_right) = &*other.internal
        {
            if let Some(inner_scale) = inner_left.exact_rational() {
                // Combine adjacent exact scales so factored symbolic products stay shallow.
                return inner_right
                    .clone()
                    .multiply(Self::rational(scale.clone() * inner_scale));
            }
            if let Some(inner_scale) = inner_right.exact_rational() {
                return inner_left
                    .clone()
                    .multiply(Self::rational(scale.clone() * inner_scale));
            }
        }
        if let Some(scale) = right_exact.as_ref()
            && let Approximation::Multiply(inner_left, inner_right) = &*self.internal
        {
            if let Some(inner_scale) = inner_left.exact_rational() {
                return inner_right
                    .clone()
                    .multiply(Self::rational(scale.clone() * inner_scale));
            }
            if let Some(inner_scale) = inner_right.exact_rational() {
                return inner_left
                    .clone()
                    .multiply(Self::rational(scale.clone() * inner_scale));
            }
        }
        Self {
            internal: Box::new(Approximation::Multiply(self, other)),
            cache: RefCell::new(Cache::Invalid),
            bound: RefCell::new(BoundCache::Invalid),
            exact_sign: RefCell::new(ExactSignCache::Invalid),
            signal: None,
        }
    }

    pub(crate) fn multiply_rational(self, scale: Rational) -> Computable {
        if scale.sign() == Sign::NoSign {
            // Multiplying by zero drops the expression tree, including any
            // pending expensive approximation work.
            return Self::zero();
        }
        if scale.is_one() {
            return self;
        }
        if scale.is_minus_one() {
            return self.negate();
        }
        if let Some((shift, sign)) = scale.power_of_two_shift() {
            // The borrowed Real fold path calls this often; recognize dyadic
            // scales before building a generic Multiply node.
            let shifted = self.shift_left(shift);
            return if sign == Sign::Minus {
                shifted.negate()
            } else {
                shifted
            };
        }
        Self {
            internal: Box::new(Approximation::Multiply(Self::rational(scale), self)),
            cache: RefCell::new(Cache::Invalid),
            bound: RefCell::new(BoundCache::Invalid),
            exact_sign: RefCell::new(ExactSignCache::Invalid),
            signal: None,
        }
    }

    /// Add some other number to this number.
    #[allow(clippy::should_implement_trait)]
    pub fn add(self, other: Computable) -> Computable {
        let left_exact = self.exact_rational();
        let right_exact = other.exact_rational();

        if matches!(left_exact.as_ref(), Some(r) if r.sign() == Sign::NoSign) {
            // Exact zero leaves are common after symbolic cancellation; avoid
            // wrapping the surviving operand in an Add node.
            return other;
        }
        if matches!(right_exact.as_ref(), Some(r) if r.sign() == Sign::NoSign) {
            // Symmetric exact-zero fast path for borrowed and owned additions.
            return self;
        }
        if let (Some(left), Some(right)) = (left_exact.as_ref(), right_exact.as_ref()) {
            // Fold exact leaf sums immediately so rational imports and parsed
            // dyadics stay outside the approximation graph.
            return Self::rational(left.clone() + right.clone());
        }
        let certified_bound = if let Some(rational) = right_exact.as_ref()
            && let Some(term) = self.shared_constant_term()
        {
            Self::constant_rational_sum_bound(&term, rational)
        } else if let Some(rational) = left_exact.as_ref()
            && let Some(term) = other.shared_constant_term()
        {
            Self::constant_rational_sum_bound(&term, rational)
        } else {
            BoundInfo::Unknown
        };
        // Store any c*K+q certificate directly on the Add node. The arithmetic
        // still falls back to a generic sum, but structural sign/fact queries
        // can answer from the certificate.
        let certified_sign = certified_bound.known_sign();
        Self {
            internal: Box::new(Approximation::Add(self, other)),
            cache: RefCell::new(Cache::Invalid),
            bound: RefCell::new(if certified_bound == BoundInfo::Unknown {
                BoundCache::Invalid
            } else {
                BoundCache::Valid(certified_bound)
            }),
            exact_sign: RefCell::new(match certified_sign {
                Some(sign) => ExactSignCache::Valid(sign),
                None => ExactSignCache::Invalid,
            }),
            signal: None,
        }
    }

    pub(crate) fn integer(n: BigInt) -> Self {
        if n == *signed::ONE {
            return Self::one();
        }
        Self {
            internal: Box::new(Approximation::Int(n)),
            cache: RefCell::new(Cache::Invalid),
            bound: RefCell::new(BoundCache::Invalid),
            exact_sign: RefCell::new(ExactSignCache::Invalid),
            signal: None,
        }
    }

    /// Attach an abort signal checked by long-running approximation routines.
    pub fn abort(&mut self, s: Signal) {
        self.signal = Some(s);
    }

    /// An approximation of this Computable scaled to a specific precision
    ///
    /// The approximation is scaled (thus, a larger value for more negative p)
    /// and should be accurate to within +/- 1 at the scale provided.
    ///
    /// Example: 0.875 is between 0 and 1 with zero bits of extra precision
    /// ```
    /// use hyperreal::{Rational,Computable};
    /// use num::{Zero,One};
    /// use num::bigint::{BigInt,ToBigInt};
    /// let n = Rational::fraction(7, 8).unwrap();
    /// let comp = Computable::rational(n);
    /// assert!((BigInt::zero() ..= BigInt::one()).contains(&comp.approx(0)));
    /// ```
    ///
    /// Example: π * 2³ is a bit more than 25 but less than 26
    /// ```
    /// use hyperreal::{Rational,Computable};
    /// use num::{Zero,One};
    /// use num::bigint::{BigInt,ToBigInt};
    /// let pi = Computable::pi();
    /// let between_25_26 = (ToBigInt::to_bigint(&25).unwrap() ..= ToBigInt::to_bigint(&26).unwrap());
    /// assert!(between_25_26.contains(&pi.approx(-3)));
    /// ```
    pub fn approx(&self, p: Precision) -> BigInt {
        self.approx_signal(&self.signal, p)
    }

    /// Like `approx` but specifying an atomic abort/ stop signal.
    pub fn approx_signal(&self, signal: &Option<Signal>, p: Precision) -> BigInt {
        enum Frame<'a> {
            Eval(&'a Computable, Precision),
            FinishNegate(&'a Computable, Precision),
            FinishAdd(&'a Computable, Precision),
            FinishOffset(&'a Computable, Precision),
        }

        if let Some(cached) = self.cached_at_precision(p) {
            return cached;
        }

        if !matches!(
            &*self.internal,
            Approximation::Negate(_) | Approximation::Add(_, _) | Approximation::Offset(_, _)
        ) {
            // Most node kinds evaluate as one kernel call. Only Negate/Add/Offset
            // are flattened below because they form the long chains seen in
            // parser, matrix, and structural-reduction workloads.
            let result = self.internal.approximate(signal, p);
            self.store_cache_value(p, result.clone());
            return result;
        }

        let mut frames = vec![Frame::Eval(self, p)];
        let mut values: Vec<BigInt> = Vec::new();

        while let Some(frame) = frames.pop() {
            match frame {
                Frame::Eval(node, prec) => {
                    if let Some(cached) = node.cached_at_precision(prec) {
                        values.push(cached);
                        continue;
                    }

                    match &*node.internal {
                        Approximation::Negate(child) => {
                            // Flatten sign wrappers so a deep chain of negated
                            // sums does not recurse through approx_signal.
                            frames.push(Frame::FinishNegate(node, prec));
                            frames.push(Frame::Eval(child, prec));
                        }
                        Approximation::Add(left, right) => {
                            // Evaluate add children at two guard bits, then
                            // round once. This mirrors the recursive add kernel
                            // but avoids stack growth for chained additions.
                            frames.push(Frame::FinishAdd(node, prec));
                            frames.push(Frame::Eval(right, prec - 2));
                            frames.push(Frame::Eval(left, prec - 2));
                        }
                        Approximation::Offset(child, n) => {
                            // Binary offsets translate the requested precision
                            // instead of doing any arithmetic at finish time.
                            frames.push(Frame::FinishOffset(node, prec));
                            frames.push(Frame::Eval(child, prec - *n));
                        }
                        _ => {
                            let result = node.internal.approximate(signal, prec);
                            node.store_cache_value(prec, result.clone());
                            values.push(result);
                        }
                    }
                }
                Frame::FinishNegate(node, prec) => {
                    let result = -values.pop().expect("negate child result should exist");
                    node.store_cache_value(prec, result.clone());
                    values.push(result);
                }
                Frame::FinishAdd(node, prec) => {
                    let right = values.pop().expect("add rhs result should exist");
                    let left = values.pop().expect("add lhs result should exist");
                    let result = scale(left + right, -2);
                    node.store_cache_value(prec, result.clone());
                    values.push(result);
                }
                Frame::FinishOffset(node, prec) => {
                    let result = values.pop().expect("offset child result should exist");
                    node.store_cache_value(prec, result.clone());
                    values.push(result);
                }
            }
        }

        values.pop().expect("evaluation should produce a result")
    }

    /// Conservatively inspect cached and structural numeric facts.
    pub fn structural_facts(&self) -> RealStructuralFacts {
        let exact = self.exact_rational();
        let exact_bound = exact.as_ref().map(BoundInfo::from_rational);

        let mut sign = self.exact_sign().map(public_sign);
        if sign.is_none()
            && let Some((_, appr)) = self.cached()
            && appr.abs() > BigInt::one()
        {
            sign = Some(public_sign(appr.sign()));
        }

        let bound = self.cheap_bound();
        if sign.is_none() {
            sign = bound.known_sign().map(public_sign);
        }
        if sign.is_none() {
            sign = exact_bound
                .as_ref()
                .and_then(BoundInfo::known_sign)
                .map(public_sign);
        }

        let zero = match sign {
            Some(RealSign::Zero) => ZeroKnowledge::Zero,
            Some(RealSign::Negative | RealSign::Positive) => ZeroKnowledge::NonZero,
            None => {
                if matches!(&bound, BoundInfo::Zero)
                    || matches!(&exact_bound, Some(BoundInfo::Zero))
                {
                    ZeroKnowledge::Zero
                } else if matches!(&bound, BoundInfo::NonZero { .. })
                    || matches!(&exact_bound, Some(BoundInfo::NonZero { .. }))
                {
                    ZeroKnowledge::NonZero
                } else {
                    ZeroKnowledge::Unknown
                }
            }
        };

        let magnitude = bound
            .magnitude_bits()
            .or_else(|| exact_bound.as_ref().and_then(BoundInfo::magnitude_bits));

        RealStructuralFacts {
            sign,
            zero,
            exact_rational: exact.is_some(),
            magnitude,
        }
    }

    /// Conservatively report whether structural inspection proves this value is zero.
    #[inline]
    pub fn zero_status(&self) -> ZeroKnowledge {
        if let Some(sign) = self.exact_sign() {
            return if sign == Sign::NoSign {
                ZeroKnowledge::Zero
            } else {
                ZeroKnowledge::NonZero
            };
        }

        match self.cheap_bound() {
            BoundInfo::Zero => ZeroKnowledge::Zero,
            BoundInfo::NonZero { .. } => ZeroKnowledge::NonZero,
            BoundInfo::Unknown => ZeroKnowledge::Unknown,
        }
    }

    /// Try to prove the sign without refining past `min_precision`.
    pub fn sign_until(&self, min_precision: Precision) -> Option<RealSign> {
        if let Some(sign) = self.exact_sign() {
            return Some(public_sign(sign));
        }
        if let Some((_, appr)) = self.cached()
            && appr.abs() > BigInt::one()
        {
            let sign = appr.sign();
            self.exact_sign.replace(ExactSignCache::Valid(sign));
            return Some(public_sign(sign));
        }

        if let Some(sign) = self.cheap_bound().known_sign() {
            return Some(public_sign(sign));
        }

        let start = if min_precision > 0 { min_precision } else { 0 };
        let mut p = start;
        loop {
            let appr = self.approx(p);
            if appr.abs() > BigInt::one() {
                let sign = appr.sign();
                self.exact_sign.replace(ExactSignCache::Valid(sign));
                return Some(public_sign(sign));
            }

            if p <= min_precision {
                break;
            }
            let next = (p * 3) / 2 - 16;
            p = if next < min_precision {
                min_precision
            } else {
                next
            };
            if should_stop(&self.signal) {
                break;
            }
        }

        if self
            .exact_rational()
            .is_some_and(|r| r.sign() == Sign::NoSign)
        {
            Some(RealSign::Zero)
        } else {
            None
        }
    }

    /// Try to determine the exact sign, refining cached approximations as needed.
    pub fn sign(&self) -> Sign {
        if let Some(sign) = self.exact_sign() {
            return sign;
        }
        {
            let cache = self.cache.borrow();
            if let Cache::Valid((_prec, cache_appr)) = &*cache {
                let sign = cache_appr.sign();
                if sign != Sign::NoSign {
                    self.exact_sign.replace(ExactSignCache::Valid(sign));
                    return sign;
                }
            }
        }
        let mut sign = Sign::NoSign;
        let mut p = 0;
        while p > -2000 && sign == Sign::NoSign {
            let appr = self.approx(p);
            p -= 10;
            sign = appr.sign();
        }
        if sign != Sign::NoSign {
            self.exact_sign.replace(ExactSignCache::Valid(sign));
        }
        sign
    }

    fn cached(&self) -> Option<(Precision, BigInt)> {
        if let Some(constant) = self.shared_constant_kind() {
            SHARED_CONSTANT_CACHES.with(|caches| {
                let caches = caches.borrow();
                match &caches[constant.cache_index()] {
                    Cache::Valid((cache_prec, cache_appr)) => {
                        Some((*cache_prec, cache_appr.clone()))
                    }
                    Cache::Invalid => None,
                }
            })
        } else {
            let cache = self.cache.borrow();
            if let Cache::Valid((cache_prec, cache_appr)) = &*cache {
                Some((*cache_prec, cache_appr.clone()))
            } else {
                None
            }
        }
    }

    /// Do not call this function if `self` and `other` may be the same.
    pub fn compare_to(&self, other: &Self) -> Ordering {
        if let (Some(left), Some(right)) = (self.exact_rational(), other.exact_rational()) {
            // Exact rationals compare directly; escalating to approximate comparison here is
            // both slower and can burn cache precision unnecessarily.
            return left
                .partial_cmp(&right)
                .expect("exact rationals should be comparable");
        }
        if let (Some(left), Some(right)) = (self.exact_sign(), other.exact_sign()) {
            match (left, right) {
                (Sign::Minus, Sign::Plus | Sign::NoSign) | (Sign::NoSign, Sign::Plus) => {
                    return Ordering::Less;
                }
                (Sign::Plus, Sign::Minus | Sign::NoSign) | (Sign::NoSign, Sign::Minus) => {
                    return Ordering::Greater;
                }
                _ => {}
            }

            if matches!(left, Sign::Plus | Sign::Minus)
                && left == right
                && let (Some(Some(left_msd)), Some(Some(right_msd))) = (
                    self.cheap_bound().known_msd(),
                    other.cheap_bound().known_msd(),
                )
                && left_msd != right_msd
            {
                // Same-sign values with different most-significant digits have a known
                // order without evaluating either value to a requested precision.
                return match left {
                    Sign::Plus => left_msd.cmp(&right_msd),
                    Sign::Minus => right_msd.cmp(&left_msd),
                    Sign::NoSign => unreachable!(),
                };
            }
        }
        let mut tolerance = -20;
        while tolerance > Precision::MIN {
            let order = self.compare_absolute(other, tolerance);
            if order != Ordering::Equal {
                return order;
            }
            tolerance *= 2;
        }
        panic!("Apparently called Computable::compare_to on equal values");
    }

    /// Compare two values to a specified tolerance (more negative numbers are more precise).
    pub fn compare_absolute(&self, other: &Self, tolerance: Precision) -> Ordering {
        if let (Some(left), Some(right)) = (self.exact_rational(), other.exact_rational()) {
            // Absolute comparison of exact rationals is still exact and should not enter
            // the computable approximation loop.
            let left_abs = if left.sign() == Sign::Minus {
                left.neg()
            } else {
                left
            };
            let right_abs = if right.sign() == Sign::Minus {
                right.neg()
            } else {
                right
            };
            return left_abs
                .partial_cmp(&right_abs)
                .expect("exact rationals should be comparable");
        }
        match (self.exact_sign(), other.exact_sign()) {
            // Zero-vs-nonzero absolute comparisons can be decided from cached exact signs.
            (Some(Sign::NoSign), Some(Sign::NoSign)) => return Ordering::Equal,
            (Some(Sign::NoSign), Some(_)) => return Ordering::Less,
            (Some(_), Some(Sign::NoSign)) => return Ordering::Greater,
            _ => {}
        }
        if let (Some(Some(left_msd)), Some(Some(right_msd))) = (
            self.cheap_bound().known_msd(),
            other.cheap_bound().known_msd(),
        ) {
            if left_msd > tolerance && right_msd < tolerance {
                // Cheap MSD bounds can prove a tolerance-separated absolute ordering before
                // allocating fresh approximations.
                return Ordering::Greater;
            }
            if right_msd > tolerance && left_msd < tolerance {
                return Ordering::Less;
            }
        }
        let needed = tolerance - 1;
        let this = self.approx(needed);
        let alt = other.approx(needed);
        let max = alt.clone() + signed::ONE.deref();
        let min = alt.clone() - signed::ONE.deref();
        if this > max {
            Ordering::Greater
        } else if this < min {
            Ordering::Less
        } else {
            Ordering::Equal
        }
    }

    /// Most Significant Digit (Bit) ?
    /// May panic or give incorrect answers if not yet discovered.
    fn known_msd(&self) -> Precision {
        if let Some((prec, appr)) = self.cached() {
            let length = appr.magnitude().bits() as Precision;
            prec + length - 1
        } else {
            panic!("Expected valid cache state for known MSD but it's invalid")
        }
    }

    /// Most Significant Digit - or perhaps None if as yet undiscovered and less than p.
    fn msd(&self, p: Precision) -> Option<Precision> {
        if let Some(msd) = self.cheap_bound().known_msd() {
            return msd;
        }

        let cache = self.cached();
        let mut try_once = false;

        if cache.is_none() {
            try_once = true;
        } else if let Some((_prec, appr)) = cache {
            let one = signed::ONE.deref();
            let minus_one = signed::MINUS_ONE.deref();

            if appr > *minus_one && appr < *one {
                try_once = true;
            }
        }

        if try_once {
            let appr = self.approx(p - 1);
            if appr.magnitude() < &BigUint::one() {
                return None;
            }
        }

        Some(self.known_msd())
    }

    const STOP_PRECISION: Precision = Precision::MIN / 3;

    /// MSD iteratively: 0, -16, -40, -76 etc. or p if that's lower.
    /// You can choose p to avoid unnecessary work.
    pub(super) fn iter_msd_stop(&self, p: Precision) -> Option<Precision> {
        let mut prec = 0;

        loop {
            let msd = self.msd(prec);
            if msd.is_some() {
                return msd;
            }
            prec = (prec * 3) / 2 - 16;
            if prec <= p {
                break;
            }
            if should_stop(&self.signal) {
                break;
            }
        }
        self.msd(p)
    }

    /// MSD but iteratively without a guess as to precision.
    pub(super) fn iter_msd(&self) -> Precision {
        self.iter_msd_stop(Self::STOP_PRECISION)
            .unwrap_or(Self::STOP_PRECISION)
    }
}

fn shift(n: BigInt, p: Precision) -> BigInt {
    match 0.cmp(&p) {
        Ordering::Greater => n >> -p,
        Ordering::Equal => n,
        Ordering::Less => n << p,
    }
}

/// Scale n by p bits, rounding if this makes n smaller.
/// e.g. scale(10, 2) == 40
///      scale(10, -2) == 3
fn scale(n: BigInt, p: Precision) -> BigInt {
    if p >= 0 {
        n << p
    } else {
        let adj = shift(n, p + 1) + signed::ONE.deref();
        adj >> 1
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use num::Signed;
    use num::bigint::BigUint;

    #[test]
    fn compare() {
        let six: BigInt = "6".parse().unwrap();
        let five: BigInt = "5".parse().unwrap();
        let four: BigInt = "4".parse().unwrap();
        let six = Computable::integer(six.clone());
        let five = Computable::integer(five.clone());
        let four = Computable::integer(four.clone());

        assert_eq!(six.compare_to(&five), Ordering::Greater);
        assert_eq!(five.compare_to(&six), Ordering::Less);
        assert_eq!(four.compare_to(&six), Ordering::Less);
    }

    #[test]
    fn bigger() {
        let six: BigInt = "6".parse().unwrap();
        let five: BigInt = "5".parse().unwrap();
        let four: BigInt = "4".parse().unwrap();
        let a = Computable::integer(six.clone());
        let b = Computable::integer(five.clone());
        assert_eq!(a.compare_absolute(&b, 0), Ordering::Greater);
        let c = Computable::integer(four.clone());
        assert_eq!(c.compare_absolute(&a, 0), Ordering::Less);
        assert_eq!(b.compare_absolute(&b, 0), Ordering::Equal);
    }

    #[test]
    fn shifted() {
        let one = BigInt::one();
        let two = &one + &one;
        assert_eq!(one, shift(two, -1));
    }

    #[test]
    fn prec() {
        let nine: BigInt = "9".parse().unwrap();
        let five: BigInt = "5".parse().unwrap();
        let two: BigInt = "2".parse().unwrap();
        let one = BigInt::one();
        let a = Computable::integer(nine.clone());
        assert_eq!(nine, a.approx(0));
        assert_eq!(five, a.approx(1));
        assert_eq!(two, a.approx(2));
        assert_eq!(one, a.approx(3));
        assert_eq!(Cache::Valid((0, nine)), a.cache.into_inner());
    }

    #[test]
    fn prec_pi() {
        let three: BigInt = "3".parse().unwrap();
        let six: BigInt = "6".parse().unwrap();
        let thirteen: BigInt = "13".parse().unwrap();
        let four_zero_two: BigInt = "402".parse().unwrap();
        let a = Computable::pi();
        assert_eq!(four_zero_two, a.approx(-7));
        assert_eq!(three, a.approx(0));
        assert_eq!(six, a.approx(-1));
        assert_eq!(thirteen, a.approx(-2));
        assert_eq!(Some((-7, four_zero_two)), a.cached());
    }

    #[test]
    fn rational_zero_and_one_use_dedicated_nodes() {
        let zero = Computable::rational(Rational::zero());
        let one = Computable::rational(Rational::one());

        // These identities are pervasive in higher-level constructors. Keep
        // them on the dedicated nodes so structural facts are available without
        // forcing the generic Ratio approximation path.
        assert!(matches!(*zero.internal, Approximation::Int(ref value) if value.is_zero()));
        assert!(matches!(*one.internal, Approximation::One));
        assert_eq!(zero.zero_status(), ZeroKnowledge::Zero);
        assert_eq!(one.zero_status(), ZeroKnowledge::NonZero);
        assert_eq!(zero.exact_sign(), Some(Sign::NoSign));
        assert_eq!(one.exact_sign(), Some(Sign::Plus));
    }

    #[test]
    fn prec_atan_5() {
        let five: BigInt = "5".parse().unwrap();
        let atan_5 = Computable::prescaled_atan(five);
        let two_zero_two: BigInt = "202".parse().unwrap();
        assert_eq!(two_zero_two, atan_5.approx(-10));
        let at_twenty: BigInt = "206984".parse().unwrap();
        assert_eq!(at_twenty, atan_5.approx(-20));
    }

    #[test]
    fn prec_atan_239() {
        let two_three_nine: BigInt = "239".parse().unwrap();
        let atan_239 = Computable::prescaled_atan(two_three_nine);
        let four: BigInt = "4".parse().unwrap();
        assert_eq!(four, atan_239.approx(-10));
        let at_twenty: BigInt = "4387".parse().unwrap();
        assert_eq!(at_twenty, atan_239.approx(-20));
    }

    #[test]
    fn msd() {
        let one: BigInt = "1".parse().unwrap();
        let a = Computable::integer(one.clone());
        assert_eq!(Some(0), a.msd(-4));
        let three: BigInt = "3".parse().unwrap();
        let d = Computable::integer(three.clone());
        assert_eq!(Some(1), d.msd(-4));
        let five: BigInt = "5".parse().unwrap();
        let e = Computable::integer(five.clone());
        assert_eq!(Some(2), e.msd(-4));
        let seven: BigInt = "7".parse().unwrap();
        let f = Computable::integer(seven.clone());
        assert_eq!(Some(2), f.msd(-4));
        let eight: BigInt = "8".parse().unwrap();
        let g = Computable::integer(eight.clone());
        assert_eq!(Some(3), g.msd(-4));
    }

    #[test]
    fn iter_msd() {
        let one = Computable::one();
        assert_eq!(one.iter_msd(), 0);
        let pi = Computable::pi();
        assert_eq!(pi.iter_msd(), 1);
        let five = Rational::new(5);
        let e = Computable::exp_rational(five);
        assert_eq!(e.iter_msd(), 7);
    }

    #[test]
    fn e_constant_cache_is_shared() {
        let e = Computable::e_constant();
        assert!(e.cached().is_none());
        let _ = e.approx(-32);

        let cached = Computable::e_constant()
            .cached()
            .expect("e cache should be shared across instances");
        assert!(cached.0 <= -32);
    }

    #[test]
    fn exp_one_uses_dedicated_e_constant() {
        let e = Computable::rational(Rational::one()).exp();
        assert!(matches!(
            &*e.internal,
            Approximation::Constant(SharedConstant::E)
        ));
    }

    #[test]
    fn pi_cache_is_shared() {
        let pi = Computable::pi();
        assert!(pi.cached().is_none());
        let _ = pi.approx(-32);

        let cached = Computable::pi()
            .cached()
            .expect("pi cache should be shared across instances");
        assert!(cached.0 <= -32);
    }

    #[test]
    fn tau_cache_is_shared() {
        let tau = Computable::tau();
        assert!(tau.cached().is_none());
        let _ = tau.approx(-32);

        let cached = Computable::tau()
            .cached()
            .expect("tau cache should be shared across instances");
        assert!(cached.0 <= -32);
    }

    #[test]
    fn tau_cache_reuses_warmed_pi_cache() {
        std::thread::spawn(|| {
            let pi = Computable::pi();
            let _ = pi.approx(-64);
            assert!(Computable::tau().cached().is_none());

            let tau_appr = Computable::tau().approx(-32);
            let pi_scaled_as_tau = Computable::pi().approx(-33);
            assert_eq!(tau_appr, pi_scaled_as_tau);

            let cached = Computable::tau()
                .cached()
                .expect("tau cache should be filled from pi cache");
            assert_eq!(cached.0, -32);
            assert_eq!(cached.1, tau_appr);
        })
        .join()
        .expect("tau cache test thread should finish");
    }

    #[test]
    fn ln_constant_cache_is_shared() {
        let ln2 = Computable::ln_constant(2).unwrap();
        assert!(ln2.cached().is_none());
        let _ = ln2.approx(-32);

        let cached = Computable::ln_constant(2)
            .unwrap()
            .cached()
            .expect("ln constant cache should be shared across instances");
        assert!(cached.0 <= -32);
    }

    #[test]
    fn negate() {
        let fifteen: BigInt = "15".parse().unwrap();
        let a = Computable::integer(fifteen.clone());
        let b = Computable::negate(a);
        let answer: BigInt = "-7".parse().unwrap();
        assert_eq!(answer, b.approx(1));
    }

    #[test]
    fn multiply() {
        let four: BigInt = "4".parse().unwrap();
        let five: BigInt = "5".parse().unwrap();
        let a = Computable::integer(four);
        let b = Computable::prescaled_atan(five);
        let m = Computable::multiply(a, b);
        let answer: BigInt = "809".parse().unwrap();
        assert_eq!(answer, m.approx(-10));
    }

    #[test]
    fn multiply_opposite() {
        let four: BigInt = "4".parse().unwrap();
        let five: BigInt = "5".parse().unwrap();
        let a = Computable::integer(four);
        let b = Computable::prescaled_atan(five);
        let m = Computable::multiply(b, a);
        let answer: BigInt = "809".parse().unwrap();
        assert_eq!(answer, m.approx(-10));
    }

    #[test]
    fn rational() {
        let sixth: Rational = "1/6".parse().unwrap();
        let c = Computable::rational(sixth);
        let zero = BigInt::zero();
        let one = BigInt::one();
        let ten: BigInt = "10".parse().unwrap();
        let eighty_five: BigInt = "85".parse().unwrap();
        assert_eq!(zero, c.approx(0));
        assert_eq!(zero, c.approx(-1));
        assert_eq!(zero, c.approx(-2));
        assert_eq!(one, c.approx(-3));
        assert_eq!(ten, c.approx(-6));
        assert_eq!(eighty_five, c.approx(-9));
    }

    #[test]
    fn scaled_ln1() {
        let zero = Computable::integer(BigInt::zero());
        let ln = Computable {
            internal: Box::new(Approximation::PrescaledLn(zero)),
            cache: RefCell::new(Cache::Invalid),
            bound: RefCell::new(BoundCache::Invalid),
            exact_sign: RefCell::new(ExactSignCache::Invalid),
            signal: None,
        };
        let zero = BigInt::zero();
        assert_eq!(zero, ln.approx(100));
    }

    #[test]
    fn scaled_ln1_4() {
        let zero_4: Rational = "0.4".parse().unwrap();
        let rational = Computable::rational(zero_4);
        let ln = Computable {
            internal: Box::new(Approximation::PrescaledLn(rational)),
            cache: RefCell::new(Cache::Invalid),
            bound: RefCell::new(BoundCache::Invalid),
            exact_sign: RefCell::new(ExactSignCache::Invalid),
            signal: None,
        };
        let five: BigInt = "5".parse().unwrap();
        assert_eq!(five, ln.approx(-4));
    }

    #[test]
    fn ln() {
        let five: BigInt = "5".parse().unwrap();
        let integer = Computable::integer(five);
        let ln = Computable::ln(integer);
        let correct: BigInt = "1769595698905".parse().unwrap();
        assert_eq!(ln.approx(-40), correct);
    }

    #[test]
    fn exp_and_ln_round_trip() {
        let seven_fifths = Computable::rational(Rational::fraction(7, 5).unwrap());
        assert_close(seven_fifths.clone().exp().ln(), seven_fifths, -40, 2);
    }

    #[test]
    fn exact_transcendental_identities() {
        let zero = Computable::rational(Rational::zero());
        let one = Computable::rational(Rational::one());
        assert_close(zero.clone().exp(), one.clone(), -40, 0);
        assert_close(one.ln(), zero.clone(), -40, 0);
        assert_close(zero.clone().sin(), zero.clone(), -40, 0);
        assert_close(zero.clone().cos(), Computable::one(), -40, 0);
        assert_close(zero.tan(), Computable::rational(Rational::zero()), -40, 0);
    }

    #[test]
    fn compare_to_uses_exact_sign_and_rational_shortcuts() {
        let minus_pi = Computable::pi().negate();
        let pi = Computable::pi();
        assert_eq!(minus_pi.compare_to(&pi), Ordering::Less);

        let left = Computable::rational(Rational::fraction(7, 8).unwrap());
        let right = Computable::rational(Rational::fraction(9, 10).unwrap());
        assert_eq!(left.compare_to(&right), Ordering::Less);
    }

    #[test]
    fn compare_to_uses_exact_msd_gap_shortcut() {
        let base = Computable::pi();
        base.approx(-16);
        let huge = base
            .clone()
            .multiply(Computable::rational(Rational::from_bigint(
                BigInt::from(1_u8) << 200,
            )));
        assert_eq!(huge.compare_to(&base), Ordering::Greater);
        assert_eq!(base.compare_to(&huge), Ordering::Less);

        let minus_base = base.negate();
        let minus_huge = huge.negate();
        assert_eq!(minus_huge.compare_to(&minus_base), Ordering::Less);
        assert_eq!(minus_base.compare_to(&minus_huge), Ordering::Greater);
    }

    #[test]
    fn compare_absolute_uses_exact_shortcuts() {
        let zero = Computable::rational(Rational::zero());
        let tiny = Computable::rational(Rational::fraction(1, 1024).unwrap());
        assert_eq!(zero.compare_absolute(&tiny, -40), Ordering::Less);

        let left = Computable::rational(Rational::fraction(-7, 8).unwrap());
        let right = Computable::rational(Rational::fraction(9, 10).unwrap());
        assert_eq!(left.compare_absolute(&right, -40), Ordering::Less);
    }

    #[test]
    fn compare_absolute_uses_exact_msd_gap_shortcut() {
        let base = Computable::pi();
        base.approx(-16);
        let huge = base
            .clone()
            .multiply(Computable::rational(Rational::from_bigint(
                BigInt::from(1_u8) << 200,
            )));
        assert_eq!(huge.compare_absolute(&base, -40), Ordering::Greater);
        assert_eq!(base.compare_absolute(&huge, -40), Ordering::Less);
    }

    #[test]
    fn warmed_zero_sum_product_stays_zero() {
        let zero = Computable::pi().add(Computable::pi().negate());
        zero.approx(-128);
        let product = zero.multiply(Computable::pi());
        assert_eq!(product.approx(-128), BigInt::zero());
    }

    #[test]
    fn exp_negative_is_inverse() {
        let eleven_tenths = Computable::rational(Rational::fraction(11, 10).unwrap());
        let product = eleven_tenths
            .clone()
            .exp()
            .multiply(eleven_tenths.negate().exp());
        assert_close(product, Computable::one(), -40, 2);
    }

    #[test]
    fn exp_near_prescaled_limit_round_trip() {
        let half = Computable::rational(Rational::fraction(1, 2).unwrap());
        assert_close(half.clone().exp().ln(), half, -40, 2);
    }

    #[test]
    fn exp_large_argument_reduces_by_ln2() {
        let exponent = BigInt::from(200);
        let offset = Computable::rational(Rational::fraction(7, 20).unwrap());
        let value = Computable::ln2()
            .multiply(Computable::integer(exponent.clone()))
            .add(offset.clone());
        let expected = offset.exp().shift_left(200);

        assert_close(value.exp(), expected, -80, 2);
    }

    #[test]
    fn cos_zero() {
        let zero = Computable::rational(Rational::zero());
        let cos = zero.cos();
        let correct: BigInt = "4294967296".parse().unwrap();
        assert_eq!(cos.approx(-32), correct);
    }

    #[test]
    fn cos_one() {
        let one = Computable::one();
        let cos = one.cos();
        let correct: BigInt = "2320580734".parse().unwrap();
        assert_eq!(cos.approx(-32), correct);
    }

    fn assert_approx(c: Computable, p: Precision, expected: &str, max_error: i32) {
        let actual = c.approx(p);
        let expected: BigInt = expected.parse().unwrap();
        let error = (&actual - &expected).abs();
        let max_error = BigInt::from(max_error);
        assert!(
            error <= max_error,
            "actual {actual}, expected {expected}, error {error}"
        );
    }

    fn assert_close(left: Computable, right: Computable, p: Precision, max_error: i32) {
        let error = (left.approx(p) - right.approx(p)).abs();
        let max_error = BigInt::from(max_error);
        assert!(error <= max_error);
    }

    fn pi_times(r: Rational) -> Computable {
        Computable::pi().multiply(Computable::rational(r))
    }

    fn shifted_cos_sin(c: Computable) -> Computable {
        pi_times(Rational::fraction(1, 2).unwrap())
            .add(c.negate())
            .cos()
    }

    #[test]
    fn sin_small_arguments() {
        let one_fifth = Computable::rational(Rational::fraction(1, 5).unwrap());
        assert_approx(one_fifth.sin(), -32, "853278278", 1);

        let zero = Computable::rational(Rational::zero());
        assert_eq!(BigInt::zero(), zero.sin().approx(-32));
    }

    #[test]
    fn sin_medium_arguments() {
        let three: BigInt = "3".parse().unwrap();
        let three = Computable::integer(three);
        assert_approx(three.sin(), -32, "606105819", 1);
    }

    #[test]
    fn sin_cos_direct_medium_exact_rationals_match_reduced_forms() {
        for rational in [
            Rational::fraction(6, 5).unwrap(),
            Rational::fraction(7, 5).unwrap(),
            Rational::fraction(47, 32).unwrap(),
            Rational::try_from(1.23456789_f64).unwrap(),
        ] {
            let value = Computable::rational(rational);
            let complement =
                pi_times(Rational::fraction(1, 2).unwrap()).add(value.clone().negate());

            assert_close(value.clone().sin(), complement.clone().cos(), -96, 2);
            assert_close(value.clone().cos(), complement.sin(), -96, 2);
            assert_close(
                value.clone().negate().sin(),
                value.clone().sin().negate(),
                -96,
                2,
            );
            assert_close(value.clone().negate().cos(), value.cos(), -96, 2);
        }
    }

    #[test]
    fn sin_large_arguments() {
        let one_two_three: BigInt = "123".parse().unwrap();
        let one_two_three = Computable::integer(one_two_three);
        assert_approx(one_two_three.sin(), -32, "-1975270452", 1);
    }

    #[test]
    fn sin_negative_arguments() {
        let negative_three_fifths = Computable::rational(Rational::fraction(-3, 5).unwrap());
        assert_approx(negative_three_fifths.sin(), -32, "-2425120957", 1);
    }

    #[test]
    fn sin_near_pi_multiples() {
        let epsilon = Computable::rational(Rational::fraction(1, 64).unwrap());
        let pi_plus_epsilon = Computable::pi().add(epsilon.clone());
        let two_pi_minus_epsilon = pi_times(Rational::new(2)).add(epsilon.clone().negate());

        assert_approx(pi_plus_epsilon.sin(), -32, "-67106133", 1);
        assert_approx(two_pi_minus_epsilon.sin(), -32, "-67106133", 1);
    }

    #[test]
    fn sin_near_half_pi() {
        let epsilon = Computable::rational(Rational::fraction(1, 64).unwrap());
        let half_pi = pi_times(Rational::fraction(1, 2).unwrap());
        let half_pi_plus_epsilon = half_pi.clone().add(epsilon.clone());
        let half_pi_minus_epsilon = half_pi.add(epsilon.negate());

        assert_approx(half_pi_plus_epsilon.sin(), -32, "4294443019", 1);
        assert_approx(half_pi_minus_epsilon.sin(), -32, "4294443019", 1);
    }

    #[test]
    fn sin_matches_shifted_cos_identity() {
        for r in ["-12", "-3/5", "0", "1/5", "3", "123"] {
            let r: Rational = r.parse().unwrap();
            let c = Computable::rational(r);
            assert_close(c.clone().sin(), shifted_cos_sin(c), -40, 1);
        }

        for r in ["-7/3", "-1/2", "1/2", "2", "41/6"] {
            let r: Rational = r.parse().unwrap();
            let c = pi_times(r);
            assert_close(c.clone().sin(), shifted_cos_sin(c), -40, 1);
        }
    }

    #[test]
    fn inverse_trig_computable_kernels_approximate_expected_values() {
        let value = Computable::rational(Rational::fraction(7, 10).unwrap());
        let negative_value = Computable::rational(Rational::fraction(-7, 10).unwrap());

        assert_approx(value.clone().asin(), -40, "852558563672", 2);
        assert_approx(negative_value.asin(), -40, "-852558563672", 2);
        assert_approx(value.acos(), -40, "874550262507", 2);
    }

    #[test]
    fn endpoint_inverse_trig_computable_kernels_approximate_expected_values() {
        let tiny = Computable::rational(Rational::fraction(1, 1_000_000_000_000).unwrap());
        let near_one = Computable::rational(Rational::fraction(999_999, 1_000_000).unwrap());

        assert_approx(tiny.clone().asin(), -80, "1208925819615", 2);
        assert_approx(tiny.clone().acos(), -40, "1727108826178", 2);
        assert_approx(tiny.atanh(), -80, "1208925819615", 2);
        assert_approx(near_one.clone().asin(), -40, "1725553881793", 2);
        assert_approx(near_one.clone().acos(), -40, "1554944386", 2);
        assert_approx(near_one.atanh(), -40, "7976218668587", 2);
    }

    #[test]
    fn inverse_hyperbolic_computable_kernels_approximate_expected_values() {
        let half = Computable::rational(Rational::fraction(1, 2).unwrap());
        let negative_half = Computable::rational(Rational::fraction(-1, 2).unwrap());
        let two = Computable::rational(Rational::new(2));

        assert_approx(half.clone().asinh(), -40, "529097997076", 2);
        assert_approx(negative_half.clone().asinh(), -40, "-529097997076", 2);
        assert_approx(two.acosh(), -40, "1448010520960", 2);
        assert_approx(
            Computable::rational(Rational::new(2)).sqrt().acosh(),
            -40,
            "969080507343",
            2,
        );
        assert_approx(half.atanh(), -40, "603968492904", 2);
        assert_approx(negative_half.atanh(), -40, "-603968492904", 2);
    }

    #[test]
    fn deep_add_chain_approximates_without_recursive_walk() {
        let mut value = Computable::one();
        for _ in 0..5000 {
            value = value.add(Computable::one());
        }

        assert_eq!(value.approx(0), BigInt::from(5001));
    }

    #[test]
    fn deep_multiply_chain_of_ones_stays_exact() {
        let mut value = Computable::one();
        for _ in 0..5000 {
            value = value.multiply(Computable::one());
        }

        assert_eq!(value.approx(0), BigInt::from(1));
    }

    #[test]
    fn deep_multiply_chain_by_one_preserves_irrational() {
        let mut value = Computable::pi();
        for _ in 0..5000 {
            value = value.multiply(Computable::one());
        }

        assert_close(value, Computable::pi(), -40, 2);
    }

    #[test]
    fn rational_msd_exact_for_small_fraction() {
        let third = Computable::rational(Rational::fraction(1, 3).unwrap());
        assert_eq!(third.msd(-4), Some(-2));
    }

    #[test]
    fn multiply_combines_exact_scales() {
        let scale = Computable::rational(Rational::fraction(7, 8).unwrap());
        let combined = Computable::pi()
            .multiply(scale.clone())
            .multiply(scale.clone())
            .multiply(scale);
        let expected =
            Computable::pi().multiply(Computable::rational(Rational::fraction(343, 512).unwrap()));
        assert_close(combined, expected, -60, 2);
    }

    #[test]
    fn square_of_scaled_irrational_reuses_exact_scale() {
        let scaled =
            Computable::pi().multiply(Computable::rational(Rational::fraction(7, 8).unwrap()));
        let expected = Computable::pi()
            .square()
            .multiply(Computable::rational(Rational::fraction(49, 64).unwrap()));
        assert_close(scaled.square(), expected, -60, 2);
    }

    #[test]
    fn inverse_of_exact_fraction_has_structural_bound() {
        let third = Computable::rational(Rational::fraction(1, 3).unwrap());
        let inverse = third.inverse();
        assert_eq!(inverse.sign(), Sign::Plus);
        assert_eq!(inverse.msd(-4), Some(1));
    }

    #[test]
    fn inverse_of_scaled_irrational_uses_structural_msd() {
        let scale = Rational::fraction(7, 8).unwrap();
        let base = Computable::pi();
        base.approx(-16);
        let value = base.multiply(Computable::rational(scale.clone()));
        assert_eq!(value.planning_msd(), Some(Some(0)));
        assert_eq!(value.msd(-4), Some(1));
        let inverse = value.inverse();
        let expected = Computable::pi()
            .inverse()
            .multiply(Computable::rational(scale.inverse().unwrap()));
        assert_close(inverse, expected, -60, 2);
    }

    #[test]
    fn square_of_negative_fraction_has_structural_bound() {
        let value = Computable::rational(Rational::fraction(-3, 8).unwrap()).square();
        assert_eq!(value.sign(), Sign::Plus);
        assert_eq!(value.msd(-4), Some(-3));
    }

    #[test]
    fn sqrt_of_scaled_square_tracks_structural_msd() {
        let value = Computable::pi()
            .multiply(Computable::rational(Rational::fraction(7, 8).unwrap()))
            .square()
            .sqrt();
        let expected =
            Computable::pi().multiply(Computable::rational(Rational::fraction(7, 8).unwrap()));
        assert_eq!(value.sign(), Sign::Plus);
        assert_eq!(value.msd(-4), Some(1));
        assert_close(value, expected, -60, 2);
    }

    #[test]
    fn sqrt_of_square_of_negative_value_returns_absolute_value() {
        let value = Computable::rational(Rational::fraction(-3, 8).unwrap())
            .square()
            .sqrt();
        assert_eq!(
            value.approx(-8),
            Computable::rational(Rational::fraction(3, 8).unwrap()).approx(-8)
        );
    }

    #[test]
    fn double_negate_collapses_at_construction() {
        let value = Computable::pi().negate().negate();
        assert_close(value, Computable::pi(), -60, 2);
    }

    #[test]
    fn inverse_of_inverse_of_nonzero_value_collapses_at_construction() {
        let base = Computable::pi();
        base.approx(-16);
        let value = base.clone().inverse().inverse();
        assert_close(value, base, -60, 2);
    }

    #[test]
    fn nested_offsets_collapse_at_construction() {
        let value = Computable::pi().shift_left(5).shift_right(3);
        let expected = Computable::pi().shift_left(2);
        assert_close(value, expected, -60, 2);
    }

    #[test]
    fn square_of_negative_value_collapses_to_square_of_positive_value() {
        let value = Computable::pi().negate().square();
        let expected = Computable::pi().square();
        assert_close(value, expected, -60, 2);
    }

    #[test]
    fn inverse_of_negative_nonzero_value_normalizes_sign() {
        let base = Computable::pi();
        base.approx(-16);
        let value = base.clone().negate().inverse();
        let expected = base.inverse().negate();
        assert_close(value, expected, -60, 2);
    }

    #[test]
    fn multiply_by_negative_one_collapses_to_negate() {
        let minus_one = Computable::rational(Rational::one().neg());
        let value = Computable::pi().multiply(minus_one);
        let expected = Computable::pi().negate();
        assert_close(value, expected, -60, 2);
    }

    #[test]
    fn multiply_by_power_of_two_fraction_collapses_to_shift() {
        let value =
            Computable::pi().multiply(Computable::rational(Rational::fraction(1, 8).unwrap()));
        let expected = Computable::pi().shift_right(3);
        assert_close(value, expected, -60, 2);
    }

    #[test]
    fn multiply_by_negative_power_of_two_fraction_collapses_to_shift_and_negate() {
        let value =
            Computable::pi().multiply(Computable::rational(Rational::fraction(-1, 8).unwrap()));
        let expected = Computable::pi().shift_right(3).negate();
        assert_close(value, expected, -60, 2);
    }

    #[test]
    fn square_of_power_of_two_scaled_value_collapses_to_shifted_square() {
        let value = Computable::pi()
            .multiply(Computable::rational(Rational::new(8)))
            .square();
        let expected = Computable::pi().square().shift_left(6);
        assert_close(value, expected, -60, 2);
    }

    #[test]
    fn sqrt_of_exactly_scaled_square_collapses_at_construction() {
        let value = Computable::pi()
            .multiply(Computable::rational(Rational::fraction(7, 8).unwrap()))
            .square()
            .sqrt();
        let expected =
            Computable::pi().multiply(Computable::rational(Rational::fraction(7, 8).unwrap()));
        assert_close(value, expected, -60, 2);
    }

    #[test]
    fn sqrt_of_exact_rational_square_is_exact() {
        let value = Computable::rational(Rational::fraction(49, 64).unwrap()).sqrt();
        let expected = Computable::rational(Rational::fraction(7, 8).unwrap());
        assert_close(value, expected, -60, 0);
    }

    #[test]
    fn square_of_sqrt_of_positive_value_collapses_at_construction() {
        let value = Computable::rational(Rational::new(2)).sqrt().square();
        let expected = Computable::rational(Rational::new(2));
        assert_close(value, expected, -60, 2);
    }

    #[test]
    fn inverse_of_shifted_nonzero_value_collapses_to_shifted_inverse() {
        let base = Computable::pi();
        base.approx(-16);
        let value = base.clone().shift_left(5).inverse();
        let expected = base.inverse().shift_right(5);
        assert_close(value, expected, -60, 2);
    }

    #[test]
    fn structural_facts_for_exact_rationals() {
        let zero = Computable::rational(Rational::zero()).structural_facts();
        assert_eq!(zero.sign, Some(RealSign::Zero));
        assert_eq!(zero.zero, ZeroKnowledge::Zero);
        assert!(zero.exact_rational);
        assert_eq!(zero.magnitude, None);

        let negative = Computable::rational(Rational::fraction(-7, 8).unwrap()).structural_facts();
        assert_eq!(negative.sign, Some(RealSign::Negative));
        assert_eq!(negative.zero, ZeroKnowledge::NonZero);
        assert!(negative.exact_rational);
        assert_eq!(
            negative.magnitude,
            Some(MagnitudeBits {
                msd: -1,
                exact_msd: true,
            })
        );
    }

    #[test]
    fn structural_facts_for_shared_constant() {
        let facts = Computable::pi().structural_facts();
        assert_eq!(facts.sign, Some(RealSign::Positive));
        assert_eq!(facts.zero, ZeroKnowledge::NonZero);
        assert!(!facts.exact_rational);
        assert_eq!(
            facts.magnitude,
            Some(MagnitudeBits {
                msd: 1,
                exact_msd: true,
            })
        );
    }

    #[test]
    fn structural_facts_for_constant_rational_offset_certificates() {
        let pi_minus_three = Computable::pi().add(Computable::rational(Rational::new(-3)));
        let facts = pi_minus_three.structural_facts();
        assert_eq!(facts.sign, Some(RealSign::Positive));
        assert_eq!(facts.zero, ZeroKnowledge::NonZero);
        assert_eq!(
            facts.magnitude,
            Some(MagnitudeBits {
                msd: -3,
                exact_msd: true,
            })
        );
        assert_eq!(pi_minus_three.sign_until(0), Some(RealSign::Positive));

        let three_minus_pi = Computable::rational(Rational::new(3)).add(Computable::pi().negate());
        let facts = three_minus_pi.structural_facts();
        assert_eq!(facts.sign, Some(RealSign::Negative));
        assert_eq!(facts.zero, ZeroKnowledge::NonZero);
        assert_eq!(
            facts.magnitude,
            Some(MagnitudeBits {
                msd: -3,
                exact_msd: true,
            })
        );

        let two_pi_minus_six = Computable::pi()
            .shift_left(1)
            .add(Computable::rational(Rational::new(-6)));
        let facts = two_pi_minus_six.structural_facts();
        assert_eq!(facts.sign, Some(RealSign::Positive));
        assert_eq!(facts.zero, ZeroKnowledge::NonZero);
        assert_eq!(
            facts.magnitude,
            Some(MagnitudeBits {
                msd: -2,
                exact_msd: true,
            })
        );
    }

    #[test]
    fn zero_status_uses_structural_facts_without_refinement() {
        assert_eq!(
            Computable::rational(Rational::zero()).zero_status(),
            ZeroKnowledge::Zero
        );
        assert_eq!(
            Computable::rational(Rational::fraction(-7, 8).unwrap()).zero_status(),
            ZeroKnowledge::NonZero
        );
        assert_eq!(Computable::pi().zero_status(), ZeroKnowledge::NonZero);

        let near_pi =
            Computable::pi().add(Computable::rational(Rational::fraction(-22, 7).unwrap()));
        assert_eq!(near_pi.zero_status(), ZeroKnowledge::NonZero);
    }

    #[test]
    fn sign_until_respects_precision_floor() {
        let near_pi = Computable::pi().add(Computable::rational(Rational::new(-3)));

        assert_eq!(near_pi.sign_until(0), Some(RealSign::Positive));
        assert_eq!(near_pi.sign_until(-8), Some(RealSign::Positive));
    }

    #[test]
    fn sign_until_uses_structural_bounds_without_refinement() {
        let value = Computable::pi()
            .multiply(Computable::rational(Rational::fraction(-7, 8).unwrap()))
            .inverse()
            .negate();

        assert_eq!(value.sign_until(0), Some(RealSign::Positive));
    }

    #[test]
    fn add_with_dominant_term_has_structural_bound() {
        let value = Computable::integer(BigInt::from(8))
            .add(Computable::rational(Rational::fraction(-1, 8).unwrap()));
        assert_eq!(value.sign(), Sign::Plus);
        assert_eq!(value.msd(-4), Some(2));
    }

    #[test]
    fn add_ignores_tiny_term_at_target_precision() {
        let big = Computable::pi();
        let tiny = Computable::rational(
            Rational::from_bigint_fraction(BigInt::from(1), BigUint::from(1_u8) << 200).unwrap(),
        );
        assert_eq!(
            big.clone().add(tiny).compare_absolute(&big, -128),
            Ordering::Equal
        );
    }

    #[test]
    fn add_does_not_ignore_tiny_opposite_sign_term() {
        let big = Computable::pi();
        let tiny = Computable::rational(
            Rational::from_bigint_fraction(BigInt::from(-1), BigUint::from(1_u8) << 200).unwrap(),
        );
        let sum = big.clone().add(tiny.clone());
        let delta = sum.add(big.negate());
        assert_eq!(delta.compare_absolute(&tiny, -180), Ordering::Equal);
    }

    #[test]
    fn deep_structural_bound_chain() {
        let scale = Computable::rational(Rational::fraction(-7, 8).unwrap());
        let mut value = Computable::pi();
        value.approx(-16);
        for _ in 0..2000 {
            value = value.multiply(scale.clone()).inverse().negate();
        }
        assert_eq!(value.sign(), Sign::Plus);
    }

    #[test]
    fn huge_trig_arguments_reduce_correctly() {
        let huge_multiple = BigInt::from(1_u8) << 200;
        let offset = Computable::rational(Rational::fraction(7, 5).unwrap());
        let huge = Computable::pi()
            .multiply(Computable::integer(huge_multiple))
            .add(offset.clone());

        assert_eq!(
            huge.clone()
                .sin()
                .compare_absolute(&offset.clone().sin(), -80),
            Ordering::Equal
        );
        assert_eq!(
            huge.clone()
                .cos()
                .compare_absolute(&offset.clone().cos(), -80),
            Ordering::Equal
        );
        assert_eq!(
            huge.tan().compare_absolute(&offset.tan(), -72),
            Ordering::Equal
        );
    }

    #[test]
    fn tan_small_and_medium_arguments() {
        let one_fifth = Computable::rational(Rational::fraction(1, 5).unwrap());
        assert_approx(one_fifth.tan(), -32, "870632973", 2);

        let seven_fifths = Computable::rational(Rational::fraction(7, 5).unwrap());
        assert_approx(seven_fifths.tan(), -32, "24901720944", 2);
    }

    #[test]
    fn tan_near_half_pi() {
        let epsilon = Computable::rational(Rational::fraction(1, 64).unwrap());
        let near_half_pi = pi_times(Rational::fraction(1, 2).unwrap()).add(epsilon.negate());
        assert_approx(near_half_pi.tan(), -32, "274855536959", 8);
    }

    #[test]
    fn ln_sqrt_pi() {
        let pi = Computable::pi();
        let sqrt = Computable::sqrt(pi);
        let ln = Computable::ln(sqrt);
        let correct: BigInt = "629321910077".parse().unwrap();
        assert_eq!(ln.approx(-40), correct);
    }

    #[test]
    fn ln_large_power_of_two() {
        let value = Computable::rational(Rational::new(1024));
        let ten = Computable::rational(Rational::new(10));
        assert_close(value.ln(), ten.multiply(Computable::ln2()), -40, 2);
    }

    #[test]
    fn ln_tiny_power_of_two() {
        let denominator = BigUint::from(1_u8) << 10;
        let value = Computable::rational(
            Rational::from_bigint_fraction(BigInt::from(1), denominator).unwrap(),
        );
        let ten = Computable::rational(Rational::new(10));
        assert_close(value.ln(), ten.multiply(Computable::ln2()).negate(), -40, 2);
    }

    #[test]
    fn ln_exact_binary_scaled_rational() {
        let denominator = BigUint::from(1_u8) << 10;
        let value = Computable::rational(
            Rational::from_bigint_fraction(BigInt::from(3), denominator).unwrap(),
        );
        let expected = Computable::rational(Rational::new(3))
            .ln()
            .add(Computable::rational(Rational::new(-10)).multiply(Computable::ln2()));
        assert_close(value.ln(), expected, -40, 2);
    }

    #[test]
    fn sqrt_square_round_trip() {
        let two = Computable::rational(Rational::new(2));
        let sqrt_two = two.clone().sqrt();
        assert_close(sqrt_two.square(), two, -40, 2);
    }

    #[test]
    fn ln_near_prescaled_limit_round_trip() {
        let value = Computable::rational(Rational::fraction(47, 32).unwrap());
        assert_close(value.clone().ln().exp(), value, -40, 2);
    }

    #[test]
    fn add() {
        let three: BigInt = "3".parse().unwrap();
        let five: BigInt = "5".parse().unwrap();
        let a = Computable::integer(three);
        let b = Computable::integer(five);
        let c = Computable::add(a, b);
        let answer: BigInt = "256".parse().unwrap();
        assert_eq!(answer, c.approx(-5));
    }

    #[test]
    fn scale_up() {
        let ten: BigInt = "10".parse().unwrap();
        let three: BigInt = "3".parse().unwrap();
        assert_eq!(ten, scale(ten.clone(), 0));
        let a = scale(ten.clone(), -2);
        assert_eq!(three, a);
        let forty: BigInt = "40".parse().unwrap();
        let b = scale(ten.clone(), 2);
        assert_eq!(forty, b);
    }
}
