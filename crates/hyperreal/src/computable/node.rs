//! Core computable expression graph.
//!
//! This file owns node layout, caches, structural facts, and constructor-time
//! rewrites. Those concerns are intentionally adjacent: the hot path is usually
//! "construct a symbolic expression, prove a cheap fact, and avoid requesting
//! approximation." Splitting individual node families into separate modules was
//! deferred until benchmarks show no cost from crossing those boundaries.
//!
//! The exact-real model follows Boehm, Cartwright, Riggle, and O'Donnell,
//! "Exact real arithmetic: a case study in higher order programming",
//! LFP 1986, https://doi.org/10.1145/319838.319860.

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

pub type Precision = i32;

#[derive(Clone, Debug, PartialEq, Default)]
pub(crate) enum Cache {
    #[default]
    Invalid,
    Valid((Precision, BigInt)),
}

#[derive(Clone, Debug, PartialEq, Default)]
pub(crate) enum BoundCache {
    #[default]
    Invalid,
    Valid(BoundInfo),
}

#[derive(Clone, Copy, Debug, PartialEq, Default)]
pub(crate) enum ExactSignCache {
    #[default]
    Invalid,
    Unknown,
    Valid(Sign),
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) enum BoundInfo {
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
            SharedConstant::InvPi => Some(-2),
            SharedConstant::Tau => Some(2),
            SharedConstant::Ln2 => Some(-1),
            SharedConstant::Asinh1 => Some(-1),
            SharedConstant::Ln3
            | SharedConstant::Ln5
            | SharedConstant::Ln6
            | SharedConstant::Ln7
            | SharedConstant::Sqrt2
            | SharedConstant::Sqrt3
            | SharedConstant::Acosh2 => Some(0),
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
            Self::InvPi => (
                Rational::fraction(113, 355).unwrap(),
                Rational::fraction(106, 333).unwrap(),
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
            Self::Acosh2 => (
                Rational::fraction(131, 100).unwrap(),
                Rational::fraction(132, 100).unwrap(),
            ),
            Self::Asinh1 => (
                Rational::fraction(88, 100).unwrap(),
                Rational::fraction(89, 100).unwrap(),
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

pub(crate) fn should_stop(signal: &Option<Signal>) -> bool {
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
    pub(super) internal: Box<Approximation>,
    #[serde(skip)]
    pub(crate) cache: RefCell<Cache>,
    #[serde(skip)]
    pub(crate) bound: RefCell<BoundCache>,
    #[serde(skip)]
    pub(crate) exact_sign: RefCell<ExactSignCache>,
    #[serde(skip)]
    pub(crate) signal: Option<Signal>,
}

pub(crate) mod signed {
    use num::{BigInt, One};
    use std::sync::LazyLock;

    // Use the narrow primitive that holds each literal so `BigInt::from`
    // dispatches directly instead of routing through the `ToBigInt` helper.
    pub(crate) static MINUS_ONE: LazyLock<BigInt> = LazyLock::new(|| BigInt::from(-1));
    pub(crate) static ONE: LazyLock<BigInt> = LazyLock::new(BigInt::one);
    pub(crate) static TWO: LazyLock<BigInt> = LazyLock::new(|| BigInt::from(2_u8));
    pub(crate) static FOUR: LazyLock<BigInt> = LazyLock::new(|| BigInt::from(4_u8));
    pub(crate) static SIX: LazyLock<BigInt> = LazyLock::new(|| BigInt::from(6_u8));
    pub(crate) static EIGHT: LazyLock<BigInt> = LazyLock::new(|| BigInt::from(8_u8));
    pub(crate) static SIXTEEN: LazyLock<BigInt> = LazyLock::new(|| BigInt::from(16_u8));
    pub(crate) static TWENTY_FOUR: LazyLock<BigInt> = LazyLock::new(|| BigInt::from(24_u8));
    pub(crate) static SIXTY_FOUR: LazyLock<BigInt> = LazyLock::new(|| BigInt::from(64_u8));
}

pub(crate) mod unsigned {
    use num::{BigUint, One};
    use std::sync::LazyLock;

    // These are small non-negative constants, so `u8` is the exact source type
    // and avoids the extra conversion trait path used before the bigint audit.
    pub(crate) static ONE: LazyLock<BigUint> = LazyLock::new(BigUint::one);
    pub(crate) static TWO: LazyLock<BigUint> = LazyLock::new(|| BigUint::from(2_u8));
    pub(crate) static TEN: LazyLock<BigUint> = LazyLock::new(|| BigUint::from(10_u8));
    pub(crate) static FIVE: LazyLock<BigUint> = LazyLock::new(|| BigUint::from(5_u8));
    pub(crate) static SIX: LazyLock<BigUint> = LazyLock::new(|| BigUint::from(6_u8));
}

static HALF_PI_SHORTCUT_RATIONAL_LIMIT: LazyLock<Rational> =
    LazyLock::new(|| Rational::fraction(3, 2).unwrap());
static INVERSE_ENDPOINT_RATIONAL_THRESHOLD: LazyLock<Rational> =
    LazyLock::new(|| Rational::fraction(7, 8).unwrap());
static HALF_RATIONAL: LazyLock<Rational> = LazyLock::new(|| Rational::fraction(1, 2).unwrap());

impl Computable {
    #[inline]
    fn half() -> Self {
        // atanh/log-ratio reductions multiply by 1/2 after exact symbolic
        // simplification. Keeping the half rational cached avoids rebuilding a
        // tiny exact leaf on every construction, and still delays approximation
        // to the final Computable graph. This follows Boehm et al.'s exact-real
        // separation of symbolic construction from numerical refinement:
        // https://doi.org/10.1145/319838.319860.
        Self::rational(HALF_RATIONAL.clone())
    }

    fn internal_structural_eq(left: &Self, right: &Self) -> bool {
        fn compare_nodes(left: &Approximation, right: &Approximation) -> bool {
            match (left, right) {
                (Approximation::One, Approximation::One) => true,
                (Approximation::Int(left), Approximation::Int(right)) => left == right,
                (Approximation::Constant(left), Approximation::Constant(right)) => left == right,
                (Approximation::Inverse(left), Approximation::Inverse(right)) => {
                    Computable::internal_structural_eq(left, right)
                }
                (Approximation::Negate(left), Approximation::Negate(right)) => {
                    Computable::internal_structural_eq(left, right)
                }
                (Approximation::Add(left, right), Approximation::Add(left_rhs, right_rhs)) => {
                    Computable::internal_structural_eq(left, left_rhs)
                        && Computable::internal_structural_eq(right, right_rhs)
                }
                (
                    Approximation::Multiply(left, right),
                    Approximation::Multiply(left_rhs, right_rhs),
                ) => {
                    Computable::internal_structural_eq(left, left_rhs)
                        && Computable::internal_structural_eq(right, right_rhs)
                }
                (Approximation::Square(left), Approximation::Square(right)) => {
                    Computable::internal_structural_eq(left, right)
                }
                (Approximation::Ratio(left), Approximation::Ratio(right)) => left == right,
                (
                    Approximation::Offset(left, left_shift),
                    Approximation::Offset(right, right_shift),
                ) => left_shift == right_shift && Computable::internal_structural_eq(left, right),
                (Approximation::PrescaledExp(left), Approximation::PrescaledExp(right)) => {
                    Computable::internal_structural_eq(left, right)
                }
                (Approximation::Sqrt(left), Approximation::Sqrt(right)) => {
                    Computable::internal_structural_eq(left, right)
                }
                (Approximation::PrescaledLn(left), Approximation::PrescaledLn(right)) => {
                    Computable::internal_structural_eq(left, right)
                }
                (Approximation::IntegralAtan(left), Approximation::IntegralAtan(right)) => {
                    left == right
                }
                (Approximation::PrescaledAtan(left), Approximation::PrescaledAtan(right)) => {
                    Computable::internal_structural_eq(left, right)
                }
                (Approximation::AtanRational(left), Approximation::AtanRational(right)) => {
                    left == right
                }
                (Approximation::AsinRational(left), Approximation::AsinRational(right)) => {
                    left == right
                }
                (Approximation::PrescaledAsin(left), Approximation::PrescaledAsin(right)) => {
                    Computable::internal_structural_eq(left, right)
                }
                (Approximation::AsinDeferred(left), Approximation::AsinDeferred(right)) => {
                    Computable::internal_structural_eq(left, right)
                }
                (Approximation::AcosPositive(left), Approximation::AcosPositive(right)) => {
                    Computable::internal_structural_eq(left, right)
                }
                (
                    Approximation::AcosPositiveRational(left),
                    Approximation::AcosPositiveRational(right),
                )
                | (
                    Approximation::AcosNegativeRational(left),
                    Approximation::AcosNegativeRational(right),
                ) => left == right,
                (Approximation::AcoshNearOne(left), Approximation::AcoshNearOne(right)) => {
                    Computable::internal_structural_eq(left, right)
                }
                (Approximation::AcoshDirect(left), Approximation::AcoshDirect(right)) => {
                    Computable::internal_structural_eq(left, right)
                }
                (Approximation::AsinhNearZero(left), Approximation::AsinhNearZero(right)) => {
                    Computable::internal_structural_eq(left, right)
                }
                (Approximation::AsinhDirect(left), Approximation::AsinhDirect(right)) => {
                    Computable::internal_structural_eq(left, right)
                }
                (Approximation::PrescaledAsinh(left), Approximation::PrescaledAsinh(right)) => {
                    Computable::internal_structural_eq(left, right)
                }
                (Approximation::AsinhRational(left), Approximation::AsinhRational(right)) => {
                    left == right
                }
                (Approximation::AtanhDirect(left), Approximation::AtanhDirect(right)) => {
                    Computable::internal_structural_eq(left, right)
                }
                (Approximation::PrescaledAtanh(left), Approximation::PrescaledAtanh(right)) => {
                    Computable::internal_structural_eq(left, right)
                }
                (Approximation::AtanhRational(left), Approximation::AtanhRational(right)) => {
                    left == right
                }
                (Approximation::PrescaledCos(left), Approximation::PrescaledCos(right)) => {
                    Computable::internal_structural_eq(left, right)
                }
                (
                    Approximation::PrescaledCosRational(left),
                    Approximation::PrescaledCosRational(right),
                ) => left == right,
                (Approximation::CosLargeRational(left), Approximation::CosLargeRational(right)) => {
                    left == right
                }
                (
                    Approximation::PrescaledCosHalfPiMinusRational(left),
                    Approximation::PrescaledCosHalfPiMinusRational(right),
                ) => left == right,
                (Approximation::PrescaledSin(left), Approximation::PrescaledSin(right)) => {
                    Computable::internal_structural_eq(left, right)
                }
                (
                    Approximation::PrescaledSinRational(left),
                    Approximation::PrescaledSinRational(right),
                ) => left == right,
                (Approximation::SinLargeRational(left), Approximation::SinLargeRational(right)) => {
                    left == right
                }
                (
                    Approximation::PrescaledSinHalfPiMinusRational(left),
                    Approximation::PrescaledSinHalfPiMinusRational(right),
                ) => left == right,
                (
                    Approximation::PrescaledCotHalfPiMinusRational(left),
                    Approximation::PrescaledCotHalfPiMinusRational(right),
                ) => left == right,
                (Approximation::TanLargeRational(left), Approximation::TanLargeRational(right)) => {
                    left == right
                }
                (Approximation::PrescaledTan(left), Approximation::PrescaledTan(right)) => {
                    Computable::internal_structural_eq(left, right)
                }
                (
                    Approximation::PrescaledTanRational(left),
                    Approximation::PrescaledTanRational(right),
                ) => left == right,
                (Approximation::PrescaledCot(left), Approximation::PrescaledCot(right)) => {
                    Computable::internal_structural_eq(left, right)
                }
                _ => false,
            }
        }

        compare_nodes(&left.internal, &right.internal)
    }

    fn compare_absolute_dominant_perturbation(
        base: &Self,
        perturbation: &Self,
        comparable: &Self,
        tolerance: Precision,
    ) -> Option<Ordering> {
        if !Computable::internal_structural_eq(base, comparable) {
            return None;
        }

        let (base_sign, base_msd) = base.planning_sign_and_msd();
        let (perturb_sign, perturb_msd) = perturbation.planning_sign_and_msd();
        let base_sign = base_sign?;
        let perturb_sign = perturb_sign?;
        let base_msd = base_msd.flatten();
        let perturb_msd = perturb_msd.flatten();

        match (base_sign, perturb_sign) {
            (Sign::NoSign, Sign::NoSign) => Some(Ordering::Equal),
            (Sign::NoSign, _) => {
                if perturb_msd.is_some_and(|msd| msd < tolerance) {
                    Some(Ordering::Equal)
                } else {
                    Some(Ordering::Greater)
                }
            }
            (base_sign, perturb_sign) if base_sign == perturb_sign => Some(
                if perturb_sign == Sign::NoSign || perturb_msd.is_some_and(|msd| msd < tolerance) {
                    Ordering::Equal
                } else {
                    Ordering::Greater
                },
            ),
            (base_sign, perturb_sign) if base_sign != perturb_sign => {
                if perturb_msd.is_some_and(|msd| msd < tolerance) {
                    Some(Ordering::Equal)
                } else if let (Some(base_msd), Some(perturb_msd)) = (base_msd, perturb_msd) {
                    if base_msd >= perturb_msd + 1 {
                        Some(Ordering::Less)
                    } else {
                        None
                    }
                } else {
                    None
                }
            }
            _ => None,
        }
    }

    /// Exactly zero.
    pub fn zero() -> Computable {
        crate::trace_dispatch!("computable", "constructor", "zero");
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
        crate::trace_dispatch!("computable", "constructor", "one");
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
        crate::trace_dispatch!("computable", "constructor", "cached-pi");
        Self::shared_constant(SharedConstant::Pi)
    }

    pub(crate) fn pi_inverse_constant() -> Computable {
        crate::trace_dispatch!("computable", "constructor", "cached-inv-pi");
        Self::shared_constant(SharedConstant::InvPi)
    }

    /// Approximate τ, the ratio of a circle's circumference to its radius.
    pub fn tau() -> Computable {
        crate::trace_dispatch!("computable", "constructor", "cached-tau");
        Self::shared_constant(SharedConstant::Tau)
    }

    /// Approximate e, Euler's number and the base of the natural logarithm.
    pub fn e() -> Computable {
        crate::trace_dispatch!("computable", "constructor", "cached-e");
        Self::e_constant()
    }

    pub(crate) fn e_constant() -> Computable {
        crate::trace_dispatch!("computable", "constructor", "cached-e-internal");
        Self::shared_constant(SharedConstant::E)
    }

    pub(crate) fn ln_constant(base: u32) -> Option<Computable> {
        // Common logarithms are shared constants so repeated symbolic ln forms
        // reuse one approximation cache across cloned Real values.
        crate::trace_dispatch!("computable", "constructor", "shared-log-constant-probe");
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
        crate::trace_dispatch!("computable", "constructor", "shared-sqrt-constant-probe");
        let constant = match n {
            2 => SharedConstant::Sqrt2,
            3 => SharedConstant::Sqrt3,
            _ => return None,
        };
        Some(Self::shared_constant(constant))
    }

    pub(crate) fn acosh2_constant() -> Computable {
        crate::trace_dispatch!("computable", "constructor", "cached-acosh2");
        Self::shared_constant(SharedConstant::Acosh2)
    }

    pub(crate) fn asinh1_constant() -> Computable {
        crate::trace_dispatch!("computable", "constructor", "cached-asinh1");
        Self::shared_constant(SharedConstant::Asinh1)
    }

    pub(crate) fn prescaled_sin(value: Computable) -> Computable {
        // Caller promises argument reduction has already happened. Keeping this
        // constructor private prevents large arguments from entering the Taylor
        // kernel directly.
        crate::trace_dispatch!("computable", "constructor", "prescaled-sin");
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
        crate::trace_dispatch!("computable", "constructor", "prescaled-cos");
        Self {
            internal: Box::new(Approximation::PrescaledCos(value)),
            cache: RefCell::new(Cache::Invalid),
            bound: RefCell::new(BoundCache::Invalid),
            exact_sign: RefCell::new(ExactSignCache::Invalid),
            signal: None,
        }
    }

    fn prescaled_cos_rational(rational: Rational) -> Computable {
        // Small exact-rational cosine construction is a scalar hot path. Store
        // the rational directly so construction avoids a child Ratio node; the
        // approximation dispatcher materializes the same kernel input later if
        // digits are requested.
        crate::trace_dispatch!("computable", "constructor", "prescaled-cos-rational");
        Self {
            internal: Box::new(Approximation::PrescaledCosRational(rational)),
            cache: RefCell::new(Cache::Invalid),
            bound: RefCell::new(BoundCache::Invalid),
            exact_sign: RefCell::new(ExactSignCache::Valid(Sign::Plus)),
            signal: None,
        }
    }

    pub(crate) fn cos_large_rational_deferred(rational: Rational) -> Computable {
        // Real::cos for large plain rationals defers the expensive half-pi
        // reduction until digits are requested. This keeps construction and
        // structural queries cheap; the approximation node then performs direct
        // residual arithmetic without allocating the generic reducer graph.
        crate::trace_dispatch!("computable", "constructor", "cos-large-rational-deferred");
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
        crate::trace_dispatch!(
            "computable",
            "constructor",
            "prescaled-cos-half-pi-minus-rational"
        );
        let internal = Approximation::PrescaledCosHalfPiMinusRational(rational);
        Self {
            internal: Box::new(internal),
            cache: RefCell::new(Cache::Invalid),
            bound: RefCell::new(BoundCache::Invalid),
            exact_sign: RefCell::new(ExactSignCache::Valid(Sign::Plus)),
            signal: None,
        }
    }

    fn prescaled_sin_half_pi_minus_rational(rational: Rational) -> Computable {
        // cos(x) for exact medium rational x is sin(pi/2 - x). This mirrors the
        // cosine shortcut above and keeps common dyadic imports off the generic
        // composite residual path.
        crate::trace_dispatch!(
            "computable",
            "constructor",
            "prescaled-sin-half-pi-minus-rational"
        );
        let internal = Approximation::PrescaledSinHalfPiMinusRational(rational);
        Self {
            internal: Box::new(internal),
            cache: RefCell::new(Cache::Invalid),
            bound: RefCell::new(BoundCache::Invalid),
            exact_sign: RefCell::new(ExactSignCache::Valid(Sign::Plus)),
            signal: None,
        }
    }

    fn prescaled_cot_half_pi_minus_rational(rational: Rational) -> Computable {
        // tan(x) near pi/2 is cot(pi/2 - x). Keeping the residual exact avoids
        // the generic complement tree and lets the approximation layer evaluate
        // the local quotient directly.
        crate::trace_dispatch!(
            "computable",
            "constructor",
            "prescaled-cot-half-pi-minus-rational"
        );
        let internal = Approximation::PrescaledCotHalfPiMinusRational(rational);
        Self {
            internal: Box::new(internal),
            cache: RefCell::new(Cache::Invalid),
            bound: RefCell::new(BoundCache::Invalid),
            exact_sign: RefCell::new(ExactSignCache::Valid(Sign::Plus)),
            signal: None,
        }
    }

    pub(crate) fn sin_large_rational_deferred(rational: Rational) -> Computable {
        // Same lazy-construction policy as cos_large_rational_deferred. The
        // approximation node evaluates the direct half-pi residual itself, so
        // exact 1e6/1e30 scalar rows avoid eager reducer graph construction.
        crate::trace_dispatch!("computable", "constructor", "sin-large-rational-deferred");
        Self {
            internal: Box::new(Approximation::SinLargeRational(rational)),
            cache: RefCell::new(Cache::Invalid),
            bound: RefCell::new(BoundCache::Invalid),
            exact_sign: RefCell::new(ExactSignCache::Invalid),
            signal: None,
        }
    }

    pub(crate) fn tan_large_rational_deferred(rational: Rational) -> Computable {
        // Tangent used to run through generic pi reduction even for exact large
        // rationals. Deferring it into a dedicated approximation node lets the
        // hot 1e6/1e30 rows share the direct half-pi residual used by sin/cos.
        crate::trace_dispatch!("computable", "constructor", "tan-large-rational-deferred");
        Self {
            internal: Box::new(Approximation::TanLargeRational(rational)),
            cache: RefCell::new(Cache::Invalid),
            bound: RefCell::new(BoundCache::Invalid),
            exact_sign: RefCell::new(ExactSignCache::Invalid),
            signal: None,
        }
    }

    pub(crate) fn prescaled_tan(value: Computable) -> Computable {
        // Same reduced-argument contract as prescaled_sin; tangent additionally
        // relies on the public constructor to handle near-pole complements.
        crate::trace_dispatch!("computable", "constructor", "prescaled-tan");
        Self {
            internal: Box::new(Approximation::PrescaledTan(value)),
            cache: RefCell::new(Cache::Invalid),
            bound: RefCell::new(BoundCache::Invalid),
            exact_sign: RefCell::new(ExactSignCache::Invalid),
            signal: None,
        }
    }

    fn prescaled_sin_rational(rational: Rational) -> Computable {
        // Small exact-rational sine construction mirrors cosine and preserves
        // the exact sign without allocating a child Computable.
        crate::trace_dispatch!("computable", "constructor", "prescaled-sin-rational");
        let sign = rational.sign();
        Self {
            internal: Box::new(Approximation::PrescaledSinRational(rational)),
            cache: RefCell::new(Cache::Invalid),
            bound: RefCell::new(BoundCache::Invalid),
            exact_sign: RefCell::new(ExactSignCache::Valid(sign)),
            signal: None,
        }
    }

    fn prescaled_tan_rational(rational: Rational) -> Computable {
        // Small exact-rational tangent uses the same construction shortcut as
        // sine; sign follows the rational argument on the reduced interval.
        crate::trace_dispatch!("computable", "constructor", "prescaled-tan-rational");
        let sign = rational.sign();
        Self {
            internal: Box::new(Approximation::PrescaledTanRational(rational)),
            cache: RefCell::new(Cache::Invalid),
            bound: RefCell::new(BoundCache::Invalid),
            exact_sign: RefCell::new(ExactSignCache::Valid(sign)),
            signal: None,
        }
    }

    fn prescaled_asinh(value: Computable) -> Computable {
        // Tiny exact-rational asinh inputs use a direct odd-power series. This
        // keeps public construction cheap for scalar endpoint benches and only
        // enters the kernel after |x| has been structurally certified tiny.
        crate::trace_dispatch!("computable", "constructor", "prescaled-asinh");
        Self {
            internal: Box::new(Approximation::PrescaledAsinh(value)),
            cache: RefCell::new(Cache::Invalid),
            bound: RefCell::new(BoundCache::Invalid),
            exact_sign: RefCell::new(ExactSignCache::Invalid),
            signal: None,
        }
    }

    fn asinh_rational_deferred(rational: Rational) -> Computable {
        // Same series as `prescaled_asinh`, but exact rationals can skip the
        // child Computable wrapper and feed the kernel directly.
        crate::trace_dispatch!("computable", "constructor", "asinh-rational-deferred");
        let sign = rational.sign();
        Self {
            internal: Box::new(Approximation::AsinhRational(rational)),
            cache: RefCell::new(Cache::Invalid),
            bound: RefCell::new(BoundCache::Invalid),
            exact_sign: RefCell::new(ExactSignCache::Valid(sign)),
            signal: None,
        }
    }

    fn atanh_rational_deferred(rational: Rational) -> Computable {
        // Tiny exact-rational atanh uses the odd series directly. Keeping the
        // Rational payload avoids rebuilding a Ratio node in cold approximation
        // benches while preserving the symbolic value until the final request.
        crate::trace_dispatch!("computable", "constructor", "atanh-rational-deferred");
        let sign = rational.sign();
        Self {
            internal: Box::new(Approximation::AtanhRational(rational)),
            cache: RefCell::new(Cache::Invalid),
            bound: RefCell::new(BoundCache::Invalid),
            exact_sign: RefCell::new(ExactSignCache::Valid(sign)),
            signal: None,
        }
    }

    fn acos_positive(value: Computable) -> Computable {
        // For x >= 0, acos(x) is reduced with 2*atan(sqrt((1-x)/(1+x))).
        // A single deferred node avoids allocating that whole formula during
        // public construction of endpoint-heavy inverse trig expressions.
        crate::trace_dispatch!("computable", "constructor", "acos-positive-deferred");
        Self {
            internal: Box::new(Approximation::AcosPositive(value)),
            cache: RefCell::new(Cache::Invalid),
            bound: RefCell::new(BoundCache::Invalid),
            exact_sign: RefCell::new(ExactSignCache::Valid(Sign::Plus)),
            signal: None,
        }
    }

    fn acos_positive_rational_deferred(rational: Rational) -> Computable {
        crate::trace_dispatch!(
            "computable",
            "constructor",
            "acos-positive-rational-deferred"
        );
        Self {
            internal: Box::new(Approximation::AcosPositiveRational(rational)),
            cache: RefCell::new(Cache::Invalid),
            bound: RefCell::new(BoundCache::Invalid),
            exact_sign: RefCell::new(ExactSignCache::Valid(Sign::Plus)),
            signal: None,
        }
    }

    fn acos_negative_rational_deferred(magnitude: Rational) -> Computable {
        crate::trace_dispatch!(
            "computable",
            "constructor",
            "acos-negative-rational-deferred"
        );
        Self {
            internal: Box::new(Approximation::AcosNegativeRational(magnitude)),
            cache: RefCell::new(Cache::Invalid),
            bound: RefCell::new(BoundCache::Invalid),
            exact_sign: RefCell::new(ExactSignCache::Valid(Sign::Plus)),
            signal: None,
        }
    }

    fn asin_deferred(value: Computable) -> Computable {
        // Generic asin uses a stable atan/sqrt half-angle transform. Deferring
        // that formula keeps symbolic-radical construction lightweight and
        // leaves the exact input graph intact until approximation is requested.
        crate::trace_dispatch!("computable", "constructor", "asin-deferred");
        let sign = value.exact_sign();
        Self {
            internal: Box::new(Approximation::AsinDeferred(value)),
            cache: RefCell::new(Cache::Invalid),
            bound: RefCell::new(BoundCache::Invalid),
            exact_sign: RefCell::new(sign.map_or(ExactSignCache::Invalid, ExactSignCache::Valid)),
            signal: None,
        }
    }

    pub(crate) fn atanh_direct_deferred(value: Computable) -> Computable {
        // Endpoint atanh uses a deferred ln-ratio node. This keeps construction
        // cheap for predicate/scalar benches while preserving the same
        // approximation identity when a numeric value is requested.
        crate::trace_dispatch!("computable", "constructor", "atanh-direct-deferred");
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
        crate::trace_dispatch!("computable", "constructor", "acosh-near-one-deferred");
        Self {
            internal: Box::new(Approximation::AcoshNearOne(value)),
            cache: RefCell::new(Cache::Invalid),
            bound: RefCell::new(BoundCache::Invalid),
            exact_sign: RefCell::new(ExactSignCache::Valid(Sign::Plus)),
            signal: None,
        }
    }

    pub(crate) fn acosh_direct_deferred(value: Computable) -> Computable {
        // Large acosh uses a deferred direct ln/sqrt identity so construction
        // paths do not eagerly allocate the sqrt/log graph.
        crate::trace_dispatch!("computable", "constructor", "acosh-direct-deferred");
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
        crate::trace_dispatch!("computable", "constructor", "asinh-near-zero-deferred");
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
        crate::trace_dispatch!("computable", "constructor", "asinh-direct-deferred");
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
        crate::trace_dispatch!("computable", "constructor", "shared-constant-wrapper");
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
            crate::trace_dispatch!("computable", "constructor", "rational-zero-canonicalized");
            return Self::zero();
        }
        if r.is_one() {
            // Route rational one through the dedicated One node so callers that
            // import exact f64/integer identities get the same cheap constructor
            // and structural facts as `Computable::one()`.
            crate::trace_dispatch!("computable", "constructor", "rational-one-canonicalized");
            return Self::one();
        }
        crate::trace_dispatch!("computable", "constructor", "rational-node");
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

    fn integer_pi_plus_rational(&self) -> Option<(BigInt, Rational)> {
        // Trig reducers often see values like k*pi + r after symbolic algebra.
        // If k is an exact integer, the period/parity can be handled without
        // estimating a quotient or building a cancellation-prone residual.
        fn extract(term: &Computable, offset: &Computable) -> Option<(BigInt, Rational)> {
            let rational = offset.exact_rational()?;
            let residual_is_kernel_sized = rational.sign() == Sign::NoSign
                || rational.msd_exact().is_some_and(|msd| msd < 0)
                || Computable::exact_rational_half_pi_shortcut_magnitude(&rational).is_some();
            if !residual_is_kernel_sized {
                return None;
            }
            let (constant, scale) = term.shared_constant_term()?;
            let pi_scale = match constant {
                SharedConstant::Pi => scale,
                SharedConstant::Tau => scale * Rational::new(2),
                _ => return None,
            };
            pi_scale
                .to_big_integer()
                .map(|multiple| (multiple, rational))
        }

        match &*self.internal {
            Approximation::Add(left, right) => {
                extract(left, right).or_else(|| extract(right, left))
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
            if constant == SharedConstant::Pi
                && let Some(cached) =
                    Self::cached_shared_constant_at_precision(SharedConstant::Tau, p + 1)
            {
                // The same identity works in reverse: a tau approximation at
                // precision p+1 is already a pi approximation at precision p.
                // This matters for applications that use tau for trig
                // construction and later format pi; reuse the costly Machin
                // approximation instead of recomputing it under a different
                // shared-constant key.
                Self::store_shared_constant_cache_value(SharedConstant::Pi, p, cached.clone());
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
            if p == cached.0 {
                // Reusing the exact cached precision avoids a no-op BigInt shift.
                Some(cached.1)
            } else {
                Some(scale(cached.1, cached.0 - p))
            }
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
                if p == *cache_prec {
                    // Reusing shared-constant precision avoids extra shift work.
                    Some(cache_appr.clone())
                } else {
                    Some(scale(cache_appr.clone(), *cache_prec - p))
                }
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
            Approximation::AtanRational(r) => Some(BoundInfo::with_sign_msd(r.sign(), None, false)),
            Approximation::AsinRational(r) => Some(BoundInfo::with_sign_msd(r.sign(), None, false)),
            Approximation::AsinhRational(r) | Approximation::AtanhRational(r) => {
                Some(BoundInfo::with_sign_msd(r.sign(), None, false))
            }
            Approximation::PrescaledSinRational(r) | Approximation::PrescaledTanRational(r) => {
                Some(BoundInfo::with_sign_msd(r.sign(), None, false))
            }
            Approximation::PrescaledCosRational(_) => {
                Some(BoundInfo::with_sign_msd(Sign::Plus, None, false))
            }
            Approximation::PrescaledCosHalfPiMinusRational(_)
            | Approximation::PrescaledSinHalfPiMinusRational(_)
            | Approximation::PrescaledCotHalfPiMinusRational(_) => {
                Some(BoundInfo::with_sign_msd(Sign::Plus, None, false))
            }
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
                Approximation::AtanRational(r) => {
                    Some(BoundInfo::with_sign_msd(r.sign(), None, false))
                }
                Approximation::AsinRational(r) => {
                    Some(BoundInfo::with_sign_msd(r.sign(), None, false))
                }
                Approximation::AsinhRational(r) | Approximation::AtanhRational(r) => {
                    Some(BoundInfo::with_sign_msd(r.sign(), None, false))
                }
                Approximation::PrescaledSinRational(r) | Approximation::PrescaledTanRational(r) => {
                    Some(BoundInfo::with_sign_msd(r.sign(), None, false))
                }
                Approximation::PrescaledCosRational(_) => {
                    Some(BoundInfo::with_sign_msd(Sign::Plus, None, false))
                }
                Approximation::PrescaledCosHalfPiMinusRational(_)
                | Approximation::PrescaledSinHalfPiMinusRational(_)
                | Approximation::PrescaledCotHalfPiMinusRational(_) => {
                    Some(BoundInfo::with_sign_msd(Sign::Plus, None, false))
                }
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

        // Reserve small fixed-size stacks because bound queries are often called
        // on long symbolic chains and should not allocate repeatedly under
        // repeated structural fact traffic.
        let mut frames = Vec::with_capacity(16);
        let mut values: Vec<BoundInfo> = Vec::with_capacity(8);
        frames.push(Frame::Eval(self));

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
                Approximation::AtanRational(r) => Some(Some(r.sign())),
                Approximation::AsinRational(r) => Some(Some(r.sign())),
                Approximation::AsinhRational(r) | Approximation::AtanhRational(r) => {
                    Some(Some(r.sign()))
                }
                Approximation::PrescaledSinRational(r) | Approximation::PrescaledTanRational(r) => {
                    Some(Some(r.sign()))
                }
                Approximation::PrescaledCosRational(_) => Some(Some(Sign::Plus)),
                Approximation::PrescaledCosHalfPiMinusRational(_)
                | Approximation::PrescaledSinHalfPiMinusRational(_)
                | Approximation::PrescaledCotHalfPiMinusRational(_) => Some(Some(Sign::Plus)),
                Approximation::AcosPositive(_)
                | Approximation::AcosPositiveRational(_)
                | Approximation::AcosNegativeRational(_)
                | Approximation::AcoshNearOne(_)
                | Approximation::AcoshDirect(_) => Some(Some(Sign::Plus)),
                Approximation::PrescaledAtan(child)
                | Approximation::PrescaledAsin(child)
                | Approximation::AsinDeferred(child)
                | Approximation::AsinhNearZero(child)
                | Approximation::AsinhDirect(child)
                | Approximation::PrescaledAsinh(child)
                | Approximation::AtanhDirect(child)
                | Approximation::PrescaledAtanh(child) => Some(child.exact_sign()),
                Approximation::PrescaledExp(_) => Some(Some(Sign::Plus)),
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

        // Structural sign on deep chains stays allocation-light so predicate-heavy
        // code does not needlessly allocate during exact-sign walks.
        let mut frames = Vec::with_capacity(16);
        let mut values: Vec<Option<Sign>> = Vec::with_capacity(8);
        frames.push(Frame::Eval(self));

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

    pub(crate) fn planning_sign_and_msd(&self) -> (Option<Sign>, Option<Option<Precision>>) {
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
        // Use fixed-precision values directly and explicit remainder correction
        // to avoid creating an extra inverse path before the caller's own
        // correction loop.
        // This keeps reduction for very large arguments on the cheap path while
        // preserving Payne-Hanek-style behavior.
        let precision: Precision = -4;
        let numerator = self.approx(precision);
        let denominator = divisor.approx(precision);
        if denominator.is_zero() {
            return BigInt::zero();
        }

        let same_sign = numerator.sign() == denominator.sign();
        let abs_numerator = numerator.magnitude().clone();
        let abs_denominator = denominator.magnitude().clone();
        let mut quotient = abs_numerator.clone() / abs_denominator.clone();
        let remainder = abs_numerator % abs_denominator.clone();
        if remainder >= (abs_denominator >> 1) {
            quotient += 1_u32;
        }

        if same_sign {
            BigInt::from_biguint(Sign::Plus, quotient)
        } else {
            BigInt::from_biguint(Sign::Minus, quotient)
        }
    }

    fn reduce_by_divisor(
        &self,
        divisor: &Self,
        low_prec: Precision,
        max_attempts: u32,
    ) -> Option<(Self, BigInt)> {
        let mut multiple = self.integer_ratio_nearest(divisor.clone());

        for _ in 0..max_attempts {
            let adjustment = divisor
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

            return Some((reduced, multiple));
        }

        None
    }

    fn prescaled_exp(self) -> Self {
        // Preserve structural form while deferring the expensive approximation of
        // exp to the requested precision; this avoids recursive constructor
        // expansion when explicit reduction has already normalized the input.
        // exp(x) stays strictly positive across all domain values, so cache
        // that fact directly for fast sign/zero checks.
        Self {
            internal: Box::new(Approximation::PrescaledExp(self)),
            cache: RefCell::new(Cache::Invalid),
            bound: RefCell::new(BoundCache::Invalid),
            exact_sign: RefCell::new(ExactSignCache::Valid(Sign::Plus)),
            signal: None,
        }
    }

    /// Natural Exponential function, raise Euler's Number to this number.
    pub fn exp(self) -> Computable {
        if self.exact_rational().as_ref().is_some_and(Rational::is_one) {
            // e^1 is the shared cached constant, not a fresh PrescaledExp node.
            crate::trace_dispatch!("computable", "exp", "exact-one-shared-e");
            return Self::e_constant();
        }
        if self
            .exact_rational()
            .is_some_and(|r| r.sign() == Sign::NoSign)
        {
            // e^0 is exact and must stay outside the approximation pipeline.
            crate::trace_dispatch!("computable", "exp", "exact-zero-one");
            return Self::one();
        }
        if let Some(msd) = self.planning_sign_and_msd().1.flatten() {
            if msd <= 2 {
                crate::trace_dispatch!("computable", "exp", "structural-small-prescaled");
                return self.prescaled_exp();
            }
            if msd >= 4 {
                crate::trace_dispatch!("computable", "exp", "structural-large-range-reduction");
                let ln2 = Self::ln2();
                let low_prec: Precision = -4;
                const REDUCTION_MAX_ATTEMPTS: u32 = 64;

                if let Some((reduced, multiple)) =
                    self.reduce_by_divisor(&ln2, low_prec, REDUCTION_MAX_ATTEMPTS)
                {
                    crate::trace_dispatch!("computable", "exp", "ln2-range-reduction");
                    return reduced.prescaled_exp().shift_left(
                        multiple
                            .try_into()
                            .expect("binary exponent should fit in i32"),
                    );
                }

                // If the cheap correction loop cannot converge at this scale,
                // prefer preserving a deferred-expensive symbolic form over
                // recursing deeper and risking stack blowup.
                crate::trace_dispatch!("computable", "exp", "ln2-range-reduction-fallback");
                return self.prescaled_exp();
            }
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
            const REDUCTION_MAX_ATTEMPTS: u32 = 64;

            if let Some((reduced, multiple)) =
                self.reduce_by_divisor(&ln2, low_prec, REDUCTION_MAX_ATTEMPTS)
            {
                crate::trace_dispatch!("computable", "exp", "ln2-range-reduction");
                return reduced.prescaled_exp().shift_left(
                    multiple
                        .try_into()
                        .expect("binary exponent should fit in i32"),
                );
            }

            // Fallback keeps large, symbolic arguments on the cold path and
            // avoids recursive expansion when the correction loop cannot
            // stabilize quickly.
            crate::trace_dispatch!("computable", "exp", "ln2-range-reduction-fallback");
            return self.prescaled_exp();
        }

        crate::trace_dispatch!("computable", "exp", "prescaled-kernel");
        self.prescaled_exp()
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
    pub(super) fn half_pi_multiple(&self) -> BigInt {
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

    pub(super) fn half_pi_multiple_exact_rational(rational: &Rational) -> Option<BigInt> {
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
            Approximation::Negate(child) => child.cheap_bound().known_msd(),
            Approximation::Offset(child, n) => child
                .cheap_bound()
                .known_msd()
                .map(|msd| msd.map(|value| value + *n)),
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
                crate::trace_dispatch!("computable", "cos", "exact-zero-one");
                return Self::one();
            }
            if rational.msd_exact().is_some_and(|msd| msd >= 3) {
                crate::trace_dispatch!("computable", "cos", "large-rational-deferred");
                return Self::cos_large_rational_deferred(rational);
            }
            if let Some(magnitude) = Self::exact_rational_half_pi_shortcut_magnitude(&rational) {
                // cos(r) = sin(pi/2 - |r|) for exact medium positive/negative
                // rationals, keeping the generic subtraction node out of the path.
                crate::trace_dispatch!("computable", "cos", "medium-rational-half-pi-rewrite");
                return Self::prescaled_sin_half_pi_minus_rational(magnitude);
            }
        }
        if let Some((multiple, residual)) = self.integer_pi_plus_rational() {
            crate::trace_dispatch!("computable", "cos", "integer-pi-plus-rational");
            let reduced = Self::rational(residual).cos();
            return if (&multiple % signed::TWO.deref()).is_zero() {
                reduced
            } else {
                reduced.negate()
            };
        }
        if let Some(msd) = self.trig_reduction_msd() {
            if msd <= 0 {
                // Known |x| < 2: go directly to the prescaled Taylor kernel.
                // The fallback rough approximation stays in place for unknown
                // magnitudes where structural bounds are not trustworthy.
                crate::trace_dispatch!("computable", "cos", "structural-small-prescaled");
                return Self::prescaled_cos(self);
            }
            if msd >= 3 {
                // Known |x| >= 8: skip the preliminary `approx(-1)` and go
                // straight to half-pi reduction. This is the hot large-argument
                // path for generic sin/cos benchmarks.
                let multiplier = Self::half_pi_multiple(&self);
                crate::trace_dispatch!("computable", "cos", "structural-large-half-pi-reduction");
                return self.cos_reduced_by_half_pi(multiplier);
            }
        }
        let rough_appr = self.approx(-1);
        let abs_rough_appr = rough_appr.magnitude();

        if abs_rough_appr < unsigned::TWO.deref() {
            crate::trace_dispatch!("computable", "cos", "rough-small-prescaled");
            return Self::prescaled_cos(self);
        }

        let multiplier = if abs_rough_appr < unsigned::SIX.deref() {
            // Medium arguments can reuse the rough quadrant table. Larger values need the
            // more expensive nearest-half-pi reduction to keep the residual small.
            crate::trace_dispatch!("computable", "cos", "rough-medium-half-pi-reduction");
            Self::medium_half_pi_multiple(&rough_appr)
        } else {
            crate::trace_dispatch!("computable", "cos", "generic-half-pi-reduction");
            Self::half_pi_multiple(&self)
        };
        self.cos_reduced_by_half_pi(multiplier)
    }

    /// Sine of this number.
    pub fn sin(self) -> Computable {
        if let Some(rational) = self.exact_rational() {
            if rational.sign() == Sign::NoSign {
                crate::trace_dispatch!("computable", "sin", "exact-zero");
                return Self::zero();
            }
            if rational.msd_exact().is_some_and(|msd| msd >= 3) {
                crate::trace_dispatch!("computable", "sin", "large-rational-deferred");
                return Self::sin_large_rational_deferred(rational);
            }
            if let Some(magnitude) = Self::exact_rational_half_pi_shortcut_magnitude(&rational) {
                // sin(r) = +/-cos(pi/2 - |r|) in the same exact medium window
                // used by cosine, preserving odd symmetry outside the kernel.
                crate::trace_dispatch!("computable", "sin", "medium-rational-half-pi-rewrite");
                let result = Self::prescaled_cos_half_pi_minus_rational(magnitude);
                return if rational.sign() == Sign::Minus {
                    result.negate()
                } else {
                    result
                };
            }
        }
        if let Some((multiple, residual)) = self.integer_pi_plus_rational() {
            crate::trace_dispatch!("computable", "sin", "integer-pi-plus-rational");
            let reduced = Self::rational(residual).sin();
            return if (&multiple % signed::TWO.deref()).is_zero() {
                reduced
            } else {
                reduced.negate()
            };
        }
        if let Some(msd) = self.trig_reduction_msd() {
            if msd <= 0 {
                // Known |x| < 2: direct prescaled sine avoids reduction setup.
                crate::trace_dispatch!("computable", "sin", "structural-small-prescaled");
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

                crate::trace_dispatch!("computable", "sin", "structural-large-half-pi-reduction");
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
            crate::trace_dispatch!("computable", "sin", "rough-small-prescaled");
            return Self::prescaled_sin(self);
        }

        if abs_rough_appr < unsigned::SIX.deref() {
            // Medium sine inputs are rewritten through exact symmetries instead of going
            // through the generic half-pi division path.
            let multiplier = Self::medium_half_pi_multiple(&rough_appr);
            crate::trace_dispatch!("computable", "sin", "rough-medium-special-rewrite");
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

        crate::trace_dispatch!("computable", "sin", "generic-half-pi-reduction");
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
        if let Some(rational) = self.exact_rational() {
            if rational.sign() == Sign::NoSign {
                crate::trace_dispatch!("computable", "tan", "exact-zero");
                return Self::zero();
            }
            if let Some(magnitude) = Self::exact_rational_half_pi_shortcut_magnitude(&rational) {
                crate::trace_dispatch!("computable", "tan", "medium-rational-half-pi-cotangent");
                let result = Self::prescaled_cot_half_pi_minus_rational(magnitude);
                return if rational.sign() == Sign::Minus {
                    result.negate()
                } else {
                    result
                };
            }
            if rational.msd_exact().is_some_and(|msd| msd >= 3) {
                crate::trace_dispatch!("computable", "tan", "large-rational-deferred");
                return Self::tan_large_rational_deferred(rational);
            }
        }
        if let Some((_multiple, residual)) = self.integer_pi_plus_rational() {
            // tan has period pi, so any exact integer pi multiple drops out.
            crate::trace_dispatch!("computable", "tan", "integer-pi-plus-rational");
            return Self::rational(residual).tan();
        }
        if self.planning_sign_and_msd().0 == Some(Sign::Minus) {
            // Odd symmetry lets known-negative values reuse the positive reducer
            // without paying a low-precision approximation just to discover sign.
            crate::trace_dispatch!("computable", "tan", "known-negative-symmetry");
            return self.negate().tan().negate();
        }
        if let Some(msd) = self.trig_reduction_msd() {
            if msd <= 0 {
                // Known |x| < 2: enter the tangent quotient kernel directly.
                crate::trace_dispatch!("computable", "tan", "structural-small-prescaled");
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
            crate::trace_dispatch!("computable", "tan", "rough-negative-symmetry");
            return self.negate().tan().negate();
        }

        let abs_rough_appr = rough_appr.magnitude();

        if abs_rough_appr < unsigned::TWO.deref() {
            crate::trace_dispatch!("computable", "tan", "rough-small-prescaled");
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
            crate::trace_dispatch!("computable", "tan", "near-half-pi-cotangent-rewrite");
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
            crate::trace_dispatch!("computable", "tan", "near-pi-reflection");
            return Self::pi().add(self.negate()).tan().negate();
        }

        let multiplier = Self::pi_multiple(&self);
        let adjustment =
            Self::pi().multiply(Self::rational(Rational::from_bigint(multiplier)).negate());
        crate::trace_dispatch!("computable", "tan", "generic-pi-reduction");
        self.add(adjustment).tan()
    }

    pub(crate) fn sin_rational(rational: Rational) -> Computable {
        // Real-level rational trig already owns the Rational. Classify it here
        // so hot scalar constructors skip Ratio allocation followed by the same
        // exact-rational rediscovery inside Computable::sin.
        if rational.sign() == Sign::NoSign {
            crate::trace_dispatch!("computable", "sin", "exact-zero");
            return Self::zero();
        }
        if rational.magnitude_at_least_power_of_two(3) {
            crate::trace_dispatch!("computable", "sin", "large-rational-deferred");
            return Self::sin_large_rational_deferred(rational);
        }
        if let Some(magnitude) = Self::exact_rational_half_pi_shortcut_magnitude(&rational) {
            crate::trace_dispatch!("computable", "sin", "medium-rational-half-pi-rewrite");
            let result = Self::prescaled_cos_half_pi_minus_rational(magnitude);
            return if rational.sign() == Sign::Minus {
                result.negate()
            } else {
                result
            };
        }
        if rational.msd_exact().is_some_and(|msd| msd < 0) {
            crate::trace_dispatch!("computable", "sin", "structural-small-prescaled");
            return Self::prescaled_sin_rational(rational);
        }
        crate::trace_dispatch!("computable", "sin", "owned-rational-generic");
        Self::rational(rational).sin()
    }

    pub(crate) fn cos_rational(rational: Rational) -> Computable {
        // Owned rational cosine mirrors sin_rational. Keeping the branch table
        // shared at this level removes a constructor-only Ratio node from every
        // plain Real::cos(rational) call without changing approximation kernels.
        if rational.sign() == Sign::NoSign {
            crate::trace_dispatch!("computable", "cos", "exact-zero-one");
            return Self::one();
        }
        if rational.magnitude_at_least_power_of_two(3) {
            crate::trace_dispatch!("computable", "cos", "large-rational-deferred");
            return Self::cos_large_rational_deferred(rational);
        }
        if let Some(magnitude) = Self::exact_rational_half_pi_shortcut_magnitude(&rational) {
            crate::trace_dispatch!("computable", "cos", "medium-rational-half-pi-rewrite");
            return Self::prescaled_sin_half_pi_minus_rational(magnitude);
        }
        if rational.msd_exact().is_some_and(|msd| msd < 0) {
            crate::trace_dispatch!("computable", "cos", "structural-small-prescaled");
            return Self::prescaled_cos_rational(rational);
        }
        crate::trace_dispatch!("computable", "cos", "owned-rational-generic");
        Self::rational(rational).cos()
    }

    pub(crate) fn tan_rational(rational: Rational) -> Computable {
        // Tangent benefits most from classifying before Ratio construction: the
        // generic path probes sign/MSD and may build symmetry wrappers before it
        // reaches the small or near-pole kernel.
        if rational.sign() == Sign::NoSign {
            crate::trace_dispatch!("computable", "tan", "exact-zero");
            return Self::zero();
        }
        if let Some(magnitude) = Self::exact_rational_half_pi_shortcut_magnitude(&rational) {
            crate::trace_dispatch!("computable", "tan", "medium-rational-half-pi-cotangent");
            let result = Self::prescaled_cot_half_pi_minus_rational(magnitude);
            return if rational.sign() == Sign::Minus {
                result.negate()
            } else {
                result
            };
        }
        if rational.magnitude_at_least_power_of_two(3) {
            crate::trace_dispatch!("computable", "tan", "large-rational-deferred");
            return Self::tan_large_rational_deferred(rational);
        }
        if rational.msd_exact().is_some_and(|msd| msd < 0) {
            crate::trace_dispatch!("computable", "tan", "structural-small-prescaled");
            return Self::prescaled_tan_rational(rational);
        }
        crate::trace_dispatch!("computable", "tan", "owned-rational-generic");
        Self::rational(rational).tan()
    }

    fn ln2() -> Self {
        Self::shared_constant(SharedConstant::Ln2)
    }

    fn factor_small_prime_power(value: &mut BigUint, prime: u32) -> i32 {
        let prime_big = BigUint::from(prime);
        let mut exponent = 0_i32;
        while !value.is_zero() && (&*value % &prime_big).is_zero() {
            *value /= &prime_big;
            exponent = exponent
                .checked_add(1)
                .expect("small-prime factor exponent should fit in i32");
        }
        exponent
    }

    fn ln_smooth_rational(rational: &Rational) -> Option<Self> {
        if rational.sign() != Sign::Plus {
            return None;
        }

        let mut numerator = rational.numerator().clone();
        let mut denominator = rational.denominator().clone();
        let mut terms = Vec::with_capacity(4);
        for base in [2_u32, 3, 5, 7] {
            let exponent = Self::factor_small_prime_power(&mut numerator, base)
                - Self::factor_small_prime_power(&mut denominator, base);
            if exponent != 0 {
                terms.push((base, exponent));
            }
        }
        if numerator != BigUint::one() || denominator != BigUint::one() || terms.is_empty() {
            return None;
        }
        if terms.len() == 1 && terms[0].1 == 1 {
            // Shared log constants approximate themselves by evaluating the
            // corresponding exact rational log. Do not rewrite ln(3) to the
            // shared ln3 node here or that internal cache fill would recurse.
            // Composite smooth values such as 9, 6, 45/14 still reduce below.
            return None;
        }

        // ln(prod p_i^e_i) = sum e_i ln(p_i). Retaining this symbolic sum
        // lets smooth exact rationals reuse shared log caches and delays all
        // series evaluation until the final requested precision. This is the
        // same argument-reduction principle used by the elementary kernels
        // below, applied at construction time for common scalar/matrix constants.
        let mut result = Self::zero();
        for (base, exponent) in terms {
            let magnitude = BigInt::from(exponent.abs());
            let mut term = Self::ln_constant(base)
                .expect("smooth-log bases are all shared")
                .multiply(Self::integer(magnitude));
            if exponent < 0 {
                term = term.negate();
            }
            result = result.add(term);
        }
        Some(result)
    }

    fn ln_shared_or_smooth_rational(rational: &Rational) -> Option<Self> {
        // Shared logs for small smooth bases reuse one approximation cache
        // across all expressions. Extracting the integer once prevents the
        // older pattern of constructing several candidate rationals just to
        // reject them. Smooth-factor decomposition below remains exact.
        if let Some(integer) = rational.to_integer_i64() {
            match integer {
                2 => {
                    crate::trace_dispatch!("computable", "ln", "shared-ln2");
                    return Some(Self::ln_constant(2).unwrap());
                }
                3 => {
                    crate::trace_dispatch!("computable", "ln", "shared-ln3");
                    return Some(Self::ln_constant(3).unwrap());
                }
                5 => {
                    crate::trace_dispatch!("computable", "ln", "shared-ln5");
                    return Some(Self::ln_constant(5).unwrap());
                }
                6 => {
                    crate::trace_dispatch!("computable", "ln", "shared-ln6");
                    return Some(Self::ln_constant(6).unwrap());
                }
                7 => {
                    crate::trace_dispatch!("computable", "ln", "shared-ln7");
                    return Some(Self::ln_constant(7).unwrap());
                }
                10 => {
                    crate::trace_dispatch!("computable", "ln", "shared-ln10");
                    return Some(Self::ln_constant(10).unwrap());
                }
                _ => {}
            }
        }
        if let Some(reduced) = Self::ln_smooth_rational(rational) {
            crate::trace_dispatch!("computable", "ln", "smooth-rational-shared-log-sum");
            return Some(reduced);
        }
        None
    }

    fn ln_exact_rational(rational: Rational) -> Self {
        // Internal exact-rational log constructor for reductions that already
        // have a positive rational argument. It reuses the shared small-log
        // constants instead of building fresh generic PrescaledLn trees.
        if rational.is_one() {
            crate::trace_dispatch!("computable", "ln", "exact-rational-one");
            return Self::zero();
        }
        if rational.sign() == Sign::Minus || rational.sign() == Sign::NoSign {
            crate::trace_dispatch!("computable", "ln", "exact-rational-domain-error");
            panic!("ArithmeticException");
        }
        if rational < Rational::one() {
            crate::trace_dispatch!("computable", "ln", "exact-rational-inverse-rewrite");
            return Self::ln_exact_rational(rational.inverse().unwrap()).negate();
        }
        if let Some(reduced) = Self::ln_shared_or_smooth_rational(&rational) {
            return reduced;
        }
        crate::trace_dispatch!("computable", "ln", "exact-rational-generic");
        Self::rational(rational).ln()
    }

    /// Natural logarithm of this number.
    pub fn ln(self) -> Computable {
        if self.exact_rational().is_some_and(|r| r.is_one()) {
            crate::trace_dispatch!("computable", "ln", "exact-one-zero");
            return Self::zero();
        }
        if let Approximation::Ratio(r) = &*self.internal
            && r.sign() == Sign::Plus
        {
            let (shift, reduced) = r.factor_two_powers();
            if shift != 0 {
                // ln(r * 2^k) = ln(r) + k ln(2). Pulling dyadic scale out keeps
                // f64-derived rationals on a cheap symbolic/log path. The
                // reduced factor is routed through exact-rational log reduction
                // so smooth values like 45/14 become cached prime-log sums
                // instead of low-precision probing plus a fresh ln1p tree.
                let reduced_ln = if reduced.is_one() {
                    Self::integer(BigInt::zero())
                } else {
                    Self::ln_exact_rational(reduced)
                };
                let shift: BigInt = shift.into();
                crate::trace_dispatch!("computable", "ln", "dyadic-scale-rewrite");
                return reduced_ln.add(Self::integer(shift).multiply(Self::ln2()));
            } else if let Some(reduced) = Self::ln_smooth_rational(r) {
                crate::trace_dispatch!("computable", "ln", "smooth-rational-shared-log-sum");
                return reduced;
            }
        }

        // Sixteenths, ie 8 == 0.5, 24 == 1.5
        let low_ln_limit = signed::EIGHT.deref();
        let high_ln_limit = signed::TWENTY_FOUR.deref();

        let low_prec = -4;
        let (known_sign, planning_msd) = self.planning_sign_and_msd();
        if known_sign == Some(Sign::Minus) {
            crate::trace_dispatch!("computable", "ln", "domain-negative-structural");
            panic!("ArithmeticException");
        }
        if let Some(msd) = planning_msd.flatten() {
            if known_sign == Some(Sign::Plus) && msd <= -2 {
                // Rewriting ln(x) -> -ln(1/x) is safe once |x| <= 1/4;
                // msd <= -2 guarantees that without extra probing.
                crate::trace_dispatch!("computable", "ln", "small-inverse-rewrite-structural");
                return self.inverse().ln().negate();
            }
            if known_sign == Some(Sign::Plus) && msd == 5 {
                let quarter = self.sqrt().sqrt().ln();
                crate::trace_dispatch!("computable", "ln", "sqrt-range-reduction");
                return quarter.shift_left(2);
            }
            if known_sign == Some(Sign::Plus) && msd >= 7 {
                // |x| >= 128 always exceeds the rough high-magnitude branch,
                // so scale first and skip the initial rough probe.
                let mut extra_bits: i32 = (msd as i32 - 5).try_into().expect(
                    "Approximation should have few enough bits to fit in a 32-bit signed integer",
                );

                let mut scaled = self.clone().shift_right(extra_bits);
                let mut scaled_rough = scaled.approx(low_prec);
                while scaled_rough >= *high_ln_limit {
                    extra_bits = extra_bits.checked_add(1).expect(
                        "Approximation should have few enough bits to fit in a 32-bit signed integer",
                    );
                    scaled = self.clone().shift_right(extra_bits);
                    scaled_rough = scaled.approx(low_prec);
                }

                let scaled_result = scaled.ln();
                let extra: BigInt = extra_bits.into();
                crate::trace_dispatch!("computable", "ln", "binary-scale-reduction");
                return scaled_result.add(Self::integer(extra).multiply(Self::ln2()));
            }
        }
        let rough_appr = self.approx(low_prec);
        if rough_appr < BigInt::zero() {
            crate::trace_dispatch!("computable", "ln", "domain-negative");
            panic!("ArithmeticException");
        }
        if rough_appr <= *low_ln_limit {
            // For values below 0.5, invert and negate so the prescaled ln1p kernel sees a
            // better-conditioned argument.
            crate::trace_dispatch!("computable", "ln", "small-inverse-rewrite");
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
                crate::trace_dispatch!("computable", "ln", "sqrt-range-reduction");
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
                crate::trace_dispatch!("computable", "ln", "binary-scale-reduction");
                return scaled_result.add(Self::integer(extra).multiply(Self::ln2()));
            }
        }

        let minus_one = Self::integer(signed::MINUS_ONE.clone());
        let fraction = Self::add(self, minus_one);
        // Final path is ln(1+x), where the prior reductions keep |x| small enough for the
        // prescaled series.
        crate::trace_dispatch!("computable", "ln", "prescaled-ln1p-kernel");
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
                Some(Sign::Plus) => {
                    crate::trace_dispatch!("computable", "sqrt", "square-positive-collapse");
                    return child.clone();
                }
                Some(Sign::Minus) => {
                    crate::trace_dispatch!("computable", "sqrt", "square-negative-abs-collapse");
                    return child.clone().negate();
                }
                Some(Sign::NoSign) => {
                    crate::trace_dispatch!("computable", "sqrt", "square-zero-collapse");
                    return Self::zero();
                }
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
                crate::trace_dispatch!("computable", "sqrt", "scaled-square-collapse");
                return value;
            }
            if let Some(scale) = right.exact_rational()
                && let Some(value) = reduced(scale, left)
            {
                crate::trace_dispatch!("computable", "sqrt", "scaled-square-collapse");
                return value;
            }
        }
        if let Some(rational) = self.exact_rational()
            && rational.sign() != Sign::Minus
            && rational.extract_square_will_succeed()
        {
            // Perfect rational squares stay exact. For scaled sqrt(2)/sqrt(3)
            // residuals, keep the irrational part shared and the exact scale
            // symbolic. Plain sqrt(2) and sqrt(3) deliberately stay on the old
            // generic node because repeated cached approximation of a single
            // node is faster than a thread-local shared-cache lookup.
            let (root, rest) = rational.extract_square_reduced();
            if rest.is_one() {
                crate::trace_dispatch!("computable", "sqrt", "exact-rational-square");
                return Self::rational(root);
            }
            if !root.is_one()
                && let Some(shared_radicand @ (2 | 3)) = rest.to_integer_i64()
            {
                // For scaled sqrt(2)/sqrt(3), reuse the shared constant cache
                // for the irrational factor and keep the exact rational scale
                // separate. This is a measured construction-time win for scaled
                // square-free inputs without changing the single-node path for
                // plain sqrt(2)/sqrt(3).
                crate::trace_dispatch!("computable", "sqrt", "shared-squarefree-rational");
                let constant = Self::sqrt_constant(shared_radicand)
                    .expect("sqrt(2) and sqrt(3) are shared constants");
                return constant.multiply(Self::rational(root));
            }
        }
        crate::trace_dispatch!("computable", "sqrt", "generic-sqrt-node");
        let exact_sign = match *self.exact_sign.borrow() {
            // Square roots are nonnegative where defined and preserve structural zero.
            ExactSignCache::Valid(Sign::NoSign) => ExactSignCache::Valid(Sign::NoSign),
            ExactSignCache::Valid(Sign::Plus) | ExactSignCache::Valid(Sign::Minus) => {
                ExactSignCache::Valid(Sign::Plus)
            }
            _ => ExactSignCache::Invalid,
        };
        Self {
            internal: Box::new(Approximation::Sqrt(self)),
            cache: RefCell::new(Cache::Invalid),
            bound: RefCell::new(BoundCache::Invalid),
            exact_sign: RefCell::new(exact_sign),
            signal: None,
        }
    }

    pub(crate) fn prescaled_atan(n: BigInt) -> Self {
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

    fn atan_rational_deferred(rational: Rational) -> Self {
        // Exact rational atan reductions used to allocate intermediate
        // add/multiply/inverse nodes before reaching the small atan series. This
        // deferred node keeps the public constructor compact and performs the
        // same range reductions directly in the approximation kernel.
        crate::trace_dispatch!("computable", "constructor", "atan-rational-deferred");
        Self {
            internal: Box::new(Approximation::AtanRational(rational)),
            cache: RefCell::new(Cache::Invalid),
            bound: RefCell::new(BoundCache::Invalid),
            exact_sign: RefCell::new(ExactSignCache::Invalid),
            signal: None,
        }
    }

    fn asin_rational_deferred(rational: Rational) -> Self {
        // Exact rational asin uses a direct series for tiny/moderate inputs.
        // Storing the Rational in the approximation node keeps the hot path out
        // of the generic sqrt/atan transform and avoids a child approx lookup.
        crate::trace_dispatch!("computable", "constructor", "asin-rational-deferred");
        let sign = rational.sign();
        Self {
            internal: Box::new(Approximation::AsinRational(rational)),
            cache: RefCell::new(Cache::Invalid),
            bound: RefCell::new(BoundCache::Invalid),
            exact_sign: RefCell::new(ExactSignCache::Valid(sign)),
            signal: None,
        }
    }

    /// Arctangent of this number.
    pub fn atan(self) -> Computable {
        if let Some(rational) = self.exact_rational() {
            if rational.sign() == Sign::NoSign {
                crate::trace_dispatch!("computable", "atan", "exact-zero");
                return Self::zero();
            }
            if rational.sign() == Sign::Plus {
                crate::trace_dispatch!("computable", "atan", "exact-rational-deferred");
                return Self::atan_rational_deferred(rational);
            }
        }
        let (known_sign, planning_msd) = self.planning_sign_and_msd();
        if known_sign == Some(Sign::Minus) {
            crate::trace_dispatch!("computable", "atan", "known-negative-symmetry");
            return self.negate().atan().negate();
        }
        if known_sign.is_none() && self.exact_sign() == Some(Sign::Minus) {
            crate::trace_dispatch!("computable", "atan", "known-negative-symmetry-fallback");
            return self.negate().atan().negate();
        }
        if let Some(msd) = planning_msd.flatten() {
            if msd < -1 {
                crate::trace_dispatch!("computable", "atan", "structural-small-prescaled");
                return Self {
                    internal: Box::new(Approximation::PrescaledAtan(self)),
                    cache: RefCell::new(Cache::Invalid),
                    bound: RefCell::new(BoundCache::Invalid),
                    exact_sign: RefCell::new(ExactSignCache::Invalid),
                    signal: None,
                };
            }
            if msd >= 5 {
                crate::trace_dispatch!("computable", "atan", "large-reciprocal-structural");
                return Self::pi()
                    .shift_right(1)
                    .add(self.inverse().atan().negate());
            }
        }

        let rough_appr = self.approx(-4);
        if rough_appr <= *signed::EIGHT {
            // Small atan arguments use the prescaled series directly.
            crate::trace_dispatch!("computable", "atan", "rough-small-prescaled");
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
            crate::trace_dispatch!("computable", "atan", "medium-atan-half-reduction");
            return Self::prescaled_atan(BigInt::from(2_u8))
                .add(numerator.multiply(denominator.inverse()).atan());
        }

        // Large positive atan uses pi/2 - atan(1/x), which converges faster.
        crate::trace_dispatch!("computable", "atan", "large-reciprocal");
        Self::pi()
            .shift_right(1)
            .add(self.inverse().atan().negate())
    }

    /// Inverse sine of this number.
    pub fn asin(self) -> Computable {
        if let Some(rational) = self.exact_rational() {
            match rational.sign() {
                Sign::NoSign => {
                    crate::trace_dispatch!("computable", "asin", "exact-zero");
                    return Self::zero();
                }
                Sign::Minus => {
                    crate::trace_dispatch!("computable", "asin", "exact-negative-symmetry");
                    return self.negate().asin().negate();
                }
                Sign::Plus => {
                    if rational.msd_exact().is_some_and(|msd| msd <= -4) {
                        // Tiny asin(x) is handled by its dedicated series; the generic
                        // atan transform builds extra sqrt/division nodes.
                        crate::trace_dispatch!("computable", "asin", "exact-tiny-rational-series");
                        return Self::asin_rational_deferred(rational);
                    }
                    if rational >= *INVERSE_ENDPOINT_RATIONAL_THRESHOLD {
                        // Near 1, use pi/2 - acos(x); acos has the endpoint transform.
                        crate::trace_dispatch!("computable", "asin", "endpoint-via-acos");
                        return Self::pi().shift_right(1).add(self.acos().negate());
                    }
                }
            }
        }
        if self.exact_sign() == Some(Sign::Minus) {
            crate::trace_dispatch!("computable", "asin", "known-negative-symmetry");
            return self.negate().asin().negate();
        }

        crate::trace_dispatch!("computable", "asin", "generic-atan-sqrt-transform");
        Self::asin_deferred(self)
    }

    /// Inverse cosine of this number.
    pub fn acos(self) -> Computable {
        if let Some(rational) = self.exact_rational() {
            if rational.is_one() {
                crate::trace_dispatch!("computable", "acos", "exact-one-zero");
                return Self::zero();
            }
            if rational.is_minus_one() {
                crate::trace_dispatch!("computable", "acos", "exact-minus-one-pi");
                return Self::pi();
            }
            if rational.sign() == Sign::NoSign {
                crate::trace_dispatch!("computable", "acos", "exact-zero-half-pi");
                return Self::pi().shift_right(1);
            }
            let rational_sign = rational.sign();
            let magnitude = if rational_sign == Sign::Minus {
                rational.neg()
            } else {
                rational
            };
            if magnitude.msd_exact().is_some_and(|msd| msd <= -4) {
                crate::trace_dispatch!("computable", "acos", "tiny-via-asin");
                return Self::pi().shift_right(1).add(self.asin().negate());
            }
            if rational_sign == Sign::Minus && magnitude >= *INVERSE_ENDPOINT_RATIONAL_THRESHOLD {
                // Negative endpoint values mirror the positive endpoint transform.
                // Store the magnitude directly so construction stays as a single
                // deferred fact instead of rebuilding pi - acos(|x|).
                crate::trace_dispatch!("computable", "acos", "negative-rational-deferred");
                return Self::acos_negative_rational_deferred(magnitude);
            }
            if rational_sign == Sign::Plus {
                crate::trace_dispatch!("computable", "acos", "positive-rational-deferred");
                return Self::acos_positive_rational_deferred(magnitude);
            }
        }

        if self.exact_sign() == Some(Sign::Plus) {
            // For positive values, acos(x) = 2 atan(sqrt((1-x)/(1+x))). This is the
            // endpoint-friendly path for values near 1.
            crate::trace_dispatch!("computable", "acos", "positive-endpoint-deferred");
            return Self::acos_positive(self);
        }

        crate::trace_dispatch!("computable", "acos", "generic-half-pi-minus-asin");
        Self::pi().shift_right(1).add(self.asin().negate())
    }

    /// Inverse hyperbolic sine of this number.
    pub fn asinh(self) -> Computable {
        let exact_rational = self.exact_rational();
        if exact_rational
            .as_ref()
            .is_some_and(|r| r.sign() == Sign::NoSign)
        {
            crate::trace_dispatch!("computable", "asinh", "exact-zero");
            return Self::zero();
        }
        let (known_sign, planned_msd) = self.planning_sign_and_msd();
        if exact_rational
            .as_ref()
            .is_some_and(|r| r.sign() == Sign::Minus)
            || known_sign == Some(Sign::Minus)
        {
            crate::trace_dispatch!("computable", "asinh", "known-negative-symmetry");
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
            crate::trace_dispatch!("computable", "asinh", "exact-tiny-prescaled");
            if let Some(rational) = exact_rational {
                return Self::asinh_rational_deferred(rational);
            }
            return Self::prescaled_asinh(self);
        }
        if exact_large {
            let radicand = self.clone().square().add(Self::one());
            crate::trace_dispatch!("computable", "asinh", "exact-large-direct-ln-sqrt");
            return self.add(radicand.sqrt()).ln();
        }
        let known_msd = planned_msd.flatten();
        let is_near_zero = match known_msd {
            Some(msd) => msd < 3,
            None => self.approx(-4) <= BigInt::from(64_u8),
        };
        if is_near_zero {
            // Direct Computable approximation benches include construction in
            // the measured work, and the eager graph caches its children better
            // than a deferred Real-only wrapper.
            let square = self.clone().square();
            let one = Self::one();
            let denominator = square.clone().add(one.clone()).sqrt().add(one);
            crate::trace_dispatch!("computable", "asinh", "near-zero-ln1p-transform");
            return self.add(square.multiply(denominator.inverse())).ln_1p();
        }

        let radicand = self.clone().square().add(Self::one());
        crate::trace_dispatch!("computable", "asinh", "generic-direct-ln-sqrt");
        self.add(radicand.sqrt()).ln()
    }

    /// Inverse hyperbolic cosine of this number. The caller is responsible for
    /// ensuring the input is in-domain.
    pub fn acosh(self) -> Computable {
        let exact_rational_msd = match self.internal.as_ref() {
            Approximation::One => {
                crate::trace_dispatch!("computable", "acosh", "exact-one-zero");
                return Self::zero();
            }
            Approximation::Ratio(r) => {
                if r.is_one() {
                    crate::trace_dispatch!("computable", "acosh", "exact-one-zero");
                    return Self::zero();
                }
                if r == &Rational::new(2) {
                    crate::trace_dispatch!("computable", "acosh", "exact-two-constant");
                    return Self::acosh2_constant();
                }
                if r >= &Rational::new(2) {
                    let radicand = r.clone() * r.clone() - Rational::one();
                    crate::trace_dispatch!(
                        "computable",
                        "acosh",
                        "exact-rational-at-least-two-direct-radicand"
                    );
                    return self.add(Self::sqrt_rational(radicand)).ln();
                }
                r.msd_exact()
            }
            Approximation::Int(n) => {
                if n == signed::ONE.deref() {
                    crate::trace_dispatch!("computable", "acosh", "exact-one-zero");
                    return Self::zero();
                }
                if n == signed::TWO.deref() {
                    crate::trace_dispatch!("computable", "acosh", "exact-two-constant");
                    return Self::acosh2_constant();
                }
                if n >= signed::TWO.deref() {
                    let r = Rational::from_bigint(n.clone());
                    let radicand = r.clone() * r - Rational::one();
                    crate::trace_dispatch!(
                        "computable",
                        "acosh",
                        "exact-integer-at-least-two-direct-radicand"
                    );
                    return self.add(Self::sqrt_rational(radicand)).ln();
                }
                if n.sign() == Sign::NoSign {
                    None
                } else {
                    Some(n.magnitude().bits() as Precision - 1)
                }
            }
            _ => None,
        };
        if let Approximation::Sqrt(child) = self.internal.as_ref()
            && child
                .exact_rational()
                .is_some_and(|r| r == Rational::new(2))
        {
            crate::trace_dispatch!("computable", "acosh", "sqrt-two-asinh-one");
            return Self::asinh1_constant();
        }
        if exact_rational_msd.is_some_and(|msd| msd >= 3) {
            // Large exact rationals skip the low-precision near-one probe and
            // use the direct acosh identity.
            let one = Self::one();
            let radicand = self.clone().square().add(one.negate());
            crate::trace_dispatch!("computable", "acosh", "exact-large-direct-ln-sqrt");
            return self.add(radicand.sqrt()).ln();
        }
        let known_msd = self.planning_sign_and_msd().1.flatten();
        let is_near_one = match known_msd {
            Some(msd) => msd < 3,
            None => self.approx(-4) <= BigInt::from(64_u8),
        };
        if is_near_one {
            // Keep the public Computable kernel eager for approximation-heavy
            // benches; Real uses a deferred wrapper when construction alone is
            // the hot path.
            let one = Self::one();
            let shifted = self.clone().add(one.clone().negate());
            let radicand = self.square().add(one.negate());
            crate::trace_dispatch!("computable", "acosh", "near-one-ln1p-transform");
            return shifted.add(radicand.sqrt()).ln_1p();
        }

        // Generic identity for already validated large inputs.
        let one = Self::one();
        let radicand = self.clone().square().add(one.negate());
        crate::trace_dispatch!("computable", "acosh", "generic-direct-ln-sqrt");
        self.add(radicand.sqrt()).ln()
    }

    /// Inverse hyperbolic tangent of this number. The caller is responsible for
    /// ensuring the input is in-domain.
    pub fn atanh(self) -> Computable {
        if let Some(rational) = self.exact_rational() {
            match rational.sign() {
                Sign::NoSign => {
                    crate::trace_dispatch!("computable", "atanh", "exact-zero");
                    return Self::zero();
                }
                Sign::Minus => {
                    crate::trace_dispatch!("computable", "atanh", "exact-negative-symmetry");
                    return self.negate().atanh().negate();
                }
                Sign::Plus => {
                    if rational.msd_exact().is_some_and(|msd| msd <= -4) {
                        // Tiny atanh(x) is best served by the direct odd series.
                        crate::trace_dispatch!("computable", "atanh", "exact-tiny-prescaled");
                        return Self::atanh_rational_deferred(rational);
                    }
                    if rational.is_one_half() {
                        crate::trace_dispatch!("computable", "atanh", "exact-half-ln3");
                        return Self::ln_constant(3)
                            .expect("ln3 is a shared log constant")
                            .multiply(Self::half());
                    }
                    if !rational.is_one() {
                        // For exact rationals, atanh(x) is one exact ln ratio.
                        // That keeps common factors in the logarithm constructor
                        // instead of building a generic quotient Computable first.
                        // The final multiply uses the cached exact 1/2 leaf so
                        // construction does not allocate a new rational after
                        // the symbolic reduction has already succeeded.
                        let one = Rational::one();
                        let ratio = (one.clone() + rational.clone()) / (one - rational);
                        crate::trace_dispatch!("computable", "atanh", "exact-log-ratio");
                        return Self::ln_exact_rational(ratio).multiply(Self::half());
                    }
                }
            }
        }
        if self.exact_sign() == Some(Sign::Minus) {
            crate::trace_dispatch!("computable", "atanh", "known-negative-symmetry");
            return self.negate().atanh().negate();
        }

        // General formula 1/2 * ln((1+x)/(1-x)). Tiny exact rationals avoid
        // this path because the odd atanh series has much less setup.
        crate::trace_dispatch!("computable", "atanh", "generic-log-ratio");
        let one = Self::one();
        let numerator = one.clone().add(self.clone());
        let denominator = one.add(self.negate());
        numerator
            .multiply(denominator.inverse())
            .ln()
            .multiply(Self::half())
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
        if let Approximation::Multiply(left, right) = self.internal.as_ref() {
            if let Some(scale) = left.exact_rational()
                && scale.sign() == Sign::Minus
            {
                crate::trace_dispatch!("computable", "negate", "exact-scale-fold");
                return right.clone().multiply_rational(scale.neg());
            }
            if let Some(scale) = right.exact_rational()
                && scale.sign() == Sign::Minus
            {
                crate::trace_dispatch!("computable", "negate", "exact-scale-fold");
                return left.clone().multiply_rational(scale.neg());
            }
        }
        let exact_sign = match *self.exact_sign.borrow() {
            // Preserve known exact signs for the cheap sign-first path in
            // predicates; this avoids a recursive sign walk on first query.
            ExactSignCache::Valid(sign) => ExactSignCache::Valid(negate_sign(sign)),
            _ => ExactSignCache::Invalid,
        };
        Self {
            internal: Box::new(Approximation::Negate(self)),
            cache: RefCell::new(Cache::Invalid),
            bound: RefCell::new(BoundCache::Invalid),
            exact_sign: RefCell::new(exact_sign),
            signal: None,
        }
    }

    /// Multiplicative inverse of this number.
    pub fn inverse(self) -> Computable {
        if self.shared_constant_kind() == Some(SharedConstant::Pi) {
            crate::trace_dispatch!("computable", "inverse", "shared-pi");
            return Self::pi_inverse_constant();
        }
        if self.shared_constant_kind() == Some(SharedConstant::InvPi) {
            crate::trace_dispatch!("computable", "inverse", "shared-inv-pi");
            return Self::pi();
        }
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
        if let Approximation::Multiply(left, right) = self.internal.as_ref() {
            if let Some(scale) = left.exact_rational()
                && let Ok(inverse_scale) = scale.inverse()
                && right.exact_sign().is_some_and(|sign| sign != Sign::NoSign)
            {
                // 1/(q*x) = (1/q)/x. Peeling the exact scale lets chains like
                // negate(inverse(x * -7/8)) collapse every other step instead
                // of building deep multiply/inverse/negate stacks.
                return right.clone().inverse().multiply_rational(inverse_scale);
            }
            if let Some(scale) = right.exact_rational()
                && let Ok(inverse_scale) = scale.inverse()
                && left.exact_sign().is_some_and(|sign| sign != Sign::NoSign)
            {
                return left.clone().inverse().multiply_rational(inverse_scale);
            }
        }
        if let Approximation::Inverse(child) = self.internal.as_ref()
            && child.exact_sign().is_some_and(|sign| sign != Sign::NoSign)
        {
            // Inverse of inverse collapses only when the inner value is
            // structurally nonzero.
            return child.clone();
        }
        if let Approximation::Square(child) = self.internal.as_ref()
            && child.exact_sign().is_some_and(|sign| sign != Sign::NoSign)
        {
            crate::trace_dispatch!("computable", "inverse", "square-of-inverse");
            return child.clone().inverse().square();
        }
        let exact_sign = match *self.exact_sign.borrow() {
            // Reciprocal preserves sign for structurally nonzero values and lets
            // sign queries remain structural through inverse chains.
            ExactSignCache::Valid(sign) if sign != Sign::NoSign => ExactSignCache::Valid(sign),
            _ => ExactSignCache::Invalid,
        };
        Self {
            internal: Box::new(Approximation::Inverse(self)),
            cache: RefCell::new(Cache::Invalid),
            bound: RefCell::new(BoundCache::Invalid),
            exact_sign: RefCell::new(exact_sign),
            signal: None,
        }
    }

    pub(crate) fn shift_left(self, n: i32) -> Self {
        if n == 0 {
            return self;
        }
        if let Approximation::Offset(child, inner) = self.internal.as_ref() {
            // Combine nested binary offsets rather than growing a chain of
            // no-op-ish wrappers.
            return child.clone().shift_left(inner + n);
        }
        // Exact sign is unchanged by binary scaling when the inner sign is
        // already proven; this makes compare/sign predicates avoid descending
        // into a one-step structural walk on hot paths.
        let exact_sign = match *self.exact_sign.borrow() {
            ExactSignCache::Valid(sign) => ExactSignCache::Valid(sign),
            _ => ExactSignCache::Invalid,
        };
        Self {
            internal: Box::new(Approximation::Offset(self, n)),
            cache: RefCell::new(Cache::Invalid),
            bound: RefCell::new(BoundCache::Invalid),
            exact_sign: RefCell::new(exact_sign),
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
        let exact_sign = match *self.exact_sign.borrow() {
            // Squared values are nonnegative when defined; structural zero is
            // preserved as exact zero.
            ExactSignCache::Valid(Sign::NoSign) => ExactSignCache::Valid(Sign::NoSign),
            ExactSignCache::Valid(Sign::Plus) | ExactSignCache::Valid(Sign::Minus) => {
                ExactSignCache::Valid(Sign::Plus)
            }
            _ => ExactSignCache::Invalid,
        };
        Self {
            internal: Box::new(Approximation::Square(self)),
            cache: RefCell::new(Cache::Invalid),
            bound: RefCell::new(BoundCache::Invalid),
            exact_sign: RefCell::new(exact_sign),
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
        let exact_sign = {
            let left_sign = left_exact.as_ref().map(Rational::sign).or_else(|| {
                match *self.exact_sign.borrow() {
                    ExactSignCache::Valid(sign) => Some(sign),
                    _ => None,
                }
            });
            let right_sign = right_exact.as_ref().map(Rational::sign).or_else(|| {
                match *other.exact_sign.borrow() {
                    ExactSignCache::Valid(sign) => Some(sign),
                    _ => None,
                }
            });
            match (left_sign, right_sign) {
                (Some(Sign::NoSign), Some(_)) | (Some(_), Some(Sign::NoSign)) => {
                    ExactSignCache::Valid(Sign::NoSign)
                }
                (Some(left), Some(right)) => ExactSignCache::Valid(if left == right {
                    Sign::Plus
                } else {
                    Sign::Minus
                }),
                _ => ExactSignCache::Invalid,
            }
        };
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
            exact_sign: RefCell::new(exact_sign),
            signal: None,
        }
    }

    pub(crate) fn multiply_rational(self, scale: Rational) -> Computable {
        if scale.sign() == Sign::NoSign {
            // Multiplying by zero drops the expression tree, including any
            // pending expensive approximation work.
            return Self::zero();
        }
        if let Some(value) = self.exact_rational() {
            // Exact symbolic leaves and exact-rational factors collapse directly.
            // This preserves cheap structural facts for chained scaling in
            // `fold_ref` and avoids building a new Multiply node.
            return Self::rational(value * scale);
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
        if let Approximation::Multiply(left, right) = &*self.internal {
            // Peel and combine exact rational factors from existing multiply
            // nodes so repeated scalar rebalances stay shallow.
            if let Some(inner_scale) = left.exact_rational() {
                return right.clone().multiply_rational(inner_scale.clone() * scale);
            }
            if let Some(inner_scale) = right.exact_rational() {
                return left.clone().multiply_rational(inner_scale.clone() * scale);
            }
        }
        let scale_sign = scale.sign();
        let exact_sign = match (*self.exact_sign.borrow(), scale_sign) {
            (ExactSignCache::Valid(Sign::NoSign), _) => ExactSignCache::Valid(Sign::NoSign),
            (ExactSignCache::Valid(Sign::Plus), Sign::Plus) => ExactSignCache::Valid(Sign::Plus),
            (ExactSignCache::Valid(Sign::Plus), Sign::Minus) => ExactSignCache::Valid(Sign::Minus),
            (ExactSignCache::Valid(Sign::Minus), Sign::Plus) => ExactSignCache::Valid(Sign::Minus),
            (ExactSignCache::Valid(Sign::Minus), Sign::Minus) => ExactSignCache::Valid(Sign::Plus),
            _ => ExactSignCache::Invalid,
        };
        Self {
            internal: Box::new(Approximation::Multiply(Self::rational(scale), self)),
            cache: RefCell::new(Cache::Invalid),
            bound: RefCell::new(BoundCache::Invalid),
            exact_sign: RefCell::new(exact_sign),
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
        let child_sign = {
            let left_sign = left_exact.as_ref().map(Rational::sign).or_else(|| {
                match *self.exact_sign.borrow() {
                    ExactSignCache::Valid(sign) => Some(sign),
                    _ => None,
                }
            });
            let right_sign = right_exact.as_ref().map(Rational::sign).or_else(|| {
                match *other.exact_sign.borrow() {
                    ExactSignCache::Valid(sign) => Some(sign),
                    _ => None,
                }
            });
            let (left_planning_sign, left_planning_msd) = self.planning_sign_and_msd();
            let (right_planning_sign, right_planning_msd) = other.planning_sign_and_msd();
            let left_planning_msd = left_planning_msd.flatten();
            let right_planning_msd = right_planning_msd.flatten();
            if let Some(sign) = match (left_sign, right_sign) {
                (Some(Sign::NoSign), Some(sign)) | (Some(sign), Some(Sign::NoSign)) => Some(sign),
                (Some(Sign::Plus), Some(Sign::Plus)) => Some(Sign::Plus),
                (Some(Sign::Minus), Some(Sign::Minus)) => Some(Sign::Minus),
                _ => None,
            } {
                Some(sign)
            } else if let (Some(left_sign), Some(right_sign), Some(left_msd), Some(right_msd)) = (
                left_planning_sign,
                right_planning_sign,
                left_planning_msd,
                right_planning_msd,
            ) {
                match (left_sign, right_sign) {
                    (Sign::Plus, Sign::Minus) if left_msd >= right_msd + 1 => Some(Sign::Plus),
                    (Sign::Minus, Sign::Plus) if right_msd >= left_msd + 1 => Some(Sign::Minus),
                    (Sign::Plus, Sign::Plus) => Some(Sign::Plus),
                    (Sign::Minus, Sign::Minus) => Some(Sign::Minus),
                    (Sign::NoSign, Sign::NoSign) => Some(Sign::NoSign),
                    _ => None,
                }
            } else {
                None
            }
        };
        let certified_sign = certified_bound.known_sign().or(child_sign);
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

    /// An approximation of this Computable scaled to a specific precision.
    ///
    /// Since the value is scaled, the approximation is roughly `value * 2^p`.
    /// Negative values of `p` request more precision.
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

        // Reserve a modest stack size for the flattened traversal path so long
        // chains of Negate/Add/Offset avoid repeated allocations.
        let mut frames = Vec::with_capacity(16);
        let mut values: Vec<BigInt> = Vec::with_capacity(8);
        frames.push(Frame::Eval(self, p));

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

        let mut sign = self.exact_sign().map(public_sign);
        #[cfg(feature = "dispatch-trace")]
        if sign.is_some() {
            crate::trace_dispatch!("computable", "structural_facts", "exact-sign-cache");
        }
        if sign.is_none()
            && let Some((_, appr)) = self.cached()
            && appr.abs() > BigInt::one()
        {
            crate::trace_dispatch!("computable", "structural_facts", "approximation-cache-sign");
            sign = Some(public_sign(appr.sign()));
        }

        let bound = self.cheap_bound();
        if sign.is_none() {
            let bound_sign = bound.known_sign();
            #[cfg(feature = "dispatch-trace")]
            if bound_sign.is_some() {
                crate::trace_dispatch!("computable", "structural_facts", "cheap-bound-sign");
            }
            sign = bound_sign.map(public_sign);
        }
        if sign.is_none() {
            let exact_bound_sign = exact
                .as_ref()
                .map(BoundInfo::from_rational)
                .as_ref()
                .and_then(BoundInfo::known_sign);
            #[cfg(feature = "dispatch-trace")]
            if exact_bound_sign.is_some() {
                crate::trace_dispatch!("computable", "structural_facts", "exact-rational-bound");
            }
            sign = exact_bound_sign.map(public_sign);
        }
        let exact_bound = if sign.is_none() {
            // Keep exact-rational bounds deferred until sign could not be proven by
            // cheaper structural facts. This avoids unnecessary conversion work.
            exact.as_ref().map(BoundInfo::from_rational)
        } else {
            None
        };

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
            crate::trace_dispatch!("computable", "zero_status", "exact-sign-cache");
            return if sign == Sign::NoSign {
                ZeroKnowledge::Zero
            } else {
                ZeroKnowledge::NonZero
            };
        }

        match self.cheap_bound() {
            BoundInfo::Zero => {
                crate::trace_dispatch!("computable", "zero_status", "cheap-bound-zero");
                ZeroKnowledge::Zero
            }
            BoundInfo::NonZero { .. } => {
                crate::trace_dispatch!("computable", "zero_status", "cheap-bound-nonzero");
                ZeroKnowledge::NonZero
            }
            BoundInfo::Unknown => {
                crate::trace_dispatch!("computable", "zero_status", "unknown");
                ZeroKnowledge::Unknown
            }
        }
    }

    /// Try to prove the sign without refining past `min_precision`.
    pub fn sign_until(&self, min_precision: Precision) -> Option<RealSign> {
        if let Some(sign) = self.exact_sign() {
            crate::trace_dispatch!("computable", "sign_until", "exact-sign-cache");
            return Some(public_sign(sign));
        }
        if let Some((_, appr)) = self.cached()
            && appr.abs() > BigInt::one()
        {
            let sign = appr.sign();
            self.exact_sign.replace(ExactSignCache::Valid(sign));
            crate::trace_dispatch!("computable", "sign_until", "approximation-cache-sign");
            return Some(public_sign(sign));
        }

        // Prefer structural facts before touching extra approximation work.
        // This keeps sign queries cheap when bounds are already strong enough.
        if let Some(sign) = self.cheap_bound().known_sign() {
            crate::trace_dispatch!("computable", "sign_until", "cheap-bound-sign");
            return Some(public_sign(sign));
        }

        crate::trace_dispatch!("computable", "sign_until", "precision-refinement");
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
            crate::trace_dispatch!("computable", "sign_until", "exact-rational-zero");
            Some(RealSign::Zero)
        } else {
            crate::trace_dispatch!("computable", "sign_until", "unknown");
            None
        }
    }

    /// Try to determine the exact sign, refining cached approximations as needed.
    pub fn sign(&self) -> Sign {
        if let Some(sign) = self.exact_sign() {
            crate::trace_dispatch!("computable", "sign", "exact-sign-cache");
            return sign;
        }
        {
            let cache = self.cache.borrow();
            if let Cache::Valid((_prec, cache_appr)) = &*cache {
                let sign = cache_appr.sign();
                if sign != Sign::NoSign {
                    self.exact_sign.replace(ExactSignCache::Valid(sign));
                    crate::trace_dispatch!("computable", "sign", "approximation-cache-sign");
                    return sign;
                }
            }
        }
        // Delay approximation refinement until after structural information has
        // had a chance to prove the sign. This avoids precision work for
        // purely symbolic queries.
        if let Some(sign) = self.cheap_bound().known_sign() {
            self.exact_sign.replace(ExactSignCache::Valid(sign));
            crate::trace_dispatch!("computable", "sign", "cheap-bound-sign");
            return sign;
        }
        crate::trace_dispatch!("computable", "sign", "precision-refinement");
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
        // Keep exact leaf comparisons allocation-free for the hot path where both
        // operands are already exact. This avoids creating temporary rationals
        // on every comparator call.
        if let Some(order) = self.exact_rational_leaf_cmp(other) {
            crate::trace_dispatch!("computable", "compare_to", "exact-rational");
            return order;
        }

        if let (Some(left), Some(right)) = (self.exact_rational(), other.exact_rational()) {
            // Exact rationals compare directly; escalating to approximate comparison here is
            // both slower and can burn cache precision unnecessarily.
            crate::trace_dispatch!("computable", "compare_to", "exact-rational");
            return left
                .partial_cmp(&right)
                .expect("exact rationals should be comparable");
        }

        if let (Some(left), Some(right)) = (self.exact_sign(), other.exact_sign()) {
            match (left, right) {
                (Sign::Minus, Sign::Plus | Sign::NoSign) | (Sign::NoSign, Sign::Plus) => {
                    crate::trace_dispatch!("computable", "compare_to", "exact-sign-opposite");
                    return Ordering::Less;
                }
                (Sign::Plus, Sign::Minus | Sign::NoSign) | (Sign::NoSign, Sign::Minus) => {
                    crate::trace_dispatch!("computable", "compare_to", "exact-sign-opposite");
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
                crate::trace_dispatch!("computable", "compare_to", "exact-sign-msd-gap");
                return match left {
                    Sign::Plus => left_msd.cmp(&right_msd),
                    Sign::Minus => right_msd.cmp(&left_msd),
                    Sign::NoSign => unreachable!(),
                };
            }
        }

        let self_bound = self.cheap_bound();
        let other_bound = other.cheap_bound();
        let self_bound_sign = self_bound.known_sign();
        let other_bound_sign = other_bound.known_sign();
        if let (Some(left), Some(right)) = (self_bound_sign, other_bound_sign) {
            match (left, right) {
                (Sign::Minus, Sign::Plus) | (Sign::NoSign, Sign::Plus) => {
                    crate::trace_dispatch!("computable", "compare_to", "cheap-bound-opposite-sign");
                    return Ordering::Less;
                }
                (Sign::Plus, Sign::Minus) | (Sign::Plus, Sign::NoSign) => {
                    crate::trace_dispatch!("computable", "compare_to", "cheap-bound-opposite-sign");
                    return Ordering::Greater;
                }
                (Sign::NoSign, Sign::NoSign) => return Ordering::Equal,
                _ => {}
            }
            if left == right
                && let (Some(Some(left_msd)), Some(Some(right_msd))) =
                    (self_bound.known_msd(), other_bound.known_msd())
                && left_msd != right_msd
            {
                // Same-sign structural bounds can decide exact ordering
                // before entering tolerance refinement.
                crate::trace_dispatch!("computable", "compare_to", "cheap-bound-msd-gap");
                return match left {
                    Sign::Plus => left_msd.cmp(&right_msd),
                    Sign::Minus => right_msd.cmp(&left_msd),
                    Sign::NoSign => Ordering::Equal,
                };
            }
        }
        crate::trace_dispatch!("computable", "compare_to", "approx-refinement");
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
        // Fast-path exact leafs before structural perturbation checks.
        if let Some(order) = self.exact_rational_leaf_cmp(other) {
            crate::trace_dispatch!("computable", "compare_absolute", "exact-rational");
            return order;
        }

        if let Approximation::Add(left, right) = &*self.internal {
            if let Some(order) = if Self::internal_structural_eq(left, other) {
                crate::trace_dispatch!(
                    "computable",
                    "compare_absolute",
                    "dominant-perturbation-self"
                );
                Self::compare_absolute_dominant_perturbation(left, right, other, tolerance)
            } else if Self::internal_structural_eq(right, other) {
                crate::trace_dispatch!(
                    "computable",
                    "compare_absolute",
                    "dominant-perturbation-self-reversed"
                );
                Self::compare_absolute_dominant_perturbation(right, left, other, tolerance)
            } else {
                None
            } {
                return order;
            }
        }
        if let Approximation::Add(left, right) = &*other.internal {
            if let Some(order) = if Self::internal_structural_eq(left, self) {
                crate::trace_dispatch!(
                    "computable",
                    "compare_absolute",
                    "dominant-perturbation-other"
                );
                Self::compare_absolute_dominant_perturbation(left, right, self, tolerance)
            } else if Self::internal_structural_eq(right, self) {
                crate::trace_dispatch!(
                    "computable",
                    "compare_absolute",
                    "dominant-perturbation-other-reversed"
                );
                Self::compare_absolute_dominant_perturbation(right, left, self, tolerance)
            } else {
                None
            } {
                return order.reverse();
            }
        }

        if let (Some(left), Some(right)) = (self.exact_rational(), other.exact_rational()) {
            // Compare exact-rational magnitudes without normalizing both operands.
            // This keeps the absolute-ordering branch allocation-light for symbolically
            // small values that are hit in compare-heavy workloads.
            crate::trace_dispatch!("computable", "compare_absolute", "exact-rational");
            return match (left.sign(), right.sign()) {
                (Sign::Minus, Sign::Minus) => right.compare_magnitude(&left),
                (Sign::Minus, Sign::Plus) => left.compare_magnitude(&right),
                (Sign::Plus, Sign::Minus) => left.compare_magnitude(&right),
                (Sign::Plus, Sign::Plus) => left.compare_magnitude(&right),
                (_, Sign::NoSign) => Ordering::Greater,
                (Sign::NoSign, _) => Ordering::Less,
            };
        }

        let self_sign = self.exact_sign();
        let other_sign = other.exact_sign();
        match (self_sign, other_sign) {
            // Exact signs can prove the nonzero ordering of absolute values.
            (Some(Sign::NoSign), Some(Sign::NoSign)) => return Ordering::Equal,
            (Some(Sign::NoSign), Some(_)) => return Ordering::Less,
            (Some(_), Some(Sign::NoSign)) => return Ordering::Greater,
            _ => {}
        }

        // Keep bound derivation lazy: only ask cheap_bound when exact sign facts
        // cannot already determine the ordering.
        if self_sign.is_none() || other_sign.is_none() {
            let self_bound = self.cheap_bound();
            let other_bound = other.cheap_bound();
            let self_structural_sign = self_sign.or(self_bound.known_sign());
            let other_structural_sign = other_sign.or(other_bound.known_sign());
            let self_msd = self_bound.known_msd();
            let other_msd = other_bound.known_msd();

            if let (BoundInfo::Zero, BoundInfo::Zero) = (&self_bound, &other_bound) {
                return Ordering::Equal;
            }
            match (self_structural_sign, other_structural_sign) {
                (Some(Sign::NoSign), Some(Sign::NoSign)) => return Ordering::Equal,
                (Some(Sign::NoSign), Some(_)) => return Ordering::Less,
                (Some(_), Some(Sign::NoSign)) => return Ordering::Greater,
                (Some(Sign::Minus), Some(Sign::Plus)) => return Ordering::Less,
                (Some(Sign::Plus), Some(Sign::Minus)) => return Ordering::Greater,
                _ => {}
            }
            if let (Some(left_sign), Some(right_sign), Some(left_msd), Some(right_msd)) = (
                self_structural_sign,
                other_structural_sign,
                self_msd,
                other_msd,
            ) {
                if left_msd != right_msd {
                    crate::trace_dispatch!("computable", "compare_absolute", "exact-sign-msd-gap");
                    match (left_sign, right_sign) {
                        (Sign::Plus, Sign::Plus) => return left_msd.cmp(&right_msd),
                        (Sign::Minus, Sign::Minus) => return right_msd.cmp(&left_msd),
                        _ => {}
                    }
                }
            }
            if let (Some(Some(left_msd)), Some(Some(right_msd))) = (self_msd, other_msd) {
                if left_msd > tolerance && right_msd < tolerance {
                    // Cheap MSD bounds can prove a tolerance-separated absolute ordering
                    // before allocating fresh approximations.
                    crate::trace_dispatch!(
                        "computable",
                        "compare_absolute",
                        "exact-sign-tolerance-gap"
                    );
                    return Ordering::Greater;
                }
                if right_msd > tolerance && left_msd < tolerance {
                    crate::trace_dispatch!(
                        "computable",
                        "compare_absolute",
                        "exact-sign-tolerance-gap"
                    );
                    return Ordering::Less;
                }
            }
        } else if self_sign == other_sign {
            let self_bound = self.cheap_bound();
            let other_bound = other.cheap_bound();
            if let (Some(Some(self_msd)), Some(Some(other_msd))) =
                (self_bound.known_msd(), other_bound.known_msd())
            {
                if let (Some(left_sign), Some(right_sign)) = (self_sign, other_sign) {
                    if left_sign == right_sign && self_msd != other_msd {
                        crate::trace_dispatch!(
                            "computable",
                            "compare_absolute",
                            "exact-sign-msd-gap"
                        );
                        return match (left_sign, right_sign) {
                            (Sign::Plus, Sign::Plus) => self_msd.cmp(&other_msd),
                            (Sign::Minus, Sign::Minus) => other_msd.cmp(&self_msd),
                            _ => Ordering::Equal,
                        };
                    }
                }
            }
        }
        crate::trace_dispatch!("computable", "compare_absolute", "approx-refinement");
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

    #[inline]
    fn exact_rational_leaf_cmp(&self, other: &Self) -> Option<Ordering> {
        match (&*self.internal, &*other.internal) {
            (Approximation::Ratio(left), Approximation::Ratio(right)) => left.partial_cmp(right),
            (Approximation::Int(left), Approximation::Int(right)) => Some(left.cmp(right)),
            (Approximation::One, Approximation::One) => Some(Ordering::Equal),
            (Approximation::One, Approximation::Int(right)) => Some(BigInt::one().cmp(right)),
            (Approximation::Int(left), Approximation::One) => Some(left.cmp(&BigInt::one())),
            _ => None,
        }
    }

    /// Most Significant Digit (Bit).
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
    pub(crate) fn msd(&self, p: Precision) -> Option<Precision> {
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
    pub(crate) fn iter_msd_stop(&self, p: Precision) -> Option<Precision> {
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

pub(crate) fn shift(n: BigInt, p: Precision) -> BigInt {
    match 0.cmp(&p) {
        Ordering::Greater => n >> -p,
        Ordering::Equal => n,
        Ordering::Less => n << p,
    }
}

/// Scale n by p bits, rounding if this makes n smaller.
/// e.g. scale(10, 2) == 40
///      scale(10, -2) == 3
pub(crate) fn scale(n: BigInt, p: Precision) -> BigInt {
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
    fn pi_cache_reuses_warmed_tau_cache() {
        std::thread::spawn(|| {
            let tau = Computable::tau();
            let _ = tau.approx(-65);
            assert!(Computable::pi().cached().is_none());

            let pi_appr = Computable::pi().approx(-64);
            let tau_scaled_as_pi = Computable::tau().approx(-63);
            assert_eq!(pi_appr, tau_scaled_as_pi);

            let cached = Computable::pi()
                .cached()
                .expect("pi cache should be filled from tau cache");
            assert_eq!(cached.0, -64);
            assert_eq!(cached.1, pi_appr);
        })
        .join()
        .expect("pi cache test thread should finish");
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
        let left = left.approx(p);
        let right = right.approx(p);
        let error = (&left - &right).abs();
        let max_error = BigInt::from(max_error);
        assert!(
            error <= max_error,
            "left {left}, right {right}, error {error}"
        );
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
    fn owned_rational_trig_helpers_match_generic_paths() {
        for rational in [
            Rational::fraction(-1, 5).unwrap(),
            Rational::fraction(1, 5).unwrap(),
            Rational::fraction(6, 5).unwrap(),
            Rational::fraction(7, 5).unwrap(),
            Rational::new(1_000_000),
        ] {
            let generic = Computable::rational(rational.clone());

            assert_close(
                Computable::sin_rational(rational.clone()),
                generic.clone().sin(),
                -80,
                8,
            );
            assert_close(
                Computable::cos_rational(rational.clone()),
                generic.clone().cos(),
                -80,
                8,
            );
            assert_close(Computable::tan_rational(rational), generic.tan(), -80, 16);
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

        let negative_scale = Rational::fraction(-7, 8).unwrap();
        let negative_value = Computable::pi().multiply(Computable::rational(negative_scale));
        let normalized = negative_value.inverse().negate();
        let expected = Computable::pi()
            .inverse()
            .multiply(Computable::rational(Rational::fraction(8, 7).unwrap()));
        assert_close(normalized, expected, -60, 2);
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
    fn inverse_of_square_of_nonzero_value_collapses_at_construction() {
        let base =
            Computable::pi().multiply(Computable::rational(Rational::fraction(7, 8).unwrap()));
        let value = base.clone().square().inverse();
        let expected = base.inverse().square();
        assert_close(value, expected, -60, 2);
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
    fn sqrt_squarefree_two_three_reuses_shared_constants() {
        let sqrt_twelve = Computable::rational(Rational::new(12)).sqrt();
        let expected = Computable::sqrt_constant(3)
            .unwrap()
            .multiply(Computable::rational(Rational::new(2)));
        assert_close(sqrt_twelve, expected, -60, 2);
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
        assert_close(value, Computable::pi(), -60, 2);
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
    fn exact_large_rational_trig_uses_correct_quadrant() {
        let million = Computable::rational(Rational::new(1_000_000));

        assert_approx(million.clone().sin(), -32, "-1503210646", 8);
        assert_approx(million.clone().cos(), -32, "4023319752", 8);
        assert_approx(million.tan(), -32, "-1604704811", 8);
    }

    #[test]
    fn exact_huge_rational_trig_uses_correct_quadrant() {
        let huge = Rational::new(10).powi(BigInt::from(30)).unwrap();
        let direct = Computable::rational(huge.clone());

        assert_approx(direct.clone().sin(), -72, "-425565037129932206620", 8);
        assert_approx(direct.clone().cos(), -72, "-4703152091704373381319", 8);
        assert_approx(direct.tan(), -72, "427303652622316740317", 16);
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
    fn ln_smooth_rational_reuses_shared_prime_logs() {
        let value = Computable::rational(Rational::fraction(45, 14).unwrap());
        let expected = Computable::ln_constant(3)
            .unwrap()
            .multiply(Computable::rational(Rational::new(2)))
            .add(Computable::ln_constant(5).unwrap())
            .add(Computable::ln_constant(2).unwrap().negate())
            .add(Computable::ln_constant(7).unwrap().negate());
        assert_close(value.ln(), expected, -50, 3);
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
