// The pinned verus_spec macro needs its method marker for associated functions.
// Conditional markers and proof attributes disappear from ordinary Rust builds.
#[cfg(verus_keep_ghost)]
use vstd::prelude::*;

#[cfg(verus_keep_ghost)]
type MaybePrecision = Option<i32>;
#[cfg(verus_keep_ghost)]
verus! {
pub open spec fn checked_metadata_exponent(value: int) -> Option<i32> {
    if i32::MIN <= value <= i32::MAX { Some(value as i32) } else { None }
}
}
pub type Precision = i32;
const ATAN2_SIGN_REFINEMENT_FLOOR: Precision = -4096;
const DEFAULT_COMPARE_REFINEMENT_FLOOR: Precision = -4096;

#[derive(Clone, Copy, Debug, PartialEq, Default)]
#[cfg_attr(verus_keep_ghost, verus_verify)]
pub(crate) enum BoundCache {
    #[default]
    Invalid,
    Valid(BoundInfo),
}

#[derive(Clone, Copy, Debug, PartialEq, Default)]
#[cfg_attr(verus_keep_ghost, verus_verify)]
pub(crate) enum ExactSignCache {
    #[default]
    Invalid,
    Unknown,
    Valid(Sign),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(verus_keep_ghost, verus_verify)]
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
    #[cfg_attr(verus_keep_ghost, allow(unused, verus_impl_method_marker))]
    #[cfg_attr(verus_keep_ghost, verus_spec(result =>
        ensures match sign {
            Sign::NoSign => result == Self::Zero,
            _ => result == Self::NonZero { sign: Some(sign), msd, exact_msd: true },
        },
    ))]
    fn with_sign(sign: Sign, msd: Option<Precision>) -> Self {
        Self::with_sign_msd(sign, msd, true)
    }

    #[cfg_attr(verus_keep_ghost, allow(unused, verus_impl_method_marker))]
    #[cfg_attr(verus_keep_ghost, verus_spec(result =>
        ensures match sign {
            Sign::NoSign => result == Self::Zero,
            _ => result == Self::NonZero { sign: Some(sign), msd, exact_msd },
        },
    ))]
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

    #[cfg(not(verus_keep_ghost))]
    fn from_rational(r: &Rational) -> Self {
        // An absent exponent can mean a nonzero magnitude outside i32. Only
        // the rational's sign can certify that its value is zero.
        Self::with_sign(r.sign(), r.msd_exact())
    }

    #[cfg_attr(verus_keep_ghost, allow(unused, verus_impl_method_marker))]
    #[cfg_attr(verus_keep_ghost, verus_spec(result =>
        requires match self {
            Self::NonZero { msd: Some(value), .. } => call_requires(f, (value,)),
            _ => true,
        },
        ensures match self {
            Self::NonZero { sign, msd, exact_msd } => match result {
                Self::NonZero { sign: result_sign, msd: result_msd, exact_msd: result_exact } =>
                    result_sign == sign && result_exact == exact_msd
                    && match msd {
                        Some(value) => call_ensures(f, (value,), result_msd),
                        None => result_msd == None,
                    },
                _ => false,
            },
            _ => result == self,
        },
    ))]
    fn map_msd(self, f: impl FnOnce(Precision) -> Option<Precision>) -> Self {
        match self {
            Self::NonZero {
                sign,
                msd,
                exact_msd,
            } => Self::NonZero {
                sign,
                msd: msd.and_then(f),
                exact_msd,
            },
            other => other,
        }
    }

    #[cfg_attr(verus_keep_ghost, allow(unused, verus_impl_method_marker))]
    #[cfg_attr(verus_keep_ghost, verus_spec(result =>
        ensures result == match self {
            Self::NonZero { sign: Some(Sign::Plus), msd, exact_msd } =>
                Self::NonZero { sign: Some(Sign::Minus), msd, exact_msd },
            Self::NonZero { sign: Some(Sign::Minus), msd, exact_msd } =>
                Self::NonZero { sign: Some(Sign::Plus), msd, exact_msd },
            _ => self,
        },
    ))]
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

    #[cfg_attr(verus_keep_ghost, allow(unused, verus_impl_method_marker))]
    #[cfg_attr(verus_keep_ghost, verus_spec(result =>
        ensures result == match self {
            Self::NonZero { sign, msd, .. } => Self::NonZero {
                sign, exact_msd: false,
                msd: match msd {
                    Some(e) => checked_metadata_exponent(1 - e as int),
                    None => None,
                },
            },
            _ => self,
        },
    ))]
    fn inverse(self) -> Self {
        match self {
            Self::NonZero { sign, msd, .. } => Self::NonZero {
                sign,
                msd: msd.and_then(
                    #[cfg_attr(verus_keep_ghost, verus_spec(result: MaybePrecision =>
                        ensures result == checked_metadata_exponent(1 - value as int),
                    ))]
                    |value| 1_i32.checked_sub(value)
                ),
                exact_msd: false,
            },
            other => other,
        }
    }

    #[cfg(not(verus_keep_ghost))]
    fn square(self) -> Self {
        match self {
            Self::Zero => Self::Zero,
            Self::NonZero {
                msd, exact_msd, ..
            } => Self::NonZero {
                sign: Some(Sign::Plus),
                // One square can estimate the result within one binade from an
                // exact child MSD. Do not recursively double an already
                // inexact estimate: the error would grow exponentially through
                // a power tree.
                msd: exact_msd.then(|| msd.and_then(|value| value.checked_mul(2))).flatten(),
                exact_msd: false,
            },
            Self::Unknown => Self::Unknown,
        }
    }

    #[cfg_attr(verus_keep_ghost, allow(unused, verus_impl_method_marker))]
    #[cfg_attr(verus_keep_ghost, verus_spec(result =>
        ensures result == match self {
            Self::Zero => Self::Zero,
            Self::NonZero { sign: Some(Sign::Plus), msd, exact_msd } => Self::NonZero {
                sign: Some(Sign::Plus), exact_msd,
                msd: match msd {
                    Some(e) => Some((e as int / 2) as i32),
                    None => None,
                },
            },
            _ => Self::Unknown,
        },
    ))]
    fn sqrt(self) -> Self {
        match self {
            Self::Zero => Self::Zero,
            Self::NonZero {
                sign: Some(Sign::Plus),
                msd,
                exact_msd,
            } => Self::NonZero {
                sign: Some(Sign::Plus),
                // 2^k <= x < 2^(k+1) implies that sqrt(x)'s binade is
                // floor(k / 2), including negative odd exponents.
                msd: msd.map(crate::verified::word::floor_half),
                exact_msd,
            },
            _ => Self::Unknown,
        }
    }

    #[cfg(not(verus_keep_ghost))]
    fn multiply(self, other: Self) -> Self {
        match (self, other) {
            (Self::Zero, _) | (_, Self::Zero) => Self::Zero,
            (
                Self::NonZero {
                    sign: left_sign,
                    msd: left_msd,
                    exact_msd: left_exact_msd,
                },
                Self::NonZero {
                    sign: right_sign,
                    msd: right_msd,
                    exact_msd: right_exact_msd,
                },
            ) => {
                let sign = match (left_sign, right_sign) {
                    (Some(Sign::Plus), Some(Sign::Plus))
                    | (Some(Sign::Minus), Some(Sign::Minus)) => Some(Sign::Plus),
                    (Some(Sign::Plus), Some(Sign::Minus))
                    | (Some(Sign::Minus), Some(Sign::Plus)) => Some(Sign::Minus),
                    _ => None,
                };
                let msd = match (
                    left_exact_msd.then_some(left_msd).flatten(),
                    right_exact_msd.then_some(right_msd).flatten(),
                ) {
                    (Some(left), Some(right)) => left.checked_add(right),
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

    #[cfg(not(verus_keep_ghost))]
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
                    exact_msd: left_exact_msd,
                },
                Self::NonZero {
                    sign: right_sign,
                    msd: right_msd,
                    exact_msd: right_exact_msd,
                },
            ) => {
                let sign = match (left_sign, right_sign) {
                    (Some(left), Some(right)) if left == right => Some(left),
                    (Some(Sign::Plus), Some(Sign::Minus))
                    | (Some(Sign::Minus), Some(Sign::Plus))
                        if left_exact_msd && right_exact_msd =>
                    {
                        match (left_msd, right_msd) {
                            (Some(left), Some(right)) if left > right => left_sign,
                            (Some(left), Some(right)) if right > left => right_sign,
                            _ => None,
                        }
                    }
                    _ => None,
                };
                let msd = match (left_msd, right_msd) {
                    (Some(left), Some(right)) if left > right => Some(left),
                    (Some(left), Some(right)) if right > left => Some(right),
                    _ => None,
                };
                match sign {
                    Some(sign) => Self::NonZero {
                        sign: Some(sign),
                        msd,
                        exact_msd: false,
                    },
                    None => Self::Unknown,
                }
            }
            _ => Self::Unknown,
        }
    }

    #[cfg_attr(verus_keep_ghost, allow(unused, verus_impl_method_marker))]
    #[cfg_attr(verus_keep_ghost, verus_spec(result =>
        ensures (result == Some(None)) <==> *self == Self::Zero,
            result == match *self {
                Self::Zero => Some(None),
                Self::NonZero { msd: Some(msd), exact_msd: true, .. } => Some(Some(msd)),
                _ => None,
            },
    ))]
    fn known_msd(&self) -> Option<Option<Precision>> {
        // Some(None) certifies zero. A nonzero value without a representable
        // exponent must remain an unresolved magnitude query.
        match self {
            Self::Unknown => None,
            Self::Zero => Some(None),
            Self::NonZero {
                msd: Some(msd),
                exact_msd: true,
                ..
            } => Some(Some(*msd)),
            Self::NonZero { .. } => None,
        }
    }

    #[cfg_attr(verus_keep_ghost, allow(unused, verus_impl_method_marker))]
    #[cfg_attr(verus_keep_ghost, verus_spec(result =>
        ensures (result == Some(None)) <==> *self == Self::Zero,
            result == match *self {
                Self::Zero => Some(None),
                Self::NonZero { msd: Some(msd), .. } => Some(Some(msd)),
                _ => None,
            },
    ))]
    fn planning_msd(&self) -> Option<Option<Precision>> {
        match self {
            Self::Unknown => None,
            Self::Zero => Some(None),
            Self::NonZero { msd: Some(msd), .. } => Some(Some(*msd)),
            Self::NonZero { msd: None, .. } => None,
        }
    }

    #[cfg_attr(verus_keep_ghost, allow(unused, verus_impl_method_marker))]
    #[cfg_attr(verus_keep_ghost, verus_spec(result =>
        ensures result == match *self {
            Self::Zero => Some(Sign::NoSign),
            Self::NonZero { sign, .. } => sign,
            Self::Unknown => None,
        },
    ))]
    fn known_sign(&self) -> Option<Sign> {
        match self {
            Self::Zero => Some(Sign::NoSign),
            Self::NonZero { sign, .. } => *sign,
            Self::Unknown => None,
        }
    }

    #[cfg_attr(verus_keep_ghost, allow(unused, verus_impl_method_marker))]
    #[cfg_attr(verus_keep_ghost, verus_spec(result =>
        ensures result.0 == match *self {
            Self::Zero => Some(Sign::NoSign),
            Self::NonZero { sign, .. } => sign,
            Self::Unknown => None,
        },
        result.1 == match *self {
            Self::Zero => Some(None),
            Self::NonZero { msd: Some(msd), exact_msd: true, .. } => Some(Some(msd)),
            _ => None,
        },
    ))]
    fn certified_sign_and_msd(&self) -> (Option<Sign>, Option<Option<Precision>>) {
        (self.known_sign(), self.known_msd())
    }

    #[cfg_attr(verus_keep_ghost, allow(unused, verus_impl_method_marker))]
    #[cfg_attr(verus_keep_ghost, verus_spec(result =>
        ensures result == match *self {
            Self::NonZero { msd: Some(msd), exact_msd, .. } =>
                Some(MagnitudeBits { msd, exact_msd }),
            _ => None,
        },
    ))]
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

#[cfg(not(verus_keep_ghost))]
impl SharedConstant {
    fn bound_info(self) -> BoundInfo {
        // Coarse but exact-enough MSD facts for shared constants. These feed
        // structural queries and trig/ln reduction planning without forcing a
        // cached approximation.
        let msd = match self {
            SharedConstant::E | SharedConstant::Pi | SharedConstant::Ln10 => Some(1),
            SharedConstant::InvPi => Some(-2),
            SharedConstant::AtanInv2 => Some(-2),
            SharedConstant::AtanInv5 => Some(-3),
            SharedConstant::Tau => Some(2),
            SharedConstant::Ln2 => Some(-1),
            SharedConstant::Asinh1 => Some(-1),
            SharedConstant::AtanThreeHalves => Some(-1),
            SharedConstant::Ln3
            | SharedConstant::Ln5
            | SharedConstant::Ln6
            | SharedConstant::Ln7
            | SharedConstant::Sqrt2
            | SharedConstant::Sqrt3
            | SharedConstant::Acosh2
            | SharedConstant::Atan2 => Some(0),
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
            Self::AtanInv5 => (
                Rational::fraction(19, 100).unwrap(),
                Rational::fraction(1, 5).unwrap(),
            ),
            Self::AtanInv2 => (
                Rational::fraction(46, 100).unwrap(),
                Rational::fraction(47, 100).unwrap(),
            ),
            Self::Atan2 => (
                Rational::fraction(110, 100).unwrap(),
                Rational::fraction(111, 100).unwrap(),
            ),
            Self::AtanThreeHalves => (
                Rational::fraction(98, 100).unwrap(),
                Rational::fraction(99, 100).unwrap(),
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

#[cfg_attr(verus_keep_ghost, verus_spec(result =>
    ensures result == match sign { Sign::Plus => Sign::Minus, Sign::Minus => Sign::Plus, Sign::NoSign => Sign::NoSign },
))]
fn negate_sign(sign: Sign) -> Sign {
    match sign {
        Sign::Plus => Sign::Minus,
        Sign::Minus => Sign::Plus,
        Sign::NoSign => Sign::NoSign,
    }
}

#[cfg_attr(verus_keep_ghost, verus_spec(result =>
    ensures result == match sign {
        Sign::Minus => RealSign::Negative,
        Sign::NoSign => RealSign::Zero,
        Sign::Plus => RealSign::Positive,
    },
))]
fn public_sign(sign: Sign) -> RealSign {
    match sign {
        Sign::Minus => RealSign::Negative,
        Sign::NoSign => RealSign::Zero,
        Sign::Plus => RealSign::Positive,
    }
}

#[cfg_attr(verus_keep_ghost, verus_spec(result =>
    ensures result == match sign {
        RealSign::Negative => Sign::Minus,
        RealSign::Zero => Sign::NoSign,
        RealSign::Positive => Sign::Plus,
    },
))]
fn private_sign(sign: RealSign) -> Sign {
    match sign {
        RealSign::Negative => Sign::Minus,
        RealSign::Zero => Sign::NoSign,
        RealSign::Positive => Sign::Plus,
    }
}

#[cfg(not(verus_keep_ghost))]
use std::sync::atomic::AtomicBool;

#[cfg(not(verus_keep_ghost))]
pub type Signal = Arc<AtomicBool>;

#[cfg(not(verus_keep_ghost))]
pub(crate) fn should_stop(signal: &Option<Signal>) -> bool {
    use std::sync::atomic::Ordering::*;
    signal.as_ref().is_some_and(|s| s.load(Relaxed))
}

// Constants are value objects, so separate `Computable::pi()` calls are common.
// A process-wide lock-free cache lets every worker reuse the finest certified
// approximation instead of recomputing constants once per thread.
#[cfg(not(verus_keep_ghost))]
static SHARED_CONSTANT_CACHES: LazyLock<[ApproximationCache; SharedConstant::COUNT]> =
    LazyLock::new(|| std::array::from_fn(|_| ApproximationCache::default()));
