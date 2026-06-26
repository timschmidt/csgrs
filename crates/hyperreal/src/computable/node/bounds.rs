pub type Precision = i32;
const ATAN2_SIGN_REFINEMENT_FLOOR: Precision = -4096;

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

fn private_sign(sign: RealSign) -> Sign {
    match sign {
        RealSign::Negative => Sign::Minus,
        RealSign::Zero => Sign::NoSign,
        RealSign::Positive => Sign::Plus,
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
