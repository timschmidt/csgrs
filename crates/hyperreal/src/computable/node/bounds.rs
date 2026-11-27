// The pinned verus_spec macro needs its method marker for associated functions.
// Conditional markers and proof attributes disappear from ordinary Rust builds.
#[cfg(verus_keep_ghost)]
use vstd::prelude::*;

#[cfg(verus_keep_ghost)]
/// Ghost-only result alias for specifications of checked metadata-exponent closures.
type MaybePrecision = Option<i32>;
#[cfg(verus_keep_ghost)]
verus! {
/// Model conversion of an unbounded mathematical exponent into the finite metadata word.
/// Out-of-range magnitudes lose their exponent field, not their separate sign/nonzero evidence.
pub open spec fn checked_metadata_exponent(value: int) -> Option<i32> {
    if i32::MIN <= value <= i32::MAX { Some(value as i32) } else { None }
}
}
/// Signed binary exponent used by approximation requests and magnitude metadata.
/// An approximation integer at p represents a value scaled by `2^-p`, with the
/// completed evaluation error contract measured in units of `2^p`; smaller p asks
/// for finer absolute accuracy. An MSD instead denotes `floor(log2(abs(x)))`.
/// The shared word type does not make those roles interchangeable or guarantee
/// that every i32 request is affordable or supported by every arithmetic path.
pub type Precision = i32;
/// Default outer sign-refinement floor for the convenience atan2 quadrant decision.
/// Failure to decide at this floor makes that wrapper panic; the fallible entry point
/// accepts a caller-selected floor. This is not a total-work or nested-precision bound.
const ATAN2_SIGN_REFINEMENT_FLOOR: Precision = -4096;
/// Default outer refinement floor for fallible exact comparison.
/// Structural proofs can decide below this scale without refinement; unresolved
/// comparison returns None. Nested evaluation work and precision are not capped here.
const DEFAULT_COMPARE_REFINEMENT_FLOOR: Precision = -4096;

#[derive(Clone, Copy, Debug, PartialEq, Default)]
#[cfg_attr(verus_keep_ghost, verus_verify)]
/// State of the structural bound slot, encoded with other facts in AtomicFacts.
/// A populated slot may still contain Unknown. These tags carry no node identity
/// or proof validation themselves; publication and strengthening belong to the cache owner.
pub(crate) enum BoundCache {
    /// No bound result has been installed; the default permits a structural walk.
    #[default]
    Invalid,
    /// A structural walk or separated approximation supplied metadata, possibly Unknown.
    Valid(
        /// Metadata for the owning immutable node, independent of the numerical approximation slot.
        BoundInfo,
    ),
}

#[derive(Clone, Copy, Debug, PartialEq, Default)]
#[cfg_attr(verus_keep_ghost, verus_verify)]
/// State of an exact sign query, distinct from both bound metadata and numerical approximations.
/// Unknown records an unresolved structural route, not zero or mathematical
/// undecidability. A later separated approximation or another proof may strengthen it.
pub(crate) enum ExactSignCache {
    /// No sign-query result has been installed; this is the default.
    #[default]
    Invalid,
    /// A structural query was attempted without certifying a sign.
    Unknown,
    /// A definite sign, including NoSign for proved zero, which unresolved publication must preserve.
    Valid(
        /// Certified sign of the owning node; independent valid proofs must agree.
        Sign,
    ),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(verus_keep_ghost, verus_verify)]
/// Conservative zero/sign evidence with an optional binary-magnitude certificate or planning hint.
/// For a nonzero x, an exact MSD e asserts `2^e <= abs(x) < 2^(e+1)`.
/// An inexact MSD is only a scheduling estimate: compositions can drift arbitrarily
/// far from the true magnitude, especially after cancellation and inversion.
/// Error-sensitive choices must use known_msd or refine independently.
///
/// The record is caller-trusted metadata, not an interval or a domain validator.
/// Constructors must supply true signs, use only Plus/Minus in NonZero, and justify
/// any exact exponent. Absent or overflowing exponents do not imply zero. Derived
/// equality compares these stored claims, not equality of the represented real values.
pub(crate) enum BoundInfo {
    /// Neither zero nor nonzero/sign is established; exact predicates must continue.
    Unknown,
    /// Exact zero, commonly inherited from a rational leaf or an annihilating product.
    Zero,
    /// Certified nonzero value, even if its sign or representable exponent is unavailable.
    NonZero {
        /// Known positive/negative sign; None withholds sign while retaining nonzero status.
        sign: Option<Sign>,
        /// Binary-magnitude exponent or planning estimate; None may mean outside i32.
        msd: Option<Precision>,
        /// Whether a present exponent is the exact binade; true with no exponent proves no magnitude.
        exact_msd: bool,
    },
}

#[cfg(verus_keep_ghost)]
verus! {
/// Ghost model of square metadata: keep nonzero positivity and double only an exact child exponent.
/// The resulting exponent is inexact; an overflow or already inexact child drops the estimate.
pub(crate) open spec fn squared_bound(bound: BoundInfo) -> BoundInfo {
    match bound {
        BoundInfo::Zero => BoundInfo::Zero,
        BoundInfo::NonZero { msd, exact_msd, .. } => BoundInfo::NonZero {
            sign: Some(Sign::Plus), exact_msd: false,
            msd: match (exact_msd, msd) {
                (true, Some(e)) => checked_metadata_exponent(2 * e as int),
                _ => None,
            },
        },
        BoundInfo::Unknown => BoundInfo::Unknown,
    }
}

/// Ghost model of product metadata with zero annihilation and independent sign multiplication.
/// Add two exact child exponents when representable, but mark the result as an estimate.
pub(crate) open spec fn product_bound(left: BoundInfo, right: BoundInfo) -> BoundInfo {
    match (left, right) {
        (BoundInfo::Zero, _) | (_, BoundInfo::Zero) => BoundInfo::Zero,
        (BoundInfo::NonZero { sign: ls, msd: lm, exact_msd: le },
         BoundInfo::NonZero { sign: rs, msd: rm, exact_msd: re }) => BoundInfo::NonZero {
            sign: match (ls, rs) {
                (Some(Sign::Plus), Some(Sign::Plus)) | (Some(Sign::Minus), Some(Sign::Minus)) => Some(Sign::Plus),
                (Some(Sign::Plus), Some(Sign::Minus)) | (Some(Sign::Minus), Some(Sign::Plus)) => Some(Sign::Minus),
                _ => None,
            },
            msd: match (le, re, lm, rm) {
                (true, true, Some(l), Some(r)) => checked_metadata_exponent(l as int + r as int),
                _ => None,
            },
            exact_msd: false,
        },
        _ => BoundInfo::Unknown,
    }
}

/// Ghost model of sum metadata: zero identity, equal-sign preservation and exact-binade dominance.
/// Opposite signs without distinct exact magnitudes stay unresolved. A distinct larger
/// stored exponent may survive as an inexact hint; it does not bound cancellation error.
pub(crate) open spec fn sum_bound(left: BoundInfo, right: BoundInfo) -> BoundInfo {
    match (left, right) {
        (BoundInfo::Zero, other) | (other, BoundInfo::Zero) => other,
        (BoundInfo::NonZero { sign: ls, msd: lm, exact_msd: le },
         BoundInfo::NonZero { sign: rs, msd: rm, exact_msd: re }) => {
            let sign = match (ls, rs) {
                (Some(l), Some(r)) if l == r => Some(l),
                (Some(Sign::Plus), Some(Sign::Minus)) | (Some(Sign::Minus), Some(Sign::Plus)) =>
                    if le && re { match (lm, rm) {
                        (Some(l), Some(r)) if l > r => ls,
                        (Some(l), Some(r)) if r > l => rs,
                        _ => None,
                    } } else { None },
                _ => None,
            };
            match sign {
                Some(s) => BoundInfo::NonZero {
                    sign: Some(s), exact_msd: false,
                    msd: match (lm, rm) {
                        (Some(l), Some(r)) if l > r => Some(l),
                        (Some(l), Some(r)) if r > l => Some(r),
                        _ => None,
                    },
                },
                None => BoundInfo::Unknown,
            }
        },
        _ => BoundInfo::Unknown,
    }
}
}

impl BoundInfo {
    #[cfg_attr(verus_keep_ghost, allow(unused, verus_impl_method_marker))]
    #[cfg_attr(verus_keep_ghost, verus_spec(result =>
        ensures match sign {
            Sign::NoSign => result == Self::Zero,
            _ => result == Self::NonZero { sign: Some(sign), msd, exact_msd: true },
        },
    ))]
    /// Construct metadata from a certified sign and an optional exact binade.
    /// NoSign yields Zero regardless of msd. For a nonzero sign, absent msd leaves
    /// nonzero/sign evidence intact; the caller must justify any exponent supplied.
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
    /// Package caller-established sign and magnitude claims with their exactness flag.
    /// NoSign becomes Zero; otherwise retain the fields without validating them against a value.
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
    /// Read an exact rational sign and, when representable, its exact binary magnitude.
    /// A missing exponent is not treated as zero. Rational sign/magnitude queries may
    /// perform normalization; this constructor provides no separate work budget.
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
    /// Transform only a present NonZero exponent, invoking f at most once.
    /// Preserve sign and exactness even if f returns None. The caller is responsible
    /// for a transformation that justifies retaining the existing exactness flag.
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
    /// Flip a known nonzero sign while retaining magnitude and its exactness.
    /// Zero, Unknown and unsigned NonZero metadata are unchanged.
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
    /// Retain nonzero sign and map any exponent e to the checked estimate `1-e`.
    /// The result is always inexact for NonZero. Zero and Unknown pass through: this
    /// is metadata transport for a caller-validated inverse, not admission of reciprocal zero.
    fn inverse(self) -> Self {
        match self {
            Self::NonZero { sign, msd, .. } => Self::NonZero {
                sign,
                msd: msd.and_then(
                    #[cfg_attr(verus_keep_ghost, verus_spec(result: MaybePrecision =>
                        ensures result == checked_metadata_exponent(1 - value as int),
                    ))]
                    |value| 1_i32.checked_sub(value),
                ),
                exact_msd: false,
            },
            other => other,
        }
    }

    #[cfg_attr(verus_keep_ghost, allow(unused, verus_impl_method_marker))]
    #[cfg_attr(verus_keep_ghost, verus_spec(result =>
        ensures result == squared_bound(self),
    ))]
    /// Map proved nonzero inputs to positive and preserve exact zero.
    /// An exact input binade e yields the checked lower-binade estimate `2*e`;
    /// the result is inexact and a subsequent square discards that estimate.
    /// Unknown stays Unknown, so this does not infer nonzero from the operation alone.
    fn square(self) -> Self {
        match self {
            Self::Zero => Self::Zero,
            Self::NonZero { msd, exact_msd, .. } => Self::NonZero {
                sign: Some(Sign::Plus),
                // One square can estimate the result within one binade from an
                // exact child MSD. Do not recursively double an already
                // inexact estimate: the error would grow exponentially through
                // a power tree.
                msd: match (exact_msd, msd) {
                    (true, Some(value)) => value.checked_mul(2),
                    _ => None,
                },
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
    /// Transport metadata for the nonnegative square root of a valid input.
    /// Keep zero; a known positive input maps exponent e to floor(e/2), preserving
    /// the exactness flag. Unknown, negative and unsigned nonzero inputs return
    /// Unknown; this method does not report a typed domain error for negative values.
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

    #[cfg_attr(verus_keep_ghost, allow(unused, verus_impl_method_marker))]
    #[cfg_attr(verus_keep_ghost, verus_spec(result =>
        ensures result == product_bound(self, other),
    ))]
    /// Combine metadata for a product, with zero annihilation before all other cases.
    /// Two nonzero inputs stay nonzero; known signs multiply. Only two exact input
    /// binades contribute a checked exponent sum, marked inexact because the true
    /// product binade may be one higher. Other combinations stay unresolved.
    /// Operand-domain validity must already hold before using the zero shortcut.
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
                    if left_exact_msd { left_msd } else { None },
                    if right_exact_msd { right_msd } else { None },
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

    #[cfg_attr(verus_keep_ghost, allow(unused, verus_impl_method_marker))]
    #[cfg_attr(verus_keep_ghost, verus_spec(result =>
        ensures result == sum_bound(self, other),
    ))]
    /// Combine metadata for a sum using zero identity and sign/binade dominance.
    /// Equal known signs retain that sign; opposite signs require distinct exact
    /// binades to select the larger-magnitude sign. Otherwise return Unknown.
    /// A distinct larger stored exponent is retained only as an inexact planning hint,
    /// which need not be close to the result after near cancellation.
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
                    (Some(Sign::Plus), Some(Sign::Plus))
                    | (Some(Sign::Minus), Some(Sign::Minus))
                    | (Some(Sign::NoSign), Some(Sign::NoSign)) => left_sign,
                    (Some(Sign::Plus), Some(Sign::Minus))
                    | (Some(Sign::Minus), Some(Sign::Plus)) => {
                        if left_exact_msd && right_exact_msd {
                            match (left_msd, right_msd) {
                                (Some(left), Some(right)) if left > right => left_sign,
                                (Some(left), Some(right)) if right > left => right_sign,
                                _ => None,
                            }
                        } else {
                            None
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
    /// Return a certified binade as Some(Some(e)), proved zero as Some(None), or None.
    /// None includes a nonzero value with a missing or merely estimated exponent.
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
    /// Expose any stored exponent, including an inexact scheduling estimate.
    /// Some(None) still means proved zero; outer None means no usable exponent.
    /// This projection alone cannot justify error-sensitive precision or range decisions.
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
    /// Project certified zero or the optional sign of a NonZero record.
    /// Unknown and unsigned nonzero metadata return None without implying zero.
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
    /// Project sign and exact-magnitude evidence independently, without exposing an inexact hint.
    /// Either component can be unresolved; Some(None) in the magnitude component proves zero.
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
    /// Export a present NonZero exponent together with its exactness flag.
    /// Zero, Unknown and missing exponents return None. Consumers must inspect
    /// the flag before treating the exported exponent as a binary-magnitude certificate.
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
    /// Return a positive sign and exact binade for each of the eighteen shared constants.
    /// These fixed mathematical bounds require no numerical approximation or cache
    /// initialization; they support structural facts and certified range planning.
    fn bound_info(self) -> BoundInfo {
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

    /// Return fixed strict rational bounds (lower, upper) for this positive constant.
    /// The hand-selected endpoints support constant-plus-rational certificates and
    /// mixed-sign native polynomial separation. They are independent of approximation
    /// caches and do not refine with requested precision. Fraction construction uses
    /// fixed nonzero denominators; the unwraps assert those literal invariants.
    fn interval(self) -> (Rational, Rational) {
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
/// Negate the three-way internal sign, leaving NoSign as exact zero.
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
/// Translate the internal three-way sign into the public RealSign vocabulary without losing zero.
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
/// Translate a public RealSign back to the internal three-way sign; inverse of public_sign.
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
/// Shared cooperative stop flag carried by computable wrappers and evaluation calls.
/// Cloning shares the flag; a true observation asks cooperating kernels to stop.
/// This is not a result/certainty receipt, cancellation deadline or ownership of
/// temporary state. Some symbolic or cached paths can finish without polling it.
pub type Signal = Arc<AtomicBool>;

#[cfg(not(verus_keep_ghost))]
/// Poll the optional shared stop flag with a relaxed atomic load; absence means false.
/// This reports only the observed Boolean, not completed cancellation or publication
/// of any other memory. Callers own polling frequency and interrupted-result handling.
pub(crate) fn should_stop(signal: &Option<Signal>) -> bool {
    use std::sync::atomic::Ordering::*;
    signal.as_ref().is_some_and(|s| s.load(Relaxed))
}

/// Process-wide approximation slots indexed by SharedConstant, shared across workers.
/// The array is initialized lazily; each EvaluationCache owns its synchronized
/// evaluation state. Reuse is precision-qualified, while per-node bound/sign facts
/// live separately. A fixed number of slots does not bound retained integer bytes.
#[cfg(not(verus_keep_ghost))]
static SHARED_CONSTANT_CACHES: LazyLock<[EvaluationCache; SharedConstant::COUNT]> =
    LazyLock::new(|| std::array::from_fn(|_| EvaluationCache::default()));
