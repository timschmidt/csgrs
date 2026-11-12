use core::fmt;

/// A failed interval operation. Errors never encode a successful empty set.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[non_exhaustive]
pub enum IntervalError {
    /// A primitive floating-point input is NaN or infinite.
    #[cfg(feature = "float-proposal")]
    NonFiniteEndpoint,
    /// The proposed lower endpoint is greater than the upper endpoint.
    ReversedEndpoints,
    /// A fixed-point scale is outside its endpoint family's supported range.
    InvalidScale,
    /// A radius or measurement uncertainty is negative.
    #[cfg(feature = "rational")]
    NegativeRadius,
    /// A monotone positive arithmetic helper received a nonpositive lower bound.
    #[cfg(feature = "rational")]
    NonPositiveLowerBound,
    /// An exact bridge could not supply certified point bounds.
    #[cfg(feature = "hyperreal")]
    EnclosureUnavailable,
    /// A purported exact dyadic endpoint is not on the required binary grid.
    #[cfg(all(feature = "rational", feature = "dyadic"))]
    NonDyadicEndpoint,
    /// An exact or outward-rounded endpoint is not representable.
    Overflow,
    /// Strict division cannot consume a denominator containing zero.
    ContainsZero,
    /// The input has no points in the real domain of the requested function.
    EmptyDomain,
    /// The input meets a pole requiring subdivision or an extended image.
    Pole,
    /// The requested branch has no implemented enclosure contract.
    UnsupportedBranch,
    /// The caller's precision budget was exhausted without the requested bound.
    PrecisionExhausted,
    /// The caller's total-work budget was exhausted before completion.
    WorkExhausted,
    /// A declared intermediate-size reservation exceeds its configured limit.
    #[cfg(feature = "dyadic")]
    SizeExhausted,
    /// The floating-point environment cannot support the certified backend.
    UnqualifiedEnvironment,
}

impl fmt::Display for IntervalError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            #[cfg(feature = "float-proposal")]
            Self::NonFiniteEndpoint => "interval endpoint is not finite",
            Self::ReversedEndpoints => "interval endpoints are reversed",
            Self::InvalidScale => "fixed-point scale is unsupported",
            #[cfg(feature = "rational")]
            Self::NegativeRadius => "interval radius is negative",
            #[cfg(feature = "rational")]
            Self::NonPositiveLowerBound => "interval lower bound must be positive",
            #[cfg(feature = "hyperreal")]
            Self::EnclosureUnavailable => "certified point enclosure unavailable",
            #[cfg(all(feature = "rational", feature = "dyadic"))]
            Self::NonDyadicEndpoint => "endpoint is not on the required dyadic grid",
            Self::Overflow => "interval endpoint overflow",
            Self::ContainsZero => "denominator interval contains zero",
            Self::EmptyDomain => "input has no points in the function domain",
            Self::Pole => "input meets a pole",
            Self::UnsupportedBranch => "interval branch is unsupported",
            Self::PrecisionExhausted => "interval precision budget exhausted",
            Self::WorkExhausted => "interval work budget exhausted",
            #[cfg(feature = "dyadic")]
            Self::SizeExhausted => "interval intermediate-size budget exhausted",
            Self::UnqualifiedEnvironment => "float environment is not qualified",
        })
    }
}

impl core::error::Error for IntervalError {}

/// Whether every source point belongs to the real domain of a function.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum DomainCoverage {
    /// No source point was excluded by the function's domain.
    Complete,
    /// The enclosure describes only the defined part of the source set.
    Partial,
}

/// Explicit rounding of a midpoint to an endpoint lattice.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum MidpointRounding {
    /// Round toward negative infinity.
    Floor,
    /// Round toward positive infinity.
    Ceiling,
    /// Choose the nearest lattice point, breaking half-unit ties to even.
    NearestTiesEven,
}

/// An image enclosure together with its source-domain qualification.
///
/// This payload does not itself certify an arbitrary user-supplied enclosure.
/// Consumers must retain the contract of the operation that produced it.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct RangeEnclosure<I> {
    /// Enclosure of the defined image.
    pub range: I,
    /// Coverage of the input by the function domain.
    pub domain: DomainCoverage,
}

/// Possible signs of members of a set; this is not a scalar predicate result.
///
/// All eight subsets of {negative, zero, positive} are represented. `Nonzero`
/// is useful for a split set. Connected intervals cannot have that sign set.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum SignSet {
    /// No members.
    Empty = 0,
    /// Only negative members.
    Negative = 1,
    /// Exactly zero.
    Zero = 2,
    /// Negative members and zero.
    NegativeOrZero = 3,
    /// Only positive members.
    Positive = 4,
    /// Negative and positive members, excluding zero.
    Nonzero = 5,
    /// Zero and positive members.
    ZeroOrPositive = 6,
    /// All three signs occur.
    Mixed = 7,
}

impl SignSet {
    /// Whether negative values may occur.
    pub const fn contains_negative(self) -> bool {
        self as u8 & 1 != 0
    }

    /// Whether zero may occur. This does not assert scalar equality to zero.
    pub const fn contains_zero(self) -> bool {
        self as u8 & 2 != 0
    }

    /// Whether positive values may occur.
    pub const fn contains_positive(self) -> bool {
        self as u8 & 4 != 0
    }

    /// Whether exactly one sign occurs (not whether the original set is a point).
    pub const fn is_singleton(self) -> bool {
        matches!(self, Self::Negative | Self::Zero | Self::Positive)
    }

    /// Union of possible signs, for example for a split reciprocal image.
    pub const fn union(self, other: Self) -> Self {
        match self as u8 | other as u8 {
            0 => Self::Empty,
            1 => Self::Negative,
            2 => Self::Zero,
            3 => Self::NegativeOrZero,
            4 => Self::Positive,
            5 => Self::Nonzero,
            6 => Self::ZeroOrPositive,
            _ => Self::Mixed,
        }
    }
}
