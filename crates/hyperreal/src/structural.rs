/// Exact sign knowledge exposed by structural inspection.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RealSign {
    /// The value is strictly less than zero.
    Negative,
    /// The value is exactly zero.
    Zero,
    /// The value is strictly greater than zero.
    Positive,
}

/// Whether structural inspection can prove zero or nonzero status.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ZeroKnowledge {
    /// The value is structurally known to be exactly zero.
    Zero,
    /// The value is structurally known not to be zero.
    NonZero,
    /// Structural inspection could not decide whether the value is zero.
    Unknown,
}

/// A known most-significant binary digit for a nonzero value.
///
/// `exact_msd` is true when `msd` is known exactly. When it is false, `msd`
/// is a conservative structural bound.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct MagnitudeBits {
    /// Most significant binary digit of the absolute value.
    pub msd: i32,
    /// Whether `msd` is exact rather than only a conservative bound.
    pub exact_msd: bool,
}

/// Conservative public facts about a real value.
///
/// Invariants:
///
/// - `zero = Zero` implies `sign = Some(RealSign::Zero)`.
/// - `zero = NonZero` means exact zero has been ruled out.
/// - `magnitude = Some(...)` is reported only for known nonzero values.
/// - `exact_rational` means an owned exact rational value can be obtained from
///   [`crate::Real::exact_rational`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RealStructuralFacts {
    /// Known sign, if structural inspection can prove it.
    pub sign: Option<RealSign>,
    /// Known zero/nonzero status.
    pub zero: ZeroKnowledge,
    /// Whether the value is exactly rational and cheaply recoverable.
    pub exact_rational: bool,
    /// Known magnitude information for nonzero values.
    pub magnitude: Option<MagnitudeBits>,
}
