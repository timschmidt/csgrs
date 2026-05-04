/// Exact sign knowledge exposed by structural inspection.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RealSign {
    Negative,
    Zero,
    Positive,
}

/// Whether structural inspection can prove zero or nonzero status.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ZeroKnowledge {
    Zero,
    NonZero,
    Unknown,
}

/// A known most-significant binary digit for a nonzero value.
///
/// `exact_msd` is true when `msd` is known exactly. When it is false, `msd`
/// is a conservative structural bound.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct MagnitudeBits {
    pub msd: i32,
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
    pub sign: Option<RealSign>,
    pub zero: ZeroKnowledge,
    pub exact_rational: bool,
    pub magnitude: Option<MagnitudeBits>,
}
