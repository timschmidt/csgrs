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
/// These facts are deliberately cheap structural certificates, not numerical
/// estimates. They serve the same role as inexpensive filters in exact
/// geometric computation: rule out impossible branches before spending work on
/// refinement. See Shewchuk, "Adaptive Precision Floating-Point Arithmetic and
/// Fast Robust Geometric Predicates", Discrete & Computational Geometry 1997,
/// and Yap, "Towards Exact Geometric Computation", Computational Geometry 1997.
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

/// Known comparison result for cheap structural threshold tests.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StructuralComparison {
    /// Structurally known to be less than the threshold.
    Less,
    /// Structurally known to be exactly equal to the threshold.
    Equal,
    /// Structurally known to be greater than the threshold.
    Greater,
    /// Not known without a more expensive query or approximation.
    Unknown,
}

/// Conservative domain status for common real-valued functions.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DomainStatus {
    /// The domain predicate is structurally known to hold.
    Valid,
    /// The domain predicate is structurally known to fail.
    Invalid,
    /// The domain predicate could not be decided cheaply.
    Unknown,
}

/// Coarse public category for the symbolic certificate carried by a `Real`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StructuralKind {
    ExactRational,
    PiLike,
    ExpLike,
    SqrtLike,
    LogLike,
    TrigExact,
    ProductConstant,
    ComputableOpaque,
}

/// Cheap exact classification for values that are commonly special-cased.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ZeroOneStatus {
    Zero,
    One,
    NeitherOrUnknown,
}

/// Coarse exact-rational storage cost bucket.
///
/// This is for planning only: callers can avoid repeating expensive exact
/// rational work in inner loops when a value is structurally large. The bucket
/// is derived from stored limb bit lengths and does not canonicalize or refine.
/// This mirrors the filter-first style of exact geometric computation; see Yap,
/// "Towards Exact Geometric Computation", Computational Geometry 1997.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RationalStorageClass {
    /// The exact rational is zero.
    Zero,
    /// Numerator and denominator each fit in a machine word.
    WordSized,
    /// Multi-limb, but not large enough to avoid exact structural dispatch.
    MultiLimb,
    /// Large enough that callers may prefer avoiding repeated rational work.
    VeryLarge,
}

/// Conservative primitive floating-point range classification.
///
/// These facts are intentionally conservative and opt-in. A previous broad
/// `to_f64_approx` preflight regressed dense symbolic conversions, so the facts
/// are exposed for callers that can amortize the query rather than being used
/// unconditionally in conversion hot paths.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PrimitiveFloatStatus {
    /// The value is structurally zero.
    Zero,
    /// The value is structurally in the normal finite range.
    NormalFinite,
    /// The value is structurally finite but may round through the subnormal path.
    SubnormalOrUnderflows,
    /// The value is structurally too large for a finite primitive float.
    Overflows,
    /// The range is not known without refinement or approximation.
    Unknown,
}

/// Primitive range facts for approximate fallback planning.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PrimitiveFacts {
    pub f32: PrimitiveFloatStatus,
    pub f64: PrimitiveFloatStatus,
}

/// Exact identity facts that are cheap enough to compute from stored fields.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct IdentityFacts {
    pub known_one: bool,
    pub known_minus_one: bool,
    pub zero_or_one: ZeroOneStatus,
}

/// Exact-rational facts derived without approximation.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RationalFacts {
    pub exact_integer: bool,
    pub exact_small_integer_i64: bool,
    pub exact_dyadic: bool,
    pub power_of_two: bool,
    pub storage: RationalStorageClass,
}

/// Cheap comparisons used by domain gates and filters.
///
/// These are exact field-derived comparisons for thresholds that appear in hot
/// kernels (`0`, `1`, and `|x| <= 1`). Keeping them explicit lets callers reject
/// invalid domains or take symbolic endpoints without canonicalizing or
/// approximating. This follows the exact-real design principle that algebraic
/// reduction and domain filtering should precede numerical refinement; see
/// Boehm, Cartwright, Riggle, and O'Donnell, "Exact Real Arithmetic: A Case
/// Study in Higher Order Programming", LISP and Functional Programming 1986.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct OrderingFacts {
    pub cmp_one: StructuralComparison,
    pub abs_cmp_one: StructuralComparison,
}

/// Domain facts for common unary functions.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct DomainFacts {
    pub sqrt: DomainStatus,
    pub log: DomainStatus,
    pub unit_interval_closed: DomainStatus,
    pub unit_interval_open: DomainStatus,
    pub acosh: DomainStatus,
}

/// Coarse facts about the symbolic certificate.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SymbolicFacts {
    pub kind: StructuralKind,
    pub has_sqrt_factor: bool,
    pub has_pi_factor: bool,
    pub has_exp_factor: bool,
    pub computable_required: bool,
}

/// Opt-in detailed facts for callers that can use richer structural dispatch.
///
/// This intentionally layers on top of [`RealStructuralFacts`] instead of
/// adding cost to the existing hot minimal query.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RealDetailedFacts {
    pub base: RealStructuralFacts,
    pub identity: IdentityFacts,
    pub rational: RationalFacts,
    pub primitive: PrimitiveFacts,
    pub ordering: OrderingFacts,
    pub domains: DomainFacts,
    pub symbolic: SymbolicFacts,
}
