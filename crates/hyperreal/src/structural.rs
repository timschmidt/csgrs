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

/// Source of a certified [`RealSign`] decision.
///
/// The variants describe how a sign was proved, not how expensive the proof was
/// in wall-clock time. Keeping this source visible is important for exact
/// geometric computation: downstream predicate crates can distinguish cheap
/// object facts from bounded refinement without treating lossy primitive-float
/// approximations as topology. See Yap, "Towards Exact Geometric Computation,"
/// *Computational Geometry* 7.1-2 (1997), and Boehm et al., "Exact Real
/// Arithmetic: A Case Study in Higher Order Programming," *Proceedings of the
/// 1998 ACM SIGPLAN International Conference on Functional Programming*.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RealSignCertificate {
    /// The sign followed from cheap structural facts already carried by `Real`.
    StructuralFacts,
    /// The rational scale of the value is exactly zero.
    ExactZeroScale,
    /// A bounded exact-real refinement proved the sign.
    ///
    /// The `min_precision` value is the caller's refinement floor. It is not a
    /// primitive-float tolerance; it is the explicit precision bound for the
    /// exact-real approximation/refinement machinery.
    BoundedRefinement {
        /// Lowest binary precision the proof was allowed to request.
        min_precision: i32,
    },
}

/// Result of asking a `Real` for a certified sign under a bounded policy.
///
/// `Known` carries the sign and its proof source. `Unknown` means the requested
/// precision budget did not prove the sign; it does not authorize approximate
/// topology. This is the scalar-side counterpart of predicate reports in
/// `hyperlimit`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CertifiedRealSign {
    /// The sign was proved.
    Known {
        /// Proven sign.
        sign: RealSign,
        /// Proof route used to establish the sign.
        certificate: RealSignCertificate,
    },
    /// The requested bounded exact-real proof did not decide the sign.
    Unknown {
        /// Lowest binary precision the proof was allowed to request.
        min_precision: i32,
    },
}

impl CertifiedRealSign {
    /// Return the certified sign, if known.
    pub const fn sign(self) -> Option<RealSign> {
        match self {
            Self::Known { sign, .. } => Some(sign),
            Self::Unknown { .. } => None,
        }
    }

    /// Return whether the sign was proved.
    pub const fn is_known(self) -> bool {
        matches!(self, Self::Known { .. })
    }
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

/// Coarse bounded expression degree for structural dispatch.
///
/// This is not a CAS degree proof. It is a cheap certificate for currently
/// recognized scalar forms: exact rationals and named symbolic constants are
/// constant with respect to future solver variables, while opaque computable
/// nodes have unknown degree. The split creates the public boundary requested
/// by Yap's exact-computation model: preserve expression shape cheaply, then
/// let higher layers decide whether a solver or predicate can exploit it. See
/// Yap, "Towards Exact Geometric Computation," *Computational Geometry* 7.1-2
/// (1997).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ExpressionDegree {
    /// The value is structurally independent of future symbolic variables.
    Constant,
    /// The current scalar certificate cannot bound expression degree cheaply.
    Unknown,
}

/// Opaque bit mask of symbolic dependencies visible in a [`Real`](crate::Real).
///
/// The mask records dependency families, not object identities. It is intended
/// for low-cost scheduling and solver prechecks: for example, a residual block
/// can know it depends on `pi` and logarithms without learning `Real`'s private
/// expression representation. Future symbolic variable leaves can extend this
/// carrier without changing callers that only need family-level facts.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct SymbolicDependencyMask(u16);

impl SymbolicDependencyMask {
    /// Dependency on no symbolic family.
    pub const NONE: Self = Self(0);
    /// Dependency on `pi`.
    pub const PI: Self = Self(1 << 0);
    /// Dependency on `e` or `exp(rational)`.
    pub const EXP: Self = Self(1 << 1);
    /// Dependency on an explicit square-root factor.
    pub const SQRT: Self = Self(1 << 2);
    /// Dependency on logarithm forms.
    pub const LOG: Self = Self(1 << 3);
    /// Dependency on exact trigonometric special forms.
    pub const TRIG: Self = Self(1 << 4);
    /// Dependency on an opaque computable expression.
    pub const OPAQUE: Self = Self(1 << 15);

    /// Build a dependency mask from raw bits.
    pub const fn from_bits(bits: u16) -> Self {
        Self(bits)
    }

    /// Return the raw mask bits.
    pub const fn bits(self) -> u16 {
        self.0
    }

    /// Return whether this mask contains `dependency`.
    pub const fn contains(self, dependency: Self) -> bool {
        (self.0 & dependency.0) == dependency.0
    }

    /// Return whether no symbolic family is present.
    pub const fn is_empty(self) -> bool {
        self.0 == 0
    }

    /// Return the union of two masks.
    pub const fn union(self, other: Self) -> Self {
        Self(self.0 | other.0)
    }
}

/// Cheap exact classification for values that are commonly special-cased.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ZeroOneStatus {
    Zero,
    One,
    NeitherOrUnknown,
}

/// Cheap exact classification for signed unit and zero values.
///
/// Matrix, vector, and predicate kernels frequently branch on `0`, `1`, and
/// `-1` to select sparse, identity, or signed-permutation schedules. Keeping
/// the combined query in `hyperreal` lets higher crates consume one structural
/// fact without duplicating scalar representation checks. This is the
/// scalar-layer counterpart to Yap's recommendation that exact geometric
/// computation preserve inexpensive object facts before expanding arithmetic;
/// see Yap, "Towards Exact Geometric Computation," *Computational Geometry*
/// 7.1-2 (1997).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ZeroOneMinusOneStatus {
    /// The value is structurally known to be exactly zero.
    Zero,
    /// The value is structurally known to be exactly one.
    One,
    /// The value is structurally known to be exactly minus one.
    MinusOne,
    /// The value is neither known zero nor a signed unit, or cannot be decided
    /// without refinement.
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

/// Primitive range facts for named rendering, IO, diagnostics, or external
/// solver export planning.
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
    pub zero_one_or_minus_one: ZeroOneMinusOneStatus,
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
///
/// These are structural domain certificates, not evaluations of the functions.
/// They let higher layers reject invalid symbolic operations or select exact
/// simplifications before asking for numerical refinement. That is the scalar
/// side of Yap's exact-geometric-computation rule that filters must produce
/// certificates or explicit uncertainty; see Yap, "Towards Exact Geometric
/// Computation," *Computational Geometry* 7.1-2 (1997).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct DomainFacts {
    pub reciprocal: DomainStatus,
    pub sqrt: DomainStatus,
    pub log: DomainStatus,
    pub asin_acos: DomainStatus,
    pub unit_interval_closed: DomainStatus,
    pub unit_interval_open: DomainStatus,
    pub acosh: DomainStatus,
    pub atanh: DomainStatus,
}

/// Coarse facts about the symbolic certificate.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SymbolicFacts {
    /// Coarse family of the current scalar representation.
    pub kind: StructuralKind,
    /// Bounded degree certificate for recognized scalar forms.
    pub degree: ExpressionDegree,
    /// Family-level symbolic dependencies retained without exposing `Real` internals.
    pub dependencies: SymbolicDependencyMask,
    /// Whether the value has an explicit square-root factor.
    pub has_sqrt_factor: bool,
    /// Whether the value depends on `pi`.
    pub has_pi_factor: bool,
    /// Whether the value depends on `e` or an exponential factor.
    pub has_exp_factor: bool,
    /// Whether the value depends on a logarithm family.
    pub has_log_factor: bool,
    /// Whether the value depends on an exact trigonometric special form.
    pub has_trig_factor: bool,
    /// Whether exact evaluation requires an opaque computable node.
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
