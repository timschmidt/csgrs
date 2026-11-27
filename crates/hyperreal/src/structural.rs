//! Public scalar fact snapshots, decision receipts and structural dispatch categories.
//!
//! Producers in Real, Computable and Rational establish the stated facts; these
//! copyable carriers do not validate caller-constructed fields or bind a report to
//! a source object. Derived equality compares report contents, including receipt
//! metadata, rather than represented scalar values. A snapshot does not update
//! when later queries learn stronger facts. Preserve the full decision variant
//! when proof route or requested precision matters; convenience projections omit it.
//!
//! Precision floors are query parameters, not total work/time/memory bounds or
//! the finest precision of previously cached evidence. Structural routes can reuse
//! certified cached facts; route tags are not complete proof transcripts. No raw
//! mask, boolean flag or manually assembled report is independent proof of geometry.

#[cfg(verus_keep_ghost)]
use vstd::prelude::*;

/// Known exact sign, shared by structural inspection and certified refinement reports.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(verus_keep_ghost, verus_verify)]
pub enum RealSign {
    /// The value is strictly less than zero.
    Negative,
    /// The value is exactly zero.
    Zero,
    /// The value is strictly greater than zero.
    Positive,
}

/// Exact zero/nonzero knowledge available to a structural query; Unknown is absence of a decision.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(verus_keep_ghost, verus_verify)]
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
/// approximations as topology.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(verus_keep_ghost, verus_verify)]
pub enum RealSignCertificate {
    /// The sign followed from cheap structural facts already carried by `Real`.
    StructuralFacts,
    /// The rational scale of the value is exactly zero.
    ExactZeroScale,
    /// The query reached the computable sign-decision route and obtained an exact sign.
    ///
    /// The `min_precision` value is the caller's refinement floor. It is not a
    /// primitive-float tolerance; it is the explicit precision bound for the
    /// exact-real approximation/refinement machinery. The route may reuse exact
    /// facts or certified filters without a fresh arbitrary-precision approximation;
    /// this tag does not count iterations or bound total work.
    BoundedRefinement {
        /// Requested binary refinement floor recorded by the query; cached evidence may be finer.
        min_precision: i32,
    },
}

/// Result of asking a `Real` for a certified sign under a bounded policy.
///
/// `Known` carries the sign and its proof source. `Unknown` means the requested
/// precision budget did not prove the sign; it does not authorize approximate
/// topology. This is the scalar-side counterpart of predicate reports in
/// `hyperlimit`. Unknown carries no separate failure/cancellation cause and is not
/// a permanent impossibility claim; a later query can have more evidence.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(verus_keep_ghost, verus_verify)]
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
        /// Requested binary refinement floor recorded by the query; cached evidence may be finer.
        min_precision: i32,
    },
}

/// Lossy convenience projections of a sign report; the copyable original retains its receipt.
impl CertifiedRealSign {
    /// Return the sign from Known, otherwise None; omit proof route and precision metadata.
    #[cfg_attr(verus_keep_ghost, allow(unused, verus_impl_method_marker))]
    #[cfg_attr(verus_keep_ghost, verus_spec(result =>
        ensures result == match self { Self::Known { sign, .. } => Some(sign), Self::Unknown { .. } => None },
    ))]
    pub const fn sign(self) -> Option<RealSign> {
        match self {
            Self::Known { sign, .. } => Some(sign),
            Self::Unknown { .. } => None,
        }
    }

    /// Test for Known without distinguishing negative, zero and positive results.
    #[cfg_attr(verus_keep_ghost, allow(unused, verus_impl_method_marker))]
    #[cfg_attr(verus_keep_ghost, verus_spec(result =>
        ensures result == matches!(self, Self::Known { .. }),
    ))]
    pub const fn is_known(self) -> bool {
        matches!(self, Self::Known { .. })
    }
}

/// Source of a certified equality or inequality decision between two `Real`
/// values.
///
/// The variants describe proof routes, not tolerance policies. Equality is
/// certified either by representation identity, exact rational comparison, or
/// by proving that the difference is exactly zero. Inequality is certified by
/// exact structural facts or by proving a nonzero sign for the difference.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(verus_keep_ghost, verus_verify)]
pub enum RealEqualityCertificate {
    /// The existing structural `PartialEq` relation proved equality.
    StructuralEquality,
    /// Both values were exactly rational and compared as rationals.
    ExactRationalComparison,
    /// Cheap structural facts about the operands proved the result.
    StructuralFacts,
    /// Cheap structural facts about `left - right` proved the result.
    DifferenceStructuralFacts,
    /// Bounded exact-real refinement of `left - right` proved the result.
    BoundedRefinement {
        /// Requested binary refinement floor recorded by the query; cached evidence may be finer.
        min_precision: i32,
    },
}

/// Source of a certified ordering decision between two `Real` values.
///
/// The certificate describes how the sign of `left - right` was proved.
/// `PartialOrd` uses this same scalar predicate and returns `None` when its
/// bounded exact-real proof budget cannot certify an ordering.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(verus_keep_ghost, verus_verify)]
pub enum RealOrderingCertificate {
    /// The operands were structurally equal.
    StructuralEquality,
    /// Both operands were exact rationals and compared as rationals.
    ExactRationalComparison,
    /// Cheap structural sign or magnitude facts about the operands proved the ordering.
    StructuralFacts,
    /// Cheap structural facts about `left - right` proved the ordering.
    DifferenceStructuralFacts,
    /// Bounded exact-real refinement of `left - right` proved the ordering.
    BoundedRefinement {
        /// Requested binary refinement floor recorded by the query; cached evidence may be finer.
        min_precision: i32,
    },
}

/// Result of asking for a certified ordering under a bounded exact-real policy.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(verus_keep_ghost, verus_verify)]
pub enum CertifiedRealOrdering {
    /// Ordering was proved.
    Known {
        /// Proven ordering.
        ordering: core::cmp::Ordering,
        /// Proof route used to establish the ordering.
        certificate: RealOrderingCertificate,
    },
    /// The requested bounded exact-real proof did not decide the ordering.
    Unknown {
        /// Requested binary refinement floor recorded by the query; cached evidence may be finer.
        min_precision: i32,
    },
}

/// Project an ordering report while leaving certificate-sensitive callers free to match its variant.
impl CertifiedRealOrdering {
    /// Return Known ordering or None for Unknown, discarding the proof route from this projection.
    #[cfg_attr(verus_keep_ghost, allow(unused, verus_impl_method_marker))]
    #[cfg_attr(verus_keep_ghost, verus_spec(result =>
        ensures result == match self { Self::Known { ordering, .. } => Some(ordering), Self::Unknown { .. } => None },
    ))]
    pub const fn ordering(self) -> Option<core::cmp::Ordering> {
        match self {
            Self::Known { ordering, .. } => Some(ordering),
            Self::Unknown { .. } => None,
        }
    }

    /// Test whether the report holds any proved ordering, including equality.
    #[cfg_attr(verus_keep_ghost, allow(unused, verus_impl_method_marker))]
    #[cfg_attr(verus_keep_ghost, verus_spec(result =>
        ensures result == matches!(self, Self::Known { .. }),
    ))]
    pub const fn is_known(self) -> bool {
        matches!(self, Self::Known { .. })
    }
}

/// Result of asking whether two `Real` values are mathematically equal under a
/// bounded exact-real policy.
///
/// `Equal` and `NotEqual` are certified decisions. `Unknown` means the requested
/// precision budget did not prove the equality status; it must not be treated
/// as approximate inequality.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(verus_keep_ghost, verus_verify)]
pub enum CertifiedRealEquality {
    /// Equality was proved.
    Equal {
        /// Proof route used to establish equality.
        certificate: RealEqualityCertificate,
    },
    /// Inequality was proved.
    NotEqual {
        /// Proof route used to establish inequality.
        certificate: RealEqualityCertificate,
    },
    /// The requested bounded exact-real proof did not decide equality.
    Unknown {
        /// Requested binary refinement floor recorded by the query; cached evidence may be finer.
        min_precision: i32,
    },
}

/// Project equality reports without mapping an undecided status to false.
impl CertifiedRealEquality {
    /// Map Equal/NotEqual to Some(true)/Some(false), and Unknown to None; omit its receipt.
    #[cfg_attr(verus_keep_ghost, allow(unused, verus_impl_method_marker))]
    #[cfg_attr(verus_keep_ghost, verus_spec(result =>
        ensures result == match self { Self::Equal { .. } => Some(true), Self::NotEqual { .. } => Some(false), Self::Unknown { .. } => None },
    ))]
    pub const fn as_bool(self) -> Option<bool> {
        match self {
            Self::Equal { .. } => Some(true),
            Self::NotEqual { .. } => Some(false),
            Self::Unknown { .. } => None,
        }
    }

    /// Return true for either certified outcome and false only for Unknown.
    #[cfg_attr(verus_keep_ghost, allow(unused, verus_impl_method_marker))]
    #[cfg_attr(verus_keep_ghost, verus_spec(result =>
        ensures result == !matches!(self, Self::Unknown { .. }),
    ))]
    pub const fn is_known(self) -> bool {
        !matches!(self, Self::Unknown { .. })
    }
}

/// Selected graph-size and coefficient-width diagnostics for an exact representation.
///
/// This measures growth across repeated constructions. It is not a canonical
/// form or a cost bound: equal values can have different representations.
/// Shared expression nodes count once; named classes exclude materialized payloads.
/// Rational widths use canonical parts and can trigger lazy reduction/cache population.
/// This is not raw retained-byte accounting. Reported by
/// [`Real::representation_size`](crate::Real::representation_size).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RepresentationSize {
    /// Distinct expression nodes. An exact rational has none; a recognized
    /// closed form such as `sqrt(2)` or `sin(pi/16)` is one node.
    pub nodes: usize,
    /// Nodes on the longest path from the value to a leaf.
    pub depth: usize,
    /// Maximum canonical numerator-plus-denominator bit length of counted rational payloads.
    /// Selected integer payloads count magnitude bits. Small tags, allocation overhead
    /// and cached approximations are excluded; this is not a sum or raw storage width.
    pub max_rational_bits: u64,
    /// False when the node limit stopped the traversal. The other fields are
    /// then lower bounds.
    pub complete: bool,
}

/// A known most-significant binary digit for a nonzero value.
///
/// `exact_msd` is true when `msd` is known exactly. When it is false, `msd`
/// is an inexact structural magnitude descriptor, not a proved binade. Consumers
/// that compare exact exponents or classify primitive exponent bands must check
/// exact_msd first. The descriptor alone supplies no two-sided interval.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(verus_keep_ghost, verus_verify)]
pub struct MagnitudeBits {
    /// Binary exponent: when exact, floor(log2(abs(value))); zero has no such exponent.
    pub msd: i32,
    /// Whether `msd` is exact rather than only a conservative bound.
    pub exact_msd: bool,
}

/// Conservative public facts about a real value.
///
/// These facts are deliberately cheap structural certificates, not numerical
/// estimates. They may reuse previously certified cached evidence and do not
/// promise constant cost or absence of cache updates. They serve the same role as inexpensive filters in exact
/// geometric computation: rule out impossible branches before spending work on
/// refinement.
///
/// Invariants for reports returned by the scalar producers (public fields do not
/// enforce these on manually constructed snapshots):
///
/// - `zero = Zero` implies `sign = Some(RealSign::Zero)`.
/// - `zero = NonZero` means exact zero has been ruled out.
/// - `magnitude = Some(...)` is reported only for known nonzero values.
/// - `exact_rational` means an owned exact rational value can be obtained from
///   [`crate::Real::exact_rational`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(verus_keep_ghost, verus_verify)]
pub struct RealStructuralFacts {
    /// Known sign, if structural inspection can prove it.
    pub sign: Option<RealSign>,
    /// Known zero/nonzero status.
    pub zero: ZeroKnowledge,
    /// Whether the producer exposes an exact rational representation directly.
    /// False does not prove irrationality; recovery may canonicalize lazy rational storage.
    pub exact_rational: bool,
    /// Known magnitude information for nonzero values.
    pub magnitude: Option<MagnitudeBits>,
}

/// Known comparison result for cheap structural threshold tests.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(verus_keep_ghost, verus_verify)]
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
#[cfg_attr(verus_keep_ghost, verus_verify)]
pub enum DomainStatus {
    /// The domain predicate is structurally known to hold.
    Valid,
    /// The domain predicate is structurally known to fail.
    Invalid,
    /// The domain predicate could not be decided cheaply.
    Unknown,
}

/// Coarse public category for the symbolic certificate carried by a `Real`.
/// This classifies retained representation rather than a unique mathematical
/// family. Equivalent values, or zero scales on different bases, can have
/// different categories; ComputableOpaque does not imply irrationality.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(verus_keep_ghost, verus_verify)]
pub enum StructuralKind {
    /// Unit basis with an exact rational coefficient.
    ExactRational,
    /// Pi, an integer pi power, or reciprocal pi basis.
    PiLike,
    /// An exponential/power basis, including pi times or divided into an exponential.
    ExpLike,
    /// A rational square-root basis, optionally multiplied by pi.
    SqrtLike,
    /// A logarithmic, affine-logarithmic or logarithmic-product certificate.
    LogLike,
    /// A retained exact sine/tangent pi-multiple certificate, not a sampled trig value.
    TrigExact,
    /// A retained pi/exponential product, additive offset or product-root certificate.
    ProductConstant,
    /// A graph whose structure is not exposed by one of the named classes.
    ComputableOpaque,
}

/// Coarse bounded expression degree for structural dispatch.
///
/// This is not a CAS degree proof. It is a cheap certificate for currently
/// recognized scalar forms: exact rationals and named symbolic constants are
/// constant with respect to future solver variables, while opaque computable
/// nodes have unknown degree. The split creates the public boundary requested
/// by the exact-computation model: preserve expression shape cheaply, then let
/// higher layers decide whether a solver or predicate can exploit it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(verus_keep_ghost, verus_verify)]
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
/// This describes retained basis families, not minimal semantic dependencies or
/// variable identity. Zero scales and further identities can remove mathematical
/// dependence without clearing the class's bits. Default is the empty mask;
/// unknown raw bits are retained rather than rejected.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(verus_keep_ghost, verus_verify)]
pub struct SymbolicDependencyMask(
    /// Raw family bitset, including unnamed bits preserved by from_bits and union.
    u16,
);

/// Const bit operations preserving every bit, including unnamed future family bits.
impl SymbolicDependencyMask {
    /// Empty mask; every mask contains it under the subset test.
    #[cfg_attr(verus_keep_ghost, verus_spec(
        ensures Self::NONE.model_bits() == 0,
    ))]
    pub const NONE: Self = Self(0);
    /// Dependency on `pi`.
    #[cfg_attr(verus_keep_ghost, verus_spec(
        ensures Self::PI.model_bits() == (1u16 << 0u32),
    ))]
    pub const PI: Self = Self(1 << 0);
    /// Dependency on `e` or `exp(rational)`.
    #[cfg_attr(verus_keep_ghost, verus_spec(
        ensures Self::EXP.model_bits() == (1u16 << 1u32),
    ))]
    pub const EXP: Self = Self(1 << 1);
    /// Dependency on an explicit square-root factor.
    #[cfg_attr(verus_keep_ghost, verus_spec(
        ensures Self::SQRT.model_bits() == (1u16 << 2u32),
    ))]
    pub const SQRT: Self = Self(1 << 2);
    /// Dependency on logarithm forms.
    #[cfg_attr(verus_keep_ghost, verus_spec(
        ensures Self::LOG.model_bits() == (1u16 << 3u32),
    ))]
    pub const LOG: Self = Self(1 << 3);
    /// Dependency on exact trigonometric special forms.
    #[cfg_attr(verus_keep_ghost, verus_spec(
        ensures Self::TRIG.model_bits() == (1u16 << 4u32),
    ))]
    pub const TRIG: Self = Self(1 << 4);
    /// Dependency on an opaque computable expression.
    #[cfg_attr(verus_keep_ghost, verus_spec(
        ensures Self::OPAQUE.model_bits() == (1u16 << 15u32),
    ))]
    pub const OPAQUE: Self = Self(1 << 15);

    /// Preserve all sixteen supplied bits without validating or clearing unnamed bits.
    #[cfg_attr(verus_keep_ghost, allow(unused, verus_impl_method_marker))]
    #[cfg_attr(verus_keep_ghost, verus_spec(result =>
        ensures result.model_bits() == bits,
    ))]
    pub const fn from_bits(bits: u16) -> Self {
        Self(bits)
    }

    /// Return every stored bit, so from_bits(bits).bits() is an exact round trip.
    #[cfg_attr(verus_keep_ghost, allow(unused, verus_impl_method_marker))]
    #[cfg_attr(verus_keep_ghost, verus_spec(result =>
        ensures result == self.model_bits(),
    ))]
    pub const fn bits(self) -> u16 {
        self.0
    }

    /// Test that all bits in dependency are present, including unnamed bits; NONE is always contained.
    #[cfg_attr(verus_keep_ghost, allow(unused, verus_impl_method_marker))]
    #[cfg_attr(verus_keep_ghost, verus_spec(result =>
        ensures result == ((self.model_bits() & dependency.model_bits()) == dependency.model_bits()),
    ))]
    pub const fn contains(self, dependency: Self) -> bool {
        (self.0 & dependency.0) == dependency.0
    }

    /// Test for zero raw bits; an unnamed nonzero bit also makes this false.
    #[cfg_attr(verus_keep_ghost, allow(unused, verus_impl_method_marker))]
    #[cfg_attr(verus_keep_ghost, verus_spec(result =>
        ensures result == (self.model_bits() == 0),
    ))]
    pub const fn is_empty(self) -> bool {
        self.0 == 0
    }

    /// Combine all raw bits with bitwise OR, without semantic reduction or validation.
    #[cfg_attr(verus_keep_ghost, allow(unused, verus_impl_method_marker))]
    #[cfg_attr(verus_keep_ghost, verus_spec(result =>
        ensures result.model_bits() == (self.model_bits() | other.model_bits()),
    ))]
    pub const fn union(self, other: Self) -> Self {
        Self(self.0 | other.0)
    }
}

/// Cheap exact classification for values that are commonly special-cased.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(verus_keep_ghost, verus_verify)]
pub enum ZeroOneStatus {
    /// The value is proved zero by the producer's structural path.
    Zero,
    /// The value is recognized as exact one.
    One,
    /// Neither identity was recognized; this combines known-other and unresolved cases.
    NeitherOrUnknown,
}

/// Cheap exact classification for signed unit and zero values.
///
/// Matrix, vector, and predicate kernels frequently branch on `0`, `1`, and
/// `-1` to select sparse, identity, or signed-permutation schedules. Keeping
/// the combined query in `hyperreal` lets higher crates consume one structural
/// fact without duplicating scalar representation checks. This is the
/// scalar-layer counterpart to preserving inexpensive object facts before
/// expanding arithmetic.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(verus_keep_ghost, verus_verify)]
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
/// uses numerator/denominator bit lengths, not allocated capacity or total bytes.
/// Rational::storage_class reads immutable stored parts without canonicalizing;
/// the storage field in detailed RationalFacts instead uses canonical parts and
/// can trigger lazy reduction. Those queries can return different buckets for
/// the same value. Thresholds are fixed at 64 bits per part and 4096 combined
/// bits, independent of the target's pointer width.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(verus_keep_ghost, verus_verify)]
pub enum RationalStorageClass {
    /// The exact rational is zero.
    Zero,
    /// Nonzero rational whose numerator and denominator each use at most 64 bits.
    WordSized,
    /// Nonzero rational outside WordSized with combined part widths at most 4096 bits.
    MultiLimb,
    /// Larger combined part widths, or a conservative placeholder when no rational view is exposed.
    VeryLarge,
}

/// Primitive floating-point exponent-range categories, not a complete conversion certificate.
///
/// The current classifier uses exact magnitude exponents only. NormalFinite
/// includes the entire upper normal binade, where some significands round to
/// infinity; it must not be used alone to certify finite conversion. Subnormal
/// classification likewise does not decide rounding to zero. These categories
/// are opt-in. A previous broad
/// `to_f64_lossy` preflight regressed dense symbolic conversions, so the facts
/// are exposed for callers that can amortize the query rather than being used
/// unconditionally in conversion hot paths.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(verus_keep_ghost, verus_verify)]
pub enum PrimitiveFloatStatus {
    /// The value is structurally zero.
    Zero,
    /// The magnitude exponent lies in the normal exponent band, including its upper binade.
    /// This label does not establish that the rounded primitive value is finite.
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
#[cfg_attr(verus_keep_ghost, verus_verify)]
pub struct PrimitiveFacts {
    /// Binary32 exponent category; NormalFinite alone is not a finite-rounding guarantee.
    pub f32: PrimitiveFloatStatus,
    /// Binary64 exponent category with the same upper-binade limitation.
    pub f64: PrimitiveFloatStatus,
}

/// Positive identity evidence from the producer's structural queries.
/// False flags mean no recognized identity, not a general proof of inequality;
/// rational identity queries can canonicalize lazy internal ratios.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(verus_keep_ghost, verus_verify)]
pub struct IdentityFacts {
    /// True when the producer recognizes an exact unit-basis one.
    pub known_one: bool,
    /// True when the producer recognizes an exact unit-basis minus one.
    pub known_minus_one: bool,
    /// Combined zero/unit recognition, retaining a neither-or-unknown category.
    pub zero_or_one: ZeroOneStatus,
    /// Combined zero/signed-unit recognition for sparse and signed-permutation dispatch.
    pub zero_one_or_minus_one: ZeroOneMinusOneStatus,
}

/// Positive exact-rational facts derived without numerical approximation.
/// Detailed rational queries canonicalize lazy ratios. For a Real without a
/// unit-basis rational, these booleans are all false and storage is VeryLarge;
/// that placeholder does not prove irrationality or actual large storage.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(verus_keep_ghost, verus_verify)]
pub struct RationalFacts {
    /// Positive evidence of a canonical denominator equal to one; false can mean no rational view.
    pub exact_integer: bool,
    /// Integer recognized with zero or at most 63 numerator magnitude bits.
    /// This conservative flag excludes i64::MIN despite that value fitting i64; false is not overflow proof.
    pub exact_small_integer_i64: bool,
    /// Positive evidence of a power-of-two canonical denominator, including integers and zero.
    pub exact_dyadic: bool,
    /// Positive evidence that the nonzero absolute value is an integer power of two.
    /// Negative values can set this flag; sign and fractional powers of two are independent.
    pub power_of_two: bool,
    /// Canonical rational size bucket, or VeryLarge when detailed Real facts have no unit-basis rational.
    pub storage: RationalStorageClass,
}

/// Cheap comparisons used by domain gates and filters.
///
/// These are exact field-derived comparisons for thresholds that appear in hot
/// kernels (`0`, `1`, and `|x| <= 1`). Keeping them explicit lets callers reject
/// invalid domains or select symbolic endpoints before numerical refinement.
/// The complete detailed-facts producer can canonicalize rational metadata;
/// this carrier is not a promise of an allocation-free or cache-free query.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(verus_keep_ghost, verus_verify)]
pub struct OrderingFacts {
    /// Known comparison of the signed value with one, or Unknown when the structural path declines.
    pub cmp_one: StructuralComparison,
    /// Known comparison of absolute value with one; Unknown is not a failed domain test.
    pub abs_cmp_one: StructuralComparison,
}

/// Domain facts for common unary functions.
///
/// These are structural domain certificates, not evaluations of the functions.
/// They let higher layers reject invalid symbolic operations or select exact
/// simplifications before asking for numerical refinement. Filters must produce
/// certificates or explicit uncertainty.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(verus_keep_ghost, verus_verify)]
pub struct DomainFacts {
    /// Nonzero domain x != 0 for reciprocal/inverse.
    pub reciprocal: DomainStatus,
    /// Nonnegative domain x >= 0 for the principal real square root.
    pub sqrt: DomainStatus,
    /// Strictly positive domain x > 0 for real logarithms.
    pub log: DomainStatus,
    /// Closed domain -1 <= x <= 1 for real arcsine/arccosine.
    pub asin_acos: DomainStatus,
    /// Closed absolute-unit domain -1 <= x <= 1, not the interval [0, 1].
    pub unit_interval_closed: DomainStatus,
    /// Open absolute-unit domain -1 < x < 1, not the interval (0, 1).
    pub unit_interval_open: DomainStatus,
    /// Domain x >= 1 for principal real inverse hyperbolic cosine.
    pub acosh: DomainStatus,
    /// Open domain -1 < x < 1 for real inverse hyperbolic tangent.
    pub atanh: DomainStatus,
}

/// Snapshot of retained class metadata, not a minimal algebraic decomposition.
/// Family flags describe the unscaled basis, including forms that admit further
/// simplification. They do not establish nonzero dependence after an outer zero scale.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(verus_keep_ghost, verus_verify)]
pub struct SymbolicFacts {
    /// Coarse family of the current scalar representation.
    pub kind: StructuralKind,
    /// Constant/Unknown relative to modeled variables, not the defining polynomial degree.
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
    /// True when a computable payload is present or the class is opaque.
    /// Named materialized constants and even signaled rationals can set this;
    /// lazy named certificates can leave it false. It is a storage-state flag,
    /// not proof that mathematical evaluation requires an opaque representation.
    pub computable_required: bool,
}

/// Opt-in detailed facts for callers that can use richer structural dispatch.
///
/// This intentionally layers on top of [`RealStructuralFacts`] instead of
/// adding cost to the existing hot minimal query. It can canonicalize lazy
/// rationals and populate retained facts; it does not promise bounded total
/// work, immutable caches, or a complete classification of arbitrary identities.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(verus_keep_ghost, verus_verify)]
pub struct RealDetailedFacts {
    /// Minimal sign, zero, magnitude and rational-availability facts used by the richer report.
    pub base: RealStructuralFacts,
    /// Recognized zero/signed-unit identities; false flags do not prove all other values.
    pub identity: IdentityFacts,
    /// Canonical rational facts when exposed, otherwise conservative false flags and a VeryLarge bucket.
    pub rational: RationalFacts,
    /// Exponent-band categories for float planning, subject to their documented rounding limitation.
    pub primitive: PrimitiveFacts,
    /// Threshold comparisons with one and absolute one, preserving unknown outcomes.
    pub ordering: OrderingFacts,
    /// Structural unary-domain decisions; these do not evaluate the requested functions.
    pub domains: DomainFacts,
    /// Retained class and family metadata, including a payload-presence flag.
    pub symbolic: SymbolicFacts,
}

#[cfg(verus_keep_ghost)]
verus! {
/// Ghost-only view used by the bit-operation specifications; no runtime query is added.
impl SymbolicDependencyMask {
    /// Expose the complete raw u16 bitset to Verus, including unnamed bits.
    pub closed spec fn model_bits(self) -> u16 { self.0 }
}

}
