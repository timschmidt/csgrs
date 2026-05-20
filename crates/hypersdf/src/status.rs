//! Public status and report types for exact-aware SDF classification.
//!
//! These types intentionally separate *sign classification* from *metric
//! distance*. Yap's exact-geometric-computation model treats topology decisions
//! as certified predicates over retained geometric objects, not as accidental
//! consequences of approximate scalar samples. See Yap, "Towards Exact
//! Geometric Computation," *Computational Geometry* 7.1-2 (1997).

use hyperlimit::{Certainty, Escalation, PredicateOutcome, RefinementNeed};

/// Domain validity for a retained SDF expression.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SdfDomainStatus {
    /// All checked domain requirements are certified valid.
    Valid,
    /// At least one checked domain requirement is certified invalid.
    Invalid,
    /// Domain validity could not be certified.
    Unknown,
}

impl SdfDomainStatus {
    /// Combine two domain statuses conservatively.
    pub const fn combine(self, other: Self) -> Self {
        match (self, other) {
            (Self::Invalid, _) | (_, Self::Invalid) => Self::Invalid,
            (Self::Unknown, _) | (_, Self::Unknown) => Self::Unknown,
            (Self::Valid, Self::Valid) => Self::Valid,
        }
    }
}

/// Availability and provenance of gradient/normal information.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SdfGradientStatus {
    /// An exact symbolic gradient is available for the retained expression.
    ExactSymbolic,
    /// Exact piecewise gradients are available, but active-branch selection is
    /// itself a predicate question.
    PiecewiseExact,
    /// Gradient data is not available for this expression.
    Unavailable,
}

impl SdfGradientStatus {
    /// Combine two statuses through CSG-style min/max composition.
    pub const fn csg_pair(self, other: Self) -> Self {
        match (self, other) {
            (Self::Unavailable, _) | (_, Self::Unavailable) => Self::Unavailable,
            (Self::PiecewiseExact, _) | (_, Self::PiecewiseExact) => Self::PiecewiseExact,
            (Self::ExactSymbolic, Self::ExactSymbolic) => Self::PiecewiseExact,
        }
    }
}

/// Conservative Lipschitz evidence for a retained scalar field.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SdfLipschitzStatus {
    /// A global exact bound is structurally known.
    GlobalExact,
    /// A bound may be derived for a bounded query domain.
    LocalOnly,
    /// No Lipschitz evidence is currently exposed.
    Unknown,
}

impl SdfLipschitzStatus {
    /// Combine two statuses through CSG-style min/max composition.
    pub const fn csg_pair(self, other: Self) -> Self {
        match (self, other) {
            (Self::GlobalExact, Self::GlobalExact) => Self::GlobalExact,
            (Self::Unknown, _) | (_, Self::Unknown) => Self::Unknown,
            _ => Self::LocalOnly,
        }
    }
}

/// Metric meaning carried by an SDF or implicit-field expression.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SdfMetricStatus {
    /// The scalar is a true signed Euclidean distance where supported.
    ExactSignedDistance,
    /// The scalar has the correct inside/outside sign and zero set, but is not
    /// certified as a Euclidean distance.
    SignEquivalent,
    /// The scalar is an unsigned distance-like quantity.
    UnsignedDistance,
    /// The scalar is a conservative lower distance bound.
    ConservativeLowerBound,
    /// The value came from a sampled approximation.
    SampledApproximation,
    /// The value came from a lossy adapter such as preview meshing or shader
    /// lowering.
    LossyAdapterValue,
    /// No metric claim is available.
    Unknown,
}

impl SdfMetricStatus {
    /// Returns the strongest status preserved by CSG-style min/max operations.
    pub const fn csg_pair(self, other: Self) -> Self {
        match (self, other) {
            (Self::ExactSignedDistance, Self::ExactSignedDistance) => Self::SignEquivalent,
            (Self::SignEquivalent, Self::ExactSignedDistance)
            | (Self::ExactSignedDistance, Self::SignEquivalent)
            | (Self::SignEquivalent, Self::SignEquivalent) => Self::SignEquivalent,
            _ => Self::Unknown,
        }
    }

    /// Returns the metric status after sign complement.
    pub const fn complemented(self) -> Self {
        self
    }
}

/// Exact evidence state attached to a report.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SdfEvidenceStatus {
    /// Classification was decided by an exact or certified Hyper predicate.
    Certified {
        /// Certainty reported by the underlying predicate.
        certainty: Certainty,
        /// Escalation stage that decided the predicate.
        stage: Escalation,
    },
    /// Classification could not be certified under the selected policy.
    Unknown {
        /// Additional capability requested by the underlying predicate.
        needed: RefinementNeed,
        /// Escalation stage where evaluation stopped.
        stage: Escalation,
    },
}

impl SdfEvidenceStatus {
    /// Convert a Hyperlimit predicate outcome into SDF evidence.
    pub const fn from_outcome<T>(outcome: &PredicateOutcome<T>) -> Self {
        match outcome {
            PredicateOutcome::Decided {
                certainty, stage, ..
            } => Self::Certified {
                certainty: *certainty,
                stage: *stage,
            },
            PredicateOutcome::Unknown { needed, stage } => Self::Unknown {
                needed: *needed,
                stage: *stage,
            },
        }
    }

    /// Returns whether the evidence is certified.
    pub const fn is_certified(self) -> bool {
        matches!(self, Self::Certified { .. })
    }
}

/// Location of a point relative to a closed negative-inside level set.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SdfPointLocation {
    /// The query lies inside the negative side of the field.
    Inside,
    /// The query lies on the zero level set.
    Boundary,
    /// The query lies outside the field.
    Outside,
    /// The selected policy could not certify the location.
    Unknown,
}

impl SdfPointLocation {
    /// Returns the complemented location for `-field`.
    pub const fn complemented(self) -> Self {
        match self {
            Self::Inside => Self::Outside,
            Self::Boundary => Self::Boundary,
            Self::Outside => Self::Inside,
            Self::Unknown => Self::Unknown,
        }
    }
}

/// Conservative location of an axis-aligned cell relative to a level set.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SdfCellLocation {
    /// Every point in the closed cell is certified inside.
    ConservativeInside,
    /// The cell is certified to meet the boundary, or it may contain both
    /// inside and outside points.
    Boundary,
    /// Every point in the closed cell is certified outside.
    ConservativeOutside,
    /// The selected policy could not certify a conservative answer.
    Unknown,
}

impl SdfCellLocation {
    /// Returns the complemented location for `-field`.
    pub const fn complemented(self) -> Self {
        match self {
            Self::ConservativeInside => Self::ConservativeOutside,
            Self::Boundary => Self::Boundary,
            Self::ConservativeOutside => Self::ConservativeInside,
            Self::Unknown => Self::Unknown,
        }
    }
}

/// Freshness of a prepared expression relative to its source construction.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SdfFreshness {
    /// The prepared carrier has no independent source version to compare.
    Unversioned,
    /// Prepared facts match the source version.
    Current,
    /// Prepared facts are known to be stale.
    Stale,
}

/// Report returned by point classification.
#[derive(Clone, Debug, PartialEq)]
pub struct SdfPointClassificationReport {
    /// Query point.
    pub point: hyperlimit::Point3,
    /// Decided or unknown point location.
    pub location: SdfPointLocation,
    /// Optional retained scalar or sign-equivalent value.
    pub scalar_value: Option<hyperreal::Real>,
    /// Metric claim for `scalar_value` and the underlying field.
    pub metric_status: SdfMetricStatus,
    /// Exact/certified evidence status.
    pub evidence: SdfEvidenceStatus,
    /// Prepared-source freshness.
    pub freshness: SdfFreshness,
}

/// Report returned by AABB/cell classification.
#[derive(Clone, Debug, PartialEq)]
pub struct SdfCellClassificationReport {
    /// Minimum corner of the closed query cell.
    pub min: hyperlimit::Point3,
    /// Maximum corner of the closed query cell.
    pub max: hyperlimit::Point3,
    /// Conservative cell location.
    pub location: SdfCellLocation,
    /// Metric claim for the underlying field.
    pub metric_status: SdfMetricStatus,
    /// Exact/certified evidence status.
    pub evidence: SdfEvidenceStatus,
    /// Prepared-source freshness.
    pub freshness: SdfFreshness,
}
