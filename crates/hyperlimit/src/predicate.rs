//! Predicate result states and escalation policy.

/// A concrete sign.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Sign {
    /// Strictly negative.
    Negative,
    /// Exactly zero.
    Zero,
    /// Strictly positive.
    Positive,
}

impl Sign {
    /// Returns the opposite sign.
    pub const fn reversed(self) -> Self {
        match self {
            Self::Negative => Self::Positive,
            Self::Zero => Self::Zero,
            Self::Positive => Self::Negative,
        }
    }
}

/// How strongly a predicate result is known.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Certainty {
    /// The result follows from exact or structural information.
    Exact,
    /// The result follows from conservative structural Real facts.
    Filtered,
}

/// What a Real value or predicate currently knows about a sign.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SignKnowledge {
    /// The sign is known with the given certainty.
    Known {
        /// Known sign.
        sign: Sign,
        /// Certainty level for the sign.
        certainty: Certainty,
    },
    /// The value is known to be nonzero but its sign has not been exposed.
    NonZero,
    /// The sign cannot be decided without escalation.
    Unknown,
}

impl SignKnowledge {
    /// Construct exactly known sign knowledge.
    pub const fn exact(sign: Sign) -> Self {
        Self::Known {
            sign,
            certainty: Certainty::Exact,
        }
    }

    /// Construct sign knowledge produced by a conservative filter.
    pub const fn filtered(sign: Sign) -> Self {
        Self::Known {
            sign,
            certainty: Certainty::Filtered,
        }
    }

    /// Return the concrete sign if it is known.
    pub const fn sign(self) -> Option<Sign> {
        match self {
            Self::Known { sign, .. } => Some(sign),
            Self::NonZero | Self::Unknown => None,
        }
    }
}

/// Which stage decided, or failed to decide, a predicate.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Escalation {
    /// Decided using structural Real facts.
    Structural,
    /// Decided using exact structural term facts.
    Filter,
    /// Decided using exact Real arithmetic.
    Exact,
    /// Decided after adaptive Real refinement.
    Refined,
    /// Not decided by the enabled stages.
    Undecided,
}

/// Exact determinant kernel selected for a predicate.
///
/// This is intentionally a predicate-layer description, not a scalar or matrix
/// implementation type. The exact-geometric-computation separation follows
/// Yap, "Towards Exact Geometric Computation," *Computational Geometry*
/// 7.1-2 (1997): higher layers can observe which certified geometric schedule
/// decided the topology without depending on the internal `Real` expression
/// tree or on a particular determinant storage representation.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ExactPredicateKernel {
    /// Rational 2x2 determinant for 2D orientation.
    Orient2dRationalDet2,
    /// Rational translated 3x3 determinant for 3D orientation.
    Orient3dRationalDet3,
    /// Rational lifted 3x3 determinant for the 2D in-circle predicate.
    Incircle2dRationalLiftedDet3,
    /// Rational lifted 4x4 determinant for the 3D in-sphere predicate.
    Insphere3dRationalLiftedDet4,
}

/// Advisory determinant schedule selected from prepared geometric facts.
///
/// This is a schedule hint, not a correctness certificate. It lets prepared
/// predicates and higher crates reuse object-level facts such as sparse support,
/// dyadic coordinates, or shared denominators before constructing generic
/// `Real` expressions. The exact predicate report remains the certificate for
/// any topology decision. This directly follows Yap's exact-geometric-
/// computation boundary between geometric object structure and arithmetic
/// packages; see Yap, "Towards Exact Geometric Computation," *Computational
/// Geometry* 7.1-2 (1997).
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum DeterminantScheduleHint {
    /// Some fixed points have certified sparse support and no fixed point has
    /// unknown zero status, so a sparse determinant schedule is a candidate.
    ///
    /// Sparse exact determinant formulas are classical arithmetic-package
    /// choices. They should still be paired with exact reduction schedules such
    /// as fraction-free elimination when appropriate; see Bareiss,
    /// "Sylvester's Identity and Multistep Integer-Preserving Gaussian
    /// Elimination," *Mathematics of Computation* 22.103 (1968).
    SparseSupportCandidate {
        /// Exact predicate kernel shape that would consume the schedule.
        kernel: ExactPredicateKernel,
        /// Number of fixed points with origin or one-hot support.
        fixed_sparse_points: u32,
    },
    /// Every fixed coordinate has one shared reduced denominator.
    ///
    /// This is the borrowed common-scale case highlighted by Yap: keep the
    /// geometric object scale available instead of immediately expanding every
    /// coordinate as an independent scalar rational.
    SharedDenominatorCandidate {
        /// Exact predicate kernel shape that would consume the schedule.
        kernel: ExactPredicateKernel,
    },
    /// Every fixed coordinate is dyadic, allowing shift-oriented exact rational
    /// schedules when the query coordinates are compatible.
    DyadicCandidate {
        /// Exact predicate kernel shape that would consume the schedule.
        kernel: ExactPredicateKernel,
    },
    /// Fixed coordinates are exact rational, but no more specific retained
    /// structure has been exposed.
    ExactRationalKernel {
        /// Exact predicate kernel shape that would consume the schedule.
        kernel: ExactPredicateKernel,
    },
    /// The prepared facts do not certify a fixed exact-rational determinant
    /// schedule; the generic `Real` predicate path is the honest fallback.
    GenericRealFallback,
}

/// Public precision-ladder stage for predicate provenance.
///
/// This is the API-facing version of the exact-computation ladder described by
/// Yap: separate geometric/topological decisions from approximate numeric
/// convenience, expose when a certified filter or exact reducer has proved a
/// sign, and make explicit approximation opt-in and auditable. See Yap,
/// "Towards Exact Geometric Computation," *Computational Geometry* 7.1-2
/// (1997).
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum PredicatePrecisionStage {
    /// Cached structural facts attached to a `Real` or geometric object proved
    /// the predicate.
    StructuralFact,
    /// Exact arithmetic or an exact fixed-kernel reducer proved the predicate.
    ExactReducer,
    /// A conservative enclosure/filter proved the predicate without full
    /// expression expansion.
    CertifiedFilter,
    /// Bounded `Real` refinement proved the predicate.
    BoundedRefinement,
    /// An explicitly requested approximate edge policy decided the predicate.
    ExplicitApproximatePolicy,
    /// No proof-producing stage is known for this certificate.
    Unknown,
}

/// Public semantic class for predicate-facing APIs.
///
/// This label is intentionally coarser than a certificate. It describes the
/// contract a caller should assume at an API boundary: exact topology,
/// deferred uncertainty, explicit approximate edge data, cache population, or
/// caller-controlled policy. The split follows Yap's exact-geometric-
/// computation discipline of keeping exact combinatorial decisions separate
/// from approximate numerical views and scheduling caches; see Yap, "Towards
/// Exact Geometric Computation," *Computational Geometry* 7.1-2 (1997).
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum PredicateApiSemantics {
    /// The API preserves exact predicate meaning when it returns a decided
    /// result.
    ExactPreserving,
    /// The API refuses to decide without a proof-producing route and returns
    /// explicit uncertainty instead of approximation.
    ApproximationDeferring,
    /// The API produces or accepts approximate edge data by explicit request.
    ApproximationForcing,
    /// The API records reusable structural/cache metadata without making that
    /// metadata a predicate proof by itself.
    CachePopulating,
    /// The API's escalation behavior is controlled by a caller-supplied policy.
    PolicyDependent,
}

/// Provenance certificate for a predicate decision or explicit non-decision.
///
/// Certificates are deliberately compact and copyable. They identify the
/// semantic route used by the predicate pipeline so applications can audit
/// topology decisions, benchmark dispatch choices, and keep approximate edge
/// policies visibly separate from exact computation.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum PredicateCertificate {
    /// Cheap facts attached to a `Real` or geometric object decided the result.
    StructuralFact,
    /// A conservative exact filter over structurally signed terms decided it.
    DeterminantFilter,
    /// A conservative exact interval enclosure decided the sign.
    CertifiedIntervalFilter,
    /// A conservative exact ball enclosure decided the sign.
    ///
    /// The ball is reduced to an exact closed interval from its center and
    /// radius, then certified by exact endpoint comparisons. This is a
    /// proof-producing filter in Yap's sense, not a primitive-float tolerance.
    CertifiedBallFilter,
    /// A fixed exact rational determinant kernel decided it.
    ExactRationalKernel {
        /// The selected exact determinant schedule.
        kernel: ExactPredicateKernel,
    },
    /// Exact scalar facts on the constructed `Real` expression decided it.
    ExactRealFact,
    /// Bounded Real refinement decided it.
    BoundedRefinement,
    /// Exact symbolic predicate support would be needed here.
    ExactSymbolicKernel,
    /// An explicitly requested approximate policy decided it.
    ApproximatePolicyFallback,
    /// No enabled certified route decided the predicate.
    Unknown,
}

impl PredicateCertificate {
    /// Return a coarse certificate for an already computed outcome.
    ///
    /// This is useful for older code paths that still return only
    /// [`PredicateOutcome`]. New exact-kernel code should prefer constructing a
    /// more specific certificate such as [`PredicateCertificate::ExactRationalKernel`].
    pub const fn from_outcome<T>(outcome: &PredicateOutcome<T>) -> Self {
        match outcome {
            PredicateOutcome::Decided { stage, .. } => match stage {
                Escalation::Structural => Self::StructuralFact,
                Escalation::Filter => Self::DeterminantFilter,
                Escalation::Exact => Self::ExactRealFact,
                Escalation::Refined => Self::BoundedRefinement,
                Escalation::Undecided => Self::Unknown,
            },
            PredicateOutcome::Unknown { .. } => Self::Unknown,
        }
    }

    /// Return the public precision-ladder stage represented by this
    /// certificate.
    ///
    /// This helper gives downstream crates a stable abstraction boundary for
    /// scheduling and diagnostics: they can react to "certified filter" or
    /// "exact reducer" without inspecting determinant internals or `Real`
    /// expression structure. That separation follows Yap's exact-geometric-
    /// computation model; see Yap, "Towards Exact Geometric Computation,"
    /// *Computational Geometry* 7.1-2 (1997).
    pub const fn precision_stage(self) -> PredicatePrecisionStage {
        match self {
            Self::StructuralFact => PredicatePrecisionStage::StructuralFact,
            Self::DeterminantFilter
            | Self::CertifiedIntervalFilter
            | Self::CertifiedBallFilter => PredicatePrecisionStage::CertifiedFilter,
            Self::ExactRationalKernel { .. } | Self::ExactRealFact => {
                PredicatePrecisionStage::ExactReducer
            }
            Self::BoundedRefinement => PredicatePrecisionStage::BoundedRefinement,
            Self::ApproximatePolicyFallback => PredicatePrecisionStage::ExplicitApproximatePolicy,
            Self::ExactSymbolicKernel | Self::Unknown => PredicatePrecisionStage::Unknown,
        }
    }

    /// Return whether this certificate represents a proof-producing predicate
    /// route.
    ///
    /// Approximate policy fallbacks are intentionally excluded: they may be
    /// acceptable at rendering or interoperability edges, but they are not
    /// exact predicate proofs in Yap's sense.
    pub const fn is_proof_producing(self) -> bool {
        match self.precision_stage() {
            PredicatePrecisionStage::StructuralFact
            | PredicatePrecisionStage::ExactReducer
            | PredicatePrecisionStage::CertifiedFilter
            | PredicatePrecisionStage::BoundedRefinement => true,
            PredicatePrecisionStage::ExplicitApproximatePolicy
            | PredicatePrecisionStage::Unknown => false,
        }
    }

    /// Return the API semantic class implied by this certificate.
    ///
    /// Proof-producing certificates are exact-preserving. Unknown certificates
    /// defer approximation by returning explicit uncertainty. Explicit
    /// approximate policy certificates remain visibly approximate so higher
    /// crates cannot accidentally treat them as topology proofs.
    pub const fn api_semantics(self) -> PredicateApiSemantics {
        match self.precision_stage() {
            PredicatePrecisionStage::StructuralFact
            | PredicatePrecisionStage::ExactReducer
            | PredicatePrecisionStage::CertifiedFilter
            | PredicatePrecisionStage::BoundedRefinement => PredicateApiSemantics::ExactPreserving,
            PredicatePrecisionStage::ExplicitApproximatePolicy => {
                PredicateApiSemantics::ApproximationForcing
            }
            PredicatePrecisionStage::Unknown => PredicateApiSemantics::ApproximationDeferring,
        }
    }
}

/// A predicate result with provenance.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum PredicateOutcome<T> {
    /// The predicate was decided.
    Decided {
        /// Decided predicate value.
        value: T,
        /// Certainty level for the result.
        certainty: Certainty,
        /// Stage that decided the result.
        stage: Escalation,
    },
    /// More capability, fallback, or refinement is needed.
    Unknown {
        /// Additional capability needed to decide the result.
        needed: RefinementNeed,
        /// Stage at which evaluation stopped.
        stage: Escalation,
    },
}

impl<T> PredicateOutcome<T> {
    /// Construct a decided predicate outcome.
    pub const fn decided(value: T, certainty: Certainty, stage: Escalation) -> Self {
        Self::Decided {
            value,
            certainty,
            stage,
        }
    }

    /// Construct an undecided predicate outcome.
    pub const fn unknown(needed: RefinementNeed, stage: Escalation) -> Self {
        Self::Unknown { needed, stage }
    }

    /// Return the decided value, or `None` when the outcome is unknown.
    pub fn value(self) -> Option<T> {
        match self {
            Self::Decided { value, .. } => Some(value),
            Self::Unknown { .. } => None,
        }
    }
}

/// A predicate outcome paired with a provenance certificate.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct PredicateReport<T> {
    /// The value or explicit uncertainty returned by the predicate.
    pub outcome: PredicateOutcome<T>,
    /// The semantic route that produced `outcome`.
    pub certificate: PredicateCertificate,
}

impl<T> PredicateReport<T> {
    /// Construct a predicate report.
    pub const fn new(outcome: PredicateOutcome<T>, certificate: PredicateCertificate) -> Self {
        Self {
            outcome,
            certificate,
        }
    }

    /// Construct a report by deriving a coarse certificate from the outcome.
    pub const fn from_outcome(outcome: PredicateOutcome<T>) -> Self {
        let certificate = PredicateCertificate::from_outcome(&outcome);
        Self {
            outcome,
            certificate,
        }
    }

    /// Return the decided value, or `None` when the outcome is unknown.
    pub fn value(self) -> Option<T> {
        self.outcome.value()
    }

    /// Return the API semantic class implied by this report's certificate.
    pub const fn api_semantics(&self) -> PredicateApiSemantics {
        self.certificate.api_semantics()
    }
}

/// What additional work would be required.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RefinementNeed {
    /// Exact arithmetic is needed.
    ExactArithmetic,
    /// More Real refinement is needed.
    RealRefinement,
    /// The Real-backed predicate pipeline cannot decide this case.
    Unsupported,
}

/// Runtime policy for predicate escalation.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct PredicatePolicy {
    /// Permit exact Real predicate paths when available.
    pub allow_exact: bool,
    /// Permit Real refinement when available.
    pub allow_refinement: bool,
    /// Lowest binary precision Real refinement may request.
    pub max_refinement_precision: Option<i32>,
}

impl PredicatePolicy {
    /// Conservative default: topology is decided by exact/refined paths.
    pub const STRICT: Self = Self {
        allow_exact: true,
        allow_refinement: true,
        max_refinement_precision: Some(-512),
    };

    /// Return the semantic class for APIs that accept this policy.
    ///
    /// The policy does not itself decide a predicate. It marks the API boundary
    /// as caller-controlled while individual predicate reports still carry exact
    /// certificates, approximation-deferring uncertainty, or explicit
    /// approximation certificates.
    pub const fn api_semantics(self) -> PredicateApiSemantics {
        PredicateApiSemantics::PolicyDependent
    }
}

impl Default for PredicatePolicy {
    fn default() -> Self {
        Self::STRICT
    }
}

#[cfg(test)]
mod tests {
    use super::{
        ExactPredicateKernel, PredicateApiSemantics, PredicateCertificate, PredicateOutcome,
        PredicatePolicy, PredicatePrecisionStage, PredicateReport, Sign,
    };

    #[test]
    fn predicate_certificate_precision_stage_classifies_provenance() {
        assert_eq!(
            PredicateCertificate::StructuralFact.precision_stage(),
            PredicatePrecisionStage::StructuralFact
        );
        assert_eq!(
            PredicateCertificate::DeterminantFilter.precision_stage(),
            PredicatePrecisionStage::CertifiedFilter
        );
        assert_eq!(
            PredicateCertificate::CertifiedIntervalFilter.precision_stage(),
            PredicatePrecisionStage::CertifiedFilter
        );
        assert_eq!(
            PredicateCertificate::CertifiedBallFilter.precision_stage(),
            PredicatePrecisionStage::CertifiedFilter
        );
        assert_eq!(
            PredicateCertificate::ExactRationalKernel {
                kernel: ExactPredicateKernel::Orient2dRationalDet2,
            }
            .precision_stage(),
            PredicatePrecisionStage::ExactReducer
        );
        assert_eq!(
            PredicateCertificate::ExactRealFact.precision_stage(),
            PredicatePrecisionStage::ExactReducer
        );
        assert_eq!(
            PredicateCertificate::BoundedRefinement.precision_stage(),
            PredicatePrecisionStage::BoundedRefinement
        );
        assert_eq!(
            PredicateCertificate::ApproximatePolicyFallback.precision_stage(),
            PredicatePrecisionStage::ExplicitApproximatePolicy
        );
        assert_eq!(
            PredicateCertificate::ExactSymbolicKernel.precision_stage(),
            PredicatePrecisionStage::Unknown
        );
        assert_eq!(
            PredicateCertificate::Unknown.precision_stage(),
            PredicatePrecisionStage::Unknown
        );
    }

    #[test]
    fn predicate_certificate_proof_producing_excludes_approximation_and_unknowns() {
        let proof_certificates = [
            PredicateCertificate::StructuralFact,
            PredicateCertificate::DeterminantFilter,
            PredicateCertificate::CertifiedIntervalFilter,
            PredicateCertificate::CertifiedBallFilter,
            PredicateCertificate::ExactRationalKernel {
                kernel: ExactPredicateKernel::Incircle2dRationalLiftedDet3,
            },
            PredicateCertificate::ExactRealFact,
            PredicateCertificate::BoundedRefinement,
        ];
        for certificate in proof_certificates {
            assert!(
                certificate.is_proof_producing(),
                "{certificate:?} should be proof-producing"
            );
        }

        let non_proof_certificates = [
            PredicateCertificate::ApproximatePolicyFallback,
            PredicateCertificate::ExactSymbolicKernel,
            PredicateCertificate::Unknown,
        ];
        for certificate in non_proof_certificates {
            assert!(
                !certificate.is_proof_producing(),
                "{certificate:?} should not be proof-producing"
            );
        }
    }

    #[test]
    fn coarse_certificate_from_outcome_still_maps_to_ladder() {
        let outcome = PredicateOutcome::decided(
            Sign::Positive,
            super::Certainty::Filtered,
            super::Escalation::Filter,
        );
        assert_eq!(
            PredicateCertificate::from_outcome(&outcome).precision_stage(),
            PredicatePrecisionStage::CertifiedFilter
        );
    }

    #[test]
    fn api_semantics_separates_exact_deferring_approximate_and_policy_boundaries() {
        assert_eq!(
            PredicateCertificate::ExactRealFact.api_semantics(),
            PredicateApiSemantics::ExactPreserving
        );
        assert_eq!(
            PredicateCertificate::Unknown.api_semantics(),
            PredicateApiSemantics::ApproximationDeferring
        );
        assert_eq!(
            PredicateCertificate::ApproximatePolicyFallback.api_semantics(),
            PredicateApiSemantics::ApproximationForcing
        );
        assert_eq!(
            PredicatePolicy::STRICT.api_semantics(),
            PredicateApiSemantics::PolicyDependent
        );

        let report = PredicateReport::new(
            PredicateOutcome::decided(
                Sign::Zero,
                super::Certainty::Exact,
                super::Escalation::Exact,
            ),
            PredicateCertificate::ExactRealFact,
        );
        assert_eq!(
            report.api_semantics(),
            PredicateApiSemantics::ExactPreserving
        );
    }
}
