//! Predicate result states and strict escalation metadata.

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
/// implementation type. Higher layers can observe which certified geometric
/// schedule decided the topology without depending on the internal `Real`
/// expression tree or on a particular determinant storage representation.
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
/// any topology decision. This preserves the exact-computation boundary between
/// geometric object structure and arithmetic packages.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum DeterminantScheduleHint {
    /// Some fixed points have certified sparse support and no fixed point has
    /// unknown zero status, so a sparse determinant schedule is a candidate.
    ///
    /// Sparse exact determinant formulas are classical arithmetic-package
    /// choices. They should still be paired with exact reduction schedules such
    /// as fraction-free elimination when appropriate.
    SparseSupportCandidate {
        /// Exact predicate kernel shape that would consume the schedule.
        kernel: ExactPredicateKernel,
        /// Number of fixed points with origin or one-hot support.
        fixed_sparse_points: u32,
    },
    /// Every fixed coordinate has one shared reduced denominator.
    ///
    /// Keep the borrowed geometric-object scale available instead of immediately expanding every
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

/// Exact predicate result with explicit uncertainty.
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

/// Strict predicate escalation policy shared with exact downstream algorithms.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct PredicatePolicy;

impl PredicatePolicy {
    /// Conservative default: topology is decided by exact/refined paths.
    pub const STRICT: Self = Self;

    /// Lowest binary precision Real refinement may request.
    pub const MAX_REFINEMENT_PRECISION: i32 = -512;
}

impl Default for PredicatePolicy {
    fn default() -> Self {
        Self::STRICT
    }
}
