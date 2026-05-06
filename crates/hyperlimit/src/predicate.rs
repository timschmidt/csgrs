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
    /// Returns the sign of a primitive floating value.
    pub fn from_f64(value: f64) -> Option<Self> {
        if value.is_nan() {
            None
        } else if value > 0.0 {
            Some(Self::Positive)
        } else if value < 0.0 {
            Some(Self::Negative)
        } else {
            Some(Self::Zero)
        }
    }

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
    /// The result follows from a conservative numeric filter.
    Filtered,
    /// The result follows from adaptive robust arithmetic on projected finite
    /// floating-point coordinates.
    RobustFloat,
    /// The result is approximate and should not be used for irreversible topology.
    Approximate,
}

/// What a scalar or predicate currently knows about a sign.
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
    /// Decided using structural scalar facts.
    Structural,
    /// Decided using a conservative numeric filter.
    Filter,
    /// Decided using exact scalar arithmetic.
    Exact,
    /// Decided using a robust backend fallback.
    RobustFallback,
    /// Decided after adaptive scalar refinement.
    Refined,
    /// Not decided by the enabled stages.
    Undecided,
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

/// What additional work would be required.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RefinementNeed {
    /// Exact arithmetic is needed.
    ExactArithmetic,
    /// A robust fallback backend is needed.
    RobustFallback,
    /// More scalar precision or refinement is needed.
    ScalarRefinement,
    /// The enabled backends cannot decide this case.
    Unsupported,
}

/// Runtime policy for predicate escalation.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct PredicatePolicy {
    /// Permit approximate signs when no proof is available.
    pub allow_approximate: bool,
    /// Permit robust backend fallback when available.
    pub allow_robust_fallback: bool,
    /// Permit exact backend paths when available.
    pub allow_exact: bool,
    /// Permit scalar refinement when available.
    pub allow_refinement: bool,
    /// Lowest binary precision scalar refinement may request.
    pub max_refinement_precision: Option<i32>,
}

impl PredicatePolicy {
    /// Conservative default: do not return approximate topology.
    pub const STRICT: Self = Self {
        allow_approximate: false,
        allow_robust_fallback: true,
        allow_exact: true,
        allow_refinement: true,
        max_refinement_precision: Some(-512),
    };

    /// Useful for prototyping, debugging, and visual previews.
    pub const APPROXIMATE: Self = Self {
        allow_approximate: true,
        allow_robust_fallback: false,
        allow_exact: false,
        allow_refinement: false,
        max_refinement_precision: None,
    };
}

impl Default for PredicatePolicy {
    fn default() -> Self {
        Self::STRICT
    }
}
