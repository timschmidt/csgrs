//! Predicate result states and centralized escalation metadata.

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
    /// The result follows from a policy-authorized terminal approximation.
    Approximate,
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
    Orient2RationalDet2,
    /// Rational translated 3x3 determinant for 3D orientation.
    Orient3RationalDet3,
    /// Rational lifted 3x3 determinant for the 2D in-circle predicate.
    Incircle2RationalLiftedDet3,
    /// Rational lifted 4x4 determinant for the 3D in-sphere predicate.
    Insphere3RationalLiftedDet4,
}

/// Advisory determinant schedule selected from retained geometric facts.
///
/// This is a schedule hint, not a correctness certificate. It lets retained
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
    /// The retained facts do not certify a fixed exact-rational determinant
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
    #[inline]
    pub fn value(self) -> Option<T> {
        match self {
            Self::Decided { value, .. } => Some(value),
            Self::Unknown { .. } => None,
        }
    }
}

impl PredicateOutcome<bool> {
    /// Combine two predicate outcomes with logical conjunction.
    ///
    /// A decided `false` operand is sufficient even when the other operand is
    /// unknown. Two decided `true` operands retain the weaker certainty and
    /// later escalation stage.
    pub const fn and(self, other: Self) -> Self {
        match (self, other) {
            (
                Self::Decided {
                    value: false,
                    certainty,
                    stage,
                },
                _,
            )
            | (
                _,
                Self::Decided {
                    value: false,
                    certainty,
                    stage,
                },
            ) => Self::decided(false, certainty, stage),
            (
                Self::Decided {
                    value: true,
                    certainty: left_certainty,
                    stage: left_stage,
                },
                Self::Decided {
                    value: true,
                    certainty: right_certainty,
                    stage: right_stage,
                },
            ) => Self::decided(
                true,
                weaker_certainty(left_certainty, right_certainty),
                later_stage(left_stage, right_stage),
            ),
            (Self::Unknown { needed, stage }, _) | (_, Self::Unknown { needed, stage }) => {
                Self::unknown(needed, stage)
            }
        }
    }

    /// Combine two predicate outcomes with logical disjunction.
    ///
    /// A decided `true` operand is sufficient even when the other operand is
    /// unknown. Two decided `false` operands retain the weaker certainty and
    /// later escalation stage.
    pub const fn or(self, other: Self) -> Self {
        match (self, other) {
            (
                Self::Decided {
                    value: true,
                    certainty,
                    stage,
                },
                _,
            )
            | (
                _,
                Self::Decided {
                    value: true,
                    certainty,
                    stage,
                },
            ) => Self::decided(true, certainty, stage),
            (
                Self::Decided {
                    value: false,
                    certainty: left_certainty,
                    stage: left_stage,
                },
                Self::Decided {
                    value: false,
                    certainty: right_certainty,
                    stage: right_stage,
                },
            ) => Self::decided(
                false,
                weaker_certainty(left_certainty, right_certainty),
                later_stage(left_stage, right_stage),
            ),
            (Self::Unknown { needed, stage }, _) | (_, Self::Unknown { needed, stage }) => {
                Self::unknown(needed, stage)
            }
        }
    }
}

const fn weaker_certainty(left: Certainty, right: Certainty) -> Certainty {
    match (left, right) {
        (Certainty::Approximate, _) | (_, Certainty::Approximate) => Certainty::Approximate,
        (Certainty::Filtered, _) | (_, Certainty::Filtered) => Certainty::Filtered,
        (Certainty::Exact, Certainty::Exact) => Certainty::Exact,
    }
}

const fn later_stage(left: Escalation, right: Escalation) -> Escalation {
    if escalation_rank(left) >= escalation_rank(right) {
        left
    } else {
        right
    }
}

const fn escalation_rank(stage: Escalation) -> u8 {
    match stage {
        Escalation::Structural => 0,
        Escalation::Filter => 1,
        Escalation::Exact => 2,
        Escalation::Refined => 3,
        Escalation::Undecided => 4,
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

/// Predicate escalation policy shared with downstream geometry algorithms.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct PredicatePolicy {
    final_approximation_precision: Option<i32>,
}

impl PredicatePolicy {
    /// Topology is decided only by exact or certified-refinement paths.
    pub const STRICT: Self = Self {
        final_approximation_precision: None,
    };

    /// Permit one terminal 512-bit approximation after certification is
    /// exhausted.
    pub const APPROXIMATE_512: Self = Self {
        final_approximation_precision: Some(-512),
    };

    /// Lowest binary precision Real refinement may request.
    pub const MAX_REFINEMENT_PRECISION: i32 = -512;

    /// Terminal approximation precision, when one is authorized.
    pub const fn final_approximation_precision(self) -> Option<i32> {
        self.final_approximation_precision
    }
}

impl Default for PredicatePolicy {
    fn default() -> Self {
        Self::APPROXIMATE_512
    }
}

/// Temporary workspace-wide predicate policy.
///
/// Existing call sites use this value directly. Keeping it centralized makes
/// returning to [`PredicatePolicy::STRICT`] a one-line policy change.
#[allow(non_upper_case_globals)]
pub const PredicatePolicy: PredicatePolicy = PredicatePolicy::APPROXIMATE_512;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn boolean_outcome_combinators_preserve_certainty_and_short_circuit_unknowns() {
        let exact_true = PredicateOutcome::decided(true, Certainty::Exact, Escalation::Structural);
        let approximate_true =
            PredicateOutcome::decided(true, Certainty::Approximate, Escalation::Refined);
        let exact_false = PredicateOutcome::decided(false, Certainty::Exact, Escalation::Exact);
        let unknown =
            PredicateOutcome::unknown(RefinementNeed::RealRefinement, Escalation::Undecided);

        assert_eq!(
            exact_true.and(approximate_true),
            PredicateOutcome::decided(true, Certainty::Approximate, Escalation::Refined)
        );
        assert_eq!(unknown.and(exact_false), exact_false);
        assert_eq!(unknown.or(exact_true), exact_true);
        assert_eq!(exact_false.or(unknown), unknown);
    }
}
