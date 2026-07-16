//! Exact fitness values and comparisons.
//!
//! Fitness is report-bearing evidence, not a primitive score. Exact scalar and
//! vector comparisons use `hyperreal::Real`; interval comparisons keep overlap
//! and invalid bounds visible instead of ranking candidates through lossy
//! midpoint or endpoint floats. Proposal search may be heuristic, but archive
//! promotion needs exact or explicitly uncertain evidence. The README lists
//! the supporting interval-analysis and exact-computation references.

use std::cmp::Ordering;

use hyperreal::Real;

use crate::{CandidateId, ReplayStatus};

/// Direction of an objective.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum FitnessDirection {
    /// Smaller values are better.
    Minimize,
    /// Larger values are better.
    Maximize,
}

/// Exact fitness payload.
#[derive(Clone, Debug, PartialEq)]
pub enum FitnessValue {
    /// Single exact objective.
    Scalar(Box<Real>),
    /// Ordered exact objectives.
    Lexicographic(Vec<Real>),
    /// Pareto objective vector.
    Pareto(Vec<Real>),
    /// Certified interval enclosure for one objective.
    Interval(Box<FitnessInterval>),
}

/// Exact interval enclosure for one objective.
#[derive(Clone, Debug, PartialEq)]
pub struct FitnessInterval {
    /// Lower enclosure endpoint.
    pub lower: Real,
    /// Upper enclosure endpoint.
    pub upper: Real,
}

/// Fitness report with replay evidence.
#[derive(Clone, Debug, PartialEq)]
pub struct FitnessReport {
    /// Candidate id.
    pub candidate: CandidateId,
    /// Fitness value.
    pub value: FitnessValue,
    /// Replay status from the domain crate.
    pub replay: ReplayStatus,
    /// Evidence strings from predicates/residuals/domain reports.
    pub evidence: Vec<String>,
}

/// Total comparison for scalar and lexicographic reports.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum FitnessComparison {
    /// Left report is better.
    Better,
    /// Right report is better.
    Worse,
    /// Reports tie under the selected objective order.
    Equal,
    /// Exact comparison could not certify an ordering.
    Unknown,
}

/// Explicit comparison result for interval fitness enclosures.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum FitnessIntervalComparison {
    /// Left interval is strictly better than right.
    Better,
    /// Right interval is strictly better than left.
    Worse,
    /// Endpoints are exactly equal.
    Equal,
    /// Intervals overlap, so the ordering is policy-dependent.
    Overlapping,
    /// At least one interval has `lower > upper`.
    InvalidBounds,
    /// Exact endpoint ordering could not be certified.
    Unknown,
}

/// Pareto relation between two objective vectors.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ParetoRelation {
    /// Left dominates right.
    Dominates,
    /// Left is dominated by right.
    Dominated,
    /// Neither vector dominates.
    NonDominated,
    /// Vectors are equal.
    Equal,
    /// Exact comparison could not certify the relation.
    Unknown,
}

impl FitnessValue {
    /// Compares scalar or lexicographic values exactly.
    pub fn compare_total(&self, other: &Self, direction: FitnessDirection) -> FitnessComparison {
        match (self, other) {
            (Self::Scalar(left), Self::Scalar(right)) => compare_real(left, right, direction),
            (Self::Lexicographic(left), Self::Lexicographic(right)) => {
                if left.len() != right.len() {
                    return FitnessComparison::Unknown;
                }
                for (left, right) in left.iter().zip(right) {
                    let comparison = compare_real(left, right, direction);
                    if comparison != FitnessComparison::Equal {
                        return comparison;
                    }
                }
                FitnessComparison::Equal
            }
            (Self::Interval(left), Self::Interval(right)) => match left.compare(right, direction) {
                FitnessIntervalComparison::Better => FitnessComparison::Better,
                FitnessIntervalComparison::Worse => FitnessComparison::Worse,
                FitnessIntervalComparison::Equal => FitnessComparison::Equal,
                FitnessIntervalComparison::Overlapping
                | FitnessIntervalComparison::InvalidBounds
                | FitnessIntervalComparison::Unknown => FitnessComparison::Unknown,
            },
            _ => FitnessComparison::Unknown,
        }
    }

    /// Compares Pareto vectors exactly under a shared objective direction.
    pub fn compare_pareto(&self, other: &Self, direction: FitnessDirection) -> ParetoRelation {
        let (Self::Pareto(left), Self::Pareto(right)) = (self, other) else {
            return ParetoRelation::Unknown;
        };
        if left.len() != right.len() {
            return ParetoRelation::Unknown;
        }

        let mut better = false;
        let mut worse = false;
        for (left, right) in left.iter().zip(right) {
            match compare_real(left, right, direction) {
                FitnessComparison::Better => better = true,
                FitnessComparison::Worse => worse = true,
                FitnessComparison::Equal => {}
                FitnessComparison::Unknown => return ParetoRelation::Unknown,
            }
        }
        match (better, worse) {
            (false, false) => ParetoRelation::Equal,
            (true, false) => ParetoRelation::Dominates,
            (false, true) => ParetoRelation::Dominated,
            (true, true) => ParetoRelation::NonDominated,
        }
    }
}

impl FitnessInterval {
    /// Creates an exact interval enclosure.
    pub fn new(lower: Real, upper: Real) -> Self {
        Self { lower, upper }
    }

    /// Returns true when `lower <= upper` can be certified exactly.
    pub fn has_valid_bounds(&self) -> Option<bool> {
        match self.lower.partial_cmp(&self.upper) {
            Some(Ordering::Less | Ordering::Equal) => Some(true),
            Some(Ordering::Greater) => Some(false),
            None => None,
        }
    }

    /// Compares two interval enclosures without midpoint or float fallback.
    ///
    /// For minimization, `self` is strictly better only when its upper endpoint
    /// is less than the other's lower endpoint. For maximization, the direction
    /// is reversed. Overlapping intervals return [`FitnessIntervalComparison::Overlapping`]
    /// so selection policies can refine, keep both, or use a named surrogate
    /// rather than silently inventing an ordering.
    pub fn compare(&self, other: &Self, direction: FitnessDirection) -> FitnessIntervalComparison {
        match (self.has_valid_bounds(), other.has_valid_bounds()) {
            (Some(true), Some(true)) => {}
            (Some(false), _) | (_, Some(false)) => {
                return FitnessIntervalComparison::InvalidBounds;
            }
            (None, _) | (_, None) => return FitnessIntervalComparison::Unknown,
        }
        if self.lower == other.lower && self.upper == other.upper {
            return FitnessIntervalComparison::Equal;
        }

        // These are the only two cross-interval orderings needed below. Keep
        // them because the overlap/unknown cases otherwise repeat both exact
        // endpoint comparisons after the strict-separation checks fail.
        let lower_against_upper = self.lower.partial_cmp(&other.upper);
        let upper_against_lower = self.upper.partial_cmp(&other.lower);
        match direction {
            FitnessDirection::Minimize => {
                if matches!(upper_against_lower, Some(Ordering::Less)) {
                    FitnessIntervalComparison::Better
                } else if matches!(lower_against_upper, Some(Ordering::Greater)) {
                    FitnessIntervalComparison::Worse
                } else if upper_against_lower.is_none() || lower_against_upper.is_none() {
                    FitnessIntervalComparison::Unknown
                } else {
                    FitnessIntervalComparison::Overlapping
                }
            }
            FitnessDirection::Maximize => {
                if matches!(lower_against_upper, Some(Ordering::Greater)) {
                    FitnessIntervalComparison::Better
                } else if matches!(upper_against_lower, Some(Ordering::Less)) {
                    FitnessIntervalComparison::Worse
                } else if lower_against_upper.is_none() || upper_against_lower.is_none() {
                    FitnessIntervalComparison::Unknown
                } else {
                    FitnessIntervalComparison::Overlapping
                }
            }
        }
    }
}

fn compare_real(left: &Real, right: &Real, direction: FitnessDirection) -> FitnessComparison {
    match left.partial_cmp(right) {
        Some(Ordering::Less) => match direction {
            FitnessDirection::Minimize => FitnessComparison::Better,
            FitnessDirection::Maximize => FitnessComparison::Worse,
        },
        Some(Ordering::Greater) => match direction {
            FitnessDirection::Minimize => FitnessComparison::Worse,
            FitnessDirection::Maximize => FitnessComparison::Better,
        },
        Some(Ordering::Equal) => FitnessComparison::Equal,
        None => FitnessComparison::Unknown,
    }
}
