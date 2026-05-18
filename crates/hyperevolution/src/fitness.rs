//! Exact fitness values and comparisons.

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
                for (left, right) in left.iter().zip(right) {
                    let comparison = compare_real(left, right, direction);
                    if comparison != FitnessComparison::Equal {
                        return comparison;
                    }
                }
                match left.len().cmp(&right.len()) {
                    Ordering::Equal => FitnessComparison::Equal,
                    Ordering::Less => FitnessComparison::Worse,
                    Ordering::Greater => FitnessComparison::Better,
                }
            }
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
