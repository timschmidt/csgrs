//! Candidate-domain certification for residual expressions.
//!
//! Exact replay should fail loudly when a candidate leaves the mathematical
//! domain of a residual, but callers often need that information before asking
//! for a scalar value. This module walks residual expression trees and reports
//! proof-bearing domain checks for division, negative powers, square roots,
//! logarithms, inverse circular functions, and inverse hyperbolic functions.
//! The checks use exact `Real` comparisons through `hyperlimit`, preserving
//! the rule that decisions consume certified facts or return
//! explicit uncertainty. The interval/domain separation also follows Moore's
//! interval-analysis discipline: domain validity is a precondition
//! certificate, not a residual tolerance.

use std::cmp::Ordering;

use hyperlimit::{PredicatePolicy, compare_reals_with_policy};
use hyperreal::Real;

use crate::eval::EvaluationContext;
use crate::model::Problem;
use crate::symbolic::{Expr, ExprEvalError, SymbolId};

/// Mathematical-domain condition checked for one expression node.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum DomainCheckKind {
    /// Denominator of `left / right` must be nonzero.
    DivisionDenominatorNonZero,
    /// Base of `base.powi(exponent)` must be nonzero when `exponent < 0`.
    NegativePowerBaseNonZero,
    /// Operand of `sqrt(value)` must be nonnegative.
    SqrtOperandNonNegative,
    /// Operand of `log10(value)` must be strictly positive.
    Log10OperandPositive,
    /// Operand of `ln(value)` must be strictly positive.
    LnOperandPositive,
    /// Operand of `asin(value)` must lie in the closed interval `[-1, 1]`.
    AsinOperandInClosedUnitInterval,
    /// Operand of `acos(value)` must lie in the closed interval `[-1, 1]`.
    AcosOperandInClosedUnitInterval,
    /// Operand of `acosh(value)` must be greater than or equal to `1`.
    AcoshOperandAtLeastOne,
    /// Operand of `atanh(value)` must lie in the open interval `(-1, 1)`.
    AtanhOperandInOpenUnitInterval,
}

/// Proof status for one domain check.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum DomainCheckStatus {
    /// The domain condition was proved exactly under the supplied policy.
    CertifiedValid,
    /// The domain condition was proved false.
    CertifiedInvalid,
    /// The exact comparison was not decidable under the supplied policy.
    Unknown,
    /// A required candidate symbol was not bound.
    UnboundSymbol {
        /// Missing solver symbol.
        symbol: SymbolId,
    },
    /// Operand evaluation failed before its domain predicate could be checked.
    EvaluationFailure {
        /// Compact diagnostic for the lower-level arithmetic failure.
        message: String,
    },
}

impl DomainCheckStatus {
    /// Returns whether this status proves the checked domain condition.
    pub const fn is_certified_valid(&self) -> bool {
        matches!(self, Self::CertifiedValid)
    }

    /// Returns whether this status proves a domain violation.
    pub const fn is_certified_invalid(&self) -> bool {
        matches!(self, Self::CertifiedInvalid)
    }

    /// Returns whether this status leaves the domain condition unresolved.
    pub const fn is_unknown(&self) -> bool {
        matches!(self, Self::Unknown)
    }
}

/// One domain check emitted while walking a residual expression.
#[derive(Clone, Debug, PartialEq)]
pub struct DomainCheck {
    /// Source constraint index in the problem.
    pub constraint_index: usize,
    /// Constraint name copied for diagnostics.
    pub constraint_name: String,
    /// Stable expression path using node labels and child indices.
    pub expression_path: String,
    /// Checked domain condition.
    pub kind: DomainCheckKind,
    /// Operand value when it could be evaluated exactly.
    pub operand: Option<Real>,
    /// Proof status for the domain condition.
    pub status: DomainCheckStatus,
}

/// Report for candidate-domain checks over active residual rows.
#[derive(Clone, Debug, PartialEq)]
pub struct CandidateDomainReport {
    /// Per-node domain checks.
    pub checks: Vec<DomainCheck>,
    /// Number of active constraints examined.
    pub active_constraint_count: usize,
    /// Number of checks proved valid.
    pub certified_valid_checks: usize,
    /// Number of checks proved invalid.
    pub certified_invalid_checks: usize,
    /// Number of checks that remained unresolved.
    pub unknown_checks: usize,
    /// Number of checks blocked by unbound symbols or evaluation failures.
    pub evaluation_failure_checks: usize,
}

impl CandidateDomainReport {
    /// Returns true when every emitted check has a validity proof.
    pub fn all_checks_certified_valid(&self) -> bool {
        !self.checks.is_empty() && self.certified_valid_checks == self.checks.len()
    }

    /// Returns true when at least one check proves a domain violation.
    pub fn has_certified_invalid_domain(&self) -> bool {
        self.certified_invalid_checks > 0
    }
}

/// Certify mathematical-domain preconditions for active residual expressions.
///
/// This is a preflight certification pass. It does not replace
/// [`crate::certify_candidate`]; it explains whether candidate evaluation is in
/// domain before residual signs are interpreted. Rows with no domain-sensitive
/// operations emit no checks.
pub fn certify_candidate_domains(
    problem: &Problem,
    context: &EvaluationContext,
    policy: PredicatePolicy,
) -> CandidateDomainReport {
    let mut checks = Vec::new();
    let mut active_constraint_count = 0;
    for (constraint_index, constraint) in problem.constraints.iter().enumerate() {
        if !constraint.active {
            continue;
        }
        active_constraint_count += 1;
        collect_domain_checks(
            &constraint.residual,
            "$",
            constraint_index,
            &constraint.name,
            context,
            policy,
            &mut checks,
        );
    }
    let certified_valid_checks = checks
        .iter()
        .filter(|check| check.status.is_certified_valid())
        .count();
    let certified_invalid_checks = checks
        .iter()
        .filter(|check| check.status.is_certified_invalid())
        .count();
    let unknown_checks = checks
        .iter()
        .filter(|check| check.status.is_unknown())
        .count();
    let evaluation_failure_checks = checks
        .iter()
        .filter(|check| {
            matches!(
                check.status,
                DomainCheckStatus::UnboundSymbol { .. }
                    | DomainCheckStatus::EvaluationFailure { .. }
            )
        })
        .count();
    CandidateDomainReport {
        checks,
        active_constraint_count,
        certified_valid_checks,
        certified_invalid_checks,
        unknown_checks,
        evaluation_failure_checks,
    }
}

fn collect_domain_checks(
    expression: &Expr,
    path: &str,
    constraint_index: usize,
    constraint_name: &str,
    context: &EvaluationContext,
    policy: PredicatePolicy,
    checks: &mut Vec<DomainCheck>,
) {
    match expression {
        Expr::Constant(_) | Expr::Symbol(_) => {}
        Expr::Add(left, right)
        | Expr::Sub(left, right)
        | Expr::Mul(left, right)
        | Expr::Div(left, right) => {
            collect_domain_checks(
                left,
                &format!("{path}.left"),
                constraint_index,
                constraint_name,
                context,
                policy,
                checks,
            );
            collect_domain_checks(
                right,
                &format!("{path}.right"),
                constraint_index,
                constraint_name,
                context,
                policy,
                checks,
            );
            if let Expr::Div(_, denominator) = expression {
                push_value_check(
                    denominator,
                    &format!("{path}.right"),
                    constraint_index,
                    constraint_name,
                    DomainCheckKind::DivisionDenominatorNonZero,
                    context,
                    policy,
                    checks,
                );
            }
        }
        Expr::Neg(value) | Expr::Sin(value) | Expr::Cos(value) => {
            collect_domain_checks(
                value,
                &format!("{path}.arg"),
                constraint_index,
                constraint_name,
                context,
                policy,
                checks,
            );
        }
        Expr::PowI(value, exponent) => {
            collect_domain_checks(
                value,
                &format!("{path}.base"),
                constraint_index,
                constraint_name,
                context,
                policy,
                checks,
            );
            if *exponent < 0 {
                push_value_check(
                    value,
                    &format!("{path}.base"),
                    constraint_index,
                    constraint_name,
                    DomainCheckKind::NegativePowerBaseNonZero,
                    context,
                    policy,
                    checks,
                );
            }
        }
        Expr::Sqrt(value) => {
            collect_domain_checks(
                value,
                &format!("{path}.arg"),
                constraint_index,
                constraint_name,
                context,
                policy,
                checks,
            );
            push_value_check(
                value,
                &format!("{path}.arg"),
                constraint_index,
                constraint_name,
                DomainCheckKind::SqrtOperandNonNegative,
                context,
                policy,
                checks,
            );
        }
        Expr::Log10(value) => {
            collect_domain_checks(
                value,
                &format!("{path}.arg"),
                constraint_index,
                constraint_name,
                context,
                policy,
                checks,
            );
            push_value_check(
                value,
                &format!("{path}.arg"),
                constraint_index,
                constraint_name,
                DomainCheckKind::Log10OperandPositive,
                context,
                policy,
                checks,
            );
        }
        Expr::Ln(value) => {
            collect_domain_checks(
                value,
                &format!("{path}.arg"),
                constraint_index,
                constraint_name,
                context,
                policy,
                checks,
            );
            push_value_check(
                value,
                &format!("{path}.arg"),
                constraint_index,
                constraint_name,
                DomainCheckKind::LnOperandPositive,
                context,
                policy,
                checks,
            );
        }
        Expr::Asin(value) => {
            collect_domain_checks(
                value,
                &format!("{path}.arg"),
                constraint_index,
                constraint_name,
                context,
                policy,
                checks,
            );
            push_value_check(
                value,
                &format!("{path}.arg"),
                constraint_index,
                constraint_name,
                DomainCheckKind::AsinOperandInClosedUnitInterval,
                context,
                policy,
                checks,
            );
        }
        Expr::Acos(value) => {
            collect_domain_checks(
                value,
                &format!("{path}.arg"),
                constraint_index,
                constraint_name,
                context,
                policy,
                checks,
            );
            push_value_check(
                value,
                &format!("{path}.arg"),
                constraint_index,
                constraint_name,
                DomainCheckKind::AcosOperandInClosedUnitInterval,
                context,
                policy,
                checks,
            );
        }
        Expr::Acosh(value) => {
            collect_domain_checks(
                value,
                &format!("{path}.arg"),
                constraint_index,
                constraint_name,
                context,
                policy,
                checks,
            );
            push_value_check(
                value,
                &format!("{path}.arg"),
                constraint_index,
                constraint_name,
                DomainCheckKind::AcoshOperandAtLeastOne,
                context,
                policy,
                checks,
            );
        }
        Expr::Atanh(value) => {
            collect_domain_checks(
                value,
                &format!("{path}.arg"),
                constraint_index,
                constraint_name,
                context,
                policy,
                checks,
            );
            push_value_check(
                value,
                &format!("{path}.arg"),
                constraint_index,
                constraint_name,
                DomainCheckKind::AtanhOperandInOpenUnitInterval,
                context,
                policy,
                checks,
            );
        }
    }
}

#[allow(clippy::too_many_arguments)]
fn push_value_check(
    operand_expression: &Expr,
    path: &str,
    constraint_index: usize,
    constraint_name: &str,
    kind: DomainCheckKind,
    context: &EvaluationContext,
    policy: PredicatePolicy,
    checks: &mut Vec<DomainCheck>,
) {
    let (operand, status) = match operand_expression.eval_real(context.bindings()) {
        Ok(value) => {
            let status = classify_domain_value(&value, kind, policy);
            (Some(value), status)
        }
        Err(ExprEvalError::UnboundSymbol(symbol)) => {
            (None, DomainCheckStatus::UnboundSymbol { symbol: symbol.id })
        }
        Err(error) => (
            None,
            DomainCheckStatus::EvaluationFailure {
                message: format!("{error:?}"),
            },
        ),
    };
    checks.push(DomainCheck {
        constraint_index,
        constraint_name: constraint_name.to_string(),
        expression_path: path.to_string(),
        kind,
        operand,
        status,
    });
}

fn classify_domain_value(
    value: &Real,
    kind: DomainCheckKind,
    policy: PredicatePolicy,
) -> DomainCheckStatus {
    match kind {
        DomainCheckKind::DivisionDenominatorNonZero | DomainCheckKind::NegativePowerBaseNonZero => {
            match compare_reals_with_policy(value, &Real::zero(), policy).value() {
                Some(Ordering::Less | Ordering::Greater) => DomainCheckStatus::CertifiedValid,
                Some(Ordering::Equal) => DomainCheckStatus::CertifiedInvalid,
                None => DomainCheckStatus::Unknown,
            }
        }
        DomainCheckKind::SqrtOperandNonNegative => {
            match compare_reals_with_policy(value, &Real::zero(), policy).value() {
                Some(Ordering::Equal | Ordering::Greater) => DomainCheckStatus::CertifiedValid,
                Some(Ordering::Less) => DomainCheckStatus::CertifiedInvalid,
                None => DomainCheckStatus::Unknown,
            }
        }
        DomainCheckKind::Log10OperandPositive | DomainCheckKind::LnOperandPositive => {
            match compare_reals_with_policy(value, &Real::zero(), policy).value() {
                Some(Ordering::Greater) => DomainCheckStatus::CertifiedValid,
                Some(Ordering::Less | Ordering::Equal) => DomainCheckStatus::CertifiedInvalid,
                None => DomainCheckStatus::Unknown,
            }
        }
        DomainCheckKind::AsinOperandInClosedUnitInterval
        | DomainCheckKind::AcosOperandInClosedUnitInterval => {
            classify_closed_unit_interval(value, policy)
        }
        DomainCheckKind::AcoshOperandAtLeastOne => {
            match compare_reals_with_policy(value, &Real::one(), policy).value() {
                Some(Ordering::Equal | Ordering::Greater) => DomainCheckStatus::CertifiedValid,
                Some(Ordering::Less) => DomainCheckStatus::CertifiedInvalid,
                None => DomainCheckStatus::Unknown,
            }
        }
        DomainCheckKind::AtanhOperandInOpenUnitInterval => {
            classify_open_unit_interval(value, policy)
        }
    }
}

fn classify_closed_unit_interval(value: &Real, policy: PredicatePolicy) -> DomainCheckStatus {
    let lower = compare_reals_with_policy(value, &(-Real::one()), policy).value();
    let upper = compare_reals_with_policy(value, &Real::one(), policy).value();
    match (lower, upper) {
        (Some(Ordering::Less), _) | (_, Some(Ordering::Greater)) => {
            DomainCheckStatus::CertifiedInvalid
        }
        (Some(Ordering::Equal | Ordering::Greater), Some(Ordering::Less | Ordering::Equal)) => {
            DomainCheckStatus::CertifiedValid
        }
        _ => DomainCheckStatus::Unknown,
    }
}

fn classify_open_unit_interval(value: &Real, policy: PredicatePolicy) -> DomainCheckStatus {
    let lower = compare_reals_with_policy(value, &(-Real::one()), policy).value();
    let upper = compare_reals_with_policy(value, &Real::one(), policy).value();
    match (lower, upper) {
        (Some(Ordering::Greater), Some(Ordering::Less)) => DomainCheckStatus::CertifiedValid,
        (Some(Ordering::Less | Ordering::Equal), _)
        | (_, Some(Ordering::Equal | Ordering::Greater)) => DomainCheckStatus::CertifiedInvalid,
        _ => DomainCheckStatus::Unknown,
    }
}
