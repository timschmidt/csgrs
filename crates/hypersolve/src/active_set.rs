//! Exact active-set audits for solver candidates.
//!
//! Active-set methods are useful proposal strategies, but the active mask is
//! not proof. This module audits the caller's active/inactive constraint flags
//! by replaying every residual exactly and certifying the signed residual
//! status before accepting the mask. Equality and soft rows are expected to be
//! active. Inequalities are expected to be active exactly when binding and
//! inactive exactly when strictly satisfied. Violations are reported regardless
//! of the mask. This keeps active-set decisions inside Yap's exact decision
//! boundary; see Yap, "Towards Exact Geometric Computation,"
//! *Computational Geometry* 7.1-2 (1997). The active-set terminology follows
//! the standard nonlinear-programming/KKT convention; see Nocedal and Wright,
//! *Numerical Optimization*, 2nd ed. (2006).

use hyperreal::{CertifiedRealSign, Real, RealSign, RealSignCertificate};

use crate::certification::CandidateCertificationConfig;
use crate::eval::EvaluationContext;
use crate::model::ConstraintKind;
use crate::prepared::PreparedProblem;

/// Certified relationship between one constraint and the supplied active mask.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum ActiveSetRowStatus {
    /// Equality or soft row is active and certified zero.
    RequiredActiveSatisfied {
        /// Scalar sign proof for the normalized residual.
        certificate: RealSignCertificate,
    },
    /// Equality or soft row was inactive even though it participates in the
    /// active solve model.
    RequiredActiveMissing {
        /// Certified sign if residual sign was decided.
        sign: Option<RealSign>,
    },
    /// Inequality row is active and certified binding.
    ActiveBinding {
        /// Scalar sign proof for the normalized residual.
        certificate: RealSignCertificate,
    },
    /// Inequality row is inactive and certified strictly satisfied.
    InactiveStrictlySatisfied {
        /// Scalar sign proof for the normalized residual.
        certificate: RealSignCertificate,
    },
    /// Inequality row is active even though it is certified strictly satisfied.
    SuperfluousActiveInequality {
        /// Certified strict sign of the normalized residual.
        sign: RealSign,
        /// Scalar sign proof for the normalized residual.
        certificate: RealSignCertificate,
    },
    /// Inequality row is inactive even though it is certified binding.
    MissedBindingInequality {
        /// Scalar sign proof for the normalized residual.
        certificate: RealSignCertificate,
    },
    /// Constraint residual is certified violated under its row kind.
    CertifiedViolation {
        /// Certified sign of the normalized residual.
        sign: RealSign,
        /// Scalar sign proof for the normalized residual.
        certificate: RealSignCertificate,
    },
    /// Exact residual replay completed, but sign certification did not decide.
    BoundedUnknown {
        /// Precision floor used for the bounded proof attempt.
        min_precision: i32,
    },
    /// Residual evaluation failed before active-set classification.
    DomainFailure {
        /// Compact diagnostic for the expression/domain failure.
        message: String,
    },
}

impl ActiveSetRowStatus {
    /// Returns true when this row's active flag is certified consistent.
    pub const fn is_consistent(&self) -> bool {
        matches!(
            self,
            Self::RequiredActiveSatisfied { .. }
                | Self::ActiveBinding { .. }
                | Self::InactiveStrictlySatisfied { .. }
        )
    }

    /// Returns true when the row is a certified mask mismatch.
    pub const fn is_mask_mismatch(&self) -> bool {
        matches!(
            self,
            Self::RequiredActiveMissing { .. }
                | Self::SuperfluousActiveInequality { .. }
                | Self::MissedBindingInequality { .. }
        )
    }

    /// Returns true when the row is a certified residual violation.
    pub const fn is_violation(&self) -> bool {
        matches!(self, Self::CertifiedViolation { .. })
    }

    /// Returns true when exact replay could not decide the row.
    pub const fn is_unknown(&self) -> bool {
        matches!(self, Self::BoundedUnknown { .. })
    }
}

/// Active-set audit row for one source constraint.
#[derive(Clone, Debug, PartialEq)]
pub struct ActiveSetAuditRow {
    /// Source constraint index.
    pub constraint_index: usize,
    /// Constraint name copied for diagnostics.
    pub name: String,
    /// Constraint kind used to normalize the residual.
    pub kind: ConstraintKind,
    /// Active flag supplied by the caller/problem.
    pub active: bool,
    /// Exact signed residual after inequality normalization.
    pub signed_residual: Option<Real>,
    /// Certified row status.
    pub status: ActiveSetRowStatus,
}

/// Exact audit report for a candidate active set.
#[derive(Clone, Debug, PartialEq)]
pub struct ActiveSetAuditReport {
    /// One row per source constraint, including inactive constraints.
    pub rows: Vec<ActiveSetAuditRow>,
    /// Rows whose active flag and residual status are certified consistent.
    pub consistent_rows: usize,
    /// Rows whose active flag is certified inconsistent.
    pub mask_mismatch_rows: usize,
    /// Rows whose residual is certified violated.
    pub violation_rows: usize,
    /// Rows whose sign could not be certified.
    pub unknown_rows: usize,
    /// Rows that failed during exact residual replay.
    pub domain_failure_rows: usize,
}

/// Exact active-set update action for one row.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum ActiveSetUpdateAction {
    /// The current active flag is certified appropriate.
    Keep,
    /// The next proposal should mark this row active.
    Activate,
    /// The next proposal should mark this inequality row inactive.
    Deactivate,
    /// The candidate is infeasible for this row, so active-set mutation is not
    /// a proof-producing fix.
    RejectCandidate,
    /// Exact replay could not decide the row.
    Unknown,
    /// Residual evaluation failed before update classification.
    DomainFailure,
}

/// Proposed active-state update for one constraint row.
#[derive(Clone, Debug, PartialEq)]
pub struct ActiveSetUpdateRow {
    /// Source constraint index.
    pub constraint_index: usize,
    /// Constraint name copied for diagnostics.
    pub name: String,
    /// Constraint kind used by the audit.
    pub kind: ConstraintKind,
    /// Active flag supplied by the current model.
    pub current_active: bool,
    /// Proposed active flag when the policy can safely choose one.
    pub proposed_active: Option<bool>,
    /// Exact update action.
    pub action: ActiveSetUpdateAction,
    /// Source audit status that justified the action.
    pub audit_status: ActiveSetRowStatus,
}

/// Exact active-set update proposal.
#[derive(Clone, Debug, PartialEq)]
pub struct ActiveSetUpdateReport {
    /// Full audit that the proposal consumed.
    pub audit: ActiveSetAuditReport,
    /// One proposed update per source constraint.
    pub rows: Vec<ActiveSetUpdateRow>,
    /// Complete proposed mask when no row rejected or remained unknown.
    pub proposed_active_mask: Option<Vec<bool>>,
    /// Number of rows proposed for activation.
    pub activate_rows: usize,
    /// Number of rows proposed for deactivation.
    pub deactivate_rows: usize,
    /// Number of rows whose current flag is retained.
    pub keep_rows: usize,
    /// Number of rows proving candidate infeasibility.
    pub rejected_rows: usize,
    /// Number of rows with undecided sign evidence.
    pub unknown_rows: usize,
    /// Number of rows with domain/evaluation failures.
    pub domain_failure_rows: usize,
}

/// Final status for an exact active-set update loop.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum ActiveSetLoopStatus {
    /// The loop reached a mask whose rows are all certified consistent.
    Stable,
    /// A row certified the candidate infeasible.
    RejectedCandidate,
    /// Exact replay left at least one row undecided.
    Unknown,
    /// Residual evaluation failed for at least one row.
    DomainFailure,
    /// The loop used the configured iteration budget before stabilizing.
    IterationLimit,
    /// The supplied initial mask did not match the constraint count.
    InvalidInitialMask,
}

/// Configuration for exact active-set update loops.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ActiveSetLoopConfig {
    /// Candidate-certification controls used by each exact replay.
    pub certification: CandidateCertificationConfig,
    /// Maximum number of exact audit/update iterations.
    pub max_iterations: usize,
}

impl Default for ActiveSetLoopConfig {
    fn default() -> Self {
        Self {
            certification: CandidateCertificationConfig::default(),
            max_iterations: 8,
        }
    }
}

/// One exact active-set loop iteration.
#[derive(Clone, Debug, PartialEq)]
pub struct ActiveSetLoopIteration {
    /// Iteration index.
    pub iteration: usize,
    /// Mask audited at the beginning of this iteration.
    pub input_mask: Vec<bool>,
    /// Exact update proposal emitted for the input mask.
    pub update: ActiveSetUpdateReport,
}

/// Exact active-set loop report.
#[derive(Clone, Debug, PartialEq)]
pub struct ActiveSetLoopReport {
    /// Final loop status.
    pub status: ActiveSetLoopStatus,
    /// Iteration reports, each backed by exact residual replay.
    pub iterations: Vec<ActiveSetLoopIteration>,
    /// Final stable or last proposed active mask, when available.
    pub final_mask: Option<Vec<bool>>,
}

impl ActiveSetUpdateReport {
    /// Returns true when the policy produced a complete next active mask.
    pub fn has_complete_mask(&self) -> bool {
        self.proposed_active_mask.is_some()
    }
}

impl ActiveSetAuditReport {
    /// Returns true when every row has a certified consistent active-set state.
    pub fn all_consistent(&self) -> bool {
        !self.rows.is_empty() && self.consistent_rows == self.rows.len()
    }

    /// Returns true when any mask mismatch or violation was proved.
    pub fn has_certified_problem(&self) -> bool {
        self.mask_mismatch_rows > 0 || self.violation_rows > 0 || self.domain_failure_rows > 0
    }
}

/// Propose an exact active-set mask update from candidate replay.
///
/// This is still not an active-set solver. It is the policy layer that turns
/// exact audit rows into a next-mask proposal: inactive required rows and
/// missed binding inequalities activate, superfluous active inequalities
/// deactivate, consistent rows keep their current flag, and certified
/// violations reject the candidate. That mirrors the active-set/KKT
/// terminology in Nocedal and Wright, *Numerical Optimization*, 2nd ed.
/// (2006), while following Yap's "Towards Exact Geometric Computation" (1997)
/// rule that only certified residual signs may drive branching.
pub fn propose_active_set_update(
    prepared: &PreparedProblem<'_>,
    context: &EvaluationContext,
    config: CandidateCertificationConfig,
) -> ActiveSetUpdateReport {
    let audit = audit_active_set(prepared, context, config);
    active_set_update_report_from_audit(audit)
}

/// Run an exact active-set update loop without mutating the source problem.
///
/// Each iteration audits a candidate active mask by exact residual replay, then
/// applies the same update policy used by [`propose_active_set_update`].
/// Certified missed bindings activate, certified superfluous inequalities
/// deactivate, and certified violations stop the loop as infeasible evidence.
/// This is the smallest solver-loop layer over the exact mask policy: it
/// follows the active-set/KKT iteration idea described by Nocedal and Wright,
/// *Numerical Optimization*, 2nd ed. (2006), while preserving Yap's "Towards
/// Exact Geometric Computation" (1997) boundary that exact reports, not
/// floating tolerances, drive every branch.
pub fn run_active_set_update_loop(
    prepared: &PreparedProblem<'_>,
    context: &EvaluationContext,
    initial_mask: &[bool],
    config: ActiveSetLoopConfig,
) -> ActiveSetLoopReport {
    if initial_mask.len() != prepared.problem().constraints.len() {
        return ActiveSetLoopReport {
            status: ActiveSetLoopStatus::InvalidInitialMask,
            iterations: Vec::new(),
            final_mask: None,
        };
    }

    let mut mask = initial_mask.to_vec();
    let mut iterations = Vec::new();
    for iteration in 0..config.max_iterations {
        let update = propose_active_mask_update(prepared, context, &mask, config.certification);
        let next_mask = update.proposed_active_mask.clone();
        let status = if update.rejected_rows > 0 {
            Some(ActiveSetLoopStatus::RejectedCandidate)
        } else if update.domain_failure_rows > 0 {
            Some(ActiveSetLoopStatus::DomainFailure)
        } else if update.unknown_rows > 0 {
            Some(ActiveSetLoopStatus::Unknown)
        } else if update.audit.all_consistent() {
            Some(ActiveSetLoopStatus::Stable)
        } else {
            None
        };
        iterations.push(ActiveSetLoopIteration {
            iteration,
            input_mask: mask.clone(),
            update,
        });
        if let Some(status) = status {
            let final_mask = if status == ActiveSetLoopStatus::Stable {
                next_mask.or(Some(mask))
            } else {
                None
            };
            return ActiveSetLoopReport {
                status,
                iterations,
                final_mask,
            };
        }
        let Some(next_mask) = next_mask else {
            return ActiveSetLoopReport {
                status: ActiveSetLoopStatus::Unknown,
                iterations,
                final_mask: None,
            };
        };
        if next_mask == mask {
            return ActiveSetLoopReport {
                status: ActiveSetLoopStatus::Stable,
                iterations,
                final_mask: Some(mask),
            };
        }
        mask = next_mask;
    }

    ActiveSetLoopReport {
        status: ActiveSetLoopStatus::IterationLimit,
        iterations,
        final_mask: Some(mask),
    }
}

/// Audit every source constraint against its supplied active flag.
///
/// This does not choose a new active set and does not run an active-set solver.
/// It is a proof-producing boundary for an active mask produced elsewhere:
/// exact residual replay certifies whether the mask is consistent, mismatched,
/// violated, or undecidable.
pub fn audit_active_set(
    prepared: &PreparedProblem<'_>,
    context: &EvaluationContext,
    config: CandidateCertificationConfig,
) -> ActiveSetAuditReport {
    let active_mask = prepared
        .problem()
        .constraints
        .iter()
        .map(|constraint| constraint.active)
        .collect::<Vec<_>>();
    audit_active_mask(prepared, context, &active_mask, config)
}

/// Audit every source constraint against an explicit active mask.
///
/// This is the non-mutating companion to [`audit_active_set`]. The supplied
/// mask must have one entry per source constraint; otherwise every row is
/// reported as a domain failure because no exact row-to-mask relationship can
/// be established.
pub fn audit_active_mask(
    prepared: &PreparedProblem<'_>,
    context: &EvaluationContext,
    active_mask: &[bool],
    config: CandidateCertificationConfig,
) -> ActiveSetAuditReport {
    if active_mask.len() != prepared.problem().constraints.len() {
        return ActiveSetAuditReport {
            rows: prepared
                .problem()
                .constraints
                .iter()
                .enumerate()
                .map(|(constraint_index, constraint)| ActiveSetAuditRow {
                    constraint_index,
                    name: constraint.name.clone(),
                    kind: constraint.kind,
                    active: false,
                    signed_residual: None,
                    status: ActiveSetRowStatus::DomainFailure {
                        message: "active mask length does not match constraint count".to_owned(),
                    },
                })
                .collect(),
            consistent_rows: 0,
            mask_mismatch_rows: 0,
            violation_rows: 0,
            unknown_rows: 0,
            domain_failure_rows: prepared.problem().constraints.len(),
        };
    }

    let mut rows = Vec::with_capacity(prepared.problem().constraints.len());
    for (constraint_index, constraint) in prepared.problem().constraints.iter().enumerate() {
        let active = active_mask[constraint_index];
        let row = match prepared.evaluate_constraint_residual(constraint_index, context) {
            Ok(value) => {
                let signed_residual = normalize_residual(value, constraint.kind);
                let status =
                    classify_active_set_row(constraint.kind, active, &signed_residual, config);
                ActiveSetAuditRow {
                    constraint_index,
                    name: constraint.name.clone(),
                    kind: constraint.kind,
                    active,
                    signed_residual: Some(signed_residual),
                    status,
                }
            }
            Err(error) => ActiveSetAuditRow {
                constraint_index,
                name: constraint.name.clone(),
                kind: constraint.kind,
                active,
                signed_residual: None,
                status: ActiveSetRowStatus::DomainFailure {
                    message: format!("{error:?}"),
                },
            },
        };
        rows.push(row);
    }

    let consistent_rows = rows.iter().filter(|row| row.status.is_consistent()).count();
    let mask_mismatch_rows = rows
        .iter()
        .filter(|row| row.status.is_mask_mismatch())
        .count();
    let violation_rows = rows.iter().filter(|row| row.status.is_violation()).count();
    let unknown_rows = rows.iter().filter(|row| row.status.is_unknown()).count();
    let domain_failure_rows = rows
        .iter()
        .filter(|row| matches!(row.status, ActiveSetRowStatus::DomainFailure { .. }))
        .count();

    ActiveSetAuditReport {
        rows,
        consistent_rows,
        mask_mismatch_rows,
        violation_rows,
        unknown_rows,
        domain_failure_rows,
    }
}

fn propose_active_mask_update(
    prepared: &PreparedProblem<'_>,
    context: &EvaluationContext,
    active_mask: &[bool],
    config: CandidateCertificationConfig,
) -> ActiveSetUpdateReport {
    let audit = audit_active_mask(prepared, context, active_mask, config);
    active_set_update_report_from_audit(audit)
}

fn active_set_update_report_from_audit(audit: ActiveSetAuditReport) -> ActiveSetUpdateReport {
    let rows = audit
        .rows
        .iter()
        .map(active_set_update_row)
        .collect::<Vec<_>>();
    let activate_rows = rows
        .iter()
        .filter(|row| row.action == ActiveSetUpdateAction::Activate)
        .count();
    let deactivate_rows = rows
        .iter()
        .filter(|row| row.action == ActiveSetUpdateAction::Deactivate)
        .count();
    let keep_rows = rows
        .iter()
        .filter(|row| row.action == ActiveSetUpdateAction::Keep)
        .count();
    let rejected_rows = rows
        .iter()
        .filter(|row| row.action == ActiveSetUpdateAction::RejectCandidate)
        .count();
    let unknown_rows = rows
        .iter()
        .filter(|row| row.action == ActiveSetUpdateAction::Unknown)
        .count();
    let domain_failure_rows = rows
        .iter()
        .filter(|row| row.action == ActiveSetUpdateAction::DomainFailure)
        .count();
    let proposed_active_mask = rows
        .iter()
        .map(|row| row.proposed_active)
        .collect::<Option<Vec<_>>>();

    ActiveSetUpdateReport {
        audit,
        rows,
        proposed_active_mask,
        activate_rows,
        deactivate_rows,
        keep_rows,
        rejected_rows,
        unknown_rows,
        domain_failure_rows,
    }
}

fn active_set_update_row(row: &ActiveSetAuditRow) -> ActiveSetUpdateRow {
    let (action, proposed_active) = match &row.status {
        ActiveSetRowStatus::RequiredActiveSatisfied { .. }
        | ActiveSetRowStatus::ActiveBinding { .. }
        | ActiveSetRowStatus::InactiveStrictlySatisfied { .. } => {
            (ActiveSetUpdateAction::Keep, Some(row.active))
        }
        ActiveSetRowStatus::RequiredActiveMissing { .. }
        | ActiveSetRowStatus::MissedBindingInequality { .. } => {
            (ActiveSetUpdateAction::Activate, Some(true))
        }
        ActiveSetRowStatus::SuperfluousActiveInequality { .. } => {
            (ActiveSetUpdateAction::Deactivate, Some(false))
        }
        ActiveSetRowStatus::CertifiedViolation { .. } => {
            (ActiveSetUpdateAction::RejectCandidate, None)
        }
        ActiveSetRowStatus::BoundedUnknown { .. } => (ActiveSetUpdateAction::Unknown, None),
        ActiveSetRowStatus::DomainFailure { .. } => (ActiveSetUpdateAction::DomainFailure, None),
    };

    ActiveSetUpdateRow {
        constraint_index: row.constraint_index,
        name: row.name.clone(),
        kind: row.kind,
        current_active: row.active,
        proposed_active,
        action,
        audit_status: row.status.clone(),
    }
}

fn classify_active_set_row(
    kind: ConstraintKind,
    active: bool,
    signed_residual: &Real,
    config: CandidateCertificationConfig,
) -> ActiveSetRowStatus {
    let CertifiedRealSign::Known { sign, certificate } =
        signed_residual.certified_sign_until(config.min_precision)
    else {
        return ActiveSetRowStatus::BoundedUnknown {
            min_precision: config.min_precision,
        };
    };

    match kind {
        ConstraintKind::Equality | ConstraintKind::Soft => match (active, sign) {
            (true, RealSign::Zero) => ActiveSetRowStatus::RequiredActiveSatisfied { certificate },
            (true, RealSign::Negative | RealSign::Positive) => {
                ActiveSetRowStatus::CertifiedViolation { sign, certificate }
            }
            (false, _) => ActiveSetRowStatus::RequiredActiveMissing { sign: Some(sign) },
        },
        ConstraintKind::LessOrEqual | ConstraintKind::GreaterOrEqual => match (active, sign) {
            (_, RealSign::Positive) => ActiveSetRowStatus::CertifiedViolation { sign, certificate },
            (true, RealSign::Zero) => ActiveSetRowStatus::ActiveBinding { certificate },
            (false, RealSign::Zero) => ActiveSetRowStatus::MissedBindingInequality { certificate },
            (true, RealSign::Negative) => {
                ActiveSetRowStatus::SuperfluousActiveInequality { sign, certificate }
            }
            (false, RealSign::Negative) => {
                ActiveSetRowStatus::InactiveStrictlySatisfied { certificate }
            }
        },
    }
}

fn normalize_residual(value: Real, kind: ConstraintKind) -> Real {
    match kind {
        ConstraintKind::Equality | ConstraintKind::LessOrEqual | ConstraintKind::Soft => value,
        ConstraintKind::GreaterOrEqual => -value,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::{Constraint, Problem};
    use crate::symbolic::{Expr, SymbolId};

    fn real(value: i64) -> Real {
        Real::from(value)
    }

    #[test]
    fn active_set_audit_accepts_binding_and_inactive_strict_inequalities() {
        let x = Expr::symbol(SymbolId(0), "x");
        let mut problem = Problem::default();
        problem.add_variable("x", real(0));
        let mut binding = Constraint::equality("binding inequality", x.clone());
        binding.kind = ConstraintKind::LessOrEqual;
        binding.active = true;
        problem.add_constraint(binding);
        let mut inactive = Constraint::equality("inactive strict inequality", x - Expr::int(1));
        inactive.kind = ConstraintKind::LessOrEqual;
        inactive.active = false;
        problem.add_constraint(inactive);

        let report = audit_active_set(
            &PreparedProblem::new(&problem),
            &crate::eval::context_from_problem(&problem),
            CandidateCertificationConfig::default(),
        );

        assert!(report.all_consistent());
        assert_eq!(report.rows.len(), 2);
        assert!(matches!(
            report.rows[0].status,
            ActiveSetRowStatus::ActiveBinding { .. }
        ));
        assert!(matches!(
            report.rows[1].status,
            ActiveSetRowStatus::InactiveStrictlySatisfied { .. }
        ));
    }

    #[test]
    fn active_set_audit_reports_mask_mismatches_and_violations() {
        let x = Expr::symbol(SymbolId(0), "x");
        let mut problem = Problem::default();
        problem.add_variable("x", real(0));
        let mut inactive_equality = Constraint::equality("inactive equality", x.clone());
        inactive_equality.active = false;
        problem.add_constraint(inactive_equality);
        let mut missed_binding = Constraint::equality("missed binding", x.clone());
        missed_binding.kind = ConstraintKind::GreaterOrEqual;
        missed_binding.active = false;
        problem.add_constraint(missed_binding);
        let mut superfluous = Constraint::equality("superfluous active", x.clone() - Expr::int(1));
        superfluous.kind = ConstraintKind::LessOrEqual;
        superfluous.active = true;
        problem.add_constraint(superfluous);
        let mut violation = Constraint::equality("violated inequality", x - Expr::int(1));
        violation.kind = ConstraintKind::GreaterOrEqual;
        violation.active = true;
        problem.add_constraint(violation);

        let report = audit_active_set(
            &PreparedProblem::new(&problem),
            &crate::eval::context_from_problem(&problem),
            CandidateCertificationConfig::default(),
        );

        assert!(report.has_certified_problem());
        assert_eq!(report.mask_mismatch_rows, 3);
        assert_eq!(report.violation_rows, 1);
        assert!(matches!(
            report.rows[0].status,
            ActiveSetRowStatus::RequiredActiveMissing { .. }
        ));
        assert!(matches!(
            report.rows[1].status,
            ActiveSetRowStatus::MissedBindingInequality { .. }
        ));
        assert!(matches!(
            report.rows[2].status,
            ActiveSetRowStatus::SuperfluousActiveInequality { .. }
        ));
        assert!(matches!(
            report.rows[3].status,
            ActiveSetRowStatus::CertifiedViolation { .. }
        ));
    }

    #[test]
    fn active_set_update_proposes_exact_mask_repairs_without_mutation() {
        let x = Expr::symbol(SymbolId(0), "x");
        let mut problem = Problem::default();
        problem.add_variable("x", real(0));
        let mut missed_binding = Constraint::equality("missed binding", x.clone());
        missed_binding.kind = ConstraintKind::LessOrEqual;
        missed_binding.active = false;
        problem.add_constraint(missed_binding);
        let mut superfluous = Constraint::equality("superfluous active", x.clone() - Expr::int(1));
        superfluous.kind = ConstraintKind::LessOrEqual;
        superfluous.active = true;
        problem.add_constraint(superfluous);
        let mut keep = Constraint::equality("kept equality", x);
        keep.active = true;
        problem.add_constraint(keep);

        let report = propose_active_set_update(
            &PreparedProblem::new(&problem),
            &crate::eval::context_from_problem(&problem),
            CandidateCertificationConfig::default(),
        );

        assert_eq!(report.activate_rows, 1);
        assert_eq!(report.deactivate_rows, 1);
        assert_eq!(report.keep_rows, 1);
        assert_eq!(report.proposed_active_mask, Some(vec![true, false, true]));
        assert_eq!(problem.constraints[0].active, false);
        assert_eq!(problem.constraints[1].active, true);
    }

    #[test]
    fn active_set_update_rejects_violated_candidate_instead_of_repairing() {
        let x = Expr::symbol(SymbolId(0), "x");
        let mut problem = Problem::default();
        problem.add_variable("x", real(0));
        let mut violated = Constraint::equality("violated lower bound", x - Expr::int(1));
        violated.kind = ConstraintKind::GreaterOrEqual;
        violated.active = true;
        problem.add_constraint(violated);

        let report = propose_active_set_update(
            &PreparedProblem::new(&problem),
            &crate::eval::context_from_problem(&problem),
            CandidateCertificationConfig::default(),
        );

        assert_eq!(report.rejected_rows, 1);
        assert_eq!(report.proposed_active_mask, None);
        assert_eq!(
            report.rows[0].action,
            ActiveSetUpdateAction::RejectCandidate
        );
    }

    #[test]
    fn active_set_loop_stabilizes_mask_without_mutating_problem() {
        let x = Expr::symbol(SymbolId(0), "x");
        let mut problem = Problem::default();
        problem.add_variable("x", real(0));
        let mut binding = Constraint::equality("binding", x.clone());
        binding.kind = ConstraintKind::LessOrEqual;
        binding.active = false;
        problem.add_constraint(binding);
        let mut strict = Constraint::equality("strict", x - Expr::int(1));
        strict.kind = ConstraintKind::LessOrEqual;
        strict.active = true;
        problem.add_constraint(strict);

        let report = run_active_set_update_loop(
            &PreparedProblem::new(&problem),
            &crate::eval::context_from_problem(&problem),
            &[false, true],
            ActiveSetLoopConfig::default(),
        );

        assert_eq!(report.status, ActiveSetLoopStatus::Stable);
        assert_eq!(report.final_mask, Some(vec![true, false]));
        assert_eq!(report.iterations.len(), 2);
        assert_eq!(problem.constraints[0].active, false);
        assert_eq!(problem.constraints[1].active, true);
    }

    #[test]
    fn active_set_loop_rejects_bad_masks_and_violated_candidates() {
        let x = Expr::symbol(SymbolId(0), "x");
        let mut problem = Problem::default();
        problem.add_variable("x", real(0));
        let mut violated = Constraint::equality("violated lower bound", x - Expr::int(1));
        violated.kind = ConstraintKind::GreaterOrEqual;
        violated.active = true;
        problem.add_constraint(violated);
        let prepared = PreparedProblem::new(&problem);
        let context = crate::eval::context_from_problem(&problem);

        let invalid =
            run_active_set_update_loop(&prepared, &context, &[], ActiveSetLoopConfig::default());
        assert_eq!(invalid.status, ActiveSetLoopStatus::InvalidInitialMask);
        assert_eq!(invalid.final_mask, None);

        let rejected = run_active_set_update_loop(
            &prepared,
            &context,
            &[true],
            ActiveSetLoopConfig::default(),
        );
        assert_eq!(rejected.status, ActiveSetLoopStatus::RejectedCandidate);
        assert_eq!(rejected.final_mask, None);
    }

    proptest::proptest! {
        #[test]
        fn generated_less_equal_active_masks_follow_exact_sign(
            residual in -64_i16..=64,
            active in proptest::bool::ANY,
        ) {
            let residual = i64::from(residual);
            let mut problem = Problem::default();
            problem.add_variable("x", real(0));
            let mut constraint = Constraint::equality("generated inequality", Expr::int(residual));
            constraint.kind = ConstraintKind::LessOrEqual;
            constraint.active = active;
            problem.add_constraint(constraint);

            let report = audit_active_set(
                &PreparedProblem::new(&problem),
                &crate::eval::context_from_problem(&problem),
                CandidateCertificationConfig::default(),
            );

            let status = &report.rows[0].status;
            match (active, residual.cmp(&0)) {
                (_, std::cmp::Ordering::Greater) => {
                    assert!(matches!(status, ActiveSetRowStatus::CertifiedViolation { .. }));
                }
                (true, std::cmp::Ordering::Equal) => {
                    assert!(matches!(status, ActiveSetRowStatus::ActiveBinding { .. }));
                }
                (false, std::cmp::Ordering::Equal) => {
                    assert!(matches!(status, ActiveSetRowStatus::MissedBindingInequality { .. }));
                }
                (true, std::cmp::Ordering::Less) => {
                    assert!(matches!(status, ActiveSetRowStatus::SuperfluousActiveInequality { .. }));
                }
                (false, std::cmp::Ordering::Less) => {
                    assert!(matches!(status, ActiveSetRowStatus::InactiveStrictlySatisfied { .. }));
                }
            }
        }
    }
}
