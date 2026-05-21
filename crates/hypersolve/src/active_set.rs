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
    let mut rows = Vec::with_capacity(prepared.problem().constraints.len());
    for (constraint_index, constraint) in prepared.problem().constraints.iter().enumerate() {
        let row = match prepared.evaluate_constraint_residual(constraint_index, context) {
            Ok(value) => {
                let signed_residual = normalize_residual(value, constraint.kind);
                let status = classify_active_set_row(
                    constraint.kind,
                    constraint.active,
                    &signed_residual,
                    config,
                );
                ActiveSetAuditRow {
                    constraint_index,
                    name: constraint.name.clone(),
                    kind: constraint.kind,
                    active: constraint.active,
                    signed_residual: Some(signed_residual),
                    status,
                }
            }
            Err(error) => ActiveSetAuditRow {
                constraint_index,
                name: constraint.name.clone(),
                kind: constraint.kind,
                active: constraint.active,
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
