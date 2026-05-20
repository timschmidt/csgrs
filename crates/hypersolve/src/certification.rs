//! Exact candidate replay and certification for solver results.
//!
//! Numeric nonlinear methods are useful proposal engines, but in an exact
//! geometry stack the proposed coordinates are not a proof. This module replays
//! residuals through prepared exact solver structure and asks `hyperreal` for
//! certified signs before accepting or rejecting each row. That follows Yap's
//! "Towards Exact Geometric Computation" (*Computational Geometry* 7.1-2,
//! 1997): combinatorial decisions should consume exact/certified information
//! or return explicit uncertainty, not primitive-float tolerances. The
//! candidate/proof split mirrors the architecture of SolveSpace's symbolic
//! residual/Jacobian layer with numerical Newton iteration, but makes the
//! post-iteration trust boundary explicit.

use hyperlimit::{PredicatePolicy, Sign, certified_ball_sign_report_with_policy};
use hyperreal::{CertifiedRealSign, Real, RealSign, RealSignCertificate};

use crate::diagnostics::{ProposalEngineKind, ProposalEnginePrecision, ProposalEngineReport};
use crate::eval::{EvalError, EvaluationContext, positive_part};
use crate::model::ConstraintKind;
use crate::prepared::PreparedProblem;

/// Certification policy for replaying one candidate solution.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct CandidateCertificationConfig {
    /// Lowest binary precision allowed for bounded exact-real sign refinement.
    ///
    /// This is not a tolerance. It is the explicit refinement floor passed to
    /// `Real::certified_sign_until` when structural facts alone do not decide a
    /// residual sign.
    pub min_precision: i32,
}

impl Default for CandidateCertificationConfig {
    fn default() -> Self {
        Self {
            min_precision: -2048,
        }
    }
}

/// Per-row certification status for a candidate solution.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum CertifiedCandidateStatus {
    /// An equality or soft row was proved exactly zero.
    CertifiedZero {
        /// Scalar proof source.
        certificate: RealSignCertificate,
    },
    /// An inequality row was proved satisfied.
    CertifiedSatisfiedInequality {
        /// Scalar proof source for the signed residual.
        certificate: RealSignCertificate,
    },
    /// A row was proved violated.
    CertifiedViolation {
        /// Proven sign of the signed residual after row-kind normalization.
        sign: RealSign,
        /// Scalar proof source.
        certificate: RealSignCertificate,
    },
    /// Exact replay completed, but bounded sign certification did not decide.
    BoundedUnknown {
        /// Precision floor used for the bounded proof attempt.
        min_precision: i32,
    },
    /// A residual ball enclosure certified the row.
    BallCertified {
        /// Sign proved for every value in the ball.
        sign: RealSign,
    },
    /// The residual ball had a negative or otherwise unsupported radius.
    InvalidBallRadius,
    /// Residual evaluation failed before a scalar sign proof could be asked.
    DomainFailure {
        /// Compact diagnostic for the expression/domain failure.
        message: String,
    },
    /// A proposal engine produced candidate data, but no exact replay proof was
    /// requested for this row.
    ///
    /// This is an explicit non-certificate. It records the selected numerical
    /// adapter boundary so callers cannot accidentally treat primitive-float
    /// convergence as exact residual truth. That separation is Yap's core
    /// exact-geometric-computation rule applied to solver outputs; see Yap,
    /// "Towards Exact Geometric Computation," *Computational Geometry* 7.1-2
    /// (1997).
    LossyAdapterOnly {
        /// Proposal engine requested by the caller.
        requested: ProposalEngineKind,
        /// Proposal engine actually used, if any.
        used: Option<ProposalEngineKind>,
        /// Precision boundary crossed by the proposal route.
        precision: ProposalEnginePrecision,
    },
}

impl CertifiedCandidateStatus {
    /// Returns whether this row is proved acceptable under its constraint kind.
    pub const fn is_certified_satisfied(&self) -> bool {
        matches!(
            self,
            Self::CertifiedZero { .. }
                | Self::CertifiedSatisfiedInequality { .. }
                | Self::BallCertified {
                    sign: RealSign::Zero,
                }
        )
    }

    /// Returns whether this row is proved unacceptable.
    pub const fn is_certified_violation(&self) -> bool {
        matches!(
            self,
            Self::CertifiedViolation { .. }
                | Self::BallCertified {
                    sign: RealSign::Positive | RealSign::Negative,
                }
        )
    }

    /// Returns whether this row remains explicitly uncertain.
    pub const fn is_unknown(&self) -> bool {
        matches!(
            self,
            Self::BoundedUnknown { .. } | Self::LossyAdapterOnly { .. }
        )
    }

    /// Returns whether this row has only numerical adapter evidence.
    pub const fn is_lossy_adapter_only(&self) -> bool {
        matches!(self, Self::LossyAdapterOnly { .. })
    }
}

/// Certification result for one active residual row.
#[derive(Clone, Debug, PartialEq)]
pub struct CertifiedCandidateRow {
    /// Source constraint index in the original problem.
    pub constraint_index: usize,
    /// Constraint name copied for diagnostics.
    pub name: String,
    /// Constraint kind used to interpret the residual.
    pub kind: ConstraintKind,
    /// Exact signed residual value after inequality normalization.
    pub signed_residual: Option<Real>,
    /// Certification status.
    pub status: CertifiedCandidateStatus,
}

/// Report for replaying a candidate solution against a prepared problem.
#[derive(Clone, Debug, PartialEq)]
pub struct CandidateCertificationReport {
    /// Per-active-row certification results.
    pub rows: Vec<CertifiedCandidateRow>,
    /// Count of rows proved satisfied.
    pub certified_satisfied_rows: usize,
    /// Count of rows proved violated.
    pub certified_violation_rows: usize,
    /// Count of rows still explicitly uncertain.
    pub bounded_unknown_rows: usize,
    /// Count of rows carrying only lossy proposal-adapter evidence.
    pub lossy_adapter_only_rows: usize,
    /// Count of rows that failed during exact replay.
    pub domain_failure_rows: usize,
}

/// Exact residual ball enclosure for one active row.
#[derive(Clone, Debug, PartialEq)]
pub struct CandidateResidualBall {
    /// Active-row ordinal in the certification report, not source constraint id.
    pub active_row: usize,
    /// Exact nonnegative radius around the replayed residual center.
    pub radius: Real,
}

impl CandidateCertificationReport {
    /// Returns true only when every active row has a proof of satisfaction.
    pub fn all_satisfied(&self) -> bool {
        !self.rows.is_empty() && self.certified_satisfied_rows == self.rows.len()
    }

    /// Returns true when at least one row has a proof of violation.
    pub fn has_certified_violation(&self) -> bool {
        self.certified_violation_rows > 0
    }

    /// Returns true when the report contains uncertainty but no proved failure.
    pub fn has_only_uncertainty(&self) -> bool {
        self.certified_violation_rows == 0
            && self.domain_failure_rows == 0
            && self.bounded_unknown_rows > 0
    }
}

/// Replay and certify a candidate using the default bounded sign policy.
pub fn certify_candidate(
    prepared: &PreparedProblem<'_>,
    context: &EvaluationContext,
) -> CandidateCertificationReport {
    certify_candidate_with_config(prepared, context, CandidateCertificationConfig::default())
}

/// Replay and certify a candidate with an explicit bounded sign policy.
pub fn certify_candidate_with_config(
    prepared: &PreparedProblem<'_>,
    context: &EvaluationContext,
    config: CandidateCertificationConfig,
) -> CandidateCertificationReport {
    let mut rows = Vec::new();

    for (constraint_index, constraint) in prepared.problem().constraints.iter().enumerate() {
        if !constraint.active {
            continue;
        }
        let replayed = if let Some(affine) = &prepared.affine_residuals()[constraint_index] {
            affine.eval_real(prepared.problem().variables.as_slice(), context.bindings())
        } else {
            constraint.residual.eval_real(context.bindings())
        };
        let row = match replayed {
            Ok(value) => {
                let signed = normalize_residual(value, constraint.kind);
                let status = classify_signed_residual(&signed, constraint.kind, config);
                CertifiedCandidateRow {
                    constraint_index,
                    name: constraint.name.clone(),
                    kind: constraint.kind,
                    signed_residual: Some(signed),
                    status,
                }
            }
            Err(error) => CertifiedCandidateRow {
                constraint_index,
                name: constraint.name.clone(),
                kind: constraint.kind,
                signed_residual: None,
                status: CertifiedCandidateStatus::DomainFailure {
                    message: format!("{:?}", EvalError::from(error)),
                },
            },
        };
        rows.push(row);
    }

    let certified_satisfied_rows = rows
        .iter()
        .filter(|row| row.status.is_certified_satisfied())
        .count();
    let certified_violation_rows = rows
        .iter()
        .filter(|row| row.status.is_certified_violation())
        .count();
    let bounded_unknown_rows = rows.iter().filter(|row| row.status.is_unknown()).count();
    let lossy_adapter_only_rows = rows
        .iter()
        .filter(|row| row.status.is_lossy_adapter_only())
        .count();
    let domain_failure_rows = rows
        .iter()
        .filter(|row| matches!(row.status, CertifiedCandidateStatus::DomainFailure { .. }))
        .count();

    CandidateCertificationReport {
        rows,
        certified_satisfied_rows,
        certified_violation_rows,
        bounded_unknown_rows,
        lossy_adapter_only_rows,
        domain_failure_rows,
    }
}

/// Build an explicit non-certificate for a numerical proposal-only candidate.
///
/// Some callers need to preserve a solver result even when they have not yet
/// replayed exact residuals. This helper emits one row per active constraint
/// with [`CertifiedCandidateStatus::LossyAdapterOnly`], carrying the selected
/// proposal-engine report. It is intentionally not an acceptance API: exact
/// replay through [`certify_candidate`] or a stronger interval/alpha proof
/// remains mandatory before trusting the candidate.
pub fn report_lossy_adapter_only_candidate(
    prepared: &PreparedProblem<'_>,
    proposal_engine: ProposalEngineReport,
) -> CandidateCertificationReport {
    let rows = prepared
        .problem()
        .constraints
        .iter()
        .enumerate()
        .filter(|(_, constraint)| constraint.active)
        .map(|(constraint_index, constraint)| CertifiedCandidateRow {
            constraint_index,
            name: constraint.name.clone(),
            kind: constraint.kind,
            signed_residual: None,
            status: CertifiedCandidateStatus::LossyAdapterOnly {
                requested: proposal_engine.requested,
                used: proposal_engine.used,
                precision: proposal_engine.precision,
            },
        })
        .collect::<Vec<_>>();
    let lossy_adapter_only_rows = rows.len();
    CandidateCertificationReport {
        rows,
        certified_satisfied_rows: 0,
        certified_violation_rows: 0,
        bounded_unknown_rows: lossy_adapter_only_rows,
        lossy_adapter_only_rows,
        domain_failure_rows: 0,
    }
}

/// Replay a candidate and certify selected rows with exact residual balls.
///
/// Ball enclosures are a standard interval-analysis shape; here they are only
/// accepted when `hyperlimit` can certify the sign of the whole ball. This is
/// the proof-producing filter layer described by Yap, not a tolerance test.
pub fn certify_candidate_with_residual_balls(
    prepared: &PreparedProblem<'_>,
    context: &EvaluationContext,
    balls: &[CandidateResidualBall],
    policy: PredicatePolicy,
) -> CandidateCertificationReport {
    let mut report = certify_candidate(prepared, context);
    for ball in balls {
        let Some(row) = report.rows.get_mut(ball.active_row) else {
            continue;
        };
        let Some(center) = row.signed_residual.as_ref() else {
            continue;
        };
        let ball_report = certified_ball_sign_report_with_policy(center, &ball.radius, policy);
        let status = match ball_report.outcome {
            hyperlimit::PredicateOutcome::Decided {
                value: Sign::Zero, ..
            } => CertifiedCandidateStatus::BallCertified {
                sign: RealSign::Zero,
            },
            hyperlimit::PredicateOutcome::Decided {
                value: Sign::Positive,
                ..
            } => CertifiedCandidateStatus::BallCertified {
                sign: RealSign::Positive,
            },
            hyperlimit::PredicateOutcome::Decided {
                value: Sign::Negative,
                ..
            } => CertifiedCandidateStatus::BallCertified {
                sign: RealSign::Negative,
            },
            hyperlimit::PredicateOutcome::Unknown {
                needed: hyperlimit::predicate::RefinementNeed::Unsupported,
                ..
            } => CertifiedCandidateStatus::InvalidBallRadius,
            hyperlimit::PredicateOutcome::Unknown { .. } => {
                CertifiedCandidateStatus::BoundedUnknown {
                    min_precision: CandidateCertificationConfig::default().min_precision,
                }
            }
        };
        row.status = status;
    }
    recompute_report_counts(&mut report);
    report
}

fn normalize_residual(value: Real, kind: ConstraintKind) -> Real {
    match kind {
        ConstraintKind::Equality | ConstraintKind::Soft => value,
        ConstraintKind::LessOrEqual => positive_part(value),
        ConstraintKind::GreaterOrEqual => positive_part(-value),
    }
}

fn classify_signed_residual(
    signed: &Real,
    kind: ConstraintKind,
    config: CandidateCertificationConfig,
) -> CertifiedCandidateStatus {
    match signed.certified_sign_until(config.min_precision) {
        CertifiedRealSign::Known { sign, certificate } => match (kind, sign) {
            (ConstraintKind::Equality | ConstraintKind::Soft, RealSign::Zero) => {
                CertifiedCandidateStatus::CertifiedZero { certificate }
            }
            (ConstraintKind::LessOrEqual | ConstraintKind::GreaterOrEqual, RealSign::Zero) => {
                CertifiedCandidateStatus::CertifiedSatisfiedInequality { certificate }
            }
            (_, RealSign::Positive | RealSign::Negative) => {
                CertifiedCandidateStatus::CertifiedViolation { sign, certificate }
            }
        },
        CertifiedRealSign::Unknown { min_precision } => {
            CertifiedCandidateStatus::BoundedUnknown { min_precision }
        }
    }
}

fn recompute_report_counts(report: &mut CandidateCertificationReport) {
    report.certified_satisfied_rows = report
        .rows
        .iter()
        .filter(|row| row.status.is_certified_satisfied())
        .count();
    report.certified_violation_rows = report
        .rows
        .iter()
        .filter(|row| row.status.is_certified_violation())
        .count();
    report.bounded_unknown_rows = report
        .rows
        .iter()
        .filter(|row| row.status.is_unknown())
        .count();
    report.lossy_adapter_only_rows = report
        .rows
        .iter()
        .filter(|row| row.status.is_lossy_adapter_only())
        .count();
    report.domain_failure_rows = report
        .rows
        .iter()
        .filter(|row| matches!(row.status, CertifiedCandidateStatus::DomainFailure { .. }))
        .count();
}
