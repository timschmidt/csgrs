//! Shared readiness-check planning and explicit execution coverage.
//!
//! Both native library adapters and file-oriented command-line workflows use
//! this runner. A selected check can never disappear silently: it either runs
//! or records why it was not applicable, uncertain, or deliberately skipped.

use serde::Serialize;

use crate::{Check, Violation};

/// Schema version for check descriptors and execution records.
pub const CHECK_REGISTRY_VERSION: u32 = 1;

/// Version of shared check execution and geometry-uncertainty reporting semantics.
pub const CHECK_IMPLEMENTATION_VERSION: u32 = 22;

/// Opinionated default check plan used by both CLI and native callers.
pub fn default_checks() -> &'static [Check] {
    crate::cli::DEFAULT_CHECKS
}

/// Outcome supplied by a check adapter after attempting one registered check.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum CheckRunDisposition {
    /// The check had sufficient inputs and completed.
    Executed,
    /// The checked design does not contain the feature governed by this check.
    NotApplicable(String),
    /// Relevant intent exists, but the available inputs cannot certify it.
    Uncertain(String),
    /// Policy deliberately disabled an otherwise applicable check.
    Skipped(String),
}

/// Deterministic policy, input, and tool metadata copied into each check record.
#[derive(Clone, Debug, Eq, PartialEq, Serialize)]
pub struct ReadinessContext {
    /// Canonical capability/rule-policy digest, when resolved.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub policy_digest: Option<String>,
    /// Sorted digests of normalized inputs consumed by the runner.
    pub normalized_input_digests: Vec<String>,
    /// Tool identity.
    pub tool: String,
    /// Tool release version.
    pub tool_version: String,
}

impl Default for ReadinessContext {
    fn default() -> Self {
        Self {
            policy_digest: None,
            normalized_input_digests: Vec::new(),
            tool: "hyperdrc".into(),
            tool_version: env!("CARGO_PKG_VERSION").into(),
        }
    }
}

/// Stable execution state recorded in release evidence.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum CheckExecutionStatus {
    /// The check executed and emitted no findings.
    Passed,
    /// The check executed and emitted one or more findings.
    Failed,
    /// The design did not contain the governed feature.
    NotApplicable,
    /// The adapter lacked evidence required to certify the check.
    Uncertain,
    /// Policy deliberately disabled the check.
    Skipped,
}

/// Coverage evidence for one registered check.
#[derive(Clone, Debug, Eq, PartialEq, Serialize)]
pub struct CheckExecutionRecord {
    /// Stable kebab-case check identifier.
    pub check: String,
    /// Version of the check contract used for this run.
    pub check_version: u32,
    /// Explicit execution disposition.
    pub status: CheckExecutionStatus,
    /// Number of findings emitted during this check.
    pub finding_count: usize,
    /// Canonical capability/rule-policy digest used for this check.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub policy_digest: Option<String>,
    /// Sorted normalized input digests used for this check.
    pub normalized_input_digests: Vec<String>,
    /// Evaluating tool identity.
    pub tool: String,
    /// Evaluating tool version.
    pub tool_version: String,
    /// Optional wall-clock duration; omitted from deterministic release runs.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub elapsed_microseconds: Option<u64>,
    /// Required explanation for any non-executed disposition.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub reason: Option<String>,
}

/// Complete ordered coverage for one readiness run.
#[derive(Clone, Debug, Default, Eq, PartialEq, Serialize)]
pub struct CheckCoverage {
    /// Registry schema version used to plan the run.
    pub registry_version: u32,
    /// One record for every selected check, in execution order.
    pub checks: Vec<CheckExecutionRecord>,
}

impl CheckCoverage {
    /// True when every selected check either executed or was not applicable.
    pub fn is_complete(&self) -> bool {
        self.checks.iter().all(|record| {
            matches!(
                record.status,
                CheckExecutionStatus::Passed
                    | CheckExecutionStatus::Failed
                    | CheckExecutionStatus::NotApplicable
            )
        })
    }

    /// Number of checks that could not produce conclusive coverage.
    pub fn inconclusive_count(&self) -> usize {
        self.checks
            .iter()
            .filter(|record| {
                matches!(
                    record.status,
                    CheckExecutionStatus::Uncertain | CheckExecutionStatus::Skipped
                )
            })
            .count()
    }

    /// Number of outcomes that must block certification.
    ///
    /// Uncertainty is always blocking. A skipped check is blocking only when
    /// the adapter failed to explain the policy decision.
    pub fn blocking_count(&self) -> usize {
        self.checks
            .iter()
            .filter(|record| {
                record.status == CheckExecutionStatus::Uncertain
                    || (record.status == CheckExecutionStatus::Skipped
                        && record
                            .reason
                            .as_deref()
                            .is_none_or(|reason| reason.trim().is_empty()))
            })
            .count()
    }
}

/// Deterministic plan over registered checks.
#[derive(Clone, Debug)]
pub struct ReadinessRunner {
    checks: Vec<Check>,
    context: ReadinessContext,
}

impl ReadinessRunner {
    /// Plans the supplied checks in caller-provided deterministic order.
    pub fn new(checks: impl IntoIterator<Item = Check>) -> Self {
        Self {
            checks: checks.into_iter().collect(),
            context: ReadinessContext::default(),
        }
    }

    /// Attaches canonical policy, input, and tool provenance to every record.
    pub fn with_context(mut self, mut context: ReadinessContext) -> Self {
        context.normalized_input_digests.sort();
        context.normalized_input_digests.dedup();
        self.context = context;
        self
    }

    /// Executes every planned check against one shared finding sink.
    ///
    /// The adapter owns check-specific inputs. The runner owns coverage
    /// accounting, so command-line and native paths cannot silently omit a
    /// selected check.
    pub fn run<E>(
        &self,
        violations: &mut Vec<Violation>,
        mut execute: impl FnMut(Check, &mut Vec<Violation>) -> Result<CheckRunDisposition, E>,
    ) -> Result<CheckCoverage, E> {
        let mut records = Vec::with_capacity(self.checks.len());
        for check in &self.checks {
            let before = violations.len();
            let disposition = execute(*check, violations)?;
            let finding_count = violations.len().saturating_sub(before);
            // Geometry gateways retain their typed failure as an attributed
            // finding. An adapter returning Executed must not flatten that
            // missing certificate into an ordinary failed/passed decision.
            let disposition = match violations[before..]
                .iter()
                .find(|finding| finding.check == "geometry-uncertainty")
            {
                Some(finding) => {
                    CheckRunDisposition::Uncertain(finding.message.clone().unwrap_or_else(|| {
                        "selected check contains an uncertified geometry query".to_owned()
                    }))
                }
                None => disposition,
            };
            let (status, reason) = match disposition {
                CheckRunDisposition::Executed if finding_count == 0 => {
                    (CheckExecutionStatus::Passed, None)
                }
                CheckRunDisposition::Executed => (CheckExecutionStatus::Failed, None),
                CheckRunDisposition::NotApplicable(reason) => {
                    (CheckExecutionStatus::NotApplicable, Some(reason))
                }
                CheckRunDisposition::Uncertain(reason) => {
                    (CheckExecutionStatus::Uncertain, Some(reason))
                }
                CheckRunDisposition::Skipped(reason) => {
                    (CheckExecutionStatus::Skipped, Some(reason))
                }
            };
            records.push(CheckExecutionRecord {
                check: check.slug(),
                check_version: CHECK_IMPLEMENTATION_VERSION,
                status,
                finding_count,
                policy_digest: self.context.policy_digest.clone(),
                normalized_input_digests: self.context.normalized_input_digests.clone(),
                tool: self.context.tool.clone(),
                tool_version: self.context.tool_version.clone(),
                elapsed_microseconds: None,
                reason,
            });
        }
        Ok(CheckCoverage {
            registry_version: CHECK_REGISTRY_VERSION,
            checks: records,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn geometry_findings_preserve_uncertainty_without_poisoning_later_checks() {
        let runner =
            ReadinessRunner::new([Check::StackupReadiness, Check::TestpointCoverageReadiness]);
        let mut violations = Vec::new();
        let coverage = runner
            .run(&mut violations, |check, findings| {
                if check == Check::StackupReadiness {
                    assert!(
                        crate::checks::geometry_result_for_check::<()>(
                            Err(crate::PcbGeometryUncertainty {
                                operation: "boundary-distance".into(),
                                source: Some("native curve fixture".into()),
                                detail: "retained unsupported boundary".into(),
                            }),
                            "stackup-readiness",
                            &["F.Cu"],
                            findings,
                        )
                        .is_none()
                    );
                }
                Ok::<_, ()>(CheckRunDisposition::Executed)
            })
            .unwrap();
        assert_eq!(coverage.checks[0].status, CheckExecutionStatus::Uncertain);
        assert_eq!(coverage.checks[0].finding_count, 1);
        assert!(
            coverage.checks[0]
                .reason
                .as_deref()
                .unwrap()
                .contains("native curve fixture")
        );
        assert_eq!(coverage.checks[1].status, CheckExecutionStatus::Passed);
        assert_eq!(coverage.checks[1].finding_count, 0);
        assert_eq!(violations[0].check, "geometry-uncertainty");
        assert_eq!(violations[0].layers, ["F.Cu"]);
    }

    #[test]
    fn runner_records_every_selected_check_and_non_execution_reason() {
        let runner =
            ReadinessRunner::new([Check::StackupReadiness, Check::TestpointCoverageReadiness]);
        let mut violations = Vec::new();
        let coverage = runner
            .run(&mut violations, |check, _| {
                Ok::<_, ()>(match check {
                    Check::StackupReadiness => CheckRunDisposition::Executed,
                    _ => CheckRunDisposition::NotApplicable(
                        "design declares no testpoint intent".into(),
                    ),
                })
            })
            .unwrap();

        assert!(coverage.is_complete());
        assert_eq!(coverage.checks.len(), 2);
        assert_eq!(coverage.checks[0].status, CheckExecutionStatus::Passed);
        assert_eq!(
            coverage.checks[1].status,
            CheckExecutionStatus::NotApplicable
        );
        assert_eq!(
            coverage.checks[1].reason.as_deref(),
            Some("design declares no testpoint intent")
        );
    }
}
