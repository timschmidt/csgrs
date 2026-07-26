//! Native design-for-test requirements and structural access evidence.

use std::collections::BTreeSet;

use serde::{Deserialize, Serialize};

use crate::{FindingSubject, Scalar, Severity, Violation};

/// How one required electrical target is claimed to be covered.
#[derive(Clone, Debug, Eq, PartialEq, Serialize, Deserialize)]
pub enum NativeTestCoverageMethod {
    /// Direct probe access exists. This is access evidence, not fault coverage.
    PhysicalAccess,
    /// A declared boundary-scan chain covers the target.
    BoundaryScan(String),
    /// A named functional test covers the target.
    Functional(String),
    /// Approved waiver retained with a reason.
    Waived(String),
    /// Explicitly impossible to test, retained with a reason.
    Untestable(String),
}

/// Release-facing disposition for one required DFT target.
#[derive(Clone, Debug, Eq, PartialEq, Serialize, Deserialize)]
pub enum NativeTestCoverageStatus {
    /// Exact physical access is present.
    Accessible,
    /// A named boundary-scan chain covers the target.
    BoundaryScan {
        /// Stable chain identity.
        chain: String,
    },
    /// A named functional procedure covers the target.
    Functional {
        /// Stable procedure identity.
        procedure: String,
    },
    /// Coverage was explicitly waived.
    Waived {
        /// Retained waiver reason.
        reason: String,
    },
    /// The target is explicitly classified as untestable.
    Untestable {
        /// Retained classification reason.
        reason: String,
    },
    /// No accepted coverage claim exists.
    Missing,
    /// A physical claim exists but has no exact location.
    InvalidPhysicalAccess,
}

/// One requirement's explicit coverage result.
#[derive(Clone, Debug, Eq, PartialEq, Serialize, Deserialize)]
pub struct NativeTestCoverageRecord {
    /// Stable authored requirement identity.
    pub requirement_id: String,
    /// Stable semantic target identity.
    pub target: String,
    /// Classified coverage disposition.
    pub status: NativeTestCoverageStatus,
    /// Stable matched claim identity, when one exists.
    pub claim_id: Option<String>,
}

/// Complete requirement-by-requirement native DFT coverage evidence.
#[derive(Clone, Debug, Default, Eq, PartialEq, Serialize, Deserialize)]
pub struct NativeTestCoverageReport {
    /// Records in authored requirement order.
    pub records: Vec<NativeTestCoverageRecord>,
}

/// One evaluation result, with durable coverage separate from report findings.
#[derive(Debug, Default)]
pub struct NativeTestCoverageEvaluation {
    /// Durable requirement-by-requirement evidence.
    pub report: NativeTestCoverageReport,
    /// Release findings produced by invalid or missing coverage.
    pub violations: Vec<Violation>,
}

/// One net/pin target that a release requires test evidence for.
#[derive(Clone, Debug, PartialEq)]
pub struct NativeTestRequirement {
    /// Stable requirement identity.
    pub id: String,
    /// Stable semantic target identity.
    pub target: String,
    /// Source-addressable authored subject.
    pub subject: FindingSubject,
    /// Accepted coverage methods. Empty means direct access is required.
    pub accepted_methods: Vec<NativeTestCoverageMethod>,
}

/// One physical or nonphysical coverage claim for a semantic test target.
#[derive(Clone, Debug, PartialEq)]
pub struct NativeTestAccess {
    /// Stable access/claim identity.
    pub id: String,
    /// Stable semantic target identity.
    pub target: String,
    /// Coverage method.
    pub method: NativeTestCoverageMethod,
    /// Exact board-space locations; empty for nonphysical coverage.
    pub locations: Vec<[Scalar; 2]>,
    /// Optional source-addressable subject.
    pub subject: Option<FindingSubject>,
}

/// Evaluates declared test requirements against retained access/coverage claims.
///
/// Direct access proves probeability only. It never claims stuck-at, parametric,
/// or functional fault coverage.
pub fn native_testpoint_coverage_readiness(
    requirements: &[NativeTestRequirement],
    access: &[NativeTestAccess],
) -> Vec<Violation> {
    native_testpoint_coverage(requirements, access).violations
}

/// Evaluates and classifies every declared native DFT requirement.
pub fn native_testpoint_coverage(
    requirements: &[NativeTestRequirement],
    access: &[NativeTestAccess],
) -> NativeTestCoverageEvaluation {
    let mut violations = Vec::new();
    let mut records = Vec::new();
    let mut requirement_ids = BTreeSet::new();
    for requirement in requirements {
        if !requirement_ids.insert(requirement.id.as_str()) {
            violations.push(
                Violation::new(
                    "testpoint-coverage-readiness",
                    Severity::Error,
                    vec!["test-intent".into()],
                    None,
                    Vec::new(),
                    Vec::new(),
                    Some(format!(
                        "duplicate native test requirement {}",
                        requirement.id
                    )),
                )
                .with_subjects(vec![requirement.subject.clone()]),
            );
            records.push(NativeTestCoverageRecord {
                requirement_id: requirement.id.clone(),
                target: requirement.target.clone(),
                status: NativeTestCoverageStatus::Missing,
                claim_id: None,
            });
            continue;
        }
        let claims = access
            .iter()
            .filter(|claim| claim.target == requirement.target)
            .collect::<Vec<_>>();
        let compatible = claims.iter().find(|claim| {
            (requirement.accepted_methods.is_empty()
                && claim.method == NativeTestCoverageMethod::PhysicalAccess)
                || requirement
                    .accepted_methods
                    .iter()
                    .any(|method| method == &claim.method)
        });
        let Some(claim) = compatible else {
            let mut subjects = vec![requirement.subject.clone()];
            subjects.extend(claims.iter().filter_map(|claim| claim.subject.clone()));
            violations.push(
                Violation::new(
                    "testpoint-coverage-readiness",
                    Severity::Error,
                    vec!["test-intent".into()],
                    None,
                    Vec::new(),
                    claims
                        .iter()
                        .flat_map(|claim| claim.locations.iter())
                        .filter_map(|point| {
                            Some([point[0].to_f64_lossy()?, point[1].to_f64_lossy()?])
                        })
                        .collect(),
                    Some(format!(
                        "required test target {} has no accepted coverage method; physical access alone is not fault coverage",
                        requirement.target
                    )),
                )
                .with_subjects(subjects),
            );
            records.push(NativeTestCoverageRecord {
                requirement_id: requirement.id.clone(),
                target: requirement.target.clone(),
                status: NativeTestCoverageStatus::Missing,
                claim_id: None,
            });
            continue;
        };
        if claim.method == NativeTestCoverageMethod::PhysicalAccess && claim.locations.is_empty() {
            let mut subjects = vec![requirement.subject.clone()];
            subjects.extend(claim.subject.clone());
            violations.push(
                Violation::new(
                    "testpoint-coverage-readiness",
                    Severity::Error,
                    vec!["test-intent".into()],
                    None,
                    Vec::new(),
                    Vec::new(),
                    Some(format!(
                        "physical test access {} for {} has no board-space location",
                        claim.id, requirement.target
                    )),
                )
                .with_subjects(subjects),
            );
            records.push(NativeTestCoverageRecord {
                requirement_id: requirement.id.clone(),
                target: requirement.target.clone(),
                status: NativeTestCoverageStatus::InvalidPhysicalAccess,
                claim_id: Some(claim.id.clone()),
            });
            continue;
        }
        let status = match &claim.method {
            NativeTestCoverageMethod::PhysicalAccess => NativeTestCoverageStatus::Accessible,
            NativeTestCoverageMethod::BoundaryScan(chain) => {
                NativeTestCoverageStatus::BoundaryScan {
                    chain: chain.clone(),
                }
            }
            NativeTestCoverageMethod::Functional(procedure) => {
                NativeTestCoverageStatus::Functional {
                    procedure: procedure.clone(),
                }
            }
            NativeTestCoverageMethod::Waived(reason) => NativeTestCoverageStatus::Waived {
                reason: reason.clone(),
            },
            NativeTestCoverageMethod::Untestable(reason) => NativeTestCoverageStatus::Untestable {
                reason: reason.clone(),
            },
        };
        records.push(NativeTestCoverageRecord {
            requirement_id: requirement.id.clone(),
            target: requirement.target.clone(),
            status,
            claim_id: Some(claim.id.clone()),
        });
    }
    NativeTestCoverageEvaluation {
        report: NativeTestCoverageReport { records },
        violations,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn subject(id: &str) -> FindingSubject {
        FindingSubject {
            kind: "test-requirement".into(),
            id: id.into(),
            source: None,
        }
    }

    #[test]
    fn physical_access_does_not_satisfy_a_functional_only_requirement() {
        let findings = native_testpoint_coverage_readiness(
            &[NativeTestRequirement {
                id: "program-boot".into(),
                target: "net:BOOT".into(),
                subject: subject("program-boot"),
                accepted_methods: vec![NativeTestCoverageMethod::Functional("boot-smoke".into())],
            }],
            &[NativeTestAccess {
                id: "TP1".into(),
                target: "net:BOOT".into(),
                method: NativeTestCoverageMethod::PhysicalAccess,
                locations: vec![[Scalar::zero(), Scalar::zero()]],
                subject: None,
            }],
        );
        assert_eq!(findings.len(), 1);
        assert!(
            findings[0]
                .message
                .as_deref()
                .unwrap()
                .contains("not fault coverage")
        );
    }

    #[test]
    fn coverage_report_distinguishes_access_scan_function_and_exceptions() {
        let methods = [
            (
                NativeTestCoverageMethod::PhysicalAccess,
                NativeTestCoverageStatus::Accessible,
            ),
            (
                NativeTestCoverageMethod::BoundaryScan("chain-a".into()),
                NativeTestCoverageStatus::BoundaryScan {
                    chain: "chain-a".into(),
                },
            ),
            (
                NativeTestCoverageMethod::Functional("smoke".into()),
                NativeTestCoverageStatus::Functional {
                    procedure: "smoke".into(),
                },
            ),
            (
                NativeTestCoverageMethod::Waived("approved".into()),
                NativeTestCoverageStatus::Waived {
                    reason: "approved".into(),
                },
            ),
            (
                NativeTestCoverageMethod::Untestable("buried".into()),
                NativeTestCoverageStatus::Untestable {
                    reason: "buried".into(),
                },
            ),
        ];
        for (index, (method, expected)) in methods.into_iter().enumerate() {
            let target = format!("net:N{index}");
            let report = native_testpoint_coverage(
                &[NativeTestRequirement {
                    id: format!("requirement-{index}"),
                    target: target.clone(),
                    subject: subject("requirement"),
                    accepted_methods: vec![method.clone()],
                }],
                &[NativeTestAccess {
                    id: format!("claim-{index}"),
                    target,
                    locations: matches!(method, NativeTestCoverageMethod::PhysicalAccess)
                        .then(|| vec![[Scalar::zero(), Scalar::zero()]])
                        .unwrap_or_default(),
                    method,
                    subject: None,
                }],
            );
            assert!(report.violations.is_empty());
            assert_eq!(report.report.records[0].status, expected);
        }
    }
}
