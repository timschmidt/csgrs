//! Native design-for-test requirements and structural access evidence.

use std::collections::BTreeSet;

use crate::{FindingSubject, Scalar, Severity, Violation};

/// How one required electrical target is claimed to be covered.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum NativeTestCoverageMethod {
    /// Direct probe access exists. This is access evidence, not fault coverage.
    PhysicalAccess,
    /// A declared boundary-scan chain covers the target.
    BoundaryScan,
    /// A named functional test covers the target.
    Functional(String),
    /// Approved waiver retained with a reason.
    Waived(String),
    /// Explicitly impossible to test, retained with a reason.
    Untestable(String),
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
    let mut violations = Vec::new();
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
        }
    }
    violations
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
}
