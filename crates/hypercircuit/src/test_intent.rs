//! Retained design-for-test requirements and coverage claims.

use std::collections::BTreeSet;

use hyperlattice::Point2;

use crate::{
    BoardSide, CircuitId, CircuitInstanceId, CircuitLibrary, CoordinateFrame2, NetId, PinRef, Real,
    SourceSpan,
};

/// Stable semantic target of a test requirement or access point.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub enum TestTarget {
    Net {
        circuit: CircuitId,
        net: NetId,
    },
    Pin {
        circuit: CircuitId,
        instance: CircuitInstanceId,
        pin: PinRef,
    },
}

impl TestTarget {
    /// Stable language-neutral identity used at the HyperDRC boundary.
    pub fn stable_id(&self) -> String {
        match self {
            Self::Net { circuit, net } => {
                format!("circuit:{}/net:{}", circuit.as_str(), net.as_str())
            }
            Self::Pin {
                circuit,
                instance,
                pin,
            } => format!(
                "circuit:{}/instance:{}/pin:{}",
                circuit.as_str(),
                instance.as_str(),
                pin.as_str()
            ),
        }
    }
}

/// Definition-qualified component identity used by DFT programming and scan intent.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub struct TestDeviceTarget {
    pub circuit: CircuitId,
    pub instance: CircuitInstanceId,
}

/// Accepted evidence for one required electrical-test target.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum TestCoverageMethod {
    PhysicalAccess,
    BoundaryScan { chain: String },
    Functional { procedure: String },
    Waived { reason: String },
    Untestable { reason: String },
}

/// One release-gating test requirement.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, PartialEq)]
pub struct TestRequirement {
    pub id: String,
    pub target: TestTarget,
    pub accepted_methods: Vec<TestCoverageMethod>,
    pub source: Option<SourceSpan>,
}

/// Exact physical access or nonphysical coverage claim.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, PartialEq)]
pub struct TestAccess {
    pub id: String,
    pub target: TestTarget,
    pub method: TestCoverageMethod,
    #[cfg_attr(
        feature = "interchange",
        serde(with = "crate::interchange::optional_point")
    )]
    pub position: Option<Point2>,
    pub frame: CoordinateFrame2,
    pub side: BoardSide,
    pub reference: Option<CircuitInstanceId>,
    pub pad_or_pin: Option<String>,
    pub probe_diameter: Option<Real>,
}

/// Ordered boundary-scan chain and covered targets.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, PartialEq)]
pub struct BoundaryScanChain {
    pub id: String,
    pub devices: Vec<TestDeviceTarget>,
    pub covered_targets: Vec<TestTarget>,
    pub instruction_register_length: Option<u32>,
}

/// Programming/debug interface required during manufacturing.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, PartialEq)]
pub struct ProgrammingTarget {
    pub id: String,
    pub device: TestDeviceTarget,
    pub interface: String,
    pub signals: Vec<TestTarget>,
    pub artifact: Option<String>,
}

/// Mechanical/electrical fixture constraint.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, PartialEq)]
pub struct FixtureConstraint {
    pub id: String,
    pub minimum_probe_spacing: Option<Real>,
    pub maximum_probe_force: Option<Real>,
    pub allowed_sides: Vec<BoardSide>,
    pub note: Option<String>,
}

/// Required power-domain state during test.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, PartialEq)]
pub struct PowerDomainTestIntent {
    pub id: String,
    pub supply: TestTarget,
    pub return_target: TestTarget,
    pub nominal_voltage: Real,
    pub current_limit: Option<Real>,
    pub sequence_after: Vec<String>,
}

/// Complete retained DFT intent.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, Default, PartialEq)]
pub struct DesignForTestIntent {
    pub requirements: Vec<TestRequirement>,
    pub access: Vec<TestAccess>,
    pub boundary_scan_chains: Vec<BoundaryScanChain>,
    pub programming_targets: Vec<ProgrammingTarget>,
    pub fixture_constraints: Vec<FixtureConstraint>,
    pub power_domains: Vec<PowerDomainTestIntent>,
}

/// Structural validation failure in retained DFT intent.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum TestIntentIssue {
    DuplicateId(String),
    EmptyId,
    InvalidSource(String),
    InvalidFrame(String),
    InvalidPhysicalAccess(String),
    InvalidProbeDiameter(String),
    EmptyProgrammingInterface(String),
    UnknownTarget {
        owner: String,
        target: TestTarget,
    },
    UnknownDevice {
        owner: String,
        device: TestDeviceTarget,
    },
    UnknownPowerDependency {
        domain: String,
        dependency: String,
    },
}

impl DesignForTestIntent {
    /// Validates identity and physical evidence without claiming fault coverage.
    pub fn validate(&self) -> Vec<TestIntentIssue> {
        let mut issues = Vec::new();
        let mut ids = BTreeSet::new();
        let all_ids = self
            .requirements
            .iter()
            .map(|item| item.id.as_str())
            .chain(self.access.iter().map(|item| item.id.as_str()))
            .chain(
                self.boundary_scan_chains
                    .iter()
                    .map(|item| item.id.as_str()),
            )
            .chain(self.programming_targets.iter().map(|item| item.id.as_str()))
            .chain(self.fixture_constraints.iter().map(|item| item.id.as_str()))
            .chain(self.power_domains.iter().map(|item| item.id.as_str()));
        for id in all_ids {
            if id.trim().is_empty() {
                issues.push(TestIntentIssue::EmptyId);
            } else if !ids.insert(id) {
                issues.push(TestIntentIssue::DuplicateId(id.into()));
            }
        }
        for requirement in &self.requirements {
            if requirement
                .source
                .as_ref()
                .is_some_and(|source| !source.is_valid())
            {
                issues.push(TestIntentIssue::InvalidSource(requirement.id.clone()));
            }
        }
        for access in &self.access {
            if access.frame.validate().is_err() {
                issues.push(TestIntentIssue::InvalidFrame(access.id.clone()));
            }
            if matches!(access.method, TestCoverageMethod::PhysicalAccess)
                && access.position.is_none()
            {
                issues.push(TestIntentIssue::InvalidPhysicalAccess(access.id.clone()));
            }
            if access
                .probe_diameter
                .as_ref()
                .is_some_and(|diameter| diameter <= &Real::zero())
            {
                issues.push(TestIntentIssue::InvalidProbeDiameter(access.id.clone()));
            }
        }
        for target in &self.programming_targets {
            if target.interface.trim().is_empty() {
                issues.push(TestIntentIssue::EmptyProgrammingInterface(
                    target.id.clone(),
                ));
            }
        }
        issues
    }

    /// Validates all semantic references against the retained circuit library.
    pub fn validate_against(&self, library: &CircuitLibrary) -> Vec<TestIntentIssue> {
        let mut issues = self.validate();
        let targets = self
            .requirements
            .iter()
            .map(|item| (item.id.as_str(), &item.target))
            .chain(
                self.access
                    .iter()
                    .map(|item| (item.id.as_str(), &item.target)),
            )
            .chain(self.boundary_scan_chains.iter().flat_map(|item| {
                item.covered_targets
                    .iter()
                    .map(move |target| (item.id.as_str(), target))
            }))
            .chain(self.programming_targets.iter().flat_map(|item| {
                item.signals
                    .iter()
                    .map(move |target| (item.id.as_str(), target))
            }))
            .chain(self.power_domains.iter().flat_map(|item| {
                [
                    (item.id.as_str(), &item.supply),
                    (item.id.as_str(), &item.return_target),
                ]
            }));
        for (owner, target) in targets {
            if !target_exists(library, target) {
                issues.push(TestIntentIssue::UnknownTarget {
                    owner: owner.into(),
                    target: target.clone(),
                });
            }
        }
        for (owner, device) in self
            .boundary_scan_chains
            .iter()
            .flat_map(|item| {
                item.devices
                    .iter()
                    .map(move |device| (item.id.as_str(), device))
            })
            .chain(
                self.programming_targets
                    .iter()
                    .map(|item| (item.id.as_str(), &item.device)),
            )
        {
            if !library
                .circuits
                .iter()
                .find(|circuit| circuit.id == device.circuit)
                .is_some_and(|circuit| {
                    circuit
                        .instances
                        .iter()
                        .any(|instance| instance.id == device.instance)
                })
            {
                issues.push(TestIntentIssue::UnknownDevice {
                    owner: owner.into(),
                    device: device.clone(),
                });
            }
        }
        let power_ids = self
            .power_domains
            .iter()
            .map(|domain| domain.id.as_str())
            .collect::<BTreeSet<_>>();
        for domain in &self.power_domains {
            for dependency in &domain.sequence_after {
                if !power_ids.contains(dependency.as_str()) {
                    issues.push(TestIntentIssue::UnknownPowerDependency {
                        domain: domain.id.clone(),
                        dependency: dependency.clone(),
                    });
                }
            }
        }
        issues
    }
}

fn target_exists(library: &CircuitLibrary, target: &TestTarget) -> bool {
    match target {
        TestTarget::Net { circuit, net } => library
            .circuits
            .iter()
            .find(|candidate| &candidate.id == circuit)
            .is_some_and(|candidate| candidate.nets.iter().any(|item| &item.id == net)),
        TestTarget::Pin {
            circuit,
            instance,
            pin,
        } => library
            .circuits
            .iter()
            .find(|candidate| &candidate.id == circuit)
            .and_then(|candidate| candidate.instances.iter().find(|item| &item.id == instance))
            .is_some_and(|candidate| candidate.pins.iter().any(|item| &item.pin == pin)),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn physical_access_requires_an_exact_position_and_valid_frame() {
        let circuit = CircuitId::new("main").unwrap();
        let intent = DesignForTestIntent {
            access: vec![TestAccess {
                id: "TP1".into(),
                target: TestTarget::Net {
                    circuit,
                    net: NetId::new("VCC").unwrap(),
                },
                method: TestCoverageMethod::PhysicalAccess,
                position: None,
                frame: CoordinateFrame2::board_default(),
                side: BoardSide::Front,
                reference: None,
                pad_or_pin: None,
                probe_diameter: Some(Real::one()),
            }],
            ..DesignForTestIntent::default()
        };
        assert_eq!(
            intent.validate(),
            vec![TestIntentIssue::InvalidPhysicalAccess("TP1".into())]
        );
    }
}
