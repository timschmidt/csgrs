//! Electrical-rule checking over retained device pins, ports, and nets.

use std::collections::{BTreeMap, BTreeSet};

use crate::{
    Circuit, CircuitId, CircuitInstanceId, CircuitLibrary, DesignIntent, FunctionalBindingTarget,
    FunctionalRole, FunctionalRoleTarget, NetId, NetKind, PartClass, PinElectricalKind, PinRef,
    PortDirection, PortId,
};

/// Stable endpoint evidence attached to an ERC finding.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ErcEndpoint {
    /// Circuit instance carrying the pin.
    pub instance: CircuitInstanceId,
    /// Logical/package pin.
    pub pin: PinRef,
    /// Declared electrical class.
    pub kind: PinElectricalKind,
}

/// Electrical conflict or missing-source condition.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum ErcIssue {
    /// A pin declared intentionally unconnected has a net binding.
    BoundNotConnectedPin(ErcEndpoint),
    /// More than one actively driven output is tied to a net.
    MultiplePushPullDrivers {
        net: NetId,
        drivers: Vec<ErcEndpoint>,
    },
    /// One or more signal inputs have no local or boundary driver.
    UndrivenInputs {
        net: NetId,
        inputs: Vec<ErcEndpoint>,
    },
    /// One or more power inputs have no power source or ground reference.
    UnpoweredInputs {
        net: NetId,
        inputs: Vec<ErcEndpoint>,
    },
    /// A boundary port declared as ground exposes a non-ground net.
    GroundPortOnSignal { port: PortId, net: NetId },
}

/// Stable configurable ERC rule identity.
#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub enum ErcRuleId {
    /// Bound pin declared intentionally unconnected.
    BoundNotConnectedPin,
    /// Multiple active push-pull/power drivers.
    MultiplePushPullDrivers,
    /// Signal input without a retained driver.
    UndrivenInputs,
    /// Power input without a retained source/reference.
    UnpoweredInputs,
    /// Ground port attached to a signal net.
    GroundPortOnSignal,
}

/// Release significance assigned to one ERC rule.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ErcSeverity {
    /// Suppress findings from this rule.
    Ignore,
    /// Retain informational evidence without blocking release.
    Info,
    /// Retain a review warning without blocking by default.
    Warning,
    /// Treat the finding as a release-blocking error.
    Error,
}

/// Per-rule ERC severity policy.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ErcRuleDeck {
    /// Explicit rule overrides; absent rules use [`ErcRuleDeck::default_severity`].
    pub severities: BTreeMap<ErcRuleId, ErcSeverity>,
    /// Severity applied to rules without an override.
    pub default_severity: ErcSeverity,
}

impl Default for ErcRuleDeck {
    fn default() -> Self {
        Self {
            severities: BTreeMap::new(),
            default_severity: ErcSeverity::Error,
        }
    }
}

impl ErcRuleDeck {
    /// Returns a copy with one rule severity overridden.
    pub fn with_severity(mut self, rule: ErcRuleId, severity: ErcSeverity) -> Self {
        self.severities.insert(rule, severity);
        self
    }

    /// Resolves the effective severity for one rule.
    pub fn severity(&self, rule: ErcRuleId) -> ErcSeverity {
        self.severities
            .get(&rule)
            .copied()
            .unwrap_or(self.default_severity)
    }
}

/// ERC issue paired with its stable rule and configured severity.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ErcFinding {
    /// Stable rule identity.
    pub rule: ErcRuleId,
    /// Effective policy severity.
    pub severity: ErcSeverity,
    /// Endpoint/net evidence.
    pub issue: ErcIssue,
}

/// Policy-evaluated ERC report suitable for release gates.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ConfiguredErcReport {
    /// Non-ignored findings in deterministic source order.
    pub findings: Vec<ErcFinding>,
}

impl ConfiguredErcReport {
    /// True when no finding has error severity.
    pub fn is_release_clean(&self) -> bool {
        !self
            .findings
            .iter()
            .any(|finding| finding.severity == ErcSeverity::Error)
    }
}

/// Deterministic electrical-rule-check report.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ErcReport {
    /// Every discovered electrical issue in circuit/net order.
    pub issues: Vec<ErcIssue>,
}

/// Concrete source counted by CopperTrace's one-source supply-rail rule.
#[derive(Clone, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub enum SupplySource {
    /// Power-output pin on a flattened component.
    ComponentPin {
        instance: CircuitInstanceId,
        pin: PinRef,
    },
    /// Root input supplied by the surrounding system.
    BoundaryPort(PortId),
    /// Functional block contract used when no lower-level source pin exists.
    FunctionalRole {
        circuit: CircuitId,
        role: FunctionalRole,
    },
}

/// CopperTrace-specific semantic ERC issue.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum CopperTraceErcIssue {
    /// Retained intent failed validation before semantic ERC.
    InvalidDesignIntent(Vec<crate::DesignIntentIssue>),
    /// Hierarchy/global-net elaboration failed.
    IntentHierarchy(crate::IntentHierarchyError),
    /// A `power_supply` net has zero or multiple independent sources.
    PowerSupplySourceCount {
        net: NetId,
        sources: Vec<SupplySource>,
    },
    /// A seed functional role omitted one required interface binding.
    MissingRoleBinding {
        target: FunctionalRoleTarget,
        role: FunctionalRole,
        binding: String,
    },
    /// A role binding targets a net whose refined kind violates the contract.
    RoleBindingKindMismatch {
        target: FunctionalRoleTarget,
        role: FunctionalRole,
        binding: String,
        expected: RoleNetKind,
        actual: NetKind,
    },
    /// A component role was assigned to an incompatible resolved part class.
    RolePartClassMismatch {
        target: FunctionalRoleTarget,
        role: FunctionalRole,
        expected: PartClass,
        actual: PartClass,
    },
    /// A current-sense resistor lacks a typed attribute needed for suitability review.
    MissingCurrentSenseAttribute {
        target: FunctionalRoleTarget,
        attribute: String,
    },
}

/// Net-kind family expected by one functional contract terminal.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RoleNetKind {
    Signal,
    Supply,
    Ground,
}

/// Combined ordinary and CopperTrace semantic ERC report.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct CopperTraceErcReport {
    /// Generic pin/driver ERC over the flattened circuit.
    pub structural: Option<ErcReport>,
    /// CopperTrace net-scope, supply, and role-contract findings.
    pub issues: Vec<CopperTraceErcIssue>,
}

impl CopperTraceErcReport {
    /// True when hierarchy elaborated and both generic and semantic ERC passed.
    pub fn is_valid(&self) -> bool {
        self.structural.as_ref().is_some_and(ErcReport::is_valid) && self.issues.is_empty()
    }
}

impl ErcReport {
    /// True when no electrical conflict or missing source was discovered.
    pub fn is_valid(&self) -> bool {
        self.issues.is_empty()
    }
}

impl Circuit {
    /// Checks driver/load compatibility using retained pin and boundary classes.
    ///
    /// Structural validation should run first. Unknown models/pins are reported
    /// there and are skipped here instead of producing speculative ERC results.
    pub fn electrical_rule_check(&self) -> ErcReport {
        let mut issues = Vec::new();
        let mut endpoints = BTreeMap::<NetId, Vec<ErcEndpoint>>::new();
        for instance in &self.instances {
            let Some(model) = self
                .device_models
                .iter()
                .find(|model| model.id == instance.model)
            else {
                continue;
            };
            for binding in &instance.pins {
                let Some(pin) = model.pins.iter().find(|pin| pin.pin == binding.pin) else {
                    continue;
                };
                let endpoint = ErcEndpoint {
                    instance: instance.id.clone(),
                    pin: binding.pin.clone(),
                    kind: pin.kind,
                };
                if pin.kind == PinElectricalKind::NotConnected {
                    issues.push(ErcIssue::BoundNotConnectedPin(endpoint.clone()));
                }
                endpoints
                    .entry(binding.net.clone())
                    .or_default()
                    .push(endpoint);
            }
        }

        for net in &self.nets {
            let connected = endpoints.get(&net.id).map(Vec::as_slice).unwrap_or(&[]);
            let push_pull = connected
                .iter()
                .filter(|endpoint| {
                    matches!(
                        endpoint.kind,
                        PinElectricalKind::Output | PinElectricalKind::PowerOutput
                    )
                })
                .cloned()
                .collect::<Vec<_>>();
            if push_pull.len() > 1 {
                issues.push(ErcIssue::MultiplePushPullDrivers {
                    net: net.id.clone(),
                    drivers: push_pull,
                });
            }

            let signal_inputs = connected
                .iter()
                .filter(|endpoint| endpoint.kind == PinElectricalKind::Input)
                .cloned()
                .collect::<Vec<_>>();
            let boundary_signal_driver = self.ports.iter().any(|port| {
                port.net == net.id
                    && matches!(
                        port.direction,
                        PortDirection::Input | PortDirection::Bidirectional
                    )
            });
            let local_signal_driver = connected.iter().any(|endpoint| {
                matches!(
                    endpoint.kind,
                    PinElectricalKind::Output
                        | PinElectricalKind::Bidirectional
                        | PinElectricalKind::OpenCollector
                        | PinElectricalKind::OpenEmitter
                )
            });
            if !signal_inputs.is_empty() && !boundary_signal_driver && !local_signal_driver {
                issues.push(ErcIssue::UndrivenInputs {
                    net: net.id.clone(),
                    inputs: signal_inputs,
                });
            }

            let power_inputs = connected
                .iter()
                .filter(|endpoint| endpoint.kind == PinElectricalKind::PowerInput)
                .cloned()
                .collect::<Vec<_>>();
            let boundary_power_source = self.ports.iter().any(|port| {
                port.net == net.id
                    && matches!(
                        port.direction,
                        PortDirection::PowerInput | PortDirection::Ground
                    )
            });
            let local_power_source = connected
                .iter()
                .any(|endpoint| endpoint.kind == PinElectricalKind::PowerOutput);
            if !power_inputs.is_empty()
                && !net.is_ground
                && !boundary_power_source
                && !local_power_source
            {
                issues.push(ErcIssue::UnpoweredInputs {
                    net: net.id.clone(),
                    inputs: power_inputs,
                });
            }
        }

        for port in &self.ports {
            if port.direction == PortDirection::Ground
                && self
                    .nets
                    .iter()
                    .find(|net| net.id == port.net)
                    .is_some_and(|net| !net.is_ground)
            {
                issues.push(ErcIssue::GroundPortOnSignal {
                    port: port.id.clone(),
                    net: port.net.clone(),
                });
            }
        }
        ErcReport { issues }
    }

    /// Runs ERC and applies a stable, configurable severity deck.
    pub fn electrical_rule_check_with(&self, deck: &ErcRuleDeck) -> ConfiguredErcReport {
        let findings = self
            .electrical_rule_check()
            .issues
            .into_iter()
            .filter_map(|issue| {
                let rule = issue.rule();
                let severity = deck.severity(rule);
                (severity != ErcSeverity::Ignore).then_some(ErcFinding {
                    rule,
                    severity,
                    issue,
                })
            })
            .collect();
        ConfiguredErcReport { findings }
    }
}

impl CircuitLibrary {
    /// Runs CopperTrace-compatible global-net, power-rail, and role-contract ERC.
    pub fn coppertrace_erc(&self, intent: &DesignIntent) -> CopperTraceErcReport {
        let intent_report = intent.validate(self);
        if !intent_report.is_valid() {
            return CopperTraceErcReport {
                structural: None,
                issues: vec![CopperTraceErcIssue::InvalidDesignIntent(
                    intent_report.issues,
                )],
            };
        }
        let flattening = match self.flatten_with_intent(intent) {
            Ok(report) => report,
            Err(error) => {
                return CopperTraceErcReport {
                    structural: None,
                    issues: vec![CopperTraceErcIssue::IntentHierarchy(error)],
                };
            }
        };
        let structural = flattening.circuit.electrical_rule_check();
        let mut issues = role_contract_issues(intent);
        issues.extend(role_part_issues(intent));

        let supply_nets = intent
            .nets
            .iter()
            .filter(|net| net.kind == NetKind::PowerSupply)
            .flat_map(|net| flattened_nets_for(self, &flattening, &net.circuit, &net.net))
            .collect::<BTreeSet<_>>();
        let mut physical_sources = BTreeMap::<NetId, BTreeSet<SupplySource>>::new();
        for instance in &flattening.circuit.instances {
            let Some(model) = flattening
                .circuit
                .device_models
                .iter()
                .find(|model| model.id == instance.model)
            else {
                continue;
            };
            for binding in &instance.pins {
                if model
                    .pins
                    .iter()
                    .any(|pin| pin.pin == binding.pin && pin.kind == PinElectricalKind::PowerOutput)
                {
                    physical_sources
                        .entry(binding.net.clone())
                        .or_default()
                        .insert(SupplySource::ComponentPin {
                            instance: instance.id.clone(),
                            pin: binding.pin.clone(),
                        });
                }
            }
        }
        for port in &flattening.circuit.ports {
            if port.direction == PortDirection::PowerInput {
                physical_sources
                    .entry(port.net.clone())
                    .or_default()
                    .insert(SupplySource::BoundaryPort(port.id.clone()));
            }
        }

        let mut role_sources = BTreeMap::<NetId, BTreeSet<SupplySource>>::new();
        for assignment in &intent.roles {
            if !matches!(
                assignment.role,
                FunctionalRole::BuckRegulator | FunctionalRole::LdoRegulator
            ) {
                continue;
            }
            let Some(binding) = assignment
                .bindings
                .iter()
                .find(|binding| canonical_binding_name(&binding.name) == "output")
            else {
                continue;
            };
            let FunctionalBindingTarget::Net { circuit, net } = &binding.target else {
                continue;
            };
            for flattened in flattened_nets_for(self, &flattening, circuit, net) {
                role_sources
                    .entry(flattened)
                    .or_default()
                    .insert(SupplySource::FunctionalRole {
                        circuit: role_definition_circuit(&assignment.target),
                        role: assignment.role.clone(),
                    });
            }
        }

        for net in supply_nets {
            let physical = physical_sources.remove(&net).unwrap_or_default();
            let sources = if physical.is_empty() {
                role_sources.remove(&net).unwrap_or_default()
            } else {
                physical
            };
            if sources.len() != 1 {
                issues.push(CopperTraceErcIssue::PowerSupplySourceCount {
                    net,
                    sources: sources.into_iter().collect(),
                });
            }
        }

        CopperTraceErcReport {
            structural: Some(structural),
            issues,
        }
    }
}

fn flattened_nets_for(
    library: &CircuitLibrary,
    flattening: &crate::CircuitFlatteningReport,
    circuit: &CircuitId,
    net: &NetId,
) -> Vec<NetId> {
    if &library.root == circuit {
        return vec![net.clone()];
    }
    flattening
        .scopes
        .iter()
        .filter(|scope| &scope.circuit == circuit)
        .filter_map(|scope| scope.nets.get(net).cloned())
        .collect()
}

fn role_contract_issues(intent: &DesignIntent) -> Vec<CopperTraceErcIssue> {
    let kinds = intent
        .nets
        .iter()
        .map(|net| ((net.circuit.clone(), net.net.clone()), net.kind.clone()))
        .collect::<BTreeMap<_, _>>();
    let mut issues = Vec::new();
    for assignment in &intent.roles {
        for &(required, expected) in required_role_bindings(&assignment.role) {
            let binding = assignment
                .bindings
                .iter()
                .find(|binding| canonical_binding_name(&binding.name) == required);
            let Some(binding) = binding else {
                issues.push(CopperTraceErcIssue::MissingRoleBinding {
                    target: assignment.target.clone(),
                    role: assignment.role.clone(),
                    binding: required.into(),
                });
                continue;
            };
            let FunctionalBindingTarget::Net { circuit, net } = &binding.target else {
                continue;
            };
            let Some(actual) = kinds.get(&(circuit.clone(), net.clone())) else {
                continue;
            };
            let compatible = match expected {
                RoleNetKind::Signal => !matches!(actual, NetKind::PowerSupply | NetKind::Ground),
                RoleNetKind::Supply => actual == &NetKind::PowerSupply,
                RoleNetKind::Ground => actual == &NetKind::Ground,
            };
            if !compatible {
                issues.push(CopperTraceErcIssue::RoleBindingKindMismatch {
                    target: assignment.target.clone(),
                    role: assignment.role.clone(),
                    binding: required.into(),
                    expected,
                    actual: actual.clone(),
                });
            }
        }
    }
    issues
}

fn role_part_issues(intent: &DesignIntent) -> Vec<CopperTraceErcIssue> {
    let evidence = intent
        .resolved_parts
        .iter()
        .map(|part| ((part.circuit.clone(), part.instance.clone()), part))
        .collect::<BTreeMap<_, _>>();
    let mut issues = Vec::new();
    for assignment in &intent.roles {
        let FunctionalRoleTarget::Instance { circuit, instance } = &assignment.target else {
            continue;
        };
        let Some(part) = evidence.get(&(circuit.clone(), instance.clone())).copied() else {
            continue;
        };
        let expected = match assignment.role {
            FunctionalRole::DecouplingCapacitor | FunctionalRole::BulkCapacitor => {
                Some(PartClass::Capacitor)
            }
            FunctionalRole::CurrentSenseResistor
            | FunctionalRole::PullupResistor
            | FunctionalRole::PulldownResistor
            | FunctionalRole::TerminationResistor => Some(PartClass::Resistor),
            _ => None,
        };
        if let Some(expected) = expected
            && part.class != expected
        {
            issues.push(CopperTraceErcIssue::RolePartClassMismatch {
                target: assignment.target.clone(),
                role: assignment.role.clone(),
                expected,
                actual: part.class.clone(),
            });
        }
        if assignment.role == FunctionalRole::CurrentSenseResistor {
            for attribute in ["resistance", "tolerance", "power-rating"] {
                if !part.attributes.contains_key(attribute) {
                    issues.push(CopperTraceErcIssue::MissingCurrentSenseAttribute {
                        target: assignment.target.clone(),
                        attribute: attribute.into(),
                    });
                }
            }
        }
    }
    issues
}

fn role_definition_circuit(target: &FunctionalRoleTarget) -> CircuitId {
    match target {
        FunctionalRoleTarget::Circuit(circuit)
        | FunctionalRoleTarget::Instance { circuit, .. }
        | FunctionalRoleTarget::Subcircuit { circuit, .. } => circuit.clone(),
    }
}

fn canonical_binding_name(name: &str) -> &str {
    match name {
        "in" | "vin" | "sig_in" | "line" => "input",
        "out" | "vout" | "sig_out" | "load" => "output",
        "ref" | "gnd" => "reference",
        other => other,
    }
}

fn required_role_bindings(role: &FunctionalRole) -> &'static [(&'static str, RoleNetKind)] {
    use RoleNetKind::{Ground, Signal, Supply};
    match role {
        FunctionalRole::LowpassFilter
        | FunctionalRole::HighpassFilter
        | FunctionalRole::BandpassFilter => {
            &[("input", Signal), ("output", Signal), ("reference", Ground)]
        }
        FunctionalRole::EmiFilter => &[("input", Signal), ("output", Signal)],
        FunctionalRole::BuckRegulator | FunctionalRole::LdoRegulator => {
            &[("input", Supply), ("output", Supply), ("reference", Ground)]
        }
        FunctionalRole::VoltageDivider => {
            &[("input", Signal), ("output", Signal), ("reference", Ground)]
        }
        FunctionalRole::CrystalOscillator => {
            &[("input", Signal), ("output", Signal), ("reference", Ground)]
        }
        FunctionalRole::OpAmpStage => {
            &[("input", Signal), ("output", Signal), ("reference", Ground)]
        }
        FunctionalRole::DecouplingCapacitor => &[
            ("supply", Supply),
            ("reference", Ground),
            ("target", Supply),
        ],
        FunctionalRole::BulkCapacitor => &[("supply", Supply), ("reference", Ground)],
        FunctionalRole::CurrentSenseResistor => &[("input", Supply), ("output", Supply)],
        FunctionalRole::PullupResistor => &[("signal", Signal), ("supply", Supply)],
        FunctionalRole::PulldownResistor => &[("signal", Signal), ("reference", Ground)],
        FunctionalRole::TerminationResistor => &[("signal", Signal), ("reference", Ground)],
        FunctionalRole::Extension { .. } => &[],
    }
}

impl ErcIssue {
    /// Stable rule identity for severity policy and report interchange.
    pub const fn rule(&self) -> ErcRuleId {
        match self {
            Self::BoundNotConnectedPin(_) => ErcRuleId::BoundNotConnectedPin,
            Self::MultiplePushPullDrivers { .. } => ErcRuleId::MultiplePushPullDrivers,
            Self::UndrivenInputs { .. } => ErcRuleId::UndrivenInputs,
            Self::UnpoweredInputs { .. } => ErcRuleId::UnpoweredInputs,
            Self::GroundPortOnSignal { .. } => ErcRuleId::GroundPortOnSignal,
        }
    }
}
