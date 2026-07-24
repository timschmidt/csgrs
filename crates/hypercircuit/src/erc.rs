//! Electrical-rule checking over retained device pins, ports, and nets.

use std::collections::BTreeMap;

use crate::{Circuit, CircuitInstanceId, NetId, PinElectricalKind, PinRef, PortDirection, PortId};

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
