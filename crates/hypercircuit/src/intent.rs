//! Language-neutral authored intent retained beside circuit and PCB semantics.
//!
//! Front ends such as CopperTrace elaborate syntax and expressions before
//! constructing HyperCircuit carriers. This module preserves the facts that
//! must survive that lowering for diagnostics, hierarchy, ERC, DRC, analysis,
//! and registry audit without teaching HyperCircuit a source-language parser.

use std::collections::{BTreeMap, BTreeSet};

use hyperreal::Real;

use crate::{
    CircuitId, CircuitInstanceId, CircuitLibrary, NetClassId, NetId, PartRef, PinRef, PortId,
    SubcircuitInstanceId,
};

/// One exact position in a source artifact.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub struct SourcePosition {
    /// Zero-based byte offset in the source artifact.
    pub byte: u64,
    /// One-based source line.
    pub line: u32,
    /// One-based source column.
    pub column: u32,
}

impl SourcePosition {
    /// Constructs a source position.
    pub const fn new(byte: u64, line: u32, column: u32) -> Self {
        Self { byte, line, column }
    }

    /// True when line and column coordinates are usable by editors.
    pub const fn is_valid(self) -> bool {
        self.line != 0 && self.column != 0
    }
}

/// Half-open source range associated with authored semantic intent.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub struct SourceSpan {
    /// File path, URI, registry locator, or other source identity.
    pub uri: String,
    /// Inclusive start position.
    pub start: SourcePosition,
    /// Exclusive end position.
    pub end: SourcePosition,
}

impl SourceSpan {
    /// Constructs one source range.
    pub fn new(uri: impl Into<String>, start: SourcePosition, end: SourcePosition) -> Self {
        Self {
            uri: uri.into(),
            start,
            end,
        }
    }

    /// True when the URI and half-open coordinates are internally consistent.
    pub fn is_valid(&self) -> bool {
        !self.uri.trim().is_empty()
            && self.start.is_valid()
            && self.end.is_valid()
            && self.start <= self.end
    }
}

/// Stable semantic subject that can be traced back to authored source.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub enum SemanticTarget {
    /// Whole circuit definition.
    Circuit(CircuitId),
    /// Net in one circuit definition.
    Net { circuit: CircuitId, net: NetId },
    /// Boundary port in one circuit definition.
    Port { circuit: CircuitId, port: PortId },
    /// Component instance in one circuit definition.
    Instance {
        circuit: CircuitId,
        instance: CircuitInstanceId,
    },
    /// Pin on one component instance.
    Pin {
        circuit: CircuitId,
        instance: CircuitInstanceId,
        pin: PinRef,
    },
    /// Child-circuit instance in its direct parent definition.
    Subcircuit {
        circuit: CircuitId,
        instance: SubcircuitInstanceId,
    },
    /// Named PCB net class.
    NetClass(NetClassId),
    /// Front-end declaration not represented by a narrower core identity.
    Declaration(String),
}

/// One source range and optional front-end label for a semantic subject.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct SemanticOrigin {
    /// Retained semantic subject.
    pub target: SemanticTarget,
    /// Exact source range.
    pub span: SourceSpan,
    /// Optional declaration/decorator/expression label.
    pub label: Option<String>,
}

/// Physical dimension of one exact elaborated quantity.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub enum QuantityDimension {
    /// Unitless scalar.
    Dimensionless,
    /// Length.
    Length,
    /// Electric potential.
    Voltage,
    /// Electric current.
    Current,
    /// Resistance.
    Resistance,
    /// Capacitance.
    Capacitance,
    /// Inductance.
    Inductance,
    /// Frequency.
    Frequency,
    /// Time.
    Time,
    /// Power.
    Power,
    /// Front-end or domain extension with a stable nonempty name.
    Custom(String),
}

/// Exact elaborated value with retained dimension and canonical unit label.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, PartialEq)]
pub struct DimensionedValue {
    /// Exact value expressed in `unit`.
    pub value: Real,
    /// Physical dimension checked by the elaborating front end.
    pub dimension: QuantityDimension,
    /// Canonical unit label, such as `V`, `ohm`, `F`, `Hz`, or `mm`.
    pub unit: String,
}

impl DimensionedValue {
    /// Constructs an exact typed quantity.
    pub fn new(value: Real, dimension: QuantityDimension, unit: impl Into<String>) -> Self {
        Self {
            value,
            dimension,
            unit: unit.into(),
        }
    }

    /// True when extension names and the canonical unit are usable.
    pub fn is_valid(&self) -> bool {
        !self.unit.trim().is_empty()
            && !matches!(&self.dimension, QuantityDimension::Custom(name) if name.trim().is_empty())
    }
}

/// CopperTrace-compatible semantic kind of an electrical net.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub enum NetKind {
    /// Unrefined electrical node.
    Generic,
    /// Supply rail expected to have exactly one source.
    PowerSupply,
    /// Shared or named return/reference node.
    Ground,
    /// Routed digital logic signal.
    DigitalSignal,
    /// Analog or measurement signal.
    AnalogSignal,
    /// One member of an explicitly declared differential pair.
    DifferentialPairMember,
    /// Versioned front-end/domain extension.
    Extension {
        /// Namespace defining the extension.
        namespace: String,
        /// Stable kind name.
        name: String,
        /// Catalog/schema version.
        version: String,
    },
}

impl NetKind {
    fn is_valid(&self) -> bool {
        !matches!(
            self,
            Self::Extension {
                namespace,
                name,
                version
            } if namespace.trim().is_empty() || name.trim().is_empty() || version.trim().is_empty()
        )
    }
}

/// Hierarchical visibility of one net declaration.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub enum NetScope {
    /// Net remains local unless connected through a boundary port.
    Local,
    /// Net joins every instantiated declaration carrying the same global key.
    Global(String),
}

/// Authored semantics attached to one retained circuit net.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, PartialEq)]
pub struct NetIntent {
    /// Circuit definition owning the declaration.
    pub circuit: CircuitId,
    /// Existing local net.
    pub net: NetId,
    /// Refined electrical kind.
    pub kind: NetKind,
    /// Hierarchical visibility.
    pub scope: NetScope,
    /// Optional physical routing/DRC class.
    pub net_class: Option<NetClassId>,
    /// Optional exact nominal electrical value.
    pub nominal_value: Option<DimensionedValue>,
}

/// Curated functional role understood by HyperCircuit and HyperDRC.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub enum FunctionalRole {
    LowpassFilter,
    HighpassFilter,
    BandpassFilter,
    EmiFilter,
    BuckRegulator,
    LdoRegulator,
    VoltageDivider,
    CrystalOscillator,
    OpAmpStage,
    DecouplingCapacitor,
    BulkCapacitor,
    CurrentSenseResistor,
    PullupResistor,
    PulldownResistor,
    TerminationResistor,
    /// Versioned role supplied by an extension catalog.
    Extension {
        namespace: String,
        name: String,
        version: String,
    },
}

impl FunctionalRole {
    fn is_valid(&self) -> bool {
        !matches!(
            self,
            Self::Extension {
                namespace,
                name,
                version
            } if namespace.trim().is_empty() || name.trim().is_empty() || version.trim().is_empty()
        )
    }
}

/// Semantic object implementing one functional role.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub enum FunctionalRoleTarget {
    /// Reusable circuit definition.
    Circuit(CircuitId),
    /// Component instance within one definition.
    Instance {
        circuit: CircuitId,
        instance: CircuitInstanceId,
    },
    /// Child-circuit instance in its parent definition.
    Subcircuit {
        circuit: CircuitId,
        instance: SubcircuitInstanceId,
    },
}

/// Net or pin bound to a named function-contract terminal.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub enum FunctionalBindingTarget {
    /// Existing net in a circuit definition.
    Net { circuit: CircuitId, net: NetId },
    /// Existing component pin in a circuit definition.
    Pin {
        circuit: CircuitId,
        instance: CircuitInstanceId,
        pin: PinRef,
    },
}

/// One named interface/analysis binding in a functional-role contract.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub struct FunctionalBinding {
    /// Contract-local terminal name, such as `supply`, `ground`, or `output`.
    pub name: String,
    /// Concrete semantic endpoint.
    pub target: FunctionalBindingTarget,
}

/// Authored role assignment with explicit contract bindings and exact parameters.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, PartialEq)]
pub struct FunctionalRoleAssignment {
    /// Object implementing the role.
    pub target: FunctionalRoleTarget,
    /// Curated or versioned extension role.
    pub role: FunctionalRole,
    /// Explicit role-interface bindings.
    pub bindings: Vec<FunctionalBinding>,
    /// Exact analysis or suitability parameters.
    pub parameters: BTreeMap<String, DimensionedValue>,
}

/// Broad physical/electrical class asserted for a resolved component.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub enum PartClass {
    Resistor,
    Capacitor,
    Inductor,
    Ic,
    Diode,
    Transistor,
    Connector,
    Crystal,
    /// Versioned class supplied by an extension catalog.
    Extension {
        namespace: String,
        name: String,
        version: String,
    },
}

impl PartClass {
    fn is_valid(&self) -> bool {
        !matches!(
            self,
            Self::Extension {
                namespace,
                name,
                version
            } if namespace.trim().is_empty() || name.trim().is_empty() || version.trim().is_empty()
        )
    }
}

/// How an elaborating front end selected a concrete part.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, PartialEq)]
pub enum PartSelectionIntent {
    /// Canonical exact registry path such as `CM::LCSC::...`.
    ExactRegistryPath {
        registry: String,
        catalog: String,
        part_id: String,
    },
    /// Bare typed value resolved under project policy.
    BareValue(DimensionedValue),
    /// Deterministic parametric query expression.
    Query {
        expression: String,
        requested: Option<DimensionedValue>,
    },
    /// Named source alias resolved earlier in the same compilation.
    Alias(String),
}

impl PartSelectionIntent {
    fn is_valid(&self) -> bool {
        match self {
            Self::ExactRegistryPath {
                registry,
                catalog,
                part_id,
            } => {
                !registry.trim().is_empty()
                    && !catalog.trim().is_empty()
                    && !part_id.trim().is_empty()
            }
            Self::BareValue(value) => value.is_valid(),
            Self::Query {
                expression,
                requested,
            } => {
                !expression.trim().is_empty()
                    && requested.as_ref().is_none_or(DimensionedValue::is_valid)
            }
            Self::Alias(alias) => !alias.trim().is_empty(),
        }
    }
}

/// Auditable evidence that one component resolved to a concrete part.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, PartialEq)]
pub struct ResolvedPartEvidence {
    /// Circuit definition containing the component instance.
    pub circuit: CircuitId,
    /// Resolved component instance.
    pub instance: CircuitInstanceId,
    /// Checked broad part class.
    pub class: PartClass,
    /// Authored selection form.
    pub selection: PartSelectionIntent,
    /// Concrete retained external part reference.
    pub resolved: PartRef,
    /// Optional immutable registry/package artifact digest.
    pub artifact_digest: Option<String>,
    /// Deterministic project policy inputs used during resolution.
    pub policy: BTreeMap<String, String>,
    /// Typed resolved attributes used by role suitability checks.
    pub attributes: BTreeMap<String, DimensionedValue>,
}

/// Versioned semantic metadata retained from an elaborating authoring language.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, Default, PartialEq)]
pub struct DesignIntent {
    /// Source ranges for declarations, decorators, and expressions.
    pub origins: Vec<SemanticOrigin>,
    /// Refined net kinds, scopes, classes, and values.
    pub nets: Vec<NetIntent>,
    /// Component/subcircuit functional roles and their contracts.
    pub roles: Vec<FunctionalRoleAssignment>,
    /// Concrete part-resolution audit records.
    pub resolved_parts: Vec<ResolvedPartEvidence>,
}

/// Invalid or stale retained semantic intent.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum DesignIntentIssue {
    InvalidSourceSpan(usize),
    InvalidOriginLabel(usize),
    UnknownOriginTarget(usize),
    DuplicateNetIntent {
        circuit: CircuitId,
        net: NetId,
    },
    UnknownIntentCircuit(CircuitId),
    UnknownIntentNet {
        circuit: CircuitId,
        net: NetId,
    },
    InvalidNetKind {
        circuit: CircuitId,
        net: NetId,
    },
    InvalidGlobalName {
        circuit: CircuitId,
        net: NetId,
    },
    GroundKindMismatch {
        circuit: CircuitId,
        net: NetId,
    },
    ConflictingGlobalNet(String),
    InvalidNominalValue {
        circuit: CircuitId,
        net: NetId,
    },
    DuplicateRoleTarget(FunctionalRoleTarget),
    InvalidFunctionalRole(FunctionalRoleTarget),
    InvalidFunctionalBindingName {
        target: FunctionalRoleTarget,
        name: String,
    },
    DuplicateFunctionalBinding {
        target: FunctionalRoleTarget,
        name: String,
    },
    UnknownFunctionalTarget(FunctionalRoleTarget),
    UnknownFunctionalBinding {
        target: FunctionalRoleTarget,
        binding: String,
    },
    InvalidRoleParameter {
        target: FunctionalRoleTarget,
        parameter: String,
    },
    DuplicateResolvedPart {
        circuit: CircuitId,
        instance: CircuitInstanceId,
    },
    UnknownResolvedPartInstance {
        circuit: CircuitId,
        instance: CircuitInstanceId,
    },
    ResolvedPartMismatch {
        circuit: CircuitId,
        instance: CircuitInstanceId,
    },
    InvalidPartClass {
        circuit: CircuitId,
        instance: CircuitInstanceId,
    },
    InvalidPartSelection {
        circuit: CircuitId,
        instance: CircuitInstanceId,
    },
    InvalidPartDigest {
        circuit: CircuitId,
        instance: CircuitInstanceId,
    },
    InvalidPartPolicy {
        circuit: CircuitId,
        instance: CircuitInstanceId,
        key: String,
    },
    InvalidPartAttribute {
        circuit: CircuitId,
        instance: CircuitInstanceId,
        attribute: String,
    },
}

/// Deterministic validation result for retained semantic intent.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct DesignIntentValidationReport {
    pub issues: Vec<DesignIntentIssue>,
}

impl DesignIntentValidationReport {
    pub fn is_valid(&self) -> bool {
        self.issues.is_empty()
    }
}

impl DesignIntent {
    /// Validates references and metadata against the complete retained hierarchy.
    pub fn validate(&self, library: &CircuitLibrary) -> DesignIntentValidationReport {
        let circuits = library
            .circuits
            .iter()
            .map(|circuit| (circuit.id.clone(), circuit))
            .collect::<BTreeMap<_, _>>();
        let mut issues = Vec::new();

        for (index, origin) in self.origins.iter().enumerate() {
            if !origin.span.is_valid() {
                issues.push(DesignIntentIssue::InvalidSourceSpan(index));
            }
            if origin
                .label
                .as_ref()
                .is_some_and(|label| label.trim().is_empty())
            {
                issues.push(DesignIntentIssue::InvalidOriginLabel(index));
            }
            if !semantic_target_exists(&origin.target, &circuits) {
                issues.push(DesignIntentIssue::UnknownOriginTarget(index));
            }
        }

        let mut net_targets = BTreeSet::new();
        for intent in &self.nets {
            if !net_targets.insert((intent.circuit.clone(), intent.net.clone())) {
                issues.push(DesignIntentIssue::DuplicateNetIntent {
                    circuit: intent.circuit.clone(),
                    net: intent.net.clone(),
                });
            }
            let Some(circuit) = circuits.get(&intent.circuit).copied() else {
                issues.push(DesignIntentIssue::UnknownIntentCircuit(
                    intent.circuit.clone(),
                ));
                continue;
            };
            let Some(net) = circuit.nets.iter().find(|net| net.id == intent.net) else {
                issues.push(DesignIntentIssue::UnknownIntentNet {
                    circuit: intent.circuit.clone(),
                    net: intent.net.clone(),
                });
                continue;
            };
            if !intent.kind.is_valid() {
                issues.push(DesignIntentIssue::InvalidNetKind {
                    circuit: intent.circuit.clone(),
                    net: intent.net.clone(),
                });
            }
            if matches!(&intent.scope, NetScope::Global(name) if name.trim().is_empty()) {
                issues.push(DesignIntentIssue::InvalidGlobalName {
                    circuit: intent.circuit.clone(),
                    net: intent.net.clone(),
                });
            }
            if net.is_ground && intent.kind != NetKind::Ground {
                issues.push(DesignIntentIssue::GroundKindMismatch {
                    circuit: intent.circuit.clone(),
                    net: intent.net.clone(),
                });
            }
            if intent
                .nominal_value
                .as_ref()
                .is_some_and(|value| !value.is_valid())
            {
                issues.push(DesignIntentIssue::InvalidNominalValue {
                    circuit: intent.circuit.clone(),
                    net: intent.net.clone(),
                });
            }
        }
        let mut global_signatures =
            BTreeMap::<String, (NetKind, Option<NetClassId>, Option<DimensionedValue>)>::new();
        for intent in &self.nets {
            let NetScope::Global(key) = &intent.scope else {
                continue;
            };
            let signature = (
                intent.kind.clone(),
                intent.net_class.clone(),
                intent.nominal_value.clone(),
            );
            if global_signatures
                .insert(key.clone(), signature.clone())
                .is_some_and(|previous| previous != signature)
            {
                issues.push(DesignIntentIssue::ConflictingGlobalNet(key.clone()));
            }
        }

        let mut role_targets = BTreeSet::new();
        for assignment in &self.roles {
            if !role_targets.insert(assignment.target.clone()) {
                issues.push(DesignIntentIssue::DuplicateRoleTarget(
                    assignment.target.clone(),
                ));
            }
            if !assignment.role.is_valid() {
                issues.push(DesignIntentIssue::InvalidFunctionalRole(
                    assignment.target.clone(),
                ));
            }
            if !functional_target_exists(&assignment.target, &circuits) {
                issues.push(DesignIntentIssue::UnknownFunctionalTarget(
                    assignment.target.clone(),
                ));
            }
            let mut names = BTreeSet::new();
            for binding in &assignment.bindings {
                if binding.name.trim().is_empty() {
                    issues.push(DesignIntentIssue::InvalidFunctionalBindingName {
                        target: assignment.target.clone(),
                        name: binding.name.clone(),
                    });
                }
                if !names.insert(binding.name.clone()) {
                    issues.push(DesignIntentIssue::DuplicateFunctionalBinding {
                        target: assignment.target.clone(),
                        name: binding.name.clone(),
                    });
                }
                if !functional_binding_exists(&binding.target, &circuits) {
                    issues.push(DesignIntentIssue::UnknownFunctionalBinding {
                        target: assignment.target.clone(),
                        binding: binding.name.clone(),
                    });
                }
            }
            for (parameter, value) in &assignment.parameters {
                if parameter.trim().is_empty() || !value.is_valid() {
                    issues.push(DesignIntentIssue::InvalidRoleParameter {
                        target: assignment.target.clone(),
                        parameter: parameter.clone(),
                    });
                }
            }
        }

        let mut resolved_targets = BTreeSet::new();
        for evidence in &self.resolved_parts {
            if !resolved_targets.insert((evidence.circuit.clone(), evidence.instance.clone())) {
                issues.push(DesignIntentIssue::DuplicateResolvedPart {
                    circuit: evidence.circuit.clone(),
                    instance: evidence.instance.clone(),
                });
            }
            let instance = circuits.get(&evidence.circuit).and_then(|circuit| {
                circuit
                    .instances
                    .iter()
                    .find(|instance| instance.id == evidence.instance)
            });
            let Some(instance) = instance else {
                issues.push(DesignIntentIssue::UnknownResolvedPartInstance {
                    circuit: evidence.circuit.clone(),
                    instance: evidence.instance.clone(),
                });
                continue;
            };
            if instance.part.as_ref() != Some(&evidence.resolved) {
                issues.push(DesignIntentIssue::ResolvedPartMismatch {
                    circuit: evidence.circuit.clone(),
                    instance: evidence.instance.clone(),
                });
            }
            if !evidence.class.is_valid() {
                issues.push(DesignIntentIssue::InvalidPartClass {
                    circuit: evidence.circuit.clone(),
                    instance: evidence.instance.clone(),
                });
            }
            if !evidence.selection.is_valid() {
                issues.push(DesignIntentIssue::InvalidPartSelection {
                    circuit: evidence.circuit.clone(),
                    instance: evidence.instance.clone(),
                });
            }
            if evidence
                .artifact_digest
                .as_ref()
                .is_some_and(|digest| digest.trim().is_empty())
            {
                issues.push(DesignIntentIssue::InvalidPartDigest {
                    circuit: evidence.circuit.clone(),
                    instance: evidence.instance.clone(),
                });
            }
            for (key, value) in &evidence.policy {
                if key.trim().is_empty() || value.trim().is_empty() {
                    issues.push(DesignIntentIssue::InvalidPartPolicy {
                        circuit: evidence.circuit.clone(),
                        instance: evidence.instance.clone(),
                        key: key.clone(),
                    });
                }
            }
            for (attribute, value) in &evidence.attributes {
                if attribute.trim().is_empty() || !value.is_valid() {
                    issues.push(DesignIntentIssue::InvalidPartAttribute {
                        circuit: evidence.circuit.clone(),
                        instance: evidence.instance.clone(),
                        attribute: attribute.clone(),
                    });
                }
            }
        }

        DesignIntentValidationReport { issues }
    }
}

fn semantic_target_exists(
    target: &SemanticTarget,
    circuits: &BTreeMap<CircuitId, &crate::Circuit>,
) -> bool {
    match target {
        SemanticTarget::Circuit(circuit) => circuits.contains_key(circuit),
        SemanticTarget::Net { circuit, net } => circuits
            .get(circuit)
            .is_some_and(|circuit| circuit.nets.iter().any(|candidate| candidate.id == *net)),
        SemanticTarget::Port { circuit, port } => circuits
            .get(circuit)
            .is_some_and(|circuit| circuit.ports.iter().any(|candidate| candidate.id == *port)),
        SemanticTarget::Instance { circuit, instance }
        | SemanticTarget::Pin {
            circuit, instance, ..
        } => circuits.get(circuit).is_some_and(|circuit| {
            circuit
                .instances
                .iter()
                .any(|candidate| candidate.id == *instance)
        }),
        SemanticTarget::Subcircuit { circuit, instance } => {
            circuits.get(circuit).is_some_and(|circuit| {
                circuit
                    .subcircuits
                    .iter()
                    .any(|candidate| candidate.id == *instance)
            })
        }
        SemanticTarget::NetClass(_) => true,
        SemanticTarget::Declaration(name) => !name.trim().is_empty(),
    }
}

fn functional_target_exists(
    target: &FunctionalRoleTarget,
    circuits: &BTreeMap<CircuitId, &crate::Circuit>,
) -> bool {
    match target {
        FunctionalRoleTarget::Circuit(circuit) => circuits.contains_key(circuit),
        FunctionalRoleTarget::Instance { circuit, instance } => circuits
            .get(circuit)
            .is_some_and(|circuit| circuit.instances.iter().any(|item| item.id == *instance)),
        FunctionalRoleTarget::Subcircuit { circuit, instance } => circuits
            .get(circuit)
            .is_some_and(|circuit| circuit.subcircuits.iter().any(|item| item.id == *instance)),
    }
}

fn functional_binding_exists(
    target: &FunctionalBindingTarget,
    circuits: &BTreeMap<CircuitId, &crate::Circuit>,
) -> bool {
    match target {
        FunctionalBindingTarget::Net { circuit, net } => circuits
            .get(circuit)
            .is_some_and(|circuit| circuit.nets.iter().any(|item| item.id == *net)),
        FunctionalBindingTarget::Pin {
            circuit,
            instance,
            pin,
        } => circuits.get(circuit).is_some_and(|circuit| {
            let Some(instance) = circuit.instances.iter().find(|item| item.id == *instance) else {
                return false;
            };
            let Some(model) = circuit
                .device_models
                .iter()
                .find(|model| model.id == instance.model)
            else {
                return false;
            };
            model.pins.iter().any(|item| item.pin == *pin)
        }),
    }
}
