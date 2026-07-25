//! Named signal bundles and directional views for hierarchical composition.
//!
//! The retained circuit IR continues to use individual [`CircuitPort`] and
//! [`SubcircuitPortBinding`] records. This layer adds nominal, ordered bundle
//! contracts over those records and deliberately lowers a checked bundle
//! connection back into the existing hierarchy representation.

use std::collections::{BTreeMap, BTreeSet};

use crate::{
    BundleEndpointId, BundleMemberId, Circuit, CircuitId, CircuitLibrary, ModportId, NetId,
    PortDirection, PortId, PortSignalType, SignalBundleId, SubcircuitInstanceId,
    SubcircuitPortBinding,
};

/// One member's electrical direction in a modport view.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ModportMember {
    /// Named member in the containing signal bundle.
    pub member: BundleMemberId,
    /// Direction presented by an endpoint selecting this view.
    pub direction: PortDirection,
}

/// Reusable directional view over every member of a signal bundle.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Modport {
    /// Identity unique within the containing bundle.
    pub id: ModportId,
    /// One direction for each bundle member.
    pub members: Vec<ModportMember>,
}

impl Modport {
    /// Creates a directional view.
    pub fn new(id: ModportId, members: Vec<ModportMember>) -> Self {
        Self { id, members }
    }

    /// Derives the conjugate view by flipping every member direction.
    pub fn dual(&self, id: ModportId) -> Self {
        Self {
            id,
            members: self
                .members
                .iter()
                .map(|member| ModportMember {
                    member: member.member.clone(),
                    direction: member.direction.dual(),
                })
                .collect(),
        }
    }

    /// Returns the direction assigned to a named member.
    pub fn direction(&self, member: &BundleMemberId) -> Option<PortDirection> {
        self.members
            .iter()
            .find(|candidate| &candidate.member == member)
            .map(|candidate| candidate.direction)
    }
}

/// One named, typed member of a nominal signal bundle.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub struct SignalBundleMember {
    /// Stable member identity within the bundle.
    pub id: BundleMemberId,
    /// Logical value shape carried by the member.
    pub signal_type: PortSignalType,
}

impl SignalBundleMember {
    /// Creates a typed bundle member.
    pub fn new(id: BundleMemberId, signal_type: PortSignalType) -> Self {
        Self { id, signal_type }
    }
}

/// Nominal ordered group of named, typed signals and its directional views.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct SignalBundle {
    /// Stable nominal bundle identity.
    pub id: SignalBundleId,
    /// Ordered typed structural members.
    pub members: Vec<SignalBundleMember>,
    /// Reusable directional views of those members.
    pub modports: Vec<Modport>,
}

impl SignalBundle {
    /// Creates a continuous-real bundle definition without directional views.
    pub fn new(id: SignalBundleId, members: Vec<BundleMemberId>) -> Self {
        Self {
            id,
            members: members
                .into_iter()
                .map(|id| SignalBundleMember::new(id, PortSignalType::Real))
                .collect(),
            modports: Vec::new(),
        }
    }

    /// Creates a typed bundle definition without directional views.
    pub fn new_typed(id: SignalBundleId, members: Vec<SignalBundleMember>) -> Self {
        Self {
            id,
            members,
            modports: Vec::new(),
        }
    }

    /// Adds one allowed directional view.
    pub fn with_modport(mut self, modport: Modport) -> Self {
        self.modports.push(modport);
        self
    }

    /// Finds a directional view by its bundle-local identity.
    pub fn modport(&self, id: &ModportId) -> Option<&Modport> {
        self.modports.iter().find(|candidate| &candidate.id == id)
    }
}

/// Mapping from one bundle member to an ordinary circuit boundary port.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct BundlePortBinding {
    /// Member in the selected nominal bundle.
    pub member: BundleMemberId,
    /// Existing boundary port in the endpoint's circuit.
    pub port: PortId,
}

/// A circuit boundary exposing one directional view of a signal bundle.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct SignalBundleEndpoint {
    /// Stable endpoint identity within the circuit.
    pub id: BundleEndpointId,
    /// Circuit owning the ordinary ports.
    pub circuit: CircuitId,
    /// Nominal bundle type.
    pub bundle: SignalBundleId,
    /// Directional view presented by this endpoint.
    pub modport: ModportId,
    /// Complete member-to-port mapping.
    pub ports: Vec<BundlePortBinding>,
}

impl SignalBundleEndpoint {
    /// Creates a circuit endpoint over existing boundary ports.
    pub fn new(
        id: BundleEndpointId,
        circuit: CircuitId,
        bundle: SignalBundleId,
        modport: ModportId,
        ports: Vec<BundlePortBinding>,
    ) -> Self {
        Self {
            id,
            circuit,
            bundle,
            modport,
            ports,
        }
    }
}

/// Structural issue in bundle definitions or circuit endpoint declarations.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum SignalBundleValidationIssue {
    /// Two nominal bundle definitions share an identity.
    DuplicateBundle(SignalBundleId),
    /// A nominal bundle has no members.
    EmptyBundle(SignalBundleId),
    /// A nominal bundle declares one member more than once.
    DuplicateBundleMember {
        bundle: SignalBundleId,
        member: BundleMemberId,
    },
    /// A packed bundle member has an explicit zero width.
    InvalidBundleMemberSignalType {
        bundle: SignalBundleId,
        member: BundleMemberId,
    },
    /// Two views in one bundle share an identity.
    DuplicateModport {
        bundle: SignalBundleId,
        modport: ModportId,
    },
    /// A view references a member absent from its bundle.
    UnknownModportMember {
        bundle: SignalBundleId,
        modport: ModportId,
        member: BundleMemberId,
    },
    /// A view assigns one member more than once.
    DuplicateModportMember {
        bundle: SignalBundleId,
        modport: ModportId,
        member: BundleMemberId,
    },
    /// A view omits a member from its containing bundle.
    MissingModportMember {
        bundle: SignalBundleId,
        modport: ModportId,
        member: BundleMemberId,
    },
    /// Two endpoints in one circuit share an identity.
    DuplicateEndpoint {
        circuit: CircuitId,
        endpoint: BundleEndpointId,
    },
    /// An endpoint names no circuit in the hierarchy.
    UnknownEndpointCircuit {
        endpoint: BundleEndpointId,
        circuit: CircuitId,
    },
    /// An endpoint names no nominal bundle definition.
    UnknownEndpointBundle {
        circuit: CircuitId,
        endpoint: BundleEndpointId,
        bundle: SignalBundleId,
    },
    /// An endpoint selects no view in its nominal bundle.
    UnknownEndpointModport {
        circuit: CircuitId,
        endpoint: BundleEndpointId,
        modport: ModportId,
    },
    /// An endpoint references a member absent from its nominal bundle.
    UnknownEndpointMember {
        circuit: CircuitId,
        endpoint: BundleEndpointId,
        member: BundleMemberId,
    },
    /// An endpoint binds one member more than once.
    DuplicateEndpointMember {
        circuit: CircuitId,
        endpoint: BundleEndpointId,
        member: BundleMemberId,
    },
    /// An endpoint omits a member required by its nominal bundle.
    MissingEndpointMember {
        circuit: CircuitId,
        endpoint: BundleEndpointId,
        member: BundleMemberId,
    },
    /// An endpoint binds one circuit port to multiple bundle members.
    DuplicateEndpointPort {
        circuit: CircuitId,
        endpoint: BundleEndpointId,
        port: PortId,
    },
    /// An endpoint references no boundary port in its circuit.
    UnknownEndpointPort {
        circuit: CircuitId,
        endpoint: BundleEndpointId,
        port: PortId,
    },
    /// A retained port direction does not implement its selected modport view.
    EndpointPortDirectionMismatch {
        circuit: CircuitId,
        endpoint: BundleEndpointId,
        member: BundleMemberId,
        port: PortId,
        expected: PortDirection,
        actual: PortDirection,
    },
    /// A retained port's logical type disagrees with its bundle member.
    EndpointPortSignalTypeMismatch {
        circuit: CircuitId,
        endpoint: BundleEndpointId,
        member: BundleMemberId,
        port: PortId,
        expected: PortSignalType,
        actual: PortSignalType,
    },
}

/// Deterministic validation result for bundle contracts and endpoints.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct SignalBundleValidationReport {
    /// Every discovered structural issue.
    pub issues: Vec<SignalBundleValidationIssue>,
}

impl SignalBundleValidationReport {
    /// True when all bundle contracts can be lowered safely.
    pub fn is_valid(&self) -> bool {
        self.issues.is_empty()
    }
}

/// Additive bundle-contract layer over an ordinary circuit library.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, Default, Eq, PartialEq)]
pub struct SignalBundleLibrary {
    /// Nominal bundle definitions shared by circuit modules.
    pub bundles: Vec<SignalBundle>,
    /// Directional bundle endpoints declared by circuit modules.
    pub endpoints: Vec<SignalBundleEndpoint>,
}

impl SignalBundleLibrary {
    /// Creates an empty bundle-contract library.
    pub fn new() -> Self {
        Self::default()
    }

    /// Adds one nominal bundle definition.
    pub fn with_bundle(mut self, bundle: SignalBundle) -> Self {
        self.bundles.push(bundle);
        self
    }

    /// Adds one directional circuit endpoint.
    pub fn with_endpoint(mut self, endpoint: SignalBundleEndpoint) -> Self {
        self.endpoints.push(endpoint);
        self
    }

    /// Validates nominal definitions and their lowering onto circuit ports.
    pub fn validate(&self, circuits: &CircuitLibrary) -> SignalBundleValidationReport {
        let mut issues = Vec::new();
        let mut bundles = BTreeMap::new();
        for bundle in &self.bundles {
            if bundles.insert(bundle.id.clone(), bundle).is_some() {
                issues.push(SignalBundleValidationIssue::DuplicateBundle(
                    bundle.id.clone(),
                ));
            }
            if bundle.members.is_empty() {
                issues.push(SignalBundleValidationIssue::EmptyBundle(bundle.id.clone()));
            }
            let mut members = BTreeSet::new();
            for member in &bundle.members {
                if !members.insert(member.id.clone()) {
                    issues.push(SignalBundleValidationIssue::DuplicateBundleMember {
                        bundle: bundle.id.clone(),
                        member: member.id.clone(),
                    });
                }
                if !member.signal_type.is_valid() {
                    issues.push(SignalBundleValidationIssue::InvalidBundleMemberSignalType {
                        bundle: bundle.id.clone(),
                        member: member.id.clone(),
                    });
                }
            }
            let mut modports = BTreeSet::new();
            for modport in &bundle.modports {
                if !modports.insert(modport.id.clone()) {
                    issues.push(SignalBundleValidationIssue::DuplicateModport {
                        bundle: bundle.id.clone(),
                        modport: modport.id.clone(),
                    });
                }
                let mut viewed = BTreeSet::new();
                for member in &modport.members {
                    if !members.contains(&member.member) {
                        issues.push(SignalBundleValidationIssue::UnknownModportMember {
                            bundle: bundle.id.clone(),
                            modport: modport.id.clone(),
                            member: member.member.clone(),
                        });
                    }
                    if !viewed.insert(member.member.clone()) {
                        issues.push(SignalBundleValidationIssue::DuplicateModportMember {
                            bundle: bundle.id.clone(),
                            modport: modport.id.clone(),
                            member: member.member.clone(),
                        });
                    }
                }
                for member in members.difference(&viewed) {
                    issues.push(SignalBundleValidationIssue::MissingModportMember {
                        bundle: bundle.id.clone(),
                        modport: modport.id.clone(),
                        member: member.clone(),
                    });
                }
            }
        }

        let circuit_by_id = circuits
            .circuits
            .iter()
            .map(|circuit| (circuit.id.clone(), circuit))
            .collect::<BTreeMap<_, _>>();
        let mut endpoint_ids = BTreeSet::new();
        for endpoint in &self.endpoints {
            if !endpoint_ids.insert((endpoint.circuit.clone(), endpoint.id.clone())) {
                issues.push(SignalBundleValidationIssue::DuplicateEndpoint {
                    circuit: endpoint.circuit.clone(),
                    endpoint: endpoint.id.clone(),
                });
            }
            let Some(circuit) = circuit_by_id.get(&endpoint.circuit).copied() else {
                issues.push(SignalBundleValidationIssue::UnknownEndpointCircuit {
                    endpoint: endpoint.id.clone(),
                    circuit: endpoint.circuit.clone(),
                });
                continue;
            };
            let Some(bundle) = bundles.get(&endpoint.bundle).copied() else {
                issues.push(SignalBundleValidationIssue::UnknownEndpointBundle {
                    circuit: endpoint.circuit.clone(),
                    endpoint: endpoint.id.clone(),
                    bundle: endpoint.bundle.clone(),
                });
                continue;
            };
            let Some(modport) = bundle.modport(&endpoint.modport) else {
                issues.push(SignalBundleValidationIssue::UnknownEndpointModport {
                    circuit: endpoint.circuit.clone(),
                    endpoint: endpoint.id.clone(),
                    modport: endpoint.modport.clone(),
                });
                continue;
            };
            validate_endpoint(endpoint, circuit, bundle, modport, &mut issues);
        }
        SignalBundleValidationReport { issues }
    }

    /// Connects dual bundle endpoints across one parent/child hierarchy edge.
    ///
    /// The returned bindings are also appended to the selected
    /// [`crate::SubcircuitInstance`]. Bundle identity and member names are
    /// resolved away; existing hierarchy validation, ERC, and flattening then
    /// operate on the ordinary per-port bindings.
    pub fn bind_subcircuit(
        &self,
        circuits: &mut CircuitLibrary,
        parent: &CircuitId,
        instance: &SubcircuitInstanceId,
        parent_endpoint: &BundleEndpointId,
        child_endpoint: &BundleEndpointId,
    ) -> Result<Vec<SubcircuitPortBinding>, SignalBundleBindingError> {
        let validation = self.validate(circuits);
        if !validation.is_valid() {
            return Err(SignalBundleBindingError::InvalidBundleLibrary(validation));
        }

        let parent_endpoint = self.endpoint(parent, parent_endpoint).ok_or_else(|| {
            SignalBundleBindingError::UnknownParentEndpoint {
                circuit: parent.clone(),
                endpoint: parent_endpoint.clone(),
            }
        })?;
        let parent_index = circuits
            .circuits
            .iter()
            .position(|circuit| &circuit.id == parent)
            .ok_or_else(|| SignalBundleBindingError::UnknownParentCircuit(parent.clone()))?;
        let child_id = circuits.circuits[parent_index]
            .subcircuits
            .iter()
            .find(|candidate| &candidate.id == instance)
            .map(|candidate| candidate.circuit.clone())
            .ok_or_else(|| SignalBundleBindingError::UnknownSubcircuitInstance {
                parent: parent.clone(),
                instance: instance.clone(),
            })?;
        let child_endpoint = self.endpoint(&child_id, child_endpoint).ok_or_else(|| {
            SignalBundleBindingError::UnknownChildEndpoint {
                circuit: child_id.clone(),
                endpoint: child_endpoint.clone(),
            }
        })?;
        if parent_endpoint.bundle != child_endpoint.bundle {
            return Err(SignalBundleBindingError::BundleMismatch {
                parent: parent_endpoint.bundle.clone(),
                child: child_endpoint.bundle.clone(),
            });
        }

        let bundle = self
            .bundles
            .iter()
            .find(|bundle| bundle.id == parent_endpoint.bundle)
            .expect("validated endpoint bundle must exist");
        let parent_view = bundle
            .modport(&parent_endpoint.modport)
            .expect("validated parent modport must exist");
        let child_view = bundle
            .modport(&child_endpoint.modport)
            .expect("validated child modport must exist");
        let parent_circuit = &circuits.circuits[parent_index];
        let mut generated = Vec::with_capacity(bundle.members.len());
        for member in &bundle.members {
            let member = &member.id;
            let parent_direction = parent_view
                .direction(member)
                .expect("validated modport must cover every member");
            let child_direction = child_view
                .direction(member)
                .expect("validated modport must cover every member");
            if !parent_direction.is_dual_to(child_direction) {
                return Err(SignalBundleBindingError::NonDualMember {
                    bundle: bundle.id.clone(),
                    member: member.clone(),
                    parent: parent_direction,
                    child: child_direction,
                });
            }
            let parent_port = endpoint_port(parent_endpoint, member);
            let child_port = endpoint_port(child_endpoint, member);
            let parent_net = parent_circuit
                .ports
                .iter()
                .find(|port| port.id == *parent_port)
                .map(|port| port.net.clone())
                .expect("validated endpoint port must exist");
            generated.push(SubcircuitPortBinding {
                port: child_port.clone(),
                net: parent_net,
            });
        }

        let target = circuits.circuits[parent_index]
            .subcircuits
            .iter_mut()
            .find(|candidate| &candidate.id == instance)
            .expect("selected subcircuit instance must still exist");
        for binding in &generated {
            if let Some(existing) = target
                .ports
                .iter()
                .find(|candidate| candidate.port == binding.port)
                && existing.net != binding.net
            {
                return Err(SignalBundleBindingError::ConflictingExistingBinding {
                    parent: parent.clone(),
                    instance: instance.clone(),
                    port: binding.port.clone(),
                    existing: existing.net.clone(),
                    proposed: binding.net.clone(),
                });
            }
        }
        let unbound = generated
            .iter()
            .filter(|binding| {
                !target
                    .ports
                    .iter()
                    .any(|candidate| candidate.port == binding.port)
            })
            .cloned()
            .collect::<Vec<_>>();
        target.ports.extend(unbound);
        Ok(generated)
    }

    fn endpoint(
        &self,
        circuit: &CircuitId,
        endpoint: &BundleEndpointId,
    ) -> Option<&SignalBundleEndpoint> {
        self.endpoints
            .iter()
            .find(|candidate| &candidate.circuit == circuit && &candidate.id == endpoint)
    }
}

fn validate_endpoint(
    endpoint: &SignalBundleEndpoint,
    circuit: &Circuit,
    bundle: &SignalBundle,
    modport: &Modport,
    issues: &mut Vec<SignalBundleValidationIssue>,
) {
    let members = bundle
        .members
        .iter()
        .map(|member| member.id.clone())
        .collect::<BTreeSet<_>>();
    let mut bound_members = BTreeSet::new();
    let mut bound_ports = BTreeSet::new();
    for binding in &endpoint.ports {
        if !members.contains(&binding.member) {
            issues.push(SignalBundleValidationIssue::UnknownEndpointMember {
                circuit: endpoint.circuit.clone(),
                endpoint: endpoint.id.clone(),
                member: binding.member.clone(),
            });
        }
        if !bound_members.insert(binding.member.clone()) {
            issues.push(SignalBundleValidationIssue::DuplicateEndpointMember {
                circuit: endpoint.circuit.clone(),
                endpoint: endpoint.id.clone(),
                member: binding.member.clone(),
            });
        }
        if !bound_ports.insert(binding.port.clone()) {
            issues.push(SignalBundleValidationIssue::DuplicateEndpointPort {
                circuit: endpoint.circuit.clone(),
                endpoint: endpoint.id.clone(),
                port: binding.port.clone(),
            });
        }
        let Some(port) = circuit
            .ports
            .iter()
            .find(|candidate| candidate.id == binding.port)
        else {
            issues.push(SignalBundleValidationIssue::UnknownEndpointPort {
                circuit: endpoint.circuit.clone(),
                endpoint: endpoint.id.clone(),
                port: binding.port.clone(),
            });
            continue;
        };
        if let Some(expected) = modport.direction(&binding.member)
            && port.direction != expected
        {
            issues.push(SignalBundleValidationIssue::EndpointPortDirectionMismatch {
                circuit: endpoint.circuit.clone(),
                endpoint: endpoint.id.clone(),
                member: binding.member.clone(),
                port: binding.port.clone(),
                expected,
                actual: port.direction,
            });
        }
        if let Some(member) = bundle
            .members
            .iter()
            .find(|member| member.id == binding.member)
        {
            let actual = circuit.port_signal_type(&port.id);
            if actual != member.signal_type {
                issues.push(
                    SignalBundleValidationIssue::EndpointPortSignalTypeMismatch {
                        circuit: endpoint.circuit.clone(),
                        endpoint: endpoint.id.clone(),
                        member: binding.member.clone(),
                        port: binding.port.clone(),
                        expected: member.signal_type.clone(),
                        actual,
                    },
                );
            }
        }
    }
    for member in members.difference(&bound_members) {
        issues.push(SignalBundleValidationIssue::MissingEndpointMember {
            circuit: endpoint.circuit.clone(),
            endpoint: endpoint.id.clone(),
            member: member.clone(),
        });
    }
}

fn endpoint_port<'a>(endpoint: &'a SignalBundleEndpoint, member: &BundleMemberId) -> &'a PortId {
    &endpoint
        .ports
        .iter()
        .find(|binding| &binding.member == member)
        .expect("validated endpoint must cover every member")
        .port
}

/// Failure to lower a checked bundle connection into hierarchy port bindings.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum SignalBundleBindingError {
    /// Bundle definitions or endpoints are structurally invalid.
    InvalidBundleLibrary(SignalBundleValidationReport),
    /// The selected parent circuit is absent.
    UnknownParentCircuit(CircuitId),
    /// The selected child instance is absent from the parent.
    UnknownSubcircuitInstance {
        parent: CircuitId,
        instance: SubcircuitInstanceId,
    },
    /// The selected parent endpoint is absent from the parent circuit.
    UnknownParentEndpoint {
        circuit: CircuitId,
        endpoint: BundleEndpointId,
    },
    /// The selected child endpoint is absent from the instantiated child circuit.
    UnknownChildEndpoint {
        circuit: CircuitId,
        endpoint: BundleEndpointId,
    },
    /// The endpoints expose different nominal bundle types.
    BundleMismatch {
        parent: SignalBundleId,
        child: SignalBundleId,
    },
    /// One member's endpoint directions are not conjugates.
    NonDualMember {
        bundle: SignalBundleId,
        member: BundleMemberId,
        parent: PortDirection,
        child: PortDirection,
    },
    /// An existing explicit binding disagrees with the bundle-derived binding.
    ConflictingExistingBinding {
        parent: CircuitId,
        instance: SubcircuitInstanceId,
        port: PortId,
        existing: NetId,
        proposed: NetId,
    },
}

impl std::fmt::Display for SignalBundleBindingError {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::InvalidBundleLibrary(report) => write!(
                formatter,
                "signal bundle library has {} validation issue(s)",
                report.issues.len()
            ),
            Self::UnknownParentCircuit(circuit) => {
                write!(formatter, "unknown parent circuit {}", circuit.as_str())
            }
            Self::UnknownSubcircuitInstance { parent, instance } => write!(
                formatter,
                "unknown subcircuit instance {} in {}",
                instance.as_str(),
                parent.as_str()
            ),
            Self::UnknownParentEndpoint { circuit, endpoint } => write!(
                formatter,
                "unknown parent bundle endpoint {} in {}",
                endpoint.as_str(),
                circuit.as_str()
            ),
            Self::UnknownChildEndpoint { circuit, endpoint } => write!(
                formatter,
                "unknown child bundle endpoint {} in {}",
                endpoint.as_str(),
                circuit.as_str()
            ),
            Self::BundleMismatch { parent, child } => write!(
                formatter,
                "bundle mismatch: parent {} and child {}",
                parent.as_str(),
                child.as_str()
            ),
            Self::NonDualMember { bundle, member, .. } => write!(
                formatter,
                "bundle {} member {} does not connect dual directions",
                bundle.as_str(),
                member.as_str()
            ),
            Self::ConflictingExistingBinding {
                parent,
                instance,
                port,
                ..
            } => write!(
                formatter,
                "bundle binding conflicts with port {} on instance {} in {}",
                port.as_str(),
                instance.as_str(),
                parent.as_str()
            ),
        }
    }
}

impl std::error::Error for SignalBundleBindingError {}
