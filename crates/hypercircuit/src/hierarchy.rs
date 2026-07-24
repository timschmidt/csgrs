//! Reusable circuit hierarchy, validation, and deterministic flattening.

use std::collections::{BTreeMap, BTreeSet};

use crate::{
    BranchId, Bus, BusId, BusSliceId, Circuit, CircuitId, CircuitInstanceId,
    CircuitModuleParameterOverride, CircuitModuleParameterTarget, CircuitParameter,
    CircuitValidationIssue, ComponentId, DeviceModelId, LinearStamp, Net, NetId, PinBinding,
    PortId, SubcircuitInstanceId,
};

/// Binding from one child-circuit boundary port to a net in its parent scope.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct SubcircuitPortBinding {
    /// Port declared by the referenced child circuit.
    pub port: PortId,
    /// Net in the parent circuit receiving that port.
    pub net: NetId,
}

/// Reusable child circuit instantiated in one parent circuit scope.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, PartialEq)]
pub struct SubcircuitInstance {
    /// Stable identity within the parent circuit.
    pub id: SubcircuitInstanceId,
    /// Referenced circuit definition.
    pub circuit: CircuitId,
    /// Explicit child-port to parent-net bindings.
    pub ports: Vec<SubcircuitPortBinding>,
    /// Exact named overrides of parameters declared by the child definition.
    #[cfg_attr(feature = "interchange", serde(default))]
    pub parameter_overrides: Vec<CircuitModuleParameterOverride>,
}

/// Library of reusable circuit definitions with one designated root.
#[derive(Clone, Debug, PartialEq)]
pub struct CircuitLibrary {
    /// Circuit definition to elaborate as the design root.
    pub root: CircuitId,
    /// Reusable definitions, including the root definition.
    pub circuits: Vec<Circuit>,
}

/// Stable local-to-flat identity map for one elaborated subcircuit scope.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct FlattenedCircuitScope {
    /// Instance path from the root circuit to this scope.
    pub path: Vec<SubcircuitInstanceId>,
    /// Reusable circuit definition instantiated at this path.
    pub circuit: CircuitId,
    /// Local net ids mapped to flattened parent or path-qualified ids.
    pub nets: BTreeMap<NetId, NetId>,
    /// Local component-instance ids mapped to path-qualified ids.
    pub instances: BTreeMap<CircuitInstanceId, CircuitInstanceId>,
}

/// Flattened circuit plus stable scope maps for layout and editor composition.
#[derive(Clone, Debug, PartialEq)]
pub struct CircuitFlatteningReport {
    /// Validated flat circuit.
    pub circuit: Circuit,
    /// Child scopes in deterministic preorder.
    pub scopes: Vec<FlattenedCircuitScope>,
}

/// Cross-definition problem found while validating a circuit library.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum CircuitLibraryValidationIssue {
    /// Two definitions share one stable circuit id.
    DuplicateCircuit(CircuitId),
    /// The designated root definition is absent.
    UnknownRoot(CircuitId),
    /// One definition has local structural errors.
    InvalidCircuit {
        circuit: CircuitId,
        issues: Vec<CircuitValidationIssue>,
    },
    /// A child instance references an absent circuit definition.
    UnknownChildCircuit {
        parent: CircuitId,
        instance: SubcircuitInstanceId,
        child: CircuitId,
    },
    /// A binding names a port absent from the child definition.
    UnknownChildPort {
        parent: CircuitId,
        instance: SubcircuitInstanceId,
        port: PortId,
    },
    /// A required child port has no parent-net binding.
    MissingChildPort {
        parent: CircuitId,
        instance: SubcircuitInstanceId,
        port: PortId,
    },
    /// Two ports exposing one child net were bound to different parent nets.
    ConflictingChildNetBindings {
        parent: CircuitId,
        instance: SubcircuitInstanceId,
        child_net: NetId,
    },
    /// A child instance overrides no declared parameter on its child definition.
    UnknownChildParameter {
        parent: CircuitId,
        instance: SubcircuitInstanceId,
        parameter: String,
    },
    /// A parent module parameter forwards to no declared child parameter.
    UnknownForwardedParameter {
        circuit: CircuitId,
        module_parameter: String,
        instance: SubcircuitInstanceId,
        parameter: String,
    },
    /// A forwarded parent/child parameter pair has incompatible units.
    ForwardedParameterUnitMismatch {
        circuit: CircuitId,
        module_parameter: String,
        instance: SubcircuitInstanceId,
        parameter: String,
        parent_unit: String,
        child_unit: String,
    },
    /// Circuit definitions contain a recursive instantiation cycle.
    RecursiveCycle(Vec<CircuitId>),
}

/// Deterministic result of validating all hierarchy references.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct CircuitLibraryValidationReport {
    /// Every local and cross-definition issue.
    pub issues: Vec<CircuitLibraryValidationIssue>,
}

impl CircuitLibraryValidationReport {
    /// True when definitions can be deterministically elaborated.
    pub fn is_valid(&self) -> bool {
        self.issues.is_empty()
    }
}

/// Failure to elaborate a circuit hierarchy.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum HierarchyError {
    /// Library validation failed before elaboration.
    InvalidLibrary(CircuitLibraryValidationReport),
    /// Namespaced elaboration unexpectedly produced an invalid flat circuit.
    InvalidFlattenedCircuit(Vec<CircuitValidationIssue>),
}

impl std::fmt::Display for HierarchyError {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::InvalidLibrary(report) => write!(
                formatter,
                "circuit hierarchy has {} validation issue(s)",
                report.issues.len()
            ),
            Self::InvalidFlattenedCircuit(issues) => write!(
                formatter,
                "flattened circuit has {} validation issue(s)",
                issues.len()
            ),
        }
    }
}

impl std::error::Error for HierarchyError {}

impl CircuitLibrary {
    /// Validates local circuit graphs, child interfaces, and recursion.
    pub fn validate(&self) -> CircuitLibraryValidationReport {
        let mut issues = Vec::new();
        let mut definitions = BTreeMap::new();
        for circuit in &self.circuits {
            if definitions.insert(circuit.id.clone(), circuit).is_some() {
                issues.push(CircuitLibraryValidationIssue::DuplicateCircuit(
                    circuit.id.clone(),
                ));
            }
            let local = circuit.validate();
            if !local.is_valid() {
                issues.push(CircuitLibraryValidationIssue::InvalidCircuit {
                    circuit: circuit.id.clone(),
                    issues: local.issues,
                });
            }
        }
        if !definitions.contains_key(&self.root) {
            issues.push(CircuitLibraryValidationIssue::UnknownRoot(
                self.root.clone(),
            ));
        }

        for parent in &self.circuits {
            for instance in &parent.subcircuits {
                let Some(child) = definitions.get(&instance.circuit).copied() else {
                    issues.push(CircuitLibraryValidationIssue::UnknownChildCircuit {
                        parent: parent.id.clone(),
                        instance: instance.id.clone(),
                        child: instance.circuit.clone(),
                    });
                    continue;
                };
                let child_ports = child
                    .ports
                    .iter()
                    .map(|port| (port.id.clone(), port))
                    .collect::<BTreeMap<_, _>>();
                let mut child_net_bindings = BTreeMap::<NetId, NetId>::new();
                for binding in &instance.ports {
                    let Some(port) = child_ports.get(&binding.port) else {
                        issues.push(CircuitLibraryValidationIssue::UnknownChildPort {
                            parent: parent.id.clone(),
                            instance: instance.id.clone(),
                            port: binding.port.clone(),
                        });
                        continue;
                    };
                    if child_net_bindings
                        .insert(port.net.clone(), binding.net.clone())
                        .is_some_and(|previous| previous != binding.net)
                    {
                        issues.push(CircuitLibraryValidationIssue::ConflictingChildNetBindings {
                            parent: parent.id.clone(),
                            instance: instance.id.clone(),
                            child_net: port.net.clone(),
                        });
                    }
                }
                let bound = instance
                    .ports
                    .iter()
                    .map(|binding| &binding.port)
                    .collect::<BTreeSet<_>>();
                for port in child.ports.iter().filter(|port| !port.optional) {
                    if !bound.contains(&port.id) {
                        issues.push(CircuitLibraryValidationIssue::MissingChildPort {
                            parent: parent.id.clone(),
                            instance: instance.id.clone(),
                            port: port.id.clone(),
                        });
                    }
                }
                for parameter in &instance.parameter_overrides {
                    if !child
                        .module_parameters
                        .iter()
                        .any(|candidate| candidate.name == parameter.parameter)
                    {
                        issues.push(CircuitLibraryValidationIssue::UnknownChildParameter {
                            parent: parent.id.clone(),
                            instance: instance.id.clone(),
                            parameter: parameter.parameter.clone(),
                        });
                    }
                }
            }
            for module_parameter in &parent.module_parameters {
                for target in &module_parameter.targets {
                    let CircuitModuleParameterTarget::SubcircuitParameter {
                        instance,
                        parameter,
                    } = target
                    else {
                        continue;
                    };
                    let child_instance = parent
                        .subcircuits
                        .iter()
                        .find(|candidate| candidate.id == *instance)
                        .expect("local circuit validation checked subcircuit parameter target");
                    let Some(child) = definitions.get(&child_instance.circuit).copied() else {
                        continue;
                    };
                    let Some(child_parameter) = child
                        .module_parameters
                        .iter()
                        .find(|candidate| candidate.name == *parameter)
                    else {
                        issues.push(CircuitLibraryValidationIssue::UnknownForwardedParameter {
                            circuit: parent.id.clone(),
                            module_parameter: module_parameter.name.clone(),
                            instance: instance.clone(),
                            parameter: parameter.clone(),
                        });
                        continue;
                    };
                    if module_parameter.unit != child_parameter.unit {
                        issues.push(
                            CircuitLibraryValidationIssue::ForwardedParameterUnitMismatch {
                                circuit: parent.id.clone(),
                                module_parameter: module_parameter.name.clone(),
                                instance: instance.clone(),
                                parameter: parameter.clone(),
                                parent_unit: module_parameter.unit.clone(),
                                child_unit: child_parameter.unit.clone(),
                            },
                        );
                    }
                }
            }
        }

        let mut finished = BTreeSet::new();
        let mut active = Vec::new();
        let definition_ids = definitions.keys().cloned().collect::<Vec<_>>();
        for id in definition_ids {
            find_cycles(&id, &definitions, &mut active, &mut finished, &mut issues);
        }
        CircuitLibraryValidationReport { issues }
    }

    /// Elaborates the root into one flat circuit with stable path-qualified ids.
    pub fn flatten(&self) -> Result<Circuit, HierarchyError> {
        self.flatten_with_scopes().map(|report| report.circuit)
    }

    /// Elaborates hierarchy and retains local-to-flat maps for every child scope.
    pub fn flatten_with_scopes(&self) -> Result<CircuitFlatteningReport, HierarchyError> {
        let validation = self.validate();
        if !validation.is_valid() {
            return Err(HierarchyError::InvalidLibrary(validation));
        }
        let definitions = self
            .circuits
            .iter()
            .map(|circuit| (circuit.id.clone(), circuit))
            .collect::<BTreeMap<_, _>>();
        let root = definitions
            .get(&self.root)
            .expect("validated root circuit must exist");
        let instantiated_root = instantiate_parameters(root, &[]);
        let mut flattened = instantiated_root.clone();
        flattened.module_parameters.clear();
        flattened.subcircuits.clear();
        let root_net_map = instantiated_root
            .nets
            .iter()
            .map(|net| (net.id.clone(), net.id.clone()))
            .collect::<BTreeMap<_, _>>();
        let mut scopes = Vec::new();
        expand_children(
            &instantiated_root,
            "",
            &[],
            &root_net_map,
            &definitions,
            &mut flattened,
            &mut scopes,
        );
        let report = flattened.validate();
        if report.is_valid() {
            Ok(CircuitFlatteningReport {
                circuit: flattened,
                scopes,
            })
        } else {
            Err(HierarchyError::InvalidFlattenedCircuit(report.issues))
        }
    }
}

fn find_cycles(
    id: &CircuitId,
    definitions: &BTreeMap<CircuitId, &Circuit>,
    active: &mut Vec<CircuitId>,
    finished: &mut BTreeSet<CircuitId>,
    issues: &mut Vec<CircuitLibraryValidationIssue>,
) {
    if finished.contains(id) {
        return;
    }
    if let Some(start) = active.iter().position(|candidate| candidate == id) {
        let mut cycle = active[start..].to_vec();
        cycle.push(id.clone());
        if !issues
            .iter()
            .any(|issue| matches!(issue, CircuitLibraryValidationIssue::RecursiveCycle(found) if found == &cycle))
        {
            issues.push(CircuitLibraryValidationIssue::RecursiveCycle(cycle));
        }
        return;
    }
    active.push(id.clone());
    if let Some(circuit) = definitions.get(id) {
        for child in &circuit.subcircuits {
            if definitions.contains_key(&child.circuit) {
                find_cycles(&child.circuit, definitions, active, finished, issues);
            }
        }
    }
    active.pop();
    finished.insert(id.clone());
}

fn expand_children(
    parent: &Circuit,
    parent_path: &str,
    parent_segments: &[SubcircuitInstanceId],
    parent_net_map: &BTreeMap<NetId, NetId>,
    definitions: &BTreeMap<CircuitId, &Circuit>,
    output: &mut Circuit,
    scopes: &mut Vec<FlattenedCircuitScope>,
) {
    for child_instance in &parent.subcircuits {
        let child_definition = definitions
            .get(&child_instance.circuit)
            .expect("validated child circuit must exist");
        let instantiated_child =
            instantiate_parameters(child_definition, &child_instance.parameter_overrides);
        let child = &instantiated_child;
        let path = if parent_path.is_empty() {
            child_instance.id.as_str().to_owned()
        } else {
            format!("{parent_path}/{}", child_instance.id.as_str())
        };
        let mut path_segments = parent_segments.to_vec();
        path_segments.push(child_instance.id.clone());
        let mut child_net_map = BTreeMap::new();
        for binding in &child_instance.ports {
            let port = child
                .ports
                .iter()
                .find(|port| port.id == binding.port)
                .expect("validated child port must exist");
            child_net_map.insert(
                port.net.clone(),
                parent_net_map
                    .get(&binding.net)
                    .expect("validated parent net must exist")
                    .clone(),
            );
        }
        for net in &child.nets {
            if child_net_map.contains_key(&net.id) {
                continue;
            }
            let id = namespaced_net(&path, &net.id);
            child_net_map.insert(net.id.clone(), id.clone());
            output.nets.push(Net {
                id,
                is_ground: net.is_ground,
            });
        }

        let bus_map = child
            .buses
            .iter()
            .map(|bus| {
                let id = namespaced_bus(&path, &bus.id);
                output.buses.push(Bus {
                    id: id.clone(),
                    nets: bus
                        .nets
                        .iter()
                        .map(|net| {
                            child_net_map
                                .get(net)
                                .expect("validated child bus net must exist")
                                .clone()
                        })
                        .collect(),
                });
                (bus.id.clone(), id)
            })
            .collect::<BTreeMap<_, _>>();
        output
            .bus_slices
            .extend(child.bus_slices.iter().map(|slice| {
                let mut slice = slice.clone();
                slice.id = namespaced_bus_slice(&path, &slice.id);
                slice.bus = bus_map
                    .get(&slice.bus)
                    .expect("validated child bus slice must reference a bus")
                    .clone();
                slice
            }));
        for rail in &child.rails {
            let mut rail = rail.clone();
            rail.net = child_net_map
                .get(&rail.net)
                .expect("validated child rail net must exist")
                .clone();
            if !output.rails.iter().any(|existing| existing == &rail) {
                output.rails.push(rail);
            }
        }

        let model_map = child
            .device_models
            .iter()
            .map(|source| {
                let source_id = source.id.clone();
                let id = namespaced_model(&path, &source_id);
                let mut model = source.clone();
                model.id = id.clone();
                output.device_models.push(model);
                (source_id, id)
            })
            .collect::<BTreeMap<_, _>>();
        let mut instance_map = BTreeMap::new();
        for instance in &child.instances {
            let local_id = instance.id.clone();
            let mut instance = instance.clone();
            instance.id = namespaced_instance(&path, &instance.id);
            instance_map.insert(local_id, instance.id.clone());
            instance.component = namespaced_component(&path, &instance.component);
            instance.model = model_map
                .get(&instance.model)
                .expect("validated child model must exist")
                .clone();
            instance.pins = instance
                .pins
                .into_iter()
                .map(|binding| PinBinding {
                    pin: binding.pin,
                    net: child_net_map
                        .get(&binding.net)
                        .expect("validated child net must exist")
                        .clone(),
                })
                .collect();
            output.instances.push(instance);
        }
        output
            .source_stimuli
            .extend(child.source_stimuli.iter().cloned().map(|mut stimulus| {
                stimulus.component = namespaced_component(&path, &stimulus.component);
                stimulus
            }));
        output.stamps.extend(
            child
                .stamps
                .iter()
                .map(|stamp| remap_stamp(stamp, &path, &child_net_map)),
        );
        scopes.push(FlattenedCircuitScope {
            path: path_segments.clone(),
            circuit: child.id.clone(),
            nets: child_net_map.clone(),
            instances: instance_map,
        });
        expand_children(
            child,
            &path,
            &path_segments,
            &child_net_map,
            definitions,
            output,
            scopes,
        );
    }
}

fn instantiate_parameters(
    definition: &Circuit,
    parameter_overrides: &[CircuitModuleParameterOverride],
) -> Circuit {
    let overrides = parameter_overrides
        .iter()
        .map(|override_| (override_.parameter.as_str(), override_))
        .collect::<BTreeMap<_, _>>();
    let mut circuit = definition.clone();
    for interface in &definition.module_parameters {
        let (value, source) = overrides
            .get(interface.name.as_str())
            .map(|override_| (override_.value.clone(), override_.source.clone()))
            .unwrap_or_else(|| (interface.default.clone(), interface.source.clone()));
        for target in &interface.targets {
            match target {
                CircuitModuleParameterTarget::InstanceParameter {
                    instance,
                    parameter,
                } => {
                    let target = circuit
                        .instances
                        .iter_mut()
                        .find(|candidate| candidate.id == *instance)
                        .and_then(|instance| {
                            instance
                                .parameters
                                .iter_mut()
                                .find(|candidate| candidate.name == *parameter)
                        })
                        .expect("validated module instance parameter target must exist");
                    assign_parameter(target, &value, &source);
                }
                CircuitModuleParameterTarget::ModelParameter { model, parameter } => {
                    let target = circuit
                        .device_models
                        .iter_mut()
                        .find(|candidate| candidate.id == *model)
                        .and_then(|model| {
                            model
                                .parameters
                                .iter_mut()
                                .find(|candidate| candidate.name == *parameter)
                        })
                        .expect("validated module model parameter target must exist");
                    assign_parameter(target, &value, &source);
                }
                CircuitModuleParameterTarget::SubcircuitParameter {
                    instance,
                    parameter,
                } => {
                    let child = circuit
                        .subcircuits
                        .iter_mut()
                        .find(|candidate| candidate.id == *instance)
                        .expect("validated nested module target must exist");
                    if let Some(override_) = child
                        .parameter_overrides
                        .iter_mut()
                        .find(|candidate| candidate.parameter == *parameter)
                    {
                        override_.value = value.clone();
                        override_.source = source.clone();
                    } else {
                        child
                            .parameter_overrides
                            .push(CircuitModuleParameterOverride {
                                parameter: parameter.clone(),
                                value: value.clone(),
                                source: source.clone(),
                            });
                    }
                }
            }
        }
    }
    circuit
}

fn assign_parameter(parameter: &mut CircuitParameter, value: &hyperreal::Real, source: &str) {
    parameter.value = value.clone();
    parameter.source = source.to_owned();
}

fn remap_stamp(stamp: &LinearStamp, path: &str, nets: &BTreeMap<NetId, NetId>) -> LinearStamp {
    let net = |source: &Option<NetId>| {
        source.as_ref().map(|source| {
            nets.get(source)
                .expect("validated stamp net must exist")
                .clone()
        })
    };
    match stamp {
        LinearStamp::Conductance {
            component,
            part,
            pos,
            neg,
            conductance,
        } => LinearStamp::Conductance {
            component: namespaced_component(path, component),
            part: part.clone(),
            pos: net(pos),
            neg: net(neg),
            conductance: conductance.clone(),
        },
        LinearStamp::CurrentSource {
            component,
            pos,
            neg,
            current,
        } => LinearStamp::CurrentSource {
            component: namespaced_component(path, component),
            pos: net(pos),
            neg: net(neg),
            current: current.clone(),
        },
        LinearStamp::VoltageSource {
            component,
            branch,
            pos,
            neg,
            voltage,
        } => LinearStamp::VoltageSource {
            component: namespaced_component(path, component),
            branch: namespaced_branch(path, branch),
            pos: net(pos),
            neg: net(neg),
            voltage: voltage.clone(),
        },
        LinearStamp::Vccs {
            component,
            pos,
            neg,
            ctrl_pos,
            ctrl_neg,
            transconductance,
        } => LinearStamp::Vccs {
            component: namespaced_component(path, component),
            pos: net(pos),
            neg: net(neg),
            ctrl_pos: net(ctrl_pos),
            ctrl_neg: net(ctrl_neg),
            transconductance: transconductance.clone(),
        },
        LinearStamp::Companion {
            component,
            pos,
            neg,
            conductance,
            history_current,
        } => LinearStamp::Companion {
            component: namespaced_component(path, component),
            pos: net(pos),
            neg: net(neg),
            conductance: conductance.clone(),
            history_current: history_current.clone(),
        },
    }
}

fn namespaced_net(path: &str, id: &NetId) -> NetId {
    NetId::new(format!("{path}/{}", id.as_str())).expect("namespaced net id is nonempty")
}

fn namespaced_model(path: &str, id: &DeviceModelId) -> DeviceModelId {
    DeviceModelId::new(format!("{path}/{}", id.as_str())).expect("namespaced model id is nonempty")
}

fn namespaced_bus(path: &str, id: &BusId) -> BusId {
    BusId::new(format!("{path}/{}", id.as_str())).expect("namespaced bus id is nonempty")
}

fn namespaced_bus_slice(path: &str, id: &BusSliceId) -> BusSliceId {
    BusSliceId::new(format!("{path}/{}", id.as_str())).expect("namespaced bus-slice id is nonempty")
}

fn namespaced_instance(path: &str, id: &CircuitInstanceId) -> CircuitInstanceId {
    CircuitInstanceId::new(format!("{path}/{}", id.as_str()))
        .expect("namespaced instance id is nonempty")
}

fn namespaced_component(path: &str, id: &ComponentId) -> ComponentId {
    ComponentId::new(format!("{path}/{}", id.as_str()))
        .expect("namespaced component id is nonempty")
}

fn namespaced_branch(path: &str, id: &BranchId) -> BranchId {
    BranchId::new(format!("{path}/{}", id.as_str())).expect("namespaced branch id is nonempty")
}
