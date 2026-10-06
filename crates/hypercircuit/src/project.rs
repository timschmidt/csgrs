//! Recursive fluent hierarchy over retained circuit and layout-module carriers.

use std::collections::{BTreeMap, BTreeSet};
use std::fmt::{Display, Formatter};

use crate::{
    CheckedDesign, Circuit, CircuitId, CircuitLibrary, CircuitModuleParameterOverride,
    DesignSourceMap, HierarchyError, LayoutAssembly, LayoutCompositionError,
    LayoutCompositionReport, LayoutModule, LayoutModuleId, LayoutModuleInstance, LayoutTransform,
    NetHandle, PcbLayout, PortHandle, SchematicLayout, SubcircuitInstance, SubcircuitInstanceId,
    SubcircuitPortBinding,
};

/// Failure while building or compiling a checked recursive design hierarchy.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum ModuleBuildError {
    /// A stable module or instance identity could not be constructed.
    InvalidIdentifier,
    /// Two direct child instances share one identity.
    DuplicateInstance(String),
    /// A parent-net handle belongs to another checked design.
    ForeignParentNet,
    /// A child-port handle belongs to another checked design.
    ForeignChildPort,
    /// A parent-net handle no longer addresses the parent circuit.
    UnknownParentNet(String),
    /// A child-port handle no longer addresses the child circuit.
    UnknownChildPort(String),
    /// One child port was bound more than once.
    DuplicatePortBinding(String),
    /// A required child port has no parent binding.
    MissingRequiredPort(String),
    /// Two ports exposing one child net were bound to different parent nets.
    ConflictingChildNetBinding(String),
    /// One circuit id was supplied with different retained definitions.
    ConflictingCircuitDefinition(CircuitId),
    /// One layout-module id was supplied with different retained definitions.
    ConflictingLayoutDefinition(LayoutModuleId),
    /// Circuit hierarchy validation or flattening failed.
    Hierarchy(HierarchyError),
    /// Layout-module composition failed.
    Layout(LayoutCompositionError),
}

impl Display for ModuleBuildError {
    fn fmt(&self, formatter: &mut Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::InvalidIdentifier => formatter.write_str("module hierarchy identity is empty"),
            Self::DuplicateInstance(instance) => {
                write!(formatter, "duplicate direct child instance {instance}")
            }
            Self::ForeignParentNet => {
                formatter.write_str("parent net handle belongs to another design")
            }
            Self::ForeignChildPort => {
                formatter.write_str("child port handle belongs to another design")
            }
            Self::UnknownParentNet(net) => write!(formatter, "unknown parent net {net}"),
            Self::UnknownChildPort(port) => write!(formatter, "unknown child port {port}"),
            Self::DuplicatePortBinding(port) => {
                write!(formatter, "child port {port} is bound more than once")
            }
            Self::MissingRequiredPort(port) => {
                write!(formatter, "required child port {port} is unbound")
            }
            Self::ConflictingChildNetBinding(net) => {
                write!(formatter, "child net {net} has conflicting parent bindings")
            }
            Self::ConflictingCircuitDefinition(circuit) => write!(
                formatter,
                "circuit {} has conflicting reusable definitions",
                circuit.as_str()
            ),
            Self::ConflictingLayoutDefinition(module) => write!(
                formatter,
                "layout module {} has conflicting reusable definitions",
                module.as_str()
            ),
            Self::Hierarchy(error) => Display::fmt(error, formatter),
            Self::Layout(error) => Display::fmt(error, formatter),
        }
    }
}

impl std::error::Error for ModuleBuildError {}

/// One recursive child-module instantiation.
#[derive(Clone, Debug, PartialEq)]
pub struct DesignModuleInstance {
    /// Stable identity within the direct parent scope.
    pub id: SubcircuitInstanceId,
    /// Reusable child implementation.
    pub module: Box<DesignModule>,
    /// Child-port to parent-net bindings.
    pub ports: Vec<SubcircuitPortBinding>,
    /// Exact child-local to parent-local layout transform.
    pub transform: LayoutTransform,
    /// Exact overrides of child module parameters.
    pub parameter_overrides: Vec<CircuitModuleParameterOverride>,
}

/// Checked reusable module with arbitrary nested child instances.
#[derive(Clone, Debug, PartialEq)]
pub struct DesignModule {
    /// Stable physical-layout definition identity.
    pub id: LayoutModuleId,
    /// Checked local circuit, schematic, PCB intent, and provenance.
    pub design: CheckedDesign,
    /// Direct reusable children.
    pub instances: Vec<DesignModuleInstance>,
}

impl DesignModule {
    /// Wraps one checked design as a reusable module definition.
    pub fn new(id: impl Into<String>, design: CheckedDesign) -> Result<Self, ModuleBuildError> {
        Ok(Self {
            id: LayoutModuleId::new(id.into()).map_err(|_| ModuleBuildError::InvalidIdentifier)?,
            design,
            instances: Vec::new(),
        })
    }

    /// Instantiates a checked child using typed child-port/parent-net bindings.
    pub fn instantiate<'a, I>(
        &mut self,
        id: impl Into<String>,
        child: DesignModule,
        bindings: I,
        transform: LayoutTransform,
    ) -> Result<SubcircuitInstanceId, ModuleBuildError>
    where
        I: IntoIterator<Item = (&'a PortHandle, &'a NetHandle)>,
    {
        self.instantiate_with_overrides(id, child, bindings, transform, Vec::new())
    }

    /// Instantiates a checked child with exact module-parameter overrides.
    pub fn instantiate_with_overrides<'a, I>(
        &mut self,
        id: impl Into<String>,
        child: DesignModule,
        bindings: I,
        transform: LayoutTransform,
        parameter_overrides: Vec<CircuitModuleParameterOverride>,
    ) -> Result<SubcircuitInstanceId, ModuleBuildError>
    where
        I: IntoIterator<Item = (&'a PortHandle, &'a NetHandle)>,
    {
        let id = SubcircuitInstanceId::new(id.into())
            .map_err(|_| ModuleBuildError::InvalidIdentifier)?;
        if self.instances.iter().any(|instance| instance.id == id) {
            return Err(ModuleBuildError::DuplicateInstance(id.as_str().into()));
        }
        let mut ports = Vec::new();
        let mut bound_ports = BTreeSet::new();
        let mut child_net_bindings = BTreeMap::new();
        for (port, net) in bindings {
            if port.owner() != child.design.owner() {
                return Err(ModuleBuildError::ForeignChildPort);
            }
            if net.owner() != self.design.owner() {
                return Err(ModuleBuildError::ForeignParentNet);
            }
            let Some(child_port) = child
                .design
                .circuit
                .ports
                .iter()
                .find(|candidate| candidate.id == *port.id())
            else {
                return Err(ModuleBuildError::UnknownChildPort(
                    port.id().as_str().into(),
                ));
            };
            if !self
                .design
                .circuit
                .nets
                .iter()
                .any(|candidate| candidate.id == *net.id())
            {
                return Err(ModuleBuildError::UnknownParentNet(net.id().as_str().into()));
            }
            if !bound_ports.insert(port.id().clone()) {
                return Err(ModuleBuildError::DuplicatePortBinding(
                    port.id().as_str().into(),
                ));
            }
            if child_net_bindings
                .insert(child_port.net.clone(), net.id().clone())
                .is_some_and(|previous| previous != *net.id())
            {
                return Err(ModuleBuildError::ConflictingChildNetBinding(
                    child_port.net.as_str().into(),
                ));
            }
            ports.push(SubcircuitPortBinding {
                port: port.id().clone(),
                net: net.id().clone(),
            });
        }
        for port in child
            .design
            .circuit
            .ports
            .iter()
            .filter(|port| !port.optional)
        {
            if !bound_ports.contains(&port.id) {
                return Err(ModuleBuildError::MissingRequiredPort(
                    port.id.as_str().into(),
                ));
            }
        }
        self.instances.push(DesignModuleInstance {
            id: id.clone(),
            module: Box::new(child),
            ports,
            transform,
            parameter_overrides,
        });
        Ok(id)
    }

    /// Compiles arbitrary nested module intent through the retained hierarchy carriers.
    pub fn compile(&self) -> Result<CheckedProject, ModuleBuildError> {
        let mut circuits = BTreeMap::new();
        let mut layouts = BTreeMap::new();
        let mut schematics = BTreeMap::new();
        let mut sources = BTreeMap::new();
        collect_definitions(
            self,
            true,
            &mut circuits,
            &mut layouts,
            &mut schematics,
            &mut sources,
        )?;
        let library = CircuitLibrary {
            root: self.design.circuit.id.clone(),
            circuits: circuits.into_values().collect(),
        };
        let mut layout_instances = Vec::new();
        collect_layout_instances(
            self,
            &[],
            &LayoutTransform::default(),
            &mut layout_instances,
        );
        let assembly = LayoutAssembly {
            board: self.design.layout.clone(),
            modules: layouts.into_values().collect(),
            instances: layout_instances,
        };
        let composed = assembly
            .compose(&library)
            .map_err(ModuleBuildError::Layout)?;
        Ok(CheckedProject {
            circuits: library,
            layouts: assembly,
            composed,
            schematics,
            sources,
        })
    }
}

/// Compiled retained hierarchy plus its validated flat routing/release view.
#[derive(Clone, Debug, PartialEq)]
pub struct CheckedProject {
    /// Reusable circuit definitions with one root.
    pub circuits: CircuitLibrary,
    /// Root board, reusable layout definitions, and hierarchy bindings.
    pub layouts: LayoutAssembly,
    /// Validated flattened circuit/layout pair used by routing and release.
    pub composed: LayoutCompositionReport,
    /// Per-definition schematic review models.
    pub schematics: BTreeMap<CircuitId, SchematicLayout>,
    /// Per-definition Rust authoring provenance.
    pub sources: BTreeMap<CircuitId, DesignSourceMap>,
}

fn collect_definitions(
    module: &DesignModule,
    is_root: bool,
    circuits: &mut BTreeMap<CircuitId, Circuit>,
    layouts: &mut BTreeMap<LayoutModuleId, LayoutModule>,
    schematics: &mut BTreeMap<CircuitId, SchematicLayout>,
    sources: &mut BTreeMap<CircuitId, DesignSourceMap>,
) -> Result<(), ModuleBuildError> {
    let mut circuit = module.design.circuit.clone();
    circuit
        .subcircuits
        .extend(module.instances.iter().map(|instance| SubcircuitInstance {
            id: instance.id.clone(),
            circuit: instance.module.design.circuit.id.clone(),
            ports: instance.ports.clone(),
            parameter_overrides: instance.parameter_overrides.clone(),
        }));
    if circuits
        .insert(circuit.id.clone(), circuit.clone())
        .is_some_and(|existing| existing != circuit)
    {
        return Err(ModuleBuildError::ConflictingCircuitDefinition(circuit.id));
    }
    if schematics
        .insert(
            module.design.circuit.id.clone(),
            module.design.schematic.clone(),
        )
        .is_some_and(|existing| existing != module.design.schematic)
    {
        return Err(ModuleBuildError::ConflictingCircuitDefinition(
            module.design.circuit.id.clone(),
        ));
    }
    sources
        .entry(module.design.circuit.id.clone())
        .or_insert_with(|| module.design.source_map.clone());
    if !is_root {
        let layout = module_layout(module)?;
        if layouts
            .insert(layout.id.clone(), layout.clone())
            .is_some_and(|existing| existing != layout)
        {
            return Err(ModuleBuildError::ConflictingLayoutDefinition(layout.id));
        }
    }
    for instance in &module.instances {
        collect_definitions(
            &instance.module,
            false,
            circuits,
            layouts,
            schematics,
            sources,
        )?;
    }
    Ok(())
}

fn module_layout(module: &DesignModule) -> Result<LayoutModule, ModuleBuildError> {
    let PcbLayout {
        land_patterns,
        placements,
        placement_constraints,
        routes,
        vias,
        zones,
        keepouts,
        rules,
        ..
    } = module.design.layout.clone();
    Ok(LayoutModule {
        id: module.id.clone(),
        circuit: module.design.circuit.id.clone(),
        land_patterns,
        placements,
        placement_constraints,
        placement_groups: Vec::new(),
        routes,
        vias,
        zones,
        keepouts,
        rules,
    })
}

fn collect_layout_instances(
    module: &DesignModule,
    parent_path: &[SubcircuitInstanceId],
    parent_transform: &LayoutTransform,
    instances: &mut Vec<LayoutModuleInstance>,
) {
    for child in &module.instances {
        let mut path = parent_path.to_vec();
        path.push(child.id.clone());
        let transform = parent_transform.compose(&child.transform);
        instances.push(LayoutModuleInstance {
            hierarchy_path: path.clone(),
            module: child.module.id.clone(),
            transform: transform.clone(),
        });
        collect_layout_instances(&child.module, &path, &transform, instances);
    }
}
