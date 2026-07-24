//! Deterministic connectivity-derived schematic layout.

use std::collections::{BTreeMap, BTreeSet, VecDeque};
use std::fmt::{Display, Formatter};

use hyperreal::{Real, RealSign};

use crate::{
    Circuit, CircuitInstance, CircuitInstanceId, DeviceModel, DeviceModelKind, PinElectricalKind,
    PortDirection, SchematicEndpoint, SchematicGraphic, SchematicGraphicFill, SchematicLabel,
    SchematicLabelId, SchematicLayout, SchematicPinPlacement, SchematicPinSide, SchematicPoint,
    SchematicPortPlacement, SchematicSymbol, SchematicSymbolDefinition,
    SchematicSymbolDefinitionId, SchematicSymbolId, SchematicSymbolUnit, SchematicWire,
    SchematicWireId,
};

/// Exact deterministic automatic-schematic layout policy.
#[derive(Clone, Debug, PartialEq)]
pub struct SchematicAutoLayoutPolicy {
    /// Upper-left origin for the first symbol center.
    pub origin: SchematicPoint,
    /// Horizontal separation between connectivity-depth columns.
    pub column_spacing: Real,
    /// Vertical separation between symbols in one column.
    pub row_spacing: Real,
    /// Generic symbol body width.
    pub body_width: Real,
    /// Minimum generic symbol body height.
    pub minimum_body_height: Real,
    /// Additional height per pin after the first.
    pub pin_pitch: Real,
    /// Distance from a generic body edge to its connection point.
    pub lead_length: Real,
    /// Hard work bound for generated symbols.
    pub max_instances: usize,
}

impl Default for SchematicAutoLayoutPolicy {
    fn default() -> Self {
        Self {
            origin: SchematicPoint::new(Real::from(20), Real::from(20)),
            column_spacing: Real::from(40),
            row_spacing: Real::from(30),
            body_width: Real::from(20),
            minimum_body_height: Real::from(12),
            pin_pitch: Real::from(4),
            lead_length: Real::from(3),
            max_instances: 10_000,
        }
    }
}

/// Failure to derive a faithful schematic from authoritative connectivity.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum SchematicAutoLayoutError {
    /// Circuit structure must be valid before drawing derivation.
    InvalidCircuit,
    /// One or more positive spacing/size fields were invalid.
    InvalidPolicy,
    /// The caller's explicit work bound was exceeded.
    InstanceLimit {
        /// Retained instance count.
        instances: usize,
        /// Caller-authored maximum.
        limit: usize,
    },
    /// A generated stable drawing identity was invalid.
    InvalidIdentifier(String),
    /// Exact coordinate arithmetic could not be completed.
    Arithmetic,
    /// The generated drawing failed its own structural/electrical replay.
    InvalidGeneratedLayout {
        /// Number of validation findings.
        issues: usize,
    },
}

impl Display for SchematicAutoLayoutError {
    fn fmt(&self, formatter: &mut Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::InvalidCircuit => {
                formatter.write_str("cannot derive a schematic from an invalid circuit")
            }
            Self::InvalidPolicy => formatter.write_str(
                "automatic schematic dimensions, spacing, and work bound must be positive",
            ),
            Self::InstanceLimit { instances, limit } => write!(
                formatter,
                "automatic schematic instance count {instances} exceeds limit {limit}"
            ),
            Self::InvalidIdentifier(identifier) => {
                write!(
                    formatter,
                    "invalid generated schematic identity {identifier:?}"
                )
            }
            Self::Arithmetic => {
                formatter.write_str("exact automatic schematic coordinate arithmetic failed")
            }
            Self::InvalidGeneratedLayout { issues } => write!(
                formatter,
                "generated schematic failed validation with {issues} issue(s)"
            ),
        }
    }
}

impl std::error::Error for SchematicAutoLayoutError {}

/// Placement evidence for one generated generic symbol.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct SchematicAutoPlacementEvidence {
    /// Authoritative circuit instance.
    pub instance: CircuitInstanceId,
    /// Generated drawing identity.
    pub symbol: SchematicSymbolId,
    /// Connectivity-derived zero-based column.
    pub column: usize,
    /// Stable zero-based row within the column.
    pub row: usize,
    /// Whether this instance seeded its connected component.
    pub component_seed: bool,
}

/// Complete derived drawing plus deterministic placement evidence.
#[derive(Clone, Debug, PartialEq)]
pub struct SchematicAutoLayoutReport {
    /// Structurally replayed schematic drawing.
    pub layout: SchematicLayout,
    /// Per-instance placement decisions.
    pub placements: Vec<SchematicAutoPlacementEvidence>,
    /// Count of non-ground connectivity edges used by breadth-first placement.
    pub connectivity_edges: usize,
    /// Count of generated typed wires.
    pub generated_wires: usize,
    /// Count of generated net labels.
    pub generated_labels: usize,
}

impl Circuit {
    /// Derives a flat generic schematic directly from retained connectivity.
    ///
    /// Non-ground nets define the placement graph. Independent sources and
    /// devices with driving pins seed breadth-first columns; components without
    /// a driver seed use their first retained instance. Ground is intentionally
    /// excluded from placement adjacency so a global reference net does not
    /// collapse every component into one column.
    pub fn auto_schematic(
        &self,
        policy: SchematicAutoLayoutPolicy,
    ) -> Result<SchematicAutoLayoutReport, SchematicAutoLayoutError> {
        validate_policy(&policy)?;
        if !self.validate().is_valid() {
            return Err(SchematicAutoLayoutError::InvalidCircuit);
        }
        if self.instances.len() > policy.max_instances {
            return Err(SchematicAutoLayoutError::InstanceLimit {
                instances: self.instances.len(),
                limit: policy.max_instances,
            });
        }

        let (adjacency, connectivity_edges) = connectivity_graph(self);
        let (columns, seeds) = connectivity_columns(self, &adjacency);
        let mut rows = BTreeMap::<usize, usize>::new();
        let mut layout = SchematicLayout::default();
        let mut placements = Vec::with_capacity(self.instances.len());
        let mut symbol_by_instance = BTreeMap::new();
        for model in &self.device_models {
            if self
                .instances
                .iter()
                .any(|instance| instance.model == model.id)
            {
                layout
                    .symbol_definitions
                    .push(generic_definition(model, &policy)?);
            }
        }

        for instance in &self.instances {
            let model = model_for(self, instance);
            let column = columns.get(&instance.id).copied().unwrap_or(0);
            let row = rows.entry(column).or_default();
            let position = SchematicPoint::new(
                policy.origin.x.clone() + policy.column_spacing.clone() * exact_index(column)?,
                policy.origin.y.clone() + policy.row_spacing.clone() * exact_index(*row)?,
            );
            let symbol_id = generated_symbol_id(instance)?;
            let symbol = generic_symbol(instance, model, symbol_id.clone(), position)?;
            symbol_by_instance.insert(instance.id.clone(), symbol_id.clone());
            placements.push(SchematicAutoPlacementEvidence {
                instance: instance.id.clone(),
                symbol: symbol_id,
                column,
                row: *row,
                component_seed: seeds.contains(&instance.id),
            });
            *row += 1;
            layout.symbols.push(symbol);
        }

        layout.ports = generated_ports(self, &columns, &policy)?;
        let endpoints = endpoints_by_net(self, &layout, &symbol_by_instance);
        let (wires, labels) = connect_and_label_nets(endpoints)?;
        layout.wires = wires;
        layout.labels = labels;

        let validation = layout.validate(self);
        if !validation.is_valid() {
            return Err(SchematicAutoLayoutError::InvalidGeneratedLayout {
                issues: validation.issues.len(),
            });
        }
        Ok(SchematicAutoLayoutReport {
            generated_wires: layout.wires.len(),
            generated_labels: layout.labels.len(),
            layout,
            placements,
            connectivity_edges,
        })
    }
}

#[cfg(feature = "layout")]
impl crate::CheckedDesign {
    /// Replaces the review drawing with a connectivity-derived schematic.
    ///
    /// This operation is explicit because a caller may prefer to preserve a
    /// hand-authored schematic. Circuit, PCB, and authoring provenance remain
    /// unchanged.
    pub fn replace_schematic_with_auto_layout(
        &mut self,
        policy: SchematicAutoLayoutPolicy,
    ) -> Result<SchematicAutoLayoutReport, SchematicAutoLayoutError> {
        let report = self.circuit.auto_schematic(policy)?;
        self.schematic = report.layout.clone();
        Ok(report)
    }
}

fn validate_policy(policy: &SchematicAutoLayoutPolicy) -> Result<(), SchematicAutoLayoutError> {
    if policy.max_instances == 0
        || [
            &policy.column_spacing,
            &policy.row_spacing,
            &policy.body_width,
            &policy.minimum_body_height,
            &policy.pin_pitch,
            &policy.lead_length,
        ]
        .into_iter()
        .any(|value| value.structural_facts().sign != Some(RealSign::Positive))
    {
        return Err(SchematicAutoLayoutError::InvalidPolicy);
    }
    Ok(())
}

fn connectivity_graph(
    circuit: &Circuit,
) -> (
    BTreeMap<CircuitInstanceId, BTreeSet<CircuitInstanceId>>,
    usize,
) {
    let mut adjacency = circuit
        .instances
        .iter()
        .map(|instance| (instance.id.clone(), BTreeSet::new()))
        .collect::<BTreeMap<_, _>>();
    let ground_nets = circuit
        .nets
        .iter()
        .filter(|net| net.is_ground)
        .map(|net| net.id.clone())
        .collect::<BTreeSet<_>>();
    let mut edges = BTreeSet::new();
    for net in circuit
        .nets
        .iter()
        .filter(|net| !ground_nets.contains(&net.id))
    {
        let connected = circuit
            .instances
            .iter()
            .filter(|instance| instance.pins.iter().any(|binding| binding.net == net.id))
            .map(|instance| instance.id.clone())
            .collect::<Vec<_>>();
        for left in 0..connected.len() {
            for right in (left + 1)..connected.len() {
                let edge = if connected[left] <= connected[right] {
                    (connected[left].clone(), connected[right].clone())
                } else {
                    (connected[right].clone(), connected[left].clone())
                };
                if edges.insert(edge.clone()) {
                    adjacency
                        .get_mut(&edge.0)
                        .expect("retained instance initialized")
                        .insert(edge.1.clone());
                    adjacency
                        .get_mut(&edge.1)
                        .expect("retained instance initialized")
                        .insert(edge.0);
                }
            }
        }
    }
    (adjacency, edges.len())
}

fn connectivity_columns(
    circuit: &Circuit,
    adjacency: &BTreeMap<CircuitInstanceId, BTreeSet<CircuitInstanceId>>,
) -> (
    BTreeMap<CircuitInstanceId, usize>,
    BTreeSet<CircuitInstanceId>,
) {
    let mut unassigned = circuit
        .instances
        .iter()
        .map(|instance| instance.id.clone())
        .collect::<BTreeSet<_>>();
    let mut columns = BTreeMap::new();
    let mut seeds = BTreeSet::new();

    for root in circuit.instances.iter().map(|instance| &instance.id) {
        if !unassigned.contains(root) {
            continue;
        }
        let mut component = Vec::new();
        let mut pending = vec![root.clone()];
        let mut seen = BTreeSet::new();
        while let Some(instance) = pending.pop() {
            if !seen.insert(instance.clone()) {
                continue;
            }
            component.push(instance.clone());
            if let Some(neighbors) = adjacency.get(&instance) {
                pending.extend(neighbors.iter().cloned());
            }
        }
        for instance in &component {
            unassigned.remove(instance);
        }
        let mut component_seeds = component
            .iter()
            .filter(|id| {
                circuit
                    .instances
                    .iter()
                    .find(|instance| &instance.id == *id)
                    .is_some_and(|instance| instance_is_driver(circuit, instance))
            })
            .cloned()
            .collect::<Vec<_>>();
        if component_seeds.is_empty() {
            component_seeds.push(root.clone());
        }
        let mut queue = VecDeque::new();
        for seed in component_seeds {
            if columns.insert(seed.clone(), 0).is_none() {
                seeds.insert(seed.clone());
                queue.push_back(seed);
            }
        }
        while let Some(instance) = queue.pop_front() {
            let next_column = columns[&instance] + 1;
            if let Some(neighbors) = adjacency.get(&instance) {
                for neighbor in neighbors {
                    if !columns.contains_key(neighbor) {
                        columns.insert(neighbor.clone(), next_column);
                        queue.push_back(neighbor.clone());
                    }
                }
            }
        }
    }
    (columns, seeds)
}

fn instance_is_driver(circuit: &Circuit, instance: &CircuitInstance) -> bool {
    let model = model_for(circuit, instance);
    matches!(
        model.kind,
        DeviceModelKind::VoltageSource | DeviceModelKind::CurrentSource
    ) || model.pins.iter().any(|pin| {
        matches!(
            pin.kind,
            PinElectricalKind::Output
                | PinElectricalKind::PowerOutput
                | PinElectricalKind::OpenCollector
                | PinElectricalKind::OpenEmitter
        )
    })
}

fn model_for<'a>(circuit: &'a Circuit, instance: &CircuitInstance) -> &'a DeviceModel {
    circuit
        .device_models
        .iter()
        .find(|model| model.id == instance.model)
        .expect("validated circuit instance model exists")
}

fn generated_symbol_id(
    instance: &CircuitInstance,
) -> Result<SchematicSymbolId, SchematicAutoLayoutError> {
    let value = format!("{}:auto:1", instance.id.as_str());
    SchematicSymbolId::new(value.clone())
        .map_err(|_| SchematicAutoLayoutError::InvalidIdentifier(value))
}

fn generated_definition_id(
    model: &DeviceModel,
) -> Result<SchematicSymbolDefinitionId, SchematicAutoLayoutError> {
    let value = format!("auto:{}", model.id.as_str());
    SchematicSymbolDefinitionId::new(value.clone())
        .map_err(|_| SchematicAutoLayoutError::InvalidIdentifier(value))
}

fn generic_definition(
    model: &DeviceModel,
    policy: &SchematicAutoLayoutPolicy,
) -> Result<SchematicSymbolDefinition, SchematicAutoLayoutError> {
    let extra_pins = model.pins.len().saturating_sub(1);
    let body_height =
        policy.minimum_body_height.clone() + policy.pin_pitch.clone() * exact_index(extra_pins)?;
    let sides = model
        .pins
        .iter()
        .enumerate()
        .map(|(index, pin)| (pin, pin_side(model, pin.kind, index)))
        .collect::<Vec<_>>();
    let mut side_totals = BTreeMap::new();
    for (_, side) in &sides {
        *side_totals.entry(side_index(*side)).or_insert(0usize) += 1;
    }
    let mut side_ranks = BTreeMap::new();
    let mut pins = Vec::with_capacity(sides.len());
    for (pin, side) in sides {
        let key = side_index(side);
        let rank = side_ranks.entry(key).or_insert(0usize);
        let total = side_totals[&key];
        let position = pin_position(side, *rank, total, &body_height, policy)?;
        *rank += 1;
        pins.push(SchematicPinPlacement {
            pin: pin.pin.clone(),
            position,
            side,
        });
    }
    let half_width = divide(policy.body_width.clone(), Real::from(2))?;
    let half_height = divide(body_height.clone(), Real::from(2))?;
    Ok(SchematicSymbolDefinition {
        id: generated_definition_id(model)?,
        model: model.id.clone(),
        name: format!("{} generic", model.id.as_str()),
        units: vec![SchematicSymbolUnit {
            unit: 1,
            body_width: policy.body_width.clone(),
            body_height,
            pins,
            graphics: vec![SchematicGraphic::Rectangle {
                start: SchematicPoint::new(-half_width.clone(), -half_height.clone()),
                end: SchematicPoint::new(half_width, half_height),
                stroke_width: Real::one(),
                fill: SchematicGraphicFill::Background,
            }],
        }],
    })
}

fn generic_symbol(
    instance: &CircuitInstance,
    model: &DeviceModel,
    id: SchematicSymbolId,
    position: SchematicPoint,
) -> Result<SchematicSymbol, SchematicAutoLayoutError> {
    Ok(SchematicSymbol {
        id,
        instance: instance.id.clone(),
        definition: generated_definition_id(model)?,
        unit: 1,
        position,
        quarter_turns: 0,
    })
}

fn pin_side(model: &DeviceModel, kind: PinElectricalKind, index: usize) -> SchematicPinSide {
    if matches!(
        model.kind,
        DeviceModelKind::VoltageSource | DeviceModelKind::CurrentSource
    ) {
        return if index == 0 {
            SchematicPinSide::Top
        } else {
            SchematicPinSide::Bottom
        };
    }
    match kind {
        PinElectricalKind::Input => SchematicPinSide::Left,
        PinElectricalKind::PowerInput => SchematicPinSide::Top,
        PinElectricalKind::Output
        | PinElectricalKind::PowerOutput
        | PinElectricalKind::OpenCollector
        | PinElectricalKind::OpenEmitter
        | PinElectricalKind::Bidirectional => SchematicPinSide::Right,
        PinElectricalKind::Passive | PinElectricalKind::NotConnected => {
            if index.is_multiple_of(2) {
                SchematicPinSide::Left
            } else {
                SchematicPinSide::Right
            }
        }
    }
}

fn side_index(side: SchematicPinSide) -> u8 {
    match side {
        SchematicPinSide::Left => 0,
        SchematicPinSide::Right => 1,
        SchematicPinSide::Top => 2,
        SchematicPinSide::Bottom => 3,
    }
}

fn pin_position(
    side: SchematicPinSide,
    rank: usize,
    total: usize,
    body_height: &Real,
    policy: &SchematicAutoLayoutPolicy,
) -> Result<SchematicPoint, SchematicAutoLayoutError> {
    let half_width = divide(policy.body_width.clone(), Real::from(2))?;
    let half_height = divide(body_height.clone(), Real::from(2))?;
    let horizontal = distributed_coordinate(&policy.body_width, rank, total)?;
    let vertical = distributed_coordinate(body_height, rank, total)?;
    Ok(match side {
        SchematicPinSide::Left => {
            SchematicPoint::new(-half_width - policy.lead_length.clone(), vertical)
        }
        SchematicPinSide::Right => {
            SchematicPoint::new(half_width + policy.lead_length.clone(), vertical)
        }
        SchematicPinSide::Top => {
            SchematicPoint::new(horizontal, -half_height - policy.lead_length.clone())
        }
        SchematicPinSide::Bottom => {
            SchematicPoint::new(horizontal, half_height + policy.lead_length.clone())
        }
    })
}

fn distributed_coordinate(
    span: &Real,
    rank: usize,
    total: usize,
) -> Result<Real, SchematicAutoLayoutError> {
    let half = divide(span.clone(), Real::from(2))?;
    let fraction = divide(exact_index(rank + 1)?, exact_index(total + 1)?)?;
    Ok(-half + span.clone() * fraction)
}

fn generated_ports(
    circuit: &Circuit,
    columns: &BTreeMap<CircuitInstanceId, usize>,
    policy: &SchematicAutoLayoutPolicy,
) -> Result<Vec<SchematicPortPlacement>, SchematicAutoLayoutError> {
    let max_column = columns.values().copied().max().unwrap_or(0);
    let mut left_row = 0usize;
    let mut right_row = 0usize;
    let mut ports = Vec::with_capacity(circuit.ports.len());
    for port in &circuit.ports {
        let right = matches!(
            port.direction,
            PortDirection::Output | PortDirection::PowerOutput
        );
        let row = if right {
            let row = right_row;
            right_row += 1;
            row
        } else {
            let row = left_row;
            left_row += 1;
            row
        };
        let x = if right {
            policy.origin.x.clone() + policy.column_spacing.clone() * exact_index(max_column + 1)?
        } else {
            policy.origin.x.clone() - policy.column_spacing.clone()
        };
        let y = policy.origin.y.clone() + policy.row_spacing.clone() * exact_index(row)?;
        ports.push(SchematicPortPlacement {
            port: port.id.clone(),
            position: SchematicPoint::new(x, y),
        });
    }
    Ok(ports)
}

fn endpoints_by_net(
    circuit: &Circuit,
    layout: &SchematicLayout,
    symbol_by_instance: &BTreeMap<CircuitInstanceId, SchematicSymbolId>,
) -> BTreeMap<crate::NetId, Vec<(SchematicEndpoint, SchematicPoint)>> {
    let mut endpoints = BTreeMap::<_, Vec<_>>::new();
    for instance in &circuit.instances {
        let Some(symbol_id) = symbol_by_instance.get(&instance.id) else {
            continue;
        };
        let symbol = layout
            .symbols
            .iter()
            .find(|symbol| &symbol.id == symbol_id)
            .expect("generated symbol map is complete");
        let unit = layout
            .symbol_unit(symbol)
            .expect("generated symbol definition is complete");
        for binding in &instance.pins {
            let Some(pin) = unit.pins.iter().find(|pin| pin.pin == binding.pin) else {
                continue;
            };
            endpoints.entry(binding.net.clone()).or_default().push((
                SchematicEndpoint::Pin {
                    symbol: symbol.id.clone(),
                    pin: binding.pin.clone(),
                },
                absolute_pin_point(symbol, pin),
            ));
        }
    }
    for port in &layout.ports {
        if let Some(retained) = circuit
            .ports
            .iter()
            .find(|candidate| candidate.id == port.port)
        {
            endpoints.entry(retained.net.clone()).or_default().push((
                SchematicEndpoint::Port(port.port.clone()),
                port.position.clone(),
            ));
        }
    }
    endpoints
}

fn absolute_pin_point(symbol: &SchematicSymbol, pin: &SchematicPinPlacement) -> SchematicPoint {
    SchematicPoint::new(
        symbol.position.x.clone() + pin.position.x.clone(),
        symbol.position.y.clone() + pin.position.y.clone(),
    )
}

fn connect_and_label_nets(
    endpoints: BTreeMap<crate::NetId, Vec<(SchematicEndpoint, SchematicPoint)>>,
) -> Result<(Vec<SchematicWire>, Vec<SchematicLabel>), SchematicAutoLayoutError> {
    let mut wires = Vec::new();
    let mut labels = Vec::new();
    for (net, endpoints) in endpoints {
        for (index, pair) in endpoints.windows(2).enumerate() {
            let midpoint_x = divide(pair[0].1.x.clone() + pair[1].1.x.clone(), Real::from(2))?;
            let id = format!("auto-wire:{}:{}", net.as_str(), index + 1);
            wires.push(SchematicWire {
                id: SchematicWireId::new(id.clone())
                    .map_err(|_| SchematicAutoLayoutError::InvalidIdentifier(id))?,
                net: net.clone(),
                from: pair[0].0.clone(),
                waypoints: vec![
                    SchematicPoint::new(midpoint_x.clone(), pair[0].1.y.clone()),
                    SchematicPoint::new(midpoint_x, pair[1].1.y.clone()),
                ],
                to: pair[1].0.clone(),
            });
        }
        if let Some((_, point)) = endpoints.first() {
            let id = format!("auto-label:{}", net.as_str());
            labels.push(SchematicLabel {
                id: SchematicLabelId::new(id.clone())
                    .map_err(|_| SchematicAutoLayoutError::InvalidIdentifier(id))?,
                text: net.as_str().into(),
                net,
                position: point.clone(),
            });
        }
    }
    Ok((wires, labels))
}

fn exact_index(index: usize) -> Result<Real, SchematicAutoLayoutError> {
    i64::try_from(index)
        .map(Real::from)
        .map_err(|_| SchematicAutoLayoutError::Arithmetic)
}

fn divide(numerator: Real, denominator: Real) -> Result<Real, SchematicAutoLayoutError> {
    (numerator / denominator).map_err(|_| SchematicAutoLayoutError::Arithmetic)
}
