//! Circuit-owned schematic placement, connectivity validation, and SVG review output.

use std::collections::{BTreeMap, BTreeSet};
use std::fmt::{Display, Formatter, Write};

use hyperreal::{Real, RealSign};

use crate::{
    Circuit, CircuitInstanceId, DeviceModelId, NetId, PinRef, PortId, SchematicLabelId,
    SchematicSheetId, SchematicSheetLinkId, SchematicSheetPortId, SchematicSymbolDefinitionId,
    SchematicSymbolId, SchematicWireId,
};

/// Exact point in schematic drawing coordinates.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, PartialEq)]
pub struct SchematicPoint {
    /// Horizontal coordinate.
    pub x: Real,
    /// Vertical coordinate.
    pub y: Real,
}

impl SchematicPoint {
    /// Constructs an exact schematic point.
    pub fn new(x: Real, y: Real) -> Self {
        Self { x, y }
    }
}

/// Body edge from which a symbol pin visually exits.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SchematicPinSide {
    /// Left edge.
    Left,
    /// Right edge.
    Right,
    /// Top edge.
    Top,
    /// Bottom edge.
    Bottom,
}

/// One pin's symbol-local connection point.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, PartialEq)]
pub struct SchematicPinPlacement {
    /// Logical device pin.
    pub pin: PinRef,
    /// Symbol-local exact connection point.
    pub position: SchematicPoint,
    /// Visual body edge.
    pub side: SchematicPinSide,
}

/// Fill policy for a reusable schematic graphic.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SchematicGraphicFill {
    /// Do not fill the enclosed region.
    None,
    /// Fill with the schematic canvas color.
    Background,
    /// Fill with the schematic stroke color.
    Foreground,
}

/// Exact symbol-local graphic retained by a reusable schematic unit.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, PartialEq)]
pub enum SchematicGraphic {
    /// Straight line segment.
    Line {
        /// First endpoint.
        from: SchematicPoint,
        /// Second endpoint.
        to: SchematicPoint,
        /// Exact stroke width.
        stroke_width: Real,
    },
    /// Axis-aligned rectangle.
    Rectangle {
        /// First corner.
        start: SchematicPoint,
        /// Opposite corner.
        end: SchematicPoint,
        /// Exact stroke width.
        stroke_width: Real,
        /// Interior fill.
        fill: SchematicGraphicFill,
    },
    /// Circle.
    Circle {
        /// Circle center.
        center: SchematicPoint,
        /// Exact radius.
        radius: Real,
        /// Exact stroke width.
        stroke_width: Real,
        /// Interior fill.
        fill: SchematicGraphicFill,
    },
    /// Circular arc defined by exact start, midpoint, and end points.
    Arc {
        /// Arc start.
        start: SchematicPoint,
        /// Point on the arc between start and end.
        mid: SchematicPoint,
        /// Arc end.
        end: SchematicPoint,
        /// Exact stroke width.
        stroke_width: Real,
    },
    /// Open or closed polyline.
    Polyline {
        /// Vertices in drawing order.
        points: Vec<SchematicPoint>,
        /// Whether the last vertex connects to the first.
        closed: bool,
        /// Exact stroke width.
        stroke_width: Real,
        /// Interior fill when closed.
        fill: SchematicGraphicFill,
    },
    /// Retained symbol-local annotation.
    Text {
        /// Text anchor.
        position: SchematicPoint,
        /// Authored text.
        text: String,
        /// Exact text height.
        size: Real,
        /// Clockwise quarter turns.
        quarter_turns: i8,
    },
}

/// One independently placeable unit in a reusable multipart symbol.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, PartialEq)]
pub struct SchematicSymbolUnit {
    /// Stable unit number, starting at one.
    pub unit: u16,
    /// Unrotated nominal body width used by layout policy.
    pub body_width: Real,
    /// Unrotated nominal body height used by layout policy.
    pub body_height: Real,
    /// Pins shown by this unit.
    pub pins: Vec<SchematicPinPlacement>,
    /// Exact reusable drawing primitives.
    pub graphics: Vec<SchematicGraphic>,
}

/// Reusable multipart schematic drawing bound to one logical device model.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, PartialEq)]
pub struct SchematicSymbolDefinition {
    /// Stable library identity.
    pub id: SchematicSymbolDefinitionId,
    /// Logical device model whose pins the drawing presents.
    pub model: DeviceModelId,
    /// Human-readable library name.
    pub name: String,
    /// Independently placeable units.
    pub units: Vec<SchematicSymbolUnit>,
}

/// One placed schematic unit representing a logical circuit instance.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, PartialEq)]
pub struct SchematicSymbol {
    /// Stable drawing identity; multiple units may reference one instance.
    pub id: SchematicSymbolId,
    /// Logical circuit instance represented by this symbol unit.
    pub instance: CircuitInstanceId,
    /// Reusable drawing definition.
    pub definition: SchematicSymbolDefinitionId,
    /// Unit number for multipart symbols.
    pub unit: u16,
    /// Exact symbol center.
    pub position: SchematicPoint,
    /// Clockwise quarter turns in drawing coordinates.
    pub quarter_turns: i8,
}

/// Display position of a retained circuit-boundary port.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, PartialEq)]
pub struct SchematicPortPlacement {
    /// Circuit port being drawn.
    pub port: PortId,
    /// Exact connection point.
    pub position: SchematicPoint,
}

/// Typed endpoint of a schematic wire.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, PartialEq)]
pub enum SchematicEndpoint {
    /// Pin shown by one symbol unit.
    Pin {
        /// Symbol drawing identity.
        symbol: SchematicSymbolId,
        /// Logical pin on that symbol.
        pin: PinRef,
    },
    /// Circuit-boundary port.
    Port(PortId),
    /// Boundary port on one explicit schematic sheet.
    SheetPort(SchematicSheetPortId),
    /// Free junction on the declared wire net.
    Junction(SchematicPoint),
}

/// One net-aware schematic wire with optional authored bend points.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, PartialEq)]
pub struct SchematicWire {
    /// Stable wire identity.
    pub id: SchematicWireId,
    /// Logical net carried by the wire.
    pub net: NetId,
    /// First typed endpoint.
    pub from: SchematicEndpoint,
    /// Intermediate exact bend points.
    pub waypoints: Vec<SchematicPoint>,
    /// Second typed endpoint.
    pub to: SchematicEndpoint,
}

/// Retained net label in the schematic drawing.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, PartialEq)]
pub struct SchematicLabel {
    /// Stable label identity.
    pub id: SchematicLabelId,
    /// Logical net labeled.
    pub net: NetId,
    /// Exact text anchor.
    pub position: SchematicPoint,
    /// Authored display text; the net id remains authoritative.
    pub text: String,
}

/// One page in an explicit hierarchical schematic book.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, PartialEq)]
pub struct SchematicSheet {
    /// Stable page identity.
    pub id: SchematicSheetId,
    /// Human-readable page title.
    pub title: String,
    /// Direct containing sheet; `None` identifies the book root.
    pub parent: Option<SchematicSheetId>,
    /// Symbol units presented on this page.
    pub symbols: Vec<SchematicSymbolId>,
    /// Circuit-port placements presented on this page.
    pub ports: Vec<PortId>,
    /// Wires presented on this page.
    pub wires: Vec<SchematicWireId>,
    /// Net labels presented on this page.
    pub labels: Vec<SchematicLabelId>,
}

/// Typed net boundary exposed on one schematic sheet.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, PartialEq)]
pub struct SchematicSheetPort {
    /// Stable boundary-port identity.
    pub id: SchematicSheetPortId,
    /// Sheet on which this boundary is drawn.
    pub sheet: SchematicSheetId,
    /// Authoritative circuit net crossing the boundary.
    pub net: NetId,
    /// Displayed boundary name.
    pub name: String,
    /// Exact sheet-local connection point.
    pub position: SchematicPoint,
}

/// Explicit parent/child connection between two sheet boundary ports.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct SchematicSheetLink {
    /// Stable link identity.
    pub id: SchematicSheetLinkId,
    /// Port drawn on the direct parent sheet.
    pub parent_port: SchematicSheetPortId,
    /// Port drawn on the direct child sheet.
    pub child_port: SchematicSheetPortId,
}

/// Complete schematic view associated with one circuit scope.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, Default, PartialEq)]
pub struct SchematicLayout {
    /// Reusable multipart symbol library.
    #[cfg_attr(feature = "interchange", serde(default))]
    pub symbol_definitions: Vec<SchematicSymbolDefinition>,
    /// Placed component symbol units.
    pub symbols: Vec<SchematicSymbol>,
    /// Placed circuit ports.
    pub ports: Vec<SchematicPortPlacement>,
    /// Net-aware wires.
    pub wires: Vec<SchematicWire>,
    /// Net labels.
    pub labels: Vec<SchematicLabel>,
    /// Explicit page hierarchy; empty retains the legacy implicit single page.
    #[cfg_attr(feature = "interchange", serde(default))]
    pub sheets: Vec<SchematicSheet>,
    /// Typed page-boundary ports.
    #[cfg_attr(feature = "interchange", serde(default))]
    pub sheet_ports: Vec<SchematicSheetPort>,
    /// Direct parent/child boundary connections.
    #[cfg_attr(feature = "interchange", serde(default))]
    pub sheet_links: Vec<SchematicSheetLink>,
}

/// Structural or electrical inconsistency in a schematic view.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum SchematicValidationIssue {
    /// Two sheets share one stable identity.
    DuplicateSheet(SchematicSheetId),
    /// A sheet has no usable display title.
    InvalidSheetTitle(SchematicSheetId),
    /// Explicit hierarchy must contain exactly one root page.
    InvalidRootSheetCount(usize),
    /// A sheet references an absent direct parent.
    UnknownSheetParent {
        sheet: SchematicSheetId,
        parent: SchematicSheetId,
    },
    /// Sheet parent links contain a cycle.
    SheetHierarchyCycle(SchematicSheetId),
    /// A sheet lists an absent symbol drawing.
    UnknownSheetSymbol {
        sheet: SchematicSheetId,
        symbol: SchematicSymbolId,
    },
    /// A sheet lists an absent circuit-port placement.
    UnknownSheetPortPlacement {
        sheet: SchematicSheetId,
        port: PortId,
    },
    /// A sheet lists an absent wire drawing.
    UnknownSheetWire {
        sheet: SchematicSheetId,
        wire: SchematicWireId,
    },
    /// A sheet lists an absent label drawing.
    UnknownSheetLabel {
        sheet: SchematicSheetId,
        label: SchematicLabelId,
    },
    /// One drawing item is assigned to more than one sheet.
    DuplicateSheetContent(String),
    /// An explicit book leaves one drawing item outside every sheet.
    UnassignedSheetContent(String),
    /// Two sheet boundary ports share one identity.
    DuplicateSheetPort(SchematicSheetPortId),
    /// A sheet boundary port references an absent page.
    UnknownSheetPortSheet(SchematicSheetPortId),
    /// A sheet boundary port references an absent circuit net.
    UnknownSheetPortNet {
        port: SchematicSheetPortId,
        net: NetId,
    },
    /// A sheet boundary port has an empty display name.
    InvalidSheetPortName(SchematicSheetPortId),
    /// Two sheet links share one identity.
    DuplicateSheetLink(SchematicSheetLinkId),
    /// A sheet link references an absent boundary port.
    UnknownSheetLinkPort(SchematicSheetLinkId),
    /// A sheet link does not connect one direct parent/child page pair.
    InvalidSheetLinkRelation(SchematicSheetLinkId),
    /// Linked sheet ports disagree on their authoritative circuit net.
    SheetLinkNetMismatch(SchematicSheetLinkId),
    /// A boundary port participates in more than one hierarchical link.
    DuplicateSheetPortLink(SchematicSheetPortId),
    /// Two symbols share one drawing identity.
    DuplicateSymbol(SchematicSymbolId),
    /// Two reusable symbol definitions share one stable identity.
    DuplicateSymbolDefinition(SchematicSymbolDefinitionId),
    /// A reusable symbol definition has no usable display name.
    InvalidSymbolDefinitionName(SchematicSymbolDefinitionId),
    /// A reusable symbol definition references an absent device model.
    UnknownSymbolDefinitionModel {
        definition: SchematicSymbolDefinitionId,
        model: DeviceModelId,
    },
    /// A reusable symbol definition repeats a unit number.
    DuplicateSymbolUnit {
        definition: SchematicSymbolDefinitionId,
        unit: u16,
    },
    /// A reusable symbol unit has an invalid number.
    InvalidSymbolUnit {
        definition: SchematicSymbolDefinitionId,
        unit: u16,
    },
    /// A reusable symbol unit has no positive exact body extent.
    InvalidSymbolUnitBody {
        definition: SchematicSymbolDefinitionId,
        unit: u16,
    },
    /// A reusable symbol unit repeats a logical pin.
    DuplicateSymbolDefinitionPin {
        definition: SchematicSymbolDefinitionId,
        unit: u16,
        pin: PinRef,
    },
    /// A reusable symbol unit presents a pin absent from its device model.
    UnknownSymbolDefinitionPin {
        definition: SchematicSymbolDefinitionId,
        unit: u16,
        pin: PinRef,
    },
    /// A reusable symbol graphic has invalid exact geometry.
    InvalidSymbolGraphic {
        definition: SchematicSymbolDefinitionId,
        unit: u16,
        graphic: usize,
    },
    /// A symbol references an absent logical instance.
    UnknownSymbolInstance(CircuitInstanceId),
    /// A symbol references an absent reusable definition.
    UnknownSymbolDefinition {
        symbol: SchematicSymbolId,
        definition: SchematicSymbolDefinitionId,
    },
    /// A symbol references a unit absent from its reusable definition.
    UnknownPlacedSymbolUnit {
        symbol: SchematicSymbolId,
        definition: SchematicSymbolDefinitionId,
        unit: u16,
    },
    /// A symbol definition targets a different device model than its instance.
    SymbolDefinitionModelMismatch(SchematicSymbolId),
    /// Two placements show one circuit port.
    DuplicatePortPlacement(PortId),
    /// A placed port is absent from the circuit.
    UnknownPortPlacement(PortId),
    /// Two wires share one identity.
    DuplicateWire(SchematicWireId),
    /// A wire references an absent net.
    UnknownWireNet { wire: SchematicWireId, net: NetId },
    /// A pin endpoint is absent from the referenced symbol.
    UnknownWirePinEndpoint { wire: SchematicWireId },
    /// A port endpoint lacks a placement or circuit declaration.
    UnknownWirePortEndpoint { wire: SchematicWireId },
    /// A sheet-port endpoint lacks a retained boundary declaration.
    UnknownWireSheetPortEndpoint { wire: SchematicWireId },
    /// A wire and its typed endpoint are assigned to different pages.
    WireEndpointSheetMismatch { wire: SchematicWireId },
    /// A typed pin/port endpoint belongs to a different net.
    WireEndpointNetMismatch { wire: SchematicWireId, net: NetId },
    /// Two labels share one identity.
    DuplicateLabel(SchematicLabelId),
    /// A label references an absent net.
    UnknownLabelNet { label: SchematicLabelId, net: NetId },
}

/// Deterministic schematic validation report.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct SchematicValidationReport {
    /// Every issue in source order.
    pub issues: Vec<SchematicValidationIssue>,
}

impl SchematicValidationReport {
    /// True when the drawing agrees with retained circuit identities.
    pub fn is_valid(&self) -> bool {
        self.issues.is_empty()
    }
}

impl SchematicLayout {
    /// Resolves a reusable symbol definition by stable identity.
    pub fn symbol_definition(
        &self,
        id: &SchematicSymbolDefinitionId,
    ) -> Option<&SchematicSymbolDefinition> {
        self.symbol_definitions
            .iter()
            .find(|definition| &definition.id == id)
    }

    /// Resolves the reusable unit drawn by one placement.
    pub fn symbol_unit(&self, symbol: &SchematicSymbol) -> Option<&SchematicSymbolUnit> {
        self.symbol_definition(&symbol.definition)?
            .units
            .iter()
            .find(|unit| unit.unit == symbol.unit)
    }

    /// Validates drawing identities and typed endpoint net consistency.
    pub fn validate(&self, circuit: &Circuit) -> SchematicValidationReport {
        let mut issues = Vec::new();
        let net_ids = circuit
            .nets
            .iter()
            .map(|net| net.id.clone())
            .collect::<BTreeSet<_>>();
        let sheet_membership = validate_sheet_hierarchy(self, &net_ids, &mut issues);
        let mut definition_ids = BTreeSet::new();
        for definition in &self.symbol_definitions {
            if !definition_ids.insert(definition.id.clone()) {
                issues.push(SchematicValidationIssue::DuplicateSymbolDefinition(
                    definition.id.clone(),
                ));
            }
            if definition.name.trim().is_empty() {
                issues.push(SchematicValidationIssue::InvalidSymbolDefinitionName(
                    definition.id.clone(),
                ));
            }
            let model = circuit
                .device_models
                .iter()
                .find(|model| model.id == definition.model);
            if model.is_none() {
                issues.push(SchematicValidationIssue::UnknownSymbolDefinitionModel {
                    definition: definition.id.clone(),
                    model: definition.model.clone(),
                });
            }
            let mut units = BTreeSet::new();
            for unit in &definition.units {
                if unit.unit == 0 {
                    issues.push(SchematicValidationIssue::InvalidSymbolUnit {
                        definition: definition.id.clone(),
                        unit: unit.unit,
                    });
                }
                if !units.insert(unit.unit) {
                    issues.push(SchematicValidationIssue::DuplicateSymbolUnit {
                        definition: definition.id.clone(),
                        unit: unit.unit,
                    });
                }
                if !positive(&unit.body_width) || !positive(&unit.body_height) {
                    issues.push(SchematicValidationIssue::InvalidSymbolUnitBody {
                        definition: definition.id.clone(),
                        unit: unit.unit,
                    });
                }
                let mut pins = BTreeSet::new();
                for pin in &unit.pins {
                    if !pins.insert(pin.pin.clone()) {
                        issues.push(SchematicValidationIssue::DuplicateSymbolDefinitionPin {
                            definition: definition.id.clone(),
                            unit: unit.unit,
                            pin: pin.pin.clone(),
                        });
                    }
                    if model.is_some_and(|model| {
                        !model.pins.iter().any(|candidate| candidate.pin == pin.pin)
                    }) {
                        issues.push(SchematicValidationIssue::UnknownSymbolDefinitionPin {
                            definition: definition.id.clone(),
                            unit: unit.unit,
                            pin: pin.pin.clone(),
                        });
                    }
                }
                for (graphic, primitive) in unit.graphics.iter().enumerate() {
                    if !valid_graphic(primitive) {
                        issues.push(SchematicValidationIssue::InvalidSymbolGraphic {
                            definition: definition.id.clone(),
                            unit: unit.unit,
                            graphic,
                        });
                    }
                }
            }
        }
        let mut symbol_ids = BTreeSet::new();
        for symbol in &self.symbols {
            if !symbol_ids.insert(symbol.id.clone()) {
                issues.push(SchematicValidationIssue::DuplicateSymbol(symbol.id.clone()));
            }
            let Some(instance) = circuit
                .instances
                .iter()
                .find(|instance| instance.id == symbol.instance)
            else {
                issues.push(SchematicValidationIssue::UnknownSymbolInstance(
                    symbol.instance.clone(),
                ));
                continue;
            };
            let Some(definition) = self.symbol_definition(&symbol.definition) else {
                issues.push(SchematicValidationIssue::UnknownSymbolDefinition {
                    symbol: symbol.id.clone(),
                    definition: symbol.definition.clone(),
                });
                continue;
            };
            if definition.model != instance.model {
                issues.push(SchematicValidationIssue::SymbolDefinitionModelMismatch(
                    symbol.id.clone(),
                ));
            }
            if !definition.units.iter().any(|unit| unit.unit == symbol.unit) {
                issues.push(SchematicValidationIssue::UnknownPlacedSymbolUnit {
                    symbol: symbol.id.clone(),
                    definition: symbol.definition.clone(),
                    unit: symbol.unit,
                });
            }
        }

        let mut port_ids = BTreeSet::new();
        for port in &self.ports {
            if !port_ids.insert(port.port.clone()) {
                issues.push(SchematicValidationIssue::DuplicatePortPlacement(
                    port.port.clone(),
                ));
            }
            if !circuit
                .ports
                .iter()
                .any(|candidate| candidate.id == port.port)
            {
                issues.push(SchematicValidationIssue::UnknownPortPlacement(
                    port.port.clone(),
                ));
            }
        }

        let mut wire_ids = BTreeSet::new();
        for wire in &self.wires {
            if !wire_ids.insert(wire.id.clone()) {
                issues.push(SchematicValidationIssue::DuplicateWire(wire.id.clone()));
            }
            if !net_ids.contains(&wire.net) {
                issues.push(SchematicValidationIssue::UnknownWireNet {
                    wire: wire.id.clone(),
                    net: wire.net.clone(),
                });
            }
            validate_endpoint(
                self,
                circuit,
                wire,
                &wire.from,
                &sheet_membership,
                &mut issues,
            );
            validate_endpoint(
                self,
                circuit,
                wire,
                &wire.to,
                &sheet_membership,
                &mut issues,
            );
        }

        let mut label_ids = BTreeSet::new();
        for label in &self.labels {
            if !label_ids.insert(label.id.clone()) {
                issues.push(SchematicValidationIssue::DuplicateLabel(label.id.clone()));
            }
            if !net_ids.contains(&label.net) {
                issues.push(SchematicValidationIssue::UnknownLabelNet {
                    label: label.id.clone(),
                    net: label.net.clone(),
                });
            }
        }
        SchematicValidationReport { issues }
    }
}

fn valid_graphic(graphic: &SchematicGraphic) -> bool {
    match graphic {
        SchematicGraphic::Line {
            from,
            to,
            stroke_width,
        } => positive(stroke_width) && from != to,
        SchematicGraphic::Rectangle {
            start,
            end,
            stroke_width,
            ..
        } => positive(stroke_width) && start.x != end.x && start.y != end.y,
        SchematicGraphic::Circle {
            radius,
            stroke_width,
            ..
        } => positive(radius) && positive(stroke_width),
        SchematicGraphic::Arc {
            start,
            mid,
            end,
            stroke_width,
        } => {
            positive(stroke_width)
                && start != mid
                && mid != end
                && start != end
                && (mid.x.clone() - start.x.clone()) * (end.y.clone() - start.y.clone())
                    - (mid.y.clone() - start.y.clone()) * (end.x.clone() - start.x.clone())
                    != Real::zero()
        }
        SchematicGraphic::Polyline {
            points,
            closed,
            stroke_width,
            ..
        } => {
            positive(stroke_width)
                && points.len() >= if *closed { 3 } else { 2 }
                && points.windows(2).all(|pair| pair[0] != pair[1])
        }
        SchematicGraphic::Text { text, size, .. } => !text.is_empty() && positive(size),
    }
}

#[derive(Default)]
struct SheetMembership {
    symbols: BTreeMap<SchematicSymbolId, SchematicSheetId>,
    ports: BTreeMap<PortId, SchematicSheetId>,
    wires: BTreeMap<SchematicWireId, SchematicSheetId>,
    labels: BTreeMap<SchematicLabelId, SchematicSheetId>,
}

fn validate_sheet_hierarchy(
    layout: &SchematicLayout,
    net_ids: &BTreeSet<NetId>,
    issues: &mut Vec<SchematicValidationIssue>,
) -> SheetMembership {
    let mut membership = SheetMembership::default();
    let mut sheet_ids = BTreeSet::new();
    for sheet in &layout.sheets {
        if !sheet_ids.insert(sheet.id.clone()) {
            issues.push(SchematicValidationIssue::DuplicateSheet(sheet.id.clone()));
        }
        if sheet.title.trim().is_empty() {
            issues.push(SchematicValidationIssue::InvalidSheetTitle(
                sheet.id.clone(),
            ));
        }
    }
    if layout.sheets.is_empty() {
        for port in &layout.sheet_ports {
            issues.push(SchematicValidationIssue::UnknownSheetPortSheet(
                port.id.clone(),
            ));
        }
        for link in &layout.sheet_links {
            issues.push(SchematicValidationIssue::UnknownSheetLinkPort(
                link.id.clone(),
            ));
        }
        return membership;
    }
    let root_count = layout
        .sheets
        .iter()
        .filter(|sheet| sheet.parent.is_none())
        .count();
    if root_count != 1 {
        issues.push(SchematicValidationIssue::InvalidRootSheetCount(root_count));
    }
    let parents = layout
        .sheets
        .iter()
        .filter_map(|sheet| {
            sheet
                .parent
                .as_ref()
                .map(|parent| (sheet.id.clone(), parent.clone()))
        })
        .collect::<BTreeMap<_, _>>();
    for sheet in &layout.sheets {
        if let Some(parent) = &sheet.parent
            && !sheet_ids.contains(parent)
        {
            issues.push(SchematicValidationIssue::UnknownSheetParent {
                sheet: sheet.id.clone(),
                parent: parent.clone(),
            });
        }
        let mut visited = BTreeSet::new();
        let mut cursor = sheet.id.clone();
        while let Some(parent) = parents.get(&cursor) {
            if !visited.insert(cursor.clone()) {
                issues.push(SchematicValidationIssue::SheetHierarchyCycle(
                    sheet.id.clone(),
                ));
                break;
            }
            if !sheet_ids.contains(parent) {
                break;
            }
            cursor = parent.clone();
        }
    }

    let symbol_ids = layout
        .symbols
        .iter()
        .map(|symbol| symbol.id.clone())
        .collect::<BTreeSet<_>>();
    let port_ids = layout
        .ports
        .iter()
        .map(|port| port.port.clone())
        .collect::<BTreeSet<_>>();
    let wire_ids = layout
        .wires
        .iter()
        .map(|wire| wire.id.clone())
        .collect::<BTreeSet<_>>();
    let label_ids = layout
        .labels
        .iter()
        .map(|label| label.id.clone())
        .collect::<BTreeSet<_>>();
    for sheet in &layout.sheets {
        for symbol in &sheet.symbols {
            if !symbol_ids.contains(symbol) {
                issues.push(SchematicValidationIssue::UnknownSheetSymbol {
                    sheet: sheet.id.clone(),
                    symbol: symbol.clone(),
                });
            }
            if membership
                .symbols
                .insert(symbol.clone(), sheet.id.clone())
                .is_some()
            {
                issues.push(SchematicValidationIssue::DuplicateSheetContent(format!(
                    "symbol:{}",
                    symbol.as_str()
                )));
            }
        }
        for port in &sheet.ports {
            if !port_ids.contains(port) {
                issues.push(SchematicValidationIssue::UnknownSheetPortPlacement {
                    sheet: sheet.id.clone(),
                    port: port.clone(),
                });
            }
            if membership
                .ports
                .insert(port.clone(), sheet.id.clone())
                .is_some()
            {
                issues.push(SchematicValidationIssue::DuplicateSheetContent(format!(
                    "port:{}",
                    port.as_str()
                )));
            }
        }
        for wire in &sheet.wires {
            if !wire_ids.contains(wire) {
                issues.push(SchematicValidationIssue::UnknownSheetWire {
                    sheet: sheet.id.clone(),
                    wire: wire.clone(),
                });
            }
            if membership
                .wires
                .insert(wire.clone(), sheet.id.clone())
                .is_some()
            {
                issues.push(SchematicValidationIssue::DuplicateSheetContent(format!(
                    "wire:{}",
                    wire.as_str()
                )));
            }
        }
        for label in &sheet.labels {
            if !label_ids.contains(label) {
                issues.push(SchematicValidationIssue::UnknownSheetLabel {
                    sheet: sheet.id.clone(),
                    label: label.clone(),
                });
            }
            if membership
                .labels
                .insert(label.clone(), sheet.id.clone())
                .is_some()
            {
                issues.push(SchematicValidationIssue::DuplicateSheetContent(format!(
                    "label:{}",
                    label.as_str()
                )));
            }
        }
    }
    for symbol in &layout.symbols {
        if !membership.symbols.contains_key(&symbol.id) {
            issues.push(SchematicValidationIssue::UnassignedSheetContent(format!(
                "symbol:{}",
                symbol.id.as_str()
            )));
        }
    }
    for port in &layout.ports {
        if !membership.ports.contains_key(&port.port) {
            issues.push(SchematicValidationIssue::UnassignedSheetContent(format!(
                "port:{}",
                port.port.as_str()
            )));
        }
    }
    for wire in &layout.wires {
        if !membership.wires.contains_key(&wire.id) {
            issues.push(SchematicValidationIssue::UnassignedSheetContent(format!(
                "wire:{}",
                wire.id.as_str()
            )));
        }
    }
    for label in &layout.labels {
        if !membership.labels.contains_key(&label.id) {
            issues.push(SchematicValidationIssue::UnassignedSheetContent(format!(
                "label:{}",
                label.id.as_str()
            )));
        }
    }

    let mut sheet_port_ids = BTreeSet::new();
    for port in &layout.sheet_ports {
        if !sheet_port_ids.insert(port.id.clone()) {
            issues.push(SchematicValidationIssue::DuplicateSheetPort(
                port.id.clone(),
            ));
        }
        if !sheet_ids.contains(&port.sheet) {
            issues.push(SchematicValidationIssue::UnknownSheetPortSheet(
                port.id.clone(),
            ));
        }
        if !net_ids.contains(&port.net) {
            issues.push(SchematicValidationIssue::UnknownSheetPortNet {
                port: port.id.clone(),
                net: port.net.clone(),
            });
        }
        if port.name.trim().is_empty() {
            issues.push(SchematicValidationIssue::InvalidSheetPortName(
                port.id.clone(),
            ));
        }
    }
    let ports = layout
        .sheet_ports
        .iter()
        .map(|port| (port.id.clone(), port))
        .collect::<BTreeMap<_, _>>();
    let sheets = layout
        .sheets
        .iter()
        .map(|sheet| (sheet.id.clone(), sheet))
        .collect::<BTreeMap<_, _>>();
    let mut link_ids = BTreeSet::new();
    let mut linked_ports = BTreeSet::new();
    for link in &layout.sheet_links {
        if !link_ids.insert(link.id.clone()) {
            issues.push(SchematicValidationIssue::DuplicateSheetLink(
                link.id.clone(),
            ));
        }
        let (Some(parent), Some(child)) =
            (ports.get(&link.parent_port), ports.get(&link.child_port))
        else {
            issues.push(SchematicValidationIssue::UnknownSheetLinkPort(
                link.id.clone(),
            ));
            continue;
        };
        let direct = sheets
            .get(&child.sheet)
            .and_then(|sheet| sheet.parent.as_ref())
            == Some(&parent.sheet);
        if !direct {
            issues.push(SchematicValidationIssue::InvalidSheetLinkRelation(
                link.id.clone(),
            ));
        }
        if parent.net != child.net {
            issues.push(SchematicValidationIssue::SheetLinkNetMismatch(
                link.id.clone(),
            ));
        }
        for port in [&link.parent_port, &link.child_port] {
            if !linked_ports.insert(port.clone()) {
                issues.push(SchematicValidationIssue::DuplicateSheetPortLink(
                    port.clone(),
                ));
            }
        }
    }
    membership
}

fn validate_endpoint(
    layout: &SchematicLayout,
    circuit: &Circuit,
    wire: &SchematicWire,
    endpoint: &SchematicEndpoint,
    sheet_membership: &SheetMembership,
    issues: &mut Vec<SchematicValidationIssue>,
) {
    let endpoint_net = match endpoint {
        SchematicEndpoint::Pin { symbol, pin } => {
            let Some(symbol) = layout
                .symbols
                .iter()
                .find(|candidate| &candidate.id == symbol)
            else {
                issues.push(SchematicValidationIssue::UnknownWirePinEndpoint {
                    wire: wire.id.clone(),
                });
                return;
            };
            if !layout
                .symbol_unit(symbol)
                .is_some_and(|unit| unit.pins.iter().any(|candidate| &candidate.pin == pin))
            {
                issues.push(SchematicValidationIssue::UnknownWirePinEndpoint {
                    wire: wire.id.clone(),
                });
                return;
            }
            validate_endpoint_sheet(
                wire,
                sheet_membership.wires.get(&wire.id),
                sheet_membership.symbols.get(&symbol.id),
                issues,
            );
            circuit
                .instances
                .iter()
                .find(|instance| instance.id == symbol.instance)
                .and_then(|instance| instance.pins.iter().find(|binding| &binding.pin == pin))
                .map(|binding| &binding.net)
        }
        SchematicEndpoint::Port(port) => {
            if !layout.ports.iter().any(|candidate| &candidate.port == port) {
                issues.push(SchematicValidationIssue::UnknownWirePortEndpoint {
                    wire: wire.id.clone(),
                });
                return;
            }
            validate_endpoint_sheet(
                wire,
                sheet_membership.wires.get(&wire.id),
                sheet_membership.ports.get(port),
                issues,
            );
            circuit
                .ports
                .iter()
                .find(|candidate| &candidate.id == port)
                .map(|port| &port.net)
        }
        SchematicEndpoint::SheetPort(port) => {
            let Some(port) = layout
                .sheet_ports
                .iter()
                .find(|candidate| &candidate.id == port)
            else {
                issues.push(SchematicValidationIssue::UnknownWireSheetPortEndpoint {
                    wire: wire.id.clone(),
                });
                return;
            };
            validate_endpoint_sheet(
                wire,
                sheet_membership.wires.get(&wire.id),
                Some(&port.sheet),
                issues,
            );
            Some(&port.net)
        }
        SchematicEndpoint::Junction(_) => Some(&wire.net),
    };
    if endpoint_net.is_some_and(|net| net != &wire.net) {
        issues.push(SchematicValidationIssue::WireEndpointNetMismatch {
            wire: wire.id.clone(),
            net: wire.net.clone(),
        });
    }
}

fn validate_endpoint_sheet(
    wire: &SchematicWire,
    wire_sheet: Option<&SchematicSheetId>,
    endpoint_sheet: Option<&SchematicSheetId>,
    issues: &mut Vec<SchematicValidationIssue>,
) {
    if let (Some(wire_sheet), Some(endpoint_sheet)) = (wire_sheet, endpoint_sheet)
        && wire_sheet != endpoint_sheet
    {
        issues.push(SchematicValidationIssue::WireEndpointSheetMismatch {
            wire: wire.id.clone(),
        });
    }
}

fn positive(value: &Real) -> bool {
    value.structural_facts().sign == Some(RealSign::Positive)
}

/// Finite review-rendering policy.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct SchematicSvgOptions {
    /// Digits written after the decimal point.
    pub decimal_places: usize,
    /// Display margin around computed content bounds.
    pub margin: usize,
}

impl Default for SchematicSvgOptions {
    fn default() -> Self {
        Self {
            decimal_places: 6,
            margin: 20,
        }
    }
}

/// Audit record for one exact schematic scalar projected to SVG.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct SchematicSvgProjection {
    /// Semantic source field.
    pub field: String,
    /// Exact-aware source spelling.
    pub source: String,
    /// Finite token written to SVG.
    pub emitted: String,
}

/// SVG review artifact and its finite projection audit.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct SchematicSvgReport {
    /// Standalone SVG document.
    pub svg: String,
    /// Every exact-to-finite coordinate or dimension projection.
    pub projections: Vec<SchematicSvgProjection>,
}

/// One page of a hierarchical schematic SVG book.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct SchematicSheetSvgReport {
    /// Explicit sheet identity, or `None` for the legacy implicit page.
    pub sheet: Option<SchematicSheetId>,
    /// Human-readable page title.
    pub title: String,
    /// Standalone page SVG and numeric projection audit.
    pub report: SchematicSvgReport,
}

/// Deterministic root-first set of independently reviewable schematic pages.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct SchematicBookSvgReport {
    /// One standalone artifact per explicit or implicit page.
    pub pages: Vec<SchematicSheetSvgReport>,
}

/// Failure to render a structurally faithful schematic review artifact.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum SchematicSvgError {
    /// Circuit or schematic validation failed.
    InvalidLayout,
    /// An exact scalar had no finite display projection.
    NonFinite(String),
    /// No renderable symbol, port, wire, or label was present.
    Empty,
}

impl Display for SchematicSvgError {
    fn fmt(&self, formatter: &mut Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::InvalidLayout => formatter.write_str("cannot render an invalid schematic"),
            Self::NonFinite(field) => write!(formatter, "non-finite schematic field: {field}"),
            Self::Empty => formatter.write_str("schematic has no renderable content"),
        }
    }
}

impl std::error::Error for SchematicSvgError {}

impl SchematicLayout {
    /// Renders every explicit sheet as an independent SVG review page.
    ///
    /// A layout without explicit sheets retains one legacy implicit page.
    pub fn to_svg_book(
        &self,
        circuit: &Circuit,
        options: SchematicSvgOptions,
    ) -> Result<SchematicBookSvgReport, SchematicSvgError> {
        if !self.validate(circuit).is_valid() {
            return Err(SchematicSvgError::InvalidLayout);
        }
        if self.sheets.is_empty() {
            return Ok(SchematicBookSvgReport {
                pages: vec![SchematicSheetSvgReport {
                    sheet: None,
                    title: circuit.id.as_str().to_owned(),
                    report: self.to_svg(circuit, options)?,
                }],
            });
        }
        let ordered = ordered_sheets(&self.sheets);
        let mut pages = Vec::with_capacity(ordered.len());
        for sheet in ordered {
            let symbol_ids = sheet.symbols.iter().cloned().collect::<BTreeSet<_>>();
            let port_ids = sheet.ports.iter().cloned().collect::<BTreeSet<_>>();
            let wire_ids = sheet.wires.iter().cloned().collect::<BTreeSet<_>>();
            let label_ids = sheet.labels.iter().cloned().collect::<BTreeSet<_>>();
            let page_sheet = SchematicSheet {
                parent: None,
                ..sheet.clone()
            };
            let page = SchematicLayout {
                symbol_definitions: self
                    .symbol_definitions
                    .iter()
                    .filter(|definition| {
                        self.symbols.iter().any(|symbol| {
                            symbol_ids.contains(&symbol.id) && symbol.definition == definition.id
                        })
                    })
                    .cloned()
                    .collect(),
                symbols: self
                    .symbols
                    .iter()
                    .filter(|symbol| symbol_ids.contains(&symbol.id))
                    .cloned()
                    .collect(),
                ports: self
                    .ports
                    .iter()
                    .filter(|port| port_ids.contains(&port.port))
                    .cloned()
                    .collect(),
                wires: self
                    .wires
                    .iter()
                    .filter(|wire| wire_ids.contains(&wire.id))
                    .cloned()
                    .collect(),
                labels: self
                    .labels
                    .iter()
                    .filter(|label| label_ids.contains(&label.id))
                    .cloned()
                    .collect(),
                sheets: vec![page_sheet],
                sheet_ports: self
                    .sheet_ports
                    .iter()
                    .filter(|port| port.sheet == sheet.id)
                    .cloned()
                    .collect(),
                sheet_links: Vec::new(),
            };
            pages.push(SchematicSheetSvgReport {
                sheet: Some(sheet.id.clone()),
                title: sheet.title.clone(),
                report: page.to_svg(circuit, options)?,
            });
        }
        Ok(SchematicBookSvgReport { pages })
    }

    /// Renders a simple standalone SVG while retaining every numeric projection.
    pub fn to_svg(
        &self,
        circuit: &Circuit,
        options: SchematicSvgOptions,
    ) -> Result<SchematicSvgReport, SchematicSvgError> {
        if options.decimal_places == 0
            || !circuit.validate().is_valid()
            || !self.validate(circuit).is_valid()
        {
            return Err(SchematicSvgError::InvalidLayout);
        }
        let mut projector = SvgProjector::new(options);
        let mut symbols = Vec::new();
        let mut bounds = Bounds::default();
        for symbol in &self.symbols {
            let unit = self
                .symbol_unit(symbol)
                .expect("validated symbol definition and unit");
            let center =
                projector.point(&format!("symbol.{}", symbol.id.as_str()), &symbol.position)?;
            let mut width = projector.value(
                &format!("symbol.{}.width", symbol.id.as_str()),
                &unit.body_width,
            )?;
            let mut height = projector.value(
                &format!("symbol.{}.height", symbol.id.as_str()),
                &unit.body_height,
            )?;
            if symbol.quarter_turns.rem_euclid(2) == 1 {
                std::mem::swap(&mut width, &mut height);
            }
            bounds.include(center.0 - width / 2.0, center.1 - height / 2.0);
            bounds.include(center.0 + width / 2.0, center.1 + height / 2.0);
            let pins = unit
                .pins
                .iter()
                .map(|pin| {
                    let point = endpoint_pin_point(symbol, pin);
                    projector.point(
                        &format!("symbol.{}.pin.{}", symbol.id.as_str(), pin.pin.as_str()),
                        &point,
                    )
                })
                .collect::<Result<Vec<_>, _>>()?;
            for point in &pins {
                bounds.include(point.0, point.1);
            }
            let graphics = unit
                .graphics
                .iter()
                .enumerate()
                .map(|(index, graphic)| {
                    project_symbol_graphic(&mut projector, symbol, graphic, index)
                })
                .collect::<Result<Vec<_>, _>>()?;
            for graphic in &graphics {
                graphic.include_bounds(&mut bounds);
            }
            symbols.push((symbol, unit, center, width, height, pins, graphics));
        }
        let mut ports = Vec::new();
        for port in &self.ports {
            let point = projector.point(&format!("port.{}", port.port.as_str()), &port.position)?;
            bounds.include(point.0, point.1);
            ports.push((port, point));
        }
        let mut sheet_ports = Vec::new();
        for port in &self.sheet_ports {
            let point =
                projector.point(&format!("sheet_port.{}", port.id.as_str()), &port.position)?;
            bounds.include(point.0 - 3.0, point.1 - 3.0);
            bounds.include(point.0 + 3.0, point.1 + 3.0);
            sheet_ports.push((port, point));
        }
        let mut wires = Vec::new();
        for wire in &self.wires {
            let mut points = vec![endpoint_point(self, &wire.from).expect("validated endpoint")];
            points.extend(wire.waypoints.clone());
            points.push(endpoint_point(self, &wire.to).expect("validated endpoint"));
            let points = points
                .iter()
                .enumerate()
                .map(|(index, point)| {
                    projector.point(&format!("wire.{}.point[{index}]", wire.id.as_str()), point)
                })
                .collect::<Result<Vec<_>, _>>()?;
            for point in &points {
                bounds.include(point.0, point.1);
            }
            wires.push((wire, points));
        }
        let mut labels = Vec::new();
        for label in &self.labels {
            let point =
                projector.point(&format!("label.{}", label.id.as_str()), &label.position)?;
            bounds.include(point.0, point.1);
            labels.push((label, point));
        }
        if !bounds.valid {
            return Err(SchematicSvgError::Empty);
        }
        let margin = options.margin as f64;
        let min_x = bounds.min_x - margin;
        let min_y = bounds.min_y - margin;
        let width = (bounds.max_x - bounds.min_x) + 2.0 * margin;
        let height = (bounds.max_y - bounds.min_y) + 2.0 * margin;
        let mut svg = format!(
            "<svg xmlns=\"http://www.w3.org/2000/svg\" viewBox=\"{min_x} {min_y} {width} {height}\">\n"
        );
        svg.push_str("<g fill=\"none\" stroke=\"#222\" stroke-width=\"1\">\n");
        for (_, points) in &wires {
            let points = points
                .iter()
                .map(|(x, y)| format!("{x},{y}"))
                .collect::<Vec<_>>()
                .join(" ");
            writeln!(svg, "<polyline points=\"{points}\" stroke=\"#087f23\"/>")
                .expect("writing to String cannot fail");
        }
        for (symbol, unit, (x, y), _, _, pins, graphics) in &symbols {
            for graphic in graphics {
                graphic.write_svg(&mut svg);
            }
            for (pin, (px, py)) in unit.pins.iter().zip(pins) {
                writeln!(
                    svg,
                    "<circle cx=\"{px}\" cy=\"{py}\" r=\"1.5\" fill=\"#fff\"/>"
                )
                .expect("writing to String cannot fail");
                writeln!(
                    svg,
                    "<text x=\"{}\" y=\"{}\" fill=\"#222\" stroke=\"none\" font-size=\"7\">{}</text>",
                    px + 2.0,
                    py - 2.0,
                    xml(pin.pin.as_str())
                )
                .expect("writing to String cannot fail");
            }
            writeln!(
                svg,
                "<text x=\"{x}\" y=\"{y}\" text-anchor=\"middle\" fill=\"#111\" stroke=\"none\" font-size=\"8\">{}</text>",
                xml(symbol.instance.as_str())
            )
            .expect("writing to String cannot fail");
        }
        for (port, (x, y)) in &ports {
            writeln!(svg, "<circle cx=\"{x}\" cy=\"{y}\" r=\"2\" fill=\"#fff\"/>")
                .expect("writing to String cannot fail");
            writeln!(
                svg,
                "<text x=\"{}\" y=\"{}\" fill=\"#111\" stroke=\"none\" font-size=\"8\">{}</text>",
                x + 3.0,
                y - 3.0,
                xml(port.port.as_str())
            )
            .expect("writing to String cannot fail");
        }
        for (port, (x, y)) in &sheet_ports {
            writeln!(
                svg,
                "<polygon data-sheet=\"{}\" data-sheet-port=\"{}\" points=\"{},{} {},{} {},{} {},{}\" fill=\"#fff\"/>",
                xml(port.sheet.as_str()),
                xml(port.id.as_str()),
                x - 3.0,
                y,
                x,
                y - 3.0,
                x + 3.0,
                y,
                x,
                y + 3.0
            )
            .expect("writing to String cannot fail");
            writeln!(
                svg,
                "<text x=\"{}\" y=\"{}\" fill=\"#7b2cbf\" stroke=\"none\" font-size=\"8\">{}</text>",
                x + 4.0,
                y - 3.0,
                xml(&port.name)
            )
            .expect("writing to String cannot fail");
        }
        for (label, (x, y)) in &labels {
            writeln!(
                svg,
                "<text x=\"{x}\" y=\"{y}\" fill=\"#0645ad\" stroke=\"none\" font-size=\"8\">{}</text>",
                xml(&label.text)
            )
            .expect("writing to String cannot fail");
        }
        svg.push_str("</g>\n</svg>\n");
        Ok(SchematicSvgReport {
            svg,
            projections: projector.projections,
        })
    }
}

fn ordered_sheets(sheets: &[SchematicSheet]) -> Vec<&SchematicSheet> {
    let mut children = BTreeMap::<Option<SchematicSheetId>, Vec<&SchematicSheet>>::new();
    for sheet in sheets {
        children
            .entry(sheet.parent.clone())
            .or_default()
            .push(sheet);
    }
    for siblings in children.values_mut() {
        siblings.sort_by(|left, right| left.id.cmp(&right.id));
    }
    let mut ordered = Vec::with_capacity(sheets.len());
    let mut stack = children.remove(&None).unwrap_or_default();
    stack.reverse();
    while let Some(sheet) = stack.pop() {
        ordered.push(sheet);
        if let Some(mut nested) = children.remove(&Some(sheet.id.clone())) {
            nested.reverse();
            stack.extend(nested);
        }
    }
    ordered
}

fn endpoint_point(
    layout: &SchematicLayout,
    endpoint: &SchematicEndpoint,
) -> Option<SchematicPoint> {
    match endpoint {
        SchematicEndpoint::Pin { symbol, pin } => {
            let symbol = layout
                .symbols
                .iter()
                .find(|candidate| &candidate.id == symbol)?;
            let pin = layout
                .symbol_unit(symbol)?
                .pins
                .iter()
                .find(|candidate| &candidate.pin == pin)?;
            Some(endpoint_pin_point(symbol, pin))
        }
        SchematicEndpoint::Port(port) => layout
            .ports
            .iter()
            .find(|candidate| &candidate.port == port)
            .map(|placement| placement.position.clone()),
        SchematicEndpoint::SheetPort(port) => layout
            .sheet_ports
            .iter()
            .find(|candidate| &candidate.id == port)
            .map(|port| port.position.clone()),
        SchematicEndpoint::Junction(point) => Some(point.clone()),
    }
}

fn endpoint_pin_point(symbol: &SchematicSymbol, pin: &SchematicPinPlacement) -> SchematicPoint {
    let (x, y) = match symbol.quarter_turns.rem_euclid(4) {
        0 => (pin.position.x.clone(), pin.position.y.clone()),
        1 => (-pin.position.y.clone(), pin.position.x.clone()),
        2 => (-pin.position.x.clone(), -pin.position.y.clone()),
        _ => (pin.position.y.clone(), -pin.position.x.clone()),
    };
    SchematicPoint::new(x + symbol.position.x.clone(), y + symbol.position.y.clone())
}

enum ProjectedSymbolGraphic {
    Polyline {
        points: Vec<(f64, f64)>,
        closed: bool,
        stroke_width: f64,
        fill: SchematicGraphicFill,
    },
    Circle {
        center: (f64, f64),
        radius: f64,
        stroke_width: f64,
        fill: SchematicGraphicFill,
    },
    Arc {
        start: (f64, f64),
        mid: (f64, f64),
        end: (f64, f64),
        stroke_width: f64,
    },
    Text {
        position: (f64, f64),
        text: String,
        size: f64,
        quarter_turns: i8,
    },
}

impl ProjectedSymbolGraphic {
    fn include_bounds(&self, bounds: &mut Bounds) {
        match self {
            Self::Polyline { points, .. } => {
                for &(x, y) in points {
                    bounds.include(x, y);
                }
            }
            Self::Circle { center, radius, .. } => {
                bounds.include(center.0 - radius, center.1 - radius);
                bounds.include(center.0 + radius, center.1 + radius);
            }
            Self::Arc {
                start, mid, end, ..
            } => {
                for &(x, y) in [start, mid, end] {
                    bounds.include(x, y);
                }
            }
            Self::Text { position, size, .. } => {
                bounds.include(position.0 - size, position.1 - size);
                bounds.include(position.0 + size, position.1 + size);
            }
        }
    }

    fn write_svg(&self, svg: &mut String) {
        match self {
            Self::Polyline {
                points,
                closed,
                stroke_width,
                fill,
            } => {
                let tag = if *closed { "polygon" } else { "polyline" };
                let points = points
                    .iter()
                    .map(|(x, y)| format!("{x},{y}"))
                    .collect::<Vec<_>>()
                    .join(" ");
                writeln!(
                    svg,
                    "<{tag} points=\"{points}\" stroke-width=\"{stroke_width}\" fill=\"{}\"/>",
                    svg_fill(*fill)
                )
                .expect("writing to String cannot fail");
            }
            Self::Circle {
                center,
                radius,
                stroke_width,
                fill,
            } => {
                writeln!(
                    svg,
                    "<circle cx=\"{}\" cy=\"{}\" r=\"{radius}\" stroke-width=\"{stroke_width}\" fill=\"{}\"/>",
                    center.0,
                    center.1,
                    svg_fill(*fill)
                )
                .expect("writing to String cannot fail");
            }
            Self::Arc {
                start,
                mid,
                end,
                stroke_width,
            } => {
                let radius = circle_radius(*start, *mid, *end);
                let cross =
                    (mid.0 - start.0) * (end.1 - mid.1) - (mid.1 - start.1) * (end.0 - mid.0);
                let sweep = usize::from(cross > 0.0);
                writeln!(
                    svg,
                    "<path d=\"M {} {} A {radius} {radius} 0 0 {sweep} {} {} A {radius} {radius} 0 0 {sweep} {} {}\" stroke-width=\"{stroke_width}\"/>",
                    start.0, start.1, mid.0, mid.1, end.0, end.1
                )
                .expect("writing to String cannot fail");
            }
            Self::Text {
                position,
                text,
                size,
                quarter_turns,
            } => {
                let angle = quarter_turns.rem_euclid(4) * 90;
                writeln!(
                    svg,
                    "<text x=\"{}\" y=\"{}\" font-size=\"{size}\" transform=\"rotate({angle} {} {})\" fill=\"#222\" stroke=\"none\">{}</text>",
                    position.0,
                    position.1,
                    position.0,
                    position.1,
                    xml(text)
                )
                .expect("writing to String cannot fail");
            }
        }
    }
}

fn project_symbol_graphic(
    projector: &mut SvgProjector,
    symbol: &SchematicSymbol,
    graphic: &SchematicGraphic,
    index: usize,
) -> Result<ProjectedSymbolGraphic, SchematicSvgError> {
    let field = format!("symbol.{}.graphics[{index}]", symbol.id.as_str());
    let point = |projector: &mut SvgProjector,
                 name: &str,
                 value: &SchematicPoint|
     -> Result<(f64, f64), SchematicSvgError> {
        projector.point(
            &format!("{field}.{name}"),
            &symbol_local_point(symbol, value),
        )
    };
    Ok(match graphic {
        SchematicGraphic::Line {
            from,
            to,
            stroke_width,
        } => ProjectedSymbolGraphic::Polyline {
            points: vec![point(projector, "from", from)?, point(projector, "to", to)?],
            closed: false,
            stroke_width: projector.value(&format!("{field}.stroke_width"), stroke_width)?,
            fill: SchematicGraphicFill::None,
        },
        SchematicGraphic::Rectangle {
            start,
            end,
            stroke_width,
            fill,
        } => {
            let corners = [
                start.clone(),
                SchematicPoint::new(end.x.clone(), start.y.clone()),
                end.clone(),
                SchematicPoint::new(start.x.clone(), end.y.clone()),
            ];
            ProjectedSymbolGraphic::Polyline {
                points: corners
                    .iter()
                    .enumerate()
                    .map(|(corner, value)| point(projector, &format!("corners[{corner}]"), value))
                    .collect::<Result<Vec<_>, _>>()?,
                closed: true,
                stroke_width: projector.value(&format!("{field}.stroke_width"), stroke_width)?,
                fill: *fill,
            }
        }
        SchematicGraphic::Circle {
            center,
            radius,
            stroke_width,
            fill,
        } => ProjectedSymbolGraphic::Circle {
            center: point(projector, "center", center)?,
            radius: projector.value(&format!("{field}.radius"), radius)?,
            stroke_width: projector.value(&format!("{field}.stroke_width"), stroke_width)?,
            fill: *fill,
        },
        SchematicGraphic::Arc {
            start,
            mid,
            end,
            stroke_width,
        } => ProjectedSymbolGraphic::Arc {
            start: point(projector, "start", start)?,
            mid: point(projector, "mid", mid)?,
            end: point(projector, "end", end)?,
            stroke_width: projector.value(&format!("{field}.stroke_width"), stroke_width)?,
        },
        SchematicGraphic::Polyline {
            points,
            closed,
            stroke_width,
            fill,
        } => ProjectedSymbolGraphic::Polyline {
            points: points
                .iter()
                .enumerate()
                .map(|(vertex, value)| point(projector, &format!("points[{vertex}]"), value))
                .collect::<Result<Vec<_>, _>>()?,
            closed: *closed,
            stroke_width: projector.value(&format!("{field}.stroke_width"), stroke_width)?,
            fill: *fill,
        },
        SchematicGraphic::Text {
            position,
            text,
            size,
            quarter_turns,
        } => ProjectedSymbolGraphic::Text {
            position: point(projector, "position", position)?,
            text: text.clone(),
            size: projector.value(&format!("{field}.size"), size)?,
            quarter_turns: symbol.quarter_turns + *quarter_turns,
        },
    })
}

fn symbol_local_point(symbol: &SchematicSymbol, point: &SchematicPoint) -> SchematicPoint {
    let (x, y) = match symbol.quarter_turns.rem_euclid(4) {
        0 => (point.x.clone(), point.y.clone()),
        1 => (-point.y.clone(), point.x.clone()),
        2 => (-point.x.clone(), -point.y.clone()),
        _ => (point.y.clone(), -point.x.clone()),
    };
    SchematicPoint::new(x + symbol.position.x.clone(), y + symbol.position.y.clone())
}

fn svg_fill(fill: SchematicGraphicFill) -> &'static str {
    match fill {
        SchematicGraphicFill::None => "none",
        SchematicGraphicFill::Background => "#fff",
        SchematicGraphicFill::Foreground => "#222",
    }
}

fn circle_radius(start: (f64, f64), mid: (f64, f64), end: (f64, f64)) -> f64 {
    let determinant =
        2.0 * (start.0 * (mid.1 - end.1) + mid.0 * (end.1 - start.1) + end.0 * (start.1 - mid.1));
    let start_square = start.0 * start.0 + start.1 * start.1;
    let mid_square = mid.0 * mid.0 + mid.1 * mid.1;
    let end_square = end.0 * end.0 + end.1 * end.1;
    let center_x = (start_square * (mid.1 - end.1)
        + mid_square * (end.1 - start.1)
        + end_square * (start.1 - mid.1))
        / determinant;
    let center_y = (start_square * (end.0 - mid.0)
        + mid_square * (start.0 - end.0)
        + end_square * (mid.0 - start.0))
        / determinant;
    ((start.0 - center_x).powi(2) + (start.1 - center_y).powi(2)).sqrt()
}

struct SvgProjector {
    options: SchematicSvgOptions,
    projections: Vec<SchematicSvgProjection>,
}

impl SvgProjector {
    fn new(options: SchematicSvgOptions) -> Self {
        Self {
            options,
            projections: Vec::new(),
        }
    }

    fn point(
        &mut self,
        field: &str,
        point: &SchematicPoint,
    ) -> Result<(f64, f64), SchematicSvgError> {
        Ok((
            self.value(&format!("{field}.x"), &point.x)?,
            self.value(&format!("{field}.y"), &point.y)?,
        ))
    }

    fn value(&mut self, field: &str, value: &Real) -> Result<f64, SchematicSvgError> {
        let Some(finite) = value.to_f64_lossy().filter(|value| value.is_finite()) else {
            return Err(SchematicSvgError::NonFinite(field.to_owned()));
        };
        let emitted = format!("{:.*}", self.options.decimal_places, finite);
        self.projections.push(SchematicSvgProjection {
            field: field.to_owned(),
            source: value.to_string(),
            emitted,
        });
        Ok(finite)
    }
}

#[derive(Default)]
struct Bounds {
    valid: bool,
    min_x: f64,
    min_y: f64,
    max_x: f64,
    max_y: f64,
}

impl Bounds {
    fn include(&mut self, x: f64, y: f64) {
        if self.valid {
            self.min_x = self.min_x.min(x);
            self.min_y = self.min_y.min(y);
            self.max_x = self.max_x.max(x);
            self.max_y = self.max_y.max(y);
        } else {
            self.valid = true;
            self.min_x = x;
            self.min_y = y;
            self.max_x = x;
            self.max_y = y;
        }
    }
}

fn xml(value: &str) -> String {
    value
        .replace('&', "&amp;")
        .replace('<', "&lt;")
        .replace('>', "&gt;")
        .replace('"', "&quot;")
        .replace('\'', "&apos;")
}
