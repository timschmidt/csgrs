//! Revision-checked, atomic editor deltas over retained semantic intent.
//!
//! Edit events span circuit and PCB identities, never materialized csgrs
//! profiles. A batch is applied to a clone, structurally validated, checked
//! against retained placement constraints, and committed only as one revision.

use std::fmt::{Display, Formatter};

use hyperlattice::Point2;
use hyperreal::Real;

use crate::{
    AdapterKind, BoardId, BoardOutline, BoardSide, Bus, BusId, BusSlice, BusSliceId, BusSliceOrder,
    CircuitId, CircuitInstance, CircuitInstanceId, CircuitModuleParameter,
    CircuitModuleParameterOverride, CircuitModuleParameterTarget, CircuitPort, ComponentId,
    CopperZone, DesignEditId, DeviceModel, DeviceModelId, KeepoutId, LandPattern, LandPatternId,
    LinearStamp, Net, NetId, PcbDesignRules, PcbKeepout, PcbLayout, PcbPlacement, PcbRoute,
    PcbRouteSegment, PcbStackup, PcbVia, PinBinding, PlacementConstraint, PlacementConstraintId,
    PortDirection, PortId, RailIntent, RailKind, RouteId, SchematicLabel, SchematicLabelId,
    SchematicLayout, SchematicPortPlacement, SchematicSheet, SchematicSheetId, SchematicSheetLink,
    SchematicSheetLinkId, SchematicSheetPort, SchematicSheetPortId, SchematicSymbol,
    SchematicSymbolDefinition, SchematicSymbolDefinitionId, SchematicSymbolId, SchematicWire,
    SchematicWireId, SemanticDocument, SemanticInterchangeError, SourceStimulus, SourceWaveform,
    SubcircuitInstance, SubcircuitInstanceId, SubcircuitPortBinding, TransientPolicy, ViaId,
    ZoneId,
};

/// Stable schema family for serialized semantic editor histories.
pub const DESIGN_HISTORY_SCHEMA: &str = "org.hypercircuit.design-history";

/// Current serialized editor-history schema revision.
pub const DESIGN_HISTORY_VERSION: u32 = 15;

/// Oldest editor-history revision upgraded by this crate.
pub const DESIGN_HISTORY_MIN_MIGRATABLE_VERSION: u32 = 1;

/// Monotonic optimistic-concurrency token for one semantic design.
#[derive(
    Clone,
    Copy,
    Debug,
    Default,
    Eq,
    Ord,
    PartialEq,
    PartialOrd,
    serde::Deserialize,
    serde::Serialize,
)]
pub struct DesignRevision(u64);

impl DesignRevision {
    /// Constructs an explicit imported or persisted revision.
    pub const fn new(value: u64) -> Self {
        Self(value)
    }

    /// Returns the numeric revision.
    pub const fn value(self) -> u64 {
        self.0
    }

    fn next(self) -> Option<Self> {
        self.0.checked_add(1).map(Self)
    }
}

/// Stable semantic object addressed by an editor delta.
#[derive(Clone, Debug, Eq, PartialEq, serde::Deserialize, serde::Serialize)]
pub enum EditTarget {
    /// One logical circuit net.
    Net(NetId),
    /// One ordered logical circuit bus.
    Bus(BusId),
    /// One named view into an ordered circuit bus.
    BusSlice(BusSliceId),
    /// One circuit boundary port.
    CircuitPort(PortId),
    /// One rail-intent record keyed by its retained net.
    Rail(NetId),
    /// One declared reusable-circuit interface parameter.
    ModuleParameter(String),
    /// One component-addressed independent-source stimulus.
    SourceStimulus(ComponentId),
    /// One reusable child-circuit instance.
    Subcircuit(SubcircuitInstanceId),
    /// The complete ordered manual linear-stamp vector.
    LinearStamps,
    /// Circuit transient and adapter policy.
    CircuitPolicy,
    /// The optional retained schematic layout.
    Schematic,
    /// One placed schematic symbol unit.
    SchematicSymbol(SchematicSymbolId),
    /// One reusable multipart schematic symbol definition.
    SchematicSymbolLibrary(SchematicSymbolDefinitionId),
    /// One placed circuit-boundary port in the schematic.
    SchematicPortPlacement(PortId),
    /// One retained schematic wire.
    SchematicWire(SchematicWireId),
    /// One retained schematic net label.
    SchematicLabel(SchematicLabelId),
    /// One retained schematic sheet.
    SchematicSheet(SchematicSheetId),
    /// One retained schematic sheet-boundary port.
    SchematicSheetPort(SchematicSheetPortId),
    /// One retained parent/child schematic sheet link.
    SchematicSheetLink(SchematicSheetLinkId),
    /// One reusable electrical device model.
    DeviceModel(DeviceModelId),
    /// One logical circuit instance.
    CircuitInstance(CircuitInstanceId),
    /// The optional retained PCB container.
    Pcb(BoardId),
    /// The retained PCB itself, keyed by stable board identity.
    Board(BoardId),
    /// One retained land pattern in the board-local footprint library.
    LandPattern(LandPatternId),
    /// One retained physical-layout keepout.
    Keepout(KeepoutId),
    /// One component placement, keyed by logical circuit instance.
    Placement(CircuitInstanceId),
    /// One retained placement constraint.
    PlacementConstraint(PlacementConstraintId),
    /// One retained copper route.
    Route(RouteId),
    /// One retained via.
    Via(ViaId),
    /// One retained copper zone.
    Zone(ZoneId),
}

/// Independently mergeable semantic property addressed by an editor delta.
///
/// This is deliberately finer grained than [`EditTarget`]: a route-width
/// change commutes with a route-centerline change even though both affect the
/// same retained route identity.
#[derive(Clone, Debug, Eq, PartialEq, serde::Deserialize, serde::Serialize)]
pub enum EditAddress {
    /// Presence and complete authored value of one logical circuit net.
    NetPresence(NetId),
    /// Reference-ground designation of one logical circuit net.
    NetGround(NetId),
    /// Presence and complete authored value of one ordered bus.
    BusPresence(BusId),
    /// Ordered member nets of one retained bus.
    BusNets(BusId),
    /// Presence and complete authored value of one named bus slice.
    BusSlicePresence(BusSliceId),
    /// Source bus, range, and ordering of one retained bus slice.
    BusSliceDefinition(BusSliceId),
    /// Presence and complete authored value of one circuit boundary port.
    CircuitPortPresence(PortId),
    /// Net, direction, and optionality of one circuit boundary port.
    CircuitPortDefinition(PortId),
    /// Presence and complete authored value of one rail-intent record.
    RailPresence(NetId),
    /// Voltage, current, and role of one retained rail-intent record.
    RailDefinition(NetId),
    /// Presence and complete authored value of one module parameter.
    ModuleParameterPresence(String),
    /// Default, unit, provenance, and targets of one module parameter.
    ModuleParameterDefinition(String),
    /// Presence and complete authored value of one source stimulus.
    SourceStimulusPresence(ComponentId),
    /// Exact waveform of one retained source stimulus.
    SourceStimulusWaveform(ComponentId),
    /// Presence and complete authored value of one child-circuit instance.
    SubcircuitPresence(SubcircuitInstanceId),
    /// Child definition, bindings, and overrides of one subcircuit instance.
    SubcircuitDefinition(SubcircuitInstanceId),
    /// Complete ordered manual linear-stamp vector.
    LinearStamps,
    /// Circuit transient and adapter policy.
    CircuitPolicy,
    /// Presence and complete authored value of the optional schematic layout.
    SchematicPresence,
    /// Presence and complete authored value of one schematic symbol unit.
    SchematicSymbolPresence(SchematicSymbolId),
    /// Complete authored definition of one retained schematic symbol unit.
    SchematicSymbolDefinition(SchematicSymbolId),
    /// Presence and complete authored value of one reusable schematic symbol definition.
    SchematicSymbolLibraryPresence(SchematicSymbolDefinitionId),
    /// Complete authored value of one reusable schematic symbol definition.
    SchematicSymbolLibraryDefinition(SchematicSymbolDefinitionId),
    /// Presence and complete authored value of one schematic circuit-port placement.
    SchematicPortPlacementPresence(PortId),
    /// Exact point of one retained schematic circuit-port placement.
    SchematicPortPlacementDefinition(PortId),
    /// Presence and complete authored value of one schematic wire.
    SchematicWirePresence(SchematicWireId),
    /// Net, endpoints, and exact waypoints of one retained schematic wire.
    SchematicWireDefinition(SchematicWireId),
    /// Presence and complete authored value of one schematic net label.
    SchematicLabelPresence(SchematicLabelId),
    /// Net, text, and exact position of one retained schematic label.
    SchematicLabelDefinition(SchematicLabelId),
    /// Presence and complete authored value of one schematic sheet.
    SchematicSheetPresence(SchematicSheetId),
    /// Metadata and ordered content membership of one retained schematic sheet.
    SchematicSheetDefinition(SchematicSheetId),
    /// Presence and complete authored value of one schematic sheet-boundary port.
    SchematicSheetPortPresence(SchematicSheetPortId),
    /// Sheet, net, name, and exact position of one retained sheet-boundary port.
    SchematicSheetPortDefinition(SchematicSheetPortId),
    /// Presence and complete authored value of one schematic sheet link.
    SchematicSheetLinkPresence(SchematicSheetLinkId),
    /// Parent and child ports of one retained schematic sheet link.
    SchematicSheetLinkDefinition(SchematicSheetLinkId),
    /// Presence and complete authored value of one reusable device model.
    DeviceModelPresence(DeviceModelId),
    /// Complete authored definition of one reusable device model.
    DeviceModelDefinition(DeviceModelId),
    /// Presence and complete authored value of one logical circuit instance.
    CircuitInstancePresence(CircuitInstanceId),
    /// Complete authored definition of one logical circuit instance.
    CircuitInstanceDefinition(CircuitInstanceId),
    /// Device model assigned to one logical circuit instance.
    CircuitInstanceModel(CircuitInstanceId),
    /// Complete ordered pin-to-net bindings of one logical circuit instance.
    CircuitInstancePins(CircuitInstanceId),
    /// Presence and complete authored value of the optional PCB container.
    PcbPresence(BoardId),
    /// Exact substrate boundary and cutouts of one retained PCB.
    BoardOutline(BoardId),
    /// Ordered physical stackup of one retained PCB.
    BoardStackup(BoardId),
    /// Complete authored routing and verification policy of one retained PCB.
    BoardRules(BoardId),
    /// Presence and complete authored value of one retained land pattern.
    LandPatternPresence(LandPatternId),
    /// Complete authored definition of one retained land pattern.
    LandPatternDefinition(LandPatternId),
    /// Presence and complete authored value of one retained keepout.
    KeepoutPresence(KeepoutId),
    /// Boundary and feature scope of one retained keepout.
    KeepoutDefinition(KeepoutId),
    /// Complete authored transform of one placement.
    PlacementTransform(CircuitInstanceId),
    /// Land pattern assigned to one placement.
    PlacementLandPattern(CircuitInstanceId),
    /// Presence and complete authored value of one placement.
    PlacementPresence(CircuitInstanceId),
    /// Presence and complete authored value of one placement constraint.
    PlacementConstraintPresence(PlacementConstraintId),
    /// Complete authored predicate of one retained placement constraint.
    PlacementConstraintDefinition(PlacementConstraintId),
    /// Finished width of one retained route.
    RouteWidth(RouteId),
    /// Ordered centerline of one retained route.
    RouteSegments(RouteId),
    /// Logical net assigned to one retained route.
    RouteNet(RouteId),
    /// Presence and complete authored value of one retained route.
    RoutePresence(RouteId),
    /// Complete authored definition of one retained route.
    RouteDefinition(RouteId),
    /// Board-space center of one via.
    ViaCenter(ViaId),
    /// Logical net assigned to one retained via.
    ViaNet(ViaId),
    /// Presence and complete authored value of one retained via.
    ViaPresence(ViaId),
    /// Complete authored definition of one retained via.
    ViaDefinition(ViaId),
    /// Closed boundary of one copper zone.
    ZoneBoundary(ZoneId),
    /// Logical net assigned to one retained copper zone.
    ZoneNet(ZoneId),
    /// Presence and complete authored value of one retained copper zone.
    ZonePresence(ZoneId),
    /// Complete authored definition of one retained copper zone.
    ZoneDefinition(ZoneId),
}

impl EditAddress {
    /// Returns the stable object containing this independently mergeable field.
    pub fn target(&self) -> EditTarget {
        match self {
            Self::NetPresence(net) | Self::NetGround(net) => EditTarget::Net(net.clone()),
            Self::BusPresence(bus) | Self::BusNets(bus) => EditTarget::Bus(bus.clone()),
            Self::BusSlicePresence(slice) | Self::BusSliceDefinition(slice) => {
                EditTarget::BusSlice(slice.clone())
            }
            Self::CircuitPortPresence(port) | Self::CircuitPortDefinition(port) => {
                EditTarget::CircuitPort(port.clone())
            }
            Self::RailPresence(net) | Self::RailDefinition(net) => EditTarget::Rail(net.clone()),
            Self::ModuleParameterPresence(parameter)
            | Self::ModuleParameterDefinition(parameter) => {
                EditTarget::ModuleParameter(parameter.clone())
            }
            Self::SourceStimulusPresence(component) | Self::SourceStimulusWaveform(component) => {
                EditTarget::SourceStimulus(component.clone())
            }
            Self::SubcircuitPresence(instance) | Self::SubcircuitDefinition(instance) => {
                EditTarget::Subcircuit(instance.clone())
            }
            Self::LinearStamps => EditTarget::LinearStamps,
            Self::CircuitPolicy => EditTarget::CircuitPolicy,
            Self::SchematicPresence => EditTarget::Schematic,
            Self::SchematicSymbolPresence(symbol) | Self::SchematicSymbolDefinition(symbol) => {
                EditTarget::SchematicSymbol(symbol.clone())
            }
            Self::SchematicSymbolLibraryPresence(definition)
            | Self::SchematicSymbolLibraryDefinition(definition) => {
                EditTarget::SchematicSymbolLibrary(definition.clone())
            }
            Self::SchematicPortPlacementPresence(port)
            | Self::SchematicPortPlacementDefinition(port) => {
                EditTarget::SchematicPortPlacement(port.clone())
            }
            Self::SchematicWirePresence(wire) | Self::SchematicWireDefinition(wire) => {
                EditTarget::SchematicWire(wire.clone())
            }
            Self::SchematicLabelPresence(label) | Self::SchematicLabelDefinition(label) => {
                EditTarget::SchematicLabel(label.clone())
            }
            Self::SchematicSheetPresence(sheet) | Self::SchematicSheetDefinition(sheet) => {
                EditTarget::SchematicSheet(sheet.clone())
            }
            Self::SchematicSheetPortPresence(port) | Self::SchematicSheetPortDefinition(port) => {
                EditTarget::SchematicSheetPort(port.clone())
            }
            Self::SchematicSheetLinkPresence(link) | Self::SchematicSheetLinkDefinition(link) => {
                EditTarget::SchematicSheetLink(link.clone())
            }
            Self::DeviceModelPresence(model) | Self::DeviceModelDefinition(model) => {
                EditTarget::DeviceModel(model.clone())
            }
            Self::CircuitInstancePresence(instance)
            | Self::CircuitInstanceDefinition(instance)
            | Self::CircuitInstanceModel(instance)
            | Self::CircuitInstancePins(instance) => EditTarget::CircuitInstance(instance.clone()),
            Self::PcbPresence(board) => EditTarget::Pcb(board.clone()),
            Self::BoardOutline(board) | Self::BoardStackup(board) | Self::BoardRules(board) => {
                EditTarget::Board(board.clone())
            }
            Self::LandPatternPresence(pattern) | Self::LandPatternDefinition(pattern) => {
                EditTarget::LandPattern(pattern.clone())
            }
            Self::KeepoutPresence(keepout) | Self::KeepoutDefinition(keepout) => {
                EditTarget::Keepout(keepout.clone())
            }
            Self::PlacementTransform(instance)
            | Self::PlacementLandPattern(instance)
            | Self::PlacementPresence(instance) => EditTarget::Placement(instance.clone()),
            Self::PlacementConstraintPresence(constraint)
            | Self::PlacementConstraintDefinition(constraint) => {
                EditTarget::PlacementConstraint(constraint.clone())
            }
            Self::RouteWidth(route)
            | Self::RouteSegments(route)
            | Self::RouteNet(route)
            | Self::RouteDefinition(route) => EditTarget::Route(route.clone()),
            Self::RoutePresence(route) => EditTarget::Route(route.clone()),
            Self::ViaCenter(via)
            | Self::ViaNet(via)
            | Self::ViaPresence(via)
            | Self::ViaDefinition(via) => EditTarget::Via(via.clone()),
            Self::ZoneBoundary(zone)
            | Self::ZoneNet(zone)
            | Self::ZonePresence(zone)
            | Self::ZoneDefinition(zone) => EditTarget::Zone(zone.clone()),
        }
    }

    fn conflicts_with(&self, other: &Self) -> bool {
        if self == other {
            return true;
        }
        match (self, other) {
            (Self::NetPresence(left), right) | (right, Self::NetPresence(left)) => {
                matches!(right.target(), EditTarget::Net(right) if left == &right)
            }
            (Self::BusPresence(left), right) | (right, Self::BusPresence(left)) => {
                matches!(right.target(), EditTarget::Bus(right) if left == &right)
            }
            (Self::BusSlicePresence(left), right) | (right, Self::BusSlicePresence(left)) => {
                matches!(right.target(), EditTarget::BusSlice(right) if left == &right)
            }
            (Self::CircuitPortPresence(left), right) | (right, Self::CircuitPortPresence(left)) => {
                matches!(right.target(), EditTarget::CircuitPort(right) if left == &right)
            }
            (Self::RailPresence(left), right) | (right, Self::RailPresence(left)) => {
                matches!(right.target(), EditTarget::Rail(right) if left == &right)
            }
            (Self::ModuleParameterPresence(left), right)
            | (right, Self::ModuleParameterPresence(left)) => {
                matches!(right.target(), EditTarget::ModuleParameter(right) if left == &right)
            }
            (Self::SourceStimulusPresence(left), right)
            | (right, Self::SourceStimulusPresence(left)) => {
                matches!(right.target(), EditTarget::SourceStimulus(right) if left == &right)
            }
            (Self::SubcircuitPresence(left), right) | (right, Self::SubcircuitPresence(left)) => {
                matches!(right.target(), EditTarget::Subcircuit(right) if left == &right)
            }
            (Self::SchematicPresence, right) | (right, Self::SchematicPresence) => matches!(
                right.target(),
                EditTarget::Schematic
                    | EditTarget::SchematicSymbol(_)
                    | EditTarget::SchematicSymbolLibrary(_)
                    | EditTarget::SchematicPortPlacement(_)
                    | EditTarget::SchematicWire(_)
                    | EditTarget::SchematicLabel(_)
                    | EditTarget::SchematicSheet(_)
                    | EditTarget::SchematicSheetPort(_)
                    | EditTarget::SchematicSheetLink(_)
            ),
            (Self::SchematicSymbolPresence(left), right)
            | (right, Self::SchematicSymbolPresence(left)) => {
                matches!(right.target(), EditTarget::SchematicSymbol(right) if left == &right)
            }
            (Self::SchematicSymbolLibraryPresence(left), right)
            | (right, Self::SchematicSymbolLibraryPresence(left)) => {
                matches!(
                    right.target(),
                    EditTarget::SchematicSymbolLibrary(right) if left == &right
                )
            }
            (Self::SchematicPortPlacementPresence(left), right)
            | (right, Self::SchematicPortPlacementPresence(left)) => {
                matches!(
                    right.target(),
                    EditTarget::SchematicPortPlacement(right) if left == &right
                )
            }
            (Self::SchematicWirePresence(left), right)
            | (right, Self::SchematicWirePresence(left)) => {
                matches!(right.target(), EditTarget::SchematicWire(right) if left == &right)
            }
            (Self::SchematicLabelPresence(left), right)
            | (right, Self::SchematicLabelPresence(left)) => {
                matches!(right.target(), EditTarget::SchematicLabel(right) if left == &right)
            }
            (Self::SchematicSheetPresence(left), right)
            | (right, Self::SchematicSheetPresence(left)) => {
                matches!(right.target(), EditTarget::SchematicSheet(right) if left == &right)
            }
            (Self::SchematicSheetPortPresence(left), right)
            | (right, Self::SchematicSheetPortPresence(left)) => {
                matches!(
                    right.target(),
                    EditTarget::SchematicSheetPort(right) if left == &right
                )
            }
            (Self::SchematicSheetLinkPresence(left), right)
            | (right, Self::SchematicSheetLinkPresence(left)) => {
                matches!(
                    right.target(),
                    EditTarget::SchematicSheetLink(right) if left == &right
                )
            }
            (Self::DeviceModelPresence(left) | Self::DeviceModelDefinition(left), right)
            | (right, Self::DeviceModelPresence(left) | Self::DeviceModelDefinition(left)) => {
                matches!(right.target(), EditTarget::DeviceModel(right) if left == &right)
            }
            (Self::CircuitInstancePresence(left), right)
            | (right, Self::CircuitInstancePresence(left)) => {
                matches!(right.target(), EditTarget::CircuitInstance(right) if left == &right)
            }
            (Self::CircuitInstanceDefinition(left), right)
            | (right, Self::CircuitInstanceDefinition(left)) => {
                matches!(right.target(), EditTarget::CircuitInstance(right) if left == &right)
            }
            (Self::PcbPresence(_), right) | (right, Self::PcbPresence(_)) => matches!(
                right.target(),
                EditTarget::Pcb(_)
                    | EditTarget::Board(_)
                    | EditTarget::LandPattern(_)
                    | EditTarget::Keepout(_)
                    | EditTarget::Placement(_)
                    | EditTarget::PlacementConstraint(_)
                    | EditTarget::Route(_)
                    | EditTarget::Via(_)
                    | EditTarget::Zone(_)
            ),
            (Self::LandPatternPresence(left), right) | (right, Self::LandPatternPresence(left)) => {
                matches!(right.target(), EditTarget::LandPattern(right) if left == &right)
            }
            (Self::KeepoutPresence(left), right) | (right, Self::KeepoutPresence(left)) => {
                matches!(right.target(), EditTarget::Keepout(right) if left == &right)
            }
            (
                Self::PlacementConstraintPresence(left) | Self::PlacementConstraintDefinition(left),
                right,
            )
            | (
                right,
                Self::PlacementConstraintPresence(left) | Self::PlacementConstraintDefinition(left),
            ) => matches!(
                right.target(),
                EditTarget::PlacementConstraint(right) if left == &right
            ),
            (Self::PlacementPresence(left), right) | (right, Self::PlacementPresence(left)) => {
                matches!(right.target(), EditTarget::Placement(right) if left == &right)
            }
            (Self::RoutePresence(left), right) | (right, Self::RoutePresence(left)) => {
                matches!(right.target(), EditTarget::Route(right) if left == &right)
            }
            (Self::RouteDefinition(left), right) | (right, Self::RouteDefinition(left)) => {
                matches!(right.target(), EditTarget::Route(right) if left == &right)
            }
            (Self::ViaPresence(left), right) | (right, Self::ViaPresence(left)) => {
                matches!(right.target(), EditTarget::Via(right) if left == &right)
            }
            (Self::ViaDefinition(left), right) | (right, Self::ViaDefinition(left)) => {
                matches!(right.target(), EditTarget::Via(right) if left == &right)
            }
            (Self::ZonePresence(left), right) | (right, Self::ZonePresence(left)) => {
                matches!(right.target(), EditTarget::Zone(right) if left == &right)
            }
            (Self::ZoneDefinition(left), right) | (right, Self::ZoneDefinition(left)) => {
                matches!(right.target(), EditTarget::Zone(right) if left == &right)
            }
            _ => false,
        }
    }
}

/// Supported exact semantic edit.
#[derive(Clone, Debug, PartialEq, serde::Deserialize, serde::Serialize)]
pub enum DesignEdit {
    /// Inserts one complete logical circuit net.
    InsertNet {
        /// New source-owned net. Its identity must not already exist.
        net: Net,
        /// Exact list position used by generated inverse edits.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        index: Option<usize>,
    },
    /// Removes one logical circuit net by stable identity.
    RemoveNet {
        /// Existing net target.
        net: NetId,
    },
    /// Changes whether one retained net is the circuit reference ground.
    SetNetGround {
        /// Existing net target.
        net: NetId,
        /// New reference-ground designation.
        is_ground: bool,
    },
    /// Inserts one complete ordered circuit bus.
    InsertBus {
        /// New source-owned bus. Its identity must not already exist.
        bus: Bus,
        /// Exact list position used by generated inverse edits.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        index: Option<usize>,
    },
    /// Removes one retained circuit bus by stable identity.
    RemoveBus {
        /// Existing bus target.
        bus: BusId,
    },
    /// Replaces the complete ordered member list of one retained bus.
    SetBusNets {
        /// Existing bus target.
        bus: BusId,
        /// New nonempty, unique ordered net members.
        nets: Vec<NetId>,
    },
    /// Inserts one complete named bus slice.
    InsertBusSlice {
        /// New source-owned slice. Its identity must not already exist.
        slice: BusSlice,
        /// Exact list position used by generated inverse edits.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        index: Option<usize>,
    },
    /// Removes one named bus slice by stable identity.
    RemoveBusSlice {
        /// Existing bus-slice target.
        slice: BusSliceId,
    },
    /// Replaces the source, range, and ordering of one bus slice.
    SetBusSliceDefinition {
        /// Existing bus-slice target.
        slice: BusSliceId,
        /// New source bus.
        bus: BusId,
        /// New zero-based first selected member.
        offset: usize,
        /// New nonzero member count.
        width: usize,
        /// New exposed member ordering.
        order: BusSliceOrder,
    },
    /// Inserts one complete circuit boundary port.
    InsertCircuitPort {
        /// New source-owned port. Its identity must not already exist.
        port: CircuitPort,
        /// Exact list position used by generated inverse edits.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        index: Option<usize>,
    },
    /// Removes one circuit boundary port by stable identity.
    RemoveCircuitPort {
        /// Existing port target.
        port: PortId,
    },
    /// Replaces the retained definition of one circuit boundary port.
    SetCircuitPortDefinition {
        /// Existing port target.
        port: PortId,
        /// New exposed logical net.
        net: NetId,
        /// New authored electrical direction.
        direction: PortDirection,
        /// Whether parents may omit this port.
        optional: bool,
    },
    /// Inserts one complete rail-intent record.
    InsertRail {
        /// New source-owned rail. Its net must not already have rail intent.
        rail: RailIntent,
        /// Exact list position used by generated inverse edits.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        index: Option<usize>,
    },
    /// Removes one rail-intent record by retained net.
    RemoveRail {
        /// Net whose existing rail intent is removed.
        net: NetId,
    },
    /// Replaces the authored definition of one retained rail.
    SetRailDefinition {
        /// Net whose rail intent is changed.
        net: NetId,
        /// New exact nominal voltage.
        nominal_voltage: Option<Real>,
        /// New exact maximum current magnitude.
        max_current: Option<Real>,
        /// New authored electrical role.
        kind: RailKind,
    },
    /// Inserts one complete reusable-circuit interface parameter.
    InsertModuleParameter {
        /// New source-owned parameter. Its name must be unique.
        parameter: CircuitModuleParameter,
        /// Exact list position used by generated inverse edits.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        index: Option<usize>,
    },
    /// Removes one reusable-circuit interface parameter by name.
    RemoveModuleParameter {
        /// Existing module-parameter name.
        parameter: String,
    },
    /// Replaces one reusable-circuit interface parameter definition.
    SetModuleParameterDefinition {
        /// Existing module-parameter name.
        parameter: String,
        /// New exact default value.
        default: Real,
        /// New required unit.
        unit: String,
        /// New provenance.
        source: String,
        /// New controlled local/nested targets.
        targets: Vec<CircuitModuleParameterTarget>,
    },
    /// Inserts one complete component-addressed source stimulus.
    InsertSourceStimulus {
        /// New source-owned stimulus. Its component must be unique.
        stimulus: SourceStimulus,
        /// Exact list position used by generated inverse edits.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        index: Option<usize>,
    },
    /// Removes one source stimulus by stable component identity.
    RemoveSourceStimulus {
        /// Existing source component target.
        component: ComponentId,
    },
    /// Replaces the exact waveform of one retained source stimulus.
    SetSourceStimulusWaveform {
        /// Existing source component target.
        component: ComponentId,
        /// New retained exact waveform.
        waveform: SourceWaveform,
    },
    /// Inserts one complete reusable child-circuit instance.
    InsertSubcircuit {
        /// New source-owned child instance. Its identity must be unique.
        subcircuit: SubcircuitInstance,
        /// Exact list position used by generated inverse edits.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        index: Option<usize>,
    },
    /// Removes one reusable child-circuit instance.
    RemoveSubcircuit {
        /// Existing child-instance target.
        subcircuit: SubcircuitInstanceId,
    },
    /// Replaces one child instance's definition, bindings, and overrides.
    SetSubcircuitDefinition {
        /// Existing child-instance target.
        subcircuit: SubcircuitInstanceId,
        /// Referenced child circuit definition.
        circuit: CircuitId,
        /// Complete child-port to parent-net bindings.
        ports: Vec<SubcircuitPortBinding>,
        /// Complete exact child-module parameter overrides.
        parameter_overrides: Vec<CircuitModuleParameterOverride>,
    },
    /// Replaces the complete ordered manual MNA stamp vector.
    SetLinearStamps {
        /// New exact manual linear stamps.
        stamps: Vec<LinearStamp>,
    },
    /// Replaces circuit transient and adapter policy together.
    SetCircuitPolicy {
        /// New transient integration policy.
        transient_policy: TransientPolicy,
        /// New solver/adapter family.
        adapter_policy: AdapterKind,
    },
    /// Attaches one complete schematic layout to a document that has none.
    InsertSchematic {
        /// New source-owned schematic layout.
        schematic: Box<SchematicLayout>,
    },
    /// Removes the complete retained schematic layout.
    RemoveSchematic,
    /// Inserts one complete placed schematic symbol unit.
    InsertSchematicSymbol {
        /// New source-owned symbol. Its identity must not already exist.
        symbol: Box<SchematicSymbol>,
        /// Exact list position used by generated inverse edits.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        index: Option<usize>,
    },
    /// Removes one placed schematic symbol unit by stable identity.
    RemoveSchematicSymbol {
        /// Existing schematic-symbol target.
        symbol: SchematicSymbolId,
    },
    /// Replaces one schematic symbol unit without changing its identity.
    SetSchematicSymbolDefinition {
        /// Complete replacement carrying the existing stable identity.
        symbol: Box<SchematicSymbol>,
    },
    /// Inserts one reusable multipart schematic symbol definition.
    InsertSchematicSymbolLibrary {
        /// New source-owned definition. Its identity must not already exist.
        definition: Box<SchematicSymbolDefinition>,
        /// Exact list position used by generated inverse edits.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        index: Option<usize>,
    },
    /// Removes one reusable schematic symbol definition by stable identity.
    RemoveSchematicSymbolLibrary {
        /// Existing library-definition target.
        definition: SchematicSymbolDefinitionId,
    },
    /// Replaces one reusable schematic symbol definition without changing its identity.
    SetSchematicSymbolLibraryDefinition {
        /// Complete replacement carrying the existing stable identity.
        definition: Box<SchematicSymbolDefinition>,
    },
    /// Inserts one circuit-boundary-port placement into the schematic.
    InsertSchematicPortPlacement {
        /// New source-owned placement. Its port must not already be placed.
        placement: SchematicPortPlacement,
        /// Exact list position used by generated inverse edits.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        index: Option<usize>,
    },
    /// Removes one schematic circuit-boundary-port placement.
    RemoveSchematicPortPlacement {
        /// Circuit port whose schematic placement is removed.
        port: PortId,
    },
    /// Replaces one retained schematic circuit-port placement.
    SetSchematicPortPlacementDefinition {
        /// Complete replacement carrying the existing circuit-port identity.
        placement: SchematicPortPlacement,
    },
    /// Inserts one complete retained schematic wire.
    InsertSchematicWire {
        /// New source-owned wire. Its identity must not already exist.
        wire: Box<SchematicWire>,
        /// Exact list position used by generated inverse edits.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        index: Option<usize>,
    },
    /// Removes one retained schematic wire by stable identity.
    RemoveSchematicWire {
        /// Existing schematic-wire target.
        wire: SchematicWireId,
    },
    /// Replaces one schematic wire without changing its identity.
    SetSchematicWireDefinition {
        /// Complete replacement carrying the existing stable identity.
        wire: Box<SchematicWire>,
    },
    /// Inserts one complete retained schematic net label.
    InsertSchematicLabel {
        /// New source-owned label. Its identity must not already exist.
        label: SchematicLabel,
        /// Exact list position used by generated inverse edits.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        index: Option<usize>,
    },
    /// Removes one retained schematic label by stable identity.
    RemoveSchematicLabel {
        /// Existing schematic-label target.
        label: SchematicLabelId,
    },
    /// Replaces one schematic label without changing its identity.
    SetSchematicLabelDefinition {
        /// Complete replacement carrying the existing stable identity.
        label: SchematicLabel,
    },
    /// Inserts one complete retained schematic sheet.
    InsertSchematicSheet {
        /// New source-owned sheet. Its identity must not already exist.
        sheet: SchematicSheet,
        /// Exact list position used by generated inverse edits.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        index: Option<usize>,
    },
    /// Removes one retained schematic sheet by stable identity.
    RemoveSchematicSheet {
        /// Existing schematic-sheet target.
        sheet: SchematicSheetId,
    },
    /// Replaces one schematic sheet without changing its identity.
    SetSchematicSheetDefinition {
        /// Complete replacement carrying the existing stable identity.
        sheet: SchematicSheet,
    },
    /// Inserts one complete schematic sheet-boundary port.
    InsertSchematicSheetPort {
        /// New source-owned sheet port. Its identity must not already exist.
        port: SchematicSheetPort,
        /// Exact list position used by generated inverse edits.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        index: Option<usize>,
    },
    /// Removes one schematic sheet-boundary port by stable identity.
    RemoveSchematicSheetPort {
        /// Existing schematic sheet-port target.
        port: SchematicSheetPortId,
    },
    /// Replaces one schematic sheet-boundary port without changing its identity.
    SetSchematicSheetPortDefinition {
        /// Complete replacement carrying the existing stable identity.
        port: SchematicSheetPort,
    },
    /// Inserts one complete parent/child schematic sheet link.
    InsertSchematicSheetLink {
        /// New source-owned sheet link. Its identity must not already exist.
        link: SchematicSheetLink,
        /// Exact list position used by generated inverse edits.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        index: Option<usize>,
    },
    /// Removes one parent/child schematic sheet link by stable identity.
    RemoveSchematicSheetLink {
        /// Existing schematic sheet-link target.
        link: SchematicSheetLinkId,
    },
    /// Replaces one schematic sheet link without changing its identity.
    SetSchematicSheetLinkDefinition {
        /// Complete replacement carrying the existing stable identity.
        link: SchematicSheetLink,
    },
    /// Inserts one complete reusable electrical device model.
    InsertDeviceModel {
        /// New source-owned model. Its identity must not already exist.
        model: Box<DeviceModel>,
        /// Exact list position used by generated inverse edits.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        index: Option<usize>,
    },
    /// Removes one reusable electrical device model by stable identity.
    RemoveDeviceModel {
        /// Existing model target.
        model: DeviceModelId,
    },
    /// Replaces one reusable device model without changing its identity.
    SetDeviceModelDefinition {
        /// Complete replacement carrying the existing stable identity.
        model: Box<DeviceModel>,
    },
    /// Inserts one complete logical circuit instance.
    InsertCircuitInstance {
        /// New source-owned instance. Its identity and component must be unique.
        instance: Box<CircuitInstance>,
        /// Exact list position used by generated inverse edits.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        index: Option<usize>,
    },
    /// Removes one logical circuit instance by stable identity.
    RemoveCircuitInstance {
        /// Existing circuit-instance target.
        instance: CircuitInstanceId,
    },
    /// Replaces one logical circuit instance without changing its identity.
    SetCircuitInstanceDefinition {
        /// Complete replacement carrying the existing stable identity.
        instance: Box<CircuitInstance>,
    },
    /// Reassigns one logical circuit instance to another retained device model.
    SetCircuitInstanceModel {
        /// Stable logical instance target.
        instance: CircuitInstanceId,
        /// Existing reusable model to assign.
        model: DeviceModelId,
    },
    /// Replaces all ordered pin-to-net bindings of one circuit instance.
    SetCircuitInstancePins {
        /// Stable logical instance target.
        instance: CircuitInstanceId,
        /// Complete new pin-binding set.
        pins: Vec<PinBinding>,
    },
    /// Attaches one complete PCB layout to a document that has none.
    InsertPcb {
        /// New source-owned retained board.
        pcb: Box<PcbLayout>,
    },
    /// Removes the complete retained PCB layout.
    RemovePcb {
        /// Stable board identity expected at the optional PCB boundary.
        board: BoardId,
    },
    /// Replaces one PCB substrate boundary and its cutouts.
    SetBoardOutline {
        /// Stable board target.
        board: BoardId,
        /// New exact mixed-curve board boundary.
        outline: BoardOutline,
    },
    /// Replaces one PCB's complete ordered physical stackup.
    SetBoardStackup {
        /// Stable board target.
        board: BoardId,
        /// New ordered physical stackup.
        stackup: PcbStackup,
    },
    /// Replaces one PCB's complete routing and verification rule deck.
    SetBoardRules {
        /// Stable board target.
        board: BoardId,
        /// New authored design-rule policy.
        rules: PcbDesignRules,
    },
    /// Inserts one complete board-local land pattern.
    InsertLandPattern {
        /// New source-owned pattern. Its identity must not already exist.
        land_pattern: Box<LandPattern>,
        /// Exact list position used by generated inverse edits.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        index: Option<usize>,
    },
    /// Removes one board-local land pattern by stable identity.
    RemoveLandPattern {
        /// Existing land-pattern target.
        land_pattern: LandPatternId,
    },
    /// Replaces one land-pattern definition without changing its identity.
    SetLandPatternDefinition {
        /// Complete replacement carrying the existing stable identity.
        land_pattern: Box<LandPattern>,
    },
    /// Inserts one complete retained physical-layout keepout.
    InsertKeepout {
        /// New source-owned keepout. Its identity must not already exist.
        keepout: PcbKeepout,
        /// Exact list position used by generated inverse edits.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        index: Option<usize>,
    },
    /// Removes one retained physical-layout keepout by stable identity.
    RemoveKeepout {
        /// Existing keepout target.
        keepout: KeepoutId,
    },
    /// Replaces one keepout boundary and scope without changing its identity.
    SetKeepoutDefinition {
        /// Complete replacement carrying the existing stable identity.
        keepout: PcbKeepout,
    },
    /// Replaces the complete authored transform of one placed instance.
    SetPlacementTransform {
        /// Stable logical instance target.
        instance: CircuitInstanceId,
        /// Exact board-space footprint origin.
        #[serde(with = "crate::interchange::point")]
        position: Point2,
        /// Exact counter-clockwise rotation in degrees.
        rotation_degrees: Real,
        /// Physical board side.
        side: BoardSide,
    },
    /// Reassigns one placement to another retained land pattern.
    SetPlacementLandPattern {
        /// Stable logical instance target.
        instance: CircuitInstanceId,
        /// Existing board-local land pattern to assign.
        land_pattern: LandPatternId,
    },
    /// Inserts one complete component placement.
    InsertPlacement {
        /// New source-owned placement. Its instance must not already be placed.
        placement: Box<PcbPlacement>,
        /// Exact list position used by generated inverse edits.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        index: Option<usize>,
    },
    /// Removes one component placement by logical instance identity.
    RemovePlacement {
        /// Existing placed circuit instance.
        instance: CircuitInstanceId,
    },
    /// Inserts one complete retained placement constraint.
    InsertPlacementConstraint {
        /// New source-owned constraint. Its identity must not already exist.
        constraint: Box<PlacementConstraint>,
        /// Exact list position used by generated inverse edits.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        index: Option<usize>,
    },
    /// Removes one retained placement constraint by stable identity.
    RemovePlacementConstraint {
        /// Existing placement-constraint target.
        constraint: PlacementConstraintId,
    },
    /// Replaces one placement constraint without changing its identity.
    SetPlacementConstraintDefinition {
        /// Complete replacement carrying the existing stable identity.
        constraint: Box<PlacementConstraint>,
    },
    /// Changes the exact finished width of one retained copper route.
    SetRouteWidth {
        /// Stable route target.
        route: RouteId,
        /// New exact copper width.
        width: Real,
    },
    /// Replaces the ordered retained centerline of one copper route.
    SetRouteSegments {
        /// Stable route target.
        route: RouteId,
        /// New ordered line/arc/Bezier centerline.
        #[serde(with = "crate::interchange::route_segments")]
        segments: Vec<PcbRouteSegment>,
    },
    /// Reassigns one retained route to another logical net.
    SetRouteNet {
        /// Stable route target.
        route: RouteId,
        /// Existing logical net to carry.
        net: NetId,
    },
    /// Inserts one complete retained copper route.
    InsertRoute {
        /// New source-owned route. Its identity must not already exist.
        route: Box<PcbRoute>,
        /// Exact list position used by generated inverse edits.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        index: Option<usize>,
    },
    /// Removes one retained copper route by stable identity.
    RemoveRoute {
        /// Existing route target.
        route: RouteId,
    },
    /// Replaces one retained route without changing its identity.
    SetRouteDefinition {
        /// Complete replacement carrying the existing stable identity.
        route: Box<PcbRoute>,
    },
    /// Moves one retained via without changing its net, span, or drill policy.
    MoveVia {
        /// Stable via target.
        via: ViaId,
        /// New exact board-space center.
        #[serde(with = "crate::interchange::point")]
        center: Point2,
    },
    /// Reassigns one retained via to another logical net.
    SetViaNet {
        /// Stable via target.
        via: ViaId,
        /// Existing logical net to carry.
        net: NetId,
    },
    /// Inserts one complete retained via.
    InsertVia {
        /// New source-owned via. Its identity must not already exist.
        via: Box<PcbVia>,
        /// Exact list position used by generated inverse edits.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        index: Option<usize>,
    },
    /// Removes one retained via by stable identity.
    RemoveVia {
        /// Existing via target.
        via: ViaId,
    },
    /// Replaces one retained via without changing its identity.
    SetViaDefinition {
        /// Complete replacement carrying the existing stable identity.
        via: Box<PcbVia>,
    },
    /// Replaces one copper-zone boundary without changing its net or fill policy.
    SetZoneBoundary {
        /// Stable zone target.
        zone: ZoneId,
        /// New exact closed-boundary vertices.
        #[serde(with = "crate::interchange::points")]
        boundary: Vec<Point2>,
    },
    /// Reassigns one retained copper zone to another logical net.
    SetZoneNet {
        /// Stable zone target.
        zone: ZoneId,
        /// Existing logical net to carry.
        net: NetId,
    },
    /// Inserts one complete retained copper zone.
    InsertZone {
        /// New source-owned zone. Its identity must not already exist.
        zone: Box<CopperZone>,
        /// Exact list position used by generated inverse edits.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        index: Option<usize>,
    },
    /// Removes one retained copper zone by stable identity.
    RemoveZone {
        /// Existing zone target.
        zone: ZoneId,
    },
    /// Replaces one retained copper zone without changing its identity.
    SetZoneDefinition {
        /// Complete replacement carrying the existing stable identity.
        zone: Box<CopperZone>,
    },
}

impl DesignEdit {
    /// Constructs a complete logical-net insertion.
    pub fn insert_net(net: Net) -> Self {
        Self::InsertNet { net, index: None }
    }

    /// Constructs a logical-net removal.
    pub fn remove_net(net: NetId) -> Self {
        Self::RemoveNet { net }
    }

    /// Constructs a complete ordered-bus insertion.
    pub fn insert_bus(bus: Bus) -> Self {
        Self::InsertBus { bus, index: None }
    }

    /// Constructs an ordered-bus removal.
    pub fn remove_bus(bus: BusId) -> Self {
        Self::RemoveBus { bus }
    }

    /// Constructs a complete named bus-slice insertion.
    pub fn insert_bus_slice(slice: BusSlice) -> Self {
        Self::InsertBusSlice { slice, index: None }
    }

    /// Constructs a named bus-slice removal.
    pub fn remove_bus_slice(slice: BusSliceId) -> Self {
        Self::RemoveBusSlice { slice }
    }

    /// Constructs a complete circuit-boundary-port insertion.
    pub fn insert_circuit_port(port: CircuitPort) -> Self {
        Self::InsertCircuitPort { port, index: None }
    }

    /// Constructs a circuit-boundary-port removal.
    pub fn remove_circuit_port(port: PortId) -> Self {
        Self::RemoveCircuitPort { port }
    }

    /// Constructs a complete rail-intent insertion.
    pub fn insert_rail(rail: RailIntent) -> Self {
        Self::InsertRail { rail, index: None }
    }

    /// Constructs a rail-intent removal.
    pub fn remove_rail(net: NetId) -> Self {
        Self::RemoveRail { net }
    }

    /// Constructs a complete module-parameter insertion.
    pub fn insert_module_parameter(parameter: CircuitModuleParameter) -> Self {
        Self::InsertModuleParameter {
            parameter,
            index: None,
        }
    }

    /// Constructs a module-parameter removal.
    pub fn remove_module_parameter(parameter: impl Into<String>) -> Self {
        Self::RemoveModuleParameter {
            parameter: parameter.into(),
        }
    }

    /// Constructs a complete source-stimulus insertion.
    pub fn insert_source_stimulus(stimulus: SourceStimulus) -> Self {
        Self::InsertSourceStimulus {
            stimulus,
            index: None,
        }
    }

    /// Constructs a source-stimulus removal.
    pub fn remove_source_stimulus(component: ComponentId) -> Self {
        Self::RemoveSourceStimulus { component }
    }

    /// Constructs a complete child-circuit-instance insertion.
    pub fn insert_subcircuit(subcircuit: SubcircuitInstance) -> Self {
        Self::InsertSubcircuit {
            subcircuit,
            index: None,
        }
    }

    /// Constructs a child-circuit-instance removal.
    pub fn remove_subcircuit(subcircuit: SubcircuitInstanceId) -> Self {
        Self::RemoveSubcircuit { subcircuit }
    }

    /// Constructs a complete schematic-layout insertion.
    pub fn insert_schematic(schematic: SchematicLayout) -> Self {
        Self::InsertSchematic {
            schematic: Box::new(schematic),
        }
    }

    /// Constructs a complete schematic-layout removal.
    pub fn remove_schematic() -> Self {
        Self::RemoveSchematic
    }

    /// Constructs a complete schematic-symbol insertion.
    pub fn insert_schematic_symbol(symbol: SchematicSymbol) -> Self {
        Self::InsertSchematicSymbol {
            symbol: Box::new(symbol),
            index: None,
        }
    }

    /// Constructs a schematic-symbol removal.
    pub fn remove_schematic_symbol(symbol: SchematicSymbolId) -> Self {
        Self::RemoveSchematicSymbol { symbol }
    }

    /// Constructs a reusable schematic-symbol-definition insertion.
    pub fn insert_schematic_symbol_library(definition: SchematicSymbolDefinition) -> Self {
        Self::InsertSchematicSymbolLibrary {
            definition: Box::new(definition),
            index: None,
        }
    }

    /// Constructs a reusable schematic-symbol-definition removal.
    pub fn remove_schematic_symbol_library(definition: SchematicSymbolDefinitionId) -> Self {
        Self::RemoveSchematicSymbolLibrary { definition }
    }

    /// Constructs a complete schematic circuit-port-placement insertion.
    pub fn insert_schematic_port_placement(placement: SchematicPortPlacement) -> Self {
        Self::InsertSchematicPortPlacement {
            placement,
            index: None,
        }
    }

    /// Constructs a schematic circuit-port-placement removal.
    pub fn remove_schematic_port_placement(port: PortId) -> Self {
        Self::RemoveSchematicPortPlacement { port }
    }

    /// Constructs a complete schematic-wire insertion.
    pub fn insert_schematic_wire(wire: SchematicWire) -> Self {
        Self::InsertSchematicWire {
            wire: Box::new(wire),
            index: None,
        }
    }

    /// Constructs a schematic-wire removal.
    pub fn remove_schematic_wire(wire: SchematicWireId) -> Self {
        Self::RemoveSchematicWire { wire }
    }

    /// Constructs a complete schematic-label insertion.
    pub fn insert_schematic_label(label: SchematicLabel) -> Self {
        Self::InsertSchematicLabel { label, index: None }
    }

    /// Constructs a schematic-label removal.
    pub fn remove_schematic_label(label: SchematicLabelId) -> Self {
        Self::RemoveSchematicLabel { label }
    }

    /// Constructs a complete schematic-sheet insertion.
    pub fn insert_schematic_sheet(sheet: SchematicSheet) -> Self {
        Self::InsertSchematicSheet { sheet, index: None }
    }

    /// Constructs a schematic-sheet removal.
    pub fn remove_schematic_sheet(sheet: SchematicSheetId) -> Self {
        Self::RemoveSchematicSheet { sheet }
    }

    /// Constructs a complete schematic sheet-port insertion.
    pub fn insert_schematic_sheet_port(port: SchematicSheetPort) -> Self {
        Self::InsertSchematicSheetPort { port, index: None }
    }

    /// Constructs a schematic sheet-port removal.
    pub fn remove_schematic_sheet_port(port: SchematicSheetPortId) -> Self {
        Self::RemoveSchematicSheetPort { port }
    }

    /// Constructs a complete schematic sheet-link insertion.
    pub fn insert_schematic_sheet_link(link: SchematicSheetLink) -> Self {
        Self::InsertSchematicSheetLink { link, index: None }
    }

    /// Constructs a schematic sheet-link removal.
    pub fn remove_schematic_sheet_link(link: SchematicSheetLinkId) -> Self {
        Self::RemoveSchematicSheetLink { link }
    }

    /// Constructs a complete reusable device-model insertion.
    pub fn insert_device_model(model: DeviceModel) -> Self {
        Self::InsertDeviceModel {
            model: Box::new(model),
            index: None,
        }
    }

    /// Constructs a reusable device-model removal.
    pub fn remove_device_model(model: DeviceModelId) -> Self {
        Self::RemoveDeviceModel { model }
    }

    /// Constructs a complete logical circuit-instance insertion.
    pub fn insert_circuit_instance(instance: CircuitInstance) -> Self {
        Self::InsertCircuitInstance {
            instance: Box::new(instance),
            index: None,
        }
    }

    /// Constructs a logical circuit-instance removal.
    pub fn remove_circuit_instance(instance: CircuitInstanceId) -> Self {
        Self::RemoveCircuitInstance { instance }
    }

    /// Constructs a complete PCB-layout insertion.
    pub fn insert_pcb(pcb: PcbLayout) -> Self {
        Self::InsertPcb { pcb: Box::new(pcb) }
    }

    /// Constructs a complete PCB-layout removal.
    pub fn remove_pcb(board: BoardId) -> Self {
        Self::RemovePcb { board }
    }

    /// Constructs a complete board-local land-pattern insertion.
    pub fn insert_land_pattern(land_pattern: LandPattern) -> Self {
        Self::InsertLandPattern {
            land_pattern: Box::new(land_pattern),
            index: None,
        }
    }

    /// Constructs a board-local land-pattern removal.
    pub fn remove_land_pattern(land_pattern: LandPatternId) -> Self {
        Self::RemoveLandPattern { land_pattern }
    }

    /// Constructs a complete physical-layout keepout insertion.
    pub fn insert_keepout(keepout: PcbKeepout) -> Self {
        Self::InsertKeepout {
            keepout,
            index: None,
        }
    }

    /// Constructs a physical-layout keepout removal.
    pub fn remove_keepout(keepout: KeepoutId) -> Self {
        Self::RemoveKeepout { keepout }
    }

    /// Constructs a complete component-placement insertion.
    pub fn insert_placement(placement: PcbPlacement) -> Self {
        Self::InsertPlacement {
            placement: Box::new(placement),
            index: None,
        }
    }

    /// Constructs a component-placement removal.
    pub fn remove_placement(instance: CircuitInstanceId) -> Self {
        Self::RemovePlacement { instance }
    }

    /// Constructs a complete placement-constraint insertion.
    pub fn insert_placement_constraint(constraint: PlacementConstraint) -> Self {
        Self::InsertPlacementConstraint {
            constraint: Box::new(constraint),
            index: None,
        }
    }

    /// Constructs a placement-constraint removal.
    pub fn remove_placement_constraint(constraint: PlacementConstraintId) -> Self {
        Self::RemovePlacementConstraint { constraint }
    }

    /// Constructs a complete retained-route insertion.
    pub fn insert_route(route: PcbRoute) -> Self {
        Self::InsertRoute {
            route: Box::new(route),
            index: None,
        }
    }

    /// Constructs a retained-route removal.
    pub fn remove_route(route: RouteId) -> Self {
        Self::RemoveRoute { route }
    }

    /// Constructs a complete retained-via insertion.
    pub fn insert_via(via: PcbVia) -> Self {
        Self::InsertVia {
            via: Box::new(via),
            index: None,
        }
    }

    /// Constructs a retained-via removal.
    pub fn remove_via(via: ViaId) -> Self {
        Self::RemoveVia { via }
    }

    /// Constructs a complete retained-zone insertion.
    pub fn insert_zone(zone: CopperZone) -> Self {
        Self::InsertZone {
            zone: Box::new(zone),
            index: None,
        }
    }

    /// Constructs a retained-zone removal.
    pub fn remove_zone(zone: ZoneId) -> Self {
        Self::RemoveZone { zone }
    }

    /// Returns the stable semantic target affected by this edit.
    pub fn target(&self) -> EditTarget {
        match self {
            Self::InsertNet { net, .. } => EditTarget::Net(net.id.clone()),
            Self::RemoveNet { net } | Self::SetNetGround { net, .. } => {
                EditTarget::Net(net.clone())
            }
            Self::InsertBus { bus, .. } => EditTarget::Bus(bus.id.clone()),
            Self::RemoveBus { bus } | Self::SetBusNets { bus, .. } => EditTarget::Bus(bus.clone()),
            Self::InsertBusSlice { slice, .. } => EditTarget::BusSlice(slice.id.clone()),
            Self::RemoveBusSlice { slice } | Self::SetBusSliceDefinition { slice, .. } => {
                EditTarget::BusSlice(slice.clone())
            }
            Self::InsertCircuitPort { port, .. } => EditTarget::CircuitPort(port.id.clone()),
            Self::RemoveCircuitPort { port } | Self::SetCircuitPortDefinition { port, .. } => {
                EditTarget::CircuitPort(port.clone())
            }
            Self::InsertRail { rail, .. } => EditTarget::Rail(rail.net.clone()),
            Self::RemoveRail { net } | Self::SetRailDefinition { net, .. } => {
                EditTarget::Rail(net.clone())
            }
            Self::InsertModuleParameter { parameter, .. } => {
                EditTarget::ModuleParameter(parameter.name.clone())
            }
            Self::RemoveModuleParameter { parameter }
            | Self::SetModuleParameterDefinition { parameter, .. } => {
                EditTarget::ModuleParameter(parameter.clone())
            }
            Self::InsertSourceStimulus { stimulus, .. } => {
                EditTarget::SourceStimulus(stimulus.component.clone())
            }
            Self::RemoveSourceStimulus { component }
            | Self::SetSourceStimulusWaveform { component, .. } => {
                EditTarget::SourceStimulus(component.clone())
            }
            Self::InsertSubcircuit { subcircuit, .. } => {
                EditTarget::Subcircuit(subcircuit.id.clone())
            }
            Self::RemoveSubcircuit { subcircuit }
            | Self::SetSubcircuitDefinition { subcircuit, .. } => {
                EditTarget::Subcircuit(subcircuit.clone())
            }
            Self::SetLinearStamps { .. } => EditTarget::LinearStamps,
            Self::SetCircuitPolicy { .. } => EditTarget::CircuitPolicy,
            Self::InsertSchematic { .. } | Self::RemoveSchematic => EditTarget::Schematic,
            Self::InsertSchematicSymbol { symbol, .. } => {
                EditTarget::SchematicSymbol(symbol.id.clone())
            }
            Self::RemoveSchematicSymbol { symbol } => EditTarget::SchematicSymbol(symbol.clone()),
            Self::SetSchematicSymbolDefinition { symbol } => {
                EditTarget::SchematicSymbol(symbol.id.clone())
            }
            Self::InsertSchematicSymbolLibrary { definition, .. } => {
                EditTarget::SchematicSymbolLibrary(definition.id.clone())
            }
            Self::RemoveSchematicSymbolLibrary { definition } => {
                EditTarget::SchematicSymbolLibrary(definition.clone())
            }
            Self::SetSchematicSymbolLibraryDefinition { definition } => {
                EditTarget::SchematicSymbolLibrary(definition.id.clone())
            }
            Self::InsertSchematicPortPlacement { placement, .. } => {
                EditTarget::SchematicPortPlacement(placement.port.clone())
            }
            Self::RemoveSchematicPortPlacement { port } => {
                EditTarget::SchematicPortPlacement(port.clone())
            }
            Self::SetSchematicPortPlacementDefinition { placement } => {
                EditTarget::SchematicPortPlacement(placement.port.clone())
            }
            Self::InsertSchematicWire { wire, .. } => EditTarget::SchematicWire(wire.id.clone()),
            Self::RemoveSchematicWire { wire } => EditTarget::SchematicWire(wire.clone()),
            Self::SetSchematicWireDefinition { wire } => EditTarget::SchematicWire(wire.id.clone()),
            Self::InsertSchematicLabel { label, .. } => {
                EditTarget::SchematicLabel(label.id.clone())
            }
            Self::RemoveSchematicLabel { label } => EditTarget::SchematicLabel(label.clone()),
            Self::SetSchematicLabelDefinition { label } => {
                EditTarget::SchematicLabel(label.id.clone())
            }
            Self::InsertSchematicSheet { sheet, .. } => {
                EditTarget::SchematicSheet(sheet.id.clone())
            }
            Self::RemoveSchematicSheet { sheet } => EditTarget::SchematicSheet(sheet.clone()),
            Self::SetSchematicSheetDefinition { sheet } => {
                EditTarget::SchematicSheet(sheet.id.clone())
            }
            Self::InsertSchematicSheetPort { port, .. } => {
                EditTarget::SchematicSheetPort(port.id.clone())
            }
            Self::RemoveSchematicSheetPort { port } => EditTarget::SchematicSheetPort(port.clone()),
            Self::SetSchematicSheetPortDefinition { port } => {
                EditTarget::SchematicSheetPort(port.id.clone())
            }
            Self::InsertSchematicSheetLink { link, .. } => {
                EditTarget::SchematicSheetLink(link.id.clone())
            }
            Self::RemoveSchematicSheetLink { link } => EditTarget::SchematicSheetLink(link.clone()),
            Self::SetSchematicSheetLinkDefinition { link } => {
                EditTarget::SchematicSheetLink(link.id.clone())
            }
            Self::InsertDeviceModel { model, .. } => EditTarget::DeviceModel(model.id.clone()),
            Self::RemoveDeviceModel { model } => EditTarget::DeviceModel(model.clone()),
            Self::SetDeviceModelDefinition { model } => EditTarget::DeviceModel(model.id.clone()),
            Self::InsertCircuitInstance { instance, .. } => {
                EditTarget::CircuitInstance(instance.id.clone())
            }
            Self::RemoveCircuitInstance { instance }
            | Self::SetCircuitInstanceModel { instance, .. }
            | Self::SetCircuitInstancePins { instance, .. } => {
                EditTarget::CircuitInstance(instance.clone())
            }
            Self::SetCircuitInstanceDefinition { instance } => {
                EditTarget::CircuitInstance(instance.id.clone())
            }
            Self::InsertPcb { pcb } => EditTarget::Pcb(pcb.id.clone()),
            Self::RemovePcb { board } => EditTarget::Pcb(board.clone()),
            Self::SetBoardOutline { board, .. }
            | Self::SetBoardStackup { board, .. }
            | Self::SetBoardRules { board, .. } => EditTarget::Board(board.clone()),
            Self::InsertLandPattern { land_pattern, .. } => {
                EditTarget::LandPattern(land_pattern.id.clone())
            }
            Self::RemoveLandPattern { land_pattern } => {
                EditTarget::LandPattern(land_pattern.clone())
            }
            Self::SetLandPatternDefinition { land_pattern } => {
                EditTarget::LandPattern(land_pattern.id.clone())
            }
            Self::InsertKeepout { keepout, .. } => EditTarget::Keepout(keepout.id.clone()),
            Self::RemoveKeepout { keepout } => EditTarget::Keepout(keepout.clone()),
            Self::SetKeepoutDefinition { keepout } => EditTarget::Keepout(keepout.id.clone()),
            Self::SetPlacementTransform { instance, .. } => EditTarget::Placement(instance.clone()),
            Self::SetPlacementLandPattern { instance, .. } => {
                EditTarget::Placement(instance.clone())
            }
            Self::InsertPlacement { placement, .. } => {
                EditTarget::Placement(placement.instance.clone())
            }
            Self::RemovePlacement { instance } => EditTarget::Placement(instance.clone()),
            Self::InsertPlacementConstraint { constraint, .. } => {
                EditTarget::PlacementConstraint(constraint.id.clone())
            }
            Self::RemovePlacementConstraint { constraint } => {
                EditTarget::PlacementConstraint(constraint.clone())
            }
            Self::SetPlacementConstraintDefinition { constraint } => {
                EditTarget::PlacementConstraint(constraint.id.clone())
            }
            Self::SetRouteWidth { route, .. }
            | Self::SetRouteSegments { route, .. }
            | Self::SetRouteNet { route, .. } => EditTarget::Route(route.clone()),
            Self::InsertRoute { route, .. } => EditTarget::Route(route.id.clone()),
            Self::RemoveRoute { route } => EditTarget::Route(route.clone()),
            Self::SetRouteDefinition { route } => EditTarget::Route(route.id.clone()),
            Self::MoveVia { via, .. } | Self::SetViaNet { via, .. } | Self::RemoveVia { via } => {
                EditTarget::Via(via.clone())
            }
            Self::InsertVia { via, .. } => EditTarget::Via(via.id.clone()),
            Self::SetViaDefinition { via } => EditTarget::Via(via.id.clone()),
            Self::SetZoneBoundary { zone, .. }
            | Self::SetZoneNet { zone, .. }
            | Self::RemoveZone { zone } => EditTarget::Zone(zone.clone()),
            Self::InsertZone { zone, .. } => EditTarget::Zone(zone.id.clone()),
            Self::SetZoneDefinition { zone } => EditTarget::Zone(zone.id.clone()),
        }
    }

    /// Returns the independently mergeable property changed by this edit.
    pub fn address(&self) -> EditAddress {
        match self {
            Self::InsertNet { net, .. } => EditAddress::NetPresence(net.id.clone()),
            Self::RemoveNet { net } => EditAddress::NetPresence(net.clone()),
            Self::SetNetGround { net, .. } => EditAddress::NetGround(net.clone()),
            Self::InsertBus { bus, .. } => EditAddress::BusPresence(bus.id.clone()),
            Self::RemoveBus { bus } => EditAddress::BusPresence(bus.clone()),
            Self::SetBusNets { bus, .. } => EditAddress::BusNets(bus.clone()),
            Self::InsertBusSlice { slice, .. } => EditAddress::BusSlicePresence(slice.id.clone()),
            Self::RemoveBusSlice { slice } => EditAddress::BusSlicePresence(slice.clone()),
            Self::SetBusSliceDefinition { slice, .. } => {
                EditAddress::BusSliceDefinition(slice.clone())
            }
            Self::InsertCircuitPort { port, .. } => {
                EditAddress::CircuitPortPresence(port.id.clone())
            }
            Self::RemoveCircuitPort { port } => EditAddress::CircuitPortPresence(port.clone()),
            Self::SetCircuitPortDefinition { port, .. } => {
                EditAddress::CircuitPortDefinition(port.clone())
            }
            Self::InsertRail { rail, .. } => EditAddress::RailPresence(rail.net.clone()),
            Self::RemoveRail { net } => EditAddress::RailPresence(net.clone()),
            Self::SetRailDefinition { net, .. } => EditAddress::RailDefinition(net.clone()),
            Self::InsertModuleParameter { parameter, .. } => {
                EditAddress::ModuleParameterPresence(parameter.name.clone())
            }
            Self::RemoveModuleParameter { parameter } => {
                EditAddress::ModuleParameterPresence(parameter.clone())
            }
            Self::SetModuleParameterDefinition { parameter, .. } => {
                EditAddress::ModuleParameterDefinition(parameter.clone())
            }
            Self::InsertSourceStimulus { stimulus, .. } => {
                EditAddress::SourceStimulusPresence(stimulus.component.clone())
            }
            Self::RemoveSourceStimulus { component } => {
                EditAddress::SourceStimulusPresence(component.clone())
            }
            Self::SetSourceStimulusWaveform { component, .. } => {
                EditAddress::SourceStimulusWaveform(component.clone())
            }
            Self::InsertSubcircuit { subcircuit, .. } => {
                EditAddress::SubcircuitPresence(subcircuit.id.clone())
            }
            Self::RemoveSubcircuit { subcircuit } => {
                EditAddress::SubcircuitPresence(subcircuit.clone())
            }
            Self::SetSubcircuitDefinition { subcircuit, .. } => {
                EditAddress::SubcircuitDefinition(subcircuit.clone())
            }
            Self::SetLinearStamps { .. } => EditAddress::LinearStamps,
            Self::SetCircuitPolicy { .. } => EditAddress::CircuitPolicy,
            Self::InsertSchematic { .. } | Self::RemoveSchematic => EditAddress::SchematicPresence,
            Self::InsertSchematicSymbol { symbol, .. } => {
                EditAddress::SchematicSymbolPresence(symbol.id.clone())
            }
            Self::RemoveSchematicSymbol { symbol } => {
                EditAddress::SchematicSymbolPresence(symbol.clone())
            }
            Self::SetSchematicSymbolDefinition { symbol } => {
                EditAddress::SchematicSymbolDefinition(symbol.id.clone())
            }
            Self::InsertSchematicSymbolLibrary { definition, .. } => {
                EditAddress::SchematicSymbolLibraryPresence(definition.id.clone())
            }
            Self::RemoveSchematicSymbolLibrary { definition } => {
                EditAddress::SchematicSymbolLibraryPresence(definition.clone())
            }
            Self::SetSchematicSymbolLibraryDefinition { definition } => {
                EditAddress::SchematicSymbolLibraryDefinition(definition.id.clone())
            }
            Self::InsertSchematicPortPlacement { placement, .. } => {
                EditAddress::SchematicPortPlacementPresence(placement.port.clone())
            }
            Self::RemoveSchematicPortPlacement { port } => {
                EditAddress::SchematicPortPlacementPresence(port.clone())
            }
            Self::SetSchematicPortPlacementDefinition { placement } => {
                EditAddress::SchematicPortPlacementDefinition(placement.port.clone())
            }
            Self::InsertSchematicWire { wire, .. } => {
                EditAddress::SchematicWirePresence(wire.id.clone())
            }
            Self::RemoveSchematicWire { wire } => EditAddress::SchematicWirePresence(wire.clone()),
            Self::SetSchematicWireDefinition { wire } => {
                EditAddress::SchematicWireDefinition(wire.id.clone())
            }
            Self::InsertSchematicLabel { label, .. } => {
                EditAddress::SchematicLabelPresence(label.id.clone())
            }
            Self::RemoveSchematicLabel { label } => {
                EditAddress::SchematicLabelPresence(label.clone())
            }
            Self::SetSchematicLabelDefinition { label } => {
                EditAddress::SchematicLabelDefinition(label.id.clone())
            }
            Self::InsertSchematicSheet { sheet, .. } => {
                EditAddress::SchematicSheetPresence(sheet.id.clone())
            }
            Self::RemoveSchematicSheet { sheet } => {
                EditAddress::SchematicSheetPresence(sheet.clone())
            }
            Self::SetSchematicSheetDefinition { sheet } => {
                EditAddress::SchematicSheetDefinition(sheet.id.clone())
            }
            Self::InsertSchematicSheetPort { port, .. } => {
                EditAddress::SchematicSheetPortPresence(port.id.clone())
            }
            Self::RemoveSchematicSheetPort { port } => {
                EditAddress::SchematicSheetPortPresence(port.clone())
            }
            Self::SetSchematicSheetPortDefinition { port } => {
                EditAddress::SchematicSheetPortDefinition(port.id.clone())
            }
            Self::InsertSchematicSheetLink { link, .. } => {
                EditAddress::SchematicSheetLinkPresence(link.id.clone())
            }
            Self::RemoveSchematicSheetLink { link } => {
                EditAddress::SchematicSheetLinkPresence(link.clone())
            }
            Self::SetSchematicSheetLinkDefinition { link } => {
                EditAddress::SchematicSheetLinkDefinition(link.id.clone())
            }
            Self::InsertDeviceModel { model, .. } => {
                EditAddress::DeviceModelPresence(model.id.clone())
            }
            Self::RemoveDeviceModel { model } => EditAddress::DeviceModelPresence(model.clone()),
            Self::SetDeviceModelDefinition { model } => {
                EditAddress::DeviceModelDefinition(model.id.clone())
            }
            Self::InsertCircuitInstance { instance, .. } => {
                EditAddress::CircuitInstancePresence(instance.id.clone())
            }
            Self::RemoveCircuitInstance { instance } => {
                EditAddress::CircuitInstancePresence(instance.clone())
            }
            Self::SetCircuitInstanceDefinition { instance } => {
                EditAddress::CircuitInstanceDefinition(instance.id.clone())
            }
            Self::InsertPcb { pcb } => EditAddress::PcbPresence(pcb.id.clone()),
            Self::RemovePcb { board } => EditAddress::PcbPresence(board.clone()),
            Self::SetCircuitInstanceModel { instance, .. } => {
                EditAddress::CircuitInstanceModel(instance.clone())
            }
            Self::SetCircuitInstancePins { instance, .. } => {
                EditAddress::CircuitInstancePins(instance.clone())
            }
            Self::SetBoardOutline { board, .. } => EditAddress::BoardOutline(board.clone()),
            Self::SetBoardStackup { board, .. } => EditAddress::BoardStackup(board.clone()),
            Self::SetBoardRules { board, .. } => EditAddress::BoardRules(board.clone()),
            Self::InsertLandPattern { land_pattern, .. } => {
                EditAddress::LandPatternPresence(land_pattern.id.clone())
            }
            Self::RemoveLandPattern { land_pattern } => {
                EditAddress::LandPatternPresence(land_pattern.clone())
            }
            Self::SetLandPatternDefinition { land_pattern } => {
                EditAddress::LandPatternDefinition(land_pattern.id.clone())
            }
            Self::InsertKeepout { keepout, .. } => EditAddress::KeepoutPresence(keepout.id.clone()),
            Self::RemoveKeepout { keepout } => EditAddress::KeepoutPresence(keepout.clone()),
            Self::SetKeepoutDefinition { keepout } => {
                EditAddress::KeepoutDefinition(keepout.id.clone())
            }
            Self::SetPlacementTransform { instance, .. } => {
                EditAddress::PlacementTransform(instance.clone())
            }
            Self::SetPlacementLandPattern { instance, .. } => {
                EditAddress::PlacementLandPattern(instance.clone())
            }
            Self::InsertPlacement { placement, .. } => {
                EditAddress::PlacementPresence(placement.instance.clone())
            }
            Self::RemovePlacement { instance } => EditAddress::PlacementPresence(instance.clone()),
            Self::InsertPlacementConstraint { constraint, .. } => {
                EditAddress::PlacementConstraintPresence(constraint.id.clone())
            }
            Self::RemovePlacementConstraint { constraint } => {
                EditAddress::PlacementConstraintPresence(constraint.clone())
            }
            Self::SetPlacementConstraintDefinition { constraint } => {
                EditAddress::PlacementConstraintDefinition(constraint.id.clone())
            }
            Self::SetRouteWidth { route, .. } => EditAddress::RouteWidth(route.clone()),
            Self::SetRouteSegments { route, .. } => EditAddress::RouteSegments(route.clone()),
            Self::SetRouteNet { route, .. } => EditAddress::RouteNet(route.clone()),
            Self::InsertRoute { route, .. } => EditAddress::RoutePresence(route.id.clone()),
            Self::RemoveRoute { route } => EditAddress::RoutePresence(route.clone()),
            Self::SetRouteDefinition { route } => EditAddress::RouteDefinition(route.id.clone()),
            Self::MoveVia { via, .. } => EditAddress::ViaCenter(via.clone()),
            Self::SetViaNet { via, .. } => EditAddress::ViaNet(via.clone()),
            Self::InsertVia { via, .. } => EditAddress::ViaPresence(via.id.clone()),
            Self::RemoveVia { via } => EditAddress::ViaPresence(via.clone()),
            Self::SetViaDefinition { via } => EditAddress::ViaDefinition(via.id.clone()),
            Self::SetZoneBoundary { zone, .. } => EditAddress::ZoneBoundary(zone.clone()),
            Self::SetZoneNet { zone, .. } => EditAddress::ZoneNet(zone.clone()),
            Self::InsertZone { zone, .. } => EditAddress::ZonePresence(zone.id.clone()),
            Self::RemoveZone { zone } => EditAddress::ZonePresence(zone.clone()),
            Self::SetZoneDefinition { zone } => EditAddress::ZoneDefinition(zone.id.clone()),
        }
    }
}

/// One atomic, revision-checked editor transaction.
#[derive(Clone, Debug, PartialEq, serde::Deserialize, serde::Serialize)]
pub struct DesignEditBatch {
    /// Stable batch identity for logs, retries, and collaboration transport.
    pub id: DesignEditId,
    /// Revision the editor observed before authoring this batch.
    pub expected_revision: DesignRevision,
    /// Nonempty editor, tool, or integration identity.
    pub editor: String,
    /// Ordered edits applied atomically.
    pub edits: Vec<DesignEdit>,
}

/// Evidence returned only after an atomic edit commit.
#[derive(Clone, Debug, Eq, PartialEq, serde::Deserialize, serde::Serialize)]
pub struct EditReplayReport {
    /// Stable committed batch identity.
    pub batch: DesignEditId,
    /// Revision on which the batch replayed.
    pub from_revision: DesignRevision,
    /// Revision assigned after commit.
    pub to_revision: DesignRevision,
    /// Targets applied in authored order.
    pub applied_targets: Vec<EditTarget>,
    /// Independently mergeable properties applied in authored order.
    #[serde(default)]
    pub applied_addresses: Vec<EditAddress>,
}

/// Why one replay occurred in a retained editor history.
#[derive(Clone, Copy, Debug, Eq, PartialEq, serde::Deserialize, serde::Serialize)]
pub enum DesignHistoryAction {
    /// A newly authored batch committed.
    Commit,
    /// A generated inverse batch committed.
    Undo,
    /// A previously undone forward batch committed again.
    Redo,
}

/// Monotonic revision evidence for one history operation.
#[derive(Clone, Debug, Eq, PartialEq, serde::Deserialize, serde::Serialize)]
pub struct DesignHistoryReplayReport {
    /// History operation.
    pub action: DesignHistoryAction,
    /// Original authored batch whose state changed.
    pub original_batch: DesignEditId,
    /// Ordinary atomic replay evidence.
    pub replay: EditReplayReport,
}

/// Evidence for a concurrent commit, including any optimistic rebase.
#[derive(Clone, Debug, Eq, PartialEq, serde::Deserialize, serde::Serialize)]
pub struct ConcurrentCommitReport {
    /// Revision observed when the incoming batch was authored.
    pub authored_revision: DesignRevision,
    /// Revision on which the batch actually replayed.
    pub replay_revision: DesignRevision,
    /// Ordinary commit and replay evidence.
    pub commit: DesignHistoryReplayReport,
}

/// Evidence describing an automatic editor-history schema upgrade.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct DesignHistoryMigrationReport {
    /// Schema revision present in the input JSON.
    pub from_version: u32,
    /// Current schema revision assigned after decoding.
    pub to_version: u32,
}

/// One reversible committed transaction retained on undo/redo stacks.
#[derive(Clone, Debug, PartialEq, serde::Deserialize, serde::Serialize)]
pub struct ReversibleDesignEdit {
    /// Original authored forward transaction.
    pub forward: DesignEditBatch,
    /// Generated inverse transaction in reverse edit order.
    pub inverse: DesignEditBatch,
}

/// Serializable validated document plus monotonic undo/redo stacks.
#[derive(Clone, Debug, PartialEq, serde::Deserialize, serde::Serialize)]
#[serde(deny_unknown_fields)]
pub struct DesignHistory {
    /// Stable history schema family.
    pub schema: String,
    /// History schema revision.
    pub version: u32,
    /// Current retained semantic document.
    document: SemanticDocument,
    /// Earliest revision for which `replay_log` can prove intervening writes.
    #[serde(default)]
    replay_base_revision: DesignRevision,
    /// Contiguous append-only replay evidence after `replay_base_revision`.
    #[serde(default)]
    replay_log: Vec<EditReplayReport>,
    /// Transactions available to undo, oldest to newest.
    undo: Vec<ReversibleDesignEdit>,
    /// Transactions available to redo, oldest to newest.
    redo: Vec<ReversibleDesignEdit>,
}

/// Typed refusal to apply an editor batch.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum DesignEditError {
    /// Batch editor identity is blank.
    InvalidEditor,
    /// Empty transactions are not revisions.
    EmptyBatch,
    /// Another batch committed after the editor's observed revision.
    RevisionConflict {
        /// Revision carried by the batch.
        expected: DesignRevision,
        /// Current document revision.
        actual: DesignRevision,
    },
    /// The document has no retained schematic to edit.
    MissingSchematic,
    /// The document has no retained PCB to edit.
    MissingPcb,
    /// A stable target is absent from the retained PCB.
    MissingTarget(EditTarget),
    /// An insertion attempted to reuse an existing stable target.
    ExistingTarget(EditTarget),
    /// A generated or imported insertion position is outside the target list.
    InvalidInsertionIndex {
        /// Object that could not be inserted.
        target: EditTarget,
        /// Requested zero-based list position.
        index: usize,
        /// Current list length, which is also the largest valid insertion position.
        len: usize,
    },
    /// A touched placement conflicts with authored placement constraints.
    PlacementConstraintConflict(CircuitInstanceId),
    /// Constraint resolution itself found one or more unsatisfied predicates.
    UnsatisfiedPlacementConstraints { issue_count: usize },
    /// The edited clone failed ordinary semantic validation.
    InvalidResult(SemanticInterchangeError),
    /// The monotonic revision counter cannot advance.
    RevisionOverflow,
}

/// Typed refusal to construct or mutate a retained editor history.
#[derive(Clone, Debug, PartialEq)]
pub enum DesignHistoryError {
    /// Current document or decoded history is semantically invalid.
    InvalidDocument(SemanticInterchangeError),
    /// A committed/undo/redo batch failed atomic replay.
    Edit(DesignEditError),
    /// No committed transaction is available to undo.
    NothingToUndo,
    /// No reverted transaction is available to redo.
    NothingToRedo,
    /// A stale batch predates the retained replay evidence needed to rebase it.
    HistoryUnavailable {
        /// Revision on which the incoming batch was authored.
        expected: DesignRevision,
        /// Oldest revision still covered by retained replay evidence.
        oldest: DesignRevision,
    },
    /// One or more independently mergeable properties changed since authoring.
    MergeConflict {
        /// Revision on which the incoming batch was authored.
        expected: DesignRevision,
        /// Current document revision.
        actual: DesignRevision,
        /// Conflicting properties in incoming edit order, without duplicates.
        addresses: Vec<EditAddress>,
    },
    /// Serialized replay evidence is not contiguous with the current document.
    InvalidReplayLog,
    /// Serialized history JSON is malformed.
    Json(String),
    /// Serialized history belongs to another schema family or revision.
    UnsupportedSchema { schema: String, version: u32 },
}

impl Display for DesignEditError {
    fn fmt(&self, formatter: &mut Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::InvalidEditor => formatter.write_str("design edit batch editor is empty"),
            Self::EmptyBatch => formatter.write_str("design edit batch contains no edits"),
            Self::RevisionConflict { expected, actual } => write!(
                formatter,
                "design edit expected revision {}, but the document is at {}",
                expected.value(),
                actual.value()
            ),
            Self::MissingSchematic => {
                formatter.write_str("semantic document has no schematic to edit")
            }
            Self::MissingPcb => formatter.write_str("semantic document has no PCB to edit"),
            Self::MissingTarget(target) => write!(formatter, "missing edit target {target:?}"),
            Self::ExistingTarget(target) => {
                write!(formatter, "edit target already exists {target:?}")
            }
            Self::InvalidInsertionIndex { target, index, len } => write!(
                formatter,
                "cannot insert edit target {target:?} at index {index} in list of length {len}"
            ),
            Self::PlacementConstraintConflict(instance) => write!(
                formatter,
                "edited placement {} conflicts with retained constraints",
                instance.as_str()
            ),
            Self::UnsatisfiedPlacementConstraints { issue_count } => write!(
                formatter,
                "edited placement has {issue_count} unsatisfied constraint(s)"
            ),
            Self::InvalidResult(error) => write!(formatter, "edited document is invalid: {error}"),
            Self::RevisionOverflow => formatter.write_str("design revision overflow"),
        }
    }
}

impl std::error::Error for DesignEditError {}

impl Display for DesignHistoryError {
    fn fmt(&self, formatter: &mut Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::InvalidDocument(error) => write!(formatter, "invalid history document: {error}"),
            Self::Edit(error) => write!(formatter, "history replay failed: {error}"),
            Self::NothingToUndo => formatter.write_str("design history has nothing to undo"),
            Self::NothingToRedo => formatter.write_str("design history has nothing to redo"),
            Self::HistoryUnavailable { expected, oldest } => write!(
                formatter,
                "revision {} predates retained replay history starting at {}",
                expected.value(),
                oldest.value()
            ),
            Self::MergeConflict {
                expected,
                actual,
                addresses,
            } => write!(
                formatter,
                "concurrent edit from revision {} conflicts with {} field(s) changed by revision {}",
                expected.value(),
                addresses.len(),
                actual.value()
            ),
            Self::InvalidReplayLog => {
                formatter.write_str("design history replay log is not contiguous")
            }
            Self::Json(error) => write!(formatter, "design history JSON error: {error}"),
            Self::UnsupportedSchema { schema, version } => {
                write!(
                    formatter,
                    "unsupported design history schema {schema}@{version}"
                )
            }
        }
    }
}

impl std::error::Error for DesignHistoryError {}

impl SemanticDocument {
    /// Replays and atomically commits one revision-checked semantic edit batch.
    pub fn apply_edit_batch(
        &mut self,
        batch: &DesignEditBatch,
    ) -> Result<EditReplayReport, DesignEditError> {
        self.apply_edit_batch_with_inverse(batch)
            .map(|(report, _)| report)
    }

    fn apply_edit_batch_with_inverse(
        &mut self,
        batch: &DesignEditBatch,
    ) -> Result<(EditReplayReport, Vec<DesignEdit>), DesignEditError> {
        if batch.editor.trim().is_empty() {
            return Err(DesignEditError::InvalidEditor);
        }
        if batch.edits.is_empty() {
            return Err(DesignEditError::EmptyBatch);
        }
        if batch.expected_revision != self.design_revision {
            return Err(DesignEditError::RevisionConflict {
                expected: batch.expected_revision,
                actual: self.design_revision,
            });
        }
        let next_revision = self
            .design_revision
            .next()
            .ok_or(DesignEditError::RevisionOverflow)?;
        let mut candidate = self.clone();
        let mut targets = Vec::with_capacity(batch.edits.len());
        let mut addresses = Vec::with_capacity(batch.edits.len());
        let mut touched_placements = Vec::new();
        let mut touched_constraints = false;
        let mut inverse = Vec::with_capacity(batch.edits.len());
        for edit in &batch.edits {
            let target = edit.target();
            inverse.push(inverse_edit(&candidate, edit)?);
            apply_edit(&mut candidate, edit)?;
            if let EditTarget::Placement(instance) = &target {
                touched_placements.push(instance.clone());
            }
            if matches!(&target, EditTarget::PlacementConstraint(_)) {
                touched_constraints = true;
            }
            targets.push(target);
            addresses.push(edit.address());
        }
        candidate.design_revision = next_revision;
        candidate
            .validate()
            .map_err(DesignEditError::InvalidResult)?;
        if !touched_placements.is_empty() || touched_constraints {
            let pcb = candidate
                .pcb
                .as_ref()
                .expect("PCB existence checked before editing");
            let resolution = pcb.resolve_placement_constraints(&candidate.circuit);
            if !resolution.is_satisfied() {
                return Err(DesignEditError::UnsatisfiedPlacementConstraints {
                    issue_count: resolution.issues.len(),
                });
            }
            let placements_to_compare = if touched_constraints {
                pcb.placements
                    .iter()
                    .map(|placement| placement.instance.clone())
                    .collect::<Vec<_>>()
            } else {
                touched_placements
            };
            for instance in placements_to_compare {
                let Some(authored) = pcb
                    .placements
                    .iter()
                    .find(|placement| placement.instance == instance)
                else {
                    continue;
                };
                let resolved = resolution
                    .placements
                    .iter()
                    .find(|placement| placement.instance == instance)
                    .expect("resolved placement target must still exist");
                if authored != resolved {
                    return Err(DesignEditError::PlacementConstraintConflict(instance));
                }
            }
        }
        let report = EditReplayReport {
            batch: batch.id.clone(),
            from_revision: self.design_revision,
            to_revision: next_revision,
            applied_targets: targets,
            applied_addresses: addresses,
        };
        *self = candidate;
        inverse.reverse();
        Ok((report, inverse))
    }
}

impl DesignHistory {
    /// Starts an empty undo/redo history around one validated document.
    pub fn new(document: SemanticDocument) -> Result<Self, DesignHistoryError> {
        document
            .validate()
            .map_err(DesignHistoryError::InvalidDocument)?;
        Ok(Self {
            schema: DESIGN_HISTORY_SCHEMA.into(),
            version: DESIGN_HISTORY_VERSION,
            replay_base_revision: document.design_revision,
            replay_log: Vec::new(),
            document,
            undo: Vec::new(),
            redo: Vec::new(),
        })
    }

    /// Returns the current immutable semantic document.
    pub fn document(&self) -> &SemanticDocument {
        &self.document
    }

    /// Consumes history and returns its current semantic document.
    pub fn into_document(self) -> SemanticDocument {
        self.document
    }

    /// Number of available undo operations.
    pub fn undo_depth(&self) -> usize {
        self.undo.len()
    }

    /// Number of available redo operations.
    pub fn redo_depth(&self) -> usize {
        self.redo.len()
    }

    /// Earliest revision covered by retained replay evidence.
    pub fn replay_base_revision(&self) -> DesignRevision {
        self.replay_base_revision
    }

    /// Ordered replay evidence retained for collaborative rebasing.
    pub fn replay_log(&self) -> &[EditReplayReport] {
        &self.replay_log
    }

    /// Commits a new authored batch and clears the abandoned redo branch.
    pub fn commit(
        &mut self,
        batch: DesignEditBatch,
    ) -> Result<DesignHistoryReplayReport, DesignHistoryError> {
        let (replay, inverse_edits) = self
            .document
            .apply_edit_batch_with_inverse(&batch)
            .map_err(DesignHistoryError::Edit)?;
        let inverse = DesignEditBatch {
            id: DesignEditId::new(format!("undo/{}", batch.id.as_str()))
                .expect("qualified nonempty edit ids remain nonempty"),
            expected_revision: replay.to_revision,
            editor: format!("undo:{}", batch.editor),
            edits: inverse_edits,
        };
        self.undo.push(ReversibleDesignEdit {
            forward: batch.clone(),
            inverse,
        });
        self.redo.clear();
        self.replay_log.push(replay.clone());
        Ok(DesignHistoryReplayReport {
            action: DesignHistoryAction::Commit,
            original_batch: batch.id,
            replay,
        })
    }

    /// Commits the newest generated inverse as a fresh monotonic revision.
    pub fn undo(&mut self) -> Result<DesignHistoryReplayReport, DesignHistoryError> {
        let Some(entry) = self.undo.pop() else {
            return Err(DesignHistoryError::NothingToUndo);
        };
        let mut inverse = entry.inverse.clone();
        inverse.expected_revision = self.document.design_revision;
        let replay = match self.document.apply_edit_batch(&inverse) {
            Ok(replay) => replay,
            Err(error) => {
                self.undo.push(entry);
                return Err(DesignHistoryError::Edit(error));
            }
        };
        let original_batch = entry.forward.id.clone();
        self.redo.push(entry);
        self.replay_log.push(replay.clone());
        Ok(DesignHistoryReplayReport {
            action: DesignHistoryAction::Undo,
            original_batch,
            replay,
        })
    }

    /// Recommits the newest reverted forward batch as a fresh monotonic revision.
    pub fn redo(&mut self) -> Result<DesignHistoryReplayReport, DesignHistoryError> {
        let Some(entry) = self.redo.pop() else {
            return Err(DesignHistoryError::NothingToRedo);
        };
        let mut forward = entry.forward.clone();
        forward.expected_revision = self.document.design_revision;
        let replay = match self.document.apply_edit_batch(&forward) {
            Ok(replay) => replay,
            Err(error) => {
                self.redo.push(entry);
                return Err(DesignHistoryError::Edit(error));
            }
        };
        let original_batch = entry.forward.id.clone();
        self.undo.push(entry);
        self.replay_log.push(replay.clone());
        Ok(DesignHistoryReplayReport {
            action: DesignHistoryAction::Redo,
            original_batch,
            replay,
        })
    }

    /// Commits a possibly stale batch when every intervening write commutes.
    ///
    /// Conflicts are detected at [`EditAddress`] granularity. For example, a
    /// route-width edit can be rebased over a route-centerline edit, while two
    /// width edits to the same route conflict.
    pub fn commit_concurrent(
        &mut self,
        mut batch: DesignEditBatch,
    ) -> Result<ConcurrentCommitReport, DesignHistoryError> {
        let authored_revision = batch.expected_revision;
        let current_revision = self.document.design_revision;
        if authored_revision > current_revision {
            return Err(DesignHistoryError::Edit(
                DesignEditError::RevisionConflict {
                    expected: authored_revision,
                    actual: current_revision,
                },
            ));
        }
        if authored_revision < self.replay_base_revision {
            return Err(DesignHistoryError::HistoryUnavailable {
                expected: authored_revision,
                oldest: self.replay_base_revision,
            });
        }
        if authored_revision < current_revision {
            let incoming = batch
                .edits
                .iter()
                .map(DesignEdit::address)
                .collect::<Vec<_>>();
            let intervening = self
                .replay_log
                .iter()
                .filter(|report| report.to_revision > authored_revision)
                .flat_map(|report| report.applied_addresses.iter())
                .collect::<Vec<_>>();
            let mut conflicts = Vec::new();
            for address in incoming {
                if intervening
                    .iter()
                    .any(|written| address.conflicts_with(written))
                    && !conflicts.contains(&address)
                {
                    conflicts.push(address);
                }
            }
            if !conflicts.is_empty() {
                return Err(DesignHistoryError::MergeConflict {
                    expected: authored_revision,
                    actual: current_revision,
                    addresses: conflicts,
                });
            }
            batch.expected_revision = current_revision;
        }
        let commit = self.commit(batch)?;
        Ok(ConcurrentCommitReport {
            authored_revision,
            replay_revision: commit.replay.from_revision,
            commit,
        })
    }

    /// Discards old merge evidence while preserving document and undo stacks.
    ///
    /// Batches authored before the returned base revision can no longer be
    /// safely rebased and will receive [`DesignHistoryError::HistoryUnavailable`].
    pub fn compact_replay_log(&mut self) -> DesignRevision {
        self.replay_base_revision = self.document.design_revision;
        self.replay_log.clear();
        self.replay_base_revision
    }

    /// Encodes a validated current document and reversible stacks.
    pub fn to_json_pretty(&self) -> Result<String, DesignHistoryError> {
        self.validate()?;
        serde_json::to_string_pretty(self)
            .map_err(|error| DesignHistoryError::Json(error.to_string()))
    }

    /// Decodes and validates a retained reversible editor history.
    pub fn from_json(json: &str) -> Result<Self, DesignHistoryError> {
        Self::from_json_migrating(json).map(|(history, _)| history)
    }

    /// Decodes history and reports an automatic upgrade to the current revision.
    pub fn from_json_migrating(
        json: &str,
    ) -> Result<(Self, DesignHistoryMigrationReport), DesignHistoryError> {
        let mut value = serde_json::from_str::<serde_json::Value>(json)
            .map_err(|error| DesignHistoryError::Json(error.to_string()))?;
        let schema = value
            .get("schema")
            .and_then(serde_json::Value::as_str)
            .ok_or_else(|| DesignHistoryError::Json("history schema must be a string".into()))?
            .to_owned();
        let version = value
            .get("version")
            .and_then(serde_json::Value::as_u64)
            .and_then(|version| u32::try_from(version).ok())
            .ok_or_else(|| {
                DesignHistoryError::Json(
                    "history version must be an unsigned 32-bit integer".into(),
                )
            })?;
        if schema != DESIGN_HISTORY_SCHEMA
            || !(DESIGN_HISTORY_MIN_MIGRATABLE_VERSION..=DESIGN_HISTORY_VERSION).contains(&version)
        {
            return Err(DesignHistoryError::UnsupportedSchema { schema, version });
        }
        if version < 15 {
            migrate_history_reusable_schematic_symbols(&mut value)?;
        }
        value["version"] = serde_json::Value::from(DESIGN_HISTORY_VERSION);
        let mut history = serde_json::from_value::<Self>(value)
            .map_err(|error| DesignHistoryError::Json(error.to_string()))?;
        if version == 1 {
            history.replay_base_revision = history.document.design_revision;
            history.replay_log.clear();
        }
        history.validate()?;
        Ok((
            history,
            DesignHistoryMigrationReport {
                from_version: version,
                to_version: DESIGN_HISTORY_VERSION,
            },
        ))
    }

    fn validate(&self) -> Result<(), DesignHistoryError> {
        if self.schema != DESIGN_HISTORY_SCHEMA || self.version != DESIGN_HISTORY_VERSION {
            return Err(DesignHistoryError::UnsupportedSchema {
                schema: self.schema.clone(),
                version: self.version,
            });
        }
        self.document
            .validate()
            .map_err(DesignHistoryError::InvalidDocument)?;
        let mut cursor = self.replay_base_revision;
        for replay in &self.replay_log {
            if replay.from_revision != cursor
                || Some(replay.to_revision.value()) != replay.from_revision.value().checked_add(1)
                || replay.applied_addresses.len() != replay.applied_targets.len()
                || replay
                    .applied_addresses
                    .iter()
                    .zip(&replay.applied_targets)
                    .any(|(address, target)| address.target() != *target)
            {
                return Err(DesignHistoryError::InvalidReplayLog);
            }
            cursor = replay.to_revision;
        }
        if cursor != self.document.design_revision {
            return Err(DesignHistoryError::InvalidReplayLog);
        }
        Ok(())
    }
}

fn migrate_history_reusable_schematic_symbols(
    value: &mut serde_json::Value,
) -> Result<(), DesignHistoryError> {
    let document_value = value
        .get_mut("document")
        .ok_or_else(|| DesignHistoryError::Json("history document is missing".into()))?;
    let document_json = serde_json::to_string(&*document_value)
        .map_err(|error| DesignHistoryError::Json(error.to_string()))?;
    let document = SemanticDocument::from_json(&document_json)
        .map_err(|error| DesignHistoryError::Json(error.to_string()))?;
    *document_value = serde_json::to_value(document)
        .map_err(|error| DesignHistoryError::Json(error.to_string()))?;

    let models_by_instance = value
        .get("document")
        .and_then(|document| document.get("circuit"))
        .and_then(|circuit| circuit.get("instances"))
        .and_then(serde_json::Value::as_array)
        .into_iter()
        .flatten()
        .filter_map(|instance| {
            Some((
                instance.get("id")?.as_str()?.to_owned(),
                instance.get("model")?.as_str()?.to_owned(),
            ))
        })
        .collect::<std::collections::BTreeMap<_, _>>();
    let mut definitions = std::collections::BTreeMap::new();
    collect_legacy_history_symbol_definitions(value, &models_by_instance, &mut definitions)?;
    migrate_legacy_history_symbol_values(value, &definitions)
}

fn collect_legacy_history_symbol_definitions(
    value: &serde_json::Value,
    models_by_instance: &std::collections::BTreeMap<String, String>,
    definitions: &mut std::collections::BTreeMap<String, serde_json::Value>,
) -> Result<(), DesignHistoryError> {
    match value {
        serde_json::Value::Array(values) => {
            for value in values {
                collect_legacy_history_symbol_definitions(value, models_by_instance, definitions)?;
            }
        }
        serde_json::Value::Object(object) => {
            if is_legacy_history_symbol(object) {
                let symbol = object
                    .get("id")
                    .and_then(serde_json::Value::as_str)
                    .expect("legacy symbol predicate checked id");
                let instance = object
                    .get("instance")
                    .and_then(serde_json::Value::as_str)
                    .expect("legacy symbol predicate checked instance");
                let model = models_by_instance.get(instance).ok_or_else(|| {
                    DesignHistoryError::Json(format!(
                        "legacy history symbol {symbol} references absent instance {instance}"
                    ))
                })?;
                let definition = format!("legacy:{symbol}");
                definitions.entry(definition.clone()).or_insert_with(|| {
                    serde_json::json!({
                        "id": definition,
                        "model": model,
                        "name": symbol,
                        "units": [{
                            "unit": object.get("unit").cloned().unwrap_or_else(|| serde_json::Value::from(1)),
                            "body_width": object["body_width"].clone(),
                            "body_height": object["body_height"].clone(),
                            "pins": object["pins"].clone(),
                            "graphics": [],
                        }],
                    })
                });
            }
            for value in object.values() {
                collect_legacy_history_symbol_definitions(value, models_by_instance, definitions)?;
            }
        }
        _ => {}
    }
    Ok(())
}

fn migrate_legacy_history_symbol_values(
    value: &mut serde_json::Value,
    definitions: &std::collections::BTreeMap<String, serde_json::Value>,
) -> Result<(), DesignHistoryError> {
    match value {
        serde_json::Value::Array(values) => {
            for value in values {
                migrate_legacy_history_symbol_values(value, definitions)?;
            }
        }
        serde_json::Value::Object(object) => {
            if is_legacy_history_symbol(object) {
                let symbol = object
                    .get("id")
                    .and_then(serde_json::Value::as_str)
                    .expect("legacy symbol predicate checked id")
                    .to_owned();
                object.remove("body_width");
                object.remove("body_height");
                object.remove("pins");
                object.insert(
                    "definition".into(),
                    serde_json::Value::String(format!("legacy:{symbol}")),
                );
            }
            for value in object.values_mut() {
                migrate_legacy_history_symbol_values(value, definitions)?;
            }
            if is_history_schematic_layout(object) {
                let library = object
                    .entry("symbol_definitions")
                    .or_insert_with(|| serde_json::Value::Array(Vec::new()))
                    .as_array_mut()
                    .ok_or_else(|| {
                        DesignHistoryError::Json(
                            "history schematic symbol_definitions must be an array".into(),
                        )
                    })?;
                let mut existing = library
                    .iter()
                    .filter_map(|definition| definition.get("id")?.as_str())
                    .map(str::to_owned)
                    .collect::<std::collections::BTreeSet<_>>();
                for (id, definition) in definitions {
                    if existing.insert(id.clone()) {
                        library.push(definition.clone());
                    }
                }
            }
        }
        _ => {}
    }
    Ok(())
}

fn is_legacy_history_symbol(object: &serde_json::Map<String, serde_json::Value>) -> bool {
    [
        "id",
        "instance",
        "unit",
        "position",
        "quarter_turns",
        "body_width",
        "body_height",
        "pins",
    ]
    .into_iter()
    .all(|field| object.contains_key(field))
}

fn is_history_schematic_layout(object: &serde_json::Map<String, serde_json::Value>) -> bool {
    [
        "symbols",
        "ports",
        "wires",
        "labels",
        "sheets",
        "sheet_ports",
        "sheet_links",
    ]
    .into_iter()
    .all(|field| object.contains_key(field))
}

fn inverse_edit(
    document: &SemanticDocument,
    edit: &DesignEdit,
) -> Result<DesignEdit, DesignEditError> {
    Ok(match edit {
        DesignEdit::InsertNet { net, .. } => {
            if document
                .circuit
                .nets
                .iter()
                .any(|existing| existing.id == net.id)
            {
                return Err(DesignEditError::ExistingTarget(EditTarget::Net(
                    net.id.clone(),
                )));
            }
            DesignEdit::RemoveNet {
                net: net.id.clone(),
            }
        }
        DesignEdit::RemoveNet { net } => {
            let (index, existing) = document
                .circuit
                .nets
                .iter()
                .enumerate()
                .find(|(_, candidate)| candidate.id == *net)
                .ok_or_else(|| DesignEditError::MissingTarget(EditTarget::Net(net.clone())))?;
            DesignEdit::InsertNet {
                net: existing.clone(),
                index: Some(index),
            }
        }
        DesignEdit::SetNetGround { net, .. } => {
            let existing = document
                .circuit
                .nets
                .iter()
                .find(|candidate| candidate.id == *net)
                .ok_or_else(|| DesignEditError::MissingTarget(EditTarget::Net(net.clone())))?;
            DesignEdit::SetNetGround {
                net: net.clone(),
                is_ground: existing.is_ground,
            }
        }
        DesignEdit::InsertBus { bus, .. } => {
            if document
                .circuit
                .buses
                .iter()
                .any(|existing| existing.id == bus.id)
            {
                return Err(DesignEditError::ExistingTarget(EditTarget::Bus(
                    bus.id.clone(),
                )));
            }
            DesignEdit::RemoveBus {
                bus: bus.id.clone(),
            }
        }
        DesignEdit::RemoveBus { bus } => {
            let (index, existing) = document
                .circuit
                .buses
                .iter()
                .enumerate()
                .find(|(_, candidate)| candidate.id == *bus)
                .ok_or_else(|| DesignEditError::MissingTarget(EditTarget::Bus(bus.clone())))?;
            DesignEdit::InsertBus {
                bus: existing.clone(),
                index: Some(index),
            }
        }
        DesignEdit::SetBusNets { bus, .. } => {
            let existing = document
                .circuit
                .buses
                .iter()
                .find(|candidate| candidate.id == *bus)
                .ok_or_else(|| DesignEditError::MissingTarget(EditTarget::Bus(bus.clone())))?;
            DesignEdit::SetBusNets {
                bus: bus.clone(),
                nets: existing.nets.clone(),
            }
        }
        DesignEdit::InsertBusSlice { slice, .. } => {
            if document
                .circuit
                .bus_slices
                .iter()
                .any(|existing| existing.id == slice.id)
            {
                return Err(DesignEditError::ExistingTarget(EditTarget::BusSlice(
                    slice.id.clone(),
                )));
            }
            DesignEdit::RemoveBusSlice {
                slice: slice.id.clone(),
            }
        }
        DesignEdit::RemoveBusSlice { slice } => {
            let (index, existing) = document
                .circuit
                .bus_slices
                .iter()
                .enumerate()
                .find(|(_, candidate)| candidate.id == *slice)
                .ok_or_else(|| {
                    DesignEditError::MissingTarget(EditTarget::BusSlice(slice.clone()))
                })?;
            DesignEdit::InsertBusSlice {
                slice: existing.clone(),
                index: Some(index),
            }
        }
        DesignEdit::SetBusSliceDefinition { slice, .. } => {
            let existing = document
                .circuit
                .bus_slices
                .iter()
                .find(|candidate| candidate.id == *slice)
                .ok_or_else(|| {
                    DesignEditError::MissingTarget(EditTarget::BusSlice(slice.clone()))
                })?;
            DesignEdit::SetBusSliceDefinition {
                slice: slice.clone(),
                bus: existing.bus.clone(),
                offset: existing.offset,
                width: existing.width,
                order: existing.order,
            }
        }
        DesignEdit::InsertCircuitPort { port, .. } => {
            if document
                .circuit
                .ports
                .iter()
                .any(|existing| existing.id == port.id)
            {
                return Err(DesignEditError::ExistingTarget(EditTarget::CircuitPort(
                    port.id.clone(),
                )));
            }
            DesignEdit::RemoveCircuitPort {
                port: port.id.clone(),
            }
        }
        DesignEdit::RemoveCircuitPort { port } => {
            let (index, existing) = document
                .circuit
                .ports
                .iter()
                .enumerate()
                .find(|(_, candidate)| candidate.id == *port)
                .ok_or_else(|| {
                    DesignEditError::MissingTarget(EditTarget::CircuitPort(port.clone()))
                })?;
            DesignEdit::InsertCircuitPort {
                port: existing.clone(),
                index: Some(index),
            }
        }
        DesignEdit::SetCircuitPortDefinition { port, .. } => {
            let existing = document
                .circuit
                .ports
                .iter()
                .find(|candidate| candidate.id == *port)
                .ok_or_else(|| {
                    DesignEditError::MissingTarget(EditTarget::CircuitPort(port.clone()))
                })?;
            DesignEdit::SetCircuitPortDefinition {
                port: port.clone(),
                net: existing.net.clone(),
                direction: existing.direction,
                optional: existing.optional,
            }
        }
        DesignEdit::InsertRail { rail, .. } => {
            if document
                .circuit
                .rails
                .iter()
                .any(|existing| existing.net == rail.net)
            {
                return Err(DesignEditError::ExistingTarget(EditTarget::Rail(
                    rail.net.clone(),
                )));
            }
            DesignEdit::RemoveRail {
                net: rail.net.clone(),
            }
        }
        DesignEdit::RemoveRail { net } => {
            let (index, existing) = document
                .circuit
                .rails
                .iter()
                .enumerate()
                .find(|(_, candidate)| candidate.net == *net)
                .ok_or_else(|| DesignEditError::MissingTarget(EditTarget::Rail(net.clone())))?;
            DesignEdit::InsertRail {
                rail: existing.clone(),
                index: Some(index),
            }
        }
        DesignEdit::SetRailDefinition { net, .. } => {
            let existing = document
                .circuit
                .rails
                .iter()
                .find(|candidate| candidate.net == *net)
                .ok_or_else(|| DesignEditError::MissingTarget(EditTarget::Rail(net.clone())))?;
            DesignEdit::SetRailDefinition {
                net: net.clone(),
                nominal_voltage: existing.nominal_voltage.clone(),
                max_current: existing.max_current.clone(),
                kind: existing.kind.clone(),
            }
        }
        DesignEdit::InsertModuleParameter { parameter, .. } => {
            if document
                .circuit
                .module_parameters
                .iter()
                .any(|existing| existing.name == parameter.name)
            {
                return Err(DesignEditError::ExistingTarget(
                    EditTarget::ModuleParameter(parameter.name.clone()),
                ));
            }
            DesignEdit::RemoveModuleParameter {
                parameter: parameter.name.clone(),
            }
        }
        DesignEdit::RemoveModuleParameter { parameter } => {
            let (index, existing) = document
                .circuit
                .module_parameters
                .iter()
                .enumerate()
                .find(|(_, candidate)| candidate.name == *parameter)
                .ok_or_else(|| {
                    DesignEditError::MissingTarget(EditTarget::ModuleParameter(parameter.clone()))
                })?;
            DesignEdit::InsertModuleParameter {
                parameter: existing.clone(),
                index: Some(index),
            }
        }
        DesignEdit::SetModuleParameterDefinition { parameter, .. } => {
            let existing = document
                .circuit
                .module_parameters
                .iter()
                .find(|candidate| candidate.name == *parameter)
                .ok_or_else(|| {
                    DesignEditError::MissingTarget(EditTarget::ModuleParameter(parameter.clone()))
                })?;
            DesignEdit::SetModuleParameterDefinition {
                parameter: parameter.clone(),
                default: existing.default.clone(),
                unit: existing.unit.clone(),
                source: existing.source.clone(),
                targets: existing.targets.clone(),
            }
        }
        DesignEdit::InsertSourceStimulus { stimulus, .. } => {
            if document
                .circuit
                .source_stimuli
                .iter()
                .any(|existing| existing.component == stimulus.component)
            {
                return Err(DesignEditError::ExistingTarget(EditTarget::SourceStimulus(
                    stimulus.component.clone(),
                )));
            }
            DesignEdit::RemoveSourceStimulus {
                component: stimulus.component.clone(),
            }
        }
        DesignEdit::RemoveSourceStimulus { component } => {
            let (index, existing) = document
                .circuit
                .source_stimuli
                .iter()
                .enumerate()
                .find(|(_, candidate)| candidate.component == *component)
                .ok_or_else(|| {
                    DesignEditError::MissingTarget(EditTarget::SourceStimulus(component.clone()))
                })?;
            DesignEdit::InsertSourceStimulus {
                stimulus: existing.clone(),
                index: Some(index),
            }
        }
        DesignEdit::SetSourceStimulusWaveform { component, .. } => {
            let existing = document
                .circuit
                .source_stimuli
                .iter()
                .find(|candidate| candidate.component == *component)
                .ok_or_else(|| {
                    DesignEditError::MissingTarget(EditTarget::SourceStimulus(component.clone()))
                })?;
            DesignEdit::SetSourceStimulusWaveform {
                component: component.clone(),
                waveform: existing.waveform.clone(),
            }
        }
        DesignEdit::InsertSubcircuit { subcircuit, .. } => {
            if document
                .circuit
                .subcircuits
                .iter()
                .any(|existing| existing.id == subcircuit.id)
            {
                return Err(DesignEditError::ExistingTarget(EditTarget::Subcircuit(
                    subcircuit.id.clone(),
                )));
            }
            DesignEdit::RemoveSubcircuit {
                subcircuit: subcircuit.id.clone(),
            }
        }
        DesignEdit::RemoveSubcircuit { subcircuit } => {
            let (index, existing) = document
                .circuit
                .subcircuits
                .iter()
                .enumerate()
                .find(|(_, candidate)| candidate.id == *subcircuit)
                .ok_or_else(|| {
                    DesignEditError::MissingTarget(EditTarget::Subcircuit(subcircuit.clone()))
                })?;
            DesignEdit::InsertSubcircuit {
                subcircuit: existing.clone(),
                index: Some(index),
            }
        }
        DesignEdit::SetSubcircuitDefinition { subcircuit, .. } => {
            let existing = document
                .circuit
                .subcircuits
                .iter()
                .find(|candidate| candidate.id == *subcircuit)
                .ok_or_else(|| {
                    DesignEditError::MissingTarget(EditTarget::Subcircuit(subcircuit.clone()))
                })?;
            DesignEdit::SetSubcircuitDefinition {
                subcircuit: subcircuit.clone(),
                circuit: existing.circuit.clone(),
                ports: existing.ports.clone(),
                parameter_overrides: existing.parameter_overrides.clone(),
            }
        }
        DesignEdit::SetLinearStamps { .. } => DesignEdit::SetLinearStamps {
            stamps: document.circuit.stamps.clone(),
        },
        DesignEdit::SetCircuitPolicy { .. } => DesignEdit::SetCircuitPolicy {
            transient_policy: document.circuit.transient_policy.clone(),
            adapter_policy: document.circuit.adapter_policy.clone(),
        },
        DesignEdit::InsertDeviceModel { model, .. } => {
            if document
                .circuit
                .device_models
                .iter()
                .any(|existing| existing.id == model.id)
            {
                return Err(DesignEditError::ExistingTarget(EditTarget::DeviceModel(
                    model.id.clone(),
                )));
            }
            DesignEdit::RemoveDeviceModel {
                model: model.id.clone(),
            }
        }
        DesignEdit::RemoveDeviceModel { model } => {
            let (index, existing) = document
                .circuit
                .device_models
                .iter()
                .enumerate()
                .find(|(_, candidate)| candidate.id == *model)
                .ok_or_else(|| {
                    DesignEditError::MissingTarget(EditTarget::DeviceModel(model.clone()))
                })?;
            DesignEdit::InsertDeviceModel {
                model: Box::new(existing.clone()),
                index: Some(index),
            }
        }
        DesignEdit::SetDeviceModelDefinition { model } => {
            let existing = document
                .circuit
                .device_models
                .iter()
                .find(|candidate| candidate.id == model.id)
                .ok_or_else(|| {
                    DesignEditError::MissingTarget(EditTarget::DeviceModel(model.id.clone()))
                })?;
            DesignEdit::SetDeviceModelDefinition {
                model: Box::new(existing.clone()),
            }
        }
        DesignEdit::InsertCircuitInstance { instance, .. } => {
            if document
                .circuit
                .instances
                .iter()
                .any(|existing| existing.id == instance.id)
            {
                return Err(DesignEditError::ExistingTarget(
                    EditTarget::CircuitInstance(instance.id.clone()),
                ));
            }
            DesignEdit::RemoveCircuitInstance {
                instance: instance.id.clone(),
            }
        }
        DesignEdit::RemoveCircuitInstance { instance } => {
            let (index, existing) = document
                .circuit
                .instances
                .iter()
                .enumerate()
                .find(|(_, candidate)| candidate.id == *instance)
                .ok_or_else(|| {
                    DesignEditError::MissingTarget(EditTarget::CircuitInstance(instance.clone()))
                })?;
            DesignEdit::InsertCircuitInstance {
                instance: Box::new(existing.clone()),
                index: Some(index),
            }
        }
        DesignEdit::SetCircuitInstanceDefinition { instance } => {
            let existing = document
                .circuit
                .instances
                .iter()
                .find(|candidate| candidate.id == instance.id)
                .ok_or_else(|| {
                    DesignEditError::MissingTarget(EditTarget::CircuitInstance(instance.id.clone()))
                })?;
            DesignEdit::SetCircuitInstanceDefinition {
                instance: Box::new(existing.clone()),
            }
        }
        DesignEdit::SetCircuitInstanceModel { instance, .. } => {
            let existing = document
                .circuit
                .instances
                .iter()
                .find(|candidate| candidate.id == *instance)
                .ok_or_else(|| {
                    DesignEditError::MissingTarget(EditTarget::CircuitInstance(instance.clone()))
                })?;
            DesignEdit::SetCircuitInstanceModel {
                instance: instance.clone(),
                model: existing.model.clone(),
            }
        }
        DesignEdit::SetCircuitInstancePins { instance, .. } => {
            let existing = document
                .circuit
                .instances
                .iter()
                .find(|candidate| candidate.id == *instance)
                .ok_or_else(|| {
                    DesignEditError::MissingTarget(EditTarget::CircuitInstance(instance.clone()))
                })?;
            DesignEdit::SetCircuitInstancePins {
                instance: instance.clone(),
                pins: existing.pins.clone(),
            }
        }
        DesignEdit::InsertPcb { pcb } => {
            if let Some(existing) = &document.pcb {
                return Err(DesignEditError::ExistingTarget(EditTarget::Pcb(
                    existing.id.clone(),
                )));
            }
            DesignEdit::RemovePcb {
                board: pcb.id.clone(),
            }
        }
        DesignEdit::RemovePcb { board } => {
            let pcb = document.pcb.as_ref().ok_or(DesignEditError::MissingPcb)?;
            if pcb.id != *board {
                return Err(DesignEditError::MissingTarget(EditTarget::Pcb(
                    board.clone(),
                )));
            }
            DesignEdit::InsertPcb {
                pcb: Box::new(pcb.clone()),
            }
        }
        DesignEdit::InsertSchematic { .. }
        | DesignEdit::RemoveSchematic
        | DesignEdit::InsertSchematicSymbol { .. }
        | DesignEdit::RemoveSchematicSymbol { .. }
        | DesignEdit::SetSchematicSymbolDefinition { .. }
        | DesignEdit::InsertSchematicSymbolLibrary { .. }
        | DesignEdit::RemoveSchematicSymbolLibrary { .. }
        | DesignEdit::SetSchematicSymbolLibraryDefinition { .. }
        | DesignEdit::InsertSchematicPortPlacement { .. }
        | DesignEdit::RemoveSchematicPortPlacement { .. }
        | DesignEdit::SetSchematicPortPlacementDefinition { .. }
        | DesignEdit::InsertSchematicWire { .. }
        | DesignEdit::RemoveSchematicWire { .. }
        | DesignEdit::SetSchematicWireDefinition { .. }
        | DesignEdit::InsertSchematicLabel { .. }
        | DesignEdit::RemoveSchematicLabel { .. }
        | DesignEdit::SetSchematicLabelDefinition { .. }
        | DesignEdit::InsertSchematicSheet { .. }
        | DesignEdit::RemoveSchematicSheet { .. }
        | DesignEdit::SetSchematicSheetDefinition { .. }
        | DesignEdit::InsertSchematicSheetPort { .. }
        | DesignEdit::RemoveSchematicSheetPort { .. }
        | DesignEdit::SetSchematicSheetPortDefinition { .. }
        | DesignEdit::InsertSchematicSheetLink { .. }
        | DesignEdit::RemoveSchematicSheetLink { .. }
        | DesignEdit::SetSchematicSheetLinkDefinition { .. } => {
            return inverse_schematic_edit(document, edit);
        }
        _ => {
            let layout = document.pcb.as_ref().ok_or(DesignEditError::MissingPcb)?;
            return inverse_layout_edit(layout, edit);
        }
    })
}

fn inverse_schematic_edit(
    document: &SemanticDocument,
    edit: &DesignEdit,
) -> Result<DesignEdit, DesignEditError> {
    if let DesignEdit::InsertSchematic { .. } = edit {
        if document.schematic.is_some() {
            return Err(DesignEditError::ExistingTarget(EditTarget::Schematic));
        }
        return Ok(DesignEdit::RemoveSchematic);
    }
    if matches!(edit, DesignEdit::RemoveSchematic) {
        let schematic = document
            .schematic
            .as_ref()
            .ok_or(DesignEditError::MissingSchematic)?;
        return Ok(DesignEdit::InsertSchematic {
            schematic: Box::new(schematic.clone()),
        });
    }

    let schematic = document
        .schematic
        .as_ref()
        .ok_or(DesignEditError::MissingSchematic)?;
    Ok(match edit {
        DesignEdit::InsertSchematicSymbol { symbol, .. } => {
            if schematic
                .symbols
                .iter()
                .any(|existing| existing.id == symbol.id)
            {
                return Err(DesignEditError::ExistingTarget(
                    EditTarget::SchematicSymbol(symbol.id.clone()),
                ));
            }
            DesignEdit::RemoveSchematicSymbol {
                symbol: symbol.id.clone(),
            }
        }
        DesignEdit::RemoveSchematicSymbol { symbol } => {
            let (index, existing) = schematic
                .symbols
                .iter()
                .enumerate()
                .find(|(_, candidate)| candidate.id == *symbol)
                .ok_or_else(|| {
                    DesignEditError::MissingTarget(EditTarget::SchematicSymbol(symbol.clone()))
                })?;
            DesignEdit::InsertSchematicSymbol {
                symbol: Box::new(existing.clone()),
                index: Some(index),
            }
        }
        DesignEdit::SetSchematicSymbolDefinition { symbol } => {
            let existing = schematic
                .symbols
                .iter()
                .find(|candidate| candidate.id == symbol.id)
                .ok_or_else(|| {
                    DesignEditError::MissingTarget(EditTarget::SchematicSymbol(symbol.id.clone()))
                })?;
            DesignEdit::SetSchematicSymbolDefinition {
                symbol: Box::new(existing.clone()),
            }
        }
        DesignEdit::InsertSchematicSymbolLibrary { definition, .. } => {
            if schematic
                .symbol_definitions
                .iter()
                .any(|existing| existing.id == definition.id)
            {
                return Err(DesignEditError::ExistingTarget(
                    EditTarget::SchematicSymbolLibrary(definition.id.clone()),
                ));
            }
            DesignEdit::RemoveSchematicSymbolLibrary {
                definition: definition.id.clone(),
            }
        }
        DesignEdit::RemoveSchematicSymbolLibrary { definition } => {
            let (index, existing) = schematic
                .symbol_definitions
                .iter()
                .enumerate()
                .find(|(_, candidate)| candidate.id == *definition)
                .ok_or_else(|| {
                    DesignEditError::MissingTarget(EditTarget::SchematicSymbolLibrary(
                        definition.clone(),
                    ))
                })?;
            DesignEdit::InsertSchematicSymbolLibrary {
                definition: Box::new(existing.clone()),
                index: Some(index),
            }
        }
        DesignEdit::SetSchematicSymbolLibraryDefinition { definition } => {
            let existing = schematic
                .symbol_definitions
                .iter()
                .find(|candidate| candidate.id == definition.id)
                .ok_or_else(|| {
                    DesignEditError::MissingTarget(EditTarget::SchematicSymbolLibrary(
                        definition.id.clone(),
                    ))
                })?;
            DesignEdit::SetSchematicSymbolLibraryDefinition {
                definition: Box::new(existing.clone()),
            }
        }
        DesignEdit::InsertSchematicPortPlacement { placement, .. } => {
            if schematic
                .ports
                .iter()
                .any(|existing| existing.port == placement.port)
            {
                return Err(DesignEditError::ExistingTarget(
                    EditTarget::SchematicPortPlacement(placement.port.clone()),
                ));
            }
            DesignEdit::RemoveSchematicPortPlacement {
                port: placement.port.clone(),
            }
        }
        DesignEdit::RemoveSchematicPortPlacement { port } => {
            let (index, existing) = schematic
                .ports
                .iter()
                .enumerate()
                .find(|(_, candidate)| candidate.port == *port)
                .ok_or_else(|| {
                    DesignEditError::MissingTarget(EditTarget::SchematicPortPlacement(port.clone()))
                })?;
            DesignEdit::InsertSchematicPortPlacement {
                placement: existing.clone(),
                index: Some(index),
            }
        }
        DesignEdit::SetSchematicPortPlacementDefinition { placement } => {
            let existing = schematic
                .ports
                .iter()
                .find(|candidate| candidate.port == placement.port)
                .ok_or_else(|| {
                    DesignEditError::MissingTarget(EditTarget::SchematicPortPlacement(
                        placement.port.clone(),
                    ))
                })?;
            DesignEdit::SetSchematicPortPlacementDefinition {
                placement: existing.clone(),
            }
        }
        DesignEdit::InsertSchematicWire { wire, .. } => {
            if schematic
                .wires
                .iter()
                .any(|existing| existing.id == wire.id)
            {
                return Err(DesignEditError::ExistingTarget(EditTarget::SchematicWire(
                    wire.id.clone(),
                )));
            }
            DesignEdit::RemoveSchematicWire {
                wire: wire.id.clone(),
            }
        }
        DesignEdit::RemoveSchematicWire { wire } => {
            let (index, existing) = schematic
                .wires
                .iter()
                .enumerate()
                .find(|(_, candidate)| candidate.id == *wire)
                .ok_or_else(|| {
                    DesignEditError::MissingTarget(EditTarget::SchematicWire(wire.clone()))
                })?;
            DesignEdit::InsertSchematicWire {
                wire: Box::new(existing.clone()),
                index: Some(index),
            }
        }
        DesignEdit::SetSchematicWireDefinition { wire } => {
            let existing = schematic
                .wires
                .iter()
                .find(|candidate| candidate.id == wire.id)
                .ok_or_else(|| {
                    DesignEditError::MissingTarget(EditTarget::SchematicWire(wire.id.clone()))
                })?;
            DesignEdit::SetSchematicWireDefinition {
                wire: Box::new(existing.clone()),
            }
        }
        DesignEdit::InsertSchematicLabel { label, .. } => {
            if schematic
                .labels
                .iter()
                .any(|existing| existing.id == label.id)
            {
                return Err(DesignEditError::ExistingTarget(EditTarget::SchematicLabel(
                    label.id.clone(),
                )));
            }
            DesignEdit::RemoveSchematicLabel {
                label: label.id.clone(),
            }
        }
        DesignEdit::RemoveSchematicLabel { label } => {
            let (index, existing) = schematic
                .labels
                .iter()
                .enumerate()
                .find(|(_, candidate)| candidate.id == *label)
                .ok_or_else(|| {
                    DesignEditError::MissingTarget(EditTarget::SchematicLabel(label.clone()))
                })?;
            DesignEdit::InsertSchematicLabel {
                label: existing.clone(),
                index: Some(index),
            }
        }
        DesignEdit::SetSchematicLabelDefinition { label } => {
            let existing = schematic
                .labels
                .iter()
                .find(|candidate| candidate.id == label.id)
                .ok_or_else(|| {
                    DesignEditError::MissingTarget(EditTarget::SchematicLabel(label.id.clone()))
                })?;
            DesignEdit::SetSchematicLabelDefinition {
                label: existing.clone(),
            }
        }
        DesignEdit::InsertSchematicSheet { sheet, .. } => {
            if schematic
                .sheets
                .iter()
                .any(|existing| existing.id == sheet.id)
            {
                return Err(DesignEditError::ExistingTarget(EditTarget::SchematicSheet(
                    sheet.id.clone(),
                )));
            }
            DesignEdit::RemoveSchematicSheet {
                sheet: sheet.id.clone(),
            }
        }
        DesignEdit::RemoveSchematicSheet { sheet } => {
            let (index, existing) = schematic
                .sheets
                .iter()
                .enumerate()
                .find(|(_, candidate)| candidate.id == *sheet)
                .ok_or_else(|| {
                    DesignEditError::MissingTarget(EditTarget::SchematicSheet(sheet.clone()))
                })?;
            DesignEdit::InsertSchematicSheet {
                sheet: existing.clone(),
                index: Some(index),
            }
        }
        DesignEdit::SetSchematicSheetDefinition { sheet } => {
            let existing = schematic
                .sheets
                .iter()
                .find(|candidate| candidate.id == sheet.id)
                .ok_or_else(|| {
                    DesignEditError::MissingTarget(EditTarget::SchematicSheet(sheet.id.clone()))
                })?;
            DesignEdit::SetSchematicSheetDefinition {
                sheet: existing.clone(),
            }
        }
        DesignEdit::InsertSchematicSheetPort { port, .. } => {
            if schematic
                .sheet_ports
                .iter()
                .any(|existing| existing.id == port.id)
            {
                return Err(DesignEditError::ExistingTarget(
                    EditTarget::SchematicSheetPort(port.id.clone()),
                ));
            }
            DesignEdit::RemoveSchematicSheetPort {
                port: port.id.clone(),
            }
        }
        DesignEdit::RemoveSchematicSheetPort { port } => {
            let (index, existing) = schematic
                .sheet_ports
                .iter()
                .enumerate()
                .find(|(_, candidate)| candidate.id == *port)
                .ok_or_else(|| {
                    DesignEditError::MissingTarget(EditTarget::SchematicSheetPort(port.clone()))
                })?;
            DesignEdit::InsertSchematicSheetPort {
                port: existing.clone(),
                index: Some(index),
            }
        }
        DesignEdit::SetSchematicSheetPortDefinition { port } => {
            let existing = schematic
                .sheet_ports
                .iter()
                .find(|candidate| candidate.id == port.id)
                .ok_or_else(|| {
                    DesignEditError::MissingTarget(EditTarget::SchematicSheetPort(port.id.clone()))
                })?;
            DesignEdit::SetSchematicSheetPortDefinition {
                port: existing.clone(),
            }
        }
        DesignEdit::InsertSchematicSheetLink { link, .. } => {
            if schematic
                .sheet_links
                .iter()
                .any(|existing| existing.id == link.id)
            {
                return Err(DesignEditError::ExistingTarget(
                    EditTarget::SchematicSheetLink(link.id.clone()),
                ));
            }
            DesignEdit::RemoveSchematicSheetLink {
                link: link.id.clone(),
            }
        }
        DesignEdit::RemoveSchematicSheetLink { link } => {
            let (index, existing) = schematic
                .sheet_links
                .iter()
                .enumerate()
                .find(|(_, candidate)| candidate.id == *link)
                .ok_or_else(|| {
                    DesignEditError::MissingTarget(EditTarget::SchematicSheetLink(link.clone()))
                })?;
            DesignEdit::InsertSchematicSheetLink {
                link: existing.clone(),
                index: Some(index),
            }
        }
        DesignEdit::SetSchematicSheetLinkDefinition { link } => {
            let existing = schematic
                .sheet_links
                .iter()
                .find(|candidate| candidate.id == link.id)
                .ok_or_else(|| {
                    DesignEditError::MissingTarget(EditTarget::SchematicSheetLink(link.id.clone()))
                })?;
            DesignEdit::SetSchematicSheetLinkDefinition {
                link: existing.clone(),
            }
        }
        DesignEdit::InsertSchematic { .. } | DesignEdit::RemoveSchematic => {
            unreachable!("schematic container edits return before collection lookup")
        }
        _ => unreachable!("non-schematic edit dispatched to schematic inverse generation"),
    })
}

fn inverse_layout_edit(
    layout: &PcbLayout,
    edit: &DesignEdit,
) -> Result<DesignEdit, DesignEditError> {
    Ok(match edit {
        DesignEdit::SetBoardOutline { board, .. } => {
            if layout.id != *board {
                return Err(DesignEditError::MissingTarget(EditTarget::Board(
                    board.clone(),
                )));
            }
            DesignEdit::SetBoardOutline {
                board: board.clone(),
                outline: layout.outline.clone(),
            }
        }
        DesignEdit::SetBoardStackup { board, .. } => {
            if layout.id != *board {
                return Err(DesignEditError::MissingTarget(EditTarget::Board(
                    board.clone(),
                )));
            }
            DesignEdit::SetBoardStackup {
                board: board.clone(),
                stackup: layout.stackup.clone(),
            }
        }
        DesignEdit::SetBoardRules { board, .. } => {
            if layout.id != *board {
                return Err(DesignEditError::MissingTarget(EditTarget::Board(
                    board.clone(),
                )));
            }
            DesignEdit::SetBoardRules {
                board: board.clone(),
                rules: layout.rules.clone(),
            }
        }
        DesignEdit::InsertLandPattern { land_pattern, .. } => {
            if layout
                .land_patterns
                .iter()
                .any(|existing| existing.id == land_pattern.id)
            {
                return Err(DesignEditError::ExistingTarget(EditTarget::LandPattern(
                    land_pattern.id.clone(),
                )));
            }
            DesignEdit::RemoveLandPattern {
                land_pattern: land_pattern.id.clone(),
            }
        }
        DesignEdit::RemoveLandPattern { land_pattern } => {
            let (index, existing) = layout
                .land_patterns
                .iter()
                .enumerate()
                .find(|(_, candidate)| candidate.id == *land_pattern)
                .ok_or_else(|| {
                    DesignEditError::MissingTarget(EditTarget::LandPattern(land_pattern.clone()))
                })?;
            DesignEdit::InsertLandPattern {
                land_pattern: Box::new(existing.clone()),
                index: Some(index),
            }
        }
        DesignEdit::SetLandPatternDefinition { land_pattern } => {
            let existing = layout
                .land_patterns
                .iter()
                .find(|candidate| candidate.id == land_pattern.id)
                .ok_or_else(|| {
                    DesignEditError::MissingTarget(EditTarget::LandPattern(land_pattern.id.clone()))
                })?;
            DesignEdit::SetLandPatternDefinition {
                land_pattern: Box::new(existing.clone()),
            }
        }
        DesignEdit::InsertKeepout { keepout, .. } => {
            if layout
                .keepouts
                .iter()
                .any(|existing| existing.id == keepout.id)
            {
                return Err(DesignEditError::ExistingTarget(EditTarget::Keepout(
                    keepout.id.clone(),
                )));
            }
            DesignEdit::RemoveKeepout {
                keepout: keepout.id.clone(),
            }
        }
        DesignEdit::RemoveKeepout { keepout } => {
            let (index, existing) = layout
                .keepouts
                .iter()
                .enumerate()
                .find(|(_, candidate)| candidate.id == *keepout)
                .ok_or_else(|| {
                    DesignEditError::MissingTarget(EditTarget::Keepout(keepout.clone()))
                })?;
            DesignEdit::InsertKeepout {
                keepout: existing.clone(),
                index: Some(index),
            }
        }
        DesignEdit::SetKeepoutDefinition { keepout } => {
            let existing = layout
                .keepouts
                .iter()
                .find(|candidate| candidate.id == keepout.id)
                .ok_or_else(|| {
                    DesignEditError::MissingTarget(EditTarget::Keepout(keepout.id.clone()))
                })?;
            DesignEdit::SetKeepoutDefinition {
                keepout: existing.clone(),
            }
        }
        DesignEdit::SetPlacementTransform { instance, .. } => {
            let placement = layout
                .placements
                .iter()
                .find(|placement| placement.instance == *instance)
                .ok_or_else(|| {
                    DesignEditError::MissingTarget(EditTarget::Placement(instance.clone()))
                })?;
            DesignEdit::SetPlacementTransform {
                instance: instance.clone(),
                position: placement.position.clone(),
                rotation_degrees: placement.rotation_degrees.clone(),
                side: placement.side,
            }
        }
        DesignEdit::SetPlacementLandPattern { instance, .. } => {
            let placement = layout
                .placements
                .iter()
                .find(|placement| placement.instance == *instance)
                .ok_or_else(|| {
                    DesignEditError::MissingTarget(EditTarget::Placement(instance.clone()))
                })?;
            DesignEdit::SetPlacementLandPattern {
                instance: instance.clone(),
                land_pattern: placement.land_pattern.clone(),
            }
        }
        DesignEdit::InsertPlacement { placement, .. } => {
            if layout
                .placements
                .iter()
                .any(|existing| existing.instance == placement.instance)
            {
                return Err(DesignEditError::ExistingTarget(EditTarget::Placement(
                    placement.instance.clone(),
                )));
            }
            DesignEdit::RemovePlacement {
                instance: placement.instance.clone(),
            }
        }
        DesignEdit::RemovePlacement { instance } => {
            let (index, existing) = layout
                .placements
                .iter()
                .enumerate()
                .find(|(_, placement)| placement.instance == *instance)
                .ok_or_else(|| {
                    DesignEditError::MissingTarget(EditTarget::Placement(instance.clone()))
                })?;
            DesignEdit::InsertPlacement {
                placement: Box::new(existing.clone()),
                index: Some(index),
            }
        }
        DesignEdit::InsertPlacementConstraint { constraint, .. } => {
            if layout
                .placement_constraints
                .iter()
                .any(|existing| existing.id == constraint.id)
            {
                return Err(DesignEditError::ExistingTarget(
                    EditTarget::PlacementConstraint(constraint.id.clone()),
                ));
            }
            DesignEdit::RemovePlacementConstraint {
                constraint: constraint.id.clone(),
            }
        }
        DesignEdit::RemovePlacementConstraint { constraint } => {
            let (index, existing) = layout
                .placement_constraints
                .iter()
                .enumerate()
                .find(|(_, candidate)| candidate.id == *constraint)
                .ok_or_else(|| {
                    DesignEditError::MissingTarget(EditTarget::PlacementConstraint(
                        constraint.clone(),
                    ))
                })?;
            DesignEdit::InsertPlacementConstraint {
                constraint: Box::new(existing.clone()),
                index: Some(index),
            }
        }
        DesignEdit::SetPlacementConstraintDefinition { constraint } => {
            let existing = layout
                .placement_constraints
                .iter()
                .find(|candidate| candidate.id == constraint.id)
                .ok_or_else(|| {
                    DesignEditError::MissingTarget(EditTarget::PlacementConstraint(
                        constraint.id.clone(),
                    ))
                })?;
            DesignEdit::SetPlacementConstraintDefinition {
                constraint: Box::new(existing.clone()),
            }
        }
        DesignEdit::SetRouteWidth { route, .. } => {
            let existing = layout
                .routes
                .iter()
                .find(|candidate| candidate.id == *route)
                .ok_or_else(|| DesignEditError::MissingTarget(EditTarget::Route(route.clone())))?;
            DesignEdit::SetRouteWidth {
                route: route.clone(),
                width: existing.width.clone(),
            }
        }
        DesignEdit::SetRouteSegments { route, .. } => {
            let existing = layout
                .routes
                .iter()
                .find(|candidate| candidate.id == *route)
                .ok_or_else(|| DesignEditError::MissingTarget(EditTarget::Route(route.clone())))?;
            DesignEdit::SetRouteSegments {
                route: route.clone(),
                segments: existing.segments.clone(),
            }
        }
        DesignEdit::SetRouteNet { route, .. } => {
            let existing = layout
                .routes
                .iter()
                .find(|candidate| candidate.id == *route)
                .ok_or_else(|| DesignEditError::MissingTarget(EditTarget::Route(route.clone())))?;
            DesignEdit::SetRouteNet {
                route: route.clone(),
                net: existing.net.clone(),
            }
        }
        DesignEdit::InsertRoute { route, .. } => {
            if layout.routes.iter().any(|existing| existing.id == route.id) {
                return Err(DesignEditError::ExistingTarget(EditTarget::Route(
                    route.id.clone(),
                )));
            }
            DesignEdit::RemoveRoute {
                route: route.id.clone(),
            }
        }
        DesignEdit::RemoveRoute { route } => {
            let (index, existing) = layout
                .routes
                .iter()
                .enumerate()
                .find(|(_, candidate)| candidate.id == *route)
                .ok_or_else(|| DesignEditError::MissingTarget(EditTarget::Route(route.clone())))?;
            DesignEdit::InsertRoute {
                route: Box::new(existing.clone()),
                index: Some(index),
            }
        }
        DesignEdit::SetRouteDefinition { route } => {
            let existing = layout
                .routes
                .iter()
                .find(|candidate| candidate.id == route.id)
                .ok_or_else(|| {
                    DesignEditError::MissingTarget(EditTarget::Route(route.id.clone()))
                })?;
            DesignEdit::SetRouteDefinition {
                route: Box::new(existing.clone()),
            }
        }
        DesignEdit::MoveVia { via, .. } => {
            let existing = layout
                .vias
                .iter()
                .find(|candidate| candidate.id == *via)
                .ok_or_else(|| DesignEditError::MissingTarget(EditTarget::Via(via.clone())))?;
            DesignEdit::MoveVia {
                via: via.clone(),
                center: existing.center.clone(),
            }
        }
        DesignEdit::SetViaNet { via, .. } => {
            let existing = layout
                .vias
                .iter()
                .find(|candidate| candidate.id == *via)
                .ok_or_else(|| DesignEditError::MissingTarget(EditTarget::Via(via.clone())))?;
            DesignEdit::SetViaNet {
                via: via.clone(),
                net: existing.net.clone(),
            }
        }
        DesignEdit::InsertVia { via, .. } => {
            if layout.vias.iter().any(|existing| existing.id == via.id) {
                return Err(DesignEditError::ExistingTarget(EditTarget::Via(
                    via.id.clone(),
                )));
            }
            DesignEdit::RemoveVia {
                via: via.id.clone(),
            }
        }
        DesignEdit::RemoveVia { via } => {
            let (index, existing) = layout
                .vias
                .iter()
                .enumerate()
                .find(|(_, candidate)| candidate.id == *via)
                .ok_or_else(|| DesignEditError::MissingTarget(EditTarget::Via(via.clone())))?;
            DesignEdit::InsertVia {
                via: Box::new(existing.clone()),
                index: Some(index),
            }
        }
        DesignEdit::SetViaDefinition { via } => {
            let existing = layout
                .vias
                .iter()
                .find(|candidate| candidate.id == via.id)
                .ok_or_else(|| DesignEditError::MissingTarget(EditTarget::Via(via.id.clone())))?;
            DesignEdit::SetViaDefinition {
                via: Box::new(existing.clone()),
            }
        }
        DesignEdit::SetZoneBoundary { zone, .. } => {
            let existing = layout
                .zones
                .iter()
                .find(|candidate| candidate.id == *zone)
                .ok_or_else(|| DesignEditError::MissingTarget(EditTarget::Zone(zone.clone())))?;
            DesignEdit::SetZoneBoundary {
                zone: zone.clone(),
                boundary: existing.boundary.clone(),
            }
        }
        DesignEdit::SetZoneNet { zone, .. } => {
            let existing = layout
                .zones
                .iter()
                .find(|candidate| candidate.id == *zone)
                .ok_or_else(|| DesignEditError::MissingTarget(EditTarget::Zone(zone.clone())))?;
            DesignEdit::SetZoneNet {
                zone: zone.clone(),
                net: existing.net.clone(),
            }
        }
        DesignEdit::InsertZone { zone, .. } => {
            if layout.zones.iter().any(|existing| existing.id == zone.id) {
                return Err(DesignEditError::ExistingTarget(EditTarget::Zone(
                    zone.id.clone(),
                )));
            }
            DesignEdit::RemoveZone {
                zone: zone.id.clone(),
            }
        }
        DesignEdit::RemoveZone { zone } => {
            let (index, existing) = layout
                .zones
                .iter()
                .enumerate()
                .find(|(_, candidate)| candidate.id == *zone)
                .ok_or_else(|| DesignEditError::MissingTarget(EditTarget::Zone(zone.clone())))?;
            DesignEdit::InsertZone {
                zone: Box::new(existing.clone()),
                index: Some(index),
            }
        }
        DesignEdit::SetZoneDefinition { zone } => {
            let existing = layout
                .zones
                .iter()
                .find(|candidate| candidate.id == zone.id)
                .ok_or_else(|| DesignEditError::MissingTarget(EditTarget::Zone(zone.id.clone())))?;
            DesignEdit::SetZoneDefinition {
                zone: Box::new(existing.clone()),
            }
        }
        DesignEdit::InsertDeviceModel { .. }
        | DesignEdit::InsertNet { .. }
        | DesignEdit::RemoveNet { .. }
        | DesignEdit::SetNetGround { .. }
        | DesignEdit::InsertBus { .. }
        | DesignEdit::RemoveBus { .. }
        | DesignEdit::SetBusNets { .. }
        | DesignEdit::InsertBusSlice { .. }
        | DesignEdit::RemoveBusSlice { .. }
        | DesignEdit::SetBusSliceDefinition { .. }
        | DesignEdit::InsertCircuitPort { .. }
        | DesignEdit::RemoveCircuitPort { .. }
        | DesignEdit::SetCircuitPortDefinition { .. }
        | DesignEdit::InsertRail { .. }
        | DesignEdit::RemoveRail { .. }
        | DesignEdit::SetRailDefinition { .. }
        | DesignEdit::InsertModuleParameter { .. }
        | DesignEdit::RemoveModuleParameter { .. }
        | DesignEdit::SetModuleParameterDefinition { .. }
        | DesignEdit::InsertSourceStimulus { .. }
        | DesignEdit::RemoveSourceStimulus { .. }
        | DesignEdit::SetSourceStimulusWaveform { .. }
        | DesignEdit::InsertSubcircuit { .. }
        | DesignEdit::RemoveSubcircuit { .. }
        | DesignEdit::SetSubcircuitDefinition { .. }
        | DesignEdit::SetLinearStamps { .. }
        | DesignEdit::SetCircuitPolicy { .. }
        | DesignEdit::InsertSchematic { .. }
        | DesignEdit::RemoveSchematic
        | DesignEdit::InsertSchematicSymbol { .. }
        | DesignEdit::RemoveSchematicSymbol { .. }
        | DesignEdit::SetSchematicSymbolDefinition { .. }
        | DesignEdit::InsertSchematicSymbolLibrary { .. }
        | DesignEdit::RemoveSchematicSymbolLibrary { .. }
        | DesignEdit::SetSchematicSymbolLibraryDefinition { .. }
        | DesignEdit::InsertSchematicPortPlacement { .. }
        | DesignEdit::RemoveSchematicPortPlacement { .. }
        | DesignEdit::SetSchematicPortPlacementDefinition { .. }
        | DesignEdit::InsertSchematicWire { .. }
        | DesignEdit::RemoveSchematicWire { .. }
        | DesignEdit::SetSchematicWireDefinition { .. }
        | DesignEdit::InsertSchematicLabel { .. }
        | DesignEdit::RemoveSchematicLabel { .. }
        | DesignEdit::SetSchematicLabelDefinition { .. }
        | DesignEdit::InsertSchematicSheet { .. }
        | DesignEdit::RemoveSchematicSheet { .. }
        | DesignEdit::SetSchematicSheetDefinition { .. }
        | DesignEdit::InsertSchematicSheetPort { .. }
        | DesignEdit::RemoveSchematicSheetPort { .. }
        | DesignEdit::SetSchematicSheetPortDefinition { .. }
        | DesignEdit::InsertSchematicSheetLink { .. }
        | DesignEdit::RemoveSchematicSheetLink { .. }
        | DesignEdit::SetSchematicSheetLinkDefinition { .. }
        | DesignEdit::RemoveDeviceModel { .. }
        | DesignEdit::SetDeviceModelDefinition { .. }
        | DesignEdit::InsertCircuitInstance { .. }
        | DesignEdit::RemoveCircuitInstance { .. }
        | DesignEdit::SetCircuitInstanceDefinition { .. }
        | DesignEdit::SetCircuitInstanceModel { .. }
        | DesignEdit::SetCircuitInstancePins { .. }
        | DesignEdit::InsertPcb { .. }
        | DesignEdit::RemovePcb { .. } => {
            unreachable!("circuit and schematic edits are dispatched before PCB inverse generation")
        }
    })
}

fn apply_edit(document: &mut SemanticDocument, edit: &DesignEdit) -> Result<(), DesignEditError> {
    match edit {
        DesignEdit::InsertNet { net, index } => {
            if document
                .circuit
                .nets
                .iter()
                .any(|existing| existing.id == net.id)
            {
                return Err(DesignEditError::ExistingTarget(EditTarget::Net(
                    net.id.clone(),
                )));
            }
            let len = document.circuit.nets.len();
            let index = index.unwrap_or(len);
            if index > len {
                return Err(DesignEditError::InvalidInsertionIndex {
                    target: EditTarget::Net(net.id.clone()),
                    index,
                    len,
                });
            }
            document.circuit.nets.insert(index, net.clone());
        }
        DesignEdit::RemoveNet { net } => {
            let index = document
                .circuit
                .nets
                .iter()
                .position(|candidate| candidate.id == *net)
                .ok_or_else(|| DesignEditError::MissingTarget(EditTarget::Net(net.clone())))?;
            document.circuit.nets.remove(index);
        }
        DesignEdit::SetNetGround { net, is_ground } => {
            let existing = document
                .circuit
                .nets
                .iter_mut()
                .find(|candidate| candidate.id == *net)
                .ok_or_else(|| DesignEditError::MissingTarget(EditTarget::Net(net.clone())))?;
            existing.is_ground = *is_ground;
        }
        DesignEdit::InsertBus { bus, index } => {
            if document
                .circuit
                .buses
                .iter()
                .any(|existing| existing.id == bus.id)
            {
                return Err(DesignEditError::ExistingTarget(EditTarget::Bus(
                    bus.id.clone(),
                )));
            }
            let len = document.circuit.buses.len();
            let index = index.unwrap_or(len);
            if index > len {
                return Err(DesignEditError::InvalidInsertionIndex {
                    target: EditTarget::Bus(bus.id.clone()),
                    index,
                    len,
                });
            }
            document.circuit.buses.insert(index, bus.clone());
        }
        DesignEdit::RemoveBus { bus } => {
            let index = document
                .circuit
                .buses
                .iter()
                .position(|candidate| candidate.id == *bus)
                .ok_or_else(|| DesignEditError::MissingTarget(EditTarget::Bus(bus.clone())))?;
            document.circuit.buses.remove(index);
        }
        DesignEdit::SetBusNets { bus, nets } => {
            let existing = document
                .circuit
                .buses
                .iter_mut()
                .find(|candidate| candidate.id == *bus)
                .ok_or_else(|| DesignEditError::MissingTarget(EditTarget::Bus(bus.clone())))?;
            existing.nets.clone_from(nets);
        }
        DesignEdit::InsertBusSlice { slice, index } => {
            if document
                .circuit
                .bus_slices
                .iter()
                .any(|existing| existing.id == slice.id)
            {
                return Err(DesignEditError::ExistingTarget(EditTarget::BusSlice(
                    slice.id.clone(),
                )));
            }
            let len = document.circuit.bus_slices.len();
            let index = index.unwrap_or(len);
            if index > len {
                return Err(DesignEditError::InvalidInsertionIndex {
                    target: EditTarget::BusSlice(slice.id.clone()),
                    index,
                    len,
                });
            }
            document.circuit.bus_slices.insert(index, slice.clone());
        }
        DesignEdit::RemoveBusSlice { slice } => {
            let index = document
                .circuit
                .bus_slices
                .iter()
                .position(|candidate| candidate.id == *slice)
                .ok_or_else(|| {
                    DesignEditError::MissingTarget(EditTarget::BusSlice(slice.clone()))
                })?;
            document.circuit.bus_slices.remove(index);
        }
        DesignEdit::SetBusSliceDefinition {
            slice,
            bus,
            offset,
            width,
            order,
        } => {
            let existing = document
                .circuit
                .bus_slices
                .iter_mut()
                .find(|candidate| candidate.id == *slice)
                .ok_or_else(|| {
                    DesignEditError::MissingTarget(EditTarget::BusSlice(slice.clone()))
                })?;
            existing.bus = bus.clone();
            existing.offset = *offset;
            existing.width = *width;
            existing.order = *order;
        }
        DesignEdit::InsertCircuitPort { port, index } => {
            if document
                .circuit
                .ports
                .iter()
                .any(|existing| existing.id == port.id)
            {
                return Err(DesignEditError::ExistingTarget(EditTarget::CircuitPort(
                    port.id.clone(),
                )));
            }
            let len = document.circuit.ports.len();
            let index = index.unwrap_or(len);
            if index > len {
                return Err(DesignEditError::InvalidInsertionIndex {
                    target: EditTarget::CircuitPort(port.id.clone()),
                    index,
                    len,
                });
            }
            document.circuit.ports.insert(index, port.clone());
        }
        DesignEdit::RemoveCircuitPort { port } => {
            let index = document
                .circuit
                .ports
                .iter()
                .position(|candidate| candidate.id == *port)
                .ok_or_else(|| {
                    DesignEditError::MissingTarget(EditTarget::CircuitPort(port.clone()))
                })?;
            document.circuit.ports.remove(index);
        }
        DesignEdit::SetCircuitPortDefinition {
            port,
            net,
            direction,
            optional,
        } => {
            let existing = document
                .circuit
                .ports
                .iter_mut()
                .find(|candidate| candidate.id == *port)
                .ok_or_else(|| {
                    DesignEditError::MissingTarget(EditTarget::CircuitPort(port.clone()))
                })?;
            existing.net = net.clone();
            existing.direction = *direction;
            existing.optional = *optional;
        }
        DesignEdit::InsertRail { rail, index } => {
            if document
                .circuit
                .rails
                .iter()
                .any(|existing| existing.net == rail.net)
            {
                return Err(DesignEditError::ExistingTarget(EditTarget::Rail(
                    rail.net.clone(),
                )));
            }
            let len = document.circuit.rails.len();
            let index = checked_insertion_index(*index, len, EditTarget::Rail(rail.net.clone()))?;
            document.circuit.rails.insert(index, rail.clone());
        }
        DesignEdit::RemoveRail { net } => {
            remove_by_identity(
                &mut document.circuit.rails,
                |candidate| candidate.net == *net,
                EditTarget::Rail(net.clone()),
            )?;
        }
        DesignEdit::SetRailDefinition {
            net,
            nominal_voltage,
            max_current,
            kind,
        } => {
            let existing = document
                .circuit
                .rails
                .iter_mut()
                .find(|candidate| candidate.net == *net)
                .ok_or_else(|| DesignEditError::MissingTarget(EditTarget::Rail(net.clone())))?;
            existing.nominal_voltage.clone_from(nominal_voltage);
            existing.max_current.clone_from(max_current);
            existing.kind.clone_from(kind);
        }
        DesignEdit::InsertModuleParameter { parameter, index } => {
            if document
                .circuit
                .module_parameters
                .iter()
                .any(|existing| existing.name == parameter.name)
            {
                return Err(DesignEditError::ExistingTarget(
                    EditTarget::ModuleParameter(parameter.name.clone()),
                ));
            }
            let len = document.circuit.module_parameters.len();
            let index = checked_insertion_index(
                *index,
                len,
                EditTarget::ModuleParameter(parameter.name.clone()),
            )?;
            document
                .circuit
                .module_parameters
                .insert(index, parameter.clone());
        }
        DesignEdit::RemoveModuleParameter { parameter } => {
            remove_by_identity(
                &mut document.circuit.module_parameters,
                |candidate| candidate.name == *parameter,
                EditTarget::ModuleParameter(parameter.clone()),
            )?;
        }
        DesignEdit::SetModuleParameterDefinition {
            parameter,
            default,
            unit,
            source,
            targets,
        } => {
            let existing = document
                .circuit
                .module_parameters
                .iter_mut()
                .find(|candidate| candidate.name == *parameter)
                .ok_or_else(|| {
                    DesignEditError::MissingTarget(EditTarget::ModuleParameter(parameter.clone()))
                })?;
            existing.default.clone_from(default);
            existing.unit.clone_from(unit);
            existing.source.clone_from(source);
            existing.targets.clone_from(targets);
        }
        DesignEdit::InsertSourceStimulus { stimulus, index } => {
            if document
                .circuit
                .source_stimuli
                .iter()
                .any(|existing| existing.component == stimulus.component)
            {
                return Err(DesignEditError::ExistingTarget(EditTarget::SourceStimulus(
                    stimulus.component.clone(),
                )));
            }
            let len = document.circuit.source_stimuli.len();
            let index = checked_insertion_index(
                *index,
                len,
                EditTarget::SourceStimulus(stimulus.component.clone()),
            )?;
            document
                .circuit
                .source_stimuli
                .insert(index, stimulus.clone());
        }
        DesignEdit::RemoveSourceStimulus { component } => {
            remove_by_identity(
                &mut document.circuit.source_stimuli,
                |candidate| candidate.component == *component,
                EditTarget::SourceStimulus(component.clone()),
            )?;
        }
        DesignEdit::SetSourceStimulusWaveform {
            component,
            waveform,
        } => {
            let existing = document
                .circuit
                .source_stimuli
                .iter_mut()
                .find(|candidate| candidate.component == *component)
                .ok_or_else(|| {
                    DesignEditError::MissingTarget(EditTarget::SourceStimulus(component.clone()))
                })?;
            existing.waveform.clone_from(waveform);
        }
        DesignEdit::InsertSubcircuit { subcircuit, index } => {
            if document
                .circuit
                .subcircuits
                .iter()
                .any(|existing| existing.id == subcircuit.id)
            {
                return Err(DesignEditError::ExistingTarget(EditTarget::Subcircuit(
                    subcircuit.id.clone(),
                )));
            }
            let len = document.circuit.subcircuits.len();
            let index = checked_insertion_index(
                *index,
                len,
                EditTarget::Subcircuit(subcircuit.id.clone()),
            )?;
            document
                .circuit
                .subcircuits
                .insert(index, subcircuit.clone());
        }
        DesignEdit::RemoveSubcircuit { subcircuit } => {
            remove_by_identity(
                &mut document.circuit.subcircuits,
                |candidate| candidate.id == *subcircuit,
                EditTarget::Subcircuit(subcircuit.clone()),
            )?;
        }
        DesignEdit::SetSubcircuitDefinition {
            subcircuit,
            circuit,
            ports,
            parameter_overrides,
        } => {
            let existing = document
                .circuit
                .subcircuits
                .iter_mut()
                .find(|candidate| candidate.id == *subcircuit)
                .ok_or_else(|| {
                    DesignEditError::MissingTarget(EditTarget::Subcircuit(subcircuit.clone()))
                })?;
            existing.circuit.clone_from(circuit);
            existing.ports.clone_from(ports);
            existing.parameter_overrides.clone_from(parameter_overrides);
        }
        DesignEdit::SetLinearStamps { stamps } => {
            document.circuit.stamps.clone_from(stamps);
        }
        DesignEdit::SetCircuitPolicy {
            transient_policy,
            adapter_policy,
        } => {
            document
                .circuit
                .transient_policy
                .clone_from(transient_policy);
            document.circuit.adapter_policy.clone_from(adapter_policy);
        }
        DesignEdit::InsertDeviceModel { model, index } => {
            if document
                .circuit
                .device_models
                .iter()
                .any(|existing| existing.id == model.id)
            {
                return Err(DesignEditError::ExistingTarget(EditTarget::DeviceModel(
                    model.id.clone(),
                )));
            }
            let len = document.circuit.device_models.len();
            let index = index.unwrap_or(len);
            if index > len {
                return Err(DesignEditError::InvalidInsertionIndex {
                    target: EditTarget::DeviceModel(model.id.clone()),
                    index,
                    len,
                });
            }
            document
                .circuit
                .device_models
                .insert(index, (**model).clone());
        }
        DesignEdit::RemoveDeviceModel { model } => {
            let index = document
                .circuit
                .device_models
                .iter()
                .position(|candidate| candidate.id == *model)
                .ok_or_else(|| {
                    DesignEditError::MissingTarget(EditTarget::DeviceModel(model.clone()))
                })?;
            document.circuit.device_models.remove(index);
        }
        DesignEdit::SetDeviceModelDefinition { model } => {
            replace_by_identity(
                &mut document.circuit.device_models,
                |candidate| candidate.id == model.id,
                (**model).clone(),
                EditTarget::DeviceModel(model.id.clone()),
            )?;
        }
        DesignEdit::InsertCircuitInstance { instance, index } => {
            if document
                .circuit
                .instances
                .iter()
                .any(|existing| existing.id == instance.id)
            {
                return Err(DesignEditError::ExistingTarget(
                    EditTarget::CircuitInstance(instance.id.clone()),
                ));
            }
            let len = document.circuit.instances.len();
            let index = index.unwrap_or(len);
            if index > len {
                return Err(DesignEditError::InvalidInsertionIndex {
                    target: EditTarget::CircuitInstance(instance.id.clone()),
                    index,
                    len,
                });
            }
            document
                .circuit
                .instances
                .insert(index, (**instance).clone());
        }
        DesignEdit::RemoveCircuitInstance { instance } => {
            let index = document
                .circuit
                .instances
                .iter()
                .position(|candidate| candidate.id == *instance)
                .ok_or_else(|| {
                    DesignEditError::MissingTarget(EditTarget::CircuitInstance(instance.clone()))
                })?;
            document.circuit.instances.remove(index);
        }
        DesignEdit::SetCircuitInstanceDefinition { instance } => {
            replace_by_identity(
                &mut document.circuit.instances,
                |candidate| candidate.id == instance.id,
                (**instance).clone(),
                EditTarget::CircuitInstance(instance.id.clone()),
            )?;
        }
        DesignEdit::SetCircuitInstanceModel { instance, model } => {
            let existing = document
                .circuit
                .instances
                .iter_mut()
                .find(|candidate| candidate.id == *instance)
                .ok_or_else(|| {
                    DesignEditError::MissingTarget(EditTarget::CircuitInstance(instance.clone()))
                })?;
            existing.model = model.clone();
        }
        DesignEdit::SetCircuitInstancePins { instance, pins } => {
            let existing = document
                .circuit
                .instances
                .iter_mut()
                .find(|candidate| candidate.id == *instance)
                .ok_or_else(|| {
                    DesignEditError::MissingTarget(EditTarget::CircuitInstance(instance.clone()))
                })?;
            existing.pins.clone_from(pins);
        }
        DesignEdit::InsertPcb { pcb } => {
            if let Some(existing) = &document.pcb {
                return Err(DesignEditError::ExistingTarget(EditTarget::Pcb(
                    existing.id.clone(),
                )));
            }
            document.pcb = Some((**pcb).clone());
        }
        DesignEdit::RemovePcb { board } => {
            let existing = document.pcb.as_ref().ok_or(DesignEditError::MissingPcb)?;
            if existing.id != *board {
                return Err(DesignEditError::MissingTarget(EditTarget::Pcb(
                    board.clone(),
                )));
            }
            document.pcb = None;
        }
        DesignEdit::InsertSchematic { .. }
        | DesignEdit::RemoveSchematic
        | DesignEdit::InsertSchematicSymbol { .. }
        | DesignEdit::RemoveSchematicSymbol { .. }
        | DesignEdit::SetSchematicSymbolDefinition { .. }
        | DesignEdit::InsertSchematicSymbolLibrary { .. }
        | DesignEdit::RemoveSchematicSymbolLibrary { .. }
        | DesignEdit::SetSchematicSymbolLibraryDefinition { .. }
        | DesignEdit::InsertSchematicPortPlacement { .. }
        | DesignEdit::RemoveSchematicPortPlacement { .. }
        | DesignEdit::SetSchematicPortPlacementDefinition { .. }
        | DesignEdit::InsertSchematicWire { .. }
        | DesignEdit::RemoveSchematicWire { .. }
        | DesignEdit::SetSchematicWireDefinition { .. }
        | DesignEdit::InsertSchematicLabel { .. }
        | DesignEdit::RemoveSchematicLabel { .. }
        | DesignEdit::SetSchematicLabelDefinition { .. }
        | DesignEdit::InsertSchematicSheet { .. }
        | DesignEdit::RemoveSchematicSheet { .. }
        | DesignEdit::SetSchematicSheetDefinition { .. }
        | DesignEdit::InsertSchematicSheetPort { .. }
        | DesignEdit::RemoveSchematicSheetPort { .. }
        | DesignEdit::SetSchematicSheetPortDefinition { .. }
        | DesignEdit::InsertSchematicSheetLink { .. }
        | DesignEdit::RemoveSchematicSheetLink { .. }
        | DesignEdit::SetSchematicSheetLinkDefinition { .. } => {
            return apply_schematic_edit(document, edit);
        }
        _ => {
            let layout = document.pcb.as_mut().ok_or(DesignEditError::MissingPcb)?;
            return apply_layout_edit(layout, edit);
        }
    }
    Ok(())
}

fn apply_schematic_edit(
    document: &mut SemanticDocument,
    edit: &DesignEdit,
) -> Result<(), DesignEditError> {
    match edit {
        DesignEdit::InsertSchematic { schematic } => {
            if document.schematic.is_some() {
                return Err(DesignEditError::ExistingTarget(EditTarget::Schematic));
            }
            document.schematic = Some((**schematic).clone());
            return Ok(());
        }
        DesignEdit::RemoveSchematic => {
            if document.schematic.take().is_none() {
                return Err(DesignEditError::MissingSchematic);
            }
            return Ok(());
        }
        _ => {}
    }

    let schematic = document
        .schematic
        .as_mut()
        .ok_or(DesignEditError::MissingSchematic)?;
    match edit {
        DesignEdit::InsertSchematicSymbol { symbol, index } => {
            if schematic
                .symbols
                .iter()
                .any(|existing| existing.id == symbol.id)
            {
                return Err(DesignEditError::ExistingTarget(
                    EditTarget::SchematicSymbol(symbol.id.clone()),
                ));
            }
            let len = schematic.symbols.len();
            let index = checked_insertion_index(
                *index,
                len,
                EditTarget::SchematicSymbol(symbol.id.clone()),
            )?;
            schematic.symbols.insert(index, (**symbol).clone());
        }
        DesignEdit::RemoveSchematicSymbol { symbol } => {
            remove_by_identity(
                &mut schematic.symbols,
                |candidate| candidate.id == *symbol,
                EditTarget::SchematicSymbol(symbol.clone()),
            )?;
        }
        DesignEdit::SetSchematicSymbolDefinition { symbol } => {
            replace_by_identity(
                &mut schematic.symbols,
                |candidate| candidate.id == symbol.id,
                (**symbol).clone(),
                EditTarget::SchematicSymbol(symbol.id.clone()),
            )?;
        }
        DesignEdit::InsertSchematicSymbolLibrary { definition, index } => {
            if schematic
                .symbol_definitions
                .iter()
                .any(|existing| existing.id == definition.id)
            {
                return Err(DesignEditError::ExistingTarget(
                    EditTarget::SchematicSymbolLibrary(definition.id.clone()),
                ));
            }
            let len = schematic.symbol_definitions.len();
            let index = checked_insertion_index(
                *index,
                len,
                EditTarget::SchematicSymbolLibrary(definition.id.clone()),
            )?;
            schematic
                .symbol_definitions
                .insert(index, (**definition).clone());
        }
        DesignEdit::RemoveSchematicSymbolLibrary { definition } => {
            remove_by_identity(
                &mut schematic.symbol_definitions,
                |candidate| candidate.id == *definition,
                EditTarget::SchematicSymbolLibrary(definition.clone()),
            )?;
        }
        DesignEdit::SetSchematicSymbolLibraryDefinition { definition } => {
            replace_by_identity(
                &mut schematic.symbol_definitions,
                |candidate| candidate.id == definition.id,
                (**definition).clone(),
                EditTarget::SchematicSymbolLibrary(definition.id.clone()),
            )?;
        }
        DesignEdit::InsertSchematicPortPlacement { placement, index } => {
            if schematic
                .ports
                .iter()
                .any(|existing| existing.port == placement.port)
            {
                return Err(DesignEditError::ExistingTarget(
                    EditTarget::SchematicPortPlacement(placement.port.clone()),
                ));
            }
            let len = schematic.ports.len();
            let index = checked_insertion_index(
                *index,
                len,
                EditTarget::SchematicPortPlacement(placement.port.clone()),
            )?;
            schematic.ports.insert(index, placement.clone());
        }
        DesignEdit::RemoveSchematicPortPlacement { port } => {
            remove_by_identity(
                &mut schematic.ports,
                |candidate| candidate.port == *port,
                EditTarget::SchematicPortPlacement(port.clone()),
            )?;
        }
        DesignEdit::SetSchematicPortPlacementDefinition { placement } => {
            replace_by_identity(
                &mut schematic.ports,
                |candidate| candidate.port == placement.port,
                placement.clone(),
                EditTarget::SchematicPortPlacement(placement.port.clone()),
            )?;
        }
        DesignEdit::InsertSchematicWire { wire, index } => {
            if schematic
                .wires
                .iter()
                .any(|existing| existing.id == wire.id)
            {
                return Err(DesignEditError::ExistingTarget(EditTarget::SchematicWire(
                    wire.id.clone(),
                )));
            }
            let len = schematic.wires.len();
            let index =
                checked_insertion_index(*index, len, EditTarget::SchematicWire(wire.id.clone()))?;
            schematic.wires.insert(index, (**wire).clone());
        }
        DesignEdit::RemoveSchematicWire { wire } => {
            remove_by_identity(
                &mut schematic.wires,
                |candidate| candidate.id == *wire,
                EditTarget::SchematicWire(wire.clone()),
            )?;
        }
        DesignEdit::SetSchematicWireDefinition { wire } => {
            replace_by_identity(
                &mut schematic.wires,
                |candidate| candidate.id == wire.id,
                (**wire).clone(),
                EditTarget::SchematicWire(wire.id.clone()),
            )?;
        }
        DesignEdit::InsertSchematicLabel { label, index } => {
            if schematic
                .labels
                .iter()
                .any(|existing| existing.id == label.id)
            {
                return Err(DesignEditError::ExistingTarget(EditTarget::SchematicLabel(
                    label.id.clone(),
                )));
            }
            let len = schematic.labels.len();
            let index =
                checked_insertion_index(*index, len, EditTarget::SchematicLabel(label.id.clone()))?;
            schematic.labels.insert(index, label.clone());
        }
        DesignEdit::RemoveSchematicLabel { label } => {
            remove_by_identity(
                &mut schematic.labels,
                |candidate| candidate.id == *label,
                EditTarget::SchematicLabel(label.clone()),
            )?;
        }
        DesignEdit::SetSchematicLabelDefinition { label } => {
            replace_by_identity(
                &mut schematic.labels,
                |candidate| candidate.id == label.id,
                label.clone(),
                EditTarget::SchematicLabel(label.id.clone()),
            )?;
        }
        DesignEdit::InsertSchematicSheet { sheet, index } => {
            if schematic
                .sheets
                .iter()
                .any(|existing| existing.id == sheet.id)
            {
                return Err(DesignEditError::ExistingTarget(EditTarget::SchematicSheet(
                    sheet.id.clone(),
                )));
            }
            let len = schematic.sheets.len();
            let index =
                checked_insertion_index(*index, len, EditTarget::SchematicSheet(sheet.id.clone()))?;
            schematic.sheets.insert(index, sheet.clone());
        }
        DesignEdit::RemoveSchematicSheet { sheet } => {
            remove_by_identity(
                &mut schematic.sheets,
                |candidate| candidate.id == *sheet,
                EditTarget::SchematicSheet(sheet.clone()),
            )?;
        }
        DesignEdit::SetSchematicSheetDefinition { sheet } => {
            replace_by_identity(
                &mut schematic.sheets,
                |candidate| candidate.id == sheet.id,
                sheet.clone(),
                EditTarget::SchematicSheet(sheet.id.clone()),
            )?;
        }
        DesignEdit::InsertSchematicSheetPort { port, index } => {
            if schematic
                .sheet_ports
                .iter()
                .any(|existing| existing.id == port.id)
            {
                return Err(DesignEditError::ExistingTarget(
                    EditTarget::SchematicSheetPort(port.id.clone()),
                ));
            }
            let len = schematic.sheet_ports.len();
            let index = checked_insertion_index(
                *index,
                len,
                EditTarget::SchematicSheetPort(port.id.clone()),
            )?;
            schematic.sheet_ports.insert(index, port.clone());
        }
        DesignEdit::RemoveSchematicSheetPort { port } => {
            remove_by_identity(
                &mut schematic.sheet_ports,
                |candidate| candidate.id == *port,
                EditTarget::SchematicSheetPort(port.clone()),
            )?;
        }
        DesignEdit::SetSchematicSheetPortDefinition { port } => {
            replace_by_identity(
                &mut schematic.sheet_ports,
                |candidate| candidate.id == port.id,
                port.clone(),
                EditTarget::SchematicSheetPort(port.id.clone()),
            )?;
        }
        DesignEdit::InsertSchematicSheetLink { link, index } => {
            if schematic
                .sheet_links
                .iter()
                .any(|existing| existing.id == link.id)
            {
                return Err(DesignEditError::ExistingTarget(
                    EditTarget::SchematicSheetLink(link.id.clone()),
                ));
            }
            let len = schematic.sheet_links.len();
            let index = checked_insertion_index(
                *index,
                len,
                EditTarget::SchematicSheetLink(link.id.clone()),
            )?;
            schematic.sheet_links.insert(index, link.clone());
        }
        DesignEdit::RemoveSchematicSheetLink { link } => {
            remove_by_identity(
                &mut schematic.sheet_links,
                |candidate| candidate.id == *link,
                EditTarget::SchematicSheetLink(link.clone()),
            )?;
        }
        DesignEdit::SetSchematicSheetLinkDefinition { link } => {
            replace_by_identity(
                &mut schematic.sheet_links,
                |candidate| candidate.id == link.id,
                link.clone(),
                EditTarget::SchematicSheetLink(link.id.clone()),
            )?;
        }
        DesignEdit::InsertSchematic { .. } | DesignEdit::RemoveSchematic => {
            unreachable!("schematic container edits return before collection mutation")
        }
        _ => unreachable!("non-schematic edit dispatched to schematic replay"),
    }
    Ok(())
}

fn checked_insertion_index(
    index: Option<usize>,
    len: usize,
    target: EditTarget,
) -> Result<usize, DesignEditError> {
    let index = index.unwrap_or(len);
    if index > len {
        return Err(DesignEditError::InvalidInsertionIndex { target, index, len });
    }
    Ok(index)
}

fn remove_by_identity<T>(
    values: &mut Vec<T>,
    matches: impl FnMut(&T) -> bool,
    target: EditTarget,
) -> Result<(), DesignEditError> {
    let index = values
        .iter()
        .position(matches)
        .ok_or(DesignEditError::MissingTarget(target))?;
    values.remove(index);
    Ok(())
}

fn replace_by_identity<T>(
    values: &mut [T],
    matches: impl FnMut(&T) -> bool,
    replacement: T,
    target: EditTarget,
) -> Result<(), DesignEditError> {
    let index = values
        .iter()
        .position(matches)
        .ok_or(DesignEditError::MissingTarget(target))?;
    values[index] = replacement;
    Ok(())
}

fn apply_layout_edit(layout: &mut PcbLayout, edit: &DesignEdit) -> Result<(), DesignEditError> {
    match edit {
        DesignEdit::SetBoardOutline { board, outline } => {
            if layout.id != *board {
                return Err(DesignEditError::MissingTarget(EditTarget::Board(
                    board.clone(),
                )));
            }
            layout.outline.clone_from(outline);
        }
        DesignEdit::SetBoardStackup { board, stackup } => {
            if layout.id != *board {
                return Err(DesignEditError::MissingTarget(EditTarget::Board(
                    board.clone(),
                )));
            }
            layout.stackup.clone_from(stackup);
        }
        DesignEdit::SetBoardRules { board, rules } => {
            if layout.id != *board {
                return Err(DesignEditError::MissingTarget(EditTarget::Board(
                    board.clone(),
                )));
            }
            layout.rules.clone_from(rules);
        }
        DesignEdit::InsertLandPattern {
            land_pattern,
            index,
        } => {
            if layout
                .land_patterns
                .iter()
                .any(|existing| existing.id == land_pattern.id)
            {
                return Err(DesignEditError::ExistingTarget(EditTarget::LandPattern(
                    land_pattern.id.clone(),
                )));
            }
            let len = layout.land_patterns.len();
            let index = index.unwrap_or(len);
            if index > len {
                return Err(DesignEditError::InvalidInsertionIndex {
                    target: EditTarget::LandPattern(land_pattern.id.clone()),
                    index,
                    len,
                });
            }
            layout.land_patterns.insert(index, (**land_pattern).clone());
        }
        DesignEdit::RemoveLandPattern { land_pattern } => {
            let index = layout
                .land_patterns
                .iter()
                .position(|candidate| candidate.id == *land_pattern)
                .ok_or_else(|| {
                    DesignEditError::MissingTarget(EditTarget::LandPattern(land_pattern.clone()))
                })?;
            layout.land_patterns.remove(index);
        }
        DesignEdit::SetLandPatternDefinition { land_pattern } => {
            replace_by_identity(
                &mut layout.land_patterns,
                |candidate| candidate.id == land_pattern.id,
                (**land_pattern).clone(),
                EditTarget::LandPattern(land_pattern.id.clone()),
            )?;
        }
        DesignEdit::InsertKeepout { keepout, index } => {
            if layout
                .keepouts
                .iter()
                .any(|existing| existing.id == keepout.id)
            {
                return Err(DesignEditError::ExistingTarget(EditTarget::Keepout(
                    keepout.id.clone(),
                )));
            }
            let len = layout.keepouts.len();
            let index =
                checked_insertion_index(*index, len, EditTarget::Keepout(keepout.id.clone()))?;
            layout.keepouts.insert(index, keepout.clone());
        }
        DesignEdit::RemoveKeepout { keepout } => {
            remove_by_identity(
                &mut layout.keepouts,
                |candidate| candidate.id == *keepout,
                EditTarget::Keepout(keepout.clone()),
            )?;
        }
        DesignEdit::SetKeepoutDefinition { keepout } => {
            replace_by_identity(
                &mut layout.keepouts,
                |candidate| candidate.id == keepout.id,
                keepout.clone(),
                EditTarget::Keepout(keepout.id.clone()),
            )?;
        }
        DesignEdit::SetPlacementTransform {
            instance,
            position,
            rotation_degrees,
            side,
        } => {
            let placement = layout
                .placements
                .iter_mut()
                .find(|placement| placement.instance == *instance)
                .ok_or_else(|| {
                    DesignEditError::MissingTarget(EditTarget::Placement(instance.clone()))
                })?;
            placement.position = position.clone();
            placement.rotation_degrees = rotation_degrees.clone();
            placement.side = *side;
        }
        DesignEdit::SetPlacementLandPattern {
            instance,
            land_pattern,
        } => {
            let placement = layout
                .placements
                .iter_mut()
                .find(|placement| placement.instance == *instance)
                .ok_or_else(|| {
                    DesignEditError::MissingTarget(EditTarget::Placement(instance.clone()))
                })?;
            placement.land_pattern = land_pattern.clone();
        }
        DesignEdit::InsertPlacement { placement, index } => {
            if layout
                .placements
                .iter()
                .any(|existing| existing.instance == placement.instance)
            {
                return Err(DesignEditError::ExistingTarget(EditTarget::Placement(
                    placement.instance.clone(),
                )));
            }
            let len = layout.placements.len();
            let index = index.unwrap_or(len);
            if index > len {
                return Err(DesignEditError::InvalidInsertionIndex {
                    target: EditTarget::Placement(placement.instance.clone()),
                    index,
                    len,
                });
            }
            layout.placements.insert(index, (**placement).clone());
        }
        DesignEdit::RemovePlacement { instance } => {
            let index = layout
                .placements
                .iter()
                .position(|placement| placement.instance == *instance)
                .ok_or_else(|| {
                    DesignEditError::MissingTarget(EditTarget::Placement(instance.clone()))
                })?;
            layout.placements.remove(index);
        }
        DesignEdit::InsertPlacementConstraint { constraint, index } => {
            if layout
                .placement_constraints
                .iter()
                .any(|existing| existing.id == constraint.id)
            {
                return Err(DesignEditError::ExistingTarget(
                    EditTarget::PlacementConstraint(constraint.id.clone()),
                ));
            }
            let len = layout.placement_constraints.len();
            let index = index.unwrap_or(len);
            if index > len {
                return Err(DesignEditError::InvalidInsertionIndex {
                    target: EditTarget::PlacementConstraint(constraint.id.clone()),
                    index,
                    len,
                });
            }
            layout
                .placement_constraints
                .insert(index, (**constraint).clone());
        }
        DesignEdit::RemovePlacementConstraint { constraint } => {
            let index = layout
                .placement_constraints
                .iter()
                .position(|candidate| candidate.id == *constraint)
                .ok_or_else(|| {
                    DesignEditError::MissingTarget(EditTarget::PlacementConstraint(
                        constraint.clone(),
                    ))
                })?;
            layout.placement_constraints.remove(index);
        }
        DesignEdit::SetPlacementConstraintDefinition { constraint } => {
            replace_by_identity(
                &mut layout.placement_constraints,
                |candidate| candidate.id == constraint.id,
                (**constraint).clone(),
                EditTarget::PlacementConstraint(constraint.id.clone()),
            )?;
        }
        DesignEdit::SetRouteWidth { route, width } => {
            let route = layout
                .routes
                .iter_mut()
                .find(|candidate| candidate.id == *route)
                .ok_or_else(|| DesignEditError::MissingTarget(EditTarget::Route(route.clone())))?;
            route.width = width.clone();
        }
        DesignEdit::SetRouteSegments { route, segments } => {
            let route = layout
                .routes
                .iter_mut()
                .find(|candidate| candidate.id == *route)
                .ok_or_else(|| DesignEditError::MissingTarget(EditTarget::Route(route.clone())))?;
            route.segments.clone_from(segments);
        }
        DesignEdit::SetRouteNet { route, net } => {
            let route = layout
                .routes
                .iter_mut()
                .find(|candidate| candidate.id == *route)
                .ok_or_else(|| DesignEditError::MissingTarget(EditTarget::Route(route.clone())))?;
            route.net = net.clone();
        }
        DesignEdit::InsertRoute { route, index } => {
            if layout.routes.iter().any(|existing| existing.id == route.id) {
                return Err(DesignEditError::ExistingTarget(EditTarget::Route(
                    route.id.clone(),
                )));
            }
            let len = layout.routes.len();
            let index = index.unwrap_or(len);
            if index > len {
                return Err(DesignEditError::InvalidInsertionIndex {
                    target: EditTarget::Route(route.id.clone()),
                    index,
                    len,
                });
            }
            layout.routes.insert(index, (**route).clone());
        }
        DesignEdit::RemoveRoute { route } => {
            let index = layout
                .routes
                .iter()
                .position(|candidate| candidate.id == *route)
                .ok_or_else(|| DesignEditError::MissingTarget(EditTarget::Route(route.clone())))?;
            layout.routes.remove(index);
        }
        DesignEdit::SetRouteDefinition { route } => {
            replace_by_identity(
                &mut layout.routes,
                |candidate| candidate.id == route.id,
                (**route).clone(),
                EditTarget::Route(route.id.clone()),
            )?;
        }
        DesignEdit::MoveVia { via, center } => {
            let via = layout
                .vias
                .iter_mut()
                .find(|candidate| candidate.id == *via)
                .ok_or_else(|| DesignEditError::MissingTarget(EditTarget::Via(via.clone())))?;
            via.center = center.clone();
        }
        DesignEdit::SetViaNet { via, net } => {
            let via = layout
                .vias
                .iter_mut()
                .find(|candidate| candidate.id == *via)
                .ok_or_else(|| DesignEditError::MissingTarget(EditTarget::Via(via.clone())))?;
            via.net = net.clone();
        }
        DesignEdit::InsertVia { via, index } => {
            if layout.vias.iter().any(|existing| existing.id == via.id) {
                return Err(DesignEditError::ExistingTarget(EditTarget::Via(
                    via.id.clone(),
                )));
            }
            let len = layout.vias.len();
            let index = index.unwrap_or(len);
            if index > len {
                return Err(DesignEditError::InvalidInsertionIndex {
                    target: EditTarget::Via(via.id.clone()),
                    index,
                    len,
                });
            }
            layout.vias.insert(index, (**via).clone());
        }
        DesignEdit::RemoveVia { via } => {
            let index = layout
                .vias
                .iter()
                .position(|candidate| candidate.id == *via)
                .ok_or_else(|| DesignEditError::MissingTarget(EditTarget::Via(via.clone())))?;
            layout.vias.remove(index);
        }
        DesignEdit::SetViaDefinition { via } => {
            replace_by_identity(
                &mut layout.vias,
                |candidate| candidate.id == via.id,
                (**via).clone(),
                EditTarget::Via(via.id.clone()),
            )?;
        }
        DesignEdit::SetZoneBoundary { zone, boundary } => {
            let zone = layout
                .zones
                .iter_mut()
                .find(|candidate| candidate.id == *zone)
                .ok_or_else(|| DesignEditError::MissingTarget(EditTarget::Zone(zone.clone())))?;
            zone.boundary = boundary.clone();
        }
        DesignEdit::SetZoneNet { zone, net } => {
            let zone = layout
                .zones
                .iter_mut()
                .find(|candidate| candidate.id == *zone)
                .ok_or_else(|| DesignEditError::MissingTarget(EditTarget::Zone(zone.clone())))?;
            zone.net = net.clone();
        }
        DesignEdit::InsertZone { zone, index } => {
            if layout.zones.iter().any(|existing| existing.id == zone.id) {
                return Err(DesignEditError::ExistingTarget(EditTarget::Zone(
                    zone.id.clone(),
                )));
            }
            let len = layout.zones.len();
            let index = index.unwrap_or(len);
            if index > len {
                return Err(DesignEditError::InvalidInsertionIndex {
                    target: EditTarget::Zone(zone.id.clone()),
                    index,
                    len,
                });
            }
            layout.zones.insert(index, (**zone).clone());
        }
        DesignEdit::RemoveZone { zone } => {
            let index = layout
                .zones
                .iter()
                .position(|candidate| candidate.id == *zone)
                .ok_or_else(|| DesignEditError::MissingTarget(EditTarget::Zone(zone.clone())))?;
            layout.zones.remove(index);
        }
        DesignEdit::SetZoneDefinition { zone } => {
            replace_by_identity(
                &mut layout.zones,
                |candidate| candidate.id == zone.id,
                (**zone).clone(),
                EditTarget::Zone(zone.id.clone()),
            )?;
        }
        DesignEdit::InsertDeviceModel { .. }
        | DesignEdit::InsertNet { .. }
        | DesignEdit::RemoveNet { .. }
        | DesignEdit::SetNetGround { .. }
        | DesignEdit::InsertBus { .. }
        | DesignEdit::RemoveBus { .. }
        | DesignEdit::SetBusNets { .. }
        | DesignEdit::InsertBusSlice { .. }
        | DesignEdit::RemoveBusSlice { .. }
        | DesignEdit::SetBusSliceDefinition { .. }
        | DesignEdit::InsertCircuitPort { .. }
        | DesignEdit::RemoveCircuitPort { .. }
        | DesignEdit::SetCircuitPortDefinition { .. }
        | DesignEdit::InsertRail { .. }
        | DesignEdit::RemoveRail { .. }
        | DesignEdit::SetRailDefinition { .. }
        | DesignEdit::InsertModuleParameter { .. }
        | DesignEdit::RemoveModuleParameter { .. }
        | DesignEdit::SetModuleParameterDefinition { .. }
        | DesignEdit::InsertSourceStimulus { .. }
        | DesignEdit::RemoveSourceStimulus { .. }
        | DesignEdit::SetSourceStimulusWaveform { .. }
        | DesignEdit::InsertSubcircuit { .. }
        | DesignEdit::RemoveSubcircuit { .. }
        | DesignEdit::SetSubcircuitDefinition { .. }
        | DesignEdit::SetLinearStamps { .. }
        | DesignEdit::SetCircuitPolicy { .. }
        | DesignEdit::InsertSchematic { .. }
        | DesignEdit::RemoveSchematic
        | DesignEdit::InsertSchematicSymbol { .. }
        | DesignEdit::RemoveSchematicSymbol { .. }
        | DesignEdit::SetSchematicSymbolDefinition { .. }
        | DesignEdit::InsertSchematicSymbolLibrary { .. }
        | DesignEdit::RemoveSchematicSymbolLibrary { .. }
        | DesignEdit::SetSchematicSymbolLibraryDefinition { .. }
        | DesignEdit::InsertSchematicPortPlacement { .. }
        | DesignEdit::RemoveSchematicPortPlacement { .. }
        | DesignEdit::SetSchematicPortPlacementDefinition { .. }
        | DesignEdit::InsertSchematicWire { .. }
        | DesignEdit::RemoveSchematicWire { .. }
        | DesignEdit::SetSchematicWireDefinition { .. }
        | DesignEdit::InsertSchematicLabel { .. }
        | DesignEdit::RemoveSchematicLabel { .. }
        | DesignEdit::SetSchematicLabelDefinition { .. }
        | DesignEdit::InsertSchematicSheet { .. }
        | DesignEdit::RemoveSchematicSheet { .. }
        | DesignEdit::SetSchematicSheetDefinition { .. }
        | DesignEdit::InsertSchematicSheetPort { .. }
        | DesignEdit::RemoveSchematicSheetPort { .. }
        | DesignEdit::SetSchematicSheetPortDefinition { .. }
        | DesignEdit::InsertSchematicSheetLink { .. }
        | DesignEdit::RemoveSchematicSheetLink { .. }
        | DesignEdit::SetSchematicSheetLinkDefinition { .. }
        | DesignEdit::RemoveDeviceModel { .. }
        | DesignEdit::SetDeviceModelDefinition { .. }
        | DesignEdit::InsertCircuitInstance { .. }
        | DesignEdit::RemoveCircuitInstance { .. }
        | DesignEdit::SetCircuitInstanceDefinition { .. }
        | DesignEdit::SetCircuitInstanceModel { .. }
        | DesignEdit::SetCircuitInstancePins { .. }
        | DesignEdit::InsertPcb { .. }
        | DesignEdit::RemovePcb { .. } => {
            unreachable!("circuit and schematic edits are dispatched before PCB replay")
        }
    }
    Ok(())
}
