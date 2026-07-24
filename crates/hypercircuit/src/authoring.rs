//! Fluent circuit-and-board authoring over the retained semantic model.
//!
//! This module is intentionally a front door rather than a second IR. A
//! [`Design`] writes directly into [`Circuit`] and [`PcbLayout`], while typed
//! handles keep string lookup at declaration boundaries.

use std::collections::BTreeSet;
use std::fmt::{Display, Formatter};
use std::panic::Location;
use std::sync::atomic::{AtomicU64, Ordering};

use hyperlattice::Point2;
use hyperpath::{CubicBezier, ExplicitCircularArc, LinePathSegment, TraceLayer};
use hyperreal::Real;

use crate::{
    AdapterKind, BoardId, BoardOutline, BoardSide, Bus, BusId, BusSlice, BusSliceId, BusSliceOrder,
    Circuit, CircuitError, CircuitId, CircuitInstance, CircuitInstanceId, CircuitParameter,
    CircuitPort, CircuitValidationIssue, CircuitValidationReport, ComponentId, CopperZone,
    CopperZoneConnection, CopperZoneFill, CopperZoneIslandPolicy, CopperZoneStitchingPolicy,
    DeviceModel, DeviceModelId, DeviceModelKind, DevicePin, DifferentialPair, DifferentialPairId,
    DrillShape, KeepoutId, KeepoutScope, LandPattern, LandPatternBody, LandPatternGraphic,
    LandPatternId, LandPatternPad, LayoutValidationIssue, LayoutValidationReport,
    LengthTuningPattern, LengthTuningPatternId, LengthTuningSide, Net, NetClass, NetClassId, NetId,
    PadId, PadPinMap, PadShape, PartRef, PcbKeepout, PcbLayout, PcbPlacement, PcbRoute,
    PcbRouteSegment, PcbStackup, PcbVia, PhaseTuningGroup, PhaseTuningGroupId, PinElectricalKind,
    PinRef, PlacementConstraint, PlacementConstraintId, PlacementConstraintKind, Plating,
    PortDirection, PortId, RailIntent, RailKind, RouteId, SchematicEndpoint, SchematicGraphic,
    SchematicGraphicFill, SchematicLayout, SchematicPinPlacement, SchematicPinSide, SchematicPoint,
    SchematicSymbol, SchematicSymbolDefinition, SchematicSymbolDefinitionId, SchematicSymbolId,
    SchematicSymbolUnit, SchematicValidationIssue, SchematicValidationReport, SchematicWire,
    SchematicWireId, SourceStimulus, SourceWaveform, TransientPolicy, ViaId, ViaMaskIntent,
    ViaStyle, ViaStyleId, ViaStyleSpan, ZoneId,
};
#[cfg(feature = "interchange")]
use crate::{PackageResolutionError, PortablePartDefinition};

static NEXT_DESIGN_OWNER: AtomicU64 = AtomicU64::new(1);

/// Owned Rust source location captured at an authoring API boundary.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub struct SourceLocation {
    /// Source file reported by the Rust compiler.
    pub file: String,
    /// One-based source line.
    pub line: u32,
    /// One-based source column.
    pub column: u32,
}

impl SourceLocation {
    #[track_caller]
    fn caller() -> Self {
        let location = Location::caller();
        Self {
            file: location.file().into(),
            line: location.line(),
            column: location.column(),
        }
    }
}

impl Display for SourceLocation {
    fn fmt(&self, formatter: &mut Formatter<'_>) -> std::fmt::Result {
        write!(formatter, "{}:{}:{}", self.file, self.line, self.column)
    }
}

/// Stable semantic object addressed by one fluent authoring operation.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum AuthoringTarget {
    /// Whole retained circuit.
    Circuit(CircuitId),
    /// Whole retained PCB.
    Board(BoardId),
    /// Logical net or rail.
    Net(NetId),
    /// Ordered logical bus.
    Bus(BusId),
    /// Named retained bus slice.
    BusSlice(BusSliceId),
    /// Circuit boundary port.
    Port(PortId),
    /// Executable or custom device model.
    DeviceModel(DeviceModelId),
    /// Reusable schematic symbol definition.
    SymbolDefinition(SchematicSymbolDefinitionId),
    /// Logical component instance.
    Instance(CircuitInstanceId),
    /// Declared logical pin.
    Pin {
        /// Owning instance.
        instance: CircuitInstanceId,
        /// Pin identity.
        pin: PinRef,
    },
    /// One pin-to-net binding.
    Connection {
        /// Owning instance.
        instance: CircuitInstanceId,
        /// Bound pin.
        pin: PinRef,
    },
    /// Reusable physical land pattern.
    LandPattern(LandPatternId),
    /// Physical pad within one land pattern.
    Pad {
        /// Owning land pattern.
        land_pattern: LandPatternId,
        /// Pad identity.
        pad: PadId,
    },
    /// Physical placement of one logical instance.
    Placement(CircuitInstanceId),
    /// One retained placement equation or predicate.
    PlacementConstraint(PlacementConstraintId),
    /// One retained routed-copper centerline.
    Route(RouteId),
    /// One retained copper-layer transition.
    Via(ViaId),
    /// One retained copper-pour source boundary.
    Zone(ZoneId),
    /// One retained physical keepout.
    Keepout(KeepoutId),
    /// One retained manufacturable via construction.
    ViaStyle(ViaStyleId),
    /// One retained net-class policy.
    NetClass(NetClassId),
    /// One retained differential-pair declaration.
    DifferentialPair(DifferentialPairId),
    /// One retained bounded route-length tuning request.
    LengthTuningPattern(LengthTuningPatternId),
    /// One retained atomic phase-matching group.
    PhaseTuningGroup(PhaseTuningGroupId),
    /// Whole retained schematic review model.
    Schematic(CircuitId),
    /// One placed schematic symbol unit.
    Symbol(SchematicSymbolId),
    /// One typed schematic wire.
    Wire(SchematicWireId),
    /// Time-dependent intent attached to one independent source component.
    Stimulus(ComponentId),
}

/// Kind of authoring operation that produced or changed retained intent.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum AuthoringAction {
    /// Declared a semantic object.
    Declare,
    /// Bound a logical pin to a net.
    Connect,
    /// Placed a physical package.
    Place,
    /// Requested direct advanced mutation through an escape hatch.
    Mutate,
}

/// One source-addressable authoring operation.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct AuthoringTrace {
    /// Semantic target affected by the operation.
    pub target: AuthoringTarget,
    /// Operation performed.
    pub action: AuthoringAction,
    /// Rust call site that requested it.
    pub source: SourceLocation,
}

/// Provenance retained beside, rather than inside, circuit and PCB semantic IR.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, Default, Eq, PartialEq)]
pub struct DesignSourceMap {
    /// Authoring operations in deterministic declaration order.
    pub traces: Vec<AuthoringTrace>,
}

impl DesignSourceMap {
    /// Returns every trace addressing `target`, preserving declaration order.
    pub fn traces_for(&self, target: &AuthoringTarget) -> Vec<&AuthoringTrace> {
        self.traces
            .iter()
            .filter(|trace| &trace.target == target)
            .collect()
    }

    fn push(&mut self, target: AuthoringTarget, action: AuthoringAction, source: SourceLocation) {
        self.traces.push(AuthoringTrace {
            target,
            action,
            source,
        });
    }
}

/// Structural issue from either authoritative retained validator.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum DesignValidationIssue {
    /// Circuit identity, hierarchy, pin, or connectivity issue.
    Circuit(CircuitValidationIssue),
    /// PCB identity, placement, geometry-carrier, or rule issue.
    Layout(LayoutValidationIssue),
    /// Schematic drawing identity or connectivity issue.
    Schematic(SchematicValidationIssue),
}

/// Structural validation issue correlated to its Rust authoring call sites.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct DesignDiagnostic {
    /// Authoritative validation finding.
    pub issue: DesignValidationIssue,
    /// Most specific known declaration/mutation sites, never empty for a
    /// design created through [`Design`].
    pub sources: Vec<SourceLocation>,
}

/// Failure while lowering fluent declarations into retained circuit/layout intent.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum DesignBuildError {
    /// A typed identity could not be constructed.
    InvalidIdentifier,
    /// A logical net already exists.
    DuplicateNet(String),
    /// An ordered bus has no members.
    EmptyBus(String),
    /// A bus repeats one logical net.
    DuplicateBusNet { bus: String, net: String },
    /// A bus identity already exists.
    DuplicateBus(String),
    /// A typed bus handle no longer addresses a retained bus.
    UnknownBus(String),
    /// A named bus slice identity already exists.
    DuplicateBusSlice(String),
    /// A bus slice is empty, overflows, or exceeds its source bus.
    InvalidBusSlice(String),
    /// A circuit boundary port identity already exists.
    DuplicatePort(String),
    /// A component reference already exists.
    DuplicateInstance(String),
    /// A reusable part definition already exists.
    DuplicatePartDefinition(String),
    /// A part declares one logical pin more than once.
    DuplicatePin { instance: String, pin: String },
    /// A footprint declares one physical pad more than once.
    DuplicatePad { instance: String, pad: String },
    /// A schematic symbol declares one logical pin more than once.
    DuplicateSymbolPin { instance: String, pin: String },
    /// A schematic symbol names a pin absent from the part declaration.
    UnknownSymbolPin { instance: String, pin: String },
    /// A reusable symbol definition repeats one unit number.
    DuplicatePartSymbolUnit { definition: String, unit: u16 },
    /// A reusable symbol placement repeats one unit number.
    DuplicateSymbolPlacement { instance: String, unit: u16 },
    /// A reusable part instance places a symbol unit it does not define.
    UnknownPartSymbolUnit { definition: String, unit: u16 },
    /// A reusable part instance requests a symbol without a symbol definition.
    MissingPartSymbolDefinition(String),
    /// A required part pin is absent from its attached schematic symbol.
    MissingSymbolPin { instance: String, pin: String },
    /// Exact schematic symbol geometry could not be constructed.
    InvalidSymbolGeometry(String),
    /// More than one waveform was attached to an independent source.
    DuplicateStimulus(String),
    /// A waveform was attached to a part other than an independent source.
    InvalidStimulusTarget(String),
    /// A placed part or pin-to-pad map has no footprint.
    MissingFootprint(String),
    /// A part with a footprint has no authored placement.
    MissingPlacement(String),
    /// A pin mapping names a pad absent from the footprint.
    UnknownPad {
        instance: String,
        pin: String,
        pad: String,
    },
    /// A typed instance handle does not contain the requested pin.
    UnknownPin { instance: String, pin: String },
    /// A connection was requested more than once.
    DuplicateConnection { instance: String, pin: String },
    /// A routed-copper identity already exists.
    DuplicateRoute(String),
    /// A route declaration is structurally invalid.
    InvalidRoute(String),
    /// A via identity already exists.
    DuplicateVia(String),
    /// A via declaration is structurally invalid.
    InvalidVia(String),
    /// A copper-zone identity already exists.
    DuplicateZone(String),
    /// A copper-zone declaration is structurally invalid.
    InvalidZone(String),
    /// A keepout identity already exists.
    DuplicateKeepout(String),
    /// A keepout declaration is structurally invalid.
    InvalidKeepout(String),
    /// A placement-constraint identity already exists.
    DuplicatePlacementConstraint(String),
    /// A placement-constraint declaration is structurally invalid.
    InvalidPlacementConstraint(String),
    /// A named via-style identity already exists.
    DuplicateViaStyle(String),
    /// A via-style declaration is structurally invalid.
    InvalidViaStyle(String),
    /// A net-class identity already exists.
    DuplicateNetClass(String),
    /// A net-class declaration is structurally invalid.
    InvalidNetClass(String),
    /// A differential-pair identity already exists.
    DuplicateDifferentialPair(String),
    /// A differential-pair declaration is structurally invalid.
    InvalidDifferentialPair(String),
    /// A route-length tuning identity already exists.
    DuplicateLengthTuningPattern(String),
    /// A route-length tuning declaration is structurally invalid.
    InvalidLengthTuningPattern(String),
    /// A phase-tuning group identity already exists.
    DuplicatePhaseTuningGroup(String),
    /// A phase-tuning group declaration is structurally invalid.
    InvalidPhaseTuningGroup(String),
    /// A typed net or pin handle belongs to another design.
    ForeignHandle,
    /// Existing circuit mutation rejected the request.
    Circuit(CircuitError),
    /// Portable package artifact rejected the request.
    #[cfg(feature = "interchange")]
    Package(PackageResolutionError),
    /// Final retained circuit or layout validation rejected the design.
    InvalidDesign(DesignCheckReport),
}

impl Display for DesignBuildError {
    fn fmt(&self, formatter: &mut Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::InvalidIdentifier => formatter.write_str("authoring identifier is empty"),
            Self::DuplicateNet(net) => write!(formatter, "duplicate authored net {net}"),
            Self::EmptyBus(bus) => write!(formatter, "authored bus {bus} has no members"),
            Self::DuplicateBusNet { bus, net } => {
                write!(formatter, "authored bus {bus} repeats net {net}")
            }
            Self::DuplicateBus(bus) => write!(formatter, "duplicate authored bus {bus}"),
            Self::UnknownBus(bus) => write!(formatter, "authored bus {bus} is no longer retained"),
            Self::DuplicateBusSlice(slice) => {
                write!(formatter, "duplicate authored bus slice {slice}")
            }
            Self::InvalidBusSlice(slice) => {
                write!(formatter, "authored bus slice {slice} has an invalid range")
            }
            Self::DuplicatePort(port) => write!(formatter, "duplicate authored port {port}"),
            Self::DuplicateInstance(instance) => {
                write!(formatter, "duplicate authored instance {instance}")
            }
            Self::DuplicatePartDefinition(definition) => {
                write!(formatter, "duplicate authored part definition {definition}")
            }
            Self::DuplicatePin { instance, pin } => {
                write!(
                    formatter,
                    "instance {instance} declares pin {pin} more than once"
                )
            }
            Self::DuplicatePad { instance, pad } => {
                write!(
                    formatter,
                    "instance {instance} footprint declares pad {pad} more than once"
                )
            }
            Self::DuplicateSymbolPin { instance, pin } => {
                write!(
                    formatter,
                    "instance {instance} schematic symbol declares pin {pin} more than once"
                )
            }
            Self::UnknownSymbolPin { instance, pin } => {
                write!(
                    formatter,
                    "instance {instance} schematic symbol declares unknown pin {pin}"
                )
            }
            Self::DuplicatePartSymbolUnit { definition, unit } => {
                write!(
                    formatter,
                    "part definition {definition} repeats schematic symbol unit {unit}"
                )
            }
            Self::DuplicateSymbolPlacement { instance, unit } => {
                write!(
                    formatter,
                    "instance {instance} places schematic symbol unit {unit} more than once"
                )
            }
            Self::UnknownPartSymbolUnit { definition, unit } => {
                write!(
                    formatter,
                    "part definition {definition} has no schematic symbol unit {unit}"
                )
            }
            Self::MissingPartSymbolDefinition(instance) => {
                write!(
                    formatter,
                    "instance {instance} requests a schematic placement without a symbol definition"
                )
            }
            Self::MissingSymbolPin { instance, pin } => {
                write!(
                    formatter,
                    "instance {instance} schematic symbol omits required pin {pin}"
                )
            }
            Self::InvalidSymbolGeometry(instance) => {
                write!(
                    formatter,
                    "instance {instance} schematic symbol has invalid geometry"
                )
            }
            Self::DuplicateStimulus(instance) => {
                write!(
                    formatter,
                    "instance {instance} already has a source waveform"
                )
            }
            Self::InvalidStimulusTarget(instance) => {
                write!(
                    formatter,
                    "instance {instance} is not an independent voltage/current source"
                )
            }
            Self::MissingFootprint(instance) => {
                write!(formatter, "instance {instance} needs a footprint")
            }
            Self::MissingPlacement(instance) => {
                write!(
                    formatter,
                    "instance {instance} has a footprint but no placement"
                )
            }
            Self::UnknownPad { instance, pin, pad } => {
                write!(
                    formatter,
                    "instance {instance} pin {pin} maps absent footprint pad {pad}"
                )
            }
            Self::UnknownPin { instance, pin } => {
                write!(formatter, "instance {instance} has no pin {pin}")
            }
            Self::DuplicateConnection { instance, pin } => {
                write!(
                    formatter,
                    "instance {instance} pin {pin} is already connected"
                )
            }
            Self::DuplicateRoute(route) => write!(formatter, "duplicate authored route {route}"),
            Self::InvalidRoute(route) => write!(formatter, "authored route {route} is invalid"),
            Self::DuplicateVia(via) => write!(formatter, "duplicate authored via {via}"),
            Self::InvalidVia(via) => write!(formatter, "authored via {via} is invalid"),
            Self::DuplicateZone(zone) => write!(formatter, "duplicate authored zone {zone}"),
            Self::InvalidZone(zone) => write!(formatter, "authored zone {zone} is invalid"),
            Self::DuplicateKeepout(keepout) => {
                write!(formatter, "duplicate authored keepout {keepout}")
            }
            Self::InvalidKeepout(keepout) => {
                write!(formatter, "authored keepout {keepout} is invalid")
            }
            Self::DuplicatePlacementConstraint(constraint) => {
                write!(
                    formatter,
                    "duplicate authored placement constraint {constraint}"
                )
            }
            Self::InvalidPlacementConstraint(constraint) => {
                write!(
                    formatter,
                    "authored placement constraint {constraint} is invalid"
                )
            }
            Self::DuplicateViaStyle(style) => {
                write!(formatter, "duplicate authored via style {style}")
            }
            Self::InvalidViaStyle(style) => {
                write!(formatter, "authored via style {style} is invalid")
            }
            Self::DuplicateNetClass(class) => {
                write!(formatter, "duplicate authored net class {class}")
            }
            Self::InvalidNetClass(class) => {
                write!(formatter, "authored net class {class} is invalid")
            }
            Self::DuplicateDifferentialPair(pair) => {
                write!(formatter, "duplicate authored differential pair {pair}")
            }
            Self::InvalidDifferentialPair(pair) => {
                write!(formatter, "authored differential pair {pair} is invalid")
            }
            Self::DuplicateLengthTuningPattern(pattern) => {
                write!(
                    formatter,
                    "duplicate authored length-tuning pattern {pattern}"
                )
            }
            Self::InvalidLengthTuningPattern(pattern) => {
                write!(
                    formatter,
                    "authored length-tuning pattern {pattern} is invalid"
                )
            }
            Self::DuplicatePhaseTuningGroup(group) => {
                write!(formatter, "duplicate authored phase-tuning group {group}")
            }
            Self::InvalidPhaseTuningGroup(group) => {
                write!(formatter, "authored phase-tuning group {group} is invalid")
            }
            Self::ForeignHandle => {
                formatter.write_str("authoring handle belongs to another design")
            }
            Self::Circuit(error) => Display::fmt(error, formatter),
            #[cfg(feature = "interchange")]
            Self::Package(error) => Display::fmt(error, formatter),
            Self::InvalidDesign(report) => write!(
                formatter,
                "authored design has {} circuit, {} schematic, and {} layout issue(s)",
                report.circuit.issues.len(),
                report.schematic.issues.len(),
                report.layout.issues.len()
            ),
        }
    }
}

impl std::error::Error for DesignBuildError {}

impl From<CircuitError> for DesignBuildError {
    fn from(error: CircuitError) -> Self {
        Self::Circuit(error)
    }
}

#[cfg(feature = "interchange")]
impl From<PackageResolutionError> for DesignBuildError {
    fn from(error: PackageResolutionError) -> Self {
        Self::Package(error)
    }
}

/// Typed handle to one net owned by a [`Design`].
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct NetHandle {
    owner: u64,
    id: NetId,
}

impl NetHandle {
    /// Returns the retained net identity.
    pub fn id(&self) -> &NetId {
        &self.id
    }

    pub(crate) fn owner(&self) -> u64 {
        self.owner
    }
}

/// Typed handle to an ordered bus owned by a [`Design`].
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct BusHandle {
    owner: u64,
    id: BusId,
    nets: Vec<NetId>,
}

impl BusHandle {
    /// Returns the retained bus identity.
    pub fn id(&self) -> &BusId {
        &self.id
    }

    /// Returns typed handles for members in authored bus order.
    pub fn members(&self) -> Vec<NetHandle> {
        self.nets
            .iter()
            .cloned()
            .map(|id| NetHandle {
                owner: self.owner,
                id,
            })
            .collect()
    }
}

/// Typed handle to a named retained subset of a bus.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct BusSliceHandle {
    owner: u64,
    id: BusSliceId,
    nets: Vec<NetId>,
}

impl BusSliceHandle {
    /// Returns the retained bus-slice identity.
    pub fn id(&self) -> &BusSliceId {
        &self.id
    }

    /// Returns typed handles in exposed slice order.
    pub fn members(&self) -> Vec<NetHandle> {
        self.nets
            .iter()
            .cloned()
            .map(|id| NetHandle {
                owner: self.owner,
                id,
            })
            .collect()
    }
}

/// Typed handle to one circuit boundary port.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct PortHandle {
    owner: u64,
    id: PortId,
    net: NetId,
}

impl PortHandle {
    /// Returns the retained port identity.
    pub fn id(&self) -> &PortId {
        &self.id
    }

    /// Returns the typed net exposed by the port.
    pub fn net(&self) -> NetHandle {
        NetHandle {
            owner: self.owner,
            id: self.net.clone(),
        }
    }

    pub(crate) fn owner(&self) -> u64 {
        self.owner
    }
}

/// Typed handle to one routed-copper object owned by a [`Design`].
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct RouteHandle {
    owner: u64,
    id: RouteId,
}

impl RouteHandle {
    /// Returns the retained route identity.
    pub fn id(&self) -> &RouteId {
        &self.id
    }

    /// Returns whether this handle belongs to `design`.
    pub fn belongs_to(&self, design: &Design) -> bool {
        self.owner == design.owner
    }
}

/// Typed handle to one via owned by a [`Design`].
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ViaHandle {
    owner: u64,
    id: ViaId,
}

impl ViaHandle {
    /// Returns the retained via identity.
    pub fn id(&self) -> &ViaId {
        &self.id
    }

    /// Returns whether this handle belongs to `design`.
    pub fn belongs_to(&self, design: &Design) -> bool {
        self.owner == design.owner
    }
}

/// Typed handle to one copper zone owned by a [`Design`].
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ZoneHandle {
    owner: u64,
    id: ZoneId,
}

impl ZoneHandle {
    /// Returns the retained zone identity.
    pub fn id(&self) -> &ZoneId {
        &self.id
    }

    /// Returns whether this handle belongs to `design`.
    pub fn belongs_to(&self, design: &Design) -> bool {
        self.owner == design.owner
    }
}

/// Typed handle to one physical keepout owned by a [`Design`].
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct KeepoutHandle {
    owner: u64,
    id: KeepoutId,
}

impl KeepoutHandle {
    /// Returns the retained keepout identity.
    pub fn id(&self) -> &KeepoutId {
        &self.id
    }

    /// Returns whether this handle belongs to `design`.
    pub fn belongs_to(&self, design: &Design) -> bool {
        self.owner == design.owner
    }
}

/// Typed handle to one placement constraint owned by a [`Design`].
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct PlacementConstraintHandle {
    owner: u64,
    id: PlacementConstraintId,
}

impl PlacementConstraintHandle {
    /// Returns the retained placement-constraint identity.
    pub fn id(&self) -> &PlacementConstraintId {
        &self.id
    }

    /// Returns whether this handle belongs to `design`.
    pub fn belongs_to(&self, design: &Design) -> bool {
        self.owner == design.owner
    }
}

/// Typed handle to one named via construction owned by a [`Design`].
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ViaStyleHandle {
    owner: u64,
    id: ViaStyleId,
}

impl ViaStyleHandle {
    /// Returns the retained via-style identity.
    pub fn id(&self) -> &ViaStyleId {
        &self.id
    }

    /// Returns whether this handle belongs to `design`.
    pub fn belongs_to(&self, design: &Design) -> bool {
        self.owner == design.owner
    }
}

/// Typed handle to one named net-class policy owned by a [`Design`].
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct NetClassHandle {
    owner: u64,
    id: NetClassId,
}

impl NetClassHandle {
    /// Returns the retained net-class identity.
    pub fn id(&self) -> &NetClassId {
        &self.id
    }

    /// Returns whether this handle belongs to `design`.
    pub fn belongs_to(&self, design: &Design) -> bool {
        self.owner == design.owner
    }
}

/// Typed handle to one differential-pair declaration owned by a [`Design`].
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct DifferentialPairHandle {
    owner: u64,
    id: DifferentialPairId,
}

impl DifferentialPairHandle {
    /// Returns the retained differential-pair identity.
    pub fn id(&self) -> &DifferentialPairId {
        &self.id
    }

    /// Returns whether this handle belongs to `design`.
    pub fn belongs_to(&self, design: &Design) -> bool {
        self.owner == design.owner
    }
}

/// Typed handle to one bounded route-length tuning request owned by a [`Design`].
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct LengthTuningPatternHandle {
    owner: u64,
    id: LengthTuningPatternId,
}

impl LengthTuningPatternHandle {
    /// Returns the retained tuning-pattern identity.
    pub fn id(&self) -> &LengthTuningPatternId {
        &self.id
    }

    /// Returns whether this handle belongs to `design`.
    pub fn belongs_to(&self, design: &Design) -> bool {
        self.owner == design.owner
    }
}

/// Typed handle to one atomic phase-tuning group owned by a [`Design`].
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct PhaseTuningGroupHandle {
    owner: u64,
    id: PhaseTuningGroupId,
}

impl PhaseTuningGroupHandle {
    /// Returns the retained phase-group identity.
    pub fn id(&self) -> &PhaseTuningGroupId {
        &self.id
    }

    /// Returns whether this handle belongs to `design`.
    pub fn belongs_to(&self, design: &Design) -> bool {
        self.owner == design.owner
    }
}

/// Typed handle to one declared instance pin.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct PinHandle {
    owner: u64,
    instance: CircuitInstanceId,
    pin: PinRef,
}

impl PinHandle {
    /// Returns the owning instance identity.
    pub fn instance(&self) -> &CircuitInstanceId {
        &self.instance
    }

    /// Returns the retained pin identity.
    pub fn pin(&self) -> &PinRef {
        &self.pin
    }
}

/// Typed handle returned after adding a part.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct InstanceHandle {
    owner: u64,
    id: CircuitInstanceId,
    pins: BTreeSet<PinRef>,
}

impl InstanceHandle {
    /// Returns the retained instance identity.
    pub fn id(&self) -> &CircuitInstanceId {
        &self.id
    }

    /// Selects one declared pin without deferring a string lookup to validation.
    pub fn pin(&self, pin: impl Into<String>) -> Result<PinHandle, DesignBuildError> {
        let pin = PinRef::new(pin.into()).map_err(|_| DesignBuildError::InvalidIdentifier)?;
        if !self.pins.contains(&pin) {
            return Err(DesignBuildError::UnknownPin {
                instance: self.id.as_str().into(),
                pin: pin.as_str().into(),
            });
        }
        Ok(PinHandle {
            owner: self.owner,
            instance: self.id.clone(),
            pin,
        })
    }
}

/// Fluent logical pin declaration with optional physical-pad mappings.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct PartPin {
    name: String,
    kind: PinElectricalKind,
    optional: bool,
    pads: Vec<String>,
    source: SourceLocation,
}

impl PartPin {
    /// Creates a required passive pin.
    #[track_caller]
    pub fn new(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            kind: PinElectricalKind::Passive,
            optional: false,
            pads: Vec::new(),
            source: SourceLocation::caller(),
        }
    }

    /// Sets the ERC electrical class.
    pub fn kind(mut self, kind: PinElectricalKind) -> Self {
        self.kind = kind;
        self
    }

    /// Marks the pin as a high-impedance input.
    pub fn input(self) -> Self {
        self.kind(PinElectricalKind::Input)
    }

    /// Marks the pin as an actively driven output.
    pub fn output(self) -> Self {
        self.kind(PinElectricalKind::Output)
    }

    /// Marks the pin as bidirectional.
    pub fn bidirectional(self) -> Self {
        self.kind(PinElectricalKind::Bidirectional)
    }

    /// Marks the pin as a supply input.
    pub fn power_input(self) -> Self {
        self.kind(PinElectricalKind::PowerInput)
    }

    /// Marks the pin as a supply source/output.
    pub fn power_output(self) -> Self {
        self.kind(PinElectricalKind::PowerOutput)
    }

    /// Marks the package terminal as intentionally unconnected.
    pub fn not_connected(self) -> Self {
        self.kind(PinElectricalKind::NotConnected).optional()
    }

    /// Marks the pin as optional at the circuit boundary.
    pub fn optional(mut self) -> Self {
        self.optional = true;
        self
    }

    /// Maps the logical pin to one physical pad. Repeating this method supports
    /// intentionally duplicated lands.
    pub fn pad(mut self, pad: impl Into<String>) -> Self {
        self.pads.push(pad.into());
        self
    }
}

/// Creates a required passive part pin.
#[track_caller]
pub fn pin(name: impl Into<String>) -> PartPin {
    PartPin::new(name)
}

/// Embedded footprint geometry used by a fluent [`Part`] declaration.
#[derive(Clone, Debug, PartialEq)]
pub struct Footprint {
    pads: Vec<LandPatternPad>,
    graphics: Vec<LandPatternGraphic>,
    body: Option<LandPatternBody>,
    models: Vec<crate::Pcb3dModelReference>,
    source: SourceLocation,
}

impl Default for Footprint {
    #[track_caller]
    fn default() -> Self {
        Self::new()
    }
}

impl Footprint {
    /// Creates an empty embedded footprint.
    #[track_caller]
    pub fn new() -> Self {
        Self {
            pads: Vec::new(),
            graphics: Vec::new(),
            body: None,
            models: Vec::new(),
            source: SourceLocation::caller(),
        }
    }

    /// Adds one fully retained physical pad.
    pub fn pad(mut self, pad: LandPatternPad) -> Self {
        self.pads.push(pad);
        self
    }

    /// Adds one footprint-local artwork primitive.
    pub fn graphic(mut self, graphic: LandPatternGraphic) -> Self {
        self.graphics.push(graphic);
        self
    }

    /// Attaches a native review body envelope.
    pub fn body(mut self, body: LandPatternBody) -> Self {
        self.body = Some(body);
        self
    }

    /// Adds one resolver-addressable external package model.
    pub fn model(mut self, model: crate::Pcb3dModelReference) -> Self {
        self.models.push(model);
        self
    }

    /// Creates a horizontal two-pad surface-mount footprint.
    ///
    /// `offset` is the absolute X distance from the footprint origin to each
    /// pad center, avoiding a hidden finite/rounding policy.
    #[track_caller]
    pub fn two_pad_smd(
        offset: Real,
        width: Real,
        height: Real,
        copper_layers: Vec<TraceLayer>,
    ) -> Self {
        let pad = |id: &str, x: Real| LandPatternPad {
            id: PadId::new(id).expect("constant pad id is nonempty"),
            center: Point2::new(x, Real::zero()),
            rotation_degrees: Real::zero(),
            copper_layers: copper_layers.clone(),
            shape: PadShape::Rectangle {
                width: width.clone(),
                height: height.clone(),
            },
            drill: None,
            plating: Plating::Unspecified,
            solder_mask_margin: None,
            paste_margin: None,
        };
        Self::new()
            .pad(pad("1", -offset.clone()))
            .pad(pad("2", offset))
    }

    /// Adds a plated round through-hole pad with a retained finished drill.
    pub fn plated_round_pad(
        self,
        id: PadId,
        center: Point2,
        copper_layers: Vec<TraceLayer>,
        land_diameter: Real,
        drill_diameter: Real,
    ) -> Self {
        self.pad(LandPatternPad {
            id,
            center,
            rotation_degrees: Real::zero(),
            copper_layers,
            shape: PadShape::Circle {
                diameter: land_diameter,
            },
            drill: Some(DrillShape::Round {
                diameter: drill_diameter,
            }),
            plating: Plating::Plated,
            solder_mask_margin: None,
            paste_margin: None,
        })
    }
}

/// Fluent routed-copper declaration lowered directly into [`PcbRoute`].
#[derive(Clone, Debug, PartialEq)]
pub struct Route {
    id: String,
    layer: TraceLayer,
    width: Real,
    segments: Vec<PcbRouteSegment>,
    source: SourceLocation,
}

impl Route {
    /// Begins an exact route on one copper layer.
    #[track_caller]
    pub fn new(id: impl Into<String>, layer: TraceLayer, width: Real) -> Self {
        Self {
            id: id.into(),
            layer,
            width,
            segments: Vec::new(),
            source: SourceLocation::caller(),
        }
    }

    /// Appends one already-certified hyperpath segment.
    pub fn segment(mut self, segment: impl Into<PcbRouteSegment>) -> Self {
        self.segments.push(segment.into());
        self
    }

    /// Appends one exact straight centerline segment.
    pub fn line(self, start: Point2, end: Point2) -> Self {
        self.segment(LinePathSegment::new(start, end))
    }

    /// Appends one exact directed circular arc.
    pub fn circular_arc(self, arc: ExplicitCircularArc) -> Self {
        self.segment(arc)
    }

    /// Appends one exact cubic Bezier segment.
    pub fn cubic_bezier(self, bezier: CubicBezier) -> Self {
        self.segment(bezier)
    }
}

/// Fluent via declaration lowered directly into [`PcbVia`].
#[derive(Clone, Debug, PartialEq)]
pub struct Via {
    id: String,
    start_layer: TraceLayer,
    end_layer: TraceLayer,
    center: Point2,
    land_diameter: Real,
    drill_diameter: Real,
    plating: Plating,
    mask: ViaMaskIntent,
    source: SourceLocation,
}

impl Via {
    /// Declares a plated layer transition with retained geometry.
    #[track_caller]
    pub fn new(
        id: impl Into<String>,
        start_layer: TraceLayer,
        end_layer: TraceLayer,
        center: Point2,
        land_diameter: Real,
        drill_diameter: Real,
    ) -> Self {
        Self {
            id: id.into(),
            start_layer,
            end_layer,
            center,
            land_diameter,
            drill_diameter,
            plating: Plating::Plated,
            mask: ViaMaskIntent::default(),
            source: SourceLocation::caller(),
        }
    }

    /// Overrides retained plating intent.
    pub fn plating(mut self, plating: Plating) -> Self {
        self.plating = plating;
        self
    }

    /// Selects independent front/back solder-mask treatment.
    pub fn mask(mut self, mask: ViaMaskIntent) -> Self {
        self.mask = mask;
        self
    }
}

/// Fluent copper-pour declaration lowered directly into [`CopperZone`].
#[derive(Clone, Debug, PartialEq)]
pub struct Zone {
    id: String,
    layer: TraceLayer,
    boundary: Vec<Point2>,
    clearance: Real,
    fill: CopperZoneFill,
    connection: CopperZoneConnection,
    islands: CopperZoneIslandPolicy,
    stitching: Option<CopperZoneStitchingPolicy>,
    priority: i32,
    source: SourceLocation,
}

impl Zone {
    /// Declares a solid, directly connected copper zone.
    #[track_caller]
    pub fn solid(id: impl Into<String>, layer: TraceLayer, boundary: Vec<Point2>) -> Self {
        Self {
            id: id.into(),
            layer,
            boundary,
            clearance: Real::zero(),
            fill: CopperZoneFill::Solid,
            connection: CopperZoneConnection::Solid,
            islands: CopperZoneIslandPolicy::retain_all(),
            stitching: None,
            priority: 0,
            source: SourceLocation::caller(),
        }
    }

    /// Sets exact foreign-net copper clearance.
    pub fn clearance(mut self, clearance: Real) -> Self {
        self.clearance = clearance;
        self
    }

    /// Selects solid or hatched fill intent.
    pub fn fill(mut self, fill: CopperZoneFill) -> Self {
        self.fill = fill;
        self
    }

    /// Selects same-net pad/via connection intent.
    pub fn connection(mut self, connection: CopperZoneConnection) -> Self {
        self.connection = connection;
        self
    }

    /// Selects disconnected-island retention or pruning.
    pub fn islands(mut self, islands: CopperZoneIslandPolicy) -> Self {
        self.islands = islands;
        self
    }

    /// Requests a bounded retained stitching-via grid.
    pub fn stitching(mut self, stitching: CopperZoneStitchingPolicy) -> Self {
        self.stitching = Some(stitching);
        self
    }

    /// Sets overlap priority; higher values claim copper first.
    pub fn priority(mut self, priority: i32) -> Self {
        self.priority = priority;
        self
    }
}

/// Fluent physical keepout declaration lowered directly into [`PcbKeepout`].
#[derive(Clone, Debug, PartialEq)]
pub struct Keepout {
    id: String,
    boundary: Vec<Point2>,
    scope: KeepoutScope,
    source: SourceLocation,
}

impl Keepout {
    /// Declares a closed keepout boundary with a typed feature scope.
    #[track_caller]
    pub fn new(id: impl Into<String>, boundary: Vec<Point2>, scope: KeepoutScope) -> Self {
        Self {
            id: id.into(),
            boundary,
            scope,
            source: SourceLocation::caller(),
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
enum PlacementRuleKind {
    Fixed {
        instance: InstanceHandle,
        position: Point2,
    },
    Relative {
        instance: InstanceHandle,
        anchor: InstanceHandle,
        offset: Point2,
    },
    AlignX {
        instances: Vec<InstanceHandle>,
    },
    AlignY {
        instances: Vec<InstanceHandle>,
    },
    Within {
        instance: InstanceHandle,
        min: Point2,
        max: Point2,
    },
    AllowedRotations {
        instance: InstanceHandle,
        rotations_degrees: Vec<Real>,
    },
    AllowedSides {
        instance: InstanceHandle,
        sides: Vec<BoardSide>,
    },
}

/// Fluent typed placement equation or predicate.
#[derive(Clone, Debug, PartialEq)]
pub struct PlacementRule {
    id: String,
    kind: PlacementRuleKind,
    source: SourceLocation,
}

impl PlacementRule {
    /// Pins one placed instance to an exact board coordinate.
    #[track_caller]
    pub fn fixed(id: impl Into<String>, instance: &InstanceHandle, position: Point2) -> Self {
        Self {
            id: id.into(),
            kind: PlacementRuleKind::Fixed {
                instance: instance.clone(),
                position,
            },
            source: SourceLocation::caller(),
        }
    }

    /// Places one instance at an exact offset from another.
    #[track_caller]
    pub fn relative(
        id: impl Into<String>,
        instance: &InstanceHandle,
        anchor: &InstanceHandle,
        offset: Point2,
    ) -> Self {
        Self {
            id: id.into(),
            kind: PlacementRuleKind::Relative {
                instance: instance.clone(),
                anchor: anchor.clone(),
                offset,
            },
            source: SourceLocation::caller(),
        }
    }

    /// Requires all selected instance origins to share an X coordinate.
    #[track_caller]
    pub fn align_x<'a, I>(id: impl Into<String>, instances: I) -> Self
    where
        I: IntoIterator<Item = &'a InstanceHandle>,
    {
        Self {
            id: id.into(),
            kind: PlacementRuleKind::AlignX {
                instances: instances.into_iter().cloned().collect(),
            },
            source: SourceLocation::caller(),
        }
    }

    /// Requires all selected instance origins to share a Y coordinate.
    #[track_caller]
    pub fn align_y<'a, I>(id: impl Into<String>, instances: I) -> Self
    where
        I: IntoIterator<Item = &'a InstanceHandle>,
    {
        Self {
            id: id.into(),
            kind: PlacementRuleKind::AlignY {
                instances: instances.into_iter().cloned().collect(),
            },
            source: SourceLocation::caller(),
        }
    }

    /// Requires one instance origin inside an inclusive exact region.
    #[track_caller]
    pub fn within(
        id: impl Into<String>,
        instance: &InstanceHandle,
        min: Point2,
        max: Point2,
    ) -> Self {
        Self {
            id: id.into(),
            kind: PlacementRuleKind::Within {
                instance: instance.clone(),
                min,
                max,
            },
            source: SourceLocation::caller(),
        }
    }

    /// Restricts one placement to exact allowed rotations.
    #[track_caller]
    pub fn allowed_rotations(
        id: impl Into<String>,
        instance: &InstanceHandle,
        rotations_degrees: Vec<Real>,
    ) -> Self {
        Self {
            id: id.into(),
            kind: PlacementRuleKind::AllowedRotations {
                instance: instance.clone(),
                rotations_degrees,
            },
            source: SourceLocation::caller(),
        }
    }

    /// Restricts one placement to one or both board sides.
    #[track_caller]
    pub fn allowed_sides(
        id: impl Into<String>,
        instance: &InstanceHandle,
        sides: Vec<BoardSide>,
    ) -> Self {
        Self {
            id: id.into(),
            kind: PlacementRuleKind::AllowedSides {
                instance: instance.clone(),
                sides,
            },
            source: SourceLocation::caller(),
        }
    }
}

/// Fluent named via construction selected by net classes.
#[derive(Clone, Debug, PartialEq)]
pub struct ViaStyleRule {
    id: String,
    land_diameter: Real,
    drill_diameter: Real,
    plating: Plating,
    mask: ViaMaskIntent,
    allowed_spans: Vec<ViaStyleSpan>,
    source: SourceLocation,
}

impl ViaStyleRule {
    /// Declares a plated via construction with no span restriction.
    #[track_caller]
    pub fn new(id: impl Into<String>, land_diameter: Real, drill_diameter: Real) -> Self {
        Self {
            id: id.into(),
            land_diameter,
            drill_diameter,
            plating: Plating::Plated,
            mask: ViaMaskIntent::default(),
            allowed_spans: Vec::new(),
            source: SourceLocation::caller(),
        }
    }

    /// Overrides retained plating intent.
    pub fn plating(mut self, plating: Plating) -> Self {
        self.plating = plating;
        self
    }

    /// Selects independent front/back solder-mask treatment.
    pub fn mask(mut self, mask: ViaMaskIntent) -> Self {
        self.mask = mask;
        self
    }

    /// Adds one allowed inclusive conductor-layer span.
    pub fn span(mut self, start_layer: TraceLayer, end_layer: TraceLayer) -> Self {
        self.allowed_spans.push(ViaStyleSpan {
            start_layer,
            end_layer,
        });
        self
    }
}

/// Fluent typed net-class declaration.
#[derive(Clone, Debug, PartialEq)]
pub struct NetClassRule {
    id: String,
    parent: Option<NetClassHandle>,
    nets: Vec<NetHandle>,
    min_trace_width: Option<Real>,
    preferred_trace_width: Option<Real>,
    min_clearance: Option<Real>,
    preferred_via_land_diameter: Option<Real>,
    preferred_via_drill_diameter: Option<Real>,
    preferred_via_style: Option<ViaStyleHandle>,
    max_length: Option<Real>,
    max_via_count: Option<usize>,
    target_impedance_ohms: Option<Real>,
    impedance_tolerance_ohms: Option<Real>,
    requires_reference_plane: bool,
    source: SourceLocation,
}

impl NetClassRule {
    /// Declares policy for an authored set of typed nets.
    #[track_caller]
    pub fn new<'a, I>(id: impl Into<String>, nets: I) -> Self
    where
        I: IntoIterator<Item = &'a NetHandle>,
    {
        Self {
            id: id.into(),
            parent: None,
            nets: nets.into_iter().cloned().collect(),
            min_trace_width: None,
            preferred_trace_width: None,
            min_clearance: None,
            preferred_via_land_diameter: None,
            preferred_via_drill_diameter: None,
            preferred_via_style: None,
            max_length: None,
            max_via_count: None,
            target_impedance_ohms: None,
            impedance_tolerance_ohms: None,
            requires_reference_plane: false,
            source: SourceLocation::caller(),
        }
    }

    /// Inherits unspecified policy from a previously declared class.
    pub fn parent(mut self, parent: &NetClassHandle) -> Self {
        self.parent = Some(parent.clone());
        self
    }

    /// Sets the exact minimum trace width.
    pub fn min_trace_width(mut self, width: Real) -> Self {
        self.min_trace_width = Some(width);
        self
    }

    /// Sets the exact preferred trace width.
    pub fn preferred_trace_width(mut self, width: Real) -> Self {
        self.preferred_trace_width = Some(width);
        self
    }

    /// Sets the exact minimum foreign-net clearance.
    pub fn min_clearance(mut self, clearance: Real) -> Self {
        self.min_clearance = Some(clearance);
        self
    }

    /// Sets preferred via land and finished-drill dimensions.
    pub fn preferred_via_geometry(mut self, land_diameter: Real, drill_diameter: Real) -> Self {
        self.preferred_via_land_diameter = Some(land_diameter);
        self.preferred_via_drill_diameter = Some(drill_diameter);
        self
    }

    /// Selects a typed named via construction.
    pub fn preferred_via_style(mut self, style: &ViaStyleHandle) -> Self {
        self.preferred_via_style = Some(style.clone());
        self
    }

    /// Sets the maximum exact routed length.
    pub fn max_length(mut self, length: Real) -> Self {
        self.max_length = Some(length);
        self
    }

    /// Sets the maximum routed via count.
    pub fn max_via_count(mut self, count: usize) -> Self {
        self.max_via_count = Some(count);
        self
    }

    /// Sets target impedance and allowed absolute deviation.
    pub fn impedance(mut self, target_ohms: Real, tolerance_ohms: Real) -> Self {
        self.target_impedance_ohms = Some(target_ohms);
        self.impedance_tolerance_ohms = Some(tolerance_ohms);
        self
    }

    /// Requires a qualified reference plane during release checks.
    pub fn reference_plane(mut self) -> Self {
        self.requires_reference_plane = true;
        self
    }
}

/// Fluent typed complementary-net routing declaration.
#[derive(Clone, Debug, PartialEq)]
pub struct DifferentialPairRule {
    id: String,
    positive: NetHandle,
    negative: NetHandle,
    spacing: Real,
    max_skew: Option<Real>,
    target_impedance_ohms: Option<Real>,
    impedance_tolerance_ohms: Option<Real>,
    neckdown: Option<crate::DifferentialPairNeckdown>,
    source: SourceLocation,
}

impl DifferentialPairRule {
    /// Couples two typed nets with exact edge spacing.
    #[track_caller]
    pub fn new(
        id: impl Into<String>,
        positive: &NetHandle,
        negative: &NetHandle,
        spacing: Real,
    ) -> Self {
        Self {
            id: id.into(),
            positive: positive.clone(),
            negative: negative.clone(),
            spacing,
            max_skew: None,
            target_impedance_ohms: None,
            impedance_tolerance_ohms: None,
            neckdown: None,
            source: SourceLocation::caller(),
        }
    }

    /// Sets the maximum exact routed-length skew.
    pub fn max_skew(mut self, max_skew: Real) -> Self {
        self.max_skew = Some(max_skew);
        self
    }

    /// Sets target differential impedance and its allowed absolute deviation.
    pub fn impedance(mut self, target_ohms: Real, tolerance_ohms: Real) -> Self {
        self.target_impedance_ohms = Some(target_ohms);
        self.impedance_tolerance_ohms = Some(tolerance_ohms);
        self
    }

    /// Allows a bounded symmetric terminal fanout from reduced pair spacing.
    pub fn neckdown(
        mut self,
        trace_width: Real,
        spacing: Real,
        maximum_transition_length: Real,
    ) -> Self {
        self.neckdown = Some(crate::DifferentialPairNeckdown {
            trace_width,
            spacing,
            maximum_transition_length,
        });
        self
    }
}

/// Fluent bounded serpentine request for one typed routed net.
#[derive(Clone, Debug, PartialEq)]
pub struct LengthTuningRule {
    id: String,
    net: NetHandle,
    route: Option<RouteHandle>,
    region: Vec<Point2>,
    target_length: Real,
    tolerance: Real,
    amplitude: Real,
    pitch: Real,
    maximum_cycles: usize,
    side: LengthTuningSide,
    source: SourceLocation,
}

impl LengthTuningRule {
    /// Targets an exact total routed length inside one board-space region.
    #[track_caller]
    pub fn new(
        id: impl Into<String>,
        net: &NetHandle,
        region: Vec<Point2>,
        target_length: Real,
        amplitude: Real,
        pitch: Real,
        maximum_cycles: usize,
    ) -> Self {
        Self {
            id: id.into(),
            net: net.clone(),
            route: None,
            region,
            target_length,
            tolerance: Real::zero(),
            amplitude,
            pitch,
            maximum_cycles,
            side: LengthTuningSide::Left,
            source: SourceLocation::caller(),
        }
    }

    /// Restricts realization to one typed retained route.
    pub fn route(mut self, route: &RouteHandle) -> Self {
        self.route = Some(route.clone());
        self
    }

    /// Accepts this absolute exact deviation from the target.
    pub fn tolerance(mut self, tolerance: Real) -> Self {
        self.tolerance = tolerance;
        self
    }

    /// Selects the directed side used for generated excursions.
    pub fn side(mut self, side: LengthTuningSide) -> Self {
        self.side = side;
        self
    }
}

/// Fluent atomic phase/length-matching declaration.
#[derive(Clone, Debug, PartialEq)]
pub struct PhaseTuningGroupRule {
    id: String,
    patterns: Vec<LengthTuningPatternHandle>,
    differential_pair: Option<DifferentialPairHandle>,
    maximum_skew: Real,
    minimum_clearance: Real,
    source: SourceLocation,
}

impl PhaseTuningGroupRule {
    /// Groups typed tuning requests behind one all-or-nothing boundary.
    #[track_caller]
    pub fn new(
        id: impl Into<String>,
        patterns: impl IntoIterator<Item = LengthTuningPatternHandle>,
    ) -> Self {
        Self {
            id: id.into(),
            patterns: patterns.into_iter().collect(),
            differential_pair: None,
            maximum_skew: Real::zero(),
            minimum_clearance: Real::zero(),
            source: SourceLocation::caller(),
        }
    }

    /// Requires the result to preserve one typed differential-pair geometry.
    pub fn differential_pair(mut self, pair: &DifferentialPairHandle) -> Self {
        self.differential_pair = Some(pair.clone());
        self
    }

    /// Sets the maximum exact final centerline-length skew.
    pub fn maximum_skew(mut self, maximum_skew: Real) -> Self {
        self.maximum_skew = maximum_skew;
        self
    }

    /// Sets minimum exact edge clearance to foreign retained copper and keepouts.
    pub fn minimum_clearance(mut self, minimum_clearance: Real) -> Self {
        self.minimum_clearance = minimum_clearance;
        self
    }
}

/// One logical pin's placement on a fluent schematic symbol.
#[derive(Clone, Debug, PartialEq)]
pub struct SymbolPin {
    name: String,
    position: SchematicPoint,
    side: SchematicPinSide,
    source: SourceLocation,
}

impl SymbolPin {
    /// Places a logical pin at an exact symbol-local coordinate.
    #[track_caller]
    pub fn new(name: impl Into<String>, position: SchematicPoint, side: SchematicPinSide) -> Self {
        Self {
            name: name.into(),
            position,
            side,
            source: SourceLocation::caller(),
        }
    }
}

/// Single-unit schematic drawing attached to one fluent [`Part`].
#[derive(Clone, Debug, PartialEq)]
pub struct Symbol {
    position: SchematicPoint,
    quarter_turns: i8,
    body_width: Real,
    body_height: Real,
    pins: Vec<SymbolPin>,
    source: SourceLocation,
}

impl Symbol {
    /// Creates an empty single-unit symbol at an exact schematic position.
    #[track_caller]
    pub fn new(position: SchematicPoint, body_width: Real, body_height: Real) -> Self {
        Self {
            position,
            quarter_turns: 0,
            body_width,
            body_height,
            pins: Vec::new(),
            source: SourceLocation::caller(),
        }
    }

    /// Adds one logical pin placement.
    pub fn pin(mut self, pin: SymbolPin) -> Self {
        self.pins.push(pin);
        self
    }

    /// Applies clockwise quarter turns in drawing coordinates.
    pub fn rotated(mut self, quarter_turns: i8) -> Self {
        self.quarter_turns = quarter_turns;
        self
    }

    /// Creates the conventional horizontal drawing for a two-terminal part.
    ///
    /// `pin_offset` is the absolute X distance from the symbol center to each
    /// connection point.
    #[track_caller]
    pub fn two_pin_horizontal(
        position: SchematicPoint,
        pin_offset: Real,
        body_width: Real,
        body_height: Real,
    ) -> Self {
        Self::new(position, body_width, body_height)
            .pin(SymbolPin::new(
                "1",
                SchematicPoint::new(-pin_offset.clone(), Real::zero()),
                SchematicPinSide::Left,
            ))
            .pin(SymbolPin::new(
                "2",
                SchematicPoint::new(pin_offset, Real::zero()),
                SchematicPinSide::Right,
            ))
    }
}

/// One reusable unit in a fluent part's schematic symbol definition.
///
/// Coordinates are symbol-local; placement belongs to [`SymbolUnitPlacement`].
#[derive(Clone, Debug, PartialEq)]
pub struct PartSymbolUnit {
    unit: u16,
    body_width: Real,
    body_height: Real,
    pins: Vec<SymbolPin>,
    graphics: Vec<SchematicGraphic>,
}

impl PartSymbolUnit {
    /// Creates an empty exact symbol unit.
    pub fn new(unit: u16, body_width: Real, body_height: Real) -> Self {
        Self {
            unit,
            body_width,
            body_height,
            pins: Vec::new(),
            graphics: Vec::new(),
        }
    }

    /// Adds one logical pin placement.
    pub fn pin(mut self, pin: SymbolPin) -> Self {
        self.pins.push(pin);
        self
    }

    /// Adds one exact reusable drawing primitive.
    pub fn graphic(mut self, graphic: SchematicGraphic) -> Self {
        self.graphics.push(graphic);
        self
    }

    /// Adds the conventional filled rectangular body for this unit.
    pub fn rectangular_body(mut self, stroke_width: Real) -> Result<Self, DesignBuildError> {
        let half_width = (self.body_width.clone() / Real::from(2))
            .map_err(|_| DesignBuildError::InvalidSymbolGeometry(format!("unit {}", self.unit)))?;
        let half_height = (self.body_height.clone() / Real::from(2))
            .map_err(|_| DesignBuildError::InvalidSymbolGeometry(format!("unit {}", self.unit)))?;
        self.graphics.push(SchematicGraphic::Rectangle {
            start: SchematicPoint::new(-half_width.clone(), -half_height.clone()),
            end: SchematicPoint::new(half_width, half_height),
            stroke_width,
            fill: SchematicGraphicFill::Background,
        });
        Ok(self)
    }
}

/// Reusable electrical, schematic, and physical library definition.
///
/// A definition lowers once into a [`DeviceModel`], optional
/// [`SchematicSymbolDefinition`], and optional [`LandPattern`]. It contains no
/// instance value, net binding, or placement.
#[derive(Clone, Debug, PartialEq)]
pub struct PartDefinition {
    id: String,
    description: String,
    part: Option<String>,
    model_kind: DeviceModelKind,
    pins: Vec<PartPin>,
    model_parameters: Vec<CircuitParameter>,
    symbol_name: Option<String>,
    symbol_units: Vec<PartSymbolUnit>,
    footprint: Option<Footprint>,
    source: SourceLocation,
}

impl PartDefinition {
    /// Creates a reusable custom part definition.
    #[track_caller]
    pub fn new(id: impl Into<String>, description: impl Into<String>) -> Self {
        let description = description.into();
        Self {
            id: id.into(),
            model_kind: DeviceModelKind::Custom(description.clone()),
            description,
            part: None,
            pins: Vec::new(),
            model_parameters: Vec::new(),
            symbol_name: None,
            symbol_units: Vec::new(),
            footprint: None,
            source: SourceLocation::caller(),
        }
    }

    /// Selects an executable or custom device-model family.
    pub fn model_kind(mut self, kind: DeviceModelKind) -> Self {
        self.model_kind = kind;
        self
    }

    /// Attaches a stable external parts-library reference shared by instances.
    pub fn part_ref(mut self, part: impl Into<String>) -> Self {
        self.part = Some(part.into());
        self
    }

    /// Adds one reusable logical pin declaration.
    pub fn pin(mut self, pin: PartPin) -> Self {
        self.pins.push(pin);
        self
    }

    /// Adds an exact model parameter shared by every instance.
    pub fn model_parameter(
        mut self,
        name: impl Into<String>,
        value: Real,
        unit: impl Into<String>,
    ) -> Self {
        self.model_parameters.push(CircuitParameter {
            name: name.into(),
            value,
            unit: unit.into(),
            source: "hypercircuit::authoring".into(),
        });
        self
    }

    /// Sets the reusable symbol's display name.
    pub fn symbol_name(mut self, name: impl Into<String>) -> Self {
        self.symbol_name = Some(name.into());
        self
    }

    /// Adds one reusable multipart schematic unit.
    pub fn symbol_unit(mut self, unit: PartSymbolUnit) -> Self {
        self.symbol_units.push(unit);
        self
    }

    /// Attaches reusable physical package geometry.
    pub fn footprint(mut self, footprint: Footprint) -> Self {
        self.footprint = Some(footprint);
        self
    }

    /// Returns the human-readable part description.
    pub fn description(&self) -> &str {
        &self.description
    }
}

/// Typed design-scoped handle to a lowered reusable part definition.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct PartDefinitionHandle {
    owner: u64,
    id: String,
    model: DeviceModelId,
    symbol: Option<SchematicSymbolDefinitionId>,
    land_pattern: Option<LandPatternId>,
    part: Option<PartRef>,
    pins: BTreeSet<PinRef>,
}

impl PartDefinitionHandle {
    /// Returns the authoring-library identity.
    pub fn id(&self) -> &str {
        &self.id
    }

    /// Returns the retained reusable device-model identity.
    pub fn model_id(&self) -> &DeviceModelId {
        &self.model
    }

    /// Returns the retained reusable symbol-definition identity, when present.
    pub fn symbol_definition_id(&self) -> Option<&SchematicSymbolDefinitionId> {
        self.symbol.as_ref()
    }

    /// Returns the retained reusable land-pattern identity, when present.
    pub fn land_pattern_id(&self) -> Option<&LandPatternId> {
        self.land_pattern.as_ref()
    }
}

/// Placement of one reusable schematic symbol unit for a part instance.
#[derive(Clone, Debug, PartialEq)]
pub struct SymbolUnitPlacement {
    unit: u16,
    position: SchematicPoint,
    quarter_turns: i8,
    source: SourceLocation,
}

impl SymbolUnitPlacement {
    /// Places one numbered symbol unit at an exact schematic coordinate.
    #[track_caller]
    pub fn new(unit: u16, position: SchematicPoint) -> Self {
        Self {
            unit,
            position,
            quarter_turns: 0,
            source: SourceLocation::caller(),
        }
    }

    /// Applies clockwise quarter turns in drawing coordinates.
    pub fn rotated(mut self, quarter_turns: i8) -> Self {
        self.quarter_turns = quarter_turns;
        self
    }
}

/// Instance-specific values and placements for a reusable [`PartDefinition`].
#[derive(Clone, Debug, PartialEq)]
pub struct PartInstance {
    reference: String,
    parameters: Vec<CircuitParameter>,
    waveform: Option<SourceWaveform>,
    waveform_source: Option<SourceLocation>,
    symbols: Vec<SymbolUnitPlacement>,
    placement: Option<(Point2, Real, BoardSide)>,
    source: SourceLocation,
    placement_source: Option<SourceLocation>,
}

impl PartInstance {
    /// Creates an unconnected instance using `reference` as stable identity.
    #[track_caller]
    pub fn new(reference: impl Into<String>) -> Self {
        Self {
            reference: reference.into(),
            parameters: Vec::new(),
            waveform: None,
            waveform_source: None,
            symbols: Vec::new(),
            placement: None,
            source: SourceLocation::caller(),
            placement_source: None,
        }
    }

    /// Adds an exact instance parameter such as resistance or capacitance.
    pub fn parameter(
        mut self,
        name: impl Into<String>,
        value: Real,
        unit: impl Into<String>,
    ) -> Self {
        self.parameters.push(CircuitParameter {
            name: name.into(),
            value,
            unit: unit.into(),
            source: "hypercircuit::authoring".into(),
        });
        self
    }

    /// Attaches retained time dependence to an independent source instance.
    #[track_caller]
    pub fn waveform(mut self, waveform: SourceWaveform) -> Self {
        self.waveform = Some(waveform);
        self.waveform_source = Some(SourceLocation::caller());
        self
    }

    /// Places one unit of the reusable schematic symbol.
    pub fn symbol(mut self, placement: SymbolUnitPlacement) -> Self {
        self.symbols.push(placement);
        self
    }

    /// Places the shared footprint on the front side with zero rotation.
    #[track_caller]
    pub fn at(mut self, position: Point2) -> Self {
        self.placement = Some((position, Real::zero(), BoardSide::Front));
        self.placement_source = Some(SourceLocation::caller());
        self
    }

    /// Places the shared footprint with an exact rotation and board side.
    #[track_caller]
    pub fn placed(mut self, position: Point2, rotation_degrees: Real, side: BoardSide) -> Self {
        self.placement = Some((position, rotation_degrees, side));
        self.placement_source = Some(SourceLocation::caller());
        self
    }
}

/// Fluent one-instance declaration lowered atomically by [`Design::add`].
#[derive(Clone, Debug, PartialEq)]
pub struct Part {
    reference: String,
    description: String,
    part: Option<String>,
    model_kind: DeviceModelKind,
    pins: Vec<PartPin>,
    parameters: Vec<CircuitParameter>,
    waveform: Option<SourceWaveform>,
    waveform_source: Option<SourceLocation>,
    symbol: Option<Symbol>,
    footprint: Option<Footprint>,
    placement: Option<(Point2, Real, BoardSide)>,
    source: SourceLocation,
    placement_source: Option<SourceLocation>,
}

impl Part {
    /// Creates a custom part using `reference` as its stable instance identity.
    #[track_caller]
    pub fn new(reference: impl Into<String>, description: impl Into<String>) -> Self {
        let description = description.into();
        Self {
            reference: reference.into(),
            model_kind: DeviceModelKind::Custom(description.clone()),
            description,
            part: None,
            pins: Vec::new(),
            parameters: Vec::new(),
            waveform: None,
            waveform_source: None,
            symbol: None,
            footprint: None,
            placement: None,
            source: SourceLocation::caller(),
            placement_source: None,
        }
    }

    /// Selects an executable or custom device-model family.
    pub fn model_kind(mut self, kind: DeviceModelKind) -> Self {
        self.model_kind = kind;
        self
    }

    /// Attaches a stable external parts-library reference.
    pub fn part_ref(mut self, part: impl Into<String>) -> Self {
        self.part = Some(part.into());
        self
    }

    /// Adds one logical pin declaration.
    pub fn pin(mut self, pin: PartPin) -> Self {
        self.pins.push(pin);
        self
    }

    /// Adds an exact instance parameter consumed by simulation or downstream policy.
    pub fn parameter(
        mut self,
        name: impl Into<String>,
        value: Real,
        unit: impl Into<String>,
    ) -> Self {
        self.parameters.push(CircuitParameter {
            name: name.into(),
            value,
            unit: unit.into(),
            source: "hypercircuit::authoring".into(),
        });
        self
    }

    /// Attaches retained time dependence to an independent voltage/current source.
    #[track_caller]
    pub fn waveform(mut self, waveform: SourceWaveform) -> Self {
        self.waveform = Some(waveform);
        self.waveform_source = Some(SourceLocation::caller());
        self
    }

    /// Attaches a single-unit schematic drawing.
    pub fn symbol(mut self, symbol: Symbol) -> Self {
        self.symbol = Some(symbol);
        self
    }

    /// Attaches embedded physical package geometry.
    pub fn footprint(mut self, footprint: Footprint) -> Self {
        self.footprint = Some(footprint);
        self
    }

    /// Places the footprint on the front side with zero rotation.
    #[track_caller]
    pub fn at(mut self, position: Point2) -> Self {
        self.placement = Some((position, Real::zero(), BoardSide::Front));
        self.placement_source = Some(SourceLocation::caller());
        self
    }

    /// Places the footprint with an exact rotation and side.
    #[track_caller]
    pub fn placed(mut self, position: Point2, rotation_degrees: Real, side: BoardSide) -> Self {
        self.placement = Some((position, rotation_degrees, side));
        self.placement_source = Some(SourceLocation::caller());
        self
    }

    /// Returns the human-readable part description.
    pub fn description(&self) -> &str {
        &self.description
    }
}

/// Common executable primitive declarations.
pub mod parts {
    use super::{Part, pin};
    use crate::{DeviceModelKind, MosfetPolarity, PinElectricalKind, PinRef};
    use hyperreal::Real;

    /// Creates a two-terminal resistor with an exact resistance in ohms.
    #[track_caller]
    pub fn resistor(reference: impl Into<String>, resistance: Real) -> Part {
        Part::new(reference, "resistor")
            .model_kind(DeviceModelKind::Resistor)
            .pin(pin("1"))
            .pin(pin("2"))
            .parameter("resistance", resistance, "ohm")
    }

    /// Creates a two-terminal capacitor with an exact capacitance in farads.
    #[track_caller]
    pub fn capacitor(reference: impl Into<String>, capacitance: Real) -> Part {
        Part::new(reference, "capacitor")
            .model_kind(DeviceModelKind::Capacitor)
            .pin(pin("1"))
            .pin(pin("2"))
            .parameter("capacitance", capacitance, "F")
    }

    /// Creates a two-terminal inductor with an exact inductance in henries.
    #[track_caller]
    pub fn inductor(reference: impl Into<String>, inductance: Real) -> Part {
        Part::new(reference, "inductor")
            .model_kind(DeviceModelKind::Inductor)
            .pin(pin("1"))
            .pin(pin("2"))
            .parameter("inductance", inductance, "H")
    }

    /// Creates an independent voltage source whose pin order is positive then negative.
    #[track_caller]
    pub fn voltage_source(reference: impl Into<String>, voltage: Real) -> Part {
        Part::new(reference, "voltage source")
            .model_kind(DeviceModelKind::VoltageSource)
            .pin(pin("pos").kind(PinElectricalKind::PowerOutput))
            .pin(pin("neg").kind(PinElectricalKind::PowerOutput))
            .parameter("voltage", voltage, "V")
    }

    /// Creates an independent current source whose pin order is positive then negative.
    #[track_caller]
    pub fn current_source(reference: impl Into<String>, current: Real) -> Part {
        Part::new(reference, "current source")
            .model_kind(DeviceModelKind::CurrentSource)
            .pin(pin("pos").kind(PinElectricalKind::PowerOutput))
            .pin(pin("neg").kind(PinElectricalKind::PowerOutput))
            .parameter("current", current, "A")
    }

    /// Creates a two-terminal Shockley diode in anode/cathode pin order.
    #[track_caller]
    pub fn diode(
        reference: impl Into<String>,
        saturation_current: Real,
        thermal_voltage: Real,
    ) -> Part {
        Part::new(reference, "Shockley diode")
            .model_kind(DeviceModelKind::Diode)
            .pin(pin("A"))
            .pin(pin("K"))
            .parameter("saturation_current", saturation_current, "A")
            .parameter("thermal_voltage", thermal_voltage, "V")
    }

    /// Creates a three-terminal, body-tied-source square-law MOSFET.
    #[track_caller]
    pub fn mosfet(
        reference: impl Into<String>,
        polarity: MosfetPolarity,
        threshold_voltage: Real,
        transconductance_parameter: Real,
        channel_length_modulation: Real,
    ) -> Part {
        let drain = PinRef::new("D").expect("constant drain pin is nonempty");
        let gate = PinRef::new("G").expect("constant gate pin is nonempty");
        let source = PinRef::new("S").expect("constant source pin is nonempty");
        Part::new(reference, "body-tied-source square-law MOSFET")
            .model_kind(DeviceModelKind::Mosfet {
                polarity,
                drain,
                gate,
                source,
            })
            .pin(pin("D"))
            .pin(pin("G").kind(PinElectricalKind::Input))
            .pin(pin("S"))
            .parameter("threshold_voltage", threshold_voltage, "V")
            .parameter(
                "transconductance_parameter",
                transconductance_parameter,
                "A/V^2",
            )
            .parameter(
                "channel_length_modulation",
                channel_length_modulation,
                "1/V",
            )
    }

    /// Creates an N-channel square-law MOSFET without channel-length modulation.
    #[track_caller]
    pub fn nmos(
        reference: impl Into<String>,
        threshold_voltage: Real,
        transconductance_parameter: Real,
    ) -> Part {
        mosfet(
            reference,
            MosfetPolarity::NChannel,
            threshold_voltage,
            transconductance_parameter,
            Real::zero(),
        )
    }

    /// Creates a P-channel square-law MOSFET without channel-length modulation.
    #[track_caller]
    pub fn pmos(
        reference: impl Into<String>,
        threshold_voltage: Real,
        transconductance_parameter: Real,
    ) -> Part {
        mosfet(
            reference,
            MosfetPolarity::PChannel,
            threshold_voltage,
            transconductance_parameter,
            Real::zero(),
        )
    }
}

/// Combined structural validation evidence for one authored design.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct DesignCheckReport {
    /// Circuit identity, hierarchy, pin, and connectivity findings.
    pub circuit: CircuitValidationReport,
    /// Schematic drawing and typed-endpoint findings.
    pub schematic: SchematicValidationReport,
    /// PCB identity, geometry-carrier, placement, and rule findings.
    pub layout: LayoutValidationReport,
    /// The same findings correlated with Rust declaration/mutation call sites.
    pub diagnostics: Vec<DesignDiagnostic>,
}

impl DesignCheckReport {
    /// Returns true only when both retained containers are structurally valid.
    pub fn is_valid(&self) -> bool {
        self.circuit.is_valid() && self.schematic.is_valid() && self.layout.is_valid()
    }
}

/// Checked retained output consumed by simulation, routing, DRC, and export.
#[derive(Clone, Debug, PartialEq)]
pub struct CheckedDesign {
    owner: u64,
    /// Authoritative logical connectivity and simulation intent.
    pub circuit: Circuit,
    /// Reviewable schematic drawing derived from the same connectivity.
    pub schematic: SchematicLayout,
    /// Authoritative PCB layout and manufacturing intent.
    pub layout: PcbLayout,
    /// Rust authoring provenance kept outside the portable semantic carriers.
    pub source_map: DesignSourceMap,
}

impl CheckedDesign {
    pub(crate) fn owner(&self) -> u64 {
        self.owner
    }

    /// Splits the checked output into its authoritative retained containers.
    pub fn into_parts(self) -> (Circuit, SchematicLayout, PcbLayout) {
        (self.circuit, self.schematic, self.layout)
    }

    /// Splits the checked output while preserving authoring provenance.
    pub fn into_parts_with_sources(self) -> (Circuit, SchematicLayout, PcbLayout, DesignSourceMap) {
        (self.circuit, self.schematic, self.layout, self.source_map)
    }
}

/// Fluent authoring surface that writes directly into retained circuit/layout intent.
#[derive(Debug, PartialEq)]
pub struct Design {
    owner: u64,
    circuit: Circuit,
    schematic: SchematicLayout,
    layout: PcbLayout,
    source_map: DesignSourceMap,
}

impl Design {
    /// Creates a static/DC design with matching circuit and board identities.
    #[track_caller]
    pub fn new(
        id: impl Into<String>,
        outline: BoardOutline,
        stackup: PcbStackup,
    ) -> Result<Self, DesignBuildError> {
        let id = id.into();
        let circuit_id =
            CircuitId::new(id.clone()).map_err(|_| DesignBuildError::InvalidIdentifier)?;
        let board_id = BoardId::new(id).map_err(|_| DesignBuildError::InvalidIdentifier)?;
        Ok(Self::from_layout(
            circuit_id,
            PcbLayout::new(board_id, outline, stackup),
        ))
    }

    /// Wraps an existing empty or partially authored layout.
    #[track_caller]
    pub fn from_layout(circuit_id: CircuitId, layout: PcbLayout) -> Self {
        let source = SourceLocation::caller();
        let board_id = layout.id.clone();
        let mut source_map = DesignSourceMap::default();
        source_map.push(
            AuthoringTarget::Circuit(circuit_id.clone()),
            AuthoringAction::Declare,
            source.clone(),
        );
        source_map.push(
            AuthoringTarget::Board(board_id),
            AuthoringAction::Declare,
            source.clone(),
        );
        source_map.push(
            AuthoringTarget::Schematic(circuit_id.clone()),
            AuthoringAction::Declare,
            source,
        );
        Self {
            owner: NEXT_DESIGN_OWNER.fetch_add(1, Ordering::Relaxed),
            circuit: Circuit::new(circuit_id, TransientPolicy::Static, AdapterKind::Dc),
            schematic: SchematicLayout::default(),
            layout,
            source_map,
        }
    }

    /// Selects the retained integration policy.
    pub fn transient_policy(mut self, policy: TransientPolicy) -> Self {
        self.circuit.transient_policy = policy;
        self
    }

    /// Selects the intended external solver/adapter family.
    pub fn adapter_policy(mut self, policy: AdapterKind) -> Self {
        self.circuit.adapter_policy = policy;
        self
    }

    /// Returns the authoritative circuit under construction.
    pub fn circuit(&self) -> &Circuit {
        &self.circuit
    }

    /// Returns the retained schematic review model under construction.
    pub fn schematic(&self) -> &SchematicLayout {
        &self.schematic
    }

    /// Returns the authoritative layout under construction.
    pub fn layout(&self) -> &PcbLayout {
        &self.layout
    }

    /// Returns Rust authoring provenance accumulated so far.
    pub fn source_map(&self) -> &DesignSourceMap {
        &self.source_map
    }

    /// Escape hatch for advanced retained circuit vocabulary.
    #[track_caller]
    pub fn circuit_mut(&mut self) -> &mut Circuit {
        self.source_map.push(
            AuthoringTarget::Circuit(self.circuit.id.clone()),
            AuthoringAction::Mutate,
            SourceLocation::caller(),
        );
        &mut self.circuit
    }

    /// Escape hatch for multi-unit symbols, sheets, labels, and routed wires.
    #[track_caller]
    pub fn schematic_mut(&mut self) -> &mut SchematicLayout {
        self.source_map.push(
            AuthoringTarget::Schematic(self.circuit.id.clone()),
            AuthoringAction::Mutate,
            SourceLocation::caller(),
        );
        &mut self.schematic
    }

    /// Escape hatch for routes, zones, rules, and other advanced PCB vocabulary.
    #[track_caller]
    pub fn layout_mut(&mut self) -> &mut PcbLayout {
        self.source_map.push(
            AuthoringTarget::Board(self.layout.id.clone()),
            AuthoringAction::Mutate,
            SourceLocation::caller(),
        );
        &mut self.layout
    }

    /// Declares a non-ground logical signal.
    #[track_caller]
    pub fn signal(&mut self, name: impl Into<String>) -> Result<NetHandle, DesignBuildError> {
        self.add_net(name.into(), false, SourceLocation::caller())
    }

    /// Declares the circuit reference net and its zero-volt rail intent.
    #[track_caller]
    pub fn ground(&mut self, name: impl Into<String>) -> Result<NetHandle, DesignBuildError> {
        let handle = self.add_net(name.into(), true, SourceLocation::caller())?;
        self.circuit.rails.push(RailIntent {
            net: handle.id.clone(),
            nominal_voltage: Some(Real::zero()),
            max_current: None,
            kind: RailKind::Ground,
        });
        Ok(handle)
    }

    /// Declares a power/reference/bias rail with exact voltage and current intent.
    #[track_caller]
    pub fn rail(
        &mut self,
        name: impl Into<String>,
        nominal_voltage: Option<Real>,
        max_current: Option<Real>,
        kind: RailKind,
    ) -> Result<NetHandle, DesignBuildError> {
        let handle = self.add_net(name.into(), false, SourceLocation::caller())?;
        self.circuit.rails.push(RailIntent {
            net: handle.id.clone(),
            nominal_voltage,
            max_current,
            kind,
        });
        Ok(handle)
    }

    fn add_net(
        &mut self,
        name: String,
        ground: bool,
        source: SourceLocation,
    ) -> Result<NetHandle, DesignBuildError> {
        let id = NetId::new(name).map_err(|_| DesignBuildError::InvalidIdentifier)?;
        if self.circuit.nets.iter().any(|net| net.id == id) {
            return Err(DesignBuildError::DuplicateNet(id.as_str().into()));
        }
        self.circuit.nets.push(Net {
            id: id.clone(),
            is_ground: ground,
        });
        self.source_map.push(
            AuthoringTarget::Net(id.clone()),
            AuthoringAction::Declare,
            source,
        );
        Ok(NetHandle {
            owner: self.owner,
            id,
        })
    }

    /// Declares an ordered nonempty bus over existing typed nets.
    #[track_caller]
    pub fn bus<'a, I>(
        &mut self,
        name: impl Into<String>,
        members: I,
    ) -> Result<BusHandle, DesignBuildError>
    where
        I: IntoIterator<Item = &'a NetHandle>,
    {
        let id = BusId::new(name.into()).map_err(|_| DesignBuildError::InvalidIdentifier)?;
        if self.circuit.buses.iter().any(|bus| bus.id == id) {
            return Err(DesignBuildError::DuplicateBus(id.as_str().into()));
        }
        let mut seen = BTreeSet::new();
        let mut nets = Vec::new();
        for member in members {
            if member.owner != self.owner {
                return Err(DesignBuildError::ForeignHandle);
            }
            if !self.circuit.nets.iter().any(|net| net.id == member.id) {
                return Err(DesignBuildError::Circuit(
                    CircuitError::MissingConnectionNet,
                ));
            }
            if !seen.insert(member.id.clone()) {
                return Err(DesignBuildError::DuplicateBusNet {
                    bus: id.as_str().into(),
                    net: member.id.as_str().into(),
                });
            }
            nets.push(member.id.clone());
        }
        if nets.is_empty() {
            return Err(DesignBuildError::EmptyBus(id.as_str().into()));
        }
        self.circuit.buses.push(Bus {
            id: id.clone(),
            nets: nets.clone(),
        });
        self.source_map.push(
            AuthoringTarget::Bus(id.clone()),
            AuthoringAction::Declare,
            SourceLocation::caller(),
        );
        Ok(BusHandle {
            owner: self.owner,
            id,
            nets,
        })
    }

    /// Declares a named slice of a typed bus.
    #[track_caller]
    pub fn bus_slice(
        &mut self,
        name: impl Into<String>,
        bus: &BusHandle,
        offset: usize,
        width: usize,
        order: BusSliceOrder,
    ) -> Result<BusSliceHandle, DesignBuildError> {
        if bus.owner != self.owner {
            return Err(DesignBuildError::ForeignHandle);
        }
        let id = BusSliceId::new(name.into()).map_err(|_| DesignBuildError::InvalidIdentifier)?;
        if self.circuit.bus_slices.iter().any(|slice| slice.id == id) {
            return Err(DesignBuildError::DuplicateBusSlice(id.as_str().into()));
        }
        let Some(retained_bus) = self
            .circuit
            .buses
            .iter()
            .find(|candidate| candidate.id == bus.id)
        else {
            return Err(DesignBuildError::UnknownBus(bus.id.as_str().into()));
        };
        let slice = BusSlice {
            id: id.clone(),
            bus: bus.id.clone(),
            offset,
            width,
            order,
        };
        let Some(nets) = slice.members(retained_bus) else {
            return Err(DesignBuildError::InvalidBusSlice(id.as_str().into()));
        };
        self.circuit.bus_slices.push(slice);
        self.source_map.push(
            AuthoringTarget::BusSlice(id.clone()),
            AuthoringAction::Declare,
            SourceLocation::caller(),
        );
        Ok(BusSliceHandle {
            owner: self.owner,
            id,
            nets,
        })
    }

    /// Declares one typed boundary port on an existing net.
    #[track_caller]
    pub fn port(
        &mut self,
        name: impl Into<String>,
        net: &NetHandle,
        direction: PortDirection,
        optional: bool,
    ) -> Result<PortHandle, DesignBuildError> {
        if net.owner != self.owner {
            return Err(DesignBuildError::ForeignHandle);
        }
        if !self
            .circuit
            .nets
            .iter()
            .any(|candidate| candidate.id == net.id)
        {
            return Err(DesignBuildError::Circuit(
                CircuitError::MissingConnectionNet,
            ));
        }
        let id = PortId::new(name.into()).map_err(|_| DesignBuildError::InvalidIdentifier)?;
        if self.circuit.ports.iter().any(|port| port.id == id) {
            return Err(DesignBuildError::DuplicatePort(id.as_str().into()));
        }
        self.circuit.ports.push(CircuitPort {
            id: id.clone(),
            net: net.id.clone(),
            direction,
            optional,
        });
        self.source_map.push(
            AuthoringTarget::Port(id.clone()),
            AuthoringAction::Declare,
            SourceLocation::caller(),
        );
        Ok(PortHandle {
            owner: self.owner,
            id,
            net: net.id.clone(),
        })
    }

    /// Atomically lowers one reusable part library definition.
    ///
    /// Device, symbol, and land-pattern records are inserted exactly once and
    /// addressed by the returned design-scoped handle.
    #[track_caller]
    pub fn define_part(
        &mut self,
        definition: PartDefinition,
    ) -> Result<PartDefinitionHandle, DesignBuildError> {
        let PartDefinition {
            id,
            description,
            part,
            model_kind,
            pins,
            model_parameters,
            symbol_name,
            symbol_units,
            footprint,
            source,
        } = definition;
        let model_id = DeviceModelId::new(format!("{id}.model"))
            .map_err(|_| DesignBuildError::InvalidIdentifier)?;
        let symbol_id = SchematicSymbolDefinitionId::new(format!("{id}.symbol"))
            .map_err(|_| DesignBuildError::InvalidIdentifier)?;
        let pattern_id = LandPatternId::new(format!("{id}.footprint"))
            .map_err(|_| DesignBuildError::InvalidIdentifier)?;
        if self
            .circuit
            .device_models
            .iter()
            .any(|model| model.id == model_id)
            || self
                .schematic
                .symbol_definitions
                .iter()
                .any(|symbol| symbol.id == symbol_id)
            || self
                .layout
                .land_patterns
                .iter()
                .any(|pattern| pattern.id == pattern_id)
        {
            return Err(DesignBuildError::DuplicatePartDefinition(id));
        }

        let part = part
            .map(PartRef::new)
            .transpose()
            .map_err(|_| DesignBuildError::InvalidIdentifier)?;
        let mut pin_ids = BTreeSet::new();
        let mut device_pins = Vec::new();
        let mut mappings = Vec::new();
        for declared in pins {
            let pin =
                PinRef::new(declared.name).map_err(|_| DesignBuildError::InvalidIdentifier)?;
            if !pin_ids.insert(pin.clone()) {
                return Err(DesignBuildError::DuplicatePin {
                    instance: id.clone(),
                    pin: pin.as_str().into(),
                });
            }
            for pad in declared.pads {
                mappings.push(PadPinMap {
                    pin: pin.clone(),
                    pad: PadId::new(pad).map_err(|_| DesignBuildError::InvalidIdentifier)?,
                });
            }
            device_pins.push(DevicePin {
                pin,
                kind: declared.kind,
                optional: declared.optional,
            });
        }

        let retained_symbol = if symbol_units.is_empty() {
            None
        } else {
            let mut unit_numbers = BTreeSet::new();
            let mut presented_pins = BTreeSet::new();
            let mut units = Vec::new();
            for unit in symbol_units {
                if unit.unit == 0 || !unit_numbers.insert(unit.unit) {
                    return Err(DesignBuildError::DuplicatePartSymbolUnit {
                        definition: id.clone(),
                        unit: unit.unit,
                    });
                }
                let mut unit_pins = BTreeSet::new();
                let mut placements = Vec::new();
                for declared in unit.pins {
                    let pin = PinRef::new(declared.name)
                        .map_err(|_| DesignBuildError::InvalidIdentifier)?;
                    if !unit_pins.insert(pin.clone()) {
                        return Err(DesignBuildError::DuplicateSymbolPin {
                            instance: id.clone(),
                            pin: pin.as_str().into(),
                        });
                    }
                    if !pin_ids.contains(&pin) {
                        return Err(DesignBuildError::UnknownSymbolPin {
                            instance: id.clone(),
                            pin: pin.as_str().into(),
                        });
                    }
                    presented_pins.insert(pin.clone());
                    placements.push(SchematicPinPlacement {
                        pin,
                        position: declared.position,
                        side: declared.side,
                    });
                }
                units.push(SchematicSymbolUnit {
                    unit: unit.unit,
                    body_width: unit.body_width,
                    body_height: unit.body_height,
                    pins: placements,
                    graphics: unit.graphics,
                });
            }
            for pin in device_pins.iter().filter(|pin| !pin.optional) {
                if !presented_pins.contains(&pin.pin) {
                    return Err(DesignBuildError::MissingSymbolPin {
                        instance: id.clone(),
                        pin: pin.pin.as_str().into(),
                    });
                }
            }
            Some(SchematicSymbolDefinition {
                id: symbol_id.clone(),
                model: model_id.clone(),
                name: symbol_name.unwrap_or_else(|| description.clone()),
                units,
            })
        };

        let retained_pattern = match footprint {
            Some(footprint) => {
                let mut pad_ids = BTreeSet::new();
                for pad in &footprint.pads {
                    if !pad_ids.insert(pad.id.clone()) {
                        return Err(DesignBuildError::DuplicatePad {
                            instance: id.clone(),
                            pad: pad.id.as_str().into(),
                        });
                    }
                }
                for pin in &device_pins {
                    if !mappings.iter().any(|mapping| mapping.pin == pin.pin)
                        && let Some(pad) =
                            pad_ids.iter().find(|pad| pad.as_str() == pin.pin.as_str())
                    {
                        mappings.push(PadPinMap {
                            pin: pin.pin.clone(),
                            pad: pad.clone(),
                        });
                    }
                }
                for mapping in &mappings {
                    if !pad_ids.contains(&mapping.pad) {
                        return Err(DesignBuildError::UnknownPad {
                            instance: id.clone(),
                            pin: mapping.pin.as_str().into(),
                            pad: mapping.pad.as_str().into(),
                        });
                    }
                }
                Some((
                    LandPattern {
                        id: pattern_id.clone(),
                        pads: footprint.pads,
                        pin_map: mappings,
                        graphics: footprint.graphics,
                        body: footprint.body,
                        models: footprint.models,
                    },
                    footprint.source,
                ))
            }
            None if mappings.is_empty() => None,
            None => return Err(DesignBuildError::MissingFootprint(id)),
        };

        self.circuit.device_models.push(DeviceModel {
            id: model_id.clone(),
            kind: model_kind,
            pins: device_pins,
            parameters: model_parameters,
        });
        self.source_map.push(
            AuthoringTarget::DeviceModel(model_id.clone()),
            AuthoringAction::Declare,
            source.clone(),
        );
        let retained_symbol_id = retained_symbol.as_ref().map(|symbol| symbol.id.clone());
        if let Some(symbol) = retained_symbol {
            self.source_map.push(
                AuthoringTarget::SymbolDefinition(symbol.id.clone()),
                AuthoringAction::Declare,
                source.clone(),
            );
            self.schematic.symbol_definitions.push(symbol);
        }
        let retained_pattern_id = retained_pattern
            .as_ref()
            .map(|(pattern, _)| pattern.id.clone());
        if let Some((pattern, footprint_source)) = retained_pattern {
            for pad in &pattern.pads {
                self.source_map.push(
                    AuthoringTarget::Pad {
                        land_pattern: pattern.id.clone(),
                        pad: pad.id.clone(),
                    },
                    AuthoringAction::Declare,
                    footprint_source.clone(),
                );
            }
            self.source_map.push(
                AuthoringTarget::LandPattern(pattern.id.clone()),
                AuthoringAction::Declare,
                footprint_source,
            );
            self.layout.land_patterns.push(pattern);
        }
        Ok(PartDefinitionHandle {
            owner: self.owner,
            id,
            model: model_id,
            symbol: retained_symbol_id,
            land_pattern: retained_pattern_id,
            part,
            pins: pin_ids,
        })
    }

    /// Exports one authored reusable definition as a portable package entry.
    #[cfg(feature = "interchange")]
    pub fn export_part(
        &self,
        definition: &PartDefinitionHandle,
    ) -> Result<PortablePartDefinition, DesignBuildError> {
        if definition.owner != self.owner {
            return Err(DesignBuildError::ForeignHandle);
        }
        let model = self
            .circuit
            .device_models
            .iter()
            .find(|model| model.id == definition.model)
            .cloned()
            .ok_or(DesignBuildError::ForeignHandle)?;
        let symbol = definition
            .symbol
            .as_ref()
            .map(|id| {
                self.schematic
                    .symbol_definitions
                    .iter()
                    .find(|symbol| symbol.id == *id)
                    .cloned()
                    .ok_or(DesignBuildError::ForeignHandle)
            })
            .transpose()?;
        let land_pattern = definition
            .land_pattern
            .as_ref()
            .map(|id| {
                self.layout
                    .land_patterns
                    .iter()
                    .find(|pattern| pattern.id == *id)
                    .cloned()
                    .ok_or(DesignBuildError::ForeignHandle)
            })
            .transpose()?;
        let portable = PortablePartDefinition {
            name: definition.id.clone(),
            part: definition.part.clone(),
            model,
            symbol,
            land_pattern,
        };
        portable.validate()?;
        Ok(portable)
    }

    /// Imports one already-verified portable package definition.
    ///
    /// The artifact's retained model, symbol, and land pattern are inserted
    /// directly; no parallel library representation is created.
    #[cfg(feature = "interchange")]
    #[track_caller]
    pub fn import_part(
        &mut self,
        definition: &PortablePartDefinition,
    ) -> Result<PartDefinitionHandle, DesignBuildError> {
        definition.validate()?;
        let source = SourceLocation::caller();
        let model_id = definition.model.id.clone();
        let symbol_id = definition.symbol.as_ref().map(|symbol| symbol.id.clone());
        let pattern_id = definition
            .land_pattern
            .as_ref()
            .map(|pattern| pattern.id.clone());
        if self
            .circuit
            .device_models
            .iter()
            .any(|model| model.id == model_id)
            || symbol_id.as_ref().is_some_and(|symbol| {
                self.schematic
                    .symbol_definitions
                    .iter()
                    .any(|candidate| candidate.id == *symbol)
            })
            || pattern_id.as_ref().is_some_and(|pattern| {
                self.layout
                    .land_patterns
                    .iter()
                    .any(|candidate| candidate.id == *pattern)
            })
        {
            return Err(DesignBuildError::DuplicatePartDefinition(
                definition.name.clone(),
            ));
        }
        let pins = definition
            .model
            .pins
            .iter()
            .map(|pin| pin.pin.clone())
            .collect::<BTreeSet<_>>();

        self.circuit.device_models.push(definition.model.clone());
        self.source_map.push(
            AuthoringTarget::DeviceModel(model_id.clone()),
            AuthoringAction::Declare,
            source.clone(),
        );
        if let Some(symbol) = &definition.symbol {
            self.schematic.symbol_definitions.push(symbol.clone());
            self.source_map.push(
                AuthoringTarget::SymbolDefinition(symbol.id.clone()),
                AuthoringAction::Declare,
                source.clone(),
            );
        }
        if let Some(pattern) = &definition.land_pattern {
            self.layout.land_patterns.push(pattern.clone());
            for pad in &pattern.pads {
                self.source_map.push(
                    AuthoringTarget::Pad {
                        land_pattern: pattern.id.clone(),
                        pad: pad.id.clone(),
                    },
                    AuthoringAction::Declare,
                    source.clone(),
                );
            }
            self.source_map.push(
                AuthoringTarget::LandPattern(pattern.id.clone()),
                AuthoringAction::Declare,
                source,
            );
        }
        Ok(PartDefinitionHandle {
            owner: self.owner,
            id: definition.name.clone(),
            model: model_id,
            symbol: symbol_id,
            land_pattern: pattern_id,
            part: definition.part.clone(),
            pins,
        })
    }

    /// Atomically creates one instance of a reusable part definition.
    #[track_caller]
    pub fn instantiate(
        &mut self,
        definition: &PartDefinitionHandle,
        instance: PartInstance,
    ) -> Result<InstanceHandle, DesignBuildError> {
        if definition.owner != self.owner {
            return Err(DesignBuildError::ForeignHandle);
        }
        let model = self
            .circuit
            .device_models
            .iter()
            .find(|model| model.id == definition.model)
            .ok_or(DesignBuildError::ForeignHandle)?;
        let instance_id = CircuitInstanceId::new(instance.reference.clone())
            .map_err(|_| DesignBuildError::InvalidIdentifier)?;
        if self
            .circuit
            .instances
            .iter()
            .any(|candidate| candidate.id == instance_id)
        {
            return Err(DesignBuildError::DuplicateInstance(instance.reference));
        }
        if instance.waveform.is_some()
            && !matches!(
                &model.kind,
                DeviceModelKind::VoltageSource | DeviceModelKind::CurrentSource
            )
        {
            return Err(DesignBuildError::InvalidStimulusTarget(
                instance_id.as_str().into(),
            ));
        }
        let component = ComponentId::new(instance_id.as_str())
            .map_err(|_| DesignBuildError::InvalidIdentifier)?;

        let mut retained_symbols = Vec::new();
        if !instance.symbols.is_empty() {
            let Some(definition_id) = &definition.symbol else {
                return Err(DesignBuildError::MissingPartSymbolDefinition(
                    instance_id.as_str().into(),
                ));
            };
            let symbol_definition = self
                .schematic
                .symbol_definitions
                .iter()
                .find(|candidate| candidate.id == *definition_id)
                .ok_or(DesignBuildError::ForeignHandle)?;
            let mut placed_units = BTreeSet::new();
            for placement in &instance.symbols {
                if !placed_units.insert(placement.unit) {
                    return Err(DesignBuildError::DuplicateSymbolPlacement {
                        instance: instance_id.as_str().into(),
                        unit: placement.unit,
                    });
                }
                if !symbol_definition
                    .units
                    .iter()
                    .any(|unit| unit.unit == placement.unit)
                {
                    return Err(DesignBuildError::UnknownPartSymbolUnit {
                        definition: definition.id.clone(),
                        unit: placement.unit,
                    });
                }
                retained_symbols.push((
                    SchematicSymbol {
                        id: SchematicSymbolId::new(format!(
                            "{}:{}",
                            instance_id.as_str(),
                            placement.unit
                        ))
                        .map_err(|_| DesignBuildError::InvalidIdentifier)?,
                        instance: instance_id.clone(),
                        definition: definition_id.clone(),
                        unit: placement.unit,
                        position: placement.position.clone(),
                        quarter_turns: placement.quarter_turns,
                    },
                    placement.source.clone(),
                ));
            }
        }

        let retained_placement = match (&definition.land_pattern, instance.placement) {
            (Some(pattern), Some((position, rotation_degrees, side))) => Some(PcbPlacement {
                instance: instance_id.clone(),
                land_pattern: pattern.clone(),
                position,
                rotation_degrees,
                side,
            }),
            (Some(_), None) => {
                return Err(DesignBuildError::MissingPlacement(
                    instance_id.as_str().into(),
                ));
            }
            (None, Some(_)) => {
                return Err(DesignBuildError::MissingFootprint(
                    instance_id.as_str().into(),
                ));
            }
            (None, None) => None,
        };

        let stimulus = instance.waveform.map(|waveform| SourceStimulus {
            component: component.clone(),
            waveform,
        });
        self.circuit.instances.push(CircuitInstance {
            id: instance_id.clone(),
            component: component.clone(),
            part: definition.part.clone(),
            model: definition.model.clone(),
            pins: Vec::new(),
            parameters: instance.parameters,
        });
        self.source_map.push(
            AuthoringTarget::Instance(instance_id.clone()),
            AuthoringAction::Declare,
            instance.source.clone(),
        );
        for pin in &definition.pins {
            self.source_map.push(
                AuthoringTarget::Pin {
                    instance: instance_id.clone(),
                    pin: pin.clone(),
                },
                AuthoringAction::Declare,
                instance.source.clone(),
            );
        }
        for (symbol, source) in retained_symbols {
            self.source_map.push(
                AuthoringTarget::Symbol(symbol.id.clone()),
                AuthoringAction::Place,
                source,
            );
            self.schematic.symbols.push(symbol);
        }
        if let Some(placement) = retained_placement {
            self.source_map.push(
                AuthoringTarget::Placement(instance_id.clone()),
                AuthoringAction::Place,
                instance
                    .placement_source
                    .unwrap_or_else(|| instance.source.clone()),
            );
            self.layout.placements.push(placement);
        }
        if let Some(stimulus) = stimulus {
            self.source_map.push(
                AuthoringTarget::Stimulus(component),
                AuthoringAction::Declare,
                instance
                    .waveform_source
                    .unwrap_or_else(|| instance.source.clone()),
            );
            self.circuit.source_stimuli.push(stimulus);
        }
        Ok(InstanceHandle {
            owner: self.owner,
            id: instance_id,
            pins: definition.pins.clone(),
        })
    }

    /// Atomically lowers one fluent part into model, instance, footprint, and placement records.
    #[track_caller]
    pub fn add(&mut self, part: Part) -> Result<InstanceHandle, DesignBuildError> {
        let add_source = SourceLocation::caller();
        let part_source = part.source.clone();
        let placement_source = part.placement_source.clone();
        let waveform_source = part.waveform_source.clone();
        let instance_id = CircuitInstanceId::new(part.reference.clone())
            .map_err(|_| DesignBuildError::InvalidIdentifier)?;
        if self
            .circuit
            .instances
            .iter()
            .any(|instance| instance.id == instance_id)
        {
            return Err(DesignBuildError::DuplicateInstance(part.reference));
        }
        let model_id = DeviceModelId::new(format!("{}.model", instance_id.as_str()))
            .map_err(|_| DesignBuildError::InvalidIdentifier)?;
        let component = ComponentId::new(instance_id.as_str())
            .map_err(|_| DesignBuildError::InvalidIdentifier)?;
        if part.waveform.is_some()
            && !matches!(
                &part.model_kind,
                DeviceModelKind::VoltageSource | DeviceModelKind::CurrentSource
            )
        {
            return Err(DesignBuildError::InvalidStimulusTarget(
                instance_id.as_str().into(),
            ));
        }
        let mut pin_ids = BTreeSet::new();
        let mut device_pins = Vec::new();
        let mut mappings = Vec::new();
        let mut pending_traces = vec![
            AuthoringTrace {
                target: AuthoringTarget::DeviceModel(model_id.clone()),
                action: AuthoringAction::Declare,
                source: part_source.clone(),
            },
            AuthoringTrace {
                target: AuthoringTarget::Instance(instance_id.clone()),
                action: AuthoringAction::Declare,
                source: part_source.clone(),
            },
        ];
        if add_source != part_source {
            pending_traces.push(AuthoringTrace {
                target: AuthoringTarget::Instance(instance_id.clone()),
                action: AuthoringAction::Declare,
                source: add_source.clone(),
            });
        }
        for declared in &part.pins {
            let pin = PinRef::new(declared.name.clone())
                .map_err(|_| DesignBuildError::InvalidIdentifier)?;
            if !pin_ids.insert(pin.clone()) {
                return Err(DesignBuildError::DuplicatePin {
                    instance: instance_id.as_str().into(),
                    pin: pin.as_str().into(),
                });
            }
            device_pins.push(DevicePin {
                pin: pin.clone(),
                kind: declared.kind,
                optional: declared.optional,
            });
            pending_traces.push(AuthoringTrace {
                target: AuthoringTarget::Pin {
                    instance: instance_id.clone(),
                    pin: pin.clone(),
                },
                action: AuthoringAction::Declare,
                source: declared.source.clone(),
            });
            for pad in &declared.pads {
                mappings.push(PadPinMap {
                    pin: pin.clone(),
                    pad: PadId::new(pad.clone())
                        .map_err(|_| DesignBuildError::InvalidIdentifier)?,
                });
            }
        }
        let schematic_symbol = match part.symbol {
            Some(symbol) => {
                let symbol_id = SchematicSymbolId::new(format!("{}:A", instance_id.as_str()))
                    .map_err(|_| DesignBuildError::InvalidIdentifier)?;
                let definition_id =
                    SchematicSymbolDefinitionId::new(format!("{}:symbol", model_id.as_str()))
                        .map_err(|_| DesignBuildError::InvalidIdentifier)?;
                let mut symbol_pins = BTreeSet::new();
                let mut placements = Vec::new();
                for declared in symbol.pins {
                    let pin = PinRef::new(declared.name)
                        .map_err(|_| DesignBuildError::InvalidIdentifier)?;
                    if !symbol_pins.insert(pin.clone()) {
                        return Err(DesignBuildError::DuplicateSymbolPin {
                            instance: instance_id.as_str().into(),
                            pin: pin.as_str().into(),
                        });
                    }
                    if !pin_ids.contains(&pin) {
                        return Err(DesignBuildError::UnknownSymbolPin {
                            instance: instance_id.as_str().into(),
                            pin: pin.as_str().into(),
                        });
                    }
                    pending_traces.push(AuthoringTrace {
                        target: AuthoringTarget::Pin {
                            instance: instance_id.clone(),
                            pin: pin.clone(),
                        },
                        action: AuthoringAction::Declare,
                        source: declared.source,
                    });
                    placements.push(SchematicPinPlacement {
                        pin,
                        position: declared.position,
                        side: declared.side,
                    });
                }
                for pin in device_pins.iter().filter(|pin| !pin.optional) {
                    if !symbol_pins.contains(&pin.pin) {
                        return Err(DesignBuildError::MissingSymbolPin {
                            instance: instance_id.as_str().into(),
                            pin: pin.pin.as_str().into(),
                        });
                    }
                }
                pending_traces.push(AuthoringTrace {
                    target: AuthoringTarget::Symbol(symbol_id.clone()),
                    action: AuthoringAction::Place,
                    source: symbol.source,
                });
                let half_width = (symbol.body_width.clone() / Real::from(2)).map_err(|_| {
                    DesignBuildError::InvalidSymbolGeometry(instance_id.as_str().into())
                })?;
                let half_height = (symbol.body_height.clone() / Real::from(2)).map_err(|_| {
                    DesignBuildError::InvalidSymbolGeometry(instance_id.as_str().into())
                })?;
                Some((
                    SchematicSymbolDefinition {
                        id: definition_id.clone(),
                        model: model_id.clone(),
                        name: format!("{} symbol", instance_id.as_str()),
                        units: vec![SchematicSymbolUnit {
                            unit: 1,
                            body_width: symbol.body_width,
                            body_height: symbol.body_height,
                            pins: placements,
                            graphics: vec![SchematicGraphic::Rectangle {
                                start: SchematicPoint::new(
                                    -half_width.clone(),
                                    -half_height.clone(),
                                ),
                                end: SchematicPoint::new(half_width, half_height),
                                stroke_width: Real::one(),
                                fill: SchematicGraphicFill::Background,
                            }],
                        }],
                    },
                    SchematicSymbol {
                        id: symbol_id,
                        instance: instance_id.clone(),
                        definition: definition_id,
                        unit: 1,
                        position: symbol.position,
                        quarter_turns: symbol.quarter_turns,
                    },
                ))
            }
            None => None,
        };
        let physical = match (part.footprint, part.placement) {
            (Some(footprint), Some((position, rotation_degrees, side))) => {
                let pattern_id = LandPatternId::new(format!("{}.footprint", instance_id.as_str()))
                    .map_err(|_| DesignBuildError::InvalidIdentifier)?;
                let footprint_source = footprint.source.clone();
                let mut pad_ids = BTreeSet::new();
                for pad in &footprint.pads {
                    if !pad_ids.insert(pad.id.clone()) {
                        return Err(DesignBuildError::DuplicatePad {
                            instance: instance_id.as_str().into(),
                            pad: pad.id.as_str().into(),
                        });
                    }
                    pending_traces.push(AuthoringTrace {
                        target: AuthoringTarget::Pad {
                            land_pattern: pattern_id.clone(),
                            pad: pad.id.clone(),
                        },
                        action: AuthoringAction::Declare,
                        source: footprint_source.clone(),
                    });
                }
                for pin in &device_pins {
                    if !mappings.iter().any(|mapping| mapping.pin == pin.pin)
                        && let Some(pad) =
                            pad_ids.iter().find(|pad| pad.as_str() == pin.pin.as_str())
                    {
                        mappings.push(PadPinMap {
                            pin: pin.pin.clone(),
                            pad: pad.clone(),
                        });
                    }
                }
                for mapping in &mappings {
                    if !pad_ids.contains(&mapping.pad) {
                        return Err(DesignBuildError::UnknownPad {
                            instance: instance_id.as_str().into(),
                            pin: mapping.pin.as_str().into(),
                            pad: mapping.pad.as_str().into(),
                        });
                    }
                }
                pending_traces.push(AuthoringTrace {
                    target: AuthoringTarget::LandPattern(pattern_id.clone()),
                    action: AuthoringAction::Declare,
                    source: footprint_source,
                });
                pending_traces.push(AuthoringTrace {
                    target: AuthoringTarget::Placement(instance_id.clone()),
                    action: AuthoringAction::Place,
                    source: placement_source.unwrap_or_else(|| add_source.clone()),
                });
                Some((
                    LandPattern {
                        id: pattern_id.clone(),
                        pads: footprint.pads,
                        pin_map: mappings,
                        graphics: footprint.graphics,
                        body: footprint.body,
                        models: footprint.models,
                    },
                    PcbPlacement {
                        instance: instance_id.clone(),
                        land_pattern: pattern_id,
                        position,
                        rotation_degrees,
                        side,
                    },
                ))
            }
            (Some(_), None) => {
                return Err(DesignBuildError::MissingPlacement(
                    instance_id.as_str().into(),
                ));
            }
            (None, Some(_)) => {
                return Err(DesignBuildError::MissingFootprint(
                    instance_id.as_str().into(),
                ));
            }
            (None, None) if !mappings.is_empty() => {
                return Err(DesignBuildError::MissingFootprint(
                    instance_id.as_str().into(),
                ));
            }
            (None, None) => None,
        };
        let part_ref = part
            .part
            .map(PartRef::new)
            .transpose()
            .map_err(|_| DesignBuildError::InvalidIdentifier)?;
        let stimulus = part.waveform.map(|waveform| SourceStimulus {
            component: component.clone(),
            waveform,
        });
        if stimulus.is_some() {
            pending_traces.push(AuthoringTrace {
                target: AuthoringTarget::Stimulus(component.clone()),
                action: AuthoringAction::Declare,
                source: waveform_source.unwrap_or_else(|| part_source.clone()),
            });
        }
        self.circuit.device_models.push(DeviceModel {
            id: model_id.clone(),
            kind: part.model_kind,
            pins: device_pins,
            parameters: Vec::new(),
        });
        self.circuit.instances.push(CircuitInstance {
            id: instance_id.clone(),
            component,
            part: part_ref,
            model: model_id,
            pins: Vec::new(),
            parameters: part.parameters,
        });
        if let Some(stimulus) = stimulus {
            self.circuit.source_stimuli.push(stimulus);
        }
        if let Some((definition, symbol)) = schematic_symbol {
            self.schematic.symbol_definitions.push(definition);
            self.schematic.symbols.push(symbol);
        }
        if let Some((pattern, placement)) = physical {
            self.layout.land_patterns.push(pattern);
            self.layout.placements.push(placement);
        }
        self.source_map.traces.extend(pending_traces);
        Ok(InstanceHandle {
            owner: self.owner,
            id: instance_id,
            pins: pin_ids,
        })
    }

    /// Declares one named manufacturable via construction.
    #[track_caller]
    pub fn via_style(&mut self, style: ViaStyleRule) -> Result<ViaStyleHandle, DesignBuildError> {
        let id = ViaStyleId::new(style.id).map_err(|_| DesignBuildError::InvalidIdentifier)?;
        if self
            .layout
            .rules
            .via_styles
            .iter()
            .any(|candidate| candidate.id == id)
        {
            return Err(DesignBuildError::DuplicateViaStyle(id.as_str().into()));
        }
        let retained = ViaStyle {
            id: id.clone(),
            land_diameter: style.land_diameter,
            drill_diameter: style.drill_diameter,
            plating: style.plating,
            mask: style.mask,
            allowed_spans: style.allowed_spans,
        };
        let mut probe = self.empty_layout_probe();
        probe.rules.via_styles.push(retained.clone());
        if probe.validate(&self.circuit).issues.iter().any(|issue| {
            matches!(
                issue,
                LayoutValidationIssue::DuplicateViaStyle(candidate)
                    | LayoutValidationIssue::InvalidViaStyle(candidate) if candidate == &id
            )
        }) {
            return Err(DesignBuildError::InvalidViaStyle(id.as_str().into()));
        }
        self.layout.rules.via_styles.push(retained);
        self.source_map.push(
            AuthoringTarget::ViaStyle(id.clone()),
            AuthoringAction::Declare,
            style.source,
        );
        Ok(ViaStyleHandle {
            owner: self.owner,
            id,
        })
    }

    /// Declares one named typed-net routing and verification policy.
    #[track_caller]
    pub fn net_class(&mut self, class: NetClassRule) -> Result<NetClassHandle, DesignBuildError> {
        let id = NetClassId::new(class.id).map_err(|_| DesignBuildError::InvalidIdentifier)?;
        if self
            .layout
            .rules
            .net_classes
            .iter()
            .any(|candidate| candidate.id == id)
        {
            return Err(DesignBuildError::DuplicateNetClass(id.as_str().into()));
        }
        let parent = class
            .parent
            .map(|parent| {
                if parent.owner != self.owner
                    || !self
                        .layout
                        .rules
                        .net_classes
                        .iter()
                        .any(|candidate| candidate.id == parent.id)
                {
                    return Err(DesignBuildError::ForeignHandle);
                }
                Ok(parent.id)
            })
            .transpose()?;
        let preferred_via_style = class
            .preferred_via_style
            .map(|style| {
                if style.owner != self.owner
                    || !self
                        .layout
                        .rules
                        .via_styles
                        .iter()
                        .any(|candidate| candidate.id == style.id)
                {
                    return Err(DesignBuildError::ForeignHandle);
                }
                Ok(style.id)
            })
            .transpose()?;
        let nets = class
            .nets
            .iter()
            .map(|net| self.checked_net_id(net))
            .collect::<Result<Vec<_>, _>>()?;
        let retained = NetClass {
            id: id.clone(),
            parent,
            nets: nets.clone(),
            min_trace_width: class.min_trace_width,
            preferred_trace_width: class.preferred_trace_width,
            min_clearance: class.min_clearance,
            preferred_via_land_diameter: class.preferred_via_land_diameter,
            preferred_via_drill_diameter: class.preferred_via_drill_diameter,
            preferred_via_style,
            max_length: class.max_length,
            max_via_count: class.max_via_count,
            target_impedance_ohms: class.target_impedance_ohms,
            impedance_tolerance_ohms: class.impedance_tolerance_ohms,
            requires_reference_plane: class.requires_reference_plane,
        };
        let mut probe = self.empty_layout_probe();
        probe.rules = self.layout.rules.clone();
        probe.rules.net_classes.push(retained.clone());
        if probe.validate(&self.circuit).issues.iter().any(|issue| {
            matches!(
                issue,
                LayoutValidationIssue::DuplicateNetClass(candidate)
                    | LayoutValidationIssue::NetClassInheritanceCycle(candidate)
                    | LayoutValidationIssue::InvalidNetClassConstraint(candidate)
                    | LayoutValidationIssue::UnknownNetClassParent {
                        class: candidate,
                        ..
                    }
                    | LayoutValidationIssue::UnknownNetClassNet {
                        class: candidate,
                        ..
                    }
                    | LayoutValidationIssue::UnknownNetClassViaStyle {
                        class: candidate,
                        ..
                    } if candidate == &id
            ) || matches!(
                issue,
                LayoutValidationIssue::NetInMultipleClasses(net) if nets.contains(net)
            )
        }) {
            return Err(DesignBuildError::InvalidNetClass(id.as_str().into()));
        }
        self.layout.rules.net_classes.push(retained);
        self.source_map.push(
            AuthoringTarget::NetClass(id.clone()),
            AuthoringAction::Declare,
            class.source,
        );
        Ok(NetClassHandle {
            owner: self.owner,
            id,
        })
    }

    /// Declares complementary typed nets as one differential pair.
    #[track_caller]
    pub fn differential_pair(
        &mut self,
        pair: DifferentialPairRule,
    ) -> Result<DifferentialPairHandle, DesignBuildError> {
        let id =
            DifferentialPairId::new(pair.id).map_err(|_| DesignBuildError::InvalidIdentifier)?;
        if self
            .layout
            .rules
            .differential_pairs
            .iter()
            .any(|candidate| candidate.id == id)
        {
            return Err(DesignBuildError::DuplicateDifferentialPair(
                id.as_str().into(),
            ));
        }
        let retained = DifferentialPair {
            id: id.clone(),
            positive: self.checked_net_id(&pair.positive)?,
            negative: self.checked_net_id(&pair.negative)?,
            spacing: pair.spacing,
            max_skew: pair.max_skew,
            target_impedance_ohms: pair.target_impedance_ohms,
            impedance_tolerance_ohms: pair.impedance_tolerance_ohms,
            neckdown: pair.neckdown,
        };
        let mut probe = self.empty_layout_probe();
        probe.rules.differential_pairs.push(retained.clone());
        if probe.validate(&self.circuit).issues.iter().any(|issue| {
            matches!(
                issue,
                LayoutValidationIssue::DuplicateDifferentialPair(candidate)
                    | LayoutValidationIssue::InvalidDifferentialPair(candidate)
                    if candidate == &id
            )
        }) {
            return Err(DesignBuildError::InvalidDifferentialPair(
                id.as_str().into(),
            ));
        }
        self.layout.rules.differential_pairs.push(retained);
        self.source_map.push(
            AuthoringTarget::DifferentialPair(id.clone()),
            AuthoringAction::Declare,
            pair.source,
        );
        Ok(DifferentialPairHandle {
            owner: self.owner,
            id,
        })
    }

    /// Declares one bounded route-length tuning request.
    #[track_caller]
    pub fn length_tuning(
        &mut self,
        rule: LengthTuningRule,
    ) -> Result<LengthTuningPatternHandle, DesignBuildError> {
        if rule.net.owner != self.owner
            || rule
                .route
                .as_ref()
                .is_some_and(|route| route.owner != self.owner)
        {
            return Err(DesignBuildError::ForeignHandle);
        }
        let id =
            LengthTuningPatternId::new(rule.id).map_err(|_| DesignBuildError::InvalidIdentifier)?;
        if self
            .layout
            .rules
            .length_tuning_patterns
            .iter()
            .any(|candidate| candidate.id == id)
        {
            return Err(DesignBuildError::DuplicateLengthTuningPattern(
                id.as_str().into(),
            ));
        }
        let retained = LengthTuningPattern {
            id: id.clone(),
            net: self.checked_net_id(&rule.net)?,
            route: rule.route.map(|route| route.id),
            region: rule.region,
            target_length: rule.target_length,
            tolerance: rule.tolerance,
            amplitude: rule.amplitude,
            pitch: rule.pitch,
            maximum_cycles: rule.maximum_cycles,
            side: rule.side,
        };
        let mut probe = self.layout.clone();
        probe.rules.length_tuning_patterns.push(retained.clone());
        if probe.validate(&self.circuit).issues.iter().any(|issue| {
            matches!(
                issue,
                LayoutValidationIssue::DuplicateLengthTuningPattern(candidate)
                    | LayoutValidationIssue::InvalidLengthTuningTarget(candidate)
                    | LayoutValidationIssue::InvalidLengthTuningPattern(candidate)
                    if candidate == &id
            )
        }) {
            return Err(DesignBuildError::InvalidLengthTuningPattern(
                id.as_str().into(),
            ));
        }
        self.layout.rules.length_tuning_patterns.push(retained);
        self.source_map.push(
            AuthoringTarget::LengthTuningPattern(id.clone()),
            AuthoringAction::Declare,
            rule.source,
        );
        Ok(LengthTuningPatternHandle {
            owner: self.owner,
            id,
        })
    }

    /// Declares one atomic multi-pattern phase-tuning group.
    #[track_caller]
    pub fn phase_tuning_group(
        &mut self,
        rule: PhaseTuningGroupRule,
    ) -> Result<PhaseTuningGroupHandle, DesignBuildError> {
        if rule
            .patterns
            .iter()
            .any(|pattern| pattern.owner != self.owner)
            || rule
                .differential_pair
                .as_ref()
                .is_some_and(|pair| pair.owner != self.owner)
        {
            return Err(DesignBuildError::ForeignHandle);
        }
        let id =
            PhaseTuningGroupId::new(rule.id).map_err(|_| DesignBuildError::InvalidIdentifier)?;
        if self
            .layout
            .rules
            .phase_tuning_groups
            .iter()
            .any(|candidate| candidate.id == id)
        {
            return Err(DesignBuildError::DuplicatePhaseTuningGroup(
                id.as_str().into(),
            ));
        }
        let retained = PhaseTuningGroup {
            id: id.clone(),
            patterns: rule
                .patterns
                .into_iter()
                .map(|pattern| pattern.id)
                .collect(),
            differential_pair: rule.differential_pair.map(|pair| pair.id),
            maximum_skew: rule.maximum_skew,
            minimum_clearance: rule.minimum_clearance,
        };
        let mut probe = self.layout.clone();
        probe.rules.phase_tuning_groups.push(retained.clone());
        if probe.validate(&self.circuit).issues.iter().any(|issue| {
            matches!(
                issue,
                LayoutValidationIssue::DuplicatePhaseTuningGroup(candidate)
                    | LayoutValidationIssue::InvalidPhaseTuningTarget(candidate)
                    | LayoutValidationIssue::InvalidPhaseTuningGroup(candidate)
                    if candidate == &id
            )
        }) {
            return Err(DesignBuildError::InvalidPhaseTuningGroup(
                id.as_str().into(),
            ));
        }
        self.layout.rules.phase_tuning_groups.push(retained);
        self.source_map.push(
            AuthoringTarget::PhaseTuningGroup(id.clone()),
            AuthoringAction::Declare,
            rule.source,
        );
        Ok(PhaseTuningGroupHandle {
            owner: self.owner,
            id,
        })
    }

    /// Declares one typed placement equation or predicate.
    #[track_caller]
    pub fn constrain(
        &mut self,
        rule: PlacementRule,
    ) -> Result<PlacementConstraintHandle, DesignBuildError> {
        let id =
            PlacementConstraintId::new(rule.id).map_err(|_| DesignBuildError::InvalidIdentifier)?;
        if self
            .layout
            .placement_constraints
            .iter()
            .any(|candidate| candidate.id == id)
        {
            return Err(DesignBuildError::DuplicatePlacementConstraint(
                id.as_str().into(),
            ));
        }
        let (kind, driven_instance) = match rule.kind {
            PlacementRuleKind::Fixed { instance, position } => {
                let instance = self.checked_placement_instance(&instance)?;
                (
                    PlacementConstraintKind::Fixed {
                        instance: instance.clone(),
                        position,
                    },
                    Some(instance),
                )
            }
            PlacementRuleKind::Relative {
                instance,
                anchor,
                offset,
            } => {
                let instance = self.checked_placement_instance(&instance)?;
                let anchor = self.checked_placement_instance(&anchor)?;
                (
                    PlacementConstraintKind::Relative {
                        instance: instance.clone(),
                        anchor,
                        offset,
                    },
                    Some(instance),
                )
            }
            PlacementRuleKind::AlignX { instances } => (
                PlacementConstraintKind::AlignX {
                    instances: instances
                        .iter()
                        .map(|instance| self.checked_placement_instance(instance))
                        .collect::<Result<_, _>>()?,
                },
                None,
            ),
            PlacementRuleKind::AlignY { instances } => (
                PlacementConstraintKind::AlignY {
                    instances: instances
                        .iter()
                        .map(|instance| self.checked_placement_instance(instance))
                        .collect::<Result<_, _>>()?,
                },
                None,
            ),
            PlacementRuleKind::Within { instance, min, max } => (
                PlacementConstraintKind::Within {
                    instance: self.checked_placement_instance(&instance)?,
                    min,
                    max,
                },
                None,
            ),
            PlacementRuleKind::AllowedRotations {
                instance,
                rotations_degrees,
            } => (
                PlacementConstraintKind::AllowedRotations {
                    instance: self.checked_placement_instance(&instance)?,
                    rotations_degrees,
                },
                None,
            ),
            PlacementRuleKind::AllowedSides { instance, sides } => (
                PlacementConstraintKind::AllowedSides {
                    instance: self.checked_placement_instance(&instance)?,
                    sides,
                },
                None,
            ),
        };
        let retained = PlacementConstraint {
            id: id.clone(),
            kind,
        };
        let mut probe = self.layout.clone();
        probe.placement_constraints.push(retained.clone());
        if probe.validate(&self.circuit).issues.iter().any(|issue| {
            matches!(
                issue,
                LayoutValidationIssue::DuplicatePlacementConstraint(candidate)
                    | LayoutValidationIssue::InvalidPlacementConstraint(candidate)
                    | LayoutValidationIssue::UnknownPlacementConstraintInstance {
                        constraint: candidate,
                        ..
                    } if candidate == &id
            ) || matches!(
                (issue, &driven_instance),
                (
                    LayoutValidationIssue::MultiplePlacementDrivers(candidate),
                    Some(driven)
                ) if candidate == driven
            )
        }) {
            return Err(DesignBuildError::InvalidPlacementConstraint(
                id.as_str().into(),
            ));
        }
        self.layout.placement_constraints.push(retained);
        self.source_map.push(
            AuthoringTarget::PlacementConstraint(id.clone()),
            AuthoringAction::Declare,
            rule.source,
        );
        Ok(PlacementConstraintHandle {
            owner: self.owner,
            id,
        })
    }

    /// Declares exact routed copper for a typed logical net.
    #[track_caller]
    pub fn route(
        &mut self,
        net: &NetHandle,
        route: Route,
    ) -> Result<RouteHandle, DesignBuildError> {
        if net.owner != self.owner {
            return Err(DesignBuildError::ForeignHandle);
        }
        if !self
            .circuit
            .nets
            .iter()
            .any(|candidate| candidate.id == net.id)
        {
            return Err(DesignBuildError::Circuit(
                CircuitError::MissingConnectionNet,
            ));
        }
        let id = RouteId::new(route.id).map_err(|_| DesignBuildError::InvalidIdentifier)?;
        if self
            .layout
            .routes
            .iter()
            .any(|candidate| candidate.id == id)
        {
            return Err(DesignBuildError::DuplicateRoute(id.as_str().into()));
        }
        let retained = PcbRoute {
            id: id.clone(),
            net: net.id.clone(),
            layer: route.layer,
            width: route.width,
            segments: route.segments,
        };
        let mut probe = self.empty_layout_probe();
        probe.routes.push(retained.clone());
        if probe.validate(&self.circuit).issues.iter().any(|issue| {
            matches!(
                issue,
                LayoutValidationIssue::UnknownRouteNet { .. }
                    | LayoutValidationIssue::UnknownRouteLayer { .. }
                    | LayoutValidationIssue::EmptyRoute(_)
                    | LayoutValidationIssue::NonPositiveRouteWidth(_)
                    | LayoutValidationIssue::DisconnectedRoute(_)
                    | LayoutValidationIssue::InvalidRouteArcWidth(_)
                    | LayoutValidationIssue::DuplicateRoute(_)
            )
        }) {
            return Err(DesignBuildError::InvalidRoute(id.as_str().into()));
        }
        self.layout.routes.push(retained);
        self.source_map.push(
            AuthoringTarget::Route(id.clone()),
            AuthoringAction::Declare,
            route.source,
        );
        Ok(RouteHandle {
            owner: self.owner,
            id,
        })
    }

    /// Declares one exact layer transition for a typed logical net.
    #[track_caller]
    pub fn via(&mut self, net: &NetHandle, via: Via) -> Result<ViaHandle, DesignBuildError> {
        if net.owner != self.owner {
            return Err(DesignBuildError::ForeignHandle);
        }
        if !self
            .circuit
            .nets
            .iter()
            .any(|candidate| candidate.id == net.id)
        {
            return Err(DesignBuildError::Circuit(
                CircuitError::MissingConnectionNet,
            ));
        }
        let id = ViaId::new(via.id).map_err(|_| DesignBuildError::InvalidIdentifier)?;
        if self.layout.vias.iter().any(|candidate| candidate.id == id) {
            return Err(DesignBuildError::DuplicateVia(id.as_str().into()));
        }
        let retained = PcbVia {
            id: id.clone(),
            net: net.id.clone(),
            start_layer: via.start_layer,
            end_layer: via.end_layer,
            center: via.center,
            land_diameter: via.land_diameter,
            drill_diameter: via.drill_diameter,
            plating: via.plating,
            mask: via.mask,
        };
        let mut probe = self.empty_layout_probe();
        probe.vias.push(retained.clone());
        if probe.validate(&self.circuit).issues.iter().any(|issue| {
            matches!(
                issue,
                LayoutValidationIssue::UnknownViaNet { .. }
                    | LayoutValidationIssue::InvalidVia(_)
                    | LayoutValidationIssue::DuplicateVia(_)
            )
        }) {
            return Err(DesignBuildError::InvalidVia(id.as_str().into()));
        }
        self.layout.vias.push(retained);
        self.source_map.push(
            AuthoringTarget::Via(id.clone()),
            AuthoringAction::Declare,
            via.source,
        );
        Ok(ViaHandle {
            owner: self.owner,
            id,
        })
    }

    /// Declares one retained copper pour for a typed logical net.
    #[track_caller]
    pub fn zone(&mut self, net: &NetHandle, zone: Zone) -> Result<ZoneHandle, DesignBuildError> {
        if net.owner != self.owner {
            return Err(DesignBuildError::ForeignHandle);
        }
        if !self
            .circuit
            .nets
            .iter()
            .any(|candidate| candidate.id == net.id)
        {
            return Err(DesignBuildError::Circuit(
                CircuitError::MissingConnectionNet,
            ));
        }
        let id = ZoneId::new(zone.id).map_err(|_| DesignBuildError::InvalidIdentifier)?;
        if self.layout.zones.iter().any(|candidate| candidate.id == id) {
            return Err(DesignBuildError::DuplicateZone(id.as_str().into()));
        }
        let retained = CopperZone {
            id: id.clone(),
            net: net.id.clone(),
            layer: zone.layer,
            boundary: zone.boundary,
            clearance: zone.clearance,
            fill: zone.fill,
            connection: zone.connection,
            islands: zone.islands,
            stitching: zone.stitching,
            priority: zone.priority,
        };
        let mut probe = self.empty_layout_probe();
        probe.zones.push(retained.clone());
        if probe.validate(&self.circuit).issues.iter().any(|issue| {
            matches!(
                issue,
                LayoutValidationIssue::UnknownZoneNet { .. }
                    | LayoutValidationIssue::UnknownZoneLayer { .. }
                    | LayoutValidationIssue::InvalidZoneBoundary(_)
                    | LayoutValidationIssue::InvalidZonePolicy(_)
                    | LayoutValidationIssue::DuplicateZone(_)
            )
        }) {
            return Err(DesignBuildError::InvalidZone(id.as_str().into()));
        }
        self.layout.zones.push(retained);
        self.source_map.push(
            AuthoringTarget::Zone(id.clone()),
            AuthoringAction::Declare,
            zone.source,
        );
        Ok(ZoneHandle {
            owner: self.owner,
            id,
        })
    }

    /// Declares one retained physical-layout keepout.
    #[track_caller]
    pub fn keepout(&mut self, keepout: Keepout) -> Result<KeepoutHandle, DesignBuildError> {
        let id = KeepoutId::new(keepout.id).map_err(|_| DesignBuildError::InvalidIdentifier)?;
        if self
            .layout
            .keepouts
            .iter()
            .any(|candidate| candidate.id == id)
        {
            return Err(DesignBuildError::DuplicateKeepout(id.as_str().into()));
        }
        let retained = PcbKeepout {
            id: id.clone(),
            boundary: keepout.boundary,
            scope: keepout.scope,
        };
        let mut probe = self.empty_layout_probe();
        probe.keepouts.push(retained.clone());
        if probe.validate(&self.circuit).issues.iter().any(|issue| {
            matches!(
                issue,
                LayoutValidationIssue::InvalidKeepoutBoundary(_)
                    | LayoutValidationIssue::DuplicateKeepout(_)
            )
        }) {
            return Err(DesignBuildError::InvalidKeepout(id.as_str().into()));
        }
        self.layout.keepouts.push(retained);
        self.source_map.push(
            AuthoringTarget::Keepout(id.clone()),
            AuthoringAction::Declare,
            keepout.source,
        );
        Ok(KeepoutHandle {
            owner: self.owner,
            id,
        })
    }

    fn empty_layout_probe(&self) -> PcbLayout {
        PcbLayout::new(
            self.layout.id.clone(),
            self.layout.outline.clone(),
            self.layout.stackup.clone(),
        )
    }

    fn checked_placement_instance(
        &self,
        instance: &InstanceHandle,
    ) -> Result<CircuitInstanceId, DesignBuildError> {
        if instance.owner != self.owner {
            return Err(DesignBuildError::ForeignHandle);
        }
        if !self
            .layout
            .placements
            .iter()
            .any(|placement| placement.instance == instance.id)
        {
            return Err(DesignBuildError::InvalidPlacementConstraint(
                instance.id.as_str().into(),
            ));
        }
        Ok(instance.id.clone())
    }

    fn checked_net_id(&self, net: &NetHandle) -> Result<NetId, DesignBuildError> {
        if net.owner != self.owner {
            return Err(DesignBuildError::ForeignHandle);
        }
        if !self
            .circuit
            .nets
            .iter()
            .any(|candidate| candidate.id == net.id)
        {
            return Err(DesignBuildError::Circuit(
                CircuitError::MissingConnectionNet,
            ));
        }
        Ok(net.id.clone())
    }

    /// Attaches or overrides scalar source parameters with retained time dependence.
    #[track_caller]
    pub fn stimulus(
        &mut self,
        source: &InstanceHandle,
        waveform: SourceWaveform,
    ) -> Result<(), DesignBuildError> {
        if source.owner != self.owner {
            return Err(DesignBuildError::ForeignHandle);
        }
        let Some(instance) = self
            .circuit
            .instances
            .iter()
            .find(|instance| instance.id == source.id)
        else {
            return Err(DesignBuildError::Circuit(CircuitError::MissingInstance));
        };
        if self
            .circuit
            .source_stimuli
            .iter()
            .any(|stimulus| stimulus.component == instance.component)
        {
            return Err(DesignBuildError::DuplicateStimulus(
                source.id.as_str().into(),
            ));
        }
        let model = self
            .circuit
            .device_models
            .iter()
            .find(|model| model.id == instance.model)
            .expect("authoring-created instance model is retained");
        if !matches!(
            &model.kind,
            DeviceModelKind::VoltageSource | DeviceModelKind::CurrentSource
        ) {
            return Err(DesignBuildError::InvalidStimulusTarget(
                source.id.as_str().into(),
            ));
        }
        let component = instance.component.clone();
        self.circuit.source_stimuli.push(SourceStimulus {
            component: component.clone(),
            waveform,
        });
        self.source_map.push(
            AuthoringTarget::Stimulus(component),
            AuthoringAction::Declare,
            SourceLocation::caller(),
        );
        Ok(())
    }

    /// Connects one or more typed pins to a retained net as one atomic request.
    #[track_caller]
    pub fn connect(
        &mut self,
        net: &NetHandle,
        pins: impl IntoIterator<Item = PinHandle>,
    ) -> Result<(), DesignBuildError> {
        let source = SourceLocation::caller();
        if net.owner != self.owner {
            return Err(DesignBuildError::ForeignHandle);
        }
        if !self
            .circuit
            .nets
            .iter()
            .any(|candidate| candidate.id == net.id)
        {
            return Err(DesignBuildError::Circuit(
                CircuitError::MissingConnectionNet,
            ));
        }
        let pins = pins.into_iter().collect::<Vec<_>>();
        let mut requested = BTreeSet::new();
        for pin in &pins {
            if pin.owner != self.owner {
                return Err(DesignBuildError::ForeignHandle);
            }
            if !requested.insert((pin.instance.clone(), pin.pin.clone())) {
                return Err(DesignBuildError::DuplicateConnection {
                    instance: pin.instance.as_str().into(),
                    pin: pin.pin.as_str().into(),
                });
            }
            let Some(instance) = self
                .circuit
                .instances
                .iter()
                .find(|instance| instance.id == pin.instance)
            else {
                return Err(DesignBuildError::Circuit(CircuitError::MissingInstance));
            };
            if instance.pins.iter().any(|binding| binding.pin == pin.pin) {
                return Err(DesignBuildError::DuplicateConnection {
                    instance: pin.instance.as_str().into(),
                    pin: pin.pin.as_str().into(),
                });
            }
            let model = self
                .circuit
                .device_models
                .iter()
                .find(|model| model.id == instance.model)
                .expect("authoring-created instance model is retained");
            if !model.pins.iter().any(|declared| declared.pin == pin.pin) {
                return Err(DesignBuildError::UnknownPin {
                    instance: pin.instance.as_str().into(),
                    pin: pin.pin.as_str().into(),
                });
            }
        }
        let existing_endpoint = self
            .circuit
            .instances
            .iter()
            .flat_map(|instance| {
                instance
                    .pins
                    .iter()
                    .filter(|binding| binding.net == net.id)
                    .map(move |binding| (&instance.id, &binding.pin))
            })
            .find_map(|(instance, pin)| self.schematic_endpoint_for_pin(instance, pin));
        let visible_endpoints = pins
            .iter()
            .filter_map(|pin| self.schematic_endpoint_for_pin(&pin.instance, &pin.pin))
            .collect::<Vec<_>>();
        let mut wire_endpoints = Vec::new();
        if let Some(endpoint) = existing_endpoint {
            wire_endpoints.push(endpoint);
        }
        wire_endpoints.extend(visible_endpoints);
        let mut staged_wires = Vec::new();
        let mut next_wire = self.schematic.wires.len();
        for endpoints in wire_endpoints.windows(2) {
            let id = loop {
                let candidate =
                    SchematicWireId::new(format!("{}.wire.{next_wire}", net.id.as_str()))
                        .map_err(|_| DesignBuildError::InvalidIdentifier)?;
                next_wire += 1;
                if !self.schematic.wires.iter().any(|wire| wire.id == candidate)
                    && !staged_wires
                        .iter()
                        .any(|wire: &SchematicWire| wire.id == candidate)
                {
                    break candidate;
                }
            };
            staged_wires.push(SchematicWire {
                id,
                net: net.id.clone(),
                from: endpoints[0].clone(),
                waypoints: Vec::new(),
                to: endpoints[1].clone(),
            });
        }
        for pin in pins {
            self.circuit
                .connect_pin(&pin.instance, pin.pin.clone(), &net.id)?;
            self.source_map.push(
                AuthoringTarget::Connection {
                    instance: pin.instance,
                    pin: pin.pin,
                },
                AuthoringAction::Connect,
                source.clone(),
            );
        }
        for wire in staged_wires {
            self.source_map.push(
                AuthoringTarget::Wire(wire.id.clone()),
                AuthoringAction::Connect,
                source.clone(),
            );
            self.schematic.wires.push(wire);
        }
        Ok(())
    }

    fn schematic_endpoint_for_pin(
        &self,
        instance: &CircuitInstanceId,
        pin: &PinRef,
    ) -> Option<SchematicEndpoint> {
        self.schematic
            .symbols
            .iter()
            .find(|symbol| {
                &symbol.instance == instance
                    && self
                        .schematic
                        .symbol_unit(symbol)
                        .is_some_and(|unit| unit.pins.iter().any(|placement| &placement.pin == pin))
            })
            .map(|symbol| SchematicEndpoint::Pin {
                symbol: symbol.id.clone(),
                pin: pin.clone(),
            })
    }

    fn collect_diagnostic_sources(
        &self,
        targets: impl IntoIterator<Item = AuthoringTarget>,
        fallback: AuthoringTarget,
    ) -> Vec<SourceLocation> {
        let mut sources = Vec::new();
        let mut seen = BTreeSet::new();
        for target in targets {
            for trace in self.source_map.traces_for(&target) {
                if seen.insert(trace.source.clone()) {
                    sources.push(trace.source.clone());
                }
            }
        }
        for trace in self
            .source_map
            .traces_for(&fallback)
            .into_iter()
            .filter(|trace| trace.action == AuthoringAction::Mutate)
        {
            if seen.insert(trace.source.clone()) {
                sources.push(trace.source.clone());
            }
        }
        if sources.is_empty() {
            for trace in self.source_map.traces_for(&fallback) {
                if seen.insert(trace.source.clone()) {
                    sources.push(trace.source.clone());
                }
            }
        }
        sources
    }

    fn instance_targets_for_component(&self, component: &ComponentId) -> Vec<AuthoringTarget> {
        self.circuit
            .instances
            .iter()
            .filter(|instance| &instance.component == component)
            .map(|instance| AuthoringTarget::Instance(instance.id.clone()))
            .collect()
    }

    fn circuit_issue_sources(&self, issue: &CircuitValidationIssue) -> Vec<SourceLocation> {
        let mut targets = match issue {
            CircuitValidationIssue::DuplicateNet(net)
            | CircuitValidationIssue::DuplicateRailIntent(net)
            | CircuitValidationIssue::UnknownRailNet(net)
            | CircuitValidationIssue::GroundRailMismatch(net)
            | CircuitValidationIssue::InvalidRailCurrent(net) => {
                vec![AuthoringTarget::Net(net.clone())]
            }
            CircuitValidationIssue::DuplicateBus(bus) | CircuitValidationIssue::EmptyBus(bus) => {
                vec![AuthoringTarget::Bus(bus.clone())]
            }
            CircuitValidationIssue::UnknownBusNet { bus, net }
            | CircuitValidationIssue::DuplicateBusNet { bus, net } => vec![
                AuthoringTarget::Bus(bus.clone()),
                AuthoringTarget::Net(net.clone()),
            ],
            CircuitValidationIssue::DuplicateBusSlice(slice)
            | CircuitValidationIssue::InvalidBusSliceRange(slice) => {
                vec![AuthoringTarget::BusSlice(slice.clone())]
            }
            CircuitValidationIssue::UnknownBusSliceBus { slice, bus } => vec![
                AuthoringTarget::BusSlice(slice.clone()),
                AuthoringTarget::Bus(bus.clone()),
            ],
            CircuitValidationIssue::DuplicatePort(port) => {
                vec![AuthoringTarget::Port(port.clone())]
            }
            CircuitValidationIssue::UnknownPortNet { port, net } => vec![
                AuthoringTarget::Port(port.clone()),
                AuthoringTarget::Net(net.clone()),
            ],
            CircuitValidationIssue::MultipleGroundNets => self
                .circuit
                .nets
                .iter()
                .filter(|net| net.is_ground)
                .map(|net| AuthoringTarget::Net(net.id.clone()))
                .collect(),
            CircuitValidationIssue::DuplicateDeviceModel(model)
            | CircuitValidationIssue::InvalidMosfetTerminals(model) => {
                vec![AuthoringTarget::DeviceModel(model.clone())]
            }
            CircuitValidationIssue::DuplicateDeviceModelPin { model, pin } => {
                let mut targets = vec![AuthoringTarget::DeviceModel(model.clone())];
                targets.extend(
                    self.circuit
                        .instances
                        .iter()
                        .filter(|instance| &instance.model == model)
                        .map(|instance| AuthoringTarget::Pin {
                            instance: instance.id.clone(),
                            pin: pin.clone(),
                        }),
                );
                targets
            }
            CircuitValidationIssue::DuplicateInstance(instance)
            | CircuitValidationIssue::UnknownInstanceModel { instance, .. } => {
                vec![AuthoringTarget::Instance(instance.clone())]
            }
            CircuitValidationIssue::UnknownInstancePin { instance, pin }
            | CircuitValidationIssue::MissingRequiredInstancePin { instance, pin }
            | CircuitValidationIssue::DuplicateInstancePin { instance, pin }
            | CircuitValidationIssue::UnknownInstanceNet { instance, pin, .. } => vec![
                AuthoringTarget::Instance(instance.clone()),
                AuthoringTarget::Pin {
                    instance: instance.clone(),
                    pin: pin.clone(),
                },
                AuthoringTarget::Connection {
                    instance: instance.clone(),
                    pin: pin.clone(),
                },
            ],
            CircuitValidationIssue::DuplicateComponent(component) => {
                self.instance_targets_for_component(component)
            }
            CircuitValidationIssue::DuplicateSourceStimulus(component)
            | CircuitValidationIssue::UnknownSourceStimulusComponent(component)
            | CircuitValidationIssue::InvalidSourceStimulusTarget(component)
            | CircuitValidationIssue::EmptySourceWaveform(component)
            | CircuitValidationIssue::NonIncreasingSourceWaveformTime(component)
            | CircuitValidationIssue::InvalidPulseSourceWaveform(component)
            | CircuitValidationIssue::InvalidSineSourceWaveform(component)
            | CircuitValidationIssue::InvalidExponentialSourceWaveform(component) => {
                let mut targets = self.instance_targets_for_component(component);
                targets.push(AuthoringTarget::Stimulus(component.clone()));
                targets
            }
            CircuitValidationIssue::UnknownStampNet { component, net } => {
                let mut targets = self.instance_targets_for_component(component);
                targets.push(AuthoringTarget::Net(net.clone()));
                targets
            }
            _ => Vec::new(),
        };
        if let CircuitValidationIssue::UnknownInstanceNet { net, .. } = issue {
            targets.push(AuthoringTarget::Net(net.clone()));
        }
        self.collect_diagnostic_sources(targets, AuthoringTarget::Circuit(self.circuit.id.clone()))
    }

    fn layout_issue_sources(&self, issue: &LayoutValidationIssue) -> Vec<SourceLocation> {
        let targets = match issue {
            LayoutValidationIssue::DuplicateLandPattern(pattern)
            | LayoutValidationIssue::InvalidLandPatternBody(pattern)
            | LayoutValidationIssue::UnknownPlacementLandPattern(pattern) => {
                vec![AuthoringTarget::LandPattern(pattern.clone())]
            }
            LayoutValidationIssue::InvalidPcb3dModelReference { land_pattern, .. } => {
                vec![AuthoringTarget::LandPattern(land_pattern.clone())]
            }
            LayoutValidationIssue::DuplicatePad { land_pattern, pad }
            | LayoutValidationIssue::UnknownMappedPad { land_pattern, pad } => vec![
                AuthoringTarget::LandPattern(land_pattern.clone()),
                AuthoringTarget::Pad {
                    land_pattern: land_pattern.clone(),
                    pad: pad.clone(),
                },
            ],
            LayoutValidationIssue::DuplicatePinPadMapping {
                land_pattern, pad, ..
            } => vec![
                AuthoringTarget::LandPattern(land_pattern.clone()),
                AuthoringTarget::Pad {
                    land_pattern: land_pattern.clone(),
                    pad: pad.clone(),
                },
            ],
            LayoutValidationIssue::UnknownPlacedInstance(instance)
            | LayoutValidationIssue::DuplicatePlacement(instance) => vec![
                AuthoringTarget::Instance(instance.clone()),
                AuthoringTarget::Placement(instance.clone()),
            ],
            LayoutValidationIssue::MultiplePlacementDrivers(instance) => {
                let mut targets = vec![
                    AuthoringTarget::Instance(instance.clone()),
                    AuthoringTarget::Placement(instance.clone()),
                ];
                targets.extend(
                    self.layout
                        .placement_constraints
                        .iter()
                        .filter(|constraint| {
                            matches!(
                                &constraint.kind,
                                PlacementConstraintKind::Fixed {
                                    instance: driven,
                                    ..
                                } | PlacementConstraintKind::Relative {
                                    instance: driven,
                                    ..
                                } if driven == instance
                            )
                        })
                        .map(|constraint| {
                            AuthoringTarget::PlacementConstraint(constraint.id.clone())
                        }),
                );
                targets
            }
            LayoutValidationIssue::DuplicatePlacementConstraint(constraint)
            | LayoutValidationIssue::InvalidPlacementConstraint(constraint)
            | LayoutValidationIssue::UnknownPlacementConstraintInstance { constraint, .. } => {
                vec![AuthoringTarget::PlacementConstraint(constraint.clone())]
            }
            LayoutValidationIssue::MissingPlacedPinPad { instance, pin }
            | LayoutValidationIssue::UnknownPlacedPatternPin { instance, pin } => vec![
                AuthoringTarget::Instance(instance.clone()),
                AuthoringTarget::Placement(instance.clone()),
                AuthoringTarget::Pin {
                    instance: instance.clone(),
                    pin: pin.clone(),
                },
                AuthoringTarget::Connection {
                    instance: instance.clone(),
                    pin: pin.clone(),
                },
            ],
            LayoutValidationIssue::UnknownRouteNet { route, net } => vec![
                AuthoringTarget::Route(route.clone()),
                AuthoringTarget::Net(net.clone()),
            ],
            LayoutValidationIssue::UnknownRouteLayer { route, .. } => {
                vec![AuthoringTarget::Route(route.clone())]
            }
            LayoutValidationIssue::EmptyRoute(route)
            | LayoutValidationIssue::NonPositiveRouteWidth(route)
            | LayoutValidationIssue::DisconnectedRoute(route)
            | LayoutValidationIssue::InvalidRouteArcWidth(route)
            | LayoutValidationIssue::DuplicateRoute(route) => {
                vec![AuthoringTarget::Route(route.clone())]
            }
            LayoutValidationIssue::UnknownViaNet { via, net } => vec![
                AuthoringTarget::Via(via.clone()),
                AuthoringTarget::Net(net.clone()),
            ],
            LayoutValidationIssue::InvalidVia(via) | LayoutValidationIssue::DuplicateVia(via) => {
                vec![AuthoringTarget::Via(via.clone())]
            }
            LayoutValidationIssue::UnknownZoneNet { zone, net } => vec![
                AuthoringTarget::Zone(zone.clone()),
                AuthoringTarget::Net(net.clone()),
            ],
            LayoutValidationIssue::UnknownZoneLayer { zone, .. } => {
                vec![AuthoringTarget::Zone(zone.clone())]
            }
            LayoutValidationIssue::InvalidZoneBoundary(zone)
            | LayoutValidationIssue::InvalidZonePolicy(zone)
            | LayoutValidationIssue::DuplicateZone(zone) => {
                vec![AuthoringTarget::Zone(zone.clone())]
            }
            LayoutValidationIssue::InvalidKeepoutBoundary(keepout)
            | LayoutValidationIssue::DuplicateKeepout(keepout) => {
                vec![AuthoringTarget::Keepout(keepout.clone())]
            }
            LayoutValidationIssue::DuplicateViaStyle(style)
            | LayoutValidationIssue::InvalidViaStyle(style) => {
                vec![AuthoringTarget::ViaStyle(style.clone())]
            }
            LayoutValidationIssue::DuplicateNetClass(class)
            | LayoutValidationIssue::NetClassInheritanceCycle(class)
            | LayoutValidationIssue::InvalidNetClassConstraint(class) => {
                vec![AuthoringTarget::NetClass(class.clone())]
            }
            LayoutValidationIssue::UnknownNetClassParent { class, parent } => vec![
                AuthoringTarget::NetClass(class.clone()),
                AuthoringTarget::NetClass(parent.clone()),
            ],
            LayoutValidationIssue::UnknownNetClassNet { class, net } => vec![
                AuthoringTarget::NetClass(class.clone()),
                AuthoringTarget::Net(net.clone()),
            ],
            LayoutValidationIssue::UnknownNetClassViaStyle { class, style } => vec![
                AuthoringTarget::NetClass(class.clone()),
                AuthoringTarget::ViaStyle(style.clone()),
            ],
            LayoutValidationIssue::NetInMultipleClasses(net) => {
                let mut targets = vec![AuthoringTarget::Net(net.clone())];
                targets.extend(
                    self.layout
                        .rules
                        .net_classes
                        .iter()
                        .filter(|class| class.nets.contains(net))
                        .map(|class| AuthoringTarget::NetClass(class.id.clone())),
                );
                targets
            }
            LayoutValidationIssue::DuplicateDifferentialPair(pair)
            | LayoutValidationIssue::InvalidDifferentialPair(pair) => {
                vec![AuthoringTarget::DifferentialPair(pair.clone())]
            }
            LayoutValidationIssue::DuplicateLengthTuningPattern(pattern)
            | LayoutValidationIssue::InvalidLengthTuningTarget(pattern)
            | LayoutValidationIssue::InvalidLengthTuningPattern(pattern) => {
                vec![AuthoringTarget::LengthTuningPattern(pattern.clone())]
            }
            LayoutValidationIssue::DuplicatePhaseTuningGroup(group)
            | LayoutValidationIssue::InvalidPhaseTuningTarget(group)
            | LayoutValidationIssue::InvalidPhaseTuningGroup(group) => {
                vec![AuthoringTarget::PhaseTuningGroup(group.clone())]
            }
            LayoutValidationIssue::UnknownEscapePolicyNet { net, .. }
            | LayoutValidationIssue::UnknownRouteConstraintRegionNet { net, .. }
            | LayoutValidationIssue::UnknownRouteRuleRegionNet { net, .. } => {
                vec![AuthoringTarget::Net(net.clone())]
            }
            _ => Vec::new(),
        };
        self.collect_diagnostic_sources(targets, AuthoringTarget::Board(self.layout.id.clone()))
    }

    fn schematic_issue_sources(&self, issue: &SchematicValidationIssue) -> Vec<SourceLocation> {
        let targets = match issue {
            SchematicValidationIssue::DuplicateSymbol(symbol)
            | SchematicValidationIssue::SymbolDefinitionModelMismatch(symbol)
            | SchematicValidationIssue::UnknownSymbolDefinition { symbol, .. }
            | SchematicValidationIssue::UnknownPlacedSymbolUnit { symbol, .. } => {
                vec![AuthoringTarget::Symbol(symbol.clone())]
            }
            SchematicValidationIssue::UnknownSymbolInstance(instance) => {
                vec![AuthoringTarget::Instance(instance.clone())]
            }
            SchematicValidationIssue::DuplicateSymbolDefinition(definition)
            | SchematicValidationIssue::InvalidSymbolDefinitionName(definition)
            | SchematicValidationIssue::InvalidSymbolUnitBody { definition, .. }
            | SchematicValidationIssue::InvalidSymbolUnit { definition, .. }
            | SchematicValidationIssue::DuplicateSymbolUnit { definition, .. }
            | SchematicValidationIssue::DuplicateSymbolDefinitionPin { definition, .. }
            | SchematicValidationIssue::UnknownSymbolDefinitionPin { definition, .. }
            | SchematicValidationIssue::InvalidSymbolGraphic { definition, .. }
            | SchematicValidationIssue::UnknownSymbolDefinitionModel { definition, .. } => {
                let mut targets = vec![AuthoringTarget::SymbolDefinition(definition.clone())];
                targets.extend(
                    self.schematic
                        .symbols
                        .iter()
                        .filter(|symbol| &symbol.definition == definition)
                        .map(|symbol| AuthoringTarget::Symbol(symbol.id.clone())),
                );
                targets
            }
            SchematicValidationIssue::DuplicateWire(wire)
            | SchematicValidationIssue::UnknownWirePinEndpoint { wire }
            | SchematicValidationIssue::UnknownWirePortEndpoint { wire }
            | SchematicValidationIssue::UnknownWireSheetPortEndpoint { wire }
            | SchematicValidationIssue::WireEndpointSheetMismatch { wire } => {
                vec![AuthoringTarget::Wire(wire.clone())]
            }
            SchematicValidationIssue::UnknownWireNet { wire, net }
            | SchematicValidationIssue::WireEndpointNetMismatch { wire, net } => vec![
                AuthoringTarget::Wire(wire.clone()),
                AuthoringTarget::Net(net.clone()),
            ],
            _ => Vec::new(),
        };
        self.collect_diagnostic_sources(
            targets,
            AuthoringTarget::Schematic(self.circuit.id.clone()),
        )
    }

    /// Replays authoritative circuit, schematic, and layout structural validation.
    pub fn check(&self) -> DesignCheckReport {
        let circuit = self.circuit.validate();
        let schematic = self.schematic.validate(&self.circuit);
        let layout = self.layout.validate(&self.circuit);
        let diagnostics = circuit
            .issues
            .iter()
            .cloned()
            .map(|issue| DesignDiagnostic {
                sources: self.circuit_issue_sources(&issue),
                issue: DesignValidationIssue::Circuit(issue),
            })
            .chain(
                schematic
                    .issues
                    .iter()
                    .cloned()
                    .map(|issue| DesignDiagnostic {
                        sources: self.schematic_issue_sources(&issue),
                        issue: DesignValidationIssue::Schematic(issue),
                    }),
            )
            .chain(layout.issues.iter().cloned().map(|issue| DesignDiagnostic {
                sources: self.layout_issue_sources(&issue),
                issue: DesignValidationIssue::Layout(issue),
            }))
            .collect();
        DesignCheckReport {
            circuit,
            schematic,
            layout,
            diagnostics,
        }
    }

    /// Finishes only after circuit, schematic, and PCB retained intent validate.
    pub fn finish(self) -> Result<CheckedDesign, DesignBuildError> {
        let report = self.check();
        if !report.is_valid() {
            return Err(DesignBuildError::InvalidDesign(report));
        }
        Ok(CheckedDesign {
            owner: self.owner,
            circuit: self.circuit,
            schematic: self.schematic,
            layout: self.layout,
            source_map: self.source_map,
        })
    }
}
