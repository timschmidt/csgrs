//! Terminal and interface metadata.

use hyperreal::Real;

use crate::{PartsError, PartsResult, TerminalId};

/// Aspect of a part, such as package, symbol, body, tool head, or material region.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct PartAspect {
    /// Aspect handle.
    pub handle: String,
    /// Aspect kind.
    pub kind: AspectKind,
}

/// Part-aspect family.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum AspectKind {
    /// EDA symbol aspect.
    Symbol,
    /// Footprint/package aspect.
    Package,
    /// Mechanical body aspect.
    Body,
    /// Electrical interface aspect.
    Electrical,
    /// Thermal interface aspect.
    Thermal,
    /// Tool/process aspect.
    Tool,
    /// Source-specific aspect.
    Custom(String),
}

/// Electrical polarity class used by safe-connection queries.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ElectricalPolarity {
    /// Ground or common return.
    Ground,
    /// Positive or negative supply rail.
    Power,
    /// Signal terminal.
    Signal,
    /// Passive or mechanically constrained terminal.
    Passive,
    /// Source did not specify polarity.
    Unknown,
}

/// Interface family for a group of terminals.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum InterfaceKind {
    /// PCB symbol/footprint electrical interface.
    Electrical,
    /// Mechanical mating interface.
    Mechanical,
    /// Fluidic interface.
    Fluid,
    /// Optical interface.
    Optical,
    /// Custom source-specific interface kind.
    Custom(String),
}

/// Exact voltage envelope for electrical terminals.
#[derive(Clone, Debug, PartialEq)]
pub struct VoltageEnvelope {
    /// Minimum supported voltage.
    pub min: Real,
    /// Maximum supported voltage.
    pub max: Real,
}

/// A named terminal on a variant.
#[derive(Clone, Debug, PartialEq)]
pub struct Terminal {
    id: TerminalId,
    name: String,
    polarity: ElectricalPolarity,
    voltage: Option<VoltageEnvelope>,
}

/// A grouped interface on a variant.
#[derive(Clone, Debug, PartialEq)]
pub struct Interface {
    name: String,
    kind: InterfaceKind,
    terminals: Vec<TerminalId>,
}

/// Port on an interface.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Port {
    /// Port handle.
    pub handle: String,
    /// Terminal ids exposed by this port.
    pub terminals: Vec<TerminalId>,
}

/// Semantic role of a terminal.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum TerminalRole {
    /// Package pin.
    Pin,
    /// PCB pad.
    Pad,
    /// Lead or wire.
    Lead,
    /// Mounting/mechanical terminal.
    Mounting,
    /// Thermal pad or sink.
    Thermal,
    /// Source-specific role.
    Custom(String),
}

/// Package pin metadata.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Pin {
    /// Pin terminal id.
    pub terminal: TerminalId,
    /// Pin number/name.
    pub name: String,
}

/// Footprint pad metadata.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Pad {
    /// Pad terminal id.
    pub terminal: TerminalId,
    /// Pad number/name.
    pub name: String,
}

/// Lead metadata.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Lead {
    /// Lead terminal id.
    pub terminal: TerminalId,
    /// Lead description.
    pub description: String,
}

/// Mounting hole metadata.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Hole {
    /// Hole handle.
    pub handle: String,
    /// Grid or drill descriptor.
    pub descriptor: String,
}

/// Mounting feature metadata.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct MountingFeature {
    /// Feature handle.
    pub handle: String,
    /// Feature descriptor.
    pub descriptor: String,
}

/// Material region handle.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct MaterialRegion {
    /// Region handle owned by a downstream crate or importer.
    pub handle: String,
    /// Material label or requirement.
    pub material: String,
}

/// Reference designator class.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum ReferenceDesignatorClass {
    /// Resistor.
    R,
    /// Capacitor.
    C,
    /// Inductor.
    L,
    /// Diode.
    D,
    /// Integrated circuit.
    U,
    /// Connector.
    J,
    /// Mechanical part.
    M,
    /// Source-specific prefix.
    Custom(String),
}

impl PartAspect {
    /// Creates a part aspect.
    pub fn new(handle: impl Into<String>, kind: AspectKind) -> Self {
        Self {
            handle: handle.into(),
            kind,
        }
    }
}

impl VoltageEnvelope {
    /// Creates an exact voltage envelope after validating `min <= max`.
    pub fn new(min: Real, max: Real) -> PartsResult<Self> {
        match crate::predicate::compare(&min, &max) {
            Some(std::cmp::Ordering::Less | std::cmp::Ordering::Equal) => Ok(Self { min, max }),
            Some(std::cmp::Ordering::Greater) | None => Err(PartsError::InvalidVoltageEnvelope),
        }
    }

    /// Returns true when two certified envelopes overlap.
    pub fn overlaps(&self, other: &Self) -> Option<bool> {
        crate::predicate::closed_intervals_overlap(&self.min, &self.max, &other.min, &other.max)
    }
}

impl Terminal {
    /// Creates a terminal with optional voltage metadata.
    pub fn new(
        id: TerminalId,
        name: impl Into<String>,
        polarity: ElectricalPolarity,
        voltage: Option<VoltageEnvelope>,
    ) -> Self {
        Self {
            id,
            name: name.into(),
            polarity,
            voltage,
        }
    }

    /// Returns the terminal id.
    pub const fn id(&self) -> &TerminalId {
        &self.id
    }

    /// Returns the display name.
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Returns the polarity classification.
    pub const fn polarity(&self) -> ElectricalPolarity {
        self.polarity
    }

    /// Returns the voltage envelope, if known.
    pub const fn voltage(&self) -> Option<&VoltageEnvelope> {
        self.voltage.as_ref()
    }
}

impl Interface {
    /// Creates an interface with terminal ids.
    pub fn new(name: impl Into<String>, kind: InterfaceKind, terminals: Vec<TerminalId>) -> Self {
        Self {
            name: name.into(),
            kind,
            terminals,
        }
    }

    /// Returns the interface name.
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Returns the interface kind.
    pub const fn kind(&self) -> &InterfaceKind {
        &self.kind
    }

    /// Returns terminal ids in this interface.
    pub fn terminals(&self) -> &[TerminalId] {
        &self.terminals
    }
}
