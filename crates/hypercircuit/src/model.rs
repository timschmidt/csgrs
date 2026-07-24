//! Exact circuit-domain carriers before solver lowering.
//!
//! Modified Nodal Analysis is the linear equation boundary, but a circuit
//! crate also needs explicit identities for circuits, instances, devices,
//! pins, nets, states, stamps, policies, and certification. This module keeps
//! those facts in circuit-owned structures before any sparse or transient
//! adapter is selected. Primitive floats are not circuit truth, and numeric
//! engines produce proposals that need exact residual replay. Netlist topology,
//! device models, stamps, state, and transient policy remain separate concerns.
//! The crate README collects the supporting SPICE, MNA, and exact-computation
//! references.

use std::collections::BTreeSet;

use hyperreal::{Real, RealSign};

use crate::{
    AdapterKind, BranchId, BusId, BusSliceId, CircuitError, CircuitId, CircuitInstanceId,
    ComponentId, DeviceModelId, LinearMnaSystem, LinearStamp, NetId, PartRef, PinRef, PortId,
    SubcircuitInstance, SubcircuitInstanceId,
};

/// Circuit net record.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Net {
    /// Net id.
    pub id: NetId,
    /// True when this net is the reference/ground net.
    pub is_ground: bool,
}

/// Ordered collection of nets exposed as one declarative bus.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Bus {
    /// Stable bus id.
    pub id: BusId,
    /// Ordered member nets. Order is part of the authored interface.
    pub nets: Vec<NetId>,
}

/// Ordering applied when exposing a retained subset of a bus.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BusSliceOrder {
    /// Preserve source bus declaration order.
    Forward,
    /// Reverse the selected source bus members.
    Reverse,
}

/// Named retained subset of an ordered bus.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct BusSlice {
    /// Stable slice identity in this circuit scope.
    pub id: BusSliceId,
    /// Source bus.
    pub bus: BusId,
    /// Zero-based first selected member in source order.
    pub offset: usize,
    /// Number of selected members.
    pub width: usize,
    /// Exposed member order.
    pub order: BusSliceOrder,
}

impl BusSlice {
    /// Resolves selected nets without duplicating them in retained storage.
    pub fn members(&self, bus: &Bus) -> Option<Vec<NetId>> {
        if self.bus != bus.id || self.width == 0 {
            return None;
        }
        let end = self.offset.checked_add(self.width)?;
        let mut members = bus.nets.get(self.offset..end)?.to_vec();
        if self.order == BusSliceOrder::Reverse {
            members.reverse();
        }
        Some(members)
    }
}

/// Electrical role of a named supply/reference rail.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum RailKind {
    /// Positive or negative power distribution rail.
    Power,
    /// Circuit reference/ground rail.
    Ground,
    /// Analog or measurement reference that is not circuit ground.
    Reference,
    /// Bias rail used by analog circuitry.
    AnalogBias,
    /// Source-specific rail role.
    Custom(String),
}

/// Exact voltage/current intent attached to one retained net.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, PartialEq)]
pub struct RailIntent {
    /// Rail net.
    pub net: NetId,
    /// Nominal voltage relative to circuit ground/reference.
    pub nominal_voltage: Option<Real>,
    /// Maximum expected sourced or carried current magnitude.
    pub max_current: Option<Real>,
    /// Authored electrical role.
    pub kind: RailKind,
}

/// Electrical direction of a circuit boundary port.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum PortDirection {
    /// Signal is driven into the circuit.
    Input,
    /// Signal is driven out of the circuit.
    Output,
    /// Signal may be driven in either direction.
    Bidirectional,
    /// Passive terminal with no asserted signal direction.
    Passive,
    /// Power source terminal.
    PowerInput,
    /// Power-producing terminal.
    PowerOutput,
    /// Reference or ground terminal.
    Ground,
}

/// Named circuit-boundary port tied to one retained net.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct CircuitPort {
    /// Stable port id.
    pub id: PortId,
    /// Net exposed through the port.
    pub net: NetId,
    /// Authored electrical direction.
    pub direction: PortDirection,
    /// Whether a parent may omit this port when instantiating the circuit.
    pub optional: bool,
}

/// Pin binding for a circuit instance.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct PinBinding {
    /// External or package pin reference.
    pub pin: PinRef,
    /// Net connected to the pin.
    pub net: NetId,
}

/// Circuit instance record.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, PartialEq)]
pub struct CircuitInstance {
    /// Instance id.
    pub id: CircuitInstanceId,
    /// Component id used by stamps.
    pub component: ComponentId,
    /// Optional part-library reference.
    pub part: Option<PartRef>,
    /// Device model used by the instance.
    pub model: DeviceModelId,
    /// Pin-to-net bindings.
    pub pins: Vec<PinBinding>,
    /// Exact instance parameters, such as R/L/C/source values.
    pub parameters: Vec<CircuitParameter>,
}

/// Exact circuit parameter with provenance text.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, PartialEq)]
pub struct CircuitParameter {
    /// Parameter name.
    pub name: String,
    /// Exact parameter value.
    pub value: Real,
    /// Unit label.
    pub unit: String,
    /// Source/provenance label.
    pub source: String,
}

/// Exact parameter target controlled by a reusable circuit-module parameter.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub enum CircuitModuleParameterTarget {
    /// One parameter retained directly on a child component instance.
    InstanceParameter {
        /// Local instance identity.
        instance: CircuitInstanceId,
        /// Existing instance-parameter name.
        parameter: String,
    },
    /// One parameter retained on a reusable device model.
    ModelParameter {
        /// Local device-model identity.
        model: DeviceModelId,
        /// Existing model-parameter name.
        parameter: String,
    },
    /// Forward the exact value into a nested subcircuit module parameter.
    SubcircuitParameter {
        /// Local nested subcircuit instance.
        instance: SubcircuitInstanceId,
        /// Parameter declared by that instance's child circuit.
        parameter: String,
    },
}

/// Declared exact interface parameter for a reusable circuit definition.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, PartialEq)]
pub struct CircuitModuleParameter {
    /// Stable name used by parent instances.
    pub name: String,
    /// Exact value used when a parent supplies no override.
    pub default: Real,
    /// Required unit shared by every target.
    pub unit: String,
    /// Default-value provenance.
    pub source: String,
    /// Explicit local parameters or nested interface parameters controlled.
    pub targets: Vec<CircuitModuleParameterTarget>,
}

/// Exact parent-authored override of one declared child-module parameter.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, PartialEq)]
pub struct CircuitModuleParameterOverride {
    /// Parameter name declared by the referenced child circuit.
    pub parameter: String,
    /// Exact replacement value.
    pub value: Real,
    /// Parent/editor/package provenance.
    pub source: String,
}

/// One exact point in a piecewise-linear independent-source waveform.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, PartialEq)]
pub struct SourceWaveformPoint {
    /// Exact simulation time.
    pub time: Real,
    /// Exact source value at `time`.
    pub value: Real,
}

/// Declarative time dependence for an independent voltage or current source.
///
/// Piecewise-linear values are held constant before the first point and after
/// the last point. Adjacent points must have strictly increasing exact times.
/// Pulse trains repeat indefinitely after their delay and retain exact
/// rise/high/fall timing rather than expanding cycles into sampled points.
/// Analytic sine and exponential sources retain their authored parameters and
/// evaluate through hyperreal rather than sampled float tables.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, PartialEq)]
pub enum SourceWaveform {
    /// A retained constant source, useful when no scalar model parameter exists.
    Constant(Real),
    /// An ideal discontinuous step: `initial` before `at`, then `final_value`.
    Step {
        /// Value strictly before the transition.
        initial: Real,
        /// Value at and after the transition.
        final_value: Real,
        /// Exact transition time.
        at: Real,
    },
    /// Linear interpolation between exact points with endpoint-value holding.
    PiecewiseLinear {
        /// Nonempty points in strictly increasing exact-time order.
        points: Vec<SourceWaveformPoint>,
    },
    /// A periodic SPICE-style pulse train with an implicit low interval.
    Pulse {
        /// Value before `delay` and outside the rise/high/fall interval.
        low_value: Real,
        /// Value at the top of the pulse.
        high_value: Real,
        /// Nonnegative time before the first pulse begins.
        delay: Real,
        /// Nonnegative linear rise duration. Zero produces an ideal edge.
        rise_time: Real,
        /// Nonnegative duration held at `high_value`.
        high_time: Real,
        /// Nonnegative linear fall duration. Zero produces an ideal edge.
        fall_time: Real,
        /// Strictly positive repetition period. It must be at least the sum of
        /// `rise_time`, `high_time`, and `fall_time`.
        period: Real,
    },
    /// A delayed, optionally damped SPICE-style sinusoid.
    Sine {
        /// Constant source offset.
        offset: Real,
        /// Peak sinusoidal excursion from `offset`.
        amplitude: Real,
        /// Nonnegative frequency in cycles per simulation-time unit.
        frequency: Real,
        /// Nonnegative time before oscillation and damping begin.
        delay: Real,
        /// Nonnegative exponential damping coefficient.
        damping: Real,
        /// Exact initial phase in degrees.
        phase_degrees: Real,
    },
    /// A SPICE-style exponential rise followed by an exponential return.
    Exponential {
        /// Value before `rise_delay`.
        initial: Real,
        /// Asymptotic value approached after `rise_delay`.
        pulsed: Real,
        /// Nonnegative rise onset.
        rise_delay: Real,
        /// Strictly positive rise time constant.
        rise_time_constant: Real,
        /// Fall onset, exactly no earlier than `rise_delay`.
        fall_delay: Real,
        /// Strictly positive fall time constant.
        fall_time_constant: Real,
    },
}

/// Component-addressed independent-source stimulus.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, PartialEq)]
pub struct SourceStimulus {
    /// Stable component identity of a voltage or current source instance.
    pub component: ComponentId,
    /// Retained exact waveform.
    pub waveform: SourceWaveform,
}

/// MOSFET channel polarity.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum MosfetPolarity {
    /// N-channel device with positive normalized gate/drain voltages.
    NChannel,
    /// P-channel device with negative physical gate/drain voltages.
    PChannel,
}

/// Device model family.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum DeviceModelKind {
    /// Resistor model.
    Resistor,
    /// Capacitor model.
    Capacitor,
    /// Inductor model.
    Inductor,
    /// Independent current source.
    CurrentSource,
    /// Independent voltage source.
    VoltageSource,
    /// Controlled source.
    ControlledSource,
    /// Two-terminal Shockley diode.
    Diode,
    /// Three-terminal Shichman-Hodges MOSFET with the body tied to source.
    Mosfet {
        /// N-channel or P-channel law orientation.
        polarity: MosfetPolarity,
        /// Model pin serving as drain.
        drain: PinRef,
        /// Model pin serving as gate.
        gate: PinRef,
        /// Model pin serving as source and body reference.
        source: PinRef,
    },
    /// Explicit placeholder for nonlinear devices.
    NonlinearPlaceholder,
    /// Source-specific model kind.
    Custom(String),
}

/// Electrical class of a modeled device pin for pre-layout ERC.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum PinElectricalKind {
    /// High-impedance signal input.
    Input,
    /// Actively driven signal output.
    Output,
    /// Signal pin that can both drive and receive.
    Bidirectional,
    /// Passive terminal, such as a resistor lead.
    Passive,
    /// Supply input.
    PowerInput,
    /// Supply source or regulator output.
    PowerOutput,
    /// Open-collector or open-drain output.
    OpenCollector,
    /// Open-emitter or open-source output.
    OpenEmitter,
    /// Intentionally unconnected package terminal.
    NotConnected,
}

/// One named pin in a reusable device model.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct DevicePin {
    /// Stable logical/package pin reference.
    pub pin: PinRef,
    /// Electrical class used by ERC.
    pub kind: PinElectricalKind,
    /// Whether an instance may omit this pin binding.
    pub optional: bool,
}

/// Device model definition.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, PartialEq)]
pub struct DeviceModel {
    /// Model id.
    pub id: DeviceModelId,
    /// Model kind.
    pub kind: DeviceModelKind,
    /// Declared electrical interface.
    pub pins: Vec<DevicePin>,
    /// Exact model parameters.
    pub parameters: Vec<CircuitParameter>,
}

/// Exact circuit state value.
#[derive(Clone, Debug, PartialEq)]
pub struct CircuitState {
    /// State name.
    pub name: String,
    /// Exact state value.
    pub value: Real,
    /// Unit label.
    pub unit: String,
}

/// Transient integration policy.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum TransientPolicy {
    /// DC or static solve, no transient companion model.
    Static,
    /// Trapezoidal companion model.
    Trapezoidal,
    /// Gear/BDF companion model.
    GearBdf { order: u8 },
    /// IDA-style DAE adapter boundary.
    IdaDaeAdapter,
}

/// Circuit-level certification report.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct CircuitCertificationReport {
    /// Report status.
    pub status: String,
    /// Evidence labels.
    pub evidence: Vec<String>,
}

/// Structural problem found while validating a declarative circuit graph.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum CircuitValidationIssue {
    /// Two nets share one stable id.
    DuplicateNet(NetId),
    /// More than one net was marked as the reference ground.
    MultipleGroundNets,
    /// Two buses share one stable id.
    DuplicateBus(BusId),
    /// A bus has no member nets.
    EmptyBus(BusId),
    /// A bus references a net absent from the circuit.
    UnknownBusNet { bus: BusId, net: NetId },
    /// A bus contains the same member net more than once.
    DuplicateBusNet { bus: BusId, net: NetId },
    /// Two named slices share one identity.
    DuplicateBusSlice(BusSliceId),
    /// A slice references a bus absent from the circuit.
    UnknownBusSliceBus { slice: BusSliceId, bus: BusId },
    /// A slice is empty, overflows, or extends beyond its source bus.
    InvalidBusSliceRange(BusSliceId),
    /// More than one rail-intent record targets the same net.
    DuplicateRailIntent(NetId),
    /// A rail-intent record references an absent net.
    UnknownRailNet(NetId),
    /// A ground rail and the net's ground marker disagree.
    GroundRailMismatch(NetId),
    /// Maximum rail current is nonpositive or indeterminate.
    InvalidRailCurrent(NetId),
    /// Two boundary ports share one stable id.
    DuplicatePort(PortId),
    /// A boundary port references a net absent from the circuit.
    UnknownPortNet { port: PortId, net: NetId },
    /// Two device models share one stable id.
    DuplicateDeviceModel(DeviceModelId),
    /// A MOSFET's D/G/S roles are duplicated or absent from its model pins.
    InvalidMosfetTerminals(DeviceModelId),
    /// A device model declares one pin more than once.
    DuplicateDeviceModelPin { model: DeviceModelId, pin: PinRef },
    /// Two instances share one stable instance id.
    DuplicateInstance(CircuitInstanceId),
    /// Two instances share one component identity used by simulation stamps.
    DuplicateComponent(ComponentId),
    /// Two child circuit instances share one identity in a circuit scope.
    DuplicateSubcircuitInstance(SubcircuitInstanceId),
    /// A child circuit binding references a parent net absent from the circuit.
    UnknownSubcircuitParentNet {
        instance: SubcircuitInstanceId,
        net: NetId,
    },
    /// A child circuit port is bound more than once.
    DuplicateSubcircuitPortBinding {
        instance: SubcircuitInstanceId,
        port: PortId,
    },
    /// A simulation stamp references a net absent from its circuit scope.
    UnknownStampNet { component: ComponentId, net: NetId },
    /// Two voltage-source stamps declare one branch-current identity.
    DuplicateStampBranch(BranchId),
    /// An instance references a device model absent from the circuit.
    UnknownInstanceModel {
        instance: CircuitInstanceId,
        model: DeviceModelId,
    },
    /// An instance binds a pin absent from its device model.
    UnknownInstancePin {
        instance: CircuitInstanceId,
        pin: PinRef,
    },
    /// An instance omits a required device-model pin.
    MissingRequiredInstancePin {
        instance: CircuitInstanceId,
        pin: PinRef,
    },
    /// An instance binds one physical/logical pin more than once.
    DuplicateInstancePin {
        instance: CircuitInstanceId,
        pin: PinRef,
    },
    /// An instance pin references a net absent from the circuit.
    UnknownInstanceNet {
        instance: CircuitInstanceId,
        pin: PinRef,
        net: NetId,
    },
    /// More than one retained source stimulus targets the same component.
    DuplicateSourceStimulus(ComponentId),
    /// A source stimulus targets no circuit instance.
    UnknownSourceStimulusComponent(ComponentId),
    /// A source stimulus targets a device other than an independent source.
    InvalidSourceStimulusTarget(ComponentId),
    /// A piecewise-linear source stimulus has no points.
    EmptySourceWaveform(ComponentId),
    /// Piecewise-linear source times are not exactly and strictly increasing.
    NonIncreasingSourceWaveformTime(ComponentId),
    /// A pulse source has negative/indeterminate timing, a nonpositive period,
    /// or active durations longer than its period.
    InvalidPulseSourceWaveform(ComponentId),
    /// A sine source has negative or indeterminate frequency, delay, or damping.
    InvalidSineSourceWaveform(ComponentId),
    /// An exponential source has invalid delay ordering or nonpositive time constants.
    InvalidExponentialSourceWaveform(ComponentId),
    /// Two module-interface parameters share one name.
    DuplicateModuleParameter(String),
    /// A module parameter has blank metadata or no targets.
    InvalidModuleParameter(String),
    /// Two module parameters attempt to control one retained target.
    DuplicateModuleParameterTarget(CircuitModuleParameterTarget),
    /// A module parameter references an absent local object or parameter.
    UnknownModuleParameterTarget {
        parameter: String,
        target: CircuitModuleParameterTarget,
    },
    /// A direct target's existing unit disagrees with its module interface.
    ModuleParameterUnitMismatch {
        parameter: String,
        target: CircuitModuleParameterTarget,
        expected: String,
        actual: String,
    },
    /// One child instance overrides the same parameter more than once.
    DuplicateSubcircuitParameterOverride {
        instance: SubcircuitInstanceId,
        parameter: String,
    },
    /// A child parameter override has a blank name or provenance label.
    InvalidSubcircuitParameterOverride {
        instance: SubcircuitInstanceId,
        parameter: String,
    },
    /// Authored manual stamps could become stale after module parameter substitution.
    ModuleParametersWithManualStamps,
}

/// Replayable structural validation of a declarative circuit graph.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct CircuitValidationReport {
    /// Every discovered issue in deterministic source order.
    pub issues: Vec<CircuitValidationIssue>,
}

impl CircuitValidationReport {
    /// True when the retained graph has no structural issues.
    pub fn is_valid(&self) -> bool {
        self.issues.is_empty()
    }
}

/// Exact MNA problem package.
#[derive(Clone, Debug, PartialEq)]
pub struct MnaProblem {
    /// Nets included in the MNA system.
    pub nets: Vec<NetId>,
    /// Exact linear stamps.
    pub stamps: Vec<LinearStamp>,
}

/// Circuit graph before solver lowering.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, PartialEq)]
pub struct Circuit {
    /// Circuit id.
    pub id: CircuitId,
    /// Nets.
    pub nets: Vec<Net>,
    /// Ordered named buses over retained nets.
    pub buses: Vec<Bus>,
    /// Named retained subsets of buses.
    #[cfg_attr(feature = "interchange", serde(default))]
    pub bus_slices: Vec<BusSlice>,
    /// Supply/reference intent attached to retained nets.
    #[cfg_attr(feature = "interchange", serde(default))]
    pub rails: Vec<RailIntent>,
    /// Named boundary ports for hierarchical composition.
    pub ports: Vec<CircuitPort>,
    /// Exact public parameters for reusable circuit instantiation.
    #[cfg_attr(feature = "interchange", serde(default))]
    pub module_parameters: Vec<CircuitModuleParameter>,
    /// Device models.
    pub device_models: Vec<DeviceModel>,
    /// Instances.
    pub instances: Vec<CircuitInstance>,
    /// Time-dependent intent for independent voltage and current sources.
    #[cfg_attr(feature = "interchange", serde(default))]
    pub source_stimuli: Vec<SourceStimulus>,
    /// Reusable child-circuit instances in this circuit scope.
    pub subcircuits: Vec<SubcircuitInstance>,
    /// Linear stamps prepared for MNA.
    pub stamps: Vec<LinearStamp>,
    /// Transient policy.
    pub transient_policy: TransientPolicy,
    /// Adapter family selected or proposed for this circuit.
    pub adapter_policy: AdapterKind,
}

impl Circuit {
    /// Creates an empty circuit with explicit transient and adapter policy.
    pub fn new(
        id: CircuitId,
        transient_policy: TransientPolicy,
        adapter_policy: AdapterKind,
    ) -> Self {
        Self {
            id,
            nets: Vec::new(),
            buses: Vec::new(),
            bus_slices: Vec::new(),
            rails: Vec::new(),
            ports: Vec::new(),
            module_parameters: Vec::new(),
            device_models: Vec::new(),
            instances: Vec::new(),
            source_stimuli: Vec::new(),
            subcircuits: Vec::new(),
            stamps: Vec::new(),
            transient_policy,
            adapter_policy,
        }
    }

    /// Adds one net.
    pub fn with_net(mut self, net: Net) -> Self {
        self.nets.push(net);
        self
    }

    /// Adds one ordered bus.
    pub fn with_bus(mut self, bus: Bus) -> Self {
        self.buses.push(bus);
        self
    }

    /// Adds one named bus slice.
    pub fn with_bus_slice(mut self, slice: BusSlice) -> Self {
        self.bus_slices.push(slice);
        self
    }

    /// Adds exact voltage/current intent for one rail net.
    pub fn with_rail(mut self, rail: RailIntent) -> Self {
        self.rails.push(rail);
        self
    }

    /// Adds one circuit-boundary port.
    pub fn with_port(mut self, port: CircuitPort) -> Self {
        self.ports.push(port);
        self
    }

    /// Declares one exact, unit-checked reusable-module parameter.
    pub fn with_module_parameter(mut self, parameter: CircuitModuleParameter) -> Self {
        self.module_parameters.push(parameter);
        self
    }

    /// Adds one device model.
    pub fn with_device_model(mut self, model: DeviceModel) -> Self {
        self.device_models.push(model);
        self
    }

    /// Adds one instance.
    pub fn with_instance(mut self, instance: CircuitInstance) -> Self {
        self.instances.push(instance);
        self
    }

    /// Adds a retained waveform for one independent-source component.
    pub fn with_source_stimulus(mut self, stimulus: SourceStimulus) -> Self {
        self.source_stimuli.push(stimulus);
        self
    }

    /// Instantiates one reusable child circuit.
    pub fn with_subcircuit(mut self, instance: SubcircuitInstance) -> Self {
        self.subcircuits.push(instance);
        self
    }

    /// Connects an instance pin to a retained circuit net.
    ///
    /// The method changes only declarative topology. Simulation stamps remain a
    /// separate lowering product so editing connectivity cannot silently leave
    /// an accepted numerical equation behind.
    pub fn connect_pin(
        &mut self,
        instance: &CircuitInstanceId,
        pin: PinRef,
        net: &NetId,
    ) -> crate::CircuitResult<()> {
        if !self.nets.iter().any(|candidate| &candidate.id == net) {
            return Err(CircuitError::MissingConnectionNet);
        }
        let target = self
            .instances
            .iter_mut()
            .find(|candidate| &candidate.id == instance)
            .ok_or(CircuitError::MissingInstance)?;
        if target.pins.iter().any(|binding| binding.pin == pin) {
            return Err(CircuitError::DuplicatePinBinding);
        }
        target.pins.push(PinBinding {
            pin,
            net: net.clone(),
        });
        Ok(())
    }

    /// Validates retained circuit identity, hierarchy boundaries, and connectivity.
    pub fn validate(&self) -> CircuitValidationReport {
        let mut issues = Vec::new();
        let mut net_ids = BTreeSet::new();
        let mut ground_count = 0_usize;
        for net in &self.nets {
            if !net_ids.insert(net.id.clone()) {
                issues.push(CircuitValidationIssue::DuplicateNet(net.id.clone()));
            }
            ground_count += usize::from(net.is_ground);
        }
        if ground_count > 1 {
            issues.push(CircuitValidationIssue::MultipleGroundNets);
        }

        let mut bus_ids = BTreeSet::new();
        for bus in &self.buses {
            if !bus_ids.insert(bus.id.clone()) {
                issues.push(CircuitValidationIssue::DuplicateBus(bus.id.clone()));
            }
            if bus.nets.is_empty() {
                issues.push(CircuitValidationIssue::EmptyBus(bus.id.clone()));
            }
            let mut members = BTreeSet::new();
            for net in &bus.nets {
                if !net_ids.contains(net) {
                    issues.push(CircuitValidationIssue::UnknownBusNet {
                        bus: bus.id.clone(),
                        net: net.clone(),
                    });
                }
                if !members.insert(net.clone()) {
                    issues.push(CircuitValidationIssue::DuplicateBusNet {
                        bus: bus.id.clone(),
                        net: net.clone(),
                    });
                }
            }
        }

        let buses = self
            .buses
            .iter()
            .map(|bus| (bus.id.clone(), bus))
            .collect::<std::collections::BTreeMap<_, _>>();
        let mut slice_ids = BTreeSet::new();
        for slice in &self.bus_slices {
            if !slice_ids.insert(slice.id.clone()) {
                issues.push(CircuitValidationIssue::DuplicateBusSlice(slice.id.clone()));
            }
            let Some(bus) = buses.get(&slice.bus) else {
                issues.push(CircuitValidationIssue::UnknownBusSliceBus {
                    slice: slice.id.clone(),
                    bus: slice.bus.clone(),
                });
                continue;
            };
            if slice.members(bus).is_none() {
                issues.push(CircuitValidationIssue::InvalidBusSliceRange(
                    slice.id.clone(),
                ));
            }
        }

        let ground_net = self
            .nets
            .iter()
            .find(|net| net.is_ground)
            .map(|net| &net.id);
        let mut rail_nets = BTreeSet::new();
        for rail in &self.rails {
            if !rail_nets.insert(rail.net.clone()) {
                issues.push(CircuitValidationIssue::DuplicateRailIntent(
                    rail.net.clone(),
                ));
            }
            if !net_ids.contains(&rail.net) {
                issues.push(CircuitValidationIssue::UnknownRailNet(rail.net.clone()));
            }
            let declares_ground = rail.kind == RailKind::Ground;
            let is_ground = ground_net == Some(&rail.net);
            if declares_ground != is_ground {
                issues.push(CircuitValidationIssue::GroundRailMismatch(rail.net.clone()));
            }
            if rail
                .max_current
                .as_ref()
                .is_some_and(|current| current.structural_facts().sign != Some(RealSign::Positive))
            {
                issues.push(CircuitValidationIssue::InvalidRailCurrent(rail.net.clone()));
            }
        }

        let mut port_ids = BTreeSet::new();
        for port in &self.ports {
            if !port_ids.insert(port.id.clone()) {
                issues.push(CircuitValidationIssue::DuplicatePort(port.id.clone()));
            }
            if !net_ids.contains(&port.net) {
                issues.push(CircuitValidationIssue::UnknownPortNet {
                    port: port.id.clone(),
                    net: port.net.clone(),
                });
            }
        }

        let mut model_ids = BTreeSet::new();
        for model in &self.device_models {
            if !model_ids.insert(model.id.clone()) {
                issues.push(CircuitValidationIssue::DuplicateDeviceModel(
                    model.id.clone(),
                ));
            }
            let mut pins = BTreeSet::new();
            for pin in &model.pins {
                if !pins.insert(pin.pin.clone()) {
                    issues.push(CircuitValidationIssue::DuplicateDeviceModelPin {
                        model: model.id.clone(),
                        pin: pin.pin.clone(),
                    });
                }
            }
            let invalid_mosfet_terminals = match &model.kind {
                DeviceModelKind::Mosfet {
                    drain,
                    gate,
                    source,
                    ..
                } => {
                    drain == gate
                        || drain == source
                        || gate == source
                        || !pins.contains(drain)
                        || !pins.contains(gate)
                        || !pins.contains(source)
                }
                _ => false,
            };
            if invalid_mosfet_terminals {
                issues.push(CircuitValidationIssue::InvalidMosfetTerminals(
                    model.id.clone(),
                ));
            }
        }

        let mut instance_ids = BTreeSet::new();
        let mut component_ids = BTreeSet::new();
        for instance in &self.instances {
            if !instance_ids.insert(instance.id.clone()) {
                issues.push(CircuitValidationIssue::DuplicateInstance(
                    instance.id.clone(),
                ));
            }
            if !component_ids.insert(instance.component.clone()) {
                issues.push(CircuitValidationIssue::DuplicateComponent(
                    instance.component.clone(),
                ));
            }
            let model = self
                .device_models
                .iter()
                .find(|candidate| candidate.id == instance.model);
            if !model_ids.contains(&instance.model) {
                issues.push(CircuitValidationIssue::UnknownInstanceModel {
                    instance: instance.id.clone(),
                    model: instance.model.clone(),
                });
            }
            let mut pins = BTreeSet::new();
            for binding in &instance.pins {
                if !pins.insert(binding.pin.clone()) {
                    issues.push(CircuitValidationIssue::DuplicateInstancePin {
                        instance: instance.id.clone(),
                        pin: binding.pin.clone(),
                    });
                }
                if !net_ids.contains(&binding.net) {
                    issues.push(CircuitValidationIssue::UnknownInstanceNet {
                        instance: instance.id.clone(),
                        pin: binding.pin.clone(),
                        net: binding.net.clone(),
                    });
                }
                if model.is_some_and(|model| {
                    !model
                        .pins
                        .iter()
                        .any(|candidate| candidate.pin == binding.pin)
                }) {
                    issues.push(CircuitValidationIssue::UnknownInstancePin {
                        instance: instance.id.clone(),
                        pin: binding.pin.clone(),
                    });
                }
            }
            if let Some(model) = model {
                for pin in model.pins.iter().filter(|pin| !pin.optional) {
                    if !instance.pins.iter().any(|binding| binding.pin == pin.pin) {
                        issues.push(CircuitValidationIssue::MissingRequiredInstancePin {
                            instance: instance.id.clone(),
                            pin: pin.pin.clone(),
                        });
                    }
                }
            }
        }

        let mut stimulus_components = BTreeSet::new();
        for stimulus in &self.source_stimuli {
            if !stimulus_components.insert(stimulus.component.clone()) {
                issues.push(CircuitValidationIssue::DuplicateSourceStimulus(
                    stimulus.component.clone(),
                ));
            }
            if let SourceWaveform::PiecewiseLinear { points } = &stimulus.waveform {
                if points.is_empty() {
                    issues.push(CircuitValidationIssue::EmptySourceWaveform(
                        stimulus.component.clone(),
                    ));
                }
                if points.windows(2).any(|pair| {
                    pair[0].time.partial_cmp(&pair[1].time) != Some(std::cmp::Ordering::Less)
                }) {
                    issues.push(CircuitValidationIssue::NonIncreasingSourceWaveformTime(
                        stimulus.component.clone(),
                    ));
                }
            }
            if let SourceWaveform::Pulse {
                delay,
                rise_time,
                high_time,
                fall_time,
                period,
                ..
            } = &stimulus.waveform
            {
                let nonnegative = |value: &Real| {
                    matches!(
                        value.structural_facts().sign,
                        Some(RealSign::Zero | RealSign::Positive)
                    )
                };
                let active_time = rise_time.clone() + high_time.clone() + fall_time.clone();
                if !nonnegative(delay)
                    || !nonnegative(rise_time)
                    || !nonnegative(high_time)
                    || !nonnegative(fall_time)
                    || period.structural_facts().sign != Some(RealSign::Positive)
                    || !matches!(
                        active_time.partial_cmp(period),
                        Some(std::cmp::Ordering::Less | std::cmp::Ordering::Equal)
                    )
                {
                    issues.push(CircuitValidationIssue::InvalidPulseSourceWaveform(
                        stimulus.component.clone(),
                    ));
                }
            }
            if let SourceWaveform::Sine {
                frequency,
                delay,
                damping,
                ..
            } = &stimulus.waveform
            {
                let nonnegative = |value: &Real| {
                    matches!(
                        value.structural_facts().sign,
                        Some(RealSign::Zero | RealSign::Positive)
                    )
                };
                if !nonnegative(frequency) || !nonnegative(delay) || !nonnegative(damping) {
                    issues.push(CircuitValidationIssue::InvalidSineSourceWaveform(
                        stimulus.component.clone(),
                    ));
                }
            }
            if let SourceWaveform::Exponential {
                rise_delay,
                rise_time_constant,
                fall_delay,
                fall_time_constant,
                ..
            } = &stimulus.waveform
            {
                let nonnegative = |value: &Real| {
                    matches!(
                        value.structural_facts().sign,
                        Some(RealSign::Zero | RealSign::Positive)
                    )
                };
                if !nonnegative(rise_delay)
                    || rise_time_constant.structural_facts().sign != Some(RealSign::Positive)
                    || !matches!(
                        rise_delay.partial_cmp(fall_delay),
                        Some(std::cmp::Ordering::Less | std::cmp::Ordering::Equal)
                    )
                    || fall_time_constant.structural_facts().sign != Some(RealSign::Positive)
                {
                    issues.push(CircuitValidationIssue::InvalidExponentialSourceWaveform(
                        stimulus.component.clone(),
                    ));
                }
            }
            let Some(instance) = self
                .instances
                .iter()
                .find(|instance| instance.component == stimulus.component)
            else {
                issues.push(CircuitValidationIssue::UnknownSourceStimulusComponent(
                    stimulus.component.clone(),
                ));
                continue;
            };
            let Some(model) = self
                .device_models
                .iter()
                .find(|model| model.id == instance.model)
            else {
                continue;
            };
            if !matches!(
                model.kind,
                DeviceModelKind::CurrentSource | DeviceModelKind::VoltageSource
            ) {
                issues.push(CircuitValidationIssue::InvalidSourceStimulusTarget(
                    stimulus.component.clone(),
                ));
            }
        }

        let mut subcircuit_ids = BTreeSet::new();
        for instance in &self.subcircuits {
            if !subcircuit_ids.insert(instance.id.clone()) {
                issues.push(CircuitValidationIssue::DuplicateSubcircuitInstance(
                    instance.id.clone(),
                ));
            }
            let mut ports = BTreeSet::new();
            for binding in &instance.ports {
                if !net_ids.contains(&binding.net) {
                    issues.push(CircuitValidationIssue::UnknownSubcircuitParentNet {
                        instance: instance.id.clone(),
                        net: binding.net.clone(),
                    });
                }
                if !ports.insert(binding.port.clone()) {
                    issues.push(CircuitValidationIssue::DuplicateSubcircuitPortBinding {
                        instance: instance.id.clone(),
                        port: binding.port.clone(),
                    });
                }
            }
            let mut overrides = BTreeSet::new();
            for parameter in &instance.parameter_overrides {
                if !overrides.insert(parameter.parameter.clone()) {
                    issues.push(
                        CircuitValidationIssue::DuplicateSubcircuitParameterOverride {
                            instance: instance.id.clone(),
                            parameter: parameter.parameter.clone(),
                        },
                    );
                }
                if parameter.parameter.trim().is_empty() || parameter.source.trim().is_empty() {
                    issues.push(CircuitValidationIssue::InvalidSubcircuitParameterOverride {
                        instance: instance.id.clone(),
                        parameter: parameter.parameter.clone(),
                    });
                }
            }
        }

        let mut module_parameter_names = BTreeSet::new();
        let mut module_parameter_targets = BTreeSet::new();
        for parameter in &self.module_parameters {
            if !module_parameter_names.insert(parameter.name.clone()) {
                issues.push(CircuitValidationIssue::DuplicateModuleParameter(
                    parameter.name.clone(),
                ));
            }
            if parameter.name.trim().is_empty()
                || parameter.unit.trim().is_empty()
                || parameter.source.trim().is_empty()
                || parameter.targets.is_empty()
            {
                issues.push(CircuitValidationIssue::InvalidModuleParameter(
                    parameter.name.clone(),
                ));
            }
            for target in &parameter.targets {
                if !module_parameter_targets.insert(target.clone()) {
                    issues.push(CircuitValidationIssue::DuplicateModuleParameterTarget(
                        target.clone(),
                    ));
                }
                let existing = match target {
                    CircuitModuleParameterTarget::InstanceParameter {
                        instance,
                        parameter,
                    } => self
                        .instances
                        .iter()
                        .find(|candidate| &candidate.id == instance)
                        .and_then(|instance| {
                            instance
                                .parameters
                                .iter()
                                .find(|candidate| candidate.name == *parameter)
                        }),
                    CircuitModuleParameterTarget::ModelParameter { model, parameter } => self
                        .device_models
                        .iter()
                        .find(|candidate| &candidate.id == model)
                        .and_then(|model| {
                            model
                                .parameters
                                .iter()
                                .find(|candidate| candidate.name == *parameter)
                        }),
                    CircuitModuleParameterTarget::SubcircuitParameter { instance, .. } => {
                        if self
                            .subcircuits
                            .iter()
                            .any(|candidate| &candidate.id == instance)
                        {
                            continue;
                        }
                        None
                    }
                };
                let Some(existing) = existing else {
                    issues.push(CircuitValidationIssue::UnknownModuleParameterTarget {
                        parameter: parameter.name.clone(),
                        target: target.clone(),
                    });
                    continue;
                };
                if existing.unit != parameter.unit {
                    issues.push(CircuitValidationIssue::ModuleParameterUnitMismatch {
                        parameter: parameter.name.clone(),
                        target: target.clone(),
                        expected: parameter.unit.clone(),
                        actual: existing.unit.clone(),
                    });
                }
            }
        }
        if !self.module_parameters.is_empty() && !self.stamps.is_empty() {
            issues.push(CircuitValidationIssue::ModuleParametersWithManualStamps);
        }

        let mut branches = BTreeSet::new();
        for stamp in &self.stamps {
            let (component, stamp_nets, branch) = match stamp {
                LinearStamp::Conductance {
                    component,
                    pos,
                    neg,
                    ..
                }
                | LinearStamp::CurrentSource {
                    component,
                    pos,
                    neg,
                    ..
                }
                | LinearStamp::Companion {
                    component,
                    pos,
                    neg,
                    ..
                } => (component, vec![pos, neg], None),
                LinearStamp::VoltageSource {
                    component,
                    branch,
                    pos,
                    neg,
                    ..
                } => (component, vec![pos, neg], Some(branch)),
                LinearStamp::Vccs {
                    component,
                    pos,
                    neg,
                    ctrl_pos,
                    ctrl_neg,
                    ..
                } => (component, vec![pos, neg, ctrl_pos, ctrl_neg], None),
            };
            for net in stamp_nets.into_iter().filter_map(Option::as_ref) {
                if !net_ids.contains(net) {
                    issues.push(CircuitValidationIssue::UnknownStampNet {
                        component: component.clone(),
                        net: net.clone(),
                    });
                }
            }
            if let Some(branch) = branch
                && !branches.insert(branch.clone())
            {
                issues.push(CircuitValidationIssue::DuplicateStampBranch(branch.clone()));
            }
        }

        CircuitValidationReport { issues }
    }

    /// Adds one exact linear stamp.
    pub fn with_stamp(mut self, stamp: LinearStamp) -> Self {
        self.stamps.push(stamp);
        self
    }

    /// Builds the exact dense MNA system for non-ground nets and linear stamps.
    pub fn linear_mna_system(&self) -> crate::CircuitResult<LinearMnaSystem> {
        let nets = self
            .nets
            .iter()
            .filter(|net| !net.is_ground)
            .map(|net| net.id.clone())
            .collect::<Vec<_>>();
        LinearMnaSystem::from_stamps(nets, &self.stamps)
    }
}

impl MnaProblem {
    /// Lowers this exact problem package to a dense MNA system.
    pub fn to_linear_system(&self) -> crate::CircuitResult<LinearMnaSystem> {
        LinearMnaSystem::from_stamps(self.nets.clone(), &self.stamps)
    }
}
