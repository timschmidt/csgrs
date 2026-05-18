//! Exact circuit-domain carriers before solver lowering.
//!
//! Modified Nodal Analysis is the linear equation boundary, but a circuit
//! crate also needs explicit identities for circuits, instances, devices,
//! pins, nets, states, stamps, policies, and certification. This module keeps
//! those facts in circuit-owned structures before any sparse or transient
//! adapter is selected. The boundary follows Yap, "Towards Exact Geometric
//! Computation," *Computational Geometry* 7(1-2), 1997
//! (<https://doi.org/10.1016/0925-7721(95)00040-2>): primitive floats are not
//! circuit truth, and numeric engines produce proposals that need exact
//! residual replay.
//!
//! The carrier split mirrors SPICE/MNA practice: netlist topology, device
//! models, stamps, state, and transient policy are separate concerns. See Ho,
//! Ruehli, and Brennan, "The Modified Nodal Approach to Network Analysis,"
//! 1975, and Nagel, "SPICE2: A Computer Program to Simulate Semiconductor
//! Circuits," 1975.

use hyperreal::Real;

use crate::{
    AdapterKind, CircuitId, CircuitInstanceId, ComponentId, DeviceModelId, LinearMnaSystem,
    LinearStamp, NetId, PartRef, PinRef,
};

/// Circuit net record.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Net {
    /// Net id.
    pub id: NetId,
    /// True when this net is the reference/ground net.
    pub is_ground: bool,
}

/// Pin binding for a circuit instance.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct PinBinding {
    /// External or package pin reference.
    pub pin: PinRef,
    /// Net connected to the pin.
    pub net: NetId,
}

/// Circuit instance record.
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

/// Device model family.
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
    /// Explicit placeholder for nonlinear devices.
    NonlinearPlaceholder,
    /// Source-specific model kind.
    Custom(String),
}

/// Device model definition.
#[derive(Clone, Debug, PartialEq)]
pub struct DeviceModel {
    /// Model id.
    pub id: DeviceModelId,
    /// Model kind.
    pub kind: DeviceModelKind,
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

/// Exact MNA problem package.
#[derive(Clone, Debug, PartialEq)]
pub struct MnaProblem {
    /// Nets included in the MNA system.
    pub nets: Vec<NetId>,
    /// Exact linear stamps.
    pub stamps: Vec<LinearStamp>,
}

/// Circuit graph before solver lowering.
#[derive(Clone, Debug, PartialEq)]
pub struct Circuit {
    /// Circuit id.
    pub id: CircuitId,
    /// Nets.
    pub nets: Vec<Net>,
    /// Device models.
    pub device_models: Vec<DeviceModel>,
    /// Instances.
    pub instances: Vec<CircuitInstance>,
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
            device_models: Vec::new(),
            instances: Vec::new(),
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
