//! Exact-aware circuit carriers and MNA residual replay.
//!
//! `hypercircuit` owns circuit-domain structure: instances, nets, linear MNA
//! stamps, unknown ordering, residual replay, and adapter reports for numeric
//! transient/DAE engines. It does not own part catalogs, geometry, routing, or
//! physics; those facts enter through explicit ids or report payloads.
//!
//! Numeric solvers may propose states, but accepted circuit facts must replay
//! through exact residual definitions or return explicit uncertainty. See the
//! crate README for the MNA, circuit-simulation, and exact-computation sources.

pub mod adapter;
pub mod coupling;
pub mod error;
pub mod identity;
pub mod mna;
pub mod model;
pub mod nonlinear;

pub use adapter::{AdapterKind, CircuitAdapterReport, ElectrothermalTraceFixture};
pub use coupling::{
    CoupledResidualBlock, ElectromechanicalPort, ElectrothermalRcReport, PhysicalElectricalPort,
    ThermalPort,
};
pub use error::{CircuitError, CircuitResult};
pub use hyperreal::Real;
pub use identity::{
    BranchId, CircuitId, CircuitInstanceId, ComponentId, DeviceModelId, NetId, PartRef, PinRef,
};
pub use mna::{LinearMnaSystem, LinearStamp, MnaUnknown, ResidualReplayReport};
pub use model::{
    Circuit, CircuitCertificationReport, CircuitInstance, CircuitParameter, CircuitState,
    DeviceModel, DeviceModelKind, MnaProblem, Net, PinBinding, TransientPolicy,
};
pub use nonlinear::{
    EventPolicy, NonlinearDeviceKind, NonlinearDeviceReport, PiecewiseLinearSegment, SwitchState,
};
