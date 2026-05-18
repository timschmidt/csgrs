//! Exact-aware circuit carriers and MNA residual replay.
//!
//! `hypercircuit` owns circuit-domain structure: instances, nets, linear MNA
//! stamps, unknown ordering, residual replay, and adapter reports for numeric
//! transient/DAE engines. It does not own part catalogs, geometry, routing, or
//! physics; those facts enter through explicit ids or report payloads.
//!
//! Linear stamping follows the Modified Nodal Analysis formulation of Ho,
//! Ruehli, and Brennan, "The Modified Nodal Approach to Network Analysis,"
//! *IEEE Transactions on Circuits and Systems* 22(6), 1975
//! (<https://doi.org/10.1109/TCS.1975.1084079>). Exact replay follows Yap,
//! "Towards Exact Geometric Computation," *Computational Geometry* 7(1-2),
//! 1997 (<https://doi.org/10.1016/0925-7721(95)00040-2>): numeric solvers may
//! propose states, but accepted circuit facts must replay through exact
//! residual definitions or return explicit uncertainty.

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
