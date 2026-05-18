//! Exact-aware physical carriers for the Hyper ecosystem.
//!
//! `hyperphysics` owns physical interpretation: materials, bodies, fixtures,
//! and mass-property reports. Geometry and scalar truth remain in the lower
//! crates. This crate therefore uses [`hyperreal::Real`] and
//! [`hyperlattice::Vector3`] directly and keeps primitive floats out of the
//! core API.
//!
//! The boundary follows Yap, "Towards Exact Geometric Computation,"
//! *Computational Geometry* 7(1-2), 1997
//! (<https://doi.org/10.1016/0925-7721(95)00040-2>): runtime engines may use
//! lossy proposal data, but physical properties derived from authored geometry
//! should retain exact object facts and return explicit uncertainty or errors
//! instead of silently accepting tolerance-based topology.

pub mod body;
pub mod contact;
pub mod em;
pub mod error;
pub mod fluid;
pub mod integration;
pub mod mass;
pub mod material;
pub mod optics;
pub mod photochemistry;
pub mod property;
pub mod residual;
pub mod shape;
pub mod thermal;

pub use body::{BodyId, BodyKind, ExactBody3, ExactFixture3, FixtureId};
pub use contact::{AabbContactReport3, ContactClassification, ContactMaterial};
pub use em::{
    BoundaryConditionKind, ElectromagneticBoundaryCondition3, ElectromagneticFieldRegion3,
    ElectromagneticMaterial, ElectromagneticRegime, ElectromagneticReportStatus, FieldRegionKind,
    LinearIsotropicElectricReport3,
};
pub use error::{PhysicsError, PhysicsResult};
pub use fluid::{
    FluidBoundary3, FluidBoundaryKind, FluidConservationReport3, FluidFixture3, FluidMaterial,
    FluidParticle3, FluidPolicy, FluidReportStatus,
};
pub use hyperlattice::Vector3;
pub use hyperreal::Real;
pub use integration::{
    CouplingPolicy, DiagnosticStatus, ForceAccumulator3, ForceContribution3, IntegrationPolicy,
    StepReplayReport3, SystemDiagnostics3,
};
pub use mass::{MassPropertyCertificate3, MassPropertyReport3, SymmetricInertia3};
pub use material::{ExactMaterial, MaterialId};
pub use optics::{
    BeerLambertSlabReport, FresnelNormalReport, OpticalInterface3, OpticalMedium, OpticalRay3,
    OpticalReportStatus, RayInterfaceClassification, SnellNormalReport,
};
pub use photochemistry::{
    CureDecision, CureStatus, DiffusiveCourantReport, ExposureMode, PhotochemicalConcentrations,
    PhotochemicalPolicy, ReactionDiffusionState, ReactionDiffusionTransport,
    VatPhotopolymerWorkingCurve, WorkingCurveReport,
};
pub use property::{
    ElasticDerivationReport, ExternalReplacementStatus, MaterialAssertion, MaterialPropertyGraph,
    MaterialPropertyKind, MaterialState, PhysicalPort, PhysicsCertificationReport,
    PropertyResolutionStatus, PropertyTensor, PropertyValue, ResolvedPropertyReport, SourceSpec,
};
pub use residual::{HypersolveResidualReplayReport, HypersolveResidualRow};
pub use shape::{
    AxisAlignedBox3, BoxPointClassification, ClosedTriangleMesh3, PhysicsShape3, Plane3,
    PlanePointClassification, PlanePointReport3, Ray3, RayPlaneClassification, RayPlaneReport3,
    Segment3, SegmentPlaneClassification, SegmentPlaneReport3, ShapeClassificationReport3,
    SupportMapReport3, Triangle3, TrianglePointClassification, TrianglePointReport3,
};
pub use thermal::{
    HeatFluxBoundary3, HeatSource3, LumpedRcThermalStepReport, LumpedThermalNode,
    SteadySlabConductionReport, TemperatureField3, ThermalContactPair3, ThermalMaterial,
    ThermalPolicy, ThermalPort3, ThermalReportStatus, TransientThermalStepReport,
};
