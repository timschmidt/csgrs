//! Exact-aware circuit carriers and MNA residual replay.
//!
//! `hypercircuit` owns circuit-domain structure: instances, nets, ports, buses,
//! declarative PCB intent, linear MNA stamps, unknown ordering, residual replay,
//! and adapter reports for numeric transient/DAE engines. `hyperpath` owns route
//! and via predicate carriers, while `csgrs` owns geometry materialization.
//!
//! Numeric solvers may propose states, but accepted circuit facts must replay
//! through exact residual definitions or return explicit uncertainty. See the
//! crate README for the MNA, circuit-simulation, and exact-computation sources.

pub mod ac;
pub mod adapter;
#[cfg(feature = "layout")]
pub mod assembly;
#[cfg(feature = "layout")]
pub mod authoring;
#[cfg(feature = "layout")]
pub mod autoroute;
pub mod behavior;
#[cfg(feature = "layout")]
pub mod coordinates;
pub mod coupling;
#[cfg(feature = "drc")]
pub mod drc;
#[cfg(feature = "interchange")]
pub mod edit;
pub mod erc;
pub mod error;
pub mod event_simulation;
#[cfg(feature = "geometry")]
pub mod fabrication;
pub mod hierarchy;
pub mod identity;
pub mod intent;
#[cfg(feature = "interchange")]
pub mod interchange;
pub mod interface;
#[cfg(feature = "layout")]
pub mod kicad;
#[cfg(feature = "layout")]
pub mod kicad_import;
#[cfg(feature = "interchange")]
pub mod kicad_library;
pub mod kicad_schematic;
#[cfg(feature = "layout")]
pub mod layout;
#[cfg(feature = "layout")]
pub mod layout_module;
#[cfg(feature = "lceda")]
pub mod lceda;
#[cfg(feature = "geometry")]
pub mod legacy_csgrs;
#[cfg(all(feature = "drc", feature = "interchange"))]
pub mod manufacturing_release;
#[cfg(feature = "geometry")]
pub mod materialize;
pub mod mna;
pub mod model;
pub mod mosfet;
pub mod nonlinear;
pub mod package;
#[cfg(feature = "layout")]
pub mod panel;
#[cfg(feature = "layout")]
pub mod placement;
#[cfg(feature = "layout")]
pub mod preview;
#[cfg(feature = "layout")]
pub mod project;
#[cfg(feature = "interchange")]
pub mod project_manifest;
#[cfg(feature = "interchange")]
pub mod release_artifact;
#[cfg(feature = "layout")]
pub mod route_constraints;
#[cfg(feature = "layout")]
pub mod routing;
pub mod schematic;
pub mod schematic_auto;
mod sexp;
pub mod simulation;
#[cfg(feature = "layout")]
pub mod stitching;
#[cfg(feature = "layout")]
pub mod test_intent;
#[cfg(feature = "layout")]
pub mod tscircuit_routing;
#[cfg(feature = "drc")]
pub mod workflow;

pub use ac::{
    AcAnalysisError, AcDeviceLoweringIssue, AcDeviceLoweringReport, AcExcitation, AcMnaSystem,
    AcNonlinearLinearization, AcOperatingPoint, AcOperatingPointDeviceKind, AcOperatingPointError,
    AcOperatingPointProvenance, AcResidualReplayReport, AcSmallSignalSweepReport, AcSolveReport,
    AcStamp, AcSweepPoint, AcSweepReport, Phasor,
};
pub use adapter::{AdapterKind, CircuitAdapterReport, ElectrothermalTraceFixture};
#[cfg(feature = "layout")]
pub use assembly::{
    ASSEMBLY_DATA_SCHEMA, ASSEMBLY_DATA_VERSION, AssemblyComponentV2, AssemblyCsvDialect,
    AssemblyCsvDocument, AssemblyCsvField, AssemblyDataV2, AssemblyDnpRow, AssemblyEvidence,
    AssemblyOutputs, AssemblyPartOverride, AssemblyRoundTripIssue, AssemblyRoundTripReport,
    AssemblyVariant, AssemblyVariantIssue, AssemblyVariantValidationReport, BomLine,
    PickAndPlaceRow,
};
#[cfg(feature = "layout")]
pub use authoring::parts;
#[cfg(feature = "layout")]
pub use authoring::{
    AuthoringAction, AuthoringTarget, AuthoringTrace, BusHandle, BusSliceHandle, CheckedDesign,
    Design, DesignBuildError, DesignCheckReport, DesignDiagnostic, DesignSourceMap,
    DesignValidationIssue, DifferentialPairHandle, DifferentialPairRule, Footprint, InstanceHandle,
    Keepout, KeepoutHandle, LengthTuningPatternHandle, LengthTuningRule, NetClassHandle,
    NetClassRule, NetHandle, Part, PartDefinition, PartDefinitionHandle, PartInstance, PartPin,
    PartSymbolUnit, PhaseTuningGroupHandle, PhaseTuningGroupRule, PinHandle,
    PlacementConstraintHandle, PlacementRule, PortHandle, Route, RouteHandle, SourceLocation,
    Symbol, SymbolPin, SymbolUnitPlacement, Via, ViaHandle, ViaStyleHandle, ViaStyleRule, Zone,
    ZoneHandle, pin,
};
#[cfg(feature = "layout")]
pub use autoroute::{
    NegotiatedAdaptiveRoutePolicy, NegotiatedAdaptiveRouteReport, NegotiatedAdaptiveRouteRound,
    NegotiatedAdaptiveRouteStatus, NegotiatedDifferentialPairEvidence,
    NegotiatedDifferentialPairNeckdownEvidence, NegotiatedEscapePolicyEvidence,
    NegotiatedGridEvidence, NegotiatedGridMode, NegotiatedGridRefinementRegion,
    NegotiatedPlanarTopology, NegotiatedRouteConflictGeometry, NegotiatedRouteConflictState,
    NegotiatedRouteConstraintEvidence, NegotiatedRouteFailure, NegotiatedRouteIteration,
    NegotiatedRouteIterationState, NegotiatedRouteNetState, NegotiatedRouteNodeState,
    NegotiatedRoutePolicy, NegotiatedRouteReport, NegotiatedRouteRuleRegionEvidence,
    NegotiatedRouteStatus, NegotiatedRouteWorkEvidence, NegotiatedRouterError,
    NegotiatedViaStyleEvidence,
};
#[cfg(feature = "behavior-async")]
pub use behavior::{AsyncBehaviorRuntime, AsyncBehaviorStatus, AsyncCircuitBehavior};
pub use behavior::{
    BehaviorContext, BehaviorError, BehaviorQueue, BehaviorRuntime, CircuitEventHandler,
    SelectiveEventMailbox,
};
#[cfg(feature = "layout")]
pub use coordinates::{
    AxisDirection, CoordinateFrame2, CoordinateTransformError, Handedness, LengthUnit,
    PanelToBoardTransform, RigidTransform2, RotationConvention, ViewConvention,
};
pub use coupling::{
    CoupledResidualBlock, ElectromechanicalPort, ElectrothermalRcReport, PhysicalElectricalPort,
    ThermalPort,
};
#[cfg(feature = "drc")]
pub use drc::{
    DrcCopperLayerImage, DrcDielectricMaterialEvidence, DrcHandoffOmission, DrcProcessLayer,
    DrcReadinessPolicy, HyperDrcHandoff, HyperDrcReadinessReport, ImpedanceTargetPolicy,
    PCB_LOSS_TANGENT_PROPERTY, PCB_RELATIVE_PERMITTIVITY_PROPERTY, PcbDielectricProperty,
    PcbMaterialPropertyIssue, PcbMaterialPropertyLibrary,
};
#[cfg(feature = "interchange")]
pub use edit::{
    ConcurrentCommitReport, DESIGN_HISTORY_MIN_MIGRATABLE_VERSION, DESIGN_HISTORY_SCHEMA,
    DESIGN_HISTORY_VERSION, DesignEdit, DesignEditBatch, DesignEditError, DesignHistory,
    DesignHistoryAction, DesignHistoryError, DesignHistoryMigrationReport,
    DesignHistoryReplayReport, DesignRevision, EditAddress, EditReplayReport, EditTarget,
    ReversibleDesignEdit,
};
pub use erc::{
    ConfiguredErcReport, CopperTraceErcIssue, CopperTraceErcReport, ErcEndpoint, ErcFinding,
    ErcIssue, ErcReport, ErcRuleDeck, ErcRuleId, ErcSeverity, RoleNetKind, SupplySource,
};
pub use error::{CircuitError, CircuitResult};
pub use event_simulation::{
    CircuitEvent, CircuitEventAgenda, CircuitEventAgendaError, CircuitEventCause,
    CircuitEventCounters, CircuitEventKind, CircuitEventLifecycleAction,
    CircuitEventLifecycleRecord, CircuitEventPhase, CircuitEventReplayIssue,
    CircuitEventReplayReport, CircuitEventRequest, CircuitEventTarget, CircuitEventValidationIssue,
    CircuitEventValue, CircuitSimulationFingerprint, ExactBreakpointError, ExactBreakpointProvider,
    ExactBreakpointSchedule, LogicValue, StochasticSample, StochasticStream, StochasticStreamError,
    earliest_exact_breakpoint_after,
};
#[cfg(feature = "geometry")]
pub use fabrication::{
    FABRICATION_MANIFEST_SCHEMA, FABRICATION_MANIFEST_VERSION, FabricationContourProjectionPolicy,
    FabricationExportOptions, FabricationFile, FabricationFileKind, FabricationIntegrityIssue,
    FabricationLengthUnit, FabricationManifest, FabricationManifestError, FabricationManifestFile,
    FabricationPackage, FabricationPackageError, FabricationTestPoint, FabricationTestPointKind,
};
#[cfg(feature = "drc")]
pub use fabrication::{
    FabricationCamRoundTripIssue, FabricationCamRoundTripReport,
    FabricationExcellonRoundTripEvidence, FabricationGerberRoundTripEvidence,
    FabricationIpc356RoundTripEvidence,
};
pub use hierarchy::{
    CircuitFlatteningReport, CircuitLibrary, CircuitLibraryValidationIssue,
    CircuitLibraryValidationReport, FlattenedCircuitScope, HierarchyError, IntentHierarchyError,
    SubcircuitInstance, SubcircuitPortBinding,
};
pub use hyperreal::Real;
pub use identity::{
    AssemblyVariantId, BoardId, BranchId, BundleEndpointId, BundleMemberId, BusId, BusSliceId,
    CircuitEventId, CircuitId, CircuitInstanceId, CircuitPackageName, ComponentId, DesignEditId,
    DeviceModelId, DifferentialPairId, EscapePolicyId, KeepoutId, LandPatternGraphicId,
    LandPatternId, LayoutModuleId, LengthTuningPatternId, ModportId, NetClassId, NetId, PadId,
    PartRef, PhaseTuningGroupId, PinRef, PlacementConstraintId, PlacementGroupId, PortId,
    RandomDrawId, RouteConstraintRegionId, RouteId, RouteRuleRegionId, SchematicLabelId,
    SchematicSheetId, SchematicSheetLinkId, SchematicSheetPortId, SchematicSymbolDefinitionId,
    SchematicSymbolId, SchematicWireId, SignalBundleId, SubcircuitInstanceId, ViaId, ViaStyleId,
    ZoneId,
};
pub use intent::{
    DesignIntent, DesignIntentIssue, DesignIntentValidationReport, DimensionedValue,
    FunctionalBinding, FunctionalBindingTarget, FunctionalRole, FunctionalRoleAssignment,
    FunctionalRoleTarget, NetIntent, NetKind, NetScope, PartClass, PartSelectionIntent,
    QuantityDimension, ResolvedPartEvidence, SemanticOrigin, SemanticTarget, SourcePosition,
    SourceSpan,
};
#[cfg(feature = "interchange")]
pub use interchange::{
    SEMANTIC_SCHEMA, SEMANTIC_SCHEMA_MIN_MIGRATABLE_VERSION, SEMANTIC_SCHEMA_VERSION,
    SemanticDocument, SemanticInterchangeError, SemanticMigrationReport, SemanticMigrationStep,
};
pub use interface::{
    BundlePortBinding, Modport, ModportMember, SignalBundle, SignalBundleBindingError,
    SignalBundleEndpoint, SignalBundleLibrary, SignalBundleMember, SignalBundleValidationIssue,
    SignalBundleValidationReport,
};
#[cfg(feature = "layout")]
pub use kicad::{
    KiCadDesignRuleProjection, KiCadExportError, KiCadExportOmission, KiCadExportOptions,
    KiCadExportReport, KiCadNumericProjection, KiCadProjectNetClassProjection,
};
#[cfg(feature = "layout")]
pub use kicad_import::{
    KiCadImportError, KiCadImportOmission, KiCadImportOptions, KiCadImportReport,
    KiCadNumericImport,
};
#[cfg(feature = "interchange")]
pub use kicad_library::{
    KiCadLibraryImportError, KiCadLibraryImportOmission, KiCadLibraryNumericImport,
    KiCadPartLibraryImportOptions, KiCadPartLibraryImportReport,
};
pub use kicad_schematic::{
    KiCadSchematicBookExportOptions, KiCadSchematicBookExportReport, KiCadSchematicBookFile,
    KiCadSchematicBookImportReport, KiCadSchematicExportError, KiCadSchematicExportOmission,
    KiCadSchematicExportOptions, KiCadSchematicExportReport, KiCadSchematicImportError,
    KiCadSchematicImportOmission, KiCadSchematicImportReport, KiCadSchematicNumericImport,
    KiCadSchematicNumericProjection, KiCadSchematicSheetImport, KiCadSchematicSheetProjection,
    KiCadSchematicSymbolProjection, KiCadSchematicWireProjection,
};
#[cfg(feature = "layout")]
pub use layout::{
    BoardBoundaryGeometry, BoardBoundaryGeometryError, BoardContour, BoardContourSegment,
    BoardOutline, BoardSide, CopperZone, CopperZoneConnection, CopperZoneFill,
    CopperZoneIslandPolicy, CopperZoneStitchingPolicy, DifferentialPair, DifferentialPairNeckdown,
    DrillShape, EscapePolicy, KeepoutScope, LandPattern, LandPatternBody, LandPatternGraphic,
    LandPatternGraphicPrimitive, LandPatternPad, LayerRole, LayoutValidationIssue,
    LayoutValidationReport, LengthTuningPattern, LengthTuningSide, NetClass,
    NetClassResolutionError, PadPinMap, PadShape, Pcb3dModelFormat, Pcb3dModelReference,
    Pcb3dModelTransform, PcbDesignRules, PcbKeepout, PcbLayout, PcbPlacement, PcbRoute,
    PcbRouteSegment, PcbStackup, PcbVia, PhaseTuningGroup, PlacementAnchor, PlacementConstraint,
    PlacementConstraintKind, PlacementResolutionIssue, PlacementResolutionReport, Plating,
    ResolvedNetClass, RouteConstraintRegion, RouteDirection, RouteRuleRegion, RoutingNetAliases,
    StackupLayer, StackupLayerKind, ViaMaskDisposition, ViaMaskIntent, ViaStyle, ViaStyleSpan,
};
#[cfg(feature = "layout")]
pub use layout_module::{
    ComposedLayoutModule, LayoutAssembly, LayoutCompositionError, LayoutCompositionReport,
    LayoutModule, LayoutModuleInstance, LayoutModuleValidationIssue, LayoutTransform,
    PlacementGroup,
};
#[cfg(feature = "lceda")]
pub use lceda::{
    LCEDA_PRO_EXPORT_SCHEMA, LCEDA_PRO_EXPORT_VERSION, LcedaExportError, LcedaExportOmission,
    LcedaImportError, LcedaImportOmission, LcedaNumericImport, LcedaNumericProjection,
    LcedaProExportOptions, LcedaProExportReport, LcedaProImportReport, LcedaSchematicImportReport,
    LcedaSourceLengthUnit,
};
#[cfg(feature = "geometry")]
pub use legacy_csgrs::{
    LEGACY_CSGRS_ELECTRONICS_REMOVAL_VERSION, LEGACY_CSGRS_ELECTRONICS_SCHEMA,
    LEGACY_CSGRS_ELECTRONICS_VERSION, LegacyCsgrsElectronicsImport,
    LegacyCsgrsElectronicsImportError, LegacyCsgrsElectronicsOmission, LegacyCsgrsTerminalClaim,
};
#[cfg(all(feature = "drc", feature = "interchange"))]
pub use manufacturing_release::{
    MANUFACTURING_RELEASE_MANIFEST_PATH, MANUFACTURING_RELEASE_SCHEMA,
    MANUFACTURING_RELEASE_VERSION, ManufacturingReleaseBundle, ManufacturingReleaseCore,
    ManufacturingReleaseDifference, ManufacturingReleaseError, ManufacturingReleaseManifest,
    ManufacturingReleaseOptions, OptionalEvidenceStatus, SignatureEnvelope,
};
#[cfg(feature = "geometry")]
pub use materialize::{
    CopperFeatureKind, DrillHit, GeometryMaterializationError, LayerImage, MaterializationOptions,
    MaterializationProjection, MaterializedCopperFeature, MaterializedCopperIdentity,
    MaterializedProcessFeature, Pcb3dAssemblyOmission, Pcb3dAssemblyReport, Pcb3dComponentBody,
    Pcb3dComponentBodyMetadata, Pcb3dComponentModel, Pcb3dComponentModelMetadata,
    Pcb3dCoordinateEncoding, Pcb3dGltfError, Pcb3dGltfReport, Pcb3dLayer, Pcb3dLayerKind,
    Pcb3dLayerMetadata, Pcb3dModelResolutionEvidence, Pcb3dModelResolver, Pcb3dSceneObject,
    Pcb3dSceneObjectKind, Pcb3dSubtractionEvidence, Pcb3dSubtractionKind, PcbMaterializationReport,
    ProcessFeatureKind, ProcessLayerImage, ProcessLayerRole, ProcessMaterializationOmission,
    ProductionTextEvidence, ProductionTextPolicy, ZoneMaterializationEvidence,
};
pub use mna::{LinearMnaSystem, LinearSolveReport, LinearStamp, MnaUnknown, ResidualReplayReport};
pub use model::{
    Bus, BusSlice, BusSliceOrder, Circuit, CircuitCertificationReport, CircuitInstance,
    CircuitModuleParameter, CircuitModuleParameterOverride, CircuitModuleParameterTarget,
    CircuitParameter, CircuitPort, CircuitPortType, CircuitState, CircuitValidationIssue,
    CircuitValidationReport, DeviceModel, DeviceModelKind, DevicePin, MnaProblem, MosfetPolarity,
    Net, PinBinding, PinElectricalKind, PortDirection, PortSignalType, RailIntent, RailKind,
    SourceStimulus, SourceWaveform, SourceWaveformPoint, TransientPolicy,
};
pub use mosfet::{
    MosfetLinearizationEvidence, MosfetNewtonIteration, MosfetNewtonPolicy, MosfetNewtonSolveError,
    MosfetNewtonSolveReport, MosfetNewtonStatus, MosfetOperatingPoint, MosfetRegion,
    MosfetResidualReplayReport, SquareLawMosfet, solve_square_law_mosfet_newton,
};
pub use nonlinear::{
    DiodeLinearizationEvidence, DiodeNewtonIteration, DiodeNewtonPolicy, DiodeNewtonSolveError,
    DiodeNewtonSolveReport, DiodeNewtonStatus, DiodeResidualReplayReport, EventPolicy,
    NonlinearDeviceKind, NonlinearDeviceReport, PiecewiseLinearDevice, PiecewiseLinearSegment,
    PiecewiseLinearSolveError, PiecewiseLinearSolveReport, ShockleyDiode, SwitchState,
    solve_piecewise_linear, solve_shockley_diode_newton,
};
#[cfg(feature = "interchange")]
pub use package::{
    CIRCUIT_LIBRARY_ARTIFACT_SCHEMA, CIRCUIT_LIBRARY_ARTIFACT_VERSION, CircuitLibraryArtifact,
    CircuitPackageStore, PART_LIBRARY_ARTIFACT_SCHEMA, PART_LIBRARY_ARTIFACT_VERSION,
    PartLibraryArtifact, PortablePartDefinition, PublishedCircuitLibrary, PublishedPartLibrary,
};
pub use package::{
    CIRCUIT_PACKAGE_LOCK_SCHEMA, CIRCUIT_PACKAGE_LOCK_VERSION, CircuitPackageCatalog,
    CircuitPackageExport, CircuitPackageExportKind, CircuitPackageLock, CircuitPackageRelease,
    LockedCircuitPackage, PackageDigest, PackageRequirement, PackageResolutionError, PackageSource,
};
#[cfg(feature = "layout")]
pub use panel::{
    PanelBoardInstance, PanelCoupon, PanelDefinition, PanelEdgeRequirement, PanelFiducial,
    PanelIssue, PanelMarkingRegion, PanelRail, PanelSeparationFeature, PanelTestAccess,
    PanelToolingHole,
};
#[cfg(feature = "layout")]
pub use placement::{
    PlacementCandidateScore, PlacementEnvelopeSource, PlacementMove, PlacementPinAccessDirection,
    PlacementPinAccessIssue, PlacementPinAccessPolicy, PlacementPinAccessProbeEvidence,
    PlacementPinAccessReport, PlacementPinAccessScore, PlacementPinAccessStatus,
    PlacementPinAccessTerminalEvidence, PlacementSolveIssue, PlacementSolvePolicy,
    PlacementSolveReport,
};
#[cfg(feature = "layout")]
pub use preview::{
    NegotiatedRouteHtmlOptions, NegotiatedRouteHtmlReport, NegotiatedRouteSvgError,
    NegotiatedRouteSvgOptions, NegotiatedRouteSvgReport, PcbSvgError, PcbSvgOptions,
    PcbSvgProjection, PcbSvgReport,
};
#[cfg(feature = "layout")]
pub use project::{CheckedProject, DesignModule, DesignModuleInstance, ModuleBuildError};
#[cfg(feature = "interchange")]
pub use project_manifest::{
    HYPERCIRCUIT_PROJECT_SCHEMA, HYPERCIRCUIT_PROJECT_VERSION, ProjectDesignProvider,
    ProjectManifest, ProjectManifestError, ProjectMetadata, ProjectPcbMaterial,
    ProjectProviderKind,
};
#[cfg(feature = "interchange")]
pub use release_artifact::{
    ArtifactCatalog, ArtifactCatalogError, PortableArtifactPath, ReleaseArtifactDescriptor,
    ReleaseArtifactRole,
};
#[cfg(feature = "layout")]
pub use route_constraints::{
    LengthTuningIssue, LengthTuningReport, LengthTuningStatus, PhaseTuningIssue,
    PhaseTuningObstacle, PhaseTuningRealizedZoneEvidence, PhaseTuningRealizedZoneStatus,
    PhaseTuningReport, PhaseTuningStatus, PhaseTuningSynthesisIssue, PhaseTuningSynthesisPolicy,
    PhaseTuningSynthesisReport, PhaseTuningSynthesisStatus, PhaseTuningZoneCollisionMode,
};
#[cfg(feature = "layout")]
pub use routing::{
    RoutingAdapterError, RoutingProblem, RoutingProblemOmission, RoutingProblemReport,
    RoutingQualityIssue, RoutingQualityNetEvidence, RoutingQualityReport, RoutingQualityStatus,
    RoutingSolution, RoutingSolutionOmission, RoutingTerminal,
};
pub use schematic::{
    SchematicBlockPlacement, SchematicBlockSize, SchematicBookSvgReport, SchematicCanvasSettings,
    SchematicConnectionStyle, SchematicEndpoint, SchematicGraphic, SchematicGraphicFill,
    SchematicLabel, SchematicLayout, SchematicPaperOrientation, SchematicPinPlacement,
    SchematicPinSide, SchematicPoint, SchematicPortPlacement, SchematicPresentation,
    SchematicPresentationIssue, SchematicSheet, SchematicSheetLink, SchematicSheetPort,
    SchematicSheetSvgReport, SchematicSvgError, SchematicSvgOptions, SchematicSvgProjection,
    SchematicSvgReport, SchematicSymbol, SchematicSymbolDefinition, SchematicSymbolUnit,
    SchematicValidationIssue, SchematicValidationReport, SchematicWire, SchematicWireMetadata,
    SchematicWireStyle,
};
pub use schematic_auto::{
    SchematicAutoLayoutError, SchematicAutoLayoutPolicy, SchematicAutoLayoutReport,
    SchematicAutoPlacementEvidence,
};
pub use simulation::{
    CircuitEventApplication, DeviceLoweringError, DeviceLoweringIssue, DiodeTransientRunError,
    DiodeTransientRunReport, DiodeTransientStepError, DiodeTransientStepEvidence,
    DiodeTransientStepReport, LinearDeviceLoweringReport, ReactiveState,
    SourceWaveformEvaluationError, TransientAdaptation, TransientHistory, TransientRunError,
    TransientRunPolicy, TransientRunReport, TransientRunStatus, TransientSample, TransientSession,
    TransientSessionAuditReport, TransientSessionError, TransientSessionStatus,
    TransientSessionStep, TransientStepDecision, TransientStepDecisionKind, TransientStepError,
    TransientStepReport,
};
#[cfg(feature = "layout")]
pub use stitching::{ZoneStitchingEvidence, ZoneStitchingRejectionCounts, ZoneStitchingReport};
#[cfg(feature = "layout")]
pub use test_intent::{
    BoundaryScanChain, DesignForTestIntent, FixtureConstraint, PowerDomainTestIntent,
    ProgrammingTarget, TestAccess, TestCoverageMethod, TestDeviceTarget, TestIntentIssue,
    TestRequirement, TestTarget,
};
#[cfg(feature = "layout")]
pub use tscircuit_routing::{
    TscircuitRoutingError, TscircuitRoutingExportOptions, TscircuitRoutingImportOptions,
    TscircuitRoutingImportReport, TscircuitRoutingNumericImport, TscircuitRoutingOmission,
    TscircuitRoutingProjection, TscircuitSimpleRouteJsonReport,
};
#[cfg(feature = "drc")]
pub use workflow::{
    ReleaseBlocker, ReleasePreparationError, ReleasePreparationOptions, ReleasePreparationReport,
};
