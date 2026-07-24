//! Cohesive release preparation from one checked declarative design.
//!
//! This module composes the authoritative retained containers and their existing
//! adapters. It deliberately introduces no alternate circuit or PCB IR.

use std::fmt::{Display, Formatter};

use hyperdrc::Severity;

#[cfg(feature = "interchange")]
use crate::SemanticDocument;
use crate::{
    AssemblyOutputs, AssemblyRoundTripReport, CheckedDesign, CheckedProject, Circuit,
    DrcReadinessPolicy, ErcReport, FabricationCamRoundTripReport, FabricationExportOptions,
    FabricationIntegrityIssue, FabricationPackage, FabricationPackageError,
    GeometryMaterializationError, HyperDrcHandoff, HyperDrcReadinessReport, MaterializationOptions,
    PcbLayout, PcbMaterialPropertyLibrary, PcbMaterializationReport, PlacementResolutionReport,
};

/// Policies used to turn one checked design into review and release evidence.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct ReleasePreparationOptions {
    /// Geometry projection and production-process defaults.
    pub materialization: MaterializationOptions,
    /// Native HyperDRC readiness thresholds.
    pub drc: DrcReadinessPolicy,
    /// Source-attributed HyperPhysics material data used by PCB electrical checks.
    pub pcb_materials: PcbMaterialPropertyLibrary,
    /// Fabrication source-unit policy.
    pub fabrication: FabricationExportOptions,
}

/// A release gate that remains open after all evidence was produced.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum ReleaseBlocker {
    /// Electrical rule checking reported this many issues.
    ElectricalRules(usize),
    /// Retained intent could not be represented completely in HyperDRC.
    DrcHandoffOmissions(usize),
    /// HyperDRC reported this many release-blocking errors.
    DrcErrors(usize),
    /// Fabrication files or their manifest failed integrity checks.
    FabricationIntegrity(usize),
    /// Production geometry still has details requiring review.
    FabricationProductionOmissions(usize),
    /// IPC-D-356 could not represent all retained connectivity.
    FabricationConnectivityOmissions(usize),
    /// Independent CAM re-import reported this many issues.
    CamRoundTrip(usize),
    /// Some fitted circuit instances have no placement output.
    UnplacedInstances(usize),
    /// Independent assembly CSV re-import reported this many issues.
    AssemblyRoundTrip(usize),
}

/// Reviewable evidence produced from the exact retained layout used for release.
#[derive(Debug)]
pub struct ReleasePreparationReport {
    /// Placement solution used by every later stage.
    pub placement: PlacementResolutionReport,
    /// Clone of the authoritative layout with resolved placements applied.
    pub resolved_layout: PcbLayout,
    /// Electrical-rule evidence from authoritative connectivity.
    pub erc: ErcReport,
    /// Source-addressable csgrs geometry projection.
    pub materialization: PcbMaterializationReport,
    /// Typed retained-intent handoff into HyperDRC.
    pub drc_handoff: HyperDrcHandoff,
    /// Native HyperDRC findings.
    pub drc: HyperDrcReadinessReport,
    /// Gerber, Excellon, IPC-D-356, and manifest package.
    pub fabrication: FabricationPackage,
    /// Package byte and manifest integrity findings.
    pub fabrication_integrity: Vec<FabricationIntegrityIssue>,
    /// Independent CAM parse and semantic-reconciliation evidence.
    pub cam_round_trip: FabricationCamRoundTripReport,
    /// BOM, pick-and-place, and DNP outputs.
    pub assembly: AssemblyOutputs,
    /// Independent assembly CSV reconciliation evidence.
    pub assembly_round_trip: AssemblyRoundTripReport,
}

impl ReleasePreparationReport {
    /// Returns every remaining release gate in deterministic workflow order.
    pub fn release_blockers(&self) -> Vec<ReleaseBlocker> {
        let mut blockers = Vec::new();
        if !self.erc.issues.is_empty() {
            blockers.push(ReleaseBlocker::ElectricalRules(self.erc.issues.len()));
        }
        if !self.drc_handoff.omissions.is_empty() {
            blockers.push(ReleaseBlocker::DrcHandoffOmissions(
                self.drc_handoff.omissions.len(),
            ));
        }
        let drc_errors = self
            .drc
            .violations
            .iter()
            .filter(|violation| violation.severity == Severity::Error)
            .count();
        if drc_errors != 0 {
            blockers.push(ReleaseBlocker::DrcErrors(drc_errors));
        }
        if !self.fabrication_integrity.is_empty() {
            blockers.push(ReleaseBlocker::FabricationIntegrity(
                self.fabrication_integrity.len(),
            ));
        }
        if !self.fabrication.production_omissions.is_empty() {
            blockers.push(ReleaseBlocker::FabricationProductionOmissions(
                self.fabrication.production_omissions.len(),
            ));
        }
        if !self.fabrication.manifest.connectivity_omissions.is_empty() {
            blockers.push(ReleaseBlocker::FabricationConnectivityOmissions(
                self.fabrication.manifest.connectivity_omissions.len(),
            ));
        }
        if !self.cam_round_trip.issues.is_empty() {
            blockers.push(ReleaseBlocker::CamRoundTrip(
                self.cam_round_trip.issues.len(),
            ));
        }
        if !self.assembly.unplaced_instances.is_empty() {
            blockers.push(ReleaseBlocker::UnplacedInstances(
                self.assembly.unplaced_instances.len(),
            ));
        }
        if !self.assembly_round_trip.issues.is_empty() {
            blockers.push(ReleaseBlocker::AssemblyRoundTrip(
                self.assembly_round_trip.issues.len(),
            ));
        }
        blockers
    }

    /// True only when every electrical, geometry, DRC, CAM, and assembly gate is clean.
    pub fn is_release_clean(&self) -> bool {
        self.release_blockers().is_empty()
    }
}

/// Failure that prevents complete release evidence from being constructed.
#[derive(Clone, Debug, PartialEq)]
pub enum ReleasePreparationError {
    /// The retained semantic document has no PCB intent to release.
    MissingPcb,
    /// Placement constraints cannot establish one intended physical layout.
    UnsatisfiedPlacement(PlacementResolutionReport),
    /// Retained PCB intent could not be lowered into certified geometry.
    Materialization(GeometryMaterializationError),
    /// Certified geometry could not become a non-lossy fabrication package.
    Fabrication(FabricationPackageError),
}

impl Display for ReleasePreparationError {
    fn fmt(&self, formatter: &mut Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::MissingPcb => formatter.write_str("semantic document has no PCB layout"),
            Self::UnsatisfiedPlacement(report) => write!(
                formatter,
                "placement constraints are unsatisfied ({} issues)",
                report.issues.len()
            ),
            Self::Materialization(error) => write!(formatter, "materialization failed: {error}"),
            Self::Fabrication(error) => write!(formatter, "fabrication failed: {error}"),
        }
    }
}

impl std::error::Error for ReleasePreparationError {}

impl From<GeometryMaterializationError> for ReleasePreparationError {
    fn from(error: GeometryMaterializationError) -> Self {
        Self::Materialization(error)
    }
}

impl From<FabricationPackageError> for ReleasePreparationError {
    fn from(error: FabricationPackageError) -> Self {
        Self::Fabrication(error)
    }
}

impl CheckedDesign {
    /// Resolves placement and produces cohesive verification and release evidence.
    ///
    /// Rule findings are returned in [`ReleasePreparationReport`], even when
    /// they block release. Errors are reserved for stages whose evidence could
    /// not be constructed.
    pub fn prepare_release(
        &self,
        options: ReleasePreparationOptions,
    ) -> Result<ReleasePreparationReport, ReleasePreparationError> {
        prepare_release(&self.circuit, &self.layout, options)
    }
}

impl CheckedProject {
    /// Produces release evidence from the validated path-qualified flat view.
    ///
    /// Reusable definitions, scope maps, schematics, and source maps remain
    /// available on this project while the existing composed
    /// [`crate::Circuit`] and [`crate::PcbLayout`] drive release preparation.
    pub fn prepare_release(
        &self,
        options: ReleasePreparationOptions,
    ) -> Result<ReleasePreparationReport, ReleasePreparationError> {
        prepare_release(&self.composed.circuit, &self.composed.layout, options)
    }
}

#[cfg(feature = "interchange")]
impl SemanticDocument {
    /// Produces release evidence directly from this validated retained document.
    ///
    /// This is the project-file entry point used by command-line and editor
    /// workflows. It consumes the same authoritative [`Circuit`] and
    /// [`PcbLayout`] as the checked authoring APIs and creates no alternate IR.
    pub fn prepare_release(
        &self,
        options: ReleasePreparationOptions,
    ) -> Result<ReleasePreparationReport, ReleasePreparationError> {
        let layout = self
            .pcb
            .as_ref()
            .ok_or(ReleasePreparationError::MissingPcb)?;
        prepare_release(&self.circuit, layout, options)
    }
}

fn prepare_release(
    circuit: &Circuit,
    layout: &PcbLayout,
    options: ReleasePreparationOptions,
) -> Result<ReleasePreparationReport, ReleasePreparationError> {
    let placement = layout.resolve_placement_constraints(circuit);
    if !placement.is_satisfied() {
        return Err(ReleasePreparationError::UnsatisfiedPlacement(placement));
    }

    let mut resolved_layout = layout.clone();
    resolved_layout.placements.clone_from(&placement.placements);
    let erc = circuit.electrical_rule_check();
    let materialization = resolved_layout.materialize(circuit, options.materialization)?;
    let drc_handoff = HyperDrcHandoff::from_materialization_with_materials(
        &resolved_layout,
        &materialization,
        &options.pcb_materials,
    );
    let drc = drc_handoff.run_readiness(&options.drc);
    let fabrication = FabricationPackage::from_materialization_with_options(
        &resolved_layout,
        &materialization,
        options.fabrication,
    )?;
    let fabrication_integrity = fabrication.verify_integrity();
    let cam_round_trip = fabrication.audit_cam_round_trip();
    let assembly = AssemblyOutputs::from_design(circuit, &resolved_layout)
        .expect("checked circuit and resolved release layout remain structurally valid");
    let assembly_round_trip = assembly.audit_csv_round_trip();

    Ok(ReleasePreparationReport {
        placement,
        resolved_layout,
        erc,
        materialization,
        drc_handoff,
        drc,
        fabrication,
        fabrication_integrity,
        cam_round_trip,
        assembly,
        assembly_round_trip,
    })
}
