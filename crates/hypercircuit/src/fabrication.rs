//! Gerber X2, Excellon, and IPC-D-356 package generation from certified
//! materialized geometry and retained semantic identities.

use std::collections::BTreeMap;
use std::fmt::{Display, Formatter, Write};
#[cfg(feature = "drc")]
use std::path::Path;

#[cfg(feature = "drc")]
use csgrs::io::gerber::FromGerber;
use csgrs::{csg::CSG, io::gerber::ToGerber, sketch::Profile};
use hyperreal::Real;
use sha2::{Digest, Sha256};

use crate::{
    DrillShape, MaterializationProjection, MaterializedCopperIdentity, PcbLayout,
    PcbMaterializationReport, Plating, ProcessLayerRole, ProcessMaterializationOmission,
    ProductionTextEvidence, StackupLayerKind,
};

/// Manufacturing-file family emitted by [`FabricationPackage`].
#[derive(Clone, Copy, Debug, Eq, PartialEq, serde::Deserialize, serde::Serialize)]
#[serde(rename_all = "kebab-case")]
pub enum FabricationFileKind {
    /// RS-274X image with X2 file attributes.
    GerberX2,
    /// Excellon drill program.
    Excellon,
    /// IPC-D-356 bare-board electrical-test netlist.
    Ipc356,
    /// Deterministic JSON inventory and integrity evidence for the other files.
    Manifest,
}

/// One deterministic manufacturing file.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct FabricationFile {
    /// Suggested archive filename.
    pub name: String,
    /// File format family.
    pub kind: FabricationFileKind,
    /// Complete file bytes.
    pub bytes: Vec<u8>,
}

/// Stable schema name written into every fabrication manifest.
pub const FABRICATION_MANIFEST_SCHEMA: &str = "hypercircuit.fabrication-manifest";
/// Current fabrication-manifest schema version.
pub const FABRICATION_MANIFEST_VERSION: u32 = 9;

/// Unit in which retained board geometry was authored before millimetre CAM output.
#[derive(Clone, Copy, Debug, Eq, PartialEq, serde::Deserialize, serde::Serialize)]
#[serde(rename_all = "kebab-case")]
pub enum FabricationLengthUnit {
    /// Millimetres.
    Millimeter,
    /// Thousandths of an inch.
    Mil,
    /// Inches.
    Inch,
}

impl FabricationLengthUnit {
    fn millimeter_factor(self) -> Real {
        match self {
            Self::Millimeter => Real::one(),
            Self::Mil => {
                (Real::from(127) / Real::from(5000)).expect("5000 is a nonzero exact denominator")
            }
            Self::Inch => {
                (Real::from(127) / Real::from(5)).expect("5 is a nonzero exact denominator")
            }
        }
    }
}

/// Explicit policy for lowering exact board-contour curves into CAM primitives.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub enum FabricationContourProjectionPolicy {
    /// Reject cubic contours rather than introducing an implicit approximation.
    #[default]
    RejectCubicBezier,
    /// Adaptively emit line segments whose requested maximum flatness is bounded.
    CubicBezierPolyline {
        /// Maximum chord/flatness error in the retained source length unit.
        chord_error: f64,
    },
}

/// Physical-unit and explicit geometry-projection policy for CAM generation.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FabricationExportOptions {
    /// Unit used by retained layout coordinates and dimensions.
    pub source_length_unit: FabricationLengthUnit,
    /// Caller-selected board-contour projection boundary.
    pub contour_projection: FabricationContourProjectionPolicy,
}

impl FabricationExportOptions {
    /// Explicit millimetre-authored design policy.
    pub const fn millimeters() -> Self {
        Self {
            source_length_unit: FabricationLengthUnit::Millimeter,
            contour_projection: FabricationContourProjectionPolicy::RejectCubicBezier,
        }
    }

    /// Enables bounded cubic-Bezier profile projection in the retained source unit.
    pub fn with_cubic_contour_chord_error(mut self, chord_error: f64) -> Self {
        self.contour_projection =
            FabricationContourProjectionPolicy::CubicBezierPolyline { chord_error };
        self
    }
}

impl Default for FabricationExportOptions {
    fn default() -> Self {
        Self::millimeters()
    }
}

/// Integrity record for one non-manifest fabrication file.
#[derive(Clone, Debug, Eq, PartialEq, serde::Deserialize, serde::Serialize)]
pub struct FabricationManifestFile {
    /// Archive-relative deterministic filename.
    pub name: String,
    /// Manufacturing format/role.
    pub kind: FabricationFileKind,
    /// Exact byte length of the emitted artifact.
    pub byte_len: u64,
    /// Lowercase SHA-256 of the complete artifact bytes.
    pub sha256: String,
}

/// Electrical-test feature class retained in the fabrication manifest.
#[derive(Clone, Copy, Debug, Eq, PartialEq, serde::Deserialize, serde::Serialize)]
#[serde(rename_all = "kebab-case")]
pub enum FabricationTestPointKind {
    /// Component land-pattern pad.
    Pad,
    /// Plated or non-plated via land.
    Via,
}

/// Expected IPC-D-356 record derived from a typed semantic copper owner.
#[derive(Clone, Debug, Eq, PartialEq, serde::Deserialize, serde::Serialize)]
pub struct FabricationTestPoint {
    /// Stable hypercircuit source identity.
    pub source: String,
    /// Logical net identity.
    pub net: String,
    /// Component or synthetic via reference field.
    pub reference: String,
    /// Logical pin or via identity.
    pub pin: String,
    /// Projected millimetre X coordinate encoded in the sidecar.
    pub x_mm: String,
    /// Projected millimetre Y coordinate encoded in the sidecar.
    pub y_mm: String,
    /// Electrical-test feature class.
    pub kind: FabricationTestPointKind,
}

/// Versioned deterministic inventory for one emitted board package.
#[derive(Clone, Debug, Eq, PartialEq, serde::Deserialize, serde::Serialize)]
pub struct FabricationManifest {
    /// Stable schema identity.
    pub schema: String,
    /// Schema revision.
    pub version: u32,
    /// Stable board identity used for output filenames.
    pub board: String,
    /// Generator crate and version.
    pub generated_by: String,
    /// Unit declared for retained source geometry.
    pub source_length_unit: FabricationLengthUnit,
    /// CAM output unit; currently always millimetres.
    pub output_length_unit: FabricationLengthUnit,
    /// Non-manifest files in package order.
    pub files: Vec<FabricationManifestFile>,
    /// Source-addressable copper features represented in the package.
    pub represented_copper_features: usize,
    /// Source-addressable mask, paste, and legend features represented.
    pub represented_process_features: usize,
    /// Drill hits represented in the package.
    pub represented_drills: usize,
    /// Connected pad/via test points represented by IPC-D-356.
    #[serde(default)]
    pub represented_test_points: usize,
    /// Expected source-addressable IPC-D-356 connectivity records.
    #[serde(default)]
    pub test_points: Vec<FabricationTestPoint>,
    /// Pad sources omitted because they carry no logical net or pin.
    #[serde(default)]
    pub connectivity_omissions: Vec<String>,
    /// Known production details that still require review.
    pub production_omissions: Vec<ProcessMaterializationOmission>,
    /// Exact font-byte identity used for production text, when selected.
    pub production_text: Option<ProductionTextEvidence>,
    /// Audited finite geometry projections used before CAM serialization.
    pub geometry_projections: Vec<MaterializationProjection>,
    /// Audited copper-zone fill, clearance, connection, and keepout realizations.
    pub zone_realizations: Vec<crate::ZoneMaterializationEvidence>,
    /// Deterministic generated stitching-via realization evidence.
    pub stitching_realizations: Vec<crate::ZoneStitchingEvidence>,
}

/// Failure while decoding or schema-checking an external fabrication manifest.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum FabricationManifestError {
    /// JSON syntax or data shape is invalid.
    Json(String),
    /// The document belongs to another manifest family.
    UnsupportedSchema(String),
    /// The manifest revision is newer or otherwise unsupported.
    UnsupportedVersion(u32),
}

impl Display for FabricationManifestError {
    fn fmt(&self, formatter: &mut Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Json(error) => write!(formatter, "invalid fabrication manifest JSON: {error}"),
            Self::UnsupportedSchema(schema) => {
                write!(
                    formatter,
                    "unsupported fabrication manifest schema: {schema}"
                )
            }
            Self::UnsupportedVersion(version) => {
                write!(
                    formatter,
                    "unsupported fabrication manifest version: {version}"
                )
            }
        }
    }
}

impl std::error::Error for FabricationManifestError {}

impl FabricationManifest {
    /// Decodes a manifest and rejects unknown schema families or revisions.
    pub fn from_json(bytes: &[u8]) -> Result<Self, FabricationManifestError> {
        let manifest = serde_json::from_slice::<Self>(bytes)
            .map_err(|error| FabricationManifestError::Json(error.to_string()))?;
        if manifest.schema != FABRICATION_MANIFEST_SCHEMA {
            return Err(FabricationManifestError::UnsupportedSchema(manifest.schema));
        }
        if !matches!(manifest.version, 7 | 8 | FABRICATION_MANIFEST_VERSION) {
            return Err(FabricationManifestError::UnsupportedVersion(
                manifest.version,
            ));
        }
        let mut manifest = manifest;
        if matches!(manifest.version, 7 | 8) {
            manifest.version = FABRICATION_MANIFEST_VERSION;
        }
        Ok(manifest)
    }

    /// Replays this external inventory against extracted non-manifest files.
    pub fn verify_files(&self, files: &[FabricationFile]) -> Vec<FabricationIntegrityIssue> {
        let mut issues = Vec::new();
        for entry in &self.files {
            let Some(file) = files
                .iter()
                .find(|file| file.kind != FabricationFileKind::Manifest && file.name == entry.name)
            else {
                issues.push(FabricationIntegrityIssue::MissingFile(entry.name.clone()));
                continue;
            };
            if file.kind != entry.kind {
                issues.push(FabricationIntegrityIssue::KindMismatch(entry.name.clone()));
            }
            if file.bytes.len() as u64 != entry.byte_len {
                issues.push(FabricationIntegrityIssue::ByteLengthMismatch(
                    entry.name.clone(),
                ));
            }
            if sha256(&file.bytes) != entry.sha256 {
                issues.push(FabricationIntegrityIssue::DigestMismatch(
                    entry.name.clone(),
                ));
            }
        }
        for file in files
            .iter()
            .filter(|file| file.kind != FabricationFileKind::Manifest)
        {
            if !self.files.iter().any(|entry| entry.name == file.name) {
                issues.push(FabricationIntegrityIssue::UnexpectedFile(file.name.clone()));
            }
        }
        issues
    }
}

/// One mismatch found while replaying package integrity evidence.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum FabricationIntegrityIssue {
    /// A manifest entry has no corresponding package file.
    MissingFile(String),
    /// A non-manifest package file is absent from the manifest.
    UnexpectedFile(String),
    /// The retained file kind differs from the manifest role.
    KindMismatch(String),
    /// The exact byte count differs.
    ByteLengthMismatch(String),
    /// Recomputed SHA-256 differs.
    DigestMismatch(String),
    /// The packaged JSON manifest differs from deterministic serialization.
    ManifestBytesMismatch,
}

/// Re-import evidence for one emitted Gerber image.
#[cfg(feature = "drc")]
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct FabricationGerberRoundTripEvidence {
    /// Archive-relative source filename.
    pub file: String,
    /// X2 file-function attribute recovered independently by HyperDRC.
    pub file_function: String,
    /// Number of retained coordinate operations.
    pub coordinate_operations: usize,
    /// Number of aperture definitions.
    pub aperture_definitions: usize,
    /// Whether csgrs independently recovered nonempty geometry.
    pub geometry_nonempty: bool,
}

/// Re-import evidence for one emitted Excellon program.
#[cfg(feature = "drc")]
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct FabricationExcellonRoundTripEvidence {
    /// Archive-relative source filename.
    pub file: String,
    /// Filename-derived plating intent retained by HyperDRC.
    pub plated: bool,
    /// Number of parsed round drill hits.
    pub round_hits: usize,
    /// Number of complete linear routed-slot spans.
    pub routed_slots: usize,
    /// Number of defined tools.
    pub defined_tools: usize,
    /// Non-blocking diagnostics retained for review.
    pub warnings: Vec<String>,
}

/// Independently recovered IPC-D-356 connectivity evidence.
#[cfg(feature = "drc")]
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct FabricationIpc356RoundTripEvidence {
    /// Archive-relative source filename.
    pub file: String,
    /// Parsed electrical-test point count.
    pub points: usize,
    /// Parsed records carrying named nets.
    pub named_net_records: usize,
    /// Distinct recovered net names.
    pub unique_nets: usize,
    /// Parsed records carrying both reference and pin fields.
    pub reference_pin_records: usize,
}

/// A typed failure found while independently parsing an emitted CAM package.
#[cfg(feature = "drc")]
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum FabricationCamRoundTripIssue {
    /// No manifest file was supplied.
    MissingManifest,
    /// More than one manifest file was supplied.
    DuplicateManifest,
    /// Manifest JSON or schema validation failed.
    Manifest(String),
    /// Byte-level inventory replay failed.
    Integrity(FabricationIntegrityIssue),
    /// Gerber metadata contained a parser diagnostic.
    GerberMetadata {
        /// Source filename.
        file: String,
        /// One-based source line.
        line: usize,
        /// Typed HyperDRC diagnostic rendered for retained review.
        detail: String,
    },
    /// Required Gerber image setup or X2 identity is missing or inconsistent.
    GerberSetup {
        /// Source filename.
        file: String,
        /// Setup failure.
        detail: String,
    },
    /// csgrs could not independently parse nonempty Gerber geometry.
    GerberGeometry {
        /// Source filename.
        file: String,
        /// Geometry parse failure.
        detail: String,
    },
    /// Excellon bytes were not valid UTF-8.
    ExcellonText(String),
    /// Excellon structure, units, coordinate format, or parser evidence failed.
    Excellon {
        /// Source filename.
        file: String,
        /// Parse/readiness failure.
        detail: String,
    },
    /// IPC-D-356 syntax or semantic reconciliation failed.
    Ipc356 {
        /// Source filename.
        file: String,
        /// Connectivity failure.
        detail: String,
    },
    /// Parsed round-hit plus routed-slot count differs from the manifest.
    DrillCount {
        /// Manifest-declared drill count.
        expected: usize,
        /// Independently recovered count.
        parsed: usize,
    },
    /// Parsed IPC-D-356 point count differs from the manifest.
    TestPointCount {
        /// Manifest-declared point count.
        expected: usize,
        /// Independently recovered point count.
        parsed: usize,
    },
}

/// Independent geometry/metadata re-import audit for a fabrication package.
#[cfg(feature = "drc")]
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct FabricationCamRoundTripReport {
    /// Validated manifest, when it could be decoded.
    pub manifest: Option<FabricationManifest>,
    /// Per-Gerber parse evidence.
    pub gerbers: Vec<FabricationGerberRoundTripEvidence>,
    /// Per-Excellon parse evidence.
    pub excellons: Vec<FabricationExcellonRoundTripEvidence>,
    /// IPC-D-356 connectivity parse evidence.
    pub ipc356: Option<FabricationIpc356RoundTripEvidence>,
    /// Release-blocking re-import findings.
    pub issues: Vec<FabricationCamRoundTripIssue>,
}

#[cfg(feature = "drc")]
impl FabricationCamRoundTripReport {
    /// Re-imports extracted package files through csgrs and HyperDRC.
    pub fn from_files(files: &[FabricationFile]) -> Self {
        let mut report = Self {
            manifest: None,
            gerbers: Vec::new(),
            excellons: Vec::new(),
            ipc356: None,
            issues: Vec::new(),
        };
        let manifests = files
            .iter()
            .filter(|file| file.kind == FabricationFileKind::Manifest)
            .collect::<Vec<_>>();
        let manifest_file = match manifests.as_slice() {
            [] => {
                report
                    .issues
                    .push(FabricationCamRoundTripIssue::MissingManifest);
                return report;
            }
            [manifest] => *manifest,
            [manifest, ..] => {
                report
                    .issues
                    .push(FabricationCamRoundTripIssue::DuplicateManifest);
                *manifest
            }
        };
        let manifest = match FabricationManifest::from_json(&manifest_file.bytes) {
            Ok(manifest) => manifest,
            Err(error) => {
                report
                    .issues
                    .push(FabricationCamRoundTripIssue::Manifest(error.to_string()));
                return report;
            }
        };
        report.issues.extend(
            manifest
                .verify_files(files)
                .into_iter()
                .map(FabricationCamRoundTripIssue::Integrity),
        );
        if serialize_manifest(&manifest).ok().as_deref() != Some(manifest_file.bytes.as_slice()) {
            report.issues.push(FabricationCamRoundTripIssue::Integrity(
                FabricationIntegrityIssue::ManifestBytesMismatch,
            ));
        }

        let mut parsed_drills = 0;
        for entry in &manifest.files {
            let Some(file) = files.iter().find(|file| file.name == entry.name) else {
                continue;
            };
            match entry.kind {
                FabricationFileKind::GerberX2 => audit_gerber(file, &mut report),
                FabricationFileKind::Excellon => {
                    parsed_drills += audit_excellon(file, &mut report);
                }
                FabricationFileKind::Ipc356 => audit_ipc356(file, &manifest, &mut report),
                FabricationFileKind::Manifest => {}
            }
        }
        if parsed_drills != manifest.represented_drills {
            report
                .issues
                .push(FabricationCamRoundTripIssue::DrillCount {
                    expected: manifest.represented_drills,
                    parsed: parsed_drills,
                });
        }
        let parsed_test_points = report
            .ipc356
            .as_ref()
            .map(|evidence| evidence.points)
            .unwrap_or(0);
        if parsed_test_points != manifest.represented_test_points {
            report
                .issues
                .push(FabricationCamRoundTripIssue::TestPointCount {
                    expected: manifest.represented_test_points,
                    parsed: parsed_test_points,
                });
        }
        report.manifest = Some(manifest);
        report
    }

    /// Whether byte integrity, CAM syntax, setup metadata and drill accounting
    /// all replayed successfully.
    pub fn is_release_clean(&self) -> bool {
        self.manifest.is_some() && self.issues.is_empty()
    }
}

/// Reviewable fabrication package plus source-feature accounting.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct FabricationPackage {
    /// Manufacturing files in deterministic stackup/plating/identity order.
    pub files: Vec<FabricationFile>,
    /// Total source-addressable copper features represented by Gerber images.
    pub represented_copper_features: usize,
    /// Total mask, paste, and legend source features represented by Gerber images.
    pub represented_process_features: usize,
    /// Total round drill hits and routed slots represented by Excellon programs.
    pub represented_drills: usize,
    /// Known production details that still require review.
    pub production_omissions: Vec<ProcessMaterializationOmission>,
    /// Versioned file inventory and content digests.
    pub manifest: FabricationManifest,
}

/// Failure that prevents a non-lossy manufacturing package from being emitted.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum FabricationPackageError {
    /// A layer image has an unresolved exact boolean.
    BlockedCopperLayer { layer: u16, blocker: String },
    /// A mask, paste, or legend image has an unresolved exact boolean.
    BlockedProcessLayer {
        role: ProcessLayerRole,
        blocker: String,
    },
    /// csgrs could not project or serialize a certified layer image.
    Gerber(String),
    /// Drill plating must be explicit before file separation.
    UnspecifiedPlating(String),
    /// Exact coordinate/diameter could not be projected to a finite Excellon token.
    NonFiniteDrill(String),
    /// Connected pad/via identity cannot be represented faithfully in IPC-D-356.
    Connectivity(String),
    /// Deterministic JSON manifest serialization failed.
    Manifest(String),
}

impl Display for FabricationPackageError {
    fn fmt(&self, formatter: &mut Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::BlockedCopperLayer { layer, .. } => {
                write!(formatter, "copper layer {layer} has an unresolved union")
            }
            Self::BlockedProcessLayer { role, .. } => {
                write!(
                    formatter,
                    "{role:?} has an unresolved production-image union"
                )
            }
            Self::Gerber(error) => write!(formatter, "Gerber serialization failed: {error}"),
            Self::UnspecifiedPlating(source) => {
                write!(formatter, "drill plating is unspecified: {source}")
            }
            Self::NonFiniteDrill(source) => {
                write!(formatter, "drill cannot be finitely projected: {source}")
            }
            Self::Connectivity(source) => {
                write!(
                    formatter,
                    "electrical-test identity cannot be represented: {source}"
                )
            }
            Self::Manifest(error) => write!(formatter, "manifest serialization failed: {error}"),
        }
    }
}

impl std::error::Error for FabricationPackageError {}

impl FabricationPackage {
    /// Emits Gerber, Excellon, and semantic electrical-test files.
    pub fn from_materialization(
        layout: &PcbLayout,
        materialized: &PcbMaterializationReport,
    ) -> Result<Self, FabricationPackageError> {
        Self::from_materialization_with_options(
            layout,
            materialized,
            FabricationExportOptions::millimeters(),
        )
    }

    /// Emits a complete production package under an explicit source-length-unit policy.
    pub fn from_materialization_with_options(
        layout: &PcbLayout,
        materialized: &PcbMaterializationReport,
        options: FabricationExportOptions,
    ) -> Result<Self, FabricationPackageError> {
        let mut files = Vec::new();
        let millimeter_factor = options.source_length_unit.millimeter_factor();
        let conductor_layers = layout
            .stackup
            .layers
            .iter()
            .filter_map(|layer| match layer.kind {
                StackupLayerKind::Conductor(index) => Some((index, layer.name.clone())),
                _ => None,
            })
            .collect::<Vec<_>>();
        for (position, (layer, name)) in conductor_layers.iter().enumerate() {
            let Some(image) = materialized
                .copper_layers
                .iter()
                .find(|image| image.layer == *layer)
            else {
                continue;
            };
            let Some(profile) = &image.copper else {
                return Err(FabricationPackageError::BlockedCopperLayer {
                    layer: layer.0,
                    blocker: image.blocker.clone().unwrap_or_else(|| "unknown".into()),
                });
            };
            let profile = scale_profile(profile, &millimeter_factor);
            let gerber = profile
                .to_gerber()
                .map_err(|error| FabricationPackageError::Gerber(format!("{error:?}")))?;
            let function = if position == 0 {
                format!("Copper,L{},Top", position + 1)
            } else if position + 1 == conductor_layers.len() {
                format!("Copper,L{},Bot", position + 1)
            } else {
                format!("Copper,L{},Inr", position + 1)
            };
            files.push(FabricationFile {
                name: format!("{}-{}.gbr", sanitize(layout.id.as_str()), sanitize(name)),
                kind: FabricationFileKind::GerberX2,
                bytes: add_x2_attributes(gerber, &function),
            });
        }

        for image in &materialized.process_layers {
            let Some(profile) = &image.image else {
                return Err(FabricationPackageError::BlockedProcessLayer {
                    role: image.role,
                    blocker: image.blocker.clone().unwrap_or_else(|| "unknown".into()),
                });
            };
            let profile = scale_profile(profile, &millimeter_factor);
            let gerber = profile
                .to_gerber()
                .map_err(|error| FabricationPackageError::Gerber(format!("{error:?}")))?;
            let (suffix, function) = process_file_role(image.role);
            files.push(FabricationFile {
                name: format!("{}-{suffix}.gbr", sanitize(layout.id.as_str())),
                kind: FabricationFileKind::GerberX2,
                bytes: add_x2_attributes(gerber, function),
            });
        }

        let (profile, contour_projections) =
            profile_gerber(layout, &millimeter_factor, options.contour_projection)?;
        files.push(FabricationFile {
            name: format!("{}-Profile.gbr", sanitize(layout.id.as_str())),
            kind: FabricationFileKind::GerberX2,
            bytes: profile,
        });

        let plated = materialized
            .drills
            .iter()
            .filter(|drill| drill.plating == Plating::Plated)
            .collect::<Vec<_>>();
        let non_plated = materialized
            .drills
            .iter()
            .filter(|drill| drill.plating == Plating::NonPlated)
            .collect::<Vec<_>>();
        if let Some(drill) = materialized
            .drills
            .iter()
            .find(|drill| drill.plating == Plating::Unspecified)
        {
            return Err(FabricationPackageError::UnspecifiedPlating(
                drill.source.clone(),
            ));
        }
        if !plated.is_empty() {
            files.push(FabricationFile {
                name: format!("{}-PTH.drl", sanitize(layout.id.as_str())),
                kind: FabricationFileKind::Excellon,
                bytes: excellon(&plated, true, &millimeter_factor)?.into_bytes(),
            });
        }
        if !non_plated.is_empty() {
            files.push(FabricationFile {
                name: format!("{}-NPTH.drl", sanitize(layout.id.as_str())),
                kind: FabricationFileKind::Excellon,
                bytes: excellon(&non_plated, false, &millimeter_factor)?.into_bytes(),
            });
        }
        let (test_points, connectivity_omissions) =
            fabrication_test_points(materialized, &millimeter_factor)?;
        if !test_points.is_empty() {
            files.push(FabricationFile {
                name: format!("{}-netlist.ipc", sanitize(layout.id.as_str())),
                kind: FabricationFileKind::Ipc356,
                bytes: ipc356(&test_points).into_bytes(),
            });
        }
        let represented_copper_features = materialized.copper_features.len();
        let represented_process_features = materialized.process_features.len();
        let represented_drills = materialized.drills.len();
        let production_omissions = materialized.process_omissions.clone();
        let manifest = FabricationManifest {
            schema: FABRICATION_MANIFEST_SCHEMA.into(),
            version: FABRICATION_MANIFEST_VERSION,
            board: layout.id.as_str().into(),
            generated_by: format!("hypercircuit/{}", env!("CARGO_PKG_VERSION")),
            source_length_unit: options.source_length_unit,
            output_length_unit: FabricationLengthUnit::Millimeter,
            files: files.iter().map(manifest_file).collect(),
            represented_copper_features,
            represented_process_features,
            represented_drills,
            represented_test_points: test_points.len(),
            test_points,
            connectivity_omissions,
            production_omissions: production_omissions.clone(),
            production_text: materialized.production_text.clone(),
            geometry_projections: materialized
                .projections
                .iter()
                .cloned()
                .chain(contour_projections)
                .collect(),
            zone_realizations: materialized.zone_realizations.clone(),
            stitching_realizations: materialized.stitching_realizations.clone(),
        };
        let manifest_bytes = serialize_manifest(&manifest)?;
        files.push(FabricationFile {
            name: format!("{}-manifest.json", sanitize(layout.id.as_str())),
            kind: FabricationFileKind::Manifest,
            bytes: manifest_bytes,
        });
        Ok(Self {
            files,
            represented_copper_features,
            represented_process_features,
            represented_drills,
            production_omissions,
            manifest,
        })
    }

    /// Recomputes byte counts, SHA-256 digests, file roles and manifest bytes.
    pub fn verify_integrity(&self) -> Vec<FabricationIntegrityIssue> {
        let mut issues = self.manifest.verify_files(&self.files);
        let expected = serialize_manifest(&self.manifest).ok();
        let packaged = self
            .files
            .iter()
            .find(|file| file.kind == FabricationFileKind::Manifest)
            .map(|file| file.bytes.as_slice());
        if expected.as_deref() != packaged {
            issues.push(FabricationIntegrityIssue::ManifestBytesMismatch);
        }
        issues
    }

    /// Independently re-imports the emitted package through csgrs and HyperDRC.
    #[cfg(feature = "drc")]
    pub fn audit_cam_round_trip(&self) -> FabricationCamRoundTripReport {
        FabricationCamRoundTripReport::from_files(&self.files)
    }
}

#[cfg(feature = "drc")]
fn audit_gerber(file: &FabricationFile, audit: &mut FabricationCamRoundTripReport) {
    let metadata = hyperdrc::gerber_metadata::parse_gerber_metadata_report(&file.bytes);
    for issue in &metadata.issues {
        audit
            .issues
            .push(FabricationCamRoundTripIssue::GerberMetadata {
                file: file.name.clone(),
                line: issue.line,
                detail: format!("{:?}", issue.kind),
            });
    }
    let file_function = match metadata.metadata.file_function.clone() {
        Some(function) => function,
        None => {
            audit
                .issues
                .push(FabricationCamRoundTripIssue::GerberSetup {
                    file: file.name.clone(),
                    detail: "missing TF.FileFunction".into(),
                });
            String::new()
        }
    };
    if metadata.metadata.part.as_deref() != Some("Single") {
        audit
            .issues
            .push(FabricationCamRoundTripIssue::GerberSetup {
                file: file.name.clone(),
                detail: "TF.Part is not Single".into(),
            });
    }
    if metadata.image_setup.units != Some(hyperdrc::gerber_metadata::GerberUnits::Millimeters) {
        audit
            .issues
            .push(FabricationCamRoundTripIssue::GerberSetup {
                file: file.name.clone(),
                detail: "Gerber output is not explicitly millimetres".into(),
            });
    }
    if metadata.image_setup.coordinate_format.is_none() {
        audit
            .issues
            .push(FabricationCamRoundTripIssue::GerberSetup {
                file: file.name.clone(),
                detail: "Gerber coordinate format is missing".into(),
            });
    }
    let geometry_nonempty = match Profile::from_gerber(&file.bytes) {
        Ok(profile) if !profile.is_empty() => true,
        Ok(_) => {
            audit
                .issues
                .push(FabricationCamRoundTripIssue::GerberGeometry {
                    file: file.name.clone(),
                    detail: "parsed geometry is empty".into(),
                });
            false
        }
        Err(error) => {
            audit
                .issues
                .push(FabricationCamRoundTripIssue::GerberGeometry {
                    file: file.name.clone(),
                    detail: error.to_string(),
                });
            false
        }
    };
    audit.gerbers.push(FabricationGerberRoundTripEvidence {
        file: file.name.clone(),
        file_function,
        coordinate_operations: metadata.coordinate_operations.len(),
        aperture_definitions: metadata.aperture_definitions.len(),
        geometry_nonempty,
    });
}

#[cfg(feature = "drc")]
fn audit_excellon(file: &FabricationFile, audit: &mut FabricationCamRoundTripReport) -> usize {
    let text = match std::str::from_utf8(&file.bytes) {
        Ok(text) => text,
        Err(error) => {
            audit
                .issues
                .push(FabricationCamRoundTripIssue::ExcellonText(
                    file.name.clone(),
                ));
            audit.issues.push(FabricationCamRoundTripIssue::Excellon {
                file: file.name.clone(),
                detail: error.to_string(),
            });
            return 0;
        }
    };
    let parsed = hyperdrc::excellon::parse_excellon_report(text, Path::new(&file.name));
    let mut warnings = Vec::new();
    for issue in &parsed.issues {
        match &issue.kind {
            hyperdrc::excellon::ExcellonIssueKind::ZeroSuppressionDeclaration { .. }
            | hyperdrc::excellon::ExcellonIssueKind::RoutedSlotCommand { .. } => {
                warnings.push(issue.message());
            }
            _ => audit.issues.push(FabricationCamRoundTripIssue::Excellon {
                file: file.name.clone(),
                detail: issue.message(),
            }),
        }
    }
    if parsed.declared_unit != Some(hyperdrc::excellon::ExcellonUnits::Metric) {
        audit.issues.push(FabricationCamRoundTripIssue::Excellon {
            file: file.name.clone(),
            detail: "output is not explicitly metric".into(),
        });
    }
    if parsed.unit_summary.coordinate_format
        != Some(hyperdrc::excellon::ExcellonCoordinateFormat {
            integer_digits: 4,
            decimal_digits: 6,
        })
    {
        audit.issues.push(FabricationCamRoundTripIssue::Excellon {
            file: file.name.clone(),
            detail: "missing or unexpected FILE_FORMAT; expected 4:6".into(),
        });
    }
    if parsed.program.header_start_line.is_none()
        || parsed.program.header_end_line.is_none()
        || parsed.program.end_of_program_line.is_none()
    {
        audit.issues.push(FabricationCamRoundTripIssue::Excellon {
            file: file.name.clone(),
            detail: "incomplete M48/%/M30 program structure".into(),
        });
    }
    let routed_slots = parsed.routing.linear_moves;
    if parsed.routing.rapid_moves != routed_slots
        || parsed.routing.tool_down_commands != routed_slots
        || parsed.routing.tool_up_commands != routed_slots
    {
        audit.issues.push(FabricationCamRoundTripIssue::Excellon {
            file: file.name.clone(),
            detail: "routed-slot rapid/down/linear/up command counts disagree".into(),
        });
    }
    let plated = hyperdrc::excellon::infer_excellon_plating_intent(Path::new(&file.name))
        == Some(hyperdrc::excellon::ExcellonPlatingIntent::Plated);
    audit.excellons.push(FabricationExcellonRoundTripEvidence {
        file: file.name.clone(),
        plated,
        round_hits: parsed.drill_summary.parsed_drills,
        routed_slots,
        defined_tools: parsed.tool_table.defined_tools,
        warnings,
    });
    parsed.drill_summary.parsed_drills + routed_slots
}

#[cfg(feature = "drc")]
fn audit_ipc356(
    file: &FabricationFile,
    manifest: &FabricationManifest,
    audit: &mut FabricationCamRoundTripReport,
) {
    let text = match std::str::from_utf8(&file.bytes) {
        Ok(text) => text,
        Err(error) => {
            audit.issues.push(FabricationCamRoundTripIssue::Ipc356 {
                file: file.name.clone(),
                detail: error.to_string(),
            });
            return;
        }
    };
    let parsed = hyperdrc::ipc356::parse_ipc356_report(text, Path::new(&file.name));
    for issue in &parsed.issues {
        audit.issues.push(FabricationCamRoundTripIssue::Ipc356 {
            file: file.name.clone(),
            detail: issue.message(),
        });
    }
    if parsed.points.len() == manifest.test_points.len() {
        for (index, (expected, point)) in
            manifest.test_points.iter().zip(&parsed.points).enumerate()
        {
            let expected_x = expected.x_mm.parse::<Real>().ok();
            let expected_y = expected.y_mm.parse::<Real>().ok();
            let expected_kind = match expected.kind {
                FabricationTestPointKind::Pad => Some(hyperdrc::ipc356::Ipc356FeatureType::Smd),
                FabricationTestPointKind::Via => Some(hyperdrc::ipc356::Ipc356FeatureType::Via),
            };
            if point.net != expected.net
                || point.reference.as_deref() != Some(expected.reference.as_str())
                || point.pin.as_deref() != Some(expected.pin.as_str())
                || expected_x.as_ref() != Some(&point.location[0])
                || expected_y.as_ref() != Some(&point.location[1])
                || point.feature_type != expected_kind
            {
                audit.issues.push(FabricationCamRoundTripIssue::Ipc356 {
                    file: file.name.clone(),
                    detail: format!(
                        "record {index} disagrees with manifest test point {}",
                        expected.source
                    ),
                });
            }
        }
    }
    audit.ipc356 = Some(FabricationIpc356RoundTripEvidence {
        file: file.name.clone(),
        points: parsed.points.len(),
        named_net_records: parsed.net_stats.named_records,
        unique_nets: parsed.net_stats.unique_nets,
        reference_pin_records: parsed.component_stats.reference_pin_records,
    });
}

fn manifest_file(file: &FabricationFile) -> FabricationManifestFile {
    FabricationManifestFile {
        name: file.name.clone(),
        kind: file.kind,
        byte_len: file.bytes.len() as u64,
        sha256: sha256(&file.bytes),
    }
}

fn serialize_manifest(manifest: &FabricationManifest) -> Result<Vec<u8>, FabricationPackageError> {
    let mut bytes = serde_json::to_vec_pretty(manifest)
        .map_err(|error| FabricationPackageError::Manifest(error.to_string()))?;
    bytes.push(b'\n');
    Ok(bytes)
}

fn sha256(bytes: &[u8]) -> String {
    format!("{:x}", Sha256::digest(bytes))
}

fn add_x2_attributes(gerber: Vec<u8>, function: &str) -> Vec<u8> {
    let attributes = format!(
        "%TF.FileFunction,{function}*%\n%TF.Part,Single*%\n%TF.GenerationSoftware,Hyper,hypercircuit,0.3.0*%\n"
    );
    let mut output = attributes.into_bytes();
    output.extend(gerber);
    output
}

fn scale_profile(profile: &Profile, factor: &Real) -> Profile {
    profile
        .clone()
        .scale(factor.clone(), factor.clone(), Real::one())
}

fn process_file_role(role: ProcessLayerRole) -> (&'static str, &'static str) {
    match role {
        ProcessLayerRole::FrontSolderMask => ("F_Mask", "Soldermask,Top"),
        ProcessLayerRole::BackSolderMask => ("B_Mask", "Soldermask,Bot"),
        ProcessLayerRole::FrontPaste => ("F_Paste", "Paste,Top"),
        ProcessLayerRole::BackPaste => ("B_Paste", "Paste,Bot"),
        ProcessLayerRole::FrontSilkscreen => ("F_Silkscreen", "Legend,Top"),
        ProcessLayerRole::BackSilkscreen => ("B_Silkscreen", "Legend,Bot"),
    }
}

fn fabrication_test_points(
    materialized: &PcbMaterializationReport,
    millimeter_factor: &Real,
) -> Result<(Vec<FabricationTestPoint>, Vec<String>), FabricationPackageError> {
    let mut points = BTreeMap::<MaterializedCopperIdentity, FabricationTestPoint>::new();
    let mut omissions = std::collections::BTreeSet::new();
    for feature in &materialized.copper_features {
        let (reference, pin, kind) = match &feature.identity {
            MaterializedCopperIdentity::Pad { instance, pin, .. } => {
                let Some(pin) = pin else {
                    omissions.insert(feature.source.clone());
                    continue;
                };
                (
                    instance.as_str(),
                    pin.as_str(),
                    FabricationTestPointKind::Pad,
                )
            }
            MaterializedCopperIdentity::Via(via) => {
                ("VIA", via.as_str(), FabricationTestPointKind::Via)
            }
            MaterializedCopperIdentity::Route(_)
            | MaterializedCopperIdentity::Zone(_)
            | MaterializedCopperIdentity::Artwork { .. } => continue,
        };
        let Some(net) = feature.net.as_ref() else {
            omissions.insert(feature.source.clone());
            continue;
        };
        for (field, value) in [
            ("net", net.as_str()),
            ("reference", reference),
            ("pin", pin),
        ] {
            if value.is_empty()
                || value.chars().any(char::is_whitespace)
                || (field == "net" && value.starts_with('/'))
            {
                return Err(FabricationPackageError::Connectivity(format!(
                    "{} has unsupported {field} token {value:?}",
                    feature.source
                )));
            }
        }
        let x_mm = ipc356_decimal(&(feature.anchor.x.clone() * millimeter_factor.clone()))
            .ok_or_else(|| FabricationPackageError::Connectivity(feature.source.clone()))?;
        let y_mm = ipc356_decimal(&(feature.anchor.y.clone() * millimeter_factor.clone()))
            .ok_or_else(|| FabricationPackageError::Connectivity(feature.source.clone()))?;
        let point = FabricationTestPoint {
            source: feature.source.clone(),
            net: net.as_str().into(),
            reference: reference.into(),
            pin: pin.into(),
            x_mm,
            y_mm,
            kind,
        };
        if let Some(previous) = points.insert(feature.identity.clone(), point.clone())
            && previous != point
        {
            return Err(FabricationPackageError::Connectivity(format!(
                "{} has inconsistent multilayer test-point facts",
                feature.source
            )));
        }
    }
    Ok((
        points.into_values().collect(),
        omissions.into_iter().collect(),
    ))
}

fn ipc356(points: &[FabricationTestPoint]) -> String {
    let mut output =
        String::from("C IPC-D-356B electrical-test netlist generated by hypercircuit\n");
    for point in points {
        writeln!(
            output,
            "327 /{} {} {} X{}Y{} FEATURE={}",
            point.net,
            point.reference,
            point.pin,
            point.x_mm,
            point.y_mm,
            match point.kind {
                FabricationTestPointKind::Pad => "SMD",
                FabricationTestPointKind::Via => "VIA",
            }
        )
        .expect("writing to String cannot fail");
    }
    output.push_str("999\n");
    output
}

fn ipc356_decimal(value: &Real) -> Option<String> {
    Some(format!("{:.6}", finite(value)?))
}

fn profile_gerber(
    layout: &PcbLayout,
    millimeter_factor: &Real,
    projection: FabricationContourProjectionPolicy,
) -> Result<(Vec<u8>, Vec<MaterializationProjection>), FabricationPackageError> {
    let mut output = String::from(
        "%TF.FileFunction,Profile,NP*%\n%TF.Part,Single*%\n%TF.GenerationSoftware,Hyper,hypercircuit,0.3.0*%\nG04 Generated by hypercircuit*\n%MOMM*%\n%FSLAX46Y46*%\n%LPD*%\n%ADD10C,0.010000*%\nD10*\nG75*\nG01*\n",
    );
    let mut projections = Vec::new();
    write_profile_contour(
        &mut output,
        &layout.outline.exterior,
        millimeter_factor,
        "board.exterior",
        projection,
        &mut projections,
    )?;
    for (index, cutout) in layout.outline.cutouts.iter().enumerate() {
        write_profile_contour(
            &mut output,
            cutout,
            millimeter_factor,
            &format!("board.cutout[{index}]"),
            projection,
            &mut projections,
        )?;
    }
    output.push_str("M02*\n");
    Ok((output.into_bytes(), projections))
}

fn write_profile_contour(
    output: &mut String,
    contour: &crate::BoardContour,
    factor: &Real,
    source: &str,
    projection: FabricationContourProjectionPolicy,
    projections: &mut Vec<MaterializationProjection>,
) -> Result<(), FabricationPackageError> {
    let Some(first) = contour.segments().first() else {
        return Err(FabricationPackageError::Gerber(format!(
            "empty board profile {source}"
        )));
    };
    let coordinate = |value: &Real| {
        gerber_coordinate(&(value.clone() * factor.clone())).ok_or_else(|| {
            FabricationPackageError::Gerber(format!("non-finite board profile {source}"))
        })
    };
    writeln!(
        output,
        "X{}Y{}D02*",
        coordinate(&first.start().x)?,
        coordinate(&first.start().y)?
    )
    .expect("writing to String cannot fail");
    for (index, segment) in contour.segments().iter().enumerate() {
        match segment {
            crate::BoardContourSegment::Line(line) => {
                writeln!(
                    output,
                    "G01*X{}Y{}D01*",
                    coordinate(&line.end().x)?,
                    coordinate(&line.end().y)?
                )
                .expect("writing to String cannot fail");
            }
            crate::BoardContourSegment::CircularArc(arc) => {
                let i = arc.center().x.clone() - arc.start().x.clone();
                let j = arc.center().y.clone() - arc.start().y.clone();
                writeln!(
                    output,
                    "{}*X{}Y{}I{}J{}D01*",
                    if arc.direction() == hyperpath::ArcDirection::Cw {
                        "G02"
                    } else {
                        "G03"
                    },
                    coordinate(&arc.end().x)?,
                    coordinate(&arc.end().y)?,
                    coordinate(&i)?,
                    coordinate(&j)?
                )
                .expect("writing to String cannot fail");
            }
            crate::BoardContourSegment::CubicBezier(bezier) => {
                let FabricationContourProjectionPolicy::CubicBezierPolyline { chord_error } =
                    projection
                else {
                    return Err(FabricationPackageError::Gerber(format!(
                        "cubic board profile {source}[{index}] requires an explicit fabrication projection policy"
                    )));
                };
                let points = crate::materialize::project_cubic_bezier(bezier, chord_error, source)
                    .map_err(|error| FabricationPackageError::Gerber(error.to_string()))?;
                for point in &points {
                    writeln!(
                        output,
                        "G01*X{}Y{}D01*",
                        coordinate(&point.x)?,
                        coordinate(&point.y)?
                    )
                    .expect("writing to String cannot fail");
                }
                projections.push(MaterializationProjection::CubicBezierBoardContourPolyline {
                    source: source.to_owned(),
                    segment: index,
                    chord_error: chord_error.to_string(),
                    generated_segments: points.len(),
                });
            }
        }
    }
    Ok(())
}

fn excellon(
    drills: &[&crate::DrillHit],
    plated: bool,
    millimeter_factor: &Real,
) -> Result<String, FabricationPackageError> {
    let mut tools = BTreeMap::<String, (Real, Vec<&crate::DrillHit>)>::new();
    for drill in drills {
        let diameter = match &drill.shape {
            DrillShape::Round { diameter } => diameter,
            DrillShape::Slot { width, .. } => width,
        };
        tools
            .entry(diameter.to_string())
            .or_insert_with(|| (diameter.clone(), Vec::new()))
            .1
            .push(drill);
    }
    let mut output = String::from("M48\n;FILE_FORMAT=4:6\nMETRIC,TZ\n");
    writeln!(
        output,
        "; #@! TF.FileFunction,{},{},{}",
        if plated { "Plated" } else { "NonPlated" },
        1,
        if plated { "PTH" } else { "NPTH" }
    )
    .expect("writing to String cannot fail");
    for (index, (_, (diameter, _))) in tools.iter().enumerate() {
        let diameter = finite(&(diameter.clone() * millimeter_factor.clone()))
            .ok_or_else(|| FabricationPackageError::NonFiniteDrill("tool diameter".into()))?;
        writeln!(output, "T{:02}C{diameter:.6}", index + 1).expect("writing to String cannot fail");
    }
    output.push_str("%\nG90\nG05\n");
    for (index, (_, (_, hits))) in tools.iter().enumerate() {
        writeln!(output, "T{:02}", index + 1).expect("writing to String cannot fail");
        for hit in hits {
            match &hit.shape {
                DrillShape::Round { .. } => {
                    let x =
                        excellon_coordinate(&(hit.center.x.clone() * millimeter_factor.clone()))
                            .ok_or_else(|| {
                                FabricationPackageError::NonFiniteDrill(hit.source.clone())
                            })?;
                    let y =
                        excellon_coordinate(&(hit.center.y.clone() * millimeter_factor.clone()))
                            .ok_or_else(|| {
                                FabricationPackageError::NonFiniteDrill(hit.source.clone())
                            })?;
                    writeln!(output, "X{x}Y{y}").expect("writing to String cannot fail");
                }
                DrillShape::Slot { start, end, .. } => {
                    let start_x =
                        excellon_coordinate(&(start.x.clone() * millimeter_factor.clone()))
                            .ok_or_else(|| {
                                FabricationPackageError::NonFiniteDrill(hit.source.clone())
                            })?;
                    let start_y =
                        excellon_coordinate(&(start.y.clone() * millimeter_factor.clone()))
                            .ok_or_else(|| {
                                FabricationPackageError::NonFiniteDrill(hit.source.clone())
                            })?;
                    let end_x = excellon_coordinate(&(end.x.clone() * millimeter_factor.clone()))
                        .ok_or_else(|| {
                        FabricationPackageError::NonFiniteDrill(hit.source.clone())
                    })?;
                    let end_y = excellon_coordinate(&(end.y.clone() * millimeter_factor.clone()))
                        .ok_or_else(|| {
                        FabricationPackageError::NonFiniteDrill(hit.source.clone())
                    })?;
                    writeln!(output, "G00X{start_x}Y{start_y}")
                        .expect("writing to String cannot fail");
                    output.push_str("M15\n");
                    writeln!(output, "G01X{end_x}Y{end_y}").expect("writing to String cannot fail");
                    output.push_str("M16\nG05\n");
                }
            }
        }
    }
    output.push_str("M30\n");
    Ok(output)
}

fn finite(value: &Real) -> Option<f64> {
    value.to_f64_lossy().filter(|value| value.is_finite())
}

fn excellon_coordinate(value: &Real) -> Option<String> {
    let scaled = (finite(value)? * 1_000_000.0).round();
    if scaled < i64::MIN as f64 || scaled > i64::MAX as f64 {
        return None;
    }
    let scaled = scaled as i64;
    if scaled < 0 {
        Some(format!("-{:010}", scaled.unsigned_abs()))
    } else {
        Some(format!("{scaled:010}"))
    }
}

fn gerber_coordinate(value: &Real) -> Option<String> {
    excellon_coordinate(value)
}

fn sanitize(value: &str) -> String {
    value
        .chars()
        .map(|character| {
            if character.is_ascii_alphanumeric() || matches!(character, '-' | '_') {
                character
            } else {
                '_'
            }
        })
        .collect()
}
