//! Deterministic, self-verifying manufacturing release bundles.

use std::collections::{BTreeMap, BTreeSet};
use std::fmt::{Display, Formatter};
use std::fs;
use std::path::Path;

use serde::{Deserialize, Serialize};

use crate::{
    ArtifactCatalog, ArtifactCatalogError, AssemblyCsvDocument, CoordinateFrame2,
    FabricationFileKind, PackageDigest, ReleaseArtifactDescriptor, ReleaseArtifactRole,
    ReleasePreparationError, ReleasePreparationOptions, ReleasePreparationReport, SemanticDocument,
};

/// Stable manufacturing-release manifest schema.
pub const MANUFACTURING_RELEASE_SCHEMA: &str = "hypercircuit.manufacturing-release";
/// Current manufacturing-release manifest version.
pub const MANUFACTURING_RELEASE_VERSION: u32 = 1;
/// Deterministic manifest path inside a release bundle.
pub const MANUFACTURING_RELEASE_MANIFEST_PATH: &str = "manufacturing-release.json";

/// Typed disposition for optional evidence.
#[derive(Clone, Debug, Eq, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum OptionalEvidenceStatus {
    NotRequired { reason: String },
    NotProvided { reason: String },
    Provided { artifact: String },
}

/// Detached signature metadata. Empty by default; signing never changes core identity.
#[derive(Clone, Debug, Eq, PartialEq, Serialize, Deserialize)]
pub struct SignatureEnvelope {
    pub algorithm: String,
    pub key_id: String,
    pub signed_digest: String,
    pub signature: String,
}

/// Low-friction release identity and policy choices.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ManufacturingReleaseOptions {
    pub product: String,
    pub revision: String,
    pub variant: String,
    pub source_revision: Option<String>,
    pub coordinate_frame: CoordinateFrame2,
    pub standards: Vec<String>,
    pub mixed_signal_evidence: OptionalEvidenceStatus,
    pub signatures: Vec<SignatureEnvelope>,
}

impl ManufacturingReleaseOptions {
    /// Opinionated defaults requiring only an existing semantic document.
    pub fn for_document(document: &SemanticDocument) -> Self {
        Self {
            product: document.circuit.id.as_str().into(),
            revision: format!("design-{}", document.design_revision.value()),
            variant: "default".into(),
            source_revision: None,
            coordinate_frame: CoordinateFrame2::board_default(),
            standards: Vec::new(),
            mixed_signal_evidence: OptionalEvidenceStatus::NotRequired {
                reason: "no release-specific mixed-signal certification requirement declared"
                    .into(),
            },
            signatures: Vec::new(),
        }
    }
}

/// Immutable release core. Detached signatures are deliberately outside its digest.
#[derive(Clone, Debug, Eq, PartialEq, Serialize, Deserialize)]
pub struct ManufacturingReleaseCore {
    pub schema: String,
    pub version: u32,
    pub product: String,
    pub revision: String,
    pub variant: String,
    pub generated_by: String,
    pub source_revision: Option<String>,
    pub coordinate_frame: CoordinateFrame2,
    pub standards: Vec<String>,
    pub capability_profile_id: String,
    pub capability_profile_revision: String,
    pub capability_profile_digest: String,
    pub fabrication_manifest_digest: PackageDigest,
    pub assembly_digest: PackageDigest,
    pub drc_report_digest: PackageDigest,
    pub design_intent_digest: PackageDigest,
    pub mixed_signal_evidence: OptionalEvidenceStatus,
    pub test_evidence: OptionalEvidenceStatus,
    pub panel_definition: Option<String>,
    pub release_clean: bool,
    pub release_blockers: Vec<String>,
    pub artifacts: ArtifactCatalog,
}

impl ManufacturingReleaseCore {
    /// Deterministic unsigned core bytes.
    pub fn canonical_bytes(&self) -> Vec<u8> {
        serde_json::to_vec(self).expect("release core is infallibly serializable")
    }

    /// Stable identity unaffected by detached signature envelopes.
    pub fn digest(&self) -> PackageDigest {
        PackageDigest::sha256(&self.canonical_bytes())
    }
}

/// Serialized manifest envelope with optional detached signatures.
#[derive(Clone, Debug, Eq, PartialEq, Serialize, Deserialize)]
pub struct ManufacturingReleaseManifest {
    pub core: ManufacturingReleaseCore,
    pub core_digest: PackageDigest,
    pub signatures: Vec<SignatureEnvelope>,
}

/// Exact bundle files and their verified manifest.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ManufacturingReleaseBundle {
    pub manifest: ManufacturingReleaseManifest,
    pub files: BTreeMap<String, Vec<u8>>,
}

/// High-level deterministic release comparison.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ManufacturingReleaseDifference {
    Identical,
    MetadataOnly,
    ArtifactOrSemantic,
}

/// Failure to construct, verify, or write a release.
#[derive(Debug)]
pub enum ManufacturingReleaseError {
    Preparation(ReleasePreparationError),
    Catalog(ArtifactCatalogError),
    MissingArtifact(String),
    UnexpectedArtifact(String),
    ArtifactMismatch(String),
    ManifestMismatch,
    Io(String),
    ManifestJson(String),
    UnsupportedManifest,
}

impl Display for ManufacturingReleaseError {
    fn fmt(&self, formatter: &mut Formatter<'_>) -> std::fmt::Result {
        write!(formatter, "{self:?}")
    }
}

impl std::error::Error for ManufacturingReleaseError {}

impl From<ReleasePreparationError> for ManufacturingReleaseError {
    fn from(value: ReleasePreparationError) -> Self {
        Self::Preparation(value)
    }
}

impl From<ArtifactCatalogError> for ManufacturingReleaseError {
    fn from(value: ArtifactCatalogError) -> Self {
        Self::Catalog(value)
    }
}

impl SemanticDocument {
    /// Builds a complete unsigned-by-default release solely through library APIs.
    pub fn build_manufacturing_release(
        &self,
        release: ManufacturingReleaseOptions,
        preparation: ReleasePreparationOptions,
    ) -> Result<ManufacturingReleaseBundle, ManufacturingReleaseError> {
        let report = self.prepare_release(preparation)?;
        ManufacturingReleaseBundle::from_report(self, &report, release)
    }
}

impl ManufacturingReleaseBundle {
    /// Packages one already-prepared release report without rerunning geometry.
    pub fn from_report(
        document: &SemanticDocument,
        report: &ReleasePreparationReport,
        mut options: ManufacturingReleaseOptions,
    ) -> Result<Self, ManufacturingReleaseError> {
        options.coordinate_frame.validate().map_err(|error| {
            ManufacturingReleaseError::Io(format!("invalid coordinate frame: {error:?}"))
        })?;
        options.standards.sort();
        options.standards.dedup();
        let mut files = BTreeMap::new();
        let mut descriptors = Vec::new();
        for file in &report.fabrication.files {
            let path = format!("fabrication/{}", file.name);
            let role = match file.kind {
                FabricationFileKind::GerberX2 => ReleaseArtifactRole::Gerber,
                FabricationFileKind::Excellon => ReleaseArtifactRole::Drill,
                FabricationFileKind::Ipc356 => ReleaseArtifactRole::Netlist,
                FabricationFileKind::Manifest => ReleaseArtifactRole::FabricationManifest,
            };
            descriptors.push(ReleaseArtifactDescriptor::from_bytes(
                path.clone(),
                role,
                &file.bytes,
            )?);
            files.insert(path, file.bytes.clone());
        }
        let assembly_files = [
            (
                "assembly/bom.csv",
                ReleaseArtifactRole::Bom,
                report.assembly.bom_csv().into_bytes(),
            ),
            (
                "assembly/pick-and-place.csv",
                ReleaseArtifactRole::PickAndPlace,
                report.assembly.pick_and_place_csv().into_bytes(),
            ),
            (
                "assembly/dnp.csv",
                ReleaseArtifactRole::AssemblyData,
                report.assembly.dnp_csv().into_bytes(),
            ),
        ];
        for (path, role, bytes) in assembly_files {
            descriptors.push(ReleaseArtifactDescriptor::from_bytes(path, role, &bytes)?);
            files.insert(path.into(), bytes);
        }
        let release_identity = format!("{}@{}", options.product, options.revision);
        let canonical_assembly_data = if let Some(panel) = &document.panel {
            let board = &document
                .pcb
                .as_ref()
                .expect("release preparation requires PCB layout")
                .id;
            report
                .assembly
                .canonical_v2_for_panel(release_identity, panel, board)
                .map_err(|error| ManufacturingReleaseError::Io(format!("{error:?}")))?
        } else {
            report.assembly.canonical_v2(
                release_identity,
                options.coordinate_frame.clone(),
                None,
                None,
            )
        };
        let canonical_assembly = serde_json::to_string_pretty(&canonical_assembly_data)
            .expect("canonical assembly data is infallibly serializable");
        let canonical_assembly_path = "assembly/assembly-v2.json";
        descriptors.push(ReleaseArtifactDescriptor::from_bytes(
            canonical_assembly_path,
            ReleaseArtifactRole::AssemblyData,
            canonical_assembly.as_bytes(),
        )?);
        files.insert(
            canonical_assembly_path.into(),
            canonical_assembly.into_bytes(),
        );
        let canonical_schema_path = "assembly/assembly-v2.schema.json";
        let canonical_schema =
            serde_json::to_vec_pretty(&crate::AssemblyOutputs::canonical_v2_json_schema())
                .expect("assembly JSON Schema is infallibly serializable");
        descriptors.push(ReleaseArtifactDescriptor::from_bytes(
            canonical_schema_path,
            ReleaseArtifactRole::Other("json-schema".into()),
            &canonical_schema,
        )?);
        files.insert(canonical_schema_path.into(), canonical_schema);
        for (document_kind, path) in [
            (AssemblyCsvDocument::Bom, "assembly/bom.csv.dialect.json"),
            (
                AssemblyCsvDocument::PickAndPlace,
                "assembly/pick-and-place.csv.dialect.json",
            ),
            (AssemblyCsvDocument::Dnp, "assembly/dnp.csv.dialect.json"),
        ] {
            let bytes =
                serde_json::to_vec_pretty(&crate::AssemblyOutputs::csv_dialect(document_kind))
                    .expect("assembly CSV dialect is infallibly serializable");
            descriptors.push(ReleaseArtifactDescriptor::from_bytes(
                path,
                ReleaseArtifactRole::AssemblyData,
                &bytes,
            )?);
            files.insert(path.into(), bytes);
        }
        #[derive(Serialize)]
        struct DrcEvidence<'a> {
            capability_profile_id: &'a str,
            capability_profile_revision: &'a str,
            capability_profile_digest: &'a str,
            coverage: &'a hyperdrc::CheckCoverage,
            test_coverage: &'a hyperdrc::NativeTestCoverageReport,
            violations: &'a [hyperdrc::Violation],
        }
        let drc_bytes = serde_json::to_vec(&DrcEvidence {
            capability_profile_id: &report.drc.capability_profile_id,
            capability_profile_revision: &report.drc.capability_profile_revision,
            capability_profile_digest: &report.drc.capability_profile_digest,
            coverage: &report.drc.coverage,
            test_coverage: &report.drc.test_coverage,
            violations: &report.drc.violations,
        })
        .expect("DRC release evidence is infallibly serializable");
        descriptors.push(ReleaseArtifactDescriptor::from_bytes(
            "evidence/hyperdrc.json",
            ReleaseArtifactRole::DrcReport,
            &drc_bytes,
        )?);
        files.insert("evidence/hyperdrc.json".into(), drc_bytes.clone());

        let fabrication_manifest_bytes = report
            .fabrication
            .files
            .iter()
            .find(|file| file.kind == FabricationFileKind::Manifest)
            .map(|file| file.bytes.as_slice())
            .ok_or_else(|| {
                ManufacturingReleaseError::MissingArtifact("fabrication manifest".into())
            })?;
        let assembly_digest = {
            let mut bytes = Vec::new();
            for (path, file) in files
                .iter()
                .filter(|(path, _)| path.starts_with("assembly/"))
            {
                bytes.extend_from_slice(&(path.len() as u64).to_be_bytes());
                bytes.extend_from_slice(path.as_bytes());
                bytes.extend_from_slice(file);
            }
            PackageDigest::sha256(&bytes)
        };
        let design_intent_bytes = serde_json::to_vec(&document.design_intent)
            .expect("design intent is infallibly serializable");
        let test_evidence = if document.test_intent == Default::default() {
            OptionalEvidenceStatus::NotRequired {
                reason: "no design-for-test requirements declared".into(),
            }
        } else {
            let path = "test/test-intent.json";
            let bytes = serde_json::to_vec_pretty(&document.test_intent)
                .expect("test intent is infallibly serializable");
            descriptors.push(ReleaseArtifactDescriptor::from_bytes(
                path,
                ReleaseArtifactRole::TestIntent,
                &bytes,
            )?);
            files.insert(path.into(), bytes);
            if let (Some(panel), Some(pcb)) = (&document.panel, &document.pcb) {
                let panel_access = panel
                    .panelize_test_access(&pcb.id, &document.test_intent)
                    .map_err(|error| ManufacturingReleaseError::Io(format!("{error:?}")))?;
                let panel_path = "test/panel-test-access.json";
                let panel_bytes = serde_json::to_vec_pretty(&panel_access)
                    .expect("panel test access is infallibly serializable");
                descriptors.push(ReleaseArtifactDescriptor::from_bytes(
                    panel_path,
                    ReleaseArtifactRole::TestIntent,
                    &panel_bytes,
                )?);
                files.insert(panel_path.into(), panel_bytes);
            }
            OptionalEvidenceStatus::Provided {
                artifact: path.into(),
            }
        };
        let panel_definition = if let Some(panel) = &document.panel {
            let json_path = "panel/panel.json";
            let json =
                serde_json::to_vec_pretty(panel).expect("panel intent is infallibly serializable");
            descriptors.push(ReleaseArtifactDescriptor::from_bytes(
                json_path,
                ReleaseArtifactRole::PanelDefinition,
                &json,
            )?);
            files.insert(json_path.into(), json);
            let svg_path = "panel/panel.svg";
            let svg = panel
                .to_svg()
                .map_err(|error| ManufacturingReleaseError::Io(format!("{error:?}")))?;
            descriptors.push(ReleaseArtifactDescriptor::from_bytes(
                svg_path,
                ReleaseArtifactRole::Drawing,
                svg.as_bytes(),
            )?);
            files.insert(svg_path.into(), svg.into_bytes());
            Some(json_path.into())
        } else {
            None
        };
        let artifacts = ArtifactCatalog::new(descriptors)?;
        let release_blockers = report
            .release_blockers()
            .into_iter()
            .map(|blocker| format!("{blocker:?}"))
            .collect::<Vec<_>>();
        let core = ManufacturingReleaseCore {
            schema: MANUFACTURING_RELEASE_SCHEMA.into(),
            version: MANUFACTURING_RELEASE_VERSION,
            product: options.product,
            revision: options.revision,
            variant: options.variant,
            generated_by: format!("hypercircuit/{}", env!("CARGO_PKG_VERSION")),
            source_revision: options.source_revision,
            coordinate_frame: options.coordinate_frame,
            standards: options.standards,
            capability_profile_id: report.drc.capability_profile_id.clone(),
            capability_profile_revision: report.drc.capability_profile_revision.clone(),
            capability_profile_digest: report.drc.capability_profile_digest.clone(),
            fabrication_manifest_digest: PackageDigest::sha256(fabrication_manifest_bytes),
            assembly_digest,
            drc_report_digest: PackageDigest::sha256(&drc_bytes),
            design_intent_digest: PackageDigest::sha256(&design_intent_bytes),
            mixed_signal_evidence: options.mixed_signal_evidence,
            test_evidence,
            panel_definition,
            release_clean: release_blockers.is_empty(),
            release_blockers,
            artifacts,
        };
        let manifest = ManufacturingReleaseManifest {
            core_digest: core.digest(),
            core,
            signatures: options.signatures,
        };
        let manifest_bytes =
            serde_json::to_vec_pretty(&manifest).expect("manifest is infallibly serializable");
        files.insert(MANUFACTURING_RELEASE_MANIFEST_PATH.into(), manifest_bytes);
        let bundle = Self { manifest, files };
        bundle.verify()?;
        Ok(bundle)
    }

    /// Verifies manifest identity and every listed/unlisted exact byte stream.
    pub fn verify(&self) -> Result<(), ManufacturingReleaseError> {
        if self.manifest.core.digest() != self.manifest.core_digest {
            return Err(ManufacturingReleaseError::ManifestMismatch);
        }
        let expected = self
            .manifest
            .core
            .artifacts
            .artifacts
            .iter()
            .map(|artifact| artifact.path.as_str())
            .collect::<BTreeSet<_>>();
        for artifact in &self.manifest.core.artifacts.artifacts {
            let bytes = self.files.get(artifact.path.as_str()).ok_or_else(|| {
                ManufacturingReleaseError::MissingArtifact(artifact.path.as_str().into())
            })?;
            if !artifact.verifies(bytes) {
                return Err(ManufacturingReleaseError::ArtifactMismatch(
                    artifact.path.as_str().into(),
                ));
            }
        }
        for path in self.files.keys() {
            if path != MANUFACTURING_RELEASE_MANIFEST_PATH && !expected.contains(path.as_str()) {
                return Err(ManufacturingReleaseError::UnexpectedArtifact(path.clone()));
            }
        }
        let expected_manifest =
            serde_json::to_vec_pretty(&self.manifest).expect("manifest is serializable");
        if self.files[MANUFACTURING_RELEASE_MANIFEST_PATH] != expected_manifest {
            return Err(ManufacturingReleaseError::ManifestMismatch);
        }
        Ok(())
    }

    /// Classifies differences without teaching callers manifest internals.
    pub fn compare(&self, other: &Self) -> ManufacturingReleaseDifference {
        if self.manifest.core_digest == other.manifest.core_digest {
            ManufacturingReleaseDifference::Identical
        } else if self.manifest.core.artifacts == other.manifest.core.artifacts {
            ManufacturingReleaseDifference::MetadataOnly
        } else {
            ManufacturingReleaseDifference::ArtifactOrSemantic
        }
    }

    /// Deterministic human/JSON inspection representation.
    pub fn manifest_json_pretty(&self) -> String {
        serde_json::to_string_pretty(&self.manifest)
            .expect("manufacturing release manifest is infallibly serializable")
    }

    /// Writes a verified release directory with portable catalog paths.
    pub fn write_directory(&self, root: &Path) -> Result<(), ManufacturingReleaseError> {
        self.verify()?;
        for (relative, bytes) in &self.files {
            let destination = root.join(relative);
            if let Some(parent) = destination.parent() {
                fs::create_dir_all(parent).map_err(|error| {
                    ManufacturingReleaseError::Io(format!(
                        "cannot create {}: {error}",
                        parent.display()
                    ))
                })?;
            }
            fs::write(&destination, bytes).map_err(|error| {
                ManufacturingReleaseError::Io(format!(
                    "cannot write {}: {error}",
                    destination.display()
                ))
            })?;
        }
        Ok(())
    }

    /// Loads only catalog-listed regular files from a release directory.
    pub fn read_directory(root: &Path) -> Result<Self, ManufacturingReleaseError> {
        let manifest_path = root.join(MANUFACTURING_RELEASE_MANIFEST_PATH);
        let manifest_bytes = fs::read(&manifest_path).map_err(|error| {
            ManufacturingReleaseError::Io(format!(
                "cannot read {}: {error}",
                manifest_path.display()
            ))
        })?;
        let manifest: ManufacturingReleaseManifest = serde_json::from_slice(&manifest_bytes)
            .map_err(|error| ManufacturingReleaseError::ManifestJson(error.to_string()))?;
        if manifest.core.schema != MANUFACTURING_RELEASE_SCHEMA
            || manifest.core.version != MANUFACTURING_RELEASE_VERSION
        {
            return Err(ManufacturingReleaseError::UnsupportedManifest);
        }
        let mut files =
            BTreeMap::from([(MANUFACTURING_RELEASE_MANIFEST_PATH.into(), manifest_bytes)]);
        for artifact in &manifest.core.artifacts.artifacts {
            let path = root.join(artifact.path.as_str());
            let metadata = fs::symlink_metadata(&path).map_err(|error| {
                ManufacturingReleaseError::Io(format!("cannot inspect {}: {error}", path.display()))
            })?;
            if !metadata.file_type().is_file() {
                return Err(ManufacturingReleaseError::Io(format!(
                    "{} is not a regular file",
                    path.display()
                )));
            }
            let bytes = fs::read(&path).map_err(|error| {
                ManufacturingReleaseError::Io(format!("cannot read {}: {error}", path.display()))
            })?;
            files.insert(artifact.path.as_str().into(), bytes);
        }
        let bundle = Self { manifest, files };
        bundle.verify()?;
        Ok(bundle)
    }
}
