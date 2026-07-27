//! Deterministic, self-verifying manufacturing release bundles.

use std::collections::{BTreeMap, BTreeSet};
use std::fmt::{Display, Formatter};
use std::fs;
use std::io::{Cursor, Read, Write};
use std::path::Path;

use ring::signature::{ED25519, Ed25519KeyPair, KeyPair, UnparsedPublicKey};
use serde::{Deserialize, Serialize};

use crate::{
    ArtifactCatalog, ArtifactCatalogError, AssemblyCsvDocument, CoordinateFrame2,
    FabricationFileKind, PackageDigest, PortableArtifactPath, ReleaseArtifactDescriptor,
    ReleaseArtifactRole, ReleasePreparationError, ReleasePreparationOptions,
    ReleasePreparationReport, SemanticDocument,
};

/// Stable manufacturing-release manifest schema.
pub const MANUFACTURING_RELEASE_SCHEMA: &str = "hypercircuit.manufacturing-release";
/// Current manufacturing-release manifest version.
pub const MANUFACTURING_RELEASE_VERSION: u32 = 1;
/// Deterministic manifest path inside a release bundle.
pub const MANUFACTURING_RELEASE_MANIFEST_PATH: &str = "manufacturing-release.json";
/// Bundled JSON Schema path for the manufacturing-release envelope.
pub const MANUFACTURING_RELEASE_JSON_SCHEMA_PATH: &str =
    "metadata/manufacturing-release-v1.schema.json";

/// Returns the committed JSON Schema for the manufacturing-release envelope.
pub fn manufacturing_release_json_schema() -> serde_json::Value {
    serde_json::from_str(include_str!(
        "../schemas/manufacturing-release-v1.schema.json"
    ))
    .expect("committed manufacturing-release JSON Schema is valid JSON")
}

/// Typed disposition for optional evidence.
#[derive(Clone, Debug, Eq, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum OptionalEvidenceStatus {
    NotRequired { reason: String },
    NotProvided { reason: String },
    Provided { artifact: String },
}

/// Factory-facing operation required by the released design.
#[derive(Clone, Debug, Eq, Ord, PartialEq, PartialOrd, Serialize, Deserialize)]
pub struct ManufacturingRequirement {
    /// Requirement category without prescribing factory execution machinery.
    pub kind: ManufacturingRequirementKind,
    /// Human- and machine-stable requirement name.
    pub name: String,
    /// Optional contract or standard revision governing the requirement.
    pub standard: Option<String>,
}

/// Design-owned manufacturing requirement category.
#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ManufacturingRequirementKind {
    /// Inspection such as AOI, AXI, dimensional, or visual review.
    Inspection,
    /// Electrical, structural, programming, or functional test.
    Test,
    /// Special process such as controlled impedance, coating, or edge plating.
    SpecialProcess,
}

/// Detached signature metadata. Empty by default; signing never changes core identity.
#[derive(Clone, Debug, Eq, PartialEq, Serialize, Deserialize)]
pub struct SignatureEnvelope {
    pub algorithm: String,
    pub key_id: String,
    pub signed_digest: String,
    /// Embedded verification key for self-contained algorithms; not a trust assertion.
    #[serde(default)]
    pub public_key: Option<String>,
    pub signature: String,
}

/// Pluggable signer seam suitable for local keys, HSMs, or remote signing services.
pub trait ReleaseSigner {
    /// Stable signature algorithm identifier.
    fn algorithm(&self) -> &str;
    /// Stable operator-controlled key identity.
    fn key_id(&self) -> &str;
    /// Optional public verification material encoded into the envelope.
    fn public_key(&self) -> Option<Vec<u8>>;
    /// Signs the domain-separated release payload.
    fn sign(&self, payload: &[u8]) -> Result<Vec<u8>, String>;
}

/// Pluggable verifier seam for externally managed signature algorithms and trust policy.
pub trait ReleaseSignatureVerifier {
    /// Verifies one envelope over the supplied domain-separated payload.
    fn verify(&self, envelope: &SignatureEnvelope, payload: &[u8]) -> Result<(), String>;
}

/// Built-in Ed25519 signing key constructed from an explicit 32-byte seed.
pub struct Ed25519ReleaseSigner {
    key_id: String,
    key_pair: Ed25519KeyPair,
}

impl Ed25519ReleaseSigner {
    /// Constructs a deterministic signing key without introducing key-generation policy.
    pub fn from_seed(key_id: impl Into<String>, seed: &[u8]) -> Result<Self, String> {
        let key_id = key_id.into();
        if key_id.trim().is_empty() {
            return Err("Ed25519 key id must not be empty".into());
        }
        let key_pair = Ed25519KeyPair::from_seed_unchecked(seed)
            .map_err(|_| "Ed25519 seed must contain exactly 32 bytes".to_owned())?;
        Ok(Self { key_id, key_pair })
    }

    /// Constructs a key from a 64-character hexadecimal seed.
    pub fn from_seed_hex(key_id: impl Into<String>, seed: &str) -> Result<Self, String> {
        let seed = decode_hex(seed.trim())?;
        Self::from_seed(key_id, &seed)
    }
}

impl ReleaseSigner for Ed25519ReleaseSigner {
    fn algorithm(&self) -> &str {
        "ed25519"
    }

    fn key_id(&self) -> &str {
        &self.key_id
    }

    fn public_key(&self) -> Option<Vec<u8>> {
        Some(self.key_pair.public_key().as_ref().to_vec())
    }

    fn sign(&self, payload: &[u8]) -> Result<Vec<u8>, String> {
        Ok(self.key_pair.sign(payload).as_ref().to_vec())
    }
}

/// Built-in verifier for self-contained Ed25519 signature envelopes.
#[derive(Clone, Copy, Debug, Default)]
pub struct Ed25519ReleaseVerifier;

impl ReleaseSignatureVerifier for Ed25519ReleaseVerifier {
    fn verify(&self, envelope: &SignatureEnvelope, payload: &[u8]) -> Result<(), String> {
        if envelope.algorithm != "ed25519" {
            return Err(format!(
                "unsupported signature algorithm {}",
                envelope.algorithm
            ));
        }
        let public_key = envelope
            .public_key
            .as_deref()
            .ok_or_else(|| "Ed25519 envelope has no public key".to_owned())
            .and_then(decode_hex)?;
        let signature = decode_hex(&envelope.signature)?;
        UnparsedPublicKey::new(&ED25519, public_key)
            .verify(payload, &signature)
            .map_err(|_| format!("invalid Ed25519 signature from {}", envelope.key_id))
    }
}

/// Resource bounds applied while reading a release ZIP.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ReleaseArchiveLimits {
    /// Maximum compressed archive byte length.
    pub maximum_archive_bytes: u64,
    /// Maximum number of regular-file entries in one archive.
    pub maximum_entries: usize,
    /// Maximum uncompressed size of any one file.
    pub maximum_file_bytes: u64,
    /// Maximum aggregate uncompressed size of all files.
    pub maximum_total_bytes: u64,
    /// Maximum uncompressed-to-compressed ratio accepted for a nonempty file.
    pub maximum_compression_ratio: u64,
}

impl Default for ReleaseArchiveLimits {
    fn default() -> Self {
        Self {
            maximum_archive_bytes: 1024 * 1024 * 1024,
            maximum_entries: 4_096,
            maximum_file_bytes: 64 * 1024 * 1024,
            maximum_total_bytes: 1024 * 1024 * 1024,
            maximum_compression_ratio: 100,
        }
    }
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
    pub requirements: Vec<ManufacturingRequirement>,
    pub mixed_signal_evidence: OptionalEvidenceStatus,
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
            requirements: Vec::new(),
            mixed_signal_evidence: OptionalEvidenceStatus::NotRequired {
                reason: "no release-specific mixed-signal certification requirement declared"
                    .into(),
            },
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
    pub requirements: Vec<ManufacturingRequirement>,
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
    /// Manifest envelopes and all exact artifact bytes are identical.
    Identical,
    /// The immutable core is identical; only detached signature envelopes differ.
    SignatureOnly,
    /// Artifact bytes and design intent match; release metadata differs.
    MetadataOnly,
    /// Design intent matches, but one or more produced artifact bytes differ.
    ArtifactOnly,
    /// Canonical design intent differs.
    Semantic,
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
    Signature(String),
    Archive(String),
    ArchiveLimit(String),
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
    /// Loads a release directory or `.zip` archive using one opinionated API.
    pub fn read_path(path: &Path) -> Result<Self, ManufacturingReleaseError> {
        if path
            .extension()
            .is_some_and(|extension| extension.eq_ignore_ascii_case("zip"))
        {
            Self::read_zip(path)
        } else {
            Self::read_directory(path)
        }
    }

    /// Loads a directory or `.zip` while applying a caller-supplied signature policy.
    pub fn read_path_with_signature_verifier(
        path: &Path,
        verifier: &dyn ReleaseSignatureVerifier,
    ) -> Result<Self, ManufacturingReleaseError> {
        if path
            .extension()
            .is_some_and(|extension| extension.eq_ignore_ascii_case("zip"))
        {
            Self::read_zip_with_signature_verifier(path, verifier)
        } else {
            Self::read_directory_with_signature_verifier(path, verifier)
        }
    }

    /// Writes a directory by default, or a deterministic ZIP for a `.zip` path.
    pub fn write_path(&self, path: &Path) -> Result<(), ManufacturingReleaseError> {
        if path
            .extension()
            .is_some_and(|extension| extension.eq_ignore_ascii_case("zip"))
        {
            self.write_zip(path)
        } else {
            self.write_directory(path)
        }
    }

    /// Packages one already-prepared release report without rerunning geometry.
    pub fn from_report(
        document: &SemanticDocument,
        report: &ReleasePreparationReport,
        mut options: ManufacturingReleaseOptions,
    ) -> Result<Self, ManufacturingReleaseError> {
        options.coordinate_frame.validate().map_err(|error| {
            ManufacturingReleaseError::Io(format!("invalid coordinate frame: {error:?}"))
        })?;
        options.product = options.product.trim().to_owned();
        options.revision = options.revision.trim().to_owned();
        options.variant = options.variant.trim().to_owned();
        if options.product.is_empty() || options.revision.is_empty() || options.variant.is_empty() {
            return Err(ManufacturingReleaseError::Io(
                "release product, revision, and variant must not be empty".into(),
            ));
        }
        options.source_revision = options
            .source_revision
            .map(|revision| revision.trim().to_owned())
            .filter(|revision| !revision.is_empty());
        options.standards = options
            .standards
            .into_iter()
            .map(|standard| standard.trim().to_owned())
            .filter(|standard| !standard.is_empty())
            .collect();
        options.standards.sort();
        options.standards.dedup();
        for requirement in &mut options.requirements {
            requirement.name = requirement.name.trim().to_owned();
            requirement.standard = requirement
                .standard
                .take()
                .map(|standard| standard.trim().to_owned())
                .filter(|standard| !standard.is_empty());
            if requirement.name.trim().is_empty() {
                return Err(ManufacturingReleaseError::Io(
                    "manufacturing requirement names must not be empty".into(),
                ));
            }
        }
        options.requirements.sort();
        options.requirements.dedup();
        let mut files = BTreeMap::new();
        let mut descriptors = Vec::new();
        for file in &report.fabrication.files {
            let path = format!("fabrication/{}", file.name);
            let role = match file.kind {
                FabricationFileKind::GerberX2 | FabricationFileKind::GerberX3 => {
                    ReleaseArtifactRole::Gerber
                }
                FabricationFileKind::GerberJob => ReleaseArtifactRole::GerberJob,
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
        let release_schema = include_bytes!("../schemas/manufacturing-release-v1.schema.json");
        descriptors.push(ReleaseArtifactDescriptor::from_bytes(
            MANUFACTURING_RELEASE_JSON_SCHEMA_PATH,
            ReleaseArtifactRole::Other("json-schema".into()),
            release_schema,
        )?);
        files.insert(
            MANUFACTURING_RELEASE_JSON_SCHEMA_PATH.into(),
            release_schema.to_vec(),
        );
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
            requirements: options.requirements,
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
            signatures: Vec::new(),
        };
        let manifest_bytes =
            serde_json::to_vec_pretty(&manifest).expect("manifest is infallibly serializable");
        files.insert(MANUFACTURING_RELEASE_MANIFEST_PATH.into(), manifest_bytes);
        let bundle = Self { manifest, files };
        bundle.verify()?;
        Ok(bundle)
    }

    /// Adds or replaces one detached signature without changing core identity.
    pub fn sign_with(
        &mut self,
        signer: &dyn ReleaseSigner,
    ) -> Result<(), ManufacturingReleaseError> {
        self.verify_integrity()?;
        let algorithm = signer.algorithm().trim();
        let key_id = signer.key_id().trim();
        if algorithm.is_empty() || key_id.is_empty() {
            return Err(ManufacturingReleaseError::Signature(
                "signature algorithm and key id must not be empty".into(),
            ));
        }
        let signed_digest = self.manifest.core_digest.canonical_text();
        let payload = signature_payload(&signed_digest);
        let envelope = SignatureEnvelope {
            algorithm: algorithm.into(),
            key_id: key_id.into(),
            signed_digest,
            public_key: signer.public_key().map(|bytes| encode_hex(&bytes)),
            signature: encode_hex(
                &signer
                    .sign(&payload)
                    .map_err(ManufacturingReleaseError::Signature)?,
            ),
        };
        self.manifest.signatures.retain(|existing| {
            existing.algorithm != envelope.algorithm || existing.key_id != envelope.key_id
        });
        self.manifest.signatures.push(envelope);
        self.manifest.signatures.sort_by(|left, right| {
            (&left.algorithm, &left.key_id).cmp(&(&right.algorithm, &right.key_id))
        });
        self.refresh_manifest_file();
        self.verify_integrity()
    }

    /// Verifies manifest, artifact bytes, and built-in Ed25519 signatures.
    pub fn verify(&self) -> Result<(), ManufacturingReleaseError> {
        self.verify_with_signature_verifier(&Ed25519ReleaseVerifier)
    }

    /// Verifies all integrity rules and delegates every signature to a caller policy.
    pub fn verify_with_signature_verifier(
        &self,
        verifier: &dyn ReleaseSignatureVerifier,
    ) -> Result<(), ManufacturingReleaseError> {
        self.verify_integrity()?;
        for envelope in &self.manifest.signatures {
            let expected_digest = self.manifest.core_digest.canonical_text();
            if envelope.signed_digest != expected_digest {
                return Err(ManufacturingReleaseError::Signature(format!(
                    "signature {}:{} names digest {} instead of {}",
                    envelope.algorithm, envelope.key_id, envelope.signed_digest, expected_digest
                )));
            }
            verifier
                .verify(envelope, &signature_payload(&expected_digest))
                .map_err(ManufacturingReleaseError::Signature)?;
        }
        Ok(())
    }

    fn verify_integrity(&self) -> Result<(), ManufacturingReleaseError> {
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
        let actual_manifest = self
            .files
            .get(MANUFACTURING_RELEASE_MANIFEST_PATH)
            .ok_or_else(|| {
                ManufacturingReleaseError::MissingArtifact(
                    MANUFACTURING_RELEASE_MANIFEST_PATH.into(),
                )
            })?;
        if actual_manifest != &expected_manifest {
            return Err(ManufacturingReleaseError::ManifestMismatch);
        }
        Ok(())
    }

    fn refresh_manifest_file(&mut self) {
        let manifest_bytes =
            serde_json::to_vec_pretty(&self.manifest).expect("manifest is serializable");
        self.files
            .insert(MANUFACTURING_RELEASE_MANIFEST_PATH.into(), manifest_bytes);
    }

    /// Classifies differences without teaching callers manifest internals.
    pub fn compare(&self, other: &Self) -> ManufacturingReleaseDifference {
        if self == other {
            ManufacturingReleaseDifference::Identical
        } else if self.manifest.core_digest == other.manifest.core_digest {
            ManufacturingReleaseDifference::SignatureOnly
        } else if self.manifest.core.design_intent_digest
            != other.manifest.core.design_intent_digest
        {
            ManufacturingReleaseDifference::Semantic
        } else if self.manifest.core.artifacts == other.manifest.core.artifacts {
            ManufacturingReleaseDifference::MetadataOnly
        } else {
            ManufacturingReleaseDifference::ArtifactOnly
        }
    }

    /// Deterministic human/JSON inspection representation.
    pub fn manifest_json_pretty(&self) -> String {
        serde_json::to_string_pretty(&self.manifest)
            .expect("manufacturing release manifest is infallibly serializable")
    }

    /// Serializes a byte-identical ZIP after verifying core and artifact integrity.
    ///
    /// Signature trust is deliberately caller-owned so externally signed
    /// envelopes can be transported without teaching HyperCircuit the trust store.
    pub fn zip_bytes(&self) -> Result<Vec<u8>, ManufacturingReleaseError> {
        self.verify_integrity()?;
        let cursor = Cursor::new(Vec::new());
        let mut writer = zip::ZipWriter::new(cursor);
        let options = zip::write::SimpleFileOptions::default()
            .compression_method(zip::CompressionMethod::Stored)
            .last_modified_time(zip::DateTime::default())
            .unix_permissions(0o644);
        for (path, bytes) in &self.files {
            writer.start_file(path, options).map_err(|error| {
                ManufacturingReleaseError::Archive(format!(
                    "cannot start ZIP entry {path}: {error}"
                ))
            })?;
            writer.write_all(bytes).map_err(|error| {
                ManufacturingReleaseError::Archive(format!(
                    "cannot write ZIP entry {path}: {error}"
                ))
            })?;
        }
        writer
            .finish()
            .map(|cursor| cursor.into_inner())
            .map_err(|error| {
                ManufacturingReleaseError::Archive(format!("cannot finish ZIP: {error}"))
            })
    }

    /// Writes one deterministic verified ZIP release.
    pub fn write_zip(&self, path: &Path) -> Result<(), ManufacturingReleaseError> {
        let bytes = self.zip_bytes()?;
        if let Some(parent) = path
            .parent()
            .filter(|parent| !parent.as_os_str().is_empty())
        {
            ensure_safe_directory(parent)?;
        }
        if fs::symlink_metadata(path)
            .ok()
            .is_some_and(|metadata| !metadata.file_type().is_file())
        {
            return Err(ManufacturingReleaseError::Io(format!(
                "{} is not a regular file",
                path.display()
            )));
        }
        fs::write(path, bytes).map_err(|error| {
            ManufacturingReleaseError::Io(format!("cannot write {}: {error}", path.display()))
        })
    }

    /// Reads a ZIP release under opinionated safe resource limits.
    pub fn read_zip(path: &Path) -> Result<Self, ManufacturingReleaseError> {
        Self::read_zip_with_limits(path, ReleaseArchiveLimits::default())
    }

    /// Reads a ZIP and applies a caller-supplied signature trust policy.
    pub fn read_zip_with_signature_verifier(
        path: &Path,
        verifier: &dyn ReleaseSignatureVerifier,
    ) -> Result<Self, ManufacturingReleaseError> {
        let limits = ReleaseArchiveLimits::default();
        let bytes = read_archive_bytes(path, limits)?;
        let bundle = Self::from_zip_bytes_integrity(&bytes, limits)?;
        bundle.verify_with_signature_verifier(verifier)?;
        Ok(bundle)
    }

    /// Reads a ZIP release under caller-selected safe resource limits.
    pub fn read_zip_with_limits(
        path: &Path,
        limits: ReleaseArchiveLimits,
    ) -> Result<Self, ManufacturingReleaseError> {
        let bytes = read_archive_bytes(path, limits)?;
        Self::from_zip_bytes_with_limits(&bytes, limits)
    }

    /// Parses an in-memory ZIP release under opinionated safe resource limits.
    pub fn from_zip_bytes(bytes: &[u8]) -> Result<Self, ManufacturingReleaseError> {
        Self::from_zip_bytes_with_limits(bytes, ReleaseArchiveLimits::default())
    }

    /// Parses an in-memory ZIP and applies a caller-supplied signature policy.
    pub fn from_zip_bytes_with_signature_verifier(
        bytes: &[u8],
        verifier: &dyn ReleaseSignatureVerifier,
    ) -> Result<Self, ManufacturingReleaseError> {
        let bundle = Self::from_zip_bytes_integrity(bytes, ReleaseArchiveLimits::default())?;
        bundle.verify_with_signature_verifier(verifier)?;
        Ok(bundle)
    }

    /// Parses an in-memory ZIP release under caller-selected safe resource limits.
    pub fn from_zip_bytes_with_limits(
        bytes: &[u8],
        limits: ReleaseArchiveLimits,
    ) -> Result<Self, ManufacturingReleaseError> {
        let bundle = Self::from_zip_bytes_integrity(bytes, limits)?;
        bundle.verify()?;
        Ok(bundle)
    }

    fn from_zip_bytes_integrity(
        bytes: &[u8],
        limits: ReleaseArchiveLimits,
    ) -> Result<Self, ManufacturingReleaseError> {
        if bytes.len() as u64 > limits.maximum_archive_bytes {
            return Err(ManufacturingReleaseError::ArchiveLimit(format!(
                "ZIP container exceeds byte limit {}",
                limits.maximum_archive_bytes
            )));
        }
        let declared_entries = declared_zip_entries(bytes)?;
        if declared_entries > limits.maximum_entries {
            return Err(ManufacturingReleaseError::ArchiveLimit(format!(
                "{declared_entries} entries exceed limit {}",
                limits.maximum_entries
            )));
        }
        let mut archive = zip::ZipArchive::new(Cursor::new(bytes))
            .map_err(|error| ManufacturingReleaseError::Archive(error.to_string()))?;
        if archive.len() != declared_entries {
            return Err(ManufacturingReleaseError::Archive(
                "duplicate ZIP entry names are forbidden".into(),
            ));
        }
        let mut files = BTreeMap::new();
        let mut total = 0_u64;
        for index in 0..archive.len() {
            let mut entry = archive.by_index(index).map_err(|error| {
                ManufacturingReleaseError::Archive(format!(
                    "cannot read ZIP entry {index}: {error}"
                ))
            })?;
            if entry.is_dir() {
                return Err(ManufacturingReleaseError::Archive(format!(
                    "directory ZIP entry {} is not part of the canonical release form",
                    entry.name()
                )));
            }
            if entry
                .unix_mode()
                .is_some_and(|mode| mode & 0o170000 == 0o120000)
            {
                return Err(ManufacturingReleaseError::Archive(format!(
                    "symbolic-link ZIP entry {} is forbidden",
                    entry.name()
                )));
            }
            let path = PortableArtifactPath::new(entry.name().to_owned())
                .map_err(ManufacturingReleaseError::Catalog)?;
            if entry.size() > limits.maximum_file_bytes {
                return Err(ManufacturingReleaseError::ArchiveLimit(format!(
                    "{} exceeds per-file limit {}",
                    path.as_str(),
                    limits.maximum_file_bytes
                )));
            }
            let compressed = entry.compressed_size();
            if entry.size() > 0
                && (compressed == 0
                    || entry.size() > compressed.saturating_mul(limits.maximum_compression_ratio))
            {
                return Err(ManufacturingReleaseError::ArchiveLimit(format!(
                    "{} exceeds compression-ratio limit {}",
                    path.as_str(),
                    limits.maximum_compression_ratio
                )));
            }
            total = total.checked_add(entry.size()).ok_or_else(|| {
                ManufacturingReleaseError::ArchiveLimit("ZIP total size overflow".into())
            })?;
            if total > limits.maximum_total_bytes {
                return Err(ManufacturingReleaseError::ArchiveLimit(format!(
                    "ZIP content exceeds total limit {}",
                    limits.maximum_total_bytes
                )));
            }
            let mut file = Vec::with_capacity(entry.size().min(usize::MAX as u64) as usize);
            entry
                .by_ref()
                .take(limits.maximum_file_bytes.saturating_add(1))
                .read_to_end(&mut file)
                .map_err(|error| {
                    ManufacturingReleaseError::Archive(format!(
                        "cannot inflate {}: {error}",
                        path.as_str()
                    ))
                })?;
            if file.len() as u64 != entry.size() {
                return Err(ManufacturingReleaseError::Archive(format!(
                    "{} declared {} bytes but yielded {}",
                    path.as_str(),
                    entry.size(),
                    file.len()
                )));
            }
            if files.insert(path.as_str().into(), file).is_some() {
                return Err(ManufacturingReleaseError::Archive(format!(
                    "duplicate ZIP entry {}",
                    path.as_str()
                )));
            }
        }
        let manifest_bytes = files
            .get(MANUFACTURING_RELEASE_MANIFEST_PATH)
            .ok_or_else(|| {
                ManufacturingReleaseError::MissingArtifact(
                    MANUFACTURING_RELEASE_MANIFEST_PATH.into(),
                )
            })?;
        let manifest: ManufacturingReleaseManifest = serde_json::from_slice(manifest_bytes)
            .map_err(|error| ManufacturingReleaseError::ManifestJson(error.to_string()))?;
        if manifest.core.schema != MANUFACTURING_RELEASE_SCHEMA
            || manifest.core.version != MANUFACTURING_RELEASE_VERSION
        {
            return Err(ManufacturingReleaseError::UnsupportedManifest);
        }
        let bundle = Self { manifest, files };
        bundle.verify_integrity()?;
        Ok(bundle)
    }

    /// Writes a release directory after verifying core and artifact integrity.
    ///
    /// Signature trust is checked on read/verification, not while transporting
    /// an envelope that may use a caller-provided signing service.
    pub fn write_directory(&self, root: &Path) -> Result<(), ManufacturingReleaseError> {
        self.verify_integrity()?;
        ensure_safe_directory(root)?;
        for (relative, bytes) in &self.files {
            let destination = root.join(relative);
            if let Some(parent) = destination.parent() {
                ensure_safe_descendant_directories(root, parent)?;
            }
            if fs::symlink_metadata(&destination)
                .ok()
                .is_some_and(|metadata| !metadata.file_type().is_file())
            {
                return Err(ManufacturingReleaseError::Io(format!(
                    "{} is not a regular file",
                    destination.display()
                )));
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
        let bundle = Self::read_directory_integrity(root)?;
        bundle.verify()?;
        Ok(bundle)
    }

    /// Loads a release directory and applies a caller-supplied signature policy.
    pub fn read_directory_with_signature_verifier(
        root: &Path,
        verifier: &dyn ReleaseSignatureVerifier,
    ) -> Result<Self, ManufacturingReleaseError> {
        let bundle = Self::read_directory_integrity(root)?;
        bundle.verify_with_signature_verifier(verifier)?;
        Ok(bundle)
    }

    fn read_directory_integrity(root: &Path) -> Result<Self, ManufacturingReleaseError> {
        let manifest_path = root.join(MANUFACTURING_RELEASE_MANIFEST_PATH);
        let manifest_metadata = fs::symlink_metadata(&manifest_path).map_err(|error| {
            ManufacturingReleaseError::Io(format!(
                "cannot inspect {}: {error}",
                manifest_path.display()
            ))
        })?;
        if !manifest_metadata.file_type().is_file() {
            return Err(ManufacturingReleaseError::Io(format!(
                "{} is not a regular file",
                manifest_path.display()
            )));
        }
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
        bundle.verify_integrity()?;
        Ok(bundle)
    }
}

fn read_archive_bytes(
    path: &Path,
    limits: ReleaseArchiveLimits,
) -> Result<Vec<u8>, ManufacturingReleaseError> {
    let metadata = fs::metadata(path).map_err(|error| {
        ManufacturingReleaseError::Io(format!("cannot inspect {}: {error}", path.display()))
    })?;
    if metadata.len() > limits.maximum_archive_bytes {
        return Err(ManufacturingReleaseError::ArchiveLimit(format!(
            "ZIP container exceeds byte limit {}",
            limits.maximum_archive_bytes
        )));
    }
    fs::read(path).map_err(|error| {
        ManufacturingReleaseError::Io(format!("cannot read {}: {error}", path.display()))
    })
}

fn declared_zip_entries(bytes: &[u8]) -> Result<usize, ManufacturingReleaseError> {
    const EOCD_SIGNATURE: &[u8; 4] = b"PK\x05\x06";
    const ZIP64_LOCATOR_SIGNATURE: &[u8; 4] = b"PK\x06\x07";
    const ZIP64_EOCD_SIGNATURE: &[u8; 4] = b"PK\x06\x06";
    const MAX_EOCD_SEARCH: usize = 65_557;

    let search_start = bytes.len().saturating_sub(MAX_EOCD_SEARCH);
    let eocd = (search_start..bytes.len().saturating_sub(3))
        .rev()
        .find(|offset| &bytes[*offset..*offset + 4] == EOCD_SIGNATURE)
        .ok_or_else(|| ManufacturingReleaseError::Archive("missing ZIP end record".into()))?;
    if eocd + 22 > bytes.len() {
        return Err(ManufacturingReleaseError::Archive(
            "truncated ZIP end record".into(),
        ));
    }
    let comment_length = u16::from_le_bytes([bytes[eocd + 20], bytes[eocd + 21]]) as usize;
    if eocd + 22 + comment_length != bytes.len() {
        return Err(ManufacturingReleaseError::Archive(
            "ZIP end-record comment length is inconsistent".into(),
        ));
    }
    let entries = u16::from_le_bytes([bytes[eocd + 10], bytes[eocd + 11]]);
    if entries != u16::MAX {
        return Ok(entries as usize);
    }
    let locator = eocd
        .checked_sub(20)
        .ok_or_else(|| ManufacturingReleaseError::Archive("missing ZIP64 end locator".into()))?;
    if &bytes[locator..locator + 4] != ZIP64_LOCATOR_SIGNATURE {
        return Err(ManufacturingReleaseError::Archive(
            "missing ZIP64 end locator".into(),
        ));
    }
    let record_offset = u64::from_le_bytes(
        bytes[locator + 8..locator + 16]
            .try_into()
            .expect("fixed ZIP64 locator slice"),
    );
    let record_offset = usize::try_from(record_offset).map_err(|_| {
        ManufacturingReleaseError::Archive("ZIP64 end record offset exceeds platform".into())
    })?;
    if record_offset + 40 > bytes.len()
        || &bytes[record_offset..record_offset + 4] != ZIP64_EOCD_SIGNATURE
    {
        return Err(ManufacturingReleaseError::Archive(
            "invalid ZIP64 end record".into(),
        ));
    }
    usize::try_from(u64::from_le_bytes(
        bytes[record_offset + 32..record_offset + 40]
            .try_into()
            .expect("fixed ZIP64 end-record slice"),
    ))
    .map_err(|_| ManufacturingReleaseError::Archive("ZIP64 entry count exceeds platform".into()))
}

fn ensure_safe_directory(path: &Path) -> Result<(), ManufacturingReleaseError> {
    if let Ok(metadata) = fs::symlink_metadata(path) {
        if !metadata.file_type().is_dir() {
            return Err(ManufacturingReleaseError::Io(format!(
                "{} is not a directory",
                path.display()
            )));
        }
        return Ok(());
    }
    fs::create_dir_all(path).map_err(|error| {
        ManufacturingReleaseError::Io(format!("cannot create {}: {error}", path.display()))
    })?;
    let metadata = fs::symlink_metadata(path).map_err(|error| {
        ManufacturingReleaseError::Io(format!("cannot inspect {}: {error}", path.display()))
    })?;
    if !metadata.file_type().is_dir() {
        return Err(ManufacturingReleaseError::Io(format!(
            "{} is not a directory",
            path.display()
        )));
    }
    Ok(())
}

fn ensure_safe_descendant_directories(
    root: &Path,
    directory: &Path,
) -> Result<(), ManufacturingReleaseError> {
    let relative = directory.strip_prefix(root).map_err(|_| {
        ManufacturingReleaseError::Io(format!(
            "{} is outside release root {}",
            directory.display(),
            root.display()
        ))
    })?;
    let mut current = root.to_path_buf();
    for component in relative.components() {
        current.push(component.as_os_str());
        match fs::symlink_metadata(&current) {
            Ok(metadata) if metadata.file_type().is_dir() => {}
            Ok(_) => {
                return Err(ManufacturingReleaseError::Io(format!(
                    "{} is not a directory",
                    current.display()
                )));
            }
            Err(error) if error.kind() == std::io::ErrorKind::NotFound => {
                fs::create_dir(&current).map_err(|error| {
                    ManufacturingReleaseError::Io(format!(
                        "cannot create {}: {error}",
                        current.display()
                    ))
                })?;
            }
            Err(error) => {
                return Err(ManufacturingReleaseError::Io(format!(
                    "cannot inspect {}: {error}",
                    current.display()
                )));
            }
        }
    }
    Ok(())
}

fn signature_payload(digest: &str) -> Vec<u8> {
    let mut payload = b"hypercircuit.manufacturing-release-signature.v1\0".to_vec();
    payload.extend_from_slice(digest.as_bytes());
    payload
}

fn encode_hex(bytes: &[u8]) -> String {
    const HEX: &[u8; 16] = b"0123456789abcdef";
    let mut encoded = String::with_capacity(bytes.len() * 2);
    for byte in bytes {
        encoded.push(HEX[(byte >> 4) as usize] as char);
        encoded.push(HEX[(byte & 0x0f) as usize] as char);
    }
    encoded
}

fn decode_hex(encoded: &str) -> Result<Vec<u8>, String> {
    if !encoded.len().is_multiple_of(2) {
        return Err("hex value has odd length".into());
    }
    encoded
        .as_bytes()
        .chunks_exact(2)
        .map(|pair| {
            let high = decode_hex_digit(pair[0])?;
            let low = decode_hex_digit(pair[1])?;
            Ok((high << 4) | low)
        })
        .collect()
}

fn decode_hex_digit(value: u8) -> Result<u8, String> {
    match value {
        b'0'..=b'9' => Ok(value - b'0'),
        b'a'..=b'f' => Ok(value - b'a' + 10),
        b'A'..=b'F' => Ok(value - b'A' + 10),
        _ => Err(format!("invalid hex digit {:?}", value as char)),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn archive_reader_rejects_duplicate_names_and_symbolic_links_before_manifest_loading() {
        let options = zip::write::SimpleFileOptions::default();
        let mut duplicate = zip::ZipWriter::new(Cursor::new(Vec::new()));
        duplicate.start_file("same", options).unwrap();
        duplicate.write_all(b"one").unwrap();
        duplicate.start_file("samo", options).unwrap();
        duplicate.write_all(b"two").unwrap();
        let mut duplicate = duplicate.finish().unwrap().into_inner();
        for index in 0..duplicate.len().saturating_sub(3) {
            if &duplicate[index..index + 4] == b"samo" {
                duplicate[index..index + 4].copy_from_slice(b"same");
            }
        }
        assert!(matches!(
            ManufacturingReleaseBundle::from_zip_bytes(&duplicate),
            Err(ManufacturingReleaseError::Archive(detail)) if detail.contains("duplicate")
        ));

        let mut symlink = zip::ZipWriter::new(Cursor::new(Vec::new()));
        symlink.start_file("link", options).unwrap();
        symlink.write_all(b"target").unwrap();
        let mut symlink = symlink.finish().unwrap().into_inner();
        let central = symlink
            .windows(4)
            .position(|window| window == b"PK\x01\x02")
            .unwrap();
        symlink[central + 5] = 3;
        symlink[central + 38..central + 42].copy_from_slice(&((0o120777_u32) << 16).to_le_bytes());
        assert!(matches!(
            ManufacturingReleaseBundle::from_zip_bytes(&symlink),
            Err(ManufacturingReleaseError::Archive(detail)) if detail.contains("symbolic-link")
        ));
    }
}
