//! Deterministic package catalog, semver resolution, lock, and provenance.
//!
//! This module resolves declarative circuit/layout library artifacts. With
//! `interchange`, it also owns a versioned portable part-library artifact and a
//! content-addressed filesystem store. Network registry transport remains an
//! adapter boundary; verified artifacts themselves are first-class.

use std::collections::{BTreeMap, BTreeSet};
use std::fmt::{Display, Formatter};
#[cfg(feature = "interchange")]
use std::fs::{self, OpenOptions};
#[cfg(feature = "interchange")]
use std::io::Write;
#[cfg(feature = "interchange")]
use std::path::{Path, PathBuf};

use semver::{Version, VersionReq};
#[cfg(feature = "interchange")]
use sha2::{Digest, Sha256};

use crate::CircuitPackageName;
#[cfg(feature = "interchange")]
use crate::{
    AdapterKind, Circuit, CircuitId, DeviceModel, LandPattern, PartRef, SchematicLayout,
    SchematicSymbolDefinition, TransientPolicy,
};

/// Stable JSON lockfile schema identity.
pub const CIRCUIT_PACKAGE_LOCK_SCHEMA: &str = "hypercircuit.package-lock";
/// Current JSON lockfile schema version.
pub const CIRCUIT_PACKAGE_LOCK_VERSION: u32 = 1;
/// Stable portable part-library artifact schema identity.
#[cfg(feature = "interchange")]
pub const PART_LIBRARY_ARTIFACT_SCHEMA: &str = "hypercircuit.part-library";
/// Current portable part-library artifact schema version.
#[cfg(feature = "interchange")]
pub const PART_LIBRARY_ARTIFACT_VERSION: u32 = 1;

/// Reproducible origin of one circuit-library package artifact.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub enum PackageSource {
    /// Named package registry and immutable release coordinate.
    Registry(String),
    /// Git repository pinned to an immutable revision supplied by the caller.
    Git {
        /// Repository URL or workspace handle.
        repository: String,
        /// Commit/revision pin; moving branch names are not reproducible.
        revision: String,
    },
    /// Local/workspace source. Reproducibility still requires a content digest.
    Path(String),
}

/// Content digest retained independently from an artifact source locator.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub struct PackageDigest {
    /// Digest algorithm, such as `sha256` or `blake3`.
    pub algorithm: String,
    /// Canonical digest text supplied or verified by the artifact loader.
    pub value: String,
}

impl PackageDigest {
    /// Constructs nonempty provenance evidence.
    pub fn new(
        algorithm: impl Into<String>,
        value: impl Into<String>,
    ) -> Result<Self, PackageResolutionError> {
        let result = Self {
            algorithm: algorithm.into(),
            value: value.into(),
        };
        if result.algorithm.trim().is_empty() || result.value.trim().is_empty() {
            return Err(PackageResolutionError::InvalidDigest);
        }
        Ok(result)
    }
}

/// Public semantic artifact exposed by a package.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub enum CircuitPackageExportKind {
    /// Unified reusable part definition.
    PartDefinition,
    /// Reusable hierarchical circuit definition.
    Circuit,
    /// Electrical/simulation device model.
    DeviceModel,
    /// Reusable PCB land pattern.
    LandPattern,
    /// Reusable schematic symbol or symbol unit.
    SchematicSymbol,
    /// Source-specific extension whose meaning remains retained.
    Custom(String),
}

/// One named semantic export in a package artifact.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub struct CircuitPackageExport {
    /// Artifact family.
    pub kind: CircuitPackageExportKind,
    /// Package-local stable export name.
    pub name: String,
}

/// Direct or transitive semver package requirement.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, PartialEq)]
pub struct PackageRequirement {
    /// Requested package.
    pub name: CircuitPackageName,
    /// Semver compatibility range.
    pub version: VersionReq,
    /// Optional required source; `None` accepts any catalog source.
    pub source: Option<PackageSource>,
}

impl PackageRequirement {
    /// Parses a semver requirement for one package.
    pub fn parse(
        name: CircuitPackageName,
        version: &str,
        source: Option<PackageSource>,
    ) -> Result<Self, PackageResolutionError> {
        Ok(Self {
            name,
            version: VersionReq::parse(version)
                .map_err(|_| PackageResolutionError::InvalidVersionRequirement(version.into()))?,
            source,
        })
    }
}

/// One immutable package release available to resolution.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, PartialEq)]
pub struct CircuitPackageRelease {
    /// Package identity.
    pub name: CircuitPackageName,
    /// Exact semantic version.
    pub version: Version,
    /// Artifact origin.
    pub source: PackageSource,
    /// Verified or expected content digest.
    pub digest: PackageDigest,
    /// Public semantic exports in the artifact.
    pub exports: Vec<CircuitPackageExport>,
    /// Transitive package requirements.
    pub dependencies: Vec<PackageRequirement>,
}

/// In-memory available-release catalog; registry transport remains caller-owned.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, Default, PartialEq)]
pub struct CircuitPackageCatalog {
    /// Available immutable releases.
    pub releases: Vec<CircuitPackageRelease>,
}

/// One unified portable electrical, schematic, and PCB part definition.
///
/// The retained model, symbol definition, and land pattern remain the same
/// authoritative structs used by a design; this wrapper only binds them into
/// one package export.
#[cfg(feature = "interchange")]
#[derive(Clone, Debug, PartialEq, serde::Deserialize, serde::Serialize)]
#[serde(deny_unknown_fields)]
pub struct PortablePartDefinition {
    /// Package-local export name.
    pub name: String,
    /// Optional stable external procurement/library identity.
    pub part: Option<PartRef>,
    /// Reusable electrical interface and simulation model.
    pub model: DeviceModel,
    /// Optional reusable multipart schematic drawing.
    pub symbol: Option<SchematicSymbolDefinition>,
    /// Optional reusable physical land pattern and pin map.
    pub land_pattern: Option<LandPattern>,
}

#[cfg(feature = "interchange")]
impl PortablePartDefinition {
    /// Validates cross-domain identities and reusable pin coverage.
    pub fn validate(&self) -> Result<(), PackageResolutionError> {
        if self.name.trim().is_empty() {
            return Err(PackageResolutionError::InvalidArtifact(
                "part export name is empty".into(),
            ));
        }
        let circuit = Circuit::new(
            CircuitId::new("part-library-validation")
                .expect("constant validation circuit id is nonempty"),
            TransientPolicy::Static,
            AdapterKind::Dc,
        )
        .with_device_model(self.model.clone());
        let circuit_report = circuit.validate();
        if !circuit_report.is_valid() {
            return Err(PackageResolutionError::InvalidArtifact(format!(
                "part {} has {} device-model validation issue(s)",
                self.name,
                circuit_report.issues.len()
            )));
        }
        let model_pins = self
            .model
            .pins
            .iter()
            .map(|pin| pin.pin.clone())
            .collect::<BTreeSet<_>>();
        if let Some(symbol) = &self.symbol {
            if symbol.model != self.model.id {
                return Err(PackageResolutionError::InvalidArtifact(format!(
                    "part {} symbol targets a different device model",
                    self.name
                )));
            }
            let schematic = SchematicLayout {
                symbol_definitions: vec![symbol.clone()],
                ..SchematicLayout::default()
            };
            let report = schematic.validate(&circuit);
            if !report.is_valid() {
                return Err(PackageResolutionError::InvalidArtifact(format!(
                    "part {} has {} symbol validation issue(s)",
                    self.name,
                    report.issues.len()
                )));
            }
            let presented = symbol
                .units
                .iter()
                .flat_map(|unit| unit.pins.iter().map(|pin| pin.pin.clone()))
                .collect::<BTreeSet<_>>();
            if let Some(pin) = self
                .model
                .pins
                .iter()
                .find(|pin| !pin.optional && !presented.contains(&pin.pin))
            {
                return Err(PackageResolutionError::InvalidArtifact(format!(
                    "part {} symbol omits required pin {}",
                    self.name,
                    pin.pin.as_str()
                )));
            }
        }
        if let Some(pattern) = &self.land_pattern {
            let mut pads = BTreeSet::new();
            for pad in &pattern.pads {
                if !pads.insert(pad.id.clone()) {
                    return Err(PackageResolutionError::InvalidArtifact(format!(
                        "part {} land pattern repeats pad {}",
                        self.name,
                        pad.id.as_str()
                    )));
                }
            }
            let mut mappings = BTreeSet::new();
            for mapping in &pattern.pin_map {
                if !model_pins.contains(&mapping.pin) {
                    return Err(PackageResolutionError::InvalidArtifact(format!(
                        "part {} maps unknown pin {}",
                        self.name,
                        mapping.pin.as_str()
                    )));
                }
                if !pads.contains(&mapping.pad) {
                    return Err(PackageResolutionError::InvalidArtifact(format!(
                        "part {} maps absent pad {}",
                        self.name,
                        mapping.pad.as_str()
                    )));
                }
                if !mappings.insert((mapping.pin.clone(), mapping.pad.clone())) {
                    return Err(PackageResolutionError::InvalidArtifact(format!(
                        "part {} repeats pin-to-pad mapping {}:{}",
                        self.name,
                        mapping.pin.as_str(),
                        mapping.pad.as_str()
                    )));
                }
            }
            if let Some(pin) = self.model.pins.iter().find(|pin| {
                !pin.optional && !pattern.pin_map.iter().any(|mapping| mapping.pin == pin.pin)
            }) {
                return Err(PackageResolutionError::InvalidArtifact(format!(
                    "part {} land pattern omits required pin {}",
                    self.name,
                    pin.pin.as_str()
                )));
            }
        }
        Ok(())
    }
}

/// Versioned package artifact containing reusable part definitions.
#[cfg(feature = "interchange")]
#[derive(Clone, Debug, PartialEq, serde::Deserialize, serde::Serialize)]
#[serde(deny_unknown_fields)]
pub struct PartLibraryArtifact {
    /// Stable schema discriminator.
    pub schema: String,
    /// Exact schema revision.
    pub schema_version: u32,
    /// Package identity.
    pub package: CircuitPackageName,
    /// Exact package release version.
    pub version: Version,
    /// Direct package requirements retained in the release metadata.
    pub dependencies: Vec<PackageRequirement>,
    /// Named reusable part exports.
    pub parts: Vec<PortablePartDefinition>,
}

#[cfg(feature = "interchange")]
impl PartLibraryArtifact {
    /// Creates the current artifact schema and validates all definitions.
    pub fn new(
        package: CircuitPackageName,
        version: Version,
        parts: Vec<PortablePartDefinition>,
    ) -> Result<Self, PackageResolutionError> {
        let artifact = Self {
            schema: PART_LIBRARY_ARTIFACT_SCHEMA.into(),
            schema_version: PART_LIBRARY_ARTIFACT_VERSION,
            package,
            version,
            dependencies: Vec::new(),
            parts,
        };
        artifact.validate()?;
        Ok(artifact)
    }

    /// Replaces direct dependency requirements and revalidates the artifact.
    pub fn with_dependencies(
        mut self,
        dependencies: Vec<PackageRequirement>,
    ) -> Result<Self, PackageResolutionError> {
        self.dependencies = dependencies;
        self.validate()?;
        Ok(self)
    }

    /// Validates schema, export identities, and every bundled definition.
    pub fn validate(&self) -> Result<(), PackageResolutionError> {
        if self.schema != PART_LIBRARY_ARTIFACT_SCHEMA
            || self.schema_version != PART_LIBRARY_ARTIFACT_VERSION
        {
            return Err(PackageResolutionError::UnsupportedArtifactSchema);
        }
        if self.parts.is_empty() {
            return Err(PackageResolutionError::InvalidArtifact(
                "part library has no exports".into(),
            ));
        }
        let mut names = BTreeSet::new();
        let mut models = BTreeSet::new();
        let mut symbols = BTreeSet::new();
        let mut patterns = BTreeSet::new();
        for part in &self.parts {
            part.validate()?;
            if !names.insert(part.name.clone()) {
                return Err(PackageResolutionError::InvalidArtifact(format!(
                    "duplicate part export {}",
                    part.name
                )));
            }
            if !models.insert(part.model.id.clone()) {
                return Err(PackageResolutionError::InvalidArtifact(format!(
                    "duplicate device model {}",
                    part.model.id.as_str()
                )));
            }
            if let Some(symbol) = &part.symbol
                && !symbols.insert(symbol.id.clone())
            {
                return Err(PackageResolutionError::InvalidArtifact(format!(
                    "duplicate symbol definition {}",
                    symbol.id.as_str()
                )));
            }
            if let Some(pattern) = &part.land_pattern
                && !patterns.insert(pattern.id.clone())
            {
                return Err(PackageResolutionError::InvalidArtifact(format!(
                    "duplicate land pattern {}",
                    pattern.id.as_str()
                )));
            }
        }
        let mut dependency_names = BTreeSet::new();
        for dependency in &self.dependencies {
            if dependency.name == self.package || !dependency_names.insert(dependency.name.clone())
            {
                return Err(PackageResolutionError::InvalidArtifact(format!(
                    "invalid or duplicate dependency {}",
                    dependency.name.as_str()
                )));
            }
        }
        Ok(())
    }

    /// Returns deterministic compact JSON bytes used for hashing and storage.
    pub fn canonical_bytes(&self) -> Result<Vec<u8>, PackageResolutionError> {
        self.validate()?;
        serde_json::to_vec(self)
            .map_err(|error| PackageResolutionError::ArtifactJson(error.to_string()))
    }

    /// Returns human-readable JSON with the same semantic content.
    pub fn to_json_pretty(&self) -> Result<String, PackageResolutionError> {
        self.validate()?;
        serde_json::to_string_pretty(self)
            .map_err(|error| PackageResolutionError::ArtifactJson(error.to_string()))
    }

    /// Parses and validates one portable library artifact.
    pub fn from_json(input: &str) -> Result<Self, PackageResolutionError> {
        let artifact: Self = serde_json::from_str(input)
            .map_err(|error| PackageResolutionError::ArtifactJson(error.to_string()))?;
        artifact.validate()?;
        Ok(artifact)
    }

    /// Computes the canonical SHA-256 content identity.
    pub fn digest(&self) -> Result<PackageDigest, PackageResolutionError> {
        let bytes = self.canonical_bytes()?;
        Ok(PackageDigest {
            algorithm: "sha256".into(),
            value: format!("{:x}", Sha256::digest(bytes)),
        })
    }

    /// Produces resolver catalog metadata for this exact artifact.
    pub fn release(
        &self,
        source: PackageSource,
    ) -> Result<CircuitPackageRelease, PackageResolutionError> {
        Ok(CircuitPackageRelease {
            name: self.package.clone(),
            version: self.version.clone(),
            source,
            digest: self.digest()?,
            exports: self
                .parts
                .iter()
                .map(|part| CircuitPackageExport {
                    kind: CircuitPackageExportKind::PartDefinition,
                    name: part.name.clone(),
                })
                .collect(),
            dependencies: self.dependencies.clone(),
        })
    }

    /// Selects one named portable part export.
    pub fn part(&self, name: &str) -> Option<&PortablePartDefinition> {
        self.parts.iter().find(|part| part.name == name)
    }
}

/// Result of publishing an immutable artifact into a local store.
#[cfg(feature = "interchange")]
#[derive(Clone, Debug, PartialEq)]
pub struct PublishedPartLibrary {
    /// Resolver metadata for the stored artifact.
    pub release: CircuitPackageRelease,
    /// Exact content-addressed file.
    pub path: PathBuf,
    /// `true` when this call created the file; `false` for an identical hit.
    pub created: bool,
}

/// Content-addressed filesystem cache for portable part-library artifacts.
#[cfg(feature = "interchange")]
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct CircuitPackageStore {
    root: PathBuf,
}

#[cfg(feature = "interchange")]
impl CircuitPackageStore {
    /// Uses `root` as an explicit package cache/publish directory.
    pub fn new(root: impl Into<PathBuf>) -> Self {
        Self { root: root.into() }
    }

    /// Returns the configured store root.
    pub fn root(&self) -> &Path {
        &self.root
    }

    /// Publishes an immutable artifact or confirms an identical existing hit.
    pub fn publish(
        &self,
        artifact: &PartLibraryArtifact,
        source: PackageSource,
    ) -> Result<PublishedPartLibrary, PackageResolutionError> {
        let release = artifact.release(source)?;
        let bytes = artifact.canonical_bytes()?;
        let path = self.artifact_path(&release.name, &release.version, &release.digest)?;
        let Some(parent) = path.parent() else {
            return Err(PackageResolutionError::ArtifactIo(
                "artifact path has no parent".into(),
            ));
        };
        fs::create_dir_all(parent)
            .map_err(|error| PackageResolutionError::ArtifactIo(error.to_string()))?;
        let created = match OpenOptions::new().write(true).create_new(true).open(&path) {
            Ok(mut file) => {
                if let Err(error) = file.write_all(&bytes).and_then(|()| file.sync_all()) {
                    let _ = fs::remove_file(&path);
                    return Err(PackageResolutionError::ArtifactIo(error.to_string()));
                }
                true
            }
            Err(error) if error.kind() == std::io::ErrorKind::AlreadyExists => {
                let existing = fs::read(&path)
                    .map_err(|read| PackageResolutionError::ArtifactIo(read.to_string()))?;
                if existing != bytes {
                    return Err(PackageResolutionError::ArtifactDigestCollision);
                }
                false
            }
            Err(error) => return Err(PackageResolutionError::ArtifactIo(error.to_string())),
        };
        Ok(PublishedPartLibrary {
            release,
            path,
            created,
        })
    }

    /// Loads and verifies the exact artifact selected by a lock coordinate.
    pub fn load(
        &self,
        locked: &LockedCircuitPackage,
    ) -> Result<PartLibraryArtifact, PackageResolutionError> {
        let path = self.artifact_path(&locked.name, &locked.version, &locked.digest)?;
        let bytes = fs::read(path)
            .map_err(|error| PackageResolutionError::ArtifactIo(error.to_string()))?;
        let actual = PackageDigest {
            algorithm: "sha256".into(),
            value: format!("{:x}", Sha256::digest(&bytes)),
        };
        if actual != locked.digest {
            return Err(PackageResolutionError::DigestMismatch(locked.name.clone()));
        }
        let artifact: PartLibraryArtifact = serde_json::from_slice(&bytes)
            .map_err(|error| PackageResolutionError::ArtifactJson(error.to_string()))?;
        artifact.validate()?;
        if artifact.canonical_bytes()? != bytes {
            return Err(PackageResolutionError::InvalidArtifact(
                "stored artifact is not in canonical encoding".into(),
            ));
        }
        if artifact.package != locked.name || artifact.version != locked.version {
            return Err(PackageResolutionError::ArtifactCoordinateMismatch);
        }
        Ok(artifact)
    }

    /// Loads every package in deterministic lock order after schema checks.
    pub fn load_lock(
        &self,
        lock: &CircuitPackageLock,
    ) -> Result<Vec<PartLibraryArtifact>, PackageResolutionError> {
        if lock.schema != CIRCUIT_PACKAGE_LOCK_SCHEMA
            || lock.schema_version != CIRCUIT_PACKAGE_LOCK_VERSION
        {
            return Err(PackageResolutionError::UnsupportedLockfileSchema);
        }
        let mut names = BTreeSet::new();
        for package in &lock.packages {
            if !names.insert(package.name.clone()) {
                return Err(PackageResolutionError::DuplicateLockedPackage(
                    package.name.clone(),
                ));
            }
        }
        lock.packages
            .iter()
            .map(|package| self.load(package))
            .collect()
    }

    /// Verifies resolver coordinates and dependencies before loading artifacts.
    pub fn load_verified_lock(
        &self,
        catalog: &CircuitPackageCatalog,
        lock: &CircuitPackageLock,
    ) -> Result<Vec<PartLibraryArtifact>, PackageResolutionError> {
        catalog.verify_lock(lock)?;
        self.load_lock(lock)
    }

    fn artifact_path(
        &self,
        package: &CircuitPackageName,
        version: &Version,
        digest: &PackageDigest,
    ) -> Result<PathBuf, PackageResolutionError> {
        if digest.algorithm != "sha256"
            || digest.value.len() != 64
            || !digest.value.bytes().all(|byte| byte.is_ascii_hexdigit())
        {
            return Err(PackageResolutionError::UnsupportedArtifactDigest);
        }
        Ok(self
            .root
            .join(hex_component(package.as_str()))
            .join(version.to_string())
            .join(format!("{}.json", digest.value.to_ascii_lowercase())))
    }
}

#[cfg(feature = "interchange")]
fn hex_component(value: &str) -> String {
    let mut encoded = String::with_capacity(value.len() * 2);
    for byte in value.as_bytes() {
        use std::fmt::Write as _;
        write!(&mut encoded, "{byte:02x}").expect("writing to String cannot fail");
    }
    encoded
}

/// Exact package coordinate recorded in a lockfile.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct LockedCircuitPackage {
    /// Package identity.
    pub name: CircuitPackageName,
    /// Exact selected semantic version.
    pub version: Version,
    /// Exact selected source.
    pub source: PackageSource,
    /// Expected artifact content digest.
    pub digest: PackageDigest,
}

/// Deterministically ordered dependency lock and artifact provenance evidence.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct CircuitPackageLock {
    /// Stable schema identity.
    pub schema: String,
    /// Schema version for explicit future migrations.
    pub schema_version: u32,
    /// Selected packages sorted by stable package identity.
    pub packages: Vec<LockedCircuitPackage>,
}

impl Default for CircuitPackageLock {
    fn default() -> Self {
        Self {
            schema: CIRCUIT_PACKAGE_LOCK_SCHEMA.into(),
            schema_version: CIRCUIT_PACKAGE_LOCK_VERSION,
            packages: Vec::new(),
        }
    }
}

/// Resolution or lock-verification failure.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum PackageResolutionError {
    /// Digest algorithm/value was empty.
    InvalidDigest,
    /// A semver requirement could not be parsed.
    InvalidVersionRequirement(String),
    /// Catalog has two releases at the same name/version/source coordinate.
    DuplicateRelease {
        /// Duplicated package.
        name: CircuitPackageName,
        /// Duplicated exact version.
        version: Version,
    },
    /// No mutually compatible release set exists for a package.
    Unsatisfied(CircuitPackageName),
    /// A lock has two coordinates for one package identity.
    DuplicateLockedPackage(CircuitPackageName),
    /// Locked coordinate is absent from the catalog.
    MissingLockedRelease(CircuitPackageName),
    /// Locked content evidence does not match the catalog release.
    DigestMismatch(CircuitPackageName),
    /// A locked release's dependency is absent or outside its semver/source range.
    LockedDependencyMismatch {
        /// Package declaring the dependency.
        package: CircuitPackageName,
        /// Unsatisfied dependency.
        dependency: CircuitPackageName,
    },
    /// JSON lockfile serialization or parsing failed.
    #[cfg(feature = "interchange")]
    LockfileJson(String),
    /// Lockfile schema identity or version is unsupported.
    UnsupportedLockfileSchema,
    /// Portable artifact JSON serialization or parsing failed.
    #[cfg(feature = "interchange")]
    ArtifactJson(String),
    /// Portable artifact schema identity or version is unsupported.
    #[cfg(feature = "interchange")]
    UnsupportedArtifactSchema,
    /// Portable artifact semantic validation failed.
    #[cfg(feature = "interchange")]
    InvalidArtifact(String),
    /// Artifact I/O failed.
    #[cfg(feature = "interchange")]
    ArtifactIo(String),
    /// Store supports only canonical SHA-256 artifact coordinates.
    #[cfg(feature = "interchange")]
    UnsupportedArtifactDigest,
    /// Stored bytes occupied a canonical digest path but differed.
    #[cfg(feature = "interchange")]
    ArtifactDigestCollision,
    /// Stored package name/version disagreed with the lock coordinate.
    #[cfg(feature = "interchange")]
    ArtifactCoordinateMismatch,
}

impl Display for PackageResolutionError {
    fn fmt(&self, formatter: &mut Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::InvalidDigest => formatter.write_str("package digest is empty"),
            Self::InvalidVersionRequirement(requirement) => {
                write!(formatter, "invalid version requirement {requirement}")
            }
            Self::DuplicateRelease { name, version } => write!(
                formatter,
                "duplicate package release {}@{version}",
                name.as_str()
            ),
            Self::Unsatisfied(name) => {
                write!(formatter, "package {} cannot be resolved", name.as_str())
            }
            Self::DuplicateLockedPackage(name) => {
                write!(formatter, "package lock repeats {}", name.as_str())
            }
            Self::MissingLockedRelease(name) => {
                write!(formatter, "locked package {} is absent", name.as_str())
            }
            Self::DigestMismatch(name) => {
                write!(formatter, "package {} digest does not match", name.as_str())
            }
            Self::LockedDependencyMismatch {
                package,
                dependency,
            } => write!(
                formatter,
                "package {} has an unsatisfied locked dependency {}",
                package.as_str(),
                dependency.as_str()
            ),
            #[cfg(feature = "interchange")]
            Self::LockfileJson(message) => {
                write!(formatter, "invalid package lock JSON: {message}")
            }
            Self::UnsupportedLockfileSchema => {
                formatter.write_str("unsupported package lock schema")
            }
            #[cfg(feature = "interchange")]
            Self::ArtifactJson(message) => {
                write!(formatter, "invalid part artifact JSON: {message}")
            }
            #[cfg(feature = "interchange")]
            Self::UnsupportedArtifactSchema => {
                formatter.write_str("unsupported part artifact schema")
            }
            #[cfg(feature = "interchange")]
            Self::InvalidArtifact(message) => write!(formatter, "invalid part artifact: {message}"),
            #[cfg(feature = "interchange")]
            Self::ArtifactIo(message) => write!(formatter, "part artifact I/O failed: {message}"),
            #[cfg(feature = "interchange")]
            Self::UnsupportedArtifactDigest => {
                formatter.write_str("unsupported part artifact digest")
            }
            #[cfg(feature = "interchange")]
            Self::ArtifactDigestCollision => {
                formatter.write_str("part artifact digest path contains different bytes")
            }
            #[cfg(feature = "interchange")]
            Self::ArtifactCoordinateMismatch => {
                formatter.write_str("part artifact coordinate does not match its lock")
            }
        }
    }
}

impl std::error::Error for PackageResolutionError {}

impl CircuitPackageCatalog {
    /// Resolves roots and transitive dependencies using deterministic
    /// highest-compatible-version backtracking.
    pub fn resolve(
        &self,
        roots: &[PackageRequirement],
    ) -> Result<CircuitPackageLock, PackageResolutionError> {
        self.validate()?;
        if roots.is_empty() {
            return Ok(CircuitPackageLock::default());
        }
        let mut constraints = BTreeMap::<CircuitPackageName, Vec<PackageRequirement>>::new();
        for requirement in roots {
            constraints
                .entry(requirement.name.clone())
                .or_default()
                .push(requirement.clone());
        }
        let selected = resolve_recursive(self, constraints, BTreeMap::new())
            .ok_or_else(|| first_unsatisfied_name(self, roots))?;
        Ok(CircuitPackageLock {
            schema: CIRCUIT_PACKAGE_LOCK_SCHEMA.into(),
            schema_version: CIRCUIT_PACKAGE_LOCK_VERSION,
            packages: selected
                .into_values()
                .map(|release| LockedCircuitPackage {
                    name: release.name,
                    version: release.version,
                    source: release.source,
                    digest: release.digest,
                })
                .collect(),
        })
    }

    /// Verifies every locked coordinate, digest, and transitive dependency
    /// against this catalog without performing artifact I/O.
    pub fn verify_lock(&self, lock: &CircuitPackageLock) -> Result<(), PackageResolutionError> {
        self.validate()?;
        if lock.schema != CIRCUIT_PACKAGE_LOCK_SCHEMA
            || lock.schema_version != CIRCUIT_PACKAGE_LOCK_VERSION
        {
            return Err(PackageResolutionError::UnsupportedLockfileSchema);
        }
        let mut locked = BTreeMap::new();
        for package in &lock.packages {
            if locked.insert(package.name.clone(), package).is_some() {
                return Err(PackageResolutionError::DuplicateLockedPackage(
                    package.name.clone(),
                ));
            }
        }
        for package in &lock.packages {
            let release = self
                .releases
                .iter()
                .find(|release| {
                    release.name == package.name
                        && release.version == package.version
                        && release.source == package.source
                })
                .ok_or_else(|| {
                    PackageResolutionError::MissingLockedRelease(package.name.clone())
                })?;
            if release.digest != package.digest {
                return Err(PackageResolutionError::DigestMismatch(package.name.clone()));
            }
            for dependency in &release.dependencies {
                let valid = locked
                    .get(&dependency.name)
                    .is_some_and(|candidate| locked_matches(candidate, dependency));
                if !valid {
                    return Err(PackageResolutionError::LockedDependencyMismatch {
                        package: package.name.clone(),
                        dependency: dependency.name.clone(),
                    });
                }
            }
        }
        Ok(())
    }

    fn validate(&self) -> Result<(), PackageResolutionError> {
        let mut coordinates = BTreeSet::new();
        for release in &self.releases {
            let coordinate = (
                release.name.clone(),
                release.version.clone(),
                release.source.clone(),
            );
            if !coordinates.insert(coordinate) {
                return Err(PackageResolutionError::DuplicateRelease {
                    name: release.name.clone(),
                    version: release.version.clone(),
                });
            }
        }
        Ok(())
    }
}

impl CircuitPackageLock {
    /// Serializes a versioned, deterministic JSON lockfile.
    #[cfg(feature = "interchange")]
    pub fn to_json(&self) -> Result<String, PackageResolutionError> {
        serde_json::to_string_pretty(self)
            .map_err(|error| PackageResolutionError::LockfileJson(error.to_string()))
    }

    /// Parses a JSON lockfile and rejects duplicate package identities.
    #[cfg(feature = "interchange")]
    pub fn from_json(input: &str) -> Result<Self, PackageResolutionError> {
        let lock: Self = serde_json::from_str(input)
            .map_err(|error| PackageResolutionError::LockfileJson(error.to_string()))?;
        if lock.schema != CIRCUIT_PACKAGE_LOCK_SCHEMA
            || lock.schema_version != CIRCUIT_PACKAGE_LOCK_VERSION
        {
            return Err(PackageResolutionError::UnsupportedLockfileSchema);
        }
        let mut names = BTreeSet::new();
        for package in &lock.packages {
            if !names.insert(package.name.clone()) {
                return Err(PackageResolutionError::DuplicateLockedPackage(
                    package.name.clone(),
                ));
            }
        }
        Ok(lock)
    }
}

fn resolve_recursive(
    catalog: &CircuitPackageCatalog,
    constraints: BTreeMap<CircuitPackageName, Vec<PackageRequirement>>,
    selected: BTreeMap<CircuitPackageName, CircuitPackageRelease>,
) -> Option<BTreeMap<CircuitPackageName, CircuitPackageRelease>> {
    if selected.iter().any(|(name, release)| {
        constraints
            .get(name)
            .is_some_and(|requirements| !requirements.iter().all(|r| release_matches(release, r)))
    }) {
        return None;
    }
    let unresolved = constraints
        .keys()
        .find(|name| !selected.contains_key(*name))?
        .clone();
    let requirements = &constraints[&unresolved];
    let mut candidates = catalog
        .releases
        .iter()
        .filter(|release| {
            release.name == unresolved
                && requirements
                    .iter()
                    .all(|requirement| release_matches(release, requirement))
        })
        .cloned()
        .collect::<Vec<_>>();
    candidates.sort_by(|left, right| {
        right
            .version
            .cmp(&left.version)
            .then_with(|| left.source.cmp(&right.source))
    });
    for candidate in candidates {
        let mut next_constraints = constraints.clone();
        for dependency in &candidate.dependencies {
            next_constraints
                .entry(dependency.name.clone())
                .or_default()
                .push(dependency.clone());
        }
        let mut next_selected = selected.clone();
        next_selected.insert(unresolved.clone(), candidate);
        if next_constraints
            .keys()
            .all(|name| next_selected.contains_key(name))
        {
            if next_selected.iter().all(|(name, release)| {
                next_constraints[name]
                    .iter()
                    .all(|requirement| release_matches(release, requirement))
            }) {
                return Some(next_selected);
            }
            continue;
        }
        if let Some(solution) = resolve_recursive(catalog, next_constraints, next_selected) {
            return Some(solution);
        }
    }
    None
}

fn release_matches(release: &CircuitPackageRelease, requirement: &PackageRequirement) -> bool {
    release.name == requirement.name
        && requirement.version.matches(&release.version)
        && requirement
            .source
            .as_ref()
            .is_none_or(|source| source == &release.source)
}

fn locked_matches(package: &LockedCircuitPackage, requirement: &PackageRequirement) -> bool {
    package.name == requirement.name
        && requirement.version.matches(&package.version)
        && requirement
            .source
            .as_ref()
            .is_none_or(|source| source == &package.source)
}

fn first_unsatisfied_name(
    catalog: &CircuitPackageCatalog,
    roots: &[PackageRequirement],
) -> PackageResolutionError {
    let name = roots
        .iter()
        .find(|requirement| {
            !catalog
                .releases
                .iter()
                .any(|release| release_matches(release, requirement))
        })
        .or_else(|| roots.first())
        .map(|requirement| requirement.name.clone())
        .unwrap_or_else(|| CircuitPackageName::new("dependency-graph").expect("non-empty"));
    PackageResolutionError::Unsatisfied(name)
}
