//! Canonical manufacturing-release artifact identity and catalogs.

use std::collections::BTreeSet;
use std::fmt::{Display, Formatter};

use serde::{Deserialize, Serialize};
use unicode_normalization::UnicodeNormalization;

use crate::PackageDigest;

/// Typed role of one file in a manufacturing release.
#[derive(Clone, Debug, Eq, Ord, PartialEq, PartialOrd, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ReleaseArtifactRole {
    FabricationManifest,
    Gerber,
    GerberJob,
    Drill,
    Netlist,
    Bom,
    PickAndPlace,
    AssemblyData,
    DrcReport,
    SemanticDocument,
    Sarif,
    PanelDefinition,
    TestIntent,
    Drawing,
    Readme,
    Other(String),
}

/// Rejected portable artifact path or catalog.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum ArtifactCatalogError {
    InvalidPath(String),
    DuplicatePath(String),
}

impl Display for ArtifactCatalogError {
    fn fmt(&self, formatter: &mut Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::InvalidPath(path) => write!(formatter, "invalid portable artifact path {path:?}"),
            Self::DuplicatePath(path) => write!(formatter, "duplicate artifact path {path:?}"),
        }
    }
}

impl std::error::Error for ArtifactCatalogError {}

/// Portable normalized relative UTF-8 path.
#[derive(Clone, Debug, Eq, Ord, PartialEq, PartialOrd, Serialize)]
#[serde(transparent)]
pub struct PortableArtifactPath(String);

impl<'de> Deserialize<'de> for PortableArtifactPath {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        let path = String::deserialize(deserializer)?;
        Self::new(path).map_err(serde::de::Error::custom)
    }
}

impl PortableArtifactPath {
    /// Validates and retains an already-normalized portable path.
    pub fn new(path: impl Into<String>) -> Result<Self, ArtifactCatalogError> {
        let path = path.into();
        let normalized = path.nfc().collect::<String>();
        let invalid = path.is_empty()
            || normalized != path
            || path.starts_with('/')
            || path.starts_with('\\')
            || path.contains('\\')
            || path.contains('\0')
            || path.split('/').any(|part| {
                part.is_empty()
                    || part == "."
                    || part == ".."
                    || part.trim() != part
                    || (part.len() == 2
                        && part.as_bytes()[0].is_ascii_alphabetic()
                        && part.as_bytes()[1] == b':')
            });
        if invalid {
            return Err(ArtifactCatalogError::InvalidPath(path));
        }
        Ok(Self(path))
    }

    /// Returns the normalized portable display form.
    pub fn as_str(&self) -> &str {
        &self.0
    }
}

/// Immutable identity of one exact artifact byte stream.
#[derive(Clone, Debug, Eq, Ord, PartialEq, PartialOrd, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ReleaseArtifactDescriptor {
    pub path: PortableArtifactPath,
    pub role: ReleaseArtifactRole,
    pub byte_length: u64,
    pub digest: PackageDigest,
}

impl ReleaseArtifactDescriptor {
    /// Describes exact bytes with canonical SHA-256 identity.
    pub fn from_bytes(
        path: impl Into<String>,
        role: ReleaseArtifactRole,
        bytes: &[u8],
    ) -> Result<Self, ArtifactCatalogError> {
        Ok(Self {
            path: PortableArtifactPath::new(path)?,
            role,
            byte_length: bytes.len() as u64,
            digest: PackageDigest::sha256(bytes),
        })
    }

    /// True when size and exact-byte digest still match this descriptor.
    pub fn verifies(&self, bytes: &[u8]) -> bool {
        self.byte_length == bytes.len() as u64 && self.digest == PackageDigest::sha256(bytes)
    }
}

/// Deterministically ordered complete release artifact inventory.
#[derive(Clone, Debug, Default, Eq, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ArtifactCatalog {
    pub artifacts: Vec<ReleaseArtifactDescriptor>,
}

impl ArtifactCatalog {
    /// Sorts descriptors by path/role and rejects duplicate paths.
    pub fn new(
        mut artifacts: Vec<ReleaseArtifactDescriptor>,
    ) -> Result<Self, ArtifactCatalogError> {
        artifacts.sort_by(|left, right| (&left.path, &left.role).cmp(&(&right.path, &right.role)));
        let mut paths = BTreeSet::new();
        for artifact in &artifacts {
            if !paths.insert(artifact.path.clone()) {
                return Err(ArtifactCatalogError::DuplicatePath(
                    artifact.path.as_str().into(),
                ));
            }
        }
        Ok(Self { artifacts })
    }

    /// Deterministic catalog bytes used by the manufacturing manifest.
    pub fn canonical_bytes(&self) -> Vec<u8> {
        serde_json::to_vec(self).expect("artifact catalogs are infallibly serializable")
    }

    /// Digest of the deterministic descriptor catalog.
    pub fn digest(&self) -> PackageDigest {
        PackageDigest::sha256(&self.canonical_bytes())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn insertion_order_does_not_change_catalog_bytes_or_digest() {
        let left = ReleaseArtifactDescriptor::from_bytes(
            "fab/top.gbr",
            ReleaseArtifactRole::Gerber,
            b"top",
        )
        .unwrap();
        let right = ReleaseArtifactDescriptor::from_bytes(
            "assembly/bom.csv",
            ReleaseArtifactRole::Bom,
            b"bom",
        )
        .unwrap();
        let a = ArtifactCatalog::new(vec![left.clone(), right.clone()]).unwrap();
        let b = ArtifactCatalog::new(vec![right, left]).unwrap();

        assert_eq!(a.canonical_bytes(), b.canonical_bytes());
        assert_eq!(a.digest(), b.digest());
    }

    #[test]
    fn path_traversal_duplicates_and_byte_mutation_are_rejected_or_detected() {
        assert!(PortableArtifactPath::new("../secret").is_err());
        assert!(serde_json::from_str::<PortableArtifactPath>("\"../secret\"").is_err());
        assert!(PortableArtifactPath::new("C:/secret").is_err());
        assert!(PortableArtifactPath::new("fab\\top.gbr").is_err());
        assert!(PortableArtifactPath::new("fab/cafe\u{301}.gbr").is_err());
        assert!(PortableArtifactPath::new("fab/caf\u{e9}.gbr").is_ok());
        let descriptor = ReleaseArtifactDescriptor::from_bytes(
            "fab/top.gbr",
            ReleaseArtifactRole::Gerber,
            b"top",
        )
        .unwrap();
        assert!(!descriptor.verifies(b"Top"));
        assert!(ArtifactCatalog::new(vec![descriptor.clone(), descriptor]).is_err());
    }
}
