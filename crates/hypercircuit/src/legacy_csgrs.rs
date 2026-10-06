//! Versioned reader for retired csgrs electronics-migration handoffs.
//!
//! Live electronics markers have been removed from csgrs. This module retains
//! the hypercircuit-owned persisted schema so previously captured claims remain
//! readable without reintroducing electrical meaning into geometry metadata.

/// Schema family for captured csgrs compatibility metadata.
pub const LEGACY_CSGRS_ELECTRONICS_SCHEMA: &str = "hypercircuit.legacy-csgrs-electronics";

/// Current schema version for captured csgrs compatibility metadata.
pub const LEGACY_CSGRS_ELECTRONICS_VERSION: u32 = 1;

/// HyperCircuit release that removes this migration-only reader.
///
/// Callers must replace every reported omission with explicit HyperCircuit
/// authoring and persist the ordinary semantic document before this release.
pub const LEGACY_CSGRS_ELECTRONICS_REMOVAL_VERSION: &str = "0.4.0";

/// A free-form terminal claim copied from legacy geometry metadata.
///
/// The role is retained verbatim. In particular, `pin` and `pad` do not become
/// a [`crate::DevicePin`] or [`crate::LandPatternPad`] without explicit caller
/// authoring.
#[derive(Clone, Debug, Eq, PartialEq, serde::Deserialize, serde::Serialize)]
pub struct LegacyCsgrsTerminalClaim {
    /// Stable geometry-side terminal handle.
    pub handle: String,
    /// Human-readable legacy terminal name.
    pub name: String,
    /// Uninterpreted legacy role text.
    pub role: String,
}

/// Semantic information that cannot safely be recovered from csgrs metadata.
#[derive(Clone, Debug, Eq, PartialEq, serde::Deserialize, serde::Serialize)]
pub enum LegacyCsgrsElectronicsOmission {
    /// A package marker does not describe pad geometry or a pin-to-pad map.
    LandPatternRequiresAuthoring { aspect: String },
    /// An electrical marker does not describe a typed device interface.
    DeviceModelRequiresAuthoring { aspect: String },
    /// A free-form terminal role cannot establish electrical identity.
    TerminalRequiresExplicitMapping { terminal: String, role: String },
}

/// Versioned, loss-audited capture of retired csgrs electronics metadata.
///
/// This is deliberately a persisted import report rather than an inferred
/// circuit object. Every missing semantic fact stays visible.
#[derive(Clone, Debug, Eq, PartialEq, serde::Deserialize, serde::Serialize)]
pub struct LegacyCsgrsElectronicsImport {
    /// Schema family.
    pub schema: String,
    /// Schema revision.
    pub version: u32,
    /// Geometry metadata handle.
    pub source_handle: String,
    /// Original part family.
    pub family_id: String,
    /// Original part variant.
    pub variant_id: String,
    /// Handles of deprecated package aspects.
    pub package_aspects: Vec<String>,
    /// Handles of deprecated electrical aspects.
    pub electrical_aspects: Vec<String>,
    /// Verbatim compatibility terminal claims.
    pub terminal_claims: Vec<LegacyCsgrsTerminalClaim>,
    /// Facts requiring explicit circuit/PCB authoring.
    pub omissions: Vec<LegacyCsgrsElectronicsOmission>,
}

/// Invalid persisted legacy-metadata handoff.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum LegacyCsgrsElectronicsImportError {
    /// The document is not the hypercircuit legacy-csgrs schema.
    Schema(String),
    /// The document uses a newer or otherwise unsupported revision.
    Version(u32),
    /// The JSON document is malformed or structurally invalid.
    Json(String),
}

impl LegacyCsgrsElectronicsImport {
    /// Whether every captured compatibility claim has been replaced by
    /// explicit hypercircuit authoring.
    ///
    /// Version 1 never infers those replacements, so callers establish
    /// completion by consuming and resolving every omission.
    pub fn requires_review(&self) -> bool {
        !self.omissions.is_empty()
    }

    /// Serializes the captured compatibility claims for migration storage.
    pub fn to_json(&self) -> Result<String, LegacyCsgrsElectronicsImportError> {
        serde_json::to_string_pretty(self)
            .map_err(|error| LegacyCsgrsElectronicsImportError::Json(error.to_string()))
    }

    /// Loads a persisted compatibility handoff after checking its schema and
    /// exact supported revision.
    pub fn from_json(json: &str) -> Result<Self, LegacyCsgrsElectronicsImportError> {
        let imported: Self = serde_json::from_str(json)
            .map_err(|error| LegacyCsgrsElectronicsImportError::Json(error.to_string()))?;
        if imported.schema != LEGACY_CSGRS_ELECTRONICS_SCHEMA {
            return Err(LegacyCsgrsElectronicsImportError::Schema(imported.schema));
        }
        if imported.version != LEGACY_CSGRS_ELECTRONICS_VERSION {
            return Err(LegacyCsgrsElectronicsImportError::Version(imported.version));
        }
        Ok(imported)
    }
}
