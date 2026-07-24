//! Versioned project/provider manifests for reproducible CLI workflows.

use std::collections::BTreeMap;
use std::fmt::{Display, Formatter};
use std::str::FromStr;

use hyperreal::{Real, RealSign};
use serde::{Deserialize, Serialize};

#[cfg(feature = "drc")]
use crate::{
    PCB_LOSS_TANGENT_PROPERTY, PCB_RELATIVE_PERMITTIVITY_PROPERTY, PcbMaterialPropertyLibrary,
};
#[cfg(feature = "drc")]
use hyperphysics::{
    MaterialAssertion, MaterialPropertyGraph, MaterialPropertyKind, MaterialState, PropertyValue,
    SourceSpec,
};

/// Stable schema family for `hypercircuit.toml`.
pub const HYPERCIRCUIT_PROJECT_SCHEMA: &str = "org.hypercircuit.project";

/// Latest project-manifest revision understood by this crate.
pub const HYPERCIRCUIT_PROJECT_VERSION: u32 = 1;

/// Human-facing project metadata and default design selection.
#[derive(Clone, Debug, Eq, PartialEq, Deserialize, Serialize)]
#[serde(deny_unknown_fields, rename_all = "kebab-case")]
pub struct ProjectMetadata {
    /// Stable project name.
    pub name: String,
    /// Caller-owned project release or workspace version.
    pub version: String,
    /// Design selected when a CLI command omits its design name.
    pub default_design: String,
}

/// Supported mechanisms for producing one semantic design document.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Deserialize, Serialize)]
#[serde(rename_all = "kebab-case")]
pub enum ProjectProviderKind {
    /// Invoke an explicit argv vector relative to the manifest directory.
    Command,
    /// Run a binary target from a Cargo package.
    Cargo,
}

/// One named provider that prints a [`crate::SemanticDocument`] as JSON.
#[derive(Clone, Debug, Eq, PartialEq, Deserialize, Serialize)]
#[serde(deny_unknown_fields, rename_all = "kebab-case")]
pub struct ProjectDesignProvider {
    /// Provider execution family.
    pub provider: ProjectProviderKind,
    /// Complete executable and argument vector for a command provider.
    #[serde(default)]
    pub command: Vec<String>,
    /// Cargo package selected by a Cargo provider.
    pub package: Option<String>,
    /// Cargo binary target selected by a Cargo provider.
    pub binary: Option<String>,
    /// Cargo features enabled while running the provider.
    #[serde(default)]
    pub features: Vec<String>,
}

/// One exact, source-attributed PCB dielectric entry available to project releases.
#[derive(Clone, Debug, Eq, PartialEq, Deserialize, Serialize)]
#[serde(deny_unknown_fields, rename_all = "kebab-case")]
pub struct ProjectPcbMaterial {
    /// Stable handle referenced by one or more retained stackup layers.
    pub handle: String,
    /// Exact decimal or rational dimensionless relative permittivity.
    pub relative_permittivity: String,
    /// Exact decimal or rational dimensionless dielectric loss tangent.
    pub loss_tangent: String,
    /// Property-source authority, such as a manufacturer or qualified lab.
    pub source_authority: String,
    /// Datasheet, test report, database record, or other source locator.
    pub source_locator: String,
    /// Optional source revision, publication date, or cache freshness tag.
    pub source_freshness: Option<String>,
    /// Optional shared test condition for the two declared properties.
    pub condition: Option<String>,
}

/// Versioned multi-design project manifest.
#[derive(Clone, Debug, Eq, PartialEq, Deserialize, Serialize)]
#[serde(deny_unknown_fields, rename_all = "kebab-case")]
pub struct ProjectManifest {
    /// Schema-family discriminator.
    pub schema: String,
    /// Exact manifest schema revision.
    pub version: u32,
    /// Project identity and default selection.
    pub project: ProjectMetadata,
    /// Named semantic-document providers.
    pub designs: BTreeMap<String, ProjectDesignProvider>,
    /// Source-attributed PCB dielectric properties available to every design.
    #[serde(default)]
    pub pcb_materials: Vec<ProjectPcbMaterial>,
}

/// Failure to parse or validate project/provider intent.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum ProjectManifestError {
    /// TOML syntax or data-shape failure.
    Toml(String),
    /// The manifest belongs to another schema family or revision.
    UnsupportedSchema { schema: String, version: u32 },
    /// A required name or version is empty.
    EmptyField(&'static str),
    /// The selected design is absent from the provider map.
    UnknownDesign(String),
    /// One design name is empty.
    EmptyDesignName,
    /// A provider's fields disagree with its selected provider kind.
    InvalidProvider { design: String, reason: String },
    /// Two PCB material entries use one stable handle.
    DuplicatePcbMaterial(String),
    /// A PCB material declaration is incomplete or physically invalid.
    InvalidPcbMaterial { material: String, reason: String },
}

impl Display for ProjectManifestError {
    fn fmt(&self, formatter: &mut Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Toml(error) => write!(formatter, "project TOML error: {error}"),
            Self::UnsupportedSchema { schema, version } => {
                write!(formatter, "unsupported project schema {schema}@{version}")
            }
            Self::EmptyField(field) => write!(formatter, "project field `{field}` is empty"),
            Self::UnknownDesign(design) => write!(formatter, "design `{design}` has no provider"),
            Self::EmptyDesignName => formatter.write_str("project contains an empty design name"),
            Self::InvalidProvider { design, reason } => {
                write!(
                    formatter,
                    "design `{design}` has an invalid provider: {reason}"
                )
            }
            Self::DuplicatePcbMaterial(material) => {
                write!(
                    formatter,
                    "PCB material `{material}` is declared more than once"
                )
            }
            Self::InvalidPcbMaterial { material, reason } => {
                write!(formatter, "PCB material `{material}` is invalid: {reason}")
            }
        }
    }
}

impl std::error::Error for ProjectManifestError {}

impl ProjectManifest {
    /// Parses and validates a `hypercircuit.toml` document.
    pub fn from_toml(input: &str) -> Result<Self, ProjectManifestError> {
        let manifest = toml::from_str::<Self>(input)
            .map_err(|error| ProjectManifestError::Toml(error.to_string()))?;
        manifest.validate()?;
        Ok(manifest)
    }

    /// Serializes a validated project manifest deterministically.
    pub fn to_toml_pretty(&self) -> Result<String, ProjectManifestError> {
        self.validate()?;
        toml::to_string_pretty(self).map_err(|error| ProjectManifestError::Toml(error.to_string()))
    }

    /// Replays schema, selection, and provider-field validation.
    pub fn validate(&self) -> Result<(), ProjectManifestError> {
        if self.schema != HYPERCIRCUIT_PROJECT_SCHEMA
            || self.version != HYPERCIRCUIT_PROJECT_VERSION
        {
            return Err(ProjectManifestError::UnsupportedSchema {
                schema: self.schema.clone(),
                version: self.version,
            });
        }
        for (field, value) in [
            ("project.name", self.project.name.as_str()),
            ("project.version", self.project.version.as_str()),
            (
                "project.default-design",
                self.project.default_design.as_str(),
            ),
        ] {
            if value.trim().is_empty() {
                return Err(ProjectManifestError::EmptyField(field));
            }
        }
        if !self.designs.contains_key(&self.project.default_design) {
            return Err(ProjectManifestError::UnknownDesign(
                self.project.default_design.clone(),
            ));
        }
        for (design, provider) in &self.designs {
            if design.trim().is_empty() {
                return Err(ProjectManifestError::EmptyDesignName);
            }
            provider.validate(design)?;
        }
        let mut material_handles = std::collections::BTreeSet::new();
        for material in &self.pcb_materials {
            material.validate()?;
            if !material_handles.insert(material.handle.as_str()) {
                return Err(ProjectManifestError::DuplicatePcbMaterial(
                    material.handle.clone(),
                ));
            }
        }
        Ok(())
    }

    /// Selects a named provider, or the retained default when no name is supplied.
    pub fn design(
        &self,
        name: Option<&str>,
    ) -> Result<(&str, &ProjectDesignProvider), ProjectManifestError> {
        let name = name.unwrap_or(&self.project.default_design);
        self.designs.get_key_value(name).map_or_else(
            || Err(ProjectManifestError::UnknownDesign(name.to_owned())),
            |(name, provider)| Ok((name.as_str(), provider)),
        )
    }

    /// Resolves validated manifest declarations into the HyperPhysics-backed
    /// material library consumed by controlled-impedance release checks.
    #[cfg(feature = "drc")]
    pub fn pcb_material_library(&self) -> Result<PcbMaterialPropertyLibrary, ProjectManifestError> {
        self.validate()?;
        let mut library = PcbMaterialPropertyLibrary::default();
        for material in &self.pcb_materials {
            let relative_permittivity = material.parse_property(
                "relative-permittivity",
                &material.relative_permittivity,
                false,
            )?;
            let loss_tangent =
                material.parse_property("loss-tangent", &material.loss_tangent, true)?;
            let mut graph = MaterialPropertyGraph::default();
            let mut source = SourceSpec::new(&material.source_authority, &material.source_locator);
            source.freshness.clone_from(&material.source_freshness);
            for (property, value) in [
                (PCB_RELATIVE_PERMITTIVITY_PROPERTY, relative_permittivity),
                (PCB_LOSS_TANGENT_PROPERTY, loss_tangent),
            ] {
                graph.push(MaterialAssertion {
                    kind: MaterialPropertyKind::Custom(property.to_owned()),
                    value: PropertyValue::exact_scalar(value),
                    unit: "1".into(),
                    state: MaterialState::Cured,
                    condition: material.condition.clone(),
                    source: source.clone(),
                });
            }
            library.insert(&material.handle, graph);
        }
        Ok(library)
    }
}

impl ProjectPcbMaterial {
    fn validate(&self) -> Result<(), ProjectManifestError> {
        for (field, value) in [
            ("handle", self.handle.as_str()),
            ("source-authority", self.source_authority.as_str()),
            ("source-locator", self.source_locator.as_str()),
        ] {
            if value.trim().is_empty() {
                return Err(self.invalid(format!("`{field}` cannot be empty")));
            }
        }
        for (field, value) in [
            ("source-freshness", self.source_freshness.as_deref()),
            ("condition", self.condition.as_deref()),
        ] {
            if value.is_some_and(|value| value.trim().is_empty()) {
                return Err(self.invalid(format!("`{field}` cannot be empty when present")));
            }
        }
        self.parse_property("relative-permittivity", &self.relative_permittivity, false)?;
        self.parse_property("loss-tangent", &self.loss_tangent, true)?;
        Ok(())
    }

    fn parse_property(
        &self,
        field: &'static str,
        source: &str,
        allow_zero: bool,
    ) -> Result<Real, ProjectManifestError> {
        let value = Real::from_str(source.trim())
            .map_err(|_| self.invalid(format!("`{field}` is not an exact scalar: {source:?}")))?;
        let sign = value.refine_sign_until(-64);
        let valid = if allow_zero {
            matches!(sign, Some(RealSign::Positive | RealSign::Zero))
        } else {
            sign == Some(RealSign::Positive)
        };
        if !valid {
            return Err(self.invalid(format!(
                "`{field}` must be {}",
                if allow_zero {
                    "nonnegative"
                } else {
                    "strictly positive"
                }
            )));
        }
        Ok(value)
    }

    fn invalid(&self, reason: String) -> ProjectManifestError {
        ProjectManifestError::InvalidPcbMaterial {
            material: self.handle.clone(),
            reason,
        }
    }
}

impl ProjectDesignProvider {
    fn validate(&self, design: &str) -> Result<(), ProjectManifestError> {
        let invalid = |reason: &str| ProjectManifestError::InvalidProvider {
            design: design.to_owned(),
            reason: reason.to_owned(),
        };
        match self.provider {
            ProjectProviderKind::Command => {
                if self
                    .command
                    .first()
                    .is_none_or(|item| item.trim().is_empty())
                {
                    return Err(invalid("`command` must contain an executable"));
                }
                if self.package.is_some() || self.binary.is_some() || !self.features.is_empty() {
                    return Err(invalid(
                        "command providers cannot set `package`, `binary`, or `features`",
                    ));
                }
            }
            ProjectProviderKind::Cargo => {
                if !self.command.is_empty() {
                    return Err(invalid("Cargo providers cannot set `command`"));
                }
                if self
                    .package
                    .as_deref()
                    .is_none_or(|value| value.trim().is_empty())
                {
                    return Err(invalid("Cargo providers require `package`"));
                }
                if self
                    .binary
                    .as_deref()
                    .is_none_or(|value| value.trim().is_empty())
                {
                    return Err(invalid("Cargo providers require `binary`"));
                }
                if self
                    .features
                    .iter()
                    .any(|feature| feature.trim().is_empty())
                {
                    return Err(invalid("Cargo features cannot be empty"));
                }
            }
        }
        Ok(())
    }
}
