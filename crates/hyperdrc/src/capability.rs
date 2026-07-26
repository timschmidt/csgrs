//! Versioned, digestible PCB manufacturing capability profiles.
//!
//! Profiles are release-review contracts, not vendor warranties. Built-ins are
//! deliberately conservative examples that make native HyperCircuit release
//! calls useful without requiring a rule-deck file.

use std::collections::BTreeMap;

use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};

use crate::Scalar;
use crate::constraint_policy::FabricationCapabilityConfig;
use crate::scalar::scalar;

/// Stable capability-profile schema identity.
pub const CAPABILITY_PROFILE_SCHEMA: &str = "hyperdrc.capability-profile";
/// Current capability-profile schema revision.
pub const CAPABILITY_PROFILE_VERSION: u32 = 1;

/// Provenance classification for a process capability profile.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CapabilityProfileClass {
    /// Conservative HyperDRC example; never a vendor warranty.
    ExampleNotWarranty,
    /// Customer- or organization-maintained contractual profile.
    CustomerSupplied,
    /// Immutable profile obtained from an external service.
    ExternalService,
}

/// Drill, routed-slot, and via construction limits.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct DrillCapability {
    /// Minimum finished mechanical drill diameter in millimeters.
    pub minimum_mechanical_drill: Option<Scalar>,
    /// Minimum laser drill diameter in millimeters.
    pub minimum_laser_drill: Option<Scalar>,
    /// Minimum routed-slot width in millimeters.
    pub minimum_routed_slot: Option<Scalar>,
    /// Minimum annular ring in millimeters.
    pub minimum_annular_ring: Option<Scalar>,
    /// Maximum mechanical-drill depth/diameter ratio.
    pub maximum_mechanical_aspect_ratio: Option<Scalar>,
    /// Maximum laser-drill depth/diameter ratio.
    pub maximum_laser_aspect_ratio: Option<Scalar>,
    /// Supported typed via processes.
    pub supported_via_processes: Vec<String>,
}

/// Etch, mask, paste, legend, and board-edge limits.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct ImagingCapability {
    /// Minimum finished trace width in millimeters.
    pub minimum_trace_width: Option<Scalar>,
    /// Minimum different-net spacing in millimeters.
    pub minimum_spacing: Option<Scalar>,
    /// Minimum copper-to-profile clearance in millimeters.
    pub minimum_copper_to_edge: Option<Scalar>,
    /// Minimum solder-mask dam in millimeters.
    pub minimum_mask_dam: Option<Scalar>,
    /// Minimum legend stroke in millimeters.
    pub minimum_legend_width: Option<Scalar>,
    /// Minimum registration tolerance in millimeters.
    pub registration_tolerance: Option<Scalar>,
    /// Whether controlled-impedance review is available.
    pub controlled_impedance: bool,
}

/// Panel, tooling, and assembly-facing process limits.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct PanelAssemblyCapability {
    /// Minimum panel rail width in millimeters.
    pub minimum_rail_width: Option<Scalar>,
    /// Minimum routed web width in millimeters.
    pub minimum_web_width: Option<Scalar>,
    /// Minimum tooling-hole diameter in millimeters.
    pub minimum_tooling_hole: Option<Scalar>,
    /// Minimum global fiducial diameter in millimeters.
    pub minimum_fiducial_diameter: Option<Scalar>,
    /// Maximum component height in millimeters.
    pub maximum_component_height: Option<Scalar>,
    /// Minimum stencil aperture area ratio.
    pub minimum_stencil_area_ratio: Option<Scalar>,
}

/// One immutable process capability and its enforcing check map.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CapabilityProfile {
    /// Stable schema identity.
    pub schema: String,
    /// Schema revision.
    pub schema_version: u32,
    /// Stable profile identity.
    pub id: String,
    /// Profile-provider revision.
    pub revision: String,
    /// Provenance and warranty classification.
    pub class: CapabilityProfileClass,
    /// Human-readable source or service locator.
    pub source: String,
    /// Required warning carried into review evidence.
    pub notice: String,
    /// Stack-up ranges consumed by stack-up readiness.
    pub stackup: FabricationCapabilityConfig,
    /// Drill and via capabilities.
    pub drilling: DrillCapability,
    /// Imaging and registration capabilities.
    pub imaging: ImagingCapability,
    /// Panel and assembly capabilities.
    pub panel_assembly: PanelAssemblyCapability,
    /// Supported normalized surface-finish names.
    pub surface_finishes: Vec<String>,
    /// Profile field path to one or more stable enforcing check IDs.
    pub enforcement: BTreeMap<String, Vec<String>>,
}

impl CapabilityProfile {
    /// Returns deterministic JSON bytes after canonical collection ordering.
    pub fn canonical_bytes(&self) -> Result<Vec<u8>, serde_json::Error> {
        let mut canonical = self.clone();
        canonical.surface_finishes.sort();
        canonical.surface_finishes.dedup();
        for checks in canonical.enforcement.values_mut() {
            checks.sort();
            checks.dedup();
        }
        serde_json::to_vec(&canonical)
    }

    /// Returns the lowercase SHA-256 digest of canonical profile bytes.
    pub fn digest(&self) -> Result<String, serde_json::Error> {
        let bytes = self.canonical_bytes()?;
        Ok(format!("sha256:{:x}", Sha256::digest(bytes)))
    }

    /// Checks identity, disclaimer, positivity, and enforcement coverage.
    pub fn validate(&self) -> Result<(), String> {
        if self.schema != CAPABILITY_PROFILE_SCHEMA
            || self.schema_version != CAPABILITY_PROFILE_VERSION
        {
            return Err("unsupported capability profile schema".into());
        }
        if self.id.trim().is_empty()
            || self.revision.trim().is_empty()
            || self.source.trim().is_empty()
            || self.notice.trim().is_empty()
        {
            return Err("capability profile identity and notice must be nonempty".into());
        }
        for (name, value) in self.scalar_limits() {
            if value <= &Scalar::zero() {
                return Err(format!("{name} must be positive"));
            }
            if !self.enforcement.contains_key(name) {
                return Err(format!("{name} has no enforcing check"));
            }
        }
        Ok(())
    }

    fn scalar_limits(&self) -> Vec<(&'static str, &Scalar)> {
        let mut limits = Vec::new();
        macro_rules! limit {
            ($path:literal, $value:expr) => {
                if let Some(value) = $value.as_ref() {
                    limits.push(($path, value));
                }
            };
        }
        limit!(
            "drilling.minimum_mechanical_drill",
            self.drilling.minimum_mechanical_drill
        );
        limit!(
            "drilling.minimum_laser_drill",
            self.drilling.minimum_laser_drill
        );
        limit!(
            "drilling.minimum_routed_slot",
            self.drilling.minimum_routed_slot
        );
        limit!(
            "drilling.minimum_annular_ring",
            self.drilling.minimum_annular_ring
        );
        limit!(
            "drilling.maximum_mechanical_aspect_ratio",
            self.drilling.maximum_mechanical_aspect_ratio
        );
        limit!(
            "drilling.maximum_laser_aspect_ratio",
            self.drilling.maximum_laser_aspect_ratio
        );
        limit!(
            "imaging.minimum_trace_width",
            self.imaging.minimum_trace_width
        );
        limit!("imaging.minimum_spacing", self.imaging.minimum_spacing);
        limit!(
            "imaging.minimum_copper_to_edge",
            self.imaging.minimum_copper_to_edge
        );
        limit!("imaging.minimum_mask_dam", self.imaging.minimum_mask_dam);
        limit!(
            "imaging.minimum_legend_width",
            self.imaging.minimum_legend_width
        );
        limit!(
            "imaging.registration_tolerance",
            self.imaging.registration_tolerance
        );
        limit!(
            "panel_assembly.minimum_rail_width",
            self.panel_assembly.minimum_rail_width
        );
        limit!(
            "panel_assembly.minimum_web_width",
            self.panel_assembly.minimum_web_width
        );
        limit!(
            "panel_assembly.minimum_tooling_hole",
            self.panel_assembly.minimum_tooling_hole
        );
        limit!(
            "panel_assembly.minimum_fiducial_diameter",
            self.panel_assembly.minimum_fiducial_diameter
        );
        limit!(
            "panel_assembly.maximum_component_height",
            self.panel_assembly.maximum_component_height
        );
        limit!(
            "panel_assembly.minimum_stencil_area_ratio",
            self.panel_assembly.minimum_stencil_area_ratio
        );
        limits
    }
}

/// Conservative built-in profile optimized for successful first use.
pub fn opinionated_prototype_profile() -> CapabilityProfile {
    example_profile(
        "generic-prototype",
        "1",
        scalar("0.15"),
        scalar("0.15"),
        scalar("0.30"),
        scalar("0.15"),
        4,
    )
}

/// Mainstream multilayer example profile.
pub fn mainstream_profile() -> CapabilityProfile {
    example_profile(
        "generic-mainstream",
        "1",
        scalar("0.10"),
        scalar("0.10"),
        scalar("0.20"),
        scalar("0.10"),
        8,
    )
}

/// Advanced/HDI example profile.
pub fn hdi_profile() -> CapabilityProfile {
    example_profile(
        "generic-hdi",
        "1",
        scalar("0.075"),
        scalar("0.075"),
        scalar("0.15"),
        scalar("0.075"),
        16,
    )
}

fn example_profile(
    id: &str,
    revision: &str,
    trace: Scalar,
    spacing: Scalar,
    drill: Scalar,
    annular: Scalar,
    layers: usize,
) -> CapabilityProfile {
    let mut enforcement = BTreeMap::new();
    for (field, checks) in [
        (
            "drilling.minimum_mechanical_drill",
            vec!["drill-table-consistency"],
        ),
        ("drilling.minimum_laser_drill", vec!["via-in-pad-readiness"]),
        (
            "drilling.minimum_routed_slot",
            vec!["authored-routed-slot-readiness"],
        ),
        ("drilling.minimum_annular_ring", vec!["annular-ring"]),
        (
            "drilling.maximum_mechanical_aspect_ratio",
            vec!["drill-aspect-ratio"],
        ),
        (
            "drilling.maximum_laser_aspect_ratio",
            vec!["drill-aspect-ratio"],
        ),
        (
            "imaging.minimum_trace_width",
            vec!["copper-width-readiness"],
        ),
        ("imaging.minimum_spacing", vec!["different-net-spacing"]),
        (
            "imaging.minimum_copper_to_edge",
            vec!["board-edge-clearance"],
        ),
        ("imaging.minimum_mask_dam", vec!["solder-mask-sliver"]),
        ("imaging.minimum_legend_width", vec!["silkscreen-min-width"]),
        (
            "imaging.registration_tolerance",
            vec!["registration-tolerance"],
        ),
        (
            "panel_assembly.minimum_rail_width",
            vec!["panelization-clearance"],
        ),
        (
            "panel_assembly.minimum_web_width",
            vec!["panelization-clearance"],
        ),
        (
            "panel_assembly.minimum_tooling_hole",
            vec!["tooling-hole-readiness"],
        ),
        (
            "panel_assembly.minimum_fiducial_diameter",
            vec!["fiducial-readiness"],
        ),
        (
            "panel_assembly.maximum_component_height",
            vec!["production-artifact-readiness"],
        ),
        (
            "panel_assembly.minimum_stencil_area_ratio",
            vec!["stencil-area-ratio-readiness"],
        ),
    ] {
        enforcement.insert(
            field.into(),
            checks.into_iter().map(str::to_owned).collect(),
        );
    }
    CapabilityProfile {
        schema: CAPABILITY_PROFILE_SCHEMA.into(),
        schema_version: CAPABILITY_PROFILE_VERSION,
        id: id.into(),
        revision: revision.into(),
        class: CapabilityProfileClass::ExampleNotWarranty,
        source: "HyperDRC built-in example".into(),
        notice: "Example capability for early review only; confirm current limits with the selected manufacturer.".into(),
        stackup: FabricationCapabilityConfig {
            min_finished_thickness: Some(scalar("0.6")),
            preferred_min_finished_thickness: Some(scalar("0.8")),
            preferred_max_finished_thickness: Some(scalar("1.6")),
            max_finished_thickness: Some(scalar("2.4")),
            max_copper_layers: Some(layers),
            preferred_max_copper_layers: Some(layers.min(4)),
            cost_escalation_copper_layers: Some(layers.min(6)),
            min_copper_weight_oz: Some(scalar("0.5")),
            preferred_min_copper_weight_oz: Some(scalar("1.0")),
            preferred_max_copper_weight_oz: Some(scalar("1.0")),
            cost_escalation_copper_weight_oz: Some(scalar("2.0")),
            max_copper_weight_oz: Some(scalar("2.0")),
            min_dielectric_thickness: Some(scalar("0.05")),
            preferred_min_dielectric_thickness: Some(scalar("0.10")),
            cost_escalation_min_dielectric_thickness: Some(scalar("0.075")),
            ..FabricationCapabilityConfig::default()
        },
        drilling: DrillCapability {
            minimum_mechanical_drill: Some(drill),
            minimum_laser_drill: Some(scalar("0.10")),
            minimum_routed_slot: Some(scalar("0.60")),
            minimum_annular_ring: Some(annular),
            maximum_mechanical_aspect_ratio: Some(scalar("8")),
            maximum_laser_aspect_ratio: Some(scalar("1")),
            supported_via_processes: vec!["plated-through".into()],
        },
        imaging: ImagingCapability {
            minimum_trace_width: Some(trace),
            minimum_spacing: Some(spacing),
            minimum_copper_to_edge: Some(scalar("0.30")),
            minimum_mask_dam: Some(scalar("0.10")),
            minimum_legend_width: Some(scalar("0.15")),
            registration_tolerance: Some(scalar("0.10")),
            controlled_impedance: true,
        },
        panel_assembly: PanelAssemblyCapability {
            minimum_rail_width: Some(scalar("5")),
            minimum_web_width: Some(scalar("2")),
            minimum_tooling_hole: Some(scalar("2")),
            minimum_fiducial_diameter: Some(scalar("1")),
            maximum_component_height: Some(scalar("15")),
            minimum_stencil_area_ratio: Some(scalar("0.66")),
        },
        surface_finishes: vec!["enig".into(), "lead-free-hasl".into(), "osp".into()],
        enforcement,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn builtins_are_valid_digestible_and_visibly_not_warranties() {
        for profile in [
            opinionated_prototype_profile(),
            mainstream_profile(),
            hdi_profile(),
        ] {
            profile.validate().unwrap();
            assert_eq!(profile.class, CapabilityProfileClass::ExampleNotWarranty);
            assert!(profile.notice.contains("confirm"));
            assert!(profile.digest().unwrap().starts_with("sha256:"));
        }
    }

    #[test]
    fn profile_mutation_changes_digest() {
        let base = opinionated_prototype_profile();
        let mut changed = base.clone();
        changed.imaging.minimum_spacing = Some(scalar("0.16"));

        assert_ne!(base.digest().unwrap(), changed.digest().unwrap());
    }
}
