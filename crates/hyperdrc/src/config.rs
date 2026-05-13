use std::path::Path;

use anyhow::{Context, Result};
use serde::Deserialize;

use crate::assembly_policy::{
    AssemblyBaseRules, AssemblyPolicyConfig, AssemblyProfile, AssemblyRules,
};
use crate::constraint_policy::{NetClassConfig, StackupConfig};
use crate::package_policy::{
    ArtifactRequirements, ArtifactRequirementsConfig, LayerRequirements, LayerRequirementsConfig,
    PackageProfile,
};

#[derive(Clone, Debug, Deserialize, Default)]
#[serde(default)]
pub struct RuleConfig {
    pub keepout: Option<f64>,
    pub clearance: Option<f64>,
    pub paste_tolerance: Option<f64>,
    pub min_paste_area_ratio: Option<f64>,
    pub max_paste_area_ratio: Option<f64>,
    pub stencil_thickness: Option<f64>,
    pub min_stencil_area_ratio: Option<f64>,
    pub min_width: Option<f64>,
    pub min_mask_width: Option<f64>,
    pub acid_trap_angle: Option<f64>,
    pub max_copper_imbalance_ratio: Option<f64>,
    pub annular_ring: Option<f64>,
    pub drill_clearance: Option<f64>,
    pub board_thickness: Option<f64>,
    pub max_drill_aspect_ratio: Option<f64>,
    pub net_clearance: Option<f64>,
    pub registration_tolerance: Option<f64>,
    pub panel_clearance: Option<f64>,
    pub ipc356_tolerance: Option<f64>,
    pub min_area: Option<f64>,
    pub max_layer_area: Option<f64>,
    pub generated_date_stale_days: Option<usize>,
    pub assembly_profile: Option<AssemblyProfile>,
    pub assembly: AssemblyPolicyConfig,
    pub package_profile: Option<PackageProfile>,
    pub required_artifacts: ArtifactRequirementsConfig,
    pub required_layers: LayerRequirementsConfig,
    pub kicad_copper_layers: Vec<String>,
    pub stackup: Option<StackupConfig>,
    pub net_classes: Vec<NetClassConfig>,
}

#[derive(Copy, Clone, Debug)]
pub struct EffectiveRules {
    pub keepout: f64,
    pub clearance: f64,
    pub paste_tolerance: f64,
    pub min_paste_area_ratio: f64,
    pub max_paste_area_ratio: f64,
    pub stencil_thickness: f64,
    pub min_stencil_area_ratio: f64,
    pub min_width: f64,
    pub min_mask_width: f64,
    pub acid_trap_angle: f64,
    pub max_copper_imbalance_ratio: f64,
    pub annular_ring: f64,
    pub drill_clearance: f64,
    pub board_thickness: f64,
    pub max_drill_aspect_ratio: f64,
    pub net_clearance: f64,
    pub registration_tolerance: f64,
    pub panel_clearance: f64,
    pub ipc356_tolerance: f64,
    pub min_area: f64,
    pub max_layer_area: Option<f64>,
    pub generated_date_stale_days: usize,
    pub assembly: AssemblyRules,
    pub package_profile: PackageProfile,
    pub required_artifacts: ArtifactRequirements,
    pub required_layers: LayerRequirements,
}

impl RuleConfig {
    pub fn load(path: &Path) -> Result<Self> {
        let text = std::fs::read_to_string(path)
            .with_context(|| format!("failed to read {}", path.display()))?;
        serde_json::from_str(&text)
            .with_context(|| format!("failed to parse config {}", path.display()))
    }
}

pub struct RuleOverrides {
    pub keepout: Option<f64>,
    pub clearance: Option<f64>,
    pub paste_tolerance: Option<f64>,
    pub min_paste_area_ratio: Option<f64>,
    pub max_paste_area_ratio: Option<f64>,
    pub stencil_thickness: Option<f64>,
    pub min_stencil_area_ratio: Option<f64>,
    pub min_width: Option<f64>,
    pub min_mask_width: Option<f64>,
    pub acid_trap_angle: Option<f64>,
    pub max_copper_imbalance_ratio: Option<f64>,
    pub annular_ring: Option<f64>,
    pub drill_clearance: Option<f64>,
    pub board_thickness: Option<f64>,
    pub max_drill_aspect_ratio: Option<f64>,
    pub net_clearance: Option<f64>,
    pub registration_tolerance: Option<f64>,
    pub panel_clearance: Option<f64>,
    pub ipc356_tolerance: Option<f64>,
    pub min_area: Option<f64>,
    pub max_layer_area: Option<f64>,
    pub generated_date_stale_days: Option<usize>,
}

pub fn effective_rules(config: &RuleConfig, overrides: RuleOverrides) -> EffectiveRules {
    let package_profile = config.package_profile.unwrap_or_default();
    let clearance = pick(overrides.clearance, config.clearance, 0.25);
    let min_width = pick(overrides.min_width, config.min_width, 0.15);
    let net_clearance = pick(overrides.net_clearance, config.net_clearance, 0.15);
    let assembly_profile = config.assembly_profile.unwrap_or_default();
    EffectiveRules {
        keepout: pick(overrides.keepout, config.keepout, 0.15),
        clearance,
        paste_tolerance: pick(overrides.paste_tolerance, config.paste_tolerance, 0.0),
        min_paste_area_ratio: pick(
            overrides.min_paste_area_ratio,
            config.min_paste_area_ratio,
            0.50,
        ),
        max_paste_area_ratio: pick(
            overrides.max_paste_area_ratio,
            config.max_paste_area_ratio,
            1.20,
        ),
        stencil_thickness: pick(overrides.stencil_thickness, config.stencil_thickness, 0.12),
        min_stencil_area_ratio: pick(
            overrides.min_stencil_area_ratio,
            config.min_stencil_area_ratio,
            0.66,
        ),
        min_width,
        min_mask_width: pick(overrides.min_mask_width, config.min_mask_width, 0.1),
        acid_trap_angle: pick(overrides.acid_trap_angle, config.acid_trap_angle, 30.0),
        max_copper_imbalance_ratio: pick(
            overrides.max_copper_imbalance_ratio,
            config.max_copper_imbalance_ratio,
            3.0,
        ),
        annular_ring: pick(overrides.annular_ring, config.annular_ring, 0.15),
        drill_clearance: pick(overrides.drill_clearance, config.drill_clearance, 0.25),
        board_thickness: pick(overrides.board_thickness, config.board_thickness, 1.6),
        max_drill_aspect_ratio: pick(
            overrides.max_drill_aspect_ratio,
            config.max_drill_aspect_ratio,
            10.0,
        ),
        net_clearance,
        registration_tolerance: pick(
            overrides.registration_tolerance,
            config.registration_tolerance,
            0.1,
        ),
        panel_clearance: pick(overrides.panel_clearance, config.panel_clearance, 0.5),
        ipc356_tolerance: pick(overrides.ipc356_tolerance, config.ipc356_tolerance, 0.1),
        min_area: pick(overrides.min_area, config.min_area, 1.0e-9),
        max_layer_area: overrides.max_layer_area.or(config.max_layer_area),
        generated_date_stale_days: overrides
            .generated_date_stale_days
            .or(config.generated_date_stale_days)
            .unwrap_or(90),
        assembly: AssemblyRules::resolve(
            assembly_profile,
            &config.assembly,
            AssemblyBaseRules {
                clearance,
                min_width,
                net_clearance,
            },
        ),
        package_profile,
        required_artifacts: ArtifactRequirements::resolve(
            package_profile,
            &config.required_artifacts,
        ),
        required_layers: LayerRequirements::resolve(package_profile, &config.required_layers),
    }
}

fn pick(override_value: Option<f64>, config_value: Option<f64>, default_value: f64) -> f64 {
    override_value.or(config_value).unwrap_or(default_value)
}

#[cfg(test)]
mod tests {
    use std::fs;

    use crate::assembly_policy::AssemblyProfile;
    use crate::package_policy::PackageProfile;

    use super::{RuleConfig, RuleOverrides, effective_rules};

    #[test]
    fn cli_overrides_config_and_defaults() {
        let config = RuleConfig {
            keepout: Some(0.2),
            min_area: Some(0.01),
            ..RuleConfig::default()
        };
        let rules = effective_rules(
            &config,
            RuleOverrides {
                keepout: Some(0.3),
                clearance: None,
                paste_tolerance: None,
                min_paste_area_ratio: None,
                max_paste_area_ratio: None,
                stencil_thickness: None,
                min_stencil_area_ratio: None,
                min_width: None,
                min_mask_width: None,
                acid_trap_angle: None,
                max_copper_imbalance_ratio: None,
                annular_ring: None,
                drill_clearance: None,
                board_thickness: None,
                max_drill_aspect_ratio: None,
                net_clearance: None,
                registration_tolerance: None,
                panel_clearance: None,
                ipc356_tolerance: None,
                min_area: None,
                max_layer_area: None,
                generated_date_stale_days: Some(30),
            },
        );

        assert_eq!(rules.keepout, 0.3);
        assert_eq!(rules.min_area, 0.01);
        assert_eq!(rules.clearance, 0.25);
        assert_eq!(rules.generated_date_stale_days, 30);
        assert_eq!(rules.assembly.profile, AssemblyProfile::ProductionSmt);
        assert_eq!(rules.assembly.component_edge_clearance, 0.5);
        assert_eq!(rules.package_profile, PackageProfile::FullProduction);
        assert!(rules.required_artifacts.bom);
        assert!(rules.required_layers.top_mask);
    }

    #[test]
    fn rejects_malformed_config_json() {
        let path =
            std::env::temp_dir().join(format!("hyperdrc-bad-config-{}.json", std::process::id()));
        fs::write(&path, "{not-json").unwrap();

        let result = RuleConfig::load(&path);

        assert!(result.is_err());
        let _ = fs::remove_file(path);
    }

    #[test]
    fn loads_config_with_unknown_fields_ignored() {
        let path =
            std::env::temp_dir().join(format!("hyperdrc-config-{}.json", std::process::id()));
        fs::write(
            &path,
            r#"{
              "keepout":0.42,
              "generated_date_stale_days":45,
              "package_profile": "fabrication-only",
              "required_artifacts": {
                "bom": true,
                "centroid": false,
                "netlist": false,
                "rout_drawing": false
              },
              "required_layers": {
                "board_outline": true,
                "drill_data": true,
                "top_paste": false,
                "bottom_paste": false,
                "top_silkscreen": false,
                "bottom_silkscreen": false
              },
              "kicad_copper_layers":["F.Cu"],
              "unknown":true
            }"#,
        )
        .unwrap();

        let config = RuleConfig::load(&path).unwrap();

        assert_eq!(config.keepout, Some(0.42));
        assert_eq!(config.generated_date_stale_days, Some(45));
        assert_eq!(
            config.package_profile,
            Some(PackageProfile::FabricationOnly)
        );
        assert_eq!(config.required_artifacts.bom, Some(true));
        assert_eq!(config.required_artifacts.centroid, Some(false));
        assert_eq!(config.required_artifacts.netlist, Some(false));
        assert_eq!(config.required_artifacts.rout_drawing, Some(false));
        assert_eq!(config.required_layers.board_outline, Some(true));
        assert_eq!(config.required_layers.drill_data, Some(true));
        assert_eq!(config.required_layers.top_paste, Some(false));
        assert_eq!(config.required_layers.bottom_paste, Some(false));
        assert_eq!(config.required_layers.top_silkscreen, Some(false));
        assert_eq!(config.required_layers.bottom_silkscreen, Some(false));
        assert_eq!(config.kicad_copper_layers, vec!["F.Cu"]);
        let _ = fs::remove_file(path);
    }

    #[test]
    fn package_profile_sets_manifest_policy_defaults() {
        let config = RuleConfig {
            package_profile: Some(PackageProfile::FabricationOnly),
            ..RuleConfig::default()
        };
        let rules = effective_rules(
            &config,
            RuleOverrides {
                keepout: None,
                clearance: None,
                paste_tolerance: None,
                min_paste_area_ratio: None,
                max_paste_area_ratio: None,
                stencil_thickness: None,
                min_stencil_area_ratio: None,
                min_width: None,
                min_mask_width: None,
                acid_trap_angle: None,
                max_copper_imbalance_ratio: None,
                annular_ring: None,
                drill_clearance: None,
                board_thickness: None,
                max_drill_aspect_ratio: None,
                net_clearance: None,
                registration_tolerance: None,
                panel_clearance: None,
                ipc356_tolerance: None,
                min_area: None,
                max_layer_area: None,
                generated_date_stale_days: None,
            },
        );

        assert_eq!(rules.package_profile, PackageProfile::FabricationOnly);
        assert!(rules.required_artifacts.fab_drawing);
        assert!(!rules.required_artifacts.centroid);
        assert!(rules.required_layers.board_outline);
        assert!(!rules.required_layers.top_paste);
    }

    #[test]
    fn explicit_manifest_policy_fields_override_profile_defaults() {
        let config = RuleConfig {
            package_profile: Some(PackageProfile::AssemblyOnly),
            required_artifacts: crate::package_policy::ArtifactRequirementsConfig {
                netlist: Some(true),
                assembly_drawing: Some(false),
                ..Default::default()
            },
            required_layers: crate::package_policy::LayerRequirementsConfig {
                drill_data: Some(true),
                top_paste: Some(false),
                ..Default::default()
            },
            ..RuleConfig::default()
        };
        let rules = effective_rules(
            &config,
            RuleOverrides {
                keepout: None,
                clearance: None,
                paste_tolerance: None,
                min_paste_area_ratio: None,
                max_paste_area_ratio: None,
                stencil_thickness: None,
                min_stencil_area_ratio: None,
                min_width: None,
                min_mask_width: None,
                acid_trap_angle: None,
                max_copper_imbalance_ratio: None,
                annular_ring: None,
                drill_clearance: None,
                board_thickness: None,
                max_drill_aspect_ratio: None,
                net_clearance: None,
                registration_tolerance: None,
                panel_clearance: None,
                ipc356_tolerance: None,
                min_area: None,
                max_layer_area: None,
                generated_date_stale_days: None,
            },
        );

        assert_eq!(rules.package_profile, PackageProfile::AssemblyOnly);
        assert!(rules.required_artifacts.bom);
        assert!(rules.required_artifacts.netlist);
        assert!(!rules.required_artifacts.assembly_drawing);
        assert!(rules.required_layers.drill_data);
        assert!(!rules.required_layers.top_paste);
        assert!(rules.required_layers.bottom_paste);
    }

    #[test]
    fn loads_stackup_and_net_class_sections() {
        let path = std::env::temp_dir().join(format!(
            "hyperdrc-constraints-config-{}.json",
            std::process::id()
        ));
        fs::write(
            &path,
            r#"{
              "stackup": {
                "copper_layer_count": 2,
                "finished_thickness": 1.6,
                "impedance_controlled": true,
                "layers": [
                  {"name":"F.Cu","kind":"copper","copper_weight_oz":1.0},
                  {"name":"dielectric","kind":"dielectric","dielectric_thickness":1.5},
                  {"name":"B.Cu","kind":"copper","copper_weight_oz":1.0}
                ]
              },
              "assembly_profile": "test-fixture",
              "assembly": {
                "component_edge_clearance": 0.6,
                "testpoint_min_diameter": 0.45,
                "tooling_min_diameter": 1.2,
                "dense_pad_pitch": 0.65
              },
              "net_classes": [
                {
                  "name": "power",
                  "nets": ["VBUS"],
                  "net_patterns": ["PWR_*"],
                  "min_width": 0.5,
                  "min_clearance": 0.25,
                  "max_layer_count": 1,
                  "min_via_count": 2,
                  "min_current_width": 0.75,
                  "min_voltage_clearance": 0.5,
                  "requires_reference_plane": true,
                  "requires_impedance_control": true,
                  "max_via_count": 4
                },
                {
                  "name": "usb-p",
                  "nets": ["USB_D+"],
                  "differential_pair": "usb",
                  "differential_role": "positive",
                  "min_pair_spacing": 0.12,
                  "max_pair_spacing": 0.22
                }
              ]
            }"#,
        )
        .unwrap();

        let config = RuleConfig::load(&path).unwrap();

        assert_eq!(config.assembly_profile, Some(AssemblyProfile::TestFixture));
        assert_eq!(config.assembly.component_edge_clearance, Some(0.6));
        assert_eq!(config.assembly.testpoint_min_diameter, Some(0.45));
        assert_eq!(config.assembly.tooling_min_diameter, Some(1.2));
        assert_eq!(config.assembly.dense_pad_pitch, Some(0.65));
        assert_eq!(config.stackup.unwrap().copper_layer_count, Some(2));
        assert_eq!(config.net_classes[0].name, "power");
        assert_eq!(config.net_classes[0].net_patterns, vec!["PWR_*"]);
        assert_eq!(config.net_classes[0].min_current_width, Some(0.75));
        assert_eq!(config.net_classes[0].min_voltage_clearance, Some(0.5));
        assert_eq!(config.net_classes[0].requires_reference_plane, Some(true));
        assert_eq!(config.net_classes[0].requires_impedance_control, Some(true));
        assert_eq!(config.net_classes[0].max_via_count, Some(4));
        assert_eq!(
            config.net_classes[1].differential_pair.as_deref(),
            Some("usb")
        );
        assert_eq!(config.net_classes[1].min_pair_spacing, Some(0.12));
        assert_eq!(config.net_classes[1].max_pair_spacing, Some(0.22));
        let _ = fs::remove_file(path);
    }
}
