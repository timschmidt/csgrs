use std::path::Path;

use anyhow::{Context, Result};
use serde::Deserialize;

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
    pub kicad_copper_layers: Vec<String>,
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
}

pub fn effective_rules(config: &RuleConfig, overrides: RuleOverrides) -> EffectiveRules {
    EffectiveRules {
        keepout: pick(overrides.keepout, config.keepout, 0.15),
        clearance: pick(overrides.clearance, config.clearance, 0.25),
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
        min_width: pick(overrides.min_width, config.min_width, 0.15),
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
        net_clearance: pick(overrides.net_clearance, config.net_clearance, 0.15),
        registration_tolerance: pick(
            overrides.registration_tolerance,
            config.registration_tolerance,
            0.1,
        ),
        panel_clearance: pick(overrides.panel_clearance, config.panel_clearance, 0.5),
        ipc356_tolerance: pick(overrides.ipc356_tolerance, config.ipc356_tolerance, 0.1),
        min_area: pick(overrides.min_area, config.min_area, 1.0e-9),
        max_layer_area: overrides.max_layer_area.or(config.max_layer_area),
    }
}

fn pick(override_value: Option<f64>, config_value: Option<f64>, default_value: f64) -> f64 {
    override_value.or(config_value).unwrap_or(default_value)
}

#[cfg(test)]
mod tests {
    use std::fs;

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
            },
        );

        assert_eq!(rules.keepout, 0.3);
        assert_eq!(rules.min_area, 0.01);
        assert_eq!(rules.clearance, 0.25);
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
            r#"{"keepout":0.42,"kicad_copper_layers":["F.Cu"],"unknown":true}"#,
        )
        .unwrap();

        let config = RuleConfig::load(&path).unwrap();

        assert_eq!(config.keepout, Some(0.42));
        assert_eq!(config.kicad_copper_layers, vec!["F.Cu"]);
        let _ = fs::remove_file(path);
    }
}
