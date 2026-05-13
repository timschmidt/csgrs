//! Assembly-readiness policy profiles and resolved thresholds.
//!
//! The assembly checks operate on geometry, but the right thresholds depend on
//! process assumptions. These profiles keep the defaults discoverable while
//! allowing a rule deck to override any individual threshold.

use serde::Deserialize;

#[derive(Copy, Clone, Debug, Deserialize, Eq, PartialEq)]
#[serde(rename_all = "kebab-case")]
pub enum AssemblyProfile {
    Prototype,
    ProductionSmt,
    DoubleSidedSmt,
    TestFixture,
}

impl Default for AssemblyProfile {
    fn default() -> Self {
        Self::ProductionSmt
    }
}

#[derive(Clone, Debug, Deserialize, Default)]
#[serde(default)]
pub struct AssemblyPolicyConfig {
    pub component_edge_clearance: Option<f64>,
    pub component_hole_clearance: Option<f64>,
    pub connector_rework_clearance: Option<f64>,
    pub connector_min_pad_dimension: Option<f64>,
    pub pad_pair_max_gap: Option<f64>,
    pub pad_pair_max_area_ratio: Option<f64>,
    pub pad_pair_max_pad_dimension: Option<f64>,
    pub testpoint_min_diameter: Option<f64>,
    pub testpoint_min_spacing: Option<f64>,
    pub testpoint_edge_clearance: Option<f64>,
    pub tooling_min_diameter: Option<f64>,
    pub tooling_max_diameter: Option<f64>,
    pub tooling_edge_clearance: Option<f64>,
    pub mouse_bite_min_diameter: Option<f64>,
    pub mouse_bite_max_diameter: Option<f64>,
    pub mouse_bite_min_spacing: Option<f64>,
    pub mouse_bite_max_spacing: Option<f64>,
    pub fiducial_edge_clearance: Option<f64>,
    pub local_fiducial_pitch: Option<f64>,
    pub local_fiducial_search_radius: Option<f64>,
    pub dense_pad_pitch: Option<f64>,
    pub dense_pad_via_search_radius: Option<f64>,
}

#[derive(Copy, Clone, Debug, PartialEq)]
pub struct AssemblyBaseRules {
    pub clearance: f64,
    pub min_width: f64,
    pub net_clearance: f64,
}

#[derive(Copy, Clone, Debug, PartialEq)]
pub struct AssemblyRules {
    pub profile: AssemblyProfile,
    pub component_edge_clearance: f64,
    pub component_hole_clearance: f64,
    pub connector_rework_clearance: f64,
    pub connector_min_pad_dimension: f64,
    pub pad_pair_max_gap: f64,
    pub pad_pair_max_area_ratio: f64,
    pub pad_pair_max_pad_dimension: f64,
    pub testpoint_min_diameter: f64,
    pub testpoint_min_spacing: f64,
    pub testpoint_edge_clearance: f64,
    pub tooling_min_diameter: f64,
    pub tooling_max_diameter: f64,
    pub tooling_edge_clearance: f64,
    pub mouse_bite_min_diameter: f64,
    pub mouse_bite_max_diameter: f64,
    pub mouse_bite_min_spacing: f64,
    pub mouse_bite_max_spacing: f64,
    pub fiducial_edge_clearance: f64,
    pub local_fiducial_pitch: f64,
    pub local_fiducial_search_radius: f64,
    pub dense_pad_pitch: f64,
    pub dense_pad_via_search_radius: f64,
}

impl AssemblyRules {
    pub fn resolve(
        profile: AssemblyProfile,
        config: &AssemblyPolicyConfig,
        base: AssemblyBaseRules,
    ) -> Self {
        let defaults = Self::for_profile(profile, base);
        Self {
            profile,
            component_edge_clearance: config
                .component_edge_clearance
                .unwrap_or(defaults.component_edge_clearance),
            component_hole_clearance: config
                .component_hole_clearance
                .unwrap_or(defaults.component_hole_clearance),
            connector_rework_clearance: config
                .connector_rework_clearance
                .unwrap_or(defaults.connector_rework_clearance),
            connector_min_pad_dimension: config
                .connector_min_pad_dimension
                .unwrap_or(defaults.connector_min_pad_dimension),
            pad_pair_max_gap: config.pad_pair_max_gap.unwrap_or(defaults.pad_pair_max_gap),
            pad_pair_max_area_ratio: config
                .pad_pair_max_area_ratio
                .unwrap_or(defaults.pad_pair_max_area_ratio),
            pad_pair_max_pad_dimension: config
                .pad_pair_max_pad_dimension
                .unwrap_or(defaults.pad_pair_max_pad_dimension),
            testpoint_min_diameter: config
                .testpoint_min_diameter
                .unwrap_or(defaults.testpoint_min_diameter),
            testpoint_min_spacing: config
                .testpoint_min_spacing
                .unwrap_or(defaults.testpoint_min_spacing),
            testpoint_edge_clearance: config
                .testpoint_edge_clearance
                .unwrap_or(defaults.testpoint_edge_clearance),
            tooling_min_diameter: config
                .tooling_min_diameter
                .unwrap_or(defaults.tooling_min_diameter),
            tooling_max_diameter: config
                .tooling_max_diameter
                .unwrap_or(defaults.tooling_max_diameter),
            tooling_edge_clearance: config
                .tooling_edge_clearance
                .unwrap_or(defaults.tooling_edge_clearance),
            mouse_bite_min_diameter: config
                .mouse_bite_min_diameter
                .unwrap_or(defaults.mouse_bite_min_diameter),
            mouse_bite_max_diameter: config
                .mouse_bite_max_diameter
                .unwrap_or(defaults.mouse_bite_max_diameter),
            mouse_bite_min_spacing: config
                .mouse_bite_min_spacing
                .unwrap_or(defaults.mouse_bite_min_spacing),
            mouse_bite_max_spacing: config
                .mouse_bite_max_spacing
                .unwrap_or(defaults.mouse_bite_max_spacing),
            fiducial_edge_clearance: config
                .fiducial_edge_clearance
                .unwrap_or(defaults.fiducial_edge_clearance),
            local_fiducial_pitch: config
                .local_fiducial_pitch
                .unwrap_or(defaults.local_fiducial_pitch),
            local_fiducial_search_radius: config
                .local_fiducial_search_radius
                .unwrap_or(defaults.local_fiducial_search_radius),
            dense_pad_pitch: config.dense_pad_pitch.unwrap_or(defaults.dense_pad_pitch),
            dense_pad_via_search_radius: config
                .dense_pad_via_search_radius
                .unwrap_or(defaults.dense_pad_via_search_radius),
        }
    }

    fn for_profile(profile: AssemblyProfile, base: AssemblyBaseRules) -> Self {
        // Defaults preserve the historical hyperdrc multipliers while grouping
        // them by assembly process. IPC-7351B frames land-pattern geometry as
        // assembly-process dependent, and IPC-9252B/fixture practice similarly
        // makes probe size and spacing process constraints rather than universal
        // constants.
        let production = Self {
            profile,
            component_edge_clearance: base.clearance * 2.0,
            component_hole_clearance: base.clearance * 2.0,
            connector_rework_clearance: base.clearance * 2.0,
            connector_min_pad_dimension: base.min_width * 3.0,
            pad_pair_max_gap: base.min_width * 8.0,
            pad_pair_max_area_ratio: 1.5,
            pad_pair_max_pad_dimension: base.min_width * 10.0,
            testpoint_min_diameter: base.min_width,
            testpoint_min_spacing: base.net_clearance * 4.0,
            testpoint_edge_clearance: base.clearance * 2.0,
            tooling_min_diameter: base.min_width * 4.0,
            tooling_max_diameter: base.min_width * 20.0,
            tooling_edge_clearance: base.clearance * 2.0,
            mouse_bite_min_diameter: base.min_width,
            mouse_bite_max_diameter: base.min_width * 4.0,
            mouse_bite_min_spacing: base.min_width * 2.0,
            mouse_bite_max_spacing: base.min_width * 8.0,
            fiducial_edge_clearance: base.clearance * 2.0,
            local_fiducial_pitch: 0.8,
            local_fiducial_search_radius: base.net_clearance * 25.0,
            dense_pad_pitch: 0.8,
            dense_pad_via_search_radius: base.net_clearance * 10.0,
        };

        match profile {
            AssemblyProfile::ProductionSmt => production,
            AssemblyProfile::Prototype => Self {
                component_edge_clearance: base.clearance,
                component_hole_clearance: base.clearance,
                connector_rework_clearance: base.clearance,
                pad_pair_max_area_ratio: 2.0,
                local_fiducial_pitch: 0.65,
                dense_pad_pitch: 0.65,
                ..production
            },
            AssemblyProfile::DoubleSidedSmt => Self {
                component_edge_clearance: base.clearance * 2.5,
                component_hole_clearance: base.clearance * 2.5,
                fiducial_edge_clearance: base.clearance * 2.5,
                local_fiducial_search_radius: base.net_clearance * 20.0,
                dense_pad_via_search_radius: base.net_clearance * 8.0,
                ..production
            },
            AssemblyProfile::TestFixture => Self {
                testpoint_min_diameter: base.min_width * 1.5,
                testpoint_min_spacing: base.net_clearance * 6.0,
                testpoint_edge_clearance: base.clearance * 3.0,
                tooling_min_diameter: base.min_width * 6.0,
                tooling_edge_clearance: base.clearance * 3.0,
                ..production
            },
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{AssemblyBaseRules, AssemblyPolicyConfig, AssemblyProfile, AssemblyRules};

    #[test]
    fn production_profile_preserves_existing_multiplier_defaults() {
        let rules =
            AssemblyRules::resolve(AssemblyProfile::ProductionSmt, &Default::default(), base());

        assert_eq!(rules.component_edge_clearance, 0.5);
        assert!((rules.connector_min_pad_dimension - 0.45).abs() < 1.0e-12);
        assert_eq!(rules.testpoint_min_spacing, 0.6);
        assert_eq!(rules.dense_pad_via_search_radius, 1.5);
    }

    #[test]
    fn assembly_profile_fields_can_be_overridden_individually() {
        let rules = AssemblyRules::resolve(
            AssemblyProfile::TestFixture,
            &AssemblyPolicyConfig {
                testpoint_min_diameter: Some(0.42),
                dense_pad_pitch: Some(0.7),
                ..Default::default()
            },
            base(),
        );

        assert_eq!(rules.profile, AssemblyProfile::TestFixture);
        assert_eq!(rules.testpoint_min_diameter, 0.42);
        assert_eq!(rules.dense_pad_pitch, 0.7);
        assert_eq!(rules.testpoint_edge_clearance, 0.75);
    }

    fn base() -> AssemblyBaseRules {
        AssemblyBaseRules {
            clearance: 0.25,
            min_width: 0.15,
            net_clearance: 0.15,
        }
    }
}
