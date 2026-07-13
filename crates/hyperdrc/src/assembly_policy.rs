//! Assembly-readiness policy profiles and resolved thresholds.
//!
//! The assembly checks operate on geometry, but the right thresholds depend on
//! process assumptions. These profiles keep the defaults discoverable while
//! allowing a rule deck to override any individual threshold.

use serde::Deserialize;

use crate::Scalar;

macro_rules! exact {
    ($value:literal) => {
        crate::scalar::scalar(stringify!($value))
    };
}

macro_rules! override_or {
    ($configured:expr, $default:expr) => {
        $configured.clone().unwrap_or_else(|| $default.clone())
    };
}

#[derive(Copy, Clone, Debug, Deserialize, Eq, PartialEq)]
#[serde(rename_all = "kebab-case")]
/// Assembly process profile used to select default readiness thresholds.
pub enum AssemblyProfile {
    /// Early prototype assembly with relaxed production assumptions.
    Prototype,
    /// Standard production SMT assembly.
    ProductionSmt,
    /// SMT assembly with populated top and bottom sides.
    DoubleSidedSmt,
    /// Test fixture or bed-of-nails focused build.
    TestFixture,
    /// Manual or low-volume hand assembly.
    HandAssembly,
    /// Selective solder process.
    SelectiveSolder,
    /// Wave solder process.
    WaveSolder,
    /// Press-fit connector process.
    PressFit,
    /// Assembly requiring conformal coating clearance.
    ConformalCoating,
}

impl Default for AssemblyProfile {
    fn default() -> Self {
        Self::ProductionSmt
    }
}

#[derive(Clone, Debug, Deserialize, Default)]
#[serde(default)]
/// Optional assembly threshold overrides from rule configuration.
pub struct AssemblyPolicyConfig {
    /// Override for component-to-board-edge clearance.
    #[serde(default, deserialize_with = "crate::scalar::deserialize_optional")]
    pub component_edge_clearance: Option<Scalar>,
    /// Override for component-to-hole clearance.
    #[serde(default, deserialize_with = "crate::scalar::deserialize_optional")]
    pub component_hole_clearance: Option<Scalar>,
    /// Override for connector rework access clearance.
    #[serde(default, deserialize_with = "crate::scalar::deserialize_optional")]
    pub connector_rework_clearance: Option<Scalar>,
    /// Override for minimum connector pad dimension used as a proxy.
    #[serde(default, deserialize_with = "crate::scalar::deserialize_optional")]
    pub connector_min_pad_dimension: Option<Scalar>,
    /// Override for neighboring pad-pair gap.
    #[serde(default, deserialize_with = "crate::scalar::deserialize_optional")]
    pub pad_pair_max_gap: Option<Scalar>,
    /// Override for neighboring pad-pair area ratio.
    #[serde(default, deserialize_with = "crate::scalar::deserialize_optional")]
    pub pad_pair_max_area_ratio: Option<Scalar>,
    /// Override for maximum pad dimension considered by pad-pair checks.
    #[serde(default, deserialize_with = "crate::scalar::deserialize_optional")]
    pub pad_pair_max_pad_dimension: Option<Scalar>,
    /// Override for minimum testpoint diameter.
    #[serde(default, deserialize_with = "crate::scalar::deserialize_optional")]
    pub testpoint_min_diameter: Option<Scalar>,
    /// Override for minimum testpoint spacing.
    #[serde(default, deserialize_with = "crate::scalar::deserialize_optional")]
    pub testpoint_min_spacing: Option<Scalar>,
    /// Override for testpoint-to-edge clearance.
    #[serde(default, deserialize_with = "crate::scalar::deserialize_optional")]
    pub testpoint_edge_clearance: Option<Scalar>,
    /// Override for minimum tooling-hole diameter.
    #[serde(default, deserialize_with = "crate::scalar::deserialize_optional")]
    pub tooling_min_diameter: Option<Scalar>,
    /// Override for maximum tooling-hole diameter.
    #[serde(default, deserialize_with = "crate::scalar::deserialize_optional")]
    pub tooling_max_diameter: Option<Scalar>,
    /// Override for tooling-hole edge clearance.
    #[serde(default, deserialize_with = "crate::scalar::deserialize_optional")]
    pub tooling_edge_clearance: Option<Scalar>,
    /// Override for minimum mouse-bite hole diameter.
    #[serde(default, deserialize_with = "crate::scalar::deserialize_optional")]
    pub mouse_bite_min_diameter: Option<Scalar>,
    /// Override for maximum mouse-bite hole diameter.
    #[serde(default, deserialize_with = "crate::scalar::deserialize_optional")]
    pub mouse_bite_max_diameter: Option<Scalar>,
    /// Override for minimum mouse-bite pitch.
    #[serde(default, deserialize_with = "crate::scalar::deserialize_optional")]
    pub mouse_bite_min_spacing: Option<Scalar>,
    /// Override for maximum mouse-bite pitch.
    #[serde(default, deserialize_with = "crate::scalar::deserialize_optional")]
    pub mouse_bite_max_spacing: Option<Scalar>,
    /// Override for fiducial-to-edge clearance.
    #[serde(default, deserialize_with = "crate::scalar::deserialize_optional")]
    pub fiducial_edge_clearance: Option<Scalar>,
    /// Override for expected local fiducial pitch.
    #[serde(default, deserialize_with = "crate::scalar::deserialize_optional")]
    pub local_fiducial_pitch: Option<Scalar>,
    /// Override for local fiducial search radius.
    #[serde(default, deserialize_with = "crate::scalar::deserialize_optional")]
    pub local_fiducial_search_radius: Option<Scalar>,
    /// Override for dense-pad pitch threshold.
    #[serde(default, deserialize_with = "crate::scalar::deserialize_optional")]
    pub dense_pad_pitch: Option<Scalar>,
    /// Override for via search radius around dense pads.
    #[serde(default, deserialize_with = "crate::scalar::deserialize_optional")]
    pub dense_pad_via_search_radius: Option<Scalar>,
    /// Override for selective-solder process keepout.
    #[serde(default, deserialize_with = "crate::scalar::deserialize_optional")]
    pub selective_solder_keepout: Option<Scalar>,
    /// Override for wave-solder process keepout.
    #[serde(default, deserialize_with = "crate::scalar::deserialize_optional")]
    pub wave_solder_keepout: Option<Scalar>,
    /// Override for press-fit process keepout.
    #[serde(default, deserialize_with = "crate::scalar::deserialize_optional")]
    pub press_fit_keepout: Option<Scalar>,
    /// Override for conformal-coating process keepout.
    #[serde(default, deserialize_with = "crate::scalar::deserialize_optional")]
    pub conformal_coating_keepout: Option<Scalar>,
}

#[derive(Clone, Debug, PartialEq)]
/// Base fabrication thresholds used to derive assembly defaults.
pub struct AssemblyBaseRules {
    /// General clearance in millimeters.
    pub clearance: Scalar,
    /// Minimum feature width in millimeters.
    pub min_width: Scalar,
    /// Electrical net clearance in millimeters.
    pub net_clearance: Scalar,
}

#[derive(Clone, Debug, PartialEq)]
/// Fully resolved assembly readiness thresholds.
pub struct AssemblyRules {
    /// Profile that supplied the defaults.
    pub profile: AssemblyProfile,
    /// Required component-to-board-edge clearance.
    pub component_edge_clearance: Scalar,
    /// Required component-to-hole clearance.
    pub component_hole_clearance: Scalar,
    /// Required connector rework access clearance.
    pub connector_rework_clearance: Scalar,
    /// Minimum connector pad dimension used as a proxy.
    pub connector_min_pad_dimension: Scalar,
    /// Maximum gap between pads considered a pair.
    pub pad_pair_max_gap: Scalar,
    /// Maximum allowed neighboring pad area ratio.
    pub pad_pair_max_area_ratio: Scalar,
    /// Maximum pad dimension considered by pad-pair checks.
    pub pad_pair_max_pad_dimension: Scalar,
    /// Minimum testpoint diameter.
    pub testpoint_min_diameter: Scalar,
    /// Minimum testpoint spacing.
    pub testpoint_min_spacing: Scalar,
    /// Required testpoint-to-edge clearance.
    pub testpoint_edge_clearance: Scalar,
    /// Minimum tooling-hole diameter.
    pub tooling_min_diameter: Scalar,
    /// Maximum tooling-hole diameter.
    pub tooling_max_diameter: Scalar,
    /// Required tooling-hole edge clearance.
    pub tooling_edge_clearance: Scalar,
    /// Minimum mouse-bite hole diameter.
    pub mouse_bite_min_diameter: Scalar,
    /// Maximum mouse-bite hole diameter.
    pub mouse_bite_max_diameter: Scalar,
    /// Minimum mouse-bite pitch.
    pub mouse_bite_min_spacing: Scalar,
    /// Maximum mouse-bite pitch.
    pub mouse_bite_max_spacing: Scalar,
    /// Required fiducial-to-edge clearance.
    pub fiducial_edge_clearance: Scalar,
    /// Expected pitch threshold for local fiducials.
    pub local_fiducial_pitch: Scalar,
    /// Search radius for local fiducials.
    pub local_fiducial_search_radius: Scalar,
    /// Dense-pad pitch threshold.
    pub dense_pad_pitch: Scalar,
    /// Via search radius around dense pads.
    pub dense_pad_via_search_radius: Scalar,
    /// Selective-solder process keepout.
    pub selective_solder_keepout: Scalar,
    /// Wave-solder process keepout.
    pub wave_solder_keepout: Scalar,
    /// Press-fit process keepout.
    pub press_fit_keepout: Scalar,
    /// Conformal-coating process keepout.
    pub conformal_coating_keepout: Scalar,
}

impl AssemblyRules {
    /// Resolve a profile plus optional overrides into concrete thresholds.
    pub fn resolve(
        profile: AssemblyProfile,
        config: &AssemblyPolicyConfig,
        base: AssemblyBaseRules,
    ) -> Self {
        let defaults = Self::for_profile(profile, base);
        Self {
            profile,
            component_edge_clearance: override_or!(
                config.component_edge_clearance,
                defaults.component_edge_clearance
            ),
            component_hole_clearance: override_or!(
                config.component_hole_clearance,
                defaults.component_hole_clearance
            ),
            connector_rework_clearance: override_or!(
                config.connector_rework_clearance,
                defaults.connector_rework_clearance
            ),
            connector_min_pad_dimension: override_or!(
                config.connector_min_pad_dimension,
                defaults.connector_min_pad_dimension
            ),
            pad_pair_max_gap: override_or!(config.pad_pair_max_gap, defaults.pad_pair_max_gap),
            pad_pair_max_area_ratio: override_or!(
                config.pad_pair_max_area_ratio,
                defaults.pad_pair_max_area_ratio
            ),
            pad_pair_max_pad_dimension: override_or!(
                config.pad_pair_max_pad_dimension,
                defaults.pad_pair_max_pad_dimension
            ),
            testpoint_min_diameter: override_or!(
                config.testpoint_min_diameter,
                defaults.testpoint_min_diameter
            ),
            testpoint_min_spacing: override_or!(
                config.testpoint_min_spacing,
                defaults.testpoint_min_spacing
            ),
            testpoint_edge_clearance: override_or!(
                config.testpoint_edge_clearance,
                defaults.testpoint_edge_clearance
            ),
            tooling_min_diameter: override_or!(
                config.tooling_min_diameter,
                defaults.tooling_min_diameter
            ),
            tooling_max_diameter: override_or!(
                config.tooling_max_diameter,
                defaults.tooling_max_diameter
            ),
            tooling_edge_clearance: override_or!(
                config.tooling_edge_clearance,
                defaults.tooling_edge_clearance
            ),
            mouse_bite_min_diameter: override_or!(
                config.mouse_bite_min_diameter,
                defaults.mouse_bite_min_diameter
            ),
            mouse_bite_max_diameter: override_or!(
                config.mouse_bite_max_diameter,
                defaults.mouse_bite_max_diameter
            ),
            mouse_bite_min_spacing: override_or!(
                config.mouse_bite_min_spacing,
                defaults.mouse_bite_min_spacing
            ),
            mouse_bite_max_spacing: override_or!(
                config.mouse_bite_max_spacing,
                defaults.mouse_bite_max_spacing
            ),
            fiducial_edge_clearance: override_or!(
                config.fiducial_edge_clearance,
                defaults.fiducial_edge_clearance
            ),
            local_fiducial_pitch: override_or!(
                config.local_fiducial_pitch,
                defaults.local_fiducial_pitch
            ),
            local_fiducial_search_radius: override_or!(
                config.local_fiducial_search_radius,
                defaults.local_fiducial_search_radius
            ),
            dense_pad_pitch: override_or!(config.dense_pad_pitch, defaults.dense_pad_pitch),
            dense_pad_via_search_radius: override_or!(
                config.dense_pad_via_search_radius,
                defaults.dense_pad_via_search_radius
            ),
            selective_solder_keepout: override_or!(
                config.selective_solder_keepout,
                defaults.selective_solder_keepout
            ),
            wave_solder_keepout: override_or!(
                config.wave_solder_keepout,
                defaults.wave_solder_keepout
            ),
            press_fit_keepout: override_or!(config.press_fit_keepout, defaults.press_fit_keepout),
            conformal_coating_keepout: override_or!(
                config.conformal_coating_keepout,
                defaults.conformal_coating_keepout
            ),
        }
    }

    fn for_profile(profile: AssemblyProfile, base: AssemblyBaseRules) -> Self {
        // Defaults preserve the historical hyperdrc multipliers while grouping
        // them by assembly process. IPC-7351B frames land-pattern geometry as
        // assembly-process dependent, IPC-9252B/fixture practice makes probe
        // size and spacing process constraints, and IPC J-STD-001H treats
        // soldering, press-fit, coating, cleanliness, and assembly workmanship
        // as process-controlled acceptance criteria rather than universal
        // geometry constants.
        let production = Self {
            profile,
            component_edge_clearance: &base.clearance * exact!(2.0),
            component_hole_clearance: &base.clearance * exact!(2.0),
            connector_rework_clearance: &base.clearance * exact!(2.0),
            connector_min_pad_dimension: &base.min_width * exact!(3.0),
            pad_pair_max_gap: &base.min_width * exact!(8.0),
            pad_pair_max_area_ratio: exact!(1.5),
            pad_pair_max_pad_dimension: &base.min_width * exact!(10.0),
            testpoint_min_diameter: base.min_width.clone(),
            testpoint_min_spacing: &base.net_clearance * exact!(4.0),
            testpoint_edge_clearance: &base.clearance * exact!(2.0),
            tooling_min_diameter: &base.min_width * exact!(4.0),
            tooling_max_diameter: &base.min_width * exact!(20.0),
            tooling_edge_clearance: &base.clearance * exact!(2.0),
            mouse_bite_min_diameter: base.min_width.clone(),
            mouse_bite_max_diameter: &base.min_width * exact!(4.0),
            mouse_bite_min_spacing: &base.min_width * exact!(2.0),
            mouse_bite_max_spacing: &base.min_width * exact!(8.0),
            fiducial_edge_clearance: &base.clearance * exact!(2.0),
            local_fiducial_pitch: exact!(0.8),
            local_fiducial_search_radius: &base.net_clearance * exact!(25.0),
            dense_pad_pitch: exact!(0.8),
            dense_pad_via_search_radius: &base.net_clearance * exact!(10.0),
            selective_solder_keepout: &base.clearance * exact!(3.0),
            wave_solder_keepout: &base.clearance * exact!(4.0),
            press_fit_keepout: &base.clearance * exact!(5.0),
            conformal_coating_keepout: &base.clearance * exact!(3.0),
        };

        match profile {
            AssemblyProfile::ProductionSmt => production,
            AssemblyProfile::Prototype => Self {
                component_edge_clearance: base.clearance.clone(),
                component_hole_clearance: base.clearance.clone(),
                connector_rework_clearance: base.clearance.clone(),
                pad_pair_max_area_ratio: exact!(2.0),
                local_fiducial_pitch: exact!(0.65),
                dense_pad_pitch: exact!(0.65),
                ..production
            },
            AssemblyProfile::DoubleSidedSmt => Self {
                component_edge_clearance: &base.clearance * exact!(2.5),
                component_hole_clearance: &base.clearance * exact!(2.5),
                fiducial_edge_clearance: &base.clearance * exact!(2.5),
                local_fiducial_search_radius: &base.net_clearance * exact!(20.0),
                dense_pad_via_search_radius: &base.net_clearance * exact!(8.0),
                ..production
            },
            AssemblyProfile::TestFixture => Self {
                testpoint_min_diameter: &base.min_width * exact!(1.5),
                testpoint_min_spacing: &base.net_clearance * exact!(6.0),
                testpoint_edge_clearance: &base.clearance * exact!(3.0),
                tooling_min_diameter: &base.min_width * exact!(6.0),
                tooling_edge_clearance: &base.clearance * exact!(3.0),
                ..production
            },
            AssemblyProfile::HandAssembly => Self {
                component_edge_clearance: &base.clearance * exact!(3.0),
                component_hole_clearance: &base.clearance * exact!(3.0),
                connector_rework_clearance: &base.clearance * exact!(4.0),
                connector_min_pad_dimension: &base.min_width * exact!(4.0),
                pad_pair_max_gap: &base.min_width * exact!(10.0),
                pad_pair_max_area_ratio: exact!(2.0),
                dense_pad_pitch: exact!(1.0),
                local_fiducial_pitch: exact!(1.0),
                ..production
            },
            AssemblyProfile::SelectiveSolder => Self {
                component_edge_clearance: &base.clearance * exact!(3.0),
                component_hole_clearance: &base.clearance * exact!(4.0),
                connector_rework_clearance: &base.clearance * exact!(3.0),
                tooling_edge_clearance: &base.clearance * exact!(3.0),
                mouse_bite_min_spacing: &base.min_width * exact!(3.0),
                mouse_bite_max_spacing: &base.min_width * exact!(10.0),
                selective_solder_keepout: &base.clearance * exact!(4.0),
                ..production
            },
            AssemblyProfile::WaveSolder => Self {
                component_edge_clearance: &base.clearance * exact!(4.0),
                component_hole_clearance: &base.clearance * exact!(5.0),
                connector_rework_clearance: &base.clearance * exact!(3.0),
                pad_pair_max_gap: &base.min_width * exact!(10.0),
                pad_pair_max_area_ratio: exact!(1.25),
                tooling_edge_clearance: &base.clearance * exact!(4.0),
                mouse_bite_min_spacing: &base.min_width * exact!(4.0),
                mouse_bite_max_spacing: &base.min_width * exact!(12.0),
                wave_solder_keepout: &base.clearance * exact!(5.0),
                ..production
            },
            AssemblyProfile::PressFit => Self {
                component_edge_clearance: &base.clearance * exact!(3.0),
                component_hole_clearance: &base.clearance * exact!(6.0),
                connector_rework_clearance: &base.clearance * exact!(5.0),
                connector_min_pad_dimension: &base.min_width * exact!(5.0),
                tooling_min_diameter: &base.min_width * exact!(6.0),
                tooling_edge_clearance: &base.clearance * exact!(4.0),
                press_fit_keepout: &base.clearance * exact!(6.0),
                ..production
            },
            AssemblyProfile::ConformalCoating => Self {
                component_edge_clearance: &base.clearance * exact!(3.0),
                component_hole_clearance: &base.clearance * exact!(3.0),
                connector_rework_clearance: &base.clearance * exact!(4.0),
                connector_min_pad_dimension: &base.min_width * exact!(4.0),
                testpoint_edge_clearance: &base.clearance * exact!(4.0),
                fiducial_edge_clearance: &base.clearance * exact!(3.0),
                tooling_edge_clearance: &base.clearance * exact!(3.0),
                conformal_coating_keepout: &base.clearance * exact!(5.0),
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

        assert_eq!(rules.component_edge_clearance, crate::scalar::scalar("0.5"));
        assert_eq!(
            rules.connector_min_pad_dimension,
            crate::scalar::scalar("0.45")
        );
        assert_eq!(rules.testpoint_min_spacing, crate::scalar::scalar("0.6"));
        assert_eq!(
            rules.dense_pad_via_search_radius,
            crate::scalar::scalar("1.5")
        );
    }

    #[test]
    fn assembly_profile_fields_can_be_overridden_individually() {
        let rules = AssemblyRules::resolve(
            AssemblyProfile::TestFixture,
            &AssemblyPolicyConfig {
                testpoint_min_diameter: Some(crate::scalar::scalar("0.42")),
                dense_pad_pitch: Some(crate::scalar::scalar("0.7")),
                ..Default::default()
            },
            base(),
        );

        assert_eq!(rules.profile, AssemblyProfile::TestFixture);
        assert_eq!(rules.testpoint_min_diameter, crate::scalar::scalar("0.42"));
        assert_eq!(rules.dense_pad_pitch, crate::scalar::scalar("0.7"));
        assert_eq!(
            rules.testpoint_edge_clearance,
            crate::scalar::scalar("0.75")
        );
    }

    #[test]
    fn hand_assembly_profile_expands_rework_access() {
        let rules =
            AssemblyRules::resolve(AssemblyProfile::HandAssembly, &Default::default(), base());

        assert_eq!(
            rules.component_edge_clearance,
            crate::scalar::scalar("0.75")
        );
        assert_eq!(
            rules.connector_rework_clearance,
            crate::scalar::scalar("1.0")
        );
        assert_eq!(rules.dense_pad_pitch, crate::scalar::scalar("1.0"));
    }

    #[test]
    fn solder_and_press_fit_profiles_tighten_process_keepouts() {
        let selective = AssemblyRules::resolve(
            AssemblyProfile::SelectiveSolder,
            &Default::default(),
            base(),
        );
        let wave = AssemblyRules::resolve(AssemblyProfile::WaveSolder, &Default::default(), base());
        let press_fit =
            AssemblyRules::resolve(AssemblyProfile::PressFit, &Default::default(), base());

        assert_eq!(
            selective.component_hole_clearance,
            crate::scalar::scalar("1.0")
        );
        assert_eq!(
            selective.selective_solder_keepout,
            crate::scalar::scalar("1.0")
        );
        assert_eq!(wave.component_edge_clearance, crate::scalar::scalar("1.0"));
        assert_eq!(wave.pad_pair_max_area_ratio, crate::scalar::scalar("1.25"));
        assert_eq!(wave.wave_solder_keepout, crate::scalar::scalar("1.25"));
        assert_eq!(
            press_fit.component_hole_clearance,
            crate::scalar::scalar("1.5")
        );
        assert_eq!(
            press_fit.connector_rework_clearance,
            crate::scalar::scalar("1.25")
        );
        assert_eq!(press_fit.press_fit_keepout, crate::scalar::scalar("1.5"));
    }

    #[test]
    fn conformal_coating_profile_expands_masking_and_probe_edges() {
        let rules = AssemblyRules::resolve(
            AssemblyProfile::ConformalCoating,
            &Default::default(),
            base(),
        );

        assert_eq!(
            rules.connector_rework_clearance,
            crate::scalar::scalar("1.0")
        );
        assert_eq!(rules.testpoint_edge_clearance, crate::scalar::scalar("1.0"));
        assert_eq!(rules.fiducial_edge_clearance, crate::scalar::scalar("0.75"));
        assert_eq!(
            rules.conformal_coating_keepout,
            crate::scalar::scalar("1.25")
        );
    }

    fn base() -> AssemblyBaseRules {
        AssemblyBaseRules {
            clearance: crate::scalar::scalar("0.25"),
            min_width: crate::scalar::scalar("0.15"),
            net_clearance: crate::scalar::scalar("0.15"),
        }
    }
}
