//! Package-profile and manifest requirement policy.
//!
//! Production handoffs vary: a fab-only quote package should not need centroid
//! data, while an assembly release normally should. These profiles provide
//! conservative defaults that can still be overridden field-by-field in the rule
//! deck.

use serde::Deserialize;

#[derive(Copy, Clone, Debug, Deserialize, Eq, PartialEq)]
#[serde(rename_all = "kebab-case")]
pub enum PackageProfile {
    FullProduction,
    FabricationOnly,
    AssemblyOnly,
    ElectricalTest,
}

impl Default for PackageProfile {
    fn default() -> Self {
        Self::FullProduction
    }
}

#[derive(Clone, Debug, Deserialize, Default)]
#[serde(default)]
pub struct ArtifactRequirementsConfig {
    pub bom: Option<bool>,
    pub centroid: Option<bool>,
    pub netlist: Option<bool>,
    pub fab_drawing: Option<bool>,
    pub assembly_drawing: Option<bool>,
    pub readme: Option<bool>,
    pub rout_drawing: Option<bool>,
}

#[derive(Clone, Debug, Deserialize, Default)]
#[serde(default)]
pub struct LayerRequirementsConfig {
    pub board_outline: Option<bool>,
    pub drill_data: Option<bool>,
    pub top_mask: Option<bool>,
    pub bottom_mask: Option<bool>,
    pub top_paste: Option<bool>,
    pub bottom_paste: Option<bool>,
    pub top_silkscreen: Option<bool>,
    pub bottom_silkscreen: Option<bool>,
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct ArtifactRequirements {
    pub bom: bool,
    pub centroid: bool,
    pub netlist: bool,
    pub fab_drawing: bool,
    pub assembly_drawing: bool,
    pub readme: bool,
    pub rout_drawing: bool,
}

impl ArtifactRequirements {
    pub fn resolve(profile: PackageProfile, config: &ArtifactRequirementsConfig) -> Self {
        let defaults = Self::for_profile(profile);
        Self {
            bom: config.bom.unwrap_or(defaults.bom),
            centroid: config.centroid.unwrap_or(defaults.centroid),
            netlist: config.netlist.unwrap_or(defaults.netlist),
            fab_drawing: config.fab_drawing.unwrap_or(defaults.fab_drawing),
            assembly_drawing: config.assembly_drawing.unwrap_or(defaults.assembly_drawing),
            readme: config.readme.unwrap_or(defaults.readme),
            rout_drawing: config.rout_drawing.unwrap_or(defaults.rout_drawing),
        }
    }

    fn for_profile(profile: PackageProfile) -> Self {
        match profile {
            PackageProfile::FullProduction => Self {
                bom: true,
                centroid: true,
                netlist: true,
                fab_drawing: true,
                assembly_drawing: true,
                readme: true,
                rout_drawing: true,
            },
            PackageProfile::FabricationOnly => Self {
                bom: false,
                centroid: false,
                netlist: false,
                fab_drawing: true,
                assembly_drawing: false,
                readme: true,
                rout_drawing: false,
            },
            PackageProfile::AssemblyOnly => Self {
                bom: true,
                centroid: true,
                netlist: false,
                fab_drawing: false,
                assembly_drawing: true,
                readme: true,
                rout_drawing: false,
            },
            PackageProfile::ElectricalTest => Self {
                bom: false,
                centroid: false,
                netlist: true,
                fab_drawing: false,
                assembly_drawing: false,
                readme: true,
                rout_drawing: false,
            },
        }
    }
}

impl Default for ArtifactRequirements {
    fn default() -> Self {
        Self::for_profile(PackageProfile::FullProduction)
    }
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct LayerRequirements {
    pub board_outline: bool,
    pub drill_data: bool,
    pub top_mask: bool,
    pub bottom_mask: bool,
    pub top_paste: bool,
    pub bottom_paste: bool,
    pub top_silkscreen: bool,
    pub bottom_silkscreen: bool,
}

impl LayerRequirements {
    pub fn resolve(profile: PackageProfile, config: &LayerRequirementsConfig) -> Self {
        let defaults = Self::for_profile(profile);
        Self {
            board_outline: config.board_outline.unwrap_or(defaults.board_outline),
            drill_data: config.drill_data.unwrap_or(defaults.drill_data),
            top_mask: config.top_mask.unwrap_or(defaults.top_mask),
            bottom_mask: config.bottom_mask.unwrap_or(defaults.bottom_mask),
            top_paste: config.top_paste.unwrap_or(defaults.top_paste),
            bottom_paste: config.bottom_paste.unwrap_or(defaults.bottom_paste),
            top_silkscreen: config.top_silkscreen.unwrap_or(defaults.top_silkscreen),
            bottom_silkscreen: config
                .bottom_silkscreen
                .unwrap_or(defaults.bottom_silkscreen),
        }
    }

    fn for_profile(profile: PackageProfile) -> Self {
        match profile {
            PackageProfile::FullProduction => Self {
                board_outline: true,
                drill_data: true,
                top_mask: true,
                bottom_mask: true,
                top_paste: true,
                bottom_paste: true,
                top_silkscreen: true,
                bottom_silkscreen: true,
            },
            PackageProfile::FabricationOnly => Self {
                board_outline: true,
                drill_data: true,
                top_mask: true,
                bottom_mask: true,
                top_paste: false,
                bottom_paste: false,
                top_silkscreen: false,
                bottom_silkscreen: false,
            },
            PackageProfile::AssemblyOnly => Self {
                board_outline: false,
                drill_data: false,
                top_mask: false,
                bottom_mask: false,
                top_paste: true,
                bottom_paste: true,
                top_silkscreen: true,
                bottom_silkscreen: true,
            },
            PackageProfile::ElectricalTest => Self {
                board_outline: false,
                drill_data: false,
                top_mask: false,
                bottom_mask: false,
                top_paste: false,
                bottom_paste: false,
                top_silkscreen: false,
                bottom_silkscreen: false,
            },
        }
    }
}

impl Default for LayerRequirements {
    fn default() -> Self {
        Self::for_profile(PackageProfile::FullProduction)
    }
}

#[cfg(test)]
mod tests {
    use super::{
        ArtifactRequirements, ArtifactRequirementsConfig, LayerRequirements,
        LayerRequirementsConfig, PackageProfile,
    };

    #[test]
    fn fabrication_profile_requires_fab_not_assembly_outputs() {
        let artifacts =
            ArtifactRequirements::resolve(PackageProfile::FabricationOnly, &Default::default());
        let layers =
            LayerRequirements::resolve(PackageProfile::FabricationOnly, &Default::default());

        assert!(artifacts.fab_drawing);
        assert!(!artifacts.bom);
        assert!(!artifacts.centroid);
        assert!(!artifacts.assembly_drawing);
        assert!(layers.board_outline);
        assert!(layers.drill_data);
        assert!(!layers.top_paste);
        assert!(!layers.top_silkscreen);
    }

    #[test]
    fn package_profile_can_be_overridden_field_by_field() {
        let artifacts = ArtifactRequirements::resolve(
            PackageProfile::FabricationOnly,
            &ArtifactRequirementsConfig {
                bom: Some(true),
                ..Default::default()
            },
        );
        let layers = LayerRequirements::resolve(
            PackageProfile::AssemblyOnly,
            &LayerRequirementsConfig {
                drill_data: Some(true),
                ..Default::default()
            },
        );

        assert!(artifacts.bom);
        assert!(artifacts.fab_drawing);
        assert!(layers.drill_data);
        assert!(layers.top_paste);
    }
}
