//! Config structures for stackup and net-class readiness policy.
//!
//! These types are deserialized from the JSON rule deck and interpreted by the
//! constraint checks. Keeping them outside `config.rs` makes the config loader
//! easier to scan while keeping electrical/manufacturing policy fields together.

use serde::Deserialize;

#[derive(Clone, Debug, Deserialize, Default)]
#[serde(default)]
pub struct StackupConfig {
    pub copper_layer_count: Option<usize>,
    pub finished_thickness: Option<f64>,
    pub impedance_controlled: Option<bool>,
    pub material_family: Option<String>,
    pub material_dielectric_constant: Option<f64>,
    pub material_loss_tangent: Option<f64>,
    pub material_tg_c: Option<f64>,
    pub surface_finish: Option<SurfaceFinish>,
    pub soldermask_process: Option<String>,
    pub soldermask_color: Option<String>,
    pub target_ipc_class: Option<String>,
    pub fabricator_profile: Option<String>,
    pub fabrication_capability: FabricationCapabilityConfig,
    pub layers: Vec<StackupLayerConfig>,
}

#[derive(Clone, Debug, Deserialize, Default)]
#[serde(default)]
pub struct FabricationCapabilityConfig {
    pub min_finished_thickness: Option<f64>,
    pub max_finished_thickness: Option<f64>,
    pub max_copper_layers: Option<usize>,
    pub min_copper_weight_oz: Option<f64>,
    pub max_copper_weight_oz: Option<f64>,
    pub min_dielectric_thickness: Option<f64>,
    pub min_dielectric_constant: Option<f64>,
    pub max_dielectric_constant: Option<f64>,
    pub max_loss_tangent: Option<f64>,
    pub min_tg_c: Option<f64>,
}

#[derive(Clone, Debug, Deserialize, Default)]
#[serde(default)]
pub struct StackupLayerConfig {
    pub name: String,
    pub kind: StackupLayerKind,
    pub copper_weight_oz: Option<f64>,
    pub dielectric_thickness: Option<f64>,
}

#[derive(Copy, Clone, Debug, Deserialize, Default, Eq, PartialEq)]
#[serde(rename_all = "kebab-case")]
pub enum StackupLayerKind {
    Copper,
    Dielectric,
    SolderMask,
    Silkscreen,
    Core,
    Prepreg,
    #[default]
    Other,
}

#[derive(Copy, Clone, Debug, Deserialize, Eq, PartialEq)]
#[serde(rename_all = "kebab-case")]
pub enum SurfaceFinish {
    Hasl,
    LeadFreeHasl,
    Enig,
    Enepig,
    Osp,
    ImmersionSilver,
    ImmersionTin,
    HardGold,
    Other,
}

#[derive(Clone, Debug, Deserialize, Default)]
#[serde(default)]
pub struct NetClassConfig {
    pub name: String,
    pub nets: Vec<String>,
    pub net_patterns: Vec<String>,
    pub min_width: Option<f64>,
    pub min_clearance: Option<f64>,
    pub max_layer_count: Option<usize>,
    pub min_via_count: Option<usize>,
    pub min_current_width: Option<f64>,
    pub min_voltage_clearance: Option<f64>,
    pub requires_reference_plane: Option<bool>,
    pub requires_impedance_control: Option<bool>,
    pub target_impedance_ohms: Option<f64>,
    pub impedance_tolerance_ohms: Option<f64>,
    pub differential_pair: Option<String>,
    pub differential_role: Option<DifferentialRole>,
    pub min_pair_spacing: Option<f64>,
    pub max_pair_spacing: Option<f64>,
    pub max_length: Option<f64>,
    pub max_pair_skew: Option<f64>,
    pub max_via_count: Option<usize>,
}

#[derive(Copy, Clone, Debug, Deserialize, Eq, PartialEq)]
#[serde(rename_all = "kebab-case")]
pub enum DifferentialRole {
    Positive,
    Negative,
}
