//! Config structures for stackup and net-class readiness policy.
//!
//! These types are deserialized from the JSON rule deck and interpreted by the
//! constraint checks. Keeping them outside `config.rs` makes the config loader
//! easier to scan while keeping electrical/manufacturing policy fields together.

use serde::Deserialize;

#[derive(Clone, Debug, Deserialize, Default)]
#[serde(default)]
/// Public data model for `StackupConfig`.
pub struct StackupConfig {
    /// Field `copper_layer_count`.
    pub copper_layer_count: Option<usize>,
    /// Field `finished_thickness`.
    pub finished_thickness: Option<f64>,
    /// Field `impedance_controlled`.
    pub impedance_controlled: Option<bool>,
    /// Field `material_family`.
    pub material_family: Option<String>,
    /// Field `material_dielectric_constant`.
    pub material_dielectric_constant: Option<f64>,
    /// Field `material_loss_tangent`.
    pub material_loss_tangent: Option<f64>,
    /// Field `material_tg_c`.
    pub material_tg_c: Option<f64>,
    /// Field `surface_finish`.
    pub surface_finish: Option<SurfaceFinish>,
    /// Field `soldermask_process`.
    pub soldermask_process: Option<String>,
    /// Field `soldermask_color`.
    pub soldermask_color: Option<String>,
    /// Field `target_ipc_class`.
    pub target_ipc_class: Option<String>,
    /// Field `fabricator_profile`.
    pub fabricator_profile: Option<String>,
    /// Field `fabrication_capability`.
    pub fabrication_capability: FabricationCapabilityConfig,
    /// Field `layers`.
    pub layers: Vec<StackupLayerConfig>,
}

#[derive(Clone, Debug, Deserialize, Default)]
#[serde(default)]
/// Public data model for `FabricationCapabilityConfig`.
pub struct FabricationCapabilityConfig {
    /// Field `min_finished_thickness`.
    pub min_finished_thickness: Option<f64>,
    /// Field `max_finished_thickness`.
    pub max_finished_thickness: Option<f64>,
    /// Field `max_copper_layers`.
    pub max_copper_layers: Option<usize>,
    /// Field `min_copper_weight_oz`.
    pub min_copper_weight_oz: Option<f64>,
    /// Field `max_copper_weight_oz`.
    pub max_copper_weight_oz: Option<f64>,
    /// Field `min_dielectric_thickness`.
    pub min_dielectric_thickness: Option<f64>,
    /// Field `min_dielectric_constant`.
    pub min_dielectric_constant: Option<f64>,
    /// Field `max_dielectric_constant`.
    pub max_dielectric_constant: Option<f64>,
    /// Field `max_loss_tangent`.
    pub max_loss_tangent: Option<f64>,
    /// Field `min_tg_c`.
    pub min_tg_c: Option<f64>,
}

#[derive(Clone, Debug, Deserialize, Default)]
#[serde(default)]
/// Public data model for `StackupLayerConfig`.
pub struct StackupLayerConfig {
    /// Field `name`.
    pub name: String,
    /// Field `kind`.
    pub kind: StackupLayerKind,
    /// Field `copper_weight_oz`.
    pub copper_weight_oz: Option<f64>,
    /// Field `dielectric_thickness`.
    pub dielectric_thickness: Option<f64>,
}

#[derive(Copy, Clone, Debug, Deserialize, Default, Eq, PartialEq)]
#[serde(rename_all = "kebab-case")]
/// Public enumeration for `StackupLayerKind`.
pub enum StackupLayerKind {
    /// Variant `Copper`.
    Copper,
    /// Variant `Dielectric`.
    Dielectric,
    /// Variant `SolderMask`.
    SolderMask,
    /// Variant `Silkscreen`.
    Silkscreen,
    /// Variant `Core`.
    Core,
    /// Variant `Prepreg`.
    Prepreg,
    #[default]
    /// Variant `Other`.
    Other,
}

#[derive(Copy, Clone, Debug, Deserialize, Eq, PartialEq)]
#[serde(rename_all = "kebab-case")]
/// Public enumeration for `SurfaceFinish`.
pub enum SurfaceFinish {
    /// Variant `Hasl`.
    Hasl,
    /// Variant `LeadFreeHasl`.
    LeadFreeHasl,
    /// Variant `Enig`.
    Enig,
    /// Variant `Enepig`.
    Enepig,
    /// Variant `Osp`.
    Osp,
    /// Variant `ImmersionSilver`.
    ImmersionSilver,
    /// Variant `ImmersionTin`.
    ImmersionTin,
    /// Variant `HardGold`.
    HardGold,
    /// Variant `Other`.
    Other,
}

#[derive(Clone, Debug, Deserialize, Default)]
#[serde(default)]
/// Public data model for `NetClassConfig`.
pub struct NetClassConfig {
    /// Field `name`.
    pub name: String,
    /// Field `extends`.
    ///
    /// Parent class names contribute unset constraint fields. Net selectors
    /// (`nets` and `net_patterns`) stay local to each class so abstract parent
    /// classes can safely carry only policy defaults.
    pub extends: Vec<String>,
    /// Field `nets`.
    pub nets: Vec<String>,
    /// Field `net_patterns`.
    pub net_patterns: Vec<String>,
    /// Field `regions`.
    ///
    /// Optional rectangular scoping windows. When present, this class applies
    /// only to matching-net copper whose parsed feature location falls inside
    /// at least one region. Parent class regions are not inherited because they
    /// are selectors, not scalar policy defaults.
    pub regions: Vec<NetClassRegionConfig>,
    /// Field `min_width`.
    pub min_width: Option<f64>,
    /// Field `min_clearance`.
    pub min_clearance: Option<f64>,
    /// Field `max_layer_count`.
    pub max_layer_count: Option<usize>,
    /// Field `min_via_count`.
    pub min_via_count: Option<usize>,
    /// Field `min_current_width`.
    pub min_current_width: Option<f64>,
    /// Field `min_voltage_clearance`.
    pub min_voltage_clearance: Option<f64>,
    /// Field `requires_reference_plane`.
    pub requires_reference_plane: Option<bool>,
    /// Field `requires_impedance_control`.
    pub requires_impedance_control: Option<bool>,
    /// Field `target_impedance_ohms`.
    pub target_impedance_ohms: Option<f64>,
    /// Field `impedance_tolerance_ohms`.
    pub impedance_tolerance_ohms: Option<f64>,
    /// Field `differential_pair`.
    pub differential_pair: Option<String>,
    /// Field `differential_role`.
    pub differential_role: Option<DifferentialRole>,
    /// Field `min_pair_spacing`.
    pub min_pair_spacing: Option<f64>,
    /// Field `max_pair_spacing`.
    pub max_pair_spacing: Option<f64>,
    /// Field `max_length`.
    pub max_length: Option<f64>,
    /// Field `max_pair_skew`.
    pub max_pair_skew: Option<f64>,
    /// Field `max_via_count`.
    pub max_via_count: Option<usize>,
}

#[derive(Clone, Debug, Deserialize, Default)]
#[serde(default)]
/// Public data model for `NetClassRegionConfig`.
pub struct NetClassRegionConfig {
    /// Field `name`.
    pub name: String,
    /// Field `min_x`.
    pub min_x: Option<f64>,
    /// Field `min_y`.
    pub min_y: Option<f64>,
    /// Field `max_x`.
    pub max_x: Option<f64>,
    /// Field `max_y`.
    pub max_y: Option<f64>,
    /// Field `layers`.
    pub layers: Vec<String>,
}

#[derive(Copy, Clone, Debug, Deserialize, Eq, PartialEq)]
#[serde(rename_all = "kebab-case")]
/// Public enumeration for `DifferentialRole`.
pub enum DifferentialRole {
    /// Variant `Positive`.
    Positive,
    /// Variant `Negative`.
    Negative,
}
