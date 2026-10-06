//! Config structures for stackup and net-class readiness policy.
//!
//! These types are deserialized from the JSON rule deck and interpreted by the
//! constraint checks. Keeping them outside `config.rs` makes the config loader
//! easier to scan while keeping electrical/manufacturing policy fields together.

use crate::Scalar;
use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, Deserialize, Default)]
#[serde(default)]
/// Public data model for `StackupConfig`.
pub struct StackupConfig {
    /// Field `copper_layer_count`.
    pub copper_layer_count: Option<usize>,
    /// Field `finished_thickness`.
    #[serde(default, deserialize_with = "crate::scalar::deserialize_optional")]
    pub finished_thickness: Option<Scalar>,
    /// Field `impedance_controlled`.
    pub impedance_controlled: Option<bool>,
    /// Field `material_family`.
    pub material_family: Option<String>,
    /// Field `material_dielectric_constant`.
    #[serde(default, deserialize_with = "crate::scalar::deserialize_optional")]
    pub material_dielectric_constant: Option<Scalar>,
    /// Field `material_loss_tangent`.
    #[serde(default, deserialize_with = "crate::scalar::deserialize_optional")]
    pub material_loss_tangent: Option<Scalar>,
    /// Field `material_tg_c`.
    #[serde(default, deserialize_with = "crate::scalar::deserialize_optional")]
    pub material_tg_c: Option<Scalar>,
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

#[derive(Clone, Debug, Deserialize, Serialize, Default, PartialEq)]
#[serde(default)]
/// Public data model for `FabricationCapabilityConfig`.
pub struct FabricationCapabilityConfig {
    /// Field `min_finished_thickness`.
    #[serde(default, deserialize_with = "crate::scalar::deserialize_optional")]
    pub min_finished_thickness: Option<Scalar>,
    /// Field `preferred_min_finished_thickness`.
    #[serde(default, deserialize_with = "crate::scalar::deserialize_optional")]
    pub preferred_min_finished_thickness: Option<Scalar>,
    /// Field `preferred_max_finished_thickness`.
    #[serde(default, deserialize_with = "crate::scalar::deserialize_optional")]
    pub preferred_max_finished_thickness: Option<Scalar>,
    /// Field `max_finished_thickness`.
    #[serde(default, deserialize_with = "crate::scalar::deserialize_optional")]
    pub max_finished_thickness: Option<Scalar>,
    /// Field `max_copper_layers`.
    pub max_copper_layers: Option<usize>,
    /// Field `preferred_max_copper_layers`.
    pub preferred_max_copper_layers: Option<usize>,
    /// Field `cost_escalation_copper_layers`.
    pub cost_escalation_copper_layers: Option<usize>,
    /// Field `min_copper_weight_oz`.
    #[serde(default, deserialize_with = "crate::scalar::deserialize_optional")]
    pub min_copper_weight_oz: Option<Scalar>,
    /// Field `preferred_min_copper_weight_oz`.
    #[serde(default, deserialize_with = "crate::scalar::deserialize_optional")]
    pub preferred_min_copper_weight_oz: Option<Scalar>,
    /// Field `preferred_max_copper_weight_oz`.
    #[serde(default, deserialize_with = "crate::scalar::deserialize_optional")]
    pub preferred_max_copper_weight_oz: Option<Scalar>,
    /// Field `cost_escalation_copper_weight_oz`.
    #[serde(default, deserialize_with = "crate::scalar::deserialize_optional")]
    pub cost_escalation_copper_weight_oz: Option<Scalar>,
    /// Field `max_copper_weight_oz`.
    #[serde(default, deserialize_with = "crate::scalar::deserialize_optional")]
    pub max_copper_weight_oz: Option<Scalar>,
    /// Field `min_dielectric_thickness`.
    #[serde(default, deserialize_with = "crate::scalar::deserialize_optional")]
    pub min_dielectric_thickness: Option<Scalar>,
    /// Field `preferred_min_dielectric_thickness`.
    #[serde(default, deserialize_with = "crate::scalar::deserialize_optional")]
    pub preferred_min_dielectric_thickness: Option<Scalar>,
    /// Field `cost_escalation_min_dielectric_thickness`.
    #[serde(default, deserialize_with = "crate::scalar::deserialize_optional")]
    pub cost_escalation_min_dielectric_thickness: Option<Scalar>,
    /// Field `min_dielectric_constant`.
    #[serde(default, deserialize_with = "crate::scalar::deserialize_optional")]
    pub min_dielectric_constant: Option<Scalar>,
    /// Field `max_dielectric_constant`.
    #[serde(default, deserialize_with = "crate::scalar::deserialize_optional")]
    pub max_dielectric_constant: Option<Scalar>,
    /// Field `max_loss_tangent`.
    #[serde(default, deserialize_with = "crate::scalar::deserialize_optional")]
    pub max_loss_tangent: Option<Scalar>,
    /// Field `min_tg_c`.
    #[serde(default, deserialize_with = "crate::scalar::deserialize_optional")]
    pub min_tg_c: Option<Scalar>,
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
    #[serde(default, deserialize_with = "crate::scalar::deserialize_optional")]
    pub copper_weight_oz: Option<Scalar>,
    /// Field `dielectric_thickness`.
    #[serde(default, deserialize_with = "crate::scalar::deserialize_optional")]
    pub dielectric_thickness: Option<Scalar>,
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
    #[serde(default, deserialize_with = "crate::scalar::deserialize_optional")]
    pub min_width: Option<Scalar>,
    /// Field `min_clearance`.
    #[serde(default, deserialize_with = "crate::scalar::deserialize_optional")]
    pub min_clearance: Option<Scalar>,
    /// Field `max_layer_count`.
    pub max_layer_count: Option<usize>,
    /// Field `min_via_count`.
    pub min_via_count: Option<usize>,
    /// Field `min_current_width`.
    #[serde(default, deserialize_with = "crate::scalar::deserialize_optional")]
    pub min_current_width: Option<Scalar>,
    /// Field `min_voltage_clearance`.
    #[serde(default, deserialize_with = "crate::scalar::deserialize_optional")]
    pub min_voltage_clearance: Option<Scalar>,
    /// Field `requires_reference_plane`.
    pub requires_reference_plane: Option<bool>,
    /// Field `requires_impedance_control`.
    pub requires_impedance_control: Option<bool>,
    /// Field `target_impedance_ohms`.
    #[serde(default, deserialize_with = "crate::scalar::deserialize_optional")]
    pub target_impedance_ohms: Option<Scalar>,
    /// Field `impedance_tolerance_ohms`.
    #[serde(default, deserialize_with = "crate::scalar::deserialize_optional")]
    pub impedance_tolerance_ohms: Option<Scalar>,
    /// Field `differential_pair`.
    pub differential_pair: Option<String>,
    /// Field `differential_role`.
    pub differential_role: Option<DifferentialRole>,
    /// Field `min_pair_spacing`.
    #[serde(default, deserialize_with = "crate::scalar::deserialize_optional")]
    pub min_pair_spacing: Option<Scalar>,
    /// Field `max_pair_spacing`.
    #[serde(default, deserialize_with = "crate::scalar::deserialize_optional")]
    pub max_pair_spacing: Option<Scalar>,
    /// Field `max_length`.
    #[serde(default, deserialize_with = "crate::scalar::deserialize_optional")]
    pub max_length: Option<Scalar>,
    /// Field `max_pair_skew`.
    #[serde(default, deserialize_with = "crate::scalar::deserialize_optional")]
    pub max_pair_skew: Option<Scalar>,
    /// Field `max_via_count`.
    pub max_via_count: Option<usize>,
    /// Preferred copper land diameter for vias used by this class.
    #[serde(default, deserialize_with = "crate::scalar::deserialize_optional")]
    pub preferred_via_land_diameter: Option<Scalar>,
    /// Preferred finished drill diameter for vias used by this class.
    #[serde(default, deserialize_with = "crate::scalar::deserialize_optional")]
    pub preferred_via_drill_diameter: Option<Scalar>,
    /// Source-authored name of the preferred via construction.
    pub preferred_via_style: Option<String>,
}

#[derive(Clone, Debug, Deserialize, Default)]
#[serde(default)]
/// Public data model for `NetClassRegionConfig`.
pub struct NetClassRegionConfig {
    /// Field `name`.
    pub name: String,
    /// Field `min_x`.
    #[serde(default, deserialize_with = "crate::scalar::deserialize_optional")]
    pub min_x: Option<Scalar>,
    /// Field `min_y`.
    #[serde(default, deserialize_with = "crate::scalar::deserialize_optional")]
    pub min_y: Option<Scalar>,
    /// Field `max_x`.
    #[serde(default, deserialize_with = "crate::scalar::deserialize_optional")]
    pub max_x: Option<Scalar>,
    /// Field `max_y`.
    #[serde(default, deserialize_with = "crate::scalar::deserialize_optional")]
    pub max_y: Option<Scalar>,
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
