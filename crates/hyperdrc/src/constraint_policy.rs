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
    pub layers: Vec<StackupLayerConfig>,
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
    pub differential_pair: Option<String>,
    pub differential_role: Option<DifferentialRole>,
    pub min_pair_spacing: Option<f64>,
    pub max_pair_spacing: Option<f64>,
    pub max_via_count: Option<usize>,
}

#[derive(Copy, Clone, Debug, Deserialize, Eq, PartialEq)]
#[serde(rename_all = "kebab-case")]
pub enum DifferentialRole {
    Positive,
    Negative,
}
