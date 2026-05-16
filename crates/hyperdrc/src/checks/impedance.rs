//! First-pass impedance estimates for stackup-aware readiness checks.
//!
//! This module intentionally implements only narrow, auditable subsets of
//! impedance review: single-ended outer-layer microstrip over the next copper
//! reference layer and centered single-ended stripline between adjacent copper
//! references. It is a readiness screen for obvious width/stackup mismatch, not
//! a substitute for field solving, fabricator stackup tuning, or frequency-
//! dependent roughness/loss review.

use std::f64::consts::PI;

use crate::constraint_policy::{StackupConfig, StackupLayerConfig, StackupLayerKind};

const CENTERED_STRIPLINE_BALANCE_TOLERANCE: f64 = 0.10;

/// Summary of a supported trace impedance estimate.
#[derive(Copy, Clone, Debug, PartialEq)]
pub(super) struct TraceImpedanceEstimate {
    /// Estimated characteristic impedance in ohms.
    pub(super) impedance_ohms: f64,
    /// Analytical model used for the estimate.
    pub(super) model: ImpedanceModel,
    /// Parsed conductor width in the same units as the stackup thickness.
    pub(super) trace_width: f64,
    /// Model dielectric height in stackup units.
    ///
    /// For outer microstrip this is the height to the adjacent reference
    /// copper. For centered stripline this is the total spacing between the two
    /// adjacent reference copper layers.
    pub(super) dielectric_height: f64,
    /// Relative dielectric constant used by the estimate.
    pub(super) dielectric_constant: f64,
}

/// Analytical model used by [`TraceImpedanceEstimate`].
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub(super) enum ImpedanceModel {
    /// Outer-layer single-ended microstrip over the next copper reference layer.
    OuterMicrostrip,
    /// Centered single-ended stripline between adjacent copper reference layers.
    CenteredStripline,
}

/// Estimate single-ended impedance for supported stackup/layer combinations.
///
/// Returns `None` when the inputs do not describe a supported single-ended
/// transmission-line geometry with positive dielectric data. The outer-layer
/// formulas use the quasi-static microstrip closed forms popularized by
/// Hammerstad and Jensen, "Accurate Models for Microstrip Computer-Aided
/// Design," 1980 IEEE MTT-S International Microwave Symposium Digest, pp.
/// 407-409, doi:10.1109/MWSYM.1980.1124303. Centered stripline uses the
/// zero-thickness first-pass approximation associated with Cohn, "Characteristic
/// Impedance of the Shielded-Strip Transmission Line," IRE Transactions on
/// Microwave Theory and Techniques, vol. MTT-2, no. 2, 1954, pp. 52-57,
/// doi:10.1109/TMTT.1954.1124875, and Wheeler, "Transmission-Line Properties
/// of a Stripline Between Parallel Planes," IEEE Transactions on Microwave
/// Theory and Techniques, vol. 26, no. 11, 1978, pp. 866-876,
/// doi:10.1109/TMTT.1978.1129505. IPC-2221B is also relevant because it frames
/// impedance as a fabrication stackup and conductor-geometry constraint rather
/// than a geometry-only universal rule.
pub(super) fn estimate_single_ended_impedance(
    stackup: &StackupConfig,
    layer_name: &str,
    trace_width: f64,
) -> Option<TraceImpedanceEstimate> {
    if !trace_width.is_finite() || trace_width <= 0.0 {
        return None;
    }
    let dielectric_constant = stackup.material_dielectric_constant?;
    if !dielectric_constant.is_finite() || dielectric_constant <= 0.0 {
        return None;
    }

    let copper_indexes = stackup
        .layers
        .iter()
        .enumerate()
        .filter_map(|(index, layer)| (layer.kind == StackupLayerKind::Copper).then_some(index))
        .collect::<Vec<_>>();
    let signal_index = stackup.layers.iter().position(|layer| {
        layer.kind == StackupLayerKind::Copper && layer.name.trim() == layer_name.trim()
    })?;
    let signal_order = copper_indexes
        .iter()
        .position(|index| *index == signal_index)?;

    if signal_order == 0 || signal_order + 1 == copper_indexes.len() {
        let reference_index = if signal_order == 0 {
            copper_indexes.get(1).copied()
        } else {
            copper_indexes.get(signal_order.wrapping_sub(1)).copied()
        }?;

        let dielectric_height =
            dielectric_height_between(&stackup.layers, signal_index, reference_index)?;
        let impedance_ohms =
            hammerstad_jensen_microstrip_ohms(trace_width, dielectric_height, dielectric_constant)?;

        return Some(TraceImpedanceEstimate {
            impedance_ohms,
            model: ImpedanceModel::OuterMicrostrip,
            trace_width,
            dielectric_height,
            dielectric_constant,
        });
    }

    let upper_reference_index = copper_indexes[signal_order - 1];
    let lower_reference_index = copper_indexes[signal_order + 1];
    let upper_height =
        dielectric_height_between(&stackup.layers, upper_reference_index, signal_index)?;
    let lower_height =
        dielectric_height_between(&stackup.layers, signal_index, lower_reference_index)?;
    if !approximately_centered_between_planes(upper_height, lower_height) {
        return None;
    }

    let dielectric_height = upper_height + lower_height;
    let impedance_ohms =
        wheeler_centered_stripline_ohms(trace_width, dielectric_height, dielectric_constant)?;

    Some(TraceImpedanceEstimate {
        impedance_ohms,
        model: ImpedanceModel::CenteredStripline,
        trace_width,
        dielectric_height,
        dielectric_constant,
    })
}

fn dielectric_height_between(
    layers: &[StackupLayerConfig],
    signal_index: usize,
    reference_index: usize,
) -> Option<f64> {
    let (start, end) = if signal_index < reference_index {
        (signal_index + 1, reference_index)
    } else {
        (reference_index + 1, signal_index)
    };
    let mut height = 0.0;
    for layer in &layers[start..end] {
        if !matches!(
            layer.kind,
            StackupLayerKind::Dielectric | StackupLayerKind::Core | StackupLayerKind::Prepreg
        ) {
            continue;
        }
        let thickness = layer.dielectric_thickness?;
        if !thickness.is_finite() || thickness <= 0.0 {
            return None;
        }
        height += thickness;
    }

    (height.is_finite() && height > 0.0).then_some(height)
}

fn hammerstad_jensen_microstrip_ohms(
    trace_width: f64,
    dielectric_height: f64,
    dielectric_constant: f64,
) -> Option<f64> {
    if !dielectric_height.is_finite()
        || dielectric_height <= 0.0
        || !dielectric_constant.is_finite()
        || dielectric_constant <= 0.0
    {
        return None;
    }

    let width_to_height = trace_width / dielectric_height;
    if !width_to_height.is_finite() || width_to_height <= 0.0 {
        return None;
    }

    let correction = if width_to_height < 1.0 {
        0.04 * (1.0 - width_to_height).powi(2)
    } else {
        0.0
    };
    let effective_dielectric_constant = (dielectric_constant + 1.0) / 2.0
        + (dielectric_constant - 1.0) / 2.0 * (1.0 + 12.0 / width_to_height).powf(-0.5)
        + correction;

    let impedance = if width_to_height <= 1.0 {
        (60.0 / effective_dielectric_constant.sqrt())
            * ((8.0 / width_to_height) + 0.25 * width_to_height).ln()
    } else {
        (120.0 * PI)
            / (effective_dielectric_constant.sqrt()
                * (width_to_height + 1.393 + 0.667 * (width_to_height + 1.444).ln()))
    };

    (impedance.is_finite() && impedance > 0.0).then_some(impedance)
}

fn approximately_centered_between_planes(upper_height: f64, lower_height: f64) -> bool {
    if !upper_height.is_finite()
        || !lower_height.is_finite()
        || upper_height <= 0.0
        || lower_height <= 0.0
    {
        return false;
    }
    let average = (upper_height + lower_height) / 2.0;
    (upper_height - lower_height).abs() <= average * CENTERED_STRIPLINE_BALANCE_TOLERANCE
}

fn wheeler_centered_stripline_ohms(
    trace_width: f64,
    plane_spacing: f64,
    dielectric_constant: f64,
) -> Option<f64> {
    if !trace_width.is_finite()
        || trace_width <= 0.0
        || !plane_spacing.is_finite()
        || plane_spacing <= 0.0
        || !dielectric_constant.is_finite()
        || dielectric_constant <= 0.0
    {
        return None;
    }

    let width_to_spacing = trace_width / plane_spacing;
    if !width_to_spacing.is_finite() || width_to_spacing <= 0.0 {
        return None;
    }

    // Cohn and Wheeler give the shielded stripline foundation that later CAD
    // tools refine with thickness and roughness corrections. This expression is
    // the common zero-thickness centered-strip first-pass form used here only to
    // flag obvious net-class/stackup mismatches before fabricator field solving.
    let impedance = (30.0 * PI / dielectric_constant.sqrt()) / (width_to_spacing + 0.441);
    (impedance.is_finite() && impedance > 0.0).then_some(impedance)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn layer(
        name: &str,
        kind: StackupLayerKind,
        copper_weight_oz: Option<f64>,
        dielectric_thickness: Option<f64>,
    ) -> StackupLayerConfig {
        StackupLayerConfig {
            name: name.to_string(),
            kind,
            copper_weight_oz,
            dielectric_thickness,
        }
    }

    #[test]
    fn outer_microstrip_estimate_matches_expected_fr4_range() {
        let stackup = StackupConfig {
            material_dielectric_constant: Some(4.2),
            layers: vec![
                layer("F.Cu", StackupLayerKind::Copper, Some(1.0), None),
                layer("Prepreg", StackupLayerKind::Prepreg, None, Some(0.18)),
                layer("B.Cu", StackupLayerKind::Copper, Some(1.0), None),
            ],
            ..StackupConfig::default()
        };

        let estimate = estimate_single_ended_impedance(&stackup, "F.Cu", 0.32)
            .expect("two-layer FR-4 stackup should support outer microstrip");

        assert_eq!(estimate.model, ImpedanceModel::OuterMicrostrip);
        assert_eq!(estimate.trace_width, 0.32);
        assert_eq!(estimate.dielectric_height, 0.18);
        assert_eq!(estimate.dielectric_constant, 4.2);
        assert!(
            (48.0..=58.0).contains(&estimate.impedance_ohms),
            "estimated impedance {} should stay in a plausible FR-4 range",
            estimate.impedance_ohms
        );
    }

    #[test]
    fn bottom_outer_microstrip_uses_previous_copper_reference() {
        let stackup = StackupConfig {
            material_dielectric_constant: Some(4.2),
            layers: vec![
                layer("F.Cu", StackupLayerKind::Copper, Some(1.0), None),
                layer("Core", StackupLayerKind::Core, None, Some(0.18)),
                layer("B.Cu", StackupLayerKind::Copper, Some(1.0), None),
            ],
            ..StackupConfig::default()
        };

        let top = estimate_single_ended_impedance(&stackup, "F.Cu", 0.32)
            .expect("top layer should use the next copper reference");
        let bottom = estimate_single_ended_impedance(&stackup, "B.Cu", 0.32)
            .expect("bottom layer should use the previous copper reference");

        assert!((top.impedance_ohms - bottom.impedance_ohms).abs() < 1.0e-12);
        assert_eq!(bottom.dielectric_height, 0.18);
        assert_eq!(bottom.model, ImpedanceModel::OuterMicrostrip);
    }

    #[test]
    fn centered_stripline_estimate_matches_expected_fr4_range() {
        let stackup = StackupConfig {
            material_dielectric_constant: Some(4.2),
            layers: vec![
                layer("F.Cu", StackupLayerKind::Copper, Some(1.0), None),
                layer("Prepreg", StackupLayerKind::Prepreg, None, Some(0.18)),
                layer("In1.Cu", StackupLayerKind::Copper, Some(1.0), None),
                layer("Core", StackupLayerKind::Core, None, Some(0.18)),
                layer("B.Cu", StackupLayerKind::Copper, Some(1.0), None),
            ],
            ..StackupConfig::default()
        };

        let estimate = estimate_single_ended_impedance(&stackup, "In1.Cu", 0.17)
            .expect("centered inner layer should support stripline estimate");

        assert_eq!(estimate.model, ImpedanceModel::CenteredStripline);
        assert_eq!(estimate.trace_width, 0.17);
        assert_eq!(estimate.dielectric_height, 0.36);
        assert!(
            (48.0..=54.0).contains(&estimate.impedance_ohms),
            "estimated stripline impedance {} should stay in a plausible FR-4 range",
            estimate.impedance_ohms
        );
    }

    #[test]
    fn estimate_rejects_inner_or_underdefined_stackups() {
        let missing_reference = StackupConfig {
            material_dielectric_constant: Some(4.2),
            layers: vec![
                layer("F.Cu", StackupLayerKind::Copper, Some(1.0), None),
                layer("Core", StackupLayerKind::Core, None, Some(1.5)),
            ],
            ..StackupConfig::default()
        };
        assert!(estimate_single_ended_impedance(&missing_reference, "F.Cu", 0.3).is_none());

        let missing_dielectric_constant = StackupConfig {
            layers: vec![
                layer("F.Cu", StackupLayerKind::Copper, Some(1.0), None),
                layer("Core", StackupLayerKind::Core, None, Some(0.18)),
                layer("B.Cu", StackupLayerKind::Copper, Some(1.0), None),
            ],
            ..StackupConfig::default()
        };
        assert!(
            estimate_single_ended_impedance(&missing_dielectric_constant, "F.Cu", 0.3).is_none()
        );

        let inner_layer = StackupConfig {
            material_dielectric_constant: Some(4.2),
            layers: vec![
                layer("F.Cu", StackupLayerKind::Copper, Some(1.0), None),
                layer("Prepreg", StackupLayerKind::Prepreg, None, Some(0.12)),
                layer("In1.Cu", StackupLayerKind::Copper, Some(1.0), None),
                layer("Core", StackupLayerKind::Core, None, Some(0.40)),
                layer("B.Cu", StackupLayerKind::Copper, Some(1.0), None),
            ],
            ..StackupConfig::default()
        };
        assert!(estimate_single_ended_impedance(&inner_layer, "In1.Cu", 0.3).is_none());
    }
}
