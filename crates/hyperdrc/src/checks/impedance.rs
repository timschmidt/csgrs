//! First-pass impedance estimates for stackup-aware readiness checks.
//!
//! This module intentionally implements only narrow, auditable subsets of
//! impedance review: single-ended outer-layer microstrip over the next copper
//! reference layer, centered single-ended stripline between adjacent copper
//! references, and equal-width edge-coupled differential forms over those two
//! geometries. It is a readiness screen for obvious width/stackup mismatch, not
//! a substitute for field solving, fabricator stackup tuning, or frequency-
//! dependent roughness/loss review.

use crate::Scalar;
use crate::constraint_policy::{StackupConfig, StackupLayerConfig, StackupLayerKind};
use crate::scalar::scalar;

/// Summary of a supported trace impedance estimate.
#[derive(Clone, Debug, PartialEq)]
pub(super) struct TraceImpedanceEstimate {
    /// Estimated characteristic impedance in ohms.
    pub(super) impedance_ohms: Scalar,
    /// Analytical model used for the estimate.
    pub(super) model: ImpedanceModel,
    /// Parsed conductor width in the same units as the stackup thickness.
    pub(super) trace_width: Scalar,
    /// Model dielectric height in stackup units.
    ///
    /// For outer microstrip this is the height to the adjacent reference
    /// copper. For centered stripline this is the total spacing between the two
    /// adjacent reference copper layers.
    pub(super) dielectric_height: Scalar,
    /// Relative dielectric constant used by the estimate.
    pub(super) dielectric_constant: Scalar,
}

/// Analytical model used by [`TraceImpedanceEstimate`].
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub(super) enum ImpedanceModel {
    /// Outer-layer single-ended microstrip over the next copper reference layer.
    OuterMicrostrip,
    /// Centered single-ended stripline between adjacent copper reference layers.
    CenteredStripline,
}

/// Summary of a supported equal-width differential-pair estimate.
#[derive(Clone, Debug, PartialEq)]
pub(super) struct DifferentialImpedanceEstimate {
    /// Estimated odd-mode differential impedance in ohms.
    pub(super) impedance_ohms: Scalar,
    /// Analytical model used for the estimate.
    pub(super) model: DifferentialImpedanceModel,
    /// Equal parsed width of each member conductor.
    pub(super) trace_width: Scalar,
    /// Edge-to-edge spacing between member conductors.
    pub(super) pair_gap: Scalar,
    /// Reference dielectric height used to normalize width and gap.
    pub(super) dielectric_height: Scalar,
    /// Relative dielectric constant used by the estimate.
    pub(super) dielectric_constant: Scalar,
}

/// Analytical coupled-line model used by [`DifferentialImpedanceEstimate`].
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub(super) enum DifferentialImpedanceModel {
    /// Equal-width edge-coupled microstrip over the adjacent reference plane.
    EdgeCoupledOuterMicrostrip,
    /// Equal-width edge-coupled centered stripline between reference planes.
    EdgeCoupledCenteredStripline,
}

/// Estimate single-ended impedance for supported stackup/layer combinations.
///
/// Returns `None` when the inputs do not describe a supported single-ended
/// transmission-line geometry with positive dielectric data. Outer-layer
/// estimates use the Hammerstad-Jensen quasi-static microstrip forms. Centered
/// stripline uses a zero-thickness first-pass approximation. IPC-2221B frames
/// impedance as a fabrication stackup and conductor-geometry constraint rather
/// than a geometry-only universal rule.
pub(super) fn estimate_single_ended_impedance(
    stackup: &StackupConfig,
    layer_name: &str,
    trace_width: Scalar,
) -> Option<TraceImpedanceEstimate> {
    if trace_width <= Scalar::zero() {
        return None;
    }
    let dielectric_constant = stackup.material_dielectric_constant.as_ref()?;
    if dielectric_constant <= &Scalar::zero() {
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
        let impedance_ohms = hammerstad_jensen_microstrip_ohms(
            &trace_width,
            &dielectric_height,
            dielectric_constant,
        )?;

        return Some(TraceImpedanceEstimate {
            impedance_ohms,
            model: ImpedanceModel::OuterMicrostrip,
            trace_width,
            dielectric_height,
            dielectric_constant: dielectric_constant.clone(),
        });
    }

    let upper_reference_index = copper_indexes[signal_order - 1];
    let lower_reference_index = copper_indexes[signal_order + 1];
    let upper_height =
        dielectric_height_between(&stackup.layers, upper_reference_index, signal_index)?;
    let lower_height =
        dielectric_height_between(&stackup.layers, signal_index, lower_reference_index)?;
    if !approximately_centered_between_planes(&upper_height, &lower_height) {
        return None;
    }

    let dielectric_height = &upper_height + &lower_height;
    let impedance_ohms =
        wheeler_centered_stripline_ohms(&trace_width, &dielectric_height, dielectric_constant)?;

    Some(TraceImpedanceEstimate {
        impedance_ohms,
        model: ImpedanceModel::CenteredStripline,
        trace_width,
        dielectric_height,
        dielectric_constant: dielectric_constant.clone(),
    })
}

/// Estimate equal-width edge-coupled differential impedance.
///
/// The retained width/gap/height ratios are restricted to the normalized
/// geometry range used by coupled-line compact models (`0.1..=10`) and Dk is
/// restricted to `1..=18`. The zero-thickness single-line estimate supplies
/// the uncoupled impedance. The edge-coupling corrections are the differential
/// microstrip and stripline screening equations documented by Lattice in
/// *PCB Layout Recommendations for BGA Packages* (HB1011, equations 6 and 7)
/// and by Texas Instruments in *High-Speed Layout Guidelines* (SLLA311). For
/// centered stripline, their `H` is the trace-to-plane height rather than the
/// total reference-plane spacing retained by the single-ended model.
///
/// This remains a static release-readiness estimate; conductor thickness,
/// mask, roughness, frequency dispersion, weave, and fabricator tuning are
/// intentionally not inferred.
pub(super) fn estimate_equal_width_differential_impedance(
    stackup: &StackupConfig,
    layer_name: &str,
    trace_width: Scalar,
    pair_gap: Scalar,
) -> Option<DifferentialImpedanceEstimate> {
    if pair_gap <= Scalar::zero() {
        return None;
    }
    let single = estimate_single_ended_impedance(stackup, layer_name, trace_width.clone())?;
    let (model, dielectric_height, amplitude, decay) = match single.model {
        ImpedanceModel::OuterMicrostrip => (
            DifferentialImpedanceModel::EdgeCoupledOuterMicrostrip,
            single.dielectric_height.clone(),
            scalar("0.48"),
            scalar("0.96"),
        ),
        ImpedanceModel::CenteredStripline => (
            DifferentialImpedanceModel::EdgeCoupledCenteredStripline,
            (&single.dielectric_height / scalar("2")).ok()?,
            scalar("0.347"),
            scalar("2.9"),
        ),
    };
    let width_ratio = (&trace_width / &dielectric_height).ok()?;
    let gap_ratio = (&pair_gap / &dielectric_height).ok()?;
    if width_ratio < scalar("0.1")
        || width_ratio > scalar("10")
        || gap_ratio < scalar("0.1")
        || gap_ratio > scalar("10")
        || single.dielectric_constant < Scalar::one()
        || single.dielectric_constant > scalar("18")
    {
        return None;
    }

    let coupling = Scalar::one() - amplitude * (-decay * gap_ratio).exp().ok()?;
    if coupling <= Scalar::zero() || coupling >= Scalar::one() {
        return None;
    }
    let impedance_ohms = scalar("2") * single.impedance_ohms * coupling;
    (impedance_ohms > Scalar::zero()).then_some(DifferentialImpedanceEstimate {
        impedance_ohms,
        model,
        trace_width,
        pair_gap,
        dielectric_height,
        dielectric_constant: single.dielectric_constant,
    })
}

fn dielectric_height_between(
    layers: &[StackupLayerConfig],
    signal_index: usize,
    reference_index: usize,
) -> Option<Scalar> {
    let (start, end) = if signal_index < reference_index {
        (signal_index + 1, reference_index)
    } else {
        (reference_index + 1, signal_index)
    };
    let mut height = Scalar::zero();
    for layer in &layers[start..end] {
        if !matches!(
            layer.kind,
            StackupLayerKind::Dielectric | StackupLayerKind::Core | StackupLayerKind::Prepreg
        ) {
            continue;
        }
        let thickness = layer.dielectric_thickness.as_ref()?;
        if thickness <= &Scalar::zero() {
            return None;
        }
        height += thickness;
    }

    (height > Scalar::zero()).then_some(height)
}

fn hammerstad_jensen_microstrip_ohms(
    trace_width: &Scalar,
    dielectric_height: &Scalar,
    dielectric_constant: &Scalar,
) -> Option<Scalar> {
    if trace_width <= &Scalar::zero()
        || dielectric_height <= &Scalar::zero()
        || dielectric_constant <= &Scalar::zero()
    {
        return None;
    }

    let width_to_height = (trace_width / dielectric_height).ok()?;
    if width_to_height <= Scalar::zero() {
        return None;
    }

    let correction = if width_to_height < Scalar::one() {
        let difference = Scalar::one() - &width_to_height;
        scalar("0.04") * (&difference * &difference)
    } else {
        Scalar::zero()
    };
    let mean_dielectric = ((dielectric_constant + Scalar::one()) / scalar("2")).ok()?;
    let contrast = ((dielectric_constant - Scalar::one()) / scalar("2")).ok()?;
    let reciprocal_term = (scalar("12") / &width_to_height).ok()?;
    let root_factor = (Scalar::one() + reciprocal_term).pow(scalar("-0.5")).ok()?;
    let effective_dielectric_constant = mean_dielectric + contrast * root_factor + correction;
    let dielectric_root = effective_dielectric_constant.sqrt().ok()?;

    let impedance = if width_to_height <= Scalar::one() {
        let scale = (scalar("60") / dielectric_root).ok()?;
        let reciprocal_width = (scalar("8") / &width_to_height).ok()?;
        let logarithm = (reciprocal_width + scalar("0.25") * &width_to_height)
            .ln()
            .ok()?;
        scale * logarithm
    } else {
        let logarithm = (&width_to_height + scalar("1.444")).ln().ok()?;
        let shape = &width_to_height + scalar("1.393") + scalar("0.667") * logarithm;
        ((scalar("120") * Scalar::pi()) / (dielectric_root * shape)).ok()?
    };

    (impedance > Scalar::zero()).then_some(impedance)
}

fn approximately_centered_between_planes(upper_height: &Scalar, lower_height: &Scalar) -> bool {
    if upper_height <= &Scalar::zero() || lower_height <= &Scalar::zero() {
        return false;
    }
    let average = ((upper_height + lower_height) / scalar("2")).expect("nonzero exact denominator");
    (upper_height - lower_height).abs() <= average * scalar("0.10")
}

fn wheeler_centered_stripline_ohms(
    trace_width: &Scalar,
    plane_spacing: &Scalar,
    dielectric_constant: &Scalar,
) -> Option<Scalar> {
    if trace_width <= &Scalar::zero()
        || plane_spacing <= &Scalar::zero()
        || dielectric_constant <= &Scalar::zero()
    {
        return None;
    }

    let width_to_spacing = (trace_width / plane_spacing).ok()?;
    if width_to_spacing <= Scalar::zero() {
        return None;
    }

    // Cohn and Wheeler give the shielded stripline foundation that later CAD
    // tools refine with thickness and roughness corrections. This expression is
    // the common zero-thickness centered-strip first-pass form used here only to
    // flag obvious net-class/stackup mismatches before fabricator field solving.
    let numerator = scalar("30") * Scalar::pi();
    let dielectric_root = dielectric_constant.clone().sqrt().ok()?;
    let denominator = dielectric_root * (width_to_spacing + scalar("0.441"));
    let impedance = (numerator / denominator).ok()?;
    (impedance > Scalar::zero()).then_some(impedance)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn layer(
        name: &str,
        kind: StackupLayerKind,
        copper_weight_oz: Option<Scalar>,
        dielectric_thickness: Option<Scalar>,
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
            material_dielectric_constant: Some(scalar("4.2")),
            layers: vec![
                layer("F.Cu", StackupLayerKind::Copper, Some(scalar("1.0")), None),
                layer(
                    "Prepreg",
                    StackupLayerKind::Prepreg,
                    None,
                    Some(scalar("0.18")),
                ),
                layer("B.Cu", StackupLayerKind::Copper, Some(scalar("1.0")), None),
            ],
            ..StackupConfig::default()
        };

        let estimate = estimate_single_ended_impedance(&stackup, "F.Cu", scalar("0.32"))
            .expect("two-layer FR-4 stackup should support outer microstrip");

        assert_eq!(estimate.model, ImpedanceModel::OuterMicrostrip);
        assert_eq!(estimate.trace_width, scalar("0.32"));
        assert_eq!(estimate.dielectric_height, scalar("0.18"));
        assert_eq!(estimate.dielectric_constant, scalar("4.2"));
        assert!(
            estimate.impedance_ohms >= scalar("48") && estimate.impedance_ohms <= scalar("58"),
            "estimated impedance {} should stay in a plausible FR-4 range",
            estimate.impedance_ohms
        );
    }

    #[test]
    fn bottom_outer_microstrip_uses_previous_copper_reference() {
        let stackup = StackupConfig {
            material_dielectric_constant: Some(scalar("4.2")),
            layers: vec![
                layer("F.Cu", StackupLayerKind::Copper, Some(scalar("1.0")), None),
                layer("Core", StackupLayerKind::Core, None, Some(scalar("0.18"))),
                layer("B.Cu", StackupLayerKind::Copper, Some(scalar("1.0")), None),
            ],
            ..StackupConfig::default()
        };

        let top = estimate_single_ended_impedance(&stackup, "F.Cu", scalar("0.32"))
            .expect("top layer should use the next copper reference");
        let bottom = estimate_single_ended_impedance(&stackup, "B.Cu", scalar("0.32"))
            .expect("bottom layer should use the previous copper reference");

        assert_eq!(top.impedance_ohms, bottom.impedance_ohms);
        assert_eq!(bottom.dielectric_height, scalar("0.18"));
        assert_eq!(bottom.model, ImpedanceModel::OuterMicrostrip);
    }

    #[test]
    fn centered_stripline_estimate_matches_expected_fr4_range() {
        let stackup = StackupConfig {
            material_dielectric_constant: Some(scalar("4.2")),
            layers: vec![
                layer("F.Cu", StackupLayerKind::Copper, Some(scalar("1.0")), None),
                layer(
                    "Prepreg",
                    StackupLayerKind::Prepreg,
                    None,
                    Some(scalar("0.18")),
                ),
                layer(
                    "In1.Cu",
                    StackupLayerKind::Copper,
                    Some(scalar("1.0")),
                    None,
                ),
                layer("Core", StackupLayerKind::Core, None, Some(scalar("0.18"))),
                layer("B.Cu", StackupLayerKind::Copper, Some(scalar("1.0")), None),
            ],
            ..StackupConfig::default()
        };

        let estimate = estimate_single_ended_impedance(&stackup, "In1.Cu", scalar("0.17"))
            .expect("centered inner layer should support stripline estimate");

        assert_eq!(estimate.model, ImpedanceModel::CenteredStripline);
        assert_eq!(estimate.trace_width, scalar("0.17"));
        assert_eq!(estimate.dielectric_height, scalar("0.36"));
        assert!(
            estimate.impedance_ohms >= scalar("48") && estimate.impedance_ohms <= scalar("54"),
            "estimated stripline impedance {} should stay in a plausible FR-4 range",
            estimate.impedance_ohms
        );
    }

    #[test]
    fn estimate_rejects_inner_or_underdefined_stackups() {
        let missing_reference = StackupConfig {
            material_dielectric_constant: Some(scalar("4.2")),
            layers: vec![
                layer("F.Cu", StackupLayerKind::Copper, Some(scalar("1.0")), None),
                layer("Core", StackupLayerKind::Core, None, Some(scalar("1.5"))),
            ],
            ..StackupConfig::default()
        };
        assert!(
            estimate_single_ended_impedance(&missing_reference, "F.Cu", scalar("0.3")).is_none()
        );

        let missing_dielectric_constant = StackupConfig {
            layers: vec![
                layer("F.Cu", StackupLayerKind::Copper, Some(scalar("1.0")), None),
                layer("Core", StackupLayerKind::Core, None, Some(scalar("0.18"))),
                layer("B.Cu", StackupLayerKind::Copper, Some(scalar("1.0")), None),
            ],
            ..StackupConfig::default()
        };
        assert!(
            estimate_single_ended_impedance(&missing_dielectric_constant, "F.Cu", scalar("0.3"))
                .is_none()
        );

        let inner_layer = StackupConfig {
            material_dielectric_constant: Some(scalar("4.2")),
            layers: vec![
                layer("F.Cu", StackupLayerKind::Copper, Some(scalar("1.0")), None),
                layer(
                    "Prepreg",
                    StackupLayerKind::Prepreg,
                    None,
                    Some(scalar("0.12")),
                ),
                layer(
                    "In1.Cu",
                    StackupLayerKind::Copper,
                    Some(scalar("1.0")),
                    None,
                ),
                layer("Core", StackupLayerKind::Core, None, Some(scalar("0.40"))),
                layer("B.Cu", StackupLayerKind::Copper, Some(scalar("1.0")), None),
            ],
            ..StackupConfig::default()
        };
        assert!(estimate_single_ended_impedance(&inner_layer, "In1.Cu", scalar("0.3")).is_none());
    }

    #[test]
    fn equal_width_outer_microstrip_pair_estimate_tracks_gap_coupling() {
        let stackup = StackupConfig {
            material_dielectric_constant: Some(scalar("4.2")),
            layers: vec![
                layer("F.Cu", StackupLayerKind::Copper, Some(scalar("1.0")), None),
                layer(
                    "Prepreg",
                    StackupLayerKind::Prepreg,
                    None,
                    Some(scalar("0.18")),
                ),
                layer("B.Cu", StackupLayerKind::Copper, Some(scalar("1.0")), None),
            ],
            ..StackupConfig::default()
        };

        let estimate = estimate_equal_width_differential_impedance(
            &stackup,
            "F.Cu",
            scalar("0.32"),
            scalar("0.18"),
        )
        .expect("ordinary equal-width FR-4 pair should be supported");
        assert_eq!(
            estimate.model,
            DifferentialImpedanceModel::EdgeCoupledOuterMicrostrip
        );
        assert_eq!(estimate.pair_gap, scalar("0.18"));
        assert!(
            estimate.impedance_ohms >= scalar("82") && estimate.impedance_ohms <= scalar("92"),
            "estimated differential impedance {} should stay in a plausible USB range",
            estimate.impedance_ohms
        );

        assert!(
            estimate_equal_width_differential_impedance(
                &stackup,
                "F.Cu",
                scalar("0.32"),
                scalar("0.001"),
            )
            .is_none(),
            "geometry outside the declared normalized model range must remain unsupported"
        );
    }

    #[test]
    fn equal_width_centered_stripline_pair_estimate_uses_trace_to_plane_height() {
        let stackup = StackupConfig {
            material_dielectric_constant: Some(scalar("4.2")),
            layers: vec![
                layer("F.Cu", StackupLayerKind::Copper, Some(scalar("1.0")), None),
                layer(
                    "Prepreg",
                    StackupLayerKind::Prepreg,
                    None,
                    Some(scalar("0.18")),
                ),
                layer(
                    "In1.Cu",
                    StackupLayerKind::Copper,
                    Some(scalar("1.0")),
                    None,
                ),
                layer("Core", StackupLayerKind::Core, None, Some(scalar("0.18"))),
                layer("B.Cu", StackupLayerKind::Copper, Some(scalar("1.0")), None),
            ],
            ..StackupConfig::default()
        };

        let estimate = estimate_equal_width_differential_impedance(
            &stackup,
            "In1.Cu",
            scalar("0.17"),
            scalar("0.18"),
        )
        .expect("centered equal-width stripline pair should be supported");
        assert_eq!(
            estimate.model,
            DifferentialImpedanceModel::EdgeCoupledCenteredStripline
        );
        assert_eq!(estimate.dielectric_height, scalar("0.18"));
        assert!(
            estimate.impedance_ohms >= scalar("96") && estimate.impedance_ohms <= scalar("105"),
            "estimated differential stripline impedance {} should stay plausible",
            estimate.impedance_ohms
        );
    }
}
