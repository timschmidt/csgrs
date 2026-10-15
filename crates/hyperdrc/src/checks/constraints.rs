//! Config-driven stackup and net-class readiness checks.
//!
//! These checks are deliberately conservative. IPC-2221B treats conductor
//! sizing, spacing, materials, and stackup as design constraints that depend on
//! voltage, current, environment, and fabrication capability; hyperdrc only
//! enforces the explicit project constraints supplied in `hyperdrc` config
//! rather than trying to infer a universal rule deck.
//!
//! Reliability note: parsed trace length, pair skew, and clearance estimates in
//! this module are suspect for meanders, arcs, zones, and vendor-specific
//! stackups. Verify any release-blocking result against the layout tool's
//! constraint engine or a fabricator rule deck.

use super::minimum_bounding_dimension;
use std::collections::{BTreeMap, BTreeSet};

use super::distance::{
    PreparedRegionBoundary, finite_enclosure, minimum_metric, region_boundary_distance,
};

use super::impedance::{
    DifferentialImpedanceModel, ImpedanceModel, estimate_equal_width_differential_impedance,
    estimate_single_ended_impedance,
};
use super::net_class::resolve_net_classes;
use super::net_scope::{matching_class_indexes_for_feature, net_class_region_diagnostics};
use crate::Scalar;
use crate::constraint_policy::{
    DifferentialRole, FabricationCapabilityConfig, NetClassConfig, StackupConfig, StackupLayerKind,
    SurfaceFinish,
};
use crate::kicad::{BoardModel, CopperFeature, CopperKind};
use crate::report::{Severity, Violation};
use crate::scalar::scalar;

/// Run the `stackup_readiness` design-readiness check or report helper.
pub fn stackup_readiness(stackup: Option<&StackupConfig>, boards: &[BoardModel]) -> Vec<Violation> {
    let Some(stackup) = stackup else {
        return Vec::new();
    };

    let mut violations = Vec::new();
    let configured_copper_layers = stackup
        .layers
        .iter()
        .filter(|layer| layer.kind == StackupLayerKind::Copper)
        .collect::<Vec<_>>();
    let parsed_layers = parsed_copper_layers(boards);

    if let Some(expected) = stackup.copper_layer_count {
        let configured_count = configured_copper_layers.len();
        if configured_count > 0 && configured_count != expected {
            violations.push(Violation::new(
                "stackup-readiness",
                Severity::Warning,
                vec!["stackup:config".to_string()],
                None,
                Vec::new(),
                Vec::new(),
                Some(format!(
                    "stackup declares {expected} copper layer(s), but lists {configured_count} copper layer object(s)"
                )),
            ));
        }

        if !parsed_layers.is_empty() && parsed_layers.len() != expected {
            violations.push(Violation::new(
                "stackup-readiness",
                Severity::Warning,
                parsed_layers.iter().cloned().collect(),
                None,
                Vec::new(),
                Vec::new(),
                Some(format!(
                    "stackup declares {expected} copper layer(s), but parsed KiCad copper uses {} layer(s)",
                    parsed_layers.len()
                )),
            ));
        }
    }

    for layer in &configured_copper_layers {
        if layer.name.trim().is_empty() {
            violations.push(Violation::new(
                "stackup-readiness",
                Severity::Warning,
                vec!["stackup:config".to_string()],
                None,
                Vec::new(),
                Vec::new(),
                Some("stackup copper layer is missing a layer name".to_string()),
            ));
        }
        if layer.copper_weight_oz.is_none() {
            violations.push(Violation::new(
                "stackup-readiness",
                Severity::Warning,
                vec![format!("stackup:{}", layer.name)],
                None,
                Vec::new(),
                Vec::new(),
                Some(format!(
                    "stackup copper layer {} is missing copper_weight_oz",
                    layer.name
                )),
            ));
        }
    }

    if !parsed_layers.is_empty() {
        for layer in &configured_copper_layers {
            if !layer.name.trim().is_empty() && !parsed_layers.contains(&layer.name) {
                violations.push(Violation::new(
                    "stackup-readiness",
                    Severity::Warning,
                    vec![format!("stackup:{}", layer.name)],
                    None,
                    Vec::new(),
                    Vec::new(),
                    Some(format!(
                        "stackup copper layer {} was not found in parsed KiCad copper",
                        layer.name
                    )),
                ));
            }
        }
    }

    if stackup.finished_thickness.is_some()
        && stackup
            .layers
            .iter()
            .filter(|layer| {
                matches!(
                    layer.kind,
                    StackupLayerKind::Dielectric
                        | StackupLayerKind::Core
                        | StackupLayerKind::Prepreg
                )
            })
            .all(|layer| layer.dielectric_thickness.is_none())
    {
        violations.push(Violation::new(
            "stackup-readiness",
            Severity::Warning,
            vec!["stackup:config".to_string()],
            None,
            Vec::new(),
            Vec::new(),
            Some(
                "stackup declares finished_thickness but no dielectric/core/prepreg thickness entries"
                    .to_string(),
            ),
        ));
    }

    violations.extend(stackup_process_metadata_readiness(stackup));
    violations.extend(stackup_fabrication_capability_readiness(
        stackup,
        &configured_copper_layers,
    ));

    violations
}

fn stackup_process_metadata_readiness(stackup: &StackupConfig) -> Vec<Violation> {
    let mut violations = Vec::new();
    if is_blank(stackup.material_family.as_deref()) {
        violations.push(stackup_metadata_violation(
            "stackup material_family is missing; review laminate family before fabrication release",
        ));
    }
    if stackup.surface_finish.is_none() {
        violations.push(stackup_metadata_violation(
            "stackup surface_finish is missing; review HASL/ENIG/ENEPIG/OSP/contact finish selection before fabrication release",
        ));
    }
    if is_blank(stackup.soldermask_color.as_deref()) {
        violations.push(stackup_metadata_violation(
            "stackup soldermask_color is missing; review mask color and process assumptions before release",
        ));
    }
    if is_blank(stackup.soldermask_process.as_deref()) {
        violations.push(stackup_metadata_violation(
            "stackup soldermask_process is missing; review LPI/dry-film/process assumptions before release",
        ));
    }
    if is_blank(stackup.target_ipc_class.as_deref()) {
        violations.push(stackup_metadata_violation(
            "stackup target_ipc_class is missing; review IPC class or fabricator acceptance class before release",
        ));
    }
    if is_blank(stackup.fabricator_profile.as_deref()) {
        violations.push(stackup_metadata_violation(
            "stackup fabricator_profile is missing; review selected fabricator capability profile before release",
        ));
    }

    if matches!(
        stackup.surface_finish,
        Some(SurfaceFinish::Hasl | SurfaceFinish::LeadFreeHasl)
    ) && stackup.impedance_controlled == Some(true)
    {
        violations.push(stackup_metadata_violation(
            "stackup combines HASL-style finish with impedance_controlled=true; review finish planarity and controlled-impedance fabrication notes",
        ));
    }
    if stackup.impedance_controlled == Some(true) {
        if invalid_positive(stackup.material_dielectric_constant.as_ref()) {
            violations.push(stackup_metadata_violation(
                "stackup impedance_controlled=true but material_dielectric_constant is missing or invalid; review laminate Dk before impedance release",
            ));
        }
        if invalid_non_negative(stackup.material_loss_tangent.as_ref()) {
            violations.push(stackup_metadata_violation(
                "stackup impedance_controlled=true but material_loss_tangent is missing or invalid; review laminate Df before impedance release",
            ));
        }
    }

    violations
}

fn stackup_metadata_violation(message: &str) -> Violation {
    Violation::new(
        "stackup-readiness",
        Severity::Warning,
        vec!["stackup:config".to_string()],
        None,
        Vec::new(),
        Vec::new(),
        Some(message.to_string()),
    )
}

#[derive(Clone, Debug, Default)]
struct FabricationCapability {
    label: String,
    min_finished_thickness: Option<Scalar>,
    preferred_min_finished_thickness: Option<Scalar>,
    preferred_max_finished_thickness: Option<Scalar>,
    max_finished_thickness: Option<Scalar>,
    max_copper_layers: Option<usize>,
    preferred_max_copper_layers: Option<usize>,
    cost_escalation_copper_layers: Option<usize>,
    min_copper_weight_oz: Option<Scalar>,
    preferred_min_copper_weight_oz: Option<Scalar>,
    preferred_max_copper_weight_oz: Option<Scalar>,
    cost_escalation_copper_weight_oz: Option<Scalar>,
    max_copper_weight_oz: Option<Scalar>,
    min_dielectric_thickness: Option<Scalar>,
    preferred_min_dielectric_thickness: Option<Scalar>,
    cost_escalation_min_dielectric_thickness: Option<Scalar>,
    min_dielectric_constant: Option<Scalar>,
    max_dielectric_constant: Option<Scalar>,
    max_loss_tangent: Option<Scalar>,
    min_tg_c: Option<Scalar>,
}

fn stackup_fabrication_capability_readiness(
    stackup: &StackupConfig,
    configured_copper_layers: &[&crate::constraint_policy::StackupLayerConfig],
) -> Vec<Violation> {
    let Some(capability) = resolved_fabrication_capability(stackup) else {
        return Vec::new();
    };

    let mut violations = Vec::new();
    if let (Some(finished_thickness), Some(minimum)) = (
        stackup.finished_thickness.as_ref(),
        capability.min_finished_thickness.as_ref(),
    ) && crate::scalar::lt(finished_thickness, minimum)
    {
        violations.push(stackup_metadata_violation(&format!(
            "stackup finished_thickness {finished_thickness:#.6} is below fabricator profile {} minimum {minimum:#.6}",
            capability.label,
            finished_thickness = finished_thickness.diagnostic_decimal(-256),
            minimum = minimum.diagnostic_decimal(-256),
        )));
    }
    if let (Some(finished_thickness), Some(maximum)) = (
        stackup.finished_thickness.as_ref(),
        capability.max_finished_thickness.as_ref(),
    ) && crate::scalar::gt(finished_thickness, maximum)
    {
        violations.push(stackup_metadata_violation(&format!(
            "stackup finished_thickness {finished_thickness:#.6} is above fabricator profile {} maximum {maximum:#.6}",
            capability.label,
            finished_thickness = finished_thickness.diagnostic_decimal(-256),
            maximum = maximum.diagnostic_decimal(-256),
        )));
    }
    if let (Some(finished_thickness), Some(preferred_minimum)) = (
        stackup.finished_thickness.as_ref(),
        capability.preferred_min_finished_thickness.as_ref(),
    ) && crate::scalar::lt(finished_thickness, preferred_minimum)
        && capability
            .min_finished_thickness
            .as_ref()
            .is_none_or(|minimum| crate::scalar::ge(finished_thickness, minimum))
    {
        violations.push(stackup_metadata_violation(&format!(
            "stackup finished_thickness {finished_thickness:#.6} is below fabricator profile {} preferred minimum {preferred_minimum:#.6}; review cost-escalation or special-process requirements",
            capability.label,
            finished_thickness = finished_thickness.diagnostic_decimal(-256),
            preferred_minimum = preferred_minimum.diagnostic_decimal(-256),
        )));
    }
    if let (Some(finished_thickness), Some(preferred_maximum)) = (
        stackup.finished_thickness.as_ref(),
        capability.preferred_max_finished_thickness.as_ref(),
    ) && crate::scalar::gt(finished_thickness, preferred_maximum)
        && capability
            .max_finished_thickness
            .as_ref()
            .is_none_or(|maximum| crate::scalar::le(finished_thickness, maximum))
    {
        violations.push(stackup_metadata_violation(&format!(
            "stackup finished_thickness {finished_thickness:#.6} is above fabricator profile {} preferred maximum {preferred_maximum:#.6}; review cost-escalation or special-process requirements",
            capability.label,
            finished_thickness = finished_thickness.diagnostic_decimal(-256),
            preferred_maximum = preferred_maximum.diagnostic_decimal(-256),
        )));
    }
    if let Some(max_copper_layers) = capability.max_copper_layers {
        let configured_count = configured_copper_layers.len();
        if configured_count > max_copper_layers {
            violations.push(stackup_metadata_violation(&format!(
                "fabricator profile {} supports up to {max_copper_layers} copper layer(s), but stackup lists {configured_count}",
                capability.label
            )));
        }
    }
    let configured_count = configured_copper_layers.len();
    if let Some(preferred_max_copper_layers) = capability.preferred_max_copper_layers
        && configured_count > preferred_max_copper_layers
        && capability
            .max_copper_layers
            .is_none_or(|maximum| configured_count <= maximum)
    {
        violations.push(stackup_metadata_violation(&format!(
                "fabricator profile {} preferred service supports up to {preferred_max_copper_layers} copper layer(s), but stackup lists {configured_count}; review cost-escalation or advanced-service selection",
                capability.label
            )));
    }
    if let Some(cost_escalation_copper_layers) = capability.cost_escalation_copper_layers
        && configured_count > cost_escalation_copper_layers
        && capability
            .max_copper_layers
            .is_none_or(|maximum| configured_count <= maximum)
    {
        violations.push(stackup_metadata_violation(&format!(
                "fabricator profile {} cost-escalation threshold is {cost_escalation_copper_layers} copper layer(s), but stackup lists {configured_count}; review quote class and fabrication lead time",
                capability.label
            )));
    }

    for layer in configured_copper_layers {
        if let (Some(weight), Some(minimum)) = (
            layer.copper_weight_oz.as_ref(),
            capability.min_copper_weight_oz.as_ref(),
        ) && crate::scalar::lt(weight, minimum)
        {
            violations.push(stackup_metadata_violation(&format!(
                "stackup copper layer {} has copper_weight_oz {weight:#.6} below fabricator profile {} minimum {minimum:#.6}",
                layer.name,
                capability.label,
                weight = weight.diagnostic_decimal(-256),
                minimum = minimum.diagnostic_decimal(-256),
            )));
        }
        if let (Some(weight), Some(maximum)) = (
            layer.copper_weight_oz.as_ref(),
            capability.max_copper_weight_oz.as_ref(),
        ) && crate::scalar::gt(weight, maximum)
        {
            violations.push(stackup_metadata_violation(&format!(
                "stackup copper layer {} has copper_weight_oz {weight:#.6} above fabricator profile {} maximum {maximum:#.6}",
                layer.name,
                capability.label,
                weight = weight.diagnostic_decimal(-256),
                maximum = maximum.diagnostic_decimal(-256),
            )));
        }
        if let (Some(weight), Some(preferred_minimum)) = (
            layer.copper_weight_oz.as_ref(),
            capability.preferred_min_copper_weight_oz.as_ref(),
        ) && crate::scalar::lt(weight, preferred_minimum)
            && capability
                .min_copper_weight_oz
                .as_ref()
                .is_none_or(|minimum| crate::scalar::ge(weight, minimum))
        {
            violations.push(stackup_metadata_violation(&format!(
                "stackup copper layer {} has copper_weight_oz {weight:#.6} below fabricator profile {} preferred minimum {preferred_minimum:#.6}; review cost-escalation or special-process requirements",
                layer.name,
                capability.label,
                weight = weight.diagnostic_decimal(-256),
                preferred_minimum = preferred_minimum.diagnostic_decimal(-256),
            )));
        }
        if let (Some(weight), Some(preferred_maximum)) = (
            layer.copper_weight_oz.as_ref(),
            capability.preferred_max_copper_weight_oz.as_ref(),
        ) && crate::scalar::gt(weight, preferred_maximum)
            && capability
                .max_copper_weight_oz
                .as_ref()
                .is_none_or(|maximum| crate::scalar::le(weight, maximum))
        {
            violations.push(stackup_metadata_violation(&format!(
                "stackup copper layer {} has copper_weight_oz {weight:#.6} above fabricator profile {} preferred maximum {preferred_maximum:#.6}; review cost-escalation or special-process requirements",
                layer.name,
                capability.label,
                weight = weight.diagnostic_decimal(-256),
                preferred_maximum = preferred_maximum.diagnostic_decimal(-256),
            )));
        }
        if let (Some(weight), Some(cost_threshold)) = (
            layer.copper_weight_oz.as_ref(),
            capability.cost_escalation_copper_weight_oz.as_ref(),
        ) && crate::scalar::gt(weight, cost_threshold)
            && capability
                .max_copper_weight_oz
                .as_ref()
                .is_none_or(|maximum| crate::scalar::le(weight, maximum))
        {
            violations.push(stackup_metadata_violation(&format!(
                "stackup copper layer {} has copper_weight_oz {weight:#.6} above fabricator profile {} cost-escalation threshold {cost_threshold:#.6}; review quote class and fabrication lead time",
                layer.name,
                capability.label,
                weight = weight.diagnostic_decimal(-256),
                cost_threshold = cost_threshold.diagnostic_decimal(-256),
            )));
        }
    }

    if let Some(minimum) = capability.min_dielectric_thickness.as_ref() {
        for layer in stackup.layers.iter().filter(|layer| {
            matches!(
                layer.kind,
                StackupLayerKind::Dielectric | StackupLayerKind::Core | StackupLayerKind::Prepreg
            )
        }) {
            if let Some(thickness) = layer.dielectric_thickness.as_ref()
                && crate::scalar::lt(thickness, minimum)
            {
                violations.push(stackup_metadata_violation(&format!(
                    "stackup dielectric layer {} has dielectric_thickness {thickness:#.6} below fabricator profile {} minimum {minimum:#.6}",
                    layer.name,
                    capability.label,
                    thickness = thickness.diagnostic_decimal(-256),
                    minimum = minimum.diagnostic_decimal(-256),
                )));
            }
        }
    }
    if let Some(preferred_minimum) = capability.preferred_min_dielectric_thickness.as_ref() {
        for layer in stackup.layers.iter().filter(|layer| {
            matches!(
                layer.kind,
                StackupLayerKind::Dielectric | StackupLayerKind::Core | StackupLayerKind::Prepreg
            )
        }) {
            if let Some(thickness) = layer.dielectric_thickness.as_ref()
                && crate::scalar::lt(thickness, preferred_minimum)
                && capability
                    .min_dielectric_thickness
                    .as_ref()
                    .is_none_or(|minimum| crate::scalar::ge(thickness, minimum))
            {
                violations.push(stackup_metadata_violation(&format!(
                    "stackup dielectric layer {} has dielectric_thickness {thickness:#.6} below fabricator profile {} preferred minimum {preferred_minimum:#.6}; review cost-escalation or special-process requirements",
                    layer.name,
                    capability.label,
                    thickness = thickness.diagnostic_decimal(-256),
                    preferred_minimum = preferred_minimum.diagnostic_decimal(-256),
                )));
            }
        }
    }
    if let Some(cost_threshold) = capability.cost_escalation_min_dielectric_thickness.as_ref() {
        for layer in stackup.layers.iter().filter(|layer| {
            matches!(
                layer.kind,
                StackupLayerKind::Dielectric | StackupLayerKind::Core | StackupLayerKind::Prepreg
            )
        }) {
            if let Some(thickness) = layer.dielectric_thickness.as_ref()
                && crate::scalar::lt(thickness, cost_threshold)
                && capability
                    .min_dielectric_thickness
                    .as_ref()
                    .is_none_or(|minimum| crate::scalar::ge(thickness, minimum))
            {
                violations.push(stackup_metadata_violation(&format!(
                    "stackup dielectric layer {} has dielectric_thickness {thickness:#.6} below fabricator profile {} cost-escalation threshold {cost_threshold:#.6}; review quote class and fabrication lead time",
                    layer.name,
                    capability.label,
                    thickness = thickness.diagnostic_decimal(-256),
                    cost_threshold = cost_threshold.diagnostic_decimal(-256),
                )));
            }
        }
    }

    // IPC-2221B treats dielectric constant and loss tangent as stackup inputs
    // for electrical behavior; these checks only verify explicit policy ranges
    // before handoff, leaving field solving to dedicated impedance tools.
    if let (Some(value), Some(minimum)) = (
        stackup.material_dielectric_constant.as_ref(),
        capability.min_dielectric_constant.as_ref(),
    ) && crate::scalar::lt(value, minimum)
    {
        violations.push(stackup_metadata_violation(&format!(
            "stackup material_dielectric_constant {value:#.6} is below fabricator profile {} minimum {minimum:#.6}",
            capability.label,
            value = value.diagnostic_decimal(-256),
            minimum = minimum.diagnostic_decimal(-256),
        )));
    }
    if let (Some(value), Some(maximum)) = (
        stackup.material_dielectric_constant.as_ref(),
        capability.max_dielectric_constant.as_ref(),
    ) && crate::scalar::gt(value, maximum)
    {
        violations.push(stackup_metadata_violation(&format!(
            "stackup material_dielectric_constant {value:#.6} is above fabricator profile {} maximum {maximum:#.6}",
            capability.label,
            value = value.diagnostic_decimal(-256),
            maximum = maximum.diagnostic_decimal(-256),
        )));
    }
    if let (Some(value), Some(maximum)) = (
        stackup.material_loss_tangent.as_ref(),
        capability.max_loss_tangent.as_ref(),
    ) && crate::scalar::gt(value, maximum)
    {
        violations.push(stackup_metadata_violation(&format!(
            "stackup material_loss_tangent {value:#.6} is above fabricator profile {} maximum {maximum:#.6}",
            capability.label,
            value = value.diagnostic_decimal(-256),
            maximum = maximum.diagnostic_decimal(-256),
        )));
    }
    if let (Some(value), Some(minimum)) =
        (stackup.material_tg_c.as_ref(), capability.min_tg_c.as_ref())
        && crate::scalar::lt(value, minimum)
    {
        violations.push(stackup_metadata_violation(&format!(
            "stackup material_tg_c {value:#.6} is below fabricator profile {} minimum {minimum:#.6}",
            capability.label,
            value = value.diagnostic_decimal(-256),
            minimum = minimum.diagnostic_decimal(-256),
        )));
    }

    violations
}

fn resolved_fabrication_capability(stackup: &StackupConfig) -> Option<FabricationCapability> {
    // IPC-2221B and IPC-6012D frame thickness, conductor build-up, dielectric
    // construction, and acceptance class as coupled design/fabrication
    // constraints. These profiles are early review thresholds, not a substitute
    // for the fabricator's current controlled-process limits.
    let mut capability = stackup
        .fabricator_profile
        .as_deref()
        .and_then(builtin_fabrication_capability);

    if capability.is_none() && has_custom_capability(&stackup.fabrication_capability) {
        capability = Some(FabricationCapability {
            label: stackup
                .fabricator_profile
                .clone()
                .unwrap_or_else(|| "custom".into()),
            ..FabricationCapability::default()
        });
    }

    capability.map(|mut capability| {
        let custom = &stackup.fabrication_capability;
        capability.min_finished_thickness = custom
            .min_finished_thickness
            .clone()
            .or(capability.min_finished_thickness);
        capability.preferred_min_finished_thickness = custom
            .preferred_min_finished_thickness
            .clone()
            .or(capability.preferred_min_finished_thickness);
        capability.preferred_max_finished_thickness = custom
            .preferred_max_finished_thickness
            .clone()
            .or(capability.preferred_max_finished_thickness);
        capability.max_finished_thickness = custom
            .max_finished_thickness
            .clone()
            .or(capability.max_finished_thickness);
        capability.max_copper_layers = custom.max_copper_layers.or(capability.max_copper_layers);
        capability.preferred_max_copper_layers = custom
            .preferred_max_copper_layers
            .or(capability.preferred_max_copper_layers);
        capability.cost_escalation_copper_layers = custom
            .cost_escalation_copper_layers
            .or(capability.cost_escalation_copper_layers);
        capability.min_copper_weight_oz = custom
            .min_copper_weight_oz
            .clone()
            .or(capability.min_copper_weight_oz);
        capability.preferred_min_copper_weight_oz = custom
            .preferred_min_copper_weight_oz
            .clone()
            .or(capability.preferred_min_copper_weight_oz);
        capability.preferred_max_copper_weight_oz = custom
            .preferred_max_copper_weight_oz
            .clone()
            .or(capability.preferred_max_copper_weight_oz);
        capability.cost_escalation_copper_weight_oz = custom
            .cost_escalation_copper_weight_oz
            .clone()
            .or(capability.cost_escalation_copper_weight_oz);
        capability.max_copper_weight_oz = custom
            .max_copper_weight_oz
            .clone()
            .or(capability.max_copper_weight_oz);
        capability.min_dielectric_thickness = custom
            .min_dielectric_thickness
            .clone()
            .or(capability.min_dielectric_thickness);
        capability.preferred_min_dielectric_thickness = custom
            .preferred_min_dielectric_thickness
            .clone()
            .or(capability.preferred_min_dielectric_thickness);
        capability.cost_escalation_min_dielectric_thickness = custom
            .cost_escalation_min_dielectric_thickness
            .clone()
            .or(capability.cost_escalation_min_dielectric_thickness);
        capability.min_dielectric_constant = custom
            .min_dielectric_constant
            .clone()
            .or(capability.min_dielectric_constant);
        capability.max_dielectric_constant = custom
            .max_dielectric_constant
            .clone()
            .or(capability.max_dielectric_constant);
        capability.max_loss_tangent = custom
            .max_loss_tangent
            .clone()
            .or(capability.max_loss_tangent);
        capability.min_tg_c = custom.min_tg_c.clone().or(capability.min_tg_c);
        capability
    })
}

fn builtin_fabrication_capability(profile: &str) -> Option<FabricationCapability> {
    macro_rules! exact {
        ($value:literal) => {
            scalar(stringify!($value))
        };
    }
    macro_rules! optional_exact {
        () => {
            None
        };
        ($value:literal) => {
            Some(exact!($value))
        };
    }
    macro_rules! capability {
        (
            $label:literal,
            thickness: [$min_thickness:literal, $preferred_min_thickness:literal, $preferred_max_thickness:literal, $max_thickness:literal],
            layers: [$preferred_layers:literal, $cost_layers:literal, $max_layers:literal],
            copper: [$min_copper:literal, $preferred_min_copper:literal, $preferred_max_copper:literal, $cost_copper:literal, $max_copper:literal],
            dielectric: [$min_dielectric:literal, $preferred_min_dielectric:literal, $cost_dielectric:literal]
            $(, material: [$min_dk:literal, $max_dk:literal, $max_df:literal, $min_tg:literal])?
        ) => {
            FabricationCapability {
                label: $label.into(),
                min_finished_thickness: Some(exact!($min_thickness)),
                preferred_min_finished_thickness: Some(exact!($preferred_min_thickness)),
                preferred_max_finished_thickness: Some(exact!($preferred_max_thickness)),
                max_finished_thickness: Some(exact!($max_thickness)),
                preferred_max_copper_layers: Some($preferred_layers),
                cost_escalation_copper_layers: Some($cost_layers),
                max_copper_layers: Some($max_layers),
                min_copper_weight_oz: Some(exact!($min_copper)),
                preferred_min_copper_weight_oz: Some(exact!($preferred_min_copper)),
                preferred_max_copper_weight_oz: Some(exact!($preferred_max_copper)),
                cost_escalation_copper_weight_oz: Some(exact!($cost_copper)),
                max_copper_weight_oz: Some(exact!($max_copper)),
                min_dielectric_thickness: Some(exact!($min_dielectric)),
                preferred_min_dielectric_thickness: Some(exact!($preferred_min_dielectric)),
                cost_escalation_min_dielectric_thickness: Some(exact!($cost_dielectric)),
                min_dielectric_constant: optional_exact!($($min_dk)?),
                max_dielectric_constant: optional_exact!($($max_dk)?),
                max_loss_tangent: optional_exact!($($max_df)?),
                min_tg_c: optional_exact!($($min_tg)?),
            }
        };
    }

    let normalized = profile.trim().to_ascii_lowercase();
    match normalized.as_str() {
        "prototype-fab" => Some(
            capability!("prototype-fab", thickness: [0.6, 0.8, 1.6, 2.4], layers: [2, 4, 4], copper: [0.5, 1.0, 1.0, 2.0, 2.0], dielectric: [0.05, 0.10, 0.075]),
        ),
        "standard-fab" => Some(
            capability!("standard-fab", thickness: [0.4, 0.8, 2.0, 3.2], layers: [4, 6, 8], copper: [0.33, 0.5, 2.0, 2.0, 3.0], dielectric: [0.04, 0.075, 0.05]),
        ),
        "jlcpcb-economy" | "jlcpcb-basic" => Some(
            capability!("jlcpcb-economy", thickness: [0.6, 0.8, 1.6, 2.0], layers: [2, 4, 4], copper: [1.0, 1.0, 1.0, 1.0, 2.0], dielectric: [0.075, 0.10, 0.075]),
        ),
        "jlcpcb-standard" => Some(
            capability!("jlcpcb-standard", thickness: [0.4, 0.8, 2.0, 3.2], layers: [4, 6, 6], copper: [0.5, 1.0, 2.0, 2.0, 3.0], dielectric: [0.04, 0.075, 0.05]),
        ),
        "jlcpcb-advanced" => Some(
            capability!("jlcpcb-advanced", thickness: [0.2, 0.6, 2.4, 4.0], layers: [8, 10, 12], copper: [0.25, 0.5, 3.0, 3.0, 4.0], dielectric: [0.025, 0.05, 0.035]),
        ),
        "pcbway-standard" => Some(
            capability!("pcbway-standard", thickness: [0.4, 0.8, 2.4, 3.2], layers: [6, 10, 14], copper: [0.5, 1.0, 2.0, 3.0, 8.0], dielectric: [0.04, 0.075, 0.05]),
        ),
        "pcbway-advanced" => Some(
            capability!("pcbway-advanced", thickness: [0.2, 0.6, 3.2, 4.5], layers: [16, 20, 30], copper: [0.25, 0.5, 3.0, 4.0, 8.0], dielectric: [0.025, 0.05, 0.035]),
        ),
        "eurocircuits-pcb-proto" | "eurocircuits-standard" => Some(
            capability!("eurocircuits-pcb-proto", thickness: [0.8, 1.0, 1.6, 2.4], layers: [2, 4, 4], copper: [0.5, 1.0, 1.0, 2.0, 2.0], dielectric: [0.075, 0.10, 0.075]),
        ),
        "eurocircuits-standard-pool" => Some(
            capability!("eurocircuits-standard-pool", thickness: [0.5, 0.8, 2.0, 3.2], layers: [4, 6, 8], copper: [0.5, 1.0, 1.0, 2.0, 2.0], dielectric: [0.05, 0.075, 0.05]),
        ),
        "eurocircuits-defined-impedance" => Some(
            capability!("eurocircuits-defined-impedance", thickness: [0.5, 0.8, 2.4, 3.2], layers: [6, 8, 12], copper: [0.5, 1.0, 2.0, 2.0, 3.0], dielectric: [0.04, 0.075, 0.05], material: [3.0, 4.8, 0.025, 130.0]),
        ),
        "advanced-fab" => Some(
            capability!("advanced-fab", thickness: [0.2, 0.6, 3.2, 4.0], layers: [8, 10, 12], copper: [0.25, 0.5, 3.0, 3.0, 4.0], dielectric: [0.025, 0.05, 0.035]),
        ),
        _ => None,
    }
}

fn has_custom_capability(capability: &FabricationCapabilityConfig) -> bool {
    capability.min_finished_thickness.is_some()
        || capability.preferred_min_finished_thickness.is_some()
        || capability.preferred_max_finished_thickness.is_some()
        || capability.max_finished_thickness.is_some()
        || capability.max_copper_layers.is_some()
        || capability.preferred_max_copper_layers.is_some()
        || capability.cost_escalation_copper_layers.is_some()
        || capability.min_copper_weight_oz.is_some()
        || capability.preferred_min_copper_weight_oz.is_some()
        || capability.preferred_max_copper_weight_oz.is_some()
        || capability.cost_escalation_copper_weight_oz.is_some()
        || capability.max_copper_weight_oz.is_some()
        || capability.min_dielectric_thickness.is_some()
        || capability.preferred_min_dielectric_thickness.is_some()
        || capability
            .cost_escalation_min_dielectric_thickness
            .is_some()
        || capability.min_dielectric_constant.is_some()
        || capability.max_dielectric_constant.is_some()
        || capability.max_loss_tangent.is_some()
        || capability.min_tg_c.is_some()
}

/// Stable check identity for target-impedance metadata, applicability, and mismatch findings.
pub const NET_IMPEDANCE_TARGET_READINESS_CHECK: &str = "net-impedance-target-readiness";

/// Run the `net_constraint_readiness` design-readiness check or report helper.
pub fn net_constraint_readiness(
    net_classes: &[NetClassConfig],
    stackup: Option<&StackupConfig>,
    boards: &[BoardModel],
    selected_layers: &[String],
) -> Vec<Violation> {
    if net_classes.is_empty() {
        return Vec::new();
    }

    let resolved_net_classes = resolve_net_classes(net_classes);
    let mut violations = resolved_net_classes.violations;
    let net_classes = resolved_net_classes.classes.as_slice();
    violations.extend(net_class_region_diagnostics(net_classes));
    for board in boards {
        let features = board
            .copper
            .iter()
            .filter(|feature| layer_selected(&feature.layer, selected_layers))
            .collect::<Vec<_>>();
        violations.extend(net_width_constraints(net_classes, &features));
        violations.extend(net_layer_and_via_constraints(net_classes, &features));
        violations.extend(net_via_geometry_constraints(net_classes, board, &features));
        violations.extend(net_clearance_constraints(net_classes, &features));
        violations.extend(net_reference_plane_constraints(net_classes, &features));
        violations.extend(net_impedance_constraints(net_classes, stackup, &features));
        violations.extend(net_impedance_target_constraints(
            net_classes,
            stackup,
            &features,
        ));
        violations.extend(net_differential_pair_constraints(net_classes, &features));
        violations.extend(net_length_constraints(net_classes, &features));
    }
    violations
}

fn net_via_geometry_constraints(
    net_classes: &[NetClassConfig],
    board: &BoardModel,
    features: &[&CopperFeature],
) -> Vec<Violation> {
    crate::checks::run_geometry_check("net-via-policy-readiness", &[&board.source], || {
        let mut violations = Vec::new();
        for drill in board.drills.iter().filter(|drill| drill.plated) {
            let Some(net) = drill.net.as_deref() else {
                continue;
            };
            let Some(via) = features.iter().copied().find(|feature| {
                feature.kind == CopperKind::Via
                    && feature.net.as_deref() == Some(net)
                    && crate::scalar::eq(&feature.location[0], &drill.location[0])
                    && crate::scalar::eq(&feature.location[1], &drill.location[1])
            }) else {
                continue;
            };
            let Some(Some(land_diameter)) = crate::checks::geometry_result_for_check(
                minimum_bounding_dimension(&via.region),
                "net-via-policy-readiness",
                &[&via.layer],
                &mut violations,
            ) else {
                continue;
            };
            for class_index in matching_class_indexes_for_feature(net_classes, via)? {
                let class = &net_classes[class_index];
                let style = class
                    .preferred_via_style
                    .as_deref()
                    .map(|style| format!(" via style {style}"))
                    .unwrap_or_default();
                if let Some(preferred) = class.preferred_via_land_diameter.as_ref()
                    && crate::scalar::ne(&land_diameter, preferred)
                {
                    violations.push(Violation::new(
                        "net-via-policy-readiness",
                        Severity::Warning,
                        vec![via.layer.clone()],
                        None,
                        Vec::new(),
                        vec![via.try_location_f64()?],
                        Some(format!(
                            "net {net} in class {} uses via land diameter {land_diameter:#.6}, not preferred{style} land diameter {preferred:#.6}",
                            class_name(class),
                            land_diameter = land_diameter.diagnostic_decimal(-256),
                            preferred = preferred.diagnostic_decimal(-256),
                        )),
                    ));
                }
                if let Some(preferred) = class.preferred_via_drill_diameter.as_ref()
                    && crate::scalar::ne(&drill.diameter, preferred)
                {
                    violations.push(Violation::new(
                        "net-via-policy-readiness",
                        Severity::Warning,
                        vec![via.layer.clone()],
                        None,
                        Vec::new(),
                        vec![drill.try_location_f64()?],
                        Some(format!(
                            "net {net} in class {} uses finished via drill diameter {:#.6}, not preferred{style} drill diameter {preferred:#.6}",
                            class_name(class),
                            (drill.diameter).diagnostic_decimal(-256),
                            preferred = preferred.diagnostic_decimal(-256),
                        )),
                    ));
                }
            }
        }
        Ok(violations)
    })
}

fn net_width_constraints(
    net_classes: &[NetClassConfig],
    features: &[&CopperFeature],
) -> Vec<Violation> {
    crate::checks::run_geometry_check("net-constraint-readiness", &[], || {
        let mut violations = Vec::new();
        for feature in features {
            let Some(net) = feature.net.as_deref() else {
                continue;
            };
            for class_index in matching_class_indexes_for_feature(net_classes, feature)? {
                let class = &net_classes[class_index];
                let Some(min_width) = class.min_width.as_ref() else {
                    if let Some(min_current_width) = class.min_current_width.as_ref() {
                        let Some(Some(width)) = crate::checks::geometry_result_for_check(
                            minimum_bounding_dimension(&feature.region),
                            "net-constraint-readiness",
                            &[&feature.layer],
                            &mut violations,
                        ) else {
                            continue;
                        };
                        if crate::scalar::gt(&width, &Scalar::zero())
                            && crate::scalar::lt(&width, min_current_width)
                        {
                            violations.push(Violation::new(
                                "net-constraint-readiness",
                                Severity::Warning,
                                vec![feature.layer.clone()],
                                None,
                                Vec::new(),
                                vec![feature.try_location_f64()?],
                                Some(format!(
                                    "net {net} in class {} has parsed {:?} width {width:#.6}, below configured current-carrying minimum {min_current_width:#.6}",
                                    class_name(class),
                                    feature.kind,
                                    width = width.diagnostic_decimal(-256),
                                    min_current_width = min_current_width.diagnostic_decimal(-256),
                                )),
                            ));
                        }
                    }
                    continue;
                };
                let Some(Some(width)) = crate::checks::geometry_result_for_check(
                    minimum_bounding_dimension(&feature.region),
                    "net-constraint-readiness",
                    &[&feature.layer],
                    &mut violations,
                ) else {
                    continue;
                };
                if crate::scalar::gt(&width, &Scalar::zero())
                    && crate::scalar::lt(&width, min_width)
                {
                    violations.push(Violation::new(
                        "net-constraint-readiness",
                        Severity::Error,
                        vec![feature.layer.clone()],
                        None,
                        Vec::new(),
                        vec![feature.try_location_f64()?],
                        Some(format!(
                            "net {net} in class {} has parsed {:?} width {width:#.6}, below configured minimum {min_width:#.6}",
                            class_name(class),
                            feature.kind,
                            width = width.diagnostic_decimal(-256),
                            min_width = min_width.diagnostic_decimal(-256),
                        )),
                    ));
                }
                if let Some(min_current_width) = class.min_current_width.as_ref()
                    && crate::scalar::gt(&width, &Scalar::zero())
                    && crate::scalar::lt(&width, min_current_width)
                {
                    violations.push(Violation::new(
                        "net-constraint-readiness",
                        Severity::Warning,
                        vec![feature.layer.clone()],
                        None,
                        Vec::new(),
                        vec![feature.try_location_f64()?],
                        Some(format!(
                            "net {net} in class {} has parsed {:?} width {width:#.6}, below configured current-carrying minimum {min_current_width:#.6}",
                            class_name(class),
                            feature.kind,
                            width = width.diagnostic_decimal(-256),
                            min_current_width = min_current_width.diagnostic_decimal(-256),
                        )),
                    ));
                }
            }
        }
        Ok(violations)
    })
}

fn net_layer_and_via_constraints(
    net_classes: &[NetClassConfig],
    features: &[&CopperFeature],
) -> Vec<Violation> {
    crate::checks::run_geometry_check("net-constraint-readiness", &[], || {
        let mut by_class_net = BTreeMap::<(usize, String), NetUse<'_>>::new();
        for feature in features {
            let Some(net) = feature.net.as_deref() else {
                continue;
            };
            for class_index in matching_class_indexes_for_feature(net_classes, feature)? {
                let entry = by_class_net
                    .entry((class_index, net.to_string()))
                    .or_default();
                entry.layers.insert(feature.layer.clone());
                entry.features.push(*feature);
                if feature.kind == CopperKind::Via {
                    entry.via_count += 1;
                }
            }
        }

        let mut violations = Vec::new();
        for ((class_index, net), usage) in by_class_net {
            let class = &net_classes[class_index];
            if let Some(max_layer_count) = class.max_layer_count
                && usage.layers.len() > max_layer_count
            {
                violations.push(Violation::new(
                    "net-constraint-readiness",
                    Severity::Warning,
                    usage.layers.iter().cloned().collect(),
                    None,
                    Vec::new(),
                    usage.try_locations()?,
                    Some(format!(
                        "net {net} in class {} appears on {} layer(s), above configured maximum {max_layer_count}",
                        class_name(class),
                        usage.layers.len()
                    )),
                ));
            }

            if let Some(min_via_count) = class.min_via_count
                && usage.layers.len() > 1
                && usage.via_count < min_via_count
            {
                violations.push(Violation::new(
                    "net-constraint-readiness",
                    Severity::Warning,
                    usage.layers.iter().cloned().collect(),
                    None,
                    Vec::new(),
                    usage.try_locations()?,
                    Some(format!(
                        "net {net} in class {} changes layers with {} parsed via(s), below configured minimum {min_via_count}",
                        class_name(class),
                        usage.via_count
                    )),
                ));
            }

            if let Some(max_via_count) = class.max_via_count
                && usage.via_count > max_via_count
            {
                violations.push(Violation::new(
                    "net-constraint-readiness",
                    Severity::Warning,
                    usage.layers.iter().cloned().collect(),
                    None,
                    Vec::new(),
                    usage.try_locations()?,
                    Some(format!(
                        "net {net} in class {} has {} parsed via(s), above configured maximum {max_via_count}",
                        class_name(class),
                        usage.via_count
                    )),
                ));
            }
        }
        Ok(violations)
    })
}

fn net_clearance_constraints(
    net_classes: &[NetClassConfig],
    features: &[&CopperFeature],
) -> Vec<Violation> {
    crate::checks::run_geometry_check("net-constraint-readiness", &[], || {
        let Some(maximum_clearance) = maximum_configured_clearance(net_classes) else {
            return Ok(Vec::new());
        };
        let mut bounded_features = features
            .iter()
            .copied()
            .map(|feature| {
                let bounds = native_region_bounds_scalar(&feature.region);
                let key = bounds
                    .as_ref()
                    .and_then(|bounds| finite_enclosure(&bounds.min_x))
                    .map(|b| b[0]);
                (
                    feature,
                    bounds,
                    key,
                    std::cell::OnceCell::<PreparedRegionBoundary>::new(),
                )
            })
            .collect::<Vec<_>>();
        bounded_features.sort_by(|left, right| {
            left.0
                .layer
                .cmp(&right.0.layer)
                .then_with(|| match (left.2, right.2) {
                    (None, None) => std::cmp::Ordering::Equal,
                    (None, Some(_)) => std::cmp::Ordering::Less,
                    (Some(_), None) => std::cmp::Ordering::Greater,
                    (Some(left), Some(right)) => left.total_cmp(&right),
                })
        });
        let mut exact_pair_count = 0_usize;
        let mut violations = Vec::new();
        for (left_index, (left, left_bounds, _, left_geometry)) in
            bounded_features.iter().enumerate()
        {
            let Some(left_net) = &left.net else {
                continue;
            };
            let maximum_x = left_bounds
                .as_ref()
                .and_then(|b| finite_enclosure(&(&b.max_x + &maximum_clearance)))
                .map(|b| b[1]);
            for (right, right_bounds, right_key, right_geometry) in
                bounded_features.iter().skip(left_index + 1)
            {
                if left.layer != right.layer {
                    break;
                }
                if maximum_x
                    .zip(*right_key)
                    .is_some_and(|(maximum, minimum)| minimum > maximum)
                {
                    break;
                }
                if left_bounds
                    .as_ref()
                    .zip(right_bounds.as_ref())
                    .is_some_and(|(left, right)| {
                        !exact_bounds_may_be_within(left, right, &maximum_clearance)
                    })
                {
                    continue;
                }
                let Some(right_net) = &right.net else {
                    continue;
                };
                if left_net == right_net {
                    continue;
                }

                let Some((class_name, min_clearance)) =
                    required_clearance(net_classes, left, right)?
                else {
                    continue;
                };
                exact_pair_count += 1;
                let measured = (|| {
                    let left = prepared_boundary(left_geometry, &left.region)?;
                    let right = prepared_boundary(right_geometry, &right.region)?;
                    left.distance_to(right)
                })();
                let Some(gap) = crate::checks::geometry_result_for_check(
                    measured,
                    "net-constraint-readiness",
                    &[&left.layer],
                    &mut violations,
                )
                .flatten() else {
                    continue;
                };
                if crate::checks::metric_cmp_for_check(
                    &gap,
                    &min_clearance,
                    "net-constraint-readiness",
                    &[&left.layer],
                    &mut violations,
                )
                .is_some_and(|order| order.is_lt())
                {
                    violations.push(Violation::new(
                        "net-constraint-readiness",
                        Severity::Error,
                        vec![left.layer.clone()],
                        None,
                        Vec::new(),
                        vec![
                            left.try_location_f64()?,
                            right.try_location_f64()?,
                        ],
                        Some(format!(
                            "net {left_net} to {right_net} spacing {gap:#.6} is below configured clearance {min_clearance:#.6} from class {class_name}",
                            gap = gap.diagnostic_decimal(-256),
                            min_clearance = min_clearance.diagnostic_decimal(-256),
                        )),
                    ));
                }
            }
        }
        log::trace!(
            "net-constraint clearance readiness: features={} maximum_clearance={maximum_clearance:#.6} exact_pairs={} violations={}",
            features.len(),
            exact_pair_count,
            violations.len(),
            maximum_clearance = maximum_clearance.diagnostic_decimal(-256),
        );
        Ok(violations)
    })
}

fn prepared_boundary<'a>(
    cache: &'a std::cell::OnceCell<PreparedRegionBoundary>,
    region: &crate::PcbRegion,
) -> Result<&'a PreparedRegionBoundary, crate::PcbGeometryUncertainty> {
    if cache.get().is_none() {
        let prepared = PreparedRegionBoundary::new(region)?;
        let _ = cache.set(prepared);
    }
    Ok(cache
        .get()
        .expect("successful preparation initialized the cell"))
}

fn net_reference_plane_constraints(
    net_classes: &[NetClassConfig],
    features: &[&CopperFeature],
) -> Vec<Violation> {
    crate::checks::run_geometry_check("net-constraint-readiness", &[], || {
        let reference_layers = features
            .iter()
            .filter(|feature| feature.kind == CopperKind::Zone)
            .filter(|feature| feature.net.as_deref().is_some_and(is_reference_net))
            .map(|feature| feature.layer.clone())
            .collect::<BTreeSet<_>>();
        if reference_layers.is_empty() {
            return Ok(net_plane_intent_violations(
                net_classes,
                features,
                "no parsed reference-plane copper was found",
            ));
        }

        let mut by_class_net = BTreeMap::<(usize, String), NetUse<'_>>::new();
        for feature in features {
            let Some(net) = feature.net.as_deref() else {
                continue;
            };
            if is_reference_net(net) {
                continue;
            }
            for class_index in matching_class_indexes_for_feature(net_classes, feature)? {
                let entry = by_class_net
                    .entry((class_index, net.to_string()))
                    .or_default();
                entry.layers.insert(feature.layer.clone());
                entry.features.push(*feature);
            }
        }

        let mut violations = Vec::new();
        for ((class_index, net), usage) in by_class_net {
            let class = &net_classes[class_index];
            if class.requires_reference_plane != Some(true) {
                continue;
            }
            // This is intentionally a presence check, not an impedance solver.
            // IPC-2221B frames conductor spacing and stackup as project-specific
            // constraints, so hyperdrc only verifies that an explicit class asking
            // for reference-plane review has some parsed ground/reference copper.
            let has_reference_layer = usage.layers.iter().any(|layer| {
                reference_layers
                    .iter()
                    .any(|reference_layer| reference_layer == layer)
            });
            if !has_reference_layer {
                violations.push(Violation::new(
                    "net-constraint-readiness",
                    Severity::Warning,
                    usage.layers.iter().cloned().collect(),
                    None,
                    Vec::new(),
                    usage.try_locations()?,
                    Some(format!(
                        "net {net} in class {} requires reference-plane review, but no parsed reference-plane zone is present on the same selected layer(s)",
                        class_name(class)
                    )),
                ));
            }
        }
        Ok(violations)
    })
}

fn net_plane_intent_violations(
    net_classes: &[NetClassConfig],
    features: &[&CopperFeature],
    reason: &str,
) -> Vec<Violation> {
    crate::checks::run_geometry_check("net-constraint-readiness", &[], || {
        let mut by_class_net = BTreeMap::<(usize, String), NetUse<'_>>::new();
        for feature in features {
            let Some(net) = feature.net.as_deref() else {
                continue;
            };
            for class_index in matching_class_indexes_for_feature(net_classes, feature)? {
                let entry = by_class_net
                    .entry((class_index, net.to_string()))
                    .or_default();
                entry.layers.insert(feature.layer.clone());
                entry.features.push(*feature);
            }
        }

        let mut violations = Vec::new();
        for ((class_index, net), usage) in by_class_net {
            let class = &net_classes[class_index];
            if class.requires_reference_plane == Some(true) {
                violations.push(Violation::new(
                    "net-constraint-readiness",
                    Severity::Warning,
                    usage.layers.iter().cloned().collect(),
                    None,
                    Vec::new(),
                    usage.try_locations()?,
                    Some(format!(
                        "net {net} in class {} requires reference-plane review, but {reason}",
                        class_name(class)
                    )),
                ));
            }
        }
        Ok(violations)
    })
}

fn net_impedance_constraints(
    net_classes: &[NetClassConfig],
    stackup: Option<&StackupConfig>,
    features: &[&CopperFeature],
) -> Vec<Violation> {
    let Some(stackup) = stackup else {
        return impedance_intent_violations(
            net_classes,
            features,
            "no stackup section was provided for impedance-control review",
        );
    };
    if stackup.impedance_controlled == Some(true) {
        return Vec::new();
    }

    let has_dielectric_thickness = stackup.layers.iter().any(|layer| {
        matches!(
            layer.kind,
            StackupLayerKind::Dielectric | StackupLayerKind::Core | StackupLayerKind::Prepreg
        ) && layer.dielectric_thickness.is_some()
    });
    let has_copper_weights = stackup
        .layers
        .iter()
        .filter(|layer| layer.kind == StackupLayerKind::Copper)
        .all(|layer| layer.copper_weight_oz.is_some());
    if has_dielectric_thickness && has_copper_weights {
        return Vec::new();
    }

    impedance_intent_violations(
        net_classes,
        features,
        "stackup lacks impedance_controlled=true or complete copper/dielectric metadata",
    )
}

fn impedance_intent_violations(
    net_classes: &[NetClassConfig],
    features: &[&CopperFeature],
    reason: &str,
) -> Vec<Violation> {
    crate::checks::run_geometry_check("net-constraint-readiness", &[], || {
        let mut by_class_net = BTreeMap::<(usize, String), NetUse<'_>>::new();
        for feature in features {
            let Some(net) = feature.net.as_deref() else {
                continue;
            };
            for class_index in matching_class_indexes_for_feature(net_classes, feature)? {
                let entry = by_class_net
                    .entry((class_index, net.to_string()))
                    .or_default();
                entry.layers.insert(feature.layer.clone());
                entry.features.push(*feature);
            }
        }

        let mut violations = Vec::new();
        for ((class_index, net), usage) in by_class_net {
            let class = &net_classes[class_index];
            if class.requires_impedance_control == Some(true) {
                violations.push(Violation::new(
                    "net-constraint-readiness",
                    Severity::Warning,
                    usage.layers.iter().cloned().collect(),
                    None,
                    Vec::new(),
                    usage.try_locations()?,
                    Some(format!(
                        "net {net} in class {} requires impedance-control review, but {reason}",
                        class_name(class)
                    )),
                ));
            }
        }
        Ok(violations)
    })
}

fn net_impedance_target_constraints(
    net_classes: &[NetClassConfig],
    stackup: Option<&StackupConfig>,
    features: &[&CopperFeature],
) -> Vec<Violation> {
    crate::checks::run_geometry_check(NET_IMPEDANCE_TARGET_READINESS_CHECK, &[], || {
        let mut by_class_net = BTreeMap::<(usize, String), NetUse<'_>>::new();
        let mut features_by_class_net = BTreeMap::<(usize, String), Vec<&CopperFeature>>::new();
        for feature in features {
            let Some(net) = feature.net.as_deref() else {
                continue;
            };
            for class_index in matching_class_indexes_for_feature(net_classes, feature)? {
                let key = (class_index, net.to_string());
                let entry = by_class_net.entry(key.clone()).or_default();
                entry.layers.insert(feature.layer.clone());
                entry.features.push(*feature);
                features_by_class_net.entry(key).or_default().push(*feature);
            }
        }

        let mut violations = Vec::new();
        let mut differential_target_classes = 0_usize;
        let mut single_ended_candidates = 0_usize;
        let mut estimated_segments = 0_usize;
        let mut unsupported_segments = 0_usize;
        for ((class_index, net), usage) in by_class_net {
            let class = &net_classes[class_index];
            if class.requires_impedance_control != Some(true) {
                continue;
            }

            let target_impedance_ohms = match class.target_impedance_ohms.as_ref() {
                Some(target) if crate::scalar::gt(target, &Scalar::zero()) => Some(target.clone()),
                Some(target) => {
                    violations.push(Violation::new(
                        NET_IMPEDANCE_TARGET_READINESS_CHECK,
                        Severity::Warning,
                        usage.layers.iter().cloned().collect(),
                        None,
                        Vec::new(),
                        usage.try_locations()?,
                        Some(format!(
                            "net {net} in class {} has invalid target_impedance_ohms {target:#.6}",
                            class_name(class),
                            target = target.diagnostic_decimal(-256),
                        )),
                    ));
                    None
                }
                None => {
                    violations.push(Violation::new(
                        NET_IMPEDANCE_TARGET_READINESS_CHECK,
                        Severity::Warning,
                        usage.layers.iter().cloned().collect(),
                        None,
                        Vec::new(),
                        usage.try_locations()?,
                        Some(format!(
                            "net {net} in class {} requires impedance-control review, but target_impedance_ohms is missing",
                            class_name(class)
                        )),
                    ));
                    None
                }
            };

            let impedance_tolerance_ohms = match class.impedance_tolerance_ohms.as_ref() {
                Some(tolerance) if crate::scalar::gt(tolerance, &Scalar::zero()) => {
                    Some(tolerance.clone())
                }
                Some(tolerance) => {
                    violations.push(Violation::new(
                        NET_IMPEDANCE_TARGET_READINESS_CHECK,
                        Severity::Warning,
                        usage.layers.iter().cloned().collect(),
                        None,
                        Vec::new(),
                        usage.try_locations()?,
                        Some(format!(
                            "net {net} in class {} has invalid impedance_tolerance_ohms {tolerance:#.6}",
                            class_name(class),
                            tolerance = tolerance.diagnostic_decimal(-256),
                        )),
                    ));
                    None
                }
                None => {
                    violations.push(Violation::new(
                        NET_IMPEDANCE_TARGET_READINESS_CHECK,
                        Severity::Warning,
                        usage.layers.iter().cloned().collect(),
                        None,
                        Vec::new(),
                        usage.try_locations()?,
                        Some(format!(
                            "net {net} in class {} requires impedance-control review, but impedance_tolerance_ohms is missing",
                            class_name(class)
                        )),
                    ));
                    None
                }
            };

            let (Some(target_impedance_ohms), Some(impedance_tolerance_ohms)) =
                (target_impedance_ohms, impedance_tolerance_ohms)
            else {
                continue;
            };
            if class.differential_pair.is_some() {
                differential_target_classes += 1;
                continue;
            }

            let Some(stackup) =
                stackup.filter(|stackup| stackup.impedance_controlled == Some(true))
            else {
                continue;
            };
            let key = (class_index, net.clone());
            let Some(net_features) = features_by_class_net.get(&key) else {
                continue;
            };
            for feature in net_features {
                if feature.kind != CopperKind::Segment {
                    continue;
                }
                single_ended_candidates += 1;
                let Some(Some(trace_width)) = crate::checks::geometry_result_for_check(
                    minimum_bounding_dimension(&feature.region),
                    NET_IMPEDANCE_TARGET_READINESS_CHECK,
                    &[&feature.layer],
                    &mut violations,
                ) else {
                    continue;
                };
                let Some(estimate) =
                    estimate_single_ended_impedance(stackup, &feature.layer, trace_width.clone())
                else {
                    unsupported_segments += 1;
                    violations.push(Violation::new(
                        NET_IMPEDANCE_TARGET_READINESS_CHECK,
                        Severity::Warning,
                        vec![feature.layer.clone()],
                        None,
                        Vec::new(),
                        vec![feature.try_location_f64()?],
                        Some(format!(
                            "net {net} in class {} has parsed width {trace_width:#.6}, but its layer/stackup geometry is outside the supported outer-microstrip and centered-stripline analytical models",
                            class_name(class),
                            trace_width = trace_width.diagnostic_decimal(-256),
                        )),
                    ));
                    continue;
                };
                estimated_segments += 1;
                let delta = (&estimate.impedance_ohms - &target_impedance_ohms).abs();
                if crate::scalar::gt(&delta, &impedance_tolerance_ohms) {
                    violations.push(Violation::new(
                        NET_IMPEDANCE_TARGET_READINESS_CHECK,
                        Severity::Warning,
                        vec![feature.layer.clone()],
                        None,
                        Vec::new(),
                        vec![feature.try_location_f64()?],
                        Some(format!(
                            "net {net} in class {} has estimated {} impedance {:#.3} ohm from parsed width {:#.6}, dielectric height/spacing {:#.6}, and Dk {:#.3}, outside target {:#.3} +/- {:#.3} ohm",
                            class_name(class),
                            impedance_model_label(estimate.model),
                            (estimate.impedance_ohms).diagnostic_decimal(-256),
                            (estimate.trace_width).diagnostic_decimal(-256),
                            (estimate.dielectric_height).diagnostic_decimal(-256),
                            (estimate.dielectric_constant).diagnostic_decimal(-256),
                            (target_impedance_ohms).diagnostic_decimal(-256),
                            (impedance_tolerance_ohms).diagnostic_decimal(-256),
                        )),
                    ));
                }
            }
        }
        let differential =
            differential_impedance_target_violations(net_classes, stackup, features)?;
        estimated_segments += differential.estimated_segments;
        unsupported_segments += differential.unsupported_segments;
        violations.extend(differential.violations);

        log::trace!(
            "net-constraint impedance target readiness: features={} single_ended_candidates={} estimated_segments={} unsupported_segments={} differential_target_classes={} violations={}",
            features.len(),
            single_ended_candidates,
            estimated_segments,
            unsupported_segments,
            differential_target_classes,
            violations.len()
        );
        Ok(violations)
    })
}

#[derive(Default)]
struct DifferentialImpedanceTargetReport {
    violations: Vec<Violation>,
    estimated_segments: usize,
    unsupported_segments: usize,
}

#[derive(Default)]
struct DifferentialImpedancePairUse<'a> {
    positive: DifferentialImpedanceSideUse<'a>,
    negative: DifferentialImpedanceSideUse<'a>,
}

impl<'a> DifferentialImpedancePairUse<'a> {
    fn side_mut(&mut self, role: DifferentialRole) -> &mut DifferentialImpedanceSideUse<'a> {
        match role {
            DifferentialRole::Positive => &mut self.positive,
            DifferentialRole::Negative => &mut self.negative,
        }
    }

    fn features(&self) -> impl Iterator<Item = &&'a CopperFeature> {
        self.positive
            .features
            .iter()
            .chain(self.negative.features.iter())
    }

    fn layers(&self) -> Vec<String> {
        self.features()
            .map(|feature| feature.layer.clone())
            .collect::<BTreeSet<_>>()
            .into_iter()
            .collect()
    }

    fn locations(&self) -> Result<Vec<[f64; 2]>, crate::PcbGeometryUncertainty> {
        self.features()
            .map(|feature| feature.try_location_f64())
            .collect()
    }
}

#[derive(Default)]
struct DifferentialImpedanceSideUse<'a> {
    features: Vec<&'a CopperFeature>,
    class_indexes: BTreeSet<usize>,
}

fn differential_impedance_target_violations(
    net_classes: &[NetClassConfig],
    stackup: Option<&StackupConfig>,
    features: &[&CopperFeature],
) -> Result<DifferentialImpedanceTargetReport, crate::PcbGeometryUncertainty> {
    let mut pairs = BTreeMap::<String, DifferentialImpedancePairUse<'_>>::new();
    for feature in features {
        if feature.kind != CopperKind::Segment {
            continue;
        }
        if feature.net.is_none() {
            continue;
        }
        for class_index in matching_class_indexes_for_feature(net_classes, feature)? {
            let class = &net_classes[class_index];
            let (Some(pair), Some(role)) = (&class.differential_pair, class.differential_role)
            else {
                continue;
            };
            if class.requires_impedance_control != Some(true) {
                continue;
            }
            let side = pairs.entry(pair.clone()).or_default().side_mut(role);
            side.features.push(*feature);
            side.class_indexes.insert(class_index);
        }
    }

    let mut report = DifferentialImpedanceTargetReport::default();
    for (pair, pair_use) in pairs {
        if pair_use.positive.features.is_empty() || pair_use.negative.features.is_empty() {
            report.unsupported_segments += 1;
            report.violations.push(Violation::new(
                NET_IMPEDANCE_TARGET_READINESS_CHECK,
                Severity::Warning,
                pair_use.layers(),
                None,
                Vec::new(),
                pair_use.locations()?,
                Some(format!(
                    "differential pair {pair} has target impedance intent but both routed member sides are required for coupled-line estimation"
                )),
            ));
            continue;
        }

        let Some((target_impedance_ohms, impedance_tolerance_ohms)) =
            consistent_differential_impedance_spec(net_classes, &pair_use)
        else {
            report.unsupported_segments += 1;
            report.violations.push(Violation::new(
                NET_IMPEDANCE_TARGET_READINESS_CHECK,
                Severity::Warning,
                pair_use.layers(),
                None,
                Vec::new(),
                pair_use.locations()?,
                Some(format!(
                    "differential pair {pair} does not have one consistent positive target_impedance_ohms and impedance_tolerance_ohms across both member classes"
                )),
            ));
            continue;
        };

        let Some(stackup) = stackup.filter(|stackup| stackup.impedance_controlled == Some(true))
        else {
            report.unsupported_segments += 1;
            report.violations.push(Violation::new(
                NET_IMPEDANCE_TARGET_READINESS_CHECK,
                Severity::Warning,
                pair_use.layers(),
                None,
                Vec::new(),
                pair_use.locations()?,
                Some(format!(
                    "differential pair {pair} has target impedance intent but no impedance-controlled stackup was supplied"
                )),
            ));
            continue;
        };

        for positive in &pair_use.positive.features {
            let Some(Some(positive_width)) = crate::checks::geometry_result_for_check(
                minimum_bounding_dimension(&positive.region),
                NET_IMPEDANCE_TARGET_READINESS_CHECK,
                &[&positive.layer],
                &mut report.violations,
            ) else {
                continue;
            };
            let closest = crate::checks::geometry_result_for_check(
                minimum_metric(
                    pair_use
                        .negative
                        .features
                        .iter()
                        .copied()
                        .filter(|negative| negative.layer == positive.layer)
                        .map(|negative| {
                            region_boundary_distance(&positive.region, &negative.region)
                                .map(|gap| gap.map(|gap| (negative, gap)))
                        }),
                ),
                NET_IMPEDANCE_TARGET_READINESS_CHECK,
                &[&positive.layer],
                &mut report.violations,
            );
            let Some(closest) = closest else { continue };
            let Some((negative, gap)) = closest else {
                report.unsupported_segments += 1;
                report.violations.push(Violation::new(
                    NET_IMPEDANCE_TARGET_READINESS_CHECK,
                    Severity::Warning,
                    vec![positive.layer.clone()],
                    None,
                    Vec::new(),
                    vec![positive.try_location_f64()?],
                    Some(format!(
                        "differential pair {pair} has no opposite-side segment on layer {} for coupled-line estimation",
                        positive.layer
                    )),
                ));
                continue;
            };
            let Some(Some(negative_width)) = crate::checks::geometry_result_for_check(
                minimum_bounding_dimension(&negative.region),
                NET_IMPEDANCE_TARGET_READINESS_CHECK,
                &[&negative.layer],
                &mut report.violations,
            ) else {
                continue;
            };
            let width_delta = (&positive_width - &negative_width).abs();
            let maximum_width = if crate::scalar::ge(&positive_width, &negative_width) {
                positive_width.clone()
            } else {
                negative_width.clone()
            };
            let parsed_width_tolerance = maximum_width * scalar("0.000001") + scalar("0.000000001");
            if crate::scalar::gt(&width_delta, &parsed_width_tolerance) {
                report.unsupported_segments += 1;
                report.violations.push(Violation::new(
                    NET_IMPEDANCE_TARGET_READINESS_CHECK,
                    Severity::Warning,
                    vec![positive.layer.clone()],
                    None,
                    Vec::new(),
                    vec![
                        positive.try_location_f64()?,
                        negative.try_location_f64()?,
                    ],
                    Some(format!(
                        "differential pair {pair} has unequal parsed member widths {positive_width:#.6} and {negative_width:#.6}; the supported coupled-line screen requires equal widths",
                        positive_width = positive_width.diagnostic_decimal(-256),
                        negative_width = negative_width.diagnostic_decimal(-256),
                    )),
                ));
                continue;
            }
            let trace_width = crate::scalar::half(&(&positive_width + &negative_width));
            let Some(estimate) = estimate_equal_width_differential_impedance(
                stackup,
                &positive.layer,
                trace_width,
                gap.clone(),
            ) else {
                report.unsupported_segments += 1;
                report.violations.push(Violation::new(
                    NET_IMPEDANCE_TARGET_READINESS_CHECK,
                    Severity::Warning,
                    vec![positive.layer.clone()],
                    None,
                    Vec::new(),
                    vec![
                        positive.try_location_f64()?,
                        negative.try_location_f64()?,
                    ],
                    Some(format!(
                        "differential pair {pair} has parsed width {positive_width:#.6} and edge gap {gap:#.6}, but its equal-width layer/stackup geometry is outside the supported edge-coupled outer-microstrip and centered-stripline analytical models",
                        positive_width = positive_width.diagnostic_decimal(-256),
                        gap = gap.diagnostic_decimal(-256),
                    )),
                ));
                continue;
            };
            report.estimated_segments += 1;
            let delta = (&estimate.impedance_ohms - &target_impedance_ohms).abs();
            if crate::scalar::gt(&delta, &impedance_tolerance_ohms) {
                report.violations.push(Violation::new(
                    NET_IMPEDANCE_TARGET_READINESS_CHECK,
                    Severity::Warning,
                    vec![positive.layer.clone()],
                    None,
                    Vec::new(),
                    vec![
                        positive.try_location_f64()?,
                        negative.try_location_f64()?,
                    ],
                    Some(format!(
                        "differential pair {pair} has estimated {} impedance {:#.3} ohm from equal parsed width {:#.6}, edge gap {:#.6}, dielectric height/spacing {:#.6}, and Dk {:#.3}, outside target {:#.3} +/- {:#.3} ohm",
                        differential_impedance_model_label(estimate.model),
                        (estimate.impedance_ohms).diagnostic_decimal(-256),
                        (estimate.trace_width).diagnostic_decimal(-256),
                        (estimate.pair_gap).diagnostic_decimal(-256),
                        (estimate.dielectric_height).diagnostic_decimal(-256),
                        (estimate.dielectric_constant).diagnostic_decimal(-256),
                        (target_impedance_ohms).diagnostic_decimal(-256),
                        (impedance_tolerance_ohms).diagnostic_decimal(-256),
                    )),
                ));
            }
        }
    }
    Ok(report)
}

fn consistent_differential_impedance_spec(
    net_classes: &[NetClassConfig],
    pair_use: &DifferentialImpedancePairUse<'_>,
) -> Option<(Scalar, Scalar)> {
    let indexes = pair_use
        .positive
        .class_indexes
        .iter()
        .chain(pair_use.negative.class_indexes.iter());
    let mut specifications = indexes.map(|index| &net_classes[*index]).map(|class| {
        (
            class.target_impedance_ohms.as_ref(),
            class.impedance_tolerance_ohms.as_ref(),
        )
    });
    let (Some(target), Some(tolerance)) = specifications.next()? else {
        return None;
    };
    if crate::scalar::le(target, &Scalar::zero()) || crate::scalar::le(tolerance, &Scalar::zero()) {
        return None;
    }
    if specifications.any(|(candidate_target, candidate_tolerance)| {
        !optional_scalar_eq(candidate_target, Some(target))
            || !optional_scalar_eq(candidate_tolerance, Some(tolerance))
    }) {
        return None;
    }
    Some((target.clone(), tolerance.clone()))
}

fn impedance_model_label(model: ImpedanceModel) -> &'static str {
    match model {
        ImpedanceModel::OuterMicrostrip => "outer microstrip",
        ImpedanceModel::CenteredStripline => "centered stripline",
    }
}

fn differential_impedance_model_label(model: DifferentialImpedanceModel) -> &'static str {
    match model {
        DifferentialImpedanceModel::EdgeCoupledOuterMicrostrip => "edge-coupled outer microstrip",
        DifferentialImpedanceModel::EdgeCoupledCenteredStripline => {
            "edge-coupled centered stripline"
        }
    }
}

fn net_differential_pair_constraints(
    net_classes: &[NetClassConfig],
    features: &[&CopperFeature],
) -> Vec<Violation> {
    crate::checks::run_geometry_check("net-constraint-readiness", &[], || {
        let mut pairs = BTreeMap::<String, DifferentialPairUse>::new();
        for feature in features {
            let Some(net) = &feature.net else {
                continue;
            };
            for class_index in matching_class_indexes_for_feature(net_classes, feature)? {
                let class = &net_classes[class_index];
                let (Some(pair), Some(role)) = (&class.differential_pair, class.differential_role)
                else {
                    continue;
                };
                let side = pairs.entry(pair.clone()).or_default().side_mut(role);
                side.net_names.insert(net.clone());
                side.layers.insert(feature.layer.clone());
                side.features.push(*feature);
                side.min_pair_spacing =
                    option_max(side.min_pair_spacing.take(), class.min_pair_spacing.clone());
                side.max_pair_spacing =
                    option_min(side.max_pair_spacing.take(), class.max_pair_spacing.clone());
                side.max_pair_skew =
                    option_min(side.max_pair_skew.take(), class.max_pair_skew.clone());
            }
        }

        let mut violations = Vec::new();
        for (pair, pair_use) in pairs {
            if pair_use.positive.features.is_empty() || pair_use.negative.features.is_empty() {
                let missing = if pair_use.positive.features.is_empty() {
                    "positive"
                } else {
                    "negative"
                };
                let present = if pair_use.positive.features.is_empty() {
                    &pair_use.negative
                } else {
                    &pair_use.positive
                };
                violations.push(Violation::new(
                    "net-constraint-readiness",
                    Severity::Warning,
                    present.layers.iter().cloned().collect(),
                    None,
                    Vec::new(),
                    present.try_locations()?,
                    Some(format!(
                        "differential pair {pair} is missing configured {missing} side copper"
                    )),
                ));
                continue;
            }

            if pair_use.positive.layers != pair_use.negative.layers {
                violations.push(Violation::new(
                    "net-constraint-readiness",
                    Severity::Warning,
                    pair_use
                        .positive
                        .layers
                        .union(&pair_use.negative.layers)
                        .cloned()
                        .collect(),
                    None,
                    Vec::new(),
                    pair_use.locations()?,
                    Some(format!(
                        "differential pair {pair} has configured sides on different selected copper layer sets"
                    )),
                ));
            }

            let min_spacing = pair_use.min_pair_spacing();
            let max_spacing = pair_use.max_pair_spacing();
            if min_spacing.is_none() && max_spacing.is_none() {
                continue;
            }

            violations.extend(differential_pair_spacing_violations(
                &pair,
                &pair_use,
                min_spacing,
                max_spacing,
            ));
        }
        Ok(violations)
    })
}

fn differential_pair_spacing_violations(
    pair: &str,
    pair_use: &DifferentialPairUse<'_>,
    min_spacing: Option<Scalar>,
    max_spacing: Option<Scalar>,
) -> Vec<Violation> {
    crate::checks::run_geometry_check("net-constraint-readiness", &[], || {
        let query_spacing = [min_spacing.as_ref(), max_spacing.as_ref()]
            .into_iter()
            .flatten()
            .filter(|spacing| crate::scalar::ge(spacing, &Scalar::zero()))
            .max_by(|left, right| {
                crate::scalar::compare(left, right)
                    .expect("exact configured spacings must be comparable")
            })
            .cloned()
            .unwrap_or_else(Scalar::zero);
        let negative_features = pair_use
            .negative
            .features
            .iter()
            .copied()
            .filter(|feature| feature.kind != CopperKind::Via)
            .collect::<Vec<_>>();
        let positive_features = pair_use
            .positive
            .features
            .iter()
            .copied()
            .filter(|feature| feature.kind != CopperKind::Via)
            .collect::<Vec<_>>();
        if positive_features.is_empty() || negative_features.is_empty() {
            return Ok(Vec::new());
        }

        let mut negative_by_layer =
            BTreeMap::<&str, Vec<(&CopperFeature, Option<ScalarBounds2>)>>::new();
        for feature in &negative_features {
            negative_by_layer
                .entry(feature.layer.as_str())
                .or_default()
                .push((feature, native_region_bounds_scalar(&feature.region)));
        }
        let mut exact_pair_count = 0_usize;
        let mut closest_pair: Option<DifferentialGap<'_>> = None;
        let mut has_pair_within_max = max_spacing.is_none();
        let mut violations = Vec::new();

        for positive in &positive_features {
            let Some(negative_candidates) = negative_by_layer.get(positive.layer.as_str()) else {
                continue;
            };
            let positive_bounds = native_region_bounds_scalar(&positive.region);
            for (negative, negative_bounds) in negative_candidates {
                if positive_bounds
                    .as_ref()
                    .zip(negative_bounds.as_ref())
                    .is_some_and(|(left, right)| {
                        !exact_bounds_may_be_within(left, right, &query_spacing)
                    })
                {
                    continue;
                }
                let Some(gap) = crate::checks::boundary_distance_for_check(
                    &positive.region,
                    &negative.region,
                    "net-constraint-readiness",
                    &[&positive.layer],
                    &mut violations,
                ) else {
                    continue;
                };
                exact_pair_count += 1;
                let observed = DifferentialGap {
                    positive,
                    negative,
                    gap: gap.clone(),
                };
                if closest_pair.as_ref().is_none_or(|closest| {
                    crate::checks::metric_cmp_for_check(
                        &gap,
                        &closest.gap,
                        "net-constraint-readiness",
                        &[&positive.layer],
                        &mut violations,
                    )
                    .is_some_and(|order| order.is_lt())
                }) {
                    closest_pair = Some(observed.clone());
                }

                if let Some(min_spacing) = min_spacing.as_ref()
                    && crate::checks::metric_cmp_for_check(
                        &gap,
                        min_spacing,
                        "net-constraint-readiness",
                        &[&positive.layer],
                        &mut violations,
                    )
                    .is_some_and(|order| order.is_lt())
                {
                    violations.push(Violation::new(
                        "net-constraint-readiness",
                        Severity::Error,
                        vec![positive.layer.clone()],
                        None,
                        Vec::new(),
                        vec![
                            positive.try_location_f64()?,
                            negative.try_location_f64()?,
                        ],
                        Some(format!(
                            "differential pair {pair} side spacing {gap:#.6} is below configured minimum {min_spacing:#.6}",
                            gap = gap.diagnostic_decimal(-256),
                            min_spacing = min_spacing.diagnostic_decimal(-256),
                        )),
                    ));
                }
                if let Some(max_spacing) = max_spacing.as_ref()
                    && crate::checks::metric_cmp_for_check(
                        &gap,
                        max_spacing,
                        "net-constraint-readiness",
                        &[&positive.layer],
                        &mut violations,
                    )
                    .is_some_and(|order| !order.is_gt())
                {
                    has_pair_within_max = true;
                }
            }
        }

        if !has_pair_within_max {
            if closest_pair.is_none() {
                closest_pair = nearest_same_layer_pair_gap(
                    &positive_features,
                    &negative_features,
                    &mut violations,
                );
            }
            if let (Some(max_spacing), Some(closest)) = (max_spacing.as_ref(), closest_pair)
                && crate::checks::metric_cmp_for_check(
                    &closest.gap,
                    max_spacing,
                    "net-constraint-readiness",
                    &[&closest.positive.layer],
                    &mut violations,
                )
                .is_some_and(|order| order.is_gt())
            {
                violations.push(Violation::new(
                    "net-constraint-readiness",
                    Severity::Warning,
                    vec![closest.positive.layer.clone()],
                    None,
                    Vec::new(),
                    vec![
                        closest.positive.try_location_f64()?,
                        closest.negative.try_location_f64()?,
                    ],
                    Some(format!(
                        "differential pair {pair} nearest side spacing {:#.6} is above configured maximum {max_spacing:#.6}",
                        (closest.gap).diagnostic_decimal(-256),
                        max_spacing = max_spacing.diagnostic_decimal(-256),
                    )),
                ));
            }
        }

        log::trace!(
            "net-constraint differential-pair spacing readiness: pair={} positives={} negatives={} exact_pairs={} violations={}",
            pair,
            positive_features.len(),
            negative_features.len(),
            exact_pair_count,
            violations.len()
        );
        Ok(violations)
    })
}

#[derive(Clone, Debug)]
struct DifferentialGap<'a> {
    positive: &'a CopperFeature,
    negative: &'a CopperFeature,
    gap: Scalar,
}

fn nearest_same_layer_pair_gap<'a>(
    positive_features: &[&'a CopperFeature],
    negative_features: &[&'a CopperFeature],
    violations: &mut Vec<Violation>,
) -> Option<DifferentialGap<'a>> {
    crate::checks::geometry_result_for_check(
        minimum_metric(
            positive_features
                .iter()
                .flat_map(|positive| {
                    negative_features
                        .iter()
                        .map(move |negative| (*positive, *negative))
                })
                .filter(|(positive, negative)| positive.layer == negative.layer)
                .map(|(positive, negative)| {
                    region_boundary_distance(&positive.region, &negative.region)
                        .map(|gap| gap.map(|gap| ((positive, negative), gap)))
                }),
        ),
        "net-constraint-readiness",
        &[],
        violations,
    )
    .flatten()
    .map(|((positive, negative), gap)| DifferentialGap {
        positive,
        negative,
        gap,
    })
}

fn net_length_constraints(
    net_classes: &[NetClassConfig],
    features: &[&CopperFeature],
) -> Vec<Violation> {
    crate::checks::run_geometry_check("net-constraint-readiness", &[], || {
        if !net_classes
            .iter()
            .any(|class| class.max_length.is_some() || class.max_pair_skew.is_some())
        {
            return Ok(Vec::new());
        }
        let mut by_class_net = BTreeMap::<(usize, String), NetUse<'_>>::new();
        let mut pairs = BTreeMap::<String, DifferentialPairUse>::new();

        let mut violations = Vec::new();
        let mut unavailable_nets = BTreeSet::new();
        let mut unavailable_pairs = BTreeSet::new();
        for feature in features {
            let Some(net) = &feature.net else {
                continue;
            };
            let class_indexes = matching_class_indexes_for_feature(net_classes, feature)?;
            if !class_indexes.iter().any(|class_index| {
                let class = &net_classes[*class_index];
                class.max_length.is_some() || class.max_pair_skew.is_some()
            }) {
                continue;
            }
            let estimate = estimated_feature_length(feature).and_then(|length| {
                length
                    .map(|length| {
                        super::distance::compare_metrics(&length, &Scalar::zero())
                            .map(|order| order.is_gt().then_some(length))
                    })
                    .transpose()
                    .map(Option::flatten)
            });
            let estimated_length = match crate::checks::geometry_result_for_check(
                estimate,
                "net-constraint-readiness",
                &[&feature.layer],
                &mut violations,
            ) {
                Some(Some(length)) => length,
                Some(None) => continue,
                None => {
                    // A partial sum cannot stand in for the net or differential pair.
                    for class_index in class_indexes {
                        unavailable_nets.insert((class_index, net.clone()));
                        if let Some(pair) = &net_classes[class_index].differential_pair {
                            unavailable_pairs.insert(pair.clone());
                        }
                    }
                    continue;
                }
            };

            for class_index in class_indexes {
                let class = &net_classes[class_index];
                let usage = by_class_net.entry((class_index, net.clone())).or_default();
                usage.layers.insert(feature.layer.clone());
                usage.features.push(*feature);
                usage.estimated_length += &estimated_length;
                usage.max_length = option_min(usage.max_length.take(), class.max_length.clone());

                let (Some(pair), Some(role)) = (&class.differential_pair, class.differential_role)
                else {
                    continue;
                };
                let side = pairs.entry(pair.clone()).or_default().side_mut(role);
                side.net_names.insert(net.clone());
                side.layers.insert(feature.layer.clone());
                side.features.push(*feature);
                side.estimated_length += &estimated_length;
                side.max_pair_skew =
                    option_min(side.max_pair_skew.take(), class.max_pair_skew.clone());
            }
        }

        for ((class_index, net), usage) in by_class_net {
            if unavailable_nets.contains(&(class_index, net.clone())) {
                continue;
            }
            let class = &net_classes[class_index];
            if let Some(max_length) = usage.max_length.as_ref()
                && crate::scalar::gt(&usage.estimated_length, max_length)
            {
                violations.push(Violation::new(
                    "net-constraint-readiness",
                    Severity::Warning,
                    usage.layers.iter().cloned().collect(),
                    None,
                    Vec::new(),
                    usage.try_locations()?,
                    Some(format!(
                        "net {net} in class {} has approximate parsed copper length {:#.6}, above configured maximum {max_length:#.6}",
                        class_name(class),
                        (usage.estimated_length).diagnostic_decimal(-256),
                        max_length = max_length.diagnostic_decimal(-256),
                    )),
                ));
            }
        }

        for (pair, pair_use) in pairs {
            if unavailable_pairs.contains(&pair) {
                continue;
            }
            let Some(max_pair_skew) = pair_use.max_pair_skew() else {
                continue;
            };
            if crate::scalar::le(&pair_use.positive.estimated_length, &Scalar::zero())
                || crate::scalar::le(&pair_use.negative.estimated_length, &Scalar::zero())
            {
                continue;
            }
            let skew =
                (&pair_use.positive.estimated_length - &pair_use.negative.estimated_length).abs();
            if crate::scalar::gt(&skew, &max_pair_skew) {
                violations.push(Violation::new(
                    "net-constraint-readiness",
                    Severity::Warning,
                    pair_use
                        .positive
                        .layers
                        .union(&pair_use.negative.layers)
                        .cloned()
                        .collect(),
                    None,
                    Vec::new(),
                    pair_use.locations()?,
                    Some(format!(
                        "differential pair {pair} has approximate parsed length skew {skew:#.6}, above configured maximum {max_pair_skew:#.6}",
                        skew = skew.diagnostic_decimal(-256),
                        max_pair_skew = max_pair_skew.diagnostic_decimal(-256),
                    )),
                ));
            }
        }

        Ok(violations)
    })
}

fn required_clearance<'a>(
    net_classes: &'a [NetClassConfig],
    left: &CopperFeature,
    right: &CopperFeature,
) -> Result<Option<(&'a str, Scalar)>, crate::PcbGeometryUncertainty> {
    Ok(matching_class_indexes_for_feature(net_classes, left)?
        .into_iter()
        .chain(matching_class_indexes_for_feature(net_classes, right)?)
        .filter_map(|class_index| {
            let class = &net_classes[class_index];
            let clearance = [
                class.min_clearance.as_ref(),
                class.min_voltage_clearance.as_ref(),
            ]
            .into_iter()
            .flatten()
            .max_by(|left, right| {
                crate::scalar::compare(left, right)
                    .expect("exact class clearances must be comparable")
            })?;
            Some((class_name(class), clearance.clone()))
        })
        .max_by(|left, right| {
            crate::scalar::compare(&left.1, &right.1)
                .expect("exact class clearances must be comparable")
        }))
}

fn maximum_configured_clearance(net_classes: &[NetClassConfig]) -> Option<Scalar> {
    net_classes
        .iter()
        .flat_map(|class| {
            [
                class.min_clearance.as_ref(),
                class.min_voltage_clearance.as_ref(),
            ]
        })
        .flatten()
        .filter(|clearance| crate::scalar::ge(clearance, &Scalar::zero()))
        .max_by(|left, right| {
            crate::scalar::compare(left, right)
                .expect("exact configured clearances must be comparable")
        })
        .cloned()
}

fn is_reference_net(net: &str) -> bool {
    let normalized = net.to_ascii_lowercase();
    normalized == "gnd"
        || normalized == "ground"
        || normalized.starts_with("gnd_")
        || normalized.starts_with("gnd-")
        || normalized.contains("shield")
        || normalized.contains("chassis")
}

fn is_blank(value: Option<&str>) -> bool {
    value.is_none_or(|value| value.trim().is_empty())
}

fn invalid_positive(value: Option<&Scalar>) -> bool {
    !value.is_some_and(|value| crate::scalar::gt(value, &Scalar::zero()))
}

fn invalid_non_negative(value: Option<&Scalar>) -> bool {
    !value.is_some_and(|value| crate::scalar::ge(value, &Scalar::zero()))
}

fn class_name(class: &NetClassConfig) -> &str {
    if class.name.trim().is_empty() {
        "unnamed"
    } else {
        &class.name
    }
}

fn parsed_copper_layers(boards: &[BoardModel]) -> BTreeSet<String> {
    boards
        .iter()
        .flat_map(|board| board.copper.iter().map(|feature| feature.layer.clone()))
        .collect()
}

fn layer_selected(layer: &str, selected_layers: &[String]) -> bool {
    selected_layers.is_empty() || selected_layers.iter().any(|selected| selected == layer)
}

#[derive(Clone, Debug)]
struct ScalarBounds2 {
    min_x: Scalar,
    min_y: Scalar,
    max_x: Scalar,
    max_y: Scalar,
}

/// Optional certified bounds. Missing certificates disable culling.
fn native_region_bounds_scalar(region: &crate::PcbRegion) -> Option<ScalarBounds2> {
    region
        .try_native_bounds()
        .ok()
        .flatten()
        .map(|bounds| ScalarBounds2 {
            min_x: bounds[0].clone(),
            min_y: bounds[1].clone(),
            max_x: bounds[2].clone(),
            max_y: bounds[3].clone(),
        })
}

fn scalar_cmp(left: &Scalar, right: &Scalar) -> Option<std::cmp::Ordering> {
    crate::scalar::compare(left, right)
}

fn scalar_gt(left: &Scalar, right: &Scalar) -> bool {
    scalar_cmp(left, right) == Some(std::cmp::Ordering::Greater)
}

fn exact_bounds_may_be_within(
    left: &ScalarBounds2,
    right: &ScalarBounds2,
    clearance: &Scalar,
) -> bool {
    !scalar_gt(&(&left.min_x - clearance), &right.max_x)
        && !scalar_gt(&right.min_x, &(&left.max_x + clearance))
        && !scalar_gt(&(&left.min_y - clearance), &right.max_y)
        && !scalar_gt(&right.min_y, &(&left.max_y + clearance))
}

fn estimated_feature_length(
    feature: &CopperFeature,
) -> Result<Option<Scalar>, crate::PcbGeometryUncertainty> {
    match feature.kind {
        // Native envelope proxy for parsed segments, not routed-path reconstruction.
        CopperKind::Segment => super::measurements::segment_length_proxy(&feature.region),
        CopperKind::Via | CopperKind::Pad | CopperKind::Zone | CopperKind::Artwork => Ok(None),
    }
}

fn optional_scalar_eq(left: Option<&Scalar>, right: Option<&Scalar>) -> bool {
    match (left, right) {
        (Some(left), Some(right)) => crate::scalar::eq(left, right),
        (None, None) => true,
        (Some(_), None) | (None, Some(_)) => false,
    }
}

#[derive(Clone, Debug)]
struct NetUse<'a> {
    layers: BTreeSet<String>,
    features: Vec<&'a CopperFeature>,
    via_count: usize,
    estimated_length: Scalar,
    max_length: Option<Scalar>,
}

impl NetUse<'_> {
    fn try_locations(&self) -> Result<Vec<[f64; 2]>, crate::PcbGeometryUncertainty> {
        self.features
            .iter()
            .map(|feature| feature.try_location_f64())
            .collect()
    }
}

impl Default for NetUse<'_> {
    fn default() -> Self {
        Self {
            layers: BTreeSet::new(),
            features: Vec::new(),
            via_count: 0,
            estimated_length: Scalar::zero(),
            max_length: None,
        }
    }
}

#[derive(Clone, Debug, Default)]
struct DifferentialPairUse<'a> {
    positive: DifferentialSideUse<'a>,
    negative: DifferentialSideUse<'a>,
}

impl<'a> DifferentialPairUse<'a> {
    fn side_mut(&mut self, role: DifferentialRole) -> &mut DifferentialSideUse<'a> {
        match role {
            DifferentialRole::Positive => &mut self.positive,
            DifferentialRole::Negative => &mut self.negative,
        }
    }

    fn locations(&self) -> Result<Vec<[f64; 2]>, crate::PcbGeometryUncertainty> {
        self.positive
            .features
            .iter()
            .chain(self.negative.features.iter())
            .map(|feature| feature.try_location_f64())
            .collect()
    }

    fn min_pair_spacing(&self) -> Option<Scalar> {
        option_max(
            self.positive.min_pair_spacing.clone(),
            self.negative.min_pair_spacing.clone(),
        )
    }

    fn max_pair_spacing(&self) -> Option<Scalar> {
        option_min(
            self.positive.max_pair_spacing.clone(),
            self.negative.max_pair_spacing.clone(),
        )
    }

    fn max_pair_skew(&self) -> Option<Scalar> {
        option_min(
            self.positive.max_pair_skew.clone(),
            self.negative.max_pair_skew.clone(),
        )
    }
}

fn option_max(left: Option<Scalar>, right: Option<Scalar>) -> Option<Scalar> {
    match (left, right) {
        (Some(left), Some(right)) => Some(if crate::scalar::ge(&left, &right) {
            left
        } else {
            right
        }),
        (Some(value), None) | (None, Some(value)) => Some(value),
        (None, None) => None,
    }
}

fn option_min(left: Option<Scalar>, right: Option<Scalar>) -> Option<Scalar> {
    match (left, right) {
        (Some(left), Some(right)) => Some(if crate::scalar::le(&left, &right) {
            left
        } else {
            right
        }),
        (Some(value), None) | (None, Some(value)) => Some(value),
        (None, None) => None,
    }
}

#[derive(Clone, Debug)]
struct DifferentialSideUse<'a> {
    net_names: BTreeSet<String>,
    layers: BTreeSet<String>,
    features: Vec<&'a CopperFeature>,
    min_pair_spacing: Option<Scalar>,
    max_pair_spacing: Option<Scalar>,
    estimated_length: Scalar,
    max_pair_skew: Option<Scalar>,
}

impl Default for DifferentialSideUse<'_> {
    fn default() -> Self {
        Self {
            net_names: BTreeSet::new(),
            layers: BTreeSet::new(),
            features: Vec::new(),
            min_pair_spacing: None,
            max_pair_spacing: None,
            estimated_length: Scalar::zero(),
            max_pair_skew: None,
        }
    }
}

impl DifferentialSideUse<'_> {
    fn try_locations(&self) -> Result<Vec<[f64; 2]>, crate::PcbGeometryUncertainty> {
        self.features
            .iter()
            .map(|feature| feature.try_location_f64())
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use crate::constraint_policy::{
        DifferentialRole, FabricationCapabilityConfig, NetClassConfig, NetClassRegionConfig,
        StackupConfig, StackupLayerConfig, StackupLayerKind, SurfaceFinish,
    };
    use crate::geometry::{circle_polygon, line_polygon, polygons_to_profile, rect_polygon};
    use crate::kicad::{BoardModel, CopperFeature, CopperKind, DrillFeature};

    use super::{
        NET_IMPEDANCE_TARGET_READINESS_CHECK, net_constraint_readiness, stackup_readiness,
    };

    #[test]
    fn stackup_readiness_reports_layer_count_and_missing_metadata() {
        let stackup = StackupConfig {
            copper_layer_count: Some(4),
            finished_thickness: Some(crate::scalar::scalar("1.6")),
            layers: vec![
                StackupLayerConfig {
                    name: "F.Cu".to_string(),
                    kind: StackupLayerKind::Copper,
                    copper_weight_oz: Some(crate::scalar::scalar("1.0")),
                    dielectric_thickness: None,
                },
                StackupLayerConfig {
                    name: "B.Cu".to_string(),
                    kind: StackupLayerKind::Copper,
                    copper_weight_oz: None,
                    dielectric_thickness: None,
                },
            ],
            ..StackupConfig::default()
        };
        let board = board_with_features(vec![feature(
            "F.Cu",
            "GND",
            CopperKind::Zone,
            [0.0, 0.0],
            2.0,
            2.0,
        )]);

        let messages = stackup_readiness(Some(&stackup), &[board])
            .into_iter()
            .filter_map(|violation| violation.message)
            .collect::<Vec<_>>();

        assert!(
            messages
                .iter()
                .any(|message| message.contains("declares 4"))
        );
        assert!(
            messages
                .iter()
                .any(|message| message.contains("missing copper_weight_oz"))
        );
        assert!(
            messages
                .iter()
                .any(|message| message.contains("finished_thickness"))
        );
        assert!(
            messages
                .iter()
                .any(|message| message.contains("material_family"))
        );
        assert!(
            messages
                .iter()
                .any(|message| message.contains("surface_finish"))
        );
    }

    #[test]
    fn stackup_readiness_accepts_complete_process_metadata() {
        let stackup = complete_stackup(Some(SurfaceFinish::Enig), Some(true));
        let board = board_with_features(vec![
            feature("F.Cu", "GND", CopperKind::Zone, [0.0, 0.0], 2.0, 2.0),
            feature("B.Cu", "GND", CopperKind::Zone, [0.0, 0.0], 2.0, 2.0),
        ]);

        assert!(stackup_readiness(Some(&stackup), &[board]).is_empty());
    }

    #[test]
    fn stackup_readiness_reports_hasl_controlled_impedance_finish_risk() {
        let stackup = complete_stackup(Some(SurfaceFinish::LeadFreeHasl), Some(true));

        let messages = stackup_readiness(Some(&stackup), &[])
            .into_iter()
            .filter_map(|violation| violation.message)
            .collect::<Vec<_>>();

        assert!(
            messages
                .iter()
                .any(|message| message.contains("HASL-style finish"))
        );
    }

    #[test]
    fn stackup_readiness_reports_fabricator_capability_thresholds() {
        let mut stackup = complete_stackup(Some(SurfaceFinish::Enig), Some(false));
        stackup.copper_layer_count = Some(6);
        stackup.finished_thickness = Some(crate::scalar::scalar("0.3"));
        stackup.layers = vec![
            StackupLayerConfig {
                name: "F.Cu".to_string(),
                kind: StackupLayerKind::Copper,
                copper_weight_oz: Some(crate::scalar::scalar("3.0")),
                dielectric_thickness: None,
            },
            StackupLayerConfig {
                name: "In1.Cu".to_string(),
                kind: StackupLayerKind::Copper,
                copper_weight_oz: Some(crate::scalar::scalar("1.0")),
                dielectric_thickness: None,
            },
            StackupLayerConfig {
                name: "In2.Cu".to_string(),
                kind: StackupLayerKind::Copper,
                copper_weight_oz: Some(crate::scalar::scalar("1.0")),
                dielectric_thickness: None,
            },
            StackupLayerConfig {
                name: "In3.Cu".to_string(),
                kind: StackupLayerKind::Copper,
                copper_weight_oz: Some(crate::scalar::scalar("1.0")),
                dielectric_thickness: None,
            },
            StackupLayerConfig {
                name: "In4.Cu".to_string(),
                kind: StackupLayerKind::Copper,
                copper_weight_oz: Some(crate::scalar::scalar("1.0")),
                dielectric_thickness: None,
            },
            StackupLayerConfig {
                name: "B.Cu".to_string(),
                kind: StackupLayerKind::Copper,
                copper_weight_oz: Some(crate::scalar::scalar("1.0")),
                dielectric_thickness: None,
            },
            StackupLayerConfig {
                name: "Core".to_string(),
                kind: StackupLayerKind::Core,
                copper_weight_oz: None,
                dielectric_thickness: Some(crate::scalar::scalar("0.02")),
            },
        ];

        let messages = stackup_readiness(Some(&stackup), &[])
            .into_iter()
            .filter_map(|violation| violation.message)
            .collect::<Vec<_>>();

        assert!(
            messages
                .iter()
                .any(|message| message.contains("supports up to 4"))
        );
        assert!(
            messages
                .iter()
                .any(|message| message.contains("finished_thickness 0.300000 is below"))
        );
        assert!(
            messages
                .iter()
                .any(|message| message.contains("copper_weight_oz 3.000000 above"))
        );
        assert!(
            messages
                .iter()
                .any(|message| message.contains("dielectric_thickness 0.020000 below"))
        );
    }

    #[test]
    fn stackup_readiness_uses_custom_fabrication_capability_overrides() {
        let mut stackup = complete_stackup(Some(SurfaceFinish::Enig), Some(false));
        stackup.fabricator_profile = Some("custom-shop".to_string());
        stackup.fabrication_capability = FabricationCapabilityConfig {
            max_copper_layers: Some(1),
            min_finished_thickness: Some(crate::scalar::scalar("2.0")),
            ..FabricationCapabilityConfig::default()
        };

        let messages = stackup_readiness(Some(&stackup), &[])
            .into_iter()
            .filter_map(|violation| violation.message)
            .collect::<Vec<_>>();

        assert!(
            messages
                .iter()
                .any(|message| message.contains("fabricator profile custom-shop supports up to 1"))
        );
        assert!(
            messages
                .iter()
                .any(|message| message.contains("finished_thickness 1.600000 is below"))
        );
    }

    #[test]
    fn stackup_readiness_reports_vendor_service_class_thresholds() {
        let mut stackup = complete_stackup(Some(SurfaceFinish::Enig), Some(false));
        stackup.fabricator_profile = Some("pcbway-standard".to_string());
        stackup.finished_thickness = Some(crate::scalar::scalar("2.8"));
        stackup.layers = (0..12)
            .map(|index| StackupLayerConfig {
                name: format!("L{}.Cu", index + 1),
                kind: StackupLayerKind::Copper,
                copper_weight_oz: Some(if index == 0 {
                    crate::scalar::scalar("3.5")
                } else {
                    crate::scalar::scalar("1.0")
                }),
                dielectric_thickness: None,
            })
            .chain(std::iter::once(StackupLayerConfig {
                name: "Core".to_string(),
                kind: StackupLayerKind::Core,
                copper_weight_oz: None,
                dielectric_thickness: Some(crate::scalar::scalar("0.045")),
            }))
            .collect();

        let messages = stackup_readiness(Some(&stackup), &[])
            .into_iter()
            .filter_map(|violation| violation.message)
            .collect::<Vec<_>>();

        assert!(
            messages
                .iter()
                .any(|message| message.contains("pcbway-standard preferred service"))
        );
        assert!(
            messages
                .iter()
                .any(|message| message.contains("pcbway-standard cost-escalation threshold"))
        );
        assert!(
            messages
                .iter()
                .any(|message| message.contains("finished_thickness 2.800000 is above"))
        );
        assert!(
            messages
                .iter()
                .any(|message| message.contains("copper_weight_oz 3.500000 above"))
        );
        assert!(
            messages
                .iter()
                .any(|message| message.contains("dielectric_thickness 0.045000 below"))
        );
    }

    #[test]
    fn stackup_readiness_uses_custom_preferred_and_cost_thresholds() {
        let mut stackup = complete_stackup(Some(SurfaceFinish::Enig), Some(false));
        stackup.fabricator_profile = Some("custom-shop".to_string());
        stackup.layers[0].copper_weight_oz = Some(crate::scalar::scalar("2.5"));
        stackup.fabrication_capability = FabricationCapabilityConfig {
            preferred_max_copper_weight_oz: Some(crate::scalar::scalar("1.0")),
            cost_escalation_copper_weight_oz: Some(crate::scalar::scalar("2.0")),
            ..FabricationCapabilityConfig::default()
        };

        let messages = stackup_readiness(Some(&stackup), &[])
            .into_iter()
            .filter_map(|violation| violation.message)
            .collect::<Vec<_>>();

        assert!(messages.iter().any(|message| {
            message.contains("fabricator profile custom-shop preferred maximum")
        }));
        assert!(
            messages.iter().any(|message| {
                message.contains("fabricator profile custom-shop cost-escalation")
            })
        );
    }

    #[test]
    fn stackup_readiness_reports_material_property_ranges() {
        let mut stackup = complete_stackup(Some(SurfaceFinish::Enig), Some(true));
        stackup.fabricator_profile = Some("custom-material-window".to_string());
        stackup.material_dielectric_constant = Some(crate::scalar::scalar("5.2"));
        stackup.material_loss_tangent = Some(crate::scalar::scalar("0.035"));
        stackup.material_tg_c = Some(crate::scalar::scalar("125.0"));
        stackup.fabrication_capability = FabricationCapabilityConfig {
            min_dielectric_constant: Some(crate::scalar::scalar("3.0")),
            max_dielectric_constant: Some(crate::scalar::scalar("4.8")),
            max_loss_tangent: Some(crate::scalar::scalar("0.02")),
            min_tg_c: Some(crate::scalar::scalar("140.0")),
            ..FabricationCapabilityConfig::default()
        };

        let messages = stackup_readiness(Some(&stackup), &[])
            .into_iter()
            .filter_map(|violation| violation.message)
            .collect::<Vec<_>>();

        assert!(
            messages
                .iter()
                .any(|message| message.contains("material_dielectric_constant 5.200000"))
        );
        assert!(
            messages
                .iter()
                .any(|message| message.contains("material_loss_tangent 0.035000"))
        );
        assert!(
            messages
                .iter()
                .any(|message| message.contains("material_tg_c 125.000000"))
        );
    }

    #[test]
    fn net_constraint_readiness_reports_width_clearance_and_via_rules() {
        let classes = vec![NetClassConfig {
            name: "power".to_string(),
            nets: vec!["VBUS".to_string()],
            net_patterns: vec!["PWR_*".to_string()],
            min_width: Some(crate::scalar::scalar("0.5")),
            min_clearance: Some(crate::scalar::scalar("0.4")),
            max_layer_count: Some(1),
            min_via_count: Some(2),
            max_length: Some(crate::scalar::scalar("1.0")),
            ..NetClassConfig::default()
        }];
        let board = board_with_features(vec![
            feature("F.Cu", "VBUS", CopperKind::Segment, [0.0, 0.0], 2.0, 0.2),
            feature("B.Cu", "VBUS", CopperKind::Segment, [0.0, 0.0], 2.0, 0.2),
            feature("F.Cu", "SIG", CopperKind::Segment, [0.3, 0.0], 0.2, 0.2),
        ]);

        let messages = net_constraint_readiness(&classes, None, &[board], &[])
            .into_iter()
            .filter_map(|violation| violation.message)
            .collect::<Vec<_>>();

        assert!(
            messages
                .iter()
                .any(|message| message.contains("below configured minimum"))
        );
        assert!(
            messages
                .iter()
                .any(|message| message.contains("below configured clearance"))
        );
        assert!(
            messages
                .iter()
                .any(|message| message.contains("above configured maximum"))
        );
        assert!(
            messages
                .iter()
                .any(|message| message.contains("below configured minimum 2"))
        );
        assert!(
            messages
                .iter()
                .any(|message| message.contains("approximate parsed copper length"))
        );
    }

    #[test]
    fn net_constraint_readiness_uses_segment_edge_length_for_diagonal_traces() {
        let classes = vec![NetClassConfig {
            name: "matched signal".to_string(),
            nets: vec!["SIG".to_string()],
            max_length: Some(crate::scalar::scalar("4.8")),
            ..NetClassConfig::default()
        }];
        let polygon = line_polygon([0.0, 0.0], [3.0, 4.0], 0.2)
            .expect("diagonal segment should produce copper geometry");
        let board = board_with_features(vec![CopperFeature {
            layer: "F.Cu".to_string(),
            net: Some("SIG".to_string()),
            kind: CopperKind::Segment,
            region: polygons_to_profile(vec![polygon], None),
            location: [crate::scalar::scalar("1.5"), crate::scalar::scalar("2")],
        }]);

        let messages = net_constraint_readiness(&classes, None, &[board], &[])
            .into_iter()
            .filter_map(|violation| violation.message)
            .collect::<Vec<_>>();

        assert!(
            messages
                .iter()
                .any(|message| message.contains("approximate parsed copper length 5.000000"))
        );
    }

    #[test]
    fn net_constraint_readiness_allows_unmatched_or_compliant_nets() {
        let classes = vec![NetClassConfig {
            name: "power".to_string(),
            nets: vec!["VBUS".to_string()],
            min_width: Some(crate::scalar::scalar("0.5")),
            min_clearance: Some(crate::scalar::scalar("0.2")),
            max_layer_count: Some(1),
            min_via_count: Some(1),
            ..NetClassConfig::default()
        }];
        let board = board_with_features(vec![
            feature("F.Cu", "VBUS", CopperKind::Segment, [0.0, 0.0], 1.0, 0.6),
            feature("F.Cu", "SIG", CopperKind::Segment, [2.0, 0.0], 0.5, 0.5),
            feature("B.Cu", "OTHER", CopperKind::Segment, [0.0, 0.0], 0.1, 0.1),
        ]);

        assert!(net_constraint_readiness(&classes, None, &[board], &[]).is_empty());
    }

    #[test]
    fn net_constraint_readiness_resolves_inherited_net_class_rules() {
        let classes = vec![
            NetClassConfig {
                name: "high-energy-defaults".to_string(),
                min_width: Some(crate::scalar::scalar("0.5")),
                min_clearance: Some(crate::scalar::scalar("0.4")),
                max_layer_count: Some(1),
                ..NetClassConfig::default()
            },
            NetClassConfig {
                name: "power".to_string(),
                extends: vec!["high-energy-defaults".to_string()],
                nets: vec!["VBUS".to_string()],
                ..NetClassConfig::default()
            },
        ];
        let board = board_with_features(vec![
            feature("F.Cu", "VBUS", CopperKind::Segment, [0.0, 0.0], 2.0, 0.2),
            feature("B.Cu", "VBUS", CopperKind::Segment, [0.0, 0.0], 2.0, 0.2),
            feature("F.Cu", "SIG", CopperKind::Segment, [0.3, 0.0], 0.2, 0.2),
        ]);

        let messages = net_constraint_readiness(&classes, None, &[board], &[])
            .into_iter()
            .filter_map(|violation| violation.message)
            .collect::<Vec<_>>();

        assert!(
            messages
                .iter()
                .any(|message| message.contains("below configured minimum"))
        );
        assert!(
            messages
                .iter()
                .any(|message| message.contains("below configured clearance"))
        );
        assert!(
            messages
                .iter()
                .any(|message| message.contains("appears on 2 layer"))
        );
    }

    #[test]
    fn net_constraint_readiness_reports_inheritance_diagnostics() {
        let classes = vec![
            NetClassConfig {
                name: "cycle-a".to_string(),
                extends: vec!["cycle-b".to_string()],
                nets: vec!["A".to_string()],
                ..NetClassConfig::default()
            },
            NetClassConfig {
                name: "cycle-b".to_string(),
                extends: vec!["cycle-a".to_string(), "missing-parent".to_string()],
                nets: vec!["B".to_string()],
                ..NetClassConfig::default()
            },
        ];
        let board = board_with_features(vec![feature(
            "F.Cu",
            "A",
            CopperKind::Segment,
            [0.0, 0.0],
            1.0,
            0.5,
        )]);

        let messages = net_constraint_readiness(&classes, None, &[board], &[])
            .into_iter()
            .filter_map(|violation| violation.message)
            .collect::<Vec<_>>();

        assert!(
            messages
                .iter()
                .any(|message| message.contains("inheritance cycle"))
        );
        assert!(
            messages
                .iter()
                .any(|message| message.contains("missing parent"))
        );
    }

    #[test]
    fn net_constraint_readiness_scopes_net_class_rules_to_regions() {
        let classes = vec![NetClassConfig {
            name: "front-end signal".to_string(),
            nets: vec!["SIG".to_string()],
            regions: vec![NetClassRegionConfig {
                name: "front-end".to_string(),
                min_x: Some(crate::scalar::scalar("0.0")),
                min_y: Some(crate::scalar::scalar("0.0")),
                max_x: Some(crate::scalar::scalar("2.0")),
                max_y: Some(crate::scalar::scalar("2.0")),
                layers: vec!["F.Cu".to_string()],
            }],
            min_width: Some(crate::scalar::scalar("0.5")),
            min_clearance: Some(crate::scalar::scalar("0.3")),
            ..NetClassConfig::default()
        }];
        let board = board_with_features(vec![
            feature("F.Cu", "SIG", CopperKind::Segment, [1.0, 1.0], 1.0, 0.1),
            feature("F.Cu", "OTHER", CopperKind::Segment, [1.0, 1.15], 0.1, 0.1),
            feature("F.Cu", "SIG", CopperKind::Segment, [5.0, 1.0], 1.0, 0.1),
            feature("F.Cu", "OTHER", CopperKind::Segment, [5.0, 1.15], 0.1, 0.1),
            feature("B.Cu", "SIG", CopperKind::Segment, [1.0, 1.0], 1.0, 0.1),
        ]);

        let messages = net_constraint_readiness(&classes, None, &[board], &[])
            .into_iter()
            .filter_map(|violation| violation.message)
            .collect::<Vec<_>>();

        assert_eq!(
            messages
                .iter()
                .filter(|message| message.contains("below configured minimum"))
                .count(),
            1
        );
        assert_eq!(
            messages
                .iter()
                .filter(|message| message.contains("below configured clearance"))
                .count(),
            1
        );
    }

    #[test]
    fn net_constraint_readiness_reports_invalid_region_scopes() {
        let classes = vec![NetClassConfig {
            name: "bad region".to_string(),
            nets: vec!["SIG".to_string()],
            regions: vec![NetClassRegionConfig {
                min_x: Some(crate::scalar::scalar("2.0")),
                min_y: Some(crate::scalar::scalar("0.0")),
                max_x: Some(crate::scalar::scalar("1.0")),
                max_y: Some(crate::scalar::scalar("2.0")),
                ..NetClassRegionConfig::default()
            }],
            min_width: Some(crate::scalar::scalar("0.5")),
            ..NetClassConfig::default()
        }];
        let board = board_with_features(vec![feature(
            "F.Cu",
            "SIG",
            CopperKind::Segment,
            [1.0, 1.0],
            1.0,
            0.1,
        )]);

        let messages = net_constraint_readiness(&classes, None, &[board], &[])
            .into_iter()
            .filter_map(|violation| violation.message)
            .collect::<Vec<_>>();

        assert!(
            messages
                .iter()
                .any(|message| message.contains("region #0 is invalid"))
        );
        assert!(
            !messages
                .iter()
                .any(|message| message.contains("below configured minimum"))
        );
    }

    #[test]
    fn net_constraint_clearance_culls_sparse_same_layer_fields() {
        let classes = vec![NetClassConfig {
            name: "power".to_string(),
            nets: vec!["VBUS".to_string()],
            min_clearance: Some(crate::scalar::scalar("0.4")),
            ..NetClassConfig::default()
        }];
        let mut copper = (0..2_000)
            .map(|index| {
                feature(
                    "F.Cu",
                    &format!("SIG{index}"),
                    CopperKind::Segment,
                    [100.0 + (index % 50) as f64 * 5.0, (index / 50) as f64 * 5.0],
                    0.2,
                    0.2,
                )
            })
            .collect::<Vec<_>>();
        copper.push(feature(
            "F.Cu",
            "VBUS",
            CopperKind::Segment,
            [0.0, 0.0],
            0.2,
            0.2,
        ));
        copper.push(feature(
            "F.Cu",
            "SIG_NEAR",
            CopperKind::Segment,
            [0.3, 0.0],
            0.2,
            0.2,
        ));
        let board = board_with_features(copper);

        let started = std::time::Instant::now();
        let violations = net_constraint_readiness(&classes, None, &[board], &[]);

        assert_eq!(
            violations
                .iter()
                .filter(|violation| violation
                    .message
                    .as_deref()
                    .is_some_and(|message| message.contains("below configured clearance")))
                .count(),
            1
        );
        assert!(
            started.elapsed() < std::time::Duration::from_secs(2),
            "net clearance constraints should cull sparse same-layer copper before exact CSG"
        );
    }

    #[test]
    fn net_constraint_readiness_reports_current_voltage_plane_and_impedance_rules() {
        let classes = vec![NetClassConfig {
            name: "critical".to_string(),
            nets: vec!["USB_D+".to_string()],
            min_current_width: Some(crate::scalar::scalar("0.25")),
            min_voltage_clearance: Some(crate::scalar::scalar("0.5")),
            requires_reference_plane: Some(true),
            requires_impedance_control: Some(true),
            ..NetClassConfig::default()
        }];
        let stackup = StackupConfig {
            copper_layer_count: Some(2),
            finished_thickness: Some(crate::scalar::scalar("1.6")),
            layers: vec![StackupLayerConfig {
                name: "F.Cu".to_string(),
                kind: StackupLayerKind::Copper,
                copper_weight_oz: None,
                dielectric_thickness: None,
            }],
            material_family: Some("FR-4".to_string()),
            surface_finish: Some(SurfaceFinish::Enig),
            soldermask_process: Some("LPI".to_string()),
            soldermask_color: Some("green".to_string()),
            target_ipc_class: Some("IPC Class 2".to_string()),
            fabricator_profile: Some("prototype-fab".to_string()),
            ..StackupConfig::default()
        };
        let board = board_with_features(vec![
            feature("F.Cu", "USB_D+", CopperKind::Segment, [0.0, 0.0], 1.0, 0.1),
            feature("F.Cu", "SIG", CopperKind::Segment, [0.4, 0.0], 0.1, 0.1),
        ]);

        let messages = net_constraint_readiness(&classes, Some(&stackup), &[board], &[])
            .into_iter()
            .filter_map(|violation| violation.message)
            .collect::<Vec<_>>();

        assert!(
            messages
                .iter()
                .any(|message| message.contains("current-carrying minimum"))
        );
        assert!(
            messages
                .iter()
                .any(|message| message.contains("below configured clearance"))
        );
        assert!(
            messages
                .iter()
                .any(|message| message.contains("reference-plane review"))
        );
        assert!(
            messages
                .iter()
                .any(|message| message.contains("impedance-control review"))
        );
        assert!(
            messages
                .iter()
                .any(|message| message.contains("target_impedance_ohms is missing"))
        );
        assert!(
            messages
                .iter()
                .any(|message| message.contains("impedance_tolerance_ohms is missing"))
        );
    }

    #[test]
    fn net_constraint_readiness_allows_explicit_plane_and_stackup_metadata() {
        let classes = vec![NetClassConfig {
            name: "critical".to_string(),
            nets: vec!["USB_D+".to_string()],
            min_current_width: Some(crate::scalar::scalar("0.25")),
            min_voltage_clearance: Some(crate::scalar::scalar("0.2")),
            requires_reference_plane: Some(true),
            requires_impedance_control: Some(true),
            target_impedance_ohms: Some(crate::scalar::scalar("50.0")),
            impedance_tolerance_ohms: Some(crate::scalar::scalar("10.0")),
            ..NetClassConfig::default()
        }];
        let stackup = StackupConfig {
            impedance_controlled: Some(true),
            material_family: Some("FR-4".to_string()),
            surface_finish: Some(SurfaceFinish::Enig),
            soldermask_process: Some("LPI".to_string()),
            soldermask_color: Some("green".to_string()),
            target_ipc_class: Some("IPC Class 2".to_string()),
            fabricator_profile: Some("prototype-fab".to_string()),
            fabrication_capability: FabricationCapabilityConfig::default(),
            material_dielectric_constant: Some(crate::scalar::scalar("4.2")),
            material_loss_tangent: Some(crate::scalar::scalar("0.018")),
            material_tg_c: Some(crate::scalar::scalar("150.0")),
            layers: vec![
                StackupLayerConfig {
                    name: "F.Cu".to_string(),
                    kind: StackupLayerKind::Copper,
                    copper_weight_oz: Some(crate::scalar::scalar("1.0")),
                    dielectric_thickness: None,
                },
                StackupLayerConfig {
                    name: "Core".to_string(),
                    kind: StackupLayerKind::Core,
                    copper_weight_oz: None,
                    dielectric_thickness: Some(crate::scalar::scalar("0.18")),
                },
                StackupLayerConfig {
                    name: "B.Cu".to_string(),
                    kind: StackupLayerKind::Copper,
                    copper_weight_oz: Some(crate::scalar::scalar("1.0")),
                    dielectric_thickness: None,
                },
            ],
            ..StackupConfig::default()
        };
        let board = board_with_features(vec![
            feature("F.Cu", "USB_D+", CopperKind::Segment, [0.0, 0.0], 0.32, 1.0),
            feature("F.Cu", "SIG", CopperKind::Segment, [1.0, 0.0], 0.1, 0.1),
            feature("F.Cu", "GND", CopperKind::Zone, [0.0, -1.0], 4.0, 0.2),
        ]);

        assert!(net_constraint_readiness(&classes, Some(&stackup), &[board], &[]).is_empty());
    }

    #[test]
    fn net_constraint_readiness_reports_microstrip_impedance_outside_target() {
        let classes = vec![NetClassConfig {
            name: "rf".to_string(),
            nets: vec!["RF".to_string()],
            requires_impedance_control: Some(true),
            target_impedance_ohms: Some(crate::scalar::scalar("50.0")),
            impedance_tolerance_ohms: Some(crate::scalar::scalar("5.0")),
            ..NetClassConfig::default()
        }];
        let stackup = controlled_microstrip_stackup();
        let board = board_with_features(vec![feature(
            "F.Cu",
            "RF",
            CopperKind::Segment,
            [0.0, 0.0],
            0.08,
            1.0,
        )]);

        let violations = net_constraint_readiness(&classes, Some(&stackup), &[board], &[]);

        assert!(violations.iter().any(|violation| {
            violation.check == NET_IMPEDANCE_TARGET_READINESS_CHECK
                && violation.message.as_deref().is_some_and(|message| {
                    message.contains("estimated outer microstrip impedance")
                        && message.contains("outside target")
                })
        }));
    }

    #[test]
    fn net_constraint_readiness_reports_unsupported_impedance_geometry() {
        let classes = vec![NetClassConfig {
            name: "rf".to_string(),
            nets: vec!["RF".to_string()],
            requires_impedance_control: Some(true),
            target_impedance_ohms: Some(crate::scalar::scalar("50.0")),
            impedance_tolerance_ohms: Some(crate::scalar::scalar("5.0")),
            ..NetClassConfig::default()
        }];
        let mut stackup = controlled_microstrip_stackup();
        stackup.layers.retain(|layer| layer.name.as_str() != "B.Cu");
        let board = board_with_features(vec![feature(
            "F.Cu",
            "RF",
            CopperKind::Segment,
            [0.0, 0.0],
            0.32,
            1.0,
        )]);

        let violations = net_constraint_readiness(&classes, Some(&stackup), &[board], &[]);

        assert!(violations.iter().any(|violation| {
            violation.check == NET_IMPEDANCE_TARGET_READINESS_CHECK
                && violation
                    .message
                    .as_deref()
                    .is_some_and(|message| message.contains("outside the supported"))
        }));
    }

    #[test]
    fn net_constraint_readiness_allows_microstrip_impedance_inside_target() {
        let classes = vec![NetClassConfig {
            name: "rf".to_string(),
            nets: vec!["RF".to_string()],
            requires_impedance_control: Some(true),
            target_impedance_ohms: Some(crate::scalar::scalar("50.0")),
            impedance_tolerance_ohms: Some(crate::scalar::scalar("10.0")),
            ..NetClassConfig::default()
        }];
        let stackup = controlled_microstrip_stackup();
        let board = board_with_features(vec![feature(
            "F.Cu",
            "RF",
            CopperKind::Segment,
            [0.0, 0.0],
            0.32,
            1.0,
        )]);

        assert!(net_constraint_readiness(&classes, Some(&stackup), &[board], &[]).is_empty());
    }

    #[test]
    fn net_constraint_readiness_reports_centered_stripline_impedance_outside_target() {
        let classes = vec![NetClassConfig {
            name: "rf".to_string(),
            nets: vec!["RF_INNER".to_string()],
            requires_impedance_control: Some(true),
            target_impedance_ohms: Some(crate::scalar::scalar("50.0")),
            impedance_tolerance_ohms: Some(crate::scalar::scalar("5.0")),
            ..NetClassConfig::default()
        }];
        let stackup = controlled_centered_stripline_stackup();
        let board = board_with_features(vec![feature(
            "In1.Cu",
            "RF_INNER",
            CopperKind::Segment,
            [0.0, 0.0],
            0.05,
            1.0,
        )]);

        let messages = net_constraint_readiness(&classes, Some(&stackup), &[board], &[])
            .into_iter()
            .filter_map(|violation| violation.message)
            .collect::<Vec<_>>();

        assert!(messages.iter().any(|message| {
            message.contains("estimated centered stripline impedance")
                && message.contains("outside target")
        }));
    }

    #[test]
    fn net_constraint_readiness_allows_centered_stripline_impedance_inside_target() {
        let classes = vec![NetClassConfig {
            name: "rf".to_string(),
            nets: vec!["RF_INNER".to_string()],
            requires_impedance_control: Some(true),
            target_impedance_ohms: Some(crate::scalar::scalar("50.0")),
            impedance_tolerance_ohms: Some(crate::scalar::scalar("6.0")),
            ..NetClassConfig::default()
        }];
        let stackup = controlled_centered_stripline_stackup();
        let board = board_with_features(vec![feature(
            "In1.Cu",
            "RF_INNER",
            CopperKind::Segment,
            [0.0, 0.0],
            0.17,
            1.0,
        )]);

        assert!(net_constraint_readiness(&classes, Some(&stackup), &[board], &[]).is_empty());
    }

    #[test]
    fn net_constraint_readiness_estimates_supported_differential_target() {
        let classes = vec![
            NetClassConfig {
                name: "usb positive".to_string(),
                nets: vec!["USB_D+".to_string()],
                requires_impedance_control: Some(true),
                target_impedance_ohms: Some(crate::scalar::scalar("90.0")),
                impedance_tolerance_ohms: Some(crate::scalar::scalar("5.0")),
                differential_pair: Some("usb".to_string()),
                differential_role: Some(DifferentialRole::Positive),
                ..NetClassConfig::default()
            },
            NetClassConfig {
                name: "usb negative".to_string(),
                nets: vec!["USB_D-".to_string()],
                requires_impedance_control: Some(true),
                target_impedance_ohms: Some(crate::scalar::scalar("90.0")),
                impedance_tolerance_ohms: Some(crate::scalar::scalar("5.0")),
                differential_pair: Some("usb".to_string()),
                differential_role: Some(DifferentialRole::Negative),
                ..NetClassConfig::default()
            },
        ];
        let stackup = controlled_microstrip_stackup();
        let board = board_with_features(vec![
            feature("F.Cu", "USB_D+", CopperKind::Segment, [0.0, 0.0], 0.32, 1.0),
            feature("F.Cu", "USB_D-", CopperKind::Segment, [0.5, 0.0], 0.32, 1.0),
        ]);

        let violations = net_constraint_readiness(&classes, Some(&stackup), &[board], &[]);
        assert!(
            violations
                .iter()
                .all(|violation| violation.check != NET_IMPEDANCE_TARGET_READINESS_CHECK),
            "{violations:?}"
        );

        let mut mismatched_classes = classes.clone();
        for class in &mut mismatched_classes {
            class.target_impedance_ohms = Some(crate::scalar::scalar("110.0"));
            class.impedance_tolerance_ohms = Some(crate::scalar::scalar("2.0"));
        }
        let board = board_with_features(vec![
            feature("F.Cu", "USB_D+", CopperKind::Segment, [0.0, 0.0], 0.32, 1.0),
            feature("F.Cu", "USB_D-", CopperKind::Segment, [0.5, 0.0], 0.32, 1.0),
        ]);
        let violations =
            net_constraint_readiness(&mismatched_classes, Some(&stackup), &[board], &[]);
        assert!(violations.iter().any(|violation| {
            violation.check == NET_IMPEDANCE_TARGET_READINESS_CHECK
                && violation.message.as_deref().is_some_and(|message| {
                    message.contains("estimated edge-coupled outer microstrip impedance")
                        && message.contains("outside target 110")
                })
        }));

        let board = board_with_features(vec![
            feature("F.Cu", "USB_D+", CopperKind::Segment, [0.0, 0.0], 0.32, 1.0),
            feature("F.Cu", "USB_D-", CopperKind::Segment, [0.5, 0.0], 0.20, 1.0),
        ]);
        let violations = net_constraint_readiness(&classes, Some(&stackup), &[board], &[]);
        assert!(violations.iter().any(|violation| {
            violation.check == NET_IMPEDANCE_TARGET_READINESS_CHECK
                && violation
                    .message
                    .as_deref()
                    .is_some_and(|message| message.contains("requires equal widths"))
        }));
    }

    #[test]
    fn net_constraint_readiness_reports_declared_differential_pair_rules() {
        let classes = vec![
            NetClassConfig {
                name: "usb-p".to_string(),
                nets: vec!["USB_D+".to_string()],
                differential_pair: Some("usb".to_string()),
                differential_role: Some(DifferentialRole::Positive),
                min_pair_spacing: Some(crate::scalar::scalar("0.2")),
                max_pair_spacing: Some(crate::scalar::scalar("0.4")),
                max_via_count: Some(0),
                max_pair_skew: Some(crate::scalar::scalar("0.5")),
                ..NetClassConfig::default()
            },
            NetClassConfig {
                name: "usb-n".to_string(),
                nets: vec!["USB_D-".to_string()],
                differential_pair: Some("usb".to_string()),
                differential_role: Some(DifferentialRole::Negative),
                min_pair_spacing: Some(crate::scalar::scalar("0.2")),
                max_pair_spacing: Some(crate::scalar::scalar("0.4")),
                max_pair_skew: Some(crate::scalar::scalar("0.5")),
                ..NetClassConfig::default()
            },
            NetClassConfig {
                name: "missing-n".to_string(),
                nets: vec!["PCIE_TX_P".to_string()],
                differential_pair: Some("pcie-tx".to_string()),
                differential_role: Some(DifferentialRole::Positive),
                ..NetClassConfig::default()
            },
        ];
        let board = board_with_features(vec![
            feature("F.Cu", "USB_D+", CopperKind::Segment, [0.0, 0.0], 0.2, 0.2),
            feature("F.Cu", "USB_D-", CopperKind::Segment, [0.25, 0.0], 0.2, 0.2),
            feature("F.Cu", "USB_D+", CopperKind::Segment, [2.0, 0.0], 2.0, 0.2),
            feature("B.Cu", "USB_D-", CopperKind::Segment, [0.0, 0.0], 0.2, 0.2),
            feature("F.Cu", "USB_D+", CopperKind::Via, [1.0, 0.0], 0.2, 0.2),
            feature(
                "F.Cu",
                "PCIE_TX_P",
                CopperKind::Segment,
                [2.0, 0.0],
                0.2,
                0.2,
            ),
        ]);

        let messages = net_constraint_readiness(&classes, None, &[board], &[])
            .into_iter()
            .filter_map(|violation| violation.message)
            .collect::<Vec<_>>();

        assert!(
            messages
                .iter()
                .any(|message| message.contains("above configured maximum 0"))
        );
        assert!(
            messages
                .iter()
                .any(|message| message.contains("different selected copper layer sets"))
        );
        assert!(
            messages
                .iter()
                .any(|message| message.contains("below configured minimum 0.200000"))
        );
        assert!(
            messages
                .iter()
                .any(|message| message.contains("missing configured negative side"))
        );
        assert!(
            messages
                .iter()
                .any(|message| message.contains("approximate parsed length skew"))
        );
    }

    #[test]
    fn net_constraint_differential_pair_spacing_culls_sparse_side_fields() {
        let classes = vec![
            NetClassConfig {
                name: "usb-p".to_string(),
                nets: vec!["USB_D+".to_string()],
                differential_pair: Some("usb".to_string()),
                differential_role: Some(DifferentialRole::Positive),
                min_pair_spacing: Some(crate::scalar::scalar("0.2")),
                max_pair_spacing: Some(crate::scalar::scalar("0.5")),
                ..NetClassConfig::default()
            },
            NetClassConfig {
                name: "usb-n".to_string(),
                nets: vec!["USB_D-".to_string()],
                differential_pair: Some("usb".to_string()),
                differential_role: Some(DifferentialRole::Negative),
                min_pair_spacing: Some(crate::scalar::scalar("0.2")),
                max_pair_spacing: Some(crate::scalar::scalar("0.5")),
                ..NetClassConfig::default()
            },
        ];
        let mut copper = (0..1_000)
            .flat_map(|index| {
                [
                    feature(
                        "F.Cu",
                        "USB_D+",
                        CopperKind::Segment,
                        [100.0 + index as f64 * 4.0, 0.0],
                        0.10,
                        0.10,
                    ),
                    feature(
                        "F.Cu",
                        "USB_D-",
                        CopperKind::Segment,
                        [100.0 + index as f64 * 4.0, 2.0],
                        0.10,
                        0.10,
                    ),
                ]
            })
            .collect::<Vec<_>>();
        copper.push(feature(
            "F.Cu",
            "USB_D+",
            CopperKind::Segment,
            [0.0, 0.0],
            0.10,
            0.10,
        ));
        copper.push(feature(
            "F.Cu",
            "USB_D-",
            CopperKind::Segment,
            [0.18, 0.0],
            0.10,
            0.10,
        ));
        let board = board_with_features(copper);

        let started = std::time::Instant::now();
        let messages = net_constraint_readiness(&classes, None, &[board], &[])
            .into_iter()
            .filter_map(|violation| violation.message)
            .collect::<Vec<_>>();

        assert!(
            messages
                .iter()
                .any(|message| message.contains("below configured minimum 0.200000"))
        );
        assert!(
            started.elapsed() < std::time::Duration::from_secs(2),
            "differential-pair spacing should cull sparse repeated side fields before exact CSG"
        );
    }

    #[test]
    fn net_constraint_differential_pair_max_spacing_reports_nearest_side_gap() {
        let classes = vec![
            NetClassConfig {
                name: "usb-p".to_string(),
                nets: vec!["USB_D+".to_string()],
                differential_pair: Some("usb".to_string()),
                differential_role: Some(DifferentialRole::Positive),
                max_pair_spacing: Some(crate::scalar::scalar("0.5")),
                ..NetClassConfig::default()
            },
            NetClassConfig {
                name: "usb-n".to_string(),
                nets: vec!["USB_D-".to_string()],
                differential_pair: Some("usb".to_string()),
                differential_role: Some(DifferentialRole::Negative),
                max_pair_spacing: Some(crate::scalar::scalar("0.5")),
                ..NetClassConfig::default()
            },
        ];
        let board = board_with_features(vec![
            feature("F.Cu", "USB_D+", CopperKind::Segment, [0.0, 0.0], 0.1, 0.1),
            feature("F.Cu", "USB_D-", CopperKind::Segment, [2.0, 0.0], 0.1, 0.1),
        ]);

        let messages = net_constraint_readiness(&classes, None, &[board], &[])
            .into_iter()
            .filter_map(|violation| violation.message)
            .collect::<Vec<_>>();

        assert_eq!(
            messages
                .iter()
                .filter(|message| message.contains("nearest side spacing"))
                .count(),
            1
        );
    }

    #[test]
    fn net_constraint_readiness_allows_declared_balanced_differential_pair() {
        let classes = vec![
            NetClassConfig {
                name: "usb-p".to_string(),
                nets: vec!["USB_D+".to_string()],
                differential_pair: Some("usb".to_string()),
                differential_role: Some(DifferentialRole::Positive),
                min_pair_spacing: Some(crate::scalar::scalar("0.2")),
                max_pair_spacing: Some(crate::scalar::scalar("0.5")),
                max_via_count: Some(1),
                ..NetClassConfig::default()
            },
            NetClassConfig {
                name: "usb-n".to_string(),
                nets: vec!["USB_D-".to_string()],
                differential_pair: Some("usb".to_string()),
                differential_role: Some(DifferentialRole::Negative),
                min_pair_spacing: Some(crate::scalar::scalar("0.2")),
                max_pair_spacing: Some(crate::scalar::scalar("0.5")),
                max_via_count: Some(1),
                ..NetClassConfig::default()
            },
        ];
        let board = board_with_features(vec![
            feature("F.Cu", "USB_D+", CopperKind::Segment, [0.0, 0.0], 0.2, 0.2),
            feature("F.Cu", "USB_D-", CopperKind::Segment, [0.5, 0.0], 0.2, 0.2),
            feature("F.Cu", "USB_D+", CopperKind::Via, [1.0, 0.0], 0.2, 0.2),
            feature("F.Cu", "USB_D-", CopperKind::Via, [1.0, 0.5], 0.2, 0.2),
        ]);

        assert!(net_constraint_readiness(&classes, None, &[board], &[]).is_empty());
    }

    #[test]
    fn net_constraint_readiness_checks_preferred_via_geometry_and_names_style() {
        let classes = vec![NetClassConfig {
            name: "power".to_string(),
            nets: vec!["VBUS".to_string()],
            preferred_via_land_diameter: Some(crate::scalar::scalar("0.8")),
            preferred_via_drill_diameter: Some(crate::scalar::scalar("0.3")),
            preferred_via_style: Some("power-through".to_string()),
            ..NetClassConfig::default()
        }];
        let mut board = board_with_features(vec![feature(
            "F.Cu",
            "VBUS",
            CopperKind::Via,
            [1.0, 2.0],
            0.6,
            0.6,
        )]);
        board.drills.push(DrillFeature {
            location: [
                crate::geometry::exact_real(1.0),
                crate::geometry::exact_real(2.0),
            ],
            diameter: crate::scalar::scalar("0.2"),
            net: Some("VBUS".to_string()),
            plated: true,
        });

        let violations = net_constraint_readiness(&classes, None, &[board], &[]);
        assert_eq!(
            violations
                .iter()
                .filter(|violation| violation.check == "net-via-policy-readiness")
                .count(),
            2
        );
        assert!(violations.iter().all(|violation| {
            violation
                .message
                .as_deref()
                .is_some_and(|message| message.contains("power-through"))
        }));
    }

    fn board_with_features(copper: Vec<CopperFeature>) -> BoardModel {
        BoardModel {
            source: "board.kicad_pcb".to_string(),
            copper,
            drills: Vec::new(),
            board_outline: None,
            panel_features: None,
        }
    }

    fn complete_stackup(
        surface_finish: Option<SurfaceFinish>,
        impedance_controlled: Option<bool>,
    ) -> StackupConfig {
        StackupConfig {
            copper_layer_count: Some(2),
            finished_thickness: Some(crate::scalar::scalar("1.6")),
            impedance_controlled,
            material_family: Some("FR-4".to_string()),
            material_dielectric_constant: Some(crate::scalar::scalar("4.2")),
            material_loss_tangent: Some(crate::scalar::scalar("0.018")),
            material_tg_c: Some(crate::scalar::scalar("150.0")),
            surface_finish,
            soldermask_process: Some("LPI".to_string()),
            soldermask_color: Some("green".to_string()),
            target_ipc_class: Some("IPC Class 2".to_string()),
            fabricator_profile: Some("prototype-fab".to_string()),
            fabrication_capability: FabricationCapabilityConfig::default(),
            layers: vec![
                StackupLayerConfig {
                    name: "F.Cu".to_string(),
                    kind: StackupLayerKind::Copper,
                    copper_weight_oz: Some(crate::scalar::scalar("1.0")),
                    dielectric_thickness: None,
                },
                StackupLayerConfig {
                    name: "Core".to_string(),
                    kind: StackupLayerKind::Core,
                    copper_weight_oz: None,
                    dielectric_thickness: Some(crate::scalar::scalar("1.5")),
                },
                StackupLayerConfig {
                    name: "B.Cu".to_string(),
                    kind: StackupLayerKind::Copper,
                    copper_weight_oz: Some(crate::scalar::scalar("1.0")),
                    dielectric_thickness: None,
                },
            ],
        }
    }

    fn controlled_microstrip_stackup() -> StackupConfig {
        StackupConfig {
            copper_layer_count: Some(2),
            finished_thickness: Some(crate::scalar::scalar("0.36")),
            impedance_controlled: Some(true),
            material_family: Some("FR-4".to_string()),
            material_dielectric_constant: Some(crate::scalar::scalar("4.2")),
            material_loss_tangent: Some(crate::scalar::scalar("0.018")),
            material_tg_c: Some(crate::scalar::scalar("150.0")),
            surface_finish: Some(SurfaceFinish::Enig),
            soldermask_process: Some("LPI".to_string()),
            soldermask_color: Some("green".to_string()),
            target_ipc_class: Some("IPC Class 2".to_string()),
            fabricator_profile: Some("prototype-fab".to_string()),
            fabrication_capability: FabricationCapabilityConfig::default(),
            layers: vec![
                StackupLayerConfig {
                    name: "F.Cu".to_string(),
                    kind: StackupLayerKind::Copper,
                    copper_weight_oz: Some(crate::scalar::scalar("1.0")),
                    dielectric_thickness: None,
                },
                StackupLayerConfig {
                    name: "Prepreg".to_string(),
                    kind: StackupLayerKind::Prepreg,
                    copper_weight_oz: None,
                    dielectric_thickness: Some(crate::scalar::scalar("0.18")),
                },
                StackupLayerConfig {
                    name: "B.Cu".to_string(),
                    kind: StackupLayerKind::Copper,
                    copper_weight_oz: Some(crate::scalar::scalar("1.0")),
                    dielectric_thickness: None,
                },
            ],
        }
    }

    fn controlled_centered_stripline_stackup() -> StackupConfig {
        StackupConfig {
            copper_layer_count: Some(3),
            finished_thickness: Some(crate::scalar::scalar("0.54")),
            impedance_controlled: Some(true),
            material_family: Some("FR-4".to_string()),
            material_dielectric_constant: Some(crate::scalar::scalar("4.2")),
            material_loss_tangent: Some(crate::scalar::scalar("0.018")),
            material_tg_c: Some(crate::scalar::scalar("150.0")),
            surface_finish: Some(SurfaceFinish::Enig),
            soldermask_process: Some("LPI".to_string()),
            soldermask_color: Some("green".to_string()),
            target_ipc_class: Some("IPC Class 2".to_string()),
            fabricator_profile: Some("prototype-fab".to_string()),
            fabrication_capability: FabricationCapabilityConfig::default(),
            layers: vec![
                StackupLayerConfig {
                    name: "F.Cu".to_string(),
                    kind: StackupLayerKind::Copper,
                    copper_weight_oz: Some(crate::scalar::scalar("1.0")),
                    dielectric_thickness: None,
                },
                StackupLayerConfig {
                    name: "Prepreg".to_string(),
                    kind: StackupLayerKind::Prepreg,
                    copper_weight_oz: None,
                    dielectric_thickness: Some(crate::scalar::scalar("0.18")),
                },
                StackupLayerConfig {
                    name: "In1.Cu".to_string(),
                    kind: StackupLayerKind::Copper,
                    copper_weight_oz: Some(crate::scalar::scalar("1.0")),
                    dielectric_thickness: None,
                },
                StackupLayerConfig {
                    name: "Core".to_string(),
                    kind: StackupLayerKind::Core,
                    copper_weight_oz: None,
                    dielectric_thickness: Some(crate::scalar::scalar("0.18")),
                },
                StackupLayerConfig {
                    name: "B.Cu".to_string(),
                    kind: StackupLayerKind::Copper,
                    copper_weight_oz: Some(crate::scalar::scalar("1.0")),
                    dielectric_thickness: None,
                },
            ],
        }
    }

    fn feature(
        layer: &str,
        net: &str,
        kind: CopperKind,
        center: [f64; 2],
        width: f64,
        height: f64,
    ) -> CopperFeature {
        let polygon = if width == height {
            circle_polygon(center, width / 2.0, 24)
        } else {
            rect_polygon(center, [width, height], 0.0)
        };
        CopperFeature {
            layer: layer.to_string(),
            net: Some(net.to_string()),
            kind,
            region: polygons_to_profile(vec![polygon], None),
            location: [
                crate::geometry::exact_real(center[0]),
                crate::geometry::exact_real(center[1]),
            ],
        }
    }
}

#[cfg(test)]
mod native_width_regressions {
    use super::minimum_bounding_dimension;
    use crate::{PREDICATE_POLICY, PcbRegion, Scalar};
    #[test]
    fn native_width_does_not_require_parser_cached_bounds() {
        let width = crate::scalar::scalar("0.08");
        let region = PcbRegion::new(
            csgrs::curve::rectangle(width.clone(), Scalar::one()).expect("valid planar fixture"),
            None,
        );
        assert!(region.exact_bounds().is_none());
        let bounds = super::native_region_bounds_scalar(&region)
            .expect("native geometry remains indexable without parser-attached bounds");
        assert_eq!(&bounds.max_x - &bounds.min_x, width);
        assert_eq!(minimum_bounding_dimension(&region).unwrap(), Some(width));
        let failed = region.with_exact_construction_error("retained width source failure");
        assert!(minimum_bounding_dimension(&failed).is_err());
        assert_eq!(
            minimum_bounding_dimension(&crate::geometry::empty_profile(None)).unwrap(),
            None
        );
    }
    #[test]
    fn denied_native_width_cannot_become_zero_or_a_cached_value() {
        let region = PcbRegion::new(
            csgrs::curve::rectangle(Scalar::one(), Scalar::from(2)).expect("valid planar fixture"),
            None,
        );
        assert_eq!(
            minimum_bounding_dimension(&region).unwrap(),
            Some(Scalar::one())
        );
        let denied = hypercurve::evaluate_with_budget(
            PREDICATE_POLICY,
            hypercurve::WorkBudget::ZERO,
            || {
                let polynomial = hypercurve::BezierParameterPolynomial::try_new_power_basis(
                    [-36, 1368, -11034, 31728, -38280, 16620]
                        .into_iter()
                        .map(Scalar::from)
                        .collect(),
                )
                .unwrap();
                assert!(polynomial.isolate_unit_interval_roots().is_err());
                assert!(minimum_bounding_dimension(&region).is_err());
            },
        );
        assert!(denied.result.is_err());
        assert_eq!(
            minimum_bounding_dimension(&region).unwrap(),
            Some(Scalar::one())
        );
    }
}
