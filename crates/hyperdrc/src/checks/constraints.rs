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

use std::collections::{BTreeMap, BTreeSet};

use super::distance::polygon_boundary_distance_scalar;
use super::impedance::{ImpedanceModel, estimate_single_ended_impedance};
use super::net_class::resolve_net_classes;
use super::net_scope::{matching_class_indexes_for_feature, net_class_region_diagnostics};
use crate::PcbSketchExt;
use crate::Scalar;
use crate::constraint_policy::{
    DifferentialRole, FabricationCapabilityConfig, NetClassConfig, StackupConfig, StackupLayerKind,
    SurfaceFinish,
};
use crate::kicad::{BoardModel, CopperFeature, CopperKind};
use crate::report::{Severity, Violation};
use crate::scalar::scalar;
use csgrs::csg::CSG;

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
    label: &'static str,
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
    ) {
        if finished_thickness < minimum {
            violations.push(stackup_metadata_violation(&format!(
                "stackup finished_thickness {finished_thickness:#.6} is below fabricator profile {} minimum {minimum:#.6}",
                capability.label
            )));
        }
    }
    if let (Some(finished_thickness), Some(maximum)) = (
        stackup.finished_thickness.as_ref(),
        capability.max_finished_thickness.as_ref(),
    ) {
        if finished_thickness > maximum {
            violations.push(stackup_metadata_violation(&format!(
                "stackup finished_thickness {finished_thickness:#.6} is above fabricator profile {} maximum {maximum:#.6}",
                capability.label
            )));
        }
    }
    if let (Some(finished_thickness), Some(preferred_minimum)) = (
        stackup.finished_thickness.as_ref(),
        capability.preferred_min_finished_thickness.as_ref(),
    ) {
        if finished_thickness < preferred_minimum
            && capability
                .min_finished_thickness
                .as_ref()
                .is_none_or(|minimum| finished_thickness >= minimum)
        {
            violations.push(stackup_metadata_violation(&format!(
                "stackup finished_thickness {finished_thickness:#.6} is below fabricator profile {} preferred minimum {preferred_minimum:#.6}; review cost-escalation or special-process requirements",
                capability.label
            )));
        }
    }
    if let (Some(finished_thickness), Some(preferred_maximum)) = (
        stackup.finished_thickness.as_ref(),
        capability.preferred_max_finished_thickness.as_ref(),
    ) {
        if finished_thickness > preferred_maximum
            && capability
                .max_finished_thickness
                .as_ref()
                .is_none_or(|maximum| finished_thickness <= maximum)
        {
            violations.push(stackup_metadata_violation(&format!(
                "stackup finished_thickness {finished_thickness:#.6} is above fabricator profile {} preferred maximum {preferred_maximum:#.6}; review cost-escalation or special-process requirements",
                capability.label
            )));
        }
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
    if let Some(preferred_max_copper_layers) = capability.preferred_max_copper_layers {
        if configured_count > preferred_max_copper_layers
            && capability
                .max_copper_layers
                .is_none_or(|maximum| configured_count <= maximum)
        {
            violations.push(stackup_metadata_violation(&format!(
                "fabricator profile {} preferred service supports up to {preferred_max_copper_layers} copper layer(s), but stackup lists {configured_count}; review cost-escalation or advanced-service selection",
                capability.label
            )));
        }
    }
    if let Some(cost_escalation_copper_layers) = capability.cost_escalation_copper_layers {
        if configured_count > cost_escalation_copper_layers
            && capability
                .max_copper_layers
                .is_none_or(|maximum| configured_count <= maximum)
        {
            violations.push(stackup_metadata_violation(&format!(
                "fabricator profile {} cost-escalation threshold is {cost_escalation_copper_layers} copper layer(s), but stackup lists {configured_count}; review quote class and fabrication lead time",
                capability.label
            )));
        }
    }

    for layer in configured_copper_layers {
        if let (Some(weight), Some(minimum)) = (
            layer.copper_weight_oz.as_ref(),
            capability.min_copper_weight_oz.as_ref(),
        ) {
            if weight < minimum {
                violations.push(stackup_metadata_violation(&format!(
                    "stackup copper layer {} has copper_weight_oz {weight:#.6} below fabricator profile {} minimum {minimum:#.6}",
                    layer.name, capability.label
                )));
            }
        }
        if let (Some(weight), Some(maximum)) = (
            layer.copper_weight_oz.as_ref(),
            capability.max_copper_weight_oz.as_ref(),
        ) {
            if weight > maximum {
                violations.push(stackup_metadata_violation(&format!(
                    "stackup copper layer {} has copper_weight_oz {weight:#.6} above fabricator profile {} maximum {maximum:#.6}",
                    layer.name, capability.label
                )));
            }
        }
        if let (Some(weight), Some(preferred_minimum)) = (
            layer.copper_weight_oz.as_ref(),
            capability.preferred_min_copper_weight_oz.as_ref(),
        ) {
            if weight < preferred_minimum
                && capability
                    .min_copper_weight_oz
                    .as_ref()
                    .is_none_or(|minimum| weight >= minimum)
            {
                violations.push(stackup_metadata_violation(&format!(
                    "stackup copper layer {} has copper_weight_oz {weight:#.6} below fabricator profile {} preferred minimum {preferred_minimum:#.6}; review cost-escalation or special-process requirements",
                    layer.name, capability.label
                )));
            }
        }
        if let (Some(weight), Some(preferred_maximum)) = (
            layer.copper_weight_oz.as_ref(),
            capability.preferred_max_copper_weight_oz.as_ref(),
        ) {
            if weight > preferred_maximum
                && capability
                    .max_copper_weight_oz
                    .as_ref()
                    .is_none_or(|maximum| weight <= maximum)
            {
                violations.push(stackup_metadata_violation(&format!(
                    "stackup copper layer {} has copper_weight_oz {weight:#.6} above fabricator profile {} preferred maximum {preferred_maximum:#.6}; review cost-escalation or special-process requirements",
                    layer.name, capability.label
                )));
            }
        }
        if let (Some(weight), Some(cost_threshold)) = (
            layer.copper_weight_oz.as_ref(),
            capability.cost_escalation_copper_weight_oz.as_ref(),
        ) {
            if weight > cost_threshold
                && capability
                    .max_copper_weight_oz
                    .as_ref()
                    .is_none_or(|maximum| weight <= maximum)
            {
                violations.push(stackup_metadata_violation(&format!(
                    "stackup copper layer {} has copper_weight_oz {weight:#.6} above fabricator profile {} cost-escalation threshold {cost_threshold:#.6}; review quote class and fabrication lead time",
                    layer.name, capability.label
                )));
            }
        }
    }

    if let Some(minimum) = capability.min_dielectric_thickness.as_ref() {
        for layer in stackup.layers.iter().filter(|layer| {
            matches!(
                layer.kind,
                StackupLayerKind::Dielectric | StackupLayerKind::Core | StackupLayerKind::Prepreg
            )
        }) {
            if let Some(thickness) = layer.dielectric_thickness.as_ref() {
                if thickness < minimum {
                    violations.push(stackup_metadata_violation(&format!(
                        "stackup dielectric layer {} has dielectric_thickness {thickness:#.6} below fabricator profile {} minimum {minimum:#.6}",
                        layer.name, capability.label
                    )));
                }
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
            if let Some(thickness) = layer.dielectric_thickness.as_ref() {
                if thickness < preferred_minimum
                    && capability
                        .min_dielectric_thickness
                        .as_ref()
                        .is_none_or(|minimum| thickness >= minimum)
                {
                    violations.push(stackup_metadata_violation(&format!(
                        "stackup dielectric layer {} has dielectric_thickness {thickness:#.6} below fabricator profile {} preferred minimum {preferred_minimum:#.6}; review cost-escalation or special-process requirements",
                        layer.name, capability.label
                    )));
                }
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
            if let Some(thickness) = layer.dielectric_thickness.as_ref() {
                if thickness < cost_threshold
                    && capability
                        .min_dielectric_thickness
                        .as_ref()
                        .is_none_or(|minimum| thickness >= minimum)
                {
                    violations.push(stackup_metadata_violation(&format!(
                        "stackup dielectric layer {} has dielectric_thickness {thickness:#.6} below fabricator profile {} cost-escalation threshold {cost_threshold:#.6}; review quote class and fabrication lead time",
                        layer.name, capability.label
                    )));
                }
            }
        }
    }

    // IPC-2221B treats dielectric constant and loss tangent as stackup inputs
    // for electrical behavior; these checks only verify explicit policy ranges
    // before handoff, leaving field solving to dedicated impedance tools.
    if let (Some(value), Some(minimum)) = (
        stackup.material_dielectric_constant.as_ref(),
        capability.min_dielectric_constant.as_ref(),
    ) {
        if value < minimum {
            violations.push(stackup_metadata_violation(&format!(
                "stackup material_dielectric_constant {value:#.6} is below fabricator profile {} minimum {minimum:#.6}",
                capability.label
            )));
        }
    }
    if let (Some(value), Some(maximum)) = (
        stackup.material_dielectric_constant.as_ref(),
        capability.max_dielectric_constant.as_ref(),
    ) {
        if value > maximum {
            violations.push(stackup_metadata_violation(&format!(
                "stackup material_dielectric_constant {value:#.6} is above fabricator profile {} maximum {maximum:#.6}",
                capability.label
            )));
        }
    }
    if let (Some(value), Some(maximum)) = (
        stackup.material_loss_tangent.as_ref(),
        capability.max_loss_tangent.as_ref(),
    ) {
        if value > maximum {
            violations.push(stackup_metadata_violation(&format!(
                "stackup material_loss_tangent {value:#.6} is above fabricator profile {} maximum {maximum:#.6}",
                capability.label
            )));
        }
    }
    if let (Some(value), Some(minimum)) =
        (stackup.material_tg_c.as_ref(), capability.min_tg_c.as_ref())
    {
        if value < minimum {
            violations.push(stackup_metadata_violation(&format!(
                "stackup material_tg_c {value:#.6} is below fabricator profile {} minimum {minimum:#.6}",
                capability.label
            )));
        }
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
            label: "custom",
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
                label: $label,
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
                $(
                    min_dielectric_constant: Some(exact!($min_dk)),
                    max_dielectric_constant: Some(exact!($max_dk)),
                    max_loss_tangent: Some(exact!($max_df)),
                    min_tg_c: Some(exact!($min_tg)),
                )?
                ..FabricationCapability::default()
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

fn net_width_constraints(
    net_classes: &[NetClassConfig],
    features: &[&CopperFeature],
) -> Vec<Violation> {
    let mut violations = Vec::new();
    for feature in features {
        let Some(net) = feature.net.as_deref() else {
            continue;
        };
        for class_index in matching_class_indexes_for_feature(net_classes, feature) {
            let class = &net_classes[class_index];
            let Some(min_width) = class.min_width.as_ref() else {
                if let Some(min_current_width) = class.min_current_width.as_ref() {
                    let width = minimum_bounding_dimension(&feature.sketch);
                    if width > Scalar::zero() && &width < min_current_width {
                        violations.push(Violation::new(
                            "net-constraint-readiness",
                            Severity::Warning,
                            vec![feature.layer.clone()],
                            None,
                            Vec::new(),
                            vec![feature.location_f64_compatibility_required()],
                            Some(format!(
                                "net {net} in class {} has parsed {:?} width {width:#.6}, below configured current-carrying minimum {min_current_width:#.6}",
                                class_name(class),
                                feature.kind
                            )),
                        ));
                    }
                }
                continue;
            };
            let width = minimum_bounding_dimension(&feature.sketch);
            if width > Scalar::zero() && &width < min_width {
                violations.push(Violation::new(
                    "net-constraint-readiness",
                    Severity::Error,
                    vec![feature.layer.clone()],
                    None,
                    Vec::new(),
                    vec![feature.location_f64_compatibility_required()],
                    Some(format!(
                        "net {net} in class {} has parsed {:?} width {width:#.6}, below configured minimum {min_width:#.6}",
                        class_name(class),
                        feature.kind
                    )),
                ));
            }
            if let Some(min_current_width) = class.min_current_width.as_ref()
                && width > Scalar::zero()
                && &width < min_current_width
            {
                violations.push(Violation::new(
                    "net-constraint-readiness",
                    Severity::Warning,
                    vec![feature.layer.clone()],
                    None,
                    Vec::new(),
                    vec![feature.location_f64_compatibility_required()],
                    Some(format!(
                        "net {net} in class {} has parsed {:?} width {width:#.6}, below configured current-carrying minimum {min_current_width:#.6}",
                        class_name(class),
                        feature.kind
                    )),
                ));
            }
        }
    }
    violations
}

fn net_layer_and_via_constraints(
    net_classes: &[NetClassConfig],
    features: &[&CopperFeature],
) -> Vec<Violation> {
    let mut by_class_net = BTreeMap::<(usize, String), NetUse>::new();
    for feature in features {
        let Some(net) = feature.net.as_deref() else {
            continue;
        };
        for class_index in matching_class_indexes_for_feature(net_classes, feature) {
            let entry = by_class_net
                .entry((class_index, net.to_string()))
                .or_default();
            entry.layers.insert(feature.layer.clone());
            entry
                .locations
                .push(feature.location_f64_compatibility_required());
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
                usage.locations.clone(),
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
                usage.locations.clone(),
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
                usage.locations.clone(),
                Some(format!(
                    "net {net} in class {} has {} parsed via(s), above configured maximum {max_via_count}",
                    class_name(class),
                    usage.via_count
                )),
            ));
        }
    }
    violations
}

fn net_clearance_constraints(
    net_classes: &[NetClassConfig],
    features: &[&CopperFeature],
) -> Vec<Violation> {
    let Some(maximum_clearance) = maximum_configured_clearance(net_classes) else {
        return Vec::new();
    };
    let mut bounded_features = features
        .iter()
        .copied()
        .filter_map(|feature| {
            native_sketch_bounds_scalar(&feature.sketch).map(|bounds| (feature, bounds))
        })
        .collect::<Vec<_>>();
    bounded_features.sort_by(
        |(left_feature, left_bounds), (right_feature, right_bounds)| {
            left_feature.layer.cmp(&right_feature.layer).then_with(|| {
                scalar_cmp(&left_bounds.min_x, &right_bounds.min_x)
                    .unwrap_or(std::cmp::Ordering::Equal)
            })
        },
    );
    let mut exact_pair_count = 0_usize;
    let mut violations = Vec::new();
    for (left_index, (left, left_bounds)) in bounded_features.iter().enumerate() {
        let Some(left_net) = &left.net else {
            continue;
        };
        let maximum_x = &left_bounds.max_x + &maximum_clearance;
        for (right, right_bounds) in bounded_features.iter().skip(left_index + 1) {
            if left.layer != right.layer {
                break;
            }
            if scalar_gt(&right_bounds.min_x, &maximum_x) {
                break;
            }
            if !exact_bounds_may_be_within(left_bounds, right_bounds, &maximum_clearance) {
                continue;
            }
            let Some(right_net) = &right.net else {
                continue;
            };
            if left_net == right_net {
                continue;
            }

            let Some((class_name, min_clearance)) = required_clearance(net_classes, left, right)
            else {
                continue;
            };
            exact_pair_count += 1;
            let Some(gap) = polygon_boundary_distance_scalar(
                &left.sketch.to_multipolygon(),
                &right.sketch.to_multipolygon(),
            ) else {
                continue;
            };
            if gap < min_clearance {
                violations.push(Violation::new(
                    "net-constraint-readiness",
                    Severity::Error,
                    vec![left.layer.clone()],
                    None,
                    Vec::new(),
                    vec![
                        left.location_f64_compatibility_required(),
                        right.location_f64_compatibility_required(),
                    ],
                    Some(format!(
                        "net {left_net} to {right_net} spacing {gap:#.6} is below configured clearance {min_clearance:#.6} from class {class_name}"
                    )),
                ));
            }
        }
    }
    log::trace!(
        "net-constraint clearance readiness: features={} maximum_clearance={maximum_clearance:#.6} exact_pairs={} violations={}",
        features.len(),
        exact_pair_count,
        violations.len()
    );
    violations
}

fn net_reference_plane_constraints(
    net_classes: &[NetClassConfig],
    features: &[&CopperFeature],
) -> Vec<Violation> {
    let reference_layers = features
        .iter()
        .filter(|feature| feature.kind == CopperKind::Zone)
        .filter(|feature| feature.net.as_deref().is_some_and(is_reference_net))
        .map(|feature| feature.layer.clone())
        .collect::<BTreeSet<_>>();
    if reference_layers.is_empty() {
        return net_plane_intent_violations(
            net_classes,
            features,
            "no parsed reference-plane copper was found",
        );
    }

    let mut by_class_net = BTreeMap::<(usize, String), NetUse>::new();
    for feature in features {
        let Some(net) = feature.net.as_deref() else {
            continue;
        };
        if is_reference_net(net) {
            continue;
        }
        for class_index in matching_class_indexes_for_feature(net_classes, feature) {
            let entry = by_class_net
                .entry((class_index, net.to_string()))
                .or_default();
            entry.layers.insert(feature.layer.clone());
            entry
                .locations
                .push(feature.location_f64_compatibility_required());
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
                usage.locations.clone(),
                Some(format!(
                    "net {net} in class {} requires reference-plane review, but no parsed reference-plane zone is present on the same selected layer(s)",
                    class_name(class)
                )),
            ));
        }
    }
    violations
}

fn net_plane_intent_violations(
    net_classes: &[NetClassConfig],
    features: &[&CopperFeature],
    reason: &str,
) -> Vec<Violation> {
    let mut by_class_net = BTreeMap::<(usize, String), NetUse>::new();
    for feature in features {
        let Some(net) = feature.net.as_deref() else {
            continue;
        };
        for class_index in matching_class_indexes_for_feature(net_classes, feature) {
            let entry = by_class_net
                .entry((class_index, net.to_string()))
                .or_default();
            entry.layers.insert(feature.layer.clone());
            entry
                .locations
                .push(feature.location_f64_compatibility_required());
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
                usage.locations.clone(),
                Some(format!(
                    "net {net} in class {} requires reference-plane review, but {reason}",
                    class_name(class)
                )),
            ));
        }
    }
    violations
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
    let mut by_class_net = BTreeMap::<(usize, String), NetUse>::new();
    for feature in features {
        let Some(net) = feature.net.as_deref() else {
            continue;
        };
        for class_index in matching_class_indexes_for_feature(net_classes, feature) {
            let entry = by_class_net
                .entry((class_index, net.to_string()))
                .or_default();
            entry.layers.insert(feature.layer.clone());
            entry
                .locations
                .push(feature.location_f64_compatibility_required());
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
                usage.locations.clone(),
                Some(format!(
                    "net {net} in class {} requires impedance-control review, but {reason}",
                    class_name(class)
                )),
            ));
        }
    }
    violations
}

fn net_impedance_target_constraints(
    net_classes: &[NetClassConfig],
    stackup: Option<&StackupConfig>,
    features: &[&CopperFeature],
) -> Vec<Violation> {
    let mut by_class_net = BTreeMap::<(usize, String), NetUse>::new();
    let mut features_by_class_net = BTreeMap::<(usize, String), Vec<&CopperFeature>>::new();
    for feature in features {
        let Some(net) = feature.net.as_deref() else {
            continue;
        };
        for class_index in matching_class_indexes_for_feature(net_classes, feature) {
            let key = (class_index, net.to_string());
            let entry = by_class_net.entry(key.clone()).or_default();
            entry.layers.insert(feature.layer.clone());
            entry
                .locations
                .push(feature.location_f64_compatibility_required());
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
            Some(target) if target > &Scalar::zero() => Some(target.clone()),
            Some(target) => {
                violations.push(Violation::new(
                    "net-constraint-readiness",
                    Severity::Warning,
                    usage.layers.iter().cloned().collect(),
                    None,
                    Vec::new(),
                    usage.locations.clone(),
                    Some(format!(
                        "net {net} in class {} has invalid target_impedance_ohms {target:#.6}",
                        class_name(class)
                    )),
                ));
                None
            }
            None => {
                violations.push(Violation::new(
                    "net-constraint-readiness",
                    Severity::Warning,
                    usage.layers.iter().cloned().collect(),
                    None,
                    Vec::new(),
                    usage.locations.clone(),
                    Some(format!(
                        "net {net} in class {} requires impedance-control review, but target_impedance_ohms is missing",
                        class_name(class)
                    )),
                ));
                None
            }
        };

        let impedance_tolerance_ohms = match class.impedance_tolerance_ohms.as_ref() {
            Some(tolerance) if tolerance > &Scalar::zero() => Some(tolerance.clone()),
            Some(tolerance) => {
                violations.push(Violation::new(
                    "net-constraint-readiness",
                    Severity::Warning,
                    usage.layers.iter().cloned().collect(),
                    None,
                    Vec::new(),
                    usage.locations.clone(),
                    Some(format!(
                        "net {net} in class {} has invalid impedance_tolerance_ohms {tolerance:#.6}",
                        class_name(class)
                    )),
                ));
                None
            }
            None => {
                violations.push(Violation::new(
                    "net-constraint-readiness",
                    Severity::Warning,
                    usage.layers.iter().cloned().collect(),
                    None,
                    Vec::new(),
                    usage.locations.clone(),
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
            // Differential impedance targets need coupled-line geometry and
            // spacing. The first-pass Hammerstad-Jensen estimate in this
            // module is single-ended only, so keep differential classes at
            // metadata-readiness until a coupled solver lands.
            continue;
        }

        let Some(stackup) = stackup.filter(|stackup| stackup.impedance_controlled == Some(true))
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
            let trace_width = minimum_bounding_dimension(&feature.sketch);
            let Some(estimate) =
                estimate_single_ended_impedance(stackup, &feature.layer, trace_width)
            else {
                unsupported_segments += 1;
                continue;
            };
            estimated_segments += 1;
            let delta = (&estimate.impedance_ohms - &target_impedance_ohms).abs();
            if delta > impedance_tolerance_ohms {
                violations.push(Violation::new(
                    "net-constraint-readiness",
                    Severity::Warning,
                    vec![feature.layer.clone()],
                    None,
                    Vec::new(),
                    vec![feature.location_f64_compatibility_required()],
                    Some(format!(
                        "net {net} in class {} has estimated {} impedance {:#.3} ohm from parsed width {:#.6}, dielectric height/spacing {:#.6}, and Dk {:#.3}, outside target {:#.3} +/- {:#.3} ohm",
                        class_name(class),
                        impedance_model_label(estimate.model),
                        estimate.impedance_ohms,
                        estimate.trace_width,
                        estimate.dielectric_height,
                        estimate.dielectric_constant,
                        target_impedance_ohms,
                        impedance_tolerance_ohms
                    )),
                ));
            }
        }
    }

    log::trace!(
        "net-constraint impedance target readiness: features={} single_ended_candidates={} estimated_segments={} unsupported_segments={} differential_target_classes={} violations={}",
        features.len(),
        single_ended_candidates,
        estimated_segments,
        unsupported_segments,
        differential_target_classes,
        violations.len()
    );
    violations
}

fn impedance_model_label(model: ImpedanceModel) -> &'static str {
    match model {
        ImpedanceModel::OuterMicrostrip => "outer microstrip",
        ImpedanceModel::CenteredStripline => "centered stripline",
    }
}

fn net_differential_pair_constraints(
    net_classes: &[NetClassConfig],
    features: &[&CopperFeature],
) -> Vec<Violation> {
    let mut pairs = BTreeMap::<String, DifferentialPairUse>::new();
    for feature in features {
        let Some(net) = &feature.net else {
            continue;
        };
        for class_index in matching_class_indexes_for_feature(net_classes, feature) {
            let class = &net_classes[class_index];
            let (Some(pair), Some(role)) = (&class.differential_pair, class.differential_role)
            else {
                continue;
            };
            let side = pairs.entry(pair.clone()).or_default().side_mut(role);
            side.net_names.insert(net.clone());
            side.layers.insert(feature.layer.clone());
            side.locations
                .push(feature.location_f64_compatibility_required());
            side.features.push(*feature);
            side.min_pair_spacing =
                option_max(side.min_pair_spacing.take(), class.min_pair_spacing.clone());
            side.max_pair_spacing =
                option_min(side.max_pair_spacing.take(), class.max_pair_spacing.clone());
            side.max_pair_skew = option_min(side.max_pair_skew.take(), class.max_pair_skew.clone());
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
                present.locations.clone(),
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
                pair_use.locations(),
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
    violations
}

fn differential_pair_spacing_violations(
    pair: &str,
    pair_use: &DifferentialPairUse<'_>,
    min_spacing: Option<Scalar>,
    max_spacing: Option<Scalar>,
) -> Vec<Violation> {
    let query_spacing = [min_spacing.as_ref(), max_spacing.as_ref()]
        .into_iter()
        .flatten()
        .filter(|spacing| *spacing >= &Scalar::zero())
        .max_by(|left, right| left.partial_cmp(right).unwrap_or(std::cmp::Ordering::Equal))
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
        return Vec::new();
    }

    let mut negative_by_layer = BTreeMap::<&str, Vec<(&CopperFeature, ScalarBounds2)>>::new();
    for feature in &negative_features {
        if let Some(bounds) = native_sketch_bounds_scalar(&feature.sketch) {
            negative_by_layer
                .entry(feature.layer.as_str())
                .or_default()
                .push((feature, bounds));
        }
    }
    for features in negative_by_layer.values_mut() {
        features.sort_by(|(_, left), (_, right)| {
            scalar_cmp(&left.min_x, &right.min_x).unwrap_or(std::cmp::Ordering::Equal)
        });
    }
    let maximum_negative_width = negative_by_layer
        .values()
        .flatten()
        .map(|(_, bounds)| &bounds.max_x - &bounds.min_x)
        .fold(Scalar::zero(), |maximum, width| {
            if scalar_gt(&width, &maximum) {
                width
            } else {
                maximum
            }
        });

    let mut exact_pair_count = 0_usize;
    let mut closest_pair: Option<DifferentialGap<'_>> = None;
    let mut has_pair_within_max = max_spacing.is_none();
    let mut violations = Vec::new();

    for positive in &positive_features {
        let Some(negative_candidates) = negative_by_layer.get(positive.layer.as_str()) else {
            continue;
        };
        let Some(positive_bounds) = native_sketch_bounds_scalar(&positive.sketch) else {
            continue;
        };
        let lower_x = &positive_bounds.min_x - &query_spacing - &maximum_negative_width;
        let upper_x = &positive_bounds.max_x + &query_spacing;
        let start =
            negative_candidates.partition_point(|(_, bounds)| scalar_lt(&bounds.min_x, &lower_x));
        let end =
            negative_candidates.partition_point(|(_, bounds)| scalar_le(&bounds.min_x, &upper_x));
        for (negative, negative_bounds) in &negative_candidates[start..end] {
            if !exact_bounds_may_be_within(&positive_bounds, negative_bounds, &query_spacing) {
                continue;
            }
            let Some(gap) = polygon_boundary_distance_scalar(
                &positive.sketch.to_multipolygon(),
                &negative.sketch.to_multipolygon(),
            ) else {
                continue;
            };
            exact_pair_count += 1;
            let observed = DifferentialGap {
                positive,
                negative,
                gap: gap.clone(),
            };
            if closest_pair
                .as_ref()
                .is_none_or(|closest| gap < closest.gap)
            {
                closest_pair = Some(observed.clone());
            }

            if let Some(min_spacing) = min_spacing.as_ref()
                && &gap < min_spacing
            {
                violations.push(Violation::new(
                    "net-constraint-readiness",
                    Severity::Error,
                    vec![positive.layer.clone()],
                    None,
                    Vec::new(),
                    vec![
                        positive.location_f64_compatibility_required(),
                        negative.location_f64_compatibility_required(),
                    ],
                    Some(format!(
                        "differential pair {pair} side spacing {gap:#.6} is below configured minimum {min_spacing:#.6}"
                    )),
                ));
            }
            if let Some(max_spacing) = max_spacing.as_ref()
                && &gap <= max_spacing
            {
                has_pair_within_max = true;
            }
        }
    }

    if !has_pair_within_max {
        if closest_pair.is_none() {
            closest_pair = first_same_layer_pair_gap(&positive_features, &negative_features);
        }
        if let (Some(max_spacing), Some(closest)) = (max_spacing.as_ref(), closest_pair)
            && &closest.gap > max_spacing
        {
            violations.push(Violation::new(
                "net-constraint-readiness",
                Severity::Warning,
                vec![closest.positive.layer.clone()],
                None,
                Vec::new(),
                vec![
                    closest.positive.location_f64_compatibility_required(),
                    closest.negative.location_f64_compatibility_required(),
                ],
                Some(format!(
                    "differential pair {pair} nearest side spacing {:#.6} is above configured maximum {max_spacing:#.6}",
                    closest.gap
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
    violations
}

#[derive(Clone, Debug)]
struct DifferentialGap<'a> {
    positive: &'a CopperFeature,
    negative: &'a CopperFeature,
    gap: Scalar,
}

fn first_same_layer_pair_gap<'a>(
    positive_features: &[&'a CopperFeature],
    negative_features: &[&'a CopperFeature],
) -> Option<DifferentialGap<'a>> {
    for positive in positive_features {
        for negative in negative_features {
            if positive.layer != negative.layer {
                continue;
            }
            let gap = polygon_boundary_distance_scalar(
                &positive.sketch.to_multipolygon(),
                &negative.sketch.to_multipolygon(),
            );
            if let Some(gap) = gap {
                return Some(DifferentialGap {
                    positive,
                    negative,
                    gap,
                });
            }
        }
    }
    None
}

fn net_length_constraints(
    net_classes: &[NetClassConfig],
    features: &[&CopperFeature],
) -> Vec<Violation> {
    if !net_classes
        .iter()
        .any(|class| class.max_length.is_some() || class.max_pair_skew.is_some())
    {
        return Vec::new();
    }
    let mut by_class_net = BTreeMap::<(usize, String), NetUse>::new();
    let mut pairs = BTreeMap::<String, DifferentialPairUse>::new();

    for feature in features {
        let Some(net) = &feature.net else {
            continue;
        };
        let class_indexes = matching_class_indexes_for_feature(net_classes, feature);
        if !class_indexes.iter().any(|class_index| {
            let class = &net_classes[*class_index];
            class.max_length.is_some() || class.max_pair_skew.is_some()
        }) {
            continue;
        }
        let estimated_length = estimated_feature_length(feature);
        if estimated_length <= Scalar::zero() {
            continue;
        }

        for class_index in class_indexes {
            let class = &net_classes[class_index];
            let usage = by_class_net.entry((class_index, net.clone())).or_default();
            usage.layers.insert(feature.layer.clone());
            usage
                .locations
                .push(feature.location_f64_compatibility_required());
            usage.estimated_length += &estimated_length;
            usage.max_length = option_min(usage.max_length.take(), class.max_length.clone());

            let (Some(pair), Some(role)) = (&class.differential_pair, class.differential_role)
            else {
                continue;
            };
            let side = pairs.entry(pair.clone()).or_default().side_mut(role);
            side.net_names.insert(net.clone());
            side.layers.insert(feature.layer.clone());
            side.locations
                .push(feature.location_f64_compatibility_required());
            side.estimated_length += &estimated_length;
            side.max_pair_skew = option_min(side.max_pair_skew.take(), class.max_pair_skew.clone());
        }
    }

    let mut violations = Vec::new();
    for ((class_index, net), usage) in by_class_net {
        let class = &net_classes[class_index];
        if let Some(max_length) = usage.max_length.as_ref()
            && &usage.estimated_length > max_length
        {
            violations.push(Violation::new(
                "net-constraint-readiness",
                Severity::Warning,
                usage.layers.iter().cloned().collect(),
                None,
                Vec::new(),
                usage.locations.clone(),
                Some(format!(
                    "net {net} in class {} has approximate parsed copper length {:#.6}, above configured maximum {max_length:#.6}",
                    class_name(class),
                    usage.estimated_length
                )),
            ));
        }
    }

    for (pair, pair_use) in pairs {
        let Some(max_pair_skew) = pair_use.max_pair_skew() else {
            continue;
        };
        if pair_use.positive.estimated_length <= Scalar::zero()
            || pair_use.negative.estimated_length <= Scalar::zero()
        {
            continue;
        }
        let skew =
            (&pair_use.positive.estimated_length - &pair_use.negative.estimated_length).abs();
        if skew > max_pair_skew {
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
                pair_use.locations(),
                Some(format!(
                    "differential pair {pair} has approximate parsed length skew {skew:#.6}, above configured maximum {max_pair_skew:#.6}"
                )),
            ));
        }
    }

    violations
}

fn required_clearance<'a>(
    net_classes: &'a [NetClassConfig],
    left: &CopperFeature,
    right: &CopperFeature,
) -> Option<(&'a str, Scalar)> {
    matching_class_indexes_for_feature(net_classes, left)
        .into_iter()
        .chain(matching_class_indexes_for_feature(net_classes, right))
        .filter_map(|class_index| {
            let class = &net_classes[class_index];
            let clearance = [
                class.min_clearance.as_ref(),
                class.min_voltage_clearance.as_ref(),
            ]
            .into_iter()
            .flatten()
            .max_by(|left, right| left.partial_cmp(right).unwrap_or(std::cmp::Ordering::Equal))?;
            Some((class_name(class), clearance.clone()))
        })
        .max_by(|left, right| {
            left.1
                .partial_cmp(&right.1)
                .unwrap_or(std::cmp::Ordering::Equal)
        })
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
        .filter(|clearance| *clearance >= &Scalar::zero())
        .max_by(|left, right| left.partial_cmp(right).unwrap_or(std::cmp::Ordering::Equal))
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
    !value.is_some_and(|value| value > &Scalar::zero())
}

fn invalid_non_negative(value: Option<&Scalar>) -> bool {
    !value.is_some_and(|value| value >= &Scalar::zero())
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

fn minimum_bounding_dimension(sketch: &crate::PcbSketch) -> Scalar {
    let Some(bounds) = native_sketch_bounds_scalar(sketch) else {
        return Scalar::zero();
    };
    let width = &bounds.max_x - &bounds.min_x;
    let height = &bounds.max_y - &bounds.min_y;
    if width <= height { width } else { height }
}

#[derive(Clone, Debug)]
struct ScalarBounds2 {
    min_x: Scalar,
    min_y: Scalar,
    max_x: Scalar,
    max_y: Scalar,
}

/// Compute a conservative envelope directly over native exact topology.
///
/// Circular arcs contribute their full circle rather than running the tighter
/// certified sweep-extrema predicate. That can admit extra broad-phase
/// candidates, but it can never discard a real violation. It also keeps sparse
/// constraint indexing independent of finite polygon projection.
fn native_sketch_bounds_scalar(sketch: &crate::PcbSketch) -> Option<ScalarBounds2> {
    if let Some(bounds) = sketch.exact_bounds() {
        return Some(ScalarBounds2 {
            min_x: bounds[0].clone(),
            min_y: bounds[1].clone(),
            max_x: bounds[2].clone(),
            max_y: bounds[3].clone(),
        });
    }
    let mut bounds = None;
    let region = sketch.as_region();
    for contour in region
        .material_contours()
        .iter()
        .chain(region.hole_contours())
    {
        if include_native_segments(&mut bounds, contour.segments()).is_none() {
            return Some(certified_sketch_bounds_scalar(sketch));
        }
    }
    for wire in sketch.wires() {
        if include_native_segments(&mut bounds, wire.segments()).is_none() {
            return Some(certified_sketch_bounds_scalar(sketch));
        }
    }
    bounds
}

fn certified_sketch_bounds_scalar(sketch: &crate::PcbSketch) -> ScalarBounds2 {
    let bounds = sketch.bounding_box();
    ScalarBounds2 {
        min_x: bounds.mins.x.clone(),
        min_y: bounds.mins.y.clone(),
        max_x: bounds.maxs.x.clone(),
        max_y: bounds.maxs.y.clone(),
    }
}

fn include_native_segments(
    bounds: &mut Option<ScalarBounds2>,
    segments: &[hypercurve::Segment2],
) -> Option<()> {
    for segment in segments {
        include_exact_point(bounds, segment.start().x(), segment.start().y())?;
        include_exact_point(bounds, segment.end().x(), segment.end().y())?;
        if let hypercurve::Segment2::Arc(arc) = segment {
            let radius = arc.radius_squared_ref().clone().sqrt().ok()?;
            include_exact_point(bounds, &(arc.center().x() - &radius), arc.center().y())?;
            include_exact_point(bounds, &(arc.center().x() + &radius), arc.center().y())?;
            include_exact_point(bounds, arc.center().x(), &(arc.center().y() - &radius))?;
            include_exact_point(bounds, arc.center().x(), &(arc.center().y() + &radius))?;
        }
    }
    Some(())
}

fn include_exact_point(bounds: &mut Option<ScalarBounds2>, x: &Scalar, y: &Scalar) -> Option<()> {
    let Some(current) = bounds else {
        *bounds = Some(ScalarBounds2 {
            min_x: x.clone(),
            min_y: y.clone(),
            max_x: x.clone(),
            max_y: y.clone(),
        });
        return Some(());
    };
    include_exact_coordinate(&mut current.min_x, &mut current.max_x, x)?;
    include_exact_coordinate(&mut current.min_y, &mut current.max_y, y)
}

fn include_exact_coordinate(
    minimum: &mut Scalar,
    maximum: &mut Scalar,
    value: &Scalar,
) -> Option<()> {
    use std::cmp::Ordering;

    match scalar_cmp(value, minimum)? {
        Ordering::Less => *minimum = value.clone(),
        Ordering::Equal | Ordering::Greater => {}
    }
    if scalar_cmp(value, maximum)? == Ordering::Greater {
        *maximum = value.clone();
    }
    Some(())
}

fn scalar_cmp(left: &Scalar, right: &Scalar) -> Option<std::cmp::Ordering> {
    match (left.exact_rational_ref(), right.exact_rational_ref()) {
        (Some(left), Some(right)) => left.partial_cmp(right),
        _ => left.partial_cmp(right),
    }
}

fn scalar_lt(left: &Scalar, right: &Scalar) -> bool {
    scalar_cmp(left, right) == Some(std::cmp::Ordering::Less)
}

fn scalar_le(left: &Scalar, right: &Scalar) -> bool {
    matches!(
        scalar_cmp(left, right),
        Some(std::cmp::Ordering::Less | std::cmp::Ordering::Equal)
    )
}

fn scalar_gt(left: &Scalar, right: &Scalar) -> bool {
    scalar_cmp(left, right) == Some(std::cmp::Ordering::Greater)
}

fn exact_bounds_may_be_within(
    left: &ScalarBounds2,
    right: &ScalarBounds2,
    clearance: &Scalar,
) -> bool {
    scalar_le(&(&left.min_x - clearance), &right.max_x)
        && scalar_le(&right.min_x, &(&left.max_x + clearance))
        && scalar_le(&(&left.min_y - clearance), &right.max_y)
        && scalar_le(&right.min_y, &(&left.max_y + clearance))
}

fn maximum_bounding_dimension(sketch: &crate::PcbSketch) -> Scalar {
    let Some(bounds) = native_sketch_bounds_scalar(sketch) else {
        return Scalar::zero();
    };
    let width = &bounds.max_x - &bounds.min_x;
    let height = &bounds.max_y - &bounds.min_y;
    if width >= height { width } else { height }
}

fn estimated_feature_length(feature: &CopperFeature) -> Scalar {
    match feature.kind {
        // KiCad segment parsing currently emits rectangular copper envelopes.
        // The longest exterior edge recovers the centerline length for those
        // envelopes, including diagonal segments where an axis-aligned bounding
        // box underestimates length. This is still readiness metadata, not
        // routed-path reconstruction or a transmission-line delay model; for
        // the underlying planar geometry assumptions see Lee and Preparata,
        // "Computational Geometry - A Survey", IEEE Transactions on Computers,
        // 1984, doi:10.1109/TC.1984.1676388.
        CopperKind::Segment => {
            let edge_length = maximum_exterior_edge_length(&feature.sketch);
            if edge_length > Scalar::zero() {
                edge_length
            } else {
                maximum_bounding_dimension(&feature.sketch)
            }
        }
        CopperKind::Via => Scalar::zero(),
        CopperKind::Pad | CopperKind::Zone => Scalar::zero(),
    }
}

fn maximum_exterior_edge_length(sketch: &crate::PcbSketch) -> Scalar {
    sketch
        .as_region()
        .material_contours()
        .iter()
        .flat_map(|contour| contour.segments())
        .filter_map(|segment| {
            let dx = segment.end().x() - segment.start().x();
            let dy = segment.end().y() - segment.start().y();
            (&dx * &dx + &dy * &dy).sqrt().ok()
        })
        .fold(Scalar::zero(), |maximum, length| {
            if length > maximum { length } else { maximum }
        })
}

#[derive(Clone, Debug)]
struct NetUse {
    layers: BTreeSet<String>,
    locations: Vec<[f64; 2]>,
    via_count: usize,
    estimated_length: Scalar,
    max_length: Option<Scalar>,
}

impl Default for NetUse {
    fn default() -> Self {
        Self {
            layers: BTreeSet::new(),
            locations: Vec::new(),
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

    fn locations(&self) -> Vec<[f64; 2]> {
        self.positive
            .locations
            .iter()
            .chain(self.negative.locations.iter())
            .copied()
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
        (Some(left), Some(right)) => Some(if left >= right { left } else { right }),
        (Some(value), None) | (None, Some(value)) => Some(value),
        (None, None) => None,
    }
}

fn option_min(left: Option<Scalar>, right: Option<Scalar>) -> Option<Scalar> {
    match (left, right) {
        (Some(left), Some(right)) => Some(if left <= right { left } else { right }),
        (Some(value), None) | (None, Some(value)) => Some(value),
        (None, None) => None,
    }
}

#[derive(Clone, Debug)]
struct DifferentialSideUse<'a> {
    net_names: BTreeSet<String>,
    layers: BTreeSet<String>,
    locations: Vec<[f64; 2]>,
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
            locations: Vec::new(),
            features: Vec::new(),
            min_pair_spacing: None,
            max_pair_spacing: None,
            estimated_length: Scalar::zero(),
            max_pair_skew: None,
        }
    }
}

#[cfg(test)]
mod tests {
    use crate::constraint_policy::{
        DifferentialRole, FabricationCapabilityConfig, NetClassConfig, NetClassRegionConfig,
        StackupConfig, StackupLayerConfig, StackupLayerKind, SurfaceFinish,
    };
    use crate::geometry::{circle_polygon, line_polygon, polygons_to_profile, rect_polygon};
    use crate::kicad::{BoardModel, CopperFeature, CopperKind};

    use super::{net_constraint_readiness, stackup_readiness};

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
                .any(|message| message.contains("fabricator profile custom supports up to 1"))
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

        assert!(
            messages
                .iter()
                .any(|message| message.contains("fabricator profile custom preferred maximum"))
        );
        assert!(
            messages
                .iter()
                .any(|message| message.contains("fabricator profile custom cost-escalation"))
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
            sketch: polygons_to_profile(vec![polygon], None),
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
            target_impedance_ohms: Some(crate::scalar::scalar("90.0")),
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
                    dielectric_thickness: Some(crate::scalar::scalar("1.5")),
                },
            ],
            ..StackupConfig::default()
        };
        let board = board_with_features(vec![
            feature("F.Cu", "USB_D+", CopperKind::Segment, [0.0, 0.0], 1.0, 0.3),
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

        let messages = net_constraint_readiness(&classes, Some(&stackup), &[board], &[])
            .into_iter()
            .filter_map(|violation| violation.message)
            .collect::<Vec<_>>();

        assert!(messages.iter().any(|message| {
            message.contains("estimated outer microstrip impedance")
                && message.contains("outside target")
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
    fn net_constraint_readiness_skips_differential_targets_for_single_ended_estimate() {
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

        assert!(net_constraint_readiness(&classes, Some(&stackup), &[board], &[]).is_empty());
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
            sketch: polygons_to_profile(vec![polygon], None),
            location: [
                crate::geometry::exact_real(center[0]),
                crate::geometry::exact_real(center[1]),
            ],
        }
    }
}
