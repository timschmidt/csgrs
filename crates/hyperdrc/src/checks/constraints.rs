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

use geo::BoundingRect;

use super::distance::polygon_boundary_distance;
use super::impedance::{ImpedanceModel, estimate_single_ended_impedance};
use super::net_class::resolve_net_classes;
use super::net_scope::{matching_class_indexes_for_feature, net_class_region_diagnostics};
use super::spatial::CopperSpatialIndex;
use crate::constraint_policy::{
    DifferentialRole, FabricationCapabilityConfig, NetClassConfig, StackupConfig, StackupLayerKind,
    SurfaceFinish,
};
use crate::kicad::{BoardModel, CopperFeature, CopperKind};
use crate::report::{Severity, Violation};

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
        if invalid_positive(stackup.material_dielectric_constant) {
            violations.push(stackup_metadata_violation(
                "stackup impedance_controlled=true but material_dielectric_constant is missing or invalid; review laminate Dk before impedance release",
            ));
        }
        if invalid_non_negative(stackup.material_loss_tangent) {
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

#[derive(Copy, Clone, Debug, Default)]
struct FabricationCapability {
    label: &'static str,
    min_finished_thickness: Option<f64>,
    preferred_min_finished_thickness: Option<f64>,
    preferred_max_finished_thickness: Option<f64>,
    max_finished_thickness: Option<f64>,
    max_copper_layers: Option<usize>,
    preferred_max_copper_layers: Option<usize>,
    cost_escalation_copper_layers: Option<usize>,
    min_copper_weight_oz: Option<f64>,
    preferred_min_copper_weight_oz: Option<f64>,
    preferred_max_copper_weight_oz: Option<f64>,
    cost_escalation_copper_weight_oz: Option<f64>,
    max_copper_weight_oz: Option<f64>,
    min_dielectric_thickness: Option<f64>,
    preferred_min_dielectric_thickness: Option<f64>,
    cost_escalation_min_dielectric_thickness: Option<f64>,
    min_dielectric_constant: Option<f64>,
    max_dielectric_constant: Option<f64>,
    max_loss_tangent: Option<f64>,
    min_tg_c: Option<f64>,
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
        stackup.finished_thickness,
        capability.min_finished_thickness,
    ) {
        if finished_thickness < minimum {
            violations.push(stackup_metadata_violation(&format!(
                "stackup finished_thickness {finished_thickness:.6} is below fabricator profile {} minimum {minimum:.6}",
                capability.label
            )));
        }
    }
    if let (Some(finished_thickness), Some(maximum)) = (
        stackup.finished_thickness,
        capability.max_finished_thickness,
    ) {
        if finished_thickness > maximum {
            violations.push(stackup_metadata_violation(&format!(
                "stackup finished_thickness {finished_thickness:.6} is above fabricator profile {} maximum {maximum:.6}",
                capability.label
            )));
        }
    }
    if let (Some(finished_thickness), Some(preferred_minimum)) = (
        stackup.finished_thickness,
        capability.preferred_min_finished_thickness,
    ) {
        if finished_thickness < preferred_minimum
            && capability
                .min_finished_thickness
                .is_none_or(|minimum| finished_thickness >= minimum)
        {
            violations.push(stackup_metadata_violation(&format!(
                "stackup finished_thickness {finished_thickness:.6} is below fabricator profile {} preferred minimum {preferred_minimum:.6}; review cost-escalation or special-process requirements",
                capability.label
            )));
        }
    }
    if let (Some(finished_thickness), Some(preferred_maximum)) = (
        stackup.finished_thickness,
        capability.preferred_max_finished_thickness,
    ) {
        if finished_thickness > preferred_maximum
            && capability
                .max_finished_thickness
                .is_none_or(|maximum| finished_thickness <= maximum)
        {
            violations.push(stackup_metadata_violation(&format!(
                "stackup finished_thickness {finished_thickness:.6} is above fabricator profile {} preferred maximum {preferred_maximum:.6}; review cost-escalation or special-process requirements",
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
        if let (Some(weight), Some(minimum)) =
            (layer.copper_weight_oz, capability.min_copper_weight_oz)
        {
            if weight < minimum {
                violations.push(stackup_metadata_violation(&format!(
                    "stackup copper layer {} has copper_weight_oz {weight:.6} below fabricator profile {} minimum {minimum:.6}",
                    layer.name, capability.label
                )));
            }
        }
        if let (Some(weight), Some(maximum)) =
            (layer.copper_weight_oz, capability.max_copper_weight_oz)
        {
            if weight > maximum {
                violations.push(stackup_metadata_violation(&format!(
                    "stackup copper layer {} has copper_weight_oz {weight:.6} above fabricator profile {} maximum {maximum:.6}",
                    layer.name, capability.label
                )));
            }
        }
        if let (Some(weight), Some(preferred_minimum)) = (
            layer.copper_weight_oz,
            capability.preferred_min_copper_weight_oz,
        ) {
            if weight < preferred_minimum
                && capability
                    .min_copper_weight_oz
                    .is_none_or(|minimum| weight >= minimum)
            {
                violations.push(stackup_metadata_violation(&format!(
                    "stackup copper layer {} has copper_weight_oz {weight:.6} below fabricator profile {} preferred minimum {preferred_minimum:.6}; review cost-escalation or special-process requirements",
                    layer.name, capability.label
                )));
            }
        }
        if let (Some(weight), Some(preferred_maximum)) = (
            layer.copper_weight_oz,
            capability.preferred_max_copper_weight_oz,
        ) {
            if weight > preferred_maximum
                && capability
                    .max_copper_weight_oz
                    .is_none_or(|maximum| weight <= maximum)
            {
                violations.push(stackup_metadata_violation(&format!(
                    "stackup copper layer {} has copper_weight_oz {weight:.6} above fabricator profile {} preferred maximum {preferred_maximum:.6}; review cost-escalation or special-process requirements",
                    layer.name, capability.label
                )));
            }
        }
        if let (Some(weight), Some(cost_threshold)) = (
            layer.copper_weight_oz,
            capability.cost_escalation_copper_weight_oz,
        ) {
            if weight > cost_threshold
                && capability
                    .max_copper_weight_oz
                    .is_none_or(|maximum| weight <= maximum)
            {
                violations.push(stackup_metadata_violation(&format!(
                    "stackup copper layer {} has copper_weight_oz {weight:.6} above fabricator profile {} cost-escalation threshold {cost_threshold:.6}; review quote class and fabrication lead time",
                    layer.name, capability.label
                )));
            }
        }
    }

    if let Some(minimum) = capability.min_dielectric_thickness {
        for layer in stackup.layers.iter().filter(|layer| {
            matches!(
                layer.kind,
                StackupLayerKind::Dielectric | StackupLayerKind::Core | StackupLayerKind::Prepreg
            )
        }) {
            if let Some(thickness) = layer.dielectric_thickness {
                if thickness < minimum {
                    violations.push(stackup_metadata_violation(&format!(
                        "stackup dielectric layer {} has dielectric_thickness {thickness:.6} below fabricator profile {} minimum {minimum:.6}",
                        layer.name, capability.label
                    )));
                }
            }
        }
    }
    if let Some(preferred_minimum) = capability.preferred_min_dielectric_thickness {
        for layer in stackup.layers.iter().filter(|layer| {
            matches!(
                layer.kind,
                StackupLayerKind::Dielectric | StackupLayerKind::Core | StackupLayerKind::Prepreg
            )
        }) {
            if let Some(thickness) = layer.dielectric_thickness {
                if thickness < preferred_minimum
                    && capability
                        .min_dielectric_thickness
                        .is_none_or(|minimum| thickness >= minimum)
                {
                    violations.push(stackup_metadata_violation(&format!(
                        "stackup dielectric layer {} has dielectric_thickness {thickness:.6} below fabricator profile {} preferred minimum {preferred_minimum:.6}; review cost-escalation or special-process requirements",
                        layer.name, capability.label
                    )));
                }
            }
        }
    }
    if let Some(cost_threshold) = capability.cost_escalation_min_dielectric_thickness {
        for layer in stackup.layers.iter().filter(|layer| {
            matches!(
                layer.kind,
                StackupLayerKind::Dielectric | StackupLayerKind::Core | StackupLayerKind::Prepreg
            )
        }) {
            if let Some(thickness) = layer.dielectric_thickness {
                if thickness < cost_threshold
                    && capability
                        .min_dielectric_thickness
                        .is_none_or(|minimum| thickness >= minimum)
                {
                    violations.push(stackup_metadata_violation(&format!(
                        "stackup dielectric layer {} has dielectric_thickness {thickness:.6} below fabricator profile {} cost-escalation threshold {cost_threshold:.6}; review quote class and fabrication lead time",
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
        stackup.material_dielectric_constant,
        capability.min_dielectric_constant,
    ) {
        if value < minimum {
            violations.push(stackup_metadata_violation(&format!(
                "stackup material_dielectric_constant {value:.6} is below fabricator profile {} minimum {minimum:.6}",
                capability.label
            )));
        }
    }
    if let (Some(value), Some(maximum)) = (
        stackup.material_dielectric_constant,
        capability.max_dielectric_constant,
    ) {
        if value > maximum {
            violations.push(stackup_metadata_violation(&format!(
                "stackup material_dielectric_constant {value:.6} is above fabricator profile {} maximum {maximum:.6}",
                capability.label
            )));
        }
    }
    if let (Some(value), Some(maximum)) =
        (stackup.material_loss_tangent, capability.max_loss_tangent)
    {
        if value > maximum {
            violations.push(stackup_metadata_violation(&format!(
                "stackup material_loss_tangent {value:.6} is above fabricator profile {} maximum {maximum:.6}",
                capability.label
            )));
        }
    }
    if let (Some(value), Some(minimum)) = (stackup.material_tg_c, capability.min_tg_c) {
        if value < minimum {
            violations.push(stackup_metadata_violation(&format!(
                "stackup material_tg_c {value:.6} is below fabricator profile {} minimum {minimum:.6}",
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
            .or(capability.min_finished_thickness);
        capability.preferred_min_finished_thickness = custom
            .preferred_min_finished_thickness
            .or(capability.preferred_min_finished_thickness);
        capability.preferred_max_finished_thickness = custom
            .preferred_max_finished_thickness
            .or(capability.preferred_max_finished_thickness);
        capability.max_finished_thickness = custom
            .max_finished_thickness
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
            .or(capability.min_copper_weight_oz);
        capability.preferred_min_copper_weight_oz = custom
            .preferred_min_copper_weight_oz
            .or(capability.preferred_min_copper_weight_oz);
        capability.preferred_max_copper_weight_oz = custom
            .preferred_max_copper_weight_oz
            .or(capability.preferred_max_copper_weight_oz);
        capability.cost_escalation_copper_weight_oz = custom
            .cost_escalation_copper_weight_oz
            .or(capability.cost_escalation_copper_weight_oz);
        capability.max_copper_weight_oz = custom
            .max_copper_weight_oz
            .or(capability.max_copper_weight_oz);
        capability.min_dielectric_thickness = custom
            .min_dielectric_thickness
            .or(capability.min_dielectric_thickness);
        capability.preferred_min_dielectric_thickness = custom
            .preferred_min_dielectric_thickness
            .or(capability.preferred_min_dielectric_thickness);
        capability.cost_escalation_min_dielectric_thickness = custom
            .cost_escalation_min_dielectric_thickness
            .or(capability.cost_escalation_min_dielectric_thickness);
        capability.min_dielectric_constant = custom
            .min_dielectric_constant
            .or(capability.min_dielectric_constant);
        capability.max_dielectric_constant = custom
            .max_dielectric_constant
            .or(capability.max_dielectric_constant);
        capability.max_loss_tangent = custom.max_loss_tangent.or(capability.max_loss_tangent);
        capability.min_tg_c = custom.min_tg_c.or(capability.min_tg_c);
        capability
    })
}

fn builtin_fabrication_capability(profile: &str) -> Option<FabricationCapability> {
    let normalized = profile.trim().to_ascii_lowercase();
    match normalized.as_str() {
        "prototype-fab" => Some(FabricationCapability {
            label: "prototype-fab",
            min_finished_thickness: Some(0.6),
            preferred_min_finished_thickness: Some(0.8),
            preferred_max_finished_thickness: Some(1.6),
            max_finished_thickness: Some(2.4),
            max_copper_layers: Some(4),
            preferred_max_copper_layers: Some(2),
            cost_escalation_copper_layers: Some(4),
            min_copper_weight_oz: Some(0.5),
            preferred_min_copper_weight_oz: Some(1.0),
            preferred_max_copper_weight_oz: Some(1.0),
            cost_escalation_copper_weight_oz: Some(2.0),
            max_copper_weight_oz: Some(2.0),
            min_dielectric_thickness: Some(0.05),
            preferred_min_dielectric_thickness: Some(0.10),
            cost_escalation_min_dielectric_thickness: Some(0.075),
            ..FabricationCapability::default()
        }),
        "standard-fab" => Some(FabricationCapability {
            label: "standard-fab",
            min_finished_thickness: Some(0.4),
            preferred_min_finished_thickness: Some(0.8),
            preferred_max_finished_thickness: Some(2.0),
            max_finished_thickness: Some(3.2),
            max_copper_layers: Some(8),
            preferred_max_copper_layers: Some(4),
            cost_escalation_copper_layers: Some(6),
            min_copper_weight_oz: Some(0.33),
            preferred_min_copper_weight_oz: Some(0.5),
            preferred_max_copper_weight_oz: Some(2.0),
            cost_escalation_copper_weight_oz: Some(2.0),
            max_copper_weight_oz: Some(3.0),
            min_dielectric_thickness: Some(0.04),
            preferred_min_dielectric_thickness: Some(0.075),
            cost_escalation_min_dielectric_thickness: Some(0.05),
            ..FabricationCapability::default()
        }),
        "jlcpcb-economy" | "jlcpcb-basic" => Some(FabricationCapability {
            label: "jlcpcb-economy",
            min_finished_thickness: Some(0.6),
            preferred_min_finished_thickness: Some(0.8),
            preferred_max_finished_thickness: Some(1.6),
            max_finished_thickness: Some(2.0),
            max_copper_layers: Some(4),
            preferred_max_copper_layers: Some(2),
            cost_escalation_copper_layers: Some(4),
            min_copper_weight_oz: Some(1.0),
            preferred_min_copper_weight_oz: Some(1.0),
            preferred_max_copper_weight_oz: Some(1.0),
            cost_escalation_copper_weight_oz: Some(1.0),
            max_copper_weight_oz: Some(2.0),
            min_dielectric_thickness: Some(0.075),
            preferred_min_dielectric_thickness: Some(0.10),
            cost_escalation_min_dielectric_thickness: Some(0.075),
            ..FabricationCapability::default()
        }),
        "jlcpcb-standard" => Some(FabricationCapability {
            label: "jlcpcb-standard",
            min_finished_thickness: Some(0.4),
            preferred_min_finished_thickness: Some(0.8),
            preferred_max_finished_thickness: Some(2.0),
            max_finished_thickness: Some(3.2),
            max_copper_layers: Some(6),
            preferred_max_copper_layers: Some(4),
            cost_escalation_copper_layers: Some(6),
            min_copper_weight_oz: Some(0.5),
            preferred_min_copper_weight_oz: Some(1.0),
            preferred_max_copper_weight_oz: Some(2.0),
            cost_escalation_copper_weight_oz: Some(2.0),
            max_copper_weight_oz: Some(3.0),
            min_dielectric_thickness: Some(0.04),
            preferred_min_dielectric_thickness: Some(0.075),
            cost_escalation_min_dielectric_thickness: Some(0.05),
            ..FabricationCapability::default()
        }),
        "jlcpcb-advanced" => Some(FabricationCapability {
            label: "jlcpcb-advanced",
            min_finished_thickness: Some(0.2),
            preferred_min_finished_thickness: Some(0.6),
            preferred_max_finished_thickness: Some(2.4),
            max_finished_thickness: Some(4.0),
            max_copper_layers: Some(12),
            preferred_max_copper_layers: Some(8),
            cost_escalation_copper_layers: Some(10),
            min_copper_weight_oz: Some(0.25),
            preferred_min_copper_weight_oz: Some(0.5),
            preferred_max_copper_weight_oz: Some(3.0),
            cost_escalation_copper_weight_oz: Some(3.0),
            max_copper_weight_oz: Some(4.0),
            min_dielectric_thickness: Some(0.025),
            preferred_min_dielectric_thickness: Some(0.05),
            cost_escalation_min_dielectric_thickness: Some(0.035),
            ..FabricationCapability::default()
        }),
        "pcbway-standard" => Some(FabricationCapability {
            label: "pcbway-standard",
            min_finished_thickness: Some(0.4),
            preferred_min_finished_thickness: Some(0.8),
            preferred_max_finished_thickness: Some(2.4),
            max_finished_thickness: Some(3.2),
            max_copper_layers: Some(14),
            preferred_max_copper_layers: Some(6),
            cost_escalation_copper_layers: Some(10),
            min_copper_weight_oz: Some(0.5),
            preferred_min_copper_weight_oz: Some(1.0),
            preferred_max_copper_weight_oz: Some(2.0),
            cost_escalation_copper_weight_oz: Some(3.0),
            max_copper_weight_oz: Some(8.0),
            min_dielectric_thickness: Some(0.04),
            preferred_min_dielectric_thickness: Some(0.075),
            cost_escalation_min_dielectric_thickness: Some(0.05),
            ..FabricationCapability::default()
        }),
        "pcbway-advanced" => Some(FabricationCapability {
            label: "pcbway-advanced",
            min_finished_thickness: Some(0.2),
            preferred_min_finished_thickness: Some(0.6),
            preferred_max_finished_thickness: Some(3.2),
            max_finished_thickness: Some(4.5),
            max_copper_layers: Some(30),
            preferred_max_copper_layers: Some(16),
            cost_escalation_copper_layers: Some(20),
            min_copper_weight_oz: Some(0.25),
            preferred_min_copper_weight_oz: Some(0.5),
            preferred_max_copper_weight_oz: Some(3.0),
            cost_escalation_copper_weight_oz: Some(4.0),
            max_copper_weight_oz: Some(8.0),
            min_dielectric_thickness: Some(0.025),
            preferred_min_dielectric_thickness: Some(0.05),
            cost_escalation_min_dielectric_thickness: Some(0.035),
            ..FabricationCapability::default()
        }),
        "eurocircuits-pcb-proto" | "eurocircuits-standard" => Some(FabricationCapability {
            label: "eurocircuits-pcb-proto",
            min_finished_thickness: Some(0.8),
            preferred_min_finished_thickness: Some(1.0),
            preferred_max_finished_thickness: Some(1.6),
            max_finished_thickness: Some(2.4),
            max_copper_layers: Some(4),
            preferred_max_copper_layers: Some(2),
            cost_escalation_copper_layers: Some(4),
            min_copper_weight_oz: Some(0.5),
            preferred_min_copper_weight_oz: Some(1.0),
            preferred_max_copper_weight_oz: Some(1.0),
            cost_escalation_copper_weight_oz: Some(2.0),
            max_copper_weight_oz: Some(2.0),
            min_dielectric_thickness: Some(0.075),
            preferred_min_dielectric_thickness: Some(0.10),
            cost_escalation_min_dielectric_thickness: Some(0.075),
            ..FabricationCapability::default()
        }),
        "eurocircuits-standard-pool" => Some(FabricationCapability {
            label: "eurocircuits-standard-pool",
            min_finished_thickness: Some(0.5),
            preferred_min_finished_thickness: Some(0.8),
            preferred_max_finished_thickness: Some(2.0),
            max_finished_thickness: Some(3.2),
            max_copper_layers: Some(8),
            preferred_max_copper_layers: Some(4),
            cost_escalation_copper_layers: Some(6),
            min_copper_weight_oz: Some(0.5),
            preferred_min_copper_weight_oz: Some(1.0),
            preferred_max_copper_weight_oz: Some(1.0),
            cost_escalation_copper_weight_oz: Some(2.0),
            max_copper_weight_oz: Some(2.0),
            min_dielectric_thickness: Some(0.05),
            preferred_min_dielectric_thickness: Some(0.075),
            cost_escalation_min_dielectric_thickness: Some(0.05),
            ..FabricationCapability::default()
        }),
        "eurocircuits-defined-impedance" => Some(FabricationCapability {
            label: "eurocircuits-defined-impedance",
            min_finished_thickness: Some(0.5),
            preferred_min_finished_thickness: Some(0.8),
            preferred_max_finished_thickness: Some(2.4),
            max_finished_thickness: Some(3.2),
            max_copper_layers: Some(12),
            preferred_max_copper_layers: Some(6),
            cost_escalation_copper_layers: Some(8),
            min_copper_weight_oz: Some(0.5),
            preferred_min_copper_weight_oz: Some(1.0),
            preferred_max_copper_weight_oz: Some(2.0),
            cost_escalation_copper_weight_oz: Some(2.0),
            max_copper_weight_oz: Some(3.0),
            min_dielectric_thickness: Some(0.04),
            preferred_min_dielectric_thickness: Some(0.075),
            cost_escalation_min_dielectric_thickness: Some(0.05),
            min_dielectric_constant: Some(3.0),
            max_dielectric_constant: Some(4.8),
            max_loss_tangent: Some(0.025),
            min_tg_c: Some(130.0),
            ..FabricationCapability::default()
        }),
        "advanced-fab" => Some(FabricationCapability {
            label: "advanced-fab",
            min_finished_thickness: Some(0.2),
            preferred_min_finished_thickness: Some(0.6),
            preferred_max_finished_thickness: Some(3.2),
            max_finished_thickness: Some(4.0),
            max_copper_layers: Some(12),
            preferred_max_copper_layers: Some(8),
            cost_escalation_copper_layers: Some(10),
            min_copper_weight_oz: Some(0.25),
            preferred_min_copper_weight_oz: Some(0.5),
            preferred_max_copper_weight_oz: Some(3.0),
            cost_escalation_copper_weight_oz: Some(3.0),
            max_copper_weight_oz: Some(4.0),
            min_dielectric_thickness: Some(0.025),
            preferred_min_dielectric_thickness: Some(0.05),
            cost_escalation_min_dielectric_thickness: Some(0.035),
            ..FabricationCapability::default()
        }),
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
            let Some(min_width) = class.min_width else {
                if let Some(min_current_width) = class.min_current_width {
                    let width = minimum_bounding_dimension(&feature.sketch);
                    if width > 0.0 && width < min_current_width {
                        violations.push(Violation::new(
                            "net-constraint-readiness",
                            Severity::Warning,
                            vec![feature.layer.clone()],
                            None,
                            Vec::new(),
                            vec![feature.location],
                            Some(format!(
                                "net {net} in class {} has parsed {:?} width {width:.6}, below configured current-carrying minimum {min_current_width:.6}",
                                class_name(class),
                                feature.kind
                            )),
                        ));
                    }
                }
                continue;
            };
            let width = minimum_bounding_dimension(&feature.sketch);
            if width > 0.0 && width < min_width {
                violations.push(Violation::new(
                    "net-constraint-readiness",
                    Severity::Error,
                    vec![feature.layer.clone()],
                    None,
                    Vec::new(),
                    vec![feature.location],
                    Some(format!(
                        "net {net} in class {} has parsed {:?} width {width:.6}, below configured minimum {min_width:.6}",
                        class_name(class),
                        feature.kind
                    )),
                ));
            }
            if let Some(min_current_width) = class.min_current_width
                && width > 0.0
                && width < min_current_width
            {
                violations.push(Violation::new(
                    "net-constraint-readiness",
                    Severity::Warning,
                    vec![feature.layer.clone()],
                    None,
                    Vec::new(),
                    vec![feature.location],
                    Some(format!(
                        "net {net} in class {} has parsed {:?} width {width:.6}, below configured current-carrying minimum {min_current_width:.6}",
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
            entry.locations.push(feature.location);
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
    // Broad-phase candidate generation follows Ericson, Real-Time Collision
    // Detection (2005): use a deterministic grid to bound same-layer candidates,
    // then keep the exact polygon-boundary distance as the release predicate.
    let feature_index = CopperSpatialIndex::new(features, maximum_clearance);
    let mut exact_pair_count = 0_usize;
    let mut violations = Vec::new();
    for (left_index, left) in features.iter().enumerate() {
        let Some(left_net) = &left.net else {
            continue;
        };
        for right_index in feature_index.same_layer_near_feature(left, maximum_clearance) {
            if right_index <= left_index {
                continue;
            }
            let right = features[right_index];
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
            let gap = polygon_boundary_distance(
                &left.sketch.to_multipolygon(),
                &right.sketch.to_multipolygon(),
            );
            if gap.is_finite() && gap < min_clearance {
                violations.push(Violation::new(
                    "net-constraint-readiness",
                    Severity::Error,
                    vec![left.layer.clone()],
                    None,
                    Vec::new(),
                    vec![left.location, right.location],
                    Some(format!(
                        "net {left_net} to {right_net} spacing {gap:.6} is below configured clearance {min_clearance:.6} from class {class_name}"
                    )),
                ));
            }
        }
    }
    log::trace!(
        "net-constraint clearance readiness: features={} spatial_buckets={} maximum_clearance={maximum_clearance:.6} exact_pairs={} violations={}",
        features.len(),
        feature_index.bucket_count(),
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
            entry.locations.push(feature.location);
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
            entry.locations.push(feature.location);
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
            entry.locations.push(feature.location);
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
            entry.locations.push(feature.location);
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

        let target_impedance_ohms = match class.target_impedance_ohms {
            Some(target) if target.is_finite() && target > 0.0 => Some(target),
            Some(target) => {
                violations.push(Violation::new(
                    "net-constraint-readiness",
                    Severity::Warning,
                    usage.layers.iter().cloned().collect(),
                    None,
                    Vec::new(),
                    usage.locations.clone(),
                    Some(format!(
                        "net {net} in class {} has invalid target_impedance_ohms {target:.6}",
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

        let impedance_tolerance_ohms = match class.impedance_tolerance_ohms {
            Some(tolerance) if tolerance.is_finite() && tolerance > 0.0 => Some(tolerance),
            Some(tolerance) => {
                violations.push(Violation::new(
                    "net-constraint-readiness",
                    Severity::Warning,
                    usage.layers.iter().cloned().collect(),
                    None,
                    Vec::new(),
                    usage.locations.clone(),
                    Some(format!(
                        "net {net} in class {} has invalid impedance_tolerance_ohms {tolerance:.6}",
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
            let delta = (estimate.impedance_ohms - target_impedance_ohms).abs();
            if delta > impedance_tolerance_ohms {
                violations.push(Violation::new(
                    "net-constraint-readiness",
                    Severity::Warning,
                    vec![feature.layer.clone()],
                    None,
                    Vec::new(),
                    vec![feature.location],
                    Some(format!(
                        "net {net} in class {} has estimated {} impedance {:.3} ohm from parsed width {:.6}, dielectric height/spacing {:.6}, and Dk {:.3}, outside target {:.3} +/- {:.3} ohm",
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
            side.locations.push(feature.location);
            side.features.push(*feature);
            side.min_pair_spacing = option_max(side.min_pair_spacing, class.min_pair_spacing);
            side.max_pair_spacing = option_min(side.max_pair_spacing, class.max_pair_spacing);
            side.max_pair_skew = option_min(side.max_pair_skew, class.max_pair_skew);
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
    min_spacing: Option<f64>,
    max_spacing: Option<f64>,
) -> Vec<Violation> {
    let query_spacing = [min_spacing, max_spacing]
        .into_iter()
        .flatten()
        .filter(|spacing| spacing.is_finite() && *spacing >= 0.0)
        .fold(0.0_f64, f64::max);
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

    // Broad-phase candidate generation follows Ericson, Real-Time Collision
    // Detection (2005). Configured pair-spacing checks still use exact polygon
    // boundary distance for every candidate, but sparse repeated pair segments
    // no longer need a full positive-by-negative scan.
    let negative_index = CopperSpatialIndex::new(&negative_features, query_spacing);
    let mut exact_pair_count = 0_usize;
    let mut closest_pair: Option<DifferentialGap<'_>> = None;
    let mut has_pair_within_max = max_spacing.is_none();
    let mut violations = Vec::new();

    for positive in &positive_features {
        for negative_index in negative_index.same_layer_near_feature(positive, query_spacing) {
            let negative = negative_features[negative_index];
            if positive.layer != negative.layer {
                continue;
            }
            let gap = polygon_boundary_distance(
                &positive.sketch.to_multipolygon(),
                &negative.sketch.to_multipolygon(),
            );
            if !gap.is_finite() {
                continue;
            }
            exact_pair_count += 1;
            let observed = DifferentialGap {
                positive,
                negative,
                gap,
            };
            if closest_pair
                .as_ref()
                .is_none_or(|closest| gap < closest.gap)
            {
                closest_pair = Some(observed);
            }

            if let Some(min_spacing) = min_spacing
                && gap < min_spacing
            {
                violations.push(Violation::new(
                    "net-constraint-readiness",
                    Severity::Error,
                    vec![positive.layer.clone()],
                    None,
                    Vec::new(),
                    vec![positive.location, negative.location],
                    Some(format!(
                        "differential pair {pair} side spacing {gap:.6} is below configured minimum {min_spacing:.6}"
                    )),
                ));
            }
            if let Some(max_spacing) = max_spacing
                && gap <= max_spacing
            {
                has_pair_within_max = true;
            }
        }
    }

    if !has_pair_within_max {
        if closest_pair.is_none() {
            closest_pair = first_same_layer_pair_gap(&positive_features, &negative_features);
        }
        if let (Some(max_spacing), Some(closest)) = (max_spacing, closest_pair)
            && closest.gap > max_spacing
        {
            violations.push(Violation::new(
                "net-constraint-readiness",
                Severity::Warning,
                vec![closest.positive.layer.clone()],
                None,
                Vec::new(),
                vec![closest.positive.location, closest.negative.location],
                Some(format!(
                    "differential pair {pair} nearest side spacing {:.6} is above configured maximum {max_spacing:.6}",
                    closest.gap
                )),
            ));
        }
    }

    log::trace!(
        "net-constraint differential-pair spacing readiness: pair={} positives={} negatives={} spatial_buckets={} exact_pairs={} violations={}",
        pair,
        positive_features.len(),
        negative_features.len(),
        negative_index.bucket_count(),
        exact_pair_count,
        violations.len()
    );
    violations
}

#[derive(Clone, Copy, Debug)]
struct DifferentialGap<'a> {
    positive: &'a CopperFeature,
    negative: &'a CopperFeature,
    gap: f64,
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
            let gap = polygon_boundary_distance(
                &positive.sketch.to_multipolygon(),
                &negative.sketch.to_multipolygon(),
            );
            if gap.is_finite() {
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
    let mut by_class_net = BTreeMap::<(usize, String), NetUse>::new();
    let mut pairs = BTreeMap::<String, DifferentialPairUse>::new();

    for feature in features {
        let Some(net) = &feature.net else {
            continue;
        };
        let estimated_length = estimated_feature_length(feature);
        if estimated_length <= 0.0 {
            continue;
        }

        for class_index in matching_class_indexes_for_feature(net_classes, feature) {
            let class = &net_classes[class_index];
            let usage = by_class_net.entry((class_index, net.clone())).or_default();
            usage.layers.insert(feature.layer.clone());
            usage.locations.push(feature.location);
            usage.estimated_length += estimated_length;
            usage.max_length = option_min(usage.max_length, class.max_length);

            let (Some(pair), Some(role)) = (&class.differential_pair, class.differential_role)
            else {
                continue;
            };
            let side = pairs.entry(pair.clone()).or_default().side_mut(role);
            side.net_names.insert(net.clone());
            side.layers.insert(feature.layer.clone());
            side.locations.push(feature.location);
            side.estimated_length += estimated_length;
            side.max_pair_skew = option_min(side.max_pair_skew, class.max_pair_skew);
        }
    }

    let mut violations = Vec::new();
    for ((class_index, net), usage) in by_class_net {
        let class = &net_classes[class_index];
        if let Some(max_length) = usage.max_length
            && usage.estimated_length > max_length
        {
            violations.push(Violation::new(
                "net-constraint-readiness",
                Severity::Warning,
                usage.layers.iter().cloned().collect(),
                None,
                Vec::new(),
                usage.locations.clone(),
                Some(format!(
                    "net {net} in class {} has approximate parsed copper length {:.6}, above configured maximum {max_length:.6}",
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
        if pair_use.positive.estimated_length <= 0.0 || pair_use.negative.estimated_length <= 0.0 {
            continue;
        }
        let skew = (pair_use.positive.estimated_length - pair_use.negative.estimated_length).abs();
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
                    "differential pair {pair} has approximate parsed length skew {skew:.6}, above configured maximum {max_pair_skew:.6}"
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
) -> Option<(&'a str, f64)> {
    matching_class_indexes_for_feature(net_classes, left)
        .into_iter()
        .chain(matching_class_indexes_for_feature(net_classes, right))
        .filter_map(|class_index| {
            let class = &net_classes[class_index];
            let clearance = [class.min_clearance, class.min_voltage_clearance]
                .into_iter()
                .flatten()
                .max_by(|left, right| {
                    left.partial_cmp(right).unwrap_or(std::cmp::Ordering::Equal)
                })?;
            Some((class_name(class), clearance))
        })
        .max_by(|left, right| {
            left.1
                .partial_cmp(&right.1)
                .unwrap_or(std::cmp::Ordering::Equal)
        })
}

fn maximum_configured_clearance(net_classes: &[NetClassConfig]) -> Option<f64> {
    net_classes
        .iter()
        .flat_map(|class| [class.min_clearance, class.min_voltage_clearance])
        .flatten()
        .filter(|clearance| clearance.is_finite() && *clearance >= 0.0)
        .max_by(|left, right| left.partial_cmp(right).unwrap_or(std::cmp::Ordering::Equal))
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

fn invalid_positive(value: Option<f64>) -> bool {
    !value.is_some_and(|value| value.is_finite() && value > 0.0)
}

fn invalid_non_negative(value: Option<f64>) -> bool {
    !value.is_some_and(|value| value.is_finite() && value >= 0.0)
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

fn minimum_bounding_dimension(sketch: &crate::PcbSketch) -> f64 {
    sketch
        .to_multipolygon()
        .bounding_rect()
        .map(|rect| rect.width().min(rect.height()))
        .unwrap_or(0.0)
}

fn maximum_bounding_dimension(sketch: &crate::PcbSketch) -> f64 {
    sketch
        .to_multipolygon()
        .bounding_rect()
        .map(|rect| rect.width().max(rect.height()))
        .unwrap_or(0.0)
}

fn estimated_feature_length(feature: &CopperFeature) -> f64 {
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
            if edge_length.is_finite() && edge_length > 0.0 {
                edge_length
            } else {
                maximum_bounding_dimension(&feature.sketch)
            }
        }
        CopperKind::Via => 0.0,
        CopperKind::Pad | CopperKind::Zone => 0.0,
    }
}

fn maximum_exterior_edge_length(sketch: &crate::PcbSketch) -> f64 {
    sketch
        .to_multipolygon()
        .0
        .iter()
        .flat_map(|polygon| polygon.exterior().0.windows(2))
        .map(|window| {
            let dx = window[1].x - window[0].x;
            let dy = window[1].y - window[0].y;
            (dx * dx + dy * dy).sqrt()
        })
        .filter(|length| length.is_finite())
        .fold(0.0, f64::max)
}

#[derive(Clone, Debug, Default)]
struct NetUse {
    layers: BTreeSet<String>,
    locations: Vec<[f64; 2]>,
    via_count: usize,
    estimated_length: f64,
    max_length: Option<f64>,
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

    fn min_pair_spacing(&self) -> Option<f64> {
        option_max(
            self.positive.min_pair_spacing,
            self.negative.min_pair_spacing,
        )
    }

    fn max_pair_spacing(&self) -> Option<f64> {
        option_min(
            self.positive.max_pair_spacing,
            self.negative.max_pair_spacing,
        )
    }

    fn max_pair_skew(&self) -> Option<f64> {
        option_min(self.positive.max_pair_skew, self.negative.max_pair_skew)
    }
}

fn option_max(left: Option<f64>, right: Option<f64>) -> Option<f64> {
    match (left, right) {
        (Some(left), Some(right)) => Some(left.max(right)),
        (Some(value), None) | (None, Some(value)) => Some(value),
        (None, None) => None,
    }
}

fn option_min(left: Option<f64>, right: Option<f64>) -> Option<f64> {
    match (left, right) {
        (Some(left), Some(right)) => Some(left.min(right)),
        (Some(value), None) | (None, Some(value)) => Some(value),
        (None, None) => None,
    }
}

#[derive(Clone, Debug, Default)]
struct DifferentialSideUse<'a> {
    net_names: BTreeSet<String>,
    layers: BTreeSet<String>,
    locations: Vec<[f64; 2]>,
    features: Vec<&'a CopperFeature>,
    min_pair_spacing: Option<f64>,
    max_pair_spacing: Option<f64>,
    estimated_length: f64,
    max_pair_skew: Option<f64>,
}

#[cfg(test)]
mod tests {
    use crate::constraint_policy::{
        DifferentialRole, FabricationCapabilityConfig, NetClassConfig, NetClassRegionConfig,
        StackupConfig, StackupLayerConfig, StackupLayerKind, SurfaceFinish,
    };
    use crate::geometry::{circle_polygon, line_polygon, polygons_to_sketch, rect_polygon};
    use crate::kicad::{BoardModel, CopperFeature, CopperKind};

    use super::{net_constraint_readiness, stackup_readiness};

    #[test]
    fn stackup_readiness_reports_layer_count_and_missing_metadata() {
        let stackup = StackupConfig {
            copper_layer_count: Some(4),
            finished_thickness: Some(1.6),
            layers: vec![
                StackupLayerConfig {
                    name: "F.Cu".to_string(),
                    kind: StackupLayerKind::Copper,
                    copper_weight_oz: Some(1.0),
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
        stackup.finished_thickness = Some(0.3);
        stackup.layers = vec![
            StackupLayerConfig {
                name: "F.Cu".to_string(),
                kind: StackupLayerKind::Copper,
                copper_weight_oz: Some(3.0),
                dielectric_thickness: None,
            },
            StackupLayerConfig {
                name: "In1.Cu".to_string(),
                kind: StackupLayerKind::Copper,
                copper_weight_oz: Some(1.0),
                dielectric_thickness: None,
            },
            StackupLayerConfig {
                name: "In2.Cu".to_string(),
                kind: StackupLayerKind::Copper,
                copper_weight_oz: Some(1.0),
                dielectric_thickness: None,
            },
            StackupLayerConfig {
                name: "In3.Cu".to_string(),
                kind: StackupLayerKind::Copper,
                copper_weight_oz: Some(1.0),
                dielectric_thickness: None,
            },
            StackupLayerConfig {
                name: "In4.Cu".to_string(),
                kind: StackupLayerKind::Copper,
                copper_weight_oz: Some(1.0),
                dielectric_thickness: None,
            },
            StackupLayerConfig {
                name: "B.Cu".to_string(),
                kind: StackupLayerKind::Copper,
                copper_weight_oz: Some(1.0),
                dielectric_thickness: None,
            },
            StackupLayerConfig {
                name: "Core".to_string(),
                kind: StackupLayerKind::Core,
                copper_weight_oz: None,
                dielectric_thickness: Some(0.02),
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
            min_finished_thickness: Some(2.0),
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
        stackup.finished_thickness = Some(2.8);
        stackup.layers = (0..12)
            .map(|index| StackupLayerConfig {
                name: format!("L{}.Cu", index + 1),
                kind: StackupLayerKind::Copper,
                copper_weight_oz: Some(if index == 0 { 3.5 } else { 1.0 }),
                dielectric_thickness: None,
            })
            .chain(std::iter::once(StackupLayerConfig {
                name: "Core".to_string(),
                kind: StackupLayerKind::Core,
                copper_weight_oz: None,
                dielectric_thickness: Some(0.045),
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
        stackup.layers[0].copper_weight_oz = Some(2.5);
        stackup.fabrication_capability = FabricationCapabilityConfig {
            preferred_max_copper_weight_oz: Some(1.0),
            cost_escalation_copper_weight_oz: Some(2.0),
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
        stackup.material_dielectric_constant = Some(5.2);
        stackup.material_loss_tangent = Some(0.035);
        stackup.material_tg_c = Some(125.0);
        stackup.fabrication_capability = FabricationCapabilityConfig {
            min_dielectric_constant: Some(3.0),
            max_dielectric_constant: Some(4.8),
            max_loss_tangent: Some(0.02),
            min_tg_c: Some(140.0),
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
            min_width: Some(0.5),
            min_clearance: Some(0.4),
            max_layer_count: Some(1),
            min_via_count: Some(2),
            max_length: Some(1.0),
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
            max_length: Some(4.8),
            ..NetClassConfig::default()
        }];
        let polygon = line_polygon([0.0, 0.0], [3.0, 4.0], 0.2)
            .expect("diagonal segment should produce copper geometry");
        let board = board_with_features(vec![CopperFeature {
            layer: "F.Cu".to_string(),
            net: Some("SIG".to_string()),
            kind: CopperKind::Segment,
            sketch: polygons_to_sketch(vec![polygon], None),
            location: [1.5, 2.0],
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
            min_width: Some(0.5),
            min_clearance: Some(0.2),
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
                min_width: Some(0.5),
                min_clearance: Some(0.4),
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
                min_x: Some(0.0),
                min_y: Some(0.0),
                max_x: Some(2.0),
                max_y: Some(2.0),
                layers: vec!["F.Cu".to_string()],
            }],
            min_width: Some(0.5),
            min_clearance: Some(0.3),
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
                min_x: Some(2.0),
                min_y: Some(0.0),
                max_x: Some(1.0),
                max_y: Some(2.0),
                ..NetClassRegionConfig::default()
            }],
            min_width: Some(0.5),
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
            min_clearance: Some(0.4),
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
            min_current_width: Some(0.25),
            min_voltage_clearance: Some(0.5),
            requires_reference_plane: Some(true),
            requires_impedance_control: Some(true),
            ..NetClassConfig::default()
        }];
        let stackup = StackupConfig {
            copper_layer_count: Some(2),
            finished_thickness: Some(1.6),
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
            min_current_width: Some(0.25),
            min_voltage_clearance: Some(0.2),
            requires_reference_plane: Some(true),
            requires_impedance_control: Some(true),
            target_impedance_ohms: Some(90.0),
            impedance_tolerance_ohms: Some(10.0),
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
            material_dielectric_constant: Some(4.2),
            material_loss_tangent: Some(0.018),
            material_tg_c: Some(150.0),
            layers: vec![
                StackupLayerConfig {
                    name: "F.Cu".to_string(),
                    kind: StackupLayerKind::Copper,
                    copper_weight_oz: Some(1.0),
                    dielectric_thickness: None,
                },
                StackupLayerConfig {
                    name: "Core".to_string(),
                    kind: StackupLayerKind::Core,
                    copper_weight_oz: None,
                    dielectric_thickness: Some(1.5),
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
            target_impedance_ohms: Some(50.0),
            impedance_tolerance_ohms: Some(5.0),
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
            target_impedance_ohms: Some(50.0),
            impedance_tolerance_ohms: Some(10.0),
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
            target_impedance_ohms: Some(50.0),
            impedance_tolerance_ohms: Some(5.0),
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
            target_impedance_ohms: Some(50.0),
            impedance_tolerance_ohms: Some(6.0),
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
                target_impedance_ohms: Some(90.0),
                impedance_tolerance_ohms: Some(5.0),
                differential_pair: Some("usb".to_string()),
                differential_role: Some(DifferentialRole::Positive),
                ..NetClassConfig::default()
            },
            NetClassConfig {
                name: "usb negative".to_string(),
                nets: vec!["USB_D-".to_string()],
                requires_impedance_control: Some(true),
                target_impedance_ohms: Some(90.0),
                impedance_tolerance_ohms: Some(5.0),
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
                min_pair_spacing: Some(0.2),
                max_pair_spacing: Some(0.4),
                max_via_count: Some(0),
                max_pair_skew: Some(0.5),
                ..NetClassConfig::default()
            },
            NetClassConfig {
                name: "usb-n".to_string(),
                nets: vec!["USB_D-".to_string()],
                differential_pair: Some("usb".to_string()),
                differential_role: Some(DifferentialRole::Negative),
                min_pair_spacing: Some(0.2),
                max_pair_spacing: Some(0.4),
                max_pair_skew: Some(0.5),
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
                min_pair_spacing: Some(0.2),
                max_pair_spacing: Some(0.5),
                ..NetClassConfig::default()
            },
            NetClassConfig {
                name: "usb-n".to_string(),
                nets: vec!["USB_D-".to_string()],
                differential_pair: Some("usb".to_string()),
                differential_role: Some(DifferentialRole::Negative),
                min_pair_spacing: Some(0.2),
                max_pair_spacing: Some(0.5),
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
                max_pair_spacing: Some(0.5),
                ..NetClassConfig::default()
            },
            NetClassConfig {
                name: "usb-n".to_string(),
                nets: vec!["USB_D-".to_string()],
                differential_pair: Some("usb".to_string()),
                differential_role: Some(DifferentialRole::Negative),
                max_pair_spacing: Some(0.5),
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
                min_pair_spacing: Some(0.2),
                max_pair_spacing: Some(0.5),
                max_via_count: Some(1),
                ..NetClassConfig::default()
            },
            NetClassConfig {
                name: "usb-n".to_string(),
                nets: vec!["USB_D-".to_string()],
                differential_pair: Some("usb".to_string()),
                differential_role: Some(DifferentialRole::Negative),
                min_pair_spacing: Some(0.2),
                max_pair_spacing: Some(0.5),
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
            finished_thickness: Some(1.6),
            impedance_controlled,
            material_family: Some("FR-4".to_string()),
            material_dielectric_constant: Some(4.2),
            material_loss_tangent: Some(0.018),
            material_tg_c: Some(150.0),
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
                    copper_weight_oz: Some(1.0),
                    dielectric_thickness: None,
                },
                StackupLayerConfig {
                    name: "Core".to_string(),
                    kind: StackupLayerKind::Core,
                    copper_weight_oz: None,
                    dielectric_thickness: Some(1.5),
                },
                StackupLayerConfig {
                    name: "B.Cu".to_string(),
                    kind: StackupLayerKind::Copper,
                    copper_weight_oz: Some(1.0),
                    dielectric_thickness: None,
                },
            ],
        }
    }

    fn controlled_microstrip_stackup() -> StackupConfig {
        StackupConfig {
            copper_layer_count: Some(2),
            finished_thickness: Some(0.36),
            impedance_controlled: Some(true),
            material_family: Some("FR-4".to_string()),
            material_dielectric_constant: Some(4.2),
            material_loss_tangent: Some(0.018),
            material_tg_c: Some(150.0),
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
                    copper_weight_oz: Some(1.0),
                    dielectric_thickness: None,
                },
                StackupLayerConfig {
                    name: "Prepreg".to_string(),
                    kind: StackupLayerKind::Prepreg,
                    copper_weight_oz: None,
                    dielectric_thickness: Some(0.18),
                },
                StackupLayerConfig {
                    name: "B.Cu".to_string(),
                    kind: StackupLayerKind::Copper,
                    copper_weight_oz: Some(1.0),
                    dielectric_thickness: None,
                },
            ],
        }
    }

    fn controlled_centered_stripline_stackup() -> StackupConfig {
        StackupConfig {
            copper_layer_count: Some(3),
            finished_thickness: Some(0.54),
            impedance_controlled: Some(true),
            material_family: Some("FR-4".to_string()),
            material_dielectric_constant: Some(4.2),
            material_loss_tangent: Some(0.018),
            material_tg_c: Some(150.0),
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
                    copper_weight_oz: Some(1.0),
                    dielectric_thickness: None,
                },
                StackupLayerConfig {
                    name: "Prepreg".to_string(),
                    kind: StackupLayerKind::Prepreg,
                    copper_weight_oz: None,
                    dielectric_thickness: Some(0.18),
                },
                StackupLayerConfig {
                    name: "In1.Cu".to_string(),
                    kind: StackupLayerKind::Copper,
                    copper_weight_oz: Some(1.0),
                    dielectric_thickness: None,
                },
                StackupLayerConfig {
                    name: "Core".to_string(),
                    kind: StackupLayerKind::Core,
                    copper_weight_oz: None,
                    dielectric_thickness: Some(0.18),
                },
                StackupLayerConfig {
                    name: "B.Cu".to_string(),
                    kind: StackupLayerKind::Copper,
                    copper_weight_oz: Some(1.0),
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
            sketch: polygons_to_sketch(vec![polygon], None),
            location: center,
        }
    }
}
