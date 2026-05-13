//! Config-driven stackup and net-class readiness checks.
//!
//! These checks are deliberately conservative. IPC-2221B treats conductor
//! sizing, spacing, materials, and stackup as design constraints that depend on
//! voltage, current, environment, and fabrication capability; hyperdrc only
//! enforces the explicit project constraints supplied in `hyperdrc` config
//! rather than trying to infer a universal rule deck.

use std::collections::{BTreeMap, BTreeSet};

use geo::BoundingRect;

use super::distance::polygon_boundary_distance;
use crate::config::{NetClassConfig, StackupConfig, StackupLayerKind};
use crate::kicad::{BoardModel, CopperFeature, CopperKind};
use crate::report::{Severity, Violation};

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
        for layer in configured_copper_layers {
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

    violations
}

pub fn net_constraint_readiness(
    net_classes: &[NetClassConfig],
    boards: &[BoardModel],
    selected_layers: &[String],
) -> Vec<Violation> {
    if net_classes.is_empty() {
        return Vec::new();
    }

    let mut violations = Vec::new();
    for board in boards {
        let features = board
            .copper
            .iter()
            .filter(|feature| layer_selected(&feature.layer, selected_layers))
            .collect::<Vec<_>>();
        violations.extend(net_width_constraints(net_classes, &features));
        violations.extend(net_layer_and_via_constraints(net_classes, &features));
        violations.extend(net_clearance_constraints(net_classes, &features));
    }
    violations
}

fn net_width_constraints(
    net_classes: &[NetClassConfig],
    features: &[&CopperFeature],
) -> Vec<Violation> {
    let mut violations = Vec::new();
    for feature in features {
        let Some(net) = &feature.net else {
            continue;
        };
        for class in matching_classes(net_classes, net) {
            let Some(min_width) = class.min_width else {
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
        }
    }
    violations
}

fn net_layer_and_via_constraints(
    net_classes: &[NetClassConfig],
    features: &[&CopperFeature],
) -> Vec<Violation> {
    let mut by_net = BTreeMap::<String, NetUse>::new();
    for feature in features {
        let Some(net) = &feature.net else {
            continue;
        };
        let entry = by_net.entry(net.clone()).or_default();
        entry.layers.insert(feature.layer.clone());
        entry.locations.push(feature.location);
        if feature.kind == CopperKind::Via {
            entry.via_count += 1;
        }
    }

    let mut violations = Vec::new();
    for (net, usage) in by_net {
        for class in matching_classes(net_classes, &net) {
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
        }
    }
    violations
}

fn net_clearance_constraints(
    net_classes: &[NetClassConfig],
    features: &[&CopperFeature],
) -> Vec<Violation> {
    let mut violations = Vec::new();
    for (left_index, left) in features.iter().enumerate() {
        let Some(left_net) = &left.net else {
            continue;
        };
        for right in features.iter().skip(left_index + 1) {
            let Some(right_net) = &right.net else {
                continue;
            };
            if left.layer != right.layer || left_net == right_net {
                continue;
            }

            let Some((class_name, min_clearance)) =
                required_clearance(net_classes, left_net, right_net)
            else {
                continue;
            };
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
    violations
}

fn required_clearance<'a>(
    net_classes: &'a [NetClassConfig],
    left_net: &str,
    right_net: &str,
) -> Option<(&'a str, f64)> {
    matching_classes(net_classes, left_net)
        .into_iter()
        .chain(matching_classes(net_classes, right_net))
        .filter_map(|class| {
            class
                .min_clearance
                .map(|clearance| (class_name(class), clearance))
        })
        .max_by(|left, right| {
            left.1
                .partial_cmp(&right.1)
                .unwrap_or(std::cmp::Ordering::Equal)
        })
}

fn matching_classes<'a>(net_classes: &'a [NetClassConfig], net: &str) -> Vec<&'a NetClassConfig> {
    net_classes
        .iter()
        .filter(|class| {
            class.nets.iter().any(|candidate| candidate == net)
                || class
                    .net_patterns
                    .iter()
                    .any(|pattern| matches_pattern(pattern, net))
        })
        .collect()
}

fn matches_pattern(pattern: &str, net: &str) -> bool {
    match pattern.split_once('*') {
        Some((prefix, suffix)) => net.starts_with(prefix) && net.ends_with(suffix),
        None => pattern == net,
    }
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

#[derive(Clone, Debug, Default)]
struct NetUse {
    layers: BTreeSet<String>,
    locations: Vec<[f64; 2]>,
    via_count: usize,
}

#[cfg(test)]
mod tests {
    use crate::config::{NetClassConfig, StackupConfig, StackupLayerConfig, StackupLayerKind};
    use crate::geometry::{circle_polygon, polygons_to_sketch, rect_polygon};
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
        }];
        let board = board_with_features(vec![
            feature("F.Cu", "VBUS", CopperKind::Segment, [0.0, 0.0], 2.0, 0.2),
            feature("B.Cu", "VBUS", CopperKind::Segment, [0.0, 0.0], 2.0, 0.2),
            feature("F.Cu", "SIG", CopperKind::Segment, [0.3, 0.0], 0.2, 0.2),
        ]);

        let messages = net_constraint_readiness(&classes, &[board], &[])
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

        assert!(net_constraint_readiness(&classes, &[board], &[]).is_empty());
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
