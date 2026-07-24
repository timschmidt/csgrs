//! Power-converter and switching-node readiness checks.
//!
//! These checks focus on the geometry that tends to dominate switch-mode power
//! converter release review: high-dV/dt nodes, inductor keepouts, and nearby
//! copper that can increase coupling or unintended loop area.
//!
//! Reliability note: switching-node and inductor checks are name/geometry
//! heuristics, not loop-area extraction or EMI simulation. Suspect findings need
//! review against the schematic, regulator layout guide, and measured layout.

use crate::PcbSketchExt;
use crate::Scalar;
use crate::checks::distance::polygon_boundary_distance_scalar;
use crate::checks::offset_for_check;
use crate::geometry::multipolygon_to_shapes_scalar;
use crate::kicad::{BoardModel, CopperFeature};
use crate::report::{Severity, Violation};
use csgrs::csg::CSG;

/// Review likely switching nodes for nearby non-ground copper.
///
/// This is an EMI/layout-readiness heuristic, not a loop-area solver. It uses
/// net-name intent plus same-layer geometry to flag copper that should be
/// reviewed against regulator or motor-drive layout guidance. Spatial
/// candidates are selected first; exact offset/intersection or boundary
/// distance decides the finding.
pub fn switch_node_keepout_readiness(
    board: &BoardModel,
    selected_layers: &[String],
    keepout: Scalar,
    min_area: &Scalar,
) -> Vec<Violation> {
    let features = selected_copper_features(board, selected_layers);
    let switching = features
        .iter()
        .copied()
        .filter(|feature| feature.net.as_deref().is_some_and(looks_switching_net))
        .collect::<Vec<_>>();
    log::trace!(
        "switch-node keepout readiness: source={} switching={} features={} keepout={keepout:#.6}",
        board.source,
        switching.len(),
        features.len(),
    );
    let mut violations = Vec::new();
    let mut candidate_count = 0_usize;
    let mut exact_pair_count = 0_usize;

    for switch_feature in switching {
        for neighbor in &features {
            let neighbor = *neighbor;
            candidate_count += 1;
            if std::ptr::eq(switch_feature, neighbor) {
                continue;
            }
            if switch_feature.layer != neighbor.layer {
                continue;
            }
            if switch_feature.net == neighbor.net {
                continue;
            }
            if neighbor.net.as_deref().is_some_and(looks_ground_net) {
                continue;
            }
            if !sketches_within_clearance(&switch_feature.sketch, &neighbor.sketch, &keepout) {
                continue;
            }
            exact_pair_count += 1;

            let expanded = match offset_for_check(
                &switch_feature.sketch,
                keepout.clone(),
                "switch-node-keepout-readiness",
                vec![switch_feature.layer.clone()],
            ) {
                Ok(expanded) => expanded,
                Err(uncertainty) => return vec![*uncertainty],
            };
            let overlap = expanded.intersection(&neighbor.sketch);
            let shapes = multipolygon_to_shapes_scalar(&overlap.to_multipolygon(), min_area);
            let fallback_hit = shapes.is_empty()
                && polygon_boundary_distance_scalar(
                    &switch_feature.sketch.to_multipolygon(),
                    &neighbor.sketch.to_multipolygon(),
                )
                .is_some_and(|distance| distance <= keepout);
            if shapes.is_empty() && !fallback_hit {
                continue;
            }

            violations.push(Violation::new(
                "switch-node-keepout-readiness",
                Severity::Warning,
                vec![switch_feature.layer.clone()],
                None,
                shapes,
                vec![
                    switch_feature.location_f64_compatibility_required(),
                    neighbor.location_f64_compatibility_required(),
                ],
                Some(format!(
                    "likely switching node {:?} is within keepout {keepout:#.6} of neighboring net {:?}; review regulator/motor loop area, EMI, and copper keepout",
                    switch_feature.net, neighbor.net
                )),
            ));
        }
    }

    log::trace!(
        "switch-node keepout readiness: source={} candidate_pairs={} exact_pairs={} violations={}",
        board.source,
        candidate_count,
        exact_pair_count,
        violations.len()
    );
    debug_assert!(exact_pair_count <= candidate_count);

    violations
}

/// Warn when likely inductor or switch-node copper has same-layer copper inside
/// a stricter copper-free review band.
///
/// This intentionally reports ground copper too. Whether a plane belongs under
/// an inductor or switch node depends on the converter topology, field
/// containment, shielded-inductor construction, and EMI strategy; HyperDRC only
/// makes the layout choice visible for review.
pub fn inductor_copper_keepout_readiness(
    board: &BoardModel,
    selected_layers: &[String],
    keepout: Scalar,
    min_area: &Scalar,
) -> Vec<Violation> {
    if keepout <= Scalar::zero() {
        return Vec::new();
    }

    let features = selected_copper_features(board, selected_layers);
    let inductors = features
        .iter()
        .copied()
        .filter(|feature| {
            feature
                .net
                .as_deref()
                .is_some_and(looks_inductor_or_switch_node)
        })
        .collect::<Vec<_>>();
    log::trace!(
        "inductor copper keepout readiness: source={} inductors={} features={} keepout={keepout:#.6}",
        board.source,
        inductors.len(),
        features.len(),
    );
    let mut violations = Vec::new();
    let mut candidate_count = 0_usize;
    let mut exact_pair_count = 0_usize;

    for inductor in inductors {
        for neighbor in &features {
            let neighbor = *neighbor;
            candidate_count += 1;
            if std::ptr::eq(inductor, neighbor) {
                continue;
            }
            if inductor.layer != neighbor.layer {
                continue;
            }
            if inductor.net == neighbor.net {
                continue;
            }
            if !sketches_within_clearance(&inductor.sketch, &neighbor.sketch, &keepout) {
                continue;
            }
            exact_pair_count += 1;

            let expanded = match offset_for_check(
                &inductor.sketch,
                keepout.clone(),
                "inductor-copper-keepout-readiness",
                vec![inductor.layer.clone()],
            ) {
                Ok(expanded) => expanded,
                Err(uncertainty) => return vec![*uncertainty],
            };
            let overlap = expanded.intersection(&neighbor.sketch);
            let shapes = multipolygon_to_shapes_scalar(&overlap.to_multipolygon(), min_area);
            let fallback_hit = shapes.is_empty()
                && polygon_boundary_distance_scalar(
                    &inductor.sketch.to_multipolygon(),
                    &neighbor.sketch.to_multipolygon(),
                )
                .is_some_and(|distance| distance <= keepout);
            if shapes.is_empty() && !fallback_hit {
                continue;
            }

            violations.push(Violation::new(
                "inductor-copper-keepout-readiness",
                Severity::Warning,
                vec![inductor.layer.clone()],
                None,
                shapes,
                vec![
                    inductor.location_f64_compatibility_required(),
                    neighbor.location_f64_compatibility_required(),
                ],
                Some(format!(
                    "likely inductor or switch-node net {:?} has copper from net {:?} inside keepout {keepout:#.6}; review inductor copper-free region, EMI coupling, and regulator layout",
                    inductor.net, neighbor.net
                )),
            ));
        }
    }

    log::trace!(
        "inductor copper keepout readiness: source={} candidate_pairs={} exact_pairs={} violations={}",
        board.source,
        candidate_count,
        exact_pair_count,
        violations.len()
    );
    debug_assert!(exact_pair_count <= candidate_count);

    violations
}

fn selected_copper_features<'a>(
    board: &'a BoardModel,
    selected_layers: &[String],
) -> Vec<&'a CopperFeature> {
    board
        .copper
        .iter()
        .filter(|feature| selected_layers.is_empty() || selected_layers.contains(&feature.layer))
        .collect()
}

fn sketches_within_clearance(
    left: &crate::PcbSketch,
    right: &crate::PcbSketch,
    clearance: &Scalar,
) -> bool {
    if left.is_empty() || right.is_empty() {
        return true;
    }
    let left_bounds = left.bounding_box();
    let right_bounds = right.bounding_box();

    // Broad-phase culling before exact offset/intersection.
    &left_bounds.mins.x - clearance <= right_bounds.maxs.x
        && &left_bounds.maxs.x + clearance >= right_bounds.mins.x
        && &left_bounds.mins.y - clearance <= right_bounds.maxs.y
        && &left_bounds.maxs.y + clearance >= right_bounds.mins.y
}

fn looks_switching_net(net: &str) -> bool {
    let normalized = net.to_ascii_uppercase();
    let tokens = [
        "SW", "PHASE", "LX", "BOOT", "BST", "GATE", "HGATE", "LGATE", "DRV", "DRIVE", "MOTOR",
        "PWM", "IND", "INDUCTOR",
    ];

    tokens.iter().any(|token| normalized.contains(token))
}

fn looks_inductor_or_switch_node(net: &str) -> bool {
    let normalized = net.to_ascii_uppercase();
    let tokens = ["SW", "PHASE", "LX", "IND", "INDUCTOR", "COIL"];

    tokens.iter().any(|token| normalized.contains(token))
}

fn looks_ground_net(net: &str) -> bool {
    let normalized = net.to_ascii_uppercase();
    matches!(
        normalized.as_str(),
        "GND" | "GROUND" | "PGND" | "AGND" | "DGND"
    ) || normalized.ends_with("_GND")
        || normalized.ends_with("-GND")
}

#[cfg(test)]
mod tests {
    use crate::LayerMetadata;
    use crate::geometry::{polygons_to_profile, rect_polygon};
    use crate::kicad::{BoardModel, CopperFeature, CopperKind};

    use super::{inductor_copper_keepout_readiness, switch_node_keepout_readiness};

    #[test]
    fn switch_node_keepout_readiness_reports_nearby_non_ground_copper() {
        let board = board_with_copper(vec![
            copper_rect("BUCK_SW", CopperKind::Segment, "F.Cu", 0.0, 0.0, 1.0, 0.5),
            copper_rect("ADC_IN", CopperKind::Segment, "F.Cu", 1.2, 0.0, 2.0, 0.5),
            copper_rect("GND", CopperKind::Zone, "F.Cu", 0.0, 2.0, 2.0, 3.0),
        ]);

        let violations = switch_node_keepout_readiness(
            &board,
            &[],
            crate::scalar::scalar("0.3"),
            &crate::scalar::scalar("1e-9"),
        );

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "switch-node-keepout-readiness");
        assert!(
            violations[0]
                .message
                .as_deref()
                .is_some_and(|message| message.contains("EMI"))
        );
    }

    #[test]
    fn switch_node_keepout_readiness_allows_same_net_ground_and_distant_copper() {
        let board = board_with_copper(vec![
            copper_rect("BUCK_SW", CopperKind::Segment, "F.Cu", 0.0, 0.0, 1.0, 0.5),
            copper_rect("BUCK_SW", CopperKind::Pad, "F.Cu", 1.1, 0.0, 1.5, 0.5),
            copper_rect("GND", CopperKind::Zone, "F.Cu", 1.2, 0.0, 2.0, 0.5),
            copper_rect("ADC_IN", CopperKind::Segment, "F.Cu", 4.0, 0.0, 5.0, 0.5),
        ]);

        assert!(
            switch_node_keepout_readiness(
                &board,
                &[],
                crate::scalar::scalar("0.3"),
                &crate::scalar::scalar("1e-9"),
            )
            .is_empty()
        );
    }

    #[test]
    fn switch_node_keepout_readiness_handles_sparse_neighbor_fields() {
        let mut copper = sparse_signal_rects("GPIO", 2_000, 100.0);
        copper.push(copper_rect(
            "BUCK_SW",
            CopperKind::Segment,
            "F.Cu",
            0.0,
            0.0,
            1.0,
            0.5,
        ));
        copper.push(copper_rect(
            "ADC_NEAR",
            CopperKind::Segment,
            "F.Cu",
            1.2,
            0.0,
            2.0,
            0.5,
        ));
        copper.push(copper_rect(
            "ADC_OTHER_LAYER",
            CopperKind::Segment,
            "B.Cu",
            1.2,
            0.0,
            2.0,
            0.5,
        ));
        let board = board_with_copper(copper);

        let violations = switch_node_keepout_readiness(
            &board,
            &[],
            crate::scalar::scalar("0.3"),
            &crate::scalar::scalar("1e-9"),
        );

        assert_eq!(violations.len(), 1);
        assert!(violations[0].message.as_ref().is_some_and(|message| {
            message.contains("ADC_NEAR") && !message.contains("ADC_OTHER_LAYER")
        }));
    }

    #[test]
    fn inductor_copper_keepout_readiness_reports_ground_under_inductor_region() {
        let board = board_with_copper(vec![
            copper_rect("BUCK_LX", CopperKind::Pad, "F.Cu", 0.0, 0.0, 1.0, 0.8),
            copper_rect("PGND", CopperKind::Zone, "F.Cu", 1.2, 0.0, 2.0, 0.8),
        ]);

        let violations = inductor_copper_keepout_readiness(
            &board,
            &[],
            crate::scalar::scalar("0.30"),
            &crate::scalar::scalar("1e-9"),
        );

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "inductor-copper-keepout-readiness");
        assert!(
            violations[0]
                .message
                .as_deref()
                .is_some_and(|message| message.contains("copper-free"))
        );
    }

    #[test]
    fn inductor_copper_keepout_readiness_respects_layer_and_same_net() {
        let board = board_with_copper(vec![
            copper_rect("COIL_SW", CopperKind::Pad, "B.Cu", 0.0, 0.0, 1.0, 0.8),
            copper_rect("COIL_SW", CopperKind::Zone, "B.Cu", 1.2, 0.0, 2.0, 0.8),
            copper_rect("GND", CopperKind::Zone, "F.Cu", 1.2, 0.0, 2.0, 0.8),
        ]);

        assert!(
            inductor_copper_keepout_readiness(
                &board,
                &[],
                crate::scalar::scalar("0.30"),
                &crate::scalar::scalar("1e-9"),
            )
            .is_empty()
        );
        assert!(
            inductor_copper_keepout_readiness(
                &board,
                &["F.Cu".to_string()],
                crate::scalar::scalar("0.30"),
                &crate::scalar::scalar("1e-9"),
            )
            .is_empty()
        );
    }

    #[test]
    fn inductor_copper_keepout_readiness_handles_sparse_copper_fields() {
        let mut copper = sparse_signal_rects("SIG", 2_000, 100.0);
        copper.push(copper_rect(
            "COIL_SW",
            CopperKind::Pad,
            "F.Cu",
            0.0,
            0.0,
            1.0,
            0.8,
        ));
        copper.push(copper_rect(
            "PGND",
            CopperKind::Zone,
            "F.Cu",
            1.2,
            0.0,
            2.0,
            0.8,
        ));
        let board = board_with_copper(copper);

        let violations = inductor_copper_keepout_readiness(
            &board,
            &[],
            crate::scalar::scalar("0.30"),
            &crate::scalar::scalar("1e-9"),
        );

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "inductor-copper-keepout-readiness");
    }

    fn board_with_copper(copper: Vec<CopperFeature>) -> BoardModel {
        BoardModel {
            source: "test".to_string(),
            copper,
            drills: Vec::new(),
            board_outline: None,
            panel_features: None,
        }
    }

    fn copper_rect(
        net: &str,
        kind: CopperKind,
        layer: &str,
        min_x: f64,
        min_y: f64,
        max_x: f64,
        max_y: f64,
    ) -> CopperFeature {
        CopperFeature {
            layer: layer.to_string(),
            net: Some(net.to_string()),
            kind,
            location: [
                crate::geometry::exact_real((min_x + max_x) / 2.0),
                crate::geometry::exact_real((min_y + max_y) / 2.0),
            ],
            sketch: polygons_to_profile(
                vec![rect_polygon(
                    [(min_x + max_x) / 2.0, (min_y + max_y) / 2.0],
                    [max_x - min_x, max_y - min_y],
                    0.0,
                )],
                Some(LayerMetadata {
                    name: "test rect".to_string(),
                }),
            ),
        }
    }

    fn sparse_signal_rects(prefix: &str, count: usize, offset_x: f64) -> Vec<CopperFeature> {
        (0..count)
            .map(|index| {
                let x = offset_x + (index % 100) as f64 * 3.0;
                let y = (index / 100) as f64 * 3.0;
                copper_rect(
                    &format!("{prefix}{index}"),
                    CopperKind::Segment,
                    "F.Cu",
                    x,
                    y,
                    x + 0.8,
                    y + 0.4,
                )
            })
            .collect()
    }
}
