//! Power-converter and switching-node readiness checks.
//!
//! These checks focus on the geometry that tends to dominate switch-mode power
//! converter release review: high-dV/dt nodes, inductor keepouts, and nearby
//! copper that can increase coupling or unintended loop area.
//!
//! Reliability note: switching-node and inductor checks are name/geometry
//! heuristics, not loop-area extraction or EMI simulation. Suspect findings need
//! review against the schematic, regulator layout guide, and measured layout.

use csgrs::csg::CSG;
use geo::BoundingRect;

use crate::checks::distance::polygon_boundary_distance;
use crate::checks::spatial::CopperSpatialIndex;
use crate::geometry::multipolygon_to_shapes;
use crate::kicad::{BoardModel, CopperFeature};
use crate::report::{Severity, Violation};

/// Run the `switch_node_keepout_readiness` design-readiness check or report helper.
pub fn switch_node_keepout_readiness(
    board: &BoardModel,
    selected_layers: &[String],
    keepout: f64,
    min_area: f64,
) -> Vec<Violation> {
    let features = selected_copper_features(board, selected_layers);
    let feature_index = CopperSpatialIndex::new(&features, keepout);
    let switching = features
        .iter()
        .copied()
        .filter(|feature| feature.net.as_deref().is_some_and(looks_switching_net))
        .collect::<Vec<_>>();
    log::trace!(
        "switch-node keepout readiness: source={} switching={} features={} buckets={} keepout={keepout:.6}",
        board.source,
        switching.len(),
        features.len(),
        feature_index.bucket_count()
    );
    let mut violations = Vec::new();
    let mut candidate_count = 0_usize;

    for switch_feature in switching {
        for neighbor_index in feature_index.same_layer_near_feature(switch_feature, keepout) {
            candidate_count += 1;
            let neighbor = features[neighbor_index];
            if std::ptr::eq(switch_feature, neighbor) {
                continue;
            }
            if switch_feature.net == neighbor.net {
                continue;
            }
            if neighbor.net.as_deref().is_some_and(looks_ground_net) {
                continue;
            }
            if !sketches_within_clearance(&switch_feature.sketch, &neighbor.sketch, keepout) {
                continue;
            }

            let overlap = switch_feature
                .sketch
                .offset(keepout)
                .intersection(&neighbor.sketch);
            let shapes = multipolygon_to_shapes(&overlap.to_multipolygon(), min_area);
            let fallback_hit = shapes.is_empty()
                && polygon_boundary_distance(
                    &switch_feature.sketch.to_multipolygon(),
                    &neighbor.sketch.to_multipolygon(),
                ) <= keepout;
            if shapes.is_empty() && !fallback_hit {
                continue;
            }

            violations.push(Violation::new(
                "switch-node-keepout-readiness",
                Severity::Warning,
                vec![switch_feature.layer.clone()],
                None,
                shapes,
                vec![switch_feature.location, neighbor.location],
                Some(format!(
                    "likely switching node {:?} is within keepout {keepout:.6} of neighboring net {:?}; review regulator/motor loop area, EMI, and copper keepout",
                    switch_feature.net, neighbor.net
                )),
            ));
        }
    }

    log::trace!(
        "switch-node keepout readiness: source={} candidate_pairs={} violations={}",
        board.source,
        candidate_count,
        violations.len()
    );

    violations
}

/// Warn when likely inductor or switch-node copper has same-layer copper inside
/// a stricter copper-free review band.
///
/// This intentionally reports ground copper too. Whether a plane belongs under
/// an inductor or switch node depends on the converter topology, field
/// containment, shielded-inductor construction, and EMI strategy; HyperDRC only
/// makes the layout choice visible for review. Bhargava, Pommerenke, Kam,
/// Centola, and Lam, "DC-DC buck converter EMI reduction using PCB layout
/// modification," *IEEE Transactions on Electromagnetic Compatibility* 53.3
/// (2011), pp. 806-813, <https://doi.org/10.1109/TEMC.2011.2145421>, shows
/// that buck-converter PCB layout changes affect loop inductance, dipole
/// moments, and far-field radiation, motivating geometry-based readiness review
/// around switching power stages.
pub fn inductor_copper_keepout_readiness(
    board: &BoardModel,
    selected_layers: &[String],
    keepout: f64,
    min_area: f64,
) -> Vec<Violation> {
    if keepout <= 0.0 {
        return Vec::new();
    }

    let features = selected_copper_features(board, selected_layers);
    let feature_index = CopperSpatialIndex::new(&features, keepout);
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
        "inductor copper keepout readiness: source={} inductors={} features={} buckets={} keepout={keepout:.6}",
        board.source,
        inductors.len(),
        features.len(),
        feature_index.bucket_count()
    );
    let mut violations = Vec::new();
    let mut candidate_count = 0_usize;

    for inductor in inductors {
        for neighbor_index in feature_index.same_layer_near_feature(inductor, keepout) {
            candidate_count += 1;
            let neighbor = features[neighbor_index];
            if std::ptr::eq(inductor, neighbor) {
                continue;
            }
            if inductor.net == neighbor.net {
                continue;
            }
            if !sketches_within_clearance(&inductor.sketch, &neighbor.sketch, keepout) {
                continue;
            }

            let overlap = inductor
                .sketch
                .offset(keepout)
                .intersection(&neighbor.sketch);
            let shapes = multipolygon_to_shapes(&overlap.to_multipolygon(), min_area);
            let fallback_hit = shapes.is_empty()
                && polygon_boundary_distance(
                    &inductor.sketch.to_multipolygon(),
                    &neighbor.sketch.to_multipolygon(),
                ) <= keepout;
            if shapes.is_empty() && !fallback_hit {
                continue;
            }

            violations.push(Violation::new(
                "inductor-copper-keepout-readiness",
                Severity::Warning,
                vec![inductor.layer.clone()],
                None,
                shapes,
                vec![inductor.location, neighbor.location],
                Some(format!(
                    "likely inductor or switch-node net {:?} has copper from net {:?} inside keepout {keepout:.6}; review inductor copper-free region, EMI coupling, and regulator layout",
                    inductor.net, neighbor.net
                )),
            ));
        }
    }

    log::trace!(
        "inductor copper keepout readiness: source={} candidate_pairs={} violations={}",
        board.source,
        candidate_count,
        violations.len()
    );

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
    clearance: f64,
) -> bool {
    let Some(left_bounds) = left.geometry.bounding_rect() else {
        return true;
    };
    let Some(right_bounds) = right.geometry.bounding_rect() else {
        return true;
    };

    // Broad-phase culling before exact offset/intersection, following the
    // broad/narrow collision pattern from Lin and Canny, "A Fast Algorithm for
    // Incremental Distance Calculation", IEEE ICRA, 1991.
    left_bounds.min().x - clearance <= right_bounds.max().x
        && left_bounds.max().x + clearance >= right_bounds.min().x
        && left_bounds.min().y - clearance <= right_bounds.max().y
        && left_bounds.max().y + clearance >= right_bounds.min().y
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
    use crate::geometry::{polygons_to_sketch, rect_polygon};
    use crate::kicad::{BoardModel, CopperFeature, CopperKind};

    use super::{inductor_copper_keepout_readiness, switch_node_keepout_readiness};

    #[test]
    fn switch_node_keepout_readiness_reports_nearby_non_ground_copper() {
        let board = board_with_copper(vec![
            copper_rect("BUCK_SW", CopperKind::Segment, "F.Cu", 0.0, 0.0, 1.0, 0.5),
            copper_rect("ADC_IN", CopperKind::Segment, "F.Cu", 1.2, 0.0, 2.0, 0.5),
            copper_rect("GND", CopperKind::Zone, "F.Cu", 0.0, 2.0, 2.0, 3.0),
        ]);

        let violations = switch_node_keepout_readiness(&board, &[], 0.3, 1.0e-9);

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

        assert!(switch_node_keepout_readiness(&board, &[], 0.3, 1.0e-9).is_empty());
    }

    #[test]
    fn switch_node_keepout_readiness_culls_sparse_neighbor_fields() {
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

        let violations = switch_node_keepout_readiness(&board, &[], 0.3, 1.0e-9);

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

        let violations = inductor_copper_keepout_readiness(&board, &[], 0.30, 1.0e-9);

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

        assert!(inductor_copper_keepout_readiness(&board, &[], 0.30, 1.0e-9).is_empty());
        assert!(
            inductor_copper_keepout_readiness(&board, &["F.Cu".to_string()], 0.30, 1.0e-9)
                .is_empty()
        );
    }

    #[test]
    fn inductor_copper_keepout_readiness_culls_sparse_copper_fields() {
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

        let violations = inductor_copper_keepout_readiness(&board, &[], 0.30, 1.0e-9);

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
            location: [(min_x + max_x) / 2.0, (min_y + max_y) / 2.0],
            sketch: polygons_to_sketch(
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
