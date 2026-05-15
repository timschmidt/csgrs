//! High-voltage, ESD, and protective-interface readiness checks.
//!
//! These checks use parsed KiCad copper, board outline geometry, and net-name
//! heuristics to flag geometry that usually needs safety, surge, or system-ESD
//! review before a production release.
//!
//! Reliability note: high-voltage and ESD intent from net names is suspect for
//! isolated domains, chassis strategies, and untranslated safety rules. Verify
//! against creepage/clearance standards and the system grounding design.

use csgrs::csg::CSG;
use geo::BoundingRect;

use crate::checks::distance::polygon_boundary_distance;
use crate::geometry::multipolygon_to_shapes;
use crate::kicad::{BoardModel, CopperFeature};
use crate::report::{Severity, Violation};

/// Warn when likely high-voltage copper enters the board-edge review band.
///
/// This is a heuristic readiness check over parsed KiCad copper. It uses
/// net-name intent and the parsed board outline to make edge creepage and
/// clearance review visible before release documentation is assembled.
pub fn high_voltage_edge_readiness(
    board: &BoardModel,
    selected_layers: &[String],
    edge_clearance: f64,
    min_area: f64,
) -> Vec<Violation> {
    let Some(outline) = &board.board_outline else {
        return Vec::new();
    };
    let allowed = outline.offset(-edge_clearance);
    let mut violations = Vec::new();

    for feature in selected_copper_features(board, selected_layers) {
        let Some(net) = &feature.net else {
            continue;
        };
        if !looks_high_voltage_net(net) {
            continue;
        }

        let intrusion = feature.sketch.difference(&allowed);
        let shapes = multipolygon_to_shapes(&intrusion.to_multipolygon(), min_area);
        if shapes.is_empty() {
            continue;
        }

        violations.push(Violation::new(
            "high-voltage-edge-readiness",
            Severity::Warning,
            vec![feature.layer.clone(), "KiCad Edge.Cuts".to_string()],
            None,
            shapes,
            vec![feature.location],
            Some(format!(
                "likely high-voltage net {net} is within {edge_clearance:.6} of the board edge; review edge creepage, clearance, and routed-slot barrier intent"
            )),
        ));
    }

    violations
}

/// Warn when likely high-voltage copper is close to unrelated copper.
///
/// The check expands likely high-voltage features by `clearance` and compares
/// them to different-net copper on the same selected layer. Boundary-distance
/// fallback keeps thin features visible even when polygon intersections produce
/// no reportable area.
pub fn voltage_clearance_readiness(
    board: &BoardModel,
    clearance: f64,
    selected_layers: &[String],
    min_area: f64,
) -> Vec<Violation> {
    let features = selected_copper_features(board, selected_layers);
    let high_voltage_indices = features
        .iter()
        .enumerate()
        .filter_map(|(index, feature)| {
            feature
                .net
                .as_deref()
                .is_some_and(looks_high_voltage_net)
                .then_some(index)
        })
        .collect::<Vec<_>>();
    log::trace!(
        "voltage-clearance readiness: source={} features={} high_voltage_features={} clearance={clearance:.6}",
        board.source,
        features.len(),
        high_voltage_indices.len()
    );
    let mut violations = Vec::new();

    for &left_index in &high_voltage_indices {
        let left = features[left_index];
        for (right_index, right) in features.iter().enumerate() {
            if left_index == right_index {
                continue;
            }
            if left.layer != right.layer || left.net.is_none() || left.net == right.net {
                continue;
            }
            let left_high_voltage = left.net.as_deref().is_some_and(looks_high_voltage_net);
            let right_high_voltage = right.net.as_deref().is_some_and(looks_high_voltage_net);
            if !left_high_voltage && !right_high_voltage {
                continue;
            }
            if right_high_voltage && right_index < left_index {
                continue;
            }
            if !sketches_within_clearance(&left.sketch, &right.sketch, clearance) {
                continue;
            }

            let overlap = left.sketch.offset(clearance).intersection(&right.sketch);
            let shapes = multipolygon_to_shapes(&overlap.to_multipolygon(), min_area);
            let locations = if shapes.is_empty()
                && polygon_boundary_distance(
                    &left.sketch.to_multipolygon(),
                    &right.sketch.to_multipolygon(),
                ) <= clearance
            {
                vec![left.location, right.location]
            } else {
                Vec::new()
            };
            if shapes.is_empty() && locations.is_empty() {
                continue;
            }

            violations.push(Violation::new(
                "voltage-clearance-readiness",
                Severity::Warning,
                vec![left.layer.clone()],
                None,
                shapes,
                locations,
                Some(format!(
                    "likely high-voltage net {:?} is within {clearance:.6} of net {:?}; review voltage-class creepage and clearance",
                    if left_high_voltage { &left.net } else { &right.net },
                    if left_high_voltage { &right.net } else { &left.net }
                )),
            ));
        }
    }

    violations
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

    // Cheap AABB broad-phase before exact offset/intersection work. This is the
    // same two-phase collision pattern described by Lin and Canny, "A Fast
    // Algorithm for Incremental Distance Calculation", IEEE ICRA, 1991.
    left_bounds.min().x - clearance <= right_bounds.max().x
        && left_bounds.max().x + clearance >= right_bounds.min().x
        && left_bounds.min().y - clearance <= right_bounds.max().y
        && left_bounds.max().y + clearance >= right_bounds.min().y
}

/// Warn when exposed connector-like nets lack nearby ESD protection copper.
///
/// Nets that look like connector, RF, or high-speed interfaces and sit near the
/// board edge should usually have nearby ESD, chassis, shield, or ground copper.
/// This check reports those entry points for system-level ESD review.
pub fn esd_protection_readiness(
    board: &BoardModel,
    selected_layers: &[String],
    edge_distance: f64,
    protection_search_radius: f64,
) -> Vec<Violation> {
    let Some(outline) = &board.board_outline else {
        return Vec::new();
    };

    let features = selected_copper_features(board, selected_layers);
    let protection_features = features
        .iter()
        .copied()
        .filter(|feature| {
            feature.net.as_deref().is_some_and(|net| {
                looks_esd_protection_net(net) || looks_chassis_net(net) || looks_ground_net(net)
            })
        })
        .collect::<Vec<_>>();
    log::trace!(
        "ESD protection readiness: source={} features={} protection_features={}",
        board.source,
        features.len(),
        protection_features.len()
    );
    let mut violations = Vec::new();

    for feature in features {
        let Some(net) = feature.net.as_deref() else {
            continue;
        };
        if !looks_connector_edge_rate_net(net) || looks_esd_protection_net(net) {
            continue;
        }

        let edge_gap = polygon_boundary_distance(
            &feature.sketch.to_multipolygon(),
            &outline.to_multipolygon(),
        );
        if edge_gap > edge_distance {
            continue;
        }

        let has_protection = protection_features
            .iter()
            .filter(|protection| protection.layer == feature.layer)
            .any(|protection| {
                !std::ptr::eq(*protection, feature)
                    && distance(protection.location, feature.location) <= protection_search_radius
            });
        if has_protection {
            continue;
        }

        violations.push(Violation::new(
            "esd-protection-readiness",
            Severity::Warning,
            vec![feature.layer.clone()],
            None,
            Vec::new(),
            vec![feature.location],
            Some(format!(
                "likely edge connector net {net:?} is {edge_gap:.6} from board edge without parsed ESD/chassis/ground protection copper within {protection_search_radius:.6}"
            )),
        ));
    }

    violations
}

/// Warn when parsed TVS/ESD clamp copper lacks a nearby low-impedance return.
///
/// System-level ESD pulses are fast enough that parasitic inductance in the
/// clamp path can dominate protection behavior. This check reports likely
/// ESD/TVS/protection nets that do not have nearby same-layer ground or chassis
/// copper. It follows the layout model described in STMicroelectronics AN576,
/// "Influence of the PCB layout on the ESD protection," which explains that
/// ESD surge `di/dt` across PCB parasitic inductance can add substantial voltage
/// at the protected device. See also IEC 61000-4-2 for the standardized ESD
/// current waveform that makes clamp-path inductance relevant.
pub fn esd_return_path_readiness(
    board: &BoardModel,
    selected_layers: &[String],
    return_search_radius: f64,
) -> Vec<Violation> {
    let features = selected_copper_features(board, selected_layers);
    let return_features = features
        .iter()
        .copied()
        .filter(|feature| {
            feature
                .net
                .as_deref()
                .is_some_and(|net| looks_ground_net(net) || looks_chassis_net(net))
        })
        .collect::<Vec<_>>();
    log::trace!(
        "ESD return-path readiness: source={} features={} return_features={} radius={return_search_radius:.6}",
        board.source,
        features.len(),
        return_features.len()
    );
    let mut violations = Vec::new();

    for feature in features {
        let Some(net) = feature.net.as_deref() else {
            continue;
        };
        if !looks_esd_protection_net(net) {
            continue;
        }

        let has_return = return_features
            .iter()
            .filter(|return_feature| return_feature.layer == feature.layer)
            .any(|return_feature| {
                !std::ptr::eq(*return_feature, feature)
                    && distance(return_feature.location, feature.location) <= return_search_radius
            });
        if has_return {
            continue;
        }

        violations.push(Violation::new(
            "esd-return-path-readiness",
            Severity::Warning,
            vec![feature.layer.clone()],
            None,
            Vec::new(),
            vec![feature.location],
            Some(format!(
                "likely ESD protection net {net:?} has no parsed same-layer ground or chassis return copper within {return_search_radius:.6}; review TVS clamp loop inductance and discharge path"
            )),
        ));
    }

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

fn looks_high_voltage_net(net: &str) -> bool {
    let normalized = net.to_ascii_uppercase();
    let tokens = [
        "HV", "HIGHV", "MAINS", "LINE", "NEUTRAL", "LIVE", "VAC", "AC_L", "AC_N", "RECT", "BULK",
        "400V", "240V", "230V", "120V", "48V",
    ];

    tokens.iter().any(|token| normalized.contains(token))
}

fn looks_high_speed_net(net: &str) -> bool {
    let normalized = net.to_ascii_uppercase();
    let tokens = [
        "USB", "D+", "D-", "DP", "DM", "CLK", "CLOCK", "TX", "RX", "SERDES", "PCIE", "PCI", "MIPI",
        "LVDS", "HDMI", "ETH", "RGMII", "SGMII", "SATA", "CAN",
    ];

    tokens.iter().any(|token| normalized.contains(token))
}

fn looks_rf_or_antenna_net(net: &str) -> bool {
    let normalized = net.to_ascii_uppercase();
    let tokens = [
        "RF", "ANT", "ANTENNA", "GNSS", "GPS", "WIFI", "BT_", "BLE", "LTE",
    ];

    tokens.iter().any(|token| normalized.contains(token))
}

fn looks_chassis_net(net: &str) -> bool {
    let normalized = net.to_ascii_uppercase();
    matches!(
        normalized.as_str(),
        "CHASSIS" | "SHIELD" | "EARTH" | "PE" | "PROTECTIVE_EARTH"
    ) || normalized.ends_with("_SHIELD")
        || normalized.ends_with("-SHIELD")
        || normalized.contains("CHASSIS")
}

fn looks_ground_net(net: &str) -> bool {
    let normalized = net.to_ascii_uppercase();
    matches!(
        normalized.as_str(),
        "GND" | "GROUND" | "PGND" | "AGND" | "DGND"
    ) || normalized.ends_with("_GND")
        || normalized.ends_with("-GND")
}

fn looks_connector_net(net: &str) -> bool {
    let normalized = net.to_ascii_uppercase();
    let tokens = [
        "CONN",
        "CONNECTOR",
        "USB",
        "JACK",
        "SOCKET",
        "PLUG",
        "HEADER",
        "VBUS",
        "SHIELD",
        "CHASSIS",
        "CARD_EDGE",
        "EDGE_CONN",
    ];

    tokens.iter().any(|token| normalized.contains(token))
}

fn looks_connector_edge_rate_net(net: &str) -> bool {
    looks_connector_net(net) || looks_high_speed_net(net) || looks_rf_or_antenna_net(net)
}

fn looks_esd_protection_net(net: &str) -> bool {
    let normalized = net.to_ascii_uppercase();
    let tokens = [
        "ESD",
        "TVS",
        "CLAMP",
        "PROTECT",
        "PROTECTION",
        "SURGE",
        "SPARK",
        "TRANSIENT",
    ];

    tokens.iter().any(|token| normalized.contains(token))
}

fn distance(left: [f64; 2], right: [f64; 2]) -> f64 {
    let dx = left[0] - right[0];
    let dy = left[1] - right[1];
    (dx * dx + dy * dy).sqrt()
}

#[cfg(test)]
mod tests {
    use crate::LayerMetadata;
    use crate::geometry::{circle_polygon, polygons_to_sketch, rect_polygon};
    use crate::kicad::{BoardModel, CopperFeature, CopperKind};

    use super::{
        esd_protection_readiness, esd_return_path_readiness, high_voltage_edge_readiness,
        voltage_clearance_readiness,
    };

    #[test]
    fn esd_return_path_readiness_reports_clamp_without_ground_or_chassis() {
        let board = board_with_copper(vec![copper_disc(
            "USB_ESD_CLAMP",
            CopperKind::Pad,
            [0.0, 0.0],
            0.20,
        )]);

        let violations = esd_return_path_readiness(&board, &[], 0.50);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "esd-return-path-readiness");
    }

    #[test]
    fn esd_return_path_readiness_accepts_nearby_return_or_selected_out_layer() {
        let board = board_with_copper(vec![
            copper_disc_on_layer("USB_ESD_CLAMP", CopperKind::Pad, "B.Cu", [0.0, 0.0], 0.20),
            copper_disc_on_layer("CHASSIS", CopperKind::Pad, "B.Cu", [0.25, 0.0], 0.20),
        ]);

        assert!(esd_return_path_readiness(&board, &[], 0.50).is_empty());
        assert!(esd_return_path_readiness(&board, &["F.Cu".to_string()], 0.50).is_empty());
    }

    #[test]
    fn voltage_clearance_readiness_reports_likely_high_voltage_proximity() {
        let board = board_with_copper(vec![
            copper_rect("HV_BUS", CopperKind::Segment, "F.Cu", 0.0, 0.0, 1.0, 0.2),
            copper_rect("GND", CopperKind::Segment, "F.Cu", 1.1, 0.0, 2.0, 0.2),
        ]);

        let violations = voltage_clearance_readiness(&board, 0.30, &[], 1.0e-9);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "voltage-clearance-readiness");
    }

    #[test]
    fn high_voltage_edge_readiness_reports_hv_copper_near_outline() {
        let mut board = board_with_outline(square(0.0, 0.0, 10.0, 10.0));
        board.copper = vec![copper_rect(
            "HV_BUS",
            CopperKind::Segment,
            "F.Cu",
            0.2,
            4.0,
            1.0,
            4.4,
        )];

        let violations = high_voltage_edge_readiness(&board, &[], 0.80, 1.0e-9);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "high-voltage-edge-readiness");
    }

    #[test]
    fn esd_protection_readiness_reports_unprotected_edge_connector_net() {
        let mut board = board_with_outline(square(0.0, 0.0, 20.0, 20.0));
        board.copper = vec![copper_rect(
            "USB_D_P",
            CopperKind::Pad,
            "F.Cu",
            0.4,
            8.0,
            1.0,
            8.6,
        )];

        let violations = esd_protection_readiness(&board, &[], 1.0, 2.0);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "esd-protection-readiness");
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

    fn board_with_outline(outline: geo::Polygon<f64>) -> BoardModel {
        BoardModel {
            source: "test".to_string(),
            copper: Vec::new(),
            drills: Vec::new(),
            board_outline: Some(polygons_to_sketch(
                vec![outline],
                Some(LayerMetadata {
                    name: "outline".to_string(),
                }),
            )),
            panel_features: None,
        }
    }

    fn square(min_x: f64, min_y: f64, max_x: f64, max_y: f64) -> geo::Polygon<f64> {
        rect_polygon(
            [(min_x + max_x) / 2.0, (min_y + max_y) / 2.0],
            [max_x - min_x, max_y - min_y],
            0.0,
        )
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

    fn copper_disc(
        net: &str,
        kind: CopperKind,
        location: [f64; 2],
        diameter: f64,
    ) -> CopperFeature {
        copper_disc_on_layer(net, kind, "F.Cu", location, diameter)
    }

    fn copper_disc_on_layer(
        net: &str,
        kind: CopperKind,
        layer: &str,
        location: [f64; 2],
        diameter: f64,
    ) -> CopperFeature {
        CopperFeature {
            layer: layer.to_string(),
            net: Some(net.to_string()),
            kind,
            location,
            sketch: polygons_to_sketch(
                vec![circle_polygon(location, diameter / 2.0, 32)],
                Some(LayerMetadata {
                    name: "test disc".to_string(),
                }),
            ),
        }
    }
}
