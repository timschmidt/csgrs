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
use crate::checks::spatial::CopperSpatialIndex;
use crate::geometry::multipolygon_to_shapes;
use crate::kicad::{BoardModel, CopperFeature};
use crate::report::{Severity, Violation};

use super::outline::{axis_aligned_outline_rect, feature_bounds_inside_rect_margin};

/// Warn when likely high-voltage copper enters the board-edge review band.
///
/// This is a heuristic readiness check over parsed KiCad copper. It uses
/// net-name intent and the parsed board outline to make edge creepage and
/// clearance review visible before release documentation is assembled. On the
/// common rectangular-board path, it first applies the shared strict edge-band
/// bounds predicate from Ericson's broad/narrow-phase pattern in *Real-Time
/// Collision Detection* (2005), then keeps exact CSG as the reporting decision.
pub fn high_voltage_edge_readiness(
    board: &BoardModel,
    selected_layers: &[String],
    edge_clearance: f64,
    min_area: f64,
) -> Vec<Violation> {
    let Some(outline) = &board.board_outline else {
        return Vec::new();
    };
    let outline_rect = axis_aligned_outline_rect(outline);
    let allowed = outline.offset(-edge_clearance);
    let mut violations = Vec::new();
    let mut skipped_rect_inside = 0_usize;
    let mut exact_difference_count = 0_usize;

    for feature in selected_copper_features(board, selected_layers) {
        let Some(net) = &feature.net else {
            continue;
        };
        if !looks_high_voltage_net(net) {
            continue;
        }
        if outline_rect
            .as_ref()
            .is_some_and(|rect| feature_bounds_inside_rect_margin(feature, rect, edge_clearance))
        {
            skipped_rect_inside += 1;
            continue;
        }

        exact_difference_count += 1;
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

    log::trace!(
        "high-voltage edge readiness: source={} selected_layers={} outline_fast_path={} skipped_rect_inside={} exact_difference_checks={} edge_clearance={edge_clearance:.6} violations={}",
        board.source,
        selected_layers.len(),
        outline_rect.is_some(),
        skipped_rect_inside,
        exact_difference_count,
        violations.len()
    );

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
    let feature_index = CopperSpatialIndex::new(&features, clearance);
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
        "voltage-clearance readiness: source={} features={} high_voltage_features={} buckets={} clearance={clearance:.6}",
        board.source,
        features.len(),
        high_voltage_indices.len(),
        feature_index.bucket_count()
    );
    let mut violations = Vec::new();
    let mut candidate_count = 0_usize;

    for &left_index in &high_voltage_indices {
        let left = features[left_index];
        for right_index in feature_index.same_layer_near_feature(left, clearance) {
            candidate_count += 1;
            let right = features[right_index];
            if left_index == right_index {
                continue;
            }
            if left.net.is_none() || left.net == right.net {
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

    log::trace!(
        "voltage-clearance readiness: source={} candidate_pairs={} violations={}",
        board.source,
        candidate_count,
        violations.len()
    );

    violations
}

/// Warn when likely high-voltage copper is close to protective earth/chassis.
///
/// General voltage spacing already reports high-voltage proximity to unrelated
/// copper. This companion gives PE/chassis boundaries their own review item so
/// safety intent does not get lost among ordinary spacing findings. IPC-2221B
/// frames electrical clearance as a board-design constraint, and Lin and Canny,
/// "A Fast Algorithm for Incremental Distance Calculation", IEEE ICRA, 1991,
/// motivates the broad/narrow geometry pattern used here: cheap bounding-box
/// rejection first, exact offset/intersection or boundary distance second.
pub fn protective_earth_spacing_readiness(
    board: &BoardModel,
    selected_layers: &[String],
    clearance: f64,
    min_area: f64,
) -> Vec<Violation> {
    let features = selected_copper_features(board, selected_layers);
    let high_voltage = features
        .iter()
        .copied()
        .filter(|feature| feature.net.as_deref().is_some_and(looks_high_voltage_net))
        .collect::<Vec<_>>();
    let protective = features
        .iter()
        .copied()
        .filter(|feature| {
            feature
                .net
                .as_deref()
                .is_some_and(looks_protective_earth_net)
        })
        .collect::<Vec<_>>();
    let protective_index = CopperSpatialIndex::new(&protective, clearance);
    log::trace!(
        "protective-earth spacing readiness: source={} high_voltage={} protective={} buckets={} clearance={clearance:.6}",
        board.source,
        high_voltage.len(),
        protective.len(),
        protective_index.bucket_count()
    );

    let mut violations = Vec::new();
    let mut candidate_count = 0_usize;
    for hv in high_voltage {
        for pe_index in protective_index.same_layer_near_feature(hv, clearance) {
            candidate_count += 1;
            let pe = protective[pe_index];
            if hv.net == pe.net {
                continue;
            }
            if !sketches_within_clearance(&hv.sketch, &pe.sketch, clearance) {
                continue;
            }

            let overlap = hv.sketch.offset(clearance).intersection(&pe.sketch);
            let shapes = multipolygon_to_shapes(&overlap.to_multipolygon(), min_area);
            let fallback_hit = shapes.is_empty()
                && polygon_boundary_distance(
                    &hv.sketch.to_multipolygon(),
                    &pe.sketch.to_multipolygon(),
                ) <= clearance;
            if shapes.is_empty() && !fallback_hit {
                continue;
            }

            violations.push(Violation::new(
                "protective-earth-spacing-readiness",
                Severity::Warning,
                vec![hv.layer.clone()],
                None,
                shapes,
                if fallback_hit {
                    vec![hv.location, pe.location]
                } else {
                    Vec::new()
                },
                Some(format!(
                    "likely high-voltage net {:?} is within {clearance:.6} of protective/chassis net {:?}; review protective-earth clearance, creepage, slots, coating, and safety documentation",
                    hv.net, pe.net
                )),
            ));
        }
    }

    log::trace!(
        "protective-earth spacing readiness: source={} candidate_pairs={} violations={}",
        board.source,
        candidate_count,
        violations.len()
    );

    violations
}

/// Warn when ordinary copper crowds likely surge/fuse/MOV protection copper.
///
/// Surge protection devices deliberately connect to hazardous or chassis domains,
/// so this check does not flag high-voltage, ground, PE/chassis, same-net, or
/// other surge-protection copper. It reports unrelated ordinary copper inside
/// the local keepout band around likely MOV, GDT, spark-gap, TVS, or fuse nets.
/// IEC 61000-4-5 defines the surge-immunity context behind this review, while
/// Lin and Canny, "A Fast Algorithm for Incremental Distance Calculation", IEEE
/// ICRA, 1991, motivates the broad/narrow geometry pattern used below.
pub fn surge_protection_keepout_readiness(
    board: &BoardModel,
    selected_layers: &[String],
    keepout: f64,
    min_area: f64,
) -> Vec<Violation> {
    let features = selected_copper_features(board, selected_layers);
    let surge = features
        .iter()
        .copied()
        .filter(|feature| {
            feature
                .net
                .as_deref()
                .is_some_and(looks_surge_protection_net)
        })
        .collect::<Vec<_>>();
    let feature_index = CopperSpatialIndex::new(&features, keepout);
    log::trace!(
        "surge protection keepout readiness: source={} features={} surge_features={} buckets={} keepout={keepout:.6}",
        board.source,
        features.len(),
        surge.len(),
        feature_index.bucket_count()
    );

    let mut violations = Vec::new();
    let mut candidate_count = 0_usize;
    for source in surge {
        for neighbor_index in feature_index.same_layer_near_feature(source, keepout) {
            candidate_count += 1;
            let neighbor = features[neighbor_index];
            if std::ptr::eq(source, neighbor) {
                continue;
            }
            if source.net == neighbor.net
                || neighbor
                    .net
                    .as_deref()
                    .is_some_and(allowed_surge_neighbor_net)
            {
                continue;
            }
            if !sketches_within_clearance(&source.sketch, &neighbor.sketch, keepout) {
                continue;
            }

            let overlap = source.sketch.offset(keepout).intersection(&neighbor.sketch);
            let shapes = multipolygon_to_shapes(&overlap.to_multipolygon(), min_area);
            let fallback_hit = shapes.is_empty()
                && polygon_boundary_distance(
                    &source.sketch.to_multipolygon(),
                    &neighbor.sketch.to_multipolygon(),
                ) <= keepout;
            if shapes.is_empty() && !fallback_hit {
                continue;
            }

            violations.push(Violation::new(
                "surge-protection-keepout-readiness",
                Severity::Warning,
                vec![source.layer.clone()],
                None,
                shapes,
                if fallback_hit {
                    vec![source.location, neighbor.location]
                } else {
                    Vec::new()
                },
                Some(format!(
                    "likely surge/fuse protection net {:?} has unrelated copper from net {:?} inside keepout {keepout:.6}; review MOV, GDT, spark-gap, fuse, and surge-current isolation geometry",
                    source.net, neighbor.net
                )),
            ));
        }
    }

    log::trace!(
        "surge protection keepout readiness: source={} candidate_pairs={} violations={}",
        board.source,
        candidate_count,
        violations.len()
    );

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
    let protection_index = CopperSpatialIndex::new(&protection_features, protection_search_radius);
    log::trace!(
        "ESD protection readiness: source={} features={} protection_features={} buckets={} edge_distance={edge_distance:.6} protection_radius={protection_search_radius:.6}",
        board.source,
        features.len(),
        protection_features.len(),
        protection_index.bucket_count()
    );
    let mut violations = Vec::new();
    let mut edge_candidate_count = 0_usize;
    let mut protection_candidate_count = 0_usize;

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
        edge_candidate_count += 1;

        let protection_candidates = protection_index.same_layer_centers_within(
            feature.location,
            &feature.layer,
            protection_search_radius,
        );
        protection_candidate_count += protection_candidates.len();
        let has_protection = protection_candidates
            .into_iter()
            .any(|protection_index| !std::ptr::eq(protection_features[protection_index], feature));
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

    log::trace!(
        "ESD protection readiness: source={} edge_candidates={} protection_candidates={} violations={}",
        board.source,
        edge_candidate_count,
        protection_candidate_count,
        violations.len()
    );

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
    let return_index = CopperSpatialIndex::new(&return_features, return_search_radius);
    log::trace!(
        "ESD return-path readiness: source={} features={} return_features={} buckets={} radius={return_search_radius:.6}",
        board.source,
        features.len(),
        return_features.len(),
        return_index.bucket_count()
    );
    let mut violations = Vec::new();
    let mut esd_count = 0_usize;
    let mut return_candidate_count = 0_usize;

    for feature in features {
        let Some(net) = feature.net.as_deref() else {
            continue;
        };
        if !looks_esd_protection_net(net) {
            continue;
        }
        esd_count += 1;

        let return_candidates = return_index.same_layer_centers_within(
            feature.location,
            &feature.layer,
            return_search_radius,
        );
        return_candidate_count += return_candidates.len();
        let has_return = return_candidates
            .into_iter()
            .any(|return_index| !std::ptr::eq(return_features[return_index], feature));
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

    log::trace!(
        "ESD return-path readiness: source={} esd_features={} return_candidates={} violations={}",
        board.source,
        esd_count,
        return_candidate_count,
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

fn looks_protective_earth_net(net: &str) -> bool {
    let normalized = net.to_ascii_uppercase();
    matches!(
        normalized.as_str(),
        "PE" | "EARTH" | "PROTECTIVE_EARTH" | "PROTECTIVE-EARTH" | "SAFETY_GND"
    ) || normalized.contains("CHASSIS")
        || normalized.contains("SHIELD")
        || normalized.ends_with("_PE")
        || normalized.ends_with("-PE")
        || normalized.contains("EARTH")
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

fn looks_surge_protection_net(net: &str) -> bool {
    let normalized = net.to_ascii_uppercase();
    let tokens = [
        "MOV", "VARISTOR", "GDT", "SPARK", "SPARKGAP", "SURGE", "FUSE", "FUSED", "TVS", "PTC",
    ];

    tokens.iter().any(|token| normalized.contains(token))
}

fn allowed_surge_neighbor_net(net: &str) -> bool {
    looks_high_voltage_net(net)
        || looks_ground_net(net)
        || looks_protective_earth_net(net)
        || looks_surge_protection_net(net)
}

#[cfg(test)]
mod tests {
    use std::time::Instant;

    use crate::LayerMetadata;
    use crate::geometry::{circle_polygon, polygons_to_sketch, rect_polygon};
    use crate::kicad::{BoardModel, CopperFeature, CopperKind};

    use super::{
        esd_protection_readiness, esd_return_path_readiness, high_voltage_edge_readiness,
        protective_earth_spacing_readiness, surge_protection_keepout_readiness,
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
    fn voltage_clearance_readiness_culls_sparse_copper_fields() {
        let mut copper = sparse_rects("GPIO", 2_000, 100.0);
        copper.push(copper_rect(
            "HV_BUS",
            CopperKind::Segment,
            "F.Cu",
            0.0,
            0.0,
            1.0,
            0.2,
        ));
        copper.push(copper_rect(
            "GND_NEAR",
            CopperKind::Segment,
            "F.Cu",
            1.1,
            0.0,
            2.0,
            0.2,
        ));
        copper.push(copper_rect(
            "GND_OTHER_LAYER",
            CopperKind::Segment,
            "B.Cu",
            1.1,
            0.0,
            2.0,
            0.2,
        ));
        let board = board_with_copper(copper);

        let violations = voltage_clearance_readiness(&board, 0.30, &[], 1.0e-9);

        assert_eq!(violations.len(), 1);
        assert!(violations[0].message.as_ref().is_some_and(|message| {
            message.contains("GND_NEAR") && !message.contains("GND_OTHER_LAYER")
        }));
    }

    #[test]
    fn protective_earth_spacing_readiness_reports_hv_near_chassis() {
        let board = board_with_copper(vec![
            copper_rect("HV_BUS", CopperKind::Segment, "F.Cu", 0.0, 0.0, 1.0, 0.2),
            copper_rect("PE", CopperKind::Segment, "F.Cu", 1.2, 0.0, 2.0, 0.2),
        ]);

        let violations = protective_earth_spacing_readiness(&board, &[], 0.30, 1.0e-9);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "protective-earth-spacing-readiness");
    }

    #[test]
    fn protective_earth_spacing_readiness_allows_distant_or_non_protective_copper() {
        let distant = board_with_copper(vec![
            copper_rect("HV_BUS", CopperKind::Segment, "F.Cu", 0.0, 0.0, 1.0, 0.2),
            copper_rect("PE", CopperKind::Segment, "F.Cu", 2.0, 0.0, 3.0, 0.2),
        ]);
        let ordinary = board_with_copper(vec![
            copper_rect("HV_BUS", CopperKind::Segment, "F.Cu", 0.0, 0.0, 1.0, 0.2),
            copper_rect("GPIO", CopperKind::Segment, "F.Cu", 1.2, 0.0, 2.0, 0.2),
        ]);

        assert!(protective_earth_spacing_readiness(&distant, &[], 0.30, 1.0e-9).is_empty());
        assert!(protective_earth_spacing_readiness(&ordinary, &[], 0.30, 1.0e-9).is_empty());
    }

    #[test]
    fn protective_earth_spacing_readiness_respects_selected_layers() {
        let board = board_with_copper(vec![
            copper_rect("HV_BUS", CopperKind::Segment, "B.Cu", 0.0, 0.0, 1.0, 0.2),
            copper_rect("CHASSIS", CopperKind::Segment, "B.Cu", 1.2, 0.0, 2.0, 0.2),
        ]);

        assert!(
            protective_earth_spacing_readiness(&board, &["F.Cu".to_string()], 0.30, 1.0e-9)
                .is_empty()
        );
        assert_eq!(
            protective_earth_spacing_readiness(&board, &["B.Cu".to_string()], 0.30, 1.0e-9).len(),
            1
        );
    }

    #[test]
    fn protective_earth_spacing_readiness_culls_sparse_large_cases() {
        let mut copper = vec![copper_rect(
            "HV_BUS",
            CopperKind::Segment,
            "F.Cu",
            0.0,
            0.0,
            1.0,
            0.2,
        )];
        for index in 0..500 {
            let x = 100.0 + index as f64 * 2.0;
            copper.push(copper_rect(
                "CHASSIS",
                CopperKind::Segment,
                "F.Cu",
                x,
                0.0,
                x + 1.0,
                0.2,
            ));
        }
        let board = board_with_copper(copper);

        let violations = protective_earth_spacing_readiness(&board, &[], 0.30, 1.0e-9);

        assert!(violations.is_empty());
    }

    #[test]
    fn surge_protection_keepout_readiness_reports_unrelated_copper() {
        let board = board_with_copper(vec![
            copper_rect("MOV_LINE", CopperKind::Pad, "F.Cu", 0.0, 0.0, 0.5, 0.5),
            copper_rect("GPIO", CopperKind::Segment, "F.Cu", 0.7, 0.0, 1.2, 0.5),
        ]);

        let violations = surge_protection_keepout_readiness(&board, &[], 0.30, 1.0e-9);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "surge-protection-keepout-readiness");
    }

    #[test]
    fn surge_protection_keepout_readiness_allows_expected_surge_neighbors() {
        let board = board_with_copper(vec![
            copper_rect("MOV_LINE", CopperKind::Pad, "F.Cu", 0.0, 0.0, 0.5, 0.5),
            copper_rect("HV_BUS", CopperKind::Segment, "F.Cu", 0.7, 0.0, 1.2, 0.5),
            copper_rect("PE", CopperKind::Segment, "F.Cu", -0.7, 0.0, -0.2, 0.5),
            copper_rect("GND", CopperKind::Segment, "F.Cu", 0.0, 0.7, 0.5, 1.2),
        ]);

        let violations = surge_protection_keepout_readiness(&board, &[], 0.30, 1.0e-9);

        assert!(violations.is_empty());
    }

    #[test]
    fn surge_protection_keepout_readiness_respects_selected_layers() {
        let board = board_with_copper(vec![
            copper_rect("FUSE_IN", CopperKind::Pad, "B.Cu", 0.0, 0.0, 0.5, 0.5),
            copper_rect("GPIO", CopperKind::Segment, "B.Cu", 0.7, 0.0, 1.2, 0.5),
        ]);

        assert!(
            surge_protection_keepout_readiness(&board, &["F.Cu".to_string()], 0.30, 1.0e-9)
                .is_empty()
        );
        assert_eq!(
            surge_protection_keepout_readiness(&board, &["B.Cu".to_string()], 0.30, 1.0e-9).len(),
            1
        );
    }

    #[test]
    fn surge_protection_keepout_readiness_culls_sparse_large_cases() {
        let mut copper = vec![copper_rect(
            "MOV_LINE",
            CopperKind::Pad,
            "F.Cu",
            0.0,
            0.0,
            0.5,
            0.5,
        )];
        for index in 0..500 {
            let x = 100.0 + index as f64 * 2.0;
            copper.push(copper_rect(
                "GPIO",
                CopperKind::Segment,
                "F.Cu",
                x,
                0.0,
                x + 0.5,
                0.5,
            ));
        }
        let board = board_with_copper(copper);

        let violations = surge_protection_keepout_readiness(&board, &[], 0.30, 1.0e-9);

        assert!(violations.is_empty());
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
    fn high_voltage_edge_readiness_culls_rectangular_interior_copper_fields() {
        let mut board = board_with_outline(square(0.0, 0.0, 100.0, 100.0));
        board.copper = (0..2_000)
            .map(|index| {
                let x = 5.0 + (index % 50) as f64 * 1.5;
                let y = 5.0 + (index / 50) as f64 * 1.5;
                copper_rect(
                    &format!("HV_BUS_{index}"),
                    CopperKind::Segment,
                    "F.Cu",
                    x,
                    y,
                    x + 0.5,
                    y + 0.2,
                )
            })
            .collect();
        board.copper.push(copper_rect(
            "MAINS_L",
            CopperKind::Segment,
            "F.Cu",
            0.20,
            50.0,
            1.0,
            50.2,
        ));

        let started = Instant::now();
        let violations = high_voltage_edge_readiness(&board, &[], 0.80, 1.0e-9);

        assert_eq!(violations.len(), 1);
        assert!(
            started.elapsed().as_secs_f64() < 2.0,
            "high-voltage edge review should skip rectangular interior copper before exact difference"
        );
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

    #[test]
    fn esd_protection_readiness_culls_sparse_protection_fields() {
        let mut board = board_with_outline(square(0.0, 0.0, 20.0, 20.0));
        board.copper = sparse_rects("USB_TVS", 2_000, 100.0);
        board.copper.push(copper_rect(
            "USB_D_P",
            CopperKind::Pad,
            "F.Cu",
            0.4,
            8.0,
            1.0,
            8.6,
        ));
        board.copper.push(copper_rect(
            "USB_TVS_NEAR",
            CopperKind::Pad,
            "F.Cu",
            1.4,
            8.0,
            2.0,
            8.6,
        ));

        assert!(esd_protection_readiness(&board, &[], 1.0, 2.0).is_empty());
    }

    #[test]
    fn esd_return_path_readiness_culls_sparse_return_fields() {
        let mut copper = sparse_rects("GND", 2_000, 100.0);
        copper.push(copper_disc(
            "USB_ESD_CLAMP",
            CopperKind::Pad,
            [0.0, 0.0],
            0.20,
        ));
        copper.push(copper_disc("CHASSIS", CopperKind::Pad, [0.25, 0.0], 0.20));
        let board = board_with_copper(copper);

        assert!(esd_return_path_readiness(&board, &[], 0.50).is_empty());
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

    fn sparse_rects(prefix: &str, count: usize, offset_x: f64) -> Vec<CopperFeature> {
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
                    x + 1.0,
                    y + 0.2,
                )
            })
            .collect()
    }
}
