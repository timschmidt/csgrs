//! Thermal and heat-spreading readiness checks.
//!
//! These checks operate on parsed KiCad copper and drill data. They cover
//! thermal-relief intent, thermal-via count and spread, exposed-pad via support,
//! hot-copper spacing, and mechanical keepouts around likely hot features.
//!
//! Reliability note: these checks are not thermal simulation. Net-name heat
//! inference, copper-area proxies, and via-count rules are suspect near heat
//! spreaders, enclosures, unusual airflow, and package-specific requirements.

use csgrs::csg::CSG;
use geo::{Area, BoundingRect};

use crate::checks::distance::polygon_boundary_distance;
use crate::checks::spatial::CopperSpatialIndex;
use crate::geometry::{circle_polygon, multipolygon_to_shapes, polygons_to_sketch};
use crate::kicad::{BoardModel, CopperFeature, CopperKind, DrillFeature};
use crate::report::{Severity, Violation};
use crate::{LayerMetadata, PcbSketch};

/// Run the `thermal_relief_readiness` design-readiness check or report helper.
pub fn thermal_relief_readiness(
    board: &BoardModel,
    selected_layers: &[String],
    min_area: f64,
) -> Vec<Violation> {
    let features = selected_copper_features(board, selected_layers);
    let anchors = features
        .iter()
        .filter(|feature| matches!(feature.kind, CopperKind::Pad | CopperKind::Via))
        .copied()
        .collect::<Vec<_>>();
    let zones = features
        .iter()
        .filter(|feature| feature.kind == CopperKind::Zone)
        .copied()
        .collect::<Vec<_>>();
    log::trace!(
        "thermal-relief readiness: source={} anchors={} zones={}",
        board.source,
        anchors.len(),
        zones.len()
    );
    let mut violations = Vec::new();

    for anchor in anchors {
        for zone in &zones {
            if anchor.layer != zone.layer || anchor.net.is_none() || anchor.net != zone.net {
                continue;
            }

            let overlap = anchor.sketch.intersection(&zone.sketch);
            let shapes = multipolygon_to_shapes(&overlap.to_multipolygon(), min_area);
            if shapes.is_empty() {
                continue;
            }

            violations.push(Violation::new(
                "thermal-relief-readiness",
                Severity::Warning,
                vec![anchor.layer.clone()],
                None,
                shapes,
                vec![anchor.location, zone.location],
                Some(format!(
                    "same-net {:?} copper intersects a copper zone; confirm thermal relief or intentional direct plane connection",
                    anchor.kind
                )),
            ));
        }
    }

    violations
}

/// Run the `thermal_via_readiness` design-readiness check or report helper.
pub fn thermal_via_readiness(
    board: &BoardModel,
    selected_layers: &[String],
    minimum_vias: usize,
    anchor_tolerance: f64,
) -> Vec<Violation> {
    let features = selected_copper_features(board, selected_layers);
    log::trace!(
        "thermal-via readiness: source={} features={} minimum_vias={minimum_vias}",
        board.source,
        features.len()
    );
    let mut violations = Vec::new();

    for zone in features
        .iter()
        .filter(|feature| feature.kind == CopperKind::Zone)
    {
        let Some(net) = &zone.net else {
            continue;
        };
        if !looks_high_current_net(net) {
            continue;
        }

        let via_count = thermal_zone_vias(&features, zone, anchor_tolerance).len();
        if via_count >= minimum_vias {
            continue;
        }

        violations.push(Violation::new(
            "thermal-via-readiness",
            Severity::Warning,
            vec![zone.layer.clone()],
            None,
            Vec::new(),
            vec![zone.location],
            Some(format!(
                "likely power or thermal zone {net} has {via_count} parsed same-net via(s), below review threshold {minimum_vias}"
            )),
        ));
    }

    violations
}

/// Warn when a likely thermal via set exists but is too clustered to distribute
/// heat across the local copper area.
///
/// This is a geometry readiness check, not a thermal solver. It reports cases
/// where a power/thermal zone has the requested number of same-net vias, but the
/// via field has a small maximum span. Thermal-via and heat-spreader geometry is
/// strongly tied to heat distribution; see Hollstein, Yang, and Weide-Zaage,
/// "Thermal analysis of the design parameters of a QFN package soldered on a
/// PCB using a simulation approach," *Microelectronics Reliability* 120 (2021),
/// article 114118, <https://doi.org/10.1016/j.microrel.2021.114118>, which
/// varies thermal via count and distribution among influential PCB parameters.
pub fn thermal_via_distribution_readiness(
    board: &BoardModel,
    selected_layers: &[String],
    minimum_vias: usize,
    minimum_spread: f64,
    anchor_tolerance: f64,
) -> Vec<Violation> {
    if minimum_vias < 2 || minimum_spread <= 0.0 {
        return Vec::new();
    }

    let features = selected_copper_features(board, selected_layers);
    log::trace!(
        "thermal-via distribution readiness: source={} features={} minimum_vias={} minimum_spread={minimum_spread:.6}",
        board.source,
        features.len(),
        minimum_vias
    );
    let mut violations = Vec::new();

    for zone in features
        .iter()
        .filter(|feature| feature.kind == CopperKind::Zone)
    {
        let Some(net) = zone.net.as_deref() else {
            continue;
        };
        if !looks_high_current_net(net) {
            continue;
        }

        let vias = thermal_zone_vias(&features, zone, anchor_tolerance);
        if vias.len() < minimum_vias {
            continue;
        }
        let spread = maximum_location_spread(&vias);
        if spread >= minimum_spread {
            continue;
        }

        violations.push(Violation::new(
            "thermal-via-distribution-readiness",
            Severity::Warning,
            vec![zone.layer.clone()],
            None,
            Vec::new(),
            vias.iter().map(|via| via.location).collect(),
            Some(format!(
                "likely power or thermal zone {net} has {} parsed same-net vias but via-field spread {spread:.6} is below {minimum_spread:.6}; review thermal via distribution and heat spreading",
                vias.len()
            )),
        ));
    }

    violations
}

/// Run the `thermal_pad_via_readiness` design-readiness check or report helper.
pub fn thermal_pad_via_readiness(
    board: &BoardModel,
    selected_layers: &[String],
    minimum_pad_dimension: f64,
) -> Vec<Violation> {
    let features = selected_copper_features(board, selected_layers);
    let vias = features
        .iter()
        .copied()
        .filter(|feature| feature.kind == CopperKind::Via)
        .collect::<Vec<_>>();
    log::trace!(
        "thermal-pad via readiness: source={} features={} vias={}",
        board.source,
        features.len(),
        vias.len()
    );
    let mut violations = Vec::new();

    for pad in features {
        if pad.kind != CopperKind::Pad {
            continue;
        }
        let Some(net) = pad.net.as_deref() else {
            continue;
        };
        if !looks_ground_net(net) && !looks_high_current_net(net) {
            continue;
        }
        let Some((min_dimension, max_dimension)) = bounding_dimensions(&pad.sketch) else {
            continue;
        };
        if min_dimension < minimum_pad_dimension {
            continue;
        }
        if max_dimension / min_dimension > 3.0 {
            continue;
        }

        let has_same_net_via = vias
            .iter()
            .filter(|via| via.layer == pad.layer)
            .filter(|via| via.net == pad.net)
            .any(|via| {
                !multipolygon_to_shapes(
                    &via.sketch.intersection(&pad.sketch).to_multipolygon(),
                    1.0e-9,
                )
                .is_empty()
            });
        if has_same_net_via {
            continue;
        }

        violations.push(Violation::new(
            "thermal-pad-via-readiness",
            Severity::Warning,
            vec![pad.layer.clone()],
            None,
            Vec::new(),
            vec![pad.location],
            Some(format!(
                "large likely thermal pad on net {net:?} has no parsed same-net via in pad; review exposed-pad thermal via array, fill, tent, and solder-voiding intent"
            )),
        ));
    }

    violations
}

/// Run the `thermal_copper_area_readiness` design-readiness check or report helper.
pub fn thermal_copper_area_readiness(
    board: &BoardModel,
    selected_layers: &[String],
    search_radius: f64,
) -> Vec<Violation> {
    let features = selected_copper_features(board, selected_layers);
    let same_net_zones = features
        .iter()
        .copied()
        .filter(|feature| feature.kind == CopperKind::Zone)
        .filter(|feature| {
            feature
                .net
                .as_deref()
                .is_some_and(looks_thermal_or_power_net)
        })
        .collect::<Vec<_>>();
    let zone_index = CopperSpatialIndex::new(&same_net_zones, search_radius);
    log::trace!(
        "thermal copper-area readiness: source={} features={} zones={} buckets={} search_radius={search_radius:.6}",
        board.source,
        features.len(),
        same_net_zones.len(),
        zone_index.bucket_count()
    );
    let mut violations = Vec::new();
    let mut thermal_feature_count = 0_usize;
    let mut candidate_count = 0_usize;

    for feature in features {
        if feature.kind == CopperKind::Zone {
            continue;
        }
        let Some(net) = feature.net.as_deref() else {
            continue;
        };
        if !looks_thermal_or_power_net(net) {
            continue;
        }
        thermal_feature_count += 1;

        let zone_candidates =
            zone_index.same_layer_centers_within(feature.location, &feature.layer, search_radius);
        candidate_count += zone_candidates.len();
        let has_nearby_same_net_zone = zone_candidates
            .into_iter()
            .any(|zone_index| same_net_zones[zone_index].net == feature.net);
        if has_nearby_same_net_zone {
            continue;
        }

        violations.push(Violation::new(
            "thermal-copper-area-readiness",
            Severity::Warning,
            vec![feature.layer.clone()],
            None,
            Vec::new(),
            vec![feature.location],
            Some(format!(
                "likely heat or power feature on net {net:?} has no parsed same-net copper zone within {search_radius:.6}; review copper area for heat spreading and current return"
            )),
        ));
    }

    log::trace!(
        "thermal copper-area readiness: source={} thermal_features={} zone_candidates={} violations={}",
        board.source,
        thermal_feature_count,
        candidate_count,
        violations.len()
    );

    violations
}

/// Run the `hot_component_spacing_readiness` design-readiness check or report helper.
pub fn hot_component_spacing_readiness(
    board: &BoardModel,
    selected_layers: &[String],
    spacing: f64,
    min_area: f64,
) -> Vec<Violation> {
    let features = selected_copper_features(board, selected_layers);
    let feature_index = CopperSpatialIndex::new(&features, spacing);
    let hot_features = features
        .iter()
        .copied()
        .filter(|feature| {
            feature.net.as_deref().is_some_and(looks_hot_component_net)
                && matches!(feature.kind, CopperKind::Pad | CopperKind::Zone)
        })
        .collect::<Vec<_>>();
    log::trace!(
        "hot-component spacing readiness: source={} hot_features={} features={} buckets={} spacing={spacing:.6}",
        board.source,
        hot_features.len(),
        features.len(),
        feature_index.bucket_count()
    );
    let mut violations = Vec::new();
    let mut candidate_count = 0_usize;

    for hot in hot_features {
        for neighbor_index in feature_index.same_layer_near_feature(hot, spacing) {
            candidate_count += 1;
            let neighbor = features[neighbor_index];
            if std::ptr::eq(hot, neighbor) {
                continue;
            }
            if hot.net == neighbor.net || neighbor.net.as_deref().is_some_and(looks_ground_net) {
                continue;
            }
            if !sketches_within_clearance(&hot.sketch, &neighbor.sketch, spacing) {
                continue;
            }

            let overlap = hot.sketch.offset(spacing).intersection(&neighbor.sketch);
            let shapes = multipolygon_to_shapes(&overlap.to_multipolygon(), min_area);
            let fallback_hit = shapes.is_empty()
                && polygon_boundary_distance(
                    &hot.sketch.to_multipolygon(),
                    &neighbor.sketch.to_multipolygon(),
                ) <= spacing;
            if shapes.is_empty() && !fallback_hit {
                continue;
            }

            violations.push(Violation::new(
                "hot-component-spacing-readiness",
                Severity::Warning,
                vec![hot.layer.clone()],
                None,
                shapes,
                vec![hot.location, neighbor.location],
                Some(format!(
                    "likely hot feature {:?} is within thermal spacing {spacing:.6} of neighboring net {:?}; review heat spreading, derating, and component placement",
                    hot.net, neighbor.net
                )),
            ));
        }
    }

    log::trace!(
        "hot-component spacing readiness: source={} candidate_pairs={} violations={}",
        board.source,
        candidate_count,
        violations.len()
    );

    violations
}

/// Run the `thermal_mechanical_keepout_readiness` design-readiness check or report helper.
pub fn thermal_mechanical_keepout_readiness(
    board: &BoardModel,
    extra_drills: &[DrillFeature],
    selected_layers: &[String],
    keepout: f64,
    min_area: f64,
) -> Vec<Violation> {
    let mut mechanical_drills = board
        .drills
        .iter()
        .chain(extra_drills.iter())
        .filter(|drill| !drill.plated)
        .collect::<Vec<_>>();
    mechanical_drills.sort_by(|left, right| {
        left.location[0]
            .total_cmp(&right.location[0])
            .then(left.location[1].total_cmp(&right.location[1]))
            .then(left.diameter.total_cmp(&right.diameter))
    });

    let hot_features = selected_copper_features(board, selected_layers)
        .into_iter()
        .filter(|feature| feature.net.as_deref().is_some_and(looks_hot_component_net))
        .collect::<Vec<_>>();
    let hot_index = CopperSpatialIndex::new(&hot_features, keepout);
    log::trace!(
        "thermal mechanical-keepout readiness: source={} hot_features={} buckets={} mechanical_drills={} keepout={keepout:.6}",
        board.source,
        hot_features.len(),
        hot_index.bucket_count(),
        mechanical_drills.len()
    );
    let mut violations = Vec::new();
    let mut candidate_count = 0_usize;

    for drill in mechanical_drills {
        let keepout_radius = drill.diameter / 2.0 + keepout;
        let keepout_sketch = polygons_to_sketch(
            vec![circle_polygon(drill.location, keepout_radius, 32)],
            Some(LayerMetadata {
                name: "thermal mechanical keepout".to_string(),
            }),
        );

        for hot_index in hot_index.all_layers_near_circle(drill.location, keepout_radius) {
            candidate_count += 1;
            let hot = hot_features[hot_index];
            if !feature_may_touch_circle(hot, drill.location, keepout_radius) {
                continue;
            }
            let overlap = keepout_sketch.intersection(&hot.sketch);
            let shapes = multipolygon_to_shapes(&overlap.to_multipolygon(), min_area);
            let fallback_hit = shapes.is_empty()
                && polygon_boundary_distance(
                    &keepout_sketch.to_multipolygon(),
                    &hot.sketch.to_multipolygon(),
                ) <= 1.0e-9;
            if shapes.is_empty() && !fallback_hit {
                continue;
            }

            violations.push(Violation::new(
                "thermal-mechanical-keepout-readiness",
                Severity::Warning,
                vec![hot.layer.clone()],
                None,
                shapes,
                vec![drill.location, hot.location],
                Some(format!(
                    "likely hot feature {:?} is inside mechanical thermal keepout {keepout:.6}; review heatsink, standoff, screw, chassis, and airflow clearance",
                    hot.net
                )),
            ));
        }
    }

    log::trace!(
        "thermal mechanical-keepout readiness: source={} candidate_pairs={} violations={}",
        board.source,
        candidate_count,
        violations.len()
    );

    violations
}

fn thermal_zone_vias<'a>(
    features: &[&'a CopperFeature],
    zone: &CopperFeature,
    anchor_tolerance: f64,
) -> Vec<&'a CopperFeature> {
    features
        .iter()
        .copied()
        .filter(|feature| {
            feature.kind == CopperKind::Via
                && feature.net == zone.net
                && copper_features_touch(feature, zone, anchor_tolerance)
        })
        .collect()
}

fn maximum_location_spread(features: &[&CopperFeature]) -> f64 {
    let mut spread: f64 = 0.0;
    for left_index in 0..features.len() {
        for right in features.iter().skip(left_index + 1) {
            spread = spread.max(distance(features[left_index].location, right.location));
        }
    }
    spread
}

fn copper_features_touch(left: &CopperFeature, right: &CopperFeature, tolerance: f64) -> bool {
    left.sketch
        .intersection(&right.sketch)
        .to_multipolygon()
        .0
        .iter()
        .any(|polygon| polygon.unsigned_area() > 0.0)
        || polygon_boundary_distance(
            &left.sketch.to_multipolygon(),
            &right.sketch.to_multipolygon(),
        ) <= tolerance
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

fn bounding_dimensions(sketch: &PcbSketch) -> Option<(f64, f64)> {
    sketch.geometry.bounding_rect().map(|bounds| {
        let width = bounds.max().x - bounds.min().x;
        let height = bounds.max().y - bounds.min().y;
        (width.min(height), width.max(height))
    })
}

fn sketches_within_clearance(left: &PcbSketch, right: &PcbSketch, clearance: f64) -> bool {
    let Some(left_bounds) = left.geometry.bounding_rect() else {
        return true;
    };
    let Some(right_bounds) = right.geometry.bounding_rect() else {
        return true;
    };

    left_bounds.min().x - clearance <= right_bounds.max().x
        && left_bounds.max().x + clearance >= right_bounds.min().x
        && left_bounds.min().y - clearance <= right_bounds.max().y
        && left_bounds.max().y + clearance >= right_bounds.min().y
}

fn feature_may_touch_circle(feature: &CopperFeature, center: [f64; 2], radius: f64) -> bool {
    let Some(bounds) = feature.sketch.geometry.bounding_rect() else {
        return true;
    };

    center[0] - radius <= bounds.max().x
        && center[0] + radius >= bounds.min().x
        && center[1] - radius <= bounds.max().y
        && center[1] + radius >= bounds.min().y
}

fn distance(left: [f64; 2], right: [f64; 2]) -> f64 {
    let dx = left[0] - right[0];
    let dy = left[1] - right[1];
    (dx * dx + dy * dy).sqrt()
}

fn looks_high_current_net(net: &str) -> bool {
    let normalized = net.to_ascii_uppercase();
    let tokens = [
        "VBAT", "VBUS", "VIN", "VCC", "VDD", "VOUT", "PWR", "POWER", "MOTOR", "PHASE", "+12V",
        "+5V", "+3V3", "12V", "5V", "3V3", "1V8",
    ];

    tokens.iter().any(|token| normalized.contains(token))
}

fn looks_thermal_or_power_net(net: &str) -> bool {
    let normalized = net.to_ascii_uppercase();
    let tokens = [
        "THERM", "THERMAL", "PAD", "EPAD", "HEAT", "HEATER", "LED", "REG", "FET", "MOSFET", "BUCK",
        "LDO",
    ];

    looks_high_current_net(net) || tokens.iter().any(|token| normalized.contains(token))
}

fn looks_hot_component_net(net: &str) -> bool {
    let normalized = net.to_ascii_uppercase();
    let tokens = [
        "THERM", "THERMAL", "HEAT", "HEATER", "LED", "REG", "FET", "MOSFET", "BUCK", "BOOST", "SW",
        "PHASE", "MOTOR", "DRV", "DRIVE", "LDO",
    ];

    looks_high_current_net(net) || tokens.iter().any(|token| normalized.contains(token))
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
    use crate::geometry::{circle_polygon, polygons_to_sketch, rect_polygon};
    use crate::kicad::{BoardModel, CopperFeature, CopperKind, DrillFeature};

    use super::{
        hot_component_spacing_readiness, thermal_copper_area_readiness,
        thermal_mechanical_keepout_readiness, thermal_pad_via_readiness, thermal_relief_readiness,
        thermal_via_distribution_readiness, thermal_via_readiness,
    };

    #[test]
    fn thermal_via_distribution_reports_clustered_via_array() {
        let board = board_with_copper(vec![
            copper_rect("VOUT", CopperKind::Zone, "F.Cu", -1.0, -1.0, 3.0, 1.0),
            copper_disc("VOUT", CopperKind::Via, [0.0, 0.0], 0.20),
            copper_disc("VOUT", CopperKind::Via, [0.25, 0.0], 0.20),
        ]);

        let violations = thermal_via_distribution_readiness(&board, &[], 2, 1.0, 0.10);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "thermal-via-distribution-readiness");
        assert!(
            violations[0]
                .message
                .as_deref()
                .is_some_and(|message| message.contains("heat spreading"))
        );
    }

    #[test]
    fn thermal_via_distribution_allows_wide_or_sparse_vias() {
        let wide = board_with_copper(vec![
            copper_rect("VOUT", CopperKind::Zone, "F.Cu", -1.0, -1.0, 3.0, 1.0),
            copper_disc("VOUT", CopperKind::Via, [0.0, 0.0], 0.20),
            copper_disc("VOUT", CopperKind::Via, [1.5, 0.0], 0.20),
        ]);
        let sparse = board_with_copper(vec![
            copper_rect("VOUT", CopperKind::Zone, "F.Cu", -1.0, -1.0, 3.0, 1.0),
            copper_disc("VOUT", CopperKind::Via, [0.0, 0.0], 0.20),
        ]);

        assert!(thermal_via_distribution_readiness(&wide, &[], 2, 1.0, 0.10).is_empty());
        assert!(thermal_via_distribution_readiness(&sparse, &[], 2, 1.0, 0.10).is_empty());
    }

    #[test]
    fn thermal_via_distribution_respects_selected_layers() {
        let board = board_with_copper(vec![
            copper_rect("VOUT", CopperKind::Zone, "B.Cu", -1.0, -1.0, 3.0, 1.0),
            copper_disc_on_layer("VOUT", CopperKind::Via, "B.Cu", [0.0, 0.0], 0.20),
            copper_disc_on_layer("VOUT", CopperKind::Via, "B.Cu", [0.25, 0.0], 0.20),
        ]);

        assert!(
            thermal_via_distribution_readiness(&board, &["F.Cu".to_string()], 2, 1.0, 0.10)
                .is_empty()
        );
        assert_eq!(
            thermal_via_distribution_readiness(&board, &["B.Cu".to_string()], 2, 1.0, 0.10).len(),
            1
        );
    }

    #[test]
    fn thermal_relief_readiness_reports_pad_embedded_in_same_net_zone() {
        let board = board_with_copper(vec![
            copper_disc("GND", CopperKind::Pad, [0.0, 0.0], 0.5),
            copper_rect("GND", CopperKind::Zone, "F.Cu", -1.0, -1.0, 1.0, 1.0),
        ]);

        let violations = thermal_relief_readiness(&board, &[], 1.0e-9);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "thermal-relief-readiness");
    }

    #[test]
    fn thermal_via_readiness_reports_power_zone_with_too_few_vias() {
        let board = board_with_copper(vec![copper_rect(
            "VDD_3V3",
            CopperKind::Zone,
            "F.Cu",
            -1.0,
            -1.0,
            1.0,
            1.0,
        )]);

        let violations = thermal_via_readiness(&board, &[], 2, 0.10);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "thermal-via-readiness");
    }

    #[test]
    fn thermal_pad_via_readiness_reports_large_power_or_ground_pads_without_vias() {
        let board = board_with_copper(vec![copper_rect(
            "GND",
            CopperKind::Pad,
            "F.Cu",
            -1.5,
            -1.5,
            1.5,
            1.5,
        )]);

        let violations = thermal_pad_via_readiness(&board, &[], 2.0);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "thermal-pad-via-readiness");
    }

    #[test]
    fn thermal_copper_area_readiness_reports_power_feature_without_nearby_zone() {
        let board = board_with_copper(vec![
            copper_disc("VOUT", CopperKind::Pad, [0.0, 0.0], 0.30),
            copper_rect("VOUT", CopperKind::Zone, "F.Cu", 5.0, 0.0, 7.0, 2.0),
        ]);

        let violations = thermal_copper_area_readiness(&board, &[], 2.0);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "thermal-copper-area-readiness");
    }

    #[test]
    fn thermal_copper_area_readiness_culls_sparse_zone_fields() {
        let mut copper = sparse_rects("VOUT", CopperKind::Zone, 2_000, 100.0);
        copper.push(copper_disc("VOUT", CopperKind::Pad, [0.0, 0.0], 0.30));
        copper.push(copper_rect(
            "VOUT",
            CopperKind::Zone,
            "F.Cu",
            0.8,
            -0.5,
            1.8,
            0.5,
        ));
        copper.push(copper_rect(
            "VOUT",
            CopperKind::Zone,
            "B.Cu",
            0.8,
            -0.5,
            1.8,
            0.5,
        ));
        let board = board_with_copper(copper);

        assert!(thermal_copper_area_readiness(&board, &[], 2.0).is_empty());
    }

    #[test]
    fn hot_component_spacing_readiness_reports_hot_feature_near_neighbor() {
        let board = board_with_copper(vec![
            copper_rect("LED_PWR", CopperKind::Pad, "F.Cu", 0.0, 0.0, 1.0, 1.0),
            copper_rect("SENSOR_OUT", CopperKind::Pad, "F.Cu", 1.2, 0.0, 2.0, 1.0),
            copper_rect("GND", CopperKind::Zone, "F.Cu", 0.0, 2.0, 2.0, 3.0),
        ]);

        let violations = hot_component_spacing_readiness(&board, &[], 0.3, 1.0e-9);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "hot-component-spacing-readiness");
    }

    #[test]
    fn hot_component_spacing_readiness_culls_sparse_neighbors() {
        let mut copper = sparse_rects("SENSOR", CopperKind::Pad, 2_000, 100.0);
        copper.push(copper_rect(
            "LED_PWR",
            CopperKind::Pad,
            "F.Cu",
            0.0,
            0.0,
            1.0,
            1.0,
        ));
        copper.push(copper_rect(
            "SENSOR_NEAR",
            CopperKind::Pad,
            "F.Cu",
            1.2,
            0.0,
            2.0,
            1.0,
        ));
        copper.push(copper_rect(
            "SENSOR_OTHER_LAYER",
            CopperKind::Pad,
            "B.Cu",
            1.2,
            0.0,
            2.0,
            1.0,
        ));
        let board = board_with_copper(copper);

        let violations = hot_component_spacing_readiness(&board, &[], 0.3, 1.0e-9);

        assert_eq!(violations.len(), 1);
        assert!(violations[0].message.as_ref().is_some_and(|message| {
            message.contains("SENSOR_NEAR") && !message.contains("SENSOR_OTHER_LAYER")
        }));
    }

    #[test]
    fn thermal_mechanical_keepout_readiness_reports_hot_feature_near_hole() {
        let mut board = board_with_copper(vec![copper_rect(
            "HEATER_OUT",
            CopperKind::Pad,
            "F.Cu",
            0.0,
            0.0,
            1.0,
            1.0,
        )]);
        board.drills = vec![DrillFeature {
            location: [1.4, 0.5],
            diameter: 0.8,
            net: None,
            plated: false,
        }];

        let violations = thermal_mechanical_keepout_readiness(&board, &[], &[], 0.2, 1.0e-9);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "thermal-mechanical-keepout-readiness");
    }

    #[test]
    fn thermal_mechanical_keepout_culls_large_sparse_hot_fields() {
        let mut copper = Vec::new();
        for index in 0..700 {
            let x = 10.0 + (index % 35) as f64 * 4.0;
            let y = (index / 35) as f64 * 4.0;
            copper.push(copper_rect(
                &format!("HEATER_{index}"),
                CopperKind::Pad,
                "F.Cu",
                x,
                y,
                x + 0.5,
                y + 0.5,
            ));
        }
        copper.push(copper_rect(
            "HEATER_NEAR",
            CopperKind::Pad,
            "F.Cu",
            0.0,
            0.0,
            1.0,
            1.0,
        ));
        let mut board = board_with_copper(copper);
        board.drills = vec![DrillFeature {
            location: [1.4, 0.5],
            diameter: 0.8,
            net: None,
            plated: false,
        }];

        let start = std::time::Instant::now();
        let violations = thermal_mechanical_keepout_readiness(&board, &[], &[], 0.2, 1.0e-9);

        assert_eq!(violations.len(), 1);
        assert!(
            start.elapsed() < std::time::Duration::from_secs(2),
            "mechanical keepout checks should cull distant hot features"
        );
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

    fn sparse_rects(
        prefix: &str,
        kind: CopperKind,
        count: usize,
        offset_x: f64,
    ) -> Vec<CopperFeature> {
        (0..count)
            .map(|index| {
                let x = offset_x + (index % 100) as f64 * 3.0;
                let y = (index / 100) as f64 * 3.0;
                copper_rect(
                    &format!("{prefix}_{index}"),
                    kind,
                    "F.Cu",
                    x,
                    y,
                    x + 0.5,
                    y + 0.5,
                )
            })
            .collect()
    }
}
