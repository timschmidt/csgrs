//! Thermal and heat-spreading readiness checks.
//!
//! These checks operate on parsed KiCad copper and drill data. They cover
//! thermal-relief intent, thermal-via count and spread, exposed-pad via support,
//! hot-copper spacing, and mechanical keepouts around likely hot features.
//!
//! Reliability note: these checks are not thermal simulation. Net-name heat
//! inference, copper-area proxies, and via-count rules are suspect near heat
//! spreaders, enclosures, unusual airflow, and package-specific requirements.

use csgrs::{csg::CSG, sketch::Profile};
use geo::BoundingRect;

use crate::checks::distance::{polygon_boundaries_within_scalar, polygon_boundary_distance_scalar};
use crate::checks::spatial::CopperSpatialIndex;
use crate::checks::spread::maximum_point_spread;
use crate::geometry::multipolygon_to_shapes_scalar;
use crate::kicad::{BoardModel, CopperFeature, CopperKind, DrillFeature};
use crate::report::{Severity, Violation};
use crate::{LayerMetadata, PcbSketch, PcbSketchExt, Scalar};

/// Run the `thermal_relief_readiness` design-readiness check or report helper.
///
/// Same-net zone candidates use the shared copper spatial index before exact
/// pad/via-to-zone CSG intersection, so sparse power/ground pours do not
/// devolve into all-zone scans.
pub fn thermal_relief_readiness(
    board: &BoardModel,
    selected_layers: &[String],
    min_area: &Scalar,
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
    let zone_index = CopperSpatialIndex::new(&zones, 0.0);
    let mut violations = Vec::new();
    let mut candidate_zones = 0usize;

    for anchor in &anchors {
        if anchor.net.is_none() {
            continue;
        }
        let candidates = zone_index.same_layer_near_feature(anchor, 0.0);
        candidate_zones += candidates.len();
        for zone_index in candidates {
            let zone = zones[zone_index];
            if anchor.net != zone.net {
                continue;
            }

            let overlap = anchor.sketch.intersection(&zone.sketch);
            let shapes = multipolygon_to_shapes_scalar(&overlap.to_multipolygon(), min_area);
            if shapes.is_empty() {
                continue;
            }

            violations.push(Violation::new(
                "thermal-relief-readiness",
                Severity::Warning,
                vec![anchor.layer.clone()],
                None,
                shapes,
                vec![
                    anchor.location_f64_compatibility_required(),
                    zone.location_f64_compatibility_required(),
                ],
                Some(format!(
                    "same-net {:?} copper intersects a copper zone; confirm thermal relief or intentional direct plane connection",
                    anchor.kind
                )),
            ));
        }
    }
    log::trace!(
        "thermal-relief readiness: source={} anchors={} zones={} zone_buckets={} candidate_zones={} violations={}",
        board.source,
        anchors.len(),
        zones.len(),
        zone_index.bucket_count(),
        candidate_zones,
        violations.len()
    );

    violations
}

/// Run the `thermal_via_readiness` design-readiness check or report helper.
///
/// Same-layer via candidates use the shared copper spatial index before exact
/// zone/via touch review. This is only a broad phase, still
/// requiring exact CSG or boundary-distance confirmation before a via counts.
pub fn thermal_via_readiness(
    board: &BoardModel,
    selected_layers: &[String],
    minimum_vias: usize,
    anchor_tolerance: &Scalar,
) -> Vec<Violation> {
    let features = selected_copper_features(board, selected_layers);
    let vias = features
        .iter()
        .copied()
        .filter(|feature| feature.kind == CopperKind::Via)
        .collect::<Vec<_>>();
    let broad_phase_tolerance = scalar_broad_phase_radius(anchor_tolerance);
    let via_index = CopperSpatialIndex::new(&vias, broad_phase_tolerance);
    let mut violations = Vec::new();
    let mut candidate_vias = 0usize;

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

        let (zone_vias, zone_candidate_vias) = thermal_zone_vias_indexed(
            &vias,
            &via_index,
            zone,
            anchor_tolerance,
            broad_phase_tolerance,
        );
        candidate_vias += zone_candidate_vias;
        let via_count = zone_vias.len();
        if via_count >= minimum_vias {
            continue;
        }

        violations.push(Violation::new(
            "thermal-via-readiness",
            Severity::Warning,
            vec![zone.layer.clone()],
            None,
            Vec::new(),
            vec![zone.location_f64_compatibility_required()],
            Some(format!(
                "likely power or thermal zone {net} has {via_count} parsed same-net via(s), below review threshold {minimum_vias}"
            )),
        ));
    }

    log::trace!(
        "thermal-via readiness: source={} features={} vias={} via_buckets={} candidate_vias={} minimum_vias={} violations={}",
        board.source,
        features.len(),
        vias.len(),
        via_index.bucket_count(),
        candidate_vias,
        minimum_vias,
        violations.len()
    );

    violations
}

/// Warn when a likely thermal via set exists but is too clustered to distribute
/// heat across the local copper area.
///
/// This is a geometry readiness check, not a thermal solver. It reports cases
/// where a power/thermal zone has the requested number of same-net vias, but the
/// via field has a small maximum span. Thermal-via count and distribution are
/// retained as influential PCB heat-distribution parameters.
pub fn thermal_via_distribution_readiness(
    board: &BoardModel,
    selected_layers: &[String],
    minimum_vias: usize,
    minimum_spread: &Scalar,
    anchor_tolerance: &Scalar,
) -> Vec<Violation> {
    if minimum_vias < 2 || minimum_spread <= &Scalar::zero() {
        return Vec::new();
    }

    let features = selected_copper_features(board, selected_layers);
    let vias = features
        .iter()
        .copied()
        .filter(|feature| feature.kind == CopperKind::Via)
        .collect::<Vec<_>>();
    let broad_phase_tolerance = scalar_broad_phase_radius(anchor_tolerance);
    let via_index = CopperSpatialIndex::new(&vias, broad_phase_tolerance);
    let mut violations = Vec::new();
    let mut candidate_vias = 0usize;

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

        let (zone_vias, zone_candidate_vias) = thermal_zone_vias_indexed(
            &vias,
            &via_index,
            zone,
            anchor_tolerance,
            broad_phase_tolerance,
        );
        candidate_vias += zone_candidate_vias;
        if zone_vias.len() < minimum_vias {
            continue;
        }
        let spread = maximum_point_spread(zone_vias.iter().map(|via| {
            (
                via.location.clone(),
                via.location_f64_compatibility_required(),
            )
        }));
        log::trace!(
            "thermal-via distribution readiness: source={} zone_net={net} vias={} hull_points={} caliper_steps={} spread={:.6}",
            board.source,
            zone_vias.len(),
            spread.hull_points,
            spread.caliper_steps,
            spread.distance
        );
        if &spread.distance >= minimum_spread {
            continue;
        }

        violations.push(Violation::new(
            "thermal-via-distribution-readiness",
            Severity::Warning,
            vec![zone.layer.clone()],
            None,
            Vec::new(),
            zone_vias.iter().map(|via| via.location_f64_compatibility_required()).collect(),
            Some(format!(
                "likely power or thermal zone {net} has {} parsed same-net vias but via-field spread {:.6} is below {minimum_spread:.6}; review thermal via distribution and heat spreading",
                zone_vias.len(),
                spread.distance
            )),
        ));
    }

    log::trace!(
        "thermal-via distribution readiness: source={} features={} vias={} via_buckets={} candidate_vias={} minimum_vias={} minimum_spread={minimum_spread:.6} violations={}",
        board.source,
        features.len(),
        vias.len(),
        via_index.bucket_count(),
        candidate_vias,
        minimum_vias,
        violations.len()
    );

    violations
}

/// Review large thermal/power pads for same-net via-in-pad evidence.
///
/// Same-layer via candidates use `CopperSpatialIndex` before exact via/pad CSG
/// intersection. The index is only a conservative broad phase. Exposed-pad and
/// via geometry remain package-level thermal review data rather than a simple
/// copper DRC.
pub fn thermal_pad_via_readiness(
    board: &BoardModel,
    selected_layers: &[String],
    minimum_pad_dimension: &Scalar,
) -> Vec<Violation> {
    let features = selected_copper_features(board, selected_layers);
    let vias = features
        .iter()
        .copied()
        .filter(|feature| feature.kind == CopperKind::Via)
        .collect::<Vec<_>>();
    let via_index = CopperSpatialIndex::new(&vias, 0.0);
    let mut violations = Vec::new();
    let mut candidate_vias = 0usize;
    let mut exact_via_checks = 0usize;

    for pad in &features {
        if pad.kind != CopperKind::Pad {
            continue;
        }
        let Some(net) = pad.net.as_deref() else {
            continue;
        };
        if !looks_ground_net(net) && !looks_high_current_net(net) {
            continue;
        }
        let Some((min_dimension, max_dimension)) = bounding_dimensions_scalar(&pad.sketch) else {
            continue;
        };
        if &min_dimension < minimum_pad_dimension {
            continue;
        }
        let Ok(aspect_ratio) = max_dimension / min_dimension else {
            continue;
        };
        if aspect_ratio > crate::scalar::scalar("3") {
            continue;
        }

        let candidates = via_index.same_layer_near_feature(pad, 0.0);
        candidate_vias += candidates.len();
        let has_same_net_via = candidates.into_iter().any(|via_index| {
            let via = vias[via_index];
            if via.net != pad.net {
                return false;
            }
            exact_via_checks += 1;
            !multipolygon_to_shapes_scalar(
                &via.sketch.intersection(&pad.sketch).to_multipolygon(),
                &crate::scalar::scalar("1.0e-9"),
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
            vec![pad.location_f64_compatibility_required()],
            Some(format!(
                "large likely thermal pad on net {net:?} has no parsed same-net via in pad; review exposed-pad thermal via array, fill, tent, and solder-voiding intent"
            )),
        ));
    }
    log::trace!(
        "thermal-pad via readiness: source={} features={} vias={} via_buckets={} candidate_vias={} exact_via_checks={} violations={}",
        board.source,
        features.len(),
        vias.len(),
        via_index.bucket_count(),
        candidate_vias,
        exact_via_checks,
        violations.len()
    );
    debug_assert!(exact_via_checks <= candidate_vias);

    violations
}

/// Review likely hot or high-current features for nearby same-net copper area.
///
/// IPC-2152 treats current capacity as a board-level design decision. This
/// check verifies parsed
/// same-net zone evidence near likely thermal/power features; it does not prove
/// a temperature-rise target.
pub fn thermal_copper_area_readiness(
    board: &BoardModel,
    selected_layers: &[String],
    search_radius: &Scalar,
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
    let broad_phase_radius = scalar_broad_phase_radius(search_radius);
    let zone_index = CopperSpatialIndex::new(&same_net_zones, broad_phase_radius);
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

        let zone_candidates = zone_index.same_layer_candidate_centers_near(
            feature.location_f64_compatibility_required(),
            &feature.layer,
            broad_phase_radius,
        );
        candidate_count += zone_candidates.len();
        let has_nearby_same_net_zone = zone_candidates.into_iter().any(|zone_index| {
            same_net_zones[zone_index].net == feature.net
                && point_distance_scalar(&feature.location, &same_net_zones[zone_index].location)
                    .is_some_and(|distance| &distance <= search_radius)
        });
        if has_nearby_same_net_zone {
            continue;
        }

        violations.push(Violation::new(
            "thermal-copper-area-readiness",
            Severity::Warning,
            vec![feature.layer.clone()],
            None,
            Vec::new(),
            vec![feature.location_f64_compatibility_required()],
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

/// Review spacing from likely hot features to neighboring non-ground copper.
///
/// This is not thermal simulation. A broad phase precedes exact offset
/// intersections that should be reviewed against derating, airflow, enclosure,
/// and package thermal data.
pub fn hot_component_spacing_readiness(
    board: &BoardModel,
    selected_layers: &[String],
    spacing: &Scalar,
    min_area: &Scalar,
) -> Vec<Violation> {
    let features = selected_copper_features(board, selected_layers);
    let broad_phase_spacing = scalar_broad_phase_radius(spacing);
    let feature_index = CopperSpatialIndex::new(&features, broad_phase_spacing);
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
    let mut exact_pair_count = 0_usize;

    for hot in hot_features {
        for neighbor_index in feature_index.same_layer_near_feature(hot, broad_phase_spacing) {
            candidate_count += 1;
            let neighbor = features[neighbor_index];
            if std::ptr::eq(hot, neighbor) {
                continue;
            }
            if hot.net == neighbor.net || neighbor.net.as_deref().is_some_and(looks_ground_net) {
                continue;
            }
            if !sketches_within_clearance(&hot.sketch, &neighbor.sketch, broad_phase_spacing) {
                continue;
            }
            exact_pair_count += 1;

            let overlap = hot
                .sketch
                .offset(spacing.clone())
                .intersection(&neighbor.sketch);
            let shapes = multipolygon_to_shapes_scalar(&overlap.to_multipolygon(), min_area);
            let fallback_hit = shapes.is_empty()
                && polygon_boundary_distance_scalar(
                    &hot.sketch.to_multipolygon(),
                    &neighbor.sketch.to_multipolygon(),
                )
                .is_some_and(|distance| &distance <= spacing);
            if shapes.is_empty() && !fallback_hit {
                continue;
            }

            violations.push(Violation::new(
                "hot-component-spacing-readiness",
                Severity::Warning,
                vec![hot.layer.clone()],
                None,
                shapes,
                vec![
                    hot.location_f64_compatibility_required(),
                    neighbor.location_f64_compatibility_required(),
                ],
                Some(format!(
                    "likely hot feature {:?} is within thermal spacing {spacing:.6} of neighboring net {:?}; review heat spreading, derating, and component placement",
                    hot.net, neighbor.net
                )),
            ));
        }
    }

    log::trace!(
        "hot-component spacing readiness: source={} candidate_pairs={} exact_pairs={} violations={}",
        board.source,
        candidate_count,
        exact_pair_count,
        violations.len()
    );
    debug_assert!(exact_pair_count <= candidate_count);

    violations
}

/// Review likely hot copper against mechanical keepout holes.
///
/// Heatsink screws, standoffs, chassis contacts, and airflow blockers are
/// mechanical/thermal constraints that copper DRC alone cannot prove. This
/// check uses non-plated drill keepouts and exact CSG overlap after spatial
/// culling to flag packages that need heatsink, enclosure, or assembly drawing
/// review.
pub fn thermal_mechanical_keepout_readiness(
    board: &BoardModel,
    extra_drills: &[DrillFeature],
    selected_layers: &[String],
    keepout: &Scalar,
    min_area: &Scalar,
) -> Vec<Violation> {
    let mut mechanical_drills = board
        .drills
        .iter()
        .chain(extra_drills.iter())
        .filter(|drill| !drill.plated)
        .collect::<Vec<_>>();
    mechanical_drills.sort_by(|left, right| {
        left.location[0]
            .partial_cmp(&right.location[0])
            .unwrap_or(std::cmp::Ordering::Equal)
            .then_with(|| {
                left.location[1]
                    .partial_cmp(&right.location[1])
                    .unwrap_or(std::cmp::Ordering::Equal)
            })
            .then_with(|| {
                left.diameter
                    .partial_cmp(&right.diameter)
                    .unwrap_or(std::cmp::Ordering::Equal)
            })
    });

    let hot_features = selected_copper_features(board, selected_layers)
        .into_iter()
        .filter(|feature| feature.net.as_deref().is_some_and(looks_hot_component_net))
        .collect::<Vec<_>>();
    let broad_phase_keepout = scalar_broad_phase_radius(keepout);
    let hot_index = CopperSpatialIndex::new(&hot_features, broad_phase_keepout);
    log::trace!(
        "thermal mechanical-keepout readiness: source={} hot_features={} buckets={} mechanical_drills={} keepout={keepout:#.6}",
        board.source,
        hot_features.len(),
        hot_index.bucket_count(),
        mechanical_drills.len()
    );
    let mut violations = Vec::new();
    let mut candidate_count = 0_usize;
    let mut exact_pair_count = 0_usize;

    for drill in mechanical_drills {
        let Ok(drill_radius) = drill.diameter.clone() / crate::scalar::scalar("2") else {
            continue;
        };
        let keepout_radius = drill_radius + keepout;
        let broad_phase_radius = scalar_broad_phase_radius(&keepout_radius);
        let center = drill.location_f64_compatibility_required();
        let broad_candidates = hot_index.all_layers_near_circle(center, broad_phase_radius);
        candidate_count += broad_candidates.len();
        let candidates = broad_candidates
            .into_iter()
            .filter(|&index| {
                feature_may_touch_circle(hot_features[index], center, broad_phase_radius)
            })
            .collect::<Vec<_>>();
        if candidates.is_empty() {
            continue;
        }
        let keepout_sketch = PcbSketch::new(
            Profile::circle(keepout_radius, 64).translate(
                drill.location[0].clone(),
                drill.location[1].clone(),
                Scalar::zero(),
            ),
            Some(LayerMetadata {
                name: "thermal mechanical keepout".to_string(),
            }),
        );

        for hot_index in candidates {
            let hot = hot_features[hot_index];
            exact_pair_count += 1;
            let (shapes, uncertain) = match keepout_sketch.try_intersection(&hot.sketch) {
                Ok(overlap) => (
                    multipolygon_to_shapes_scalar(&overlap.to_multipolygon(), min_area),
                    false,
                ),
                Err(error) => {
                    log::warn!(
                        "thermal mechanical keepout used conservative finding after uncertified profile intersection: {error}"
                    );
                    (Vec::new(), true)
                }
            };
            let fallback_hit = shapes.is_empty()
                && polygon_boundaries_within_scalar(
                    &keepout_sketch.to_multipolygon(),
                    &hot.sketch.to_multipolygon(),
                    &Scalar::zero(),
                );
            if shapes.is_empty() && !fallback_hit && !uncertain {
                continue;
            }

            violations.push(Violation::new(
                "thermal-mechanical-keepout-readiness",
                Severity::Warning,
                vec![hot.layer.clone()],
                None,
                shapes,
                vec![
                    drill.location_f64_compatibility_required(),
                    hot.location_f64_compatibility_required(),
                ],
                Some(format!(
                    "likely hot feature {:?} is inside mechanical thermal keepout {keepout:#.6}; review heatsink, standoff, screw, chassis, and airflow clearance",
                    hot.net
                )),
            ));
        }
    }

    log::trace!(
        "thermal mechanical-keepout readiness: source={} candidate_pairs={} exact_pairs={} violations={}",
        board.source,
        candidate_count,
        exact_pair_count,
        violations.len()
    );
    debug_assert!(exact_pair_count <= candidate_count);

    violations
}

fn thermal_zone_vias_indexed<'a>(
    vias: &[&'a CopperFeature],
    via_index: &CopperSpatialIndex<'a>,
    zone: &CopperFeature,
    anchor_tolerance: &Scalar,
    broad_phase_tolerance: f64,
) -> (Vec<&'a CopperFeature>, usize) {
    let candidates = via_index.same_layer_near_feature(zone, broad_phase_tolerance);
    let candidate_count = candidates.len();
    let zone_vias = candidates
        .into_iter()
        .filter_map(|via_index| {
            let via = vias[via_index];
            (via.net == zone.net
                && (feature_contains_point_scalar(zone, &via.location)
                    || copper_features_touch_scalar(via, zone, anchor_tolerance)))
            .then_some(via)
        })
        .collect();
    (zone_vias, candidate_count)
}

fn copper_features_touch_scalar(
    left: &CopperFeature,
    right: &CopperFeature,
    tolerance: &Scalar,
) -> bool {
    !left
        .sketch
        .intersection(&right.sketch)
        .native_contours()
        .material_contours()
        .is_empty()
        || polygon_boundary_distance_scalar(
            &left.sketch.to_multipolygon(),
            &right.sketch.to_multipolygon(),
        )
        .is_some_and(|distance| &distance <= tolerance)
}

fn feature_contains_point_scalar(feature: &CopperFeature, point: &[Scalar; 2]) -> bool {
    feature
        .sketch
        .contains_xy(point[0].clone(), point[1].clone())
        == Some(true)
}

fn scalar_broad_phase_radius(value: &Scalar) -> f64 {
    let projected = value
        .to_f64_lossy()
        .expect("thermal broad-phase radius must fit the finite compatibility index");
    if projected > 0.0 {
        projected.next_up()
    } else {
        0.0
    }
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

fn bounding_dimensions_scalar(sketch: &PcbSketch) -> Option<(Scalar, Scalar)> {
    let (width, height) = if let Some(bounds) = sketch.exact_bounds() {
        (&bounds[2] - &bounds[0], &bounds[3] - &bounds[1])
    } else {
        let bounds = sketch.geometry().bounding_rect()?;
        (
            Scalar::try_from(bounds.max().x).ok()? - Scalar::try_from(bounds.min().x).ok()?,
            Scalar::try_from(bounds.max().y).ok()? - Scalar::try_from(bounds.min().y).ok()?,
        )
    };
    Some(if width <= height {
        (width, height)
    } else {
        (height, width)
    })
}

fn point_distance_scalar(left: &[Scalar; 2], right: &[Scalar; 2]) -> Option<Scalar> {
    let dx = &left[0] - &right[0];
    let dy = &left[1] - &right[1];
    (&dx * &dx + &dy * &dy).sqrt().ok()
}

fn sketches_within_clearance(left: &PcbSketch, right: &PcbSketch, clearance: f64) -> bool {
    let Some(left_bounds) = left.geometry().bounding_rect() else {
        return true;
    };
    let Some(right_bounds) = right.geometry().bounding_rect() else {
        return true;
    };

    left_bounds.min().x - clearance <= right_bounds.max().x
        && left_bounds.max().x + clearance >= right_bounds.min().x
        && left_bounds.min().y - clearance <= right_bounds.max().y
        && left_bounds.max().y + clearance >= right_bounds.min().y
}

fn feature_may_touch_circle(feature: &CopperFeature, center: [f64; 2], radius: f64) -> bool {
    let Some(bounds) = feature.sketch.geometry().bounding_rect() else {
        return true;
    };

    center[0] - radius <= bounds.max().x
        && center[0] + radius >= bounds.min().x
        && center[1] - radius <= bounds.max().y
        && center[1] + radius >= bounds.min().y
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
    use crate::geometry::{circle_polygon, polygons_to_profile, rect_polygon};
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

        let violations = thermal_via_distribution_readiness(
            &board,
            &[],
            2,
            &crate::scalar::scalar("1.0"),
            &crate::scalar::scalar("0.10"),
        );

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

        assert!(
            thermal_via_distribution_readiness(
                &wide,
                &[],
                2,
                &crate::scalar::scalar("1.0"),
                &crate::scalar::scalar("0.10")
            )
            .is_empty()
        );
        assert!(
            thermal_via_distribution_readiness(
                &sparse,
                &[],
                2,
                &crate::scalar::scalar("1.0"),
                &crate::scalar::scalar("0.10")
            )
            .is_empty()
        );
    }

    #[test]
    fn thermal_via_distribution_respects_selected_layers() {
        let board = board_with_copper(vec![
            copper_rect("VOUT", CopperKind::Zone, "B.Cu", -1.0, -1.0, 3.0, 1.0),
            copper_disc_on_layer("VOUT", CopperKind::Via, "B.Cu", [0.0, 0.0], 0.20),
            copper_disc_on_layer("VOUT", CopperKind::Via, "B.Cu", [0.25, 0.0], 0.20),
        ]);

        assert!(
            thermal_via_distribution_readiness(
                &board,
                &["F.Cu".to_string()],
                2,
                &crate::scalar::scalar("1.0"),
                &crate::scalar::scalar("0.10")
            )
            .is_empty()
        );
        assert_eq!(
            thermal_via_distribution_readiness(
                &board,
                &["B.Cu".to_string()],
                2,
                &crate::scalar::scalar("1.0"),
                &crate::scalar::scalar("0.10")
            )
            .len(),
            1
        );
    }

    #[test]
    fn thermal_via_distribution_handles_large_clustered_via_fields() {
        let mut copper = vec![copper_rect(
            "VOUT",
            CopperKind::Zone,
            "F.Cu",
            -2.0,
            -2.0,
            2.0,
            2.0,
        )];
        copper.extend((0..2_000).map(|index| {
            copper_disc(
                "VOUT",
                CopperKind::Via,
                [(index % 50) as f64 * 0.02, (index / 50) as f64 * 0.02],
                0.01,
            )
        }));
        let board = board_with_copper(copper);

        let started = std::time::Instant::now();
        let violations = thermal_via_distribution_readiness(
            &board,
            &[],
            2,
            &crate::scalar::scalar("5.0"),
            &crate::scalar::scalar("0.0"),
        );

        assert_eq!(violations.len(), 1);
        assert!(
            started.elapsed() < std::time::Duration::from_secs(2),
            "thermal-via distribution should use hull diameter instead of all-pairs spread"
        );
    }

    #[test]
    fn thermal_via_distribution_culls_sparse_via_fields_per_zone() {
        let mut copper = (0..2_000)
            .map(|index| {
                let x = 100.0 + (index % 100) as f64 * 3.0;
                let y = (index / 100) as f64 * 3.0;
                copper_disc("VOUT", CopperKind::Via, [x, y], 0.02)
            })
            .collect::<Vec<_>>();
        copper.push(copper_rect(
            "VOUT",
            CopperKind::Zone,
            "F.Cu",
            -1.0,
            -1.0,
            1.0,
            1.0,
        ));
        copper.push(copper_disc("VOUT", CopperKind::Via, [0.0, 0.0], 0.20));
        copper.push(copper_disc("VOUT", CopperKind::Via, [0.25, 0.0], 0.20));
        let board = board_with_copper(copper);

        let started = std::time::Instant::now();
        let violations = thermal_via_distribution_readiness(
            &board,
            &[],
            2,
            &crate::scalar::scalar("1.0"),
            &crate::scalar::scalar("0.10"),
        );

        assert_eq!(violations.len(), 1);
        assert!(
            started.elapsed() < std::time::Duration::from_secs(2),
            "thermal-via distribution should index sparse via fields before exact zone anchoring"
        );
    }

    #[test]
    fn thermal_relief_readiness_reports_pad_embedded_in_same_net_zone() {
        let board = board_with_copper(vec![
            copper_disc("GND", CopperKind::Pad, [0.0, 0.0], 0.5),
            copper_rect("GND", CopperKind::Zone, "F.Cu", -1.0, -1.0, 1.0, 1.0),
        ]);

        let violations = thermal_relief_readiness(&board, &[], &crate::scalar::scalar("1.0e-9"));

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "thermal-relief-readiness");
    }

    #[test]
    fn thermal_relief_readiness_culls_sparse_zone_fields() {
        let mut copper = sparse_rects("GND", CopperKind::Zone, 2_000, 100.0);
        copper.push(copper_disc("GND", CopperKind::Pad, [0.0, 0.0], 0.5));
        copper.push(copper_rect(
            "GND",
            CopperKind::Zone,
            "F.Cu",
            -1.0,
            -1.0,
            1.0,
            1.0,
        ));
        copper.push(copper_rect(
            "GND",
            CopperKind::Zone,
            "B.Cu",
            -1.0,
            -1.0,
            1.0,
            1.0,
        ));
        let board = board_with_copper(copper);

        let started = std::time::Instant::now();
        let violations = thermal_relief_readiness(&board, &[], &crate::scalar::scalar("1.0e-9"));

        assert_eq!(violations.len(), 1);
        assert!(
            started.elapsed() < std::time::Duration::from_secs(2),
            "thermal relief should index sparse same-layer zones before exact intersection"
        );
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

        let violations = thermal_via_readiness(&board, &[], 2, &crate::scalar::scalar("0.10"));

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "thermal-via-readiness");
    }

    #[test]
    fn thermal_via_readiness_culls_sparse_via_fields_per_zone() {
        let mut copper = (0..2_000)
            .map(|index| {
                let x = 100.0 + (index % 100) as f64 * 3.0;
                let y = (index / 100) as f64 * 3.0;
                copper_disc("VDD_3V3", CopperKind::Via, [x, y], 0.02)
            })
            .collect::<Vec<_>>();
        copper.push(copper_rect(
            "VDD_3V3",
            CopperKind::Zone,
            "F.Cu",
            -1.0,
            -1.0,
            1.0,
            1.0,
        ));
        let board = board_with_copper(copper);

        let started = std::time::Instant::now();
        let violations = thermal_via_readiness(&board, &[], 2, &crate::scalar::scalar("0.10"));

        assert_eq!(violations.len(), 1);
        assert!(
            started.elapsed() < std::time::Duration::from_secs(2),
            "thermal-via readiness should index sparse via fields before exact zone anchoring"
        );
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

        let violations = thermal_pad_via_readiness(&board, &[], &crate::scalar::scalar("2.0"));

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "thermal-pad-via-readiness");
    }

    #[test]
    fn thermal_pad_via_readiness_culls_sparse_via_fields() {
        let mut copper = (0..2_000)
            .map(|index| {
                let x = 100.0 + (index % 100) as f64 * 3.0;
                let y = (index / 100) as f64 * 3.0;
                copper_disc("GND", CopperKind::Via, [x, y], 0.20)
            })
            .collect::<Vec<_>>();
        copper.push(copper_rect(
            "GND",
            CopperKind::Pad,
            "F.Cu",
            -1.5,
            -1.5,
            1.5,
            1.5,
        ));
        let board = board_with_copper(copper);

        let started = std::time::Instant::now();
        let violations = thermal_pad_via_readiness(&board, &[], &crate::scalar::scalar("2.0"));

        assert_eq!(violations.len(), 1);
        assert!(
            started.elapsed() < std::time::Duration::from_secs(2),
            "thermal-pad via should index sparse via fields before exact pad overlap review"
        );
    }

    #[test]
    fn thermal_copper_area_readiness_reports_power_feature_without_nearby_zone() {
        let board = board_with_copper(vec![
            copper_disc("VOUT", CopperKind::Pad, [0.0, 0.0], 0.30),
            copper_rect("VOUT", CopperKind::Zone, "F.Cu", 5.0, 0.0, 7.0, 2.0),
        ]);

        let violations = thermal_copper_area_readiness(&board, &[], &crate::scalar::scalar("2.0"));

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

        assert!(
            thermal_copper_area_readiness(&board, &[], &crate::scalar::scalar("2.0")).is_empty()
        );
    }

    #[test]
    fn hot_component_spacing_readiness_reports_hot_feature_near_neighbor() {
        let board = board_with_copper(vec![
            copper_rect("LED_PWR", CopperKind::Pad, "F.Cu", 0.0, 0.0, 1.0, 1.0),
            copper_rect("SENSOR_OUT", CopperKind::Pad, "F.Cu", 1.2, 0.0, 2.0, 1.0),
            copper_rect("GND", CopperKind::Zone, "F.Cu", 0.0, 2.0, 2.0, 3.0),
        ]);

        let violations = hot_component_spacing_readiness(
            &board,
            &[],
            &crate::scalar::scalar("0.3"),
            &crate::scalar::scalar("1.0e-9"),
        );

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

        let violations = hot_component_spacing_readiness(
            &board,
            &[],
            &crate::scalar::scalar("0.3"),
            &crate::scalar::scalar("1.0e-9"),
        );

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
            location: [
                crate::geometry::exact_real(1.4),
                crate::geometry::exact_real(0.5),
            ],
            diameter: crate::scalar::scalar("0.8"),
            net: None,
            plated: false,
        }];

        let violations = thermal_mechanical_keepout_readiness(
            &board,
            &[],
            &[],
            &crate::scalar::scalar("0.2"),
            &crate::scalar::scalar("1.0e-9"),
        );

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
            location: [
                crate::geometry::exact_real(1.4),
                crate::geometry::exact_real(0.5),
            ],
            diameter: crate::scalar::scalar("0.8"),
            net: None,
            plated: false,
        }];

        let start = std::time::Instant::now();
        let violations = thermal_mechanical_keepout_readiness(
            &board,
            &[],
            &[],
            &crate::scalar::scalar("0.2"),
            &crate::scalar::scalar("1.0e-9"),
        );

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
            location: [
                crate::geometry::exact_real(location[0]),
                crate::geometry::exact_real(location[1]),
            ],
            sketch: polygons_to_profile(
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
