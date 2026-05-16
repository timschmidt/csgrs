//! Assembly, fixture-access, and DFA readiness checks.
//!
//! These checks operate on parsed KiCad pads/drills plus optional sidecars and
//! focus on whether a board package is ready for placement, probing, tooling,
//! and fine-pitch assembly review.
//!
//! Reliability note: assembly checks use copper footprints as proxies for real
//! bodies, tooling envelopes, and process keepouts. Suspect results need review
//! against the assembly drawing, package data, and fixture/process constraints.

use std::collections::{BTreeMap, BTreeSet};

use csgrs::csg::CSG;
use geo::{Area, BoundingRect};

use crate::checks::distance::polygon_boundary_distance;
use crate::checks::outline::{axis_aligned_outline_rect, feature_bounds_inside_rect_margin};
use crate::checks::spatial::{CopperSpatialIndex, PointSpatialIndex};
use crate::geometry::{circle_polygon, multipolygon_to_shapes, polygons_to_sketch};
use crate::ipc356::{Ipc356AccessSide, Ipc356FeatureType, Ipc356Point, Ipc356Soldermask};
use crate::kicad::{BoardModel, CopperFeature, CopperKind, DrillFeature};
use crate::report::{Severity, Violation};
use crate::{LayerMetadata, PcbSketch};

const TESTPOINT_GRID_EPSILON: f64 = 1.0e-9;
const FEATURE_GRID_EPSILON: f64 = 1.0e-9;

/// Review component-pad proxies against the assembly edge-clearance band.
///
/// IPC-7351B treats component placement courtyard and fabrication/assembly
/// tolerances as process constraints. Until parsed package courtyards are
/// available, this check uses non-fiducial KiCad pads as conservative body
/// proxies, then applies an Ericson-style rectangular broad phase before exact
/// outline distance review on simple board rectangles.
pub fn component_edge_clearance_readiness(
    board: &BoardModel,
    selected_layers: &[String],
    clearance: f64,
) -> Vec<Violation> {
    let Some(outline) = &board.board_outline else {
        return Vec::new();
    };

    let outline_rect = axis_aligned_outline_rect(outline);
    let candidate_pads = selected_copper_features(board, selected_layers)
        .into_iter()
        .filter(|feature| feature.kind == CopperKind::Pad)
        .filter(|feature| !likely_fiducial(feature))
        .filter(|feature| !feature.net.as_deref().is_some_and(looks_edge_intent_net))
        .collect::<Vec<_>>();
    let mut rect_rejections = 0_usize;
    let mut exact_checks = 0_usize;
    let violations = candidate_pads
        .iter()
        .filter_map(|feature| {
            if outline_rect
                .as_ref()
                .is_some_and(|rect| feature_bounds_inside_rect_margin(feature, rect, clearance))
            {
                rect_rejections += 1;
                return None;
            }
            exact_checks += 1;
            let edge_gap = polygon_boundary_distance(
                &feature.sketch.to_multipolygon(),
                &outline.to_multipolygon(),
            );
            (edge_gap < clearance).then(|| {
                Violation::new(
                    "component-edge-clearance-readiness",
                    Severity::Warning,
                    vec![feature.layer.clone()],
                    None,
                    Vec::new(),
                    vec![feature.location],
                    Some(format!(
                        "component pad is {edge_gap:.6} from board edge, below assembly edge clearance {clearance:.6}; review pick-and-place, depanelization, clamp, and rework access"
                    )),
                )
            })
        })
        .collect::<Vec<_>>();

    log::trace!(
        "component-edge clearance readiness: source={} pads={} rect_rejections={} exact_checks={} clearance={clearance:.6} violations={}",
        board.source,
        candidate_pads.len(),
        rect_rejections,
        exact_checks,
        violations.len()
    );

    violations
}

/// Run the `component_hole_clearance_readiness` design-readiness check or report helper.
pub fn component_hole_clearance_readiness(
    board: &BoardModel,
    extra_drills: &[DrillFeature],
    selected_layers: &[String],
    clearance: f64,
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

    let pads = selected_copper_features(board, selected_layers)
        .into_iter()
        .filter(|feature| feature.kind == CopperKind::Pad)
        .filter(|feature| !likely_fiducial(feature))
        .collect::<Vec<_>>();
    let pad_index = FeatureGridIndex::new(&pads, clearance);
    let mut violations = Vec::new();
    let mut candidate_count = 0_usize;
    log::trace!(
        "component-hole clearance readiness: source={} mechanical_drills={} pads={} buckets={} clearance={clearance:.6} min_area={min_area:.9}",
        board.source,
        mechanical_drills.len(),
        pads.len(),
        pad_index.bucket_count()
    );

    for drill in mechanical_drills {
        let keepout_radius = drill.diameter / 2.0 + clearance;
        let keepout = polygons_to_sketch(
            vec![circle_polygon(drill.location, keepout_radius, 32)],
            Some(LayerMetadata {
                name: "mechanical hole keepout".to_string(),
            }),
        );

        for pad_index in pad_index.near_circle(drill.location, keepout_radius) {
            candidate_count += 1;
            let pad = pads[pad_index];
            if !feature_may_touch_circle(pad, drill.location, keepout_radius) {
                continue;
            }
            let overlap = keepout.intersection(&pad.sketch);
            let shapes = multipolygon_to_shapes(&overlap.to_multipolygon(), min_area);
            let fallback_hit = shapes.is_empty()
                && polygon_boundary_distance(
                    &keepout.to_multipolygon(),
                    &pad.sketch.to_multipolygon(),
                ) <= 1.0e-9;
            if shapes.is_empty() && !fallback_hit {
                continue;
            }

            violations.push(Violation::new(
                "component-hole-clearance-readiness",
                Severity::Warning,
                vec![pad.layer.clone()],
                None,
                shapes,
                vec![drill.location, pad.location],
                Some(format!(
                    "component pad is within mechanical hole clearance {clearance:.6}; review screw, standoff, slot, chassis, or connector keepout"
                )),
            ));
        }
    }

    log::trace!(
        "component-hole clearance readiness: source={} candidate_pairs={} violations={}",
        board.source,
        candidate_count,
        violations.len()
    );

    violations
}

/// Run the `component_spacing_readiness` design-readiness check or report helper.
pub fn component_spacing_readiness(
    board: &BoardModel,
    selected_layers: &[String],
    clearance: f64,
    minimum_pad_dimension: f64,
) -> Vec<Violation> {
    let pads = selected_copper_features(board, selected_layers)
        .into_iter()
        .filter(|feature| feature.kind == CopperKind::Pad)
        .filter(|feature| !likely_fiducial(feature))
        .filter(|feature| minimum_bounding_dimension(&feature.sketch) >= minimum_pad_dimension)
        .collect::<Vec<_>>();
    let mut violations = Vec::new();

    let candidate_pairs = same_layer_feature_candidate_pairs(&pads, clearance);
    log::trace!(
        "component spacing readiness: source={} pads={} candidate_pairs={} clearance={clearance:.6} minimum_pad_dimension={minimum_pad_dimension:.6}",
        board.source,
        pads.len(),
        candidate_pairs.len()
    );
    for (left_index, right_index) in candidate_pairs {
        let left = pads[left_index];
        let right = pads[right_index];
        if !sketches_within_clearance(&left.sketch, &right.sketch, clearance) {
            continue;
        }

        let gap = polygon_boundary_distance(
            &left.sketch.to_multipolygon(),
            &right.sketch.to_multipolygon(),
        );
        if gap >= clearance {
            continue;
        }

        // Full component-to-component review needs courtyard/body data. Until
        // the KiCad model carries that, use only large pad copper as a
        // conservative proxy for connectors, modules, and bulky packages.
        // IPC-7351B frames land patterns and courtyard spacing as assembly
        // process constraints; this check is a review signal, not a final
        // courtyard DRC.
        violations.push(Violation::new(
            "component-spacing-readiness",
            Severity::Warning,
            vec![left.layer.clone()],
            None,
            Vec::new(),
            vec![left.location, right.location],
            Some(format!(
                "large component pad proxies are {gap:.6} apart, below assembly component spacing {clearance:.6}; review courtyard/body clearance and rework access"
            )),
        ));
    }

    violations
}

/// Run the `connector_rework_clearance_readiness` design-readiness check or report helper.
pub fn connector_rework_clearance_readiness(
    board: &BoardModel,
    selected_layers: &[String],
    clearance: f64,
    minimum_pad_dimension: f64,
) -> Vec<Violation> {
    let pads = selected_copper_features(board, selected_layers)
        .into_iter()
        .filter(|feature| feature.kind == CopperKind::Pad)
        .filter(|feature| !likely_fiducial(feature))
        .collect::<Vec<_>>();
    let connector_pads = pads
        .iter()
        .enumerate()
        .filter_map(|(index, feature)| {
            (feature.net.as_deref().is_some_and(looks_connector_net)
                && minimum_bounding_dimension(&feature.sketch) >= minimum_pad_dimension)
                .then_some(index)
        })
        .collect::<BTreeSet<_>>();
    let mut violations = Vec::new();

    let candidate_pairs = same_layer_feature_candidate_pairs(&pads, clearance);
    log::trace!(
        "connector rework clearance readiness: source={} pads={} connectors={} candidate_pairs={} clearance={clearance:.6} minimum_pad_dimension={minimum_pad_dimension:.6}",
        board.source,
        pads.len(),
        connector_pads.len(),
        candidate_pairs.len()
    );
    for (left_index, right_index) in candidate_pairs {
        let (connector_index, neighbor_index) = match (
            connector_pads.contains(&left_index),
            connector_pads.contains(&right_index),
        ) {
            (true, false) => (left_index, right_index),
            (false, true) => (right_index, left_index),
            // Connector-to-connector collisions are usually component-spacing
            // review, while non-connector pairs are irrelevant here.
            _ => continue,
        };
        let connector = pads[connector_index];
        let neighbor = pads[neighbor_index];
        if connector.net.is_some() && connector.net == neighbor.net {
            continue;
        }

        let gap = polygon_boundary_distance(
            &connector.sketch.to_multipolygon(),
            &neighbor.sketch.to_multipolygon(),
        );
        if gap >= clearance {
            continue;
        }

        // IPC-7711/7721 rework guidance treats connector removal/replacement as
        // a tool-access and thermal-control problem, not only an electrical DRC
        // issue. Candidate generation is broad-phase; this exact polygon gap
        // remains the finding decision.
        violations.push(Violation::new(
            "connector-rework-clearance-readiness",
            Severity::Warning,
            vec![connector.layer.clone()],
            None,
            Vec::new(),
            vec![connector.location, neighbor.location],
            Some(format!(
                "likely connector pad on net {:?} is {gap:.6} from neighboring pad, below rework clearance {clearance:.6}; review soldering iron and connector rework access",
                connector.net
            )),
        ));
    }

    violations
}

/// Warn when a likely two-terminal pad pair has asymmetric copper area.
///
/// The same wetting-force balance that drives tombstoning is sensitive to land
/// geometry: unequal pad areas change solder volume and wetting force. IPC-7351
/// land-pattern guidance and Eurocircuits' tombstoning notes both describe
/// symmetric pad geometry as a mitigation for chip resistors/capacitors.
pub fn pad_pair_asymmetry_readiness(
    board: &BoardModel,
    selected_layers: &[String],
    max_pair_gap: f64,
    max_area_ratio: f64,
    max_pad_dimension: f64,
) -> Vec<Violation> {
    let pads = selected_copper_features(board, selected_layers)
        .into_iter()
        .filter(|feature| feature.kind == CopperKind::Pad)
        .filter(|feature| !likely_fiducial(feature))
        .filter_map(|feature| {
            let area = feature.sketch.to_multipolygon().unsigned_area();
            let (_, max_dimension) = bounding_dimensions(&feature.sketch)?;
            (area > 0.0 && max_dimension <= max_pad_dimension).then_some((
                feature,
                area,
                max_dimension,
            ))
        })
        .collect::<Vec<_>>();
    let mut violations = Vec::new();

    let pad_features = pads
        .iter()
        .map(|(feature, _, _)| *feature)
        .collect::<Vec<_>>();
    let candidate_pairs = same_layer_feature_candidate_pairs(&pad_features, max_pair_gap);
    log::trace!(
        "pad-pair asymmetry readiness: source={} pads={} candidate_pairs={} max_pair_gap={max_pair_gap:.6} max_area_ratio={max_area_ratio:.3} max_pad_dimension={max_pad_dimension:.6}",
        board.source,
        pads.len(),
        candidate_pairs.len()
    );
    for (left_index, right_index) in candidate_pairs {
        let (left, left_area, _) = pads[left_index];
        let (right, right_area, _) = pads[right_index];
        if left.net.is_some() && left.net == right.net {
            continue;
        }
        if !sketches_within_clearance(&left.sketch, &right.sketch, max_pair_gap) {
            continue;
        }

        let gap = polygon_boundary_distance(
            &left.sketch.to_multipolygon(),
            &right.sketch.to_multipolygon(),
        );
        if gap > max_pair_gap {
            continue;
        }

        let area_ratio = left_area.max(right_area) / left_area.min(right_area);
        if area_ratio <= max_area_ratio {
            continue;
        }

        violations.push(Violation::new(
            "pad-pair-asymmetry-readiness",
            Severity::Warning,
            vec![left.layer.clone()],
            None,
            Vec::new(),
            vec![left.location, right.location],
            Some(format!(
                "neighboring small pads have copper area ratio {area_ratio:.3}, above {max_area_ratio:.3}; review two-terminal land pattern symmetry and tombstoning risk"
            )),
        ));
    }

    violations
}

/// Compare likely critical KiCad nets against IPC-D-356 test records.
///
/// IPC-9252-style fixture planning and practical DFT guidance such as the
/// FixturFab bed-of-nails notes both reduce to the same early-readiness signal:
/// every critical production net needs parsed, repeatable probe evidence before
/// the board is ready for fixture build. This check treats normalized IPC-D-356
/// net records as that evidence and reports missing critical KiCad nets.
pub fn testpoint_coverage_readiness(
    board: &BoardModel,
    points: &[Ipc356Point],
    selected_layers: &[String],
) -> Vec<Violation> {
    let covered_nets = points
        .iter()
        .map(|point| normalize_net(&point.net))
        .collect::<BTreeSet<_>>();
    let mut required_nets: BTreeMap<String, Vec<[f64; 2]>> = BTreeMap::new();
    let mut required_feature_count = 0_usize;

    for feature in selected_copper_features(board, selected_layers) {
        let Some(net) = feature.net.as_deref() else {
            continue;
        };
        if !looks_testpoint_required_net(net) {
            continue;
        }
        required_feature_count += 1;
        required_nets
            .entry(net.to_string())
            .or_default()
            .push(feature.location);
    }

    let violations = required_nets
        .into_iter()
        .filter(|(net, _)| !covered_nets.contains(&normalize_net(net)))
        .map(|(net, locations)| {
            Violation::new(
                "testpoint-coverage-readiness",
                Severity::Warning,
                vec![net.clone()],
                None,
                Vec::new(),
                locations.into_iter().take(3).collect(),
                Some(format!(
                    "critical net {net:?} has parsed KiCad copper but no matching IPC-D-356 test record"
                )),
            )
        })
        .collect::<Vec<_>>();

    log::trace!(
        "testpoint coverage readiness: source={} ipc_points={} covered_nets={} required_features={} missing_nets={} selected_layers={}",
        board.source,
        points.len(),
        covered_nets.len(),
        required_feature_count,
        violations.len(),
        selected_layers.len()
    );

    violations
}

/// Check fixture-probe diameter, edge clearance, and nearest-neighbor spacing.
///
/// Bed-of-nails fixture guidance from FixturFab and IPC-9252-oriented electrical
/// test practices describe the same mechanical idea this check encodes: test
/// probes need reliable pad size, spacing, and fixture clearance so the probe
/// plate can contact every required net repeatably.
pub fn testpoint_accessibility_readiness(
    board: &BoardModel,
    points: &[Ipc356Point],
    minimum_diameter: f64,
    minimum_spacing: f64,
    edge_clearance: f64,
) -> Vec<Violation> {
    let mut violations = Vec::new();
    let side_parity_features = board
        .copper
        .iter()
        .filter(|feature| matches!(feature.kind, CopperKind::Pad | CopperKind::Via))
        .filter(|feature| copper_layer_access_side(&feature.layer).is_some())
        .collect::<Vec<_>>();
    let side_parity_index = CopperSpatialIndex::new(
        &side_parity_features,
        minimum_diameter / 2.0 + minimum_spacing,
    );
    log::trace!(
        "testpoint accessibility readiness: source={} points={} side_parity_features={} side_parity_buckets={} minimum_diameter={minimum_diameter:.6} minimum_spacing={minimum_spacing:.6} edge_clearance={edge_clearance:.6}",
        board.source,
        points.len(),
        side_parity_features.len(),
        side_parity_index.bucket_count()
    );

    for point in points {
        if matches!(
            point.soldermask,
            Some(Ipc356Soldermask::Covered | Ipc356Soldermask::Unknown)
        ) {
            violations.push(Violation::new(
                "testpoint-accessibility-readiness",
                Severity::Warning,
                vec![format!("net:{}", point.net)],
                None,
                Vec::new(),
                vec![point.location],
                Some(if matches!(point.soldermask, Some(Ipc356Soldermask::Covered)) {
                    "IPC-D-356 testpoint is marked soldermask-covered; review probe opening or test access"
                        .to_string()
                } else {
                    "IPC-D-356 testpoint has unknown soldermask access; review exposed probe opening"
                        .to_string()
                }),
            ));
        }
        if point.soldermask.is_none()
            && matches!(
                point.feature_type,
                None | Some(Ipc356FeatureType::Smd | Ipc356FeatureType::ThroughHole)
            )
        {
            violations.push(Violation::new(
                "testpoint-accessibility-readiness",
                Severity::Warning,
                vec![format!("net:{}", point.net)],
                None,
                Vec::new(),
                vec![point.location],
                Some(
                    "IPC-D-356 testpoint has no soldermask access flag; review exposed probe opening"
                        .to_string(),
                ),
            ));
        }
        if point.access_side.is_none() {
            violations.push(Violation::new(
                "testpoint-accessibility-readiness",
                Severity::Warning,
                vec![format!("net:{}", point.net)],
                None,
                Vec::new(),
                vec![point.location],
                Some(
                    "IPC-D-356 testpoint has no parsed access side; review top/bottom fixture access"
                        .to_string(),
                ),
            ));
        }
        if matches!(point.access_side, Some(Ipc356AccessSide::Both))
            && matches!(point.feature_type, Some(Ipc356FeatureType::Smd))
        {
            violations.push(Violation::new(
                "testpoint-accessibility-readiness",
                Severity::Warning,
                vec![format!("net:{}", point.net)],
                None,
                Vec::new(),
                vec![point.location],
                Some(
                    "IPC-D-356 SMD testpoint is marked accessible from both sides; review fixture side intent"
                        .to_string(),
                ),
            ));
        }
        if let Some(side_violation) = testpoint_side_parity_violation(
            point,
            minimum_diameter,
            minimum_spacing,
            &side_parity_features,
            &side_parity_index,
        ) {
            violations.push(side_violation);
        }

        match point.diameter {
            Some(diameter) if diameter < minimum_diameter => {
                violations.push(Violation::new(
                    "testpoint-accessibility-readiness",
                    Severity::Warning,
                    vec![format!("net:{}", point.net)],
                    None,
                    Vec::new(),
                    vec![point.location],
                    Some(format!(
                        "IPC-D-356 testpoint diameter {diameter:.6} is below minimum probe diameter {minimum_diameter:.6}"
                    )),
                ));
            }
            Some(_) => {}
            None => {
                violations.push(Violation::new(
                    "testpoint-accessibility-readiness",
                    Severity::Warning,
                    vec![format!("net:{}", point.net)],
                    None,
                    Vec::new(),
                    vec![point.location],
                    Some(
                        "IPC-D-356 testpoint has no parsed probe diameter; review fixture probe access"
                            .to_string(),
                    ),
                ));
            }
        }

        if let Some(outline) = &board.board_outline {
            let probe_diameter = point.diameter.unwrap_or(minimum_diameter);
            let probe = circle_polygon(point.location, probe_diameter / 2.0, 32);
            let probe_sketch = polygons_to_sketch(
                vec![probe],
                Some(LayerMetadata {
                    name: "IPC-D-356 testpoint probe".to_string(),
                }),
            );
            let edge_gap = polygon_boundary_distance(
                &probe_sketch.to_multipolygon(),
                &outline.to_multipolygon(),
            );
            if edge_gap < edge_clearance {
                violations.push(Violation::new(
                    "testpoint-accessibility-readiness",
                    Severity::Warning,
                    vec![format!("net:{}", point.net)],
                    None,
                    Vec::new(),
                    vec![point.location],
                    Some(format!(
                        "IPC-D-356 testpoint is {edge_gap:.6} from board edge, below fixture edge clearance {edge_clearance:.6}"
                    )),
                ));
            }
        }
    }

    violations.extend(testpoint_spacing_violations(points, minimum_spacing));

    violations
}

fn testpoint_spacing_violations(points: &[Ipc356Point], minimum_spacing: f64) -> Vec<Violation> {
    let indexed_points = points
        .iter()
        .enumerate()
        .filter_map(|(index, point)| point.diameter.map(|diameter| (index, point, diameter)))
        .collect::<Vec<_>>();
    if indexed_points.len() < 2 {
        return Vec::new();
    }

    let maximum_diameter = indexed_points
        .iter()
        .map(|(_, _, diameter)| *diameter)
        .fold(0.0_f64, f64::max);
    let cell_size = (minimum_spacing + maximum_diameter).max(TESTPOINT_GRID_EPSILON);
    let mut buckets: BTreeMap<(i64, i64), Vec<(usize, &Ipc356Point, f64)>> = BTreeMap::new();
    for (index, point, diameter) in indexed_points {
        buckets
            .entry(testpoint_bucket(point.location, cell_size))
            .or_default()
            .push((index, point, diameter));
    }

    let mut comparisons = 0_usize;
    let mut violations = Vec::new();
    for (&(bucket_x, bucket_y), bucket_points) in &buckets {
        for &(left_index, left, left_diameter) in bucket_points {
            for x_delta in -1..=1 {
                for y_delta in -1..=1 {
                    let Some(candidate_points) =
                        buckets.get(&(bucket_x + x_delta, bucket_y + y_delta))
                    else {
                        continue;
                    };
                    for &(right_index, right, right_diameter) in candidate_points {
                        if right_index <= left_index {
                            continue;
                        }
                        comparisons += 1;
                        let edge_gap = distance(left.location, right.location)
                            - left_diameter / 2.0
                            - right_diameter / 2.0;
                        if edge_gap >= minimum_spacing {
                            continue;
                        }

                        violations.push(Violation::new(
                            "testpoint-accessibility-readiness",
                            Severity::Warning,
                            vec![format!("net:{}", left.net), format!("net:{}", right.net)],
                            None,
                            Vec::new(),
                            vec![left.location, right.location],
                            Some(format!(
                                "IPC-D-356 testpoint spacing {edge_gap:.6} is below fixture probe spacing {minimum_spacing:.6}"
                            )),
                        ));
                    }
                }
            }
        }
    }

    // The grid is a broad phase in the sense of Lin and Canny, "A Fast
    // Algorithm for Incremental Distance Calculation", IEEE ICRA, 1991: nearby
    // candidates are found cheaply, then exact Euclidean edge spacing remains
    // the narrow-phase decision used for the fixture-readiness finding.
    log::trace!(
        "testpoint spacing readiness: points={} buckets={} comparisons={} violations={} cell_size={cell_size:.6}",
        points.len(),
        buckets.len(),
        comparisons,
        violations.len()
    );

    violations
}

fn testpoint_bucket(location: [f64; 2], cell_size: f64) -> (i64, i64) {
    (
        (location[0] / cell_size).floor() as i64,
        (location[1] / cell_size).floor() as i64,
    )
}

/// Run the `testpoint_copper_clearance_readiness` design-readiness check or report helper.
///
/// The probe keepout is a circular fixture envelope around each IPC-D-356 test
/// access point. Candidate copper is found with the shared broad-phase grid
/// described by Ericson, *Real-Time Collision Detection* (2005), then exact CSG
/// intersection remains the narrow-phase decision. IPC-9252B-style DFT practice
/// treats probe access as both electrical and mechanical evidence; unrelated
/// nearby copper can create fixture shorts even when the IPC-D-356 record is
/// otherwise complete.
pub fn testpoint_copper_clearance_readiness(
    board: &BoardModel,
    points: &[Ipc356Point],
    selected_layers: &[String],
    minimum_diameter: f64,
    clearance: f64,
    min_area: f64,
) -> Vec<Violation> {
    let copper = selected_copper_features(board, selected_layers);
    let copper_index = CopperSpatialIndex::new(&copper, minimum_diameter / 2.0 + clearance);
    let mut violations = Vec::new();
    let mut candidate_count = 0_usize;
    log::trace!(
        "testpoint copper clearance readiness: source={} points={} copper={} buckets={} minimum_diameter={minimum_diameter:.6} clearance={clearance:.6} min_area={min_area:.9}",
        board.source,
        points.len(),
        copper.len(),
        copper_index.bucket_count()
    );

    for point in points {
        let probe_diameter = point
            .diameter
            .unwrap_or(minimum_diameter)
            .max(minimum_diameter);
        let keepout_radius = probe_diameter / 2.0 + clearance;
        let keepout = polygons_to_sketch(
            vec![circle_polygon(point.location, keepout_radius, 32)],
            Some(LayerMetadata {
                name: "IPC-D-356 probe copper keepout".to_string(),
            }),
        );
        let point_net = normalize_net(&point.net);

        for feature_index in copper_index.all_layers_near_circle(point.location, keepout_radius) {
            candidate_count += 1;
            let feature = copper[feature_index];
            if feature
                .net
                .as_deref()
                .is_some_and(|net| normalize_net(net) == point_net)
            {
                continue;
            }
            if !feature_may_touch_circle(feature, point.location, keepout_radius) {
                continue;
            }

            // IPC-9252B and DFT fixture practice treat probe access as both an
            // electrical and mechanical condition: nearby unrelated copper can
            // create fixture shorts or unreliable contact even when the
            // IPC-D-356 testpoint metadata itself is complete.
            let overlap = keepout.intersection(&feature.sketch);
            let shapes = multipolygon_to_shapes(&overlap.to_multipolygon(), min_area);
            let fallback_hit = shapes.is_empty()
                && polygon_boundary_distance(
                    &keepout.to_multipolygon(),
                    &feature.sketch.to_multipolygon(),
                ) <= 1.0e-9;
            if shapes.is_empty() && !fallback_hit {
                continue;
            }

            violations.push(Violation::new(
                "testpoint-copper-clearance-readiness",
                Severity::Warning,
                vec![feature.layer.clone(), format!("net:{}", point.net)],
                None,
                shapes,
                vec![point.location, feature.location],
                Some(format!(
                    "IPC-D-356 testpoint probe keepout {clearance:.6} around net {:?} intersects unrelated KiCad copper {:?}; review fixture short risk and probe clearance",
                    point.net, feature.net
                )),
            ));
        }
    }

    log::trace!(
        "testpoint copper clearance readiness: source={} candidate_pairs={} violations={}",
        board.source,
        candidate_count,
        violations.len()
    );

    violations
}

fn testpoint_side_parity_violation(
    point: &Ipc356Point,
    minimum_diameter: f64,
    minimum_spacing: f64,
    features: &[&CopperFeature],
    spatial_index: &CopperSpatialIndex<'_>,
) -> Option<Violation> {
    // IPC-D-356B carries electrical-test access evidence, while DFT fixture
    // guidance treats probe side as a production constraint; cross-checking the
    // sidecar against nearby KiCad copper catches common top/bottom export
    // mistakes before fixture build. See IPC-D-356B (IPC, 2002) and FixturFab,
    // "Design for Test: How to Design Test Points for PCB Testing."
    let expected_side = match point.access_side? {
        Ipc356AccessSide::Top => Ipc356AccessSide::Top,
        Ipc356AccessSide::Bottom => Ipc356AccessSide::Bottom,
        Ipc356AccessSide::Both => return None,
    };
    let search_radius =
        point.diameter.unwrap_or(minimum_diameter) / 2.0 + minimum_spacing.max(0.25);
    let point_net = normalize_net(&point.net);
    let mut nearby_sides = BTreeSet::new();
    let mut candidate_count = 0_usize;

    for feature_index in spatial_index.all_layers_near_circle(point.location, search_radius) {
        candidate_count += 1;
        let feature = features[feature_index];
        if !feature
            .net
            .as_deref()
            .is_some_and(|net| normalize_net(net) == point_net)
        {
            continue;
        }
        if distance(feature.location, point.location) > search_radius {
            continue;
        }
        if let Some(side) = copper_layer_access_side(&feature.layer) {
            nearby_sides.insert(side);
        }
    }
    log::trace!(
        "testpoint side parity: net={} candidates={} nearby_sides={} search_radius={search_radius:.6}",
        point.net,
        candidate_count,
        nearby_sides.len()
    );

    if nearby_sides.is_empty() || nearby_sides.contains(&expected_side) {
        return None;
    }

    let observed = nearby_sides
        .iter()
        .map(|side| match side {
            Ipc356AccessSide::Top => "top",
            Ipc356AccessSide::Bottom => "bottom",
            Ipc356AccessSide::Both => "both",
        })
        .collect::<Vec<_>>()
        .join("/");
    let expected = match expected_side {
        Ipc356AccessSide::Top => "top",
        Ipc356AccessSide::Bottom => "bottom",
        Ipc356AccessSide::Both => "both",
    };

    Some(Violation::new(
        "testpoint-accessibility-readiness",
        Severity::Warning,
        vec![format!("net:{}", point.net)],
        None,
        Vec::new(),
        vec![point.location],
        Some(format!(
            "IPC-D-356 testpoint access side is {expected}, but nearby same-net KiCad pad/via copper is only on {observed}; review fixture side and exported testpoint side"
        )),
    ))
}

fn copper_layer_access_side(layer: &str) -> Option<Ipc356AccessSide> {
    let normalized = layer.to_ascii_lowercase();
    if normalized == "f.cu"
        || normalized.contains("front")
        || normalized.contains("top")
        || normalized.contains("primary")
    {
        Some(Ipc356AccessSide::Top)
    } else if normalized == "b.cu"
        || normalized.contains("back")
        || normalized.contains("bottom")
        || normalized.contains("secondary")
    {
        Some(Ipc356AccessSide::Bottom)
    } else {
        None
    }
}

/// Run the `tooling_hole_readiness` design-readiness check or report helper.
///
/// Tooling holes are filtered by plated state and finished diameter before
/// outline proximity is checked. The edge review stays exact through
/// boundary-distance geometry, while trace counters make sparse drill-table
/// behavior visible in fixture and package smoke runs.
pub fn tooling_hole_readiness(
    board: &BoardModel,
    extra_drills: &[DrillFeature],
    minimum_diameter: f64,
    maximum_diameter: f64,
    edge_clearance: f64,
) -> Vec<Violation> {
    let mut drills = board.drills.clone();
    drills.extend_from_slice(extra_drills);

    let candidates = drills
        .iter()
        .filter(|drill| !drill.plated)
        .filter(|drill| drill.diameter >= minimum_diameter && drill.diameter <= maximum_diameter)
        .collect::<Vec<_>>();
    let mut violations = Vec::new();
    let mut edge_checks = 0_usize;

    if candidates.len() < 2 {
        violations.push(Violation::new(
            "tooling-hole-readiness",
            Severity::Warning,
            vec!["tooling-holes".to_string()],
            None,
            Vec::new(),
            candidates.iter().map(|drill| drill.location).collect(),
            Some(format!(
                "found {} likely tooling hole(s); assembly panels usually need at least two non-plated tooling holes between {minimum_diameter:.6} and {maximum_diameter:.6}",
                candidates.len()
            )),
        ));
    }

    if let Some(outline) = &board.board_outline {
        for drill in &candidates {
            edge_checks += 1;
            let keepout = polygons_to_sketch(
                vec![circle_polygon(drill.location, drill.diameter / 2.0, 32)],
                Some(LayerMetadata {
                    name: "tooling hole".to_string(),
                }),
            );
            let edge_gap =
                polygon_boundary_distance(&keepout.to_multipolygon(), &outline.to_multipolygon());
            if edge_gap >= edge_clearance {
                continue;
            }

            violations.push(Violation::new(
                "tooling-hole-readiness",
                Severity::Warning,
                vec!["tooling-holes".to_string()],
                None,
                Vec::new(),
                vec![drill.location],
                Some(format!(
                    "likely tooling hole is {edge_gap:.6} from board edge, below fixture edge clearance {edge_clearance:.6}"
                )),
            ));
        }
    }

    log::trace!(
        "tooling-hole readiness: source={} board_drills={} extra_drills={} candidates={} edge_checks={} minimum_diameter={minimum_diameter:.6} maximum_diameter={maximum_diameter:.6} edge_clearance={edge_clearance:.6} violations={}",
        board.source,
        board.drills.len(),
        extra_drills.len(),
        candidates.len(),
        edge_checks,
        violations.len()
    );

    violations
}

/// Run the `mouse_bite_readiness` design-readiness check or report helper.
///
/// Mouse-bite rows are center-spacing patterns, so candidate selection uses the
/// same Ericson-style broad/narrow phase as other readiness checks: a point grid
/// from *Real-Time Collision Detection* (2005) finds nearby drill centers, then
/// exact Euclidean spacing decides whether the drill pitch is too tight,
/// acceptable, or isolated beyond the maximum expected row pitch.
pub fn mouse_bite_readiness(
    board: &BoardModel,
    extra_drills: &[DrillFeature],
    minimum_diameter: f64,
    maximum_diameter: f64,
    minimum_spacing: f64,
    maximum_spacing: f64,
) -> Vec<Violation> {
    let mut drills = board.drills.clone();
    drills.extend_from_slice(extra_drills);
    let candidates = drills
        .iter()
        .filter(|drill| !drill.plated && drill.diameter <= maximum_diameter)
        .collect::<Vec<_>>();
    let candidate_points = candidates
        .iter()
        .map(|drill| drill.location)
        .collect::<Vec<_>>();
    let point_index = PointSpatialIndex::new(candidate_points, maximum_spacing);
    let mut violations = Vec::new();
    let mut neighbor_hits = 0_usize;

    for drill in &candidates {
        if drill.diameter >= minimum_diameter {
            continue;
        }

        violations.push(Violation::new(
            "mouse-bite-readiness",
            Severity::Warning,
            vec!["mouse-bites".to_string()],
            None,
            Vec::new(),
            vec![drill.location],
            Some(format!(
                "likely mouse-bite drill diameter {:.6} is below minimum {:.6}",
                drill.diameter, minimum_diameter
            )),
        ));
    }

    for (left_index, left) in candidates.iter().enumerate() {
        let nearby = point_index
            .centers_within(left.location, maximum_spacing)
            .into_iter()
            .filter(|&right_index| right_index != left_index)
            .collect::<Vec<_>>();
        neighbor_hits += nearby.len();

        let Some((right, center_spacing)) = nearby
            .into_iter()
            .map(|right_index| {
                (
                    candidates[right_index],
                    distance(left.location, candidates[right_index].location),
                )
            })
            .min_by(|left, right| left.1.total_cmp(&right.1))
        else {
            violations.push(Violation::new(
                "mouse-bite-readiness",
                Severity::Warning,
                vec!["mouse-bites".to_string()],
                None,
                Vec::new(),
                vec![left.location],
                Some(format!(
                    "likely mouse-bite drill has no neighboring mouse-bite center within maximum expected spacing {maximum_spacing:.6}"
                )),
            ));
            continue;
        };
        if center_spacing >= minimum_spacing && center_spacing <= maximum_spacing {
            continue;
        }

        violations.push(Violation::new(
            "mouse-bite-readiness",
            Severity::Warning,
            vec!["mouse-bites".to_string()],
            None,
            Vec::new(),
            vec![left.location, right.location],
            Some(format!(
                "likely mouse-bite drill center spacing {center_spacing:.6} is outside expected range {minimum_spacing:.6}..{maximum_spacing:.6}"
            )),
        ));
    }

    log::trace!(
        "mouse-bite readiness: source={} candidates={} point_buckets={} neighbor_hits={} minimum_diameter={minimum_diameter:.6} maximum_diameter={maximum_diameter:.6} minimum_spacing={minimum_spacing:.6} maximum_spacing={maximum_spacing:.6} violations={}",
        board.source,
        candidates.len(),
        point_index.bucket_count(),
        neighbor_hits,
        violations.len()
    );

    violations
}

/// Infer global fiducials and review count plus edge clearance by board side.
///
/// IPC-7351B treats fiducials as assembly registration targets with optical
/// keepout needs. This readiness check infers likely unnetted pad targets and,
/// on rectangular boards, uses the same broad/narrow phase from Ericson,
/// *Real-Time Collision Detection* (2005) to reject clearly interior candidates
/// before exact outline-distance review.
pub fn fiducial_readiness(
    board: &BoardModel,
    selected_layers: &[String],
    edge_clearance: f64,
) -> Vec<Violation> {
    let mut candidates_by_layer: BTreeMap<String, Vec<&CopperFeature>> = BTreeMap::new();
    for feature in selected_copper_features(board, selected_layers) {
        if likely_fiducial(feature) {
            candidates_by_layer
                .entry(feature.layer.clone())
                .or_default()
                .push(feature);
        }
    }

    let mut expected_layers = board
        .copper
        .iter()
        .filter(|feature| selected_layers.is_empty() || selected_layers.contains(&feature.layer))
        .map(|feature| feature.layer.clone())
        .collect::<BTreeSet<_>>();
    expected_layers.retain(|layer| layer == "F.Cu" || layer == "B.Cu");
    if expected_layers.is_empty() {
        expected_layers.extend(candidates_by_layer.keys().cloned());
    }

    let mut violations = Vec::new();
    let outline_rect = board
        .board_outline
        .as_ref()
        .and_then(axis_aligned_outline_rect);
    let mut edge_candidates = 0_usize;
    let mut rect_rejections = 0_usize;
    let mut exact_edge_checks = 0_usize;
    for layer in expected_layers {
        let candidates = candidates_by_layer.get(&layer).cloned().unwrap_or_default();
        if candidates.len() < 2 {
            violations.push(Violation::new(
                "fiducial-readiness",
                Severity::Warning,
                vec![layer.clone()],
                None,
                Vec::new(),
                candidates.iter().map(|feature| feature.location).collect(),
                Some(format!(
                    "layer {layer} has {} likely fiducial(s); assembly usually expects at least two per populated side",
                    candidates.len()
                )),
            ));
        }

        if let Some(outline) = &board.board_outline {
            for candidate in candidates {
                edge_candidates += 1;
                if outline_rect.as_ref().is_some_and(|rect| {
                    feature_bounds_inside_rect_margin(candidate, rect, edge_clearance)
                }) {
                    rect_rejections += 1;
                    continue;
                }
                exact_edge_checks += 1;
                let distance_to_edge = polygon_boundary_distance(
                    &candidate.sketch.to_multipolygon(),
                    &outline.to_multipolygon(),
                );
                if distance_to_edge >= edge_clearance {
                    continue;
                }
                violations.push(Violation::new(
                    "fiducial-readiness",
                    Severity::Warning,
                    vec![layer.clone()],
                    None,
                    Vec::new(),
                    vec![candidate.location],
                    Some(format!(
                        "likely fiducial is {:.6} from board edge, below clearance {:.6}",
                        distance_to_edge, edge_clearance
                    )),
                ));
            }
        }
    }

    log::trace!(
        "fiducial readiness: source={} layers={} edge_candidates={} rect_rejections={} exact_edge_checks={} edge_clearance={edge_clearance:.6} violations={}",
        board.source,
        candidates_by_layer.len(),
        edge_candidates,
        rect_rejections,
        exact_edge_checks,
        violations.len()
    );

    violations
}

/// Run the `fiducial_keepout_readiness` design-readiness check or report helper.
pub fn fiducial_keepout_readiness(
    board: &BoardModel,
    selected_layers: &[String],
    clearance: f64,
    min_area: f64,
) -> Vec<Violation> {
    let features = selected_copper_features(board, selected_layers);
    let fiducials = features
        .iter()
        .copied()
        .filter(|feature| likely_fiducial(feature))
        .collect::<Vec<_>>();
    let blockers = features
        .into_iter()
        .filter(|feature| !likely_fiducial(feature))
        .collect::<Vec<_>>();
    let blocker_index = FeatureGridIndex::new(&blockers, clearance);
    let mut violations = Vec::new();
    let mut candidate_count = 0_usize;
    log::trace!(
        "fiducial keepout readiness: source={} fiducials={} blockers={} buckets={} clearance={clearance:.6} min_area={min_area:.9}",
        board.source,
        fiducials.len(),
        blockers.len(),
        blocker_index.bucket_count()
    );

    for fiducial in fiducials {
        let keepout = fiducial.sketch.offset(clearance);
        for blocker_index in
            blocker_index.near_circle(fiducial.location, feature_query_radius(fiducial, clearance))
        {
            candidate_count += 1;
            let blocker = blockers[blocker_index];
            if fiducial.layer != blocker.layer {
                continue;
            }
            if !sketches_within_clearance(&fiducial.sketch, &blocker.sketch, clearance) {
                continue;
            }

            // IPC-7351B treats fiducials as assembly registration features. A
            // clear copper-free annulus around the target improves optical
            // contrast for placement cameras; this models that annulus as an
            // offset target region and reports same-layer copper intrusions.
            let overlap = keepout.intersection(&blocker.sketch);
            let shapes = multipolygon_to_shapes(&overlap.to_multipolygon(), min_area);
            let fallback_hit = shapes.is_empty()
                && polygon_boundary_distance(
                    &keepout.to_multipolygon(),
                    &blocker.sketch.to_multipolygon(),
                ) <= 1.0e-9;
            if shapes.is_empty() && !fallback_hit {
                continue;
            }

            violations.push(Violation::new(
                "fiducial-keepout-readiness",
                Severity::Warning,
                vec![fiducial.layer.clone()],
                None,
                shapes,
                vec![fiducial.location, blocker.location],
                Some(format!(
                    "likely fiducial has same-layer copper inside optical keepout {clearance:.6}; review placement-camera contrast, mask opening, and assembly fiducial keepout"
                )),
            ));
        }
    }

    log::trace!(
        "fiducial keepout readiness: source={} candidate_pairs={} violations={}",
        board.source,
        candidate_count,
        violations.len()
    );

    violations
}

/// Review solder-process clearance around likely through-hole solder features.
///
/// This is a process-readiness heuristic, not a solder-flow simulation. IPC
/// J-STD-001H treats through-hole soldering workmanship as process controlled;
/// hyperdrc therefore flags likely wave/selective solder features that are
/// close to other pads so the engineer can confirm pallet, solder-thief, and
/// masking intent before release.
pub fn selective_wave_solder_keepout_readiness(
    board: &BoardModel,
    selected_layers: &[String],
    keepout: f64,
    min_area: f64,
) -> Vec<Violation> {
    let pads = selected_copper_features(board, selected_layers)
        .into_iter()
        .filter(|feature| feature.kind == CopperKind::Pad)
        .filter(|feature| !likely_fiducial(feature))
        .collect::<Vec<_>>();
    let pad_index = FeatureGridIndex::new(&pads, keepout);
    let solder_drills = board
        .drills
        .iter()
        .filter(|drill| drill.plated)
        .filter(|drill| drill.net.as_deref().is_some_and(looks_solder_process_net))
        .collect::<Vec<_>>();
    let mut violations = Vec::new();
    let mut candidate_count = 0_usize;
    log::trace!(
        "selective/wave solder keepout readiness: source={} drills={} pads={} buckets={} keepout={keepout:.6} min_area={min_area:.9}",
        board.source,
        solder_drills.len(),
        pads.len(),
        pad_index.bucket_count()
    );

    for drill in solder_drills {
        let keepout_radius = drill.diameter / 2.0 + keepout;
        let keepout_sketch = polygons_to_sketch(
            vec![circle_polygon(drill.location, keepout_radius, 32)],
            Some(LayerMetadata {
                name: "selective/wave solder keepout".to_string(),
            }),
        );
        for pad_index in pad_index.near_circle(drill.location, keepout_radius) {
            candidate_count += 1;
            let pad = pads[pad_index];
            if drill.net.is_some() && drill.net == pad.net {
                continue;
            }
            if !feature_may_touch_circle(pad, drill.location, keepout_radius) {
                continue;
            }
            let overlap = keepout_sketch.intersection(&pad.sketch);
            let shapes = multipolygon_to_shapes(&overlap.to_multipolygon(), min_area);
            let touching = shapes.is_empty()
                && polygon_boundary_distance(
                    &keepout_sketch.to_multipolygon(),
                    &pad.sketch.to_multipolygon(),
                ) <= 1.0e-9;
            if shapes.is_empty() && !touching {
                continue;
            }

            violations.push(Violation::new(
                "selective-wave-solder-keepout-readiness",
                Severity::Warning,
                vec![pad.layer.clone()],
                None,
                shapes,
                vec![drill.location, pad.location],
                Some(format!(
                    "likely through-hole solder feature on net {:?} is within solder-process keepout {keepout:.6} of neighboring pad {:?}; review selective/wave solder pallet, solder thief, and masking clearance",
                    drill.net, pad.net
                )),
            ));
        }
    }

    log::trace!(
        "selective/wave solder keepout readiness: source={} candidate_pairs={} violations={}",
        board.source,
        candidate_count,
        violations.len()
    );

    violations
}

/// Review press-fit insertion clearance around likely connector holes.
///
/// Press-fit hardware needs insertion-tool and deformation clearance that is not
/// represented by copper clearance alone. This check intentionally keys off
/// connector-like net names and plated drill geometry so it stays conservative.
pub fn press_fit_keepout_readiness(
    board: &BoardModel,
    selected_layers: &[String],
    keepout: f64,
    min_area: f64,
) -> Vec<Violation> {
    process_drill_keepout_readiness(
        board,
        selected_layers,
        keepout,
        min_area,
        "press-fit-keepout-readiness",
        "press-fit insertion",
        looks_press_fit_net,
    )
}

/// Review coating-mask clearance around likely contacts, fiducials, and probes.
///
/// IPC J-STD-001H treats conformal coating as a workmanship/process control
/// item. Geometry cannot prove a coating mask exists, but nearby copper around
/// likely no-coat features is a useful release-review prompt. Neighboring pad
/// candidates use the lightweight assembly feature grid, following Ericson,
/// *Real-Time Collision Detection* (2005), before exact keepout/pad CSG
/// intersection.
pub fn conformal_coating_keepout_readiness(
    board: &BoardModel,
    selected_layers: &[String],
    keepout: f64,
    min_area: f64,
) -> Vec<Violation> {
    let features = selected_copper_features(board, selected_layers);
    let no_coat_features = features
        .iter()
        .copied()
        .filter(|feature| likely_no_coat_feature(feature))
        .collect::<Vec<_>>();
    let pads = features
        .iter()
        .copied()
        .filter(|feature| feature.kind == CopperKind::Pad)
        .collect::<Vec<_>>();
    let pad_index = FeatureGridIndex::new(&pads, keepout);
    let mut violations = Vec::new();
    let mut candidate_count = 0usize;

    for &no_coat in &no_coat_features {
        let keepout_sketch = no_coat.sketch.offset(keepout);
        let query_radius = feature_query_radius(no_coat, keepout);
        for neighbor_index in pad_index.near_circle(no_coat.location, query_radius) {
            candidate_count += 1;
            let neighbor = pads[neighbor_index];
            if std::ptr::eq(no_coat, neighbor) {
                continue;
            }
            if no_coat.layer != neighbor.layer {
                continue;
            }
            if !sketches_within_clearance(&no_coat.sketch, &neighbor.sketch, keepout) {
                continue;
            }
            if no_coat.net.is_some() && no_coat.net == neighbor.net {
                continue;
            }
            let overlap = keepout_sketch.intersection(&neighbor.sketch);
            let shapes = multipolygon_to_shapes(&overlap.to_multipolygon(), min_area);
            if shapes.is_empty() {
                continue;
            }

            violations.push(Violation::new(
                "conformal-coating-keepout-readiness",
                Severity::Warning,
                vec![no_coat.layer.clone()],
                None,
                shapes,
                vec![no_coat.location, neighbor.location],
                Some(format!(
                    "likely no-coat feature {:?} has neighboring pad {:?} inside coating keepout {keepout:.6}; review conformal-coating mask, cleanliness, and contact/test access",
                    no_coat.net, neighbor.net
                )),
            ));
        }
    }

    log::trace!(
        "conformal-coating keepout readiness: source={} no_coat_features={} pads={} buckets={} candidate_pairs={} keepout={keepout:.6} violations={}",
        board.source,
        no_coat_features.len(),
        pads.len(),
        pad_index.bucket_count(),
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

fn minimum_bounding_dimension(sketch: &PcbSketch) -> f64 {
    sketch
        .geometry
        .bounding_rect()
        .map(|bounds| (bounds.max().x - bounds.min().x).min(bounds.max().y - bounds.min().y))
        .unwrap_or(0.0)
}

fn bounding_dimensions(sketch: &PcbSketch) -> Option<(f64, f64)> {
    sketch.geometry.bounding_rect().map(|bounds| {
        let width = bounds.max().x - bounds.min().x;
        let height = bounds.max().y - bounds.min().y;
        (width.min(height), width.max(height))
    })
}

fn feature_query_radius(feature: &CopperFeature, clearance: f64) -> f64 {
    feature
        .sketch
        .geometry
        .bounding_rect()
        .map(|bounds| {
            let width = bounds.max().x - bounds.min().x;
            let height = bounds.max().y - bounds.min().y;
            (width.hypot(height) / 2.0) + clearance
        })
        .unwrap_or(clearance)
}

struct FeatureGridIndex<'a> {
    features: &'a [&'a CopperFeature],
    buckets: BTreeMap<(i64, i64), Vec<usize>>,
    cell_size: f64,
    maximum_dimension: f64,
}

impl<'a> FeatureGridIndex<'a> {
    fn new(features: &'a [&'a CopperFeature], clearance: f64) -> Self {
        let maximum_dimension = features
            .iter()
            .filter_map(|feature| bounding_dimensions(&feature.sketch).map(|(_, maximum)| maximum))
            .fold(0.0_f64, f64::max);
        let cell_size = (maximum_dimension + clearance).max(FEATURE_GRID_EPSILON);
        let mut buckets: BTreeMap<(i64, i64), Vec<usize>> = BTreeMap::new();
        for (index, feature) in features.iter().enumerate() {
            buckets
                .entry(feature_bucket(feature.location, cell_size))
                .or_default()
                .push(index);
        }

        Self {
            features,
            buckets,
            cell_size,
            maximum_dimension,
        }
    }

    fn bucket_count(&self) -> usize {
        self.buckets.len()
    }

    fn near_circle(&self, center: [f64; 2], radius: f64) -> Vec<usize> {
        if self.features.is_empty() {
            return Vec::new();
        }

        let query_radius = radius + self.maximum_dimension / 2.0;
        let min_bucket = feature_bucket(
            [center[0] - query_radius, center[1] - query_radius],
            self.cell_size,
        );
        let max_bucket = feature_bucket(
            [center[0] + query_radius, center[1] + query_radius],
            self.cell_size,
        );
        let mut candidates = Vec::new();
        for bucket_x in min_bucket.0..=max_bucket.0 {
            for bucket_y in min_bucket.1..=max_bucket.1 {
                if let Some(indices) = self.buckets.get(&(bucket_x, bucket_y)) {
                    candidates.extend(indices.iter().copied());
                }
            }
        }

        // Broad-phase bucket lookup follows the same collision-query structure
        // as Lin and Canny, "A Fast Algorithm for Incremental Distance
        // Calculation", IEEE ICRA, 1991. The caller still checks bounding boxes
        // and exact polygon overlap before reporting a readiness finding.
        log::trace!(
            "feature grid circle query: center=({:.6},{:.6}) radius={radius:.6} query_radius={query_radius:.6} candidates={} cell_size={:.6}",
            center[0],
            center[1],
            candidates.len(),
            self.cell_size
        );

        candidates
    }
}

fn same_layer_feature_candidate_pairs(
    features: &[&CopperFeature],
    clearance: f64,
) -> Vec<(usize, usize)> {
    if features.len() < 2 {
        return Vec::new();
    }

    let maximum_dimension = features
        .iter()
        .filter_map(|feature| bounding_dimensions(&feature.sketch).map(|(_, maximum)| maximum))
        .fold(0.0_f64, f64::max);
    let cell_size = (maximum_dimension + clearance).max(FEATURE_GRID_EPSILON);
    let mut buckets: BTreeMap<(String, i64, i64), Vec<usize>> = BTreeMap::new();
    for (index, feature) in features.iter().enumerate() {
        let (bucket_x, bucket_y) = feature_bucket(feature.location, cell_size);
        buckets
            .entry((feature.layer.clone(), bucket_x, bucket_y))
            .or_default()
            .push(index);
    }

    let mut pairs = Vec::new();
    for ((layer, bucket_x, bucket_y), bucket_indices) in &buckets {
        for &left_index in bucket_indices {
            for x_delta in -1..=1 {
                for y_delta in -1..=1 {
                    let Some(candidate_indices) =
                        buckets.get(&(layer.clone(), bucket_x + x_delta, bucket_y + y_delta))
                    else {
                        continue;
                    };
                    for &right_index in candidate_indices {
                        if right_index > left_index {
                            pairs.push((left_index, right_index));
                        }
                    }
                }
            }
        }
    }

    // This is the same broad/narrow phase structure used for fixture spacing:
    // candidate generation follows Lin and Canny, "A Fast Algorithm for
    // Incremental Distance Calculation", IEEE ICRA, 1991, while the caller
    // still performs exact geometry distance checks before reporting.
    log::trace!(
        "same-layer feature candidate grid: features={} buckets={} pairs={} cell_size={cell_size:.6}",
        features.len(),
        buckets.len(),
        pairs.len()
    );

    pairs
}

fn feature_bucket(location: [f64; 2], cell_size: f64) -> (i64, i64) {
    (
        (location[0] / cell_size).floor() as i64,
        (location[1] / cell_size).floor() as i64,
    )
}

fn sketches_within_clearance(left: &PcbSketch, right: &PcbSketch, clearance: f64) -> bool {
    let Some(left_bounds) = left.geometry.bounding_rect() else {
        return true;
    };
    let Some(right_bounds) = right.geometry.bounding_rect() else {
        return true;
    };

    // AABB broad-phase before exact segment/polygon distance. This follows the
    // broad/narrow phase collision structure from Lin and Canny, "A Fast
    // Algorithm for Incremental Distance Calculation", IEEE ICRA, 1991.
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

fn likely_fiducial(feature: &CopperFeature) -> bool {
    if feature.kind != CopperKind::Pad || feature.net.is_some() {
        return false;
    }

    let Some(bounds) = feature.sketch.geometry.bounding_rect() else {
        return false;
    };
    let width = bounds.max().x - bounds.min().x;
    let height = bounds.max().y - bounds.min().y;
    let min_dimension = width.min(height);
    let max_dimension = width.max(height);

    min_dimension >= 0.5 && max_dimension <= 2.5 && min_dimension / max_dimension >= 0.75
}

fn distance(left: [f64; 2], right: [f64; 2]) -> f64 {
    let dx = left[0] - right[0];
    let dy = left[1] - right[1];
    (dx * dx + dy * dy).sqrt()
}

fn process_drill_keepout_readiness(
    board: &BoardModel,
    selected_layers: &[String],
    keepout: f64,
    min_area: f64,
    check: &str,
    process_label: &str,
    net_predicate: fn(&str) -> bool,
) -> Vec<Violation> {
    let pads = selected_copper_features(board, selected_layers)
        .into_iter()
        .filter(|feature| feature.kind == CopperKind::Pad)
        .filter(|feature| !likely_fiducial(feature))
        .collect::<Vec<_>>();
    let pad_index = FeatureGridIndex::new(&pads, keepout);
    let drills = board
        .drills
        .iter()
        .filter(|drill| drill.plated)
        .filter(|drill| drill.net.as_deref().is_some_and(net_predicate))
        .collect::<Vec<_>>();
    let mut violations = Vec::new();
    let mut candidate_count = 0_usize;
    log::trace!(
        "{check}: source={} process={} drills={} pads={} buckets={} keepout={keepout:.6} min_area={min_area:.9}",
        board.source,
        process_label,
        drills.len(),
        pads.len(),
        pad_index.bucket_count()
    );

    for drill in drills {
        let keepout_radius = drill.diameter / 2.0 + keepout;
        let keepout_sketch = polygons_to_sketch(
            vec![circle_polygon(drill.location, keepout_radius, 32)],
            Some(LayerMetadata {
                name: format!("{process_label} keepout"),
            }),
        );
        for pad_index in pad_index.near_circle(drill.location, keepout_radius) {
            candidate_count += 1;
            let pad = pads[pad_index];
            if drill.net.is_some() && drill.net == pad.net {
                continue;
            }
            if !feature_may_touch_circle(pad, drill.location, keepout_radius) {
                continue;
            }
            let overlap = keepout_sketch.intersection(&pad.sketch);
            let shapes = multipolygon_to_shapes(&overlap.to_multipolygon(), min_area);
            if shapes.is_empty() {
                continue;
            }
            violations.push(Violation::new(
                check,
                Severity::Warning,
                vec![pad.layer.clone()],
                None,
                shapes,
                vec![drill.location, pad.location],
                Some(format!(
                    "likely {process_label} feature on net {:?} is within keepout {keepout:.6} of neighboring pad {:?}; review insertion tooling, component keepout, and assembly drawing notes",
                    drill.net, pad.net
                )),
            ));
        }
    }

    log::trace!(
        "{check}: source={} candidate_pairs={} violations={}",
        board.source,
        candidate_count,
        violations.len()
    );

    violations
}

fn looks_edge_intent_net(net: &str) -> bool {
    looks_gold_finger_net(net) || looks_chassis_net(net)
}

fn looks_gold_finger_net(net: &str) -> bool {
    let normalized = net.to_ascii_uppercase();
    ["GOLD", "FINGER", "EDGE", "CARD_EDGE", "CONN_EDGE"]
        .iter()
        .any(|token| normalized.contains(token))
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

fn looks_connector_net(net: &str) -> bool {
    let normalized = net.to_ascii_uppercase();
    [
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
    ]
    .iter()
    .any(|token| normalized.contains(token))
}

fn looks_solder_process_net(net: &str) -> bool {
    let normalized = net.to_ascii_uppercase();
    looks_connector_net(net)
        || [
            "THT",
            "THROUGH",
            "PTH",
            "WAVE",
            "SELECTIVE",
            "HEADER",
            "PIN",
        ]
        .iter()
        .any(|token| normalized.contains(token))
}

fn looks_press_fit_net(net: &str) -> bool {
    let normalized = net.to_ascii_uppercase();
    looks_connector_net(net)
        || ["PRESS", "PRESSFIT", "PRESS_FIT", "PIN", "BACKPLANE"]
            .iter()
            .any(|token| normalized.contains(token))
}

fn likely_no_coat_feature(feature: &CopperFeature) -> bool {
    likely_fiducial(feature)
        || feature
            .net
            .as_deref()
            .is_some_and(|net| looks_connector_net(net) || looks_testpoint_required_net(net))
}

fn looks_testpoint_required_net(net: &str) -> bool {
    let normalized = net.to_ascii_uppercase();
    let tokens = [
        "RESET", "RST", "BOOT", "JTAG", "SWD", "SWCLK", "SWDIO", "TCK", "TMS", "TDI", "TDO",
        "UART", "TXD", "RXD", "DEBUG", "PROG", "TEST",
    ];

    looks_ground_net(net)
        || looks_high_current_net(net)
        || looks_high_speed_net(net)
        || looks_high_voltage_net(net)
        || looks_sensitive_net(net)
        || looks_chassis_net(net)
        || tokens.iter().any(|token| normalized.contains(token))
}

fn looks_ground_net(net: &str) -> bool {
    let normalized = net.to_ascii_uppercase();
    matches!(
        normalized.as_str(),
        "GND" | "GROUND" | "PGND" | "AGND" | "DGND"
    ) || normalized.ends_with("_GND")
        || normalized.ends_with("-GND")
}

fn looks_high_current_net(net: &str) -> bool {
    let normalized = net.to_ascii_uppercase();
    [
        "VBAT", "VBUS", "VIN", "VCC", "VDD", "VOUT", "PWR", "POWER", "MOTOR", "PHASE", "+12V",
        "+5V", "+3V3", "12V", "5V", "3V3", "1V8",
    ]
    .iter()
    .any(|token| normalized.contains(token))
}

fn looks_high_speed_net(net: &str) -> bool {
    let normalized = net.to_ascii_uppercase();
    [
        "USB", "D+", "D-", "DP", "DM", "CLK", "CLOCK", "TX", "RX", "SERDES", "PCIE", "PCI", "MIPI",
        "LVDS", "HDMI", "ETH", "RGMII", "SGMII", "SATA", "CAN",
    ]
    .iter()
    .any(|token| normalized.contains(token))
}

fn looks_high_voltage_net(net: &str) -> bool {
    let normalized = net.to_ascii_uppercase();
    [
        "HV", "HIGHV", "MAINS", "LINE", "NEUTRAL", "LIVE", "VAC", "AC_L", "AC_N", "RECT", "BULK",
        "400V", "240V", "230V", "120V", "48V",
    ]
    .iter()
    .any(|token| normalized.contains(token))
}

fn looks_sensitive_net(net: &str) -> bool {
    let normalized = net.to_ascii_uppercase();
    [
        "RF", "ANT", "AUDIO", "MIC", "ADC", "DAC", "AIN", "AOUT", "ANALOG", "SENSE", "SNS", "XTAL",
        "CRYSTAL", "OSC",
    ]
    .iter()
    .any(|token| normalized.contains(token))
}

fn normalize_net(net: &str) -> String {
    net.trim().to_ascii_uppercase()
}

#[cfg(test)]
mod tests {
    use super::{
        component_hole_clearance_readiness, component_spacing_readiness,
        conformal_coating_keepout_readiness, connector_rework_clearance_readiness,
        fiducial_keepout_readiness, mouse_bite_readiness, pad_pair_asymmetry_readiness,
        press_fit_keepout_readiness, selective_wave_solder_keepout_readiness,
        testpoint_accessibility_readiness, testpoint_copper_clearance_readiness,
    };
    use crate::LayerMetadata;
    use crate::geometry::{polygons_to_sketch, rect_polygon};
    use crate::ipc356::{Ipc356AccessSide, Ipc356FeatureType, Ipc356Point, Ipc356Soldermask};
    use crate::kicad::{BoardModel, CopperFeature, CopperKind, DrillFeature};

    #[test]
    fn pad_pair_asymmetry_readiness_reports_mismatched_neighbor_pads() {
        let board = board_with_copper(vec![
            copper_pad("A", [0.0, 0.0], 0.5, 0.5),
            copper_pad("B", [0.7, 0.0], 1.1, 0.5),
        ]);

        let violations = pad_pair_asymmetry_readiness(&board, &[], 0.3, 1.5, 2.0);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "pad-pair-asymmetry-readiness");
        assert!(
            violations[0]
                .message
                .as_deref()
                .is_some_and(|message| message.contains("tombstoning"))
        );
    }

    #[test]
    fn pad_pair_asymmetry_readiness_allows_balanced_distant_or_large_pads() {
        let balanced = board_with_copper(vec![
            copper_pad("A", [0.0, 0.0], 0.5, 0.5),
            copper_pad("B", [0.7, 0.0], 0.55, 0.5),
        ]);
        let distant = board_with_copper(vec![
            copper_pad("A", [0.0, 0.0], 0.5, 0.5),
            copper_pad("B", [2.0, 0.0], 1.1, 0.5),
        ]);
        let large_connector = board_with_copper(vec![
            copper_pad("A", [0.0, 0.0], 2.5, 0.5),
            copper_pad("B", [0.8, 0.0], 0.5, 0.5),
        ]);

        assert!(pad_pair_asymmetry_readiness(&balanced, &[], 0.3, 1.5, 2.0).is_empty());
        assert!(pad_pair_asymmetry_readiness(&distant, &[], 0.3, 1.5, 2.0).is_empty());
        assert!(pad_pair_asymmetry_readiness(&large_connector, &[], 0.3, 1.5, 2.0).is_empty());
    }

    #[test]
    fn pad_pair_asymmetry_readiness_culls_sparse_pad_fields() {
        let mut copper = (0..2_000)
            .map(|index| {
                copper_pad(
                    &format!("R{index}"),
                    [(index % 50) as f64 * 5.0, (index / 50) as f64 * 5.0],
                    0.5,
                    0.5,
                )
            })
            .collect::<Vec<_>>();
        copper.push(copper_pad("NEAR_A", [-10.0, -10.0], 0.5, 0.5));
        copper.push(copper_pad("NEAR_B", [-9.35, -10.0], 1.1, 0.5));
        let board = board_with_copper(copper);

        let start = std::time::Instant::now();
        let violations = pad_pair_asymmetry_readiness(&board, &[], 0.3, 1.5, 2.0);

        assert_eq!(violations.len(), 1);
        assert!(
            start.elapsed() < std::time::Duration::from_secs(2),
            "pad-pair asymmetry should cull distant sparse fields by grid bucket"
        );
    }

    #[test]
    fn selective_wave_solder_keepout_reports_neighboring_pad() {
        let board = board_with_copper_and_drills(
            vec![copper_pad("SIG", [0.35, 0.0], 0.25, 0.25)],
            vec![plated_drill("J1_PIN1", [0.0, 0.0], 0.6)],
        );

        let violations = selective_wave_solder_keepout_readiness(&board, &[], 0.25, 1.0e-9);

        assert_eq!(violations.len(), 1);
        assert_eq!(
            violations[0].check,
            "selective-wave-solder-keepout-readiness"
        );
    }

    #[test]
    fn press_fit_keepout_reports_neighboring_pad() {
        let board = board_with_copper_and_drills(
            vec![copper_pad("SIG", [0.45, 0.0], 0.25, 0.25)],
            vec![plated_drill("PRESS_FIT_CONN", [0.0, 0.0], 0.6)],
        );

        let violations = press_fit_keepout_readiness(&board, &[], 0.35, 1.0e-9);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "press-fit-keepout-readiness");
    }

    #[test]
    fn conformal_coating_keepout_reports_contact_neighbor() {
        let board = board_with_copper(vec![
            copper_pad("USB_DP", [0.0, 0.0], 0.4, 0.4),
            copper_pad("SIG", [0.55, 0.0], 0.3, 0.3),
        ]);

        let violations = conformal_coating_keepout_readiness(&board, &[], 0.3, 1.0e-9);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "conformal-coating-keepout-readiness");
    }

    #[test]
    fn conformal_coating_keepout_culls_large_sparse_pad_fields() {
        let mut copper = vec![copper_pad("USB_DP", [0.0, 0.0], 0.4, 0.4)];
        for index in 0..2_000 {
            copper.push(copper_pad(
                &format!("SIG{index}"),
                [
                    10.0 + (index % 50) as f64 * 3.0,
                    10.0 + (index / 50) as f64 * 3.0,
                ],
                0.3,
                0.3,
            ));
        }
        copper.push(copper_pad("SIG_NEAR", [0.55, 0.0], 0.3, 0.3));
        let board = board_with_copper(copper);

        let start = std::time::Instant::now();
        let violations = conformal_coating_keepout_readiness(&board, &[], 0.3, 1.0e-9);

        assert_eq!(violations.len(), 1);
        assert!(
            start.elapsed() < std::time::Duration::from_secs(2),
            "conformal-coating keepout should cull distant pads by spatial index"
        );
    }

    #[test]
    fn process_keepouts_allow_distant_or_unmatched_features() {
        let board = board_with_copper_and_drills(
            vec![
                copper_pad("SIG", [3.0, 0.0], 0.25, 0.25),
                copper_pad("GND", [4.0, 0.0], 0.25, 0.25),
            ],
            vec![plated_drill("NET1", [0.0, 0.0], 0.6)],
        );

        assert!(selective_wave_solder_keepout_readiness(&board, &[], 0.25, 1.0e-9).is_empty());
        assert!(press_fit_keepout_readiness(&board, &[], 0.35, 1.0e-9).is_empty());
        assert!(conformal_coating_keepout_readiness(&board, &[], 0.3, 1.0e-9).is_empty());
    }

    #[test]
    fn mouse_bite_readiness_culls_sparse_drill_rows() {
        let mut drills = (0..1_000)
            .flat_map(|index| {
                let x = index as f64 * 10.0;
                [
                    DrillFeature {
                        location: [x, 0.0],
                        diameter: 0.30,
                        net: None,
                        plated: false,
                    },
                    DrillFeature {
                        location: [x + 0.70, 0.0],
                        diameter: 0.30,
                        net: None,
                        plated: false,
                    },
                ]
            })
            .collect::<Vec<_>>();
        drills.push(DrillFeature {
            location: [50_000.0, 0.0],
            diameter: 0.30,
            net: None,
            plated: false,
        });

        let start = std::time::Instant::now();
        let violations = mouse_bite_readiness(
            &board_with_copper(Vec::new()),
            &drills,
            0.25,
            0.50,
            0.40,
            1.20,
        );

        assert_eq!(violations.len(), 1);
        assert!(
            violations[0]
                .message
                .as_deref()
                .is_some_and(|message| message.contains("no neighboring mouse-bite center"))
        );
        assert!(
            start.elapsed() < std::time::Duration::from_secs(2),
            "mouse-bite spacing should use point buckets instead of all-candidate nearest scans"
        );
    }

    #[test]
    fn process_drill_keepouts_cull_sparse_pad_fields() {
        let copper = (0..2_000)
            .map(|index| {
                copper_pad(
                    &format!("SIG{index}"),
                    [(index % 50) as f64 * 5.0, (index / 50) as f64 * 5.0],
                    0.25,
                    0.25,
                )
            })
            .chain([
                copper_pad("SIG_SELECTIVE", [-9.62, -10.0], 0.25, 0.25),
                copper_pad("SIG_PRESS", [-9.55, -8.0], 0.25, 0.25),
            ])
            .collect::<Vec<_>>();
        let mut drills = (0..400)
            .map(|index| {
                plated_drill(
                    "PIN_REMOTE",
                    [500.0 + (index % 40) as f64 * 5.0, (index / 40) as f64 * 5.0],
                    0.5,
                )
            })
            .collect::<Vec<_>>();
        drills.push(plated_drill("WAVE_SOLDER", [-10.0, -10.0], 0.5));
        drills.push(plated_drill("PRESS_FIT_CONN", [-10.0, -8.0], 0.5));
        let board = board_with_copper_and_drills(copper, drills);

        let start = std::time::Instant::now();
        let selective = selective_wave_solder_keepout_readiness(&board, &[], 0.25, 1.0e-9);
        let press = press_fit_keepout_readiness(&board, &[], 0.35, 1.0e-9);

        assert_eq!(selective.len(), 2);
        assert_eq!(press.len(), 1);
        assert!(
            start.elapsed() < std::time::Duration::from_secs(2),
            "process drill keepouts should cull distant sparse pad fields by grid bucket"
        );
    }

    #[test]
    fn component_hole_clearance_readiness_reports_pad_near_npth() {
        let board = board_with_copper_and_drills(
            vec![copper_pad("SIG", [0.45, 0.0], 0.25, 0.25)],
            vec![npth_drill([0.0, 0.0], 0.5)],
        );

        let violations = component_hole_clearance_readiness(&board, &[], &[], 0.25, 1.0e-9);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "component-hole-clearance-readiness");
        assert_eq!(violations[0].locations, vec![[0.0, 0.0], [0.45, 0.0]]);
    }

    #[test]
    fn component_hole_clearance_readiness_culls_sparse_pad_and_hole_fields() {
        let copper = (0..2_000)
            .map(|index| {
                copper_pad(
                    &format!("SIG{index}"),
                    [(index % 50) as f64 * 5.0, (index / 50) as f64 * 5.0],
                    0.4,
                    0.4,
                )
            })
            .collect::<Vec<_>>();
        let mut drills = (0..400)
            .map(|index| {
                npth_drill(
                    [500.0 + (index % 40) as f64 * 5.0, (index / 40) as f64 * 5.0],
                    0.5,
                )
            })
            .collect::<Vec<_>>();
        drills.push(npth_drill([0.18, 0.0], 0.5));
        let board = board_with_copper_and_drills(copper, drills);

        let start = std::time::Instant::now();
        let violations = component_hole_clearance_readiness(&board, &[], &[], 0.25, 1.0e-9);

        assert_eq!(violations.len(), 1);
        assert!(
            start.elapsed() < std::time::Duration::from_secs(2),
            "component-hole clearance should cull distant pads by grid bucket"
        );
    }

    #[test]
    fn component_spacing_readiness_reports_close_large_pad_proxies() {
        let board = board_with_copper(vec![
            copper_pad("J1", [0.0, 0.0], 1.0, 0.8),
            copper_pad("J2", [1.15, 0.0], 1.0, 0.8),
        ]);

        let violations = component_spacing_readiness(&board, &[], 0.25, 0.5);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "component-spacing-readiness");
        assert_eq!(violations[0].locations, vec![[0.0, 0.0], [1.15, 0.0]]);
    }

    #[test]
    fn component_spacing_readiness_accepts_distant_small_other_layer_or_selected_out_pads() {
        let distant = board_with_copper(vec![
            copper_pad("J1", [0.0, 0.0], 1.0, 0.8),
            copper_pad("J2", [2.0, 0.0], 1.0, 0.8),
        ]);
        assert!(component_spacing_readiness(&distant, &[], 0.25, 0.5).is_empty());

        let small = board_with_copper(vec![
            copper_pad("R1", [0.0, 0.0], 0.3, 0.3),
            copper_pad("R2", [0.4, 0.0], 0.3, 0.3),
        ]);
        assert!(component_spacing_readiness(&small, &[], 0.25, 0.5).is_empty());

        let other_layer = board_with_copper(vec![
            copper_pad("J1", [0.0, 0.0], 1.0, 0.8),
            copper_pad_on_layer("B.Cu", "J2", [1.15, 0.0], 1.0, 0.8),
        ]);
        assert!(component_spacing_readiness(&other_layer, &[], 0.25, 0.5).is_empty());

        let selected_out = board_with_copper(vec![
            copper_pad_on_layer("B.Cu", "J1", [0.0, 0.0], 1.0, 0.8),
            copper_pad_on_layer("B.Cu", "J2", [1.15, 0.0], 1.0, 0.8),
        ]);
        assert!(
            component_spacing_readiness(&selected_out, &["F.Cu".to_string()], 0.25, 0.5).is_empty()
        );
    }

    #[test]
    fn connector_rework_clearance_readiness_reports_tight_neighboring_pad() {
        let board = board_with_copper(vec![
            copper_pad("USB_DP", [0.0, 0.0], 1.0, 0.8),
            copper_pad("SIG", [0.82, 0.0], 0.25, 0.25),
            copper_pad("USB_DM", [0.0, 1.0], 1.0, 0.8),
        ]);

        let violations = connector_rework_clearance_readiness(&board, &[], 0.25, 0.5);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "connector-rework-clearance-readiness");
        assert_eq!(violations[0].locations, vec![[0.0, 0.0], [0.82, 0.0]]);
    }

    #[test]
    fn connector_rework_clearance_readiness_culls_sparse_pad_fields() {
        let mut copper = (0..2_000)
            .map(|index| {
                copper_pad(
                    &format!("SIG{index}"),
                    [(index % 50) as f64 * 5.0, (index / 50) as f64 * 5.0],
                    0.5,
                    0.5,
                )
            })
            .collect::<Vec<_>>();
        copper.push(copper_pad("USB_DP", [-10.0, -10.0], 1.0, 0.8));
        copper.push(copper_pad("SIG_NEAR", [-9.18, -10.0], 0.25, 0.25));
        copper.push(copper_pad("USB_DM", [250.0, 250.0], 1.0, 0.8));
        let board = board_with_copper(copper);

        let start = std::time::Instant::now();
        let violations = connector_rework_clearance_readiness(&board, &[], 0.25, 0.5);

        assert_eq!(violations.len(), 1);
        assert!(
            start.elapsed() < std::time::Duration::from_secs(2),
            "connector rework clearance should cull distant sparse pads by grid bucket"
        );
    }

    #[test]
    fn component_spacing_readiness_culls_sparse_component_fields() {
        let mut copper = (0..2_000)
            .map(|index| {
                copper_pad(
                    &format!("J{index}"),
                    [(index % 50) as f64 * 5.0, (index / 50) as f64 * 5.0],
                    1.0,
                    0.8,
                )
            })
            .collect::<Vec<_>>();
        copper.push(copper_pad("NEAR_A", [-10.0, -10.0], 1.0, 0.8));
        copper.push(copper_pad("NEAR_B", [-8.85, -10.0], 1.0, 0.8));
        let board = board_with_copper(copper);

        let start = std::time::Instant::now();
        let violations = component_spacing_readiness(&board, &[], 0.25, 0.5);

        assert_eq!(violations.len(), 1);
        assert!(
            start.elapsed() < std::time::Duration::from_secs(2),
            "component spacing should cull distant sparse fields by grid bucket"
        );
    }

    #[test]
    fn fiducial_keepout_readiness_reports_same_layer_copper_intrusion() {
        let board = board_with_copper(vec![
            fiducial("F.Cu", [0.0, 0.0], 0.8),
            copper_pad("SIG", [0.75, 0.0], 0.25, 0.25),
        ]);

        let violations = fiducial_keepout_readiness(&board, &[], 0.25, 1.0e-9);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "fiducial-keepout-readiness");
        assert_eq!(violations[0].locations, vec![[0.0, 0.0], [0.75, 0.0]]);
    }

    #[test]
    fn fiducial_keepout_readiness_accepts_clear_other_layer_or_selected_out_copper() {
        let clear = board_with_copper(vec![
            fiducial("F.Cu", [0.0, 0.0], 0.8),
            copper_pad("SIG", [2.0, 0.0], 0.25, 0.25),
        ]);
        assert!(fiducial_keepout_readiness(&clear, &[], 0.25, 1.0e-9).is_empty());

        let other_layer = board_with_copper(vec![
            fiducial("F.Cu", [0.0, 0.0], 0.8),
            copper_pad_on_layer("B.Cu", "SIG", [0.75, 0.0], 0.25, 0.25),
        ]);
        assert!(fiducial_keepout_readiness(&other_layer, &[], 0.25, 1.0e-9).is_empty());

        let selected_out = board_with_copper(vec![fiducial("B.Cu", [0.0, 0.0], 0.8)]);
        assert!(
            fiducial_keepout_readiness(&selected_out, &["F.Cu".to_string()], 0.25, 1.0e-9)
                .is_empty()
        );
    }

    #[test]
    fn fiducial_keepout_readiness_culls_sparse_blocker_fields() {
        let mut copper = vec![fiducial("F.Cu", [-10.0, -10.0], 0.8)];
        copper.extend((0..2_000).map(|index| {
            copper_pad(
                &format!("SIG{index}"),
                [(index % 50) as f64 * 5.0, (index / 50) as f64 * 5.0],
                0.25,
                0.25,
            )
        }));
        copper.push(copper_pad("SIG_NEAR", [-9.35, -10.0], 0.25, 0.25));
        let board = board_with_copper(copper);

        let start = std::time::Instant::now();
        let violations = fiducial_keepout_readiness(&board, &[], 0.25, 1.0e-9);

        assert_eq!(violations.len(), 1);
        assert!(
            start.elapsed() < std::time::Duration::from_secs(2),
            "fiducial keepout should cull distant blockers by grid bucket"
        );
    }

    #[test]
    fn testpoint_copper_clearance_readiness_reports_unrelated_nearby_copper() {
        let board = board_with_copper(vec![
            copper_pad("TP_NET", [0.0, 0.0], 0.4, 0.4),
            copper_pad("OTHER", [0.4, 0.0], 0.25, 0.25),
        ]);
        let point = ipc_point("TP_NET", [0.0, 0.0], Some(0.4));

        let violations =
            testpoint_copper_clearance_readiness(&board, &[point], &[], 0.4, 0.1, 1.0e-9);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "testpoint-copper-clearance-readiness");
        assert_eq!(violations[0].locations, vec![[0.0, 0.0], [0.4, 0.0]]);
    }

    #[test]
    fn testpoint_copper_clearance_readiness_accepts_same_net_far_or_selected_out_copper() {
        let same_net = board_with_copper(vec![
            copper_pad("TP_NET", [0.0, 0.0], 0.4, 0.4),
            copper_pad("TP_NET", [0.55, 0.0], 0.25, 0.25),
        ]);
        let point = ipc_point("TP_NET", [0.0, 0.0], Some(0.4));
        assert!(
            testpoint_copper_clearance_readiness(&same_net, &[point], &[], 0.4, 0.1, 1.0e-9)
                .is_empty()
        );

        let far = board_with_copper(vec![
            copper_pad("TP_NET", [0.0, 0.0], 0.4, 0.4),
            copper_pad("OTHER", [2.0, 0.0], 0.25, 0.25),
        ]);
        let point = ipc_point("TP_NET", [0.0, 0.0], Some(0.4));
        assert!(
            testpoint_copper_clearance_readiness(&far, &[point], &[], 0.4, 0.1, 1.0e-9).is_empty()
        );

        let selected_out = board_with_copper(vec![copper_pad_on_layer(
            "B.Cu",
            "OTHER",
            [0.55, 0.0],
            0.25,
            0.25,
        )]);
        let point = ipc_point("TP_NET", [0.0, 0.0], Some(0.4));
        assert!(
            testpoint_copper_clearance_readiness(
                &selected_out,
                &[point],
                &["F.Cu".to_string()],
                0.4,
                0.1,
                1.0e-9
            )
            .is_empty()
        );
    }

    #[test]
    fn testpoint_copper_clearance_readiness_culls_sparse_copper_fields() {
        let mut copper = (0..2_000)
            .map(|index| {
                copper_pad(
                    &format!("SIG{index}"),
                    [100.0 + (index % 50) as f64 * 5.0, (index / 50) as f64 * 5.0],
                    0.25,
                    0.25,
                )
            })
            .collect::<Vec<_>>();
        copper.push(copper_pad("TP_NET", [-10.0, -10.0], 0.4, 0.4));
        copper.push(copper_pad("OTHER_NEAR", [-9.62, -10.0], 0.25, 0.25));
        let board = board_with_copper(copper);
        let point = ipc_point("TP_NET", [-10.0, -10.0], Some(0.4));

        let start = std::time::Instant::now();
        let violations =
            testpoint_copper_clearance_readiness(&board, &[point], &[], 0.4, 0.1, 1.0e-9);

        assert_eq!(violations.len(), 1);
        assert!(
            start.elapsed() < std::time::Duration::from_secs(2),
            "testpoint copper clearance should cull distant copper by grid bucket"
        );
    }

    #[test]
    fn testpoint_accessibility_readiness_reports_close_probe_spacing() {
        let board = board_with_copper(Vec::new());
        let points = vec![
            accessible_ipc_point("TP1", [0.0, 0.0], 0.5),
            accessible_ipc_point("TP2", [0.8, 0.0], 0.5),
            accessible_ipc_point("TP3", [3.0, 0.0], 0.5),
        ];

        let violations = testpoint_accessibility_readiness(&board, &points, 0.4, 0.35, 0.5);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "testpoint-accessibility-readiness");
        assert_eq!(violations[0].layers, vec!["net:TP1", "net:TP2"]);
    }

    #[test]
    fn testpoint_accessibility_readiness_culls_sparse_probe_fields() {
        let board = board_with_copper(Vec::new());
        let mut points = (0..2_000)
            .map(|index| {
                accessible_ipc_point(
                    &format!("TP{index}"),
                    [(index % 50) as f64 * 5.0, (index / 50) as f64 * 5.0],
                    0.5,
                )
            })
            .collect::<Vec<_>>();
        points.push(accessible_ipc_point("NEAR_A", [-10.0, -10.0], 0.5));
        points.push(accessible_ipc_point("NEAR_B", [-9.22, -10.0], 0.5));

        let start = std::time::Instant::now();
        let violations = testpoint_accessibility_readiness(&board, &points, 0.4, 0.35, 0.5);

        assert_eq!(violations.len(), 1);
        assert!(
            start.elapsed() < std::time::Duration::from_secs(2),
            "testpoint accessibility should cull distant probe pairs by grid bucket"
        );
    }

    #[test]
    fn testpoint_accessibility_readiness_culls_sparse_side_parity_copper() {
        let mut copper = (0..2_000)
            .map(|index| {
                copper_pad_on_layer(
                    "F.Cu",
                    "TP_NET",
                    [100.0 + (index % 50) as f64 * 5.0, (index / 50) as f64 * 5.0],
                    0.25,
                    0.25,
                )
            })
            .collect::<Vec<_>>();
        copper.push(copper_pad_on_layer(
            "B.Cu",
            "TP_NET",
            [-10.0, -10.0],
            0.35,
            0.35,
        ));
        let board = board_with_copper(copper);
        let point = accessible_ipc_point("TP_NET", [-10.0, -10.0], 0.4);

        let start = std::time::Instant::now();
        let violations = testpoint_accessibility_readiness(&board, &[point], 0.4, 0.35, 0.5);

        assert_eq!(violations.len(), 1);
        assert!(violations[0].message.as_deref().is_some_and(|message| {
            message.contains("access side is top") && message.contains("only on bottom")
        }));
        assert!(
            start.elapsed() < std::time::Duration::from_secs(2),
            "testpoint side parity should cull distant KiCad pads by grid bucket"
        );
    }

    fn board_with_copper(copper: Vec<CopperFeature>) -> BoardModel {
        board_with_copper_and_drills(copper, Vec::new())
    }

    fn board_with_copper_and_drills(
        copper: Vec<CopperFeature>,
        drills: Vec<DrillFeature>,
    ) -> BoardModel {
        BoardModel {
            source: "test".to_string(),
            copper,
            drills,
            board_outline: None,
            panel_features: None,
        }
    }

    fn copper_pad(net: &str, location: [f64; 2], width: f64, height: f64) -> CopperFeature {
        copper_pad_on_layer("F.Cu", net, location, width, height)
    }

    fn copper_pad_on_layer(
        layer: &str,
        net: &str,
        location: [f64; 2],
        width: f64,
        height: f64,
    ) -> CopperFeature {
        CopperFeature {
            layer: layer.to_string(),
            net: Some(net.to_string()),
            kind: CopperKind::Pad,
            location,
            sketch: polygons_to_sketch(
                vec![rect_polygon(location, [width, height], 0.0)],
                Some(LayerMetadata {
                    name: "pad".to_string(),
                }),
            ),
        }
    }

    fn fiducial(layer: &str, location: [f64; 2], diameter: f64) -> CopperFeature {
        CopperFeature {
            layer: layer.to_string(),
            net: None,
            kind: CopperKind::Pad,
            location,
            sketch: polygons_to_sketch(
                vec![rect_polygon(location, [diameter, diameter], 0.0)],
                Some(LayerMetadata {
                    name: "fiducial".to_string(),
                }),
            ),
        }
    }

    fn plated_drill(net: &str, location: [f64; 2], diameter: f64) -> DrillFeature {
        DrillFeature {
            location,
            diameter,
            net: Some(net.to_string()),
            plated: true,
        }
    }

    fn npth_drill(location: [f64; 2], diameter: f64) -> DrillFeature {
        DrillFeature {
            location,
            diameter,
            net: None,
            plated: false,
        }
    }

    fn ipc_point(net: &str, location: [f64; 2], diameter: Option<f64>) -> Ipc356Point {
        Ipc356Point {
            net: net.to_string(),
            reference: Some("TP1".to_string()),
            pin: Some("1".to_string()),
            location,
            diameter,
            access_side: None,
            feature_type: None,
            soldermask: None,
        }
    }

    fn accessible_ipc_point(net: &str, location: [f64; 2], diameter: f64) -> Ipc356Point {
        Ipc356Point {
            net: net.to_string(),
            reference: Some(net.to_string()),
            pin: Some("1".to_string()),
            location,
            diameter: Some(diameter),
            access_side: Some(Ipc356AccessSide::Top),
            feature_type: Some(Ipc356FeatureType::Smd),
            soldermask: Some(Ipc356Soldermask::Open),
        }
    }
}
