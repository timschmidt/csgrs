//! Mechanical board-feature readiness checks.
//!
//! These checks sit between pure drill checks and full board-context checks:
//! they need KiCad drill/copper context, but the primary design question is
//! mechanical production intent around mounting holes, panel fixtures, and
//! chassis attachment features.
//!
//! Reliability note: parsed mounting-hole and panel geometry may omit hardware
//! stackups, fixtures, plating notes, and chassis bonds. Treat suspect findings
//! as prompts for mechanical drawing and fabrication-note verification.

use csgrs::csg::CSG;
use geo::BoundingRect;

use super::distance::polygon_boundary_distance;
use super::outline::{
    axis_aligned_outline_rect, drill_keepout_inside_rect, feature_bounds_inside_rect,
};
use super::spatial::{CopperSpatialIndex, DrillSpatialIndex};
use super::spread::maximum_point_spread;
use crate::LayerMetadata;
use crate::geometry::{circle_polygon, multipolygon_to_shapes, polygons_to_sketch};
use crate::kicad::{BoardModel, CopperFeature, DrillFeature};
use crate::report::{Severity, Violation};

/// Review likely mounting holes for nearby ground or chassis bonding copper.
///
/// IPC-2221B treats mounting holes and conductive hardware as part of the
/// board-level mechanical/electrical design envelope. This readiness check uses
/// large non-plated drill geometry as a conservative hardware proxy and applies
/// the shared spatial broad phase before exact center-distance review for
/// parsed ground/chassis copper.
pub fn mounting_hole_grounding_readiness(
    board: &BoardModel,
    selected_layers: &[String],
    grounding_distance: f64,
) -> Vec<Violation> {
    let copper = selected_copper_features(board, selected_layers);
    let grounding_features = copper
        .iter()
        .filter(|feature| {
            feature
                .net
                .as_deref()
                .is_some_and(looks_ground_or_chassis_net)
        })
        .copied()
        .collect::<Vec<_>>();
    let grounding_index = CopperSpatialIndex::new(&grounding_features, grounding_distance);
    let mut violations = Vec::new();
    let mut candidate_count = 0_usize;
    let mut exact_distance_count = 0_usize;
    log::trace!(
        "mounting-hole grounding readiness: source={} holes={} grounding_features={} buckets={} grounding_distance={grounding_distance:.6}",
        board.source,
        likely_mounting_holes(board, grounding_distance).len(),
        grounding_features.len(),
        grounding_index.bucket_count()
    );

    for drill in likely_mounting_holes(board, grounding_distance) {
        let search_radius = drill.diameter / 2.0 + grounding_distance;
        let has_grounding_intent = grounding_index
            .all_layers_near_circle(drill.location, search_radius)
            .into_iter()
            .any(|feature_index| {
                candidate_count += 1;
                let feature = grounding_features[feature_index];
                exact_distance_count += 1;
                distance(drill.location, feature.location) <= search_radius
            });
        if has_grounding_intent {
            continue;
        }

        violations.push(Violation::new(
            "mounting-hole-grounding-readiness",
            Severity::Warning,
            vec![board.source.clone()],
            None,
            Vec::new(),
            vec![drill.location],
            Some(format!(
                "likely mounting hole diameter {:.6} has no parsed ground or chassis copper within {:.6}; review chassis bonding, isolation, or keepout intent",
                drill.diameter, grounding_distance
            )),
        ));
    }

    log::trace!(
        "mounting-hole grounding readiness: source={} candidate_pairs={} exact_distance_checks={} violations={}",
        board.source,
        candidate_count,
        exact_distance_count,
        violations.len()
    );
    debug_assert!(exact_distance_count <= candidate_count);

    violations
}

/// Review likely mounting-hole screw/standoff keepouts against non-ground copper.
///
/// IPC-2221B frames mounting hardware, edge constraints, and conductor clearance
/// as layout review data. This check models the hardware region as a circular
/// keepout, uses the copper spatial index as a broad phase, then reports exact
/// CSG or boundary-distance hits against non-ground copper.
pub fn mounting_hole_copper_keepout_readiness(
    board: &BoardModel,
    selected_layers: &[String],
    keepout: f64,
    min_area: f64,
) -> Vec<Violation> {
    let copper = selected_copper_features(board, selected_layers);
    let copper_index = CopperSpatialIndex::new(&copper, keepout);
    let mut violations = Vec::new();
    let mut candidate_count = 0_usize;
    let mut exact_pair_count = 0_usize;
    log::trace!(
        "mounting-hole copper keepout readiness: source={} holes={} copper={} buckets={} keepout={keepout:.6} min_area={min_area:.9}",
        board.source,
        likely_mounting_holes(board, keepout).len(),
        copper.len(),
        copper_index.bucket_count()
    );

    for drill in likely_mounting_holes(board, keepout) {
        let keepout_sketch = drill_keepout(drill, keepout);
        let query_radius = drill.diameter / 2.0 + keepout * 2.0;
        for feature_index in copper_index.all_layers_near_circle(drill.location, query_radius) {
            candidate_count += 1;
            let feature = copper[feature_index];
            if feature
                .net
                .as_deref()
                .is_some_and(looks_ground_or_chassis_net)
            {
                continue;
            }
            exact_pair_count += 1;

            // IPC-2221B treats mounting holes, conductive hardware, and board
            // edge/mechanical constraints as layout-clearance concerns. This is
            // a conservative geometric readiness check: model the screw/hole
            // region as a circular keepout and report nearby non-ground copper
            // so the release package makes the grounding/isolation intent clear.
            let overlap = keepout_sketch.intersection(&feature.sketch);
            let shapes = multipolygon_to_shapes(&overlap.to_multipolygon(), min_area);
            let fallback_hit = shapes.is_empty()
                && polygon_boundary_distance(
                    &keepout_sketch.to_multipolygon(),
                    &feature.sketch.to_multipolygon(),
                ) <= keepout;
            if shapes.is_empty() && !fallback_hit {
                continue;
            }

            violations.push(Violation::new(
                "mounting-hole-copper-keepout-readiness",
                Severity::Warning,
                vec![feature.layer.clone()],
                None,
                shapes,
                if fallback_hit {
                    vec![drill.location, feature.location]
                } else {
                    Vec::new()
                },
                Some(format!(
                    "non-ground {:?} copper is inside likely mounting-hole keepout {:.6}; review screw, standoff, washer, or chassis clearance",
                    feature.kind, keepout
                )),
            ));
        }
    }

    log::trace!(
        "mounting-hole copper keepout readiness: source={} candidate_pairs={} exact_pairs={} violations={}",
        board.source,
        candidate_count,
        exact_pair_count,
        violations.len()
    );
    debug_assert!(exact_pair_count <= candidate_count);

    violations
}

/// Review mounting-hole screw/washer keepouts against the parsed board outline.
///
/// Rectangular board outlines use an analytic containment fast path, while
/// general outlines fall back to exact CSG difference. This keeps common board
/// shapes cheap without weakening the release-review signal for enclosure,
/// clamp, washer, and edge-clearance intent.
pub fn mounting_hole_edge_clearance_readiness(
    board: &BoardModel,
    edge_clearance: f64,
    min_area: f64,
) -> Vec<Violation> {
    let Some(outline) = &board.board_outline else {
        return Vec::new();
    };
    let outline_rect = axis_aligned_outline_rect(outline);
    let mut violations = Vec::new();
    let mut skipped_rect_inside = 0_usize;
    let mut exact_difference_count = 0_usize;

    for drill in likely_mounting_holes(board, edge_clearance) {
        if outline_rect
            .as_ref()
            .is_some_and(|rect| drill_keepout_inside_rect(drill, rect, edge_clearance))
        {
            skipped_rect_inside += 1;
            continue;
        }

        let keepout_sketch = drill_keepout(drill, edge_clearance);
        exact_difference_count += 1;
        let outside_outline = keepout_sketch.difference(outline);
        let shapes = multipolygon_to_shapes(&outside_outline.to_multipolygon(), min_area);
        if shapes.is_empty() {
            continue;
        }

        violations.push(Violation::new(
            "mounting-hole-edge-clearance-readiness",
            Severity::Warning,
            vec![board.source.clone()],
            None,
            shapes,
            vec![drill.location],
            Some(format!(
                "likely mounting-hole screw or washer keepout {:.6} extends beyond the board outline; review enclosure, clamp, or edge-clearance intent",
                edge_clearance
            )),
        ));
    }

    log::trace!(
        "mounting-hole edge clearance readiness: source={} outline_fast_path={} skipped_rect_inside={} exact_difference_checks={} violations={} edge_clearance={edge_clearance:.6} min_area={min_area:.9}",
        board.source,
        outline_rect.is_some(),
        skipped_rect_inside,
        exact_difference_count,
        violations.len()
    );

    violations
}

/// Review large plated mounting-style holes for explicit bonding intent.
///
/// Plated hardware holes can be intentional chassis bonds or accidental plating
/// ambiguity. IPC-2221B treats conductive hardware and bonding as design intent,
/// so this check accepts either a ground/chassis drill net or nearby parsed
/// bonding copper after spatial candidate filtering.
pub fn mounting_hole_plating_intent_readiness(
    board: &BoardModel,
    selected_layers: &[String],
    grounding_distance: f64,
) -> Vec<Violation> {
    let copper = selected_copper_features(board, selected_layers);
    let grounding_features = copper
        .iter()
        .filter(|feature| {
            feature
                .net
                .as_deref()
                .is_some_and(looks_ground_or_chassis_net)
        })
        .copied()
        .collect::<Vec<_>>();
    let grounding_index = CopperSpatialIndex::new(&grounding_features, grounding_distance);
    let mut violations = Vec::new();
    let mut candidate_count = 0_usize;
    let mut exact_distance_count = 0_usize;
    log::trace!(
        "mounting-hole plating intent readiness: source={} plated_holes={} grounding_features={} buckets={} grounding_distance={grounding_distance:.6}",
        board.source,
        likely_plated_mounting_holes(board, grounding_distance).len(),
        grounding_features.len(),
        grounding_index.bucket_count()
    );

    for drill in likely_plated_mounting_holes(board, grounding_distance) {
        let drill_net_is_ground = drill
            .net
            .as_deref()
            .is_some_and(looks_ground_or_chassis_net);
        let search_radius = drill.diameter / 2.0 + grounding_distance;
        let has_grounding_copper = grounding_index
            .all_layers_near_circle(drill.location, search_radius)
            .into_iter()
            .any(|feature_index| {
                candidate_count += 1;
                let feature = grounding_features[feature_index];
                exact_distance_count += 1;
                distance(drill.location, feature.location) <= search_radius
            });
        if drill_net_is_ground || has_grounding_copper {
            continue;
        }

        violations.push(Violation::new(
            "mounting-hole-plating-intent-readiness",
            Severity::Warning,
            vec![board.source.clone()],
            None,
            Vec::new(),
            vec![drill.location],
            Some(format!(
                "large plated mounting-style hole diameter {:.6} has no parsed ground/chassis net or nearby bonding copper; review plated versus isolated hardware intent",
                drill.diameter
            )),
        ));
    }

    log::trace!(
        "mounting-hole plating intent readiness: source={} candidate_pairs={} exact_distance_checks={} violations={}",
        board.source,
        candidate_count,
        exact_distance_count,
        violations.len()
    );
    debug_assert!(exact_distance_count <= candidate_count);

    violations
}

/// Run the `mounting_hole_distribution_readiness` design-readiness check or report helper.
///
/// The check compares the exact maximum span of likely hardware holes against
/// the requested review spacing. The span calculation reduces hole centers to a
/// convex hull and then uses rotating calipers; Andrew (1979), "Another
/// Efficient Algorithm for Convex Hulls in Two Dimensions", and Toussaint
/// (1983), "Solving Geometric Problems with the Rotating Calipers", describe
/// the two geometric primitives used here.
pub fn mounting_hole_distribution_readiness(
    board: &BoardModel,
    minimum_spacing: f64,
) -> Vec<Violation> {
    let holes = likely_hardware_holes(board, minimum_spacing);
    if holes.is_empty() {
        return Vec::new();
    }

    if holes.len() == 1 {
        return vec![Violation::new(
            "mounting-hole-distribution-readiness",
            Severity::Warning,
            vec![board.source.clone()],
            None,
            Vec::new(),
            vec![holes[0].location],
            Some(format!(
                "only one likely mounting hole was parsed; review enclosure support, board retention, or intentional single-fastener design"
            )),
        )];
    }

    let spread = maximum_point_spread(holes.iter().map(|hole| hole.location));
    let maximum_spacing = spread.distance;
    let span_locations = spread
        .endpoints
        .map(|endpoints| endpoints.to_vec())
        .unwrap_or_default();
    log::trace!(
        "mounting-hole distribution readiness: source={} holes={} hull_points={} caliper_steps={} maximum_spacing={maximum_spacing:.6} minimum_spacing={minimum_spacing:.6}",
        board.source,
        holes.len(),
        spread.hull_points,
        spread.caliper_steps
    );

    if maximum_spacing >= minimum_spacing {
        return Vec::new();
    }

    // IPC-2221B frames board mounting and hardware features as part of the
    // mechanical design envelope. A clustered set of fastener holes can be valid
    // for small modules, but it is weak release evidence for larger enclosure or
    // fixture retention, so report it as review-only readiness.
    vec![Violation::new(
        "mounting-hole-distribution-readiness",
        Severity::Warning,
        vec![board.source.clone()],
        None,
        Vec::new(),
        span_locations,
        Some(format!(
            "likely mounting holes span only {maximum_spacing:.6}, below review spacing {minimum_spacing:.6}; review enclosure support and board retention distribution"
        )),
    )]
}

/// Review edge-to-edge spacing between likely hardware holes.
///
/// Hardware holes are filtered first, then the shared drill spatial index
/// provides broad-phase candidates before exact center and edge-spacing math.
/// This follows the broad/narrow geometric-query pattern in Ericson,
/// *Real-Time Collision Detection* (2005), while the finding itself remains a
/// mechanical review prompt for screw heads, washers, standoffs, and breakout.
pub fn mounting_hole_spacing_readiness(
    board: &BoardModel,
    minimum_edge_spacing: f64,
) -> Vec<Violation> {
    let holes = likely_hardware_holes(board, minimum_edge_spacing * 4.0);
    let indexed_holes = holes
        .iter()
        .map(|drill| (*drill).clone())
        .collect::<Vec<_>>();
    let hole_index = DrillSpatialIndex::new(&indexed_holes, minimum_edge_spacing);
    log::trace!(
        "mounting-hole spacing readiness: source={} holes={} spatial_buckets={} minimum_edge_spacing={minimum_edge_spacing:.6}",
        board.source,
        holes.len(),
        hole_index.bucket_count()
    );
    let mut violations = Vec::new();
    let mut candidate_count = 0_usize;
    let mut exact_pair_count = 0_usize;

    for left_index in 0..holes.len() {
        let left = holes[left_index];
        for right_index in
            hole_index.later_candidates_within_spacing(left_index, minimum_edge_spacing)
        {
            candidate_count += 1;
            let right = holes[right_index];
            let center_spacing = distance(left.location, right.location);
            let edge_spacing = center_spacing - (left.diameter + right.diameter) / 2.0;
            exact_pair_count += 1;
            if edge_spacing >= minimum_edge_spacing {
                continue;
            }

            violations.push(Violation::new(
                "mounting-hole-spacing-readiness",
                Severity::Warning,
                vec![board.source.clone()],
                None,
                Vec::new(),
                vec![left.location, right.location],
                Some(format!(
                    "likely mounting-hole edge spacing {edge_spacing:.6} is below review spacing {minimum_edge_spacing:.6}; review screw head, washer, standoff, and drill breakout spacing"
                )),
            ));
        }
    }

    log::trace!(
        "mounting-hole spacing readiness: source={} candidate_pairs={} exact_pairs={} violations={}",
        board.source,
        candidate_count,
        exact_pair_count,
        violations.len()
    );
    debug_assert!(exact_pair_count <= candidate_count);

    violations
}

/// Review parsed panel/rout graphics against the board outline.
///
/// IPC-2221B treats routed profiles and board mechanical definition as release
/// package data, while IPC-7351B discusses fiducial/panel setup as part of the
/// assembly datum system. This check keeps that review geometric: each parsed
/// panel feature is measured against the board outline and reported when it is
/// too far away to be credible tab-route, V-score, rail, or routed-panel
/// evidence. Trace output records feature and exact boundary-distance counts so
/// large decorative mechanical layers can be separated from real panelization
/// issues during fixture triage.
pub fn panel_feature_outline_readiness(
    board: &BoardModel,
    edge_distance: f64,
    min_area: f64,
) -> Vec<Violation> {
    let Some(panel_features) = &board.panel_features else {
        return Vec::new();
    };

    let Some(outline) = &board.board_outline else {
        return vec![Violation::new(
            "panel-feature-outline-readiness",
            Severity::Warning,
            vec![board.source.clone()],
            None,
            Vec::new(),
            Vec::new(),
            Some(
                "KiCad panel or rout graphics were parsed, but no Edge.Cuts outline is available for edge proximity review"
                    .to_string(),
            ),
        )];
    };

    let outline_geometry = outline.to_multipolygon();
    let mut violations = Vec::new();
    let mut feature_count = 0_usize;
    let mut exact_boundary_count = 0_usize;
    for polygon in panel_features.to_multipolygon().0 {
        feature_count += 1;
        let feature = polygons_to_sketch(
            vec![polygon],
            Some(LayerMetadata {
                name: "KiCad panel feature".to_string(),
            }),
        );
        exact_boundary_count += 1;
        if polygon_boundary_distance(&feature.to_multipolygon(), &outline_geometry) <= edge_distance
        {
            continue;
        }

        let shapes = multipolygon_to_shapes(&feature.to_multipolygon(), min_area);
        if shapes.is_empty() {
            continue;
        }

        violations.push(Violation::new(
            "panel-feature-outline-readiness",
            Severity::Warning,
            vec![board.source.clone()],
            None,
            shapes,
            Vec::new(),
            Some(format!(
                "parsed panel/rout feature is more than {edge_distance:.6} from the board outline; review tab-route, V-score, rail, or mechanical-layer intent"
            )),
        ));
    }

    log::trace!(
        "panel-feature outline readiness: source={} features={} exact_boundary_checks={} violations={} edge_distance={edge_distance:.6} min_area={min_area:.9}",
        board.source,
        feature_count,
        exact_boundary_count,
        violations.len()
    );
    debug_assert!(exact_boundary_count <= feature_count);

    violations
}

/// Review edge-reaching copper for explicit plated-edge or pullback intent.
///
/// IPC-2221B and IPC-6012D both make board-edge fabrication details part of the
/// release handoff: exposed edge copper may be intentional castellations,
/// plated edges, card fingers, or a pullback omission. Rectangular outlines use
/// an AABB edge classifier before exact difference geometry; general outlines
/// use exact boundary distance. The check reports review findings rather than
/// deciding that edge copper is electrically wrong.
pub fn edge_plating_intent_readiness(
    board: &BoardModel,
    selected_layers: &[String],
    edge_distance: f64,
    min_area: f64,
) -> Vec<Violation> {
    let Some(outline) = &board.board_outline else {
        return Vec::new();
    };

    let outline_geometry = outline.to_multipolygon();
    let outline_rect = axis_aligned_outline_rect(outline);
    let mut violations = Vec::new();
    let mut exact_boundary_count = 0_usize;
    let mut skipped_interior_count = 0_usize;
    for feature in selected_copper_features(board, selected_layers) {
        let feature_geometry = feature.sketch.to_multipolygon();
        let reaches_outline = if let Some(rect) = &outline_rect {
            feature_near_rect_outline(feature, rect, edge_distance)
        } else {
            exact_boundary_count += 1;
            polygon_boundary_distance(&feature_geometry, &outline_geometry) <= edge_distance
        };

        if !reaches_outline
            && outline_rect
                .as_ref()
                .is_some_and(|rect| feature_bounds_inside_rect(feature, rect))
        {
            skipped_interior_count += 1;
            continue;
        }

        let outside_outline = feature.sketch.difference(outline);
        let shapes = multipolygon_to_shapes(&outside_outline.to_multipolygon(), min_area);
        if !reaches_outline && shapes.is_empty() {
            continue;
        }

        if shapes.is_empty() && !looks_edge_plating_net(feature.net.as_deref()) {
            continue;
        }

        // IPC-2221B treats edge contacts, plated edges, and board-outline copper
        // as fabrication notes that need explicit mechanical intent. This check
        // does not decide whether edge copper is wrong; it catches geometry that
        // should be paired with an edge-plating, castellation, bevel, or copper
        // pullback instruction before release.
        violations.push(Violation::new(
            "edge-plating-intent-readiness",
            Severity::Warning,
            vec![feature.layer.clone()],
            None,
            shapes,
            if reaches_outline {
                vec![feature.location]
            } else {
                Vec::new()
            },
            Some(format!(
                "{:?} copper on net {:?} reaches or crosses the board outline; review edge-plating, castellation, bevel, or copper-pullback fabrication intent",
                feature.kind, feature.net
            )),
        ));
    }

    log::trace!(
        "edge-plating intent readiness: source={} copper={} outline_fast_path={} skipped_interior={} exact_boundary_checks={} violations={}",
        board.source,
        selected_copper_features(board, selected_layers).len(),
        outline_rect.is_some(),
        skipped_interior_count,
        exact_boundary_count,
        violations.len()
    );

    violations
}

/// Review edge-to-edge pitch between likely castellated holes.
///
/// IPC-6012D and IPC-2221B frame plated through-hole and edge features as
/// fabrication capability/intent data. This check first classifies plated holes
/// near the board outline, then applies the shared drill spatial index before
/// exact center and edge-spacing math, matching the broad/narrow collision-query
/// pattern described by Ericson, *Real-Time Collision Detection* (2005).
pub fn castellation_pitch_readiness(
    board: &BoardModel,
    minimum_edge_spacing: f64,
) -> Vec<Violation> {
    let holes = plated_edge_holes(board, minimum_edge_spacing);
    let indexed_holes = holes
        .iter()
        .map(|drill| (*drill).clone())
        .collect::<Vec<_>>();
    let hole_index = DrillSpatialIndex::new(&indexed_holes, minimum_edge_spacing);
    log::trace!(
        "castellation pitch readiness: source={} edge_holes={} spatial_buckets={} minimum_edge_spacing={minimum_edge_spacing:.6}",
        board.source,
        holes.len(),
        hole_index.bucket_count()
    );
    let mut violations = Vec::new();
    let mut candidate_count = 0_usize;
    let mut exact_pair_count = 0_usize;

    for left_index in 0..holes.len() {
        let left = holes[left_index];
        for right_index in
            hole_index.later_candidates_within_spacing(left_index, minimum_edge_spacing)
        {
            candidate_count += 1;
            let right = holes[right_index];
            let edge_spacing =
                distance(left.location, right.location) - (left.diameter + right.diameter) / 2.0;
            exact_pair_count += 1;
            if edge_spacing >= minimum_edge_spacing {
                continue;
            }

            violations.push(Violation::new(
                "castellation-pitch-readiness",
                Severity::Warning,
                vec![board.source.clone()],
                None,
                Vec::new(),
                vec![left.location, right.location],
                Some(format!(
                    "plated edge-hole spacing {edge_spacing:.6} is below castellation review spacing {minimum_edge_spacing:.6}; review half-hole pitch, breakout, and routed-edge plating capability"
                )),
            ));
        }
    }

    log::trace!(
        "castellation pitch readiness: source={} candidate_pairs={} exact_pairs={} violations={}",
        board.source,
        candidate_count,
        exact_pair_count,
        violations.len()
    );
    debug_assert!(exact_pair_count <= candidate_count);

    violations
}

fn likely_mounting_holes(board: &BoardModel, clearance_hint: f64) -> Vec<&DrillFeature> {
    let minimum_diameter = clearance_hint.max(1.0);
    board
        .drills
        .iter()
        .filter(|drill| !drill.plated && drill.diameter >= minimum_diameter)
        .collect()
}

fn likely_plated_mounting_holes(board: &BoardModel, clearance_hint: f64) -> Vec<&DrillFeature> {
    let minimum_diameter = clearance_hint.max(1.0);
    board
        .drills
        .iter()
        .filter(|drill| drill.plated && drill.diameter >= minimum_diameter)
        .collect()
}

fn likely_hardware_holes(board: &BoardModel, spacing_hint: f64) -> Vec<&DrillFeature> {
    let minimum_diameter = (spacing_hint * 0.25).clamp(1.0, 3.2);
    board
        .drills
        .iter()
        .filter(|drill| drill.diameter >= minimum_diameter)
        .collect()
}

fn plated_edge_holes(board: &BoardModel, edge_distance: f64) -> Vec<&DrillFeature> {
    let Some(outline) = &board.board_outline else {
        return Vec::new();
    };
    if let Some(rect) = axis_aligned_outline_rect(outline) {
        let holes = board
            .drills
            .iter()
            .filter(|drill| drill.plated && drill_near_rect_outline(drill, &rect, edge_distance))
            .collect::<Vec<_>>();
        log::trace!(
            "plated edge-hole classification: source={} outline=axis-aligned-rect drills={} edge_holes={} edge_distance={edge_distance:.6}",
            board.source,
            board.drills.len(),
            holes.len()
        );
        return holes;
    }

    let outline_geometry = outline.to_multipolygon();

    let holes = board
        .drills
        .iter()
        .filter(|drill| {
            drill.plated
                && polygon_boundary_distance(
                    &drill_keepout(drill, 0.0).to_multipolygon(),
                    &outline_geometry,
                ) <= edge_distance
        })
        .collect::<Vec<_>>();
    log::trace!(
        "plated edge-hole classification: source={} outline=general drills={} edge_holes={} edge_distance={edge_distance:.6}",
        board.source,
        board.drills.len(),
        holes.len()
    );
    holes
}

fn drill_near_rect_outline(
    drill: &DrillFeature,
    rect: &geo::Rect<f64>,
    edge_distance: f64,
) -> bool {
    let radius = drill.diameter / 2.0;
    let min = rect.min();
    let max = rect.max();
    let x = drill.location[0];
    let y = drill.location[1];

    let outside_dx = if x < min.x {
        min.x - x
    } else if x > max.x {
        x - max.x
    } else {
        0.0
    };
    let outside_dy = if y < min.y {
        min.y - y
    } else if y > max.y {
        y - max.y
    } else {
        0.0
    };
    let boundary_gap = if outside_dx > 0.0 || outside_dy > 0.0 {
        outside_dx.hypot(outside_dy) - radius
    } else {
        (x - min.x).min(max.x - x).min(y - min.y).min(max.y - y) - radius
    };

    // This is an analytic narrow phase for the common rectangular board
    // outline. It avoids constructing drill CSG for every plated hole while
    // preserving the same circle-to-outline distance predicate used by the
    // general fallback. The broad/narrow split follows Ericson, Real-Time
    // Collision Detection (2005), applied here to fabrication edge-hole review.
    boundary_gap <= edge_distance
}

fn feature_near_rect_outline(
    feature: &CopperFeature,
    rect: &geo::Rect<f64>,
    edge_distance: f64,
) -> bool {
    let Some(bounds) = feature.sketch.geometry.bounding_rect() else {
        return true;
    };
    let min = rect.min();
    let max = rect.max();
    let feature_min = bounds.min();
    let feature_max = bounds.max();

    let outside = feature_max.x < min.x
        || feature_min.x > max.x
        || feature_max.y < min.y
        || feature_min.y > max.y;
    if outside {
        let dx = if feature_max.x < min.x {
            min.x - feature_max.x
        } else if feature_min.x > max.x {
            feature_min.x - max.x
        } else {
            0.0
        };
        let dy = if feature_max.y < min.y {
            min.y - feature_max.y
        } else if feature_min.y > max.y {
            feature_min.y - max.y
        } else {
            0.0
        };
        return dx.hypot(dy) <= edge_distance;
    }

    let inside_gap = (feature_min.x - min.x)
        .min(max.x - feature_max.x)
        .min(feature_min.y - min.y)
        .min(max.y - feature_max.y);

    // IPC-2221B treats board-edge copper and plated-edge intent as mechanical
    // release data. For rectangular outlines this AABB-vs-edge classifier is a
    // conservative broad phase in Ericson's broad/narrow sense: it may report
    // a feature near the edge, but it avoids expensive CSG distance checks for
    // ordinary interior copper and still lets the existing difference geometry
    // catch actual outline crossings.
    inside_gap <= edge_distance
}

fn drill_keepout(drill: &DrillFeature, keepout: f64) -> crate::PcbSketch {
    polygons_to_sketch(
        vec![circle_polygon(
            drill.location,
            drill.diameter / 2.0 + keepout,
            48,
        )],
        Some(LayerMetadata {
            name: "mounting-hole keepout".to_string(),
        }),
    )
}

fn selected_copper_features<'a>(
    board: &'a BoardModel,
    selected_layers: &[String],
) -> Vec<&'a CopperFeature> {
    board
        .copper
        .iter()
        .filter(|feature| {
            selected_layers.is_empty()
                || selected_layers.iter().any(|layer| layer == &feature.layer)
        })
        .collect()
}

fn looks_ground_or_chassis_net(net: &str) -> bool {
    let normalized = net.to_ascii_lowercase();
    normalized.contains("gnd")
        || normalized.contains("ground")
        || normalized.contains("earth")
        || normalized.contains("chassis")
        || normalized.contains("shield")
        || normalized == "0v"
}

fn looks_edge_plating_net(net: Option<&str>) -> bool {
    let Some(net) = net else {
        return false;
    };
    let normalized = net.to_ascii_lowercase();
    normalized.contains("castell")
        || normalized.contains("edge_plate")
        || normalized.contains("edge-plate")
        || normalized.contains("edgeplating")
        || normalized.contains("edge_plating")
        || normalized.contains("plated_edge")
        || normalized.contains("plated-edge")
}

fn distance(start: [f64; 2], end: [f64; 2]) -> f64 {
    let dx = end[0] - start[0];
    let dy = end[1] - start[1];
    (dx * dx + dy * dy).sqrt()
}

#[cfg(test)]
mod tests {
    use crate::LayerMetadata;
    use crate::geometry::{circle_polygon, polygons_to_sketch};
    use crate::kicad::{BoardModel, CopperFeature, CopperKind, DrillFeature};

    use super::{
        castellation_pitch_readiness, edge_plating_intent_readiness,
        mounting_hole_copper_keepout_readiness, mounting_hole_distribution_readiness,
        mounting_hole_edge_clearance_readiness, mounting_hole_grounding_readiness,
        mounting_hole_plating_intent_readiness, mounting_hole_spacing_readiness,
        panel_feature_outline_readiness,
    };

    #[test]
    fn mounting_hole_grounding_readiness_reports_unreferenced_large_npth() {
        let board = board_with(
            vec![],
            vec![npth([10.0, 10.0], 3.2), npth([20.0, 10.0], 0.4)],
        );

        let violations = mounting_hole_grounding_readiness(&board, &[], 1.0);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "mounting-hole-grounding-readiness");
        assert_eq!(violations[0].locations, vec![[10.0, 10.0]]);
    }

    #[test]
    fn mounting_hole_grounding_readiness_accepts_nearby_chassis_copper() {
        let board = board_with(
            vec![copper("CHASSIS", CopperKind::Pad, [11.0, 10.0], 0.4)],
            vec![npth([10.0, 10.0], 3.2)],
        );

        assert!(mounting_hole_grounding_readiness(&board, &[], 1.0).is_empty());
    }

    #[test]
    fn mounting_hole_grounding_readiness_culls_sparse_ground_fields() {
        let copper = (0..2_000)
            .map(|index| {
                copper(
                    "GND",
                    CopperKind::Zone,
                    [100.0 + (index % 50) as f64 * 5.0, (index / 50) as f64 * 5.0],
                    0.2,
                )
            })
            .collect::<Vec<_>>();
        let board = board_with(copper, vec![npth([-10.0, -10.0], 3.2)]);

        let started = std::time::Instant::now();
        let violations = mounting_hole_grounding_readiness(&board, &[], 1.0);

        assert_eq!(violations.len(), 1);
        assert!(
            started.elapsed() < std::time::Duration::from_secs(2),
            "mounting-hole grounding should cull distant ground/chassis copper by grid bucket"
        );
    }

    #[test]
    fn mounting_hole_copper_keepout_reports_non_ground_copper_intrusion() {
        let board = board_with(
            vec![
                copper("SIG", CopperKind::Segment, [11.4, 10.0], 0.4),
                copper("GND", CopperKind::Zone, [8.8, 10.0], 0.4),
            ],
            vec![npth([10.0, 10.0], 2.0)],
        );

        let violations = mounting_hole_copper_keepout_readiness(&board, &[], 0.5, 0.0);

        assert_eq!(violations.len(), 1);
        assert_eq!(
            violations[0].check,
            "mounting-hole-copper-keepout-readiness"
        );
        assert_eq!(violations[0].layers, vec!["F.Cu"]);
    }

    #[test]
    fn mounting_hole_copper_keepout_respects_selected_layers() {
        let board = board_with(
            vec![CopperFeature {
                layer: "B.Cu".to_string(),
                net: Some("SIG".to_string()),
                kind: CopperKind::Pad,
                sketch: polygons_to_sketch(
                    vec![circle_polygon([10.8, 10.0], 0.4, 32)],
                    Some(LayerMetadata {
                        name: "B.Cu pad".to_string(),
                    }),
                ),
                location: [10.8, 10.0],
            }],
            vec![npth([10.0, 10.0], 2.0)],
        );

        assert!(
            mounting_hole_copper_keepout_readiness(&board, &["F.Cu".to_string()], 0.5, 0.0)
                .is_empty()
        );
    }

    #[test]
    fn mounting_hole_copper_keepout_culls_sparse_copper_fields() {
        let mut copper_features = (0..2_000)
            .map(|index| {
                copper(
                    &format!("SIG{index}"),
                    CopperKind::Pad,
                    [100.0 + (index % 50) as f64 * 5.0, (index / 50) as f64 * 5.0],
                    0.2,
                )
            })
            .collect::<Vec<_>>();
        copper_features.push(copper("SIG_NEAR", CopperKind::Pad, [-8.8, -10.0], 0.2));
        let board = board_with(copper_features, vec![npth([-10.0, -10.0], 2.0)]);

        let started = std::time::Instant::now();
        let violations = mounting_hole_copper_keepout_readiness(&board, &[], 0.5, 1.0e-9);

        assert_eq!(violations.len(), 1);
        assert!(
            started.elapsed() < std::time::Duration::from_secs(2),
            "mounting-hole copper keepout should cull distant copper by grid bucket"
        );
    }

    #[test]
    fn mounting_hole_edge_clearance_reports_keepout_beyond_outline() {
        let mut board = board_with(vec![], vec![npth([1.0, 5.0], 2.0)]);
        board.board_outline = Some(polygons_to_sketch(
            vec![crate::geometry::rect_polygon([5.0, 5.0], [10.0, 10.0], 0.0)],
            Some(LayerMetadata {
                name: "outline".to_string(),
            }),
        ));

        let violations = mounting_hole_edge_clearance_readiness(&board, 0.5, 1.0e-9);

        assert_eq!(violations.len(), 1);
        assert_eq!(
            violations[0].check,
            "mounting-hole-edge-clearance-readiness"
        );
    }

    #[test]
    fn mounting_hole_edge_clearance_allows_inset_or_missing_outline() {
        let mut board = board_with(vec![], vec![npth([5.0, 5.0], 2.0)]);
        assert!(mounting_hole_edge_clearance_readiness(&board, 0.5, 1.0e-9).is_empty());

        board.board_outline = Some(polygons_to_sketch(
            vec![crate::geometry::rect_polygon([5.0, 5.0], [10.0, 10.0], 0.0)],
            Some(LayerMetadata {
                name: "outline".to_string(),
            }),
        ));
        assert!(mounting_hole_edge_clearance_readiness(&board, 0.5, 1.0e-9).is_empty());
    }

    #[test]
    fn mounting_hole_edge_clearance_culls_sparse_rectangular_outline_holes() {
        let mut drills = (0..2_000)
            .map(|index| {
                npth(
                    [
                        5.0 + (index % 50) as f64 * 0.05,
                        5.0 + (index / 50) as f64 * 0.05,
                    ],
                    2.0,
                )
            })
            .collect::<Vec<_>>();
        drills.push(npth([1.0, 5.0], 2.0));
        let mut board = board_with(vec![], drills);
        board.board_outline = Some(outline());

        let started = std::time::Instant::now();
        let violations = mounting_hole_edge_clearance_readiness(&board, 0.5, 1.0e-9);

        assert_eq!(violations.len(), 1);
        assert!(
            started.elapsed() < std::time::Duration::from_secs(2),
            "mounting-hole edge clearance should skip CSG for clear rectangular-outline holes"
        );
    }

    #[test]
    fn mounting_hole_plating_intent_reports_unbonded_large_plated_hole() {
        let board = board_with(vec![], vec![pth([10.0, 10.0], 3.2, Some("MOUNT"))]);

        let violations = mounting_hole_plating_intent_readiness(&board, &[], 1.0);

        assert_eq!(violations.len(), 1);
        assert_eq!(
            violations[0].check,
            "mounting-hole-plating-intent-readiness"
        );
    }

    #[test]
    fn mounting_hole_plating_intent_accepts_ground_net_or_nearby_bonding_copper() {
        let ground_net = board_with(vec![], vec![pth([10.0, 10.0], 3.2, Some("GND"))]);
        assert!(mounting_hole_plating_intent_readiness(&ground_net, &[], 1.0).is_empty());

        let nearby_chassis = board_with(
            vec![copper("CHASSIS", CopperKind::Pad, [11.0, 10.0], 0.4)],
            vec![pth([10.0, 10.0], 3.2, Some("MOUNT"))],
        );
        assert!(mounting_hole_plating_intent_readiness(&nearby_chassis, &[], 1.0).is_empty());
    }

    #[test]
    fn mounting_hole_plating_intent_culls_sparse_ground_fields() {
        let copper = (0..2_000)
            .map(|index| {
                copper(
                    "GND",
                    CopperKind::Zone,
                    [100.0 + (index % 50) as f64 * 5.0, (index / 50) as f64 * 5.0],
                    0.2,
                )
            })
            .collect::<Vec<_>>();
        let board = board_with(copper, vec![pth([-10.0, -10.0], 3.2, Some("MOUNT"))]);

        let started = std::time::Instant::now();
        let violations = mounting_hole_plating_intent_readiness(&board, &[], 1.0);

        assert_eq!(violations.len(), 1);
        assert!(
            started.elapsed() < std::time::Duration::from_secs(2),
            "mounting-hole plating intent should cull distant ground/chassis copper by grid bucket"
        );
    }

    #[test]
    fn mounting_hole_distribution_reports_single_or_clustered_holes() {
        let single = board_with(vec![], vec![npth([10.0, 10.0], 3.2)]);
        let single_violations = mounting_hole_distribution_readiness(&single, 8.0);
        assert_eq!(single_violations.len(), 1);
        assert_eq!(
            single_violations[0].check,
            "mounting-hole-distribution-readiness"
        );

        let clustered = board_with(
            vec![],
            vec![npth([10.0, 10.0], 3.2), pth([12.0, 10.0], 3.2, Some("GND"))],
        );
        let clustered_violations = mounting_hole_distribution_readiness(&clustered, 8.0);
        assert_eq!(clustered_violations.len(), 1);
        assert_eq!(
            clustered_violations[0].check,
            "mounting-hole-distribution-readiness"
        );
        assert_eq!(
            clustered_violations[0].locations,
            vec![[10.0, 10.0], [12.0, 10.0]]
        );
    }

    #[test]
    fn mounting_hole_distribution_accepts_absent_or_well_spaced_holes() {
        let absent = board_with(vec![], vec![npth([0.0, 0.0], 0.5)]);
        assert!(mounting_hole_distribution_readiness(&absent, 8.0).is_empty());

        let spaced = board_with(
            vec![],
            vec![npth([0.0, 0.0], 3.2), pth([20.0, 0.0], 3.2, Some("GND"))],
        );
        assert!(mounting_hole_distribution_readiness(&spaced, 8.0).is_empty());
    }

    #[test]
    fn mounting_hole_distribution_culls_sparse_clustered_hole_fields() {
        let drills = (0..2_000)
            .map(|index| {
                npth(
                    [
                        10.0 + (index % 50) as f64 * 0.01,
                        10.0 + (index / 50) as f64 * 0.01,
                    ],
                    3.2,
                )
            })
            .collect::<Vec<_>>();
        let board = board_with(vec![], drills);

        let started = std::time::Instant::now();
        let violations = mounting_hole_distribution_readiness(&board, 8.0);

        assert_eq!(violations.len(), 1);
        assert!(
            started.elapsed() < std::time::Duration::from_secs(2),
            "mounting-hole distribution should compute clustered span by hull/calipers, not all pairs"
        );
    }

    #[test]
    fn mounting_hole_spacing_reports_tight_hardware_holes() {
        let board = board_with(
            vec![],
            vec![npth([0.0, 0.0], 3.0), pth([3.4, 0.0], 3.0, Some("GND"))],
        );

        let violations = mounting_hole_spacing_readiness(&board, 0.5);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "mounting-hole-spacing-readiness");
        assert_eq!(violations[0].locations, vec![[0.0, 0.0], [3.4, 0.0]]);
    }

    #[test]
    fn mounting_hole_spacing_accepts_distant_or_tiny_holes() {
        let board = board_with(
            vec![],
            vec![
                npth([0.0, 0.0], 3.0),
                pth([5.0, 0.0], 3.0, Some("GND")),
                npth([1.0, 1.0], 0.4),
            ],
        );

        assert!(mounting_hole_spacing_readiness(&board, 0.5).is_empty());
    }

    #[test]
    fn mounting_hole_spacing_culls_large_sparse_hole_fields() {
        let mut drills = (0..2_000)
            .map(|index| npth([20.0 + index as f64 * 4.0, 20.0], 3.0))
            .collect::<Vec<_>>();
        drills.push(npth([0.0, 0.0], 3.0));
        drills.push(pth([3.4, 0.0], 3.0, Some("GND")));
        let board = board_with(vec![], drills);

        let started = std::time::Instant::now();
        let violations = mounting_hole_spacing_readiness(&board, 0.5);

        assert_eq!(violations.len(), 1);
        assert!(
            started.elapsed() < std::time::Duration::from_secs(2),
            "mounting-hole spacing should index sparse hardware holes before exact spacing review"
        );
    }

    #[test]
    fn panel_feature_outline_readiness_reports_missing_outline_or_interior_feature() {
        let mut missing_outline = board_with(vec![], vec![]);
        missing_outline.panel_features = Some(polygons_to_sketch(
            vec![crate::geometry::rect_polygon([5.0, 5.0], [1.0, 1.0], 0.0)],
            Some(LayerMetadata {
                name: "panel".to_string(),
            }),
        ));
        let missing = panel_feature_outline_readiness(&missing_outline, 0.5, 1.0e-9);
        assert_eq!(missing.len(), 1);
        assert_eq!(missing[0].check, "panel-feature-outline-readiness");

        let mut interior = board_with(vec![], vec![]);
        interior.board_outline = Some(outline());
        interior.panel_features = Some(polygons_to_sketch(
            vec![crate::geometry::rect_polygon([5.0, 5.0], [1.0, 1.0], 0.0)],
            Some(LayerMetadata {
                name: "panel".to_string(),
            }),
        ));
        let interior_violations = panel_feature_outline_readiness(&interior, 0.5, 1.0e-9);
        assert_eq!(interior_violations.len(), 1);
        assert_eq!(
            interior_violations[0].check,
            "panel-feature-outline-readiness"
        );
    }

    #[test]
    fn panel_feature_outline_readiness_accepts_edge_adjacent_or_absent_features() {
        let absent = board_with(vec![], vec![]);
        assert!(panel_feature_outline_readiness(&absent, 0.5, 1.0e-9).is_empty());

        let mut near_edge = board_with(vec![], vec![]);
        near_edge.board_outline = Some(outline());
        near_edge.panel_features = Some(polygons_to_sketch(
            vec![crate::geometry::rect_polygon([0.1, 5.0], [0.2, 2.0], 0.0)],
            Some(LayerMetadata {
                name: "panel".to_string(),
            }),
        ));
        assert!(panel_feature_outline_readiness(&near_edge, 0.5, 1.0e-9).is_empty());
    }

    #[test]
    fn panel_feature_outline_readiness_handles_sparse_panel_artwork() {
        let mut board = board_with(vec![], vec![]);
        board.board_outline = Some(outline());
        board.panel_features = Some(polygons_to_sketch(
            (0..1_000)
                .map(|index| {
                    crate::geometry::rect_polygon(
                        [
                            0.10 + (index % 50) as f64 * 0.001,
                            0.10 + (index / 50) as f64 * 0.001,
                        ],
                        [0.05, 0.05],
                        0.0,
                    )
                })
                .chain([crate::geometry::rect_polygon([5.0, 5.0], [0.5, 0.5], 0.0)])
                .collect(),
            Some(LayerMetadata {
                name: "panel".to_string(),
            }),
        ));

        let started = std::time::Instant::now();
        let violations = panel_feature_outline_readiness(&board, 0.5, 1.0e-9);

        assert_eq!(violations.len(), 1);
        assert!(
            started.elapsed() < std::time::Duration::from_secs(2),
            "panel-feature outline review should keep sparse decorative panel artwork bounded"
        );
    }

    #[test]
    fn edge_plating_intent_readiness_reports_edge_intent_or_crossing_copper() {
        let mut edge_intent = board_with(
            vec![copper("EDGE_PLATING", CopperKind::Pad, [0.25, 5.0], 0.2)],
            vec![],
        );
        edge_intent.board_outline = Some(outline());
        let intent_violations = edge_plating_intent_readiness(&edge_intent, &[], 0.5, 1.0e-9);
        assert_eq!(intent_violations.len(), 1);
        assert_eq!(intent_violations[0].check, "edge-plating-intent-readiness");

        let mut crossing = board_with(
            vec![copper("SIG", CopperKind::Segment, [-0.1, 5.0], 0.4)],
            vec![],
        );
        crossing.board_outline = Some(outline());
        let crossing_violations = edge_plating_intent_readiness(&crossing, &[], 0.5, 1.0e-9);
        assert_eq!(crossing_violations.len(), 1);
        assert_eq!(
            crossing_violations[0].check,
            "edge-plating-intent-readiness"
        );
    }

    #[test]
    fn edge_plating_intent_readiness_accepts_interior_or_selected_out_copper() {
        let mut interior = board_with(
            vec![copper("SIG", CopperKind::Segment, [5.0, 5.0], 0.4)],
            vec![],
        );
        interior.board_outline = Some(outline());
        assert!(edge_plating_intent_readiness(&interior, &[], 0.5, 1.0e-9).is_empty());

        let mut back_layer = board_with(
            vec![CopperFeature {
                layer: "B.Cu".to_string(),
                net: Some("EDGE_PLATING".to_string()),
                kind: CopperKind::Pad,
                sketch: polygons_to_sketch(
                    vec![circle_polygon([0.25, 5.0], 0.2, 32)],
                    Some(LayerMetadata {
                        name: "B.Cu copper".to_string(),
                    }),
                ),
                location: [0.25, 5.0],
            }],
            vec![],
        );
        back_layer.board_outline = Some(outline());
        assert!(
            edge_plating_intent_readiness(&back_layer, &["F.Cu".to_string()], 0.5, 1.0e-9)
                .is_empty()
        );
    }

    #[test]
    fn edge_plating_intent_readiness_culls_sparse_rectangular_outline_fields() {
        let mut copper_features = (0..2_000)
            .map(|index| {
                copper(
                    &format!("SIG{index}"),
                    CopperKind::Segment,
                    [
                        2.0 + (index % 50) as f64 * 0.1,
                        2.0 + (index / 50) as f64 * 0.1,
                    ],
                    0.02,
                )
            })
            .collect::<Vec<_>>();
        copper_features.push(copper("EDGE_PLATING", CopperKind::Pad, [0.25, 5.0], 0.2));
        let mut board = board_with(copper_features, vec![]);
        board.board_outline = Some(outline());

        let started = std::time::Instant::now();
        let violations = edge_plating_intent_readiness(&board, &[], 0.5, 1.0e-9);

        assert_eq!(violations.len(), 1);
        assert!(
            started.elapsed() < std::time::Duration::from_secs(2),
            "edge-plating intent should use the rectangular outline fast path for sparse interior copper"
        );
    }

    #[test]
    fn castellation_pitch_readiness_reports_tight_edge_plated_holes() {
        let mut board = board_with(
            vec![],
            vec![
                pth([0.0, 3.0], 0.6, Some("CAST")),
                pth([0.0, 3.7], 0.6, Some("CAST")),
            ],
        );
        board.board_outline = Some(outline());

        let violations = castellation_pitch_readiness(&board, 0.5);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "castellation-pitch-readiness");
        assert_eq!(violations[0].locations, vec![[0.0, 3.0], [0.0, 3.7]]);
    }

    #[test]
    fn castellation_pitch_readiness_accepts_distant_or_interior_plated_holes() {
        let mut board = board_with(
            vec![],
            vec![
                pth([0.0, 2.0], 0.6, Some("CAST")),
                pth([0.0, 4.0], 0.6, Some("CAST")),
                pth([5.0, 5.0], 0.6, Some("VIA")),
            ],
        );
        board.board_outline = Some(outline());

        assert!(castellation_pitch_readiness(&board, 0.5).is_empty());
    }

    #[test]
    fn castellation_pitch_readiness_culls_sparse_edge_hole_fields() {
        let mut drills = (0..2_000)
            .map(|index| pth([0.0, 20.0 + index as f64 * 4.0], 0.6, Some("CAST")))
            .collect::<Vec<_>>();
        drills.push(pth([0.0, 3.0], 0.6, Some("CAST")));
        drills.push(pth([0.0, 3.7], 0.6, Some("CAST")));
        let mut board = board_with(vec![], drills);
        board.board_outline = Some(polygons_to_sketch(
            vec![crate::geometry::rect_polygon(
                [5.0, 4_000.0],
                [10.0, 8_100.0],
                0.0,
            )],
            Some(LayerMetadata {
                name: "large outline".to_string(),
            }),
        ));

        let started = std::time::Instant::now();
        let violations = castellation_pitch_readiness(&board, 0.5);

        assert_eq!(violations.len(), 1);
        assert!(
            started.elapsed() < std::time::Duration::from_secs(2),
            "castellation pitch should index sparse edge holes before exact spacing review"
        );
    }

    fn board_with(copper: Vec<CopperFeature>, drills: Vec<DrillFeature>) -> BoardModel {
        BoardModel {
            source: "board.kicad_pcb".to_string(),
            copper,
            drills,
            board_outline: None,
            panel_features: None,
        }
    }

    fn npth(location: [f64; 2], diameter: f64) -> DrillFeature {
        DrillFeature {
            location,
            diameter,
            net: None,
            plated: false,
        }
    }

    fn pth(location: [f64; 2], diameter: f64, net: Option<&str>) -> DrillFeature {
        DrillFeature {
            location,
            diameter,
            net: net.map(str::to_string),
            plated: true,
        }
    }

    fn outline() -> crate::PcbSketch {
        polygons_to_sketch(
            vec![crate::geometry::rect_polygon([5.0, 5.0], [10.0, 10.0], 0.0)],
            Some(LayerMetadata {
                name: "outline".to_string(),
            }),
        )
    }

    fn copper(net: &str, kind: CopperKind, location: [f64; 2], radius: f64) -> CopperFeature {
        CopperFeature {
            layer: "F.Cu".to_string(),
            net: Some(net.to_string()),
            kind,
            sketch: polygons_to_sketch(
                vec![circle_polygon(location, radius, 32)],
                Some(LayerMetadata {
                    name: "F.Cu copper".to_string(),
                }),
            ),
            location,
        }
    }
}
