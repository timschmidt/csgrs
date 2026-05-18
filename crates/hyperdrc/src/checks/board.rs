//! Board-level checks that need nets, drills, vias, or panel features.
//!
//! Reliability note: many board-level checks infer intent from net names,
//! component-like copper geometry, or parsed KiCad features. These areas are
//! suspect for false positives and false negatives on unusual naming schemes,
//! custom footprints, filled zones, and panel drawings; release decisions should
//! double-check the source board and fabrication notes.

use std::collections::{BTreeMap, BTreeSet, HashMap};

use csgrs::csg::CSG;
use geo::{Area, BoundingRect};
use hyperlimit::{PredicatePolicy, compare_reals_with_policy};

use super::distance::{polygon_boundary_distance, polygon_boundary_distance_with_grid};
use super::outline::{
    axis_aligned_outline_rect_with_grid, feature_bounds_inside_rect_margin_with_grid,
    feature_bounds_inside_rect_with_grid,
};
use super::spatial::{CopperSpatialIndex, DrillSpatialIndex, PointSpatialIndex};
use crate::checks::drill::drills_to_sketch;
use crate::geometry::{
    RuleGeometryProvenance, SourceGridFacts, circle_polygon, multipolygon_to_shapes,
    polygons_to_sketch,
};
use crate::ipc356::Ipc356Point;
use crate::kicad::{BoardModel, CopperFeature, CopperKind, DrillFeature};
use crate::report::{Severity, Violation};
use crate::{LayerMetadata, PcbSketch};

/// Warn when parsed KiCad copper is narrower than the configured width.
///
/// IPC-2221B treats conductor geometry as a board-level design constraint, and
/// Tang et al., "Study on Wet Chemical Etching of Flexible Printed Circuit
/// Board with 16-um Line Pitch", *Journal of Electronic Materials*, 2023,
/// motivates keeping etch-sensitive narrow copper visible during readiness
/// review. This check uses the parsed feature bounding box as a conservative
/// proxy, so suspect findings should be verified against the native CAD rule
/// deck and fabrication limits.
pub fn copper_width_readiness(
    board: &BoardModel,
    selected_layers: &[String],
    minimum_width: f64,
) -> Vec<Violation> {
    let features = selected_copper_features(board, selected_layers);
    let mut measured_features = 0usize;
    let mut violations = Vec::new();

    for feature in &features {
        let width = minimum_bounding_dimension(&feature.sketch);
        if width <= 0.0 {
            continue;
        }
        measured_features += 1;
        if width >= minimum_width {
            continue;
        }

        violations.push(Violation::new(
            "copper-width-readiness",
            Severity::Error,
            vec![feature.layer.clone()],
            None,
            Vec::new(),
            vec![feature.location],
            Some(format!(
                "parsed {:?} copper width {width:.6} is below minimum {minimum_width:.6}",
                feature.kind
            )),
        ));
    }

    log::trace!(
        "copper-width readiness: source={} selected_features={} measured_features={} selected_layers={} minimum_width={minimum_width:.6} violations={}",
        board.source,
        features.len(),
        measured_features,
        selected_layers.len(),
        violations.len()
    );

    violations
}

/// Warn when parsed KiCad copper has no assigned net.
///
/// IPC-D-356B exists to exchange bare-board electrical test intent; after KiCad
/// parsing and any IPC-D-356 annotation pass, remaining unnetted copper is
/// suspect for intentional shields, mechanical copper, parser misses, or true
/// release-data gaps. This check only reports the ambiguity so a reviewer can
/// confirm the source board and test handoff.
pub fn copper_net_intent(board: &BoardModel, selected_layers: &[String]) -> Vec<Violation> {
    let features = selected_copper_features(board, selected_layers);
    let mut violations = Vec::new();

    for feature in &features {
        if feature.net.is_some() {
            continue;
        }
        violations.push(Violation::new(
            "copper-net-intent",
            Severity::Warning,
            vec![feature.layer.clone()],
            None,
            Vec::new(),
            vec![feature.location],
            Some(format!(
                "parsed {:?} copper has no net after KiCad parsing and IPC-D-356 annotation",
                feature.kind
            )),
        ));
    }

    log::trace!(
        "copper-net intent: source={} selected_features={} selected_layers={} violations={}",
        board.source,
        features.len(),
        selected_layers.len(),
        violations.len()
    );

    violations
}

/// Run the `via_in_pad_readiness` design-readiness check or report helper.
///
/// This is a broad/narrow-phase geometry review in the sense of Ericson,
/// *Real-Time Collision Detection* (2005): the spatial index only proposes
/// nearby same-layer pads, then exact CSG overlap decides the finding. The
/// manufacturing risk being made visible here is the via-in-pad fill/tent/paste
/// handoff discussed by Jonnalagadda, "Reliability of Via-in-Pad Structures in
/// Mechanical Cycling Fatigue" (2002).
pub fn via_in_pad_readiness(
    board: &BoardModel,
    selected_layers: &[String],
    min_area: f64,
) -> Vec<Violation> {
    let features = selected_copper_features(board, selected_layers);
    let vias = features
        .iter()
        .filter(|feature| feature.kind == CopperKind::Via)
        .copied()
        .collect::<Vec<_>>();
    let pads = features
        .iter()
        .filter(|feature| feature.kind == CopperKind::Pad)
        .copied()
        .collect::<Vec<_>>();
    let pad_spatial_index = CopperSpatialIndex::new(&pads, 1.0);
    log::trace!(
        "via-in-pad readiness: source={} vias={} pads={} buckets={} min_area={min_area:.9}",
        board.source,
        vias.len(),
        pads.len(),
        pad_spatial_index.bucket_count()
    );
    let mut violations = Vec::new();
    let mut candidate_count = 0_usize;

    for via in vias {
        for pad_index in pad_spatial_index.same_layer_near_feature(via, 0.0) {
            candidate_count += 1;
            let pad = pads[pad_index];
            if via.layer != pad.layer || via.net.is_none() || via.net != pad.net {
                continue;
            }
            if !sketches_within_clearance(&via.sketch, &pad.sketch, 0.0) {
                continue;
            }

            let overlap = via.sketch.intersection(&pad.sketch);
            let shapes = multipolygon_to_shapes(&overlap.to_multipolygon(), min_area);
            if shapes.is_empty() {
                continue;
            }

            violations.push(Violation::new(
                "via-in-pad-readiness",
                Severity::Warning,
                vec![via.layer.clone()],
                None,
                shapes,
                vec![via.location, pad.location],
                Some(
                    "via copper overlaps a same-net pad; confirm via-in-pad fill, tenting, or paste treatment"
                        .to_string(),
                ),
            ));
        }
    }

    log::trace!(
        "via-in-pad readiness: source={} candidate_pairs={} violations={}",
        board.source,
        candidate_count,
        violations.len()
    );

    violations
}

/// Run the `teardrop_readiness` design-readiness check or report helper.
///
/// The rule is a release-readiness proxy for pad/via entry robustness rather
/// than a true teardrop synthesizer. IPC-2221B treats conductor-to-land joins as
/// a design reliability concern; HyperDRC keeps that review visible by finding
/// narrow same-net segment entries after a spatial broad phase filters the
/// candidate anchors.
pub fn teardrop_readiness(
    board: &BoardModel,
    selected_layers: &[String],
    min_neck_width: f64,
    min_area: f64,
) -> Vec<Violation> {
    let features = selected_copper_features(board, selected_layers);
    let segments = features
        .iter()
        .filter(|feature| feature.kind == CopperKind::Segment)
        .copied()
        .collect::<Vec<_>>();
    let anchors = features
        .iter()
        .filter(|feature| matches!(feature.kind, CopperKind::Pad | CopperKind::Via))
        .copied()
        .collect::<Vec<_>>();
    let anchor_index = CopperSpatialIndex::new(&anchors, 1.0);
    log::trace!(
        "teardrop readiness: source={} segments={} anchors={} buckets={} min_neck_width={min_neck_width:.6} min_area={min_area:.9}",
        board.source,
        segments.len(),
        anchors.len(),
        anchor_index.bucket_count()
    );
    let mut violations = Vec::new();
    let mut candidate_count = 0_usize;

    for segment in segments {
        let segment_width = minimum_bounding_dimension(&segment.sketch);
        if segment_width >= min_neck_width {
            continue;
        }

        for anchor_index in anchor_index.same_layer_near_feature(segment, 0.0) {
            candidate_count += 1;
            let anchor = anchors[anchor_index];
            if segment.layer != anchor.layer || segment.net.is_none() || segment.net != anchor.net {
                continue;
            }

            let overlap = segment.sketch.intersection(&anchor.sketch);
            let shapes = multipolygon_to_shapes(&overlap.to_multipolygon(), min_area);
            if shapes.is_empty() {
                continue;
            }

            violations.push(Violation::new(
                "teardrop-readiness",
                Severity::Warning,
                vec![segment.layer.clone()],
                None,
                shapes,
                vec![segment.location, anchor.location],
                Some(format!(
                    "same-net segment neck width {segment_width:.6} into {:?} is below {min_neck_width:.6}; consider teardrops or wider entry geometry",
                    anchor.kind
                )),
            ));
        }
    }

    log::trace!(
        "teardrop readiness: source={} candidate_pairs={} violations={}",
        board.source,
        candidate_count,
        violations.len()
    );

    violations
}

/// Run the `plane_clearance_readiness` design-readiness check or report helper.
///
/// Non-plated drill keepouts are matched to candidate copper zones with the
/// deterministic grid broad phase described by Ericson, *Real-Time Collision
/// Detection* (2005), before exact CSG overlap review. This preserves the
/// conservative antipad-readiness predicate while avoiding every drill scanning
/// every zone on sparse mechanical-heavy boards.
pub fn plane_clearance_readiness(
    board: &BoardModel,
    selected_layers: &[String],
    min_area: f64,
) -> Vec<Violation> {
    let zones = selected_copper_features(board, selected_layers)
        .into_iter()
        .filter(|feature| feature.kind == CopperKind::Zone)
        .collect::<Vec<_>>();
    let maximum_drill_radius = board
        .drills
        .iter()
        .filter(|drill| !drill.plated)
        .map(|drill| drill.diameter / 2.0)
        .fold(0.0_f64, f64::max);
    let zone_index = CopperSpatialIndex::new(&zones, maximum_drill_radius);
    let mut violations = Vec::new();
    let mut drill_count = 0usize;
    let mut candidate_pairs = 0usize;

    for drill in &board.drills {
        if drill.plated {
            continue;
        }
        drill_count += 1;

        let hole = polygons_to_sketch(
            vec![circle_polygon(drill.location, drill.diameter / 2.0, 64)],
            Some(LayerMetadata {
                name: "mechanical hole".to_string(),
            }),
        );

        for candidate_index in
            zone_index.all_layers_near_circle(drill.location, drill.diameter / 2.0)
        {
            candidate_pairs += 1;
            let zone = zones[candidate_index];
            let overlap = hole.intersection(&zone.sketch);
            let shapes = multipolygon_to_shapes(&overlap.to_multipolygon(), min_area);
            if shapes.is_empty() {
                continue;
            }

            violations.push(Violation::new(
                "plane-clearance-readiness",
                Severity::Warning,
                vec![zone.layer.clone(), "KiCad NPTH drills".to_string()],
                None,
                shapes,
                vec![drill.location, zone.location],
                Some(
                    "non-plated mechanical hole intersects copper zone; review plane antipad or pour clearance intent"
                        .to_string(),
                ),
            ));
        }
    }
    log::trace!(
        "plane clearance readiness: source={} zones={} non_plated_drills={} spatial_buckets={} candidate_pairs={} violations={}",
        board.source,
        zones.len(),
        drill_count,
        zone_index.bucket_count(),
        candidate_pairs,
        violations.len()
    );

    violations
}

/// Run the `board_edge_exposure` design-readiness check or report helper.
pub fn board_edge_exposure(
    board: &BoardModel,
    selected_layers: &[String],
    min_area: f64,
) -> Vec<Violation> {
    board_edge_exposure_with_grid(
        board,
        selected_layers,
        min_area,
        SourceGridFacts::PRIMITIVE_FLOAT_EDGE,
    )
}

/// Run board-edge exposure with retained source-grid facts.
///
/// The rectangular fast path is a certified broad-phase decision using exact
/// lifted comparisons. This follows Yap's EGC boundary from "Towards Exact
/// Geometric Computation," Computational Geometry 7.1-2 (1997): retain source
/// representation facts until a predicate chooses arithmetic.
pub fn board_edge_exposure_with_grid(
    board: &BoardModel,
    selected_layers: &[String],
    min_area: f64,
    grid: SourceGridFacts,
) -> Vec<Violation> {
    let Some(outline) = &board.board_outline else {
        return Vec::new();
    };
    let outline_rect = axis_aligned_outline_rect_with_grid(outline, grid);
    let mut violations = Vec::new();
    let mut skipped_rect_inside = 0_usize;
    let mut exact_difference_count = 0_usize;

    for feature in selected_copper_features(board, selected_layers) {
        if outline_rect
            .as_ref()
            .is_some_and(|rect| feature_bounds_inside_rect_with_grid(feature, rect, grid))
        {
            skipped_rect_inside += 1;
            continue;
        }

        exact_difference_count += 1;
        let outside_outline = feature.sketch.difference(outline);
        let shapes = multipolygon_to_shapes(&outside_outline.to_multipolygon(), min_area);
        if shapes.is_empty() {
            continue;
        }

        violations.push(Violation::new(
            "board-edge-exposure",
            Severity::Warning,
            vec![feature.layer.clone(), "KiCad Edge.Cuts".to_string()],
            None,
            shapes,
            vec![feature.location],
            Some(format!(
                "parsed {:?} copper extends outside the board outline; confirm edge plating, castellations, or copper pullback intent",
                feature.kind
            )),
        ));
    }

    log::trace!(
        "board-edge exposure: source={} selected_layers={} outline_fast_path={} skipped_rect_inside={} exact_difference_checks={} violations={}",
        board.source,
        selected_layers.len(),
        outline_rect.is_some(),
        skipped_rect_inside,
        exact_difference_count,
        violations.len()
    );

    violations
}

/// Run the `high_speed_edge_readiness` design-readiness check or report helper.
pub fn high_speed_edge_readiness(
    board: &BoardModel,
    selected_layers: &[String],
    edge_clearance: f64,
    min_area: f64,
) -> Vec<Violation> {
    high_speed_edge_readiness_with_grid(
        board,
        selected_layers,
        edge_clearance,
        min_area,
        SourceGridFacts::PRIMITIVE_FLOAT_EDGE,
    )
}

/// Run high-speed edge readiness with retained source-grid facts.
///
/// The rectangular edge-band predicate uses exact lifted comparisons with
/// source-unit provenance before allowing an interior feature to skip CSG,
/// matching Yap, "Towards Exact Geometric Computation," Computational Geometry
/// 7.1-2 (1997).
pub fn high_speed_edge_readiness_with_grid(
    board: &BoardModel,
    selected_layers: &[String],
    edge_clearance: f64,
    min_area: f64,
    grid: SourceGridFacts,
) -> Vec<Violation> {
    let Some(outline) = &board.board_outline else {
        return Vec::new();
    };
    let outline_rect = axis_aligned_outline_rect_with_grid(outline, grid);
    let allowed = outline.offset(-edge_clearance);
    let mut violations = Vec::new();
    let mut skipped_rect_inside = 0_usize;
    let mut exact_difference_count = 0_usize;

    for feature in selected_copper_features(board, selected_layers) {
        let Some(net) = &feature.net else {
            continue;
        };
        if !looks_high_speed_net(net) {
            continue;
        }
        if outline_rect.as_ref().is_some_and(|rect| {
            feature_bounds_inside_rect_margin_with_grid(feature, rect, edge_clearance, grid)
        }) {
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
            "high-speed-edge-readiness",
            Severity::Warning,
            vec![feature.layer.clone(), "KiCad Edge.Cuts".to_string()],
            None,
            shapes,
            vec![feature.location],
            Some(format!(
                "likely high-speed net {net} is within {edge_clearance:.6} of the board edge; review EMC, return-current, and connector-edge intent"
            )),
        ));
    }

    log::trace!(
        "high-speed edge readiness: source={} selected_layers={} outline_fast_path={} skipped_rect_inside={} exact_difference_checks={} edge_clearance={edge_clearance:.6} violations={}",
        board.source,
        selected_layers.len(),
        outline_rect.is_some(),
        skipped_rect_inside,
        exact_difference_count,
        violations.len()
    );

    violations
}

/// Warn when non-edge intent copper appears inside the board-edge pullback band.
pub fn edge_copper_pullback_readiness(
    board: &BoardModel,
    selected_layers: &[String],
    edge_clearance: f64,
    min_area: f64,
) -> Vec<Violation> {
    edge_copper_pullback_readiness_with_grid(
        board,
        selected_layers,
        edge_clearance,
        min_area,
        SourceGridFacts::PRIMITIVE_FLOAT_EDGE,
    )
}

/// Run edge-copper pullback readiness with retained source-grid facts.
///
/// Both the rectangular edge-band fast path and the boundary-distance fallback
/// consume exact lifted predicates with rule/source provenance. This keeps the
/// topology gate aligned with Yap's exact-geometric-computation discipline; the
/// final CSG shapes remain report geometry at HyperDRC's current compatibility
/// boundary.
pub fn edge_copper_pullback_readiness_with_grid(
    board: &BoardModel,
    selected_layers: &[String],
    edge_clearance: f64,
    min_area: f64,
    grid: SourceGridFacts,
) -> Vec<Violation> {
    let Some(outline) = &board.board_outline else {
        return Vec::new();
    };
    let outline_rect = axis_aligned_outline_rect_with_grid(outline, grid);
    let allowed = outline.offset(-edge_clearance);
    let mut violations = Vec::new();
    let mut skipped_rect_inside = 0_usize;
    let mut exact_difference_count = 0_usize;

    for feature in selected_copper_features(board, selected_layers) {
        if let Some(net) = &feature.net {
            if looks_high_speed_net(net)
                || looks_high_voltage_net(net)
                || looks_edge_intent_net(net)
            {
                continue;
            }
        }
        if outline_rect.as_ref().is_some_and(|rect| {
            feature_bounds_inside_rect_margin_with_grid(feature, rect, edge_clearance, grid)
        }) {
            skipped_rect_inside += 1;
            continue;
        }

        exact_difference_count += 1;
        let intrusion = feature.sketch.difference(&allowed);
        let shapes = multipolygon_to_shapes(&intrusion.to_multipolygon(), min_area);
        let has_edge_intrusion = !shapes.is_empty()
            || polygon_boundary_distance_with_grid(
                &feature.sketch.to_multipolygon(),
                &outline.to_multipolygon(),
                grid,
            ) <= edge_clearance;
        if !has_edge_intrusion {
            continue;
        }

        violations.push(Violation::new(
            "edge-copper-pullback-readiness",
            Severity::Warning,
            vec![feature.layer.clone(), "KiCad Edge.Cuts".to_string()],
            None,
            shapes,
            vec![feature.location],
            Some(format!(
                "non-edge-intent copper appears within {edge_clearance:.6} board-edge clearance band; review edge pullback and copper-to-edge intent"
            )),
        ));
    }

    log::trace!(
        "edge copper-pullback readiness: source={} selected_layers={} outline_fast_path={} skipped_rect_inside={} exact_difference_checks={} edge_clearance={edge_clearance:.6} violations={}",
        board.source,
        selected_layers.len(),
        outline_rect.is_some(),
        skipped_rect_inside,
        exact_difference_count,
        violations.len()
    );

    violations
}

/// Warn when high-speed or RF/antenna nets near the edge have no nearby ground
/// stitching via for intended return-path reinforcement.
///
/// Ground-stitch centers use the point-grid broad phase from Ericson,
/// *Real-Time Collision Detection* (2005), before the exact center-radius
/// predicate. The result remains a readiness proxy: board-edge return-current
/// quality still needs stackup, chassis, and enclosure review.
pub fn edge_stitching_readiness(
    board: &BoardModel,
    selected_layers: &[String],
    edge_clearance: f64,
    stitching_distance: f64,
    min_area: f64,
) -> Vec<Violation> {
    edge_stitching_readiness_with_grid(
        board,
        selected_layers,
        edge_clearance,
        stitching_distance,
        min_area,
        SourceGridFacts::PRIMITIVE_FLOAT_EDGE,
    )
}

/// Run edge-stitching readiness with retained source-grid facts.
///
/// The edge-band fast path and boundary-distance fallback use exact lifted
/// predicates before allowing any candidate to skip slower geometry. This is
/// the same "geometric object carries representation facts to the predicate"
/// discipline Yap argues for in "Towards Exact Geometric Computation,"
/// Computational Geometry 7.1-2 (1997), applied at HyperDRC's current
/// compatibility boundary where parsed board geometry is still stored as
/// `f64`.
pub fn edge_stitching_readiness_with_grid(
    board: &BoardModel,
    selected_layers: &[String],
    edge_clearance: f64,
    stitching_distance: f64,
    min_area: f64,
    grid: SourceGridFacts,
) -> Vec<Violation> {
    let Some(outline) = &board.board_outline else {
        return Vec::new();
    };
    let outline_rect = axis_aligned_outline_rect_with_grid(outline, grid);
    let allowed = outline.offset(-edge_clearance);
    let features = selected_copper_features(board, selected_layers);
    let ground_vias = features
        .iter()
        .copied()
        .filter(|feature| feature.kind == CopperKind::Via)
        .filter(|feature| feature.net.as_deref().is_some_and(looks_ground_net))
        .collect::<Vec<_>>();
    let ground_points = ground_vias
        .iter()
        .map(|feature| feature.location)
        .collect::<Vec<_>>();
    let ground_index = PointSpatialIndex::new(ground_points, stitching_distance);

    let mut violations = Vec::new();
    let mut candidate_features = 0usize;
    let mut skipped_rect_inside = 0usize;
    let mut stitch_hits = 0usize;
    for feature in &features {
        let feature = *feature;
        let Some(net) = &feature.net else {
            continue;
        };
        if !looks_high_speed_net(net) && !looks_rf_or_antenna_net(net) {
            continue;
        }
        candidate_features += 1;
        if outline_rect.as_ref().is_some_and(|rect| {
            feature_bounds_inside_rect_margin_with_grid(feature, rect, edge_clearance, grid)
        }) {
            skipped_rect_inside += 1;
            continue;
        }

        let intrusion = feature.sketch.difference(&allowed);
        let shapes = multipolygon_to_shapes(&intrusion.to_multipolygon(), min_area);
        let has_edge_intrusion = !shapes.is_empty()
            || polygon_boundary_distance_with_grid(
                &feature.sketch.to_multipolygon(),
                &outline.to_multipolygon(),
                grid,
            ) <= edge_clearance;
        if !has_edge_intrusion {
            continue;
        }

        let nearby_stitches = point_candidates_within_radius_with_grid(
            &ground_index,
            feature.location,
            stitching_distance,
            grid,
            "edge-stitching-readiness",
        );
        stitch_hits += nearby_stitches.len();
        let has_stitch = !nearby_stitches.is_empty();
        if has_stitch {
            continue;
        }

        violations.push(Violation::new(
            "edge-stitching-readiness",
            Severity::Warning,
            vec![feature.layer.clone(), "KiCad Edge.Cuts".to_string()],
            None,
            shapes,
            vec![feature.location],
            Some(format!(
                "likely high-speed or RF net {net} is near board edge without nearby ground stitch vias within {stitching_distance:.6}"
            )),
        ));
    }

    log::trace!(
        "edge stitching readiness: source={} candidate_features={} ground_vias={} ground_buckets={} stitch_hits={} outline_fast_path={} skipped_rect_inside={} edge_clearance={edge_clearance:.6} stitching_distance={stitching_distance:.6} violations={}",
        board.source,
        candidate_features,
        ground_vias.len(),
        ground_index.bucket_count(),
        stitch_hits,
        outline_rect.is_some(),
        skipped_rect_inside,
        violations.len()
    );

    violations
}

/// Run the `acid-trap` KiCad trace-junction readiness helper.
///
/// Layer-level acid-trap checks find acute polygon vertices after all copper is
/// flattened. This board-level helper keeps KiCad segment identity long enough
/// to identify same-net traces that join at an acute angle. The check is a DFM
/// review heuristic, not a wet-etch process simulator: Tang et al., "Study on
/// Wet Chemical Etching of Flexible Printed Circuit Board with 16-um Line
/// Pitch," *Journal of Electronic Materials* 52 (2023), pp. 4030-4036,
/// <https://doi.org/10.1007/s11664-023-10368-z>, shows that copper wet etch
/// profiles depend on process transport conditions. HyperDRC therefore reports
/// suspect junction geometry for review instead of claiming a specific
/// corrosion or over-etch failure. Segment-pair candidate generation uses the
/// deterministic grid broad phase described by Ericson, *Real-Time Collision
/// Detection* (2005), before exact CSG overlap and junction-angle review.
pub fn trace_junction_acid_trap_readiness(
    board: &BoardModel,
    selected_layers: &[String],
    max_angle_degrees: f64,
    min_area: f64,
) -> Vec<Violation> {
    let segments = selected_copper_features(board, selected_layers)
        .into_iter()
        .filter(|feature| feature.kind == CopperKind::Segment)
        .filter(|feature| feature.net.is_some())
        .collect::<Vec<_>>();
    let segment_bounds = segments
        .iter()
        .map(|segment| segment.sketch.geometry().bounding_rect())
        .collect::<Vec<_>>();
    let segment_index = CopperSpatialIndex::new(&segments, 0.0);
    let mut violations = Vec::new();
    let mut candidate_pairs = 0usize;
    let mut exact_pairs = 0usize;

    for left_index in 0..segments.len() {
        let left = segments[left_index];
        let Some(left_bounds) = &segment_bounds[left_index] else {
            continue;
        };
        for right_index in segment_index.same_layer_near_feature(left, 0.0) {
            if right_index <= left_index {
                continue;
            }
            let right = segments[right_index];
            if left.layer != right.layer || left.net != right.net {
                continue;
            }
            candidate_pairs += 1;
            let Some(right_bounds) = &segment_bounds[right_index] else {
                continue;
            };
            if !rects_overlap(left_bounds, right_bounds) {
                continue;
            }

            exact_pairs += 1;
            let overlap = left.sketch.intersection(&right.sketch);
            let overlap_polygons = overlap.to_multipolygon();
            let shapes = multipolygon_to_shapes(&overlap_polygons, min_area);
            if shapes.is_empty() {
                continue;
            }
            let Some(junction) = multipolygon_center(&overlap_polygons) else {
                continue;
            };
            let angle = point_angle_degrees(left.location, junction, right.location);
            if angle <= 0.0 || angle >= max_angle_degrees {
                continue;
            }

            let net = left.net.as_deref().unwrap_or("unknown");
            violations.push(Violation::new(
                "acid-trap-trace-junction",
                Severity::Warning,
                vec![left.layer.clone()],
                None,
                shapes,
                vec![junction, left.location, right.location],
                Some(format!(
                    "same-net trace junction on {net} forms an acute {angle:.3} degree angle below {max_angle_degrees:.3}; review acid-trap and etch-cleanout risk"
                )),
            ));
        }
    }
    log::trace!(
        "trace-junction acid-trap readiness: source={} segments={} spatial_buckets={} candidate_pairs={} exact_pairs={} selected_layers={} violations={}",
        board.source,
        segments.len(),
        segment_index.bucket_count(),
        candidate_pairs,
        exact_pairs,
        selected_layers.len(),
        violations.len()
    );

    violations
}

/// Run the `controlled_impedance_readiness` design-readiness check or report helper.
/// Warn when likely high-speed nets change layers without a parsed via.
///
/// This is not an impedance solver. It is a release-readiness guard based on
/// IPC-2221B's requirement to treat controlled routing and conductor geometry
/// as explicit board-design constraints: when a high-speed net appears on
/// multiple selected copper layers without a same-net via, the parser view is
/// internally inconsistent and should be reviewed before relying on downstream
/// field or stackup checks.
pub fn controlled_impedance_readiness(
    board: &BoardModel,
    selected_layers: &[String],
) -> Vec<Violation> {
    let mut nets: BTreeMap<String, NetLayerUse> = BTreeMap::new();
    let mut high_speed_features = 0usize;

    for feature in selected_copper_features(board, selected_layers) {
        let Some(net) = &feature.net else {
            continue;
        };
        if !looks_high_speed_net(net) {
            continue;
        }
        high_speed_features += 1;

        let entry = nets.entry(net.clone()).or_default();
        entry.layers.insert(feature.layer.clone());
        entry.locations.push(feature.location);
        if feature.kind == CopperKind::Via {
            entry.has_via = true;
        }
    }

    let net_count = nets.len();
    let violations = nets
        .into_iter()
        .filter(|(_, usage)| usage.layers.len() > 1 && !usage.has_via)
        .map(|(net, usage)| {
            let layers = usage.layers.into_iter().collect::<Vec<_>>();
            Violation::new(
                "controlled-impedance-readiness",
                Severity::Warning,
                layers.clone(),
                None,
                Vec::new(),
                usage.locations,
                Some(format!(
                    "likely high-speed net {net} appears on {} copper layers without a parsed same-net via; confirm layer-change and return-path intent",
                    layers.len()
                )),
            )
        })
        .collect::<Vec<_>>();
    log::trace!(
        "controlled-impedance readiness: source={} high_speed_features={} high_speed_nets={} selected_layers={} violations={}",
        board.source,
        high_speed_features,
        net_count,
        selected_layers.len(),
        violations.len()
    );

    violations
}

/// Warn when inferred differential-pair sides are missing or split by layer.
///
/// Pair membership is inferred from common suffixes, so the result is a
/// readiness prompt rather than a formal constraint proof. Kirschning and
/// Jansen, "Accurate Wide-Range Design Equations for the Frequency-Dependent
/// Characteristic of Parallel Coupled Microstrip Lines", IEEE Transactions on
/// Microwave Theory and Techniques, 1984, motivates treating coupled pair
/// structure as explicit design intent; this check makes missing or
/// layer-divergent inferred sides visible before release.
pub fn differential_pair_readiness(
    board: &BoardModel,
    selected_layers: &[String],
) -> Vec<Violation> {
    let mut pairs: BTreeMap<String, DifferentialPairUse> = BTreeMap::new();
    let mut inferred_features = 0usize;

    for feature in selected_copper_features(board, selected_layers) {
        let Some(net) = &feature.net else {
            continue;
        };
        let Some((pair, side)) = differential_pair_key(net) else {
            continue;
        };
        inferred_features += 1;

        let entry = pairs.entry(pair).or_default();
        match side {
            DifferentialSide::Positive => {
                entry.positive_layers.insert(feature.layer.clone());
                entry.positive_locations.push(feature.location);
            }
            DifferentialSide::Negative => {
                entry.negative_layers.insert(feature.layer.clone());
                entry.negative_locations.push(feature.location);
            }
        }
    }

    let pair_count = pairs.len();
    let mut violations = Vec::new();
    for (pair, usage) in pairs {
        let has_positive = !usage.positive_layers.is_empty();
        let has_negative = !usage.negative_layers.is_empty();
        if !has_positive || !has_negative {
            let mut layers = usage.positive_layers.clone();
            layers.extend(usage.negative_layers.clone());
            let mut locations = usage.positive_locations.clone();
            locations.extend(usage.negative_locations.clone());
            let missing = if has_positive { "negative" } else { "positive" };
            violations.push(Violation::new(
                "differential-pair-readiness",
                Severity::Warning,
                layers.into_iter().collect(),
                None,
                Vec::new(),
                locations,
                Some(format!(
                    "likely differential pair {pair} is missing its {missing} side on selected copper layers"
                )),
            ));
            continue;
        }

        if usage.positive_layers != usage.negative_layers {
            let mut layers = usage.positive_layers.clone();
            layers.extend(usage.negative_layers.clone());
            let mut locations = usage.positive_locations;
            locations.extend(usage.negative_locations);
            violations.push(Violation::new(
                "differential-pair-readiness",
                Severity::Warning,
                layers.into_iter().collect(),
                None,
                Vec::new(),
                locations,
                Some(format!(
                    "likely differential pair {pair} has positive and negative sides on different selected copper layers"
                )),
            ));
        }
    }

    log::trace!(
        "differential-pair readiness: source={} inferred_features={} inferred_pairs={} selected_layers={} violations={}",
        board.source,
        inferred_features,
        pair_count,
        selected_layers.len(),
        violations.len()
    );

    violations
}

/// Warn when inferred differential-pair sides are farther apart than expected.
///
/// This is a readiness heuristic over parsed copper, not a coupled-line field
/// solver. Kirschning and Jansen, "Accurate Wide-Range Design Equations for the
/// Frequency-Dependent Characteristic of Parallel Coupled Microstrip Lines",
/// IEEE Transactions on Microwave Theory and Techniques, 1984, motivates
/// treating pair spacing as explicit routing intent. Candidate positive/negative
/// side matches use the shared copper spatial broad phase from Ericson,
/// *Real-Time Collision Detection* (2005), before exact boundary-distance
/// checks. If no close candidate exists, the check falls back to exact nearest
/// distance so true "too far apart" findings still include a measured gap.
pub fn differential_pair_spacing_readiness(
    board: &BoardModel,
    selected_layers: &[String],
    maximum_pair_gap: f64,
) -> Vec<Violation> {
    let mut pairs: BTreeMap<String, DifferentialPairFeatureUse<'_>> = BTreeMap::new();
    let mut inferred_features = 0usize;

    for feature in selected_copper_features(board, selected_layers) {
        let Some(net) = &feature.net else {
            continue;
        };
        let Some((pair, side)) = differential_pair_key(net) else {
            continue;
        };
        inferred_features += 1;

        let entry = pairs.entry(pair).or_default();
        match side {
            DifferentialSide::Positive => entry.positive.push(feature),
            DifferentialSide::Negative => entry.negative.push(feature),
        }
    }

    let pair_count = pairs.len();
    let mut candidate_pairs = 0usize;
    let mut exact_pairs = 0usize;
    let mut violations = Vec::new();
    for (pair, usage) in pairs {
        let mut layers = BTreeSet::new();
        layers.extend(usage.positive.iter().map(|feature| feature.layer.clone()));
        layers.extend(usage.negative.iter().map(|feature| feature.layer.clone()));

        for layer in layers {
            let positives = usage
                .positive
                .iter()
                .copied()
                .filter(|feature| feature.layer == layer)
                .collect::<Vec<_>>();
            let negatives = usage
                .negative
                .iter()
                .copied()
                .filter(|feature| feature.layer == layer)
                .collect::<Vec<_>>();
            if positives.is_empty() || negatives.is_empty() {
                continue;
            }

            let negative_index = CopperSpatialIndex::new(&negatives, maximum_pair_gap);
            let has_close_side = positives.iter().any(|positive| {
                negative_index
                    .same_layer_near_feature(positive, maximum_pair_gap)
                    .into_iter()
                    .any(|negative_index| {
                        candidate_pairs += 1;
                        exact_pairs += 1;
                        polygon_boundary_distance(
                            &positive.sketch.to_multipolygon(),
                            &negatives[negative_index].sketch.to_multipolygon(),
                        ) <= maximum_pair_gap
                    })
            });
            if has_close_side {
                continue;
            }

            let nearest = positives
                .iter()
                .flat_map(|positive| negatives.iter().map(move |negative| (*positive, *negative)))
                .map(|(positive, negative)| {
                    exact_pairs += 1;
                    (
                        polygon_boundary_distance(
                            &positive.sketch.to_multipolygon(),
                            &negative.sketch.to_multipolygon(),
                        ),
                        positive.location,
                        negative.location,
                    )
                })
                .min_by(|left, right| {
                    left.0
                        .partial_cmp(&right.0)
                        .unwrap_or(std::cmp::Ordering::Equal)
                });
            let Some((gap, positive_location, negative_location)) = nearest else {
                continue;
            };

            violations.push(Violation::new(
                "differential-pair-spacing-readiness",
                Severity::Warning,
                vec![layer],
                None,
                Vec::new(),
                vec![positive_location, negative_location],
                Some(format!(
                    "likely differential pair {pair} has nearest parsed pair-side spacing {gap:.6} above review threshold {maximum_pair_gap:.6}"
                )),
            ));
        }
    }

    log::trace!(
        "differential-pair spacing readiness: source={} inferred_features={} inferred_pairs={} candidate_pairs={} exact_pairs={} selected_layers={} threshold={maximum_pair_gap:.6} violations={}",
        board.source,
        inferred_features,
        pair_count,
        candidate_pairs,
        exact_pairs,
        selected_layers.len(),
        violations.len()
    );

    violations
}

/// Warn when one side of a parsed differential pair has asymmetric via count or
/// layer coverage relative to the opposite side.
pub fn differential_pair_via_symmetry_readiness(
    board: &BoardModel,
    selected_layers: &[String],
) -> Vec<Violation> {
    let mut pairs: BTreeMap<String, DifferentialPairViaUse> = BTreeMap::new();

    for feature in selected_copper_features(board, selected_layers) {
        let Some(net) = &feature.net else {
            continue;
        };
        let Some((pair, side)) = differential_pair_key(net) else {
            continue;
        };
        if feature.kind != CopperKind::Via {
            continue;
        }

        let entry = pairs.entry(pair).or_default();
        match side {
            DifferentialSide::Positive => {
                entry.positive_via_count += 1;
                entry.positive_via_layers.insert(feature.layer.clone());
                entry.positive_via_locations.push(feature.location);
            }
            DifferentialSide::Negative => {
                entry.negative_via_count += 1;
                entry.negative_via_layers.insert(feature.layer.clone());
                entry.negative_via_locations.push(feature.location);
            }
        }
    }

    let mut violations = Vec::new();
    for (pair, usage) in pairs {
        if usage.positive_via_count == 0 && usage.negative_via_count == 0 {
            continue;
        }
        if usage.positive_via_count == usage.negative_via_count
            && usage.positive_via_layers == usage.negative_via_layers
        {
            continue;
        }

        let mut layers = usage.positive_via_layers.clone();
        layers.extend(usage.negative_via_layers.clone());
        let mut locations = usage.positive_via_locations;
        locations.extend(usage.negative_via_locations);

        let mismatch = if usage.positive_via_count != usage.negative_via_count {
            format!(
                "uneven via count {}:{}",
                usage.positive_via_count, usage.negative_via_count
            )
        } else {
            "mismatched via layers".to_string()
        };

        violations.push(Violation::new(
            "differential-pair-via-symmetry-readiness",
            Severity::Warning,
            layers.into_iter().collect(),
            None,
            Vec::new(),
            locations,
            Some(format!(
                "differential pair {pair} has asymmetric via symmetry ({mismatch})"
            )),
        ));
    }

    violations
}

/// Warn when likely differential-pair copper lacks nearby same-layer ground.
///
/// This is a guard/return-path readiness check, not a field solver. IPC-2221B
/// treats conductor spacing and return-path planning as board-design concerns;
/// here we simply require some parsed ground copper near each differential side
/// so missing guard/coplanar/reference intent is visible before release.
/// Ground candidates use the deterministic grid broad phase described by
/// Ericson, *Real-Time Collision Detection* (2005), before exact geometry
/// distance/overlap review.
pub fn differential_pair_return_readiness(
    board: &BoardModel,
    selected_layers: &[String],
    guard_distance: f64,
) -> Vec<Violation> {
    let features = selected_copper_features(board, selected_layers);
    let ground_features = features
        .iter()
        .copied()
        .filter(|feature| feature.net.as_deref().is_some_and(looks_ground_net))
        .collect::<Vec<_>>();
    let ground_index = CopperSpatialIndex::new(&ground_features, guard_distance);
    let mut violations = Vec::new();
    let mut candidate_pairs = 0usize;

    for feature in &features {
        let feature = *feature;
        let Some(net) = &feature.net else {
            continue;
        };
        let Some((pair, side)) = differential_pair_key(net) else {
            continue;
        };

        let has_guard = ground_index
            .same_layer_near_feature(feature, guard_distance)
            .into_iter()
            .any(|ground_index| {
                candidate_pairs += 1;
                copper_features_touch(feature, ground_features[ground_index], guard_distance)
            });
        if has_guard {
            continue;
        }

        let side_label = match side {
            DifferentialSide::Positive => "positive",
            DifferentialSide::Negative => "negative",
        };
        violations.push(Violation::new(
            "differential-pair-return-readiness",
            Severity::Warning,
            vec![feature.layer.clone()],
            None,
            Vec::new(),
            vec![feature.location],
            Some(format!(
                "likely differential pair {pair} {side_label} side on net {net} has no parsed same-layer ground copper within {guard_distance:.6}; review guard, reference, and return-path intent"
            )),
        ));
    }
    log::trace!(
        "differential-pair return readiness: source={} features={} ground_features={} ground_buckets={} candidate_pairs={} guard_distance={guard_distance:.6} violations={}",
        board.source,
        features.len(),
        ground_features.len(),
        ground_index.bucket_count(),
        candidate_pairs,
        violations.len()
    );

    violations
}

/// Warn when likely high-speed nets have no parsed ground-zone context.
///
/// IPC-2221B treats return-path and layer-stack planning as board-design
/// constraints. This coarse check only verifies that at least one parsed ground
/// zone exists on the selected copper set before higher-fidelity void and
/// split-plane checks try to reason about coverage.
pub fn reference_plane_readiness(board: &BoardModel, selected_layers: &[String]) -> Vec<Violation> {
    let features = selected_copper_features(board, selected_layers);
    let has_ground_zone = features.iter().any(|feature| {
        feature.kind == CopperKind::Zone && feature.net.as_deref().is_some_and(looks_ground_net)
    });
    if has_ground_zone {
        log::trace!(
            "reference-plane readiness: source={} features={} selected_layers={} has_ground_zone=true violations=0",
            board.source,
            features.len(),
            selected_layers.len()
        );
        return Vec::new();
    }

    let mut nets: BTreeMap<String, NetLayerUse> = BTreeMap::new();
    for feature in &features {
        let feature = *feature;
        let Some(net) = &feature.net else {
            continue;
        };
        if !looks_high_speed_net(net) {
            continue;
        }

        let entry = nets.entry(net.clone()).or_default();
        entry.layers.insert(feature.layer.clone());
        entry.locations.push(feature.location);
    }

    let high_speed_nets = nets.len();
    let violations = nets
        .into_iter()
        .map(|(net, usage)| {
            Violation::new(
                "reference-plane-readiness",
                Severity::Warning,
                usage.layers.into_iter().collect(),
                None,
                Vec::new(),
                usage.locations,
                Some(format!(
                    "likely high-speed net {net} has no parsed ground zone on selected copper layers; review reference-plane and return-path continuity"
                )),
            )
        })
        .collect::<Vec<_>>();
    log::trace!(
        "reference-plane readiness: source={} features={} high_speed_nets={} selected_layers={} has_ground_zone=false violations={}",
        board.source,
        features.len(),
        high_speed_nets,
        selected_layers.len(),
        violations.len()
    );

    violations
}

/// Run the `reference_plane_void_readiness` design-readiness check or report helper.
///
/// Ground-zone candidates use the shared copper spatial index before exact
/// feature-minus-reference-plane CSG subtraction. This keeps reference-plane
/// review on the same deterministic broad-phase/narrow-phase footing described
/// by Ericson, *Real-Time Collision Detection* (2005), while preserving exact
/// geometry for the final void decision.
pub fn reference_plane_void_readiness(
    board: &BoardModel,
    selected_layers: &[String],
    min_area: f64,
) -> Vec<Violation> {
    let features = selected_copper_features(board, selected_layers);
    let ground_features = features
        .iter()
        .copied()
        .filter(|feature| {
            feature.kind == CopperKind::Zone && feature.net.as_deref().is_some_and(looks_ground_net)
        })
        .collect::<Vec<_>>();
    if ground_features.is_empty() {
        return Vec::new();
    }
    let ground_index = CopperSpatialIndex::new(&ground_features, 0.0);

    let mut violations = Vec::new();
    let mut candidate_features = 0usize;
    let mut candidate_ground_zones = 0usize;
    for feature in features {
        let Some(net) = &feature.net else {
            continue;
        };
        if !looks_high_speed_net(net) || feature.kind == CopperKind::Via {
            continue;
        }
        candidate_features += 1;

        let candidates = ground_index.all_layers_near_feature(feature, 0.0);
        candidate_ground_zones += candidates.len();
        let shapes = if candidates.is_empty() {
            multipolygon_to_shapes(&feature.sketch.to_multipolygon(), min_area)
        } else {
            let ground_polygons = candidates
                .into_iter()
                .flat_map(|ground_index| ground_features[ground_index].sketch.to_multipolygon().0)
                .collect::<Vec<_>>();
            let ground = polygons_to_sketch(
                ground_polygons,
                Some(LayerMetadata {
                    name: "KiCad ground zones".to_string(),
                }),
            );
            let uncovered = feature.sketch.difference(&ground);
            multipolygon_to_shapes(&uncovered.to_multipolygon(), min_area)
        };
        if shapes.is_empty() {
            continue;
        }

        violations.push(Violation::new(
            "reference-plane-void-readiness",
            Severity::Warning,
            vec![feature.layer.clone(), "KiCad ground zones".to_string()],
            None,
            shapes,
            vec![feature.location],
            Some(format!(
                "likely high-speed net {net} has copper without overlapping parsed ground-zone coverage; review split-plane and void-crossing return path"
            )),
        ));
    }

    log::trace!(
        "reference-plane void readiness: source={} high_speed_features={} ground_zones={} ground_buckets={} candidate_ground_zones={} violations={}",
        board.source,
        candidate_features,
        ground_features.len(),
        ground_index.bucket_count(),
        candidate_ground_zones,
        violations.len()
    );

    violations
}

/// Run the `orphaned_zone_readiness` design-readiness check or report helper.
///
/// Parsed pad, via, and segment anchors are indexed once, then each zone only
/// reviews same-layer nearby anchor candidates before exact CSG or boundary
/// distance confirmation. This follows Ericson, *Real-Time Collision
/// Detection* (2005): the grid is a conservative broad phase, while zone
/// connectivity remains an exact geometry decision.
pub fn orphaned_zone_readiness(
    board: &BoardModel,
    selected_layers: &[String],
    anchor_tolerance: f64,
) -> Vec<Violation> {
    let features = selected_copper_features(board, selected_layers);
    let anchors = features
        .iter()
        .copied()
        .filter(|feature| feature.kind != CopperKind::Zone)
        .collect::<Vec<_>>();
    let anchor_index = CopperSpatialIndex::new(&anchors, anchor_tolerance);
    let mut violations = Vec::new();
    let mut candidate_anchors = 0usize;

    for zone in features
        .iter()
        .filter(|feature| feature.kind == CopperKind::Zone)
    {
        let Some(net) = &zone.net else {
            continue;
        };
        let candidates = anchor_index.same_layer_near_feature(zone, anchor_tolerance);
        candidate_anchors += candidates.len();
        let has_anchor = candidates.into_iter().any(|anchor_index| {
            let anchor = anchors[anchor_index];
            anchor.net.as_deref() == Some(net.as_str())
                && (anchor
                    .sketch
                    .intersection(&zone.sketch)
                    .to_multipolygon()
                    .0
                    .iter()
                    .any(|polygon| polygon.unsigned_area() > 0.0)
                    || polygon_boundary_distance(
                        &anchor.sketch.to_multipolygon(),
                        &zone.sketch.to_multipolygon(),
                    ) <= anchor_tolerance)
        });
        if has_anchor {
            continue;
        }

        violations.push(Violation::new(
            "orphaned-zone-readiness",
            Severity::Warning,
            vec![zone.layer.clone()],
            None,
            Vec::new(),
            vec![zone.location],
            Some(format!(
                "copper zone on net {net} has no parsed same-net pad, via, or segment within {anchor_tolerance:.6}; review zone refill and connectivity"
            )),
        ));
    }

    log::trace!(
        "orphaned-zone readiness: source={} zones={} anchors={} anchor_buckets={} candidate_anchors={} violations={}",
        board.source,
        features
            .iter()
            .filter(|feature| feature.kind == CopperKind::Zone)
            .count(),
        anchors.len(),
        anchor_index.bucket_count(),
        candidate_anchors,
        violations.len()
    );

    violations
}

/// Run the `same_net_island_readiness` design-readiness check or report helper.
///
/// Same-net connectivity is still decided by exact overlap/distance predicates,
/// but candidate edges are selected with the deterministic grid broad phase from
/// Ericson, *Real-Time Collision Detection* (2005). That keeps sparse same-net
/// copper fields from requiring an all-pairs connectivity walk.
pub fn same_net_island_readiness(
    board: &BoardModel,
    selected_layers: &[String],
    connection_tolerance: f64,
) -> Vec<Violation> {
    let mut by_net_layer: BTreeMap<(String, String), Vec<&CopperFeature>> = BTreeMap::new();
    for feature in selected_copper_features(board, selected_layers) {
        let Some(net) = &feature.net else {
            continue;
        };
        by_net_layer
            .entry((net.clone(), feature.layer.clone()))
            .or_default()
            .push(feature);
    }

    let mut violations = Vec::new();
    for ((net, layer), features) in by_net_layer {
        if features.len() < 2 {
            continue;
        }
        let component_result = copper_components(&features, connection_tolerance);
        let components = component_result.components;
        log::trace!(
            "same-net island readiness: net={} layer={} features={} spatial_buckets={} exact_pairs={} components={}",
            net,
            layer,
            features.len(),
            component_result.spatial_buckets,
            component_result.exact_pairs,
            components.len()
        );
        if components.len() < 2 {
            continue;
        }

        let locations = components
            .iter()
            .filter_map(|component| component.first().map(|index| features[*index].location))
            .collect::<Vec<_>>();
        violations.push(Violation::new(
            "same-net-island-readiness",
            Severity::Warning,
            vec![layer],
            None,
            Vec::new(),
            locations,
            Some(format!(
                "net {net} appears as {} disconnected copper islands on one selected layer; review routing, zone refill, or intentional no-connect handling",
                components.len()
            )),
        ));
    }

    violations
}

/// Warn when likely high-current nets change layers with too few parsed vias.
///
/// IPC-2152 frames current carrying capacity as a design-specific board
/// constraint, while Black's electromigration survey, "Electromigration--A
/// Brief Survey and Some Recent Results", IEEE Transactions on Electron
/// Devices, 1969, motivates reviewing current density bottlenecks before
/// release. This check is intentionally conservative: it only flags
/// layer-changing likely power nets whose parsed same-net via count is below the
/// redundancy threshold used by this readiness profile.
pub fn high_current_readiness(board: &BoardModel, selected_layers: &[String]) -> Vec<Violation> {
    let mut nets: BTreeMap<String, NetLayerUse> = BTreeMap::new();
    let mut high_current_features = 0usize;

    for feature in selected_copper_features(board, selected_layers) {
        let Some(net) = &feature.net else {
            continue;
        };
        if !looks_high_current_net(net) {
            continue;
        }
        high_current_features += 1;

        let entry = nets.entry(net.clone()).or_default();
        entry.layers.insert(feature.layer.clone());
        entry.locations.push(feature.location);
        if feature.kind == CopperKind::Via {
            entry.via_count += 1;
        }
    }

    let net_count = nets.len();
    let violations = nets
        .into_iter()
        .filter(|(_, usage)| usage.layers.len() > 1 && usage.via_count < 2)
        .map(|(net, usage)| {
            let layers = usage.layers.into_iter().collect::<Vec<_>>();
            Violation::new(
                "high-current-readiness",
                Severity::Warning,
                layers.clone(),
                None,
                Vec::new(),
                usage.locations,
                Some(format!(
                    "likely high-current net {net} changes across {} copper layers with only {} parsed same-net via(s); review via array and current-sharing intent",
                    layers.len(),
                    usage.via_count
                )),
            )
        })
        .collect::<Vec<_>>();
    log::trace!(
        "high-current readiness: source={} high_current_features={} high_current_nets={} selected_layers={} violations={}",
        board.source,
        high_current_features,
        net_count,
        selected_layers.len(),
        violations.len()
    );

    violations
}

/// Run the `power_via_array_readiness` design-readiness check or report helper.
///
/// Same-net via neighbors are queried with a point-grid broad phase before the
/// exact pitch predicate, following Ericson, *Real-Time Collision Detection*
/// (2005). This keeps sparse high-current via fields from degenerating into
/// all-pairs center-distance scans.
pub fn power_via_array_readiness(
    board: &BoardModel,
    selected_layers: &[String],
    maximum_isolated_pitch: f64,
) -> Vec<Violation> {
    let mut nets: BTreeMap<String, ViaArrayUse> = BTreeMap::new();

    for feature in selected_copper_features(board, selected_layers) {
        let Some(net) = &feature.net else {
            continue;
        };
        if feature.kind != CopperKind::Via || !looks_high_current_net(net) {
            continue;
        }

        let entry = nets.entry(net.clone()).or_default();
        entry.layers.insert(feature.layer.clone());
        entry.locations.push(feature.location);
    }

    let mut violations = Vec::new();
    let mut indexed_nets = 0usize;
    let mut spatial_buckets = 0usize;
    let mut candidate_hits = 0usize;
    for (net, usage) in nets {
        if usage.locations.len() < 2 {
            continue;
        }

        let via_index =
            PointSpatialIndex::new(usage.locations.iter().copied(), maximum_isolated_pitch);
        indexed_nets += 1;
        spatial_buckets += via_index.bucket_count();
        let isolated = usage
            .locations
            .iter()
            .enumerate()
            .filter(|(location_index, location)| {
                let nearby = via_index.centers_within(**location, maximum_isolated_pitch);
                candidate_hits += nearby.len();
                !nearby.into_iter().any(|other_index| {
                    other_index != *location_index && usage.locations[other_index] != **location
                })
            })
            .map(|(_, location)| *location)
            .collect::<Vec<_>>();
        if isolated.is_empty() {
            continue;
        }

        violations.push(Violation::new(
            "power-via-array-readiness",
            Severity::Warning,
            usage.layers.into_iter().collect(),
            None,
            Vec::new(),
            isolated,
            Some(format!(
                "likely high-current net {net} has isolated vias farther than {maximum_isolated_pitch:.6} from the rest of the same-net via array"
            )),
        ));
    }
    log::trace!(
        "power-via array readiness: source={} indexed_nets={} spatial_buckets={} candidate_hits={} maximum_isolated_pitch={maximum_isolated_pitch:.6} violations={}",
        board.source,
        indexed_nets,
        spatial_buckets,
        candidate_hits,
        violations.len()
    );

    violations
}

/// Warn when likely high-current nets have no parsed same-net copper zone.
///
/// IPC-2152 frames current carrying capacity as a board-specific design
/// decision rather than a simple trace-width constant. This readiness check is
/// therefore conservative: it infers power intent from net names and asks for a
/// parsed pour/plane/zone on each selected high-current net so reviewers can
/// confirm current-spreading intent before release.
pub fn power_plane_readiness(board: &BoardModel, selected_layers: &[String]) -> Vec<Violation> {
    let mut nets: BTreeMap<String, NetLayerUse> = BTreeMap::new();
    let mut high_current_features = 0usize;

    for feature in selected_copper_features(board, selected_layers) {
        let Some(net) = &feature.net else {
            continue;
        };
        if !looks_high_current_net(net) {
            continue;
        }
        high_current_features += 1;

        let entry = nets.entry(net.clone()).or_default();
        entry.layers.insert(feature.layer.clone());
        entry.locations.push(feature.location);
        if feature.kind == CopperKind::Zone {
            entry.has_zone = true;
        }
    }

    let net_count = nets.len();
    let violations = nets
        .into_iter()
        .filter(|(_, usage)| !usage.has_zone)
        .map(|(net, usage)| {
            Violation::new(
                "power-plane-readiness",
                Severity::Warning,
                usage.layers.into_iter().collect(),
                None,
                Vec::new(),
                usage.locations,
                Some(format!(
                    "likely high-current net {net} has no parsed same-net copper zone on selected layers; review pour, plane, and current-spreading intent"
                )),
            )
        })
        .collect::<Vec<_>>();
    log::trace!(
        "power-plane readiness: source={} high_current_features={} high_current_nets={} selected_layers={} violations={}",
        board.source,
        high_current_features,
        net_count,
        selected_layers.len(),
        violations.len()
    );

    violations
}

/// Warn when likely high-current copper has a narrow local neck.
///
/// IPC-2152 covers board-level current-carrying capacity, and Black's
/// "Electromigration--A Brief Survey and Some Recent Results", IEEE
/// Transactions on Electron Devices, 1969, motivates reviewing local current
/// density constrictions. This check uses parsed copper bounds as a conservative
/// neck proxy, so suspect findings should be verified against native geometry,
/// stackup, copper weight, temperature rise, and current requirements.
pub fn high_current_neck_readiness(
    board: &BoardModel,
    selected_layers: &[String],
    minimum_power_width: f64,
) -> Vec<Violation> {
    let features = selected_copper_features(board, selected_layers);
    let mut high_current_features = 0usize;
    let mut measured_features = 0usize;
    let mut violations = Vec::new();

    for feature in &features {
        let Some(net) = feature.net.as_deref() else {
            continue;
        };
        if !looks_high_current_net(net) {
            continue;
        }
        high_current_features += 1;

        let width = minimum_bounding_dimension(&feature.sketch);
        if width <= 0.0 {
            continue;
        }
        measured_features += 1;
        if width >= minimum_power_width {
            continue;
        }

        violations.push(Violation::new(
            "high-current-neck-readiness",
            Severity::Warning,
            vec![feature.layer.clone()],
            None,
            Vec::new(),
            vec![feature.location],
            Some(format!(
                "likely high-current net {net} has {:?} copper neck width {width:.6} below preferred power width {minimum_power_width:.6}",
                feature.kind
            )),
        ));
    }

    log::trace!(
        "high-current neck readiness: source={} selected_features={} high_current_features={} measured_features={} selected_layers={} minimum_power_width={minimum_power_width:.6} violations={}",
        board.source,
        features.len(),
        high_current_features,
        measured_features,
        selected_layers.len(),
        violations.len()
    );

    violations
}

/// Warn when chassis/shield nets appear without a nearby parsed ground via for
/// stitching or bonding intent.
///
/// The parsed ground via centers are indexed with the same deterministic point
/// grid used by drill-table matching. Following Ericson, *Real-Time Collision
/// Detection* (2005), the grid is only a broad phase; the readiness rule remains
/// a center-distance bonding proxy that should be verified against the chassis
/// and enclosure design.
pub fn chassis_stitching_readiness(
    board: &BoardModel,
    selected_layers: &[String],
    stitching_distance: f64,
) -> Vec<Violation> {
    chassis_stitching_readiness_with_grid(
        board,
        selected_layers,
        stitching_distance,
        SourceGridFacts::PRIMITIVE_FLOAT_EDGE,
    )
}

/// Run chassis-stitching readiness with retained source-grid facts.
///
/// The point index is only a broad phase. The final center-radius decision
/// compares squared exact `Real` distances after lifting coordinates with
/// source-unit provenance, matching Yap's exact-geometric-computation boundary
/// from "Towards Exact Geometric Computation," Computational Geometry 7.1-2
/// (1997). The squared-distance reduction avoids a square root, as in
/// de Berg, Cheong, van Kreveld, and Overmars, *Computational Geometry:
/// Algorithms and Applications*, 3rd ed., Springer, 2008.
pub fn chassis_stitching_readiness_with_grid(
    board: &BoardModel,
    selected_layers: &[String],
    stitching_distance: f64,
    grid: SourceGridFacts,
) -> Vec<Violation> {
    let features = selected_copper_features(board, selected_layers);
    let ground_vias = features
        .iter()
        .copied()
        .filter(|feature| feature.kind == CopperKind::Via)
        .filter(|feature| feature.net.as_deref().is_some_and(looks_ground_net))
        .collect::<Vec<_>>();
    let ground_points = ground_vias
        .iter()
        .map(|feature| feature.location)
        .collect::<Vec<_>>();
    let ground_index = PointSpatialIndex::new(ground_points, stitching_distance);
    let mut violations = Vec::new();
    let mut candidate_features = 0usize;
    let mut stitch_hits = 0usize;

    for feature in features {
        let Some(net) = &feature.net else {
            continue;
        };
        if !looks_chassis_net(net) {
            continue;
        }
        candidate_features += 1;

        let nearby_stitches = point_candidates_within_radius_with_grid(
            &ground_index,
            feature.location,
            stitching_distance,
            grid,
            "chassis-stitching-readiness",
        );
        stitch_hits += nearby_stitches.len();
        let has_stitch = !nearby_stitches.is_empty();
        if has_stitch {
            continue;
        }

        violations.push(Violation::new(
            "chassis-stitching-readiness",
            Severity::Warning,
            vec![feature.layer.clone()],
            None,
            Vec::new(),
            vec![feature.location],
            Some(format!(
                "likely chassis or shield net {net} has no parsed ground stitching via within {stitching_distance:.6}; review shield bonding and EMC stitching intent"
            )),
        ));
    }

    log::trace!(
        "chassis stitching readiness: source={} candidate_features={} ground_vias={} ground_buckets={} stitch_hits={} stitching_distance={stitching_distance:.6} violations={}",
        board.source,
        candidate_features,
        ground_vias.len(),
        ground_index.bucket_count(),
        stitch_hits,
        violations.len()
    );

    violations
}

/// Warn when likely card-edge/gold-finger nets contain via copper.
///
/// IPC-4552B and IPC-4553A surface-finish guidance treats edge-contact finish,
/// plating, and wear surfaces as explicit fabrication intent. This readiness
/// check is deliberately conservative: it infers finger nets from names and
/// reports vias on those nets so reviewers can confirm no-via finger geometry,
/// bevel keepouts, and plating notes before release.
pub fn gold_finger_readiness(board: &BoardModel, selected_layers: &[String]) -> Vec<Violation> {
    let features = selected_copper_features(board, selected_layers);
    let mut finger_features = 0usize;
    let mut finger_vias = 0usize;
    let mut violations = Vec::new();

    for feature in &features {
        let Some(net) = feature.net.as_deref() else {
            continue;
        };
        if !looks_gold_finger_net(net) {
            continue;
        }
        finger_features += 1;
        if feature.kind != CopperKind::Via {
            continue;
        }
        finger_vias += 1;

        violations.push(Violation::new(
            "gold-finger-readiness",
            Severity::Warning,
            vec![feature.layer.clone()],
            None,
            Vec::new(),
            vec![feature.location],
            Some(format!(
                "likely gold-finger net {net} has via copper; review no-via finger plating and bevel keepout rules"
            )),
        ));
    }

    log::trace!(
        "gold-finger readiness: source={} selected_features={} finger_features={} finger_vias={} selected_layers={} violations={}",
        board.source,
        features.len(),
        finger_features,
        finger_vias,
        selected_layers.len(),
        violations.len()
    );

    violations
}

/// Warn when likely card-edge contact copper is not close to the board edge.
///
/// IPC-4552B and IPC-4553A surface-finish guidance makes contact plating an
/// explicit fabrication handoff. Card-edge contacts also depend on bevel and
/// edge placement, so this check treats likely finger pads/segments far from the
/// parsed outline as suspect release data. It uses exact polygon boundary
/// distance; naming and outline inference are still conservative readiness
/// signals that should be confirmed against the fabrication drawing.
pub fn gold_finger_edge_readiness(
    board: &BoardModel,
    selected_layers: &[String],
    edge_distance: f64,
) -> Vec<Violation> {
    let Some(outline) = &board.board_outline else {
        log::trace!(
            "gold-finger edge readiness: source={} selected_layers={} has_outline=false edge_distance={edge_distance:.6} violations=0",
            board.source,
            selected_layers.len()
        );
        return Vec::new();
    };

    let features = selected_copper_features(board, selected_layers);
    let mut finger_features = 0usize;
    let mut measured_features = 0usize;
    let mut violations = Vec::new();
    let outline_geometry = outline.to_multipolygon();

    for feature in &features {
        let Some(net) = feature.net.as_deref() else {
            continue;
        };
        if !looks_gold_finger_net(net)
            || !matches!(feature.kind, CopperKind::Pad | CopperKind::Segment)
        {
            continue;
        }
        finger_features += 1;
        let gap = polygon_boundary_distance(&feature.sketch.to_multipolygon(), &outline_geometry);
        if !gap.is_finite() {
            continue;
        }
        measured_features += 1;
        if gap <= edge_distance {
            continue;
        }

        violations.push(Violation::new(
            "gold-finger-edge-readiness",
            Severity::Warning,
            vec![feature.layer.clone()],
            None,
            Vec::new(),
            vec![feature.location],
            Some(format!(
                "likely gold-finger net {net} is {gap:.6} from board edge, beyond expected edge-finger band {edge_distance:.6}; review card-edge placement and bevel intent"
            )),
        ));
    }

    log::trace!(
        "gold-finger edge readiness: source={} selected_features={} finger_features={} measured_features={} selected_layers={} has_outline=true edge_distance={edge_distance:.6} violations={}",
        board.source,
        features.len(),
        finger_features,
        measured_features,
        selected_layers.len(),
        violations.len()
    );

    violations
}

/// Run the `gold_finger_spacing_readiness` design-readiness check or report helper.
///
/// Candidate contact pairs are selected with the shared spatial broad phase and
/// then checked with the exact offset/intersection predicate. This follows the
/// broad/narrow collision-query pattern described by Ericson, *Real-Time
/// Collision Detection* (2005), and keeps sparse card-edge connector fields from
/// degrading into all-pairs CSG work.
pub fn gold_finger_spacing_readiness(
    board: &BoardModel,
    selected_layers: &[String],
    minimum_spacing: f64,
    min_area: f64,
) -> Vec<Violation> {
    let fingers = selected_copper_features(board, selected_layers)
        .into_iter()
        .filter(|feature| feature.net.as_deref().is_some_and(looks_gold_finger_net))
        .filter(|feature| matches!(feature.kind, CopperKind::Pad | CopperKind::Segment))
        .collect::<Vec<_>>();
    let finger_index = CopperSpatialIndex::new(&fingers, minimum_spacing);
    let mut exact_pair_count = 0_usize;
    let mut violations = Vec::new();

    for left_index in 0..fingers.len() {
        let left = fingers[left_index];
        for right_index in finger_index.same_layer_near_feature(left, minimum_spacing) {
            if right_index <= left_index {
                continue;
            }
            let right = fingers[right_index];
            if left.net == right.net {
                continue;
            }

            exact_pair_count += 1;
            let overlap = left
                .sketch
                .offset(minimum_spacing)
                .intersection(&right.sketch);
            let shapes = multipolygon_to_shapes(&overlap.to_multipolygon(), min_area);
            let fallback_hit = shapes.is_empty()
                && polygon_boundary_distance(
                    &left.sketch.to_multipolygon(),
                    &right.sketch.to_multipolygon(),
                ) <= minimum_spacing;
            if shapes.is_empty() && !fallback_hit {
                continue;
            }

            violations.push(Violation::new(
                "gold-finger-spacing-readiness",
                Severity::Warning,
                vec![left.layer.clone()],
                None,
                shapes,
                vec![left.location, right.location],
                Some(format!(
                    "likely gold-finger nets {:?} and {:?} are within finger spacing {minimum_spacing:.6}; review contact pitch, plating, mask opening, and bevel tolerances",
                    left.net, right.net
                )),
            ));
        }
    }
    log::trace!(
        "gold-finger spacing readiness: source={} fingers={} spatial_buckets={} exact_pairs={} minimum_spacing={minimum_spacing:.6} violations={}",
        board.source,
        fingers.len(),
        finger_index.bucket_count(),
        exact_pair_count,
        violations.len()
    );

    violations
}

/// Run the `gold_finger_drill_keepout_readiness` design-readiness check or report helper.
///
/// Drill keepouts are circular mechanical blockers and gold-finger copper can
/// be sparse on large panels, so the check uses the shared spatial broad phase
/// before exact keepout/copper CSG intersection. This follows the
/// broad/narrow-phase collision pattern in Ericson, *Real-Time Collision
/// Detection* (2005).
pub fn gold_finger_drill_keepout_readiness(
    board: &BoardModel,
    extra_drills: &[DrillFeature],
    selected_layers: &[String],
    keepout: f64,
    min_area: f64,
) -> Vec<Violation> {
    let finger_features = selected_copper_features(board, selected_layers)
        .into_iter()
        .filter(|feature| feature.net.as_deref().is_some_and(looks_gold_finger_net))
        .collect::<Vec<_>>();
    let finger_index = CopperSpatialIndex::new(&finger_features, keepout);
    let mut drills = board.drills.clone();
    drills.extend_from_slice(extra_drills);
    let mut exact_pair_count = 0_usize;
    let mut violations = Vec::new();

    for drill in &drills {
        let keepout_radius = drill.diameter / 2.0 + keepout;
        let keepout_sketch = polygons_to_sketch(
            vec![circle_polygon(drill.location, keepout_radius, 32)],
            Some(LayerMetadata {
                name: "gold finger drill keepout".to_string(),
            }),
        );

        for finger_index in finger_index.all_layers_near_circle(drill.location, keepout_radius) {
            let finger = finger_features[finger_index];
            exact_pair_count += 1;
            let overlap = keepout_sketch.intersection(&finger.sketch);
            let shapes = multipolygon_to_shapes(&overlap.to_multipolygon(), min_area);
            let fallback_hit = shapes.is_empty()
                && polygon_boundary_distance(
                    &keepout_sketch.to_multipolygon(),
                    &finger.sketch.to_multipolygon(),
                ) <= 1.0e-9;
            if shapes.is_empty() && !fallback_hit {
                continue;
            }

            violations.push(Violation::new(
                "gold-finger-drill-keepout-readiness",
                Severity::Warning,
                vec![finger.layer.clone()],
                None,
                shapes,
                vec![drill.location, finger.location],
                Some(format!(
                    "likely gold-finger copper {:?} intersects drill/mechanical keepout {keepout:.6}; review no-drill finger plating and bevel keepout",
                    finger.net
                )),
            ));
        }
    }
    log::trace!(
        "gold-finger drill keepout readiness: source={} fingers={} drills={} spatial_buckets={} exact_pairs={} keepout={keepout:.6} violations={}",
        board.source,
        finger_features.len(),
        drills.len(),
        finger_index.bucket_count(),
        exact_pair_count,
        violations.len()
    );

    violations
}

/// Run the `connector_return_path_readiness` design-readiness check or report helper.
///
/// This is a center-proximity readiness proxy for edge connector return intent.
/// Same-layer ground candidates use the shared copper spatial index before the
/// exact center-distance predicate, following Ericson, *Real-Time Collision
/// Detection* (2005). The finding still needs schematic, stackup, and field
/// review before it becomes release-blocking.
pub fn connector_return_path_readiness(
    board: &BoardModel,
    selected_layers: &[String],
    edge_distance: f64,
    ground_search_radius: f64,
) -> Vec<Violation> {
    let Some(outline) = &board.board_outline else {
        return Vec::new();
    };

    let features = selected_copper_features(board, selected_layers);
    let ground_features = features
        .iter()
        .copied()
        .filter(|feature| feature.net.as_deref().is_some_and(looks_ground_net))
        .collect::<Vec<_>>();
    let ground_index = CopperSpatialIndex::new(&ground_features, ground_search_radius);
    let mut violations = Vec::new();
    let mut candidate_features = 0usize;
    let mut ground_hits = 0usize;

    for feature in features {
        let Some(net) = feature.net.as_deref() else {
            continue;
        };
        if !looks_connector_edge_rate_net(net) {
            continue;
        }
        candidate_features += 1;

        let edge_gap = polygon_boundary_distance(
            &feature.sketch.to_multipolygon(),
            &outline.to_multipolygon(),
        );
        if edge_gap > edge_distance {
            continue;
        }
        let nearby_ground = ground_index.same_layer_centers_within(
            feature.location,
            &feature.layer,
            ground_search_radius,
        );
        ground_hits += nearby_ground.len();
        let has_ground_return = !nearby_ground.is_empty();
        if has_ground_return {
            continue;
        }

        violations.push(Violation::new(
            "connector-return-path-readiness",
            Severity::Warning,
            vec![feature.layer.clone()],
            None,
            Vec::new(),
            vec![feature.location],
            Some(format!(
                "likely connector edge-rate net {net:?} is {edge_gap:.6} from board edge without parsed same-layer ground return copper within {ground_search_radius:.6}"
            )),
        ));
    }
    log::trace!(
        "connector return-path readiness: source={} candidate_features={} ground_features={} ground_buckets={} ground_hits={} edge_distance={edge_distance:.6} ground_search_radius={ground_search_radius:.6} violations={}",
        board.source,
        candidate_features,
        ground_features.len(),
        ground_index.bucket_count(),
        ground_hits,
        violations.len()
    );

    violations
}

/// Run the `decoupling_proximity_readiness` design-readiness check or report helper.
///
/// This is a loop-area readiness proxy, not a placement optimizer. Same-layer
/// ground candidates are selected through the Ericson-style grid broad phase in
/// `CopperSpatialIndex` before exact center-distance review.
pub fn decoupling_proximity_readiness(
    board: &BoardModel,
    selected_layers: &[String],
    ground_search_radius: f64,
) -> Vec<Violation> {
    let features = selected_copper_features(board, selected_layers);
    let ground_features = features
        .iter()
        .copied()
        .filter(|feature| feature.net.as_deref().is_some_and(looks_ground_net))
        .collect::<Vec<_>>();
    let ground_index = CopperSpatialIndex::new(&ground_features, ground_search_radius);
    let mut violations = Vec::new();
    let mut candidate_features = 0usize;
    let mut ground_hits = 0usize;

    for feature in features {
        if !matches!(feature.kind, CopperKind::Pad | CopperKind::Via) {
            continue;
        }
        let Some(net) = feature.net.as_deref() else {
            continue;
        };
        if !looks_high_current_net(net) {
            continue;
        }
        candidate_features += 1;

        let nearby_ground = ground_index.same_layer_centers_within(
            feature.location,
            &feature.layer,
            ground_search_radius,
        );
        ground_hits += nearby_ground.len();
        let has_nearby_ground = !nearby_ground.is_empty();
        if has_nearby_ground {
            continue;
        }

        violations.push(Violation::new(
            "decoupling-proximity-readiness",
            Severity::Warning,
            vec![feature.layer.clone()],
            None,
            Vec::new(),
            vec![feature.location],
            Some(format!(
                "likely power feature on net {net:?} has no parsed same-layer ground copper within {ground_search_radius:.6}; review decoupling capacitor loop area and return proximity"
            )),
        ));
    }
    log::trace!(
        "decoupling proximity readiness: source={} candidate_features={} ground_features={} ground_buckets={} ground_hits={} ground_search_radius={ground_search_radius:.6} violations={}",
        board.source,
        candidate_features,
        ground_features.len(),
        ground_index.bucket_count(),
        ground_hits,
        violations.len()
    );

    violations
}

/// Run the `return_path_readiness` design-readiness check or report helper.
///
/// High-speed via transitions are matched to nearby ground stitching via centers
/// with a point-grid broad phase before exact center-radius review. This follows
/// the broad/narrow phase pattern from Ericson, *Real-Time Collision Detection*
/// (2005), and keeps sparse ground-stitch fields bounded while retaining the
/// documented return-path heuristic.
pub fn return_path_readiness(
    board: &BoardModel,
    stitching_distance: f64,
    selected_layers: &[String],
) -> Vec<Violation> {
    let features = selected_copper_features(board, selected_layers);
    let ground_vias = features
        .iter()
        .filter(|feature| feature.kind == CopperKind::Via)
        .filter(|feature| feature.net.as_deref().is_some_and(looks_ground_net))
        .copied()
        .collect::<Vec<_>>();
    let ground_points = ground_vias
        .iter()
        .map(|feature| feature.location)
        .collect::<Vec<_>>();
    let ground_index = PointSpatialIndex::new(ground_points, stitching_distance);
    let signal_vias = features
        .iter()
        .filter(|feature| feature.kind == CopperKind::Via)
        .filter(|feature| feature.net.as_deref().is_some_and(looks_high_speed_net))
        .copied()
        .collect::<Vec<_>>();
    let mut violations = Vec::new();
    let mut stitch_hits = 0usize;

    for via in signal_vias {
        let nearby_ground = ground_index.centers_within(via.location, stitching_distance);
        stitch_hits += nearby_ground.len();
        let has_nearby_ground = !nearby_ground.is_empty();
        if has_nearby_ground {
            continue;
        }

        violations.push(Violation::new(
            "return-path-readiness",
            Severity::Warning,
            vec![via.layer.clone()],
            None,
            Vec::new(),
            vec![via.location],
            Some(format!(
                "likely high-speed net {:?} changes layers without a parsed ground stitching via within {stitching_distance:.6}",
                via.net
            )),
        ));
    }

    log::trace!(
        "return-path readiness: source={} signal_vias={} ground_vias={} ground_buckets={} stitch_hits={} stitching_distance={stitching_distance:.6} violations={}",
        board.source,
        features
            .iter()
            .filter(|feature| feature.kind == CopperKind::Via)
            .filter(|feature| feature.net.as_deref().is_some_and(looks_high_speed_net))
            .count(),
        ground_vias.len(),
        ground_index.bucket_count(),
        stitch_hits,
        violations.len()
    );

    violations
}

#[derive(Default)]
struct NetLayerUse {
    layers: BTreeSet<String>,
    locations: Vec<[f64; 2]>,
    has_via: bool,
    via_count: usize,
    has_zone: bool,
}

#[derive(Copy, Clone, Eq, PartialEq)]
enum DifferentialSide {
    Positive,
    Negative,
}

#[derive(Default)]
struct DifferentialPairUse {
    positive_layers: BTreeSet<String>,
    negative_layers: BTreeSet<String>,
    positive_locations: Vec<[f64; 2]>,
    negative_locations: Vec<[f64; 2]>,
}

#[derive(Default)]
struct DifferentialPairFeatureUse<'a> {
    positive: Vec<&'a CopperFeature>,
    negative: Vec<&'a CopperFeature>,
}

#[derive(Default)]
struct DifferentialPairViaUse {
    positive_via_count: usize,
    negative_via_count: usize,
    positive_via_layers: BTreeSet<String>,
    negative_via_layers: BTreeSet<String>,
    positive_via_locations: Vec<[f64; 2]>,
    negative_via_locations: Vec<[f64; 2]>,
}

#[derive(Default)]
struct ViaArrayUse {
    layers: BTreeSet<String>,
    locations: Vec<[f64; 2]>,
}

fn differential_pair_key(net: &str) -> Option<(String, DifferentialSide)> {
    let normalized = net.trim().to_ascii_uppercase();
    let patterns = [
        ("_DP", DifferentialSide::Positive),
        ("-DP", DifferentialSide::Positive),
        (".DP", DifferentialSide::Positive),
        ("_DM", DifferentialSide::Negative),
        ("-DM", DifferentialSide::Negative),
        (".DM", DifferentialSide::Negative),
        ("D+", DifferentialSide::Positive),
        ("D-", DifferentialSide::Negative),
        ("_P", DifferentialSide::Positive),
        ("-P", DifferentialSide::Positive),
        (".P", DifferentialSide::Positive),
        ("_N", DifferentialSide::Negative),
        ("-N", DifferentialSide::Negative),
        (".N", DifferentialSide::Negative),
    ];

    for (suffix, side) in patterns {
        let Some(base) = normalized.strip_suffix(suffix) else {
            continue;
        };
        let base = base
            .trim_end_matches(['_', '-', '.', '/', ' '])
            .trim_start_matches(['_', '-', '.', '/', ' ']);
        if !base.is_empty() {
            return Some((base.to_string(), side));
        }
    }

    None
}

fn looks_high_speed_net(net: &str) -> bool {
    let normalized = net.to_ascii_uppercase();
    let tokens = [
        "USB", "D+", "D-", "DP", "DM", "CLK", "CLOCK", "TX", "RX", "SERDES", "PCIE", "PCI", "MIPI",
        "LVDS", "HDMI", "ETH", "RGMII", "SGMII", "SATA", "CAN",
    ];

    tokens.iter().any(|token| normalized.contains(token))
}

fn looks_high_current_net(net: &str) -> bool {
    let normalized = net.to_ascii_uppercase();
    let tokens = [
        "VBAT", "VBUS", "VIN", "VCC", "VDD", "VOUT", "PWR", "POWER", "MOTOR", "PHASE", "+12V",
        "+5V", "+3V3", "12V", "5V", "3V3", "1V8",
    ];

    tokens.iter().any(|token| normalized.contains(token))
}

fn looks_high_voltage_net(net: &str) -> bool {
    let normalized = net.to_ascii_uppercase();
    let tokens = [
        "HV", "HIGHV", "MAINS", "LINE", "NEUTRAL", "LIVE", "VAC", "AC_L", "AC_N", "RECT", "BULK",
        "400V", "240V", "230V", "120V", "48V",
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

fn looks_gold_finger_net(net: &str) -> bool {
    let normalized = net.to_ascii_uppercase();
    let tokens = ["GOLD", "FINGER", "EDGE", "CARD_EDGE", "CONN_EDGE"];

    tokens.iter().any(|token| normalized.contains(token))
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

fn looks_edge_intent_net(net: &str) -> bool {
    looks_gold_finger_net(net) || looks_chassis_net(net)
}

/// Review selected same-layer KiCad copper for different-net spacing.
///
/// Candidate generation uses the shared broad/narrow-phase grid described by
/// Ericson, *Real-Time Collision Detection* (2005), and every surviving pair
/// still runs the exact offset/intersection predicate below. The exact
/// clearance test models the Minkowski-style offset region discussed in Lee and
/// Preparata, "Computational Geometry - A Survey", IEEE TC, 1984.
pub fn net_spacing(
    board: &BoardModel,
    clearance: f64,
    selected_layers: &[String],
    min_area: f64,
) -> Vec<Violation> {
    let features = selected_copper_features(board, selected_layers)
        .into_iter()
        .filter(|feature| feature.sketch.geometry().bounding_rect().is_some())
        .collect::<Vec<_>>();
    let bounds = features
        .iter()
        .map(|feature| {
            feature
                .sketch
                .geometry()
                .bounding_rect()
                .expect("net-spacing features are filtered to bounded geometry")
        })
        .collect::<Vec<_>>();
    let feature_index = CopperSpatialIndex::new(&features, clearance);
    let mut violations = Vec::new();
    let mut candidate_pairs = 0_usize;
    let mut exact_pairs = 0_usize;

    for left_index in 0..features.len() {
        let left = features[left_index];
        if left.net.is_none() {
            continue;
        }
        for right_index in feature_index.same_layer_near_feature(left, clearance) {
            if right_index <= left_index {
                continue;
            }
            candidate_pairs += 1;
            let right = features[right_index];
            if left.net == right.net {
                continue;
            }
            if !rects_within_clearance(&bounds[left_index], &bounds[right_index], clearance) {
                continue;
            }
            exact_pairs += 1;
            collect_net_spacing_violation(left, right, clearance, min_area, &mut violations);
        }
    }

    log::trace!(
        "different-net spacing: source={} features={} buckets={} candidate_pairs={} exact_pairs={} clearance={clearance:.6} selected_layers={} violations={}",
        board.source,
        features.len(),
        feature_index.bucket_count(),
        candidate_pairs,
        exact_pairs,
        selected_layers.len(),
        violations.len()
    );

    violations
}

fn collect_net_spacing_violation(
    left: &CopperFeature,
    right: &CopperFeature,
    clearance: f64,
    min_area: f64,
    violations: &mut Vec<Violation>,
) {
    if !sketches_within_clearance(&left.sketch, &right.sketch, clearance) {
        return;
    }

    // Clearance is modeled by a Minkowski sum of the left copper feature with a
    // disk of radius `clearance`, followed by an intersection with the right
    // feature. In computational geometry terms this is a set-membership test
    // against an offset region; see Lee and Preparata, "Computational Geometry -
    // A Survey", IEEE TC, 1984.
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
        return;
    }

    violations.push(Violation::new(
        "different-net-spacing",
        Severity::Error,
        vec![left.layer.clone()],
        None,
        shapes,
        locations,
        Some(format!(
            "net {:?} is within {clearance} of net {:?}",
            left.net, right.net
        )),
    ));
}

fn rects_within_clearance(left: &geo::Rect<f64>, right: &geo::Rect<f64>, clearance: f64) -> bool {
    left.min().x - clearance <= right.max().x
        && left.max().x + clearance >= right.min().x
        && left.min().y - clearance <= right.max().y
        && left.max().y + clearance >= right.min().y
}

/// Review cross-layer copper proximity under fabrication registration tolerance.
///
/// Like [`net_spacing`], this uses Ericson's broad/narrow-phase pattern from
/// *Real-Time Collision Detection* (2005): a layer-aware spatial grid proposes
/// cross-layer feature pairs, and the exact offset/intersection predicate makes
/// the finding decision. The exact test is the same Minkowski-style offset
/// region described by Lee and Preparata, "Computational Geometry - A Survey",
/// IEEE TC, 1984.
pub fn registration_tolerance(board: &BoardModel, tolerance: f64, min_area: f64) -> Vec<Violation> {
    let features = selected_copper_features(board, &[])
        .into_iter()
        .filter(|feature| feature.sketch.geometry().bounding_rect().is_some())
        .collect::<Vec<_>>();
    let bounds = features
        .iter()
        .map(|feature| {
            feature
                .sketch
                .geometry()
                .bounding_rect()
                .expect("registration-tolerance features are filtered to bounded geometry")
        })
        .collect::<Vec<_>>();
    let feature_index = CopperSpatialIndex::new(&features, tolerance);
    let mut violations = Vec::new();
    let layers = features
        .iter()
        .map(|feature| feature.layer.clone())
        .collect::<BTreeSet<_>>();
    let mut candidate_pairs = 0_usize;
    let mut exact_pairs = 0_usize;

    for left_index in 0..features.len() {
        let left = features[left_index];
        for right_index in feature_index.all_layers_near_feature(left, tolerance) {
            if right_index <= left_index {
                continue;
            }
            let right = features[right_index];
            if left.layer == right.layer {
                continue;
            }
            candidate_pairs += 1;
            if !rects_within_clearance(&bounds[left_index], &bounds[right_index], tolerance) {
                continue;
            }
            exact_pairs += 1;
            let (first, second) = if left.layer <= right.layer {
                (left, right)
            } else {
                (right, left)
            };
            let first_layer = first.layer.clone();
            let second_layer = second.layer.clone();
            collect_registration_tolerance_violation(
                first,
                second,
                &first_layer,
                &second_layer,
                tolerance,
                min_area,
                &mut violations,
            );
        }
    }

    log::trace!(
        "registration tolerance: source={} features={} layers={} buckets={} candidate_pairs={} exact_pairs={} tolerance={tolerance:.6} violations={}",
        board.source,
        features.len(),
        layers.len(),
        feature_index.bucket_count(),
        candidate_pairs,
        exact_pairs,
        violations.len()
    );

    violations
}

fn collect_registration_tolerance_violation(
    left: &CopperFeature,
    right: &CopperFeature,
    left_layer: &str,
    right_layer: &str,
    tolerance: f64,
    min_area: f64,
    violations: &mut Vec<Violation>,
) {
    if !sketches_within_clearance(&left.sketch, &right.sketch, tolerance) {
        return;
    }

    // Treat registration tolerance as a feature-level proximity query rather
    // than a whole-layer boolean operation. Whole copper layers can contain
    // thousands of disconnected islands; broad-phase feature culling keeps the
    // exact Minkowski offset bounded while preserving the same conservative
    // geometric predicate.
    let overlap = left.sketch.offset(tolerance).intersection(&right.sketch);
    let shapes = multipolygon_to_shapes(&overlap.to_multipolygon(), min_area);
    let locations = if shapes.is_empty()
        && polygon_boundary_distance(
            &left.sketch.to_multipolygon(),
            &right.sketch.to_multipolygon(),
        ) <= tolerance
    {
        vec![left.location, right.location]
    } else {
        Vec::new()
    };
    if shapes.is_empty() && locations.is_empty() {
        return;
    }

    violations.push(Violation::new(
        "layer-registration-tolerance",
        Severity::Warning,
        vec![left_layer.to_string(), right_layer.to_string()],
        None,
        shapes,
        locations,
        Some(format!(
            "features on paired layers are within registration tolerance {tolerance}"
        )),
    ));
}

/// Run the `panelization_clearance` design-readiness check or report helper.
///
/// Panel tabs, V-scores, route graphics, and stamp-hole drills are treated as
/// blocker geometry. Copper/blocker pairs first pass an AABB broad phase in the
/// Ericson, *Real-Time Collision Detection* (2005) sense; exact intersection
/// and boundary-distance checks still decide every reported clearance finding.
pub fn panelization_clearance(
    board: &BoardModel,
    extra_drills: &[DrillFeature],
    clearance: f64,
    min_area: f64,
) -> Vec<Violation> {
    let mut blockers = Vec::new();

    if let Some(panel_features) = &board.panel_features {
        blockers.push(panel_features.clone());
    }

    if !extra_drills.is_empty() {
        blockers.push(drills_to_sketch(extra_drills, "Excellon panel drills"));
    }

    let npth = board
        .drills
        .iter()
        .filter(|drill| !drill.plated)
        .cloned()
        .collect::<Vec<_>>();
    if !npth.is_empty() {
        blockers.push(drills_to_sketch(&npth, "KiCad NPTH panel drills"));
    }

    let bounded_copper = board
        .copper
        .iter()
        .filter_map(|copper| {
            copper
                .sketch
                .geometry()
                .bounding_rect()
                .map(|bounds| (copper, bounds))
        })
        .collect::<Vec<_>>();
    let mut violations = Vec::new();
    let blocker_count = blockers.len();
    let mut blocker_polygon_count = 0_usize;
    let mut candidate_pairs = 0_usize;
    let mut exact_intersections = 0_usize;
    let mut fallback_hits = 0_usize;
    for blocker in blockers {
        let mut shapes = Vec::new();
        let mut fallback_hit = false;
        let mut locations = Vec::new();
        let mut layers = BTreeSet::new();

        for blocker_polygon in blocker.to_multipolygon().0 {
            blocker_polygon_count += 1;
            let blocker_piece = polygons_to_sketch(vec![blocker_polygon], None);
            let blocker_geometry = blocker_piece.to_multipolygon();
            let Some(blocker_bounds) = blocker_piece.geometry().bounding_rect() else {
                continue;
            };

            for (copper, copper_bounds) in &bounded_copper {
                if !rects_within_clearance(&blocker_bounds, copper_bounds, clearance) {
                    continue;
                }

                candidate_pairs += 1;
                // Panel features can be long routed tabs or score lines.
                // Offsetting the entire panel sketch is unnecessarily expensive
                // on dense boards, so use exact feature intersections plus the
                // same boundary distance predicate the offset fallback used to
                // represent copper inside the requested keepout band.
                exact_intersections += 1;
                let overlap = blocker_piece.intersection(&copper.sketch);
                let feature_shapes = multipolygon_to_shapes(&overlap.to_multipolygon(), min_area);
                if feature_shapes.is_empty()
                    && polygon_boundary_distance(
                        &blocker_geometry,
                        &copper.sketch.to_multipolygon(),
                    ) <= clearance
                {
                    fallback_hit = true;
                    fallback_hits += 1;
                    locations.push(copper.location);
                }
                shapes.extend(feature_shapes);
                layers.insert(copper.layer.clone());
            }
        }

        if shapes.is_empty() && !fallback_hit {
            continue;
        }

        violations.push(Violation::new(
            "panelization-clearance",
            Severity::Warning,
            if layers.is_empty() {
                vec!["KiCad copper".to_string()]
            } else {
                layers.into_iter().collect()
            },
            None,
            shapes,
            locations,
            Some(format!(
                "copper is within panel feature clearance {clearance}"
            )),
        ));
    }

    log::trace!(
        "panelization clearance: source={} blockers={} blocker_polygons={} bounded_copper={} candidate_pairs={} exact_intersections={} fallback_hits={} clearance={clearance:.6} min_area={min_area:.9} violations={}",
        board.source,
        blocker_count,
        blocker_polygon_count,
        bounded_copper.len(),
        candidate_pairs,
        exact_intersections,
        fallback_hits,
        violations.len()
    );

    violations
}

/// Run the `apply_ipc356_nets` design-readiness check or report helper.
///
/// IPC-D-356 records are point-like electrical-test observations. Matching them
/// against parsed KiCad feature centers uses the same grid broad phase described
/// by Ericson, *Real-Time Collision Detection* (2005), before the exact
/// center-distance predicate. This keeps sidecar annotation linear-ish on large
/// fixture files and avoids assigning drill diameter metadata from unrelated
/// distant records.
pub fn apply_ipc356_nets(board: &mut BoardModel, points: &[Ipc356Point], tolerance: f64) {
    let point_index = PointSpatialIndex::new(points.iter().map(|point| point.location), tolerance);
    let copper_locations = board
        .copper
        .iter()
        .map(|copper| copper.location)
        .collect::<Vec<_>>();
    let copper_index = PointSpatialIndex::new(copper_locations, tolerance);
    let drill_locations = board
        .drills
        .iter()
        .map(|drill| drill.location)
        .collect::<Vec<_>>();
    let drill_index = PointSpatialIndex::new(drill_locations, tolerance);
    let mut copper_matches = 0_usize;
    let mut drill_matches = 0_usize;

    for point in points {
        for copper_index in copper_index.centers_within(point.location, tolerance) {
            let copper = &mut board.copper[copper_index];
            if copper.net.is_none() {
                copper.net = Some(point.net.clone());
                copper_matches += 1;
            }
        }

        for drill_index in drill_index.centers_within(point.location, tolerance) {
            let drill = &mut board.drills[drill_index];
            if drill.net.is_none() {
                drill.net = Some(point.net.clone());
                drill_matches += 1;
            }
            if drill.diameter == 0.0
                && let Some(diameter) = point.diameter
            {
                drill.diameter = diameter;
            }
        }
    }

    log::trace!(
        "apply IPC-D-356 nets: points={} point_buckets={} copper_buckets={} drill_buckets={} copper_matches={} drill_matches={} tolerance={tolerance:.6}",
        points.len(),
        point_index.bucket_count(),
        copper_index.bucket_count(),
        drill_index.bucket_count(),
        copper_matches,
        drill_matches
    );
}

/// Run the `ipc356_coverage` design-readiness check or report helper.
///
/// Coverage is a point-to-center proximity query, so the check uses a
/// deterministic point spatial index as a broad phase before exact distance.
/// This follows Ericson's broad/narrow collision-detection pattern and keeps
/// large ICT/netlist sidecars bounded.
pub fn ipc356_coverage(
    board: &BoardModel,
    points: &[Ipc356Point],
    tolerance: f64,
) -> Vec<Violation> {
    let copper_index = PointSpatialIndex::new(
        board.copper.iter().map(|feature| feature.location),
        tolerance,
    );
    let mut violations = Vec::new();

    for point in points {
        let has_copper = !copper_index
            .centers_within(point.location, tolerance)
            .is_empty();
        if has_copper {
            continue;
        }

        let label = match (&point.reference, &point.pin) {
            (Some(reference), Some(pin)) => format!("{reference}.{pin}"),
            (Some(reference), None) => reference.clone(),
            _ => "IPC-D-356 test record".to_string(),
        };
        violations.push(Violation::new(
            "ipc356-coverage",
            Severity::Warning,
            vec![point.net.clone()],
            None,
            Vec::new(),
            vec![point.location],
            Some(format!(
                "{label} has no parsed KiCad copper feature within {tolerance}"
            )),
        ));
    }
    log::trace!(
        "IPC-D-356 coverage readiness: points={} copper={} spatial_buckets={} tolerance={tolerance:.6} violations={}",
        points.len(),
        board.copper.len(),
        copper_index.bucket_count(),
        violations.len()
    );

    violations
}

/// Run the `ipc356_drill_diameter` design-readiness check or report helper.
///
/// Diameter comparison uses a drill-center grid as a broad phase before exact
/// tolerance comparison. This keeps cross-source drill-table checks bounded on
/// dense IPC-D-356 fixtures while preserving the caller-visible diagnostic.
pub fn ipc356_drill_diameter(
    board: &BoardModel,
    points: &[Ipc356Point],
    tolerance: f64,
) -> Vec<Violation> {
    let drill_index = DrillSpatialIndex::new(&board.drills, tolerance);
    let mut candidate_count = 0_usize;
    let mut violations = Vec::new();

    for point in points {
        let Some(ipc_diameter) = point.diameter else {
            continue;
        };
        for drill_index in drill_index.centers_within(point.location, tolerance) {
            candidate_count += 1;
            let drill = &board.drills[drill_index];
            if drill.diameter == 0.0 || (drill.diameter - ipc_diameter).abs() <= tolerance {
                continue;
            }

            violations.push(Violation::new(
                "ipc356-drill-diameter",
                Severity::Warning,
                vec![point.net.clone()],
                None,
                Vec::new(),
                vec![drill.location, point.location],
                Some(format!(
                    "drill diameter {:.6} differs from IPC-D-356 diameter {:.6}",
                    drill.diameter, ipc_diameter
                )),
            ));
        }
    }
    log::trace!(
        "IPC-D-356 drill diameter readiness: points={} drills={} spatial_buckets={} candidate_pairs={} tolerance={tolerance:.6} violations={}",
        points.len(),
        board.drills.len(),
        drill_index.bucket_count(),
        candidate_count,
        violations.len()
    );

    violations
}

fn copper_features_touch(left: &CopperFeature, right: &CopperFeature, tolerance: f64) -> bool {
    if !sketches_within_clearance(&left.sketch, &right.sketch, tolerance) {
        return false;
    }

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

fn rects_overlap(left: &geo::Rect<f64>, right: &geo::Rect<f64>) -> bool {
    left.min().x <= right.max().x
        && left.max().x >= right.min().x
        && left.min().y <= right.max().y
        && left.max().y >= right.min().y
}

struct CopperComponentResult {
    components: Vec<Vec<usize>>,
    spatial_buckets: usize,
    exact_pairs: usize,
}

fn copper_components(features: &[&CopperFeature], tolerance: f64) -> CopperComponentResult {
    let feature_index = CopperSpatialIndex::new(features, tolerance);
    let mut visited = vec![false; features.len()];
    let mut components = Vec::new();
    let mut exact_pairs = 0usize;

    for start in 0..features.len() {
        if visited[start] {
            continue;
        }

        let mut stack = vec![start];
        let mut component = Vec::new();
        visited[start] = true;
        while let Some(index) = stack.pop() {
            component.push(index);
            for candidate in feature_index.same_layer_near_feature(features[index], tolerance) {
                if visited[candidate] {
                    continue;
                }
                exact_pairs += 1;
                if !copper_features_touch(features[index], features[candidate], tolerance) {
                    continue;
                }
                visited[candidate] = true;
                stack.push(candidate);
            }
        }
        components.push(component);
    }

    CopperComponentResult {
        components,
        spatial_buckets: feature_index.bucket_count(),
        exact_pairs,
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

fn minimum_bounding_dimension(sketch: &PcbSketch) -> f64 {
    sketch
        .geometry()
        .bounding_rect()
        .map(|bounds| (bounds.max().x - bounds.min().x).min(bounds.max().y - bounds.min().y))
        .unwrap_or(0.0)
}

fn sketches_within_clearance(left: &PcbSketch, right: &PcbSketch, clearance: f64) -> bool {
    let Some(left_bounds) = left.geometry().bounding_rect() else {
        return true;
    };
    let Some(right_bounds) = right.geometry().bounding_rect() else {
        return true;
    };

    // Axis-aligned bounding boxes are used only as a broad-phase rejection
    // before exact polygon predicates. This is the standard two-phase collision
    // pattern described by Lin and Canny, "A Fast Algorithm for Incremental
    // Distance Calculation", IEEE ICRA, 1991: cheap conservative culling first,
    // exact distance/intersection only for surviving candidates.
    left_bounds.min().x - clearance <= right_bounds.max().x
        && left_bounds.max().x + clearance >= right_bounds.min().x
        && left_bounds.min().y - clearance <= right_bounds.max().y
        && left_bounds.max().y + clearance >= right_bounds.min().y
}

fn point_candidates_within_radius_with_grid(
    index: &PointSpatialIndex,
    center: [f64; 2],
    radius: f64,
    grid: SourceGridFacts,
    rule_name: &'static str,
) -> Vec<usize> {
    index
        .candidate_centers_near(center, radius)
        .into_iter()
        .filter(|&candidate| {
            point_within_radius_with_grid(index.point(candidate), center, radius, grid, rule_name)
        })
        .collect()
}

fn point_within_radius_with_grid(
    point: [f64; 2],
    center: [f64; 2],
    radius: f64,
    grid: SourceGridFacts,
    rule_name: &'static str,
) -> bool {
    // The bucket lookup is only a broad phase. This predicate compares
    // `dx*dx + dy*dy <= r*r` after lifting all finite compatibility values into
    // exact `Real`s with source-unit facts. That keeps the center-radius
    // decision inside Yap's EGC model while using the standard squared-distance
    // reduction from de Berg et al., *Computational Geometry: Algorithms and
    // Applications*, 3rd ed., Springer, 2008.
    let provenance = RuleGeometryProvenance::new(rule_name, grid);
    let Some(point_x) = provenance.lift_f64(point[0]) else {
        return false;
    };
    let Some(point_y) = provenance.lift_f64(point[1]) else {
        return false;
    };
    let Some(center_x) = provenance.lift_f64(center[0]) else {
        return false;
    };
    let Some(center_y) = provenance.lift_f64(center[1]) else {
        return false;
    };
    let Some(radius) = provenance.lift_f64(radius) else {
        return false;
    };

    let dx = &point_x - &center_x;
    let dy = &point_y - &center_y;
    let distance_squared = &(&dx * &dx) + &(&dy * &dy);
    let radius_squared = &radius * &radius;

    compare_reals_with_policy(&distance_squared, &radius_squared, PredicatePolicy::STRICT)
        .value()
        .is_some_and(|ordering| ordering != std::cmp::Ordering::Greater)
}

fn multipolygon_center(multipolygon: &geo::MultiPolygon<f64>) -> Option<[f64; 2]> {
    let bounds = multipolygon.bounding_rect()?;
    Some([
        (bounds.min().x + bounds.max().x) / 2.0,
        (bounds.min().y + bounds.max().y) / 2.0,
    ])
}

fn point_angle_degrees(previous: [f64; 2], current: [f64; 2], next: [f64; 2]) -> f64 {
    let ax = previous[0] - current[0];
    let ay = previous[1] - current[1];
    let bx = next[0] - current[0];
    let by = next[1] - current[1];
    let a_len = (ax * ax + ay * ay).sqrt();
    let b_len = (bx * bx + by * by).sqrt();
    if a_len == 0.0 || b_len == 0.0 {
        return 0.0;
    }

    let cos = ((ax * bx + ay * by) / (a_len * b_len)).clamp(-1.0, 1.0);
    cos.acos().to_degrees()
}

/// Run the `layer_names_csv` design-readiness check or report helper.
pub fn layer_names_csv(board: &BoardModel) -> String {
    let mut counts = HashMap::new();
    for feature in &board.copper {
        *counts
            .entry((feature.layer.clone(), feature.kind))
            .or_insert(0usize) += 1;
    }

    let mut layers = counts.into_iter().collect::<Vec<_>>();
    layers.sort_by(|left, right| left.0.cmp(&right.0));
    layers
        .into_iter()
        .map(|((layer, kind), count)| format!("{layer}:{}({count})", kind.as_str()))
        .collect::<Vec<_>>()
        .join(", ")
}

impl CopperKind {
    fn as_str(self) -> &'static str {
        match self {
            CopperKind::Pad => "pad",
            CopperKind::Via => "via",
            CopperKind::Segment => "segment",
            CopperKind::Zone => "zone",
        }
    }
}

#[cfg(test)]
mod tests {
    use std::time::Instant;

    use geo::{Coord, LineString, Polygon};

    use crate::geometry::{
        SourceGridFacts, SourceUnit, circle_polygon, line_polygon, polygons_to_sketch,
    };
    use crate::kicad::{BoardModel, CopperFeature, CopperKind, DrillFeature};
    use crate::{LayerMetadata, PcbSketch};

    use crate::checks::{
        annular_ring, annular_ring_tolerance, board_outline_drill_clearance,
        castellation_hole_readiness, castellation_intent, component_edge_clearance_readiness,
        component_hole_clearance_readiness, connector_rework_clearance_readiness,
        dense_pad_escape_readiness, drill_aspect_ratio, drill_spacing, drill_table_consistency,
        drill_to_copper_clearance, drills_to_sketch, esd_protection_readiness, fiducial_readiness,
        high_voltage_edge_readiness, hot_component_spacing_readiness, local_fiducial_readiness,
        mouse_bite_readiness, plating_intent, rf_keepout_readiness, rf_via_fence_readiness,
        routed_slot_readiness, sensitive_net_spacing_readiness, sensitive_return_readiness,
        switch_node_keepout_readiness, testpoint_accessibility_readiness,
        testpoint_coverage_readiness, thermal_copper_area_readiness,
        thermal_mechanical_keepout_readiness, thermal_pad_via_readiness, thermal_relief_readiness,
        thermal_via_readiness, tooling_hole_readiness, voltage_clearance_readiness,
    };
    use crate::ipc356::{Ipc356AccessSide, Ipc356FeatureType, Ipc356Point, Ipc356Soldermask};

    use super::{
        apply_ipc356_nets, board_edge_exposure, board_edge_exposure_with_grid,
        chassis_stitching_readiness, chassis_stitching_readiness_with_grid,
        connector_return_path_readiness, controlled_impedance_readiness, copper_net_intent,
        copper_width_readiness, decoupling_proximity_readiness, differential_pair_readiness,
        differential_pair_return_readiness, differential_pair_spacing_readiness,
        differential_pair_via_symmetry_readiness, edge_copper_pullback_readiness,
        edge_copper_pullback_readiness_with_grid, edge_stitching_readiness,
        edge_stitching_readiness_with_grid, gold_finger_drill_keepout_readiness,
        gold_finger_edge_readiness, gold_finger_readiness, gold_finger_spacing_readiness,
        high_current_neck_readiness, high_current_readiness, high_speed_edge_readiness,
        high_speed_edge_readiness_with_grid, ipc356_coverage, ipc356_drill_diameter, net_spacing,
        orphaned_zone_readiness, panelization_clearance, plane_clearance_readiness,
        power_plane_readiness, power_via_array_readiness, reference_plane_readiness,
        reference_plane_void_readiness, registration_tolerance, return_path_readiness,
        same_net_island_readiness, teardrop_readiness, trace_junction_acid_trap_readiness,
        via_in_pad_readiness,
    };

    #[test]
    fn annular_ring_flags_small_pad() {
        let board = BoardModel {
            source: "test".to_string(),
            copper: vec![CopperFeature {
                layer: "F.Cu".to_string(),
                net: Some("GND".to_string()),
                kind: CopperKind::Pad,
                location: [0.0, 0.0],
                sketch: polygons_to_sketch(
                    vec![circle_polygon([0.0, 0.0], 0.4, 32)],
                    Some(LayerMetadata {
                        name: "pad".to_string(),
                    }),
                ),
            }],
            drills: vec![DrillFeature {
                location: [0.0, 0.0],
                diameter: 0.7,
                net: Some("GND".to_string()),
                plated: true,
            }],
            board_outline: None,
            panel_features: None,
        };

        assert_eq!(annular_ring(&board, 0.1, &[]).len(), 1);
    }

    #[test]
    fn annular_ring_allows_via_at_minimum_ring() {
        let board = BoardModel {
            source: "test".to_string(),
            copper: vec![CopperFeature {
                layer: "F.Cu".to_string(),
                net: Some("GND".to_string()),
                kind: CopperKind::Via,
                location: [0.0, 0.0],
                sketch: polygons_to_sketch(
                    vec![circle_polygon([0.0, 0.0], 0.5, 64)],
                    Some(LayerMetadata {
                        name: "via".to_string(),
                    }),
                ),
            }],
            drills: vec![DrillFeature {
                location: [0.0, 0.0],
                diameter: 0.6,
                net: Some("GND".to_string()),
                plated: true,
            }],
            board_outline: None,
            panel_features: None,
        };

        assert!(annular_ring(&board, 0.1, &[]).is_empty());
    }

    #[test]
    fn annular_ring_tolerance_reports_nominal_pass_worst_case_failure() {
        let board = BoardModel {
            source: "test".to_string(),
            copper: vec![copper_disc("GND", CopperKind::Via, [0.0, 0.0], 0.5)],
            drills: vec![DrillFeature {
                location: [0.0, 0.0],
                diameter: 0.7,
                net: Some("GND".to_string()),
                plated: true,
            }],
            board_outline: None,
            panel_features: None,
        };

        let violations = annular_ring_tolerance(&board, 0.14, 0.02, &[]);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "annular-ring-tolerance");
    }

    #[test]
    fn annular_ring_tolerance_allows_compliant_or_already_nominal_failures() {
        let compliant = BoardModel {
            source: "test".to_string(),
            copper: vec![copper_disc("GND", CopperKind::Via, [0.0, 0.0], 0.5)],
            drills: vec![DrillFeature {
                location: [0.0, 0.0],
                diameter: 0.7,
                net: Some("GND".to_string()),
                plated: true,
            }],
            board_outline: None,
            panel_features: None,
        };
        let nominal_failure = BoardModel {
            source: "test".to_string(),
            copper: vec![copper_disc("GND", CopperKind::Via, [0.0, 0.0], 0.4)],
            drills: vec![DrillFeature {
                location: [0.0, 0.0],
                diameter: 0.7,
                net: Some("GND".to_string()),
                plated: true,
            }],
            board_outline: None,
            panel_features: None,
        };

        assert!(annular_ring_tolerance(&compliant, 0.12, 0.02, &[]).is_empty());
        assert!(annular_ring_tolerance(&nominal_failure, 0.14, 0.02, &[]).is_empty());
    }

    #[test]
    fn annular_ring_tolerance_respects_selected_layers() {
        let board = BoardModel {
            source: "test".to_string(),
            copper: vec![copper_disc_on_layer(
                "GND",
                CopperKind::Via,
                "B.Cu",
                [0.0, 0.0],
                0.5,
            )],
            drills: vec![DrillFeature {
                location: [0.0, 0.0],
                diameter: 0.7,
                net: Some("GND".to_string()),
                plated: true,
            }],
            board_outline: None,
            panel_features: None,
        };

        assert!(annular_ring_tolerance(&board, 0.14, 0.02, &["F.Cu".to_string()]).is_empty());
        assert_eq!(
            annular_ring_tolerance(&board, 0.14, 0.02, &["B.Cu".to_string()]).len(),
            1
        );
    }

    #[test]
    fn plating_intent_reports_npth_with_nearby_copper() {
        let board = BoardModel {
            source: "test".to_string(),
            copper: vec![copper_disc("GND", CopperKind::Pad, [0.0, 0.0], 0.4)],
            drills: vec![DrillFeature {
                location: [0.0, 0.0],
                diameter: 0.6,
                net: None,
                plated: false,
            }],
            board_outline: None,
            panel_features: None,
        };

        let violations = plating_intent(&board, &[], 0.05);

        assert_eq!(violations.len(), 1);
        assert!(
            violations[0]
                .message
                .as_deref()
                .unwrap()
                .contains("non-plated")
        );
    }

    #[test]
    fn plating_intent_reports_plated_drill_without_pad_or_via_copper() {
        let board = BoardModel {
            source: "test".to_string(),
            copper: vec![copper_line(
                "GND",
                CopperKind::Segment,
                [0.0, 0.0],
                [1.0, 0.0],
                0.1,
            )],
            drills: vec![DrillFeature {
                location: [0.5, 0.0],
                diameter: 0.3,
                net: Some("GND".to_string()),
                plated: true,
            }],
            board_outline: None,
            panel_features: None,
        };

        let violations = plating_intent(&board, &[], 0.05);

        assert_eq!(violations.len(), 1);
        assert!(
            violations[0]
                .message
                .as_deref()
                .unwrap()
                .contains("plated drill")
        );
    }

    #[test]
    fn plating_intent_allows_plated_drill_with_same_net_pad() {
        let board = BoardModel {
            source: "test".to_string(),
            copper: vec![copper_disc("GND", CopperKind::Pad, [0.0, 0.0], 0.4)],
            drills: vec![DrillFeature {
                location: [0.01, 0.0],
                diameter: 0.3,
                net: Some("GND".to_string()),
                plated: true,
            }],
            board_outline: None,
            panel_features: None,
        };

        assert!(plating_intent(&board, &[], 0.05).is_empty());
    }

    #[test]
    fn plating_intent_culls_sparse_copper_fields() {
        let mut copper = (0..2_000)
            .map(|index| {
                copper_disc(
                    &format!("SIG{index}"),
                    CopperKind::Pad,
                    [100.0 + index as f64 * 2.0, 100.0],
                    0.4,
                )
            })
            .collect::<Vec<_>>();
        copper.push(copper_disc("GND", CopperKind::Pad, [0.01, 0.0], 0.4));
        copper.push(copper_disc("SIG_NEAR", CopperKind::Pad, [0.0, 2.0], 0.4));
        let board = BoardModel {
            source: "test".to_string(),
            copper,
            drills: vec![
                DrillFeature {
                    location: [0.0, 0.0],
                    diameter: 0.3,
                    net: Some("GND".to_string()),
                    plated: true,
                },
                DrillFeature {
                    location: [0.0, 2.0],
                    diameter: 0.6,
                    net: None,
                    plated: false,
                },
            ],
            board_outline: None,
            panel_features: None,
        };

        let started = std::time::Instant::now();
        let violations = plating_intent(&board, &[], 0.05);

        assert_eq!(violations.len(), 1);
        assert!(
            violations[0]
                .message
                .as_deref()
                .unwrap()
                .contains("non-plated")
        );
        assert!(
            started.elapsed() < std::time::Duration::from_secs(2),
            "plating intent should cull sparse copper fields before exact center checks"
        );
    }

    #[test]
    fn routed_slot_readiness_reports_small_npth_mechanical_drills() {
        let board = BoardModel {
            source: "test".to_string(),
            copper: Vec::new(),
            drills: vec![DrillFeature {
                location: [1.0, 2.0],
                diameter: 0.18,
                net: None,
                plated: false,
            }],
            board_outline: None,
            panel_features: None,
        };

        let violations = routed_slot_readiness(&board, 0.25);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "routed-slot-readiness");
        assert_eq!(violations[0].locations, vec![[1.0, 2.0]]);
    }

    #[test]
    fn routed_slot_readiness_allows_plated_zero_or_large_drills() {
        let board = BoardModel {
            source: "test".to_string(),
            copper: Vec::new(),
            drills: vec![
                DrillFeature {
                    location: [0.0, 0.0],
                    diameter: 0.18,
                    net: Some("GND".to_string()),
                    plated: true,
                },
                DrillFeature {
                    location: [1.0, 0.0],
                    diameter: 0.0,
                    net: None,
                    plated: false,
                },
                DrillFeature {
                    location: [2.0, 0.0],
                    diameter: 0.35,
                    net: None,
                    plated: false,
                },
            ],
            board_outline: None,
            panel_features: None,
        };

        assert!(routed_slot_readiness(&board, 0.25).is_empty());
    }

    #[test]
    fn drill_aspect_ratio_flags_small_holes_for_board_thickness() {
        let drills = vec![
            DrillFeature {
                location: [0.0, 0.0],
                diameter: 0.15,
                net: None,
                plated: true,
            },
            DrillFeature {
                location: [1.0, 0.0],
                diameter: 0.4,
                net: None,
                plated: true,
            },
        ];

        let violations = drill_aspect_ratio("drills", &drills, 1.6, 10.0);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].locations, vec![[0.0, 0.0]]);
    }

    #[test]
    fn drill_aspect_ratio_reports_zero_diameter_without_dividing() {
        let drills = vec![DrillFeature {
            location: [0.0, 0.0],
            diameter: 0.0,
            net: None,
            plated: true,
        }];

        let violations = drill_aspect_ratio("drills", &drills, 1.6, 10.0);

        assert_eq!(violations.len(), 1);
        assert!(
            violations[0]
                .message
                .as_deref()
                .unwrap()
                .contains("undefined")
        );
    }

    #[test]
    fn drill_table_consistency_reports_kicad_excellon_diameter_conflicts() {
        let board_drills = vec![DrillFeature {
            location: [0.0, 0.0],
            diameter: 0.30,
            net: Some("GND".to_string()),
            plated: true,
        }];
        let excellon_drills = vec![DrillFeature {
            location: [0.01, 0.0],
            diameter: 0.45,
            net: None,
            plated: true,
        }];

        let violations = drill_table_consistency(&board_drills, &excellon_drills, &[], 0.05);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "drill-table-consistency");
    }

    #[test]
    fn drill_table_consistency_reports_excellon_ipc356_diameter_conflicts() {
        let excellon_drills = vec![DrillFeature {
            location: [1.0, 0.0],
            diameter: 0.30,
            net: None,
            plated: true,
        }];
        let points = vec![Ipc356Point {
            net: "GND".to_string(),
            reference: Some("V1".to_string()),
            pin: None,
            location: [1.01, 0.0],
            diameter: Some(0.50),
            access_side: None,
            feature_type: None,
            soldermask: None,
        }];

        let violations = drill_table_consistency(&[], &excellon_drills, &points, 0.05);

        assert_eq!(violations.len(), 1);
        assert_eq!(
            violations[0].layers,
            vec![
                "Excellon drills".to_string(),
                "IPC-D-356 drills".to_string()
            ]
        );
    }

    #[test]
    fn drill_table_consistency_allows_matching_or_unmatched_records() {
        let board_drills = vec![DrillFeature {
            location: [0.0, 0.0],
            diameter: 0.30,
            net: Some("GND".to_string()),
            plated: true,
        }];
        let excellon_drills = vec![
            DrillFeature {
                location: [0.01, 0.0],
                diameter: 0.31,
                net: None,
                plated: true,
            },
            DrillFeature {
                location: [10.0, 0.0],
                diameter: 0.90,
                net: None,
                plated: true,
            },
        ];

        assert!(drill_table_consistency(&board_drills, &excellon_drills, &[], 0.05).is_empty());
    }

    #[test]
    fn copper_width_readiness_reports_narrow_kicad_feature() {
        let board = board_with_copper(vec![copper_line(
            "SIG",
            CopperKind::Segment,
            [0.0, 0.0],
            [1.0, 0.0],
            0.08,
        )]);

        let violations = copper_width_readiness(&board, &[], 0.12);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "copper-width-readiness");
    }

    #[test]
    fn copper_width_readiness_allows_wide_or_degenerate_features() {
        let mut degenerate = copper_disc("SIG", CopperKind::Zone, [2.0, 0.0], 0.0);
        degenerate.sketch = polygons_to_sketch(Vec::new(), None);
        let board = board_with_copper(vec![
            copper_line("SIG", CopperKind::Segment, [0.0, 0.0], [1.0, 0.0], 0.16),
            degenerate,
        ]);

        assert!(copper_width_readiness(&board, &[], 0.12).is_empty());
    }

    #[test]
    fn copper_width_readiness_respects_selected_layers() {
        let board = board_with_copper(vec![copper_line_on_layer(
            "SIG",
            CopperKind::Segment,
            "B.Cu",
            [0.0, 0.0],
            [1.0, 0.0],
            0.08,
        )]);

        assert!(copper_width_readiness(&board, &["F.Cu".to_string()], 0.12).is_empty());
        assert_eq!(
            copper_width_readiness(&board, &["B.Cu".to_string()], 0.12).len(),
            1
        );
    }

    #[test]
    fn copper_width_readiness_handles_large_sparse_feature_sets() {
        let copper = (0..2_000)
            .map(|index| {
                let x = index as f64 * 2.0;
                copper_line("SIG", CopperKind::Segment, [x, 0.0], [x + 1.0, 0.0], 0.16)
            })
            .collect::<Vec<_>>();
        let board = board_with_copper(copper);

        let started = std::time::Instant::now();
        let violations = copper_width_readiness(&board, &[], 0.12);

        assert!(violations.is_empty());
        assert!(
            started.elapsed() < std::time::Duration::from_secs(2),
            "copper width readiness should stay linear over sparse feature sets"
        );
    }

    #[test]
    fn copper_net_intent_reports_unnetted_kicad_copper() {
        let mut unnetted = copper_disc("GND", CopperKind::Zone, [0.0, 0.0], 0.5);
        unnetted.net = None;
        let board = board_with_copper(vec![
            copper_disc("GND", CopperKind::Pad, [1.0, 0.0], 0.5),
            unnetted,
        ]);

        let violations = copper_net_intent(&board, &[]);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "copper-net-intent");
        assert_eq!(violations[0].locations, vec![[0.0, 0.0]]);
    }

    #[test]
    fn copper_net_intent_respects_selected_layers() {
        let mut unnetted_front =
            copper_disc_on_layer("GND", CopperKind::Zone, "F.Cu", [0.0, 0.0], 0.5);
        unnetted_front.net = None;
        let mut unnetted_back =
            copper_disc_on_layer("GND", CopperKind::Zone, "B.Cu", [1.0, 0.0], 0.5);
        unnetted_back.net = None;
        let board = board_with_copper(vec![unnetted_front, unnetted_back]);

        let violations = copper_net_intent(&board, &["B.Cu".to_string()]);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].layers, vec!["B.Cu".to_string()]);
    }

    #[test]
    fn copper_net_intent_handles_large_sparse_feature_sets() {
        let mut copper = (0..2_000)
            .map(|index| {
                let x = index as f64 * 2.0;
                copper_line("SIG", CopperKind::Segment, [x, 0.0], [x + 1.0, 0.0], 0.16)
            })
            .collect::<Vec<_>>();
        copper.push(unnetted_copper_disc_on_layer("F.Cu", [0.0, 2.0], 0.20));
        let board = board_with_copper(copper);

        let started = std::time::Instant::now();
        let violations = copper_net_intent(&board, &[]);

        assert_eq!(violations.len(), 1);
        assert!(
            started.elapsed() < std::time::Duration::from_secs(2),
            "copper net-intent readiness should stay linear over sparse feature sets"
        );
    }

    #[test]
    fn net_spacing_flags_close_different_nets() {
        let board = BoardModel {
            source: "test".to_string(),
            copper: vec![feature("A", [0.0, 0.0]), feature("B", [0.9, 0.0])],
            drills: Vec::new(),
            board_outline: None,
            panel_features: None,
        };

        assert_eq!(net_spacing(&board, 0.2, &[], 1.0e-9).len(), 1);
    }

    #[test]
    fn net_spacing_covers_pad_via_and_via_spacing() {
        let selected_layers = vec!["F.Cu".to_string()];
        let pad = copper_disc("PAD", CopperKind::Pad, [0.0, 0.0], 0.12);
        let via_a = copper_disc("VIA_A", CopperKind::Via, [0.28, 0.0], 0.12);
        let via_b = copper_disc("VIA_B", CopperKind::Via, [0.56, 0.0], 0.12);
        let board = BoardModel {
            source: "test".to_string(),
            copper: vec![pad, via_a, via_b],
            drills: Vec::new(),
            board_outline: None,
            panel_features: None,
        };

        let violations = net_spacing(&board, 0.20, &selected_layers, 1.0e-9);

        assert_eq!(violations.len(), 2);
    }

    #[test]
    fn net_spacing_culls_large_sparse_feature_sets() {
        let mut copper = Vec::new();
        for index in 0..600 {
            copper.push(feature(
                &format!("N{index}"),
                [(index % 30) as f64 * 5.0, (index / 30) as f64 * 5.0],
            ));
        }
        copper.push(feature("NEAR", [0.6, 0.0]));
        let board = board_with_copper(copper);

        let start = std::time::Instant::now();
        let violations = net_spacing(&board, 0.2, &[], 1.0e-9);

        assert_eq!(violations.len(), 1);
        assert!(
            start.elapsed() < std::time::Duration::from_secs(2),
            "large sparse net-spacing check should stay in the broad phase"
        );
    }

    #[test]
    fn via_in_pad_readiness_reports_same_net_overlap() {
        let board = BoardModel {
            source: "test".to_string(),
            copper: vec![
                copper_disc("GND", CopperKind::Pad, [0.0, 0.0], 0.30),
                copper_disc("GND", CopperKind::Via, [0.05, 0.0], 0.12),
            ],
            drills: Vec::new(),
            board_outline: None,
            panel_features: None,
        };

        let violations = via_in_pad_readiness(&board, &[], 1.0e-9);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "via-in-pad-readiness");
    }

    #[test]
    fn via_in_pad_readiness_allows_distant_or_different_net_vias() {
        let board = BoardModel {
            source: "test".to_string(),
            copper: vec![
                copper_disc("GND", CopperKind::Pad, [0.0, 0.0], 0.30),
                copper_disc("GND", CopperKind::Via, [1.0, 0.0], 0.12),
                copper_disc("SIG", CopperKind::Via, [0.0, 0.0], 0.12),
            ],
            drills: Vec::new(),
            board_outline: None,
            panel_features: None,
        };

        assert!(via_in_pad_readiness(&board, &[], 1.0e-9).is_empty());
    }

    #[test]
    fn via_in_pad_readiness_respects_selected_layers() {
        let board = BoardModel {
            source: "test".to_string(),
            copper: vec![
                copper_disc_on_layer("GND", CopperKind::Pad, "B.Cu", [0.0, 0.0], 0.30),
                copper_disc_on_layer("GND", CopperKind::Via, "B.Cu", [0.0, 0.0], 0.12),
            ],
            drills: Vec::new(),
            board_outline: None,
            panel_features: None,
        };

        assert!(via_in_pad_readiness(&board, &["F.Cu".to_string()], 1.0e-9).is_empty());
        assert_eq!(
            via_in_pad_readiness(&board, &["B.Cu".to_string()], 1.0e-9).len(),
            1
        );
    }

    #[test]
    fn via_in_pad_readiness_culls_sparse_pad_fields() {
        let mut copper = (0..2_000)
            .map(|index| {
                copper_disc(
                    &format!("GND_{index}"),
                    CopperKind::Pad,
                    [100.0 + index as f64 * 4.0, 100.0],
                    0.30,
                )
            })
            .collect::<Vec<_>>();
        copper.push(copper_disc("GND", CopperKind::Pad, [0.0, 0.0], 0.30));
        copper.push(copper_disc("GND", CopperKind::Via, [0.05, 0.0], 0.12));
        let board = board_with_copper(copper);

        let started = std::time::Instant::now();
        let violations = via_in_pad_readiness(&board, &[], 1.0e-9);

        assert_eq!(violations.len(), 1);
        assert!(
            started.elapsed() < std::time::Duration::from_secs(2),
            "via-in-pad should spatially cull distant pads before exact overlap review"
        );
    }

    #[test]
    fn teardrop_readiness_reports_narrow_segment_into_pad() {
        let board = board_with_copper(vec![
            copper_disc("SIG", CopperKind::Pad, [0.0, 0.0], 0.25),
            copper_line("SIG", CopperKind::Segment, [0.0, 0.0], [1.0, 0.0], 0.08),
        ]);

        let violations = teardrop_readiness(&board, &[], 0.12, 1.0e-9);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "teardrop-readiness");
    }

    #[test]
    fn teardrop_readiness_allows_wide_or_different_net_segment_entries() {
        let board = board_with_copper(vec![
            copper_disc("SIG", CopperKind::Pad, [0.0, 0.0], 0.25),
            copper_line("SIG", CopperKind::Segment, [0.0, 0.0], [1.0, 0.0], 0.16),
            copper_line("OTHER", CopperKind::Segment, [0.0, 0.0], [0.0, 1.0], 0.08),
        ]);

        assert!(teardrop_readiness(&board, &[], 0.12, 1.0e-9).is_empty());
    }

    #[test]
    fn teardrop_readiness_respects_selected_layers() {
        let board = board_with_copper(vec![
            copper_disc_on_layer("SIG", CopperKind::Pad, "B.Cu", [0.0, 0.0], 0.25),
            copper_line_on_layer(
                "SIG",
                CopperKind::Segment,
                "B.Cu",
                [0.0, 0.0],
                [1.0, 0.0],
                0.08,
            ),
        ]);

        assert!(teardrop_readiness(&board, &["F.Cu".to_string()], 0.12, 1.0e-9).is_empty());
        assert_eq!(
            teardrop_readiness(&board, &["B.Cu".to_string()], 0.12, 1.0e-9).len(),
            1
        );
    }

    #[test]
    fn teardrop_readiness_culls_sparse_anchor_fields() {
        let mut copper = (0..2_000)
            .map(|index| {
                copper_disc(
                    &format!("SIG_{index}"),
                    CopperKind::Pad,
                    [100.0 + index as f64 * 4.0, 100.0],
                    0.25,
                )
            })
            .collect::<Vec<_>>();
        copper.push(copper_disc("SIG", CopperKind::Pad, [0.0, 0.0], 0.25));
        copper.push(copper_line(
            "SIG",
            CopperKind::Segment,
            [0.0, 0.0],
            [1.0, 0.0],
            0.08,
        ));
        let board = board_with_copper(copper);

        let started = std::time::Instant::now();
        let violations = teardrop_readiness(&board, &[], 0.12, 1.0e-9);

        assert_eq!(violations.len(), 1);
        assert!(
            started.elapsed() < std::time::Duration::from_secs(2),
            "teardrop review should spatially cull distant anchors before exact overlap review"
        );
    }

    #[test]
    fn thermal_relief_readiness_reports_pad_embedded_in_same_net_zone() {
        let board = board_with_copper(vec![
            copper_disc("GND", CopperKind::Pad, [0.0, 0.0], 0.20),
            copper_rect("GND", CopperKind::Zone, "F.Cu", -1.0, -1.0, 1.0, 1.0),
        ]);

        let violations = thermal_relief_readiness(&board, &[], 1.0e-9);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "thermal-relief-readiness");
    }

    #[test]
    fn thermal_relief_readiness_allows_distant_or_different_net_zones() {
        let board = board_with_copper(vec![
            copper_disc("GND", CopperKind::Pad, [0.0, 0.0], 0.20),
            copper_rect("SIG", CopperKind::Zone, "F.Cu", -1.0, -1.0, 1.0, 1.0),
            copper_rect("GND", CopperKind::Zone, "F.Cu", 2.0, 2.0, 3.0, 3.0),
        ]);

        assert!(thermal_relief_readiness(&board, &[], 1.0e-9).is_empty());
    }

    #[test]
    fn thermal_relief_readiness_respects_selected_layers() {
        let board = board_with_copper(vec![
            copper_disc_on_layer("GND", CopperKind::Via, "B.Cu", [0.0, 0.0], 0.20),
            copper_rect("GND", CopperKind::Zone, "B.Cu", -1.0, -1.0, 1.0, 1.0),
        ]);

        assert!(thermal_relief_readiness(&board, &["F.Cu".to_string()], 1.0e-9).is_empty());
        assert_eq!(
            thermal_relief_readiness(&board, &["B.Cu".to_string()], 1.0e-9).len(),
            1
        );
    }

    #[test]
    fn plane_clearance_readiness_reports_npth_intersecting_zone() {
        let mut board = board_with_copper(vec![copper_rect(
            "GND",
            CopperKind::Zone,
            "F.Cu",
            -1.0,
            -1.0,
            1.0,
            1.0,
        )]);
        board.drills = vec![DrillFeature {
            location: [0.0, 0.0],
            diameter: 0.5,
            net: None,
            plated: false,
        }];

        let violations = plane_clearance_readiness(&board, &[], 1.0e-9);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "plane-clearance-readiness");
    }

    #[test]
    fn plane_clearance_readiness_allows_plated_or_distant_holes() {
        let mut board = board_with_copper(vec![copper_rect(
            "GND",
            CopperKind::Zone,
            "F.Cu",
            -1.0,
            -1.0,
            1.0,
            1.0,
        )]);
        board.drills = vec![
            DrillFeature {
                location: [0.0, 0.0],
                diameter: 0.5,
                net: Some("GND".to_string()),
                plated: true,
            },
            DrillFeature {
                location: [3.0, 0.0],
                diameter: 0.5,
                net: None,
                plated: false,
            },
        ];

        assert!(plane_clearance_readiness(&board, &[], 1.0e-9).is_empty());
    }

    #[test]
    fn plane_clearance_readiness_respects_selected_layers() {
        let mut board = board_with_copper(vec![copper_rect(
            "GND",
            CopperKind::Zone,
            "B.Cu",
            -1.0,
            -1.0,
            1.0,
            1.0,
        )]);
        board.drills = vec![DrillFeature {
            location: [0.0, 0.0],
            diameter: 0.5,
            net: None,
            plated: false,
        }];

        assert!(plane_clearance_readiness(&board, &["F.Cu".to_string()], 1.0e-9).is_empty());
        assert_eq!(
            plane_clearance_readiness(&board, &["B.Cu".to_string()], 1.0e-9).len(),
            1
        );
    }

    #[test]
    fn plane_clearance_readiness_culls_sparse_zone_and_drill_fields() {
        let mut board = board_with_copper(
            (0..2_000)
                .map(|index| {
                    let x = 100.0 + index as f64 * 3.0;
                    copper_rect("GND", CopperKind::Zone, "F.Cu", x, 100.0, x + 1.0, 101.0)
                })
                .collect(),
        );
        board.drills = (0..400)
            .map(|index| DrillFeature {
                location: [index as f64 * 3.0, 0.0],
                diameter: 0.5,
                net: None,
                plated: false,
            })
            .collect();
        let start = Instant::now();

        let violations = plane_clearance_readiness(&board, &[], 1.0e-9);

        assert!(violations.is_empty());
        assert!(
            start.elapsed().as_secs_f64() < 2.0,
            "plane clearance should index sparse zone/drill fields"
        );
    }

    #[test]
    fn board_edge_exposure_reports_copper_outside_outline() {
        let mut board = board_with_outline(square(0.0, 0.0, 10.0, 10.0));
        board.copper = vec![copper_disc("EDGE", CopperKind::Pad, [0.1, 5.0], 0.3)];

        let violations = board_edge_exposure(&board, &[], 1.0e-9);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "board-edge-exposure");
        assert_eq!(violations[0].locations, vec![[0.1, 5.0]]);
    }

    #[test]
    fn board_edge_exposure_accepts_retained_kicad_grid_for_rect_fast_path() {
        let mut board = board_with_outline(square(0.0, 0.0, 10.0, 10.0));
        board.copper = vec![copper_disc("SIG", CopperKind::Pad, [1.0, 5.0], 0.3)];
        let grid = SourceGridFacts::primitive_float_edge(SourceUnit::KiCadMillimeter);

        assert!(board_edge_exposure_with_grid(&board, &[], 1.0e-9, grid).is_empty());
    }

    #[test]
    fn board_edge_exposure_allows_inset_copper_or_missing_outline() {
        let mut board = board_with_outline(square(0.0, 0.0, 10.0, 10.0));
        board.copper = vec![copper_disc("SIG", CopperKind::Pad, [1.0, 5.0], 0.3)];

        assert!(board_edge_exposure(&board, &[], 1.0e-9).is_empty());
        assert!(board_edge_exposure(&board_with_copper(board.copper), &[], 1.0e-9).is_empty());
    }

    #[test]
    fn board_edge_exposure_respects_selected_layers() {
        let mut board = board_with_outline(square(0.0, 0.0, 10.0, 10.0));
        board.copper = vec![copper_disc_on_layer(
            "EDGE",
            CopperKind::Segment,
            "B.Cu",
            [0.1, 5.0],
            0.3,
        )];

        assert!(board_edge_exposure(&board, &["F.Cu".to_string()], 1.0e-9).is_empty());
        assert_eq!(
            board_edge_exposure(&board, &["B.Cu".to_string()], 1.0e-9).len(),
            1
        );
    }

    #[test]
    fn board_edge_exposure_culls_rectangular_interior_copper_fields() {
        let mut board = board_with_outline(square(0.0, 0.0, 100.0, 100.0));
        board.copper = (0..2_000)
            .map(|index| {
                copper_disc(
                    &format!("SIG{index}"),
                    CopperKind::Pad,
                    [
                        5.0 + (index % 50) as f64 * 1.5,
                        5.0 + (index / 50) as f64 * 1.5,
                    ],
                    0.30,
                )
            })
            .collect();
        board.copper.push(copper_disc(
            "EDGE",
            CopperKind::Segment,
            [99.95, 50.0],
            0.30,
        ));

        let started = Instant::now();
        let violations = board_edge_exposure(&board, &[], 1.0e-9);

        assert_eq!(violations.len(), 1);
        assert!(
            started.elapsed().as_secs_f64() < 2.0,
            "board-edge exposure should skip rectangular interior copper before exact difference"
        );
    }

    #[test]
    fn high_speed_edge_readiness_reports_edge_rate_copper_near_outline() {
        let mut board = board_with_outline(square(0.0, 0.0, 10.0, 10.0));
        board.copper = vec![copper_line(
            "USB_D+",
            CopperKind::Segment,
            [0.10, 1.0],
            [0.90, 1.0],
            0.10,
        )];

        let violations = high_speed_edge_readiness(&board, &[], 0.50, 1.0e-9);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "high-speed-edge-readiness");
    }

    #[test]
    fn high_speed_edge_readiness_accepts_retained_kicad_grid_for_edge_band() {
        let mut board = board_with_outline(square(0.0, 0.0, 10.0, 10.0));
        board.copper = vec![copper_line(
            "USB_D+",
            CopperKind::Segment,
            [2.0, 2.0],
            [3.0, 2.0],
            0.10,
        )];
        let grid = SourceGridFacts::primitive_float_edge(SourceUnit::KiCadMillimeter);

        assert!(high_speed_edge_readiness_with_grid(&board, &[], 0.50, 1.0e-9, grid).is_empty());
    }

    #[test]
    fn high_speed_edge_readiness_allows_inset_or_low_speed_copper() {
        let mut board = board_with_outline(square(0.0, 0.0, 10.0, 10.0));
        board.copper = vec![
            copper_line("USB_D+", CopperKind::Segment, [2.0, 2.0], [3.0, 2.0], 0.10),
            copper_line("GPIO1", CopperKind::Segment, [0.10, 1.0], [0.90, 1.0], 0.10),
        ];

        assert!(high_speed_edge_readiness(&board, &[], 0.50, 1.0e-9).is_empty());
    }

    #[test]
    fn high_speed_edge_readiness_respects_selected_layers() {
        let mut board = board_with_outline(square(0.0, 0.0, 10.0, 10.0));
        board.copper = vec![copper_line_on_layer(
            "PCIE_RX0",
            CopperKind::Segment,
            "B.Cu",
            [0.10, 1.0],
            [0.90, 1.0],
            0.10,
        )];

        assert!(high_speed_edge_readiness(&board, &["F.Cu".to_string()], 0.50, 1.0e-9).is_empty());
        assert_eq!(
            high_speed_edge_readiness(&board, &["B.Cu".to_string()], 0.50, 1.0e-9).len(),
            1
        );
    }

    #[test]
    fn high_speed_edge_readiness_culls_rectangular_interior_copper_fields() {
        let mut board = board_with_outline(square(0.0, 0.0, 100.0, 100.0));
        board.copper = (0..2_000)
            .map(|index| {
                copper_line(
                    &format!("USB_D{index}+"),
                    CopperKind::Segment,
                    [
                        5.0 + (index % 50) as f64 * 1.5,
                        5.0 + (index / 50) as f64 * 1.5,
                    ],
                    [
                        5.5 + (index % 50) as f64 * 1.5,
                        5.0 + (index / 50) as f64 * 1.5,
                    ],
                    0.10,
                )
            })
            .collect();
        board.copper.push(copper_line(
            "PCIE_RX0",
            CopperKind::Segment,
            [0.10, 50.0],
            [0.90, 50.0],
            0.10,
        ));

        let started = Instant::now();
        let violations = high_speed_edge_readiness(&board, &[], 0.50, 1.0e-9);

        assert_eq!(violations.len(), 1);
        assert!(
            started.elapsed().as_secs_f64() < 2.0,
            "high-speed edge review should skip rectangular interior copper before exact difference"
        );
    }

    #[test]
    fn high_voltage_edge_readiness_reports_hv_copper_near_outline() {
        let mut board = board_with_outline(square(0.0, 0.0, 10.0, 10.0));
        board.copper = vec![copper_line(
            "HV_400V",
            CopperKind::Segment,
            [0.20, 1.0],
            [0.90, 1.0],
            0.10,
        )];

        let violations = high_voltage_edge_readiness(&board, &[], 0.80, 1.0e-9);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "high-voltage-edge-readiness");
    }

    #[test]
    fn high_voltage_edge_readiness_allows_inset_or_low_voltage_copper() {
        let mut board = board_with_outline(square(0.0, 0.0, 10.0, 10.0));
        board.copper = vec![
            copper_line("HV_400V", CopperKind::Segment, [2.0, 2.0], [3.0, 2.0], 0.10),
            copper_line("GPIO1", CopperKind::Segment, [0.2, 1.0], [0.9, 1.0], 0.10),
        ];

        assert!(high_voltage_edge_readiness(&board, &[], 0.80, 1.0e-9).is_empty());
    }

    #[test]
    fn high_voltage_edge_readiness_respects_selected_layers() {
        let mut board = board_with_outline(square(0.0, 0.0, 10.0, 10.0));
        board.copper = vec![copper_line_on_layer(
            "MAINS_L",
            CopperKind::Segment,
            "B.Cu",
            [0.20, 1.0],
            [0.90, 1.0],
            0.10,
        )];

        assert!(
            high_voltage_edge_readiness(&board, &["F.Cu".to_string()], 0.80, 1.0e-9).is_empty()
        );
        assert_eq!(
            high_voltage_edge_readiness(&board, &["B.Cu".to_string()], 0.80, 1.0e-9).len(),
            1
        );
    }

    #[test]
    fn edge_copper_pullback_readiness_flags_non_edge_nets_near_outline() {
        let mut board = board_with_outline(square(0.0, 0.0, 10.0, 10.0));
        board.copper = vec![copper_line(
            "SIG1",
            CopperKind::Segment,
            [0.10, 1.0],
            [0.90, 1.0],
            0.10,
        )];

        let violations = edge_copper_pullback_readiness(&board, &[], 0.50, 1.0e-9);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "edge-copper-pullback-readiness");
    }

    #[test]
    fn edge_copper_pullback_readiness_accepts_retained_kicad_grid_for_boundary_distance() {
        let mut board = board_with_outline(square(0.0, 0.0, 10.0, 10.0));
        board.copper = vec![copper_line(
            "SIG1",
            CopperKind::Segment,
            [0.10, 1.0],
            [0.90, 1.0],
            0.10,
        )];
        let grid = SourceGridFacts::primitive_float_edge(SourceUnit::KiCadMillimeter);

        let violations = edge_copper_pullback_readiness_with_grid(&board, &[], 0.50, 1.0e-9, grid);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "edge-copper-pullback-readiness");
    }

    #[test]
    fn edge_copper_pullback_readiness_skips_edge_intent_nets() {
        let mut board = board_with_outline(square(0.0, 0.0, 10.0, 10.0));
        board.copper = vec![
            copper_line(
                "USB_D+",
                CopperKind::Segment,
                [0.10, 1.0],
                [0.90, 1.0],
                0.10,
            ),
            copper_line(
                "GOLD_EDGE_1",
                CopperKind::Segment,
                [0.10, 1.5],
                [0.90, 1.5],
                0.10,
            ),
            copper_line(
                "CHASSIS",
                CopperKind::Segment,
                [0.10, 2.0],
                [0.90, 2.0],
                0.10,
            ),
        ];

        assert!(edge_copper_pullback_readiness(&board, &[], 0.50, 1.0e-9).is_empty());
    }

    #[test]
    fn edge_copper_pullback_readiness_respects_selected_layers() {
        let mut board = board_with_outline(square(0.0, 0.0, 10.0, 10.0));
        board.copper = vec![
            copper_line_on_layer(
                "SIG1",
                CopperKind::Segment,
                "B.Cu",
                [2.0, 2.0],
                [3.0, 2.0],
                0.10,
            ),
            copper_line_on_layer(
                "SIG1",
                CopperKind::Segment,
                "F.Cu",
                [0.10, 0.1],
                [0.90, 0.1],
                0.10,
            ),
        ];

        assert_eq!(
            edge_copper_pullback_readiness(&board, &["F.Cu".to_string()], 0.50, 1.0e-9).len(),
            1
        );
        assert!(
            edge_copper_pullback_readiness(&board, &["B.Cu".to_string()], 0.50, 1.0e-9).is_empty()
        );
    }

    #[test]
    fn edge_stitching_readiness_flags_high_speed_nets_near_edge_without_stitch() {
        let mut board = board_with_outline(square(0.0, 0.0, 10.0, 10.0));
        board.copper = vec![
            copper_line(
                "USB_D+",
                CopperKind::Segment,
                [0.10, 1.0],
                [0.90, 1.0],
                0.10,
            ),
            copper_disc("VDD", CopperKind::Via, [0.30, 1.0], 0.10),
        ];

        let violations = edge_stitching_readiness(&board, &[], 0.50, 0.30, 1.0e-9);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "edge-stitching-readiness");
    }

    #[test]
    fn edge_stitching_readiness_accepts_retained_kicad_grid_for_edge_checks() {
        let mut board = board_with_outline(square(0.0, 0.0, 10.0, 10.0));
        board.copper = vec![
            copper_line(
                "PCIe_TX_P",
                CopperKind::Segment,
                [0.10, 1.0],
                [0.90, 1.0],
                0.10,
            ),
            copper_disc("VDD", CopperKind::Via, [0.30, 1.0], 0.10),
        ];
        let grid = SourceGridFacts::primitive_float_edge(SourceUnit::KiCadMillimeter);

        let violations = edge_stitching_readiness_with_grid(&board, &[], 0.50, 0.30, 1.0e-9, grid);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "edge-stitching-readiness");
    }

    #[test]
    fn edge_stitching_readiness_skips_non_high_speed_or_rf_nets() {
        let mut board = board_with_outline(square(0.0, 0.0, 10.0, 10.0));
        board.copper = vec![
            copper_line("GPIO1", CopperKind::Segment, [0.10, 1.0], [0.90, 1.0], 0.10),
            copper_disc("GND", CopperKind::Via, [0.30, 1.0], 0.10),
        ];

        assert!(edge_stitching_readiness(&board, &[], 0.50, 0.30, 1.0e-9).is_empty());
    }

    #[test]
    fn edge_stitching_readiness_respects_selected_layers() {
        let mut board = board_with_outline(square(0.0, 0.0, 10.0, 10.0));
        board.copper = vec![
            copper_line_on_layer(
                "USB_D+",
                CopperKind::Segment,
                "B.Cu",
                [0.10, 1.0],
                [0.90, 1.0],
                0.10,
            ),
            copper_disc_on_layer("GND", CopperKind::Via, "F.Cu", [0.30, 1.0], 0.10),
        ];

        assert!(
            edge_stitching_readiness(&board, &["F.Cu".to_string()], 0.50, 0.30, 1.0e-9).is_empty()
        );
        assert_eq!(
            edge_stitching_readiness(&board, &["B.Cu".to_string()], 0.50, 0.30, 1.0e-9).len(),
            1
        );
    }

    #[test]
    fn edge_stitching_readiness_allows_nearby_ground_stitching_via() {
        let mut board = board_with_outline(square(0.0, 0.0, 10.0, 10.0));
        board.copper = vec![
            copper_line(
                "WIFI_ANT",
                CopperKind::Segment,
                [0.10, 1.0],
                [0.90, 1.0],
                0.10,
            ),
            copper_disc("GND", CopperKind::Via, [0.30, 1.0], 0.10),
            copper_line("VDD", CopperKind::Segment, [2.0, 1.0], [3.0, 1.0], 0.10),
        ];

        assert!(edge_stitching_readiness(&board, &[], 0.50, 0.30, 1.0e-9).is_empty());
    }

    #[test]
    fn edge_stitching_readiness_culls_sparse_ground_vias() {
        let mut board = board_with_outline(square(0.0, 0.0, 10.0, 10.0));
        board.copper = (0..2_000)
            .map(|index| {
                copper_disc(
                    "GND",
                    CopperKind::Via,
                    [100.0 + index as f64 * 2.0, 50.0],
                    0.10,
                )
            })
            .chain([copper_line(
                "USB_D+",
                CopperKind::Segment,
                [0.10, 1.0],
                [0.90, 1.0],
                0.10,
            )])
            .collect();

        let start = Instant::now();
        let violations = edge_stitching_readiness(&board, &[], 0.50, 0.30, 1.0e-9);

        assert_eq!(violations.len(), 1);
        assert!(
            start.elapsed().as_secs_f64() < 2.0,
            "edge stitching should index sparse ground-via fields"
        );
    }

    #[test]
    fn controlled_impedance_readiness_reports_high_speed_net_layer_change_without_via() {
        let board = board_with_copper(vec![
            copper_line_on_layer(
                "USB_D+",
                CopperKind::Segment,
                "F.Cu",
                [0.0, 0.0],
                [1.0, 0.0],
                0.12,
            ),
            copper_line_on_layer(
                "USB_D+",
                CopperKind::Segment,
                "B.Cu",
                [2.0, 0.0],
                [3.0, 0.0],
                0.12,
            ),
        ]);

        let violations = controlled_impedance_readiness(&board, &[]);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "controlled-impedance-readiness");
        assert_eq!(violations[0].layers, vec!["B.Cu", "F.Cu"]);
    }

    #[test]
    fn controlled_impedance_readiness_allows_low_speed_or_via_transitioned_nets() {
        let board = board_with_copper(vec![
            copper_line_on_layer(
                "GPIO1",
                CopperKind::Segment,
                "F.Cu",
                [0.0, 0.0],
                [1.0, 0.0],
                0.12,
            ),
            copper_line_on_layer(
                "GPIO1",
                CopperKind::Segment,
                "B.Cu",
                [2.0, 0.0],
                [3.0, 0.0],
                0.12,
            ),
            copper_line_on_layer(
                "CLK_OUT",
                CopperKind::Segment,
                "F.Cu",
                [0.0, 1.0],
                [1.0, 1.0],
                0.12,
            ),
            copper_line_on_layer(
                "CLK_OUT",
                CopperKind::Segment,
                "B.Cu",
                [2.0, 1.0],
                [3.0, 1.0],
                0.12,
            ),
            copper_disc("CLK_OUT", CopperKind::Via, [1.5, 1.0], 0.15),
        ]);

        assert!(controlled_impedance_readiness(&board, &[]).is_empty());
    }

    #[test]
    fn controlled_impedance_readiness_respects_selected_layers() {
        let board = board_with_copper(vec![
            copper_line_on_layer(
                "PCIE_TX0",
                CopperKind::Segment,
                "F.Cu",
                [0.0, 0.0],
                [1.0, 0.0],
                0.12,
            ),
            copper_line_on_layer(
                "PCIE_TX0",
                CopperKind::Segment,
                "B.Cu",
                [2.0, 0.0],
                [3.0, 0.0],
                0.12,
            ),
        ]);

        assert!(controlled_impedance_readiness(&board, &["F.Cu".to_string()]).is_empty());
        assert_eq!(
            controlled_impedance_readiness(&board, &["F.Cu".to_string(), "B.Cu".to_string()]).len(),
            1
        );
    }

    #[test]
    fn differential_pair_readiness_reports_missing_mate() {
        let board = board_with_copper(vec![copper_line(
            "USB_D+",
            CopperKind::Segment,
            [0.0, 0.0],
            [1.0, 0.0],
            0.12,
        )]);

        let violations = differential_pair_readiness(&board, &[]);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "differential-pair-readiness");
        assert!(
            violations[0]
                .message
                .as_deref()
                .is_some_and(|message| message.contains("missing its negative side"))
        );
    }

    #[test]
    fn differential_pair_readiness_reports_layer_mismatch() {
        let board = board_with_copper(vec![
            copper_line_on_layer(
                "PCIE_TX0_P",
                CopperKind::Segment,
                "F.Cu",
                [0.0, 0.0],
                [1.0, 0.0],
                0.12,
            ),
            copper_line_on_layer(
                "PCIE_TX0_N",
                CopperKind::Segment,
                "B.Cu",
                [0.0, 0.2],
                [1.0, 0.2],
                0.12,
            ),
        ]);

        let violations = differential_pair_readiness(&board, &[]);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "differential-pair-readiness");
        assert_eq!(violations[0].layers, vec!["B.Cu", "F.Cu"]);
    }

    #[test]
    fn differential_pair_readiness_allows_matched_pair_on_same_layer() {
        let board = board_with_copper(vec![
            copper_line("USB_D+", CopperKind::Segment, [0.0, 0.0], [1.0, 0.0], 0.12),
            copper_line("USB_D-", CopperKind::Segment, [0.0, 0.2], [1.0, 0.2], 0.12),
        ]);

        assert!(differential_pair_readiness(&board, &[]).is_empty());
    }

    #[test]
    fn differential_pair_readiness_respects_selected_layers() {
        let board = board_with_copper(vec![copper_line_on_layer(
            "LVDS_CLK_P",
            CopperKind::Segment,
            "B.Cu",
            [0.0, 0.0],
            [1.0, 0.0],
            0.12,
        )]);

        assert!(differential_pair_readiness(&board, &["F.Cu".to_string()]).is_empty());
        assert_eq!(
            differential_pair_readiness(&board, &["B.Cu".to_string()]).len(),
            1
        );
    }

    #[test]
    fn differential_pair_spacing_readiness_reports_loose_pair_gap() {
        let board = board_with_copper(vec![
            copper_line("USB_D+", CopperKind::Segment, [0.0, 0.0], [1.0, 0.0], 0.10),
            copper_line("USB_D-", CopperKind::Segment, [0.0, 1.0], [1.0, 1.0], 0.10),
        ]);

        let violations = differential_pair_spacing_readiness(&board, &[], 0.30);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "differential-pair-spacing-readiness");
    }

    #[test]
    fn differential_pair_spacing_readiness_allows_close_or_unpaired_features() {
        let board = board_with_copper(vec![
            copper_line("USB_D+", CopperKind::Segment, [0.0, 0.0], [1.0, 0.0], 0.10),
            copper_line(
                "USB_D-",
                CopperKind::Segment,
                [0.0, 0.20],
                [1.0, 0.20],
                0.10,
            ),
            copper_line("GPIO1", CopperKind::Segment, [0.0, 1.0], [1.0, 1.0], 0.10),
        ]);

        assert!(differential_pair_spacing_readiness(&board, &[], 0.30).is_empty());
    }

    #[test]
    fn differential_pair_spacing_readiness_culls_large_passing_pair_fields() {
        let mut copper = vec![
            copper_line("USB_D+", CopperKind::Segment, [0.0, 0.0], [1.0, 0.0], 0.10),
            copper_line(
                "USB_D-",
                CopperKind::Segment,
                [0.0, 0.20],
                [1.0, 0.20],
                0.10,
            ),
        ];
        for index in 0..800 {
            let x = 100.0 + index as f64 * 2.0;
            copper.push(copper_line(
                "USB_D+",
                CopperKind::Segment,
                [x, 10.0],
                [x + 0.5, 10.0],
                0.10,
            ));
            copper.push(copper_line(
                "USB_D-",
                CopperKind::Segment,
                [x, 20.0],
                [x + 0.5, 20.0],
                0.10,
            ));
        }
        let board = board_with_copper(copper);

        let started = std::time::Instant::now();
        let violations = differential_pair_spacing_readiness(&board, &[], 0.30);
        let elapsed = started.elapsed();

        assert!(violations.is_empty());
        assert!(
            elapsed < std::time::Duration::from_secs(2),
            "passing pair spacing should stop after a spatially nearby side match, took {elapsed:?}"
        );
    }

    #[test]
    fn differential_pair_spacing_readiness_respects_selected_layers() {
        let board = board_with_copper(vec![
            copper_line_on_layer(
                "LVDS_CLK_P",
                CopperKind::Segment,
                "B.Cu",
                [0.0, 0.0],
                [1.0, 0.0],
                0.10,
            ),
            copper_line_on_layer(
                "LVDS_CLK_N",
                CopperKind::Segment,
                "B.Cu",
                [0.0, 1.0],
                [1.0, 1.0],
                0.10,
            ),
        ]);

        assert!(
            differential_pair_spacing_readiness(&board, &["F.Cu".to_string()], 0.30).is_empty()
        );
        assert_eq!(
            differential_pair_spacing_readiness(&board, &["B.Cu".to_string()], 0.30).len(),
            1
        );
    }

    #[test]
    fn differential_pair_via_symmetry_readiness_reports_uneven_counts() {
        let board = board_with_copper(vec![
            copper_line(
                "PCIE_TX0_P",
                CopperKind::Segment,
                [0.0, 0.0],
                [1.0, 0.0],
                0.10,
            ),
            copper_disc("PCIE_TX0_P", CopperKind::Via, [0.5, -0.1], 0.12),
            copper_line(
                "PCIE_TX0_N",
                CopperKind::Segment,
                [0.0, 0.2],
                [1.0, 0.2],
                0.10,
            ),
        ]);

        let violations = differential_pair_via_symmetry_readiness(&board, &[]);

        assert_eq!(violations.len(), 1);
        assert_eq!(
            violations[0].check,
            "differential-pair-via-symmetry-readiness"
        );
        assert!(
            violations[0]
                .message
                .as_deref()
                .is_some_and(|message| message.contains("asymmetric via symmetry"))
        );
    }

    #[test]
    fn differential_pair_via_symmetry_readiness_reports_layer_mismatch() {
        let board = board_with_copper(vec![
            copper_disc_on_layer("PCIE_TX0_P", CopperKind::Via, "F.Cu", [0.0, 0.0], 0.12),
            copper_disc_on_layer("PCIE_TX0_N", CopperKind::Via, "B.Cu", [1.0, 0.0], 0.12),
        ]);

        let violations = differential_pair_via_symmetry_readiness(&board, &[]);

        assert_eq!(violations.len(), 1);
    }

    #[test]
    fn differential_pair_via_symmetry_readiness_ignores_non_via_features() {
        let board = board_with_copper(vec![
            copper_line(
                "PCIE_TX0_P",
                CopperKind::Segment,
                [0.0, 0.0],
                [1.0, 0.0],
                0.10,
            ),
            copper_line(
                "PCIE_TX0_N",
                CopperKind::Segment,
                [0.0, 0.2],
                [1.0, 0.2],
                0.10,
            ),
        ]);

        assert!(differential_pair_via_symmetry_readiness(&board, &[]).is_empty());
    }

    #[test]
    fn differential_pair_via_symmetry_readiness_allows_balanced_pair_via_layers() {
        let board = board_with_copper(vec![
            copper_disc_on_layer("PCIE_TX0_P", CopperKind::Via, "F.Cu", [0.0, 0.0], 0.12),
            copper_disc_on_layer("PCIE_TX0_N", CopperKind::Via, "F.Cu", [1.0, 0.0], 0.12),
        ]);

        assert!(differential_pair_via_symmetry_readiness(&board, &[]).is_empty());
    }

    #[test]
    fn differential_pair_return_readiness_reports_pair_without_same_layer_ground_guard() {
        let board = board_with_copper(vec![
            copper_line("USB_D+", CopperKind::Segment, [0.0, 0.0], [1.0, 0.0], 0.10),
            copper_line(
                "USB_D-",
                CopperKind::Segment,
                [0.0, 0.20],
                [1.0, 0.20],
                0.10,
            ),
        ]);

        let violations = differential_pair_return_readiness(&board, &[], 0.30);

        assert_eq!(violations.len(), 2);
        assert!(
            violations
                .iter()
                .all(|violation| violation.check == "differential-pair-return-readiness")
        );
    }

    #[test]
    fn differential_pair_return_readiness_allows_guarded_or_unpaired_copper() {
        let board = board_with_copper(vec![
            copper_line("USB_D+", CopperKind::Segment, [0.0, 0.0], [1.0, 0.0], 0.10),
            copper_line(
                "USB_D-",
                CopperKind::Segment,
                [0.0, 0.20],
                [1.0, 0.20],
                0.10,
            ),
            copper_line("GND", CopperKind::Segment, [0.0, -0.20], [1.0, -0.20], 0.10),
            copper_line("GND", CopperKind::Segment, [0.0, 0.40], [1.0, 0.40], 0.10),
            copper_line("GPIO1", CopperKind::Segment, [2.0, 0.0], [3.0, 0.0], 0.10),
        ]);

        assert!(differential_pair_return_readiness(&board, &[], 0.30).is_empty());
    }

    #[test]
    fn differential_pair_return_readiness_respects_selected_layers() {
        let board = board_with_copper(vec![copper_line_on_layer(
            "LVDS_CLK_P",
            CopperKind::Segment,
            "B.Cu",
            [0.0, 0.0],
            [1.0, 0.0],
            0.10,
        )]);

        assert!(differential_pair_return_readiness(&board, &["F.Cu".to_string()], 0.30).is_empty());
        assert_eq!(
            differential_pair_return_readiness(&board, &["B.Cu".to_string()], 0.30).len(),
            1
        );
    }

    #[test]
    fn differential_pair_return_readiness_culls_sparse_ground_fields() {
        let mut copper = vec![
            copper_line("USB_D+", CopperKind::Segment, [0.0, 0.0], [1.0, 0.0], 0.10),
            copper_line(
                "USB_D-",
                CopperKind::Segment,
                [0.0, 0.20],
                [1.0, 0.20],
                0.10,
            ),
        ];
        for index in 0..2_000 {
            let x = 100.0 + index as f64 * 0.50;
            copper.push(copper_disc("GND", CopperKind::Via, [x, 10.0], 0.12));
        }
        let board = board_with_copper(copper);
        let start = Instant::now();

        let violations = differential_pair_return_readiness(&board, &[], 0.30);

        assert_eq!(violations.len(), 2);
        assert!(
            start.elapsed().as_secs_f64() < 2.0,
            "differential return lookup should index sparse same-layer ground fields"
        );
    }

    #[test]
    fn trace_junction_acid_trap_reports_acute_same_net_segments() {
        let shallow_angle_degrees = 20.0_f64;
        let board = board_with_copper(vec![
            copper_line("SIG", CopperKind::Segment, [0.0, 0.0], [2.0, 0.0], 0.12),
            copper_line(
                "SIG",
                CopperKind::Segment,
                [0.0, 0.0],
                [
                    2.0 * shallow_angle_degrees.to_radians().cos(),
                    2.0 * shallow_angle_degrees.to_radians().sin(),
                ],
                0.12,
            ),
        ]);

        let violations = trace_junction_acid_trap_readiness(&board, &[], 30.0, 1.0e-9);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "acid-trap-trace-junction");
        assert!(
            violations[0]
                .message
                .as_deref()
                .is_some_and(|message| message.contains("acute"))
        );
    }

    #[test]
    fn trace_junction_acid_trap_allows_obtuse_or_different_net_segments() {
        let obtuse = board_with_copper(vec![
            copper_line("SIG", CopperKind::Segment, [0.0, 0.0], [2.0, 0.0], 0.12),
            copper_line("SIG", CopperKind::Segment, [0.0, 0.0], [-1.0, 1.0], 0.12),
        ]);
        let different_net = board_with_copper(vec![
            copper_line("A", CopperKind::Segment, [0.0, 0.0], [2.0, 0.0], 0.12),
            copper_line("B", CopperKind::Segment, [0.0, 0.0], [2.0, 0.2], 0.12),
        ]);

        assert!(trace_junction_acid_trap_readiness(&obtuse, &[], 30.0, 1.0e-9).is_empty());
        assert!(trace_junction_acid_trap_readiness(&different_net, &[], 30.0, 1.0e-9).is_empty());
    }

    #[test]
    fn trace_junction_acid_trap_culls_sparse_segment_fields() {
        let mut copper = Vec::new();
        for index in 0..2_000 {
            let y = index as f64 * 0.50;
            copper.push(copper_line(
                "SIG",
                CopperKind::Segment,
                [100.0, y],
                [101.0, y],
                0.10,
            ));
        }
        let board = board_with_copper(copper);
        let start = Instant::now();

        let violations = trace_junction_acid_trap_readiness(&board, &[], 30.0, 1.0e-9);

        assert!(violations.is_empty());
        assert!(
            start.elapsed().as_secs_f64() < 2.0,
            "trace-junction acid-trap review should index sparse same-net segment fields"
        );
    }

    #[test]
    fn reference_plane_readiness_reports_high_speed_without_ground_zone() {
        let board = board_with_copper(vec![copper_line(
            "USB_D+",
            CopperKind::Segment,
            [0.0, 0.0],
            [1.0, 0.0],
            0.12,
        )]);

        let violations = reference_plane_readiness(&board, &[]);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "reference-plane-readiness");
    }

    #[test]
    fn reference_plane_readiness_allows_ground_zone() {
        let board = board_with_copper(vec![
            copper_line("CLK_OUT", CopperKind::Segment, [0.0, 0.0], [1.0, 0.0], 0.12),
            copper_rect("GND", CopperKind::Zone, "F.Cu", -1.0, -1.0, 2.0, 1.0),
        ]);

        assert!(reference_plane_readiness(&board, &[]).is_empty());
    }

    #[test]
    fn reference_plane_readiness_respects_selected_layers() {
        let board = board_with_copper(vec![copper_line_on_layer(
            "HDMI_CLK",
            CopperKind::Segment,
            "B.Cu",
            [0.0, 0.0],
            [1.0, 0.0],
            0.12,
        )]);

        assert!(reference_plane_readiness(&board, &["F.Cu".to_string()]).is_empty());
        assert_eq!(
            reference_plane_readiness(&board, &["B.Cu".to_string()]).len(),
            1
        );
    }

    #[test]
    fn reference_plane_void_readiness_reports_high_speed_copper_outside_ground_zone() {
        let board = board_with_copper(vec![
            copper_line("USB_D+", CopperKind::Segment, [0.0, 0.0], [2.0, 0.0], 0.12),
            copper_rect("GND", CopperKind::Zone, "B.Cu", -0.5, -0.5, 0.5, 0.5),
        ]);

        let violations = reference_plane_void_readiness(&board, &[], 1.0e-9);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "reference-plane-void-readiness");
    }

    #[test]
    fn reference_plane_void_readiness_allows_covered_or_low_speed_copper() {
        let board = board_with_copper(vec![
            copper_line("USB_D+", CopperKind::Segment, [0.0, 0.0], [1.0, 0.0], 0.12),
            copper_line("GPIO1", CopperKind::Segment, [2.0, 0.0], [3.0, 0.0], 0.12),
            copper_rect("GND", CopperKind::Zone, "B.Cu", -1.0, -1.0, 4.0, 1.0),
        ]);

        assert!(reference_plane_void_readiness(&board, &[], 1.0e-9).is_empty());
    }

    #[test]
    fn reference_plane_void_readiness_respects_selected_layers() {
        let board = board_with_copper(vec![
            copper_line_on_layer(
                "HDMI_CLK",
                CopperKind::Segment,
                "B.Cu",
                [0.0, 0.0],
                [1.0, 0.0],
                0.12,
            ),
            copper_rect("GND", CopperKind::Zone, "F.Cu", -1.0, -1.0, 2.0, 1.0),
        ]);

        assert!(reference_plane_void_readiness(&board, &["F.Cu".to_string()], 1.0e-9).is_empty());
        assert_eq!(
            reference_plane_void_readiness(
                &board,
                &["F.Cu".to_string(), "B.Cu".to_string()],
                1.0e-9
            )
            .len(),
            0
        );
    }

    #[test]
    fn reference_plane_void_readiness_culls_sparse_ground_zones() {
        let mut copper = (0..2_000)
            .map(|index| {
                let x = 100.0 + (index % 100) as f64 * 3.0;
                let y = (index / 100) as f64 * 3.0;
                copper_rect("GND", CopperKind::Zone, "B.Cu", x, y, x + 0.5, y + 0.5)
            })
            .collect::<Vec<_>>();
        copper.push(copper_line(
            "USB_D+",
            CopperKind::Segment,
            [0.0, 0.0],
            [1.0, 0.0],
            0.12,
        ));
        copper.push(copper_rect(
            "GND",
            CopperKind::Zone,
            "B.Cu",
            -1.0,
            -1.0,
            2.0,
            1.0,
        ));
        let board = board_with_copper(copper);

        let started = std::time::Instant::now();
        let violations = reference_plane_void_readiness(&board, &[], 1.0e-9);

        assert!(violations.is_empty());
        assert!(
            started.elapsed() < std::time::Duration::from_secs(2),
            "reference-plane void should index sparse ground zones before exact subtraction"
        );
    }

    #[test]
    fn orphaned_zone_readiness_reports_zone_without_same_net_anchor() {
        let board = board_with_copper(vec![copper_rect(
            "GND",
            CopperKind::Zone,
            "F.Cu",
            -1.0,
            -1.0,
            1.0,
            1.0,
        )]);

        let violations = orphaned_zone_readiness(&board, &[], 0.10);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "orphaned-zone-readiness");
    }

    #[test]
    fn orphaned_zone_readiness_allows_intersecting_or_near_same_net_anchor() {
        let board = board_with_copper(vec![
            copper_rect("GND", CopperKind::Zone, "F.Cu", -1.0, -1.0, 1.0, 1.0),
            copper_disc("GND", CopperKind::Via, [0.0, 0.0], 0.12),
            copper_rect("VBUS", CopperKind::Zone, "F.Cu", 2.0, 2.0, 3.0, 3.0),
            copper_line("VBUS", CopperKind::Segment, [3.05, 2.5], [3.5, 2.5], 0.10),
        ]);

        assert!(orphaned_zone_readiness(&board, &[], 0.10).is_empty());
    }

    #[test]
    fn orphaned_zone_readiness_respects_selected_layers() {
        let board = board_with_copper(vec![copper_rect(
            "GND",
            CopperKind::Zone,
            "B.Cu",
            -1.0,
            -1.0,
            1.0,
            1.0,
        )]);

        assert!(orphaned_zone_readiness(&board, &["F.Cu".to_string()], 0.10).is_empty());
        assert_eq!(
            orphaned_zone_readiness(&board, &["B.Cu".to_string()], 0.10).len(),
            1
        );
    }

    #[test]
    fn orphaned_zone_readiness_culls_sparse_anchor_fields() {
        let mut copper = (0..2_000)
            .map(|index| {
                let x = 100.0 + (index % 100) as f64 * 3.0;
                let y = (index / 100) as f64 * 3.0;
                copper_disc("GND", CopperKind::Via, [x, y], 0.10)
            })
            .collect::<Vec<_>>();
        copper.push(copper_rect(
            "GND",
            CopperKind::Zone,
            "F.Cu",
            -1.0,
            -1.0,
            1.0,
            1.0,
        ));
        copper.push(copper_disc("GND", CopperKind::Via, [0.0, 0.0], 0.12));
        let board = board_with_copper(copper);

        let started = std::time::Instant::now();
        let violations = orphaned_zone_readiness(&board, &[], 0.10);

        assert!(violations.is_empty());
        assert!(
            started.elapsed() < std::time::Duration::from_secs(2),
            "orphaned-zone readiness should index sparse anchors before exact zone connectivity"
        );
    }

    #[test]
    fn same_net_island_readiness_reports_disconnected_same_layer_copper() {
        let board = board_with_copper(vec![
            copper_line("GPIO1", CopperKind::Segment, [0.0, 0.0], [1.0, 0.0], 0.10),
            copper_line("GPIO1", CopperKind::Segment, [3.0, 0.0], [4.0, 0.0], 0.10),
        ]);

        let violations = same_net_island_readiness(&board, &[], 0.10);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "same-net-island-readiness");
        assert_eq!(violations[0].locations.len(), 2);
    }

    #[test]
    fn same_net_island_readiness_allows_touching_or_different_layer_copper() {
        let board = board_with_copper(vec![
            copper_line("GPIO1", CopperKind::Segment, [0.0, 0.0], [1.0, 0.0], 0.10),
            copper_line("GPIO1", CopperKind::Segment, [1.0, 0.0], [2.0, 0.0], 0.10),
            copper_line_on_layer(
                "GPIO1",
                CopperKind::Segment,
                "B.Cu",
                [3.0, 0.0],
                [4.0, 0.0],
                0.10,
            ),
        ]);

        assert!(same_net_island_readiness(&board, &[], 0.10).is_empty());
    }

    #[test]
    fn same_net_island_readiness_respects_selected_layers() {
        let board = board_with_copper(vec![
            copper_line_on_layer(
                "GPIO1",
                CopperKind::Segment,
                "B.Cu",
                [0.0, 0.0],
                [1.0, 0.0],
                0.10,
            ),
            copper_line_on_layer(
                "GPIO1",
                CopperKind::Segment,
                "B.Cu",
                [3.0, 0.0],
                [4.0, 0.0],
                0.10,
            ),
        ]);

        assert!(same_net_island_readiness(&board, &["F.Cu".to_string()], 0.10).is_empty());
        assert_eq!(
            same_net_island_readiness(&board, &["B.Cu".to_string()], 0.10).len(),
            1
        );
    }

    #[test]
    fn same_net_island_readiness_culls_sparse_same_net_fields() {
        let mut copper = Vec::new();
        for index in 0..2_000 {
            let x = index as f64 * 2.0;
            copper.push(copper_line(
                "GPIO1",
                CopperKind::Segment,
                [x, 0.0],
                [x + 0.5, 0.0],
                0.10,
            ));
        }
        let board = board_with_copper(copper);
        let start = Instant::now();

        let violations = same_net_island_readiness(&board, &[], 0.10);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].locations.len(), 2_000);
        assert!(
            start.elapsed().as_secs_f64() < 2.0,
            "same-net island connectivity should index sparse copper fields"
        );
    }

    #[test]
    fn high_current_readiness_reports_power_net_layer_change_with_single_via() {
        let board = board_with_copper(vec![
            copper_line_on_layer(
                "VBUS",
                CopperKind::Segment,
                "F.Cu",
                [0.0, 0.0],
                [1.0, 0.0],
                0.35,
            ),
            copper_line_on_layer(
                "VBUS",
                CopperKind::Segment,
                "B.Cu",
                [2.0, 0.0],
                [3.0, 0.0],
                0.35,
            ),
            copper_disc("VBUS", CopperKind::Via, [1.5, 0.0], 0.15),
        ]);

        let violations = high_current_readiness(&board, &[]);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "high-current-readiness");
        assert_eq!(violations[0].layers, vec!["B.Cu", "F.Cu"]);
    }

    #[test]
    fn high_current_readiness_allows_low_speed_or_redundant_via_nets() {
        let board = board_with_copper(vec![
            copper_line_on_layer(
                "GPIO1",
                CopperKind::Segment,
                "F.Cu",
                [0.0, 0.0],
                [1.0, 0.0],
                0.12,
            ),
            copper_line_on_layer(
                "GPIO1",
                CopperKind::Segment,
                "B.Cu",
                [2.0, 0.0],
                [3.0, 0.0],
                0.12,
            ),
            copper_line_on_layer(
                "+5V",
                CopperKind::Segment,
                "F.Cu",
                [0.0, 1.0],
                [1.0, 1.0],
                0.35,
            ),
            copper_line_on_layer(
                "+5V",
                CopperKind::Segment,
                "B.Cu",
                [2.0, 1.0],
                [3.0, 1.0],
                0.35,
            ),
            copper_disc("+5V", CopperKind::Via, [1.3, 1.0], 0.15),
            copper_disc("+5V", CopperKind::Via, [1.7, 1.0], 0.15),
        ]);

        assert!(high_current_readiness(&board, &[]).is_empty());
    }

    #[test]
    fn high_current_readiness_respects_selected_layers() {
        let board = board_with_copper(vec![
            copper_line_on_layer(
                "12V",
                CopperKind::Segment,
                "F.Cu",
                [0.0, 0.0],
                [1.0, 0.0],
                0.35,
            ),
            copper_line_on_layer(
                "12V",
                CopperKind::Segment,
                "B.Cu",
                [2.0, 0.0],
                [3.0, 0.0],
                0.35,
            ),
        ]);

        assert!(high_current_readiness(&board, &["F.Cu".to_string()]).is_empty());
        assert_eq!(
            high_current_readiness(&board, &["F.Cu".to_string(), "B.Cu".to_string()]).len(),
            1
        );
    }

    #[test]
    fn power_via_array_readiness_reports_isolated_power_vias() {
        let board = board_with_copper(vec![
            copper_disc("VBUS", CopperKind::Via, [0.0, 0.0], 0.12),
            copper_disc("VBUS", CopperKind::Via, [2.0, 0.0], 0.12),
        ]);

        let violations = power_via_array_readiness(&board, &[], 0.50);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "power-via-array-readiness");
        assert_eq!(violations[0].locations.len(), 2);
    }

    #[test]
    fn power_via_array_readiness_allows_clustered_or_low_current_vias() {
        let board = board_with_copper(vec![
            copper_disc("VBUS", CopperKind::Via, [0.0, 0.0], 0.12),
            copper_disc("VBUS", CopperKind::Via, [0.30, 0.0], 0.12),
            copper_disc("GPIO1", CopperKind::Via, [2.0, 0.0], 0.12),
            copper_disc("GPIO1", CopperKind::Via, [4.0, 0.0], 0.12),
        ]);

        assert!(power_via_array_readiness(&board, &[], 0.50).is_empty());
    }

    #[test]
    fn power_via_array_readiness_respects_selected_layers() {
        let board = board_with_copper(vec![
            copper_disc_on_layer("12V", CopperKind::Via, "B.Cu", [0.0, 0.0], 0.12),
            copper_disc_on_layer("12V", CopperKind::Via, "B.Cu", [2.0, 0.0], 0.12),
        ]);

        assert!(power_via_array_readiness(&board, &["F.Cu".to_string()], 0.50).is_empty());
        assert_eq!(
            power_via_array_readiness(&board, &["B.Cu".to_string()], 0.50).len(),
            1
        );
    }

    #[test]
    fn power_via_array_readiness_culls_sparse_via_fields() {
        let mut copper = Vec::new();
        for index in 0..2_000 {
            copper.push(copper_disc(
                "VBUS",
                CopperKind::Via,
                [index as f64 * 2.0, 0.0],
                0.12,
            ));
        }
        let board = board_with_copper(copper);
        let start = Instant::now();

        let violations = power_via_array_readiness(&board, &[], 0.50);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].locations.len(), 2_000);
        assert!(
            start.elapsed().as_secs_f64() < 2.0,
            "power via isolation should use indexed nearest-neighbor lookup"
        );
    }

    #[test]
    fn thermal_via_readiness_reports_power_zone_with_too_few_vias() {
        let board = board_with_copper(vec![
            copper_rect("VBUS", CopperKind::Zone, "F.Cu", -1.0, -1.0, 1.0, 1.0),
            copper_disc("VBUS", CopperKind::Via, [0.0, 0.0], 0.12),
        ]);

        let violations = thermal_via_readiness(&board, &[], 2, 0.10);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "thermal-via-readiness");
    }

    #[test]
    fn thermal_via_readiness_allows_low_current_or_sufficient_vias() {
        let board = board_with_copper(vec![
            copper_rect("SIG", CopperKind::Zone, "F.Cu", -1.0, -1.0, 1.0, 1.0),
            copper_rect("VBUS", CopperKind::Zone, "F.Cu", 2.0, 2.0, 4.0, 4.0),
            copper_disc("VBUS", CopperKind::Via, [2.5, 2.5], 0.12),
            copper_disc("VBUS", CopperKind::Via, [3.5, 3.5], 0.12),
        ]);

        assert!(thermal_via_readiness(&board, &[], 2, 0.10).is_empty());
    }

    #[test]
    fn thermal_via_readiness_respects_selected_layers() {
        let board = board_with_copper(vec![copper_rect(
            "12V",
            CopperKind::Zone,
            "B.Cu",
            -1.0,
            -1.0,
            1.0,
            1.0,
        )]);

        assert!(thermal_via_readiness(&board, &["F.Cu".to_string()], 2, 0.10).is_empty());
        assert_eq!(
            thermal_via_readiness(&board, &["B.Cu".to_string()], 2, 0.10).len(),
            1
        );
    }

    #[test]
    fn power_plane_readiness_reports_power_net_without_zone() {
        let board = board_with_copper(vec![copper_line(
            "VBUS",
            CopperKind::Segment,
            [0.0, 0.0],
            [1.0, 0.0],
            0.25,
        )]);

        let violations = power_plane_readiness(&board, &[]);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "power-plane-readiness");
    }

    #[test]
    fn power_plane_readiness_allows_same_net_zone_or_low_current_net() {
        let board = board_with_copper(vec![
            copper_line("GPIO1", CopperKind::Segment, [0.0, 0.0], [1.0, 0.0], 0.12),
            copper_rect("VDD_3V3", CopperKind::Zone, "F.Cu", -1.0, -1.0, 2.0, 1.0),
        ]);

        assert!(power_plane_readiness(&board, &[]).is_empty());
    }

    #[test]
    fn power_plane_readiness_respects_selected_layers() {
        let board = board_with_copper(vec![copper_line_on_layer(
            "12V",
            CopperKind::Segment,
            "B.Cu",
            [0.0, 0.0],
            [1.0, 0.0],
            0.25,
        )]);

        assert!(power_plane_readiness(&board, &["F.Cu".to_string()]).is_empty());
        assert_eq!(
            power_plane_readiness(&board, &["B.Cu".to_string()]).len(),
            1
        );
    }

    #[test]
    fn power_plane_readiness_handles_large_sparse_power_fields() {
        let mut copper = (0..2_000)
            .map(|index| {
                let x = index as f64 * 2.0;
                copper_line("GPIO1", CopperKind::Segment, [x, 0.0], [x + 1.0, 0.0], 0.12)
            })
            .collect::<Vec<_>>();
        copper.push(copper_line(
            "VBUS",
            CopperKind::Segment,
            [0.0, 2.0],
            [1.0, 2.0],
            0.25,
        ));
        let board = board_with_copper(copper);

        let started = std::time::Instant::now();
        let violations = power_plane_readiness(&board, &[]);

        assert_eq!(violations.len(), 1);
        assert!(
            started.elapsed() < std::time::Duration::from_secs(2),
            "power-plane readiness should stay linear over sparse low-current fields"
        );
    }

    #[test]
    fn high_current_neck_readiness_reports_narrow_power_copper() {
        let board = board_with_copper(vec![copper_line(
            "VBUS",
            CopperKind::Segment,
            [0.0, 0.0],
            [1.0, 0.0],
            0.18,
        )]);

        let violations = high_current_neck_readiness(&board, &[], 0.30);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "high-current-neck-readiness");
    }

    #[test]
    fn high_current_neck_readiness_allows_low_current_or_wide_power_copper() {
        let board = board_with_copper(vec![
            copper_line("GPIO1", CopperKind::Segment, [0.0, 0.0], [1.0, 0.0], 0.12),
            copper_line("VBUS", CopperKind::Segment, [0.0, 1.0], [1.0, 1.0], 0.35),
        ]);

        assert!(high_current_neck_readiness(&board, &[], 0.30).is_empty());
    }

    #[test]
    fn high_current_neck_readiness_respects_selected_layers() {
        let board = board_with_copper(vec![copper_line_on_layer(
            "12V",
            CopperKind::Segment,
            "B.Cu",
            [0.0, 0.0],
            [1.0, 0.0],
            0.18,
        )]);

        assert!(high_current_neck_readiness(&board, &["F.Cu".to_string()], 0.30).is_empty());
        assert_eq!(
            high_current_neck_readiness(&board, &["B.Cu".to_string()], 0.30).len(),
            1
        );
    }

    #[test]
    fn high_current_neck_readiness_handles_large_sparse_power_fields() {
        let mut copper = (0..2_000)
            .map(|index| {
                let x = index as f64 * 2.0;
                copper_line("GPIO1", CopperKind::Segment, [x, 0.0], [x + 1.0, 0.0], 0.12)
            })
            .collect::<Vec<_>>();
        copper.push(copper_line(
            "VBUS",
            CopperKind::Segment,
            [0.0, 2.0],
            [1.0, 2.0],
            0.18,
        ));
        let board = board_with_copper(copper);

        let started = std::time::Instant::now();
        let violations = high_current_neck_readiness(&board, &[], 0.30);

        assert_eq!(violations.len(), 1);
        assert!(
            started.elapsed() < std::time::Duration::from_secs(2),
            "high-current neck readiness should stay linear over sparse low-current fields"
        );
    }

    #[test]
    fn voltage_clearance_readiness_reports_likely_high_voltage_proximity() {
        let board = board_with_copper(vec![
            copper_line("HV_400V", CopperKind::Segment, [0.0, 0.0], [1.0, 0.0], 0.1),
            copper_line("GPIO1", CopperKind::Segment, [0.0, 0.35], [1.0, 0.35], 0.1),
        ]);

        let violations = voltage_clearance_readiness(&board, 0.30, &[], 1.0e-9);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "voltage-clearance-readiness");
    }

    #[test]
    fn voltage_clearance_readiness_allows_low_voltage_or_distant_nets() {
        let board = board_with_copper(vec![
            copper_line("GPIO1", CopperKind::Segment, [0.0, 0.0], [1.0, 0.0], 0.1),
            copper_line("GPIO2", CopperKind::Segment, [0.0, 0.12], [1.0, 0.12], 0.1),
            copper_line("MAINS_L", CopperKind::Segment, [0.0, 2.0], [1.0, 2.0], 0.1),
            copper_line("GND", CopperKind::Segment, [0.0, 2.6], [1.0, 2.6], 0.1),
        ]);

        assert!(voltage_clearance_readiness(&board, 0.30, &[], 1.0e-9).is_empty());
    }

    #[test]
    fn voltage_clearance_readiness_respects_selected_layers() {
        let board = board_with_copper(vec![
            copper_line_on_layer(
                "48V",
                CopperKind::Segment,
                "B.Cu",
                [0.0, 0.0],
                [1.0, 0.0],
                0.1,
            ),
            copper_line_on_layer(
                "GND",
                CopperKind::Segment,
                "B.Cu",
                [0.0, 0.35],
                [1.0, 0.35],
                0.1,
            ),
        ]);

        assert!(
            voltage_clearance_readiness(&board, 0.30, &["F.Cu".to_string()], 1.0e-9).is_empty()
        );
        assert_eq!(
            voltage_clearance_readiness(&board, 0.30, &["B.Cu".to_string()], 1.0e-9).len(),
            1
        );
    }

    #[test]
    fn sensitive_net_spacing_readiness_reports_sensitive_near_noisy_net() {
        let board = board_with_copper(vec![
            copper_line("RF_ANT", CopperKind::Segment, [0.0, 0.0], [1.0, 0.0], 0.1),
            copper_line(
                "MOTOR_PWM",
                CopperKind::Segment,
                [0.0, 0.35],
                [1.0, 0.35],
                0.1,
            ),
        ]);

        let violations = sensitive_net_spacing_readiness(&board, 0.30, &[], 1.0e-9);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "sensitive-net-spacing-readiness");
    }

    #[test]
    fn sensitive_net_spacing_readiness_allows_distant_or_non_noisy_neighbors() {
        let board = board_with_copper(vec![
            copper_line("RF_ANT", CopperKind::Segment, [0.0, 0.0], [1.0, 0.0], 0.1),
            copper_line("GPIO1", CopperKind::Segment, [0.0, 0.12], [1.0, 0.12], 0.1),
            copper_line("MIC_IN", CopperKind::Segment, [0.0, 2.0], [1.0, 2.0], 0.1),
            copper_line("SW_NODE", CopperKind::Segment, [0.0, 2.6], [1.0, 2.6], 0.1),
        ]);

        assert!(sensitive_net_spacing_readiness(&board, 0.30, &[], 1.0e-9).is_empty());
    }

    #[test]
    fn sensitive_net_spacing_readiness_respects_selected_layers() {
        let board = board_with_copper(vec![
            copper_line_on_layer(
                "ADC_IN",
                CopperKind::Segment,
                "B.Cu",
                [0.0, 0.0],
                [1.0, 0.0],
                0.1,
            ),
            copper_line_on_layer(
                "CLK_OUT",
                CopperKind::Segment,
                "B.Cu",
                [0.0, 0.35],
                [1.0, 0.35],
                0.1,
            ),
        ]);

        assert!(
            sensitive_net_spacing_readiness(&board, 0.30, &["F.Cu".to_string()], 1.0e-9).is_empty()
        );
        assert_eq!(
            sensitive_net_spacing_readiness(&board, 0.30, &["B.Cu".to_string()], 1.0e-9).len(),
            1
        );
    }

    #[test]
    fn sensitive_return_readiness_reports_missing_same_layer_ground_guard() {
        let board = board_with_copper(vec![copper_line(
            "ADC_IN",
            CopperKind::Segment,
            [0.0, 0.0],
            [1.0, 0.0],
            0.10,
        )]);

        let violations = sensitive_return_readiness(&board, &[], 0.30);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "sensitive-return-readiness");
    }

    #[test]
    fn sensitive_return_readiness_allows_nearby_ground_or_low_sensitivity() {
        let board = board_with_copper(vec![
            copper_line("ADC_IN", CopperKind::Segment, [0.0, 0.0], [1.0, 0.0], 0.10),
            copper_line("GND", CopperKind::Segment, [0.0, 0.20], [1.0, 0.20], 0.10),
            copper_line("GPIO1", CopperKind::Segment, [2.0, 0.0], [3.0, 0.0], 0.10),
        ]);

        assert!(sensitive_return_readiness(&board, &[], 0.30).is_empty());
    }

    #[test]
    fn sensitive_return_readiness_respects_selected_layers() {
        let board = board_with_copper(vec![copper_line_on_layer(
            "MIC_IN",
            CopperKind::Segment,
            "B.Cu",
            [0.0, 0.0],
            [1.0, 0.0],
            0.10,
        )]);

        assert!(sensitive_return_readiness(&board, &["F.Cu".to_string()], 0.30).is_empty());
        assert_eq!(
            sensitive_return_readiness(&board, &["B.Cu".to_string()], 0.30).len(),
            1
        );
    }

    #[test]
    fn rf_keepout_readiness_reports_antenna_near_non_ground_copper() {
        let board = board_with_copper(vec![
            copper_line("RF_ANT", CopperKind::Segment, [0.0, 0.0], [1.0, 0.0], 0.10),
            copper_line("GPIO1", CopperKind::Segment, [0.0, 0.50], [1.0, 0.50], 0.10),
        ]);

        let violations = rf_keepout_readiness(&board, 0.60, &[], 1.0e-9);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "rf-keepout-readiness");
    }

    #[test]
    fn rf_keepout_readiness_allows_ground_or_distant_copper() {
        let board = board_with_copper(vec![
            copper_line("RF_ANT", CopperKind::Segment, [0.0, 0.0], [1.0, 0.0], 0.10),
            copper_line("GND", CopperKind::Segment, [0.0, 0.20], [1.0, 0.20], 0.10),
            copper_line("GPIO1", CopperKind::Segment, [0.0, 2.0], [1.0, 2.0], 0.10),
        ]);

        assert!(rf_keepout_readiness(&board, 0.60, &[], 1.0e-9).is_empty());
    }

    #[test]
    fn rf_keepout_readiness_respects_selected_layers() {
        let board = board_with_copper(vec![
            copper_line_on_layer(
                "WIFI_ANT",
                CopperKind::Segment,
                "B.Cu",
                [0.0, 0.0],
                [1.0, 0.0],
                0.10,
            ),
            copper_line_on_layer(
                "GPIO1",
                CopperKind::Segment,
                "B.Cu",
                [0.0, 0.50],
                [1.0, 0.50],
                0.10,
            ),
        ]);

        assert!(rf_keepout_readiness(&board, 0.60, &["F.Cu".to_string()], 1.0e-9).is_empty());
        assert_eq!(
            rf_keepout_readiness(&board, 0.60, &["B.Cu".to_string()], 1.0e-9).len(),
            1
        );
    }

    #[test]
    fn rf_via_fence_readiness_reports_rf_copper_without_nearby_ground_via() {
        let board = board_with_copper(vec![copper_line(
            "RF_ANT",
            CopperKind::Segment,
            [0.0, 0.0],
            [1.0, 0.0],
            0.10,
        )]);

        let violations = rf_via_fence_readiness(&board, &[], 0.60);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "rf-via-fence-readiness");
    }

    #[test]
    fn rf_via_fence_readiness_allows_nearby_ground_via_or_non_rf_copper() {
        let board = board_with_copper(vec![
            copper_line("RF_ANT", CopperKind::Segment, [0.0, 0.0], [1.0, 0.0], 0.10),
            copper_disc("GND", CopperKind::Via, [0.2, 0.2], 0.12),
            copper_line("GPIO1", CopperKind::Segment, [3.0, 0.0], [4.0, 0.0], 0.10),
        ]);

        assert!(rf_via_fence_readiness(&board, &[], 0.60).is_empty());
    }

    #[test]
    fn rf_via_fence_readiness_respects_selected_layers() {
        let board = board_with_copper(vec![copper_line_on_layer(
            "RF_FEED",
            CopperKind::Segment,
            "B.Cu",
            [0.0, 0.0],
            [1.0, 0.0],
            0.10,
        )]);

        assert!(rf_via_fence_readiness(&board, &["F.Cu".to_string()], 0.60).is_empty());
        assert_eq!(
            rf_via_fence_readiness(&board, &["B.Cu".to_string()], 0.60).len(),
            1
        );
    }

    #[test]
    fn chassis_stitching_readiness_reports_shield_without_ground_via() {
        let board = board_with_copper(vec![copper_line(
            "USB_SHIELD",
            CopperKind::Segment,
            [0.0, 0.0],
            [1.0, 0.0],
            0.20,
        )]);

        let violations = chassis_stitching_readiness(&board, &[], 0.50);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "chassis-stitching-readiness");
    }

    #[test]
    fn chassis_stitching_readiness_allows_nearby_ground_via_or_signal_net() {
        let board = board_with_copper(vec![
            copper_line(
                "USB_SHIELD",
                CopperKind::Segment,
                [0.0, 0.0],
                [1.0, 0.0],
                0.20,
            ),
            copper_disc("GND", CopperKind::Via, [0.30, 0.0], 0.12),
            copper_line("GPIO1", CopperKind::Segment, [2.0, 0.0], [3.0, 0.0], 0.10),
        ]);

        assert!(chassis_stitching_readiness(&board, &[], 0.50).is_empty());
    }

    #[test]
    fn chassis_stitching_readiness_accepts_retained_kicad_grid_for_point_radius() {
        let board = board_with_copper(vec![
            copper_disc("USB_SHIELD", CopperKind::Via, [0.0, 0.0], 0.20),
            copper_disc("GND", CopperKind::Via, [0.0, 0.50], 0.12),
        ]);
        let grid = SourceGridFacts::primitive_float_edge(SourceUnit::KiCadMillimeter);

        assert!(chassis_stitching_readiness_with_grid(&board, &[], 0.50, grid).is_empty());
        assert_eq!(
            chassis_stitching_readiness_with_grid(&board, &[], 0.499_999, grid).len(),
            1,
            "exact squared-distance comparison must keep the radius boundary strict"
        );
    }

    #[test]
    fn chassis_stitching_readiness_respects_selected_layers() {
        let board = board_with_copper(vec![copper_line_on_layer(
            "CHASSIS",
            CopperKind::Segment,
            "B.Cu",
            [0.0, 0.0],
            [1.0, 0.0],
            0.20,
        )]);

        assert!(chassis_stitching_readiness(&board, &["F.Cu".to_string()], 0.50).is_empty());
        assert_eq!(
            chassis_stitching_readiness(&board, &["B.Cu".to_string()], 0.50).len(),
            1
        );
    }

    #[test]
    fn chassis_stitching_readiness_culls_sparse_ground_vias() {
        let board = board_with_copper(
            (0..2_000)
                .map(|index| {
                    copper_disc(
                        "GND",
                        CopperKind::Via,
                        [100.0 + index as f64 * 2.0, 50.0],
                        0.10,
                    )
                })
                .chain([copper_line(
                    "USB_SHIELD",
                    CopperKind::Segment,
                    [0.0, 0.0],
                    [1.0, 0.0],
                    0.20,
                )])
                .collect(),
        );

        let start = Instant::now();
        let violations = chassis_stitching_readiness(&board, &[], 0.50);

        assert_eq!(violations.len(), 1);
        assert!(
            start.elapsed().as_secs_f64() < 2.0,
            "chassis stitching should index sparse ground-via fields"
        );
    }

    #[test]
    fn gold_finger_readiness_reports_via_on_likely_finger_net() {
        let board = board_with_copper(vec![copper_disc(
            "GOLD_FINGER_1",
            CopperKind::Via,
            [0.0, 0.0],
            0.12,
        )]);

        let violations = gold_finger_readiness(&board, &[]);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "gold-finger-readiness");
    }

    #[test]
    fn gold_finger_readiness_allows_non_finger_vias_or_finger_pads() {
        let board = board_with_copper(vec![
            copper_disc("SIG", CopperKind::Via, [0.0, 0.0], 0.12),
            copper_disc("GOLD_FINGER_1", CopperKind::Pad, [1.0, 0.0], 0.20),
        ]);

        assert!(gold_finger_readiness(&board, &[]).is_empty());
    }

    #[test]
    fn gold_finger_readiness_respects_selected_layers() {
        let board = board_with_copper(vec![copper_disc_on_layer(
            "EDGE_CONN_1",
            CopperKind::Via,
            "B.Cu",
            [0.0, 0.0],
            0.12,
        )]);

        assert!(gold_finger_readiness(&board, &["F.Cu".to_string()]).is_empty());
        assert_eq!(
            gold_finger_readiness(&board, &["B.Cu".to_string()]).len(),
            1
        );
    }

    #[test]
    fn gold_finger_readiness_handles_large_sparse_non_finger_fields() {
        let mut copper = (0..2_000)
            .map(|index| {
                copper_disc(
                    "SIG",
                    CopperKind::Via,
                    [100.0 + index as f64 * 2.0, 50.0],
                    0.10,
                )
            })
            .collect::<Vec<_>>();
        copper.push(copper_disc(
            "EDGE_CONN_1",
            CopperKind::Via,
            [0.0, 0.0],
            0.12,
        ));
        let board = board_with_copper(copper);

        let started = Instant::now();
        let violations = gold_finger_readiness(&board, &[]);

        assert_eq!(violations.len(), 1);
        assert!(
            started.elapsed() < std::time::Duration::from_secs(2),
            "gold-finger via-intent review should stay linear over sparse non-finger fields"
        );
    }

    #[test]
    fn gold_finger_edge_readiness_reports_finger_copper_away_from_edge() {
        let mut board = board_with_outline(square(0.0, 0.0, 20.0, 20.0));
        board.copper = vec![copper_rect(
            "GOLD_FINGER_1",
            CopperKind::Pad,
            "F.Cu",
            5.0,
            9.0,
            6.0,
            10.0,
        )];

        let violations = gold_finger_edge_readiness(&board, &[], 1.0);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "gold-finger-edge-readiness");
        assert!(
            violations[0]
                .message
                .as_deref()
                .is_some_and(|message| message.contains("bevel intent"))
        );
    }

    #[test]
    fn gold_finger_edge_readiness_accepts_edge_fingers_or_missing_outline() {
        let mut board = board_with_outline(square(0.0, 0.0, 20.0, 20.0));
        board.copper = vec![copper_rect(
            "GOLD_FINGER_1",
            CopperKind::Pad,
            "F.Cu",
            0.2,
            9.0,
            0.8,
            10.0,
        )];
        let no_outline = board_with_copper(vec![copper_rect(
            "GOLD_FINGER_2",
            CopperKind::Pad,
            "F.Cu",
            5.0,
            9.0,
            6.0,
            10.0,
        )]);

        assert!(gold_finger_edge_readiness(&board, &[], 1.0).is_empty());
        assert!(gold_finger_edge_readiness(&no_outline, &[], 1.0).is_empty());
    }

    #[test]
    fn gold_finger_edge_readiness_handles_sparse_non_finger_fields() {
        let mut board = board_with_outline(square(0.0, 0.0, 20.0, 20.0));
        board.copper = (0..2_000)
            .map(|index| {
                let x = 2.0 + (index % 100) as f64 * 0.10;
                let y = 2.0 + (index / 100) as f64 * 0.10;
                copper_rect("SIG", CopperKind::Pad, "F.Cu", x, y, x + 0.04, y + 0.04)
            })
            .chain([copper_rect(
                "GOLD_FINGER_1",
                CopperKind::Pad,
                "F.Cu",
                9.0,
                9.0,
                10.0,
                10.0,
            )])
            .collect();

        let started = Instant::now();
        let violations = gold_finger_edge_readiness(&board, &[], 1.0);

        assert_eq!(violations.len(), 1);
        assert!(
            started.elapsed() < std::time::Duration::from_secs(2),
            "gold-finger edge review should stay linear over sparse non-finger fields"
        );
    }

    #[test]
    fn gold_finger_spacing_readiness_reports_tight_finger_pitch() {
        let board = board_with_copper(vec![
            copper_rect("GOLD_FINGER_1", CopperKind::Pad, "F.Cu", 0.0, 0.0, 0.5, 2.0),
            copper_rect(
                "GOLD_FINGER_2",
                CopperKind::Pad,
                "F.Cu",
                0.55,
                0.0,
                1.0,
                2.0,
            ),
        ]);

        let violations = gold_finger_spacing_readiness(&board, &[], 0.10, 1.0e-9);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "gold-finger-spacing-readiness");
        assert!(
            violations[0]
                .message
                .as_deref()
                .is_some_and(|message| message.contains("contact pitch"))
        );
    }

    #[test]
    fn gold_finger_spacing_readiness_allows_distant_or_same_net_fingers() {
        let board = board_with_copper(vec![
            copper_rect("GOLD_FINGER_1", CopperKind::Pad, "F.Cu", 0.0, 0.0, 0.5, 2.0),
            copper_rect(
                "GOLD_FINGER_1",
                CopperKind::Pad,
                "F.Cu",
                0.55,
                0.0,
                1.0,
                2.0,
            ),
            copper_rect("GOLD_FINGER_2", CopperKind::Pad, "F.Cu", 2.0, 0.0, 2.5, 2.0),
        ]);

        assert!(gold_finger_spacing_readiness(&board, &[], 0.10, 1.0e-9).is_empty());
    }

    #[test]
    fn gold_finger_spacing_readiness_culls_sparse_finger_fields() {
        let mut copper = (0..2_000)
            .map(|index| {
                copper_rect(
                    &format!("GOLD_FINGER_{index}"),
                    CopperKind::Pad,
                    "F.Cu",
                    100.0 + index as f64 * 5.0,
                    0.0,
                    100.5 + index as f64 * 5.0,
                    2.0,
                )
            })
            .collect::<Vec<_>>();
        copper.push(copper_rect(
            "GOLD_FINGER_A",
            CopperKind::Pad,
            "F.Cu",
            0.0,
            0.0,
            0.5,
            2.0,
        ));
        copper.push(copper_rect(
            "GOLD_FINGER_B",
            CopperKind::Pad,
            "F.Cu",
            0.55,
            0.0,
            1.0,
            2.0,
        ));
        let board = board_with_copper(copper);

        let started = std::time::Instant::now();
        let violations = gold_finger_spacing_readiness(&board, &[], 0.10, 1.0e-9);

        assert_eq!(violations.len(), 1);
        assert!(
            started.elapsed() < std::time::Duration::from_secs(2),
            "gold-finger spacing should cull sparse card-edge contacts before exact CSG"
        );
    }

    #[test]
    fn gold_finger_drill_keepout_readiness_reports_nearby_drills() {
        let board = board_with_copper(vec![copper_rect(
            "EDGE_CONN_1",
            CopperKind::Pad,
            "F.Cu",
            0.0,
            0.0,
            1.0,
            2.0,
        )]);
        let sidecar_drills = vec![DrillFeature {
            location: [1.3, 1.0],
            diameter: 0.6,
            net: None,
            plated: false,
        }];

        let violations =
            gold_finger_drill_keepout_readiness(&board, &sidecar_drills, &[], 0.2, 1.0e-9);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "gold-finger-drill-keepout-readiness");
        assert!(
            violations[0]
                .message
                .as_deref()
                .is_some_and(|message| message.contains("no-drill finger plating"))
        );
    }

    #[test]
    fn gold_finger_drill_keepout_readiness_accepts_distant_drills() {
        let board = board_with_copper(vec![copper_rect(
            "EDGE_CONN_1",
            CopperKind::Pad,
            "F.Cu",
            0.0,
            0.0,
            1.0,
            2.0,
        )]);
        let sidecar_drills = vec![DrillFeature {
            location: [4.0, 1.0],
            diameter: 0.6,
            net: None,
            plated: false,
        }];

        assert!(
            gold_finger_drill_keepout_readiness(&board, &sidecar_drills, &[], 0.2, 1.0e-9)
                .is_empty()
        );
    }

    #[test]
    fn gold_finger_drill_keepout_readiness_culls_sparse_finger_fields() {
        let copper = (0..2_000)
            .map(|index| {
                copper_rect(
                    &format!("GOLD_FINGER_{index}"),
                    CopperKind::Pad,
                    "F.Cu",
                    100.0 + index as f64 * 5.0,
                    0.0,
                    100.5 + index as f64 * 5.0,
                    2.0,
                )
            })
            .chain([copper_rect(
                "GOLD_FINGER_NEAR",
                CopperKind::Pad,
                "F.Cu",
                0.0,
                0.0,
                1.0,
                2.0,
            )])
            .collect::<Vec<_>>();
        let board = board_with_copper(copper);
        let sidecar_drills = vec![DrillFeature {
            location: [1.25, 1.0],
            diameter: 0.6,
            net: None,
            plated: false,
        }];

        let started = std::time::Instant::now();
        let violations =
            gold_finger_drill_keepout_readiness(&board, &sidecar_drills, &[], 0.4, 1.0e-9);

        assert_eq!(violations.len(), 1);
        assert!(
            started.elapsed() < std::time::Duration::from_secs(2),
            "gold-finger drill keepout should cull sparse finger fields before exact CSG"
        );
    }

    #[test]
    fn component_edge_clearance_readiness_reports_close_component_pads() {
        let mut board = board_with_outline(square(0.0, 0.0, 20.0, 20.0));
        board.copper = vec![
            copper_disc("U1_IO", CopperKind::Pad, [0.7, 10.0], 0.4),
            copper_disc("CARD_EDGE_1", CopperKind::Pad, [0.2, 2.0], 0.4),
            unnetted_copper_disc_on_layer("F.Cu", [0.8, 5.0], 0.4),
        ];

        let violations = component_edge_clearance_readiness(&board, &[], 0.5);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "component-edge-clearance-readiness");
        assert!(
            violations[0]
                .message
                .as_deref()
                .is_some_and(|message| message.contains("depanelization"))
        );
    }

    #[test]
    fn component_edge_clearance_readiness_allows_inset_or_selected_out_pads() {
        let mut board = board_with_outline(square(0.0, 0.0, 20.0, 20.0));
        board.copper = vec![
            copper_disc_on_layer("U1_IO", CopperKind::Pad, "B.Cu", [0.7, 10.0], 0.4),
            copper_disc_on_layer("U2_IO", CopperKind::Pad, "F.Cu", [10.0, 10.0], 0.4),
        ];

        assert!(component_edge_clearance_readiness(&board, &["F.Cu".to_string()], 0.5).is_empty());
    }

    #[test]
    fn component_edge_clearance_readiness_culls_sparse_rectangular_boards() {
        let mut board = board_with_outline(square(0.0, 0.0, 200.0, 200.0));
        board.copper = (0..2_000)
            .map(|index| {
                copper_disc(
                    &format!("U{index}_IO"),
                    CopperKind::Pad,
                    [
                        20.0 + (index % 80) as f64 * 2.0,
                        20.0 + (index / 80) as f64 * 2.0,
                    ],
                    0.50,
                )
            })
            .collect();
        board
            .copper
            .push(copper_disc("U_NEAR", CopperKind::Pad, [0.65, 100.0], 0.30));

        let started = std::time::Instant::now();
        let violations = component_edge_clearance_readiness(&board, &[], 0.5);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].layers, vec!["F.Cu".to_string()]);
        assert!(
            started.elapsed() < std::time::Duration::from_secs(2),
            "component-edge clearance should reject interior rectangular-board pads before exact outline distance"
        );
    }

    #[test]
    fn component_hole_clearance_readiness_reports_pads_near_mechanical_holes() {
        let board = board_with_copper(vec![copper_disc(
            "U1_IO",
            CopperKind::Pad,
            [1.0, 0.0],
            0.30,
        )]);
        let sidecar_drills = vec![DrillFeature {
            location: [0.0, 0.0],
            diameter: 1.0,
            net: None,
            plated: false,
        }];

        let violations =
            component_hole_clearance_readiness(&board, &sidecar_drills, &[], 0.30, 1.0e-9);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "component-hole-clearance-readiness");
        assert!(
            violations[0]
                .message
                .as_deref()
                .is_some_and(|message| message.contains("standoff"))
        );
    }

    #[test]
    fn component_hole_clearance_readiness_allows_distant_or_plated_holes() {
        let mut board = board_with_copper(vec![copper_disc(
            "U1_IO",
            CopperKind::Pad,
            [5.0, 0.0],
            0.30,
        )]);
        board.drills = vec![
            DrillFeature {
                location: [0.0, 0.0],
                diameter: 1.0,
                net: None,
                plated: false,
            },
            DrillFeature {
                location: [5.0, 0.0],
                diameter: 1.0,
                net: Some("U1_IO".to_string()),
                plated: true,
            },
        ];

        assert!(component_hole_clearance_readiness(&board, &[], &[], 0.30, 1.0e-9).is_empty());
    }

    #[test]
    fn connector_rework_clearance_readiness_reports_tight_neighboring_pads() {
        let board = board_with_copper(vec![
            copper_rect("USB_VBUS", CopperKind::Pad, "F.Cu", 0.0, 0.0, 1.0, 1.0),
            copper_rect("GPIO1", CopperKind::Pad, "F.Cu", 1.10, 0.0, 1.60, 1.0),
        ]);

        let violations = connector_rework_clearance_readiness(&board, &[], 0.20, 0.40);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "connector-rework-clearance-readiness");
        assert!(
            violations[0]
                .message
                .as_deref()
                .is_some_and(|message| message.contains("rework access"))
        );
    }

    #[test]
    fn connector_rework_clearance_readiness_allows_distant_or_small_pads() {
        let board = board_with_copper(vec![
            copper_rect("USB_VBUS", CopperKind::Pad, "F.Cu", 0.0, 0.0, 0.20, 0.20),
            copper_rect("GPIO1", CopperKind::Pad, "F.Cu", 1.10, 0.0, 1.60, 1.0),
            copper_rect("USB_D_P", CopperKind::Pad, "F.Cu", 5.0, 0.0, 6.0, 1.0),
            copper_rect("GPIO2", CopperKind::Pad, "F.Cu", 7.0, 0.0, 8.0, 1.0),
        ]);

        assert!(connector_rework_clearance_readiness(&board, &[], 0.20, 0.40).is_empty());
    }

    #[test]
    fn connector_return_path_readiness_reports_edge_rate_connector_without_ground_return() {
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

        let violations = connector_return_path_readiness(&board, &[], 1.0, 2.0);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "connector-return-path-readiness");
        assert!(
            violations[0]
                .message
                .as_deref()
                .is_some_and(|message| message.contains("ground return"))
        );
    }

    #[test]
    fn connector_return_path_readiness_accepts_nearby_ground_or_inset_connector() {
        let mut board = board_with_outline(square(0.0, 0.0, 20.0, 20.0));
        board.copper = vec![
            copper_rect("USB_D_P", CopperKind::Pad, "F.Cu", 0.4, 8.0, 1.0, 8.6),
            copper_disc("GND", CopperKind::Via, [1.5, 8.3], 0.12),
            copper_rect("USB_D_N", CopperKind::Pad, "F.Cu", 5.0, 8.0, 5.6, 8.6),
        ];

        assert!(connector_return_path_readiness(&board, &[], 1.0, 2.0).is_empty());
    }

    #[test]
    fn connector_return_path_readiness_culls_sparse_ground_fields() {
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
        for index in 0..2_000 {
            board.copper.push(copper_disc(
                "GND",
                CopperKind::Via,
                [100.0 + index as f64 * 2.0, 50.0],
                0.12,
            ));
        }
        let start = Instant::now();

        let violations = connector_return_path_readiness(&board, &[], 1.0, 2.0);

        assert_eq!(violations.len(), 1);
        assert!(
            start.elapsed().as_secs_f64() < 2.0,
            "connector return-path lookup should index sparse ground fields"
        );
    }

    #[test]
    fn decoupling_proximity_readiness_reports_power_feature_without_ground_return() {
        let board = board_with_copper(vec![
            copper_disc("VDD_3V3", CopperKind::Pad, [0.0, 0.0], 0.3),
            copper_disc("GND", CopperKind::Pad, [5.0, 0.0], 0.3),
        ]);

        let violations = decoupling_proximity_readiness(&board, &[], 1.0);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "decoupling-proximity-readiness");
        assert!(
            violations[0]
                .message
                .as_deref()
                .is_some_and(|message| message.contains("decoupling capacitor loop"))
        );
    }

    #[test]
    fn decoupling_proximity_readiness_allows_nearby_ground_or_selected_out_layers() {
        let board = board_with_copper(vec![
            copper_disc("VDD_3V3", CopperKind::Pad, [0.0, 0.0], 0.3),
            copper_disc("GND", CopperKind::Pad, [0.8, 0.0], 0.3),
            copper_disc_on_layer("VBUS", CopperKind::Via, "B.Cu", [5.0, 0.0], 0.3),
            copper_disc_on_layer("GND", CopperKind::Via, "B.Cu", [5.8, 0.0], 0.3),
        ]);

        assert!(decoupling_proximity_readiness(&board, &[], 1.0).is_empty());
        assert!(decoupling_proximity_readiness(&board, &["F.Cu".to_string()], 1.0).is_empty());
    }

    #[test]
    fn decoupling_proximity_readiness_culls_sparse_ground_fields() {
        let mut copper = vec![copper_disc("VDD_3V3", CopperKind::Pad, [0.0, 0.0], 0.3)];
        for index in 0..2_000 {
            copper.push(copper_disc(
                "GND",
                CopperKind::Pad,
                [100.0 + index as f64 * 2.0, 20.0],
                0.3,
            ));
        }
        let board = board_with_copper(copper);
        let start = Instant::now();

        let violations = decoupling_proximity_readiness(&board, &[], 1.0);

        assert_eq!(violations.len(), 1);
        assert!(
            start.elapsed().as_secs_f64() < 2.0,
            "decoupling proximity lookup should index sparse ground fields"
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
        assert!(
            violations[0]
                .message
                .as_deref()
                .is_some_and(|message| message.contains("ESD"))
        );
    }

    #[test]
    fn esd_protection_readiness_accepts_nearby_tvs_chassis_or_inset_net() {
        let mut board = board_with_outline(square(0.0, 0.0, 20.0, 20.0));
        board.copper = vec![
            copper_rect("USB_D_P", CopperKind::Pad, "F.Cu", 0.4, 8.0, 1.0, 8.6),
            copper_disc("USB_ESD_CLAMP", CopperKind::Pad, [1.5, 8.3], 0.12),
            copper_rect("USB_D_N", CopperKind::Pad, "F.Cu", 5.0, 8.0, 5.6, 8.6),
        ];

        assert!(esd_protection_readiness(&board, &[], 1.0, 2.0).is_empty());
    }

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
    fn testpoint_coverage_readiness_reports_critical_nets_missing_ipc_records() {
        let board = board_with_copper(vec![
            copper_disc("VBUS", CopperKind::Pad, [0.0, 0.0], 0.4),
            copper_disc("STATUS_LED", CopperKind::Pad, [5.0, 0.0], 0.4),
            copper_disc("GND", CopperKind::Pad, [10.0, 0.0], 0.4),
        ]);
        let ipc_points = vec![Ipc356Point {
            net: "GND".to_string(),
            reference: Some("TP1".to_string()),
            pin: Some("1".to_string()),
            location: [10.0, 0.0],
            diameter: None,
            access_side: None,
            feature_type: None,
            soldermask: None,
        }];

        let violations = testpoint_coverage_readiness(&board, &ipc_points, &[]);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "testpoint-coverage-readiness");
        assert_eq!(violations[0].layers, vec!["VBUS"]);
    }

    #[test]
    fn testpoint_coverage_readiness_allows_covered_or_noncritical_nets() {
        let board = board_with_copper(vec![
            copper_disc("VBUS", CopperKind::Pad, [0.0, 0.0], 0.4),
            copper_disc("STATUS_LED", CopperKind::Pad, [5.0, 0.0], 0.4),
        ]);
        let ipc_points = vec![Ipc356Point {
            net: "vbus".to_string(),
            reference: Some("TP1".to_string()),
            pin: Some("1".to_string()),
            location: [0.0, 0.0],
            diameter: None,
            access_side: None,
            feature_type: None,
            soldermask: None,
        }];

        assert!(testpoint_coverage_readiness(&board, &ipc_points, &[]).is_empty());
    }

    #[test]
    fn testpoint_coverage_readiness_handles_sparse_critical_nets() {
        let mut copper = (0..2_000)
            .map(|index| {
                copper_disc(
                    &format!("VBUS_{index}"),
                    CopperKind::Pad,
                    [index as f64 * 2.0, 0.0],
                    0.4,
                )
            })
            .collect::<Vec<_>>();
        copper.push(copper_disc(
            "VDD_MISSING",
            CopperKind::Pad,
            [9_000.0, 0.0],
            0.4,
        ));
        let points = (0..2_000)
            .map(|index| Ipc356Point {
                net: format!("VBUS_{index}"),
                reference: Some(format!("TP{index}")),
                pin: Some("1".to_string()),
                location: [index as f64 * 2.0, 0.0],
                diameter: None,
                access_side: None,
                feature_type: None,
                soldermask: None,
            })
            .collect::<Vec<_>>();
        let board = board_with_copper(copper);

        let started = std::time::Instant::now();
        let violations = testpoint_coverage_readiness(&board, &points, &[]);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].layers, vec!["VDD_MISSING"]);
        assert!(
            started.elapsed() < std::time::Duration::from_secs(2),
            "testpoint coverage should stay bounded on sparse critical-net sets"
        );
    }

    #[test]
    fn testpoint_accessibility_readiness_reports_probe_diameter_spacing_and_edge_risks() {
        let board = board_with_outline(square(0.0, 0.0, 20.0, 20.0));
        let ipc_points = vec![
            Ipc356Point {
                net: "GND".to_string(),
                reference: Some("TP1".to_string()),
                pin: Some("1".to_string()),
                location: [0.8, 10.0],
                diameter: Some(0.20),
                access_side: None,
                feature_type: None,
                soldermask: None,
            },
            Ipc356Point {
                net: "VBUS".to_string(),
                reference: Some("TP2".to_string()),
                pin: Some("1".to_string()),
                location: [1.1, 10.0],
                diameter: Some(0.30),
                access_side: None,
                feature_type: None,
                soldermask: None,
            },
            Ipc356Point {
                net: "RESET".to_string(),
                reference: Some("TP3".to_string()),
                pin: Some("1".to_string()),
                location: [10.0, 10.0],
                diameter: None,
                access_side: None,
                feature_type: None,
                soldermask: None,
            },
        ];

        let violations = testpoint_accessibility_readiness(&board, &ipc_points, 0.25, 0.20, 1.0);
        let messages = violations
            .iter()
            .filter_map(|violation| violation.message.as_deref())
            .collect::<Vec<_>>();

        assert!(
            messages
                .iter()
                .any(|message| message.contains("below minimum probe diameter"))
        );
        assert!(
            messages
                .iter()
                .any(|message| message.contains("below fixture edge clearance"))
        );
        assert!(
            messages
                .iter()
                .any(|message| message.contains("below fixture probe spacing"))
        );
        assert!(
            messages
                .iter()
                .any(|message| message.contains("no parsed probe diameter"))
        );
        assert!(
            messages
                .iter()
                .any(|message| message.contains("no soldermask access flag"))
        );
        assert!(
            messages
                .iter()
                .any(|message| message.contains("no parsed access side"))
        );
    }

    #[test]
    fn testpoint_accessibility_readiness_allows_complete_accessible_points() {
        let board = board_with_outline(square(0.0, 0.0, 20.0, 20.0));
        let ipc_points = vec![
            Ipc356Point {
                net: "GND".to_string(),
                reference: Some("TP1".to_string()),
                pin: Some("1".to_string()),
                location: [5.0, 5.0],
                diameter: Some(0.35),
                access_side: Some(Ipc356AccessSide::Top),
                feature_type: Some(Ipc356FeatureType::Smd),
                soldermask: Some(Ipc356Soldermask::Open),
            },
            Ipc356Point {
                net: "VBUS".to_string(),
                reference: Some("TP2".to_string()),
                pin: Some("1".to_string()),
                location: [8.0, 5.0],
                diameter: Some(0.35),
                access_side: Some(Ipc356AccessSide::Top),
                feature_type: Some(Ipc356FeatureType::Smd),
                soldermask: Some(Ipc356Soldermask::Open),
            },
        ];

        assert!(testpoint_accessibility_readiness(&board, &ipc_points, 0.25, 0.20, 1.0).is_empty());
    }

    #[test]
    fn testpoint_accessibility_readiness_reports_mask_and_side_metadata_risks() {
        let board = board_with_outline(square(0.0, 0.0, 20.0, 20.0));
        let ipc_points = vec![
            Ipc356Point {
                net: "TP_COVERED".to_string(),
                reference: Some("TP1".to_string()),
                pin: Some("1".to_string()),
                location: [5.0, 5.0],
                diameter: Some(0.35),
                access_side: Some(Ipc356AccessSide::Top),
                feature_type: Some(Ipc356FeatureType::Smd),
                soldermask: Some(Ipc356Soldermask::Covered),
            },
            Ipc356Point {
                net: "TP_UNKNOWN".to_string(),
                reference: Some("TP2".to_string()),
                pin: Some("1".to_string()),
                location: [8.0, 5.0],
                diameter: Some(0.35),
                access_side: Some(Ipc356AccessSide::Top),
                feature_type: Some(Ipc356FeatureType::Smd),
                soldermask: Some(Ipc356Soldermask::Unknown),
            },
            Ipc356Point {
                net: "TP_BOTH".to_string(),
                reference: Some("TP3".to_string()),
                pin: Some("1".to_string()),
                location: [11.0, 5.0],
                diameter: Some(0.35),
                access_side: Some(Ipc356AccessSide::Both),
                feature_type: Some(Ipc356FeatureType::Smd),
                soldermask: Some(Ipc356Soldermask::Open),
            },
        ];

        let violations = testpoint_accessibility_readiness(&board, &ipc_points, 0.25, 0.20, 1.0);
        let messages = violations
            .iter()
            .filter_map(|violation| violation.message.as_deref())
            .collect::<Vec<_>>();

        assert!(
            messages
                .iter()
                .any(|message| message.contains("marked soldermask-covered"))
        );
        assert!(
            messages
                .iter()
                .any(|message| message.contains("unknown soldermask access"))
        );
        assert!(
            messages
                .iter()
                .any(|message| message.contains("accessible from both sides"))
        );
    }

    #[test]
    fn testpoint_accessibility_readiness_reports_ipc_side_disagreeing_with_kicad_pad_side() {
        let board = BoardModel {
            source: "test".to_string(),
            copper: vec![copper_disc_on_layer(
                "USB_D+",
                CopperKind::Pad,
                "B.Cu",
                [5.0, 5.0],
                0.20,
            )],
            drills: Vec::new(),
            board_outline: Some(sketch(vec![square(0.0, 0.0, 20.0, 20.0)])),
            panel_features: None,
        };
        let ipc_points = vec![Ipc356Point {
            net: "USB_D+".to_string(),
            reference: Some("TP1".to_string()),
            pin: Some("1".to_string()),
            location: [5.0, 5.0],
            diameter: Some(0.35),
            access_side: Some(Ipc356AccessSide::Top),
            feature_type: Some(Ipc356FeatureType::Smd),
            soldermask: Some(Ipc356Soldermask::Open),
        }];

        let violations = testpoint_accessibility_readiness(&board, &ipc_points, 0.25, 0.20, 1.0);

        assert!(violations.iter().any(|violation| {
            violation.message.as_deref().is_some_and(|message| {
                message.contains("access side is top")
                    && message.contains("KiCad pad/via copper is only on bottom")
            })
        }));
    }

    #[test]
    fn testpoint_accessibility_readiness_allows_ipc_side_matching_kicad_pad_side() {
        let board = BoardModel {
            source: "test".to_string(),
            copper: vec![copper_disc_on_layer(
                "USB_D+",
                CopperKind::Pad,
                "B.Cu",
                [5.0, 5.0],
                0.20,
            )],
            drills: Vec::new(),
            board_outline: Some(sketch(vec![square(0.0, 0.0, 20.0, 20.0)])),
            panel_features: None,
        };
        let ipc_points = vec![Ipc356Point {
            net: "USB_D+".to_string(),
            reference: Some("TP1".to_string()),
            pin: Some("1".to_string()),
            location: [5.0, 5.0],
            diameter: Some(0.35),
            access_side: Some(Ipc356AccessSide::Bottom),
            feature_type: Some(Ipc356FeatureType::Smd),
            soldermask: Some(Ipc356Soldermask::Open),
        }];
        let messages = testpoint_accessibility_readiness(&board, &ipc_points, 0.25, 0.20, 1.0)
            .into_iter()
            .filter_map(|violation| violation.message)
            .collect::<Vec<_>>();

        assert!(
            !messages
                .iter()
                .any(|message| message.contains("nearby same-net KiCad pad/via copper"))
        );
    }

    #[test]
    fn tooling_hole_readiness_reports_missing_or_edge_close_tooling_holes() {
        let mut board = board_with_outline(square(0.0, 0.0, 20.0, 20.0));
        board.drills = vec![DrillFeature {
            location: [0.8, 10.0],
            diameter: 1.0,
            net: None,
            plated: false,
        }];

        let violations = tooling_hole_readiness(&board, &[], 0.8, 4.0, 1.0);
        let messages = violations
            .iter()
            .filter_map(|violation| violation.message.as_deref())
            .collect::<Vec<_>>();

        assert!(
            messages
                .iter()
                .any(|message| message.contains("found 1 likely tooling hole"))
        );
        assert!(
            messages
                .iter()
                .any(|message| message.contains("below fixture edge clearance"))
        );
    }

    #[test]
    fn tooling_hole_readiness_accepts_inset_board_and_sidecar_tooling_holes() {
        let mut board = board_with_outline(square(0.0, 0.0, 20.0, 20.0));
        board.drills = vec![DrillFeature {
            location: [5.0, 5.0],
            diameter: 1.5,
            net: None,
            plated: false,
        }];
        let sidecar_drills = vec![DrillFeature {
            location: [15.0, 15.0],
            diameter: 1.5,
            net: None,
            plated: false,
        }];

        assert!(tooling_hole_readiness(&board, &sidecar_drills, 0.8, 4.0, 1.0).is_empty());
    }

    #[test]
    fn tooling_hole_readiness_filters_sparse_drill_tables() {
        let mut board = board_with_outline(square(0.0, 0.0, 100.0, 100.0));
        board.drills = (0..2_000)
            .map(|index| DrillFeature {
                location: [200.0 + index as f64 * 2.0, 200.0],
                diameter: 0.40,
                net: None,
                plated: false,
            })
            .chain([
                DrillFeature {
                    location: [10.0, 10.0],
                    diameter: 1.50,
                    net: None,
                    plated: false,
                },
                DrillFeature {
                    location: [90.0, 90.0],
                    diameter: 1.50,
                    net: None,
                    plated: false,
                },
            ])
            .collect();

        let started = std::time::Instant::now();
        let violations = tooling_hole_readiness(&board, &[], 0.8, 4.0, 1.0);

        assert!(violations.is_empty());
        assert!(
            started.elapsed() < std::time::Duration::from_secs(2),
            "tooling-hole readiness should filter sparse drill tables before outline edge review"
        );
    }

    #[test]
    fn mouse_bite_readiness_reports_bad_diameter_and_spacing() {
        let sidecar_drills = vec![
            DrillFeature {
                location: [0.0, 0.0],
                diameter: 0.20,
                net: None,
                plated: false,
            },
            DrillFeature {
                location: [0.30, 0.0],
                diameter: 0.30,
                net: None,
                plated: false,
            },
            DrillFeature {
                location: [2.00, 0.0],
                diameter: 0.30,
                net: None,
                plated: false,
            },
        ];

        let violations = mouse_bite_readiness(
            &board_with_copper(Vec::new()),
            &sidecar_drills,
            0.25,
            0.50,
            0.40,
            1.20,
        );
        let messages = violations
            .iter()
            .filter_map(|violation| violation.message.as_deref())
            .collect::<Vec<_>>();

        assert!(
            messages
                .iter()
                .any(|message| message.contains("diameter") && message.contains("below minimum"))
        );
        assert!(
            messages
                .iter()
                .any(|message| message.contains("center spacing")
                    && message.contains("outside expected range"))
        );
    }

    #[test]
    fn mouse_bite_readiness_accepts_reasonable_small_npth_rows() {
        let sidecar_drills = vec![
            DrillFeature {
                location: [0.0, 0.0],
                diameter: 0.30,
                net: None,
                plated: false,
            },
            DrillFeature {
                location: [0.70, 0.0],
                diameter: 0.30,
                net: None,
                plated: false,
            },
            DrillFeature {
                location: [1.40, 0.0],
                diameter: 0.30,
                net: None,
                plated: false,
            },
        ];

        assert!(
            mouse_bite_readiness(
                &board_with_copper(Vec::new()),
                &sidecar_drills,
                0.25,
                0.50,
                0.40,
                1.20
            )
            .is_empty()
        );
    }

    #[test]
    fn fiducial_readiness_reports_missing_and_edge_close_fiducials() {
        let mut board = board_with_outline(square(0.0, 0.0, 20.0, 20.0));
        board.copper = vec![
            unnetted_copper_disc_on_layer("F.Cu", [5.0, 5.0], 0.5),
            unnetted_copper_disc_on_layer("F.Cu", [0.6, 10.0], 0.5),
            copper_disc_on_layer("GND", CopperKind::Pad, "B.Cu", [10.0, 10.0], 0.5),
        ];

        let violations = fiducial_readiness(&board, &[], 1.0);

        assert!(violations.iter().any(|violation| {
            violation
                .message
                .as_deref()
                .is_some_and(|message| message.contains("likely fiducial is"))
        }));
        assert!(violations.iter().any(|violation| {
            violation.layers == vec!["B.Cu"]
                && violation
                    .message
                    .as_deref()
                    .is_some_and(|message| message.contains("0 likely fiducial"))
        }));
    }

    #[test]
    fn fiducial_readiness_allows_two_inset_side_fiducials() {
        let mut board = board_with_outline(square(0.0, 0.0, 20.0, 20.0));
        board.copper = vec![
            unnetted_copper_disc_on_layer("F.Cu", [4.0, 4.0], 0.5),
            unnetted_copper_disc_on_layer("F.Cu", [16.0, 16.0], 0.5),
        ];

        assert!(fiducial_readiness(&board, &[], 1.0).is_empty());
    }

    #[test]
    fn fiducial_readiness_culls_sparse_rectangular_interior_targets() {
        let mut board = board_with_outline(square(0.0, 0.0, 200.0, 200.0));
        board.copper = (0..2_000)
            .map(|index| {
                unnetted_copper_disc_on_layer(
                    "F.Cu",
                    [
                        20.0 + (index % 80) as f64 * 2.0,
                        20.0 + (index / 80) as f64 * 2.0,
                    ],
                    0.30,
                )
            })
            .collect();

        let started = std::time::Instant::now();
        let violations = fiducial_readiness(&board, &[], 1.0);

        assert!(violations.is_empty());
        assert!(
            started.elapsed() < std::time::Duration::from_secs(2),
            "fiducial readiness should reject interior rectangular-board fiducials before exact outline distance"
        );
    }

    #[test]
    fn local_fiducial_readiness_reports_dense_clusters_without_nearby_fiducials() {
        let mut copper = dense_pad_cluster();
        copper.push(unnetted_copper_disc_on_layer("F.Cu", [20.0, 20.0], 0.5));
        copper.push(unnetted_copper_disc_on_layer("F.Cu", [22.0, 20.0], 0.5));

        let violations = local_fiducial_readiness(&board_with_copper(copper), &[], 0.8, 5.0);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "local-fiducial-readiness");
        assert!(
            violations[0]
                .message
                .as_deref()
                .is_some_and(|message| message.contains("only 0 likely local fiducial"))
        );
    }

    #[test]
    fn local_fiducial_readiness_accepts_nearby_local_fiducials_or_sparse_pads() {
        let mut copper = dense_pad_cluster();
        copper.push(unnetted_copper_disc_on_layer("F.Cu", [0.0, 3.0], 0.5));
        copper.push(unnetted_copper_disc_on_layer("F.Cu", [3.0, 0.0], 0.5));

        assert!(local_fiducial_readiness(&board_with_copper(copper), &[], 0.8, 5.0).is_empty());

        let sparse = (0..8)
            .map(|index| {
                copper_disc(
                    &format!("PAD_{index}"),
                    CopperKind::Pad,
                    [index as f64, 0.0],
                    0.12,
                )
            })
            .collect::<Vec<_>>();
        assert!(local_fiducial_readiness(&board_with_copper(sparse), &[], 0.8, 5.0).is_empty());
    }

    #[test]
    fn dense_pad_escape_readiness_reports_fine_pitch_cluster_without_escape_vias() {
        let board = board_with_copper(dense_pad_cluster());

        let violations = dense_pad_escape_readiness(&board, &[], 0.8, 2.0);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "dense-pad-escape-readiness");
    }

    #[test]
    fn dense_pad_escape_readiness_allows_sparse_or_via_escaped_clusters() {
        let mut sparse = Vec::new();
        for index in 0..16 {
            sparse.push(copper_disc(
                &format!("PAD_{index}"),
                CopperKind::Pad,
                [index as f64, 0.0],
                0.12,
            ));
        }
        assert!(dense_pad_escape_readiness(&board_with_copper(sparse), &[], 0.8, 2.0).is_empty());

        let mut escaped = Vec::new();
        for x in 0..4 {
            for y in 0..4 {
                escaped.push(copper_disc(
                    &format!("BGA_{x}_{y}"),
                    CopperKind::Pad,
                    [x as f64 * 0.5, y as f64 * 0.5],
                    0.12,
                ));
            }
        }
        escaped.push(copper_disc("ESCAPE", CopperKind::Via, [0.75, 0.75], 0.15));

        assert!(dense_pad_escape_readiness(&board_with_copper(escaped), &[], 0.8, 2.0).is_empty());
    }

    #[test]
    fn thermal_pad_via_readiness_reports_large_power_or_ground_pads_without_vias() {
        let board = board_with_copper(vec![
            copper_rect("GND", CopperKind::Pad, "F.Cu", 0.0, 0.0, 3.0, 3.0),
            copper_rect("GPIO", CopperKind::Pad, "F.Cu", 5.0, 0.0, 8.0, 3.0),
        ]);

        let violations = thermal_pad_via_readiness(&board, &[], 2.0);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "thermal-pad-via-readiness");
        assert!(
            violations[0]
                .message
                .as_deref()
                .is_some_and(|message| message.contains("thermal via array"))
        );
    }

    #[test]
    fn thermal_pad_via_readiness_accepts_same_net_via_in_large_pad() {
        let board = board_with_copper(vec![
            copper_rect("GND", CopperKind::Pad, "F.Cu", 0.0, 0.0, 3.0, 3.0),
            copper_disc("GND", CopperKind::Via, [1.5, 1.5], 0.12),
        ]);

        assert!(thermal_pad_via_readiness(&board, &[], 2.0).is_empty());
    }

    #[test]
    fn thermal_copper_area_readiness_reports_power_feature_without_nearby_zone() {
        let board = board_with_copper(vec![
            copper_disc("VREG_OUT", CopperKind::Pad, [0.0, 0.0], 0.30),
            copper_rect("VREG_OUT", CopperKind::Zone, "F.Cu", 5.0, 0.0, 7.0, 2.0),
        ]);

        let violations = thermal_copper_area_readiness(&board, &[], 2.0);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "thermal-copper-area-readiness");
        assert!(
            violations[0]
                .message
                .as_deref()
                .is_some_and(|message| message.contains("heat spreading"))
        );
    }

    #[test]
    fn thermal_copper_area_readiness_accepts_nearby_same_net_zone_or_low_power_net() {
        let board = board_with_copper(vec![
            copper_disc("VREG_OUT", CopperKind::Pad, [0.0, 0.0], 0.30),
            copper_rect("VREG_OUT", CopperKind::Zone, "F.Cu", 0.8, 0.0, 2.0, 1.0),
            copper_disc("GPIO1", CopperKind::Pad, [5.0, 0.0], 0.30),
        ]);

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
        assert!(
            violations[0]
                .message
                .as_deref()
                .is_some_and(|message| message.contains("derating"))
        );
    }

    #[test]
    fn hot_component_spacing_readiness_allows_ground_same_net_or_distant_neighbors() {
        let board = board_with_copper(vec![
            copper_rect("LED_PWR", CopperKind::Pad, "F.Cu", 0.0, 0.0, 1.0, 1.0),
            copper_rect("LED_PWR", CopperKind::Zone, "F.Cu", 1.2, 0.0, 2.0, 1.0),
            copper_rect("GND", CopperKind::Zone, "F.Cu", 1.2, 1.5, 2.0, 2.5),
            copper_rect("SENSOR_OUT", CopperKind::Pad, "F.Cu", 5.0, 0.0, 6.0, 1.0),
        ]);

        assert!(hot_component_spacing_readiness(&board, &[], 0.3, 1.0e-9).is_empty());
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
        assert!(
            violations[0]
                .message
                .as_deref()
                .is_some_and(|message| message.contains("heatsink"))
        );
    }

    #[test]
    fn thermal_mechanical_keepout_readiness_accepts_distant_or_plated_holes() {
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
            net: Some("HEATER_OUT".to_string()),
            plated: true,
        }];
        let sidecar_drills = vec![DrillFeature {
            location: [5.0, 0.0],
            diameter: 0.8,
            net: None,
            plated: false,
        }];

        assert!(
            thermal_mechanical_keepout_readiness(&board, &sidecar_drills, &[], 0.2, 1.0e-9)
                .is_empty()
        );
    }

    #[test]
    fn return_path_readiness_reports_high_speed_via_without_ground_stitch() {
        let board = board_with_copper(vec![copper_disc(
            "USB_D+",
            CopperKind::Via,
            [0.0, 0.0],
            0.12,
        )]);

        let violations = return_path_readiness(&board, 0.50, &[]);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "return-path-readiness");
    }

    #[test]
    fn return_path_readiness_allows_nearby_ground_stitch_or_low_speed_vias() {
        let board = board_with_copper(vec![
            copper_disc("USB_D+", CopperKind::Via, [0.0, 0.0], 0.12),
            copper_disc("GND", CopperKind::Via, [0.30, 0.0], 0.12),
            copper_disc("GPIO1", CopperKind::Via, [2.0, 0.0], 0.12),
        ]);

        assert!(return_path_readiness(&board, 0.50, &[]).is_empty());
    }

    #[test]
    fn return_path_readiness_respects_selected_layers() {
        let board = board_with_copper(vec![copper_disc_on_layer(
            "PCIE_RX0",
            CopperKind::Via,
            "B.Cu",
            [0.0, 0.0],
            0.12,
        )]);

        assert!(return_path_readiness(&board, 0.50, &["F.Cu".to_string()]).is_empty());
        assert_eq!(
            return_path_readiness(&board, 0.50, &["B.Cu".to_string()]).len(),
            1
        );
    }

    #[test]
    fn return_path_readiness_culls_sparse_ground_vias() {
        let board = board_with_copper(
            (0..2_000)
                .map(|index| {
                    copper_disc(
                        "GND",
                        CopperKind::Via,
                        [100.0 + index as f64 * 2.0, 50.0],
                        0.10,
                    )
                })
                .chain([copper_disc("USB_D+", CopperKind::Via, [0.0, 0.0], 0.12)])
                .collect(),
        );

        let start = Instant::now();
        let violations = return_path_readiness(&board, 0.50, &[]);

        assert_eq!(violations.len(), 1);
        assert!(
            start.elapsed().as_secs_f64() < 2.0,
            "return-path via stitching should index sparse ground-via fields"
        );
    }

    #[test]
    fn net_spacing_distance_fallback_covers_trace_clearances() {
        assert_eq!(
            net_spacing(
                &board_with_copper(vec![
                    copper_line("A", CopperKind::Segment, [0.0, 0.0], [1.0, 0.0], 0.1),
                    copper_line("B", CopperKind::Segment, [0.0, 0.18], [1.0, 0.18], 0.1),
                ]),
                0.10,
                &[],
                1.0e-9,
            )
            .len(),
            1
        );

        assert_eq!(
            net_spacing(
                &board_with_copper(vec![
                    copper_line("A", CopperKind::Segment, [0.0, 0.0], [1.0, 0.0], 0.1),
                    copper_disc("B", CopperKind::Pad, [1.15, 0.0], 0.06),
                ]),
                0.10,
                &[],
                1.0e-9,
            )
            .len(),
            1
        );

        assert_eq!(
            net_spacing(
                &board_with_copper(vec![
                    copper_line("A", CopperKind::Segment, [0.0, 0.0], [1.0, 0.0], 0.1),
                    copper_disc("B", CopperKind::Via, [0.5, 0.20], 0.06),
                ]),
                0.10,
                &[],
                1.0e-9,
            )
            .len(),
            1
        );
    }

    #[test]
    fn net_spacing_allows_trace_clearances_above_threshold() {
        let board = board_with_copper(vec![
            copper_line("A", CopperKind::Segment, [0.0, 0.0], [1.0, 0.0], 0.1),
            copper_line("B", CopperKind::Segment, [0.0, 0.30], [1.0, 0.30], 0.1),
        ]);

        assert!(net_spacing(&board, 0.10, &[], 1.0e-9).is_empty());
    }

    #[test]
    fn net_spacing_culls_sparse_different_net_fields() {
        let mut copper = (0..2_000)
            .map(|index| {
                copper_disc(
                    &format!("SIG{index}"),
                    CopperKind::Pad,
                    [100.0 + index as f64 * 2.0, 100.0],
                    0.10,
                )
            })
            .collect::<Vec<_>>();
        copper.push(copper_disc("A", CopperKind::Pad, [0.0, 0.0], 0.20));
        copper.push(copper_disc("B", CopperKind::Pad, [0.25, 0.0], 0.20));

        let started = Instant::now();
        let violations = net_spacing(&board_with_copper(copper), 0.10, &[], 1.0e-9);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "different-net-spacing");
        assert!(
            started.elapsed() < std::time::Duration::from_secs(2),
            "different-net spacing should use the copper spatial index before exact offsets"
        );
    }

    #[test]
    fn drill_to_copper_clearance_flags_hole_trace_and_slot_trace_cases() {
        let trace = copper_line("SIG", CopperKind::Segment, [0.0, 0.0], [2.0, 0.0], 0.1);
        let board = BoardModel {
            source: "test".to_string(),
            copper: vec![trace],
            drills: Vec::new(),
            board_outline: None,
            panel_features: None,
        };
        let extra_drills = vec![
            DrillFeature {
                location: [0.5, 0.18],
                diameter: 0.2,
                net: None,
                plated: false,
            },
            DrillFeature {
                location: [1.5, 0.32],
                diameter: 0.2,
                net: None,
                plated: false,
            },
        ];

        let violations = drill_to_copper_clearance(&board, &extra_drills, 0.15, &[], 1.0e-9);

        assert_eq!(violations.len(), 1);
    }

    #[test]
    fn drill_to_copper_clearance_respects_layer_selection() {
        let trace = copper_line_on_layer(
            "SIG",
            CopperKind::Segment,
            "B.Cu",
            [0.0, 0.0],
            [2.0, 0.0],
            0.10,
        );
        let board = BoardModel {
            source: "test".to_string(),
            copper: vec![trace],
            drills: Vec::new(),
            board_outline: None,
            panel_features: None,
        };
        let extra_drills = vec![DrillFeature {
            location: [1.0, 0.06],
            diameter: 0.1,
            net: None,
            plated: false,
        }];

        let unselected =
            drill_to_copper_clearance(&board, &extra_drills, 0.02, &["F.Cu".to_string()], 1.0e-9);
        let selected =
            drill_to_copper_clearance(&board, &extra_drills, 0.02, &["B.Cu".to_string()], 1.0e-9);

        assert!(unselected.is_empty());
        assert_eq!(selected.len(), 1);
    }

    #[test]
    fn drill_to_copper_clearance_includes_sidecar_drills() {
        let board = BoardModel {
            source: "test".to_string(),
            copper: vec![copper_line(
                "SIG",
                CopperKind::Segment,
                [0.0, 0.0],
                [2.0, 0.0],
                0.10,
            )],
            drills: vec![DrillFeature {
                location: [0.0, 0.0],
                diameter: 0.8,
                net: None,
                plated: false,
            }],
            board_outline: None,
            panel_features: None,
        };
        let sidecar_drills = vec![DrillFeature {
            location: [1.2, 0.55],
            diameter: 0.4,
            net: None,
            plated: false,
        }];

        let violations = drill_to_copper_clearance(&board, &sidecar_drills, 0.15, &[], 1.0e-9);

        assert_eq!(violations.len(), 1);
    }

    #[test]
    fn drill_to_copper_clearance_ignores_same_net_plated_drills() {
        let trace = copper_line("GND", CopperKind::Segment, [0.0, 0.0], [2.0, 0.0], 0.1);
        let board = BoardModel {
            source: "test".to_string(),
            copper: vec![trace],
            drills: vec![DrillFeature {
                location: [1.0, 0.0],
                diameter: 0.3,
                net: Some("GND".to_string()),
                plated: true,
            }],
            board_outline: None,
            panel_features: None,
        };

        let violations = drill_to_copper_clearance(&board, &[], 0.15, &[], 1.0e-9);

        assert!(violations.is_empty());
    }

    #[test]
    fn drill_to_copper_clearance_flags_same_net_npth_drills() {
        let trace = copper_line("GND", CopperKind::Segment, [0.0, 0.0], [2.0, 0.0], 0.1);
        let board = BoardModel {
            source: "test".to_string(),
            copper: vec![trace],
            drills: vec![DrillFeature {
                location: [1.0, 0.0],
                diameter: 0.3,
                net: Some("GND".to_string()),
                plated: false,
            }],
            board_outline: None,
            panel_features: None,
        };

        let violations = drill_to_copper_clearance(&board, &[], 0.15, &[], 1.0e-9);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "drill-to-copper-clearance");
    }

    #[test]
    fn drill_spacing_allows_tangent_drills_at_zero_clearance() {
        let drills = vec![
            DrillFeature {
                location: [0.0, 0.0],
                diameter: 0.4,
                net: None,
                plated: false,
            },
            DrillFeature {
                location: [0.4, 0.0],
                diameter: 0.4,
                net: None,
                plated: false,
            },
        ];

        assert!(drill_spacing(&drills, &[], 0.0).is_empty());
    }

    #[test]
    fn drill_spacing_reports_multiple_violating_pairs() {
        let drills = vec![
            DrillFeature {
                location: [0.0, 0.0],
                diameter: 0.4,
                net: None,
                plated: false,
            },
            DrillFeature {
                location: [0.2, 0.0],
                diameter: 0.4,
                net: None,
                plated: false,
            },
            DrillFeature {
                location: [0.4, 0.0],
                diameter: 0.4,
                net: None,
                plated: false,
            },
        ];

        assert_eq!(drill_spacing(&drills, &[], 0.20).len(), 3);
    }

    #[test]
    fn drill_table_consistency_treats_exact_matches_with_tolerance_as_clean() {
        let board_drills = vec![DrillFeature {
            location: [0.0, 0.0],
            diameter: 0.30,
            net: None,
            plated: true,
        }];
        let excellon_drills = vec![DrillFeature {
            location: [0.001, 0.0],
            diameter: 0.32,
            net: None,
            plated: true,
        }];

        assert!(drill_table_consistency(&board_drills, &excellon_drills, &[], 0.04).is_empty());
    }

    #[test]
    fn drill_table_consistency_reports_kicad_excellon_and_ipc_conflicts() {
        let board_drills = vec![DrillFeature {
            location: [0.0, 0.0],
            diameter: 0.20,
            net: None,
            plated: true,
        }];
        let excellon_drills = vec![DrillFeature {
            location: [0.001, 0.0],
            diameter: 0.35,
            net: None,
            plated: true,
        }];
        let points = vec![Ipc356Point {
            net: "SIG".to_string(),
            reference: Some("X1".to_string()),
            pin: Some("1".to_string()),
            location: [0.001, 0.0],
            diameter: Some(0.50),
            access_side: None,
            feature_type: None,
            soldermask: None,
        }];

        let violations = drill_table_consistency(&board_drills, &excellon_drills, &points, 0.04);

        assert_eq!(violations.len(), 2);
    }

    #[test]
    fn drill_spacing_flags_close_holes_and_allows_compliant_holes() {
        let drills = vec![
            DrillFeature {
                location: [0.0, 0.0],
                diameter: 0.4,
                net: None,
                plated: false,
            },
            DrillFeature {
                location: [0.55, 0.0],
                diameter: 0.4,
                net: None,
                plated: false,
            },
            DrillFeature {
                location: [2.0, 0.0],
                diameter: 0.4,
                net: None,
                plated: false,
            },
        ];

        let violations = drill_spacing(&drills, &[], 0.20);

        assert_eq!(violations.len(), 1);
        assert!(
            violations[0]
                .message
                .as_deref()
                .unwrap()
                .contains("below clearance")
        );
    }

    #[test]
    fn drill_spacing_includes_excellon_sidecar_hits() {
        let board_drills = vec![DrillFeature {
            location: [0.0, 0.0],
            diameter: 0.4,
            net: Some("GND".to_string()),
            plated: true,
        }];
        let excellon_drills = vec![DrillFeature {
            location: [0.5, 0.0],
            diameter: 0.3,
            net: None,
            plated: false,
        }];

        let violations = drill_spacing(&board_drills, &excellon_drills, 0.20);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].locations.len(), 2);
    }

    #[test]
    fn board_outline_drill_clearance_reports_hole_near_edge() {
        let outline = sketch(vec![square(0.0, 0.0, 10.0, 10.0)]);
        let drills = vec![DrillFeature {
            location: [0.4, 5.0],
            diameter: 0.4,
            net: None,
            plated: false,
        }];

        let violations = board_outline_drill_clearance(
            "KiCad drills",
            "KiCad Edge.Cuts",
            &outline,
            &drills,
            &[],
            0.25,
            1.0e-9,
        );

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "board-outline-drill-clearance");
    }

    #[test]
    fn board_outline_drill_clearance_allows_inset_hole() {
        let outline = sketch(vec![square(0.0, 0.0, 10.0, 10.0)]);
        let drills = vec![DrillFeature {
            location: [1.0, 5.0],
            diameter: 0.4,
            net: None,
            plated: false,
        }];

        assert!(
            board_outline_drill_clearance(
                "KiCad drills",
                "KiCad Edge.Cuts",
                &outline,
                &drills,
                &[],
                0.25,
                1.0e-9,
            )
            .is_empty()
        );
    }

    #[test]
    fn board_outline_drill_clearance_allows_clearance_boundary_touch() {
        // When the drill keepout only touches the outline boundary, there is no
        // positive-area area outside the board profile to report as a violation.
        let outline = sketch(vec![square(0.0, 0.0, 10.0, 10.0)]);
        let drills = vec![DrillFeature {
            location: [0.45, 5.0],
            diameter: 0.4,
            net: None,
            plated: false,
        }];

        let violations = board_outline_drill_clearance(
            "KiCad drills",
            "KiCad Edge.Cuts",
            &outline,
            &drills,
            &[],
            0.25,
            1.0e-9,
        );

        assert!(violations.is_empty());
    }

    #[test]
    fn board_outline_drill_clearance_flags_just_outside_clearance() {
        // A drill shifted slightly inside the clearance envelope should still
        // generate a concrete geometry violation.
        let outline = sketch(vec![square(0.0, 0.0, 10.0, 10.0)]);
        let drills = vec![DrillFeature {
            location: [0.449, 5.0],
            diameter: 0.4,
            net: None,
            plated: false,
        }];

        let violations = board_outline_drill_clearance(
            "KiCad drills",
            "KiCad Edge.Cuts",
            &outline,
            &drills,
            &[],
            0.25,
            1.0e-9,
        );

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].locations, vec![[0.449, 5.0]]);
    }

    #[test]
    fn board_outline_drill_clearance_includes_all_drill_sources_in_label() {
        let outline = sketch(vec![square(0.0, 0.0, 10.0, 10.0)]);
        let board_drills = vec![DrillFeature {
            location: [9.6, 5.0],
            diameter: 0.4,
            net: None,
            plated: false,
        }];
        let extra_drills = vec![DrillFeature {
            location: [0.2, 5.0],
            diameter: 0.4,
            net: None,
            plated: false,
        }];

        let violations = board_outline_drill_clearance(
            "KiCad + Excellon drills",
            "KiCad Edge.Cuts",
            &outline,
            &board_drills,
            &extra_drills,
            0.25,
            1.0e-9,
        );

        assert_eq!(violations.len(), 2);
        for violation in &violations {
            assert_eq!(
                violation.layers,
                vec![
                    "KiCad + Excellon drills".to_string(),
                    "KiCad Edge.Cuts".to_string()
                ]
            );
        }
        assert!(
            violations
                .iter()
                .any(|violation| violation.locations == vec![[0.2, 5.0]])
        );
        assert!(
            violations
                .iter()
                .any(|violation| violation.locations == vec![[9.6, 5.0]])
        );
    }

    #[test]
    fn board_outline_drill_clearance_min_area_filters_tiny_penetration() {
        // Tiny floating-point-edge intrusions should be suppressible through
        // min_area, which prevents reporting extremely small geometry artifacts.
        let outline = sketch(vec![square(0.0, 0.0, 10.0, 10.0)]);
        let drills = vec![DrillFeature {
            // Keepout is 0.35 radius; center is only 0.0001 mm outside the
            // minimum 0.25-mm clearance.
            location: [0.3501, 5.0],
            diameter: 0.2,
            net: None,
            plated: false,
        }];

        let violations = board_outline_drill_clearance(
            "KiCad drills",
            "KiCad Edge.Cuts",
            &outline,
            &drills,
            &[],
            0.25,
            1.0e-4,
        );

        assert!(violations.is_empty());
    }

    #[test]
    fn board_outline_drill_clearance_includes_excellon_sidecar_drills() {
        let outline = sketch(vec![square(0.0, 0.0, 10.0, 10.0)]);
        let extra_drills = vec![DrillFeature {
            location: [9.8, 5.0],
            diameter: 0.4,
            net: None,
            plated: false,
        }];

        let violations = board_outline_drill_clearance(
            "KiCad plus Excellon drills",
            "KiCad Edge.Cuts",
            &outline,
            &[],
            &extra_drills,
            0.25,
            1.0e-9,
        );

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].locations, vec![[9.8, 5.0]]);
    }

    #[test]
    fn board_outline_drill_clearance_is_orientation_invariant() {
        let outline = sketch(vec![reversed_square(0.0, 0.0, 10.0, 10.0)]);
        let drills = vec![DrillFeature {
            location: [0.4, 5.0],
            diameter: 0.4,
            net: None,
            plated: false,
        }];

        let violations = board_outline_drill_clearance(
            "KiCad drills",
            "KiCad Edge.Cuts",
            &outline,
            &drills,
            &[],
            0.25,
            1.0e-9,
        );

        assert_eq!(violations.len(), 1);
    }

    #[test]
    fn board_outline_drill_clearance_with_empty_drill_sources_is_noop() {
        let outline = sketch(vec![square(0.0, 0.0, 10.0, 10.0)]);

        let violations = board_outline_drill_clearance(
            "KiCad drills",
            "KiCad Edge.Cuts",
            &outline,
            &[],
            &[],
            0.25,
            1.0e-9,
        );

        assert!(violations.is_empty());
    }

    #[test]
    fn board_outline_drill_clearance_flags_each_hole_that_intrudes_clearance_band() {
        let outline = sketch(vec![square(0.0, 0.0, 10.0, 10.0)]);
        let board_drills = vec![
            DrillFeature {
                location: [0.4, 2.5],
                diameter: 0.4,
                net: None,
                plated: false,
            },
            DrillFeature {
                location: [5.0, 5.0],
                diameter: 0.4,
                net: None,
                plated: false,
            },
            DrillFeature {
                location: [9.6, 5.0],
                diameter: 0.4,
                net: None,
                plated: false,
            },
        ];

        let violations = board_outline_drill_clearance(
            "KiCad drills",
            "KiCad Edge.Cuts",
            &outline,
            &board_drills,
            &[],
            0.25,
            1.0e-9,
        );

        assert_eq!(violations.len(), 2);
    }

    #[test]
    fn castellation_intent_reports_plated_hole_crossing_outline() {
        let mut board = board_with_outline(square(0.0, 0.0, 10.0, 10.0));
        board.drills = vec![DrillFeature {
            location: [0.1, 5.0],
            diameter: 0.4,
            net: Some("EDGE".to_string()),
            plated: true,
        }];

        let violations = castellation_intent(&board, 1.0e-9);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "castellation-intent");
    }

    #[test]
    fn castellation_intent_allows_inset_or_non_plated_holes() {
        let mut board = board_with_outline(square(0.0, 0.0, 10.0, 10.0));
        board.drills = vec![
            DrillFeature {
                location: [1.0, 5.0],
                diameter: 0.4,
                net: Some("PTH".to_string()),
                plated: true,
            },
            DrillFeature {
                location: [0.1, 5.0],
                diameter: 0.4,
                net: None,
                plated: false,
            },
        ];

        assert!(castellation_intent(&board, 1.0e-9).is_empty());
    }

    #[test]
    fn castellation_hole_readiness_reports_undersized_edge_hole() {
        let mut board = board_with_outline(square(0.0, 0.0, 10.0, 10.0));
        board.drills = vec![DrillFeature {
            location: [0.1, 5.0],
            diameter: 0.3,
            net: Some("EDGE".to_string()),
            plated: true,
        }];

        let violations = castellation_hole_readiness(&board, 0.5, 1.0e-9);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "castellation-hole-readiness");
    }

    #[test]
    fn castellation_hole_readiness_allows_large_inset_or_non_plated_holes() {
        let mut board = board_with_outline(square(0.0, 0.0, 10.0, 10.0));
        board.drills = vec![
            DrillFeature {
                location: [0.1, 5.0],
                diameter: 0.6,
                net: Some("EDGE".to_string()),
                plated: true,
            },
            DrillFeature {
                location: [1.0, 5.0],
                diameter: 0.3,
                net: Some("PTH".to_string()),
                plated: true,
            },
            DrillFeature {
                location: [0.1, 4.0],
                diameter: 0.3,
                net: None,
                plated: false,
            },
        ];

        assert!(castellation_hole_readiness(&board, 0.5, 1.0e-9).is_empty());
    }

    #[test]
    fn drill_spacing_flags_conservative_slot_keepouts() {
        let rectangular_slots = vec![
            DrillFeature {
                location: [0.0, 0.0],
                diameter: 1.8,
                net: None,
                plated: false,
            },
            DrillFeature {
                location: [2.0, 0.0],
                diameter: 1.7,
                net: None,
                plated: false,
            },
            DrillFeature {
                location: [5.0, 0.0],
                diameter: 1.0,
                net: None,
                plated: false,
            },
        ];

        let violations = drill_spacing(&rectangular_slots, &[], 0.30);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "drill-spacing");
    }

    #[test]
    fn panelization_clearance_flags_copper_near_panel_features_and_stamp_holes() {
        let board = BoardModel {
            source: "test".to_string(),
            copper: vec![copper_disc("SIG", CopperKind::Pad, [0.12, 0.0], 0.08)],
            drills: vec![DrillFeature {
                location: [1.0, 0.0],
                diameter: 0.2,
                net: None,
                plated: false,
            }],
            board_outline: None,
            panel_features: Some(polygons_to_sketch(
                vec![line_polygon([0.0, -1.0], [0.0, 1.0], 0.05).unwrap()],
                Some(LayerMetadata {
                    name: "KiCad Panel".to_string(),
                }),
            )),
        };
        let extra_drills = vec![DrillFeature {
            location: [0.2, 0.0],
            diameter: 0.2,
            net: None,
            plated: false,
        }];

        let violations = panelization_clearance(&board, &extra_drills, 0.25, 1.0e-9);

        assert_eq!(violations.len(), 2);
    }

    #[test]
    fn panelization_clearance_flags_copper_near_tab_route_and_v_score() {
        let board = BoardModel {
            source: "test".to_string(),
            copper: vec![
                copper_disc("TAB", CopperKind::Pad, [0.0, 0.0], 0.08),
                copper_disc("VSCORE", CopperKind::Pad, [2.0, 0.0], 0.08),
            ],
            drills: Vec::new(),
            board_outline: None,
            panel_features: Some(polygons_to_sketch(
                vec![
                    line_polygon([0.0, -1.0], [0.0, 1.0], 0.05).unwrap(),
                    line_polygon([2.0, -1.0], [2.0, 1.0], 0.05).unwrap(),
                ],
                Some(LayerMetadata {
                    name: "KiCad panel features".to_string(),
                }),
            )),
        };

        let violations = panelization_clearance(&board, &[], 0.25, 1.0e-9);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "panelization-clearance");
    }

    #[test]
    fn panelization_clearance_checks_sidecar_drills_without_panel_features() {
        let board = BoardModel {
            source: "test".to_string(),
            copper: vec![copper_disc("SIG", CopperKind::Pad, [0.12, 0.0], 0.08)],
            drills: Vec::new(),
            board_outline: None,
            panel_features: None,
        };
        let sidecar_drills = vec![DrillFeature {
            location: [0.2, 0.0],
            diameter: 0.2,
            net: None,
            plated: false,
        }];

        let violations = panelization_clearance(&board, &sidecar_drills, 0.25, 1.0e-9);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "panelization-clearance");
    }

    #[test]
    fn panelization_clearance_culls_large_sparse_copper_fields() {
        let mut copper = Vec::new();
        for index in 0..800 {
            copper.push(copper_disc(
                &format!("N{index}"),
                CopperKind::Pad,
                [10.0 + (index % 40) as f64 * 5.0, (index / 40) as f64 * 5.0],
                0.08,
            ));
        }
        copper.push(copper_disc("NEAR", CopperKind::Pad, [0.12, 0.0], 0.08));
        let board = BoardModel {
            source: "test".to_string(),
            copper,
            drills: Vec::new(),
            board_outline: None,
            panel_features: Some(polygons_to_sketch(
                vec![line_polygon([0.0, -1.0], [0.0, 1.0], 0.05).unwrap()],
                Some(LayerMetadata {
                    name: "KiCad panel features".to_string(),
                }),
            )),
        };

        let start = std::time::Instant::now();
        let violations = panelization_clearance(&board, &[], 0.25, 1.0e-9);

        assert_eq!(violations.len(), 1);
        assert!(
            start.elapsed() < std::time::Duration::from_secs(2),
            "panelization clearance should avoid whole-board copper intersections"
        );
    }

    #[test]
    fn panelization_clearance_with_no_panel_blockers_is_noop() {
        let board = BoardModel {
            source: "test".to_string(),
            copper: vec![copper_disc("SIG", CopperKind::Pad, [0.10, 0.0], 0.08)],
            drills: Vec::new(),
            board_outline: None,
            panel_features: None,
        };
        let violations = panelization_clearance(&board, &[], 0.25, 1000.0);

        assert!(violations.is_empty());
    }

    #[test]
    fn drills_to_sketch_preserves_metadata() {
        let holes = vec![
            DrillFeature {
                location: [0.0, 0.0],
                diameter: 0.2,
                net: None,
                plated: false,
            },
            DrillFeature {
                location: [1.0, 1.0],
                diameter: 0.4,
                net: None,
                plated: true,
            },
        ];

        let sketch = drills_to_sketch(&holes, "test panel drills");

        assert_eq!(sketch.metadata.as_ref().unwrap().name, "test panel drills");
        assert_eq!(sketch.to_multipolygon().0.len(), 2);
    }

    #[test]
    fn registration_tolerance_flags_close_features_on_different_layers() {
        let board = BoardModel {
            source: "test".to_string(),
            copper: vec![
                copper_disc_on_layer("TOP", CopperKind::Pad, "F.Cu", [0.0, 0.0], 0.2),
                copper_disc_on_layer("BOT", CopperKind::Pad, "B.Cu", [0.3, 0.0], 0.2),
                copper_disc_on_layer("INNER", CopperKind::Pad, "In1.Cu", [2.0, 0.0], 0.2),
            ],
            drills: Vec::new(),
            board_outline: None,
            panel_features: None,
        };

        let violations = registration_tolerance(&board, 0.15, 1.0e-9);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].layers, vec!["B.Cu", "F.Cu"]);
    }

    #[test]
    fn registration_tolerance_culls_large_sparse_layer_sets() {
        let mut copper = Vec::new();
        for index in 0..400 {
            let x = (index % 20) as f64 * 5.0;
            let y = (index / 20) as f64 * 5.0;
            copper.push(copper_disc_on_layer(
                &format!("F{index}"),
                CopperKind::Pad,
                "F.Cu",
                [x, y],
                0.2,
            ));
            copper.push(copper_disc_on_layer(
                &format!("B{index}"),
                CopperKind::Pad,
                "B.Cu",
                [x + 2.0, y + 2.0],
                0.2,
            ));
        }
        copper.push(copper_disc_on_layer(
            "B_NEAR",
            CopperKind::Pad,
            "B.Cu",
            [0.3, 0.0],
            0.2,
        ));
        let board = board_with_copper(copper);

        let start = std::time::Instant::now();
        let violations = registration_tolerance(&board, 0.15, 1.0e-9);

        assert_eq!(violations.len(), 1);
        assert!(
            start.elapsed() < std::time::Duration::from_secs(2),
            "large sparse registration check should stay in the broad phase"
        );
    }

    #[test]
    fn ipc356_points_annotate_nearby_copper() {
        let mut board = BoardModel {
            source: "test".to_string(),
            copper: vec![CopperFeature {
                layer: "F.Cu".to_string(),
                net: None,
                kind: CopperKind::Pad,
                location: [1.0, 2.0],
                sketch: polygons_to_sketch(
                    vec![circle_polygon([1.0, 2.0], 0.5, 32)],
                    Some(LayerMetadata {
                        name: "feature".to_string(),
                    }),
                ),
            }],
            drills: Vec::new(),
            board_outline: None,
            panel_features: None,
        };
        let points = vec![Ipc356Point {
            net: "GND".to_string(),
            reference: Some("U1".to_string()),
            pin: Some("1".to_string()),
            location: [1.02, 2.0],
            diameter: None,
            access_side: None,
            feature_type: None,
            soldermask: None,
        }];

        apply_ipc356_nets(&mut board, &points, 0.1);

        assert_eq!(board.copper[0].net.as_deref(), Some("GND"));
        assert!(ipc356_coverage(&board, &points, 0.1).is_empty());
    }

    #[test]
    fn ipc356_points_recover_missing_drill_net_and_diameter() {
        let mut board = BoardModel {
            source: "test".to_string(),
            copper: Vec::new(),
            drills: vec![DrillFeature {
                location: [1.0, 2.0],
                diameter: 0.0,
                net: None,
                plated: true,
            }],
            board_outline: None,
            panel_features: None,
        };
        let points = vec![Ipc356Point {
            net: "PWR".to_string(),
            reference: Some("TP1".to_string()),
            pin: None,
            location: [1.01, 2.0],
            diameter: Some(0.45),
            access_side: None,
            feature_type: None,
            soldermask: None,
        }];

        apply_ipc356_nets(&mut board, &points, 0.1);

        assert_eq!(board.drills[0].net.as_deref(), Some("PWR"));
        assert_eq!(board.drills[0].diameter, 0.45);
    }

    #[test]
    fn ipc356_points_do_not_copy_far_drill_diameter() {
        let mut board = BoardModel {
            source: "test".to_string(),
            copper: Vec::new(),
            drills: vec![DrillFeature {
                location: [1.0, 2.0],
                diameter: 0.0,
                net: None,
                plated: true,
            }],
            board_outline: None,
            panel_features: None,
        };
        let points = vec![Ipc356Point {
            net: "PWR".to_string(),
            reference: Some("TP1".to_string()),
            pin: None,
            location: [10.0, 20.0],
            diameter: Some(0.45),
            access_side: None,
            feature_type: None,
            soldermask: None,
        }];

        apply_ipc356_nets(&mut board, &points, 0.1);

        assert!(board.drills[0].net.is_none());
        assert_eq!(board.drills[0].diameter, 0.0);
    }

    #[test]
    fn ipc356_annotation_culls_sparse_copper_and_drill_fields() {
        let mut board = BoardModel {
            source: "test".to_string(),
            copper: (0..2_000)
                .map(|index| {
                    copper_disc(
                        "NO_NET",
                        CopperKind::Pad,
                        [100.0 + index as f64 * 5.0, 0.0],
                        0.2,
                    )
                })
                .chain([CopperFeature {
                    layer: "F.Cu".to_string(),
                    net: None,
                    kind: CopperKind::Pad,
                    location: [1.0, 2.0],
                    sketch: polygons_to_sketch(
                        vec![circle_polygon([1.0, 2.0], 0.5, 32)],
                        Some(LayerMetadata {
                            name: "feature".to_string(),
                        }),
                    ),
                }])
                .collect(),
            drills: (0..2_000)
                .map(|index| DrillFeature {
                    location: [100.0 + index as f64 * 5.0, 10.0],
                    diameter: 0.2,
                    net: None,
                    plated: true,
                })
                .chain([DrillFeature {
                    location: [1.0, 3.0],
                    diameter: 0.0,
                    net: None,
                    plated: true,
                }])
                .collect(),
            board_outline: None,
            panel_features: None,
        };
        let points = vec![
            Ipc356Point {
                net: "GND".to_string(),
                reference: Some("U1".to_string()),
                pin: Some("1".to_string()),
                location: [1.02, 2.0],
                diameter: None,
                access_side: None,
                feature_type: None,
                soldermask: None,
            },
            Ipc356Point {
                net: "PWR".to_string(),
                reference: Some("TP1".to_string()),
                pin: None,
                location: [1.01, 3.0],
                diameter: Some(0.45),
                access_side: None,
                feature_type: None,
                soldermask: None,
            },
        ];

        let started = std::time::Instant::now();
        apply_ipc356_nets(&mut board, &points, 0.1);

        assert_eq!(
            board.copper.last().and_then(|copper| copper.net.as_deref()),
            Some("GND")
        );
        assert_eq!(
            board.drills.last().and_then(|drill| drill.net.as_deref()),
            Some("PWR")
        );
        assert_eq!(board.drills.last().map(|drill| drill.diameter), Some(0.45));
        assert!(
            started.elapsed() < std::time::Duration::from_secs(2),
            "IPC-D-356 annotation should index sparse copper and drill fields"
        );
    }

    #[test]
    fn ipc356_coverage_reports_missing_test_record_copper() {
        let board = BoardModel {
            source: "test".to_string(),
            copper: Vec::new(),
            drills: Vec::new(),
            board_outline: None,
            panel_features: None,
        };
        let points = vec![Ipc356Point {
            net: "N/C".to_string(),
            reference: Some("J1".to_string()),
            pin: Some("2".to_string()),
            location: [10.0, 20.0],
            diameter: None,
            access_side: None,
            feature_type: None,
            soldermask: None,
        }];

        let violations = ipc356_coverage(&board, &points, 0.1);

        assert_eq!(violations.len(), 1);
        assert!(violations[0].message.as_deref().unwrap().contains("J1.2"));
    }

    #[test]
    fn ipc356_coverage_culls_sparse_copper_fields() {
        let board = BoardModel {
            source: "test".to_string(),
            copper: (0..2_000)
                .map(|index| {
                    copper_disc(
                        "SIG",
                        CopperKind::Pad,
                        [100.0 + index as f64 * 5.0, 0.0],
                        0.2,
                    )
                })
                .chain([copper_disc("GND", CopperKind::Pad, [1.0, 2.0], 0.2)])
                .collect(),
            drills: Vec::new(),
            board_outline: None,
            panel_features: None,
        };
        let points = vec![Ipc356Point {
            net: "GND".to_string(),
            reference: Some("U1".to_string()),
            pin: Some("1".to_string()),
            location: [1.02, 2.0],
            diameter: None,
            access_side: None,
            feature_type: None,
            soldermask: None,
        }];

        let started = std::time::Instant::now();
        let violations = ipc356_coverage(&board, &points, 0.1);

        assert!(violations.is_empty());
        assert!(
            started.elapsed() < std::time::Duration::from_secs(2),
            "IPC-D-356 coverage should cull sparse copper fields by point index"
        );
    }

    #[test]
    fn ipc356_drill_diameter_reports_conflicting_drill_table_data() {
        let board = BoardModel {
            source: "test".to_string(),
            copper: Vec::new(),
            drills: vec![DrillFeature {
                location: [1.0, 2.0],
                diameter: 0.30,
                net: Some("GND".to_string()),
                plated: true,
            }],
            board_outline: None,
            panel_features: None,
        };
        let points = vec![Ipc356Point {
            net: "GND".to_string(),
            reference: Some("V1".to_string()),
            pin: None,
            location: [1.01, 2.0],
            diameter: Some(0.50),
            access_side: None,
            feature_type: None,
            soldermask: None,
        }];

        let violations = ipc356_drill_diameter(&board, &points, 0.05);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "ipc356-drill-diameter");
    }

    #[test]
    fn ipc356_drill_diameter_allows_matching_drills_within_tolerance() {
        let board = BoardModel {
            source: "test".to_string(),
            copper: Vec::new(),
            drills: vec![DrillFeature {
                location: [1.0, 2.0],
                diameter: 0.30,
                net: Some("GND".to_string()),
                plated: true,
            }],
            board_outline: None,
            panel_features: None,
        };
        let points = vec![Ipc356Point {
            net: "GND".to_string(),
            reference: Some("V1".to_string()),
            pin: None,
            location: [1.01, 2.0],
            diameter: Some(0.31),
            access_side: None,
            feature_type: None,
            soldermask: None,
        }];

        assert!(ipc356_drill_diameter(&board, &points, 0.05).is_empty());
    }

    #[test]
    fn ipc356_drill_diameter_culls_sparse_drill_fields() {
        let board = BoardModel {
            source: "test".to_string(),
            copper: Vec::new(),
            drills: (0..2_000)
                .map(|index| DrillFeature {
                    location: [100.0 + index as f64 * 5.0, 0.0],
                    diameter: 0.30,
                    net: Some("GND".to_string()),
                    plated: true,
                })
                .chain([DrillFeature {
                    location: [1.0, 2.0],
                    diameter: 0.30,
                    net: Some("GND".to_string()),
                    plated: true,
                }])
                .collect(),
            board_outline: None,
            panel_features: None,
        };
        let points = vec![Ipc356Point {
            net: "GND".to_string(),
            reference: Some("V1".to_string()),
            pin: None,
            location: [1.01, 2.0],
            diameter: Some(0.50),
            access_side: None,
            feature_type: None,
            soldermask: None,
        }];

        let started = std::time::Instant::now();
        let violations = ipc356_drill_diameter(&board, &points, 0.05);

        assert_eq!(violations.len(), 1);
        assert!(
            started.elapsed() < std::time::Duration::from_secs(2),
            "IPC-D-356 drill diameter comparison should index sparse drill fields"
        );
    }

    fn feature(net: &str, location: [f64; 2]) -> CopperFeature {
        copper_disc(net, CopperKind::Pad, location, 0.5)
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

    fn board_with_outline(outline: Polygon<f64>) -> BoardModel {
        BoardModel {
            source: "test".to_string(),
            copper: Vec::new(),
            drills: Vec::new(),
            board_outline: Some(sketch(vec![outline])),
            panel_features: None,
        }
    }

    fn sketch(polygons: Vec<Polygon<f64>>) -> PcbSketch {
        polygons_to_sketch(
            polygons,
            Some(LayerMetadata {
                name: "outline".to_string(),
            }),
        )
    }

    fn square(min_x: f64, min_y: f64, max_x: f64, max_y: f64) -> Polygon<f64> {
        Polygon::new(
            LineString::from(vec![
                Coord { x: min_x, y: min_y },
                Coord { x: max_x, y: min_y },
                Coord { x: max_x, y: max_y },
                Coord { x: min_x, y: max_y },
                Coord { x: min_x, y: min_y },
            ]),
            Vec::new(),
        )
    }

    fn reversed_square(min_x: f64, min_y: f64, max_x: f64, max_y: f64) -> Polygon<f64> {
        Polygon::new(
            LineString::from(vec![
                Coord { x: min_x, y: min_y },
                Coord { x: min_x, y: max_y },
                Coord { x: max_x, y: max_y },
                Coord { x: max_x, y: min_y },
                Coord { x: min_x, y: min_y },
            ]),
            Vec::new(),
        )
    }

    fn copper_disc(net: &str, kind: CopperKind, location: [f64; 2], radius: f64) -> CopperFeature {
        copper_disc_on_layer(net, kind, "F.Cu", location, radius)
    }

    fn copper_disc_on_layer(
        net: &str,
        kind: CopperKind,
        layer: &str,
        location: [f64; 2],
        radius: f64,
    ) -> CopperFeature {
        CopperFeature {
            layer: layer.to_string(),
            net: Some(net.to_string()),
            kind,
            location,
            sketch: polygons_to_sketch(
                vec![circle_polygon(location, radius, 32)],
                Some(LayerMetadata {
                    name: "feature".to_string(),
                }),
            ),
        }
    }

    fn unnetted_copper_disc_on_layer(
        layer: &str,
        location: [f64; 2],
        radius: f64,
    ) -> CopperFeature {
        CopperFeature {
            layer: layer.to_string(),
            net: None,
            kind: CopperKind::Pad,
            location,
            sketch: polygons_to_sketch(
                vec![circle_polygon(location, radius, 32)],
                Some(LayerMetadata {
                    name: "fiducial".to_string(),
                }),
            ),
        }
    }

    fn copper_line(
        net: &str,
        kind: CopperKind,
        start: [f64; 2],
        end: [f64; 2],
        width: f64,
    ) -> CopperFeature {
        copper_line_on_layer(net, kind, "F.Cu", start, end, width)
    }

    fn copper_line_on_layer(
        net: &str,
        kind: CopperKind,
        layer: &str,
        start: [f64; 2],
        end: [f64; 2],
        width: f64,
    ) -> CopperFeature {
        CopperFeature {
            layer: layer.to_string(),
            net: Some(net.to_string()),
            kind,
            location: [(start[0] + end[0]) / 2.0, (start[1] + end[1]) / 2.0],
            sketch: polygons_to_sketch(
                vec![line_polygon(start, end, width).unwrap()],
                Some(LayerMetadata {
                    name: "feature".to_string(),
                }),
            ),
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
                vec![square(min_x, min_y, max_x, max_y)],
                Some(LayerMetadata {
                    name: "feature".to_string(),
                }),
            ),
        }
    }

    fn dense_pad_cluster() -> Vec<CopperFeature> {
        let mut copper = Vec::new();
        for x in 0..4 {
            for y in 0..4 {
                copper.push(copper_disc(
                    &format!("BGA_{x}_{y}"),
                    CopperKind::Pad,
                    [x as f64 * 0.5, y as f64 * 0.5],
                    0.12,
                ));
            }
        }
        copper
    }
}
