//! Drill, hole, slot, and castellation readiness checks.
//!
//! This module owns checks where the primary object is a KiCad, Excellon, or
//! IPC-D-356 drill record. Keeping them separate from board-wide electrical
//! checks makes mechanical fabrication rules easier to find and extend.
//!
//! Reliability note: slot handling and drill keepouts use conservative circular
//! or rectangular proxies when source data is incomplete. Verify suspect annular
//! ring, slot, and clearance findings against the drill drawing and CAM import.

use csgrs::csg::CSG;
use csgrs::sketch::Profile;
use geo::BoundingRect;
use hyperlimit::{PredicatePolicy, compare_reals_with_policy};

use crate::geometry::{
    RuleGeometryProvenance, SourceGridFacts, SourceUnit, multipolygon_area_scalar,
    multipolygon_to_shapes_scalar,
};
use crate::ipc356::Ipc356Point;
use crate::kicad::{BoardModel, CopperFeature, CopperKind, DrillFeature};
use crate::report::{Severity, Violation};
use crate::{LayerMetadata, PcbSketch, PcbSketchExt, Scalar};

use super::outline::axis_aligned_outline_rect_with_grid;
use super::spatial::{CopperSpatialIndex, DrillSpatialIndex, PointSpatialIndex};
use super::{difference_for_check, intersection_for_check, union_for_check};

/// Review plated drill land margin using an area-equivalent copper radius.
///
/// IPC-2221B and IPC-6012D treat annular ring as a registration-sensitive
/// finished-hole-to-land margin. Parsed KiCad pad geometry can be rectangular,
/// oval, or custom, so this readiness check uses an area-equivalent circular
/// radius as a conservative, shape-agnostic proxy and reports suspect rings for
/// drill drawing/CAM verification rather than claiming exact pad-stack
/// containment.
pub fn annular_ring(
    board: &BoardModel,
    minimum_ring: &Scalar,
    selected_layers: &[String],
) -> Vec<Violation> {
    let mut violations = Vec::new();

    for drill in &board.drills {
        if !drill.plated {
            continue;
        }

        let Some(nearest) = nearest_matching_copper(board, drill, selected_layers) else {
            continue;
        };

        // KiCad pad geometry can be rectangular, oval, or custom. For the first
        // pass we use an area-equivalent circular radius so annular-ring checks
        // remain shape-agnostic. IPC-2221B and IPC-6012D both treat annular ring
        // as a finished-hole-to-land registration margin; exact containment can
        // be tightened once pad stack drill spans are modeled.
        let Some(copper_radius) = equivalent_radius_scalar(&nearest.sketch) else {
            continue;
        };
        let drill_diameter = drill.diameter.clone();
        let Ok(drill_radius) = drill_diameter / crate::scalar::scalar("2") else {
            continue;
        };
        let ring = copper_radius - drill_radius;
        if &ring < minimum_ring {
            violations.push(Violation::new(
                "annular-ring-readiness",
                Severity::Error,
                vec![nearest.layer.clone()],
                None,
                Vec::new(),
                vec![drill.location_f64_compatibility_required()],
                Some(format!(
                    "annular ring {ring:.6} is below minimum {minimum_ring:.6}"
                )),
            ));
        }
    }

    violations
}

/// Review nominal annular rings against a registration-tolerance budget.
///
/// IPC-6012D distinguishes nominal finished geometry from manufacturing
/// acceptance, and IPC-2221B frames annular-ring clearance as a design margin.
/// This check reports holes that pass nominally but fail after subtracting the
/// configured registration tolerance, keeping tolerance-sensitive drill/pad
/// stacks visible before fabrication release.
pub fn annular_ring_tolerance(
    board: &BoardModel,
    minimum_ring: &Scalar,
    registration_tolerance: &Scalar,
    selected_layers: &[String],
) -> Vec<Violation> {
    let mut violations = Vec::new();

    for drill in &board.drills {
        if !drill.plated {
            continue;
        }

        let Some(nearest) = nearest_matching_copper(board, drill, selected_layers) else {
            continue;
        };

        let Some(copper_radius) = equivalent_radius_scalar(&nearest.sketch) else {
            continue;
        };
        let drill_diameter = drill.diameter.clone();
        let Ok(drill_radius) = drill_diameter / crate::scalar::scalar("2") else {
            continue;
        };
        let nominal_ring = copper_radius - drill_radius;
        let worst_case_ring = &nominal_ring - registration_tolerance;
        if &nominal_ring >= minimum_ring && &worst_case_ring < minimum_ring {
            violations.push(Violation::new(
                "annular-ring-tolerance",
                Severity::Warning,
                vec![nearest.layer.clone()],
                None,
                Vec::new(),
                vec![drill.location_f64_compatibility_required()],
                Some(format!(
                    "nominal annular ring {nominal_ring:.6} passes minimum {minimum_ring:.6}, but worst-case ring {worst_case_ring:.6} after tolerance {registration_tolerance:.6} does not"
                )),
            ));
        }
    }

    violations
}

/// Run the `plating_intent` design-readiness check or report helper.
///
/// A deterministic grid proposes nearby copper by parsed location, then exact
/// center distance, net, and
/// pad/via kind remain the readiness decision.
pub fn plating_intent(
    board: &BoardModel,
    selected_layers: &[String],
    tolerance: &Scalar,
) -> Vec<Violation> {
    let copper_features = selected_copper_features(board, selected_layers);
    let broad_phase_tolerance = scalar_broad_phase_radius(tolerance);
    let copper_index = CopperSpatialIndex::new(&copper_features, broad_phase_tolerance);
    let mut violations = Vec::new();
    let mut candidate_hits = 0_usize;

    for drill in &board.drills {
        if drill.plated {
            let candidates = copper_index.all_layers_near_circle(
                drill.location_f64_compatibility_required(),
                broad_phase_tolerance,
            );
            candidate_hits += candidates.len();
            if has_plated_drill_copper(drill, &copper_features, &candidates, tolerance) {
                continue;
            }

            violations.push(Violation::new(
                "plating-intent",
                Severity::Warning,
                vec!["KiCad drills".to_string()],
                None,
                Vec::new(),
                vec![drill.location_f64_compatibility_required()],
                Some("plated drill has no nearby same-net pad or via copper".to_string()),
            ));
        } else {
            let drill_diameter = drill.diameter.clone();
            let Ok(drill_radius) = drill_diameter / crate::scalar::scalar("2") else {
                continue;
            };
            let search_radius = drill_radius + tolerance;
            let broad_phase_radius = scalar_broad_phase_radius(&search_radius);
            let candidates = copper_index.all_layers_near_circle(
                drill.location_f64_compatibility_required(),
                broad_phase_radius,
            );
            candidate_hits += candidates.len();
            if !has_nearby_copper(
                &drill.location,
                &copper_features,
                &candidates,
                &search_radius,
            ) {
                continue;
            }

            violations.push(Violation::new(
                "plating-intent",
                Severity::Warning,
                vec!["KiCad NPTH drills".to_string()],
                None,
                Vec::new(),
                vec![drill.location_f64_compatibility_required()],
                Some(
                    "non-plated drill has nearby copper that may imply plated-hole intent"
                        .to_string(),
                ),
            ));
        }
    }

    log::trace!(
        "plating intent: source={} drills={} copper_features={} copper_buckets={} candidate_hits={} selected_layers={} tolerance={tolerance:.6} violations={}",
        board.source,
        board.drills.len(),
        copper_features.len(),
        copper_index.bucket_count(),
        candidate_hits,
        selected_layers.len(),
        violations.len()
    );

    violations
}

/// Review small non-plated mechanical drills as likely routed-slot risks.
///
/// Exact slot geometry is not fully preserved by every input path yet, so this
/// check treats small NPTH mechanical drill diameters as a cutter-capability
/// proxy. The finding is intentionally a warning: IPC-2221B mechanical-outline
/// guidance and common fabricator DFM rules require the routed cutter width to
/// be explicit in drawings or drill/rout files.
pub fn routed_slot_readiness(board: &BoardModel, minimum_route_width: &Scalar) -> Vec<Violation> {
    board
        .drills
        .iter()
        .filter(|drill| {
            !drill.plated
                && drill.diameter > Scalar::zero()
                && &drill.diameter < minimum_route_width
        })
        .map(|drill| {
            Violation::new(
                "routed-slot-readiness",
                Severity::Warning,
                vec!["KiCad NPTH drills".to_string()],
                None,
                Vec::new(),
                vec![drill.location_f64_compatibility_required()],
                Some(format!(
                    "non-plated mechanical drill diameter {:.6} is below minimum route width {:.6}; review routed slot or cutter capability",
                    drill.diameter, minimum_route_width
                )),
            )
        })
        .collect()
}

/// Review drill keepouts against selected copper.
///
/// Drill openings are modeled as circular keepouts with the configured
/// clearance, then checked against KiCad copper. The copper grid is a broad
/// phase; exact
/// CSG intersection still decides violations. Slot-like drill records remain
/// conservative circular proxies until exact routed-slot geometry is preserved.
pub fn drill_to_copper_clearance(
    board: &BoardModel,
    extra_drills: &[DrillFeature],
    clearance: &Scalar,
    selected_layers: &[String],
    min_area: &Scalar,
) -> Vec<Violation> {
    drill_to_copper_clearance_with_grid(
        board,
        extra_drills,
        SourceGridFacts::PRIMITIVE_FLOAT_EDGE,
        clearance,
        selected_layers,
        min_area,
    )
}

/// Review drill keepouts against selected copper with retained extra-drill grid facts.
///
/// KiCad board drills are treated as KiCad millimeter compatibility edges,
/// while sidecar drills carry the caller-provided source grid. The grid only
/// selects exact comparison provenance for broad-phase rejection; CSG overlap
/// still decides surviving clearance violations.
pub fn drill_to_copper_clearance_with_grid(
    board: &BoardModel,
    extra_drills: &[DrillFeature],
    extra_drill_grid: SourceGridFacts,
    clearance: &Scalar,
    selected_layers: &[String],
    min_area: &Scalar,
) -> Vec<Violation> {
    let mut drills = board
        .drills
        .iter()
        .cloned()
        .map(|drill| {
            (
                drill,
                SourceGridFacts::primitive_float_edge(SourceUnit::KiCadMillimeter),
            )
        })
        .collect::<Vec<_>>();
    drills.extend(
        extra_drills
            .iter()
            .cloned()
            .map(|drill| (drill, extra_drill_grid)),
    );
    let copper_features = selected_copper_features(board, selected_layers);
    let maximum_keepout_radius = drills
        .iter()
        .filter_map(|(drill, _)| drill_radius_with_clearance(drill, clearance))
        .fold(None, |maximum: Option<Scalar>, radius| {
            Some(match maximum {
                Some(maximum) if maximum >= radius => maximum,
                _ => radius,
            })
        })
        .map_or(0.0, |radius| scalar_broad_phase_radius(&radius));
    // Clearance is still decided by exact geometry below. This grid is only the
    // broad phase, reducing sparse drill/copper fields before CSG intersection.
    let copper_index = CopperSpatialIndex::new(&copper_features, maximum_keepout_radius);
    log::trace!(
        "drill-to-copper clearance: source={} drills={} copper_features={} spatial_buckets={} clearance={clearance:.6}",
        board.source,
        drills.len(),
        copper_features.len(),
        copper_index.bucket_count()
    );
    let mut violations = Vec::new();
    let mut candidate_pairs = 0_usize;
    let mut exact_intersections = 0_usize;

    for (drill, drill_grid) in drills {
        let Some(keepout_radius_scalar) = drill_radius_with_clearance(&drill, clearance) else {
            continue;
        };
        let keepout_radius = scalar_broad_phase_radius(&keepout_radius_scalar);
        let mut keepout = None;

        for candidate_index in copper_index
            .all_layers_near_circle(drill.location_f64_compatibility_required(), keepout_radius)
        {
            candidate_pairs += 1;
            let copper = copper_features[candidate_index];
            if drill.plated && drill.net.is_some() && drill.net == copper.net {
                continue;
            }
            if !copper_may_touch_drill_keepout_with_grid(
                copper,
                drill.location_f64_compatibility_required(),
                keepout_radius,
                drill_grid,
            ) {
                continue;
            }

            let Some(exact_keepout) =
                drill_keepout_sketch(&drill, keepout_radius_scalar.clone(), "drill keepout")
            else {
                continue;
            };
            let keepout = keepout.get_or_insert(exact_keepout);
            exact_intersections += 1;
            let overlap = match intersection_for_check(
                keepout,
                &copper.sketch,
                "drill-to-copper-clearance",
                vec![copper.layer.clone()],
            ) {
                Ok(overlap) => overlap,
                Err(uncertainty) => return vec![*uncertainty],
            };
            let shapes = multipolygon_to_shapes_scalar(&overlap.to_multipolygon(), min_area);
            if shapes.is_empty() {
                continue;
            }

            violations.push(Violation::new(
                "drill-to-copper-clearance",
                Severity::Error,
                vec![copper.layer.clone()],
                None,
                shapes,
                vec![drill.location_f64_compatibility_required()],
                Some(format!(
                    "drill keepout with clearance {clearance} intersects copper"
                )),
            ));
        }
    }

    log::trace!(
        "drill-to-copper clearance: source={} candidate_pairs={} exact_intersections={} violations={}",
        board.source,
        candidate_pairs,
        exact_intersections,
        violations.len()
    );
    debug_assert!(exact_intersections <= candidate_pairs);

    violations
}

fn copper_may_touch_drill_keepout_with_grid(
    copper: &CopperFeature,
    drill_location: [f64; 2],
    keepout_radius: f64,
    grid: SourceGridFacts,
) -> bool {
    let Some(bounds) = copper.sketch.geometry().bounding_rect() else {
        return true;
    };

    // The drill keepout is circular, so its broad-phase box is just the drill
    // center expanded by radius. Exact polygon intersection still decides
    // surviving candidates; the box test only rejects impossible contacts.
    //
    // This is a broad-phase rejection that can skip CSG, so the comparisons
    // are exact lifted predicates with source-grid provenance. The grid is
    // non-certifying metadata: CSG overlap remains the clearance certificate
    // for candidates.
    exact_le_with_grid(drill_location[0] - keepout_radius, bounds.max().x, grid)
        && exact_ge_with_grid(drill_location[0] + keepout_radius, bounds.min().x, grid)
        && exact_le_with_grid(drill_location[1] - keepout_radius, bounds.max().y, grid)
        && exact_ge_with_grid(drill_location[1] + keepout_radius, bounds.min().y, grid)
}

fn exact_ge_with_grid(left: f64, right: f64, grid: SourceGridFacts) -> bool {
    exact_cmp_with_grid(left, right, grid)
        .is_some_and(|ordering| ordering != std::cmp::Ordering::Less)
}

fn exact_le_with_grid(left: f64, right: f64, grid: SourceGridFacts) -> bool {
    exact_cmp_with_grid(left, right, grid)
        .is_some_and(|ordering| ordering != std::cmp::Ordering::Greater)
}

fn exact_cmp_with_grid(left: f64, right: f64, grid: SourceGridFacts) -> Option<std::cmp::Ordering> {
    let provenance = RuleGeometryProvenance::new("drill-copper-broad-phase", grid);
    let left = provenance.lift_f64(left)?;
    let right = provenance.lift_f64(right)?;
    compare_reals_with_policy(&left, &right, PredicatePolicy::STRICT).value()
}

/// Review edge-to-edge spacing between KiCad and sidecar drills.
///
/// Drill centers use the shared grid broad phase before exact center and
/// edge-gap math. This keeps sparse drill tables and
/// Excellon sidecars bounded while still reporting exact edge-spacing values.
pub fn drill_spacing(
    board_drills: &[DrillFeature],
    extra_drills: &[DrillFeature],
    clearance: &Scalar,
) -> Vec<Violation> {
    let mut drills = board_drills.to_vec();
    drills.extend_from_slice(extra_drills);
    let broad_phase_clearance = scalar_broad_phase_radius(clearance);
    let drill_index = DrillSpatialIndex::new(&drills, broad_phase_clearance);
    log::trace!(
        "drill spacing: drills={} spatial_buckets={} clearance={clearance:.6}",
        drills.len(),
        drill_index.bucket_count()
    );
    let mut violations = Vec::new();
    let mut candidate_pairs = 0_usize;
    let mut exact_pairs = 0_usize;

    for left_index in 0..drills.len() {
        let left = &drills[left_index];
        for right_index in
            drill_index.later_candidates_within_spacing(left_index, broad_phase_clearance)
        {
            candidate_pairs += 1;
            let right = &drills[right_index];
            let Some(center_distance) = exact_distance_scalar(&left.location, &right.location)
            else {
                continue;
            };
            let left_diameter = left.diameter.clone();
            let right_diameter = right.diameter.clone();
            let Ok(left_radius) = left_diameter / crate::scalar::scalar("2") else {
                continue;
            };
            let Ok(right_radius) = right_diameter / crate::scalar::scalar("2") else {
                continue;
            };
            let edge_gap = center_distance - left_radius - right_radius;
            exact_pairs += 1;
            if &edge_gap >= clearance {
                continue;
            }

            violations.push(Violation::new(
                "drill-spacing",
                Severity::Error,
                vec!["drills".to_string()],
                None,
                Vec::new(),
                vec![
                    left.location_f64_compatibility_required(),
                    right.location_f64_compatibility_required(),
                ],
                Some(format!(
                    "drill edge spacing {edge_gap:.6} is below clearance {clearance:.6}"
                )),
            ));
        }
    }

    log::trace!(
        "drill spacing: candidate_pairs={} exact_pairs={} violations={}",
        candidate_pairs,
        exact_pairs,
        violations.len()
    );
    debug_assert!(exact_pairs <= candidate_pairs);

    violations
}

/// Run the `board_outline_drill_clearance` design-readiness check or report helper.
pub fn board_outline_drill_clearance(
    drill_source: &str,
    outline_name: &str,
    outline: &PcbSketch,
    board_drills: &[DrillFeature],
    extra_drills: &[DrillFeature],
    clearance: &Scalar,
    min_area: &Scalar,
) -> Vec<Violation> {
    board_outline_drill_clearance_with_grid(
        drill_source,
        outline_name,
        outline,
        board_drills,
        extra_drills,
        clearance,
        min_area,
        SourceGridFacts::PRIMITIVE_FLOAT_EDGE,
    )
}

/// Run board-outline drill clearance with retained source-grid facts.
///
/// The grid facts are advisory scheduling metadata for exact rectangle
/// predicates. They do not certify the clearance outcome by themselves: CSG
/// difference still handles boundary candidates and non-rectangular outlines.
#[allow(clippy::too_many_arguments)] // Preserve the public check API's named input channels.
pub fn board_outline_drill_clearance_with_grid(
    drill_source: &str,
    outline_name: &str,
    outline: &PcbSketch,
    board_drills: &[DrillFeature],
    extra_drills: &[DrillFeature],
    clearance: &Scalar,
    min_area: &Scalar,
    grid: SourceGridFacts,
) -> Vec<Violation> {
    let mut drills = board_drills.to_vec();
    drills.extend_from_slice(extra_drills);
    let mut violations = Vec::new();
    let outline_rect = axis_aligned_outline_rect_with_grid(outline, grid);
    let mut skipped_rect_inside = 0_usize;
    let mut exact_difference_count = 0_usize;

    for drill in drills {
        if outline_rect.as_ref().is_some_and(|rect| {
            drill_keepout_inside_rect_scalar_with_grid(&drill, rect, clearance, grid)
        }) {
            skipped_rect_inside += 1;
            continue;
        }

        let Some(keepout_radius) = drill_radius_with_clearance(&drill, clearance) else {
            continue;
        };
        let Some(keepout) = drill_keepout_sketch(&drill, keepout_radius, "drill edge keepout")
        else {
            continue;
        };
        exact_difference_count += 1;
        let outside_outline = match difference_for_check(
            &keepout,
            outline,
            "board-outline-drill-clearance",
            vec![drill_source.to_string(), outline_name.to_string()],
        ) {
            Ok(outside_outline) => outside_outline,
            Err(uncertainty) => return vec![*uncertainty],
        };
        let shapes = multipolygon_to_shapes_scalar(&outside_outline.to_multipolygon(), min_area);
        if shapes.is_empty() {
            continue;
        }

        violations.push(Violation::new(
            "board-outline-drill-clearance",
            Severity::Error,
            vec![drill_source.to_string(), outline_name.to_string()],
            None,
            shapes,
            vec![drill.location_f64_compatibility_required()],
            Some(format!(
                "drill edge is within board outline clearance {clearance}"
            )),
        ));
    }

    log::trace!(
        "board-outline drill clearance: drill_source={drill_source} outline={outline_name} drills={} outline_fast_path={} skipped_rect_inside={} exact_difference_checks={} violations={} clearance={clearance:.6} min_area={min_area:.9}",
        board_drills.len() + extra_drills.len(),
        outline_rect.is_some(),
        skipped_rect_inside,
        exact_difference_count,
        violations.len()
    );

    violations
}

/// Review plated holes that cross the board outline for castellation intent.
///
/// IPC-6012D and IPC-2221B make plated holes and edge features fabrication
/// intent, not decoration. This check subtracts the parsed board outline from
/// each plated hole and reports exact outside-outline geometry so half-hole,
/// plated-edge, or accidental-outline-crossing cases are visible before
/// release.
pub fn castellation_intent(board: &BoardModel, min_area: &Scalar) -> Vec<Violation> {
    let Some(outline) = &board.board_outline else {
        return Vec::new();
    };
    let mut violations = Vec::new();
    let mut plated_holes = 0_usize;
    let mut exact_difference_count = 0_usize;

    for drill in &board.drills {
        if !drill.plated {
            continue;
        }
        plated_holes += 1;

        let Ok(drill_radius) = drill.diameter.clone() / crate::scalar::scalar("2") else {
            continue;
        };
        let Some(hole) = drill_keepout_sketch(drill, drill_radius, "plated drill hole") else {
            continue;
        };
        exact_difference_count += 1;
        let outside_outline = match difference_for_check(
            &hole,
            outline,
            "castellation-intent",
            vec![board.source.clone(), "KiCad Edge.Cuts".into()],
        ) {
            Ok(outside) => outside,
            Err(uncertainty) => return vec![*uncertainty],
        };
        let shapes = multipolygon_to_shapes_scalar(&outside_outline.to_multipolygon(), min_area);
        if shapes.is_empty() {
            continue;
        }

        violations.push(Violation::new(
            "castellation-intent",
            Severity::Warning,
            vec![board.source.clone(), "KiCad Edge.Cuts".to_string()],
            None,
            shapes,
            vec![drill.location_f64_compatibility_required()],
            Some(
                "plated drill hole crosses the board outline; confirm castellation or plated-edge intent"
                    .to_string(),
            ),
        ));
    }

    log::trace!(
        "castellation intent: source={} plated_holes={} exact_difference_checks={} violations={} min_area={min_area:#.9}",
        board.source,
        plated_holes,
        exact_difference_count,
        violations.len()
    );
    debug_assert!(exact_difference_count <= plated_holes);

    violations
}

/// Review undersized plated edge holes that look like castellations.
///
/// IPC-6012D classifies plated-hole workmanship and acceptance separately from
/// design intent, while fabricators often set minimum half-hole diameter and
/// edge breakout limits. This check reports exact outside-outline geometry for
/// plated holes below the configured castellation diameter so routed-edge
/// plating capability can be reviewed before release.
pub fn castellation_hole_readiness(
    board: &BoardModel,
    minimum_diameter: &Scalar,
    min_area: &Scalar,
) -> Vec<Violation> {
    let Some(outline) = &board.board_outline else {
        return Vec::new();
    };
    let mut violations = Vec::new();
    let mut undersized_plated_holes = 0_usize;
    let mut exact_difference_count = 0_usize;

    for drill in &board.drills {
        if !drill.plated || &drill.diameter >= minimum_diameter {
            continue;
        }
        undersized_plated_holes += 1;

        let Ok(drill_radius) = drill.diameter.clone() / crate::scalar::scalar("2") else {
            continue;
        };
        let Some(hole) = drill_keepout_sketch(drill, drill_radius, "plated drill hole") else {
            continue;
        };
        exact_difference_count += 1;
        let outside_outline = match difference_for_check(
            &hole,
            outline,
            "castellation-hole-readiness",
            vec![board.source.clone(), "KiCad Edge.Cuts".into()],
        ) {
            Ok(outside) => outside,
            Err(uncertainty) => return vec![*uncertainty],
        };
        let shapes = multipolygon_to_shapes_scalar(&outside_outline.to_multipolygon(), min_area);
        if shapes.is_empty() {
            continue;
        }

        violations.push(Violation::new(
            "castellation-hole-readiness",
            Severity::Warning,
            vec![board.source.clone(), "KiCad Edge.Cuts".to_string()],
            None,
            shapes,
            vec![drill.location_f64_compatibility_required()],
            Some(format!(
                "plated drill crossing the board outline has diameter {:.6} below minimum castellation diameter {:.6}",
                drill.diameter, minimum_diameter
            )),
        ));
    }

    log::trace!(
        "castellation hole readiness: source={} undersized_plated_holes={} exact_difference_checks={} violations={} minimum_diameter={minimum_diameter:.6} min_area={min_area:.9}",
        board.source,
        undersized_plated_holes,
        exact_difference_count,
        violations.len()
    );
    debug_assert!(exact_difference_count <= undersized_plated_holes);

    violations
}

/// Run the `drill_aspect_ratio` design-readiness check or report helper.
pub fn drill_aspect_ratio(
    source: &str,
    drills: &[DrillFeature],
    board_thickness: &Scalar,
    max_aspect_ratio: &Scalar,
) -> Vec<Violation> {
    let mut violations = Vec::new();

    for drill in drills {
        let diameter = drill.diameter.clone();
        if diameter <= Scalar::zero() {
            violations.push(Violation::new(
                "drill-aspect-ratio",
                Severity::Warning,
                vec![source.to_string()],
                None,
                Vec::new(),
                vec![drill.location_f64_compatibility_required()],
                Some("drill diameter is not positive, so aspect ratio is undefined".to_string()),
            ));
            continue;
        }

        let Ok(aspect_ratio) = board_thickness / &diameter else {
            continue;
        };
        if &aspect_ratio <= max_aspect_ratio {
            continue;
        }

        violations.push(Violation::new(
            "drill-aspect-ratio",
            Severity::Warning,
            vec![source.to_string()],
            None,
            Vec::new(),
            vec![drill.location_f64_compatibility_required()],
            Some(format!(
                "drill aspect ratio {aspect_ratio:.3} exceeds maximum {max_aspect_ratio:.3} for board thickness {board_thickness:.3}"
            )),
        ));
    }

    violations
}

/// Cross-check KiCad, Excellon, and IPC-D-356 drill diameters.
///
/// IPC-D-356 can carry test-point drill evidence while Excellon carries
/// fabrication drill hits. Cross-source center matching uses drill/point grids
/// before exact diameter conflict review, keeping sparse sidecars bounded while
/// still requiring a real tolerance-exceeding diameter mismatch before a
/// warning is emitted.
pub fn drill_table_consistency(
    board_drills: &[DrillFeature],
    extra_drills: &[DrillFeature],
    ipc356_points: &[Ipc356Point],
    tolerance: &Scalar,
) -> Vec<Violation> {
    let mut violations = Vec::new();
    let broad_phase_tolerance = scalar_broad_phase_radius(tolerance);
    let extra_drill_index = DrillSpatialIndex::new(extra_drills, broad_phase_tolerance);
    let ipc_point_index = PointSpatialIndex::new(
        ipc356_points
            .iter()
            .filter_map(Ipc356Point::location_f64_compatibility),
        broad_phase_tolerance,
    );
    log::trace!(
        "drill table consistency: board_drills={} extra_drills={} ipc356_points={} extra_drill_buckets={} ipc356_buckets={} tolerance={tolerance:.6}",
        board_drills.len(),
        extra_drills.len(),
        ipc356_points.len(),
        extra_drill_index.bucket_count(),
        ipc_point_index.bucket_count()
    );
    let mut board_extra_candidates = 0_usize;
    let mut board_extra_exact = 0_usize;
    let mut extra_ipc_candidates = 0_usize;
    let mut extra_ipc_exact = 0_usize;

    for board_drill in board_drills {
        for extra_index in extra_drill_index.candidate_centers_near(
            board_drill.location_f64_compatibility_required(),
            broad_phase_tolerance,
        ) {
            board_extra_candidates += 1;
            let extra_drill = &extra_drills[extra_index];
            if !exact_distance_scalar(&board_drill.location, &extra_drill.location)
                .is_some_and(|distance| &distance <= tolerance)
            {
                continue;
            }
            board_extra_exact += 1;
            let board_diameter = board_drill.diameter.clone();
            let extra_diameter = extra_drill.diameter.clone();
            if !diameters_conflict(&board_diameter, &extra_diameter, tolerance) {
                continue;
            }

            violations.push(drill_table_violation(
                "KiCad drills",
                &board_diameter,
                "Excellon drills",
                &extra_diameter,
                vec![
                    board_drill.location_f64_compatibility_required(),
                    extra_drill.location_f64_compatibility_required(),
                ],
            ));
        }
    }

    for extra_drill in extra_drills {
        for point_index in ipc_point_index.candidate_centers_near(
            extra_drill.location_f64_compatibility_required(),
            broad_phase_tolerance,
        ) {
            extra_ipc_candidates += 1;
            let point = &ipc356_points[point_index];
            let Some(point_location) = point.location_f64_compatibility() else {
                continue;
            };
            if !exact_distance_scalar(&extra_drill.location, &point.location)
                .is_some_and(|distance| &distance <= tolerance)
            {
                continue;
            }
            let Some(ipc_diameter) = &point.diameter else {
                continue;
            };
            extra_ipc_exact += 1;
            let extra_diameter = extra_drill.diameter.clone();
            if !diameters_conflict(&extra_diameter, ipc_diameter, tolerance) {
                continue;
            }

            violations.push(drill_table_violation(
                "Excellon drills",
                &extra_diameter,
                "IPC-D-356 drills",
                ipc_diameter,
                vec![
                    extra_drill.location_f64_compatibility_required(),
                    point_location,
                ],
            ));
        }
    }

    log::trace!(
        "drill table consistency: board_extra_candidates={} board_extra_exact={} extra_ipc_candidates={} extra_ipc_exact={} violations={}",
        board_extra_candidates,
        board_extra_exact,
        extra_ipc_candidates,
        extra_ipc_exact,
        violations.len()
    );
    debug_assert!(board_extra_exact <= board_extra_candidates);
    debug_assert!(extra_ipc_exact <= extra_ipc_candidates);

    violations
}

/// Build exact native drill circles for a geometry check.
pub fn drills_to_sketch(
    drills: &[DrillFeature],
    name: &str,
    requested_check: &str,
) -> Result<PcbSketch, Box<Violation>> {
    let layers = vec![name.to_string()];
    let metadata = Some(LayerMetadata {
        name: name.to_string(),
    });
    let mut combined: Option<PcbSketch> = None;
    for drill in drills {
        let Ok(radius) = drill.diameter.clone() / crate::scalar::scalar("2") else {
            continue;
        };
        let circle = PcbSketch::new(
            Profile::circle(radius, 48).translate(
                drill.location[0].clone(),
                drill.location[1].clone(),
                Scalar::zero(),
            ),
            metadata.clone(),
        );
        combined = Some(match combined {
            None => circle,
            Some(current) => union_for_check(&current, &circle, requested_check, layers.clone())?,
        });
    }
    Ok(combined.unwrap_or_else(|| PcbSketch::new(Profile::empty(), metadata)))
}

fn nearest_matching_copper<'a>(
    board: &'a BoardModel,
    drill: &DrillFeature,
    selected_layers: &[String],
) -> Option<&'a CopperFeature> {
    selected_copper_features(board, selected_layers)
        .into_iter()
        .filter(|feature| drill.net.is_none() || feature.net == drill.net)
        .fold(None, |nearest, feature| {
            let distance = exact_distance_scalar(&feature.location, &drill.location)?;
            match nearest {
                Some((current, current_distance)) if current_distance <= distance => {
                    Some((current, current_distance))
                }
                _ => Some((feature, distance)),
            }
        })
        .map(|(feature, _)| feature)
}

fn has_plated_drill_copper(
    drill: &DrillFeature,
    copper_features: &[&CopperFeature],
    candidate_indices: &[usize],
    tolerance: &Scalar,
) -> bool {
    candidate_indices.iter().any(|&feature_index| {
        let feature = copper_features[feature_index];
        matches!(feature.kind, CopperKind::Pad | CopperKind::Via)
            && (drill.net.is_none() || feature.net == drill.net)
            && exact_distance_scalar(&feature.location, &drill.location)
                .is_some_and(|distance| &distance <= tolerance)
    })
}

fn has_nearby_copper(
    location: &[Scalar; 2],
    copper_features: &[&CopperFeature],
    candidate_indices: &[usize],
    tolerance: &Scalar,
) -> bool {
    candidate_indices.iter().any(|&feature_index| {
        exact_distance_scalar(&copper_features[feature_index].location, location)
            .is_some_and(|distance| &distance <= tolerance)
    })
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

fn equivalent_radius_scalar(sketch: &PcbSketch) -> Option<Scalar> {
    let area = multipolygon_area_scalar(&sketch.to_multipolygon())?;
    let area_over_pi = (area / Scalar::pi()).ok()?;
    area_over_pi.sqrt().ok()
}

fn diameters_conflict(left: &Scalar, right: &Scalar, tolerance: &Scalar) -> bool {
    left > &Scalar::zero() && right > &Scalar::zero() && &(left - right).abs() > tolerance
}

fn drill_table_violation(
    left_source: &str,
    left_diameter: &Scalar,
    right_source: &str,
    right_diameter: &Scalar,
    locations: Vec<[f64; 2]>,
) -> Violation {
    Violation::new(
        "drill-table-consistency",
        Severity::Warning,
        vec![left_source.to_string(), right_source.to_string()],
        None,
        Vec::new(),
        locations,
        Some(format!(
            "{left_source} diameter {left_diameter:.6} conflicts with {right_source} diameter {right_diameter:.6}"
        )),
    )
}

fn exact_distance_scalar(left: &[Scalar; 2], right: &[Scalar; 2]) -> Option<Scalar> {
    let dx = &left[0] - &right[0];
    let dy = &left[1] - &right[1];
    (&dx * &dx + &dy * &dy).sqrt().ok()
}

pub(super) fn drill_radius_with_clearance(
    drill: &DrillFeature,
    clearance: &Scalar,
) -> Option<Scalar> {
    let radius = (drill.diameter.clone() / crate::scalar::scalar("2")).ok()?;
    Some(radius + clearance)
}

fn drill_keepout_sketch(drill: &DrillFeature, radius: Scalar, name: &str) -> Option<PcbSketch> {
    let x = drill.location[0].clone();
    let y = drill.location[1].clone();
    let profile = Profile::circle(radius, 64).translate(x, y, Scalar::zero());
    Some(PcbSketch::new(
        profile,
        Some(LayerMetadata {
            name: name.to_string(),
        }),
    ))
}

fn drill_keepout_inside_rect_scalar_with_grid(
    drill: &DrillFeature,
    rect: &geo::Rect<f64>,
    clearance: &Scalar,
    _grid: SourceGridFacts,
) -> bool {
    let Some(radius) = drill_radius_with_clearance(drill, clearance) else {
        return false;
    };
    let (Ok(min_x), Ok(min_y), Ok(max_x), Ok(max_y)) = (
        Scalar::try_from(rect.min().x),
        Scalar::try_from(rect.min().y),
        Scalar::try_from(rect.max().x),
        Scalar::try_from(rect.max().y),
    ) else {
        return false;
    };

    &drill.location[0] - &radius >= min_x
        && &drill.location[0] + &radius <= max_x
        && &drill.location[1] - &radius >= min_y
        && &drill.location[1] + &radius <= max_y
}

fn scalar_broad_phase_radius(value: &Scalar) -> f64 {
    let projected = value
        .to_f64_lossy()
        .expect("drill broad-phase radius must fit the finite compatibility index");
    if projected > 0.0 {
        projected.next_up()
    } else {
        0.0
    }
}

#[cfg(test)]
mod tests {
    use super::{
        board_outline_drill_clearance, board_outline_drill_clearance_with_grid,
        drill_table_consistency, drill_to_copper_clearance, drill_to_copper_clearance_with_grid,
    };
    use crate::LayerMetadata;
    use crate::geometry::{
        SourceGridFacts, SourceUnit, line_polygon, polygons_to_profile, rect_polygon,
    };
    use crate::ipc356::Ipc356Point;
    use crate::kicad::{BoardModel, CopperFeature, CopperKind, DrillFeature};

    #[test]
    fn drill_to_copper_clearance_reports_other_net_copper() {
        let mut board = board_with_copper(vec![segment("SIG", [-1.0, 0.0], [1.0, 0.0], 0.20)]);
        board.drills = vec![drill(Some("GND"), [0.0, 0.0], 0.40, true)];

        let violations = drill_to_copper_clearance(
            &board,
            &[],
            &crate::scalar::scalar("0.20"),
            &[],
            &crate::scalar::scalar("1.0e-9"),
        );

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "drill-to-copper-clearance");
    }

    #[test]
    fn drill_to_copper_clearance_accepts_retained_excellon_grid_for_broad_phase() {
        let board = board_with_copper(vec![segment("SIG", [-1.0, 0.0], [1.0, 0.0], 0.20)]);
        let extra_drills = vec![drill(None, [0.0, 0.0], 0.40, false)];
        let grid = SourceGridFacts::source_grid(SourceUnit::Excellon, 1_000);

        let violations = drill_to_copper_clearance_with_grid(
            &board,
            &extra_drills,
            grid,
            &crate::scalar::scalar("0.20"),
            &[],
            &crate::scalar::scalar("1.0e-9"),
        );

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "drill-to-copper-clearance");
    }

    #[test]
    fn drill_to_copper_clearance_allows_same_net_plated_and_selected_out_copper() {
        let mut same_net = board_with_copper(vec![segment("SIG", [-1.0, 0.0], [1.0, 0.0], 0.20)]);
        same_net.drills = vec![drill(Some("SIG"), [0.0, 0.0], 0.40, true)];
        assert!(
            drill_to_copper_clearance(
                &same_net,
                &[],
                &crate::scalar::scalar("0.20"),
                &[],
                &crate::scalar::scalar("1.0e-9")
            )
            .is_empty()
        );

        let mut selected_out = board_with_copper(vec![segment_on_layer(
            "B.Cu",
            "SIG",
            [-1.0, 0.0],
            [1.0, 0.0],
            0.20,
        )]);
        selected_out.drills = vec![drill(None, [0.0, 0.0], 0.40, false)];
        assert!(
            drill_to_copper_clearance(
                &selected_out,
                &[],
                &crate::scalar::scalar("0.20"),
                &["F.Cu".to_string()],
                &crate::scalar::scalar("1.0e-9")
            )
            .is_empty()
        );
    }

    #[test]
    fn drill_to_copper_clearance_culls_large_sparse_feature_sets() {
        let mut copper = Vec::new();
        for index in 0..900 {
            copper.push(pad(
                &format!("N{index}"),
                [20.0 + index as f64 * 2.0, 20.0],
                [0.4, 0.4],
            ));
        }
        copper.push(segment("SIG", [-1.0, 0.0], [1.0, 0.0], 0.20));
        let mut board = board_with_copper(copper);
        board.drills = vec![drill(None, [0.0, 0.0], 0.40, false)];

        let started = std::time::Instant::now();
        let violations = drill_to_copper_clearance(
            &board,
            &[],
            &crate::scalar::scalar("0.20"),
            &[],
            &crate::scalar::scalar("1.0e-9"),
        );

        assert_eq!(violations.len(), 1);
        assert!(
            started.elapsed() < std::time::Duration::from_secs(2),
            "drill-to-copper clearance should cull sparse copper before exact CSG"
        );
    }

    #[test]
    fn drill_spacing_reports_close_holes_and_allows_distant_holes() {
        let drills = vec![
            drill(None, [0.0, 0.0], 0.40, true),
            drill(None, [0.55, 0.0], 0.40, true),
            drill(None, [3.0, 0.0], 0.40, true),
        ];

        let violations = super::drill_spacing(&drills, &[], &crate::scalar::scalar("0.20"));

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "drill-spacing");
    }

    #[test]
    fn drill_spacing_culls_large_sparse_drill_fields() {
        let mut drills = (0..2_000)
            .map(|index| drill(None, [10.0 + index as f64 * 2.0, 10.0], 0.30, true))
            .collect::<Vec<_>>();
        drills.push(drill(None, [0.0, 0.0], 0.40, true));
        drills.push(drill(None, [0.55, 0.0], 0.40, true));

        let started = std::time::Instant::now();
        let violations = super::drill_spacing(&drills, &[], &crate::scalar::scalar("0.20"));

        assert_eq!(violations.len(), 1);
        assert!(
            started.elapsed() < std::time::Duration::from_secs(2),
            "drill spacing should cull sparse drill fields before exact edge-gap review"
        );
    }

    #[test]
    fn board_outline_drill_clearance_skips_rectangular_interior_drill_fields() {
        let outline = sketch(vec![rect_polygon([50.0, 50.0], [100.0, 100.0], 0.0)]);
        let mut drills = (0..2_000)
            .map(|index| {
                drill(
                    None,
                    [
                        5.0 + (index % 50) as f64 * 1.5,
                        5.0 + (index / 50) as f64 * 1.5,
                    ],
                    0.30,
                    false,
                )
            })
            .collect::<Vec<_>>();
        drills.push(drill(None, [0.35, 50.0], 0.30, false));

        let started = std::time::Instant::now();
        let violations = board_outline_drill_clearance(
            "KiCad drills",
            "KiCad Edge.Cuts",
            &outline,
            &drills,
            &[],
            &crate::scalar::scalar("0.25"),
            &crate::scalar::scalar("1.0e-9"),
        );

        assert_eq!(violations.len(), 1);
        assert!(
            started.elapsed() < std::time::Duration::from_secs(2),
            "board-outline drill clearance should skip inset drills on rectangular outlines before exact CSG"
        );
    }

    #[test]
    fn board_outline_drill_clearance_accepts_retained_excellon_grid_for_rect_fast_path() {
        let outline = sketch(vec![rect_polygon([50.0, 50.0], [100.0, 100.0], 0.0)]);
        let drills = vec![
            drill(None, [50.0, 50.0], 0.30, false),
            drill(None, [0.35, 50.0], 0.30, false),
        ];
        let grid = SourceGridFacts::source_grid(SourceUnit::Excellon, 1_000);

        let violations = board_outline_drill_clearance_with_grid(
            "Excellon drills",
            "Gerber profile",
            &outline,
            &[],
            &drills,
            &crate::scalar::scalar("0.25"),
            &crate::scalar::scalar("1.0e-9"),
            grid,
        );

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "board-outline-drill-clearance");
        assert_eq!(violations[0].locations, vec![[0.35, 50.0]]);
    }

    #[test]
    fn drill_table_consistency_reports_nearby_cross_source_conflicts() {
        let board_drills = vec![drill(None, [0.0, 0.0], 0.40, true)];
        let extra_drills = vec![drill(None, [0.05, 0.0], 0.60, true)];
        let ipc_points = vec![ipc_point([0.06, 0.0], Some(0.80))];

        let violations = drill_table_consistency(
            &board_drills,
            &extra_drills,
            &ipc_points,
            &crate::scalar::scalar("0.10"),
        );

        assert_eq!(violations.len(), 2);
        assert!(
            violations
                .iter()
                .all(|violation| violation.check == "drill-table-consistency")
        );
    }

    #[test]
    fn drill_table_consistency_culls_large_sparse_sidecars() {
        let board_drills = vec![drill(None, [0.0, 0.0], 0.40, true)];
        let mut extra_drills = (0..2_000)
            .map(|index| drill(None, [20.0 + index as f64 * 2.0, 20.0], 0.60, true))
            .collect::<Vec<_>>();
        extra_drills.push(drill(None, [0.05, 0.0], 0.60, true));
        let mut ipc_points = (0..2_000)
            .map(|index| ipc_point([40.0 + index as f64 * 2.0, 40.0], Some(0.80)))
            .collect::<Vec<_>>();
        ipc_points.push(ipc_point([0.06, 0.0], Some(0.80)));

        let started = std::time::Instant::now();
        let violations = drill_table_consistency(
            &board_drills,
            &extra_drills,
            &ipc_points,
            &crate::scalar::scalar("0.10"),
        );

        assert_eq!(violations.len(), 2);
        assert!(
            started.elapsed() < std::time::Duration::from_secs(2),
            "drill-table consistency should index sparse sidecar records before exact diameter review"
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

    fn sketch(polygons: Vec<geo::Polygon<f64>>) -> crate::PcbSketch {
        polygons_to_profile(
            polygons,
            Some(LayerMetadata {
                name: "outline".to_string(),
            }),
        )
    }

    fn segment(net: &str, start: [f64; 2], end: [f64; 2], width: f64) -> CopperFeature {
        segment_on_layer("F.Cu", net, start, end, width)
    }

    fn segment_on_layer(
        layer: &str,
        net: &str,
        start: [f64; 2],
        end: [f64; 2],
        width: f64,
    ) -> CopperFeature {
        CopperFeature {
            layer: layer.to_string(),
            net: Some(net.to_string()),
            kind: CopperKind::Segment,
            location: [
                crate::geometry::exact_real((start[0] + end[0]) / 2.0),
                crate::geometry::exact_real((start[1] + end[1]) / 2.0),
            ],
            sketch: polygons_to_profile(
                vec![line_polygon(start, end, width).expect("test segment should be valid")],
                Some(LayerMetadata {
                    name: "segment".to_string(),
                }),
            ),
        }
    }

    fn pad(net: &str, location: [f64; 2], size: [f64; 2]) -> CopperFeature {
        CopperFeature {
            layer: "F.Cu".to_string(),
            net: Some(net.to_string()),
            kind: CopperKind::Pad,
            location: [
                crate::geometry::exact_real(location[0]),
                crate::geometry::exact_real(location[1]),
            ],
            sketch: polygons_to_profile(
                vec![rect_polygon(location, size, 0.0)],
                Some(LayerMetadata {
                    name: "pad".to_string(),
                }),
            ),
        }
    }

    fn drill(net: Option<&str>, location: [f64; 2], diameter: f64, plated: bool) -> DrillFeature {
        DrillFeature {
            location: [
                crate::geometry::exact_real(location[0]),
                crate::geometry::exact_real(location[1]),
            ],
            diameter: crate::geometry::exact_real(diameter),
            net: net.map(str::to_string),
            plated,
        }
    }

    fn ipc_point(location: [f64; 2], diameter: Option<f64>) -> Ipc356Point {
        Ipc356Point {
            net: "SIG".to_string(),
            reference: None,
            pin: None,
            location: [
                crate::geometry::exact_real(location[0]),
                crate::geometry::exact_real(location[1]),
            ],
            diameter: diameter.map(crate::geometry::exact_real),
            access_side: None,
            feature_type: None,
            soldermask: None,
        }
    }
}
