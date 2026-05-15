//! Layer-level geometry checks over already-flattened sketches.
//!
//! These checks are independent of KiCad concepts such as nets and drills, so
//! Gerber-derived layers and KiCad-derived layers share the same behavior.
//!
//! Reliability note: offset/opening operations, local-density windows, and
//! boundary-distance fallbacks approximate manufacturing concerns over polygon
//! data. Double-check suspect results against CAM tooling when geometry is
//! self-touching, highly fragmented, or near rule thresholds.

use std::collections::BTreeMap;

use csgrs::csg::CSG;
use geo::{
    Area, BoundingRect, Coord, Line, LineString, MultiPolygon, Polygon,
    line_intersection::{LineIntersection, line_intersection},
};

use crate::checks::distance::polygon_boundary_distance;
use crate::geometry::{
    multipolygon_to_shapes, polygon_to_sketch, polygons_to_sketch, rect_polygon,
};
use crate::report::{Severity, Violation};
use crate::{LayerMetadata, PcbSketch};

const LOCAL_COPPER_DENSITY_FLOOR: f64 = 0.05;
const LOCAL_COPPER_DENSITY_DELTA: f64 = 0.50;
const DUPLICATE_LAYER_OVERLAP_RATIO: f64 = 0.999_999;
const DUPLICATE_LAYER_SIGNATURE_SCALE: f64 = 1_000_000.0;

/// Run the `mask_island_keepout` design-readiness check or report helper.
pub fn mask_island_keepout(
    layer_name: &str,
    sketch: &PcbSketch,
    keepout: f64,
    min_area: f64,
) -> Vec<Violation> {
    let polygons = sketch.to_multipolygon().0;
    log::trace!(
        "mask-island keepout: layer={layer_name} islands={} keepout={keepout:.6}",
        polygons.len()
    );
    let mut violations = Vec::new();

    for island_index in 0..polygons.len() {
        for neighbor_index in (island_index + 1)..polygons.len() {
            if !polygons_within_clearance(
                &polygons[island_index],
                &polygons[neighbor_index],
                keepout * 2.0,
            ) {
                continue;
            }

            // Offset both candidate islands and intersect only after a bounding
            // box broad phase. This avoids rebuilding and offsetting the rest
            // of a dense soldermask layer for every island while preserving the
            // same exact polygon predicate for nearby island pairs.
            let island =
                polygon_to_sketch(polygons[island_index].clone(), Some(metadata(layer_name)));
            let neighbor =
                polygon_to_sketch(polygons[neighbor_index].clone(), Some(metadata(layer_name)));
            let overlap = island
                .offset(keepout)
                .intersection(&neighbor.offset(keepout));
            let shapes = multipolygon_to_shapes(&overlap.to_multipolygon(), min_area);

            if !shapes.is_empty() {
                violations.push(Violation::new(
                    "mask-island-keepout",
                    Severity::Error,
                    vec![layer_name.to_string()],
                    Some(island_index),
                    shapes,
                    Vec::new(),
                    Some(format!(
                        "island keepout intersects neighboring mask geometry after {keepout} offset"
                    )),
                ));
            }
        }
    }

    violations
}

fn polygons_within_clearance(left: &Polygon<f64>, right: &Polygon<f64>, clearance: f64) -> bool {
    let Some(left_bounds) = left.bounding_rect() else {
        return true;
    };
    let Some(right_bounds) = right.bounding_rect() else {
        return true;
    };

    left_bounds.min().x - clearance <= right_bounds.max().x
        && left_bounds.max().x + clearance >= right_bounds.min().x
        && left_bounds.min().y - clearance <= right_bounds.max().y
        && left_bounds.max().y + clearance >= right_bounds.min().y
}

/// Run the `copper_overlap` design-readiness check or report helper.
pub fn copper_overlap(
    left_name: &str,
    left: &PcbSketch,
    right_name: &str,
    right: &PcbSketch,
    min_area: f64,
) -> Vec<Violation> {
    intersection_violation(
        PairCheck {
            check: "copper-overlap",
            severity: Severity::Error,
            message: "copper regions overlap across layers",
        },
        left_name,
        left,
        right_name,
        right,
        min_area,
    )
}

/// Run the `board_edge_clearance` design-readiness check or report helper.
pub fn board_edge_clearance(
    copper_name: &str,
    copper: &PcbSketch,
    board_name: &str,
    board: &PcbSketch,
    clearance: f64,
    min_area: f64,
) -> Vec<Violation> {
    let allowed = board.offset(-clearance);
    let intrusion = copper.difference(&allowed);
    let shapes = multipolygon_to_shapes(&intrusion.to_multipolygon(), min_area);

    if shapes.is_empty() {
        return Vec::new();
    }

    vec![Violation::new(
        "copper-to-board-edge-clearance",
        Severity::Error,
        vec![copper_name.to_string(), board_name.to_string()],
        None,
        shapes,
        Vec::new(),
        Some(format!(
            "copper falls outside the board outline eroded by clearance {clearance}"
        )),
    )]
}

/// Warn when geometry enters board-cutout regions created by nested outline
/// contours. KiCad can emit outline contours for slots, windows, and other
/// removed areas; this readiness check flags copper, masks, or other layers that
/// enters a nested contour region. For each nested contour, any feature
/// touching or intruding into the clearance band is reported.
pub fn board_outline_cutout_clearance(
    subject_name: &str,
    subject: &PcbSketch,
    outline_name: &str,
    outline: &PcbSketch,
    clearance: f64,
    min_area: f64,
) -> Vec<Violation> {
    let mut violations = Vec::new();
    let outline_polygons = outline.to_multipolygon();
    for cutout in board_outline_cutouts(&outline_polygons) {
        let cutout = polygon_to_sketch(cutout, Some(metadata("board cutout")));
        let clearance_band = if clearance > 0.0 {
            cutout.offset(clearance)
        } else {
            cutout.clone()
        };

        let intrusion = subject.intersection(&clearance_band);
        let shapes = multipolygon_to_shapes(&intrusion.to_multipolygon(), min_area);
        let touches_cutout = shapes.is_empty()
            && polygon_boundary_distance(&subject.to_multipolygon(), &cutout.to_multipolygon())
                <= clearance;
        if shapes.is_empty() && !touches_cutout {
            continue;
        }

        violations.push(Violation::new(
            "board-outline-cutout-clearance",
            Severity::Warning,
            vec![subject_name.to_string(), outline_name.to_string()],
            None,
            shapes,
            Vec::new(),
            Some(format!(
                "subject geometry touches or intrudes into a nested board contour (cutout) with clearance {clearance}"
            )),
        ));
    }

    violations
}

fn board_outline_cutouts(outline: &MultiPolygon<f64>) -> Vec<Polygon<f64>> {
    let polygons = &outline.0;
    if polygons.len() < 2 {
        return Vec::new();
    }

    let mut cutouts = Vec::new();
    for inner_index in 0..polygons.len() {
        let inner = &polygons[inner_index];
        if inner.unsigned_area() <= 0.0 {
            continue;
        }

        let is_nested = (0..polygons.len())
            .filter(|&outer_index| outer_index != inner_index)
            .any(|outer_index| {
                let outer = &polygons[outer_index];
                polygon_contains_other_outer(
                    outer,
                    inner,
                    BOARD_OUTLINE_NESTED_OVERLAP_RATIO,
                    BOARD_OUTLINE_GEOMETRY_TOLERANCE,
                )
            });
        if !is_nested {
            continue;
        }

        let Some(point) = representative_point(inner) else {
            continue;
        };
        if cutouts
            .iter()
            .filter_map(representative_point)
            .any(|candidate| location_is_close(&candidate, &point))
        {
            continue;
        }

        cutouts.push(inner.clone());
    }

    cutouts
}

/// Run the `silkscreen_board_edge_clearance` design-readiness check or report helper.
pub fn silkscreen_board_edge_clearance(
    silk_name: &str,
    silk: &PcbSketch,
    board_name: &str,
    board: &PcbSketch,
    clearance: f64,
    min_area: f64,
) -> Vec<Violation> {
    let allowed = board.offset(-clearance);
    let intrusion = silk.difference(&allowed);
    shapes_violation(
        "silkscreen-to-board-edge-clearance",
        Severity::Warning,
        vec![silk_name.to_string(), board_name.to_string()],
        intrusion,
        min_area,
        format!("silkscreen falls outside the board outline eroded by clearance {clearance}"),
    )
}

/// Run the `solder_mask_board_edge_clearance` design-readiness check or report helper.
pub fn solder_mask_board_edge_clearance(
    mask_name: &str,
    mask: &PcbSketch,
    board_name: &str,
    board: &PcbSketch,
    clearance: f64,
    min_area: f64,
) -> Vec<Violation> {
    let allowed = board.offset(-clearance);
    let intrusion = mask.difference(&allowed);
    shapes_violation(
        "solder-mask-to-board-edge-clearance",
        Severity::Warning,
        vec![mask_name.to_string(), board_name.to_string()],
        intrusion,
        min_area,
        format!(
            "solder mask opening falls outside the board outline eroded by clearance {clearance}"
        ),
    )
}

/// Run the `paste_overhang` design-readiness check or report helper.
pub fn paste_overhang(
    paste_name: &str,
    paste: &PcbSketch,
    copper_name: &str,
    copper: &PcbSketch,
    tolerance: f64,
    min_area: f64,
) -> Vec<Violation> {
    let allowed = copper.offset(tolerance);
    let overhang = paste.difference(&allowed);
    shapes_violation(
        "paste-aperture-overhang",
        Severity::Warning,
        vec![paste_name.to_string(), copper_name.to_string()],
        overhang,
        min_area,
        format!("paste extends outside copper expanded by tolerance {tolerance}"),
    )
}

/// Run the `paste_aperture_coverage` design-readiness check or report helper.
pub fn paste_aperture_coverage(
    paste_name: &str,
    paste: &PcbSketch,
    copper_name: &str,
    copper: &PcbSketch,
    min_area: f64,
) -> Vec<Violation> {
    let uncovered_copper = copper.difference(paste);
    shapes_violation(
        "paste-aperture-coverage",
        Severity::Warning,
        vec![paste_name.to_string(), copper_name.to_string()],
        uncovered_copper,
        min_area,
        "copper is not covered by a paste aperture".to_string(),
    )
}

/// Run the `solder_mask_overlap_clearance` design-readiness check or report helper.
pub fn solder_mask_overlap_clearance(
    copper_name: &str,
    copper: &PcbSketch,
    mask_name: &str,
    mask: &PcbSketch,
    clearance: f64,
    min_area: f64,
) -> Vec<Violation> {
    let mask_clearance_band = mask.offset(clearance).difference(mask);
    let vulnerable_copper = copper.intersection(&mask_clearance_band);
    shapes_violation(
        "solder-mask-overlap-clearance",
        Severity::Warning,
        vec![copper_name.to_string(), mask_name.to_string()],
        vulnerable_copper,
        min_area,
        format!("covered copper is within mask opening clearance {clearance}"),
    )
}

/// Run the `paste_aperture_ratio` design-readiness check or report helper.
pub fn paste_aperture_ratio(
    paste_name: &str,
    paste: &PcbSketch,
    copper_name: &str,
    copper: &PcbSketch,
    min_ratio: f64,
    max_ratio: f64,
    min_area: f64,
) -> Vec<Violation> {
    let mut violations = Vec::new();
    let paste_polygons = paste.to_multipolygon().0;

    for (island_index, copper_polygon) in copper.to_multipolygon().0.into_iter().enumerate() {
        let copper_area = copper_polygon.unsigned_area();
        if copper_area <= min_area {
            continue;
        }

        let island = polygon_to_sketch(copper_polygon, Some(metadata(copper_name)));
        let paste_area = paste_polygons
            .iter()
            .filter(|paste_polygon| {
                let paste_island =
                    polygon_to_sketch((*paste_polygon).clone(), Some(metadata(paste_name)));
                island
                    .intersection(&paste_island)
                    .to_multipolygon()
                    .unsigned_area()
                    > min_area
            })
            .map(Polygon::unsigned_area)
            .sum::<f64>();
        let ratio = paste_area / copper_area;

        if ratio >= min_ratio && ratio <= max_ratio {
            continue;
        }

        violations.push(Violation::new(
            "paste-aperture-ratio",
            Severity::Warning,
            vec![paste_name.to_string(), copper_name.to_string()],
            Some(island_index),
            multipolygon_to_shapes(&island.to_multipolygon(), min_area),
            Vec::new(),
            Some(format!(
                "paste-to-copper area ratio {ratio:.3} is outside configured range {min_ratio:.3}..{max_ratio:.3}"
            )),
        ));
    }

    violations
}

/// Run the `minimum_paste_aperture` design-readiness check or report helper.
pub fn minimum_paste_aperture(
    paste_name: &str,
    paste: &PcbSketch,
    min_width: f64,
    min_area: f64,
) -> Vec<Violation> {
    let mut violations = Vec::new();

    for (island_index, polygon) in paste.to_multipolygon().0.into_iter().enumerate() {
        let Some(bounds) = polygon.bounding_rect() else {
            continue;
        };
        let width = bounds.max().x - bounds.min().x;
        let height = bounds.max().y - bounds.min().y;
        let smallest_dimension = width.min(height);

        if smallest_dimension >= min_width || polygon.unsigned_area() <= min_area {
            continue;
        }

        let aperture = polygon_to_sketch(polygon, Some(metadata(paste_name)));
        violations.push(Violation::new(
            "minimum-paste-aperture",
            Severity::Warning,
            vec![paste_name.to_string()],
            Some(island_index),
            multipolygon_to_shapes(&aperture.to_multipolygon(), min_area),
            Vec::new(),
            Some(format!(
                "paste aperture minimum dimension {smallest_dimension:.6} is below {min_width:.6}"
            )),
        ));
    }

    violations
}

/// Run the `paste_aperture_spacing` design-readiness check or report helper.
pub fn paste_aperture_spacing(
    paste_name: &str,
    paste: &PcbSketch,
    min_spacing: f64,
    min_area: f64,
) -> Vec<Violation> {
    let polygons = paste.to_multipolygon().0;
    let mut violations = Vec::new();
    let expansion = min_spacing / 2.0;

    for island_index in 0..polygons.len() {
        let island = polygon_to_sketch(polygons[island_index].clone(), Some(metadata(paste_name)));
        let remaining_polygons = polygons
            .iter()
            .enumerate()
            .filter_map(|(index, polygon)| (index != island_index).then_some(polygon.clone()))
            .collect::<Vec<_>>();

        if remaining_polygons.is_empty() {
            continue;
        }

        let remaining = polygons_to_sketch(remaining_polygons, Some(metadata(paste_name)));
        let overlap = island
            .offset(expansion)
            .intersection(&remaining.offset(expansion));
        let shapes = multipolygon_to_shapes(&overlap.to_multipolygon(), min_area);
        if shapes.is_empty() {
            continue;
        }

        violations.push(Violation::new(
            "paste-aperture-spacing",
            Severity::Warning,
            vec![paste_name.to_string()],
            Some(island_index),
            shapes,
            Vec::new(),
            Some(format!(
                "paste apertures are closer than minimum spacing {min_spacing}"
            )),
        ));
    }

    violations
}

/// Run the `paste_mask_alignment` design-readiness check or report helper.
pub fn paste_mask_alignment(
    paste_name: &str,
    paste: &PcbSketch,
    mask_name: &str,
    mask: &PcbSketch,
    min_area: f64,
) -> Vec<Violation> {
    let outside_mask_opening = paste.difference(mask);
    shapes_violation(
        "paste-mask-alignment",
        Severity::Warning,
        vec![paste_name.to_string(), mask_name.to_string()],
        outside_mask_opening,
        min_area,
        "paste aperture extends outside the paired solder mask opening".to_string(),
    )
}

/// Run the `exposed_copper` design-readiness check or report helper.
pub fn exposed_copper(
    copper_name: &str,
    copper: &PcbSketch,
    mask_name: &str,
    mask_openings: &PcbSketch,
    min_area: f64,
) -> Vec<Violation> {
    intersection_violation(
        PairCheck {
            check: "exposed-copper",
            severity: Severity::Warning,
            message: "copper intersects solder mask openings",
        },
        copper_name,
        copper,
        mask_name,
        mask_openings,
        min_area,
    )
}

/// Run the `solder_mask_opening_coverage` design-readiness check or report helper.
pub fn solder_mask_opening_coverage(
    copper_name: &str,
    copper: &PcbSketch,
    mask_name: &str,
    mask_openings: &PcbSketch,
    min_area: f64,
) -> Vec<Violation> {
    let covered_copper = copper.difference(mask_openings);
    shapes_violation(
        "solder-mask-opening-coverage",
        Severity::Error,
        vec![copper_name.to_string(), mask_name.to_string()],
        covered_copper,
        min_area,
        "copper is not covered by a solder mask opening".to_string(),
    )
}

/// Run the `solder_mask_expansion` design-readiness check or report helper.
pub fn solder_mask_expansion(
    copper_name: &str,
    copper: &PcbSketch,
    mask_name: &str,
    mask_openings: &PcbSketch,
    max_expansion: f64,
    min_area: f64,
) -> Vec<Violation> {
    let allowed_opening = copper.offset(max_expansion);
    let excessive_opening = mask_openings.difference(&allowed_opening);
    shapes_violation(
        "solder-mask-expansion",
        Severity::Warning,
        vec![copper_name.to_string(), mask_name.to_string()],
        excessive_opening,
        min_area,
        format!("solder mask opening exceeds copper expansion {max_expansion}"),
    )
}

/// Run the `silkscreen_overlap` design-readiness check or report helper.
pub fn silkscreen_overlap(
    silk_name: &str,
    silk: &PcbSketch,
    blocker_name: &str,
    blocker: &PcbSketch,
    min_area: f64,
) -> Vec<Violation> {
    intersection_violation(
        PairCheck {
            check: "silkscreen-overlap",
            severity: Severity::Warning,
            message: "silkscreen overlaps copper or exposed-pad geometry",
        },
        silk_name,
        silk,
        blocker_name,
        blocker,
        min_area,
    )
}

/// Run the `silkscreen_clearance` design-readiness check or report helper.
pub fn silkscreen_clearance(
    silk_name: &str,
    silk: &PcbSketch,
    blocker_name: &str,
    blocker: &PcbSketch,
    clearance: f64,
    min_area: f64,
) -> Vec<Violation> {
    let clearance_region = blocker.offset(clearance);
    let intrusion = silk.intersection(&clearance_region);
    shapes_violation(
        "silkscreen-clearance",
        Severity::Warning,
        vec![silk_name.to_string(), blocker_name.to_string()],
        intrusion,
        min_area,
        format!("silkscreen is within clearance {clearance} of blocker geometry"),
    )
}

/// Run the `silkscreen_min_width` design-readiness check or report helper.
pub fn silkscreen_min_width(
    silk_name: &str,
    silk: &PcbSketch,
    min_width: f64,
    min_area: f64,
) -> Vec<Violation> {
    let radius = min_width / 2.0;
    let polygons = silk.to_multipolygon().0;
    log::trace!(
        "silkscreen-min-width: layer={silk_name} polygons={} min_width={min_width:.6}",
        polygons.len()
    );
    let mut shapes = Vec::new();

    for polygon in polygons {
        // Apply morphological opening to one disconnected legend island at a
        // time. Whole-layer opening can create pathological boolean operations
        // on dense Gerber packages, while island-local opening is equivalent
        // for independent silkscreen strokes.
        let island = polygon_to_sketch(polygon, Some(metadata(silk_name)));
        let reconstructed = island.offset(-radius).offset(radius);
        let thin_features = island.difference(&reconstructed);
        shapes.extend(multipolygon_to_shapes(
            &thin_features.to_multipolygon(),
            min_area,
        ));
    }

    if shapes.is_empty() {
        return Vec::new();
    }

    vec![Violation::new(
        "silkscreen-min-width",
        Severity::Warning,
        vec![silk_name.to_string()],
        None,
        shapes,
        Vec::new(),
        Some(format!(
            "silkscreen features are removed by opening with width {min_width}"
        )),
    )]
}

/// Run the `min_copper_neck_width` design-readiness check or report helper.
pub fn min_copper_neck_width(
    copper_name: &str,
    copper: &PcbSketch,
    min_width: f64,
    min_area: f64,
) -> Vec<Violation> {
    let radius = min_width / 2.0;
    let source_polygons = copper.to_multipolygon().0;
    log::trace!(
        "min-copper-neck: layer={copper_name} polygons={} min_width={min_width:.6} min_area={min_area:.6}",
        source_polygons.len()
    );

    // Morphological opening: erode by r, then dilate by r. Features that cannot
    // contain a disk of radius r disappear, which makes this a useful fast
    // approximation for "minimum neck width" checks on copper. This follows the
    // dilation/erosion algebra formalized in Heijmans and Ronse,
    // "The algebraic basis of mathematical morphology I. Dilations and erosions",
    // Computer Vision, Graphics, and Image Processing, 1990.
    //
    // Run the opening per polygon island instead of against the whole layer.
    // Real production pours can contain hundreds or thousands of independent
    // islands; feeding that entire multipolygon into one offset/difference can
    // create pathological boolean work in the geometry kernel even though the
    // minimum-neck question is local to each island. Per-island opening preserves
    // the check's intent while bounding each offset operation to a much smaller
    // contour set.
    let mut shapes = Vec::new();
    for (island_index, polygon) in source_polygons.iter().cloned().enumerate() {
        log::trace!(
            "min-copper-neck: layer={copper_name} island={island_index} exterior_vertices={} holes={}",
            polygon.exterior().0.len(),
            polygon.interiors().len()
        );
        let source = MultiPolygon(vec![polygon.clone()]);
        let island = polygon_to_sketch(polygon, Some(metadata(copper_name)));
        let reconstructed = island.offset(-radius).offset(radius);
        let thin_features = island.difference(&reconstructed);
        let thin = thin_features.to_multipolygon();
        if whole_feature_removal_is_width_compliant(&source, &thin, min_width) {
            continue;
        }
        shapes.extend(multipolygon_to_shapes(&thin, min_area));
    }

    if shapes.is_empty() {
        return Vec::new();
    }

    vec![Violation::new(
        "minimum-copper-neck-width",
        Severity::Warning,
        vec![copper_name.to_string()],
        None,
        shapes,
        Vec::new(),
        Some(format!(
            "copper features are removed by opening with width {min_width}"
        )),
    )]
}

fn whole_feature_removal_is_width_compliant(
    source: &MultiPolygon<f64>,
    removed: &MultiPolygon<f64>,
    min_width: f64,
) -> bool {
    let source_area = source.unsigned_area();
    if source_area == 0.0 || (removed.unsigned_area() - source_area).abs() > source_area * 1.0e-6 {
        return false;
    }

    source
        .0
        .iter()
        .all(|polygon| shortest_exterior_segment(polygon) >= min_width)
}

fn shortest_exterior_segment(polygon: &Polygon<f64>) -> f64 {
    polygon
        .exterior()
        .0
        .windows(2)
        .map(|segment| {
            let dx = segment[1].x - segment[0].x;
            let dy = segment[1].y - segment[0].y;
            (dx * dx + dy * dy).sqrt()
        })
        .filter(|length| *length > 1.0e-12)
        .fold(f64::INFINITY, f64::min)
}

/// Run the `solder_mask_sliver` design-readiness check or report helper.
pub fn solder_mask_sliver(
    mask_name: &str,
    mask: &PcbSketch,
    min_width: f64,
    min_area: f64,
) -> Vec<Violation> {
    let radius = min_width / 2.0;
    // Same opening operation as the copper neck-width check, applied to residual
    // mask geometry. The result is the geometry that is too thin to survive the
    // configured web width.
    let reconstructed = mask.offset(-radius).offset(radius);
    let slivers = mask.difference(&reconstructed);
    shapes_violation(
        "solder-mask-sliver",
        Severity::Warning,
        vec![mask_name.to_string()],
        slivers,
        min_area,
        format!("solder mask geometry is removed by opening with width {min_width}"),
    )
}

/// Run the `minimum_mask_opening` design-readiness check or report helper.
pub fn minimum_mask_opening(
    mask_name: &str,
    mask: &PcbSketch,
    min_opening: f64,
    min_area: f64,
) -> Vec<Violation> {
    let mut violations = Vec::new();

    for (island_index, polygon) in mask.to_multipolygon().0.into_iter().enumerate() {
        let Some(bounds) = polygon.bounding_rect() else {
            continue;
        };
        let width = bounds.max().x - bounds.min().x;
        let height = bounds.max().y - bounds.min().y;
        let smallest_dimension = width.min(height);

        if smallest_dimension >= min_opening || polygon.unsigned_area() <= min_area {
            continue;
        }

        let opening = polygon_to_sketch(polygon, Some(metadata(mask_name)));
        violations.push(Violation::new(
            "minimum-mask-opening",
            Severity::Warning,
            vec![mask_name.to_string()],
            Some(island_index),
            multipolygon_to_shapes(&opening.to_multipolygon(), min_area),
            Vec::new(),
            Some(format!(
                "solder mask opening minimum dimension {smallest_dimension:.6} is below {min_opening:.6}"
            )),
        ));
    }

    violations
}

/// Run the `solder_mask_opening_spacing` design-readiness check or report helper.
pub fn solder_mask_opening_spacing(
    mask_name: &str,
    mask: &PcbSketch,
    min_spacing: f64,
    min_area: f64,
) -> Vec<Violation> {
    let openings = mask.to_multipolygon().0;
    log::trace!(
        "solder-mask-opening-spacing: layer={mask_name} openings={} min_spacing={min_spacing:.6}",
        openings.len()
    );
    let mut violations = Vec::new();
    let expansion = min_spacing / 2.0;

    for opening_index in 0..openings.len() {
        for neighbor_index in (opening_index + 1)..openings.len() {
            if !polygons_within_clearance(
                &openings[opening_index],
                &openings[neighbor_index],
                min_spacing,
            ) {
                continue;
            }

            let opening =
                polygon_to_sketch(openings[opening_index].clone(), Some(metadata(mask_name)));
            let neighbor =
                polygon_to_sketch(openings[neighbor_index].clone(), Some(metadata(mask_name)));
            let bridge_conflict = opening
                .offset(expansion)
                .intersection(&neighbor.offset(expansion));
            let shapes = multipolygon_to_shapes(&bridge_conflict.to_multipolygon(), min_area);
            if shapes.is_empty() {
                continue;
            }

            violations.push(Violation::new(
                "solder-mask-opening-spacing",
                Severity::Warning,
                vec![mask_name.to_string()],
                Some(opening_index),
                shapes,
                Vec::new(),
                Some(format!(
                    "solder mask openings are closer than minimum bridge width {min_spacing}"
                )),
            ));
        }
    }

    violations
}

/// Run the `acid_trap_candidates` design-readiness check or report helper.
pub fn acid_trap_candidates(
    copper_name: &str,
    copper: &PcbSketch,
    max_angle_degrees: f64,
) -> Vec<Violation> {
    let mut locations = Vec::new();

    for polygon in copper.to_multipolygon().0 {
        collect_acute_vertices(polygon.exterior(), max_angle_degrees, &mut locations);
        for hole in polygon.interiors() {
            collect_acute_vertices(hole, max_angle_degrees, &mut locations);
        }
    }

    if locations.is_empty() {
        return Vec::new();
    }

    vec![Violation::new(
        "acid-trap-candidate",
        Severity::Warning,
        vec![copper_name.to_string()],
        None,
        Vec::new(),
        locations,
        Some(format!(
            "copper polygon vertices below {max_angle_degrees} degrees"
        )),
    )]
}

/// Run the `layer_sanity` design-readiness check or report helper.
pub fn layer_sanity(
    layer_name: &str,
    sketch: &PcbSketch,
    max_layer_area: Option<f64>,
) -> Vec<Violation> {
    let mut violations = Vec::new();
    let multipolygon = sketch.to_multipolygon();
    let area = multipolygon.unsigned_area();

    if multipolygon_has_non_finite_coordinates(&multipolygon) {
        violations.push(Violation::new(
            "layer-sanity",
            Severity::Error,
            vec![layer_name.to_string()],
            None,
            Vec::new(),
            Vec::new(),
            Some(
                "layer contains non-finite coordinates that cannot be validated geometrically"
                    .to_string(),
            ),
        ));
    }

    let intersections = collect_ring_self_intersections(&multipolygon);
    if !intersections.is_empty() {
        violations.push(Violation::new(
            "layer-sanity",
            Severity::Error,
            vec![layer_name.to_string()],
            None,
            Vec::new(),
            intersections,
            Some("layer contains self-intersecting contours".to_string()),
        ));
    }

    if area <= 0.0 {
        violations.push(Violation::new(
            "layer-sanity",
            Severity::Warning,
            vec![layer_name.to_string()],
            None,
            Vec::new(),
            Vec::new(),
            Some("layer parsed to empty polygon geometry".to_string()),
        ));
    }

    if let Some(max_layer_area) = max_layer_area
        && area > max_layer_area
    {
        let shapes = multipolygon_to_shapes(&multipolygon, 0.0);
        violations.push(Violation::new(
            "layer-sanity",
            Severity::Warning,
            vec![layer_name.to_string()],
            None,
            shapes,
            Vec::new(),
            Some(format!(
                "layer area {area:.9} exceeds maximum expected area {max_layer_area:.9}"
            )),
        ));
    }

    if sketch.geometry.bounding_rect().is_none() {
        violations.push(Violation::new(
            "layer-sanity",
            Severity::Warning,
            vec![layer_name.to_string()],
            None,
            Vec::new(),
            Vec::new(),
            Some("layer has no finite bounding rectangle".to_string()),
        ));
    }

    violations
}

/// Warn when a parsed layer contains polygon islands at or below the configured
/// reportable feature area.
///
/// This is a lightweight Gerber-sanity check for tiny aperture flashes,
/// fractured slivers, and parser artifacts. It intentionally uses the existing
/// `min_area` threshold so projects can decide when a small object is relevant
/// to the process. Tang et al. (2023) discuss wet-etch limits for very fine
/// flexible PCB features; HyperDRC uses this as a review trigger rather than a
/// claim that the specific island cannot be fabricated.
pub fn tiny_layer_feature_readiness(
    layer_name: &str,
    sketch: &PcbSketch,
    min_area: f64,
) -> Vec<Violation> {
    if min_area <= 0.0 || !min_area.is_finite() {
        return Vec::new();
    }

    let multipolygon = sketch.to_multipolygon();
    let tiny_polygons = multipolygon
        .0
        .iter()
        .filter(|polygon| {
            let area = polygon.unsigned_area();
            area > 0.0 && area <= min_area
        })
        .cloned()
        .collect::<Vec<_>>();
    log::trace!(
        "tiny-layer feature readiness: layer={layer_name} polygons={} tiny_polygons={} min_area={min_area:.9}",
        multipolygon.0.len(),
        tiny_polygons.len()
    );

    if tiny_polygons.is_empty() {
        return Vec::new();
    }

    let tiny = MultiPolygon(tiny_polygons);
    let locations = duplicate_layer_locations(&tiny);
    let shapes = multipolygon_to_shapes(&tiny, 0.0);
    vec![Violation::new(
        "layer-sanity",
        Severity::Warning,
        vec![layer_name.to_string()],
        None,
        shapes,
        locations,
        Some(format!(
            "layer contains polygon islands at or below reportable feature area {min_area:.9}; review for tiny aperture flashes, fractured slivers, or stale artifacts"
        )),
    )]
}

/// Warn when a parsed layer contains long, skinny polygon islands.
///
/// This complements [`tiny_layer_feature_readiness`]: a route shard, overdrawn
/// line, or fragmented CAM export can have enough area to pass an area filter
/// while still being narrower than the process feature-width threshold. Tang et
/// al. (2023) describe fine-line etch sensitivity in flexible PCB processing;
/// HyperDRC uses a bounding-box width proxy here so the check remains cheap and
/// docs.rs-friendly rather than pretending to be a fabricator-specific etch
/// simulation.
pub fn skinny_layer_feature_readiness(
    layer_name: &str,
    sketch: &PcbSketch,
    min_width: f64,
    min_area: f64,
) -> Vec<Violation> {
    if min_width <= 0.0 || !min_width.is_finite() {
        return Vec::new();
    }

    let multipolygon = sketch.to_multipolygon();
    let skinny_polygons = multipolygon
        .0
        .iter()
        .filter(|polygon| {
            let area = polygon.unsigned_area();
            area > min_area
                && polygon_minimum_bounding_dimension(polygon)
                    .is_some_and(|dimension| dimension > 0.0 && dimension < min_width)
        })
        .cloned()
        .collect::<Vec<_>>();
    log::trace!(
        "skinny-layer feature readiness: layer={layer_name} polygons={} skinny_polygons={} min_width={min_width:.6} min_area={min_area:.9}",
        multipolygon.0.len(),
        skinny_polygons.len()
    );

    if skinny_polygons.is_empty() {
        return Vec::new();
    }

    let skinny = MultiPolygon(skinny_polygons);
    let locations = duplicate_layer_locations(&skinny);
    let shapes = multipolygon_to_shapes(&skinny, min_area);
    vec![Violation::new(
        "layer-sanity",
        Severity::Warning,
        vec![layer_name.to_string()],
        None,
        shapes,
        locations,
        Some(format!(
            "layer contains polygon islands whose minimum bounding dimension is below feature width {min_width:.6}; review for hairline fragments, slivers, or overdrawn route artifacts"
        )),
    )]
}

fn polygon_minimum_bounding_dimension(polygon: &Polygon<f64>) -> Option<f64> {
    let bounds = polygon.bounding_rect()?;
    let width = bounds.max().x - bounds.min().x;
    let height = bounds.max().y - bounds.min().y;
    Some(width.min(height))
}

/// Warn when one parsed layer contains duplicate polygon islands.
///
/// Intra-layer duplicates can come from repeated Gerber flashes, stale aperture
/// macro expansion, or CAM exports that wrote the same contour twice. The
/// geometry is usually harmless after boolean union, but it is still a release
/// readiness signal because duplicate primitives can confuse downstream CAM,
/// quoting, or review tools. The pairwise filter uses the same conservative
/// set-overlap idea as [`duplicate_layer_geometry_readiness`], following the
/// computational-geometry set-intersection framing surveyed by Lee and
/// Preparata (1984).
pub fn duplicate_layer_island_readiness(
    layer_name: &str,
    sketch: &PcbSketch,
    min_area: f64,
) -> Vec<Violation> {
    let multipolygon = sketch.to_multipolygon();
    let mut buckets = BTreeMap::<DuplicateIslandSignature, Vec<&Polygon<f64>>>::new();
    for polygon in &multipolygon.0 {
        if polygon.unsigned_area() <= min_area {
            continue;
        }
        let Some(signature) = duplicate_island_signature(polygon) else {
            continue;
        };
        buckets.entry(signature).or_default().push(polygon);
    }
    let comparable_polygons = buckets.values().map(Vec::len).sum::<usize>();
    log::trace!(
        "duplicate-layer island readiness: layer={layer_name} polygons={} comparable_polygons={} candidate_buckets={} min_area={min_area:.9}",
        multipolygon.0.len(),
        comparable_polygons,
        buckets.values().filter(|bucket| bucket.len() > 1).count()
    );

    let mut locations = Vec::new();
    for bucket in buckets.values().filter(|bucket| bucket.len() > 1) {
        for left_index in 0..bucket.len() {
            let mut matched_left = false;
            for right_index in (left_index + 1)..bucket.len() {
                if polygons_are_duplicate(
                    bucket[left_index],
                    bucket[right_index],
                    BOARD_OUTLINE_GEOMETRY_TOLERANCE,
                ) {
                    if let Some(location) = polygon_bounds_center(bucket[left_index]) {
                        push_unique_location(&mut locations, location);
                    }
                    matched_left = true;
                    break;
                }
            }
            if matched_left {
                continue;
            }
        }
    }

    if locations.is_empty() {
        return Vec::new();
    }

    vec![Violation::new(
        "layer-sanity",
        Severity::Warning,
        vec![layer_name.to_string()],
        None,
        Vec::new(),
        locations,
        Some(
            "layer contains duplicate polygon island geometry; review for repeated flashes, duplicated contours, or stale CAM artifacts"
                .to_string(),
        ),
    )]
}

#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
struct DuplicateIslandSignature {
    area: i64,
    min_x: i64,
    min_y: i64,
    max_x: i64,
    max_y: i64,
}

fn duplicate_island_signature(polygon: &Polygon<f64>) -> Option<DuplicateIslandSignature> {
    let bounds = polygon.bounding_rect()?;
    Some(DuplicateIslandSignature {
        area: quantize_layer_signature_value(polygon.unsigned_area()),
        min_x: quantize_layer_signature_value(bounds.min().x),
        min_y: quantize_layer_signature_value(bounds.min().y),
        max_x: quantize_layer_signature_value(bounds.max().x),
        max_y: quantize_layer_signature_value(bounds.max().y),
    })
}

fn quantize_layer_signature_value(value: f64) -> i64 {
    if !value.is_finite() {
        return 0;
    }
    (value * DUPLICATE_LAYER_SIGNATURE_SCALE).round() as i64
}

/// Warn when two parsed layers carry effectively identical polygon geometry.
///
/// Duplicate manufacturing layers are a package-readiness signal rather than a
/// proof of electrical failure: panel rails, mirrored sides, or intentional
/// copper symmetry can look similar. The check therefore lives under
/// `layer-sanity` as a warning and should be reviewed against the fabrication
/// file manifest. The overlap test follows the broad computational-geometry
/// set-intersection framing surveyed by Lee and Preparata (1984) while keeping
/// the final predicate intentionally conservative for CAM handoff use.
pub fn duplicate_layer_geometry_readiness(
    layers: &[(String, PcbSketch)],
    min_area: f64,
) -> Vec<Violation> {
    let prepared = layers
        .iter()
        .filter_map(|(name, sketch)| {
            let multipolygon = sketch.to_multipolygon();
            let area = multipolygon.unsigned_area();
            let bounds = multipolygon.bounding_rect()?;
            (area > min_area).then_some(DuplicateLayer {
                name,
                sketch,
                multipolygon,
                area,
                bounds,
            })
        })
        .collect::<Vec<_>>();
    log::trace!(
        "duplicate-layer geometry readiness: input_layers={} comparable_layers={} min_area={min_area:.9}",
        layers.len(),
        prepared.len()
    );

    let mut violations = Vec::new();
    for left_index in 0..prepared.len() {
        for right_index in (left_index + 1)..prepared.len() {
            let left = &prepared[left_index];
            let right = &prepared[right_index];
            if !areas_approximately_equal(left.area, right.area, BOARD_OUTLINE_GEOMETRY_TOLERANCE)
                || !rects_approximately_equal(
                    &left.bounds,
                    &right.bounds,
                    BOARD_OUTLINE_GEOMETRY_TOLERANCE,
                )
            {
                continue;
            }

            let overlap = left.sketch.intersection(right.sketch);
            let overlap_area = overlap.to_multipolygon().unsigned_area();
            let left_coverage = overlap_area / left.area;
            let right_coverage = overlap_area / right.area;
            if left_coverage < DUPLICATE_LAYER_OVERLAP_RATIO
                || right_coverage < DUPLICATE_LAYER_OVERLAP_RATIO
            {
                continue;
            }

            let shapes = multipolygon_to_shapes(&overlap.to_multipolygon(), min_area);
            let locations = duplicate_layer_locations(&left.multipolygon);
            violations.push(Violation::new(
                "layer-sanity",
                Severity::Warning,
                vec![left.name.to_string(), right.name.to_string()],
                None,
                shapes,
                locations,
                Some(format!(
                    "layers appear to contain duplicate geometry ({:.6}% overlap by area); review for duplicate or stale fabrication outputs",
                    left_coverage.min(right_coverage) * 100.0
                )),
            ));
        }
    }

    violations
}

struct DuplicateLayer<'a> {
    name: &'a str,
    sketch: &'a PcbSketch,
    multipolygon: MultiPolygon<f64>,
    area: f64,
    bounds: geo::Rect<f64>,
}

fn duplicate_layer_locations(multipolygon: &MultiPolygon<f64>) -> Vec<[f64; 2]> {
    multipolygon
        .0
        .iter()
        .filter_map(polygon_bounds_center)
        .take(8)
        .collect()
}

fn polygon_bounds_center(polygon: &Polygon<f64>) -> Option<[f64; 2]> {
    let bounds = polygon.bounding_rect()?;
    Some([
        (bounds.min().x + bounds.max().x) / 2.0,
        (bounds.min().y + bounds.max().y) / 2.0,
    ])
}

fn rects_approximately_equal(
    left: &geo::Rect<f64>,
    right: &geo::Rect<f64>,
    tolerance: f64,
) -> bool {
    (left.min().x - right.min().x).abs() <= tolerance
        && (left.min().y - right.min().y).abs() <= tolerance
        && (left.max().x - right.max().x).abs() <= tolerance
        && (left.max().y - right.max().y).abs() <= tolerance
}

fn multipolygon_has_non_finite_coordinates(multipolygon: &MultiPolygon<f64>) -> bool {
    for polygon in &multipolygon.0 {
        if !ring_has_finite_coordinates(polygon.exterior()) {
            return true;
        }

        for hole in polygon.interiors() {
            if !ring_has_finite_coordinates(hole) {
                return true;
            }
        }
    }

    false
}

fn ring_has_finite_coordinates(ring: &LineString<f64>) -> bool {
    ring.0
        .iter()
        .all(|coord| coord.x.is_finite() && coord.y.is_finite())
}

/// Run the `copper_balance` design-readiness check or report helper.
pub fn copper_balance(
    copper_layers: &[(String, PcbSketch)],
    max_imbalance_ratio: f64,
    min_area: f64,
) -> Vec<Violation> {
    let mut measured = copper_layers
        .iter()
        .filter_map(|(name, sketch)| {
            let area = sketch.to_multipolygon().unsigned_area();
            (area > min_area).then_some((name.clone(), area))
        })
        .collect::<Vec<_>>();

    if measured.len() < 2 {
        return Vec::new();
    }

    measured.sort_by(|left, right| {
        left.1
            .partial_cmp(&right.1)
            .unwrap_or(std::cmp::Ordering::Equal)
    });
    let (smallest_layer, smallest_area) = &measured[0];
    let (largest_layer, largest_area) = &measured[measured.len() - 1];
    let ratio = largest_area / smallest_area;

    if ratio <= max_imbalance_ratio {
        return Vec::new();
    }

    vec![Violation::new(
        "copper-balance-readiness",
        Severity::Warning,
        vec![smallest_layer.clone(), largest_layer.clone()],
        None,
        Vec::new(),
        Vec::new(),
        Some(format!(
            "copper area imbalance ratio {ratio:.3} exceeds maximum {max_imbalance_ratio:.3}; smallest layer {smallest_layer} area {smallest_area:.6}, largest layer {largest_layer} area {largest_area:.6}"
        )),
    )]
}

/// Run a local copper-density balance check over matching windows on each layer.
///
/// The global [`copper_balance`] check catches whole-layer area mismatch. This
/// helper catches the more local DFM case called out in the design-readiness
/// plan: a dense copper island on one layer with sparse copper in the same board
/// region on another layer. It is still a readiness heuristic rather than a CAM
/// compensation model; the review signal is useful because copper pattern
/// density influences etch and copper plating uniformity in PCB production.
///
/// The implementation intentionally uses rectangular windows so the result is
/// deterministic, inexpensive, and friendly to docs.rs examples/tests. For the
/// manufacturing motivation, see Sun et al., "Multi-physics coupling aid
/// uniformity improvement in pattern plating," *Applied Thermal Engineering*
/// 108 (2016), and Lee et al., "Effect of pulse-reverse plating on copper:
/// Thermal mechanical properties and microstructure relationship,"
/// *Microelectronics Reliability* 100-101 (2019).
pub fn local_copper_density_readiness(
    copper_layers: &[(String, PcbSketch)],
    window_size: f64,
    max_density_ratio: f64,
    min_area: f64,
) -> Vec<Violation> {
    if copper_layers.len() < 2 || window_size <= 0.0 {
        return Vec::new();
    }

    let prepared_layers = prepare_density_layers(copper_layers, min_area);
    let Some(bounds) = combined_density_bounds(&prepared_layers) else {
        return Vec::new();
    };
    log::trace!(
        "local copper-density readiness: layers={} window_size={window_size:.6} ratio={max_density_ratio:.6}",
        prepared_layers.len()
    );

    let mut violations = Vec::new();
    let min_x = bounds.min().x;
    let min_y = bounds.min().y;
    let max_x = bounds.max().x;
    let max_y = bounds.max().y;
    let mut y = min_y;

    while y < max_y {
        let mut x = min_x;
        while x < max_x {
            let width = (max_x - x).min(window_size);
            let height = (max_y - y).min(window_size);
            if width * height > min_area {
                collect_local_density_window(
                    &prepared_layers,
                    [x + width / 2.0, y + height / 2.0],
                    [width, height],
                    max_density_ratio,
                    min_area,
                    &mut violations,
                );
            }
            x += window_size;
        }
        y += window_size;
    }

    violations
}

struct DensityLayer {
    name: String,
    polygons: Vec<DensityPolygon>,
}

struct DensityPolygon {
    bounds: geo::Rect<f64>,
    area: f64,
}

fn prepare_density_layers(
    copper_layers: &[(String, PcbSketch)],
    min_area: f64,
) -> Vec<DensityLayer> {
    copper_layers
        .iter()
        .filter_map(|(name, sketch)| {
            let polygons = sketch
                .to_multipolygon()
                .0
                .into_iter()
                .filter_map(|polygon| {
                    let area = polygon.unsigned_area();
                    let bounds = polygon.bounding_rect()?;
                    (area > min_area).then_some(DensityPolygon { bounds, area })
                })
                .collect::<Vec<_>>();
            (!polygons.is_empty()).then_some(DensityLayer {
                name: name.clone(),
                polygons,
            })
        })
        .collect()
}

fn collect_local_density_window(
    copper_layers: &[DensityLayer],
    center: [f64; 2],
    size: [f64; 2],
    max_density_ratio: f64,
    min_area: f64,
    violations: &mut Vec<Violation>,
) {
    let window_polygon = rect_polygon(center, size, 0.0);
    let window_area = window_polygon.unsigned_area();
    if window_area <= min_area {
        return;
    }
    let Some(window_bounds) = window_polygon.bounding_rect() else {
        return;
    };
    let mut densities = copper_layers
        .iter()
        .map(|layer| {
            let copper_area = layer
                .polygons
                .iter()
                .map(|polygon| approximate_window_polygon_area(&window_bounds, polygon))
                .sum::<f64>();
            (&layer.name, (copper_area / window_area).clamp(0.0, 1.0))
        })
        .collect::<Vec<_>>();

    densities.sort_by(|left, right| {
        left.1
            .partial_cmp(&right.1)
            .unwrap_or(std::cmp::Ordering::Equal)
    });
    let Some((sparse_layer, sparse_density)) = densities.first() else {
        return;
    };
    let Some((dense_layer, dense_density)) = densities.last() else {
        return;
    };
    if *dense_density < LOCAL_COPPER_DENSITY_FLOOR {
        return;
    }

    let ratio = *dense_density / sparse_density.max(min_area);
    let delta = *dense_density - *sparse_density;
    if ratio <= max_density_ratio || delta < LOCAL_COPPER_DENSITY_DELTA {
        return;
    }

    let window = polygons_to_sketch(vec![window_polygon], Some(metadata("density window")));
    let shapes = multipolygon_to_shapes(&window.to_multipolygon(), min_area);
    violations.push(Violation::new(
        "local-copper-density-readiness",
        Severity::Warning,
        vec![(*sparse_layer).clone(), (*dense_layer).clone()],
        None,
        shapes,
        vec![center],
        Some(format!(
            "local copper density imbalance ratio {ratio:.3} exceeds maximum {max_density_ratio:.3}; sparse layer {sparse_layer} density {sparse_density:.3}, dense layer {dense_layer} density {dense_density:.3}"
        )),
    ));
}

fn approximate_window_polygon_area(window: &geo::Rect<f64>, polygon: &DensityPolygon) -> f64 {
    let overlap_width = (window.max().x.min(polygon.bounds.max().x)
        - window.min().x.max(polygon.bounds.min().x))
    .max(0.0);
    let overlap_height = (window.max().y.min(polygon.bounds.max().y)
        - window.min().y.max(polygon.bounds.min().y))
    .max(0.0);
    if overlap_width == 0.0 || overlap_height == 0.0 {
        return 0.0;
    }

    let bounds_area = (polygon.bounds.max().x - polygon.bounds.min().x)
        * (polygon.bounds.max().y - polygon.bounds.min().y);
    if bounds_area <= 0.0 {
        return 0.0;
    }

    // The local density check is a DFM heuristic, so it uses a conservative
    // raster-like area accumulator instead of exact CSG window clipping. This
    // mirrors the gridded density maps used for copper CMP/plating review while
    // avoiding a pathological "number of windows times number of layers" boolean
    // workload on large pours. See Kahng et al., "Filling Algorithms and
    // Analyses for Layout Density Control", IEEE TCAD, 1999.
    polygon.area * (overlap_width * overlap_height / bounds_area)
}

fn combined_density_bounds(copper_layers: &[DensityLayer]) -> Option<geo::Rect<f64>> {
    copper_layers
        .iter()
        .flat_map(|layer| layer.polygons.iter().map(|polygon| polygon.bounds))
        .reduce(|left, right| {
            geo::Rect::new(
                Coord {
                    x: left.min().x.min(right.min().x),
                    y: left.min().y.min(right.min().y),
                },
                Coord {
                    x: left.max().x.max(right.max().x),
                    y: left.max().y.max(right.max().y),
                },
            )
        })
}

/// Run the `mechanical_layer_geometry` design-readiness check or report helper.
pub fn mechanical_layer_geometry(
    layer_name: &str,
    sketch: &PcbSketch,
    min_area: f64,
) -> Vec<Violation> {
    if !looks_like_mechanical_layer(layer_name) {
        return Vec::new();
    }

    let shapes = multipolygon_to_shapes(&sketch.to_multipolygon(), min_area);
    if shapes.is_empty() {
        return Vec::new();
    }

    vec![Violation::new(
        "mechanical-layer-geometry",
        Severity::Warning,
        vec![layer_name.to_string()],
        None,
        shapes,
        Vec::new(),
        Some("geometry is present on a mechanical/user layer".to_string()),
    )]
}

/// Run the `board_outline_sanity` design-readiness check or report helper.
pub fn board_outline_sanity(
    layer_name: &str,
    outline: &PcbSketch,
    min_area: f64,
) -> Vec<Violation> {
    let shapes = multipolygon_to_shapes(&outline.to_multipolygon(), min_area);
    if !shapes.is_empty() {
        return Vec::new();
    }

    vec![Violation::new(
        "board-outline-sanity",
        Severity::Warning,
        vec![layer_name.to_string()],
        None,
        Vec::new(),
        Vec::new(),
        Some("board outline layer has no closed polygon area".to_string()),
    )]
}

/// Run the `board_outline_fragments` design-readiness check or report helper.
pub fn board_outline_fragments(
    layer_name: &str,
    outline: &PcbSketch,
    min_area: f64,
) -> Vec<Violation> {
    let shapes = multipolygon_to_shapes(&outline.to_multipolygon(), min_area);
    if shapes.len() <= 1 {
        return Vec::new();
    }

    vec![Violation::new(
        "board-outline-fragments",
        Severity::Warning,
        vec![layer_name.to_string()],
        None,
        shapes,
        Vec::new(),
        Some("board outline parsed to multiple disconnected regions".to_string()),
    )]
}

/// Reject outline rings that self-intersect, which usually produces an invalid
/// profile for profile-based CAM preparation.
pub fn board_outline_self_intersection_readiness(
    layer_name: &str,
    outline: &PcbSketch,
) -> Vec<Violation> {
    let intersections = collect_ring_self_intersections(&outline.to_multipolygon());
    if intersections.is_empty() {
        return Vec::new();
    }

    vec![Violation::new(
        "board-outline-self-intersection-readiness",
        Severity::Error,
        vec![layer_name.to_string()],
        None,
        Vec::new(),
        intersections,
        Some("board outline contains self-intersecting contour edges".to_string()),
    )]
}

/// Flag strong inside-corners on board outlines where a narrow notch is likely to
/// exceed router capability.
pub fn board_outline_notch_readiness(layer_name: &str, outline: &PcbSketch) -> Vec<Violation> {
    let mut locations = Vec::new();

    let multipolygon = outline.to_multipolygon();
    for polygon in &multipolygon.0 {
        collect_board_outline_notches(polygon.exterior(), &mut locations);
        for hole in polygon.interiors() {
            collect_board_outline_notches(hole, &mut locations);
        }
    }

    if locations.is_empty() {
        return Vec::new();
    }

    vec![Violation::new(
        "board-outline-notch-readiness",
        Severity::Warning,
        vec![layer_name.to_string()],
        None,
        Vec::new(),
        locations,
        Some("board outline contains sharp notch inside-corners".to_string()),
    )]
}

/// Warn when the outline contains duplicated contour polygons that would indicate
/// accidental repeated or merged contour definitions.
pub fn board_outline_duplicate_readiness(layer_name: &str, outline: &PcbSketch) -> Vec<Violation> {
    let mut locations = Vec::new();

    collect_board_outline_overlapping_exteriors(
        &outline.to_multipolygon(),
        BOARD_OUTLINE_DUPLICATE_OVERLAP_RATIO,
        BOARD_OUTLINE_GEOMETRY_TOLERANCE,
        false,
        &mut locations,
    );

    if locations.is_empty() {
        return Vec::new();
    }

    vec![Violation::new(
        "board-outline-duplicate-readiness",
        Severity::Warning,
        vec![layer_name.to_string()],
        None,
        Vec::new(),
        locations,
        Some("board outline contains duplicate contour geometry".to_string()),
    )]
}

/// Warn when one contour is fully contained by another, which can indicate
/// malformed nested board cutouts or accidental profile duplication.
pub fn board_outline_nesting_readiness(layer_name: &str, outline: &PcbSketch) -> Vec<Violation> {
    let mut locations = Vec::new();

    collect_board_outline_overlapping_exteriors(
        &outline.to_multipolygon(),
        BOARD_OUTLINE_NESTED_OVERLAP_RATIO,
        BOARD_OUTLINE_GEOMETRY_TOLERANCE,
        true,
        &mut locations,
    );

    if locations.is_empty() {
        return Vec::new();
    }

    vec![Violation::new(
        "board-outline-nesting-readiness",
        Severity::Warning,
        vec![layer_name.to_string()],
        None,
        Vec::new(),
        locations,
        Some("board outline contains nested contour geometry".to_string()),
    )]
}

fn intersection_violation(
    spec: PairCheck<'_>,
    left_name: &str,
    left: &PcbSketch,
    right_name: &str,
    right: &PcbSketch,
    min_area: f64,
) -> Vec<Violation> {
    let overlap = left.intersection(right);
    shapes_violation(
        spec.check,
        spec.severity,
        vec![left_name.to_string(), right_name.to_string()],
        overlap,
        min_area,
        spec.message.to_string(),
    )
}

const BOARD_OUTLINE_NOTCH_ANGLE_DEGREES: f64 = 300.0;
const BOARD_OUTLINE_GEOMETRY_TOLERANCE: f64 = 1.0e-6;
const BOARD_OUTLINE_DUPLICATE_OVERLAP_RATIO: f64 = 0.999_999;
const BOARD_OUTLINE_NESTED_OVERLAP_RATIO: f64 = 0.999_99;

fn collect_ring_self_intersections(multipolygon: &MultiPolygon<f64>) -> Vec<[f64; 2]> {
    let mut locations = Vec::new();

    for polygon in &multipolygon.0 {
        collect_segment_self_intersections(polygon.exterior(), &mut locations);
        for hole in polygon.interiors() {
            collect_segment_self_intersections(hole, &mut locations);
        }
    }

    locations
}

fn collect_segment_self_intersections(ring: &LineString<f64>, locations: &mut Vec<[f64; 2]>) {
    let coords = open_ring_coords(ring);
    if coords.len() < 4 {
        return;
    }

    let edge_count = coords.len();
    for left in 0..edge_count {
        for right in (left + 1)..edge_count {
            if are_ring_edges_adjacent(left, right, edge_count) {
                continue;
            }

            let intersection = ring_segment_intersection(
                coords[left],
                coords[(left + 1) % edge_count],
                coords[right],
                coords[(right + 1) % edge_count],
            );

            if let Some(location) = intersection {
                push_unique_location(locations, location);
            }
        }
    }
}

fn collect_board_outline_notches(ring: &LineString<f64>, locations: &mut Vec<[f64; 2]>) {
    let coords = open_ring_coords(ring);
    if coords.len() < 3 {
        return;
    }

    let is_ccw = ring_is_ccw(ring);
    for index in 0..coords.len() {
        let previous = coords[(index + coords.len() - 1) % coords.len()];
        let current = coords[index];
        let next = coords[(index + 1) % coords.len()];

        let Some(interior_angle) =
            board_outline_notch_interior_angle(previous, current, next, is_ccw)
        else {
            continue;
        };
        if interior_angle < BOARD_OUTLINE_NOTCH_ANGLE_DEGREES {
            continue;
        }

        push_unique_location(locations, [current.x, current.y]);
    }
}

fn collect_board_outline_overlapping_exteriors(
    multipolygon: &MultiPolygon<f64>,
    containment_ratio: f64,
    geometry_tolerance: f64,
    detect_nesting: bool,
    locations: &mut Vec<[f64; 2]>,
) {
    let polygons = &multipolygon.0;
    if polygons.len() < 2 {
        return;
    }

    for outer_index in 0..polygons.len() {
        for inner_index in (outer_index + 1)..polygons.len() {
            let outer = &polygons[outer_index];
            let inner = &polygons[inner_index];

            if detect_nesting {
                if polygons_are_duplicate(outer, inner, geometry_tolerance) {
                    continue;
                }
                if polygon_contains_other_outer(outer, inner, containment_ratio, geometry_tolerance)
                {
                    if let Some(point) = representative_point(inner) {
                        push_unique_location(locations, point);
                    }
                }

                if polygon_contains_other_outer(inner, outer, containment_ratio, geometry_tolerance)
                {
                    if let Some(point) = representative_point(outer) {
                        push_unique_location(locations, point);
                    }
                }
            } else if polygons_are_duplicate(outer, inner, geometry_tolerance) {
                if let Some(point) = representative_point(outer) {
                    push_unique_location(locations, point);
                }
            }
        }
    }
}

fn polygons_are_duplicate(left: &Polygon<f64>, right: &Polygon<f64>, tolerance: f64) -> bool {
    let left_area = left.unsigned_area();
    let right_area = right.unsigned_area();
    if left_area <= 0.0 || right_area <= 0.0 {
        return false;
    }

    if !areas_approximately_equal(left_area, right_area, tolerance) {
        return false;
    }

    let overlap = polygon_intersection_area(left, right);
    let left_delta = (left_area - overlap).abs();
    let right_delta = (right_area - overlap).abs();
    left_delta <= tolerance_area(left_area) && right_delta <= tolerance_area(right_area)
}

fn polygon_contains_other_outer(
    outer: &Polygon<f64>,
    inner: &Polygon<f64>,
    ratio: f64,
    tolerance: f64,
) -> bool {
    let outer_area = outer.unsigned_area();
    let inner_area = inner.unsigned_area();
    if outer_area <= 0.0 || inner_area <= 0.0 || outer_area <= inner_area {
        return false;
    }

    let overlap = polygon_intersection_area(outer, inner);
    if overlap <= inner_area * 0.25 {
        return false;
    }

    let coverage = overlap / inner_area;
    let area_gap = outer_area - inner_area;
    coverage >= ratio
        && area_gap > tolerance_area(outer_area)
        && !areas_approximately_equal(outer_area, inner_area, tolerance)
}

fn polygon_intersection_area(left: &Polygon<f64>, right: &Polygon<f64>) -> f64 {
    let left_sketch = polygon_to_sketch(left.clone(), None);
    let right_sketch = polygon_to_sketch(right.clone(), None);
    left_sketch
        .intersection(&right_sketch)
        .to_multipolygon()
        .unsigned_area()
}

fn representative_point(polygon: &Polygon<f64>) -> Option<[f64; 2]> {
    polygon.bounding_rect().map(|bounds| {
        [
            (bounds.min().x + bounds.max().x) / 2.0,
            (bounds.min().y + bounds.max().y) / 2.0,
        ]
    })
}

fn tolerance_area(area: f64) -> f64 {
    (area.abs() * 1.0e-9).max(1.0e-12)
}

fn areas_approximately_equal(left_area: f64, right_area: f64, tolerance: f64) -> bool {
    let diff = (left_area - right_area).abs();
    let scale = left_area.abs().max(right_area.abs()).max(1.0);
    diff <= tolerance * scale
}

fn board_outline_notch_interior_angle(
    previous: Coord<f64>,
    current: Coord<f64>,
    next: Coord<f64>,
    is_ccw: bool,
) -> Option<f64> {
    let forward_one = vector(current, previous);
    let forward_two = vector(next, current);
    if vector_length(forward_one) == 0.0 || vector_length(forward_two) == 0.0 {
        return None;
    }

    let cross = cross_product(forward_one, forward_two);
    let dot = dot_product(forward_one, forward_two);
    let raw_turn = cross.atan2(dot).to_degrees();
    let is_reflex = if is_ccw {
        raw_turn < 0.0
    } else {
        raw_turn > 0.0
    };
    if !is_reflex {
        return None;
    }

    Some(360.0 - raw_turn.abs())
}

fn ring_is_ccw(ring: &LineString<f64>) -> bool {
    Polygon::new(ring.clone(), vec![]).signed_area() >= 0.0
}

fn are_ring_edges_adjacent(left: usize, right: usize, edge_count: usize) -> bool {
    right == left + 1 || right + 1 == left || (left == 0 && right == edge_count - 1)
}

fn ring_segment_intersection(
    start_a: Coord<f64>,
    end_a: Coord<f64>,
    start_b: Coord<f64>,
    end_b: Coord<f64>,
) -> Option<[f64; 2]> {
    let segment_a = Line::new(start_a, end_a);
    let segment_b = Line::new(start_b, end_b);
    let intersection = line_intersection(segment_a, segment_b)?;

    match intersection {
        LineIntersection::SinglePoint {
            intersection: point,
            is_proper: false,
        } if point == start_a || point == end_a || point == start_b || point == end_b => None,
        LineIntersection::SinglePoint {
            intersection: point,
            ..
        } => Some([point.x, point.y]),
        LineIntersection::Collinear { intersection } => Some([
            (intersection.start.x + intersection.end.x) / 2.0,
            (intersection.start.y + intersection.end.y) / 2.0,
        ]),
    }
}

fn push_unique_location(points: &mut Vec<[f64; 2]>, point: [f64; 2]) {
    if !points
        .iter()
        .any(|current| location_is_close(current, &point))
    {
        points.push(point);
    }
}

fn location_is_close(left: &[f64; 2], right: &[f64; 2]) -> bool {
    const EPSILON: f64 = 1e-9;
    (left[0] - right[0]).abs() < EPSILON && (left[1] - right[1]).abs() < EPSILON
}

fn vector(end: Coord<f64>, start: Coord<f64>) -> (f64, f64) {
    (end.x - start.x, end.y - start.y)
}

fn cross_product(left: (f64, f64), right: (f64, f64)) -> f64 {
    left.0 * right.1 - left.1 * right.0
}

fn dot_product(left: (f64, f64), right: (f64, f64)) -> f64 {
    left.0 * right.0 + left.1 * right.1
}

fn vector_length(vector: (f64, f64)) -> f64 {
    (vector.0 * vector.0 + vector.1 * vector.1).sqrt()
}

struct PairCheck<'a> {
    check: &'a str,
    severity: Severity,
    message: &'a str,
}

fn shapes_violation(
    check: &str,
    severity: Severity,
    layers: Vec<String>,
    sketch: PcbSketch,
    min_area: f64,
    message: String,
) -> Vec<Violation> {
    let shapes = multipolygon_to_shapes(&sketch.to_multipolygon(), min_area);

    if shapes.is_empty() {
        return Vec::new();
    }

    vec![Violation::new(
        check,
        severity,
        layers,
        None,
        shapes,
        Vec::new(),
        Some(message),
    )]
}

fn collect_acute_vertices(
    ring: &LineString<f64>,
    max_angle_degrees: f64,
    locations: &mut Vec<[f64; 2]>,
) {
    let coords = open_ring_coords(ring);
    if coords.len() < 3 {
        return;
    }

    for index in 0..coords.len() {
        let previous = coords[(index + coords.len() - 1) % coords.len()];
        let current = coords[index];
        let next = coords[(index + 1) % coords.len()];
        // This is a local vertex-angle heuristic, not a full manufacturability
        // proof. It intentionally reports candidates for review because acute
        // copper notches can be caused by polygon decomposition as well as by
        // intentional footprint geometry.
        let angle = angle_degrees(previous, current, next);

        if angle > 0.0 && angle < max_angle_degrees {
            locations.push([current.x, current.y]);
        }
    }
}

fn open_ring_coords(ring: &LineString<f64>) -> Vec<Coord<f64>> {
    let mut coords = ring.0.clone();
    if coords.len() > 1 && coords.first() == coords.last() {
        coords.pop();
    }
    coords
}

fn angle_degrees(previous: Coord<f64>, current: Coord<f64>, next: Coord<f64>) -> f64 {
    let ax = previous.x - current.x;
    let ay = previous.y - current.y;
    let bx = next.x - current.x;
    let by = next.y - current.y;
    let dot = ax * bx + ay * by;
    let a_len = (ax * ax + ay * ay).sqrt();
    let b_len = (bx * bx + by * by).sqrt();

    if a_len == 0.0 || b_len == 0.0 {
        return 0.0;
    }

    let cos = (dot / (a_len * b_len)).clamp(-1.0, 1.0);
    cos.acos().to_degrees()
}

fn looks_like_mechanical_layer(layer_name: &str) -> bool {
    let lower = layer_name.to_ascii_lowercase();
    lower.contains("mechanical")
        || lower.contains("mech")
        || lower.contains("user.")
        || lower.contains("dwgs.user")
        || lower.contains("cmts.user")
        || lower.contains("fab")
        || lower.contains("eco")
        || lower.contains("margin")
}

fn metadata(layer_name: &str) -> LayerMetadata {
    LayerMetadata {
        name: layer_name.to_string(),
    }
}

#[cfg(test)]
mod tests {
    use std::path::Path;
    use std::process::Command;
    use std::time::{Duration, Instant};

    use geo::{Coord, LineString, Polygon};

    use super::{
        acid_trap_candidates, board_edge_clearance, board_outline_cutout_clearance,
        board_outline_duplicate_readiness, board_outline_fragments,
        board_outline_nesting_readiness, board_outline_notch_readiness, board_outline_sanity,
        board_outline_self_intersection_readiness, copper_balance, copper_overlap,
        duplicate_layer_geometry_readiness, duplicate_layer_island_readiness, exposed_copper,
        layer_sanity, local_copper_density_readiness, mask_island_keepout,
        mechanical_layer_geometry, min_copper_neck_width, minimum_mask_opening,
        minimum_paste_aperture, paste_aperture_coverage, paste_aperture_ratio,
        paste_aperture_spacing, paste_mask_alignment, paste_overhang,
        silkscreen_board_edge_clearance, silkscreen_clearance, silkscreen_min_width,
        silkscreen_overlap, skinny_layer_feature_readiness, solder_mask_board_edge_clearance,
        solder_mask_expansion, solder_mask_opening_coverage, solder_mask_opening_spacing,
        solder_mask_overlap_clearance, solder_mask_sliver, tiny_layer_feature_readiness,
    };
    use crate::LayerMetadata;
    use crate::geometry::{empty_sketch, line_polygon, polygons_to_sketch, rect_polygon};
    use crate::kicad::load_kicad_pcb;

    #[test]
    fn mask_island_keepout_reports_expanded_island_collision() {
        let layer = sketch(
            "mask",
            vec![square(0.0, 0.0, 1.0, 1.0), square(1.1, 0.0, 2.1, 1.0)],
        );

        let violations = mask_island_keepout("mask", &layer, 0.1, 1.0e-9);

        assert_eq!(violations.len(), 1);
        assert!(
            violations
                .iter()
                .all(|violation| violation.total_area > 0.0)
        );
    }

    #[test]
    fn mask_island_keepout_allows_distant_islands() {
        let layer = sketch(
            "mask",
            vec![square(0.0, 0.0, 1.0, 1.0), square(5.0, 0.0, 6.0, 1.0)],
        );

        let violations = mask_island_keepout("mask", &layer, 0.1, 1.0e-9);

        assert!(violations.is_empty());
    }

    #[test]
    fn copper_overlap_reports_intersection_coordinates() {
        let top = sketch("top", vec![square(0.0, 0.0, 2.0, 2.0)]);
        let bottom = sketch("bottom", vec![square(1.0, 1.0, 3.0, 3.0)]);

        let violations = copper_overlap("top", &top, "bottom", &bottom, 1.0e-9);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].polygons.len(), 1);
        assert!((violations[0].polygons[0].area - 1.0).abs() < 1.0e-9);
    }

    #[test]
    fn copper_balance_reports_large_area_imbalance() {
        let layers = vec![
            (
                "F.Cu".to_string(),
                sketch("F.Cu", vec![square(0.0, 0.0, 1.0, 1.0)]),
            ),
            (
                "B.Cu".to_string(),
                sketch("B.Cu", vec![square(0.0, 0.0, 4.0, 4.0)]),
            ),
        ];

        let violations = copper_balance(&layers, 3.0, 1.0e-9);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "copper-balance-readiness");
    }

    #[test]
    fn copper_balance_allows_similar_or_single_sided_inputs() {
        let balanced = vec![
            (
                "F.Cu".to_string(),
                sketch("F.Cu", vec![square(0.0, 0.0, 2.0, 2.0)]),
            ),
            (
                "B.Cu".to_string(),
                sketch("B.Cu", vec![square(0.0, 0.0, 2.5, 2.0)]),
            ),
        ];
        let single = vec![(
            "F.Cu".to_string(),
            sketch("F.Cu", vec![square(0.0, 0.0, 2.0, 2.0)]),
        )];

        assert!(copper_balance(&balanced, 3.0, 1.0e-9).is_empty());
        assert!(copper_balance(&single, 3.0, 1.0e-9).is_empty());
    }

    #[test]
    fn local_copper_density_reports_dense_island_over_sparse_layer() {
        let layers = vec![
            (
                "F.Cu".to_string(),
                polygons_to_sketch(
                    vec![rect_polygon([5.0, 5.0], [10.0, 10.0], 0.0)],
                    Some(LayerMetadata {
                        name: "F.Cu".to_string(),
                    }),
                ),
            ),
            (
                "B.Cu".to_string(),
                polygons_to_sketch(
                    vec![rect_polygon([5.0, 5.0], [1.0, 1.0], 0.0)],
                    Some(LayerMetadata {
                        name: "B.Cu".to_string(),
                    }),
                ),
            ),
        ];

        let violations = local_copper_density_readiness(&layers, 10.0, 3.0, 1.0e-9);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "local-copper-density-readiness");
        assert!(
            violations[0]
                .message
                .as_deref()
                .is_some_and(|message| message.contains("density imbalance ratio"))
        );
    }

    #[test]
    fn local_copper_density_allows_balanced_matching_windows() {
        let layers = vec![
            (
                "F.Cu".to_string(),
                polygons_to_sketch(
                    vec![rect_polygon([5.0, 5.0], [7.0, 7.0], 0.0)],
                    Some(LayerMetadata {
                        name: "F.Cu".to_string(),
                    }),
                ),
            ),
            (
                "B.Cu".to_string(),
                polygons_to_sketch(
                    vec![rect_polygon([5.0, 5.0], [6.0, 6.0], 0.0)],
                    Some(LayerMetadata {
                        name: "B.Cu".to_string(),
                    }),
                ),
            ),
        ];

        assert!(local_copper_density_readiness(&layers, 10.0, 3.0, 1.0e-9).is_empty());
    }

    #[test]
    fn board_edge_clearance_reports_copper_outside_eroded_outline() {
        let board = sketch("edge", vec![square(0.0, 0.0, 10.0, 10.0)]);
        let copper = sketch("top", vec![square(0.1, 0.1, 1.0, 1.0)]);

        let violations = board_edge_clearance("top", &copper, "edge", &board, 0.25, 1.0e-9);

        assert_eq!(violations.len(), 1);
    }

    #[test]
    fn silkscreen_board_edge_clearance_reports_legend_near_edge() {
        let board = sketch("edge", vec![square(0.0, 0.0, 10.0, 10.0)]);
        let silk = sketch("silk", vec![square(0.1, 0.1, 1.0, 1.0)]);

        let violations =
            silkscreen_board_edge_clearance("silk", &silk, "edge", &board, 0.25, 1.0e-9);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "silkscreen-to-board-edge-clearance");
    }

    #[test]
    fn silkscreen_board_edge_clearance_allows_inset_legend() {
        let board = sketch("edge", vec![square(0.0, 0.0, 10.0, 10.0)]);
        let silk = sketch("silk", vec![square(1.0, 1.0, 2.0, 2.0)]);

        assert!(
            silkscreen_board_edge_clearance("silk", &silk, "edge", &board, 0.25, 1.0e-9).is_empty()
        );
    }

    #[test]
    fn solder_mask_board_edge_clearance_reports_opening_near_edge() {
        let board = sketch("edge", vec![square(0.0, 0.0, 10.0, 10.0)]);
        let mask = sketch("mask", vec![square(0.1, 0.1, 1.0, 1.0)]);

        let violations =
            solder_mask_board_edge_clearance("mask", &mask, "edge", &board, 0.25, 1.0e-9);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "solder-mask-to-board-edge-clearance");
    }

    #[test]
    fn solder_mask_board_edge_clearance_allows_inset_opening() {
        let board = sketch("edge", vec![square(0.0, 0.0, 10.0, 10.0)]);
        let mask = sketch("mask", vec![square(1.0, 1.0, 2.0, 2.0)]);

        assert!(
            solder_mask_board_edge_clearance("mask", &mask, "edge", &board, 0.25, 1.0e-9)
                .is_empty()
        );
    }

    #[test]
    fn paste_overhang_reports_paste_outside_copper() {
        let copper = sketch("top", vec![square(0.0, 0.0, 1.0, 1.0)]);
        let paste = sketch("paste", vec![square(-0.1, 0.0, 1.0, 1.0)]);

        let violations = paste_overhang("paste", &paste, "top", &copper, 0.0, 1.0e-9);

        assert_eq!(violations.len(), 1);
    }

    #[test]
    fn paste_aperture_coverage_reports_undersized_or_missing_apertures() {
        let copper = sketch(
            "top",
            vec![square(0.0, 0.0, 1.0, 1.0), square(2.0, 0.0, 3.0, 1.0)],
        );
        let paste = sketch("paste", vec![square(0.1, 0.1, 0.9, 0.9)]);

        let violations = paste_aperture_coverage("paste", &paste, "top", &copper, 1.0e-9);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "paste-aperture-coverage");
    }

    #[test]
    fn paste_aperture_coverage_allows_full_apertures() {
        let copper = sketch("top", vec![square(0.0, 0.0, 1.0, 1.0)]);
        let paste = sketch("paste", vec![square(-0.1, -0.1, 1.1, 1.1)]);

        let violations = paste_aperture_coverage("paste", &paste, "top", &copper, 1.0e-9);

        assert!(violations.is_empty());
    }

    #[test]
    fn paste_aperture_ratio_reports_under_and_over_pasted_islands() {
        let copper = sketch(
            "top",
            vec![square(0.0, 0.0, 1.0, 1.0), square(2.0, 0.0, 3.0, 1.0)],
        );
        let paste = sketch(
            "paste",
            vec![square(0.0, 0.0, 0.25, 1.0), square(1.9, -0.1, 3.1, 1.1)],
        );

        let violations = paste_aperture_ratio("paste", &paste, "top", &copper, 0.5, 1.2, 1.0e-9);

        assert_eq!(violations.len(), 2);
        assert!(
            violations
                .iter()
                .all(|violation| violation.check == "paste-aperture-ratio")
        );
    }

    #[test]
    fn paste_aperture_ratio_allows_configured_ratio_range() {
        let copper = sketch("top", vec![square(0.0, 0.0, 1.0, 1.0)]);
        let paste = sketch("paste", vec![square(0.0, 0.0, 0.8, 1.0)]);

        assert!(paste_aperture_ratio("paste", &paste, "top", &copper, 0.5, 1.2, 1.0e-9).is_empty());
    }

    #[test]
    fn minimum_paste_aperture_reports_too_narrow_apertures() {
        let paste = sketch("paste", vec![square(0.0, 0.0, 0.05, 0.3)]);

        let violations = minimum_paste_aperture("paste", &paste, 0.1, 1.0e-9);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "minimum-paste-aperture");
    }

    #[test]
    fn minimum_paste_aperture_allows_large_apertures() {
        let paste = sketch("paste", vec![square(0.0, 0.0, 0.2, 0.3)]);

        assert!(minimum_paste_aperture("paste", &paste, 0.1, 1.0e-9).is_empty());
    }

    #[test]
    fn paste_aperture_spacing_reports_close_apertures() {
        let paste = sketch(
            "paste",
            vec![square(0.0, 0.0, 1.0, 1.0), square(1.05, 0.0, 2.05, 1.0)],
        );

        let violations = paste_aperture_spacing("paste", &paste, 0.1, 1.0e-9);

        assert_eq!(violations.len(), 2);
        assert!(
            violations
                .iter()
                .all(|violation| violation.check == "paste-aperture-spacing")
        );
    }

    #[test]
    fn paste_aperture_spacing_allows_compliant_apertures() {
        let paste = sketch(
            "paste",
            vec![square(0.0, 0.0, 1.0, 1.0), square(1.2, 0.0, 2.2, 1.0)],
        );

        assert!(paste_aperture_spacing("paste", &paste, 0.1, 1.0e-9).is_empty());
    }

    #[test]
    fn paste_mask_alignment_reports_paste_outside_mask_opening() {
        let paste = sketch("paste", vec![square(0.0, 0.0, 1.0, 1.0)]);
        let mask = sketch("mask", vec![square(0.1, 0.0, 1.0, 1.0)]);

        let violations = paste_mask_alignment("paste", &paste, "mask", &mask, 1.0e-9);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "paste-mask-alignment");
    }

    #[test]
    fn paste_mask_alignment_allows_paste_inside_mask_opening() {
        let paste = sketch("paste", vec![square(0.1, 0.1, 0.9, 0.9)]);
        let mask = sketch("mask", vec![square(0.0, 0.0, 1.0, 1.0)]);

        assert!(paste_mask_alignment("paste", &paste, "mask", &mask, 1.0e-9).is_empty());
    }

    #[test]
    fn solder_mask_sliver_reports_thin_mask_webs() {
        let mask = sketch("mask", vec![square(0.0, 0.0, 0.05, 2.0)]);

        let violations = solder_mask_sliver("mask", &mask, 0.1, 1.0e-9);

        assert_eq!(violations.len(), 1);
    }

    #[test]
    fn minimum_mask_opening_reports_too_small_openings() {
        let mask = sketch("mask", vec![square(0.0, 0.0, 0.05, 0.2)]);

        let violations = minimum_mask_opening("mask", &mask, 0.1, 1.0e-9);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "minimum-mask-opening");
    }

    #[test]
    fn minimum_mask_opening_allows_large_openings() {
        let mask = sketch("mask", vec![square(0.0, 0.0, 0.2, 0.2)]);

        assert!(minimum_mask_opening("mask", &mask, 0.1, 1.0e-9).is_empty());
    }

    #[test]
    fn solder_mask_opening_spacing_reports_narrow_bridge() {
        let mask = sketch(
            "mask",
            vec![square(0.0, 0.0, 1.0, 1.0), square(1.05, 0.0, 2.05, 1.0)],
        );

        let violations = solder_mask_opening_spacing("mask", &mask, 0.1, 1.0e-9);

        assert_eq!(violations.len(), 1);
        assert!(
            violations
                .iter()
                .all(|violation| violation.check == "solder-mask-opening-spacing")
        );
    }

    #[test]
    fn solder_mask_opening_spacing_allows_compliant_bridge() {
        let mask = sketch(
            "mask",
            vec![square(0.0, 0.0, 1.0, 1.0), square(1.2, 0.0, 2.2, 1.0)],
        );

        assert!(solder_mask_opening_spacing("mask", &mask, 0.1, 1.0e-9).is_empty());
    }

    #[test]
    fn minimum_line_width_flags_trace_below_three_mil_threshold() {
        let three_mil_mm = 0.0762;
        let narrow_trace = sketch(
            "top",
            vec![line_polygon([0.0, 0.0], [1.0, 0.0], three_mil_mm * 0.8).unwrap()],
        );

        assert_eq!(
            min_copper_neck_width("top", &narrow_trace, three_mil_mm, 1.0e-9).len(),
            1
        );
    }

    #[test]
    fn minimum_line_width_allows_six_mil_preferred_trace() {
        let three_mil_mm = 0.0762;
        let six_mil_mm = 0.1524;
        let preferred_trace = sketch(
            "top",
            vec![line_polygon([0.0, 0.0], [2.0, 0.0], six_mil_mm).unwrap()],
        );

        let violations = min_copper_neck_width("top", &preferred_trace, three_mil_mm, 1.0e-9);

        assert!(
            violations.is_empty(),
            "unexpected six mil trace violation area: {:?}",
            violations
                .iter()
                .map(|violation| violation.total_area)
                .collect::<Vec<_>>()
        );
    }

    const COMPLEX_PROJECT_FIXTURES: &[(&str, &str)] = &[
        (
            "docs/CPArti FPGA dev board.zip",
            "CPArti FPGA dev board.kicad_pcb",
        ),
        ("docs/HVP109A.zip", "HVP109A.kicad_pcb"),
    ];

    #[test]
    fn min_copper_neck_width_completes_on_complex_project_copper_layers() {
        let started = Instant::now();
        let min_width = 0.0762;
        let min_area = 1.0e-9;

        for (zip_path, board_entry) in COMPLEX_PROJECT_FIXTURES {
            let Some(board_bytes) = unzip_fixture_entry(zip_path, board_entry) else {
                continue;
            };
            let board_path = write_temp_fixture(board_entry, &board_bytes);
            let board =
                load_kicad_pcb(&board_path).expect("complex project KiCad fixture should parse");
            for (layer_name, copper) in board.copper_layers(&[]) {
                if layer_name != "F.Cu" && layer_name != "B.Cu" {
                    continue;
                }
                let _ = min_copper_neck_width(&layer_name, &copper, min_width, min_area);
            }
            let _ = std::fs::remove_file(board_path);
        }

        assert!(
            started.elapsed() < Duration::from_secs(20),
            "complex project copper neck regression fixture took {:?}",
            started.elapsed()
        );
    }

    #[test]
    fn board_edge_clearance_covers_trace_below_point_two_mm() {
        let board = sketch("edge", vec![square(0.0, 0.0, 10.0, 10.0)]);
        let too_close_trace = sketch(
            "top",
            vec![line_polygon([0.10, 1.0], [0.10, 9.0], 0.05).unwrap()],
        );
        let compliant_trace = sketch(
            "top",
            vec![line_polygon([0.35, 1.0], [0.35, 9.0], 0.05).unwrap()],
        );

        assert_eq!(
            board_edge_clearance("top", &too_close_trace, "edge", &board, 0.20, 1.0e-9).len(),
            1
        );
        assert!(
            board_edge_clearance("top", &compliant_trace, "edge", &board, 0.20, 1.0e-9).is_empty()
        );
    }

    #[test]
    fn board_edge_clearance_reports_pad_crossing_outline() {
        let board = sketch("edge", vec![square(0.0, 0.0, 10.0, 10.0)]);
        let pad = sketch("top", vec![square(9.8, 4.0, 10.2, 4.4)]);

        let violations = board_edge_clearance("top", &pad, "edge", &board, 0.0, 1.0e-9);

        assert_eq!(violations.len(), 1);
    }

    #[test]
    fn board_outline_sanity_reports_empty_outline_layers() {
        let outline = empty_sketch(Some(LayerMetadata {
            name: "edge".to_string(),
        }));

        let violations = board_outline_sanity("edge", &outline, 1.0e-9);

        assert_eq!(violations.len(), 1);
    }

    #[test]
    fn board_outline_sanity_accepts_closed_outline_area() {
        let outline = sketch("edge", vec![square(0.0, 0.0, 10.0, 10.0)]);

        assert!(board_outline_sanity("edge", &outline, 1.0e-9).is_empty());
    }

    #[test]
    fn board_outline_fragments_reports_multiple_disconnected_regions() {
        let outline = sketch(
            "edge",
            vec![square(0.0, 0.0, 1.0, 1.0), square(2.0, 0.0, 3.0, 1.0)],
        );

        let violations = board_outline_fragments("edge", &outline, 1.0e-9);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "board-outline-fragments");
    }

    #[test]
    fn board_outline_fragments_allows_single_region() {
        let outline = sketch("edge", vec![square(0.0, 0.0, 1.0, 1.0)]);

        assert!(board_outline_fragments("edge", &outline, 1.0e-9).is_empty());
    }

    #[test]
    fn board_outline_self_intersection_readiness_reports_bow_tie() {
        let outline = sketch(
            "edge",
            vec![Polygon::new(
                LineString(vec![
                    Coord { x: 0.0, y: 0.0 },
                    Coord { x: 4.0, y: 4.0 },
                    Coord { x: 0.0, y: 4.0 },
                    Coord { x: 4.0, y: 0.0 },
                    Coord { x: 0.0, y: 0.0 },
                ]),
                vec![],
            )],
        );

        let violations = board_outline_self_intersection_readiness("edge", &outline);

        assert_eq!(violations.len(), 1);
        assert_eq!(
            violations[0].check,
            "board-outline-self-intersection-readiness"
        );
    }

    #[test]
    fn board_outline_self_intersection_readiness_allows_rectangle() {
        let outline = sketch("edge", vec![square(0.0, 0.0, 10.0, 10.0)]);

        assert!(board_outline_self_intersection_readiness("edge", &outline).is_empty());
    }

    #[test]
    fn board_outline_notch_readiness_reports_sharp_notch() {
        let outline = sketch(
            "edge",
            vec![Polygon::new(
                LineString(vec![
                    Coord { x: 0.0, y: 0.0 },
                    Coord { x: 10.0, y: 0.0 },
                    Coord { x: 10.0, y: 10.0 },
                    Coord { x: 6.0, y: 10.0 },
                    Coord { x: 6.0, y: 9.9 },
                    Coord { x: 5.0, y: 9.5 },
                    Coord { x: 4.0, y: 9.9 },
                    Coord { x: 4.0, y: 10.0 },
                    Coord { x: 0.0, y: 10.0 },
                    Coord { x: 0.0, y: 0.0 },
                ]),
                vec![],
            )],
        );

        let violations = board_outline_notch_readiness("edge", &outline);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "board-outline-notch-readiness");
        assert!(!violations[0].locations.is_empty());
    }

    #[test]
    fn board_outline_notch_readiness_allows_convex_geometry() {
        let outline = sketch("edge", vec![square(0.0, 0.0, 10.0, 10.0)]);

        assert!(board_outline_notch_readiness("edge", &outline).is_empty());
    }

    #[test]
    fn board_outline_notch_readiness_is_orientation_agnostic() {
        let ccw = vec![
            Coord { x: 0.0, y: 0.0 },
            Coord { x: 10.0, y: 0.0 },
            Coord { x: 10.0, y: 10.0 },
            Coord { x: 6.0, y: 10.0 },
            Coord { x: 6.0, y: 9.9 },
            Coord { x: 5.0, y: 9.5 },
            Coord { x: 4.0, y: 9.9 },
            Coord { x: 4.0, y: 10.0 },
            Coord { x: 0.0, y: 10.0 },
            Coord { x: 0.0, y: 0.0 },
        ];
        let clockwise = {
            let mut reversed = ccw.clone();
            reversed.reverse();
            Polygon::new(LineString(reversed), vec![])
        };

        let outline = sketch("edge", vec![clockwise]);

        let violations = board_outline_notch_readiness("edge", &outline);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "board-outline-notch-readiness");
    }

    #[test]
    fn board_outline_notch_readiness_detects_notch_in_hole() {
        let mut outer = square(0.0, 0.0, 10.0, 10.0).exterior().0.clone();
        outer.pop();
        let outline = sketch(
            "edge",
            vec![Polygon::new(
                LineString(outer),
                vec![LineString(vec![
                    Coord { x: 2.0, y: 2.0 },
                    Coord { x: 8.0, y: 2.0 },
                    Coord { x: 8.0, y: 8.0 },
                    Coord { x: 6.0, y: 8.0 },
                    Coord { x: 6.0, y: 7.9 },
                    Coord { x: 5.0, y: 7.5 },
                    Coord { x: 4.0, y: 7.9 },
                    Coord { x: 4.0, y: 8.0 },
                    Coord { x: 2.0, y: 8.0 },
                    Coord { x: 2.0, y: 2.0 },
                ])],
            )],
        );

        let violations = board_outline_notch_readiness("edge", &outline);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "board-outline-notch-readiness");
        assert!(!violations[0].locations.is_empty());
    }

    #[test]
    fn board_outline_duplicate_readiness_reports_identical_contours() {
        let outline = sketch(
            "edge",
            vec![square(0.0, 0.0, 10.0, 10.0), square(0.0, 0.0, 10.0, 10.0)],
        );

        let violations = board_outline_duplicate_readiness("edge", &outline);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "board-outline-duplicate-readiness");
        assert!(!violations[0].locations.is_empty());
    }

    #[test]
    fn board_outline_duplicate_readiness_allows_discrete_outer_regions() {
        let outline = sketch(
            "edge",
            vec![square(0.0, 0.0, 10.0, 10.0), square(20.0, 0.0, 30.0, 10.0)],
        );

        assert!(board_outline_duplicate_readiness("edge", &outline).is_empty());
    }

    #[test]
    fn board_outline_nesting_readiness_reports_nested_contour() {
        let outline = sketch(
            "edge",
            vec![square(0.0, 0.0, 10.0, 10.0), square(2.0, 2.0, 4.0, 4.0)],
        );

        let violations = board_outline_nesting_readiness("edge", &outline);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "board-outline-nesting-readiness");
        assert!(!violations[0].locations.is_empty());
    }

    #[test]
    fn board_outline_nesting_readiness_allows_non_nested_discrete_regions() {
        let outline = sketch(
            "edge",
            vec![square(0.0, 0.0, 10.0, 10.0), square(20.0, 0.0, 30.0, 10.0)],
        );

        assert!(board_outline_nesting_readiness("edge", &outline).is_empty());
    }

    #[test]
    fn board_outline_nesting_readiness_allows_touching_contours() {
        let outline = sketch(
            "edge",
            vec![square(0.0, 0.0, 10.0, 10.0), square(10.0, 4.0, 12.0, 6.0)],
        );

        assert!(board_outline_nesting_readiness("edge", &outline).is_empty());
    }

    #[test]
    fn board_outline_duplicate_readiness_reports_reversed_duplicate_contour() {
        let mut outer = square(0.0, 0.0, 10.0, 10.0).exterior().0.clone();
        outer.pop();
        outer.reverse();
        outer.push(outer[0]);
        let duplicate = Polygon::new(LineString(outer), vec![]);

        let outline = sketch("edge", vec![square(0.0, 0.0, 10.0, 10.0), duplicate]);

        let violations = board_outline_duplicate_readiness("edge", &outline);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "board-outline-duplicate-readiness");
    }

    #[test]
    fn board_outline_cutout_clearance_reports_nested_inner_region_intrusion() {
        let outline = sketch(
            "edge",
            vec![square(0.0, 0.0, 10.0, 10.0), square(3.0, 3.0, 7.0, 7.0)],
        );
        let subject = sketch("top", vec![square(4.0, 4.0, 6.0, 6.0)]);

        let violations =
            board_outline_cutout_clearance("top", &subject, "edge", &outline, 0.0, 1.0e-9);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "board-outline-cutout-clearance");
    }

    #[test]
    fn board_outline_cutout_clearance_reports_nearby_geometry_with_clearance() {
        let outline = sketch(
            "edge",
            vec![square(0.0, 0.0, 10.0, 10.0), square(3.0, 3.0, 7.0, 7.0)],
        );
        let near = sketch("top", vec![square(7.15, 4.0, 7.45, 6.0)]);

        let violations =
            board_outline_cutout_clearance("top", &near, "edge", &outline, 0.25, 1.0e-9);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "board-outline-cutout-clearance");
    }

    #[test]
    fn board_outline_cutout_clearance_allows_geometry_outside_clearance_band() {
        let outline = sketch(
            "edge",
            vec![square(0.0, 0.0, 10.0, 10.0), square(3.0, 3.0, 7.0, 7.0)],
        );
        let far = sketch("top", vec![square(7.8, 4.0, 8.2, 6.0)]);

        let violations =
            board_outline_cutout_clearance("top", &far, "edge", &outline, 0.25, 1.0e-9);

        assert!(violations.is_empty());
    }

    #[test]
    fn board_outline_cutout_clearance_allows_geometry_outside_cutout_region() {
        let outline = sketch(
            "edge",
            vec![square(0.0, 0.0, 10.0, 10.0), square(3.0, 3.0, 7.0, 7.0)],
        );
        let subject = sketch("top", vec![square(1.0, 1.0, 2.0, 2.0)]);

        let violations =
            board_outline_cutout_clearance("top", &subject, "edge", &outline, 0.0, 1.0e-9);

        assert!(violations.is_empty());
    }

    #[test]
    fn board_outline_cutout_clearance_allows_non_nested_outline_regions() {
        let outline = sketch(
            "edge",
            vec![square(0.0, 0.0, 10.0, 10.0), square(12.0, 0.0, 15.0, 2.0)],
        );
        let subject = sketch("top", vec![square(1.0, 1.0, 2.0, 2.0)]);

        assert!(
            board_outline_cutout_clearance("top", &subject, "edge", &outline, 0.0, 1.0e-9)
                .is_empty()
        );
    }

    #[test]
    fn board_outline_cutout_clearance_reports_multiple_nested_regions() {
        let outline = sketch(
            "edge",
            vec![
                square(0.0, 0.0, 20.0, 20.0),
                square(3.0, 3.0, 5.0, 5.0),
                square(12.0, 12.0, 14.0, 14.0),
            ],
        );
        let subject = sketch(
            "top",
            vec![square(4.0, 4.0, 4.5, 4.5), square(13.0, 13.0, 13.5, 13.5)],
        );

        let violations =
            board_outline_cutout_clearance("top", &subject, "edge", &outline, 0.0, 1.0e-9);

        assert_eq!(violations.len(), 2);
    }

    #[test]
    fn board_outline_cutout_clearance_flags_zero_clearance_touching_geometry() {
        let outline = sketch(
            "edge",
            vec![square(0.0, 0.0, 10.0, 10.0), square(3.0, 3.0, 7.0, 7.0)],
        );
        let touching = sketch("top", vec![square(2.0, 4.0, 3.0, 6.0)]);

        let violations =
            board_outline_cutout_clearance("top", &touching, "edge", &outline, 0.0, 1.0e-9);

        assert_eq!(violations.len(), 1);
    }

    #[test]
    fn board_outline_cutout_clearance_is_orientation_tolerant_for_cutouts() {
        let mut inner = square(3.0, 3.0, 7.0, 7.0).exterior().0.clone();
        inner.pop();
        inner.reverse();
        inner.push(inner[0]);
        let outline = sketch(
            "edge",
            vec![
                square(0.0, 0.0, 10.0, 10.0),
                Polygon::new(LineString(inner), vec![]),
            ],
        );
        let near = sketch("top", vec![square(7.15, 4.0, 7.45, 6.0)]);

        let violations =
            board_outline_cutout_clearance("top", &near, "edge", &outline, 0.25, 1.0e-9);

        assert_eq!(violations.len(), 1);
    }

    #[test]
    fn exposed_copper_reports_oversized_mask_opening_touching_neighbor() {
        let copper = sketch(
            "top",
            vec![square(0.0, 0.0, 1.0, 1.0), square(1.2, 0.0, 2.2, 1.0)],
        );
        let mask_opening = sketch("mask", vec![square(-0.1, -0.1, 1.35, 1.1)]);

        let violations = exposed_copper("top", &copper, "mask", &mask_opening, 1.0e-9);

        assert_eq!(violations.len(), 1);
    }

    #[test]
    fn solder_mask_opening_coverage_reports_undersized_or_missing_openings() {
        let copper = sketch(
            "top",
            vec![square(0.0, 0.0, 1.0, 1.0), square(2.0, 0.0, 3.0, 1.0)],
        );
        let mask_openings = sketch("mask", vec![square(0.1, 0.1, 0.9, 0.9)]);

        let violations =
            solder_mask_opening_coverage("top", &copper, "mask", &mask_openings, 1.0e-9);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "solder-mask-opening-coverage");
    }

    #[test]
    fn solder_mask_opening_coverage_allows_full_openings() {
        let copper = sketch("top", vec![square(0.0, 0.0, 1.0, 1.0)]);
        let mask_openings = sketch("mask", vec![square(-0.1, -0.1, 1.1, 1.1)]);

        let violations =
            solder_mask_opening_coverage("top", &copper, "mask", &mask_openings, 1.0e-9);

        assert!(violations.is_empty());
    }

    #[test]
    fn solder_mask_expansion_reports_oversized_opening() {
        let copper = sketch("top", vec![square(0.0, 0.0, 1.0, 1.0)]);
        let mask_openings = sketch("mask", vec![square(-0.2, -0.2, 1.2, 1.2)]);

        let violations = solder_mask_expansion("top", &copper, "mask", &mask_openings, 0.1, 1.0e-9);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "solder-mask-expansion");
    }

    #[test]
    fn solder_mask_expansion_allows_configured_opening_growth() {
        let copper = sketch("top", vec![square(0.0, 0.0, 1.0, 1.0)]);
        let mask_openings = sketch("mask", vec![square(-0.05, -0.05, 1.05, 1.05)]);

        assert!(
            solder_mask_expansion("top", &copper, "mask", &mask_openings, 0.1, 1.0e-9).is_empty()
        );
    }

    #[test]
    fn solder_mask_overlap_clearance_reports_adjacent_covered_copper() {
        let copper = sketch("top", vec![square(1.05, 0.0, 1.20, 1.0)]);
        let mask_openings = sketch("mask", vec![square(0.0, 0.0, 1.0, 1.0)]);

        let violations =
            solder_mask_overlap_clearance("top", &copper, "mask", &mask_openings, 0.1, 1.0e-9);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "solder-mask-overlap-clearance");
    }

    #[test]
    fn solder_mask_overlap_clearance_ignores_intentionally_open_copper() {
        let copper = sketch("top", vec![square(0.1, 0.1, 0.9, 0.9)]);
        let mask_openings = sketch("mask", vec![square(0.0, 0.0, 1.0, 1.0)]);

        assert!(
            solder_mask_overlap_clearance("top", &copper, "mask", &mask_openings, 0.1, 1.0e-9)
                .is_empty()
        );
    }

    #[test]
    fn solder_mask_overlap_clearance_allows_distant_covered_copper() {
        let copper = sketch("top", vec![square(1.2, 0.0, 1.4, 1.0)]);
        let mask_openings = sketch("mask", vec![square(0.0, 0.0, 1.0, 1.0)]);

        assert!(
            solder_mask_overlap_clearance("top", &copper, "mask", &mask_openings, 0.1, 1.0e-9)
                .is_empty()
        );
    }

    #[test]
    fn silkscreen_overlap_reports_legend_over_pad_or_slot() {
        let pad_opening = sketch("mask", vec![square(0.0, 0.0, 1.0, 1.0)]);
        let silk_text_stroke = sketch(
            "silk",
            vec![line_polygon([-0.2, 0.5], [1.2, 0.5], 0.08).unwrap()],
        );

        let violations =
            silkscreen_overlap("silk", &silk_text_stroke, "mask", &pad_opening, 1.0e-9);

        assert_eq!(violations.len(), 1);
    }

    #[test]
    fn silkscreen_overlap_reports_legend_over_v_score_or_slot_geometry() {
        let panel_feature = sketch(
            "V-Score",
            vec![line_polygon([0.5, -1.0], [0.5, 1.0], 0.12).unwrap()],
        );
        let silk_text_stroke = sketch(
            "B.SilkS",
            vec![line_polygon([0.0, 0.0], [1.0, 0.0], 0.08).unwrap()],
        );

        let violations = silkscreen_overlap(
            "B.SilkS",
            &silk_text_stroke,
            "V-Score",
            &panel_feature,
            1.0e-9,
        );

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "silkscreen-overlap");
    }

    #[test]
    fn silkscreen_clearance_reports_legend_near_blocker() {
        let pad_opening = sketch("mask", vec![square(0.0, 0.0, 1.0, 1.0)]);
        let silk_text_stroke = sketch(
            "silk",
            vec![line_polygon([1.08, 0.5], [1.8, 0.5], 0.05).unwrap()],
        );

        let violations =
            silkscreen_clearance("silk", &silk_text_stroke, "mask", &pad_opening, 0.1, 1.0e-9);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "silkscreen-clearance");
    }

    #[test]
    fn silkscreen_clearance_allows_distant_legend() {
        let pad_opening = sketch("mask", vec![square(0.0, 0.0, 1.0, 1.0)]);
        let silk_text_stroke = sketch(
            "silk",
            vec![line_polygon([1.3, 0.5], [1.8, 0.5], 0.05).unwrap()],
        );

        assert!(
            silkscreen_clearance("silk", &silk_text_stroke, "mask", &pad_opening, 0.1, 1.0e-9)
                .is_empty()
        );
    }

    #[test]
    fn silkscreen_min_width_reports_thin_legend_strokes() {
        let silk = sketch(
            "silk",
            vec![line_polygon([0.0, 0.0], [2.0, 0.0], 0.08).unwrap()],
        );

        let violations = silkscreen_min_width("silk", &silk, 0.12, 1.0e-9);

        assert_eq!(violations.len(), 1);
    }

    #[test]
    fn silkscreen_min_width_allows_wide_legend_strokes() {
        let silk = sketch("silk", vec![square(0.0, 0.0, 1.0, 1.0)]);

        let violations = silkscreen_min_width("silk", &silk, 0.12, 1.0e-9);

        assert!(violations.is_empty());
    }

    #[test]
    fn layer_sanity_reports_empty_or_unbounded_layers() {
        let empty = empty_sketch(Some(LayerMetadata {
            name: "empty mask".to_string(),
        }));

        let violations = layer_sanity("empty mask", &empty, None);

        assert_eq!(violations.len(), 2);
        assert!(
            violations.iter().any(|violation| violation
                .message
                .as_deref()
                .unwrap()
                .contains("empty"))
        );
        assert!(
            violations.iter().any(|violation| violation
                .message
                .as_deref()
                .unwrap()
                .contains("bounding"))
        );
    }

    #[test]
    fn layer_sanity_reports_malformed_contours() {
        let bad_outline = sketch(
            "bad layer",
            vec![Polygon::new(
                LineString(vec![
                    Coord { x: 0.0, y: 0.0 },
                    Coord { x: 4.0, y: 0.0 },
                    Coord { x: 0.0, y: 4.0 },
                    Coord { x: 4.0, y: 4.0 },
                    Coord { x: 0.0, y: 0.0 },
                ]),
                vec![],
            )],
        );

        let violations = layer_sanity("bad layer", &bad_outline, None);

        assert!(violations.iter().any(|violation| {
            violation
                .message
                .as_deref()
                .is_some_and(|message| message.contains("self-intersecting"))
        }));
    }

    #[test]
    fn layer_sanity_reports_self_intersection_inside_hole() {
        let bad_outline = sketch(
            "bad layer",
            vec![Polygon::new(
                LineString(vec![
                    Coord { x: 0.0, y: 0.0 },
                    Coord { x: 10.0, y: 0.0 },
                    Coord { x: 10.0, y: 10.0 },
                    Coord { x: 0.0, y: 10.0 },
                    Coord { x: 0.0, y: 0.0 },
                ]),
                vec![LineString(vec![
                    Coord { x: 2.0, y: 2.0 },
                    Coord { x: 6.0, y: 6.0 },
                    Coord { x: 2.0, y: 6.0 },
                    Coord { x: 6.0, y: 2.0 },
                    Coord { x: 2.0, y: 2.0 },
                ])],
            )],
        );

        let violations = layer_sanity("bad layer", &bad_outline, None);

        assert!(violations.iter().any(|violation| {
            violation
                .message
                .as_deref()
                .is_some_and(|message| message.contains("self-intersecting"))
        }));
        assert_eq!(violations.len(), 1);
    }

    #[test]
    fn layer_sanity_reports_non_finite_coordinates_in_hole() {
        let invalid = sketch(
            "invalid layer",
            vec![Polygon::new(
                LineString(vec![
                    Coord { x: 0.0, y: 0.0 },
                    Coord { x: 10.0, y: 0.0 },
                    Coord { x: 10.0, y: 10.0 },
                    Coord { x: 0.0, y: 10.0 },
                    Coord { x: 0.0, y: 0.0 },
                ]),
                vec![LineString(vec![
                    Coord { x: 2.0, y: 2.0 },
                    Coord {
                        x: f64::NAN,
                        y: 2.0,
                    },
                    Coord { x: 6.0, y: 2.0 },
                    Coord { x: 6.0, y: 6.0 },
                    Coord { x: 2.0, y: 2.0 },
                ])],
            )],
        );

        let violations = layer_sanity("invalid layer", &invalid, None);

        assert!(violations.iter().any(|violation| {
            violation
                .message
                .as_deref()
                .is_some_and(|message| message.contains("non-finite"))
        }));
    }

    #[test]
    fn layer_sanity_reports_non_finite_coordinates() {
        let invalid = sketch(
            "invalid layer",
            vec![Polygon::new(
                LineString(vec![
                    Coord {
                        x: f64::NAN,
                        y: 0.0,
                    },
                    Coord { x: 1.0, y: 0.0 },
                    Coord { x: 1.0, y: 1.0 },
                    Coord { x: 0.0, y: 1.0 },
                    Coord {
                        x: f64::NAN,
                        y: 0.0,
                    },
                ]),
                vec![],
            )],
        );

        let violations = layer_sanity("invalid layer", &invalid, None);

        assert!(violations.iter().any(|violation| {
            violation
                .message
                .as_deref()
                .is_some_and(|message| message.contains("non-finite"))
        }));
    }

    #[test]
    fn layer_sanity_reports_area_excursions() {
        let flood = sketch("inner", vec![square(0.0, 0.0, 20.0, 20.0)]);

        let violations = layer_sanity("inner", &flood, Some(100.0));

        assert_eq!(violations.len(), 1);
        assert!(
            violations[0]
                .message
                .as_deref()
                .unwrap()
                .contains("exceeds maximum")
        );
    }

    #[test]
    fn layer_sanity_allows_area_equal_to_limit() {
        let flood = sketch("inner", vec![square(0.0, 0.0, 10.0, 10.0)]);

        let violations = layer_sanity("inner", &flood, Some(100.0));

        assert!(violations.iter().all(|violation| {
            violation
                .message
                .as_deref()
                .is_none_or(|message| !message.contains("exceeds maximum"))
        }));
    }

    #[test]
    fn duplicate_layer_geometry_readiness_reports_identical_layers() {
        let top = sketch("F.Cu", vec![square(0.0, 0.0, 10.0, 10.0)]);
        let duplicate = sketch("B.Cu", vec![square(0.0, 0.0, 10.0, 10.0)]);
        let layers = vec![("F.Cu".to_string(), top), ("B.Cu".to_string(), duplicate)];

        let violations = duplicate_layer_geometry_readiness(&layers, 1.0e-9);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "layer-sanity");
        assert_eq!(violations[0].layers, vec!["F.Cu", "B.Cu"]);
        assert!(
            violations[0]
                .message
                .as_deref()
                .unwrap_or_default()
                .contains("duplicate geometry")
        );
        assert!(!violations[0].locations.is_empty());
    }

    #[test]
    fn duplicate_layer_geometry_readiness_allows_different_or_empty_layers() {
        let top = sketch("F.Cu", vec![square(0.0, 0.0, 10.0, 10.0)]);
        let shifted = sketch("B.Cu", vec![square(12.0, 0.0, 22.0, 10.0)]);
        let empty = empty_sketch(Some(LayerMetadata {
            name: "empty".to_string(),
        }));
        let layers = vec![
            ("F.Cu".to_string(), top),
            ("B.Cu".to_string(), shifted),
            ("empty".to_string(), empty),
        ];

        assert!(duplicate_layer_geometry_readiness(&layers, 1.0e-9).is_empty());
    }

    #[test]
    fn tiny_layer_feature_readiness_reports_islands_below_area_gate() {
        let layer = sketch(
            "F.Cu",
            vec![
                square(0.0, 0.0, 10.0, 10.0),
                square(20.0, 20.0, 20.05, 20.05),
            ],
        );

        let violations = tiny_layer_feature_readiness("F.Cu", &layer, 0.01);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "layer-sanity");
        assert!(!violations[0].locations.is_empty());
        assert!(
            violations[0]
                .message
                .as_deref()
                .unwrap_or_default()
                .contains("tiny aperture flashes")
        );
    }

    #[test]
    fn tiny_layer_feature_readiness_allows_larger_or_unconfigured_features() {
        let layer = sketch("F.Cu", vec![square(0.0, 0.0, 10.0, 10.0)]);

        assert!(tiny_layer_feature_readiness("F.Cu", &layer, 0.01).is_empty());
        assert!(tiny_layer_feature_readiness("F.Cu", &layer, 0.0).is_empty());
        assert!(tiny_layer_feature_readiness("F.Cu", &layer, f64::NAN).is_empty());
    }

    #[test]
    fn skinny_layer_feature_readiness_reports_long_slivers_above_area_gate() {
        let layer = sketch(
            "F.Cu",
            vec![
                square(0.0, 0.0, 10.0, 10.0),
                rect_polygon([20.0, 20.0], [4.0, 0.05], 0.0),
            ],
        );

        let violations = skinny_layer_feature_readiness("F.Cu", &layer, 0.10, 0.01);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "layer-sanity");
        assert!(!violations[0].locations.is_empty());
        assert!(
            violations[0]
                .message
                .as_deref()
                .unwrap_or_default()
                .contains("hairline fragments")
        );
    }

    #[test]
    fn skinny_layer_feature_readiness_allows_wide_tiny_or_unconfigured_features() {
        let wide = sketch("F.Cu", vec![square(0.0, 0.0, 10.0, 10.0)]);
        let tiny = sketch("F.Cu", vec![rect_polygon([0.0, 0.0], [0.09, 0.09], 0.0)]);

        assert!(skinny_layer_feature_readiness("F.Cu", &wide, 0.10, 0.01).is_empty());
        assert!(skinny_layer_feature_readiness("F.Cu", &tiny, 0.10, 0.01).is_empty());
        assert!(skinny_layer_feature_readiness("F.Cu", &wide, 0.0, 0.01).is_empty());
        assert!(skinny_layer_feature_readiness("F.Cu", &wide, f64::NAN, 0.01).is_empty());
    }

    #[test]
    fn duplicate_layer_island_readiness_reports_repeated_polygon_geometry() {
        let duplicate = square(0.0, 0.0, 10.0, 10.0);
        let layer = sketch("F.Cu", vec![duplicate.clone(), duplicate]);

        let violations = duplicate_layer_island_readiness("F.Cu", &layer, 1.0e-9);

        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "layer-sanity");
        assert!(!violations[0].locations.is_empty());
        assert!(
            violations[0]
                .message
                .as_deref()
                .unwrap_or_default()
                .contains("duplicate polygon island")
        );
    }

    #[test]
    fn duplicate_layer_island_readiness_allows_discrete_or_tiny_polygons() {
        let discrete = sketch(
            "F.Cu",
            vec![square(0.0, 0.0, 10.0, 10.0), square(12.0, 0.0, 22.0, 10.0)],
        );
        let tiny_duplicate = rect_polygon([0.0, 0.0], [0.03, 0.03], 0.0);
        let tiny = sketch("F.Cu", vec![tiny_duplicate.clone(), tiny_duplicate]);

        assert!(duplicate_layer_island_readiness("F.Cu", &discrete, 1.0e-9).is_empty());
        assert!(duplicate_layer_island_readiness("F.Cu", &tiny, 0.01).is_empty());
    }

    #[test]
    fn board_outline_self_intersection_readiness_reports_hole_self_intersection() {
        let outline = sketch(
            "edge",
            vec![Polygon::new(
                LineString(vec![
                    Coord { x: 0.0, y: 0.0 },
                    Coord { x: 10.0, y: 0.0 },
                    Coord { x: 10.0, y: 10.0 },
                    Coord { x: 0.0, y: 10.0 },
                    Coord { x: 0.0, y: 0.0 },
                ]),
                vec![LineString(vec![
                    Coord { x: 2.0, y: 2.0 },
                    Coord { x: 6.0, y: 6.0 },
                    Coord { x: 2.0, y: 6.0 },
                    Coord { x: 6.0, y: 2.0 },
                    Coord { x: 2.0, y: 2.0 },
                ])],
            )],
        );

        let violations = board_outline_self_intersection_readiness("edge", &outline);

        assert_eq!(violations.len(), 1);
        assert_eq!(
            violations[0].check,
            "board-outline-self-intersection-readiness"
        );
    }

    #[test]
    fn board_outline_duplicate_and_nesting_helpers_operate_on_shared_edge_case() {
        let outer = square(0.0, 0.0, 10.0, 10.0);
        let touching = Polygon::new(
            LineString(vec![
                Coord { x: 10.0, y: 4.0 },
                Coord { x: 12.0, y: 4.0 },
                Coord { x: 12.0, y: 6.0 },
                Coord { x: 10.0, y: 6.0 },
                Coord { x: 10.0, y: 4.0 },
            ]),
            vec![],
        );

        assert!(!super::polygon_contains_other_outer(
            &outer,
            &touching,
            super::BOARD_OUTLINE_NESTED_OVERLAP_RATIO,
            super::BOARD_OUTLINE_GEOMETRY_TOLERANCE,
        ));
        assert!(super::polygons_are_duplicate(
            &outer,
            &outer,
            super::BOARD_OUTLINE_GEOMETRY_TOLERANCE,
        ));
    }

    #[test]
    fn mechanical_layer_geometry_reports_shapes_on_user_or_mechanical_layers() {
        let user = sketch("Dwgs.User", vec![square(0.0, 0.0, 1.0, 1.0)]);
        let mechanical = sketch("board-Mechanical.gbr", vec![square(2.0, 0.0, 3.0, 1.0)]);

        assert_eq!(
            mechanical_layer_geometry("Dwgs.User", &user, 1.0e-9).len(),
            1
        );
        assert_eq!(
            mechanical_layer_geometry("board-Mechanical.gbr", &mechanical, 1.0e-9).len(),
            1
        );
    }

    #[test]
    fn mechanical_layer_geometry_ignores_normal_copper_layers() {
        let copper = sketch("F.Cu", vec![square(0.0, 0.0, 1.0, 1.0)]);

        assert!(mechanical_layer_geometry("F.Cu", &copper, 1.0e-9).is_empty());
    }

    #[test]
    fn acid_trap_reports_acute_polygon_vertices() {
        let copper = sketch(
            "top",
            vec![Polygon::new(
                LineString(vec![
                    Coord { x: 0.0, y: 0.0 },
                    Coord { x: 2.0, y: 0.0 },
                    Coord { x: 0.1, y: 0.2 },
                    Coord { x: 0.0, y: 2.0 },
                    Coord { x: 0.0, y: 0.0 },
                ]),
                vec![],
            )],
        );

        let violations = acid_trap_candidates("top", &copper, 30.0);

        assert_eq!(violations.len(), 1);
        assert!(!violations[0].locations.is_empty());
    }

    fn sketch(name: &str, polygons: Vec<Polygon<f64>>) -> crate::PcbSketch {
        polygons_to_sketch(
            polygons,
            Some(LayerMetadata {
                name: name.to_string(),
            }),
        )
    }

    fn unzip_fixture_entry(zip_name: &str, entry_name: &str) -> Option<Vec<u8>> {
        let zip_path = Path::new(env!("CARGO_MANIFEST_DIR")).join(zip_name);
        if !zip_path.exists() {
            eprintln!(
                "skipping complex project fixture regression; missing {}",
                zip_path.display()
            );
            return None;
        }

        let output = Command::new("unzip")
            .arg("-p")
            .arg(&zip_path)
            .arg(entry_name)
            .output()
            .ok()?;
        if !output.status.success() {
            eprintln!(
                "skipping complex project fixture regression; could not extract {entry_name} from {}",
                zip_path.display()
            );
            return None;
        }

        Some(output.stdout)
    }

    fn write_temp_fixture(entry_name: &str, bytes: &[u8]) -> std::path::PathBuf {
        let sanitized = entry_name.replace(['/', '\\'], "_");
        let path = std::env::temp_dir().join(format!(
            "hyperdrc-complex-project-fixture-{}-{sanitized}",
            std::process::id()
        ));
        std::fs::write(&path, bytes).expect("temporary complex project fixture should be writable");
        path
    }

    fn square(min_x: f64, min_y: f64, max_x: f64, max_y: f64) -> Polygon<f64> {
        Polygon::new(
            LineString(vec![
                Coord { x: min_x, y: min_y },
                Coord { x: max_x, y: min_y },
                Coord { x: max_x, y: max_y },
                Coord { x: min_x, y: max_y },
                Coord { x: min_x, y: min_y },
            ]),
            vec![],
        )
    }
}
